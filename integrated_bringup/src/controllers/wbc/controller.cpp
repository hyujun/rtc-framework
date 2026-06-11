#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "integrated_bringup/support/demo_shared_config.hpp"
#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_base/utils/clamp_commands.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/constraints/eom_constraint.hpp"
#include "rtc_tsid/constraints/friction_cone_constraint.hpp"
#include "rtc_tsid/constraints/joint_limit_constraint.hpp"
#include "rtc_tsid/constraints/torque_limit_constraint.hpp"
#include "rtc_tsid/tasks/contact_consistency_task.hpp"
#include "rtc_tsid/tasks/force_task.hpp"
#include "rtc_tsid/tasks/internal_force_task.hpp"
#include "rtc_tsid/tasks/object_se3_task.hpp"
#include "rtc_tsid/tasks/object_wrench_task.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"
#include "rtc_tsid/tasks/se3_task.hpp"

namespace integrated_bringup {

// ── Constructor ──────────────────────────────────────────────────────────────

DemoWbcController::DemoWbcController(std::string_view urdf_path) : urdf_path_(urdf_path) {
  // Model is built in LoadConfig() using system model config.
}

DemoWbcController::~DemoWbcController() {
  // Stop the MPC solve thread before any member auto-destruction. See header
  // comment for the use-after-free race this prevents.
  if (mpc_thread_) {
    mpc_thread_->StopAndJoin();
  }
}

// ── Model initialization ─────────────────────────────────────────────────────

void DemoWbcController::InitModels(const rtc_urdf_bridge::ModelConfig& config) {
  namespace rub = rtc_urdf_bridge;

  // Prefer the shared builder injected by RtControllerNode so the URDF is
  // parsed only once across every controller. Fall back to building our own
  // (e.g. when the shared build failed or when called via fallback path with
  // a custom config).
  if (auto shared = GetSharedModelBuilder()) {
    builder_ = std::move(shared);
  } else {
    builder_ = std::make_shared<rub::PinocchioModelBuilder>(config);
  }

  // Arm sub-model (6-DoF) for FK / task-space logging
  const auto primary = GetPrimaryDeviceName();
  std::string arm_model_name = "arm";
  for (const auto& sm : config.sub_models) {
    if (sm.name == primary) {
      arm_model_name = primary;
      break;
    }
  }
  arm_handle_ = std::make_unique<rub::RtModelHandle>(builder_->GetReducedModel(arm_model_name));

  // Control model. Prefer the passive-locked reduced tree `wbc` built by
  // PinocchioModelBuilder (Analyzer auto-classifies mimic/closed-chain/hint
  // as passive → buildReducedModel locks them), which yields nq == nv == 16
  // for UR5e + 10-DoF hand. TSID/MPC/state
  // buffers all operate in this reduced space, so mimic DoFs are handled
  // by the hardware driver / MuJoCo's built-in tendon. If the tree isn't
  // configured in YAML, fall back to the raw URDF-parsed full model (nq=26,
  // nv=21 with Pinocchio first-class mimic) — this preserves pre-reduction
  // behaviour for URDFs without <mimic> tags.
  try {
    full_model_ptr_ = builder_->GetTreeModel("wbc");
    RCLCPP_INFO(logger_, "[wbc] control model: reduced tree 'wbc' (nq=%d nv=%d)",
                full_model_ptr_->nq, full_model_ptr_->nv);
  } catch (const std::exception& e) {
    full_model_ptr_ = builder_->GetFullModel();
    RCLCPP_INFO(logger_,
                "[wbc] control model: URDF full model (nq=%d nv=%d) — tree 'wbc' "
                "missing (%s); MPC handler-mode + TSID will see mimic joints",
                full_model_ptr_->nq, full_model_ptr_->nv, e.what());
  }

  RCLCPP_INFO(logger_, "Models initialized: arm nv=%d, control nq=%d nv=%d", arm_handle_->nv(),
              full_model_ptr_->nq, full_model_ptr_->nv);
}

void DemoWbcController::BuildJointReorderMap() {
  if (!full_model_ptr_) {
    return;
  }
  const auto& model = *full_model_ptr_;

  const auto* arm_cfg = GetDeviceNameConfig(GetPrimaryDeviceName());
  const auto* hand_cfg = GetDeviceNameConfig(GetSecondaryDeviceName());
  if (!arm_cfg) {
    RCLCPP_WARN(logger_, "Primary device config not available, using identity mapping");
    for (int i = 0; i < full_dof_; ++i) {
      ext_to_pin_q_[static_cast<std::size_t>(i)] = i;
      ext_to_pin_v_[static_cast<std::size_t>(i)] = i;
    }
    joint_reorder_valid_ = true;
    return;
  }

  int ext_idx = 0;

  // Arm joints
  for (const auto& jname : arm_cfg->joint_state_names) {
    if (!model.existJointName(jname)) {
      RCLCPP_ERROR(logger_, "Joint '%s' not found in full model", jname.c_str());
      continue;
    }
    const auto jid = model.getJointId(jname);
    const auto eidx = static_cast<std::size_t>(ext_idx);
    ext_to_pin_q_[eidx] = model.idx_qs[jid];
    ext_to_pin_v_[eidx] = model.idx_vs[jid];
    ++ext_idx;
  }

  // Hand joints (optional — single-device controllers leave hand_dof_ == 0)
  if (hand_cfg) {
    for (const auto& jname : hand_cfg->joint_state_names) {
      if (!model.existJointName(jname)) {
        RCLCPP_ERROR(logger_, "Joint '%s' not found in full model", jname.c_str());
        continue;
      }
      const auto jid = model.getJointId(jname);
      const auto eidx = static_cast<std::size_t>(ext_idx);
      ext_to_pin_q_[eidx] = model.idx_qs[jid];
      ext_to_pin_v_[eidx] = model.idx_vs[jid];
      ++ext_idx;
    }
  }

  joint_reorder_valid_ = (ext_idx == full_dof_);
  if (!joint_reorder_valid_) {
    RCLCPP_ERROR(logger_, "Joint reorder incomplete: mapped %d/%d joints", ext_idx, full_dof_);
  }
}

// ── TSID task/constraint factories ───────────────────────────────────────────
//
// Dispatch by YAML `type` string. Tasks/constraints are created with
// unique_ptr and transferred to formulation via add_task/add_constraint.
// After construction each is Init()'d with its own sub-node.

void DemoWbcController::BuildTsidTasks(const YAML::Node& tsid_node) {
  if (!full_model_ptr_ || !tsid_node || !tsid_node["tasks"]) {
    return;
  }
  const auto& model = *full_model_ptr_;
  auto& formulation = tsid_controller_.Formulation();

  for (auto it = tsid_node["tasks"].begin(); it != tsid_node["tasks"].end(); ++it) {
    const auto key = it->first.as<std::string>();
    // yaml-cpp iterator::operator->() returns a proxy holding a prvalue Node;
    // binding `it->second` by reference leaves a dangling handle once the
    // proxy temporary dies. Copy by value — Node is a lightweight handle.
    const YAML::Node task_cfg = it->second;
    const auto type = task_cfg["type"].as<std::string>("");

    if (type == "posture") {
      auto task = std::make_unique<rtc::tsid::PostureTask>();
      task->Init(model, robot_info_, pinocchio_cache_, task_cfg);
      formulation.AddTask(std::move(task));
      // Optional arm/hand gain split. The scalar/vector kp/kd in task_cfg are
      // consumed by PostureTask::Init above; when `arm`/`hand` sub-maps are
      // present they override per-DoF in ApplyPostureGains (post-reorder).
      ParsePostureSplitGains(task_cfg);
    } else if (type == "se3") {
      auto task = std::make_unique<rtc::tsid::SE3Task>();
      task->Init(model, robot_info_, pinocchio_cache_, task_cfg);
      // SE3Task's identity comes from its inner `name:` field (default "se3"),
      // NOT the YAML map key. The phase presets, the per-tick reference push
      // (GetTask(key)), and se3_task_active_in_phase all look the task up by
      // the map key — if it diverges from the resolved name the task is
      // unreachable: never deactivated by a preset, never given a reference, so
      // it tracks identity and fights every phase. Surface the mismatch loudly.
      const std::string resolved_name{task->Name()};
      if (resolved_name != key) {
        RCLCPP_WARN(logger_,
                    "[wbc] se3 task '%s' resolves to name '%s' — they must match; add "
                    "`name: \"%s\"` to the task block or GetTask(\"%s\") returns null",
                    key.c_str(), resolved_name.c_str(), key.c_str(), key.c_str());
      }
      formulation.AddTask(std::move(task));
      // F-2: capture base_frame for OnDeviceConfigsSet consistency check.
      // Skip when key absent — universe fallback path stays unchecked here.
      // (Cache the YAML::Node into a local to avoid yaml-cpp's
      // unnamed-temporary lifetime trap that GCC flags as dangling.)
      const YAML::Node base_frame_node = task_cfg["base_frame"];
      if (base_frame_node) {
        base_frame_yaml_entries_.emplace_back("tsid.tasks." + key + ".base_frame",
                                              base_frame_node.as<std::string>());
      }
    } else if (type == "force") {
      auto task = std::make_unique<rtc::tsid::ForceTask>();
      task->Init(model, robot_info_, pinocchio_cache_, task_cfg);
      task->SetContactManager(&contact_mgr_config_);
      formulation.AddTask(std::move(task));
    } else if (type == "contact_consistency") {
      // Stage A-2: soft task — residual J_c·a + dotJ_c·v. Activated in
      // closure/hold per phase preset; deactivated in pre_grasp.
      auto task = std::make_unique<rtc::tsid::ContactConsistencyTask>();
      task->Init(model, robot_info_, pinocchio_cache_, task_cfg);
      task->SetContactManager(&contact_mgr_config_);
      formulation.AddTask(std::move(task));
    } else if (type == "object_wrench") {
      // Stage B-5: drives Σ G·λ → w_obj_des (object-origin wrench). Shares
      // the controller-owned ContactManager / GraspCache / ObjectFrame
      // instances so the per-tick cache populated in ComputeTSIDPosition
      // is reused across all three object-level tasks.
      auto task = std::make_unique<rtc::tsid::ObjectWrenchTask>();
      task->Init(model, robot_info_, pinocchio_cache_, task_cfg);
      task->SetContactManager(&contact_mgr_config_);
      task->SetContactManagerRuntime(&contact_mgr_);
      task->SetGraspCache(&grasp_cache_);
      task->SetObjectFrame(&object_frame_);
      formulation.AddTask(std::move(task));
    } else if (type == "internal_force") {
      // Stage B-5: drives the squeeze λ component inside Null(G) toward a
      // projected reference (P_N · λ_squeeze_des). Stage B-5 keeps the
      // reference at zero — a future dynamic squeeze planner will write
      // into squeeze_lambda_des_ on phase entry.
      auto task = std::make_unique<rtc::tsid::InternalForceTask>();
      task->Init(model, robot_info_, pinocchio_cache_, task_cfg);
      task->SetContactManager(&contact_mgr_config_);
      task->SetContactManagerRuntime(&contact_mgr_);
      task->SetGraspCache(&grasp_cache_);
      formulation.AddTask(std::move(task));
    } else if (type == "object_se3") {
      // Stage B-5: drives the object SE(3) pose toward
      // object_state_provider_'s placement via the kinematic chain implied
      // by the active contacts. IdentityObjectStateProvider keeps the pose
      // at object_frame_ (set in LoadConfig) until a concrete provider is
      // plumbed in — IsValid() stays true so ResidualDim() = 6 whenever
      // ≥ 1 contact is active.
      auto task = std::make_unique<rtc::tsid::ObjectSE3Task>();
      task->Init(model, robot_info_, pinocchio_cache_, task_cfg);
      task->SetContactManager(&contact_mgr_config_);
      task->SetContactManagerRuntime(&contact_mgr_);
      task->SetGraspCache(&grasp_cache_);
      task->SetObjectStateProvider(&object_state_provider_);
      formulation.AddTask(std::move(task));
    } else {
      RCLCPP_ERROR(logger_, "[wbc] unknown task type '%s' for entry '%s' — skipping", type.c_str(),
                   key.c_str());
    }
  }
}

void DemoWbcController::BuildTsidConstraints(const YAML::Node& tsid_node) {
  if (!full_model_ptr_ || !tsid_node || !tsid_node["constraints"]) {
    return;
  }
  const auto& model = *full_model_ptr_;
  auto& formulation = tsid_controller_.Formulation();

  for (auto it = tsid_node["constraints"].begin(); it != tsid_node["constraints"].end(); ++it) {
    const auto key = it->first.as<std::string>();
    // yaml-cpp iterator::operator->() returns a proxy holding a prvalue Node;
    // see BuildTsidTasks comment above for the lifetime trap.
    const YAML::Node c_cfg = it->second;
    const auto type = c_cfg["type"].as<std::string>("");

    if (type == "eom") {
      auto c = std::make_unique<rtc::tsid::EomConstraint>();
      c->Init(model, robot_info_, pinocchio_cache_, c_cfg);
      // Floating-base + surface contact 시 cdim != 3 인 column offset 을 정확히
      // 잡기 위해 필수. point-only 회로에서는 fallback(cdim=3) 으로도 동작하지만,
      // mixed point/surface 가 들어오면 무성 alignment 깨짐.
      c->SetContactManager(&contact_mgr_config_);
      formulation.AddConstraint(std::move(c));
    } else if (type == "joint_limit") {
      auto c = std::make_unique<rtc::tsid::JointLimitConstraint>();
      c->Init(model, robot_info_, pinocchio_cache_, c_cfg);
      formulation.AddConstraint(std::move(c));
    } else if (type == "friction_cone") {
      auto c = std::make_unique<rtc::tsid::FrictionConeConstraint>();
      c->Init(model, robot_info_, pinocchio_cache_, c_cfg);
      c->SetContactManager(&contact_mgr_config_);
      formulation.AddConstraint(std::move(c));
    } else if (type == "torque_limit") {
      auto c = std::make_unique<rtc::tsid::TorqueLimitConstraint>();
      c->Init(model, robot_info_, pinocchio_cache_, c_cfg);
      // τ = S·(M·a + h − Jcᵀ·λ) 역산 시 surface(cdim=6) λ 의 column offset 정확.
      c->SetContactManager(&contact_mgr_config_);
      formulation.AddConstraint(std::move(c));
    } else {
      RCLCPP_ERROR(logger_, "[wbc] unknown constraint type '%s' for entry '%s' — skipping",
                   type.c_str(), key.c_str());
    }
  }
}

// ── Posture task split gains (arm vs hand) ───────────────────────────────────

void DemoWbcController::ParsePostureSplitGains(const YAML::Node& posture_cfg) {
  // Both sub-maps required: a partial split (only `arm` or only `hand`) is
  // ambiguous, so fall back to the scalar/vector kp/kd PostureTask::Init read.
  posture_split_gains_ = false;
  if (!posture_cfg || !posture_cfg["arm"] || !posture_cfg["hand"]) {
    return;
  }
  const YAML::Node arm = posture_cfg["arm"];
  const YAML::Node hand = posture_cfg["hand"];
  if (arm["kp"]) {
    posture_kp_arm_ = arm["kp"].as<double>();
  }
  if (arm["kd"]) {
    posture_kd_arm_ = arm["kd"].as<double>();
  }
  if (hand["kp"]) {
    posture_kp_hand_ = hand["kp"].as<double>();
  }
  if (hand["kd"]) {
    posture_kd_hand_ = hand["kd"].as<double>();
  }
  posture_split_gains_ = true;
}

void DemoWbcController::AssemblePostureGains(int arm_dof, int full_dof, int nv,
                                             const std::array<int, kMaxFullDof>& ext_to_pin_v,
                                             double kp_arm, double kd_arm, double kp_hand,
                                             double kd_hand, Eigen::VectorXd& kp_out,
                                             Eigen::VectorXd& kd_out) noexcept {
  const int n = std::min(full_dof, static_cast<int>(kMaxFullDof));
  for (int i = 0; i < n; ++i) {
    const bool is_hand = (i >= arm_dof);
    const int pv = ext_to_pin_v[static_cast<std::size_t>(i)];
    if (pv < 0 || pv >= nv) {
      continue;
    }
    kp_out[pv] = is_hand ? kp_hand : kp_arm;
    kd_out[pv] = is_hand ? kd_hand : kd_arm;
  }
}

void DemoWbcController::ApplyPostureGains() noexcept {
  if (!posture_split_gains_ || !tsid_initialized_ || !joint_reorder_valid_) {
    return;
  }
  auto* task = tsid_controller_.Formulation().GetTask("posture");
  if (task == nullptr) {
    return;
  }
  const int nv = robot_info_.nv;
  if (nv <= 0) {
    return;
  }
  // Pre-fill with arm gains; AssemblePostureGains overwrites the hand slots
  // (and re-affirms arm slots) at their permuted Pinocchio velocity indices.
  Eigen::VectorXd kp = Eigen::VectorXd::Constant(nv, posture_kp_arm_);
  Eigen::VectorXd kd = Eigen::VectorXd::Constant(nv, posture_kd_arm_);
  AssemblePostureGains(arm_dof_, full_dof_, nv, ext_to_pin_v_, posture_kp_arm_, posture_kd_arm_,
                       posture_kp_hand_, posture_kd_hand_, kp, kd);
  static_cast<rtc::tsid::PostureTask*>(task)->SetGains(kp, kd);
  RCLCPP_INFO(logger_,
              "[wbc] posture split gains applied: arm kp=%.1f kd=%.1f, hand kp=%.1f kd=%.1f",
              posture_kp_arm_, posture_kd_arm_, posture_kp_hand_, posture_kd_hand_);
}

// ── Controller registry hooks ────────────────────────────────────────────────

void DemoWbcController::LoadConfig(const YAML::Node& cfg) {
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) {
    return;
  }

  // F-2: reset captured base_frame entries; LoadConfig may run twice
  // (PreConfigure + idempotent on_configure path).
  base_frame_yaml_entries_.clear();
  base_frame_mismatch_ = false;
  base_frame_mismatch_detail_.clear();

  // ── 1. Build models from system model config ──────────────────────────
  const auto* sys_cfg = GetSystemModelConfig();
  if (sys_cfg && !sys_cfg->urdf_path.empty()) {
    InitModels(*sys_cfg);
  } else if (!urdf_path_.empty()) {
    // Fallback: simple arm-only model
    rtc_urdf_bridge::ModelConfig model_cfg;
    model_cfg.urdf_path = urdf_path_;
    model_cfg.root_joint_type = "fixed";
    model_cfg.sub_models.push_back({"arm", "base_link", "tool0"});
    InitModels(model_cfg);
  }

  if (!full_model_ptr_) {
    RCLCPP_ERROR(logger_, "Full model not available — TSID disabled");
    return;
  }

  // ── 2. TSID initialization ────────────────────────────────────────────
  const auto tsid_node = cfg["tsid"];
  if (tsid_node) {
    const auto& model = *full_model_ptr_;

    // RobotModelInfo
    robot_info_.Build(model, tsid_node);

    // ContactManagerConfig — pass the whole tsid subtree; load() looks up
    // "contacts" itself (same pattern as rtc_tsid/test_force_task.cpp).
    // Passing tsid_node["contacts"] would double-index and silently yield
    // max_contacts = 0 (bug observed as "TSID initialized: contacts=0"
    // in the sim launch prior to this fix).
    if (tsid_node["contacts"]) {
      contact_mgr_config_.Load(tsid_node, model);
    }

    // PinocchioCache
    pinocchio_cache_.Init(full_model_ptr_, contact_mgr_config_);

    // ContactState
    contact_state_.Init(contact_mgr_config_.max_contacts);
    // Stage A-4: seed per-entry normals from YAML defaults (manager.contacts[*].
    // default_normal). Existing YAMLs without `normal:` keep world +Z.
    contact_state_.SeedNormals(contact_mgr_config_);

    // ControlReference & CommandOutput pre-allocate
    control_ref_.Init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated,
                      contact_mgr_config_.max_contact_vars);
    tsid_output_.Init(robot_info_.nv, robot_info_.n_actuated, contact_mgr_config_.max_contact_vars);

    // Stage A-3: pre-allocate λ_des buffer for ForceReferenceUpdater output.
    // Sized once here so the RT-tick SetForceReferences() path never resizes.
    force_lambda_des_ = Eigen::VectorXd::Zero(contact_mgr_config_.max_contact_vars);

    // Stage B-5: object-level WBC infrastructure. The controller owns the
    // ContactManager / GraspCache / ObjectFrame / IdentityObjectStateProvider
    // so the three object-level tasks share one populated-per-tick cache.
    // YAML schema (out-of-scope expansion: R_w / center_of_mass / vision
    // provider — Stage B-5+):
    //   tsid.object_frame:
    //     p_w: [x, y, z]   # object origin in world frame (default [0,0,0])
    //     mass: <scalar>   # used to seed w_obj_des = [0,0,m·g,0,0,0]
    if (tsid_node["object_frame"]) {
      const auto& of = tsid_node["object_frame"];
      if (of["p_w"] && of["p_w"].IsSequence() && of["p_w"].size() == 3) {
        object_frame_.p_w = Eigen::Vector3d(of["p_w"][0].as<double>(), of["p_w"][1].as<double>(),
                                            of["p_w"][2].as<double>());
      }
      // R_w stays identity (reserved per object_frame.hpp Stage B-2 note).
      if (of["mass"]) {
        const double m = of["mass"].as<double>();
        if (m >= 0.0) {
          object_mass_kg_ = m;
        }
      }
    }
    object_state_provider_.SetPose(object_frame_);
    object_state_provider_.SetTwist(Eigen::Matrix<double, 6, 1>::Zero());
    object_state_provider_.SetValid(true);

    // ContactManager + GraspCache + workspace allocation. All sized to
    // (max_contact_vars, nv) once here; RT path only writes top-left
    // active blocks via Eigen::Ref into these buffers.
    contact_mgr_.Init(contact_mgr_config_, robot_info_.nv);
    grasp_cache_.Init(contact_mgr_config_.max_contact_vars);
    grasp_G_workspace_ = Eigen::MatrixXd::Zero(6, contact_mgr_config_.max_contact_vars);
    squeeze_lambda_des_ = Eigen::VectorXd::Zero(contact_mgr_config_.max_contact_vars);

    // Parse optional `tsid.force_pi` block into the updater config. Missing
    // block → defaults from ForceReferenceUpdaterConfig (kp=0.5, ki=2.0,
    // i_max=2, lambda∈[0,20], f_des_default=2 N).
    if (tsid_node["force_pi"]) {
      const auto& fp = tsid_node["force_pi"];
      ::integrated_bringup::wbc::ForceReferenceUpdaterConfig fp_cfg =
          force_ref_updater_.GetConfig();
      if (fp["kp"]) {
        fp_cfg.kp = fp["kp"].as<double>();
      }
      if (fp["ki"]) {
        fp_cfg.ki = fp["ki"].as<double>();
      }
      if (fp["i_max"]) {
        fp_cfg.i_max = fp["i_max"].as<double>();
      }
      if (fp["lambda_min"]) {
        fp_cfg.lambda_min = fp["lambda_min"].as<double>();
      }
      if (fp["lambda_max"]) {
        fp_cfg.lambda_max = fp["lambda_max"].as<double>();
      }
      if (fp["f_des_default"]) {
        fp_cfg.f_des_default = fp["f_des_default"].as<double>();
      }
      force_ref_updater_.SetConfig(fp_cfg);
    }

    // Stage A-5b: contact activation ramp time (default 0.1 s ≈ 100 ms).
    if (tsid_node["contacts_default_ramp_sec"]) {
      const double rs = tsid_node["contacts_default_ramp_sec"].as<double>();
      if (rs >= 0.0) {
        contact_ramp_sec_ = rs;
      }
    }

    // ControlState pre-allocate
    ctrl_state_.q = Eigen::VectorXd::Zero(robot_info_.nq);
    ctrl_state_.v = Eigen::VectorXd::Zero(robot_info_.nv);

    // TSIDController init (creates formulation; tasks/constraints added below)
    tsid_controller_.Init(model, robot_info_, tsid_node);

    // Build tasks + constraints from YAML (auto-dispatch by `type` field)
    BuildTsidTasks(tsid_node);
    BuildTsidConstraints(tsid_node);

    // Verify contact frames resolved (catch mis-named fingertip frames early)
    for (const auto& c : contact_mgr_config_.contacts) {
      if (c.frame_id == 0) {
        RCLCPP_ERROR(logger_, "[wbc] contact '%s' frame '%s' not found in full model",
                     c.name.c_str(), c.frame_name.c_str());
      }
    }

    // Pre-resolve phase presets
    phase_preset_valid_.fill(false);
    if (tsid_node["phase_presets"]) {
      auto presets = rtc::tsid::LoadPhasePresets(tsid_node);
      auto map_preset = [&](WbcPhase phase, const std::string& name) {
        auto it = presets.find(name);
        if (it != presets.end()) {
          const auto idx = static_cast<std::size_t>(phase);
          phase_presets_[idx] = it->second;
          phase_preset_valid_[idx] = true;
        }
      };
      map_preset(WbcPhase::kIdle, "idle");
      map_preset(WbcPhase::kApproach, "approach");
      map_preset(WbcPhase::kPreGrasp, "pre_grasp");
      map_preset(WbcPhase::kClosure, "closure");
      map_preset(WbcPhase::kHold, "hold");
      map_preset(WbcPhase::kRelease, "release");
    }

    // Cache per-phase SE3-task activity (YAML is SSoT — phase ID hardcoding
    // would drift the moment se3_tcp is toggled off in a phase preset).
    // Consumed by OnPhaseEnter (edge detection) + ComputeTSIDPosition
    // (per-tick trajectory gate).
    se3_task_active_in_phase_.fill(false);
    for (int p = 0; p < kNumPhases; ++p) {
      const auto idx = static_cast<std::size_t>(p);
      if (!phase_preset_valid_[idx]) {
        continue;
      }
      for (const auto& tp : phase_presets_[idx].task_presets) {
        if (tp.task_name == "se3_tcp" && tp.active) {
          se3_task_active_in_phase_[idx] = true;
          break;
        }
      }
    }

    tsid_initialized_ = true;
    RCLCPP_INFO(logger_, "TSID initialized: nq=%d nv=%d n_act=%d contacts=%d", robot_info_.nq,
                robot_info_.nv, robot_info_.n_actuated, contact_mgr_config_.max_contacts);
  }

  // ── 3. Integration buffers ────────────────────────────────────────────
  const int nv = full_model_ptr_->nv;
  q_curr_full_ = Eigen::VectorXd::Zero(nv);
  v_curr_full_ = Eigen::VectorXd::Zero(nv);
  q_next_full_ = Eigen::VectorXd::Zero(nv);
  v_next_full_ = Eigen::VectorXd::Zero(nv);
  // Same layout as q_curr_full_ so it is a drop-in replacement for the
  // posture reference (control_ref_.q_des = q_curr_full_) on the RT path.
  q_des_target_full_ = Eigen::VectorXd::Zero(nv);
  v_seed_ = Eigen::VectorXd::Zero(nv);

  // Joint limits with safety margins + force-rate filter (required)
  if (!cfg["integration"] || !cfg["integration"].IsMap()) {
    throw std::runtime_error("demo_wbc_controller: required 'integration' section is missing");
  }
  {
    const auto int_node = cfg["integration"];
    position_margin_ = int_node["position_margin"].as<double>(0.02);
    velocity_scale_ = int_node["velocity_scale"].as<double>(0.95);
    // Optional: bound (rad/s²) on the one-tick command-velocity step when the
    // integrator re-seeds from measured q̇ on a target update. Default rarely
    // engages when the low-level tracker keeps v_curr ≈ previous command.
    v_jerk_limit_ = int_node["reseed_velocity_jerk"].as<double>(v_jerk_limit_);
    // Optional experimental toggle: integrate from the freshly measured (q, v)
    // each tick (closed-loop) instead of the carry-forward command buffers
    // (open-loop). Default false preserves the carry-forward behaviour.
    integrate_from_measured_ = int_node["integrate_from_measured"].as<bool>(false);

    if (!int_node["force_rate_alpha"]) {
      throw std::runtime_error(
          "demo_wbc_controller: required 'integration.force_rate_alpha' "
          "is missing");
    }
    const double alpha = int_node["force_rate_alpha"].as<double>();
    if (!(alpha >= 0.0 && alpha <= 1.0)) {
      throw std::runtime_error(
          "demo_wbc_controller: 'integration.force_rate_alpha' out of "
          "range [0, 1]");
    }
    force_rate_alpha_ = static_cast<float>(alpha);
  }
  q_min_clamped_ = full_model_ptr_->lowerPositionLimit.array() + position_margin_;
  q_max_clamped_ = full_model_ptr_->upperPositionLimit.array() - position_margin_;
  v_limit_ = full_model_ptr_->velocityLimit * velocity_scale_;

  // ── 4. FSM thresholds ─────────────────────────────────────────────────
  if (cfg["fsm"]) {
    const auto fsm = cfg["fsm"];
    epsilon_approach_ = fsm["epsilon_approach"].as<double>(0.01);
    epsilon_pregrasp_ = fsm["epsilon_pregrasp"].as<double>(0.005);
    force_contact_threshold_ = fsm["force_contact_threshold"].as<double>(0.2);
    min_contacts_for_hold_ = fsm["min_contacts_for_hold"].as<int>(min_contacts_for_hold_);
    slip_rate_threshold_ = fsm["slip_rate_threshold"].as<double>(slip_rate_threshold_);
    deformation_threshold_ = fsm["deformation_threshold"].as<double>(deformation_threshold_);
    max_qp_fail_before_fallback_ = fsm["max_qp_fail_before_fallback"].as<int>(5);
    // Clamp at >= 1 tick of the slowest supported control rate (100 Hz → 10 ms).
    // release_ramp_sec=0 would snap all contact activations to 0 in a single
    // tick, deactivating friction_cone rows simultaneously with active EOM
    // constraints → one-tick ill-conditioned QP.
    release_ramp_sec_ = std::max(0.01, fsm["release_ramp_sec"].as<double>(release_ramp_sec_));
    auto g = gains_lock_.Load();
    g.arm_trajectory_speed =
        std::max(1e-6, fsm["approach_speed"].as<double>(g.arm_trajectory_speed));
    gains_lock_.Store(g);
  }

  // ── 4c. Shared grasp thresholds (mirrors joint/task) ──────────────────
  // Defaults from demo_shared.yaml, overridden by per-controller cfg keys.
  // grasp_contact_threshold is sensor-A-only (consulted when
  // has_native_contact_=true); the other two are common across sensor types.
  {
    DemoSharedConfig shared;
    const std::string variant = node_ && node_->has_parameter("config_variant")
                                    ? node_->get_parameter("config_variant").as_string()
                                    : std::string{};
    LoadDemoSharedYamlFile(shared, variant);
    ApplyDemoSharedConfig(cfg, shared);
    auto g = gains_lock_.Load();
    g.grasp_contact_threshold = shared.grasp_contact_threshold;
    g.grasp_force_threshold = shared.grasp_force_threshold;
    g.grasp_min_fingertips = shared.grasp_min_fingertips;
    gains_lock_.Store(g);
  }

  // ── 4b. Lift L2: E-STOP arm safe position (required) ─────────────────
  //
  // Runtime arm_dof_ is established from the YAML's `estop.arm_safe_position`
  // length. This is the authoritative arm DoF at LoadConfig time (device
  // configs aren't yet available — they arrive in OnDeviceConfigsSet, which
  // cross-checks joint_state_names size against arm_dof_).
  if (!cfg["estop"] || !cfg["estop"]["arm_safe_position"] ||
      !cfg["estop"]["arm_safe_position"].IsSequence()) {
    throw std::runtime_error(
        "demo_wbc_controller: required 'estop.arm_safe_position' must be a sequence");
  }
  {
    const auto seq_size = cfg["estop"]["arm_safe_position"].size();
    if (seq_size == 0 || seq_size > static_cast<std::size_t>(kMaxArmDof)) {
      throw std::runtime_error("demo_wbc_controller: 'estop.arm_safe_position' size " +
                               std::to_string(seq_size) + " out of range [1, " +
                               std::to_string(kMaxArmDof) + "]");
    }
    arm_dof_ = static_cast<int>(seq_size);
    const auto sp = ParseArmSafePosition(cfg, seq_size, "demo_wbc_controller");
    safe_position_.fill(0.0);
    for (std::size_t i = 0; i < seq_size; ++i) {
      safe_position_[i] = sp[i];
    }
  }

  // ── 5. Trajectory speeds ──────────────────────────────────────────────
  {
    auto g = gains_lock_.Load();
    if (cfg["arm_trajectory_speed"]) {
      g.arm_trajectory_speed = std::max(1e-6, cfg["arm_trajectory_speed"].as<double>());
    }
    if (cfg["hand_trajectory_speed"]) {
      g.hand_trajectory_speed = std::max(1e-6, cfg["hand_trajectory_speed"].as<double>());
    }
    if (cfg["arm_max_traj_velocity"]) {
      g.arm_max_traj_velocity = cfg["arm_max_traj_velocity"].as<double>();
    }
    if (cfg["hand_max_traj_velocity"]) {
      g.hand_max_traj_velocity = cfg["hand_max_traj_velocity"].as<double>();
    }
    if (cfg["tcp_trajectory_speed"]) {
      g.tcp_trajectory_speed = std::max(1e-6, cfg["tcp_trajectory_speed"].as<double>());
    }
    if (cfg["tcp_trajectory_angular_speed"]) {
      g.tcp_trajectory_angular_speed =
          std::max(1e-6, cfg["tcp_trajectory_angular_speed"].as<double>());
    }
    if (cfg["tcp_max_traj_velocity"]) {
      g.tcp_max_traj_velocity = cfg["tcp_max_traj_velocity"].as<double>();
    }
    if (cfg["tcp_max_traj_angular_velocity"]) {
      g.tcp_max_traj_angular_velocity = cfg["tcp_max_traj_angular_velocity"].as<double>();
    }
    if (cfg["pi_rotation_margin"]) {
      g.pi_rotation_margin = cfg["pi_rotation_margin"].as<double>();
    }
    gains_lock_.Store(g);
  }

  // ── 6. Command type ───────────────────────────────────────────────────
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }

  // ── 7. MPC integration ────────────────────────────────────────────────
  //
  // If `mpc.enabled: true`, size the reference buffers and initialise the
  // MPC solution manager. The thread itself is spawned in
  // SpawnMpcThreadIfNeeded (called from on_activate on the aux thread) so the
  // heap-allocating MPCFactory::Create stays off the RT path. If `mpc:` is
  // missing or disabled, all MPC paths are
  // short-circuited and TSID self-hold behaviour is preserved bit-exactly.
  //
  // `mpc.engine` (default "mock") selects MockMPCThread vs
  // HandlerMPCThread. Handler mode additionally loads mpc/phase_config.yaml
  // + mpc/contact_light.yaml + mpc/contact_rich.yaml from the package share
  // and pre-builds the RobotModelHandler + GraspPhaseManager for startup.
  if (const auto mpc_cfg = cfg["mpc"]; mpc_cfg && full_model_ptr_) {
    mpc_enabled_ = mpc_cfg["enabled"] && mpc_cfg["enabled"].as<bool>();

    const auto engine_str = mpc_cfg["engine"] ? mpc_cfg["engine"].as<std::string>("mock") : "mock";
    if (engine_str == "handler") {
      mpc_engine_ = MpcEngine::kHandler;
    } else if (engine_str == "mock") {
      mpc_engine_ = MpcEngine::kMock;
    } else {
      RCLCPP_ERROR(logger_, "[wbc] unknown mpc.engine '%s' — falling back to 'mock'",
                   engine_str.c_str());
      mpc_engine_ = MpcEngine::kMock;
    }

    if (const auto freq_node = mpc_cfg["target_frequency_hz"]) {
      const double freq = freq_node.as<double>(20.0);
      if (freq > 0.0) {
        mpc_target_frequency_hz_ = freq;
      } else {
        RCLCPP_WARN(logger_,
                    "[wbc] mpc.target_frequency_hz=%.3f non-positive — "
                    "keeping default %.1f Hz",
                    freq, mpc_target_frequency_hz_);
      }
    }

    // TSID and MPC share full_model_ptr_ (the reduced tree when available).
    // No projection layer needed — ComputeReference writes directly into the
    // reference buffers that TSID consumes via control_ref_.
    const int mpc_nq = full_model_ptr_->nq;
    const int mpc_nv = full_model_ptr_->nv;
    const int mpc_n_contact = contact_mgr_config_.max_contact_vars;

    mpc_q_ref_ = Eigen::VectorXd::Zero(mpc_nq);
    mpc_v_ref_ = Eigen::VectorXd::Zero(mpc_nv);
    mpc_a_ff_ = Eigen::VectorXd::Zero(mpc_nv);
    mpc_lambda_ref_ = Eigen::VectorXd::Zero(std::max(1, mpc_n_contact));
    mpc_u_fb_ = Eigen::VectorXd::Zero(mpc_nv);

    mpc_manager_.Init(mpc_cfg, mpc_nq, mpc_nv, mpc_n_contact);
    const char* active_engine = (mpc_engine_ == MpcEngine::kHandler) ? "handler" : "mock";
    RCLCPP_INFO(logger_,
                "MPC integration: enabled=%d engine=%s (yaml=%s) nq=%d nv=%d "
                "n_contact=%d",
                mpc_enabled_, active_engine, engine_str.c_str(), mpc_nq, mpc_nv, mpc_n_contact);

    if (mpc_engine_ == MpcEngine::kHandler) {
      // Resolve factory YAML paths relative to the integrated_bringup share dir.
      std::string share;
      try {
        share = ament_index_cpp::get_package_share_directory("integrated_bringup");
      } catch (...) {
        RCLCPP_ERROR(logger_,
                     "[wbc] cannot resolve integrated_bringup share dir "
                     "— handler mode disabled");
        mpc_engine_ = MpcEngine::kMock;
      }

      if (mpc_engine_ == MpcEngine::kHandler) {
        const auto phase_path =
            mpc_cfg["phase_config_path"]
                ? mpc_cfg["phase_config_path"].as<std::string>()
                : std::string{"config/ur5e_hand/controllers/mpc/phase_config.yaml"};
        const auto light_path =
            mpc_cfg["contact_light_path"]
                ? mpc_cfg["contact_light_path"].as<std::string>()
                : std::string{"config/ur5e_hand/controllers/mpc/contact_light.yaml"};
        const auto rich_path =
            mpc_cfg["contact_rich_path"]
                ? mpc_cfg["contact_rich_path"].as<std::string>()
                : std::string{"config/ur5e_hand/controllers/mpc/contact_rich.yaml"};

        const auto join = [&](const std::string& p) { return share + "/" + p; };

        try {
          phase_cfg_ = YAML::LoadFile(join(phase_path));
          mpc_light_cfg_ = YAML::LoadFile(join(light_path));
          mpc_rich_cfg_ = YAML::LoadFile(join(rich_path));
        } catch (const std::exception& e) {
          RCLCPP_ERROR(logger_,
                       "[wbc] failed to load handler YAML (%s) — "
                       "handler mode disabled",
                       e.what());
          mpc_engine_ = MpcEngine::kMock;
        }
      }

      if (mpc_engine_ == MpcEngine::kHandler) {
        // Build RobotModelHandler from contact_light.yaml's `model:` subtree
        // (identical to contact_rich.yaml by contract — cross-mode swap
        // requires matching contact_frames).
        const auto model_node = mpc_light_cfg_["mpc"] && mpc_light_cfg_["mpc"]["model"]
                                    ? mpc_light_cfg_["mpc"]["model"]
                                    : YAML::Node{};
        // F-2: capture MPC model.base_frame for OnDeviceConfigsSet
        // consistency check. Both contact_light and contact_rich share the
        // same base_frame contract; record each so a divergent edit is
        // surfaced as a mismatch against the device root_link.
        if (model_node && model_node["base_frame"]) {
          base_frame_yaml_entries_.emplace_back("mpc.light.model.base_frame",
                                                model_node["base_frame"].as<std::string>());
        }
        if (mpc_rich_cfg_ && mpc_rich_cfg_["mpc"] && mpc_rich_cfg_["mpc"]["model"] &&
            mpc_rich_cfg_["mpc"]["model"]["base_frame"]) {
          base_frame_yaml_entries_.emplace_back(
              "mpc.rich.model.base_frame",
              mpc_rich_cfg_["mpc"]["model"]["base_frame"].as<std::string>());
        }
        mpc_model_handler_ = std::make_unique<rtc::mpc::RobotModelHandler>();
        const auto model_err = mpc_model_handler_->Init(*full_model_ptr_, model_node);
        if (model_err != rtc::mpc::RobotModelInitError::kNoError) {
          RCLCPP_ERROR(logger_,
                       "[wbc] RobotModelHandler::Init failed (code %d) — "
                       "handler mode disabled",
                       static_cast<int>(model_err));
          mpc_model_handler_.reset();
          mpc_engine_ = MpcEngine::kMock;
        }
      }

      if (mpc_engine_ == MpcEngine::kHandler) {
        // GraspPhaseManager is built eagerly here (non-RT) so any YAML
        // schema errors surface before the RT thread starts. It is handed
        // over to HandlerMPCThread::Configure in SpawnMpcThreadIfNeeded.
        auto pm =
            std::make_unique<integrated_bringup::phase::GraspPhaseManager>(*mpc_model_handler_);
        const auto phase_err = pm->Load(
            phase_cfg_["grasp_phase_manager"] ? phase_cfg_["grasp_phase_manager"] : phase_cfg_);
        if (phase_err != integrated_bringup::phase::GraspPhaseInitError::kNoError) {
          RCLCPP_ERROR(logger_,
                       "[wbc] GraspPhaseManager::Load failed (code %d) — "
                       "handler mode disabled",
                       static_cast<int>(phase_err));
          mpc_engine_ = MpcEngine::kMock;
        } else {
          phase_manager_owned_ = std::move(pm);
          phase_manager_ptr_ = phase_manager_owned_.get();
        }
      }

      RCLCPP_INFO(logger_, "[wbc] MPC handler-mode preconditions %s",
                  mpc_engine_ == MpcEngine::kHandler ? "ready" : "DEGRADED");
    }
  }

  // ── Phase C: parse `logs:` section ──────────────────────────────────────
  parsed_log_entries_.clear();
  if (cfg["logs"]) {
    if (!cfg["logs"].IsSequence()) {
      throw std::runtime_error("DemoWbcController: 'logs' must be a sequence");
    }
    for (const auto& entry : cfg["logs"]) {
      if (!entry.IsMap() || !entry["msg_type"]) {
        throw std::runtime_error("DemoWbcController: each `logs` entry needs `msg_type`");
      }
      ParsedLogEntry e;
      e.msg_type = entry["msg_type"].as<std::string>();
      if (entry["instance"]) {
        e.instance = entry["instance"].as<std::string>();
      }
      if (e.msg_type != "rtc_msgs/DeviceStateLog" && e.msg_type != "rtc_msgs/DeviceSensorLog" &&
          e.msg_type != "integrated_bringup/DeviceWbcLog" &&
          e.msg_type != "integrated_bringup/WbcDiagLog") {
        throw std::runtime_error("DemoWbcController: unknown msg_type in `logs`: " + e.msg_type);
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }
}

void DemoWbcController::OnDeviceConfigsSet() {
  // ── Runtime DoF resolution ─────────────────────────────────────────────
  // arm_dof_ was set from YAML `estop.arm_safe_position` in LoadConfig.
  // Here we resolve hand_dof_ from the secondary device's joint_state_names
  // (0 when no secondary device exists), cap-check, and cross-validate the
  // arm side against the primary device's joint_state_names.
  if (const auto* primary_cfg = GetDeviceNameConfig(GetPrimaryDeviceName())) {
    const auto js_size = static_cast<int>(primary_cfg->joint_state_names.size());
    if (js_size > 0 && js_size != arm_dof_) {
      RCLCPP_ERROR(logger_,
                   "[wbc] arm DoF mismatch: estop.arm_safe_position=%d but primary device "
                   "'%s' joint_state_names size=%d",
                   arm_dof_, GetPrimaryDeviceName().c_str(), js_size);
    }
  }
  hand_dof_ = 0;
  if (const auto secondary = GetSecondaryDeviceName(); !secondary.empty()) {
    if (const auto* cfg = GetDeviceNameConfig(secondary)) {
      hand_dof_ = static_cast<int>(cfg->joint_state_names.size());
    }
  }
  if (hand_dof_ < 0 || hand_dof_ > kMaxHandDof) {
    RCLCPP_ERROR(logger_, "[wbc] hand DoF %d exceeds capacity kMaxHandDof=%d — clamping", hand_dof_,
                 kMaxHandDof);
    hand_dof_ = std::min(std::max(hand_dof_, 0), kMaxHandDof);
  }
  full_dof_ = arm_dof_ + hand_dof_;

  // ── Arm frame IDs ─────────────────────────────────────────────────────
  // arm_handle_ is null when the model wasn't built (e.g. unit tests that
  // exercise lifecycle hooks without a URDF). Skip frame resolution in
  // that case; consistency checks below remain valid.
  if (auto* cfg = GetDeviceNameConfig(GetPrimaryDeviceName()); cfg && arm_handle_) {
    if (cfg->urdf && !cfg->urdf->tip_link.empty()) {
      auto fid = arm_handle_->GetFrameId(cfg->urdf->tip_link);
      if (fid != 0) {
        tip_frame_id_ = fid;
      }
    }
    if (cfg->urdf && !cfg->urdf->root_link.empty()) {
      auto fid = arm_handle_->GetFrameId(cfg->urdf->root_link);
      if (fid != 0) {
        root_frame_id_ = fid;
        use_root_frame_ = true;
      }
    }
  }

  // F-2: validate that every captured SE3/MPC `base_frame` YAML value
  // matches the primary arm device's `urdf.root_link`. A mismatch means
  // the user edited one knob but not the other — silently lifting the
  // wrong frame would produce a quietly broken TCP target. We cannot
  // throw from here (CM calls SetDeviceNameConfigs without a try/catch),
  // so flag the failure for on_configure to surface as
  // CallbackReturn::FAILURE.
  if (!base_frame_yaml_entries_.empty()) {
    const auto* primary_cfg = GetDeviceNameConfig(GetPrimaryDeviceName());
    if (primary_cfg && primary_cfg->urdf && !primary_cfg->urdf->root_link.empty()) {
      const auto& expected = primary_cfg->urdf->root_link;
      for (const auto& [label, value] : base_frame_yaml_entries_) {
        if (value != expected) {
          if (!base_frame_mismatch_) {
            base_frame_mismatch_detail_ =
                label + "='" + value + "' != urdf.root_link='" + expected + "'";
          }
          base_frame_mismatch_ = true;
          RCLCPP_ERROR(logger_,
                       "[wbc] base_frame mismatch: %s='%s' but '%s' device "
                       "urdf.root_link='%s'. SE3/MPC reference frame must match.",
                       label.c_str(), value.c_str(), GetPrimaryDeviceName().c_str(),
                       expected.c_str());
        }
      }
    }
  }

  // ── Lift L1: per-device joint limits loaded from device_name_configs_ ──
  // in topic_config_ group order; missing slots get ±2π / 2 rad/s fallbacks
  // so RT clamping always has valid bounds.
  LoadDeviceLimitsFromConfig(device_position_lower_, device_position_upper_, device_max_velocity_,
                             -6.2832, 6.2832, 2.0);

  // ── Joint reorder map ─────────────────────────────────────────────────
  BuildJointReorderMap();

  // Phase C: capture joint/sensor names for CSV header expansion.
  if (auto* cfg = GetDeviceNameConfig(GetPrimaryDeviceName()); cfg) {
    primary_joint_names_ = cfg->joint_state_names;
  }
  if (const auto secondary = GetSecondaryDeviceName(); !secondary.empty()) {
    if (auto* cfg = GetDeviceNameConfig(secondary); cfg) {
      secondary_joint_names_ = cfg->joint_state_names;
      secondary_motor_names_ = cfg->motor_state_names;
      secondary_sensor_names_ = cfg->sensor_names;
    }
  }

  // Cache fingertip sensor capability flags from the secondary (hand) device.
  // See demo_joint_controller.hpp for rationale.
  const auto secondary_name = GetSecondaryDeviceName();
  if (!secondary_name.empty()) {
    if (const auto layout = GetSensorLayout(secondary_name); layout.has_value()) {
      has_native_contact_ = layout->has_native_contact;
      has_native_displacement_ = layout->has_native_displacement;
    }
  }
  RCLCPP_INFO(logger_,
              "[wbc] fingertip sensor capability: has_native_contact=%d "
              "has_native_displacement=%d (secondary='%s')",
              static_cast<int>(has_native_contact_),
              static_cast<int>(has_native_displacement_), secondary_name.c_str());
}

DemoWbcController::FingertipReport DemoWbcController::GetFingertipReportForTesting(
    int fingertip_idx) const noexcept {
  FingertipReport r{};
  if (fingertip_idx < 0 || fingertip_idx >= static_cast<int>(rtc::kMaxSensorGroups)) {
    return r;
  }
  const auto& ft = fingertip_data_[static_cast<std::size_t>(fingertip_idx)];
  r.force_magnitude = ft.force_magnitude;
  r.force_rate = ft.force_rate;
  // FingertipReport.contact_flag is the public test API; controller now
  // derives in_contact (bool) instead of carrying a raw float flag.
  r.contact_flag = ft.in_contact ? 1.0F : 0.0F;
  r.valid = ft.valid;
  return r;
}

// ── RT control loop ──────────────────────────────────────────────────────────

ControllerOutput DemoWbcController::Compute(const ControllerState& state) noexcept {
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();

  ReadState(state);
  DrainTargetSlot(state);

  // E-STOP takes priority over FSM
  estop_active_ = estopped_.load(std::memory_order_acquire);
  if (estop_active_) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    // E-8 (resolved): do NOT push the WBC state / diag channels on the E-STOP
    // path — tsid_output_ is stale there. Sensor logging continues (raw HW
    // telemetry is meaningful during E-STOP).
    if (secondary_sensor_log_handle_) {
      integrated_bringup::DeviceSensorLogPod pod{};
      FillDeviceSensorLogPod(state, /*device_idx=*/1, num_active_fingertips_, pod);
      secondary_sensor_log_handle_.Push(pod);
    }
    return out;
  }

  // Seed deferred this tick (device 0 not valid yet — see DrainTargetSlot).
  // Command the passthrough hold set there and skip TSID until the idle hold
  // target is seeded from a real measured configuration; otherwise
  // ComputeControl would run TSID against the un-seeded (zero) references.
  if (!target_initialized_.load(std::memory_order_acquire)) {
    auto out = WriteJointCommand(state);
    out.command_type = command_type_;
    return out;
  }

  ComputeControl(state, dt);
  // Output composition split by consumer (wire / log / publish). See
  // demo_joint_controller.hpp for the bucket assignment rationale.
  auto output = WriteJointCommand(state);
  FillLogOutput(state, output);
  FillPublishOutput(state, output);

  // ── Phase C: push log PODs (only from inside the normal Compute() path) ──
  // WBC arm/hand state use the superset DeviceWbcLog (a_opt / SE3 ramp /
  // fingertip force); wbc_diag is the per-tick TSID/QP solution.
  if (primary_wbc_log_handle_) {
    integrated_bringup::DeviceWbcLogPod pod{};
    FillDeviceWbcLogPod(state, output, /*device_idx=*/0, /*role=*/0, pod);
    primary_wbc_log_handle_.Push(pod);
  }
  if (secondary_wbc_log_handle_) {
    integrated_bringup::DeviceWbcLogPod pod{};
    FillDeviceWbcLogPod(state, output, /*device_idx=*/1, /*role=*/1, pod);
    secondary_wbc_log_handle_.Push(pod);
  }
  if (wbc_diag_log_handle_) {
    integrated_bringup::WbcDiagLogPod pod{};
    FillWbcDiagLogPod(state, pod);
    wbc_diag_log_handle_.Push(pod);
  }
  if (secondary_sensor_log_handle_) {
    integrated_bringup::DeviceSensorLogPod pod{};
    FillDeviceSensorLogPod(state, /*device_idx=*/1, num_active_fingertips_, pod);
    secondary_sensor_log_handle_.Push(pod);
  }
  return output;
}

// ── Phase 1: Read state ──────────────────────────────────────────────────────

void DemoWbcController::SetDeviceTarget(int device_idx, std::span<const double> target) noexcept {
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices) {
    return;
  }
  PendingTarget pending{};
  pending.device_idx = device_idx;
  const std::size_t nch = std::min(target.size(), static_cast<std::size_t>(kMaxDeviceChannels));
  pending.num_values = static_cast<int>(nch);
  for (std::size_t i = 0; i < nch; ++i) {
    pending.values[i] = target[i];
  }
  // Off-RT marshal — the RT thread drains pending_targets_ inside Compute()
  // and is the SOLE writer of target_seqlock_.
  (void)pending_targets_.Push(pending);
}

// RT-thread-only. Refreshes current_target_slot_, drains pending entries,
// runs first-tick self-init (seeded from current device state).
void DemoWbcController::DrainTargetSlot(const ControllerState& state) noexcept {
  current_target_slot_ = target_seqlock_.Load();
  bool slot_dirty = false;

  if (!target_initialized_.load(std::memory_order_acquire)) {
    // Fallback DoF when LoadConfig/OnDeviceConfigsSet hasn't populated runtime
    // dimensions (e.g. unit tests that bypass YAML).
    if (arm_dof_ == 0 && state.num_devices > 0) {
      arm_dof_ = std::min(state.devices[0].num_channels, kMaxArmDof);
    }
    if (hand_dof_ == 0 && state.num_devices > 1 && state.devices[1].valid) {
      hand_dof_ = std::min(state.devices[1].num_channels, kMaxHandDof);
    }
    if (full_dof_ == 0) {
      full_dof_ = arm_dof_ + hand_dof_;
    }

    // First-tick self-init must capture a VALID measured arm configuration.
    // If device 0 (arm) has not published a real state yet (the simulator is
    // not yet streaming on the first control tick), seeding now would lock the
    // idle hold target (posture ref + SE3 hold pose) to q=0; idle then
    // regulates toward the zero config and drives the arm hard toward it,
    // saturating the joint/torque-limit constraints into an infeasible QP
    // (ProxQP grind 0.5–1.4 s → fallback cycling). Defer: leave
    // target_initialized_ false and command a passthrough hold so the next
    // tick re-attempts the seed from a real measured configuration. The hand
    // path below already guards on dev.valid; device 0 is the missing guard.
    if (state.num_devices == 0 || !state.devices[0].valid) {
      const auto& dev0 = state.devices[0];
      for (int i = 0; i < arm_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        robot_computed_.positions[idx] = dev0.positions[idx];
        robot_computed_.velocities[idx] = 0.0;
      }
      if (state.num_devices > 1 && state.devices[1].valid) {
        const auto& dev1 = state.devices[1];
        for (int i = 0; i < hand_dof_; ++i) {
          const auto idx = static_cast<std::size_t>(i);
          hand_computed_.positions[idx] = dev1.positions[idx];
          hand_computed_.velocities[idx] = 0.0;
        }
      }
      return;
    }

    // Robot arm: initialize trajectory at current position (zero velocity)
    {
      const auto& dev0 = state.devices[0];
      trajectory::JointSpaceTrajectory<kMaxArmDof>::State hold{};
      for (int i = 0; i < arm_dof_; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        current_target_slot_.targets[0][idx] = dev0.positions[idx];
        hold.positions[idx] = dev0.positions[idx];
        robot_computed_.positions[idx] = dev0.positions[idx];
        robot_computed_.velocities[idx] = 0.0;
      }
      robot_trajectory_.initialize(hold, hold, 0.01);
      robot_trajectory_time_ = 0.0;
      robot_new_target_pending_ = false;
    }

    // Hand
    for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices); ++d) {
      const auto& dev = state.devices[d];
      if (!dev.valid) {
        continue;
      }
      for (std::size_t i = 0;
           i < static_cast<std::size_t>(dev.num_channels) && i < kMaxDeviceChannels; ++i) {
        current_target_slot_.targets[d][i] = dev.positions[i];
      }
      if (d == 1) {
        trajectory::JointSpaceTrajectory<kMaxHandDof>::State hold{};
        for (int i = 0; i < hand_dof_; ++i) {
          const auto idx = static_cast<std::size_t>(i);
          hold.positions[idx] = dev.positions[idx];
          hand_computed_.positions[idx] = dev.positions[idx];
          hand_computed_.velocities[idx] = 0.0;
        }
        hand_trajectory_.initialize(hold, hold, 0.01);
        hand_trajectory_time_ = 0.0;
        hand_new_target_pending_ = false;
      }
    }

    // Reset FSM
    phase_ = WbcPhase::kIdle;
    tcp_goal_valid_ = false;
    current_target_slot_.tcp_goal_valid = false;
    qp_fail_count_ = 0;
    grasp_cmd_.store(0, std::memory_order_relaxed);

    // Idle hold snapshot from the measured first-tick configuration. kIdle's
    // OnPhaseEnter does not fire on enable (phase_ is already kIdle, so there is
    // no transition edge), so the posture reference (q_des_target_full_) and the
    // SE3 hold pose must be seeded here or idle would command the zero vector.
    SeedHoldFromMeasured(state);

    // For the same reason (no kIdle transition edge on enable), the idle phase
    // preset is never applied via OnPhaseEnter at startup. Apply it here, or the
    // TSID tasks keep their base `tasks:` config — se3_tcp / contact_consistency
    // / object_* all active at full weight — and idle runs as a multi-task fight
    // that drives large spurious accelerations into the integrator (QP failure →
    // fallback cycling). With the preset applied, idle is the configured
    // joint-space hold (posture-only by default).
    {
      const auto idle_idx = static_cast<std::size_t>(WbcPhase::kIdle);
      if (tsid_initialized_ && phase_preset_valid_[idle_idx]) {
        tsid_controller_.ApplyPhasePreset(phase_presets_[idle_idx]);
      }
    }

    // Idle starts in free space — there are no contacts to support. The Entry
    // default is activation=1.0, and OnPhaseEnter(kIdle) (which ramps it to 0)
    // does not fire on enable, so seed the contacts inactive here. Instant
    // (t_ramp=0) — unlike a post-grasp idle entry there is nothing to gently
    // release. The first ComputeTSIDPosition UpdateActivation snaps activation
    // to 0 so the very first solve already reports n_active=0.
    for (int i = 0; i < static_cast<int>(contact_state_.contacts.size()); ++i) {
      contact_state_.SetActivationTarget(i, 0.0, 0.0);
    }
    contact_state_.RecomputeActive(contact_mgr_config_);

    target_initialized_.store(true, std::memory_order_release);
    slot_dirty = true;
    // Seed the carry-forward integrator from the (measured) first-tick state.
    reseed_integration_pending_ = true;
  } else {
    // Restore RT-thread working SE3 from POD storage so reader sites see a
    // consistent rotation matrix every tick.
    tcp_goal_valid_ = current_target_slot_.tcp_goal_valid;
    if (tcp_goal_valid_) {
      std::memcpy(tcp_goal_.rotation().data(), current_target_slot_.tcp_goal_rot.data(),
                  sizeof(current_target_slot_.tcp_goal_rot));
      std::memcpy(tcp_goal_.translation().data(), current_target_slot_.tcp_goal_t.data(),
                  sizeof(current_target_slot_.tcp_goal_t));
    }
  }

  PendingTarget pending{};
  while (pending_targets_.Pop(pending)) {
    const auto didx = static_cast<std::size_t>(pending.device_idx);
    if (didx >= ControllerState::kMaxDevices) {
      continue;
    }
    const std::size_t nch = std::min(static_cast<std::size_t>(pending.num_values),
                                     static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < nch; ++i) {
      current_target_slot_.targets[didx][i] = pending.values[i];
    }
    if (pending.device_idx == 0) {
      robot_new_target_pending_ = true;
    } else if (pending.device_idx == 1) {
      hand_new_target_pending_ = true;
    }
    // Phase-independent: any target arrival re-seeds the integrator from the
    // current measured state (the integral's initial condition), regardless of
    // FSM phase. q_des_target_full_ itself is rebuilt at phase entry so the
    // posture + SE3 references stay a consistent snapshot.
    reseed_integration_pending_ = true;
    slot_dirty = true;
  }

  if (slot_dirty) {
    target_seqlock_.Store(current_target_slot_);
  }
}

// Aux-thread spawn of MPC thread. Called from on_activate (heap-allocating;
// must not run on the RT path). Idempotent — re-activation reuses the
// existing thread.
void DemoWbcController::SpawnMpcThreadIfNeeded() noexcept {
  if (mpc_enabled_ && !mpc_thread_) {
    const auto thread_configs = rtc::SelectThreadConfigs();
    const int nq = static_cast<int>(q_curr_full_.size());
    const int nv = static_cast<int>(v_curr_full_.size());

    rtc::mpc::MpcThreadLaunchConfig launch{};
    launch.main = thread_configs.mpc.main;
    launch.num_workers = thread_configs.mpc.num_workers;
    for (int i = 0; i < launch.num_workers && i < rtc::mpc::kMaxMpcWorkers; ++i) {
      launch.workers[static_cast<std::size_t>(i)] =
          thread_configs.mpc.workers[static_cast<std::size_t>(i)];
    }
    launch.target_frequency_hz = mpc_target_frequency_hz_;

    bool thread_started = false;

    if (mpc_engine_ == MpcEngine::kHandler && mpc_model_handler_ && phase_manager_owned_) {
      // Build the initial PhaseContext from the idle phase so the factory
      // can size its OCP + solver workspace. GraspPhaseManager and all
      // downstream solvers live in the reduced (mimic-locked) MPC model's
      // index space, so seed zeros at those dims — not the full_model's.
      Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(full_model_ptr_->nq);
      Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(full_model_ptr_->nv);
      Eigen::VectorXd sensor_zero = Eigen::VectorXd::Zero(0);
      const pinocchio::SE3 tcp_identity = pinocchio::SE3::Identity();
      const auto initial_ctx =
          phase_manager_owned_->Update(q_zero, v_zero, sensor_zero, tcp_identity, /*t=*/0.0);

      std::unique_ptr<rtc::mpc::MPCHandlerBase> handler;
      const auto status =
          rtc::mpc::MPCFactory::Create(mpc_light_cfg_, *mpc_model_handler_, initial_ctx, handler);

      if (status.error != rtc::mpc::MPCFactoryError::kNoError || !handler) {
        RCLCPP_ERROR(logger_,
                     "[wbc] MPCFactory::Create failed (err=%d, init=%d) — "
                     "falling back to mock engine",
                     static_cast<int>(status.error), static_cast<int>(status.init_error));
        mpc_engine_ = MpcEngine::kMock;
      } else {
        auto hthread = std::make_unique<rtc::mpc::HandlerMPCThread>();
        hthread->Configure(*mpc_model_handler_, std::move(handler), std::move(phase_manager_owned_),
                           mpc_light_cfg_, mpc_rich_cfg_);
        hthread->Init(mpc_manager_, launch);
        hthread->Start();
        mpc_thread_ = std::move(hthread);
        mpc_manager_.SetEnabled(true);
        thread_started = true;
        RCLCPP_INFO(logger_,
                    "MPC thread started (handler): core=%d prio=%d workers=%d "
                    "freq=%.1f Hz",
                    launch.main.cpu_core, launch.main.sched_priority, launch.num_workers,
                    launch.target_frequency_hz);
      }
    }

    if (!thread_started) {
      // Mock fallback — linear interpolation placeholder.
      auto mock = std::make_unique<rtc::mpc::MockMPCThread>();
      mock->Configure(nq, nv, /*horizon=*/10, /*dt_node=*/0.01);
      mock->SetTarget(q_curr_full_);
      mock->Init(mpc_manager_, launch);
      mock->Start();
      mpc_thread_ = std::move(mock);
      mpc_manager_.SetEnabled(true);
      RCLCPP_INFO(logger_,
                  "MPC thread started (mock): core=%d prio=%d workers=%d "
                  "freq=%.1f Hz",
                  launch.main.cpu_core, launch.main.sched_priority, launch.num_workers,
                  launch.target_frequency_hz);
    }
  }
}

// ── E-STOP ───────────────────────────────────────────────────────────────────

void DemoWbcController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void DemoWbcController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
  // Force the controller back through self-init on the next Compute() tick:
  // seeds target slot from current state, resets phase_ to kIdle, and rebuilds
  // trajectory baselines. RT-thread sole-writer invariant preserved (the
  // SeqLock store happens inside DrainTargetSlot on the next RT tick).
  target_initialized_.store(false, std::memory_order_release);
}

bool DemoWbcController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void DemoWbcController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

void DemoWbcController::PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept {
  const auto wbc_loaded = wbc_state_lock_.Load();
  PublishOwnedTopicsFromSnapshot(snap, owned_topics_, /*grasp=*/nullptr, /*wbc=*/&wbc_loaded,
                                 /*tof=*/nullptr);
}

}  // namespace integrated_bringup
