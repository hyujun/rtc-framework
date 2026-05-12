#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
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
#include "rtc_tsid/tasks/force_task.hpp"
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

  const auto* arm_cfg = GetDeviceNameConfig("ur5e");
  const auto* hand_cfg = GetDeviceNameConfig("hand");
  if (!arm_cfg || !hand_cfg) {
    RCLCPP_WARN(logger_, "Device configs not available, using identity mapping");
    for (int i = 0; i < kFullDof; ++i) {
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

  // Hand joints
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

  joint_reorder_valid_ = (ext_idx == kFullDof);
  if (!joint_reorder_valid_) {
    RCLCPP_ERROR(logger_, "Joint reorder incomplete: mapped %d/%d joints", ext_idx, kFullDof);
  }
}

// ── TSID task/constraint factories ───────────────────────────────────────────
//
// Dispatch by YAML `type` string. Tasks/constraints are created with
// unique_ptr and transferred to formulation via add_task/add_constraint.
// After construction each is init()'d with its own sub-node.

void DemoWbcController::BuildTsidTasks(const YAML::Node& tsid_node) {
  if (!full_model_ptr_ || !tsid_node || !tsid_node["tasks"]) {
    return;
  }
  const auto& model = *full_model_ptr_;
  auto& formulation = tsid_controller_.formulation();

  for (auto it = tsid_node["tasks"].begin(); it != tsid_node["tasks"].end(); ++it) {
    const auto key = it->first.as<std::string>();
    const auto& task_cfg = it->second;
    const auto type = task_cfg["type"].as<std::string>("");

    if (type == "posture") {
      auto task = std::make_unique<rtc::tsid::PostureTask>();
      task->init(model, robot_info_, pinocchio_cache_, task_cfg);
      formulation.add_task(std::move(task));
    } else if (type == "se3") {
      auto task = std::make_unique<rtc::tsid::SE3Task>();
      task->init(model, robot_info_, pinocchio_cache_, task_cfg);
      formulation.add_task(std::move(task));
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
      task->init(model, robot_info_, pinocchio_cache_, task_cfg);
      task->set_contact_manager(&contact_mgr_config_);
      formulation.add_task(std::move(task));
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
  auto& formulation = tsid_controller_.formulation();

  for (auto it = tsid_node["constraints"].begin(); it != tsid_node["constraints"].end(); ++it) {
    const auto key = it->first.as<std::string>();
    const auto& c_cfg = it->second;
    const auto type = c_cfg["type"].as<std::string>("");

    if (type == "eom") {
      auto c = std::make_unique<rtc::tsid::EomConstraint>();
      c->init(model, robot_info_, pinocchio_cache_, c_cfg);
      formulation.add_constraint(std::move(c));
    } else if (type == "joint_limit") {
      auto c = std::make_unique<rtc::tsid::JointLimitConstraint>();
      c->init(model, robot_info_, pinocchio_cache_, c_cfg);
      formulation.add_constraint(std::move(c));
    } else if (type == "friction_cone") {
      auto c = std::make_unique<rtc::tsid::FrictionConeConstraint>();
      c->init(model, robot_info_, pinocchio_cache_, c_cfg);
      c->set_contact_manager(&contact_mgr_config_);
      formulation.add_constraint(std::move(c));
    } else {
      RCLCPP_ERROR(logger_, "[wbc] unknown constraint type '%s' for entry '%s' — skipping",
                   type.c_str(), key.c_str());
    }
  }
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
    robot_info_.build(model, tsid_node);

    // ContactManagerConfig — pass the whole tsid subtree; load() looks up
    // "contacts" itself (same pattern as rtc_tsid/test_force_task.cpp).
    // Passing tsid_node["contacts"] would double-index and silently yield
    // max_contacts = 0 (bug observed as "TSID initialized: contacts=0"
    // in the sim launch prior to this fix).
    if (tsid_node["contacts"]) {
      contact_mgr_config_.load(tsid_node, model);
    }

    // PinocchioCache
    pinocchio_cache_.init(full_model_ptr_, contact_mgr_config_);

    // ContactState
    contact_state_.init(contact_mgr_config_.max_contacts);

    // ControlReference & CommandOutput pre-allocate
    control_ref_.init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated,
                      contact_mgr_config_.max_contact_vars);
    tsid_output_.init(robot_info_.nv, robot_info_.n_actuated, contact_mgr_config_.max_contact_vars);

    // ControlState pre-allocate
    ctrl_state_.q = Eigen::VectorXd::Zero(robot_info_.nq);
    ctrl_state_.v = Eigen::VectorXd::Zero(robot_info_.nv);

    // TSIDController init (creates formulation; tasks/constraints added below)
    tsid_controller_.init(model, robot_info_, tsid_node);

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
      auto presets = rtc::tsid::load_phase_presets(tsid_node);
      auto map_preset = [&](WbcPhase phase, const std::string& name) {
        auto it = presets.find(name);
        if (it != presets.end()) {
          const auto idx = static_cast<std::size_t>(phase);
          phase_presets_[idx] = it->second;
          phase_preset_valid_[idx] = true;
        }
      };
      map_preset(WbcPhase::kPreGrasp, "pre_grasp");
      map_preset(WbcPhase::kClosure, "closure");
      map_preset(WbcPhase::kHold, "hold");
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

  // Joint limits with safety margins + force-rate filter (required)
  if (!cfg["integration"] || !cfg["integration"].IsMap()) {
    throw std::runtime_error("demo_wbc_controller: required 'integration' section is missing");
  }
  {
    const auto int_node = cfg["integration"];
    position_margin_ = int_node["position_margin"].as<double>(0.02);
    velocity_scale_ = int_node["velocity_scale"].as<double>(0.95);

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
    force_hold_threshold_ = fsm["force_hold_threshold"].as<double>(1.0);
    min_contacts_for_hold_ = fsm["min_contacts_for_hold"].as<int>(min_contacts_for_hold_);
    slip_rate_threshold_ = fsm["slip_rate_threshold"].as<double>(slip_rate_threshold_);
    deformation_threshold_ = fsm["deformation_threshold"].as<double>(deformation_threshold_);
    max_qp_fail_before_fallback_ = fsm["max_qp_fail_before_fallback"].as<int>(5);
    auto g = gains_lock_.Load();
    g.arm_trajectory_speed =
        std::max(1e-6, fsm["approach_speed"].as<double>(g.arm_trajectory_speed));
    gains_lock_.Store(g);
  }

  // ── 4b. Lift L2: E-STOP arm safe position (required) ─────────────────
  {
    const auto sp =
        ParseArmSafePosition(cfg, static_cast<std::size_t>(kArmDof), "demo_wbc_controller");
    for (std::size_t i = 0; i < static_cast<std::size_t>(kArmDof); ++i) {
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
  // InitializeHoldPosition so it starts from the first valid RT tick's
  // state snapshot. If `mpc:` is missing or disabled, all MPC paths are
  // short-circuited and TSID self-hold behaviour is preserved bit-exactly.
  //
  // `mpc.engine` (default "mock") selects MockMPCThread vs
  // HandlerMPCThread. Handler mode additionally loads mpc/phase_config.yaml
  // + mpc/light_contact.yaml + mpc/contact_rich.yaml from the package share
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
            mpc_cfg["light_contact_path"]
                ? mpc_cfg["light_contact_path"].as<std::string>()
                : std::string{"config/ur5e_hand/controllers/mpc/light_contact.yaml"};
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
        // Build RobotModelHandler from light_contact.yaml's `model:` subtree
        // (identical to contact_rich.yaml by contract — cross-mode swap
        // requires matching contact_frames).
        const auto model_node = mpc_light_cfg_["mpc"] && mpc_light_cfg_["mpc"]["model"]
                                    ? mpc_light_cfg_["mpc"]["model"]
                                    : YAML::Node{};
        // F-2: capture MPC model.base_frame for OnDeviceConfigsSet
        // consistency check. Both light_contact and contact_rich share the
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
        // over to HandlerMPCThread::Configure in InitializeHoldPosition.
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
      if (e.msg_type != "rtc_msgs/DeviceStateLog" && e.msg_type != "rtc_msgs/DeviceSensorLog") {
        throw std::runtime_error("DemoWbcController: unknown msg_type in `logs`: " + e.msg_type);
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }
}

void DemoWbcController::OnDeviceConfigsSet() {
  // ── Arm frame IDs ─────────────────────────────────────────────────────
  // arm_handle_ is null when the model wasn't built (e.g. unit tests that
  // exercise lifecycle hooks without a URDF). Skip frame resolution in
  // that case; consistency checks below remain valid.
  if (auto* cfg = GetDeviceNameConfig("ur5e"); cfg && arm_handle_) {
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
  if (auto* cfg = GetDeviceNameConfig("ur5e"); cfg) {
    ur5e_joint_names_ = cfg->joint_state_names;
  }
  if (auto* cfg = GetDeviceNameConfig("hand"); cfg) {
    hand_joint_names_ = cfg->joint_state_names;
    hand_motor_names_ = cfg->motor_state_names;
    hand_sensor_names_ = cfg->sensor_names;
  }
}

DemoWbcController::FingertipReport DemoWbcController::GetFingertipReportForTesting(
    int fingertip_idx) const noexcept {
  FingertipReport r{};
  if (fingertip_idx < 0 || fingertip_idx >= static_cast<int>(rtc::kMaxFingertips)) {
    return r;
  }
  const auto& ft = fingertip_data_[static_cast<std::size_t>(fingertip_idx)];
  r.force_magnitude = ft.force_magnitude;
  r.force_rate = ft.force_rate;
  r.contact_flag = ft.contact_flag;
  r.valid = ft.valid;
  return r;
}

// ── RT control loop ──────────────────────────────────────────────────────────

ControllerOutput DemoWbcController::Compute(const ControllerState& state) noexcept {
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();

  ReadState(state);

  // E-STOP takes priority over FSM
  estop_active_ = estopped_.load(std::memory_order_acquire);
  if (estop_active_) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    if (ur5e_state_log_handle_) {
      integrated_bringup::DeviceStateLogPod pod{};
      FillUr5eStateLogPod(state, out, pod);
      ur5e_state_log_handle_.Push(pod);
    }
    if (hand_state_log_handle_) {
      integrated_bringup::DeviceStateLogPod pod{};
      FillHandStateLogPod(state, out, pod);
      hand_state_log_handle_.Push(pod);
    }
    if (hand_sensor_log_handle_) {
      integrated_bringup::DeviceSensorLogPod pod{};
      FillHandSensorLogPod(state, num_active_fingertips_, pod);
      hand_sensor_log_handle_.Push(pod);
    }
    return out;
  }

  ComputeControl(state, dt);
  auto output = WriteOutput(state);

  // ── Phase C: push log PODs (only from inside Compute()) ──────────────
  if (ur5e_state_log_handle_) {
    integrated_bringup::DeviceStateLogPod pod{};
    FillUr5eStateLogPod(state, output, pod);
    ur5e_state_log_handle_.Push(pod);
  }
  if (hand_state_log_handle_) {
    integrated_bringup::DeviceStateLogPod pod{};
    FillHandStateLogPod(state, output, pod);
    hand_state_log_handle_.Push(pod);
  }
  if (hand_sensor_log_handle_) {
    integrated_bringup::DeviceSensorLogPod pod{};
    FillHandSensorLogPod(state, num_active_fingertips_, pod);
    hand_sensor_log_handle_.Push(pod);
  }
  return output;
}

// ── Phase 1: Read state ──────────────────────────────────────────────────────

void DemoWbcController::SetDeviceTarget(int device_idx, std::span<const double> target) noexcept {
  std::lock_guard lock(target_mutex_);
  const auto didx = static_cast<std::size_t>(device_idx);
  if (didx >= device_targets_.size()) {
    return;
  }

  const auto n = std::min(target.size(), static_cast<std::size_t>(kMaxDeviceChannels));
  std::copy_n(target.data(), n, device_targets_[didx].data());

  if (device_idx == 0) {
    robot_new_target_.store(true, std::memory_order_release);
  } else if (device_idx == 1) {
    hand_new_target_.store(true, std::memory_order_release);
  }
}

void DemoWbcController::InitializeHoldPosition(const ControllerState& state) noexcept {
  std::lock_guard lock(target_mutex_);

  // Robot arm: initialize trajectory at current position (zero velocity)
  {
    const auto& dev0 = state.devices[0];
    trajectory::JointSpaceTrajectory<kNumRobotJoints>::State hold{};
    for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
      device_targets_[0][i] = dev0.positions[i];
      hold.positions[i] = dev0.positions[i];
      robot_computed_.positions[i] = dev0.positions[i];
      robot_computed_.velocities[i] = 0.0;
    }
    robot_trajectory_.initialize(hold, hold, 0.01);
    robot_trajectory_time_ = 0.0;
    robot_new_target_.store(false, std::memory_order_relaxed);
  }

  // Hand
  for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices); ++d) {
    const auto& dev = state.devices[d];
    if (!dev.valid)
      continue;
    for (std::size_t i = 0;
         i < static_cast<std::size_t>(dev.num_channels) && i < kMaxDeviceChannels; ++i) {
      device_targets_[d][i] = dev.positions[i];
    }
    if (d == 1) {
      trajectory::JointSpaceTrajectory<kHandMotorCount>::State hold{};
      for (std::size_t i = 0; i < kHandMotorCount; ++i) {
        hold.positions[i] = dev.positions[i];
        hand_computed_.positions[i] = dev.positions[i];
        hand_computed_.velocities[i] = 0.0;
      }
      hand_trajectory_.initialize(hold, hold, 0.01);
      hand_trajectory_time_ = 0.0;
      hand_new_target_.store(false, std::memory_order_relaxed);
    }
  }

  // Reset FSM
  phase_ = WbcPhase::kIdle;
  tcp_goal_valid_ = false;
  qp_fail_count_ = 0;
  grasp_cmd_.store(0, std::memory_order_relaxed);

  // ── MPC thread lifecycle ────────────────────────────────────────────────
  //
  // Spawn the MPC thread lazily on first InitializeHoldPosition call so it
  // starts from a known-good RT state. Subsequent calls (e.g. controller
  // re-init after E-STOP clear) leave the existing thread running.
  //
  // Handler mode additionally builds the initial PhaseContext from the idle
  // phase, runs MPCFactory::Create, and hands both the handler and the
  // pre-loaded GraspPhaseManager to HandlerMPCThread::Configure. On factory
  // failure we fall back to mock mode so the RT loop can still make progress.
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
}

bool DemoWbcController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void DemoWbcController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

void DemoWbcController::PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept {
  PublishOwnedTopicsFromSnapshot(snap, owned_topics_);
}

}  // namespace integrated_bringup
