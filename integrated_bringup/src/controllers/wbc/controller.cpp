#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "integrated_bringup/support/demo_shared_config.hpp"
#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_controller_interface/device_readability.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_math/se3/so3.hpp"

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
#include <pinocchio/math.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/constraints/contact_constraint.hpp"
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
  // For extended (closed-chain) hands the path-based 'wbc' tree locks off-path
  // actuated joints — a 4-bar finger's active DIP reaches the tip through a
  // passive coupler branch, so it falls outside the root→tip path and gets
  // locked (proto_1b: 13/16, reorder then fails). Prefer the builder's actuated
  // model: it locks only the loop-passives and keeps every actuated joint
  // movable, so nq==nv and the device's full actuated set maps 1:1 (16/16).
  // Non-extended (plain/mimic) URDFs get a null actuated model and fall through
  // to the existing tree/full path unchanged (byte-for-byte).
  // Model selection + q/v buffer alloc delegate to the shared cache (#174).
  // WBC uses SelectModel (not the bundled InitModel) because its cache Init must
  // wait until the TSID contact frame ids are parsed in LoadConfig — which itself
  // needs this selected model — so the cache().Init runs there, not here.
  (void)combined_cache_.SelectModel(*builder_, "[wbc]", logger_);

  RCLCPP_INFO(logger_, "Models initialized: arm nv=%d, control nq=%d nv=%d", arm_handle_->nv(),
              combined_cache_.model()->nq, combined_cache_.model()->nv);

  // #123 Phase 2: serial hand tree-model + fingertip/hand-root frame ids for
  // the loop-consistent fingertip FK publish surface (SetJointOrder + closed-
  // chain wiring run in OnDeviceConfigsSet once device configs are available).
  InitHandModel(config);
}

// ── #123 Phase 2: hand fingertip FK — serial tree-model + frame resolution ───
void DemoWbcController::InitHandModel(const rtc_urdf_bridge::ModelConfig& config) {
  namespace rub = rtc_urdf_bridge;
  // Secondary device name == tree_model name (robot-agnostic: e.g. "p1b" for
  // ur5e_p1b). Single-device controllers leave this empty → no hand FK.
  const auto secondary = GetSecondaryDeviceName();
  if (secondary.empty()) {
    return;
  }
  // Guard the tree-model lookup: a malformed config (secondary device declared
  // without a matching tree_model) would otherwise throw std::out_of_range,
  // caught by the base class as an on_configure FAILURE. Mirror the control-
  // model selection's fallback and graceful-degrade to no hand FK (#125 F2).
  try {
    hand_handle_ = std::make_unique<rub::RtModelHandle>(builder_->GetTreeModel(secondary));
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_,
                "[wbc] secondary device '%s' has no matching tree_model (%s) — "
                "hand fingertip FK disabled",
                secondary.c_str(), e.what());
    return;
  }

  // Resolve fingertip tip_links + hand-root from the secondary tree-model.
  // The hand-only tree (root="base_adapter" ≡ tool0) yields nq == hand DoF, so
  // the serial FK path fills hand_q_ from the hand device 1:1.
  for (const auto& tm : config.tree_models) {
    if (tm.name != secondary) {
      continue;
    }
    if (!tm.root_link.empty()) {
      hand_root_frame_id_ = hand_handle_->GetFrameId(tm.root_link);
      if (hand_root_frame_id_ != 0) {
        use_hand_root_frame_ = true;
      }
    }
    for (std::size_t i = 0; i < std::min(tm.tip_links.size(), kNumFingertips); ++i) {
      fingertip_frame_ids_[i] = hand_handle_->GetFrameId(tm.tip_links[i]);
      // Runtime check A: confirm the reduced tree retains the tip link frame
      // after loop-passive branches are locked (0 = universe = not found).
      if (fingertip_frame_ids_[i] == 0) {
        RCLCPP_WARN(logger_, "[wbc] fingertip tip_link '%s' unresolved in tree '%s' — skipped",
                    tm.tip_links[i].c_str(), secondary.c_str());
      }
    }
    break;
  }

  hand_q_ = Eigen::VectorXd::Zero(hand_handle_->nq());
  for (auto& p : fingertip_positions_) {
    p = Eigen::Vector3d::Zero();
  }
  for (auto& r : fingertip_rotations_) {
    r = Eigen::Matrix3d::Identity();
  }
}

// ── #123 Phase 2: closed-chain-consistent hand FK wiring (non-RT) ─────────────
// Mirrors DemoTaskController::ConfigureClosedChainHandFk. Wires closed_hand_fk_
// over the full spanning-tree model when the hand has loop closure and a
// fingertip is downstream of a loop-passive joint; else leaves it inactive and
// the serial hand_handle_ path runs byte-for-byte. Called from OnDeviceConfigsSet
// (needs device joint_state_names for the q_a bridge). Publish-surface only —
// does not touch the TSID/actuated control model.
void DemoWbcController::ConfigureClosedChainHandFk() {
  if (!builder_) {
    return;
  }
  const auto primary = GetPrimaryDeviceName();
  const auto secondary = GetSecondaryDeviceName();

  // device_joint_names index order must match ExtractFullState's dev0/dev1:
  // primary=0, secondary=1. The closed handle's independent joints may span both.
  std::vector<std::vector<std::string>> dev_names;
  if (const auto* c = GetDeviceNameConfig(primary)) {
    dev_names.push_back(c->joint_state_names);
  } else {
    dev_names.emplace_back();
  }
  if (!secondary.empty()) {
    if (const auto* c = GetDeviceNameConfig(secondary)) {
      dev_names.push_back(c->joint_state_names);
    } else {
      dev_names.emplace_back();
    }
  }

  // fingertip links + hand-root frame from the secondary tree-model definition.
  std::vector<std::string> tips;
  std::string hand_root;
  if (const auto* sys = GetSystemModelConfig()) {
    for (const auto& tm : sys->tree_models) {
      if (tm.name == secondary) {
        tips = tm.tip_links;
        hand_root = tm.root_link;
        break;
      }
    }
  }

  const auto res =
      closed_hand_fk_.Configure(builder_->GetFullModel(), builder_->GetConstraintModels(),
                                builder_->GetClosureActuatedJointIds(),
                                builder_->GetClosureReferenceConfig(), dev_names, tips, hand_root);
  LogHandFkWiring(logger_, "[wbc]", res, closed_hand_fk_.missing_joint());
}

// ── #120: closed-chain 축약 동역학 provider 배선 (non-RT) ─────────────────────
// control model 이 actuated(closed-chain) 모델일 때만 provider 를 Configure 해
// combined_cache_.cache().reduced_provider 로 주입한다. 이후 매 RT tick 의 cache.Update 가
// open-chain M/h/g 계산 직후 provider 를 호출해 constraint-consistent 축약값으로 덮는다.
// 비-extended (GetActuatedModel()==null → control model 이 tree/full) 이거나 좌표 정렬 미매칭이면
// 미주입 → cache.reduced_provider==nullptr → open-chain 경로 byte-for-byte 유지.
void DemoWbcController::ConfigureReducedDynamicsProvider() {
  combined_cache_.cache().reduced_provider = nullptr;
  if (!builder_ || !combined_cache_.model()) {
    return;
  }
  // 게이트: control model 이 정확히 actuated 모델이어야 한다 (InitModels 의 선택과 동일 인스턴스).
  const auto actuated = builder_->GetActuatedModel();
  if (!actuated || actuated.get() != combined_cache_.model().get()) {
    return;  // tree/full fallback control model → 축약 동역학 비대상
  }

  const bool ok = wbc_reduced_dynamics_.Configure(
      builder_->GetFullModel(), builder_->GetConstraintModels(),
      builder_->GetClosureActuatedJointIds(), builder_->GetClosureReferenceConfig(),
      *combined_cache_.model());
  if (ok) {
    // Phase ③: loop-하류 contact frame 을 loop-consistent J·oMf override 대상으로 판정 (non-RT).
    // cache.Init 이 확정한 contact frame id(control 모델)를 이름→full 모델 fid 로 매핑한다.
    wbc_reduced_dynamics_.ConfigureContactFrames(*combined_cache_.model(),
                                                 rtc::tsid::ContactFrameIds(contact_mgr_config_));
    // 매핑 실패 contact frame 은 loop-하류 판정 불가 → override 에서 빠지고 open-chain(frozen-loop)
    // 값이 조용히 유지된다. config/모델 naming drift 를 잡도록 노출 (silent degradation 방지).
    for (const std::string& fname : wbc_reduced_dynamics_.unmapped_contact_frames()) {
      RCLCPP_WARN(logger_,
                  "[wbc] contact frame '%s' full 모델 매핑 실패 — loop-consistent override 불가, "
                  "open-chain(frozen-loop) 값 유지",
                  fname.c_str());
    }
    combined_cache_.cache().reduced_provider = &wbc_reduced_dynamics_;
    RCLCPP_INFO(logger_, "[wbc] closed-chain 축약 동역학 활성 (n_a=%d) — TSID EOM M/h/g 대체",
                wbc_reduced_dynamics_.n_a());
  } else if (!wbc_reduced_dynamics_.missing_joint().empty()) {
    RCLCPP_WARN(
        logger_,
        "[wbc] closed-chain 축약 동역학 비활성 — 좌표 정렬 미매칭 joint '%s' (open-chain 유지)",
        wbc_reduced_dynamics_.missing_joint().c_str());
  } else {
    RCLCPP_INFO(logger_,
                "[wbc] closed-chain 축약 동역학 비활성 (well-posed closure 아님, open-chain 유지)");
  }
}

// ── TSID task/constraint factories ───────────────────────────────────────────
//
// Dispatch by YAML `type` string. Tasks/constraints are created with
// unique_ptr and transferred to formulation via add_task/add_constraint.
// After construction each is Init()'d with its own sub-node.

void DemoWbcController::BuildTsidTasks(const YAML::Node& tsid_node) {
  if (!combined_cache_.model() || !tsid_node || !tsid_node["tasks"]) {
    return;
  }
  const auto& model = *combined_cache_.model();
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
      task->Init(model, robot_info_, combined_cache_.cache(), task_cfg);
      formulation.AddTask(std::move(task));
      // Optional arm/hand gain split. The scalar/vector kp/kd in task_cfg are
      // consumed by PostureTask::Init above; when `arm`/`hand` sub-maps are
      // present they override per-DoF in ApplyPostureGains (post-reorder).
      ParsePostureSplitGains(task_cfg);
    } else if (type == "se3") {
      auto task = std::make_unique<rtc::tsid::SE3Task>();
      task->Init(model, robot_info_, combined_cache_.cache(), task_cfg);
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
      task->Init(model, robot_info_, combined_cache_.cache(), task_cfg);
      task->SetContactManager(&contact_mgr_config_);
      formulation.AddTask(std::move(task));
    } else if (type == "contact_consistency") {
      // Stage A-2: soft task — residual J_c·a + dotJ_c·v. Activated in
      // closure/hold per phase preset; deactivated in idle/approach.
      auto task = std::make_unique<rtc::tsid::ContactConsistencyTask>();
      task->Init(model, robot_info_, combined_cache_.cache(), task_cfg);
      task->SetContactManager(&contact_mgr_config_);
      formulation.AddTask(std::move(task));
    } else if (type == "object_wrench") {
      // Stage B-5: drives Σ G·λ → w_obj_des (object-origin wrench). Shares
      // the controller-owned ContactManager / GraspCache / ObjectFrame
      // instances so the per-tick cache populated in ComputeTSIDPosition
      // is reused across all three object-level tasks.
      auto task = std::make_unique<rtc::tsid::ObjectWrenchTask>();
      task->Init(model, robot_info_, combined_cache_.cache(), task_cfg);
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
      task->Init(model, robot_info_, combined_cache_.cache(), task_cfg);
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
      task->Init(model, robot_info_, combined_cache_.cache(), task_cfg);
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
  if (!combined_cache_.model() || !tsid_node || !tsid_node["constraints"]) {
    return;
  }
  const auto& model = *combined_cache_.model();
  auto& formulation = tsid_controller_.Formulation();

  for (auto it = tsid_node["constraints"].begin(); it != tsid_node["constraints"].end(); ++it) {
    const auto key = it->first.as<std::string>();
    // yaml-cpp iterator::operator->() returns a proxy holding a prvalue Node;
    // see BuildTsidTasks comment above for the lifetime trap.
    const YAML::Node c_cfg = it->second;
    const auto type = c_cfg["type"].as<std::string>("");

    if (type == "eom") {
      auto c = std::make_unique<rtc::tsid::EomConstraint>();
      c->Init(model, robot_info_, combined_cache_.cache(), c_cfg);
      // Floating-base + surface contact 시 cdim != 3 인 column offset 을 정확히
      // 잡기 위해 필수. point-only 회로에서는 fallback(cdim=3) 으로도 동작하지만,
      // mixed point/surface 가 들어오면 무성 alignment 깨짐.
      c->SetContactManager(&contact_mgr_config_);
      formulation.AddConstraint(std::move(c));
    } else if (type == "contact_constraint") {
      // Stage C-0.3: hard acceleration-level no-slip equality
      // (Jc_i·a + dJc_i·v = 0) per active contact. Off by default in the
      // shipped configs — the soft contact_consistency task carries the
      // no-slip objective there. Opt-in by adding a `contact_constraint`
      // entry under tsid.constraints. SetContactManager mirrors the eom
      // branch so the active-contact column offsets (cdim 3/6) align.
      auto c = std::make_unique<rtc::tsid::ContactConstraint>();
      c->Init(model, robot_info_, combined_cache_.cache(), c_cfg);
      c->SetContactManager(&contact_mgr_config_);
      formulation.AddConstraint(std::move(c));
    } else if (type == "joint_limit") {
      auto c = std::make_unique<rtc::tsid::JointLimitConstraint>();
      c->Init(model, robot_info_, combined_cache_.cache(), c_cfg);
      formulation.AddConstraint(std::move(c));
    } else if (type == "friction_cone") {
      // Stage C-0.1: FrictionConeConstraint takes its face count from each
      // contact (contacts[*].n_faces / friction_faces), not from this block.
      // A `n_faces` here is a dead key — WARN so the misconception that caused
      // the silent friction_faces=4 fallback does not recur.
      if (c_cfg["n_faces"]) {
        RCLCPP_WARN(logger_,
                    "[wbc] friction_cone.n_faces is ignored — set the face count per "
                    "contact via contacts[*].n_faces (or friction_faces)");
      }
      auto c = std::make_unique<rtc::tsid::FrictionConeConstraint>();
      c->Init(model, robot_info_, combined_cache_.cache(), c_cfg);
      c->SetContactManager(&contact_mgr_config_);
      formulation.AddConstraint(std::move(c));
    } else if (type == "torque_limit") {
      auto c = std::make_unique<rtc::tsid::TorqueLimitConstraint>();
      c->Init(model, robot_info_, combined_cache_.cache(), c_cfg);
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

void DemoWbcController::BuildClikJointIndexSets(int arm_dof, int full_dof, int nv,
                                                const std::array<int, kMaxFullDof>& ext_to_pin_v,
                                                std::vector<int>& arm_v_idx,
                                                std::vector<int>& hand_v_idx) noexcept {
  arm_v_idx.clear();
  hand_v_idx.clear();
  const int n = std::min(full_dof, static_cast<int>(kMaxFullDof));
  for (int i = 0; i < n; ++i) {
    const int pv = ext_to_pin_v[static_cast<std::size_t>(i)];
    if (pv < 0 || pv >= nv) {
      continue;
    }
    if (i < arm_dof) {
      arm_v_idx.push_back(pv);
    } else {
      hand_v_idx.push_back(pv);
    }
  }
}

void DemoWbcController::InitClik() noexcept {
  clik_enabled_ = false;
  clik_tcp_frame_idx_ = -1;
  clik_base_frame_idx_ = -1;
  if (!tsid_initialized_ || !combined_cache_.reorder_valid() || !combined_cache_.model() ||
      arm_dof_ <= 0) {
    return;
  }
  const int nq = combined_cache_.model()->nq;
  const int nv = combined_cache_.model()->nv;
  // CLIK contract: nq == nv (reduced revolute/prismatic tree) so velocity
  // indices address q directly and q_ref = q + v·dt is valid. The full URDF
  // model (nq=26, nv=21 with first-class mimic) violates this. CLIK is now the
  // SOLE position backbone (the integrator was removed), so a non-reduced model
  // has no backbone — leave CLIK disabled and let on_configure FAIL the
  // lifecycle transition (DEC-1 ⓐ: reject the config rather than degrade).
  if (nq != nv) {
    RCLCPP_ERROR(logger_,
                 "[wbc] CLIK disabled: control model nq=%d != nv=%d (needs reduced nq==nv tree). "
                 "CLIK is the required position backbone — configure will fail.",
                 nq, nv);
    return;
  }

  // Register the se3_tcp tip/base frames on the shared cache. RegisterFrame
  // dedups by frame_id, so this reuses the SE3Task's existing registration
  // (returning the same index). Must run before the first RT cache.Update()
  // locks registration — InitClik is called from on_configure.
  if (tip_frame_id_ == 0) {
    RCLCPP_WARN(logger_, "[wbc] CLIK disabled: tip frame unresolved (OnDeviceConfigsSet).");
    return;
  }
  clik_tcp_frame_idx_ = combined_cache_.cache().RegisterFrame("clik_tcp", tip_frame_id_);
  if (clik_tcp_frame_idx_ < 0) {
    RCLCPP_WARN(logger_, "[wbc] CLIK disabled: tip frame registration failed (cache locked).");
    return;
  }
  // Base frame: mirror SE3Task's universe fast-path (base_frame_idx < 0) when
  // the controller uses the world frame; otherwise register the root link.
  if (use_root_frame_ && root_frame_id_ != 0) {
    clik_base_frame_idx_ = combined_cache_.cache().RegisterFrame("clik_base", root_frame_id_);
    if (clik_base_frame_idx_ < 0) {
      RCLCPP_WARN(logger_, "[wbc] CLIK disabled: base frame registration failed (cache locked).");
      clik_tcp_frame_idx_ = -1;
      return;
    }
  } else {
    clik_base_frame_idx_ = -1;  // universe
  }

  std::vector<int> arm_v_idx;
  std::vector<int> hand_v_idx;
  BuildClikJointIndexSets(arm_dof_, full_dof_, nv, combined_cache_.ext_to_pin_v_map(), arm_v_idx,
                          hand_v_idx);

  rtc::tsid::ClikReferenceGenerator::Config cfg;
  cfg.arm_v_idx = std::move(arm_v_idx);
  cfg.hand_v_idx = std::move(hand_v_idx);
  cfg.damping_sq = clik_damping_sq_;
  cfg.v_limit = clik_v_limit_;
  cfg.w_task = clik_w_task_;
  cfg.w_arm = clik_w_arm_;
  cfg.w_hand = clik_w_hand_;
  cfg.anchor_drift_max = clik_anchor_drift_max_;
  // Position-aware velocity box. Reuse the margin-clamped limits the integrator
  // already applies (LoadConfig step 3, computed before InitClik) so both
  // Kinematic-WBC paths respect the identical joint envelope. Sized [nv]; the
  // nq==nv guard above guarantees a match with the CLIK box.
  if (q_min_clamped_.size() == nv && q_max_clamped_.size() == nv) {
    cfg.q_min = q_min_clamped_;
    cfg.q_max = q_max_clamped_;
  }
  try {
    clik_.Init(nv, cfg);
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "[wbc] CLIK disabled: Init failed: %s", e.what());
    clik_tcp_frame_idx_ = -1;
    clik_base_frame_idx_ = -1;
    return;
  }
  clik_enabled_ = true;
  RCLCPP_INFO(logger_,
              "[wbc] CLIK reference enabled: arm_v=%d hand_v=%d damping_sq=%.1e v_limit=%.2f "
              "(tip_frame_idx=%d base_frame_idx=%d)",
              static_cast<int>(cfg.arm_v_idx.size()), static_cast<int>(cfg.hand_v_idx.size()),
              clik_damping_sq_, clik_v_limit_, clik_tcp_frame_idx_, clik_base_frame_idx_);
}

void DemoWbcController::ApplyPostureGains() noexcept {
  if (!posture_split_gains_ || !tsid_initialized_ || !combined_cache_.reorder_valid()) {
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
  AssemblePostureGains(arm_dof_, full_dof_, nv, combined_cache_.ext_to_pin_v_map(), posture_kp_arm_,
                       posture_kd_arm_, posture_kp_hand_, posture_kd_hand_, kp, kd);
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

  if (!combined_cache_.model()) {
    RCLCPP_ERROR(logger_, "Full model not available — TSID disabled");
    return;
  }

  // ── 2. TSID initialization ────────────────────────────────────────────
  const auto tsid_node = cfg["tsid"];
  if (tsid_node) {
    const auto& model = *combined_cache_.model();

    // RobotModelInfo
    robot_info_.Build(model, tsid_node);

    // ContactManagerConfig — pass the whole tsid subtree; load() looks up
    // "contacts" itself (same pattern as rtc_tsid/test_force_task.cpp).
    // Passing tsid_node["contacts"] would double-index and silently yield
    // max_contacts = 0 (bug observed as "TSID initialized: contacts=0"
    // in the sim launch prior to this fix).
    if (tsid_node["contacts"]) {
      contact_mgr_config_.Load(tsid_node, model);
      // Stage C-0.1: the loader has no logger, so surface the resolved friction
      // params here (one-shot init — RT-3 logging exception). Guards the
      // friction_coeff/mu + friction_faces/n_faces silent-fallback bug.
      if (contact_mgr_config_.friction_key_conflict) {
        RCLCPP_WARN(logger_,
                    "[wbc] friction key conflict: a contact sets both friction_coeff/mu "
                    "(or friction_faces/n_faces) with different values — the explicit "
                    "friction_coeff/friction_faces wins");
      }
      int max_cone_rows = 0;
      for (const auto& cc : contact_mgr_config_.contacts) {
        // Per FrictionConeConstraint: point = n_faces+1, surface = n_faces+7.
        max_cone_rows += cc.friction_faces + (cc.contact_dim == 6 ? 7 : 1);
        RCLCPP_INFO(logger_, "[wbc] contact '%s': mu=%.3f n_faces=%d (%s)", cc.name.c_str(),
                    cc.friction_coeff, cc.friction_faces,
                    cc.contact_dim == 6 ? "surface" : "point");
      }
      RCLCPP_INFO(logger_, "[wbc] friction cone: %d contacts, max %d inequality rows",
                  contact_mgr_config_.max_contacts, max_cone_rows);
    }

    // PinocchioCache
    // Deferred cache Init (see InitModels): the model was selected early, but
    // the contact frame ids are only now parsed, so Init the shared cache here.
    // InitCacheDeferred (not raw cache().Init) so the helper's cache-ready gate
    // on Update()/ArmTcpPoseFromCache() stays self-enforcing.
    combined_cache_.InitCacheDeferred(rtc::tsid::ContactFrameIds(contact_mgr_config_));

    // #120: closed-chain 축약 동역학 provider 배선 (extended 로봇에서 M/h/g 대체). 비-extended
    // 는 게이트 미충족 → open-chain byte-for-byte. cache.Init 직후 (model_ptr 확정) 배선.
    ConfigureReducedDynamicsProvider();

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

  // ── 3. Reference buffers ───────────────────────────────────────────────
  // (Measured q/v buffers are owned + sized by combined_cache_.SelectModel.)
  const int nv = combined_cache_.model()->nv;
  // Same layout as combined_cache_.q() so it is a drop-in replacement for the
  // posture reference (control_ref_.q_des = combined_cache_.q()) on the RT path.
  q_des_target_full_ = Eigen::VectorXd::Zero(nv);

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
  q_min_clamped_ = combined_cache_.model()->lowerPositionLimit.array() + position_margin_;
  q_max_clamped_ = combined_cache_.model()->upperPositionLimit.array() - position_margin_;
  v_limit_ = combined_cache_.model()->upperVelocityLimit * velocity_scale_;

  // ── 3b. Stage C-2: CLIK (Kinematic WBC) configuration ─────────────────
  // CLIK is the sole position backbone. clik.{damping_sq,v_limit,w_*} are
  // structural (consumed by ClikReferenceGenerator::Init in on_configure);
  // clik.{kx_pos,kx_rot,ka,kh} are runtime gains (SeqLock + ROS params).
  {
    auto g = gains_lock_.Load();
    if (cfg["clik"] && cfg["clik"].IsMap()) {
      const auto clik = cfg["clik"];
      clik_damping_sq_ = clik["damping_sq"].as<double>(clik_damping_sq_);
      clik_v_limit_ = clik["v_limit"].as<double>(clik_v_limit_);
      // Kinematic CLIK-QP soft-priority weights (w_task ≫ w_arm,w_hand ≫ μ²).
      clik_w_task_ = clik["w_task"].as<double>(clik_w_task_);
      clik_w_arm_ = clik["w_arm"].as<double>(clik_w_arm_);
      clik_w_hand_ = clik["w_hand"].as<double>(clik_w_hand_);
      clik_anchor_drift_max_ = clik["anchor_drift_max"].as<double>(clik_anchor_drift_max_);
      g.clik_kx_pos = clik["kx_pos"].as<double>(g.clik_kx_pos);
      g.clik_kx_rot = clik["kx_rot"].as<double>(g.clik_kx_rot);
      g.clik_ka = clik["ka"].as<double>(g.clik_ka);
      g.clik_kh = clik["kh"].as<double>(g.clik_kh);
    }
    gains_lock_.Store(g);
  }

  // ── 4. FSM thresholds ─────────────────────────────────────────────────
  if (cfg["fsm"]) {
    const auto fsm = cfg["fsm"];
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

    // ── #167 P3: pull estimator + tip-link → contact slot resolve ─────────
    // Slot order = tsid.contacts order (frame names), which maps 1:1 onto the
    // fingertip sensor lanes (Stage A-3 contract) and onto
    // combined_cache_.cache().contact_frames. Empty contacts (no tsid block, e.g.
    // unit fixtures) leaves the estimator disabled; a configured tip link
    // matching no contact frame throws → configure FAILURE.
    {
      // Cap at the fingertip sensor-lane capacity: a slot must index both
      // contact_frames AND fingertip_data_ (1:1 mapping).
      const std::size_t n_slots =
          std::min(contact_mgr_config_.contacts.size(), fingertip_data_.size());
      std::vector<std::string> contact_links;
      contact_links.reserve(n_slots);
      for (std::size_t i = 0; i < n_slots; ++i) {
        contact_links.push_back(contact_mgr_config_.contacts[i].frame_name);
      }
      ConfigurePullEstimatorWiring(shared, 1.0 / GetDefaultDt(), contact_links, pull_wiring_);
      LogPullEstimatorWiring(logger_, pull_wiring_, shared);
    }
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
    if (s == "torque") {
      command_type_ = CommandType::kTorque;
    } else if (s == "pd_feedforward") {
      command_type_ = CommandType::kPdFeedforward;
    } else {
      command_type_ = CommandType::kPosition;
    }
  }

  // ── 7. MPC integration ────────────────────────────────────────────────
  ConfigureMpc(cfg);

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
          e.msg_type != "integrated_bringup/WbcDiagLog" &&
          e.msg_type != integrated_bringup::kPullEstimatorLogMsgType) {
        throw std::runtime_error("DemoWbcController: unknown msg_type in `logs`: " + e.msg_type);
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }
}

// LoadConfig section 7 — see header. Parses `mpc:` and pre-builds handler-mode
// preconditions. mpc_engine_ may be downgraded to kMock on any precondition
// failure; mpc_enabled_/TSID self-hold semantics preserved bit-exactly when
// `mpc:` is absent or disabled.
void DemoWbcController::ConfigureMpc(const YAML::Node& cfg) {
  // If `mpc.enabled: true`, size the reference buffers and initialise the
  // MPC solution manager. The thread itself is spawned in
  // SpawnMpcThreadIfNeeded (called from on_activate on the aux thread) so the
  // heap-allocating MPCFactory::Create stays off the RT path.
  //
  // `mpc.engine` (default "mock") selects MockMPCThread vs HandlerMPCThread.
  // Handler mode additionally loads mpc/phase_config.yaml + mpc/contact_light.yaml
  // + mpc/contact_rich.yaml from the package share and pre-builds the
  // RobotModelHandler + GraspPhaseManager for startup.
  if (const auto mpc_cfg = cfg["mpc"]; mpc_cfg && combined_cache_.model()) {
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

    // TSID and MPC share combined_cache_.model() (the reduced tree when available).
    // No projection layer needed — ComputeReference writes directly into the
    // reference buffers that TSID consumes via control_ref_.
    const int mpc_nq = combined_cache_.model()->nq;
    const int mpc_nv = combined_cache_.model()->nv;
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
                : std::string{"config/ur5e_p1a/controllers/mpc/phase_config.yaml"};
        const auto light_path =
            mpc_cfg["contact_light_path"]
                ? mpc_cfg["contact_light_path"].as<std::string>()
                : std::string{"config/ur5e_p1a/controllers/mpc/contact_light.yaml"};
        const auto rich_path =
            mpc_cfg["contact_rich_path"]
                ? mpc_cfg["contact_rich_path"].as<std::string>()
                : std::string{"config/ur5e_p1a/controllers/mpc/contact_rich.yaml"};

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
        const auto model_err = mpc_model_handler_->Init(*combined_cache_.model(), model_node);
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
                             -kJointLimitFallbackRad, kJointLimitFallbackRad, 2.0);

  // ── Joint reorder map (delegated to the shared cache, #174) ────────────
  {
    const auto* arm_cfg = GetDeviceNameConfig(GetPrimaryDeviceName());
    const auto* hand_cfg = GetDeviceNameConfig(GetSecondaryDeviceName());
    combined_cache_.BuildReorderMap(arm_cfg ? &arm_cfg->joint_state_names : nullptr,
                                    hand_cfg ? &hand_cfg->joint_state_names : nullptr, full_dof_,
                                    "[wbc]", logger_);
  }

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
              static_cast<int>(has_native_contact_), static_cast<int>(has_native_displacement_),
              secondary_name.c_str());

  // #123 Phase 2: wire the closed-chain-consistent hand FK (publish-surface only).
  ConfigureClosedChainHandFk();

  // Serial hand joint reorder (device joint_state_names → Pinocchio order) is
  // only consulted when the closed path is inactive — RunHandForwardKinematics
  // routes an active closure straight to closed_hand_fk_ without touching
  // hand_handle_'s serial FK. So skip (and its warning) when closure is active:
  // for a loop-closed hand the reduced serial tree deliberately omits the
  // loop-locked DoFs, so the device names wouldn't all map anyway.
  if (hand_handle_ && !secondary_name.empty() && !closed_hand_fk_.active()) {
    if (const auto* hand_cfg = GetDeviceNameConfig(secondary_name)) {
      if (!hand_handle_->SetJointOrder(hand_cfg->joint_state_names)) {
        RCLCPP_WARN(logger_,
                    "[wbc] secondary device '%s' SetJointOrder failed — joint_state_names "
                    "not all in hand tree model",
                    secondary_name.c_str());
      }
    }
  }
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
  // Layer 2 span: this controller's entire Compute() (the concrete work the
  // CM dispatches). Nests under "CM::Compute"; the FSM-dispatched sub-steps
  // (ComputeControl → ComputeWbcCommon/ComputeKinematicWbc/...) nest below.
  RTC_TRACE_SCOPE("DemoWbcController::Compute");
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();

  // #1 (safety): single per-tick owner of the hand τ_ff active flag. Reset here
  // — before any early-return (E-STOP / !target_initialized) and before the FSM
  // dispatch — so every non-TSID path (fallback / position / release hold) sees
  // false. ComputeTSIDPosition is the only writer that sets it true (this tick's
  // QP converged). Without this, a stale true from a prior TSID tick would make
  // WriteJointCommand replay last tick's feedforward on a kFallback (QP-divergence)
  // tick — the exact path that must be a conservative position hold.
  hand_tauff_active_ = false;

  // F5 gate (#236 S7b). Loaded before ReadState — earlier than estop_active_
  // below, because ReadState's model scatter is already a consumer — so the
  // scatter, the FSM dispatch, WriteJointCommand and ComputeEstop all answer
  // "is device 0 usable this tick?" the same way.
  arm_readable_ = rtc::IsDeviceReadable(state.devices[0], arm_dof_);

  ReadState(state);
  DrainTargetSlot(state);

  // E-STOP takes priority over FSM
  estop_active_ = estopped_.load(std::memory_order_acquire);
  if (estop_active_) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    // E-8 (resolved): do NOT push the WBC state / diag CSV channels on the
    // E-STOP path — tsid_output_ is stale there. Sensor logging continues (raw
    // HW telemetry is meaningful during E-STOP).
    if (secondary_sensor_log_handle_) {
      integrated_bringup::DeviceSensorLogPod pod{};
      FillDeviceSensorLogPod(state, /*device_idx=*/1, num_active_fingertips_, pod);
      secondary_sensor_log_handle_.Push(pod);
    }
    // #234 P-1: the same rule applied to the *wire* required a store, not a
    // skip — the publish thread re-loads the SeqLock under a fresh stamp every
    // tick, so "don't store" published the pre-E-STOP body as if it were
    // current. FillEstopPublishState stores a body with the TSID-derived
    // fields reported as not-solved and the pull estimate marked invalid.
    FillEstopPublishState(dt);
    PushPullEstimatorLog(pull_estimator_log_handle_, pull_wiring_, state.t_relative_s,
                         state.iteration);
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

  // Mid-phase commanded SE3 jog: a new SE3 target arrived without a phase-entry
  // edge (so OnPhaseEnter's seed did not run this tick). Re-point tcp_goal_ at
  // the commanded pose; when the SE3 task is active in this phase (MPC-disabled),
  // rebuild the TCP ramp from the current FK so the move is smoothed. The
  // commanded SE3 is a LIVE arm target in every phase except the transient
  // finger-open (kRelease) and the safety hold (kFallback), which own the arm
  // goal. Idle (se3 task YAML-deactivated) tracks the goal step directly through
  // CLIK, which rate-limits it via its own v_limit.
  if (arm_task_new_target_pending_) {
    const bool target_live_phase = (phase_ != WbcPhase::kRelease && phase_ != WbcPhase::kFallback);
    // Apply the commanded SE3 to tcp_goal_ unconditionally (URDF-independent
    // copy) so CLIK/SE3Task readers see the new goal. Only the trajectory ramp
    // re-init needs arm_handle_ FK; skip it when there is no model (unit tests).
    if (target_live_phase && ApplyCommandedSe3IfPresent()) {
      target_seqlock_.Store(current_target_slot_);
      const bool mpc_on = mpc_enabled_ && mpc_manager_.Enabled();
      // arm_readable_ joins the guard (#265 audit W3): FK on a device narrower
      // than arm_dof_ runs at the ZERO configuration, and its result would
      // become the trajectory START pose.
      if (arm_readable_ && arm_handle_ && !mpc_on && Se3TaskActiveInPhase(phase_)) {
        std::span<const double> q_arm(state.devices[0].positions.data(),
                                      static_cast<std::size_t>(arm_dof_));
        arm_handle_->ComputeForwardKinematics(q_arm);  // start = current FK
        InitTcpTrajectory(state);
      }
    }
    arm_task_new_target_pending_ = false;
  }

  // Mid-phase hand joint jog: the hand joint target is a LIVE command in every
  // phase except kRelease (its finger-open hand_trajectory_ ramp owns the hand)
  // and kFallback (safety hold). Fold the fresh target into the posture
  // reference's hand block so the CLIK posture term drives the hand toward it
  // THIS tick — instead of the target only being consumed on a closure
  // phase-entry edge (BuildTargetPosture). DrainTargetSlot already stored the
  // new values in current_target_slot_.targets[1].
  if (hand_new_target_pending_ && phase_ != WbcPhase::kRelease && phase_ != WbcPhase::kFallback) {
    // Clear the pending flag only when the fold actually applied (hand device
    // valid + reorder map ready). Otherwise keep it pending so a target that
    // arrives during a transient hand-device dropout is not silently lost — it
    // re-folds on the next tick once the device is valid again.
    if (BuildHandTargetPosture(state)) {
      hand_new_target_pending_ = false;
    }
  }

  // #167: the pull estimator only trusts contact geometry produced on a
  // TSID-routing tick. Reset here; ComputeWbcCommon sets it true. NOTE (Task B):
  // this flag no longer means "oMf is fresh" — ComputeControl's Stage-1
  // cache.Update runs every tick, so oMf/contact_frames are fresh on kFallback and
  // every non-TSID tick too. The flag PURELY gates the pull estimate to TSID ticks.
  contact_geometry_fresh_ = false;

  // ── FSM phase transition (option C): hoisted OUT of ComputeControl to Compute
  // scope so ComputeControl is a clean compute-model → control-law body. UpdatePhase
  // is FSM-transition logic (DrainTargetSlot-natured), and its ComputeTcpError reads
  // the PREVIOUS tick's oMf (one-tick lag contract). Running it here — before
  // ComputeControl's Stage-1 cache.Update refreshes oMf this tick — preserves that
  // lag exactly → kApproach/kClosure transition timing byte-for-byte.
  UpdatePhase(state);

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
  PushPullEstimatorLog(pull_estimator_log_handle_, pull_wiring_, state.t_relative_s,
                       state.iteration);
  return output;
}

// ── Phase 1: Read state ──────────────────────────────────────────────────────

void DemoWbcController::SetDeviceTarget(int device_idx, std::span<const double> target) noexcept {
  // Off-RT marshal. The base stamps the activation generation, bounds the
  // index and the width, and queues; the RT thread drains inside Compute() and
  // is the SOLE writer of target_seqlock_.
  PushPendingTarget(device_idx, target, /*is_task=*/false);
}

void DemoWbcController::SetDeviceTaskTarget(int device_idx,
                                            std::span<const double> task6) noexcept {
  // Off-RT marshal — same mailbox as joint targets, tagged so the RT side
  // converts it to a commanded SE3 instead of overwriting the joint slot.
  PushPendingTarget(device_idx, task6, /*is_task=*/true);
}

// RT-thread-only. Refreshes current_target_slot_, drains pending entries,
// runs first-tick self-init (seeded from current device state).
void DemoWbcController::DrainTargetSlot(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoWbcController::DrainTargetSlot");
  current_target_slot_ = target_seqlock_.Load();
  bool slot_dirty = false;

  if (!target_initialized_.load(std::memory_order_acquire)) {
    // First-tick self-init must capture a VALID measured configuration for
    // EVERY configured device (arm AND hand). If device 0 (arm) has not
    // published a real state yet (the simulator is not yet streaming on the
    // first control tick), seeding now would lock the idle hold target (posture
    // ref + SE3 hold pose) to q=0; idle then regulates toward the zero config
    // and drives the arm hard toward it, saturating the joint/torque-limit
    // constraints into an infeasible QP (ProxQP grind 0.5–1.4 s → fallback
    // cycling). The hand (device 1) has the SAME failure mode: the inline hand
    // loop below guards on dev.valid, so a hand that is still coming up would
    // leave the posture reference's hand block (q_des_target_full_) at zero and
    // drive every finger to 0 (ur5e fine / p1b collapse) — while
    // target_initialized_ latches true and never re-seeds. Defer the WHOLE init
    // until both are valid: leave target_initialized_ false and command a
    // passthrough hold so the next tick re-attempts the seed from a real
    // measured configuration once every device streams.
    // `valid` alone was the wrong test (#265 audit W1): a device that has
    // reported only part of the arm passes it, and the passthrough hold below
    // then reads dev0.positions arm_dof_ deep — the unreported joints coming
    // back as 0. IsDeviceReadable is the same test plus the channel axis.
    const bool arm_ready = state.num_devices > 0 && arm_readable_;
    const bool hand_ready = state.num_devices <= 1 || state.devices[1].valid;
    if (!arm_ready || !hand_ready) {
      const auto& dev0 = state.devices[0];
      // Arm passthrough only when the arm itself is readable — this branch is
      // also taken when only the HAND is missing, and then the arm hold is a
      // real measurement worth passing through.
      if (arm_readable_) {
        for (int i = 0; i < arm_dof_; ++i) {
          const auto idx = static_cast<std::size_t>(i);
          robot_computed_.positions[idx] = dev0.positions[idx];
          robot_computed_.velocities[idx] = 0.0;
        }
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

    // Fallback DoF when LoadConfig/OnDeviceConfigsSet hasn't populated runtime
    // dimensions (e.g. unit tests that bypass YAML). Resolved AFTER the defer
    // guard so hand_dof_/full_dof_ are computed only once the hand device is
    // valid — otherwise full_dof_ would latch to arm_dof_ and never grow.
    if (arm_dof_ == 0 && state.num_devices > 0) {
      arm_dof_ = std::min(state.devices[0].num_channels, kMaxArmDof);
    }
    if (hand_dof_ == 0 && state.num_devices > 1 && state.devices[1].valid) {
      hand_dof_ = std::min(state.devices[1].num_channels, kMaxHandDof);
    }
    if (full_dof_ == 0) {
      full_dof_ = arm_dof_ + hand_dof_;
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
    // Drop any stale commanded SE3 on (re)enable / E-STOP-clear self-init so the
    // first idle hold regulates to the measured pose, not a pre-estop command.
    current_target_slot_.tcp_cmd_valid = false;
    arm_task_new_target_pending_ = false;
    dyn_qp_fail_count_ = 0;
    kin_qp_fail_count_ = 0;
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
    // First-tick init: re-anchor the CLIK desired to the measured state.
    clik_reseed_pending_ = true;
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

  // Base drain: pops everything queued, drops what a later activation
  // invalidated (#196 §3), and calls ApplyPendingTarget for each survivor.
  // The dirty flag comes from Apply rather than the entry count because a
  // surviving entry can still reach no slot — see drained_slot_dirty_.
  drained_slot_dirty_ = false;
  (void)DrainPendingTargets();
  if (drained_slot_dirty_) {
    slot_dirty = true;
  }

  if (slot_dirty) {
    target_seqlock_.Store(current_target_slot_);
  }
}

// RT tick, called once per surviving mailbox entry from DrainTargetSlot().
// Writes the RT working copy, which DrainTargetSlot publishes once at the end.
void DemoWbcController::ApplyPendingTarget(int device_idx, std::span<const double> values,
                                           bool is_task) noexcept {
  const auto didx = static_cast<std::size_t>(device_idx);
  if (is_task) {
    // Commanded SE3 — arm (device 0) only; the hand has no SE3 task slot, so a
    // task goal on device 1+ is ignored. Convert (x,y,z,r,p,y)→SE3 with the ZYX
    // (yaw·pitch·roll) convention shared with DemoTask through
    // rtc::math::se3::RpyToRotationZyx, so the convention has one owner rather
    // than a copy per binding. trig only, no alloc (RT-1/RT-4); the joint slot
    // (targets[0]) is left untouched so arm joint posture and the commanded SE3
    // stay independent.
    if (device_idx == 0 && values.size() >= 6) {
      const Eigen::Matrix3d rotation =
          rtc::math::se3::RpyToRotationZyx(Eigen::Vector3d(values[3], values[4], values[5]));
      const Eigen::Vector3d translation(values[0], values[1], values[2]);
      std::memcpy(current_target_slot_.tcp_cmd_rot.data(), rotation.data(),
                  sizeof(current_target_slot_.tcp_cmd_rot));
      std::memcpy(current_target_slot_.tcp_cmd_t.data(), translation.data(),
                  sizeof(current_target_slot_.tcp_cmd_t));
      current_target_slot_.tcp_cmd_valid = true;
      arm_task_new_target_pending_ = true;
      drained_slot_dirty_ = true;
    }
    return;
  }

  for (std::size_t i = 0; i < values.size() && i < kMaxDeviceChannels; ++i) {
    current_target_slot_.targets[didx][i] = values[i];
  }
  if (device_idx == 0) {
    robot_new_target_pending_ = true;
  } else if (device_idx == 1) {
    hand_new_target_pending_ = true;
  }
  // q_des_target_full_ is rebuilt at phase entry so the posture + SE3
  // references stay a consistent snapshot. A new target is a goal edge: re-anchor
  // the CLIK desired to the measured state (otherwise it carries forward).
  clik_reseed_pending_ = true;
  drained_slot_dirty_ = true;
}

bool DemoWbcController::ApplyCommandedSe3IfPresent() noexcept {
  if (!current_target_slot_.tcp_cmd_valid) {
    return false;
  }
  // Commanded SE3 takes priority over the measured/joint-target FK seed.
  std::memcpy(tcp_goal_.rotation().data(), current_target_slot_.tcp_cmd_rot.data(),
              sizeof(current_target_slot_.tcp_cmd_rot));
  std::memcpy(tcp_goal_.translation().data(), current_target_slot_.tcp_cmd_t.data(),
              sizeof(current_target_slot_.tcp_cmd_t));
  tcp_goal_valid_ = true;
  // Mirror into the FK-seed POD so the per-tick DrainTargetSlot restore keeps
  // tcp_goal_ at the commanded pose on subsequent ticks (single-writer: RT).
  std::memcpy(current_target_slot_.tcp_goal_rot.data(), tcp_goal_.rotation().data(),
              sizeof(current_target_slot_.tcp_goal_rot));
  std::memcpy(current_target_slot_.tcp_goal_t.data(), tcp_goal_.translation().data(),
              sizeof(current_target_slot_.tcp_goal_t));
  current_target_slot_.tcp_goal_valid = true;
  return true;
}

// Aux-thread spawn of MPC thread. Called from on_activate (heap-allocating;
// must not run on the RT path). Idempotent — re-activation reuses the
// existing thread.
void DemoWbcController::SpawnMpcThreadIfNeeded() noexcept {
  if (mpc_enabled_ && !mpc_thread_) {
    const auto thread_configs = rtc::SelectThreadConfigs();
    const int nq = static_cast<int>(combined_cache_.q().size());
    const int nv = static_cast<int>(combined_cache_.v().size());

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
      Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(combined_cache_.model()->nq);
      Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(combined_cache_.model()->nv);
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
      mock->SetTarget(combined_cache_.q());
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
