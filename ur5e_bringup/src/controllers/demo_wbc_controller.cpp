#include "ur5e_bringup/controllers/demo_wbc_controller.hpp"

#include "rtc_base/threading/thread_utils.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/tasks/force_task.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"
#include "rtc_tsid/tasks/se3_task.hpp"

#include "rtc_tsid/constraints/eom_constraint.hpp"
#include "rtc_tsid/constraints/friction_cone_constraint.hpp"
#include "rtc_tsid/constraints/joint_limit_constraint.hpp"

namespace ur5e_bringup {

// ── Constructor ──────────────────────────────────────────────────────────────

DemoWbcController::DemoWbcController(std::string_view urdf_path)
    : urdf_path_(urdf_path) {
  // Model is built in LoadConfig() using system model config.
}

// ── Model initialization ─────────────────────────────────────────────────────

void DemoWbcController::InitModels(const rtc_urdf_bridge::ModelConfig &config) {
  namespace rub = rtc_urdf_bridge;

  builder_ = std::make_unique<rub::PinocchioModelBuilder>(config);

  // Arm sub-model (6-DoF) for FK / task-space logging
  const auto primary = GetPrimaryDeviceName();
  std::string arm_model_name = "arm";
  for (const auto &sm : config.sub_models) {
    if (sm.name == primary) {
      arm_model_name = primary;
      break;
    }
  }
  arm_handle_ = std::make_unique<rub::RtModelHandle>(
      builder_->GetReducedModel(arm_model_name));

  // Control model. Prefer the mimic-locked reduced tree `mpc` built by
  // PinocchioModelBuilder (IncorporateMimicAsPassive → buildReducedModel),
  // which yields nq == nv == 16 for UR5e + 10-DoF hand. TSID/MPC/state
  // buffers all operate in this reduced space, so mimic DoFs are handled
  // by the hardware driver / MuJoCo's built-in tendon. If the tree isn't
  // configured in YAML, fall back to the raw URDF-parsed full model (nq=26,
  // nv=21 with Pinocchio first-class mimic) — this preserves pre-reduction
  // behaviour for URDFs without <mimic> tags.
  try {
    full_model_ptr_ = builder_->GetTreeModel("mpc");
    RCLCPP_INFO(logger_,
                "[wbc] control model: reduced tree 'mpc' (nq=%d nv=%d)",
                full_model_ptr_->nq, full_model_ptr_->nv);
  } catch (const std::exception &e) {
    full_model_ptr_ = builder_->GetFullModel();
    RCLCPP_INFO(
        logger_,
        "[wbc] control model: URDF full model (nq=%d nv=%d) — tree 'mpc' "
        "missing (%s); MPC handler-mode + TSID will see mimic joints",
        full_model_ptr_->nq, full_model_ptr_->nv, e.what());
  }

  RCLCPP_INFO(logger_, "Models initialized: arm nv=%d, control nq=%d nv=%d",
              arm_handle_->nv(), full_model_ptr_->nq, full_model_ptr_->nv);
}

void DemoWbcController::BuildJointReorderMap() {
  if (!full_model_ptr_) {
    return;
  }
  const auto &model = *full_model_ptr_;

  const auto *arm_cfg = GetDeviceNameConfig("ur5e");
  const auto *hand_cfg = GetDeviceNameConfig("hand");
  if (!arm_cfg || !hand_cfg) {
    RCLCPP_WARN(logger_,
                "Device configs not available, using identity mapping");
    for (int i = 0; i < kFullDof; ++i) {
      ext_to_pin_q_[static_cast<std::size_t>(i)] = i;
      ext_to_pin_v_[static_cast<std::size_t>(i)] = i;
    }
    joint_reorder_valid_ = true;
    return;
  }

  int ext_idx = 0;

  // Arm joints
  for (const auto &jname : arm_cfg->joint_state_names) {
    if (!model.existJointName(jname)) {
      RCLCPP_ERROR(logger_, "Joint '%s' not found in full model",
                   jname.c_str());
      continue;
    }
    const auto jid = model.getJointId(jname);
    const auto eidx = static_cast<std::size_t>(ext_idx);
    ext_to_pin_q_[eidx] = model.idx_qs[jid];
    ext_to_pin_v_[eidx] = model.idx_vs[jid];
    ++ext_idx;
  }

  // Hand joints
  for (const auto &jname : hand_cfg->joint_state_names) {
    if (!model.existJointName(jname)) {
      RCLCPP_ERROR(logger_, "Joint '%s' not found in full model",
                   jname.c_str());
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
    RCLCPP_ERROR(logger_, "Joint reorder incomplete: mapped %d/%d joints",
                 ext_idx, kFullDof);
  }
}

// ── TSID task/constraint factories ───────────────────────────────────────────
//
// Dispatch by YAML `type` string. Tasks/constraints are created with
// unique_ptr and transferred to formulation via add_task/add_constraint.
// After construction each is init()'d with its own sub-node.

void DemoWbcController::BuildTsidTasks(const YAML::Node &tsid_node) {
  if (!full_model_ptr_ || !tsid_node || !tsid_node["tasks"]) {
    return;
  }
  const auto &model = *full_model_ptr_;
  auto &formulation = tsid_controller_.formulation();

  for (auto it = tsid_node["tasks"].begin(); it != tsid_node["tasks"].end();
       ++it) {
    const auto key = it->first.as<std::string>();
    const auto &task_cfg = it->second;
    const auto type = task_cfg["type"].as<std::string>("");

    if (type == "posture") {
      auto task = std::make_unique<rtc::tsid::PostureTask>();
      task->init(model, robot_info_, pinocchio_cache_, task_cfg);
      formulation.add_task(std::move(task));
    } else if (type == "se3") {
      auto task = std::make_unique<rtc::tsid::SE3Task>();
      task->init(model, robot_info_, pinocchio_cache_, task_cfg);
      formulation.add_task(std::move(task));
    } else if (type == "force") {
      auto task = std::make_unique<rtc::tsid::ForceTask>();
      task->init(model, robot_info_, pinocchio_cache_, task_cfg);
      task->set_contact_manager(&contact_mgr_config_);
      formulation.add_task(std::move(task));
    } else {
      RCLCPP_ERROR(logger_,
                   "[wbc] unknown task type '%s' for entry '%s' — skipping",
                   type.c_str(), key.c_str());
    }
  }
}

void DemoWbcController::BuildTsidConstraints(const YAML::Node &tsid_node) {
  if (!full_model_ptr_ || !tsid_node || !tsid_node["constraints"]) {
    return;
  }
  const auto &model = *full_model_ptr_;
  auto &formulation = tsid_controller_.formulation();

  for (auto it = tsid_node["constraints"].begin();
       it != tsid_node["constraints"].end(); ++it) {
    const auto key = it->first.as<std::string>();
    const auto &c_cfg = it->second;
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
      RCLCPP_ERROR(
          logger_,
          "[wbc] unknown constraint type '%s' for entry '%s' — skipping",
          type.c_str(), key.c_str());
    }
  }
}

// ── Controller registry hooks ────────────────────────────────────────────────

void DemoWbcController::LoadConfig(const YAML::Node &cfg) {
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) {
    return;
  }

  // ── 1. Build models from system model config ──────────────────────────
  const auto *sys_cfg = GetSystemModelConfig();
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
    const auto &model = *full_model_ptr_;

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
    tsid_output_.init(robot_info_.nv, robot_info_.n_actuated,
                      contact_mgr_config_.max_contact_vars);

    // ControlState pre-allocate
    ctrl_state_.q = Eigen::VectorXd::Zero(robot_info_.nq);
    ctrl_state_.v = Eigen::VectorXd::Zero(robot_info_.nv);

    // TSIDController init (creates formulation; tasks/constraints added below)
    tsid_controller_.init(model, robot_info_, tsid_node);

    // Build tasks + constraints from YAML (auto-dispatch by `type` field)
    BuildTsidTasks(tsid_node);
    BuildTsidConstraints(tsid_node);

    // Verify contact frames resolved (catch mis-named fingertip frames early)
    for (const auto &c : contact_mgr_config_.contacts) {
      if (c.frame_id == 0) {
        RCLCPP_ERROR(logger_,
                     "[wbc] contact '%s' frame '%s' not found in full model",
                     c.name.c_str(), c.frame_name.c_str());
      }
    }

    // Pre-resolve phase presets
    phase_preset_valid_.fill(false);
    if (tsid_node["phase_presets"]) {
      auto presets = rtc::tsid::load_phase_presets(tsid_node);
      auto map_preset = [&](WbcPhase phase, const std::string &name) {
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
    RCLCPP_INFO(logger_, "TSID initialized: nq=%d nv=%d n_act=%d contacts=%d",
                robot_info_.nq, robot_info_.nv, robot_info_.n_actuated,
                contact_mgr_config_.max_contacts);
  }

  // ── 3. Integration buffers ────────────────────────────────────────────
  const int nv = full_model_ptr_->nv;
  q_curr_full_ = Eigen::VectorXd::Zero(nv);
  v_curr_full_ = Eigen::VectorXd::Zero(nv);
  q_next_full_ = Eigen::VectorXd::Zero(nv);
  v_next_full_ = Eigen::VectorXd::Zero(nv);

  // Joint limits with safety margins + force-rate filter (required)
  if (!cfg["integration"] || !cfg["integration"].IsMap()) {
    throw std::runtime_error(
        "demo_wbc_controller: required 'integration' section is missing");
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
  q_min_clamped_ =
      full_model_ptr_->lowerPositionLimit.array() + position_margin_;
  q_max_clamped_ =
      full_model_ptr_->upperPositionLimit.array() - position_margin_;
  v_limit_ = full_model_ptr_->velocityLimit * velocity_scale_;

  // ── 4. FSM thresholds ─────────────────────────────────────────────────
  if (cfg["fsm"]) {
    const auto fsm = cfg["fsm"];
    epsilon_approach_ = fsm["epsilon_approach"].as<double>(0.01);
    epsilon_pregrasp_ = fsm["epsilon_pregrasp"].as<double>(0.005);
    force_contact_threshold_ = fsm["force_contact_threshold"].as<double>(0.2);
    force_hold_threshold_ = fsm["force_hold_threshold"].as<double>(1.0);
    min_contacts_for_hold_ =
        fsm["min_contacts_for_hold"].as<int>(min_contacts_for_hold_);
    slip_rate_threshold_ =
        fsm["slip_rate_threshold"].as<double>(slip_rate_threshold_);
    deformation_threshold_ =
        fsm["deformation_threshold"].as<double>(deformation_threshold_);
    max_qp_fail_before_fallback_ =
        fsm["max_qp_fail_before_fallback"].as<int>(5);
    auto g = gains_lock_.Load();
    g.arm_trajectory_speed = std::max(
        1e-6, fsm["approach_speed"].as<double>(g.arm_trajectory_speed));
    gains_lock_.Store(g);
  }

  // ── 4b. E-STOP arm safe position (required) ──────────────────────────
  if (!cfg["estop"] || !cfg["estop"].IsMap()) {
    throw std::runtime_error(
        "demo_wbc_controller: required 'estop' section is missing");
  }
  {
    const auto estop_node = cfg["estop"];
    if (!estop_node["arm_safe_position"] ||
        !estop_node["arm_safe_position"].IsSequence()) {
      throw std::runtime_error(
          "demo_wbc_controller: required 'estop.arm_safe_position' "
          "must be a sequence");
    }
    const auto sp = estop_node["arm_safe_position"];
    if (sp.size() != static_cast<std::size_t>(kArmDof)) {
      throw std::runtime_error(
          "demo_wbc_controller: 'estop.arm_safe_position' length " +
          std::to_string(sp.size()) + " != expected " +
          std::to_string(kArmDof));
    }
    for (std::size_t i = 0; i < static_cast<std::size_t>(kArmDof); ++i) {
      safe_position_[i] = sp[i].as<double>();
    }
  }

  // ── 5. Trajectory speeds ──────────────────────────────────────────────
  {
    auto g = gains_lock_.Load();
    if (cfg["arm_trajectory_speed"]) {
      g.arm_trajectory_speed =
          std::max(1e-6, cfg["arm_trajectory_speed"].as<double>());
    }
    if (cfg["hand_trajectory_speed"]) {
      g.hand_trajectory_speed =
          std::max(1e-6, cfg["hand_trajectory_speed"].as<double>());
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
    command_type_ =
        (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }

  // ── 7. MPC integration (Phase 5 + 7b) ─────────────────────────────────
  //
  // If `mpc.enabled: true`, size the reference buffers and initialise the
  // MPC solution manager. The thread itself is spawned in
  // InitializeHoldPosition so it starts from the first valid RT tick's
  // state snapshot. If `mpc:` is missing or disabled, all MPC paths are
  // short-circuited and Phase 4 behaviour is preserved bit-exactly.
  //
  // Phase 7b: `mpc.engine` (default "mock") selects MockMPCThread vs
  // HandlerMPCThread. Handler mode additionally loads phase_config.yaml +
  // mpc_kinodynamics.yaml + mpc_fulldynamics.yaml from the package share
  // and pre-builds the RobotModelHandler + GraspPhaseManager for startup.
  if (const auto mpc_cfg = cfg["mpc"]; mpc_cfg && full_model_ptr_) {
    mpc_enabled_ = mpc_cfg["enabled"] && mpc_cfg["enabled"].as<bool>();

    const auto engine_str =
        mpc_cfg["engine"] ? mpc_cfg["engine"].as<std::string>("mock") : "mock";
    if (engine_str == "handler") {
      mpc_engine_ = MpcEngine::kHandler;
    } else if (engine_str == "mock") {
      mpc_engine_ = MpcEngine::kMock;
    } else {
      RCLCPP_ERROR(logger_,
                   "[wbc] unknown mpc.engine '%s' — falling back to 'mock'",
                   engine_str.c_str());
      mpc_engine_ = MpcEngine::kMock;
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
    const char *active_engine =
        (mpc_engine_ == MpcEngine::kHandler) ? "handler" : "mock";
    RCLCPP_INFO(logger_,
                "MPC integration: enabled=%d engine=%s (yaml=%s) nq=%d nv=%d "
                "n_contact=%d",
                mpc_enabled_, active_engine, engine_str.c_str(), mpc_nq, mpc_nv,
                mpc_n_contact);

    if (mpc_engine_ == MpcEngine::kHandler) {
      // Resolve factory YAML paths relative to the ur5e_bringup share dir.
      std::string share;
      try {
        share = ament_index_cpp::get_package_share_directory("ur5e_bringup");
      } catch (...) {
        RCLCPP_ERROR(logger_, "[wbc] cannot resolve ur5e_bringup share dir "
                              "— handler mode disabled");
        mpc_engine_ = MpcEngine::kMock;
      }

      if (mpc_engine_ == MpcEngine::kHandler) {
        const auto phase_path =
            mpc_cfg["phase_config_path"]
                ? mpc_cfg["phase_config_path"].as<std::string>()
                : std::string{"config/controllers/phase_config.yaml"};
        const auto light_path =
            mpc_cfg["light_contact_path"]
                ? mpc_cfg["light_contact_path"].as<std::string>()
                : std::string{"config/controllers/mpc_kinodynamics.yaml"};
        const auto rich_path =
            mpc_cfg["contact_rich_path"]
                ? mpc_cfg["contact_rich_path"].as<std::string>()
                : std::string{"config/controllers/mpc_fulldynamics.yaml"};

        const auto join = [&](const std::string &p) { return share + "/" + p; };

        try {
          phase_cfg_ = YAML::LoadFile(join(phase_path));
          mpc_light_cfg_ = YAML::LoadFile(join(light_path));
          mpc_rich_cfg_ = YAML::LoadFile(join(rich_path));
        } catch (const std::exception &e) {
          RCLCPP_ERROR(logger_,
                       "[wbc] failed to load handler YAML (%s) — "
                       "handler mode disabled",
                       e.what());
          mpc_engine_ = MpcEngine::kMock;
        }
      }

      if (mpc_engine_ == MpcEngine::kHandler) {
        // Build RobotModelHandler from mpc_kinodynamics.yaml's `model:`
        // subtree (identical to mpc_fulldynamics.yaml by contract — cross-
        // mode swap requires matching contact_frames).
        const auto model_node =
            mpc_light_cfg_["mpc"] && mpc_light_cfg_["mpc"]["model"]
                ? mpc_light_cfg_["mpc"]["model"]
                : YAML::Node{};
        mpc_model_handler_ = std::make_unique<rtc::mpc::RobotModelHandler>();
        const auto model_err =
            mpc_model_handler_->Init(*full_model_ptr_, model_node);
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
        auto pm = std::make_unique<ur5e_bringup::phase::GraspPhaseManager>(
            *mpc_model_handler_);
        const auto phase_err = pm->Load(phase_cfg_["grasp_phase_manager"]
                                            ? phase_cfg_["grasp_phase_manager"]
                                            : phase_cfg_);
        if (phase_err != ur5e_bringup::phase::GraspPhaseInitError::kNoError) {
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
  // ── Arm frame IDs ─────────────────────────────────────────────────────
  if (auto *cfg = GetDeviceNameConfig("ur5e"); cfg) {
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
    if (cfg->joint_limits) {
      if (!cfg->joint_limits->max_velocity.empty()) {
        device_max_velocity_[0] = cfg->joint_limits->max_velocity;
      }
      if (!cfg->joint_limits->position_lower.empty()) {
        device_position_lower_[0] = cfg->joint_limits->position_lower;
      }
      if (!cfg->joint_limits->position_upper.empty()) {
        device_position_upper_[0] = cfg->joint_limits->position_upper;
      }
    }
  }

  // ── Hand limits ───────────────────────────────────────────────────────
  if (auto *cfg = GetDeviceNameConfig("hand"); cfg && cfg->joint_limits) {
    if (!cfg->joint_limits->max_velocity.empty()) {
      device_max_velocity_[1] = cfg->joint_limits->max_velocity;
    }
    if (!cfg->joint_limits->position_lower.empty()) {
      device_position_lower_[1] = cfg->joint_limits->position_lower;
    }
    if (!cfg->joint_limits->position_upper.empty()) {
      device_position_upper_[1] = cfg->joint_limits->position_upper;
    }
  }

  // Fallback defaults
  for (auto &v : device_max_velocity_) {
    if (v.empty())
      v.assign(kMaxDeviceChannels, 2.0);
  }
  for (auto &v : device_position_lower_) {
    if (v.empty())
      v.assign(kMaxDeviceChannels, -6.2832);
  }
  for (auto &v : device_position_upper_) {
    if (v.empty())
      v.assign(kMaxDeviceChannels, 6.2832);
  }

  // ── Joint reorder map ─────────────────────────────────────────────────
  BuildJointReorderMap();
}

void DemoWbcController::UpdateGainsFromMsg(
    std::span<const double> gains) noexcept {
  // Layout (9 entries as of Phase 5):
  //   [ grasp_cmd, grasp_target_force,
  //     arm_traj_speed, hand_traj_speed,
  //     se3_weight, force_weight, posture_weight,
  //     mpc_enable (0/1), riccati_gain_scale (0..1) ]
  // Indices 0-6 are backwards-compatible with the Phase 4 7-entry layout.
  if (gains.size() >= 1) {
    grasp_cmd_.store(static_cast<int>(gains[0]), std::memory_order_release);
  }
  auto g = gains_lock_.Load();
  if (gains.size() >= 2) {
    g.grasp_target_force = gains[1];
  }
  if (gains.size() >= 3) {
    g.arm_trajectory_speed = std::max(1e-6, gains[2]);
  }
  if (gains.size() >= 4) {
    g.hand_trajectory_speed = std::max(1e-6, gains[3]);
  }
  if (gains.size() >= 5) {
    g.se3_weight = gains[4];
  }
  if (gains.size() >= 6) {
    g.force_weight = gains[5];
  }
  if (gains.size() >= 7) {
    g.posture_weight = gains[6];
  }
  gains_lock_.Store(g);
  if (gains.size() >= 8) {
    const bool requested = gains[7] > 0.5;
    mpc_manager_.SetEnabled(requested && mpc_enabled_);
  }
  if (gains.size() >= 9) {
    mpc_manager_.SetRiccatiGainScale(gains[8]);
  }
}

std::vector<double> DemoWbcController::GetCurrentGains() const noexcept {
  const auto g = gains_lock_.Load();
  return {static_cast<double>(grasp_cmd_.load(std::memory_order_relaxed)),
          g.grasp_target_force,
          g.arm_trajectory_speed,
          g.hand_trajectory_speed,
          g.se3_weight,
          g.force_weight,
          g.posture_weight,
          mpc_manager_.Enabled() ? 1.0 : 0.0,
          mpc_manager_.RiccatiGainScale()};
}

std::optional<rtc::MpcSolveStats>
DemoWbcController::GetMpcSolveStats() const noexcept {
  if (!mpc_manager_.Enabled()) {
    return std::nullopt;
  }
  const auto s = mpc_manager_.GetSolveStats();
  if (s.count == 0) {
    return std::nullopt;
  }
  rtc::MpcSolveStats out;
  out.count = s.count;
  out.window = s.window;
  out.last_ns = s.last_ns;
  out.min_ns = s.min_ns;
  out.max_ns = s.max_ns;
  out.p50_ns = s.p50_ns;
  out.p99_ns = s.p99_ns;
  out.mean_ns = s.mean_ns;
  return out;
}

DemoWbcController::FingertipReport
DemoWbcController::GetFingertipReportForTesting(
    int fingertip_idx) const noexcept {
  FingertipReport r{};
  if (fingertip_idx < 0 ||
      fingertip_idx >= static_cast<int>(rtc::kMaxFingertips)) {
    return r;
  }
  const auto &ft = fingertip_data_[static_cast<std::size_t>(fingertip_idx)];
  r.force_magnitude = ft.force_magnitude;
  r.force_rate = ft.force_rate;
  r.contact_flag = ft.contact_flag;
  r.valid = ft.valid;
  return r;
}

// ── RT control loop ──────────────────────────────────────────────────────────

ControllerOutput
DemoWbcController::Compute(const ControllerState &state) noexcept {
  const double dt = (state.dt > 0.0) ? state.dt : GetDefaultDt();

  ReadState(state);

  // E-STOP takes priority over FSM
  estop_active_ = estopped_.load(std::memory_order_acquire);
  if (estop_active_) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    return out;
  }

  ComputeControl(state, dt);
  return WriteOutput(state);
}

// ── Phase 1: Read state ──────────────────────────────────────────────────────

void DemoWbcController::ReadState(const ControllerState &state) noexcept {
  // Parse fingertip F/T inference data + raw sensor channels for
  // contact detection (kClosure -> kHold) and anomaly monitoring (kHold).
  // Layout mirrors DemoJointController::ReadState for consistency.
  num_active_fingertips_ = 0;
  if (state.num_devices <= 1 || !state.devices[1].valid) {
    return;
  }

  const auto &dev1 = state.devices[1];
  const int num_sensor_ch = dev1.num_sensor_channels;
  const int num_fingertips =
      (rtc::kSensorValuesPerFingertip > 0)
          ? (num_sensor_ch / rtc::kSensorValuesPerFingertip)
          : 0;
  num_active_fingertips_ =
      std::min(num_fingertips, static_cast<int>(rtc::kMaxFingertips));

  const double inv_dt = (state.dt > 0.0) ? (1.0 / state.dt) : 500.0;

  for (int f = 0; f < num_active_fingertips_; ++f) {
    auto &ft = fingertip_data_[static_cast<std::size_t>(f)];
    const int base = f * rtc::kSensorValuesPerFingertip;

    for (std::size_t j = 0; j < rtc::kBarometerCount; ++j) {
      ft.baro[j] = dev1.sensor_data[static_cast<std::size_t>(base) + j];
    }
    for (std::size_t j = 0; j < 3; ++j) {
      ft.tof[j] = dev1.sensor_data[static_cast<std::size_t>(base) +
                                   rtc::kBarometerCount + j];
    }

    ft.valid = dev1.inference_enable[static_cast<std::size_t>(f)];
    if (ft.valid) {
      const int ft_base = f * rtc::kFTValuesPerFingertip;
      ft.contact_flag = dev1.inference_data[static_cast<std::size_t>(ft_base)];
      for (int j = 0; j < 3; ++j) {
        ft.force[static_cast<std::size_t>(j)] =
            dev1.inference_data[static_cast<std::size_t>(ft_base + 1 + j)];
        ft.displacement[static_cast<std::size_t>(j)] =
            dev1.inference_data[static_cast<std::size_t>(ft_base + 4 + j)];
      }
      const float fx = ft.force[0];
      const float fy = ft.force[1];
      const float fz = ft.force[2];
      ft.force_magnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
    } else {
      ft.contact_flag = 0.0f;
      ft.force = {};
      ft.displacement = {};
      ft.force_magnitude = 0.0f;
    }

    // df/dt with EMA smoothing; skip on first tick to avoid startup spike
    if (force_rate_initialized_) {
      const float raw_rate = static_cast<float>(
          (ft.force_magnitude - ft.prev_force_magnitude) * inv_dt);
      ft.force_rate = force_rate_alpha_ * raw_rate +
                      (1.0f - force_rate_alpha_) * ft.force_rate;
    } else {
      ft.force_rate = 0.0f;
    }
    ft.prev_force_magnitude = ft.force_magnitude;
  }
  force_rate_initialized_ = true;
}

// ── Phase 2: Compute control ─────────────────────────────────────────────────

void DemoWbcController::ComputeControl(const ControllerState &state,
                                       double dt) noexcept {
  UpdatePhase(state);

  // Keep MPC state fresh across all phases: HandlerMPCThread::Solve rejects
  // dim-mismatched snapshots, so non-TSID phases (kIdle/kApproach/kRetreat/
  // kRelease) would otherwise starve the solver and leave mpc_solve_timing.csv
  // with only the header.
  if (mpc_enabled_ && mpc_manager_.Enabled()) {
    ExtractFullState(state);
    const uint64_t now_ns =
        static_cast<uint64_t>(state.iteration) * 2'000'000ULL;
    mpc_manager_.WriteState(q_curr_full_, v_curr_full_, now_ns);
  }

  switch (phase_) {
  case WbcPhase::kIdle:
  case WbcPhase::kApproach:
  case WbcPhase::kRetreat:
  case WbcPhase::kRelease:
    ComputePositionMode(dt);
    break;

  case WbcPhase::kPreGrasp:
  case WbcPhase::kClosure:
  case WbcPhase::kHold:
    if (tsid_initialized_) {
      ComputeTSIDPosition(state, dt);
    } else {
      ComputePositionMode(dt); // Fallback if TSID not available
    }
    break;

  case WbcPhase::kFallback:
    ComputeFallback();
    break;
  }
}

// ── FSM ──────────────────────────────────────────────────────────────────────

void DemoWbcController::UpdatePhase(const ControllerState &state) noexcept {
  const int cmd = grasp_cmd_.load(std::memory_order_acquire);
  WbcPhase next = phase_;

  switch (phase_) {
  case WbcPhase::kIdle:
    // grasp_cmd=1 + valid target → approach
    if (cmd == 1 && robot_new_target_.load(std::memory_order_acquire)) {
      next = WbcPhase::kApproach;
    }
    break;

  case WbcPhase::kApproach: {
    // Trajectory complete → pre-grasp (TSID)
    if (robot_trajectory_time_ >= robot_trajectory_.duration()) {
      if (tsid_initialized_) {
        next = WbcPhase::kPreGrasp;
      } else {
        next = WbcPhase::kIdle; // No TSID, stay in position mode
      }
    }
    // Abort
    if (cmd == 0) {
      next = WbcPhase::kIdle;
    }
    break;
  }

  case WbcPhase::kPreGrasp: {
    // TCP close enough to goal → closure (Phase 4B)
    if (tcp_goal_valid_) {
      const double err = ComputeTcpError(tcp_goal_);
      if (err < epsilon_pregrasp_) {
        next = WbcPhase::kClosure; // Phase 4B
      }
    }
    // Abort
    if (cmd == 0) {
      next = WbcPhase::kIdle;
    }
    break;
  }

  case WbcPhase::kClosure: {
    // Count active contacts from parsed fingertip forces
    int active_contacts = 0;
    for (int f = 0; f < num_active_fingertips_; ++f) {
      const auto &ft = fingertip_data_[static_cast<std::size_t>(f)];
      if (ft.valid && ft.force_magnitude > force_contact_threshold_) {
        ++active_contacts;
      }
    }
    if (active_contacts >= min_contacts_for_hold_) {
      next = WbcPhase::kHold;
    }
    if (cmd == 0) {
      next = WbcPhase::kIdle;
    }
    break;
  }

  case WbcPhase::kHold: {
    // Anomaly detection: slip (|df/dt|) or excessive deformation
    for (int f = 0; f < num_active_fingertips_; ++f) {
      const auto &ft = fingertip_data_[static_cast<std::size_t>(f)];
      if (!ft.valid) {
        continue;
      }
      if (std::abs(ft.force_rate) > slip_rate_threshold_) {
        RCLCPP_WARN_THROTTLE(
            logger_, log_clock_, ur5e_bringup::logging::kThrottleSlowMs,
            "[wbc] slip detected f=%d df/dt=%.2f N/s > %.2f", f,
            static_cast<double>(ft.force_rate), slip_rate_threshold_);
        next = WbcPhase::kFallback;
        break;
      }
      const float dmag = std::sqrt(ft.displacement[0] * ft.displacement[0] +
                                   ft.displacement[1] * ft.displacement[1] +
                                   ft.displacement[2] * ft.displacement[2]);
      if (dmag > deformation_threshold_) {
        RCLCPP_WARN_THROTTLE(
            logger_, log_clock_, ur5e_bringup::logging::kThrottleSlowMs,
            "[wbc] deformation detected f=%d |d|=%.3f > %.3f", f,
            static_cast<double>(dmag), deformation_threshold_);
        next = WbcPhase::kFallback;
        break;
      }
    }
    if (cmd == 2) {
      next = WbcPhase::kRetreat;
    }
    if (cmd == 0) {
      next = WbcPhase::kIdle;
    }
    break;
  }

  case WbcPhase::kRetreat:
    // Phase 4B: trajectory complete → release
    if (robot_trajectory_time_ >= robot_trajectory_.duration()) {
      next = WbcPhase::kRelease;
    }
    if (cmd == 0) {
      next = WbcPhase::kIdle;
    }
    break;

  case WbcPhase::kRelease:
    // Phase 4B: hand open complete → idle
    if (hand_trajectory_time_ >= hand_trajectory_.duration()) {
      next = WbcPhase::kIdle;
    }
    break;

  case WbcPhase::kFallback:
    // Manual recovery only: grasp_cmd=0 → idle
    if (cmd == 0) {
      next = WbcPhase::kIdle;
    }
    break;
  }

  if (next != phase_) {
    RCLCPP_INFO_THROTTLE(logger_, log_clock_,
                         ur5e_bringup::logging::kThrottleFastMs,
                         "[wbc] phase %d -> %d", static_cast<int>(phase_),
                         static_cast<int>(next));
    prev_phase_ = phase_;
    OnPhaseEnter(next, state);
    phase_ = next;
  }
}

void DemoWbcController::OnPhaseEnter(WbcPhase new_phase,
                                     const ControllerState &state) noexcept {
  // Atomic gains snapshot — phase transitions are RT-safe reads.
  const auto gains = gains_lock_.Load();
  const auto &dev0 = state.devices[0];
  const auto &dev1 = state.devices[1];

  switch (new_phase) {
  case WbcPhase::kIdle: {
    // Hold current position
    for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
      robot_computed_.positions[i] = dev0.positions[i];
      robot_computed_.velocities[i] = 0.0;
    }
    if (state.num_devices > 1 && dev1.valid) {
      for (std::size_t i = 0; i < kNumHandMotors; ++i) {
        hand_computed_.positions[i] = dev1.positions[i];
        hand_computed_.velocities[i] = 0.0;
      }
    }
    tcp_goal_valid_ = false;
    qp_fail_count_ = 0;
    // Deactivate all contacts
    for (auto &c : contact_state_.contacts) {
      c.active = false;
    }
    contact_state_.recompute_active(contact_mgr_config_);
    break;
  }

  case WbcPhase::kApproach: {
    // Build quintic trajectory: current → target (arm)
    std::lock_guard lock(target_mutex_);
    trajectory::JointSpaceTrajectory<kNumRobotJoints>::State start{};
    trajectory::JointSpaceTrajectory<kNumRobotJoints>::State goal{};
    double max_delta = 0.0;
    for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
      start.positions[i] = dev0.positions[i];
      q_approach_start_[i] = dev0.positions[i]; // save for kRetreat
      goal.positions[i] = device_targets_[0][i];
      const double delta = std::abs(goal.positions[i] - start.positions[i]);
      if (delta > max_delta) {
        max_delta = delta;
      }
    }
    const double duration =
        std::max(max_delta / gains.arm_trajectory_speed, 0.1);
    robot_trajectory_.initialize(start, goal, duration);
    robot_trajectory_time_ = 0.0;
    robot_new_target_.store(false, std::memory_order_relaxed);

    // Compute FK of arm target for SE3Task reference in kPreGrasp
    if (arm_handle_) {
      std::span<const double> q_target(device_targets_[0].data(),
                                       kNumRobotJoints);
      arm_handle_->ComputeForwardKinematics(q_target);
      tcp_goal_ = arm_handle_->GetFramePlacement(tip_frame_id_);
      if (use_root_frame_) {
        tcp_goal_ =
            arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp_goal_);
      }
      tcp_goal_valid_ = true;
    }

    // Hand trajectory (pre-shape)
    if (hand_new_target_.load(std::memory_order_acquire) &&
        state.num_devices > 1 && dev1.valid) {
      trajectory::JointSpaceTrajectory<kNumHandMotors>::State hstart{};
      trajectory::JointSpaceTrajectory<kNumHandMotors>::State hgoal{};
      double hmax = 0.0;
      for (std::size_t i = 0; i < kNumHandMotors; ++i) {
        hstart.positions[i] = dev1.positions[i];
        hgoal.positions[i] = device_targets_[1][i];
        const double hd = std::abs(hgoal.positions[i] - hstart.positions[i]);
        if (hd > hmax) {
          hmax = hd;
        }
      }
      const double hdur = std::max(hmax / gains.hand_trajectory_speed, 0.1);
      hand_trajectory_.initialize(hstart, hgoal, hdur);
      hand_trajectory_time_ = 0.0;
      hand_new_target_.store(false, std::memory_order_relaxed);
    }
    break;
  }

  case WbcPhase::kPreGrasp:
  case WbcPhase::kClosure:
  case WbcPhase::kHold: {
    // Apply TSID phase preset (RT-safe: uses pre-resolved PhasePreset)
    const auto idx = static_cast<std::size_t>(new_phase);
    if (phase_preset_valid_[idx]) {
      tsid_controller_.apply_phase_preset(phase_presets_[idx]);
    }

    // Set TSID integration initial conditions from current state
    ExtractFullState(state);
    q_next_full_ = q_curr_full_;
    v_next_full_ = v_curr_full_;

    // Set SE3Task reference (TCP goal)
    if (tcp_goal_valid_) {
      auto *se3_task = tsid_controller_.formulation().get_task("se3_tcp");
      if (se3_task) {
        static_cast<rtc::tsid::SE3Task *>(se3_task)->set_se3_reference(
            tcp_goal_);
      }
    }

    // Contact activation (Phase 4B)
    if (new_phase == WbcPhase::kClosure || new_phase == WbcPhase::kHold) {
      for (auto &c : contact_state_.contacts) {
        c.active = true;
      }
      contact_state_.recompute_active(contact_mgr_config_);

      // Set per-contact force reference: +Z normal = gains.grasp_target_force
      auto *force_task = tsid_initialized_
                             ? tsid_controller_.formulation().get_task("force")
                             : nullptr;
      if (force_task) {
        const int n = contact_mgr_config_.max_contact_vars;
        if (n > 0) {
          Eigen::VectorXd lambda_des = Eigen::VectorXd::Zero(n);
          int offset = 0;
          for (const auto &c : contact_mgr_config_.contacts) {
            const int cdim = c.contact_dim;
            if (offset + cdim > n) {
              break;
            }
            // Point contact: lambda = [fx, fy, fz]; push +Z target force
            if (cdim >= 3) {
              lambda_des[offset + 2] = gains.grasp_target_force;
            }
            offset += cdim;
          }
          static_cast<rtc::tsid::ForceTask *>(force_task)
              ->set_force_references(lambda_des);
        }
      }

      // Ramp hand joint target toward stored target (user-provided close pose)
      if (state.num_devices > 1 && dev1.valid) {
        trajectory::JointSpaceTrajectory<kNumHandMotors>::State hstart{};
        trajectory::JointSpaceTrajectory<kNumHandMotors>::State hgoal{};
        double hmax = 0.0;
        for (std::size_t i = 0; i < kNumHandMotors; ++i) {
          hstart.positions[i] = dev1.positions[i];
          hgoal.positions[i] = device_targets_[1][i];
          const double hd = std::abs(hgoal.positions[i] - hstart.positions[i]);
          if (hd > hmax) {
            hmax = hd;
          }
        }
        const double hdur = std::max(hmax / gains.hand_trajectory_speed, 0.1);
        hand_trajectory_.initialize(hstart, hgoal, hdur);
        hand_trajectory_time_ = 0.0;
      }
    }

    qp_fail_count_ = 0;
    break;
  }

  case WbcPhase::kRetreat: {
    // Deactivate contacts
    for (auto &c : contact_state_.contacts) {
      c.active = false;
    }
    contact_state_.recompute_active(contact_mgr_config_);

    // Arm trajectory: current → saved approach-start pose
    trajectory::JointSpaceTrajectory<kNumRobotJoints>::State start{};
    trajectory::JointSpaceTrajectory<kNumRobotJoints>::State goal{};
    double max_delta = 0.0;
    for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
      start.positions[i] = dev0.positions[i];
      goal.positions[i] = q_approach_start_[i];
      const double delta = std::abs(goal.positions[i] - start.positions[i]);
      if (delta > max_delta) {
        max_delta = delta;
      }
    }
    const double duration =
        std::max(max_delta / gains.arm_trajectory_speed, 0.1);
    robot_trajectory_.initialize(start, goal, duration);
    robot_trajectory_time_ = 0.0;
    tcp_goal_valid_ = false;
    break;
  }

  case WbcPhase::kRelease: {
    // Hand open: all motors → 0
    if (state.num_devices > 1 && dev1.valid) {
      trajectory::JointSpaceTrajectory<kNumHandMotors>::State hstart{};
      trajectory::JointSpaceTrajectory<kNumHandMotors>::State hgoal{};
      double hmax = 0.0;
      for (std::size_t i = 0; i < kNumHandMotors; ++i) {
        hstart.positions[i] = dev1.positions[i];
        hgoal.positions[i] = 0.0;
        const double hd = std::abs(hstart.positions[i]);
        if (hd > hmax) {
          hmax = hd;
        }
      }
      const double hdur = std::max(hmax / gains.hand_trajectory_speed, 0.1);
      hand_trajectory_.initialize(hstart, hgoal, hdur);
      hand_trajectory_time_ = 0.0;
    }
    // Arm holds current pose during release
    for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
      robot_computed_.positions[i] = dev0.positions[i];
      robot_computed_.velocities[i] = 0.0;
    }
    // Freeze arm trajectory (duration=0 so ComputePositionMode clamps)
    trajectory::JointSpaceTrajectory<kNumRobotJoints>::State hold{};
    for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
      hold.positions[i] = dev0.positions[i];
    }
    robot_trajectory_.initialize(hold, hold, 0.01);
    robot_trajectory_time_ = 0.0;
    break;
  }

  case WbcPhase::kFallback: {
    // Hold current position, deactivate contacts
    for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
      robot_computed_.positions[i] = dev0.positions[i];
      robot_computed_.velocities[i] = 0.0;
    }
    if (state.num_devices > 1 && dev1.valid) {
      for (std::size_t i = 0; i < kNumHandMotors; ++i) {
        hand_computed_.positions[i] = dev1.positions[i];
        hand_computed_.velocities[i] = 0.0;
      }
    }
    for (auto &c : contact_state_.contacts) {
      c.active = false;
    }
    contact_state_.recompute_active(contact_mgr_config_);
    qp_fail_count_ = 0;
    break;
  }
  }

  // ── GraspPhaseManager bridge (Phase 7b, handler mode only) ────────────────
  //
  // WBC FSM is authoritative for the demo; the grasp phase manager mirrors
  // it via ForcePhase so rtc_mpc picks up the matching OCP type
  // (light_contact vs contact_rich) on every WBC edge. `ForcePhase` is
  // atomic and RT-safe (see grasp_phase_manager.hpp thread-safety notes);
  // `SetTaskTarget` uses a non-RT mutex but fires at most once per WBC edge
  // (not per 500 Hz tick), which is acceptable off the TSID hot path.
  // WBC has no direct MANIPULATE analogue — kClosure maps to CLOSURE and
  // kHold to HOLD; MANIPULATE is reserved for a future WBC extension.
  if (phase_manager_ptr_) {
    namespace phase = ur5e_bringup::phase;
    int grasp_id = static_cast<int>(phase::GraspPhaseId::kIdle);
    switch (new_phase) {
    case WbcPhase::kIdle:
      grasp_id = static_cast<int>(phase::GraspPhaseId::kIdle);
      break;
    case WbcPhase::kApproach:
      grasp_id = static_cast<int>(phase::GraspPhaseId::kApproach);
      if (tcp_goal_valid_) {
        phase::GraspTarget gt{};
        gt.grasp_pose = tcp_goal_;
        gt.pregrasp_pose = tcp_goal_;
        gt.approach_start = tcp_goal_;
        phase_manager_ptr_->SetTaskTarget(gt);
      }
      break;
    case WbcPhase::kPreGrasp:
      grasp_id = static_cast<int>(phase::GraspPhaseId::kPreGrasp);
      break;
    case WbcPhase::kClosure:
      grasp_id = static_cast<int>(phase::GraspPhaseId::kClosure);
      break;
    case WbcPhase::kHold:
      grasp_id = static_cast<int>(phase::GraspPhaseId::kHold);
      break;
    case WbcPhase::kRetreat:
      grasp_id = static_cast<int>(phase::GraspPhaseId::kRetreat);
      break;
    case WbcPhase::kRelease:
      grasp_id = static_cast<int>(phase::GraspPhaseId::kRelease);
      break;
    case WbcPhase::kFallback:
      grasp_id = static_cast<int>(phase::GraspPhaseId::kIdle);
      break;
    }
    phase_manager_ptr_->ForcePhase(grasp_id);
  }
}

// ── Control modes ────────────────────────────────────────────────────────────

void DemoWbcController::ComputePositionMode(double dt) noexcept {
  // Robot arm trajectory
  robot_trajectory_time_ += dt;
  const auto rstate = robot_trajectory_.compute(
      std::min(robot_trajectory_time_, robot_trajectory_.duration()));
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    robot_computed_.positions[i] = rstate.positions[i];
    robot_computed_.velocities[i] = rstate.velocities[i];
  }

  // Hand trajectory
  hand_trajectory_time_ += dt;
  const auto hstate = hand_trajectory_.compute(
      std::min(hand_trajectory_time_, hand_trajectory_.duration()));
  for (std::size_t i = 0; i < kNumHandMotors; ++i) {
    hand_computed_.positions[i] = hstate.positions[i];
    hand_computed_.velocities[i] = hstate.velocities[i];
  }
}

void DemoWbcController::ComputeTSIDPosition(const ControllerState &state,
                                            double dt) noexcept {
  // 1. Extract full state (sensor values, every tick)
  ExtractFullState(state);

  // 2. Update Pinocchio cache (M, h, g, Jacobians)
  pinocchio_cache_.update(q_curr_full_, v_curr_full_, contact_state_);

  // 2b. MPC reference injection (Phase 5).
  //
  // Publish the current RT state to the MPC thread, then try to consume
  // the freshest MPC solution. If we have a valid interpolated reference,
  // it replaces the Phase 4 self-regularising hold target on the next
  // line. If not (MPC disabled / not yet publishing / too many stale
  // cycles), we fall through to the Phase 4 behaviour.
  bool mpc_ref_valid = false;
  if (mpc_enabled_ && mpc_manager_.Enabled()) {
    const uint64_t now_ns =
        static_cast<uint64_t>(state.iteration) * 2'000'000ULL; // 500 Hz tick
    rtc::mpc::InterpMeta meta;
    mpc_manager_.WriteState(q_curr_full_, v_curr_full_, now_ns);
    mpc_ref_valid = mpc_manager_.ComputeReference(
        q_curr_full_, v_curr_full_, now_ns, mpc_q_ref_, mpc_v_ref_, mpc_a_ff_,
        mpc_lambda_ref_, mpc_u_fb_, meta);
  }

  // 3. Set posture reference (regularization toward MPC q_ref if valid,
  //    else toward current position for self-holding behaviour).
  if (mpc_ref_valid) {
    control_ref_.q_des = mpc_q_ref_;
    control_ref_.v_des = mpc_v_ref_;
    // TSID will combine a_ff with task PD correction. u_fb from Riccati
    // is additive acceleration feedback on the actuated joints only.
    control_ref_.a_des = mpc_a_ff_;
    const int n_fb = std::min(static_cast<int>(mpc_u_fb_.size()),
                              static_cast<int>(control_ref_.a_des.size()));
    control_ref_.a_des.head(n_fb) += mpc_u_fb_.head(n_fb);
  } else {
    control_ref_.q_des = q_curr_full_;
    control_ref_.v_des.setZero();
    control_ref_.a_des.setZero();
  }

  // 4. Build ControlState
  ctrl_state_.q = q_curr_full_;
  ctrl_state_.v = v_curr_full_;
  ctrl_state_.timestamp_ns = state.iteration;

  // 5. TSID solve
  tsid_output_ = tsid_controller_.compute(ctrl_state_, control_ref_,
                                          pinocchio_cache_, contact_state_);

  // 6. QP failure handling
  if (!tsid_output_.qp_converged) {
    ++qp_fail_count_;
    RCLCPP_WARN_THROTTLE(
        logger_, log_clock_, ur5e_bringup::logging::kThrottleSlowMs,
        "[wbc] QP failed (%d/%d), solve=%.0fus", qp_fail_count_,
        max_qp_fail_before_fallback_, tsid_output_.solve_time_us);

    if (qp_fail_count_ >= max_qp_fail_before_fallback_) {
      phase_ = WbcPhase::kFallback;
      ComputeFallback();
      return;
    }
    // This tick: hold last valid output
    return;
  }
  qp_fail_count_ = 0;

  // 7. Semi-implicit Euler integration: a → v → q
  const auto &a = tsid_output_.a_opt;

  // v_next = v_curr + a · dt
  v_next_full_.noalias() = v_curr_full_ + a * dt;

  // Velocity clamp
  v_next_full_ = v_next_full_.cwiseMax(-v_limit_).cwiseMin(v_limit_);

  // q_next = q_curr + v_next · dt
  q_next_full_.noalias() = q_curr_full_ + v_next_full_ * dt;

  // Position clamp (safety net, should not trigger under normal TSID)
  q_next_full_ = q_next_full_.cwiseMax(q_min_clamped_).cwiseMin(q_max_clamped_);

  // 8. Map Pinocchio order → device order
  for (int i = 0; i < kArmDof; ++i) {
    const auto pin_idx =
        static_cast<std::size_t>(ext_to_pin_q_[static_cast<std::size_t>(i)]);
    robot_computed_.positions[static_cast<std::size_t>(i)] =
        q_next_full_[static_cast<Eigen::Index>(pin_idx)];
    robot_computed_.velocities[static_cast<std::size_t>(i)] =
        v_next_full_[static_cast<Eigen::Index>(pin_idx)];
  }
  for (int i = 0; i < kHandDof; ++i) {
    const auto ext_i = static_cast<std::size_t>(kArmDof + i);
    const auto pin_idx = static_cast<std::size_t>(ext_to_pin_q_[ext_i]);
    hand_computed_.positions[static_cast<std::size_t>(i)] =
        q_next_full_[static_cast<Eigen::Index>(pin_idx)];
    hand_computed_.velocities[static_cast<std::size_t>(i)] =
        v_next_full_[static_cast<Eigen::Index>(pin_idx)];
  }

  RCLCPP_INFO_THROTTLE(logger_, log_clock_,
                       ur5e_bringup::logging::kThrottleSlowMs,
                       "[wbc] TSID solve=%.0fus qp_ok phase=%d",
                       tsid_output_.solve_time_us, static_cast<int>(phase_));
}

void DemoWbcController::ComputeFallback() noexcept {
  // Hold last computed positions (already in robot_computed_/hand_computed_)
  // Set velocities to zero
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    robot_computed_.velocities[i] = 0.0;
  }
  for (std::size_t i = 0; i < kNumHandMotors; ++i) {
    hand_computed_.velocities[i] = 0.0;
  }
}

// ── Phase 3: Write output ────────────────────────────────────────────────────

ControllerOutput
DemoWbcController::WriteOutput(const ControllerState &state) noexcept {
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.command_type = command_type_;

  // ── Robot arm output ──────────────────────────────────────────────────
  const auto &dev0 = state.devices[0];
  auto &out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kJoint;

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.commands[i] = robot_computed_.positions[i];
    out0.target_positions[i] = robot_computed_.positions[i];
    out0.target_velocities[i] = robot_computed_.velocities[i];
    out0.trajectory_positions[i] = robot_computed_.positions[i];
    out0.trajectory_velocities[i] = robot_computed_.velocities[i];
    out0.goal_positions[i] = device_targets_[0][i];
  }
  ClampCommands(out0.commands, nc0, device_position_lower_[0],
                device_position_upper_[0]);

  // ── Task-space logging (FK) ───────────────────────────────────────────
  if (arm_handle_) {
    std::span<const double> q_span(dev0.positions.data(),
                                   static_cast<std::size_t>(nc0));
    arm_handle_->ComputeForwardKinematics(q_span);
    pinocchio::SE3 tcp = arm_handle_->GetFramePlacement(tip_frame_id_);
    if (use_root_frame_) {
      tcp = arm_handle_->GetFramePlacement(root_frame_id_).actInv(tcp);
    }
    Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp.rotation());
    output.actual_task_positions[0] = tcp.translation().x();
    output.actual_task_positions[1] = tcp.translation().y();
    output.actual_task_positions[2] = tcp.translation().z();
    output.actual_task_positions[3] = rpy[0];
    output.actual_task_positions[4] = rpy[1];
    output.actual_task_positions[5] = rpy[2];

    // Task goal = TCP goal if valid, else mirror actual
    if (tcp_goal_valid_) {
      Eigen::Vector3d grpy = pinocchio::rpy::matrixToRpy(tcp_goal_.rotation());
      output.task_goal_positions[0] = tcp_goal_.translation().x();
      output.task_goal_positions[1] = tcp_goal_.translation().y();
      output.task_goal_positions[2] = tcp_goal_.translation().z();
      output.task_goal_positions[3] = grpy[0];
      output.task_goal_positions[4] = grpy[1];
      output.task_goal_positions[5] = grpy[2];
    } else {
      output.task_goal_positions = output.actual_task_positions;
    }
  }

  // ── Hand output ───────────────────────────────────────────────────────
  if (state.num_devices > 1 && state.devices[1].valid) {
    const int nc1 = state.devices[1].num_channels;
    auto &out1 = output.devices[1];
    out1.num_channels = nc1;
    out1.goal_type = GoalType::kJoint;

    for (std::size_t i = 0; i < static_cast<std::size_t>(nc1); ++i) {
      out1.commands[i] = hand_computed_.positions[i];
      out1.target_positions[i] = hand_computed_.positions[i];
      out1.target_velocities[i] = hand_computed_.velocities[i];
      out1.trajectory_positions[i] = hand_computed_.positions[i];
      out1.trajectory_velocities[i] = hand_computed_.velocities[i];
      out1.goal_positions[i] = device_targets_[1][i];
    }
    ClampCommands(out1.commands, nc1, device_position_lower_[1],
                  device_position_upper_[1]);
  }

  output.valid = true;
  return output;
}

// ── Target management ────────────────────────────────────────────────────────

void DemoWbcController::SetDeviceTarget(
    int device_idx, std::span<const double> target) noexcept {
  std::lock_guard lock(target_mutex_);
  const auto didx = static_cast<std::size_t>(device_idx);
  if (didx >= device_targets_.size()) {
    return;
  }

  const auto n =
      std::min(target.size(), static_cast<std::size_t>(kMaxDeviceChannels));
  std::copy_n(target.data(), n, device_targets_[didx].data());

  if (device_idx == 0) {
    robot_new_target_.store(true, std::memory_order_release);
  } else if (device_idx == 1) {
    hand_new_target_.store(true, std::memory_order_release);
  }
}

void DemoWbcController::InitializeHoldPosition(
    const ControllerState &state) noexcept {
  std::lock_guard lock(target_mutex_);

  // Robot arm: initialize trajectory at current position (zero velocity)
  {
    const auto &dev0 = state.devices[0];
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
  for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices);
       ++d) {
    const auto &dev = state.devices[d];
    if (!dev.valid)
      continue;
    for (std::size_t i = 0; i < static_cast<std::size_t>(dev.num_channels) &&
                            i < kMaxDeviceChannels;
         ++i) {
      device_targets_[d][i] = dev.positions[i];
    }
    if (d == 1) {
      trajectory::JointSpaceTrajectory<kNumHandMotors>::State hold{};
      for (std::size_t i = 0; i < kNumHandMotors; ++i) {
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

  // ── MPC thread lifecycle (Phase 5 + 7b) ─────────────────────────────────
  //
  // Spawn the MPC thread lazily on first InitializeHoldPosition call so it
  // starts from a known-good RT state. Subsequent calls (e.g. controller
  // re-init after E-STOP clear) leave the existing thread running.
  //
  // Handler mode (Phase 7b) additionally builds the initial PhaseContext
  // from the idle phase, runs MPCFactory::Create, and hands both the
  // handler and the pre-loaded GraspPhaseManager to HandlerMPCThread::
  // Configure. On factory failure we fall back to mock mode so the RT loop
  // can still make progress.
  if (mpc_enabled_ && !mpc_thread_) {
    const auto thread_configs = rtc::SelectThreadConfigs();
    const int nq = static_cast<int>(q_curr_full_.size());
    const int nv = static_cast<int>(v_curr_full_.size());

    rtc::mpc::MpcThreadLaunchConfig launch{};
    launch.main = thread_configs.mpc.main;
    launch.num_workers = thread_configs.mpc.num_workers;
    for (int i = 0; i < launch.num_workers && i < rtc::mpc::kMaxMpcWorkers;
         ++i) {
      launch.workers[static_cast<std::size_t>(i)] =
          thread_configs.mpc.workers[static_cast<std::size_t>(i)];
    }
    launch.target_frequency_hz = 20.0;

    bool thread_started = false;

    if (mpc_engine_ == MpcEngine::kHandler && mpc_model_handler_ &&
        phase_manager_owned_) {
      // Build the initial PhaseContext from the idle phase so the factory
      // can size its OCP + solver workspace. GraspPhaseManager and all
      // downstream solvers live in the reduced (mimic-locked) MPC model's
      // index space, so seed zeros at those dims — not the full_model's.
      Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(full_model_ptr_->nq);
      Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(full_model_ptr_->nv);
      Eigen::VectorXd sensor_zero = Eigen::VectorXd::Zero(0);
      const pinocchio::SE3 tcp_identity = pinocchio::SE3::Identity();
      const auto initial_ctx = phase_manager_owned_->Update(
          q_zero, v_zero, sensor_zero, tcp_identity, /*t=*/0.0);

      std::unique_ptr<rtc::mpc::MPCHandlerBase> handler;
      const auto status = rtc::mpc::MPCFactory::Create(
          mpc_light_cfg_, *mpc_model_handler_, initial_ctx, handler);

      if (status.error != rtc::mpc::MPCFactoryError::kNoError || !handler) {
        RCLCPP_ERROR(logger_,
                     "[wbc] MPCFactory::Create failed (err=%d, init=%d) — "
                     "falling back to mock engine",
                     static_cast<int>(status.error),
                     static_cast<int>(status.init_error));
        mpc_engine_ = MpcEngine::kMock;
      } else {
        auto hthread = std::make_unique<rtc::mpc::HandlerMPCThread>();
        hthread->Configure(*mpc_model_handler_, std::move(handler),
                           std::move(phase_manager_owned_), mpc_light_cfg_,
                           mpc_rich_cfg_);
        hthread->Init(mpc_manager_, launch);
        hthread->Start();
        mpc_thread_ = std::move(hthread);
        mpc_manager_.SetEnabled(true);
        thread_started = true;
        RCLCPP_INFO(logger_,
                    "MPC thread started (handler): core=%d prio=%d workers=%d",
                    launch.main.cpu_core, launch.main.sched_priority,
                    launch.num_workers);
      }
    }

    if (!thread_started) {
      // Mock fallback — matches Phase 5 behaviour exactly.
      auto mock = std::make_unique<rtc::mpc::MockMPCThread>();
      mock->Configure(nq, nv, /*horizon=*/10, /*dt_node=*/0.01);
      mock->SetTarget(q_curr_full_);
      mock->Init(mpc_manager_, launch);
      mock->Start();
      mpc_thread_ = std::move(mock);
      mpc_manager_.SetEnabled(true);
      RCLCPP_INFO(
          logger_, "MPC thread started (mock): core=%d prio=%d workers=%d",
          launch.main.cpu_core, launch.main.sched_priority, launch.num_workers);
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

ControllerOutput
DemoWbcController::ComputeEstop(const ControllerState &state) noexcept {
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.valid = true;
  output.command_type = command_type_;

  // Hold safe position (arm)
  auto &out0 = output.devices[0];
  out0.num_channels = state.devices[0].num_channels;
  out0.goal_type = GoalType::kJoint;
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    out0.commands[i] = safe_position_[i];
    out0.target_positions[i] = safe_position_[i];
  }

  // Hold current position (hand)
  if (state.num_devices > 1 && state.devices[1].valid) {
    auto &out1 = output.devices[1];
    out1.num_channels = state.devices[1].num_channels;
    out1.goal_type = GoalType::kJoint;
    for (std::size_t i = 0;
         i < static_cast<std::size_t>(state.devices[1].num_channels) &&
         i < kMaxDeviceChannels;
         ++i) {
      out1.commands[i] = state.devices[1].positions[i];
      out1.target_positions[i] = state.devices[1].positions[i];
    }
  }

  return output;
}

// ── Utility ──────────────────────────────────────────────────────────────────

void DemoWbcController::ExtractFullState(
    const ControllerState &state) noexcept {
  if (!joint_reorder_valid_) {
    return;
  }

  const auto &dev0 = state.devices[0];
  // Arm joints: external [0..5] → Pinocchio order
  for (int i = 0; i < kArmDof; ++i) {
    const auto eidx = static_cast<std::size_t>(i);
    const auto pq = static_cast<Eigen::Index>(ext_to_pin_q_[eidx]);
    const auto pv = static_cast<Eigen::Index>(ext_to_pin_v_[eidx]);
    q_curr_full_[pq] = dev0.positions[eidx];
    v_curr_full_[pv] = dev0.velocities[eidx];
  }

  // Hand joints: external [6..15] → Pinocchio order
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto &dev1 = state.devices[1];
    for (int i = 0; i < kHandDof; ++i) {
      const auto eidx = static_cast<std::size_t>(kArmDof + i);
      const auto pq = static_cast<Eigen::Index>(ext_to_pin_q_[eidx]);
      const auto pv = static_cast<Eigen::Index>(ext_to_pin_v_[eidx]);
      q_curr_full_[pq] = dev1.positions[static_cast<std::size_t>(i)];
      v_curr_full_[pv] = dev1.velocities[static_cast<std::size_t>(i)];
    }
  }
}

double
DemoWbcController::ComputeTcpError(const pinocchio::SE3 &target) noexcept {
  if (!arm_handle_) {
    return 1e10;
  }
  const pinocchio::SE3 tcp = arm_handle_->GetFramePlacement(tip_frame_id_);
  return (tcp.translation() - target.translation()).norm();
}

void DemoWbcController::ClampCommands(
    std::array<double, kMaxDeviceChannels> &commands, int n,
    const std::vector<double> &lower,
    const std::vector<double> &upper) noexcept {
  for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
    const double lo = (i < lower.size()) ? lower[i] : -6.2832;
    const double hi = (i < upper.size()) ? upper[i] : 6.2832;
    commands[i] = std::clamp(commands[i], lo, hi);
  }
}

} // namespace ur5e_bringup
