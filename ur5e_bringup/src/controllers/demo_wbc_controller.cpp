#include "ur5e_bringup/controllers/demo_wbc_controller.hpp"

#include <algorithm>
#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/tasks/posture_task.hpp"
#include "rtc_tsid/tasks/se3_task.hpp"

namespace ur5e_bringup
{

// ── Constructor ──────────────────────────────────────────────────────────────

DemoWbcController::DemoWbcController(std::string_view urdf_path)
: urdf_path_(urdf_path)
{
  // Model is built in LoadConfig() using system model config.
}

// ── Model initialization ─────────────────────────────────────────────────────

void DemoWbcController::InitModels(
  const rtc_urdf_bridge::ModelConfig & config)
{
  namespace rub = rtc_urdf_bridge;

  builder_ = std::make_unique<rub::PinocchioModelBuilder>(config);

  // Arm sub-model (6-DoF) for FK / task-space logging
  const auto primary = GetPrimaryDeviceName();
  std::string arm_model_name = "arm";
  for (const auto& sm : config.sub_models) {
    if (sm.name == primary) { arm_model_name = primary; break; }
  }
  arm_handle_ = std::make_unique<rub::RtModelHandle>(
    builder_->GetReducedModel(arm_model_name));

  // Full combined model (16-DoF) for TSID
  full_model_ptr_ = builder_->GetFullModel();

  RCLCPP_INFO(logger_, "Models initialized: arm nv=%d, full nv=%d",
    arm_handle_->nv(), full_model_ptr_->nv);
}

void DemoWbcController::BuildJointReorderMap()
{
  if (!full_model_ptr_) { return; }
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
    RCLCPP_ERROR(logger_, "Joint reorder incomplete: mapped %d/%d joints",
      ext_idx, kFullDof);
  }
}

// ── Controller registry hooks ────────────────────────────────────────────────

void DemoWbcController::LoadConfig(const YAML::Node & cfg)
{
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) { return; }

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

    // ContactManagerConfig
    if (tsid_node["contacts"]) {
      contact_mgr_config_.load(tsid_node["contacts"], model);
    }

    // PinocchioCache
    pinocchio_cache_.init(full_model_ptr_, contact_mgr_config_);

    // ContactState
    contact_state_.init(contact_mgr_config_.max_contacts);

    // ControlReference & CommandOutput pre-allocate
    control_ref_.init(robot_info_.nq, robot_info_.nv,
      robot_info_.n_actuated, contact_mgr_config_.max_contact_vars);
    tsid_output_.init(robot_info_.nv, robot_info_.n_actuated,
      contact_mgr_config_.max_contact_vars);

    // ControlState pre-allocate
    ctrl_state_.q = Eigen::VectorXd::Zero(robot_info_.nq);
    ctrl_state_.v = Eigen::VectorXd::Zero(robot_info_.nv);

    // TSIDController init (creates formulation + tasks + constraints)
    tsid_controller_.init(model, robot_info_, tsid_node);

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

  // Joint limits with safety margins
  if (cfg["integration"]) {
    position_margin_ = cfg["integration"]["position_margin"].as<double>(0.02);
    velocity_scale_ = cfg["integration"]["velocity_scale"].as<double>(0.95);
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
    max_qp_fail_before_fallback_ =
      fsm["max_qp_fail_before_fallback"].as<int>(5);
    gains_.arm_trajectory_speed =
      fsm["approach_speed"].as<double>(gains_.arm_trajectory_speed);
  }

  // ── 5. Trajectory speeds ──────────────────────────────────────────────
  if (cfg["arm_trajectory_speed"]) {
    gains_.arm_trajectory_speed = cfg["arm_trajectory_speed"].as<double>();
  }
  if (cfg["hand_trajectory_speed"]) {
    gains_.hand_trajectory_speed = cfg["hand_trajectory_speed"].as<double>();
  }
  if (cfg["arm_max_traj_velocity"]) {
    gains_.arm_max_traj_velocity = cfg["arm_max_traj_velocity"].as<double>();
  }
  if (cfg["hand_max_traj_velocity"]) {
    gains_.hand_max_traj_velocity = cfg["hand_max_traj_velocity"].as<double>();
  }

  // ── 6. Command type ───────────────────────────────────────────────────
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

void DemoWbcController::OnDeviceConfigsSet()
{
  // ── Arm frame IDs ─────────────────────────────────────────────────────
  if (auto* cfg = GetDeviceNameConfig("ur5e"); cfg) {
    if (cfg->urdf && !cfg->urdf->tip_link.empty()) {
      auto fid = arm_handle_->GetFrameId(cfg->urdf->tip_link);
      if (fid != 0) { tip_frame_id_ = fid; }
    }
    if (cfg->urdf && !cfg->urdf->root_link.empty()) {
      auto fid = arm_handle_->GetFrameId(cfg->urdf->root_link);
      if (fid != 0) { root_frame_id_ = fid; use_root_frame_ = true; }
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
  if (auto* cfg = GetDeviceNameConfig("hand"); cfg && cfg->joint_limits) {
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
  for (auto& v : device_max_velocity_) {
    if (v.empty()) v.assign(kMaxDeviceChannels, 2.0);
  }
  for (auto& v : device_position_lower_) {
    if (v.empty()) v.assign(kMaxDeviceChannels, -6.2832);
  }
  for (auto& v : device_position_upper_) {
    if (v.empty()) v.assign(kMaxDeviceChannels, 6.2832);
  }

  // ── Joint reorder map ─────────────────────────────────────────────────
  BuildJointReorderMap();
}

void DemoWbcController::UpdateGainsFromMsg(
  std::span<const double> gains) noexcept
{
  // Layout: [grasp_cmd, grasp_target_force,
  //          arm_traj_speed, hand_traj_speed,
  //          se3_weight, force_weight, posture_weight] = 7
  if (gains.size() >= 1) {
    grasp_cmd_.store(static_cast<int>(gains[0]), std::memory_order_release);
  }
  if (gains.size() >= 2) { gains_.grasp_target_force = gains[1]; }
  if (gains.size() >= 3) { gains_.arm_trajectory_speed = gains[2]; }
  if (gains.size() >= 4) { gains_.hand_trajectory_speed = gains[3]; }
  if (gains.size() >= 5) { gains_.se3_weight = gains[4]; }
  if (gains.size() >= 6) { gains_.force_weight = gains[5]; }
  if (gains.size() >= 7) { gains_.posture_weight = gains[6]; }
}

std::vector<double> DemoWbcController::GetCurrentGains() const noexcept
{
  return {
    static_cast<double>(grasp_cmd_.load(std::memory_order_relaxed)),
    gains_.grasp_target_force,
    gains_.arm_trajectory_speed,
    gains_.hand_trajectory_speed,
    gains_.se3_weight,
    gains_.force_weight,
    gains_.posture_weight
  };
}

// ── RT control loop ──────────────────────────────────────────────────────────

ControllerOutput DemoWbcController::Compute(
  const ControllerState & state) noexcept
{
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

void DemoWbcController::ReadState(
  const ControllerState & /*state*/) noexcept
{
  // Sensor data parsing for contact detection (Phase 4B).
  // Currently a no-op: contact phases are skeleton-only.
}

// ── Phase 2: Compute control ─────────────────────────────────────────────────

void DemoWbcController::ComputeControl(
  const ControllerState & state, double dt) noexcept
{
  UpdatePhase(state);

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
        ComputePositionMode(dt);  // Fallback if TSID not available
      }
      break;

    case WbcPhase::kFallback:
      ComputeFallback();
      break;
  }
}

// ── FSM ──────────────────────────────────────────────────────────────────────

void DemoWbcController::UpdatePhase(
  const ControllerState & state) noexcept
{
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
          next = WbcPhase::kIdle;  // No TSID, stay in position mode
        }
      }
      // Abort
      if (cmd == 0) { next = WbcPhase::kIdle; }
      break;
    }

    case WbcPhase::kPreGrasp: {
      // TCP close enough to goal → closure (Phase 4B)
      if (tcp_goal_valid_) {
        const double err = ComputeTcpError(tcp_goal_);
        if (err < epsilon_pregrasp_) {
          next = WbcPhase::kClosure;  // Phase 4B
        }
      }
      // Abort
      if (cmd == 0) { next = WbcPhase::kIdle; }
      break;
    }

    case WbcPhase::kClosure:
      // Phase 4B: contact detection → kHold
      // Skeleton: stay until abort
      if (cmd == 0) { next = WbcPhase::kIdle; }
      break;

    case WbcPhase::kHold:
      // Phase 4B: release command → kRetreat
      if (cmd == 2) { next = WbcPhase::kRetreat; }
      if (cmd == 0) { next = WbcPhase::kIdle; }
      break;

    case WbcPhase::kRetreat:
      // Phase 4B: trajectory complete → release
      if (robot_trajectory_time_ >= robot_trajectory_.duration()) {
        next = WbcPhase::kRelease;
      }
      if (cmd == 0) { next = WbcPhase::kIdle; }
      break;

    case WbcPhase::kRelease:
      // Phase 4B: hand open complete → idle
      if (hand_trajectory_time_ >= hand_trajectory_.duration()) {
        next = WbcPhase::kIdle;
      }
      break;

    case WbcPhase::kFallback:
      // Manual recovery only: grasp_cmd=0 → idle
      if (cmd == 0) { next = WbcPhase::kIdle; }
      break;
  }

  if (next != phase_) {
    RCLCPP_INFO_THROTTLE(logger_, log_clock_,
      ur5e_bringup::logging::kThrottleFastMs,
      "[wbc] phase %d -> %d",
      static_cast<int>(phase_), static_cast<int>(next));
    prev_phase_ = phase_;
    OnPhaseEnter(next, state);
    phase_ = next;
  }
}

void DemoWbcController::OnPhaseEnter(
  WbcPhase new_phase,
  const ControllerState & state) noexcept
{
  const auto & dev0 = state.devices[0];
  const auto & dev1 = state.devices[1];

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
      for (auto& c : contact_state_.contacts) { c.active = false; }
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
        goal.positions[i] = device_targets_[0][i];
        const double delta = std::abs(goal.positions[i] - start.positions[i]);
        if (delta > max_delta) { max_delta = delta; }
      }
      const double duration = std::max(
        max_delta / gains_.arm_trajectory_speed, 0.1);
      robot_trajectory_.initialize(start, goal, duration);
      robot_trajectory_time_ = 0.0;
      robot_new_target_.store(false, std::memory_order_relaxed);

      // Compute FK of arm target for SE3Task reference in kPreGrasp
      if (arm_handle_) {
        std::span<const double> q_target(
          device_targets_[0].data(), kNumRobotJoints);
        arm_handle_->ComputeForwardKinematics(q_target);
        tcp_goal_ = arm_handle_->GetFramePlacement(tip_frame_id_);
        if (use_root_frame_) {
          tcp_goal_ = arm_handle_->GetFramePlacement(
            root_frame_id_).actInv(tcp_goal_);
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
          if (hd > hmax) { hmax = hd; }
        }
        const double hdur = std::max(
          hmax / gains_.hand_trajectory_speed, 0.1);
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
        auto* se3_task = tsid_controller_.formulation().get_task("se3_tcp");
        if (se3_task) {
          static_cast<rtc::tsid::SE3Task*>(se3_task)->set_se3_reference(
            tcp_goal_);
        }
      }

      // Contact activation (Phase 4B)
      if (new_phase == WbcPhase::kClosure || new_phase == WbcPhase::kHold) {
        for (auto& c : contact_state_.contacts) { c.active = true; }
        contact_state_.recompute_active(contact_mgr_config_);
      }

      qp_fail_count_ = 0;
      break;
    }

    case WbcPhase::kRetreat: {
      // Deactivate contacts
      for (auto& c : contact_state_.contacts) { c.active = false; }
      contact_state_.recompute_active(contact_mgr_config_);
      // Phase 4B: build retreat trajectory
      break;
    }

    case WbcPhase::kRelease: {
      // Phase 4B: hand open trajectory
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
      for (auto& c : contact_state_.contacts) { c.active = false; }
      contact_state_.recompute_active(contact_mgr_config_);
      qp_fail_count_ = 0;
      break;
    }
  }
}

// ── Control modes ────────────────────────────────────────────────────────────

void DemoWbcController::ComputePositionMode(double dt) noexcept
{
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

void DemoWbcController::ComputeTSIDPosition(
  const ControllerState & state, double dt) noexcept
{
  // 1. Extract full state (sensor values, every tick)
  ExtractFullState(state);

  // 2. Update Pinocchio cache (M, h, g, Jacobians)
  pinocchio_cache_.update(q_curr_full_, v_curr_full_, contact_state_);

  // 3. Set posture reference (regularization toward current target)
  control_ref_.q_des = q_curr_full_;
  control_ref_.v_des.setZero();
  control_ref_.a_des.setZero();

  // 4. Build ControlState
  ctrl_state_.q = q_curr_full_;
  ctrl_state_.v = v_curr_full_;
  ctrl_state_.timestamp_ns = state.iteration;

  // 5. TSID solve
  tsid_output_ = tsid_controller_.compute(
    ctrl_state_, control_ref_, pinocchio_cache_, contact_state_);

  // 6. QP failure handling
  if (!tsid_output_.qp_converged) {
    ++qp_fail_count_;
    RCLCPP_WARN_THROTTLE(logger_, log_clock_,
      ur5e_bringup::logging::kThrottleSlowMs,
      "[wbc] QP failed (%d/%d), solve=%.0fus",
      qp_fail_count_, max_qp_fail_before_fallback_,
      tsid_output_.solve_time_us);

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
  const auto& a = tsid_output_.a_opt;

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
    const auto pin_idx = static_cast<std::size_t>(
      ext_to_pin_q_[static_cast<std::size_t>(i)]);
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

void DemoWbcController::ComputeFallback() noexcept
{
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

ControllerOutput DemoWbcController::WriteOutput(
  const ControllerState & state) noexcept
{
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.command_type = command_type_;

  // ── Robot arm output ──────────────────────────────────────────────────
  const auto & dev0 = state.devices[0];
  auto & out0 = output.devices[0];
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
  ClampCommands(out0.commands, nc0,
    device_position_lower_[0], device_position_upper_[0]);

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
      Eigen::Vector3d grpy = pinocchio::rpy::matrixToRpy(
        tcp_goal_.rotation());
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
    auto & out1 = output.devices[1];
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
    ClampCommands(out1.commands, nc1,
      device_position_lower_[1], device_position_upper_[1]);
  }

  output.valid = true;
  return output;
}

// ── Target management ────────────────────────────────────────────────────────

void DemoWbcController::SetDeviceTarget(
  int device_idx, std::span<const double> target) noexcept
{
  std::lock_guard lock(target_mutex_);
  const auto didx = static_cast<std::size_t>(device_idx);
  if (didx >= device_targets_.size()) { return; }

  const auto n = std::min(target.size(),
    static_cast<std::size_t>(kMaxDeviceChannels));
  std::copy_n(target.data(), n, device_targets_[didx].data());

  if (device_idx == 0) {
    robot_new_target_.store(true, std::memory_order_release);
  } else if (device_idx == 1) {
    hand_new_target_.store(true, std::memory_order_release);
  }
}

void DemoWbcController::InitializeHoldPosition(
  const ControllerState & state) noexcept
{
  std::lock_guard lock(target_mutex_);

  // Robot arm: initialize trajectory at current position (zero velocity)
  {
    const auto & dev0 = state.devices[0];
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
    const auto & dev = state.devices[d];
    if (!dev.valid) continue;
    for (std::size_t i = 0;
         i < static_cast<std::size_t>(dev.num_channels) &&
         i < kMaxDeviceChannels; ++i) {
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
}

// ── E-STOP ───────────────────────────────────────────────────────────────────

void DemoWbcController::TriggerEstop() noexcept
{
  estopped_.store(true, std::memory_order_release);
}

void DemoWbcController::ClearEstop() noexcept
{
  estopped_.store(false, std::memory_order_release);
}

bool DemoWbcController::IsEstopped() const noexcept
{
  return estopped_.load(std::memory_order_acquire);
}

void DemoWbcController::SetHandEstop(bool active) noexcept
{
  hand_estopped_.store(active, std::memory_order_release);
}

ControllerOutput DemoWbcController::ComputeEstop(
  const ControllerState & state) noexcept
{
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.valid = true;
  output.command_type = command_type_;

  // Hold safe position (arm)
  auto & out0 = output.devices[0];
  out0.num_channels = state.devices[0].num_channels;
  out0.goal_type = GoalType::kJoint;
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    out0.commands[i] = kSafePosition[i];
    out0.target_positions[i] = kSafePosition[i];
  }

  // Hold current position (hand)
  if (state.num_devices > 1 && state.devices[1].valid) {
    auto & out1 = output.devices[1];
    out1.num_channels = state.devices[1].num_channels;
    out1.goal_type = GoalType::kJoint;
    for (std::size_t i = 0;
         i < static_cast<std::size_t>(state.devices[1].num_channels) &&
         i < kMaxDeviceChannels; ++i) {
      out1.commands[i] = state.devices[1].positions[i];
      out1.target_positions[i] = state.devices[1].positions[i];
    }
  }

  return output;
}

// ── Utility ──────────────────────────────────────────────────────────────────

void DemoWbcController::ExtractFullState(
  const ControllerState & state) noexcept
{
  if (!joint_reorder_valid_) { return; }

  const auto & dev0 = state.devices[0];
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
    const auto & dev1 = state.devices[1];
    for (int i = 0; i < kHandDof; ++i) {
      const auto eidx = static_cast<std::size_t>(kArmDof + i);
      const auto pq = static_cast<Eigen::Index>(ext_to_pin_q_[eidx]);
      const auto pv = static_cast<Eigen::Index>(ext_to_pin_v_[eidx]);
      q_curr_full_[pq] = dev1.positions[static_cast<std::size_t>(i)];
      v_curr_full_[pv] = dev1.velocities[static_cast<std::size_t>(i)];
    }
  }
}

double DemoWbcController::ComputeTcpError(
  const pinocchio::SE3 & target) noexcept
{
  if (!arm_handle_) { return 1e10; }
  const pinocchio::SE3 tcp = arm_handle_->GetFramePlacement(tip_frame_id_);
  return (tcp.translation() - target.translation()).norm();
}

void DemoWbcController::ClampCommands(
  std::array<double, kMaxDeviceChannels>& commands, int n,
  const std::vector<double>& lower,
  const std::vector<double>& upper) noexcept
{
  for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
    const double lo = (i < lower.size()) ? lower[i] : -6.2832;
    const double hi = (i < upper.size()) ? upper[i] :  6.2832;
    commands[i] = std::clamp(commands[i], lo, hi);
  }
}

}  // namespace ur5e_bringup
