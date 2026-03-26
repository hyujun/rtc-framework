#include "ur5e_bringup/controllers/demo_joint_controller.hpp"

#include <algorithm>  // std::copy, std::clamp
#include <pinocchio/math/rpy.hpp>

namespace ur5e_bringup
{

DemoJointController::DemoJointController(std::string_view urdf_path)
: DemoJointController(urdf_path, Gains{}) {}

DemoJointController::DemoJointController(std::string_view urdf_path, Gains gains)
: gains_(gains), data_(pinocchio::Model{})
{
  pinocchio::urdf::buildModel(std::string(urdf_path), model_);
  data_   = pinocchio::Data(model_);
  end_id_ = static_cast<pinocchio::JointIndex>(model_.njoints - 1);
  q_      = Eigen::VectorXd::Zero(model_.nv);
}

void DemoJointController::OnDeviceConfigsSet()
{
  if (auto* cfg = GetDeviceNameConfig("ur5e"); cfg) {
    if (cfg->urdf && !cfg->urdf->tip_link.empty()) {
      if (model_.existFrame(cfg->urdf->tip_link)) {
        auto fid = model_.getFrameId(cfg->urdf->tip_link);
        end_id_ = model_.frames[fid].parentJoint;
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
}

ControllerOutput DemoJointController::Compute(const ControllerState & state) noexcept
{
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  ReadState(state);
  ComputeControl(state, dt);
  return WriteOutput(state, dt);
}

// ── Phase 1: Read joint states + sensor data ────────────────────────────────

void DemoJointController::ReadState(const ControllerState & state) noexcept
{
  // Robot arm joint positions → Eigen vector (for FK logging)
  const auto & dev0 = state.devices[0];
  const int nc0 = dev0.num_channels;
  for (int i = 0; i < nc0; ++i) {
    q_[static_cast<Eigen::Index>(i)] = dev0.positions[i];
  }

  // Hand sensor data (per-fingertip)
  num_active_fingertips_ = 0;
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto & dev1 = state.devices[1];
    const int num_sensor_ch = dev1.num_sensor_channels;
    const int num_fingertips = num_sensor_ch / rtc::kSensorValuesPerFingertip;
    num_active_fingertips_ = std::min(num_fingertips, static_cast<int>(rtc::kMaxFingertips));

    for (int f = 0; f < num_active_fingertips_; ++f) {
      auto & ft = fingertip_data_[static_cast<std::size_t>(f)];
      const int base = f * rtc::kSensorValuesPerFingertip;

      for (int j = 0; j < static_cast<int>(rtc::kBarometerCount); ++j) {
        ft.baro[static_cast<std::size_t>(j)] = dev1.sensor_data[base + j];
      }
      for (int j = 0; j < 3; ++j) {
        ft.tof[static_cast<std::size_t>(j)] =
            dev1.sensor_data[base + static_cast<int>(rtc::kBarometerCount) + j];
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
      } else {
        ft.contact_flag = 0.0f;
        ft.force = {};
        ft.displacement = {};
      }
    }
  }
}

// ── Phase 2: Compute control (trajectory + sensor-based logic) ──────────────

void DemoJointController::ComputeControl(
  const ControllerState & state, double dt) noexcept
{
  const auto & dev0 = state.devices[0];

  // ── Robot arm trajectory ────────────────────────────────────────────────
  if (robot_new_target_.load(std::memory_order_acquire)) {
    std::unique_lock lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      trajectory::JointSpaceTrajectory<kNumRobotJoints>::State start_state;
      trajectory::JointSpaceTrajectory<kNumRobotJoints>::State goal_state;

      double max_dist = 0.0;
      for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
        start_state.positions[i]     = dev0.positions[i];
        start_state.velocities[i]    = 0.0;
        start_state.accelerations[i] = 0.0;

        goal_state.positions[i]     = device_targets_[0][i];
        goal_state.velocities[i]    = 0.0;
        goal_state.accelerations[i] = 0.0;

        max_dist = std::max(max_dist,
          std::abs(device_targets_[0][i] - dev0.positions[i]));
      }

      // Duration from trajectory_speed, then enforce max trajectory velocity.
      // Quintic rest-to-rest peak velocity = (15/8) * max_dist / T.
      const double T_speed = max_dist / gains_.robot_trajectory_speed;
      const double T_vel = (gains_.robot_max_traj_velocity > 0.0)
          ? (1.875 * max_dist / gains_.robot_max_traj_velocity)
          : 0.0;
      const double duration = std::max({0.01, T_speed, T_vel});
      robot_trajectory_.initialize(start_state, goal_state, duration);
      robot_trajectory_time_ = 0.0;
      robot_new_target_.store(false, std::memory_order_relaxed);
    }
  }

  const auto robot_traj = robot_trajectory_.compute(robot_trajectory_time_);
  robot_trajectory_time_ += dt;

  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    robot_computed_.positions[i] = robot_traj.positions[i];
    robot_computed_.velocities[i] = robot_traj.velocities[i];
  }

  // ── Hand motor trajectory ──────────────────────────────────────────────
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto & dev1 = state.devices[1];

    if (hand_new_target_.load(std::memory_order_acquire)) {
      std::unique_lock lock(target_mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        trajectory::JointSpaceTrajectory<kNumHandMotors>::State start_state;
        trajectory::JointSpaceTrajectory<kNumHandMotors>::State goal_state;

        double max_dist = 0.0;
        for (std::size_t i = 0; i < kNumHandMotors; ++i) {
          start_state.positions[i]     = dev1.positions[i];
          start_state.velocities[i]    = 0.0;
          start_state.accelerations[i] = 0.0;

          goal_state.positions[i]     = device_targets_[1][i];
          goal_state.velocities[i]    = 0.0;
          goal_state.accelerations[i] = 0.0;

          max_dist = std::max(max_dist,
            std::abs(device_targets_[1][i] - dev1.positions[i]));
        }

        const double T_speed = max_dist / gains_.hand_trajectory_speed;
        const double T_vel = (gains_.hand_max_traj_velocity > 0.0)
            ? (1.875 * max_dist / gains_.hand_max_traj_velocity)
            : 0.0;
        const double duration = std::max({0.01, T_speed, T_vel});
        hand_trajectory_.initialize(start_state, goal_state, duration);
        hand_trajectory_time_ = 0.0;
        hand_new_target_.store(false, std::memory_order_relaxed);
      }
    }

    const auto hand_traj = hand_trajectory_.compute(hand_trajectory_time_);
    hand_trajectory_time_ += dt;

    for (std::size_t i = 0; i < kNumHandMotors; ++i) {
      hand_computed_.positions[i] = hand_traj.positions[i];
      hand_computed_.velocities[i] = hand_traj.velocities[i];
    }
  }

  // ── Sensor-based control logic (확장 포인트) ────────────────────────────
  // fingertip_data_[0..num_active_fingertips_-1] 에 파싱된 센서 데이터 사용 가능
  // robot_computed_, hand_computed_ 보정 가능
}

// ── Phase 3: Write output ────────────────────────────────────────────────────

ControllerOutput DemoJointController::WriteOutput(
  const ControllerState & state, double /*dt*/) noexcept
{
  ControllerOutput output;
  output.num_devices = state.num_devices;

  // ── Robot arm output ────────────────────────────────────────────────────
  const auto & dev0 = state.devices[0];
  auto & out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kJoint;

  for (int i = 0; i < nc0; ++i) {
    out0.commands[i] = robot_computed_.positions[i];
    out0.target_positions[i] = robot_computed_.positions[i];
    out0.target_velocities[i] = robot_computed_.velocities[i];
    out0.trajectory_positions[i] = robot_computed_.positions[i];
    out0.trajectory_velocities[i] = robot_computed_.velocities[i];
    out0.goal_positions[i] = device_targets_[0][i];
  }
  ClampCommands(out0.commands, nc0, device_position_lower_[0], device_position_upper_[0]);

  // ── Forward kinematics for task-space logging ──────────────────────────
  pinocchio::forwardKinematics(model_, data_, q_);
  const pinocchio::SE3 & tcp = data_.oMi[end_id_];
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp.rotation());

  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  // Joint mode: no explicit task goal from GUI, mirror FK result
  output.task_goal_positions = output.actual_task_positions;

  // ── Hand output ────────────────────────────────────────────────────────
  if (state.num_devices > 1 && state.devices[1].valid) {
    const int nc1 = state.devices[1].num_channels;
    auto & out1 = output.devices[1];
    out1.num_channels = nc1;
    out1.goal_type = GoalType::kJoint;

    for (int i = 0; i < nc1; ++i) {
      out1.commands[i] = hand_computed_.positions[i];
      out1.target_positions[i] = hand_computed_.positions[i];
      out1.target_velocities[i] = hand_computed_.velocities[i];
      out1.trajectory_positions[i] = hand_computed_.positions[i];
      out1.trajectory_velocities[i] = hand_computed_.velocities[i];
      out1.goal_positions[i] = device_targets_[1][i];
    }
    ClampCommands(out1.commands, nc1, device_position_lower_[1], device_position_upper_[1]);
  }

  output.command_type = command_type_;
  return output;
}

void DemoJointController::SetDeviceTarget(
  int device_idx, std::span<const double> target) noexcept
{
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices) return;
  const int n = std::min(static_cast<int>(target.size()), kMaxDeviceChannels);
  {
    std::lock_guard lock(target_mutex_);
    for (int i = 0; i < n; ++i) {
      device_targets_[device_idx][i] = target[i];
    }
  }
  if (device_idx == 0) {
    robot_new_target_.store(true, std::memory_order_release);
  } else if (device_idx == 1) {
    hand_new_target_.store(true, std::memory_order_release);
  }
}

void DemoJointController::InitializeHoldPosition(
  const ControllerState & state) noexcept
{
  std::lock_guard lock(target_mutex_);

  // Robot
  {
    const auto & dev0 = state.devices[0];
    trajectory::JointSpaceTrajectory<kNumRobotJoints>::State hold_state;
    for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
      device_targets_[0][i] = dev0.positions[i];
      hold_state.positions[i]     = dev0.positions[i];
      hold_state.velocities[i]    = 0.0;
      hold_state.accelerations[i] = 0.0;
    }
    robot_trajectory_.initialize(hold_state, hold_state, 0.01);
    robot_trajectory_time_ = 0.0;
    robot_new_target_.store(false, std::memory_order_relaxed);
  }

  // Hand
  for (int d = 1; d < state.num_devices; ++d) {
    const auto & dev = state.devices[d];
    if (!dev.valid) continue;
    for (int i = 0; i < dev.num_channels && i < kMaxDeviceChannels; ++i) {
      device_targets_[d][i] = dev.positions[i];
    }
    if (d == 1) {
      trajectory::JointSpaceTrajectory<kNumHandMotors>::State hold_state;
      for (std::size_t i = 0; i < kNumHandMotors; ++i) {
        hold_state.positions[i]     = dev.positions[i];
        hold_state.velocities[i]    = 0.0;
        hold_state.accelerations[i] = 0.0;
      }
      hand_trajectory_.initialize(hold_state, hold_state, 0.01);
      hand_trajectory_time_ = 0.0;
      hand_new_target_.store(false, std::memory_order_relaxed);
    }
  }
}

void DemoJointController::ClampCommands(
  std::array<double, kMaxDeviceChannels>& commands, int n,
  const std::vector<double>& lower,
  const std::vector<double>& upper) noexcept
{
  for (int i = 0; i < n; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double lo = (ui < lower.size()) ? lower[ui] : -6.2832;
    const double hi = (ui < upper.size()) ? upper[ui] :  6.2832;
    commands[i] = std::clamp(commands[i], lo, hi);
  }
}

// ── Controller registry hooks ────────────────────────────────────────────────

void DemoJointController::LoadConfig(const YAML::Node & cfg)
{
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) { return; }

  if (cfg["robot_trajectory_speed"]) {
    gains_.robot_trajectory_speed = cfg["robot_trajectory_speed"].as<double>();
  }
  if (cfg["hand_trajectory_speed"]) {
    gains_.hand_trajectory_speed = cfg["hand_trajectory_speed"].as<double>();
  }
  if (cfg["robot_max_traj_velocity"]) {
    gains_.robot_max_traj_velocity = cfg["robot_max_traj_velocity"].as<double>();
  }
  if (cfg["hand_max_traj_velocity"]) {
    gains_.hand_max_traj_velocity = cfg["hand_max_traj_velocity"].as<double>();
  }

  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

void DemoJointController::UpdateGainsFromMsg(std::span<const double> gains) noexcept
{
  // layout: [robot_trajectory_speed, hand_trajectory_speed,
  //          robot_max_traj_velocity, hand_max_traj_velocity] = 4 values
  if (gains.size() >= 1) {
    gains_.robot_trajectory_speed = gains[0];
  }
  if (gains.size() >= 2) {
    gains_.hand_trajectory_speed = gains[1];
  }
  if (gains.size() >= 3) {
    gains_.robot_max_traj_velocity = gains[2];
  }
  if (gains.size() >= 4) {
    gains_.hand_max_traj_velocity = gains[3];
  }
}

std::vector<double> DemoJointController::GetCurrentGains() const noexcept
{
  // layout: [robot_trajectory_speed, hand_trajectory_speed,
  //          robot_max_traj_velocity, hand_max_traj_velocity] = 4 values
  return {
    gains_.robot_trajectory_speed,
    gains_.hand_trajectory_speed,
    gains_.robot_max_traj_velocity,
    gains_.hand_max_traj_velocity,
  };
}

}  // namespace ur5e_bringup
