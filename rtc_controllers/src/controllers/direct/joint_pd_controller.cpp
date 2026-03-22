#include "rtc_controllers/direct/joint_pd_controller.hpp"

#include <algorithm>
#include <cstddef>
#include <pinocchio/math/rpy.hpp>

namespace rtc
{

// ── Constructors ─────────────────────────────────────────────────────────────

JointPDController::JointPDController(std::string_view urdf_path)
: JointPDController(urdf_path, Gains{}) {}

JointPDController::JointPDController(
  std::string_view urdf_path,
  Gains gains)
: data_(pinocchio::Model{}), gains_(gains)
{
  pinocchio::urdf::buildModel(std::string(urdf_path), model_);
  data_ = pinocchio::Data(model_);

  q_ = pinocchio::neutral(model_);
  v_ = Eigen::VectorXd::Zero(model_.nv);
  coriolis_forces_ = Eigen::VectorXd::Zero(model_.nv);
  jacobian_ = Eigen::MatrixXd::Zero(6, model_.nv);

  trajectory_.initialize({}, {}, 0.0);
}

void JointPDController::OnDeviceConfigsSet()
{
  if (auto* cfg = GetDeviceNameConfig("ur5e"); cfg && cfg->joint_limits) {
    max_joint_velocity_ = cfg->joint_limits->max_velocity;
    max_joint_torque_   = cfg->joint_limits->max_torque;
  }
  if (max_joint_velocity_.empty()) {
    max_joint_velocity_.assign(kMaxDeviceChannels, 2.0);
  }
  if (max_joint_torque_.empty()) {
    max_joint_torque_.assign(kMaxDeviceChannels, 150.0);
  }
}

// ── RTControllerInterface implementation ─────────────────────────────────────

ControllerOutput JointPDController::Compute(
  const ControllerState & state) noexcept
{
  if (estopped_.load(std::memory_order_acquire)) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    return out;
  }

  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  const auto & dev0 = state.devices[0];

  UpdateDynamics(dev0);

  if (new_target_.load(std::memory_order_acquire)) {
    std::unique_lock lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      trajectory::JointSpaceTrajectory<kNumRobotJoints>::State start_state;
      trajectory::JointSpaceTrajectory<kNumRobotJoints>::State goal_state;

      double max_dist = 0.0;
      for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
        start_state.positions[i]     = dev0.positions[i];
        start_state.velocities[i]    = dev0.velocities[i];
        start_state.accelerations[i] = 0.0;

        goal_state.positions[i]     = device_targets_[0][i];
        goal_state.velocities[i]    = 0.0;
        goal_state.accelerations[i] = 0.0;

        max_dist = std::max(max_dist,
          std::abs(device_targets_[0][i] - dev0.positions[i]));
      }

      const double duration = std::max(0.01, max_dist / gains_.trajectory_speed);
      trajectory_.initialize(start_state, goal_state, duration);
      trajectory_time_ = 0.0;
      new_target_.store(false, std::memory_order_relaxed);
    }
  }

  const auto traj_state = trajectory_.compute(trajectory_time_);
  trajectory_time_ += dt;

  // PD control + feedforward velocity + optional gravity/Coriolis compensation
  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto & out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;

  for (int i = 0; i < nc0; ++i) {
    const double e  = traj_state.positions[i] - dev0.positions[i];
    const double de = (e - prev_error_[i]) / dt;

    out0.commands[i] = gains_.kp[i] * e + gains_.kd[i] * de;
    if (command_type_ != CommandType::kTorque) {
      out0.commands[i] += traj_state.velocities[i];
    }

    if (gains_.enable_gravity_compensation) {
      out0.commands[i] += gravity_torques_[i];
    }
    if (gains_.enable_coriolis_compensation) {
      out0.commands[i] +=
        coriolis_forces_[static_cast<Eigen::Index>(i)];
    }

    prev_error_[i] = e;
  }

  for (int i = 0; i < nc0; ++i) {
    out0.target_positions[i] = traj_state.positions[i];
    out0.goal_positions[i] = device_targets_[0][i];
    out0.target_velocities[i] = traj_state.velocities[i];
  }

  // Device 1 (hand): pass-through goals
  if (state.num_devices > 1) {
    const int nc1 = state.devices[1].num_channels;
    auto & out1 = output.devices[1];
    out1.num_channels = nc1;
    for (int i = 0; i < nc1; ++i) {
      out1.goal_positions[i] = device_targets_[1][i];
    }
  }

  ClampCommands(out0.commands, nc0, command_type_);

  // TCP pose output (task_positions: [x, y, z, roll, pitch, yaw])
  const auto last_joint =
    static_cast<pinocchio::JointIndex>(model_.njoints - 1);
  const pinocchio::SE3 & tcp = data_.oMi[last_joint];
  const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp.rotation());
  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  output.command_type = command_type_;
  return output;
}

void JointPDController::SetDeviceTarget(
  int device_idx, std::span<const double> target) noexcept
{
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices) return;
  const int n = std::min(static_cast<int>(target.size()), kMaxDeviceChannels);
  if (device_idx == 0) {
    std::lock_guard lock(target_mutex_);
    for (int i = 0; i < n; ++i) {
      device_targets_[0][i] = target[i];
    }
    new_target_.store(true, std::memory_order_release);
  } else {
    for (int i = 0; i < n; ++i) {
      device_targets_[device_idx][i] = target[i];
    }
  }
}

void JointPDController::InitializeHoldPosition(
  const ControllerState & state) noexcept
{
  const auto & dev0 = state.devices[0];
  std::lock_guard lock(target_mutex_);
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    device_targets_[0][i] = dev0.positions[i];
  }
  trajectory::JointSpaceTrajectory<kNumRobotJoints>::State hold_state;
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    hold_state.positions[i]     = dev0.positions[i];
    hold_state.velocities[i]    = 0.0;
    hold_state.accelerations[i] = 0.0;
  }
  trajectory_.initialize(hold_state, hold_state, 0.01);
  trajectory_time_ = 0.0;
  prev_error_ = {};
  new_target_.store(false, std::memory_order_relaxed);

  for (int d = 1; d < state.num_devices; ++d) {
    const auto & dev = state.devices[d];
    if (!dev.valid) continue;
    for (int i = 0; i < dev.num_channels && i < kMaxDeviceChannels; ++i) {
      device_targets_[d][i] = dev.positions[i];
    }
  }
}

// ── E-STOP ────────────────────────────────────────────────────────────────────

void JointPDController::TriggerEstop() noexcept
{
  estopped_.store(true, std::memory_order_release);
}

void JointPDController::ClearEstop() noexcept
{
  estopped_.store(false, std::memory_order_release);
  prev_error_ = {};
  new_target_.store(true, std::memory_order_relaxed);
}

bool JointPDController::IsEstopped() const noexcept
{
  return estopped_.load(std::memory_order_acquire);
}

void JointPDController::SetHandEstop(bool active) noexcept
{
  hand_estopped_.store(active, std::memory_order_release);
}

// ── Diagnostic accessors ─────────────────────────────────────────────────────

std::array<double, kNumRobotJoints>
JointPDController::gravity_torques() const noexcept
{
  return gravity_torques_;
}

std::array<double, 3> JointPDController::tcp_position() const noexcept
{
  return tcp_position_;
}

// ── Controller registry hooks ────────────────────────────────────────────────

void JointPDController::LoadConfig(const YAML::Node & cfg)
{
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) {return;}

  if (cfg["kp"] && cfg["kp"].IsSequence() && cfg["kp"].size() == 6) {
    for (std::size_t i = 0; i < 6; ++i) {
      gains_.kp[i] = cfg["kp"][i].as<double>();
    }
  }
  if (cfg["kd"] && cfg["kd"].IsSequence() && cfg["kd"].size() == 6) {
    for (std::size_t i = 0; i < 6; ++i) {
      gains_.kd[i] = cfg["kd"][i].as<double>();
    }
  }
  if (cfg["enable_gravity_compensation"]) {
    gains_.enable_gravity_compensation =
      cfg["enable_gravity_compensation"].as<bool>();
  }
  if (cfg["enable_coriolis_compensation"]) {
    gains_.enable_coriolis_compensation =
      cfg["enable_coriolis_compensation"].as<bool>();
  }
  if (cfg["trajectory_speed"]) {
    gains_.trajectory_speed = cfg["trajectory_speed"].as<double>();
  }
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

void JointPDController::UpdateGainsFromMsg(
  std::span<const double> gains) noexcept
{
  // layout: [kp×6, kd×6, enable_gravity(0/1), enable_coriolis(0/1), trajectory_speed]
  if (gains.size() < 14) {return;}

  for (std::size_t i = 0; i < 6; ++i) {gains_.kp[i] = gains[i];}
  for (std::size_t i = 0; i < 6; ++i) {gains_.kd[i] = gains[6 + i];}
  gains_.enable_gravity_compensation  = gains[12] > 0.5;
  gains_.enable_coriolis_compensation = gains[13] > 0.5;
  if (gains.size() >= 15) {gains_.trajectory_speed = gains[14];}
}

std::vector<double> JointPDController::GetCurrentGains() const noexcept
{
  // layout: [kp×6, kd×6, enable_gravity(0/1), enable_coriolis(0/1), trajectory_speed]
  std::vector<double> v;
  v.reserve(15);
  v.insert(v.end(), gains_.kp.begin(), gains_.kp.end());
  v.insert(v.end(), gains_.kd.begin(), gains_.kd.end());
  v.push_back(gains_.enable_gravity_compensation ? 1.0 : 0.0);
  v.push_back(gains_.enable_coriolis_compensation ? 1.0 : 0.0);
  v.push_back(gains_.trajectory_speed);
  return v;
}

// ── Internal helpers ─────────────────────────────────────────────────────────

ControllerOutput JointPDController::ComputeEstop(
  const ControllerState & state) noexcept
{
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  const auto & dev0 = state.devices[0];

  ControllerOutput output;
  output.num_devices = state.num_devices;
  auto & out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;

  for (int i = 0; i < nc0; ++i) {
    const double e  = kSafePosition[i] - dev0.positions[i];
    const double de = (e - prev_error_[i]) / dt;
    out0.commands[i] = gains_.kp[i] * e + gains_.kd[i] * de;
    prev_error_[i] = e;
  }

  for (int i = 0; i < nc0; ++i) {
    out0.target_positions[i] = kSafePosition[i];
    out0.goal_positions[i] = kSafePosition[i];
  }
  ClampCommands(out0.commands, nc0, command_type_);
  new_target_.store(true, std::memory_order_relaxed);
  return output;
}

void JointPDController::UpdateDynamics(const DeviceState & dev) noexcept
{
  const std::size_t nv = static_cast<std::size_t>(model_.nv);
  const std::size_t n  = std::min(static_cast<std::size_t>(kNumRobotJoints), nv);

  for (std::size_t i = 0; i < n; ++i) {
    q_[static_cast<Eigen::Index>(i)] = dev.positions[i];
    v_[static_cast<Eigen::Index>(i)] = dev.velocities[i];
  }

  // ── Gravity torque g(q) ─────────────────────────────────────────────────
  if (gains_.enable_gravity_compensation) {
    const Eigen::VectorXd & g =
      pinocchio::computeGeneralizedGravity(model_, data_, q_);
    for (std::size_t i = 0; i < n; ++i) {
      gravity_torques_[i] = g[static_cast<Eigen::Index>(i)];
    }
  }

  // ── Forward kinematics (FK) ──────────────────────────────────────────────
  pinocchio::forwardKinematics(model_, data_, q_, v_);

  // ── TCP position cache ───────────────────────────────────────────────────
  const auto last_joint =
    static_cast<pinocchio::JointIndex>(model_.njoints - 1);
  const pinocchio::SE3 & tcp = data_.oMi[last_joint];
  tcp_position_[0] = tcp.translation()[0];
  tcp_position_[1] = tcp.translation()[1];
  tcp_position_[2] = tcp.translation()[2];

  // ── Jacobian ─────────────────────────────────────────────────────────────
  pinocchio::computeJointJacobian(model_, data_, q_, last_joint, jacobian_);

  // ── Coriolis/centrifugal C(q,v)·v ────────────────────────────────────────
  if (gains_.enable_coriolis_compensation) {
    const Eigen::MatrixXd & C =
      pinocchio::computeCoriolisMatrix(model_, data_, q_, v_);
    coriolis_forces_.noalias() = C * v_;
  }
}

void JointPDController::ClampCommands(
  std::array<double, kMaxDeviceChannels>& cmds, int n, CommandType type) const noexcept
{
  const auto& limits = (type == CommandType::kTorque)
                            ? max_joint_torque_ : max_joint_velocity_;
  for (int i = 0; i < n; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double lim = (ui < limits.size()) ? limits[ui] : 2.0;
    cmds[i] = std::clamp(cmds[i], -lim, lim);
  }
}

}  // namespace rtc
