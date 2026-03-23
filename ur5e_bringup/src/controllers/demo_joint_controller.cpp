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
    if (cfg->joint_limits && !cfg->joint_limits->max_velocity.empty()) {
      device_max_velocity_[0] = cfg->joint_limits->max_velocity;
    }
  }
  if (auto* cfg = GetDeviceNameConfig("hand"); cfg && cfg->joint_limits) {
    if (!cfg->joint_limits->max_velocity.empty()) {
      device_max_velocity_[1] = cfg->joint_limits->max_velocity;
    }
  }
  // Fallback defaults
  for (auto& v : device_max_velocity_) {
    if (v.empty()) v.assign(kMaxDeviceChannels, 2.0);
  }
}

ControllerOutput DemoJointController::Compute(const ControllerState & state) noexcept
{
  ControllerOutput output;
  output.num_devices = state.num_devices;

  const auto & dev0 = state.devices[0];
  auto & out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;
  out0.goal_type = GoalType::kJoint;

  // ── Robot arm P control (identical to PController) ────────────────────────
  for (int i = 0; i < nc0; ++i) {
    const double error = device_targets_[0][i] - dev0.positions[i];
    out0.commands[i] =
      dev0.positions[i] + gains_.robot_kp[i] * error * state.dt;
    q_[static_cast<Eigen::Index>(i)] = dev0.positions[i];
  }
  ClampCommands(out0.commands, nc0, device_max_velocity_[0]);

  // ── Forward kinematics for task-space logging ─────────────────────────────
  pinocchio::forwardKinematics(model_, data_, q_);
  const pinocchio::SE3 & tcp = data_.oMi[end_id_];
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp.rotation());

  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  for (int i = 0; i < nc0; ++i) {
    out0.target_positions[i] = device_targets_[0][i];
    out0.target_velocities[i] = gains_.robot_kp[i] * (device_targets_[0][i] - dev0.positions[i]);
    out0.goal_positions[i] = device_targets_[0][i];
  }

  // ── Hand motor P control (same formula applied to hand motors) ─────────
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto & dev1 = state.devices[1];
    const int nc1 = dev1.num_channels;
    auto & out1 = output.devices[1];
    out1.num_channels = nc1;
    out1.goal_type = GoalType::kJoint;
    for (int i = 0; i < nc1; ++i) {
      const double error = device_targets_[1][i] - dev1.positions[i];
      out1.commands[i] =
        dev1.positions[i] +
        static_cast<double>(gains_.hand_kp[i]) * error * state.dt;
      out1.target_positions[i] = device_targets_[1][i];
      out1.target_velocities[i] = static_cast<double>(gains_.hand_kp[i]) * error;
      out1.goal_positions[i] = device_targets_[1][i];
    }
    ClampCommands(out1.commands, nc1, device_max_velocity_[1]);

    // ── Hand sensor data (per-fingertip) ──────────────────────────────────
    const int num_sensor_ch = dev1.num_sensor_channels;
    const int num_fingertips = num_sensor_ch / rtc::kSensorValuesPerFingertip;
    for (int f = 0; f < num_fingertips && f < rtc::kMaxFingertips; ++f) {
      const int base = f * rtc::kSensorValuesPerFingertip;
      const int32_t * baro = &dev1.sensor_data[base];
      const int32_t * tof  = &dev1.sensor_data[base + rtc::kBarometerCount];

      std::array<float, 3> F{};
      std::array<float, 3> u{};
      float contact_flag = 0.0f;

      if (dev1.inference_enable[static_cast<std::size_t>(f)]) {
        const int ft_base = f * rtc::kFTValuesPerFingertip;
        contact_flag = dev1.inference_data[static_cast<std::size_t>(ft_base)];
        for (int j = 0; j < 3; ++j) {
          F[static_cast<std::size_t>(j)] =
              dev1.inference_data[static_cast<std::size_t>(ft_base + 1 + j)];
          u[static_cast<std::size_t>(j)] =
              dev1.inference_data[static_cast<std::size_t>(ft_base + 4 + j)];
        }
      }

      // baro, tof, F, u, contact_flag available for control logic
      static_cast<void>(baro);
      static_cast<void>(tof);
      static_cast<void>(F);
      static_cast<void>(u);
      static_cast<void>(contact_flag);
    }
  }

  output.command_type = command_type_;
  return output;
}

void DemoJointController::SetDeviceTarget(
  int device_idx, std::span<const double> target) noexcept
{
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices) return;
  const int n = std::min(static_cast<int>(target.size()), kMaxDeviceChannels);
  for (int i = 0; i < n; ++i) {
    device_targets_[device_idx][i] = target[i];
  }
}

void DemoJointController::InitializeHoldPosition(
  const ControllerState & state) noexcept
{
  for (int d = 0; d < state.num_devices; ++d) {
    const auto & dev = state.devices[d];
    if (!dev.valid && d > 0) continue;
    for (int i = 0; i < dev.num_channels && i < kMaxDeviceChannels; ++i) {
      device_targets_[d][i] = dev.positions[i];
    }
  }
}

void DemoJointController::ClampCommands(
  std::array<double, kMaxDeviceChannels>& commands, int n,
  const std::vector<double>& limits) noexcept
{
  for (int i = 0; i < n; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double lim = (ui < limits.size()) ? limits[ui] : 2.0;
    commands[i] = std::clamp(commands[i], -lim, lim);
  }
}

// ── Controller registry hooks ────────────────────────────────────────────────

void DemoJointController::LoadConfig(const YAML::Node & cfg)
{
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) { return; }

  if (cfg["robot_kp"] && cfg["robot_kp"].IsSequence() &&
      cfg["robot_kp"].size() == static_cast<std::size_t>(kNumRobotJoints))
  {
    for (std::size_t i = 0; i < static_cast<std::size_t>(kNumRobotJoints); ++i) {
      gains_.robot_kp[i] = cfg["robot_kp"][i].as<double>();
    }
  }

  if (cfg["hand_kp"] && cfg["hand_kp"].IsSequence() &&
      cfg["hand_kp"].size() == static_cast<std::size_t>(kNumHandMotors))
  {
    for (std::size_t i = 0; i < static_cast<std::size_t>(kNumHandMotors); ++i) {
      gains_.hand_kp[i] = cfg["hand_kp"][i].as<float>();
    }
  }

  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

void DemoJointController::UpdateGainsFromMsg(std::span<const double> gains) noexcept
{
  // layout: [robot_kp×6, hand_kp×10] = 16 values
  constexpr std::size_t kRobot = static_cast<std::size_t>(kNumRobotJoints);
  constexpr std::size_t kHand  = static_cast<std::size_t>(kNumHandMotors);

  if (gains.size() < kRobot) { return; }
  for (std::size_t i = 0; i < kRobot; ++i) {
    gains_.robot_kp[i] = gains[i];
  }
  if (gains.size() >= kRobot + kHand) {
    for (std::size_t i = 0; i < kHand; ++i) {
      gains_.hand_kp[i] = static_cast<float>(gains[kRobot + i]);
    }
  }
}

std::vector<double> DemoJointController::GetCurrentGains() const noexcept
{
  // layout: [robot_kp×6, hand_kp×10] = 16 values
  std::vector<double> out;
  out.reserve(static_cast<std::size_t>(kNumRobotJoints + kNumHandMotors));
  for (const double v : gains_.robot_kp) { out.push_back(v); }
  for (const float  v : gains_.hand_kp)  { out.push_back(static_cast<double>(v)); }
  return out;
}

}  // namespace ur5e_bringup
