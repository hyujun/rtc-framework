#include "rtc_controllers/indirect/p_controller.hpp"

#include <algorithm> // std::copy, std::clamp
#include <pinocchio/math/rpy.hpp>

namespace rtc
{

PController::PController(std::string_view urdf_path)
: PController(urdf_path, Gains{}) {}

PController::PController(std::string_view urdf_path, Gains gains)
: gains_(gains), data_(pinocchio::Model{})
{
  pinocchio::urdf::buildModel(std::string(urdf_path), model_);
  data_ = pinocchio::Data(model_);
  end_id_ = static_cast<pinocchio::JointIndex>(model_.njoints - 1);
  q_ = Eigen::VectorXd::Zero(model_.nv);
}

ControllerOutput PController::Compute(const ControllerState & state) noexcept
{
  ControllerOutput output;
  output.num_devices = state.num_devices;

  // Device 0 (robot arm): P control
  const auto & dev0 = state.devices[0];
  auto & out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;

  for (int i = 0; i < nc0; ++i) {
    const double error = device_targets_[0][i] - dev0.positions[i];
    out0.commands[i] = dev0.positions[i] + gains_.kp[i] * error * state.dt;
    q_[static_cast<Eigen::Index>(i)] = dev0.positions[i];
  }

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
    out0.goal_positions[i] = device_targets_[0][i];
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

  ClampCommands(out0.commands, nc0);
  output.command_type = command_type_;
  return output;
}

void PController::SetDeviceTarget(
  int device_idx, std::span<const double> target) noexcept
{
  if (device_idx < 0 || device_idx >= ControllerState::kMaxDevices) return;
  const int n = std::min(static_cast<int>(target.size()), kMaxDeviceChannels);
  for (int i = 0; i < n; ++i) {
    device_targets_[device_idx][i] = target[i];
  }
}

void PController::InitializeHoldPosition(
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

void PController::ClampCommands(
  std::array<double, kMaxDeviceChannels>& commands, int n) noexcept
{
  for (int i = 0; i < n; ++i) {
    commands[i] = std::clamp(commands[i], -kMaxJointVelocity, kMaxJointVelocity);
  }
}

// ── Controller registry hooks ────────────────────────────────────────────────

void PController::LoadConfig(const YAML::Node & cfg)
{
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) {return;}
  if (cfg["kp"] && cfg["kp"].IsSequence() && cfg["kp"].size() == 6) {
    for (std::size_t i = 0; i < 6; ++i) {
      gains_.kp[i] = cfg["kp"][i].as<double>();
    }
  }
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

void PController::UpdateGainsFromMsg(std::span<const double> gains) noexcept
{
  // layout: [kp×6]
  if (gains.size() < 6) {return;}
  for (std::size_t i = 0; i < 6; ++i) {
    gains_.kp[i] = gains[i];
  }
}

std::vector<double> PController::GetCurrentGains() const noexcept
{
  // layout: [kp×6]
  return {gains_.kp.begin(), gains_.kp.end()};
}

}  // namespace rtc
