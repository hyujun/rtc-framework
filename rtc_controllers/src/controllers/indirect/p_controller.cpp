#include "rtc_controllers/indirect/p_controller.hpp"

#include <algorithm> // std::copy, std::clamp

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

namespace rtc
{

PController::PController(std::string_view urdf_path)
: PController(urdf_path, Gains{}) {}

PController::PController(std::string_view urdf_path, Gains gains)
: gains_(gains)
{
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = std::string(urdf_path);
  config.root_joint_type = "fixed";

  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  model_ptr_ = builder.GetFullModel();
  handle_ = std::make_unique<rtc_urdf_bridge::RtModelHandle>(model_ptr_);

  // Default: use the last frame in the model as tip
  tip_frame_id_ = static_cast<pinocchio::FrameIndex>(model_ptr_->nframes - 1);
}

void PController::OnDeviceConfigsSet()
{
  const auto primary = GetPrimaryDeviceName();
  if (auto* cfg = GetDeviceNameConfig(primary); cfg) {
    if (cfg->urdf && !cfg->urdf->tip_link.empty()) {
      auto fid = handle_->GetFrameId(cfg->urdf->tip_link);
      if (fid != 0) {
        tip_frame_id_ = fid;
      }
    }
    if (cfg->joint_limits) {
      max_joint_velocity_ = cfg->joint_limits->max_velocity;
    }
  }
  if (max_joint_velocity_.empty()) {
    max_joint_velocity_.assign(kMaxDeviceChannels, kDefaultMaxJointVelocity);
  }
}

ControllerOutput PController::Compute(const ControllerState & state) noexcept
{
  if (estopped_.load(std::memory_order_acquire)) {
    // E-STOP: hold current position, zero velocity
    ControllerOutput output{};
    output.valid = true;
    output.command_type = command_type_;
    for (std::size_t d = 0; d < static_cast<std::size_t>(state.num_devices); ++d) {
      for (std::size_t j = 0; j < static_cast<std::size_t>(state.devices[d].num_channels); ++j) {
        output.devices[d].commands[j] = state.devices[d].positions[j];
        device_targets_[d][j] = state.devices[d].positions[j];
      }
    }
    return output;
  }

  ControllerOutput output;
  output.num_devices = state.num_devices;

  // Device 0 (robot arm): P control
  const auto & dev0 = state.devices[0];
  auto & out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;

  // Build q span for FK
  std::array<double, kMaxDeviceChannels> q_buf{};
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    const double error = device_targets_[0][i] - dev0.positions[i];
    out0.commands[i] = dev0.positions[i] + gains_.kp[i] * error * state.dt;
    q_buf[i] = dev0.positions[i];
  }

  const auto nv = handle_->nv();
  std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv));
  handle_->ComputeForwardKinematics(q_span);
  const pinocchio::SE3 & tcp = handle_->GetFramePlacement(tip_frame_id_);
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp.rotation());

  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.target_positions[i] = device_targets_[0][i];
    out0.target_velocities[i] = gains_.kp[i] * (device_targets_[0][i] - dev0.positions[i]);
    out0.goal_positions[i] = device_targets_[0][i];
  }

  // Device 1+ : pass-through goals
  for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices); ++d) {
    const auto & devN = state.devices[d];
    auto & outN = output.devices[d];
    const int ncN = devN.num_channels;
    outN.num_channels = ncN;
    for (std::size_t i = 0; i < static_cast<std::size_t>(ncN); ++i) {
      outN.commands[i] = device_targets_[d][i];
      outN.target_positions[i] = device_targets_[d][i];
      outN.goal_positions[i] = device_targets_[d][i];
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
  const auto ud = static_cast<std::size_t>(device_idx);
  const std::size_t n = std::min(target.size(), static_cast<std::size_t>(kMaxDeviceChannels));
  for (std::size_t i = 0; i < n; ++i) {
    device_targets_[ud][i] = target[i];
  }
}

void PController::InitializeHoldPosition(
  const ControllerState & state) noexcept
{
  for (std::size_t d = 0; d < static_cast<std::size_t>(state.num_devices); ++d) {
    const auto & dev = state.devices[d];
    if (!dev.valid && d > 0) continue;
    for (std::size_t i = 0; i < static_cast<std::size_t>(dev.num_channels) && i < static_cast<std::size_t>(kMaxDeviceChannels); ++i) {
      device_targets_[d][i] = dev.positions[i];
    }
  }
}

void PController::ClampCommands(
  std::array<double, kMaxDeviceChannels>& commands, int n) const noexcept
{
  for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
    const double lim = (i < max_joint_velocity_.size()) ? max_joint_velocity_[i] : kDefaultMaxJointVelocity;
    commands[i] = std::clamp(commands[i], -lim, lim);
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
