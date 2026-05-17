#include "rtc_controllers/indirect/p_controller.hpp"

#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_base/utils/device_passthrough.hpp"

#include <algorithm>  // std::copy, std::clamp

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

namespace rtc {

PController::PController(std::string_view urdf_path) : PController(urdf_path, Gains{}) {}

PController::PController(std::string_view urdf_path, Gains gains) : gains_lock_(gains) {
  rtc_urdf_bridge::ModelConfig config;
  config.urdf_path = std::string(urdf_path);
  config.root_joint_type = "fixed";

  rtc_urdf_bridge::PinocchioModelBuilder builder(config);
  model_ptr_ = builder.GetFullModel();
  handle_ = std::make_unique<rtc_urdf_bridge::RtModelHandle>(model_ptr_);

  // Default: use the last frame in the model as tip
  tip_frame_id_ = static_cast<pinocchio::FrameIndex>(model_ptr_->nframes - 1);
}

void PController::OnDeviceConfigsSet() {
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

ControllerOutput PController::Compute(const ControllerState& state) noexcept {
  // ── Target slot maintenance (RT thread is the single SeqLock writer) ──
  TargetSlot slot = target_seqlock_.Load();
  bool slot_dirty = false;

  if (!target_initialized_.load(std::memory_order_acquire)) {
    for (std::size_t d = 0; d < static_cast<std::size_t>(state.num_devices); ++d) {
      const auto& dev = state.devices[d];
      if (!dev.valid && d > 0) {
        continue;
      }
      const std::size_t nch = std::min(static_cast<std::size_t>(dev.num_channels),
                                       static_cast<std::size_t>(kMaxDeviceChannels));
      for (std::size_t i = 0; i < nch; ++i) {
        slot.targets[d][i] = dev.positions[i];
      }
    }
    target_initialized_.store(true, std::memory_order_release);
    slot_dirty = true;
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
      slot.targets[didx][i] = pending.values[i];
    }
    slot_dirty = true;
  }

  if (estopped_.load(std::memory_order_acquire)) {
    // E-STOP: hold current position, zero velocity. Also seed the target
    // slot from the current state so a subsequent ClearEstop resumes from
    // there (single-writer path — RT thread itself updates the slot).
    ControllerOutput output{};
    output.valid = true;
    output.command_type = command_type_;
    for (std::size_t d = 0; d < static_cast<std::size_t>(state.num_devices); ++d) {
      for (std::size_t j = 0; j < static_cast<std::size_t>(state.devices[d].num_channels); ++j) {
        output.devices[d].commands[j] = state.devices[d].positions[j];
        slot.targets[d][j] = state.devices[d].positions[j];
      }
    }
    target_seqlock_.Store(slot);
    return output;
  }

  if (slot_dirty) {
    target_seqlock_.Store(slot);
  }

  ControllerOutput output;
  output.num_devices = state.num_devices;

  // Device 0 (robot arm): P control
  const auto& dev0 = state.devices[0];
  auto& out0 = output.devices[0];
  const int nc0 = dev0.num_channels;
  out0.num_channels = nc0;

  // Consistent gains snapshot for this tick
  const auto gains = gains_lock_.Load();

  // Build q span for FK
  std::array<double, kMaxDeviceChannels> q_buf{};
  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    const double error = slot.targets[0][i] - dev0.positions[i];
    out0.commands[i] = dev0.positions[i] + gains.kp[i] * error * state.dt;
    q_buf[i] = dev0.positions[i];
  }

  const auto nv = handle_->nv();
  std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv));
  handle_->ComputeForwardKinematics(q_span);
  const pinocchio::SE3& tcp = handle_->GetFramePlacement(tip_frame_id_);
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp.rotation());

  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  for (std::size_t i = 0; i < static_cast<std::size_t>(nc0); ++i) {
    out0.target_positions[i] = slot.targets[0][i];
    out0.target_velocities[i] = gains.kp[i] * (slot.targets[0][i] - dev0.positions[i]);
    out0.goal_positions[i] = slot.targets[0][i];
  }

  rtc::utils::PassthroughSecondaryDevices(state, output, slot.targets);

  ClampCommands(out0.commands, nc0);
  output.command_type = command_type_;
  return output;
}

void PController::SetDeviceTarget(int device_idx, std::span<const double> target) noexcept {
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
  (void)pending_targets_.Push(pending);
}

void PController::ClampCommands(std::array<double, kMaxDeviceChannels>& commands,
                                int n) const noexcept {
  rtc::utils::ClampSymmetric(commands, n, std::span<const double>(max_joint_velocity_),
                             kDefaultMaxJointVelocity);
}

// ── Controller registry hooks ────────────────────────────────────────────────

void PController::LoadConfig(const YAML::Node& cfg) {
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) {
    return;
  }
  auto g = gains_lock_.Load();
  if (cfg["kp"] && cfg["kp"].IsSequence() && cfg["kp"].size() == 6) {
    for (std::size_t i = 0; i < 6; ++i) {
      g.kp[i] = cfg["kp"][i].as<double>();
    }
  }
  gains_lock_.Store(g);
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

}  // namespace rtc
