// ── Includes: project header first, then C++ stdlib ────────────────────────────
#include "ur5e_rt_controller/controllers/pd_controller.hpp"

#include <algorithm>  // std::copy, std::clamp
#include <cmath>

namespace ur5e_rt_controller {

PDController::PDController(Gains gains) noexcept : gains_(gains) {}

ControllerOutput PDController::Compute(const ControllerState& state) noexcept {
  ControllerOutput output;

  // When E-STOP is active, drive toward the safe position instead of the
  // user-specified target.
  const auto& effective_target =
      estopped_.load(std::memory_order_acquire) ? kSafePosition : robot_target_;

  const double dt = (state.robot.dt > 0.0) ? state.robot.dt : 0.002;

  for (int i = 0; i < kNumRobotJoints; ++i) {
    const double error      = effective_target[i] - state.robot.positions[i];
    const double derivative = ComputeDerivative(error, previous_errors_[i], dt);
    output.robot_commands[i] = gains_.kp * error + gains_.kd * derivative;
    previous_errors_[i]      = error;
  }

  output.robot_commands = ClampCommands(output.robot_commands);
  return output;
}

void PDController::SetRobotTarget(
    std::span<const double, kNumRobotJoints> target) noexcept {
  std::copy(target.begin(), target.end(), robot_target_.begin());
}

void PDController::SetHandTarget(
    std::span<const double, kNumHandJoints> target) noexcept {
  std::copy(target.begin(), target.end(), hand_target_.begin());
}

void PDController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void PDController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
}

bool PDController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void PDController::SetHandEstop(bool enabled) noexcept {
  hand_estopped_.store(enabled, std::memory_order_release);
}

std::array<double, kNumRobotJoints> PDController::ClampCommands(
    std::span<const double, kNumRobotJoints> commands) noexcept {
  std::array<double, kNumRobotJoints> clamped{};
  for (int i = 0; i < kNumRobotJoints; ++i) {
    clamped[static_cast<std::size_t>(i)] = std::clamp(
        commands[static_cast<std::size_t>(i)],
        -kMaxJointVelocity, kMaxJointVelocity);
  }
  return clamped;
}

double PDController::ComputeDerivative(
    double current_error, double previous_error, double dt) noexcept {
  return (current_error - previous_error) / dt;
}

}  // namespace ur5e_rt_controller
