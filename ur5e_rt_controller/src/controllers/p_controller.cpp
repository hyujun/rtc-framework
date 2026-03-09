#include "ur5e_rt_controller/controllers/p_controller.hpp"

#include <algorithm> // std::copy, std::clamp

namespace ur5e_rt_controller
{

PController::PController() noexcept
: gains_(Gains{})
{
}

PController::PController(Gains gains) noexcept
: gains_(gains)
{
}

ControllerOutput PController::Compute(const ControllerState & state) noexcept
{
  ControllerOutput output;

  for (int i = 0; i < kNumRobotJoints; ++i) {
    const double error = robot_target_[i] - state.robot.positions[i];
    output.robot_commands[i] = gains_.kp[i] * error;
  }

  output.actual_target_positions = robot_target_;
  output.robot_commands = ClampCommands(output.robot_commands);
  return output;
}

void PController::SetRobotTarget(
  std::span<const double, kNumRobotJoints> target) noexcept
{
  std::copy(target.begin(), target.end(), robot_target_.begin());
}

void PController::SetHandTarget(
  std::span<const double, kNumHandJoints> target) noexcept
{
  std::copy(target.begin(), target.end(), hand_target_.begin());
}

std::array<double, kNumRobotJoints> PController::ClampCommands(
  std::span<const double, kNumRobotJoints> commands) noexcept
{
  std::array<double, kNumRobotJoints> clamped{};
  for (int i = 0; i < kNumRobotJoints; ++i) {
    clamped[static_cast<std::size_t>(i)] = std::clamp(
        commands[static_cast<std::size_t>(i)],
        -kMaxJointVelocity, kMaxJointVelocity);
  }
  return clamped;
}

// ── Controller registry hooks ────────────────────────────────────────────────

void PController::LoadConfig(const YAML::Node & cfg)
{
  if (!cfg) {return;}
  if (cfg["kp"] && cfg["kp"].IsSequence() && cfg["kp"].size() == 6) {
    for (std::size_t i = 0; i < 6; ++i) {
      gains_.kp[i] = cfg["kp"][i].as<double>();
    }
  }
}

void PController::UpdateGainsFromMsg(std::span<const double> gains) noexcept
{
  // layout: [kp×6]
  if (gains.size() < 6) {return;}
  for (std::size_t i = 0; i < 6; ++i) {gains_.kp[i] = gains[i];}
}

}  // namespace ur5e_rt_controller
