#include "ur5e_rt_controller/controllers/p_controller.hpp"

#include <algorithm> // std::copy, std::clamp

namespace ur5e_rt_controller
{

ControllerOutput PController::Compute(const ControllerState & state) noexcept
{
  ControllerOutput output;

  for (int i = 0; i < kNumRobotJoints; ++i) {
    const double error = robot_target_[i] - state.robot.positions[i];
    output.robot_commands[i] = kp_ * error;
  }

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

}  // namespace ur5e_rt_controller
