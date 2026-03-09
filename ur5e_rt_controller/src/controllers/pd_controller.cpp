// ── Includes: project header first, then C++ stdlib ────────────────────────────
#include "ur5e_rt_controller/controllers/pd_controller.hpp"

#include <algorithm>  // std::copy, std::clamp
#include <cmath>

namespace ur5e_rt_controller
{

PDController::PDController() noexcept
: gains_(Gains{})
{
  trajectory_.initialize({}, {}, 0.0);
}

PDController::PDController(Gains gains) noexcept
: gains_(gains) {}

ControllerOutput PDController::Compute(const ControllerState & state) noexcept
{
  ControllerOutput output;
  const double dt = (state.robot.dt > 0.0) ? state.robot.dt : 0.002;

  // When E-STOP is active, drive toward the safe position directly (bypass trajectory)
  if (estopped_.load(std::memory_order_acquire)) {
    for (int i = 0; i < kNumRobotJoints; ++i) {
      const double error = kSafePosition[i] - state.robot.positions[i];
      const double derivative = ComputeDerivative(error, previous_errors_[i], dt);
      output.robot_commands[i] = gains_.kp[i] * error + gains_.kd[i] * derivative;
      previous_errors_[i] = error;
    }
    output.actual_target_positions = kSafePosition;
    output.robot_commands = ClampCommands(output.robot_commands);
    new_target_ = true; // force trajectory regeneration when E-STOP clears
    return output;
  }

  // Normal operation: generate and evaluate trajectory
  if (new_target_) {
    trajectory::JointSpaceTrajectory<kNumRobotJoints>::State start_state;
    trajectory::JointSpaceTrajectory<kNumRobotJoints>::State goal_state;

    double max_dist = 0.0;
    for (int i = 0; i < kNumRobotJoints; ++i) {
      start_state.positions[i] = state.robot.positions[i];
      start_state.velocities[i] = state.robot.velocities[i];
      start_state.accelerations[i] = 0.0;

      goal_state.positions[i] = robot_target_[i];
      goal_state.velocities[i] = 0.0;
      goal_state.accelerations[i] = 0.0;

      max_dist = std::max(max_dist, std::abs(goal_state.positions[i] - start_state.positions[i]));
    }

    // Heuristic duration based on configured speed
    double duration = std::max(0.01, max_dist / gains_.trajectory_speed);

    trajectory_.initialize(start_state, goal_state, duration);
    trajectory_time_ = 0.0;
    new_target_ = false;
  }

  auto traj_state = trajectory_.compute(trajectory_time_);
  trajectory_time_ += dt;

  for (int i = 0; i < kNumRobotJoints; ++i) {
    const double error = traj_state.positions[i] - state.robot.positions[i];
    const double derivative = ComputeDerivative(error, previous_errors_[i], dt);

    // Feedforward velocity + PD feedback
    output.robot_commands[i] = traj_state.velocities[i] + gains_.kp[i] * error + gains_.kd[i] *
      derivative;
    previous_errors_[i] = error;
  }

  output.actual_target_positions = traj_state.positions;
  output.robot_commands = ClampCommands(output.robot_commands);
  return output;
}

void PDController::SetRobotTarget(
  std::span<const double, kNumRobotJoints> target) noexcept
{
  std::copy(target.begin(), target.end(), robot_target_.begin());
  new_target_ = true;
}

void PDController::SetHandTarget(
  std::span<const double, kNumHandJoints> target) noexcept
{
  std::copy(target.begin(), target.end(), hand_target_.begin());
}

void PDController::TriggerEstop() noexcept
{
  estopped_.store(true, std::memory_order_release);
}

void PDController::ClearEstop() noexcept
{
  estopped_.store(false, std::memory_order_release);
}

bool PDController::IsEstopped() const noexcept
{
  return estopped_.load(std::memory_order_acquire);
}

void PDController::SetHandEstop(bool enabled) noexcept
{
  hand_estopped_.store(enabled, std::memory_order_release);
}

std::array<double, kNumRobotJoints> PDController::ClampCommands(
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

double PDController::ComputeDerivative(
  double current_error, double previous_error, double dt) noexcept
{
  return (current_error - previous_error) / dt;
}

// ── Controller registry hooks ────────────────────────────────────────────────

void PDController::LoadConfig(const YAML::Node & cfg)
{
  if (!cfg) {return;}
  if (cfg["kp"] && cfg["kp"].IsSequence() && cfg["kp"].size() == 6) {
    for (std::size_t i = 0; i < 6; ++i) {gains_.kp[i] = cfg["kp"][i].as<double>();}
  }
  if (cfg["kd"] && cfg["kd"].IsSequence() && cfg["kd"].size() == 6) {
    for (std::size_t i = 0; i < 6; ++i) {gains_.kd[i] = cfg["kd"][i].as<double>();}
  }
  if (cfg["trajectory_speed"]) {
    gains_.trajectory_speed = cfg["trajectory_speed"].as<double>();
  }
}

void PDController::UpdateGainsFromMsg(std::span<const double> gains) noexcept
{
  // layout: [kp×6, kd×6]
  if (gains.size() < 12) {return;}
  for (std::size_t i = 0; i < 6; ++i) {gains_.kp[i] = gains[i];}
  for (std::size_t i = 0; i < 6; ++i) {gains_.kd[i] = gains[6 + i];}
}

}  // namespace ur5e_rt_controller
