#include "ur5e_rt_controller/controllers/indirect/p_controller.hpp"

#include <algorithm> // std::copy, std::clamp
#include <pinocchio/math/rpy.hpp>

namespace ur5e_rt_controller
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

  for (int i = 0; i < kNumRobotJoints; ++i) {
    const double error = robot_target_[i] - state.robot.positions[i];
    output.robot_commands[i] = state.robot.positions[i] + gains_.kp[i] * error * state.robot.dt;
    q_[static_cast<Eigen::Index>(i)] = state.robot.positions[i];
  }

  // forwardKinematics already updates data_.oMi (joint placements).
  // updateFramePlacements is unnecessary — it computes placements for all
  // frames (links, sensors, etc.) which are not needed here.
  pinocchio::forwardKinematics(model_, data_, q_);

  const pinocchio::SE3 & tcp = data_.oMi[end_id_];
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp.rotation());

  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  output.actual_target_positions = robot_target_;
  output.goal_positions = robot_target_;        // P controller: no trajectory, goal == target
  // target_velocities: zero (no trajectory generator)
  output.hand_goal_positions = hand_target_;
  output.robot_commands = ClampCommands(output.robot_commands);
  output.command_type = command_type_;
  return output;
}

void PController::SetRobotTarget(
  std::span<const double, kNumRobotJoints> target) noexcept
{
  std::copy(target.begin(), target.end(), robot_target_.begin());
}

void PController::SetHandTarget(
  std::span<const float, kNumHandMotors> target) noexcept
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

}  // namespace ur5e_rt_controller
