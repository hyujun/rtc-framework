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

ControllerOutput DemoJointController::Compute(const ControllerState & state) noexcept
{
  ControllerOutput output;

  // ── Robot arm P control (identical to PController) ────────────────────────
  for (int i = 0; i < kNumRobotJoints; ++i) {
    const double error = robot_target_[i] - state.robot.positions[i];
    output.robot_commands[i] =
      state.robot.positions[i] + gains_.robot_kp[i] * error * state.robot.dt;
    q_[static_cast<Eigen::Index>(i)] = state.robot.positions[i];
  }
  output.robot_commands = ClampRobotCommands(output.robot_commands);

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

  output.actual_target_positions = robot_target_;
  output.goal_positions = robot_target_;        // P controller: no trajectory, goal == target
  // target_velocities: zero (no trajectory generator)
  output.hand_goal_positions = hand_target_;

  // ── Hand motor P control (same formula applied to 10 hand motors) ─────────
  if (state.hand.valid) {
    for (int i = 0; i < kNumHandMotors; ++i) {
      const float error =
        hand_target_[static_cast<std::size_t>(i)] -
        state.hand.motor_positions[static_cast<std::size_t>(i)];
      output.hand_commands[static_cast<std::size_t>(i)] =
        state.hand.motor_positions[static_cast<std::size_t>(i)] +
        gains_.hand_kp[static_cast<std::size_t>(i)] * error *
        static_cast<float>(state.dt);
    }
    output.hand_commands = ClampHandCommands(output.hand_commands);
  }

  output.command_type = command_type_;
  return output;
}

void DemoJointController::SetRobotTarget(
  std::span<const double, kNumRobotJoints> target) noexcept
{
  std::copy(target.begin(), target.end(), robot_target_.begin());
}

void DemoJointController::SetHandTarget(
  std::span<const float, kNumHandMotors> target) noexcept
{
  std::copy(target.begin(), target.end(), hand_target_.begin());
}

void DemoJointController::InitializeHoldPosition(
  const ControllerState & state) noexcept
{
  std::copy(state.robot.positions.begin(), state.robot.positions.end(),
            robot_target_.begin());
  if (state.hand.valid) {
    std::copy(state.hand.motor_positions.begin(),
              state.hand.motor_positions.end(), hand_target_.begin());
  }
}

std::array<double, kNumRobotJoints> DemoJointController::ClampRobotCommands(
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

std::array<float, kNumHandMotors> DemoJointController::ClampHandCommands(
  std::span<const float, kNumHandMotors> commands) noexcept
{
  std::array<float, kNumHandMotors> clamped{};
  for (int i = 0; i < kNumHandMotors; ++i) {
    clamped[static_cast<std::size_t>(i)] = std::clamp(
      commands[static_cast<std::size_t>(i)],
      -kMaxHandVelocity, kMaxHandVelocity);
  }
  return clamped;
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
