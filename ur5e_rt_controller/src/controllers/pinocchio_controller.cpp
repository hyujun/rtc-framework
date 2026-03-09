// ── Includes: project header first, then C++ stdlib ────────────────────────────
#include "ur5e_rt_controller/controllers/pinocchio_controller.hpp"

#include <algorithm>
#include <cstddef>

namespace ur5e_rt_controller
{

// ── Constructor ─────────────────────────────────────────────────────────────

PinocchioController::PinocchioController(
  std::string_view urdf_path,
  Gains gains)
: data_(pinocchio::Model{}), gains_(gains)
{
  // Build model from URDF — may throw if the file is missing or malformed.
  // This runs only once at startup, not on the RT path.
  pinocchio::urdf::buildModel(std::string(urdf_path), model_);

  // Re-create Data with the actual model dimensions.
  data_ = pinocchio::Data(model_);

  // Pre-allocate all Eigen work buffers to their final sizes.
  q_ = pinocchio::neutral(model_);   // neutral config
  v_ = Eigen::VectorXd::Zero(model_.nv);
  coriolis_forces_ = Eigen::VectorXd::Zero(model_.nv);
  jacobian_ = Eigen::MatrixXd::Zero(6, model_.nv);   // 6×nv
}

// ── RTControllerInterface implementation ────────────────────────────────────

ControllerOutput PinocchioController::Compute(
  const ControllerState & state) noexcept
{
  if (estopped_) {
    return ComputeEstop(state);
  }

  // Step 1 — Update Pinocchio state and compute dynamics (no allocation).
  UpdateDynamics(state.robot);

  // Step 2 — PD control + gravity (+ optional Coriolis) compensation.
  ControllerOutput output;
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);

  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    const double e = robot_target_[i] - state.robot.positions[i];
    const double de = (e - prev_error_[i]) / dt;

    output.robot_commands[i] = gains_.kp * e + gains_.kd * de + gravity_torques_[i];

    if (gains_.enable_coriolis_compensation) {
      output.robot_commands[i] +=
        coriolis_forces_[static_cast<Eigen::Index>(i)];
    }

    prev_error_[i] = e;
  }

  output.robot_commands = ClampCommands(output.robot_commands);
  return output;
}

void PinocchioController::SetRobotTarget(
  std::span<const double, kNumRobotJoints> target) noexcept
{
  const std::size_t n = kNumRobotJoints;
  for (std::size_t i = 0; i < n; ++i) {
    robot_target_[i] = target[i];
  }
}

void PinocchioController::SetHandTarget(
  std::span<const double, kNumHandJoints> target) noexcept
{
  const std::size_t n = kNumHandJoints;
  for (std::size_t i = 0; i < n; ++i) {
    hand_target_[i] = target[i];
  }
}

std::string_view PinocchioController::Name() const noexcept
{
  return "PinocchioController";
}

void PinocchioController::TriggerEstop() noexcept
{
  estopped_.store(true, std::memory_order_relaxed);
}

void PinocchioController::ClearEstop() noexcept
{
  estopped_.store(false, std::memory_order_relaxed);
  prev_error_ = {};   // reset derivative term on recovery
}

bool PinocchioController::IsEstopped() const noexcept
{
  return estopped_.load(std::memory_order_relaxed);
}

void PinocchioController::SetHandEstop(bool active) noexcept
{
  hand_estopped_.store(active, std::memory_order_relaxed);
}

std::array<double, kNumRobotJoints>
PinocchioController::gravity_torques() const noexcept
{
  return gravity_torques_;
}

std::array<double, 3> PinocchioController::tcp_position() const noexcept
{
  return tcp_position_;
}

// ── Private helpers ──────────────────────────────────────────────────────────

ControllerOutput PinocchioController::ComputeEstop(
  const ControllerState & state) noexcept
{
  ControllerOutput output;
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    output.robot_commands[i] =
      gains_.kp * (kSafePosition[i] - state.robot.positions[i]);
  }
  output.robot_commands = ClampCommands(output.robot_commands);
  return output;
}

std::array<double, kNumRobotJoints> PinocchioController::ClampCommands(
  std::array<double, kNumRobotJoints> cmds) noexcept
{
  for (auto & c : cmds) {
    c = std::clamp(c, -kMaxJointVelocity, kMaxJointVelocity);
  }
  return cmds;
}

void PinocchioController::UpdateDynamics(
  const RobotState & robot) noexcept
{
  // Copy std::array → Eigen vectors (no allocation — writes into pre-allocated
  // q_ and v_ which are already the correct size from the constructor).
  const std::size_t nv =
    static_cast<std::size_t>(model_.nv);
  const std::size_t n = std::min(static_cast<std::size_t>(kNumRobotJoints), nv);

  for (std::size_t i = 0; i < n; ++i) {
    q_[static_cast<Eigen::Index>(i)] = robot.positions[i];
    v_[static_cast<Eigen::Index>(i)] = robot.velocities[i];
  }

  // ── Gravity torque vector g(q) ─────────────────────────────────────────
  // computeGeneralizedGravity runs RNEA with v=0, a=0.
  // It stores the result in data_.g and returns a const ref to it.
  if (gains_.enable_gravity_compensation) {
    const Eigen::VectorXd & g =
      pinocchio::computeGeneralizedGravity(model_, data_, q_);
    for (std::size_t i = 0; i < n; ++i) {
      gravity_torques_[i] = g[static_cast<Eigen::Index>(i)];
    }
  }

  // ── Forward kinematics ────────────────────────────────────────────────
  // forwardKinematics updates data_.oMi (joint placements in world frame).
  pinocchio::forwardKinematics(model_, data_, q_, v_);

  // ── TCP position — last movable joint's world-frame translation ───────
  // For UR5e the last joint index is model_.njoints - 1.
  // Use pinocchio::updateFramePlacements if a named "tool0" frame exists.
  const auto last_joint =
    static_cast<pinocchio::JointIndex>(model_.njoints - 1);
  const pinocchio::SE3 & tcp = data_.oMi[last_joint];
  tcp_position_[0] = tcp.translation()[0];
  tcp_position_[1] = tcp.translation()[1];
  tcp_position_[2] = tcp.translation()[2];

  // ── End-effector Jacobian ─────────────────────────────────────────────
  // computeJointJacobian requires an updated kinematics call (done above).
  // The result is a 6×nv matrix stored in data_.J via jacobian_.
  pinocchio::computeJointJacobian(model_, data_, q_, last_joint, jacobian_);

  // ── Coriolis / centrifugal forces  C(q,v)·v ──────────────────────────
  if (gains_.enable_coriolis_compensation) {
    const Eigen::MatrixXd & C =
      pinocchio::computeCoriolisMatrix(model_, data_, q_, v_);
    // noalias() avoids a temporary allocation — result written directly
    // into the pre-allocated coriolis_forces_ buffer.
    coriolis_forces_.noalias() = C * v_;
  }
}

}  // namespace ur5e_rt_controller
