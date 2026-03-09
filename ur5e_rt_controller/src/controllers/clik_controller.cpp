// ── Includes: project header first, then C++ stdlib ────────────────────────────
#include "ur5e_rt_controller/controllers/clik_controller.hpp"

#include <algorithm>
#include <cstddef>

namespace ur5e_rt_controller
{

// ── Constructor ─────────────────────────────────────────────────────────────

ClikController::ClikController(std::string_view urdf_path, Gains gains)
: data_(pinocchio::Model{}), gains_(gains)
{
  pinocchio::urdf::buildModel(std::string(urdf_path), model_);
  data_ = pinocchio::Data(model_);
  end_id_ = static_cast<pinocchio::JointIndex>(model_.njoints - 1);

  // Pre-allocate all Eigen buffers to their final sizes.
  q_ = Eigen::VectorXd::Zero(model_.nv);
  J_full_ = Eigen::MatrixXd::Zero(6, model_.nv);
  J_pos_ = Eigen::MatrixXd::Zero(3, model_.nv);
  JJt_ = Eigen::Matrix3d::Zero();
  JJt_inv_ = Eigen::Matrix3d::Zero();
  Jpinv_ = Eigen::MatrixXd::Zero(model_.nv, 3);
  N_ = Eigen::MatrixXd::Identity(model_.nv, model_.nv);
  dq_ = Eigen::VectorXd::Zero(model_.nv);
  null_err_ = Eigen::VectorXd::Zero(model_.nv);
  null_dq_ = Eigen::VectorXd::Zero(model_.nv);
  pos_error_ = Eigen::Vector3d::Zero();
}

// ── RTControllerInterface implementation ────────────────────────────────────

ControllerOutput ClikController::Compute(
  const ControllerState & state) noexcept
{
  if (estopped_) {return ComputeEstop(state);}

  // ── Step 1: copy joint state into Eigen vector ───────────────────────────
  for (Eigen::Index i = 0; i < model_.nv; ++i) {
    q_[i] = state.robot.positions[static_cast<std::size_t>(i)];
  }

  // ── Step 2: FK + Jacobians ───────────────────────────────────────────────
  // computeJointJacobians performs FK internally — data_.oMi is updated.
  pinocchio::computeJointJacobians(model_, data_, q_);
  pinocchio::getJointJacobian(model_, data_, end_id_,
                               pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
  // Translational Jacobian: rows 0..2
  J_pos_.noalias() = J_full_.topRows(3);

  // ── Step 3: Cartesian position error ─────────────────────────────────────
  const Eigen::Vector3d tcp = data_.oMi[end_id_].translation();

  if (new_target_) {
    pinocchio::SE3 start_pose = pinocchio::SE3::Identity();
    start_pose.translation() = tcp;

    pinocchio::SE3 goal_pose = pinocchio::SE3::Identity();
    goal_pose.translation() = Eigen::Vector3d(tcp_target_[0], tcp_target_[1], tcp_target_[2]);

    double max_dist = (goal_pose.translation() - start_pose.translation()).norm();
    double duration = std::max(0.01, max_dist / gains_.trajectory_speed);

    trajectory_.initialize(start_pose, pinocchio::Motion::Zero(),
                           goal_pose, pinocchio::Motion::Zero(),
                           duration);

    trajectory_time_ = 0.0;
    new_target_ = false;
  }

  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  auto traj_state = trajectory_.compute(trajectory_time_);
  trajectory_time_ += dt;

  tcp_position_ = {tcp[0], tcp[1], tcp[2]};
  for (int i = 0; i < 3; ++i) {
    pos_error_[i] = traj_state.pose.translation()[i] - tcp[i];
  }

  // ── Step 4: Damped pseudoinverse  J_pos^# = J_pos^T (J_pos J_pos^T + λ²I)^{-1}
  // JJt_ is Eigen::Matrix3d (fixed-size) — LDLT is also fixed-size, no heap.
  JJt_.noalias() = J_pos_ * J_pos_.transpose();
  JJt_.diagonal().array() += gains_.damping * gains_.damping;
  ldlt_.compute(JJt_);
  // Solve (J_pos J_pos^T + λ²I) X = I₃  →  X = JJt_^{-1}  (fixed-size, no heap)
  JJt_inv_.noalias() = ldlt_.solve(Eigen::Matrix3d::Identity());
  // J_pos^# = J_pos^T * JJt_^{-1}   (nv×3)
  Jpinv_.noalias() = J_pos_.transpose() * JJt_inv_;

  // ── Step 5: Primary task  dq = J_pos^# * (kp * pos_error + feedforward) ───
  Eigen::Vector3d kp_vec(gains_.kp[0], gains_.kp[1], gains_.kp[2]);
  Eigen::Vector3d task_vel = kp_vec.cwiseProduct(pos_error_) + traj_state.velocity.linear();
  dq_.noalias() = Jpinv_ * task_vel;

  // ── Step 6: Null-space secondary task ────────────────────────────────────
  // N = I − J_pos^# * J_pos  maps joint velocities into the null-space of
  // J_pos, leaving the primary Cartesian task unaffected.
  if (gains_.enable_null_space) {
    N_.setIdentity();
    // N_ -= Jpinv_ * J_pos_  →  in-place, noalias avoids temporaries
    N_.noalias() -= Jpinv_ * J_pos_;

    for (Eigen::Index i = 0; i < model_.nv; ++i) {
      null_err_[i] = null_target_[static_cast<std::size_t>(i)] -
        state.robot.positions[static_cast<std::size_t>(i)];
    }
    null_dq_.noalias() = N_ * null_err_;
    null_dq_ *= gains_.null_kp;
    dq_ += null_dq_;
  }

  // ── Step 7: Clamp joint velocity and integrate ────────────────────────────
  // q_cmd = q + clamp(dq, ±v_max) * dt
  std::array<double, kNumRobotJoints> dq_arr{};
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    dq_arr[i] = dq_[static_cast<Eigen::Index>(i)];
  }
  dq_arr = ClampVelocity(dq_arr);

  ControllerOutput output;
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    output.robot_commands[i] = state.robot.positions[i] + dq_arr[i] * dt;
  }
  for (int i = 0; i < 3; ++i) {
    output.actual_target_positions[i] = traj_state.pose.translation()[i];
  }
  for (int i = 3; i < kNumRobotJoints; ++i) {
    output.actual_target_positions[i] = null_target_[i];
  }
  return output;
}

void ClikController::SetRobotTarget(
  std::span<const double, kNumRobotJoints> target) noexcept
{
  // First 3 values: Cartesian [x, y, z]
  const std::size_t n_cart = 3;
  for (std::size_t i = 0; i < n_cart; ++i) {
    tcp_target_[i] = target[i];
  }
  // Values 3..5: null-space reference for joints 3, 4, 5
  for (std::size_t i = 3; i < kNumRobotJoints; ++i) {
    null_target_[i] = target[i];
  }
  new_target_ = true;
}

void ClikController::SetHandTarget(
  std::span<const double, kNumHandJoints> target) noexcept
{
  const std::size_t n = kNumHandJoints;
  for (std::size_t i = 0; i < n; ++i) {
    hand_target_[i] = target[i];
  }
}

std::string_view ClikController::Name() const noexcept
{
  return "ClikController";
}

void ClikController::TriggerEstop() noexcept
{
  estopped_.store(true, std::memory_order_relaxed);
}

void ClikController::ClearEstop() noexcept
{
  estopped_.store(false, std::memory_order_relaxed);
}

bool ClikController::IsEstopped() const noexcept
{
  return estopped_.load(std::memory_order_relaxed);
}

void ClikController::SetHandEstop(bool active) noexcept
{
  hand_estopped_.store(active, std::memory_order_relaxed);
}

// ── Private helpers ──────────────────────────────────────────────────────────

ControllerOutput ClikController::ComputeEstop(
  const ControllerState & state) noexcept
{
  ControllerOutput output;
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    output.robot_commands[i] = state.robot.positions[i] +
      std::clamp(kSafePosition[i] - state.robot.positions[i],
                     -kMaxJointVelocity, kMaxJointVelocity) *
      ((state.dt > 0.0) ? state.dt : (1.0 / 500.0));
  }
  return output;
}

std::array<double, kNumRobotJoints> ClikController::ClampVelocity(
  std::array<double, kNumRobotJoints> dq) noexcept
{
  for (auto & v : dq) {
    v = std::clamp(v, -kMaxJointVelocity, kMaxJointVelocity);
  }
  return dq;
}

}  // namespace ur5e_rt_controller
