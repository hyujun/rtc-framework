// ── Includes: project header first, then C++ stdlib ────────────────────────────
#include "ur5e_rt_controller/controllers/indirect/clik_controller.hpp"

#include <algorithm>
#include <cstddef>
#include <pinocchio/math/rpy.hpp>

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

  JJt_6d_ = Eigen::Matrix<double, 6, 6>::Zero();
  JJt_inv_6d_ = Eigen::Matrix<double, 6, 6>::Zero();
  Jpinv_6d_ = Eigen::MatrixXd::Zero(model_.nv, 6);
  pos_error_6d_ = Eigen::Matrix<double, 6, 1>::Zero();
}

// ── RTControllerInterface implementation ────────────────────────────────────

ControllerOutput ClikController::Compute(
  const ControllerState & state) noexcept
{
  if (estopped_.load(std::memory_order_acquire)) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    return out;
  }

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

  // ── Step 3: Cartesian position/orientation error ──────────────────────────
  const pinocchio::SE3 tcp_pose = data_.oMi[end_id_];
  const Eigen::Vector3d tcp = tcp_pose.translation();

  if (!target_initialized_) {
    tcp_target_pose_ = tcp_pose;
    // Keep internal target synchronized
    tcp_target_ = {tcp[0], tcp[1], tcp[2]};
    target_initialized_ = true;
  }

  if (new_target_.load(std::memory_order_acquire)) {
    pinocchio::SE3 start_pose = tcp_pose;
    pinocchio::SE3 goal_pose;

    if (gains_.control_6dof) {
      goal_pose = tcp_target_pose_;
    } else {
      goal_pose = start_pose; // keep current rotation
      goal_pose.translation() = Eigen::Vector3d(tcp_target_[0], tcp_target_[1], tcp_target_[2]);
    }

    double max_dist = (goal_pose.translation() - start_pose.translation()).norm();
    // For 6-DOF, we might also consider angular distance
    if (gains_.control_6dof) {
      double angular_dist = pinocchio::log3(start_pose.rotation().transpose() *
          goal_pose.rotation()).norm();
      max_dist = std::max(max_dist, angular_dist * 0.2); // Rough heuristic: 1 rad ~ 0.2 m
    }
    double duration = std::max(0.01, max_dist / gains_.trajectory_speed);

    trajectory_.initialize(start_pose, pinocchio::Motion::Zero(),
                           goal_pose, pinocchio::Motion::Zero(),
                           duration);

    trajectory_time_ = 0.0;
    new_target_.store(false, std::memory_order_relaxed);
  }

  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  auto traj_state = trajectory_.compute(trajectory_time_);
  trajectory_time_ += dt;

  tcp_position_ = {tcp[0], tcp[1], tcp[2]};

  // Positional error is trajectory - current
  // Rotation error is computed using log6/log3
  pinocchio::SE3 T_current_desired = tcp_pose.actInv(traj_state.pose); // relative pose
  pinocchio::Motion twist_error = pinocchio::log6(T_current_desired);

  // The twist_error is the spatial velocity needed in the LOCAL frame to reach target.
  // Converting it to the LOCAL_WORLD_ALIGNED frame (where J_full_ is expressed).
  Eigen::Vector3d p_err = traj_state.pose.translation() - tcp;
  Eigen::Vector3d r_err = twist_error.angular();
  // We can just use p_err in world frame and r_err in world frame:
  r_err = tcp_pose.rotation() * twist_error.angular();

  pos_error_6d_.head<3>() = p_err;
  pos_error_6d_.tail<3>() = r_err;

  for (int i = 0; i < 3; ++i) {
    pos_error_[i] = p_err[i];
  }

  // ── Step 4 & 5: Damped pseudoinverse & Primary task ──────────────────────
  if (gains_.control_6dof) {
    JJt_6d_.noalias() = J_full_ * J_full_.transpose();
    JJt_6d_.diagonal().array() += gains_.damping * gains_.damping;
    ldlt_6d_.compute(JJt_6d_);
    JJt_inv_6d_.noalias() = ldlt_6d_.solve(Eigen::Matrix<double, 6, 6>::Identity());
    Jpinv_6d_.noalias() = J_full_.transpose() * JJt_inv_6d_;

    Eigen::Matrix<double, 6, 1> kp_vec_6d;
    for (int i = 0; i < 6; ++i) {
      kp_vec_6d[i] = gains_.kp[i];
    }

    Eigen::Matrix<double, 6, 1> task_vel_6d = kp_vec_6d.cwiseProduct(pos_error_6d_);
    // Feedforward
    task_vel_6d.head<3>() += traj_state.velocity.linear();
    task_vel_6d.tail<3>() += tcp_pose.rotation() * traj_state.velocity.angular(); // to world frame

    dq_.noalias() = Jpinv_6d_ * task_vel_6d;
  } else {
    // 3D version
    JJt_.noalias() = J_pos_ * J_pos_.transpose();
    JJt_.diagonal().array() += gains_.damping * gains_.damping;
    ldlt_.compute(JJt_);
    JJt_inv_.noalias() = ldlt_.solve(Eigen::Matrix3d::Identity());
    Jpinv_.noalias() = J_pos_.transpose() * JJt_inv_;

    Eigen::Vector3d kp_vec(gains_.kp[0], gains_.kp[1], gains_.kp[2]);
    Eigen::Vector3d task_vel = kp_vec.cwiseProduct(pos_error_) + traj_state.velocity.linear();
    dq_.noalias() = Jpinv_ * task_vel;
  }

  // ── Step 6: Null-space secondary task ────────────────────────────────────
  if (gains_.enable_null_space) {
    // N = I − J^# * J  maps joint velocities into the null-space of
    // J, leaving the primary task unaffected.
    N_.setIdentity();
    if (gains_.control_6dof) {
      N_.noalias() -= Jpinv_6d_ * J_full_;
    } else {
      N_.noalias() -= Jpinv_ * J_pos_;
    }

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

  const pinocchio::SE3 & tcp_current = data_.oMi[end_id_];
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp_current.rotation());
  output.actual_task_positions[0] = tcp_current.translation().x();
  output.actual_task_positions[1] = tcp_current.translation().y();
  output.actual_task_positions[2] = tcp_current.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  output.command_type = command_type_;
  return output;
}

void ClikController::SetRobotTarget(
  std::span<const double, kNumRobotJoints> target) noexcept
{
  if (gains_.control_6dof) {
    // 6-DOF target mode: [x, y, z, roll, pitch, yaw]
    tcp_target_[0] = target[0];
    tcp_target_[1] = target[1];
    tcp_target_[2] = target[2];

    // Convert Roll-Pitch-Yaw to Rotation Matrix
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    double r = target[3];
    double p = target[4];
    double y = target[5];

    Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;

    tcp_target_pose_.translation() << target[0], target[1], target[2];
    tcp_target_pose_.rotation() = q.matrix();
  } else {
    // 3-DOF target mode: [x, y, z, null3, null4, null5]
    for (std::size_t i = 0; i < 3; ++i) {
      tcp_target_[i] = target[i];
    }
    for (std::size_t i = 3; i < kNumRobotJoints; ++i) {
      null_target_[i] = target[i];
    }
  }
  new_target_.store(true, std::memory_order_release);
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
  estopped_.store(true, std::memory_order_release);
}

void ClikController::ClearEstop() noexcept
{
  estopped_.store(false, std::memory_order_release);
}

bool ClikController::IsEstopped() const noexcept
{
  return estopped_.load(std::memory_order_acquire);
}

void ClikController::SetHandEstop(bool active) noexcept
{
  hand_estopped_.store(active, std::memory_order_release);
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

// ── Controller registry hooks ────────────────────────────────────────────────

void ClikController::LoadConfig(const YAML::Node & cfg)
{
  if (!cfg) {return;}
  if (cfg["kp"] && cfg["kp"].IsSequence()) {
    std::size_t n = std::min<std::size_t>(cfg["kp"].size(), 6);
    for (std::size_t i = 0; i < n; ++i) {
      gains_.kp[i] = cfg["kp"][i].as<double>();
    }
  }
  if (cfg["damping"]) {gains_.damping = cfg["damping"].as<double>();}
  if (cfg["null_kp"]) {gains_.null_kp = cfg["null_kp"].as<double>();}
  if (cfg["enable_null_space"]) {gains_.enable_null_space = cfg["enable_null_space"].as<bool>();}
  if (cfg["trajectory_speed"]) {gains_.trajectory_speed = cfg["trajectory_speed"].as<double>();}
  if (cfg["control_6dof"]) {gains_.control_6dof = cfg["control_6dof"].as<bool>();}
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

void ClikController::UpdateGainsFromMsg(std::span<const double> gains) noexcept
{
  // layout: [kp×6, damping, null_kp, enable_null_space(0/1), control_6dof(0/1)]
  if (gains.size() < 10) {return;}
  for (std::size_t i = 0; i < 6; ++i) {
    gains_.kp[i] = gains[i];
  }
  gains_.damping = gains[6];
  gains_.null_kp = gains[7];
  gains_.enable_null_space = gains[8] > 0.5;
  gains_.control_6dof = gains[9] > 0.5;
}

std::vector<double> ClikController::GetCurrentGains() const noexcept
{
  // layout: [kp×6, damping, null_kp, enable_null_space(0/1), control_6dof(0/1)]
  std::vector<double> v;
  v.reserve(10);
  v.insert(v.end(), gains_.kp.begin(), gains_.kp.end());
  v.push_back(gains_.damping);
  v.push_back(gains_.null_kp);
  v.push_back(gains_.enable_null_space ? 1.0 : 0.0);
  v.push_back(gains_.control_6dof ? 1.0 : 0.0);
  return v;
}

}  // namespace ur5e_rt_controller
