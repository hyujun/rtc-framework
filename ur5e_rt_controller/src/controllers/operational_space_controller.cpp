// ── Includes: project header first, then C++ stdlib ────────────────────────────
#include "ur5e_rt_controller/controllers/operational_space_controller.hpp"

#include <algorithm>
#include <cstddef>

namespace ur5e_rt_controller
{

// ── Constructor ─────────────────────────────────────────────────────────────

OperationalSpaceController::OperationalSpaceController(
  std::string_view urdf_path, Gains gains)
: data_(pinocchio::Model{}), gains_(gains)
{
  pinocchio::urdf::buildModel(std::string(urdf_path), model_);
  data_ = pinocchio::Data(model_);
  end_id_ = static_cast<pinocchio::JointIndex>(model_.njoints - 1);

  // Pre-allocate all Eigen buffers to their final sizes.
  q_ = Eigen::VectorXd::Zero(model_.nv);
  v_ = Eigen::VectorXd::Zero(model_.nv);
  J_full_ = Eigen::MatrixXd::Zero(6, model_.nv);
  Jpinv_ = Eigen::MatrixXd::Zero(model_.nv, 6);
  dq_ = Eigen::VectorXd::Zero(model_.nv);
  JJt_.setZero();
  task_err_.setZero();
  task_vel_.setZero();
  tcp_vel_.setZero();
}

// ── RTControllerInterface implementation ────────────────────────────────────

ControllerOutput OperationalSpaceController::Compute(
  const ControllerState & state) noexcept
{
  if (estopped_) {return ComputeEstop(state);}

  // ── Step 1: copy joint state into Eigen vectors ──────────────────────────
  for (Eigen::Index i = 0; i < model_.nv; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    q_[i] = state.robot.positions[ui];
    v_[i] = state.robot.velocities[ui];
  }

  // ── Step 2: FK + full Jacobian ────────────────────────────────────────────
  // computeJointJacobians also performs FK — data_.oMi is updated.
  pinocchio::computeJointJacobians(model_, data_, q_);
  pinocchio::getJointJacobian(model_, data_, end_id_,
                               pinocchio::LOCAL_WORLD_ALIGNED, J_full_);

  // ── Step 3: current task-space velocity  tcp_vel = J * dq ────────────────
  tcp_vel_.noalias() = J_full_ * v_;

  // ── Step 4: 6D pose error ─────────────────────────────────────────────────
  const pinocchio::SE3 & tcp = data_.oMi[end_id_];

  // 3D position error
  const Eigen::Vector3d pos_err =
    Eigen::Vector3d(pose_target_[0], pose_target_[1], pose_target_[2]) -
    tcp.translation();
  tcp_position_ = {tcp.translation()[0],
    tcp.translation()[1],
    tcp.translation()[2]};

  // 3D orientation error via SO(3) logarithm:
  //   err_rot = log₃(R_des * R_current^T)
  // This gives the axis-angle vector (in world frame) that rotates the
  // current orientation toward the desired one.
  const Eigen::Matrix3d R_err = R_desired_ * tcp.rotation().transpose();
  const Eigen::Vector3d rot_err = pinocchio::log3(R_err);

  task_err_.head<3>() = pos_err;
  task_err_.tail<3>() = rot_err;

  // Cache for diagnostics (non-RT reads via pose_error())
  for (int i = 0; i < 6; ++i) {
    pose_error_cache_[static_cast<std::size_t>(i)] = task_err_[i];
  }

  // ── Step 5: desired task-space velocity (PD law in Cartesian space) ───────
  Eigen::Vector3d kp_p(gains_.kp_pos[0], gains_.kp_pos[1], gains_.kp_pos[2]);
  Eigen::Vector3d kd_p(gains_.kd_pos[0], gains_.kd_pos[1], gains_.kd_pos[2]);
  Eigen::Vector3d kp_r(gains_.kp_rot[0], gains_.kp_rot[1], gains_.kp_rot[2]);
  Eigen::Vector3d kd_r(gains_.kd_rot[0], gains_.kd_rot[1], gains_.kd_rot[2]);

  task_vel_.head<3>() = kp_p.cwiseProduct(pos_err) -
    kd_p.cwiseProduct(tcp_vel_.head<3>());
  task_vel_.tail<3>() = kp_r.cwiseProduct(rot_err) -
    kd_r.cwiseProduct(tcp_vel_.tail<3>());

  // ── Step 6: Damped pseudoinverse  J^# = J^T (J J^T + λ²I₆)^{−1} ─────────
  // JJt_ and lu_ are fixed-size 6×6 — no dynamic allocation.
  JJt_.noalias() = J_full_ * J_full_.transpose();
  JJt_.diagonal().array() += gains_.damping * gains_.damping;
  lu_.compute(JJt_);
  // J^# = J^T * JJt_^{−1}   (nv×6)
  // Solve JJt_ * X = I₆  to get JJt_^{−1}
  Jpinv_.noalias() = J_full_.transpose() *
    lu_.solve(Eigen::Matrix<double, 6, 6>::Identity());

  // ── Step 7: joint velocity from task-space velocity ───────────────────────
  dq_.noalias() = Jpinv_ * task_vel_;

  // ── Step 8: optional gravity compensation ────────────────────────────────
  // Adds the gravity torque g(q) as a feedforward bias.
  // Note: g(q) is in [N·m]; treating it as a velocity offset works as a
  // heuristic feedforward on most position-controlled UR setups.
  if (gains_.enable_gravity_compensation) {
    const Eigen::VectorXd & g =
      pinocchio::computeGeneralizedGravity(model_, data_, q_);
    dq_ += g;
  }

  // ── Step 9: clamp joint velocity and integrate ────────────────────────────
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  std::array<double, kNumRobotJoints> dq_arr{};
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    dq_arr[i] = dq_[static_cast<Eigen::Index>(i)];
  }
  dq_arr = ClampVelocity(dq_arr);

  ControllerOutput output;
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    output.robot_commands[i] = state.robot.positions[i] + dq_arr[i] * dt;
  }
  output.actual_target_positions = pose_target_;
  return output;
}

void OperationalSpaceController::SetRobotTarget(
  std::span<const double, kNumRobotJoints> target) noexcept
{
  // target[0..2] = desired TCP position [x, y, z]
  // target[3..5] = desired TCP orientation [roll, pitch, yaw]
  const std::size_t n = 6;
  for (std::size_t i = 0; i < n; ++i) {
    pose_target_[i] = target[i];
  }
  // Precompute R_desired from roll/pitch/yaw — called from the sensor thread,
  // NOT on the 500 Hz RT path.
  if (target.size() >= 6) {
    R_desired_ = RpyToMatrix(target[3], target[4], target[5]);
  }
}

void OperationalSpaceController::SetHandTarget(
  std::span<const double, kNumHandJoints> target) noexcept
{
  const std::size_t n = kNumHandJoints;
  for (std::size_t i = 0; i < n; ++i) {
    hand_target_[i] = target[i];
  }
}

std::string_view OperationalSpaceController::Name() const noexcept
{
  return "OperationalSpaceController";
}

void OperationalSpaceController::TriggerEstop() noexcept
{
  estopped_.store(true, std::memory_order_relaxed);
}

void OperationalSpaceController::ClearEstop() noexcept
{
  estopped_.store(false, std::memory_order_relaxed);
}

bool OperationalSpaceController::IsEstopped() const noexcept
{
  return estopped_.load(std::memory_order_relaxed);
}

void OperationalSpaceController::SetHandEstop(bool active) noexcept
{
  hand_estopped_.store(active, std::memory_order_relaxed);
}

// ── Private helpers ──────────────────────────────────────────────────────────

ControllerOutput OperationalSpaceController::ComputeEstop(
  const ControllerState & state) noexcept
{
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  ControllerOutput output;
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    output.robot_commands[i] = state.robot.positions[i] +
      std::clamp(kSafePosition[i] - state.robot.positions[i],
                     -kMaxJointVelocity, kMaxJointVelocity) *
      dt;
  }
  return output;
}

std::array<double, kNumRobotJoints>
OperationalSpaceController::ClampVelocity(
  std::array<double, kNumRobotJoints> dq) noexcept
{
  for (auto & v : dq) {
    v = std::clamp(v, -kMaxJointVelocity, kMaxJointVelocity);
  }
  return dq;
}

Eigen::Matrix3d OperationalSpaceController::RpyToMatrix(
  double roll, double pitch, double yaw) noexcept
{
  // ZYX Euler convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
         .toRotationMatrix();
}

// ── Controller registry hooks ────────────────────────────────────────────────

void OperationalSpaceController::LoadConfig(const YAML::Node & cfg)
{
  if (!cfg) {return;}
  auto load3 = [](const YAML::Node & n, std::array<double, 3> & arr) {
      if (n && n.IsSequence() && n.size() == 3) {
        for (std::size_t i = 0; i < 3; ++i) {arr[i] = n[i].as<double>();}
      }
    };
  load3(cfg["kp_pos"], gains_.kp_pos);
  load3(cfg["kd_pos"], gains_.kd_pos);
  load3(cfg["kp_rot"], gains_.kp_rot);
  load3(cfg["kd_rot"], gains_.kd_rot);
  if (cfg["damping"])                    {gains_.damping = cfg["damping"].as<double>();}
  if (cfg["enable_gravity_compensation"]) {
    gains_.enable_gravity_compensation = cfg["enable_gravity_compensation"].as<bool>();
  }
}

void OperationalSpaceController::UpdateGainsFromMsg(std::span<const double> gains) noexcept
{
  // layout: [kp_pos×3, kd_pos×3, kp_rot×3, kd_rot×3, damping, enable_gravity(0/1)]
  if (gains.size() < 14) {return;}
  for (std::size_t i = 0; i < 3; ++i) {gains_.kp_pos[i] = gains[i];}
  for (std::size_t i = 0; i < 3; ++i) {gains_.kd_pos[i] = gains[3 + i];}
  for (std::size_t i = 0; i < 3; ++i) {gains_.kp_rot[i] = gains[6 + i];}
  for (std::size_t i = 0; i < 3; ++i) {gains_.kd_rot[i] = gains[9 + i];}
  gains_.damping                    = gains[12];
  gains_.enable_gravity_compensation = gains[13] > 0.5;
}

}  // namespace ur5e_rt_controller
