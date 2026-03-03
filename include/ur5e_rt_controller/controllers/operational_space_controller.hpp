// ── Includes: project header first, then third-party, then C++ stdlib ──────────
#pragma once

#include "ur5e_rt_controller/rt_controller_interface.hpp"

// Suppress warnings emitted by Pinocchio / Eigen headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/jacobian.hpp>     // computeJointJacobians, getJointJacobian
#include <pinocchio/algorithm/kinematics.hpp>   // forwardKinematics
#include <pinocchio/algorithm/rnea.hpp>         // computeGeneralizedGravity
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>         // pinocchio::log3 — SO(3) logarithm
#pragma GCC diagnostic pop

#include <Eigen/Core>
#include <Eigen/LU>  // PartialPivLU

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <span>
#include <string>
#include <string_view>

namespace ur5e_rt_controller {

/// Operational Space Controller (OSC) — full 6-DOF Cartesian PD control.
///
/// Controls both end-effector **position** and **orientation** simultaneously,
/// with optional gravity compensation from the Pinocchio dynamics model.
/// All six Cartesian DOF are controlled, leaving no null-space for UR5e.
///
/// ### Control law
/// @code
///   pos_error   = p_des − p_FK(q)              [3D, metres]
///   rot_error   = log₃(R_des * R_FK(q)^T)      [3D axis-angle, LOCAL_WORLD_ALIGNED]
///
///   tcp_vel     = J * dq                        [6D, current task-space velocity]
///
///   task_vel[0:3] = kp_pos * pos_error  −  kd_pos * tcp_vel[0:3]
///   task_vel[3:6] = kp_rot * rot_error  −  kd_rot * tcp_vel[3:6]
///
///   JJt         = J * J^T + λ²I₆               [6×6, damped]
///   J^#         = J^T * JJt^{−1}               [nv×6, damped pseudoinverse]
///
///   dq          = J^# * task_vel               [joint velocity]
///   q_cmd       = q + clamp(dq, ±v_max) * dt
/// @endcode
///
/// If `enable_gravity_compensation` is set, the gravity torque vector g(q)
/// from Pinocchio RNEA is added to the joint velocity command (feedforward).
///
/// ### Target convention (`SetRobotTarget` / `/target_joint_positions` topic)
/// The 6 values are **NOT** joint angles; they represent a full TCP pose:
///   - `target[0..2]` = desired TCP position  [x, y, z]  in world frame (m)
///   - `target[3..5]` = desired TCP orientation [roll, pitch, yaw]  (rad, ZYX)
///
/// ### Usage — swap into custom_controller.cpp
/// @code
///   // 1. Include this header instead of pd_controller.hpp
///   // 2. Change controller_ member type to RTControllerInterface
///   // 3. Initialise:
///   //      controller_(std::make_unique<urtc::OperationalSpaceController>(
///   //          "/opt/ros/humble/share/ur_description/urdf/ur5e.urdf",
///   //          urtc::OperationalSpaceController::Gains{
///   //              .kp_pos = 1.0, .kd_pos = 0.1,
///   //              .kp_rot = 0.5, .kd_rot = 0.05,
///   //              .damping = 0.01}))
///   // 4. Remove the set_gains() call in DeclareAndLoadParameters()
/// @endcode
class OperationalSpaceController final : public RTControllerInterface {
 public:
  // ── Gain / feature configuration ─────────────────────────────────────────
  struct Gains {
    double kp_pos{1.0};                     ///< Cartesian position gain      [1/s]
    double kd_pos{0.1};                     ///< Cartesian position damping   [—]
    double kp_rot{0.5};                     ///< Cartesian orientation gain   [1/s]
    double kd_rot{0.05};                    ///< Cartesian orientation damping[—]
    double damping{0.01};                   ///< Damping factor λ for J^#  (singularity robustness)
    bool   enable_gravity_compensation{false}; ///< Add g(q) feedforward term
  };

  /// @param urdf_path  Absolute path to the UR5e URDF file.
  /// @param gains      PD gains and feature flags.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  explicit OperationalSpaceController(std::string_view urdf_path,
                                       Gains gains = {});

  // ── RTControllerInterface — all methods are noexcept (RT safety) ──────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;

  void SetRobotTarget(std::span<const double> target) noexcept override;
  void SetHandTarget(std::span<const double> target)  noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override;

  void TriggerEstop()                            noexcept override;
  void ClearEstop()                              noexcept override;
  [[nodiscard]] bool IsEstopped() const          noexcept override;
  void SetHandEstop(bool active)                 noexcept override;

  // ── Accessors (non-RT reads only) ─────────────────────────────────────────
  void set_gains(const Gains& g) noexcept { gains_ = g; }
  [[nodiscard]] Gains gains()    const noexcept { return gains_; }

  /// Cached TCP position (world frame) from the most recent Compute().
  [[nodiscard]] std::array<double, 3> tcp_position() const noexcept {
    return tcp_position_;
  }

  /// Cached 6D pose error [pos; rot] from the most recent Compute().
  [[nodiscard]] std::array<double, 6> pose_error() const noexcept {
    return pose_error_cache_;
  }

 private:
  // ── Pinocchio model + pre-allocated Data ─────────────────────────────────
  pinocchio::Model      model_;
  pinocchio::Data       data_;
  pinocchio::JointIndex end_id_{0};   ///< last joint index (end-effector)

  // ── Pre-allocated Eigen work buffers ─────────────────────────────────────
  Eigen::VectorXd q_;        ///< nv: joint positions
  Eigen::VectorXd v_;        ///< nv: joint velocities

  Eigen::MatrixXd J_full_;   ///< 6×nv: full spatial Jacobian (LOCAL_WORLD_ALIGNED)

  // JJt_ and LU decomposition use a fixed-size 6×6 type so that PartialPivLU
  // stores its data inline — no dynamic heap allocation on the RT path.
  Eigen::Matrix<double, 6, 6> JJt_;       ///< J * J^T + λ²I
  Eigen::MatrixXd             Jpinv_;     ///< nv×6: damped pseudoinverse J^#
  Eigen::VectorXd             dq_;        ///< nv: joint velocity command

  // Task-space vectors — fixed 6×1, stack-allocated
  Eigen::Matrix<double, 6, 1> task_err_;  ///< [pos_error(3); rot_error(3)]
  Eigen::Matrix<double, 6, 1> task_vel_;  ///< desired task-space velocity
  Eigen::Matrix<double, 6, 1> tcp_vel_;   ///< current TCP velocity = J * v_

  // PartialPivLU on a fixed-size 6×6 matrix — zero dynamic allocation.
  Eigen::PartialPivLU<Eigen::Matrix<double, 6, 6>> lu_;

  // Desired end-effector rotation matrix, updated in SetRobotTarget().
  Eigen::Matrix3d R_desired_{Eigen::Matrix3d::Identity()};

  // ── Controller state ──────────────────────────────────────────────────────
  Gains gains_;
  std::array<double, 6>              pose_target_{};   ///< [x,y,z,r,p,yaw]
  std::array<double, kNumHandJoints> hand_target_{};
  std::array<double, 3>              tcp_position_{};  ///< diagnostic cache
  std::array<double, 6>              pose_error_cache_{}; ///< diagnostic cache

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  static constexpr std::array<double, kNumRobotJoints> kSafePosition{
      0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  static constexpr double kMaxJointVelocity{2.0};

  // ── Helpers ───────────────────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState& state) noexcept;

  [[nodiscard]] static std::array<double, kNumRobotJoints> ClampVelocity(
      std::array<double, kNumRobotJoints> dq) noexcept;

  /// Compute R_desired from roll/pitch/yaw (ZYX Euler convention).
  /// Called from SetRobotTarget — NOT on the 500 Hz RT path.
  static Eigen::Matrix3d RpyToMatrix(double roll, double pitch,
                                      double yaw) noexcept;
};

// ── Constructor ─────────────────────────────────────────────────────────────

inline OperationalSpaceController::OperationalSpaceController(
    std::string_view urdf_path, Gains gains)
    : data_(pinocchio::Model{}), gains_(gains) {
  pinocchio::urdf::buildModel(std::string(urdf_path), model_);
  data_   = pinocchio::Data(model_);
  end_id_ = static_cast<pinocchio::JointIndex>(model_.njoints - 1);

  // Pre-allocate all Eigen buffers to their final sizes.
  q_      = Eigen::VectorXd::Zero(model_.nv);
  v_      = Eigen::VectorXd::Zero(model_.nv);
  J_full_ = Eigen::MatrixXd::Zero(6, model_.nv);
  Jpinv_  = Eigen::MatrixXd::Zero(model_.nv, 6);
  dq_     = Eigen::VectorXd::Zero(model_.nv);
  JJt_.setZero();
  task_err_.setZero();
  task_vel_.setZero();
  tcp_vel_.setZero();
}

// ── RTControllerInterface implementation ────────────────────────────────────

inline ControllerOutput OperationalSpaceController::Compute(
    const ControllerState& state) noexcept {
  if (estopped_) return ComputeEstop(state);

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
  const pinocchio::SE3& tcp = data_.oMi[end_id_];

  // 3D position error
  const Eigen::Vector3d pos_err =
      Eigen::Vector3d(pose_target_[0], pose_target_[1], pose_target_[2])
      - tcp.translation();
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
  task_vel_.head<3>() = gains_.kp_pos * pos_err
                       - gains_.kd_pos * tcp_vel_.head<3>();
  task_vel_.tail<3>() = gains_.kp_rot * rot_err
                       - gains_.kd_rot * tcp_vel_.tail<3>();

  // ── Step 6: Damped pseudoinverse  J^# = J^T (J J^T + λ²I₆)^{−1} ─────────
  // JJt_ and lu_ are fixed-size 6×6 — no dynamic allocation.
  JJt_.noalias() = J_full_ * J_full_.transpose();
  JJt_.diagonal().array() += gains_.damping * gains_.damping;
  lu_.compute(JJt_);
  // J^# = J^T * JJt_^{−1}   (nv×6)
  // Solve JJt_ * X = I₆  to get JJt_^{−1}
  Jpinv_.noalias() = J_full_.transpose()
                   * lu_.solve(Eigen::Matrix<double, 6, 6>::Identity());

  // ── Step 7: joint velocity from task-space velocity ───────────────────────
  dq_.noalias() = Jpinv_ * task_vel_;

  // ── Step 8: optional gravity compensation ────────────────────────────────
  // Adds the gravity torque g(q) as a feedforward bias.
  // Note: g(q) is in [N·m]; treating it as a velocity offset works as a
  // heuristic feedforward on most position-controlled UR setups.
  if (gains_.enable_gravity_compensation) {
    const Eigen::VectorXd& g =
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
  return output;
}

inline void OperationalSpaceController::SetRobotTarget(
    std::span<const double> target) noexcept {
  // target[0..2] = desired TCP position [x, y, z]
  // target[3..5] = desired TCP orientation [roll, pitch, yaw]
  const std::size_t n = std::min(target.size(), std::size_t{6});
  for (std::size_t i = 0; i < n; ++i) {
    pose_target_[i] = target[i];
  }
  // Precompute R_desired from roll/pitch/yaw — called from the sensor thread,
  // NOT on the 500 Hz RT path.
  if (target.size() >= 6) {
    R_desired_ = RpyToMatrix(target[3], target[4], target[5]);
  }
}

inline void OperationalSpaceController::SetHandTarget(
    std::span<const double> target) noexcept {
  const std::size_t n = std::min(target.size(), kNumHandJoints);
  for (std::size_t i = 0; i < n; ++i) {
    hand_target_[i] = target[i];
  }
}

inline std::string_view OperationalSpaceController::Name() const noexcept {
  return "OperationalSpaceController";
}

inline void OperationalSpaceController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_relaxed);
}

inline void OperationalSpaceController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_relaxed);
}

inline bool OperationalSpaceController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_relaxed);
}

inline void OperationalSpaceController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_relaxed);
}

// ── Private helpers ──────────────────────────────────────────────────────────

inline ControllerOutput OperationalSpaceController::ComputeEstop(
    const ControllerState& state) noexcept {
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  ControllerOutput output;
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    output.robot_commands[i] = state.robot.positions[i]
        + std::clamp(kSafePosition[i] - state.robot.positions[i],
                     -kMaxJointVelocity, kMaxJointVelocity)
        * dt;
  }
  return output;
}

inline std::array<double, kNumRobotJoints>
OperationalSpaceController::ClampVelocity(
    std::array<double, kNumRobotJoints> dq) noexcept {
  for (auto& v : dq) {
    v = std::clamp(v, -kMaxJointVelocity, kMaxJointVelocity);
  }
  return dq;
}

inline Eigen::Matrix3d OperationalSpaceController::RpyToMatrix(
    double roll, double pitch, double yaw) noexcept {
  // ZYX Euler convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  return (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()))
         .toRotationMatrix();
}

}  // namespace ur5e_rt_controller
