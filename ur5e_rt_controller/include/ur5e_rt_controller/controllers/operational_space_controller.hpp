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

#include "ur5e_rt_controller/trajectory/task_space_trajectory.hpp"

#include <Eigen/Core>
#include <Eigen/LU>  // PartialPivLU

#include <array>
#include <atomic>
#include <span>
#include <string>
#include <string_view>

namespace ur5e_rt_controller
{

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
/// ### Usage — swap into rt_controller.cpp
/// @code
///   // 1. Include this header instead of pd_controller.hpp
///   // 2. Change controller_ member type to RTControllerInterface
///   // 3. Initialise:
///   //      controller_(std::make_unique<urtc::OperationalSpaceController>(
///   //          "$(ros2 pkg prefix ur5e_description)/share/ur5e_description/robots/ur5e/urdf/ur5e.urdf",
///   //          urtc::OperationalSpaceController::Gains{
///   //              .kp_pos = 1.0, .kd_pos = 0.1,
///   //              .kp_rot = 0.5, .kd_rot = 0.05,
///   //              .damping = 0.01}))
///   // 4. Remove the set_gains() call in DeclareAndLoadParameters()
/// @endcode
class OperationalSpaceController final : public RTControllerInterface {
public:
  // ── Gain / feature configuration ─────────────────────────────────────────
  struct Gains
  {
    std::array<double, 3> kp_pos{{1.0, 1.0, 1.0}};            ///< Cartesian position gain      [1/s]
    std::array<double, 3> kd_pos{{0.1, 0.1, 0.1}};            ///< Cartesian position damping   [—]
    std::array<double, 3> kp_rot{{0.5, 0.5, 0.5}};            ///< Cartesian orientation gain   [1/s]
    std::array<double, 3> kd_rot{{0.05, 0.05, 0.05}};         ///< Cartesian orientation damping[—]
    double damping{0.01};                   ///< Damping factor λ for J^#  (singularity robustness)
    bool   enable_gravity_compensation{false}; ///< Add g(q) feedforward term
    double trajectory_speed{0.1};           ///< Max translational speed for trajectory [m/s]
    double trajectory_angular_speed{0.5};   ///< Max angular speed for trajectory [rad/s]
  };

  /// @param urdf_path  Absolute path to the UR5e URDF file.
  /// @param gains      PD gains and feature flags.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  explicit OperationalSpaceController(
    std::string_view urdf_path,
    Gains gains);

  // ── RTControllerInterface — all methods are noexcept (RT safety) ──────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState & state) noexcept override;

  void SetRobotTarget(std::span<const double, kNumRobotJoints> target) noexcept override;
  void SetHandTarget(std::span<const double, kNumHandJoints> target)  noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override;

  void TriggerEstop()                            noexcept override;
  void ClearEstop()                              noexcept override;
  [[nodiscard]] bool IsEstopped() const          noexcept override;
  void SetHandEstop(bool active)                 noexcept override;

  // ── Controller registry hooks ────────────────────────────────────────────
  // gains layout: [kp_pos×3, kd_pos×3, kp_rot×3, kd_rot×3, damping, enable_gravity(0/1),
  //                trajectory_speed, trajectory_angular_speed]
  void LoadConfig(const YAML::Node & cfg) override;
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;

  // ── Accessors (non-RT reads only) ─────────────────────────────────────────
  void set_gains(const Gains & g) noexcept {gains_ = g;}
  [[nodiscard]] Gains get_gains() const noexcept {return gains_;}

  /// Cached TCP position (world frame) from the most recent Compute().
  [[nodiscard]] std::array<double, 3> tcp_position() const noexcept
  {
    return tcp_position_;
  }

  /// Cached 6D pose error [pos; rot] from the most recent Compute().
  [[nodiscard]] std::array<double, 6> pose_error() const noexcept
  {
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

  // Desired SE3 goal pose, updated in SetRobotTarget() and used to initialise the trajectory.
  pinocchio::SE3 goal_pose_{pinocchio::SE3::Identity()};

  bool new_target_{false};
  trajectory::TaskSpaceTrajectory trajectory_;
  double trajectory_time_{0.0};

  // ── Controller state ──────────────────────────────────────────────────────
  Gains gains_;
  std::array<double, 6> pose_target_{};                ///< [x,y,z,r,p,yaw]
  std::array<double, kNumHandJoints> hand_target_{};
  std::array<double, 3> tcp_position_{};               ///< diagnostic cache
  std::array<double, 6> pose_error_cache_{};              ///< diagnostic cache

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  static constexpr std::array<double, kNumRobotJoints> kSafePosition{
    0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  static constexpr double kMaxJointVelocity{2.0};

  // ── Helpers ───────────────────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState & state) noexcept;

  [[nodiscard]] static std::array<double, kNumRobotJoints> ClampVelocity(
    std::array<double, kNumRobotJoints> dq) noexcept;

  /// Compute rotation matrix from roll/pitch/yaw (ZYX Euler convention).
  /// Called from SetRobotTarget — NOT on the 500 Hz RT path.
  static Eigen::Matrix3d RpyToMatrix(double roll, double pitch, double yaw) noexcept;
};

}  // namespace ur5e_rt_controller
