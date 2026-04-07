// ── Includes: project header first, then third-party, then C++ stdlib ──────────
#pragma once

#include "rtc_controller_interface/rt_controller_interface.hpp"

#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/rt_model_handle.hpp>

// Suppress warnings emitted by Pinocchio / Eigen headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#pragma GCC diagnostic pop

#include "rtc_controllers/trajectory/task_space_trajectory.hpp"

#include <Eigen/Core>
#include <Eigen/LU>  // PartialPivLU

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rtc
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
/// ### Target convention (`SetRobotTarget` / `/target_joint_positions` topic)
/// The 6 values are **NOT** joint angles; they represent a full TCP pose:
///   - `target[0..2]` = desired TCP position  [x, y, z]  in world frame (m)
///   - `target[3..5]` = desired TCP orientation [roll, pitch, yaw]  (rad, ZYX)
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

    // Trajectory speed
    double trajectory_speed{0.1};           ///< Max translational speed for trajectory [m/s]
    double trajectory_angular_speed{0.5};   ///< Max angular speed for trajectory [rad/s]

    // Trajectory velocity limits
    double max_traj_velocity{0.5};             ///< Max TCP velocity during task-space trajectory [m/s]
    double max_traj_angular_velocity{1.0};     ///< Max TCP angular velocity during trajectory [rad/s]
  };

  /// @param urdf_path  Absolute path to the robot URDF file.
  /// @param gains      PD gains and feature flags.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  explicit OperationalSpaceController(
    std::string_view urdf_path,
    Gains gains);

  // ── RTControllerInterface — all methods are noexcept (RT safety) ──────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState & state) noexcept override;

  void SetDeviceTarget(
    int device_idx, std::span<const double> target) noexcept override;

  void InitializeHoldPosition(const ControllerState & state) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override;

  void TriggerEstop()                            noexcept override;
  void ClearEstop()                              noexcept override;
  [[nodiscard]] bool IsEstopped() const          noexcept override;
  void SetHandEstop(bool active)                 noexcept override;

  // ── Controller registry hooks ────────────────────────────────────────────
  // gains layout: [kp_pos×3, kd_pos×3, kp_rot×3, kd_rot×3, damping, enable_gravity(0/1),
  //                trajectory_speed, trajectory_angular_speed,
  //                max_traj_velocity, max_traj_angular_velocity] = 18 values
  void LoadConfig(const YAML::Node & cfg) override;
  void OnDeviceConfigsSet() override;
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;
  [[nodiscard]] std::vector<double> GetCurrentGains() const noexcept override;
  [[nodiscard]] CommandType GetCommandType() const noexcept override {return command_type_;}

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
  // ── Pinocchio via rtc_urdf_bridge ──────────────────────────────────
  std::shared_ptr<const pinocchio::Model> model_ptr_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_frame_id_{0};

  // ── Pre-allocated Eigen work buffers ─────────────────────────────────────
  Eigen::VectorXd q_;        ///< nv: joint positions (for gravity only)
  Eigen::VectorXd v_;        ///< nv: joint velocities (for gravity only)

  Eigen::MatrixXd J_full_;   ///< 6×nv: full spatial Jacobian (LOCAL_WORLD_ALIGNED)

  Eigen::Matrix<double, 6, 6> JJt_;       ///< J * J^T + λ²I
  Eigen::MatrixXd             Jpinv_;     ///< nv×6: damped pseudoinverse J^#
  Eigen::VectorXd             dq_;        ///< nv: joint velocity command
  Eigen::VectorXd             traj_dq_;   ///< nv: feedforward-only trajectory velocity (for logging)

  // Task-space vectors — fixed 6×1, stack-allocated
  Eigen::Matrix<double, 6, 1> task_err_;  ///< [pos_error(3); rot_error(3)]
  Eigen::Matrix<double, 6, 1> task_vel_;  ///< desired task-space velocity
  Eigen::Matrix<double, 6, 1> tcp_vel_;   ///< current TCP velocity = J * v_

  // PartialPivLU on a fixed-size 6×6 matrix — zero dynamic allocation.
  Eigen::PartialPivLU<Eigen::Matrix<double, 6, 6>> lu_;

  // Desired SE3 goal pose, updated in SetRobotTarget() and used to initialise the trajectory.
  pinocchio::SE3 goal_pose_{pinocchio::SE3::Identity()};

  std::atomic<bool> new_target_{false};
  std::mutex target_mutex_;
  trajectory::TaskSpaceTrajectory trajectory_;
  trajectory::TaskSpaceTrajectory::State traj_state_{};
  double trajectory_time_{0.0};

  // ── Multi-segment trajectory (π-rotation defense) ──────────────
  pinocchio::SE3 pending_goal_pose_{pinocchio::SE3::Identity()};
  double pending_duration_{0.0};
  bool has_pending_segment_{false};

  // ── Controller state ──────────────────────────────────────────────────────
  Gains gains_;
  std::array<double, 6> pose_target_{};                ///< [x,y,z,r,p,yaw]
  std::array<std::array<double, kMaxDeviceChannels>, ControllerState::kMaxDevices> device_targets_{};
  std::array<double, 3> tcp_position_{};               ///< diagnostic cache
  std::array<double, 6> pose_error_cache_{};              ///< diagnostic cache

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  std::vector<double> safe_position_;
  std::vector<double> max_joint_velocity_;

  CommandType command_type_{CommandType::kTorque};

  // ── Helpers ───────────────────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState & state) noexcept;

  void ClampVelocity(
    std::array<double, kMaxDeviceChannels>& dq, int n) const noexcept;

  static Eigen::Matrix3d RpyToMatrix(double roll, double pitch, double yaw) noexcept;
};

}  // namespace rtc
