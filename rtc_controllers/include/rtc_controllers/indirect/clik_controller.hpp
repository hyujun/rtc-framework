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

#include <Eigen/Cholesky>   // LDLT
#include <Eigen/Core>

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

/// Closed-Loop Inverse Kinematics (CLIK) controller.
///
/// Controls the end-effector **position** (3-DOF) in Cartesian space.
/// Joint velocities are computed via the damped Jacobian pseudoinverse and
/// then integrated to produce absolute joint-position commands.  Surplus
/// DOF are exploited via a null-space secondary task that drives the robot
/// toward a preferred joint configuration, e.g. for joint-limit avoidance.
///
/// ### Control law
/// @code
///   pos_error    = x_des − FK(q)               [3D, metres]
///   J_pos        = J[0:3, :]                   [translational Jacobian, 3×nv]
///   J_pos^#      = J_pos^T (J_pos J_pos^T + λ²I)^{−1}   [damped pseudoinverse]
///   N            = I − J_pos^# J_pos            [null-space projector, nv×nv]
///
///   dq           = kp      * J_pos^# * pos_error          [primary task]
///              +   null_kp * N        * (q_null − q)       [secondary task]
///
///   q_des       += clamp(dq, ±v_max) * dt        [q_des = q_actual at trajectory init]
///   q_cmd        = q_des
/// @endcode
///
/// ### Target convention (`SetRobotTarget` / `/target_joint_positions` topic)
/// The 6 values sent to this controller are **NOT** joint angles:
///   - `target[0..2]` = desired TCP position  [x, y, z]  in world frame (m)
///   - `target[3..5]` = null-space reference joints 3–5 (rad);
///                      joints 0–2 use `kNullTarget` as reference
class ClikController final : public RTControllerInterface {
public:
  // ── Gain / feature configuration ─────────────────────────────────────────
  struct Gains
  {
    std::array<double, 3> kp_translation{{1.0, 1.0, 1.0}}; ///< Translation proportional gain (x,y,z) [1/s]
    std::array<double, 3> kp_rotation{{1.0, 1.0, 1.0}};    ///< Rotation proportional gain (rx,ry,rz) [1/s]
    double damping{0.01};            ///< Damping factor λ for J^#  (singularity robustness)
    double null_kp{0.5};             ///< Null-space joint-centering gain [1/s]
    bool   enable_null_space{true};  ///< Enable null-space secondary task
    bool   control_6dof{false};      ///< Enable 6-DOF (translation + orientation) control

    // Trajectory speed
    double trajectory_speed{0.1};           ///< TCP translational speed for trajectory duration [m/s]
    double trajectory_angular_speed{0.5};   ///< TCP rotational speed for trajectory duration [rad/s]

    // Trajectory velocity limits
    double max_traj_velocity{0.5};             ///< Max TCP velocity during task-space trajectory [m/s]
    double max_traj_angular_velocity{1.0};     ///< Max TCP angular velocity during trajectory [rad/s]
  };

  /// @param urdf_path  Absolute path to the robot URDF file.
  /// @param gains      Gains and feature flags.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  explicit ClikController(std::string_view urdf_path, Gains gains);

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
  // gains layout: [kp_translation×3, kp_rotation×3, damping, null_kp,
  //                enable_null_space(0/1), control_6dof(0/1),
  //                trajectory_speed, trajectory_angular_speed,
  //                max_traj_velocity, max_traj_angular_velocity] = 14 values
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

  /// Cached 3D Cartesian position error from the most recent Compute().
  [[nodiscard]] std::array<double, 3> position_error() const noexcept
  {
    return {pos_error_[0], pos_error_[1], pos_error_[2]};
  }

private:
  // ── Pinocchio via rtc_urdf_bridge ──────────────────────────────────
  std::shared_ptr<const pinocchio::Model> model_ptr_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_frame_id_{0};

  // ── Pre-allocated Eigen work buffers — zero heap alloc on the RT path ────
  Eigen::MatrixXd J_full_;     ///< 6×nv: full spatial Jacobian (LOCAL_WORLD_ALIGNED)
  Eigen::MatrixXd J_pos_;      ///< 3×nv: translational part of J_full_
  Eigen::Matrix3d JJt_;        ///< 3×3: J_pos * J_pos^T + λ²I
  Eigen::Matrix3d JJt_inv_;    ///< 3×3: (J_pos * J_pos^T + λ²I)^{-1}
  Eigen::MatrixXd Jpinv_;      ///< nv×3: damped pseudoinverse J_pos^#
  Eigen::MatrixXd N_;          ///< nv×nv: null-space projector I − J_pos^# J_pos
  Eigen::VectorXd dq_;         ///< nv: joint velocity command
  Eigen::VectorXd desired_q_;  ///< nv: integrated desired joint position
  Eigen::VectorXd traj_dq_;    ///< nv: feedforward-only trajectory velocity (for logging)
  Eigen::VectorXd null_err_;   ///< nv: (q_null − q_current)
  Eigen::VectorXd null_dq_;    ///< nv: null-space contribution to dq
  Eigen::Vector3d pos_error_;  ///< 3: Cartesian position error
  Eigen::Matrix<double, 6, 6> JJt_6d_;       ///< 6x6: J_full * J_full^T + λ²I
  Eigen::Matrix<double, 6, 6> JJt_inv_6d_;   ///< 6x6: (J_full * J_full^T + λ²I)^{-1}
  Eigen::MatrixXd Jpinv_6d_;                 ///< nv×6: damped pseudoinverse J_full^#
  Eigen::Matrix<double, 6, 1> pos_error_6d_; ///< 6: Cartesian position+orientation error

  // LDLT decomposition of JJt_ (3×3) — fixed-size → lives on the stack,
  // no dynamic allocation at construction or on the RT path.
  Eigen::LDLT<Eigen::Matrix3d> ldlt_;
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt_6d_;

  // ── Controller state ──────────────────────────────────────────────────────
  Gains gains_;
  pinocchio::SE3 tcp_target_pose_{pinocchio::SE3::Identity()};
  std::array<double, 3> tcp_target_{};
  /// Null-space reference configuration.  Joints 0–2 from this array;
  /// joints 3–5 are overwritten by SetRobotTarget(target[3..5]).
  std::array<double, kNumRobotJoints> null_target_{0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  std::array<std::array<double, kMaxDeviceChannels>, ControllerState::kMaxDevices> device_targets_{};
  std::array<double, 6> pose_error_cache_{};               ///< diagnostic cache
  std::array<double, 3> tcp_position_{};                ///< diagnostic cache

  bool target_initialized_{false};
  std::atomic<bool> new_target_{false};
  std::mutex target_mutex_;
  trajectory::TaskSpaceTrajectory trajectory_;
  trajectory::TaskSpaceTrajectory::State traj_state_{};
  double trajectory_time_{0.0};

  // ── Multi-segment trajectory (π-rotation defense) ──────────────
  pinocchio::SE3 pending_goal_pose_{pinocchio::SE3::Identity()};
  double pending_duration_{0.0};
  bool has_pending_segment_{false};

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  std::vector<double> safe_position_;
  std::vector<double> max_joint_velocity_;

  CommandType command_type_{CommandType::kPosition};

  // ── Helpers ───────────────────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState & state) noexcept;

  void ClampVelocity(
    std::array<double, kMaxDeviceChannels>& dq, int n) const noexcept;
};

}  // namespace rtc
