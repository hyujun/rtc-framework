// ── Includes: project header first, then third-party, then C++ stdlib
// ──────────
#pragma once

#include "rtc_controller_interface/rt_controller_interface.hpp"
#include <rtc_base/concurrency/spsc_queue.hpp>
#include <rtc_base/threading/seqlock.hpp>
#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/rt_model_handle.hpp>

// Suppress warnings emitted by Pinocchio / Eigen headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial.hpp>
#pragma GCC diagnostic pop

#include "rtc_controllers/trajectory/task_space_trajectory.hpp"

#include <Eigen/Core>
#include <Eigen/LU>  // PartialPivLU

#include <array>
#include <atomic>
#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace rtc {

/// Operational Space Controller (OSC) — full 6-DOF Cartesian **torque** control.
///
/// Controls end-effector position and orientation via the operational-space
/// (Khatib) torque formulation: a task-space PD acceleration is mapped through
/// the task inertia Λ and Jacobian transpose into joint torque, with full
/// joint-space Coriolis + gravity compensation. Output command is **N·m**.
/// The command type is fixed to `kTorque`; a redundancy (nv > 6) posture task
/// is projected through the dynamically-consistent null space.
///
/// ### Control law
/// @code
///   pos_error = p_des − p_FK(q)               [3D world, metres]
///   rot_error = log₃(R_des · R_FK(q)^T)       [3D world axis-angle]
///   ẋ         = J · v                          [6D current task velocity]
///
///   a_task[0:3] = kp_pos·pos_error + kd_pos·(ẋ_d,lin − ẋ_lin) + a_ff,lin
///   a_task[3:6] = kp_rot·rot_error + kd_rot·(ẋ_d,ang − ẋ_ang) + a_ff,ang
///
///   M       = symmetrise(M(q))                 [nv×nv joint-space inertia]
///   h       = C(q,v)·v + g(q)                  [nv nonlinear effects]
///   Λ⁻¹     = J·M⁻¹·Jᵀ + λ²I₆                  [6×6 damped task inertia inverse]
///   F       = Λ · a_task                        [6D task-space force]
///   τ       = Jᵀ·F + h + Nᵀ·τ₀                  [nv joint torque, N·m]
///     with  J̄ᵀ = Λ·J·M⁻¹,  Nᵀ = I − Jᵀ·J̄ᵀ   [dynamically-consistent null space]
///           τ₀ = null_kp·(q_safe − q) − null_kd·v   [posture secondary task]
/// @endcode
///
/// Λ carries λ² damping on its inverse for singularity robustness. All linear
/// solves use pre-sized in-place Cholesky factorisations (RT alloc-free).
///
/// ### Target convention (`SetRobotTarget` / `/target_joint_positions` topic)
/// The 6 values are **NOT** joint angles; they represent a full TCP pose:
///   - `target[0..2]` = desired TCP position  [x, y, z]  in world frame (m)
///   - `target[3..5]` = desired TCP orientation [roll, pitch, yaw]  (rad, ZYX)
class OperationalSpaceController final : public RTControllerInterface {
 public:
  // ── Gain / feature configuration ─────────────────────────────────────────
  struct Gains {
    // Task-space PD (acceleration form): a = kp·e_pose + kd·(ẋ_d − ẋ) + a_ff.
    // kp is a stiffness-like gain [1/s²], kd a derivative gain [1/s]. Defaults
    // are sane torque-OSC starting points — retune per robot (examples only).
    std::array<double, 3> kp_pos{{100.0, 100.0, 100.0}};  ///< Cartesian position gain     [1/s²]
    std::array<double, 3> kd_pos{{20.0, 20.0, 20.0}};     ///< Cartesian position damping  [1/s]
    std::array<double, 3> kp_rot{{50.0, 50.0, 50.0}};     ///< Cartesian orientation gain  [1/s²]
    std::array<double, 3> kd_rot{{10.0, 10.0, 10.0}};     ///< Cartesian orientation damp  [1/s]
    double damping{0.01};  ///< Damping factor λ on Λ⁻¹  (singularity robustness)

    // Dynamically-consistent null-space posture task (only meaningful when
    // nv > 6; for a non-redundant 6-DOF arm Nᵀ ≈ 0 so these are inert).
    // τ₀ is summed directly into joint torque (not through M), so these are
    // stiffness/damping in TORQUE units, NOT the acceleration-form gains above.
    double null_kp{0.0};  ///< Posture centering stiffness toward safe_position [N·m/rad]
    double null_kd{1.0};  ///< Null-space joint damping                         [N·m·s/rad]

    // Retained for YAML back-compat; torque OSC always compensates g(q)+C·v
    // (required for the control law). This flag is parsed but ignored.
    bool enable_gravity_compensation{true};  ///< [deprecated] no-op in torque mode

    // Trajectory speed
    double trajectory_speed{0.1};          ///< Max translational speed for trajectory [m/s]
    double trajectory_angular_speed{0.5};  ///< Max angular speed for trajectory [rad/s]

    // Trajectory velocity limits
    double max_traj_velocity{0.5};          ///< Max TCP velocity during task-space trajectory [m/s]
    double max_traj_angular_velocity{1.0};  ///< Max TCP angular velocity during trajectory [rad/s]
  };

  /// @param urdf_path  Absolute path to the robot URDF file.
  /// @param gains      PD gains and feature flags.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  explicit OperationalSpaceController(std::string_view urdf_path, Gains gains);

  // ── RTControllerInterface — all methods are noexcept (RT safety) ──────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;

  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override;

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  // ── Controller registry hooks ────────────────────────────────────────────
  // gains layout: [kp_pos×3, kd_pos×3, kp_rot×3, kd_rot×3, damping,
  // enable_gravity(0/1),
  //                trajectory_speed, trajectory_angular_speed,
  //                max_traj_velocity, max_traj_angular_velocity] = 18 values
  void LoadConfig(const YAML::Node& cfg) override;
  void OnDeviceConfigsSet() override;

  [[nodiscard]] CommandType GetCommandType() const noexcept override { return command_type_; }

  // ── Accessors (non-RT reads only) ─────────────────────────────────────────
  void set_gains(const Gains& g) noexcept { gains_lock_.Store(g); }

  [[nodiscard]] Gains get_gains() const noexcept { return gains_lock_.Load(); }

  /// Cached TCP position (world frame) from the most recent Compute().
  [[nodiscard]] std::array<double, 3> tcp_position() const noexcept { return tcp_position_; }

  /// Cached 6D pose error [pos; rot] from the most recent Compute().
  [[nodiscard]] std::array<double, 6> pose_error() const noexcept { return pose_error_cache_; }

 private:
  // ── Pinocchio via rtc_urdf_bridge ──────────────────────────────────
  std::shared_ptr<const pinocchio::Model> model_ptr_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_frame_id_{0};

  // ── Pre-allocated Eigen work buffers (all sized in the ctor; RT alloc-free) ─
  Eigen::MatrixXd J_full_;  ///< 6×nv: full spatial Jacobian (LOCAL_WORLD_ALIGNED)
  Eigen::MatrixXd Jt_;      ///< nv×6: Jᵀ (materialised for products / RHS)

  // Joint-space dynamics
  Eigen::MatrixXd M_;        ///< nv×nv: symmetrised joint-space inertia M(q)
  Eigen::MatrixXd MinvJt_;   ///< nv×6: M⁻¹ Jᵀ (via in-place Cholesky solve)
  Eigen::VectorXd h_;        ///< nv: nonlinear effects h = C·v + g
  Eigen::VectorXd tau_out_;  ///< nv: joint torque command [N·m]

  // Task-space quantities — fixed 6×1 / 6×6, stack-allocated
  Eigen::Matrix<double, 6, 6> LambdaInv_;  ///< J M⁻¹ Jᵀ + λ²I  (task inertia inverse)
  Eigen::Matrix<double, 6, 1> task_err_;   ///< [pos_error(3); rot_error(3)]
  Eigen::Matrix<double, 6, 1> a_task_;     ///< desired task-space acceleration
  Eigen::Matrix<double, 6, 1> F_;          ///< task-space force  F = Λ a_task
  Eigen::Matrix<double, 6, 1> tcp_vel_;    ///< current TCP velocity = J · v

  // Dynamically-consistent null-space posture task
  Eigen::MatrixXd JbarT_;     ///< 6×nv: dynamically-consistent inverse transpose Λ J M⁻¹
  Eigen::MatrixXd NT_;        ///< nv×nv: null-space projector transpose I − Jᵀ J̄ᵀ
  Eigen::VectorXd tau0_;      ///< nv: raw posture torque (pre-projection)
  Eigen::VectorXd null_tmp_;  ///< nv: projected null-space torque

  // In-place Cholesky factorisations — pre-sized, RT alloc-free.
  Eigen::LLT<Eigen::MatrixXd> llt_M_;             ///< M(q) factor (nv×nv, SPD)
  Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt6_;  ///< Λ⁻¹ factor (6×6, SPD)

  // RT-thread-only working copies materialised from the SeqLock POD at the
  // start of each Compute(). Not shared across threads.
  pinocchio::SE3 goal_pose_{pinocchio::SE3::Identity()};

  trajectory::TaskSpaceTrajectory trajectory_;
  trajectory::TaskSpaceTrajectory::State traj_state_{};
  double trajectory_time_{0.0};

  // ── Multi-segment trajectory (π-rotation defense) ──────────────
  pinocchio::SE3 pending_goal_pose_{pinocchio::SE3::Identity()};
  double pending_duration_{0.0};
  bool has_pending_segment_{false};

  // ── Controller state ──────────────────────────────────────────────────────
  SeqLock<Gains> gains_lock_;

  // TargetSlot — Eigen / pinocchio::SE3 are NOT trivially copyable (false
  // negative documented in [[feedback_eigen_seqlock_pod_wrapper]]), so the
  // goal pose is mirrored as a plain double[9] + double[3] flat array. The
  // RT thread is the sole SeqLock writer; off-RT writers push onto
  // pending_targets_ instead.
  static constexpr std::size_t kSE3RotDoubles = 9;
  static constexpr std::size_t kSE3TransDoubles = 3;

  struct TargetSlot {
    std::array<double, 6> pose_target{};            // [x,y,z,r,p,yaw]
    std::array<double, kSE3RotDoubles> goal_rot{};  // 3x3 col-major
    std::array<double, kSE3TransDoubles> goal_t{};
    std::array<std::array<double, kMaxDeviceChannels>, ControllerState::kMaxDevices> targets{};
  };

  static_assert(std::is_trivially_copyable_v<TargetSlot>,
                "TargetSlot must be trivially copyable for SeqLock<TargetSlot>");

  struct PendingTarget {
    int device_idx{0};
    int num_values{0};
    std::array<double, kMaxDeviceChannels> values{};
  };

  static_assert(std::is_trivially_copyable_v<PendingTarget>,
                "PendingTarget must be trivially copyable for SpscQueue");

  static constexpr std::size_t kPendingTargetDepth = 4;

  SeqLock<TargetSlot> target_seqlock_;
  SpscQueue<PendingTarget, kPendingTargetDepth> pending_targets_;
  std::atomic<bool> target_initialized_{false};
  bool new_target_pending_{false};  // RT-thread-only; gates trajectory re-init

  std::array<double, 3> tcp_position_{};      ///< diagnostic cache
  std::array<double, 6> pose_error_cache_{};  ///< diagnostic cache

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  std::vector<double> safe_position_;
  std::vector<double> max_joint_velocity_;  ///< E-STOP position slew limit [rad/s]
  std::vector<double> max_joint_torque_;    ///< torque command clamp [N·m]

  // Torque-only: fixed to kTorque. LoadConfig rejects any other command_type.
  CommandType command_type_{CommandType::kTorque};

  // ── Helpers ───────────────────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState& state) noexcept;

  static Eigen::Matrix3d RpyToMatrix(double roll, double pitch, double yaw) noexcept;
};

}  // namespace rtc
