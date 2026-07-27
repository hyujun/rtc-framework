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

#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/compliance/torque_estop.hpp"
#include "rtc_controllers/trajectory/task_space_trajectory.hpp"

#include <Eigen/Core>
#include <Eigen/LU>  // PartialPivLU

#include <array>
#include <atomic>
#include <cstdint>
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
/// Λ, J̄ and Nᵀ are formed by `compliance::TaskDynamics` (#236 S2b), which is the
/// same object the impedance and cascade controllers use — including its §6.5
/// σ_min-adaptive λ². All linear solves use pre-sized in-place Cholesky
/// factorisations (RT alloc-free).
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
    // §6.5 σ_min-adaptive DLS damping — the same two parameters, with the same
    // names and defaults, the impedance/admittance/cascade controllers use. This
    // replaced a CONSTANT λ = max(1e-4, damping) in #236 S2b: that constant was
    // the outlier, and converging it onto compliance/task_dynamics.hpp is the
    // point of the migration, not a side effect. Behavioural consequence, stated
    // plainly: away from singularities λ² is now exactly 0 (it was damping²), so
    // Λ is undamped there and the nullspace orthogonality J M⁻¹ Nᵀ = 0 is exact.
    double max_damping{0.05};            ///< λ_max for the DLS ramp
    double singularity_threshold{0.02};  ///< σ₀: DLS engages below this

    // Dynamically-consistent null-space posture task (only meaningful when
    // nv > 6; for a non-redundant 6-DOF arm Nᵀ ≈ 0 so these are inert).
    // τ₀ is summed directly into joint torque (not through M), so these are
    // stiffness/damping in TORQUE units, NOT the acceleration-form gains above.
    double null_kp{0.0};  ///< Posture centering stiffness toward safe_position [N·m/rad]
    double null_kd{1.0};  ///< Null-space joint damping                         [N·m·s/rad]

    // Torque E-STOP hold (E-8, #184): τ = ĝ(q) − D·q̇ clamped to ±τ_max. D bleeds
    // residual kinetic energy while ĝ(q) holds the arm against gravity. Same helper
    // (compliance::GravityCompDampedHold) TaskImpedanceController::ComputeEstop uses.
    double estop_damping{5.0};  ///< D for the torque E-STOP hold ĝ(q) − D·q̇ [N·m·s/rad]

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
  // gains layout: [kp_pos×3, kd_pos×3, kp_rot×3, kd_rot×3, max_damping,
  //                singularity_threshold, enable_gravity(0/1),
  //                trajectory_speed, trajectory_angular_speed,
  //                max_traj_velocity, max_traj_angular_velocity] = 19 values
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

  /// Whether the most recent Compute() ran the null-space posture task.
  ///
  /// This exists because the gate is NUMERICALLY INERT when closed: with
  /// null_kp = null_kd = 0 the posture torque is a signed zero, so Nᵀτ₀ adds
  /// nothing and every torque lane is bit-identical whether the gate is
  /// evaluated or not. A mutation that deletes the gain half of the condition is
  /// therefore invisible to any output comparison — measured, not assumed
  /// (#236 S2b, mutation M2; the same false-green
  /// agent_docs/testing-debug.md §Revert-verification records as its third
  /// type, and the same answer TaskAdmittanceController reached with
  /// diag.nullspace_active). Non-RT reads only; dies with this adapter in S7.
  [[nodiscard]] bool nullspace_active() const noexcept { return nullspace_active_; }

 private:
  // ── Pinocchio via rtc_urdf_bridge ──────────────────────────────────
  std::shared_ptr<const pinocchio::Model> model_ptr_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_frame_id_{0};

  // ── Pre-allocated Eigen work buffers (all sized in the ctor; RT alloc-free) ─
  Eigen::MatrixXd J_full_;  ///< 6×nv: full spatial Jacobian (LOCAL_WORLD_ALIGNED)
  Eigen::MatrixXd Jt_;      ///< nv×6: Jᵀ (materialised for the τ = Jᵀ F product)

  // Joint-space dynamics
  Eigen::MatrixXd M_;        ///< nv×nv: symmetrised joint-space inertia M(q)
  Eigen::VectorXd h_;        ///< nv: nonlinear effects h = C·v + g
  Eigen::VectorXd tau_out_;  ///< nv: joint torque command [N·m]

  // Torque E-STOP hold buffers (#184): gravity-comp damped hold τ = ĝ(q) − D·q̇.
  Eigen::VectorXd gravity_estop_;  ///< nv: ĝ(q) in Pinocchio order (E-STOP hold)
  Eigen::VectorXd grav_dev_;       ///< nv: ĝ(q) scattered to device order
  Eigen::VectorXd qdot_dev_;       ///< nv: measured velocity in device order
  Eigen::VectorXd tau_dev_;        ///< nv: E-STOP hold torque in device order

  // Task-space quantities — fixed 6×1, stack-allocated
  Eigen::Matrix<double, 6, 1> task_err_;  ///< [pos_error(3); rot_error(3)]
  Eigen::Matrix<double, 6, 1> a_task_;    ///< desired task-space acceleration
  Eigen::Matrix<double, 6, 1> tcp_vel_;   ///< current TCP velocity = J · v

  // Λ_S, J̄_S and Nᵀ all come from the shared §6.4–§6.5 helper (#236 S2b); the
  // inline 6×6 factorisation and projector this class used to own are gone.
  compliance::TaskDynamics dyn_;

  // Dynamically-consistent null-space posture task
  Eigen::VectorXd
      q_ref_null_;  ///< nv: posture reference in DEVICE order (safe_position_, 0 past its end)
  Eigen::VectorXd tau0_;  ///< nv: raw posture torque (Pinocchio order, pre-projection)
  Eigen::VectorXd
      tau0_dev_;  ///< nv: posture torque formed in DEVICE order (#172 A2), gathered → tau0_
  Eigen::VectorXd null_tmp_;  ///< nv: projected null-space torque

  // In-place Cholesky factorisation of M — pre-sized, RT alloc-free. TaskDynamics
  // takes the factor rather than M so the nv×nv work happens exactly once.
  Eigen::LLT<Eigen::MatrixXd> llt_M_;  ///< M(q) factor (nv×nv, SPD)

  /// Last tick's posture-gate state — see nullspace_active(). Written on the RT
  /// tick, read non-RT by tests only.
  bool nullspace_active_{false};

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
    // Activation generation observed by the off-RT pusher. The RT drain drops
    // entries a later activation has invalidated — see
    // RTControllerInterface::ActivationGeneration (#196 §3).
    std::uint32_t generation{0};
  };

  static_assert(std::is_trivially_copyable_v<PendingTarget>,
                "PendingTarget must be trivially copyable for SpscQueue");

  static constexpr std::size_t kPendingTargetDepth = 4;

  SeqLock<TargetSlot> target_seqlock_;
  SpscQueue<PendingTarget, kPendingTargetDepth> pending_targets_;
  std::atomic<bool> target_initialized_{false};

  // Activation uses the same signal as ClearEstop: re-seed the pose target and
  // trajectory from the current state on the next tick. Without this a
  // re-activated controller drove back to its pre-deactivation pose.
  void ResetTargetInitialization() noexcept override {
    target_initialized_.store(false, std::memory_order_release);
  }

  bool new_target_pending_{false};  // RT-thread-only; gates trajectory re-init

  std::array<double, 3> tcp_position_{};      ///< diagnostic cache
  std::array<double, 6> pose_error_cache_{};  ///< diagnostic cache

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  std::vector<double> safe_position_;
  std::vector<double> max_joint_velocity_;  ///< from joint_limits [rad/s]; not read by the
                                            ///< torque E-STOP hold (#184) — retained for config
  std::vector<double> max_joint_torque_;    ///< torque command clamp [N·m] (grown to ≥nv)

  // Torque-only: fixed to kTorque. LoadConfig rejects any other command_type.
  CommandType command_type_{CommandType::kTorque};

  // ── Helpers ───────────────────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState& state,
                                              const Gains& gains) noexcept;

  static Eigen::Matrix3d RpyToMatrix(double roll, double pitch, double yaw) noexcept;

  // (Re)build handle_ + all nv-sized Eigen/Cholesky buffers from a Pinocchio
  // model. Called by the ctor (full model) and by MaybeSelectSubModel (reduced
  // submodel) — off-RT only (#172 Phase 3 A1). OSC has no fixed-capacity cap.
  void InitFromModel(std::shared_ptr<const pinocchio::Model> model);

  // Switch handle_ to the primary device's reduced submodel when the injected
  // system model config declares it. No system config / no match → keep the full
  // model from the ctor (no regression).
  void MaybeSelectSubModel();
};

}  // namespace rtc
