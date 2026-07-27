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

#include "rtc_controllers/compliance/differential_ik.hpp"
#include "rtc_controllers/trajectory/task_space_trajectory.hpp"

#include <Eigen/Core>

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
///              +             N        * null_kp * (q_null − q)   [secondary task]
///
/// J_pos^# and N are formed by `compliance::DifferentialIk` (#236 S3b / #258) —
/// the same object TaskAdmittanceController uses, including its §6.5
/// σ_min-adaptive λ². The posture term is `joint::ComputePostureVelocity`.
///
///   q_des       += clamp(dq, ±v_max) * dt        [q_des = q_actual at
///   trajectory init] q_cmd        = q_des
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
  struct Gains {
    std::array<double, 3> kp_translation{
        {1.0, 1.0, 1.0}};  ///< Translation proportional gain (x,y,z) [1/s]
    std::array<double, 3> kp_rotation{
        {1.0, 1.0, 1.0}};  ///< Rotation proportional gain (rx,ry,rz) [1/s]
    // §6.5 σ_min-adaptive DLS damping — the same two parameters, with the same
    // names and defaults, the impedance/admittance/cascade controllers use. This
    // replaced a CONSTANT λ in #236 S3b (issue #258): that constant was the
    // outlier, and converging it onto compliance/differential_ik.hpp is the
    // point of the migration, not a side effect. Behavioural consequence, stated
    // plainly: away from singularities λ² is now exactly 0 (it was damping²), so
    // J⁺ is the undamped pseudoinverse there and tracking is not biased.
    double max_damping{0.05};            ///< λ_max for the DLS ramp
    double singularity_threshold{0.02};  ///< σ₀: DLS engages below this
    double null_kp{0.5};                 ///< Null-space joint-centering gain [1/s]
    bool enable_null_space{true};        ///< Enable null-space secondary task
    bool control_6dof{false};            ///< Enable 6-DOF (translation + orientation) control

    // Trajectory speed
    double trajectory_speed{0.1};  ///< TCP translational speed for trajectory duration [m/s]
    double trajectory_angular_speed{0.5};  ///< TCP rotational speed for trajectory duration [rad/s]

    // Trajectory velocity limits
    double max_traj_velocity{0.5};          ///< Max TCP velocity during task-space trajectory [m/s]
    double max_traj_angular_velocity{1.0};  ///< Max TCP angular velocity during trajectory [rad/s]
  };

  /// @param urdf_path  Absolute path to the robot URDF file.
  /// @param gains      Gains and feature flags.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  explicit ClikController(std::string_view urdf_path, Gains gains);

  // ── RTControllerInterface — all methods are noexcept (RT safety) ──────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;

  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override;

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  // ── Controller registry hooks ────────────────────────────────────────────
  // gains layout: [kp_translation×3, kp_rotation×3, max_damping,
  //                singularity_threshold, null_kp,
  //                enable_null_space(0/1), control_6dof(0/1),
  //                trajectory_speed, trajectory_angular_speed,
  //                max_traj_velocity, max_traj_angular_velocity] = 15 values
  void LoadConfig(const YAML::Node& cfg) override;
  void OnDeviceConfigsSet() override;

  [[nodiscard]] CommandType GetCommandType() const noexcept override { return command_type_; }

  // ── Accessors (non-RT reads only) ─────────────────────────────────────────
  void set_gains(const Gains& g) noexcept { gains_lock_.Store(g); }

  [[nodiscard]] Gains get_gains() const noexcept { return gains_lock_.Load(); }

  /// Cached TCP position (world frame) from the most recent Compute().
  [[nodiscard]] std::array<double, 3> tcp_position() const noexcept { return tcp_position_; }

  /// Cached 3D Cartesian position error from the most recent Compute().
  [[nodiscard]] std::array<double, 3> position_error() const noexcept {
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
  Eigen::VectorXd dq_;         ///< nv: joint velocity command
  Eigen::VectorXd desired_q_;  ///< nv: integrated desired joint position
  Eigen::VectorXd traj_dq_;    ///< nv: feedforward-only trajectory velocity (for logging)
  Eigen::VectorXd null_dq_dev_;  ///< nv: posture velocity kp·(q_null − q) in DEVICE order (#172 A2)
  Eigen::VectorXd qdot_null_;  ///< nv: the same, gathered to Pinocchio order for the N product
  Eigen::Vector3d pos_error_;  ///< 3: Cartesian position error
  Eigen::Matrix<double, 6, 1> pos_error_6d_;  ///< 6: Cartesian position+orientation error

  // J⁺ and N come from the shared §6.5/§7.3 kinematic helper (#236 S3b / #258);
  // the inline constant-λ LDLT blocks this class used to own are gone. TWO
  // instances because `control_6dof` is a SeqLock gain that set_gains() may flip
  // between ticks: both task dimensions must already be sized, since Resize()
  // allocates and is off-RT only.
  compliance::DifferentialIk ik_6d_;  ///< m = 6 (position + orientation)
  compliance::DifferentialIk ik_3d_;  ///< m = 3 (position only)

  // ── Controller state ──────────────────────────────────────────────────────
  SeqLock<Gains> gains_lock_;

  // RT-thread-only working copy of the SE3 target pose. Materialised from
  // the SeqLock POD each tick so the existing Eigen-based maths can index
  // ::translation() / ::rotation() directly.
  pinocchio::SE3 tcp_target_pose_{pinocchio::SE3::Identity()};

  // TargetSlot — SE3 mirrored as flat doubles (Eigen is_trivially_copyable
  // false-negative). RT thread is the SOLE writer of target_seqlock_; off-RT
  // writers push onto pending_targets_.
  static constexpr std::size_t kSE3RotDoubles = 9;
  static constexpr std::size_t kSE3TransDoubles = 3;

  struct TargetSlot {
    std::array<double, 3> tcp_target{};
    std::array<double, kSE3RotDoubles> tcp_target_rot{};
    std::array<double, kSE3TransDoubles> tcp_target_t{};
    std::array<double, kMaxRobotDOF> null_target{};
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

  // Initial null-space reference seeded from safe_position in
  // OnDeviceConfigsSet (off-RT, before any Compute()). The RT thread copies
  // this into TargetSlot::null_target during first-tick self-init.
  std::array<double, kMaxRobotDOF> null_target_init_{};

  std::array<double, 6> pose_error_cache_{};  ///< diagnostic cache
  std::array<double, 3> tcp_position_{};      ///< diagnostic cache

  std::atomic<bool> target_initialized_{false};

  // Every activation re-seeds the pose target + trajectory from the current
  // state. CLIK previously had no on_activate override and no ClearEstop reset,
  // so a re-activated controller replayed its pre-deactivation pose target.
  void ResetTargetInitialization() noexcept override {
    target_initialized_.store(false, std::memory_order_release);
  }

  bool new_target_pending_{false};  // RT-thread-only; gates trajectory re-init
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
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState& state) noexcept;

  void ClampVelocity(std::array<double, kMaxDeviceChannels>& dq, int n) const noexcept;

  // (Re)build handle_ + all nv-sized Eigen buffers from a Pinocchio model,
  // enforcing the fixed-capacity check. Called by the ctor (full model) and by
  // MaybeSelectSubModel (reduced submodel) — off-RT only (#172 Phase 3 A1).
  void InitFromModel(std::shared_ptr<const pinocchio::Model> model);

  // Switch handle_ to the primary device's reduced submodel when the injected
  // system model config declares it. No system config / no match → keep the full
  // model from the ctor (no regression).
  void MaybeSelectSubModel();
};

}  // namespace rtc
