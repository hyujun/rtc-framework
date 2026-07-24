// ── Includes: project header first, then third-party, then C++ stdlib ───────
#pragma once

#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/compliance/compliance_state_machine.hpp"
#include "rtc_controllers/compliance/safety_limiter.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/compliance/torque_estop.hpp"
#include <rtc_base/concurrency/spsc_queue.hpp>
#include <rtc_base/threading/seqlock.hpp>
#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/rt_model_handle.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace rtc {

/// Task-space impedance controller — Cartesian compliance via the
/// Jacobian-transpose stiffness law (spec §6.2, A=NONE, no external wrench).
///
/// ### Control law (`kJacobianTranspose`, A=NONE)
/// @code
///   e  = computePoseError(X, X_d, SplitWorld)     [6D, LOCAL_WORLD_ALIGNED]
///   ė  = ν_d − ν = −J·q̇                            [ν_d = 0 for a static setpoint]
///   S  = selection matrix (FULL_SE3: I₆ | TRANSLATION_ONLY: [I₃ 0])
///   τ  = Jᵀ Sᵀ[K_p·S·e + K_d·S·ė] + τ_null + ĝ(q)
///     τ_null = Nᵀ·[K_pⁿ(q_null − q) − K_dⁿ·q̇]     [nv > m only]
/// @endcode
///
/// Unlike the OSC (`F = Λ·a_task`, acceleration-gain form), this law does NOT
/// use the task inertia Λ, so it is **free of task-space singularities** — Λ only
/// enters the dynamically-consistent nullspace projector (task_dynamics.hpp),
/// whose σ_min-adaptive DLS keeps it finite there too. Output is **N·m**; the
/// command type is fixed to `kTorque` and the scope is the **MuJoCo backend**
/// (no torque-capable real backend is bound — see docs/compliance-conventions).
///
/// `TRANSLATION_ONLY` regulates position only; orientation is left to the
/// nullspace posture task, so `nullspace_stiffness == 0` with `TRANSLATION_ONLY`
/// is a **configure error** (§6.1 — rotation would drift uncontrolled).
class TaskImpedanceController final : public RTControllerInterface {
 public:
  /// Task selection (axis B): which Cartesian DoF the impedance law regulates.
  enum class TaskSelection : std::uint8_t { kFullSe3, kTranslationOnly };

  // ── Gain / feature configuration (trivially copyable POD for SeqLock) ──────
  struct Gains {
    // Cartesian stiffness/damping, per axis. K_p is a stiffness [N/m or N·m/rad],
    // K_d a damping [N·s/m or N·m·s/rad] — NOT the OSC acceleration-form gains.
    std::array<double, 3> kp_pos{{200.0, 200.0, 200.0}};  ///< translation stiffness
    std::array<double, 3> kd_pos{{28.0, 28.0, 28.0}};     ///< translation damping (≈2√kp)
    std::array<double, 3> kp_rot{{20.0, 20.0, 20.0}};     ///< rotation stiffness (FULL_SE3 only)
    std::array<double, 3> kd_rot{{9.0, 9.0, 9.0}};        ///< rotation damping (FULL_SE3 only)

    // Nullspace posture task (bites only when nv > task DoF m). Kd ≥ 2√Kp is
    // enforced in LoadConfig (§6.4); Kp = 0 with Kd > 0 (pure damping) is allowed
    // EXCEPT under TRANSLATION_ONLY (§6.1).
    double nullspace_kp{0.0};  ///< posture centering stiffness toward q_null [N·m/rad]
    double nullspace_kd{2.0};  ///< nullspace joint damping [N·m·s/rad]

    // σ_min-adaptive DLS for the nullspace Λ_S (§6.5).
    double singularity_threshold{0.02};  ///< σ₀: DLS engages below this (also DEGRADED)
    double singularity_critical{0.005};  ///< σ_min below this → SAFE_STOP
    double max_damping{0.05};            ///< λ_max for the DLS ramp

    // Safety layer (§5.3, §10.5).
    double joint_limit_margin{0.1};  ///< δ [rad]: repulsive band width
    double joint_limit_kp{0.0};      ///< k_lim [N·m/rad]; 0 disables the SPRING term only
    double joint_limit_kd{
        2.0};  ///< d_lim [N·m·s/rad]; independent of k_lim (pure damping if k_lim=0)
    double max_torque_rate{2000.0};  ///< [N·m/s] slew limit (dt-scaled, never 500 Hz)
    double pose_error_limit{1.5};    ///< ‖e‖ bound → SAFE_STOP when exceeded

    // Activation and E-STOP.
    double activation_ramp_time{0.5};     ///< [s] gain 0→1 linear ramp (§10.7); ≤0 = no ramp
    double estop_damping{5.0};            ///< D for the torque E-STOP hold ĝ(q) − D·q̇ (E-8)
    double saturation_persist_time{0.1};  ///< [s] saturation held longer → DEGRADED
  };

  /// @param urdf_path  Absolute path to the robot URDF.
  /// @param gains      Impedance / nullspace / safety gains.
  /// @param selection  FULL_SE3 (6-DoF task) or TRANSLATION_ONLY (position task).
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  TaskImpedanceController(std::string_view urdf_path, Gains gains,
                          TaskSelection selection = TaskSelection::kFullSe3);

  // ── RTControllerInterface — all methods noexcept (RT safety) ──────────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;
  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;
  [[nodiscard]] std::string_view Name() const noexcept override;

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  void LoadConfig(const YAML::Node& cfg) override;
  void OnDeviceConfigsSet() override;

  [[nodiscard]] CommandType GetCommandType() const noexcept override { return command_type_; }

  // Clear a LATCHED controller-local SAFE_STOP fault (the ~/reset_fault service;
  // wiring deferred). Deliberately SEPARATE from ClearEstop (E-8): a global
  // E-STOP clear must NOT release a controller fault, and vice versa.
  void ResetFault() noexcept { reset_fault_requested_.store(true, std::memory_order_release); }

  // ── Accessors (non-RT reads only) ─────────────────────────────────────────
  void set_gains(const Gains& g) noexcept { gains_lock_.Store(g); }

  [[nodiscard]] Gains get_gains() const noexcept { return gains_lock_.Load(); }

  [[nodiscard]] TaskSelection selection() const noexcept { return selection_; }

  [[nodiscard]] int task_dim() const noexcept { return m_; }

  // Diagnostic snapshot published every tick (incl. E-STOP / early return) with
  // per-field validity, so a stale body never rides out under a fresh stamp.
  struct Diagnostics {
    std::uint8_t state{0};  ///< ComplianceState
    double sigma_min{0.0};
    double lambda_sq{0.0};
    std::array<double, 6> pose_error{};
    bool saturated{false};
    bool rate_limited{false};
    bool nullspace_active{false};
    bool estopped{false};
    bool control_valid{false};  ///< false on E-STOP / degenerate-dynamics ticks
  };

  static_assert(std::is_trivially_copyable_v<Diagnostics>, "Diagnostics must be SeqLock-safe");

  [[nodiscard]] Diagnostics GetDiagnosticsForTesting() const noexcept { return diag_lock_.Load(); }

  // Force the next Compute() to re-seed the desired pose / posture from the
  // measured state — the effect on_activate has in the CM (which also bumps the
  // activation generation). Test-only entry point for that re-seed.
  void ResetTargetInitializationForTesting() noexcept { ResetTargetInitialization(); }

 private:
  static_assert(std::is_trivially_copyable_v<Gains>,
                "Gains must be trivially copyable for SeqLock");

  // (Re)build handle_ + all nv-sized buffers from a model. Off-RT (ctor / submodel).
  void InitFromModel(std::shared_ptr<const pinocchio::Model> model);
  void MaybeSelectSubModel();
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState& state, bool control_valid,
                                              const Diagnostics& diag) noexcept;

  // ── Model ──────────────────────────────────────────────────────────────────
  std::shared_ptr<const pinocchio::Model> model_ptr_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_frame_id_{0};

  const TaskSelection selection_;
  int m_{6};  ///< task dimension (6 = FULL_SE3, 3 = TRANSLATION_ONLY)

  compliance::TaskDynamics dyn_;
  compliance::ComplianceStateMachine sm_;

  // ── Pre-allocated Eigen work buffers (sized in InitFromModel; RT alloc-free) ─
  Eigen::MatrixXd J_full_;           ///< 6×nv full spatial Jacobian (LOCAL_WORLD_ALIGNED)
  Eigen::MatrixXd J_S_;              ///< m×nv selected Jacobian S·J
  Eigen::MatrixXd M_;                ///< nv×nv symmetrised joint inertia
  Eigen::VectorXd gravity_;          ///< nv ĝ(q)
  Eigen::VectorXd tau_;              ///< nv joint torque (Pinocchio order)
  Eigen::VectorXd tau_posture_dev_;  ///< nv posture torque (device order)
  Eigen::VectorXd tau_posture_;      ///< nv posture torque (Pinocchio order)
  Eigen::VectorXd tau_null_;         ///< nv projected nullspace torque
  Eigen::VectorXd tcp_vel_;          ///< 6 current task twist J·q̇ (LWA)
  Eigen::VectorXd q_null_;           ///< nv posture setpoint (Pinocchio order), seeded on activate
  Eigen::LLT<Eigen::MatrixXd> llt_M_;

  // Device-order safety buffers.
  Eigen::VectorXd tau_dev_;       ///< nv command in device order
  Eigen::VectorXd tau_prev_dev_;  ///< nv rate-limit history (device order)
  Eigen::VectorXd q_dev_;         ///< nv measured position (device order)
  Eigen::VectorXd qdot_dev_;      ///< nv measured velocity (device order)
  Eigen::VectorXd grav_dev_;      ///< nv gravity in device order (for E-STOP hold)

  // ── Target (desired pose) — flat-double SE3 mirror (SeqLock needs POD) ─────
  static constexpr std::size_t kSE3RotDoubles = 9;
  static constexpr std::size_t kSE3TransDoubles = 3;

  struct TargetSlot {
    std::array<double, kSE3RotDoubles> goal_rot{};  // 3×3 col-major
    std::array<double, kSE3TransDoubles> goal_t{};
    std::array<std::array<double, kMaxDeviceChannels>, ControllerState::kMaxDevices> targets{};
  };

  static_assert(std::is_trivially_copyable_v<TargetSlot>, "TargetSlot must be SeqLock-safe");

  struct PendingTarget {
    int device_idx{0};
    int num_values{0};
    std::array<double, kMaxDeviceChannels> values{};
    std::uint32_t generation{0};
  };

  static_assert(std::is_trivially_copyable_v<PendingTarget>,
                "PendingTarget must be SpscQueue-safe");
  static constexpr std::size_t kPendingTargetDepth = 4;

  SeqLock<Gains> gains_lock_;
  SeqLock<TargetSlot> target_seqlock_;
  SeqLock<Diagnostics> diag_lock_;
  SpscQueue<PendingTarget, kPendingTargetDepth> pending_targets_;
  std::atomic<bool> target_initialized_{false};
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};
  std::atomic<bool> reset_fault_requested_{false};

  pinocchio::SE3 goal_pose_{pinocchio::SE3::Identity()};  // RT working copy
  double activation_elapsed_{0.0};                        // RT-only: gain-ramp accumulator
  double saturation_elapsed_{0.0};                        // RT-only: saturation-persist accumulator

  void ResetTargetInitialization() noexcept override {
    target_initialized_.store(false, std::memory_order_release);
  }

  // Per-device limits (device channel order).
  std::vector<double> max_joint_torque_;
  std::vector<double> max_joint_velocity_;
  std::vector<double> position_lower_;
  std::vector<double> position_upper_;

  CommandType command_type_{CommandType::kTorque};
};

}  // namespace rtc
