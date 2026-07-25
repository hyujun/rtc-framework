// ── Includes: project header first, then third-party, then C++ stdlib ───────
#pragma once

#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/compliance/admittance_integrator.hpp"
#include "rtc_controllers/compliance/compliance_state_machine.hpp"
#include "rtc_controllers/compliance/external_wrench.hpp"
#include "rtc_controllers/compliance/impedance_law.hpp"
#include "rtc_controllers/compliance/safety_limiter.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/compliance/torque_estop.hpp"
#include "rtc_controllers/compliance/wrench_pipeline.hpp"
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
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace rtc {

/// Cascaded compliance controller — **outer task admittance, inner task
/// impedance** (spec §7.6). The structure the specification names as the correct
/// way to make a robot follow an external force, and the one it prefers over
/// §6.3 (adding the measured wrench into the impedance law directly).
///
/// ### Control law
/// @code
///   // OUTER (slow band): the compliant frame, driven by the measured wrench
///   Λ_d ẍ̃_c + K_d^a ẋ̃_c + K_p^a x̃_c = α·f^ext_LWA      [x̃_c = X_c rel. to X_d]
///   X_c = (R̃·R_d, p_d + p̃),   ν_c = ẋ̃_c                [semi-implicit + exp3]
///
///   // INNER (fast band): §6.2 impedance TRACKING that frame
///   e  = computePoseError(X, X_c, SplitWorld)            [current → compliant]
///   τ  = Jᵀ·α[K_p^i·e + K_d^i·(ν_c − ν)] + α·Nᵀτ_posture + ĝ(q)
/// @endcode
///
/// The division of labour is the point: the OUTER loop alone decides how the
/// robot yields to force (that is the compliance the operator tunes), while the
/// INNER loop only has to be stiff and fast enough to realise the frame the
/// outer loop asks for. Compare `TaskImpedanceController`'s §6.3 path, where the
/// wrench enters the same equation as the stiffness and the two objectives are
/// coupled through Λ_d.
///
/// ### The wrench is consumed EXACTLY once (§7.6 MUST-4)
/// The outer loop has already spent `Ŵ_ext`. An inner law that also consumed
/// `f_ext` would count the same physical force twice — roughly doubling the
/// response, with no fault raised because every number stays finite. That is
/// enforced STRUCTURALLY rather than by a runtime check: this controller has no
/// `formulation` axis and no `inertia_shaping` knob, so the §6.3 term through
/// which `f_ext` could re-enter does not exist (D19). `DoubleCountingReference…`
/// pins what the mistake would have cost.
///
/// ### Bandwidth separation (§7.6 MUST-1)
/// A cascade is only meaningful when the inner loop is materially faster than
/// the outer one; the spec asks for ω_i/ω_a ≥ 3. ω_i needs the task inertia
/// Λ_S(q), which does not exist at configure time (there is no q yet), so the
/// ratio is evaluated ONCE on the seeding tick — where M(q) and its Cholesky are
/// already on the pre-allocated path — and reported as the diagnostic flag
/// `bandwidth_ratio_low`. It is deliberately NOT a fault: a low ratio is a
/// tuning statement about two gain sets, not a runtime failure, and latching
/// SAFE_STOP for it would stop a robot that is merely sluggish (D20). RT ticks
/// cannot log (RT-3), which is why this is a flag and not a warning.
///
/// ### Activation ramp α
/// α ramps the wrench into the OUTER loop and the task force out of the INNER
/// one. Both, because each covers a different discontinuity: ramping only the
/// torque lets X_c drift away under a standing load while the arm is still
/// soft, so it lunges when α reaches 1; ramping only the wrench leaves the
/// inner damping term −K_d^i·ν as a torque step at activation. During the ramp
/// the force-driven response is therefore ~α², which is monotone and confined
/// to `activation_ramp_time`. Gravity compensation is never ramped (the arm
/// must not sag).
///
/// ### Scope
/// FULL_SE3 only (no `TaskSelection` axis — same reasoning as
/// `TaskAdmittanceController`), torque output, external wrench REQUIRED: with
/// no force input the outer loop never leaves X_d and the cascade degenerates
/// into a plain §6.2 impedance controller, which already exists. Owns **no node,
/// no subscription and no message type** — the wrench arrives through the
/// non-RT `SetExternalWrench()` setter and crosses to RT through a SeqLock.
class CascadedComplianceController final : public RTControllerInterface {
 public:
  /// Task dimension. Fixed at 6 — see the scope note above.
  static constexpr int kTaskDim = 6;

  // ── Gain / feature configuration (trivially copyable POD for SeqLock) ──────
  struct Gains {
    /// OUTER loop: §7.2 virtual dynamics + §7.4/§7.5 bounds. `stiffness` = K_p^a;
    /// setting it to zero is hand-guiding (§7.6 MUST-3: the frame then STAYS
    /// where the force left it instead of returning to X_d).
    compliance::AdmittanceParams admittance{};

    /// INNER loop: §6.2 Cartesian stiffness / damping. These are the gains that
    /// make the arm track X_c; they are NOT the compliance the operator tunes —
    /// that is `admittance` above.
    compliance::ImpedanceParams impedance{};

    /// §10.6 staleness / contact.
    compliance::WrenchPipelineParams wrench{};

    // Nullspace posture task (bites only when nv > 6), torque domain like the
    // impedance controller: Nᵀ-projected, so it cannot disturb the task.
    double nullspace_kp{0.0};  ///< posture centering stiffness [N·m/rad]
    double nullspace_kd{2.0};  ///< nullspace joint damping [N·m·s/rad]

    // §6.5 σ_min-adaptive DLS for Λ_S (nullspace projector + the §7.6 MUST-1
    // ratio). The inner law itself is Jacobian-transpose and needs no inverse.
    double singularity_threshold{0.02};  ///< σ₀: DLS engages below this (also DEGRADED)
    double singularity_critical{0.005};  ///< σ_min below this → SAFE_STOP
    double max_damping{0.05};            ///< λ_max for the DLS ramp

    // Safety layer (§5.3, §10.5) — torque domain.
    double joint_limit_margin{0.1};  ///< δ [rad]: repulsive band width
    double joint_limit_kp{0.0};      ///< k_lim [N·m/rad]; 0 disables the SPRING term only
    double joint_limit_kd{2.0};      ///< d_lim [N·m·s/rad]; independent of k_lim
    double max_torque_rate{2000.0};  ///< [N·m/s] slew limit (dt-scaled)
    double pose_error_limit{1.5};    ///< ‖e(X, X_c)‖ bound → SAFE_STOP

    // Activation and E-STOP.
    double activation_ramp_time{0.5};     ///< [s] 0→1 linear ramp (§10.7); ≤0 = no ramp
    double estop_damping{5.0};            ///< D for the torque E-STOP hold ĝ(q) − D·q̇ (E-8)
    double saturation_persist_time{0.1};  ///< [s] saturation held longer → DEGRADED

    /// §7.6 MUST-1 minimum ω_i/ω_a. Diagnostic only (see the class note); 0
    /// silences the flag rather than latching anything.
    double min_bandwidth_ratio{3.0};
  };

  /// @param urdf_path  Absolute path to the robot URDF.
  /// @param gains      Outer admittance / inner impedance / safety gains.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  CascadedComplianceController(std::string_view urdf_path, Gains gains);

  /// Same, with the spec-default gains. A `Gains gains = Gains{}` DEFAULT
  /// ARGUMENT cannot be written here: `Gains`'s own member initializers are not
  /// yet parsed inside its enclosing class, so the delegation lives in the .cpp.
  explicit CascadedComplianceController(std::string_view urdf_path);

  // ── RTControllerInterface — all methods noexcept (RT safety) ──────────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;
  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;
  [[nodiscard]] std::string_view Name() const noexcept override;

  /// Publish a measured external wrench (§3.2.1). NON-RT, single producer,
  /// wait-free. RAW `[f;τ]` in the configured `sensor_frame`'s BODY frame, SI
  /// units, sign = the wrench the ENVIRONMENT applies ON the robot. No bias
  /// removal, gravity compensation, frame transform or filtering by the caller —
  /// all of it needs the robot model and happens on the RT side.
  void SetExternalWrench(std::span<const double, compliance::kWrenchDim> wrench) noexcept;

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  void LoadConfig(const YAML::Node& cfg) override;
  void OnDeviceConfigsSet() override;

  [[nodiscard]] CommandType GetCommandType() const noexcept override { return command_type_; }

  /// Clear a LATCHED controller-local SAFE_STOP (the ~/reset_fault service;
  /// wiring deferred, issue #260). Deliberately SEPARATE from ClearEstop (E-8).
  void ResetFault() noexcept { reset_fault_requested_.store(true, std::memory_order_release); }

  // ── Accessors (non-RT reads only) ─────────────────────────────────────────
  void set_gains(const Gains& g) noexcept { gains_lock_.Store(g); }

  [[nodiscard]] Gains get_gains() const noexcept { return gains_lock_.Load(); }

  [[nodiscard]] int task_dim() const noexcept { return kTaskDim; }

  [[nodiscard]] bool external_wrench_enabled() const noexcept { return wrench_enabled_; }

  /// Diagnostic snapshot published every tick (incl. E-STOP / fault ticks) with
  /// per-field validity, so a stale body never rides out under a fresh stamp.
  struct Diagnostics {
    std::uint8_t state{0};  ///< ComplianceState
    double sigma_min{0.0};
    double lambda_sq{0.0};
    /// x̃_c = [p̃ ; log3(R̃)] — the compliant frame relative to X_d (§7.2).
    std::array<double, 6> compliant_deviation{};
    std::array<double, 6> compliant_velocity{};  ///< ν_c, LWA
    std::array<double, 6> pose_error{};          ///< e(X, X_c), LWA — inner tracking error
    /// Conditioned external wrench in LWA at the tip, after the staleness fade
    /// but BEFORE the activation ramp α — the physical estimate of the load, not
    /// the ramped input to the outer loop.
    std::array<double, 6> wrench_lwa{};
    double wrench_age{0.0};
    double wrench_fade{1.0};
    /// Samples the input's finiteness gate dropped since construction. Rising
    /// while `wrench_valid` stays true = an intermittently garbage producer.
    std::uint32_t wrench_rejected{0};
    /// §7.6 MUST-1 min over task axes of ω_i/ω_a, measured at the seeding pose.
    /// Infinite when no axis is evaluable (e.g. K_p^a = 0 on every axis, which
    /// is hand-guiding: an outer loop with no bandwidth to separate from).
    double bandwidth_ratio{std::numeric_limits<double>::infinity()};
    bool bandwidth_ratio_low{false};   ///< ratio < min_bandwidth_ratio (NOT a fault)
    bool displacement_limited{false};  ///< §7.5 barrier engaged on the compliant frame
    bool velocity_limited{false};      ///< §7.5 ‖ẋ̃_c‖ scaled back
    bool saturated{false};             ///< torque saturation this tick
    bool rate_limited{false};          ///< torque slew limit engaged
    bool nullspace_active{false};
    bool wrench_valid{false};
    bool wrench_stale{false};
    bool bias_calibrated{true};
    bool in_contact{false};
    bool estopped{false};
    bool control_valid{false};  ///< false on E-STOP / SAFE_STOP / degenerate ticks
  };

  static_assert(std::is_trivially_copyable_v<Diagnostics>, "Diagnostics must be SeqLock-safe");

  [[nodiscard]] Diagnostics GetDiagnosticsForTesting() const noexcept { return diag_lock_.Load(); }

  /// Bias estimate committed by BIAS_CALIBRATING. Test-only: the RT thread owns
  /// the pipeline, so this is safe to read only between Compute() calls.
  [[nodiscard]] const compliance::Wrench6& GetWrenchBiasForTesting() const noexcept {
    return wrench_.bias();
  }

  /// Force the next Compute() to re-seed from the measured state — the effect
  /// on_activate has in the CM. Test-only entry point for that re-seed.
  void ResetTargetInitializationForTesting() noexcept { ResetTargetInitialization(); }

 private:
  static_assert(std::is_trivially_copyable_v<Gains>,
                "Gains must be trivially copyable for SeqLock");

  void InitFromModel(std::shared_ptr<const pinocchio::Model> model);
  void MaybeSelectSubModel();

  /// E-STOP / SAFE_STOP hold, TORQUE domain: τ = ĝ(q) − D·q̇, clamped (E-8).
  /// Recomputed from the measured state every held tick rather than latched — a
  /// gravity-compensating damper has no pose to remember, which is why the
  /// position-hold latch `TaskAdmittanceController` needs (and the invalidation
  /// rules that come with it) has no counterpart here.
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState& state, bool control_valid,
                                              const Diagnostics& diag, const Gains& gains) noexcept;

  /// The primary device's joint state is unusable this tick. Emits a zero-length
  /// command for device 0 (the CM's own "no update" idiom), keeps secondary
  /// passthrough, steps the FSM into DEGRADED and forces a re-seed. Without this
  /// gate the unread channels read as 0 and the whole law runs at the ZERO
  /// configuration — a full-arm move to the origin with every number finite and
  /// no fault raised.
  [[nodiscard]] ControllerOutput ComputeNoJointState(const ControllerState& state,
                                                     const Gains& gains,
                                                     Diagnostics& diag) noexcept;

  /// §7.6 MUST-1, evaluated on the seeding tick from Λ_S(q₀) (RT, no logging).
  /// Writes bandwidth_ratio_ / bandwidth_ratio_low_.
  void EvaluateBandwidthSeparation(const Gains& gains) noexcept;

  [[nodiscard]] pinocchio::FrameIndex LookupSensorFrame(const std::string& name) const;
  void ResolveSensorFrame();

  // ── Model ─────────────────────────────────────────────────────────────────
  std::shared_ptr<const pinocchio::Model> model_ptr_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_frame_id_{0};

  // Written off-RT (LoadConfig, before activation) and read on the RT tick — the
  // same lifetime contract command_type_ has: the CM configures, then activates.
  bool wrench_enabled_{true};
  std::string sensor_frame_name_;  ///< empty ⇒ the tip frame
  pinocchio::FrameIndex sensor_frame_id_{0};
  Eigen::Vector3d gravity_world_{Eigen::Vector3d::Zero()};  ///< from the model, not 9.81

  compliance::AdmittanceIntegrator integrator_;
  compliance::TaskDynamics dyn_;
  compliance::ComplianceStateMachine sm_;
  compliance::WrenchPipeline wrench_;
  bool bias_gate_{true};  ///< RT-only mirror of the pipeline's FSM gate

  // ── Pre-allocated Eigen work buffers (sized in InitFromModel; RT alloc-free) ─
  Eigen::MatrixXd J_full_;           ///< 6×nv full spatial Jacobian (LOCAL_WORLD_ALIGNED)
  Eigen::MatrixXd M_;                ///< nv×nv symmetrised joint inertia
  Eigen::VectorXd gravity_;          ///< nv ĝ(q)
  Eigen::VectorXd tau_;              ///< nv joint torque (Pinocchio order)
  Eigen::VectorXd tau_posture_dev_;  ///< nv posture torque (device order)
  Eigen::VectorXd tau_posture_;      ///< nv posture torque (Pinocchio order)
  Eigen::VectorXd tau_null_;         ///< nv projected nullspace torque
  Eigen::VectorXd tcp_vel_;          ///< 6 current task twist ν = J·q̇ (LWA)
  Eigen::VectorXd q_null_;           ///< nv posture setpoint (Pinocchio order)
  Eigen::LLT<Eigen::MatrixXd> llt_M_;

  // Device-order buffers.
  Eigen::VectorXd tau_dev_;
  Eigen::VectorXd tau_prev_dev_;  ///< nv rate-limit history (device order)
  Eigen::VectorXd q_dev_;
  Eigen::VectorXd qdot_dev_;
  Eigen::VectorXd grav_dev_;

  // ── Target (desired pose X_d) — flat-double SE3 mirror (SeqLock needs POD) ──
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

  pinocchio::SE3 goal_pose_{pinocchio::SE3::Identity()};             // X_d, RT working copy
  pinocchio::SE3 compliant_pose_{pinocchio::SE3::Identity()};        // X_c, RT working copy
  double activation_elapsed_{0.0};                                   // RT-only: ramp accumulator
  double saturation_elapsed_{0.0};                                   // RT-only: saturation-persist
  double bandwidth_ratio_{std::numeric_limits<double>::infinity()};  // RT-only (§7.6 MUST-1)
  bool bandwidth_ratio_low_{false};                                  // RT-only

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
