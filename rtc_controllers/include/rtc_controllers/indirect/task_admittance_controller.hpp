// ── Includes: project header first, then third-party, then C++ stdlib ───────
#pragma once

#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/compliance/admittance_integrator.hpp"
#include "rtc_controllers/compliance/compliance_state_machine.hpp"
#include "rtc_controllers/compliance/differential_ik.hpp"
#include "rtc_controllers/compliance/external_wrench.hpp"
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
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace rtc {

/// Task-space **admittance** controller — force in, motion out (spec §7, Rule 3).
///
/// ### Control law
/// @code
///   // 1. compliant frame X_c, integrated from the measured external wrench (§7.2)
///   Λ_d ẍ̃_c + K_d ẋ̃_c + K_p x̃_c = f^ext_LWA        [x̃_c = X_c relative to X_d]
///   X_c = (R̃·R_d, p_d + p̃)                          [semi-implicit Euler + exp3 retract]
///
///   // 2. differential IK onto the compliant frame (§7.3 방식 1)
///   ν  = ν_c + K^ik·e(X, X_c)                       [e via SplitWorld = LWA]
///   q̇  = J⁺ν + (I − J⁺J)·K^n(q_null − q)            [J⁺ = §6.5 σ_min-adaptive DLS]
///   q_cmd = q_meas + q̇·Δt        (integrate_from_measured: true, the default)
///         | q_cmd,prev + q̇·Δt    (false — guarded by a ‖q_cmd − q_meas‖ bound)
/// @endcode
///
/// ### Admittance vs impedance — the same push moves the arm the OTHER way
/// Under the shared input contract (positive = the wrench the environment applies
/// ON the robot) a `+x` push makes `TaskImpedanceController` command a `−x`
/// torque (it RESISTS, §11.4.1 P2) and makes this controller move `+x` (it
/// FOLLOWS, P3). Both are correct; they are different control objectives, and
/// `AdmittanceFollowsThePushWhileImpedanceResistsIt` pins the pair on one fixture
/// so a sign regression in either cannot hide.
///
/// ### Why position output and not a new `CommandType::kVelocity` (D14)
/// §7.3's own formula ends in `q_cmd = q_meas + q̇·Δt`, so the integration belongs
/// here; adding a velocity command type would touch `rtc_base`'s enum, the
/// `-Wswitch`-enforced `CommandTypeToString`, `DeviceBackend::AcceptsCommandType`
/// and the simulator's separate actuator-mode enum — 3–4 packages for no gain.
/// Position output also makes this the ONLY compliance path that a real
/// position-only backend can currently accept at all (D15).
///
/// ### Scope (slice 3)
/// FULL_SE3 only — there is no `TaskSelection` axis here. Under
/// `TRANSLATION_ONLY` the orientation of the compliant frame would be integrated
/// from torques that no task then regulates, and the honest way to add it is
/// with the nullspace-posture contract §6.1 spells out, as a later slice.
/// External wrench is REQUIRED (§7.1: axis A ≠ NONE) — `external_wrench.enabled:
/// false` is a configure error, unlike in the impedance controller where A=NONE
/// is a first-class law.
///
/// Owns **no node, no subscription and no message type**: the wrench arrives
/// through the non-RT `SetExternalWrench()` setter (the `SetDeviceTarget` idiom)
/// and crosses to the RT tick through a SeqLock.
class TaskAdmittanceController final : public RTControllerInterface {
 public:
  /// Task dimension. Fixed at 6 — see the scope note above.
  static constexpr int kTaskDim = 6;

  // ── Gain / feature configuration (trivially copyable POD for SeqLock) ──────
  struct Gains {
    /// §7.2 virtual dynamics + §7.4/§7.5 bounds.
    compliance::AdmittanceParams admittance{};

    /// §10.6 staleness / contact.
    compliance::WrenchPipelineParams wrench{};

    // ── §7.3 differential IK ─────────────────────────────────────────────────
    /// Closed-loop pose gain on e(X, X_c) [1/s]. §7.3's formula is pure
    /// feedforward (`ν = ν_c`); with a tracking error that never gets corrected
    /// the arm drifts away from the compliant frame it is supposed to realise,
    /// so a CLIK term is added here and reported as a spec gap (§12). Set to 0
    /// to reproduce §7.3 literally.
    std::array<double, 3> ik_kp_pos{{2.0, 2.0, 2.0}};
    std::array<double, 3> ik_kp_rot{{2.0, 2.0, 2.0}};
    /// Nullspace posture centering toward q_null [1/s]; bites only when nv > 6.
    double nullspace_kp{0.5};
    /// `true` (§7.3 default): q_cmd = q_meas + q̇Δt — no position windup, but a
    /// lagging lower controller drags the command back. `false`: integrate the
    /// command itself (smoother, can wind away from the arm), which is why it is
    /// the only mode guarded by `command_divergence_limit`.
    bool integrate_from_measured{true};

    // ── §6.5 σ_min-adaptive DLS ──────────────────────────────────────────────
    double singularity_threshold{0.02};  ///< σ₀: DLS engages below this (also DEGRADED)
    double singularity_critical{0.005};  ///< σ_min below this → SAFE_STOP
    double max_damping{0.05};            ///< λ_max for the DLS ramp

    // ── Safety / activation ──────────────────────────────────────────────────
    double pose_error_limit{0.5};          ///< ‖e(X, X_c)‖ bound → SAFE_STOP
    double command_divergence_limit{0.5};  ///< ‖q_cmd − q_meas‖ [rad] → SAFE_STOP
    double joint_limit_margin{0.0};        ///< δ [rad] shrinking the q_cmd clamp band
    double activation_ramp_time{0.5};      ///< [s] wrench 0→1 linear ramp (§10.7); ≤0 = none
    double saturation_persist_time{0.1};   ///< [s] velocity clamp held longer → DEGRADED
  };

  /// @param urdf_path  Absolute path to the robot URDF.
  /// @param gains      Admittance / IK / safety gains.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  TaskAdmittanceController(std::string_view urdf_path, Gains gains);

  /// Same, with the spec-default gains. A `Gains gains = Gains{}` DEFAULT
  /// ARGUMENT cannot be written here: `Gains`'s own member initializers are not
  /// yet parsed inside its enclosing class, so the overload does the delegation
  /// from the .cpp instead.
  explicit TaskAdmittanceController(std::string_view urdf_path);

  // ── RTControllerInterface — all methods noexcept (RT safety) ──────────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;
  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;
  [[nodiscard]] std::string_view Name() const noexcept override;

  /// Publish a measured external wrench (§3.2.1). NON-RT, single producer,
  /// wait-free. RAW `[f;τ]` in the configured `sensor_frame`'s BODY frame, SI
  /// units, sign = the wrench the ENVIRONMENT applies ON the robot. No bias
  /// removal, gravity compensation, frame transform or filtering by the caller.
  void SetExternalWrench(std::span<const double, compliance::kWrenchDim> wrench) noexcept;

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  void LoadConfig(const YAML::Node& cfg) override;
  void OnDeviceConfigsSet() override;

  [[nodiscard]] CommandType GetCommandType() const noexcept override { return command_type_; }

  /// Clear a LATCHED controller-local SAFE_STOP (the ~/reset_fault service;
  /// wiring deferred). Deliberately SEPARATE from ClearEstop (E-8).
  void ResetFault() noexcept {
    reset_fault_requested_.store(true, std::memory_order_release);
    InvalidateEstopHold();
  }

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
    std::array<double, 6> pose_error{};          ///< e(X, X_c), LWA — IK tracking error
    /// Conditioned external wrench in LWA at the tip, after the staleness fade
    /// but BEFORE the activation ramp α.
    std::array<double, 6> wrench_lwa{};
    double wrench_age{0.0};
    double wrench_fade{1.0};
    /// Samples the input's finiteness gate dropped since construction. Rising
    /// while `wrench_valid` stays true = an intermittently garbage producer,
    /// which is otherwise invisible (a dropped sample simply never arrives).
    std::uint32_t wrench_rejected{0};
    double command_divergence{0.0};      ///< ‖q_cmd − q_meas‖ [rad]
    bool displacement_limited{false};    ///< §7.5 saturating spring engaged
    bool velocity_limited{false};        ///< §7.5 ‖ẋ̃_c‖ scaled back
    bool joint_velocity_limited{false};  ///< a joint hit its device velocity bound
    bool nullspace_active{false};
    bool wrench_valid{false};
    bool wrench_stale{false};
    bool bias_calibrated{true};
    bool in_contact{false};
    bool estopped{false};
    bool control_valid{false};  ///< false on E-STOP / SAFE_STOP / degenerate-IK ticks
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

  /// E-STOP / SAFE_STOP hold. POSITION-domain (this controller emits positions,
  /// so `compliance/torque_estop.hpp`'s ĝ(q) − D·q̇ does not apply), and a true
  /// HOLD: the command is LATCHED at the pose measured on the first held tick
  /// and repeated. Re-emitting q_meas every tick would look like a hold and
  /// behave like a free joint — a backdrivable arm pushed during the stop would
  /// have its command follow it, with nothing left to resist. That is also why
  /// this does not reuse CLIK's ComputeEstop, which SLEWS toward safe_position:
  /// a compliance controller must not start a motion the operator did not ask
  /// for at the moment the E-STOP fires.
  ///
  /// The latch describes a pose the arm was at. Any boundary that can move the
  /// arm while this controller is not driving it — a deactivate/reactivate, an
  /// E-STOP clear, a fault reset — therefore has to retire it, or the first held
  /// tick after the boundary commands the OLD pose in one unbounded step
  /// (`InvalidateEstopHold`, E-8). What the latch must survive is the per-tick
  /// re-seed a latched SAFE_STOP forces; those are different events, which is
  /// why the request below is separate from `target_initialized_`.
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState& state, const Gains& gains,
                                              bool control_valid, const Diagnostics& diag) noexcept;

  /// The primary device's joint state is unusable this tick. Emits a zero-length
  /// command for device 0 (the "no update" idiom the CM's own BuildHoldOutput
  /// uses) while secondary devices keep their passthrough, steps the FSM into
  /// DEGRADED, and forces the next controllable tick to re-seed.
  [[nodiscard]] ControllerOutput ComputeNoJointState(const ControllerState& state,
                                                     const Gains& gains,
                                                     Diagnostics& diag) noexcept;

  /// Discard every queued off-RT target. Both held paths (E-STOP, unusable joint
  /// state) force a measured-pose re-seed on the next controllable tick, and
  /// neither bumps the activation generation — so a target pushed DURING the hold
  /// still passes IsCurrentGeneration and would overwrite that re-seed on the
  /// tick right after it. The RT thread is the sole SPSC consumer, so draining
  /// here is what keeps "a command issued while held does not survive recovery"
  /// true for both paths rather than only for E-STOP.
  void DrainPendingTargets() noexcept;

  /// Ask the RT thread to retire the hold latch on its next tick. Callable from
  /// any thread; `estop_hold_*` itself stays RT-owned (RT-4 — no lock, and no
  /// off-RT writer to race with).
  void InvalidateEstopHold() noexcept {
    estop_hold_invalidate_.store(true, std::memory_order_release);
  }

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
  compliance::DifferentialIk ik_;
  compliance::ComplianceStateMachine sm_;
  compliance::WrenchPipeline wrench_;
  bool bias_gate_{true};  ///< RT-only mirror of the pipeline's FSM gate

  // ── Pre-allocated Eigen work buffers (sized in InitFromModel; RT alloc-free) ─
  Eigen::MatrixXd J_full_;         ///< 6×nv full spatial Jacobian (LOCAL_WORLD_ALIGNED)
  Eigen::VectorXd dq_;             ///< nv joint velocity command (Pinocchio order)
  Eigen::VectorXd dq_dev_;         ///< nv joint velocity command (device order)
  Eigen::VectorXd qdot_null_;      ///< nv posture velocity (Pinocchio order)
  Eigen::VectorXd qdot_null_dev_;  ///< nv posture velocity (device order)
  Eigen::VectorXd q_null_;         ///< nv posture setpoint (device order), seeded on activate
  Eigen::VectorXd q_dev_;          ///< nv measured position (device order)
  Eigen::VectorXd desired_q_;      ///< nv integrated joint command (device order)
  /// nv integration base of the CURRENT tick (device order) — q_meas or the
  /// previous command per `integrate_from_measured`. Held because the post-clamp
  /// rate bound has to measure the step against it after desired_q_ has been
  /// overwritten.
  Eigen::VectorXd q_base_;

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

  pinocchio::SE3 goal_pose_{pinocchio::SE3::Identity()};       // X_d, RT working copy
  pinocchio::SE3 compliant_pose_{pinocchio::SE3::Identity()};  // X_c, RT working copy
  double activation_elapsed_{0.0};                             // RT-only: wrench-ramp accumulator
  double saturation_elapsed_{0.0};  // RT-only: velocity-clamp persist accumulator
  std::array<double, kMaxDeviceChannels> estop_hold_{};  // RT-only: latched hold command
  // RT-only: the hold command actually EMITTED last tick. The rate bound below
  // is taken against this and not against q_meas — bounding against the measured
  // position would make the hold follow a backdriven arm, which is the failure
  // the latch exists to prevent.
  std::array<double, kMaxDeviceChannels> estop_last_command_{};
  bool estop_hold_valid_{false};
  std::atomic<bool> estop_hold_invalidate_{false};

  void ResetTargetInitialization() noexcept override {
    target_initialized_.store(false, std::memory_order_release);
    // on_activate: the arm may have been moved by another controller since this
    // one last ran, so a hold latched before now describes nothing (E-8).
    InvalidateEstopHold();
  }

  // Per-device limits (device channel order).
  std::vector<double> max_joint_velocity_;
  std::vector<double> position_lower_;
  std::vector<double> position_upper_;

  CommandType command_type_{CommandType::kPosition};
};

}  // namespace rtc
