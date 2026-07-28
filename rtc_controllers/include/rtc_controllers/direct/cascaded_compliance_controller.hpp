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
#include "rtc_controllers/params/cascaded_compliance_params.hpp"
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
/// ### Activation ramp α — two clocks, not one
/// α ramps the wrench into the OUTER loop and the task force out of the INNER
/// one. Both, because each covers a different discontinuity: ramping only the
/// torque lets X_c drift away under a standing load while the arm is still
/// soft, so it lunges when α reaches 1; ramping only the wrench leaves the
/// inner damping term −K_d^i·ν as a torque step at activation. Gravity
/// compensation is never ramped (the arm must not sag).
///
/// The two share `activation_ramp_time` but NOT a clock. `α_track` (inner) runs
/// from the seeding tick, as the tracking discontinuity is present from that
/// tick. `α_wrench` (outer) runs only from the first tick the pipeline actually
/// emits a force — i.e. once a sample has arrived AND the §3.2.1 bias average is
/// committed. A single clock silently defeats the ramp on the path that needs it
/// most: BIAS_CALIBRATING returns a ZERO wrench for `bias_calibration_samples`
/// producer samples (100 at 100 Hz ≈ 1 s, and it re-runs on every re-seed —
/// including after each E-STOP), which is longer than the default 0.5 s ramp, so
/// α would already be 1 when the first real force arrives and the outer loop
/// would take it as a step. Freezing the SHARED clock instead is worse than the
/// bug: it also zeroes the inner force, leaving the arm on gravity comp alone —
/// free-floating — for that same second.
///
/// Both clocks advance only on clean control ticks, so the force-driven response
/// is still ~α_track·α_wrench, monotone, and each factor is confined to
/// `activation_ramp_time` from the moment its own discontinuity begins.
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

  /// The gains POD lives beside the laws rather than inside this adapter —
  /// see params/cascaded_compliance_params.hpp for why (#236 S7c-2, D-B/G2).
  /// The alias keeps every existing spelling
  /// (`CascadedComplianceController::Gains`) resolving to the lifted
  /// definition.
  using Gains = params::CascadedComplianceParams;

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

  /// Clear a LATCHED controller-local SAFE_STOP (the /rtc_cm/reset_fault
  /// service). Deliberately SEPARATE from ClearEstop (E-8). The flag is consumed
  /// at the head of the next Compute(), ahead of the E-STOP early return, so a
  /// controller fault can be cleared while the global latch is up.
  void ResetFault() noexcept override {
    reset_fault_requested_.store(true, std::memory_order_release);
  }

  /// SAFE_STOP is the only latch this controller keeps (§10.6). Read off-RT from
  /// the per-tick diagnostic snapshot rather than from sm_, which the RT thread
  /// owns; every Compute() exit path stores it. A controller that has not ticked
  /// yet reads as not latched.
  [[nodiscard]] bool HasLatchedFault() const noexcept override {
    return diag_lock_.Load().state ==
           static_cast<std::uint8_t>(compliance::ComplianceState::kSafeStop);
  }

  // ── Accessors (non-RT reads only) ─────────────────────────────────────────
  void set_gains(const Gains& g) noexcept {
    gains_lock_.Store(g);
    // The §7.6 MUST-1 ratio compares TWO gain sets; a new POD retires the
    // published figure. Re-armed rather than recomputed here because Λ_S(q) only
    // exists on the RT side (D20) — the next tick with a valid Cholesky does it.
    bandwidth_eval_pending_.store(true, std::memory_order_release);
  }

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

  /// Discard every queued off-RT target. Both held paths (E-STOP, unusable joint
  /// state) force a measured-pose re-seed on the next controllable tick, and
  /// neither bumps the activation generation — so a target pushed DURING the hold
  /// still passes IsCurrentGeneration and would overwrite that re-seed on the
  /// tick right after it. The RT thread is the sole SPSC consumer, so draining
  /// here is what keeps "a command issued while held does not survive recovery"
  /// true for both paths rather than only for E-STOP.
  ///
  /// Deliberately hides RTControllerInterface::DiscardPendingTargets(): same
  /// meaning, but this controller still owns its own pre-#206 queue, so the
  /// base's version would empty an always-empty one. The declaration goes away
  /// with the controller in #236 S7c, after which the base's is the only one.
  void DiscardPendingTargets() noexcept;

  /// §7.6 MUST-1, evaluated on the seeding tick from Λ_S(q₀) (RT, no logging).
  /// Writes bandwidth_ratio_ / bandwidth_ratio_low_.
  void EvaluateBandwidthSeparation(const Gains& gains) noexcept;

  /// Resolve `name` against an ARBITRARY handle/tip pair. Taking them as
  /// parameters is what lets InitFromModel validate the incoming model before it
  /// replaces the live one — the lookup is the only step there that can throw.
  /// @throws std::runtime_error  if `name` is not a frame of that model.
  [[nodiscard]] static pinocchio::FrameIndex LookupSensorFrame(
      const rtc_urdf_bridge::RtModelHandle& handle, pinocchio::FrameIndex tip,
      const std::string& name);
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
  /// nv measured joint velocity gathered into PINOCCHIO order. J_full_ is a
  /// Pinocchio-order matrix (ComputeJacobians reorders q on the way in), so the
  /// q̇ multiplied by it must be gathered too — feeding the device-order vector
  /// permutes ν silently, with every number finite.
  Eigen::VectorXd qdot_;
  Eigen::VectorXd q_null_;  ///< nv posture setpoint (DEVICE order, like q_dev_)
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
  /// Set off-RT by set_gains() / LoadConfig, consumed by the RT tick: the §7.6
  /// MUST-1 ratio must be recomputed for the gains actually in force, not only
  /// on the seeding tick. Cleared only once an evaluation succeeds, so a
  /// rank-deficient pose retries instead of publishing a retired figure.
  std::atomic<bool> bandwidth_eval_pending_{false};

  pinocchio::SE3 goal_pose_{pinocchio::SE3::Identity()};       // X_d, RT working copy
  pinocchio::SE3 compliant_pose_{pinocchio::SE3::Identity()};  // X_c, RT working copy
  double activation_elapsed_{0.0};  // RT-only: inner (α_track) ramp accumulator
  /// RT-only: outer (α_wrench) ramp accumulator. Separate from the above because
  /// it starts when the pipeline first emits a force, not when the controller
  /// activates — see "Activation ramp α" in the class note.
  double wrench_elapsed_{0.0};
  double saturation_elapsed_{0.0};                                   // RT-only: saturation-persist
  double bandwidth_ratio_{std::numeric_limits<double>::infinity()};  // RT-only (§7.6 MUST-1)
  bool bandwidth_ratio_low_{false};                                  // RT-only

  void ResetTargetInitialization() noexcept override {
    target_initialized_.store(false, std::memory_order_release);
    // An UNCONSUMED reset request must not cross an activation boundary (#260) —
    // see TaskImpedanceController for the full rationale (E-8 laundering).
    reset_fault_requested_.store(false, std::memory_order_release);
  }

  // Per-device limits (device channel order).
  std::vector<double> max_joint_torque_;
  std::vector<double> max_joint_velocity_;
  std::vector<double> position_lower_;
  std::vector<double> position_upper_;

  CommandType command_type_{CommandType::kTorque};
};

}  // namespace rtc
