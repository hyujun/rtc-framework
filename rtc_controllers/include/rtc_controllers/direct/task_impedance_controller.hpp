// ── Includes: project header first, then third-party, then C++ stdlib ───────
#pragma once

#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/compliance/compliance_state_machine.hpp"
#include "rtc_controllers/compliance/external_wrench.hpp"
#include "rtc_controllers/compliance/impedance_law.hpp"
#include "rtc_controllers/compliance/inertia_shaping.hpp"
#include "rtc_controllers/compliance/safety_limiter.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/compliance/torque_estop.hpp"
#include "rtc_controllers/compliance/wrench_conditioning.hpp"
#include "rtc_controllers/compliance/wrench_pipeline.hpp"
#include "rtc_controllers/params/task_impedance_params.hpp"
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
/// Jacobian-transpose stiffness law (spec §6.2), optionally inertia-shaped with
/// a measured external wrench (§6.3).
///
/// ### Control law (`kJacobianTranspose`, A=NONE — the default)
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
/// ### Control law (`kInertiaShaping`, A≠NONE — spec §6.3)
/// @code
///   B  = Λ_S Λ_d⁻¹                                 [m×m, clamped by max_inertia_ratio]
///   τ  = Jᵀ Sᵀ[B(K_p·S·e + K_d·S·ė) + (B − I)·S·f_ext] + τ_null + ĝ(q)
/// @endcode
/// which enforces the closed loop `Λ_d ë + K_d ė + K_p e = −f_ext`, the same
/// relation §6.2 gives with `Λ_d = Λ_S` — as it must, since `B = I` collapses the
/// two laws. The wrench term carries `+(B − I)` and not the spec's `(B − I)(−·)`
/// because `f_ext` here is the ENVIRONMENT-ON-ROBOT wrench of this package's input
/// contract, the opposite sign to the one that expression is written in; the
/// bridge is derived where the law lives, compliance/inertia_shaping.hpp. With
/// `Λ_d = Λ_S` (the `desired_inertia: natural` configuration) `B = I` and the
/// second term vanishes, so the law collapses **exactly** onto §6.2 — that
/// identity is the A=NONE ↔ A≠NONE boundary check (§11.4 T4.1) and is asserted
/// to 1e-9 by the test suite.
///
/// Inertia shaping is **off by default** (§5.2 MUST): `Λ_d ≠ Λ_S` amplifies both
/// wrench-measurement noise and model error, the more so the lighter `Λ_d` gets.
///
/// ### External wrench (§3.2.1)
/// The controller owns **no node, no subscription and no message type** — it is a
/// pure control algorithm. `SetExternalWrench()` is a non-RT setter (the
/// `SetDeviceTarget` idiom) taking the RAW wrench in the configured
/// `sensor_frame`; conditioning (bias → static gravity → deadband → saturation →
/// filter) and the `Ad^{-T}` transform to LOCAL_WORLD_ALIGNED are done here,
/// because both need the robot model the caller would otherwise have to carry.
/// Sign convention: **positive = the wrench the environment applies ON the robot**.
///
/// `TRANSLATION_ONLY` regulates position only; orientation is left to the
/// nullspace posture task, so `nullspace_stiffness == 0` with `TRANSLATION_ONLY`
/// is a **configure error** (§6.1 — rotation would drift uncontrolled).
class TaskImpedanceController final : public RTControllerInterface {
 public:
  /// The gains POD and the two axis enums live beside the laws rather than
  /// inside this adapter — see params/task_impedance_params.hpp for why (#236
  /// S7c-2, D-B/G2). These aliases keep every existing spelling
  /// (`TaskImpedanceController::Gains`, `::TaskSelection`, `::Formulation`)
  /// resolving to the lifted definitions.
  using TaskSelection = params::TaskSelection;
  using Formulation = params::TaskImpedanceFormulation;
  using Gains = params::TaskImpedanceParams;

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

  /// Publish a measured external wrench (§3.2.1). NON-RT entry point, same
  /// contract as SetDeviceTarget: call it off the RT thread, from a SINGLE
  /// producer (the SeqLock underneath is single-writer). noexcept, wait-free.
  ///
  /// @param wrench  RAW `[f;τ]` in the configured `sensor_frame`'s BODY frame,
  ///                SI units (N, N·m), sign = wrench applied BY the environment
  ///                ON the robot. No bias removal, gravity compensation, frame
  ///                transform or filtering by the caller — all of it happens
  ///                here, since all of it needs the robot model.
  ///
  /// Dropped silently when no wrench source is configured
  /// (`external_wrench.enabled: false`, the default), so a stray producer cannot
  /// perturb a controller that is running the A=NONE law.
  void SetExternalWrench(std::span<const double, compliance::kWrenchDim> wrench) noexcept;

  void TriggerEstop() noexcept override;
  void ClearEstop() noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  void LoadConfig(const YAML::Node& cfg) override;
  void OnDeviceConfigsSet() override;

  [[nodiscard]] CommandType GetCommandType() const noexcept override { return command_type_; }

  // Clear a LATCHED controller-local SAFE_STOP fault (the /rtc_cm/reset_fault
  // service). Deliberately SEPARATE from ClearEstop (E-8): a global E-STOP
  // clear must NOT release a controller fault, and vice versa. The flag is
  // consumed at the head of the next Compute(), ahead of the E-STOP early
  // return, so a controller fault can be cleared while the global latch is up.
  void ResetFault() noexcept override {
    reset_fault_requested_.store(true, std::memory_order_release);
  }

  // SAFE_STOP is the only latch this controller keeps (§10.6). Read off-RT from
  // the per-tick diagnostic snapshot rather than from sm_, which the RT thread
  // owns; every Compute() exit path stores it, including the E-STOP and
  // no-joint-state paths, so the answer is never a tick stale for a running
  // controller. A controller that has not ticked yet reads as not latched.
  [[nodiscard]] bool HasLatchedFault() const noexcept override {
    return diag_lock_.Load().state ==
           static_cast<std::uint8_t>(compliance::ComplianceState::kSafeStop);
  }

  // ── Accessors (non-RT reads only) ─────────────────────────────────────────
  void set_gains(const Gains& g) noexcept { gains_lock_.Store(g); }

  [[nodiscard]] Gains get_gains() const noexcept { return gains_lock_.Load(); }

  [[nodiscard]] TaskSelection selection() const noexcept { return selection_; }

  [[nodiscard]] int task_dim() const noexcept { return m_; }

  [[nodiscard]] Formulation formulation() const noexcept { return formulation_; }

  [[nodiscard]] bool external_wrench_enabled() const noexcept { return wrench_enabled_; }

  // Diagnostic snapshot published every tick (incl. E-STOP / early return) with
  // per-field validity, so a stale body never rides out under a fresh stamp.
  struct Diagnostics {
    std::uint8_t state{0};  ///< ComplianceState
    double sigma_min{0.0};
    double lambda_sq{0.0};
    std::array<double, 6> pose_error{};
    /// Conditioned external wrench in LOCAL_WORLD_ALIGNED at the tip, after the
    /// staleness fade but BEFORE the activation ramp α — i.e. the physical
    /// estimate of the environment load, not the ramped control-law input.
    std::array<double, 6> wrench_lwa{};
    double wrench_age{0.0};   ///< s since the last NEW sample (§10.6, D9)
    double wrench_fade{1.0};  ///< 1 fresh → 0 fully faded out
    /// Samples the input's finiteness gate dropped since construction. A count
    /// rising while `wrench_valid` stays true = an intermittently garbage
    /// producer, which is otherwise invisible: a dropped sample simply does not
    /// arrive, and the last good one keeps ageing normally.
    std::uint32_t wrench_rejected{0};
    bool saturated{false};
    bool rate_limited{false};
    bool nullspace_active{false};
    bool wrench_valid{false};  ///< a sample has arrived and a source is configured
    bool wrench_stale{false};  ///< age > wrench_timeout (→ DEGRADED, never SAFE_STOP)
    /// The bias in use is a committed §3.2.1 calibration — or none is needed
    /// (no wrench source). FALSE means the wrench is running on a zero bias, or
    /// is being suppressed while the average accumulates; without this field a
    /// calibration that never happened is indistinguishable from one that did.
    bool bias_calibrated{true};
    bool in_contact{false};       ///< contact hysteresis latch (RUNNING_CONTACT)
    bool inertia_clamped{false};  ///< §5.2 max_inertia_ratio clamp engaged this tick
    /// Λ_d could not be factored, so this tick silently ran §6.2 (B = I) instead
    /// of §6.3. Kept SEPARATE from inertia_clamped: one is a tuning bound doing
    /// its job, the other is a numerical breakdown, and an operator that cannot
    /// tell them apart has no way to act on either. (Sharing one flag also let a
    /// clamp test pass on a factorisation failure.)
    bool inertia_solve_failed{false};
    bool estopped{false};
    bool control_valid{false};  ///< false on E-STOP / degenerate-dynamics ticks
  };

  static_assert(std::is_trivially_copyable_v<Diagnostics>, "Diagnostics must be SeqLock-safe");

  [[nodiscard]] Diagnostics GetDiagnosticsForTesting() const noexcept { return diag_lock_.Load(); }

  // Bias estimate committed by BIAS_CALIBRATING. Test-only: the RT thread owns
  // the pipeline, so this is safe to read only between Compute() calls.
  [[nodiscard]] const compliance::Wrench6& GetWrenchBiasForTesting() const noexcept {
    return wrench_.bias();
  }

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
                                              const Diagnostics& diag, const Gains& gains) noexcept;

  /// The primary device's joint state is unusable this tick. Emits a zero-length
  /// command for device 0 (the CM's own "no update" idiom), keeps secondary
  /// passthrough, steps the FSM into DEGRADED and forces a re-seed. Without this
  /// gate the unread channels read as 0 and the whole law runs at the ZERO
  /// configuration — a full-arm move to the origin with every number finite and
  /// no fault raised. Mirrors CascadedComplianceController's gate of the same
  /// name; see that class for why the torque domain keeps riding through rather
  /// than escalating on a timer (issue #236 E-8).
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

  // Frame name → index against the CURRENT model; empty name ⇒ the tip frame.
  // Off-RT. Throws when a configured frame name is absent (fail-fast at
  // configure) — LoadConfig calls it into a local so a bad name cannot leave the
  // controller half-reconfigured.
  [[nodiscard]] pinocchio::FrameIndex LookupSensorFrame(const std::string& name) const;

  // Re-resolve sensor_frame_id_ in place. Called after every InitFromModel and
  // from OnDeviceConfigsSet, since both can move the frame the index refers to.
  void ResolveSensorFrame();

  // RT: read → condition → Ad^{-T} to LWA → fade. Writes the wrench diagnostics
  // fields and returns the wrench that enters the control law (zero when no
  // source is configured, so the §6.2 path is untouched).
  Eigen::Matrix<double, 6, 1> UpdateExternalWrench(const pinocchio::SE3& tcp, double dt,
                                                   const Gains& gains, Diagnostics& diag) noexcept;

  // ── Model ──────────────────────────────────────────────────────────────────
  std::shared_ptr<const pinocchio::Model> model_ptr_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_frame_id_{0};

  const TaskSelection selection_;
  int m_{6};  ///< task dimension (6 = FULL_SE3, 3 = TRANSLATION_ONLY)

  // Axis A / wrench wiring. Written off-RT (LoadConfig, before activation) and
  // read on the RT tick — the same lifetime contract command_type_ already has,
  // so no atomic is needed: the CM configures, then activates.
  Formulation formulation_{Formulation::kJacobianTranspose};
  bool wrench_enabled_{false};
  std::string sensor_frame_name_;  ///< empty ⇒ the tip frame
  pinocchio::FrameIndex sensor_frame_id_{0};
  Eigen::Vector3d gravity_world_{Eigen::Vector3d::Zero()};  ///< from the model, not 9.81

  compliance::TaskDynamics dyn_;
  compliance::ComplianceStateMachine sm_;
  // Read → bias two-latch → condition → Ad^{-T} → fade → contact, in that order
  // (§3.2.1, §3.3, §10.6). Shared with TaskAdmittanceController; the two-latch
  // bias design and the ORDER are what live inside it.
  compliance::WrenchPipeline wrench_;
  // RT-only mirror of the pipeline's FSM gate ("stop holding BIAS_CALIBRATING").
  // Deliberately NOT the same thing as "a calibration is still owed" — that debt
  // is the pipeline's `bias_pending_`, and collapsing the two is what would let
  // a cold-start ordering (controller activates before the F/T driver publishes)
  // skip §3.2.1's mandated calibration for a whole activation.
  bool bias_gate_{true};

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
  /// nv measured joint velocity gathered into PINOCCHIO order. J_full_ is a
  /// Pinocchio-order matrix (ComputeJacobians reorders q on the way in), so the
  /// q̇ multiplied by it must be gathered too — feeding the device-order vector
  /// permutes the task twist silently, with every number finite.
  Eigen::VectorXd qdot_;
  Eigen::VectorXd q_null_;  ///< nv posture setpoint (DEVICE order), seeded on activate
  Eigen::LLT<Eigen::MatrixXd> llt_M_;

  // No §6.3 scratch members: compliance::ComputeShapedTaskForce owns its own,
  // and it is heap-free because the storage is compile-time bounded at 6×6
  // (D-S4a). A member buffer here would only re-create the coupling the
  // extraction removed.

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
    // An UNCONSUMED reset request must not cross an activation boundary (#260).
    // Compute() runs only while active, so a request that arrived while this
    // controller was inactive would be consumed on the first tick after
    // activation and unlatch a SAFE_STOP nobody re-authorised at that moment —
    // the exact laundering BeginBiasCalibration() refuses (E-8).
    // /rtc_cm/reset_fault also refuses inactive targets; this closes the same
    // door on the controller side, where the latch actually lives.
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
