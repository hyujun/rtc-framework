#ifndef RTC_CONTROLLERS_ESTIMATION_MOMENTUM_OBSERVER_HPP_
#define RTC_CONTROLLERS_ESTIMATION_MOMENTUM_OBSERVER_HPP_

#include "rtc_base/types/types.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace rtc::estimation {

/// Upper bound on observed DoF — mirrors kMaxDeviceChannels so every per-joint
/// array stays fixed-size (RT no-alloc). Actual nv is a runtime value given to
/// Init().
inline constexpr int kMaxObserverDof = kMaxDeviceChannels;

/// Why a tick did not advance the residual. `kNone` iff MomentumObserver::valid().
/// Reported in evaluation order (the first gate that failed). Values are
/// diagnostic constants — never renumber.
enum class MomentumInvalidReason : std::uint8_t {
  kNone = 0,             ///< Valid tick; the residual advanced.
  kNotInitialized = 1,   ///< Update()/Hold() before a successful Init().
  kHeld = 2,             ///< Caller's input gate was closed — Hold() was called.
  kShortInput = 3,       ///< Some input span was shorter than dof().
  kNonFiniteInput = 4,   ///< Some input element was NaN/Inf.
  kNonPositiveDt = 5,    ///< dt <= 0 — a non-advancing clock.
};

// ── Observer ─────────────────────────────────────────────────────────────────

/// Generalized-momentum observer — Layer 1 of #135. Produces the joint-space
/// residual `r`, a first-order-filtered estimate of the external generalized
/// torque:
///
///   beta_k = tau_m,k + Cᵀ_k q̇_k - g_k                              [MO-3a]
///   r_k    = K_I/(1 + dt·K_I) · (p_k - p_0 - intg_{k-1} - dt·beta_k) [MO-3b]
///   intg_k = intg_{k-1} + dt · (beta_k + r_k)                       [MO-3c]
///
/// which realizes `ṙ = K_I(tau_ext - r)` [MO-2], so `r → tau_ext` in steady
/// state and lags a time-varying tau_ext by ~1/K_I. The `+ r_k` in [MO-3c] is
/// what makes this a first-order filter rather than an open integrator; drop it
/// and the residual ramps without bound instead of settling (pinned by
/// test_momentum_observer).
///
/// THE CALLER ASSEMBLES THE DYNAMICS. Inputs are plain nv-vectors — momentum
/// `p = M(q) q̇`, the Coriolis product `Cᵀ q̇`, gravity `g(q)` and the measured
/// torque `tau_m`. No matrix ever enters this class. Two reasons, and the first
/// is not about cost:
///
///   1. COORDINATE ORDER. A Pinocchio handle's M / C / g getters are in the
///      model's internal joint order while DeviceState's arrays are in device
///      order, and mixing the two silently produces a finite, smooth, wrong
///      residual. Taking `p` instead of `M` means every input here is one
///      vector in one order and the output is in that same order, so this class
///      is order-agnostic by construction and the reorder contract lives in
///      exactly one place — the caller.
///   2. RT SHAPE. State is three nv-vectors and Update() is O(nv) element-wise:
///      no nv×nv argument, no matrix product, and no Eigen expression to alias.
///
/// `tau_m` UNIT AND COMPLETENESS CONTRACT. `tau_m` is JOINT TORQUE in N·m
/// (DeviceState::efforts, "torques for robot arm"), and it must account for
/// every generalized force acting on the joints other than the `M q̈ + C q̇ + g`
/// this observer models and the `tau_ext` it estimates. A backend that measures
/// motor current converts to joint torque at the backend boundary — the raw
/// current has its own lane (DeviceState::motor_efforts) and does not belong
/// here. Both halves are load-bearing and NEITHER is checkable from inside this
/// class: a lane that is fresh, full and in the right unit but missing a force
/// term reads as a genuine external torque. The one producer known to have
/// broken this was MuJoCo's position mode, which left its gravity-compensation
/// term out of `efforts` and so drove the residual to +g(q) under no load. It
/// was closed at the producer (#447 — rtc_mujoco_sim now sums qfrc_gravcomp
/// into the lane), not gated here: real hardware reports joint torque in every
/// command mode, so a command-mode gate in this class would reject valid
/// operation while still not detecting an incomplete lane.
///
/// tau_f (friction) is not modeled — [MO-3a] omits it, which per #135 lumps
/// friction into the residual along with model uncertainty. It enters as an
/// extra subtrahend in beta when it arrives.
///
/// RT contract: Init() is non-RT and may throw; Update() / Hold() are noexcept,
/// heap-free, fixed-size only.
class MomentumObserver {
 public:
  MomentumObserver() = default;

  /// Validate and latch the observer gains (non-RT). `gains` is K_I per joint;
  /// larger is faster to converge and noisier. Throws std::invalid_argument on:
  /// nv outside [1, kMaxObserverDof], `gains` shorter than nv, or any gain that
  /// is non-positive or non-finite. A zero gain is rejected rather than treated
  /// as "disable this joint" — it would pin that component of r at 0 forever,
  /// which is indistinguishable from a measured absence of load.
  void Init(int nv, std::span<const double> gains);

  /// Per-tick update on the RT thread. All spans must cover at least dof()
  /// entries and are read in the SAME joint order.
  ///
  ///   p     — generalized momentum M(q) q̇
  ///   c_t_v — Cᵀ(q, q̇) q̇   (NOT C q̇, and not h = C q̇ + g: Cᵀv != Cv)
  ///   g     — generalized gravity g(q)
  ///   tau_m — measured joint torque [N·m] (see the unit contract above)
  ///   dt    — tick period [s], > 0
  ///
  /// A rejected tick leaves the residual and the integrator exactly as they
  /// were and reports why via invalid_reason(); it never substitutes a zero or
  /// a clamped input, because both are assertions about the robot that the
  /// rejected data did not support.
  void Update(std::span<const double> p, std::span<const double> c_t_v, std::span<const double> g,
              std::span<const double> tau_m, double dt) noexcept;

  /// Declare this tick's inputs unusable without offering any (RT).
  ///
  /// This is the caller's lane gate reaching the observer. The gate itself
  /// cannot live here: it is rtc::IsLaneReadable(dev, StateLane::kVelocity |
  /// kEffort, dof), which lives in rtc_controller_interface, and rtc_controllers
  /// does not depend on that package — so the verdict arrives as this call.
  ///
  /// The residual and the integrator freeze and valid() goes false. The observer
  /// also arms a reseed: see ResetRtState() for why resuming is not the same as
  /// continuing.
  void Hold() noexcept;

  /// Drop every per-tick latch — momentum reference, integrator, residual — so
  /// a resumed loop cannot inherit state from before a gap. Init-time state
  /// (dof, gains) is kept. Call from on_activate. noexcept, heap-free.
  ///
  /// The next Update() re-captures `p_0` from that tick's momentum, which is
  /// also what happens automatically after any rejected or held tick. Continuing
  /// instead would be wrong in a specific way: `p` keeps moving while the
  /// integral does not, so on resume the unobserved momentum change appears in
  /// `p_k - p_0 - intg` and the observer reports it as an external torque that
  /// never existed. Re-seeding trades the accumulated estimate — r restarts at
  /// zero and re-converges over roughly 1/K_I — for never fabricating a load.
  /// Consumers that must not read that transient as "no load" gate on
  /// ticks_since_seed().
  void ResetRtState() noexcept;

  /// Residual r [N·m], dof() entries, in the caller's joint order. Meaningful
  /// only while valid(); frozen at its last value otherwise.
  [[nodiscard]] std::span<const double> residual() const noexcept;

  /// True iff the most recent Update() advanced the residual.
  [[nodiscard]] bool valid() const noexcept { return reason_ == MomentumInvalidReason::kNone; }

  [[nodiscard]] MomentumInvalidReason invalid_reason() const noexcept { return reason_; }

  /// Valid ticks accumulated since the momentum reference was last captured —
  /// the re-seeding tick itself counts as 1, and 0 means no valid tick has run
  /// since the last gap. While this is small the residual is still converging
  /// from zero, so a small |r| does not yet mean "no external torque"; a
  /// consumer that must not confuse the two waits for roughly 3/K_I worth of
  /// ticks. Saturates rather than wrapping.
  [[nodiscard]] std::uint32_t ticks_since_seed() const noexcept { return ticks_since_seed_; }

  [[nodiscard]] int dof() const noexcept { return dof_; }

  [[nodiscard]] bool initialized() const noexcept { return initialized_; }

 private:
  /// Freeze the residual and integrator, record why, and arm the reseed.
  void RejectTick(MomentumInvalidReason reason) noexcept;

  // ── Fixed at Init ──
  int dof_{0};
  bool initialized_{false};
  std::array<double, kMaxObserverDof> gains_{};

  // ── RT state ──
  std::array<double, kMaxObserverDof> p0_{};
  std::array<double, kMaxObserverDof> intg_{};
  std::array<double, kMaxObserverDof> residual_{};
  /// Set whenever the momentum reference no longer describes the current
  /// trajectory (Init, ResetRtState, or any rejected/held tick). Consumed by
  /// the next accepted Update().
  bool seed_pending_{true};
  std::uint32_t ticks_since_seed_{0};
  MomentumInvalidReason reason_{MomentumInvalidReason::kNotInitialized};
};

}  // namespace rtc::estimation

#endif  // RTC_CONTROLLERS_ESTIMATION_MOMENTUM_OBSERVER_HPP_
