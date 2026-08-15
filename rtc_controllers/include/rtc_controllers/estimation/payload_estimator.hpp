#ifndef RTC_CONTROLLERS_ESTIMATION_PAYLOAD_ESTIMATOR_HPP_
#define RTC_CONTROLLERS_ESTIMATION_PAYLOAD_ESTIMATOR_HPP_

#include "rtc_controllers/compliance/differential_ik.hpp"
#include "rtc_controllers/estimation/momentum_observer.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <span>

namespace rtc::estimation {

/// Why a tick produced no payload estimate. `kNone` iff PayloadEstimator::valid().
/// Reported in evaluation order (the first gate that failed). Values are
/// diagnostic constants — never renumber.
///
/// The split between the reasons this class decides and the ones it is TOLD is
/// the same split MomentumObserver draws, and for the same reason: a gate whose
/// evidence does not reach this header cannot live in it. kArmMoving /
/// kHandMoving / kNotConverged arrive through Hold() because joint velocities
/// and the observer's seed age belong to the caller's device view.
enum class PayloadInvalidReason : std::uint8_t {
  kNone = 0,              ///< Valid tick; the estimate advanced.
  kNotInitialized = 1,    ///< Update()/Hold() before a successful Init().
  kHeld = 2,              ///< Caller's gate was closed with no more specific reason.
  kShortInput = 3,        ///< `r` was shorter than nv(), or J had the wrong shape.
  kNonFiniteInput = 4,    ///< Some element of J, r or gravity was NaN/Inf.
  kObserverInvalid = 5,   ///< Upstream residual was not valid this tick.
  kNotConverged = 6,      ///< Residual has not settled since its last re-seed.
  kArmMoving = 7,         ///< Quasi-static assumption violated by the arm.
  kHandMoving = 8,        ///< Quasi-static assumption violated by the hand (D14).
  kDegenerateGravity = 9, ///< ‖ᵂg‖ below the floor — the mass map is undefined.
  kSolverFailed = 10,     ///< The damped normal equations would not factor.
  kRankDeficient = 11,    ///< σ_min(J) below the floor — the wrench is unobservable.
  kPoorFit = 12,          ///< ‖Jᵀŵ − r‖∞ too large — r is not a payload wrench.
};

/// One tick's payload estimate plus the diagnostics that justify trusting it.
struct PayloadEstimate {
  /// ŵ_ext [N, N·m] at the payload frame, LOCAL_WORLD_ALIGNED — force first.
  /// This is the wrench the payload applies TO the robot (see the sign
  /// contract in the class preamble), so a hanging mass gives a DOWNWARD force.
  std::array<double, 6> wrench{};
  double mass{0.0};        ///< m̂ [kg]. Positive for a real payload.
  double sigma_min{0.0};   ///< σ_min(J) — how observable the wrench was.
  double lambda_sq{0.0};   ///< §6.5 damping actually applied (0 = far from singular).
  double fit_error{0.0};   ///< ‖Jᵀŵ − r‖∞ [N·m] — how well the wrench explains r.
};

// ── Estimator ────────────────────────────────────────────────────────────────

/// Quasi-static payload estimator — Layer 2A of #135. Inverts the joint-space
/// residual `r` produced by MomentumObserver into a Cartesian wrench and a
/// payload mass:
///
///   ŵ_ext = argmin_w ‖J_p(q)ᵀ w − r‖² + μ‖w‖²                    [WRENCH-A]
///   m̂     = (f̂_ext · ᵂg) / ‖ᵂg‖²                                 [MASS-A]
///
/// THE SIGN CONTRACT IS INHERITED, NOT CHOSEN. MomentumObserver's [MO-3a]
/// (`β = τ_m + Cᵀq̇ − g`) presupposes the equation of motion
/// `M q̈ + C q̇ + g = τ_m + τ_ext`, so its residual converges to the generalized
/// torque the ENVIRONMENT APPLIES TO THE ROBOT — the same side of the equation
/// as the actuator torque. Two independent confirmations: test_momentum_observer
/// derives its measured torque as `τ_m = g − τ_ext`, and the #447 anecdote in
/// that class's preamble (a missing gravity-compensation term driving `r → +g`)
/// reproduces exactly under this convention and under no other.
///
/// Consequently `τ_ext = J_pᵀ w` with `w` also applied TO the robot, and a
/// payload of mass m contributes `f = m·ᵂg` — a force pointing DOWN, not up.
/// [MASS-A] is written against ᵂg rather than an "up axis" for that reason: the
/// dot product recovers a POSITIVE mass precisely because both vectors point
/// the same way, and no separate axis convention has to be kept in sync. Sign
/// and magnitude are pinned by PayloadEstimatorTest.RecoversKnownHangingMass.
///
/// ᵂg IS THE MODEL'S GRAVITY, NOT 9.81·−Z. It is passed in because this class is
/// robot-agnostic (ARCH-1) and the URDF decides both the axis and the magnitude;
/// `rtc::compliance::StaticGravityWrench` takes it the same way for the same
/// reason. Dividing by ‖ᵂg‖² rather than by a scalar `g` means a model with
/// gravity along Y, or a lunar one, needs no code change.
///
/// COORDINATE ORDER. `J` and `r` must be in the SAME joint order — J's COLUMNS
/// and r's ENTRIES index the same velocity space. This class never learns which
/// order that is, exactly as MomentumObserver never does, so it is
/// order-agnostic by construction and the reorder contract stays in the caller.
/// The trap this avoids is real and asymmetric: RtModelHandle::GetFrameJacobian
/// hands back PINOCCHIO-order columns while MomentumObserverWiring::residual()
/// hands back DEVICE order, and pairing those two produces a finite, smooth,
/// wrong wrench. The caller must therefore project against the residual BEFORE
/// it is reordered. Nothing downstream can detect the mix — which is why the
/// wiring, not this class, owns that decision.
///
/// THE SOLVER IS REUSED, NOT REWRITTEN (P5). The normal equations of [WRENCH-A]
/// are `(J Jᵀ + μI) w = J r`, so `ŵ = (J⁺)ᵀ r` with `J⁺ = Jᵀ(J Jᵀ + μI)⁻¹` —
/// which is exactly what compliance::DifferentialIk builds, down to the LLT and
/// the §6.5 σ_min-adaptive damping law. Reusing it means NUM-1 is satisfied by
/// the same rule the controllers already obey rather than a second damping
/// policy, and the NaN-pivot trap (Eigen's LLT reports Success on a matrix full
/// of NaN) is already closed there by an explicit allFinite() check.
///
/// WHY THE DAMPING STAYS ADAPTIVE. λ² is 0 away from singularities, so a
/// well-conditioned pose gets the undamped least-squares estimate and m̂ carries
/// no shrinkage bias. A constant μ floor would trade that bias for lower
/// variance; it is deliberately NOT applied, because a floor biases m̂ toward
/// zero at EVERY pose — always under-reporting the payload — and no measurement
/// yet justifies paying that everywhere to damp noise that the validity gates
/// already exclude. Callers log `sigma_min` / `lambda_sq` so the choice stays
/// observable.
///
/// The DoF bound is MomentumObserver's kMaxObserverDof rather than a second
/// constant of this class's own: `r` comes from that observer, so a bound this
/// class could exceed would be one the residual can never reach.
///
/// RT contract: Init() is non-RT and may throw; Update() / Hold() are noexcept,
/// heap-free, fixed-size only.
class PayloadEstimator {
 public:
  /// Thresholds. All are configuration, never robot constants (ARCH-1): the
  /// values that suit a 7-DoF arm at 500 Hz are not the ones a different robot
  /// or a different sensor noise floor wants.
  struct Config {
    /// §6.5 adaptive-damping knee σ₀ [same units as σ(J)]. Damping switches on
    /// below this. Must be > 0 — at σ₀ ≤ 0 AdaptiveDampingSquared short-circuits
    /// to λ² = 0 everywhere, which is not a weaker guard but no guard.
    double sigma0{0.0};
    /// §6.5 damping ceiling λ_max. Floored by compliance::FloorMaxDamping at the
    /// point of use, per the NUM-1 rule that a bound named by an invariant is
    /// applied next to the law it protects.
    double lambda_max{0.0};
    /// σ_min(J) below this rejects the tick as kRankDeficient. Distinct from
    /// σ₀: σ₀ says "start damping", this says "stop believing the answer".
    double min_sigma{0.0};
    /// ‖Jᵀŵ − r‖∞ above this rejects the tick as kPoorFit [N·m]. This is the
    /// only gate that can tell a payload apart from an unmodelled joint-level
    /// torque, because a wrench that no Cartesian force could have produced
    /// leaves a residual the projection cannot absorb.
    double max_fit_error{0.0};
    /// ‖ᵂg‖ below this rejects the tick as kDegenerateGravity [m/s²] (NUM-2 /
    /// NUM-4): [MASS-A] divides by ‖ᵂg‖², so a gravity-free model has no mass
    /// map at all — the honest answer is "invalid", not a huge number.
    double min_gravity{0.0};
  };

  PayloadEstimator() = default;

  /// Validate the thresholds and size every buffer (non-RT).
  /// @throws std::invalid_argument on nv outside [1, kMaxObserverDof], or any
  ///         threshold that is non-positive or non-finite. Non-positive is
  ///         rejected rather than read as "disable this gate": a disabled gate
  ///         and a passing one are indistinguishable downstream, and every one
  ///         of these guards exists because its absence is invisible.
  void Init(int nv, const Config& config);

  /// Per-tick estimate on the RT thread (noexcept, heap-free).
  ///
  ///   J        — payload-frame Jacobian, 6×nv, LOCAL_WORLD_ALIGNED
  ///   r        — joint-space residual [N·m], nv entries, SAME order as J's columns
  ///   gravity  — ᵂg [m/s²] from the model, in the frame J's rows are aligned to
  ///
  /// A rejected tick leaves the previous estimate untouched and reports why via
  /// invalid_reason(); it never substitutes a zero mass, because "no payload"
  /// and "could not tell" are different claims about the robot.
  void Update(const Eigen::Ref<const Eigen::MatrixXd>& J, std::span<const double> r,
              const Eigen::Vector3d& gravity) noexcept;

  /// Declare this tick unusable without offering inputs (RT). The caller's
  /// kinematic gates — arm quasi-static, hand static, residual converged —
  /// reach the estimator through this call, carrying which one closed.
  void Hold(PayloadInvalidReason reason) noexcept;

  /// Drop the latched estimate so a resumed loop cannot report a payload
  /// measured before a gap. Init-time state (nv, thresholds) is kept.
  /// Call from on_activate. noexcept, heap-free.
  void ResetRtState() noexcept;

  /// Last accepted estimate. Meaningful only while valid(); frozen otherwise.
  [[nodiscard]] const PayloadEstimate& estimate() const noexcept { return estimate_; }

  [[nodiscard]] bool valid() const noexcept { return reason_ == PayloadInvalidReason::kNone; }

  [[nodiscard]] PayloadInvalidReason invalid_reason() const noexcept { return reason_; }

  [[nodiscard]] int nv() const noexcept { return nv_; }

 private:
  int nv_{0};
  Config config_{};
  bool initialized_{false};
  PayloadInvalidReason reason_{PayloadInvalidReason::kNotInitialized};
  PayloadEstimate estimate_{};

  rtc::compliance::DifferentialIk dik_;
  Eigen::Matrix<double, 6, 1> wrench_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::VectorXd r_buf_;    ///< r gathered into an Eigen vector (nv)
  Eigen::VectorXd fit_buf_;  ///< Jᵀŵ (nv), for the reconstruction gate
};

}  // namespace rtc::estimation

#endif  // RTC_CONTROLLERS_ESTIMATION_PAYLOAD_ESTIMATOR_HPP_
