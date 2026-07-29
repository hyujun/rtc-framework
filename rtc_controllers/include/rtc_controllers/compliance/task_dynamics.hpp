// ── Task-space dynamics helper for compliance controllers ───────────────────
// Shared numerical core extracted for reuse (design-principles P5): the task
// inertia Λ_S, the dynamically-consistent nullspace projector Nᵀ, and the
// σ_min-adaptive damped-least-squares (DLS) singularity handling. Pure Eigen,
// RT-safe (fixed after Resize(): noexcept, no heap, no throw).
//
// This WAS not the OSC controller's inline block: OSC used a *constant* damping
// λ = max(1e-4, gains.damping), where the compliance spec (§6.5) adapts λ² to
// σ_min(J_S) so a well-conditioned pose adds zero damping (exact nullspace
// orthogonality) and damping only grows inside the singular shell. #236 S2b
// converged OSC onto this class; the constant-λ spelling is gone. That migration
// is deliberately NOT bit-identical — the λ change is a law change (tier 1) and
// materialising Λ_S rather than solving in place is rounding (tier 2, max
// relative difference ≤ 1.7e-12). It was acceptable because the OSC adapter is
// not wired into any bringup: runtime exposure was zero (#236 D-S2b).
//
// Ordering / symbols follow spec §6.4–§6.5:
//   J_S = S·J        selected task Jacobian            (m×nv, m = task DoF)
//   Λ_S = (J_S M⁻¹ J_Sᵀ + λ²I)⁻¹                       (m×m, DLS task inertia)
//   J̄_S = M⁻¹ J_Sᵀ Λ_S   dynamically-consistent inverse (nv×m)
//   Nᵀ  = I − J_Sᵀ J̄_Sᵀ  torque nullspace proj(transpose form) (nv×nv)
//   τ_null = Nᵀ · τ_posture
#pragma once

#include <Eigen/Cholesky>  // LLT
#include <Eigen/Core>
#include <Eigen/Eigenvalues>  // SelfAdjointEigenSolver (σ_min)

#include <algorithm>
#include <cmath>

namespace rtc::compliance {

// ── The two bounds that keep §6.5 armed ─────────────────────────────────────
// They live next to the law they protect, not in five anonymous namespaces. All
// five task-space schemas apply the same two, and invariants.md NUM-1 names them
// as ONE thing across all five — so five private copies means five edits to
// change one documented invariant, and any single miss leaves a consumer whose
// singularity guard silently differs from the doc that describes it.
//
// Where each is applied, post-#282:
//   λ_max (NUM-1) — in the parsers, and again at the point of USE, because a
//     gains POD written straight into a SeqLock bypasses configure entirely.
//     That second half lived in the adapters' Compute() and was deleted with
//     them (#298 S7c-2); demo_task_controller is the first shipped binding to
//     exercise it again.
//   σ₀ (NUM-2) — the SAME two places, and for the same reason. It used to be
//     parser-only on the argument that it merely parameterises where the ramp
//     starts; that argument does not survive the surface `set_gains()` opens.
//     σ₀ ≤ 0 does not start the ramp later, it disarms §6.5 EVERYWHERE —
//     AdaptiveDampingSquared short-circuits to λ² = 0 for every σ_min — so the
//     missing half is not a weaker guard, it is no guard.
inline constexpr double kMinMaxDamping = 1e-4;

/// The NUM-1 λ_max floor, in ONE place — the §6.5 counterpart of NUM-6's
/// `rtc::FloorNonNegativeGain`, and deliberately the same shape, because it is the
/// same problem: a bound named by an invariant must be a symbol a future binding
/// can GREP, not a `std::max` open-coded at each site plus a row in a doc table.
/// Every consumer applies it twice (the parser unconditionally, and again at the
/// point of use before the value reaches the law, because `set_gains()` writes
/// the Gains POD straight into the SeqLock and bypasses configure).
///
/// Why not `std::max(kMinMaxDamping, x)` written out at each site: `std::max` is
/// `a < b ? b : a`, and `1e-4 < NaN` is FALSE, so it returns **1e-4** for a NaN
/// λ_max. That is not a floor, it is laundering — it turns a corrupt gain into a
/// plausible one and hands the caller a damping shell that looks armed. A
/// non-finite λ_max is instead passed through UNCHANGED, so λ² is non-finite and
/// a finite-output check downstream sees it. WHICH check depends on the lane, and
/// both exist: on the torque lane `compliance::AllFinite` (§10.5, evaluated
/// before saturation can mask an ∞), and on the position lane
/// `urtc::ValidateControllerOutput`, which screens every command at the actuator
/// boundary and makes the manager replace the tick with a hold and escalate to
/// E-STOP after ~100 ms of consecutive rejects. (`cascaded_compliance_params`
/// rejects non-finite before it gets here, so there this branch is unreachable
/// defence-in-depth; the other four parsers reach it.)
///
/// The law itself still floors nothing — `AdaptiveDampingSquared` takes λ_max as
/// a plain argument and must keep satisfying its own named property, λ² ≤ λ_max²
/// (pinned by AdaptiveDampingLaw.NeverExceedsLambdaMaxSquared). An internal
/// floor would make that property FALSE below 1e-4 while every existing oracle,
/// which drives λ_max = 0.05 throughout, stayed green. This is a policy the
/// CONSUMER applies, next to the law it protects.
[[nodiscard]] inline double FloorMaxDamping(double lambda_max) noexcept {
  return std::isfinite(lambda_max) ? std::max(kMinMaxDamping, lambda_max) : lambda_max;
}

// σ₀ ≤ 0 does not narrow the shell — AdaptiveDampingSquared short-circuits and
// returns λ² = 0 for EVERY σ_min, i.e. no damping anywhere.
inline constexpr double kMinSigma0 = 1e-6;

/// The NUM-2 σ₀ floor, in ONE place — FloorMaxDamping's partner, deliberately
/// the same shape because it is the same problem one gain over.
///
/// Why not `std::max(kMinSigma0, x)` at each site: `1e-6 < NaN` is FALSE, so a
/// NaN σ₀ comes back as **1e-6** — and for σ₀ that laundering is worse than for
/// λ_max, because the result still looks armed at every callsite. A shell of
/// radius 1e-6 is one no reachable pose enters, so §6.5 never engages, no fault
/// fires and nothing is logged: the operator's corrupt gain has been traded for
/// a silently disarmed singularity guard. Passed through UNCHANGED instead, a
/// non-finite σ₀ propagates: `sigma0 <= 0.0` and `sigma_min >= sigma0` are BOTH
/// false against NaN, so AdaptiveDampingSquared reaches the ratio and returns a
/// non-finite λ², which the same downstream finite-command check catches.
///
/// Applied at BOTH surfaces like λ_max, and for the same reason — `set_gains()`
/// writes the Gains POD straight into the SeqLock and passes through neither
/// parser nor parameter callback.
[[nodiscard]] inline double FloorSigma0(double sigma0) noexcept {
  return std::isfinite(sigma0) ? std::max(kMinSigma0, sigma0) : sigma0;
}

// σ_min-adaptive DLS damping λ² (spec §6.5):
//   λ² = 0                              if σ_min ≥ σ₀        (no damping)
//   λ² = λ_max²·(1 − (σ_min/σ₀)²)       if σ_min < σ₀        (grows into shell)
// Returning exactly 0 outside the shell is what preserves the exact nullspace
// orthogonality J_S M⁻¹ Nᵀ = 0 (T4.5) away from singularities; inside the shell
// the residual is bounded by λ² by construction.
[[nodiscard]] inline double AdaptiveDampingSquared(double sigma_min, double sigma0,
                                                   double lambda_max) noexcept {
  if (sigma0 <= 0.0 || sigma_min >= sigma0)
    return 0.0;
  const double ratio = sigma_min / sigma0;  // sigma0 > 0 guaranteed here (NUM-2 guard)
  return lambda_max * lambda_max * (1.0 - ratio * ratio);
}

// σ_min(J) = sqrt(λ_min(J Jᵀ)) — the kinematic distance to a singularity, and
// the input AdaptiveDampingSquared is parameterised on. The clamp at 0 is not
// cosmetic: J Jᵀ is SPSD, so round-off routinely puts λ_min a few ulp BELOW
// zero at a genuinely singular pose and the sqrt would return NaN — which then
// compares false against every fault threshold and silently disarms §6.5.
//
// Free function (not a TaskDynamics method) so the kinematic differential-IK
// path can share the exact definition instead of re-deriving it; the caller
// supplies the two work buffers, keeping this allocation-free on the RT tick.
[[nodiscard]] inline double SmallestSingularValue(
    const Eigen::Ref<const Eigen::MatrixXd>& J, Eigen::MatrixXd& jjt_buf,
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>& saes) noexcept {
  jjt_buf.noalias() = J * J.transpose();
  saes.compute(jjt_buf, Eigen::EigenvaluesOnly);
  return std::sqrt(std::max(0.0, saes.eigenvalues()(0)));  // ascending order
}

// Holds pre-sized work buffers so Compute() is heap-free on the RT path. Resize()
// is off-RT (lifecycle) and allocates; Compute() must be called only after it.
class TaskDynamics {
 public:
  struct Result {
    double sigma_min{0.0};  ///< σ_min(J_S) — kinematic distance to singularity
    double lambda_sq{0.0};  ///< DLS damping λ² applied this call (0 = far from singular)
    bool ok{false};         ///< false if the SPD factorisation failed (NaN-safe abort)
  };

  // Size all buffers for an nv-DoF model and an m-dimensional task. Off-RT only.
  void Resize(int nv, int m) {
    nv_ = nv;
    m_ = m;
    J_S_.resize(m, nv);
    MinvJsT_.resize(nv, m);
    A_.resize(m, m);    // J_S M⁻¹ J_Sᵀ (+ λ²I)
    JJt_.resize(m, m);  // J_S J_Sᵀ (for σ_min)
    LambdaS_.resize(m, m);
    JbarS_.resize(nv, m);  // J̄_S = M⁻¹ J_Sᵀ Λ_S
    Nt_.resize(nv, nv);    // Nᵀ
    llt_A_ = Eigen::LLT<Eigen::MatrixXd>(m);
    saes_ = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(m);
  }

  // Compute Λ_S, J̄_S and Nᵀ from the selected Jacobian J_S (m×nv) and a
  // pre-factored joint inertia M (its LLT). `llt_M` must already hold a
  // successful factorisation of the SPD matrix M(q) (nv×nv). sigma0/lambda_max
  // parameterise the §6.5 DLS. RT-safe: noexcept, no allocation.
  //
  // A false `ok` means Λ_S / J̄_S / Nᵀ are STALE and the caller must fall back
  // (every caller holds a safe torque, τ = ĝ + C·v). Note what does NOT trigger
  // it: J_S M⁻¹ J_Sᵀ is positive SEMI-definite, and Eigen's LLT accepts a
  // singular one, so a rank-deficient pose reports ok WITH σ_min ≈ 0 — the σ_min
  // faults are what catch that. The reachable failure is a non-finite J_S.
  [[nodiscard]] Result Compute(const Eigen::Ref<const Eigen::MatrixXd>& J_S,
                               const Eigen::LLT<Eigen::MatrixXd>& llt_M, double sigma0,
                               double lambda_max) noexcept {
    Result r;
    // Checked explicitly because NOTHING downstream does, and every individual
    // step launders the NaN into something that looks healthy: LLT reports
    // Success on a matrix full of NaN (every "is this pivot positive" test is a
    // comparison, and comparisons against NaN are false), and σ_min comes back
    // as a clean 0.0 because std::max(0.0, NaN) returns the FIRST argument. So
    // without this the result is r.ok == true, a finite-looking σ_min, and a
    // Λ_S of NaN — i.e. the caller's τ = h fallback never fires and the NaN is
    // published as a torque command. Same guard, same reason, as the kinematic
    // sibling DifferentialIk::Compute.
    if (!J_S.allFinite())
      return r;  // r.ok stays false; Λ_S/J̄_S/Nᵀ keep the last good contents
    J_S_ = J_S;

    r.sigma_min = SmallestSingularValue(J_S_, JJt_, saes_);
    r.lambda_sq = AdaptiveDampingSquared(r.sigma_min, sigma0, lambda_max);

    // M⁻¹ J_Sᵀ via the joint-inertia Cholesky (in place, no alloc).
    MinvJsT_ = J_S_.transpose();
    llt_M.solveInPlace(MinvJsT_);  // MinvJsT_ = M⁻¹ J_Sᵀ  (nv×m)

    // Λ_S⁻¹ = J_S M⁻¹ J_Sᵀ + λ²I, factor it.
    A_.noalias() = J_S_ * MinvJsT_;
    A_.diagonal().array() += r.lambda_sq;
    llt_A_.compute(A_);
    if (llt_A_.info() != Eigen::Success)
      return r;  // r.ok stays false — caller must treat output as invalid

    // Λ_S = A⁻¹, then J̄_S = M⁻¹ J_Sᵀ Λ_S  (nv×m).
    //
    // Unconditional, including for a caller whose posture gate is shut. That was
    // measured rather than assumed before being left alone: the impedance and
    // cascade controllers gate this whole call, so only OSC pays, and OSC needs
    // Λ_S every tick regardless (F = Λ_S·a_task) — the skippable part is J̄_S and
    // Nᵀ alone. OSC's gate is `nv > 6`, so the skip could only ever fire on a
    // non-redundant arm, where Nᵀ is nv×nv = 6×6 and the products cost O(400)
    // flops against a 6×6 SelfAdjointEigenSolver that §6.5 makes unavoidable.
    // A `with_nullspace` mode parameter on a shared RT helper is not worth that,
    // and it would leave J̄_S/Nᵀ silently stale behind the accessors.
    FormJbarAndLambda();

    // Nᵀ = I − J_Sᵀ J̄_Sᵀ = I − J_Sᵀ (M⁻¹ J_Sᵀ Λ_S)ᵀ.  J̄_Sᵀ is m×nv.
    Nt_.setIdentity();
    Nt_.noalias() -= J_S_.transpose() * JbarS_.transpose();

    r.ok = true;
    return r;
  }

  // Project a joint-space posture torque into the task nullspace: τ_null = Nᵀ·τ.
  // Writes into `out` (nv). RT-safe. Requires a successful preceding Compute().
  void ProjectNullspace(const Eigen::Ref<const Eigen::VectorXd>& tau_posture,
                        Eigen::Ref<Eigen::VectorXd> out) const noexcept {
    out.noalias() = Nt_ * tau_posture;
  }

  [[nodiscard]] const Eigen::MatrixXd& LambdaS() const noexcept { return LambdaS_; }

  [[nodiscard]] const Eigen::MatrixXd& NullspaceProjectorTranspose() const noexcept { return Nt_; }

  [[nodiscard]] const Eigen::MatrixXd& MinvJsT() const noexcept { return MinvJsT_; }

  [[nodiscard]] int nv() const noexcept { return nv_; }

  [[nodiscard]] int m() const noexcept { return m_; }

 private:
  // Materialise Λ_S = A⁻¹ (A = Λ_S⁻¹, SPD) and J̄_S = (M⁻¹ J_Sᵀ)·Λ_S. MinvJsT_
  // must already hold M⁻¹ J_Sᵀ and llt_A_ the factorisation of A.
  void FormJbarAndLambda() noexcept {
    // Λ_S = A⁻¹ via solving A·Λ_S = I.
    LambdaS_.setIdentity();
    llt_A_.solveInPlace(LambdaS_);           // Λ_S (m×m)
    JbarS_.noalias() = MinvJsT_ * LambdaS_;  // J̄_S = M⁻¹ J_Sᵀ Λ_S  (nv×m)
  }

  int nv_{0};
  int m_{0};
  Eigen::MatrixXd J_S_;
  Eigen::MatrixXd MinvJsT_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd JJt_;
  Eigen::MatrixXd LambdaS_;
  Eigen::MatrixXd JbarS_;
  Eigen::MatrixXd Nt_;
  Eigen::LLT<Eigen::MatrixXd> llt_A_;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes_;
};

}  // namespace rtc::compliance
