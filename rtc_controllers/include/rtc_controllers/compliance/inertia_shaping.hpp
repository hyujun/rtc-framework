// ── Task-space inertia shaping (spec §6.3) ──────────────────────────────────
// The §6.3 shaping factor applied to the §6.2 task force, and nothing else:
//
//     B   = Λ_S Λ_d⁻¹                                  (m×m)
//     B_c = I + s·(B − I),  s = min(1, (r−1)/‖B−I‖∞)   (§5.2 deviation clamp)
//     f_cmd = B_c·f_task + (B_c − I)·f_ext             (first m task rows)
//
// The caller owns Λ_S (and therefore the model, the DLS and the σ_min faults it
// exposes), the §6.2 force this multiplies, the wrench pipeline that produces
// f_ext, the activation ramp already folded into both, and the τ = Jᵀ Sᵀ f_cmd
// that consumes the result.
//
// ── Why Λ_S is an argument and not a TaskDynamics reference ─────────────────
// The pre-extraction code read `dyn_.LambdaS()` directly. Taking the helper
// instead of its output would make this law know about compliance::TaskDynamics
// — and #236 S2b still has to decide whether OSC's inline Λ block converges ONTO
// that helper or the helper moves. A law that names the helper pre-empts that
// decision. Same boundary the sibling cores draw around the error definition and
// the frame transport; the rule is agent_docs/design-principles.md §코어의 형태.
//
// ── Why the scratch is Matrix<double, Dynamic, Dynamic, 0, 6, 6> (D-S4a) ────
// A task is at most 6-dimensional, so the temptation is a fixed 6×6. That does
// not work: under TRANSLATION_ONLY m = 3, and feeding a 3×3 block to an
// LLT<Matrix<double,6,6>> ABORTS at run time (PlainObjectBase::resize assert)
// while compiling cleanly. The max-size dynamic type keeps the logical size
// dynamic and the storage fixed — heap-free like the fixed type, sized like
// MatrixXd. That it is also BIT-identical to the MatrixXd the adapter used was
// measured before the extraction (20k trials × 720k doubles, -O0 and -O2, m =
// 3..6, zero mismatch; the control group — materialising Λ_d⁻¹ and multiplying —
// differed on 364,699 of them, which is what makes the zero meaningful). The
// literal oracle in test_inertia_shaping_core.cpp keeps the MatrixXd form and
// re-measures that here rather than trusting the probe.
//
// ── The sign bridge lives here, deliberately ────────────────────────────────
// f_ext is the wrench the ENVIRONMENT applies ON the robot (this package's input
// contract, §2 / §3.1) — the opposite of the design spec's convention, so the
// spec's (B − I)(−S·f_ext) becomes +(B − I)·f_ext on substitution. The physics
// settles it without appealing to either convention: at Λ_d → ∞ (B → 0) an
// infinitely heavy desired inertia must be immovable under an external push,
// which needs f_cmd → −f_ext in Λν̇ = f_cmd + f_ext. The negated form yields
// +f_ext — twice the unshaped compliance exactly where the operator asked for
// none. Pinned by TaskImpedanceWrench.HeavyDesiredInertiaCancelsTheExternalWrench
// and documented in docs/compliance-conventions.md §3.3.
//
// ── Extraction note (#236 S4) ───────────────────────────────────────────────
// This lived inline as TaskImpedanceController::ApplyInertiaShaping
// (task_impedance_controller.cpp:247-327 at 697f16d2). It was lifted here with
// the same operation order, the same association and the same expression shapes;
// only the scratch TYPE changed (above) and the two diagnostic flags became
// return values instead of writes through a Diagnostics&.
// InertiaShaping.MatchesThePreExtractionInlineFormBitwise pins that against a
// literal copy of the pre-extraction form, and CoreDrivenShim* pins the wiring
// against the live adapter while the adapter still exists.
#pragma once

#include <Eigen/Cholesky>  // LLT
#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cstddef>

namespace rtc::compliance {

/// Λ_d and the §5.2 bound of the §6.3 law. Trivially copyable so a controller
/// can hold it inside a SeqLock'd Gains POD.
struct InertiaShapingParams {
  /// Λ_d diagonal [kg | kg·m²], task-axis order [x,y,z, rx,ry,rz]. Every entry
  /// used (i < m) must be > 0 — the caller validates at configure time; a zero
  /// here is not a NUM-2 divide but a Cholesky failure, reported as such.
  std::array<double, 6> desired_inertia{{1.0, 1.0, 1.0, 0.1, 0.1, 0.1}};
  /// Λ_d := Λ_S ⇒ B = I exactly. Reached through the REAL factorisation and
  /// solve rather than by short-circuiting, which is what makes the neutral case
  /// a test of this code path instead of a test of an `if` (§11.4 T4.1).
  bool desired_inertia_natural{true};
  /// §5.2 MUST: bound on ‖B‖∞, enforced on the deviation from identity.
  double max_inertia_ratio{3.0};
};

/// Outcome of one §6.3 evaluation: the shaped task force plus the two conditions
/// an operator has to be able to tell apart.
struct InertiaShapingResult {
  /// Shaped task force. Rows ≥ m carry @p f_task unchanged, which is what the
  /// selection matrix leaves them as.
  Eigen::Matrix<double, 6, 1> f_cmd{Eigen::Matrix<double, 6, 1>::Zero()};
  /// Λ_d could not be factorised ⇒ degraded to B = I (the §6.2 law) and @p f_cmd
  /// is @p f_task. Its OWN flag: a numerical breakdown and the safety clamp below
  /// call for different operator responses, and sharing one flag would let the
  /// clamp test pass on a factorisation failure.
  bool solve_failed{false};
  /// The §5.2 deviation clamp engaged this tick (a tuning bound doing its job).
  bool clamped{false};
};

/// @brief f_cmd = B_c·f_task + (B_c − I)·f_ext with B = Λ_S Λ_d⁻¹ (spec §6.3).
///
/// @param p         Λ_d specification and the §5.2 ratio bound
/// @param lambda_s  task inertia Λ_S (at least m×m; SPD in the regime this is
///                  used — the DLS in the caller is what keeps it so)
/// @param f_task    §6.2 task force, already ramped by α (6; rows ≥ m ignored
///                  except that they pass through to the result)
/// @param f_ext     external wrench in the SAME frame as @p f_task, sign =
///                  environment ON robot, already ramped by α (6)
/// @param m         task dimension; clamped to [0,6] so a mis-sized caller
///                  cannot walk past the fixed 6-vectors
/// @return the shaped force and the two condition flags.
///
/// @note RT-safe: the only storage is max-size-fixed (above), so there is no
///       heap and no allocation even at m < 6. No divide except by ‖B−I‖∞,
///       which is guarded by the branch that reaches it (dev_inf > r−1 ≥ 0).
/// @note Not a `noexcept` lie: Eigen's LLT on a fixed-storage matrix cannot
///       throw, and a non-factorable Λ_d is reported through @c solve_failed
///       rather than an exception (RT-2).
[[nodiscard]] inline InertiaShapingResult ComputeShapedTaskForce(
    const InertiaShapingParams& p, const Eigen::Ref<const Eigen::MatrixXd>& lambda_s,
    const Eigen::Matrix<double, 6, 1>& f_task, const Eigen::Matrix<double, 6, 1>& f_ext,
    int m) noexcept {
  // Storage is compile-time bounded at 6×6, the logical size is m×m. See the
  // header note: the purely fixed type aborts at m < 6.
  using ScratchMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 6>;

  InertiaShapingResult out;
  out.f_cmd = f_task;  // the degraded answer, and the rows ≥ m answer
  const int rows = std::clamp(m, 0, 6);

  // Λ_d. "natural" means Λ_d := Λ_S, which drives B to the identity through the
  // real solve rather than by short-circuiting it.
  ScratchMatrix lambda_d(rows, rows);
  if (p.desired_inertia_natural) {
    lambda_d = lambda_s.topLeftCorner(rows, rows);
  } else {
    lambda_d.setZero();
    for (int i = 0; i < rows; ++i)
      lambda_d(i, i) = p.desired_inertia[static_cast<std::size_t>(i)];  // > 0 (caller validates)
  }

  Eigen::LLT<ScratchMatrix> llt_lambda_d;
  llt_lambda_d.compute(lambda_d);
  if (llt_lambda_d.info() != Eigen::Success) {
    // Λ_d not factorable — Λ_S can lose definiteness at a singular pose even
    // with the DLS. Degrade to B = I (i.e. the §6.2 law, which needs no Λ at
    // all) instead of emitting a garbage shaping matrix; f_cmd already holds
    // f_task. The caller's σ_min faults still fire on the underlying condition.
    out.solve_failed = true;
    return out;
  }

  // Bᵀ = Λ_d⁻¹ Λ_S. Both operands are symmetric, so B = Λ_S Λ_d⁻¹ = (Bᵀ)ᵀ and
  // B(i,j) reads as b_transp(j,i) — no explicit transpose, no temporary.
  ScratchMatrix b_transp = lambda_s.topLeftCorner(rows, rows);
  llt_lambda_d.solveInPlace(b_transp);

  // §5.2 MUST — bound ‖Λ_S Λ_d⁻¹‖. Enforced on the DEVIATION from identity:
  // with s = (r_max − 1)/‖B − I‖∞ the clamped B_c = I + s(B − I) satisfies
  // ‖B_c‖∞ ≤ ‖I‖∞ + s‖B − I‖∞ = r_max exactly. Clamping the deviation rather
  // than scaling B keeps the map continuous AND leaves B = I untouched, so a
  // neutral Λ_d = Λ_S can never be perturbed by the safety clamp (T4.1).
  double dev_inf = 0.0;
  for (int i = 0; i < rows; ++i) {
    double row = 0.0;
    for (int j = 0; j < rows; ++j)
      row += std::abs(b_transp(j, i) - (i == j ? 1.0 : 0.0));
    dev_inf = std::max(dev_inf, row);
  }
  const double ratio_max = std::max(1.0, p.max_inertia_ratio);
  double s = 1.0;
  if (dev_inf > ratio_max - 1.0) {
    s = (ratio_max - 1.0) / dev_inf;  // dev_inf > ratio_max−1 ≥ 0 ⇒ dev_inf > 0
    out.clamped = true;
  }

  // f_cmd = B·f_task + (B − I)·f_ext, component-wise over the m task rows. The
  // sign of the second term is the bridge documented in the header note.
  //
  // Explicit loops (m ≤ 6) rather than an Eigen expression: fixed-size, provably
  // heap-free, and no `auto`-aliasing trap (RT-1 / RT-5).
  for (int i = 0; i < rows; ++i) {
    double acc = 0.0;
    for (int j = 0; j < rows; ++j) {
      const double eye = (i == j) ? 1.0 : 0.0;
      const double b_ij = eye + s * (b_transp(j, i) - eye);
      acc += b_ij * f_task(j) + (b_ij - eye) * f_ext(j);
    }
    out.f_cmd(i) = acc;
  }
  return out;
}

}  // namespace rtc::compliance
