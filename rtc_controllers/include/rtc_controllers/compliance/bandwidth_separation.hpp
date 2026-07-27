// ── §7.6 MUST-1 bandwidth separation verdict (#236 S6) ──────────────────────
// A cascade of an outer admittance loop inside an inner impedance loop is only
// well-posed if the inner loop is FASTER than the outer one. The spec states
// that as a per-axis frequency ratio:
//
//     ω_i = √(K_p^i / Λ_S)   inner, at the seeding pose
//     ω_a = √(K_p^a / Λ_d)   outer
//     verdict = min over task axes of  ω_i / ω_a
//
// and nothing else: the caller owns the Λ_S that feeds it, when to evaluate,
// where to publish the figure and what (if anything) to do about a low one.
//
// ── Why the WORST axis and not an average ───────────────────────────────────
// A cascade separated on five axes and coupled on the sixth is coupled. The
// figure is a min, and the axes that carry no verdict drop out of it rather
// than defaulting to something that would dominate the min in either direction:
//
//   * K_p^a = 0 is hand-guiding (§7.6 MUST-3) — the outer loop has no restoring
//     frequency to be separated FROM, so the axis carries no ratio at all.
//   * Λ_d ≤ 0 or a non-positive Λ_S diagonal means the ratio cannot be FORMED —
//     a degenerate seeding pose, or a gain the §7.4 floor should have caught.
//   * K_p^i = 0 is emphatically NOT "not evaluable": ω_i = 0 under a positive
//     ω_a is the worst separation there is, an inner loop with no restoring
//     bandwidth beneath an outer loop that has one. Skipping it removed the
//     single most coupled axis from a min() whose whole job is to report the
//     most coupled axis, and `inner.kp_pos: [0,0,0]` passes LoadConfig (>= 0),
//     so the flag read "fine" for exactly the configuration it exists to catch.
//
// ── Why this is a law and not a controller method ───────────────────────────
// §7.6 MUST-1 is a statement about two gain sets and an inertia — a property of
// the SPEC, not a circumstance of the class that happened to implement it. It
// reads no model handle, no mailbox and no lifecycle state: Λ_S arrives as a
// matrix, exactly as compliance/inertia_shaping.hpp takes it, so the verdict
// stays clear of the #236 S2b decision about which helper should produce it.
//
// ── Extraction note (#236 S6) ───────────────────────────────────────────────
// This lived as CascadedComplianceController::EvaluateBandwidthSeparation
// (8526a4c9, cascaded_compliance_controller.cpp:225-273), which wrote two
// members. It was lifted here verbatim — same axis order, same comparison
// forms, same early-outs — and now RETURNS the pair instead;
// BandwidthSeparation.MatchesThePreExtractionInlineFormBitwise pins that
// against a literal copy of the pre-extraction body.
#pragma once

#include <rtc_controllers/compliance/admittance_integrator.hpp>
#include <rtc_controllers/compliance/impedance_law.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace rtc::compliance {

/// The §7.6 MUST-1 verdict. Diagnostic only: a sluggish inner loop is a tuning
/// statement about two gain sets, and latching a fault for it would stop a robot
/// that is merely soft (#236 D20).
struct BandwidthSeparation {
  /// min over task axes of ω_i/ω_a. +∞ means NO axis carried a verdict — an
  /// all-hand-guiding or degenerate configuration, not a good separation.
  double ratio{std::numeric_limits<double>::infinity()};
  /// ratio is finite AND below the configured minimum. Never true for +∞, so
  /// "not evaluable" cannot masquerade as "separated" or as "coupled".
  bool low{false};
};

/// Evaluate §7.6 MUST-1 for a cascade.
///
/// @param outer      outer admittance gains — K_p^a (`stiffness`) and Λ_d
///                   (`inertia`, floored by `min_inertia_lin`/`_ang` exactly as
///                   AdmittanceIntegrator::Step() floors it, so the reported
///                   ratio cannot disagree with the Λ_d the integrator runs)
/// @param inner      inner impedance gains — K_p^i (`kp_pos` / `kp_rot`)
/// @param lambda_s   task-space inertia Λ_S at the pose being judged; only the
///                   DIAGONAL is read. Passed as a matrix rather than as a
///                   diagonal vector because `lambda_s.diagonal()` has a
///                   non-unit inner stride, and binding that to a plain
///                   Eigen::Ref<const VectorXd> would heap-copy on the RT path
///                   (RT-1).
/// @param min_ratio  the configured minimum; 0 silences @ref
///                   BandwidthSeparation::low rather than latching anything
/// @return the worst-axis ratio and the flag
///
/// Axes beyond `lambda_s`'s size carry no verdict. Eigen::Ref arguments bring
/// their own dimensions, so a caller that hands over a smaller Λ_S than the six
/// task axes gets those axes dropped rather than a read past the buffer (the
/// #236 S4 lesson: `eigen_assert` cannot be the sensor, because the library TU
/// is built -DNDEBUG). The shipping caller passes 6×6 and the bound is a no-op.
///
/// RT-safe: noexcept, no heap, fixed loop of at most six axes. The two divides
/// are guarded by the positivity tests above them (NUM-2), and √ of a positive
/// quotient cannot be NaN.
[[nodiscard]] inline BandwidthSeparation EvaluateBandwidthSeparation(
    const AdmittanceParams& outer, const ImpedanceParams& inner,
    const Eigen::Ref<const Eigen::MatrixXd>& lambda_s, double min_ratio) noexcept {
  constexpr int kTaskDim = 6;
  const int axes = static_cast<int>(
      std::min<Eigen::Index>(kTaskDim, std::min(lambda_s.rows(), lambda_s.cols())));
  double worst = std::numeric_limits<double>::infinity();
  for (int i = 0; i < axes; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double kp_a = outer.stiffness[ui];
    if (!(kp_a > 0.0))
      continue;
    const double lambda_d =
        std::max(outer.inertia[ui], (i < 3) ? outer.min_inertia_lin : outer.min_inertia_ang);
    const double lambda_i = lambda_s(i, i);
    const double kp_i = (i < 3) ? inner.kp_pos[ui] : inner.kp_rot[ui - 3];
    if (!(lambda_d > 0.0) || !(lambda_i > 0.0))
      continue;
    if (!(kp_i > 0.0)) {
      worst = 0.0;
      continue;
    }
    const double omega_a = std::sqrt(kp_a / lambda_d);
    const double omega_i = std::sqrt(kp_i / lambda_i);
    if (!(omega_a > 0.0))
      continue;
    worst = std::min(worst, omega_i / omega_a);
  }
  return BandwidthSeparation{worst, std::isfinite(worst) && (worst < min_ratio)};
}

}  // namespace rtc::compliance
