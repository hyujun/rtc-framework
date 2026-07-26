// ── Task-space impedance law (spec §6.2) ────────────────────────────────────
// The bracketed task force of the Jacobian-transpose stiffness law
//
//     f_task = α·[K_p·e + K_d·ė],      ė = ν_d − ν      (all LOCAL_WORLD_ALIGNED)
//
// and nothing else: the caller owns the Jacobian (τ = Jᵀ Sᵀ f_task), the model,
// the gravity term and the safety layer. That boundary is what makes this unit
// testable with nothing but Eigen — and what lets the SAME law serve both a
// standalone impedance controller and the inner loop of a cascade.
//
// ── Why ν_d is an explicit argument ─────────────────────────────────────────
// A standalone task-impedance controller regulates to a STATIC setpoint, so
// ν_d = 0 and ė collapses to −ν = −J·q̇. The §7.6 cascade does not: its outer
// admittance loop produces a moving compliant frame (X_c, ν_c), and the inner
// impedance loop must track that frame's VELOCITY as well as its pose. Feeding
// ν_d = 0 there would make the inner loop fight the outer loop's motion with
// K_d·ν_c of spurious damping — the whole point of the cascade is that the
// compliant frame moves and the impedance follows it. Hence ν_d is a parameter
// and not an assumption; `TaskImpedanceController` passes zero, the cascade
// passes ν_c.
//
// ── Sign convention (§6.2, compliance-conventions.md §3.3) ──────────────────
// `e` runs CURRENT → DESIRED (rtc::math::se3::computePoseError(X, X_d)), so the
// force is +K_p·e: it pulls the tip TOWARD the goal. Note this is the opposite
// direction to the admittance deviation x̃_c = e(X_d, X_c) of
// admittance_integrator.hpp, which is exactly the seam slice 2 already shipped
// one sign defect at — the two are adjacent in a cascade and read the same way.
//
// ── Extraction note (#236 D18) ──────────────────────────────────────────────
// This expression lived inline in TaskImpedanceController::Compute(). It was
// lifted here verbatim — same operation order, same association — so the
// migration is bit-for-bit inert; ImpedanceLaw.MatchesThePreExtractionInlineForm
// pins that against a literal copy of the pre-extraction form. Preserve the
// association if you touch it: `α·(kp·e + kd·ė)` is NOT bitwise `α·kp·e +
// α·kd·ė`.
#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cstddef>

namespace rtc::compliance {

/// Per-axis Cartesian stiffness / damping of the §6.2 law. K_p is a STIFFNESS
/// [N/m | N·m/rad] and K_d a DAMPING [N·s/m | N·m·s/rad] — not the OSC's
/// acceleration-form gains, which differ from these by a factor of Λ. Trivially
/// copyable so a controller can hold it inside a SeqLock'd Gains POD.
struct ImpedanceParams {
  std::array<double, 3> kp_pos{{200.0, 200.0, 200.0}};  ///< translation stiffness
  std::array<double, 3> kd_pos{{28.0, 28.0, 28.0}};     ///< translation damping (≈2√kp)
  std::array<double, 3> kp_rot{{20.0, 20.0, 20.0}};     ///< rotation stiffness (m = 6 only)
  std::array<double, 3> kd_rot{{9.0, 9.0, 9.0}};        ///< rotation damping (m = 6 only)
};

/// f_task = α·[K_p·e + K_d·(ν_d − ν)] on the first `m` task rows; rows ≥ m are
/// left at ZERO, which is what the selection matrix S does under
/// TRANSLATION_ONLY (m = 3) — the caller then uses only head(m).
///
/// @param k      stiffness / damping, task-axis order [x,y,z, rx,ry,rz]
/// @param e      pose error CURRENT → DESIRED, LWA (6)
/// @param nu     measured task twist ν = J·q̇, LWA (6)
/// @param nu_d   desired task twist ν_d, LWA (6) — zero for a static setpoint
/// @param alpha  activation ramp ∈ [0,1] (§10.7); the gravity term the caller
///               adds is never ramped, only this force is
/// @param m      task dimension; clamped to [0,6] so a mis-sized caller cannot
///               write past the fixed 6-vector
///
/// RT-safe: fixed-size return (stack), no heap, no branch on data, no divide —
/// there is nothing here for NUM-2 to guard. Explicit loops rather than an Eigen
/// expression keep it free of the `auto`-aliasing trap (RT-5) and provably
/// allocation-free (RT-1).
[[nodiscard]] inline Eigen::Matrix<double, 6, 1> ComputeImpedanceForce(
    const ImpedanceParams& k, const Eigen::Ref<const Eigen::VectorXd>& e,
    const Eigen::Ref<const Eigen::VectorXd>& nu, const Eigen::Ref<const Eigen::VectorXd>& nu_d,
    double alpha, int m) noexcept {
  Eigen::Matrix<double, 6, 1> f_task = Eigen::Matrix<double, 6, 1>::Zero();
  const int rows = std::clamp(m, 0, 6);
  for (int i = 0; i < rows; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double kp = (i < 3) ? k.kp_pos[ui] : k.kp_rot[ui - 3];
    const double kd = (i < 3) ? k.kd_pos[ui] : k.kd_rot[ui - 3];
    const double edot = nu_d(i) - nu(i);
    f_task(i) = alpha * (kp * e(i) + kd * edot);  // +K_p·e (sign per §6.2)
  }
  return f_task;
}

}  // namespace rtc::compliance
