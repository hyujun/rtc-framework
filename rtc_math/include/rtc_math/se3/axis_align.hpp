// ── Approach-axis alignment error (Eigen-only, RT-safe) ─────────────────────
// Rotation-vector error that turns a unit axis z onto a unit target axis a_d,
// its first-order rate, and a saturated angular-velocity reference. This is a
// 2-DoF (5-DoF task) error: rotation ABOUT z (roll) is not constrained.
//
//   m = z × a_d,  c = zᵀa_d,  θ = atan2(‖m‖, c) ∈ [0, π]
//   e_a = θ · m/‖m‖                         (rotation vector, ‖e_a‖ = θ)
//   exp([e_a]×) z = a_d                      (exact, not a small-angle form)
//
// Rate (ω expressed in the same frame as z, a_d; a_d held fixed):
//   ė_a = J_a ω,   J_a = f'(c)·m mᵀ + f(c)·[a_d]×[z]×,
//   f(c) = θ/sinθ,  f'(c) = (θc/sinθ − 1)/sin²θ.
// θ→0 has finite limits (f→1, f'→−1/3); θ→π diverges because the rotation
// axis is undefined there — a property of the problem, not of the formula.
//
// Why a rotation vector and not z × a_d: ‖z × a_d‖ = sinθ is non-monotone —
// the reference collapses near 180° and jumps at any antiparallel threshold.
// ‖e_a‖ = θ is continuous and monotone (derivation and measurements:
// docs/dynamic_catching/L4_reference.md §4.5).
//
// Every function here returns FINITE values for every input, and reports the
// branch it took in AxisAlignRegion (NUM-7: no clamp that silently launders a
// bad input into a plausible value):
//   - kAlignedDeadband      ‖m‖ < sin_eps, c > 0: e_a = 0, J_a = 0.
//   - kAntiparallelDeadband ‖m‖ < sin_eps, c ≤ 0: e_a = π·u⊥ for a fixed unit
//     u⊥ ⟂ z (the axis is undefined; any perpendicular axis is a valid
//     half-turn), J_a = 0. Both deadbands return J_a = 0 because e_a is
//     constant inside them — a nonzero J_a there would no longer be the
//     derivative of the returned e_a (L4 §4.5 "구현 주의 2").
//   - kJacobianCapped       c < 0 and sin_eps ≤ ‖m‖ < jacobian_sin_floor:
//     e_a is exact, J_a evaluates f, f' at ‖m‖ = jacobian_sin_floor, which
//     bounds ‖J_a‖ by ≈ π/jacobian_sin_floor and is continuous across the
//     floor. Only AxisAlignJacobian reports this region.
//   - kInvalidInput         non-finite or non-unit z / a_d (|‖v‖−1| >
//     kAxisAlignUnitTol), or sin_eps / jacobian_sin_floor out of range:
//     every output is zero.
//
// The small-angle series for f, f' is used ONLY when c > 0. sinθ vanishes at
// θ→0 AND θ→π, so a series keyed on sinθ alone would silently replace the
// divergent antiparallel value with ≈2.645 (L4 §4.5 "구현 주의 1").
//
// Pass the SAME sin_eps to AxisAlignError and AxisAlignJacobian. With
// different values the deadbands disagree, and inside the band J_a would be
// nonzero while e_a is constant.
//
// RT: noexcept, fixed-size Eigen only, no heap, no throw, no logging.
#pragma once

#include "rtc_math/se3/so3.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace rtc::math::se3 {

/// Default deadband on ‖z × a_d‖ below which the rotation axis is treated as
/// undefined (aligned or antiparallel).
inline constexpr double kAxisAlignSinEps = 1e-6;

/// Default floor on ‖z × a_d‖ for the Jacobian near antiparallel (c < 0).
/// Below it f, f' are evaluated at the floor, so ‖J_a‖ ≲ π/1e-3 ≈ 3.1e3; the
/// floor is crossed only above θ ≈ 179.94°, well outside the 1–170° range the
/// finite-difference gate (G4-D) checks.
inline constexpr double kAxisAlignJacobianSinFloor = 1e-3;

/// Tolerance on |‖v‖ − 1| for the unit-axis inputs.
inline constexpr double kAxisAlignUnitTol = 1e-6;

/// Branch an axis-alignment function took. Only kInvalidInput means the
/// outputs must not be used; the other values are diagnostics.
enum class AxisAlignRegion : std::uint8_t {
  kRegular,
  kAlignedDeadband,
  kAntiparallelDeadband,
  kJacobianCapped,
  kInvalidInput,
};

/// Rotation-vector alignment error and the branch that produced it.
struct AxisAlignErrorResult {
  Vec3 error{Vec3::Zero()};  ///< e_a [rad], ⟂ z; zero when invalid
  AxisAlignRegion region{AxisAlignRegion::kInvalidInput};

  [[nodiscard]] bool IsValid() const noexcept { return region != AxisAlignRegion::kInvalidInput; }
};

/// J_a with ė_a = J_a ω, and the branch that produced it.
struct AxisAlignJacobianResult {
  Mat3 jacobian{Mat3::Zero()};  ///< dimensionless; zero when invalid or in a deadband
  AxisAlignRegion region{AxisAlignRegion::kInvalidInput};

  [[nodiscard]] bool IsValid() const noexcept { return region != AxisAlignRegion::kInvalidInput; }
};

/// Saturated angular-velocity reference ω = K_a·e_a, ‖ω‖ ≤ w_max.
struct AxisAlignOmegaResult {
  Vec3 omega{Vec3::Zero()};  ///< [rad/s]; zero when invalid
  bool saturated{false};     ///< ‖K_a·e_a‖ exceeded w_max and was scaled down
  bool valid{false};         ///< false → non-finite error or invalid gains; omega is zero
};

namespace detail {

[[nodiscard]] inline bool IsUnitAxis(const Vec3& v) noexcept {
  return v.allFinite() && std::abs(v.norm() - 1.0) <= kAxisAlignUnitTol;
}

// sin_eps must be a positive finite value below 1 (‖z × a_d‖ ≤ 1 for unit
// inputs, so sin_eps ≥ 1 would make every input a deadband).
[[nodiscard]] inline bool IsValidSinBound(double s) noexcept {
  return std::isfinite(s) && s > 0.0 && s < 1.0;
}

// A fixed unit axis ⟂ z. Deterministic in z so the antiparallel error does not
// flicker between calls.
[[nodiscard]] inline Vec3 PerpendicularUnit(const Vec3& z) noexcept {
  const Vec3 r = (std::abs(z.x()) < 0.9) ? Vec3::UnitX() : Vec3::UnitY();
  return z.cross(r).normalized();
}

// Below this θ (and only for c > 0) f, f' use their 2nd-order series. The
// closed form f' = (θc/n − 1)/n² cancels catastrophically as θ→0 (relative
// error ≈ εₘ/θ² ≈ 1e-10 at 1e-3); the series truncation error is O(θ⁴) ≈ 1e-12.
inline constexpr double kAxisAlignSeriesTheta = 1e-3;

}  // namespace detail

/// Rotation-vector error e_a with exp([e_a]×) z = a_d.
/// @param z       current unit axis
/// @param a_d     target unit axis (same frame as z)
/// @param sin_eps deadband on ‖z × a_d‖, in (0, 1)
[[nodiscard]] inline AxisAlignErrorResult AxisAlignError(
    const Vec3& z, const Vec3& a_d, double sin_eps = kAxisAlignSinEps) noexcept {
  AxisAlignErrorResult out;
  if (!detail::IsUnitAxis(z) || !detail::IsUnitAxis(a_d) || !detail::IsValidSinBound(sin_eps)) {
    return out;  // kInvalidInput, error = 0
  }
  const Vec3 m = z.cross(a_d);
  const double n = m.norm();
  const double c = z.dot(a_d);
  if (n < sin_eps) {
    if (c > 0.0) {
      out.region = AxisAlignRegion::kAlignedDeadband;
      return out;  // error = 0
    }
    out.error = std::numbers::pi * detail::PerpendicularUnit(z);
    out.region = AxisAlignRegion::kAntiparallelDeadband;
    return out;
  }
  out.error = (std::atan2(n, c) / n) * m;
  out.region = AxisAlignRegion::kRegular;
  return out;
}

/// J_a with ė_a = J_a ω (ω in the frame of z and a_d, a_d held fixed).
/// @param z                  current unit axis
/// @param a_d                target unit axis
/// @param sin_eps            deadband, must equal the value given to AxisAlignError
/// @param jacobian_sin_floor floor on ‖z × a_d‖ for c < 0, in [sin_eps, 1)
[[nodiscard]] inline AxisAlignJacobianResult AxisAlignJacobian(
    const Vec3& z, const Vec3& a_d, double sin_eps = kAxisAlignSinEps,
    double jacobian_sin_floor = kAxisAlignJacobianSinFloor) noexcept {
  AxisAlignJacobianResult out;
  if (!detail::IsUnitAxis(z) || !detail::IsUnitAxis(a_d) || !detail::IsValidSinBound(sin_eps) ||
      !detail::IsValidSinBound(jacobian_sin_floor) || jacobian_sin_floor < sin_eps) {
    return out;  // kInvalidInput, jacobian = 0
  }
  const Vec3 m = z.cross(a_d);
  const double n = m.norm();
  // Unit inputs give |c| ≤ 1 up to rounding; keep atan2 and the series on the
  // mathematical range.
  const double c = std::clamp(z.dot(a_d), -1.0, 1.0);
  if (n < sin_eps) {
    out.region =
        (c > 0.0) ? AxisAlignRegion::kAlignedDeadband : AxisAlignRegion::kAntiparallelDeadband;
    return out;  // jacobian = 0: e_a is constant inside both deadbands
  }
  const double theta = std::atan2(n, c);
  double f{0.0};   // θ / sinθ
  double fp{0.0};  // df/dc
  if (c > 0.0 && theta < detail::kAxisAlignSeriesTheta) {
    const double t2 = theta * theta;
    f = 1.0 + t2 / 6.0;
    fp = -1.0 / 3.0 - 2.0 * t2 / 15.0;
    out.region = AxisAlignRegion::kRegular;
  } else {
    double n_eval = n;
    out.region = AxisAlignRegion::kRegular;
    if (c < 0.0 && n < jacobian_sin_floor) {
      n_eval = jacobian_sin_floor;
      out.region = AxisAlignRegion::kJacobianCapped;
    }
    f = theta / n_eval;
    fp = (theta * c / n_eval - 1.0) / (n_eval * n_eval);
  }
  out.jacobian.noalias() = fp * (m * m.transpose());
  out.jacobian.noalias() += f * (hat(a_d) * hat(z));
  return out;
}

/// ω = k_axis·e_a scaled down to ‖ω‖ ≤ w_max.
/// @param error  e_a from AxisAlignError (must be finite)
/// @param k_axis gain [1/s], finite and ≥ 0
/// @param w_max  norm limit [rad/s], finite and > 0
[[nodiscard]] inline AxisAlignOmegaResult AxisAlignOmega(const Vec3& error, double k_axis,
                                                         double w_max) noexcept {
  AxisAlignOmegaResult out;
  if (!error.allFinite() || !std::isfinite(k_axis) || k_axis < 0.0 || !std::isfinite(w_max) ||
      w_max <= 0.0) {
    return out;  // valid = false, omega = 0
  }
  out.omega = k_axis * error;
  const double norm = out.omega.norm();
  if (!std::isfinite(norm)) {  // k_axis·error overflowed
    out.omega.setZero();
    return out;
  }
  if (norm > w_max) {
    out.omega *= w_max / norm;
    out.saturated = true;
  }
  out.valid = true;
  return out;
}

}  // namespace rtc::math::se3
