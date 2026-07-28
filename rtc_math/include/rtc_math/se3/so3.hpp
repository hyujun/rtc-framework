// ── SO(3) math core (Eigen-only, RT-safe) ───────────────────────────────────
// Header-only rotation-group primitives for the SE(3) pose/twist-error module.
//
// Design (see se3/README or agent_docs): the numeric core depends on Eigen ONLY
// — no Pinocchio — so it cross-validates against pinocchio::log3 / Jlog3 in the
// tests and stays reusable by any consumer. All functions are RT-safe:
//   - noexcept, no heap allocation, fixed-size Eigen only
//   - no `auto` on Eigen expressions (aliasing — RT rule §3 / rt-path.md)
//   - no throw / no logging
//
// References:
//   Murray, Li, Sastry 1994; Lynch & Park, Modern Robotics 2017 §3.2;
//   Solà et al., "A micro Lie theory" arXiv:1812.01537 (Jacobian conventions);
//   Nakanishi et al. 2008 (quaternion error).
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace rtc::math::se3 {

// ── Fixed-size aliases shared across the module ──────────────────────────────
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Iso3 = Eigen::Isometry3d;

// Small-angle threshold for trig-coefficient branches. Chosen at 1e-4 (NOT
// machine-eps): below ~1e-4 the closed forms (1-cosθ)/θ² etc. lose precision to
// catastrophic cancellation, while a 2nd-order Taylor has error O(θ⁴) ≈ 1e-16
// there — so the Taylor branch is the *more* accurate one in that range. The
// two branches agree to far better than test tolerances at the crossover, which
// makes the θ→0 continuity test (test 2) pass trivially.
inline constexpr double kSmallAngle = 1e-4;
// Separate (tighter) guard for log3's quaternion-vector norm: atan2 has no
// cancellation, so the exact branch is valid down to very small norms.
inline constexpr double kQuatVecEps = 1e-8;

// ── hat / vee ────────────────────────────────────────────────────────────────

// hat: ℝ³ → so(3), the skew-symmetric matrix [w]× with [w]× u = w × u.
[[nodiscard]] inline Mat3 hat(const Vec3& w) noexcept {
  Mat3 W;
  W << 0.0, -w.z(), w.y(), w.z(), 0.0, -w.x(), -w.y(), w.x(), 0.0;
  return W;
}

// vee: so(3) → ℝ³, inverse of hat (reads the skew part; assumes W antisymmetric).
[[nodiscard]] inline Vec3 vee(const Mat3& W) noexcept {
  return Vec3(W(2, 1) - W(1, 2), W(0, 2) - W(2, 0), W(1, 0) - W(0, 1)) * 0.5;
}

// ── log3 (quaternion + atan2) ────────────────────────────────────────────────
//
// Returns the rotation vector φ = θ·n (axis n, angle θ) with R = exp([φ]×).
//
// Quaternion + atan2 is MANDATORY here (not acos(½(tr R − 1))): acos's condition
// number blows up at θ≈0 and θ≈π, dropping accuracy to O(√εₘ). With the unit
// quaternion q = (w, v):
//   θ = 2·atan2(‖v‖, w)         is well-conditioned over the whole range,
// and forcing w ≥ 0 pins θ ∈ [0, π] (shortest path, avoids "unwinding").
[[nodiscard]] inline Vec3 log3(const Mat3& R) noexcept {
  Eigen::Quaterniond q(R);
  q.normalize();
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;  // θ ∈ [0, π]: pick the hemisphere with w ≥ 0
  }
  const Vec3 v = q.vec();
  const double vn = v.norm();
  if (vn < kQuatVecEps) {
    // sin(θ/2) ≈ θ/2 ⇒ φ = θ·n ≈ 2·v (first order, no division).
    return 2.0 * v;
  }
  const double theta = 2.0 * std::atan2(vn, q.w());
  return (theta / vn) * v;
}

// ── exp3 (Rodrigues) ─────────────────────────────────────────────────────────
//
// exp([φ]×) = I + a·[φ]× + b·[φ]×²,  a = sinθ/θ,  b = (1−cosθ)/θ²,  θ = ‖φ‖.
[[nodiscard]] inline Mat3 exp3(const Vec3& phi) noexcept {
  const double t2 = phi.squaredNorm();
  const double theta = std::sqrt(t2);
  const Mat3 W = hat(phi);
  double a;  // sinθ/θ
  double b;  // (1−cosθ)/θ²
  if (theta < kSmallAngle) {
    a = 1.0 - t2 / 6.0;   // sinθ/θ
    b = 0.5 - t2 / 24.0;  // (1−cosθ)/θ²
  } else {
    a = std::sin(theta) / theta;
    b = (1.0 - std::cos(theta)) / t2;
  }
  return Mat3::Identity() + a * W + b * (W * W);
}

// ── Left Jacobian J_l and its inverse ────────────────────────────────────────
//
// J_l(φ) = I + c1·[φ]× + c2·[φ]×²,  c1 = (1−cosθ)/θ²,  c2 = (θ−sinθ)/θ³.
// (Barfoot 2017 / micro-Lie-theory; left/global trivialization.)
[[nodiscard]] inline Mat3 leftJacobian(const Vec3& phi) noexcept {
  const double t2 = phi.squaredNorm();
  const double theta = std::sqrt(t2);
  const Mat3 W = hat(phi);
  double c1;  // (1−cosθ)/θ²
  double c2;  // (θ−sinθ)/θ³
  if (theta < kSmallAngle) {
    c1 = 0.5 - t2 / 24.0;
    c2 = 1.0 / 6.0 - t2 / 120.0;
  } else {
    c1 = (1.0 - std::cos(theta)) / t2;
    c2 = (theta - std::sin(theta)) / (t2 * theta);
  }
  return Mat3::Identity() + c1 * W + c2 * (W * W);
}

// J_l(φ)⁻¹ = I − ½[φ]× + c·[φ]×²,  c = (1 − (θ/2)cot(θ/2)) / θ²,  Taylor c → 1/12.
//
// The cot(θ/2) factor diverges only at θ = 2π. Because every φ fed here comes
// from log3 (which guarantees θ ≤ π ⇒ θ/2 ≤ π/2 ⇒ cot(θ/2) ≥ 0 finite), that
// singularity is unreachable — documented, not guarded.
[[nodiscard]] inline Mat3 leftJacobianInverse(const Vec3& phi) noexcept {
  const double t2 = phi.squaredNorm();
  const double theta = std::sqrt(t2);
  const Mat3 W = hat(phi);
  double c;  // (1 − (θ/2)cot(θ/2)) / θ²
  if (theta < kSmallAngle) {
    c = 1.0 / 12.0 + t2 / 720.0;
  } else {
    const double half = 0.5 * theta;
    // (θ/2)cot(θ/2) = half · cos(half)/sin(half)
    const double gamma = half * std::cos(half) / std::sin(half);
    c = (1.0 - gamma) / t2;
  }
  return Mat3::Identity() - 0.5 * W + c * (W * W);
}

// ── Jlog3: right (local) perturbation Jacobian of log3 ───────────────────────
//
// Right convention (matches Pinocchio's Jlog3):
//   log3(R · exp([δ]×)) ≈ log3(R) + Jlog3(R)·δ.
// This is the inverse right Jacobian at φ = log3(R):
//   Jlog3(R) = J_r⁻¹(φ) = J_l⁻¹(φ)ᵀ = I + ½[φ]× + c·[φ]×²   (same c as J_l⁻¹).
[[nodiscard]] inline Mat3 Jlog3(const Mat3& R) noexcept {
  const Vec3 phi = log3(R);
  const double t2 = phi.squaredNorm();
  const double theta = std::sqrt(t2);
  const Mat3 W = hat(phi);
  double c;
  if (theta < kSmallAngle) {
    c = 1.0 / 12.0 + t2 / 720.0;
  } else {
    const double half = 0.5 * theta;
    const double gamma = half * std::cos(half) / std::sin(half);
    c = (1.0 - gamma) / t2;
  }
  return Mat3::Identity() + 0.5 * W + c * (W * W);
}

// ── RPY → R, the boundary convention ────────────────────────────────────────
//
// R = Rz(yaw)·Ry(pitch)·Rx(roll), i.e. intrinsic Z-Y'-X'' — the "ZYX Euler at
// boundaries" rule in CLAUDE.md §10. Internal math stays on quaternions and
// log/exp; this exists only for the wire edge, where operators and messages
// still speak (x, y, z, r, p, y).
//
// Placed here rather than re-derived per call site: the same three AngleAxisd
// lines were written twice in the controller bindings with different spellings,
// which is the drift class #206 was opened to remove. Changing the convention
// must be one edit.
//
// Singular at pitch = ±π/2 in the INVERSE direction only (gimbal lock is a
// property of the R → rpy map); this direction is total and well conditioned.
//
// Takes one Vec3 rather than three doubles on purpose: three adjacent scalars
// of the same type are the easiest possible thing to transpose at a call site,
// and a transposed roll/yaw produces a plausible-looking rotation rather than
// an obvious failure.
[[nodiscard]] inline Mat3 RpyToRotationZyx(const Vec3& rpy) noexcept {
  const Eigen::AngleAxisd rx(rpy.x(), Vec3::UnitX());
  const Eigen::AngleAxisd ry(rpy.y(), Vec3::UnitY());
  const Eigen::AngleAxisd rz(rpy.z(), Vec3::UnitZ());
  return (rz * ry * rx).toRotationMatrix();
}

}  // namespace rtc::math::se3
