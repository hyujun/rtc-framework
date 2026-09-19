// ── Ball flight model for the catching core's tests (test-only fixture) ──────
// Quadratic-drag point-mass ball: a = g − k‖v‖v, with the drag coefficient k
// carried as a constant 7th state so the state-transition matrix Φ = ∂x(T)/∂x(0)
// also covers ∂/∂k. RK4 integration, with the variational equation dΦ/dt = A(x)Φ
// integrated in the SAME stages (integrating the state by RK4 and taking
// Φ ≈ I + Ah drops Φ to first order).
//
// Why test-only: the controller never re-propagates the ball — it trusts the
// vision prediction and only interpolates between its samples
// (docs/dynamic_catching/L0_core.md §1, §5.1). This model generates the
// reference trajectories that the sampler / reference-generator suites compare
// against. It lives under test/include, which is never installed, so no
// production target can include it (plan S1.6).
//
// Header-only, allocation-free, noexcept — not an RT requirement here (no RT
// gate applies to a fixture) but no reason to lose it either.
#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace rtc::catching::fixture {

using BallState = Eigen::Matrix<double, 7, 1>;  // [p(3) m; v(3) m/s; k 1/m]
using BallMat7 = Eigen::Matrix<double, 7, 7>;

struct BallModel {
  Eigen::Vector3d g{0.0, 0.0, -9.81};  // world [m/s²]
  // Lower bound on ‖v‖ used ONLY in the vvᵀ/‖v‖ term of the Jacobian, to keep
  // it finite at rest. ∂v̇/∂k = −‖v‖v deliberately does not use it (below).
  double v_eps{1e-3};  // [m/s]
};

/// Continuous-time vector field f(x). k is a constant state (k̇ = 0).
[[nodiscard]] inline BallState F(const BallModel& m, const BallState& x) noexcept {
  const Eigen::Vector3d v = x.segment<3>(3);
  BallState dx;
  dx.segment<3>(0) = v;
  dx.segment<3>(3) = m.g - x(6) * v.norm() * v;
  dx(6) = 0.0;
  return dx;
}

/// A = ∂f/∂x.
///
/// The v_eps floor applies to the vvᵀ/‖v‖ term only. ∂v̇/∂k = −‖v‖v uses the
/// true ‖v‖: flooring it there too would make f() and A two different models
/// below v_eps, and Φ would be wrong by a factor v_eps/‖v‖ (reference C8).
[[nodiscard]] inline BallMat7 Jacobian(const BallModel& m, const BallState& x) noexcept {
  const Eigen::Vector3d v = x.segment<3>(3);
  const double v_true = v.norm();
  const double vn = std::max(v_true, m.v_eps);
  const double k = x(6);
  BallMat7 A = BallMat7::Zero();
  A.block<3, 3>(0, 3).setIdentity();
  A.block<3, 3>(3, 3) = -k * (v_true * Eigen::Matrix3d::Identity() + (v * v.transpose()) / vn);
  A.block<3, 1>(3, 6) = -v_true * v;
  return A;
}

/// One RK4 step, state only.
[[nodiscard]] inline BallState Rk4(const BallModel& m, const BallState& x, double h) noexcept {
  const BallState k1 = F(m, x);
  const BallState k2 = F(m, x + 0.5 * h * k1);
  const BallState k3 = F(m, x + 0.5 * h * k2);
  const BallState k4 = F(m, x + h * k3);
  return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

/// One RK4 step of the state and its state-transition matrix Φ.
inline void Rk4WithStm(const BallModel& m, BallState& x, BallMat7& Phi, double h) noexcept {
  const BallState k1 = F(m, x);
  const BallMat7 P1 = Jacobian(m, x) * Phi;
  const BallState x2 = x + 0.5 * h * k1;
  const BallState k2 = F(m, x2);
  const BallMat7 P2 = Jacobian(m, x2) * (Phi + 0.5 * h * P1);
  const BallState x3 = x + 0.5 * h * k2;
  const BallState k3 = F(m, x3);
  const BallMat7 P3 = Jacobian(m, x3) * (Phi + 0.5 * h * P2);
  const BallState x4 = x + h * k3;
  const BallState k4 = F(m, x4);
  const BallMat7 P4 = Jacobian(m, x4) * (Phi + h * P3);
  x += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  Phi += (h / 6.0) * (P1 + 2.0 * P2 + 2.0 * P3 + P4);
}

/// Result of Propagate(). `steps == max_steps` does NOT mean the cap was hit —
/// an exact division lands there too — so callers read `truncated` (C11).
struct PropagateResult {
  int steps{0};
  bool truncated{false};  // the step cap forced h > h_max
  double h_used{0.0};     // [s]
};

/// Integrate x over [0, T] in equal sub-steps of at most h_max, capped at
/// max_steps so the worst-case cost is bounded.
inline PropagateResult Propagate(const BallModel& m, BallState& x, double T, double h_max,
                                 int max_steps) noexcept {
  if (!(T > 0.0) || !(h_max > 0.0) || max_steps <= 0)
    return {};
  int n = static_cast<int>(std::ceil(T / h_max));
  const bool truncated = (n > max_steps);
  if (truncated)
    n = max_steps;
  const double h = T / n;
  for (int i = 0; i < n; ++i)
    x = Rk4(m, x, h);
  return {n, truncated, h};
}

}  // namespace rtc::catching::fixture
