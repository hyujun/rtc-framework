// ── SE(3) math core (Eigen-only, RT-safe) ───────────────────────────────────
// Twist ordering is [linear, angular] (ρ on top, φ below) — identical to
// pinocchio::Motion, so log6/exp6/adjoint/Jlog6 cross-validate against
// Pinocchio with no reordering. (Modern Robotics / Featherstone use the
// opposite [angular, linear] order — do NOT mix.)
//
// References: Murray–Li–Sastry 1994; Lynch & Park 2017 §3.3, §11.3;
// Barfoot 2017 (SE(3) Jacobian Q-block); Solà et al. arXiv:1812.01537.
#pragma once

#include "rtc_urdf_bridge/se3/so3.hpp"

namespace rtc_urdf_bridge::se3 {

// ── log6 / exp6 ──────────────────────────────────────────────────────────────
//
// SE(3) exp:  exp6([ρ;φ]) = (exp3(φ), J_l(φ)·ρ).   [linear, angular] ordering.
// SE(3) log:  log6(T)     = [ J_l(φ)⁻¹·p ; φ ],  φ = log3(R).
// The translation uses the SAME SO(3) left Jacobian J_l that maps the algebra
// translation ρ to the group translation p — this is what makes exp6∘log6 = id.
[[nodiscard]] inline Vec6 log6(const Iso3& T) noexcept {
  const Vec3 phi = log3(T.linear());
  Vec6 xi;
  xi.head<3>() = leftJacobianInverse(phi) * T.translation();
  xi.tail<3>() = phi;
  return xi;
}

[[nodiscard]] inline Iso3 exp6(const Vec6& xi) noexcept {
  const Vec3 rho = xi.head<3>();
  const Vec3 phi = xi.tail<3>();
  Iso3 T = Iso3::Identity();
  T.linear() = exp3(phi);
  T.translation() = leftJacobian(phi) * rho;
  return T;
}

// ── adjoint ──────────────────────────────────────────────────────────────────
//
// Ad_T transports a twist between frames. For [linear, angular] ordering and
// T = (R, p):
//   Ad_T [v; ω] = [ R·v + p×(R·ω) ; R·ω ]  ⇒  Ad_T = [[R, [p]×R], [0, R]].
// Derivation: the linear velocity of a frame-attached point picks up the lever
// arm p crossed into the transported angular velocity R·ω — hence the [p]×R
// coupling in the top-right block.
[[nodiscard]] inline Mat6 adjoint(const Iso3& T) noexcept {
  const Mat3 R = T.linear();
  Mat6 Ad;
  Ad.setZero();
  Ad.block<3, 3>(0, 0) = R;
  Ad.block<3, 3>(0, 3) = hat(T.translation()) * R;
  Ad.block<3, 3>(3, 3) = R;
  return Ad;
}

// ── SE(3) coupling block Q (Barfoot 2017, eq. for the left Jacobian) ─────────
//
// J_l^{SE3}(ξ) = [[J_l(φ), Q(ρ,φ)], [0, J_l(φ)]].  A = [φ]×, B = [ρ]×.
namespace detail {
[[nodiscard]] inline Mat3 couplingQ(const Vec3& rho, const Vec3& phi) noexcept {
  const Mat3 A = hat(phi);
  const Mat3 B = hat(rho);
  const double t2 = phi.squaredNorm();
  const double theta = std::sqrt(t2);
  double b1;  // (θ−sinθ)/θ³
  double b2;  // −(1−θ²/2−cosθ)/θ⁴
  double b3;  // −½·((1−θ²/2−cosθ)/θ⁴ − 3(θ−sinθ−θ³/6)/θ⁵)
  if (theta < kSmallAngle) {
    b1 = 1.0 / 6.0 - t2 / 120.0;
    b2 = 1.0 / 24.0 - t2 / 720.0;
    b3 = 1.0 / 120.0 - t2 / 2520.0;
  } else {
    const double s = std::sin(theta);
    const double c = std::cos(theta);
    const double t3 = t2 * theta;
    const double t4 = t2 * t2;
    const double t5 = t4 * theta;
    const double k2 = (1.0 - t2 / 2.0 - c) / t4;  // (1−θ²/2−cosθ)/θ⁴
    b1 = (theta - s) / t3;
    b2 = -k2;
    b3 = -0.5 * (k2 - 3.0 * (theta - s - t3 / 6.0) / t5);
  }
  const Mat3 AB = A * B;
  const Mat3 BA = B * A;
  const Mat3 ABA = AB * A;
  return 0.5 * B + b1 * (AB + BA + ABA) + b2 * (A * AB + BA * A - 3.0 * ABA) +
         b3 * (ABA * A + A * ABA);
}
}  // namespace detail

// ── Jlog6: right (local) perturbation Jacobian of log6 ───────────────────────
//
// Right convention (matches Pinocchio): log6(F·exp6(δ)) ≈ log6(F) + Jlog6(F)·δ.
// This is the SE(3) inverse right Jacobian J_r⁻¹(ξ) = J_l^{SE3}⁻¹(−ξ),
// ξ = log6(F). Block-upper-triangular in [linear, angular] ordering:
//   J_l^{SE3}⁻¹ = [[J_l⁻¹, −J_l⁻¹ Q J_l⁻¹], [0, J_l⁻¹]],  evaluated at −ξ.
// The diagonal block equals Jlog3(R) = J_l⁻¹(−φ) — consistent with so3.hpp.
[[nodiscard]] inline Mat6 Jlog6(const Iso3& F) noexcept {
  const Vec6 xi = log6(F);
  const Vec3 nrho = -xi.head<3>();
  const Vec3 nphi = -xi.tail<3>();
  const Mat3 Jli = leftJacobianInverse(nphi);  // = Jlog3(R)
  const Mat3 Q = detail::couplingQ(nrho, nphi);
  Mat6 J;
  J.setZero();
  J.block<3, 3>(0, 0) = Jli;
  J.block<3, 3>(3, 3) = Jli;
  J.block<3, 3>(0, 3) = -Jli * Q * Jli;
  return J;
}

// ── Jlog6_numerical: central-difference reference (validation / fallback) ────
//
// Column j = ∂ log6(F·exp6(ε·e_j)) / ∂ε via central difference (right
// perturbation, matching Jlog6's convention). NOT for the RT path — it calls
// exp6/log6 12× — but it is the ground-truth oracle the analytic Jlog6 is
// tested against (and the path the prompt sanctions when the closed form is
// hard to trust).
[[nodiscard]] inline Mat6 Jlog6_numerical(const Iso3& F, double eps = 1e-6) noexcept {
  Mat6 J;
  for (int j = 0; j < 6; ++j) {
    Vec6 dp = Vec6::Zero();
    Vec6 dm = Vec6::Zero();
    dp(j) = eps;
    dm(j) = -eps;
    const Vec6 lp = log6(F * exp6(dp));
    const Vec6 lm = log6(F * exp6(dm));
    J.col(j) = (lp - lm) / (2.0 * eps);
  }
  return J;
}

}  // namespace rtc_urdf_bridge::se3
