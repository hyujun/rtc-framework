// ── Unit tests for the shared SE(3) task-error helper (rtc::tsid) ────────────
// Locks the LWA frame convention of ComputeTaskPoseError / ComputeTaskVelocityError
// (U1 unification). The numeric core (rtc_math) is cross-validated separately in
// rtc_math/test/test_se3_module.cpp; here we pin the rtc_tsid frame contract.
#include "rtc_tsid/kinematics/se3_error.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace rtc::tsid {
namespace {

namespace se3 = rtc::math::se3;
using se3::Iso3;
using se3::Mat3;
using se3::Vec3;
using se3::Vec6;

pinocchio::SE3 MakeSE3(const Vec3& rpy, const Vec3& p) {
  const Mat3 R =
      (Eigen::AngleAxisd(rpy.z(), Vec3::UnitZ()) * Eigen::AngleAxisd(rpy.y(), Vec3::UnitY()) *
       Eigen::AngleAxisd(rpy.x(), Vec3::UnitX()))
          .toRotationMatrix();
  return pinocchio::SE3(R, p);
}

constexpr double kTight = 1e-10;

// ── Identity: zero pose error when T == T_d ──────────────────────────────────
TEST(TaskSe3Error, IdentityPoseZero) {
  const pinocchio::SE3 T = MakeSE3(Vec3(0.3, -0.2, 0.5), Vec3(0.1, 0.2, 0.3));
  EXPECT_LT(ComputeTaskPoseError(T, T).norm(), kTight);
}

// ── Identity: zero velocity error when T==T_d and ν==ν_d ─────────────────────
TEST(TaskSe3Error, IdentityVelocityZero) {
  const pinocchio::SE3 T = MakeSE3(Vec3(0.3, -0.2, 0.5), Vec3(0.1, 0.2, 0.3));
  Vec6 nu;
  nu << 0.5, -0.3, 0.2, 0.1, -0.2, 0.4;
  EXPECT_LT(ComputeTaskVelocityError(T, T, nu, nu).norm(), kTight);
}

// ── twistWorldToLocal ∘ twistLocalToWorld == I (frame round-trip) ────────────
TEST(TaskSe3Error, TwistTransportRoundTrip) {
  const Iso3 T = se3::toIso3(MakeSE3(Vec3(0.4, 0.1, -0.6), Vec3::Zero()));
  Vec6 x;
  x << 1.0, -2.0, 3.0, -0.5, 0.7, 0.2;
  const Vec6 back = se3::twistWorldToLocal(T, se3::twistLocalToWorld(T, x));
  EXPECT_LT((back - x).norm(), kTight);
}

// ── Pure translation: no screw coupling, error == [p_d−p ; 0] in LWA ─────────
TEST(TaskSe3Error, PoseErrorPureTranslation) {
  const Vec3 rpy(0.4, -0.3, 0.7);  // same orientation for T and T_d
  const pinocchio::SE3 T = MakeSE3(rpy, Vec3(0.1, 0.2, 0.3));
  const pinocchio::SE3 T_d = MakeSE3(rpy, Vec3(0.15, 0.18, 0.42));
  const Vec6 e = ComputeTaskPoseError(T, T_d);
  // Linear part equals the plain world position difference; angular part zero.
  EXPECT_LT((e.head<3>() - (T_d.translation() - T.translation())).norm(), kTight);
  EXPECT_LT(e.tail<3>().norm(), kTight);
}

// ── Pure rotation: linear part ~0, angular magnitude == θ, world-aligned ─────
TEST(TaskSe3Error, PoseErrorPureRotationMagnitude) {
  const double theta = 0.6;
  const Vec3 p(0.2, -0.1, 0.4);
  const pinocchio::SE3 T = MakeSE3(Vec3::Zero(), p);
  // Rotate about a fixed axis; same translation → pure rotation error.
  const Mat3 R_d = Eigen::AngleAxisd(theta, Vec3(1, 1, 1).normalized()).toRotationMatrix();
  const pinocchio::SE3 T_d(R_d, p);
  const Vec6 e = ComputeTaskPoseError(T, T_d);
  EXPECT_LT(e.head<3>().norm(), kTight) << "pure rotation → zero linear error";
  EXPECT_NEAR(e.tail<3>().norm(), theta, 1e-9) << "angular magnitude == θ";
}

// ── Velocity error reduces to the twist difference at small pose error ───────
TEST(TaskSe3Error, VelocitySmallErrorReducesToTwistDiff) {
  const pinocchio::SE3 T = MakeSE3(Vec3(0.3, -0.2, 0.5), Vec3(0.1, 0.2, 0.3));
  // Desired pose a tiny screw away → Jlog6 ≈ I, Ad ≈ I.
  Vec6 dx;
  dx << 1e-4, -2e-4, 1.5e-4, 1e-4, -1e-4, 2e-4;
  const pinocchio::SE3 T_d = se3::toSE3(se3::toIso3(T) * se3::exp6(dx));
  Vec6 nu, nu_d;
  nu << 0.3, -0.1, 0.2, 0.05, -0.1, 0.08;
  nu_d << 0.1, 0.2, -0.1, -0.02, 0.03, 0.01;
  const Vec6 e_vel = ComputeTaskVelocityError(T, T_d, nu, nu_d);
  // First order: e_vel ≈ ν_d − ν (the quantity v_des − J·v it replaces).
  EXPECT_LT((e_vel - (nu_d - nu)).norm(), 1e-3);
}

// ── Frame consistency: e_vel == d/dt(e_pos) (exact when ω=0, no dropped term) ─
TEST(TaskSe3Error, VelocityMatchesFiniteDiffZeroAngular) {
  const pinocchio::SE3 T = MakeSE3(Vec3(0.5, -0.4, 0.3), Vec3(0.1, -0.2, 0.3));
  const pinocchio::SE3 T_d = MakeSE3(Vec3(0.2, 0.3, -0.5), Vec3(0.4, 0.1, 0.2));
  // Pure-linear LWA twists (zero angular) → R_T, R_Td constant → the U1 transport
  // drops no term, so the analytic e_vel matches the central finite difference.
  Vec6 nu, nu_d;
  nu << 0.4, -0.3, 0.2, 0.0, 0.0, 0.0;
  nu_d << -0.1, 0.2, 0.3, 0.0, 0.0, 0.0;
  const Vec6 nu_local = se3::twistWorldToLocal(se3::toIso3(T), nu);
  const Vec6 nu_d_local = se3::twistWorldToLocal(se3::toIso3(T_d), nu_d);

  const double dt = 1e-6;
  const Iso3 Tp = se3::toIso3(T) * se3::exp6(nu_local * dt);
  const Iso3 Tm = se3::toIso3(T) * se3::exp6(nu_local * -dt);
  const Iso3 Tdp = se3::toIso3(T_d) * se3::exp6(nu_d_local * dt);
  const Iso3 Tdm = se3::toIso3(T_d) * se3::exp6(nu_d_local * -dt);
  const Vec6 fd = (ComputeTaskPoseError(se3::toSE3(Tp), se3::toSE3(Tdp)) -
                   ComputeTaskPoseError(se3::toSE3(Tm), se3::toSE3(Tdm))) /
                  (2.0 * dt);

  const Vec6 e_vel = ComputeTaskVelocityError(T, T_d, nu, nu_d);
  EXPECT_LT((e_vel - fd).norm(), 1e-4) << "e_vel must equal d/dt(e_pos)";
}

}  // namespace
}  // namespace rtc::tsid
