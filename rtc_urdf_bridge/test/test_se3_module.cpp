// ── Unit tests for the SE(3) pose/twist-error module ─────────────────────────
// Cross-validates the Eigen-only numeric core against Pinocchio (log6/Jlog6/
// log3/Jlog3) and verifies the velocity-error trivialization via finite
// differences. Fixed RNG seed for reproducibility.
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include "rtc_urdf_bridge/se3/pose_error.hpp"
#include "rtc_urdf_bridge/se3/se3.hpp"
#include "rtc_urdf_bridge/se3/so3.hpp"
#include "rtc_urdf_bridge/se3/velocity_error.hpp"

namespace se3 = rtc_urdf_bridge::se3;
using se3::ErrorType;
using se3::Iso3;
using se3::Mat3;
using se3::Mat6;
using se3::Vec3;
using se3::Vec6;

namespace {

constexpr int kNumSamples = 1000;
constexpr double kTight = 1e-10;

// Deterministic RNG.
std::mt19937 MakeRng() {
  return std::mt19937(0xC0FFEEu);
}

Mat3 RandomRotation(std::mt19937& rng) {
  std::uniform_real_distribution<double> u(0.0, 1.0);
  // Shoemake uniform random quaternion.
  const double u1 = u(rng), u2 = u(rng), u3 = u(rng);
  const double s1 = std::sqrt(1.0 - u1), s2 = std::sqrt(u1);
  Eigen::Quaterniond q(s2 * std::cos(2.0 * M_PI * u3), s1 * std::sin(2.0 * M_PI * u2),
                       s1 * std::cos(2.0 * M_PI * u2), s2 * std::sin(2.0 * M_PI * u3));
  q.normalize();
  return q.toRotationMatrix();
}

Iso3 RandomPose(std::mt19937& rng) {
  std::uniform_real_distribution<double> t(-1.0, 1.0);
  Iso3 T = Iso3::Identity();
  T.linear() = RandomRotation(rng);
  T.translation() = Vec3(t(rng), t(rng), t(rng));
  return T;
}

pinocchio::SE3 ToPin(const Iso3& T) {
  return pinocchio::SE3(T.linear(), T.translation());
}

// Rotation of a given angle about a random axis.
Mat3 RotationOfAngle(std::mt19937& rng, double angle) {
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  Vec3 axis(u(rng), u(rng), u(rng));
  if (axis.norm() < 1e-9)
    axis = Vec3::UnitX();
  axis.normalize();
  return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

}  // namespace

// ── Test 1: exp∘log identities ───────────────────────────────────────────────
TEST(So3Core, ExpLogIdentity) {
  auto rng = MakeRng();
  for (int i = 0; i < kNumSamples; ++i) {
    const Mat3 R = RandomRotation(rng);
    EXPECT_LT((se3::exp3(se3::log3(R)) - R).norm(), kTight) << "exp3∘log3 sample " << i;

    const Iso3 T = RandomPose(rng);
    const Iso3 T2 = se3::exp6(se3::log6(T));
    EXPECT_LT((T2.linear() - T.linear()).norm(), kTight) << "exp6∘log6 R sample " << i;
    EXPECT_LT((T2.translation() - T.translation()).norm(), kTight) << "exp6∘log6 p sample " << i;
  }
}

// ── Test 1b: cross-check log6 / log3 against Pinocchio ────────────────────────
TEST(So3Core, PinocchioLogMatch) {
  auto rng = MakeRng();
  for (int i = 0; i < kNumSamples; ++i) {
    const Iso3 T = RandomPose(rng);
    const Vec3 phi_pin = pinocchio::log3(T.linear());
    EXPECT_LT((se3::log3(T.linear()) - phi_pin).norm(), kTight) << "log3 vs pin sample " << i;

    const Vec6 xi_pin = pinocchio::log6(ToPin(T)).toVector();
    EXPECT_LT((se3::log6(T) - xi_pin).norm(), kTight) << "log6 vs pin sample " << i;
  }
}

// ── Test 2: θ→0 branch continuity ────────────────────────────────────────────
TEST(So3Core, SmallAngleContinuity) {
  auto rng = MakeRng();
  const double theta = 1e-8;
  for (int i = 0; i < 100; ++i) {
    const Mat3 Rm = RotationOfAngle(rng, theta * (1.0 - 1e-3));
    const Mat3 Rp = RotationOfAngle(rng, theta * (1.0 + 1e-3));
    EXPECT_TRUE(se3::log3(Rm).allFinite());
    EXPECT_TRUE(se3::log3(Rp).allFinite());
    const Vec3 phi(theta, 0.0, 0.0);
    // J_l⁻¹ continuous across the Taylor/closed-form crossover.
    EXPECT_LT((se3::leftJacobianInverse(phi * (1.0 - 1e-3)) -
               se3::leftJacobianInverse(phi * (1.0 + 1e-3)))
                  .norm(),
              1e-6);
  }
}

// ── Test 3: θ→π robustness ───────────────────────────────────────────────────
TEST(So3Core, NearPiRobust) {
  auto rng = MakeRng();
  std::uniform_real_distribution<double> d(0.0, 1e-3);
  for (int i = 0; i < kNumSamples; ++i) {
    const double theta = M_PI - d(rng);
    const Mat3 R = RotationOfAngle(rng, theta);
    const Vec3 phi = se3::log3(R);
    EXPECT_TRUE(phi.allFinite()) << "log3 NaN near π sample " << i;
    EXPECT_NEAR(phi.norm(), theta, 1e-6) << "‖φ‖≈θ sample " << i;
    EXPECT_LT((se3::exp3(phi) - R).norm(), 1e-8) << "exp3∘log3 near π sample " << i;
  }
}

// ── Test 6: Jlog3 / Jlog6 vs central diff + Pinocchio ────────────────────────
TEST(So3Core, JacobianMatch) {
  auto rng = MakeRng();
  const double eps = 1e-6;
  for (int i = 0; i < kNumSamples; ++i) {
    const Iso3 T = RandomPose(rng);

    // Jlog3 vs Pinocchio.
    Mat3 Jlog3_pin;
    pinocchio::Jlog3(se3::log3(T.linear()).norm(), se3::log3(T.linear()), Jlog3_pin);
    EXPECT_LT((se3::Jlog3(T.linear()) - Jlog3_pin).norm(), 1e-9) << "Jlog3 vs pin sample " << i;

    // Jlog6 analytic vs central diff.
    const Mat6 Jn = se3::Jlog6_numerical(T, eps);
    EXPECT_LT((se3::Jlog6(T) - Jn).norm(), 1e-4) << "Jlog6 vs num sample " << i;

    // Jlog6 analytic vs Pinocchio.
    Mat6 Jlog6_pin;
    pinocchio::Jlog6(ToPin(T), Jlog6_pin);
    EXPECT_LT((se3::Jlog6(T) - Jlog6_pin).norm(), 1e-9) << "Jlog6 vs pin sample " << i;
  }
}

// ── Test 8: J(ξ)ξ = ξ identity ───────────────────────────────────────────────
TEST(So3Core, JacobianFixedDirection) {
  auto rng = MakeRng();
  for (int i = 0; i < kNumSamples; ++i) {
    const Iso3 F = RandomPose(rng);
    const Vec6 xi = se3::log6(F);
    EXPECT_LT((se3::Jlog6(F) * xi - xi).norm(), 1e-8) << "Jlog6(F)·log6(F)=log6(F) sample " << i;
    const Mat3 R = F.linear();
    const Vec3 phi = se3::log3(R);
    EXPECT_LT((se3::Jlog3(R) * phi - phi).norm(), 1e-8) << "Jlog3·φ=φ sample " << i;
  }
}

namespace {
Vec6 RandomTwist(std::mt19937& rng, double scale = 1.0) {
  std::uniform_real_distribution<double> u(-scale, scale);
  Vec6 nu;
  for (int k = 0; k < 6; ++k)
    nu(k) = u(rng);
  return nu;
}
}  // namespace

// ── Test 4: pure-rotation rotation-error scale (θ / sinθ / 2sin(θ/2)) ─────────
TEST(PoseError, RotationScale) {
  auto rng = MakeRng();
  for (double theta : {30.0, 90.0, 150.0, 179.0}) {
    const double th = theta * M_PI / 180.0;
    const Iso3 T = Iso3::Identity();
    Iso3 T_d = Iso3::Identity();
    T_d.linear() = RotationOfAngle(rng, th);
    EXPECT_NEAR(se3::computePoseError(T, T_d, ErrorType::SplitWorld).tail<3>().norm(), th, 1e-9);
    EXPECT_NEAR(se3::computePoseError(T, T_d, ErrorType::SplitLee).tail<3>().norm(), std::sin(th),
                1e-9);
    EXPECT_NEAR(se3::computePoseError(T, T_d, ErrorType::SplitQuat).tail<3>().norm(),
                2.0 * std::sin(th / 2.0), 1e-9);
    EXPECT_NEAR(se3::computePoseError(T, T_d, ErrorType::SplitBodyRot).tail<3>().norm(), th, 1e-9);
  }
}

// ── Test 5: SpatialLog6 = adjoint(T)·BodyLog6 ────────────────────────────────
TEST(PoseError, SpatialIsAdjointOfBody) {
  auto rng = MakeRng();
  for (int i = 0; i < kNumSamples; ++i) {
    const Iso3 T = RandomPose(rng);
    const Iso3 T_d = RandomPose(rng);
    const Vec6 e_body = se3::computePoseError(T, T_d, ErrorType::BodyLog6);
    const Vec6 e_spatial = se3::computePoseError(T, T_d, ErrorType::SpatialLog6);
    EXPECT_LT((e_spatial - se3::adjoint(T) * e_body).norm(), 1e-9) << "sample " << i;
  }
}

// ── Test 7: exactPoseErrorRate vs central finite difference (ALL types) ★ ────
TEST(VelocityError, ExactRateMatchesFiniteDiff) {
  auto rng = MakeRng();
  const double dt = 1e-6;
  const ErrorType types[] = {ErrorType::SplitWorld, ErrorType::BodyLog6,  ErrorType::SpatialLog6,
                             ErrorType::SplitLee,   ErrorType::SplitQuat, ErrorType::SplitBodyRot};
  for (int i = 0; i < 300; ++i) {
    // Moderate rotations (≤2.5 rad) keep both poses away from the log3 cusp at
    // π where the finite difference of log degrades — the rate formulas
    // themselves are exact, this only protects the numerical reference.
    Iso3 T = RandomPose(rng);
    Iso3 T_d = RandomPose(rng);
    T.linear() = RotationOfAngle(rng, std::uniform_real_distribution<double>(0.1, 2.5)(rng));
    T_d.linear() = RotationOfAngle(rng, std::uniform_real_distribution<double>(0.1, 2.5)(rng));
    const Vec6 nu = RandomTwist(rng);    // LOCAL (body) twist
    const Vec6 nu_d = RandomTwist(rng);  // LOCAL (body) twist
    for (ErrorType type : types) {
      const Iso3 Tp = T * se3::exp6(nu * dt);
      const Iso3 Tm = T * se3::exp6(nu * -dt);
      const Iso3 Tdp = T_d * se3::exp6(nu_d * dt);
      const Iso3 Tdm = T_d * se3::exp6(nu_d * -dt);
      const Vec6 fd =
          (se3::computePoseError(Tp, Tdp, type) - se3::computePoseError(Tm, Tdm, type)) /
          (2.0 * dt);
      const Vec6 exact = se3::exactPoseErrorRate(T, T_d, nu, nu_d, type);
      EXPECT_LT((fd - exact).norm(), 1e-4 * (1.0 + exact.norm()))
          << "type " << static_cast<int>(type) << " sample " << i;
    }
  }
}

// ── Test 9: scalar gain ⇒ exact exponential decay (BodyLog6, MR §11.3) ★ ─────
TEST(VelocityError, ScalarGainExponentialDecay) {
  auto rng = MakeRng();
  const double k = 2.0;
  const double dt = 1e-3;
  const int steps = 2000;  // 2 s
  for (int trial = 0; trial < 20; ++trial) {
    Iso3 T = RandomPose(rng);
    Iso3 T_d = RandomPose(rng);
    T_d.linear() =
        RotationOfAngle(rng, std::uniform_real_distribution<double>(0.1, 2.0)(rng)) * T.linear();
    const Vec6 nu_d = RandomTwist(rng, 0.3);  // time-varying desired (body) twist
    const double e0 = se3::computePoseError(T, T_d, ErrorType::BodyLog6).norm();
    for (int s = 0; s < steps; ++s) {
      const Iso3 F = T.inverse() * T_d;
      const Vec6 e = se3::log6(F);
      // MR law: ν = Ad_F·ν_d + k·e  ⇒  ė = −k·e exactly (scalar gain).
      const Vec6 nu = se3::adjoint(F) * nu_d + k * e;
      T = T * se3::exp6(nu * dt);
      T_d = T_d * se3::exp6(nu_d * dt);
    }
    const double eT = se3::computePoseError(T, T_d, ErrorType::BodyLog6).norm();
    const double expected = e0 * std::exp(-k * dt * steps);
    EXPECT_LT(std::abs(eT - expected), 1e-2 * e0 + 1e-9)
        << "trial " << trial << " eT=" << eT << " expected=" << expected;
  }
}

// ── Test 10: first-order agreement of all definitions as ‖e‖→0 ───────────────
TEST(PoseError, SmallErrorFirstOrderAgreement) {
  auto rng = MakeRng();
  const double s = 1e-6;
  for (int i = 0; i < kNumSamples; ++i) {
    const Iso3 T = RandomPose(rng);
    Vec6 xi0 = RandomTwist(rng);
    xi0.normalize();
    const Iso3 T_d = T * se3::exp6(s * xi0);
    const Mat3 R = T.linear();
    const Vec6 e_body = se3::computePoseError(T, T_d, ErrorType::BodyLog6);
    const Vec6 e_sw = se3::computePoseError(T, T_d, ErrorType::SplitWorld);
    const Vec6 e_lee = se3::computePoseError(T, T_d, ErrorType::SplitLee);
    const Vec6 e_quat = se3::computePoseError(T, T_d, ErrorType::SplitQuat);
    // BodyLog6 ≈ s·xi0 to first order.
    EXPECT_LT((e_body - s * xi0).norm() / s, 1e-4) << "body sample " << i;
    // SplitWorld ≈ blockdiag(R,R)·e_body (axes rotated to world).
    Mat6 BR;
    BR.setZero();
    BR.block<3, 3>(0, 0) = R;
    BR.block<3, 3>(3, 3) = R;
    EXPECT_LT((e_sw - BR * e_body).norm() / s, 1e-4) << "split-world sample " << i;
    // Lee/Quat agree with SplitWorld to first order (scales differ at O(θ³)).
    EXPECT_LT((e_lee - e_sw).norm() / s, 1e-4) << "lee sample " << i;
    EXPECT_LT((e_quat - e_sw).norm() / s, 1e-4) << "quat sample " << i;
  }
}
