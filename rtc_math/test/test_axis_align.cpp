// Axis-alignment error tests (dynamic_catching S2.1, gate G4-D —
// docs/dynamic_catching/L4_reference.md §4.5, §9).
//
//   ExpIdentity            exp([e_a]×) z = a_d residual < 1e-12 (regular region)
//   OmegaGridContinuity    1° grid ‖ω_ref‖ step < 1.5·K_a·π/180 over 0–180°
//   JacobianFiniteDiff     J_a vs central difference < 1e-5 over 1–170°
//   Deadband*/Capped*      every output finite in and around both deadbands
//   InvalidInput*          non-finite / non-unit / bad bounds → zero + invalid
//   AllocationFree         ScopedNoMalloc 0 violations, noexcept
//
// The oracles do not use the module under test: rotations are applied with
// Eigen::AngleAxisd, not se3::exp3.

// Must precede every Eigen include (the header enforces it with #error).
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_math/se3/axis_align.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <random>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

namespace se3 = rtc::math::se3;
using se3::AxisAlignRegion;
using se3::Mat3;
using se3::Vec3;

constexpr double kPi = std::numbers::pi;
constexpr double kDeg = kPi / 180.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// G4-D gains from the L4 §4.5 comparison table.
constexpr double kAxisGain = 8.0;
constexpr double kOmegaMax = 6.0;

Vec3 RandomUnit(std::mt19937& rng) {
  std::normal_distribution<double> nd(0.0, 1.0);
  Vec3 v(nd(rng), nd(rng), nd(rng));
  return v.normalized();
}

// Unit vector ⟂ z, random direction.
Vec3 RandomPerpendicular(const Vec3& z, std::mt19937& rng) {
  Vec3 r = RandomUnit(rng);
  r -= r.dot(z) * z;
  return r.normalized();
}

// Target at a given angle from z: a_d = cos(θ)·z + sin(θ)·p (p ⟂ z, unit).
Vec3 AxisAtAngle(const Vec3& z, const Vec3& p, double theta) {
  return (std::cos(theta) * z + std::sin(theta) * p).normalized();
}

// Target whose cross-product norm with z is exactly-ish `n`, on either side.
Vec3 AxisWithSin(const Vec3& z, const Vec3& p, double n, bool antiparallel) {
  const double c = std::sqrt(1.0 - n * n);
  return ((antiparallel ? -c : c) * z + n * p).normalized();
}

Vec3 RotateBy(const Vec3& omega, double h, const Vec3& v) {
  const double w = omega.norm();
  if (w == 0.0) {
    return v;
  }
  return Eigen::AngleAxisd(h * w, omega / w) * v;
}

// ── G4-D: exact rotation ────────────────────────────────────────────────────
TEST(AxisAlign, ExpIdentity) {
  std::mt19937 rng(20260919);
  double worst = 0.0;
  for (int trial = 0; trial < 20; ++trial) {
    const Vec3 z = RandomUnit(rng);
    const Vec3 p = RandomPerpendicular(z, rng);
    for (double deg = 0.5; deg < 180.0; deg += 0.5) {
      const Vec3 a_d = AxisAtAngle(z, p, deg * kDeg);
      const se3::AxisAlignErrorResult r = se3::AxisAlignError(z, a_d);
      ASSERT_EQ(r.region, AxisAlignRegion::kRegular) << deg;
      const double theta = r.error.norm();
      EXPECT_NEAR(theta, deg * kDeg, 1e-12) << deg;
      EXPECT_NEAR(r.error.dot(z), 0.0, 1e-12) << "e_a must be ⟂ z at " << deg;
      const Vec3 rotated = Eigen::AngleAxisd(theta, r.error / theta) * z;
      worst = std::max(worst, (rotated - a_d).norm());
    }
  }
  EXPECT_LT(worst, 1e-12);
}

// Inside the deadbands the identity holds only up to the band width.
TEST(AxisAlign, DeadbandErrorValues) {
  std::mt19937 rng(7);
  const Vec3 z = RandomUnit(rng);
  const Vec3 p = RandomPerpendicular(z, rng);

  const se3::AxisAlignErrorResult same = se3::AxisAlignError(z, z);
  EXPECT_EQ(same.region, AxisAlignRegion::kAlignedDeadband);
  EXPECT_TRUE(same.error.isZero(0.0));

  const se3::AxisAlignErrorResult opposite = se3::AxisAlignError(z, -z);
  EXPECT_EQ(opposite.region, AxisAlignRegion::kAntiparallelDeadband);
  EXPECT_NEAR(opposite.error.norm(), kPi, 1e-15);
  EXPECT_NEAR(opposite.error.dot(z), 0.0, 1e-15);
  const Vec3 flipped = Eigen::AngleAxisd(kPi, opposite.error / kPi) * z;
  EXPECT_LT((flipped + z).norm(), 1e-15);

  // Just inside the antiparallel band the fixed axis still lands within ~sin_eps.
  const Vec3 a_near = AxisWithSin(z, p, 0.5 * se3::kAxisAlignSinEps, /*antiparallel=*/true);
  const se3::AxisAlignErrorResult near = se3::AxisAlignError(z, a_near);
  EXPECT_EQ(near.region, AxisAlignRegion::kAntiparallelDeadband);
  const Vec3 landed = Eigen::AngleAxisd(kPi, near.error / kPi) * z;
  EXPECT_LT((landed - a_near).norm(), 2.0 * se3::kAxisAlignSinEps);

  // Same z → same fallback axis (no flicker between calls).
  EXPECT_TRUE(near.error.isApprox(opposite.error, 0.0));
}

// ── G4-D: 1° grid continuity of ‖ω_ref‖ ────────────────────────────────────
TEST(AxisAlign, OmegaGridContinuity) {
  std::mt19937 rng(11);
  const double limit = 1.5 * kAxisGain * kDeg;
  for (int trial = 0; trial < 20; ++trial) {
    const Vec3 z = RandomUnit(rng);
    const Vec3 p = RandomPerpendicular(z, rng);
    double prev = 0.0;
    for (int deg = 0; deg <= 180; ++deg) {
      const se3::AxisAlignErrorResult e = se3::AxisAlignError(z, AxisAtAngle(z, p, deg * kDeg));
      ASSERT_TRUE(e.IsValid());
      const se3::AxisAlignOmegaResult w = se3::AxisAlignOmega(e.error, kAxisGain, kOmegaMax);
      ASSERT_TRUE(w.valid);
      const double norm = w.omega.norm();
      EXPECT_LE(norm, kOmegaMax + 1e-12);
      if (deg > 0) {
        EXPECT_LT(std::abs(norm - prev), limit) << "step into " << deg << "°";
      }
      prev = norm;
    }
    // The table's endpoint: saturated at 180° instead of collapsing (v0.1 sinθ form).
    EXPECT_NEAR(prev, kOmegaMax, 1e-12);
  }
}

// ── G4-D: Jacobian vs finite difference, 1–170° ─────────────────────────────
TEST(AxisAlign, JacobianFiniteDifference) {
  std::mt19937 rng(13);
  constexpr double kStep = 1e-6;
  double worst = 0.0;
  for (int trial = 0; trial < 10; ++trial) {
    const Vec3 z = RandomUnit(rng);
    const Vec3 p = RandomPerpendicular(z, rng);
    for (int deg = 1; deg <= 170; ++deg) {
      const Vec3 a_d = AxisAtAngle(z, p, deg * kDeg);
      const se3::AxisAlignJacobianResult jr = se3::AxisAlignJacobian(z, a_d);
      ASSERT_EQ(jr.region, AxisAlignRegion::kRegular) << deg;
      for (int k = 0; k < 3; ++k) {
        const Vec3 omega = Vec3::Unit(k);
        const Vec3 e_plus = se3::AxisAlignError(RotateBy(omega, kStep, z), a_d).error;
        const Vec3 e_minus = se3::AxisAlignError(RotateBy(omega, -kStep, z), a_d).error;
        const Vec3 fd = (e_plus - e_minus) / (2.0 * kStep);
        worst = std::max(worst, (jr.jacobian * omega - fd).cwiseAbs().maxCoeff());
      }
    }
  }
  EXPECT_LT(worst, 1e-5);
}

// The small-angle series branch agrees with the closed form at its threshold
// and with a finite difference below it.
TEST(AxisAlign, JacobianSeriesBranchMatchesClosedForm) {
  std::mt19937 rng(17);
  const Vec3 z = RandomUnit(rng);
  const Vec3 p = RandomPerpendicular(z, rng);
  constexpr double kTinyEps = 1e-12;  // keep small angles out of the deadband
  constexpr double kCross = 1e-3;     // detail::kAxisAlignSeriesTheta
  const Mat3 below =
      se3::AxisAlignJacobian(z, AxisAtAngle(z, p, kCross * (1.0 - 1e-6)), kTinyEps, kTinyEps)
          .jacobian;
  const Mat3 above =
      se3::AxisAlignJacobian(z, AxisAtAngle(z, p, kCross * (1.0 + 1e-6)), kTinyEps, kTinyEps)
          .jacobian;
  EXPECT_LT((below - above).cwiseAbs().maxCoeff(), 1e-8);

  constexpr double kStep = 1e-8;
  for (const double theta : {1e-7, 1e-5, 5e-4}) {
    const Vec3 a_d = AxisAtAngle(z, p, theta);
    const se3::AxisAlignJacobianResult jr = se3::AxisAlignJacobian(z, a_d, kTinyEps, kTinyEps);
    ASSERT_EQ(jr.region, AxisAlignRegion::kRegular);
    for (int k = 0; k < 3; ++k) {
      const Vec3 omega = Vec3::Unit(k);
      const Vec3 fd = (se3::AxisAlignError(RotateBy(omega, kStep, z), a_d, kTinyEps).error -
                       se3::AxisAlignError(RotateBy(omega, -kStep, z), a_d, kTinyEps).error) /
                      (2.0 * kStep);
      EXPECT_LT((jr.jacobian * omega - fd).cwiseAbs().maxCoeff(), 1e-5) << theta;
    }
  }
}

// L4 §4.5 구현 주의 1: near θ = π the Jacobian must diverge, not fall back to the
// small-angle series value (≈2.645). With the cap pushed below the probe, the
// probe sees the real magnitude.
TEST(AxisAlign, JacobianNearPiIsNotTheSeriesValue) {
  std::mt19937 rng(19);
  const Vec3 z = RandomUnit(rng);
  const Vec3 p = RandomPerpendicular(z, rng);
  constexpr double kTiny = 1e-12;
  const Vec3 a_d = AxisWithSin(z, p, 1e-8, /*antiparallel=*/true);  // θ ≈ π − 1e-8
  const se3::AxisAlignJacobianResult jr = se3::AxisAlignJacobian(z, a_d, kTiny, kTiny);
  ASSERT_EQ(jr.region, AxisAlignRegion::kRegular);
  EXPECT_TRUE(jr.jacobian.allFinite());
  EXPECT_GT(jr.jacobian.norm(), 1e7);
}

// ── Finiteness in and around both deadbands (G4-D, S2.1) ────────────────────
TEST(AxisAlign, OutputsFiniteAroundDeadbands) {
  std::mt19937 rng(23);
  const double cap_bound = 4.0 * kPi / se3::kAxisAlignJacobianSinFloor;
  for (int trial = 0; trial < 10; ++trial) {
    const Vec3 z = RandomUnit(rng);
    const Vec3 p = RandomPerpendicular(z, rng);
    for (const bool antiparallel : {false, true}) {
      for (int k = 0; k <= 36; ++k) {
        const double n = std::pow(10.0, -0.5 * k);  // 1 … 1e-18
        const Vec3 a_d = AxisWithSin(z, p, std::min(n, 1.0), antiparallel);
        const se3::AxisAlignErrorResult e = se3::AxisAlignError(z, a_d);
        const se3::AxisAlignJacobianResult j = se3::AxisAlignJacobian(z, a_d);
        ASSERT_TRUE(e.IsValid());
        ASSERT_TRUE(j.IsValid());
        EXPECT_TRUE(e.error.allFinite()) << n;
        EXPECT_TRUE(j.jacobian.allFinite()) << n;
        EXPECT_LE(e.error.norm(), kPi + 1e-12);
        EXPECT_LE(j.jacobian.norm(), cap_bound) << n;
        const se3::AxisAlignOmegaResult w = se3::AxisAlignOmega(e.error, kAxisGain, kOmegaMax);
        EXPECT_TRUE(w.valid);
        EXPECT_TRUE(w.omega.allFinite());

        const bool in_band = e.region == AxisAlignRegion::kAlignedDeadband ||
                             e.region == AxisAlignRegion::kAntiparallelDeadband;
        EXPECT_EQ(j.region == AxisAlignRegion::kAlignedDeadband ||
                      j.region == AxisAlignRegion::kAntiparallelDeadband,
                  in_band)
            << "error and Jacobian must agree on the deadband at n=" << n;
        if (in_band) {
          EXPECT_TRUE(j.jacobian.isZero(0.0)) << "J_a must be 0 where e_a is constant";
        }
      }
    }
  }
}

TEST(AxisAlign, JacobianCappedRegionIsContinuousAtFloor) {
  std::mt19937 rng(29);
  const Vec3 z = RandomUnit(rng);
  const Vec3 p = RandomPerpendicular(z, rng);
  const double floor = se3::kAxisAlignJacobianSinFloor;

  const se3::AxisAlignJacobianResult inside =
      se3::AxisAlignJacobian(z, AxisWithSin(z, p, floor * (1.0 - 1e-9), true));
  const se3::AxisAlignJacobianResult outside =
      se3::AxisAlignJacobian(z, AxisWithSin(z, p, floor * (1.0 + 1e-9), true));
  EXPECT_EQ(inside.region, AxisAlignRegion::kJacobianCapped);
  EXPECT_EQ(outside.region, AxisAlignRegion::kRegular);
  EXPECT_LT((inside.jacobian - outside.jacobian).norm(), 1e-5 * outside.jacobian.norm());

  // The error itself stays exact in the capped region.
  const se3::AxisAlignErrorResult e = se3::AxisAlignError(z, AxisWithSin(z, p, 0.1 * floor, true));
  EXPECT_EQ(e.region, AxisAlignRegion::kRegular);

  // Aligned side (c > 0) is never capped.
  EXPECT_EQ(se3::AxisAlignJacobian(z, AxisWithSin(z, p, 0.1 * floor, false)).region,
            AxisAlignRegion::kRegular);
}

// ── Invalid input: zero outputs, invalid flag ───────────────────────────────
TEST(AxisAlign, InvalidInputsGiveZeroAndInvalid) {
  const Vec3 z = Vec3::UnitZ();
  const Vec3 good = Vec3::UnitX();
  const std::vector<Vec3> bad = {
      Vec3(kNaN, 0.0, 1.0), Vec3(0.0, kInf, 0.0), Vec3::Zero(), Vec3(0.0, 0.0, 1.01),  // non-unit
      Vec3(0.0, 0.0, 0.5),
  };
  for (const Vec3& v : bad) {
    for (int side = 0; side < 2; ++side) {
      const Vec3& zz = (side == 0) ? v : z;
      const Vec3& aa = (side == 0) ? good : v;
      const se3::AxisAlignErrorResult e = se3::AxisAlignError(zz, aa);
      const se3::AxisAlignJacobianResult j = se3::AxisAlignJacobian(zz, aa);
      EXPECT_FALSE(e.IsValid());
      EXPECT_FALSE(j.IsValid());
      EXPECT_TRUE(e.error.isZero(0.0));
      EXPECT_TRUE(j.jacobian.isZero(0.0));
    }
  }
  for (const double eps : {0.0, -1e-6, 1.0, 2.0, kNaN, kInf}) {
    EXPECT_FALSE(se3::AxisAlignError(z, good, eps).IsValid()) << eps;
    EXPECT_FALSE(se3::AxisAlignJacobian(z, good, eps).IsValid()) << eps;
    EXPECT_FALSE(se3::AxisAlignJacobian(z, good, 1e-6, eps).IsValid()) << eps;
  }
  // Floor below the deadband would leave a region the deadband claims.
  EXPECT_FALSE(se3::AxisAlignJacobian(z, good, 1e-3, 1e-4).IsValid());
  // Within the unit tolerance is accepted.
  EXPECT_TRUE(se3::AxisAlignError(Vec3(0.0, 0.0, 1.0 + 1e-9), good).IsValid());
}

TEST(AxisAlign, OmegaSaturationAndInvalidGains) {
  const Vec3 e(0.0, 1.0, 0.0);
  const se3::AxisAlignOmegaResult small = se3::AxisAlignOmega(e, 2.0, 6.0);
  EXPECT_TRUE(small.valid);
  EXPECT_FALSE(small.saturated);
  EXPECT_TRUE(small.omega.isApprox(Vec3(0.0, 2.0, 0.0)));

  const se3::AxisAlignOmegaResult big = se3::AxisAlignOmega(e, 8.0, 6.0);
  EXPECT_TRUE(big.valid);
  EXPECT_TRUE(big.saturated);
  EXPECT_NEAR(big.omega.norm(), 6.0, 1e-15);
  EXPECT_NEAR(big.omega.normalized().dot(e), 1.0, 1e-15) << "saturation keeps the direction";

  struct Bad {
    Vec3 error;
    double k;
    double w_max;
  };

  const std::vector<Bad> cases = {
      {Vec3(kNaN, 0.0, 0.0), 1.0, 1.0},
      {e, -1.0, 1.0},
      {e, kNaN, 1.0},
      {e, kInf, 1.0},
      {e, 1.0, 0.0},
      {e, 1.0, -1.0},
      {e, 1.0, kNaN},
      {e, 1.0, kInf},
      {Vec3(1e300, 1e300, 0.0), 1e300, 1.0},
  };
  for (const Bad& c : cases) {
    const se3::AxisAlignOmegaResult w = se3::AxisAlignOmega(c.error, c.k, c.w_max);
    EXPECT_FALSE(w.valid);
    EXPECT_TRUE(w.omega.isZero(0.0));
  }
}

// ── RT: noexcept, no Eigen heap allocation ──────────────────────────────────
static_assert(noexcept(se3::AxisAlignError(std::declval<const Vec3&>(),
                                           std::declval<const Vec3&>())));
static_assert(noexcept(se3::AxisAlignJacobian(std::declval<const Vec3&>(),
                                              std::declval<const Vec3&>())));
static_assert(noexcept(se3::AxisAlignOmega(std::declval<const Vec3&>(), 0.0, 0.0)));
static_assert(std::is_trivially_copyable_v<se3::AxisAlignRegion>);

TEST(AxisAlign, AllocationFree) {
  std::mt19937 rng(31);
  const Vec3 z = RandomUnit(rng);
  const Vec3 p = RandomPerpendicular(z, rng);
  const std::vector<Vec3> targets = {
      z,
      -z,
      AxisAtAngle(z, p, 0.3),
      AxisAtAngle(z, p, 3.0),
      AxisWithSin(z, p, 1e-4, true),
      Vec3(kNaN, 0.0, 0.0),
  };
  double sink = 0.0;
  rtc::testing::ScopedNoMalloc gate;
  for (const Vec3& a_d : targets) {
    const se3::AxisAlignErrorResult e = se3::AxisAlignError(z, a_d);
    const se3::AxisAlignJacobianResult j = se3::AxisAlignJacobian(z, a_d);
    const se3::AxisAlignOmegaResult w = se3::AxisAlignOmega(e.error, kAxisGain, kOmegaMax);
    sink += e.error.sum() + j.jacobian.sum() + w.omega.sum();
  }
  EXPECT_EQ(gate.violations(), 0U);
  EXPECT_TRUE(std::isfinite(sink));
}

}  // namespace
