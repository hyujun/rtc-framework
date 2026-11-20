// ── test_imu_rate_export_model.cpp ───────────────────────────────────────────
// The tick-mean IMU identities (imu_rate_export.hpp) without a model: on an
// exact constant-rate trajectory — R(t) = R0 Exp(ω t), constant site-frame
// specific force f, Δv = R0 Γ₁(ωΔt) f Δt + g Δt — the export returns ω and f.
// Γ₁ here is its power series, independent of the closed form under test.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/imu_rate_export.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace rtc {
namespace {

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;

Mat3 Mul(const Mat3& a, const Mat3& b) {
  Mat3 o{};
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j)
      for (std::size_t k = 0; k < 3; ++k)
        o[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
  return o;
}

Vec3 Mul(const Mat3& a, const Vec3& v) {
  Vec3 o{};
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t k = 0; k < 3; ++k)
      o[i] += a[i * 3 + k] * v[k];
  return o;
}

Mat3 Skew(const Vec3& p) {
  return {0.0, -p[2], p[1], p[2], 0.0, -p[0], -p[1], p[0], 0.0};
}

/// Σ_{n ≥ 0} φ^n / (n + shift)!  — shift 0 is Exp, shift 1 is Γ₁.
Mat3 Series(const Vec3& phi, int shift) {
  const Mat3 s = Skew(phi);
  Mat3 term{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  double fact = 1.0;
  for (int k = 2; k <= shift; ++k)
    fact *= k;
  Mat3 sum{};
  for (std::size_t n = 0; n < 40; ++n) {
    for (std::size_t i = 0; i < 9; ++i)
      sum[i] += term[i] / fact;
    term = Mul(term, s);
    fact *= static_cast<double>(n + 1) + static_cast<double>(shift);
  }
  return sum;
}

double Norm(const Vec3& v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

struct Case {
  ImuSiteSnapshot prev;
  ImuSiteSnapshot cur;
  Vec3 omega;
  Vec3 force;
};

constexpr double kG[3] = {0.0, 0.0, -9.81};

Case ConstantRate(const Vec3& omega, double dt) {
  Case c;
  c.omega = omega;
  c.force = {1.7, -0.4, 9.3};
  c.prev.rot = Series({0.4, -1.1, 0.7}, 0);
  c.prev.vel = {0.3, -0.2, 0.05};
  c.prev.time = 1.25;
  const Vec3 phi{omega[0] * dt, omega[1] * dt, omega[2] * dt};
  c.cur.rot = Mul(c.prev.rot, Series(phi, 0));
  const Vec3 dv = Mul(Mul(c.prev.rot, Series(phi, 1)), c.force);
  for (std::size_t a = 0; a < 3; ++a)
    c.cur.vel[a] = c.prev.vel[a] + (dv[a] + kG[a]) * dt;
  c.cur.time = c.prev.time + dt;
  return c;
}

TEST(ImuRateExportModel, ParsesTheTwoSpellingsOnly) {
  EXPECT_EQ(ParseImuRateExport("sample"), ImuRateExport::kSample);
  EXPECT_EQ(ParseImuRateExport("tick_mean"), ImuRateExport::kTickMean);
  for (const char* bad : {"", "Sample", "tick-mean", "mean", "tick_mean "})
    EXPECT_FALSE(ParseImuRateExport(bad).has_value()) << '"' << bad << '"';
  EXPECT_STREQ(ToString(ImuRateExport::kSample), "sample");
  EXPECT_STREQ(ToString(ImuRateExport::kTickMean), "tick_mean");
}

TEST(ImuRateExportModel, RotationLogInvertsExpFromTinyToNearPi) {
  const Vec3 axis{0.48, -0.6, 0.64};  // unit
  for (const double th : {0.0, 1e-12, 1e-6, 1e-3, 0.3, 1.5, 2.9, 3.14159}) {
    const Vec3 phi{th * axis[0], th * axis[1], th * axis[2]};
    const Vec3 got = RotationLog(Series(phi, 0));
    for (std::size_t a = 0; a < 3; ++a)
      EXPECT_NEAR(got[a], phi[a], 1e-12 + 1e-9 * th) << "θ " << th << " axis " << a;
  }
}

TEST(ImuRateExportModel, RecoversRateAndSpecificForceOnAConstantRateStep) {
  // Small-angle (series) and closed-form branches of Γ₁⁻¹: θ = |ω| Δt.
  for (const double scale : {0.0, 1e-4, 0.5, 30.0, 400.0}) {
    const Vec3 omega{0.8 * scale, -0.3 * scale, 0.5 * scale};
    const double dt = 0.006;
    const Case c = ConstantRate(omega, dt);
    double gyro[3] = {};
    double accel[3] = {};
    ASSERT_TRUE(TickMeanGyro(c.prev, c.cur, gyro));
    ASSERT_TRUE(TickMeanAccel(c.prev, c.cur, kG, accel));
    for (std::size_t a = 0; a < 3; ++a) {
      EXPECT_NEAR(gyro[a], omega[a], 1e-12 * (1.0 + Norm(omega))) << "θ " << Norm(omega) * dt;
      EXPECT_NEAR(accel[a], c.force[a], 1e-12 * Norm(c.force))
          << "θ " << Norm(omega) * dt << " axis " << a;
    }
  }
}

TEST(ImuRateExportModel, AtRestReadsMinusGravityInTheSiteFrame) {
  ImuSiteSnapshot s;
  s.rot = Series({0.2, 0.9, -0.4}, 0);
  s.vel = {0.1, 0.2, 0.3};
  ImuSiteSnapshot t = s;
  t.time = s.time + 0.002;
  double accel[3] = {};
  ASSERT_TRUE(TickMeanAccel(s, t, kG, accel));
  for (std::size_t a = 0; a < 3; ++a) {
    const double want = s.rot[2 * 3 + a] * 9.81;  // Rᵀ (0, 0, 9.81)
    EXPECT_NEAR(accel[a], want, 1e-13);
  }
}

TEST(ImuRateExportModel, NonPositiveOrNonFiniteIntervalLeavesTheOutputAlone) {
  const Case c = ConstantRate({1.0, 2.0, 3.0}, 0.002);
  for (const double t : {c.prev.time, c.prev.time - 0.002, std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
    ImuSiteSnapshot cur = c.cur;
    cur.time = t;
    double out[3] = {7.0, 8.0, 9.0};
    EXPECT_FALSE(TickMeanGyro(c.prev, cur, out));
    EXPECT_FALSE(TickMeanAccel(c.prev, cur, kG, out));
    EXPECT_EQ(out[0], 7.0);
    EXPECT_EQ(out[1], 8.0);
    EXPECT_EQ(out[2], 9.0);
  }
  double out[3] = {};
  EXPECT_FALSE(TickMeanGyro(c.prev, c.cur, nullptr));
  EXPECT_FALSE(TickMeanAccel(c.prev, c.cur, nullptr, out));
}

}  // namespace
}  // namespace rtc
