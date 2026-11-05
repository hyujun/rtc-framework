// MuJoCo ↔ pinocchio floating-base conventions (W0-F02, #976).
//
// The header is the ONE place the quaternion order (wxyz ↔ xyzw) and the
// twist frame (world ↔ base) are converted, and both mistakes are silent: a
// swapped quaternion is still unit, a world twist is still finite. Two kinds
// of case pin it:
//   - directed cases whose answer is known by hand — a yaw of 90° turns the
//     world x axis into the body's −y, and q = (.5,.5,.5,.5) is the cyclic
//     axis permutation x→y→z, so a sign or an axis-order slip lands on a
//     DIFFERENT finite vector rather than a nearby one;
//   - 1000 seeded random pose/twist pairs through the forward and inverse
//     maps, which catch what the directed cases cannot (a transposed R, an
//     active/passive mix-up that happens to agree on the directed inputs).
// 1e-12 is the Sprint's tolerance: double round-off of one rotation and one
// reorder is ~1e-15, so it leaves three orders of headroom while refusing a
// float32 detour (~1e-7).

#include "integrated_bringup/backends/floating_base_conventions.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <string>

namespace {

constexpr double kTol = 1e-12;

using Vec3 = std::array<double, 3>;
using Quat4 = std::array<double, 4>;

// ── Directed: quaternion order ──────────────────────────────────────────────

TEST(FloatingBaseConventions, QuaternionOrderIsWxyzToXyzw) {
  // Components all distinct so any permutation other than the right one is
  // visible in at least one slot.
  const Quat4 q_wxyz = {0.1, 0.2, 0.3, 0.4};
  Quat4 unit = q_wxyz;
  const double n = std::sqrt(0.01 + 0.04 + 0.09 + 0.16);
  for (auto& c : unit)
    c /= n;
  const auto s =
      rtc::MujocoFrameToPinocchio({0.0, 0.0, 0.0}, unit, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(s.pose[3], unit[1]);  // x
  EXPECT_DOUBLE_EQ(s.pose[4], unit[2]);  // y
  EXPECT_DOUBLE_EQ(s.pose[5], unit[3]);  // z
  EXPECT_DOUBLE_EQ(s.pose[6], unit[0]);  // w — last in pinocchio
}

TEST(FloatingBaseConventions, PositionIsCopiedVerbatim) {
  const auto s = rtc::MujocoFrameToPinocchio({1.5, -2.5, 0.75}, {1.0, 0.0, 0.0, 0.0},
                                             {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(s.pose[0], 1.5);
  EXPECT_DOUBLE_EQ(s.pose[1], -2.5);
  EXPECT_DOUBLE_EQ(s.pose[2], 0.75);
}

// ── Directed: twist frame ───────────────────────────────────────────────────

// Yaw +90° about z: the body's x axis points along world +y, so a world
// velocity along +x is seen from the body as −y. An un-transposed R (active
// instead of passive) gives +y — the same magnitude, the opposite sign.
TEST(FloatingBaseConventions, Yaw90WorldXBecomesBodyMinusY) {
  const double h = std::sqrt(0.5);
  const Quat4 q_wxyz = {h, 0.0, 0.0, h};  // w, x, y, z
  const auto s =
      rtc::MujocoFrameToPinocchio({0.0, 0.0, 0.0}, q_wxyz, {1.0, 0.0, 0.0}, {0.0, 0.0, 2.0});
  EXPECT_NEAR(s.twist[0], 0.0, kTol);
  EXPECT_NEAR(s.twist[1], -1.0, kTol);
  EXPECT_NEAR(s.twist[2], 0.0, kTol);
  // ω about the rotation axis itself is unchanged by the rotation.
  EXPECT_NEAR(s.twist[3], 0.0, kTol);
  EXPECT_NEAR(s.twist[4], 0.0, kTol);
  EXPECT_NEAR(s.twist[5], 2.0, kTol);
}

// q = (.5,.5,.5,.5) is the 120° rotation about (1,1,1): R maps x→y, y→z, z→x.
// Rᵀ therefore maps world x → body z (and y → x, z → y). An axis-order slip
// in the quaternion constructor, or R used in place of Rᵀ, both land on a
// different unit axis — never on a near miss.
TEST(FloatingBaseConventions, CyclicPermutationQuaternionOrdersAxes) {
  const Quat4 q_wxyz = {0.5, 0.5, 0.5, 0.5};
  const auto s =
      rtc::MujocoFrameToPinocchio({0.0, 0.0, 0.0}, q_wxyz, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
  EXPECT_NEAR(s.twist[0], 0.0, kTol);
  EXPECT_NEAR(s.twist[1], 0.0, kTol);
  EXPECT_NEAR(s.twist[2], 1.0, kTol);  // world x → body z
  EXPECT_NEAR(s.twist[3], 1.0, kTol);  // world y → body x
  EXPECT_NEAR(s.twist[4], 0.0, kTol);
  EXPECT_NEAR(s.twist[5], 0.0, kTol);
}

// Positive control for the test above against an identity-only
// implementation: with the identity quaternion the twist passes through.
TEST(FloatingBaseConventions, IdentityLeavesTwistUnchanged) {
  const auto s = rtc::MujocoFrameToPinocchio({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}, {1.0, 2.0, 3.0},
                                             {4.0, 5.0, 6.0});
  for (int i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(s.twist[static_cast<std::size_t>(i)], static_cast<double>(i + 1)) << i;
  }
}

// ── Round trip: 1000 seeded random pose / twist pairs ───────────────────────

TEST(FloatingBaseConventions, RoundTrip1000RandomWithin1e12) {
  std::mt19937_64 rng(0x5EED'F02ULL);  // fixed seed — a failure is reproducible
  std::uniform_real_distribution<double> pos_d(-5.0, 5.0);
  std::uniform_real_distribution<double> vel_d(-20.0, 20.0);
  std::normal_distribution<double> gauss(0.0, 1.0);

  double worst = 0.0;
  for (int k = 0; k < 1000; ++k) {
    // Uniform random unit quaternion: normalised 4-D Gaussian.
    Eigen::Quaterniond q(gauss(rng), gauss(rng), gauss(rng), gauss(rng));
    q.normalize();
    const Quat4 q_wxyz = {q.w(), q.x(), q.y(), q.z()};
    const Vec3 p = {pos_d(rng), pos_d(rng), pos_d(rng)};
    const Vec3 v_w = {vel_d(rng), vel_d(rng), vel_d(rng)};
    const Vec3 w_w = {vel_d(rng), vel_d(rng), vel_d(rng)};

    const auto s = rtc::MujocoFrameToPinocchio(p, q_wxyz, v_w, w_w);

    // The body-frame twist is an independent check against Eigen's own
    // rotation matrix — not merely the inverse map undoing the forward one.
    const Eigen::Matrix3d R = q.toRotationMatrix();
    const Eigen::Vector3d v_b = R.transpose() * Eigen::Vector3d(v_w[0], v_w[1], v_w[2]);
    const Eigen::Vector3d w_b = R.transpose() * Eigen::Vector3d(w_w[0], w_w[1], w_w[2]);
    for (int i = 0; i < 3; ++i) {
      worst = std::max(worst, std::abs(s.twist[static_cast<std::size_t>(i)] - v_b[i]));
      worst = std::max(worst, std::abs(s.twist[static_cast<std::size_t>(3 + i)] - w_b[i]));
    }

    Vec3 p2{};
    Quat4 q2{};
    Vec3 v2{};
    Vec3 w2{};
    rtc::PinocchioToMujocoFrame(s, p2, q2, v2, w2);
    for (int i = 0; i < 3; ++i) {
      const auto iu = static_cast<std::size_t>(i);
      worst = std::max(worst, std::abs(p2[iu] - p[iu]));
      worst = std::max(worst, std::abs(v2[iu] - v_w[iu]));
      worst = std::max(worst, std::abs(w2[iu] - w_w[iu]));
    }
    for (std::size_t i = 0; i < 4; ++i) {
      worst = std::max(worst, std::abs(q2[i] - q_wxyz[i]));
    }
    ASSERT_LE(worst, kTol) << "sample " << k;
  }
  // The bound has headroom, not slack: report how much so a future regression
  // that stays under 1e-12 is still visible in the log.
  RecordProperty("worst_abs_err", std::to_string(worst));
  EXPECT_GT(worst, 0.0) << "1000 random samples cannot all round-trip to exactly 0";
}

// ── Guard ───────────────────────────────────────────────────────────────────

TEST(FloatingBaseConventions, UnitQuaternionGuard) {
  EXPECT_TRUE(rtc::IsUnitQuaternionWxyz({1.0, 0.0, 0.0, 0.0}));
  EXPECT_TRUE(rtc::IsUnitQuaternionWxyz({0.5, 0.5, 0.5, 0.5}));
  EXPECT_FALSE(rtc::IsUnitQuaternionWxyz({0.0, 0.0, 0.0, 0.0}));
  EXPECT_FALSE(rtc::IsUnitQuaternionWxyz({2.0, 0.0, 0.0, 0.0}));
  EXPECT_FALSE(rtc::IsUnitQuaternionWxyz({std::nan(""), 0.0, 0.0, 0.0}));
  EXPECT_FALSE(rtc::IsUnitQuaternionWxyz({1.0, 0.0, 0.0, std::numeric_limits<double>::infinity()}));
}

}  // namespace
