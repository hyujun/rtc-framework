/// Unit tests for bt_utils.hpp: ParseCsvList, EstimateHandTrajectoryDuration, LookupOrThrow.

#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace rtc_bt;

// ── ParseCsvList<int> ─────────────────────────────────────────────────────

TEST(ParseCsvList, IntValid)
{
  auto v = ParseCsvList<int>("1,2,3");
  ASSERT_EQ(v.size(), 3u);
  EXPECT_EQ(v[0], 1);
  EXPECT_EQ(v[1], 2);
  EXPECT_EQ(v[2], 3);
}

TEST(ParseCsvList, IntSingle)
{
  auto v = ParseCsvList<int>("42");
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v[0], 42);
}

TEST(ParseCsvList, IntNegative)
{
  auto v = ParseCsvList<int>("-1,-2");
  ASSERT_EQ(v.size(), 2u);
  EXPECT_EQ(v[0], -1);
  EXPECT_EQ(v[1], -2);
}

TEST(ParseCsvList, IntEmpty)
{
  // Empty string: getline produces one empty token.
  // stoi("") throws → wrapped as RuntimeError, OR returns empty depending on impl.
  // Current impl: empty token → stoi throws → RuntimeError
  // If the impl swallows it, just check the behavior is consistent.
  try {
    auto v = ParseCsvList<int>("");
    // If no throw, result should be empty (no valid tokens)
    EXPECT_TRUE(v.empty());
  } catch (const BT::RuntimeError&) {
    // Also acceptable: throws on empty token
    SUCCEED();
  }
}

TEST(ParseCsvList, IntInvalidToken)
{
  EXPECT_THROW(ParseCsvList<int>("1,abc,3"), BT::RuntimeError);
}

// ── ParseCsvList<double> ──────────────────────────────────────────────────

TEST(ParseCsvList, DoubleValid)
{
  auto v = ParseCsvList<double>("1.5,2.5,3.5");
  ASSERT_EQ(v.size(), 3u);
  EXPECT_DOUBLE_EQ(v[0], 1.5);
  EXPECT_DOUBLE_EQ(v[1], 2.5);
  EXPECT_DOUBLE_EQ(v[2], 3.5);
}

TEST(ParseCsvList, DoubleNegative)
{
  auto v = ParseCsvList<double>("-0.5,-1.5");
  ASSERT_EQ(v.size(), 2u);
  EXPECT_DOUBLE_EQ(v[0], -0.5);
}

// ── LookupOrThrow ─────────────────────────────────────────────────────────

TEST(LookupOrThrow, Found)
{
  std::map<std::string, int> m = {{"a", 1}, {"b", 2}};
  EXPECT_EQ(LookupOrThrow(m, "a", "test"), 1);
}

TEST(LookupOrThrow, NotFoundThrows)
{
  std::map<std::string, int> m = {{"a", 1}};
  EXPECT_THROW(LookupOrThrow(m, "z", "test"), BT::RuntimeError);
}

// ── EstimateHandTrajectoryDuration (indexed version) ──────────────────────

TEST(TrajectoryDuration, IndexedZeroDistance)
{
  std::vector<double> current(10, 0.5);
  HandPose target{};
  for (auto& v : target) v = 0.5;
  std::vector<int> indices = {0, 1, 2};

  double d = EstimateHandTrajectoryDuration(current, target, indices, 1.0, 2.0);
  // Zero distance → minimum 0.01 * margin(1.1)
  EXPECT_NEAR(d, 0.011, 1e-6);
}

TEST(TrajectoryDuration, IndexedKnownDistance)
{
  std::vector<double> current(10, 0.0);
  HandPose target{};
  target[0] = 1.0;  // 1 radian distance at index 0
  std::vector<int> indices = {0};

  // speed=1.0 → T_speed = 1.0/1.0 = 1.0
  // max_vel=2.0 → T_vel = 1.875*1.0/2.0 = 0.9375
  // max(1.0, 0.9375) = 1.0 * margin(1.1) = 1.1
  double d = EstimateHandTrajectoryDuration(current, target, indices, 1.0, 2.0);
  EXPECT_NEAR(d, 1.1, 1e-6);
}

TEST(TrajectoryDuration, IndexedMaxVelDominates)
{
  std::vector<double> current(10, 0.0);
  HandPose target{};
  target[3] = 2.0;  // 2 radian distance
  std::vector<int> indices = {3};

  // speed=10.0 → T_speed = 2.0/10.0 = 0.2
  // max_vel=1.0 → T_vel = 1.875*2.0/1.0 = 3.75
  // max(0.2, 3.75) = 3.75 * margin = 4.125
  double d = EstimateHandTrajectoryDuration(current, target, indices, 10.0, 1.0);
  EXPECT_NEAR(d, 4.125, 1e-6);
}

// ── EstimateHandTrajectoryDuration (full 10-DoF version) ──────────────────

TEST(TrajectoryDuration, FullZeroDistance)
{
  std::vector<double> current(10, 0.0);
  std::vector<double> target(10, 0.0);
  double d = EstimateHandTrajectoryDuration(current, target, 1.0, 2.0);
  EXPECT_NEAR(d, 0.011, 1e-6);
}

TEST(TrajectoryDuration, FullKnownDistance)
{
  std::vector<double> current(10, 0.0);
  std::vector<double> target(10, 0.0);
  target[5] = 0.5;  // max dist = 0.5

  // speed=1.0 → T_speed = 0.5
  // max_vel=2.0 → T_vel = 1.875*0.5/2.0 = 0.46875
  // max = 0.5 * 1.1 = 0.55
  double d = EstimateHandTrajectoryDuration(current, target, 1.0, 2.0);
  EXPECT_NEAR(d, 0.55, 1e-6);
}

TEST(TrajectoryDuration, CustomMargin)
{
  std::vector<double> current(10, 0.0);
  std::vector<double> target(10, 0.0);
  target[0] = 1.0;

  // speed=1.0 → T_speed = 1.0, margin=1.5 → 1.5
  double d = EstimateHandTrajectoryDuration(current, target, 1.0, 2.0, 1.5);
  EXPECT_NEAR(d, 1.5, 1e-6);
}

// ── Hand pose config constants ────────────────────────────────────────────

TEST(HandPoseConfig, FingerJointIndices)
{
  EXPECT_EQ(kFingerJointIndices.at("thumb").size(), 3u);
  EXPECT_EQ(kFingerJointIndices.at("index").size(), 3u);
  EXPECT_EQ(kFingerJointIndices.at("middle").size(), 3u);
  EXPECT_EQ(kFingerJointIndices.at("ring").size(), 1u);

  // Verify index ranges
  EXPECT_EQ(kFingerJointIndices.at("thumb")[0], 0);
  EXPECT_EQ(kFingerJointIndices.at("index")[0], 3);
  EXPECT_EQ(kFingerJointIndices.at("middle")[0], 6);
  EXPECT_EQ(kFingerJointIndices.at("ring")[0], 9);
}

TEST(HandPoseConfig, DefaultPosesExist)
{
  EXPECT_TRUE(kHandPoses.count("home") > 0);
  EXPECT_TRUE(kHandPoses.count("full_flex") > 0);
  EXPECT_TRUE(kUR5ePoses.count("home_pose") > 0);
  EXPECT_TRUE(kUR5ePoses.count("demo_pose") > 0);
}

TEST(HandPoseConfig, HomePoseIsZero)
{
  const auto& home = kHandPoses.at("home");
  for (int i = 0; i < kHandDofCount; ++i) {
    EXPECT_DOUBLE_EQ(home[i], 0.0) << "home[" << i << "] != 0";
  }
}

TEST(HandPoseConfig, DegToRadConversion)
{
  // full_flex thumb CMC abd = 30 deg
  const auto& ff = kHandPoses.at("full_flex");
  EXPECT_NEAR(ff[0], 30.0 * kDeg2Rad, 1e-10);
}
