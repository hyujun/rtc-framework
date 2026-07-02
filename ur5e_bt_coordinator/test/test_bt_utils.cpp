/// Unit tests for bt_utils.hpp: ParseCsvList, EstimateHandTrajectoryDuration, LookupOrThrow.

#include "ur5e_bt_coordinator/bt_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace rtc_bt;

// ── ParseCsvList<int> ─────────────────────────────────────────────────────

TEST(ParseCsvList, IntValid) {
  auto v = ParseCsvList<int>("1,2,3");
  ASSERT_EQ(v.size(), 3u);
  EXPECT_EQ(v[0], 1);
  EXPECT_EQ(v[1], 2);
  EXPECT_EQ(v[2], 3);
}

TEST(ParseCsvList, IntSingle) {
  auto v = ParseCsvList<int>("42");
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v[0], 42);
}

TEST(ParseCsvList, IntNegative) {
  auto v = ParseCsvList<int>("-1,-2");
  ASSERT_EQ(v.size(), 2u);
  EXPECT_EQ(v[0], -1);
  EXPECT_EQ(v[1], -2);
}

TEST(ParseCsvList, IntEmpty) {
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

TEST(ParseCsvList, IntInvalidToken) {
  EXPECT_THROW(ParseCsvList<int>("1,abc,3"), BT::RuntimeError);
}

// ── ParseCsvList<double> ──────────────────────────────────────────────────

TEST(ParseCsvList, DoubleValid) {
  auto v = ParseCsvList<double>("1.5,2.5,3.5");
  ASSERT_EQ(v.size(), 3u);
  EXPECT_DOUBLE_EQ(v[0], 1.5);
  EXPECT_DOUBLE_EQ(v[1], 2.5);
  EXPECT_DOUBLE_EQ(v[2], 3.5);
}

TEST(ParseCsvList, DoubleNegative) {
  auto v = ParseCsvList<double>("-0.5,-1.5");
  ASSERT_EQ(v.size(), 2u);
  EXPECT_DOUBLE_EQ(v[0], -0.5);
}

// ── LookupOrThrow ─────────────────────────────────────────────────────────

TEST(LookupOrThrow, Found) {
  std::map<std::string, int> m = {{"a", 1}, {"b", 2}};
  EXPECT_EQ(LookupOrThrow(m, "a", "test"), 1);
}

TEST(LookupOrThrow, NotFoundThrows) {
  std::map<std::string, int> m = {{"a", 1}};
  EXPECT_THROW(LookupOrThrow(m, "z", "test"), BT::RuntimeError);
}

// ── EstimateHandTrajectoryDuration (indexed version) ──────────────────────

TEST(TrajectoryDuration, IndexedZeroDistance) {
  std::vector<double> current(10, 0.5);
  HandPose target{};
  for (auto& v : target)
    v = 0.5;
  std::vector<int> indices = {0, 1, 2};

  double d = EstimateHandTrajectoryDuration(current, target, indices, 1.0, 2.0);
  // Zero distance → minimum 0.01 * margin(1.1)
  EXPECT_NEAR(d, 0.011, 1e-6);
}

TEST(TrajectoryDuration, IndexedKnownDistance) {
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

TEST(TrajectoryDuration, IndexedMaxVelDominates) {
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

TEST(TrajectoryDuration, FullZeroDistance) {
  std::vector<double> current(10, 0.0);
  std::vector<double> target(10, 0.0);
  double d = EstimateHandTrajectoryDuration(current, target, 1.0, 2.0);
  EXPECT_NEAR(d, 0.011, 1e-6);
}

TEST(TrajectoryDuration, FullKnownDistance) {
  std::vector<double> current(10, 0.0);
  std::vector<double> target(10, 0.0);
  target[5] = 0.5;  // max dist = 0.5

  // speed=1.0 → T_speed = 0.5
  // max_vel=2.0 → T_vel = 1.875*0.5/2.0 = 0.46875
  // max = 0.5 * 1.1 = 0.55
  double d = EstimateHandTrajectoryDuration(current, target, 1.0, 2.0);
  EXPECT_NEAR(d, 0.55, 1e-6);
}

TEST(TrajectoryDuration, CustomMargin) {
  std::vector<double> current(10, 0.0);
  std::vector<double> target(10, 0.0);
  target[0] = 1.0;

  // speed=1.0 → T_speed = 1.0, margin=1.5 → 1.5
  double d = EstimateHandTrajectoryDuration(current, target, 1.0, 2.0, 1.5);
  EXPECT_NEAR(d, 1.5, 1e-6);
}

// ── Hand pose config constants ────────────────────────────────────────────

// FingerJointIndices() derives finger→joint index groups from joint_states
// names by prefix matching, so per-finger DoF is not hardcoded.
TEST(HandPoseConfig, FingerJointIndicesAssmV1) {
  // assm_v1 hand joint order (thumb:3 / index:3 / middle:3 / ring:1).
  const std::vector<std::string> names = {
      "thumb_cmc_aa", "thumb_cmc_fe",  "thumb_mcp_fe",  "index_mcp_aa",  "index_mcp_fe",
      "index_dip_fe", "middle_mcp_aa", "middle_mcp_fe", "middle_dip_fe", "ring_mcp_fe"};

  EXPECT_EQ(FingerJointIndices(names, "thumb"), (std::vector<int>{0, 1, 2}));
  EXPECT_EQ(FingerJointIndices(names, "index"), (std::vector<int>{3, 4, 5}));
  EXPECT_EQ(FingerJointIndices(names, "middle"), (std::vector<int>{6, 7, 8}));
  EXPECT_EQ(FingerJointIndices(names, "ring"), (std::vector<int>{9}));

  // Sub-groups (used by FlexExtendFinger partial flex) via `<finger>_<seg>` prefix.
  EXPECT_EQ(FingerJointIndices(names, "thumb_mcp"), (std::vector<int>{2}));
  EXPECT_EQ(FingerJointIndices(names, "index_dip"), (std::vector<int>{5}));
  EXPECT_EQ(FingerJointIndices(names, "middle_dip"), (std::vector<int>{8}));

  // Unknown finger / empty names → empty (caller throws).
  EXPECT_TRUE(FingerJointIndices(names, "pinky").empty());
  EXPECT_TRUE(FingerJointIndices({}, "thumb").empty());
}

// Variable per-finger DoF (proto_1b: thumb:4 / index:3 / middle:2 / ring:1).
// Same helper, no code change — driven entirely by the joint names.
TEST(HandPoseConfig, FingerJointIndicesProto1b) {
  const std::vector<std::string> names = {
      "thumb_cmc_aa", "thumb_cmc_fe", "thumb_mcp_aa",  "thumb_mcp_fe",  "index_mcp_aa",
      "index_mcp_fe", "index_dip_fe", "middle_mcp_aa", "middle_mcp_fe", "ring_mcp_fe"};

  EXPECT_EQ(FingerJointIndices(names, "thumb"), (std::vector<int>{0, 1, 2, 3}));  // 4 DoF
  EXPECT_EQ(FingerJointIndices(names, "index"), (std::vector<int>{4, 5, 6}));     // 3 DoF
  EXPECT_EQ(FingerJointIndices(names, "middle"), (std::vector<int>{7, 8}));       // 2 DoF
  EXPECT_EQ(FingerJointIndices(names, "ring"), (std::vector<int>{9}));            // 1 DoF
  EXPECT_EQ(FingerJointIndices(names, "thumb_mcp"), (std::vector<int>{2, 3}));
}

TEST(HandPoseConfig, DefaultPosesExist) {
  EXPECT_TRUE(kHandPoses.count("home") > 0);
  EXPECT_TRUE(kHandPoses.count("full_flex") > 0);
  EXPECT_TRUE(kUR5ePoses.count("home_pose") > 0);
  EXPECT_TRUE(kUR5ePoses.count("demo_pose") > 0);
}

TEST(HandPoseConfig, HomePoseIsZero) {
  const auto& home = kHandPoses.at("home");
  for (int i = 0; i < kHandDofCount; ++i) {
    EXPECT_DOUBLE_EQ(home[i], 0.0) << "home[" << i << "] != 0";
  }
}

TEST(HandPoseConfig, DegToRadConversion) {
  // full_flex thumb CMC abd = 30 deg
  const auto& ff = kHandPoses.at("full_flex");
  EXPECT_NEAR(ff[0], 30.0 * kDeg2Rad, 1e-10);
}
