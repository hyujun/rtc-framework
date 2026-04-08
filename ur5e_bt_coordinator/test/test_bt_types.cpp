/// Unit tests for bt_types.hpp: Pose6D and BT string conversions.

#include "ur5e_bt_coordinator/bt_types.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using rtc_bt::Pose6D;

// ── Pose6D distance tests ─────────────────────────────────────────────────

TEST(Pose6D, PositionDistanceToSelf)
{
  Pose6D p{1.0, 2.0, 3.0, 0.1, 0.2, 0.3};
  EXPECT_DOUBLE_EQ(p.PositionDistanceTo(p), 0.0);
}

TEST(Pose6D, PositionDistanceKnown)
{
  Pose6D a{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Pose6D b{3.0, 4.0, 0.0, 0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(a.PositionDistanceTo(b), 5.0);
}

TEST(Pose6D, PositionDistanceNegative)
{
  Pose6D a{-1.0, -2.0, -3.0, 0.0, 0.0, 0.0};
  Pose6D b{ 1.0,  2.0,  3.0, 0.0, 0.0, 0.0};
  double expected = std::sqrt(4.0 + 16.0 + 36.0);
  EXPECT_NEAR(a.PositionDistanceTo(b), expected, 1e-12);
}

TEST(Pose6D, PositionDistanceSymmetric)
{
  Pose6D a{1.0, 2.0, 3.0, 0.0, 0.0, 0.0};
  Pose6D b{4.0, 5.0, 6.0, 0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(a.PositionDistanceTo(b), b.PositionDistanceTo(a));
}

TEST(Pose6D, OrientationDistanceToSelf)
{
  Pose6D p{0.0, 0.0, 0.0, 1.0, 2.0, 3.0};
  EXPECT_DOUBLE_EQ(p.OrientationDistanceTo(p), 0.0);
}

TEST(Pose6D, OrientationDistanceKnown)
{
  Pose6D a{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Pose6D b{0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
  double expected = std::sqrt(0.01 + 0.04 + 0.09);
  EXPECT_NEAR(a.OrientationDistanceTo(b), expected, 1e-12);
}

TEST(Pose6D, OrientationIgnoresPosition)
{
  Pose6D a{100.0, 200.0, 300.0, 0.1, 0.2, 0.3};
  Pose6D b{  0.0,   0.0,   0.0, 0.1, 0.2, 0.3};
  EXPECT_DOUBLE_EQ(a.OrientationDistanceTo(b), 0.0);
}

// ── BT::convertFromString<Pose6D> tests ───────────────────────────────────

TEST(BtConvert, Pose6DValid)
{
  auto p = BT::convertFromString<Pose6D>("1.0;2.0;3.0;0.1;0.2;0.3");
  EXPECT_DOUBLE_EQ(p.x, 1.0);
  EXPECT_DOUBLE_EQ(p.y, 2.0);
  EXPECT_DOUBLE_EQ(p.z, 3.0);
  EXPECT_DOUBLE_EQ(p.roll, 0.1);
  EXPECT_DOUBLE_EQ(p.pitch, 0.2);
  EXPECT_DOUBLE_EQ(p.yaw, 0.3);
}

TEST(BtConvert, Pose6DNegativeValues)
{
  auto p = BT::convertFromString<Pose6D>("-1.5;-2.5;-3.5;-0.1;-0.2;-0.3");
  EXPECT_DOUBLE_EQ(p.x, -1.5);
  EXPECT_DOUBLE_EQ(p.y, -2.5);
  EXPECT_DOUBLE_EQ(p.z, -3.5);
}

TEST(BtConvert, Pose6DWrongCountThrows)
{
  EXPECT_THROW(BT::convertFromString<Pose6D>("1.0;2.0;3.0"), BT::RuntimeError);
  EXPECT_THROW(BT::convertFromString<Pose6D>("1;2;3;4;5;6;7"), BT::RuntimeError);
}

TEST(BtConvert, Pose6DEmptyThrows)
{
  EXPECT_THROW(BT::convertFromString<Pose6D>(""), BT::RuntimeError);
}

// ── BT::convertFromString<vector<double>> tests ──────────────────────────

TEST(BtConvert, VectorDoubleValid)
{
  auto v = BT::convertFromString<std::vector<double>>("0.1;0.2;0.3");
  ASSERT_EQ(v.size(), 3u);
  EXPECT_DOUBLE_EQ(v[0], 0.1);
  EXPECT_DOUBLE_EQ(v[1], 0.2);
  EXPECT_DOUBLE_EQ(v[2], 0.3);
}

TEST(BtConvert, VectorDoubleSingle)
{
  auto v = BT::convertFromString<std::vector<double>>("42.0");
  ASSERT_EQ(v.size(), 1u);
  EXPECT_DOUBLE_EQ(v[0], 42.0);
}

TEST(BtConvert, VectorDoubleNegative)
{
  auto v = BT::convertFromString<std::vector<double>>("-1.0;-2.0");
  ASSERT_EQ(v.size(), 2u);
  EXPECT_DOUBLE_EQ(v[0], -1.0);
  EXPECT_DOUBLE_EQ(v[1], -2.0);
}

// ── BT::convertFromString<vector<Pose6D>> tests ─────────────────────────

TEST(BtConvert, VectorPose6DValid)
{
  auto v = BT::convertFromString<std::vector<Pose6D>>(
      "1;2;3;0.1;0.2;0.3|4;5;6;0.4;0.5;0.6");
  ASSERT_EQ(v.size(), 2u);
  EXPECT_DOUBLE_EQ(v[0].x, 1.0);
  EXPECT_DOUBLE_EQ(v[1].x, 4.0);
  EXPECT_DOUBLE_EQ(v[1].yaw, 0.6);
}

TEST(BtConvert, VectorPose6DSingle)
{
  auto v = BT::convertFromString<std::vector<Pose6D>>("1;2;3;4;5;6");
  ASSERT_EQ(v.size(), 1u);
  EXPECT_DOUBLE_EQ(v[0].x, 1.0);
}
