// ── test_msg_conversions.cpp ─────────────────────────────────────────────────
// Pure conversion tests: rtc_msgs::ToFSnapshot ↔ internal ToFSnapshot, and
// internal ShapeEstimate → shape_estimation_msgs::ShapeEstimate. No ROS spin.
// ──────────────────────────────────────────────────────────────────────────────
#include "shape_estimation/msg_conversions.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

namespace se = shape_estimation;
namespace {

// Identity-pose snapshot msg with uniform distance/validity on all 6 sensors.
rtc_msgs::msg::ToFSnapshot MakeUniformMsg(double dist, bool valid) {
  rtc_msgs::msg::ToFSnapshot msg;
  msg.stamp.sec = 2;
  msg.stamp.nanosec = 500'000'000U;
  for (std::size_t i = 0; i < 6; ++i) {
    msg.distances[i] = dist;
    msg.valid[i] = valid;
  }
  for (std::size_t f = 0; f < 3; ++f) {
    msg.tip_poses[f].position.x = 0.0;
    msg.tip_poses[f].position.y = 0.0;
    msg.tip_poses[f].position.z = 0.0;
    msg.tip_poses[f].orientation.w = 1.0;  // identity rotation → beam = +z
    msg.tip_poses[f].orientation.x = 0.0;
    msg.tip_poses[f].orientation.y = 0.0;
    msg.tip_poses[f].orientation.z = 0.0;
  }
  return msg;
}

// ── ConvertFromMsg ──────────────────────────────────────────────────────────

TEST(ConvertFromMsg, TimestampCombinesSecAndNanosec) {
  const auto snap = se::ConvertFromMsg(MakeUniformMsg(0.05, true));
  EXPECT_EQ(snap.timestamp_ns, 2'500'000'000ULL);
}

TEST(ConvertFromMsg, NegativeSecClampedToZero) {
  auto msg = MakeUniformMsg(0.05, true);
  msg.stamp.sec = -5;
  msg.stamp.nanosec = 7U;
  const auto snap = se::ConvertFromMsg(msg);
  EXPECT_EQ(snap.timestamp_ns, 7ULL);  // sec clamped to 0
}

TEST(ConvertFromMsg, ValidInRangeReadingsGeometry) {
  const double d = 0.05;  // within [kTofMinRange, kTofMaxRange]
  const auto snap = se::ConvertFromMsg(MakeUniformMsg(d, true));

  for (int i = 0; i < se::kTotalSensors; ++i) {
    EXPECT_TRUE(snap.readings[static_cast<std::size_t>(i)].valid);
  }
  // Identity orientation → beam along +z.
  for (int f = 0; f < se::kNumFingers; ++f) {
    const auto& beam = snap.beam_directions_world[static_cast<std::size_t>(f)];
    EXPECT_NEAR(beam.x(), 0.0, 1e-9);
    EXPECT_NEAR(beam.y(), 0.0, 1e-9);
    EXPECT_NEAR(beam.z(), 1.0, 1e-9);
  }
  // Sensor A at +x offset, B at -x offset; surface point = sensor + d*beam.
  EXPECT_NEAR(snap.sensor_positions_world[0].x(), se::kTofOffsetX, 1e-9);
  EXPECT_NEAR(snap.sensor_positions_world[1].x(), -se::kTofOffsetX, 1e-9);
  EXPECT_NEAR(snap.surface_points_world[0].z(), d, 1e-9);
}

TEST(ConvertFromMsg, OutOfRangeReadingsRejected) {
  // 0.5 m exceeds kTofMaxRange (0.30) → reading invalid despite msg.valid=true.
  const auto snap = se::ConvertFromMsg(MakeUniformMsg(0.5, true));
  for (int i = 0; i < se::kTotalSensors; ++i) {
    EXPECT_FALSE(snap.readings[static_cast<std::size_t>(i)].valid);
  }
  for (int f = 0; f < se::kNumFingers; ++f) {
    EXPECT_FALSE(snap.curvature_valid[static_cast<std::size_t>(f)]);
  }
}

TEST(ConvertFromMsg, EqualDistancesGiveZeroCurvature) {
  const auto snap = se::ConvertFromMsg(MakeUniformMsg(0.05, true));
  for (int f = 0; f < se::kNumFingers; ++f) {
    EXPECT_TRUE(snap.curvature_valid[static_cast<std::size_t>(f)]);
    EXPECT_NEAR(snap.local_curvatures[static_cast<std::size_t>(f)], 0.0, 1e-12);
  }
}

TEST(ConvertFromMsg, AsymmetricDistancesGivePositiveCurvature) {
  auto msg = MakeUniformMsg(0.05, true);
  msg.distances[0] = 0.05;  // finger 0, sensor A
  msg.distances[1] = 0.04;  // finger 0, sensor B
  const auto snap = se::ConvertFromMsg(msg);

  ASSERT_TRUE(snap.curvature_valid[0]);
  // κ = 2·Δd / (Δx² + Δd²), Δd = 0.01, Δx = kTofSeparation = 0.004
  const double dd = 0.01;
  const double dx2 = se::kTofSeparation * se::kTofSeparation;
  const double expected = 2.0 * dd / (dx2 + dd * dd);
  EXPECT_NEAR(snap.local_curvatures[0], expected, 1e-6);
  EXPECT_GT(snap.local_curvatures[0], 0.0);
}

// ── ToMsg (estimate only) ───────────────────────────────────────────────────

se::ShapeEstimate MakeEstimate() {
  se::ShapeEstimate e;
  e.type = se::ShapeType::kSphere;
  e.confidence = 0.7;
  e.center = Eigen::Vector3d(1.0, 2.0, 3.0);
  e.axis = Eigen::Vector3d(0.0, 0.0, 1.0);
  e.radius = 0.5;
  e.dimensions = Eigen::Vector3d(0.1, 0.2, 0.3);
  e.num_points_used = 42;
  return e;
}

TEST(ToMsg, EstimateOnlyMapsFieldsAndZeroesCurvature) {
  std_msgs::msg::Header header;
  header.stamp.sec = 5;
  const auto msg = se::ToMsg(MakeEstimate(), header);

  EXPECT_EQ(msg.shape_type, static_cast<uint8_t>(se::ShapeType::kSphere));
  EXPECT_DOUBLE_EQ(msg.confidence, 0.7);
  EXPECT_DOUBLE_EQ(msg.center.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.center.z, 3.0);
  EXPECT_DOUBLE_EQ(msg.radius, 0.5);
  EXPECT_DOUBLE_EQ(msg.dimensions.y, 0.2);
  EXPECT_EQ(msg.num_points_used, 42U);
  EXPECT_EQ(msg.stamp.sec, 5);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(msg.local_curvatures[i], 0.0);
    EXPECT_FALSE(msg.curvature_valid[i]);
  }
  EXPECT_FALSE(msg.has_protuberance);
}

TEST(ToMsg, WithSnapshotCopiesCurvature) {
  se::ToFSnapshot snap;
  snap.local_curvatures = {1.0, 2.0, 3.0};
  snap.curvature_valid = {true, false, true};
  std_msgs::msg::Header header;
  const auto msg = se::ToMsg(MakeEstimate(), snap, header);

  EXPECT_DOUBLE_EQ(msg.local_curvatures[0], 1.0);
  EXPECT_DOUBLE_EQ(msg.local_curvatures[2], 3.0);
  EXPECT_TRUE(msg.curvature_valid[0]);
  EXPECT_FALSE(msg.curvature_valid[1]);
}

TEST(ToMsg, WithProtuberanceDetectedPopulatesFields) {
  se::ToFSnapshot snap;
  se::ProtuberanceResult prot;
  prot.detected = true;
  se::Protuberance p;
  p.centroid = Eigen::Vector3d(0.1, 0.2, 0.3);
  p.direction = Eigen::Vector3d(0.0, 0.0, 1.0);
  p.protrusion_depth = 0.012;
  p.extent_along_surface = 0.05;
  p.confidence = 0.9;
  p.has_gap = true;
  p.num_points = 7;
  prot.protuberances.push_back(p);

  std_msgs::msg::Header header;
  const auto msg = se::ToMsg(MakeEstimate(), snap, prot, header);

  EXPECT_TRUE(msg.has_protuberance);
  EXPECT_DOUBLE_EQ(msg.protuberance_centroid.x, 0.1);
  EXPECT_DOUBLE_EQ(msg.protuberance_direction.z, 1.0);
  EXPECT_DOUBLE_EQ(msg.protuberance_depth, 0.012);
  EXPECT_EQ(msg.protuberance_num_points, 7U);
  EXPECT_TRUE(msg.protuberance_has_gap);
}

TEST(ToMsg, WithProtuberanceNotDetectedLeavesFlagFalse) {
  se::ToFSnapshot snap;
  se::ProtuberanceResult prot;  // detected = false, empty list
  std_msgs::msg::Header header;
  const auto msg = se::ToMsg(MakeEstimate(), snap, prot, header);
  EXPECT_FALSE(msg.has_protuberance);
}

}  // namespace
