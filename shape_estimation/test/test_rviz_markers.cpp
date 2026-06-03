// ── test_rviz_markers.cpp ────────────────────────────────────────────────────
// Pure marker-builder tests: color mappers, DELETE-all array, beam markers.
// No ROS node / spin — the builders just construct visualization_msgs structs.
// ──────────────────────────────────────────────────────────────────────────────
#include "shape_estimation/rviz_markers.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <vector>

namespace viz = shape_estimation::viz;
namespace se = shape_estimation;
namespace {

// ── ConfidenceToColor (green → yellow → orange → red) ───────────────────────

TEST(ConfidenceToColor, ThresholdBuckets) {
  const auto green = viz::ConfidenceToColor(0.9);
  EXPECT_FLOAT_EQ(green.r, 0.0F);
  EXPECT_FLOAT_EQ(green.g, 1.0F);
  EXPECT_FLOAT_EQ(green.b, 0.0F);

  const auto yellow = viz::ConfidenceToColor(0.6);
  EXPECT_FLOAT_EQ(yellow.r, 1.0F);
  EXPECT_FLOAT_EQ(yellow.g, 1.0F);

  const auto orange = viz::ConfidenceToColor(0.3);
  EXPECT_FLOAT_EQ(orange.r, 1.0F);
  EXPECT_FLOAT_EQ(orange.g, 0.5F);

  const auto red = viz::ConfidenceToColor(0.1);
  EXPECT_FLOAT_EQ(red.r, 1.0F);
  EXPECT_FLOAT_EQ(red.g, 0.0F);
}

TEST(ConfidenceToColor, AlphaPassThrough) {
  EXPECT_FLOAT_EQ(viz::ConfidenceToColor(0.9, 0.4).a, 0.4F);
  EXPECT_FLOAT_EQ(viz::ConfidenceToColor(0.9).a, 0.4F);  // default alpha
}

// ── CurvatureToColor (blue → green → red, on |κ|) ───────────────────────────

TEST(CurvatureToColor, MagnitudeBuckets) {
  const auto blue = viz::CurvatureToColor(2.0);  // |κ| < 5
  EXPECT_FLOAT_EQ(blue.r, 0.3F);
  EXPECT_FLOAT_EQ(blue.b, 1.0F);

  const auto green = viz::CurvatureToColor(10.0);  // 5 ≤ |κ| < 20
  EXPECT_FLOAT_EQ(green.g, 1.0F);
  EXPECT_FLOAT_EQ(green.r, 0.0F);

  const auto red = viz::CurvatureToColor(30.0);  // |κ| ≥ 20
  EXPECT_FLOAT_EQ(red.r, 1.0F);
  EXPECT_FLOAT_EQ(red.g, 0.0F);
}

TEST(CurvatureToColor, UsesAbsoluteValue) {
  const auto pos = viz::CurvatureToColor(30.0);
  const auto neg = viz::CurvatureToColor(-30.0);
  EXPECT_FLOAT_EQ(neg.r, pos.r);
  EXPECT_FLOAT_EQ(neg.g, pos.g);
  EXPECT_FLOAT_EQ(neg.b, pos.b);
}

// ── BuildDeleteAllMarkers ───────────────────────────────────────────────────

TEST(BuildDeleteAllMarkers, AllMarkersAreDeleteAction) {
  const auto ma = viz::BuildDeleteAllMarkers();
  // 6 beams + 3 curvature texts + 9 singletons = 18.
  EXPECT_EQ(ma.markers.size(),
            static_cast<std::size_t>(se::kTotalSensors + se::kNumFingers + 9));
  for (const auto& m : ma.markers) {
    EXPECT_EQ(m.action, visualization_msgs::msg::Marker::DELETE);
  }
  EXPECT_EQ(ma.markers.front().ns, "tof_beams");
}

// ── BuildBeamMarkers ────────────────────────────────────────────────────────

se::ToFSnapshot MakeSnapshot(bool valid) {
  se::ToFSnapshot snap;
  for (int i = 0; i < se::kTotalSensors; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    snap.readings[idx].distance_m = 0.05;
    snap.readings[idx].valid = valid;
    snap.sensor_positions_world[idx] = Eigen::Vector3d(0.01 * i, 0, 0);
    snap.surface_points_world[idx] = Eigen::Vector3d(0.01 * i, 0, 0.05);
  }
  for (int f = 0; f < se::kNumFingers; ++f) {
    snap.beam_directions_world[static_cast<std::size_t>(f)] = Eigen::Vector3d::UnitZ();
  }
  return snap;
}

TEST(BuildBeamMarkers, OneArrowPerSensorWithFrameId) {
  builtin_interfaces::msg::Time stamp;
  const auto ma = viz::BuildBeamMarkers(MakeSnapshot(true), stamp, "tool0");

  ASSERT_EQ(ma.markers.size(), static_cast<std::size_t>(se::kTotalSensors));
  for (const auto& m : ma.markers) {
    EXPECT_EQ(m.type, visualization_msgs::msg::Marker::ARROW);
    EXPECT_EQ(m.action, visualization_msgs::msg::Marker::ADD);
    EXPECT_EQ(m.ns, "tof_beams");
    EXPECT_EQ(m.header.frame_id, "tool0");
    ASSERT_EQ(m.points.size(), 2U);
  }
  // Valid reading → green arrow.
  EXPECT_FLOAT_EQ(ma.markers.front().color.g, 1.0F);
}

TEST(BuildBeamMarkers, InvalidReadingDrawnGrey) {
  builtin_interfaces::msg::Time stamp;
  const auto ma = viz::BuildBeamMarkers(MakeSnapshot(false), stamp);
  ASSERT_EQ(ma.markers.size(), static_cast<std::size_t>(se::kTotalSensors));
  // Invalid → grey (0.5, 0.5, 0.5).
  EXPECT_FLOAT_EQ(ma.markers.front().color.r, 0.5F);
  EXPECT_FLOAT_EQ(ma.markers.front().color.g, 0.5F);
  EXPECT_FLOAT_EQ(ma.markers.front().color.b, 0.5F);
}

// ── BuildCurvatureTextMarkers ───────────────────────────────────────────────

TEST(BuildCurvatureTextMarkers, OneTextPerFingerValidAndInvalid) {
  se::ToFSnapshot snap = MakeSnapshot(true);
  snap.curvature_valid = {true, false, true};
  snap.local_curvatures = {10.0, 0.0, 0.0005};  // f0 normal, f2 ~flat
  builtin_interfaces::msg::Time stamp;
  const auto ma = viz::BuildCurvatureTextMarkers(snap, stamp, "tool0");

  ASSERT_EQ(ma.markers.size(), static_cast<std::size_t>(se::kNumFingers));
  for (const auto& m : ma.markers) {
    EXPECT_EQ(m.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    EXPECT_EQ(m.ns, "curvature_text");
    EXPECT_FALSE(m.text.empty());
  }
  EXPECT_NE(ma.markers[0].text.find("k="), std::string::npos);   // valid
  EXPECT_NE(ma.markers[1].text.find("N/A"), std::string::npos);  // invalid
  EXPECT_NE(ma.markers[2].text.find("flat"), std::string::npos);  // |κ| ≤ 1e-3
}

// ── BuildPointCloud2 ────────────────────────────────────────────────────────

TEST(BuildPointCloud2, LayoutAndDataSize) {
  std::vector<se::PointWithNormal> pts(4);
  for (auto& p : pts) {
    p.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    p.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
    p.curvature = 0.5;
  }
  builtin_interfaces::msg::Time stamp;
  const auto cloud = viz::BuildPointCloud2(pts, stamp, "tool0");

  EXPECT_EQ(cloud.height, 1U);
  EXPECT_EQ(cloud.width, 4U);
  EXPECT_EQ(cloud.point_step, 28U);          // 7 × float32
  EXPECT_EQ(cloud.fields.size(), 7U);
  EXPECT_EQ(cloud.data.size(), 28U * 4U);
  EXPECT_EQ(cloud.header.frame_id, "tool0");
  EXPECT_TRUE(cloud.is_dense);
}

// ── BuildPrimitiveMarkers (3 markers: primitive + axis + text) ──────────────

TEST(BuildPrimitiveMarkers, SphereAddsSphereDeletesAxisTextLabelled) {
  se::ShapeEstimate e;
  e.type = se::ShapeType::kSphere;
  e.confidence = 0.9;
  e.radius = 0.05;
  builtin_interfaces::msg::Time stamp;
  const auto ma = viz::BuildPrimitiveMarkers(e, stamp, "tool0");

  ASSERT_EQ(ma.markers.size(), 3U);
  EXPECT_EQ(ma.markers[0].type, visualization_msgs::msg::Marker::SPHERE);
  EXPECT_EQ(ma.markers[0].action, visualization_msgs::msg::Marker::ADD);
  EXPECT_DOUBLE_EQ(ma.markers[0].scale.x, 0.10);  // 2 × radius
  EXPECT_EQ(ma.markers[1].action, visualization_msgs::msg::Marker::DELETE);  // axis: cyl/plane only
  EXPECT_EQ(ma.markers[2].type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_NE(ma.markers[2].text.find("SPHERE"), std::string::npos);
}

TEST(BuildPrimitiveMarkers, CylinderAddsAxisArrow) {
  se::ShapeEstimate e;
  e.type = se::ShapeType::kCylinder;
  e.radius = 0.03;
  e.axis = Eigen::Vector3d(1.0, 0.0, 0.0);  // non-z → AxisToQuaternion general path
  builtin_interfaces::msg::Time stamp;
  const auto ma = viz::BuildPrimitiveMarkers(e, stamp);

  ASSERT_EQ(ma.markers.size(), 3U);
  EXPECT_EQ(ma.markers[0].type, visualization_msgs::msg::Marker::CYLINDER);
  EXPECT_EQ(ma.markers[1].type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_EQ(ma.markers[1].action, visualization_msgs::msg::Marker::ADD);
}

TEST(BuildPrimitiveMarkers, BoxAndPlaneUseCube) {
  builtin_interfaces::msg::Time stamp;

  se::ShapeEstimate box;
  box.type = se::ShapeType::kBox;
  box.dimensions = Eigen::Vector3d(0.1, 0.2, 0.3);  // axis defaults to +z → identity quat branch
  const auto ma_box = viz::BuildPrimitiveMarkers(box, stamp);
  EXPECT_EQ(ma_box.markers[0].type, visualization_msgs::msg::Marker::CUBE);
  EXPECT_DOUBLE_EQ(ma_box.markers[0].scale.y, 0.2);

  se::ShapeEstimate plane;
  plane.type = se::ShapeType::kPlane;
  plane.axis = Eigen::Vector3d(0.0, 0.0, -1.0);  // -z → (a+z)≈0 branch in AxisToQuaternion
  const auto ma_plane = viz::BuildPrimitiveMarkers(plane, stamp);
  EXPECT_EQ(ma_plane.markers[0].type, visualization_msgs::msg::Marker::CUBE);
  EXPECT_EQ(ma_plane.markers[1].type, visualization_msgs::msg::Marker::ARROW);  // plane has axis
}

TEST(BuildPrimitiveMarkers, UnknownDeletesPrimitiveAndText) {
  se::ShapeEstimate e;  // type defaults to kUnknown
  builtin_interfaces::msg::Time stamp;
  const auto ma = viz::BuildPrimitiveMarkers(e, stamp);

  ASSERT_EQ(ma.markers.size(), 3U);
  EXPECT_EQ(ma.markers[0].action, visualization_msgs::msg::Marker::DELETE);
  EXPECT_EQ(ma.markers[2].action, visualization_msgs::msg::Marker::DELETE);
}

}  // namespace
