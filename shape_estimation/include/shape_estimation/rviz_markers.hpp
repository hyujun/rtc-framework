#pragma once

#include "shape_estimation/tof_shape_types.hpp"

#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#pragma GCC diagnostic pop

namespace shape_estimation::viz {

/// 6개 ToF 빔을 MarkerArray로 변환
[[nodiscard]] visualization_msgs::msg::MarkerArray BuildBeamMarkers(
    const ToFSnapshot& snapshot,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id = "base_link");

/// 3개 곡률 텍스트를 MarkerArray로 변환
[[nodiscard]] visualization_msgs::msg::MarkerArray BuildCurvatureTextMarkers(
    const ToFSnapshot& snapshot,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id = "base_link");

/// VoxelPointCloud → PointCloud2 변환
[[nodiscard]] sensor_msgs::msg::PointCloud2 BuildPointCloud2(
    const std::vector<PointWithNormal>& points,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id = "base_link");

/// ShapeEstimate → Primitive + Axis + Text MarkerArray 변환
[[nodiscard]] visualization_msgs::msg::MarkerArray BuildPrimitiveMarkers(
    const ShapeEstimate& estimate,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id = "base_link");

/// 모든 ns의 marker를 DELETE하는 MarkerArray 생성
[[nodiscard]] visualization_msgs::msg::MarkerArray BuildDeleteAllMarkers();

/// Confidence → RGBA color 변환 (초록→노랑→주황→빨강)
[[nodiscard]] std_msgs::msg::ColorRGBA ConfidenceToColor(double confidence, double alpha = 0.4);

/// Curvature → RGBA color 변환 (파랑→초록→빨강)
[[nodiscard]] std_msgs::msg::ColorRGBA CurvatureToColor(double curvature, double alpha = 0.8);

}  // namespace shape_estimation::viz
