#pragma once

#include "shape_estimation/tof_shape_types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <shape_estimation_msgs/msg/shape_estimate.hpp>
#include <shape_estimation_msgs/msg/to_f_snapshot.hpp>
#include <std_msgs/msg/header.hpp>
#pragma GCC diagnostic pop

namespace shape_estimation {

/// ROS msg → 내부 ToFSnapshot 변환 (센서 위치/표면점/곡률 계산 포함)
[[nodiscard]] ToFSnapshot ConvertFromMsg(
    const shape_estimation_msgs::msg::ToFSnapshot& msg);

/// 내부 ShapeEstimate → ROS msg 변환
[[nodiscard]] shape_estimation_msgs::msg::ShapeEstimate ToMsg(
    const ShapeEstimate& estimate,
    const std_msgs::msg::Header& header);

/// 내부 ShapeEstimate → ROS msg 변환 (곡률 정보 포함)
[[nodiscard]] shape_estimation_msgs::msg::ShapeEstimate ToMsg(
    const ShapeEstimate& estimate,
    const ToFSnapshot& snapshot,
    const std_msgs::msg::Header& header);

}  // namespace shape_estimation
