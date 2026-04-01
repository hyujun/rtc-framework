#include "shape_estimation/msg_conversions.hpp"

#include <cmath>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Geometry>
#pragma GCC diagnostic pop

namespace shape_estimation {

// ── 내부 유틸 ────────────────────────────────────────────────────────────────

static Eigen::Quaterniond QuaternionFromMsg(const geometry_msgs::msg::Quaternion& q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

static Eigen::Vector3d PositionFromMsg(const geometry_msgs::msg::Point& p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}

// ── ConvertFromMsg ───────────────────────────────────────────────────────────

ToFSnapshot ConvertFromMsg(const shape_estimation_msgs::msg::ToFSnapshot& msg) {
  ToFSnapshot snapshot;

  // 타임스탬프
  const auto sec = static_cast<uint64_t>(
      std::max(static_cast<int32_t>(0), msg.stamp.sec));
  snapshot.timestamp_ns = sec * 1'000'000'000ULL +
      static_cast<uint64_t>(msg.stamp.nanosec);

  // 6개 ToF readings
  for (int i = 0; i < kTotalSensors; ++i) {
    const auto idx = static_cast<size_t>(i);
    snapshot.readings[idx].distance_m = msg.distances[idx];
    snapshot.readings[idx].valid = msg.valid[idx] &&
        msg.distances[idx] >= kTofMinRange &&
        msg.distances[idx] <= kTofMaxRange;
  }

  // 3개 tip_pose로부터 센서 위치, 빔 방향, 표면점 계산
  for (int f = 0; f < kNumFingers; ++f) {
    const auto fi = static_cast<size_t>(f);
    const auto& pose = msg.tip_poses[fi];

    const Eigen::Vector3d tip_pos = PositionFromMsg(pose.position);
    const Eigen::Matrix3d rot = QuaternionFromMsg(pose.orientation).toRotationMatrix();

    // 빔 방향: tip_link +z축
    snapshot.beam_directions_world[fi] = rot * Eigen::Vector3d::UnitZ();

    // 센서 A: x = +2mm offset
    const int idx_a = f * kSensorsPerFinger;
    const int idx_b = idx_a + 1;
    const auto a = static_cast<size_t>(idx_a);
    const auto b = static_cast<size_t>(idx_b);

    snapshot.sensor_positions_world[a] = tip_pos + rot * Eigen::Vector3d(kTofOffsetX, 0, 0);
    snapshot.sensor_positions_world[b] = tip_pos + rot * Eigen::Vector3d(-kTofOffsetX, 0, 0);

    // 표면점: p = sensor_pos + d * beam_dir
    const auto& beam = snapshot.beam_directions_world[fi];

    if (snapshot.readings[a].valid) {
      snapshot.surface_points_world[a] =
          snapshot.sensor_positions_world[a] + snapshot.readings[a].distance_m * beam;
    }
    if (snapshot.readings[b].valid) {
      snapshot.surface_points_world[b] =
          snapshot.sensor_positions_world[b] + snapshot.readings[b].distance_m * beam;
    }

    // 로컬 곡률: κ = 2*Δd / (Δx² + Δd²)
    if (snapshot.readings[a].valid && snapshot.readings[b].valid) {
      const double d_a = snapshot.readings[a].distance_m;
      const double d_b = snapshot.readings[b].distance_m;
      const double delta_d = d_a - d_b;
      const double delta_x_sq = kTofSeparation * kTofSeparation;
      snapshot.local_curvatures[fi] = 2.0 * delta_d / (delta_x_sq + delta_d * delta_d);
      snapshot.curvature_valid[fi] = true;
    } else {
      snapshot.local_curvatures[fi] = 0.0;
      snapshot.curvature_valid[fi] = false;
    }
  }

  return snapshot;
}

// ── ToMsg (ShapeEstimate only) ───────────────────────────────────────────────

shape_estimation_msgs::msg::ShapeEstimate ToMsg(
    const ShapeEstimate& estimate,
    const std_msgs::msg::Header& header) {
  shape_estimation_msgs::msg::ShapeEstimate msg;
  msg.stamp = header.stamp;
  msg.shape_type = static_cast<uint8_t>(estimate.type);
  msg.confidence = estimate.confidence;
  msg.center.x = estimate.center.x();
  msg.center.y = estimate.center.y();
  msg.center.z = estimate.center.z();
  msg.axis.x = estimate.axis.x();
  msg.axis.y = estimate.axis.y();
  msg.axis.z = estimate.axis.z();
  msg.radius = estimate.radius;
  msg.dimensions.x = estimate.dimensions.x();
  msg.dimensions.y = estimate.dimensions.y();
  msg.dimensions.z = estimate.dimensions.z();
  msg.num_points_used = estimate.num_points_used;

  // 곡률 정보 없음
  for (int i = 0; i < 3; ++i) {
    msg.local_curvatures[static_cast<size_t>(i)] = 0.0;
    msg.curvature_valid[static_cast<size_t>(i)] = false;
  }

  return msg;
}

// ── ToMsg (ShapeEstimate + 곡률) ─────────────────────────────────────────────

shape_estimation_msgs::msg::ShapeEstimate ToMsg(
    const ShapeEstimate& estimate,
    const ToFSnapshot& snapshot,
    const std_msgs::msg::Header& header) {
  auto msg = ToMsg(estimate, header);

  for (int i = 0; i < kNumFingers; ++i) {
    const auto idx = static_cast<size_t>(i);
    msg.local_curvatures[idx] = snapshot.local_curvatures[idx];
    msg.curvature_valid[idx] = snapshot.curvature_valid[idx];
  }

  return msg;
}

}  // namespace shape_estimation
