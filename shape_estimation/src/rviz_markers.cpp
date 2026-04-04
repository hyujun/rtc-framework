#include "shape_estimation/rviz_markers.hpp"

#include <cmath>
#include <cstring>
#include <sstream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <sensor_msgs/msg/point_field.hpp>
#pragma GCC diagnostic pop

namespace shape_estimation::viz {

// ── 유틸 ─────────────────────────────────────────────────────────────────────

static std_msgs::msg::ColorRGBA MakeColor(float r, float g, float b, float a) {
  std_msgs::msg::ColorRGBA c;
  c.r = r; c.g = g; c.b = b; c.a = a;
  return c;
}

static geometry_msgs::msg::Point ToPoint(const Eigen::Vector3d& v) {
  geometry_msgs::msg::Point p;
  p.x = v.x(); p.y = v.y(); p.z = v.z();
  return p;
}

// axis 방향을 z축으로 하는 quaternion 계산
static geometry_msgs::msg::Quaternion AxisToQuaternion(const Eigen::Vector3d& axis) {
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d a = axis.normalized();

  geometry_msgs::msg::Quaternion q;
  if ((a - z).norm() < 1e-6) {
    q.w = 1.0; q.x = 0.0; q.y = 0.0; q.z = 0.0;
  } else if ((a + z).norm() < 1e-6) {
    q.w = 0.0; q.x = 1.0; q.y = 0.0; q.z = 0.0;
  } else {
    const Eigen::Vector3d v = z.cross(a);
    const double s = v.norm();
    const double c = z.dot(a);
    const double h = 1.0 / (1.0 + c);
    const Eigen::Matrix3d vx = (Eigen::Matrix3d() <<
        0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0).finished();
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + vx + vx * vx * h;
    const Eigen::Quaterniond eq(R);
    q.w = eq.w(); q.x = eq.x(); q.y = eq.y(); q.z = eq.z();
    (void)s;
  }
  return q;
}

// ── BuildBeamMarkers ─────────────────────────────────────────────────────────

visualization_msgs::msg::MarkerArray BuildBeamMarkers(
    const ToFSnapshot& snapshot,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id) {
  visualization_msgs::msg::MarkerArray ma;
  ma.markers.reserve(kTotalSensors);

  for (int i = 0; i < kTotalSensors; ++i) {
    const auto idx = static_cast<size_t>(i);
    const int finger_idx = i / kSensorsPerFinger;
    const auto& reading = snapshot.readings[idx];

    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "tof_beams";
    m.id = i;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.points.resize(2);
    m.points[0] = ToPoint(snapshot.sensor_positions_world[idx]);

    if (reading.valid) {
      m.points[1] = ToPoint(snapshot.surface_points_world[idx]);
      m.color = MakeColor(0.0F, 1.0F, 0.0F, 0.8F);  // 초록
    } else {
      // 무효: max range로 표시
      const auto& beam = snapshot.beam_directions_world[static_cast<size_t>(finger_idx)];
      const Eigen::Vector3d end = snapshot.sensor_positions_world[idx] + kTofMaxRange * beam;
      m.points[1] = ToPoint(end);
      m.color = MakeColor(0.5F, 0.5F, 0.5F, 0.3F);  // 회색 반투명
    }

    m.scale.x = 0.001;  // shaft diameter 1mm
    m.scale.y = 0.002;  // head diameter 2mm
    m.scale.z = 0.0;
    m.lifetime.sec = 0;
    m.lifetime.nanosec = 150'000'000;  // 150ms

    ma.markers.push_back(std::move(m));
  }

  return ma;
}

// ── BuildCurvatureTextMarkers ────────────────────────────────────────────────

visualization_msgs::msg::MarkerArray BuildCurvatureTextMarkers(
    const ToFSnapshot& snapshot,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id) {
  visualization_msgs::msg::MarkerArray ma;
  ma.markers.reserve(kNumFingers);

  for (int k = 0; k < kNumFingers; ++k) {
    const auto idx = static_cast<size_t>(k);
    // tip 위치 = 두 센서 위치의 중간점
    const size_t a_idx = static_cast<size_t>(k * kSensorsPerFinger);
    const Eigen::Vector3d tip_pos =
        (snapshot.sensor_positions_world[a_idx] +
         snapshot.sensor_positions_world[a_idx + 1]) / 2.0;

    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "curvature_text";
    m.id = k;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position = ToPoint(tip_pos + Eigen::Vector3d(0, 0, 0.02));
    m.scale.z = 0.010;

    if (snapshot.curvature_valid[idx]) {
      const double kappa = snapshot.local_curvatures[idx];
      std::ostringstream oss;
      oss.precision(1);
      oss << std::fixed;
      if (std::abs(kappa) > 1e-3) {
        oss << "k=" << kappa << " (r=" << static_cast<int>(1000.0 / std::abs(kappa)) << "mm)";
      } else {
        oss << "k=0.0 (flat)";
      }
      m.text = oss.str();
      m.color = CurvatureToColor(kappa);
    } else {
      m.text = "k=N/A";
      m.color = MakeColor(0.5F, 0.5F, 0.5F, 0.5F);
    }

    m.lifetime.sec = 0;
    m.lifetime.nanosec = 150'000'000;

    ma.markers.push_back(std::move(m));
  }

  return ma;
}

// ── BuildPointCloud2 ─────────────────────────────────────────────────────────

sensor_msgs::msg::PointCloud2 BuildPointCloud2(
    const std::vector<PointWithNormal>& points,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = stamp;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  // 필드 정의: x, y, z, normal_x, normal_y, normal_z, curvature
  const uint32_t point_step = 28;  // 7 * sizeof(float32)
  cloud.point_step = point_step;
  cloud.row_step = point_step * cloud.width;

  auto add_field = [&cloud](const std::string& name, uint32_t offset) {
    sensor_msgs::msg::PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    cloud.fields.push_back(f);
  };

  add_field("x", 0);
  add_field("y", 4);
  add_field("z", 8);
  add_field("normal_x", 12);
  add_field("normal_y", 16);
  add_field("normal_z", 20);
  add_field("curvature", 24);

  cloud.data.resize(static_cast<size_t>(cloud.row_step));

  for (size_t i = 0; i < points.size(); ++i) {
    const size_t offset = i * point_step;
    const auto& p = points[i];

    const auto write_float = [&cloud, offset](size_t field_offset, float val) {
      std::memcpy(&cloud.data[offset + field_offset], &val, sizeof(float));
    };

    write_float(0, static_cast<float>(p.position.x()));
    write_float(4, static_cast<float>(p.position.y()));
    write_float(8, static_cast<float>(p.position.z()));
    write_float(12, static_cast<float>(p.normal.x()));
    write_float(16, static_cast<float>(p.normal.y()));
    write_float(20, static_cast<float>(p.normal.z()));
    write_float(24, static_cast<float>(p.curvature));
  }

  return cloud;
}

// ── BuildPrimitiveMarkers ────────────────────────────────────────────────────

visualization_msgs::msg::MarkerArray BuildPrimitiveMarkers(
    const ShapeEstimate& estimate,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id) {
  visualization_msgs::msg::MarkerArray ma;

  // Marker 0 — Primitive Shape
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "shape_primitive";
    m.id = 0;

    if (estimate.type == ShapeType::kUnknown) {
      m.action = visualization_msgs::msg::Marker::DELETE;
      ma.markers.push_back(std::move(m));
    } else {
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position = ToPoint(estimate.center);
      m.color = ConfidenceToColor(estimate.confidence);

      switch (estimate.type) {
        case ShapeType::kSphere:
          m.type = visualization_msgs::msg::Marker::SPHERE;
          m.scale.x = m.scale.y = m.scale.z = 2.0 * estimate.radius;
          m.pose.orientation.w = 1.0;
          break;

        case ShapeType::kCylinder:
          m.type = visualization_msgs::msg::Marker::CYLINDER;
          m.scale.x = m.scale.y = 2.0 * estimate.radius;
          m.scale.z = 0.10;  // 포인트 클라우드 extent로 대체 가능
          m.pose.orientation = AxisToQuaternion(estimate.axis);
          break;

        case ShapeType::kPlane:
          m.type = visualization_msgs::msg::Marker::CUBE;
          m.scale.x = m.scale.y = 0.15;
          m.scale.z = 0.002;
          m.pose.orientation = AxisToQuaternion(estimate.axis);
          break;

        case ShapeType::kBox:
          m.type = visualization_msgs::msg::Marker::CUBE;
          m.scale.x = estimate.dimensions.x();
          m.scale.y = estimate.dimensions.y();
          m.scale.z = estimate.dimensions.z();
          m.pose.orientation = AxisToQuaternion(estimate.axis);
          break;

        default:
          break;
      }

      ma.markers.push_back(std::move(m));
    }
  }

  // Marker 1 — 축 방향 화살표 (cylinder/plane)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "shape_axis";
    m.id = 1;

    if (estimate.type == ShapeType::kCylinder || estimate.type == ShapeType::kPlane) {
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.points.resize(2);
      m.points[0] = ToPoint(estimate.center - 0.05 * estimate.axis);
      m.points[1] = ToPoint(estimate.center + 0.05 * estimate.axis);
      m.scale.x = 0.003;  // shaft diameter
      m.scale.y = 0.006;  // head diameter
      m.color = MakeColor(1.0F, 1.0F, 1.0F, 0.8F);
    } else {
      m.action = visualization_msgs::msg::Marker::DELETE;
    }
    ma.markers.push_back(std::move(m));
  }

  // Marker 2 — Confidence 텍스트
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "shape_text";
    m.id = 2;

    if (estimate.type != ShapeType::kUnknown) {
      m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position = ToPoint(estimate.center + Eigen::Vector3d(0, 0, 0.05));
      m.scale.z = 0.015;
      m.color = MakeColor(1.0F, 1.0F, 1.0F, 1.0F);

      std::ostringstream oss;
      oss << ShapeTypeToString(estimate.type)
          << " (conf: " << static_cast<int>(estimate.confidence * 100) << "%";
      if (estimate.type == ShapeType::kSphere || estimate.type == ShapeType::kCylinder) {
        oss << ", r: " << static_cast<int>(estimate.radius * 1000) << "mm";
      }
      oss << ")";
      m.text = oss.str();
    } else {
      m.action = visualization_msgs::msg::Marker::DELETE;
    }
    ma.markers.push_back(std::move(m));
  }

  return ma;
}

// ── BuildDeleteAllMarkers ────────────────────────────────────────────────────

visualization_msgs::msg::MarkerArray BuildExploreStatusMarkers(
    ExplorePhase phase,
    const ExplorationStats& stats,
    const ShapeEstimate& current_estimate,
    const std::array<double, 6>& current_ee_pose,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id) {
  visualization_msgs::msg::MarkerArray ma;

  // 현재 EE 위치에 phase 텍스트 표시
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = "explore_status";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  m.action = visualization_msgs::msg::Marker::ADD;

  m.pose.position.x = current_ee_pose[0];
  m.pose.position.y = current_ee_pose[1];
  m.pose.position.z = current_ee_pose[2] + 0.08;
  m.scale.z = 0.015;

  std::ostringstream oss;
  oss << "[" << ExplorePhaseToString(phase) << "] "
      << std::fixed;
  oss.precision(1);
  oss << stats.elapsed_sec << "s"
      << " pts=" << stats.total_snapshots
      << " goals=" << stats.goals_sent;
  if (current_estimate.type != ShapeType::kUnknown) {
    oss << " | " << ShapeTypeToString(current_estimate.type)
        << " " << static_cast<int>(current_estimate.confidence * 100) << "%";
  }
  m.text = oss.str();

  // Phase에 따른 색상
  switch (phase) {
    case ExplorePhase::kApproach:
    case ExplorePhase::kServo:
      m.color = MakeColor(1.0F, 1.0F, 0.0F, 1.0F);  // 노랑
      break;
    case ExplorePhase::kSweepX:
    case ExplorePhase::kSweepY:
    case ExplorePhase::kTilt:
      m.color = MakeColor(0.0F, 0.8F, 1.0F, 1.0F);  // 하늘색
      break;
    case ExplorePhase::kEvaluate:
      m.color = MakeColor(1.0F, 0.5F, 0.0F, 1.0F);  // 주황
      break;
    case ExplorePhase::kSucceeded:
      m.color = MakeColor(0.0F, 1.0F, 0.0F, 1.0F);  // 초록
      break;
    case ExplorePhase::kFailed:
    case ExplorePhase::kAborted:
      m.color = MakeColor(1.0F, 0.0F, 0.0F, 1.0F);  // 빨강
      break;
    default:
      m.color = MakeColor(0.5F, 0.5F, 0.5F, 1.0F);  // 회색
      break;
  }

  m.lifetime.sec = 0;
  m.lifetime.nanosec = 300'000'000;  // 300ms
  ma.markers.push_back(std::move(m));

  return ma;
}

// ── BuildTargetGoalMarker ─────��──────────────────────────────────��───────────

visualization_msgs::msg::MarkerArray BuildTargetGoalMarker(
    const TaskSpaceGoal& goal,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id) {
  visualization_msgs::msg::MarkerArray ma;

  if (!goal.valid) {
    visualization_msgs::msg::Marker m;
    m.ns = "target_goal";
    m.id = 0;
    m.action = visualization_msgs::msg::Marker::DELETE;
    ma.markers.push_back(std::move(m));
    return ma;
  }

  // 목표 위치에 작은 구
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = "target_goal";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;

  m.pose.position.x = goal.pose[0];
  m.pose.position.y = goal.pose[1];
  m.pose.position.z = goal.pose[2];
  m.pose.orientation.w = 1.0;

  m.scale.x = m.scale.y = m.scale.z = 0.008;  // 8mm 구
  m.color = MakeColor(1.0F, 0.0F, 1.0F, 0.8F);  // 보라색
  m.lifetime.sec = 0;
  m.lifetime.nanosec = 300'000'000;
  ma.markers.push_back(std::move(m));

  return ma;
}

// ── BuildProtuberanceMarkers ────────────────────────────────────────────────

visualization_msgs::msg::MarkerArray BuildProtuberanceMarkers(
    const ProtuberanceResult& result,
    const ShapeEstimate& /*primitive*/,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id) {
  visualization_msgs::msg::MarkerArray ma;

  if (!result.detected || result.protuberances.empty()) {
    // DELETE 마커
    for (int i = 0; i < 4; ++i) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = frame_id;
      m.ns = (i == 0) ? "protuberance" :
             (i == 1) ? "protuberance_dir" :
             (i == 2) ? "protuberance_text" : "protuberance_gap";
      m.id = 0;
      m.action = visualization_msgs::msg::Marker::DELETE;
      ma.markers.push_back(std::move(m));
    }
    return ma;
  }

  // 가장 신뢰도 높은 1개만 시각화
  const auto& best = result.protuberances.front();

  // Marker 0: 돌출부 위치 (구)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    m.ns = "protuberance";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = ToPoint(best.centroid);
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.010;  // 10mm 구

    if (best.confidence > 0.7) {
      m.color = MakeColor(1.0F, 0.0F, 0.0F, 0.8F);  // 빨강
    } else if (best.confidence > 0.4) {
      m.color = MakeColor(1.0F, 0.5F, 0.0F, 0.8F);  // 주황
    } else {
      m.color = MakeColor(1.0F, 1.0F, 0.0F, 0.8F);  // 노랑
    }

    ma.markers.push_back(std::move(m));
  }

  // Marker 1: 돌출 방향 화살표
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    m.ns = "protuberance_dir";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.points.push_back(ToPoint(best.centroid));
    m.points.push_back(ToPoint(best.centroid + 0.030 * best.direction));
    m.scale.x = 0.003;  // shaft diameter
    m.scale.y = 0.006;  // head diameter
    m.color = MakeColor(1.0F, 0.0F, 0.0F, 0.8F);

    ma.markers.push_back(std::move(m));
  }

  // Marker 2: 텍스트
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    m.ns = "protuberance_text";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = ToPoint(
        best.centroid + Eigen::Vector3d(0, 0, 0.02));
    m.pose.orientation.w = 1.0;
    m.scale.z = 0.010;

    std::ostringstream oss;
    oss << "HANDLE? depth=" << static_cast<int>(best.protrusion_depth * 1000.0)
        << "mm ext=" << static_cast<int>(best.extent_along_surface * 1000.0)
        << "mm (conf:" << static_cast<int>(best.confidence * 100.0) << "%)";
    if (best.has_gap) {
      oss << " +GAP";
    }
    m.text = oss.str();
    m.color = MakeColor(1.0F, 1.0F, 1.0F, 1.0F);

    ma.markers.push_back(std::move(m));
  }

  // Marker 3: Gap 위치 (gap 동반 시만)
  if (best.has_gap) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    m.ns = "protuberance_gap";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = ToPoint(best.centroid);
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.015;  // 15mm (gap 영역)
    m.color = MakeColor(1.0F, 0.5F, 0.0F, 0.3F);  // 반투명 주황

    ma.markers.push_back(std::move(m));
  } else {
    visualization_msgs::msg::Marker m;
    m.ns = "protuberance_gap";
    m.id = 0;
    m.action = visualization_msgs::msg::Marker::DELETE;
    ma.markers.push_back(std::move(m));
  }

  return ma;
}

// ── BuildDeleteAllMarkers ────────���───────────────────────────���───────────────

visualization_msgs::msg::MarkerArray BuildDeleteAllMarkers() {
  visualization_msgs::msg::MarkerArray ma;

  const std::array<std::pair<std::string, int>, 11> ns_ids = {{
      {"tof_beams", kTotalSensors},
      {"curvature_text", kNumFingers},
      {"shape_primitive", 1},
      {"shape_axis", 1},
      {"shape_text", 1},
      {"explore_status", 1},
      {"target_goal", 1},
      {"protuberance", 1},
      {"protuberance_dir", 1},
      {"protuberance_text", 1},
      {"protuberance_gap", 1},
  }};

  for (const auto& [ns, count] : ns_ids) {
    for (int i = 0; i < count; ++i) {
      visualization_msgs::msg::Marker m;
      m.ns = ns;
      m.id = i;
      m.action = visualization_msgs::msg::Marker::DELETE;
      ma.markers.push_back(std::move(m));
    }
  }

  return ma;
}

// ── Color 유틸 ───────────────────────────────────────────────────────────────

std_msgs::msg::ColorRGBA ConfidenceToColor(double confidence, double alpha) {
  const auto a = static_cast<float>(alpha);
  if (confidence > 0.8) {
    return MakeColor(0.0F, 1.0F, 0.0F, a);        // 초록
  } else if (confidence > 0.5) {
    return MakeColor(1.0F, 1.0F, 0.0F, a);        // 노랑
  } else if (confidence > 0.2) {
    return MakeColor(1.0F, 0.5F, 0.0F, a);        // 주황
  } else {
    return MakeColor(1.0F, 0.0F, 0.0F, a);        // 빨강
  }
}

std_msgs::msg::ColorRGBA CurvatureToColor(double curvature, double alpha) {
  const double abs_k = std::abs(curvature);
  const auto a = static_cast<float>(alpha);
  if (abs_k < 5.0) {
    return MakeColor(0.3F, 0.3F, 1.0F, a);        // 파랑 (flat)
  } else if (abs_k < 20.0) {
    return MakeColor(0.0F, 1.0F, 0.0F, a);        // 초록 (medium)
  } else {
    return MakeColor(1.0F, 0.0F, 0.0F, a);        // 빨강 (high curvature)
  }
}

}  // namespace shape_estimation::viz
