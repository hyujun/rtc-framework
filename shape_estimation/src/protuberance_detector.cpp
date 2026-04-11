#include "shape_estimation/protuberance_detector.hpp"
#include "shape_estimation/shape_logging.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <rclcpp/logging.hpp>

namespace shape_estimation {

namespace {
auto logger() { return ::rtc::shape::logging::ProtusLogger(); }
}  // namespace

ProtuberanceDetector::ProtuberanceDetector(const ProtuberanceConfig& config)
    : config_(config) {}

void ProtuberanceDetector::UpdateConfig(const ProtuberanceConfig& config) {
  config_ = config;
}

// ═════════════════════════════════════════════════════════════════════════════
// Detect (메인 인터페이스)
// ═════════════════════════════════════════════════════════════════════════════

ProtuberanceResult ProtuberanceDetector::Detect(
    const ShapeEstimate& fitted_primitive,
    const std::vector<PointWithNormal>& points,
    const std::vector<ToFSnapshot>& recent_snapshots) const {
  ProtuberanceResult result;

  // Unknown 타입이면 탐지 불가
  if (fitted_primitive.type == ShapeType::kUnknown || points.empty()) {
    RCLCPP_DEBUG(logger(), "돌출 탐지 스킵: %s",
                 points.empty() ? "포인트 없음" : "primitive=Unknown");
    return result;
  }

  // Step 1: Signed residual 계산
  auto residuals = ComputeSignedResiduals(fitted_primitive, points);

  // Step 2: 음의 잔차 클러스터링
  auto clusters = ClusterNegativeResiduals(residuals);

  if (clusters.empty()) {
    // 돌출 없음 — base residual RMS 계산
    double sum_sq = 0.0;
    for (const auto& r : residuals) {
      sum_sq += r.signed_residual * r.signed_residual;
    }
    result.base_residual_rms =
        residuals.empty() ? 0.0 : std::sqrt(sum_sq / static_cast<double>(residuals.size()));
    RCLCPP_DEBUG(logger(), "돌출 없음: base_residual_rms=%.4f, points=%zu",
                 result.base_residual_rms, points.size());
    return result;
  }

  // Step 3: Gap 분석
  auto gaps = DetectGaps(recent_snapshots);

  // Step 4: 각 클러스터를 Protuberance로 변환
  for (const auto& cluster : clusters) {
    auto prot = BuildProtuberance(cluster, residuals, fitted_primitive, gaps);
    result.protuberances.push_back(prot);
  }

  // 신뢰도 기준 내림차순 정렬
  std::sort(result.protuberances.begin(), result.protuberances.end(),
            [](const Protuberance& a, const Protuberance& b) {
              return a.confidence > b.confidence;
            });

  result.detected = true;

  // base residual RMS: 클러스터에 속하지 않는 포인트의 잔차
  std::vector<bool> in_cluster(residuals.size(), false);
  for (const auto& cluster : clusters) {
    for (uint32_t idx : cluster) {
      if (idx < in_cluster.size()) {
        in_cluster[idx] = true;
      }
    }
  }
  double sum_sq = 0.0;
  uint32_t count = 0;
  for (size_t i = 0; i < residuals.size(); ++i) {
    if (!in_cluster[i]) {
      sum_sq += residuals[i].signed_residual * residuals[i].signed_residual;
      ++count;
    }
  }
  result.base_residual_rms =
      count > 0 ? std::sqrt(sum_sq / static_cast<double>(count)) : 0.0;

  RCLCPP_DEBUG(logger(),
               "돌출 탐지 완료: %zu개 protuberance, "
               "base_rms=%.4f, gaps=%zu",
               result.protuberances.size(), result.base_residual_rms,
               gaps.size());

  return result;
}

// ═════════════════════════════════════════════════════════════════════════════
// Step 1: Signed Residual 계산
// ═════════════════════════════════════════════════════════════════════════════

double ProtuberanceDetector::SignedDistanceFromPrimitive(
    const Eigen::Vector3d& point,
    const ShapeEstimate& primitive) {
  switch (primitive.type) {
    case ShapeType::kSphere: {
      // 양수 = 구 내부 (정상), 음수 = 구 외부 (돌출)
      return primitive.radius - (point - primitive.center).norm();
    }
    case ShapeType::kCylinder: {
      // 축 직교 방향 거리
      const Eigen::Vector3d v = point - primitive.center;
      const double along_axis = v.dot(primitive.axis);
      const Eigen::Vector3d radial = v - along_axis * primitive.axis;
      return primitive.radius - radial.norm();
    }
    case ShapeType::kPlane: {
      // 법선 방향 signed distance (양수 = 법선 방향, 음수 = 반대)
      // 돌출 = 법선 방향으로 표면 위에 있음 → signed_dist > 0이면 돌출
      // 부호 반전하여 양수=내부 규약 유지
      return -(point - primitive.center).dot(primitive.axis);
    }
    case ShapeType::kBox: {
      // 축 정렬 근사: center로부터의 각 축 거리 - half_extent
      // 가장 큰 침투(양수 = 내부) 반환
      const Eigen::Vector3d half = primitive.dimensions * 0.5;
      const Eigen::Vector3d delta = (point - primitive.center).cwiseAbs();
      // 각 축에서의 부호 거리 (양수 = 내부)
      const Eigen::Vector3d dists = half - delta;
      return dists.minCoeff();  // 가장 작은 margin = 표면까지 거리
    }
    default:
      return 0.0;
  }
}

std::vector<ProtuberanceDetector::PointResidual>
ProtuberanceDetector::ComputeSignedResiduals(
    const ShapeEstimate& primitive,
    const std::vector<PointWithNormal>& points) const {
  std::vector<PointResidual> results;
  results.reserve(points.size());

  for (uint32_t i = 0; i < static_cast<uint32_t>(points.size()); ++i) {
    const double signed_dist =
        SignedDistanceFromPrimitive(points[i].position, primitive);
    // signed_dist < 0 → primitive 외부 (돌출) → residual < 0
    results.push_back(
        PointResidual{i, signed_dist, points[i].position});
  }

  return results;
}

// ═════════════════════════════════════════════════════════════════════════════
// Step 2: 음의 잔차 클러스터링 (Union-Find)
// ═════════════════════════════════════════════════════════════════════════════

std::vector<std::vector<uint32_t>>
ProtuberanceDetector::ClusterNegativeResiduals(
    const std::vector<PointResidual>& residuals) const {
  // threshold 이하인 포인트 추출
  std::vector<uint32_t> negative_indices;
  for (size_t i = 0; i < residuals.size(); ++i) {
    if (residuals[i].signed_residual < config_.residual_threshold) {
      negative_indices.push_back(static_cast<uint32_t>(i));
    }
  }

  const auto n = negative_indices.size();
  if (n < config_.min_cluster_points) {
    RCLCPP_DEBUG(logger(),
                 "클러스터링: 음의 잔차 포인트 부족 (%zu < %u)",
                 n, config_.min_cluster_points);
    return {};
  }

  // Union-Find
  std::vector<uint32_t> parent(n);
  std::iota(parent.begin(), parent.end(), 0U);

  // Find with path compression
  auto find = [&parent](uint32_t x) -> uint32_t {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  };

  auto unite = [&parent, &find](uint32_t a, uint32_t b) {
    const uint32_t ra = find(a);
    const uint32_t rb = find(b);
    if (ra != rb) {
      parent[ra] = rb;
    }
  };

  // O(N²) brute force — N은 보통 수십개 이하
  const double radius_sq = config_.cluster_radius * config_.cluster_radius;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      const double dist_sq =
          (residuals[negative_indices[i]].position -
           residuals[negative_indices[j]].position)
              .squaredNorm();
      if (dist_sq < radius_sq) {
        unite(static_cast<uint32_t>(i), static_cast<uint32_t>(j));
      }
    }
  }

  // 같은 root를 가진 포인트들을 그룹화
  std::unordered_map<uint32_t, std::vector<uint32_t>> groups;
  for (size_t i = 0; i < n; ++i) {
    groups[find(static_cast<uint32_t>(i))].push_back(
        negative_indices[i]);
  }

  // min_cluster_points 이상인 그룹만 반환
  std::vector<std::vector<uint32_t>> result;
  for (auto& [root, indices] : groups) {
    if (indices.size() >= config_.min_cluster_points) {
      result.push_back(std::move(indices));
    }
  }

  return result;
}

// ═════════════════════════════════════════════════════════════════════════════
// Step 3: Gap 패턴 분석
// ═════════════════════════════════════════════════════════════════════════════

std::vector<ProtuberanceDetector::GapEvent>
ProtuberanceDetector::DetectGaps(
    const std::vector<ToFSnapshot>& snapshots) const {
  if (snapshots.size() < 3) {
    return {};
  }

  std::vector<GapEvent> gaps;

  for (int sensor_idx = 0; sensor_idx < kTotalSensors; ++sensor_idx) {
    const auto si = static_cast<size_t>(sensor_idx);
    bool in_gap = false;
    size_t gap_start = 0;
    double last_valid_distance = 0.0;
    Eigen::Vector3d last_valid_position = Eigen::Vector3d::Zero();

    for (size_t t = 0; t < snapshots.size(); ++t) {
      const auto& reading = snapshots[t].readings[si];

      if (!reading.valid && !in_gap) {
        // gap 시작
        in_gap = true;
        gap_start = t;
      } else if (reading.valid && in_gap) {
        // gap 종료
        in_gap = false;
        const auto gap_duration = static_cast<uint32_t>(t - gap_start);

        if (gap_duration >= config_.min_gap_invalid_count) {
          const double d_after = reading.distance_m;
          const Eigen::Vector3d& pos_after =
              snapshots[t].surface_points_world[si];

          // gap 전후 거리 차이 확인
          if (std::abs(last_valid_distance - d_after) >
              config_.gap_distance_jump) {
            gaps.push_back(GapEvent{
                snapshots[t].timestamp_ns,
                static_cast<uint8_t>(sensor_idx),
                last_valid_position,
                pos_after,
                last_valid_distance,
                d_after,
                gap_duration});
          }
        }
      }

      if (reading.valid) {
        last_valid_distance = reading.distance_m;
        last_valid_position = snapshots[t].surface_points_world[si];
      }
    }
  }

  return gaps;
}

// ═════════════════════════════════════════════════════════════════════════════
// Step 4: 통합
// ═════════════════════════════════════════════════════════════════════════════

Eigen::Vector3d ProtuberanceDetector::PrimitiveOutwardNormal(
    const Eigen::Vector3d& point,
    const ShapeEstimate& primitive) {
  switch (primitive.type) {
    case ShapeType::kSphere: {
      const Eigen::Vector3d diff = point - primitive.center;
      const double norm = diff.norm();
      if (norm > 1e-9) {
        return Eigen::Vector3d(diff / norm);
      }
      return Eigen::Vector3d::UnitZ();
    }
    case ShapeType::kCylinder: {
      const Eigen::Vector3d v = point - primitive.center;
      const double along = v.dot(primitive.axis);
      const Eigen::Vector3d radial = v - along * primitive.axis;
      const double norm = radial.norm();
      if (norm > 1e-9) {
        return Eigen::Vector3d(radial / norm);
      }
      return Eigen::Vector3d::UnitX();
    }
    case ShapeType::kPlane:
      return primitive.axis;
    case ShapeType::kBox: {
      // 가장 가까운 면의 법선 반환 (간소화)
      const Eigen::Vector3d delta = point - primitive.center;
      const Eigen::Vector3d half = primitive.dimensions * 0.5;
      int max_axis = 0;
      double max_ratio = 0.0;
      for (int i = 0; i < 3; ++i) {
        const double ratio =
            half(i) > 1e-9 ? std::abs(delta(i)) / half(i) : 0.0;
        if (ratio > max_ratio) {
          max_ratio = ratio;
          max_axis = i;
        }
      }
      Eigen::Vector3d normal = Eigen::Vector3d::Zero();
      normal(max_axis) = delta(max_axis) >= 0.0 ? 1.0 : -1.0;
      return normal;
    }
    default:
      return Eigen::Vector3d::UnitZ();
  }
}

double ProtuberanceDetector::ComputeSurfaceExtent(
    const std::vector<uint32_t>& cluster_indices,
    const std::vector<PointResidual>& residuals) {
  double max_dist = 0.0;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    for (size_t j = i + 1; j < cluster_indices.size(); ++j) {
      const double d =
          (residuals[cluster_indices[i]].position -
           residuals[cluster_indices[j]].position)
              .norm();
      max_dist = std::max(max_dist, d);
    }
  }
  return max_dist;
}

double ProtuberanceDetector::ComputeConfidence(
    uint32_t num_points,
    double protrusion_depth,
    bool has_gap) const {
  // 각 요소를 [0,1]로 정규화하여 가중 합산
  const double score_points =
      std::min(1.0, static_cast<double>(num_points) / 10.0);
  const double score_depth = std::min(1.0, protrusion_depth / 0.020);
  const double score_gap = has_gap ? 1.0 : 0.0;

  return config_.weight_num_points * score_points +
         config_.weight_depth * score_depth +
         config_.weight_gap * score_gap;
}

Protuberance ProtuberanceDetector::BuildProtuberance(
    const std::vector<uint32_t>& cluster_indices,
    const std::vector<PointResidual>& residuals,
    const ShapeEstimate& primitive,
    const std::vector<GapEvent>& all_gaps) const {
  // 클러스터 centroid
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  double mean_abs_residual = 0.0;
  for (uint32_t idx : cluster_indices) {
    centroid += residuals[idx].position;
    mean_abs_residual += std::abs(residuals[idx].signed_residual);
  }
  const auto n = static_cast<double>(cluster_indices.size());
  centroid /= n;
  mean_abs_residual /= n;

  // 돌출 방향
  const Eigen::Vector3d direction =
      PrimitiveOutwardNormal(centroid, primitive);

  // 표면 방향 크기
  const double extent = ComputeSurfaceExtent(cluster_indices, residuals);

  // 근처 gap 탐색
  bool has_gap = false;
  for (const auto& gap : all_gaps) {
    const Eigen::Vector3d mid_point =
        (gap.position_before + gap.position_after) * 0.5;
    if ((mid_point - centroid).norm() <
        config_.gap_cluster_association_radius) {
      has_gap = true;
      break;
    }
  }

  // 신뢰도
  const auto num_points = static_cast<uint32_t>(cluster_indices.size());
  const double confidence =
      ComputeConfidence(num_points, mean_abs_residual, has_gap);

  Protuberance prot;
  prot.centroid = centroid;
  prot.direction = direction;
  prot.extent_along_surface = extent;
  prot.protrusion_depth = mean_abs_residual;
  prot.confidence = confidence;
  prot.num_points = num_points;
  prot.has_gap = has_gap;

  RCLCPP_DEBUG(logger(),
               "BuildProtuberance: centroid=[%.3f, %.3f, %.3f], "
               "depth=%.4f, extent=%.4f, confidence=%.2f, "
               "n=%u, has_gap=%d",
               centroid.x(), centroid.y(), centroid.z(),
               mean_abs_residual, extent, confidence,
               num_points, has_gap ? 1 : 0);

  return prot;
}

}  // namespace shape_estimation
