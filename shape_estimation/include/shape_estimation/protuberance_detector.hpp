#pragma once

#include "shape_estimation/tof_shape_types.hpp"

#include <cstdint>
#include <vector>

namespace shape_estimation {

// 피팅된 primitive 잔차의 공간적 분포를 분석하여 돌출 구조를 탐지한다.
// ROS 비의존 순수 C++ 클래스.
class ProtuberanceDetector {
 public:
  explicit ProtuberanceDetector(
      const ProtuberanceConfig& config = ProtuberanceConfig{});

  // 메인 인터페이스: 피팅된 primitive + 포인트 + 최근 snapshot → 돌출 탐지
  [[nodiscard]] ProtuberanceResult Detect(
      const ShapeEstimate& fitted_primitive,
      const std::vector<PointWithNormal>& points,
      const std::vector<ToFSnapshot>& recent_snapshots) const;

  void UpdateConfig(const ProtuberanceConfig& config);

 private:
  ProtuberanceConfig config_;

  // ── Step 1: Signed Residual 계산 ──────────────────────────────────────────
  struct PointResidual {
    uint32_t point_index;
    double signed_residual;  // [m], 음수 = 돌출
    Eigen::Vector3d position;
  };

  [[nodiscard]] std::vector<PointResidual> ComputeSignedResiduals(
      const ShapeEstimate& primitive,
      const std::vector<PointWithNormal>& points) const;

  // Primitive 표면에서의 signed distance
  // 양수 = 내부 (정상), 음수 = 외부 (돌출)
  [[nodiscard]] static double SignedDistanceFromPrimitive(
      const Eigen::Vector3d& point,
      const ShapeEstimate& primitive);

  // ── Step 2: 음의 잔차 클러스터 추출 ───────────────────────────────────────
  [[nodiscard]] std::vector<std::vector<uint32_t>> ClusterNegativeResiduals(
      const std::vector<PointResidual>& residuals) const;

  // ── Step 3: Gap 패턴 분석 ─────────────────────────────────────────────────
  struct GapEvent {
    uint64_t timestamp_ns;
    uint8_t sensor_index;
    Eigen::Vector3d position_before;
    Eigen::Vector3d position_after;
    double distance_before;
    double distance_after;
    uint32_t gap_duration_samples;
  };

  [[nodiscard]] std::vector<GapEvent> DetectGaps(
      const std::vector<ToFSnapshot>& snapshots) const;

  // ── Step 4: 클러스터 + Gap 통합 → Protuberance 생성 ──────────────────────
  [[nodiscard]] Protuberance BuildProtuberance(
      const std::vector<uint32_t>& cluster_indices,
      const std::vector<PointResidual>& residuals,
      const ShapeEstimate& primitive,
      const std::vector<GapEvent>& all_gaps) const;

  // Primitive 표면의 바깥 방향 법선
  [[nodiscard]] static Eigen::Vector3d PrimitiveOutwardNormal(
      const Eigen::Vector3d& point,
      const ShapeEstimate& primitive);

  // 클러스터의 표면 투영 크기 계산
  [[nodiscard]] static double ComputeSurfaceExtent(
      const std::vector<uint32_t>& cluster_indices,
      const std::vector<PointResidual>& residuals);

  // 신뢰도 계산
  [[nodiscard]] double ComputeConfidence(
      uint32_t num_points,
      double protrusion_depth,
      bool has_gap) const;
};

}  // namespace shape_estimation
