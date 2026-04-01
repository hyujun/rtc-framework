#pragma once

#include "shape_estimation/tof_shape_types.hpp"

#include <array>

namespace shape_estimation {

/// 곡률 기반 rule-based 1차 형상 분류
/// 실시간 (매 snapshot마다) 호출 가능. 계산 비용이 낮음.
class FastShapeClassifier {
 public:
  struct Config {
    double flat_curvature_threshold{5.0};       // [1/m], 반지름 200mm 이상이면 flat
    double curvature_uniformity_threshold{2.0}; // 곡률 분산 임계값
  };

  FastShapeClassifier();
  explicit FastShapeClassifier(const Config& config);

  /// 곡률 + 거리 기반 형상 분류
  [[nodiscard]] ShapeEstimate Classify(
      const std::array<double, kNumFingers>& curvatures,
      const std::array<bool, kNumFingers>& curvature_valid,
      const std::array<ToFReading, kTotalSensors>& readings) const;

 private:
  Config config_;
};

}  // namespace shape_estimation
