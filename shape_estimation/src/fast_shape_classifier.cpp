#include "shape_estimation/fast_shape_classifier.hpp"
#include "shape_estimation/shape_logging.hpp"

#include <cmath>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

namespace shape_estimation {

namespace {
auto logger() { return ::rtc::shape::logging::ClassifyLogger(); }
}  // namespace

FastShapeClassifier::FastShapeClassifier() : FastShapeClassifier(Config{}) {}

FastShapeClassifier::FastShapeClassifier(const Config& config)
    : config_(config) {}

ShapeEstimate FastShapeClassifier::Classify(
    const std::array<double, kNumFingers>& curvatures,
    const std::array<bool, kNumFingers>& curvature_valid,
    const std::array<ToFReading, kTotalSensors>& readings) const {
  ShapeEstimate result;

  // 유효 곡률 수집
  int n_valid = 0;
  double kappa_sum = 0.0;
  double kappa_sq_sum = 0.0;
  double kappa_abs_max = 0.0;
  std::array<double, kNumFingers> valid_kappas{};

  for (int i = 0; i < kNumFingers; ++i) {
    if (curvature_valid[static_cast<size_t>(i)]) {
      const double k = curvatures[static_cast<size_t>(i)];
      valid_kappas[static_cast<size_t>(n_valid)] = k;
      kappa_sum += k;
      kappa_sq_sum += k * k;
      kappa_abs_max = std::max(kappa_abs_max, std::abs(k));
      ++n_valid;
    }
  }

  if (n_valid < 2) {
    result.type = ShapeType::kUnknown;
    result.confidence = 0.0;
    RCLCPP_DEBUG(logger(), "유효 곡률 부족 (n_valid=%d < 2) → Unknown", n_valid);
    return result;
  }

  // 유효 거리 수
  int n_valid_readings = 0;
  for (int i = 0; i < kTotalSensors; ++i) {
    if (readings[static_cast<size_t>(i)].valid) {
      ++n_valid_readings;
    }
  }
  result.num_points_used = static_cast<uint32_t>(n_valid_readings);

  // 곡률 통계
  const double kappa_avg = kappa_sum / static_cast<double>(n_valid);
  const double kappa_var = kappa_sq_sum / static_cast<double>(n_valid) - kappa_avg * kappa_avg;
  const double kappa_std = std::sqrt(std::max(kappa_var, 0.0));

  const double eps_flat = config_.flat_curvature_threshold;
  const double eps_uniform = config_.curvature_uniformity_threshold;

  // 곡률 기반 1차 분류
  if (kappa_abs_max < eps_flat) {
    // 모든 곡률이 flat → PLANE
    result.type = ShapeType::kPlane;
    result.confidence = 1.0 - kappa_abs_max / eps_flat;
  } else if (kappa_avg > 0.0 && kappa_std < eps_uniform) {
    // 양의 곡률, 균일 → SPHERE
    result.type = ShapeType::kSphere;
    result.radius = 1.0 / kappa_avg;
    result.confidence = 1.0 - kappa_std / eps_uniform;
  } else if (n_valid >= 2) {
    // 곡률 패턴 분석: 한 방향만 곡률이 있으면 CYLINDER
    // index와 middle의 곡률이 유사하고 thumb이 다르면 cylinder 후보
    bool cylinder_pattern = false;

    if (n_valid >= 2 && curvature_valid[1] && curvature_valid[2]) {
      const double k_index = curvatures[1];
      const double k_middle = curvatures[2];
      const double diff = std::abs(k_index - k_middle);
      if (diff < eps_uniform && std::abs(k_index) > eps_flat) {
        cylinder_pattern = true;
        result.radius = 1.0 / ((k_index + k_middle) / 2.0);
      }
    }

    if (cylinder_pattern) {
      result.type = ShapeType::kCylinder;
      result.confidence = 0.6;
    } else {
      result.type = ShapeType::kUnknown;
      result.confidence = 0.3;
    }
  } else {
    result.type = ShapeType::kUnknown;
    result.confidence = 0.0;
  }

  static rclcpp::Clock steady_clock{RCL_STEADY_TIME};
  RCLCPP_DEBUG_THROTTLE(logger(), steady_clock,
                        ::rtc::shape::logging::kThrottleFastMs,
                        "분류 결과: type=%s, confidence=%.2f, "
                        "kappa_avg=%.2f, kappa_std=%.2f, n_valid=%d",
                        ShapeTypeToString(result.type).data(), result.confidence,
                        kappa_avg, kappa_std, n_valid);

  return result;
}

}  // namespace shape_estimation
