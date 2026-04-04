#include "shape_estimation/primitive_fitter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/logging.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wshadow"
#include <Eigen/Dense>
#include <Eigen/SVD>
#pragma GCC diagnostic pop

namespace shape_estimation {

static const auto kLogger = rclcpp::get_logger("PrimitiveFitter");

PrimitiveFitter::PrimitiveFitter() : PrimitiveFitter(Config{}) {}

PrimitiveFitter::PrimitiveFitter(const Config& config)
    : config_(config) {}

// ── 유틸: centroid 계산 ──────────────────────────────────────────────────────

static Eigen::Vector3d ComputeCentroid(const std::vector<PointWithNormal>& points) {
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& p : points) {
    centroid += p.position;
  }
  return centroid / static_cast<double>(points.size());
}

// ── 유틸: 3x3 공분산 행렬 ────────────────────────────────────────────────────

static Eigen::Matrix3d ComputeCovariance(
    const std::vector<PointWithNormal>& points,
    const Eigen::Vector3d& centroid) {
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto& p : points) {
    const Eigen::Vector3d d = p.position - centroid;
    cov += d * d.transpose();
  }
  return cov / static_cast<double>(points.size());
}

// ── 구 피팅 ──────────────────────────────────────────────────────────────────
// 대수적 방법: Ax + By + Cz + D = x² + y² + z² 선형화
// [2x 2y 2z 1] * [cx cy cz (cx²+cy²+cz²-r²)]^T = x²+y²+z²

ShapeEstimate PrimitiveFitter::FitSphere(
    const std::vector<PointWithNormal>& points) const {
  ShapeEstimate result;
  result.type = ShapeType::kSphere;

  const auto n = static_cast<int>(points.size());
  if (n < config_.min_points_sphere) {
    result.type = ShapeType::kUnknown;
    result.confidence = 0.0;
    RCLCPP_WARN(kLogger, "구 피팅: 포인트 부족 (%d < %d)", n, config_.min_points_sphere);
    return result;
  }

  // A * x = b 구성
  Eigen::MatrixXd A(n, 4);
  Eigen::VectorXd b(n);

  for (int i = 0; i < n; ++i) {
    const auto& p = points[static_cast<size_t>(i)].position;
    A(i, 0) = 2.0 * p.x();
    A(i, 1) = 2.0 * p.y();
    A(i, 2) = 2.0 * p.z();
    A(i, 3) = 1.0;
    b(i) = p.squaredNorm();
  }

  // SVD로 least squares 풀기
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd x = svd.solve(b);

  result.center = Eigen::Vector3d(x(0), x(1), x(2));
  const double r_sq = x(3) + result.center.squaredNorm();
  if (r_sq <= 0.0) {
    result.type = ShapeType::kUnknown;
    result.confidence = 0.0;
    RCLCPP_WARN(kLogger, "구 피팅: 음수 반지름² (r²=%.6f)", r_sq);
    return result;
  }
  result.radius = std::sqrt(r_sq);

  // 잔차로 confidence 계산
  double residual_sum = 0.0;
  for (const auto& p : points) {
    const double dist = (p.position - result.center).norm() - result.radius;
    residual_sum += dist * dist;
  }
  const double rms = std::sqrt(residual_sum / static_cast<double>(n));
  result.confidence = std::max(0.0, 1.0 - rms / result.radius);
  result.num_points_used = static_cast<uint32_t>(n);

  RCLCPP_DEBUG(kLogger,
               "구 피팅: center=[%.3f, %.3f, %.3f], r=%.4f, "
               "RMS=%.4f, confidence=%.2f, n=%d",
               result.center.x(), result.center.y(), result.center.z(),
               result.radius, rms, result.confidence, n);

  return result;
}

// ── 실린더 피팅 ──────────────────────────────────────────────────────────────
// 1) PCA로 주축 추정 (최대 고유값의 고유벡터)
// 2) 축 직교 평면에 투영 → 2D circle fit

ShapeEstimate PrimitiveFitter::FitCylinder(
    const std::vector<PointWithNormal>& points) const {
  ShapeEstimate result;
  result.type = ShapeType::kCylinder;

  const auto n = static_cast<int>(points.size());
  if (n < config_.min_points_cylinder) {
    result.type = ShapeType::kUnknown;
    result.confidence = 0.0;
    RCLCPP_WARN(kLogger, "실린더 피팅: 포인트 부족 (%d < %d)", n, config_.min_points_cylinder);
    return result;
  }

  const Eigen::Vector3d centroid = ComputeCentroid(points);
  const Eigen::Matrix3d cov = ComputeCovariance(points, centroid);

  // PCA: 고유값 분해
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(cov);
  const Eigen::Vector3d eigenvalues = eigen_solver.eigenvalues();
  const Eigen::Matrix3d eigenvectors = eigen_solver.eigenvectors();

  // 최대 고유값의 고유벡터 = cylinder 축 방향
  int max_idx = 0;
  eigenvalues.maxCoeff(&max_idx);
  const Eigen::Vector3d axis = eigenvectors.col(max_idx).normalized();

  // 축 직교 평면에 투영하여 2D circle fit
  // 축과 직교하는 2개 기저 벡터
  const int mid_idx = (max_idx == 2) ? 1 : ((max_idx == 0) ? 1 : 0);
  const int min_idx = 3 - max_idx - mid_idx;
  const Eigen::Vector3d u = eigenvectors.col(mid_idx);
  const Eigen::Vector3d v = eigenvectors.col(min_idx);

  // 2D 좌표로 투영
  Eigen::MatrixXd A2d(n, 3);
  Eigen::VectorXd b2d(n);

  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3d d = points[static_cast<size_t>(i)].position - centroid;
    const double pu = d.dot(u);
    const double pv = d.dot(v);
    A2d(i, 0) = 2.0 * pu;
    A2d(i, 1) = 2.0 * pv;
    A2d(i, 2) = 1.0;
    b2d(i) = pu * pu + pv * pv;
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd2d(A2d, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd x2d = svd2d.solve(b2d);

  // 2D center → 3D center
  const Eigen::Vector3d center_2d_3d = centroid + x2d(0) * u + x2d(1) * v;
  const double r_sq = x2d(2) + x2d(0) * x2d(0) + x2d(1) * x2d(1);

  if (r_sq <= 0.0) {
    result.type = ShapeType::kUnknown;
    result.confidence = 0.0;
    RCLCPP_WARN(kLogger, "실린더 피팅: 음수 반지름² (r²=%.6f)", r_sq);
    return result;
  }

  result.center = center_2d_3d;
  result.axis = axis;
  result.radius = std::sqrt(r_sq);

  // 잔차 계산 (축까지의 거리 - 반지름)
  double residual_sum = 0.0;
  for (const auto& p : points) {
    const Eigen::Vector3d d = p.position - result.center;
    const double along_axis = d.dot(axis);
    const double perp_dist = std::sqrt(std::max(0.0, d.squaredNorm() - along_axis * along_axis));
    const double err = perp_dist - result.radius;
    residual_sum += err * err;
  }
  const double rms = std::sqrt(residual_sum / static_cast<double>(n));
  result.confidence = std::max(0.0, 1.0 - rms / result.radius);
  result.num_points_used = static_cast<uint32_t>(n);

  RCLCPP_DEBUG(kLogger,
               "실린더 피팅: r=%.4f, axis=[%.3f, %.3f, %.3f], "
               "RMS=%.4f, confidence=%.2f, n=%d",
               result.radius, axis.x(), axis.y(), axis.z(),
               rms, result.confidence, n);

  return result;
}

// ── 평면 피팅 ────────────────────────────────────────────────────────────────
// SVD: centroid 빼고, 최소 고유값의 고유벡터 = 법선

ShapeEstimate PrimitiveFitter::FitPlane(
    const std::vector<PointWithNormal>& points) const {
  ShapeEstimate result;
  result.type = ShapeType::kPlane;

  const auto n = static_cast<int>(points.size());
  if (n < config_.min_points_plane) {
    result.type = ShapeType::kUnknown;
    result.confidence = 0.0;
    RCLCPP_WARN(kLogger, "평면 피팅: 포인트 부족 (%d < %d)", n, config_.min_points_plane);
    return result;
  }

  const Eigen::Vector3d centroid = ComputeCentroid(points);
  const Eigen::Matrix3d cov = ComputeCovariance(points, centroid);

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(cov);
  const Eigen::Vector3d eigenvalues = eigen_solver.eigenvalues();
  const Eigen::Matrix3d eigenvectors = eigen_solver.eigenvectors();

  // 최소 고유값의 고유벡터 = 평면 법선
  int min_idx = 0;
  eigenvalues.minCoeff(&min_idx);
  const Eigen::Vector3d normal = eigenvectors.col(min_idx).normalized();

  result.center = centroid;
  result.axis = normal;

  // 법선 일관성: 포인트 법선과 비교
  double normal_consistency = 0.0;
  int normal_count = 0;
  for (const auto& p : points) {
    if (p.normal.squaredNorm() > 0.5) {
      normal_consistency += std::abs(p.normal.dot(normal));
      ++normal_count;
    }
  }
  if (normal_count > 0) {
    normal_consistency /= static_cast<double>(normal_count);
  } else {
    normal_consistency = 0.5;
  }

  // 잔차: 포인트-평면 거리
  double residual_sum = 0.0;
  for (const auto& p : points) {
    const double dist = (p.position - centroid).dot(normal);
    residual_sum += dist * dist;
  }
  const double rms = std::sqrt(residual_sum / static_cast<double>(n));

  // 평면 "두께"에 비례한 confidence (rms가 1mm 미만이면 높은 confidence)
  const double thickness_conf = std::max(0.0, 1.0 - rms / 0.005);
  result.confidence = 0.5 * thickness_conf + 0.5 * normal_consistency;
  result.num_points_used = static_cast<uint32_t>(n);

  RCLCPP_DEBUG(kLogger,
               "평면 피팅: normal=[%.3f, %.3f, %.3f], RMS=%.4f, "
               "normal_consistency=%.2f, confidence=%.2f, n=%d",
               normal.x(), normal.y(), normal.z(),
               rms, normal_consistency, result.confidence, n);

  return result;
}

// ── 박스 피팅 ────────────────────────────────────────────────────────────────
// PCA 기반 Oriented Bounding Box

ShapeEstimate PrimitiveFitter::FitBox(
    const std::vector<PointWithNormal>& points) const {
  ShapeEstimate result;
  result.type = ShapeType::kBox;

  const auto n = static_cast<int>(points.size());
  if (n < config_.min_points_box) {
    result.type = ShapeType::kUnknown;
    result.confidence = 0.0;
    RCLCPP_WARN(kLogger, "박스 피팅: 포인트 부족 (%d < %d)", n, config_.min_points_box);
    return result;
  }

  const Eigen::Vector3d centroid = ComputeCentroid(points);
  const Eigen::Matrix3d cov = ComputeCovariance(points, centroid);

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(cov);
  const Eigen::Matrix3d axes = eigen_solver.eigenvectors();

  // 각 PCA 축을 따른 min/max extent
  Eigen::Vector3d min_ext = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector3d max_ext = Eigen::Vector3d::Constant(-std::numeric_limits<double>::max());

  for (const auto& p : points) {
    const Eigen::Vector3d local = axes.transpose() * (p.position - centroid);
    min_ext = min_ext.cwiseMin(local);
    max_ext = max_ext.cwiseMax(local);
  }

  result.center = centroid + axes * ((min_ext + max_ext) / 2.0);
  result.dimensions = max_ext - min_ext;
  result.axis = axes.col(2);  // 주축 (최대 분산 방향)

  // 잔차: 각 포인트와 가장 가까운 면까지의 거리
  double residual_sum = 0.0;
  const Eigen::Vector3d half = result.dimensions / 2.0;

  for (const auto& p : points) {
    const Eigen::Vector3d local = axes.transpose() * (p.position - result.center);
    const Eigen::Vector3d abs_local = local.cwiseAbs();
    // 면까지의 거리 = max(|local_i| - half_i, 0)
    double face_dist = 0.0;
    for (int i = 0; i < 3; ++i) {
      const double d = abs_local(i) - half(i);
      if (d > 0.0) {
        face_dist += d * d;
      }
    }
    residual_sum += face_dist;
  }
  const double rms = std::sqrt(residual_sum / static_cast<double>(n));
  const double avg_dim = result.dimensions.mean();
  result.confidence = (avg_dim > 1e-6) ? std::max(0.0, 1.0 - rms / avg_dim) : 0.0;
  result.num_points_used = static_cast<uint32_t>(n);

  RCLCPP_DEBUG(kLogger,
               "박스 피팅: dims=[%.4f, %.4f, %.4f], "
               "RMS=%.4f, confidence=%.2f, n=%d",
               result.dimensions.x(), result.dimensions.y(), result.dimensions.z(),
               rms, result.confidence, n);

  return result;
}

// ── 최적 피팅 선택 ───────────────────────────────────────────────────────────
// 모든 primitive 피팅 후 BIC 기반 모델 선택
// BIC = n*log(RSS/n) + k*log(n), k = 파라미터 수

ShapeEstimate PrimitiveFitter::FitBestPrimitive(
    const std::vector<PointWithNormal>& points) const {
  if (points.empty()) {
    ShapeEstimate result;
    result.type = ShapeType::kUnknown;
    return result;
  }

  struct Candidate {
    ShapeEstimate estimate;
    int num_params;
  };

  std::vector<Candidate> candidates;
  candidates.reserve(4);

  // 각 primitive 피팅
  if (auto est = FitSphere(points); est.type != ShapeType::kUnknown) {
    candidates.push_back({est, 4});  // center(3) + radius(1)
  }
  if (auto est = FitCylinder(points); est.type != ShapeType::kUnknown) {
    candidates.push_back({est, 7});  // center(3) + axis(3) + radius(1)
  }
  if (auto est = FitPlane(points); est.type != ShapeType::kUnknown) {
    candidates.push_back({est, 4});  // center(3) + normal(3) - 1 constraint
  }
  if (auto est = FitBox(points); est.type != ShapeType::kUnknown) {
    candidates.push_back({est, 9});  // center(3) + axis(3) + dims(3)
  }

  if (candidates.empty()) {
    ShapeEstimate result;
    result.type = ShapeType::kUnknown;
    RCLCPP_WARN(kLogger,
                "FitBestPrimitive: 모든 피팅 실패 (n=%zu) → Unknown",
                points.size());
    return result;
  }

  // confidence 기준으로 최적 선택 (BIC 대신 단순 confidence 우선)
  // 동일 confidence에서는 파라미터 수가 적은 모델 선호
  auto best = std::max_element(candidates.begin(), candidates.end(),
      [](const Candidate& a, const Candidate& b) {
        if (std::abs(a.estimate.confidence - b.estimate.confidence) > 0.1) {
          return a.estimate.confidence < b.estimate.confidence;
        }
        // confidence가 비슷하면 단순한 모델 선호
        return a.num_params > b.num_params;
      });

  auto result = best->estimate;
  if (result.confidence < config_.min_confidence) {
    RCLCPP_WARN(kLogger,
                "FitBestPrimitive: 최고 confidence(%.2f) < 최소(%.2f) → Unknown",
                result.confidence, config_.min_confidence);
    result.type = ShapeType::kUnknown;
  } else {
    RCLCPP_DEBUG(kLogger,
                 "FitBestPrimitive: 선택=%s, confidence=%.2f, candidates=%zu",
                 ShapeTypeToString(result.type).data(), result.confidence,
                 candidates.size());
  }

  return result;
}

}  // namespace shape_estimation
