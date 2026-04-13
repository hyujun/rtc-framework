#pragma once

#include "shape_estimation/tof_shape_types.hpp"

#include <vector>

namespace shape_estimation {

/// Least Squares 기반 primitive 피팅
/// 포인트 클라우드에서 형상 파라미터를 추정
class PrimitiveFitter {
 public:
  struct Config {
    double min_confidence{0.3};   // 이 미만이면 UNKNOWN 반환
    int min_points_sphere{4};
    int min_points_cylinder{5};
    int min_points_plane{3};
    int min_points_box{6};
  };

  PrimitiveFitter();
  explicit PrimitiveFitter(const Config& config);

  /// 구 피팅: 대수적 sphere fit (SVD)
  [[nodiscard]] ShapeEstimate FitSphere(
      const std::vector<PointWithNormal>& points) const;

  /// 실린더 피팅: PCA 축 추정 → 2D circle fit
  [[nodiscard]] ShapeEstimate FitCylinder(
      const std::vector<PointWithNormal>& points) const;

  /// 평면 피팅: SVD 기반
  [[nodiscard]] ShapeEstimate FitPlane(
      const std::vector<PointWithNormal>& points) const;

  /// 박스 피팅: PCA 기반 OBB
  [[nodiscard]] ShapeEstimate FitBox(
      const std::vector<PointWithNormal>& points) const;

  /// 모든 후보 피팅 → 잔차 비교 → 최적 선택
  [[nodiscard]] ShapeEstimate FitBestPrimitive(
      const std::vector<PointWithNormal>& points) const;

 private:
  /// Centroid + Eigendecomposition 결과를 공유하는 내부 피팅
  [[nodiscard]] ShapeEstimate FitCylinderImpl(
      const std::vector<PointWithNormal>& points,
      const Eigen::Vector3d& centroid,
      const Eigen::Vector3d& eigenvalues,
      const Eigen::Matrix3d& eigenvectors) const;
  [[nodiscard]] ShapeEstimate FitPlaneImpl(
      const std::vector<PointWithNormal>& points,
      const Eigen::Vector3d& centroid,
      const Eigen::Vector3d& eigenvalues,
      const Eigen::Matrix3d& eigenvectors) const;
  [[nodiscard]] ShapeEstimate FitBoxImpl(
      const std::vector<PointWithNormal>& points,
      const Eigen::Vector3d& centroid,
      const Eigen::Matrix3d& eigenvectors) const;

  Config config_;
};

}  // namespace shape_estimation
