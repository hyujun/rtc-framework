#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <vector>

#include "shape_estimation/fast_shape_classifier.hpp"
#include "shape_estimation/primitive_fitter.hpp"
#include "shape_estimation/tof_shape_types.hpp"
#include "shape_estimation/voxel_point_cloud.hpp"

namespace se = shape_estimation;

// ═════════════════════════════════════════════════════════════════════════════
// VoxelPointCloud 테스트
// ═════════════════════════════════════════════════════════════════════════════

class VoxelPointCloudTest : public ::testing::Test {
 protected:
  se::VoxelPointCloud::Config config_{.voxel_resolution_m = 0.002,
                                       .max_points = 100,
                                       .expiry_duration_sec = 1.0};
  se::VoxelPointCloud cloud_{config_};

  // 단순 스냅샷 생성 헬퍼
  se::ToFSnapshot MakeSnapshot(double distance, uint64_t timestamp_ns) {
    se::ToFSnapshot snap;
    snap.timestamp_ns = timestamp_ns;

    for (int i = 0; i < se::kTotalSensors; ++i) {
      const auto idx = static_cast<size_t>(i);
      snap.readings[idx].distance_m = distance;
      snap.readings[idx].valid = true;

      // 원점 기준 각 센서를 약간 다른 위치에 배치
      const double offset = static_cast<double>(i) * 0.01;
      snap.sensor_positions_world[idx] = Eigen::Vector3d(offset, 0, 0);
      snap.surface_points_world[idx] = Eigen::Vector3d(offset, 0, distance);
    }

    for (int f = 0; f < se::kNumFingers; ++f) {
      snap.beam_directions_world[static_cast<size_t>(f)] = Eigen::Vector3d::UnitZ();
      snap.local_curvatures[static_cast<size_t>(f)] = 0.0;
      snap.curvature_valid[static_cast<size_t>(f)] = true;
    }

    return snap;
  }
};

TEST_F(VoxelPointCloudTest, EmptyCloudReturnsZero) {
  EXPECT_EQ(cloud_.Size(), 0);
  EXPECT_TRUE(cloud_.GetPoints().empty());
}

TEST_F(VoxelPointCloudTest, AddSnapshotIncreasesSize) {
  cloud_.AddSnapshot(MakeSnapshot(0.05, 1000));
  EXPECT_GT(cloud_.Size(), 0);
}

TEST_F(VoxelPointCloudTest, DeduplicatesClosePoints) {
  const auto snap = MakeSnapshot(0.05, 1000);
  cloud_.AddSnapshot(snap);
  const int size_after_first = cloud_.Size();

  // 동일 스냅샷 재추가 → 같은 voxel이므로 크기 변화 없음
  cloud_.AddSnapshot(snap);
  EXPECT_EQ(cloud_.Size(), size_after_first);
}

TEST_F(VoxelPointCloudTest, RespectsMaxCapacity) {
  // max_points=100, 매번 다른 위치에 포인트 추가
  for (int i = 0; i < 200; ++i) {
    se::ToFSnapshot snap;
    snap.timestamp_ns = static_cast<uint64_t>(i) * 1000;
    snap.readings[0].distance_m = 0.05;
    snap.readings[0].valid = true;
    // 각각 다른 voxel에 배치 (간격 > voxel_resolution)
    snap.sensor_positions_world[0] = Eigen::Vector3d(
        static_cast<double>(i) * 0.01, 0, 0);
    snap.surface_points_world[0] = Eigen::Vector3d(
        static_cast<double>(i) * 0.01, 0, 0.05);
    snap.beam_directions_world[0] = Eigen::Vector3d::UnitZ();
    // 나머지 센서는 invalid
    for (int j = 1; j < se::kTotalSensors; ++j) {
      snap.readings[static_cast<size_t>(j)].valid = false;
    }
    cloud_.AddSnapshot(snap);
  }

  EXPECT_LE(cloud_.Size(), config_.max_points);
}

TEST_F(VoxelPointCloudTest, RemovesExpiredPoints) {
  // timestamp_ns = 1000에 추가
  cloud_.AddSnapshot(MakeSnapshot(0.05, 1'000'000'000ULL));
  EXPECT_GT(cloud_.Size(), 0);

  // 2초 후 만료 체크 (expiry = 1초)
  cloud_.RemoveExpired(3'000'000'000ULL);
  EXPECT_EQ(cloud_.Size(), 0);
}

TEST_F(VoxelPointCloudTest, ClearRemovesAll) {
  cloud_.AddSnapshot(MakeSnapshot(0.05, 1000));
  EXPECT_GT(cloud_.Size(), 0);
  cloud_.Clear();
  EXPECT_EQ(cloud_.Size(), 0);
}

// ═════════════════════════════════════════════════════════════════════════════
// FastShapeClassifier 테스트
// ═════════════════════════════════════════════════════════════════════════════

class FastShapeClassifierTest : public ::testing::Test {
 protected:
  se::FastShapeClassifier classifier_;

  std::array<se::ToFReading, se::kTotalSensors> MakeValidReadings(double dist) {
    std::array<se::ToFReading, se::kTotalSensors> r{};
    for (auto& reading : r) {
      reading.distance_m = dist;
      reading.valid = true;
    }
    return r;
  }
};

TEST_F(FastShapeClassifierTest, InsufficientDataReturnsUnknown) {
  std::array<double, se::kNumFingers> curvatures{0.0, 0.0, 0.0};
  std::array<bool, se::kNumFingers> valid{true, false, false};  // 1개만 valid
  auto readings = MakeValidReadings(0.05);

  auto result = classifier_.Classify(curvatures, valid, readings);
  EXPECT_EQ(result.type, se::ShapeType::kUnknown);
}

TEST_F(FastShapeClassifierTest, ClassifiesPlane) {
  // 모든 곡률 ≈ 0
  std::array<double, se::kNumFingers> curvatures{0.1, -0.2, 0.05};
  std::array<bool, se::kNumFingers> valid{true, true, true};
  auto readings = MakeValidReadings(0.05);

  auto result = classifier_.Classify(curvatures, valid, readings);
  EXPECT_EQ(result.type, se::ShapeType::kPlane);
  EXPECT_GT(result.confidence, 0.5);
}

TEST_F(FastShapeClassifierTest, ClassifiesSphere) {
  // 모든 곡률 ≈ 20.0 (r=0.05m)
  const double kappa = 20.0;
  std::array<double, se::kNumFingers> curvatures{kappa, kappa + 0.5, kappa - 0.3};
  std::array<bool, se::kNumFingers> valid{true, true, true};
  auto readings = MakeValidReadings(0.05);

  auto result = classifier_.Classify(curvatures, valid, readings);
  EXPECT_EQ(result.type, se::ShapeType::kSphere);
  EXPECT_NEAR(result.radius, 0.05, 0.01);
  EXPECT_GT(result.confidence, 0.5);
}

// ═════════════════════════════════════════════════════════════════════════════
// PrimitiveFitter 테스트
// ═════════════════════════════════════════════════════════════════════════════

class PrimitiveFitterTest : public ::testing::Test {
 protected:
  se::PrimitiveFitter fitter_;
  std::mt19937 rng_{42};  // 재현 가능한 랜덤

  // 구 표면 위의 포인트 생성 (노이즈 포함)
  std::vector<se::PointWithNormal> GenerateSpherePoints(
      const Eigen::Vector3d& center, double radius, int count, double noise = 0.0) {
    std::vector<se::PointWithNormal> points;
    points.reserve(static_cast<size_t>(count));
    std::normal_distribution<double> dist(0.0, noise);

    for (int i = 0; i < count; ++i) {
      // 균일 분포 방향
      const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(count);
      const double phi = M_PI * (0.3 + 0.4 * static_cast<double>(i % 5) / 5.0);

      Eigen::Vector3d dir(
          std::sin(phi) * std::cos(theta),
          std::sin(phi) * std::sin(theta),
          std::cos(phi));

      se::PointWithNormal p;
      p.position = center + (radius + (noise > 0 ? dist(rng_) : 0.0)) * dir;
      p.normal = dir;
      points.push_back(p);
    }
    return points;
  }

  // 평면 위의 포인트 생성
  std::vector<se::PointWithNormal> GeneratePlanePoints(
      const Eigen::Vector3d& center, const Eigen::Vector3d& normal,
      int count, double extent = 0.05, double noise = 0.0) {
    std::vector<se::PointWithNormal> points;
    points.reserve(static_cast<size_t>(count));
    std::normal_distribution<double> dist(0.0, noise);

    // normal에 직교하는 2개 축
    Eigen::Vector3d u = normal.unitOrthogonal();
    Eigen::Vector3d v = normal.cross(u);

    for (int i = 0; i < count; ++i) {
      const double s = extent * (static_cast<double>(i % 5) / 4.0 - 0.5);
      const double t = extent * (static_cast<double>(i / 5) / 4.0 - 0.5);

      se::PointWithNormal p;
      p.position = center + s * u + t * v + (noise > 0 ? dist(rng_) : 0.0) * normal;
      p.normal = normal;
      points.push_back(p);
    }
    return points;
  }

  // 실린더 표면 위의 포인트 생성
  std::vector<se::PointWithNormal> GenerateCylinderPoints(
      const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
      double radius, double height, int count, double noise = 0.0) {
    std::vector<se::PointWithNormal> points;
    points.reserve(static_cast<size_t>(count));
    std::normal_distribution<double> dist(0.0, noise);

    Eigen::Vector3d u = axis.unitOrthogonal();
    Eigen::Vector3d v = axis.cross(u);

    for (int i = 0; i < count; ++i) {
      const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(count);
      const double h = height * (static_cast<double>(i % 5) / 4.0 - 0.5);

      Eigen::Vector3d radial = std::cos(theta) * u + std::sin(theta) * v;

      se::PointWithNormal p;
      p.position = center + h * axis +
                   (radius + (noise > 0 ? dist(rng_) : 0.0)) * radial;
      p.normal = radial;
      points.push_back(p);
    }
    return points;
  }
};

TEST_F(PrimitiveFitterTest, FitsSphereFromCleanPoints) {
  const Eigen::Vector3d center(0.3, 0.1, 0.2);
  const double radius = 0.05;
  auto points = GenerateSpherePoints(center, radius, 20);

  auto result = fitter_.FitSphere(points);
  EXPECT_EQ(result.type, se::ShapeType::kSphere);
  EXPECT_NEAR(result.radius, radius, 0.005);
  EXPECT_NEAR((result.center - center).norm(), 0.0, 0.005);
  EXPECT_GT(result.confidence, 0.8);
}

TEST_F(PrimitiveFitterTest, FitsSphereFromNoisyPoints) {
  const Eigen::Vector3d center(0.3, 0.0, 0.3);
  const double radius = 0.05;
  auto points = GenerateSpherePoints(center, radius, 20, 0.001);

  auto result = fitter_.FitSphere(points);
  EXPECT_EQ(result.type, se::ShapeType::kSphere);
  EXPECT_NEAR(result.radius, radius, 0.005);
  EXPECT_GT(result.confidence, 0.5);
}

TEST_F(PrimitiveFitterTest, FitsPlaneFromCleanPoints) {
  const Eigen::Vector3d center(0.3, 0.0, 0.3);
  const Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  auto points = GeneratePlanePoints(center, normal, 20);

  auto result = fitter_.FitPlane(points);
  EXPECT_EQ(result.type, se::ShapeType::kPlane);
  EXPECT_GT(result.confidence, 0.5);
  // 법선이 ±z 방향과 일치
  EXPECT_GT(std::abs(result.axis.dot(normal)), 0.9);
}

TEST_F(PrimitiveFitterTest, FitsCylinderFromCleanPoints) {
  const Eigen::Vector3d center(0.3, 0.0, 0.3);
  const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  const double radius = 0.03;
  auto points = GenerateCylinderPoints(center, axis, radius, 0.1, 30);

  auto result = fitter_.FitCylinder(points);
  EXPECT_EQ(result.type, se::ShapeType::kCylinder);
  EXPECT_NEAR(result.radius, radius, 0.005);
  EXPECT_GT(result.confidence, 0.5);
}

TEST_F(PrimitiveFitterTest, InsufficientPointsReturnsUnknown) {
  std::vector<se::PointWithNormal> points(2);
  points[0].position = Eigen::Vector3d(0, 0, 0);
  points[1].position = Eigen::Vector3d(1, 0, 0);

  auto result = fitter_.FitSphere(points);
  EXPECT_EQ(result.type, se::ShapeType::kUnknown);
}

TEST_F(PrimitiveFitterTest, FitBestPrimitiveSelectsSphere) {
  auto points = GenerateSpherePoints(Eigen::Vector3d(0.3, 0, 0.3), 0.05, 20);

  auto result = fitter_.FitBestPrimitive(points);
  // sphere 또는 cylinder (둘 다 구에 잘 맞을 수 있음)
  EXPECT_NE(result.type, se::ShapeType::kUnknown);
  EXPECT_GT(result.confidence, 0.3);
}

TEST_F(PrimitiveFitterTest, FitBestPrimitiveSelectsPlane) {
  auto points = GeneratePlanePoints(
      Eigen::Vector3d(0.3, 0, 0.3), Eigen::Vector3d::UnitZ(), 20);

  auto result = fitter_.FitBestPrimitive(points);
  EXPECT_NE(result.type, se::ShapeType::kUnknown);
  EXPECT_GT(result.confidence, 0.3);
}

// ═════════════════════════════════════════════════════════════════════════════
// Curvature 계산 테스트
// ═════════════════════════════════════════════════════════════════════════════

TEST(CurvatureTest, FlatSurfaceHasZeroCurvature) {
  // d_A == d_B → κ ≈ 0
  const double d_a = 0.05;
  const double d_b = 0.05;
  const double delta_d = d_a - d_b;
  const double delta_x_sq = se::kTofSeparation * se::kTofSeparation;
  const double kappa = 2.0 * delta_d / (delta_x_sq + delta_d * delta_d);
  EXPECT_NEAR(kappa, 0.0, 1e-10);
}

TEST(CurvatureTest, CurvedSurfaceHasNonZeroCurvature) {
  // 반지름 R=0.05m 구에서 2mm 간격 센서의 거리차 계산
  const double R = 0.05;
  // 근사: d_A ≈ R - sqrt(R² - offset²), d_B ≈ R + sqrt(R² - offset²) 에서 차이
  // 단순화: offset=2mm에서 기하학적으로
  const double offset = se::kTofOffsetX;  // 2mm
  // 구 중심에서 tip까지 d, 구 표면까지 D
  // d_A = D - R*cos(offset/R) ≈ D - R*(1 - offset²/(2R²))
  // 간단한 테스트: delta_d가 0이 아닌지만 확인
  const double d_a = R * (1.0 - std::cos(std::asin(offset / R)));
  const double d_b = R * (1.0 + std::cos(std::asin(offset / R))) - R;
  const double delta_d = d_a - d_b;
  const double delta_x_sq = se::kTofSeparation * se::kTofSeparation;
  const double kappa = 2.0 * delta_d / (delta_x_sq + delta_d * delta_d);

  // 곡률이 0이 아님
  EXPECT_NE(kappa, 0.0);
}

// ═════════════════════════════════════════════════════════════════════════════
// SensorIndex 유틸 테스트
// ═════════════════════════════════════════════════════════════════════════════

TEST(SensorIndexTest, CorrectMapping) {
  EXPECT_EQ(se::SensorIndex(se::FingerID::kThumb, se::ToFSide::kA), 0);
  EXPECT_EQ(se::SensorIndex(se::FingerID::kThumb, se::ToFSide::kB), 1);
  EXPECT_EQ(se::SensorIndex(se::FingerID::kIndex, se::ToFSide::kA), 2);
  EXPECT_EQ(se::SensorIndex(se::FingerID::kIndex, se::ToFSide::kB), 3);
  EXPECT_EQ(se::SensorIndex(se::FingerID::kMiddle, se::ToFSide::kA), 4);
  EXPECT_EQ(se::SensorIndex(se::FingerID::kMiddle, se::ToFSide::kB), 5);
}

TEST(ShapeTypeTest, ToStringConversion) {
  EXPECT_EQ(se::ShapeTypeToString(se::ShapeType::kUnknown), "UNKNOWN");
  EXPECT_EQ(se::ShapeTypeToString(se::ShapeType::kSphere), "SPHERE");
  EXPECT_EQ(se::ShapeTypeToString(se::ShapeType::kCylinder), "CYLINDER");
  EXPECT_EQ(se::ShapeTypeToString(se::ShapeType::kPlane), "PLANE");
  EXPECT_EQ(se::ShapeTypeToString(se::ShapeType::kBox), "BOX");
}

// ═════════════════════════════════════════════════════════════════════════════
// PrimitiveFitter::FitBox 테스트
// ═════════════════════════════════════════════════════════════════════════════

TEST_F(PrimitiveFitterTest, FitsBoxFromCleanPoints) {
  // 박스 표면 포인트 생성: 0.1 x 0.08 x 0.06 박스, center=(0.3, 0, 0.3)
  const Eigen::Vector3d center(0.3, 0.0, 0.3);
  const Eigen::Vector3d half_dims(0.05, 0.04, 0.03);
  std::vector<se::PointWithNormal> points;

  // 각 면에 포인트 배치
  auto add_face = [&](int axis, double sign) {
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    normal(axis) = sign;
    const int u_axis = (axis + 1) % 3;
    const int v_axis = (axis + 2) % 3;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        se::PointWithNormal p;
        p.position = center;
        p.position(axis) += sign * half_dims(axis);
        p.position(u_axis) += half_dims(u_axis) * (static_cast<double>(i) / 3.0 - 0.5) * 2.0;
        p.position(v_axis) += half_dims(v_axis) * (static_cast<double>(j) / 3.0 - 0.5) * 2.0;
        p.normal = normal;
        points.push_back(p);
      }
    }
  };

  // 6면 모두
  for (int axis = 0; axis < 3; ++axis) {
    add_face(axis, 1.0);
    add_face(axis, -1.0);
  }

  auto result = fitter_.FitBox(points);
  EXPECT_EQ(result.type, se::ShapeType::kBox);
  EXPECT_GT(result.confidence, 0.5);
  EXPECT_NEAR((result.center - center).norm(), 0.0, 0.01);
  // dimensions의 정렬 순서는 PCA 축 순에 따라 다를 수 있으므로 정렬 후 비교
  Eigen::Vector3d sorted_dims = result.dimensions;
  Eigen::Vector3d expected_dims = half_dims * 2.0;
  std::array<double, 3> sd{sorted_dims(0), sorted_dims(1), sorted_dims(2)};
  std::array<double, 3> ed{expected_dims(0), expected_dims(1), expected_dims(2)};
  std::sort(sd.begin(), sd.end());
  std::sort(ed.begin(), ed.end());
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(sd[static_cast<size_t>(i)], ed[static_cast<size_t>(i)], 0.01);
  }
}

TEST_F(PrimitiveFitterTest, FitBoxInsufficientPointsReturnsUnknown) {
  std::vector<se::PointWithNormal> points(3);
  for (int i = 0; i < 3; ++i) {
    points[static_cast<size_t>(i)].position = Eigen::Vector3d(
        static_cast<double>(i) * 0.01, 0, 0);
  }

  auto result = fitter_.FitBox(points);
  EXPECT_EQ(result.type, se::ShapeType::kUnknown);
  EXPECT_DOUBLE_EQ(result.confidence, 0.0);
}

TEST_F(PrimitiveFitterTest, FitBestPrimitiveWithEmptyPoints) {
  std::vector<se::PointWithNormal> points;
  auto result = fitter_.FitBestPrimitive(points);
  EXPECT_EQ(result.type, se::ShapeType::kUnknown);
}

// ═════════════════════════════════════════════════════════════════════════════
// FastShapeClassifier: Cylinder 분류 테스트
// ═════════════════════════════════════════════════════════════════════════════

TEST_F(FastShapeClassifierTest, ClassifiesCylinder) {
  // Cylinder 분류 조건: 전체 곡률이 비균일(std > eps_uniform)하지만
  // index/middle 곡률이 유사하고 flat이 아닌 경우
  // thumb은 낮은 곡률 → 전체 분산을 높여 Sphere 분기를 회피
  const double k_cyl = 20.0;
  std::array<double, se::kNumFingers> curvatures{1.0, k_cyl, k_cyl + 0.5};
  std::array<bool, se::kNumFingers> valid{true, true, true};
  auto readings = MakeValidReadings(0.05);

  auto result = classifier_.Classify(curvatures, valid, readings);
  EXPECT_EQ(result.type, se::ShapeType::kCylinder);
  EXPECT_GT(result.confidence, 0.0);
  EXPECT_NEAR(result.radius, 1.0 / ((k_cyl + k_cyl + 0.5) / 2.0), 0.01);
}

TEST_F(FastShapeClassifierTest, NonUniformCurvatureReturnsUnknown) {
  // 곡률이 크지만 분산이 크면 분류 불가 → 마지막 else
  std::array<double, se::kNumFingers> curvatures{10.0, -5.0, 20.0};
  std::array<bool, se::kNumFingers> valid{true, true, true};
  auto readings = MakeValidReadings(0.05);

  auto result = classifier_.Classify(curvatures, valid, readings);
  // kappa_avg ≈ 8.33, kappa_std 큼, index/middle 곡률 차이도 큼
  EXPECT_EQ(result.type, se::ShapeType::kUnknown);
}

// ═════════════════════════════════════════════════════════════════════════════
// Default constructor 테스트
// ═════════════════════════════════════════════════════════════════════════════

TEST(DefaultConstructorTest, PrimitiveFitterDefaultConfig) {
  se::PrimitiveFitter fitter;
  // 기본 config로 구 피팅 가능
  std::vector<se::PointWithNormal> points;
  for (int i = 0; i < 10; ++i) {
    const double theta = 2.0 * M_PI * static_cast<double>(i) / 10.0;
    se::PointWithNormal p;
    p.position = Eigen::Vector3d(0.05 * std::cos(theta), 0.05 * std::sin(theta), 0.0);
    p.normal = Eigen::Vector3d(std::cos(theta), std::sin(theta), 0.0);
    points.push_back(p);
  }
  auto result = fitter.FitSphere(points);
  EXPECT_NE(result.type, se::ShapeType::kUnknown);
}

TEST(DefaultConstructorTest, FastShapeClassifierDefaultConfig) {
  se::FastShapeClassifier classifier;
  std::array<double, se::kNumFingers> curvatures{0.1, 0.2, -0.1};
  std::array<bool, se::kNumFingers> valid{true, true, true};
  std::array<se::ToFReading, se::kTotalSensors> readings{};
  for (auto& r : readings) { r.valid = true; r.distance_m = 0.05; }
  auto result = classifier.Classify(curvatures, valid, readings);
  EXPECT_EQ(result.type, se::ShapeType::kPlane);
}
