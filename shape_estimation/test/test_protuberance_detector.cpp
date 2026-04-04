#include <gtest/gtest.h>

#include "shape_estimation/protuberance_detector.hpp"
#include "shape_estimation/snapshot_history.hpp"

#include <cmath>
#include <vector>

namespace shape_estimation {
namespace {

// ── 헬퍼 ────────────────────────────────────────────────────────────────────

// cylinder 표면 위 포인트 생성 (축 = Z, 중심 = origin)
std::vector<PointWithNormal> MakeCylinderSurfacePoints(
    double radius, int count, double z_min = -0.05, double z_max = 0.05) {
  std::vector<PointWithNormal> points;
  points.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double angle =
        2.0 * M_PI * static_cast<double>(i) / static_cast<double>(count);
    const double z = z_min + (z_max - z_min) *
                                 static_cast<double>(i) /
                                 static_cast<double>(count);
    PointWithNormal p;
    p.position = Eigen::Vector3d(radius * std::cos(angle),
                                 radius * std::sin(angle), z);
    p.normal = Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0);
    p.timestamp_ns = static_cast<uint64_t>(i) * 10'000'000ULL;
    points.push_back(p);
  }
  return points;
}

// sphere 표면 위 포인트 생성
std::vector<PointWithNormal> MakeSphereSurfacePoints(
    const Eigen::Vector3d& center, double radius, int count) {
  std::vector<PointWithNormal> points;
  points.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    // 균일하지 않지만 테스트에는 충분
    const double theta =
        M_PI * static_cast<double>(i) / static_cast<double>(count);
    const double phi =
        2.0 * M_PI * static_cast<double>(i * 7 % count) /
        static_cast<double>(count);
    const Eigen::Vector3d dir(
        std::sin(theta) * std::cos(phi),
        std::sin(theta) * std::sin(phi),
        std::cos(theta));
    PointWithNormal p;
    p.position = center + radius * dir;
    p.normal = dir;
    p.timestamp_ns = static_cast<uint64_t>(i) * 10'000'000ULL;
    points.push_back(p);
  }
  return points;
}

// plane 표면 위 포인트 생성 (z = 0 평면, 법선 = +Z)
std::vector<PointWithNormal> MakePlaneSurfacePoints(int count) {
  std::vector<PointWithNormal> points;
  points.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double x = -0.05 + 0.1 * static_cast<double>(i) /
                                 static_cast<double>(count);
    const double y = -0.05 + 0.1 * static_cast<double>((i * 3) % count) /
                                 static_cast<double>(count);
    PointWithNormal p;
    p.position = Eigen::Vector3d(x, y, 0.0);
    p.normal = Eigen::Vector3d(0, 0, 1);
    p.timestamp_ns = static_cast<uint64_t>(i) * 10'000'000ULL;
    points.push_back(p);
  }
  return points;
}

ShapeEstimate MakeCylinderEstimate(double radius) {
  ShapeEstimate est;
  est.type = ShapeType::kCylinder;
  est.center = Eigen::Vector3d::Zero();
  est.axis = Eigen::Vector3d::UnitZ();
  est.radius = radius;
  est.confidence = 0.9;
  est.num_points_used = 30;
  return est;
}

ShapeEstimate MakeSphereEstimate(const Eigen::Vector3d& center,
                                 double radius) {
  ShapeEstimate est;
  est.type = ShapeType::kSphere;
  est.center = center;
  est.axis = Eigen::Vector3d::UnitZ();
  est.radius = radius;
  est.confidence = 0.9;
  est.num_points_used = 30;
  return est;
}

ShapeEstimate MakePlaneEstimate() {
  ShapeEstimate est;
  est.type = ShapeType::kPlane;
  est.center = Eigen::Vector3d::Zero();
  est.axis = Eigen::Vector3d::UnitZ();
  est.confidence = 0.9;
  est.num_points_used = 20;
  return est;
}

// 시계열 snapshot 생성 (센서 0만 유효)
std::vector<ToFSnapshot> MakeSnapshotSeries(
    const std::vector<double>& distances,
    const std::vector<bool>& valid_flags) {
  std::vector<ToFSnapshot> snaps;
  snaps.reserve(distances.size());
  for (size_t i = 0; i < distances.size(); ++i) {
    ToFSnapshot snap;
    snap.timestamp_ns = (i + 1) * 10'000'000ULL;  // 10ms 간격
    snap.readings[0].distance_m = distances[i];
    snap.readings[0].valid = valid_flags[i];
    if (valid_flags[i]) {
      snap.surface_points_world[0] =
          Eigen::Vector3d(0.0, 0.0, distances[i]);
    }
    snaps.push_back(snap);
  }
  return snaps;
}

// ═════════════════════════════════════════════════════════════════════════════
// Signed Residual 테스트
// ═════════════════════════════════════════════════════════════════════════════

TEST(ProtuberanceDetector, CylinderResidualZeroForSurfacePoints) {
  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 20);
  auto est = MakeCylinderEstimate(radius);
  ProtuberanceDetector det;

  auto result = det.Detect(est, points, {});

  // 모든 잔차 ≈ 0이므로 돌출 없음
  EXPECT_FALSE(result.detected);
  EXPECT_TRUE(result.protuberances.empty());
}

TEST(ProtuberanceDetector, CylinderResidualNegativeForProtrudingPoints) {
  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 20);
  auto est = MakeCylinderEstimate(radius);

  // 표면에서 15mm 돌출된 포인트 5개 추가 (같은 영역)
  for (int i = 0; i < 5; ++i) {
    PointWithNormal p;
    const double angle = 0.5 + 0.1 * static_cast<double>(i);
    const double r_out = radius + 0.015;  // 15mm 돌출
    p.position = Eigen::Vector3d(r_out * std::cos(angle),
                                 r_out * std::sin(angle), 0.01);
    p.normal = Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0);
    points.push_back(p);
  }

  ProtuberanceDetector det;
  auto result = det.Detect(est, points, {});

  EXPECT_TRUE(result.detected);
  ASSERT_FALSE(result.protuberances.empty());
  // 돌출 깊이 ≈ 15mm
  EXPECT_NEAR(result.protuberances[0].protrusion_depth, 0.015, 0.003);
}

TEST(ProtuberanceDetector, SphereResidualCorrectSign) {
  const Eigen::Vector3d center(0.3, 0.0, 0.2);
  const double radius = 0.050;
  auto points = MakeSphereSurfacePoints(center, radius, 20);
  auto est = MakeSphereEstimate(center, radius);

  // 돌출 포인트 5개 추가 (표면보다 10mm 바깥)
  for (int i = 0; i < 5; ++i) {
    const double theta = 1.0 + 0.1 * static_cast<double>(i);
    const Eigen::Vector3d dir(std::sin(theta), std::cos(theta), 0.0);
    PointWithNormal p;
    p.position = center + (radius + 0.010) * dir;
    p.normal = dir;
    points.push_back(p);
  }

  ProtuberanceDetector det;
  auto result = det.Detect(est, points, {});

  EXPECT_TRUE(result.detected);
  ASSERT_FALSE(result.protuberances.empty());
  EXPECT_NEAR(result.protuberances[0].protrusion_depth, 0.010, 0.003);
}

TEST(ProtuberanceDetector, PlaneResidualCorrectSign) {
  auto points = MakePlaneSurfacePoints(20);
  auto est = MakePlaneEstimate();

  // 평면 위 10mm 돌출 포인트 4개 (인접)
  for (int i = 0; i < 4; ++i) {
    PointWithNormal p;
    p.position = Eigen::Vector3d(
        0.01 * static_cast<double>(i), 0.0, 0.010);
    p.normal = Eigen::Vector3d(0, 0, 1);
    points.push_back(p);
  }

  ProtuberanceConfig cfg;
  cfg.min_cluster_points = 3;
  ProtuberanceDetector det(cfg);
  auto result = det.Detect(est, points, {});

  EXPECT_TRUE(result.detected);
  ASSERT_FALSE(result.protuberances.empty());
  EXPECT_NEAR(result.protuberances[0].protrusion_depth, 0.010, 0.003);
}

// ═════════════════════════════════════════════════════════════════════════════
// 클러스터링 테스트
// ═════════════════════════════════════════════════════════════════════════════

TEST(ProtuberanceDetector, NoClusterWhenAllResidualSmall) {
  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 30);
  auto est = MakeCylinderEstimate(radius);

  ProtuberanceDetector det;
  auto result = det.Detect(est, points, {});

  EXPECT_FALSE(result.detected);
}

TEST(ProtuberanceDetector, SingleClusterFromAdjacentPoints) {
  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 10);
  auto est = MakeCylinderEstimate(radius);

  // 5개 인접 돌출 포인트 (10mm 간격 이내)
  for (int i = 0; i < 5; ++i) {
    PointWithNormal p;
    p.position = Eigen::Vector3d(
        radius + 0.015, 0.002 * static_cast<double>(i), 0.0);
    p.normal = Eigen::Vector3d(1, 0, 0);
    points.push_back(p);
  }

  ProtuberanceDetector det;
  auto result = det.Detect(est, points, {});

  EXPECT_TRUE(result.detected);
  EXPECT_EQ(result.protuberances.size(), 1U);
}

TEST(ProtuberanceDetector, TwoClustersFromSeparatedPoints) {
  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 10);
  auto est = MakeCylinderEstimate(radius);

  // 클러스터 1: x+ 방향
  for (int i = 0; i < 4; ++i) {
    PointWithNormal p;
    p.position = Eigen::Vector3d(
        radius + 0.015, 0.002 * static_cast<double>(i), 0.0);
    p.normal = Eigen::Vector3d(1, 0, 0);
    points.push_back(p);
  }

  // 클러스터 2: x- 방향 (cluster_radius 이상 떨어짐)
  for (int i = 0; i < 4; ++i) {
    PointWithNormal p;
    p.position = Eigen::Vector3d(
        -(radius + 0.015), 0.002 * static_cast<double>(i), 0.0);
    p.normal = Eigen::Vector3d(-1, 0, 0);
    points.push_back(p);
  }

  ProtuberanceConfig cfg;
  cfg.min_cluster_points = 3;
  ProtuberanceDetector det(cfg);
  auto result = det.Detect(est, points, {});

  EXPECT_TRUE(result.detected);
  EXPECT_EQ(result.protuberances.size(), 2U);
}

TEST(ProtuberanceDetector, ClusterIgnoredIfTooFewPoints) {
  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 20);
  auto est = MakeCylinderEstimate(radius);

  // 2개 돌출 포인트 (< min_cluster_points=3)
  for (int i = 0; i < 2; ++i) {
    PointWithNormal p;
    p.position = Eigen::Vector3d(
        radius + 0.015, 0.002 * static_cast<double>(i), 0.0);
    p.normal = Eigen::Vector3d(1, 0, 0);
    points.push_back(p);
  }

  ProtuberanceDetector det;
  auto result = det.Detect(est, points, {});

  EXPECT_FALSE(result.detected);
}

// ═════════════════════════════════════════════════════════════════════════════
// Gap 테스트
// ═════════════════════════════════════════════════════════════════════════════

TEST(ProtuberanceDetector, DetectsGapInTimeSeries) {
  // valid valid invalid invalid invalid valid valid
  std::vector<double> dists = {0.030, 0.030, 0.0, 0.0, 0.0, 0.012, 0.012};
  std::vector<bool> valid = {true, true, false, false, false, true, true};
  auto snaps = MakeSnapshotSeries(dists, valid);

  // gap 전후 거리 차이 = |30mm - 12mm| = 18mm
  ProtuberanceConfig cfg;
  cfg.gap_distance_jump = 0.015;
  cfg.gap_cluster_association_radius = 0.10;  // 넓은 연결 반경
  ProtuberanceDetector det(cfg);

  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 20);
  auto est = MakeCylinderEstimate(radius);

  // 돌출 포인트 추가 (z=0.012 근처, gap surface_point와 가까운 위치)
  for (int i = 0; i < 5; ++i) {
    PointWithNormal p;
    p.position = Eigen::Vector3d(
        radius + 0.015, 0.002 * static_cast<double>(i), 0.012);
    p.normal = Eigen::Vector3d(1, 0, 0);
    points.push_back(p);
  }

  auto result = det.Detect(est, points, snaps);
  EXPECT_TRUE(result.detected);
  ASSERT_FALSE(result.protuberances.empty());
  EXPECT_TRUE(result.protuberances[0].has_gap);
}

TEST(ProtuberanceDetector, NoGapWhenDistanceSimilar) {
  // gap 전후 거리 비슷 → gap으로 카운트하지 않음
  std::vector<double> dists = {0.030, 0.030, 0.0, 0.0, 0.0, 0.031, 0.031};
  std::vector<bool> valid = {true, true, false, false, false, true, true};
  auto snaps = MakeSnapshotSeries(dists, valid);

  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 20);
  auto est = MakeCylinderEstimate(radius);

  // 돌출 포인트 추가
  for (int i = 0; i < 5; ++i) {
    PointWithNormal p;
    p.position = Eigen::Vector3d(
        radius + 0.015, 0.002 * static_cast<double>(i), 0.030);
    p.normal = Eigen::Vector3d(1, 0, 0);
    points.push_back(p);
  }

  ProtuberanceDetector det;
  auto result = det.Detect(est, points, snaps);
  EXPECT_TRUE(result.detected);
  // gap 전후 거리 차이 1mm < 20mm 임계값 → has_gap = false
  ASSERT_FALSE(result.protuberances.empty());
  EXPECT_FALSE(result.protuberances[0].has_gap);
}

TEST(ProtuberanceDetector, GapWithLargeDistanceJump) {
  std::vector<double> dists = {0.030, 0.030, 0.0, 0.0, 0.012, 0.012};
  std::vector<bool> valid = {true, true, false, false, true, true};
  auto snaps = MakeSnapshotSeries(dists, valid);

  ProtuberanceConfig cfg;
  cfg.gap_distance_jump = 0.015;
  cfg.gap_cluster_association_radius = 0.10;  // 넓은 연결 반경

  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 20);
  auto est = MakeCylinderEstimate(radius);

  for (int i = 0; i < 5; ++i) {
    PointWithNormal p;
    p.position = Eigen::Vector3d(
        radius + 0.015, 0.002 * static_cast<double>(i), 0.012);
    p.normal = Eigen::Vector3d(1, 0, 0);
    points.push_back(p);
  }

  ProtuberanceDetector det(cfg);
  auto result = det.Detect(est, points, snaps);
  EXPECT_TRUE(result.detected);
  ASSERT_FALSE(result.protuberances.empty());
  EXPECT_TRUE(result.protuberances[0].has_gap);
}

TEST(ProtuberanceDetector, NoGapWhenInvalidTooShort) {
  // invalid 1개 (< min_gap_invalid_count=2)
  std::vector<double> dists = {0.030, 0.030, 0.0, 0.012, 0.012};
  std::vector<bool> valid = {true, true, false, true, true};
  auto snaps = MakeSnapshotSeries(dists, valid);

  ProtuberanceConfig cfg;
  cfg.gap_distance_jump = 0.015;
  cfg.gap_cluster_association_radius = 0.050;

  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 20);
  auto est = MakeCylinderEstimate(radius);

  for (int i = 0; i < 5; ++i) {
    PointWithNormal p;
    p.position = Eigen::Vector3d(
        radius + 0.015, 0.002 * static_cast<double>(i), 0.012);
    p.normal = Eigen::Vector3d(1, 0, 0);
    points.push_back(p);
  }

  ProtuberanceDetector det(cfg);
  auto result = det.Detect(est, points, snaps);
  EXPECT_TRUE(result.detected);
  ASSERT_FALSE(result.protuberances.empty());
  // invalid 1개 < 2 → gap 아님
  EXPECT_FALSE(result.protuberances[0].has_gap);
}

// ═════════════════════════════════════════════════════════════════════════════
// 통합 탐지 테스트
// ═════════════════════════════════════════════════════════════════════════════

TEST(ProtuberanceDetector, DetectsHandleOnCylinder) {
  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 30);
  auto est = MakeCylinderEstimate(radius);

  // 손잡이: 표면에서 20mm 돌출, 5개 포인트
  for (int i = 0; i < 5; ++i) {
    PointWithNormal p;
    p.position = Eigen::Vector3d(
        radius + 0.020, 0.002 * static_cast<double>(i), 0.0);
    p.normal = Eigen::Vector3d(1, 0, 0);
    points.push_back(p);
  }

  // gap 이벤트 포함 snapshot
  std::vector<double> dists = {0.030, 0.030, 0.0, 0.0, 0.0, 0.010, 0.010};
  std::vector<bool> valid = {true, true, false, false, false, true, true};
  auto snaps = MakeSnapshotSeries(dists, valid);

  ProtuberanceConfig cfg;
  cfg.gap_distance_jump = 0.015;
  cfg.gap_cluster_association_radius = 0.10;  // 넓은 연결
  ProtuberanceDetector det(cfg);
  auto result = det.Detect(est, points, snaps);

  EXPECT_TRUE(result.detected);
  ASSERT_EQ(result.protuberances.size(), 1U);
  EXPECT_NEAR(result.protuberances[0].protrusion_depth, 0.020, 0.003);
  EXPECT_TRUE(result.protuberances[0].has_gap);
  EXPECT_GT(result.protuberances[0].confidence, 0.7);
}

TEST(ProtuberanceDetector, NoProtuberanceOnCleanCylinder) {
  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 30);
  auto est = MakeCylinderEstimate(radius);

  ProtuberanceDetector det;
  auto result = det.Detect(est, points, {});

  EXPECT_FALSE(result.detected);
  EXPECT_TRUE(result.protuberances.empty());
}

TEST(ProtuberanceDetector, NoProtuberanceOnCleanSphere) {
  const Eigen::Vector3d center(0.3, 0.0, 0.2);
  const double radius = 0.050;
  auto points = MakeSphereSurfacePoints(center, radius, 30);
  auto est = MakeSphereEstimate(center, radius);

  ProtuberanceDetector det;
  auto result = det.Detect(est, points, {});

  EXPECT_FALSE(result.detected);
}

TEST(ProtuberanceDetector, LowConfidenceWithoutGap) {
  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 20);
  auto est = MakeCylinderEstimate(radius);

  // 돌출 포인트 (gap 없음)
  for (int i = 0; i < 5; ++i) {
    PointWithNormal p;
    p.position = Eigen::Vector3d(
        radius + 0.010, 0.002 * static_cast<double>(i), 0.0);
    p.normal = Eigen::Vector3d(1, 0, 0);
    points.push_back(p);
  }

  ProtuberanceDetector det;
  auto result = det.Detect(est, points, {});

  EXPECT_TRUE(result.detected);
  ASSERT_FALSE(result.protuberances.empty());
  // gap 없으므로 weight_gap 기여 = 0 → 신뢰도 < 0.7
  EXPECT_LT(result.protuberances[0].confidence, 0.7);
  EXPECT_FALSE(result.protuberances[0].has_gap);
}

TEST(ProtuberanceDetector, MultipleProtuberances) {
  const double radius = 0.040;
  auto points = MakeCylinderSurfacePoints(radius, 10);
  auto est = MakeCylinderEstimate(radius);

  // 영역 1
  for (int i = 0; i < 4; ++i) {
    PointWithNormal p;
    p.position = Eigen::Vector3d(
        radius + 0.015, 0.002 * static_cast<double>(i), 0.0);
    p.normal = Eigen::Vector3d(1, 0, 0);
    points.push_back(p);
  }

  // 영역 2 (충분히 떨어진 위치)
  for (int i = 0; i < 4; ++i) {
    PointWithNormal p;
    p.position = Eigen::Vector3d(
        -(radius + 0.015), 0.002 * static_cast<double>(i), 0.0);
    p.normal = Eigen::Vector3d(-1, 0, 0);
    points.push_back(p);
  }

  ProtuberanceConfig cfg;
  cfg.min_cluster_points = 3;
  ProtuberanceDetector det(cfg);
  auto result = det.Detect(est, points, {});

  EXPECT_TRUE(result.detected);
  EXPECT_EQ(result.protuberances.size(), 2U);
}

// ═════════════════════════════════════════════════════════════════════════════
// SnapshotHistory 테스트
// ═════════════════════════════════════════════════════════════════════════════

TEST(SnapshotHistory, PushAndRetrieve) {
  SnapshotHistory history(10.0, 100);

  for (int i = 0; i < 10; ++i) {
    ToFSnapshot snap;
    snap.timestamp_ns = static_cast<uint64_t>(i + 1) * 100'000'000ULL;
    history.Push(snap);
  }

  EXPECT_EQ(history.Size(), 10U);
  // 시간순 정렬 확인
  const auto& snaps = history.Snapshots();
  for (size_t i = 1; i < snaps.size(); ++i) {
    EXPECT_GE(snaps[i].timestamp_ns, snaps[i - 1].timestamp_ns);
  }
}

TEST(SnapshotHistory, PrunesByTime) {
  SnapshotHistory history(1.0, 1000);  // 1초 유지

  // 2초치 데이터 (10ms 간격 = 200개)
  for (int i = 0; i < 200; ++i) {
    ToFSnapshot snap;
    snap.timestamp_ns = static_cast<uint64_t>(i) * 10'000'000ULL;  // 10ms
    history.Push(snap);
  }

  // 최신 = 1.99초, 1초 이내 = 0.99초~1.99초 → ~100개
  EXPECT_LE(history.Size(), 110U);  // 약간의 여유
  EXPECT_GE(history.Size(), 90U);
}

TEST(SnapshotHistory, PrunesByCount) {
  SnapshotHistory history(100.0, 5);  // 최대 5개

  for (int i = 0; i < 10; ++i) {
    ToFSnapshot snap;
    snap.timestamp_ns = static_cast<uint64_t>(i + 1) * 100'000'000ULL;
    history.Push(snap);
  }

  EXPECT_EQ(history.Size(), 5U);
  // 최신 5개만 남음
  EXPECT_EQ(history.Snapshots().front().timestamp_ns, 600'000'000ULL);
  EXPECT_EQ(history.Snapshots().back().timestamp_ns, 1'000'000'000ULL);
}

TEST(SnapshotHistory, ClearResetsAll) {
  SnapshotHistory history;

  for (int i = 0; i < 5; ++i) {
    ToFSnapshot snap;
    snap.timestamp_ns = static_cast<uint64_t>(i + 1) * 100'000'000ULL;
    history.Push(snap);
  }

  history.Clear();
  EXPECT_EQ(history.Size(), 0U);
  EXPECT_TRUE(history.Snapshots().empty());
}

}  // namespace
}  // namespace shape_estimation
