#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wshadow"
#include <Eigen/Core>
#include <Eigen/Geometry>
#pragma GCC diagnostic pop

namespace shape_estimation {

// ── 센서 배치 상수 ───────────────────────────────────────────────────────────

// 3개 fingertip (thumb, index, middle), 각 2개 ToF 센서
inline constexpr int kNumFingers = 3;
inline constexpr int kSensorsPerFinger = 2;
inline constexpr int kTotalSensors = kNumFingers * kSensorsPerFinger;  // 6

// ToF_A/B 오프셋: tip_link 기준 x = +/-2mm, 빔 방향 = +z
inline constexpr double kTofOffsetX = 0.002;      // [m]
inline constexpr double kTofSeparation = 0.004;    // [m] A-B 간격

// ToF 유효 범위
inline constexpr double kTofMinRange = 0.01;   // [m]
inline constexpr double kTofMaxRange = 0.30;   // [m]

// ── 손가락 / 센서 식별자 ─────────────────────────────────────────────────────

enum class FingerID : uint8_t {
  kThumb  = 0,
  kIndex  = 1,
  kMiddle = 2
};

enum class ToFSide : uint8_t {
  kA = 0,  // x = +2mm
  kB = 1   // x = -2mm
};

// readings 배열 인덱스: finger * 2 + side
[[nodiscard]] constexpr int SensorIndex(FingerID finger, ToFSide side) noexcept {
  return static_cast<int>(finger) * kSensorsPerFinger + static_cast<int>(side);
}

// ── 단일 ToF 측정값 ─────────────────────────────────────────────────────────

struct ToFReading {
  double distance_m{0.0};  // 측정 거리 [m]
  bool valid{false};       // 유효성 플래그
};

// ── 500Hz RT thread에서 생성하는 스냅샷 ──────────────────────────────────────

struct alignas(32) ToFSnapshot {
  uint64_t timestamp_ns{0};

  // 6개 측정값: [thumb_A, thumb_B, index_A, index_B, middle_A, middle_B]
  std::array<ToFReading, kTotalSensors> readings{};

  // 6개 센서의 월드 좌표 위치 (FK 결과 + offset 적용 완료)
  std::array<Eigen::Vector3d, kTotalSensors> sensor_positions_world{};

  // 3개 tip_link의 z축 방향 (빔 방향 = 표면 법선 추정의 반대)
  std::array<Eigen::Vector3d, kNumFingers> beam_directions_world{};

  // 6개 표면 접촉점 월드 좌표: p = sensor_pos + d * beam_dir
  std::array<Eigen::Vector3d, kTotalSensors> surface_points_world{};

  // 3개 로컬 곡률 (각 손가락의 A/B 거리차 기반)
  std::array<double, kNumFingers> local_curvatures{};

  // 곡률 유효성 (양쪽 센서 모두 valid일 때만 true)
  std::array<bool, kNumFingers> curvature_valid{};
};

// ── 형상 primitive 타입 ──────────────────────────────────────────────────────

enum class ShapeType : uint8_t {
  kUnknown  = 0,
  kPlane    = 1,
  kSphere   = 2,
  kCylinder = 3,
  kBox      = 4
};

[[nodiscard]] constexpr std::string_view ShapeTypeToString(ShapeType t) noexcept {
  switch (t) {
    case ShapeType::kPlane:    return "PLANE";
    case ShapeType::kSphere:   return "SPHERE";
    case ShapeType::kCylinder: return "CYLINDER";
    case ShapeType::kBox:      return "BOX";
    default:                   return "UNKNOWN";
  }
}

// ── 형상 추정 결과 ───────────────────────────────────────────────────────────

struct alignas(32) ShapeEstimate {
  uint64_t timestamp_ns{0};
  ShapeType type{ShapeType::kUnknown};
  double confidence{0.0};  // [0, 1]

  // Primitive 파라미터 (타입에 따라 해석)
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};     // cylinder 축 또는 plane 법선
  double radius{0.0};                                   // sphere/cylinder 반지름
  Eigen::Vector3d dimensions{Eigen::Vector3d::Zero()};  // box (w, h, d)

  uint32_t num_points_used{0};
};

// ── Voxel 포인트 클라우드용 포인트 ───────────────────────────────────────────

struct PointWithNormal {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
  double curvature{0.0};
  uint64_t timestamp_ns{0};
};

// ── 돌출 구조 탐지 ──────────────────────────────────────────────────────────

// 단일 돌출 구조 정보
struct Protuberance {
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};   // 돌출부 중심 (월드 프레임)
  Eigen::Vector3d direction{Eigen::Vector3d::Zero()};  // 돌출 방향 (표면 바깥)
  double extent_along_surface{0.0};  // 표면 방향 크기 [m]
  double protrusion_depth{0.0};      // 돌출 깊이 [m]
  double confidence{0.0};            // [0, 1]
  uint32_t num_points{0};            // 클러스터 포인트 수
  bool has_gap{false};               // gap 패턴 동반 여부
};

// 돌출 구조 탐지 결과
struct ProtuberanceResult {
  bool detected{false};
  std::vector<Protuberance> protuberances;  // 다중 돌출 가능
  double base_residual_rms{0.0};            // 돌출부 제외 시 잔차 RMS [m]
};

// 탐지 설정
struct ProtuberanceConfig {
  // 잔차 클러스터 판정
  double residual_threshold{-0.005};     // [m] 음의 잔차 임계값
  uint32_t min_cluster_points{3};        // 최소 클러스터 포인트 수
  double cluster_radius{0.015};          // [m] 공간적 연결 반경

  // Gap 판정
  double gap_distance_jump{0.020};       // [m] gap 전후 거리 차이 임계값
  uint32_t min_gap_invalid_count{2};     // 연속 invalid 최소 수

  // 곡률 급변 판정
  double curvature_jump_threshold{15.0}; // [1/m] 곡률 급변 임계값

  // Gap-클러스터 연결
  double gap_cluster_association_radius{0.020};  // [m]

  // 신뢰도 가중치
  double weight_num_points{0.3};
  double weight_depth{0.4};
  double weight_gap{0.3};
};

}  // namespace shape_estimation
