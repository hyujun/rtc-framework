#pragma once

#include "shape_estimation/tof_shape_types.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <string_view>

namespace shape_estimation {

// ── SE3 목표 (DemoTaskController의 RobotTarget.task_target 호환) ─────────────

struct TaskSpaceGoal {
  std::array<double, 6> pose{};  // [x, y, z, roll, pitch, yaw]
  bool valid{false};
};

// ── 탐색 설정 (YAML 파라미터에서 로드) ──────────────────────────────────────

struct ExplorationConfig {
  // Phase 0: Approach
  double approach_step_size{0.005};      // [m]
  double approach_timeout_sec{5.0};

  // Phase 1: Servo
  double servo_target_distance{0.030};   // [m]
  double servo_step_gain{0.5};
  double servo_max_step{0.005};          // [m]
  double servo_converge_tol{0.003};      // [m]
  double servo_timeout_sec{3.0};
  uint8_t servo_min_valid_sensors{3};

  // Phase 2: Sweep
  double sweep_step_size{0.003};         // [m]
  double sweep_width{0.06};              // [m]
  double sweep_normal_gain{0.5};
  double sweep_normal_max_step{0.003};   // [m]

  // Phase 3: Tilt
  double tilt_amplitude_deg{15.0};       // [deg]
  uint32_t tilt_steps{10};

  // Safety
  double min_distance{0.005};            // [m]
  double max_step_size{0.010};           // [m]

  // Evaluation
  double confidence_threshold{0.8};
  uint32_t min_points_for_success{20};
  double max_total_time_sec{10.0};
  uint8_t max_sweep_cycles{3};

  // ── 센서 가중치: index/middle이 classification에 더 기여 ─────────────────
  // finger별 가중치 [thumb, index, middle] (거리 계산 시 사용)
  std::array<double, 3> finger_weights{0.2, 0.4, 0.4};

  // Servo 전이 조건: index/middle 중 최소 이 수만큼 finger가 pair-valid 필수
  // (pair-valid = 해당 finger의 A, B 센서 모두 valid → 곡률 계산 가능)
  uint8_t min_classification_fingers{1};

  // Evaluation: index/middle pair-valid 비율 최소 기준 (0.0~1.0)
  double min_classification_coverage{0.5};
};

// ── 탐색 FSM 상태 ──────────────────────────────────────────────────────────

enum class ExplorePhase : uint8_t {
  kIdle = 0,
  kApproach,     // 물체 방향 step-by-step 접근
  kServo,        // ToF 거리 기반 접근 서보
  kSweepX,       // X방향 표면 추종 sweep
  kSweepY,       // Y방향 표면 추종 sweep
  kTilt,         // 손목 틸트 스캔
  kEvaluate,     // 추정 결과 평가
  kSucceeded,
  kFailed,
  kAborted
};

[[nodiscard]] constexpr std::string_view ExplorePhaseToString(ExplorePhase p) noexcept {
  switch (p) {
    case ExplorePhase::kIdle:      return "IDLE";
    case ExplorePhase::kApproach:  return "APPROACH";
    case ExplorePhase::kServo:     return "SERVO";
    case ExplorePhase::kSweepX:    return "SWEEP_X";
    case ExplorePhase::kSweepY:    return "SWEEP_Y";
    case ExplorePhase::kTilt:      return "TILT";
    case ExplorePhase::kEvaluate:  return "EVALUATE";
    case ExplorePhase::kSucceeded: return "SUCCEEDED";
    case ExplorePhase::kFailed:    return "FAILED";
    case ExplorePhase::kAborted:   return "ABORTED";
  }
  return "UNKNOWN";
}

// ── Step 결과 ───────────────────────────────────────────────────────────────

struct StepResult {
  TaskSpaceGoal goal;
  ExplorePhase phase{ExplorePhase::kIdle};
  std::string status_message;
};

// ── 통계 ────────────────────────────────────────────────────────────────────

struct ExplorationStats {
  double elapsed_sec{0.0};
  uint32_t total_snapshots{0};
  uint32_t goals_sent{0};
  uint8_t sweep_cycles_completed{0};
};

// ── ExplorationMotionGenerator ──────────────────────────────────────────────

/// 탐색 모션 생성기: ROS 비의존 순수 C++
/// ToF 피드백 기반으로 SE3 waypoint를 생성하여 DemoTaskController에 전달.
class ExplorationMotionGenerator {
 public:
  explicit ExplorationMotionGenerator(const ExplorationConfig& config = {});

  /// 탐색 시작
  /// @param current_pose 현재 EE의 [x,y,z,r,p,y]
  /// @param object_position 물체의 대략적 위치 [x,y,z]
  void Start(const std::array<double, 6>& current_pose,
             const std::array<double, 3>& object_position);

  /// 탐색 중단
  void Abort();

  /// 매 사이클(~10Hz) 호출
  /// @param snapshot 현재 ToF 데이터
  /// @param current_estimate 현재 형상 추정 결과
  /// @param current_pose 현재 EE [x,y,z,r,p,y]
  /// @param dt 시간 간격 [s]
  [[nodiscard]] StepResult Step(
      const ToFSnapshot& snapshot,
      const ShapeEstimate& current_estimate,
      const std::array<double, 6>& current_pose,
      double dt);

  /// 현재 phase
  [[nodiscard]] ExplorePhase CurrentPhase() const noexcept { return phase_; }

  /// 통계
  [[nodiscard]] ExplorationStats Stats() const noexcept { return stats_; }

  /// 설정 변경
  void SetConfig(const ExplorationConfig& config) { config_ = config; }
  [[nodiscard]] const ExplorationConfig& GetConfig() const noexcept { return config_; }

 private:
  // Phase별 waypoint 생성
  TaskSpaceGoal GenerateApproach(const ToFSnapshot& snapshot,
                                 const std::array<double, 6>& current);
  TaskSpaceGoal GenerateServo(const ToFSnapshot& snapshot,
                              const std::array<double, 6>& current);
  TaskSpaceGoal GenerateSweep(const ToFSnapshot& snapshot,
                              const std::array<double, 6>& current);
  TaskSpaceGoal GenerateTilt(const ToFSnapshot& snapshot,
                             const std::array<double, 6>& current);

  // 안전 검증: min_distance, max_step_size 클램핑
  bool ValidateGoal(const TaskSpaceGoal& goal,
                    const std::array<double, 6>& current,
                    const ToFSnapshot& snapshot) const;

  // Phase 전이 판정
  bool ShouldTransitionToServo(const ToFSnapshot& snapshot) const;
  bool IsServoConverged(const ToFSnapshot& snapshot) const;
  bool IsSweepAtEdge() const;
  bool ShouldEvaluate() const;

  // ToF 유틸
  double MeanValidDistance(const ToFSnapshot& snapshot) const;
  int CountValidSensors(const ToFSnapshot& snapshot) const;

  /// finger_weights 기반 가중 평균 거리 (index/middle 우선)
  double WeightedMeanDistance(const ToFSnapshot& snapshot) const;

  /// index/middle 중 A+B 모두 valid인 finger 수 (곡률 계산 가능 = classification 유효)
  int CountClassificationFingerPairs(const ToFSnapshot& snapshot) const;

  ExplorationConfig config_;
  ExplorePhase phase_{ExplorePhase::kIdle};
  ExplorationStats stats_{};

  // 탐색 상태
  std::array<double, 3> object_position_{};
  std::array<double, 6> start_pose_{};
  Eigen::Vector3d approach_direction_{Eigen::Vector3d::Zero()};

  // Sweep 상태
  double sweep_position_{0.0};        // sweep 진행 위치 [-width/2, +width/2]
  int sweep_direction_{1};            // +1 or -1
  bool sweep_x_done_{false};

  // Tilt 상태
  uint32_t tilt_step_count_{0};

  // Phase별 경과 시간
  double phase_elapsed_{0.0};
};

}  // namespace shape_estimation
