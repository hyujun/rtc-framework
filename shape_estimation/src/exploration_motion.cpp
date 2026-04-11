#include "shape_estimation/exploration_motion.hpp"
#include "shape_estimation/shape_logging.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

namespace shape_estimation {

namespace {
auto logger() { return ::rtc::shape::logging::ExploreLogger(); }
}  // namespace

// ── 유틸 ────────────────────────────────────────────────────────────────────

static constexpr double kDegToRad = M_PI / 180.0;

/// 3D 벡터 노름
static double Norm3(const std::array<double, 6>& a, const std::array<double, 6>& b) {
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  const double dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// pose의 위치 부분을 Eigen::Vector3d로 변환
static Eigen::Vector3d PosePosition(const std::array<double, 6>& pose) {
  return {pose[0], pose[1], pose[2]};
}

// ── ExplorationMotionGenerator ──────────────────────────────────────────────

ExplorationMotionGenerator::ExplorationMotionGenerator(const ExplorationConfig& config)
    : config_(config) {}

void ExplorationMotionGenerator::Start(
    const std::array<double, 6>& current_pose,
    const std::array<double, 3>& object_position) {
  phase_ = ExplorePhase::kApproach;
  stats_ = {};
  object_position_ = object_position;
  start_pose_ = current_pose;
  phase_elapsed_ = 0.0;

  // 현재 EE에서 물체 방향 벡터 계산
  const Eigen::Vector3d ee_pos = PosePosition(current_pose);
  const Eigen::Vector3d obj_pos(object_position[0], object_position[1], object_position[2]);
  const Eigen::Vector3d dir = obj_pos - ee_pos;
  const double dist = dir.norm();
  if (dist > 1e-6) {
    approach_direction_ = dir / dist;
  } else {
    approach_direction_ = Eigen::Vector3d::UnitX();
  }

  RCLCPP_INFO(logger(),
              "탐색 시작: object=[%.3f, %.3f, %.3f], "
              "approach_dir=[%.3f, %.3f, %.3f], dist=%.3f",
              object_position[0], object_position[1], object_position[2],
              approach_direction_.x(), approach_direction_.y(),
              approach_direction_.z(), dist);

  // Sweep 상태 초기화
  sweep_position_ = 0.0;
  sweep_direction_ = 1;
  sweep_x_done_ = false;

  // Tilt 상태 초기화
  tilt_step_count_ = 0;
}

void ExplorationMotionGenerator::Abort() {
  if (phase_ != ExplorePhase::kIdle &&
      phase_ != ExplorePhase::kSucceeded &&
      phase_ != ExplorePhase::kFailed) {
    phase_ = ExplorePhase::kAborted;
  }
}

StepResult ExplorationMotionGenerator::Step(
    const ToFSnapshot& snapshot,
    const ShapeEstimate& current_estimate,
    const std::array<double, 6>& current_pose,
    double dt) {

  StepResult result;
  result.phase = phase_;

  // 종료 상태이면 바로 반환
  if (phase_ == ExplorePhase::kIdle ||
      phase_ == ExplorePhase::kSucceeded ||
      phase_ == ExplorePhase::kFailed ||
      phase_ == ExplorePhase::kAborted) {
    result.status_message = std::string(ExplorePhaseToString(phase_));
    return result;
  }

  // 전체 시간 업데이트
  stats_.elapsed_sec += dt;
  stats_.total_snapshots++;
  phase_elapsed_ += dt;

  // 전체 타임아웃 검사
  if (stats_.elapsed_sec >= config_.max_total_time_sec) {
    phase_ = ExplorePhase::kFailed;
    result.phase = phase_;
    result.status_message = "max_total_time exceeded";
    RCLCPP_ERROR(logger(),
                 "전체 시간 초과 (%.1fs >= %.1fs) → FAILED",
                 stats_.elapsed_sec, config_.max_total_time_sec);
    return result;
  }

  // Phase별 처리
  switch (phase_) {
    case ExplorePhase::kApproach: {
      if (ShouldTransitionToServo(snapshot)) {
        phase_ = ExplorePhase::kServo;
        phase_elapsed_ = 0.0;
        result.phase = phase_;
        result.status_message = "ToF sensors detected → SERVO";
        RCLCPP_INFO(logger(), "Phase: APPROACH → SERVO (%.1fs)", phase_elapsed_);
        // Servo로 전이 즉시 servo goal 생성
        result.goal = GenerateServo(snapshot, current_pose);
      } else if (phase_elapsed_ >= config_.approach_timeout_sec) {
        phase_ = ExplorePhase::kFailed;
        result.phase = phase_;
        result.status_message = "approach timeout";
        RCLCPP_WARN(logger(), "Approach 타임아웃 (%.1fs) → FAILED",
                    config_.approach_timeout_sec);
      } else {
        result.goal = GenerateApproach(snapshot, current_pose);
        result.status_message = "approaching object";
      }
      break;
    }

    case ExplorePhase::kServo: {
      if (IsServoConverged(snapshot)) {
        phase_ = ExplorePhase::kSweepX;
        phase_elapsed_ = 0.0;
        sweep_position_ = 0.0;
        sweep_direction_ = 1;
        result.phase = phase_;
        result.status_message = "servo converged → SWEEP_X";
        RCLCPP_INFO(logger(), "Phase: SERVO → SWEEP_X (converged, %.1fs)", phase_elapsed_);
        result.goal = GenerateSweep(snapshot, current_pose);
      } else if (phase_elapsed_ >= config_.servo_timeout_sec) {
        // Servo 타임아웃이어도 sweep으로 진행
        phase_ = ExplorePhase::kSweepX;
        phase_elapsed_ = 0.0;
        sweep_position_ = 0.0;
        sweep_direction_ = 1;
        result.phase = phase_;
        result.status_message = "servo timeout → SWEEP_X";
        RCLCPP_WARN(logger(), "Servo 타임아웃 (%.1fs) → SWEEP_X",
                    config_.servo_timeout_sec);
        result.goal = GenerateSweep(snapshot, current_pose);
      } else {
        result.goal = GenerateServo(snapshot, current_pose);
        result.status_message = "servo to target distance";
      }
      break;
    }

    case ExplorePhase::kSweepX: {
      if (IsSweepAtEdge() && sweep_x_done_) {
        // X sweep 완료, Y sweep으로 전이
        phase_ = ExplorePhase::kSweepY;
        phase_elapsed_ = 0.0;
        sweep_position_ = 0.0;
        sweep_direction_ = 1;
        result.phase = phase_;
        result.status_message = "sweep X done → SWEEP_Y";
        RCLCPP_INFO(logger(), "Phase: SWEEP_X → SWEEP_Y");
      }
      result.goal = GenerateSweep(snapshot, current_pose);
      if (result.status_message.empty()) {
        result.status_message = "sweeping X";
      }
      break;
    }

    case ExplorePhase::kSweepY: {
      if (IsSweepAtEdge()) {
        // Y sweep 완료, Tilt으로 전이
        phase_ = ExplorePhase::kTilt;
        phase_elapsed_ = 0.0;
        tilt_step_count_ = 0;
        result.phase = phase_;
        result.status_message = "sweep Y done → TILT";
        RCLCPP_INFO(logger(), "Phase: SWEEP_Y → TILT");
      }
      result.goal = GenerateSweep(snapshot, current_pose);
      if (result.status_message.empty()) {
        result.status_message = "sweeping Y";
      }
      break;
    }

    case ExplorePhase::kTilt: {
      if (tilt_step_count_ >= config_.tilt_steps) {
        phase_ = ExplorePhase::kEvaluate;
        phase_elapsed_ = 0.0;
        result.phase = phase_;
        result.status_message = "tilt done → EVALUATE";
        RCLCPP_INFO(logger(), "Phase: TILT → EVALUATE");
      } else {
        result.goal = GenerateTilt(snapshot, current_pose);
        result.status_message = "tilting";
      }
      break;
    }

    case ExplorePhase::kEvaluate: {
      // 추정 결과 평가
      if (current_estimate.confidence >= config_.confidence_threshold &&
          current_estimate.num_points_used >= config_.min_points_for_success) {
        phase_ = ExplorePhase::kSucceeded;
        result.phase = phase_;
        std::ostringstream oss;
        oss << "shape estimation succeeded: "
            << ShapeTypeToString(current_estimate.type)
            << " (conf=" << static_cast<int>(current_estimate.confidence * 100) << "%)";
        result.status_message = oss.str();
        RCLCPP_INFO(logger(), "탐색 성공: %s", result.status_message.c_str());
      } else if (stats_.sweep_cycles_completed >= config_.max_sweep_cycles) {
        phase_ = ExplorePhase::kFailed;
        result.phase = phase_;
        result.status_message = "max sweep cycles reached";
        RCLCPP_WARN(logger(),
                    "최대 sweep cycle 도달 (%u >= %u) → FAILED",
                    stats_.sweep_cycles_completed, config_.max_sweep_cycles);
      } else {
        // 추가 sweep 사이클 실행
        stats_.sweep_cycles_completed++;
        phase_ = ExplorePhase::kSweepX;
        phase_elapsed_ = 0.0;
        sweep_position_ = 0.0;
        sweep_direction_ = 1;
        sweep_x_done_ = false;
        result.phase = phase_;
        std::ostringstream oss;
        oss << "need more data, cycle " << static_cast<int>(stats_.sweep_cycles_completed)
            << " → SWEEP_X";
        result.status_message = oss.str();
        RCLCPP_WARN(logger(),
                    "Evaluate: confidence=%.2f, points=%u → 추가 sweep (cycle %u)",
                    current_estimate.confidence, current_estimate.num_points_used,
                    stats_.sweep_cycles_completed);
      }
      break;
    }

    default:
      break;
  }

  // 유효한 goal 생성 시 검증
  if (result.goal.valid) {
    if (!ValidateGoal(result.goal, current_pose, snapshot)) {
      result.goal.valid = false;
      result.status_message += " (goal rejected by safety)";
    } else {
      stats_.goals_sent++;
    }
  }

  return result;
}

// ── Phase별 Goal 생성 ──────────────────────────────────────────────────────

TaskSpaceGoal ExplorationMotionGenerator::GenerateApproach(
    const ToFSnapshot& /*snapshot*/,
    const std::array<double, 6>& current) {
  TaskSpaceGoal goal;
  goal.pose = current;

  // 물체 방향으로 step_size만큼 전진
  goal.pose[0] += approach_direction_.x() * config_.approach_step_size;
  goal.pose[1] += approach_direction_.y() * config_.approach_step_size;
  goal.pose[2] += approach_direction_.z() * config_.approach_step_size;
  goal.valid = true;

  return goal;
}

TaskSpaceGoal ExplorationMotionGenerator::GenerateServo(
    const ToFSnapshot& snapshot,
    const std::array<double, 6>& current) {
  TaskSpaceGoal goal;
  goal.pose = current;

  const double mean_dist = MeanValidDistance(snapshot);
  if (mean_dist <= 0.0) {
    // 유효한 센서가 없으면 approach 방향으로 계속 전진
    goal.pose[0] += approach_direction_.x() * config_.servo_max_step;
    goal.pose[1] += approach_direction_.y() * config_.servo_max_step;
    goal.pose[2] += approach_direction_.z() * config_.servo_max_step;
    goal.valid = true;
    return goal;
  }

  // 목표 거리와의 오차에 비례하여 접근/후퇴
  const double error = mean_dist - config_.servo_target_distance;
  double step = config_.servo_step_gain * error;
  step = std::clamp(step, -config_.servo_max_step, config_.servo_max_step);

  goal.pose[0] += approach_direction_.x() * step;
  goal.pose[1] += approach_direction_.y() * step;
  goal.pose[2] += approach_direction_.z() * step;
  goal.valid = true;

  static rclcpp::Clock steady_clock{RCL_STEADY_TIME};
  RCLCPP_DEBUG_THROTTLE(logger(), steady_clock,
                        ::rtc::shape::logging::kThrottleFastMs,
                        "Servo: mean_dist=%.4f, target=%.4f, step=%.4f",
                        mean_dist, config_.servo_target_distance, step);

  return goal;
}

TaskSpaceGoal ExplorationMotionGenerator::GenerateSweep(
    const ToFSnapshot& snapshot,
    const std::array<double, 6>& current) {
  TaskSpaceGoal goal;
  goal.pose = current;

  // Sweep 방향 결정 (approach_direction에 수직인 2개 축)
  // approach_direction이 z에 가까우면 x, y를 sweep 축으로 사용
  Eigen::Vector3d sweep_axis;
  if (phase_ == ExplorePhase::kSweepX) {
    // approach_direction과 world-Z의 외적 → lateral 방향
    sweep_axis = approach_direction_.cross(Eigen::Vector3d::UnitZ());
    if (sweep_axis.norm() < 1e-6) {
      sweep_axis = approach_direction_.cross(Eigen::Vector3d::UnitY());
    }
    sweep_axis.normalize();
  } else {
    // SweepY: approach_direction과 sweep_x 방향의 외적
    Eigen::Vector3d sweep_x = approach_direction_.cross(Eigen::Vector3d::UnitZ());
    if (sweep_x.norm() < 1e-6) {
      sweep_x = approach_direction_.cross(Eigen::Vector3d::UnitY());
    }
    sweep_x.normalize();
    sweep_axis = approach_direction_.cross(sweep_x);
    sweep_axis.normalize();
  }

  // Sweep 위치 업데이트
  sweep_position_ += static_cast<double>(sweep_direction_) * config_.sweep_step_size;

  // Edge에서 방향 반전
  const double half_width = config_.sweep_width / 2.0;
  if (sweep_position_ >= half_width) {
    sweep_position_ = half_width;
    sweep_direction_ = -1;
    if (phase_ == ExplorePhase::kSweepX) {
      sweep_x_done_ = true;
    }
  } else if (sweep_position_ <= -half_width) {
    sweep_position_ = -half_width;
    sweep_direction_ = 1;
  }

  // Lateral 이동
  goal.pose[0] += sweep_axis.x() * static_cast<double>(sweep_direction_) * config_.sweep_step_size;
  goal.pose[1] += sweep_axis.y() * static_cast<double>(sweep_direction_) * config_.sweep_step_size;
  goal.pose[2] += sweep_axis.z() * static_cast<double>(sweep_direction_) * config_.sweep_step_size;

  // 표면 법선 방향 보정: ToF 거리를 기반으로 표면과의 거리 유지
  const double mean_dist = MeanValidDistance(snapshot);
  if (mean_dist > 0.0) {
    const double normal_error = mean_dist - config_.servo_target_distance;
    double normal_step = config_.sweep_normal_gain * normal_error;
    normal_step = std::clamp(normal_step, -config_.sweep_normal_max_step,
                             config_.sweep_normal_max_step);
    goal.pose[0] += approach_direction_.x() * normal_step;
    goal.pose[1] += approach_direction_.y() * normal_step;
    goal.pose[2] += approach_direction_.z() * normal_step;
  }

  goal.valid = true;
  return goal;
}

TaskSpaceGoal ExplorationMotionGenerator::GenerateTilt(
    const ToFSnapshot& /*snapshot*/,
    const std::array<double, 6>& current) {
  TaskSpaceGoal goal;
  goal.pose = current;

  // 사인파 틸트: roll과 pitch를 교대로 변경
  const double amp = config_.tilt_amplitude_deg * kDegToRad;
  const double phase_ratio = static_cast<double>(tilt_step_count_) /
                             static_cast<double>(std::max(config_.tilt_steps, 1u));
  const double angle = amp * std::sin(2.0 * M_PI * phase_ratio);

  // roll과 pitch에 적용 (start_pose 기준)
  if (tilt_step_count_ % 2 == 0) {
    goal.pose[3] = start_pose_[3] + angle;  // roll
  } else {
    goal.pose[4] = start_pose_[4] + angle;  // pitch
  }

  tilt_step_count_++;
  goal.valid = true;
  return goal;
}

// ── 안전 검증 ───────────────────────────────────────────────────────────────

bool ExplorationMotionGenerator::ValidateGoal(
    const TaskSpaceGoal& goal,
    const std::array<double, 6>& current,
    const ToFSnapshot& snapshot) const {
  // 1) max step size 검사
  const double step_dist = Norm3(goal.pose, current);
  if (step_dist > config_.max_step_size) {
    RCLCPP_WARN(logger(),
                "ValidateGoal: step_dist=%.4f > max=%.4f → 거부",
                step_dist, config_.max_step_size);
    return false;
  }

  // 2) min distance 검사: goal이 물체에 너무 가까운지
  const double mean_dist = MeanValidDistance(snapshot);
  if (mean_dist > 0.0) {
    // goal이 approach 방향으로 전진하는 양 계산
    const Eigen::Vector3d delta = PosePosition(goal.pose) - PosePosition(current);
    const double forward_step = delta.dot(approach_direction_);
    // 현재 ToF 거리에서 전진량을 빼면 예상 거리
    const double predicted_dist = mean_dist - forward_step;
    if (predicted_dist < config_.min_distance) {
      RCLCPP_WARN(logger(),
                  "ValidateGoal: predicted_dist=%.4f < min=%.4f → 거부",
                  predicted_dist, config_.min_distance);
      return false;
    }
  }

  return true;
}

// ── Phase 전이 판정 ─────────────────────────────────────────────────────────

bool ExplorationMotionGenerator::ShouldTransitionToServo(const ToFSnapshot& snapshot) const {
  return CountValidSensors(snapshot) >= config_.servo_min_valid_sensors;
}

bool ExplorationMotionGenerator::IsServoConverged(const ToFSnapshot& snapshot) const {
  const double mean_dist = MeanValidDistance(snapshot);
  if (mean_dist <= 0.0) {
    return false;
  }
  return std::abs(mean_dist - config_.servo_target_distance) < config_.servo_converge_tol;
}

bool ExplorationMotionGenerator::IsSweepAtEdge() const {
  const double half_width = config_.sweep_width / 2.0;
  return std::abs(sweep_position_) >= half_width - 1e-6;
}

bool ExplorationMotionGenerator::ShouldEvaluate() const {
  return tilt_step_count_ >= config_.tilt_steps;
}

// ── ToF 유틸 ────────────────────────────────────────────────────────────────

double ExplorationMotionGenerator::MeanValidDistance(const ToFSnapshot& snapshot) const {
  double sum = 0.0;
  int count = 0;
  for (int i = 0; i < kTotalSensors; ++i) {
    const auto idx = static_cast<size_t>(i);
    if (snapshot.readings[idx].valid) {
      sum += snapshot.readings[idx].distance_m;
      count++;
    }
  }
  return (count > 0) ? (sum / static_cast<double>(count)) : 0.0;
}

int ExplorationMotionGenerator::CountValidSensors(const ToFSnapshot& snapshot) const {
  int count = 0;
  for (int i = 0; i < kTotalSensors; ++i) {
    if (snapshot.readings[static_cast<size_t>(i)].valid) {
      count++;
    }
  }
  return count;
}

}  // namespace shape_estimation
