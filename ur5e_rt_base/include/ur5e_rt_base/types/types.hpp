#ifndef UR5E_RT_BASE_TYPES_HPP_
#define UR5E_RT_BASE_TYPES_HPP_

// Shared compile-time constants and plain-data structures used by all packages
// in the ur5e RT controller stack.
//
// Namespace ur5e_rt_controller is kept consistent across all packages so that
// downstream code (controllers, nodes) needs no namespace changes after the
// multi-package split.

#include <array>
#include <concepts>
#include <cstdint>
#include <string>
#include <vector>

namespace ur5e_rt_controller {

// ── Compile-time constants ─────────────────────────────────────────────────────
inline constexpr int kNumRobotJoints = 6;
inline constexpr int kNumHandMotors          = 10;
inline constexpr int kNumFingertips          = 4;

// Fingertip sensor layout per fingertip (packet contains 16 uint32 values):
//   barometer[8] + reserved[5] (skipped) + tof[3]
// Only 11 useful values are stored (reserved is discarded).
inline constexpr int kBarometerCount              = 8;
inline constexpr int kReservedCount               = 5;   // in packet only, not stored
inline constexpr int kTofCount                    = 3;
inline constexpr int kSensorDataPerPacket         = kBarometerCount + kReservedCount + kTofCount;  // 16
inline constexpr int kSensorValuesPerFingertip    = kBarometerCount + kTofCount;                   // 11
inline constexpr int kNumHandSensors              = kNumFingertips * kSensorValuesPerFingertip;    // 44

// Legacy alias — downstream code still references kNumHandJoints.
inline constexpr int kNumHandJoints = kNumHandMotors;

// ── C++20 Concepts ─────────────────────────────────────────────────────────────
// Constrains template parameters to floating-point types (double, float, etc.).
// Used for gain parameters and filter coefficients.
template <typename T>
concept FloatingPointType = std::floating_point<T>;

// ── Data structures (aggregate, zero-initialised by default) ──────────────────
struct RobotState {
  std::array<double, kNumRobotJoints> positions{};
  std::array<double, kNumRobotJoints> velocities{};
  std::array<double, kNumRobotJoints> torques{};       ///< optional — zero if unavailable
  std::array<double, 3>               tcp_position{};
  double   dt{0.002};
  uint64_t iteration{0};
};

struct HandState {
  std::array<float, kNumHandMotors>  motor_positions{};
  std::array<float, kNumHandMotors>  motor_velocities{};
  std::array<float, kNumHandSensors> sensor_data{};   // 4 fingertips × 10 values
  bool valid{false};
};

struct ControllerState {
  RobotState robot{};
  HandState  hand{};
  // NOTE: dt and iteration duplicate robot.dt and robot.iteration.
  // Both MUST be kept in sync when assembling ControllerState.
  // Controllers may read either field; inconsistency causes subtle bugs.
  double   dt{0.002};
  uint64_t iteration{0};
};

enum class CommandType { kPosition, kTorque };

struct ControllerOutput {
  std::array<double, kNumRobotJoints> robot_commands{};
  std::array<float, kNumHandMotors>   hand_commands{};
  std::array<double, kNumRobotJoints> actual_target_positions{};
  std::array<double, 6>               actual_task_positions{};
  bool        valid{true};
  CommandType command_type{CommandType::kPosition};

  // ── Extended fields for logging (v5.9.0) ──────────────────────────────────
  // Goal = final target set by SetRobotTarget() (step-like)
  // Target = trajectory-interpolated position (smooth)
  // Target velocity = trajectory-interpolated velocity
  std::array<double, kNumRobotJoints> goal_positions{};
  std::array<double, kNumRobotJoints> target_velocities{};
  std::array<float, kNumHandMotors>   hand_goal_positions{};
};

// ── Topic configuration for per-controller subscribe/publish routing ─────────

enum class SubscribeRole {
  kJointState,      // 카테고리 2: 로봇 현재 상태 (sensor_msgs/JointState)
  kHandState,       // 카테고리 2: 핸드 현재 상태 (Float64MultiArray)
  kGoal,            // 카테고리 1: 궤적 최종 목표 (Float64MultiArray)
};

enum class PublishRole {
  // 카테고리 3: Control Command
  kPositionCommand,    // 로봇 위치 명령 (Float64MultiArray)
  kTorqueCommand,      // 로봇 토크 명령 (Float64MultiArray)
  kHandCommand,        // 핸드 모터 명령 (Float64MultiArray)
  // 카테고리 4: Logging/Monitoring
  kTaskPosition,       // 현재 TCP 위치 (Float64MultiArray, size=6)
  kTrajectoryState,    // 궤적 보간 상태 (Float64MultiArray, size=18: goal[6]+traj_pos[6]+traj_vel[6])
  kControllerState,    // 제어기 내부 상태 (Float64MultiArray, size=18: actual_pos[6]+actual_vel[6]+command[6])
};

struct SubscribeTopicEntry {
  std::string topic_name;
  SubscribeRole role;
};

struct PublishTopicEntry {
  std::string topic_name;
  PublishRole role;
  int data_size{0};  // pre-allocate message size (0 = use default for role)
};

struct TopicConfig {
  std::vector<SubscribeTopicEntry> subscribe;
  std::vector<PublishTopicEntry> publish;
};

// ── Role → string conversion (for ROS2 parameter exposure) ──────────────────

inline const char * SubscribeRoleToString(SubscribeRole role) noexcept {
  switch (role) {
    case SubscribeRole::kJointState: return "joint_state";
    case SubscribeRole::kHandState:  return "hand_state";
    case SubscribeRole::kGoal:       return "goal";
  }
  return "unknown";
}

inline const char * PublishRoleToString(PublishRole role) noexcept {
  switch (role) {
    case PublishRole::kPositionCommand: return "position_command";
    case PublishRole::kTorqueCommand:   return "torque_command";
    case PublishRole::kHandCommand:     return "hand_command";
    case PublishRole::kTaskPosition:    return "task_position";
    case PublishRole::kTrajectoryState: return "trajectory_state";
    case PublishRole::kControllerState: return "controller_state";
  }
  return "unknown";
}

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_BASE_TYPES_HPP_
