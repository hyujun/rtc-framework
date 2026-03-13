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
};

// ── Topic configuration for per-controller subscribe/publish routing ─────────

enum class SubscribeRole {
  kJointState,      // robot joint positions/velocities (sensor_msgs/JointState)
  kHandState,       // hand joint state (Float64MultiArray)
  kTarget,          // target positions (Float64MultiArray)
};

enum class PublishRole {
  kPositionCommand,  // robot position commands (Float64MultiArray)
  kTorqueCommand,    // robot torque commands (Float64MultiArray)
  kHandCommand,      // hand motor commands (Float64MultiArray)
  kTaskPosition,     // current task-space position (Float64MultiArray)
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

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_BASE_TYPES_HPP_
