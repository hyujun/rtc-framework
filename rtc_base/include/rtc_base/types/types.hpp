#ifndef RTC_BASE_TYPES_HPP_
#define RTC_BASE_TYPES_HPP_

// Shared compile-time constants and plain-data structures used by all packages
// in the RTC framework.

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

namespace rtc {

// ── Hardware cache line size (shared across all threading primitives) ─────────
// Use a fixed constant to avoid -Winterference-size across compiler/CPU combos.
inline constexpr std::size_t kCacheLineSize = 64;

// ── Compile-time constants ─────────────────────────────────────────────────────
inline constexpr int kNumRobotJoints = 6;  // default channel count (UR5e); runtime count from YAML
inline constexpr int kMaxRobotDOF            = 12;   // max joints for generic robots
inline constexpr int kNumHandMotors          = 10;  // default channel count; runtime count from YAML
inline constexpr int kMaxDeviceChannels      = 64;   // max channels for generic devices
inline constexpr int kMaxSensorChannels      = 128;  // max sensor data channels
inline constexpr int kMaxInferenceValues     = 64;   // max inference output values

// Fingertip 수: YAML에서 런타임 설정 가능. 배열 크기는 kMaxFingertips로 고정.
inline constexpr int kDefaultNumFingertips   = 4;    // YAML 미설정 시 기본값
inline constexpr int kMaxFingertips          = 8;    // 배열 상한 (RT 경로 힙 할당 방지)

// Fingertip sensor layout per fingertip (packet contains 16 uint32 values):
//   barometer[8] + reserved[5] (skipped) + tof[3]
// Only 11 useful values are stored (reserved is discarded).
inline constexpr int kBarometerCount              = 8;
inline constexpr int kReservedCount               = 5;   // in packet only, not stored
inline constexpr int kTofCount                    = 3;
inline constexpr int kSensorDataPerPacket         = kBarometerCount + kReservedCount + kTofCount;  // 16
inline constexpr int kSensorValuesPerFingertip    = kBarometerCount + kTofCount;                   // 11
inline constexpr int kMaxHandSensors              = kMaxFingertips * kSensorValuesPerFingertip;    // 88

// 기본값 기반 상수 (하위 호환)
inline constexpr int kNumFingertips               = kDefaultNumFingertips;                         // 4
inline constexpr int kNumHandSensors              = kNumFingertips * kSensorValuesPerFingertip;    // 44

// Legacy alias — downstream code still references kNumHandJoints.
inline constexpr int kNumHandJoints = kNumHandMotors;

// ── Default joint/motor/fingertip names ─────────────────────────────────────
// YAML 미설정 시 사용되는 기본 이름. URDF/XML 검증의 기준.
inline const std::vector<std::string> kDefaultRobotJointNames = {
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
};

inline const std::vector<std::string> kDefaultHandMotorNames = {
  "thumb_cmc_aa", "thumb_cmc_fe", "thumb_mcp_fe",
  "index_mcp_aa", "index_mcp_fe", "index_dip_fe",
  "middle_mcp_aa", "middle_mcp_fe", "middle_dip_fe",
  "ring_mcp_fe"
};

inline const std::vector<std::string> kDefaultFingertipNames = {
  "thumb", "index", "middle", "ring"
};

// ── C++20 Concepts ─────────────────────────────────────────────────────────────
// Constrains template parameters to floating-point types (double, float, etc.).
// Used for gain parameters and filter coefficients in downstream packages
// (e.g. bessel_filter.hpp, kalman_filter.hpp).
template <typename T>
concept FloatingPointType = std::floating_point<T>;

// Constrains types to be usable in lock-free primitives (SeqLock, SPSC buffers).
template <typename T>
concept TriviallyCopyableType = std::is_trivially_copyable_v<T>;

// Fingertip F/T inference 관련 상수
// Output layout: [contact_prob(1), F(3), u(3)] = 7
// (3-head model: output0=contact logit→sigmoid, output1=F, output2=u)
inline constexpr int kFTValuesPerFingertip = 7;
inline constexpr int kFTInputSize          = 2 * kBarometerCount;  // baro(8) + delta(8) = 16
inline constexpr int kFTHistoryLength      = 12;                   // FIFO history rows for ONNX input

// ── Data structures (aggregate, zero-initialised by default) ──────────────────

// Fingertip F/T 추론 결과 (SeqLock 호환: trivially_copyable)
struct FingertipFTState {
  static constexpr int kMaxFTValues = kMaxFingertips * kFTValuesPerFingertip;  // 56
  std::array<float, kMaxFTValues> ft_data{};
  std::array<bool, kMaxFingertips> per_fingertip_valid{};  // per-fingertip inference ready
  int  num_fingertips{0};
  bool valid{false};
};

// Legacy structs — used by ur5e_hand_driver and other packages that haven't
// migrated to the unified DeviceState yet.  Will be removed in PR3.
struct RobotState {
  std::array<double, kNumRobotJoints> positions{};
  std::array<double, kNumRobotJoints> velocities{};
  std::array<double, kNumRobotJoints> torques{};
  std::array<double, 3>               tcp_position{};
  double   dt{0.002};
  uint64_t iteration{0};
};

struct HandState {
  std::array<float, kNumHandMotors>    motor_positions{};
  std::array<float, kNumHandMotors>    motor_velocities{};
  std::array<int32_t, kMaxHandSensors> sensor_data{};
  std::array<int32_t, kMaxHandSensors> sensor_data_raw{};
  int  num_fingertips{kDefaultNumFingertips};
  bool valid{false};
};

// Unified device state — used for all device groups (robot arm, hand, gripper, …)
struct DeviceState {
  int num_channels{0};
  std::array<double, kMaxDeviceChannels> positions{};
  std::array<double, kMaxDeviceChannels> velocities{};
  std::array<double, kMaxDeviceChannels> efforts{};          // torques for robot arm
  std::array<int32_t, kMaxSensorChannels> sensor_data{};     // post-filter
  std::array<int32_t, kMaxSensorChannels> sensor_data_raw{}; // pre-filter
  int num_sensor_channels{0};
  // Inference (force/displacement per fingertip)
  std::array<float, kMaxInferenceValues> inference_data{};   // [contact,F(3),u(3)] × fingertips
  std::array<bool, kMaxFingertips> inference_enable{};       // per-fingertip flag
  int num_inference_fingertips{0};
  bool valid{false};
};

struct ControllerState {
  static constexpr int kMaxDevices = 4;
  std::array<DeviceState, kMaxDevices> devices{};
  int      num_devices{0};
  double   dt{0.002};
  uint64_t iteration{0};
};

enum class CommandType { kPosition, kTorque };
enum class GoalType : uint8_t { kJoint, kTask };

[[nodiscard]] inline constexpr const char * GoalTypeToString(GoalType g) noexcept {
  switch (g) {
    case GoalType::kJoint: return "joint";
    case GoalType::kTask:  return "task";
  }
  return "unknown";
}

// Unified device output — per-device commands, goals, and trajectory data
struct DeviceOutput {
  int num_channels{0};
  std::array<double, kMaxDeviceChannels> commands{};
  std::array<double, kMaxDeviceChannels> goal_positions{};
  std::array<double, kMaxDeviceChannels> target_positions{};    // controller-specific target
  std::array<double, kMaxDeviceChannels> target_velocities{};   // controller-specific target vel
  std::array<double, kMaxDeviceChannels> trajectory_positions{}; // pure trajectory reference position
  std::array<double, kMaxDeviceChannels> trajectory_velocities{};// pure trajectory reference velocity
  GoalType goal_type{GoalType::kJoint};
};

struct ControllerOutput {
  static constexpr int kMaxDevices = 4;
  std::array<DeviceOutput, kMaxDevices> devices{};
  int num_devices{0};

  // Shared fields (not per-device)
  std::array<double, 6> actual_task_positions{};  // TCP FK result
  std::array<double, 6> task_goal_positions{};    // task-space goal target from GUI
  bool        valid{true};
  CommandType command_type{CommandType::kPosition};
};

// ── Per-device name + URDF configuration ─────────────────────────────────────

struct DeviceUrdfConfig {
  std::string package;     // ament package name (e.g. "ur5e_description")
  std::string path;        // relative to package share dir (e.g. "robots/ur5e/urdf/ur5e.urdf")
  std::string root_link;   // kinematic chain root link name
  std::string tip_link;    // end-effector link name (for FK/IK frame)
};

struct DeviceJointLimits {
  std::vector<double> max_velocity;      // per-joint (rad/s)
  std::vector<double> max_acceleration;  // per-joint (rad/s²), optional
  std::vector<double> max_torque;        // per-joint (Nm), optional
  std::vector<double> position_lower;    // per-joint lower bound (rad)
  std::vector<double> position_upper;    // per-joint upper bound (rad)
};

struct DeviceNameConfig {
  std::string device_name;
  std::vector<std::string> joint_state_names;
  std::vector<std::string> joint_command_names;  // empty → defaults to joint_state_names
  std::vector<std::string> sensor_names;
  std::optional<DeviceUrdfConfig> urdf;          // nullopt if no URDF for this device
  std::optional<DeviceJointLimits> joint_limits;  // nullopt if no limits configured
};

// ── Topic configuration for per-controller subscribe/publish routing ─────────

enum class SubscribeRole {
  kState,           // sensor_msgs/JointState (ur5e & hand 공통)
  kSensorState,     // Float64MultiArray (센서 전용, e.g. 촉각)
  kTarget,          // Float64MultiArray (외부 목표)
};

enum class PublishRole {
  // Control Command
  kJointCommand,       // rtc_msgs/JointCommand (통합 명령)
  kRos2Command,        // Float64MultiArray (ros2_control 전용)
  // GUI / Monitoring
  kGuiPosition,        // rtc_msgs/GuiPosition (joint_pos + task_pos)
  // Topic-based State/Command/Goal/Log
  kRobotTarget,        // rtc_msgs/RobotTarget (joint/task 목표)
  kDeviceStateLog,     // rtc_msgs/DeviceStateLog (통합 상태 로그)
  kDeviceSensorLog,    // rtc_msgs/DeviceSensorLog (센서 + inference 로그)
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

// ── Device topic grouping ────────────────────────────────────────────────────

struct DeviceTopicGroup {
  std::vector<SubscribeTopicEntry> subscribe;
  std::vector<PublishTopicEntry> publish;
};

// Dynamic topic configuration: groups keyed by device name ("ur5e", "hand", …)
// Uses a vector of pairs to preserve YAML insertion order.  Device indices
// throughout the system (ControllerState, ControllerOutput, LogEntry, CSV)
// are derived from the iteration order of this container, so alphabetical
// reordering (as std::map would do) breaks the mapping.
struct TopicConfig {
  std::vector<std::pair<std::string, DeviceTopicGroup>> groups;

  // Insert-or-access by name (preserves insertion order for new entries).
  DeviceTopicGroup& operator[](const std::string& name) {
    for (auto& [n, g] : groups) { if (n == name) return g; }
    groups.emplace_back(name, DeviceTopicGroup{});
    return groups.back().second;
  }

  // True if the named group exists and has at least one topic entry.
  [[nodiscard]] bool HasGroup(const std::string& name) const noexcept {
    for (const auto& [n, g] : groups) {
      if (n == name) return !g.subscribe.empty() || !g.publish.empty();
    }
    return false;
  }

  // True if the named group has a subscribe entry with the given role.
  [[nodiscard]] bool HasSubscribeRole(
      const std::string& group_name, SubscribeRole role) const noexcept {
    for (const auto& [n, g] : groups) {
      if (n != group_name) continue;
      for (const auto& e : g.subscribe) {
        if (e.role == role) return true;
      }
      return false;
    }
    return false;
  }

  // Returns the topic name for the first subscribe entry matching the role,
  // or an empty string if not found.
  //
  // WARNING: NOT RT-safe — returns std::string (potential heap allocation).
  // Call only during initialisation, not from the 500 Hz control loop.
  [[nodiscard]] std::string GetSubscribeTopicName(
      const std::string& group_name, SubscribeRole role) const {
    for (const auto& [n, g] : groups) {
      if (n != group_name) continue;
      for (const auto& e : g.subscribe) {
        if (e.role == role) return e.topic_name;
      }
      return {};
    }
    return {};
  }
};

// ── Role → string conversion (for ROS2 parameter exposure) ──────────────────

[[nodiscard]] inline constexpr const char * SubscribeRoleToString(SubscribeRole role) noexcept {
  switch (role) {
    case SubscribeRole::kState:       return "state";
    case SubscribeRole::kSensorState: return "sensor_state";
    case SubscribeRole::kTarget:      return "target";
  }
  return "unknown";
}

[[nodiscard]] inline constexpr const char * PublishRoleToString(PublishRole role) noexcept {
  switch (role) {
    case PublishRole::kJointCommand:    return "joint_command";
    case PublishRole::kRos2Command:     return "ros2_command";
    case PublishRole::kGuiPosition:     return "gui_position";
    case PublishRole::kRobotTarget:     return "robot_target";
    case PublishRole::kDeviceStateLog:  return "device_state_log";
    case PublishRole::kDeviceSensorLog: return "device_sensor_log";
  }
  return "unknown";
}

}  // namespace rtc

#endif  // RTC_BASE_TYPES_HPP_
