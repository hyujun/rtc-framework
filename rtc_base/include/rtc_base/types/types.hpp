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

// ── Hardware cache line size (shared across all threading primitives)
// ───────── Use a fixed constant to avoid -Winterference-size across
// compiler/CPU combos.
inline constexpr std::size_t kCacheLineSize = 64;

// ── Compile-time constants
// ─────────────────────────────────────────────────────
//
// Naming convention (read carefully — the distinction is load-bearing for
// the framework's robot-agnostic property):
//
//   kNum*  : default value used when YAML omits the field. Per-device count
//            is set at runtime via DeviceState::num_channels (or analogous).
//            Code MUST NOT assume the actual count equals the kNum* default.
//   kMax*  : compile-time UPPER-BOUND capacity. Sized so that std::array
//            members of DeviceState/ControllerState etc. stay trivially
//            copyable (SeqLock-compatible) and avoid any heap allocation on
//            the RT path. New robots/devices stay within these bounds; if
//            you need to exceed one, raise the constant — never branch on it.
//
// kNumRobotJoints == 6 reflects the most common manipulator class (UR5/UR10/
// Franka-like 6/7-DOF). It is a default, not an assumption — algorithms in
// rtc_* packages must operate on runtime num_channels.
inline constexpr int kNumRobotJoints =
    6;  // default channel count; actual DOF from YAML (num_channels)
inline constexpr int kMaxRobotDOF = 12;         // upper-bound DOF (covers 7-DOF arms + redundancy)
inline constexpr int kMaxDeviceChannels = 64;   // upper-bound per DeviceState array
inline constexpr int kMaxSensorChannels = 128;  // upper-bound sensor channels per device
inline constexpr int kMaxInferenceValues = 64;  // upper-bound ONNX output values per device
inline constexpr int kTaskSpaceDim = 6;         // SE(3) DOF — geometry constant, not configurable

// Default fallback limits (used when device config is unavailable)
inline constexpr double kDefaultMaxJointVelocity = 2.0;  // rad/s
inline constexpr double kDefaultMaxJointTorque = 150.0;  // N·m

// ── RT control loop rate ────────────────────────────────────────────────────
//
// The RT control loop rate is a runtime YAML parameter (`control_rate`) — the
// framework is rate-agnostic and supports the entire range below. The values
// here are *defaults / bounds*, not assumptions. Code on the RT path that
// needs a per-tick dt MUST derive it from the configured rate (e.g. via
// `RTControllerInterface::GetDefaultDt()` or `ControllerState::dt` filled by
// the controller manager). Hard-coding `0.002` or `500.0` anywhere on the RT
// path violates the rate-agnostic contract.
//
// kDefaultControlRateHz is *only* used when:
//   1. YAML omits `control_rate` (declare_parameter default), or
//   2. A defensive fallback fires because `control_rate_` was somehow zero.
// The fallback path (2) is a misconfiguration signal — see callers for the
// log/error each invocation produces.
inline constexpr double kDefaultControlRateHz = 500.0;
inline constexpr double kMinControlRateHz = 100.0;                           // lower design bound
inline constexpr double kMaxControlRateHz = 5000.0;                          // upper design bound
inline constexpr double kDefaultControlDtSec = 1.0 / kDefaultControlRateHz;  // 2 ms

// Generic per-device array capacity for grouped/inferenced sensor blocks.
// rtc_* code uses this only as a compile-time upper bound on std::array
// dimensions (e.g. ToFSnapshotData::tip_poses, DeviceState::inference_enable);
// the runtime count comes from the device's YAML sensor_layout. Sized
// generously so future devices with more groups stay within the bound.
//
// (Pre-decoupling this was named `kMaxFingertips` — fingertip-as-concept
// removal is deferred to a follow-up sprint, hence the name is preserved
// here for now to keep the diff small.)
inline constexpr int kMaxFingertips = 8;

// ── C++20 Concepts
// ───────────────────────────────────────────────────────────── Constrains
// template parameters to floating-point types (double, float, etc.). Used for
// gain parameters and filter coefficients in downstream packages (e.g.
// bessel_filter.hpp, kalman_filter.hpp).
template <typename T>
concept FloatingPointType = std::floating_point<T>;

// Constrains types to be usable in lock-free primitives (SeqLock, SPSC
// buffers).
template <typename T>
concept TriviallyCopyableType = std::is_trivially_copyable_v<T>;

// ── Data structures (aggregate, zero-initialised by default)
// ──────────────────

// Unified device state — used for all device groups (robot arm, hand, gripper,
// …)
struct DeviceState {
  int num_channels{0};
  std::array<double, kMaxDeviceChannels> positions{};
  std::array<double, kMaxDeviceChannels> velocities{};
  std::array<double, kMaxDeviceChannels> efforts{};  // torques for robot arm
  // Motor-space data (separate from joint-space, e.g. hand motor encoder
  // values)
  int num_motor_channels{0};
  std::array<double, kMaxDeviceChannels> motor_positions{};
  std::array<double, kMaxDeviceChannels> motor_velocities{};
  std::array<double, kMaxDeviceChannels> motor_efforts{};     // motor currents
  std::array<int32_t, kMaxSensorChannels> sensor_data{};      // post-filter
  std::array<int32_t, kMaxSensorChannels> sensor_data_raw{};  // pre-filter
  int num_sensor_channels{0};
  // Inference (force/displacement per fingertip)
  std::array<float, kMaxInferenceValues> inference_data{};  // [contact,F(3),u(3)] × fingertips
  std::array<bool, kMaxFingertips> inference_enable{};      // per-fingertip flag
  int num_inference_fingertips{0};
  bool valid{false};
};

struct ControllerState {
  static constexpr int kMaxDevices = 8;
  std::array<DeviceState, kMaxDevices> devices{};
  int num_devices{0};
  // Per-tick period [s]. Filled by RtControllerNode each tick from the
  // configured `control_rate` YAML parameter (1 / control_rate). The
  // initialiser here is a placeholder; controllers must NOT assume the
  // default value reflects the runtime rate. See kDefaultControlDtSec.
  double dt{kDefaultControlDtSec};
  uint64_t iteration{0};

  // Session-relative wall time in seconds (current_tick - first-tick origin),
  // filled by CM RT loop before each Compute() dispatch. Controllers must
  // read this for any timestamp embedded in logs/telemetry — calling
  // chrono::*::now() inside Compute() bypasses the shared session origin
  // and breaks cross-controller log alignment. The origin is captured at
  // the very first RT tick (loop_count_ == 0) and is NEVER reset on
  // controller switch.
  double t_relative_s{0.0};
};

enum class CommandType { kPosition, kTorque };
enum class GoalType : uint8_t { kJoint, kTask };

[[nodiscard]] inline constexpr const char* GoalTypeToString(GoalType g) noexcept {
  switch (g) {
    case GoalType::kJoint:
      return "joint";
    case GoalType::kTask:
      return "task";
  }
  return "unknown";
}

// Grasp detection state — trivially copyable, RT-safe (SPSC buffer 호환)
struct GraspStateData {
  std::array<float, kMaxFingertips> force_magnitude{};
  std::array<float, kMaxFingertips> contact_flag{};
  std::array<bool, kMaxFingertips> inference_valid{};
  int num_fingertips{0};
  int num_active_contacts{0};
  float max_force{0.0f};
  bool grasp_detected{false};
  float force_threshold{1.0f};
  int min_fingertips_for_grasp{2};

  // Force-PI grasp controller state (grasp_controller_type == "force_pi" 전용)
  uint8_t grasp_phase{0};                                     // GraspPhase enum
  std::array<float, kMaxFingertips> finger_s{};               // grasp parameter [0,1]
  std::array<float, kMaxFingertips> finger_filtered_force{};  // filtered force [N]
  std::array<float, kMaxFingertips> finger_force_error{};     // force error [N]
  float grasp_target_force{0.0f};                             // active target force [N]
};

// WBC state — published by TSID-based whole-body controllers. RT-safe POD.
// Parallel role to GraspStateData but reflects WBC's TSID-based grasp
// algorithm (no Force-PI fields). Consumers (BT coordinator / GUI) pick
// between grasp_state and wbc_state based on the active controller name.
struct WbcStateData {
  uint8_t phase{0};                                     // WbcPhase enum
  std::array<float, kMaxFingertips> force_magnitude{};  // |F| per fingertip [N]
  std::array<float, kMaxFingertips> contact_flag{};     // contact probability [0,1]
  std::array<float, kMaxFingertips> displacement{};     // raw displacement [m]
  int num_fingertips{0};
  int num_active_contacts{0};
  float max_force{0.0f};           // max across fingertips [N]
  float grasp_target_force{0.0f};  // active target force [N]
  bool grasp_detected{false};
  int min_fingertips_for_grasp{2};
  // TSID solver diagnostics (informational — not safety-critical)
  float tsid_solve_us{0.0f};
  bool tsid_solver_ok{true};
  int qp_fail_count{0};
};

// ── SE3 pose (RT-safe, trivially copyable) ─────────────────────────────────
// Generic free-standing pose carrier used by PublishSnapshot to ferry
// FK results from the RT loop to the publish thread (e.g. for tf publish).
// Hamilton quaternion convention (w, x, y, z) — identity = (1, 0, 0, 0).
struct Pose {
  std::array<double, 3> position{};
  std::array<double, 4> quaternion{1.0, 0.0, 0.0, 0.0};  // w, x, y, z
};

// ToF snapshot data — trivially copyable, RT-safe (SPSC buffer 호환)
// 상한값(kMax*) 기반 고정 배열 + 런타임 num_fingers/sensors_per_finger
struct ToFSnapshotData {
  static constexpr int kMaxFingers = kMaxFingertips;                           // 8
  static constexpr int kMaxSensorsPerFinger = 3;                               // upper bound
  static constexpr int kMaxTotalSensors = kMaxFingers * kMaxSensorsPerFinger;  // 24

  // 거리 [m]
  std::array<double, kMaxTotalSensors> distances{};
  std::array<bool, kMaxTotalSensors> valid{};

  // Fingertip SE3 poses — world frame. Re-uses the free-standing rtc::Pose
  // type so other consumers (e.g. PublishSnapshot tf slots) can share.
  std::array<Pose, kMaxFingers> tip_poses{};

  int num_fingers{0};         // 실제 사용 핑거 수
  int sensors_per_finger{0};  // 실제 핑거당 센서 수
  bool populated{false};      // true when controller has valid ToF + FK data
};

// Unified device output — per-device commands, goals, and trajectory data
struct DeviceOutput {
  int num_channels{0};
  std::array<double, kMaxDeviceChannels> commands{};
  std::array<double, kMaxDeviceChannels> goal_positions{};
  std::array<double, kMaxDeviceChannels> target_positions{};   // controller-specific target
  std::array<double, kMaxDeviceChannels> target_velocities{};  // controller-specific target vel
  std::array<double, kMaxDeviceChannels>
      trajectory_positions{};  // pure trajectory reference position
  std::array<double, kMaxDeviceChannels>
      trajectory_velocities{};  // pure trajectory reference velocity
  GoalType goal_type{GoalType::kJoint};
};

struct ControllerOutput {
  static constexpr int kMaxDevices = 8;
  std::array<DeviceOutput, kMaxDevices> devices{};
  int num_devices{0};

  // Shared fields (not per-device)
  std::array<double, kTaskSpaceDim> actual_task_positions{};  // TCP FK result
  std::array<double, kTaskSpaceDim> task_goal_positions{};    // task-space goal target from GUI
  std::array<double, kTaskSpaceDim>
      trajectory_task_positions{};  // task-space trajectory reference pose
  std::array<double, kTaskSpaceDim> trajectory_task_velocities{};  // task-space trajectory velocity
  bool valid{true};
  CommandType command_type{CommandType::kPosition};
  GraspStateData grasp_state{};
  WbcStateData wbc_state{};
  ToFSnapshotData tof_snapshot{};

  // ── TF source poses for kRobotTransforms publish role ─────────────────
  // RT compute fills these in addition to actual_task_positions when the
  // controller broadcasts a TFMessage (independent of tof_snapshot, which
  // only populates when ToF is active). Mirrors the SE3 slots in
  // PublishSnapshot::GroupCommandSlot — CM's RT-loop copies output → snap.
  Pose arm_tip_pose{};
  bool arm_tip_pose_valid{false};
  Pose virtual_tcp_pose{};
  bool virtual_tcp_pose_valid{false};
  std::array<Pose, kMaxFingertips> fingertip_poses{};
  std::array<bool, kMaxFingertips> fingertip_pose_valid{};
};

// ── Per-device name + URDF configuration ─────────────────────────────────────

struct DeviceUrdfConfig {
  std::string package;    // ament package name providing the URDF
  std::string path;       // relative to package share dir (e.g. "robots/<name>/urdf/<name>.urdf")
  std::string root_link;  // kinematic chain root link name
  std::string tip_link;   // end-effector link name (for FK/IK frame)
};

struct DeviceJointLimits {
  std::vector<double> max_velocity;      // per-joint (rad/s)
  std::vector<double> max_acceleration;  // per-joint (rad/s²), optional
  std::vector<double> max_torque;        // per-joint (Nm), optional
  std::vector<double> position_lower;    // per-joint lower bound (rad)
  std::vector<double> position_upper;    // per-joint upper bound (rad)
};

// Per-device sensor packing layout — describes how `DeviceState::sensor_data`
// (a flat int32 array) is laid out for one logical "group" (e.g. one
// fingertip on a tactile hand, one strain-gauge cluster on a force sensor).
// rtc_* code uses these counts only for stride/offset arithmetic — it does
// NOT know what the values mean. The semantics (barometer vs ToF, etc.) are
// the device-driver's private concern and live in the driver package.
struct DeviceSensorLayout {
  int primary_count_per_group{0};     // first sensor block per group
  int secondary_count_per_group{0};   // second sensor block per group
  int values_per_group{0};            // = primary + secondary (per-group stride)
  int inference_values_per_group{0};  // ML inference output size per group
};

// Per-device backend binding — parsed from `devices.<group>.backend:` in
// sim.yaml / robot.yaml. SSoT for HW/sim adapter type + wire-format topics.
// Phase 4: replaces backend-type inference + topic harvesting from controller
// YAML lanes (state/joint_command/ros2_command/motor_state/sensor_state).
//
// `type` keys are registry tags (see DeviceBackendRegistry) — current values:
//   "mujoco_native"    — sim mode, JointState ↔ JointCommand
//   "ur_driver_native" — UR ros2_control driver (JointState ↔ Float64MultiArray)
//   "udp_hand_native"  — UDP hand driver (joint + motor + sensor lanes)
//
// Required: type, state_topic, command_topic.
// Optional: motor_topic (motor-space lane), sensor_topic (packed sensor lane),
// joint_command_names (output ordering for ur_driver_native).
struct DeviceBackendBinding {
  std::string type;
  std::string state_topic;
  std::string command_topic;
  std::string motor_topic;   // empty → backend has no motor lane
  std::string sensor_topic;  // empty → backend has no sensor lane
};

struct DeviceNameConfig {
  std::string device_name;
  std::vector<std::string> joint_state_names;
  std::vector<std::string> joint_command_names;  // empty → defaults to joint_state_names
  std::vector<std::string> motor_state_names;    // motor-space names (e.g. motor_1..10)
  std::vector<std::string> sensor_names;
  std::optional<DeviceUrdfConfig> urdf;             // nullopt if no URDF for this device
  std::optional<DeviceJointLimits> joint_limits;    // nullopt if no limits configured
  std::optional<DeviceSensorLayout> sensor_layout;  // nullopt if device has no sensor block
  std::optional<DeviceBackendBinding> backend;      // nullopt if no `backend:` block in YAML
  std::vector<double> safe_position;                // E-STOP target position (per-joint, rad)
};

// ── Device capability bitmask (selective data copy in RT loop) ───────────────
// Phase 4: derived from the DeviceBackend impl (HasMotorState / HasSensorState
// + presence of an inference layout) by RtControllerNode at backend create
// time, then propagated into ControllerSlotMapping::capabilities for RT-path
// gating. The RT loop checks capability bits instead of per-field count
// checks, enabling the compiler to eliminate entire copy blocks for devices
// that don't provide certain data types (e.g. robot arm has no sensor_data /
// inference).

enum class DeviceCapability : uint16_t {
  kNone = 0,
  kJointState = 1 << 0,  ///< positions / velocities / efforts
  kMotorState = 1 << 1,  ///< motor_positions / motor_velocities / motor_efforts
  kSensorData = 1 << 2,  ///< sensor_data / sensor_data_raw
  kInference = 1 << 3,   ///< inference_data / inference_enable
};

[[nodiscard]] inline constexpr uint16_t operator|(DeviceCapability lhs,
                                                  DeviceCapability rhs) noexcept {
  return static_cast<uint16_t>(static_cast<uint16_t>(lhs) | static_cast<uint16_t>(rhs));
}

[[nodiscard]] inline constexpr bool HasCapability(uint16_t caps, DeviceCapability flag) noexcept {
  return (caps & static_cast<uint16_t>(flag)) != 0;
}

// ── Topic configuration for per-controller subscribe/publish routing ─────────

// Phase 4 trailing cleanup: SubscribeRole enum dropped — after the device-wire
// roles (state / motor_state / sensor_state) moved to `devices.<group>.backend`,
// the only remaining role was `kTarget`. A singleton enum carries no
// discrimination, so `SubscribeTopicEntry` no longer tags a role. The YAML
// parser still validates the `role:` string ("target" / "goal") as readable
// documentation + drift detection.

enum class PublishRole {
  // Phase 4: joint_command / ros2_command roles deleted — device-wire command
  // publication is owned by DeviceBackend impls via devices.<group>.backend.
  // (Phase 4: kGuiPosition removed — consumers use /rtc_cm/<group>/joint_states
  // for joint state and <config_key>/transforms (tf2_msgs/TFMessage) for TCP pose.)
  // (Phase C: kDeviceStateLog / kDeviceSensorLog removed — controller
  // data CSVs flow through ControllerLogSet, not CM publish.)
  // (Controller-owned isolation sprint: kGraspState / kWbcState / kToFSnapshot
  // removed — each controller owns a per-output SeqLock<T> and publishes
  // directly without going through PublishSnapshot.)
  kRobotTarget,  // rtc_msgs/RobotTarget (joint/task 목표)
  // Digital Twin
  kDigitalTwinState,  // sensor_msgs/JointState (RELIABLE republish for digital
                      // twin)
  // Per-controller TF array — controller가 사용하는 frame들을 한 토픽에
  // tf2_msgs/TFMessage 로 묶어 발행. frame_id는 system YAML urdf.{sub,tree}_models
  // 의 root_link/tip_link 에서 자동 추출, child_frame_id는 "<link>_actual" suffix.
  kRobotTransforms,  // tf2_msgs/TFMessage (controller-owned)
};

// ── Topic ownership tier ────────────────────────────────────────────────────
// Who creates the ROS2 subscription/publisher for a given topic entry:
//   kManager    : RtControllerNode (CM) — RT-adjacent, HW ↔ control-loop
//                 traffic (state, commands, logs). Default when YAML omits
//                 the field.
//   kController : per-controller LifecycleNode — external GUI / BT / planner
//                 traffic that is non-RT and may differ per controller.
//                 Controller-owned topics inherit the node's namespace
//                 (`/<config_key>/...`) when the YAML path is relative.
enum class TopicOwnership : uint8_t {
  kManager = 0,
  kController = 1,
};

[[nodiscard]] inline constexpr const char* TopicOwnershipToString(TopicOwnership own) noexcept {
  switch (own) {
    case TopicOwnership::kManager:
      return "manager";
    case TopicOwnership::kController:
      return "controller";
  }
  return "unknown";
}

struct SubscribeTopicEntry {
  std::string topic_name;
  TopicOwnership ownership{TopicOwnership::kManager};
};

struct PublishTopicEntry {
  std::string topic_name;
  PublishRole role;
  int data_size{0};  // pre-allocate message size (0 = use default for role)
  TopicOwnership ownership{TopicOwnership::kManager};
};

// ── Device topic grouping ────────────────────────────────────────────────────

// Phase 4: `capability` field removed — DeviceCapability bitmask is now
// derived per device from DeviceBackendBinding by RtControllerNode and
// propagated into ControllerSlotMapping at backend create time.
struct DeviceTopicGroup {
  std::vector<SubscribeTopicEntry> subscribe;
  std::vector<PublishTopicEntry> publish;
};

// Dynamic topic configuration: groups keyed by device name (arbitrary strings
// chosen by YAML, e.g. arm/hand/gripper).
// Uses a vector of pairs to preserve YAML insertion order.  Device indices
// throughout the system (ControllerState, ControllerOutput, LogEntry, CSV)
// are derived from the iteration order of this container, so alphabetical
// reordering (as std::map would do) breaks the mapping.
struct TopicConfig {
  std::vector<std::pair<std::string, DeviceTopicGroup>> groups;

  // Insert-or-access by name (preserves insertion order for new entries).
  DeviceTopicGroup& operator[](const std::string& name) {
    for (auto& [n, g] : groups) {
      if (n == name)
        return g;
    }
    groups.emplace_back(name, DeviceTopicGroup{});
    return groups.back().second;
  }

  // True if the named group exists and has at least one topic entry.
  [[nodiscard]] bool HasGroup(const std::string& name) const noexcept {
    for (const auto& [n, g] : groups) {
      if (n == name)
        return !g.subscribe.empty() || !g.publish.empty();
    }
    return false;
  }

  // True if the named group has at least one subscribe entry.
  [[nodiscard]] bool HasSubscribeTopic(const std::string& group_name) const noexcept {
    for (const auto& [n, g] : groups) {
      if (n != group_name)
        continue;
      return !g.subscribe.empty();
    }
    return false;
  }

  // Returns the topic name for the first subscribe entry in the named group,
  // or an empty string if the group is missing or has no subscribe entries.
  //
  // WARNING: NOT RT-safe — returns std::string (potential heap allocation).
  // Call only during initialisation, not from the RT control loop.
  [[nodiscard]] std::string GetFirstSubscribeTopic(const std::string& group_name) const {
    for (const auto& [n, g] : groups) {
      if (n != group_name)
        continue;
      if (g.subscribe.empty())
        return {};
      return g.subscribe.front().topic_name;
    }
    return {};
  }
};

// ── Role → string conversion (for ROS2 parameter exposure) ──────────────────

[[nodiscard]] inline constexpr const char* PublishRoleToString(PublishRole role) noexcept {
  switch (role) {
    case PublishRole::kRobotTarget:
      return "robot_target";
    case PublishRole::kDigitalTwinState:
      return "digital_twin_state";
    case PublishRole::kRobotTransforms:
      return "robot_transforms";
  }
  return "unknown";
}

}  // namespace rtc

#endif  // RTC_BASE_TYPES_HPP_
