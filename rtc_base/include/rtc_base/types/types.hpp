#ifndef RTC_BASE_TYPES_HPP_
#define RTC_BASE_TYPES_HPP_

// Shared compile-time constants and plain-data structures used by all packages
// in the RTC framework.

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
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

// Generic per-device array capacities used by rtc_* compile-time POD types.
// The runtime count comes from each device's YAML sensor_layout — these
// constants are only upper bounds for std::array dimensions on the RT path.
// Sized generously so future devices stay within the bound; if you need to
// exceed one, raise the constant — never branch on it.
//
// kMaxSensorGroups : DeviceState inference-enable / per-group sensor blocks.
// kMaxTaskLinks    : ControllerOutput task-link pose array (any FK frame the
//                    controller publishes alongside its joint command —
//                    fingertip, virtual TCP, elbow, mounting plate, etc.).
inline constexpr int kMaxSensorGroups = 8;
inline constexpr int kMaxTaskLinks = 8;

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

// A per-slot device mask is one uint64_t, so the slot capacity must fit in it.
// Raising kMaxDeviceChannels past 64 is allowed by the "never branch on it"
// rule above, but it silently truncates every hole_mask — so it has to fail
// here instead, and the fix is a wider mask type, not a wider bound alone.
static_assert(kMaxDeviceChannels <= std::numeric_limits<uint64_t>::digits,
              "DeviceState::hole_mask is a uint64_t — widen it with kMaxDeviceChannels");

// Bits [0, n) of a per-slot device mask, saturating at both ends.
//
// The one place the two boundary rules of DeviceState::hole_mask live, because
// both the producer (integrated_bringup's WriteJointStateToCache) and the
// consumer (rtc_controller_interface's IsDeviceReadable) need the same answer
// and neither package includes the other:
//
//   n <= 0                    → 0         no slot is required / none was
//       written. For the gate this is what makes an unresolved model_dim
//       degrade to a plain validity check instead of demanding bits.
//   n >= kMaxDeviceChannels   → all ones  that many slots exist and no more,
//       so asking for more is asking for all of them. Writing the shift
//       directly would be undefined behaviour once n reaches the mask's width,
//       and at today's capacity those two bounds COINCIDE — the saturating
//       branch is the DEFAULT path for a full-width model, not a corner.
[[nodiscard]] constexpr uint64_t SlotMaskBelow(int n) noexcept {
  if (n <= 0)
    return 0;
  if (n >= kMaxDeviceChannels)
    return ~static_cast<uint64_t>(0);
  return (static_cast<uint64_t>(1) << n) - 1;
}

// Unified device state — used for all device groups (robot arm, hand, gripper,
// …)
struct DeviceState {
  int num_channels{0};
  std::array<double, kMaxDeviceChannels> positions{};
  std::array<double, kMaxDeviceChannels> velocities{};
  std::array<double, kMaxDeviceChannels> efforts{};  // torques for robot arm
  // Per-slot freshness of the `positions` lane (issue #284).
  //
  // Bit i set  ⇒ slot i was NOT written by the most recent state message, so
  //              positions[i] still holds whatever the PERSISTENT cache had
  //              before (initially 0).
  // Bit i clear⇒ no such claim.
  //
  // POSITIONS ONLY — the other two lanes have their own masks below. The three
  // are separate fields rather than one because the lanes have DIFFERENT hole
  // structures: a JointState carries `position`, `velocity` and `effort` at
  // three independent lengths, and WriteJointStateToCache copies each under its
  // own, so a slot can be fresh in q and stale in q_dot from the same message.
  //
  // #284 scoped this field to positions (2026-08-05) on the argument that every
  // consumer of the gate indexed q for FK/Jacobian/control-law. That premise
  // did not survive: CombinedModelCache::ExtractFullState reads `velocities`
  // immediately after passing the positions gate, so a lane the gate never
  // judged was already reaching a shipped control law (#446). The scoping
  // decision was right for the consumers of the day and is superseded, not
  // reversed — the gate still asks the positions question, and the lane axis
  // gets its own predicate (rtc::IsLaneReadable) rather than being folded in,
  // so a consumer that only reads q is not closed by a hole in q_dot.
  //
  // `num_channels` cannot express this: it is the WIRE length, and with an
  // active reorder map the slots actually written are the MATCHED reference
  // indices, so a message can be wide enough to pass the width gate while
  // leaving holes behind it. That is the whole of issue #284, and narrowing
  // `num_channels` to mean "fresh" was rejected (#265 decision B) because the
  // same field is already the egress bound in ValidateControllerOutput.
  //
  // POLARITY IS DELIBERATE — a set bit is BAD NEWS, so the aggregate's
  // zero-init means "no holes claimed", which is exactly the answer every
  // producer that predates this field should give. The opposite convention
  // (bit = written) would turn every `DeviceState{}` in the repository into a
  // fully-stale device and fail the gate closed on fixtures and future
  // backends alike. The cost of this choice is that a producer which forgets
  // to fill the mask is silently indistinguishable from a hole-free one; the
  // propagation test in rtc_controller_manager is what covers that.
  //
  // Produced by WriteJointStateToCache (integrated_bringup), carried across
  // the DeviceStateCache→DeviceState copy by RtControllerNode's ReadDeviceState,
  // and consumed by rtc::IsDeviceReadable. kMaxDeviceChannels == 64 is what
  // makes one uint64_t exactly enough — raise both together or not at all.
  uint64_t hole_mask{0};
  // Per-slot freshness of the `velocities` and `efforts` lanes (issue #446).
  //
  // Same polarity, same producer, same propagation seam and same "a producer
  // that never fills it is indistinguishable from a hole-free one" cost as
  // `hole_mask` above — read that comment for all of it. What is NOT the same
  // is the width term: `num_channels` is the POSITION wire length and bounds
  // none of these two, which is exactly why the lanes need their own record.
  //
  // An empty lane is the COMMON case, not a fault. `sensor_msgs/JointState`
  // makes `velocity` and `effort` optional and shipped drivers routinely omit
  // one or both, so a permanently all-ones mask here means "this device does
  // not report that lane" and a consumer of it must refuse rather than read
  // the zero-initialised array as a measurement (#446: an absent effort lane
  // reads as tau_m == 0, which a momentum observer resolves into a fabricated
  // payload rather than an error).
  uint64_t velocity_hole_mask{0};
  uint64_t effort_hole_mask{0};
  // Motor-space data (separate from joint-space, e.g. hand motor encoder
  // values)
  int num_motor_channels{0};
  std::array<double, kMaxDeviceChannels> motor_positions{};
  std::array<double, kMaxDeviceChannels> motor_velocities{};
  std::array<double, kMaxDeviceChannels> motor_efforts{};     // motor currents
  std::array<int32_t, kMaxSensorChannels> sensor_data{};      // post-filter
  std::array<int32_t, kMaxSensorChannels> sensor_data_raw{};  // pre-filter
  int num_sensor_channels{0};
  // Per-sensor-group inference output (e.g. force / displacement per group)
  std::array<float, kMaxInferenceValues> inference_data{};  // device-specific layout × groups
  std::array<bool, kMaxSensorGroups> inference_enable{};    // per-group enable flag
  int num_inference_groups{0};
  bool valid{false};
};

// Which of the three joint-space lanes a freshness question is about (#446).
//
// Exists so the lane axis is one predicate with a parameter instead of three
// near-identical predicates: the copies would be where the next lane-shaped
// rule drifts, and `IsSlotFresh`'s history in device_readability.hpp is the
// repository's worked example of a hand-rolled copy staying behind after the
// original was strengthened.
enum class StateLane { kPosition, kVelocity, kEffort };

// The per-slot hole mask for `lane`. The one place the lane→field mapping
// lives, because the producer (WriteJointStateToCache) and the consumers
// (rtc::IsLaneReadable and the diagnostics beside it) must agree and neither
// package includes the other — the same reason SlotMaskBelow sits here.
//
// An unknown enumerator answers "all holes" rather than "no holes": this is a
// freshness claim, and the safe answer to a question this function does not
// understand is to withhold the lane, not to certify it.
[[nodiscard]] constexpr uint64_t LaneHoleMask(const DeviceState& dev, StateLane lane) noexcept {
  switch (lane) {
    case StateLane::kPosition:
      return dev.hole_mask;
    case StateLane::kVelocity:
      return dev.velocity_hole_mask;
    case StateLane::kEffort:
      return dev.effort_hole_mask;
  }
  return ~static_cast<uint64_t>(0);
}

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

// kPdFeedforward: PD position-servo backbone (values = position target) PLUS a
// per-joint feedforward torque overlay (DeviceOutput::feedforward). Used for the
// mixed-command case (e.g. WBC arm = kPosition, hand = kPdFeedforward). The wire
// format (rtc_msgs/JointCommand) carries the matching "pd_feedforward" string;
// mujoco_sim injects feedforward into qfrc_applied. Real position-only backends
// fall back to position tracking (they ignore the feedforward channel).
enum class CommandType { kPosition, kTorque, kPdFeedforward };
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

// Wire string for the JointCommand.command_type field. Single source for the
// CommandType→string mapping shared by every backend that encodes the mode on
// the wire (mujoco_native, udp_hand_native, …) — keeps the spelling identical.
[[nodiscard]] inline constexpr const char* CommandTypeToString(CommandType c) noexcept {
  switch (c) {
    case CommandType::kPosition:
      return "position";
    case CommandType::kTorque:
      return "torque";
    case CommandType::kPdFeedforward:
      return "pd_feedforward";
  }
  return "unknown";
}

// ── SE3 pose (RT-safe, trivially copyable) ─────────────────────────────────
// Generic free-standing pose carrier used by PublishSnapshot to ferry
// FK results from the RT loop to the publish thread (e.g. for tf publish).
// Hamilton quaternion convention (w, x, y, z) — identity = (1, 0, 0, 0).
struct Pose {
  std::array<double, 3> position{};
  std::array<double, 4> quaternion{1.0, 0.0, 0.0, 0.0};  // w, x, y, z
};

// Note: GraspStateData / WbcStateData / ToFSnapshotData used to live here.
// They were relocated to their natural domain packages in ARCH-1 Phase 4d:
//   - rtc::grasp::GraspStateData      → rtc_controllers/grasp/grasp_state.hpp
//   - integrated_bringup::WbcStateData → integrated_bringup/controllers/wbc/wbc_state.hpp
//   - integrated_bringup::ToFSnapshotData → integrated_bringup/controllers/tof_snapshot.hpp
// rtc_base keeps only robot-agnostic primitives (Pose, DeviceState, DeviceOutput).

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
  // Per-joint feedforward torque (Nm), consumed only when the resolved command
  // type is kPdFeedforward. Zero-initialised; ignored by kPosition/kTorque.
  std::array<double, kMaxDeviceChannels> feedforward{};
  GoalType goal_type{GoalType::kJoint};
  // Per-device command type override. nullopt → inherit ControllerOutput::
  // command_type (the global default), so single-mode controllers need not set
  // it. Set explicitly for mixed-command output (e.g. WBC arm=kPosition,
  // hand=kPdFeedforward). std::optional<CommandType> stays trivially copyable.
  std::optional<CommandType> command_type{};
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

  // Note: grasp_state / wbc_state / tof_snapshot fields used to live here as
  // a controller→publish staging area. They were removed in ARCH-1 Phase 4c
  // — each controller now owns a per-output SeqLock<T> and publishes through
  // its own LifecyclePublisher without going through ControllerOutput. Test
  // fixtures read internal state via Get*StateForTesting() accessors instead.

  // ── TF source poses for kRobotTransforms publish role ─────────────────
  // RT compute fills these in addition to actual_task_positions when the
  // controller broadcasts a TFMessage (independent of tof_snapshot, which
  // only populates when ToF is active). Mirrors the SE3 slots in
  // PublishSnapshot::GroupCommandSlot — CM's RT-loop copies output → snap.
  //
  // task_link_poses is a generic array of FK frames the controller publishes
  // alongside its joint command — fingertip, virtual TCP, elbow, mounting
  // plate, or any other link the controller's hand/arm tree_model exposes.
  // Index meaning is controller-specific (the controller fills slots
  // 0..N-1 and sets the matching _valid bits; consumers read in lockstep).
  Pose arm_tip_pose{};
  bool arm_tip_pose_valid{false};
  Pose virtual_tcp_pose{};
  bool virtual_tcp_pose_valid{false};
  std::array<Pose, kMaxTaskLinks> task_link_poses{};
  std::array<bool, kMaxTaskLinks> task_link_pose_valid{};
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
//
// `has_native_*` flags advertise *backend-provided* (vs controller-derived)
// signal availability. They are opaque bools to rtc_* code — only consumers
// (controllers) interpret them. Operator sets per device-group in yaml to
// match the active backend's runtime capability (e.g. udp_hand_native with
// `ft_inferencer.enabled=true` → has_native_contact=true, mujoco backend
// without contact emulation → false). Default false (assume derived path).
struct DeviceSensorLayout {
  int primary_count_per_group{0};       // first sensor block per group
  int secondary_count_per_group{0};     // second sensor block per group
  int values_per_group{0};              // = primary + secondary (per-group stride)
  int inference_values_per_group{0};    // ML inference output size per group
  bool has_native_contact{false};       // backend emits per-group contact signal
  bool has_native_displacement{false};  // backend emits per-group displacement signal
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

// Every value here must have a working publisher behind it. Issue #196 Phase 5
// removed kRobotTarget / kDigitalTwinState: the YAML parser mapped them, but no
// consumer ever created a publisher, so a controller declaring one came up with
// a silently dead topic. New controller-owned non-RT outputs use the
// SeqLock<T> + Setup*Publisher pattern instead of extending this enum
// (CLAUDE.md §6 E-11), so the set is expected to stay at one.
//
// Previously removed for the same or adjacent reasons: joint_command /
// ros2_command (Phase 4 — device-wire publication is owned by DeviceBackend
// impls via devices.<group>.backend), kGuiPosition (Phase 4 — consumers use
// /rtc_cm/<group>/joint_states and <config_key>/transforms), kDeviceStateLog /
// kDeviceSensorLog (Phase C — controller data CSVs flow through
// ControllerLogSet), kGraspState / kWbcState / kToFSnapshot (controller-owned
// isolation sprint — per-output SeqLock<T>).
enum class PublishRole {
  // Per-controller TF array — controller가 사용하는 frame들을 한 토픽에
  // tf2_msgs/TFMessage 로 묶어 발행. frame_id는 system YAML urdf.{sub,tree}_models
  // 의 root_link/tip_link 에서 자동 추출, child_frame_id는 "<link>_actual" suffix.
  kRobotTransforms,  // tf2_msgs/TFMessage (controller-owned)
};

// ── Topic entries ───────────────────────────────────────────────────────────
// Issue #138 (post-Phase 4): controller-YAML topic entries are controller-owned
// by contract — the per-controller LifecycleNode creates every subscribe/publish
// entry (external GUI / BT / planner traffic, relative paths inheriting the
// node's `/<config_key>/...` namespace). Device-wire state/command lanes live in
// `devices.<group>.backend` (DeviceBackend-owned); CM's own fixed topics are
// hard-coded, not YAML-declared. There is no manager-owned controller-YAML lane,
// so the former `TopicOwnership` tier no longer exists.

struct SubscribeTopicEntry {
  std::string topic_name;
};

// Issue #196 Phase 5: the `data_size` field was dropped. The parser read it
// from YAML and stored it, but no publisher ever pre-allocated from it, so it
// documented a capability the framework did not have.
struct PublishTopicEntry {
  std::string topic_name;
  PublishRole role;
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
    case PublishRole::kRobotTransforms:
      return "robot_transforms";
  }
  return "unknown";
}

}  // namespace rtc

#endif  // RTC_BASE_TYPES_HPP_
