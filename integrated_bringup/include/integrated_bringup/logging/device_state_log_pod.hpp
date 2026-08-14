#ifndef UR5E_BRINGUP_LOGGING_DEVICE_STATE_LOG_POD_HPP_
#define UR5E_BRINGUP_LOGGING_DEVICE_STATE_LOG_POD_HPP_

// UR5e-bringup POD mirror of rtc_msgs/DeviceStateLog for controller-owned
// CSV logging. Defines the column set + per-row writer used by the
// generic rtc::ThreadCsvLogger<DeviceStateLogPod>.
//
// Robot-specific caps (kMaxJoints, kMaxMotors) live HERE in integrated_bringup,
// not in rtc_base — different robots (KUKA, Franka, …) define their own
// POD with their own caps and the same rtc_msgs/*Log column contract
// (Q-MSG-2(d) lock).
//
// SPSC constraint: trivially copyable. No std::vector / std::string —
// fixed-size std::array storage with runtime num_joints / num_motors.
//
// Header writer is hand-written and joint_names / motor_names are passed
// by span at file open (NOT stored in the POD — strings break trivial
// copy and force per-tick allocation). Per-joint columns expand using
// those names captured once at header-write time.

#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>

namespace integrated_bringup {

struct DeviceStateLogPod {
  // ── Capacities (chosen for ur5e arm + 10-DoF hand) ────────────────────────
  static constexpr std::size_t kMaxJoints = 16;  // 6 arm + 10 hand
  static constexpr std::size_t kMaxMotors = 16;
  static constexpr std::size_t kTaskDim = 6;  // x,y,z,roll,pitch,yaw

  // ── Timestamp (CM-provided, session-relative) ─────────────────────────────
  double t_relative_s{0.0};

  // ── Sizes (runtime, ≤ caps above) ─────────────────────────────────────────
  std::uint8_t num_joints{0};
  std::uint8_t num_motors{0};

  // ── Joint-space state ────────────────────────────────────────────────────
  std::array<double, kMaxJoints> actual_positions{};
  std::array<double, kMaxJoints> actual_velocities{};
  std::array<double, kMaxJoints> efforts{};
  std::array<double, kMaxJoints> commands{};
  std::array<double, kMaxJoints> joint_goal{};
  std::array<double, kMaxJoints> trajectory_positions{};
  std::array<double, kMaxJoints> trajectory_velocities{};

  // ── Task-space ───────────────────────────────────────────────────────────
  std::array<double, kTaskDim> task_goal{};
  std::array<double, kTaskDim> actual_task_positions{};

  // ── Motor state (optional; num_motors == 0 → skip) ────────────────────────
  std::array<double, kMaxMotors> motor_positions{};
  std::array<double, kMaxMotors> motor_velocities{};
  std::array<double, kMaxMotors> motor_efforts{};

  // ── Categorical (mirrored as ints; CSV writer translates) ─────────────────
  // command_type: 0=position, 1=torque, 2=pd_feedforward
  std::uint8_t command_type{0};
  // goal_type:    0=joint, 1=task
  std::uint8_t goal_type{0};
};

static_assert(std::is_trivially_copyable_v<DeviceStateLogPod>,
              "DeviceStateLogPod must be trivially copyable for SPSC ring");

/// Column geometry for one registered DeviceStateLog channel, captured ONCE at
/// registration and handed to BOTH writers.
///
/// Both widths are CONFIGURED (joint_names / motor_names, clamped to the POD
/// capacities), never the per-tick runtime counts. The header is written once
/// at first Open, before any pod exists, so a row sized by `p.num_joints` /
/// `p.num_motors` diverges from it whenever the backend reports a different
/// channel count than the config named — and it can: `num_channels` is
/// whatever `ReadState` filled, which is 0 for a device whose backend never
/// came up (rt_controller_node_rt_loop.cpp). The sibling sensor channel wrote
/// 138,248 rows through exactly that gap before anyone noticed (#440).
///
/// Rows therefore always carry `joints` / `motors` entries per array block:
/// channels the device did not report carry 0, and the trailing `num_joints` /
/// `num_motors` columns record what it actually reported, so a zero column is
/// legible as absence rather than as a measured zero.
struct DeviceStateLogColumns {
  std::size_t joints{0};
  std::size_t motors{0};
};

/// Derive the geometry from the configured names. The single place either
/// width is computed, so the two writers cannot be handed different numbers.
[[nodiscard]] inline DeviceStateLogColumns DeviceStateLogColumnsFor(
    std::span<const std::string> joint_names, std::span<const std::string> motor_names) {
  return {std::min(joint_names.size(), DeviceStateLogPod::kMaxJoints),
          std::min(motor_names.size(), DeviceStateLogPod::kMaxMotors)};
}

/// Emit the entire CSV header line for a DeviceStateLogPod. Widths come from
/// `cols`; `joint_names` / `motor_names` supply labels only and fall back to
/// `j<index>` / `m<index>` past the end of the list, so the emitted width is
/// `cols` no matter what the arguments say about each other.
/// The logger appends '\n'.
inline void WriteDeviceStateLogHeader(std::ostream& os, std::span<const std::string> joint_names,
                                      std::span<const std::string> motor_names,
                                      const DeviceStateLogColumns& cols) {
  os << "t_relative_s";

  const auto n_j = std::min(cols.joints, DeviceStateLogPod::kMaxJoints);
  const auto n_m = std::min(cols.motors, DeviceStateLogPod::kMaxMotors);
  // Header write is once-per-file and off the RT path, so building the label
  // string here is free.
  auto joint_label = [&](std::size_t i) {
    return i < joint_names.size() ? joint_names[i] : ("j" + std::to_string(i));
  };
  auto motor_label = [&](std::size_t i) {
    return i < motor_names.size() ? motor_names[i] : ("m" + std::to_string(i));
  };

  auto emit_joint_col = [&](std::string_view prefix) {
    for (std::size_t i = 0; i < n_j; ++i) {
      os << ',' << prefix << '_' << joint_label(i);
    }
  };
  emit_joint_col("actual_pos");
  emit_joint_col("actual_vel");
  emit_joint_col("effort");
  emit_joint_col("command");
  emit_joint_col("joint_goal");
  emit_joint_col("traj_pos");
  emit_joint_col("traj_vel");

  os << ",task_goal_x,task_goal_y,task_goal_z";
  os << ",task_goal_roll,task_goal_pitch,task_goal_yaw";
  os << ",task_pos_x,task_pos_y,task_pos_z";
  os << ",task_pos_roll,task_pos_pitch,task_pos_yaw";

  auto emit_motor_col = [&](std::string_view prefix) {
    for (std::size_t i = 0; i < n_m; ++i) {
      os << ',' << prefix << motor_label(i);
    }
  };
  emit_motor_col("motor_pos_");
  emit_motor_col("motor_vel_");
  emit_motor_col("motor_eff_");

  os << ",command_type,goal_type";
  // Appended last: the columns that keep a padded (or truncated) row
  // self-describing — see DeviceStateLogColumns.
  os << ",num_joints,num_motors";
}

/// Translate enum-as-int back to string for the CSV row.
inline std::string_view DeviceStateLogCommandTypeStr(std::uint8_t v) noexcept {
  return v == 0 ? "position" : (v == 1 ? "torque" : (v == 2 ? "pd_feedforward" : "unknown"));
}

inline std::string_view DeviceStateLogGoalTypeStr(std::uint8_t v) noexcept {
  return v == 0 ? "joint" : (v == 1 ? "task" : "unknown");
}

/// Emit one row. Width is taken from the SAME `cols` the header was written
/// with, so the two agree by construction rather than by the caller's care.
/// Channels the device did not report this tick are emitted as 0; the trailing
/// `num_joints` / `num_motors` columns say how many were real. The logger
/// appends '\n' + flush.
inline void WriteDeviceStateLogRow(std::ostream& os, const DeviceStateLogPod& p,
                                   const DeviceStateLogColumns& cols) {
  os << p.t_relative_s;

  const auto n_j = std::min(cols.joints, DeviceStateLogPod::kMaxJoints);
  const auto n_m = std::min(cols.motors, DeviceStateLogPod::kMaxMotors);
  const auto reported_j = std::min(static_cast<std::size_t>(p.num_joints), n_j);
  const auto reported_m = std::min(static_cast<std::size_t>(p.num_motors), n_m);

  auto emit_joint_array = [&](const std::array<double, DeviceStateLogPod::kMaxJoints>& a) {
    for (std::size_t i = 0; i < n_j; ++i) {
      os << ',' << (i < reported_j ? a[i] : 0.0);
    }
  };
  emit_joint_array(p.actual_positions);
  emit_joint_array(p.actual_velocities);
  emit_joint_array(p.efforts);
  emit_joint_array(p.commands);
  emit_joint_array(p.joint_goal);
  emit_joint_array(p.trajectory_positions);
  emit_joint_array(p.trajectory_velocities);

  for (std::size_t i = 0; i < DeviceStateLogPod::kTaskDim; ++i) {
    os << ',' << p.task_goal[i];
  }
  for (std::size_t i = 0; i < DeviceStateLogPod::kTaskDim; ++i) {
    os << ',' << p.actual_task_positions[i];
  }

  auto emit_motor_array = [&](const std::array<double, DeviceStateLogPod::kMaxMotors>& a) {
    for (std::size_t i = 0; i < n_m; ++i) {
      os << ',' << (i < reported_m ? a[i] : 0.0);
    }
  };
  emit_motor_array(p.motor_positions);
  emit_motor_array(p.motor_velocities);
  emit_motor_array(p.motor_efforts);

  os << ',' << DeviceStateLogCommandTypeStr(p.command_type);
  os << ',' << DeviceStateLogGoalTypeStr(p.goal_type);
  os << ',' << static_cast<unsigned>(p.num_joints);
  os << ',' << static_cast<unsigned>(p.num_motors);
}

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_LOGGING_DEVICE_STATE_LOG_POD_HPP_
