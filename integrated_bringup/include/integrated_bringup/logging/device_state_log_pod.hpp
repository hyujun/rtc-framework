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
  // command_type: 0=position, 1=torque
  std::uint8_t command_type{0};
  // goal_type:    0=joint, 1=task
  std::uint8_t goal_type{0};
};

static_assert(std::is_trivially_copyable_v<DeviceStateLogPod>,
              "DeviceStateLogPod must be trivially copyable for SPSC ring");

/// Emit the entire CSV header line for a DeviceStateLogPod.
/// `joint_names` and `motor_names` must match the runtime num_joints /
/// num_motors used during Push (captured once at on_configure, not RT).
/// The logger appends '\n'.
inline void WriteDeviceStateLogHeader(std::ostream& os, std::span<const std::string> joint_names,
                                      std::span<const std::string> motor_names) {
  os << "t_relative_s";

  auto emit_joint_col = [&](std::string_view prefix) {
    for (const auto& n : joint_names) {
      os << ',' << prefix << '_' << n;
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

  for (const auto& n : motor_names) {
    os << ",motor_pos_" << n;
  }
  for (const auto& n : motor_names) {
    os << ",motor_vel_" << n;
  }
  for (const auto& n : motor_names) {
    os << ",motor_eff_" << n;
  }

  os << ",command_type,goal_type";
}

/// Translate enum-as-int back to string for the CSV row.
inline std::string_view DeviceStateLogCommandTypeStr(std::uint8_t v) noexcept {
  return v == 0 ? "position" : (v == 1 ? "torque" : "unknown");
}

inline std::string_view DeviceStateLogGoalTypeStr(std::uint8_t v) noexcept {
  return v == 0 ? "joint" : (v == 1 ? "task" : "unknown");
}

/// Emit one row. Column count must match the header writer above (assert
/// not possible at compile time because column count depends on runtime
/// joint/motor counts — but the header *writer* and the row writer agree
/// to read the *same* num_* fields). The logger appends '\n' + flush.
inline void WriteDeviceStateLogRow(std::ostream& os, const DeviceStateLogPod& p) {
  os << p.t_relative_s;

  auto emit_joint_array = [&](const std::array<double, DeviceStateLogPod::kMaxJoints>& a) {
    for (std::size_t i = 0; i < p.num_joints; ++i) {
      os << ',' << a[i];
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
    for (std::size_t i = 0; i < p.num_motors; ++i) {
      os << ',' << a[i];
    }
  };
  emit_motor_array(p.motor_positions);
  emit_motor_array(p.motor_velocities);
  emit_motor_array(p.motor_efforts);

  os << ',' << DeviceStateLogCommandTypeStr(p.command_type);
  os << ',' << DeviceStateLogGoalTypeStr(p.goal_type);
}

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_LOGGING_DEVICE_STATE_LOG_POD_HPP_
