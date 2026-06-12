#ifndef INTEGRATED_BRINGUP_LOGGING_DEVICE_WBC_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_DEVICE_WBC_LOG_POD_HPP_

// WBC-specific per-device state log POD — superset of DeviceStateLogPod for
// the TSID-based whole-body controller. Adds fields that are controller-
// private (not in rtc_base ControllerOutput) and therefore cannot flow
// through the generic FillDeviceStateLogPod path:
//
//   - accelerations  : TSID solution q̈ (a_opt device slice), NOT finite-diff
//   - trajectory_task_*  : current SE3 quintic-ramp setpoint (arm role)
//   - fingertip_force    : per-fingertip |F| from WbcStateData (hand role)
//
// One POD serves both arm and hand devices; a `role` byte (0=arm, 1=hand)
// drives role-gated column blocks in the writer so each CSV only carries the
// columns meaningful for that device (arm → SE3 task block, hand → motor +
// fingertip-force block). This mirrors the DeviceStateLogPod pattern where
// empty motor_names suppress the motor columns.
//
// SPSC constraint: trivially copyable — fixed-size std::array, no
// vector/string. Names are passed by span at header-write time, not stored
// (strings break trivial copy and force per-tick allocation).
//
// Path A (POD-only): no rtc_msgs/.msg, no rtc_base change. The `msg_type`
// string used in the YAML `logs:` block is "integrated_bringup/DeviceWbcLog"
// — an integrated_bringup-private channel id, not a published message.

#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>

namespace integrated_bringup {

struct DeviceWbcLogPod {
  // ── Capacities (humanoid-class headroom; arm 6 + hand 16) ─────────────────
  static constexpr std::size_t kMaxJoints = 32;
  static constexpr std::size_t kMaxMotors = 16;
  static constexpr std::size_t kTaskDim = 6;  // x,y,z,roll,pitch,yaw
  static constexpr std::size_t kMaxFingertips = 8;

  // role: 0 = arm (SE3 task block), 1 = hand (motor + fingertip-force block)
  std::uint8_t role{0};

  // ── Timestamp (CM-provided, session-relative) ─────────────────────────────
  double t_relative_s{0.0};

  // ── Sizes (runtime, ≤ caps above) ─────────────────────────────────────────
  std::uint8_t num_joints{0};
  std::uint8_t num_motors{0};
  std::uint8_t num_fingertips{0};

  // ── Joint-space state (WBC output) ───────────────────────────────────────
  std::array<double, kMaxJoints> actual_positions{};
  std::array<double, kMaxJoints> actual_velocities{};
  std::array<double, kMaxJoints> efforts{};
  std::array<double, kMaxJoints> accelerations{};  // a_opt device slice (q̈)
  std::array<double, kMaxJoints> commands{};
  std::array<double, kMaxJoints> joint_goal{};
  std::array<double, kMaxJoints> trajectory_positions{};
  std::array<double, kMaxJoints> trajectory_velocities{};

  // ── Task-space (arm role only) ───────────────────────────────────────────
  std::array<double, kTaskDim> task_goal{};
  std::array<double, kTaskDim> actual_task_positions{};
  std::array<double, kTaskDim> trajectory_task_positions{};   // SE3 ramp setpoint
  std::array<double, kTaskDim> trajectory_task_velocities{};  // SE3 ramp vel

  // ── Motor state (hand role only; num_motors == 0 → skip) ─────────────────
  std::array<double, kMaxMotors> motor_positions{};
  std::array<double, kMaxMotors> motor_velocities{};
  std::array<double, kMaxMotors> motor_efforts{};

  // ── Fingertip force (hand role only; |F| per fingertip [N]) ──────────────
  std::array<double, kMaxFingertips> fingertip_force{};

  // ── Categorical (mirrored as ints; writer translates) ────────────────────
  std::uint8_t command_type{0};  // 0=position, 1=torque, 2=pd_feedforward
  std::uint8_t goal_type{0};     // 0=joint, 1=task
};

static_assert(std::is_trivially_copyable_v<DeviceWbcLogPod>,
              "DeviceWbcLogPod must be trivially copyable for SPSC ring");

inline std::string_view DeviceWbcLogCommandTypeStr(std::uint8_t v) noexcept {
  return v == 0 ? "position" : (v == 1 ? "torque" : (v == 2 ? "pd_feedforward" : "unknown"));
}

inline std::string_view DeviceWbcLogGoalTypeStr(std::uint8_t v) noexcept {
  return v == 0 ? "joint" : (v == 1 ? "task" : "unknown");
}

/// Emit the entire CSV header line for a DeviceWbcLogPod. `role` selects the
/// role-gated blocks (must equal the `role` field of every pushed pod for
/// this channel). For role==0 (arm) `motor_names` / `fingertip_names` are
/// ignored; for role==1 (hand) the SE3 task block is suppressed.
/// The logger appends '\n'.
inline void WriteDeviceWbcLogHeader(std::ostream& os, std::uint8_t role,
                                    std::span<const std::string> joint_names,
                                    std::span<const std::string> motor_names,
                                    std::span<const std::string> fingertip_names) {
  os << "t_relative_s";

  auto emit_joint_col = [&](std::string_view prefix) {
    for (const auto& n : joint_names) {
      os << ',' << prefix << '_' << n;
    }
  };
  emit_joint_col("actual_pos");
  emit_joint_col("actual_vel");
  emit_joint_col("effort");
  emit_joint_col("accel");
  emit_joint_col("command");
  emit_joint_col("joint_goal");
  emit_joint_col("traj_pos");
  emit_joint_col("traj_vel");

  if (role == 0) {  // arm: SE3 task block
    os << ",task_goal_x,task_goal_y,task_goal_z";
    os << ",task_goal_roll,task_goal_pitch,task_goal_yaw";
    os << ",task_pos_x,task_pos_y,task_pos_z";
    os << ",task_pos_roll,task_pos_pitch,task_pos_yaw";
    os << ",traj_task_pos_x,traj_task_pos_y,traj_task_pos_z";
    os << ",traj_task_pos_roll,traj_task_pos_pitch,traj_task_pos_yaw";
    os << ",traj_task_vel_x,traj_task_vel_y,traj_task_vel_z";
    os << ",traj_task_vel_roll,traj_task_vel_pitch,traj_task_vel_yaw";
  } else {  // hand: motor + fingertip-force block
    for (const auto& n : motor_names) {
      os << ",motor_pos_" << n;
    }
    for (const auto& n : motor_names) {
      os << ",motor_vel_" << n;
    }
    for (const auto& n : motor_names) {
      os << ",motor_eff_" << n;
    }
    // Fingertip-force columns are FIXED-WIDTH (kMaxFingertips). The active
    // fingertip count (`num_active_fingertips_`) is a per-tick runtime value
    // derived from backend inference groups — unknown when this header is
    // written (once, at registration) and not predictable from sensor_names
    // (LEAP has 0 tactile sensors yet reports contact-force fingertips). A
    // fixed block is the only width that guarantees header == row. Columns
    // beyond the runtime count carry zeros. Label by fingertip_names when
    // provided, else numeric index.
    for (std::size_t i = 0; i < DeviceWbcLogPod::kMaxFingertips; ++i) {
      os << ",fingertip_force_";
      if (i < fingertip_names.size()) {
        os << fingertip_names[i];
      } else {
        os << i;
      }
    }
  }

  os << ",command_type,goal_type";
}

/// Emit one row. Block selection reads `p.role` so it agrees with a header
/// written for the same role. Per-array loops are bounded by the runtime
/// num_* fields. The logger appends '\n' + flush.
inline void WriteDeviceWbcLogRow(std::ostream& os, const DeviceWbcLogPod& p) {
  os << p.t_relative_s;

  auto emit_joint_array = [&](const std::array<double, DeviceWbcLogPod::kMaxJoints>& a) {
    for (std::size_t i = 0; i < p.num_joints; ++i) {
      os << ',' << a[i];
    }
  };
  emit_joint_array(p.actual_positions);
  emit_joint_array(p.actual_velocities);
  emit_joint_array(p.efforts);
  emit_joint_array(p.accelerations);
  emit_joint_array(p.commands);
  emit_joint_array(p.joint_goal);
  emit_joint_array(p.trajectory_positions);
  emit_joint_array(p.trajectory_velocities);

  if (p.role == 0) {
    for (std::size_t i = 0; i < DeviceWbcLogPod::kTaskDim; ++i) {
      os << ',' << p.task_goal[i];
    }
    for (std::size_t i = 0; i < DeviceWbcLogPod::kTaskDim; ++i) {
      os << ',' << p.actual_task_positions[i];
    }
    for (std::size_t i = 0; i < DeviceWbcLogPod::kTaskDim; ++i) {
      os << ',' << p.trajectory_task_positions[i];
    }
    for (std::size_t i = 0; i < DeviceWbcLogPod::kTaskDim; ++i) {
      os << ',' << p.trajectory_task_velocities[i];
    }
  } else {
    auto emit_motor_array = [&](const std::array<double, DeviceWbcLogPod::kMaxMotors>& a) {
      for (std::size_t i = 0; i < p.num_motors; ++i) {
        os << ',' << a[i];
      }
    };
    emit_motor_array(p.motor_positions);
    emit_motor_array(p.motor_velocities);
    emit_motor_array(p.motor_efforts);
    // Fixed-width fingertip block (see WriteDeviceWbcLogHeader): emit all
    // kMaxFingertips slots; entries beyond the runtime count are zero.
    for (std::size_t i = 0; i < DeviceWbcLogPod::kMaxFingertips; ++i) {
      os << ',' << p.fingertip_force[i];
    }
  }

  os << ',' << DeviceWbcLogCommandTypeStr(p.command_type);
  os << ',' << DeviceWbcLogGoalTypeStr(p.goal_type);
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_DEVICE_WBC_LOG_POD_HPP_
