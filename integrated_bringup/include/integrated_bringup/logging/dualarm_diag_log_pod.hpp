#ifndef INTEGRATED_BRINGUP_LOGGING_DUALARM_DIAG_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_DUALARM_DIAG_LOG_POD_HPP_

// Per-tick record of the dual-arm controller: what the tick did (ran the CLIK
// solve, or held and why), the solve's own diagnostics, and per task frame the
// pose error, the reference, the commanded pose and the measured pose.
//
// One row per tick, EVERY tick — a held tick (E-STOP, latched fault, unreadable
// device) still writes its row with `clik_ran = 0` and the fields it did not
// compute at zero, so a gap in the file means a dropped row and nothing else.
//
// Three poses per task, all in the task's base frame:
//   ref  — the reference T^d(t) the solve was given this tick;
//   cmd  — the frame at the COMMAND state the solve was evaluated at, i.e. at
//          the command the previous tick left. `err_*` is ref against cmd,
//          which is the error the solve itself fed back;
//   meas — the frame at the MEASURED joint state of this tick.
// Servo lag is cmd against meas at the matching time shift; it is not in
// `err_*` (the solve is evaluated at the command, not at the measurement).
//
// SPSC constraint: trivially copyable. No rtc_msgs/.msg — this is
// controller-internal. YAML `msg_type` id is "integrated_bringup/DualArmDiagLog".

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace integrated_bringup {

/// One solve per tick per controller: a single fixed instance.
inline constexpr std::string_view kDualArmDiagLogMsgType = "integrated_bringup/DualArmDiagLog";
inline constexpr std::string_view kDualArmDiagLogInstance = "dualarm_diag";

struct DualArmDiagLogPod {
  static constexpr std::size_t kMaxTasks = 4;
  static constexpr std::size_t kMaxJoints = 32;

  /// Why the tick did not run the solve. kNone = it ran.
  enum class Hold : std::uint8_t {
    kNone = 0,
    kEstop,       ///< global E-STOP: measured positions are held
    kFault,       ///< controller-local fault latch: the last command is held
    kUnreadable,  ///< the body device did not report its joints this tick
    kNotSeeded,   ///< no readable tick since activation / E-STOP release yet
    kNoModel,     ///< the solve was never configured
  };

  /// What raised the controller-local fault latch.
  enum class FaultCause : std::uint8_t {
    kNone = 0,
    kQpFailStreak,  ///< `fault.max_qp_fail_ticks` consecutive failed solves
    kTrackError,    ///< max |q_meas − q_cmd| over `fault.track_err_max`
    /// A seed found a body joint outside the solve's position box by more than
    /// one tick at its speed limit: the solve is not started from there.
    kSeedOutsideBox,
  };

  /// Why the RT tick dropped a task goal the ingress had accepted.
  enum class GoalDrop : std::uint8_t {
    kStale = 0,  ///< sent before this activation
    /// The tick could not build a reference from it: a rotation from the
    /// current reference too close to π, a pose or duration that is not
    /// finite, or a frame slot the runtime does not carry.
    kUnusable,
    kHeld,  ///< arrived while the fault latch was up
    kCount,
  };

  struct Task {
    bool valid{false};                ///< the fields below were computed this tick
    bool traj_active{false};          ///< the reference is still moving toward its goal
    std::uint32_t goal_sequence{0};   ///< sequence of the goal in force (0 = hold pose)
    double err_lin{0.0};              ///< ‖position error‖ [m], ref against cmd
    double err_ang{0.0};              ///< ‖rotation error‖ [rad], ref against cmd
    std::array<double, 3> ref_pos{};  ///< [m]
    std::array<double, 4> ref_quat{1.0, 0.0, 0.0, 0.0};  ///< w, x, y, z
    std::array<double, 3> cmd_pos{};
    std::array<double, 4> cmd_quat{1.0, 0.0, 0.0, 0.0};
    bool meas_valid{false};  ///< the body device was readable this tick
    std::array<double, 3> meas_pos{};
    std::array<double, 4> meas_quat{1.0, 0.0, 0.0, 0.0};
    /// Lifetime counts. Ingress refusals by TaskGoalReject (index = enum value;
    /// slot 0, "accepted", is the accepted count) and RT-side drops by GoalDrop.
    std::array<std::uint32_t, 4> ingress_counts{};
    std::array<std::uint32_t, static_cast<std::size_t>(GoalDrop::kCount)> drop_counts{};
  };

  // ── Stamps (true of every tick) ───────────────────────────────────────────
  double t_relative_s{0.0};
  std::uint64_t tick{0};

  // ── What the tick did ─────────────────────────────────────────────────────
  Hold hold{Hold::kNone};
  bool clik_ran{false};  ///< the solve was called this tick
  bool estop{false};
  bool fault_latched{false};
  FaultCause fault_cause{FaultCause::kNone};
  bool body_readable{false};
  bool hand_readable{false};
  bool reseeded{false};  ///< this tick re-seeded the command from the measurement

  // ── The solve (zero on a tick that did not call it) ───────────────────────
  bool reached_solve{false};
  bool converged{false};
  bool rejected_input{false};
  bool non_finite{false};
  bool command_mismatch{false};
  bool accel_rows_violated{false};
  bool brake_box_empty{false};
  std::int32_t status{-1};
  std::int32_t iterations{0};
  double solve_time_us{0.0};
  std::int32_t accel_rows{0};
  std::int32_t accel_rows_binding{0};
  std::uint32_t fb_saturated{0};  ///< bit 2k = task k linear, 2k+1 = angular
  std::uint32_t rot_near_pi{0};   ///< bit k = task k
  std::uint64_t brake_active{0};  ///< bit i = model velocity index i
  std::uint64_t brake_static_infeasible{0};

  // ── Supervision ───────────────────────────────────────────────────────────
  std::int32_t qp_fail_streak{0};
  double track_err{0.0};  ///< max_i |q_meas,i − q_cmd,i| over the body joints [rad]
  /// Lifetime count of goals refused on the device-group topics (a task goal
  /// there, or a joint goal that does not name every joint of the group).
  std::uint32_t group_goal_rejects{0};

  // ── Per task / per joint ──────────────────────────────────────────────────
  std::uint8_t num_tasks{0};
  std::array<Task, kMaxTasks> tasks{};
  std::uint8_t num_joints{0};
  std::array<double, kMaxJoints> q_cmd{};  ///< body command, device order [rad]
};

static_assert(std::is_trivially_copyable_v<DualArmDiagLogPod>,
              "DualArmDiagLogPod must be trivially copyable for SPSC ring");

/// Column geometry, derived ONCE from the configured names and captured by both
/// writers: the header is written before any pod exists, so a row sized from a
/// pod's own runtime widths would stop lining up with it.
struct DualArmDiagLogColumns {
  std::size_t tasks{0};
  std::size_t joints{0};
};

[[nodiscard]] inline DualArmDiagLogColumns DualArmDiagLogColumnsFor(
    const std::vector<std::string>& task_names, const std::vector<std::string>& joint_names) {
  return {std::min(task_names.size(), DualArmDiagLogPod::kMaxTasks),
          std::min(joint_names.size(), DualArmDiagLogPod::kMaxJoints)};
}

/// Emit the CSV header. The logger appends '\n'.
inline void WriteDualArmDiagLogHeader(std::ostream& os, const std::vector<std::string>& task_names,
                                      const std::vector<std::string>& joint_names,
                                      const DualArmDiagLogColumns& cols) {
  os << "t_relative_s,tick,hold,clik_ran,estop,fault_latched,fault_cause,body_readable,"
        "hand_readable,reseeded";
  os << ",reached_solve,converged,rejected_input,non_finite,command_mismatch,accel_rows_violated,"
        "brake_box_empty,status,iterations,solve_time_us,accel_rows,accel_rows_binding,"
        "fb_saturated,rot_near_pi,brake_active,brake_static_infeasible";
  os << ",qp_fail_streak,track_err,group_goal_rejects";
  for (std::size_t k = 0; k < cols.tasks; ++k) {
    const std::string& n = task_names[k];
    os << ',' << n << "_valid," << n << "_traj_active," << n << "_goal_sequence," << n
       << "_err_lin," << n << "_err_ang";
    for (const char* pose : {"ref", "cmd"}) {
      os << ',' << n << '_' << pose << "_x," << n << '_' << pose << "_y," << n << '_' << pose
         << "_z," << n << '_' << pose << "_qw," << n << '_' << pose << "_qx," << n << '_' << pose
         << "_qy," << n << '_' << pose << "_qz";
    }
    os << ',' << n << "_meas_valid," << n << "_meas_x," << n << "_meas_y," << n << "_meas_z," << n
       << "_meas_qw," << n << "_meas_qx," << n << "_meas_qy," << n << "_meas_qz";
    os << ',' << n << "_goals_accepted," << n << "_reject_goal_type," << n << "_reject_non_finite,"
       << n << "_reject_unknown_frame," << n << "_drop_stale," << n << "_drop_unusable," << n
       << "_drop_held";
  }
  for (std::size_t i = 0; i < cols.joints; ++i) {
    os << ",q_cmd_" << joint_names[i];
  }
}

/// Emit one row. The logger appends '\n' + flush.
inline void WriteDualArmDiagLogRow(std::ostream& os, const DualArmDiagLogPod& p,
                                   const DualArmDiagLogColumns& cols) {
  const auto bit = [](bool b) { return b ? 1 : 0; };
  os << p.t_relative_s << ',' << p.tick << ',' << static_cast<int>(p.hold) << ',' << bit(p.clik_ran)
     << ',' << bit(p.estop) << ',' << bit(p.fault_latched) << ',' << static_cast<int>(p.fault_cause)
     << ',' << bit(p.body_readable) << ',' << bit(p.hand_readable) << ',' << bit(p.reseeded);
  os << ',' << bit(p.reached_solve) << ',' << bit(p.converged) << ',' << bit(p.rejected_input)
     << ',' << bit(p.non_finite) << ',' << bit(p.command_mismatch) << ','
     << bit(p.accel_rows_violated) << ',' << bit(p.brake_box_empty) << ',' << p.status << ','
     << p.iterations << ',' << p.solve_time_us << ',' << p.accel_rows << ',' << p.accel_rows_binding
     << ',' << p.fb_saturated << ',' << p.rot_near_pi << ',' << p.brake_active << ','
     << p.brake_static_infeasible;
  os << ',' << p.qp_fail_streak << ',' << p.track_err << ',' << p.group_goal_rejects;
  for (std::size_t k = 0; k < cols.tasks; ++k) {
    const auto& t = p.tasks[k];
    os << ',' << bit(t.valid) << ',' << bit(t.traj_active) << ',' << t.goal_sequence << ','
       << t.err_lin << ',' << t.err_ang;
    for (const auto* pose : {&t.ref_pos, &t.cmd_pos}) {
      const auto& quat = (pose == &t.ref_pos) ? t.ref_quat : t.cmd_quat;
      os << ',' << (*pose)[0] << ',' << (*pose)[1] << ',' << (*pose)[2] << ',' << quat[0] << ','
         << quat[1] << ',' << quat[2] << ',' << quat[3];
    }
    os << ',' << bit(t.meas_valid) << ',' << t.meas_pos[0] << ',' << t.meas_pos[1] << ','
       << t.meas_pos[2] << ',' << t.meas_quat[0] << ',' << t.meas_quat[1] << ',' << t.meas_quat[2]
       << ',' << t.meas_quat[3];
    os << ',' << t.ingress_counts[0] << ',' << t.ingress_counts[1] << ',' << t.ingress_counts[2]
       << ',' << t.ingress_counts[3] << ',' << t.drop_counts[0] << ',' << t.drop_counts[1] << ','
       << t.drop_counts[2];
  }
  for (std::size_t i = 0; i < cols.joints; ++i) {
    os << ',' << p.q_cmd[i];
  }
}

/// Columns a row carries for a given geometry — for tests that pin the header
/// and the row to the same width.
[[nodiscard]] constexpr std::size_t DualArmDiagLogColumnCount(
    const DualArmDiagLogColumns& cols) noexcept {
  constexpr std::size_t kFixed = 10 + 16 + 3;
  constexpr std::size_t kPerTask = 5 + 7 + 7 + 8 + 7;
  return kFixed + (cols.tasks * kPerTask) + cols.joints;
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_DUALARM_DIAG_LOG_POD_HPP_
