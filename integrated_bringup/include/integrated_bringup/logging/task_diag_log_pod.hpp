#ifndef INTEGRATED_BRINGUP_LOGGING_TASK_DIAG_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_TASK_DIAG_LOG_POD_HPP_

// Per-controller §6.5 singularity diagnostics for the task (CLIK) controller.
// One DifferentialIk solve serves the whole arm, so this is a single per-tick
// row (one `task_diag.csv`), NOT per-device — the same shape as
// `wbc_diag_log_pod.hpp`, which this deliberately mirrors.
//
// Why this exists (#310): `compliance::DifferentialIk::Result` already computes
// σ_min and λ² on EVERY tick and the controller used to discard both. Driving
// the arm into a wrist singularity therefore damped correctly but produced no
// fault, no warning and no published diagnostic — the operator saw only degraded
// tracking. The values are free; only the lane was missing.
//
// Fields source (controller-private, all from one DifferentialIk::Result):
//   - sigma_min / lambda_sq / ik_ok   ← DifferentialIk::Result
//   - sigma0 / lambda_max             ← floored Gains actually used this tick
//   - control_6dof                    ← which Jacobian (J_full_ vs J_pos_) fed it
//
// σ₀ and λ_max are logged alongside the measurement on purpose: `sigma_min <
// sigma0` is the whole reading, and a stored CSV must stay decodable without
// that run's YAML or ROS log (the gains are live-writable through set_gains()
// and the parameter callback, so they are not a launch-time constant).
//
// NOT a fault channel. σ_min < σ₀ is the §6.5 damping law's normal operating
// region — it is exactly the condition that switches λ² on — so promoting it to
// a fault would fire on every intended activation. The real safety nets sit
// elsewhere: a non-finite Jacobian holds via `!ok`, and a NaN that reaches the
// command is rejected by ValidateControllerOutput until E-STOP.
//
// SPSC constraint: trivially copyable. Path A: no rtc_msgs/.msg — these are
// controller-internal solver health, not device state. YAML `msg_type` id is
// "integrated_bringup/TaskDiagLog".

#include <ostream>
#include <string_view>
#include <type_traits>

namespace integrated_bringup {

/// One DifferentialIk solve per tick per controller, so this channel is a
/// single fixed instance — the PullEstimatorLog shape, not the per-device maps.
inline constexpr std::string_view kTaskDiagLogMsgType = "integrated_bringup/TaskDiagLog";
inline constexpr std::string_view kTaskDiagLogInstance = "task_diag";

struct TaskDiagLogPod {
  // ── Timestamp (CM-provided, session-relative) ─────────────────────────────
  double t_relative_s{0.0};

  // ── §6.5 measurement ──────────────────────────────────────────────────────
  double sigma_min{0.0};  ///< σ_min(J_S) — kinematic distance to singularity
  double lambda_sq{0.0};  ///< DLS damping λ² applied this tick (0 = far from singular)

  // ── Gains actually in force this tick (post-floor) ────────────────────────
  double sigma0{0.0};      ///< §6.5 activation threshold σ₀ (FloorSigma0 applied)
  double lambda_max{0.0};  ///< λ_max cap (FloorMaxDamping applied)

  // ── Context ───────────────────────────────────────────────────────────────
  bool ik_ok{false};         ///< DifferentialIk::Result::ok — false = held on non-finite J
  bool control_6dof{false};  ///< true = J_full_ (6D) solve, false = J_pos_ (3D)

  /// The CLIK law actually ran this tick. False on E-STOP / unreadable arm /
  /// invalid reorder, where ComputeControl early-returns before the solve and
  /// every σ field below is therefore the PREVIOUS tick's. Without this column
  /// a held tick is indistinguishable from a live one that happened to repeat
  /// its reading — the row is still emitted so the CSV has no gaps to interpret.
  bool valid{false};
};

static_assert(std::is_trivially_copyable_v<TaskDiagLogPod>,
              "TaskDiagLogPod must be trivially copyable for SPSC ring");

/// Emit the CSV header. Fixed width — no runtime column expansion. The logger
/// appends '\n'.
inline void WriteTaskDiagLogHeader(std::ostream& os) {
  os << "t_relative_s,valid,sigma_min,lambda_sq,sigma0,lambda_max,ik_ok,control_6dof";
  // Derived convenience column: 1 exactly when §6.5 damping is engaged on a
  // tick that actually solved. Written out rather than left to the reader
  // because `sigma_min < sigma0` is the reading this file exists for.
  os << ",damping_active";
}

/// Emit one row. The logger appends '\n' + flush.
inline void WriteTaskDiagLogRow(std::ostream& os, const TaskDiagLogPod& p) {
  os << p.t_relative_s;
  os << ',' << (p.valid ? 1 : 0);
  os << ',' << p.sigma_min;
  os << ',' << p.lambda_sq;
  os << ',' << p.sigma0;
  os << ',' << p.lambda_max;
  os << ',' << (p.ik_ok ? 1 : 0);
  os << ',' << (p.control_6dof ? 1 : 0);
  os << ',' << (p.valid && p.lambda_sq > 0.0 ? 1 : 0);
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_TASK_DIAG_LOG_POD_HPP_
