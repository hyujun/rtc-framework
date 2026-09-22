#ifndef INTEGRATED_BRINGUP_LOGGING_CATCHING_DIAG_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_CATCHING_DIAG_LOG_POD_HPP_

// ── One tick of the catching controller (dynamic_catching L8 §5.2) ──────────
//
// ONE POD, TWO CONSUMERS. The RT tick fills this once and hands the same
// object to both the CSV ring (`catching_diag.csv`) and the SeqLock the
// publish thread reads to build `rtc_msgs/CatchingState`. Two structs would
// be two chances for the number on the operator's screen and the number in
// the report to disagree about the same tick — the reason the GUI's ρ imports
// `rtc_tools.analysis.hand_close` instead of recomputing it (plan §13 S4).
//
// The state message carries ONE block this POD does not: the ingress
// counters, which are updated by the subscription callback and would be a
// data race to read from the RT tick. They ride their own snapshot
// (`CatchingIngressSnapshot`), so the file and the topic differ exactly by
// the block that is not per-tick.
//
// EVERY TICK PUSHES A ROW (PROC-7). Including E-STOP ticks, stale-input
// ticks, no-plan ticks and abort ticks. A tick that did not compute a block
// leaves that block ZERO and its `*_valid` flag false rather than carrying the
// previous tick's values forward — so a gap in the file means a dropped row
// (#234 P-20), and a repeated value means the controller really recomputed it.
// The RT tick gets this by construction: it default-constructs a fresh POD
// per tick instead of mutating a member.
//
// SPSC constraint: trivially copyable. Fixed-size std::array storage with
// runtime widths; names are captured once at header-write time (the
// DeviceStateLogPod contract).
//
// YAML `msg_type` id is "integrated_bringup/CatchingDiagLog".

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

struct CatchingDiagLogPod {
  /// Arm joint capacity. Sized like DeviceStateLogPod's: this is a
  /// bringup-domain POD, so the cap lives here rather than in rtc_base.
  static constexpr std::size_t kMaxArmJoints = 16;
  /// Fingertip capacity — rtc::kMaxSensorGroups, restated so this header
  /// carries no dependency on the framework types.
  static constexpr std::size_t kMaxTips = 8;

  // ── Timestamp (CM-provided, session-relative) ────────────────────────────
  double t_relative_s{0.0};
  std::uint64_t tick{0};
  /// T_arm [s] — the lead the reference is evaluated at. Recorded per row
  /// because a run whose T_arm was reconfigured mid-session cannot be read
  /// against one number from the YAML.
  double t_arm_s{0.0};

  // ── Supervisor (L7) ──────────────────────────────────────────────────────
  std::uint8_t mode{0};     // rtc::catching::Mode
  std::uint8_t reason{0};   // rtc::catching::Reason
  std::uint8_t outcome{0};  // rtc::catching::Outcome (S8)
  bool armed{false};
  bool estop_active{false};
  bool fault_latched{false};
  bool armable{false};
  bool law_enabled{false};
  bool real_arm_config{false};

  // ── Vision input, as THIS TICK judged it (L1) ────────────────────────────
  bool input_valid{false};
  bool input_stale{true};
  bool input_expired{false};
  bool input_new{false};
  std::int32_t input_n{0};
  std::uint64_t input_generation{0};
  std::uint64_t input_snapshot_sequence{0};
  std::uint64_t input_activation_generation{0};
  double input_age_s{0.0};
  double input_horizon_s{0.0};

  // ── Plan (L3; the S5 oracle stands in for the planner) ───────────────────
  bool plan_valid{false};
  std::uint32_t plan_id{0};
  double plan_t_c_s{0.0};  // seconds from now to the catch instant
  double plan_age_s{0.0};
  std::array<double, 3> plan_p_c{};
  std::array<double, 3> plan_a_d{};
  std::array<double, 3> plan_v_c{};
  double plan_gamma_f{0.0};
  double plan_w5{0.0};
  double plan_w6{0.0};
  double plan_sigma_c{0.0};
  double plan_score{0.0};
  std::uint8_t plan_reason{0};  // rtc::catching::PlanReason

  // ── Reference (L4) ───────────────────────────────────────────────────────
  // xdd is the REALISED acceleration, u_des the pre-saturation demand. Both,
  // because a saturated interval is uninterpretable from either alone.
  bool ref_valid{false};
  bool ref_saturated{false};
  std::array<double, 3> ref_x{};
  std::array<double, 3> ref_xd{};
  std::array<double, 3> ref_xdd{};
  std::array<double, 3> ref_u_des{};
  std::array<double, 3> ref_e{};
  std::array<double, 3> ref_ed{};
  double ref_gamma{0.0};
  double ref_gamma_d{0.0};
  double ref_gamma_dd{0.0};

  // ── Joint command (L5, CLIK) ─────────────────────────────────────────────
  bool clik_ran{false};
  bool clik_converged{false};
  bool clik_bound_conflict{false};
  bool clik_command_mismatch{false};
  std::int32_t clik_status{-1};
  std::int32_t clik_iterations{0};
  double clik_solve_us{0.0};
  std::uint64_t clik_conflict_mask{0};
  std::int32_t qp_fail_streak{0};

  // ── Tracking ─────────────────────────────────────────────────────────────
  double track_err_rad{0.0};
  std::uint8_t num_arm_joints{0};
  std::array<double, kMaxArmJoints> q_cmd{};
  std::array<double, kMaxArmJoints> q_meas{};
  bool abort_stopped{false};

  // ── Hand (L6, from S7) ───────────────────────────────────────────────────
  bool hand_phase_valid{false};
  std::uint8_t hand_phase{0};
  double hand_rho{0.0};
  bool hand_timeout{false};

  // ── Fingertip sensors (D-24) ─────────────────────────────────────────────
  // `tip_age_s` is filled from S5.4 because it is a MEASUREMENT — the D-24
  // receipt instant minus now, needing no policy. `tip_fresh` and
  // `tip_contact` are not, because both need a threshold L7 §4's TIP_STALE
  // has not fixed yet, and a threshold invented here would be the one every
  // later stage inherits (the same reason A-S5-5 deferred the ν̄ metric).
  // They stay false until S7 supplies it; `tip_age_s` tells the truth in the
  // meantime.
  std::uint8_t num_tips{0};
  std::array<double, kMaxTips> tip_force{};
  std::array<bool, kMaxTips> tip_contact{};
  std::array<bool, kMaxTips> tip_fresh{};
  std::array<double, kMaxTips> tip_age_s{};
};

static_assert(std::is_trivially_copyable_v<CatchingDiagLogPod>,
              "CatchingDiagLogPod must be trivially copyable for the SPSC ring and the SeqLock");

/// Column geometry, derived ONCE and handed to both writers. The header is
/// written before any pod exists, so a row sized from the pod's own runtime
/// widths diverges from it the moment the two disagree — the gap that wrote
/// 138,248 mislabelled rows in the sibling sensor channel (#440).
struct CatchingDiagLogColumns {
  std::size_t num_arm_joints{0};
  std::size_t num_tips{0};
};

[[nodiscard]] inline CatchingDiagLogColumns CatchingDiagLogColumnsFor(
    const std::vector<std::string>& arm_joint_names, const std::vector<std::string>& tip_names) {
  CatchingDiagLogColumns cols;
  cols.num_arm_joints = std::min(arm_joint_names.size(), CatchingDiagLogPod::kMaxArmJoints);
  cols.num_tips = std::min(tip_names.size(), CatchingDiagLogPod::kMaxTips);
  return cols;
}

/// Mode name for the CSV. Values mirror rtc::catching::Mode; kept standalone
/// so this header has no dependency on the catching core.
[[nodiscard]] inline std::string_view CatchingModeStr(std::uint8_t v) noexcept {
  switch (v) {
    case 0:
      return "idle";
    case 1:
      return "armed";
    case 2:
      return "tracking";
    case 3:
      return "approach";
    case 4:
      return "committed";
    case 5:
      return "closing";
    case 6:
      return "decel";
    case 7:
      return "hold";
    case 8:
      return "retreat";
    case 9:
      return "abort_safe";
    case 10:
      return "fault";
    default:
      return "unknown";
  }
}

/// Reason name for the CSV. Values mirror rtc::catching::Reason.
[[nodiscard]] inline std::string_view CatchingReasonStr(std::uint8_t v) noexcept {
  switch (v) {
    case 0:
      return "none";
    case 1:
      return "ball_stale";
    case 2:
      return "ball_stale_committed";
    case 3:
      return "ball_stale_long";
    case 4:
      return "track_changed";
    case 5:
      return "horizon_extrap";
    case 6:
      return "pred_inconsistent";
    case 7:
      return "no_catchable_plan";
    case 8:
      return "plan_invalid";
    case 9:
      return "qp_failed";
    case 10:
      return "ref_saturated";
    case 11:
      return "joint_conflict";
    case 12:
      return "track_err";
    case 13:
      return "abort_escalated";
    case 14:
      return "estop";
    case 15:
      return "fault_reset";
    case 16:
      return "speed_scaling";
    case 17:
      return "clock_unhealthy";
    case 18:
      return "params_tbd";
    case 19:
      return "hand_timeout";
    case 20:
      return "tip_stale";
    default:
      return "unknown";
  }
}

namespace detail {

/// Values only — the header spells its own column names as literals so the
/// python-side oracle can read them back out of this file (#440's lesson,
/// applied to a writer whose fixed block is large enough to drift silently).
inline void WriteXyzRow(std::ostream& os, const std::array<double, 3>& v) {
  os << ',' << v[0] << ',' << v[1] << ',' << v[2];
}

}  // namespace detail

/// Emit the CSV header. `cols` sizes the per-joint and per-tip blocks and the
/// row writer must be handed the SAME value. The logger appends '\n'.
///
/// `mode` / `reason` are written BOTH as the raw enum value and as a name
/// (`mode_name` / `reason_name`), which is the convention the catchability-map
/// CSV of this same epic already uses. The raw value is what survives an enum
/// gaining a member; the name is what makes the file readable without this
/// build's header. A single string-valued `mode` column would instead collide
/// with the plotter's global "these columns are categorical" set, which is
/// keyed on bare names and shared by every log type.
inline void WriteCatchingDiagLogHeader(std::ostream& os,
                                       const std::vector<std::string>& arm_joint_names,
                                       const std::vector<std::string>& tip_names,
                                       const CatchingDiagLogColumns& cols) {
  os << "t_relative_s,tick,t_arm_s";
  os << ",mode,mode_name,reason,reason_name,outcome";
  os << ",armed,estop_active,fault_latched,armable,law_enabled,real_arm_config";
  os << ",input_valid,input_stale,input_expired,input_new,input_n";
  os << ",input_generation,input_snapshot_sequence,input_activation_generation";
  os << ",input_age_s,input_horizon_s";
  os << ",plan_valid,plan_id,plan_t_c_s,plan_age_s";
  os << ",plan_p_c_x,plan_p_c_y,plan_p_c_z";
  os << ",plan_a_d_x,plan_a_d_y,plan_a_d_z";
  os << ",plan_v_c_x,plan_v_c_y,plan_v_c_z";
  os << ",plan_gamma_f,plan_w5,plan_w6,plan_sigma_c,plan_score,plan_reason";
  os << ",ref_valid,ref_saturated";
  os << ",ref_x_x,ref_x_y,ref_x_z";
  os << ",ref_xd_x,ref_xd_y,ref_xd_z";
  os << ",ref_xdd_x,ref_xdd_y,ref_xdd_z";
  os << ",ref_u_des_x,ref_u_des_y,ref_u_des_z";
  os << ",ref_e_x,ref_e_y,ref_e_z";
  os << ",ref_ed_x,ref_ed_y,ref_ed_z";
  os << ",ref_gamma,ref_gamma_d,ref_gamma_dd";
  os << ",clik_ran,clik_converged,clik_bound_conflict,clik_command_mismatch";
  os << ",clik_status,clik_iterations,clik_solve_us,clik_conflict_mask,qp_fail_streak";
  os << ",track_err_rad,abort_stopped";
  os << ",hand_phase_valid,hand_phase,hand_rho,hand_timeout";
  // Per-joint and per-tip blocks come LAST, so everything above is a fixed
  // column list a reader can rely on without knowing the robot.
  for (std::size_t i = 0; i < cols.num_arm_joints; ++i) {
    os << ",q_cmd_" << arm_joint_names[i];
  }
  for (std::size_t i = 0; i < cols.num_arm_joints; ++i) {
    os << ",q_meas_" << arm_joint_names[i];
  }
  for (std::size_t i = 0; i < cols.num_tips; ++i) {
    os << ",tip_force_" << tip_names[i];
  }
  for (std::size_t i = 0; i < cols.num_tips; ++i) {
    os << ",tip_contact_" << tip_names[i];
  }
  for (std::size_t i = 0; i < cols.num_tips; ++i) {
    os << ",tip_fresh_" << tip_names[i];
  }
  for (std::size_t i = 0; i < cols.num_tips; ++i) {
    os << ",tip_age_s_" << tip_names[i];
  }
}

/// Emit one row, sized by the SAME `cols` the header was written with.
inline void WriteCatchingDiagLogRow(std::ostream& os, const CatchingDiagLogPod& p,
                                    const CatchingDiagLogColumns& cols) {
  os << p.t_relative_s << ',' << p.tick << ',' << p.t_arm_s;
  os << ',' << static_cast<int>(p.mode) << ',' << CatchingModeStr(p.mode) << ','
     << static_cast<int>(p.reason) << ',' << CatchingReasonStr(p.reason) << ','
     << static_cast<int>(p.outcome);
  os << ',' << (p.armed ? 1 : 0) << ',' << (p.estop_active ? 1 : 0) << ','
     << (p.fault_latched ? 1 : 0) << ',' << (p.armable ? 1 : 0) << ',' << (p.law_enabled ? 1 : 0)
     << ',' << (p.real_arm_config ? 1 : 0);
  os << ',' << (p.input_valid ? 1 : 0) << ',' << (p.input_stale ? 1 : 0) << ','
     << (p.input_expired ? 1 : 0) << ',' << (p.input_new ? 1 : 0) << ',' << p.input_n;
  os << ',' << p.input_generation << ',' << p.input_snapshot_sequence << ','
     << p.input_activation_generation;
  os << ',' << p.input_age_s << ',' << p.input_horizon_s;
  os << ',' << (p.plan_valid ? 1 : 0) << ',' << p.plan_id << ',' << p.plan_t_c_s << ','
     << p.plan_age_s;
  detail::WriteXyzRow(os, p.plan_p_c);
  detail::WriteXyzRow(os, p.plan_a_d);
  detail::WriteXyzRow(os, p.plan_v_c);
  os << ',' << p.plan_gamma_f << ',' << p.plan_w5 << ',' << p.plan_w6 << ',' << p.plan_sigma_c
     << ',' << p.plan_score << ',' << static_cast<int>(p.plan_reason);
  os << ',' << (p.ref_valid ? 1 : 0) << ',' << (p.ref_saturated ? 1 : 0);
  detail::WriteXyzRow(os, p.ref_x);
  detail::WriteXyzRow(os, p.ref_xd);
  detail::WriteXyzRow(os, p.ref_xdd);
  detail::WriteXyzRow(os, p.ref_u_des);
  detail::WriteXyzRow(os, p.ref_e);
  detail::WriteXyzRow(os, p.ref_ed);
  os << ',' << p.ref_gamma << ',' << p.ref_gamma_d << ',' << p.ref_gamma_dd;
  os << ',' << (p.clik_ran ? 1 : 0) << ',' << (p.clik_converged ? 1 : 0) << ','
     << (p.clik_bound_conflict ? 1 : 0) << ',' << (p.clik_command_mismatch ? 1 : 0);
  os << ',' << p.clik_status << ',' << p.clik_iterations << ',' << p.clik_solve_us << ','
     << p.clik_conflict_mask << ',' << p.qp_fail_streak;
  os << ',' << p.track_err_rad << ',' << (p.abort_stopped ? 1 : 0);
  os << ',' << (p.hand_phase_valid ? 1 : 0) << ',' << static_cast<int>(p.hand_phase) << ','
     << p.hand_rho << ',' << (p.hand_timeout ? 1 : 0);
  for (std::size_t i = 0; i < cols.num_arm_joints; ++i) {
    os << ',' << p.q_cmd[i];
  }
  for (std::size_t i = 0; i < cols.num_arm_joints; ++i) {
    os << ',' << p.q_meas[i];
  }
  for (std::size_t i = 0; i < cols.num_tips; ++i) {
    os << ',' << p.tip_force[i];
  }
  for (std::size_t i = 0; i < cols.num_tips; ++i) {
    os << ',' << (p.tip_contact[i] ? 1 : 0);
  }
  for (std::size_t i = 0; i < cols.num_tips; ++i) {
    os << ',' << (p.tip_fresh[i] ? 1 : 0);
  }
  for (std::size_t i = 0; i < cols.num_tips; ++i) {
    os << ',' << p.tip_age_s[i];
  }
}

/// The single fixed instance + YAML `msg_type` id for this channel.
inline constexpr const char* kCatchingDiagLogMsgType = "integrated_bringup/CatchingDiagLog";
inline constexpr const char* kCatchingDiagLogInstance = "catching_diag";

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_CATCHING_DIAG_LOG_POD_HPP_
