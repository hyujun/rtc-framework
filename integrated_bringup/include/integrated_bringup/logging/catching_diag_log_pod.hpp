#ifndef INTEGRATED_BRINGUP_LOGGING_CATCHING_DIAG_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_CATCHING_DIAG_LOG_POD_HPP_

// ── One tick of the catching controller (dynamic_catching L8 §5.2) ──────────
//
// ONE POD, TWO CONSUMERS. The RT tick fills this once and hands the same
// object to both the CSV ring (`catching_diag.csv`) and the SeqLock the
// publish thread reads to build `rtc_msgs/CatchingState`. Two structs would
// be two chances for the number on the operator's screen and the number in
// the report to disagree about the same tick — the reason the GUI's ρ imports
// `rtc_tools.analysis.hand_close` instead of recomputing it (L8 §11).
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
#include <limits>
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
  /// Negative = never received (see CatchingState.msg). NOT 0, which on this
  /// wire means "arrived this instant".
  double input_age_s{-1.0};
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
  /// What went out on the wire, hold latch included; NaN on a tick that
  /// commanded nothing. Mirrors the controller's own command selection.
  std::array<double, kMaxArmJoints> q_cmd{};
  std::array<double, kMaxArmJoints> q_meas{};
  bool abort_stopped{false};

  // ── Fault latch (#537 S9b, D-S9-D1/D2/D3) ────────────────────────────────
  /// What raised the fault latch that is up NOW (0 while it is down). The
  /// transition that follows is ABORT_ESCALATED whatever the cause, so this is
  /// the only place a reader can tell a QP failure streak from a motion that
  /// missed its deadline. CSV column; not on the state message.
  enum class FaultCause : std::uint8_t {
    kNone = 0,
    kQpFailures = 1,      ///< supervisor.n_qp trials in a row ended by a CLIK failure
    kStopDeadline = 2,    ///< a stop (ABORT_SAFE ramp / RETREAT stop) passed deadline.stop_s
    kReturnDeadline = 3,  ///< RETREAT's return passed deadline.return_s
  };
  FaultCause fault_cause{FaultCause::kNone};
  /// Moves once per latch; the publish thread WARNs on its edge. Not a column.
  std::uint32_t fault_latch_seq{0};
  /// Why a fault reset was REFUSED on this tick (0 on every other tick, and on
  /// a reset that cleared the latch). The latch stays up; the operator asks
  /// again once the arm has stopped (D-S9-D3). CSV column.
  enum class FaultResetRefusal : std::uint8_t {
    kNone = 0,
    kCommandMoving = 1,       ///< the carried arm command still has a velocity
    kArmMoving = 2,           ///< a measured |q̇| above supervisor.homing.qd_tol
    kVelocityUnreadable = 3,  ///< the arm's velocity lane is not vouched for (fail-closed)
  };
  FaultResetRefusal fault_reset_refused{FaultResetRefusal::kNone};
  /// Moves once per refusal; the publish thread WARNs on its edge with the
  /// reason and, for kArmMoving, the joint and |q̇|. Not columns.
  std::uint32_t fault_reset_refuse_seq{0};
  FaultResetRefusal fault_reset_refuse_reason{FaultResetRefusal::kNone};
  int fault_reset_refuse_joint{-1};
  double fault_reset_refuse_value{0.0};

  // ── Hand (L6, from S7) ───────────────────────────────────────────────────
  bool hand_phase_valid{false};
  std::uint8_t hand_phase{0};
  double hand_rho{0.0};
  bool hand_timeout{false};
  /// D-S8-8 (b)'s hand-joint witness (#537 S8-C): caging joints stalled part-
  /// way while pushing toward q_close, the largest signed τ/τ_max over them
  /// (NaN when not evaluated — not in Hold, a lane not vouched for, or the
  /// witness off), how long the current unbroken stall has lasted [s] as of
  /// THIS tick's hand stage (0 when not blocked — the verdict, taken on the
  /// next tick, compares its own clock: this value plus one period), and which witness
  /// the recorded `outcome` rests on (0 none, 1 fingertips, 2 hand, 3 both).
  std::uint8_t hand_stalled_n{0};
  double hand_effort_frac{std::numeric_limits<double>::quiet_NaN()};
  double hand_blocked_s{0.0};
  std::uint8_t outcome_source{0};
  /// S8-I (`planner.wait_pose_source`): the wait pose the trial homes to and
  /// the planner seeds from. `wait_pose_adopted` is 1 when it is the arm's
  /// switched-in pose (source `current`), 0 when it is the YAML's; the
  /// per-joint values below are the pose in force either way, so a reader
  /// never has to consult the read-only mirror (which is the YAML value).
  /// `wait_pose_adopt_seq` moves once per adoption — the non-RT log's edge.
  bool wait_pose_adopted{false};
  std::uint32_t wait_pose_adopt_seq{0};
  /// Why a switched-in pose was NOT adopted (source `current`). Not a CSV
  /// column: `wait_pose_adopted` 0 plus the publish thread's WARN carry it.
  enum class WaitPoseRefusal : std::uint8_t {
    kNone = 0,
    kNoBox = 1,       ///< no margined joint box to admit the pose against
    kOutsideBox = 2,  ///< a joint reading outside the margined box (or NaN)
    kMoving = 3,      ///< armed before the arm came to rest
    kEstop = 4,       ///< the deciding tick was under an E-STOP
    /// Armed while nobody vouched for the arm's velocity lane (#537 Q9): a
    /// reading that may be a hole is not a reading of rest.
    kVelocityUnreadable = 5,
  };
  /// The device's positions are readable but its VELOCITY lane has a hole
  /// this tick (#537 pre-S10 R3). While it lasts the controller does not
  /// enter ARMED, does not finish homing or the RETREAT return, does not
  /// adopt a switched-in wait pose (arm) and refuses a fault reset (arm). Not
  /// a CSV column: the publish thread warns on the edge.
  bool arm_velocity_unreadable{false};
  bool hand_velocity_unreadable{false};
  /// Whether this tick could JUDGE the lane at all, i.e. the device's
  /// positions were readable (#610). With the position gate closed the flag
  /// above is false whatever the lane holds, and the publish thread must keep
  /// the episode it is in rather than read that as a recovery.
  bool arm_velocity_judged{false};
  bool hand_velocity_judged{false};
  /// The activation this tick belongs to: the publish thread's edge memory
  /// starts over with each one (#610). Not a CSV column.
  std::uint32_t velocity_report_activation{0};
  std::uint32_t wait_pose_refuse_seq{0};
  WaitPoseRefusal wait_pose_refuse_reason{WaitPoseRefusal::kNone};
  int wait_pose_refuse_joint{-1};
  double wait_pose_refuse_value{0.0};
  std::array<double, kMaxArmJoints> wait_pose{};

  // ── Segment MPC follower (MPC E1-F04) ──────────────────────────────────────
  // Every field is a CSV column (E1-F05, #631). The state message carries
  // none of them: its field set is frozen (D-20). `segment_event` and
  // `segment_refusal` are written as their integer values — the tables are in
  // the integrated_bringup README and in rtc_tools' catching plotter.
  /// What happened to a segment this tick. One value per tick: the
  /// law's (a switch, or the reason there was nothing to follow) wins over
  /// the lane's (an admission), which ran earlier in the tick.
  enum class SegmentEvent : std::uint8_t {
    kNone = 0,
    kAdmitted = 1,       ///< a segment entered the pending slot
    kDeferred = 2,       ///< admissible, left in the box: the slot holds another grid point (MD-37)
    kWorkspace = 3,      ///< retired (MD-73): the RT judged the stop against a catch box that
                         ///< no longer exists (MD-94). Never written; the number stays for
                         ///< the logs and tools that carry it
    kSwitched = 4,       ///< the pending segment became the followed one
    kGateRefused = 5,    ///< pending dropped: the continuity gate refused it (MD-39)
    kPlanMismatch = 6,   ///< a segment does not end the followed plan (MD-35)
    kSampleFailed = 7,   ///< a segment could not be sampled at now_lead + h
    kNotDue = 8,         ///< DECEL entry with a pending segment whose node 0 is later
    kNoSegment = 9,      ///< nothing followed and nothing pending (→ ABORT_SAFE, MD-44)
    kReplaced = 10,      ///< a newer segment for the pending one's node 0 took the slot (MD-58)
    kPairAdmitted = 11,  ///< APPROACH: a replacement plan entered the slot with its first segment
    kPlanSwitched = 12,  ///< that segment became the followed one and its plan the followed plan
  };
  /// The lane judged the box this tick (mode mpc, COMMITTED / CLOSING / DECEL);
  /// `segment_refusal` is meaningful only then (rtc::catching::SegmentRefusal).
  bool segment_judged{false};
  std::uint8_t segment_refusal{0};
  SegmentEvent segment_event{SegmentEvent::kNone};
  /// The RT stepped the CLIK toward a segment sample this tick; seq and k0
  /// are the followed segment's, the targets its sample at now_lead + h.
  bool segment_following{false};
  std::uint32_t segment_seq{0};
  std::int32_t segment_k0{0};
  std::array<double, 3> segment_p_d{};
  std::array<double, 3> segment_v_ff{};
  bool segment_held{false};  ///< past the last node (the stop is held)
  /// The switch gate's account on a tick that judged one (kSwitched or
  /// kGateRefused): ρ, the largest |Δq|, |Δq̇| and the refusing joint (−1).
  double segment_rho{0.0};
  double segment_dq_max{0.0};
  double segment_dqd_max{0.0};
  std::int32_t segment_gate_joint{-1};

  // ── Fingertip sensors (D-24) ─────────────────────────────────────────────
  // `tip_age_s` is filled from S5.4 because it is a MEASUREMENT — the D-24
  // receipt instant minus now, needing no policy. `tip_fresh` and
  // `tip_contact` are not, because both need a threshold L7 §4's TIP_STALE
  // has not fixed yet, and a threshold invented here would be the one every
  // later stage inherits (the same reason A-S5-5 deferred the ν̄ metric).
  // S7.3's contact lane fills them now (`supervisor.contact.*`, L7 §4.4);
  // before it, only `tip_age_s` was filled.
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
  os << ",armed,estop_active,fault_latched,armable,law_enabled,real_arm_config,wait_pose_adopted";
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
  os << ",track_err_rad,abort_stopped,fault_cause,fault_reset_refused";
  os << ",hand_phase_valid,hand_phase,hand_rho,hand_timeout";
  os << ",hand_stalled_n,hand_effort_frac,hand_blocked_s,outcome_source";
  // One literal per statement: rtc_tools' test_cpp_header_matches_this_list
  // reads the first literal of each `os <<` in this function.
  os << ",segment_judged,segment_refusal,segment_event,segment_following";
  os << ",segment_seq,segment_k0,segment_held";
  os << ",segment_p_d_x,segment_p_d_y,segment_p_d_z";
  os << ",segment_v_ff_x,segment_v_ff_y,segment_v_ff_z";
  os << ",segment_rho,segment_dq_max,segment_dqd_max,segment_gate_joint";
  // Per-joint and per-tip blocks come LAST, so everything above is a fixed
  // column list a reader can rely on without knowing the robot.
  for (std::size_t i = 0; i < cols.num_arm_joints; ++i) {
    os << ",q_cmd_" << arm_joint_names[i];
  }
  for (std::size_t i = 0; i < cols.num_arm_joints; ++i) {
    os << ",q_meas_" << arm_joint_names[i];
  }
  for (std::size_t i = 0; i < cols.num_arm_joints; ++i) {
    os << ",q_wait_" << arm_joint_names[i];
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
     << ',' << (p.real_arm_config ? 1 : 0) << ',' << (p.wait_pose_adopted ? 1 : 0);
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
  os << ',' << p.track_err_rad << ',' << (p.abort_stopped ? 1 : 0) << ','
     << static_cast<int>(p.fault_cause) << ',' << static_cast<int>(p.fault_reset_refused);
  os << ',' << (p.hand_phase_valid ? 1 : 0) << ',' << static_cast<int>(p.hand_phase) << ','
     << p.hand_rho << ',' << (p.hand_timeout ? 1 : 0);
  os << ',' << static_cast<int>(p.hand_stalled_n) << ',' << p.hand_effort_frac << ','
     << p.hand_blocked_s << ',' << static_cast<int>(p.outcome_source);
  // segment_refusal is a std::uint8_t and segment_event an enum over one: both go
  // out through int, or the stream writes the raw byte.
  os << ',' << (p.segment_judged ? 1 : 0) << ',' << static_cast<int>(p.segment_refusal) << ','
     << static_cast<int>(p.segment_event) << ',' << (p.segment_following ? 1 : 0) << ','
     << p.segment_seq << ',' << p.segment_k0 << ',' << (p.segment_held ? 1 : 0);
  detail::WriteXyzRow(os, p.segment_p_d);
  detail::WriteXyzRow(os, p.segment_v_ff);
  os << ',' << p.segment_rho << ',' << p.segment_dq_max << ',' << p.segment_dqd_max << ','
     << p.segment_gate_joint;
  for (std::size_t i = 0; i < cols.num_arm_joints; ++i) {
    os << ',' << p.q_cmd[i];
  }
  for (std::size_t i = 0; i < cols.num_arm_joints; ++i) {
    os << ',' << p.q_meas[i];
  }
  for (std::size_t i = 0; i < cols.num_arm_joints; ++i) {
    os << ',' << p.wait_pose[i];
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
