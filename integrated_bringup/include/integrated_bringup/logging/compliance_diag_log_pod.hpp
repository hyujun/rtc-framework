#ifndef INTEGRATED_BRINGUP_LOGGING_COMPLIANCE_DIAG_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_COMPLIANCE_DIAG_LOG_POD_HPP_

// Per-tick §7 task-admittance diagnostics for demo_compliance_controller
// (#469 S4). One row per RT tick (`compliance_diag.csv`), single fixed instance
// — the `task_diag` / `grasp_diag` / `pull_estimator` shape, not the per-device
// maps.
//
// WHY THIS EXISTS. S3 bound the §7 law to a wrench source and gave it no
// observation surface at all. When the arm moved, nothing in the session
// directory could say whether that was the trajectory or the pull, whether the
// §10.7 ramp was still fading the wrench in, whether the sample was stale, or
// whether the FSM had dropped to DEGRADED. `task_diag.csv` carries σ/λ² and
// `pull_estimator.csv` carries the source's own numbers, so the whole span
// BETWEEN the source and the law was empty — which is exactly the span S5 has
// to tune on hardware.
//
// THE ONE-TICK SKEW, AND WHY THIS FILE DOES NOT PAPER OVER IT. The wrench is
// published from ComputeSecondary and consumed by the NEXT tick's
// ComputeControl (D-A14, and compute.cpp says so at the publish site). So at
// the push tail — which runs after both — `wrench_invalid_reason_` already
// belongs to the sample the law has not seen yet, while `wrench_lwa_` is the
// one it just used. Reconstructing either from the other is the #425 latch-order
// mistake that hid a dead estimator for six months, so nothing here is
// reconstructed: every field is STAGED AT ITS POINT OF USE inside
// ComputeControl, and the row therefore describes one self-consistent tick of
// the law. `invalid_reason` and `quality_low` are the verdict that gated the
// wrench this row's `wrench_*` columns carry, not the verdict computed later in
// the same tick.
//
// DECODABLE WITHOUT THAT RUN'S YAML (#429). The K_p / K_d / Λ_d diagonals and
// the §7.5 displacement bounds ride every row for the same reason task_diag
// carries sigma0/lambda_max: a flat x̃ trace is either "converged" or "pinned
// against the box", and those two readings are indistinguishable without the
// bound. `disp_limited` states which one it was outright — the barrier's own
// report, not an inference from the columns.
//
// WHAT DELIBERATELY HAS NO COLUMN: `command_divergence`, `saturation_persist`
// and `posture_authority_lost`. This binding wires none of the three, so a
// column for them would be permanently 0 — and a permanently-0 fault column
// reads as "this never happened" rather than "this was never watched".
//
// WHY each one is unwired is NOT restated here — the fault-classification block
// in `controllers/compliance/compute.cpp` owns that judgement, and one of the
// three (`command_divergence`) is an unwatched gap rather than an unreachable
// mode, which is a distinction only that block is in a position to keep true.
// This header carried its own copy of the reasoning until the #469 review found
// the copy asserting the opposite of what the command lane actually does; both
// places were wrong together, because nobody edits a rationale twice. What this
// header owns is the COLUMN rule above.
//
// SPSC constraint: trivially copyable. Path A: no rtc_msgs/.msg — these are
// controller-internal law diagnostics, not device state. YAML `msg_type` id is
// "integrated_bringup/ComplianceDiagLog".

#include <array>
#include <cstdint>
#include <ostream>
#include <string_view>
#include <type_traits>

namespace integrated_bringup {

/// One §7 law per controller, so this channel is a single fixed instance.
inline constexpr std::string_view kComplianceDiagLogMsgType =
    "integrated_bringup/ComplianceDiagLog";
inline constexpr std::string_view kComplianceDiagLogInstance = "compliance_diag";

struct ComplianceDiagLogPod {
  // ── Timestamp / alignment ─────────────────────────────────────────────────
  double t_relative_s{0.0};
  /// CM RT-loop tick index (ControllerState::iteration). Exactly one row per
  /// tick, so a gap here is a DROPPED row (SPSC ring overflow) and nothing else
  /// — held ticks emit valid=0. Joins this file to `pull_estimator.csv` and
  /// `task_diag.csv` from the same run, which is how a suspicious deviation is
  /// traced back to the estimate that caused it.
  std::uint64_t tick{0};

  // ── Validity ──────────────────────────────────────────────────────────────
  /// The §7 law actually ran this tick. False on E-STOP, an unreadable arm, an
  /// invalid joint reorder or a missing TCP frame — every branch of the F5 gate
  /// that early-returns before the admittance step. On those ticks the wrench
  /// and deviation columns are ZERO, not the previous tick's values: the
  /// controller's own falling-edge reset zeroes them (§10.6 forbids holding a
  /// wrench nobody measured), and these are slow-moving quantities, so a frozen
  /// sample would read as a live one. Same call `grasp_diag` makes per finger,
  /// and the opposite of `task_diag`, whose σ is a solver reading that has no
  /// meaningful zero.
  bool valid{false};

  // ── Source identity ───────────────────────────────────────────────────────
  /// `ComplianceWrenchSource` wire value (0 = pull_estimator). Configure-time
  /// and exclusive, so it is constant for the file — carried anyway because the
  /// question "which measurement moved the arm" is the first one asked of a
  /// stored session, and S6 adds a second legal value.
  std::uint8_t wrench_source{0};

  // ── FSM / ramp ────────────────────────────────────────────────────────────
  std::uint8_t fsm_state{0};  ///< rtc::compliance::ComplianceState wire value
  //
  // No `engaged` column. The controller has such a flag and it is tempting to
  // ship, but it is a latch that is true on exactly the ticks `valid` is true —
  // the F5 gate stages a held row on every other one. A column that can never
  // disagree with its neighbour gets read as though it could.
  /// §10.7 activation ramp coefficient actually multiplied into the wrench this
  /// tick. 0 → the law was fed nothing however large the measurement was, which
  /// is otherwise indistinguishable from "no force was there".
  float alpha{0.0F};

  // ── The wrench the law consumed (LWA, at the control-frame origin) ────────
  /// Post-conditioning f_lwa — deadband, bias, filter, fade and lever-arm
  /// transport all applied. The source's pre-conditioning numbers live in
  /// `pull_estimator.csv`; the difference between the two files is what the
  /// conditioning did, so neither is redundant.
  std::array<float, 6> wrench{};

  // ── WrenchPipelineStatus, as of the consuming tick ────────────────────────
  float wrench_age{0.0F};    ///< [s] since the last NEW sample (§10.6)
  float wrench_fade{0.0F};   ///< 1 fresh → 0 fully faded
  bool wrench_valid{false};  ///< a sample has arrived and a source is configured
  bool wrench_stale{false};  ///< age > timeout ⇒ DEGRADED (never SAFE_STOP)
  bool in_contact{false};
  bool bias_calibrated{false};     ///< a committed §3.2.1 calibration is in force
  bool bias_gate_released{false};  ///< the FSM may leave BIAS_CALIBRATING
  bool bias_begin{false};          ///< this tick armed a (re)calibration — a one-tick pulse
  /// Running count of samples the pipeline's finiteness gate dropped. A rising
  /// count under a healthy `wrench_valid` is an intermittently garbage
  /// producer, which is invisible otherwise: a dropped sample simply never
  /// arrives.
  std::uint32_t rejected_samples{0};

  // ── Source verdict that gated THAT wrench (see the skew note above) ───────
  bool quality_low{false};
  /// `rtc::grasp::PullInvalidReason` wire value: 0=valid 1=not-initialised
  /// 2=degenerate-normal 3=required-contact-missing 4=insufficient-contacts.
  /// The pair (`quality_low=1`, `invalid_reason=0`) is its own signature —
  /// "the pull was fine, the numbers or the application point were not" — which
  /// no ordinary invalid tick can produce.
  std::uint8_t invalid_reason{0};

  // ── Compliant-frame state ─────────────────────────────────────────────────
  std::array<float, 6> x_tilde{};  ///< x̃ = [p̃ ; log3(R̃)] deviation from X_d [m, rad]
  std::array<float, 6> nu_c{};     ///< ν_c — the compliant frame's own twist [m/s, rad/s]
  /// Control-frame origin, world. The lever-arm base the wrench was transported
  /// to, and the column S5's 15 cm envelope is measured against.
  std::array<float, 3> task_origin{};

  /// §7.5 guards as the integrator reported them THIS step — not inferred from
  /// x̃ against the bound below, which cannot distinguish "just touching the
  /// box" from "held there".
  bool disp_limited{false};
  bool vel_limited{false};
  /// False ⇒ the step was REFUSED and the compliant frame is unchanged. Distinct
  /// from `valid`: the law ran, and declined.
  bool adm_finite{false};

  // ── Parameter snapshot (see the decodability note above) ──────────────────
  std::array<float, 6> kp{};  ///< K_p diagonal [N/m ×3, N·m/rad ×3] (0 = hand-guiding)
  std::array<float, 6> kd{};  ///< K_d diagonal
  std::array<float, 6> md{};  ///< Λ_d diagonal [kg ×3, kg·m² ×3]
  float max_disp_lin{0.0F};   ///< [m] §7.5 bound; ≤ 0 = guard disabled
  float max_disp_ang{0.0F};   ///< [rad]
};

static_assert(std::is_trivially_copyable_v<ComplianceDiagLogPod>,
              "ComplianceDiagLogPod must be trivially copyable for SPSC ring");

/// Emit the CSV header. Fixed width — no runtime column expansion, because
/// every block here is 6-DOF by construction rather than sized by a device.
/// The logger appends '\n'.
///
/// `x_tilde_*` is this file's fingerprint for `plot_rtc_log` (#429): no other
/// POD emits that prefix. None of the names below carries the generic `_raw_`
/// or `_filt_` token on purpose — those are what the sensor_log fallback
/// matches on, and staying clear of them is cheaper than depending on branch
/// order to win.
inline void WriteComplianceDiagLogHeader(std::ostream& os) {
  os << "t_relative_s,tick,valid";
  os << ",wrench_source,fsm_state,alpha";
  os << ",wrench_fx,wrench_fy,wrench_fz,wrench_tx,wrench_ty,wrench_tz";
  os << ",wrench_age,wrench_fade,wrench_valid,wrench_stale,in_contact";
  os << ",bias_calibrated,bias_gate_released,bias_begin,rejected_samples";
  os << ",quality_low,invalid_reason";
  os << ",x_tilde_x,x_tilde_y,x_tilde_z,x_tilde_rx,x_tilde_ry,x_tilde_rz";
  os << ",nu_c_x,nu_c_y,nu_c_z,nu_c_rx,nu_c_ry,nu_c_rz";
  os << ",task_origin_x,task_origin_y,task_origin_z";
  os << ",disp_limited,vel_limited,adm_finite";
  os << ",kp_x,kp_y,kp_z,kp_rx,kp_ry,kp_rz";
  os << ",kd_x,kd_y,kd_z,kd_rx,kd_ry,kd_rz";
  os << ",md_x,md_y,md_z,md_rx,md_ry,md_rz";
  os << ",max_disp_lin,max_disp_ang";
}

/// Emit one row. The logger appends '\n' + flush.
inline void WriteComplianceDiagLogRow(std::ostream& os, const ComplianceDiagLogPod& p) {
  const auto six = [&os](const std::array<float, 6>& v) {
    for (const float x : v) {
      os << ',' << x;
    }
  };
  os << p.t_relative_s;
  os << ',' << p.tick;
  os << ',' << (p.valid ? 1 : 0);
  os << ',' << static_cast<unsigned>(p.wrench_source);
  os << ',' << static_cast<unsigned>(p.fsm_state);
  os << ',' << p.alpha;
  six(p.wrench);
  os << ',' << p.wrench_age;
  os << ',' << p.wrench_fade;
  os << ',' << (p.wrench_valid ? 1 : 0);
  os << ',' << (p.wrench_stale ? 1 : 0);
  os << ',' << (p.in_contact ? 1 : 0);
  os << ',' << (p.bias_calibrated ? 1 : 0);
  os << ',' << (p.bias_gate_released ? 1 : 0);
  os << ',' << (p.bias_begin ? 1 : 0);
  os << ',' << p.rejected_samples;
  os << ',' << (p.quality_low ? 1 : 0);
  os << ',' << static_cast<unsigned>(p.invalid_reason);
  six(p.x_tilde);
  six(p.nu_c);
  for (const float x : p.task_origin) {
    os << ',' << x;
  }
  os << ',' << (p.disp_limited ? 1 : 0);
  os << ',' << (p.vel_limited ? 1 : 0);
  os << ',' << (p.adm_finite ? 1 : 0);
  six(p.kp);
  six(p.kd);
  six(p.md);
  os << ',' << p.max_disp_lin;
  os << ',' << p.max_disp_ang;
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_COMPLIANCE_DIAG_LOG_POD_HPP_
