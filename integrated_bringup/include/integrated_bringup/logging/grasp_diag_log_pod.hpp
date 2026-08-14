#ifndef INTEGRATED_BRINGUP_LOGGING_GRASP_DIAG_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_GRASP_DIAG_LOG_POD_HPP_

// Per-tick Force-PI grasp diagnostics for the joint / task demo controllers
// (#428). One row per RT tick (`grasp_diag.csv`), single fixed instance — the
// `task_diag` / `pull_estimator` shape, not the per-device maps.
//
// Why this exists: every value here is already computed on every tick and then
// dropped. #426 had to decide whether the online stiffness estimator was
// salvageable on real hardware, and that decision needed the force noise σ of
// the lane the estimator actually reads plus the spread of its raw samples.
// This file supplied both, and the answer was no — adaptation is retired and
// pinned off (grasp_tuning_guide.md 6.9). The channel stays: it is how the
// evidence for that call is still readable, and `plot_rtc_log` identifies this
// log type by the `k_est_` column prefix. Neither quantity was obtainable from
// what already shipped:
//
//   - `<device>_sensor.csv` carries `fingertip_force_filt_`, the contact_stop
//     filter bank (deployed 50 Hz). The Force-PI law reads a SEPARATE bank,
//     `fingertip_force_mag_filt_grasp_` (deployed 25 Hz), with its own YAML
//     cutoff. Measuring σ from the sensor CSV answers a question about a lane
//     the estimator never sees — quietly, since both columns are called force.
//   - `ros2 topic echo --csv` at 500 Hz drops samples. σ survives random drops;
//     the estimator transient this file exists to show does not.
//
// `tick` is the CM RT-loop iteration, so rows align with `pull_estimator.csv`
// and `<device>_sensor.csv` from the same run — which is what lets the 25 Hz
// and 50 Hz banks be compared directly rather than assumed distinct.
//
// K_inst is NOT reconstructible from this file's s and f columns, and no reader
// should try: the estimator consumes s_{t-1} - s_{t-2} while a row written
// after Update() returns carries s_t and s_{t-1} (see FingerState::s_prev).
// That one-tick skew is what hid a dead estimator for six months (#425), so the
// raw pair is recorded at the point of use and shipped verbatim in
// `delta_s_*` / `delta_f_*` / `k_inst_raw_*`.
//
// SPSC constraint: trivially copyable. Path A: no rtc_msgs/.msg — these are
// controller-internal servo diagnostics, not device state. YAML `msg_type` id
// is "integrated_bringup/GraspDiagLog".

#include "rtc_controllers/grasp/grasp_types.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace integrated_bringup {

/// One grasp controller per demo controller, so this channel is a single fixed
/// instance — the TaskDiagLog / PullEstimatorLog shape.
inline constexpr std::string_view kGraspDiagLogMsgType = "integrated_bringup/GraspDiagLog";
inline constexpr std::string_view kGraspDiagLogInstance = "grasp_diag";

/// Per-finger column capacity. Matches the controller's own fixed capacity, so
/// a row can always carry every finger the FSM could be servoing; the header
/// names only the fingers actually configured, and the row writer emits only
/// that many columns.
inline constexpr std::size_t kGraspDiagMaxFingers =
    static_cast<std::size_t>(rtc::grasp::kMaxGraspFingers);

struct GraspDiagLogPod {
  // ── Timestamp / alignment ─────────────────────────────────────────────────
  double t_relative_s{0.0};
  /// CM RT-loop tick index (ControllerState::iteration). Exactly one row is
  /// pushed per tick, so a gap here is a DROPPED row (SPSC ring overflow), not
  /// a tick the controller declined to log — E-STOP ticks emit valid=0 and a
  /// disabled channel produces no file at all. Same contract as
  /// `pull_estimator.csv` (#234 P-20), and the reason both files can be joined
  /// on this column.
  std::uint64_t tick{0};

  // ── Validity ──────────────────────────────────────────────────────────────
  /// The Force-PI law actually ran this tick. False on E-STOP and on any tick
  /// where the hand is not driven by the PI law; every per-finger field is then
  /// zero rather than the previous tick's value. Zeroing rather than freezing
  /// matches the published GraspState (#424): these are slow-moving quantities,
  /// so a frozen sample is indistinguishable from a live one, whereas 0.0 is
  /// unreachable for k_est while the servo runs (seed 1.0, every update a
  /// convex combination of positive terms). The last converged value stays
  /// readable on the preceding valid=1 row.
  bool valid{false};

  // ── FSM ───────────────────────────────────────────────────────────────────
  std::uint8_t grasp_phase{0};  ///< rtc::grasp::GraspPhase wire value
  float target_force{0.0F};     ///< active force setpoint [N]

  // ── Estimator parameters in force this tick ───────────────────────────────
  //
  // Logged for the same reason task_diag logs sigma0/lambda_max: all three are
  // live-writable through set_params(), so a stored CSV cannot be decoded from
  // that run's YAML alone. K_est_max in particular decides whether a flat
  // k_est trace means "converged" or "pinned at the cap", which is exactly the
  // question the hardware sessions answered (#424 / #426). Post-#426 a flat
  // trace at the seed is the expected reading on every deployed run.
  float beta{0.0F};       ///< gain-schedule coefficient: gain_scale = 1/(1+beta*K)
  float alpha_ema{0.0F};  ///< EMA retention on K_contact_est
  float K_est_max{0.0F};  ///< clamp applied to K_inst before it enters the EMA

  // ── Per-finger block (index < num_fingers is meaningful) ──────────────────
  std::uint8_t num_fingers{0};

  std::array<float, kGraspDiagMaxFingers> s{};           ///< grasp parameter [0,1]
  std::array<float, kGraspDiagMaxFingers> f_desired{};   ///< force reference [N]
  /// Filtered force the PI law consumed — the 25 Hz Force-PI bank
  /// (`fingertip_force_mag_filt_grasp_`), NOT the contact_stop bank that
  /// `<device>_sensor.csv` carries. The σ #426 needs is the standard deviation
  /// of this column over a stationary hold.
  std::array<float, kGraspDiagMaxFingers> f_measured{};
  /// f_desired - f_measured. Derived, but written out because it is the reading
  /// this file exists for — same call as task_diag's `damping_active`.
  std::array<float, kGraspDiagMaxFingers> f_error{};
  std::array<float, kGraspDiagMaxFingers> k_est{};       ///< K_contact_est after this tick's EMA
  /// Instantaneous delta_f/delta_s BEFORE the K_est_max clamp; 0 when no sample
  /// existed. The clamped value is what enters the EMA, so a post-clamp figure
  /// cannot show how far the raw samples land — the quantity #426 disputes.
  std::array<float, kGraspDiagMaxFingers> k_inst_raw{};
  std::array<float, kGraspDiagMaxFingers> delta_s{};     ///< s increment the estimator divided by
  std::array<float, kGraspDiagMaxFingers> delta_f{};     ///< force step attributed to it [N]
  /// 1/(1 + beta*k_est) — the gain multiplier actually applied to Kp and Ki.
  std::array<float, kGraspDiagMaxFingers> gain_scale{};

  /// Sample cleared both estimator guards and moved the EMA. Distinguishes
  /// "no sample this tick" from "a sample that was rejected" — otherwise the
  /// same all-zeros row.
  std::array<std::uint8_t, kGraspDiagMaxFingers> est_updated{};
  std::array<std::uint8_t, kGraspDiagMaxFingers> integrator_frozen{};
  std::array<std::uint8_t, kGraspDiagMaxFingers> contact{};
};

static_assert(std::is_trivially_copyable_v<GraspDiagLogPod>,
              "GraspDiagLogPod must be trivially copyable for SPSC ring");

/// Emit the CSV header. `finger_names` is the configured grasp-finger order;
/// the names are stamped into the per-finger column names so a stored CSV
/// decodes without that run's YAML or ROS log (the `pull_estimator` mask-role
/// stamp rationale, #234 P-14). An empty list falls back to indices, and the
/// column count then still matches whatever the row writer emits. The logger
/// appends '\n'.
inline void WriteGraspDiagLogHeader(std::ostream& os,
                                    std::span<const std::string> finger_names = {}) {
  os << "t_relative_s,tick,valid,grasp_phase,target_force";
  os << ",beta,alpha_ema,K_est_max";
  // Per-finger blocks are grouped by quantity rather than by finger so that a
  // spreadsheet reader can select one column family across all fingers.
  static constexpr std::string_view kPerFinger[] = {
      "s",       "f_desired", "f_measured", "f_error",    "k_est",
      "k_inst_raw", "delta_s", "delta_f",   "gain_scale", "est_updated",
      "integrator_frozen", "contact"};
  // Clamped exactly as WriteGraspDiagLogRow clamps `num_columns`: without it a
  // finger list longer than the POD capacity makes the header wider than any
  // row the writer can emit.
  const auto n = std::min(finger_names.size(), kGraspDiagMaxFingers);
  for (const auto& quantity : kPerFinger) {
    for (std::size_t i = 0; i < n; ++i) {
      os << ',' << quantity << '_' << finger_names[i];
    }
  }
}

/// Emit one row. Emits exactly `finger_names.size()` entries per quantity, so
/// the caller must pass the same list it gave the header writer. The logger
/// appends '\n' + flush.
inline void WriteGraspDiagLogRow(std::ostream& os, const GraspDiagLogPod& p,
                                 std::size_t num_columns) {
  os << p.t_relative_s;
  os << ',' << p.tick;
  os << ',' << (p.valid ? 1 : 0);
  os << ',' << static_cast<unsigned>(p.grasp_phase);
  os << ',' << p.target_force;
  os << ',' << p.beta << ',' << p.alpha_ema << ',' << p.K_est_max;

  const auto n = std::min(num_columns, kGraspDiagMaxFingers);
  const std::array<const std::array<float, kGraspDiagMaxFingers>*, 9> floats = {
      &p.s,       &p.f_desired, &p.f_measured, &p.f_error,    &p.k_est,
      &p.k_inst_raw, &p.delta_s, &p.delta_f,   &p.gain_scale};
  for (const auto* col : floats) {
    for (std::size_t i = 0; i < n; ++i) {
      os << ',' << (*col)[i];
    }
  }
  const std::array<const std::array<std::uint8_t, kGraspDiagMaxFingers>*, 3> flags = {
      &p.est_updated, &p.integrator_frozen, &p.contact};
  for (const auto* col : flags) {
    for (std::size_t i = 0; i < n; ++i) {
      os << ',' << static_cast<unsigned>((*col)[i]);
    }
  }
}

/// Mirror the grasp controller's per-tick state into the log POD (RT tick path
/// — noexcept, heap-free). `states` is GraspController::finger_states(); the
/// caller supplies the FSM scalars because they are controller-level, not
/// per-finger. Fingers beyond kGraspDiagMaxFingers are dropped, matching the
/// controller's own fixed capacity.
inline void FillGraspDiagLogPod(std::span<const rtc::grasp::FingerState> states,
                                const rtc::grasp::GraspParams& params, rtc::grasp::GraspPhase phase,
                                double target_force, double t_relative_s, std::uint64_t tick,
                                GraspDiagLogPod& pod) noexcept {
  pod.t_relative_s = t_relative_s;
  pod.tick = tick;
  pod.valid = true;
  pod.grasp_phase = static_cast<std::uint8_t>(phase);
  pod.target_force = static_cast<float>(target_force);
  pod.beta = static_cast<float>(params.beta);
  pod.alpha_ema = static_cast<float>(params.alpha_ema);
  pod.K_est_max = static_cast<float>(params.K_est_max);

  const auto n = std::min(states.size(), kGraspDiagMaxFingers);
  pod.num_fingers = static_cast<std::uint8_t>(n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto& fs = states[i];
    pod.s[i] = static_cast<float>(fs.s);
    pod.f_desired[i] = static_cast<float>(fs.f_desired);
    pod.f_measured[i] = static_cast<float>(fs.f_measured);
    pod.f_error[i] = static_cast<float>(fs.f_desired - fs.f_measured);
    pod.k_est[i] = static_cast<float>(fs.K_contact_est);
    pod.k_inst_raw[i] = static_cast<float>(fs.K_inst_raw);
    pod.delta_s[i] = static_cast<float>(fs.delta_s_used);
    pod.delta_f[i] = static_cast<float>(fs.delta_f_used);
    // Recomputed rather than read back from the controller: the law applies it
    // to Kp/Ki inside ComputeAdaptivePI and keeps no member, and this is the
    // same expression against the same post-update K_contact_est.
    pod.gain_scale[i] = static_cast<float>(1.0 / (1.0 + params.beta * fs.K_contact_est));
    pod.est_updated[i] = fs.estimate_updated ? 1U : 0U;
    pod.integrator_frozen[i] = fs.integrator_frozen ? 1U : 0U;
    pod.contact[i] = fs.contact_detected ? 1U : 0U;
  }
}

/// Column names for the grasp fingers: the hand's fingertip sensor names,
/// truncated to the grasp finger count.
///
/// Grasp finger f is servoed from fingertip sensor slot f (the force feed loop
/// reads `fingertip_force_mag_filt_grasp_[f]`), so the sensor name IS the
/// finger's name — there is no separate name list in the YAML to drift from it.
/// A hand reporting fewer sensors than the configured grasp fingers yields the
/// shorter list, and the header/row pair then agree on that shorter count
/// rather than emitting columns for fingers with no force lane.
inline std::vector<std::string> GraspDiagFingerNames(std::span<const std::string> sensor_names,
                                                     int num_grasp_fingers) {
  const auto n = std::min(sensor_names.size(),
                          static_cast<std::size_t>(std::max(num_grasp_fingers, 0)));
  return {sensor_names.begin(), sensor_names.begin() + static_cast<std::ptrdiff_t>(n)};
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_GRASP_DIAG_LOG_POD_HPP_
