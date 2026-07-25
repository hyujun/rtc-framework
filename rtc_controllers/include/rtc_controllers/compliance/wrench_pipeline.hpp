// ── External-wrench RT pipeline (spec §3.2.1, §3.3, §10.6) ──────────────────
// The per-tick sequence every compliance controller runs on its external wrench,
// bundled with the state that sequence needs:
//
//     WrenchInput::Read (generation staleness)  →  bias calibration gate
//       →  WrenchConditioner::Apply (bias → gravity → deadband → sat → filter)
//       →  Ad^{-T} into LOCAL_WORLD_ALIGNED at the task frame
//       →  × staleness fade  →  contact hysteresis
//
// The pieces were already separate, testable units (external_wrench.hpp,
// wrench_conditioning.hpp); what was NOT shared was the ORDER and the two
// bias latches, which is precisely where the subtle failures live — fading the
// raw sample re-injects −f_grav, and collapsing the two latches skips §3.2.1's
// calibration for a whole activation whenever the F/T driver starts publishing
// after the controller activates (compliance-conventions.md §3.1). Slice 3 needs
// exactly that sequence, so it is extracted here rather than copied (P5).
//
// TaskImpedanceController still carries its own copy: extracting a helper and
// migrating a shipped controller onto it are separate regression surfaces, and
// this repo already took that call once — task_dynamics.hpp landed without
// touching the OSC for the same reason (#236 D16). The migration is a follow-up,
// and the day it happens these two must produce byte-identical wrenches.
//
// Deliberately model-free and FSM-free: the caller owns the robot model (it
// supplies the two frames) and the ComplianceStateMachine (it acts on the
// `begin_bias_calibration` request). That keeps this unit testable with nothing
// but Eigen, and keeps the E-8 latch semantics in one place.
#pragma once

#include "rtc_controllers/compliance/external_wrench.hpp"
#include "rtc_controllers/compliance/wrench_conditioning.hpp"
#include <rtc_math/se3/wrench.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rtc::compliance {

/// Per-tick knobs (§10.6). Kept out of the class so a controller can hold them
/// in its own SeqLock'd Gains POD and pass a coherent snapshot down.
struct WrenchPipelineParams {
  double timeout{0.05};           ///< [s] older than this → fade + DEGRADED
  double fadeout_time{0.1};       ///< [s] linear ramp to ZERO (never hold the last value)
  double contact_threshold{5.0};  ///< [N] ‖f_LWA‖ above this → in contact; ≤0 = detection off
  double contact_release_ratio{0.6};  ///< release at ratio·threshold (hysteresis MUST)
};

/// What the caller needs to drive diagnostics and the state machine.
struct WrenchPipelineStatus {
  double age{0.0};    ///< s since the last NEW sample (§10.6, D9)
  double fade{1.0};   ///< 1 fresh → 0 fully faded out
  bool valid{false};  ///< a sample has arrived and a source is configured
  bool stale{false};  ///< age > timeout (→ DEGRADED, never SAFE_STOP)
  bool in_contact{false};
  /// A committed §3.2.1 calibration is in force (or none was needed). False ⇒
  /// the wrench is running on a zero bias or is suppressed while averaging.
  bool bias_calibrated{true};
  /// Feed to ComplianceStateMachine::Step(..., bias_done, ...). Released even
  /// while calibration is still owed — see the two-latch note above.
  bool bias_gate_released{true};
  /// The caller must call ComplianceStateMachine::BeginBiasCalibration() this
  /// tick. Routed out instead of done here so the E-8 SAFE_STOP latch (which
  /// refuses that transition) stays the state machine's business alone.
  bool begin_bias_calibration{false};
  /// Running count of samples the input's finiteness gate dropped. A rising
  /// count with a healthy `valid` means the producer is intermittently sending
  /// garbage — invisible otherwise, since a dropped sample simply does not
  /// arrive.
  std::uint32_t rejected_samples{0};
};

class WrenchPipeline {
 public:
  /// Off-RT. Validates and commits the conditioning config all-or-nothing.
  /// @throws std::runtime_error on a cutoff at/above Nyquist or a negative bound.
  void Configure(const WrenchConditioningConfig& cfg, double sample_rate_hz) {
    conditioner_.Configure(cfg, sample_rate_hz);
  }

  /// Non-RT, wait-free, SINGLE writer (the SeqLock invariant) — the
  /// SetDeviceTarget idiom. Raw sample in the sensor body frame; every stage of
  /// conditioning happens on the RT side because all of it needs the model.
  void Publish(std::span<const double, kWrenchDim> wrench) noexcept { input_.Set(wrench); }

  /// RT: (re)activation reset. Drops the accrued age, the SAMPLE ITSELF, the
  /// bias, the filter history and the contact latch — an activation must
  /// inherit none of them. Returns true when the caller should enter
  /// BIAS_CALIBRATING.
  ///
  /// `Invalidate()` rather than `ResetTiming()`: re-dating the sample present at
  /// reset revives a dead producer's last reading as fresh (§10.6 says an
  /// expired wrench goes to ZERO, never held). With a live producer the
  /// difference is the handful of ticks until its next publish, and the two-latch
  /// bias design below already handles "data arrives after the gate was
  /// released" as a first-class path.
  ///
  /// `bias_pending_` is armed only when there is work: bias_calibration_samples
  /// == 0 means the operator declined the average, so the zero bias IS the
  /// calibration and the first sample must not be spent re-entering the state.
  [[nodiscard]] bool ResetForActivation() noexcept {
    input_.Invalidate();
    conditioner_.Reset();
    contact_.Reset();
    bias_done_ = false;
    bias_pending_ = conditioner_.config().bias_samples > 0;
    return true;
  }

  /// RT: one tick of the whole sequence. Returns the conditioned wrench in
  /// LOCAL_WORLD_ALIGNED at the task frame, already faded. Zero (and a status
  /// saying so) until a producer has published.
  ///
  /// @param R_world_sensor  ᵂR_S — orientation of the sensor body frame
  /// @param p_sensor        sensor origin, world
  /// @param p_task          task (LWA) frame origin, world — the lever-arm base
  /// @param gravity_world   ᵂg from the MODEL (never a hard-coded 9.81·−Z)
  [[nodiscard]] Eigen::Matrix<double, 6, 1> Update(const Eigen::Matrix3d& R_world_sensor,
                                                   const Eigen::Vector3d& p_sensor,
                                                   const Eigen::Vector3d& p_task,
                                                   const Eigen::Vector3d& gravity_world, double dt,
                                                   const WrenchPipelineParams& params,
                                                   WrenchPipelineStatus& out) noexcept {
    Eigen::Matrix<double, 6, 1> f_lwa = Eigen::Matrix<double, 6, 1>::Zero();
    out = WrenchPipelineStatus{};

    const WrenchRead sample = input_.Read(dt, params.timeout, params.fadeout_time);
    out.rejected_samples = input_.rejected_samples();
    out.valid = sample.valid;
    out.age = sample.age;
    out.fade = sample.fade;
    // Only a sample that ever arrived can be stale — "no producer yet" is the
    // activation transient, not a timeout, and must not fault the first ticks.
    out.stale = sample.valid && sample.stale;
    out.bias_calibrated = !bias_pending_;

    if (!sample.valid) {
      // Release the FSM gate so the controller does not sit in BIAS_CALIBRATING
      // forever (which would mask every later wrench fault), but KEEP the debt:
      // the §3.2.1 average is still owed and runs the moment data appears.
      bias_done_ = true;
      out.bias_gate_released = true;
      return f_lwa;
    }

    // Static payload gravity, expressed at the sensor origin in the SENSOR frame.
    const Wrench6 grav =
        StaticGravityWrench(R_world_sensor, gravity_world, conditioner_.config().payload_mass,
                            conditioner_.config().payload_com);

    if (bias_pending_ && bias_done_) {
      // Data arrived after the gate was released: re-enter BIAS_CALIBRATING and
      // do the owed work now.
      bias_done_ = false;
      out.begin_bias_calibration = true;
    }

    if (!bias_done_) {
      // BIAS_CALIBRATING: feed the average, emit nothing. Returning zero is what
      // keeps an uncalibrated offset out of the control law.
      //
      // Only NEW samples are folded in: §3.2.1 asks for an N-SAMPLE average, and
      // a 500–5000 Hz RT tick against a 100–1000 Hz F/T source would otherwise
      // fold each reading in rate-ratio times over.
      if (sample.is_new && conditioner_.AccumulateBias(sample.value, grav)) {
        bias_done_ = true;
        bias_pending_ = false;
        out.bias_calibrated = true;
        conditioner_.SeedFromSample(sample.value, grav);  // §3.3: seed at the signal, not at 0
      } else if (sample.stale) {
        // The producer died part-way through. Release the gate so wrench_timeout
        // reaches the FSM as DEGRADED instead of being masked, but keep the
        // partial sum: restarting would let a flaky producer block the bias
        // forever.
        bias_done_ = true;
      }
      out.bias_gate_released = bias_done_;
      return f_lwa;
    }
    out.bias_gate_released = true;

    const Wrench6 w_sensor = conditioner_.Apply(sample.value, grav);

    // Sensor body frame → LOCAL_WORLD_ALIGNED at the task frame. The LWA frame
    // is (identity rotation, p_task), so ᴬT_S = (R_sensor, p_sensor − p_task) and
    // the wrench rides Ad^{-T} — NOT Ad^T (compliance-conventions.md §2; pinned
    // by rtc_math's power-duality test, never assumed).
    rtc::math::se3::Iso3 t_lwa_sensor = rtc::math::se3::Iso3::Identity();
    t_lwa_sensor.linear() = R_world_sensor;
    t_lwa_sensor.translation() = p_sensor - p_task;
    rtc::math::se3::Vec6 w_vec;
    for (int i = 0; i < 6; ++i)
      w_vec(i) = w_sensor[static_cast<std::size_t>(i)];
    f_lwa = rtc::math::se3::transformWrench(t_lwa_sensor, w_vec);

    // §10.6 MUST: an expired wrench fades linearly to ZERO — never held. Applied
    // AFTER conditioning so bias/gravity removal cannot re-inject a constant on
    // the way down (fading the raw sample would leave −f_grav behind).
    f_lwa *= sample.fade;

    if (params.contact_threshold > 0.0) {
      // Release ratio clamped strictly below 1 so the ⇄ transition always keeps
      // a hysteresis band; equal enter/exit thresholds chatter at the boundary.
      out.in_contact = contact_.Update(
          f_lwa.head<3>().norm(), params.contact_threshold,
          std::clamp(params.contact_release_ratio, 0.0, 0.99) * params.contact_threshold);
    } else {
      // A non-positive threshold degenerates to enter == exit == 0, i.e. the bare
      // comparator the ratio clamp exists to prevent, and latches on any nonzero
      // noise. "Detection off" is the honest reading.
      contact_.Reset();
      out.in_contact = false;
    }
    return f_lwa;
  }

  [[nodiscard]] const Wrench6& bias() const noexcept { return conditioner_.bias(); }

  [[nodiscard]] const WrenchConditioningConfig& config() const noexcept {
    return conditioner_.config();
  }

 private:
  WrenchInput input_;
  WrenchConditioner conditioner_;
  ContactHysteresis contact_;
  // `bias_done_` is the FSM GATE ("stop holding BIAS_CALIBRATING"),
  // `bias_pending_` is the WORK ("a calibration is still owed"). They differ
  // exactly where the gate must be released without the work being finished.
  bool bias_done_{true};
  bool bias_pending_{false};
};

}  // namespace rtc::compliance
