// ── Compliance controller state machine (§10.6) ─────────────────────────────
// Full §10.6 lattice as of slice 2 (the external wrench source landed, so the
// two wrench-driven states slice 1 left out are now live):
//
//   BIAS_CALIBRATING ──bias done──▶ HOLDING ──ramp done──▶ RUNNING ⇄ RUNNING_CONTACT
//   (wrench source only)                                      │  ▲         │
//                                                      degrade│  │recovered│degrade
//                                                             ▼  │+timer   ▼
//                                                          DEGRADED ◀──────┘
//   (any non-latched state) ──critical──▶ SAFE_STOP
//
// NAMING: `kRunning` IS the spec's RUNNING_FREE_SPACE — it was named before the
// contact split existed and is kept so slice-1 assertions stay literally
// unchanged (PROC-6). `kRunningContact` is its contact-side twin; both are
// "running" for every purpose except diagnostics, so callers that only care
// whether control is nominal should use IsRunning().
//
// SAFE_STOP is LATCHED: it never auto-recovers (§10.6 "자동 복구 금지"), the only
// exit is ResetFault() (the ~/reset_fault service). This latch is deliberately
// SEPARATE from the CM global E-STOP bool — a controller-local fault must not be
// cleared by a global E-STOP clear, and vice versa (E-8 design). BeginBiasCalibration()
// respects the latch for the same reason: re-activation must not launder a fault.
//
// Wrench loss is DELIBERATELY not critical. §10.6 opens by calling a dead wrench
// source "정상과 치명적 사이의 중간 상태", and collapsing it to either end gives
// over-stopping or dangerous ignoring — so wrench_timeout / quality_low degrade
// and never latch.
#pragma once

#include <cstdint>

namespace rtc::compliance {

enum class ComplianceState : std::uint8_t {
  kBiasCalibrating,  ///< N-sample wrench bias average (§3.2.1); wrench source only
  kHolding,          ///< gain ramp-up after activation (§10.7)
  kRunning,          ///< nominal compliant control — spec RUNNING_FREE_SPACE
  kRunningContact,   ///< nominal compliant control, in contact (‖f‖ > threshold)
  kDegraded,         ///< reduced gains / increased compliance; recoverable
  kSafeStop,         ///< latched; exits only via ResetFault()
};

[[nodiscard]] inline constexpr const char* ToString(ComplianceState s) noexcept {
  switch (s) {
    case ComplianceState::kBiasCalibrating:
      return "BIAS_CALIBRATING";
    case ComplianceState::kHolding:
      return "HOLDING";
    case ComplianceState::kRunning:
      return "RUNNING_FREE_SPACE";
    case ComplianceState::kRunningContact:
      return "RUNNING_CONTACT";
    case ComplianceState::kDegraded:
      return "DEGRADED";
    case ComplianceState::kSafeStop:
      return "SAFE_STOP";
  }
  return "UNKNOWN";
}

/// True for both halves of the RUNNING_FREE_SPACE ⇄ RUNNING_CONTACT split.
[[nodiscard]] inline constexpr bool IsRunning(ComplianceState s) noexcept {
  return s == ComplianceState::kRunning || s == ComplianceState::kRunningContact;
}

// Per-tick fault inputs.
struct ComplianceFaults {
  // → SAFE_STOP (critical, latched)
  bool nan_inf{false};               ///< non-finite command detected (§10.5 step 5)
  bool pose_error_exceeded{false};   ///< task pose error past its safety bound
  bool sigma_below_critical{false};  ///< σ_min(J_S) < singularity_critical
  /// ‖q_cmd − q_meas‖ past its bound — only reachable on the position-output
  /// path with `integrate_from_measured: false` (§7.3 MUST: "‖q_cmd−q_meas‖
  /// 상한 감시 필수"). CRITICAL rather than degrading: the command has already
  /// wound away from the arm, so continuing widens the gap, and there is no
  /// reduced-authority mode to fall back to the way a lost wrench has one.
  bool command_divergence{false};

  // → DEGRADED (recoverable)
  bool saturation_persist{false};     ///< torque saturation held > saturation_persist_time
  bool sigma_below_threshold{false};  ///< σ_min(J_S) < singularity_threshold
  bool wrench_timeout{false};         ///< external wrench older than wrench_timeout (§10.6)
  bool quality_low{false};            ///< wrench source quality below its bound (§3.2.4)
  /// The primary device's joint state is unusable this tick — `valid == false`
  /// (the backend has not reported yet, or stopped) or fewer channels than the
  /// model has DOF. DEGRADED and not critical: the backend recovering is the
  /// normal case, and there is a reduced-authority answer (emit no command)
  /// that a lost joint state does not rule out the way a diverged command does.
  bool device_state_invalid{false};

  [[nodiscard]] bool AnyCritical() const noexcept {
    return nan_inf || pose_error_exceeded || sigma_below_critical || command_divergence;
  }

  [[nodiscard]] bool AnyDegrade() const noexcept {
    return saturation_persist || sigma_below_threshold || wrench_timeout || quality_low ||
           device_state_invalid;
  }
};

class ComplianceStateMachine {
 public:
  // Advance one tick. `ramp_done` gates HOLDING→RUNNING; `dt` (s) accrues the
  // DEGRADED→RUNNING recovery timer. A critical fault forces SAFE_STOP from any
  // non-latched state. Returns the resulting state. RT-safe (noexcept, no alloc).
  //
  // `bias_done` and `in_contact` default to the slice-1 answers (no wrench
  // source ⇒ nothing to calibrate, never in contact), so every pre-existing
  // 4-argument call keeps its exact previous behaviour.
  ComplianceState Step(const ComplianceFaults& f, bool ramp_done, double dt,
                       double degraded_recovery_time, bool bias_done = true,
                       bool in_contact = false) noexcept {
    // SAFE_STOP is latched — no fault input can leave it, only ResetFault().
    if (state_ == ComplianceState::kSafeStop)
      return state_;

    if (f.AnyCritical()) {
      state_ = ComplianceState::kSafeStop;
      degraded_elapsed_ = 0.0;
      return state_;
    }

    const ComplianceState running =
        in_contact ? ComplianceState::kRunningContact : ComplianceState::kRunning;

    switch (state_) {
      case ComplianceState::kBiasCalibrating:
        // Bias averaging precedes the gain ramp (§10.6 lattice): commit the bias
        // first, then HOLDING runs the ramp. A degrade cause raised while
        // calibrating is honoured at the HOLDING→RUNNING edge below, not here —
        // leaving BIAS_CALIBRATING early would publish an uncalibrated bias.
        if (bias_done)
          state_ = ComplianceState::kHolding;
        break;

      case ComplianceState::kHolding:
        if (ramp_done)
          state_ = f.AnyDegrade() ? ComplianceState::kDegraded : running;
        break;

      case ComplianceState::kRunning:
      case ComplianceState::kRunningContact:
        if (f.AnyDegrade()) {
          state_ = ComplianceState::kDegraded;
          degraded_elapsed_ = 0.0;
        } else {
          state_ = running;  // free-space ⇄ contact, hysteresis owned by the caller
        }
        break;

      case ComplianceState::kDegraded:
        if (f.AnyDegrade()) {
          degraded_elapsed_ = 0.0;  // cause still present — restart the timer
        } else {
          degraded_elapsed_ += (dt > 0.0 ? dt : 0.0);
          if (degraded_elapsed_ >= degraded_recovery_time)
            state_ = running;
        }
        break;

      case ComplianceState::kSafeStop:
        break;  // unreachable (handled above)
    }
    return state_;
  }

  // Enter BIAS_CALIBRATING (§3.2.1 "on_activate 시 자동 1회 수행"). Called by the
  // controller on every (re)seed when a wrench source is configured.
  //
  // A latched SAFE_STOP wins: re-activation happens on the tick after every hold
  // tick, so accepting it here would silently launder a latched fault into a
  // clean state and defeat E-8. ResetFault() remains the only exit.
  void BeginBiasCalibration() noexcept {
    if (state_ == ComplianceState::kSafeStop)
      return;
    state_ = ComplianceState::kBiasCalibrating;
    degraded_elapsed_ = 0.0;
  }

  // The ONLY exit from SAFE_STOP (~/reset_fault). Returns to HOLDING so the
  // controller re-seeds and re-ramps from the current measured state. (With a
  // wrench source the following re-seed pushes it on to BIAS_CALIBRATING.)
  void ResetFault() noexcept {
    state_ = ComplianceState::kHolding;
    degraded_elapsed_ = 0.0;
  }

  [[nodiscard]] ComplianceState state() const noexcept { return state_; }

  [[nodiscard]] bool in_safe_stop() const noexcept { return state_ == ComplianceState::kSafeStop; }

 private:
  ComplianceState state_{ComplianceState::kHolding};
  double degraded_elapsed_{0.0};
};

}  // namespace rtc::compliance
