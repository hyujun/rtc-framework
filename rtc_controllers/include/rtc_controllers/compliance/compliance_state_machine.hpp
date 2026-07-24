// ── Compliance controller state machine (reduced §10.6) ─────────────────────
// Slice-1 reduction of spec §10.6: no wrench source yet, so BIAS_CALIBRATING and
// the RUNNING_FREE_SPACE⇄RUNNING_CONTACT split (both wrench-driven) are omitted.
// The retained lattice is
//
//   HOLDING ──ramp done──▶ RUNNING ──degrade──▶ DEGRADED
//                              ▲                    │
//                              └──recovered+timer───┘
//   (any of RUNNING/DEGRADED/HOLDING) ──critical──▶ SAFE_STOP
//
// SAFE_STOP is LATCHED: it never auto-recovers (§10.6 "자동 복구 금지"), the only
// exit is ResetFault() (the ~/reset_fault service). This latch is deliberately
// SEPARATE from the CM global E-STOP bool — a controller-local fault must not be
// cleared by a global E-STOP clear, and vice versa (E-8 design).
#pragma once

#include <cstdint>

namespace rtc::compliance {

enum class ComplianceState : std::uint8_t {
  kHolding,   ///< gain ramp-up after activation (§10.7)
  kRunning,   ///< nominal compliant control
  kDegraded,  ///< reduced gains / increased compliance; recoverable
  kSafeStop,  ///< latched; exits only via ResetFault()
};

[[nodiscard]] inline constexpr const char* ToString(ComplianceState s) noexcept {
  switch (s) {
    case ComplianceState::kHolding:
      return "HOLDING";
    case ComplianceState::kRunning:
      return "RUNNING";
    case ComplianceState::kDegraded:
      return "DEGRADED";
    case ComplianceState::kSafeStop:
      return "SAFE_STOP";
  }
  return "UNKNOWN";
}

// Per-tick fault inputs. Slice 1 has no wrench-timeout / estimator-quality
// signals; those fields arrive with the wrench source (slice 2).
struct ComplianceFaults {
  // → SAFE_STOP (critical, latched)
  bool nan_inf{false};               ///< non-finite command detected (§10.5 step 5)
  bool pose_error_exceeded{false};   ///< task pose error past its safety bound
  bool sigma_below_critical{false};  ///< σ_min(J_S) < singularity_critical

  // → DEGRADED (recoverable)
  bool saturation_persist{false};     ///< torque saturation held > saturation_persist_time
  bool sigma_below_threshold{false};  ///< σ_min(J_S) < singularity_threshold

  [[nodiscard]] bool AnyCritical() const noexcept {
    return nan_inf || pose_error_exceeded || sigma_below_critical;
  }

  [[nodiscard]] bool AnyDegrade() const noexcept {
    return saturation_persist || sigma_below_threshold;
  }
};

class ComplianceStateMachine {
 public:
  // Advance one tick. `ramp_done` gates HOLDING→RUNNING; `dt` (s) accrues the
  // DEGRADED→RUNNING recovery timer. A critical fault forces SAFE_STOP from any
  // non-latched state. Returns the resulting state. RT-safe (noexcept, no alloc).
  ComplianceState Step(const ComplianceFaults& f, bool ramp_done, double dt,
                       double degraded_recovery_time) noexcept {
    // SAFE_STOP is latched — no fault input can leave it, only ResetFault().
    if (state_ == ComplianceState::kSafeStop)
      return state_;

    if (f.AnyCritical()) {
      state_ = ComplianceState::kSafeStop;
      degraded_elapsed_ = 0.0;
      return state_;
    }

    switch (state_) {
      case ComplianceState::kHolding:
        if (ramp_done)
          state_ = f.AnyDegrade() ? ComplianceState::kDegraded : ComplianceState::kRunning;
        break;

      case ComplianceState::kRunning:
        if (f.AnyDegrade()) {
          state_ = ComplianceState::kDegraded;
          degraded_elapsed_ = 0.0;
        }
        break;

      case ComplianceState::kDegraded:
        if (f.AnyDegrade()) {
          degraded_elapsed_ = 0.0;  // cause still present — restart the timer
        } else {
          degraded_elapsed_ += (dt > 0.0 ? dt : 0.0);
          if (degraded_elapsed_ >= degraded_recovery_time)
            state_ = ComplianceState::kRunning;
        }
        break;

      case ComplianceState::kSafeStop:
        break;  // unreachable (handled above)
    }
    return state_;
  }

  // The ONLY exit from SAFE_STOP (~/reset_fault). Returns to HOLDING so the
  // controller re-seeds and re-ramps from the current measured state.
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
