// ── Fingertip contact debouncer ──────────────────────────────────────────────
// Per fingertip i: a bias b_i and noise σ̂_i are estimated by a fixed-memory
// moving average while the hand is settled at `q_pre` (ARMED/TRACKING window, driven by
// the caller — this core does not know about FSM Mode). The live decision is
//
//   f_i(t) = ‖F_i(t) − b_i‖,   c_i(t) = 1[f_i > max(f_min, k_σ·σ̂_i)]
//
// and N_deb CONSECUTIVE true samples confirm contact on sensor i. F_i [N] is
// fingertip i's force (finger-on-object sign in sim and on hardware); the
// decision sees only the magnitude of the residual, so it carries no sign
// logic to get wrong.
//
// Fixed capacity `kMaxFingertips`. No heap, noexcept throughout.
//
// ── Estimator and failure choices ────────────────────────────────────────────
//  1. The moving average is an exponential moving average (EMA) of the RAW
//     3-vector for the bias, and an EMA of the squared residual norm for the
//     noise variance:
//       b_i ← b_i + α (F_i − b_i),   σ̂_i² ← σ̂_i² + α (‖F_i − b_i‖² − σ̂_i²)
//     (residual taken before the bias update; the first sample seeds b_i with
//     σ̂_i² = 0). EMA is the only O(1)-memory estimator that fits the RT no-heap
//     constraint without fixing a sample count up front. α = `baseline_alpha`
//     is caller-supplied, so the effective window length is a deployment
//     choice, not baked in here.
//  2. A non-finite force sample is rejected without touching the estimates; a
//     non-finite LIVE sample also breaks the debounce streak (resets the
//     consecutive-true counter to 0) rather than leaving the streak untouched
//     or counting it as a hit — fail-closed: a garbage reading must never
//     contribute towards confirming contact, and "hold the streak" would let a
//     later genuine sample complete a confirmation partly built on garbage.
//  3. An out-of-range fingertip index is rejected (no state mutated, sample
//     reported as not accepted) rather than clamped or asserted; this core
//     never indexes past `kMaxFingertips`.
#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rtc::catching {

/// Hand fingertip capacity.
inline constexpr std::size_t kMaxFingertips = 4;

/// Per-fingertip bias/noise estimate, updated only while the caller judges
/// the hand to be in the learning window (ARMED/TRACKING, hand `Open`).
struct FingertipBaseline {
  Eigen::Vector3d bias{Eigen::Vector3d::Zero()};  ///< b_i [N]
  double variance{0.0};                           ///< EMA of ‖F_i − b_i‖² [N²]
  bool initialized{false};                        ///< first sample seeds bias directly
};

/// Static contact-decision configuration (YAML `supervisor.contact.*`).
struct ContactDebounceConfig {
  double f_min{0.0};            ///< [N] absolute force floor
  double k_sigma{3.0};          ///< noise multiplier
  std::uint32_t n_debounce{3};  ///< consecutive true samples to confirm
  double baseline_alpha{0.02};  ///< EMA rate for bias/variance, in (0, 1]

  /// A config is usable iff every field is finite and in its documented
  /// range (below). `n_debounce == 0` is invalid — "0 consecutive samples"
  /// is not a debounce, it is "confirm immediately", which defeats the
  /// mechanism's purpose.
  [[nodiscard]] constexpr bool Valid() const noexcept {
    return std::isfinite(f_min) && f_min >= 0.0 && std::isfinite(k_sigma) && k_sigma >= 0.0 &&
           n_debounce >= 1 && std::isfinite(baseline_alpha) && baseline_alpha > 0.0 &&
           baseline_alpha <= 1.0;
  }
};

/// Fixed-capacity, per-attempt contact debouncer for up to `kMaxFingertips`
/// sensors. One instance is owned by the supervisor and reset at
/// RETREAT→ARMED (baseline window AND debounce counters both).
class ContactDebouncer {
 public:
  /// Replace the configuration. Returns false (and leaves the previous
  /// config in effect) if `cfg` fails `Valid()` — fail-closed: an invalid
  /// config must not silently start accepting garbage thresholds.
  [[nodiscard]] bool Configure(const ContactDebounceConfig& cfg) noexcept {
    if (!cfg.Valid())
      return false;
    config_ = cfg;
    config_valid_ = true;
    return true;
  }

  /// Rearm reset: bias/noise window AND debounce counters AND the
  /// latched confirmed flags. Split into the two calls below if a caller
  /// ever needs just one of the two lists independently.
  void ResetForRearm() noexcept {
    ResetBaselineWindow();
    ResetDebounceCounters();
  }

  /// Empties the bias/noise moving-average window for every fingertip.
  void ResetBaselineWindow() noexcept {
    for (FingertipBaseline& b : baseline_)
      b = FingertipBaseline{};
  }

  /// Zeroes every fingertip's consecutive-true counter and latched
  /// confirmation.
  void ResetDebounceCounters() noexcept {
    consecutive_true_.fill(0);
    confirmed_.fill(false);
  }

  /// Feed one sample into fingertip `idx`'s bias/noise estimate. Call only
  /// while the caller's FSM judges the hand to be in the learning window
  /// (ARMED/TRACKING, hand settled at `q_pre`) — this core has no notion of Mode.
  ///
  /// @return true iff the sample was finite, `idx` in range, and accepted.
  ///         A rejected sample never touches `baseline_[idx]`.
  bool UpdateBaseline(std::size_t idx, const Eigen::Vector3d& f_sample) noexcept {
    if (idx >= kMaxFingertips || !config_valid_)
      return false;
    if (!f_sample.allFinite())
      return false;

    FingertipBaseline& b = baseline_[idx];
    if (!b.initialized) {
      b.bias = f_sample;
      b.variance = 0.0;
      b.initialized = true;
      return true;
    }

    const Eigen::Vector3d residual = f_sample - b.bias;
    const double residual_sq = residual.squaredNorm();
    const double alpha = config_.baseline_alpha;
    b.bias += residual * alpha;
    b.variance += alpha * (residual_sq - b.variance);
    return true;
  }

  /// Feed one live sample into fingertip `idx`'s contact decision and
  /// debounce counter. Call every tick regardless of Mode — the contact
  /// window (the result judgement) is the caller's concern, not this core's.
  ///
  /// A non-finite sample or an out-of-range `idx` breaks the debounce streak
  /// (choices 2/3 above) and reports not confirmed; it never
  /// touches the baseline.
  ///
  /// @return true iff fingertip `idx` is confirmed in contact AFTER this
  ///         sample (i.e. the current value of `IsConfirmed(idx)`).
  bool UpdateContact(std::size_t idx, const Eigen::Vector3d& f_sample) noexcept {
    if (idx >= kMaxFingertips)
      return false;
    if (!config_valid_ || !f_sample.allFinite() || !baseline_[idx].initialized) {
      consecutive_true_[idx] = 0;
      confirmed_[idx] = false;
      return false;
    }

    const double f_i = (f_sample - baseline_[idx].bias).norm();
    const double sigma_hat =
        std::sqrt(baseline_[idx].variance > 0.0 ? baseline_[idx].variance : 0.0);
    const double threshold = std::max(config_.f_min, config_.k_sigma * sigma_hat);
    const bool c_i = f_i > threshold;

    if (c_i) {
      if (consecutive_true_[idx] < config_.n_debounce)
        ++consecutive_true_[idx];
    } else {
      consecutive_true_[idx] = 0;
      confirmed_[idx] = false;
      return false;
    }

    if (consecutive_true_[idx] >= config_.n_debounce)
      confirmed_[idx] = true;
    return confirmed_[idx];
  }

  [[nodiscard]] bool IsConfirmed(std::size_t idx) const noexcept {
    return idx < kMaxFingertips && confirmed_[idx];
  }

  [[nodiscard]] std::uint32_t ConsecutiveTrueCount(std::size_t idx) const noexcept {
    return idx < kMaxFingertips ? consecutive_true_[idx] : 0;
  }

  [[nodiscard]] const FingertipBaseline& Baseline(std::size_t idx) const noexcept {
    return baseline_[idx < kMaxFingertips ? idx : 0];
  }

  [[nodiscard]] bool ConfigValid() const noexcept { return config_valid_; }

 private:
  ContactDebounceConfig config_{};
  bool config_valid_{false};
  std::array<FingertipBaseline, kMaxFingertips> baseline_{};
  std::array<std::uint32_t, kMaxFingertips> consecutive_true_{};
  std::array<bool, kMaxFingertips> confirmed_{};
};

}  // namespace rtc::catching
