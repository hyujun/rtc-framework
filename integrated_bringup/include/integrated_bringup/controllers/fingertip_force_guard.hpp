#ifndef INTEGRATED_BRINGUP_CONTROLLERS_FINGERTIP_FORCE_GUARD_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_FINGERTIP_FORCE_GUARD_HPP_

// Delta-spike guard on the raw fingertip force triplet, applied on the path into
// the per-axis force LPF banks — BOTH of them: contact_stop's and force_pi's.
// Rejecting wire artefacts is a property of the signal source, not of the
// consumer, so the two banks share one guard instance per fingertip and differ
// only in cutoff.
//
// Why this exists (260730_1017 p1b session, ~123 s / 122864 ticks): six samples
// exceeded 6 N on `fz` at the index / ring fingertips, each lasting exactly
// 2 ticks (~4 ms at 500 Hz) before returning to 0.00~0.04 N. Because the
// Bessel LPF ate those samples directly, a 6.406 N wire spike left ~0.170 N of
// filtered tail behind it — a false contact_stop trigger waiting to happen at a
// 1 N threshold. A per-axis jump that large in one tick is not mechanics; it is
// the wire.
//
// Contract:
//   * The guard NEVER touches the raw force. `ft.force`, GraspState,
//     the pull estimator and the CSV `ft_<name>_fx|fy|fz` columns keep the
//     driver's wire value — this type only chooses what the LPFs are fed.
//   * The output is NOT a filtered signal. Every value it emits is a triplet the
//     sensor actually reported (this one, or the last accepted one), so a
//     consumer downstream of the guard is still looking at unsmoothed samples.
//   * Rejection is per TRIPLET, not per axis: one bad axis holds all three, so
//     the vector fed to the filter is always a triplet the sensor actually
//     reported and never a mix of two epochs.
//   * `last_accepted` is the comparison base for every following tick, not the
//     previous raw sample — otherwise a spike would re-baseline the guard and
//     the return to the true value would itself be rejected.
//
// Escape hatch (`max_hold_ticks`) — the part that is NOT in the original spec.
// Holding forever whenever |Δ| stays above the threshold makes a genuine fast
// contact step unobservable: raw 6 N sustained is `|6-0| > 4` on EVERY tick, so
// the filters would stay pinned at the pre-contact value, contact_stop would
// never fire and the Force-PI law would never see the object it is squeezing
// (fail-dangerous, and pinned by the existing
// FilteredForceLagsRawWhilePublishedStaysRaw tests). After `max_hold_ticks`
// consecutive rejections the raw triplet is accepted and becomes the new base:
// the observed 2-tick spikes are still fully suppressed, while a real step
// costs `max_hold_ticks` ticks of delay (~4 ms) against the filter's own
// ~6.7 ms group delay.
//
// NaN / Inf never escape. A non-finite sample admitted into an IIR poisons the
// delay line permanently, and downstream `std::max` / clamp arithmetic launders
// the NaN into a plausible-looking number instead of failing loudly. On the
// force_pi side that laundering runs all the way out: `std::clamp` passes NaN
// through (both of its comparisons are false), no controller checks finiteness
// on the command path, and `s` reaches the hand as a joint command.
//
// RT-safe: fixed-size state, no allocation, no logging, no locks. Header-only
// so the no-malloc gate can instantiate it in the test TU.

#include <array>
#include <cmath>
#include <cstddef>

namespace integrated_bringup {

/// Tunables, YAML-sourced per robot (`fsm.contact_stop_force_guard_*`).
struct FingertipForceGuardConfig {
  /// Per-axis rejection threshold [N]. A triplet is rejected when ANY axis
  /// differs from the last accepted triplet's same axis by more than this.
  /// Must be > 0 and finite.
  double delta_max_n{4.0};
  /// Consecutive rejections tolerated before the out-of-band raw triplet is
  /// accepted anyway [ticks]. 0 disables the delta hold entirely (non-finite
  /// samples are still rejected). Does not apply to NaN / Inf.
  int max_hold_ticks{2};
};

/// Per-fingertip guard state. One instance per fingertip: an invalid lane must
/// be able to drop its own history without disturbing its neighbours.
class FingertipForceGuard {
 public:
  struct Result {
    /// What the LPF should be fed: the raw triplet when accepted, the last
    /// accepted triplet when held, all-zero while no finite sample has ever
    /// been seen.
    std::array<double, 3> value{};
    /// True on ticks where the raw triplet was replaced by the held value.
    bool rejected{false};
  };

  /// Advance one tick. `raw` is the driver's wire force for this fingertip.
  [[nodiscard]] Result Apply(const std::array<double, 3>& raw,
                             const FingertipForceGuardConfig& cfg) noexcept {
    if (!IsFinite(raw)) {
      // No escape hatch here — see the header comment. Unprimed stays at the
      // documented safe zero rather than publishing a garbage triplet.
      return Reject();
    }
    if (!primed_) {
      return Accept(raw);
    }
    bool out_of_band = false;
    for (std::size_t j = 0; j < 3; ++j) {
      if (std::abs(raw[j] - last_accepted_[j]) > cfg.delta_max_n) {
        out_of_band = true;
        break;
      }
    }
    if (out_of_band && reject_run_ < cfg.max_hold_ticks) {
      return Reject();
    }
    // Either in band, or the hold cap is exhausted: this value is the sensor's
    // considered opinion, so it becomes the new base.
    return Accept(raw);
  }

  /// Drop all history. Called when the fingertip lane goes invalid, together
  /// with the LPF's own primed flag: after a dropout the next finite sample is
  /// a first sample again, and comparing it against a base the world has left
  /// behind would reject the live force indefinitely.
  void Invalidate() noexcept {
    primed_ = false;
    reject_run_ = 0;
    last_accepted_ = {};
  }

  /// True once a finite triplet has been accepted.
  [[nodiscard]] bool primed() const noexcept { return primed_; }

  /// Consecutive rejections so far (resets on any acceptance).
  [[nodiscard]] int consecutive_rejections() const noexcept { return reject_run_; }

  /// The current comparison base / held output.
  [[nodiscard]] const std::array<double, 3>& last_accepted() const noexcept {
    return last_accepted_;
  }

 private:
  [[nodiscard]] static bool IsFinite(const std::array<double, 3>& v) noexcept {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
  }

  [[nodiscard]] Result Accept(const std::array<double, 3>& raw) noexcept {
    last_accepted_ = raw;
    primed_ = true;
    reject_run_ = 0;
    return {raw, false};
  }

  [[nodiscard]] Result Reject() noexcept {
    ++reject_run_;
    // Unprimed: last_accepted_ is still the zero-initialised triplet, which is
    // exactly the safe-zero contract.
    return {last_accepted_, true};
  }

  std::array<double, 3> last_accepted_{};
  bool primed_{false};
  int reject_run_{0};
};

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_FINGERTIP_FORCE_GUARD_HPP_
