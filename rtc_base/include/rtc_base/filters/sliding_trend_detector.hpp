#ifndef RTC_BASE_FILTERS_SLIDING_TREND_DETECTOR_HPP_
#define RTC_BASE_FILTERS_SLIDING_TREND_DETECTOR_HPP_

// Real-time Sliding-Window Linear Trend Detector (OLS)
//
// Detects signal drift in uniformly-sampled sensor channels using Ordinary
// Least Squares over a fixed-size sliding window with O(1) per-tick updates.
//
// === O(1) Mathematical Optimization ==========================================
//
// For a window of N samples with uniform x-indices {0, 1, ..., N-1},
// the OLS slope is:
//
//   beta = (N * Sxy - Sx * Sy) / (N * Sxx - Sx^2)
//
// where:
//   Sx  = sum of x     = N*(N-1)/2               <- CONSTANT (precomputed)
//   Sxx = sum of x^2   = N*(N-1)*(2N-1)/6        <- CONSTANT (precomputed)
//   Sy  = sum of y                                <- maintained incrementally
//   Sxy = sum of x*y                              <- maintained incrementally
//
// When the window slides (remove y_old at index 0, insert y_new at index N-1),
// all remaining samples effectively shift their x-index down by 1.
// This gives the O(1) recurrence:
//
//   Sxy' = Sxy - Sy + y_old + (N-1) * y_new      (must use OLD Sy)
//   Sy'  = Sy  - y_old + y_new
//
// Proof sketch:
//   Old:  Sxy = 0*y0 + 1*y1 + ... + (N-1)*y_{N-1}
//   New window {y1,...,y_{N-1},y_new} re-indexed to {0,...,N-1}:
//   Sxy' = sum_{k=1}^{N-1} (k-1)*yk + (N-1)*y_new
//        = (Sxy - 0*y0) - (Sy - y0) + (N-1)*y_new
//        = Sxy - Sy + y0 + (N-1)*y_new   QED
//
// Cost: ~10 FLOPs per channel per tick (3 add + 1 mul for update,
//       2 mul + 1 add for slope).  For 8 channels: ~80 FLOPs total.
//
// === RT Safety ===============================================================
// - Zero dynamic allocation: std::array circular buffer, fixed at compile time.
// - All methods are noexcept.
// - No branches in hot path beyond the warmup check.
// ============================================================================

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "rtc_base/types/types.hpp"

namespace rtc {

template <std::size_t NumChannels, std::size_t MaxWindowSize = 2500>
class SlidingTrendDetector {
  static_assert(NumChannels > 0, "Must have at least 1 channel");
  static_assert(MaxWindowSize >= 2, "Window must hold at least 2 samples");

 public:
  struct Result {
    std::array<double, NumChannels> slopes{};       // per-channel slope
    std::array<bool, NumChannels>   drift_flags{};  // |slope| > threshold
    bool window_full{false};                        // true once warmup complete
  };

  // Initialise detector parameters.  Call once before Update().
  //
  // window_size      : sliding window length in samples (clamped to [2, MaxWindowSize])
  // drift_threshold  : flag channels whose |slope| exceeds this value
  // sample_rate_hz   : if > 0, slopes are reported in [units/second]
  //                    if == 0 (default), slopes are in [units/sample-index]
  //                    drift_threshold must be in the same unit system.
  void Init(std::size_t window_size, double drift_threshold,
            double sample_rate_hz = 0.0) noexcept {
    n_ = (window_size < 2) ? 2
       : (window_size > MaxWindowSize) ? MaxWindowSize
       : window_size;
    drift_threshold_ = drift_threshold;
    sample_rate_hz_  = sample_rate_hz;

    // Precompute constants for x = {0, 1, ..., N-1}
    const double nd = static_cast<double>(n_);
    sum_x_    = nd * (nd - 1.0) * 0.5;                          // N(N-1)/2
    double sum_xx = nd * (nd - 1.0) * (2.0 * nd - 1.0) / 6.0;  // N(N-1)(2N-1)/6
    double denom  = nd * sum_xx - sum_x_ * sum_x_;               // N*Sxx - Sx^2
    inv_denom_    = 1.0 / denom;  // multiply by reciprocal in hot path
    nm1_          = static_cast<double>(n_ - 1);

    Reset();
  }

  // Reset all internal state (sums, buffer, counters).
  void Reset() noexcept {
    count_ = 0;
    head_  = 0;
    sum_y_.fill(0.0);
    sum_xy_.fill(0.0);
    // buffer_ elements are overwritten before read — no need to zero.
  }

  // Feed one multi-channel sample and return the current detection result.
  // O(1) time complexity.  Called once per tick (e.g. every 2 ms at 500 Hz).
  [[nodiscard]] Result Update(
      const std::array<double, NumChannels>& sample) noexcept {

    if (count_ < n_) {
      // ── Warmup phase: accumulate sums, no removal ─────────────────────
      const double idx = static_cast<double>(count_);
      buffer_[head_] = sample;
      for (std::size_t c = 0; c < NumChannels; ++c) {
        sum_y_[c]  += sample[c];
        sum_xy_[c] += idx * sample[c];
      }
      AdvanceHead();
      ++count_;
    } else {
      // ── Steady state: O(1) sliding update ─────────────────────────────
      // head_ points to the oldest sample (next to be overwritten).
      const auto& y_old = buffer_[head_];

      for (std::size_t c = 0; c < NumChannels; ++c) {
        // Update Sxy BEFORE Sy — the recurrence uses the OLD Sy.
        sum_xy_[c] = sum_xy_[c] - sum_y_[c] + y_old[c] + nm1_ * sample[c];
        sum_y_[c]  = sum_y_[c]  - y_old[c]  + sample[c];
      }

      buffer_[head_] = sample;
      AdvanceHead();
    }

    // ── Compute slopes & drift flags ──────────────────────────────────────
    Result result{};
    result.window_full = (count_ >= n_);

    if (result.window_full) {
      const double nd    = static_cast<double>(n_);
      const double scale = (sample_rate_hz_ > 0.0) ? sample_rate_hz_ : 1.0;

      for (std::size_t c = 0; c < NumChannels; ++c) {
        // slope = (N * Sxy - Sx * Sy) / D,  using precomputed 1/D
        const double beta = (nd * sum_xy_[c] - sum_x_ * sum_y_[c]) * inv_denom_;
        result.slopes[c]      = beta * scale;
        result.drift_flags[c] = std::abs(result.slopes[c]) > drift_threshold_;
      }
    }

    return result;
  }

  // ── Accessors ────────────────────────────────────────────────────────────

  [[nodiscard]] std::size_t window_size()  const noexcept { return n_; }
  [[nodiscard]] std::size_t count()        const noexcept { return count_; }
  [[nodiscard]] bool        window_full()  const noexcept { return count_ >= n_; }
  [[nodiscard]] double      drift_threshold() const noexcept { return drift_threshold_; }

  // Update drift threshold at runtime (e.g. based on operating mode).
  void set_drift_threshold(double threshold) noexcept {
    drift_threshold_ = threshold;
  }

  // Update sample rate at runtime (e.g. from SensorRateEstimator).
  void set_sample_rate_hz(double hz) noexcept {
    sample_rate_hz_ = hz;
  }

 private:
  void AdvanceHead() noexcept {
    // Branchless-friendly: avoids integer division (modulo).
    if (++head_ >= n_) head_ = 0;
  }

  // ── Configuration ──────────────────────────────────────────────────────
  std::size_t n_{MaxWindowSize};     // runtime window size (≤ MaxWindowSize)
  double drift_threshold_{0.0};
  double sample_rate_hz_{0.0};       // 0 = slopes in per-index units

  // ── Precomputed OLS constants ──────────────────────────────────────────
  double sum_x_{0.0};       // Σx   = N(N-1)/2
  double inv_denom_{0.0};   // 1 / (N·Σx² - (Σx)²)
  double nm1_{0.0};         // N - 1  (hot-path constant)

  // ── Running sums (per channel) ─────────────────────────────────────────
  std::array<double, NumChannels> sum_y_{};    // Σ y_i
  std::array<double, NumChannels> sum_xy_{};   // Σ i·y_i

  // ── Circular buffer ────────────────────────────────────────────────────
  // buffer_[head_] = oldest sample (next overwrite target in steady state).
  // Only the first n_ slots are used; remaining MaxWindowSize-n_ are unused.
  std::array<std::array<double, NumChannels>, MaxWindowSize> buffer_{};
  std::size_t head_{0};
  std::size_t count_{0};   // samples ingested so far (capped at n_)
};

// ── Convenience alias ─────────────────────────────────────────────────────────

// Per-fingertip barometer drift detector (8 baro channels, 5s window at 500Hz)
using BarometerTrendDetector = SlidingTrendDetector<kBarometerCount, 2500>;

}  // namespace rtc

#endif  // RTC_BASE_FILTERS_SLIDING_TREND_DETECTOR_HPP_
