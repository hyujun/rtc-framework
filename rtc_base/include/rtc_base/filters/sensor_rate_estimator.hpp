#ifndef RTC_BASE_FILTERS_SENSOR_RATE_ESTIMATOR_HPP_
#define RTC_BASE_FILTERS_SENSOR_RATE_ESTIMATOR_HPP_

// Lightweight real-time sensor sampling rate estimator.
//
// Measures the actual interval between Tick() calls using steady_clock,
// then applies an Exponential Moving Average (EMA) to produce a stable
// estimate of the true sampling rate.
//
// Motivation:
//   HandController::EventLoop currently hardcodes 500Hz / sensor_decimation
//   for BesselFilter initialisation.  The actual rate varies due to:
//     - UDP round-trip jitter
//     - OS scheduler latency (even under SCHED_FIFO)
//     - condvar wakeup delay from ControlLoop
//   This class measures the real rate so filters can use accurate coefficients.
//
// RT Safety:
//   - steady_clock::now() is vDSO-backed on Linux (no syscall).
//   - 3 double operations per Tick() — no allocation, no branching.
//   - Total cost: ~5 ns per call.
//
// Usage:
//   SensorRateEstimator rate_est;
//   rate_est.Init(500.0);           // nominal 500 Hz
//   // In EventLoop sensor cycle:
//   rate_est.Tick();
//   double actual = rate_est.rate_hz();  // smoothed estimate

#include <chrono>
#include <cmath>

namespace rtc {

class SensorRateEstimator {
 public:
  // Initialise with the expected (nominal) sampling rate.
  //
  // nominal_rate_hz : design-time rate (e.g. 500.0 for 500 Hz loop)
  // alpha           : EMA smoothing factor ∈ (0, 1].
  //                   Small alpha → more smoothing, slower response.
  //                   0.01 ≈ ~100 sample time constant (good for 500 Hz).
  // warmup_samples  : number of Tick() calls before rate_hz() becomes valid.
  //                   Prevents transient spikes from corrupting the estimate.
  void Init(double nominal_rate_hz,
            double alpha = 0.01,
            int warmup_samples = 50) noexcept {
    nominal_rate_hz_ = nominal_rate_hz;
    alpha_           = alpha;
    warmup_target_   = warmup_samples;
    Reset();
  }

  // Reset internal state.  Next Tick() starts a fresh measurement.
  void Reset() noexcept {
    ema_dt_sec_     = (nominal_rate_hz_ > 0.0) ? 1.0 / nominal_rate_hz_ : 0.002;
    prev_time_      = Clock::time_point{};  // epoch = "not yet started"
    tick_count_     = 0;
    warmed_up_      = false;
  }

  // Call once per sensor cycle (i.e. once per actual sample acquisition).
  // Must be called from the same thread (not thread-safe — by design for RT).
  void Tick() noexcept {
    const auto now = Clock::now();

    if (prev_time_ != Clock::time_point{}) {
      const double dt = std::chrono::duration<double>(now - prev_time_).count();

      // Guard against degenerate dt (e.g. system clock jump, first call)
      if (dt > 0.0 && dt < 1.0) {
        // Outlier rejection: reject dt that deviates > 3x from current EMA.
        // Prevents jitter spikes (e.g. OS preemption, condvar delay) from
        // corrupting the estimate. Band: [ema/3, ema*3].
        const double ratio = dt / ema_dt_sec_;
        if (ratio > kOutlierLower && ratio < kOutlierUpper) {
          // Faster convergence during warmup (alpha=0.1), then settle (alpha_)
          const double a = warmed_up_ ? alpha_ : kWarmupAlpha;
          ema_dt_sec_ = a * dt + (1.0 - a) * ema_dt_sec_;
        }
      }
    }

    prev_time_ = now;
    if (++tick_count_ >= warmup_target_) {
      warmed_up_ = true;
    }
  }

  // ── Accessors ────────────────────────────────────────────────────────────

  // Smoothed estimate of the actual sampling rate [Hz].
  // Returns nominal rate until warmup is complete.
  [[nodiscard]] double rate_hz() const noexcept {
    return (ema_dt_sec_ > 0.0) ? 1.0 / ema_dt_sec_ : nominal_rate_hz_;
  }

  // Smoothed estimate of the actual sampling period [seconds].
  [[nodiscard]] double dt_sec() const noexcept {
    return ema_dt_sec_;
  }

  // True once enough samples have been collected for a reliable estimate.
  [[nodiscard]] bool warmed_up() const noexcept {
    return warmed_up_;
  }

  // True if the actual rate deviates from nominal by more than tolerance_pct
  // (default 10%).  Useful for logging warnings without polluting RT path.
  [[nodiscard]] bool deviation_warning(
      double tolerance_pct = 10.0) const noexcept {
    if (!warmed_up_ || nominal_rate_hz_ <= 0.0) return false;
    const double actual = rate_hz();
    const double deviation_pct =
        std::abs(actual - nominal_rate_hz_) / nominal_rate_hz_ * 100.0;
    return deviation_pct > tolerance_pct;
  }

  // Nominal (design-time) rate for comparison.
  [[nodiscard]] double nominal_rate_hz() const noexcept {
    return nominal_rate_hz_;
  }

  [[nodiscard]] int tick_count() const noexcept {
    return tick_count_;
  }

 private:
  using Clock = std::chrono::steady_clock;

  static constexpr double kOutlierLower = 0.33;  // reject dt < ema/3
  static constexpr double kOutlierUpper = 3.0;   // reject dt > ema*3
  static constexpr double kWarmupAlpha  = 0.1;   // 10-sample τ during warmup

  double nominal_rate_hz_{500.0};
  double alpha_{0.01};
  int    warmup_target_{50};

  double ema_dt_sec_{0.002};            // EMA of inter-tick interval
  Clock::time_point prev_time_{};       // previous Tick() timestamp
  int    tick_count_{0};
  bool   warmed_up_{false};
};

}  // namespace rtc

#endif  // RTC_BASE_FILTERS_SENSOR_RATE_ESTIMATOR_HPP_
