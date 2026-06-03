// Unit tests for rtc drift / rate filters:
//   - SlidingTrendDetector (sliding_trend_detector.hpp): O(1) OLS slope over a
//     fixed sliding window — window clamping, warmup gating, exact slope on a
//     linear ramp (validates the steady-state recurrence), drift flagging,
//     sample-rate scaling, multi-channel independence, Reset.
//   - SensorRateEstimator (sensor_rate_estimator.hpp): nominal seed, warmup
//     gating, deterministic outlier rejection (rapid ticks), Reset, and a
//     direction-only convergence check (robust under CI load — no tight
//     wall-clock bound).
//
// Pure logic; sandbox-safe (no RT permissions needed).

#include "rtc_base/filters/sensor_rate_estimator.hpp"
#include "rtc_base/filters/sliding_trend_detector.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <thread>

namespace {

using namespace std::chrono_literals;

// ── SlidingTrendDetector ─────────────────────────────────────────────────────

TEST(SlidingTrendDetectorTest, InitClampsWindow) {
  rtc::SlidingTrendDetector<1, 16> det;
  det.Init(1, 0.1);  // below minimum → clamped to 2
  EXPECT_EQ(det.window_size(), 2U);
  det.Init(9999, 0.1);  // above MaxWindowSize → clamped to 16
  EXPECT_EQ(det.window_size(), 16U);
}

TEST(SlidingTrendDetectorTest, WarmupGatesResult) {
  rtc::SlidingTrendDetector<1, 16> det;
  det.Init(8, 0.0);
  for (std::size_t k = 0; k < 7U; ++k) {
    const auto r = det.Update({static_cast<double>(k)});
    EXPECT_FALSE(r.window_full);
    EXPECT_FALSE(det.window_full());
  }
  const auto r8 = det.Update({7.0});
  EXPECT_TRUE(r8.window_full);
  EXPECT_TRUE(det.window_full());
  EXPECT_EQ(det.count(), 8U);
}

TEST(SlidingTrendDetectorTest, ZeroSlopeOnConstantSignal) {
  rtc::SlidingTrendDetector<1, 16> det;
  det.Init(8, 1e-6);
  rtc::SlidingTrendDetector<1, 16>::Result r{};
  for (std::size_t k = 0; k < 12U; ++k) {
    r = det.Update({3.14});
  }
  EXPECT_NEAR(r.slopes[0], 0.0, 1e-9);
  EXPECT_FALSE(r.drift_flags[0]);
}

TEST(SlidingTrendDetectorTest, ExactSlopeOnLinearRamp) {
  // For uniform x-indices a pure linear ramp y = a + b*t has OLS slope == b
  // exactly, independent of window position — this validates the O(1) sliding
  // recurrence (Sxy' = Sxy - Sy + y_old + (N-1)*y_new) across many slides.
  rtc::SlidingTrendDetector<1, 16> det;
  det.Init(8, 0.05);  // per-index units (sample_rate_hz == 0)
  constexpr double b = 0.1;
  rtc::SlidingTrendDetector<1, 16>::Result r{};
  for (std::size_t k = 0; k < 20U; ++k) {  // run well past warmup → many slides
    r = det.Update({2.0 + b * static_cast<double>(k)});
  }
  EXPECT_NEAR(r.slopes[0], b, 1e-9);
  EXPECT_TRUE(r.drift_flags[0]);  // |0.1| > 0.05
}

TEST(SlidingTrendDetectorTest, SampleRateScalesSlope) {
  rtc::SlidingTrendDetector<1, 16> det;
  constexpr double rate = 100.0;
  det.Init(8, 0.0, rate);
  constexpr double b = 0.1;  // per-index slope
  rtc::SlidingTrendDetector<1, 16>::Result r{};
  for (std::size_t k = 0; k < 20U; ++k) {
    r = det.Update({b * static_cast<double>(k)});
  }
  EXPECT_NEAR(r.slopes[0], b * rate, 1e-7);  // reported in units/second
}

TEST(SlidingTrendDetectorTest, MultiChannelIndependent) {
  rtc::SlidingTrendDetector<2, 16> det;
  det.Init(8, 0.05);
  rtc::SlidingTrendDetector<2, 16>::Result r{};
  for (std::size_t k = 0; k < 20U; ++k) {
    const double kd = static_cast<double>(k);
    r = det.Update({0.2 * kd, 0.0});  // ch0 ramps, ch1 constant
  }
  EXPECT_NEAR(r.slopes[0], 0.2, 1e-9);
  EXPECT_NEAR(r.slopes[1], 0.0, 1e-9);
  EXPECT_TRUE(r.drift_flags[0]);
  EXPECT_FALSE(r.drift_flags[1]);
}

TEST(SlidingTrendDetectorTest, ResetClearsState) {
  rtc::SlidingTrendDetector<1, 16> det;
  det.Init(8, 0.0);
  for (std::size_t k = 0; k < 10U; ++k) {
    (void)det.Update({static_cast<double>(k)});
  }
  det.Reset();
  EXPECT_EQ(det.count(), 0U);
  EXPECT_FALSE(det.window_full());
}

// ── SensorRateEstimator ──────────────────────────────────────────────────────

TEST(SensorRateEstimatorTest, NominalSeedBeforeWarmup) {
  rtc::SensorRateEstimator est;
  est.Init(500.0);
  EXPECT_DOUBLE_EQ(est.dt_sec(), 1.0 / 500.0);
  EXPECT_DOUBLE_EQ(est.rate_hz(), 500.0);
  EXPECT_FALSE(est.warmed_up());
  EXPECT_EQ(est.tick_count(), 0);
  EXPECT_FALSE(est.deviation_warning());
}

TEST(SensorRateEstimatorTest, WarmupGatingAndRapidTicksRejected) {
  rtc::SensorRateEstimator est;
  est.Init(500.0, 0.01, 5);  // warmup after 5 ticks
  for (int k = 0; k < 5; ++k) {
    est.Tick();
  }
  EXPECT_TRUE(est.warmed_up());
  EXPECT_EQ(est.tick_count(), 5);
  // Rapid back-to-back ticks → dt ≈ 0 → ratio < kOutlierLower (0.33) → every
  // sample rejected → EMA stays at the nominal seed → estimate unchanged.
  EXPECT_NEAR(est.rate_hz(), 500.0, 1e-6);
  EXPECT_FALSE(est.deviation_warning(10.0));
}

TEST(SensorRateEstimatorTest, EstimateMovesTowardSlowerMeasuredRate) {
  rtc::SensorRateEstimator est;
  est.Init(500.0, 0.1, 3);  // nominal 500 Hz (2 ms seed), warmup 3
  for (int k = 0; k < 12; ++k) {
    std::this_thread::sleep_for(4ms);  // ~250 Hz cadence — slower than nominal
    est.Tick();
  }
  // Direction-only assertion: a sustained slower cadence must pull the estimate
  // below nominal. Robust to CI jitter (measured dt only grows under load, and
  // outliers >3x are rejected so the estimate stays bounded).
  EXPECT_LT(est.rate_hz(), 500.0);
  EXPECT_GT(est.rate_hz(), 50.0);  // still sane (mean dt < 20 ms)
  EXPECT_TRUE(est.warmed_up());
}

TEST(SensorRateEstimatorTest, ResetRestoresNominal) {
  rtc::SensorRateEstimator est;
  est.Init(500.0, 0.1, 3);
  for (int k = 0; k < 5; ++k) {
    std::this_thread::sleep_for(1ms);
    est.Tick();
  }
  est.Reset();
  EXPECT_EQ(est.tick_count(), 0);
  EXPECT_FALSE(est.warmed_up());
  EXPECT_DOUBLE_EQ(est.dt_sec(), 1.0 / 500.0);
}

}  // namespace
