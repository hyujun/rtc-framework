// ── test_filters.cpp ────────────────────────────────────────────────────────
// Unit tests for rtc::BesselFilterN<N> and rtc::KalmanFilterN<N>.
//
// Covers: initialisation, DC passthrough, step response, high-frequency
// attenuation, state reset (Bessel); constant signal convergence, linear
// ramp velocity estimation, noise rejection (Kalman).
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_base/filters/bessel_filter.hpp>
#include <rtc_base/filters/kalman_filter.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <random>

namespace {

// ── BesselFilterN tests ─────────────────────────────────────────────────────

TEST(BesselFilter, InitDoesNotThrow) {
  rtc::BesselFilterN<4> filter;
  EXPECT_NO_THROW(filter.Init(50.0, 500.0));
  EXPECT_TRUE(filter.initialized());
  EXPECT_DOUBLE_EQ(filter.cutoff_hz(), 50.0);
  EXPECT_DOUBLE_EQ(filter.sample_rate_hz(), 500.0);
}

TEST(BesselFilter, InitRejectsInvalidParams) {
  rtc::BesselFilterN<1> filter;
  EXPECT_THROW(filter.Init(0.0, 500.0), std::invalid_argument);
  EXPECT_THROW(filter.Init(-1.0, 500.0), std::invalid_argument);
  EXPECT_THROW(filter.Init(50.0, 0.0), std::invalid_argument);
  // Cutoff at or above Nyquist.
  EXPECT_THROW(filter.Init(250.0, 500.0), std::invalid_argument);
  EXPECT_THROW(filter.Init(300.0, 500.0), std::invalid_argument);
}

TEST(BesselFilter, DCPassthrough) {
  // A constant (DC) signal should pass through the low-pass filter unchanged.
  rtc::BesselFilterN<2> filter;
  filter.Init(50.0, 500.0);

  const std::array<double, 2> dc_input = {1.5, -3.7};
  std::array<double, 2> output{};

  // Run for 200 samples to let the filter settle.
  for (int i = 0; i < 200; ++i) {
    output = filter.Apply(dc_input);
  }

  for (std::size_t ch = 0; ch < 2; ++ch) {
    EXPECT_NEAR(output[ch], dc_input[ch], 1e-6)
        << "DC passthrough failed on channel " << ch;
  }
}

TEST(BesselFilter, StepResponse) {
  // Step from 0 to 1.0 — output should settle near 1.0 with bounded overshoot.
  // A 4th-order Bessel filter has ~2.2% overshoot in its step response, which
  // is normal and expected (maximally flat group delay, NOT monotone convergence).
  // With 50 Hz cutoff at 500 Hz sample rate, 100 samples = 0.2 s.
  rtc::BesselFilterN<1> filter;
  filter.Init(50.0, 500.0);

  const std::array<double, 1> zero = {0.0};
  const std::array<double, 1> step = {1.0};

  // Feed zeros to initialise.
  for (int i = 0; i < 50; ++i) {
    (void)filter.Apply(zero);
  }

  double max_output = 0.0;
  std::array<double, 1> output{};
  for (int i = 0; i < 100; ++i) {
    output = filter.Apply(step);
    max_output = std::max(max_output, output[0]);
  }

  // Overshoot should be bounded (4th-order Bessel: ~2.2%, allow up to 5%).
  EXPECT_LT(max_output, 1.05) << "Overshoot exceeds 5%";
  EXPECT_GT(max_output, 1.0) << "4th-order Bessel should exhibit slight overshoot";

  // After 100 samples at 500 Hz (0.2s), should be within 0.1% of final value.
  EXPECT_NEAR(output[0], 1.0, 0.001);
}

TEST(BesselFilter, HighFreqAttenuation) {
  // A 250 Hz sine at 500 Hz sample rate is at the Nyquist limit.
  // Use a signal well above the 50 Hz cutoff to verify attenuation.
  // We use 200 Hz (4x the cutoff) which should be heavily attenuated.
  rtc::BesselFilterN<1> filter;
  filter.Init(50.0, 500.0);

  constexpr double sample_rate = 500.0;
  constexpr double signal_freq = 200.0;
  constexpr double amplitude = 1.0;

  // Warm up the filter for 200 samples to clear transients.
  for (int i = 0; i < 200; ++i) {
    const double t = static_cast<double>(i) / sample_rate;
    const std::array<double, 1> input = {
        amplitude * std::sin(2.0 * std::numbers::pi * signal_freq * t)};
    (void)filter.Apply(input);
  }

  // Measure the peak output amplitude over the next 200 samples.
  double max_output = 0.0;
  for (int i = 200; i < 400; ++i) {
    const double t = static_cast<double>(i) / sample_rate;
    const std::array<double, 1> input = {
        amplitude * std::sin(2.0 * std::numbers::pi * signal_freq * t)};
    const auto output = filter.Apply(input);
    max_output = std::max(max_output, std::abs(output[0]));
  }

  // Output amplitude should be well below 10% of input.
  EXPECT_LT(max_output, 0.1 * amplitude);
}

TEST(BesselFilter, ResetClearsState) {
  rtc::BesselFilterN<1> filter;
  filter.Init(50.0, 500.0);

  const std::array<double, 1> step = {1.0};

  // First run: apply step for 20 samples and capture output.
  std::array<double, 1> first_run_samples[20];
  for (int i = 0; i < 20; ++i) {
    first_run_samples[i] = filter.Apply(step);
  }

  // Apply many more samples to change internal state significantly.
  for (int i = 0; i < 200; ++i) {
    (void)filter.Apply(step);
  }

  // Reset and re-run the same step input.
  filter.Reset();
  for (int i = 0; i < 20; ++i) {
    const auto output = filter.Apply(step);
    EXPECT_DOUBLE_EQ(output[0], first_run_samples[i][0])
        << "Reset did not fully clear state at sample " << i;
  }
}

// ── KalmanFilterN tests ─────────────────────────────────────────────────────

TEST(KalmanFilter, InitDoesNotThrow) {
  rtc::KalmanFilterN<4> kf;
  EXPECT_NO_THROW(kf.Init(0.001, 0.01, 0.1, 0.002));
  EXPECT_TRUE(kf.initialized());
  EXPECT_DOUBLE_EQ(kf.params().q_pos, 0.001);
  EXPECT_DOUBLE_EQ(kf.params().q_vel, 0.01);
  EXPECT_DOUBLE_EQ(kf.params().r, 0.1);
  EXPECT_DOUBLE_EQ(kf.params().dt, 0.002);
}

TEST(KalmanFilter, InitRejectsInvalidParams) {
  rtc::KalmanFilterN<1> kf;
  EXPECT_THROW(kf.Init(-0.001, 0.01, 0.1, 0.002), std::invalid_argument);
  EXPECT_THROW(kf.Init(0.001, -0.01, 0.1, 0.002), std::invalid_argument);
  EXPECT_THROW(kf.Init(0.001, 0.01, 0.0, 0.002), std::invalid_argument);
  EXPECT_THROW(kf.Init(0.001, 0.01, -0.1, 0.002), std::invalid_argument);
  EXPECT_THROW(kf.Init(0.001, 0.01, 0.1, 0.0), std::invalid_argument);
  EXPECT_THROW(kf.Init(0.001, 0.01, 0.1, -0.002), std::invalid_argument);
}

TEST(KalmanFilter, ConstantSignal) {
  // Feeding a constant measurement: position should converge to the constant,
  // velocity should converge to approximately zero.
  rtc::KalmanFilterN<2> kf;
  kf.Init(0.001, 0.01, 0.1, 0.002);

  const std::array<double, 2> constant = {5.0, -3.0};

  for (int i = 0; i < 500; ++i) {
    (void)kf.PredictAndUpdate(constant);
  }

  for (std::size_t ch = 0; ch < 2; ++ch) {
    EXPECT_NEAR(kf.position(ch), constant[ch], 0.01)
        << "Position did not converge on channel " << ch;
    EXPECT_NEAR(kf.velocity(ch), 0.0, 0.05)
        << "Velocity did not converge to zero on channel " << ch;
  }
}

TEST(KalmanFilter, LinearRamp) {
  // Feed a linearly increasing signal: velocity should converge to the slope.
  rtc::KalmanFilterN<1> kf;
  constexpr double dt = 0.002;          // 500 Hz
  constexpr double slope = 2.0;         // 2 units/s
  kf.Init(0.001, 0.1, 0.01, dt);       // higher q_vel to track velocity quickly

  for (int i = 0; i < 2000; ++i) {
    const double t = static_cast<double>(i) * dt;
    const std::array<double, 1> measurement = {slope * t};
    (void)kf.PredictAndUpdate(measurement);
  }

  EXPECT_NEAR(kf.velocity(0), slope, 0.1)
      << "Velocity estimate did not converge to slope";
}

TEST(KalmanFilter, NoiseRejection) {
  // Feed constant + Gaussian noise: output variance should be less than input.
  rtc::KalmanFilterN<1> kf;
  constexpr double dt = 0.002;
  constexpr double true_value = 10.0;
  constexpr double noise_stddev = 0.5;
  kf.Init(0.001, 0.01, noise_stddev * noise_stddev, dt);

  std::mt19937 rng(42);  // deterministic seed
  std::normal_distribution<double> noise(0.0, noise_stddev);

  // Warm up the filter.
  for (int i = 0; i < 500; ++i) {
    const std::array<double, 1> meas = {true_value + noise(rng)};
    (void)kf.PredictAndUpdate(meas);
  }

  // Measure output variance over the next 1000 samples.
  double sum_sq_error_input = 0.0;
  double sum_sq_error_output = 0.0;
  constexpr int kMeasureSamples = 1000;

  for (int i = 0; i < kMeasureSamples; ++i) {
    const double n = noise(rng);
    const std::array<double, 1> meas = {true_value + n};
    const auto filtered = kf.PredictAndUpdate(meas);

    sum_sq_error_input += n * n;
    const double output_error = filtered[0] - true_value;
    sum_sq_error_output += output_error * output_error;
  }

  const double input_variance = sum_sq_error_input / kMeasureSamples;
  const double output_variance = sum_sq_error_output / kMeasureSamples;

  EXPECT_LT(output_variance, input_variance)
      << "Kalman filter did not reduce noise variance"
      << " (input=" << input_variance << ", output=" << output_variance << ")";
}

}  // namespace
