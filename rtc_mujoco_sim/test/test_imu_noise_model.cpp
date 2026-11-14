// ── test_imu_noise_model.cpp ─────────────────────────────────────────────────
// ImuNoiseModel on its own — no MJCF, no physics. Statistics over N = 20000
// samples (white σ and mean, the bias walk's increment σ), the bias held
// constant when it is the only term, and determinism: one seed gives one
// realisation, run after run and after a Reset.
//
// Bounds: a sample σ within ±3 % of the configured one is about six standard
// errors at this N (the relative s.e. of a sample σ is 1/√(2N) ≈ 0.5 %), and
// |mean| ≤ 4 σ/√N is four standard errors of the mean.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/imu_noise_model.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <vector>

namespace rtc {
namespace {

constexpr std::size_t kN = 20000;
constexpr double kDt = 0.002;
constexpr std::uint64_t kSeed = 20261114;
constexpr double kSigmaLo = 0.97;
constexpr double kSigmaHi = 1.03;
constexpr std::size_t kAxes = ImuNoiseModel::kAxes;

using Sample = std::array<double, kAxes>;

/// N samples of channel 0 (of `channels.size()` channels, every one applied
/// once per sample, in order) fed with x = 0, so each sample is the noise.
std::vector<Sample> Draw(ImuNoiseModel& model, std::size_t n, std::size_t channel = 0) {
  std::vector<Sample> out;
  out.reserve(n);
  for (std::size_t k = 0; k < n; ++k) {
    for (std::size_t c = 0; c < model.NumChannels(); ++c) {
      Sample y{};
      model.Apply(c, y.data());
      if (c == channel)
        out.push_back(y);
    }
  }
  return out;
}

ImuNoiseModel Make(const ImuNoiseConfig& cfg,
                   std::vector<ImuNoiseKind> channels = {ImuNoiseKind::kGyro}) {
  ImuNoiseModel m;
  m.Configure(cfg, kDt, std::move(channels));
  return m;
}

bool BitEqual(const std::vector<Sample>& a, const std::vector<Sample>& b) {
  return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(Sample)) == 0;
}

void ExpectSigmaAndMean(const std::vector<double>& v, double sigma, bool check_mean,
                        const char* what) {
  const auto n = static_cast<double>(v.size());
  double mean = 0.0;
  for (double x : v)
    mean += x;
  mean /= n;
  double var = 0.0;
  for (double x : v)
    var += (x - mean) * (x - mean);
  const double s = std::sqrt(var / (n - 1.0));
  EXPECT_GE(s, kSigmaLo * sigma) << what;
  EXPECT_LE(s, kSigmaHi * sigma) << what;
  if (check_mean) {
    EXPECT_LE(std::fabs(mean), 4.0 * sigma / std::sqrt(n)) << what;
  }
}

ImuNoiseConfig Full() {
  ImuNoiseConfig cfg;
  cfg.enabled = true;
  cfg.seed = kSeed;
  cfg.gyro = {5e-4, 1e-3, 1e-5};
  cfg.accel = {1e-2, 5e-2, 1e-4};
  return cfg;
}

TEST(ImuNoiseModel, WhiteOnlyHasTheConfiguredSigmaAndZeroMean) {
  ImuNoiseConfig cfg;
  cfg.seed = kSeed;
  cfg.gyro.white_sigma = 5e-4;
  cfg.accel.white_sigma = 1e-2;
  // Two channels of different kinds, so each is also checked to take its own σ.
  auto model = Make(cfg, {ImuNoiseKind::kGyro, ImuNoiseKind::kAccel});
  for (std::size_t c = 0; c < 2; ++c) {
    model.Reset();
    const auto s = Draw(model, kN, c);
    const double sigma = c == 0 ? cfg.gyro.white_sigma : cfg.accel.white_sigma;
    for (std::size_t a = 0; a < kAxes; ++a) {
      std::vector<double> axis(kN);
      for (std::size_t k = 0; k < kN; ++k)
        axis[k] = s[k][a];
      ExpectSigmaAndMean(axis, sigma, true, c == 0 ? "gyro" : "accel");
    }
  }
}

TEST(ImuNoiseModel, BiasOnlyIsOneConstantPerAxisAndRepeats) {
  ImuNoiseConfig cfg;
  cfg.seed = kSeed;
  cfg.accel.bias_sigma = 5e-2;
  auto a = Make(cfg, {ImuNoiseKind::kAccel});
  auto b = Make(cfg, {ImuNoiseKind::kAccel});
  const auto ra = Draw(a, kN);
  const auto rb = Draw(b, kN);
  for (std::size_t ax = 0; ax < kAxes; ++ax) {
    EXPECT_NE(ra[0][ax], 0.0) << "axis " << ax << ": no bias drawn";
    for (std::size_t k = 1; k < kN; ++k) {
      ASSERT_EQ(ra[k][ax], ra[0][ax]) << "axis " << ax << " sample " << k;
    }
  }
  EXPECT_TRUE(BitEqual(ra, rb)) << "same seed, two runs";
}

TEST(ImuNoiseModel, BiasWalkIncrementHasSigmaTimesRootDt) {
  ImuNoiseConfig cfg;
  cfg.seed = kSeed;
  cfg.gyro.bias_walk_sigma = 1e-5;
  auto model = Make(cfg);
  const auto s = Draw(model, kN + 1);
  EXPECT_EQ(s[0], Sample{}) << "no initial bias and no white term: the walk starts at 0";
  for (std::size_t a = 0; a < kAxes; ++a) {
    std::vector<double> inc(kN);
    for (std::size_t k = 0; k < kN; ++k)
      inc[k] = s[k + 1][a] - s[k][a];
    ExpectSigmaAndMean(inc, cfg.gyro.bias_walk_sigma * std::sqrt(kDt), false, "walk increment");
  }
}

TEST(ImuNoiseModel, SameSeedTwoRunsAreBitIdentical) {
  auto a = Make(Full(), {ImuNoiseKind::kGyro, ImuNoiseKind::kAccel});
  auto b = Make(Full(), {ImuNoiseKind::kGyro, ImuNoiseKind::kAccel});
  for (std::size_t c = 0; c < 2; ++c) {
    a.Reset();
    b.Reset();
    EXPECT_TRUE(BitEqual(Draw(a, kN, c), Draw(b, kN, c))) << "channel " << c;
  }
  auto other_cfg = Full();
  other_cfg.seed = kSeed + 1;
  auto other = Make(other_cfg, {ImuNoiseKind::kGyro, ImuNoiseKind::kAccel});
  a.Reset();
  EXPECT_FALSE(BitEqual(Draw(a, 100), Draw(other, 100))) << "a different seed must differ";
}

TEST(ImuNoiseModel, ResetReplaysTheRunStart) {
  // One channel: Configure draws 3 biases and every sample 6 values, so the
  // draw count at the reset is odd and the distribution holds a cached half —
  // the state a reset that only re-seeds the RNG would get wrong.
  auto model = Make(Full());
  const auto start = Draw(model, 100);
  static_cast<void>(Draw(model, 777));
  model.Reset();
  EXPECT_TRUE(BitEqual(Draw(model, 100), start));
}

TEST(ImuNoiseModel, ZeroSeedIsResolvedOnceAndReplays) {
  auto cfg = Full();
  cfg.seed = 0;
  auto model = Make(cfg);
  const auto start = Draw(model, 100);
  model.Reset();
  EXPECT_TRUE(BitEqual(Draw(model, 100), start));
}

TEST(ImuNoiseModel, OutOfRangeChannelIsUntouched) {
  auto model = Make(Full());
  Sample y{1.0, 2.0, 3.0};
  model.Apply(1, y.data());
  EXPECT_EQ(y, (Sample{1.0, 2.0, 3.0}));
}

}  // namespace
}  // namespace rtc
