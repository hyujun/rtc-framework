// ── imu_noise_model.cpp ──────────────────────────────────────────────────────
#include "rtc_mujoco_sim/imu_noise_model.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <utility>

namespace rtc {

void ImuNoiseModel::Configure(const ImuNoiseConfig& config, double dt,
                              std::vector<ImuNoiseKind> channels) {
  config_ = config;
  sqrt_dt_ = dt > 0.0 ? std::sqrt(dt) : 0.0;
  if (config.seed != 0) {
    seed_ = config.seed;
  } else {
    std::random_device rd;
    seed_ = (static_cast<std::uint64_t>(rd()) << 32) ^ static_cast<std::uint64_t>(rd());
  }
  // The same seed may drive another of the simulator's RNGs (a profile with
  // one experiment seed), so the stream is tagged, as the camera noise is.
  std::seed_seq seq{static_cast<std::uint32_t>(seed_), static_cast<std::uint32_t>(seed_ >> 32U),
                    static_cast<std::uint32_t>(0x696d75U)};
  std::array<std::uint32_t, 2> words{};
  seq.generate(words.begin(), words.end());
  stream_seed_ = (static_cast<std::uint64_t>(words[0]) << 32U) | words[1];
  kinds_ = std::move(channels);
  bias_.assign(kinds_.size(), std::array<double, kAxes>{});
  Reset();
}

void ImuNoiseModel::Reset() noexcept {
  rng_.seed(stream_seed_);
  // normal_distribution draws in pairs and keeps the second one; without this
  // a reset after an odd number of draws would start from the stale half.
  unit_.reset();
  for (std::size_t c = 0; c < kinds_.size(); ++c) {
    const double sigma = ParamsOf(c).bias_sigma;
    for (auto& b : bias_[c]) {
      b = sigma * unit_(rng_);
    }
  }
}

void ImuNoiseModel::Apply(std::size_t channel, double* xyz) noexcept {
  if (channel >= kinds_.size() || xyz == nullptr) {
    return;
  }
  const ImuNoiseParams& p = ParamsOf(channel);
  const double walk = p.bias_walk_sigma * sqrt_dt_;
  auto& bias = bias_[channel];
  for (std::size_t a = 0; a < kAxes; ++a) {
    xyz[a] += bias[a] + p.white_sigma * unit_(rng_);
    bias[a] += walk * unit_(rng_);
  }
}

std::array<double, ImuNoiseModel::kAxes> ImuNoiseModel::Bias(std::size_t channel) const noexcept {
  return channel < bias_.size() ? bias_[channel] : std::array<double, kAxes>{};
}

const ImuNoiseParams& ImuNoiseModel::ParamsOf(std::size_t channel) const noexcept {
  return kinds_[channel] == ImuNoiseKind::kGyro ? config_.gyro : config_.accel;
}

}  // namespace rtc
