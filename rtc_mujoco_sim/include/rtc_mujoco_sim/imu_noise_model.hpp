// ── imu_noise_model.hpp ──────────────────────────────────────────────────────
// White noise + bias + bias random walk for the simulator's IMU export.
//
// MuJoCo does not add an MJCF sensor's noise= to its output (it is metadata
// only), so a simulated gyro and accelerometer are exact unless the producer
// adds the noise itself. This is that model, kept free of physics: it is given
// a sensor kind, the sample period and the values, and nothing else, so its
// statistics and its determinism are testable without stepping a model.
// ──────────────────────────────────────────────────────────────────────────────
#ifndef RTC_MUJOCO_SIM_IMU_NOISE_MODEL_HPP_
#define RTC_MUJOCO_SIM_IMU_NOISE_MODEL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace rtc {

/// Per-axis noise of one IMU sensor kind, in that sensor's units (rad/s for a
/// gyro, m/s² for an accelerometer).
struct ImuNoiseParams {
  double white_sigma{0.0};      ///< σ_w: n ~ N(0, σ_w²), independent every sample
  double bias_sigma{0.0};       ///< σ_b: b ~ N(0, σ_b²), drawn at Configure and on Reset
  double bias_walk_sigma{0.0};  ///< σ_bw [unit/√s]: b += N(0, σ_bw²·dt) every sample
};

/// `robot_response.<group>.imu_noise`. Applies to every gyro and every
/// accelerometer the group exports; nothing names a sensor, a site or a robot.
struct ImuNoiseConfig {
  bool enabled{false};    ///< false = the export is the raw sensordata, bit for bit
  std::uint64_t seed{0};  ///< 0 = std::random_device (non-reproducible); else fixed
  ImuNoiseParams gyro;
  ImuNoiseParams accel;
};

/// Which parameter set a channel uses.
enum class ImuNoiseKind : std::uint8_t { kGyro, kAccel };

/// y = x + b + n per axis, for a fixed list of 3-axis channels sharing one
/// seeded RNG stream.
///
/// One Apply per channel is one sample: it adds the current bias and a fresh
/// white draw, then advances that channel's bias by one walk increment, so the
/// k-th output of a channel carries b_k and b_{k+1} − b_k ~ N(0, σ_bw²·dt).
/// The draws per Apply do not depend on which sigmas are zero, so a parameter
/// set to 0 leaves the realisation of the others unchanged.
///
/// Configure allocates; Reset and Apply do not (the RNG and the distribution
/// are members), so they may run on the simulation step.
class ImuNoiseModel {
 public:
  static constexpr std::size_t kAxes = 3;

  /// Build the channel list and draw the initial biases. A seed of 0 is
  /// resolved here, once, so a later Reset replays the same realisation. The
  /// seed is mixed with a stream tag of its own, so an IMU seed equal to
  /// another simulator RNG's seed does not replay that RNG's draws.
  /// `dt` is the sample period [s] (one Apply per channel per period).
  void Configure(const ImuNoiseConfig& config, double dt, std::vector<ImuNoiseKind> channels);

  /// Re-seed the RNG, clear the distribution's cached draw and redraw the
  /// biases: the samples that follow are the ones that followed Configure.
  void Reset() noexcept;

  /// Add the noise to `xyz[0..2]` of channel `channel`, in place, and advance
  /// its bias. An out-of-range channel is left untouched.
  void Apply(std::size_t channel, double* xyz) noexcept;

  [[nodiscard]] std::size_t NumChannels() const noexcept { return kinds_.size(); }

  /// The seed actually in use (the resolved one when configured with 0).
  [[nodiscard]] std::uint64_t Seed() const noexcept { return seed_; }

  /// Current bias of a channel (the one the next Apply adds).
  [[nodiscard]] std::array<double, kAxes> Bias(std::size_t channel) const noexcept;

 private:
  [[nodiscard]] const ImuNoiseParams& ParamsOf(std::size_t channel) const noexcept;

  ImuNoiseConfig config_{};
  double sqrt_dt_{0.0};
  std::uint64_t seed_{0};
  std::uint64_t stream_seed_{0};  // seed_ mixed with the IMU stream tag
  std::vector<ImuNoiseKind> kinds_;
  std::vector<std::array<double, kAxes>> bias_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> unit_{0.0, 1.0};
};

}  // namespace rtc

#endif  // RTC_MUJOCO_SIM_IMU_NOISE_MODEL_HPP_
