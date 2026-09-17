#ifndef RTC_MUJOCO_SIM_PROJECTILE_BALL_HPP_
#define RTC_MUJOCO_SIM_PROJECTILE_BALL_HPP_

#include <array>
#include <cstdint>
#include <random>
#include <string>

namespace rtc {

struct ProjectileBallConfig {
  bool enabled{false};
  std::string body_name{"projectile_ball"};
  double radius_m{0.025};
  double mass_kg{0.05};
  int collision_contype{2};
  int collision_conaffinity{1};
  std::array<double, 3> friction{1.0, 0.5, 0.01};
  std::array<double, 3> spawn_position_m{0.0, 0.0, 0.5};
  std::array<double, 3> park_position_m{0.0, 0.0, -50.0};
  std::array<double, 3> launch_direction{1.0, 0.0, 0.0};
  double launch_angle_deg{0.0};
  double launch_angle_variation_deg{0.0};
  double launch_speed_m_s{1.0};
  double launch_speed_variation_m_s{0.0};
  std::uint64_t seed{0};
};

struct ProjectileBallLaunchSample {
  double angle_deg{0.0};
  double speed_m_s{0.0};
  std::array<double, 3> linear_velocity_m_s{0.0, 0.0, 0.0};
};

[[nodiscard]] bool ValidateProjectileBallConfig(const ProjectileBallConfig& config,
                                                std::string& error) noexcept;

[[nodiscard]] ProjectileBallLaunchSample SampleProjectileBallLaunch(
    const ProjectileBallConfig& config, std::mt19937_64& rng) noexcept;

/// Sim-time publish throttle. Returns true (and records sim_time_sec) when at
/// least period_sec has elapsed since the last accepted sample; a negative
/// last_publish_time_sec means "nothing published yet". A sim_time_sec earlier
/// than the recorded stamp (sim reset) re-opens the gate immediately.
[[nodiscard]] bool ShouldPublishProjectileBallSample(double sim_time_sec, double period_sec,
                                                     double& last_publish_time_sec) noexcept;

}  // namespace rtc

#endif  // RTC_MUJOCO_SIM_PROJECTILE_BALL_HPP_