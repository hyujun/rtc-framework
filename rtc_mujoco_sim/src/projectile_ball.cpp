#include "rtc_mujoco_sim/projectile_ball.hpp"

#include <algorithm>
#include <cmath>

namespace rtc {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-12;

[[nodiscard]] bool IsFinite(const std::array<double, 3>& values) noexcept {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] double Norm(const std::array<double, 3>& values) noexcept {
  return std::sqrt(values[0] * values[0] + values[1] * values[1] + values[2] * values[2]);
}

}  // namespace

bool ValidateProjectileBallConfig(const ProjectileBallConfig& config, std::string& error) noexcept {
  if (config.body_name.empty()) {
    error = "projectile_ball.body_name must not be empty";
    return false;
  }
  if (!std::isfinite(config.radius_m) || config.radius_m <= 0.0) {
    error = "projectile_ball.radius_m must be finite and > 0";
    return false;
  }
  if (!std::isfinite(config.mass_kg) || config.mass_kg <= 0.0) {
    error = "projectile_ball.mass_kg must be finite and > 0";
    return false;
  }
  if (config.collision_contype <= 0 || config.collision_conaffinity <= 0 ||
      !IsFinite(config.friction) || config.friction[0] < 0.0 || config.friction[1] < 0.0 ||
      config.friction[2] < 0.0) {
    error = "projectile_ball collision filters and friction are invalid";
    return false;
  }
  if (!IsFinite(config.spawn_position_m) || !IsFinite(config.park_position_m) ||
      !IsFinite(config.launch_direction)) {
    error = "projectile_ball positions and launch_direction must be finite";
    return false;
  }
  if (Norm(config.launch_direction) <= kEpsilon) {
    error = "projectile_ball.launch_direction must not be zero";
    return false;
  }
  // Elevation comes from launch_angle_deg alone; a tilted direction would add
  // a second, silent elevation on top of it.
  if (std::abs(config.launch_direction[2]) > kEpsilon) {
    error = "projectile_ball.launch_direction must be horizontal (z = 0); use launch_angle_deg";
    return false;
  }
  if (!std::isfinite(config.launch_angle_deg) ||
      !std::isfinite(config.launch_angle_variation_deg) ||
      config.launch_angle_variation_deg < 0.0) {
    error = "projectile_ball launch angle values are invalid";
    return false;
  }
  if (!std::isfinite(config.launch_speed_m_s) || config.launch_speed_m_s < 0.0 ||
      !std::isfinite(config.launch_speed_variation_m_s) ||
      config.launch_speed_variation_m_s < 0.0) {
    error = "projectile_ball launch speed values are invalid";
    return false;
  }
  return true;
}

ProjectileBallLaunchSample SampleProjectileBallLaunch(const ProjectileBallConfig& config,
                                                      std::mt19937_64& rng) noexcept {
  std::uniform_real_distribution<double> angle_distribution(
      config.launch_angle_deg - config.launch_angle_variation_deg,
      config.launch_angle_deg + config.launch_angle_variation_deg);
  std::uniform_real_distribution<double> speed_distribution(
      config.launch_speed_m_s - config.launch_speed_variation_m_s,
      config.launch_speed_m_s + config.launch_speed_variation_m_s);

  const double angle_deg = angle_distribution(rng);
  const double speed_m_s = std::max(0.0, speed_distribution(rng));
  const double direction_norm = Norm(config.launch_direction);
  const std::array<double, 3> direction = {config.launch_direction[0] / direction_norm,
                                           config.launch_direction[1] / direction_norm,
                                           config.launch_direction[2] / direction_norm};
  const double angle_rad = angle_deg * kPi / 180.0;
  const double horizontal_scale = std::cos(angle_rad);
  const double vertical_component = std::sin(angle_rad) * speed_m_s;
  const double horizontal_component = horizontal_scale * speed_m_s;

  ProjectileBallLaunchSample sample;
  sample.angle_deg = angle_deg;
  sample.speed_m_s = speed_m_s;
  sample.linear_velocity_m_s = {horizontal_component * direction[0],
                                horizontal_component * direction[1],
                                horizontal_component * direction[2] + vertical_component};
  return sample;
}

bool ShouldPublishProjectileBallSample(double sim_time_sec, double period_sec,
                                       double& last_publish_time_sec) noexcept {
  // A sim reset rewinds mjData::time to 0. Without this, the gate would stay
  // closed until sim time caught up with the pre-reset stamp.
  if (last_publish_time_sec >= 0.0 && sim_time_sec < last_publish_time_sec) {
    last_publish_time_sec = -1.0;
  }
  if (last_publish_time_sec >= 0.0 &&
      sim_time_sec - last_publish_time_sec + kEpsilon < period_sec) {
    return false;
  }
  last_publish_time_sec = sim_time_sec;
  return true;
}

}  // namespace rtc
