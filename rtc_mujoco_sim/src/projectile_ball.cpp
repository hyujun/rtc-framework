#include "rtc_mujoco_sim/projectile_ball.hpp"

#include <algorithm>
#include <cmath>

namespace rtc {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-12;
// Below this airspeed drag and lift are both < 1e-9 N for any ball in the
// preset range, and S = r|w|/|v| would blow up.
constexpr double kMinAirspeed = 1e-6;
// Contact duration target in physics substeps. Measured on MuJoCo 3.7.0
// (implicitfast, elliptic cone): 12 substeps keeps restitution within ~0.04 of
// target for 2-6 m/s impacts at 0.67-2 ms substeps; 8 already biases low, and
// 3-4 overshoots past 1.0.
constexpr double kContactSubsteps = 12.0;

// Presets. Values and their sources are tabulated in the rtc_mujoco_sim README
// (Projectile Ball -> ball_type); keep the two in sync. Restitution, friction
// and rolling resistance are for a rigid floor; the rolling and torsional
// ratios are estimates (no measurements found).
constexpr ProjectileBallPhysics kTennisPhysics{
    0.55,    // inertia_ratio: Cross 2003 (6 mm rubber shell + felt)
    0.75,    // restitution: ITF drop test 0.745 +- 2.3 %
    0.6,     // sliding_friction: Cross 2003, hard court 0.6-0.75
    0.05,    // torsional_friction_ratio: estimate
    0.02,    // rolling_friction_ratio: estimate
    0.55,    // drag_coefficient: 0.51 (free flight) - 0.65 (wind tunnel)
    0.981,   // lift_a: Stepanek 1988, C_L = 1 / (2.022 + 0.981 / S)
    2.022};  // lift_b
constexpr ProjectileBallPhysics kBeanbagPhysics{
    0.4,   // inertia_ratio: filled; placeholder (the fill shifts)
    0.1,   // restitution: estimate, the fill absorbs the impact
    0.5,   // sliding_friction: estimate, fabric on a hard floor
    0.3,   // torsional_friction_ratio: estimate, flattens on contact
    0.5,   // rolling_friction_ratio: estimate, barely rolls
    0.5,   // drag_coefficient: estimate, rough sphere
    0.0,   // lift_a: none — a floppy bag sustains no useful spin
    0.0};  // lift_b
constexpr ProjectileBallPhysics kHardPhysics{
    0.378,  // inertia_ratio: Brody (baseball)
    0.55,   // restitution: ASTM F1887 0.546 at 27 m/s; higher at low speed (estimate)
    0.5,    // sliding_friction: Sawicki 2003 (leather, ball-bat)
    0.02,   // torsional_friction_ratio: estimate
    0.005,  // rolling_friction_ratio: estimate
    0.5,    // drag_coefficient: Cross / Kensrud, subcritical Re
    0.667,  // lift_a: fitted to Sawicki 2003 bilinear (slope 1.5, 0.27 at S = 0.3)
    1.48};  // lift_b

[[nodiscard]] bool IsFinite(const std::array<double, 3>& values) noexcept {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool IsNonNegative(const std::array<double, 3>& values) noexcept {
  return std::all_of(values.begin(), values.end(), [](double value) { return value >= 0.0; });
}

[[nodiscard]] double Norm(const std::array<double, 3>& values) noexcept {
  return std::sqrt(values[0] * values[0] + values[1] * values[1] + values[2] * values[2]);
}

[[nodiscard]] std::array<double, 3> Cross(const std::array<double, 3>& a,
                                          const std::array<double, 3>& b) noexcept {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

// A zero variation returns the mean without consuming a draw — a zero-stddev
// normal distribution is undefined.
[[nodiscard]] double Draw(ProjectileBallNoise noise, double mean, double variation,
                          std::mt19937_64& rng) noexcept {
  if (variation <= 0.0) {
    return mean;
  }
  if (noise == ProjectileBallNoise::kNormal) {
    return std::normal_distribution<double>(mean, variation)(rng);
  }
  return std::uniform_real_distribution<double>(mean - variation, mean + variation)(rng);
}

// Rebound speed ratio of x'' = max(-2 zeta x' - x, 0) entering at x = 0 with
// x' = -1 (unit natural frequency, x < 0 is penetration).
[[nodiscard]] double ClampedSpringDamperRestitution(double zeta) noexcept {
  constexpr int kStepsPerRadian = 1000;
  constexpr int kMaxSteps = 100 * kStepsPerRadian;
  const double h = 1.0 / kStepsPerRadian;
  double x = 0.0;
  double v = -1.0;
  for (int step = 0; step < kMaxSteps; ++step) {
    const double a = std::max(-2.0 * zeta * v - x, 0.0);
    v += h * a;
    x += h * v;
    if (x >= 0.0) {
      return std::max(v, 0.0);
    }
  }
  return 0.0;
}

}  // namespace

const ProjectileBallPhysics& GetProjectileBallPhysics(ProjectileBallType type) noexcept {
  switch (type) {
    case ProjectileBallType::kBeanbag:
      return kBeanbagPhysics;
    case ProjectileBallType::kHard:
      return kHardPhysics;
    case ProjectileBallType::kTennis:
    default:
      return kTennisPhysics;
  }
}

bool ParseProjectileBallType(std::string_view name, ProjectileBallType& type) noexcept {
  if (name == "tennis") {
    type = ProjectileBallType::kTennis;
  } else if (name == "beanbag") {
    type = ProjectileBallType::kBeanbag;
  } else if (name == "hard") {
    type = ProjectileBallType::kHard;
  } else {
    return false;
  }
  return true;
}

bool ParseProjectileBallNoise(std::string_view name, ProjectileBallNoise& noise) noexcept {
  if (name == "uniform") {
    noise = ProjectileBallNoise::kUniform;
  } else if (name == "normal") {
    noise = ProjectileBallNoise::kNormal;
  } else {
    return false;
  }
  return true;
}

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
  if (config.collision_contype <= 0 || config.collision_conaffinity <= 0) {
    error = "projectile_ball collision filters must be > 0";
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
      config.launch_angle_variation_deg < 0.0 ||
      !std::isfinite(config.launch_azimuth_variation_deg) ||
      config.launch_azimuth_variation_deg < 0.0) {
    error = "projectile_ball launch angle values are invalid";
    return false;
  }
  if (!std::isfinite(config.launch_speed_m_s) || config.launch_speed_m_s < 0.0 ||
      !std::isfinite(config.launch_speed_variation_m_s) ||
      config.launch_speed_variation_m_s < 0.0) {
    error = "projectile_ball launch speed values are invalid";
    return false;
  }
  if (!IsFinite(config.launch_spin_rad_s) || !IsFinite(config.launch_spin_variation_rad_s) ||
      !IsNonNegative(config.launch_spin_variation_rad_s)) {
    error = "projectile_ball launch spin values are invalid";
    return false;
  }
  return true;
}

double ProjectileBallDampingRatioForRestitution(double restitution) noexcept {
  const double target = std::clamp(restitution, 1e-3, 0.999);
  // Restitution falls monotonically with zeta; by zeta = 5 the push-only
  // contact returns < 1e-3 of the impact speed.
  double low = 0.0;
  double high = 5.0;
  for (int iteration = 0; iteration < 50; ++iteration) {
    const double mid = 0.5 * (low + high);
    if (ClampedSpringDamperRestitution(mid) > target) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return 0.5 * (low + high);
}

ProjectileBallContact ComputeProjectileBallContact(const ProjectileBallConfig& config,
                                                   double substep_s) noexcept {
  const ProjectileBallPhysics& physics = GetProjectileBallPhysics(config.type);
  // Half an oscillation period (pi / sqrt(k)) spans kContactSubsteps substeps.
  const double natural_frequency = kPi / (kContactSubsteps * substep_s);
  const double zeta = ProjectileBallDampingRatioForRestitution(physics.restitution);

  ProjectileBallContact contact;
  contact.stiffness = natural_frequency * natural_frequency;
  contact.damping = 2.0 * zeta * natural_frequency;
  contact.friction = {physics.sliding_friction, physics.torsional_friction_ratio * config.radius_m,
                      physics.rolling_friction_ratio * config.radius_m};
  return contact;
}

std::array<double, 3> ComputeProjectileBallAeroForce(
    const ProjectileBallPhysics& physics, double radius_m, double air_density_kg_m3,
    const std::array<double, 3>& velocity_m_s,
    const std::array<double, 3>& angular_velocity_rad_s) noexcept {
  std::array<double, 3> force{0.0, 0.0, 0.0};
  const double speed = Norm(velocity_m_s);
  if (!(speed > kMinAirspeed)) {
    return force;
  }
  const double area = kPi * radius_m * radius_m;
  const double dynamic_pressure_area = 0.5 * air_density_kg_m3 * area;

  // Drag: -1/2 rho Cd A |v| v.
  const double drag_scale = -dynamic_pressure_area * physics.drag_coefficient * speed;
  for (std::size_t i = 0; i < 3; ++i) {
    force[i] = drag_scale * velocity_m_s[i];
  }

  // Magnus lift along w x v with magnitude 1/2 rho C_L(S) A |v|^2. Only the
  // spin component perpendicular to v generates lift: |w x v| = |w_perp| |v|.
  if (physics.lift_a <= 0.0) {
    return force;
  }
  const std::array<double, 3> lift_axis = Cross(angular_velocity_rad_s, velocity_m_s);
  const double lift_axis_norm = Norm(lift_axis);
  if (!(lift_axis_norm > kEpsilon)) {
    return force;
  }
  const double spin_parameter = radius_m * lift_axis_norm / (speed * speed);
  const double lift_coefficient =
      spin_parameter / (physics.lift_a + physics.lift_b * spin_parameter);
  const double lift_scale =
      dynamic_pressure_area * lift_coefficient * speed * speed / lift_axis_norm;
  for (std::size_t i = 0; i < 3; ++i) {
    force[i] += lift_scale * lift_axis[i];
  }
  return force;
}

ProjectileBallLaunchSample SampleProjectileBallLaunch(const ProjectileBallConfig& config,
                                                      std::mt19937_64& rng) noexcept {
  const ProjectileBallNoise noise = config.launch_noise;
  const double angle_deg =
      Draw(noise, config.launch_angle_deg, config.launch_angle_variation_deg, rng);
  const double speed_m_s =
      std::max(0.0, Draw(noise, config.launch_speed_m_s, config.launch_speed_variation_m_s, rng));
  const double azimuth_deg = Draw(noise, 0.0, config.launch_azimuth_variation_deg, rng);
  std::array<double, 3> spin_launch{0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < 3; ++i) {
    spin_launch[i] =
        Draw(noise, config.launch_spin_rad_s[i], config.launch_spin_variation_rad_s[i], rng);
  }

  // Launch frame: forward = launch_direction rotated by the azimuth draw,
  // left = up x forward, up = world z.
  const double direction_norm = Norm(config.launch_direction);
  const double azimuth_rad = azimuth_deg * kPi / 180.0;
  const double cos_azimuth = std::cos(azimuth_rad);
  const double sin_azimuth = std::sin(azimuth_rad);
  const double base_x = config.launch_direction[0] / direction_norm;
  const double base_y = config.launch_direction[1] / direction_norm;
  const std::array<double, 3> forward = {cos_azimuth * base_x - sin_azimuth * base_y,
                                         sin_azimuth * base_x + cos_azimuth * base_y, 0.0};
  const std::array<double, 3> left = {-forward[1], forward[0], 0.0};

  const double angle_rad = angle_deg * kPi / 180.0;
  const double horizontal_component = std::cos(angle_rad) * speed_m_s;
  const double vertical_component = std::sin(angle_rad) * speed_m_s;

  ProjectileBallLaunchSample sample;
  sample.angle_deg = angle_deg;
  sample.azimuth_deg = azimuth_deg;
  sample.speed_m_s = speed_m_s;
  for (std::size_t i = 0; i < 3; ++i) {
    sample.linear_velocity_m_s[i] = horizontal_component * forward[i];
    sample.angular_velocity_rad_s[i] = spin_launch[0] * forward[i] + spin_launch[1] * left[i];
  }
  sample.linear_velocity_m_s[2] += vertical_component;
  sample.angular_velocity_rad_s[2] += spin_launch[2];
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
