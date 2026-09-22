#ifndef RTC_MUJOCO_SIM_PROJECTILE_BALL_HPP_
#define RTC_MUJOCO_SIM_PROJECTILE_BALL_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <random>
#include <string>
#include <string_view>

namespace rtc {

/// Physical ball presets. The YAML fixes only radius and mass; every other
/// physical quantity (inertia distribution, restitution, friction, drag, lift)
/// comes from the preset table in projectile_ball.cpp.
enum class ProjectileBallType { kTennis, kBeanbag, kHard };

/// Distribution the launch variations are drawn from. For kUniform a
/// `*_variation` value is the half-width; for kNormal it is the standard
/// deviation.
enum class ProjectileBallNoise { kUniform, kNormal };

/// Dimensionless preset — scaled by the configured radius and mass at startup.
struct ProjectileBallPhysics {
  double inertia_ratio{0.4};             ///< I / (m r^2): 0.4 solid, 2/3 thin shell
  double restitution{0.5};               ///< normal coefficient of restitution on a rigid surface
  double sliding_friction{0.5};          ///< Coulomb mu
  double torsional_friction_ratio{0.0};  ///< MuJoCo torsional friction [m] / r
  double rolling_friction_ratio{0.0};    ///< MuJoCo rolling friction [m] / r (rolling resistance)
  double drag_coefficient{0.47};         ///< Cd, constant (subcritical Re at throwing speeds)
  /// Lift C_L(S) = S / (lift_a + lift_b * S) with spin parameter S = r|w_perp| / |v|.
  /// Small-S slope is 1 / lift_a, saturation 1 / lift_b. lift_a <= 0 disables lift.
  double lift_a{0.0};
  double lift_b{0.0};
};

struct ProjectileBallConfig {
  bool enabled{false};
  std::string body_name{"projectile_ball"};
  ProjectileBallType type{ProjectileBallType::kTennis};
  double radius_m{0.0335};
  double mass_kg{0.057};
  int collision_contype{2};
  int collision_conaffinity{1};
  /// Quadratic drag + Magnus lift on the ball only (robot links are untouched).
  bool aerodynamics_enabled{false};
  std::array<double, 3> spawn_position_m{0.0, 0.0, 0.5};
  std::array<double, 3> park_position_m{0.0, 0.0, -50.0};
  std::array<double, 3> launch_direction{1.0, 0.0, 0.0};
  ProjectileBallNoise launch_noise{ProjectileBallNoise::kUniform};
  double launch_angle_deg{0.0};
  double launch_angle_variation_deg{0.0};
  /// Rotation of launch_direction about world +z.
  double launch_azimuth_variation_deg{0.0};
  double launch_speed_m_s{1.0};
  double launch_speed_variation_m_s{0.0};
  /// Initial angular velocity in the launch frame: x = sampled horizontal launch
  /// direction, z = world up, y = z x x (left). Backspin (lift up) is negative y.
  std::array<double, 3> launch_spin_rad_s{0.0, 0.0, 0.0};
  std::array<double, 3> launch_spin_variation_rad_s{0.0, 0.0, 0.0};
  std::uint64_t seed{0};
};

/// A launch state the CALLER named, as opposed to one the simulator drew from
/// the configured distribution. Trivially copyable on purpose: it is handed to
/// the physics thread through a SeqLock, which requires that.
///
/// Every field is world-frame. Body and world axes coincide at release because
/// the writer resets the ball's orientation to identity, so `angular_velocity`
/// needs no frame qualifier at the instant it is applied.
struct ProjectileBallLaunchCommand {
  std::array<double, 3> position_m{0.0, 0.0, 0.0};
  std::array<double, 3> linear_velocity_m_s{0.0, 0.0, 0.0};
  std::array<double, 3> angular_velocity_rad_s{0.0, 0.0, 0.0};
};

struct ProjectileBallLaunchSample {
  double angle_deg{0.0};
  double azimuth_deg{0.0};
  double speed_m_s{0.0};
  std::array<double, 3> linear_velocity_m_s{0.0, 0.0, 0.0};
  std::array<double, 3> angular_velocity_rad_s{0.0, 0.0, 0.0};  ///< world frame
};

/// Contact parameters resolved from the preset, the radius and the physics
/// substep. Written onto the ball geom, which outranks every scene geom by
/// priority so the same bounce holds against the floor and the hand.
struct ProjectileBallContact {
  double stiffness{0.0};  ///< solref[0] = -stiffness (direct form)
  double damping{0.0};    ///< solref[1] = -damping
  std::array<double, 5> solimp{0.99, 0.99, 0.001, 0.5, 2.0};
  std::array<double, 3> friction{0.0, 0.0, 0.0};  ///< sliding, torsional [m], rolling [m]
};

/// Air density at 20 degC, sea level [kg/m^3].
inline constexpr double kProjectileBallAirDensity = 1.204;

[[nodiscard]] const ProjectileBallPhysics& GetProjectileBallPhysics(
    ProjectileBallType type) noexcept;

/// "tennis" | "beanbag" | "hard". Returns false on an unknown name.
[[nodiscard]] bool ParseProjectileBallType(std::string_view name,
                                           ProjectileBallType& type) noexcept;

/// "uniform" | "normal". Returns false on an unknown name.
[[nodiscard]] bool ParseProjectileBallNoise(std::string_view name,
                                            ProjectileBallNoise& noise) noexcept;

[[nodiscard]] bool ValidateProjectileBallConfig(const ProjectileBallConfig& config,
                                                std::string& error) noexcept;

/// Damping ratio of a force-clamped (push-only) linear spring-damper whose
/// rebound speed ratio equals `restitution` in (0, 1). Push-only matters: a
/// heavily damped contact lets go while still compressed, skipping the pulling
/// phase that would have dissipated energy, so the textbook
/// exp(-zeta pi / sqrt(1 - zeta^2)) relation under-predicts the needed damping.
[[nodiscard]] double ProjectileBallDampingRatioForRestitution(double restitution) noexcept;

/// Contact stiffness is chosen so a contact lasts ~12 physics substeps; MuJoCo's
/// discrete contact overshoots the target restitution once it spans fewer.
[[nodiscard]] ProjectileBallContact ComputeProjectileBallContact(const ProjectileBallConfig& config,
                                                                 double substep_s) noexcept;

/// World-frame aerodynamic force [N] on the ball centre: quadratic drag plus
/// Magnus lift. Zero below a negligible airspeed.
[[nodiscard]] std::array<double, 3> ComputeProjectileBallAeroForce(
    const ProjectileBallPhysics& physics, double radius_m, double air_density_kg_m3,
    const std::array<double, 3>& velocity_m_s,
    const std::array<double, 3>& angular_velocity_rad_s) noexcept;

[[nodiscard]] ProjectileBallLaunchSample SampleProjectileBallLaunch(
    const ProjectileBallConfig& config, std::mt19937_64& rng) noexcept;

/// Reject a launch state the physics cannot act on. Only finiteness is checked:
/// a zero velocity is a drop, a zero spin is a spinless throw, and a position
/// below the floor is a caller's business — none of those are errors. NaN and
/// infinity are, because they would reach mjData::qpos/qvel and poison the
/// whole scene rather than just the ball.
[[nodiscard]] bool ValidateProjectileBallLaunchCommand(const ProjectileBallLaunchCommand& command,
                                                       std::string& error) noexcept;

/// Sim-time publish throttle. Returns true (and records sim_time_sec) when at
/// least period_sec has elapsed since the last accepted sample; a negative
/// last_publish_time_sec means "nothing published yet". A sim_time_sec earlier
/// than the recorded stamp (sim reset) re-opens the gate immediately.
[[nodiscard]] bool ShouldPublishProjectileBallSample(double sim_time_sec, double period_sec,
                                                     double& last_publish_time_sec) noexcept;

/// steady_clock now as ns since its epoch — the axis the RT path and the D-3
/// clock lane use.
[[nodiscard]] inline std::int64_t SteadyNowNs() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

/// Steady instant [ns] to stamp a ball sample with: the sim-time axis laid
/// onto the wall from the launch instant,
/// `anchor_wall_ns + (sim_time_sec − anchor_sim_sec) / rtf`. Consecutive
/// samples spaced evenly in sim time get evenly spaced stamps whatever
/// burst-and-sleep rhythm the stepper ran with. The stamp differs from the
/// wall by the phase error the flight accumulates after launch — behind it
/// while the stepper stalls, ahead of it while it catches up — which is the
/// quantity the D-3 clock lane measures, in both signs; consumers' future-skew
/// tolerances must cover the lead in sim. (Clamping the stamp to the wall was
/// tried and rejected: it re-introduces the wake jitter every time the sim
/// runs ahead of its launch pace.) Anchoring at launch keeps any sim-vs-wall
/// deficit accumulated BEFORE the launch (idle stepping without a controller,
/// a pause) out of the flight's stamps. Unthrottled (`rtf` ≤ 0), or when the
/// mapping is not representable, `actual_steady_ns` is returned unchanged.
[[nodiscard]] std::int64_t ProjectileBallStampSteadyNs(double sim_time_sec, double anchor_sim_sec,
                                                       std::int64_t anchor_wall_ns, double rtf,
                                                       std::int64_t actual_steady_ns) noexcept;

}  // namespace rtc

#endif  // RTC_MUJOCO_SIM_PROJECTILE_BALL_HPP_
