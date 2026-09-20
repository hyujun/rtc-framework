#include "rtc_mujoco_sim/object_pool.hpp"
#include "rtc_mujoco_sim/projectile_ball.hpp"
#include "test_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifndef POOL_SCENE_MJCF_PATH
#error "POOL_SCENE_MJCF_PATH must be defined by CMake"
#endif
#ifndef BALL_SCENE_MJCF_PATH
#error "BALL_SCENE_MJCF_PATH must be defined by CMake"
#endif
#ifndef BALL_SCENE_HIGH_PRIORITY_MJCF_PATH
#error "BALL_SCENE_HIGH_PRIORITY_MJCF_PATH must be defined by CMake"
#endif
#ifndef BALL_POOL_OBJECTS_DIR
#error "BALL_POOL_OBJECTS_DIR must be defined by CMake"
#endif

using namespace std::chrono_literals;

namespace rtc {
namespace {

struct BallIds {
  int body{-1};
  int geom{-1};
  int qpos{-1};
  int dof{-1};
};

// Resolve the generated ball from the compiled model rather than assuming it
// is the last body / last six dofs.
BallIds FindBall(const mjModel* model) {
  BallIds ids;
  ids.body = mj_name2id(model, mjOBJ_BODY, "projectile_ball");
  if (ids.body < 0) {
    return ids;
  }
  for (int g = 0; g < model->ngeom; ++g) {
    if (model->geom_bodyid[g] == ids.body) {
      ids.geom = g;
    }
  }
  for (int j = 0; j < model->njnt; ++j) {
    if (model->jnt_bodyid[j] == ids.body && model->jnt_type[j] == mjJNT_FREE) {
      ids.qpos = model->jnt_qposadr[j];
      ids.dof = model->jnt_dofadr[j];
    }
  }
  return ids;
}

int CountBallContacts(const mjData* data, int ball_geom) {
  int count = 0;
  for (int i = 0; i < data->ncon; ++i) {
    if (data->contact[i].geom[0] == ball_geom || data->contact[i].geom[1] == ball_geom) {
      ++count;
    }
  }
  return count;
}

// Fixture with a floor plane (default contact filters). The plane is what makes parking under the
// scene dangerous: MuJoCo treats it as a halfspace, so a parked ball with its
// contact filters enabled is ejected.
MuJoCoSimulator::Config MakeFloorSceneConfigWithBall() {
  MuJoCoSimulator::Config config;
  config.model_path = POOL_SCENE_MJCF_PATH;
  config.enable_viewer = false;
  config.n_substeps = 1;
  JointGroupConfig group;
  group.name = "arm";
  group.command_joint_names = {"j1", "j2"};
  group.state_joint_names = {"j1", "j2"};
  group.command_topic = "/arm/cmd";
  group.state_topic = "/arm/state";
  group.is_robot = true;
  config.groups.push_back(group);
  config.projectile_ball.enabled = true;
  config.projectile_ball.park_position_m = {0.0, 0.0, -5.0};
  config.projectile_ball.launch_direction = {1.0, 0.0, 0.0};
  config.projectile_ball.launch_angle_deg = 0.0;
  config.projectile_ball.seed = 7;
  return config;
}

TEST(ProjectileBall, RejectsInvalidConfiguration) {
  ProjectileBallConfig config;
  std::string error;

  config.radius_m = 0.0;
  EXPECT_FALSE(ValidateProjectileBallConfig(config, error));
  EXPECT_NE(error.find("radius_m"), std::string::npos);

  config.radius_m = 0.025;
  config.launch_direction = {0.0, 0.0, 0.0};
  EXPECT_FALSE(ValidateProjectileBallConfig(config, error));
  EXPECT_NE(error.find("launch_direction"), std::string::npos);

  // Elevation belongs to launch_angle_deg; a tilted direction is rejected.
  error.clear();
  config.launch_direction = {1.0, 0.0, 0.5};
  EXPECT_FALSE(ValidateProjectileBallConfig(config, error));
  EXPECT_NE(error.find("horizontal"), std::string::npos);

  config.launch_direction = {0.0, 1.0, 0.0};
  EXPECT_TRUE(ValidateProjectileBallConfig(config, error));

  config.launch_spin_variation_rad_s = {0.0, -1.0, 0.0};
  EXPECT_FALSE(ValidateProjectileBallConfig(config, error));
  EXPECT_NE(error.find("spin"), std::string::npos);
}

TEST(ProjectileBall, PublishThrottleReopensAfterSimTimeRewind) {
  double last = -1.0;
  const double period = 0.01;
  EXPECT_TRUE(ShouldPublishProjectileBallSample(5.000, period, last));
  EXPECT_FALSE(ShouldPublishProjectileBallSample(5.004, period, last));
  EXPECT_TRUE(ShouldPublishProjectileBallSample(5.010, period, last));
  // Sim reset: mjData::time restarts at 0. The gate must open immediately
  // instead of waiting for sim time to pass 5.01 again.
  EXPECT_TRUE(ShouldPublishProjectileBallSample(0.002, period, last));
  EXPECT_DOUBLE_EQ(last, 0.002);
  EXPECT_FALSE(ShouldPublishProjectileBallSample(0.006, period, last));
}

TEST(ProjectileBall, ComputesElevationFromConfiguredDirection) {
  ProjectileBallConfig config;
  config.launch_direction = {1.0, 0.0, 0.0};
  config.launch_angle_deg = 45.0;
  config.launch_speed_m_s = 10.0;
  config.launch_speed_variation_m_s = 0.0;
  config.launch_angle_variation_deg = 0.0;

  std::mt19937_64 rng(42);
  const auto sample = SampleProjectileBallLaunch(config, rng);
  EXPECT_NEAR(sample.linear_velocity_m_s[0], std::sqrt(50.0), 1e-12);
  EXPECT_NEAR(sample.linear_velocity_m_s[1], 0.0, 1e-12);
  EXPECT_NEAR(sample.linear_velocity_m_s[2], std::sqrt(50.0), 1e-12);
}

TEST(ProjectileBall, SamplingIsReproducibleAndBounded) {
  ProjectileBallConfig config;
  config.launch_angle_deg = 30.0;
  config.launch_angle_variation_deg = 5.0;
  config.launch_speed_m_s = 4.0;
  config.launch_speed_variation_m_s = 0.5;

  std::mt19937_64 first_rng(1234);
  std::mt19937_64 second_rng(1234);
  for (int i = 0; i < 20; ++i) {
    const auto first = SampleProjectileBallLaunch(config, first_rng);
    const auto second = SampleProjectileBallLaunch(config, second_rng);
    EXPECT_DOUBLE_EQ(first.angle_deg, second.angle_deg);
    EXPECT_DOUBLE_EQ(first.speed_m_s, second.speed_m_s);
    EXPECT_GE(first.angle_deg, 25.0);
    EXPECT_LE(first.angle_deg, 35.0);
    EXPECT_GE(first.speed_m_s, 3.5);
    EXPECT_LE(first.speed_m_s, 4.5);
  }
}

TEST(ProjectileBall, CompilesAndLaunchesGeneratedFreeBody) {
  auto config = test::MakeMinimalConfig();
  config.projectile_ball.enabled = true;
  config.projectile_ball.spawn_position_m = {0.0, 0.0, 0.5};
  config.projectile_ball.launch_direction = {1.0, 0.0, 0.0};
  config.projectile_ball.launch_angle_deg = 0.0;
  config.projectile_ball.launch_speed_m_s = 2.0;
  config.projectile_ball.seed = 7;

  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());
  ASSERT_TRUE(sim.HasProjectileBall());
  const auto* model = sim.GetModel();
  ASSERT_NE(model, nullptr);
  const int ball_body_id = mj_name2id(model, mjOBJ_BODY, "projectile_ball");
  ASSERT_GE(ball_body_id, 0);
  int ball_geom_id = -1;
  for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
    if (model->geom_bodyid[geom_id] == ball_body_id) {
      ball_geom_id = geom_id;
    }
  }
  ASSERT_GE(ball_geom_id, 0);
  // Parked at startup: contacts are enabled only by a launch.
  EXPECT_EQ(model->geom_contype[ball_geom_id], 0);
  EXPECT_EQ(model->geom_conaffinity[ball_geom_id], 0);

  const BallIds ball = FindBall(model);
  ASSERT_GE(ball.dof, 0);

  sim.RequestProjectileBallLaunch();
  sim.StepForTest();
  const auto* data = sim.GetData();
  ASSERT_NE(data, nullptr);
  EXPECT_GT(data->qvel[ball.dof], 0.0);
  EXPECT_EQ(model->body_gravcomp[ball.body], 0.0);

  sim.RequestProjectileBallReset();
  sim.StepForTest();
  EXPECT_EQ(model->geom_contype[ball.geom], 0);
  EXPECT_EQ(model->geom_conaffinity[ball.geom], 0);
  EXPECT_EQ(model->body_gravcomp[ball.body], 1.0);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(data->qvel[ball.dof + i], 0.0, 1e-9) << "axis " << i;
  }
}

TEST(ProjectileBall, StaysParkedWithoutContactAfterInitialize) {
  auto config = MakeFloorSceneConfigWithBall();
  config.projectile_ball.park_position_m = {0.0, 0.0, -50.0};
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());
  const BallIds ball = FindBall(sim.GetModel());
  ASSERT_GE(ball.geom, 0);
  ASSERT_GE(ball.qpos, 0);

  for (int step = 0; step < 200; ++step) {
    sim.StepForTest();
    ASSERT_EQ(CountBallContacts(sim.GetData(), ball.geom), 0) << "step " << step;
  }
  const auto* data = sim.GetData();
  // Neither ejected by the ground (halfspace contact) nor free-falling.
  EXPECT_NEAR(data->qpos[ball.qpos + 2], -50.0, 1e-6);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(data->qvel[ball.dof + i], 0.0, 1e-6) << "axis " << i;
  }
}

TEST(ProjectileBall, LaunchedBallContactsGround) {
  auto config = MakeFloorSceneConfigWithBall();
  // Touching the floor plane, well clear of the arm at the origin.
  config.projectile_ball.spawn_position_m = {1.0, 1.0, 0.02};
  config.projectile_ball.launch_speed_m_s = 0.0;
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());
  const BallIds ball = FindBall(sim.GetModel());
  ASSERT_GE(ball.geom, 0);

  sim.RequestProjectileBallLaunch();
  sim.StepForTest();
  EXPECT_GT(CountBallContacts(sim.GetData(), ball.geom), 0);
}

constexpr std::array<ProjectileBallType, 3> kAllBallTypes = {
    ProjectileBallType::kTennis, ProjectileBallType::kBeanbag, ProjectileBallType::kHard};
constexpr double kGravity = 9.81;

const char* TypeName(ProjectileBallType type) {
  switch (type) {
    case ProjectileBallType::kBeanbag:
      return "beanbag";
    case ProjectileBallType::kHard:
      return "hard";
    case ProjectileBallType::kTennis:
    default:
      return "tennis";
  }
}

// Ball scene at the ur5e_p1b substep (0.002 s / 3).
MuJoCoSimulator::Config MakeBallSceneConfig(ProjectileBallType type) {
  auto config = MakeFloorSceneConfigWithBall();
  config.model_path = BALL_SCENE_MJCF_PATH;
  config.n_substeps = 3;
  config.projectile_ball.type = type;
  config.projectile_ball.radius_m = 0.0335;
  config.projectile_ball.mass_kg = 0.057;
  config.projectile_ball.park_position_m = {0.0, 0.0, -50.0};
  return config;
}

double Horizontal(const mjData* data, const BallIds& ball) {
  return std::hypot(data->qvel[ball.dof], data->qvel[ball.dof + 1]);
}

// Drops the ball straight down onto the surface whose top is at surface_z and
// returns rebound / impact vertical speed across the first contact.
double MeasureRestitution(ProjectileBallType type, double x, double y, double surface_z,
                          double impact_speed_m_s) {
  auto config = MakeBallSceneConfig(type);
  config.projectile_ball.spawn_position_m = {x, y,
                                             surface_z + config.projectile_ball.radius_m + 0.01};
  config.projectile_ball.launch_angle_deg = -90.0;
  config.projectile_ball.launch_speed_m_s = impact_speed_m_s;
  MuJoCoSimulator sim(std::move(config));
  if (!sim.Initialize()) {
    ADD_FAILURE() << "Initialize failed";
    return -1.0;
  }
  const BallIds ball = FindBall(sim.GetModel());
  sim.RequestProjectileBallLaunch();
  double impact = 0.0;
  bool in_contact = false;
  for (int step = 0; step < 3000; ++step) {
    const double vz_before = sim.GetData()->qvel[ball.dof + 2];
    sim.StepForTest();
    const bool touching = CountBallContacts(sim.GetData(), ball.geom) > 0;
    if (!in_contact && touching) {
      impact = -vz_before;
      in_contact = true;
    } else if (in_contact && !touching) {
      return sim.GetData()->qvel[ball.dof + 2] / impact;
    }
  }
  // Never left the surface: no rebound.
  return in_contact ? 0.0 : -1.0;
}

TEST(ProjectileBall, ParsesTypeAndNoiseNames) {
  ProjectileBallType type = ProjectileBallType::kTennis;
  EXPECT_TRUE(ParseProjectileBallType("beanbag", type));
  EXPECT_EQ(type, ProjectileBallType::kBeanbag);
  EXPECT_TRUE(ParseProjectileBallType("hard", type));
  EXPECT_EQ(type, ProjectileBallType::kHard);
  EXPECT_TRUE(ParseProjectileBallType("tennis", type));
  EXPECT_EQ(type, ProjectileBallType::kTennis);
  EXPECT_FALSE(ParseProjectileBallType("Tennis", type));

  ProjectileBallNoise noise = ProjectileBallNoise::kUniform;
  EXPECT_TRUE(ParseProjectileBallNoise("normal", noise));
  EXPECT_EQ(noise, ProjectileBallNoise::kNormal);
  EXPECT_FALSE(ParseProjectileBallNoise("gaussian", noise));
}

// Oracle independent of the bisection: for light damping the push-only clamp
// barely binds, so the textbook underdamped relation holds; for heavy damping
// the clamp cuts off the pulling phase that would have dissipated energy, so
// more damping than the textbook value is needed.
TEST(ProjectileBall, DampingRatioMatchesSpringDamperLimits) {
  const auto textbook_zeta = [](double restitution) {
    const double log_e = -std::log(restitution);
    return log_e / std::sqrt(M_PI * M_PI + log_e * log_e);
  };
  EXPECT_NEAR(ProjectileBallDampingRatioForRestitution(0.95), textbook_zeta(0.95),
              0.05 * textbook_zeta(0.95));
  EXPECT_GT(ProjectileBallDampingRatioForRestitution(0.1), textbook_zeta(0.1));
  double previous = 0.0;
  for (const double restitution : {0.9, 0.75, 0.55, 0.3, 0.1}) {
    const double zeta = ProjectileBallDampingRatioForRestitution(restitution);
    EXPECT_GT(zeta, previous) << "restitution " << restitution;
    previous = zeta;
  }
}

TEST(ProjectileBall, ContactScalesWithRadiusAndSubstep) {
  ProjectileBallConfig config;
  config.type = ProjectileBallType::kTennis;
  config.radius_m = 0.04;
  const auto coarse = ComputeProjectileBallContact(config, 0.002);
  const auto fine = ComputeProjectileBallContact(config, 0.001);
  // Contact spans a fixed number of substeps: halving dt quadruples stiffness.
  EXPECT_NEAR(fine.stiffness / coarse.stiffness, 4.0, 1e-9);
  const auto& physics = GetProjectileBallPhysics(config.type);
  EXPECT_DOUBLE_EQ(coarse.friction[0], physics.sliding_friction);
  EXPECT_NEAR(coarse.friction[1], physics.torsional_friction_ratio * 0.04, 1e-15);
  EXPECT_NEAR(coarse.friction[2], physics.rolling_friction_ratio * 0.04, 1e-15);
}

TEST(ProjectileBall, CompilesPresetMassInertiaAndContactModel) {
  for (const auto type : kAllBallTypes) {
    SCOPED_TRACE(TypeName(type));
    MuJoCoSimulator sim(MakeBallSceneConfig(type));
    ASSERT_TRUE(sim.Initialize());
    const auto* model = sim.GetModel();
    const BallIds ball = FindBall(model);
    ASSERT_GE(ball.geom, 0);
    const double r = 0.0335;
    const double m = 0.057;
    EXPECT_NEAR(model->body_mass[ball.body], m, 1e-12);
    const double expected_inertia = GetProjectileBallPhysics(type).inertia_ratio * m * r * r;
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(model->body_inertia[3 * ball.body + i], expected_inertia, 1e-12);
      EXPECT_NEAR(model->body_ipos[3 * ball.body + i], 0.0, 1e-12);
    }
    EXPECT_EQ(model->geom_condim[ball.geom], 6);
    EXPECT_LT(model->geom_solref[mjNREF * ball.geom], 0.0) << "direct stiffness form";
    for (int g = 0; g < model->ngeom; ++g) {
      if (g != ball.geom) {
        EXPECT_GT(model->geom_priority[ball.geom], model->geom_priority[g]) << "geom " << g;
      }
    }
  }
}

TEST(ProjectileBall, RejectsSceneGeomThatOutranksTheBall) {
  auto config = MakeBallSceneConfig(ProjectileBallType::kTennis);
  config.model_path = BALL_SCENE_HIGH_PRIORITY_MJCF_PATH;
  MuJoCoSimulator sim(std::move(config));
  EXPECT_FALSE(sim.Initialize());
}

TEST(ProjectileBall, BouncesWithPresetRestitutionOnFloorAndPad) {
  for (const auto type : kAllBallTypes) {
    const double target = GetProjectileBallPhysics(type).restitution;
    for (const double speed : {2.0, 4.0}) {
      // Floor plane, and the fingertip-like pad whose own solref is critically damped.
      const double floor = MeasureRestitution(type, 0.0, 1.0, 0.0, speed);
      const double pad = MeasureRestitution(type, 2.0, 2.0, 0.30, speed);
      SCOPED_TRACE(std::string(TypeName(type)) + " @ " + std::to_string(speed) + " m/s");
      if (type == ProjectileBallType::kBeanbag) {
        EXPECT_GE(floor, 0.0);
        EXPECT_LE(floor, target + 0.05);
        EXPECT_GE(pad, 0.0);
        EXPECT_LE(pad, target + 0.05);
      } else {
        EXPECT_NEAR(floor, target, 0.05);
        EXPECT_NEAR(pad, target, 0.05);
      }
    }
  }
}

// condim 6 is what makes rolling friction act: a beanbag stops within a
// fraction of a second, a tennis ball keeps rolling but slows down.
TEST(ProjectileBall, RollingResistanceSlowsRollingBall) {
  const auto roll = [](ProjectileBallType type, double seconds_a, double seconds_b) {
    auto config = MakeBallSceneConfig(type);
    config.projectile_ball.spawn_position_m = {0.0, 1.0, config.projectile_ball.radius_m};
    config.projectile_ball.launch_speed_m_s = 1.0;
    MuJoCoSimulator sim(std::move(config));
    EXPECT_TRUE(sim.Initialize());
    const BallIds ball = FindBall(sim.GetModel());
    sim.RequestProjectileBallLaunch();
    std::array<double, 2> speeds{0.0, 0.0};
    while (sim.GetData()->time < seconds_a) {
      sim.StepForTest();
    }
    speeds[0] = Horizontal(sim.GetData(), ball);
    while (sim.GetData()->time < seconds_b) {
      sim.StepForTest();
    }
    speeds[1] = Horizontal(sim.GetData(), ball);
    return speeds;
  };
  const auto beanbag = roll(ProjectileBallType::kBeanbag, 0.2, 1.0);
  EXPECT_LT(beanbag[1], 0.02);
  const auto tennis = roll(ProjectileBallType::kTennis, 0.5, 2.0);
  EXPECT_GT(tennis[1], 0.1);
  EXPECT_LT(tennis[1], tennis[0] - 0.1);
}

TEST(ProjectileBall, AeroForceDragAndMagnusDirections) {
  const auto& tennis = GetProjectileBallPhysics(ProjectileBallType::kTennis);
  const double r = 0.0335;
  const double rho = kProjectileBallAirDensity;
  const double area = M_PI * r * r;

  // No spin: pure drag, opposite to v, 1/2 rho Cd A v^2.
  const auto drag = ComputeProjectileBallAeroForce(tennis, r, rho, {5.0, 0.0, 0.0}, {0, 0, 0});
  EXPECT_NEAR(drag[0], -0.5 * rho * tennis.drag_coefficient * area * 25.0, 1e-12);
  EXPECT_DOUBLE_EQ(drag[1], 0.0);
  EXPECT_DOUBLE_EQ(drag[2], 0.0);

  // Backspin on a ball flying +x is spin about -y; lift points up and is
  // perpendicular to v.
  const auto spin = ComputeProjectileBallAeroForce(tennis, r, rho, {5.0, 0.0, 0.0}, {0, -100, 0});
  EXPECT_NEAR(spin[0], drag[0], 1e-12);
  EXPECT_GT(spin[2], 0.0);
  const double s = r * 100.0 / 5.0;
  const double lift_coefficient = 1.0 / (2.022 + 0.981 / s);  // Stepanek form
  EXPECT_NEAR(spin[2], 0.5 * rho * lift_coefficient * area * 25.0, 1e-9);

  // Spin parallel to v gives no lift.
  const auto axial = ComputeProjectileBallAeroForce(tennis, r, rho, {5.0, 0.0, 0.0}, {100, 0, 0});
  EXPECT_NEAR(axial[2], 0.0, 1e-12);

  // At rest: nothing, and no NaN from S = r|w|/|v|.
  const auto rest = ComputeProjectileBallAeroForce(tennis, r, rho, {0, 0, 0}, {0, -100, 0});
  for (const double f : rest) {
    EXPECT_EQ(f, 0.0);
  }

  const auto& beanbag = GetProjectileBallPhysics(ProjectileBallType::kBeanbag);
  const auto no_lift =
      ComputeProjectileBallAeroForce(beanbag, r, rho, {5.0, 0.0, 0.0}, {0, -100, 0});
  EXPECT_DOUBLE_EQ(no_lift[2], 0.0);
}

// Free fall with quadratic drag: v(t) = v_t tanh(g t / v_t).
TEST(ProjectileBall, AerodynamicFreeFallMatchesAnalyticTerminalApproach) {
  for (const bool aero : {false, true}) {
    auto config = MakeBallSceneConfig(ProjectileBallType::kTennis);
    config.projectile_ball.spawn_position_m = {0.0, 1.0, 20.0};
    config.projectile_ball.launch_speed_m_s = 0.0;
    config.projectile_ball.aerodynamics_enabled = aero;
    const double r = config.projectile_ball.radius_m;
    const double m = config.projectile_ball.mass_kg;
    MuJoCoSimulator sim(std::move(config));
    ASSERT_TRUE(sim.Initialize());
    const BallIds ball = FindBall(sim.GetModel());
    sim.RequestProjectileBallLaunch();
    sim.StepForTest();
    const double t0 = sim.GetData()->time;
    while (sim.GetData()->time - t0 < 1.0 - 1e-9) {
      sim.StepForTest();
    }
    const double t = sim.GetData()->time - t0;
    const double fall_speed = -sim.GetData()->qvel[ball.dof + 2];
    if (!aero) {
      // One step of gravity before t0.
      EXPECT_NEAR(fall_speed, kGravity * (t + sim.GetModel()->opt.timestep), 1e-6);
      continue;
    }
    const double cd = GetProjectileBallPhysics(ProjectileBallType::kTennis).drag_coefficient;
    const double terminal =
        std::sqrt(2.0 * m * kGravity / (kProjectileBallAirDensity * cd * M_PI * r * r));
    const double expected = terminal * std::tanh(kGravity * t / terminal);
    EXPECT_NEAR(fall_speed, expected, 0.01 * expected);
    // Positive control: drag is measurable at this horizon.
    EXPECT_LT(fall_speed, kGravity * t - 0.3);
  }
}

TEST(ProjectileBall, SpinIsSetInLaunchFrameAndBackspinLifts) {
  ProjectileBallConfig sample_config;
  sample_config.launch_direction = {0.0, 1.0, 0.0};
  sample_config.launch_spin_rad_s = {0.0, -50.0, 10.0};
  std::mt19937_64 rng(3);
  const auto sample = SampleProjectileBallLaunch(sample_config, rng);
  // forward = +y, left = z x y = -x: -50 * left + 10 * up.
  EXPECT_NEAR(sample.angular_velocity_rad_s[0], 50.0, 1e-12);
  EXPECT_NEAR(sample.angular_velocity_rad_s[1], 0.0, 1e-12);
  EXPECT_NEAR(sample.angular_velocity_rad_s[2], 10.0, 1e-12);

  const auto fly = [](double backspin_rad_s) {
    auto config = MakeBallSceneConfig(ProjectileBallType::kTennis);
    config.projectile_ball.spawn_position_m = {-2.0, -2.0, 3.0};
    config.projectile_ball.launch_speed_m_s = 6.0;
    config.projectile_ball.aerodynamics_enabled = true;
    config.projectile_ball.launch_spin_rad_s = {0.0, -backspin_rad_s, 0.0};
    MuJoCoSimulator sim(std::move(config));
    EXPECT_TRUE(sim.Initialize());
    const BallIds ball = FindBall(sim.GetModel());
    sim.RequestProjectileBallLaunch();
    sim.StepForTest();
    const std::array<double, 3> spin = {sim.GetData()->qvel[ball.dof + 3],
                                        sim.GetData()->qvel[ball.dof + 4],
                                        sim.GetData()->qvel[ball.dof + 5]};
    while (sim.GetData()->time < 0.4) {
      sim.StepForTest();
    }
    return std::make_pair(spin, sim.GetData()->qpos[ball.qpos + 2]);
  };
  const auto [spin, height_with_backspin] = fly(150.0);
  EXPECT_NEAR(spin[1], -150.0, 1e-6);
  const auto [no_spin, height_without_spin] = fly(0.0);
  // 150 rad/s at ~6 m/s: S ~ 0.8, C_L ~ 0.32, lift ~ 0.024 N on 57 g -> ~0.4 m/s^2,
  // i.e. ~3 cm higher after 0.4 s.
  EXPECT_GT(height_with_backspin, height_without_spin + 0.02);
}

TEST(ProjectileBall, SampleReportsAngularVelocityInWorldFrame) {
  auto config = MakeBallSceneConfig(ProjectileBallType::kTennis);
  config.projectile_ball.spawn_position_m = {-2.0, -2.0, 0.3};
  config.projectile_ball.launch_angle_deg = -45.0;
  config.projectile_ball.launch_speed_m_s = 3.0;
  config.projectile_ball.launch_spin_rad_s = {0.0, 0.0, 20.0};
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());
  const BallIds ball = FindBall(sim.GetModel());
  ASSERT_GE(ball.dof, 0);
  sim.RequestProjectileBallLaunch();
  // A free sphere spins about a fixed axis, so body and world components stay
  // equal. The slanted bounce adds topspin (about y) by sliding friction while
  // the ball is already turned about z, which is what makes the frames differ.
  while (sim.GetData()->time < 0.3) {
    sim.StepForTest();
  }
  const auto* data = sim.GetData();
  const auto& sample = sim.GetProjectileBallSampleForTest();
  ASSERT_TRUE(sample.active);
  mjtNum world[3];
  mju_rotVecQuat(world, data->qvel + ball.dof + 3, data->qpos + ball.qpos + 3);
  double body_world_gap = 0.0;
  for (int i = 0; i < 3; ++i) {
    const auto k = static_cast<std::size_t>(i);
    EXPECT_NEAR(sample.angular_velocity[k], world[i], 1e-9) << "axis " << i;
    body_world_gap = std::max(body_world_gap, std::abs(world[i] - data->qvel[ball.dof + 3 + i]));
  }
  // Positive control: the raw body-frame qvel must not already equal world.
  EXPECT_GT(body_world_gap, 0.5);
}

TEST(ProjectileBall, AzimuthAndNormalNoiseSampling) {
  ProjectileBallConfig config;
  config.launch_speed_m_s = 3.0;
  config.launch_azimuth_variation_deg = 10.0;
  std::mt19937_64 rng(11);
  bool saw_left = false;
  bool saw_right = false;
  for (int i = 0; i < 200; ++i) {
    const auto sample = SampleProjectileBallLaunch(config, rng);
    EXPECT_LE(std::abs(sample.azimuth_deg), 10.0);
    const double heading = std::atan2(sample.linear_velocity_m_s[1], sample.linear_velocity_m_s[0]);
    EXPECT_NEAR(heading * 180.0 / M_PI, sample.azimuth_deg, 1e-9);
    saw_left = saw_left || sample.azimuth_deg > 5.0;
    saw_right = saw_right || sample.azimuth_deg < -5.0;
  }
  EXPECT_TRUE(saw_left);
  EXPECT_TRUE(saw_right);

  config.launch_noise = ProjectileBallNoise::kNormal;
  config.launch_speed_variation_m_s = 0.5;
  std::mt19937_64 first(5);
  std::mt19937_64 second(5);
  double sum = 0.0;
  double sum_sq = 0.0;
  bool beyond_half_width = false;
  constexpr int kDraws = 4000;
  for (int i = 0; i < kDraws; ++i) {
    const auto a = SampleProjectileBallLaunch(config, first);
    const auto b = SampleProjectileBallLaunch(config, second);
    EXPECT_DOUBLE_EQ(a.speed_m_s, b.speed_m_s);
    sum += a.speed_m_s;
    sum_sq += a.speed_m_s * a.speed_m_s;
    // A uniform draw with the same variation could never leave [2.5, 3.5].
    beyond_half_width = beyond_half_width || std::abs(a.speed_m_s - 3.0) > 0.5;
  }
  const double mean = sum / kDraws;
  const double stddev = std::sqrt(sum_sq / kDraws - mean * mean);
  EXPECT_NEAR(mean, 3.0, 0.05);
  EXPECT_NEAR(stddev, 0.5, 0.05);
  EXPECT_TRUE(beyond_half_width);
}

// The ball is a freejoint body, so object_state discovers it. Parked, it sits
// under the floor and must not appear in /sim/object_transforms; launched, it
// is a free body under physics like any other object.
TEST(ProjectileBall, ObjectStateOmitsBallOnlyWhileParked) {
  auto config = MakeFloorSceneConfigWithBall();
  config.object_state.enabled = true;
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());

  const auto& infos = sim.GetObjectStateInfos();
  std::size_t ball_index = infos.size();
  for (std::size_t i = 0; i < infos.size(); ++i) {
    if (infos[i].name == "projectile_ball") {
      ball_index = i;
    }
  }
  ASSERT_LT(ball_index, infos.size()) << "the ball must be discovered as a free body";

  sim.StepForTest();
  EXPECT_FALSE(sim.GetObjectStateSamplesForTest()[ball_index].active) << "parked at startup";

  sim.RequestProjectileBallLaunch();
  sim.StepForTest();
  EXPECT_TRUE(sim.GetObjectStateSamplesForTest()[ball_index].active) << "launched";

  sim.RequestProjectileBallReset();
  sim.StepForTest();
  EXPECT_FALSE(sim.GetObjectStateSamplesForTest()[ball_index].active) << "parked by reset";
}

// ── Stated launch (D-14, /sim/launch_ball_at) ───────────────────────────────

TEST(ProjectileBall, RejectsNonFiniteLaunchCommand) {
  ProjectileBallLaunchCommand command;
  std::string error;
  EXPECT_TRUE(ValidateProjectileBallLaunchCommand(command, error))
      << "an all-zero command is a drop, not an error";

  command.position_m = {0.0, std::numeric_limits<double>::quiet_NaN(), 0.0};
  EXPECT_FALSE(ValidateProjectileBallLaunchCommand(command, error));
  EXPECT_NE(error.find("position"), std::string::npos);

  command.position_m = {0.0, 0.0, 1.0};
  command.linear_velocity_m_s = {std::numeric_limits<double>::infinity(), 0.0, 0.0};
  EXPECT_FALSE(ValidateProjectileBallLaunchCommand(command, error));
  EXPECT_NE(error.find("velocity"), std::string::npos);

  command.linear_velocity_m_s = {1.0, 0.0, 0.0};
  command.angular_velocity_rad_s = {0.0, 0.0, -std::numeric_limits<double>::infinity()};
  EXPECT_FALSE(ValidateProjectileBallLaunchCommand(command, error));
  EXPECT_NE(error.find("angular_velocity"), std::string::npos);
}

TEST(ProjectileBall, StatedLaunchArmsExactlyTheRequestedState) {
  auto config = MakeFloorSceneConfigWithBall();
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());
  const BallIds ball = FindBall(sim.GetModel());
  ASSERT_GE(ball.qpos, 0);
  ASSERT_GE(ball.dof, 0);
  const auto* data = sim.GetData();

  // StepForTest arms AND integrates, so the readback is the armed state plus
  // exactly one step. Asking for a state whose horizontal motion is zero makes
  // that step observable instead of confounding: with aerodynamics off, the
  // only force on a free ball is gravity, so x, y and the orientation cannot
  // move at all and can be compared exactly. Anything the arming write got
  // wrong in those components has nowhere to hide.
  const ProjectileBallLaunchCommand resting{{0.31, -0.22, 2.75}, {}, {}};
  std::string error;
  ASSERT_TRUE(sim.RequestProjectileBallLaunchAt(resting, error)) << error;
  sim.StepForTest();

  EXPECT_DOUBLE_EQ(data->qpos[ball.qpos + 0], resting.position_m[0]);
  EXPECT_DOUBLE_EQ(data->qpos[ball.qpos + 1], resting.position_m[1]);
  EXPECT_DOUBLE_EQ(data->qpos[ball.qpos + 3], 1.0) << "orientation is reset to identity";
  for (int i = 1; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(data->qpos[ball.qpos + 3 + i], 0.0) << "quaternion component " << i;
  }
  // Fell, and fell by no more than one step of free fall — it was armed at the
  // requested height rather than dropped from the configured spawn.
  const double dz = resting.position_m[2] - data->qpos[ball.qpos + 2];
  EXPECT_GT(dz, 0.0);
  EXPECT_LT(dz, 0.5 * 9.81 * sim.GetModel()->opt.timestep * sim.GetModel()->opt.timestep * 4.0);

  // Velocity and spin are armed verbatim too. Horizontal velocity and the spin
  // of an isotropic sphere are untouched by gravity, so these are exact.
  const ProjectileBallLaunchCommand moving{{0.0, 0.0, 3.0}, {1.5, -0.5, 3.25}, {0.0, -40.0, 0.0}};
  ASSERT_TRUE(sim.RequestProjectileBallLaunchAt(moving, error)) << error;
  sim.StepForTest();

  EXPECT_DOUBLE_EQ(data->qvel[ball.dof + 0], moving.linear_velocity_m_s[0]);
  EXPECT_DOUBLE_EQ(data->qvel[ball.dof + 1], moving.linear_velocity_m_s[1]);
  EXPECT_LT(data->qvel[ball.dof + 2], moving.linear_velocity_m_s[2]) << "gravity acted on vz";
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(data->qvel[ball.dof + 3 + i],
                moving.angular_velocity_rad_s[static_cast<std::size_t>(i)], 1e-12)
        << "spin axis " << i;
  }
}

TEST(ProjectileBall, StatedLaunchRefusesWhenNoBallExists) {
  auto config = MakeFloorSceneConfigWithBall();
  config.projectile_ball.enabled = false;
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());
  ASSERT_LT(FindBall(sim.GetModel()).body, 0) << "no ball body with the lane disabled";

  std::string error;
  EXPECT_FALSE(sim.RequestProjectileBallLaunchAt({{0.0, 0.0, 2.0}, {1.0, 0.0, 0.0}, {}}, error));
  EXPECT_NE(error.find("disabled"), std::string::npos);

  // The refusal must be the end of it. A staged-but-unlaunchable request would
  // fire the moment a ball appeared, which is the silent action the srv
  // contract rules out.
  EXPECT_NO_FATAL_FAILURE(sim.StepForTest());
}

TEST(ProjectileBall, StatedLaunchRefusesNonFiniteAndStagesNothing) {
  auto config = MakeFloorSceneConfigWithBall();
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());
  const BallIds ball = FindBall(sim.GetModel());
  ASSERT_GE(ball.qpos, 0);
  const auto* data = sim.GetData();

  sim.StepForTest();
  const double parked_z = data->qpos[ball.qpos + 2];

  std::string error;
  EXPECT_FALSE(sim.RequestProjectileBallLaunchAt(
      {{0.0, 0.0, std::numeric_limits<double>::quiet_NaN()}, {1.0, 0.0, 0.0}, {}}, error));

  sim.StepForTest();
  // Still parked: a refused request changed nothing. Had the NaN been staged it
  // would have reached qpos and poisoned every body in the scene, not just the
  // ball.
  EXPECT_NEAR(data->qpos[ball.qpos + 2], parked_z, 1e-9);
  EXPECT_FALSE(sim.GetProjectileBallSampleForTest().active);
}

TEST(ProjectileBall, StatedLaunchReplaysBitIdentically) {
  auto config = MakeFloorSceneConfigWithBall();
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());
  const BallIds ball = FindBall(sim.GetModel());
  ASSERT_GE(ball.qpos, 0);
  const auto* data = sim.GetData();

  const ProjectileBallLaunchCommand command{{0.0, 0.0, 3.0}, {1.25, 0.4, 0.75}, {0.0, -25.0, 0.0}};
  constexpr int kFlightSteps = 40;

  const auto fly = [&]() {
    std::string error;
    EXPECT_TRUE(sim.RequestProjectileBallLaunchAt(command, error)) << error;
    std::vector<double> track;
    track.reserve(static_cast<std::size_t>(kFlightSteps) * 3);
    for (int s = 0; s < kFlightSteps; ++s) {
      sim.StepForTest();
      for (int i = 0; i < 3; ++i) {
        track.push_back(data->qpos[ball.qpos + i]);
      }
    }
    return track;
  };

  const std::vector<double> first = fly();
  const std::vector<double> second = fly();

  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    // Bit-identical, not near: the whole reason this lane exists is to re-run
    // one throw while something else changes, and a tolerance here would hide
    // exactly the drift that makes such a comparison meaningless.
    EXPECT_EQ(first[i], second[i]) << "sample " << i;
  }
  // The flight must actually have gone somewhere, or the comparison above is
  // two copies of the arming state.
  EXPECT_GT(std::abs(first.front() - first.back()), 1e-6);
}

TEST(ProjectileBall, StatedLaunchDoesNotDisturbTheSampledRngStream) {
  const auto draw_two_sampled = [](bool interleave_stated) {
    auto config = MakeFloorSceneConfigWithBall();
    // Without variation every draw is the same vector and the comparison below
    // would pass on a dead RNG. The final EXPECT_NE guards that, but the spread
    // has to exist for the test to have any power in the first place.
    config.projectile_ball.launch_speed_m_s = 6.0;
    config.projectile_ball.launch_speed_variation_m_s = 1.5;
    config.projectile_ball.launch_angle_deg = 30.0;
    config.projectile_ball.launch_angle_variation_deg = 8.0;
    MuJoCoSimulator sim(std::move(config));
    EXPECT_TRUE(sim.Initialize());
    std::vector<std::array<double, 3>> draws;

    sim.RequestProjectileBallLaunch();
    sim.StepForTest();
    draws.push_back(sim.GetProjectileBallSampleForTest().linear_velocity);

    if (interleave_stated) {
      std::string error;
      EXPECT_TRUE(sim.RequestProjectileBallLaunchAt({{0.0, 0.0, 4.0}, {9.0, 9.0, 9.0}, {}}, error))
          << error;
      sim.StepForTest();
    }

    sim.RequestProjectileBallLaunch();
    sim.StepForTest();
    draws.push_back(sim.GetProjectileBallSampleForTest().linear_velocity);
    return draws;
  };

  const auto clean = draw_two_sampled(false);
  const auto interleaved = draw_two_sampled(true);

  // A stated launch must not consume the seeded stream. If it did, a run that
  // inserts one stated throw would silently renumber every sampled throw after
  // it, and a "same seed" sweep would stop being the same sweep.
  ASSERT_EQ(clean.size(), interleaved.size());
  for (std::size_t d = 0; d < clean.size(); ++d) {
    for (std::size_t i = 0; i < 3; ++i) {
      EXPECT_EQ(clean[d][i], interleaved[d][i]) << "draw " << d << " axis " << i;
    }
  }
  // Guard against the assertion above passing because both draws are equal to
  // each other (a dead RNG would satisfy it trivially).
  EXPECT_NE(clean[0][0], clean[1][0]);
}

// ── Clock phase lane (D-3 / S3.1a) ──────────────────────────────────────────

TEST(SimClockLane, StaysSilentUntilEnabled) {
  auto config = MakeFloorSceneConfigWithBall();
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());

  for (int s = 0; s < 20; ++s) {
    sim.StepForTest();
  }
  std::array<SimClockSample, 8> batch{};
  EXPECT_EQ(sim.DrainClockLane(batch.data(), batch.size()), 0U)
      << "a measurement lane nobody asked for must not cost a ring write per step";
  EXPECT_EQ(sim.ClockLaneDropped(), 0U);
}

TEST(SimClockLane, PairsEveryStepWithBothClocks) {
  auto config = MakeFloorSceneConfigWithBall();
  config.clock_lane_enabled = true;
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());

  constexpr int kSteps = 32;
  for (int s = 0; s < kSteps; ++s) {
    sim.StepForTest();
  }

  std::array<SimClockSample, 64> batch{};
  const std::size_t n = sim.DrainClockLane(batch.data(), batch.size());
  ASSERT_EQ(n, static_cast<std::size_t>(kSteps)) << "one sample per completed step, no more";
  EXPECT_EQ(sim.ClockLaneDropped(), 0U);

  const double dt = sim.GetModel()->opt.timestep * static_cast<double>(1);
  for (std::size_t i = 0; i < n; ++i) {
    EXPECT_EQ(batch[i].step, i + 1) << "steps are numbered from 1 and never skipped";
    // Sim time must advance by exactly the physics step. This is the axis the
    // lane exists to expose, so an off-by-a-substep here would silently rescale
    // every phase measurement taken from it.
    EXPECT_NEAR(batch[i].sim_time_sec, static_cast<double>(i + 1) * dt, 1e-12) << "sample " << i;
    EXPECT_GT(batch[i].steady_ns, 0);
    if (i > 0) {
      EXPECT_GE(batch[i].steady_ns, batch[i - 1].steady_ns) << "steady clock must not go back";
    }
  }

  std::array<SimClockSample, 4> empty{};
  EXPECT_EQ(sim.DrainClockLane(empty.data(), empty.size()), 0U) << "draining is consuming";
}

TEST(SimClockLane, CountsOverflowInsteadOfDroppingSilently) {
  auto config = MakeFloorSceneConfigWithBall();
  config.clock_lane_enabled = true;
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());

  // Never drain, so the ring fills and then overflows. This is the case that
  // matters: a lane that drops quietly under-reports exactly the tail — the
  // large-delta, long-pause steps — that D-3 is trying to find, and it does so
  // while still producing a full-looking CSV.
  const std::size_t capacity = MuJoCoSimulator::kClockLaneCapacity;
  const int steps = static_cast<int>(capacity) + 50;
  for (int s = 0; s < steps; ++s) {
    sim.StepForTest();
  }

  EXPECT_GT(sim.ClockLaneDropped(), 0U) << "overflow must be visible, not inferred";

  std::size_t held = 0;
  std::array<SimClockSample, 256> batch{};
  std::size_t n = 0;
  while ((n = sim.DrainClockLane(batch.data(), batch.size())) > 0) {
    held += n;
  }
  // Every step is accounted for: what was kept plus what was counted as lost.
  EXPECT_EQ(held + sim.ClockLaneDropped(), static_cast<std::uint64_t>(steps));
}

// ── Ball contact truth (S3.3) ───────────────────────────────────────────────

namespace {
// Clear of the two-link arm in pool_scene.xml, so what the ball lands on is the
// floor and nothing else. Dropping at the origin lands it on the robot, which
// is a fine contact but not a controlled one.
constexpr double kClearDropX = 1.5;
constexpr double kClearDropY = 1.5;

struct DropTrace {
  std::vector<BallContactEvent> events;
  std::vector<double> sim_time;  ///< per step, after the step
  std::vector<double> vz;        ///< ball world vz at the same instant
};

DropTrace DropOnFloor(MuJoCoSimulator& sim, const BallIds& ball, double z, int steps) {
  std::string error;
  EXPECT_TRUE(sim.RequestProjectileBallLaunchAt({{kClearDropX, kClearDropY, z}, {}, {}}, error))
      << error;
  DropTrace trace;
  std::array<BallContactEvent, 16> batch{};
  const auto* data = sim.GetData();
  for (int s = 0; s < steps; ++s) {
    sim.StepForTest();
    trace.sim_time.push_back(data->time);
    trace.vz.push_back(ball.dof >= 0 ? data->qvel[ball.dof + 2] : 0.0);
    const std::size_t n = sim.DrainBallContactLane(batch.data(), batch.size());
    for (std::size_t i = 0; i < n; ++i) {
      trace.events.push_back(batch[i]);
    }
  }
  return trace;
}
}  // namespace

TEST(BallContactLane, StaysSilentUntilEnabled) {
  auto config = MakeFloorSceneConfigWithBall();
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());
  const BallIds ball = FindBall(sim.GetModel());
  EXPECT_TRUE(DropOnFloor(sim, ball, 0.20, 400).events.empty())
      << "a lane nobody asked for must not scan contacts every substep";
  EXPECT_EQ(sim.BallContactDropped(), 0U);
}

TEST(BallContactLane, ImpulseOnTheBallPointsAwayFromWhatItHit) {
  auto config = MakeFloorSceneConfigWithBall();
  config.ball_contact_lane_enabled = true;
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());
  const BallIds ball = FindBall(sim.GetModel());
  ASSERT_GE(ball.dof, 0);

  const auto trace = DropOnFloor(sim, ball, 0.20, 400);
  ASSERT_FALSE(trace.events.empty()) << "a ball dropped on the floor must register a contact";

  const auto& first = trace.events.front();
  ASSERT_EQ(first.first_body_id, 0) << "expected the world body (floor plane), not the robot";

  // The sign is the whole point. mj_contactForce reports the force on geom[1]
  // of the pair, and the ball is on either side depending on geom id order, so
  // an unflipped read would give an impulse pointing INTO the floor for half
  // the scenes in this repo while still looking like a plausible number.
  EXPECT_GT(first.impulse_world[2], 0.0) << "the floor pushes the ball UP";
  // A vertical drop onto a horizontal plane: the lateral impulse is friction
  // only, so it must be small next to the normal one rather than exactly zero.
  const double lateral = std::hypot(first.impulse_world[0], first.impulse_world[1]);
  EXPECT_LT(lateral, 0.1 * first.impulse_world[2]);

  EXPECT_GT(first.peak_force_n, 0.0);
  EXPECT_GE(first.substeps, 1);
  EXPECT_GE(first.end_sim_time_sec, first.begin_sim_time_sec);
  EXPECT_LT(first.velocity_at_begin[2], 0.0) << "it was still falling when it touched";
  EXPECT_GE(first.distinct_bodies, 1);
  EXPECT_EQ(sim.BallContactDropped(), 0U);
}

TEST(BallContactLane, SignIsRightWhenTheBallIsTheFirstGeomOfThePair) {
  // mj_contactForce reports the force on geom[1], so the accumulator flips the
  // sign when the ball is geom[0]. Against the floor and the robot the ball is
  // always geom[1] — it is attached to the spec last, so its geom id is the
  // highest — and the flip is dead code that no assertion can reach. A pool
  // object IS attached after the ball, which is the one way in this package to
  // put the ball on the first side of a pair.
  auto config = MakeFloorSceneConfigWithBall();
  config.ball_contact_lane_enabled = true;
  config.object_pool.enabled = true;
  config.object_pool.directory = BALL_POOL_OBJECTS_DIR;
  config.object_pool.position = {0.0, 0.0, 0.10};
  config.object_pool.park_position = {0.0, 0.0, -50.0};
  config.object_pool.selection = ObjectSelection::kFixed;
  config.object_pool.pose = PoseSampling::kFixed;
  config.object_pool.spawn_on_start = true;
  config.object_pool.seed = 20260920;
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());
  const BallIds ball = FindBall(sim.GetModel());
  ASSERT_GE(ball.geom, 0);

  // Let the spawned object settle before anything is dropped on it.
  for (int s = 0; s < 300; ++s) {
    sim.StepForTest();
  }
  std::array<BallContactEvent, 16> flush{};
  while (sim.DrainBallContactLane(flush.data(), flush.size()) > 0) {
  }

  std::string error;
  ASSERT_TRUE(sim.RequestProjectileBallLaunchAt({{0.0, 0.0, 0.40}, {0.0, 0.0, -1.5}, {}}, error))
      << error;

  std::vector<BallContactEvent> events;
  std::array<BallContactEvent, 16> batch{};
  for (int s = 0; s < 300 && events.empty(); ++s) {
    sim.StepForTest();
    const std::size_t n = sim.DrainBallContactLane(batch.data(), batch.size());
    for (std::size_t i = 0; i < n; ++i) {
      events.push_back(batch[i]);
    }
  }
  ASSERT_FALSE(events.empty());

  const auto& e = events.front();
  // The pool object sits above the floor, so what the ball met first is not the
  // world body — which is what makes this the pair with the ball first.
  ASSERT_NE(e.first_body_id, 0) << "expected the spawned object, not the floor";
  EXPECT_GT(e.impulse_world[2], 0.0)
      << "an object under the ball pushes it UP; an unflipped sign reads as a pull DOWN";
}

TEST(BallContactLane, IntegratesEverySubstepNotJustTheLastOne) {
  // StepForTest runs ONE mj_step, so every unit test above exercises
  // n_substeps == 1 — and with one substep, accumulating inside the substep
  // loop and accumulating after it are the same code. The hazard this lane was
  // written around (mjData::contact is rebuilt by each mj_step, so a read
  // placed after the loop sees only the last one) is therefore invisible to
  // them. This drives the real SimLoop with n_substeps > 1 instead.
  auto config = MakeFloorSceneConfigWithBall();
  config.ball_contact_lane_enabled = true;
  config.n_substeps = 4;
  config.sync_timeout_ms = 10.0;
  config.max_rtf = 0.0;
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());

  std::mutex trace_mutex;
  std::vector<std::pair<double, double>> vz_trace;  // (sim_time, vz)
  sim.SetProjectileBallCallback([&](const ProjectileBallSample& s) {
    if (!s.active) {
      return;
    }
    std::lock_guard lock(trace_mutex);
    vz_trace.emplace_back(s.sim_time_sec, s.linear_velocity[2]);
  });

  sim.Start();
  std::string error;
  EXPECT_TRUE(sim.RequestProjectileBallLaunchAt({{kClearDropX, kClearDropY, 0.30}, {}, {}}, error))
      << error;
  std::vector<BallContactEvent> events;
  std::array<BallContactEvent, 16> batch{};
  for (int i = 0; i < 600 && events.empty(); ++i) {
    sim.SetCommand(0, {0.0, 0.0});
    std::this_thread::sleep_for(1ms);
    const std::size_t n = sim.DrainBallContactLane(batch.data(), batch.size());
    for (std::size_t k = 0; k < n; ++k) {
      events.push_back(batch[k]);
    }
  }
  sim.Stop();
  ASSERT_FALSE(events.empty()) << "the ball never touched the floor within the run";

  std::vector<std::pair<double, double>> trace;
  {
    std::lock_guard lock(trace_mutex);
    trace = vz_trace;
  }
  ASSERT_GE(trace.size(), 4U);

  const auto& e = events.front();
  ASSERT_GT(e.substeps, 1) << "this test only means something if the bounce spans substeps";

  std::size_t before = trace.size(), after = trace.size();
  for (std::size_t i = 0; i < trace.size(); ++i) {
    if (trace[i].first <= e.begin_sim_time_sec) {
      before = i;
    }
    if (after == trace.size() && trace[i].first >= e.end_sim_time_sec &&
        trace[i].first > e.begin_sim_time_sec) {
      after = i;
    }
  }
  ASSERT_LT(before, trace.size());
  ASSERT_LT(after, trace.size());

  const int ball_body = mj_name2id(sim.GetModel(), mjOBJ_BODY, "projectile_ball");
  ASSERT_GE(ball_body, 0);
  const double mass = sim.GetModel()->body_mass[ball_body];
  const double gz = sim.GetModel()->opt.gravity[2];
  const double window = trace[after].first - trace[before].first;
  const double expected = mass * ((trace[after].second - trace[before].second) - gz * window);

  // Reading only the last substep would under-count the impulse by roughly the
  // number of substeps the bounce spanned, which this tolerance cannot absorb.
  EXPECT_NEAR(e.impulse_world[2], expected, 0.15 * std::abs(expected))
      << "reported " << e.impulse_world[2] << " vs momentum-derived " << expected << " over "
      << e.substeps << " substeps";
}

TEST(BallContactLane, ImpulseMatchesTheMomentumChangeItCauses) {
  auto config = MakeFloorSceneConfigWithBall();
  config.ball_contact_lane_enabled = true;
  MuJoCoSimulator sim(std::move(config));
  ASSERT_TRUE(sim.Initialize());
  const BallIds ball = FindBall(sim.GetModel());
  ASSERT_GE(ball.dof, 0);

  const auto trace = DropOnFloor(sim, ball, 0.20, 400);
  ASSERT_FALSE(trace.events.empty());
  const auto& e = trace.events.front();

  // Independent oracle: the ball's own momentum. It never reads mjData::contact,
  // so a bug in the accumulation cannot also produce the number it is checked
  // against. Window it by the episode's own timestamps — the last sample before
  // the episode began and the first one after it ended — so gravity is
  // integrated over exactly the same interval as the contact.
  std::size_t before = trace.sim_time.size(), after = trace.sim_time.size();
  for (std::size_t i = 0; i < trace.sim_time.size(); ++i) {
    if (trace.sim_time[i] <= e.begin_sim_time_sec) {
      before = i;
    }
    if (after == trace.sim_time.size() && trace.sim_time[i] >= e.end_sim_time_sec &&
        trace.sim_time[i] > e.begin_sim_time_sec) {
      after = i;
    }
  }
  ASSERT_LT(before, trace.sim_time.size());
  ASSERT_LT(after, trace.sim_time.size());
  ASSERT_GT(after, before);

  const double mass = sim.GetModel()->body_mass[ball.body];
  const double gz = sim.GetModel()->opt.gravity[2];
  const double window = trace.sim_time[after] - trace.sim_time[before];

  // m * dv = J_contact + m * g * dt   =>   J_contact = m * (dv - g * dt)
  const double expected = mass * ((trace.vz[after] - trace.vz[before]) - gz * window);
  EXPECT_NEAR(e.impulse_world[2], expected, 0.10 * std::abs(expected))
      << "reported " << e.impulse_world[2] << " vs momentum-derived " << expected << " over ["
      << trace.sim_time[before] << ", " << trace.sim_time[after] << "]";
  EXPECT_GT(std::abs(expected), 1e-4) << "the bounce must be big enough for this to mean anything";
}

}  // namespace
}  // namespace rtc
