#include "rtc_mujoco_sim/projectile_ball.hpp"
#include "test_fixture.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <string>

#ifndef POOL_SCENE_MJCF_PATH
#error "POOL_SCENE_MJCF_PATH must be defined by CMake"
#endif

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

}  // namespace
}  // namespace rtc