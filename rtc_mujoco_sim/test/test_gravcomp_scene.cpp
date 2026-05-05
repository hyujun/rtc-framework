// ── test_gravcomp_scene.cpp ──────────────────────────────────────────────────
// Regression: position-servo gravcomp must NOT lift free objects in the scene.
// Earlier strategy disabled world gravity globally, which made object-lift /
// manipulation simulations meaningless.  The fix uses MuJoCo's per-body
// gravcomp on the robot's body chain only.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <chrono>
#include <thread>

#ifndef SCENE_WITH_OBJECT_MJCF_PATH
#error "SCENE_WITH_OBJECT_MJCF_PATH must be defined by CMake"
#endif

namespace rtc {
namespace {

MuJoCoSimulator::Config MakeSceneConfig() {
  MuJoCoSimulator::Config cfg;
  cfg.model_path = SCENE_WITH_OBJECT_MJCF_PATH;
  cfg.enable_viewer = false;
  cfg.sync_timeout_ms = 10.0;
  cfg.max_rtf = 0.0;
  cfg.n_substeps = 1;
  cfg.use_yaml_servo_gains = false;

  JointGroupConfig group;
  group.name = "arm";
  group.command_joint_names = {"j1", "j2"};
  group.state_joint_names = {"j1", "j2"};
  group.command_topic = "/arm/cmd";
  group.state_topic = "/arm/state";
  group.is_robot = true;
  cfg.groups.push_back(group);
  return cfg;
}

TEST(GravcompScene, WorldGravityStaysOnInPositionServo) {
  // Even though the robot is in position-servo mode (gravcomp ON for its body
  // chain), opt.gravity must remain at the original world value so that other
  // free bodies fall normally.
  MuJoCoSimulator sim(MakeSceneConfig());
  ASSERT_TRUE(sim.Initialize());
  EXPECT_FALSE(sim.IsInTorqueMode(0));
  EXPECT_TRUE(sim.IsGroupGravcompEnabled(0));
  EXPECT_TRUE(sim.IsWorldGravityEnabled());
}

TEST(GravcompScene, RobotBodiesTaggedFreeObjectUntagged) {
  // Direct assertion on body_gravcomp[]: robot link bodies must be tagged 1.0,
  // free_object must remain 0.0 (default).
  MuJoCoSimulator sim(MakeSceneConfig());
  ASSERT_TRUE(sim.Initialize());

  const mjModel* model = sim.GetModel();
  ASSERT_NE(model, nullptr);

  const int free_body_id = mj_name2id(model, mjOBJ_BODY, "free_object");
  const int link1_id = mj_name2id(model, mjOBJ_BODY, "link1");
  const int link2_id = mj_name2id(model, mjOBJ_BODY, "link2");
  ASSERT_GT(free_body_id, 0);
  ASSERT_GT(link1_id, 0);
  ASSERT_GT(link2_id, 0);

  EXPECT_DOUBLE_EQ(model->body_gravcomp[link1_id], 1.0);
  EXPECT_DOUBLE_EQ(model->body_gravcomp[link2_id], 1.0);
  EXPECT_DOUBLE_EQ(model->body_gravcomp[free_body_id], 0.0);
}

TEST(GravcompScene, FreeObjectFallsUnderWorldGravity) {
  // The motivating regression: free objects in the same scene must keep
  // falling even while the robot group is in position-servo mode.  Read the
  // free body's Z position before and after a short run and assert descent
  // close to the analytical -0.5*g*t^2.
  MuJoCoSimulator sim(MakeSceneConfig());
  ASSERT_TRUE(sim.Initialize());

  const mjModel* model = sim.GetModel();
  const mjData* data = sim.GetData();
  ASSERT_NE(model, nullptr);
  ASSERT_NE(data, nullptr);

  const int free_joint_id = mj_name2id(model, mjOBJ_JOINT, "free_object_root");
  ASSERT_GE(free_joint_id, 0);
  const int z_qpos_idx = model->jnt_qposadr[free_joint_id] + 2;

  const double z_before = data->qpos[z_qpos_idx];
  EXPECT_NEAR(z_before, 1.0, 1e-9);  // matches XML pos="1.0 0 1.0"

  sim.Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  sim.Stop();  // joins SimLoop, so reading data is safe afterwards

  const double z_after = data->qpos[z_qpos_idx];
  const double sim_t = sim.SimTimeSec();
  ASSERT_GT(sim_t, 0.02) << "SimLoop did not advance — wall time too short?";

  // Free fall under world gravity: Δz ≈ -0.5 * 9.81 * t^2.  Tolerate 50% of
  // the analytical drop to absorb SimLoop sync wakeups, no-command throttling,
  // and the body's nonzero initial qvel solver settling.
  const double expected_drop = 0.5 * 9.81 * sim_t * sim_t;
  const double observed_drop = z_before - z_after;
  EXPECT_GT(observed_drop, 0.5 * expected_drop)
      << "free_object did not fall as expected: sim_t=" << sim_t << "s  drop=" << observed_drop
      << "m  analytical=" << expected_drop << "m";

  // While the robot was in position-servo mode the whole time.
  EXPECT_TRUE(sim.IsGroupGravcompEnabled(0));
  EXPECT_TRUE(sim.IsWorldGravityEnabled());
}

}  // namespace
}  // namespace rtc
