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
#include <cmath>
#include <cstddef>
#include <thread>
#include <vector>

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

  // Regression guard: writing body_gravcomp[] alone is not enough — MuJoCo's
  // mj_passive() early-outs when ngravcomp==0, which is the compiled value
  // since the MJCF doesn't declare gravcomp. Init must recount.
  EXPECT_GE(model->ngravcomp, 2) << "ngravcomp must be recounted after Init's "
                                    "body_gravcomp writes";
}

TEST(GravcompScene, GravcompForceActuallyAppliedAfterForward) {
  // The other tests verify *intent* (body_gravcomp tags). This one verifies
  // *effect*: after mj_forward (which calls mj_passive → mj_gravcomp),
  // qfrc_gravcomp[] must be nonzero on the tagged DoFs. If RefreshNgravcomp()
  // is missing, mj_gravcomp() early-outs on ngravcomp==0 and leaves the
  // freshly-zeroed qfrc_gravcomp at all-zero.
  //
  // Using mj_forward directly (instead of SimLoop) sidesteps the test
  // fixture's sync-barrier timing — SimLoop blocks on cmd_pending and only
  // advances a handful of steps in 50 ms wall.
  MuJoCoSimulator sim(MakeSceneConfig());
  ASSERT_TRUE(sim.Initialize());

  const mjModel* model = sim.GetModel();
  ASSERT_NE(model, nullptr);
  // mj_forward needs a writable mjData; reuse the simulator's by const_cast on
  // its accessor — the simulator is idle (Start() not called) so there is no
  // concurrent writer.
  auto* data = const_cast<mjData*>(sim.GetData());
  ASSERT_NE(data, nullptr);

  // Rotate j2 (the y-axis hinge below the z-axis hinge j1) away from the
  // gravity-aligned zero pose so link2's inertial offset develops a nonzero
  // lever arm against world gravity. Without this, the symmetric fixture has
  // qfrc_gravcomp == 0 at qpos == 0 even when mj_gravcomp() runs correctly.
  data->qpos[1] = 1.0;  // j2 ≈ 57°
  mj_forward(model, data);

  double max_abs = 0.0;
  for (int i = 0; i < model->nv; ++i) {
    const double v = std::fabs(data->qfrc_gravcomp[i]);
    if (v > max_abs)
      max_abs = v;
  }
  EXPECT_GT(max_abs, 1e-6) << "qfrc_gravcomp[] is all zero — mj_passive() probably skipped the "
                              "gravcomp loop because ngravcomp==0";
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

// ── #447: the efforts lane must carry the gravity-holding term ───────────────
// DeviceState::efforts is joint torque ("torques for robot arm", rtc_base
// types.hpp) and carries every generalized force acting on the joint. In
// kPosition, body_gravcomp moves the gravity-holding force out of qfrc_actuator
// and into qfrc_gravcomp, so ReadState has to sum it back in. Skipping it does
// not empty the lane — it fills it with a different physical quantity than
// hardware reports, which a momentum observer reads as a real external torque
// converging to +g(q) under no load.
TEST(GravcompScene, PositionModeEffortCarriesGravityTerm) {
  MuJoCoSimulator sim(MakeSceneConfig());
  ASSERT_TRUE(sim.Initialize());
  ASSERT_TRUE(sim.IsGroupGravcompEnabled(0));

  const mjModel* model = sim.GetModel();
  ASSERT_NE(model, nullptr);
  auto* data = const_cast<mjData*>(sim.GetData());
  ASSERT_NE(data, nullptr);

  // Same off-axis pose as GravcompForceActuallyAppliedAfterForward: j2 rotated
  // away from the gravity-aligned zero so link2's inertial offset develops a
  // lever arm. At qpos == 0 the fixture is symmetric and g(q) == 0.
  data->qpos[0] = 0.0;
  data->qpos[1] = 1.0;  // j2 ~= 57 deg
  for (int i = 0; i < model->nv; ++i)
    data->qvel[i] = 0.0;

  // Oracle: at qvel == 0 the RNE bias force IS the generalized gravity torque
  // g(q). It comes from mj_rne, not from mj_gravcomp, so it is an independent
  // statement of the physics rather than a restatement of what ReadState sums.
  mj_forward(model, data);
  const double g_j2 = data->qfrc_bias[1];
  ASSERT_GT(std::fabs(g_j2), 1e-3) << "pose carries no gravity load — the assertion below would "
                                      "pass on an empty lane too";

  // Drive the PD error to zero so the actuator contributes nothing and the lane
  // is the gravity term alone. In the fixture m1 is a direct-torque motor
  // (ctrl = torque) and m2 a position servo (ctrl = target), so commanding
  // {0, current j2} zeroes both.
  sim.SetCommand(0, std::vector<double>{0.0, 1.0});
  sim.StepForTest();

  const auto efforts = sim.GetEfforts(0);
  ASSERT_EQ(efforts.size(), 2u);
  EXPECT_NEAR(efforts[1], g_j2, 1e-9)
      << "position-mode effort must include the gravity-holding term (#447)";

  // And the pre-fix sum misses it by exactly that term. This is the half that
  // goes red if `+ qfrc_gravcomp` is dropped from ReadState.
  const double actuator_plus_applied =
      sim.GetActuatorForceForTest(0, 1) + sim.GetAppliedForceForTest(0, 1);
  EXPECT_GT(std::fabs(efforts[1] - actuator_plus_applied), 1e-3)
      << "actuator+applied alone should differ from the lane by g(q)";
}

// AC2 of #447: the added term must be a no-op wherever gravcomp is OFF. That
// holds only because MuJoCo *writes* qfrc_gravcomp as zero instead of leaving
// the previous mode's values in place — mj_passive zeroes the array before its
// ngravcomp == 0 early-out (measured on 3.7.0 by poisoning the array with a
// sentinel and watching mj_forward/mj_step wipe it). Asserted directly, so a
// MuJoCo upgrade that changed it surfaces here rather than as a stale torque
// leaking into the lane after a mode switch.
TEST(GravcompScene, TorqueModeEffortIsUnaffectedByTheGravcompTerm) {
  MuJoCoSimulator sim(MakeSceneConfig());
  ASSERT_TRUE(sim.Initialize());

  const mjModel* model = sim.GetModel();
  ASSERT_NE(model, nullptr);
  auto* data = const_cast<mjData*>(sim.GetData());
  ASSERT_NE(data, nullptr);

  data->qpos[1] = 1.0;  // gravity load present, so a stale term would be visible
  for (int i = 0; i < model->nv; ++i)
    data->qvel[i] = 0.0;

  // One position-mode step first: this fills qfrc_gravcomp with g(q), so the
  // switch below is the runtime transition that could strand a stale value.
  sim.SetCommand(0, std::vector<double>{0.0, 1.0});
  sim.StepForTest();
  ASSERT_GT(std::fabs(data->qfrc_gravcomp[1]), 1e-3);

  sim.SetControlMode(0, JointControlMode::kTorque);
  const std::vector<double> tau = {0.5, 0.3};
  sim.SetCommand(0, tau);
  sim.StepForTest();
  ASSERT_FALSE(sim.IsGroupGravcompEnabled(0));

  EXPECT_DOUBLE_EQ(data->qfrc_gravcomp[0], 0.0);
  EXPECT_DOUBLE_EQ(data->qfrc_gravcomp[1], 0.0);

  const auto efforts = sim.GetEfforts(0);
  ASSERT_EQ(efforts.size(), 2u);
  for (std::size_t i = 0; i < efforts.size(); ++i) {
    const double actuator_plus_applied =
        sim.GetActuatorForceForTest(0, i) + sim.GetAppliedForceForTest(0, i);
    EXPECT_DOUBLE_EQ(efforts[i], actuator_plus_applied)
        << "torque mode must be byte-identical to the pre-#447 sum, joint " << i;
    // Torque mode is a pass-through (gain 1, bias 0), so the lane is the command.
    EXPECT_NEAR(efforts[i], tau[i], 1e-9) << "joint " << i;
  }
}

}  // namespace
}  // namespace rtc
