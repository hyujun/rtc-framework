// ── test_command_state_io.cpp ─────────────────────────────────────────────────
// Tests SetCommand / SetControlMode / SetFakeTarget / GetPositions
// without running the simulation thread (no Start() calls).
// ──────────────────────────────────────────────────────────────────────────────
#include "test_fixture.hpp"

#include <gtest/gtest.h>

namespace rtc {
namespace {

TEST(CommandStateIO, SetCommandOnRobotGroupAccepted) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  // Does not throw, does not crash
  sim.SetCommand(0, {0.1, 0.2});
  SUCCEED();
}

TEST(CommandStateIO, SetCommandOnOutOfRangeGroupIsNoOp) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.SetCommand(99, {0.1, 0.2});  // no crash
  SUCCEED();
}

TEST(CommandStateIO, InitialPositionsZero) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  const auto pos = sim.GetPositions(0);
  ASSERT_EQ(pos.size(), 2u);
  // No keyframe in minimal.xml → zero initial
  EXPECT_DOUBLE_EQ(pos[0], 0.0);
  EXPECT_DOUBLE_EQ(pos[1], 0.0);
}

TEST(CommandStateIO, InitialVelocitiesZero) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  const auto vel = sim.GetVelocities(0);
  ASSERT_EQ(vel.size(), 2u);
  EXPECT_DOUBLE_EQ(vel[0], 0.0);
  EXPECT_DOUBLE_EQ(vel[1], 0.0);
}

TEST(CommandStateIO, ControlModeDefaultsToServo) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  EXPECT_FALSE(sim.IsInTorqueMode(0));
}

TEST(CommandStateIO, SetControlModeTorque) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.SetControlMode(0, true);
  EXPECT_TRUE(sim.IsInTorqueMode(0));
}

TEST(CommandStateIO, SetControlModeServoLocksGravity) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  // Start in torque mode → servo, expect gravity locked by servo
  sim.SetControlMode(0, true);
  sim.SetControlMode(0, false);
  EXPECT_TRUE(sim.IsGravityLockedByServo());
  EXPECT_FALSE(sim.IsGravityEnabled());
}

TEST(CommandStateIO, AllTorqueUnlocksGravity) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.SetControlMode(0, true);
  EXPECT_FALSE(sim.IsGravityLockedByServo());
  EXPECT_TRUE(sim.IsGravityEnabled());
}

TEST(CommandStateIO, FakeGroupTargetLpfRoundtrip) {
  auto cfg = test::MakeMinimalConfig();
  // Add a fake group
  JointGroupConfig fake;
  fake.name                = "hand";
  fake.command_joint_names = {"finger1"};
  fake.state_joint_names   = {"finger1"};
  fake.command_topic       = "/hand/cmd";
  fake.state_topic         = "/hand/state";
  fake.is_robot            = false;
  fake.filter_alpha        = 1.0;  // immediate convergence
  cfg.groups.push_back(fake);
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());

  sim.SetFakeTarget(1, {0.7});
  sim.AdvanceFakeLPF(1);
  const auto state = sim.GetFakeState(1);
  ASSERT_EQ(state.size(), 1u);
  EXPECT_DOUBLE_EQ(state[0], 0.7);
}

TEST(CommandStateIO, FakeGroupRejectsRobotOpsGracefully) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  // Group 0 is robot → SetFakeTarget should no-op, not crash
  sim.SetFakeTarget(0, {1.0});
  EXPECT_TRUE(sim.GetFakeState(0).empty());  // robot groups have empty fake_state
}

TEST(CommandStateIO, StateCallbackRegistration) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  bool called = false;
  sim.SetStateCallback(0, [&](const std::vector<double>&,
                              const std::vector<double>&,
                              const std::vector<double>&) {
    called = true;
  });
  // Callback is only fired inside SimLoop; here we only validate registration.
  // Direct invocation is private; lifecycle test will cover actual firing.
  EXPECT_FALSE(called);
}

TEST(CommandStateIO, JointNamesOrderMatchesConfig) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  const auto& names = sim.GetStateJointNames(0);
  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "j1");
  EXPECT_EQ(names[1], "j2");
}

}  // namespace
}  // namespace rtc
