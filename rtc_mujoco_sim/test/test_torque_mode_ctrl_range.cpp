// ── test_torque_mode_ctrl_range.cpp ──────────────────────────────────────────
// Torque mode on actuators the XML wrote as servos, with a ctrlrange (#815).
//
// The torque lane turns every actuator of the group into force = ctrl. On a
// `<position>` or `<velocity>` actuator the XML's ctrlrange was written for
// the servo's target — a joint range in rad, a speed in rad/s — and MuJoCo
// clamps ctrl to it before the force is computed. Left in place it caps the
// torque at the size of the joint range: a 50 N m command on a joint with a
// +-3.14 rad range came out as 3.14 N m, and nothing said so.
//
// What is pinned here, by the force on the joint and not by the flag alone:
//   - servo actuators in torque mode     → the torque passes its ctrlrange
//   - their forcerange                   → still bounds it
//   - a `<motor>` with a ctrlrange       → keeps it: that range is a torque
//   - leaving torque mode, either lane   → the target is clamped again
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"
#include "sim_config_fixture.hpp"

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <cstddef>
#include <vector>

#ifndef SERVO_ARM_CTRL_LIMITED_MJCF_PATH
#error "SERVO_ARM_CTRL_LIMITED_MJCF_PATH must be defined by CMake"
#endif

namespace rtc {
namespace {

// Actuator order of the fixture.
constexpr std::size_t kPos = 0;
constexpr std::size_t kVel = 1;
constexpr std::size_t kMot = 2;

// The fixture's numbers (asserted against the compiled model below).
constexpr double kPosKp = 100.0;
constexpr double kPosCtrlMax = 1.0;
constexpr double kVelKv = 10.0;
constexpr double kVelCtrlMax = 2.0;
constexpr double kVelForceMax = 30.0;
constexpr double kMotCtrlMax = 0.5;

MuJoCoSimulator::Config MakeConfig() {
  auto cfg = test::HeadlessConfig(SERVO_ARM_CTRL_LIMITED_MJCF_PATH, 10.0);
  cfg.groups.push_back(test::RobotGroup("arm", {"j1", "j2", "j3"}));
  return cfg;
}

void StepTorque(MuJoCoSimulator& sim, const std::vector<double>& torque) {
  sim.StageCommand(0, JointControlMode::kTorque, torque, {}, {}, {});
  sim.StepForTest();
}

bool CtrlLimited(const MuJoCoSimulator& sim, std::size_t actuator) {
  return sim.GetModel()->actuator_ctrllimited[actuator] != 0;
}

// Guards the premise: every case below reads a clamp that the fixture has to
// have compiled, on the kind of actuator it is named for.
TEST(TorqueModeCtrlRange, FixtureCompilesTwoServosAndAMotorAllCtrlLimited) {
  MuJoCoSimulator sim(MakeConfig());
  ASSERT_TRUE(sim.Initialize());
  const mjModel* m = sim.GetModel();
  ASSERT_EQ(m->nu, 3);

  EXPECT_EQ(m->actuator_biastype[kPos], mjBIAS_AFFINE);
  EXPECT_EQ(m->actuator_biastype[kVel], mjBIAS_AFFINE);
  EXPECT_EQ(m->actuator_biastype[kMot], mjBIAS_NONE);

  EXPECT_DOUBLE_EQ(m->actuator_gainprm[kPos * mjNGAIN], kPosKp);
  EXPECT_DOUBLE_EQ(m->actuator_gainprm[kVel * mjNGAIN], kVelKv);
  EXPECT_DOUBLE_EQ(m->actuator_ctrlrange[2 * kPos + 1], kPosCtrlMax);
  EXPECT_DOUBLE_EQ(m->actuator_ctrlrange[2 * kVel + 1], kVelCtrlMax);
  EXPECT_DOUBLE_EQ(m->actuator_ctrlrange[2 * kMot + 1], kMotCtrlMax);
  EXPECT_DOUBLE_EQ(m->actuator_forcerange[2 * kVel + 1], kVelForceMax);
  EXPECT_EQ(m->actuator_forcelimited[kPos], 0);

  // Before any command the group is in a position mode: the XML's clamps hold.
  sim.StepForTest();
  EXPECT_TRUE(CtrlLimited(sim, kPos));
  EXPECT_TRUE(CtrlLimited(sim, kVel));
  EXPECT_TRUE(CtrlLimited(sim, kMot));
}

// The group's first command is a torque — there is no earlier mode to leave.
TEST(TorqueModeCtrlRange, TorqueBeyondAServoCtrlRangeIsApplied) {
  MuJoCoSimulator sim(MakeConfig());
  ASSERT_TRUE(sim.Initialize());

  // 5 and -7 are 5x and 3.5x the two ctrlranges; 0.2 is inside the motor's.
  StepTorque(sim, {5.0, -7.0, 0.2});

  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, kPos), 5.0);
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, kVel), -7.0);
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, kMot), 0.2);
}

// What bounds the torque of a servo actuator in torque mode is its forcerange.
TEST(TorqueModeCtrlRange, ForceRangeStillBoundsTheTorque) {
  MuJoCoSimulator sim(MakeConfig());
  ASSERT_TRUE(sim.Initialize());

  StepTorque(sim, {0.0, -70.0, 0.0});
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, kVel), -kVelForceMax);

  StepTorque(sim, {0.0, 70.0, 0.0});
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, kVel), kVelForceMax);
}

// A motor's ctrlrange was a torque limit before the mode switch and is one
// after it.
TEST(TorqueModeCtrlRange, MotorKeepsItsCtrlRange) {
  MuJoCoSimulator sim(MakeConfig());
  ASSERT_TRUE(sim.Initialize());

  StepTorque(sim, {0.0, 0.0, 3.0});
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, kMot), kMotCtrlMax);
  EXPECT_TRUE(CtrlLimited(sim, kMot));

  StepTorque(sim, {0.0, 0.0, -3.0});
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, kMot), -kMotCtrlMax);
}

// Back on the XML's own actuators a target outside the ctrlrange is clamped
// again. The force is the servo law on the CLAMPED target, evaluated at the
// state the step started from — so it is read before the step.
TEST(TorqueModeCtrlRange, LeavingTorqueModeRestoresTheClampOnTheXmlLane) {
  MuJoCoSimulator sim(MakeConfig());
  ASSERT_TRUE(sim.Initialize());

  // A small torque: one step of it leaves the joints nearly at rest, so the
  // velocity servo's force below stays inside its force range.
  StepTorque(sim, {0.3, -0.2, 0.1});
  ASSERT_FALSE(CtrlLimited(sim, kPos));
  ASSERT_FALSE(CtrlLimited(sim, kVel));

  const auto q = sim.GetPositions(0);
  const auto qd = sim.GetVelocities(0);
  ASSERT_EQ(q.size(), 3u);
  ASSERT_EQ(qd.size(), 3u);

  // 2.5 rad and 5 rad/s: unclamped they would ask for 250 - 100 q and (past
  // the force range) 30; clamped, 100 - 100 q and 20 - 10 qd.
  sim.StageCommand(0, JointControlMode::kPosition, {2.5, 5.0, 3.0}, {}, {}, {});
  sim.StepForTest();

  EXPECT_TRUE(CtrlLimited(sim, kPos));
  EXPECT_TRUE(CtrlLimited(sim, kVel));
  EXPECT_TRUE(CtrlLimited(sim, kMot));
  EXPECT_NEAR(sim.GetActuatorForceForTest(0, kPos), kPosKp * (kPosCtrlMax - q[kPos]), 1e-9);
  const double vel_force = kVelKv * (kVelCtrlMax - qd[kVel]);
  ASSERT_LT(vel_force, kVelForceMax) << "the force range would hide the ctrl clamp";
  EXPECT_NEAR(sim.GetActuatorForceForTest(0, kVel), vel_force, 1e-9);
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, kMot), kMotCtrlMax);
}

// The gain lane is the other way out of torque mode.
TEST(TorqueModeCtrlRange, LeavingTorqueModeRestoresTheClampOnTheGainLane) {
  MuJoCoSimulator sim(MakeConfig());
  ASSERT_TRUE(sim.Initialize());

  StepTorque(sim, {5.0, -7.0, 0.2});
  ASSERT_FALSE(CtrlLimited(sim, kPos));

  const auto q = sim.GetPositions(0);
  const auto qd = sim.GetVelocities(0);
  const std::vector<double> kp{40.0, 20.0, 10.0};
  const std::vector<double> kd{2.0, 1.0, 0.5};
  sim.StageCommand(0, JointControlMode::kPosition, {2.5, 0.0, 0.0}, {}, kp, kd);
  sim.StepForTest();

  EXPECT_TRUE(CtrlLimited(sim, kPos));
  EXPECT_TRUE(CtrlLimited(sim, kVel));
  EXPECT_TRUE(CtrlLimited(sim, kMot));
  EXPECT_NEAR(sim.GetActuatorForceForTest(0, kPos),
              kp[kPos] * (kPosCtrlMax - q[kPos]) - kd[kPos] * qd[kPos], 1e-9);
}

// The clamp follows the mode on every switch, not only the first.
TEST(TorqueModeCtrlRange, ReenteringTorqueModeLiftsTheClampAgain) {
  MuJoCoSimulator sim(MakeConfig());
  ASSERT_TRUE(sim.Initialize());

  StepTorque(sim, {5.0, -7.0, 0.2});
  sim.StageCommand(0, JointControlMode::kPosition, {0.0, 0.0, 0.0}, {}, {}, {});
  sim.StepForTest();
  ASSERT_TRUE(CtrlLimited(sim, kPos));

  StepTorque(sim, {-4.0, 6.0, -0.1});
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, kPos), -4.0);
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, kVel), 6.0);
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, kMot), -0.1);
}

}  // namespace
}  // namespace rtc
