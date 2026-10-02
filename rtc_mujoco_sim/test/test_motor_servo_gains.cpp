// ── test_motor_servo_gains.cpp ───────────────────────────────────────────────
// Position-servo gains on a scene whose actuators are torque motors.
//
// A `<motor>` compiles to biastype none: MuJoCo then ignores biasprm, and
// force = gainprm[0] * ctrl. The servo lane used to write kp into gainprm and
// (0, -kp, -kd) into biasprm and stop there, which on such a scene is not a
// servo at all — a position command of 0.4 rad became a constant 0.4 * kp
// torque. Every other fixture in this package uses `<position>` actuators
// (already affine), so the lane had never run on a motor.
//
// What is pinned here, lane by lane:
//   - gains installed (YAML or runtime)  → the joint holds a non-zero target
//   - torque mode                        → the motor is a motor again
//   - position mode, no gains            → the XML's own actuator, untouched:
//                                          ctrl is a torque (scenes rely on it)
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"
#include "sim_config_fixture.hpp"

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <cmath>
#include <cstddef>
#include <vector>

#ifndef MOTOR_ARM_MJCF_PATH
#error "MOTOR_ARM_MJCF_PATH must be defined by CMake"
#endif

namespace rtc {
namespace {

// kd / kp = 0.05 s. Deliberately not symmetric across the two joints, and the
// target is non-zero on both with opposite signs: a lane that mixed the joints
// up, or held only the zero pose, would not land on these numbers.
const std::vector<double> kKp{40.0, 20.0};
const std::vector<double> kKd{2.0, 1.0};
const std::vector<double> kTarget{0.4, -0.3};
constexpr int kSettleSteps = 2000;  // 4 s of sim time at 2 ms
constexpr double kHoldTolRad = 1e-3;

MuJoCoSimulator::Config MakeMotorConfig() {
  auto cfg = test::HeadlessConfig(MOTOR_ARM_MJCF_PATH, 10.0);
  cfg.groups.push_back(test::RobotGroup("arm", {"j1", "j2"}));
  return cfg;
}

int BiasType(const MuJoCoSimulator& sim, int actuator) {
  return sim.GetModel()->actuator_biastype[actuator];
}

void ExpectHeld(const MuJoCoSimulator& sim) {
  const auto pos = sim.GetPositions(0);
  const auto vel = sim.GetVelocities(0);
  ASSERT_EQ(pos.size(), kTarget.size());
  ASSERT_EQ(vel.size(), kTarget.size());
  for (std::size_t i = 0; i < kTarget.size(); ++i) {
    EXPECT_NEAR(pos[i], kTarget[i], kHoldTolRad) << "joint " << i;
    EXPECT_NEAR(vel[i], 0.0, 1e-3) << "joint " << i << " still moving";
  }
}

// Guards the premise: if the fixture ever grew `<position>` actuators, every
// case below would pass on the unfixed lane.
TEST(MotorServoGains, FixtureActuatorsCompileToNoBias) {
  MuJoCoSimulator sim(MakeMotorConfig());
  ASSERT_TRUE(sim.Initialize());
  ASSERT_EQ(sim.GetModel()->nu, 2);
  EXPECT_EQ(BiasType(sim, 0), mjBIAS_NONE);
  EXPECT_EQ(BiasType(sim, 1), mjBIAS_NONE);
}

TEST(MotorServoGains, YamlGainsHoldANonZeroTarget) {
  auto cfg = MakeMotorConfig();
  cfg.use_yaml_servo_gains = true;
  cfg.servo_kp = kKp;
  cfg.servo_kd = kKd;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());

  for (int i = 0; i < kSettleSteps; ++i) {
    sim.StageCommand(0, JointControlMode::kPosition, kTarget, {}, {}, {});
    sim.StepForTest();
  }

  EXPECT_EQ(BiasType(sim, 0), mjBIAS_AFFINE);
  EXPECT_EQ(BiasType(sim, 1), mjBIAS_AFFINE);
  ExpectHeld(sim);
}

// Runtime gains take the same lane through a different door (gains_overridden
// instead of use_yaml_servo_gains).
TEST(MotorServoGains, RuntimeGainsHoldANonZeroTarget) {
  MuJoCoSimulator sim(MakeMotorConfig());
  ASSERT_TRUE(sim.Initialize());

  for (int i = 0; i < kSettleSteps; ++i) {
    sim.StageCommand(0, JointControlMode::kPosition, kTarget, {}, kKp, kKd);
    sim.StepForTest();
  }

  EXPECT_EQ(BiasType(sim, 0), mjBIAS_AFFINE);
  EXPECT_EQ(BiasType(sim, 1), mjBIAS_AFFINE);
  ExpectHeld(sim);
}

// Leaving the servo lane must undo the bias type it installed. Left affine with
// zeroed biasprm the force would still equal ctrl, so the type is asserted
// directly rather than inferred from the force.
TEST(MotorServoGains, TorqueModeRestoresTheMotor) {
  auto cfg = MakeMotorConfig();
  cfg.use_yaml_servo_gains = true;
  cfg.servo_kp = kKp;
  cfg.servo_kd = kKd;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());

  sim.StageCommand(0, JointControlMode::kPosition, kTarget, {}, {}, {});
  sim.StepForTest();
  ASSERT_EQ(BiasType(sim, 0), mjBIAS_AFFINE);

  const std::vector<double> torque{0.7, -0.2};
  sim.StageCommand(0, JointControlMode::kTorque, torque, {}, {}, {});
  sim.StepForTest();

  EXPECT_EQ(BiasType(sim, 0), mjBIAS_NONE);
  EXPECT_EQ(BiasType(sim, 1), mjBIAS_NONE);
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, 0), torque[0]);
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, 1), torque[1]);
}

// With no gains installed the lane hands the actuator back exactly as the XML
// compiled it. On a motor that means a "position" command is a torque — the
// behaviour test_gravcomp_scene and test_contact_wrench_known_load drive their
// scenes with, so it is a contract and not an accident. (The simulator says so
// once on stderr; that text is not asserted.)
TEST(MotorServoGains, WithoutGainsThePositionCommandIsATorque) {
  MuJoCoSimulator sim(MakeMotorConfig());
  ASSERT_TRUE(sim.Initialize());

  const std::vector<double> cmd{0.5, -0.25};
  sim.StageCommand(0, JointControlMode::kPosition, cmd, {}, {}, {});
  sim.StepForTest();

  EXPECT_EQ(BiasType(sim, 0), mjBIAS_NONE);
  EXPECT_EQ(BiasType(sim, 1), mjBIAS_NONE);
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, 0), cmd[0]);
  EXPECT_DOUBLE_EQ(sim.GetActuatorForceForTest(0, 1), cmd[1]);
}

}  // namespace
}  // namespace rtc
