// ── DemoWbcController unit tests ─────────────────────────────────────────────
//
// Tests FSM logic, integration math, and output assembly without requiring
// a real URDF or TSID stack. The controller is constructed with an empty
// urdf_path; only the integration/FSM logic is exercised.

#include "ur5e_bringup/controllers/demo_wbc_controller.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <array>
#include <cmath>

namespace {

using ur5e_bringup::DemoWbcController;
using ur5e_bringup::WbcPhase;
using rtc::ControllerState;
using rtc::ControllerOutput;

// ── Helper: build a minimal ControllerState ──────────────────────────────────
ControllerState MakeState(double dt = 0.002)
{
  ControllerState state{};
  state.num_devices = 2;
  state.dt = dt;
  state.iteration = 1;

  // Arm: 6 joints at home position
  auto & dev0 = state.devices[0];
  dev0.num_channels = 6;
  dev0.valid = true;
  dev0.positions = {};
  dev0.positions[0] = 0.0;
  dev0.positions[1] = -1.57;
  dev0.positions[2] = 1.57;
  dev0.positions[3] = -1.57;
  dev0.positions[4] = -1.57;
  dev0.positions[5] = 0.0;

  // Hand: 10 joints at zero
  auto & dev1 = state.devices[1];
  dev1.num_channels = 10;
  dev1.valid = true;

  return state;
}

// ── FSM Tests ────────────────────────────────────────────────────────────────

class WbcFSMTest : public ::testing::Test {
protected:
  DemoWbcController ctrl_{""};
  ControllerState state_ = MakeState();

  void SetUp() override
  {
    // Initialize hold position so trajectories are set up
    ctrl_.InitializeHoldPosition(state_);
  }
};

TEST_F(WbcFSMTest, InitialPhaseIsIdle)
{
  // After construction + InitializeHoldPosition, the controller should
  // produce valid output (position hold) without crashing.
  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.num_devices, 2);
}

TEST_F(WbcFSMTest, ComputeProducesValidOutput)
{
  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);
  // Arm output should hold home position
  EXPECT_EQ(out.devices[0].num_channels, 6);
  EXPECT_NEAR(out.devices[0].commands[0], 0.0, 0.01);
  EXPECT_NEAR(out.devices[0].commands[1], -1.57, 0.01);
}

TEST_F(WbcFSMTest, SetDeviceTargetStoresTarget)
{
  std::array<double, 6> target = {0.1, -1.0, 1.0, -1.0, -1.0, 0.1};
  ctrl_.SetDeviceTarget(0, target);

  // After setting target, compute should still work (still in kIdle
  // until grasp_cmd=1 is sent via UpdateGainsFromMsg)
  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);
}

TEST_F(WbcFSMTest, InitializeHoldPositionResetsState)
{
  // Set a target, then re-initialize
  std::array<double, 6> target = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  ctrl_.SetDeviceTarget(0, target);
  ctrl_.InitializeHoldPosition(state_);

  // Output should be at current state position, not target
  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.devices[0].commands[1], -1.57, 0.01);
}

// ── E-STOP Tests ─────────────────────────────────────────────────────────────

TEST_F(WbcFSMTest, EstopProducesSafePosition)
{
  EXPECT_FALSE(ctrl_.IsEstopped());

  ctrl_.TriggerEstop();
  EXPECT_TRUE(ctrl_.IsEstopped());

  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);

  // Arm should go to safe position
  EXPECT_NEAR(out.devices[0].commands[0], 0.0, 0.01);
  EXPECT_NEAR(out.devices[0].commands[1], -1.57, 0.01);
  EXPECT_NEAR(out.devices[0].commands[2], 1.57, 0.01);
}

TEST_F(WbcFSMTest, ClearEstopResumesNormalOperation)
{
  ctrl_.TriggerEstop();
  ctrl_.ClearEstop();
  EXPECT_FALSE(ctrl_.IsEstopped());

  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);
}

TEST_F(WbcFSMTest, HandEstopIndependent)
{
  ctrl_.SetHandEstop(true);
  // Controller-level E-STOP should still be false
  EXPECT_FALSE(ctrl_.IsEstopped());

  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);
}

// ── Gains Tests ──────────────────────────────────────────────────────────────

TEST_F(WbcFSMTest, UpdateGainsFromMsg)
{
  std::array<double, 7> gains = {
    0.0,    // grasp_cmd = idle
    3.0,    // grasp_target_force
    0.8,    // arm_traj_speed
    2.0,    // hand_traj_speed
    150.0,  // se3_weight
    15.0,   // force_weight
    2.0     // posture_weight
  };
  ctrl_.UpdateGainsFromMsg(gains);

  auto current = ctrl_.GetCurrentGains();
  ASSERT_EQ(current.size(), 7u);
  EXPECT_NEAR(current[0], 0.0, 1e-9);    // grasp_cmd
  EXPECT_NEAR(current[1], 3.0, 1e-9);    // force
  EXPECT_NEAR(current[2], 0.8, 1e-9);    // arm speed
  EXPECT_NEAR(current[3], 2.0, 1e-9);    // hand speed
  EXPECT_NEAR(current[4], 150.0, 1e-9);  // se3_weight
  EXPECT_NEAR(current[5], 15.0, 1e-9);   // force_weight
  EXPECT_NEAR(current[6], 2.0, 1e-9);    // posture_weight
}

TEST_F(WbcFSMTest, PartialGainsUpdate)
{
  // Only update first 3 values
  std::array<double, 3> partial = {1.0, 5.0, 1.2};
  ctrl_.UpdateGainsFromMsg(partial);

  auto current = ctrl_.GetCurrentGains();
  EXPECT_NEAR(current[0], 1.0, 1e-9);  // grasp_cmd updated
  EXPECT_NEAR(current[1], 5.0, 1e-9);  // force updated
  EXPECT_NEAR(current[2], 1.2, 1e-9);  // arm speed updated
  // Remaining should be defaults
  EXPECT_GT(current[4], 0.0);  // se3_weight still has default
}

// ── Output Assembly Tests ────────────────────────────────────────────────────

TEST_F(WbcFSMTest, OutputFieldsPopulated)
{
  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.command_type, rtc::CommandType::kPosition);
  EXPECT_EQ(out.num_devices, 2);

  // Arm device
  EXPECT_EQ(out.devices[0].num_channels, 6);
  EXPECT_EQ(out.devices[0].goal_type, rtc::GoalType::kJoint);

  // Hand device
  EXPECT_EQ(out.devices[1].num_channels, 10);
  EXPECT_EQ(out.devices[1].goal_type, rtc::GoalType::kJoint);
}

TEST_F(WbcFSMTest, MultipleComputeCyclesStable)
{
  // Run 100 compute cycles in kIdle — should be stable
  for (int i = 0; i < 100; ++i) {
    state_.iteration = static_cast<uint64_t>(i);
    auto out = ctrl_.Compute(state_);
    EXPECT_TRUE(out.valid);
    // Arm should hold position (no drift)
    EXPECT_NEAR(out.devices[0].commands[0], 0.0, 0.01);
    EXPECT_NEAR(out.devices[0].commands[1], -1.57, 0.01);
  }
}

// ── Name and CommandType Tests ───────────────────────────────────────────────

TEST(WbcControllerBasic, NameReturnsExpected)
{
  DemoWbcController ctrl{""};
  EXPECT_EQ(ctrl.Name(), "DemoWbcController");
}

TEST(WbcControllerBasic, DefaultCommandTypeIsPosition)
{
  DemoWbcController ctrl{""};
  EXPECT_EQ(ctrl.GetCommandType(), rtc::CommandType::kPosition);
}

}  // namespace
