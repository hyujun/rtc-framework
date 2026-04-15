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

// ── ReadState / Fingertip Sensor Parsing Tests ───────────────────────────────

// Populate fingertip f's force vector (Fx, Fy, Fz) into state.devices[1].
static void InjectFingertipForce(ControllerState & s, int f,
                                 float fx, float fy, float fz,
                                 float contact = 1.0f)
{
  auto & dev1 = s.devices[1];
  dev1.valid = true;
  // Ensure enough sensor channels for f+1 fingertips so parser reports it
  dev1.num_sensor_channels = std::max<int>(dev1.num_sensor_channels,
      (f + 1) * rtc::kSensorValuesPerFingertip);
  dev1.inference_enable[static_cast<std::size_t>(f)] = true;

  const int ft_base = f * rtc::kFTValuesPerFingertip;
  dev1.inference_data[static_cast<std::size_t>(ft_base)] = contact;
  dev1.inference_data[static_cast<std::size_t>(ft_base + 1)] = fx;
  dev1.inference_data[static_cast<std::size_t>(ft_base + 2)] = fy;
  dev1.inference_data[static_cast<std::size_t>(ft_base + 3)] = fz;
  dev1.inference_data[static_cast<std::size_t>(ft_base + 4)] = 0.0f;
  dev1.inference_data[static_cast<std::size_t>(ft_base + 5)] = 0.0f;
  dev1.inference_data[static_cast<std::size_t>(ft_base + 6)] = 0.0f;
}

TEST_F(WbcFSMTest, ReadStateParsesFingertipForces)
{
  // Inject a known force on fingertip 0: |F| = 5.0
  InjectFingertipForce(state_, 0, 3.0f, 4.0f, 0.0f, /*contact=*/1.0f);
  // Trigger ReadState via Compute
  (void)ctrl_.Compute(state_);

  EXPECT_GE(ctrl_.GetNumActiveFingertipsForTesting(), 1);
  auto r0 = ctrl_.GetFingertipReportForTesting(0);
  EXPECT_TRUE(r0.valid);
  EXPECT_NEAR(r0.force_magnitude, 5.0f, 1e-4f);
  EXPECT_NEAR(r0.contact_flag, 1.0f, 1e-6f);
  // First tick: force_rate initializes to 0 (prev=0, but rate set to 0 on init)
  EXPECT_NEAR(r0.force_rate, 0.0f, 1e-6f);
}

TEST_F(WbcFSMTest, ReadStateComputesForceRate)
{
  InjectFingertipForce(state_, 0, 0.0f, 0.0f, 0.0f);
  (void)ctrl_.Compute(state_);  // tick 1: prev=0, rate=0, initialized=true
  (void)ctrl_.Compute(state_);  // tick 2: prev=0, current=0, rate=0

  // Step up force to 1.0 N within one dt (0.002s) → raw rate = 500 N/s
  // With EMA alpha=0.1, smoothed rate ≈ 50 N/s
  InjectFingertipForce(state_, 0, 1.0f, 0.0f, 0.0f);
  (void)ctrl_.Compute(state_);
  auto r = ctrl_.GetFingertipReportForTesting(0);
  EXPECT_TRUE(r.valid);
  EXPECT_NEAR(r.force_magnitude, 1.0f, 1e-4f);
  EXPECT_GT(r.force_rate, 10.0f);   // smoothed, well above noise
  EXPECT_LT(r.force_rate, 100.0f);  // bounded by EMA
}

// ── Phase Transition Tests (kClosure / kHold / kRetreat / kRelease) ─────────

TEST_F(WbcFSMTest, ClosureToHoldOnSufficientContacts)
{
  // Force phase = kClosure directly
  ctrl_.ForcePhaseForTesting(WbcPhase::kClosure);
  // Inject forces on fingertips 0, 1 above 0.2 N threshold (default)
  InjectFingertipForce(state_, 0, 0.0f, 0.0f, 0.5f);
  InjectFingertipForce(state_, 1, 0.0f, 0.0f, 0.5f);
  ctrl_.SetGraspCmdForTesting(1);  // not abort
  (void)ctrl_.Compute(state_);      // parses + UpdatePhase → kHold
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kHold);
}

TEST_F(WbcFSMTest, ClosureStaysOnInsufficientContacts)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kClosure);
  // Only one contact → below min_contacts_for_hold (default 2)
  InjectFingertipForce(state_, 0, 0.0f, 0.0f, 0.5f);
  ctrl_.SetGraspCmdForTesting(1);
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kClosure);
}

TEST_F(WbcFSMTest, ClosureAbortsOnGraspCmdZero)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kClosure);
  ctrl_.SetGraspCmdForTesting(0);
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kIdle);
}

TEST_F(WbcFSMTest, HoldTransitionsToRetreatOnCmd2)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kHold);
  ctrl_.SetGraspCmdForTesting(2);
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kRetreat);
}

TEST_F(WbcFSMTest, HoldTransitionsToFallbackOnSlip)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kHold);
  ctrl_.SetGraspCmdForTesting(1);
  // Step force up fast: 0 → 1 → 5 over two ticks at dt=0.002
  // df/dt ≈ 2000 N/s, smoothed rate crosses 5 N/s threshold quickly
  InjectFingertipForce(state_, 0, 0.0f, 0.0f, 0.0f);
  (void)ctrl_.Compute(state_);
  InjectFingertipForce(state_, 0, 0.0f, 0.0f, 5.0f);
  (void)ctrl_.Compute(state_);
  InjectFingertipForce(state_, 0, 0.0f, 0.0f, 10.0f);
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kFallback);
}

TEST_F(WbcFSMTest, RetreatProducesValidOutput)
{
  // Set approach target then force straight to kRetreat
  std::array<double, 6> target = {0.1, -1.0, 1.0, -1.0, -1.0, 0.1};
  ctrl_.SetDeviceTarget(0, target);
  ctrl_.ForcePhaseForTesting(WbcPhase::kRetreat);
  ctrl_.SetGraspCmdForTesting(2);
  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kRetreat);
}

TEST_F(WbcFSMTest, ReleaseReturnsToIdleOnTrajectoryComplete)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kRelease);
  // Run many ticks so hand trajectory completes
  for (int i = 0; i < 1000; ++i) {
    state_.iteration = static_cast<uint64_t>(i);
    (void)ctrl_.Compute(state_);
    if (ctrl_.GetPhaseForTesting() == WbcPhase::kIdle) { break; }
  }
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kIdle);
}

TEST_F(WbcFSMTest, ReadStateHandlesDisabledFingertip)
{
  // inference_enable=false for fingertip 0
  state_.devices[1].num_sensor_channels =
      1 * rtc::kSensorValuesPerFingertip;
  state_.devices[1].inference_enable[0] = false;
  (void)ctrl_.Compute(state_);

  auto r = ctrl_.GetFingertipReportForTesting(0);
  EXPECT_FALSE(r.valid);
  EXPECT_NEAR(r.force_magnitude, 0.0f, 1e-6f);
}

// Inject displacement (deformation magnitude vector) on fingertip f.
static void InjectFingertipDisplacement(ControllerState & s, int f,
                                        float dx, float dy, float dz)
{
  auto & dev1 = s.devices[1];
  dev1.valid = true;
  dev1.num_sensor_channels = std::max<int>(dev1.num_sensor_channels,
      (f + 1) * rtc::kSensorValuesPerFingertip);
  dev1.inference_enable[static_cast<std::size_t>(f)] = true;

  const int ft_base = f * rtc::kFTValuesPerFingertip;
  dev1.inference_data[static_cast<std::size_t>(ft_base + 4)] = dx;
  dev1.inference_data[static_cast<std::size_t>(ft_base + 5)] = dy;
  dev1.inference_data[static_cast<std::size_t>(ft_base + 6)] = dz;
}

// ── Phase 4 Comprehensive: Deformation / Abort / Fallback Recovery ──────────

TEST_F(WbcFSMTest, HoldTransitionsToFallbackOnDeformation)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kHold);
  ctrl_.SetGraspCmdForTesting(1);
  // Steady force (no slip) but excessive displacement (> 0.015 m default)
  InjectFingertipForce(state_, 0, 0.0f, 0.0f, 1.0f);
  InjectFingertipDisplacement(state_, 0, 0.0f, 0.0f, 0.05f);
  // Two ticks to get past force_rate init
  (void)ctrl_.Compute(state_);
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kFallback);
}

TEST_F(WbcFSMTest, HoldStaysHoldUnderNormalContact)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kHold);
  ctrl_.SetGraspCmdForTesting(1);
  // Steady, low-force, no displacement: should not trigger anomaly
  InjectFingertipForce(state_, 0, 0.0f, 0.0f, 1.0f);
  for (int i = 0; i < 50; ++i) {
    (void)ctrl_.Compute(state_);
  }
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kHold);
}

TEST_F(WbcFSMTest, HoldAbortsOnGraspCmdZero)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kHold);
  ctrl_.SetGraspCmdForTesting(0);
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kIdle);
}

TEST_F(WbcFSMTest, RetreatAbortsOnGraspCmdZero)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kRetreat);
  ctrl_.SetGraspCmdForTesting(0);
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kIdle);
}

TEST_F(WbcFSMTest, RetreatTransitionsToReleaseOnTrajectoryComplete)
{
  std::array<double, 6> target = {0.05, -1.5, 1.5, -1.5, -1.5, 0.05};
  ctrl_.SetDeviceTarget(0, target);
  ctrl_.ForcePhaseForTesting(WbcPhase::kRetreat);
  ctrl_.SetGraspCmdForTesting(2);
  // Run enough ticks for trajectory to complete (default arm_traj_speed=0.5,
  // small delta → ~few seconds at dt=0.002 = ~2000 ticks)
  bool reached_release = false;
  for (int i = 0; i < 5000; ++i) {
    (void)ctrl_.Compute(state_);
    if (ctrl_.GetPhaseForTesting() == WbcPhase::kRelease) {
      reached_release = true;
      break;
    }
  }
  EXPECT_TRUE(reached_release);
}

TEST_F(WbcFSMTest, FallbackExitsOnlyOnGraspCmdZero)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kFallback);
  // cmd=1 should NOT recover
  ctrl_.SetGraspCmdForTesting(1);
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kFallback);
  // cmd=2 should NOT recover
  ctrl_.SetGraspCmdForTesting(2);
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kFallback);
  // cmd=0 → kIdle
  ctrl_.SetGraspCmdForTesting(0);
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kIdle);
}

TEST_F(WbcFSMTest, FallbackProducesSafeOutput)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kFallback);
  ctrl_.SetGraspCmdForTesting(1);
  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);
  // Hand commands should not be NaN
  for (std::size_t i = 0; i < 10; ++i) {
    EXPECT_FALSE(std::isnan(out.devices[1].commands[i]));
  }
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FALSE(std::isnan(out.devices[0].commands[i]));
  }
}

// ── ReadState Boundary / Bounds Tests ──────────────────────────────────────

TEST_F(WbcFSMTest, ReadStateZeroSensorChannelsReportsZeroFingertips)
{
  state_.devices[1].num_sensor_channels = 0;
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetNumActiveFingertipsForTesting(), 0);
}

TEST_F(WbcFSMTest, GetFingertipReportOutOfBoundsReturnsDefault)
{
  auto r_low  = ctrl_.GetFingertipReportForTesting(-1);
  auto r_high = ctrl_.GetFingertipReportForTesting(
      static_cast<int>(rtc::kMaxFingertips) + 5);
  EXPECT_FALSE(r_low.valid);
  EXPECT_FALSE(r_high.valid);
  EXPECT_NEAR(r_low.force_magnitude,  0.0f, 1e-6f);
  EXPECT_NEAR(r_high.force_magnitude, 0.0f, 1e-6f);
}

TEST_F(WbcFSMTest, NumActiveFingertipsScalesWithSensorChannels)
{
  // 3 fingertips' worth of channels
  state_.devices[1].num_sensor_channels = 3 * rtc::kSensorValuesPerFingertip;
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetNumActiveFingertipsForTesting(), 3);
  // Increase to 5
  state_.devices[1].num_sensor_channels = 5 * rtc::kSensorValuesPerFingertip;
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetNumActiveFingertipsForTesting(), 5);
}

TEST_F(WbcFSMTest, ContactFlagPropagatesToReport)
{
  InjectFingertipForce(state_, 2, 0.0f, 0.0f, 0.5f, /*contact=*/0.85f);
  (void)ctrl_.Compute(state_);
  auto r = ctrl_.GetFingertipReportForTesting(2);
  EXPECT_TRUE(r.valid);
  EXPECT_NEAR(r.contact_flag, 0.85f, 1e-5f);
}

TEST_F(WbcFSMTest, ForceRateEMASmoothsImpulse)
{
  // Inject sudden force step then drop to zero — EMA should produce
  // intermediate values and decay back below threshold given no further change
  InjectFingertipForce(state_, 0, 0.0f, 0.0f, 0.0f);
  (void)ctrl_.Compute(state_);
  InjectFingertipForce(state_, 0, 0.0f, 0.0f, 2.0f);
  (void)ctrl_.Compute(state_);
  const float rate_after_step =
      ctrl_.GetFingertipReportForTesting(0).force_rate;
  EXPECT_GT(rate_after_step, 0.0f);

  // Hold force constant for many ticks → rate should decay toward 0
  for (int i = 0; i < 100; ++i) {
    (void)ctrl_.Compute(state_);
  }
  const float rate_after_decay =
      ctrl_.GetFingertipReportForTesting(0).force_rate;
  EXPECT_LT(std::abs(rate_after_decay), std::abs(rate_after_step));
  EXPECT_LT(std::abs(rate_after_decay), 0.5f);
}

TEST_F(WbcFSMTest, PartialContactsBelowThresholdDoNotTrigger)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kClosure);
  ctrl_.SetGraspCmdForTesting(1);
  // Two fingertips but force below 0.2 N threshold
  InjectFingertipForce(state_, 0, 0.0f, 0.0f, 0.05f);
  InjectFingertipForce(state_, 1, 0.0f, 0.0f, 0.10f);
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kClosure);
}

TEST_F(WbcFSMTest, EstopDuringHoldProducesSafeAndCanResume)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kHold);
  ctrl_.SetGraspCmdForTesting(1);
  ctrl_.TriggerEstop();
  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);
  EXPECT_NEAR(out.devices[0].commands[1], -1.57, 0.01);  // safe pose

  ctrl_.ClearEstop();
  // After clear + InitializeHoldPosition phase resets to kIdle
  ctrl_.InitializeHoldPosition(state_);
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kIdle);
}

TEST_F(WbcFSMTest, CommandTypeStableAcrossPhases)
{
  const auto baseline = ctrl_.GetCommandType();
  for (auto p : {WbcPhase::kIdle, WbcPhase::kApproach,
                 WbcPhase::kRetreat, WbcPhase::kRelease,
                 WbcPhase::kFallback}) {
    ctrl_.ForcePhaseForTesting(p);
    auto out = ctrl_.Compute(state_);
    EXPECT_EQ(out.command_type, baseline)
        << "phase=" << static_cast<int>(p);
  }
}

TEST_F(WbcFSMTest, GraspCmdRoundTripPreservesValueAcrossUpdate)
{
  ctrl_.SetGraspCmdForTesting(2);
  auto current = ctrl_.GetCurrentGains();
  EXPECT_NEAR(current[0], 2.0, 1e-9);
  ctrl_.SetGraspCmdForTesting(1);
  current = ctrl_.GetCurrentGains();
  EXPECT_NEAR(current[0], 1.0, 1e-9);
}

TEST_F(WbcFSMTest, ApproachSavesQApproachStartForRetreat)
{
  // Set an approach target, transition idle -> approach (UpdatePhase path)
  std::array<double, 6> target = {0.5, -1.0, 1.0, -1.0, -1.0, 0.5};
  ctrl_.SetDeviceTarget(0, target);
  ctrl_.SetGraspCmdForTesting(1);
  // First Compute should trigger UpdatePhase: kIdle -> kApproach,
  // OnPhaseEnter saves q_approach_start_ from current state positions.
  (void)ctrl_.Compute(state_);
  // State positions are the home pose [-1.57, ...] — verify by forcing
  // kRetreat which builds trajectory from current → q_approach_start_.
  // We can't read q_approach_start_ directly, but we can verify Retreat
  // produces valid output (regression: would crash if unsaved).
  ctrl_.ForcePhaseForTesting(WbcPhase::kRetreat);
  ctrl_.SetGraspCmdForTesting(2);
  auto out = ctrl_.Compute(state_);
  EXPECT_TRUE(out.valid);
}

TEST_F(WbcFSMTest, MultipleComputeInClosureRemainsStable)
{
  ctrl_.ForcePhaseForTesting(WbcPhase::kClosure);
  ctrl_.SetGraspCmdForTesting(1);
  // No contacts → stays in closure, no NaN, no phase drift
  for (int i = 0; i < 100; ++i) {
    auto out = ctrl_.Compute(state_);
    EXPECT_TRUE(out.valid);
    for (std::size_t j = 0; j < 6; ++j) {
      EXPECT_FALSE(std::isnan(out.devices[0].commands[j]));
    }
  }
  EXPECT_EQ(ctrl_.GetPhaseForTesting(), WbcPhase::kClosure);
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
