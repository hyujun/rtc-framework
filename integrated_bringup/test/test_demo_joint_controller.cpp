// ── DemoJointController grasp-detection unit tests ───────────────────────────
//
// Exercises the force-derived grasp detection, the fingertip-count derivation
// (num_inference_groups preferred, sensor-stride fallback), the ToF snapshot
// sensor-lane gate, and the grasp_controller_type=="none" no-op branch — all
// without a real URDF. The controller is built with an empty urdf_path; the
// arm_handle_ FK is guarded (mirrors DemoWbcController) so only the
// grasp/sensor logic runs. arm/task-space output is not asserted here.

#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/controllers/hand_sensor_layout.hpp"

#include <gtest/gtest.h>

#include <array>

namespace {

using integrated_bringup::DemoJointController;
using integrated_bringup::kHandInferenceValuesPerFingertipCapacity;
using integrated_bringup::kHandSensorValuesPerFingertipCapacity;
using rtc::ControllerOutput;
using rtc::ControllerState;

// Arm: 6 joints, hand: 10 joints — both valid, at zero unless overridden.
ControllerState MakeState(double dt = 0.002) {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = dt;
  state.iteration = 1;

  auto& dev0 = state.devices[0];
  dev0.num_channels = 6;
  dev0.valid = true;

  auto& dev1 = state.devices[1];
  dev1.num_channels = 10;
  dev1.valid = true;

  return state;
}

// Write a fingertip force (Fz) + raw native-contact slot into inference_data.
// Leaves the count fields (num_inference_groups / num_sensor_channels) to the
// caller so each test pins the exact stream shape it targets.
void SetFingertipForce(ControllerState& s, int f, float fz, float contact = 0.0f) {
  auto& dev1 = s.devices[1];
  dev1.inference_enable[static_cast<std::size_t>(f)] = true;
  const int base = f * static_cast<int>(kHandInferenceValuesPerFingertipCapacity);
  dev1.inference_data[static_cast<std::size_t>(base)] = contact;   // native contact (slot 0)
  dev1.inference_data[static_cast<std::size_t>(base + 1)] = 0.0f;  // Fx
  dev1.inference_data[static_cast<std::size_t>(base + 2)] = 0.0f;  // Fy
  dev1.inference_data[static_cast<std::size_t>(base + 3)] = fz;    // Fz
}

class JointGraspTest : public ::testing::Test {
 protected:
  DemoJointController ctrl_{""};
  ControllerState state_ = MakeState();

  void SetUp() override {
    // First Compute() runs the controller-internal self-init (fallback DoF from
    // device channel counts, hold-trajectory seeding).
    (void)ctrl_.Compute(state_);
  }
};

// ── Fingertip-count derivation + force-only detection ──────────────────────

// p1b real-hw shape: force-only inference lane (num_sensor_channels=0), a fixed
// native contact_flag=0, capability flags false. Grasp detection must fall back
// to |F| > grasp_force_threshold and still report contacts + a derived binary
// contact_flag.
TEST_F(JointGraspTest, ForceOnlyFallbackDetectsContactWithZeroContactFlag) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 5.0f, /*contact=*/0.0f);
  SetFingertipForce(state_, 1, 5.0f, /*contact=*/0.0f);
  (void)ctrl_.Compute(state_);

  const auto& gs = ctrl_.GetGraspStateForTesting();
  EXPECT_EQ(gs.num_fingertips, 2);
  EXPECT_EQ(gs.num_active_contacts, 2);
  EXPECT_TRUE(gs.grasp_detected);
  EXPECT_NEAR(gs.force_magnitude[0], 5.0f, 1e-4f);
  // Derived binary contact_flag (1.0) even though the raw native slot was 0.
  EXPECT_NEAR(gs.contact_flag[0], 1.0f, 1e-5f);
  EXPECT_NEAR(gs.contact_flag[1], 1.0f, 1e-5f);
}

// Legacy mock shape: no inference groups, fingertip count derived from the
// sensor channel stride. This path must keep working after the derivation
// switched to prefer num_inference_groups.
TEST_F(JointGraspTest, ChannelStrideFallbackStillWorks) {
  state_.devices[1].num_inference_groups = 0;
  state_.devices[1].num_sensor_channels =
      2 * static_cast<int>(kHandSensorValuesPerFingertipCapacity);
  SetFingertipForce(state_, 0, 5.0f);
  SetFingertipForce(state_, 1, 5.0f);
  (void)ctrl_.Compute(state_);

  const auto& gs = ctrl_.GetGraspStateForTesting();
  EXPECT_EQ(gs.num_fingertips, 2);
  EXPECT_EQ(gs.num_active_contacts, 2);
  EXPECT_TRUE(gs.grasp_detected);
}

// Neither lane populated → zero fingertips, no grasp, no crash.
TEST_F(JointGraspTest, ZeroChannelsZeroGroupsReportsZeroFingertips) {
  state_.devices[1].num_inference_groups = 0;
  state_.devices[1].num_sensor_channels = 0;
  (void)ctrl_.Compute(state_);

  const auto& gs = ctrl_.GetGraspStateForTesting();
  EXPECT_EQ(gs.num_fingertips, 0);
  EXPECT_EQ(gs.num_active_contacts, 0);
  EXPECT_FALSE(gs.grasp_detected);
}

// ── ToF snapshot sensor-lane gate ──────────────────────────────────────────

// The ToF snapshot is gated by the raw sensor-lane fingertip count, NOT the
// inference-group count, so a force-only stream never publishes a junk all-zero
// snapshot. (populated stays false regardless because hand_handle_ is null in
// this fixture; the gate counter is the observable contract.)
TEST_F(JointGraspTest, ToFSnapshotGateUsesSensorLaneCount) {
  state_.devices[1].num_inference_groups = 4;
  state_.devices[1].num_sensor_channels = 0;
  for (int f = 0; f < 4; ++f) {
    state_.devices[1].inference_enable[static_cast<std::size_t>(f)] = true;
  }
  (void)ctrl_.Compute(state_);

  EXPECT_EQ(ctrl_.GetGraspStateForTesting().num_fingertips, 4);
  EXPECT_EQ(ctrl_.GetNumSensorFingertipsForTesting(), 0);
  EXPECT_FALSE(ctrl_.GetToFSnapshotForTesting().populated);

  // A real sensor lane makes the gate counter track the channel stride.
  state_.devices[1].num_sensor_channels =
      4 * static_cast<int>(kHandSensorValuesPerFingertipCapacity);
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetNumSensorFingertipsForTesting(), 4);
}

// ── grasp_controller_type branch behaviour ─────────────────────────────────

// Drive the hand from 0.3 toward 0.8 with a strong contact present. With the
// hand device at 0.3 and 2 fingertips reporting 5 N (> threshold), the two hand
// modes must diverge: "none" lets the trajectory pass through; "contact_stop"
// freezes the hand at its current position.
namespace {
constexpr double kHandStart = 0.3;
constexpr double kHandGoal = 0.8;

void PrimeHandMotion(DemoJointController& ctrl, ControllerState& state) {
  for (int i = 0; i < 10; ++i) {
    state.devices[1].positions[static_cast<std::size_t>(i)] = kHandStart;
  }
  state.devices[1].num_inference_groups = 2;
  state.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state, 0, 5.0f);
  SetFingertipForce(state, 1, 5.0f);

  std::array<double, 10> tgt{};
  tgt.fill(kHandGoal);
  ctrl.SetDeviceTarget(1, tgt);
}

ControllerOutput RunHandTicks(DemoJointController& ctrl, ControllerState& state, int n) {
  ControllerOutput out{};
  for (int i = 0; i < n; ++i) {
    state.iteration = static_cast<uint64_t>(i + 2);
    out = ctrl.Compute(state);
  }
  return out;
}
}  // namespace

TEST_F(JointGraspTest, NoneTypeSkipsHandIntervention) {
  ctrl_.SetGraspControllerTypeForTesting("none");
  PrimeHandMotion(ctrl_, state_);
  // 300 ticks × 2 ms = 0.6 s > trajectory duration (~0.5 s): motion completes.
  auto out = RunHandTicks(ctrl_, state_, 300);

  // Contact is still observed (GraspState publishing is unaffected by "none").
  EXPECT_TRUE(ctrl_.GetGraspStateForTesting().grasp_detected);
  // Hand followed the trajectory well past the 0.3 start — no freeze.
  EXPECT_GT(out.devices[1].commands[0], 0.5);
}

TEST_F(JointGraspTest, ContactStopFreezesHandAtCurrentPosition) {
  // Default grasp_controller_type is "contact_stop" (control group).
  PrimeHandMotion(ctrl_, state_);
  auto out = RunHandTicks(ctrl_, state_, 300);

  EXPECT_TRUE(ctrl_.GetGraspStateForTesting().grasp_detected);
  // Hand frozen at the current actual position (0.3), not the 0.8 goal.
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-6);
}

}  // namespace
