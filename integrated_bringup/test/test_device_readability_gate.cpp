// ── F5 device-readability gate, at the binding (issue #236 S7b) ──────────────
//
// rtc_controller_interface/test/test_device_readability.cpp pins the base
// contract in isolation. This file pins that the shipped controllers actually
// OBEY it — the part the predicates cannot prove on their own: that the gate
// sits ahead of every read, that a silenced tick emits zero-length rather than
// nc0 zeros, that the hand keeps being commanded, and that the E-STOP lane
// answers the same way as the normal one.
//
// No URDF: every controller here is built with an empty urdf_path, so
// arm_handle_ is null and the arm-tip FK is skipped. That is deliberate twice
// over — it keeps the fixture independent of robot data, and it sidesteps the
// serial_6dof vacuity trap (§3.3 of rtc_controllers/docs/
// compliance-conventions.md: +Z-coaxial joints make gravity and TCP
// translation shape-invariant, so those assertions pass with the gate deleted).
// Everything asserted below is a channel count or a joint command.

#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "integrated_bringup/controllers/hand_sensor_layout.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstddef>
#include <map>
#include <string>

namespace {

using integrated_bringup::DemoJointController;
using integrated_bringup::DemoTaskController;
using integrated_bringup::DemoWbcController;
using rtc::ControllerState;

constexpr int kArmDof = 6;
constexpr int kHandChannels = 10;
constexpr double kDt = 0.002;

// Distinct, non-zero, and inside the ±2π default clamp — so "held at the
// measured value" is distinguishable from both "zero" and "any other channel".
double MeasuredArm(std::size_t i) {
  return 0.11 + 0.07 * static_cast<double>(i);
}

double MeasuredHand(std::size_t i) {
  return -0.23 - 0.05 * static_cast<double>(i);
}

// Safe position differs from every measurement, so an E-STOP ramp step is
// observable, and it is non-zero, so "ramping toward safe" is distinguishable
// from "ramping toward the origin" — the exact confusion #265 audit J5 found.
double SafeArm(std::size_t i) {
  return 1.30 - 0.09 * static_cast<double>(i);
}

// Minimal YAML DemoJointController::LoadConfig accepts: the required `arm_dof`
// key plus the two required sections (estop.arm_safe_position, fsm) and nothing
// else. No model_config, so no URDF is parsed. arm_dof_ comes from `arm_dof`
// (#340) — that is what makes a narrower device refusable. The safe-position
// sequence stays kArmDof wide because the E-STOP ramp assertions below read its
// VALUES; its width is now cross-checked against arm_dof rather than defining it.
std::string MinimalJointYaml() {
  std::string yaml = "arm_dof: " + std::to_string(kArmDof) + "\nestop:\n  arm_safe_position: [";
  for (int i = 0; i < kArmDof; ++i) {
    yaml += (i ? ", " : "") + std::to_string(SafeArm(static_cast<std::size_t>(i)));
  }
  yaml += "]\nfsm:\n  contact_stop_release_eps: 0.005\n  contact_stop_lpf_cutoff_hz: 20.0\n";
  return yaml + "command_type: \"position\"\n";
}

// `hand_channels` defaults to a full-width hand so every arm-axis test above is
// unaffected; the #291 section below varies it to exercise the secondary gate.
ControllerState MakeState(int arm_channels, int hand_channels = kHandChannels) {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = kDt;
  state.iteration = 1;

  auto& dev0 = state.devices[0];
  dev0.num_channels = arm_channels;
  dev0.valid = true;
  // ONLY the reported channels carry a measurement. The rest keep DeviceState's
  // zero-initialisation, which is what the hazard actually looks like: an
  // unreported joint reads back as a perfectly finite 0.0 and nothing faults.
  // Filling the whole 64-wide array instead would make every gate assertion
  // vacuous — the unread channels would hold real values and a controller that
  // ignored the gate would still produce the "correct" answer.
  for (std::size_t i = 0; i < static_cast<std::size_t>(arm_channels); ++i) {
    dev0.positions[i] = MeasuredArm(i);
  }

  auto& dev1 = state.devices[1];
  dev1.num_channels = hand_channels;
  dev1.valid = true;
  // Same rule as the arm above: ONLY the reported finger channels carry a
  // measurement, so an unreported one reads back as a finite 0.0 and the gate
  // assertions stay falsifiable.
  for (std::size_t i = 0; i < static_cast<std::size_t>(hand_channels); ++i) {
    dev1.positions[i] = MeasuredHand(i);
  }
  return state;
}

// A silenced tick must report THIS tick's measurement, not the last readable
// tick's reference. The fixture's measurements are otherwise identical across
// ticks, so every parked-position assertion shifts the arm first — the shift is
// what makes those assertions falsifiable.
constexpr double kArmShift = 0.031;

void ShiftArmMeasurements(ControllerState& state, int channels) {
  for (std::size_t i = 0; i < static_cast<std::size_t>(channels); ++i) {
    state.devices[0].positions[i] = MeasuredArm(i) + kArmShift;
  }
}

// arm_dof_ resolved from YAML (6), so a device narrower than 6 is the state the
// gate must refuse. Resolving it from the device instead — the arm_dof_ == 0
// fallback — would make every fixture trivially readable and the suite vacuous.
class DemoJointGateTest : public ::testing::Test {
 protected:
  DemoJointController ctrl_{""};

  void SetUp() override { ASSERT_NO_THROW(ctrl_.LoadConfig(YAML::Load(MinimalJointYaml()))); }

  // One readable tick, which is what latches the hold target from measurement.
  void SeedFromReadableDevice() {
    ControllerState seed = MakeState(kArmDof);
    const auto out = ctrl_.Compute(seed);
    ASSERT_EQ(out.devices[0].num_channels, kArmDof) << "fixture precondition: a full-width device "
                                                       "must NOT be gated";
  }
};

// ── The gate refuses, and the refusal is silence ─────────────────────────────

TEST_F(DemoJointGateTest, ANarrowDeviceSilencesTheArm) {
  SeedFromReadableDevice();

  ControllerState narrow = MakeState(kArmDof - 2);
  const auto out = ctrl_.Compute(narrow);

  // Zero-length, NOT four zeros. The rejected alternative is a real position
  // command whose unreported joints read 0 — "servo to the origin" (§3.7).
  EXPECT_EQ(out.devices[0].num_channels, 0);
  EXPECT_NE(out.devices[0].num_channels, kArmDof - 2)
      << "an nc0-length command is the answer #236 E-8 rejected";
}

TEST_F(DemoJointGateTest, AnUnreportedDeviceSilencesTheArm) {
  SeedFromReadableDevice();

  ControllerState invalid = MakeState(kArmDof);
  invalid.devices[0].valid = false;
  const auto out = ctrl_.Compute(invalid);

  EXPECT_EQ(out.devices[0].num_channels, 0);
}

TEST_F(DemoJointGateTest, TheHandKeepsBeingCommandedWhileTheArmIsSilent) {
  SeedFromReadableDevice();

  ControllerState narrow = MakeState(kArmDof - 2);
  const auto out = ctrl_.Compute(narrow);

  ASSERT_EQ(out.devices[0].num_channels, 0);
  // §3.7: a hand does not stop being commandable because the arm's state went
  // missing. Its self-init seeded the hold from measurement on the seed tick.
  ASSERT_EQ(out.devices[1].num_channels, kHandChannels);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kHandChannels); ++i) {
    EXPECT_NEAR(out.devices[1].commands[i], MeasuredHand(i), 1e-9) << "hand channel " << i;
  }
  EXPECT_EQ(out.num_devices, 2);
}

TEST_F(DemoJointGateTest, ASilencedTickReportsTheParkedPositionNotZeros) {
  constexpr int kNarrow = kArmDof - 2;
  SeedFromReadableDevice();

  ControllerState narrow = MakeState(kNarrow);
  // Move the arm since the seed tick. Without this the last readable tick's
  // reference and this tick's measurement are the same numbers, and the
  // assertion below cannot tell "parked at the measurement" from "replayed the
  // stale reference" — it would pass with the fill deleted.
  ShiftArmMeasurements(narrow, kNarrow);
  const auto out = ctrl_.Compute(narrow);

  ASSERT_EQ(out.devices[0].num_channels, 0);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kNarrow); ++i) {
    // Not robot_computed_ (the last readable tick's trajectory) and not 0 (a
    // row that reads as a command to the origin) — this tick's measurement.
    EXPECT_NEAR(out.devices[0].target_positions[i], MeasuredArm(i) + kArmShift, 1e-9)
        << "channel " << i;
    EXPECT_NEAR(out.devices[0].trajectory_positions[i], MeasuredArm(i) + kArmShift, 1e-9)
        << "channel " << i;
    EXPECT_DOUBLE_EQ(out.devices[0].trajectory_velocities[i], 0.0)
        << "a parked joint is not moving, channel " << i;
  }
}

TEST_F(DemoJointGateTest, ASilencedEstopTickAlsoReportsTheParkedPosition) {
  // The E-STOP lane runs no Fill*, so it owns this fill itself. Without it the
  // one lane an operator reads during an E-STOP incident is a row of zeros.
  constexpr int kNarrow = kArmDof - 2;
  SeedFromReadableDevice();
  ctrl_.TriggerEstop();

  ControllerState narrow = MakeState(kNarrow);
  // Move the arm since the seed tick. Without this the last readable tick's
  // reference and this tick's measurement are the same numbers, and the
  // assertion below cannot tell "parked at the measurement" from "replayed the
  // stale reference" — it would pass with the fill deleted.
  ShiftArmMeasurements(narrow, kNarrow);
  const auto out = ctrl_.Compute(narrow);

  ASSERT_EQ(out.devices[0].num_channels, 0);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kNarrow); ++i) {
    EXPECT_NEAR(out.devices[0].target_positions[i], MeasuredArm(i) + kArmShift, 1e-9)
        << "channel " << i;
    EXPECT_NEAR(out.devices[0].trajectory_positions[i], MeasuredArm(i) + kArmShift, 1e-9)
        << "channel " << i;
  }
}

TEST_F(DemoJointGateTest, TheEstopLaneAnswersTheSameWay) {
  SeedFromReadableDevice();
  ctrl_.TriggerEstop();

  ControllerState narrow = MakeState(kArmDof - 2);
  const auto out = ctrl_.Compute(narrow);

  // The E-8 change this slice was approved for: ComputeEstop used to emit an
  // nc0-length ramp built on unread joints. `ĝ(q)`-style holds and
  // `q + clamp(q_safe - q)` ramps are both only holds while q is measured.
  EXPECT_EQ(out.devices[0].num_channels, 0);
  ASSERT_EQ(out.devices[1].num_channels, kHandChannels)
      << "the hand still holds under E-STOP — the gate silences device 0 only";
}

// ── Self-init is deferred, not latched from unread state (audit J1) ──────────

TEST_F(DemoJointGateTest, SelfInitWaitsForAReadableDeviceInsteadOfSeedingZeros) {
  // First tick ever is narrow: the hold target must NOT latch here. It used to,
  // seeding arm_dof_ slots from dev0.positions without consulting nc0, so the
  // unreported joints latched at 0 — and the latch made it permanent.
  ControllerState narrow = MakeState(kArmDof - 2);
  const auto gated = ctrl_.Compute(narrow);
  ASSERT_EQ(gated.devices[0].num_channels, 0);

  // Device recovers. The hold must come from THIS tick's measurement.
  ControllerState full = MakeState(kArmDof);
  const auto out = ctrl_.Compute(full);

  ASSERT_EQ(out.devices[0].num_channels, kArmDof);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kArmDof); ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], MeasuredArm(i), 1e-9)
        << "joint " << i << " was seeded from the unreadable tick";
  }
}

// ── Channels past the model: bound + tail, not fresh zeros ───────────────────

TEST_F(DemoJointGateTest, ChannelsPastTheModelHoldTheirMeasuredPosition) {
  // nc0 > arm_dof_ is a NORMAL input — num_channels is the wire length and the
  // reorder map skips unreferenced names. The extra channels used to be left
  // unwritten, i.e. a fresh zero, which on a position lane is "go to the
  // origin" (#265 audit J3).
  SeedFromReadableDevice();

  ControllerState wide = MakeState(kArmDof + 4);
  const auto out = ctrl_.Compute(wide);

  ASSERT_EQ(out.devices[0].num_channels, kArmDof + 4);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kArmDof); ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], MeasuredArm(i), 1e-9) << "modelled joint " << i;
  }
  for (auto i = static_cast<std::size_t>(kArmDof); i < static_cast<std::size_t>(kArmDof + 4); ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], MeasuredArm(i), 1e-9) << "tail channel " << i;
    EXPECT_NE(out.devices[0].commands[i], 0.0) << "tail channel " << i << " went to the origin";
  }
}

TEST_F(DemoJointGateTest, TheEstopRampStaysInsideTheFixedWidthSafePositionArray) {
  // The J5 shape: safe_position_ is kDemoJointMaxArmDof wide and only
  // [0, arm_dof_) is configured, while nc0 may reach kMaxDeviceChannels. The
  // ramp loop used to run to nc0 — an out-of-range read past
  // kDemoJointMaxArmDof, and a ramp toward safe_position_[i] == 0.0 (the
  // origin) on every channel between arm_dof_ and it.
  constexpr int kWide = rtc::kMaxDeviceChannels;
  static_assert(kWide > integrated_bringup::kDemoJointMaxArmDof,
                "fixture is vacuous unless the device can out-report safe_position_");

  SeedFromReadableDevice();
  ctrl_.TriggerEstop();

  ControllerState wide = MakeState(kWide);
  const auto out = ctrl_.Compute(wide);

  ASSERT_EQ(out.devices[0].num_channels, kWide);
  // Modelled joints ramp toward their configured safe position.
  for (std::size_t i = 0; i < static_cast<std::size_t>(kArmDof); ++i) {
    const double cmd = out.devices[0].commands[i];
    const double step = (SafeArm(i) - MeasuredArm(i)) * kDt;  // |Δ| < the 2.0 rad/s default
    EXPECT_NEAR(cmd, MeasuredArm(i) + step, 1e-9) << "modelled joint " << i;
  }
  // Everything past the model holds where it is — including the channels that
  // used to be an out-of-range read.
  for (auto i = static_cast<std::size_t>(kArmDof); i < static_cast<std::size_t>(kWide); ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], MeasuredArm(i), 1e-9) << "tail channel " << i;
  }
  const auto past_safe_array =
      static_cast<std::size_t>(integrated_bringup::kDemoJointMaxArmDof) + 3;
  EXPECT_NEAR(out.devices[0].commands[past_safe_array], MeasuredArm(past_safe_array), 1e-9)
      << "channel " << past_safe_array << " is past safe_position_ entirely";
}

// ── The gate does not fire on states it must not refuse ──────────────────────

TEST_F(DemoJointGateTest, AWideDeviceIsNotTreatedAsAFault) {
  // Over-reporting must stay a normal input. A gate written as `nc0 != nv`
  // would turn every broad state topic into a permanent DEGRADE.
  SeedFromReadableDevice();

  ControllerState wide = MakeState(kArmDof + 20);
  const auto out = ctrl_.Compute(wide);

  EXPECT_EQ(out.devices[0].num_channels, kArmDof + 20);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kArmDof); ++i) {
    EXPECT_TRUE(std::isfinite(out.devices[0].commands[i]));
  }
}

// ── demo_task: the same contract, a different binding ────────────────────────
//
// DemoTaskController is registered for runtime switching, so its answer to an
// unreadable device has to match demo_joint's. Model-less on purpose (see the
// file header): the wire and E-STOP lanes below need no URDF, and
// ComputeControl early-returns without a registered TCP frame — which is
// precisely the state in which the tail policy has to be right anyway.

std::string MinimalTaskYaml() {
  std::string yaml = "arm_dof: " + std::to_string(kArmDof) + "\nestop:\n  arm_safe_position: [";
  for (int i = 0; i < kArmDof; ++i) {
    yaml += (i ? ", " : "") + std::to_string(SafeArm(static_cast<std::size_t>(i)));
  }
  yaml += "]\nfsm:\n  pi_rotation_margin: 0.15\n  contact_stop_release_eps: 0.005\n";
  return yaml;
}

class DemoTaskGateTest : public ::testing::Test {
 protected:
  DemoTaskController ctrl_{"", DemoTaskController::Gains{}};

  void SetUp() override { ASSERT_NO_THROW(ctrl_.LoadConfig(YAML::Load(MinimalTaskYaml()))); }
};

TEST_F(DemoTaskGateTest, ANarrowDeviceSilencesTheArm) {
  ControllerState narrow = MakeState(kArmDof - 2);
  const auto out = ctrl_.Compute(narrow);

  EXPECT_EQ(out.devices[0].num_channels, 0);
  ASSERT_EQ(out.devices[1].num_channels, kHandChannels)
      << "the hand keeps its own passthrough (§3.7)";
}

TEST_F(DemoTaskGateTest, TheHandKeepsBeingCommandedWhileTheArmIsSilent) {
  // The count alone does not say the hand is being driven — it says an array
  // was declared. This asserts the VALUES, because the way this binding failed
  // was that the hand lane lived inside ComputeControl and the arm gate's early
  // return froze hand_computed_ at its zero-init: device 1 got a full-length,
  // perfectly valid command to the hand's origin. That is the arm hazard moved
  // one device over, and §3.7's secondary passthrough forbids it.
  ControllerState narrow = MakeState(kArmDof - 2);
  const auto out = ctrl_.Compute(narrow);

  ASSERT_EQ(out.devices[0].num_channels, 0);
  ASSERT_EQ(out.devices[1].num_channels, kHandChannels);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kHandChannels); ++i) {
    EXPECT_NEAR(out.devices[1].commands[i], MeasuredHand(i), 1e-9) << "hand channel " << i;
    EXPECT_NE(out.devices[1].commands[i], 0.0) << "hand channel " << i << " commanded to origin";
  }
}

TEST_F(DemoTaskGateTest, ASilencedTickReportsTheParkedPositionNotZeros) {
  constexpr int kNarrow = kArmDof - 2;
  ControllerState narrow = MakeState(kNarrow);
  // Shifted so "this tick's measurement" is distinguishable from every other
  // number in the fixture — see ShiftArmMeasurements.
  ShiftArmMeasurements(narrow, kNarrow);
  const auto out = ctrl_.Compute(narrow);

  // The log POD is bounded by the DEVICE's channel count, not the output's, and
  // has no field for the emitted width — so leaving the reference lanes at
  // their fresh zero records the silenced tick as a command to the origin.
  ASSERT_EQ(out.devices[0].num_channels, 0);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kNarrow); ++i) {
    EXPECT_NEAR(out.devices[0].target_positions[i], MeasuredArm(i) + kArmShift, 1e-9)
        << "channel " << i;
    EXPECT_NEAR(out.devices[0].trajectory_positions[i], MeasuredArm(i) + kArmShift, 1e-9)
        << "channel " << i;
  }
}

TEST_F(DemoTaskGateTest, TheEstopLaneAnswersTheSameWay) {
  ctrl_.TriggerEstop();

  ControllerState narrow = MakeState(kArmDof - 2);
  const auto out = ctrl_.Compute(narrow);

  EXPECT_EQ(out.devices[0].num_channels, 0);
}

TEST_F(DemoTaskGateTest, TheEstopRampStaysInsideTheFixedWidthSafePositionArray) {
  // T8 — the J5 shape again, in the second binding. safe_position_ is
  // kDemoTaskMaxArmDof wide; nc0 can be twice that.
  constexpr int kWide = rtc::kMaxDeviceChannels;
  static_assert(kWide > integrated_bringup::kDemoTaskMaxArmDof,
                "fixture is vacuous unless the device can out-report safe_position_");
  ctrl_.TriggerEstop();

  ControllerState wide = MakeState(kWide);
  const auto out = ctrl_.Compute(wide);

  ASSERT_EQ(out.devices[0].num_channels, kWide);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kArmDof); ++i) {
    const double step = (SafeArm(i) - MeasuredArm(i)) * kDt;
    EXPECT_NEAR(out.devices[0].commands[i], MeasuredArm(i) + step, 1e-9) << "modelled joint " << i;
  }
  for (auto i = static_cast<std::size_t>(kArmDof); i < static_cast<std::size_t>(kWide); ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], MeasuredArm(i), 1e-9) << "tail channel " << i;
  }
}

// ── demo_wbc: ur5e_p1a / iiwa7_leap initial_controller ───────────────────────
//
// arm_dof_ here is resolved by the self-init fallback (min(nc0, kMaxArmDof))
// rather than YAML, which makes the first readable tick both the seed and the
// resolution. That is worth pinning on its own: a fallback derived FROM nc0
// must never be able to re-open the gate it just passed.

class DemoWbcGateTest : public ::testing::Test {
 protected:
  DemoWbcController ctrl_{""};

  void SeedFromReadableDevice() {
    ControllerState seed = MakeState(kArmDof);
    const auto out = ctrl_.Compute(seed);
    ASSERT_EQ(out.devices[0].num_channels, kArmDof)
        << "fixture precondition: a full-width device must NOT be gated";
  }
};

TEST_F(DemoWbcGateTest, ANarrowDeviceSilencesTheArm) {
  SeedFromReadableDevice();

  ControllerState narrow = MakeState(kArmDof - 2);
  const auto out = ctrl_.Compute(narrow);

  EXPECT_EQ(out.devices[0].num_channels, 0);
  ASSERT_EQ(out.devices[1].num_channels, kHandChannels)
      << "the hand keeps its own passthrough (§3.7)";
}

TEST_F(DemoWbcGateTest, TheHandKeepsBeingCommandedWhileTheArmIsSilent) {
  // Values, not just the count — same reason as the demo_task twin: a hand lane
  // that replays a frozen buffer declares the right length and commands the
  // origin. This binding holds the hand from measurement in DrainTargetSlot's
  // deferral, so the assertion pins the behaviour rather than fixing it.
  SeedFromReadableDevice();

  ControllerState narrow = MakeState(kArmDof - 2);
  const auto out = ctrl_.Compute(narrow);

  ASSERT_EQ(out.devices[0].num_channels, 0);
  ASSERT_EQ(out.devices[1].num_channels, kHandChannels);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kHandChannels); ++i) {
    EXPECT_NEAR(out.devices[1].commands[i], MeasuredHand(i), 1e-9) << "hand channel " << i;
  }
}

TEST_F(DemoWbcGateTest, ASilencedTickReportsTheParkedPositionNotZeros) {
  // Both output lanes, not one: this binding's publish fill was gated first and
  // the log fill kept mirroring robot_computed_, so the CSV and the topic
  // disagreed about the same tick. Both write out0, so one assertion covers the
  // agreement.
  constexpr int kNarrow = kArmDof - 2;
  SeedFromReadableDevice();

  ControllerState narrow = MakeState(kNarrow);
  // Move the arm since the seed tick. Without this the last readable tick's
  // reference and this tick's measurement are the same numbers, and the
  // assertion below cannot tell "parked at the measurement" from "replayed the
  // stale reference" — it would pass with the fill deleted.
  ShiftArmMeasurements(narrow, kNarrow);
  const auto out = ctrl_.Compute(narrow);

  ASSERT_EQ(out.devices[0].num_channels, 0);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kNarrow); ++i) {
    EXPECT_NEAR(out.devices[0].target_positions[i], MeasuredArm(i) + kArmShift, 1e-9)
        << "channel " << i;
    EXPECT_NEAR(out.devices[0].trajectory_positions[i], MeasuredArm(i) + kArmShift, 1e-9)
        << "channel " << i;
  }
}

TEST_F(DemoWbcGateTest, TheEstopLaneAnswersTheSameWay) {
  SeedFromReadableDevice();
  ctrl_.TriggerEstop();

  ControllerState narrow = MakeState(kArmDof - 2);
  const auto out = ctrl_.Compute(narrow);

  EXPECT_EQ(out.devices[0].num_channels, 0);
}

TEST_F(DemoWbcGateTest, ChannelsPastTheModelHoldTheirMeasuredPosition) {
  // W6 — robot_computed_ is only written [0, arm_dof_) by every FSM branch, so
  // the extra channels used to leave the wire as a fresh zero.
  SeedFromReadableDevice();

  ControllerState wide = MakeState(kArmDof + 6);
  const auto out = ctrl_.Compute(wide);

  ASSERT_EQ(out.devices[0].num_channels, kArmDof + 6);
  for (auto i = static_cast<std::size_t>(kArmDof); i < static_cast<std::size_t>(kArmDof + 6); ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], MeasuredArm(i), 1e-9) << "tail channel " << i;
    EXPECT_NE(out.devices[0].commands[i], 0.0) << "tail channel " << i << " went to the origin";
  }
}

TEST_F(DemoWbcGateTest, TheEstopTailHoldsInsteadOfCommandingTheOrigin) {
  // W7 — the ramp loop is arm_dof_ deep but num_channels was declared nc0, so
  // [arm_dof_, nc0) went out as a fresh zero under E-STOP.
  SeedFromReadableDevice();
  ctrl_.TriggerEstop();

  ControllerState wide = MakeState(kArmDof + 6);
  const auto out = ctrl_.Compute(wide);

  ASSERT_EQ(out.devices[0].num_channels, kArmDof + 6);
  for (auto i = static_cast<std::size_t>(kArmDof); i < static_cast<std::size_t>(kArmDof + 6); ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], MeasuredArm(i), 1e-9) << "tail channel " << i;
  }
  // safe_position_ is all-zero without LoadConfig, so the modelled joints ramp
  // toward the origin — which is exactly why the tail must NOT: an unmodelled
  // channel has no safe position to ramp to, only a place to stay.
  EXPECT_LT(out.devices[0].commands[0], MeasuredArm(0)) << "modelled joint 0 should be ramping";
}

TEST_F(DemoWbcGateTest, TheDofFallbackCannotReopenTheGateItJustPassed) {
  // arm_dof_ starts unresolved, so the first tick's gate degrades to a plain
  // validity check and the fallback then sets arm_dof_ = min(nc0, kMaxArmDof).
  // Bounded by nc0, so the resolved value can never exceed what was reported.
  ControllerState first = MakeState(kArmDof);
  const auto seeded = ctrl_.Compute(first);
  EXPECT_EQ(seeded.devices[0].num_channels, kArmDof);

  // ...and from now on a narrower device is refused.
  ControllerState narrow = MakeState(kArmDof - 1);
  EXPECT_EQ(ctrl_.Compute(narrow).devices[0].num_channels, 0);
}

// ── The same gate on the SECONDARY (hand) axis (issue #291) ──────────────────
//
// Everything above asks what the bindings do when the ARM goes narrow. This
// section asks the question #236 S7b never audited, because its axis was nc0:
// what happens when the HAND does. The hazard is structurally identical —
// hand_dof_ comes from the declared joint_state_names while num_channels is the
// wire length, so a start-up mismatch reads the unreported fingers back as a
// finite 0 — and the answer is the same one: silence device 1.
//
// FIXTURE PRECONDITION, and the reason for the machinery below. hand_dof_ is
// resolved in OnDeviceConfigsSet from the SECONDARY device's joint_state_names,
// and GetSecondaryDeviceName() reads topic_config_.groups[1] — so without a
// two-group `topics:` section there IS no secondary name, hand_dof_ stays 0,
// and the runtime fallback sets it to min(nc1, kMaxHandDof). That fallback is
// bounded BY nc1, so hand_readable_ could never be false and every assertion in
// this section would pass with the gate deleted. The two-group topics section
// plus the declared 10-name hand config is what makes these tests falsifiable.

constexpr int kHandDof = 10;    // declared width (joint_state_names.size())
constexpr int kNarrowHand = 6;  // what the wire actually reports

std::string TopicsTwoGroups() {
  return "topics:\n"
         "  arm:\n    subscribe:\n      - {topic: \"target\", role: \"target\"}\n"
         "  hand:\n    subscribe:\n      - {topic: \"hand_target\", role: \"target\"}\n";
}

std::map<std::string, rtc::DeviceNameConfig> MakeHandDeclaringConfigs() {
  std::map<std::string, rtc::DeviceNameConfig> configs;

  rtc::DeviceNameConfig arm;
  arm.device_name = "arm";
  for (int i = 0; i < kArmDof; ++i) {
    arm.joint_state_names.push_back("a" + std::to_string(i));
  }
  configs["arm"] = std::move(arm);

  // Ten declared finger joints — the ur5e_p1b shape (_base.yaml declares 10)
  // that #291 is about. The device below reports six of them.
  rtc::DeviceNameConfig hand;
  hand.device_name = "hand";
  for (int i = 0; i < kHandDof; ++i) {
    hand.joint_state_names.push_back("h" + std::to_string(i));
  }
  configs["hand"] = std::move(hand);
  return configs;
}

// Same falsifiability trick as ShiftArmMeasurements, on the hand: a silenced
// tick must report THIS tick's finger measurement, not the last readable
// tick's reference.
constexpr double kHandShift = 0.047;

void ShiftHandMeasurements(ControllerState& state, int channels) {
  for (std::size_t i = 0; i < static_cast<std::size_t>(channels); ++i) {
    state.devices[1].positions[i] = MeasuredHand(i) + kHandShift;
  }
}

class DemoJointHandGateTest : public ::testing::Test {
 protected:
  DemoJointController ctrl_{""};

  void SetUp() override {
    ASSERT_NO_THROW(ctrl_.LoadConfig(YAML::Load(MinimalJointYaml() + TopicsTwoGroups())));
    ctrl_.SetDeviceNameConfigs(MakeHandDeclaringConfigs());
  }

  // One tick with a hand that reports all ten declared channels.
  void SeedFromReadableHand() {
    ControllerState seed = MakeState(kArmDof, kHandDof);
    const auto out = ctrl_.Compute(seed);
    ASSERT_EQ(out.devices[1].num_channels, kHandDof)
        << "fixture precondition: a full-width hand must NOT be gated";
  }
};

TEST_F(DemoJointHandGateTest, ANarrowHandSilencesTheHand) {
  SeedFromReadableHand();

  ControllerState narrow = MakeState(kArmDof, kNarrowHand);
  const auto out = ctrl_.Compute(narrow);

  // Zero-length, NOT six zeros — and NOT a six-long command either, which is
  // the shape that looks harmless and is the one #291 rejects: the four
  // unreported fingers would have been read as 0 and ramped origin-ward.
  EXPECT_EQ(out.devices[1].num_channels, 0);
  EXPECT_NE(out.devices[1].num_channels, kNarrowHand);
}

TEST_F(DemoJointHandGateTest, TheArmKeepsBeingCommandedWhileTheHandIsSilent) {
  // §3.7's secondary-passthrough rule, read in the other direction: the hand's
  // own gate must not silence the ARM. This is the mirror of
  // TheHandKeepsBeingCommandedWhileTheArmIsSilent above, and the two together
  // are what make the two flags separate rather than one widened predicate.
  SeedFromReadableHand();

  ControllerState narrow = MakeState(kArmDof, kNarrowHand);
  const auto out = ctrl_.Compute(narrow);

  ASSERT_EQ(out.devices[1].num_channels, 0);
  ASSERT_EQ(out.devices[0].num_channels, kArmDof) << "the hand's gate silenced the arm";
  for (std::size_t i = 0; i < static_cast<std::size_t>(kArmDof); ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], MeasuredArm(i), 1e-9) << "arm joint " << i;
  }
}

TEST_F(DemoJointHandGateTest, ASilencedHandTickReportsTheParkedFingerPositionNotZeros) {
  SeedFromReadableHand();

  ControllerState narrow = MakeState(kArmDof, kNarrowHand);
  // Move the fingers since the seed tick, so "parked at this tick's
  // measurement" is distinguishable from "replayed the stale reference".
  ShiftHandMeasurements(narrow, kNarrowHand);
  const auto out = ctrl_.Compute(narrow);

  ASSERT_EQ(out.devices[1].num_channels, 0);
  // The device-state log POD is bounded by the DEVICE's channel count, not the
  // output's, so leaving these at their fresh zero records the silenced tick as
  // "commanded every finger to the origin" — the one misreading §3.7 exists to
  // prevent, on the one failure mode this gate is for.
  for (std::size_t i = 0; i < static_cast<std::size_t>(kNarrowHand); ++i) {
    EXPECT_NEAR(out.devices[1].target_positions[i], MeasuredHand(i) + kHandShift, 1e-9)
        << "finger " << i;
    EXPECT_NEAR(out.devices[1].trajectory_positions[i], MeasuredHand(i) + kHandShift, 1e-9)
        << "finger " << i;
    EXPECT_DOUBLE_EQ(out.devices[1].trajectory_velocities[i], 0.0)
        << "a parked finger is not moving, channel " << i;
  }
}

TEST_F(DemoJointHandGateTest, HandSelfInitWaitsForAReadableHandInsteadOfSeedingZeros) {
  // The site class the #291 issue table missed (it greps as `dev.positions`
  // inside a `devices[d]` loop, not `dev1.`). It is the worst of the A sites
  // because hand_target_initialized_ LATCHES: an ordinary read site re-poisons
  // every tick and therefore heals the tick the gate goes true, but a poisoned
  // seed here is never re-seeded.
  ControllerState narrow = MakeState(kArmDof, kNarrowHand);
  const auto gated = ctrl_.Compute(narrow);
  ASSERT_EQ(gated.devices[1].num_channels, 0);

  // Hand recovers and reports all ten. The hold must come from THIS tick.
  ControllerState full = MakeState(kArmDof, kHandDof);
  const auto out = ctrl_.Compute(full);

  ASSERT_EQ(out.devices[1].num_channels, kHandDof);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kHandDof); ++i) {
    // Channels [kNarrowHand, kHandDof) are the mutation signal: with the gate
    // removed the first tick latches them at 0 and no later tick re-seeds, so
    // they stay 0 here while the reported ones look fine.
    EXPECT_NEAR(out.devices[1].commands[i], MeasuredHand(i), 1e-9)
        << "finger " << i << " was seeded from the unreadable tick";
  }
}

// Write a fingertip force (Fz) into the inference lane, enough to read as a
// hard contact. Mirrors SetFingertipForce in test_demo_joint_controller.cpp.
void SetFingertipForce(ControllerState& s, int f, float fz) {
  auto& dev1 = s.devices[1];
  dev1.inference_enable[static_cast<std::size_t>(f)] = true;
  const int base =
      f * static_cast<int>(integrated_bringup::kHandInferenceValuesPerFingertipCapacity);
  dev1.inference_data[static_cast<std::size_t>(base)] = 0.0F;      // native contact slot
  dev1.inference_data[static_cast<std::size_t>(base + 1)] = 0.0F;  // Fx
  dev1.inference_data[static_cast<std::size_t>(base + 2)] = 0.0F;  // Fy
  dev1.inference_data[static_cast<std::size_t>(base + 3)] = fz;    // Fz
}

TEST_F(DemoJointHandGateTest, AnUnreadableHandCannotLatchContactStopFromPhantomFingers) {
  // The B-axis assertion, and the one the issue asks be made on the BRANCH
  // rather than on a number: the contact-stop path does not merely compute a
  // slightly-wrong hand position from unread fingers, it takes a different
  // path through the FSM. Two things in that block read unread state — the
  // hold LPF (an IIR, so the contamination outlives the tick) and the release
  // gate's `d = target - dev1.positions[gi]` at a MODEL index — and both feed
  // a latch.
  SeedFromReadableHand();

  // The hand goes narrow AND the fingertips report a hard contact. Without the
  // gate this tick latches contact_stop: contact_now is true, so
  // hand_hold_position_ is snapshotted from the LPF'd measurement — whose last
  // four fingers are the phantom 0 — and contact_latched_ makes it permanent.
  ControllerState narrow = MakeState(kArmDof, kNarrowHand);
  narrow.devices[1].num_inference_groups = 2;
  SetFingertipForce(narrow, 0, 5.0F);
  SetFingertipForce(narrow, 1, 5.0F);
  ASSERT_EQ(ctrl_.Compute(narrow).devices[1].num_channels, 0);

  // Hand readable again, contact gone. THE BRANCH is what differs: a latched
  // controller replays the frozen hold (contact_latched_ survives a contact
  // drop by design — only the release gate or an E-STOP clears it), while an
  // unlatched one lets the trajectory drive. Fingers [kNarrowHand, kHandDof)
  // are where the two answers separate, because those are the ones whose
  // frozen hold would have been built from a phantom 0.
  ControllerState full = MakeState(kArmDof, kHandDof);
  const auto out = ctrl_.Compute(full);

  ASSERT_EQ(out.devices[1].num_channels, kHandDof);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kHandDof); ++i) {
    EXPECT_NEAR(out.devices[1].commands[i], MeasuredHand(i), 1e-9)
        << "finger " << i << ": contact_stop latched on a tick that could not read the hand";
  }
}

TEST_F(DemoJointHandGateTest, AWideHandIsNotTreatedAsAFault) {
  // Over-reporting is a normal input on this axis too — a hand state topic
  // broader than the declared joint list must not become a permanent silence.
  SeedFromReadableHand();

  ControllerState wide = MakeState(kArmDof, kHandDof + 4);
  const auto out = ctrl_.Compute(wide);

  EXPECT_EQ(out.devices[1].num_channels, kHandDof + 4);
}

// ── demo_task: the same secondary-axis contract, a different binding ──────────

class DemoTaskHandGateTest : public ::testing::Test {
 protected:
  DemoTaskController ctrl_{"", DemoTaskController::Gains{}};

  void SetUp() override {
    ASSERT_NO_THROW(ctrl_.LoadConfig(YAML::Load(MinimalTaskYaml() + TopicsTwoGroups())));
    ctrl_.SetDeviceNameConfigs(MakeHandDeclaringConfigs());
  }

  void SeedFromReadableHand() {
    ControllerState seed = MakeState(kArmDof, kHandDof);
    const auto out = ctrl_.Compute(seed);
    ASSERT_EQ(out.devices[1].num_channels, kHandDof)
        << "fixture precondition: a full-width hand must NOT be gated";
  }
};

TEST_F(DemoTaskHandGateTest, ANarrowHandSilencesTheHand) {
  SeedFromReadableHand();

  ControllerState narrow = MakeState(kArmDof, kNarrowHand);
  const auto out = ctrl_.Compute(narrow);

  EXPECT_EQ(out.devices[1].num_channels, 0);
  EXPECT_EQ(out.devices[0].num_channels, kArmDof) << "the hand's gate silenced the arm";
}

TEST_F(DemoTaskHandGateTest, ASilencedHandTickReportsTheParkedFingerPositionNotZeros) {
  SeedFromReadableHand();

  ControllerState narrow = MakeState(kArmDof, kNarrowHand);
  ShiftHandMeasurements(narrow, kNarrowHand);
  const auto out = ctrl_.Compute(narrow);

  ASSERT_EQ(out.devices[1].num_channels, 0);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kNarrowHand); ++i) {
    EXPECT_NEAR(out.devices[1].target_positions[i], MeasuredHand(i) + kHandShift, 1e-9)
        << "finger " << i;
    EXPECT_NEAR(out.devices[1].trajectory_positions[i], MeasuredHand(i) + kHandShift, 1e-9)
        << "finger " << i;
  }
}

TEST_F(DemoTaskHandGateTest, HandSelfInitWaitsForAReadableHandInsteadOfSeedingZeros) {
  ControllerState narrow = MakeState(kArmDof, kNarrowHand);
  ASSERT_EQ(ctrl_.Compute(narrow).devices[1].num_channels, 0);

  ControllerState full = MakeState(kArmDof, kHandDof);
  const auto out = ctrl_.Compute(full);

  ASSERT_EQ(out.devices[1].num_channels, kHandDof);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kHandDof); ++i) {
    EXPECT_NEAR(out.devices[1].commands[i], MeasuredHand(i), 1e-9)
        << "finger " << i << " was seeded from the unreadable tick";
  }
}

// ── demo_wbc: the defer guard, whose hand half was still `valid` ──────────────

class DemoWbcHandGateTest : public ::testing::Test {
 protected:
  DemoWbcController ctrl_{""};

  void SetUp() override {
    ASSERT_NO_THROW(ctrl_.LoadConfig(YAML::Load(TopicsTwoGroups())));
    ctrl_.SetDeviceNameConfigs(MakeHandDeclaringConfigs());
  }
};

TEST_F(DemoWbcHandGateTest, TheDeferGuardWaitsForAReadableHandNotMerelyAValidOne) {
  // DrainTargetSlot's first-tick self-init defers until EVERY device can be
  // seeded from a real measurement. Its arm half became IsDeviceReadable in
  // #265 audit W1; its hand half stayed `valid`, so a partially-reporting hand
  // passed the guard and latched targets[1] — and through SeedHoldFromMeasured
  // the posture REFERENCE — with a phantom 0 per unreported finger, with
  // target_initialized_ true and no re-seed to follow.
  ControllerState narrow = MakeState(kArmDof, kNarrowHand);
  const auto gated = ctrl_.Compute(narrow);
  EXPECT_EQ(gated.devices[1].num_channels, 0);

  ControllerState full = MakeState(kArmDof, kHandDof);
  const auto out = ctrl_.Compute(full);

  ASSERT_EQ(out.devices[1].num_channels, kHandDof);
  for (std::size_t i = 0; i < static_cast<std::size_t>(kHandDof); ++i) {
    EXPECT_NEAR(out.devices[1].commands[i], MeasuredHand(i), 1e-9)
        << "finger " << i << " was seeded from the unreadable tick";
  }
}

}  // namespace
