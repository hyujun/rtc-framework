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

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstddef>

namespace {

using integrated_bringup::DemoJointController;
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

// Minimal YAML DemoJointController::LoadConfig accepts: the two required
// sections (estop.arm_safe_position, fsm) and nothing else. No model_config, so
// no URDF is parsed. arm_dof_ is the safe-position length — that is what makes
// a narrower device refusable.
std::string MinimalJointYaml() {
  std::string yaml = "estop:\n  arm_safe_position: [";
  for (int i = 0; i < kArmDof; ++i) {
    yaml += (i ? ", " : "") + std::to_string(SafeArm(static_cast<std::size_t>(i)));
  }
  yaml += "]\nfsm:\n  contact_stop_release_eps: 0.005\n  contact_stop_lpf_cutoff_hz: 20.0\n";
  return yaml + "command_type: \"position\"\n";
}

ControllerState MakeState(int arm_channels) {
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
  dev1.num_channels = kHandChannels;
  dev1.valid = true;
  for (std::size_t i = 0; i < static_cast<std::size_t>(kHandChannels); ++i) {
    dev1.positions[i] = MeasuredHand(i);
  }
  return state;
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

}  // namespace
