// Contract tests for the base F5 device-readability gate (issue #236 S7b).
//
// The contract itself is §3.7 of rtc_controllers/docs/compliance-conventions.md
// and decision B of issue #265. This file pins the parts of it that are
// expressible without a robot: which states the gate refuses, what the refusal
// looks like on the wire, why the OOB bound is NOT a substitute for the gate,
// and how the tail of an over-reporting device is filled.
//
// No URDF fixture here on purpose. The gate is a predicate over counts, so a
// model would only add the serial_6dof vacuity trap (§3.3: every joint is
// +Z-coaxial, so ĝ(q) ≡ 0 and TCP translation is shape-invariant — a gravity
// assertion passes with the gate deleted). Behaviour that genuinely needs FK
// is pinned against a real controller in integrated_bringup.

#include "rtc_controller_interface/device_readability.hpp"

#include <gtest/gtest.h>

#include <array>
#include <span>
#include <vector>

using rtc::CommandType;
using rtc::DeviceOutput;
using rtc::DeviceState;
using rtc::FillCommandTail;
using rtc::IsDeviceReadable;
using rtc::kMaxDeviceChannels;
using rtc::ModelChannelBound;
using rtc::SilenceDeviceOutput;

namespace {

// A device that reported `n` channels, each holding a distinguishable value.
// Slots past `n` keep DeviceState's zero-initialisation — which is precisely
// the hazard the gate exists for: they read as a perfectly finite 0.0.
DeviceState MakeDevice(int n, bool valid = true) {
  DeviceState dev;
  dev.valid = valid;
  dev.num_channels = n;
  for (int i = 0; i < n && i < static_cast<int>(kMaxDeviceChannels); ++i) {
    dev.positions[static_cast<std::size_t>(i)] = 1.0 + static_cast<double>(i);
  }
  return dev;
}

// ── IsDeviceReadable — the gate ──────────────────────────────────────────────

TEST(IsDeviceReadableTest, RefusesADeviceThatHasNotReported) {
  const DeviceState dev = MakeDevice(0, /*valid=*/false);
  EXPECT_FALSE(IsDeviceReadable(dev, 6));
}

TEST(IsDeviceReadableTest, RefusesADeviceNarrowerThanTheModel) {
  // The only condition that actually fires in production (#265: `valid` is a
  // latching flag and a mid-run dropout stalls the stamp instead) — a startup
  // configuration mismatch.
  EXPECT_FALSE(IsDeviceReadable(MakeDevice(5), 6));
  EXPECT_FALSE(IsDeviceReadable(MakeDevice(1), 6));
}

TEST(IsDeviceReadableTest, AcceptsExactWidthAndOverReporting) {
  // Over-reporting is a NORMAL input, not a fault: num_channels is the wire
  // length and BuildJointStateReorder is built to skip unreferenced names. The
  // gate must not turn a broad state topic into a permanent DEGRADE.
  EXPECT_TRUE(IsDeviceReadable(MakeDevice(6), 6));
  EXPECT_TRUE(IsDeviceReadable(MakeDevice(16), 6));
  EXPECT_TRUE(IsDeviceReadable(MakeDevice(static_cast<int>(kMaxDeviceChannels)), 6));
}

TEST(IsDeviceReadableTest, UnresolvedModelDimDegradesToAValidityCheck) {
  // A controller whose runtime DOF is not resolved yet (unit fixtures that
  // bypass YAML, or the first tick before self-init) knows of nothing missing.
  EXPECT_TRUE(IsDeviceReadable(MakeDevice(3), 0));
  EXPECT_FALSE(IsDeviceReadable(MakeDevice(3, /*valid=*/false), 0));
}

// ── ModelChannelBound — OOB defence only ─────────────────────────────────────

TEST(ModelChannelBoundTest, ClampsAnOverReportingDeviceToTheModelWidth) {
  EXPECT_EQ(ModelChannelBound(16, 6), 6);
  EXPECT_EQ(ModelChannelBound(6, 6), 6);
}

TEST(ModelChannelBoundTest, ClampsToWhatTheDeviceActuallyReported) {
  EXPECT_EQ(ModelChannelBound(4, 6), 4);
}

TEST(ModelChannelBoundTest, ClampsNegativeInputsToZero) {
  // The result is used as a loop bound and as a std::span length, where a
  // negative int would convert to an enormous std::size_t.
  EXPECT_EQ(ModelChannelBound(-1, 6), 0);
  EXPECT_EQ(ModelChannelBound(6, -1), 0);
}

TEST(ModelChannelBoundTest, BoundsAFixedWidthBufferAgainstAFullWidthDevice) {
  // The J5/T8 shape: a 32-wide safe_position_ indexed by a device that may
  // report up to kMaxDeviceChannels (64). Without the bound this is an
  // out-of-range read on every channel past 32.
  constexpr int kFixedWidth = 32;
  const int nc0 = static_cast<int>(kMaxDeviceChannels);
  ASSERT_GT(nc0, kFixedWidth) << "fixture is vacuous unless the device can out-report the buffer";

  std::array<double, kFixedWidth> safe_position{};
  const int n = ModelChannelBound(nc0, kFixedWidth);
  EXPECT_EQ(n, kFixedWidth);
  EXPECT_LE(static_cast<std::size_t>(n), safe_position.size());
}

// ── Role separation — decision B pinned in code ──────────────────────────────

TEST(GateVsBoundTest, BoundAloneLeavesAPartiallyZeroConfiguration) {
  // The claim decision B rests on, made falsifiable: for a device NARROWER
  // than the model, applying the bound to a scatter into a PERSISTENT buffer
  // produces byte-for-byte the same configuration as reading the unreported
  // channels outright. The bound removed the crash and kept the hazard.
  constexpr int kNv = 6;
  const DeviceState dev = MakeDevice(4);

  // What ExtractFullState / CopyToEigen do: scatter min(nc0, nv) entries into a
  // buffer that persists across ticks (zero on the first one).
  std::array<double, kNv> bounded{};
  const int n = ModelChannelBound(dev.num_channels, kNv);
  ASSERT_EQ(n, 4);
  for (int i = 0; i < n; ++i) {
    bounded[static_cast<std::size_t>(i)] = dev.positions[static_cast<std::size_t>(i)];
  }

  // What an unbounded read would have produced: DeviceState::positions is
  // zero-filled past num_channels, so the unreported joints read as 0.0.
  std::array<double, kNv> unbounded{};
  for (int i = 0; i < kNv; ++i) {
    unbounded[static_cast<std::size_t>(i)] = dev.positions[static_cast<std::size_t>(i)];
  }

  EXPECT_EQ(bounded, unbounded) << "if these ever differ, the bound really did remove the hazard "
                                   "and decision B needs revisiting";
  EXPECT_EQ(bounded[4], 0.0);
  EXPECT_EQ(bounded[5], 0.0);

  // ...and the gate is the only thing in this header that refuses that state.
  EXPECT_FALSE(IsDeviceReadable(dev, kNv));
}

TEST(GateVsBoundTest, TheTwoPredicatesDisagreeOnExactlyTheDangerousState) {
  // Misusing the bound AS a gate reads as "nothing was truncated, so we are
  // fine". That test passes on the one state the gate refuses, which is why
  // the two must not share a name.
  constexpr int kNv = 6;

  const DeviceState narrow = MakeDevice(4);
  EXPECT_EQ(ModelChannelBound(narrow.num_channels, kNv), narrow.num_channels)
      << "bound is a no-op here — it truncates nothing and reports nothing wrong";
  EXPECT_FALSE(IsDeviceReadable(narrow, kNv));

  const DeviceState wide = MakeDevice(16);
  EXPECT_LT(ModelChannelBound(wide.num_channels, kNv), wide.num_channels)
      << "bound DOES truncate here — the case it is actually for";
  EXPECT_TRUE(IsDeviceReadable(wide, kNv)) << "and this state is perfectly usable";
}

// ── SilenceDeviceOutput — the answer to a false gate ─────────────────────────

TEST(SilenceDeviceOutputTest, ZeroLengthIsDistinctFromAZeroCommand) {
  DeviceOutput out;
  out.num_channels = 6;
  for (std::size_t i = 0; i < 6; ++i) {
    out.commands[i] = 0.0;  // the REJECTED alternative: nc0 real zeros
  }

  SilenceDeviceOutput(out);

  // A backend loops to min(num_channels, values.size()), so zero-length is
  // "no update" while six zeros are six real 0 N·m / 0 rad commands. The two
  // are only distinguishable by this count.
  EXPECT_EQ(out.num_channels, 0);
}

TEST(SilenceDeviceOutputTest, TouchesOnlyTheDeviceItIsGiven) {
  // §3.7 secondary passthrough: a hand does not stop being commandable because
  // the arm's state went missing.
  rtc::ControllerOutput output;
  output.num_devices = 2;
  output.devices[0].num_channels = 6;
  output.devices[1].num_channels = 10;
  for (std::size_t i = 0; i < 10; ++i) {
    output.devices[1].commands[i] = 0.5 + static_cast<double>(i);
  }

  SilenceDeviceOutput(output.devices[0]);

  EXPECT_EQ(output.devices[0].num_channels, 0);
  EXPECT_EQ(output.devices[1].num_channels, 10);
  for (std::size_t i = 0; i < 10; ++i) {
    EXPECT_DOUBLE_EQ(output.devices[1].commands[i], 0.5 + static_cast<double>(i));
  }
  EXPECT_EQ(output.num_devices, 2) << "the device count still describes the state, not the gate";
}

// ── FillCommandTail — the [model_bound, nc0) domain split ────────────────────

// Every tail fixture pre-fills with a sentinel: a zero-initialised array would
// satisfy the torque assertion with the function deleted.
constexpr double kSentinel = -12345.0;

std::array<double, kMaxDeviceChannels> SentinelCommands() {
  std::array<double, kMaxDeviceChannels> commands;
  commands.fill(kSentinel);
  return commands;
}

TEST(FillCommandTailTest, TorqueTailIsZero) {
  auto commands = SentinelCommands();
  const DeviceState dev = MakeDevice(9);

  FillCommandTail(commands, /*model_bound=*/6, /*num_channels=*/9, CommandType::kTorque,
                  dev.positions);

  EXPECT_DOUBLE_EQ(commands[5], kSentinel) << "the modelled channels are not the tail's business";
  EXPECT_DOUBLE_EQ(commands[6], 0.0);
  EXPECT_DOUBLE_EQ(commands[7], 0.0);
  EXPECT_DOUBLE_EQ(commands[8], 0.0);
  EXPECT_DOUBLE_EQ(commands[9], kSentinel) << "and neither is anything past what was reported";
}

TEST(FillCommandTailTest, PositionTailHoldsTheMeasuredValue) {
  auto commands = SentinelCommands();
  const DeviceState dev = MakeDevice(9);

  FillCommandTail(commands, /*model_bound=*/6, /*num_channels=*/9, CommandType::kPosition,
                  dev.positions);

  // 0.0 here would be "servo to the origin" on every channel the model does
  // not cover — the failure this split exists to prevent.
  EXPECT_DOUBLE_EQ(commands[6], 7.0);
  EXPECT_DOUBLE_EQ(commands[7], 8.0);
  EXPECT_DOUBLE_EQ(commands[8], 9.0);
  EXPECT_NE(commands[6], 0.0);
}

TEST(FillCommandTailTest, PdFeedforwardTailHoldsTheMeasuredValue) {
  // kPdFeedforward is a position-servo backbone with a torque overlay, so its
  // `commands` lane follows the position rule, not the torque one.
  auto commands = SentinelCommands();
  const DeviceState dev = MakeDevice(9);

  FillCommandTail(commands, 6, 9, CommandType::kPdFeedforward, dev.positions);

  EXPECT_DOUBLE_EQ(commands[6], 7.0);
  EXPECT_DOUBLE_EQ(commands[8], 9.0);
}

TEST(FillCommandTailTest, FallsBackToZeroWhereEvenTheMeasurementIsMissing) {
  auto commands = SentinelCommands();
  const std::vector<double> short_measurement{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};

  FillCommandTail(commands, 6, 9, CommandType::kPosition, short_measurement);

  EXPECT_DOUBLE_EQ(commands[6], 7.0);
  EXPECT_DOUBLE_EQ(commands[7], 0.0);
  EXPECT_DOUBLE_EQ(commands[8], 0.0);
}

TEST(FillCommandTailTest, NeverWritesPastTheCommandSpan) {
  std::array<double, 8> commands;
  commands.fill(kSentinel);
  const DeviceState dev = MakeDevice(static_cast<int>(kMaxDeviceChannels));

  FillCommandTail(commands, 6, static_cast<int>(kMaxDeviceChannels), CommandType::kPosition,
                  dev.positions);

  EXPECT_DOUBLE_EQ(commands[6], 7.0);
  EXPECT_DOUBLE_EQ(commands[7], 8.0);
}

TEST(FillCommandTailTest, IsEmptyWhenTheModelCoversEveryReportedChannel) {
  auto commands = SentinelCommands();
  const DeviceState dev = MakeDevice(6);

  FillCommandTail(commands, ModelChannelBound(dev.num_channels, 6), dev.num_channels,
                  CommandType::kPosition, dev.positions);

  for (std::size_t i = 0; i < 8; ++i) {
    EXPECT_DOUBLE_EQ(commands[i], kSentinel) << "channel " << i;
  }
}

TEST(FillCommandTailTest, IgnoresNegativeBounds) {
  auto commands = SentinelCommands();
  const DeviceState dev = MakeDevice(3);

  FillCommandTail(commands, -4, -1, CommandType::kPosition, dev.positions);

  EXPECT_DOUBLE_EQ(commands[0], kSentinel);
}

}  // namespace
