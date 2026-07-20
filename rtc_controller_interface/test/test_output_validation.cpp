// Unit tests for ValidateControllerOutput — the actuator-boundary screen a
// ControllerOutput must pass before the controller manager forwards it to a
// DeviceBackend (issue #196 Phase 4).
//
// The function is pure, so these tests hold it to its contract directly:
// which shapes are rejected, which are deliberately NOT rejected, and that a
// well-formed output from a normal controller is untouched. The RT-loop side
// of the contract (hold substitution, reject counting, E-STOP escalation)
// lives in rtc_controller_manager/test/test_rt_loop_pipeline.cpp.

#include "rtc_controller_interface/rt_controller_interface.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// One device, two channels, everything in contract — the baseline every test
// below mutates a single field of, so a failure names exactly one cause.
rtc::ControllerState MakeState(int num_devices = 1, int num_channels = 2) {
  rtc::ControllerState state{};
  state.num_devices = num_devices;
  for (int i = 0; i < num_devices; ++i) {
    auto& dev = state.devices[static_cast<std::size_t>(i)];
    dev.num_channels = num_channels;
    dev.valid = true;
    for (int c = 0; c < num_channels; ++c) {
      dev.positions[static_cast<std::size_t>(c)] = 0.1 * (c + 1);
    }
  }
  return state;
}

rtc::ControllerOutput MakeOutput(const rtc::ControllerState& state) {
  rtc::ControllerOutput out{};
  out.num_devices = state.num_devices;
  for (int i = 0; i < state.num_devices; ++i) {
    const auto& dstate = state.devices[static_cast<std::size_t>(i)];
    auto& dout = out.devices[static_cast<std::size_t>(i)];
    dout.num_channels = dstate.num_channels;
    for (int c = 0; c < dstate.num_channels; ++c) {
      dout.commands[static_cast<std::size_t>(c)] = 0.5 * (c + 1);
    }
  }
  return out;
}

// ── Accepted ────────────────────────────────────────────────────────────────

TEST(ValidateControllerOutput, WellFormedOutputIsAccepted) {
  const auto state = MakeState();
  const auto out = MakeOutput(state);

  const auto v = rtc::ValidateControllerOutput(out, state);

  EXPECT_TRUE(v.Ok());
  EXPECT_EQ(rtc::OutputRejectReason::kNone, v.reason);
}

TEST(ValidateControllerOutput, ZeroDevicesIsAccepted) {
  // A controller with no device groups is a degenerate but legal
  // configuration; it must not be treated as a fault.
  rtc::ControllerState state{};
  rtc::ControllerOutput out{};

  EXPECT_TRUE(rtc::ValidateControllerOutput(out, state).Ok());
}

TEST(ValidateControllerOutput, FewerCommandedChannelsThanReportedIsAccepted) {
  // Commanding a subset of a device's channels is in contract — only the
  // opposite direction (more than the device has) is a fault.
  const auto state = MakeState(/*num_devices=*/1, /*num_channels=*/4);
  auto out = MakeOutput(state);
  out.devices[0].num_channels = 2;

  EXPECT_TRUE(rtc::ValidateControllerOutput(out, state).Ok());
}

TEST(ValidateControllerOutput, NonFinitePastCommandedChannelCountIsIgnored) {
  // Only [0, num_channels) reaches the wire, so garbage in the unused tail of
  // the fixed array must not fail an otherwise sound output — controllers are
  // not required to zero the whole 64-wide array every tick.
  const auto state = MakeState();
  auto out = MakeOutput(state);
  out.devices[0].commands[5] = kNaN;
  out.devices[0].feedforward[7] = kInf;

  EXPECT_TRUE(rtc::ValidateControllerOutput(out, state).Ok());
}

TEST(ValidateControllerOutput, NonFiniteTelemetryFieldIsIgnored) {
  // Deliberate scope limit: task poses and trajectory references feed log/GUI
  // lanes, not actuators. Rejecting a whole tick over them would trade a
  // cosmetic defect for a motion stop.
  const auto state = MakeState();
  auto out = MakeOutput(state);
  out.devices[0].target_positions[0] = kNaN;
  out.devices[0].trajectory_velocities[1] = kInf;
  out.actual_task_positions[0] = kNaN;

  EXPECT_TRUE(rtc::ValidateControllerOutput(out, state).Ok());
}

// ── Rejected ────────────────────────────────────────────────────────────────

TEST(ValidateControllerOutput, ValidFlagFalseIsRejected) {
  const auto state = MakeState();
  auto out = MakeOutput(state);
  out.valid = false;

  const auto v = rtc::ValidateControllerOutput(out, state);

  EXPECT_FALSE(v.Ok());
  EXPECT_EQ(rtc::OutputRejectReason::kInvalidFlag, v.reason);
}

TEST(ValidateControllerOutput, DeviceCountMismatchIsRejected) {
  const auto state = MakeState(/*num_devices=*/2);
  auto out = MakeOutput(state);
  out.num_devices = 1;

  const auto v = rtc::ValidateControllerOutput(out, state);

  EXPECT_FALSE(v.Ok());
  EXPECT_EQ(rtc::OutputRejectReason::kDeviceCountMismatch, v.reason);
}

TEST(ValidateControllerOutput, DeviceCountPastCapacityIsRejected) {
  // Equal counts on both sides must still not license an out-of-range index:
  // state.num_devices reaching this function is bounded by the CM read path,
  // but the check has to stand on its own for any other caller.
  rtc::ControllerState state{};
  state.num_devices = rtc::ControllerOutput::kMaxDevices + 1;
  rtc::ControllerOutput out{};
  out.num_devices = rtc::ControllerOutput::kMaxDevices + 1;

  const auto v = rtc::ValidateControllerOutput(out, state);

  EXPECT_FALSE(v.Ok());
  EXPECT_EQ(rtc::OutputRejectReason::kDeviceCountMismatch, v.reason);
}

TEST(ValidateControllerOutput, NegativeDeviceCountIsRejected) {
  rtc::ControllerState state{};
  state.num_devices = -1;
  rtc::ControllerOutput out{};
  out.num_devices = -1;

  const auto v = rtc::ValidateControllerOutput(out, state);

  EXPECT_FALSE(v.Ok());
  EXPECT_EQ(rtc::OutputRejectReason::kDeviceCountMismatch, v.reason);
}

TEST(ValidateControllerOutput, NegativeChannelCountIsRejected) {
  const auto state = MakeState();
  auto out = MakeOutput(state);
  out.devices[0].num_channels = -1;

  const auto v = rtc::ValidateControllerOutput(out, state);

  EXPECT_FALSE(v.Ok());
  EXPECT_EQ(rtc::OutputRejectReason::kChannelCountOutOfRange, v.reason);
  EXPECT_EQ(0, v.device_idx);
}

TEST(ValidateControllerOutput, ChannelCountPastArrayCapacityIsRejected) {
  const auto state = MakeState();
  auto out = MakeOutput(state);
  out.devices[0].num_channels = rtc::kMaxDeviceChannels + 1;

  const auto v = rtc::ValidateControllerOutput(out, state);

  EXPECT_FALSE(v.Ok());
  EXPECT_EQ(rtc::OutputRejectReason::kChannelCountOutOfRange, v.reason);
}

TEST(ValidateControllerOutput, ChannelCountPastDeviceReportIsRejected) {
  // The case the array-capacity bound alone cannot catch: 3 is a legal index
  // into the fixed array but the device only has 2 channels, so the third
  // command addresses hardware that does not exist.
  const auto state = MakeState(/*num_devices=*/1, /*num_channels=*/2);
  auto out = MakeOutput(state);
  out.devices[0].num_channels = 3;

  const auto v = rtc::ValidateControllerOutput(out, state);

  EXPECT_FALSE(v.Ok());
  EXPECT_EQ(rtc::OutputRejectReason::kChannelCountOutOfRange, v.reason);
  EXPECT_EQ(0, v.device_idx);
}

TEST(ValidateControllerOutput, NanCommandIsRejected) {
  const auto state = MakeState();
  auto out = MakeOutput(state);
  out.devices[0].commands[1] = kNaN;

  const auto v = rtc::ValidateControllerOutput(out, state);

  EXPECT_FALSE(v.Ok());
  EXPECT_EQ(rtc::OutputRejectReason::kNonFiniteCommand, v.reason);
  EXPECT_EQ(0, v.device_idx);
  EXPECT_EQ(1, v.channel_idx);
}

TEST(ValidateControllerOutput, InfCommandIsRejected) {
  const auto state = MakeState();
  auto out = MakeOutput(state);
  out.devices[0].commands[0] = -kInf;

  const auto v = rtc::ValidateControllerOutput(out, state);

  EXPECT_FALSE(v.Ok());
  EXPECT_EQ(rtc::OutputRejectReason::kNonFiniteCommand, v.reason);
  EXPECT_EQ(0, v.channel_idx);
}

TEST(ValidateControllerOutput, NanFeedforwardIsRejectedRegardlessOfCommandType) {
  // feedforward only reaches an actuator under kPdFeedforward, but the type
  // that decides comes from the same untrusted output. Screening it
  // unconditionally is what keeps a corrupted type field from smuggling a NaN
  // through.
  const auto state = MakeState();
  auto out = MakeOutput(state);
  out.command_type = rtc::CommandType::kPosition;
  out.devices[0].feedforward[1] = kNaN;

  const auto v = rtc::ValidateControllerOutput(out, state);

  EXPECT_FALSE(v.Ok());
  EXPECT_EQ(rtc::OutputRejectReason::kNonFiniteCommand, v.reason);
  EXPECT_EQ(1, v.channel_idx);
}

TEST(ValidateControllerOutput, FaultInSecondDeviceIsReported) {
  const auto state = MakeState(/*num_devices=*/2);
  auto out = MakeOutput(state);
  out.devices[1].commands[0] = kNaN;

  const auto v = rtc::ValidateControllerOutput(out, state);

  EXPECT_FALSE(v.Ok());
  EXPECT_EQ(1, v.device_idx);
  EXPECT_EQ(0, v.channel_idx);
}

// ── Reason strings ──────────────────────────────────────────────────────────

TEST(OutputRejectReasonToString, EveryReasonHasADistinctName) {
  const rtc::OutputRejectReason reasons[] = {
      rtc::OutputRejectReason::kNone,
      rtc::OutputRejectReason::kInvalidFlag,
      rtc::OutputRejectReason::kDeviceCountMismatch,
      rtc::OutputRejectReason::kChannelCountOutOfRange,
      rtc::OutputRejectReason::kNonFiniteCommand,
  };
  for (std::size_t i = 0; i < std::size(reasons); ++i) {
    const std::string a = rtc::OutputRejectReasonToString(reasons[i]);
    EXPECT_FALSE(a.empty());
    EXPECT_NE("unknown", a);
    for (std::size_t j = i + 1; j < std::size(reasons); ++j) {
      EXPECT_NE(a, std::string(rtc::OutputRejectReasonToString(reasons[j])));
    }
  }
}

}  // namespace
