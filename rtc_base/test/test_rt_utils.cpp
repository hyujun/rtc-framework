// Unit tests for rtc::utils RT-path command helpers:
//   - ClampSymmetric / ClampRange (clamp_commands.hpp): per-channel saturation
//     using per-index limits, falling back to the default limit when the
//     channel index runs past the supplied limits span.
//   - PassthroughSecondaryDevices (device_passthrough.hpp): mirror cached
//     targets into every device beyond the primary (index 0), leaving the
//     primary output untouched and respecting the num_devices bound.
//
// All pure-logic, allocation-free, sandbox-safe (no RT permissions needed).

#include "rtc_base/types/types.hpp"
#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_base/utils/device_passthrough.hpp"

#include <gtest/gtest.h>

#include <array>
#include <span>

namespace {

using rtc::kMaxDeviceChannels;

TEST(ClampSymmetricTest, WithinLimitsUnchanged) {
  std::array<double, kMaxDeviceChannels> cmd{};
  cmd[0] = 0.5;
  cmd[1] = -0.5;
  cmd[2] = 0.0;
  const std::array<double, 3> lim{1.0, 1.0, 1.0};
  rtc::utils::ClampSymmetric(cmd, 3, std::span<const double>(lim), 1.0);
  EXPECT_DOUBLE_EQ(cmd[0], 0.5);
  EXPECT_DOUBLE_EQ(cmd[1], -0.5);
  EXPECT_DOUBLE_EQ(cmd[2], 0.0);
}

TEST(ClampSymmetricTest, SaturatesAboveAndBelow) {
  std::array<double, kMaxDeviceChannels> cmd{};
  cmd[0] = 5.0;
  cmd[1] = -5.0;
  const std::array<double, 2> lim{2.0, 3.0};
  rtc::utils::ClampSymmetric(cmd, 2, std::span<const double>(lim), 1.0);
  EXPECT_DOUBLE_EQ(cmd[0], 2.0);
  EXPECT_DOUBLE_EQ(cmd[1], -3.0);
}

TEST(ClampSymmetricTest, DefaultLimitWhenIndexBeyondSpan) {
  std::array<double, kMaxDeviceChannels> cmd{};
  cmd[0] = 5.0;
  cmd[1] = 5.0;  // only index 0 has an explicit limit
  const std::array<double, 1> lim{2.0};
  rtc::utils::ClampSymmetric(cmd, 2, std::span<const double>(lim), 4.0);
  EXPECT_DOUBLE_EQ(cmd[0], 2.0);  // explicit limit
  EXPECT_DOUBLE_EQ(cmd[1], 4.0);  // default_limit
}

TEST(ClampSymmetricTest, EmptyLimitsUsesDefaultForAll) {
  std::array<double, kMaxDeviceChannels> cmd{};
  cmd[0] = 9.0;
  cmd[1] = -9.0;
  rtc::utils::ClampSymmetric(cmd, 2, std::span<const double>{}, 3.0);
  EXPECT_DOUBLE_EQ(cmd[0], 3.0);
  EXPECT_DOUBLE_EQ(cmd[1], -3.0);
}

TEST(ClampSymmetricTest, OnlyFirstNChannelsTouched) {
  std::array<double, kMaxDeviceChannels> cmd{};
  cmd[0] = 9.0;
  cmd[3] = 9.0;  // index 3 is beyond n = 1
  const std::array<double, 1> lim{1.0};
  rtc::utils::ClampSymmetric(cmd, 1, std::span<const double>(lim), 1.0);
  EXPECT_DOUBLE_EQ(cmd[0], 1.0);
  EXPECT_DOUBLE_EQ(cmd[3], 9.0);  // untouched
}

TEST(ClampRangeTest, AsymmetricBounds) {
  std::array<double, kMaxDeviceChannels> cmd{};
  cmd[0] = 5.0;
  cmd[1] = -5.0;
  cmd[2] = 0.0;
  const std::array<double, 3> lo{-1.0, -2.0, -1.0};
  const std::array<double, 3> hi{2.0, 1.0, 1.0};
  rtc::utils::ClampRange(cmd, 3, std::span<const double>(lo), std::span<const double>(hi), -10.0,
                         10.0);
  EXPECT_DOUBLE_EQ(cmd[0], 2.0);   // above upper
  EXPECT_DOUBLE_EQ(cmd[1], -2.0);  // below lower
  EXPECT_DOUBLE_EQ(cmd[2], 0.0);   // within
}

TEST(ClampRangeTest, DefaultBoundsBeyondSpan) {
  std::array<double, kMaxDeviceChannels> cmd{};
  cmd[0] = 50.0;
  cmd[1] = -50.0;
  const std::array<double, 1> lo{-1.0};
  const std::array<double, 1> hi{1.0};
  rtc::utils::ClampRange(cmd, 2, std::span<const double>(lo), std::span<const double>(hi), -5.0,
                         5.0);
  EXPECT_DOUBLE_EQ(cmd[0], 1.0);   // explicit upper bound
  EXPECT_DOUBLE_EQ(cmd[1], -5.0);  // default lower bound (index beyond span)
}

TEST(PassthroughSecondaryDevicesTest, MirrorsSecondaryLeavesPrimary) {
  rtc::ControllerState state{};
  rtc::ControllerOutput output{};
  state.num_devices = 2;
  state.devices[0].num_channels = 6;  // arm (primary)
  state.devices[1].num_channels = 3;  // hand (secondary)

  std::array<std::array<double, kMaxDeviceChannels>, rtc::ControllerState::kMaxDevices> targets{};
  targets[1][0] = 0.1;
  targets[1][1] = 0.2;
  targets[1][2] = 0.3;

  rtc::utils::PassthroughSecondaryDevices<rtc::ControllerState::kMaxDevices>(state, output, targets);

  // Primary (index 0) output left untouched.
  EXPECT_EQ(output.devices[0].num_channels, 0);
  EXPECT_DOUBLE_EQ(output.devices[0].commands[0], 0.0);

  // Secondary mirrored across commands / target_positions / goal_positions.
  EXPECT_EQ(output.devices[1].num_channels, 3);
  for (std::size_t i = 0; i < 3U; ++i) {
    EXPECT_DOUBLE_EQ(output.devices[1].commands[i], targets[1][i]);
    EXPECT_DOUBLE_EQ(output.devices[1].target_positions[i], targets[1][i]);
    EXPECT_DOUBLE_EQ(output.devices[1].goal_positions[i], targets[1][i]);
  }
}

TEST(PassthroughSecondaryDevicesTest, RespectsNumDevicesBound) {
  rtc::ControllerState state{};
  rtc::ControllerOutput output{};
  state.num_devices = 1;              // only the primary is present
  state.devices[1].num_channels = 3;  // stale data beyond num_devices

  std::array<std::array<double, kMaxDeviceChannels>, rtc::ControllerState::kMaxDevices> targets{};
  targets[1][0] = 9.9;

  rtc::utils::PassthroughSecondaryDevices<rtc::ControllerState::kMaxDevices>(state, output, targets);

  // device 1 must not be processed (loop guard d < num_devices == 1 fails).
  EXPECT_EQ(output.devices[1].num_channels, 0);
  EXPECT_DOUBLE_EQ(output.devices[1].commands[0], 0.0);
}

}  // namespace
