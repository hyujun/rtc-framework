#ifndef RTC_BASE_UTILS_DEVICE_PASSTHROUGH_HPP_
#define RTC_BASE_UTILS_DEVICE_PASSTHROUGH_HPP_

#include <array>
#include <cstddef>

#include "rtc_base/types/types.hpp"

namespace rtc::utils {

// Pass through cached `device_targets[d]` into devices[d] of `output` for
// every device beyond the primary (index 0). Each output channel's commands /
// target_positions / goal_positions are set to the same target value, and
// num_channels is mirrored from the input state. Used by every controller
// whose primary device is the arm and whose secondary device(s) are simple
// joint passthroughs (hand, gripper, ...).
//
// `kMaxDevices` defines the upper bound of `output.devices` /
// `state.devices` arrays; `device_targets` must be sized accordingly.
// RT-safe (no allocation, no exceptions).
template <std::size_t kMaxDevicesT>
inline void PassthroughSecondaryDevices(
    const ControllerState &state, ControllerOutput &output,
    const std::array<std::array<double, kMaxDeviceChannels>, kMaxDevicesT>
        &device_targets) noexcept {
  for (std::size_t d = 1;
       d < static_cast<std::size_t>(state.num_devices) && d < kMaxDevicesT;
       ++d) {
    const auto &devN = state.devices[d];
    auto &outN = output.devices[d];
    const int ncN = devN.num_channels;
    outN.num_channels = ncN;
    for (std::size_t i = 0; i < static_cast<std::size_t>(ncN); ++i) {
      outN.commands[i] = device_targets[d][i];
      outN.target_positions[i] = device_targets[d][i];
      outN.goal_positions[i] = device_targets[d][i];
    }
  }
}

} // namespace rtc::utils

#endif // RTC_BASE_UTILS_DEVICE_PASSTHROUGH_HPP_
