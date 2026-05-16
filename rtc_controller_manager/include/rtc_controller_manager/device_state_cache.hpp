#ifndef RTC_CONTROLLER_MANAGER_DEVICE_STATE_CACHE_H_
#define RTC_CONTROLLER_MANAGER_DEVICE_STATE_CACHE_H_

#include "rtc_base/threading/publish_buffer.hpp"
#include "rtc_base/types/types.hpp"

#include <array>
#include <cstdint>
#include <type_traits>

namespace rtc {

// ── DeviceStateCache ─────────────────────────────────────────────────────────
//
// Per-device decoded state. Held in a SeqLock (single-writer, multi-reader)
// inside RtControllerNode and (Phase 2+) populated by DeviceBackend
// implementations. Trivially copyable so SeqLock can memcpy.
struct DeviceStateCache {
  int num_channels{0};
  std::array<double, kMaxDeviceChannels> positions{};
  std::array<double, kMaxDeviceChannels> velocities{};
  std::array<double, kMaxDeviceChannels> efforts{};
  // Motor-space data (separate from joint-space)
  int num_motor_channels{0};
  std::array<double, kMaxDeviceChannels> motor_positions{};
  std::array<double, kMaxDeviceChannels> motor_velocities{};
  std::array<double, kMaxDeviceChannels> motor_efforts{};
  std::array<int32_t, kMaxSensorChannels> sensor_data{};
  std::array<int32_t, kMaxSensorChannels> sensor_data_raw{};
  int num_sensor_channels{0};
  // Inference output per sensor group (force/displacement)
  std::array<float, kMaxInferenceValues> inference_data{};
  std::array<bool, kMaxSensorGroups> inference_enable{};
  int num_inference_groups{0};
  bool valid{false};
};

static_assert(std::is_trivially_copyable_v<DeviceStateCache>,
              "DeviceStateCache must be trivially copyable for SeqLock");

inline constexpr int kMaxDevices = PublishSnapshot::kMaxGroups;

}  // namespace rtc

#endif  // RTC_CONTROLLER_MANAGER_DEVICE_STATE_CACHE_H_
