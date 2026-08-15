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
  // Per-slot freshness of the `positions` array. Same field, same polarity
  // (bit set = slot NOT written by the latest message) and same rationale as
  // DeviceState::hole_mask in rtc_base/types/types.hpp — that declaration owns
  // the contract; this one exists because the cache is where the backends
  // write and DeviceState is where the controllers read, and the two are
  // separate PODs joined by a field-by-field copy in RtControllerNode.
  //
  // This mirror is the reason the copy needs a test: a new field added here
  // and forgotten there compiles, and the ingress keeps working, but the
  // controllers see a permanent 0 — which under this polarity reads as
  // "hole-free" and silently restores the very gap #284 closed.
  uint64_t hole_mask{0};
  // The other two lanes' masks (#446). Mirrors DeviceState::velocity_hole_mask
  // and ::effort_hole_mask; that declaration owns the contract, including why
  // an all-ones mask here is the NORMAL report for a driver that omits the
  // lane. The mirror-seam warning above applies to these two identically —
  // forgetting either in RtControllerNode's copy compiles, keeps the ingress
  // working, and hands the controllers a permanent 0 that this polarity spells
  // "hole-free".
  uint64_t velocity_hole_mask{0};
  uint64_t effort_hole_mask{0};
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
