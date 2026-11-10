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
// implementations. Trivially copyable so SeqLock can copy it byte-wise.
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
  // Per-group receipt time and sample counter of the inference lane.
  // Mirrors DeviceState::inference_recv_steady_ns
  // and ::inference_sequence; that declaration owns the contract, including
  // the "0 = never received" polarity and why `header.stamp` is not used.
  //
  // The mirror-seam warning on `hole_mask` above applies identically, and with
  // a sharper edge: dropping these from RtControllerNode's copy leaves a
  // permanent 0, which this polarity spells "never received" — the consumer
  // then withholds the lane forever, and the symptom is a catching controller
  // that always reports TIP_STALE rather than a crash.
  std::array<int64_t, kMaxSensorGroups> inference_recv_steady_ns{};
  std::array<uint64_t, kMaxSensorGroups> inference_sequence{};
  // Floating-base lane (WD-16, #976). Mirrors DeviceState::base_pose,
  // ::base_twist, ::imu, ::num_imu_sites, ::base_recv_steady_ns,
  // ::base_sequence and ::base_valid; that declaration owns the contract —
  // field order (p_xyz · q_xyzw; v · ω in the BASE frame), the IDENTITY
  // default of `base_pose` (a zero quaternion is not a rotation), the "0 =
  // never received" stamp polarity, and values held under a false `base_valid`.
  //
  // The mirror-seam warning on `hole_mask` above applies to every one of
  // these: a field dropped from RtControllerNode's copy compiles and keeps the
  // backend's own tests green, and a controller then reads a zero pose with an
  // identity-less quaternion and `base_valid` false forever — fail-closed, but
  // with nothing to say why.
  std::array<double, kBasePoseDim> base_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  std::array<double, kBaseTwistDim> base_twist{};
  std::array<std::array<double, kImuChannelsPerSite>, kMaxImuSites> imu{};
  int num_imu_sites{0};
  int64_t base_recv_steady_ns{0};
  uint64_t base_sequence{0};
  bool base_valid{false};
  bool valid{false};
};

static_assert(std::is_trivially_copyable_v<DeviceStateCache>,
              "DeviceStateCache must be trivially copyable for SeqLock");

inline constexpr int kMaxDevices = PublishSnapshot::kMaxGroups;

}  // namespace rtc

#endif  // RTC_CONTROLLER_MANAGER_DEVICE_STATE_CACHE_H_
