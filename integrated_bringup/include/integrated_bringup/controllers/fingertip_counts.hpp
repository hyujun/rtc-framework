#ifndef INTEGRATED_BRINGUP_CONTROLLERS_FINGERTIP_COUNTS_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_FINGERTIP_COUNTS_HPP_

#include "integrated_bringup/controllers/hand_sensor_layout.hpp"
#include "rtc_base/types/types.hpp"

#include <algorithm>

namespace integrated_bringup {

// Derived per-tick fingertip counts for the demo controllers' hand-sensor
// parsing. `active` is the inference-group count (force / contact lane) — the
// count all grasp logic iterates. `sensor` is the raw sensor-lane (baro / ToF)
// count that gates the ToF snapshot. They are equal on shipping hands but are
// tracked separately so a force-only stream (0 sensor channels) never drives
// the ToF snapshot from junk zero data.
struct FingertipCounts {
  int active{0};
  int sensor{0};
};

// Prefer num_inference_groups (set by mujoco_native and udp_hand backends
// regardless of sensor_layout), fall back to the sensor-channel stride for
// older mocks that only fill the sensor lane. Both counts are clamped to
// kMaxSensorGroups. Shared by the joint / task / wbc demo controllers so the
// derivation cannot drift between them.
[[nodiscard]] inline FingertipCounts DeriveFingertipCounts(const rtc::DeviceState& dev) noexcept {
  constexpr int kCap = static_cast<int>(rtc::kMaxSensorGroups);
  constexpr int kStride = static_cast<int>(kHandSensorValuesPerFingertipCapacity);
  const int sensor_ft = (kStride > 0) ? (dev.num_sensor_channels / kStride) : 0;
  const int active_ft = (dev.num_inference_groups > 0) ? dev.num_inference_groups : sensor_ft;
  return {std::min(active_ft, kCap), std::min(sensor_ft, kCap)};
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_FINGERTIP_COUNTS_HPP_
