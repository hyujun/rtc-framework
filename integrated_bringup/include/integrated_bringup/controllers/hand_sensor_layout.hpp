#ifndef INTEGRATED_BRINGUP_CONTROLLERS_HAND_SENSOR_LAYOUT_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_HAND_SENSOR_LAYOUT_HPP_

// integrated_bringup-local capacity constants for the demo controllers'
// per-fingertip sensor parsing. These mirror the assm_v1 hand schema but
// live HERE — no rtc_* or udp_hand_driver constant defines them. The boundary
// between integrated_bringup and udp_hand_driver is the rtc_msgs
// FingertipSensor.msg named fields (barometer[8], tof[3]); array sizes here
// are integrated_bringup's own SSoT for compile-time std::array dimensions.
//
// Runtime stride (values_per_group, inference_values_per_group) is delivered
// per-device by the YAML sensor_layout block (rtc::DeviceSensorLayout). The
// demo controllers' fingertip parsers do NOT follow it — they read the raw
// lane at the fixed stride kHandSensorValuesPerFingertipCapacity and the
// inference lane at kHandInferenceValuesPerFingertipCapacity — so their
// on_configure refuses any layout HandSensorLayoutError rejects (#885). A
// stride the parser does not match would otherwise shift every fingertip
// after the first under the right names, with nothing to flag it.

#include "rtc_base/types/types.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

namespace integrated_bringup {

// Per-fingertip sensor block capacities (compile-time array sizing).
inline constexpr std::size_t kHandBaroChannelsCapacity = 8;
inline constexpr std::size_t kHandTofChannelsCapacity = 3;
inline constexpr std::size_t kHandSensorValuesPerFingertipCapacity =
    kHandBaroChannelsCapacity + kHandTofChannelsCapacity;  // 11

// Per-fingertip ML inference output capacity: contact(1) + F(3) + u(3) = 7.
// Stride 7 is a *sensor union* — different backends populate different subsets:
//   slot 0   = native contact probability (sigmoid output)
//                — populated by udp_hand_native when ft_inferencer.enabled,
//                  zero from force-only backends (e.g. mujoco fingertip wrench).
//   slot 1..3 = force vector fx/fy/fz [N]
//                — populated by all fingertip-bearing backends.
//                Contract (#167, SSoT: rtc_msgs/msg/FingertipSensor.msg):
//                fingertip *link* frame (sensor extrinsic baked into the
//                publisher), env-on-fingertip sign — consumers needing
//                finger-on-object multiply by -1.
//   slot 4..6 = native displacement vector ux/uy/uz
//                — populated by udp_hand_native only (capacitive direction),
//                  zero from force-only backends.
// Controllers honor per-slot availability via DeviceSensorLayout
// `has_native_contact` / `has_native_displacement` (yaml-declared per device
// group); reading raw slots without consulting the capability flags risks
// treating zero-fill as a real signal.
inline constexpr std::size_t kHandInferenceValuesPerFingertipCapacity = 7;

// Hand actuator count (assm_v1 hand: 10-DoF). integrated_bringup's own SSoT
// for compile-time JointSpaceTrajectory<N> sizing — udp_hand_driver carries
// the same number under its own roof; the boundary contract is the
// rtc_msgs JointCommand named arrays.
inline constexpr std::size_t kHandMotorCount = 10;

// Why `layout` (the hand device's, named `device`) cannot be read by the demo
// controllers' fixed-stride fingertip parsers, or empty when it can. The raw
// lane must be absent (every count 0) or exactly kHandBaroChannelsCapacity +
// kHandTofChannelsCapacity per fingertip; the inference lane must be absent
// (0) or exactly kHandInferenceValuesPerFingertipCapacity per fingertip. No
// layout at all is accepted — the device then carries no packed sensor lane.
// The generic capacity bounds are rtc_controller_manager's, checked earlier.
[[nodiscard]] inline std::string HandSensorLayoutError(
    const std::optional<rtc::DeviceSensorLayout>& layout, std::string_view device) {
  if (!layout.has_value()) {
    return {};
  }
  const auto& sl = *layout;
  const bool raw_absent = sl.primary_count_per_group == 0 && sl.secondary_count_per_group == 0 &&
                          sl.values_per_group == 0;
  const bool raw_matches =
      sl.primary_count_per_group == static_cast<int>(kHandBaroChannelsCapacity) &&
      sl.secondary_count_per_group == static_cast<int>(kHandTofChannelsCapacity) &&
      sl.values_per_group == static_cast<int>(kHandSensorValuesPerFingertipCapacity);
  const bool inference_matches =
      sl.inference_values_per_group == 0 ||
      sl.inference_values_per_group == static_cast<int>(kHandInferenceValuesPerFingertipCapacity);
  if ((raw_absent || raw_matches) && inference_matches) {
    return {};
  }
  return "device '" + std::string(device) +
         "' sensor_layout primary=" + std::to_string(sl.primary_count_per_group) +
         " secondary=" + std::to_string(sl.secondary_count_per_group) +
         " values_per_group=" + std::to_string(sl.values_per_group) +
         " inference_values_per_group=" + std::to_string(sl.inference_values_per_group) +
         " does not match the fixed fingertip parser: the raw lane must be 0/0/0 or " +
         std::to_string(kHandBaroChannelsCapacity) + "/" +
         std::to_string(kHandTofChannelsCapacity) + "/" +
         std::to_string(kHandSensorValuesPerFingertipCapacity) +
         " and inference_values_per_group 0 or " +
         std::to_string(kHandInferenceValuesPerFingertipCapacity);
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_HAND_SENSOR_LAYOUT_HPP_
