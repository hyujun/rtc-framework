#ifndef INTEGRATED_BRINGUP_CONTROLLERS_HAND_SENSOR_LAYOUT_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_HAND_SENSOR_LAYOUT_HPP_

// integrated_bringup-local capacity constants for the demo controllers'
// per-fingertip sensor parsing. These mirror the assm_v1 hand schema but
// live HERE — neither rtc_* nor udp_hand_driver is referenced. The boundary
// between integrated_bringup and udp_hand_driver is the rtc_msgs
// FingertipSensor.msg named fields (barometer[8], tof[3]); array sizes here
// are integrated_bringup's own SSoT for compile-time std::array dimensions.
//
// Runtime stride (values_per_group, inference_values_per_group) is delivered
// per-device by the YAML sensor_layout block (rtc::DeviceSensorLayout); use
// `RTControllerInterface::GetDeviceNameConfig(GetSecondaryDeviceName())
// ->sensor_layout` at on_configure to pick it up.

#include <cstddef>

namespace integrated_bringup {

// Per-fingertip sensor block capacities (compile-time array sizing).
inline constexpr std::size_t kHandBaroChannelsCapacity = 8;
inline constexpr std::size_t kHandTofChannelsCapacity = 3;
inline constexpr std::size_t kHandSensorValuesPerFingertipCapacity =
    kHandBaroChannelsCapacity + kHandTofChannelsCapacity;  // 11

// Per-fingertip ML inference output capacity: contact(1) + F(3) + u(3) = 7.
inline constexpr std::size_t kHandInferenceValuesPerFingertipCapacity = 7;

// Hand actuator count (assm_v1 hand: 10-DoF). integrated_bringup's own SSoT
// for compile-time JointSpaceTrajectory<N> sizing — udp_hand_driver carries
// the same number under its own roof; the boundary contract is the
// rtc_msgs JointCommand named arrays.
inline constexpr std::size_t kHandMotorCount = 10;

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_HAND_SENSOR_LAYOUT_HPP_
