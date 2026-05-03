#ifndef UDP_HAND_DRIVER_UDP_HAND_STATE_HPP_
#define UDP_HAND_DRIVER_UDP_HAND_STATE_HPP_

#include "udp_hand_driver/udp_hand_constants.hpp"
#include <rtc_base/types/types.hpp>

#include <array>
#include <cstdint>

namespace udp_hand_driver {

struct UdpHandState {
  // Motor-space (from kMotor read)
  std::array<float, kNumHandMotors> motor_positions{};
  std::array<float, kNumHandMotors> motor_velocities{};
  std::array<float, kNumHandMotors> motor_currents{};
  // Joint-space (from kJoint read)
  std::array<float, kNumHandMotors> joint_positions{};
  std::array<float, kNumHandMotors> joint_velocities{};
  std::array<float, kNumHandMotors> joint_currents{};
  // Sensor
  std::array<int32_t, udp_hand_driver::kMaxHandSensors> sensor_data{};
  std::array<int32_t, udp_hand_driver::kMaxHandSensors> sensor_data_raw{};
  int num_fingertips{udp_hand_driver::kDefaultNumFingertips};
  bool valid{false};                  ///< any read succeeded this cycle
  bool joint_valid{false};            ///< kJoint read succeeded this cycle
  bool motor_valid{false};            ///< kMotor read succeeded this cycle
  uint8_t received_joint_mode{0x00};  ///< 0x00=motor, 0x01=joint (from response packet)
};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_UDP_HAND_STATE_HPP_
