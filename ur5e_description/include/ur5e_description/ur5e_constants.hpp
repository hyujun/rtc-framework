#ifndef UR5E_DESCRIPTION_UR5E_CONSTANTS_HPP_
#define UR5E_DESCRIPTION_UR5E_CONSTANTS_HPP_

/// @file ur5e_constants.hpp
/// @brief UR5e + custom hand hardware-specific constants and legacy types.
///
/// These constants were previously in rtc_base/types/types.hpp but are
/// specific to the UR5e + 10-DOF hand platform.  Generic rtc_* packages
/// should NOT include this header.

#include <rtc_base/types/types.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace rtc
{

// ── UR5e + hand hardware constants ──────────────────────────────────────────

/// Number of hand motor channels (UR5e custom hand).
inline constexpr int kNumHandMotors  = 10;

/// Legacy alias -- downstream code still references kNumHandJoints.
inline constexpr int kNumHandJoints  = kNumHandMotors;

// ── Default joint/motor/fingertip names (UR5e-specific) ─────────────────────
// Used as fallback when YAML configuration does not specify names.

inline const std::vector<std::string> kDefaultRobotJointNames = {
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
};

inline const std::vector<std::string> kDefaultHandMotorNames = {
  "thumb_cmc_aa", "thumb_cmc_fe", "thumb_mcp_fe",
  "index_mcp_aa", "index_mcp_fe", "index_dip_fe",
  "middle_mcp_aa", "middle_mcp_fe", "middle_dip_fe",
  "ring_mcp_fe"
};

inline const std::vector<std::string> kDefaultFingertipNames = {
  "thumb", "index", "middle", "ring"
};

// ── Legacy structs ──────────────────────────────────────────────────────────
// Used by ur5e_hand_driver and packages that haven't migrated to the unified
// DeviceState.  Will be removed in a future major version.

struct RobotState {
  std::array<double, kNumRobotJoints> positions{};
  std::array<double, kNumRobotJoints> velocities{};
  std::array<double, kNumRobotJoints> torques{};
  std::array<double, 3>               tcp_position{};
  double   dt{0.002};
  uint64_t iteration{0};
};

struct HandState {
  // Motor-space (from kMotor read)
  std::array<float, kNumHandMotors>    motor_positions{};
  std::array<float, kNumHandMotors>    motor_velocities{};
  std::array<float, kNumHandMotors>    motor_currents{};
  // Joint-space (from kJoint read)
  std::array<float, kNumHandMotors>    joint_positions{};
  std::array<float, kNumHandMotors>    joint_velocities{};
  std::array<float, kNumHandMotors>    joint_currents{};
  // Sensor
  std::array<int32_t, kMaxHandSensors> sensor_data{};
  std::array<int32_t, kMaxHandSensors> sensor_data_raw{};
  int  num_fingertips{kDefaultNumFingertips};
  bool valid{false};         ///< any read succeeded this cycle
  bool joint_valid{false};   ///< kJoint read succeeded this cycle
  bool motor_valid{false};   ///< kMotor read succeeded this cycle
  uint8_t received_joint_mode{0x00};  ///< 0x00=motor, 0x01=joint (from response packet)
};

}  // namespace rtc

#endif  // UR5E_DESCRIPTION_UR5E_CONSTANTS_HPP_
