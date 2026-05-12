#ifndef UR5E_BRINGUP_LOGGING_POD_FILL_HPP_
#define UR5E_BRINGUP_LOGGING_POD_FILL_HPP_

// RT-safe POD fill helpers shared by integrated_bringup demo controllers.
// Pull device data + controller output into the device POD mirrors that
// flow through ControllerLogSet. No allocation, no string ops, bounded
// by runtime num_* fields on the input arrays.
//
// Robot-agnostic: callers pass a device index (0=primary, 1=secondary, ...)
// matching the topic_config_.groups order. The primary device (index 0) is
// the only one that fills task_goal / actual_task_positions; secondary
// devices fill motor_* arrays when present.

#include "integrated_bringup/logging/device_sensor_log_pod.hpp"
#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "rtc_base/types/types.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace integrated_bringup {

inline void FillDeviceStateLogPod(const rtc::ControllerState& state,
                                  const rtc::ControllerOutput& output, std::size_t device_idx,
                                  integrated_bringup::DeviceStateLogPod& pod) noexcept {
  pod.t_relative_s = state.t_relative_s;
  if (static_cast<std::size_t>(state.num_devices) <= device_idx) {
    return;
  }
  const auto& dev = state.devices[device_idx];
  const auto& out = output.devices[device_idx];
  const auto n = std::min(static_cast<std::size_t>(dev.num_channels),
                          integrated_bringup::DeviceStateLogPod::kMaxJoints);
  pod.num_joints = static_cast<std::uint8_t>(n);
  for (std::size_t i = 0; i < n; ++i) {
    pod.actual_positions[i] = dev.positions[i];
    pod.actual_velocities[i] = dev.velocities[i];
    pod.efforts[i] = dev.efforts[i];
    pod.commands[i] = out.commands[i];
    pod.joint_goal[i] = out.goal_positions[i];
    pod.trajectory_positions[i] = out.trajectory_positions[i];
    pod.trajectory_velocities[i] = out.trajectory_velocities[i];
  }

  // task_goal / actual_task_positions are populated only for the primary
  // device (index 0); secondary device rows inherit zeros.
  if (device_idx == 0) {
    for (std::size_t i = 0; i < integrated_bringup::DeviceStateLogPod::kTaskDim; ++i) {
      pod.task_goal[i] = output.task_goal_positions[i];
      pod.actual_task_positions[i] = output.actual_task_positions[i];
    }
  }

  const auto nm = std::min(static_cast<std::size_t>(dev.num_motor_channels),
                           integrated_bringup::DeviceStateLogPod::kMaxMotors);
  pod.num_motors = static_cast<std::uint8_t>(nm);
  for (std::size_t i = 0; i < nm; ++i) {
    pod.motor_positions[i] = dev.motor_positions[i];
    pod.motor_velocities[i] = dev.motor_velocities[i];
    pod.motor_efforts[i] = dev.motor_efforts[i];
  }

  pod.command_type = (output.command_type == rtc::CommandType::kTorque) ? 1 : 0;
  pod.goal_type = (out.goal_type == rtc::GoalType::kTask) ? 1 : 0;
}

inline void FillDeviceSensorLogPod(const rtc::ControllerState& state, std::size_t device_idx,
                                   int num_active_fingertips,
                                   integrated_bringup::DeviceSensorLogPod& pod) noexcept {
  pod.t_relative_s = state.t_relative_s;
  if (static_cast<std::size_t>(state.num_devices) <= device_idx) {
    return;
  }
  const auto& dev = state.devices[device_idx];
  const auto num_fingertips = std::min(static_cast<std::size_t>(num_active_fingertips),
                                       integrated_bringup::DeviceSensorLogPod::kMaxFingertips);
  pod.num_fingertips = static_cast<std::uint8_t>(num_fingertips);
  pod.inference_valid = false;

  const auto n_sensor =
      num_fingertips * integrated_bringup::DeviceSensorLogPod::kSensorValuesPerFingertip;
  for (std::size_t i = 0; i < n_sensor && i < dev.sensor_data_raw.size(); ++i) {
    pod.sensor_data_raw[i] = dev.sensor_data_raw[i];
    pod.sensor_data[i] = dev.sensor_data[i];
  }

  const auto n_inf = num_fingertips * integrated_bringup::DeviceSensorLogPod::kFTValuesPerFingertip;
  for (std::size_t i = 0; i < n_inf && i < dev.inference_data.size(); ++i) {
    pod.inference_output[i] = dev.inference_data[i];
  }
  for (std::size_t f = 0; f < num_fingertips; ++f) {
    if (dev.inference_enable[f]) {
      pod.inference_valid = true;
      break;
    }
  }
}

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_LOGGING_POD_FILL_HPP_
