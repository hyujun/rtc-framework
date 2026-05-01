#ifndef RTC_BASE_LOG_BUFFER_HPP_
#define RTC_BASE_LOG_BUFFER_HPP_

// Lock-free single-producer / single-consumer (SPSC) ring buffer for
// transferring log entries from the RT control thread (producer) to the
// logging thread (consumer) without any blocking or heap allocation on the
// RT path.
//
// Constraints:
//   - Exactly ONE producer thread (the 500 Hz RT loop).
//   - Exactly ONE consumer thread (the log drain timer, Core 4).
//   - Push() called only from the producer; Pop() called only from the
//   consumer.

#include "rtc_base/concurrency/spsc_queue.hpp"
#include "rtc_base/types/types.hpp"

#include <array>
#include <cstddef>

namespace rtc
{

// Per-device log data slot (used in LogEntry for each device group)
struct DeviceLogSlot
{
  int num_channels{0};
  std::array<double, kMaxDeviceChannels> goal_positions{};
  std::array<double, kMaxDeviceChannels> actual_positions{};
  std::array<double, kMaxDeviceChannels> actual_velocities{};
  std::array<double, kMaxDeviceChannels> efforts{};
  std::array<double, kMaxDeviceChannels> commands{};
  std::array<double, kMaxDeviceChannels> trajectory_positions{};
  std::array<double, kMaxDeviceChannels> trajectory_velocities{};
  // Motor-space data
  int num_motor_channels{0};
  std::array<double, kMaxDeviceChannels> motor_positions{};
  std::array<double, kMaxDeviceChannels> motor_velocities{};
  std::array<double, kMaxDeviceChannels> motor_efforts{};
  std::array<float, kMaxSensorChannels> sensor_data{};
  std::array<float, kMaxSensorChannels> sensor_data_raw{};
  int num_sensor_channels{0};
  bool valid{false};
  GoalType goal_type{GoalType::kJoint};
};

// One row of the device-state / device-sensor logs, written by DataLogger.
//
// Per-tick *timing* data lives in a separate stream now — see
// rtc_base/timing/cm_timing_sample.hpp + thread_timing_*.hpp. This struct
// carries only the device-rich fields the device CSVs need.
struct LogEntry
{
  static constexpr int kMaxDevices = 8;

  double timestamp{0.0};

  // ── Shared ─────────────────────────────────────────────────────────────
  std::array<double, kTaskSpaceDim> actual_task_positions{};
  std::array<double, kTaskSpaceDim> task_goal_positions{};
  std::array<double, kTaskSpaceDim> trajectory_task_positions{};
  std::array<double, kTaskSpaceDim> trajectory_task_velocities{};
  CommandType command_type{CommandType::kPosition};

  // ── Per-device data ────────────────────────────────────────────────────
  std::array<DeviceLogSlot, kMaxDevices> devices{};
  int num_devices{0};

  // Inference output (e.g. ONNX model output) — not per-device
  std::array<float, kMaxInferenceValues> inference_output{};
  bool inference_valid{false};
  int num_inference_values{0};
};

// SpscLogBuffer is an alias of the generalized rtc::SpscQueue. Existing
// callers use SpscLogBuffer<N> / ControlLogBuffer; both names are preserved.
template<std::size_t N> using SpscLogBuffer = SpscQueue<LogEntry, N>;

constexpr std::size_t kControlLogBufferCapacity = 512;
// 512 slots ≈ 1 s of entries at 500 Hz with headroom for drain latency.
using ControlLogBuffer = SpscLogBuffer<kControlLogBufferCapacity>;

} // namespace rtc

#endif // RTC_BASE_LOG_BUFFER_HPP_
