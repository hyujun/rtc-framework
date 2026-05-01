#ifndef RTC_BASE_PUBLISH_BUFFER_HPP_
#define RTC_BASE_PUBLISH_BUFFER_HPP_

// Lock-free single-producer / single-consumer (SPSC) ring buffer for
// offloading ROS2 publish() calls from the RT control thread to a
// dedicated non-RT publish thread.
//
// Constraints:
//   - Exactly ONE producer thread (the 500 Hz RT loop).
//   - Exactly ONE consumer thread (the publish thread).
//   - Push() called only from the producer; Pop() called only from the
//   consumer.
//
// Design rationale:
//   DDS serialization and transport are unbounded-latency operations.
//   Moving all publish() calls off the RT path eliminates ~60-200 µs of
//   variable overhead per tick.  The publish thread consumes snapshots
//   and performs all DDS work on a non-RT core.

#include "rtc_base/concurrency/spsc_queue.hpp"
#include "rtc_base/types/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace rtc
{

// Snapshot of all data needed for ROS2 publishing in one control tick.
// Populated by the RT loop (producer), consumed by the publish thread.
struct PublishSnapshot
{
  // ── Per-group data slots (RT-safe fixed-size) ──────────────────────────
  static constexpr int kMaxGroups = 8;
  struct GroupCommandSlot
  {
    int num_channels{0};        // from controller output (command channels)
    int actual_num_channels{0}; // from device state (state channels for GUI)
    std::array<double, kMaxDeviceChannels> commands{};
    std::array<double, kMaxDeviceChannels> goal_positions{};
    std::array<double, kMaxDeviceChannels> target_positions{};
    std::array<double, kMaxDeviceChannels> target_velocities{};
    std::array<double, kMaxDeviceChannels> trajectory_positions{};
    std::array<double, kMaxDeviceChannels> trajectory_velocities{};
    std::array<double, kMaxDeviceChannels> actual_positions{};
    std::array<double, kMaxDeviceChannels> actual_velocities{};
    std::array<double, kMaxDeviceChannels> efforts{};
    // Motor-space data (hand motor encoder values)
    int num_motor_channels{0};
    std::array<double, kMaxDeviceChannels> motor_positions{};
    std::array<double, kMaxDeviceChannels> motor_velocities{};
    std::array<double, kMaxDeviceChannels> motor_efforts{};
    // Sensor data (hand device)
    std::array<int32_t, kMaxSensorChannels> sensor_data{};
    std::array<int32_t, kMaxSensorChannels> sensor_data_raw{};
    int num_sensor_channels{0};
    // Inference output
    std::array<float, kMaxInferenceValues> inference_output{};
    bool inference_valid{false};
    int num_inference_values{0};
    GoalType goal_type{GoalType::kJoint};
    // Grasp state (from controller output)
    GraspStateData grasp_state{};
    // WBC state (from controller output, TSID-based whole-body controllers)
    WbcStateData wbc_state{};
    // ToF snapshot (from controller output)
    ToFSnapshotData tof_snapshot{};
  };
  std::array<GroupCommandSlot, kMaxGroups> group_commands{};
  int num_groups{0};

  // ── Shared data (group-independent) ────────────────────────────────────
  CommandType command_type{CommandType::kPosition};
  std::array<double, kTaskSpaceDim> actual_task_positions{};
  // Per-group task goals (for kRobotTarget / kDeviceStateLog)
  std::array<std::array<double, kTaskSpaceDim>, kMaxGroups> task_goals{};

  // JointCommand header stamp (monotonic nanoseconds)
  int64_t stamp_ns{0};

  // Active controller index (for topic config lookup in publish thread)
  int active_controller_idx{0};
};

// SpscPublishBuffer is an alias of the generalized rtc::SpscQueue. Existing
// callers use SpscPublishBuffer<N> / ControlPublishBuffer; both names are
// preserved.
template<std::size_t N>
using SpscPublishBuffer = SpscQueue<PublishSnapshot, N>;

// 512 slots ≈ 1 s at 500 Hz.  Matches ControlLogBuffer capacity.
inline constexpr std::size_t kPublishBufferCapacity = 512;
using ControlPublishBuffer = SpscPublishBuffer<kPublishBufferCapacity>;

} // namespace rtc

#endif // RTC_BASE_PUBLISH_BUFFER_HPP_
