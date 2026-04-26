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

#include "rtc_base/types/types.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <new>

namespace rtc {

// Snapshot of all data needed for ROS2 publishing in one control tick.
// Populated by the RT loop (producer), consumed by the publish thread.
struct PublishSnapshot {
  // ── Per-group data slots (RT-safe fixed-size) ──────────────────────────
  static constexpr int kMaxGroups = 8;
  struct GroupCommandSlot {
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

// SPSC ring buffer of capacity N entries (N must be a power of 2).
// Identical pattern to SpscLogBuffer — see log_buffer.hpp.
template <std::size_t N> class SpscPublishBuffer {
  static_assert(N > 0 && (N & (N - 1)) == 0, "N must be a power of 2");

public:
  // Called from the RT thread.  Returns false (and drops the entry) if the
  // buffer is full — no blocking, no allocation.
  [[nodiscard]] bool Push(const PublishSnapshot &entry) noexcept {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) & (N - 1);

    if (next == cached_tail_) [[unlikely]] {
      cached_tail_ = tail_.load(std::memory_order_acquire);
      if (next == cached_tail_) [[unlikely]] {
        drop_count_.fetch_add(1, std::memory_order_relaxed);
        return false;
      }
    }

    buffer_[head] = entry;
    head_.store(next, std::memory_order_release);
    return true;
  }

  // Called from the publish thread.  Returns false when the buffer is empty.
  [[nodiscard]] bool Pop(PublishSnapshot &out) noexcept {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);

    if (tail == cached_head_) {
      cached_head_ = head_.load(std::memory_order_acquire);
      if (tail == cached_head_) {
        return false;
      }
    }

    out = buffer_[tail];
    tail_.store((tail + 1) & (N - 1), std::memory_order_release);
    return true;
  }

  [[nodiscard]] uint64_t drop_count() const noexcept {
    return drop_count_.load(std::memory_order_relaxed);
  }

private:
  std::array<PublishSnapshot, N> buffer_{};

  // Separate cache lines to avoid false sharing between producer and consumer.
  alignas(kCacheLineSize) std::atomic<std::size_t> head_{0};
  std::size_t cached_tail_{0};

  alignas(kCacheLineSize) std::atomic<std::size_t> tail_{0};
  std::size_t cached_head_{0};

  alignas(kCacheLineSize) std::atomic<uint64_t> drop_count_{0};
};

// 512 slots ≈ 1 s at 500 Hz.  Matches ControlLogBuffer capacity.
inline constexpr std::size_t kPublishBufferCapacity = 512;
using ControlPublishBuffer = SpscPublishBuffer<kPublishBufferCapacity>;

} // namespace rtc

#endif // RTC_BASE_PUBLISH_BUFFER_HPP_
