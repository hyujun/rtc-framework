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
//   - Push() called only from the producer; Pop() called only from the consumer.

#include "rtc_base/types/types.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <new>

namespace rtc {

// Per-device log data slot (used in LogEntry for each device group)
struct DeviceLogSlot {
  int num_channels{0};
  std::array<double, kMaxDeviceChannels> goal_positions{};
  std::array<double, kMaxDeviceChannels> actual_positions{};
  std::array<double, kMaxDeviceChannels> actual_velocities{};
  std::array<double, kMaxDeviceChannels> efforts{};
  std::array<double, kMaxDeviceChannels> commands{};
  std::array<double, kMaxDeviceChannels> trajectory_positions{};
  std::array<double, kMaxDeviceChannels> trajectory_velocities{};
  std::array<float, kMaxSensorChannels>  sensor_data{};
  std::array<float, kMaxSensorChannels>  sensor_data_raw{};
  int num_sensor_channels{0};
  bool valid{false};
  GoalType goal_type{GoalType::kJoint};
};

// One row of the control log, split across CSV files by the DataLogger.
struct LogEntry {
  static constexpr int kMaxDevices = 4;

  double timestamp{0.0};

  // ── Timing data → timing_log.csv ───────────────────────────────────────
  double t_state_acquire_us{0.0};
  double t_compute_us{0.0};
  double t_publish_us{0.0};
  double t_total_us{0.0};
  double jitter_us{0.0};

  // ── Shared ─────────────────────────────────────────────────────────────
  std::array<double, 6> actual_task_positions{};
  std::array<double, 6> task_goal_positions{};
  std::array<double, 6> trajectory_task_positions{};
  std::array<double, 6> trajectory_task_velocities{};
  CommandType command_type{CommandType::kPosition};

  // ── Per-device data ────────────────────────────────────────────────────
  std::array<DeviceLogSlot, kMaxDevices> devices{};
  int num_devices{0};

  // Inference output (e.g. ONNX model output) — not per-device
  std::array<float, kMaxInferenceValues> inference_output{};
  bool inference_valid{false};
  int num_inference_values{0};

  // Legacy alias
  [[nodiscard]] double compute_time_us() const noexcept { return t_compute_us; }
};

// SPSC ring buffer of capacity N entries (N must be a power of 2).
// head_ is owned by the producer; tail_ is owned by the consumer.
// Both indices are on separate cache lines to prevent false sharing.
template <std::size_t N>
class SpscLogBuffer {
  // N must be a power of 2 to use bitwise AND modulo.
  static_assert(N > 0 && (N & (N - 1)) == 0, "N must be a power of 2");

 public:
  // Called from the RT thread. Returns false (and drops the entry) if the
  // buffer is full — no blocking, no allocation.
  [[nodiscard]] bool Push(const LogEntry& entry) noexcept {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) & (N - 1);  // Fast modulo using bitwise AND

    // Use cached tail_ index to minimize atomic operations and cache bouncing
    if (next == cached_tail_) [[unlikely]] {
      cached_tail_ = tail_.load(std::memory_order_acquire);
      if (next == cached_tail_) [[unlikely]] {
        drop_count_.fetch_add(1, std::memory_order_relaxed);
        return false;  // buffer full — entry dropped
      }
    }

    buffer_[head] = entry;
    head_.store(next, std::memory_order_release);
    return true;
  }

  // Called from the log thread. Returns false when the buffer is empty.
  [[nodiscard]] bool Pop(LogEntry& out) noexcept {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);

    // Use cached head_ index to minimize atomic operations and cache bouncing
    if (tail == cached_head_) {
      cached_head_ = head_.load(std::memory_order_acquire);
      if (tail == cached_head_) {
        return false;  // buffer empty
      }
    }

    out = buffer_[tail];
    tail_.store((tail + 1) & (N - 1), std::memory_order_release);
    return true;
  }

  // Number of entries dropped due to buffer-full since construction.
  // Safe to call from any thread (relaxed atomic).
  [[nodiscard]] uint64_t drop_count() const noexcept {
    return drop_count_.load(std::memory_order_relaxed);
  }

 private:
  std::array<LogEntry, N> buffer_{};

  // Separate cache lines to avoid false sharing between producer and consumer groups.
  alignas(kCacheLineSize) std::atomic<std::size_t> head_{0};  // written by producer
  std::size_t cached_tail_{0};                                // producer local cache

  alignas(kCacheLineSize) std::atomic<std::size_t> tail_{0};  // written by consumer
  std::size_t cached_head_{0};                                // consumer local cache

  // Drop counter — written by producer only, readable from any thread.
  // Placed on its own cache line to avoid interfering with the hot
  // producer (head_/cached_tail_) or consumer (tail_/cached_head_) groups.
  alignas(kCacheLineSize) std::atomic<uint64_t> drop_count_{0};
};

constexpr std::size_t kControlLogBufferCapacity = 512;
// 512 slots ≈ 1 s of entries at 500 Hz with headroom for drain latency.
using ControlLogBuffer = SpscLogBuffer<kControlLogBufferCapacity>;

}  // namespace rtc

#endif  // RTC_BASE_LOG_BUFFER_HPP_
