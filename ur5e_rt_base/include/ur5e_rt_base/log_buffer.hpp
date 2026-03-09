#ifndef UR5E_RT_BASE_LOG_BUFFER_HPP_
#define UR5E_RT_BASE_LOG_BUFFER_HPP_

// Lock-free single-producer / single-consumer (SPSC) ring buffer for
// transferring log entries from the RT control thread (producer) to the
// logging thread (consumer) without any blocking or heap allocation on the
// RT path.
//
// Constraints:
//   - Exactly ONE producer thread (the 500 Hz RT loop).
//   - Exactly ONE consumer thread (the log drain timer, Core 4).
//   - Push() called only from the producer; Pop() called only from the consumer.

#include "ur5e_rt_base/types.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <new>

namespace ur5e_rt_controller {

// Cache line size determination (C++17 hardware destructive interference size)
#ifdef __cpp_lib_hardware_interference_size
  constexpr std::size_t kCacheLineSize = std::hardware_destructive_interference_size;
#else
  constexpr std::size_t kCacheLineSize = 64;
#endif

// One row of the control log CSV.
struct LogEntry {
  double timestamp{0.0};
  std::array<double, kNumRobotJoints> current_positions{};
  std::array<double, kNumRobotJoints> target_positions{};
  std::array<double, kNumRobotJoints> commands{};
  // Wall-clock duration of the most recent Compute() call (µs).
  // Populated by ControllerTimingProfiler; zero when profiling is disabled.
  double compute_time_us{0.0};
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
    if (next == cached_tail_) {
      cached_tail_ = tail_.load(std::memory_order_acquire);
      if (next == cached_tail_) {
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

 private:
  std::array<LogEntry, N> buffer_{};

  // Separate cache lines to avoid false sharing between producer and consumer groups.
  alignas(kCacheLineSize) std::atomic<std::size_t> head_{0};  // written by producer
  std::size_t cached_tail_{0};                                // producer local cache

  alignas(kCacheLineSize) std::atomic<std::size_t> tail_{0};  // written by consumer
  std::size_t cached_head_{0};                                // consumer local cache
};

constexpr std::size_t kControlLogBufferCapacity = 512;
// 512 slots ≈ 1 s of entries at 500 Hz with headroom for drain latency.
using ControlLogBuffer = SpscLogBuffer<kControlLogBufferCapacity>;

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_BASE_LOG_BUFFER_HPP_
