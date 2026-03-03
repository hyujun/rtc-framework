#ifndef UR5E_RT_CONTROLLER_LOG_BUFFER_HPP_
#define UR5E_RT_CONTROLLER_LOG_BUFFER_HPP_

// Lock-free single-producer / single-consumer (SPSC) ring buffer for
// transferring log entries from the RT control thread (producer) to the
// logging thread (consumer) without any blocking or heap allocation on the
// RT path.
//
// Constraints:
//   - Exactly ONE producer thread (the 500 Hz RT loop).
//   - Exactly ONE consumer thread (the log drain timer, Core 4).
//   - Push() called only from the producer; Pop() called only from the consumer.

#include "ur5e_rt_controller/rt_controller_interface.hpp"

#include <array>
#include <atomic>
#include <cstddef>

namespace ur5e_rt_controller {

// One row of the control log CSV.
struct LogEntry {
  double timestamp;
  std::array<double, kNumRobotJoints> current_positions;
  std::array<double, kNumRobotJoints> target_positions;
  std::array<double, kNumRobotJoints> commands;
};

// SPSC ring buffer of capacity N entries (N must be a power of 2).
// head_ is owned by the producer; tail_ is owned by the consumer.
// Both indices are on separate cache lines to prevent false sharing.
template <std::size_t N>
class SpscLogBuffer {
 public:
  // Called from the RT thread. Returns false (and drops the entry) if the
  // buffer is full — no blocking, no allocation.
  [[nodiscard]] bool Push(const LogEntry& entry) noexcept {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) % N;
    if (next == tail_.load(std::memory_order_acquire)) {
      return false;  // buffer full — entry dropped
    }
    buffer_[head] = entry;
    head_.store(next, std::memory_order_release);
    return true;
  }

  // Called from the log thread. Returns false when the buffer is empty.
  [[nodiscard]] bool Pop(LogEntry& out) noexcept {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      return false;  // buffer empty
    }
    out = buffer_[tail];
    tail_.store((tail + 1) % N, std::memory_order_release);
    return true;
  }

 private:
  std::array<LogEntry, N> buffer_{};

  // Separate cache lines to avoid false sharing between producer and consumer.
  alignas(64) std::atomic<std::size_t> head_{0};  // written by producer
  alignas(64) std::atomic<std::size_t> tail_{0};  // written by consumer
};

// 512 slots ≈ 1 s of entries at 500 Hz with headroom for drain latency.
using ControlLogBuffer = SpscLogBuffer<512>;

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_LOG_BUFFER_HPP_
