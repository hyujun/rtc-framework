#ifndef RTC_BASE_PUBLISH_BUFFER_HPP_
#define RTC_BASE_PUBLISH_BUFFER_HPP_

// Lock-free single-producer / single-consumer (SPSC) ring buffer for
// offloading ROS2 publish() calls from the RT control thread to a
// dedicated non-RT publish thread.
//
// Constraints:
//   - Exactly ONE producer thread (the 500 Hz RT loop).
//   - Exactly ONE consumer thread (the publish thread).
//   - Push() called only from the producer; Pop() called only from the consumer.
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

// Cache line size (shared with log_buffer.hpp)
#ifdef __cpp_lib_hardware_interference_size
  inline constexpr std::size_t kPublishCacheLineSize =
      std::hardware_destructive_interference_size;
#else
  inline constexpr std::size_t kPublishCacheLineSize = 64;
#endif

// Snapshot of all data needed for ROS2 publishing in one control tick.
// Populated by the RT loop (producer), consumed by the publish thread.
// ~640 bytes — fits in 10 cache lines.
struct PublishSnapshot {
  // ── Robot commands ──────────────────────────────────────────────────────
  std::array<double, kNumRobotJoints> robot_commands{};
  CommandType command_type{CommandType::kPosition};

  // ── Task position (6-DOF FK output) ────────────────────────────────────
  std::array<double, 6> actual_task_positions{};

  // ── Trajectory state (kTrajectoryState topic) ──────────────────────────
  std::array<double, kNumRobotJoints> goal_positions{};
  std::array<double, kNumRobotJoints> actual_target_positions{};
  std::array<double, kNumRobotJoints> target_velocities{};

  // ── Controller state (kControllerState topic) ──────────────────────────
  std::array<double, kNumRobotJoints> actual_positions{};
  std::array<double, kNumRobotJoints> actual_velocities{};

  // ── Hand commands ──────────────────────────────────────────────────────
  std::array<float, kNumHandMotors> hand_commands{};

  // ── JointCommand header stamp (monotonic nanoseconds) ──────────────────
  int64_t stamp_ns{0};

  // ── Active controller index (for topic config lookup) ──────────────────
  int active_controller_idx{0};

  // ── Device flags (resolved per-controller) ─────────────────────────────
  bool ur5e_enabled{false};
  bool hand_enabled{false};
  bool hand_sim_enabled{false};
};

// SPSC ring buffer of capacity N entries (N must be a power of 2).
// Identical pattern to SpscLogBuffer — see log_buffer.hpp.
template <std::size_t N>
class SpscPublishBuffer {
  static_assert(N > 0 && (N & (N - 1)) == 0, "N must be a power of 2");

 public:
  // Called from the RT thread.  Returns false (and drops the entry) if the
  // buffer is full — no blocking, no allocation.
  [[nodiscard]] bool Push(const PublishSnapshot& entry) noexcept {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) & (N - 1);

    if (next == cached_tail_) {
      cached_tail_ = tail_.load(std::memory_order_acquire);
      if (next == cached_tail_) {
        drop_count_.fetch_add(1, std::memory_order_relaxed);
        return false;
      }
    }

    buffer_[head] = entry;
    head_.store(next, std::memory_order_release);
    return true;
  }

  // Called from the publish thread.  Returns false when the buffer is empty.
  [[nodiscard]] bool Pop(PublishSnapshot& out) noexcept {
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
  alignas(kPublishCacheLineSize) std::atomic<std::size_t> head_{0};
  std::size_t cached_tail_{0};

  alignas(kPublishCacheLineSize) std::atomic<std::size_t> tail_{0};
  std::size_t cached_head_{0};

  alignas(kPublishCacheLineSize) std::atomic<uint64_t> drop_count_{0};
};

// 512 slots ≈ 1 s at 500 Hz.  Matches ControlLogBuffer capacity.
inline constexpr std::size_t kPublishBufferCapacity = 512;
using ControlPublishBuffer = SpscPublishBuffer<kPublishBufferCapacity>;

}  // namespace rtc

#endif  // RTC_BASE_PUBLISH_BUFFER_HPP_
