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

// Cache line size determination (C++17 hardware destructive interference size)
#ifdef __cpp_lib_hardware_interference_size
  constexpr std::size_t kCacheLineSize = std::hardware_destructive_interference_size;
#else
  constexpr std::size_t kCacheLineSize = 64;
#endif

// One row of the control log, split across three CSV files by the DataLogger.
// Fields are grouped by the 4-category topic convention:
//   1. Goal State — 외부 입력 목표
//   2. Current State — 센서 피드백
//   3. Control Command — 액추에이터 출력
//   4. Trajectory/Logging — 궤적 보간 내부 상태
struct LogEntry {
  double timestamp{0.0};

  // ── Timing data → timing_log.csv ───────────────────────────────────────
  double t_state_acquire_us{0.0};  // try_lock + state copy
  double t_compute_us{0.0};        // controller Compute() wall-clock
  double t_publish_us{0.0};        // cmd_pub try_lock + publish
  double t_total_us{0.0};          // entire ControlLoop() callback
  double jitter_us{0.0};           // |actual_period - expected_period|

  // ══════════════════════════════════════════════════════════════════════════
  // ── Robot joint data → robot_log.csv ───────────────────────────────────
  // ══════════════════════════════════════════════════════════════════════════

  // 카테고리 1: Goal State (외부 입력 목표)
  std::array<double, kNumRobotJoints> goal_positions{};

  // 카테고리 2: Current State (센서 피드백)
  std::array<double, kNumRobotJoints> actual_positions{};
  std::array<double, kNumRobotJoints> actual_velocities{};
  std::array<double, kNumRobotJoints> actual_torques{};        // 실제 토크
  std::array<double, 6>               actual_task_positions{};  // TCP 위치

  // 카테고리 3: Control Command (액추에이터 출력)
  std::array<double, kNumRobotJoints> robot_commands{};
  CommandType command_type{CommandType::kPosition};

  // 카테고리 4: Trajectory State (궤적 보간 내부 상태)
  std::array<double, kNumRobotJoints> trajectory_positions{};   // 궤적 보간 위치
  std::array<double, kNumRobotJoints> trajectory_velocities{};  // 궤적 보간 속도

  // ══════════════════════════════════════════════════════════════════════════
  // ── Hand data → hand_log.csv ───────────────────────────────────────────
  // ══════════════════════════════════════════════════════════════════════════
  bool hand_valid{false};

  // 카테고리 1: Goal State
  std::array<float, kNumHandMotors>  hand_goal_positions{};

  // 카테고리 2: Current State
  std::array<float, kNumHandMotors>  hand_actual_positions{};
  std::array<float, kNumHandMotors>  hand_actual_velocities{};
  std::array<int32_t, kMaxHandSensors> hand_sensors{};         // 필터링된 센서 (post-LPF)
  std::array<int32_t, kMaxHandSensors> hand_sensors_raw{};    // 원본 센서 (pre-LPF)
  int num_fingertips{kDefaultNumFingertips};                   // 실제 사용 fingertip 수

  // F/T 추론 결과 (ONNX 출력)
  std::array<float, FingertipFTState::kMaxFTValues> hand_ft_data{};
  bool hand_ft_valid{false};

  // 카테고리 3: Control Command
  std::array<float, kNumHandMotors>  hand_commands{};

  // Legacy alias — kept for backward compatibility with external tools.
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
    if (next == cached_tail_) {
      cached_tail_ = tail_.load(std::memory_order_acquire);
      if (next == cached_tail_) {
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
