#ifndef RTC_BASE_TIMING_THREAD_TIMING_PRODUCER_HPP_
#define RTC_BASE_TIMING_THREAD_TIMING_PRODUCER_HPP_

// Producer-side helper for per-tick timing CSV logging.
//
// Composes:
//   - A SpscQueue<ThreadTimingSample<Payload>, N> for lock-free transfer.
//   - A monotonic tick counter that the producer increments once per push.
//   - Drain callbacks the non-RT consumer uses to write CSV rows.
//
// Same pattern that previously lived ad-hoc in MPCSolutionManager (MPC tick
// → SPSC → MpcSolveSampleBuffer) and in RtControllerNode (RT tick →
// ControlLogBuffer → DataLogger). One template now covers both, plus any
// future RT/soft-RT thread that wants per-tick CSV output.
//
// Threading contract:
//   - Push() called from the producer thread only.
//   - Drain() / DropCount() called from the consumer thread only.
//   - Producer and consumer may differ; SpscQueue handles the publication.
//
// Header-only.

#include "rtc_base/concurrency/spsc_queue.hpp"
#include "rtc_base/timing/thread_timing_sample.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace rtc
{

template<typename Payload, std::size_t N>
class ThreadTimingProducer {
public:
  using Sample = ThreadTimingSample<Payload>;
  static constexpr std::size_t kCapacity = N;

  /// Push a sample with the current steady_clock timestamp and the next
  /// monotonic tick count. Wait-free; on a full ring the sample is dropped
  /// and `DropCount()` increments. RT-safe.
  [[nodiscard]] bool Push(const Payload & payload) noexcept
  {
    Sample s{};
    s.t_wall_ns = NowNs();
    s.tick_count = ++tick_count_;
    s.payload = payload;
    return queue_.Push(s);
  }

  /// Drain pending samples in FIFO order. Returns the number drained.
  /// Non-RT.
  template<typename Fn> std::size_t Drain(Fn && on_sample) noexcept
  {
    std::size_t n = 0;
    Sample s{};
    while (queue_.Pop(s)) {
      on_sample(s);
      ++n;
    }
    return n;
  }

  /// Lifetime count of samples dropped due to a full ring. Non-RT.
  [[nodiscard]] std::uint64_t DropCount() const noexcept
  {
    return queue_.drop_count();
  }

  /// Producer-side tick counter. Same value the most recent Push set on
  /// `tick_count`. Useful for the producer when computing rates or
  /// coordinating periodic side-effects (INFO summary every N ticks).
  [[nodiscard]] std::uint64_t LastTickCount() const noexcept
  {
    return tick_count_;
  }

private:
  static std::uint64_t NowNs() noexcept
  {
    return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
      .count());
  }

  // tick_count_ is owned by the producer thread (single writer); the
  // consumer reads it only via the published Sample.tick_count, never
  // directly, so no atomic is needed.
  std::uint64_t tick_count_{0};

  SpscQueue<Sample, N> queue_;
};

} // namespace rtc

#endif // RTC_BASE_TIMING_THREAD_TIMING_PRODUCER_HPP_
