#ifndef RTC_BASE_TIMING_THREAD_TIMING_SAMPLE_HPP_
#define RTC_BASE_TIMING_THREAD_TIMING_SAMPLE_HPP_

// Generic per-tick timing sample. One instance is pushed onto a
// ThreadTimingProducer's SPSC ring every iteration of an RT or soft-RT
// thread (CM RT loop @ control_rate, MPC ~20 Hz loop, hand UDP cycle, ONNX
// inference, etc.). A non-RT consumer drains the ring and writes one CSV row per
// sample — preserving full per-tick granularity.
//
// The sample is split into a fixed-shape header (`t_wall_ns`, `tick_count`)
// and a thread-specific `Payload`. The header columns are common across
// every CSV produced by ThreadTimingCsvLogger so cross-thread analysis
// scripts can assume their presence; payload columns are entirely up to
// the producing thread.
//
// Constraints on `Payload`:
//   - Must be trivially copyable (RT-safe SPSC push/pop).
//   - Should be small (kept by value inside SpscQueue's std::array).
//
// Header-only.

#include <cstdint>
#include <type_traits>

namespace rtc {

template <typename Payload>
struct ThreadTimingSample {
  static_assert(std::is_trivially_copyable_v<Payload>,
                "ThreadTimingSample Payload must be trivially copyable");

  /// steady_clock-epoch nanoseconds at the moment the sample was pushed.
  std::uint64_t t_wall_ns{0};

  /// Lifetime sequence number (post-increment of producer's internal
  /// counter). Useful for detecting drops in the CSV stream.
  std::uint64_t tick_count{0};

  /// Thread-specific timing data (e.g. phase breakdown, solve duration).
  Payload payload{};
};

}  // namespace rtc

#endif  // RTC_BASE_TIMING_THREAD_TIMING_SAMPLE_HPP_
