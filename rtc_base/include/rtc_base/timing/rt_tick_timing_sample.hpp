#ifndef RTC_BASE_TIMING_RT_TICK_TIMING_SAMPLE_HPP_
#define RTC_BASE_TIMING_RT_TICK_TIMING_SAMPLE_HPP_

// Unified per-tick timing payload shared by every RT / soft-RT thread that
// publishes timing CSV output (CM 500 Hz loop, MPC solve thread, future
// channels). The transport (SPSC ring) and CSV shell are generic — see
// thread_timing_producer.hpp / thread_timing_csv_logger.hpp.
//
// One row per tick is appended to a thread-specific CSV path with schema
//   t_wall_ns,tick_count,t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us
// where the first two columns are emitted by ThreadTimingCsvLogger and the
// next five are the per-phase wall-clock breakdown captured by the producer.
//
// Field semantics (held identical between CM and MPC so analysis scripts
// can join across threads):
//   t_state_us    state acquisition phase (cache load / SeqLock / etc.)
//   t_compute_us  controller Compute() / MPC Solve() phase
//   t_publish_us  publish phase (SPSC push / TripleBuffer write / etc.)
//   t_total_us    end-of-tick − start-of-tick
//   jitter_us     |actual_period − expected_period| against the previous tick
//
// Producers may leave fields zero when not applicable to their thread (e.g.
// a thread without a publish phase sets t_publish_us = 0). The schema stays
// fixed so a single set of tooling consumes every CSV.

#include "rtc_base/timing/thread_timing_csv_logger.hpp"
#include "rtc_base/timing/thread_timing_producer.hpp"
#include "rtc_base/timing/thread_timing_sample.hpp"

#include <cstddef>
#include <ostream>
#include <type_traits>

namespace rtc {

struct RtTickTimingPayload {
  double t_state_us{0.0};
  double t_compute_us{0.0};
  double t_publish_us{0.0};
  double t_total_us{0.0};
  double jitter_us{0.0};
};
static_assert(std::is_trivially_copyable_v<RtTickTimingPayload>,
              "RtTickTimingPayload must be trivially copyable");

inline void WriteRtTickTimingHeader(std::ostream &os) {
  os << ",t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us";
}

inline void WriteRtTickTimingRow(std::ostream &os,
                                 const RtTickTimingPayload &p) {
  os << ',' << p.t_state_us << ',' << p.t_compute_us << ',' << p.t_publish_us
     << ',' << p.t_total_us << ',' << p.jitter_us;
}

/// SPSC ring capacity for the CM RT loop (500 Hz). 512 slots ≈ 1 s of
/// headroom — drained at 100 Hz by the log thread.
inline constexpr std::size_t kCmTimingBufferCapacity = 512;

/// SPSC ring capacity for the MPC solve thread (≤ 100 Hz). 128 slots ≈
/// 6 s of headroom at 20 Hz, comfortably covering a 1 s drain interval.
inline constexpr std::size_t kMpcTimingBufferCapacity = 128;

using RtTickTimingSample = ThreadTimingSample<RtTickTimingPayload>;

using CmTimingBuffer =
    ThreadTimingProducer<RtTickTimingPayload, kCmTimingBufferCapacity>;

using MpcTimingBuffer =
    ThreadTimingProducer<RtTickTimingPayload, kMpcTimingBufferCapacity>;

} // namespace rtc

#endif // RTC_BASE_TIMING_RT_TICK_TIMING_SAMPLE_HPP_
