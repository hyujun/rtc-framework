#ifndef RTC_BASE_TIMING_CM_TIMING_SAMPLE_HPP_
#define RTC_BASE_TIMING_CM_TIMING_SAMPLE_HPP_

// CM RT-loop-specific payload + ThreadTiming{Producer,CsvLogger} aliases for
// the per-tick timing CSV. Same generic transport as MPC (see
// thread_timing_producer.hpp / thread_timing_csv_logger.hpp); only the
// payload schema differs.
//
// One row per RT tick is appended to
//   <session>/controller/timing_log.csv
// with schema
//   t_wall_ns,tick_count,t_state_acquire_us,t_compute_us,t_publish_us,
//                        t_total_us,jitter_us
//
// The first two columns are emitted by ThreadTimingCsvLogger; the next
// five are the per-phase wall-clock breakdown captured by RtControllerNode
// each iteration of its 500 Hz loop.

#include "rtc_base/timing/thread_timing_csv_logger.hpp"
#include "rtc_base/timing/thread_timing_producer.hpp"
#include "rtc_base/timing/thread_timing_sample.hpp"

#include <cstddef>
#include <ostream>
#include <type_traits>

namespace rtc
{

struct CmTimingPayload
{
  double t_state_acquire_us{0.0};
  double t_compute_us{0.0};
  double t_publish_us{0.0};
  double t_total_us{0.0};
  double jitter_us{0.0};
};
static_assert(std::is_trivially_copyable_v<CmTimingPayload>,
              "CmTimingPayload must be trivially copyable");

inline void WriteCmTimingHeader(std::ostream & os)
{
  os << ",t_state_acquire_us,t_compute_us,t_publish_us,t_total_us,jitter_us";
}

inline void WriteCmTimingRow(std::ostream & os, const CmTimingPayload & p)
{
  os << ',' << p.t_state_acquire_us << ',' << p.t_compute_us << ','
     << p.t_publish_us << ',' << p.t_total_us << ',' << p.jitter_us;
}

/// SPSC ring capacity for CM RT loop. 512 slots ≈ 1 s of headroom at
/// 500 Hz, matching the original ControlLogBuffer capacity.
inline constexpr std::size_t kCmTimingBufferCapacity = 512;

using CmTimingBuffer =
  ThreadTimingProducer<CmTimingPayload, kCmTimingBufferCapacity>;

using CmTimingSample = ThreadTimingSample<CmTimingPayload>;

} // namespace rtc

#endif // RTC_BASE_TIMING_CM_TIMING_SAMPLE_HPP_
