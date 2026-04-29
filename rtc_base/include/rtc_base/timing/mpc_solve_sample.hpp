#ifndef RTC_BASE_TIMING_MPC_SOLVE_SAMPLE_HPP_
#define RTC_BASE_TIMING_MPC_SOLVE_SAMPLE_HPP_

// MPC-specific payload + ThreadTiming{Producer,CsvLogger} aliases for the
// per-MPC-tick timing CSV. The transport (SPSC ring) and CSV formatting
// shell are generic — see thread_timing_producer.hpp /
// thread_timing_csv_logger.hpp.
//
// One row per MPC solve is appended to
//   <session>/controllers/<config_key>/mpc_solve_timing.csv
// with schema
//   t_wall_ns,tick_count,solve_ns
// The first two columns are emitted by ThreadTimingCsvLogger; the third
// is the only payload column for MPC.

#include "rtc_base/timing/thread_timing_csv_logger.hpp"
#include "rtc_base/timing/thread_timing_producer.hpp"
#include "rtc_base/timing/thread_timing_sample.hpp"

#include <cstddef>
#include <cstdint>
#include <ostream>
#include <type_traits>

namespace rtc
{

/// Single payload column for MPC: solve duration in nanoseconds.
struct MpcTimingPayload
{
  std::uint64_t solve_ns{0};
};
static_assert(std::is_trivially_copyable_v<MpcTimingPayload>,
              "MpcTimingPayload must be trivially copyable");

/// Header writer for MpcTimingPayload — emits the leading-comma payload
/// column names (",solve_ns").
inline void WriteMpcTimingHeader(std::ostream & os) {os << ",solve_ns";}

/// Row writer for MpcTimingPayload — emits the leading-comma payload
/// values (",<solve_ns>").
inline void WriteMpcTimingRow(std::ostream & os, const MpcTimingPayload & p)
{
  os << ',' << p.solve_ns;
}

/// SPSC ring capacity for MPC solve samples. 128 slots ≈ 6 s of headroom
/// at 20 Hz, which comfortably covers a 1 s drain interval. Increase if
/// MPC frequency rises significantly above 100 Hz with the same drain
/// cadence.
inline constexpr std::size_t kMpcSolveSampleBufferCapacity = 128;

using MpcSolveSampleBuffer =
  ThreadTimingProducer<MpcTimingPayload, kMpcSolveSampleBufferCapacity>;

using MpcSolveSample = ThreadTimingSample<MpcTimingPayload>;

} // namespace rtc

#endif // RTC_BASE_TIMING_MPC_SOLVE_SAMPLE_HPP_
