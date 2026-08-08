#ifndef RTC_BASE_TIMING_RT_TICK_TIMING_SAMPLE_HPP_
#define RTC_BASE_TIMING_RT_TICK_TIMING_SAMPLE_HPP_

// Unified per-tick timing payload shared by every RT / soft-RT thread that
// publishes timing CSV output (CM RT loop @ control_rate, MPC solve thread,
// hand UDP EventLoop, future channels). The transport (SPSC ring) and CSV shell are generic — see
// thread_timing_producer.hpp / thread_timing_csv_logger.hpp.
//
// One row per tick is appended to a thread-specific CSV path with schema
//   t_wall_ns,tick_count,run_id,t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us
// where the first three columns are emitted by ThreadTimingCsvLogger — run_id
// separates two starts that share one minute-resolution session dir, which the
// append mode otherwise merges silently (#376) — and the next five are the
// per-phase wall-clock breakdown captured by the producer.
//
// Field semantics (held identical between CM and MPC so analysis scripts
// can join across threads):
//   t_state_us    state acquisition phase (cache load / SeqLock / etc.)
//   t_compute_us  controller Compute() / MPC Solve() phase
//   t_publish_us  publish phase (SPSC push / TripleBuffer write / etc.)
//   t_total_us    end-of-tick − start-of-tick
//   jitter_us     |actual_period − expected_period| against the previous tick;
//                 producers with non-deadline wakeups (e.g. CM sim mode that
//                 blocks on a CV until /joint_states arrives) emit 0.0 — see
//                 PeriodicRtThread::JitterMeaningful()
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

inline void WriteRtTickTimingHeader(std::ostream& os) {
  os << ",t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us";
}

inline void WriteRtTickTimingRow(std::ostream& os, const RtTickTimingPayload& p) {
  os << ',' << p.t_state_us << ',' << p.t_compute_us << ',' << p.t_publish_us << ',' << p.t_total_us
     << ',' << p.jitter_us;
}

/// SPSC ring capacity for the CM RT loop. 512 slots ≈ 1 s of headroom at the
/// default 500 Hz `control_rate` (proportionally less at higher rates; e.g.
/// ~250 ms at 2 kHz). Drained at 100 Hz by the log thread.
inline constexpr std::size_t kCmTimingBufferCapacity = 512;

/// SPSC ring capacity for the MPC solve thread (≤ 100 Hz). 128 slots ≈
/// 6 s of headroom at 20 Hz, comfortably covering a 1 s drain interval.
inline constexpr std::size_t kMpcTimingBufferCapacity = 128;

/// SPSC ring capacity for the hand UDP EventLoop (typically driven by the
/// CM at `control_rate`, default 500 Hz). 512 slots ≈ 1 s of headroom at
/// 500 Hz; drained at 1 Hz by the node's aux timer.
inline constexpr std::size_t kHandUdpTimingBufferCapacity = 512;

/// SPSC ring capacity for the rt_callback lane (issue #349). Sized above the
/// CM lane because this producer is fed by EVERY device state callback on the
/// thread, not one periodic tick: an arm joint lane plus a hand joint / motor /
/// sensor triple all push here. 2048 slots ≈ 1 s of headroom for four lanes at
/// 500 Hz; drained at 100 Hz alongside cm_timing_log.csv.
inline constexpr std::size_t kRtCallbackTimingBufferCapacity = 2048;

using RtTickTimingSample = ThreadTimingSample<RtTickTimingPayload>;

using CmTimingBuffer = ThreadTimingProducer<RtTickTimingPayload, kCmTimingBufferCapacity>;

using MpcTimingBuffer = ThreadTimingProducer<RtTickTimingPayload, kMpcTimingBufferCapacity>;

using HandUdpTimingBuffer = ThreadTimingProducer<RtTickTimingPayload, kHandUdpTimingBufferCapacity>;

/// Per-callback timing for the rt_callback thread (SCHED_FIFO 70, layout v4.1
/// slot 2). Single-producer holds because CM hands every backend the SAME
/// MutuallyExclusive callback group, so the lanes are serialised onto one
/// thread — the same invariant the backends' SeqLock single-writer rule
/// already leans on. Field mapping for this lane:
///
///   t_state_us    decode + SeqLock store (callback entry → NotifyStateReady)
///   t_compute_us  0 — this lane runs no control law
///   t_publish_us  the mailbox hand-off (dirty bit + eventfd), 0 on lanes that
///                 do not notify
///   t_total_us    whole callback span — the numerator of slot-2 duty
///   jitter_us     0 — arrivals are event-driven, not deadline-scheduled, so
///                 the schema's "non-deadline wakeup producers emit 0" rule
///                 applies. Dispatch cadence is recoverable from consecutive
///                 `t_wall_ns` deltas, which every row already carries.
using RtCallbackTimingBuffer =
    ThreadTimingProducer<RtTickTimingPayload, kRtCallbackTimingBufferCapacity>;

}  // namespace rtc

#endif  // RTC_BASE_TIMING_RT_TICK_TIMING_SAMPLE_HPP_
