#ifndef RTC_MPC_THREAD_SOLVE_RECORD_HPP_
#define RTC_MPC_THREAD_SOLVE_RECORD_HPP_

/// @file solve_record.hpp
/// @brief The last finished solve of a `HandlerMPCThread`, for a consumer's
///        per-tick log.
///
/// The thread writes one record per solve ATTEMPT that reached a verdict, at
/// the end of its tick (after `PublishSolution`, so a published record carries
/// the plan id the consumer will sample), into a `TripleBuffer` the consumer
/// owns and hands over with `HandlerMPCThread::SetSolveRecordSink` before
/// `Start`. The consumer reads the newest with `TryAcquireLatest` /
/// `Current` — wait-free on both sides, so a reader on an RT tick is never
/// held by a preempted writer (a SeqLock reader would spin).
///
/// Which `Solve` returns write a record (outcome):
///   - no handler / phase manager / model             → kFailed, err −1
///   - snapshot dimensions or values refused          → kFailed, err
///   kStateDimMismatch
///   - cross-mode swap refused                        → kFailed, err
///   kRebuildRequired
///   - the handler's own error                        → kFailed, err its code
///   - returned after `publish_deadline_ms`           → kDeadline
///   - an online solve that started cold, unconverged → kColdWithheld
///   - the consumed segment's residual over the tol   → kWithheld
///   - published                                      → kPublished
/// and which do not: a repeated observation (`RepeatedObservationSkips` — not
/// an attempt) and a tick before the first stored state (no `Solve` call).
/// So, once the thread has stopped, the last record's `counts` equal the
/// thread's counters: kFailed = `FailedSolves`, kDeadline =
/// `DeadlineMissCount`, kColdWithheld = `ColdWithheldCount`, kWithheld =
/// `WithheldCount`, kPublished = the plans this thread published.
///
/// Within one thread run `seq` counts records from 1; 0 with kNone is "no
/// record yet" (what the consumer should store before `Start`). A consumer
/// sees the NEWEST record, not every one: two attempts can finish between
/// two of its reads, which `seq` and `counts` show.

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rtc::mpc {

/// The verdict of one solve attempt. Values are the log's integers.
enum class SolveOutcome : std::int8_t {
  kNone = -1,  ///< no record yet
  kPublished = 0,
  kWithheld = 1,      ///< residual of the consumed segment over `publish_prim_tol`
  kDeadline = 2,      ///< returned after `publish_deadline_ms`
  kColdWithheld = 3,  ///< online, started cold, not converged
  kFailed = 4,        ///< no iterates (see `err`)
};

inline constexpr std::size_t kSolveOutcomeCount = 5;  ///< kPublished .. kFailed

struct SolveRecord {
  std::uint64_t seq{0};  ///< records of this thread run, 1 = first; 0 = none
  SolveOutcome outcome{SolveOutcome::kNone};
  /// `MPCSolveError` of a kFailed attempt (−1: no handler), 0 otherwise.
  std::int32_t err{0};
  // The iterates' summary — 0 / false / NaN on a kFailed attempt (it has none).
  std::int32_t iterations{0};
  bool converged{false};
  bool cold_start{false};
  double prim_inf{0.0};       ///< `residuals.prim_inf` of the whole horizon
  double consumed_prim{0.0};  ///< max node residual of the segment the consumer
                              ///< plays (the publish rule's figure)
  /// From this thread's `Solve` entry to the verdict [µs].
  double compute_us{0.0};
  std::uint64_t state_sequence{0};  ///< `MPCStateSnapshot::sequence` of the attempt
  std::uint32_t state_epoch{0};     ///< `MPCStateSnapshot::state_epoch` of the attempt
  std::uint64_t plan_id{0};         ///< the manager's id when kPublished, 0 otherwise
  /// Records by outcome so far in this run (index = outcome value).
  std::array<std::uint64_t, kSolveOutcomeCount> counts{};
};

static_assert(std::is_trivially_copyable_v<SolveRecord>,
              "SolveRecord rides a TripleBuffer and must be trivially copyable");

}  // namespace rtc::mpc

#endif  // RTC_MPC_THREAD_SOLVE_RECORD_HPP_
