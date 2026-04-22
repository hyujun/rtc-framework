#ifndef RTC_BASE_MPC_SOLVE_STATS_HPP_
#define RTC_BASE_MPC_SOLVE_STATS_HPP_

#include <cstdint>

// Neutral observability record for MPC solve-loop latency. Mirrors the
// shape of `rtc::mpc::MPCSolutionManager::SolveTimingStats` but lives in
// rtc_base so `rtc_controller_interface` can surface it without taking a
// dependency on rtc_mpc (which would invert the intended dependency
// graph — see agent_docs/architecture.md).
//
// The override in a controller that holds an MPCSolutionManager is
// expected to construct this struct from the manager snapshot and return
// it via `RTControllerInterface::GetMpcSolveStats()`.

namespace rtc {

struct MpcSolveStats {
  std::uint64_t count{0};   ///< lifetime solves observed (monotonic)
  std::uint32_t window{0};  ///< samples averaged (<= ring-buffer size)
  std::uint64_t last_ns{0}; ///< most recent sample
  std::uint64_t min_ns{0};  ///< window min (0 when window==0)
  std::uint64_t max_ns{0};  ///< window max
  std::uint64_t p50_ns{0};  ///< window median
  std::uint64_t p99_ns{0};  ///< window 99-th percentile
  double mean_ns{0.0};      ///< window arithmetic mean
};

} // namespace rtc

#endif // RTC_BASE_MPC_SOLVE_STATS_HPP_
