#ifndef RTC_CONTROLLER_MANAGER_CONTROLLER_TIMING_PROFILER_HPP_
#define RTC_CONTROLLER_MANAGER_CONTROLLER_TIMING_PROFILER_HPP_

// ── ControllerTimingProfiler ───────────────────────────────────────────────────
//
// Measures the wall-clock time of RTControllerInterface::Compute() calls and
// maintains a lock-free running histogram + summary statistics.
//
// Intended usage (inside CustomController::ControlLoop()):
//
//   const auto output = timing_profiler_.MeasuredCompute(*controller_, state);
//
// Every 1 000 iterations print a summary:
//
//   RCLCPP_INFO(get_logger(), "%s",
//       timing_profiler_.Summary(controller_->Name().data()).c_str());
//
// Thread safety:
//   MeasuredCompute() / Update() are called ONLY from the RT control thread
//   (single producer).  GetStats() / Summary() may be called from any thread;
//   they use relaxed atomics so they may see slightly stale values, which is
//   acceptable for diagnostic statistics.
//
// Timing accuracy:
//   kFreeRun mode  — measured time includes OS scheduling jitter.
//   kSyncStep mode — step latency ≈ pure Compute() time (1:1 sync).

#include "rtc_base/timing/timing_profiler_base.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>

namespace rtc
{

class ControllerTimingProfiler
  : public TimingProfilerBase<20, 100, 2000> {
public:
  // Re-export base types under the names existing callers expect
  using Stats = BaseStats;

  // ── Core measurement ─────────────────────────────────────────────────────────

  [[nodiscard]] ControllerOutput MeasuredCompute(
    RTControllerInterface & ctrl,
    const ControllerState & state) noexcept
  {
    const auto t0 = std::chrono::steady_clock::now();
    auto output = ctrl.Compute(state);
    const auto t1 = std::chrono::steady_clock::now();

    const double us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
    UpdateTotal(us);
    return output;
  }

  // ── Statistics ───────────────────────────────────────────────────────────────

  [[nodiscard]] Stats GetStats() const noexcept
  {
    return GetBaseStats();
  }

  [[nodiscard]] double LastComputeUs() const noexcept
  {
    return LastUs();
  }

  void Reset() noexcept
  {
    ResetBase();
  }

  // Human-readable summary line suitable for RCLCPP_INFO.
  [[nodiscard]] std::string Summary(const std::string & ctrl_name) const noexcept
  {
    const Stats s = GetStats();
    if (s.count == 0) {return ctrl_name + " timing: no data";}

    const double over_pct = static_cast<double>(s.over_budget) * 100.0 /
      static_cast<double>(s.count);
    char buf[512];
    std::snprintf(
        buf, sizeof(buf),
        "%s timing: count=%lu  mean=%.1f\xc2\xb5s  min=%.1f\xc2\xb5s"
        "  max=%.1f\xc2\xb5s  p95=%.1f\xc2\xb5s  p99=%.1f\xc2\xb5s"
        "  over_budget=%lu (%.1f%%)",
        ctrl_name.c_str(),
        static_cast<unsigned long>(s.count),
        s.mean_us, s.min_us, s.max_us,
        s.p95_us, s.p99_us,
        static_cast<unsigned long>(s.over_budget),
        over_pct);
    return std::string(buf);
  }
};

}  // namespace rtc

#endif  // RTC_CONTROLLER_MANAGER_CONTROLLER_TIMING_PROFILER_HPP_
