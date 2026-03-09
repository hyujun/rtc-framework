#ifndef UR5E_RT_CONTROLLER_CONTROLLER_TIMING_PROFILER_HPP_
#define UR5E_RT_CONTROLLER_CONTROLLER_TIMING_PROFILER_HPP_

// ── Includes: project first, then C++ stdlib ──────────────────────────────────
#include "ur5e_rt_controller/rt_controller_interface.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

namespace ur5e_rt_controller
{

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
//
class ControllerTimingProfiler {
public:
  // Histogram: 0–2 000 µs range, 100 µs buckets.
  // Bucket kBuckets holds all samples ≥ 2 000 µs (overflow).
  static constexpr int    kBuckets = 20;
  static constexpr int    kBucketWidthUs = 100;
  static constexpr double kBudgetUs = 2000.0;       // 500 Hz = 2 ms budget

  struct Stats
  {
    uint64_t count{0};
    double   min_us{0.0};
    double   max_us{0.0};
    double   mean_us{0.0};
    double   stddev_us{0.0};
    double   p95_us{0.0};  // approximate — histogram-based
    double   p99_us{0.0};  // approximate — histogram-based
    double   last_us{0.0};
    uint64_t over_budget{0};  // samples that exceeded kBudgetUs
    std::array<uint64_t, kBuckets + 1> histogram{};
  };

  // ── Core measurement ─────────────────────────────────────────────────────────

  // Wrap Compute() with wall-clock timing, update statistics, and return output.
  // Must be called from a single thread (the RT control loop).
  [[nodiscard]] ControllerOutput MeasuredCompute(
    RTControllerInterface & ctrl,
    const ControllerState & state) noexcept
  {
    const auto t0 = std::chrono::steady_clock::now();
    auto output = ctrl.Compute(state);
    const auto t1 = std::chrono::steady_clock::now();

    const double us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
    Update(us);
    return output;
  }

  // ── Statistics ───────────────────────────────────────────────────────────────

  // Snapshot of current statistics (may be slightly stale — relaxed atomics).
  [[nodiscard]] Stats GetStats() const noexcept
  {
    Stats s;
    s.count = count_.load(std::memory_order_relaxed);
    s.min_us = min_us_.load(std::memory_order_relaxed);
    s.max_us = max_us_.load(std::memory_order_relaxed);
    s.last_us = last_us_.load(std::memory_order_relaxed);
    s.over_budget = over_budget_.load(std::memory_order_relaxed);

    if (s.count > 0) {
      s.mean_us = sum_us_.load(std::memory_order_relaxed) /
        static_cast<double>(s.count);
      const double var =
        sum_sq_us_.load(std::memory_order_relaxed) /
        static_cast<double>(s.count) -
        s.mean_us * s.mean_us;
      s.stddev_us = (var > 0.0) ? std::sqrt(var) : 0.0;
    }

    for (int b = 0; b <= kBuckets; ++b) {
      s.histogram[static_cast<std::size_t>(b)] =
        histogram_[static_cast<std::size_t>(b)].load(
              std::memory_order_relaxed);
    }
    ComputePercentiles(s);
    return s;
  }

  // Most recent single Compute() duration in µs.
  [[nodiscard]] double LastComputeUs() const noexcept
  {
    return last_us_.load(std::memory_order_relaxed);
  }

  void Reset() noexcept
  {
    count_.store(0, std::memory_order_relaxed);
    min_us_.store(1e9, std::memory_order_relaxed);
    max_us_.store(0.0, std::memory_order_relaxed);
    last_us_.store(0.0, std::memory_order_relaxed);
    sum_us_.store(0.0, std::memory_order_relaxed);
    sum_sq_us_.store(0.0, std::memory_order_relaxed);
    over_budget_.store(0, std::memory_order_relaxed);
    for (auto & b : histogram_) {
      b.store(0, std::memory_order_relaxed);
    }
  }

  // Human-readable summary line suitable for RCLCPP_INFO.
  // Example output:
  //   PDController timing: count=1000  mean=3.2µs  min=2.1µs  max=18.4µs
  //     p95=5.6µs  p99=9.2µs  over_budget=0 (0.0%)
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

private:
  // ── Internal update (called from single RT thread) ───────────────────────────

  void Update(double us) noexcept
  {
    count_.fetch_add(1, std::memory_order_relaxed);

    AtomicMin(min_us_, us);
    AtomicMax(max_us_, us);
    last_us_.store(us, std::memory_order_relaxed);

    // Non-atomic RMW: safe because Update() is single-producer.
    sum_us_.store(
        sum_us_.load(std::memory_order_relaxed) + us,
        std::memory_order_relaxed);
    sum_sq_us_.store(
        sum_sq_us_.load(std::memory_order_relaxed) + us * us,
        std::memory_order_relaxed);

    if (us > kBudgetUs) {
      over_budget_.fetch_add(1, std::memory_order_relaxed);
    }

    const int bucket = std::min(
        static_cast<int>(us / static_cast<double>(kBucketWidthUs)),
        kBuckets);
    histogram_[static_cast<std::size_t>(bucket)].fetch_add(
        1, std::memory_order_relaxed);
  }

  // ── Percentile computation from histogram ─────────────────────────────────────

  static void ComputePercentiles(Stats & s) noexcept
  {
    if (s.count == 0) {return;}
    const uint64_t p95_target = (s.count * 95) / 100;
    const uint64_t p99_target = (s.count * 99) / 100;

    uint64_t cumulative = 0;
    bool p95_done = false, p99_done = false;

    for (int b = 0; b <= kBuckets; ++b) {
      cumulative += s.histogram[static_cast<std::size_t>(b)];

      if (!p95_done && cumulative >= p95_target) {
        s.p95_us = (b < kBuckets) ?
          static_cast<double>(b * kBucketWidthUs) :
          static_cast<double>(kBuckets * kBucketWidthUs);
        p95_done = true;
      }
      if (!p99_done && cumulative >= p99_target) {
        s.p99_us = (b < kBuckets) ?
          static_cast<double>(b * kBucketWidthUs) :
          static_cast<double>(kBuckets * kBucketWidthUs);
        p99_done = true;
      }
      if (p95_done && p99_done) {break;}
    }
  }

  // ── Atomic helpers for double min/max (compare_exchange loop) ─────────────────

  static void AtomicMin(std::atomic<double> & a, double v) noexcept
  {
    double old = a.load(std::memory_order_relaxed);
    while (v < old &&
      !a.compare_exchange_weak(old, v, std::memory_order_relaxed))
    {
    }
  }

  static void AtomicMax(std::atomic<double> & a, double v) noexcept
  {
    double old = a.load(std::memory_order_relaxed);
    while (v > old &&
      !a.compare_exchange_weak(old, v, std::memory_order_relaxed))
    {
    }
  }

  // ── Atomic members ────────────────────────────────────────────────────────────
  std::atomic<uint64_t> count_{0};
  std::atomic<double> min_us_{1e9};
  std::atomic<double> max_us_{0.0};
  std::atomic<double> last_us_{0.0};
  std::atomic<double> sum_us_{0.0};
  std::atomic<double> sum_sq_us_{0.0};
  std::atomic<uint64_t> over_budget_{0};
  std::array<std::atomic<uint64_t>, kBuckets + 1> histogram_{};
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_CONTROLLER_TIMING_PROFILER_HPP_
