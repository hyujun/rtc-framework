// ── test_controller_timing_profiler.cpp
// ─────────────────────────────────────── Unit tests for
// rtc::ControllerTimingProfiler.
//
// Covers: initial state, MeasuredCompute timing, statistics (min/max/mean,
// percentiles), over-budget detection, Reset, Summary string format, and
// histogram bucket assignment.
//
// Uses a stub controller with configurable busy-wait to produce deterministic
// timing measurements.
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_controller_interface/rt_controller_interface.hpp>
#include <rtc_controller_manager/controller_timing_profiler.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <string>

namespace {

// Stub controller with configurable Compute() duration via busy-wait.
class ProfilerStubController : public rtc::RTControllerInterface {
public:
  ProfilerStubController() = default;

  [[nodiscard]] rtc::ControllerOutput
  Compute(const rtc::ControllerState &) noexcept override {
    if (busy_wait_us_ > 0.0) {
      const auto t0 = std::chrono::steady_clock::now();
      while (std::chrono::duration<double, std::micro>(
                 std::chrono::steady_clock::now() - t0)
                 .count() < busy_wait_us_) {
      }
    }
    rtc::ControllerOutput out{};
    out.valid = true;
    return out;
  }

  void SetDeviceTarget(int, std::span<const double>) noexcept override {}
  [[nodiscard]] std::string_view Name() const noexcept override {
    return "ProfilerStub";
  }
  void InitializeHoldPosition(const rtc::ControllerState &) noexcept override {}

  void set_busy_wait_us(double us) noexcept { busy_wait_us_ = us; }

private:
  double busy_wait_us_{0.0};
};

// ═══════════════════════════════════════════════════════════════════════════════
// Initial state
// ═══════════════════════════════════════════════════════════════════════════════

TEST(ControllerTimingProfilerTest, InitialStateNoData) {
  rtc::ControllerTimingProfiler profiler;
  const auto stats = profiler.GetStats();

  EXPECT_EQ(stats.count, uint64_t{0});
  EXPECT_DOUBLE_EQ(stats.mean_us, 0.0);
  EXPECT_EQ(stats.over_budget, uint64_t{0});
}

TEST(ControllerTimingProfilerTest, InitialLastComputeUs) {
  rtc::ControllerTimingProfiler profiler;
  EXPECT_DOUBLE_EQ(profiler.LastComputeUs(), 0.0);
}

// ═══════════════════════════════════════════════════════════════════════════════
// MeasuredCompute records timing
// ═══════════════════════════════════════════════════════════════════════════════

TEST(ControllerTimingProfilerTest, MeasuredComputeRecordsTiming) {
  rtc::ControllerTimingProfiler profiler;
  ProfilerStubController ctrl;
  rtc::ControllerState state{};
  state.dt = 0.002;

  auto output = profiler.MeasuredCompute(ctrl, state);

  EXPECT_TRUE(output.valid);
  EXPECT_EQ(profiler.GetStats().count, uint64_t{1});
  EXPECT_GT(profiler.LastComputeUs(), 0.0);
}

TEST(ControllerTimingProfilerTest, MultipleMeasurements) {
  rtc::ControllerTimingProfiler profiler;
  ProfilerStubController ctrl;
  rtc::ControllerState state{};

  constexpr int kN = 100;
  for (int i = 0; i < kN; ++i) {
    (void)profiler.MeasuredCompute(ctrl, state);
  }

  const auto stats = profiler.GetStats();
  EXPECT_EQ(stats.count, uint64_t{kN});
  EXPECT_GT(stats.mean_us, 0.0);
  EXPECT_GE(stats.max_us, stats.min_us);
  // Near-instant compute should not exceed budget (2000 µs)
  EXPECT_EQ(stats.over_budget, uint64_t{0});
}

// ═══════════════════════════════════════════════════════════════════════════════
// Statistics correctness
// ═══════════════════════════════════════════════════════════════════════════════

TEST(ControllerTimingProfilerTest, StatsMinMaxWithSlowCompute) {
  rtc::ControllerTimingProfiler profiler;
  ProfilerStubController ctrl;
  rtc::ControllerState state{};

  // Fast computes
  ctrl.set_busy_wait_us(0.0);
  for (int i = 0; i < 10; ++i) {
    (void)profiler.MeasuredCompute(ctrl, state);
  }

  // One slower compute (~500 µs)
  ctrl.set_busy_wait_us(500.0);
  (void)profiler.MeasuredCompute(ctrl, state);

  const auto stats = profiler.GetStats();
  EXPECT_EQ(stats.count, uint64_t{11});
  // max should capture the slow compute (allow tolerance for scheduling)
  EXPECT_GE(stats.max_us, 400.0);
}

TEST(ControllerTimingProfilerTest, PercentilesComputed) {
  rtc::ControllerTimingProfiler profiler;
  ProfilerStubController ctrl;
  rtc::ControllerState state{};

  for (int i = 0; i < 200; ++i) {
    (void)profiler.MeasuredCompute(ctrl, state);
  }

  const auto stats = profiler.GetStats();
  EXPECT_GE(stats.p95_us, 0.0);
  EXPECT_GE(stats.p99_us, 0.0);
  EXPECT_GE(stats.p99_us, stats.p95_us);
}

TEST(ControllerTimingProfilerTest, StddevComputed) {
  rtc::ControllerTimingProfiler profiler;
  ProfilerStubController ctrl;
  rtc::ControllerState state{};

  for (int i = 0; i < 50; ++i) {
    (void)profiler.MeasuredCompute(ctrl, state);
  }

  const auto stats = profiler.GetStats();
  // stddev should be non-negative
  EXPECT_GE(stats.stddev_us, 0.0);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Over-budget detection
// ═══════════════════════════════════════════════════════════════════════════════

TEST(ControllerTimingProfilerTest, OverBudgetDetection) {
  rtc::ControllerTimingProfiler profiler;
  ProfilerStubController ctrl;
  rtc::ControllerState state{};

  // Budget is 2000 µs — busy-wait 3000 µs to exceed it
  ctrl.set_busy_wait_us(3000.0);
  (void)profiler.MeasuredCompute(ctrl, state);

  const auto stats = profiler.GetStats();
  EXPECT_EQ(stats.count, uint64_t{1});
  EXPECT_EQ(stats.over_budget, uint64_t{1});
  EXPECT_GE(stats.max_us, 2000.0);
}

TEST(ControllerTimingProfilerTest, UnderBudgetNotCounted) {
  rtc::ControllerTimingProfiler profiler;
  ProfilerStubController ctrl;
  rtc::ControllerState state{};

  // 100 µs busy-wait — well within 2000 µs budget
  ctrl.set_busy_wait_us(100.0);
  for (int i = 0; i < 10; ++i) {
    (void)profiler.MeasuredCompute(ctrl, state);
  }

  EXPECT_EQ(profiler.GetStats().over_budget, uint64_t{0});
}

// ═══════════════════════════════════════════════════════════════════════════════
// Reset
// ═══════════════════════════════════════════════════════════════════════════════

TEST(ControllerTimingProfilerTest, ResetClearsAllStats) {
  rtc::ControllerTimingProfiler profiler;
  ProfilerStubController ctrl;
  rtc::ControllerState state{};

  for (int i = 0; i < 50; ++i) {
    (void)profiler.MeasuredCompute(ctrl, state);
  }
  EXPECT_EQ(profiler.GetStats().count, uint64_t{50});

  profiler.Reset();

  const auto stats = profiler.GetStats();
  EXPECT_EQ(stats.count, uint64_t{0});
  EXPECT_DOUBLE_EQ(stats.mean_us, 0.0);
  EXPECT_DOUBLE_EQ(stats.max_us, 0.0);
  EXPECT_EQ(stats.over_budget, uint64_t{0});
  EXPECT_DOUBLE_EQ(profiler.LastComputeUs(), 0.0);
}

TEST(ControllerTimingProfilerTest, ResetHistogramCleared) {
  rtc::ControllerTimingProfiler profiler;
  ProfilerStubController ctrl;
  rtc::ControllerState state{};

  for (int i = 0; i < 20; ++i) {
    (void)profiler.MeasuredCompute(ctrl, state);
  }
  profiler.Reset();

  const auto stats = profiler.GetStats();
  uint64_t total_histogram = 0;
  for (const auto &bucket : stats.histogram) {
    total_histogram += bucket;
  }
  EXPECT_EQ(total_histogram, uint64_t{0});
}

// ═══════════════════════════════════════════════════════════════════════════════
// Summary string
// ═══════════════════════════════════════════════════════════════════════════════

TEST(ControllerTimingProfilerTest, SummaryNoData) {
  rtc::ControllerTimingProfiler profiler;
  const auto summary = profiler.Summary("TestCtrl");
  EXPECT_EQ(summary, "TestCtrl timing: no data");
}

TEST(ControllerTimingProfilerTest, SummaryWithData) {
  rtc::ControllerTimingProfiler profiler;
  ProfilerStubController ctrl;
  rtc::ControllerState state{};

  (void)profiler.MeasuredCompute(ctrl, state);

  const auto summary = profiler.Summary("TestCtrl");
  EXPECT_NE(summary.find("TestCtrl"), std::string::npos);
  EXPECT_NE(summary.find("count=1"), std::string::npos);
  EXPECT_NE(summary.find("mean="), std::string::npos);
  EXPECT_NE(summary.find("p95="), std::string::npos);
  EXPECT_NE(summary.find("p99="), std::string::npos);
  EXPECT_NE(summary.find("over_budget="), std::string::npos);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Histogram bucket assignment
// ═══════════════════════════════════════════════════════════════════════════════

TEST(ControllerTimingProfilerTest, HistogramBucketZero) {
  rtc::ControllerTimingProfiler profiler;
  ProfilerStubController ctrl;
  rtc::ControllerState state{};

  // Near-instant compute should land in bucket 0 (0–100 µs)
  ctrl.set_busy_wait_us(0.0);
  (void)profiler.MeasuredCompute(ctrl, state);

  const auto stats = profiler.GetStats();
  EXPECT_GT(stats.histogram[0], uint64_t{0});
}

TEST(ControllerTimingProfilerTest, HistogramTotalMatchesCount) {
  rtc::ControllerTimingProfiler profiler;
  ProfilerStubController ctrl;
  rtc::ControllerState state{};

  constexpr int kN = 30;
  for (int i = 0; i < kN; ++i) {
    (void)profiler.MeasuredCompute(ctrl, state);
  }

  const auto stats = profiler.GetStats();
  uint64_t total = 0;
  for (const auto &bucket : stats.histogram) {
    total += bucket;
  }
  EXPECT_EQ(total, stats.count);
}

// Regression: p99 must NOT clip to the overflow-bucket lower edge when
// samples exceed the histogram range. Before the fix, p99 was capped at
// kBuckets * kBucketWidthUs (2000 µs for ControllerTimingProfiler) even if
// max_us was much larger.
TEST(ControllerTimingProfilerTest, OverflowBucketP99ExceedsBucketEdge) {
  rtc::ControllerTimingProfiler profiler;
  ProfilerStubController ctrl;
  rtc::ControllerState state{};

  // 40 near-instant samples → bucket 0
  ctrl.set_busy_wait_us(0.0);
  for (int i = 0; i < 40; ++i) {
    (void)profiler.MeasuredCompute(ctrl, state);
  }

  // 10 samples of ~3000 µs → overflow bucket (bucket index == kBuckets).
  // p99 rank = 49.5, cumulative[0] = 40, so the rank lands in overflow.
  ctrl.set_busy_wait_us(3000.0);
  for (int i = 0; i < 10; ++i) {
    (void)profiler.MeasuredCompute(ctrl, state);
  }

  const auto stats = profiler.GetStats();
  constexpr double kOverflowLo =
      static_cast<double>(rtc::ControllerTimingProfiler::kBuckets) *
      static_cast<double>(rtc::ControllerTimingProfiler::kBucketWidthUs);
  // Must exceed the overflow lower edge (not clipped) …
  EXPECT_GT(stats.p99_us, kOverflowLo);
  // … and must never exceed the observed max.
  EXPECT_LE(stats.p99_us, stats.max_us);
}

} // namespace
