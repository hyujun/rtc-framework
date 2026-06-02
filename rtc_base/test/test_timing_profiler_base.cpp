// Unit tests for rtc::TimingProfilerBase (timing_profiler_base.hpp):
//   - Running count / min / max / mean / stddev / last / over-budget.
//   - Histogram bucketing (including the overflow bucket).
//   - Histogram-based percentile interpolation with clamp-to-max, and the
//     overflow-bucket "stretch to max" path that prevents p99 clipping to the
//     overflow lower edge.
//   - ResetBase.
//
// Single-producer in-process math; sandbox-safe (no RT permissions needed).

#include "rtc_base/timing/timing_profiler_base.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

namespace {

// Buckets = 10, width = 100 us, budget = 500 us → buckets [0,100) .. [900,1000)
// with index 10 acting as the overflow bucket (>= 1000 us).
using Profiler = rtc::TimingProfilerBase<10, 100, 500>;

TEST(TimingProfilerBaseTest, FreshStatsAreZero) {
  Profiler p;
  const auto s = p.GetBaseStats();
  EXPECT_EQ(s.count, 0U);
  EXPECT_DOUBLE_EQ(s.min_us, 0.0);  // sentinel 1e9 mapped to 0 when empty
  EXPECT_DOUBLE_EQ(s.max_us, 0.0);
  EXPECT_DOUBLE_EQ(s.p95_us, 0.0);
  EXPECT_DOUBLE_EQ(s.p99_us, 0.0);
}

TEST(TimingProfilerBaseTest, RunningStatsAndHistogram) {
  Profiler p;
  for (double v : {50.0, 150.0, 250.0, 50.0, 350.0}) {
    p.UpdateTotal(v);
  }
  const auto s = p.GetBaseStats();
  EXPECT_EQ(s.count, 5U);
  EXPECT_DOUBLE_EQ(s.min_us, 50.0);
  EXPECT_DOUBLE_EQ(s.max_us, 350.0);
  EXPECT_DOUBLE_EQ(s.last_us, 350.0);
  EXPECT_DOUBLE_EQ(s.mean_us, 170.0);  // 850 / 5
  EXPECT_EQ(s.over_budget, 0U);        // none exceed the 500 us budget
  EXPECT_EQ(s.histogram[0U], 2U);      // 50, 50 → bucket 0
  EXPECT_EQ(s.histogram[1U], 1U);      // 150 → bucket 1
  EXPECT_EQ(s.histogram[2U], 1U);      // 250 → bucket 2
  EXPECT_EQ(s.histogram[3U], 1U);      // 350 → bucket 3
  // variance = 212500/5 - 170^2 = 13600
  EXPECT_NEAR(s.stddev_us, std::sqrt(13600.0), 1e-9);
}

TEST(TimingProfilerBaseTest, OverBudgetCounted) {
  Profiler p;
  p.UpdateTotal(600.0);  // > 500 us budget
  p.UpdateTotal(100.0);
  const auto s = p.GetBaseStats();
  EXPECT_EQ(s.over_budget, 1U);
  EXPECT_EQ(s.histogram[6U], 1U);  // 600 / 100 = bucket 6
}

TEST(TimingProfilerBaseTest, PercentilesClampToMaxOnUniformSamples) {
  Profiler p;
  for (std::size_t k = 0; k < 100U; ++k) {
    p.UpdateTotal(50.0);  // all land in bucket 0
  }
  const auto s = p.GetBaseStats();
  EXPECT_EQ(s.count, 100U);
  // Uniform-fill extrapolation would place p95 at 95 us within the [0,100)
  // bucket, but the estimate is clamped to the observed max (50 us).
  EXPECT_DOUBLE_EQ(s.p95_us, 50.0);
  EXPECT_DOUBLE_EQ(s.p99_us, 50.0);
}

TEST(TimingProfilerBaseTest, OverflowBucketPercentileStretchesToMax) {
  Profiler p;
  for (std::size_t k = 0; k < 100U; ++k) {
    p.UpdateTotal(1500.0);  // overflow bucket (>= 1000 us)
  }
  const auto s = p.GetBaseStats();
  EXPECT_EQ(s.histogram[10U], 100U);  // overflow bucket index == kBuckets
  EXPECT_DOUBLE_EQ(s.max_us, 1500.0);
  // p95/p99 interpolate between the overflow-bucket lower edge (1000) and the
  // observed max (1500) and must never exceed max.
  EXPECT_GT(s.p95_us, 1000.0);
  EXPECT_LE(s.p95_us, 1500.0);
  EXPECT_GE(s.p99_us, s.p95_us);
  EXPECT_LE(s.p99_us, 1500.0);
}

TEST(TimingProfilerBaseTest, ResetClearsAll) {
  Profiler p;
  for (std::size_t k = 0; k < 10U; ++k) {
    p.UpdateTotal(123.0);
  }
  p.ResetBase();
  const auto s = p.GetBaseStats();
  EXPECT_EQ(s.count, 0U);
  EXPECT_DOUBLE_EQ(s.max_us, 0.0);
  EXPECT_EQ(s.over_budget, 0U);
}

}  // namespace
