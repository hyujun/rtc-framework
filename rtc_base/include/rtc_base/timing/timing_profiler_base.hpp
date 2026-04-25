#ifndef RTC_BASE_TIMING_TIMING_PROFILER_BASE_HPP_
#define RTC_BASE_TIMING_TIMING_PROFILER_BASE_HPP_

// ── TimingProfilerBase
// ────────────────────────────────────────────────────────
//
// Lock-free, single-producer timing profiler with histogram-based percentiles.
//
// Provides the common core shared by ControllerTimingProfiler and
// HandTimingProfiler:
//   - Running count / sum / sum-of-squares / min / max / last
//   - Fixed-width histogram with linear-interpolated percentiles (p95, p99)
//   - Over-budget counter
//   - PhaseStats helper for per-phase (sum, min, max) tracking
//
// Template parameters let each subclass customize the histogram range:
//   Buckets      – number of histogram buckets (last bucket = overflow)
//   BucketWidthUs – width of each bucket in microseconds
//   BudgetUs     – over-budget threshold in microseconds
//
// Thread safety:
//   UpdateTotal() must be called from a single producer thread.
//   GetBaseStats() may be called from any thread (relaxed atomics).

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace rtc {

template <int Buckets = 20, int BucketWidthUs = 100, int BudgetUs = 2000>
class TimingProfilerBase {
public:
  static constexpr int kBuckets = Buckets;
  static constexpr int kBucketWidthUs = BucketWidthUs;
  static constexpr double kBudgetUs = static_cast<double>(BudgetUs);

  // ── Stats snapshot
  // ──────────────────────────────────────────────────────────

  struct BaseStats {
    uint64_t count{0};
    double min_us{0.0};
    double max_us{0.0};
    double mean_us{0.0};
    double stddev_us{0.0};
    double p95_us{0.0};
    double p99_us{0.0};
    double last_us{0.0};
    uint64_t over_budget{0};
    std::array<uint64_t, static_cast<std::size_t>(Buckets + 1)> histogram{};
  };

  // Per-phase mean/min/max (used by subclasses for phase-level tracking)
  struct PhaseStats {
    double mean_us{0.0};
    double min_us{0.0};
    double max_us{0.0};
  };

  // ── Update (single-producer) ──────────────────────────────────────────────
  //
  // Publication order matters for GetBaseStats() snapshot consistency:
  // all other fields are updated first, and count_ is incremented LAST with
  // release semantics. Paired with an acquire load of count_ in
  // GetBaseStats(), this guarantees sum(histogram) >= count for the
  // consumer, which in turn guarantees that p95_rank / p99_rank are always
  // reached during the histogram scan (no p99=0.0 blips from torn reads).

  void UpdateTotal(double us) noexcept {
    AtomicMin(min_us_, us);
    AtomicMax(max_us_, us);
    last_us_.store(us, std::memory_order_relaxed);

    // Single-producer RMW — relaxed load+store is safe
    sum_us_.store(sum_us_.load(std::memory_order_relaxed) + us,
                  std::memory_order_relaxed);
    sum_sq_us_.store(sum_sq_us_.load(std::memory_order_relaxed) + us * us,
                     std::memory_order_relaxed);

    if (us > kBudgetUs) {
      over_budget_.fetch_add(1, std::memory_order_relaxed);
    }

    const int bucket = std::min(
        static_cast<int>(us / static_cast<double>(kBucketWidthUs)), kBuckets);
    histogram_[static_cast<std::size_t>(bucket)].fetch_add(
        1, std::memory_order_relaxed);

    // count_ last, with release — acts as a publish barrier for all of the
    // above relaxed stores.
    count_.fetch_add(1, std::memory_order_release);
  }

  // ── Statistics snapshot ───────────────────────────────────────────────────

  [[nodiscard]] BaseStats GetBaseStats() const noexcept {
    BaseStats s;
    // Acquire load pairs with the release fetch_add in UpdateTotal(),
    // so every field observed below reflects at least the first s.count
    // producer ticks. Any later ticks may leave histogram_ slightly ahead
    // of s.count (sum(histogram) >= count) — harmless for percentile math.
    s.count = count_.load(std::memory_order_acquire);
    const double raw_min = min_us_.load(std::memory_order_relaxed);
    s.min_us = (raw_min >= 1e8) ? 0.0 : raw_min;
    s.max_us = max_us_.load(std::memory_order_relaxed);
    s.last_us = last_us_.load(std::memory_order_relaxed);
    s.over_budget = over_budget_.load(std::memory_order_relaxed);

    if (s.count > 0) {
      s.mean_us = sum_us_.load(std::memory_order_relaxed) /
                  static_cast<double>(s.count);
      const double var = sum_sq_us_.load(std::memory_order_relaxed) /
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

  [[nodiscard]] double LastUs() const noexcept {
    return last_us_.load(std::memory_order_relaxed);
  }

  void ResetBase() noexcept {
    count_.store(0, std::memory_order_relaxed);
    min_us_.store(1e9, std::memory_order_relaxed);
    max_us_.store(0.0, std::memory_order_relaxed);
    last_us_.store(0.0, std::memory_order_relaxed);
    sum_us_.store(0.0, std::memory_order_relaxed);
    sum_sq_us_.store(0.0, std::memory_order_relaxed);
    over_budget_.store(0, std::memory_order_relaxed);
    for (auto &b : histogram_)
      b.store(0, std::memory_order_relaxed);
  }

protected:
  // ── Phase-level helpers (for subclass use) ────────────────────────────────

  static void AtomicMin(std::atomic<double> &a, double v) noexcept {
    double old = a.load(std::memory_order_relaxed);
    while (v < old &&
           !a.compare_exchange_weak(old, v, std::memory_order_relaxed)) {
    }
  }

  static void AtomicMax(std::atomic<double> &a, double v) noexcept {
    double old = a.load(std::memory_order_relaxed);
    while (v > old &&
           !a.compare_exchange_weak(old, v, std::memory_order_relaxed)) {
    }
  }

  static void UpdatePhase(std::atomic<double> &sum,
                          std::atomic<double> &min_val,
                          std::atomic<double> &max_val, double us) noexcept {
    sum.store(sum.load(std::memory_order_relaxed) + us,
              std::memory_order_relaxed);
    AtomicMin(min_val, us);
    AtomicMax(max_val, us);
  }

  [[nodiscard]] static PhaseStats
  LoadPhaseStats(const std::atomic<double> &sum,
                 const std::atomic<double> &min_val,
                 const std::atomic<double> &max_val, uint64_t count) noexcept {
    PhaseStats ps;
    ps.mean_us =
        sum.load(std::memory_order_relaxed) / static_cast<double>(count);
    const double raw_min = min_val.load(std::memory_order_relaxed);
    ps.min_us = (raw_min >= 1e8) ? 0.0 : raw_min;
    ps.max_us = max_val.load(std::memory_order_relaxed);
    return ps;
  }

  static void ResetPhase(std::atomic<double> &sum, std::atomic<double> &min_val,
                         std::atomic<double> &max_val) noexcept {
    sum.store(0.0, std::memory_order_relaxed);
    min_val.store(1e9, std::memory_order_relaxed);
    max_val.store(0.0, std::memory_order_relaxed);
  }

private:
  // ── Percentile computation ────────────────────────────────────────────────

  static void ComputePercentiles(BaseStats &s) noexcept {
    if (s.count == 0)
      return;
    const double p95_rank = static_cast<double>(s.count) * 0.95;
    const double p99_rank = static_cast<double>(s.count) * 0.99;

    uint64_t cumulative = 0;
    bool p95_done = false, p99_done = false;

    for (int b = 0; b <= kBuckets; ++b) {
      const uint64_t prev_cum = cumulative;
      cumulative += s.histogram[static_cast<std::size_t>(b)];
      const uint64_t bucket_count = s.histogram[static_cast<std::size_t>(b)];
      if (!p95_done && cumulative >= static_cast<uint64_t>(p95_rank)) {
        s.p95_us =
            InterpolateBucket(b, prev_cum, bucket_count, p95_rank, s.max_us);
        p95_done = true;
      }
      if (!p99_done && cumulative >= static_cast<uint64_t>(p99_rank)) {
        s.p99_us =
            InterpolateBucket(b, prev_cum, bucket_count, p99_rank, s.max_us);
        p99_done = true;
      }
      if (p95_done && p99_done)
        break;
    }
  }

  // Linear interpolation inside a bucket. For the final (overflow) bucket
  // the bucket-width is unknown, so we stretch the interpolation between
  // the bucket's lower edge and the observed max. Without this, p99 would
  // silently clip to the overflow-bucket lower edge (e.g. 2000 µs for the
  // controller profiler) even when max_us is much larger.
  //
  // The result is clamped to max_us. By definition any percentile p ≤ max,
  // but linear interpolation assumes uniform fill within the bucket — when
  // samples cluster in the lower part of a bucket (e.g. all 27 µs samples
  // in the [0, 100) bucket), the unclamped estimate can exceed max.
  static double InterpolateBucket(int bucket_idx, uint64_t prev_cum,
                                  uint64_t bucket_count, double target_rank,
                                  double max_us) noexcept {
    const double bucket_lo = static_cast<double>(bucket_idx * kBucketWidthUs);
    if (bucket_count == 0)
      return std::min(bucket_lo, max_us);
    const double frac = (target_rank - static_cast<double>(prev_cum)) /
                        static_cast<double>(bucket_count);
    const double bucket_span = (bucket_idx >= kBuckets)
                                   ? std::max(0.0, max_us - bucket_lo)
                                   : static_cast<double>(kBucketWidthUs);
    return std::min(bucket_lo + frac * bucket_span, max_us);
  }

  // ── Atomic members ────────────────────────────────────────────────────────
  std::atomic<uint64_t> count_{0};
  std::atomic<double> min_us_{1e9};
  std::atomic<double> max_us_{0.0};
  std::atomic<double> last_us_{0.0};
  std::atomic<double> sum_us_{0.0};
  std::atomic<double> sum_sq_us_{0.0};
  std::atomic<uint64_t> over_budget_{0};
  std::array<std::atomic<uint64_t>, static_cast<std::size_t>(kBuckets + 1)>
      histogram_{};
};

} // namespace rtc

#endif // RTC_BASE_TIMING_TIMING_PROFILER_BASE_HPP_
