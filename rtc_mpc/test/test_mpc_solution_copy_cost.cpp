/// @file test_mpc_solution_copy_cost.cpp
/// @brief Cost of the `MPCSolution` copies that sit on RT paths (publish
///        memcpy on the MPC thread, the interpolator's copy on the control
///        tick). Reports cold-cache and warm-cache copy times.
///
/// The timing assertion is a pre-registered gate but only armed when
/// `RTC_RUN_TIMING_GATES=1` is set (Release build, pinned core — wall-clock
/// numbers on a shared developer box are otherwise too noisy to fail a
/// build on). Without the variable the test prints the numbers and passes.
///
/// Method: for each of several runs, repeat the copy many times, taking the
/// p99 of the per-copy durations; the reported value is the median of the
/// per-run p99s. "Cold" evicts the source and destination from cache
/// between copies by streaming through a large scratch buffer.

#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

namespace rtc::mpc {
namespace {

// Armed (RTC_RUN_TIMING_GATES=1): the pre-registered measurement. Unarmed:
// a short smoke pass so an ordinary test run stays cheap.
constexpr int kRunsArmed = 5;
constexpr int kCopiesPerRunArmed = 200;
constexpr int kRunsSmoke = 1;
constexpr int kCopiesPerRunSmoke = 3;
constexpr std::size_t kEvictBytes = 32u * 1024u * 1024u;
constexpr double kColdP99LimitUs = 200.0;  // pre-registered: a tenth of a 500 Hz tick

bool TimingGatesArmed() {
  const char* v = std::getenv("RTC_RUN_TIMING_GATES");
  return v != nullptr && std::strcmp(v, "1") == 0;
}

template <typename T>
struct CopyBench {
  std::unique_ptr<T> src = std::make_unique<T>();
  std::unique_ptr<T> dst = std::make_unique<T>();
  std::vector<std::byte> evict = std::vector<std::byte>(kEvictBytes);
  volatile std::byte sink{};

  void Evict() {
    for (std::size_t i = 0; i < evict.size(); i += 64) {
      evict[i] = static_cast<std::byte>(i);
    }
    std::byte acc{};
    for (std::size_t i = 0; i < evict.size(); i += 4096) {
      acc ^= evict[i];
    }
    sink = acc;
  }

  double OneCopyUs() {
    const auto t0 = std::chrono::steady_clock::now();
    std::memcpy(dst.get(), src.get(), sizeof(T));
    const auto t1 = std::chrono::steady_clock::now();
    sink = *reinterpret_cast<const std::byte*>(dst.get());
    return std::chrono::duration<double, std::micro>(t1 - t0).count();
  }

  /// Median over runs of the per-run p99 of a single copy.
  double MedianOfRunP99Us(bool cold, int runs, int copies_per_run) {
    std::vector<double> p99(static_cast<std::size_t>(runs));
    std::vector<double> samples(static_cast<std::size_t>(copies_per_run));
    for (int r = 0; r < runs; ++r) {
      for (int i = 0; i < copies_per_run; ++i) {
        if (cold) {
          Evict();
        }
        samples[static_cast<std::size_t>(i)] = OneCopyUs();
      }
      std::sort(samples.begin(), samples.end());
      const std::size_t idx =
          static_cast<std::size_t>(0.99 * static_cast<double>(copies_per_run - 1) + 0.5);
      p99[static_cast<std::size_t>(r)] = samples[std::min(idx, samples.size() - 1)];
    }
    std::sort(p99.begin(), p99.end());
    return p99[static_cast<std::size_t>(runs / 2)];
  }
};

TEST(MpcSolutionCopyCost, ColdAndWarmCopy) {
  const bool armed = TimingGatesArmed();
  const int runs = armed ? kRunsArmed : kRunsSmoke;
  const int copies = armed ? kCopiesPerRunArmed : kCopiesPerRunSmoke;
  CopyBench<MPCSolution> bench;

  const double cold_us = bench.MedianOfRunP99Us(true, runs, copies);
  const double warm_us = bench.MedianOfRunP99Us(false, runs, copies);

  std::cout << "[copy-cost] sizeof(MPCSolution) = " << sizeof(MPCSolution) << " B\n"
            << "[copy-cost] cold p99 (median of runs) = " << cold_us << " us, warm = " << warm_us
            << " us\n"
            << "[copy-cost] runs = " << runs << ", copies per run = " << copies << ", timing gate "
            << (armed ? "ARMED" : "not armed") << " (RTC_RUN_TIMING_GATES=1 arms it)\n";

  EXPECT_GT(cold_us, 0.0);
  EXPECT_GT(warm_us, 0.0);
  if (armed) {
    EXPECT_LE(cold_us, kColdP99LimitUs) << "cold copy p99 exceeded the pre-registered limit";
  }
}

}  // namespace
}  // namespace rtc::mpc
