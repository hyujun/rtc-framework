#pragma once

/// @file alloc_counter.hpp
/// @brief Test-only malloc/free counter backing the Phase 6 Step 5 alloc
///        tracer. Provides an arm/disarm window and atomic counters; the
///        operator new/delete overrides live in each consuming TU.
///
/// Usage pattern:
///   1. Warm up any static / thread-local state that would allocate on first
///      use (SSO threshold strings, chrono::steady_clock::now(), thread-id,
///      Aligator solver workspace via one warm solve).
///   2. `AllocCounter::Arm()` — counters now increment.
///   3. Drive the `handler_->Solve(...)` tight loop (do not use the threaded
///      loop; jthread and steady_clock introduce confounding allocs).
///   4. `AllocCounter::Disarm()` — counters frozen.
///   5. Assert `Count() == 0`.
///
/// Thread-safety — counters are `std::atomic`; the overrides use
/// `memory_order_relaxed`. The arm/disarm flag is also atomic so a single
/// test thread can drive it without locking.

#include <atomic>
#include <cstdint>

namespace rtc::mpc::test_utils {

struct AllocCounter {
  inline static std::atomic<std::int64_t> alloc_count{0};
  inline static std::atomic<std::int64_t> free_count{0};
  inline static std::atomic<bool> armed{false};

  static void Arm() noexcept {
    alloc_count.store(0, std::memory_order_relaxed);
    free_count.store(0, std::memory_order_relaxed);
    armed.store(true, std::memory_order_release);
  }

  static void Disarm() noexcept {
    armed.store(false, std::memory_order_release);
  }

  [[nodiscard]] static bool IsArmed() noexcept {
    return armed.load(std::memory_order_acquire);
  }

  [[nodiscard]] static std::int64_t AllocCount() noexcept {
    return alloc_count.load(std::memory_order_relaxed);
  }

  [[nodiscard]] static std::int64_t FreeCount() noexcept {
    return free_count.load(std::memory_order_relaxed);
  }

  /// Called from operator new/delete overrides. Increment only while armed;
  /// unarmed calls (warm-up / tear-down) are ignored so the fixture code
  /// itself doesn't trip the assertion.
  static void RecordAlloc() noexcept {
    if (armed.load(std::memory_order_acquire)) {
      alloc_count.fetch_add(1, std::memory_order_relaxed);
    }
  }

  static void RecordFree() noexcept {
    if (armed.load(std::memory_order_acquire)) {
      free_count.fetch_add(1, std::memory_order_relaxed);
    }
  }
};

} // namespace rtc::mpc::test_utils
