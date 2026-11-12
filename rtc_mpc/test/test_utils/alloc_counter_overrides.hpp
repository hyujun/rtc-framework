#pragma once

/// @file alloc_counter_overrides.hpp
/// @brief The global operator new / delete overrides that feed AllocCounter.
///        They define non-inline functions: include this in EXACTLY ONE
///        translation unit per test binary (it may share that TU with
///        rtc_base/testing/malloc_gate.hpp).

#include "test_utils/alloc_counter.hpp"

#include <cstdlib>
#include <new>

// ── Global operator new / delete overrides ────────────────────────────────
// These live in this TU only; gtest links as a dynamic library (libgtest) so
// its allocations happen through its own translation-unit-resolved calls and
// do not route through these overrides. Even if they did, the `armed` flag
// gates counter updates — overrides are a no-op cost on the unarmed path.
//
// GCC 13+ -Wmismatched-new-delete fires on the `std::free(p)` calls below:
// it traces a `new TestClass` expression through inlined std/gtest paths into
// our delete overrides and flags `new ... -> free` as a mismatch. The pairing
// is correct (our `operator new` itself calls malloc), but the diagnostic
// doesn't model that. Suppress locally — global allocator overrides are the
// canonical case where this check has no signal.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif

void* operator new(std::size_t sz) {
  void* p = std::malloc(sz);
  if (p == nullptr) {
    throw std::bad_alloc{};
  }
  rtc::mpc::test_utils::AllocCounter::RecordAlloc();
  return p;
}

void* operator new[](std::size_t sz) {
  void* p = std::malloc(sz);
  if (p == nullptr) {
    throw std::bad_alloc{};
  }
  rtc::mpc::test_utils::AllocCounter::RecordAlloc();
  return p;
}

void operator delete(void* p) noexcept {
  rtc::mpc::test_utils::AllocCounter::RecordFree();
  std::free(p);
}

void operator delete[](void* p) noexcept {
  rtc::mpc::test_utils::AllocCounter::RecordFree();
  std::free(p);
}

void operator delete(void* p, std::size_t) noexcept {
  rtc::mpc::test_utils::AllocCounter::RecordFree();
  std::free(p);
}

void operator delete[](void* p, std::size_t) noexcept {
  rtc::mpc::test_utils::AllocCounter::RecordFree();
  std::free(p);
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
