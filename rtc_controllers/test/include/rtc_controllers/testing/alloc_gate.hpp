// ── Global heap-allocation gate for RT-path golden-vector suites (test-only) ──
// An RT tick must not touch the heap (RT-1). This gate replaces the global
// `operator new` family with malloc-backed versions that COUNT allocations while
// a ScopedAllocGate is alive, so a test can assert that a core's compute path
// allocated nothing.
//
// Why this and not rtc_base/testing/no_malloc_scope.hpp: that header is an
// EIGEN allocation tripwire (it rides `eigen_assert` +
// EIGEN_RUNTIME_NO_MALLOC), so it sees Eigen's allocator and nothing else. This
// one sees `operator new` — including one from a header-inline helper or a
// std::vector — and NOT Eigen, because Eigen's `internal::aligned_malloc` calls
// `std::malloc` / `handmade_aligned_malloc` directly and never routes through
// `operator new`. The two are COMPLEMENTARY, not substitutes: an Eigen-only core
// needs both (a regression that turns a fixed-size temporary into a
// runtime-sized one is invisible to this gate), while an Eigen-free core needs
// only this one.
//
// CONTRACT — read before use:
//
//  1. Include this header in EXACTLY ONE translation unit per test binary. It
//     DEFINES replacement global operators, which the standard forbids from
//     being `inline`; a second TU in the same binary is a multiple-definition
//     LINK error (loud, not silent). One `ament_add_gtest` target per test .cpp
//     is what makes this hold in this package.
//
//  2. The target needs `-Wno-mismatched-new-delete`. Pairing `new`→`malloc` with
//     `delete`→`free` makes GCC false-positive on every delete site
//     program-wide.
//
//  3. Arm the gate with a ScopedAllocGate object, never with a bare flag: an
//     ASSERT_* inside the measured region returns from the test, and a bare
//     disarm line would then never run — leaving counting on for every
//     subsequent test in the binary, where nothing reads it.
#pragma once

#include <cstddef>
#include <cstdlib>
#include <new>

namespace rtc::testing {

namespace detail {

// Nesting depth of active ScopedAllocGate regions on the current thread; the
// replacement `operator new` counts only while this is > 0. `inline` gives one
// symbol across TUs, `thread_local` isolates background threads, and both are
// trivially destructible so first touch cannot itself allocate (a thread_local
// with a non-trivial destructor would register with __cxa_thread_atexit).
[[nodiscard]] inline int& AllocGateDepth() noexcept {
  static thread_local int depth = 0;
  return depth;
}

[[nodiscard]] inline std::size_t& AllocGateCount() noexcept {
  static thread_local std::size_t count = 0;
  return count;
}

}  // namespace detail

/// @brief RAII: count global heap allocations on the current thread for the
///        scope's lifetime.
///
/// Construction zeroes the counter and arms the gate; destruction disarms it,
/// so an early return out of the measured region cannot leak a live gate into
/// the rest of the binary (contract item 3 above).
///
/// @note Nesting is depth-counted, but an inner scope re-zeroes the shared
///       counter — read count() before opening one.
class ScopedAllocGate {
 public:
  ScopedAllocGate() noexcept {
    detail::AllocGateCount() = 0;
    ++detail::AllocGateDepth();
  }

  ~ScopedAllocGate() { --detail::AllocGateDepth(); }

  ScopedAllocGate(const ScopedAllocGate&) = delete;
  ScopedAllocGate& operator=(const ScopedAllocGate&) = delete;
  ScopedAllocGate(ScopedAllocGate&&) = delete;
  ScopedAllocGate& operator=(ScopedAllocGate&&) = delete;

  /// @brief Global heap allocations observed since this scope began.
  /// @return allocation count on the current thread.
  [[nodiscard]] std::size_t count() const noexcept { return detail::AllocGateCount(); }
};

}  // namespace rtc::testing

// ── Replacement global operators (contract item 1: one TU per binary) ────────

void* operator new(std::size_t n) {
  if (::rtc::testing::detail::AllocGateDepth() > 0)
    ++::rtc::testing::detail::AllocGateCount();
  void* p = std::malloc(n != 0 ? n : 1);
  if (p == nullptr)
    throw std::bad_alloc();
  return p;
}

void* operator new[](std::size_t n) {
  return ::operator new(n);
}

void operator delete(void* p) noexcept {
  std::free(p);
}

void operator delete[](void* p) noexcept {
  std::free(p);
}

void operator delete(void* p, std::size_t) noexcept {
  std::free(p);
}

void operator delete[](void* p, std::size_t) noexcept {
  std::free(p);
}
