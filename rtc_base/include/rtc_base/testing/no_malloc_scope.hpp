// ── Zero-allocation gate for RT hot paths (test-only) ───────────────────────
// An RT tick must not touch the heap (RT-1). This RAII scope turns any heap
// allocation performed by Eigen on the current thread into a recorded
// violation, so a test can assert that a controller's Compute()/update() path
// allocates nothing — spec §11.7 T7.1.
//
// FAIL-CLOSED contract — read before use:
//
//  1. Include this header BEFORE any Eigen header. It defines
//     EIGEN_RUNTIME_NO_MALLOC and installs a custom `eigen_assert`; if Eigen is
//     already included both are ignored and the gate would silently pass. A
//     `#error` below enforces the ordering.
//
//  2. Detection does NOT ride Eigen's default `eigen_assert` (= `assert`, a
//     no-op under -DNDEBUG — this repo builds Release, so the stock guard would
//     be fail-open). We install our own `eigen_assert` that records violations
//     while a scope is active, so the gate is effective in Release too. Outside
//     a ScopedNoMalloc region a failing eigen_assert still aborts, preserving
//     normal assertion semantics.
//
//  3. `set_is_malloc_allowed` only instruments Eigen allocations compiled in
//     THIS translation unit. Code under test whose Eigen ops live in another TU
//     (a separately-compiled .cpp / .so) is NOT observed. Drive the compute
//     path through code that is header-inline / instantiated in the test TU, or
//     compile that TU with EIGEN_RUNTIME_NO_MALLOC. This util targets Eigen
//     allocation specifically; non-Eigen heap use needs malloc interposition.
#pragma once

#if defined(EIGEN_CORE_H)
#error "include rtc_base/testing/no_malloc_scope.hpp before any Eigen header"
#endif

#ifndef EIGEN_RUNTIME_NO_MALLOC
#define EIGEN_RUNTIME_NO_MALLOC
#endif

#include <cstdint>
#include <cstdlib>

namespace rtc::testing::detail {

// Nesting depth of active ScopedNoMalloc regions on the current thread. The
// `eigen_assert` tripwire consults this to tell a malloc-gate trip (depth > 0)
// from a genuine Eigen assertion (depth == 0, must still abort). `inline` gives
// one symbol across TUs; `thread_local` isolates background threads.
[[nodiscard]] inline int& NoMallocDepth() noexcept {
  static thread_local int depth = 0;
  return depth;
}

[[nodiscard]] inline std::uint64_t& MallocViolations() noexcept {
  static thread_local std::uint64_t count = 0;
  return count;
}

}  // namespace rtc::testing::detail

// NDEBUG-independent tripwire, defined before <Eigen/Core> so every Eigen inline
// in this TU sees it. Eigen calls eigen_assert(is_malloc_allowed()) from
// check_that_malloc_is_allowed(); inside a ScopedNoMalloc region the condition
// is false and we record the violation (without aborting — the allocation still
// completes, so there is no UB and the test reports cleanly). Referencing only
// our own symbols keeps the macro valid at every Eigen expansion site (Eigen's
// own is_malloc_allowed is not yet declared at the earliest ones). A failing
// assert outside any region is a genuine Eigen assertion and still aborts.
#define eigen_assert(cond)                             \
  do {                                                 \
    if (!(cond)) {                                     \
      if (::rtc::testing::detail::NoMallocDepth() > 0) \
        ++::rtc::testing::detail::MallocViolations();  \
      else                                             \
        ::std::abort();                                \
    }                                                  \
  } while (0)

#include <Eigen/Core>

namespace rtc::testing {

// RAII: forbid Eigen heap allocation on the current thread for the scope's
// lifetime. Construction resets the violation counter and disallows malloc;
// destruction re-allows it (at the outermost scope). Query violations() after
// (or during) the scope.
class ScopedNoMalloc {
 public:
  ScopedNoMalloc() noexcept {
    detail::MallocViolations() = 0;
    ++detail::NoMallocDepth();
    Eigen::internal::set_is_malloc_allowed(false);
  }

  ~ScopedNoMalloc() {
    if (--detail::NoMallocDepth() == 0)
      Eigen::internal::set_is_malloc_allowed(true);
  }

  ScopedNoMalloc(const ScopedNoMalloc&) = delete;
  ScopedNoMalloc& operator=(const ScopedNoMalloc&) = delete;
  ScopedNoMalloc(ScopedNoMalloc&&) = delete;
  ScopedNoMalloc& operator=(ScopedNoMalloc&&) = delete;

  // Number of Eigen heap allocations observed since this scope began.
  [[nodiscard]] std::uint64_t violations() const noexcept { return detail::MallocViolations(); }
};

}  // namespace rtc::testing
