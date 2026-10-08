// ── C-level heap-allocation gate (test-only) ──────────────────────────────────
// A global `operator new` counter sees only C++ allocations;
// no_malloc_scope.hpp trips on Eigen's allocator, but ONLY for Eigen code
// inlined into the TU that defines the scope. Neither sees a C `malloc` made
// inside a library that was compiled elsewhere — a shared object with explicit
// template instantiations (pinocchio), a QP solver wrapped in another package,
// or the package's own static library TUs (agent_docs/testing-debug.md §RT-1).
// A core whose RT path calls into those libraries therefore needs THIS gate for
// an "allocates nothing" claim to mean anything.
//
// How: the test executable DEFINES malloc/free/calloc/realloc and the aligned
// family. The dynamic linker resolves every shared object's malloc reference to
// the first definition in the lookup scope, and the executable comes first — so
// allocations inside libpinocchio, a sibling package's library or libstdc++
// land here too. Each replacement forwards to glibc's `__libc_*` entry point
// and counts while a ScopedMallocGate is alive on the calling thread.
//
// CONTRACT — read before use:
//
//  1. Include this header in EXACTLY ONE translation unit per test binary (it
//     defines non-inline C functions; a second TU is a multiple-definition link
//     error). An operator-new gate follows the same rule, and the two may share
//     that TU.
//  2. glibc only: forwards to `__libc_malloc` & co. (exported by glibc ≥ 2.2.5).
//     Covers malloc, calloc, realloc, memalign, aligned_alloc, posix_memalign,
//     valloc and pvalloc — every public allocation entry point of glibc.
//     Other C libraries are not supported and fail to link, not silently.
//  3. Arm with ScopedMallocGate (RAII), never a bare flag — an ASSERT_* inside
//     the measured region returns from the test and would leave a bare flag
//     armed.
//  4. PROVE THE GATE FIRES before trusting a zero: the positive control must be
//     an allocation made INSIDE a library (e.g. constructing pinocchio::Data),
//     not one in the test TU, or it proves only what an operator-new counter
//     already proves.
//
// `free` is not counted: releasing memory on the RT path is also an RT-1
// violation, but every free pairs with a counted allocation in the same
// steady-state loop, so counting allocations is sufficient and keeps the
// report to one number.
#pragma once

// glibc declares the malloc family `noexcept` (__THROW), and so does GCC's
// <mm_malloc.h>, which the SIMD headers pull in. The replacements below carry
// the same specifier, so a declaration seen before OR after this header agrees
// with them — the header does not depend on its include position.
#include <malloc.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdlib>

extern "C" {
void* __libc_malloc(std::size_t size);
void __libc_free(void* ptr);
void* __libc_calloc(std::size_t nmemb, std::size_t size);
void* __libc_realloc(void* ptr, std::size_t size);
void* __libc_memalign(std::size_t alignment, std::size_t size);
}

namespace rtc::testing {

namespace detail {

// Trivially-destructible thread_locals in the executable live in static TLS,
// so touching them from inside malloc cannot itself allocate or recurse.
[[nodiscard]] inline int& MallocGateDepth() noexcept {
  static thread_local int depth = 0;
  return depth;
}

[[nodiscard]] inline std::size_t& MallocGateCount() noexcept {
  static thread_local std::size_t count = 0;
  return count;
}

inline void MallocGateNote() noexcept {
  if (MallocGateDepth() > 0) {
    ++MallocGateCount();
  }
}

}  // namespace detail

/// @brief RAII: count C-level heap allocations (malloc family, from any
///        library) on the current thread for the scope's lifetime.
/// @note An inner scope re-zeroes the shared counter — read count() first.
class ScopedMallocGate {
 public:
  ScopedMallocGate() noexcept {
    detail::MallocGateCount() = 0;
    ++detail::MallocGateDepth();
  }

  ~ScopedMallocGate() { --detail::MallocGateDepth(); }

  ScopedMallocGate(const ScopedMallocGate&) = delete;
  ScopedMallocGate& operator=(const ScopedMallocGate&) = delete;
  ScopedMallocGate(ScopedMallocGate&&) = delete;
  ScopedMallocGate& operator=(ScopedMallocGate&&) = delete;

  /// @brief malloc-family calls observed since this scope began.
  [[nodiscard]] std::size_t count() const noexcept { return detail::MallocGateCount(); }
};

}  // namespace rtc::testing

// ── Replacement C allocator entry points (contract item 1: one TU per binary) ─

extern "C" {

void* malloc(std::size_t size) noexcept {
  ::rtc::testing::detail::MallocGateNote();
  return __libc_malloc(size);
}

void free(void* ptr) noexcept {
  __libc_free(ptr);
}

void* calloc(std::size_t nmemb, std::size_t size) noexcept {
  ::rtc::testing::detail::MallocGateNote();
  return __libc_calloc(nmemb, size);
}

void* realloc(void* ptr, std::size_t size) noexcept {
  ::rtc::testing::detail::MallocGateNote();
  return __libc_realloc(ptr, size);
}

void* memalign(std::size_t alignment, std::size_t size) noexcept {
  ::rtc::testing::detail::MallocGateNote();
  return __libc_memalign(alignment, size);
}

void* aligned_alloc(std::size_t alignment, std::size_t size) noexcept {
  ::rtc::testing::detail::MallocGateNote();
  return __libc_memalign(alignment, size);
}

// glibc routes valloc/pvalloc through its internal _mid_memalign, not the
// interposable memalign, so they need their own entry points.
void* valloc(std::size_t size) noexcept {
  ::rtc::testing::detail::MallocGateNote();
  return __libc_memalign(static_cast<std::size_t>(::sysconf(_SC_PAGESIZE)), size);
}

void* pvalloc(std::size_t size) noexcept {
  ::rtc::testing::detail::MallocGateNote();
  const auto page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
  const std::size_t rounded = (size + page - 1) / page * page;
  return __libc_memalign(page, rounded);
}

int posix_memalign(void** out, std::size_t alignment, std::size_t size) noexcept {
  ::rtc::testing::detail::MallocGateNote();
  // posix_memalign's contract: alignment is a power of two multiple of
  // sizeof(void*), EINVAL otherwise.
  if (alignment < sizeof(void*) || (alignment & (alignment - 1)) != 0) {
    return EINVAL;
  }
  void* p = __libc_memalign(alignment, size);
  if (p == nullptr) {
    return ENOMEM;
  }
  *out = p;
  return 0;
}

}  // extern "C"
