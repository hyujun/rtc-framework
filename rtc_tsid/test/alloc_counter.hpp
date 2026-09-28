#pragma once

// ── TU-local global allocation counter for RT zero-alloc tests ───────────────
// Counts global new/delete while armed; unarmed (fixture / gtest) traffic is
// ignored. (rtc_mpc test_utils::AllocCounter pattern.)
//
// This header DEFINES the global operator new/delete replacements, so it must be
// included in exactly ONE translation unit per test executable. Some test
// binaries here link several .cpp files (add_tsid_gtest with stems in
// CMakeLists.txt); a file that includes this header is registered as a binary
// of its own, so each gets its own private copy of the overrides.
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <new>

namespace rtc::tsid::test {

struct AllocCounter {
  inline static std::atomic<std::int64_t> alloc_count{0};
  inline static std::atomic<bool> armed{false};

  static void Arm() noexcept {
    alloc_count.store(0, std::memory_order_relaxed);
    armed.store(true, std::memory_order_release);
  }

  static void Disarm() noexcept { armed.store(false, std::memory_order_release); }

  static void Record() noexcept {
    if (armed.load(std::memory_order_acquire)) {
      alloc_count.fetch_add(1, std::memory_order_relaxed);
    }
  }
};

}  // namespace rtc::tsid::test

// GCC 13+ -Wmismatched-new-delete fires on `new ... -> std::free` pairs it
// traces through these overrides; the pairing is correct (our operator new
// itself mallocs). Canonical suppression for global allocator overrides.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif

inline void* operator new(std::size_t sz) {
  void* p = std::malloc(sz);
  if (p == nullptr) {
    throw std::bad_alloc{};
  }
  rtc::tsid::test::AllocCounter::Record();
  return p;
}

inline void* operator new[](std::size_t sz) {
  void* p = std::malloc(sz);
  if (p == nullptr) {
    throw std::bad_alloc{};
  }
  rtc::tsid::test::AllocCounter::Record();
  return p;
}

inline void operator delete(void* p) noexcept {
  std::free(p);
}

inline void operator delete[](void* p) noexcept {
  std::free(p);
}

inline void operator delete(void* p, std::size_t) noexcept {
  std::free(p);
}

inline void operator delete[](void* p, std::size_t) noexcept {
  std::free(p);
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
