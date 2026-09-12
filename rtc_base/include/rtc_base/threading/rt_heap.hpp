#ifndef RTC_BASE_THREADING_RT_HEAP_HPP_
#define RTC_BASE_THREADING_RT_HEAP_HPP_

// RT process heap policy — call once per RT process, next to mlockall().
//
// The RT path does not allocate (RT-1), with one recorded exception: ONNX
// Runtime's Run(), which allocates on every call whatever the binding style
// (agent_docs/invariants.md §"RT-1 기록된 예외"). What keeps that exception
// bounded is that, once the process has warmed up, those allocations never
// leave user space:
//
//   M_TRIM_THRESHOLD = -1  memory freed at the top of a heap is never handed
//                          back, so a free never becomes brk/madvise;
//   M_MMAP_MAX       =  0  large blocks come from the heap arenas instead of
//                          one mmap per block, so neither an allocation nor
//                          its free becomes mmap/munmap.
//
// With mlockall(MCL_CURRENT | MCL_FUTURE) (RT-HOST-1) on top, a steady-state
// allocation is a free-list operation on pages that are already resident. The
// price is the intended one: the resident set stays at its high-water mark.
//
// Both settings are process-global — every thread, every arena — so the call
// belongs at the top of main(), before the RT threads exist. It returns false
// when the C library refuses either setting, or is not glibc; the caller
// warns and runs on, because the process still works — it just no longer
// meets the exception's condition.

#include <cstdlib>

#if defined(__GLIBC__)
#include <malloc.h>
#endif

namespace rtc {

[[nodiscard]] inline bool ConfigureRtHeap() noexcept {
#if defined(__GLIBC__)
  const bool no_trim = mallopt(M_TRIM_THRESHOLD, -1) == 1;
  const bool no_mmap = mallopt(M_MMAP_MAX, 0) == 1;
  return no_trim && no_mmap;
#else
  return false;
#endif
}

}  // namespace rtc

#endif  // RTC_BASE_THREADING_RT_HEAP_HPP_
