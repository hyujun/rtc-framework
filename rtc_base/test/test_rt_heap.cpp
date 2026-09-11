// ── ConfigureRtHeap(): both glibc settings actually take effect ──────────────
//
// Its own binary on purpose: mallopt() is process-global and cannot be undone
// within a process, so sharing a binary would change the heap under every
// other suite in it.
//
// The observable is mallinfo2(): `hblkhd` counts mmapped bytes, `arena` the
// heap the process holds. A 64 MiB block is the probe because it is above
// glibc's largest dynamic mmap threshold (32 MiB on 64-bit) — without
// M_MMAP_MAX 0 it is ALWAYS mmapped, so the threshold's own adaptation cannot
// make the first assertion pass by accident. Freeing it leaves 64 MiB at the
// top of the heap, far above the default trim threshold, so without
// M_TRIM_THRESHOLD -1 the free always trims.

#include "rtc_base/threading/rt_heap.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdlib>

#if defined(__GLIBC__)
#include <malloc.h>
#endif

namespace {

constexpr std::size_t kProbeBytes = std::size_t{64} << 20;

}  // namespace

TEST(RtHeap, LargeBlocksStayInTheHeapAndFreedMemoryIsKept) {
#if defined(__GLIBC__) && __GLIBC_PREREQ(2, 33)
  ASSERT_TRUE(rtc::ConfigureRtHeap());

  const struct mallinfo2 before = mallinfo2();
  // volatile: a malloc whose result is never used may be elided.
  void* volatile block = std::malloc(kProbeBytes);
  ASSERT_NE(block, nullptr);
  const struct mallinfo2 held = mallinfo2();
  std::free(block);
  const struct mallinfo2 after = mallinfo2();

  EXPECT_EQ(held.hblkhd, before.hblkhd) << "the block was mmapped — M_MMAP_MAX 0 is not in effect";
  EXPECT_GE(held.arena, before.arena + kProbeBytes) << "the block did not come from the heap";
  EXPECT_EQ(after.arena, held.arena)
      << "freeing it gave memory back to the kernel — M_TRIM_THRESHOLD -1 is not in effect";
#else
  GTEST_SKIP() << "glibc >= 2.33 only (mallopt semantics + mallinfo2)";
#endif
}
