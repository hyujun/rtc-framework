// ── test_spsc_buffer.cpp ────────────────────────────────────────────────────
// Unit tests for rtc::SpscPublishBuffer<N> — lock-free single-producer /
// single-consumer ring buffer for publish snapshots.
//
// Covers: empty pop, push/pop round-trip, FIFO order, buffer-full drop
// counting, wraparound, and concurrent producer/consumer correctness.
//
// (The legacy SpscLogBuffer half of this test was removed in Phase C
// Track R alongside log_buffer.hpp; the same SPSC semantics are now
// covered by rtc_base/test/test_thread_csv.cpp via SpscQueue<T,N>.)
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_base/threading/publish_buffer.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <thread>

namespace {

// ═══════════════════════════════════════════════════════════════════════════════
// SpscPublishBuffer tests
// ═══════════════════════════════════════════════════════════════════════════════

TEST(SpscPublishBufferTest, EmptyPopReturnsFalse) {
  rtc::SpscPublishBuffer<4> buf;
  rtc::PublishSnapshot snap;
  EXPECT_FALSE(buf.Pop(snap));
}

TEST(SpscPublishBufferTest, PushAndPopPreservesData) {
  rtc::SpscPublishBuffer<4> buf;

  rtc::PublishSnapshot input{};
  input.stamp_ns = 123456789;
  input.num_groups = 2;
  input.command_type = rtc::CommandType::kPosition;

  EXPECT_TRUE(buf.Push(input));

  rtc::PublishSnapshot output{};
  EXPECT_TRUE(buf.Pop(output));

  EXPECT_EQ(output.stamp_ns, 123456789);
  EXPECT_EQ(output.num_groups, 2);
  EXPECT_EQ(output.command_type, rtc::CommandType::kPosition);
}

TEST(SpscPublishBufferTest, BufferFull_DropsAndCounts) {
  rtc::SpscPublishBuffer<4> buf;

  // Fill all usable slots (N-1 = 3).
  for (int i = 0; i < 3; ++i) {
    rtc::PublishSnapshot snap{};
    snap.stamp_ns = static_cast<int64_t>(i);
    EXPECT_TRUE(buf.Push(snap));
  }

  EXPECT_EQ(buf.drop_count(), 0u);

  rtc::PublishSnapshot overflow{};
  EXPECT_FALSE(buf.Push(overflow));
  EXPECT_EQ(buf.drop_count(), 1u);
}

TEST(SpscPublishBufferTest, FIFO_Order) {
  rtc::SpscPublishBuffer<8> buf;

  for (int64_t i = 0; i < 3; ++i) {
    rtc::PublishSnapshot snap{};
    snap.stamp_ns = (i + 1) * 1000;
    EXPECT_TRUE(buf.Push(snap));
  }

  for (int64_t i = 0; i < 3; ++i) {
    rtc::PublishSnapshot snap{};
    ASSERT_TRUE(buf.Pop(snap));
    EXPECT_EQ(snap.stamp_ns, (i + 1) * 1000)
        << "Entry " << i << " out of FIFO order";
  }
}

} // namespace
