// Unit tests for rtc::SpscQueue<T, N> (concurrency/spsc_queue.hpp), the generic
// lock-free SPSC ring underlying ThreadCsvProducer / ThreadTimingProducer. The
// publish-buffer specialisation is covered separately in test_spsc_buffer.cpp;
// this exercises the generic template directly:
//   - Empty pop, push/pop round-trip with data preservation.
//   - Full-buffer drop accounting (ring holds N-1 usable slots).
//   - FIFO order across index wraparound.
//   - Single-producer / single-consumer threaded correctness (strict FIFO,
//     no torn reads); the producer spins on full so every item is delivered.

#include "rtc_base/concurrency/spsc_queue.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <thread>

namespace {

struct Entry {
  std::uint64_t seq{0};
  double value{0.0};
};

TEST(SpscQueueTest, EmptyPopReturnsFalse) {
  rtc::SpscQueue<Entry, 4> q;
  Entry out;
  EXPECT_FALSE(q.Pop(out));
}

TEST(SpscQueueTest, PushPopRoundTripPreservesData) {
  rtc::SpscQueue<Entry, 4> q;
  EXPECT_TRUE(q.Push(Entry{7, 3.5}));
  Entry out;
  ASSERT_TRUE(q.Pop(out));
  EXPECT_EQ(out.seq, 7U);
  EXPECT_DOUBLE_EQ(out.value, 3.5);
  EXPECT_FALSE(q.Pop(out));  // emptied
}

TEST(SpscQueueTest, FullBufferDropsAndCounts) {
  rtc::SpscQueue<int, 4> q;  // ring holds N-1 = 3 usable slots
  EXPECT_TRUE(q.Push(1));
  EXPECT_TRUE(q.Push(2));
  EXPECT_TRUE(q.Push(3));
  EXPECT_FALSE(q.Push(4));  // full → dropped
  EXPECT_EQ(q.drop_count(), 1U);
}

TEST(SpscQueueTest, FifoOrderWithWraparound) {
  rtc::SpscQueue<int, 4> q;
  int out = 0;
  // Push/pop more than capacity to exercise the power-of-two index wraparound.
  for (int k = 0; k < 20; ++k) {
    ASSERT_TRUE(q.Push(k));
    ASSERT_TRUE(q.Pop(out));
    EXPECT_EQ(out, k);
  }
  EXPECT_EQ(q.drop_count(), 0U);
}

TEST(SpscQueueTest, SingleProducerSingleConsumerOrdered) {
  rtc::SpscQueue<std::uint64_t, 1024> q;
  constexpr std::uint64_t kCount = 100000;

  std::thread producer([&] {
    for (std::uint64_t i = 0; i < kCount; ++i) {
      while (!q.Push(i)) {
        // spin until the consumer frees a slot — guarantees no drops
      }
    }
  });

  std::uint64_t expected = 0;
  std::uint64_t v = 0;
  while (expected < kCount) {
    if (q.Pop(v)) {
      ASSERT_EQ(v, expected);  // strict FIFO, no torn / reordered reads
      ++expected;
    }
  }
  producer.join();
  EXPECT_EQ(expected, kCount);  // every item delivered in order
  // Note: drop_count() is NOT asserted here — Push() increments it on every
  // full-failure, so the producer's spin-retry inflates it without losing data.
}

}  // namespace
