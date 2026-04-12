// ── test_spsc_buffer.cpp ────────────────────────────────────────────────────
// Unit tests for rtc::SpscLogBuffer<N> and rtc::SpscPublishBuffer<N> —
// lock-free single-producer / single-consumer ring buffers.
//
// Covers: empty pop, push/pop round-trip, FIFO order, buffer-full drop
// counting, wraparound, and concurrent producer/consumer correctness.
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_base/logging/log_buffer.hpp>
#include <rtc_base/threading/publish_buffer.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <thread>

namespace {

// ═══════════════════════════════════════════════════════════════════════════════
// SpscLogBuffer tests
// ═══════════════════════════════════════════════════════════════════════════════

// ── EmptyPopReturnsFalse ────────────────────────────────────────────────────
TEST(SpscLogBufferTest, EmptyPopReturnsFalse) {
  rtc::SpscLogBuffer<4> buf;
  rtc::LogEntry entry;
  EXPECT_FALSE(buf.Pop(entry));
}

// ── PushAndPop ──────────────────────────────────────────────────────────────
// Push one LogEntry, Pop returns true with matching data.
TEST(SpscLogBufferTest, PushAndPop) {
  rtc::SpscLogBuffer<4> buf;

  rtc::LogEntry input{};
  input.timestamp = 1.234;
  input.t_compute_us = 55.0;
  input.num_devices = 2;

  EXPECT_TRUE(buf.Push(input));

  rtc::LogEntry output{};
  EXPECT_TRUE(buf.Pop(output));

  EXPECT_DOUBLE_EQ(output.timestamp, 1.234);
  EXPECT_DOUBLE_EQ(output.t_compute_us, 55.0);
  EXPECT_EQ(output.num_devices, 2);
}

// ── FIFO_Order ──────────────────────────────────────────────────────────────
// Push 3 entries, Pop in FIFO order (verify timestamp ordering).
TEST(SpscLogBufferTest, FIFO_Order) {
  rtc::SpscLogBuffer<8> buf;

  for (int i = 0; i < 3; ++i) {
    rtc::LogEntry entry{};
    entry.timestamp = static_cast<double>(i + 1) * 0.001;
    entry.num_devices = i;
    EXPECT_TRUE(buf.Push(entry));
  }

  for (int i = 0; i < 3; ++i) {
    rtc::LogEntry entry{};
    ASSERT_TRUE(buf.Pop(entry));
    EXPECT_DOUBLE_EQ(entry.timestamp, static_cast<double>(i + 1) * 0.001)
        << "Entry " << i << " out of FIFO order";
    EXPECT_EQ(entry.num_devices, i);
  }

  // Buffer should now be empty.
  rtc::LogEntry empty{};
  EXPECT_FALSE(buf.Pop(empty));
}

// ── BufferFull_DropsAndCounts ───────────────────────────────────────────────
// Fill buffer to capacity (N=4, usable slots = N-1 = 3), push one more
// -> returns false, drop_count increments.
TEST(SpscLogBufferTest, BufferFull_DropsAndCounts) {
  rtc::SpscLogBuffer<4> buf;  // capacity 4, usable = 3 (one slot reserved)

  // Fill all usable slots.
  for (int i = 0; i < 3; ++i) {
    rtc::LogEntry entry{};
    entry.timestamp = static_cast<double>(i);
    EXPECT_TRUE(buf.Push(entry)) << "Push #" << i << " should succeed";
  }

  EXPECT_EQ(buf.drop_count(), 0u);

  // Next push should fail (buffer full).
  rtc::LogEntry overflow{};
  overflow.timestamp = 999.0;
  EXPECT_FALSE(buf.Push(overflow));
  EXPECT_EQ(buf.drop_count(), 1u);

  // Another overflow.
  EXPECT_FALSE(buf.Push(overflow));
  EXPECT_EQ(buf.drop_count(), 2u);
}

// ── Wraparound ──────────────────────────────────────────────────────────────
// Push N-1 entries, pop all, push N-1 more (wraps around), pop all — verify
// data correctness across the wrap boundary.
TEST(SpscLogBufferTest, Wraparound) {
  rtc::SpscLogBuffer<4> buf;  // usable = 3

  // First fill + drain.
  for (int i = 0; i < 3; ++i) {
    rtc::LogEntry entry{};
    entry.timestamp = static_cast<double>(i);
    EXPECT_TRUE(buf.Push(entry));
  }
  for (int i = 0; i < 3; ++i) {
    rtc::LogEntry entry{};
    ASSERT_TRUE(buf.Pop(entry));
    EXPECT_DOUBLE_EQ(entry.timestamp, static_cast<double>(i));
  }

  // Second fill + drain (wraps around internal indices).
  for (int i = 10; i < 13; ++i) {
    rtc::LogEntry entry{};
    entry.timestamp = static_cast<double>(i);
    EXPECT_TRUE(buf.Push(entry));
  }
  for (int i = 10; i < 13; ++i) {
    rtc::LogEntry entry{};
    ASSERT_TRUE(buf.Pop(entry));
    EXPECT_DOUBLE_EQ(entry.timestamp, static_cast<double>(i))
        << "Wraparound data mismatch at i=" << i;
  }

  EXPECT_EQ(buf.drop_count(), 0u);
}

// ── ConcurrentProducerConsumer ──────────────────────────────────────────────
// Producer pushes 10000 entries, consumer pops — verify no data loss:
//   push_count - drop_count == pop_count
TEST(SpscLogBufferTest, ConcurrentProducerConsumer) {
  rtc::SpscLogBuffer<64> buf;  // Moderate size to exercise both full/empty paths.

  constexpr int kNumEntries = 10000;
  std::atomic<bool> producer_done{false};
  std::atomic<int> push_count{0};
  std::atomic<int> pop_count{0};

  std::jthread producer([&](std::stop_token /*st*/) {
    for (int i = 0; i < kNumEntries; ++i) {
      rtc::LogEntry entry{};
      entry.timestamp = static_cast<double>(i);
      entry.num_devices = i;
      if (buf.Push(entry)) {
        push_count.fetch_add(1, std::memory_order_relaxed);
      }
    }
    producer_done.store(true, std::memory_order_release);
  });

  std::jthread consumer([&](std::stop_token /*st*/) {
    int local_pop = 0;
    while (true) {
      rtc::LogEntry entry{};
      if (buf.Pop(entry)) {
        ++local_pop;
      } else if (producer_done.load(std::memory_order_acquire)) {
        // Producer finished — drain any remaining entries.
        while (buf.Pop(entry)) {
          ++local_pop;
        }
        break;
      } else {
        std::this_thread::yield();
      }
    }
    pop_count.store(local_pop, std::memory_order_relaxed);
  });

  producer.join();
  consumer.join();

  // Fundamental invariant: pushed + dropped == total attempted.
  const auto pushed = push_count.load(std::memory_order_relaxed);
  const auto dropped = static_cast<int>(buf.drop_count());
  const auto popped = pop_count.load(std::memory_order_relaxed);

  EXPECT_EQ(pushed + dropped, kNumEntries);
  // Everything successfully pushed must have been popped.
  EXPECT_EQ(pushed, popped);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SpscPublishBuffer tests (same SPSC pattern, different payload type)
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

}  // namespace
