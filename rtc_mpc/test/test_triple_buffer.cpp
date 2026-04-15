// TripleBuffer unit + stress tests.
//
// Covers:
//   * Initial state has no data available.
//   * Single-producer / single-consumer round-trips return the most recent
//     value, not any intermediate.
//   * Stress: 1 producer + 1 consumer driving ~1M write/read cycles.
//     - Published sequence numbers never regress.
//     - Consumer never sees torn / in-flight data (we publish only fully
//       initialized payloads, so a torn read would surface as inconsistency
//       between the sequence number and the payload).

#include "rtc_mpc/comm/triple_buffer.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <thread>
#include <type_traits>

namespace rtc::mpc {
namespace {

constexpr std::size_t kPayloadWords = 8;
constexpr uint64_t kSentinelSeq = 42;
constexpr uint64_t kBurstCount = 5;
constexpr uint64_t kStressIterations = 1'000'000;

/// Simple POD payload where every word encodes the producer's sequence number.
/// Redundant copies enable torn-read detection in the stress test.
struct Message {
  uint64_t seq;
  std::array<uint64_t, kPayloadWords> payload;
};

static_assert(std::is_trivially_copyable_v<Message>);

TEST(TripleBuffer, EmptyOnInit) {
  TripleBuffer<Message> buf;
  EXPECT_FALSE(buf.HasNewData());
  EXPECT_EQ(buf.TryAcquireLatest(), nullptr);
}

TEST(TripleBuffer, SinglePublishRoundTrip) {
  TripleBuffer<Message> buf;

  auto& slot = buf.StartWrite();
  slot.seq = kSentinelSeq;
  for (auto& word : slot.payload) {
    word = kSentinelSeq;
  }
  buf.FinishWrite();

  EXPECT_TRUE(buf.HasNewData());
  const Message* got = buf.TryAcquireLatest();
  ASSERT_NE(got, nullptr);
  EXPECT_EQ(got->seq, kSentinelSeq);
  EXPECT_EQ(got->payload[0], kSentinelSeq);

  // Acquire again without a new publish — must report no new data.
  EXPECT_EQ(buf.TryAcquireLatest(), nullptr);

  // Current() still reflects the most recently acquired buffer.
  EXPECT_EQ(buf.Current().seq, kSentinelSeq);
}

TEST(TripleBuffer, LatestOnlyWhenProducerOutpacesConsumer) {
  TripleBuffer<Message> buf;

  for (uint64_t i = 1; i <= kBurstCount; ++i) {
    auto& slot = buf.StartWrite();
    slot.seq = i;
    for (auto& word : slot.payload) {
      word = i;
    }
    buf.FinishWrite();
  }

  const Message* got = buf.TryAcquireLatest();
  ASSERT_NE(got, nullptr);
  EXPECT_EQ(got->seq, kBurstCount)
      << "Consumer must skip to the freshest buffer";
}

TEST(TripleBuffer, StressOneProducerOneConsumer) {
  TripleBuffer<Message> buf;

  std::atomic<bool> producer_done{false};
  uint64_t max_observed = 0;
  uint64_t torn_reads = 0;
  uint64_t regressions = 0;
  uint64_t last_seen = 0;

  std::jthread producer([&] {
    for (uint64_t i = 1; i <= kStressIterations; ++i) {
      auto& slot = buf.StartWrite();
      slot.seq = i;
      for (auto& word : slot.payload) {
        word = i;
      }
      buf.FinishWrite();
    }
    producer_done.store(true, std::memory_order_release);
  });

  // Consumer drains until producer finishes and no more data remains.
  while (true) {
    const Message* got = buf.TryAcquireLatest();
    if (got == nullptr) {
      if (producer_done.load(std::memory_order_acquire) &&
          !buf.HasNewData()) {
        break;
      }
      std::this_thread::yield();
      continue;
    }
    const uint64_t seq = got->seq;
    for (const auto& word : got->payload) {
      if (word != seq) {
        ++torn_reads;
        break;
      }
    }
    if (seq < last_seen) {
      ++regressions;
    }
    last_seen = seq;
    if (seq > max_observed) {
      max_observed = seq;
    }
  }

  producer.join();
  EXPECT_EQ(torn_reads, 0U) << "Consumer saw a torn payload";
  EXPECT_EQ(regressions, 0U) << "Acquired sequence went backwards";
  EXPECT_EQ(max_observed, kStressIterations)
      << "Final acquire must observe the last published buffer";
}

}  // namespace
}  // namespace rtc::mpc
