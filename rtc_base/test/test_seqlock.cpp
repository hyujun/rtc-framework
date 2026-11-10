// ── test_seqlock.cpp ────────────────────────────────────────────────────────
// Unit tests for rtc::SeqLock<T> — lock-free single-writer / multi-reader
// synchronisation primitive.
//
// Covers: default construction, store/load, sequence monotonicity, large
// struct correctness, concurrent read-during-write consistency, and the
// word-by-word payload copy: sizes that are not a multiple of 8 (tail bytes),
// 4-byte-aligned T, and default construction honouring T's default member
// initialisers.
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_base/threading/seqlock.hpp>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <thread>

namespace {

// Simple trivially-copyable struct used throughout the tests.
struct TestData {
  std::array<double, 16> values{};
  int64_t counter{0};
};

static_assert(std::is_trivially_copyable_v<TestData>,
              "TestData must be trivially copyable for SeqLock");

// ── DefaultConstruction ─────────────────────────────────────────────────────
// Load from a default-constructed SeqLock returns zero-initialized T.
TEST(SeqLockTest, DefaultConstruction) {
  rtc::SeqLock<TestData> sl;
  const TestData loaded = sl.Load();

  EXPECT_EQ(loaded.counter, 0);
  for (std::size_t i = 0; i < loaded.values.size(); ++i) {
    EXPECT_DOUBLE_EQ(loaded.values[i], 0.0) << "index " << i;
  }
  EXPECT_EQ(sl.sequence(), 0u);
}

// ── StoreAndLoad ────────────────────────────────────────────────────────────
// Store a value, Load returns the same value.
TEST(SeqLockTest, StoreAndLoad) {
  rtc::SeqLock<TestData> sl;

  TestData written{};
  written.counter = 42;
  for (std::size_t i = 0; i < written.values.size(); ++i) {
    written.values[i] = static_cast<double>(i) * 1.5;
  }

  sl.Store(written);
  const TestData loaded = sl.Load();

  EXPECT_EQ(loaded.counter, 42);
  for (std::size_t i = 0; i < loaded.values.size(); ++i) {
    EXPECT_DOUBLE_EQ(loaded.values[i], static_cast<double>(i) * 1.5) << "index " << i;
  }
}

// ── MultipleStoreLoad ───────────────────────────────────────────────────────
// Store 3 different values sequentially, Load returns the latest.
TEST(SeqLockTest, MultipleStoreLoad) {
  rtc::SeqLock<TestData> sl;

  for (int64_t n = 1; n <= 3; ++n) {
    TestData d{};
    d.counter = n;
    d.values.fill(static_cast<double>(n));
    sl.Store(d);
  }

  const TestData loaded = sl.Load();
  EXPECT_EQ(loaded.counter, 3);
  for (const auto& v : loaded.values) {
    EXPECT_DOUBLE_EQ(v, 3.0);
  }
}

// ── SequenceMonotonicity ────────────────────────────────────────────────────
// sequence() increases by 2 after each Store (even numbers only when stable).
TEST(SeqLockTest, SequenceMonotonicity) {
  rtc::SeqLock<TestData> sl;
  EXPECT_EQ(sl.sequence(), 0u);

  TestData d{};
  for (uint32_t i = 1; i <= 5; ++i) {
    d.counter = static_cast<int64_t>(i);
    sl.Store(d);
    const uint32_t seq = sl.sequence();
    EXPECT_EQ(seq, i * 2) << "after Store #" << i;
    EXPECT_EQ(seq % 2, 0u) << "sequence must be even when no write in progress";
  }
}

// ── LargeStruct ─────────────────────────────────────────────────────────────
// Test with a struct containing large arrays to verify memcpy correctness.
TEST(SeqLockTest, LargeStruct) {
  // Use a struct similar in spirit to DeviceState (many doubles).
  struct LargeData {
    std::array<double, 64> positions{};
    std::array<double, 64> velocities{};
    std::array<double, 64> efforts{};
    int64_t id{0};
  };

  static_assert(std::is_trivially_copyable_v<LargeData>);

  rtc::SeqLock<LargeData> sl;

  LargeData written{};
  written.id = 99;
  for (std::size_t i = 0; i < 64; ++i) {
    written.positions[i] = static_cast<double>(i);
    written.velocities[i] = static_cast<double>(i) * 0.1;
    written.efforts[i] = static_cast<double>(i) * -0.5;
  }

  sl.Store(written);
  const LargeData loaded = sl.Load();

  EXPECT_EQ(loaded.id, 99);
  for (std::size_t i = 0; i < 64; ++i) {
    EXPECT_DOUBLE_EQ(loaded.positions[i], static_cast<double>(i));
    EXPECT_DOUBLE_EQ(loaded.velocities[i], static_cast<double>(i) * 0.1);
    EXPECT_DOUBLE_EQ(loaded.efforts[i], static_cast<double>(i) * -0.5);
  }
}

// ── ConcurrentReadDuringWrite ───────────────────────────────────────────────
// Launch a writer thread that stores 1000 values and a reader thread that
// loads concurrently.  Verify Load never returns partially-written data:
// if counter == N, all values[i] must equal N.
TEST(SeqLockTest, ConcurrentReadDuringWrite) {
  rtc::SeqLock<TestData> sl;

  constexpr int kNumWrites = 1000;
  std::atomic<bool> writer_done{false};
  std::atomic<int> inconsistency_count{0};

  // Writer thread: stores values 1..kNumWrites.
  std::jthread writer([&](std::stop_token /*st*/) {
    for (int n = 1; n <= kNumWrites; ++n) {
      TestData d{};
      d.counter = n;
      d.values.fill(static_cast<double>(n));
      sl.Store(d);
    }
    writer_done.store(true, std::memory_order_release);
  });

  // Reader thread: loads continuously and checks consistency.
  std::jthread reader([&](std::stop_token /*st*/) {
    while (!writer_done.load(std::memory_order_acquire)) {
      const TestData snapshot = sl.Load();
      const auto expected = static_cast<double>(snapshot.counter);
      for (std::size_t i = 0; i < snapshot.values.size(); ++i) {
        if (snapshot.values[i] != expected) {
          inconsistency_count.fetch_add(1, std::memory_order_relaxed);
          return;  // One failure is enough to detect the problem.
        }
      }
    }
  });

  writer.join();
  reader.join();

  EXPECT_EQ(inconsistency_count.load(), 0) << "SeqLock returned partially-written (torn) data";

  // Final load must return the last written value.
  const TestData final_val = sl.Load();
  EXPECT_EQ(final_val.counter, kNumWrites);
  for (const auto& v : final_val.values) {
    EXPECT_DOUBLE_EQ(v, static_cast<double>(kNumWrites));
  }
}

// LoadInto is the same read into caller storage: consistent under a
// concurrent writer, and equal to Load() once the writer is done.
TEST(SeqLockTest, LoadIntoIsConsistentUnderAConcurrentWriter) {
  rtc::SeqLock<TestData> sl;
  constexpr int kNumWrites = 1000;
  std::atomic<bool> writer_done{false};
  std::atomic<int> inconsistency_count{0};
  std::jthread writer([&](std::stop_token /*st*/) {
    for (int n = 1; n <= kNumWrites; ++n) {
      TestData d{};
      d.counter = n;
      d.values.fill(static_cast<double>(n));
      sl.Store(d);
    }
    writer_done.store(true, std::memory_order_release);
  });
  std::jthread reader([&](std::stop_token /*st*/) {
    TestData snapshot{};
    while (!writer_done.load(std::memory_order_acquire)) {
      sl.LoadInto(snapshot);
      for (const auto& v : snapshot.values) {
        if (v != static_cast<double>(snapshot.counter)) {
          inconsistency_count.fetch_add(1, std::memory_order_relaxed);
          return;
        }
      }
    }
  });
  writer.join();
  reader.join();
  EXPECT_EQ(inconsistency_count.load(), 0) << "LoadInto returned torn data";
  TestData out{};
  sl.LoadInto(out);
  EXPECT_EQ(out.counter, kNumWrites);
  EXPECT_EQ(out.counter, sl.Load().counter);
}

// ── Word-by-word payload copy ───────────────────────────────────────────────
// The payload is copied as 8-byte atomic words plus a tail; these pin the
// tail path, a T whose alignment is below the word size, and default values.

// 4-byte aligned, 20 bytes: two full words + a 4-byte tail.
struct OddSized {
  std::array<uint32_t, 5> v{};
};

static_assert(sizeof(OddSized) % 8 != 0);
static_assert(alignof(OddSized) == 4);

// 1-byte aligned, 1157 bytes: 144 full words + a 5-byte tail — byte-level
// layout so a misplaced tail byte is visible.
struct ByteSized {
  std::array<uint8_t, 1157> b{};
};

static_assert(sizeof(ByteSized) % 8 == 5);

TEST(SeqLockTest, OddSizedRoundTripIncludesTailBytes) {
  rtc::SeqLock<OddSized> sl;
  OddSized written{};
  for (std::size_t i = 0; i < written.v.size(); ++i) {
    written.v[i] = 0xA5A50000u + static_cast<uint32_t>(i);
  }
  sl.Store(written);
  const OddSized loaded = sl.Load();
  for (std::size_t i = 0; i < written.v.size(); ++i) {
    EXPECT_EQ(loaded.v[i], written.v[i]) << "index " << i;
  }
}

TEST(SeqLockTest, ByteSizedRoundTripThroughLoadAndLoadInto) {
  rtc::SeqLock<ByteSized> sl;
  ByteSized written{};
  for (std::size_t i = 0; i < written.b.size(); ++i) {
    written.b[i] = static_cast<uint8_t>(i * 7u + 1u);
  }
  sl.Store(written);

  const ByteSized by_value = sl.Load();
  // LoadInto into storage pre-filled with a different pattern, so a tail byte
  // the copy skips would keep the old value.
  ByteSized into{};
  into.b.fill(0xEE);
  sl.LoadInto(into);
  for (std::size_t i = 0; i < written.b.size(); ++i) {
    ASSERT_EQ(by_value.b[i], written.b[i]) << "Load, index " << i;
    ASSERT_EQ(into.b[i], written.b[i]) << "LoadInto, index " << i;
  }
}

TEST(SeqLockTest, InitialValueConstructorHoldsTheValue) {
  OddSized initial{};
  initial.v = {1u, 2u, 3u, 4u, 5u};
  const rtc::SeqLock<OddSized> sl(initial);
  const OddSized loaded = sl.Load();
  for (std::size_t i = 0; i < initial.v.size(); ++i) {
    EXPECT_EQ(loaded.v[i], initial.v[i]) << "index " << i;
  }
  EXPECT_EQ(sl.sequence(), 0u);
}

// Several SeqLock payloads in the tree have non-zero default member
// initialisers; a default-constructed SeqLock must hold T{}, not zero bytes.
struct NonZeroDefaults {
  double gain{1.0};
  int32_t status{-1};
  bool stale{true};
};

TEST(SeqLockTest, DefaultConstructionHoldsValueInitialisedT) {
  const rtc::SeqLock<NonZeroDefaults> sl;
  const NonZeroDefaults loaded = sl.Load();
  EXPECT_DOUBLE_EQ(loaded.gain, 1.0);
  EXPECT_EQ(loaded.status, -1);
  EXPECT_TRUE(loaded.stale);
}

}  // namespace
