// Unit tests for the per-tick thread-timing infrastructure:
//   ThreadTimingProducer<P, N>  (rtc_base/timing/thread_timing_producer.hpp)
//   ThreadTimingCsvLogger<P>    (rtc_base/timing/thread_timing_csv_logger.hpp)
//
// Producer-side: SPSC push, monotonic tick_count, drop accounting.
// Logger-side:   header emission, payload columns, append-on-reopen,
//                no-op when not open, end-to-end producer→drain→csv.

#include "rtc_base/timing/thread_timing_csv_logger.hpp"
#include "rtc_base/timing/thread_timing_producer.hpp"
#include "rtc_base/timing/thread_timing_sample.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <string>
#include <vector>

namespace
{

namespace fs = std::filesystem;

struct TestPayload
{
  double a{0.0};
  std::uint32_t b{0};
};

void WriteHeader(std::ostream & os) {os << ",a,b";}
void WriteRow(std::ostream & os, const TestPayload & p)
{
  os << ',' << p.a << ',' << p.b;
}

class ScopedTempDir {
public:
  ScopedTempDir()
  {
    auto base = fs::temp_directory_path() / "rtc_thread_timing_test";
    fs::create_directories(base);
    dir_ =
      base / ("t_" + std::to_string(reinterpret_cast<std::uintptr_t>(this) &
                                      0xFFFFFFFFu));
    fs::create_directories(dir_);
  }
  ~ScopedTempDir()
  {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  const fs::path & dir() const noexcept {return dir_;}

private:
  fs::path dir_;
};

std::vector<std::string> ReadAllLines(const fs::path & p)
{
  std::vector<std::string> lines;
  std::ifstream in(p);
  for (std::string line; std::getline(in, line); ) {
    lines.push_back(line);
  }
  return lines;
}

} // namespace

// ── Producer tests ──────────────────────────────────────────────────────────

TEST(ThreadTimingProducer, PushIncrementsTickAndDrainPreservesOrder) {
  rtc::ThreadTimingProducer<TestPayload, 8> producer;

  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(
        producer.Push({static_cast<double>(i), static_cast<uint32_t>(i * 10)}));
  }
  EXPECT_EQ(producer.LastTickCount(), 5u);
  EXPECT_EQ(producer.DropCount(), 0u);

  std::vector<rtc::ThreadTimingSample<TestPayload>> drained;
  const auto n = producer.Drain([&](const auto & s) {drained.push_back(s);});
  EXPECT_EQ(n, 5u);
  ASSERT_EQ(drained.size(), 5u);
  for (std::size_t i = 0; i < drained.size(); ++i) {
    EXPECT_EQ(drained[i].tick_count, i + 1);
    EXPECT_EQ(drained[i].payload.a, static_cast<double>(i));
    EXPECT_EQ(drained[i].payload.b, static_cast<uint32_t>(i * 10));
    EXPECT_GT(drained[i].t_wall_ns, 0u);
  }
}

TEST(ThreadTimingProducer, FullRingDrops) {
  // Capacity 4 → 3 usable slots.
  rtc::ThreadTimingProducer<TestPayload, 4> producer;
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(producer.Push({}));
  }
  EXPECT_FALSE(producer.Push({})); // full
  EXPECT_EQ(producer.DropCount(), 1u);
  // Tick count counts attempts including the dropped one.
  EXPECT_EQ(producer.LastTickCount(), 4u);
}

// ── Logger tests ────────────────────────────────────────────────────────────

TEST(ThreadTimingCsvLogger, FirstOpenWritesHeaderWithPayloadColumns) {
  ScopedTempDir scope;
  const auto path = scope.dir() / "t.csv";
  rtc::ThreadTimingCsvLogger<TestPayload> logger;

  ASSERT_TRUE(logger.Open(path, &WriteHeader, &WriteRow));
  const auto lines = ReadAllLines(path);
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "t_wall_ns,tick_count,a,b");
}

TEST(ThreadTimingCsvLogger, EmptyPayloadHeaderHasOnlyTimingColumns) {
  ScopedTempDir scope;
  const auto path = scope.dir() / "t.csv";
  rtc::ThreadTimingCsvLogger<TestPayload> logger;

  // Pass a no-op header writer to mean "no payload columns".
  ASSERT_TRUE(logger.Open(
      path, [](std::ostream &) {}, [](std::ostream &, const TestPayload &) {}));
  const auto lines = ReadAllLines(path);
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "t_wall_ns,tick_count");
}

TEST(ThreadTimingCsvLogger, ReopenAppendsWithoutDuplicateHeader) {
  ScopedTempDir scope;
  const auto path = scope.dir() / "t.csv";
  {
    rtc::ThreadTimingCsvLogger<TestPayload> logger;
    ASSERT_TRUE(logger.Open(path, &WriteHeader, &WriteRow));
    rtc::ThreadTimingSample<TestPayload> s{};
    s.t_wall_ns = 1000;
    s.tick_count = 1;
    s.payload = {1.5, 7};
    logger.Log(s);
  }
  {
    rtc::ThreadTimingCsvLogger<TestPayload> logger;
    ASSERT_TRUE(logger.Open(path, &WriteHeader, &WriteRow));
    rtc::ThreadTimingSample<TestPayload> s{};
    s.t_wall_ns = 2000;
    s.tick_count = 2;
    s.payload = {2.5, 14};
    logger.Log(s);
  }

  const auto lines = ReadAllLines(path);
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_NE(lines[0].find("t_wall_ns"), std::string::npos);
  EXPECT_EQ(lines[0], "t_wall_ns,tick_count,a,b");
  EXPECT_EQ(lines[1], "1000,1,1.5,7");
  EXPECT_EQ(lines[2], "2000,2,2.5,14");
}

TEST(ThreadTimingCsvLogger, LogIsNoopWhenNotOpen) {
  rtc::ThreadTimingCsvLogger<TestPayload> logger;
  EXPECT_FALSE(logger.IsOpen());
  rtc::ThreadTimingSample<TestPayload> s{};
  // Must not throw / crash even though Open() was never called.
  logger.Log(s);
  EXPECT_FALSE(logger.IsOpen());
}

// ── End-to-end ──────────────────────────────────────────────────────────────

TEST(ThreadTimingEndToEnd, ProducerDrainIntoCsv) {
  ScopedTempDir scope;
  const auto path = scope.dir() / "t.csv";
  rtc::ThreadTimingProducer<TestPayload, 16> producer;
  rtc::ThreadTimingCsvLogger<TestPayload> logger;
  ASSERT_TRUE(logger.Open(path, &WriteHeader, &WriteRow));

  // Use integer-valued doubles so the default ostream format matches a
  // simple substring (no locale / precision concerns).
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(producer.Push({static_cast<double>((i + 1) * 10),
        static_cast<uint32_t>((i + 1) * 100)}));
  }
  const auto n = producer.Drain([&](const auto & s) {logger.Log(s);});
  EXPECT_EQ(n, 4u);

  const auto lines = ReadAllLines(path);
  ASSERT_EQ(lines.size(), 5u); // header + 4 rows
  EXPECT_EQ(lines[0], "t_wall_ns,tick_count,a,b");
  for (int i = 0; i < 4; ++i) {
    const auto & row = lines[static_cast<std::size_t>(i + 1)];
    // Match a suffix the form ",<tick>,<a>,<b>". Default std::ostream
    // formats 10.0 as "10".
    const std::string expect_suffix = "," + std::to_string(i + 1) + "," +
      std::to_string((i + 1) * 10) + "," +
      std::to_string((i + 1) * 100);
    EXPECT_NE(row.find(expect_suffix), std::string::npos)
        << "row=" << row << " expected suffix " << expect_suffix;
  }
}
