// Unit tests for the generic per-tick CSV infrastructure used by
// controller-owned data logging:
//   ThreadCsvProducer<P, N>  (rtc_base/logging/thread_csv_producer.hpp)
//   ThreadCsvLogger<P>       (rtc_base/logging/thread_csv_logger.hpp)
//
// Distinct from ThreadTiming* (which auto-emits t_wall_ns / tick_count) —
// here the Payload owns the entire CSV row.

#include "rtc_base/logging/thread_csv_logger.hpp"
#include "rtc_base/logging/thread_csv_producer.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

struct TestRow {
  double t_relative_s{0.0};
  std::uint32_t tag{0};
};

void WriteHeader(std::ostream& os) {
  os << "t_relative_s,tag";
}

void WriteRow(std::ostream& os, const TestRow& r) {
  os << r.t_relative_s << ',' << r.tag;
}

class ScopedTempDir {
 public:
  ScopedTempDir() {
    auto base = fs::temp_directory_path() / "rtc_thread_csv_test";
    fs::create_directories(base);
    dir_ = base / ("t_" + std::to_string(reinterpret_cast<std::uintptr_t>(this) & 0xFFFFFFFFu));
    fs::create_directories(dir_);
  }

  ~ScopedTempDir() {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }

  const fs::path& dir() const noexcept { return dir_; }

 private:
  fs::path dir_;
};

std::vector<std::string> ReadAllLines(const fs::path& p) {
  std::vector<std::string> lines;
  std::ifstream in(p);
  for (std::string line; std::getline(in, line);) {
    lines.push_back(line);
  }
  return lines;
}

}  // namespace

TEST(ThreadCsvProducer, PushDrainPreservesFifoOrder) {
  rtc::ThreadCsvProducer<TestRow, 8> producer;
  for (std::uint32_t i = 0; i < 4; ++i) {
    EXPECT_TRUE(producer.Push(TestRow{0.001 * i, i}));
  }
  std::vector<std::uint32_t> seen;
  const auto n = producer.Drain([&](const TestRow& r) { seen.push_back(r.tag); });
  EXPECT_EQ(n, 4u);
  ASSERT_EQ(seen.size(), 4u);
  for (std::uint32_t i = 0; i < 4; ++i) {
    EXPECT_EQ(seen[i], i);
  }
}

TEST(ThreadCsvProducer, FullRingDropsAndCounts) {
  rtc::ThreadCsvProducer<TestRow, 4> producer;
  // SPSC ring of capacity 4 holds at most 3 entries (one slot reserved
  // for full/empty disambiguation).
  for (std::uint32_t i = 0; i < 3; ++i) {
    EXPECT_TRUE(producer.Push(TestRow{0.0, i}));
  }
  EXPECT_FALSE(producer.Push(TestRow{0.0, 99}));
  EXPECT_EQ(producer.DropCount(), 1u);
}

TEST(ThreadCsvLogger, FirstOpenWritesCallerProvidedHeader) {
  ScopedTempDir td;
  const auto path = td.dir() / "data.csv";
  rtc::ThreadCsvLogger<TestRow> logger;
  ASSERT_TRUE(logger.Open(path, &WriteHeader, &WriteRow));
  EXPECT_TRUE(logger.IsOpen());

  const auto lines = ReadAllLines(path);
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "t_relative_s,tag");
}

TEST(ThreadCsvLogger, ReopenAppendDoesNotDuplicateHeader) {
  ScopedTempDir td;
  const auto path = td.dir() / "data.csv";
  {
    rtc::ThreadCsvLogger<TestRow> logger;
    ASSERT_TRUE(logger.Open(path, &WriteHeader, &WriteRow));
    logger.Log(TestRow{1.0, 1});
  }
  {
    rtc::ThreadCsvLogger<TestRow> logger;
    ASSERT_TRUE(logger.Open(path, &WriteHeader, &WriteRow));
    logger.Log(TestRow{2.0, 2});
  }
  const auto lines = ReadAllLines(path);
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[0], "t_relative_s,tag");
  EXPECT_EQ(lines[1], "1,1");
  EXPECT_EQ(lines[2], "2,2");
}

TEST(ThreadCsvLogger, LogIsNoopWhenNotOpen) {
  rtc::ThreadCsvLogger<TestRow> logger;
  EXPECT_FALSE(logger.IsOpen());
  logger.Log(TestRow{42.0, 7});  // must not throw / crash
  EXPECT_FALSE(logger.IsOpen());
}

TEST(ThreadCsvProducer, EndToEndProducerToDrainToCsv) {
  ScopedTempDir td;
  const auto path = td.dir() / "e2e.csv";

  rtc::ThreadCsvProducer<TestRow, 8> producer;
  rtc::ThreadCsvLogger<TestRow> logger;
  ASSERT_TRUE(logger.Open(path, &WriteHeader, &WriteRow));

  for (std::uint32_t i = 0; i < 3; ++i) {
    EXPECT_TRUE(producer.Push(TestRow{0.001 * (i + 1), i + 1}));
  }
  const auto n = producer.Drain([&](const TestRow& r) { logger.Log(r); });
  EXPECT_EQ(n, 3u);

  const auto lines = ReadAllLines(path);
  ASSERT_EQ(lines.size(), 4u);
  EXPECT_EQ(lines[0], "t_relative_s,tag");
  EXPECT_EQ(lines[1], "0.001,1");
  EXPECT_EQ(lines[2], "0.002,2");
  EXPECT_EQ(lines[3], "0.003,3");
}
