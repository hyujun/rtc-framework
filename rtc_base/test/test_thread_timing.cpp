// Unit tests for the per-tick thread-timing infrastructure:
//   ThreadTimingProducer<P, N>  (rtc_base/timing/thread_timing_producer.hpp)
//   ThreadTimingCsvLogger<P>    (rtc_base/timing/thread_timing_csv_logger.hpp)
//
// Producer-side: SPSC push, monotonic tick_count, drop accounting.
// Logger-side:   header emission, payload columns, append-on-reopen,
//                run-boundary stamping (#376), no-op when not open,
//                end-to-end producer→drain→csv.

#include "rtc_base/timing/thread_timing_csv_logger.hpp"
#include "rtc_base/timing/thread_timing_producer.hpp"
#include "rtc_base/timing/thread_timing_sample.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

struct TestPayload {
  double a{0.0};
  std::uint32_t b{0};
};

void WriteHeader(std::ostream& os) {
  os << ",a,b";
}

void WriteRow(std::ostream& os, const TestPayload& p) {
  os << ',' << p.a << ',' << p.b;
}

class ScopedTempDir {
 public:
  ScopedTempDir() {
    auto base = fs::temp_directory_path() / "rtc_thread_timing_test";
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

// Pins RTC_RUN_ID so the run_id column is predictable. With the variable
// unset ResolveRunId() falls back to getpid() (rtc_base/logging/run_id.hpp),
// which no exact-string row assertion can spell.
class ScopedRunId {
 public:
  explicit ScopedRunId(const char* value) {
    if (const char* prev = std::getenv("RTC_RUN_ID")) {
      prev_ = prev;
      had_prev_ = true;
    }
    ::setenv("RTC_RUN_ID", value, 1);
  }

  ~ScopedRunId() {
    if (had_prev_) {
      ::setenv("RTC_RUN_ID", prev_.c_str(), 1);
    } else {
      ::unsetenv("RTC_RUN_ID");
    }
  }

  ScopedRunId(const ScopedRunId&) = delete;
  ScopedRunId& operator=(const ScopedRunId&) = delete;

 private:
  std::string prev_;
  bool had_prev_{false};
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

// ── Producer tests ──────────────────────────────────────────────────────────

TEST(ThreadTimingProducer, PushIncrementsTickAndDrainPreservesOrder) {
  rtc::ThreadTimingProducer<TestPayload, 8> producer;

  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(producer.Push({static_cast<double>(i), static_cast<uint32_t>(i * 10)}));
  }
  EXPECT_EQ(producer.LastTickCount(), 5u);
  EXPECT_EQ(producer.DropCount(), 0u);

  std::vector<rtc::ThreadTimingSample<TestPayload>> drained;
  const auto n = producer.Drain([&](const auto& s) { drained.push_back(s); });
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
  EXPECT_FALSE(producer.Push({}));  // full
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
  EXPECT_EQ(lines[0], "t_wall_ns,tick_count,run_id,a,b");
}

TEST(ThreadTimingCsvLogger, EmptyPayloadHeaderHasOnlyTimingColumns) {
  ScopedTempDir scope;
  const auto path = scope.dir() / "t.csv";
  rtc::ThreadTimingCsvLogger<TestPayload> logger;

  // Pass a no-op header writer to mean "no payload columns".
  ASSERT_TRUE(logger.Open(path, [](std::ostream&) {}, [](std::ostream&, const TestPayload&) {}));
  const auto lines = ReadAllLines(path);
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "t_wall_ns,tick_count,run_id");
}

TEST(ThreadTimingCsvLogger, ReopenAppendsWithoutDuplicateHeader) {
  ScopedTempDir scope;
  // Same run id across both opens: this test owns the "append does not repeat
  // the header" contract, and ReopenWithNewRunIdSeparatesRunsInOneFile below
  // owns the boundary. Keeping them apart is why this one can stay exact.
  const ScopedRunId run{"4242"};
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
  EXPECT_EQ(lines[0], "t_wall_ns,tick_count,run_id,a,b");
  EXPECT_EQ(lines[1], "1000,1,4242,1.5,7");
  EXPECT_EQ(lines[2], "2000,2,4242,2.5,14");
}

// #376 — 같은 분 안의 두 기동은 같은 세션 디렉토리를 얻고, append 로 열리는
// 로거는 두 런의 행을 한 파일에 쌓는다. 그 자체는 의도된 설계이고 (재시작 세션
// 이어쓰기), 결함은 **경계가 파일에 기록되지 않는 것**이었다. run_id 열이 그
// 경계다: 헤더는 여전히 한 번만 나오고 행만 런별로 갈린다.
TEST(ThreadTimingCsvLogger, ReopenWithNewRunIdSeparatesRunsInOneFile) {
  ScopedTempDir scope;
  const auto path = scope.dir() / "t.csv";
  {
    const ScopedRunId run{"11"};
    rtc::ThreadTimingCsvLogger<TestPayload> logger;
    ASSERT_TRUE(logger.Open(path, &WriteHeader, &WriteRow));
    rtc::ThreadTimingSample<TestPayload> s{};
    s.t_wall_ns = 1000;
    s.tick_count = 5978;  // 앞 런의 잔여 (#376 실기 관측값)
    s.payload = {1.5, 7};
    logger.Log(s);
  }
  {
    const ScopedRunId run{"22"};
    rtc::ThreadTimingCsvLogger<TestPayload> logger;
    ASSERT_TRUE(logger.Open(path, &WriteHeader, &WriteRow));
    rtc::ThreadTimingSample<TestPayload> s{};
    s.t_wall_ns = 2000;
    s.tick_count = 1;  // 새 런은 1 부터 — 이 비단조가 이전의 유일한 단서였다
    s.payload = {2.5, 14};
    logger.Log(s);
  }

  const auto lines = ReadAllLines(path);
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[0], "t_wall_ns,tick_count,run_id,a,b");
  EXPECT_EQ(lines[1], "1000,5978,11,1.5,7");
  EXPECT_EQ(lines[2], "2000,1,22,2.5,14");
}

// 반대 방향: 한 launch 안의 lifecycle 재configure 는 env 가 그대로이므로 같은
// run_id 로 이어진다. 재Open 을 무조건 새 런으로 세면 정상 재configure 가 가짜
// 경계를 만들어 낸다.
TEST(ThreadTimingCsvLogger, ReopenWithSameRunIdStaysOneRun) {
  ScopedTempDir scope;
  const ScopedRunId run{"77"};
  const auto path = scope.dir() / "t.csv";
  for (std::uint64_t tick : {std::uint64_t{1}, std::uint64_t{2}}) {
    rtc::ThreadTimingCsvLogger<TestPayload> logger;
    ASSERT_TRUE(logger.Open(path, &WriteHeader, &WriteRow));
    rtc::ThreadTimingSample<TestPayload> s{};
    s.t_wall_ns = tick * 1000;
    s.tick_count = tick;
    logger.Log(s);
  }

  const auto lines = ReadAllLines(path);
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[1], "1000,1,77,0,0");
  EXPECT_EQ(lines[2], "2000,2,77,0,0");
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
  const ScopedRunId run{"8899"};
  const auto path = scope.dir() / "t.csv";
  rtc::ThreadTimingProducer<TestPayload, 16> producer;
  rtc::ThreadTimingCsvLogger<TestPayload> logger;
  ASSERT_TRUE(logger.Open(path, &WriteHeader, &WriteRow));

  // Use integer-valued doubles so the default ostream format matches a
  // simple substring (no locale / precision concerns).
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(
        producer.Push({static_cast<double>((i + 1) * 10), static_cast<uint32_t>((i + 1) * 100)}));
  }
  const auto n = producer.Drain([&](const auto& s) { logger.Log(s); });
  EXPECT_EQ(n, 4u);

  const auto lines = ReadAllLines(path);
  ASSERT_EQ(lines.size(), 5u);  // header + 4 rows
  EXPECT_EQ(lines[0], "t_wall_ns,tick_count,run_id,a,b");
  for (int i = 0; i < 4; ++i) {
    const auto& row = lines[static_cast<std::size_t>(i + 1)];
    // Match a suffix of the form ",<tick>,<run_id>,<a>,<b>". Default
    // std::ostream formats 10.0 as "10".
    const std::string expect_suffix = "," + std::to_string(i + 1) + ",8899," +
                                      std::to_string((i + 1) * 10) + "," +
                                      std::to_string((i + 1) * 100);
    EXPECT_NE(row.find(expect_suffix), std::string::npos)
        << "row=" << row << " expected suffix " << expect_suffix;
  }
}
