// MpcSolveTimingLogger unit tests — verifies path resolution, header
// emission, append-on-reopen, per-sample Log() column count, and no-op
// behaviour when Open() failed.
//
// Schema is the per-tick raw-sample format: `t_wall_ns,tick_count,solve_ns`
// (one CSV row per MPC solve). The 9-column aggregate schema used
// previously has been replaced — readers compute percentiles in post over
// the raw stream.
//
// `RTC_SESSION_DIR` env var is used to redirect the resolver chain to a
// per-test tempdir; ResolveSessionDir() honours that env first.

#include "rtc_mpc/logging/mpc_solve_timing_logger.hpp"

#include "rtc_base/timing/mpc_solve_sample.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

class ScopedSessionDir {
public:
  ScopedSessionDir() {
    const char *prev = std::getenv("RTC_SESSION_DIR");
    if (prev) {
      had_prev_ = true;
      prev_value_ = prev;
    }
    auto base = fs::temp_directory_path() / "rtc_mpc_logger_test";
    fs::create_directories(base);
    dir_ =
        fs::path(base) /
        ("session_" +
         std::to_string(::testing::UnitTest::GetInstance()
                            ->current_test_info()
                            ->result()
                            ->test_property_count()) +
         "_" +
         std::to_string(reinterpret_cast<std::uintptr_t>(this) & 0xFFFFFFFFu));
    fs::create_directories(dir_);
    std::error_code canon_ec;
    auto canon = fs::canonical(dir_, canon_ec);
    if (!canon_ec) {
      dir_ = canon;
    }
    ::setenv("RTC_SESSION_DIR", dir_.c_str(), 1);
  }

  ~ScopedSessionDir() {
    if (had_prev_) {
      ::setenv("RTC_SESSION_DIR", prev_value_.c_str(), 1);
    } else {
      ::unsetenv("RTC_SESSION_DIR");
    }
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }

  const fs::path &dir() const noexcept { return dir_; }

private:
  fs::path dir_;
  bool had_prev_{false};
  std::string prev_value_;
};

std::vector<std::string> ReadAllLines(const fs::path &p) {
  std::vector<std::string> lines;
  std::ifstream in(p);
  for (std::string line; std::getline(in, line);) {
    lines.push_back(line);
  }
  return lines;
}

int CountCommas(const std::string &s) {
  int n = 0;
  for (char c : s) {
    if (c == ',')
      ++n;
  }
  return n;
}

} // namespace

TEST(MpcSolveTimingLogger, OpenResolvesPathUnderControllerSubdir) {
  ScopedSessionDir scope;
  rtc::mpc::MpcSolveTimingLogger logger;

  ASSERT_TRUE(logger.Open("foo_controller"));
  EXPECT_TRUE(logger.IsOpen());

  const auto &p = logger.Path();
  const auto expected =
      scope.dir() / "controllers" / "foo_controller" / "mpc_solve_timing.csv";
  EXPECT_EQ(p, expected) << "logger path: " << p;
  EXPECT_TRUE(fs::exists(p));
}

TEST(MpcSolveTimingLogger, FirstOpenWritesHeader) {
  ScopedSessionDir scope;
  rtc::mpc::MpcSolveTimingLogger logger;

  ASSERT_TRUE(logger.Open("hdr_test"));
  const auto lines = ReadAllLines(logger.Path());
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "t_wall_ns,tick_count,solve_ns");
}

TEST(MpcSolveTimingLogger, ReopenAppendDoesNotDuplicateHeader) {
  ScopedSessionDir scope;
  {
    rtc::mpc::MpcSolveTimingLogger logger;
    ASSERT_TRUE(logger.Open("dup_test"));
    rtc::MpcSolveSample sample{};
    sample.t_wall_ns = 1000;
    sample.tick_count = 1;
    sample.payload.solve_ns = 100;
    logger.Log(sample);
  }
  {
    rtc::mpc::MpcSolveTimingLogger logger;
    ASSERT_TRUE(logger.Open("dup_test"));
    rtc::MpcSolveSample sample{};
    sample.t_wall_ns = 2000;
    sample.tick_count = 2;
    sample.payload.solve_ns = 200;
    logger.Log(sample);
  }
  rtc::mpc::MpcSolveTimingLogger probe;
  ASSERT_TRUE(probe.Open("dup_test"));
  const auto lines = ReadAllLines(probe.Path());
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_NE(lines[0].find("t_wall_ns"), std::string::npos);
  EXPECT_EQ(lines[1].find("t_wall_ns"), std::string::npos);
  EXPECT_EQ(lines[2].find("t_wall_ns"), std::string::npos);
}

TEST(MpcSolveTimingLogger, LogWritesThreeCommaSeparatedColumns) {
  ScopedSessionDir scope;
  rtc::mpc::MpcSolveTimingLogger logger;
  ASSERT_TRUE(logger.Open("col_test"));

  rtc::MpcSolveSample sample{};
  sample.t_wall_ns = 1234567890;
  sample.tick_count = 42;
  sample.payload.solve_ns = 12345;
  logger.Log(sample);

  const auto lines = ReadAllLines(logger.Path());
  ASSERT_EQ(lines.size(), 2u); // header + 1 row
  // 3 columns → 2 commas
  EXPECT_EQ(CountCommas(lines[1]), 2);
  EXPECT_EQ(lines[1], "1234567890,42,12345") << "row was: " << lines[1];
}

TEST(MpcSolveTimingLogger, LogIsNoopWhenNotOpen) {
  rtc::mpc::MpcSolveTimingLogger logger;
  ASSERT_FALSE(logger.IsOpen());
  rtc::MpcSolveSample sample{};
  sample.tick_count = 1;
  // Must not throw / crash even though Open() was never called.
  logger.Log(sample);
  EXPECT_FALSE(logger.IsOpen());
}
