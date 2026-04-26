// MpcSolveTimingLogger unit tests — verifies path resolution, header
// emission, append-mode (no duplicate header), Log() column count, and
// no-op behaviour when Open() failed.
//
// `RTC_SESSION_DIR` env var is used to redirect the resolver chain to a
// per-test tempdir; ResolveSessionDir() honours that env first.

#include "rtc_mpc/logging/mpc_solve_timing_logger.hpp"

#include "rtc_base/timing/mpc_solve_stats.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

// RAII helper: redirect ResolveSessionDir() to a fresh tempdir for the
// duration of one test, restoring any prior RTC_SESSION_DIR value.
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
    // Make a unique subdir per construction so parallel tests don't collide.
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
    // Canonicalize *after* creation so symlinks in /tmp resolve consistently
    // (without this, std::tmp may be /private/tmp on macOS or a /tmp symlink).
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
  // No Log() yet — file should contain just the header row.
  const auto lines = ReadAllLines(logger.Path());
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0],
            "t_wall_ns,count,window,last_ns,min_ns,p50_ns,p99_ns,max_ns,"
            "mean_ns");
}

TEST(MpcSolveTimingLogger, ReopenAppendDoesNotDuplicateHeader) {
  ScopedSessionDir scope;
  // First session: open + log a row, then drop logger.
  {
    rtc::mpc::MpcSolveTimingLogger logger;
    ASSERT_TRUE(logger.Open("dup_test"));
    rtc::MpcSolveStats s;
    s.count = 1;
    s.window = 1;
    s.last_ns = 100;
    s.min_ns = 100;
    s.p50_ns = 100;
    s.p99_ns = 100;
    s.max_ns = 100;
    s.mean_ns = 100.0;
    logger.Log(s);
  }
  // Second session: re-open same controller_key → append, no second header.
  {
    rtc::mpc::MpcSolveTimingLogger logger;
    ASSERT_TRUE(logger.Open("dup_test"));
    rtc::MpcSolveStats s;
    s.count = 2;
    s.window = 2;
    s.last_ns = 200;
    s.min_ns = 100;
    s.p50_ns = 150;
    s.p99_ns = 200;
    s.max_ns = 200;
    s.mean_ns = 150.0;
    logger.Log(s);
  }
  // Expect: header (1) + 2 data rows = 3 lines, exactly one header.
  rtc::mpc::MpcSolveTimingLogger probe;
  ASSERT_TRUE(probe.Open("dup_test"));
  const auto lines = ReadAllLines(probe.Path());
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_NE(lines[0].find("t_wall_ns"), std::string::npos);
  EXPECT_EQ(lines[1].find("t_wall_ns"), std::string::npos);
  EXPECT_EQ(lines[2].find("t_wall_ns"), std::string::npos);
}

TEST(MpcSolveTimingLogger, LogWritesNineCommaSeparatedColumns) {
  ScopedSessionDir scope;
  rtc::mpc::MpcSolveTimingLogger logger;
  ASSERT_TRUE(logger.Open("col_test"));

  rtc::MpcSolveStats s;
  s.count = 42;
  s.window = 32;
  s.last_ns = 12345;
  s.min_ns = 100;
  s.p50_ns = 5000;
  s.p99_ns = 11000;
  s.max_ns = 12000;
  s.mean_ns = 4321.0;
  logger.Log(s);

  const auto lines = ReadAllLines(logger.Path());
  ASSERT_EQ(lines.size(), 2u); // header + 1 row
  // 9 columns → 8 commas
  EXPECT_EQ(CountCommas(lines[1]), 8);
  // Spot-check a couple of fields are present in the expected order.
  EXPECT_NE(lines[1].find(",42,32,12345,100,5000,11000,12000,4321"),
            std::string::npos)
      << "row was: " << lines[1];
}

TEST(MpcSolveTimingLogger, LogIsNoopWhenNotOpen) {
  rtc::mpc::MpcSolveTimingLogger logger;
  ASSERT_FALSE(logger.IsOpen());
  rtc::MpcSolveStats s;
  s.count = 1;
  // Must not throw / crash even though Open() was never called.
  logger.Log(s);
  EXPECT_FALSE(logger.IsOpen());
}
