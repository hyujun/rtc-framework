// MpcTimingLogger unit tests — verifies path resolution, header emission,
// append-on-reopen, per-sample Log() column count, and no-op behaviour
// when Open() failed.
//
// Schema is the unified per-tick format shared with the CM RT loop:
//   t_wall_ns,tick_count,t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us
// One CSV row per MPC main-loop iteration.
//
// `RTC_SESSION_DIR` env var is used to redirect the resolver chain to a
// per-test tempdir; ResolveSessionDir() honours that env first.

#include "rtc_mpc/logging/mpc_timing_logger.hpp"

#include "rtc_base/timing/rt_tick_timing_sample.hpp"

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

TEST(MpcTimingLogger, OpenResolvesPathUnderTimingDir) {
  ScopedSessionDir scope;
  rtc::mpc::MpcTimingLogger logger;

  ASSERT_TRUE(logger.Open());
  EXPECT_TRUE(logger.IsOpen());

  const auto &p = logger.Path();
  const auto expected = scope.dir() / "timing" / "mpc_timing_log.csv";
  EXPECT_EQ(p, expected) << "logger path: " << p;
  EXPECT_TRUE(fs::exists(p));
}

TEST(MpcTimingLogger, FirstOpenWritesHeader) {
  ScopedSessionDir scope;
  rtc::mpc::MpcTimingLogger logger;

  ASSERT_TRUE(logger.Open());
  const auto lines = ReadAllLines(logger.Path());
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0],
            "t_wall_ns,tick_count,t_state_us,t_compute_us,t_publish_us,"
            "t_total_us,jitter_us");
}

TEST(MpcTimingLogger, ReopenAppendDoesNotDuplicateHeader) {
  ScopedSessionDir scope;
  {
    rtc::mpc::MpcTimingLogger logger;
    ASSERT_TRUE(logger.Open());
    rtc::RtTickTimingSample sample{};
    sample.t_wall_ns = 1000;
    sample.tick_count = 1;
    sample.payload.t_compute_us = 1.0;
    logger.Log(sample);
  }
  {
    rtc::mpc::MpcTimingLogger logger;
    ASSERT_TRUE(logger.Open());
    rtc::RtTickTimingSample sample{};
    sample.t_wall_ns = 2000;
    sample.tick_count = 2;
    sample.payload.t_compute_us = 2.0;
    logger.Log(sample);
  }
  rtc::mpc::MpcTimingLogger probe;
  ASSERT_TRUE(probe.Open());
  const auto lines = ReadAllLines(probe.Path());
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_NE(lines[0].find("t_wall_ns"), std::string::npos);
  EXPECT_EQ(lines[1].find("t_wall_ns"), std::string::npos);
  EXPECT_EQ(lines[2].find("t_wall_ns"), std::string::npos);
}

TEST(MpcTimingLogger, LogWritesSevenCommaSeparatedColumns) {
  ScopedSessionDir scope;
  rtc::mpc::MpcTimingLogger logger;
  ASSERT_TRUE(logger.Open());

  rtc::RtTickTimingSample sample{};
  sample.t_wall_ns = 1234567890;
  sample.tick_count = 42;
  sample.payload.t_state_us = 1.0;
  sample.payload.t_compute_us = 2.0;
  sample.payload.t_publish_us = 3.0;
  sample.payload.t_total_us = 6.0;
  sample.payload.jitter_us = 0.5;
  logger.Log(sample);

  const auto lines = ReadAllLines(logger.Path());
  ASSERT_EQ(lines.size(), 2u); // header + 1 row
  // 7 columns → 6 commas
  EXPECT_EQ(CountCommas(lines[1]), 6);
  EXPECT_EQ(lines[1], "1234567890,42,1,2,3,6,0.5") << "row was: " << lines[1];
}

TEST(MpcTimingLogger, LogIsNoopWhenNotOpen) {
  rtc::mpc::MpcTimingLogger logger;
  ASSERT_FALSE(logger.IsOpen());
  rtc::RtTickTimingSample sample{};
  sample.tick_count = 1;
  // Must not throw / crash even though Open() was never called.
  logger.Log(sample);
  EXPECT_FALSE(logger.IsOpen());
}
