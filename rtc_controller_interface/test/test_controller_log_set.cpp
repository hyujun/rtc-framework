// Unit tests for ControllerLogSet — opt-in helper for controller-owned
// CSV data logs.
//
// Verifies:
//   - RegisterLog<PodT> returns a bound handle when path is writable.
//   - Push() inside Compute() goes to SPSC; DrainAll() writes CSV rows.
//   - File path = <session>/controllers/<config_key>/<instance>.csv.
//   - Multiple PodT types coexist via type erasure.
//   - Drop accounting per channel.
//
// RTC_SESSION_DIR is redirected to a per-test tempdir.

#include "rtc_controller_interface/controller_log_set.hpp"

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

struct PodA {
  double t_relative_s{0.0};
  std::int32_t a{0};
};

struct PodB {
  double t_relative_s{0.0};
  double x{0.0};
  double y{0.0};
};

void HeaderA(std::ostream& os) {
  os << "t_relative_s,a";
}

void RowA(std::ostream& os, const PodA& p) {
  os << p.t_relative_s << ',' << p.a;
}

void HeaderB(std::ostream& os) {
  os << "t_relative_s,x,y";
}

void RowB(std::ostream& os, const PodB& p) {
  os << p.t_relative_s << ',' << p.x << ',' << p.y;
}

class ScopedSessionDir {
 public:
  ScopedSessionDir() {
    if (const char* prev = std::getenv("RTC_SESSION_DIR")) {
      had_prev_ = true;
      prev_value_ = prev;
    }
    auto base = fs::temp_directory_path() / "rtc_log_set_test";
    fs::create_directories(base);
    dir_ = base / ("s_" + std::to_string(reinterpret_cast<std::uintptr_t>(this) & 0xFFFFFFFFu));
    fs::create_directories(dir_);
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

  const fs::path& dir() const noexcept { return dir_; }

 private:
  fs::path dir_;
  bool had_prev_{false};
  std::string prev_value_;
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

TEST(ControllerLogSet, EmptyByDefault) {
  rtc::ControllerLogSet set;
  EXPECT_TRUE(set.empty());
  EXPECT_EQ(set.size(), 0u);
  EXPECT_EQ(set.DrainAll(), 0u);
}

TEST(ControllerLogSet, RegisterLogResolvesPathUnderConfigKey) {
  ScopedSessionDir scope;
  rtc::ControllerLogSet set{"my_controller"};
  auto handle = set.RegisterLog<PodA>("ur5e", &HeaderA, &RowA);
  ASSERT_TRUE(static_cast<bool>(handle));
  ASSERT_EQ(set.size(), 1u);

  const auto channels = set.Channels();
  ASSERT_EQ(channels.size(), 1u);
  const auto expected = scope.dir() / "controllers" / "my_controller" / "ur5e.csv";
  EXPECT_EQ(channels[0].second, expected);
  EXPECT_TRUE(fs::exists(expected));
}

TEST(ControllerLogSet, FirstOpenWritesHeader) {
  ScopedSessionDir scope;
  rtc::ControllerLogSet set{"c"};
  auto handle = set.RegisterLog<PodA>("inst", &HeaderA, &RowA);
  ASSERT_TRUE(static_cast<bool>(handle));

  const auto channels = set.Channels();
  const auto lines = ReadAllLines(channels[0].second);
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "t_relative_s,a");
}

TEST(ControllerLogSet, PushDrainProducesRows) {
  ScopedSessionDir scope;
  rtc::ControllerLogSet set{"c"};
  auto handle = set.RegisterLog<PodA>("data", &HeaderA, &RowA);
  ASSERT_TRUE(static_cast<bool>(handle));

  EXPECT_TRUE(handle.Push(PodA{0.001, 1}));
  EXPECT_TRUE(handle.Push(PodA{0.002, 2}));
  EXPECT_TRUE(handle.Push(PodA{0.003, 3}));

  const auto n = set.DrainAll();
  EXPECT_EQ(n, 3u);

  const auto channels = set.Channels();
  const auto lines = ReadAllLines(channels[0].second);
  ASSERT_EQ(lines.size(), 4u);  // header + 3 rows
  EXPECT_EQ(lines[0], "t_relative_s,a");
  EXPECT_EQ(lines[1], "0.001,1");
  EXPECT_EQ(lines[2], "0.002,2");
  EXPECT_EQ(lines[3], "0.003,3");
}

TEST(ControllerLogSet, MultiplePodTypesCoexist) {
  ScopedSessionDir scope;
  rtc::ControllerLogSet set{"mixed"};
  auto handle_a = set.RegisterLog<PodA>("a", &HeaderA, &RowA);
  auto handle_b = set.RegisterLog<PodB>("b", &HeaderB, &RowB);
  ASSERT_TRUE(static_cast<bool>(handle_a));
  ASSERT_TRUE(static_cast<bool>(handle_b));
  EXPECT_EQ(set.size(), 2u);

  EXPECT_TRUE(handle_a.Push(PodA{0.1, 7}));
  EXPECT_TRUE(handle_b.Push(PodB{0.2, 1.5, 2.5}));
  EXPECT_TRUE(handle_a.Push(PodA{0.3, 8}));

  const auto n = set.DrainAll();
  EXPECT_EQ(n, 3u);

  const auto channels = set.Channels();
  ASSERT_EQ(channels.size(), 2u);
  // Order matches registration order.
  EXPECT_EQ(channels[0].first, "a");
  EXPECT_EQ(channels[1].first, "b");

  const auto lines_a = ReadAllLines(channels[0].second);
  const auto lines_b = ReadAllLines(channels[1].second);
  ASSERT_EQ(lines_a.size(), 3u);  // header + 2 rows
  ASSERT_EQ(lines_b.size(), 2u);  // header + 1 row
  EXPECT_EQ(lines_b[1], "0.2,1.5,2.5");
}

TEST(ControllerLogSet, UnboundHandlePushIsNoOp) {
  rtc::LogHandle<PodA> empty;
  EXPECT_FALSE(static_cast<bool>(empty));
  EXPECT_FALSE(empty.Push(PodA{0.0, 0}));  // must not crash
}

TEST(ControllerLogSet, ReopenAppendDoesNotDuplicateHeader) {
  ScopedSessionDir scope;
  fs::path log_path;
  {
    rtc::ControllerLogSet set{"c"};
    auto handle = set.RegisterLog<PodA>("inst", &HeaderA, &RowA);
    ASSERT_TRUE(static_cast<bool>(handle));
    EXPECT_TRUE(handle.Push(PodA{1.0, 1}));
    set.DrainAll();
    log_path = set.Channels()[0].second;
  }
  {
    rtc::ControllerLogSet set{"c"};
    auto handle = set.RegisterLog<PodA>("inst", &HeaderA, &RowA);
    ASSERT_TRUE(static_cast<bool>(handle));
    EXPECT_TRUE(handle.Push(PodA{2.0, 2}));
    set.DrainAll();
  }
  const auto lines = ReadAllLines(log_path);
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[0], "t_relative_s,a");
  EXPECT_EQ(lines[1], "1,1");
  EXPECT_EQ(lines[2], "2,2");
}

TEST(ControllerLogSet, DropCountsTrackPerChannelOverflow) {
  ScopedSessionDir scope;
  // Capacity 4 → effective ring holds at most 3.
  rtc::ControllerLogSet set{"c"};
  auto handle = set.RegisterLog<PodA, /*Capacity=*/4>("small", &HeaderA, &RowA);
  ASSERT_TRUE(static_cast<bool>(handle));
  for (int i = 0; i < 5; ++i) {
    handle.Push(PodA{0.0, i});  // 5th drops
  }
  const auto drops = set.DropCounts();
  ASSERT_EQ(drops.size(), 1u);
  EXPECT_GT(drops[0], 0u);
}
