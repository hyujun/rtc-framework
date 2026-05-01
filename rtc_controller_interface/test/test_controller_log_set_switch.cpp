// Switch-while-logging contract test (C-X4t / Q-ACTIVITY-GATING).
//
// Models the rule "controllers push to ControllerLogSet only from inside
// Compute()" — which CM enforces at the dispatch level by calling Compute()
// only on the active controller. Inactive controllers' CSVs must not grow
// during the other's active window.
//
// Two ControllerLogSet instances represent two controllers (A, B) under the
// same session. A is "active" first → only A's handle is Pushed. We confirm
// B's CSV is header-only (size frozen). Then we switch: B becomes active
// (only B is Pushed), and we confirm A's CSV row count is unchanged from
// before the switch.

#include "rtc_controller_interface/controller_log_set.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <string>
#include <sys/stat.h>

namespace {

namespace fs = std::filesystem;

struct StatePod {
  double t_relative_s{0.0};
  std::int32_t tick{0};
};

void Header(std::ostream &os) { os << "t_relative_s,tick"; }
void Row(std::ostream &os, const StatePod &p) {
  os << p.t_relative_s << ',' << p.tick;
}

class ScopedSessionDir {
public:
  ScopedSessionDir() {
    if (const char *prev = std::getenv("RTC_SESSION_DIR")) {
      had_prev_ = true;
      prev_value_ = prev;
    }
    auto base = fs::temp_directory_path() / "rtc_log_set_switch";
    fs::create_directories(base);
    dir_ =
        base / ("s_" + std::to_string(reinterpret_cast<std::uintptr_t>(this) &
                                      0xFFFFFFFFu));
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
  const fs::path &dir() const noexcept { return dir_; }

private:
  fs::path dir_;
  bool had_prev_{false};
  std::string prev_value_;
};

std::uintmax_t FileSize(const fs::path &p) noexcept {
  std::error_code ec;
  return fs::file_size(p, ec);
}

std::size_t CountRows(const fs::path &p) {
  std::ifstream in(p);
  std::size_t n = 0;
  for (std::string line; std::getline(in, line);) {
    ++n;
  }
  return n; // includes header
}

} // namespace

TEST(ControllerLogSetSwitchGate, InactiveCsvFrozenWhileOtherIsActive) {
  ScopedSessionDir scope;

  rtc::ControllerLogSet set_a{"controller_a"};
  rtc::ControllerLogSet set_b{"controller_b"};

  auto handle_a = set_a.RegisterLog<StatePod>("data", &Header, &Row);
  auto handle_b = set_b.RegisterLog<StatePod>("data", &Header, &Row);
  ASSERT_TRUE(static_cast<bool>(handle_a));
  ASSERT_TRUE(static_cast<bool>(handle_b));

  const auto path_a = set_a.Channels()[0].second;
  const auto path_b = set_b.Channels()[0].second;

  // Both files should exist and contain only the header at this point.
  ASSERT_TRUE(fs::exists(path_a));
  ASSERT_TRUE(fs::exists(path_b));
  EXPECT_EQ(CountRows(path_a), 1u); // header
  EXPECT_EQ(CountRows(path_b), 1u);

  // ── Phase 1: A active, B inactive ─────────────────────────────────────
  for (std::int32_t i = 1; i <= 50; ++i) {
    EXPECT_TRUE(handle_a.Push(StatePod{0.001 * i, i}));
    // handle_b NOT pushed (gating rule: only active controller pushes).
  }
  set_a.DrainAll();
  set_b.DrainAll(); // empty drain — should not write anything

  const auto rows_a_after_p1 = CountRows(path_a);
  const auto rows_b_after_p1 = CountRows(path_b);
  EXPECT_EQ(rows_a_after_p1, 1u + 50u);
  EXPECT_EQ(rows_b_after_p1, 1u) << "B's CSV grew while inactive";

  const auto size_b_frozen = FileSize(path_b);

  // ── Phase 2: switch — B active, A inactive ────────────────────────────
  for (std::int32_t i = 1; i <= 30; ++i) {
    EXPECT_TRUE(handle_b.Push(StatePod{1.0 + 0.001 * i, i}));
  }
  set_a.DrainAll(); // empty drain
  set_b.DrainAll();

  const auto rows_a_after_p2 = CountRows(path_a);
  const auto rows_b_after_p2 = CountRows(path_b);
  EXPECT_EQ(rows_a_after_p2, rows_a_after_p1)
      << "A's CSV grew during B's active window";
  EXPECT_EQ(rows_b_after_p2, 1u + 30u);

  // B's file size must equal the post-Phase-2 size only after Phase-2
  // pushes; verify that pre-Phase-2 frozen size is strictly smaller (the
  // row growth happens in Phase 2 after the snapshot).
  EXPECT_GT(FileSize(path_b), size_b_frozen);
}

TEST(ControllerLogSetSwitchGate, TimestampMonotonicAcrossSwitch) {
  ScopedSessionDir scope;

  rtc::ControllerLogSet set_a{"controller_a"};
  rtc::ControllerLogSet set_b{"controller_b"};

  auto handle_a = set_a.RegisterLog<StatePod>("data", &Header, &Row);
  auto handle_b = set_b.RegisterLog<StatePod>("data", &Header, &Row);

  // Simulate session-wide t_relative_s (CM provides this — never resets
  // on controller switch, so timestamps must be strictly increasing
  // across A→B handoff).
  double t = 0.0;
  for (int i = 0; i < 10; ++i) {
    t += 0.002;
    EXPECT_TRUE(handle_a.Push(StatePod{t, i}));
  }
  // CM "switches" → only B's Compute is called now; t continues from
  // its monotonic origin.
  for (int i = 0; i < 10; ++i) {
    t += 0.002;
    EXPECT_TRUE(handle_b.Push(StatePod{t, i}));
  }
  set_a.DrainAll();
  set_b.DrainAll();

  // Read A's last row and B's first row, parse t_relative_s, assert
  // A_last < B_first (no clock reset on switch).
  auto last_t = [](const fs::path &p) {
    std::ifstream in(p);
    double last = -1.0;
    for (std::string line; std::getline(in, line);) {
      const auto comma = line.find(',');
      if (comma != std::string::npos &&
          line.find_first_of("0123456789") < std::string::npos) {
        try {
          last = std::stod(line.substr(0, comma));
        } catch (...) {
          // header line, skip
        }
      }
    }
    return last;
  };
  auto first_t_after_header = [](const fs::path &p) {
    std::ifstream in(p);
    std::string line;
    std::getline(in, line); // header
    if (!std::getline(in, line))
      return -1.0;
    const auto comma = line.find(',');
    return std::stod(line.substr(0, comma));
  };

  const double a_last = last_t(set_a.Channels()[0].second);
  const double b_first = first_t_after_header(set_b.Channels()[0].second);
  EXPECT_LT(a_last, b_first)
      << "Timestamp not monotonic across switch: a_last=" << a_last
      << " b_first=" << b_first;
}
