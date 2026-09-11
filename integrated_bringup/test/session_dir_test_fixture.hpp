// ── Point controller CSV logs at a throwaway session directory ───────────────
//
// A controller configured from a shipped YAML that carries a `logs:` block
// opens its CSV files during on_configure. With no RTC_SESSION_DIR set, the
// session resolver falls back to the workspace's real logging root, so a test
// binary would leave a timestamped session of test rows among the operator's
// runs — and prune real sessions past the retention count on the way.
//
// Registered once per binary as a gtest Environment: every test in it logs
// into one fresh temp directory, removed at teardown. `Dir()` is for the test
// that wants to read a file back.
#pragma once

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>

namespace integrated_bringup::testfx {

class IsolatedSessionDir : public ::testing::Environment {
 public:
  void SetUp() override {
    if (const char* prev = std::getenv("RTC_SESSION_DIR")) {
      had_prev_ = true;
      prev_ = prev;
    }
    std::string tmpl = (std::filesystem::temp_directory_path() / "rtc_session_XXXXXX").string();
    ASSERT_NE(::mkdtemp(tmpl.data()), nullptr) << "mkdtemp failed for " << tmpl;
    dir_ = tmpl;
    ::setenv("RTC_SESSION_DIR", dir_.c_str(), 1);
  }

  void TearDown() override {
    if (had_prev_) {
      ::setenv("RTC_SESSION_DIR", prev_.c_str(), 1);
    } else {
      ::unsetenv("RTC_SESSION_DIR");
    }
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  [[nodiscard]] const std::filesystem::path& Dir() const noexcept { return dir_; }

 private:
  std::filesystem::path dir_;
  std::string prev_;
  bool had_prev_{false};
};

}  // namespace integrated_bringup::testfx
