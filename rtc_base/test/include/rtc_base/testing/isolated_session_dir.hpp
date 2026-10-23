// Shared test-only gtest Environment: point RTC_SESSION_DIR at a throwaway
// session for the whole test binary.
//
// A node configured with logging on resolves its session through
// rtc::ResolveSessionDir(). With no RTC_SESSION_DIR that is the workspace's
// real logging root, so a test binary would leave a timestamped session of test
// rows among the operator's runs — and a retention pass (CleanupOldSessions on
// the session's parent) would prune real sessions on the way.
//
// Layout: <mkdtemp>/logging_data/session. The session is nested one level below
// a root of its own so that retention, which runs on the session's PARENT,
// scans this private root and never the shared temp dir. "session" does not
// match the YYMMDD_HHMM pattern, so retention never deletes it either. The
// whole <mkdtemp> tree is removed at teardown.
//
// One instance per binary: a package's CMakeLists calls
// rtc_base_isolate_test_sessions() last in its BUILD_TESTING block, which
// compiles isolated_session_dir_env.cpp (calls SessionDir() from a static
// initializer) into every `test_*` executable, so a new test binary cannot
// forget it. SessionDir() is inline with a function-local static, so a test
// file that asks for the directory from its own static initializer gets the
// same instance whatever the TU order.
//
// Never installed — see testing/wait_until.hpp for why test headers live under
// test/include. Consumed via rtc_base_add_test_includes().
#ifndef RTC_BASE_TESTING_ISOLATED_SESSION_DIR_HPP_
#define RTC_BASE_TESTING_ISOLATED_SESSION_DIR_HPP_

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>

namespace rtc::testing {

class IsolatedSessionDir : public ::testing::Environment {
 public:
  void SetUp() override {
    if (const char* prev = std::getenv("RTC_SESSION_DIR")) {
      had_prev_ = true;
      prev_ = prev;
    }
    std::string tmpl = (std::filesystem::temp_directory_path() / "rtc_session_XXXXXX").string();
    ASSERT_NE(::mkdtemp(tmpl.data()), nullptr) << "mkdtemp failed for " << tmpl;
    root_ = tmpl;
    dir_ = root_ / "logging_data" / "session";
    std::error_code ec;
    std::filesystem::create_directories(dir_, ec);
    ASSERT_FALSE(ec) << "cannot create " << dir_ << ": " << ec.message();
    ::setenv("RTC_SESSION_DIR", dir_.c_str(), 1);
  }

  void TearDown() override {
    if (had_prev_) {
      ::setenv("RTC_SESSION_DIR", prev_.c_str(), 1);
    } else {
      ::unsetenv("RTC_SESSION_DIR");
    }
    std::error_code ec;
    std::filesystem::remove_all(root_, ec);
  }

  /// The session directory (RTC_SESSION_DIR while the binary runs).
  [[nodiscard]] const std::filesystem::path& Dir() const noexcept { return dir_; }

 private:
  std::filesystem::path root_;
  std::filesystem::path dir_;
  std::string prev_;
  bool had_prev_{false};
};

/// The binary's one registered instance. Its directory is set up before any
/// test runs.
inline IsolatedSessionDir& SessionDir() {
  static auto* const env =
      static_cast<IsolatedSessionDir*>(::testing::AddGlobalTestEnvironment(new IsolatedSessionDir));
  return *env;
}

}  // namespace rtc::testing

#endif  // RTC_BASE_TESTING_ISOLATED_SESSION_DIR_HPP_
