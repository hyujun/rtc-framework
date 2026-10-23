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
// A test that needs a session of its own — an empty directory to read one
// controller's CSVs back from, or a fresh session per case — takes a
// ScopedSessionDir below instead of setting RTC_SESSION_DIR by hand (#935): it
// lives under the same private root, so it needs the same registration.
//
// Never installed — see testing/wait_until.hpp for why test headers live under
// test/include. Consumed via rtc_base_add_test_includes().
#ifndef RTC_BASE_TESTING_ISOLATED_SESSION_DIR_HPP_
#define RTC_BASE_TESTING_ISOLATED_SESSION_DIR_HPP_

#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
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
    tmp_root_ = tmpl;
    dir_ = tmp_root_ / "logging_data" / "session";
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
    std::filesystem::remove_all(tmp_root_, ec);
  }

  /// The session directory (RTC_SESSION_DIR while the binary runs).
  [[nodiscard]] const std::filesystem::path& Dir() const noexcept { return dir_; }

  /// The private logging root Dir() sits in — the parent a retention pass
  /// scans. Empty until SetUp() has run.
  [[nodiscard]] std::filesystem::path Root() const { return dir_.parent_path(); }

 private:
  std::filesystem::path tmp_root_;
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

/// RTC_SESSION_DIR pointed at a fresh, empty session of its own for the
/// object's lifetime: `<Root()>/<tag>_<n>`, n counting the binary's scopes.
/// The previous value comes back on destruction, so scopes nest (LIFO), and
/// the directory is removed with everything written into it.
///
/// It sits next to the binary's own session, under the private root, so a
/// retention pass that a configure runs on its parent can only ever see this
/// binary's directories — and none of their names matches YYMMDD_HHMM, which
/// is why a tag of six digits is refused. A tag with a '/' is refused too: it
/// would place the directory, and the remove_all() on it, outside that root.
///
/// Requires the binary's IsolatedSessionDir (rtc_base_isolate_test_sessions()).
/// Without it the constructor throws std::logic_error, which gtest reports as
/// a failed test — the destructor does not run. That call also registers the
/// Environment too late to set anything up; its TearDown() still runs at exit
/// and unsets RTC_SESSION_DIR once. Acceptable on a path that already failed.
class ScopedSessionDir {
 public:
  explicit ScopedSessionDir(std::string_view tag = "s") {
    const std::filesystem::path root = SessionDir().Root();
    if (root.empty()) {
      throw std::logic_error(
          "ScopedSessionDir needs the binary's IsolatedSessionDir — call "
          "rtc_base_isolate_test_sessions() last in the package's BUILD_TESTING block");
    }
    if (tag.find('/') != std::string_view::npos) {
      throw std::logic_error("ScopedSessionDir tag '" + std::string(tag) +
                             "' would put the directory outside the private root");
    }
    if (tag.size() == 6 && tag.find_first_not_of("0123456789") == std::string_view::npos) {
      throw std::logic_error("ScopedSessionDir tag '" + std::string(tag) +
                             "' would let the directory match retention's YYMMDD_HHMM");
    }
    static std::atomic<unsigned> next{0};
    dir_ = root / (std::string(tag) + "_" + std::to_string(next.fetch_add(1)));
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
    std::filesystem::create_directories(dir_, ec);
    if (ec) {
      throw std::runtime_error("cannot create " + dir_.string() + ": " + ec.message());
    }
    if (const char* prev = std::getenv("RTC_SESSION_DIR")) {
      had_prev_ = true;
      prev_ = prev;
    }
    ::setenv("RTC_SESSION_DIR", dir_.c_str(), 1);
  }

  ~ScopedSessionDir() {
    if (had_prev_) {
      ::setenv("RTC_SESSION_DIR", prev_.c_str(), 1);
    } else {
      ::unsetenv("RTC_SESSION_DIR");
    }
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  ScopedSessionDir(const ScopedSessionDir&) = delete;
  ScopedSessionDir& operator=(const ScopedSessionDir&) = delete;
  ScopedSessionDir(ScopedSessionDir&&) = delete;
  ScopedSessionDir& operator=(ScopedSessionDir&&) = delete;

  /// This scope's session directory (RTC_SESSION_DIR while it lives).
  [[nodiscard]] const std::filesystem::path& Dir() const noexcept { return dir_; }

 private:
  std::filesystem::path dir_;
  std::string prev_;
  bool had_prev_{false};
};

}  // namespace rtc::testing

#endif  // RTC_BASE_TESTING_ISOLATED_SESSION_DIR_HPP_
