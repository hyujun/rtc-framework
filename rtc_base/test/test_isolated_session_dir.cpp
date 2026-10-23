// Self-test for the test-only session isolation (testing/isolated_session_dir.hpp).
// The binary registers its IsolatedSessionDir the way every consumer does —
// isolated_session_dir_env.cpp is compiled in (see CMakeLists.txt) — so these
// cases run against the same layout a package's tests get.
#include "rtc_base/logging/session_dir.hpp"
#include "rtc_base/testing/isolated_session_dir.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace {

namespace fs = std::filesystem;
using rtc::testing::ScopedSessionDir;
using rtc::testing::SessionDir;

std::string Env() {
  const char* v = std::getenv("RTC_SESSION_DIR");
  return v == nullptr ? std::string() : std::string(v);
}

// The instance a test file reaches is the one the registrar TU set up: the
// inline accessor dedups it across translation units.
TEST(IsolatedSessionDirTest, TheRegisteredSessionIsSetUpBeforeAnyTest) {
  ASSERT_FALSE(SessionDir().Dir().empty());
  EXPECT_TRUE(fs::is_directory(SessionDir().Dir()));
  EXPECT_EQ(SessionDir().Dir().string(), Env());
  EXPECT_EQ(SessionDir().Dir(), rtc::ResolveSessionDir());
  EXPECT_EQ(SessionDir().Dir().parent_path(), SessionDir().Root());
}

TEST(IsolatedSessionDirTest, AScopeIsAFreshSessionUnderThePrivateRoot) {
  const ScopedSessionDir scope("scope_test");
  EXPECT_EQ(SessionDir().Root(), scope.Dir().parent_path());
  EXPECT_NE(SessionDir().Dir(), scope.Dir());
  EXPECT_TRUE(fs::is_directory(scope.Dir()));
  EXPECT_TRUE(fs::is_empty(scope.Dir()));
  EXPECT_EQ(scope.Dir().string(), Env());
  EXPECT_EQ(scope.Dir(), rtc::ResolveSessionDir());
}

TEST(IsolatedSessionDirTest, AScopeRestoresTheSessionAndRemovesItsDirectory) {
  fs::path dir;
  {
    const ScopedSessionDir scope;
    dir = scope.Dir();
    ASSERT_TRUE(fs::is_directory(dir));
  }
  EXPECT_FALSE(fs::exists(dir));
  EXPECT_EQ(SessionDir().Dir().string(), Env());
}

TEST(IsolatedSessionDirTest, NestedScopesRestoreInReverseOrder) {
  const ScopedSessionDir outer("outer");
  {
    const ScopedSessionDir inner("inner");
    EXPECT_NE(outer.Dir(), inner.Dir());
    EXPECT_EQ(inner.Dir().string(), Env());
  }
  EXPECT_EQ(outer.Dir().string(), Env());
}

// A configure runs retention on its session's parent, which for a scope is the
// private root — it must not take this binary's directories for old sessions.
TEST(IsolatedSessionDirTest, RetentionOnThePrivateRootKeepsEveryScope) {
  const ScopedSessionDir scope;
  EXPECT_TRUE(rtc::ListSessionDirs(SessionDir().Root()).empty());
  rtc::CleanupOldSessions(SessionDir().Root(), 0);
  EXPECT_TRUE(fs::is_directory(scope.Dir()));
  EXPECT_TRUE(fs::is_directory(SessionDir().Dir()));
}

TEST(IsolatedSessionDirTest, ASixDigitTagIsRefused) {
  // "<tag>_<n>" with a six-digit tag reaches YYMMDD_HHMM once n has four digits.
  EXPECT_THROW(ScopedSessionDir("260314"), std::logic_error);
  EXPECT_EQ(SessionDir().Dir().string(), Env());
}

TEST(IsolatedSessionDirTest, ATagThatLeavesThePrivateRootIsRefused) {
  EXPECT_THROW(ScopedSessionDir("../escape"), std::logic_error);
  EXPECT_EQ(SessionDir().Dir().string(), Env());
}

}  // namespace
