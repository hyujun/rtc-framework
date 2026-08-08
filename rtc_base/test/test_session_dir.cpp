// ── test_session_dir.cpp ─────────────────────────────────────────────────────
// rtc::ResolveLoggingRoot() / ResolveSessionDir() 4단 체인 검증 +
// rtc::ResolveRunId() 2단 체인 (그 헤더의 형제이고 같은 env 규율을 쓴다).
//
// 각 테스트는 환경변수(COLCON_PREFIX_PATH, RTC_SESSION_DIR, RTC_RUN_ID)
// 와 cwd 를 독립적으로 세팅하도록 scoped fixture 로 관리합니다.
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_base/logging/run_id.hpp>
#include <rtc_base/logging/session_dir.hpp>

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

namespace {

// ScopedEnv: RAII 로 환경변수를 세팅하고 테스트 종료 시 복원.
class ScopedEnv {
 public:
  ScopedEnv(const char* name, const std::string& value) : name_(name) {
    if (const char* prev = std::getenv(name)) {
      prev_ = prev;
      had_prev_ = true;
    }
    ::setenv(name, value.c_str(), 1);
  }

  ~ScopedEnv() {
    if (had_prev_) {
      ::setenv(name_, prev_.c_str(), 1);
    } else {
      ::unsetenv(name_);
    }
  }

  ScopedEnv(const ScopedEnv&) = delete;
  ScopedEnv& operator=(const ScopedEnv&) = delete;

 private:
  const char* name_;
  std::string prev_;
  bool had_prev_{false};
};

// ScopedUnsetEnv: 특정 환경변수를 강제로 해제.
class ScopedUnsetEnv {
 public:
  explicit ScopedUnsetEnv(const char* name) : name_(name) {
    if (const char* prev = std::getenv(name)) {
      prev_ = prev;
      had_prev_ = true;
    }
    ::unsetenv(name);
  }

  ~ScopedUnsetEnv() {
    if (had_prev_) {
      ::setenv(name_, prev_.c_str(), 1);
    }
  }

 private:
  const char* name_;
  std::string prev_;
  bool had_prev_{false};
};

// ScopedCwd: chdir 기반 fixture.
class ScopedCwd {
 public:
  explicit ScopedCwd(const fs::path& new_cwd) {
    prev_ = fs::current_path();
    fs::current_path(new_cwd);
  }

  ~ScopedCwd() {
    std::error_code err;
    fs::current_path(prev_, err);
  }

 private:
  fs::path prev_;
};

// 각 테스트마다 독립된 tmp 디렉토리를 만들고 종료 시 삭제.
class SessionDirTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tmp_root_ = fs::temp_directory_path() / ("rtc_session_test_" + std::to_string(::getpid()) +
                                             "_" + std::to_string(counter_++));
    fs::create_directories(tmp_root_);
  }

  void TearDown() override {
    std::error_code err;
    fs::remove_all(tmp_root_, err);
  }

  fs::path tmp_root_;
  static inline int counter_{0};
};

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// ResolveLoggingRoot() — 3단 체인
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SessionDirTest, LoggingRoot_UsesColconPrefixPathParent) {
  // install/ 디렉토리를 만들고 COLCON_PREFIX_PATH 로 지정.
  const fs::path ws = tmp_root_ / "ws";
  const fs::path install = ws / "install";
  fs::create_directories(install);

  ScopedUnsetEnv clear_rtc{"RTC_SESSION_DIR"};
  ScopedEnv prefix{"COLCON_PREFIX_PATH", install.string()};

  const fs::path root = rtc::ResolveLoggingRoot();
  EXPECT_EQ(root, ws / "logging_data");
}

TEST_F(SessionDirTest, LoggingRoot_FirstColonEntryWins) {
  // 여러 overlay 중 첫 entry 만 사용하는지 확인.
  const fs::path ws1 = tmp_root_ / "ws1";
  const fs::path ws2 = tmp_root_ / "ws2";
  fs::create_directories(ws1 / "install");
  fs::create_directories(ws2 / "install");

  const std::string joined = (ws1 / "install").string() + ":" + (ws2 / "install").string();

  ScopedUnsetEnv clear_rtc{"RTC_SESSION_DIR"};
  ScopedEnv prefix{"COLCON_PREFIX_PATH", joined};

  EXPECT_EQ(rtc::ResolveLoggingRoot(), ws1 / "logging_data");
}

TEST_F(SessionDirTest, LoggingRoot_FallsBackWhenPrefixMissing) {
  // 존재하지 않는 install/ 경로는 스킵되고 cwd 상위 탐색으로 넘어감.
  const fs::path ghost = tmp_root_ / "ghost" / "install";
  const fs::path ws = tmp_root_ / "real_ws";
  fs::create_directories(ws / "install");
  fs::create_directories(ws / "src");

  ScopedUnsetEnv clear_rtc{"RTC_SESSION_DIR"};
  ScopedEnv prefix{"COLCON_PREFIX_PATH", ghost.string()};
  ScopedCwd cwd{ws};

  EXPECT_EQ(rtc::ResolveLoggingRoot(), ws / "logging_data");
}

TEST_F(SessionDirTest, LoggingRoot_FindsInstallSrcPairByWalkingUp) {
  // ws/install, ws/src 가 있으면 어느 깊이에서 실행해도 ws/logging_data 반환.
  const fs::path ws = tmp_root_ / "walk_ws";
  const fs::path deep = ws / "src" / "pkg" / "include" / "subdir";
  fs::create_directories(ws / "install");
  fs::create_directories(deep);

  ScopedUnsetEnv clear_rtc{"RTC_SESSION_DIR"};
  ScopedUnsetEnv clear_prefix{"COLCON_PREFIX_PATH"};
  ScopedCwd cwd{deep};

  EXPECT_EQ(rtc::ResolveLoggingRoot(), ws / "logging_data");
}

TEST_F(SessionDirTest, LoggingRoot_FinalFallbackIsCwd) {
  // 아무 marker 도 없으면 cwd / logging_data.
  const fs::path bare = tmp_root_ / "bare";
  fs::create_directories(bare);

  ScopedUnsetEnv clear_rtc{"RTC_SESSION_DIR"};
  ScopedUnsetEnv clear_prefix{"COLCON_PREFIX_PATH"};
  ScopedCwd cwd{bare};

  EXPECT_EQ(rtc::ResolveLoggingRoot(), bare / "logging_data");
}

// ─────────────────────────────────────────────────────────────────────────────
// ResolveSessionDir() — env 우선, subdir 생성 검증
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SessionDirTest, SessionDir_UsesRtcSessionDirEnv) {
  const fs::path explicit_session = tmp_root_ / "explicit_session";
  ScopedEnv rtc_env{"RTC_SESSION_DIR", explicit_session.string()};

  const fs::path got = rtc::ResolveSessionDir();
  EXPECT_EQ(got, explicit_session);
  // 표준 서브디렉토리가 모두 만들어져 있어야 함.
  for (const char* sub : {"timing", "monitor", "device", "sim", "plots", "motions"}) {
    EXPECT_TRUE(fs::is_directory(explicit_session / sub)) << sub;
  }
  // 단수 controller/ 는 더 이상 생성하지 않음 (per-controller CSV 는 controllers/).
  EXPECT_FALSE(fs::is_directory(explicit_session / "controller"));
  // TimingDir() helper는 동일한 timing/ 경로를 반환해야 함.
  EXPECT_EQ(rtc::TimingDir(explicit_session), explicit_session / "timing");
}

TEST_F(SessionDirTest, SessionDir_NoEnv_CreatesUnderResolvedRoot) {
  // env 없음 + cwd 에 install/+src/ 존재 → ws/logging_data/YYMMDD_HHMM 생성.
  const fs::path ws = tmp_root_ / "auto_ws";
  fs::create_directories(ws / "install");
  fs::create_directories(ws / "src");

  ScopedUnsetEnv clear_rtc{"RTC_SESSION_DIR"};
  ScopedUnsetEnv clear_prefix{"COLCON_PREFIX_PATH"};
  ScopedCwd cwd{ws};

  const fs::path session = rtc::ResolveSessionDir();
  EXPECT_EQ(session.parent_path(), ws / "logging_data");
  EXPECT_TRUE(fs::is_directory(session / "timing"));
}

// ─────────────────────────────────────────────────────────────────────────────
// CleanupOldSessions — max_sessions 제한
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SessionDirTest, CleanupOldSessions_KeepsOnlyMaxSessions) {
  const fs::path root = tmp_root_ / "logging_data";
  fs::create_directories(root);
  // 오름차순으로 정렬되는 타임스탬프 형식 5개
  for (const char* ts :
       {"260101_0900", "260102_1000", "260103_1100", "260104_1200", "260105_1300"}) {
    fs::create_directories(root / ts);
  }
  // 비정상 이름 — 대상 아님
  fs::create_directories(root / "not_a_session");

  rtc::CleanupOldSessions(root, 2);

  EXPECT_FALSE(fs::exists(root / "260101_0900"));
  EXPECT_FALSE(fs::exists(root / "260102_1000"));
  EXPECT_FALSE(fs::exists(root / "260103_1100"));
  EXPECT_TRUE(fs::exists(root / "260104_1200"));
  EXPECT_TRUE(fs::exists(root / "260105_1300"));
  EXPECT_TRUE(fs::exists(root / "not_a_session"));
}

// ─────────────────────────────────────────────────────────────────────────────
// ResolveRunId — $RTC_RUN_ID → getpid() 2단 체인 (#376)
//
// 세션 디렉토리가 분 해상도라 같은 분의 두 기동은 같은 timing CSV 에 append
// 된다. run_id 가 그 경계이므로, 이 체인이 조용히 상수로 떨어지면 (예: 파싱
// 실패를 0 으로 읽어 두 런이 같은 값을 갖는 경우) 경계가 다시 사라진다.
// ─────────────────────────────────────────────────────────────────────────────

TEST(ResolveRunId, UsesEnvWhenSet) {
  const ScopedEnv run_id{"RTC_RUN_ID", "260808143052"};
  EXPECT_EQ(rtc::ResolveRunId(), 260808143052ULL);
}

TEST(ResolveRunId, FallsBackToPidWhenUnset) {
  const ScopedUnsetEnv clear_run_id{"RTC_RUN_ID"};
  const auto pid = static_cast<std::uint64_t>(::getpid());
  EXPECT_EQ(rtc::ResolveRunId(), pid);
  // 한 프로세스 안에서 상수여야 한다 — 같은 런의 두 로거(cm / rt_callback)가
  // 다른 값을 받으면 한 런이 두 런으로 보인다.
  EXPECT_EQ(rtc::ResolveRunId(), rtc::ResolveRunId());
}

TEST(ResolveRunId, EmptyEnvFallsBackToPid) {
  const ScopedEnv run_id{"RTC_RUN_ID", ""};
  EXPECT_EQ(rtc::ResolveRunId(), static_cast<std::uint64_t>(::getpid()));
}

TEST(ResolveRunId, PartialParseIsRejected) {
  // "12abc" 를 12 로 읽으면 오타가 조용히 유효한 런 번호가 된다. 거부하고
  // PID 폴백으로 떨어져야 경계가 남는다.
  const ScopedEnv run_id{"RTC_RUN_ID", "12abc"};
  EXPECT_EQ(rtc::ResolveRunId(), static_cast<std::uint64_t>(::getpid()));
}

TEST(ResolveRunId, NonNumericFallsBackToPid) {
  const ScopedEnv run_id{"RTC_RUN_ID", "not-a-number"};
  EXPECT_EQ(rtc::ResolveRunId(), static_cast<std::uint64_t>(::getpid()));
}
