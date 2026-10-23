#ifndef RTC_BASE_SESSION_DIR_HPP_
#define RTC_BASE_SESSION_DIR_HPP_

#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <regex>
#include <string>
#include <string_view>
#include <vector>

namespace rtc {

// ── 세션 디렉토리 유틸리티 ───────────────────────────────────────────────────
//
// 모든 패키지에서 공유하는 세션 기반 로깅 디렉토리 관리.
// 세션 디렉토리 구조:
//   logging_data/YYMMDD_HHMM/
//     controllers/  — per-controller 데이터 CSV (예:
//                     demo_wbc_controller/<log_name>.csv); on-demand 생성
//     timing/       — per-tick 스레드 타이밍 CSV (cm_timing_log,
//                     mpc_timing_log, hand_timing_log,
//                     rt_callback_timing_log)
//     monitor/      — 모니터링 로그
//     device/       — device 통신 통계 / 센서 로그
//     sim/          — mujoco 스크린샷
//     plots/        — 플롯 출력
//     motions/      — 모션 에디터 JSON
//     perf/         — Linux perf record 출력 (perf.data; Hotspot 으로 뷰)
//
// 로깅 루트 결정 체인 (ResolveLoggingRoot):
//   1) $COLCON_PREFIX_PATH 첫 entry 의 parent  (ws/logging_data)
//   2) cwd 에서 상위로 올라가며 install/+src/ 쌍을 찾음
//   3) $PWD/logging_data
//
// 세션 디렉토리 결정 체인 (ResolveSessionDir):
//   1) $RTC_SESSION_DIR
//   2) ResolveLoggingRoot() / ProcessSessionName()  (프로세스당 한 YYMMDD_HHMM)
//
// Header-only — rtc_base의 링크 정책 유지.

/// "YYMMDD_HHMM" 형식의 타임스탬프 문자열 생성
inline std::string GenerateSessionTimestamp() {
  const auto now = std::chrono::system_clock::now();
  const auto time_t = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
  localtime_r(&time_t, &local_tm);
  std::array<char, 16> ts{};
  std::strftime(ts.data(), ts.size(), "%y%m%d_%H%M", &local_tm);
  return std::string(ts.data());
}

/// 세션 디렉토리 내 표준 서브디렉토리 생성
inline void EnsureSessionSubdirs(const std::filesystem::path& session_dir) {
  static constexpr const char* kSubdirs[] = {"timing", "monitor", "device", "sim",
                                             "plots",  "motions", "tracing"};
  for (const auto* sub : kSubdirs) {
    std::filesystem::create_directories(session_dir / sub);
  }
}

/// per-tick 스레드 타이밍 CSV 들의 공용 디렉토리.
/// (cm_timing_log.csv, mpc_timing_log.csv, hand_timing_log.csv,
///  rt_callback_timing_log.csv)
inline std::filesystem::path TimingDir(const std::filesystem::path& session_dir) {
  return session_dir / "timing";
}

/// 로깅 루트 디렉토리 결정 (env 우선순위 제외, 물리 경로만).
///
/// 1) $COLCON_PREFIX_PATH 첫 entry 가 쓰기 가능한 dir → parent / "logging_data"
/// 2) cwd 에서 "/" 까지 올라가며 install/ + src/ 쌍 발견 → that /
/// "logging_data" 3) cwd / "logging_data"
inline std::filesystem::path ResolveLoggingRoot() {
  namespace fs = std::filesystem;

  // 1) COLCON_PREFIX_PATH
  if (const char* raw = std::getenv("COLCON_PREFIX_PATH"); raw && *raw) {
    std::string_view view{raw};
    const auto colon = view.find(':');
    const std::string first{view.substr(0, colon)};
    std::error_code ec;
    if (!first.empty() && fs::is_directory(first, ec) && ::access(first.c_str(), W_OK) == 0) {
      return fs::path(first).parent_path() / "logging_data";
    }
  }

  // 2) cwd 상위 탐색 (install/ + src/ 쌍)
  std::error_code err_code;
  fs::path cwd = fs::current_path(err_code);
  if (!err_code) {
    for (fs::path dir = cwd;;) {
      if (fs::is_directory(dir / "install", err_code) && fs::is_directory(dir / "src", err_code)) {
        return dir / "logging_data";
      }
      fs::path parent = dir.parent_path();
      if (parent == dir) {
        break;
      }
      dir = std::move(parent);
    }
  }

  // 3) cwd 폴백
  return (cwd.empty() ? fs::path(".") : cwd) / "logging_data";
}

/// env 폴백 세션의 이름 — 이 프로세스에서 처음 불린 시각의 "YYMMDD_HHMM".
///
/// 호출처는 한 프로세스의 ResolveSessionDir() 호출이 모두 같은 세션이라고
/// 가정한다 (주기 저장 타이머, on_activate 의 여러 번, `log_dir` 없이 configure
/// 하는 CM 과 컨트롤러 log set). 매 호출 시각을 찍으면 분이 바뀔 때마다 새
/// 디렉토리가 생기고 한 런의 파일이 흩어진다 (#930). run_id.hpp 는 값이 env
/// 또는 PID 라 원래 상수여서 캐시하지 않지만, 이 이름은 시각이므로 고정하려면
/// 캐시가 필요하다. 루트는 캐시하지 않는다 — 노드는 실행 중에 cwd 나
/// $COLCON_PREFIX_PATH 를 바꾸지 않고, 테스트는 케이스마다 루트를 바꾼다.
///
/// 함수-local static 은 이 헤더를 STATIC 라이브러리·실행 파일로 링크하는 한
/// 프로세스에 하나다. 다음 경우에는 둘로 갈린다 (g++ 13.3 실측): 동적 심볼을
/// 내보내지 않는 (`-rdynamic` 없는) 실행 파일과 dlopen 한 .so, 그리고 hidden
/// visibility 로 빌드한 SHARED 라이브러리 (링크해도). 세션 해석을 그런 .so 로
/// 옮기려면 그 전에 out-of-line 정의로 바꾼다. 첫 호출은 초기화 guard 를
/// 잡으므로 RT 경로에서 부르지 않는다.
inline const std::string& ProcessSessionName() {
  static const std::string name = GenerateSessionTimestamp();
  return name;
}

/// 세션 디렉토리를 결정하고 서브디렉토리까지 생성.
///
/// 1) $RTC_SESSION_DIR — 매 호출 확인
/// 2) ResolveLoggingRoot() / ProcessSessionName() — 루트는 매 호출 다시 풀고,
///    이름은 프로세스의 첫 폴백 호출에서 한 번 정해진다
///
/// @return 세션 디렉토리 절대 경로 (e.g. .../logging_data/260314_1430)
inline std::filesystem::path ResolveSessionDir() {
  // 1. 환경변수 우선 (RTC_SESSION_DIR)
  const char* env = std::getenv("RTC_SESSION_DIR");
  if (env != nullptr && env[0] != '\0') {
    std::filesystem::path session_dir{env};
    std::filesystem::create_directories(session_dir);
    EnsureSessionSubdirs(session_dir);
    return session_dir;
  }

  // 2. 자체 세션 (3단 체인으로 루트 결정, 이름은 프로세스당 하나)
  const std::filesystem::path logging_root = ResolveLoggingRoot();
  std::filesystem::path session_dir = logging_root / ProcessSessionName();
  std::filesystem::create_directories(session_dir);
  EnsureSessionSubdirs(session_dir);
  return session_dir;
}

/// YYMMDD_HHMM 패턴과 일치하는 세션 디렉토리 목록을 정렬하여 반환
inline std::vector<std::filesystem::path> ListSessionDirs(
    const std::filesystem::path& logging_root) {
  std::vector<std::filesystem::path> dirs;
  if (!std::filesystem::exists(logging_root)) {
    return dirs;
  }

  // YYMMDD_HHMM 패턴: 6자리 날짜 _ 4자리 시간
  const std::regex pattern(R"(\d{6}_\d{4})");
  for (const auto& entry : std::filesystem::directory_iterator(logging_root)) {
    if (entry.is_directory()) {
      const std::string name = entry.path().filename().string();
      if (std::regex_match(name, pattern)) {
        dirs.push_back(entry.path());
      }
    }
  }
  std::sort(dirs.begin(), dirs.end());
  return dirs;
}

/// 오래된 세션 디렉토리를 삭제하여 max_sessions 개수 이하로 유지.
/// logging_root 내 YYMMDD_HHMM 패턴 디렉토리만 대상.
///
/// `keep` 은 호출자가 지금 쓰는 세션이다 — 개수에는 들지만 지우지 않는다.
/// 오래 사는 프로세스의 세션은 이름이 고정이라 (ProcessSessionName, 또는
/// launch 가 준 $RTC_SESSION_DIR) 재-configure 때 가장 오래된 것일 수 있다.
inline void CleanupOldSessions(const std::filesystem::path& logging_root, int max_sessions,
                               const std::filesystem::path& keep = {}) {
  auto dirs = ListSessionDirs(logging_root);
  if (!keep.empty()) {
    std::error_code err;
    const auto kept = std::find_if(dirs.begin(), dirs.end(), [&](const auto& dir) {
      return std::filesystem::equivalent(dir, keep, err);
    });
    if (kept != dirs.end()) {
      dirs.erase(kept);
      --max_sessions;
    }
  }
  while (!dirs.empty() && static_cast<int>(dirs.size()) > max_sessions) {
    std::filesystem::remove_all(dirs.front());
    dirs.erase(dirs.begin());
  }
}

}  // namespace rtc

#endif  // RTC_BASE_SESSION_DIR_HPP_
