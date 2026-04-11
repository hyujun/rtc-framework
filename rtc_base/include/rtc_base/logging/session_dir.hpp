#ifndef RTC_BASE_SESSION_DIR_HPP_
#define RTC_BASE_SESSION_DIR_HPP_

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

#include <unistd.h>

namespace rtc {

// ── 세션 디렉토리 유틸리티 ───────────────────────────────────────────────────
//
// 모든 패키지에서 공유하는 세션 기반 로깅 디렉토리 관리.
// 세션 디렉토리 구조:
//   logging_data/YYMMDD_HHMM/
//     controller/   — rt_controller CSV 로그
//     monitor/      — 모니터링 로그
//     device/       — device 통신 통계 / 센서 로그
//     sim/          — mujoco 스크린샷
//     plots/        — 플롯 출력
//     motions/      — 모션 에디터 JSON
//
// 로깅 루트 결정 체인 (ResolveLoggingRoot):
//   1) $COLCON_PREFIX_PATH 첫 entry 의 parent  (ws/logging_data)
//   2) cwd 에서 상위로 올라가며 install/+src/ 쌍을 찾음
//   3) $PWD/logging_data
//
// 세션 디렉토리 결정 체인 (ResolveSessionDir):
//   1) $RTC_SESSION_DIR (fallback: $UR5E_SESSION_DIR)
//   2) ResolveLoggingRoot() / YYMMDD_HHMM
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
inline void EnsureSessionSubdirs(const std::filesystem::path & session_dir) {
  static constexpr const char* kSubdirs[] = {
    "controller", "monitor", "device", "sim", "plots", "motions"
  };
  for (const auto* sub : kSubdirs) {
    std::filesystem::create_directories(session_dir / sub);
  }
}

/// 로깅 루트 디렉토리 결정 (env 우선순위 제외, 물리 경로만).
///
/// 1) $COLCON_PREFIX_PATH 첫 entry 가 쓰기 가능한 dir → parent / "logging_data"
/// 2) cwd 에서 "/" 까지 올라가며 install/ + src/ 쌍 발견 → that / "logging_data"
/// 3) cwd / "logging_data"
inline std::filesystem::path ResolveLoggingRoot() {
  namespace fs = std::filesystem;

  // 1) COLCON_PREFIX_PATH
  if (const char* raw = std::getenv("COLCON_PREFIX_PATH"); raw && *raw) {
    std::string_view view{raw};
    const auto colon = view.find(':');
    const std::string first{view.substr(0, colon)};
    std::error_code ec;
    if (!first.empty() && fs::is_directory(first, ec) &&
        ::access(first.c_str(), W_OK) == 0) {
      return fs::path(first).parent_path() / "logging_data";
    }
  }

  // 2) cwd 상위 탐색 (install/ + src/ 쌍)
  std::error_code err_code;
  fs::path cwd = fs::current_path(err_code);
  if (!err_code) {
    for (fs::path dir = cwd; ; ) {
      if (fs::is_directory(dir / "install", err_code) &&
          fs::is_directory(dir / "src", err_code)) {
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

/// 세션 디렉토리를 결정하고 서브디렉토리까지 생성.
///
/// 1) $RTC_SESSION_DIR → $UR5E_SESSION_DIR (하위 호환)
/// 2) ResolveLoggingRoot() / YYMMDD_HHMM
///
/// @return 세션 디렉토리 절대 경로 (e.g. .../logging_data/260314_1430)
inline std::filesystem::path ResolveSessionDir() {
  // 1. 환경변수 우선 (RTC_SESSION_DIR → UR5E_SESSION_DIR fallback)
  const char* env = std::getenv("RTC_SESSION_DIR");
  if (!env) {
    env = std::getenv("UR5E_SESSION_DIR");  // backward compat
  }
  if (env != nullptr && env[0] != '\0') {
    std::filesystem::path session_dir{env};
    std::filesystem::create_directories(session_dir);
    EnsureSessionSubdirs(session_dir);
    return session_dir;
  }

  // 2. 자체 세션 생성 (3단 체인으로 루트 결정)
  const std::filesystem::path logging_root = ResolveLoggingRoot();
  const std::string ts = GenerateSessionTimestamp();
  std::filesystem::path session_dir = logging_root / ts;
  std::filesystem::create_directories(session_dir);
  EnsureSessionSubdirs(session_dir);
  return session_dir;
}

/// YYMMDD_HHMM 패턴과 일치하는 세션 디렉토리 목록을 정렬하여 반환
inline std::vector<std::filesystem::path> ListSessionDirs(
    const std::filesystem::path & logging_root) {
  std::vector<std::filesystem::path> dirs;
  if (!std::filesystem::exists(logging_root)) {
    return dirs;
  }

  // YYMMDD_HHMM 패턴: 6자리 날짜 _ 4자리 시간
  const std::regex pattern(R"(\d{6}_\d{4})");
  for (const auto & entry : std::filesystem::directory_iterator(logging_root)) {
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
inline void CleanupOldSessions(
    const std::filesystem::path & logging_root,
    int max_sessions) {
  auto dirs = ListSessionDirs(logging_root);
  while (static_cast<int>(dirs.size()) > max_sessions) {
    std::filesystem::remove_all(dirs.front());
    dirs.erase(dirs.begin());
  }
}

}  // namespace rtc

#endif  // RTC_BASE_SESSION_DIR_HPP_
