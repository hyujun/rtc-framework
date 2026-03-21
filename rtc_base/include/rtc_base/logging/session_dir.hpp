#ifndef RTC_BASE_SESSION_DIR_HPP_
#define RTC_BASE_SESSION_DIR_HPP_

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <regex>
#include <string>
#include <vector>

namespace rtc {

// ── 세션 디렉토리 유틸리티 ───────────────────────────────────────────────────
//
// 모든 패키지에서 공유하는 세션 기반 로깅 디렉토리 관리.
// 세션 디렉토리 구조:
//   logging_data/YYMMDD_HHMM/
//     controller/   — rt_controller CSV 로그
//     monitor/      — status_monitor 장애/통계
//     hand/         — hand_udp 통신 통계
//     sim/          — mujoco 스크린샷
//     plots/        — 플롯 출력
//     motions/      — 모션 에디터 JSON
//
// 환경변수 UR5E_SESSION_DIR로 세션 경로를 전파.
// Header-only — ur5e_rt_base의 링크 정책 유지.

/// "YYMMDD_HHMM" 형식의 타임스탬프 문자열 생성
inline std::string GenerateSessionTimestamp() {
  const auto now = std::chrono::system_clock::now();
  const auto time_t = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
  localtime_r(&time_t, &local_tm);
  char ts[16];
  std::strftime(ts, sizeof(ts), "%y%m%d_%H%M", &local_tm);
  return std::string(ts);
}

/// 세션 디렉토리 내 표준 서브디렉토리 생성
inline void EnsureSessionSubdirs(const std::filesystem::path & session_dir) {
  static constexpr const char* kSubdirs[] = {
    "controller", "monitor", "hand", "sim", "plots", "motions"
  };
  for (const auto* sub : kSubdirs) {
    std::filesystem::create_directories(session_dir / sub);
  }
}

/// UR5E_SESSION_DIR 환경변수에서 세션 디렉토리를 읽거나,
/// 없으면 fallback_logging_root 아래에 새 세션 디렉토리를 생성.
///
/// @param fallback_logging_root  환경변수 미설정 시 사용할 logging_data 루트 경로
/// @return 세션 디렉토리 절대 경로 (e.g. .../logging_data/260314_1430)
inline std::filesystem::path ResolveSessionDir(
    const std::string & fallback_logging_root) {
  // 1. 환경변수 우선
  const char* env = std::getenv("UR5E_SESSION_DIR");
  if (env != nullptr && env[0] != '\0') {
    std::filesystem::path session_dir{env};
    std::filesystem::create_directories(session_dir);
    EnsureSessionSubdirs(session_dir);
    return session_dir;
  }

  // 2. Fallback: 자체 세션 생성
  std::filesystem::path logging_root{fallback_logging_root};
  // ~ 확장
  if (!fallback_logging_root.empty() && fallback_logging_root[0] == '~') {
    const char* home = std::getenv("HOME");
    if (home != nullptr) {
      logging_root = std::string(home) + fallback_logging_root.substr(1);
    }
  }

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
