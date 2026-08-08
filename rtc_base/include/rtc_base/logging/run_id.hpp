#ifndef RTC_BASE_LOGGING_RUN_ID_HPP_
#define RTC_BASE_LOGGING_RUN_ID_HPP_

#include <unistd.h>

#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <system_error>

namespace rtc {

// ── 런 식별자 ────────────────────────────────────────────────────────────────
//
// 한 세션 디렉토리 *안에서* 이번 기동과 직전 기동을 가르는 경계 표식.
//
// 세션 디렉토리는 분 해상도이고 (session_dir.hpp 의 "%y%m%d_%H%M"), 타이밍 CSV
// 로거는 의도적으로 append 로 연다 (재시작 세션 이어쓰기 — 헤더 중복 없음).
// 따라서 같은 분 안의 두 기동은 한 파일을 공유하는데, #376 이전에는 그 파일에
// 런 경계가 **어디에도 기록되지 않았다**: `n / span` 으로 레이트를 내는 분석이
// 두 런을 한 런으로 읽고도 아무 센서가 발화하지 않았다.
//
// 결정 체인 (ResolveRunId):
//   1) $RTC_RUN_ID — launch 가 $RTC_SESSION_DIR 과 함께 싣는다. 한 launch 의
//      모든 프로세스가 **같은 값**을 보고하므로 스레드별 CSV (cm / mpc /
//      hand_udp / rt_callback) 를 run 단위로 join 할 수 있다.
//   2) getpid() — launch 밖에서 기동된 프로세스 (`ros2 run`, 테스트) 의 폴백.
//      프로세스 간 join 은 포기하지만 **각 파일 안의 런 경계는 여전히 보인다**
//      (재기동 = 다른 PID). 시각 기반 폴백을 쓰지 않는 이유는 같은 프로세스의
//      두 로거가 초 경계를 사이에 두고 Open 되면 한 런이 두 값으로 갈리기
//      때문이다 — PID 는 프로세스 수명 동안 불변이라 그 창이 없다.
//
// Header-only — rtc_base 의 링크 정책 유지.

/// 이번 런의 식별자. 위 결정 체인을 매 호출 평가한다 (캐시 없음 — 값이 env
/// 또는 PID 라 프로세스 수명 동안 상수이므로 캐시가 사는 것이 없고, 캐시가
/// 있으면 테스트가 한 바이너리 안에서 두 런을 흉내낼 수 없다).
[[nodiscard]] inline std::uint64_t ResolveRunId() noexcept {
  if (const char* env = std::getenv("RTC_RUN_ID"); env != nullptr && env[0] != '\0') {
    const char* const end = env + std::strlen(env);
    std::uint64_t parsed = 0;
    const auto [ptr, ec] = std::from_chars(env, end, parsed);
    // 부분 파싱은 거부한다 ("12abc" 를 12 로 읽으면 오타가 조용히 유효한 런
    // 번호가 된다). 거부 시 PID 폴백이라 경계는 여전히 남는다.
    if (ec == std::errc{} && ptr == end) {
      return parsed;
    }
  }
  return static_cast<std::uint64_t>(::getpid());
}

}  // namespace rtc

#endif  // RTC_BASE_LOGGING_RUN_ID_HPP_
