---
paths:
  - "rtc_*/**/*.cpp"
  - "rtc_*/**/*.hpp"
  - "rtc_*/**/*.h"
  - "rtc_*/**/*.cc"
  - "**/rtc_*/**/*.cpp"
  - "**/rtc_*/**/*.hpp"
  - "**/rtc_*/**/*.h"
  - "**/rtc_*/**/*.cc"
---

# RT path 절대 금지 (정기 tick — `control_rate` YAML, 100 Hz–5 kHz, default 500 Hz)

`rtc_*` C++ 핫패스를 편집할 때만 로드되는 path-scoped rule. RT 정기 tick 경로에서 다음을 절대 사용 금지 (CLAUDE.md §3 의 상세판; 위반 필요시 §6 Escalation `[CONCERN]` 포맷 보고).

1. `new` / `malloc` / `push_back` / `emplace_back` / `resize` — pre-allocated fixed-size 사용
2. `throw` / `catch` — error code, `std::optional`, `std::expected`
3. `RCLCPP_INFO/WARN/ERROR/DEBUG/FATAL` 직접 호출 — SPSC → aux thread defer. **예외**: one-shot init, `RCLCPP_*_THROTTLE` with RT-safe msg (단순 format + 기본 타입만; `fmt::format` / `to_string` / string concat 금지)
4. `std::mutex::lock` / `lock_guard` / `scoped_lock` — `try_lock`, `SeqLock`, SPSC, atomic
5. `auto` with Eigen expression — aliasing 버그; 명시 타입
6. Quaternion `lerp` / `nlerp` — `slerp` only
7. 기존 test assertion 수정 — 새 코드를 고쳐라
8. `std::shared_ptr` 복사 — atomic ref-count contention; raw ref 또는 `const std::shared_ptr<T>&`

세부 규칙·grep 패턴·복구 절차: [../../agent_docs/invariants.md](../../agent_docs/invariants.md). Architecture / Process / Numerical invariants (ARCH·PROC·NUM) 는 RT 코드 밖에서도 적용되므로 CLAUDE.md §3 에 상주.
