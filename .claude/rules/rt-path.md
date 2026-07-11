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

# RT path 절대 금지 (정기 tick — `control_rate` YAML; rate 범위·default 는 [invariants.md](../../agent_docs/invariants.md) §RT Path Invariants 가 SSoT)

이 rule 은 `rtc_*` C++ 파일을 편집할 때 **reminder 로 로드**된다 (glob 은 파일 단위라 tick hot-path 만 골라낼 수 없다). 실제로 **구속하는 대상은 RT 정기 tick / SCHED_FIFO dedicated-core 경로뿐**이다 — 비-RT 코드 (`on_configure`/`on_activate` 등 lifecycle 콜백, `DrainLog()` aux thread, 1 Hz aux 타이머, 파라미터 콜백, **test / init 코드**) 는 면제. RT path 정의와 false-positive 판정 절차는 [invariants.md](../../agent_docs/invariants.md) §RT Path Invariants. RT tick 경로에서 다음을 절대 사용 금지 (CLAUDE.md §3 의 상세판; 위반 필요시 §6 Escalation `[CONCERN]` 포맷 보고).

1. `new` / `malloc` / `push_back` / `emplace_back` / `resize` — pre-allocated fixed-size 사용
2. `throw` / `catch` — error code, `std::optional`, `std::expected`
3. `RCLCPP_INFO/WARN/ERROR/DEBUG/FATAL` 직접 호출 — SPSC → aux thread defer. **예외**: one-shot init, `RCLCPP_*_THROTTLE` with RT-safe msg (단순 format + 기본 타입만; `fmt::format` / `to_string` / string concat 금지)
4. `std::mutex::lock` / `lock_guard` / `scoped_lock` — `try_lock`, `SeqLock`, SPSC, atomic
5. `auto` with Eigen expression — aliasing 버그; 명시 타입
6. Quaternion `lerp` / `nlerp` — `slerp` only
7. `std::shared_ptr` 복사 — atomic ref-count contention; raw ref 또는 `const std::shared_ptr<T>&`

**여기 없는 것 — "기존 test assertion 수정"**: 이는 timing-safety 가 아니라 **process 규칙** (회귀 은폐 방지) 이므로 RT 목록에서 분리했다 → [invariants.md](../../agent_docs/invariants.md) PROC-6 / CLAUDE.md §6 E-6. Test 가 진짜 틀렸거나 spec 이 바뀐 경우의 정당한 변경까지 막지 않는다 (근거 + 별도 commit).

세부 규칙·grep 패턴·복구 절차: [../../agent_docs/invariants.md](../../agent_docs/invariants.md). Architecture / Process / Numerical invariants (ARCH·PROC·NUM) 는 RT 코드 밖에서도 적용되므로 CLAUDE.md §3 에 상주.
