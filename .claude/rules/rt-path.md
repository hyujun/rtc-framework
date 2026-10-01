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
  - "integrated_bringup/**/*.cpp"
  - "integrated_bringup/**/*.hpp"
  - "integrated_bringup/**/*.h"
  - "integrated_bringup/**/*.cc"
  - "**/integrated_bringup/**/*.cpp"
  - "**/integrated_bringup/**/*.hpp"
  - "**/integrated_bringup/**/*.h"
  - "**/integrated_bringup/**/*.cc"
---

# RT path 절대 금지 (정기 tick — `control_rate` YAML; rate 범위·default 는 [invariants.md](../../agent_docs/invariants.md) §RT Path Invariants 가 SSoT)

`rtc_*` · `integrated_bringup` C++ 파일을 읽거나 편집할 때 로드되는 reminder 다. **구속 대상은 RT 정기 tick / SCHED_FIFO dedicated-core 경로뿐**이다 (`integrated_bringup` 에서는 device backend 의 rt_callback lane 과 controller `Compute()`) — 비-RT 코드 (`on_configure`/`on_activate` 등 lifecycle 콜백, `DrainLog()` aux thread, 1 Hz aux 타이머, 파라미터 콜백, **test / init 코드**) 는 면제. RT path 정의·전체 목록·탐지 패턴·복구·false-positive 판정 절차는 [invariants.md](../../agent_docs/invariants.md) §RT Path Invariants 가 SSoT. RT tick 경로에서 다음을 쓰지 않는다 (위반이 필요하면 AGENTS.md §6 의 `[CONCERN]`):

- **RT-1** `new` / `malloc` / `push_back` / `emplace_back` / `resize` — pre-allocated fixed-size 사용. invariants.md 에 기록된 알려진 위반은 새 RT 코드의 할당 근거가 아니다
- **RT-2** `throw` / `catch` — error code, `std::optional`, `std::expected`
- **RT-3** `RCLCPP_INFO/WARN/ERROR/DEBUG/FATAL` 직접 호출 — SPSC → aux thread defer. **예외**: one-shot init, `RCLCPP_*_THROTTLE` with RT-safe msg (단순 format + 기본 타입만; `fmt::format` / `to_string` / string concat 금지)
- **RT-4** `std::mutex::lock` / `lock_guard` / `scoped_lock` — `try_lock`, `SeqLock`, SPSC, atomic
- **RT-5** `auto` with Eigen expression — aliasing 버그; 명시 타입
- **RT-6** Quaternion `lerp` / `nlerp` — `slerp` only
- **RT-8** `std::shared_ptr` 복사 — atomic ref-count contention; raw ref 또는 `const std::shared_ptr<T>&`
- **RT-9** `get_lifecycle_state()` / `get_current_state()` — 내부 state machine 동기화; `on_activate` 에서 `std::atomic<uint8_t>` 캐시에 store 하고 RT loop 는 load
- **RT-10** `std::condition_variable` 의 `notify_*` / `wait*` — `notify` 자체가 내부 mutex 보유; eventfd + non-blocking write/poll, 또는 SPSC + polling

## 이 파일이 RT path 인지 판정하는 법

glob 은 파일 단위라 여기 로드됐다는 사실만으로는 아무것도 결정되지 않는다. **편집하려는 함수가 어느 execution context 에서 실행되는지**를 [architecture.md](../../agent_docs/architecture.md) §Execution Contexts 표에서 찾는다:

1. 그 함수의 호출자를 따라 올라가 진입점을 찾는다 — `ControlLoop` tick / dedicated SCHED_FIFO thread 의 loop / executor 콜백 / lifecycle 콜백 중 하나다.
2. Executor 콜백이면 **어느 callback group** 인지 본다. `cb_group_rt_callback_` (backend state/motor/sensor sub) 이면 RT, controller LifecycleNode 의 default group (RobotTarget sub, `grasp_command` 서비스) 이면 비-RT 다. 이름이 아니라 group 이 판정한다.
3. 여전히 애매하면 비-RT 로 가정하지 말고 AGENTS.md §6 의 `[CONCERN]` 으로 보고한다.

흔한 오판 두 가지: `PublishNonRtSnapshot()` 은 이름과 달리 **비-RT** (전용 jthread, SCHED_OTHER) 이고, `on_configure` 안의 `push_back` 은 **위반이 아니다** (lifecycle = 비-RT).

기존 test assertion 수정은 RT 규칙이 아니라 process 규칙이다 ([invariants.md](../../agent_docs/invariants.md) PROC-6 · AGENTS.md §6 E-6). ARCH · PROC · NUM invariant 은 RT 코드 밖에서도 적용된다 (AGENTS.md §3).
