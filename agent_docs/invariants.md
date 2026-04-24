# Invariants

이 파일의 규칙은 **위반 시 아키텍처가 깨진다**. 작업 중 이 중 하나를 건드려야 할 것 같으면, 코드를 수정하기 **전에** `[CONCERN] Severity: Critical` 을 보고하고 사용자 컨펌을 받아야 한다.

규칙을 보완하는 문서:
- [design-principles.md](design-principles.md) — ARCH 섹션의 근거 (robot-agnostic, 5 principles)
- [anti-patterns.md](anti-patterns.md) — 여기 invariant을 위반했던 실제 실수 사례
- [.claude/rules/rt-safety.md](../.claude/rules/rt-safety.md) — `rtc_controller_*` / `rtc_controllers` / `ur5e_hand_driver`에 자동 로드되는 스코프 stub (이 파일의 RT 섹션을 가리킴)

## RT Path Invariants

**RT path 정의**: 500 Hz 이상으로 실행되는 경로. 구체적으로 `RtControllerNode::ControlLoop()`, `RTControllerInterface::Compute()` / `SetDeviceTarget()` / `InitializeHoldPosition()` / `PublishNonRtSnapshot()` 내부 기본 tick, UDP receive 콜백, sensor/target 구독 콜백, `CheckTimeouts` 50 Hz 분기. **비-RT path**: `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` lifecycle 콜백, `DrainLog()` aux thread, 1 Hz aux 타이머 (`GetMpcSolveStats` 등), ROS 파라미터 콜백.

| # | 금지 패턴 | 이유 | 위반 탐지 | 복구 |
|---|----------|------|-----------|------|
| RT-1 | `new` / `malloc` / `push_back` / `emplace_back` / `resize` | Heap alloc은 100 µs+ jitter + priority inversion | `grep -nE '(\\bnew [A-Za-z_]\|malloc\\(\|\\.push_back\\(\|\\.emplace_back\\(\|\\.resize\\()' <RT file>` | `std::array`, 사전 할당된 fixed-size `Eigen::Matrix<fixed>` |
| RT-2 | `throw` / `catch` | `noexcept` 위반 = unwinding latency 비결정, process kill 리스크 | `grep -nE '(\\bthrow \|\\bcatch ?\\()' <RT file>` | Error code, `std::optional`, `std::expected` |
| RT-3 | 정기 tick에서 `RCLCPP_INFO/WARN/ERROR/DEBUG/FATAL` 직접 호출 | Blocking I/O (rosout queue / network) | `grep -nE 'RCLCPP_(INFO\|WARN\|ERROR\|DEBUG\|FATAL)\\(' <RT file>` | SPSC log buffer → `DrainLog()` aux thread ([rt_controller_node_estop.cpp](../rtc_controller_manager/src/rt_controller_node_estop.cpp) 참조) |
| RT-4 | `std::mutex::lock()` / `std::lock_guard` / `std::scoped_lock` | 우선순위 역전, blocking | `grep -nE '(lock_guard\|scoped_lock\|::lock\\(\\))' <RT file>` | `std::try_to_lock`, `SeqLock<T>`, SPSC, atomic |
| RT-5 | `auto` with Eigen expression | Expression template lazy-eval → aliasing 버그 (같은 메모리 r/w) | `grep -nE 'auto [^=]*=.*\\.(matrix\|transpose\|inverse\|adjoint\|block)\\(' <file>` | 명시 타입: `Eigen::MatrixXd M = ...` |
| RT-6 | Quaternion `lerp` / `nlerp` | Non-unit 결과 → 회전축 변형, drift | `grep -nE '(nlerp\|\\.lerp\\()' <file>` | `Eigen::Quaterniond::slerp(t, q_b)` only |
| RT-7 | 기존 테스트 assertion을 통과시키려 수정 | 회귀 은폐 | `git diff test/` 에서 `EXPECT_*` / `ASSERT_*` 값 변경 | 새 코드를 고쳐라. assertion이 진짜 틀렸다면 별도 commit으로 논증 |
| RT-8 | `std::shared_ptr` 복사 | Atomic ref-count contention | `grep -nE 'std::shared_ptr<' <RT file>` (값 인자/반환 검사) | Raw ref 또는 `const std::shared_ptr<T>&` |

### RT-3 세부 스펙

- **정기 tick 경로**: `RCLCPP_*` 직접 호출 금지. SPSC → aux로 defer. 500 Hz × 단 한 줄 블록 = 대형 지터 원인.
- **One-shot init 경로 (허용)**: `init_timeout` fatal, `auto-hold initialized` 최초 1회 등 — 1회 발생 후 `rclcpp::shutdown()` 또는 활성화 완료로 더 이상 실행되지 않는 분기. 현재 코드에 다수 존재하며 의도된 상태.
- **THROTTLE 변종 (허용, 단 msg 최적화)**: `RCLCPP_INFO_THROTTLE` / `RCLCPP_WARN_THROTTLE` 등 허용. 단 msg 내용은 RT-safe해야 함:
  - ✅ 단순 format string + 기본 타입 (`int`, `double`, `const char*`, fixed-size `std::array<char, N>::data()`)
  - ❌ 문자열 concat (`std::string + std::string`), `std::stringstream`, `fmt::format`, `std::to_string` — 내부적으로 heap alloc
  - 예: `RCLCPP_WARN_THROTTLE(logger, clock, 1000, "MPC p99=%.0f us", p99_us)` ✅

## Architecture Invariants

| # | 규칙 | 이유 | 위반 탐지 |
|---|------|------|-----------|
| ARCH-1 | `rtc_*` 패키지에 로봇 이름·joint 수·HW ID 하드코딩 금지 | robot-agnostic 훼손 ([design-principles.md](design-principles.md) §Generality) | `grep -rniE '(ur5e\|6.?dof\|10.?dof\|num.?joints = [0-9])' rtc_*/` |
| ARCH-2 | 의존성 그래프 상향 의존 금지 ([architecture.md](architecture.md#L84) 그래프 기준) | Cyclic dep / abstraction leak | `rtc_base/`가 `rtc_controllers/` include, `rtc_*/`가 `ur5e_*/` include 등 |
| ARCH-3 | Abstract interface 없이 두 번째 구체 구현 추가 금지 | 확장성 훼손 → 세 번째 impl에서 `#ifdef` 지옥 | 새 `.cpp`에 대응하는 pure-virtual base 부재 |
| ARCH-4 | `ur5e_*` 헤더가 `rtc_*` private 헤더 include 금지 | 경계 훼손, robot-specific leak | `grep -rn '#include "rtc_.*/src/' ur5e_*/` |

## Process Invariants

| # | 규칙 | 이유 |
|---|------|------|
| PROC-1 | 코드 변경 시 대응 문서·YAML·CMakeLists·package.xml 동기화 ([modification-guide.md](modification-guide.md) 6단계) | Drift 방지 — git log에서 반복 수정 커밋 다수 확인됨 |
| PROC-2 | 공개 API 변경 시 downstream 패키지 재빌드·재테스트 | ABI 호환성 |
| PROC-3 | `rtc_base` / `rtc_msgs` 변경 시 전체 빌드·전체 테스트 | 광범위 영향 — 20개 패키지 중 대부분 의존 |
| PROC-4 | E-STOP trigger는 idempotent (`compare_exchange_strong`) | 중복 트리거 안전성 |

## Numerical Invariants

| # | 규칙 | 이유 | 구현 위치 |
|---|------|------|-----------|
| NUM-1 | 특이점 근처: damped pseudoinverse 필수 (`damping` YAML 주입) | Unbounded magnification | ClikController, OSC |
| NUM-2 | `dt` near-zero guard | `1/dt` 발산 | 모든 trajectory generator |
| NUM-3 | Quaternion 정규화 매 곱 후 | Drift → non-unit | SE3 trajectory, orientation PD |
| NUM-4 | `trajectory_speed`: `std::max(1e-6, val)` 클램프 | IEEE 754 `1/0 = INF` → hang | [controller-safety-improvements.md](controller-safety-improvements.md) Phase 2 R-4; 45개 `1e-6` 상수 현재 존재 |

## 이 파일의 규칙을 건드려야 할 것 같을 때

1. 수정 **전** `[CONCERN]` 보고:
   ```
   [CONCERN] <한 줄 요약>
   Severity: Critical
   Detail: <어떤 invariant에 저촉되는가, 영향 범위, 대안 검토 결과>
   Alternative: <우회 안 1개 이상 — interface 추가, SPSC defer, aux thread 이동 등>
   ```
2. 사용자 컨펌 후 진행
3. "임시로 위반 → 나중에 정리"는 허용되지 않음. Warning 이상은 별도 리팩터 task로 분리
