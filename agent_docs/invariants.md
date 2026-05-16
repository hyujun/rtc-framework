# Invariants

이 파일의 규칙은 **위반 시 아키텍처가 깨진다**. 작업 중 이 중 하나를 건드려야 할 것 같으면, 코드를 수정하기 **전에** `[CONCERN] Severity: Critical` 을 보고하고 사용자 컨펌을 받아야 한다.

규칙을 보완하는 문서:
- [design-principles.md](design-principles.md) — ARCH 섹션의 근거 (robot-agnostic, 5 principles)
- [anti-patterns.md](anti-patterns.md) — 여기 invariant을 위반했던 실제 실수 사례
- [.claude/rules/rt-safety.md](../.claude/rules/rt-safety.md) — `rtc_controller_*` / `rtc_controllers` / `udp_hand_driver`에 자동 로드되는 스코프 stub (이 파일의 RT 섹션을 가리킴)

## RT Path Invariants

**RT path 정의**: `control_rate` YAML 파라미터로 설정된 정기 tick에서 실행되는 모든 경로. 프레임워크는 rate-agnostic (설계 범위 100 Hz–5 kHz, default 500 Hz; 상수: `rtc::kMin/kMax/kDefaultControlRateHz`)으로, **"500 Hz"는 default 일 뿐 가정으로 박지 말 것** — RT 안전성은 *모든* 지원 rate에서 성립해야 한다. 구체적으로 `RtControllerNode::ControlLoop()`, `RTControllerInterface::Compute()` / `SetDeviceTarget()` / `InitializeHoldPosition()` / `PublishNonRtSnapshot()` 내부 기본 tick, UDP receive 콜백, sensor/target 구독 콜백, `CheckTimeouts` 50 Hz 분기. **비-RT path**: `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` lifecycle 콜백, `DrainLog()` aux thread, controller LifecycleNode의 1 Hz aux 타이머 (timing CSV drain 등), ROS 파라미터 콜백.

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

- **정기 tick 경로**: `RCLCPP_*` 직접 호출 금지. SPSC → aux로 defer. 정기 tick 주파수 × 단 한 줄 블록 = 대형 지터 원인 (예: default 500 Hz × 1줄 = 500 발생/초; 2 kHz면 4배 더 심각).
- **One-shot init 경로 (허용)**: `init_timeout` fatal, `auto-hold initialized` 최초 1회 등 — 1회 발생 후 `rclcpp::shutdown()` 또는 활성화 완료로 더 이상 실행되지 않는 분기. 현재 코드에 다수 존재하며 의도된 상태.
- **THROTTLE 변종 (허용, 단 msg 최적화)**: `RCLCPP_INFO_THROTTLE` / `RCLCPP_WARN_THROTTLE` 등 허용. 단 msg 내용은 RT-safe해야 함:
  - ✅ 단순 format string + 기본 타입 (`int`, `double`, `const char*`, fixed-size `std::array<char, N>::data()`)
  - ❌ 문자열 concat (`std::string + std::string`), `std::stringstream`, `fmt::format`, `std::to_string` — 내부적으로 heap alloc
  - 예: `RCLCPP_WARN_THROTTLE(logger, clock, 1000, "MPC p99=%.0f us", p99_us)` ✅

## Architecture Invariants

| # | 규칙 | 이유 | 위반 탐지 |
|---|------|------|-----------|
| ARCH-1 | `rtc_*` 패키지에 로봇 이름·joint 수·HW ID 하드코딩 금지 | robot-agnostic 훼손 ([design-principles.md](design-principles.md) §Generality) | `grep -rniE '(ur5e\|6.?dof\|10.?dof\|num.?joints = [0-9])' rtc_*/` |
| ARCH-2 | 의존성 그래프 상향 의존 금지 ([architecture.md](architecture.md) §Dependency Graph) | Cyclic dep / abstraction leak | `rtc_base/`가 `rtc_controllers/` include, `rtc_*/`가 `ur5e_*/` include 등 |
| ARCH-3 | Abstract interface 없이 두 번째 구체 구현 추가 금지 | 확장성 훼손 → 세 번째 impl에서 `#ifdef` 지옥 | 새 `.cpp`에 대응하는 pure-virtual base 부재 |
| ARCH-4 | `ur5e_*` 헤더가 `rtc_*` private 헤더 include 금지 | 경계 훼손, robot-specific leak | `grep -rn '#include "rtc_.*/src/' ur5e_*/` |
| ARCH-5 | `robot_descriptions`는 data-only — build-time 의존 금지 | 빌드 토폴로지 부담 + "share만 있으면 OK" 모델 훼손 | `grep -rn 'find_package(robot_descriptions\|ament_target_dependencies.*robot_descriptions' --include=CMakeLists.txt .` 그리고 `grep -rn '<depend>robot_descriptions</depend>\|<build_depend>robot_descriptions' --include=package.xml .` |

### ARCH-5 세부 스펙

`robot_descriptions`는 C++ target / 헤더 / 라이브러리 export가 0건인 data-only 패키지다 ([robot_descriptions/CMakeLists.txt](../robot_descriptions/CMakeLists.txt)는 `install(DIRECTORY robots/)` 한 줄뿐). 소비 패키지는 다음만 사용한다:

**허용**:
- `package.xml`: `<exec_depend>robot_descriptions</exec_depend>`
- C++: `ament_index_cpp::get_package_share_directory("robot_descriptions")`
- Python: `ament_index_python.packages.get_package_share_directory("robot_descriptions")`
- URDF/MJCF/launch/YAML: `package://robot_descriptions/robots/<name>/...` URL, 또는 패키지명 문자열 (rtc_controller_manager 가 런타임 resolve — `rtc_controller_manager/src/rt_controller_node_params.cpp` 참조)

**금지**:
- `find_package(robot_descriptions ...)` (CMakeLists.txt)
- `<depend>` / `<build_depend>` (package.xml)
- `ament_target_dependencies(... robot_descriptions)`
- `ament_export_dependencies(... robot_descriptions)`

**근거**: 빌드 시점에 link 할 artifact 가 0개이므로 build-dep 효과는 0. 그러나 build-dep 을 걸면 colcon 이 강제 토폴로지 엣지를 만들어 "이 디렉토리를 워크스페이스 어디 두든 — 형제 디렉토리든 별도 overlay 든 — `install/robot_descriptions/share/` 만 있으면 동작" 모델이 깨진다 (사용자 정책).

**복구**: build-dep 줄 제거 + `<exec_depend>` 로 강등. 일반적으로 코드 수정 0 줄.

**예외**: 미래에 `robot_descriptions`가 진짜 C++ 라이브러리를 export하게 되면 별도 패키지 (`robot_descriptions_utils` 등)로 split — 이 invariant는 그대로 유지.

## Process Invariants

| # | 규칙 | 이유 |
|---|------|------|
| PROC-1 | 코드 변경 시 대응 문서·YAML·CMakeLists·package.xml 동기화 ([modification-guide.md](modification-guide.md) 6단계) | Drift 방지 — git log에서 반복 수정 커밋 다수 확인됨 |
| PROC-2 | 공개 API 변경 시 downstream 패키지 재빌드·재테스트 | ABI 호환성 |
| PROC-3 | `rtc_base` / `rtc_msgs` 변경 시 전체 빌드·전체 테스트 | 광범위 영향 — 대부분 패키지가 의존 |
| PROC-4 | E-STOP trigger는 idempotent (`compare_exchange_strong`) | 중복 트리거 안전성 |

## Numerical Invariants

| # | 규칙 | 이유 | 구현 위치 |
|---|------|------|-----------|
| NUM-1 | 특이점 근처: damped pseudoinverse 필수 (`damping` YAML 주입) | Unbounded magnification | ClikController, OSC |
| NUM-2 | `dt` near-zero guard | `1/dt` 발산 | 모든 trajectory generator |
| NUM-3 | Quaternion 정규화 매 곱 후 | Drift → non-unit | SE3 trajectory, orientation PD |
| NUM-4 | `trajectory_speed`: `std::max(1e-6, val)` 클램프 | IEEE 754 `1/0 = INF` → hang | [archive/controller-safety-improvements.md](archive/controller-safety-improvements.md) Phase 2 R-4 |

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

## False-positive 처리

위 grep 명령은 **path-blind**(RT path 외 코드도 매칭)이거나 **role-blind**(one-shot init / aux thread 허용 케이스도 매칭)이다. 정당한 사용을 invariant로 잘못 차단하면 [CLAUDE.md](../CLAUDE.md) §11 *Harness pruning 신호*에 해당.

판단 절차:

1. **Path 확인**: 매칭된 파일이 RT path 정의(이 문서 §RT Path Invariants 첫 단락)에 들어가는가? 아니면 비-RT path (lifecycle 콜백 / `DrainLog()` aux / 1Hz aux 타이머 / 파라미터 콜백)인가?
2. **Role 확인** (RT-3 한정): one-shot init? `RCLCPP_*_THROTTLE` + RT-safe msg? 둘 중 하나면 **허용** ([RT-3 세부 스펙](#rt-3-세부-스펙) 참조).
3. **Aliasing 확인** (RT-5 한정): `auto`가 받는 게 단순 scalar/index인가, 아니면 Eigen expression (`.matrix()`, `.transpose()`, `.inverse()`, `.adjoint()`, `.block()`, `*` 연산)인가? Scalar/index는 false-positive.

False-positive 판정이면:
- 코드는 그대로 진행
- 한 줄 보고: `false-positive: <rule-id> at <file:line>, reason=<RT path 외 / one-shot / scalar auto / ...>`
- 동일 패턴이 반복 false-positive로 보고되면 [anti-patterns.md](anti-patterns.md) 또는 본 문서의 grep 명령을 좁히는 별도 task 후보

**금지**: false-positive 추정이라며 사용자 보고 없이 invariant 우회. 의심스러우면 §"이 파일의 규칙을 건드려야 할 것 같을 때" 의 `[CONCERN]` 절차를 따른다.
