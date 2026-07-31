# Conventions

**이 파일은 스타일 가이드다.**
- 위반 시 escalation 대상인 규칙(RT / ARCH / PROC / NUM) → [invariants.md](invariants.md)
- 재발성 실수 패턴과 탐지/복구 → [anti-patterns.md](anti-patterns.md)
- `[CONCERN]` 포맷과 Critical Thinking 자가 점검 → [../CLAUDE.md](../CLAUDE.md) §6 Escalation

## Domain Conventions

`rtc_*` 패키지 전체에 적용. integration 패키지 (`integrated_bringup`, `udp_hand_driver`, `shape_estimation`, `ur5e_bt_coordinator` 등) 는 이를 상속하고 하드웨어 제약을 추가할 수 있다.

- **Coordinate frame**: Right-hand rule, ZYX Euler (roll-pitch-yaw)
- **Rotation**: Internal = quaternion (`Eigen::Quaterniond`, Hamilton). Euler only at API boundaries
- **Quaternion interpolation**: `slerp` only — `lerp`/`nlerp` 금지 ([invariants.md](invariants.md) RT-6)
- **Units**: SI base (m, rad, s, kg, N). Degree inputs는 radian으로 명시 변환
- **Jacobian**: Body Jacobian 기본. Spatial은 `_spatial` suffix
- **Dynamics**: $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$. Pinocchio RNEA 기반
- **Variable naming**: Paper notation — `J_b` (body Jacobian), `q_d` (desired joint), `x_e` (EE pose), `K_d` (stiffness)
<!-- validate-docs: allow D10 -->
> **`§N.M` 표기 규칙** — 숫자 절번호는 세 네임스페이스에 쓰인다: 헌법 절번호(`CLAUDE.md §6`처럼 파일명과 함께), compliance 규범 명세(`compliance §6.5`처럼 접두사 필수 — [rtc_controllers/docs/compliance-conventions.md](../rtc_controllers/docs/compliance-conventions.md) 가 참조하는 원명세), 그리고 파일 내부 명명 섹션(`§RT Path Invariants`). 접두사 없는 맨 `§6.5` 는 쓰지 않는다.

- **Singularity**: compliance §6.5 σ_min-적응형 damped pseudoinverse (`max_damping` = λ_max, `singularity_threshold` = σ₀ via YAML — 다섯 task-space 컨트롤러 공통 키), near-zero division guard ([invariants.md](invariants.md) NUM-1)

## Code Conventions

- **Namespace**: `rtc` (all packages)
- **Naming**: Google C++ — `PascalCase` methods/types/free functions, `snake_case_` private members, `snake_case` local variables/parameters, `kConstant` for constants. `.clang-tidy` 의 `readability-identifier-naming.MethodCase: CamelCase` 가 기계 SSoT.
- **C++20**: `jthread` / `stop_token`, `std::span`, `string_view`, concepts, `[[likely]]/[[unlikely]]`, `constexpr`, structured bindings, `optional` / `expected`
- **RAII**: 모든 리소스 획득은 RAII. Raw `new` / `delete` 금지
- **`noexcept`** on all RT paths
- **`[[nodiscard]]`** on status-returning functions; **`static_assert`** on template params
- **Include order**: project → ROS 2 / third-party → C++ stdlib (alphabetical)
- **Method split 은 consumer 이름으로**: 한 method 가 여러 downstream (RT wire / CSV log / ROS publish ...) 을 동시에 채우고 있어 쪼갤 때는, 각 조각을 그것이 섬기는 **consumer** 이름으로 짓는다 — `WriteJointCommand()` / `FillLogOutput()` / `FillPublishOutput()`. Stage·순서 이름 (`Phase3a`, `Step2`) 이나 데이터 source 이름 금지: stage 명은 body 의 목적을 감춰 "이 method 가 왜 존재하나" 를 다른 문서에서 찾게 만들지만, consumer-named method 아래의 모든 라인은 그 consumer 를 위한 것임이 자명하다. 두 consumer 가 같은 필드를 읽으면 공유 helper 로 빼지 말고 **양쪽에서 독립적으로 write** 한다 (공유 추출은 ordering drift 를 부르며, 그 비용이 중복 write 비용보다 크다). RT / wire-bound method 를 가장 앞에 가장 짧게 두고, header 에 각 출력 필드의 consumer 를 밝히는 한 줄 주석을 단다.
- **Eigen**: pre-allocated buffers, `noalias()`, zero heap on the RT path (any configured `control_rate`). `auto`로 Eigen expression 받지 말 것 ([invariants.md](invariants.md) RT-5)
- **Lifecycle**: 핵심 C++ 노드는 `rclcpp_lifecycle::LifecycleNode`. Empty constructor; `on_configure` (Tier 1) / `on_activate` (Tier 2). Safety publishers은 standalone `rclcpp::create_publisher` 사용. 어떤 노드가 LifecycleNode 인지는 [architecture.md](architecture.md) 참조 — 개수 박제 금지 ([anti-patterns.md](anti-patterns.md) AP-DOC-1)
- **ROS 2 API**: 명시 `rclcpp::QoS` (모든 topic 은 `KEEP_LAST` depth **1** — [invariants.md](invariants.md) ARCH-6; reliability/durability 는 lane별 유지), `MutuallyExclusiveCallbackGroup`, 범위 지정 `ParameterDescriptor`
- **Formatting SSoT**: C++ 는 [`.clang-format`](../.clang-format) (Google base, ColumnLimit 100, PointerAlignment Left), Python 은 [`pyproject.toml`](../pyproject.toml) (ruff, line 99). PostToolUse hook ([.claude/hooks/format-code.sh](../.claude/hooks/format-code.sh)) 이 모든 Edit 마다 자동 적용. **`ament_uncrustify` / `ament_lint_common` 사용 금지** — uncrustify 의 ROS 2 / Eclipse-CDT 표준 스타일 (Allman braces, `T *p`, `T &r`) 이 clang-format Google base 와 영구 충돌. `set(ament_cmake_uncrustify_FOUND TRUE)` 스킵 트릭은 작동하지 않는다 (extras hook `ament_cmake_uncrustify_lint_hook.cmake` 가 무조건 등록됨) — `ament_lint_common` meta 자체를 쓰지 말 것. 새 패키지는 개별 lint depend 만: `<test_depend>ament_cmake_{cppcheck,lint_cmake,xmllint}</test_depend>` + CMakeLists 에 `find_package(...)` + `ament_cppcheck()` / `ament_lint_cmake()` / `ament_xmllint()` 개별 호출. 기존 패키지에서 uncrustify 를 떼낸 직후엔 `build/<pkg>/test_results/<pkg>/uncrustify.xunit.xml` 이 남아있을 수 있으므로 `rm -rf build/<pkg>/test_results` 후 colcon test 로 검증.
- **Include grouping**: `.clang-format` `IncludeBlocks: Regroup` + `IncludeCategories` 가 4 Priority 로 분류 (SSoT 는 `.clang-format`):
    - **P1** 본 저장소 프로젝트 헤더 — `"rtc_*"`/`"shape_estimation*"`/`"integrated_bringup*"`/`"ur5e_bt_coordinator*"`/`"udp_hand_driver*"`/`"robot_descriptions*"` (quote/angle 둘 다)
    - **P2** ROS 2 — `<rclcpp/*>`, `<std_msgs/*>` 등
    - **P3** third-party — Eigen, pinocchio, fmt, MuJoCo, BehaviorTree 등
    - **P4** stdlib — `<algorithm>` 등
  - 같은 Priority 안의 blank line 은 clang-format 이 강제 collapse 한다. own header (`"foo.hpp"`) 와 `<rtc_base/...>` 가 같은 P1 이라 둘 사이 빈 줄 제거는 **의도된 동작** — `/code-review` 가 "include grouping 부족" 으로 지적하면 거절 정당.
  - 분리가 필요하면 `.clang-format` 패턴 추가가 정답이지 manual blank line 이 아니다.
  - **예외**: `#ifdef ... #endif` 사이에 낀 include 는 clang-format 이 grouping 못 해 blank line 이 살아 있다.

## Logging

로그 한 줄에서 **어느 패키지** · **어느 컨트롤러**인지 즉시 읽히도록 logger 이름을 다음 규약으로 짓는다. (터미널에는 `[<logger>]: <msg>` 형태로 찍힘.)

| 계층 | Logger 이름 포맷 | 예시 |
|------|------------------|------|
| Node-owned (robot bringup의 lifecycle node) | `<exec_name>` (= ROS 노드 이름 = 실행 파일 이름) | `integrated_rt_controller` |
| Library-level (agnostic base/framework) | `<full_package_name>` | `rtc_controller_interface` |
| Controller-level (구체 컨트롤러) | `<package>.<controller_key>` | `integrated_bringup.demo_joint_controller`, `integrated_bringup.demo_task_controller`, `integrated_bringup.demo_wbc_controller` |

**구현 원칙:**
- ROS 노드 이름 = 실행 파일 이름 (예: `integrated_rt_controller`). `node_->get_logger()`는 그 이름을 그대로 반환. `rtc_controller_manager`는 library-only (실행 파일 없음) — runtime identity는 robot-specific bringup이 소유. 자세한 원칙: [design-principles.md](design-principles.md)
- 컨트롤러 내부 로그는 `rclcpp::get_logger("<pkg>.<controller>")` 정적 logger를 멤버 캐시로 보유 (예: [integrated_bringup/include/integrated_bringup/support/bringup_logging.hpp](../integrated_bringup/include/integrated_bringup/support/bringup_logging.hpp))
- Base class (`RTControllerInterface`)에서 찍는 공통 로그는 `rclcpp::get_logger("rtc_controller_interface")` + 메시지 본문에 `[<controller_name>]` prefix 로 어느 컨트롤러에서 호출됐는지 표시
- 점(`.`) 하나만 허용. 패키지 prefix를 축약하지 말 것 (예: `bringup.demo_joint` 사용 금지 → `integrated_bringup.demo_joint_controller`)

RT path logging 금지 규칙과 SPSC 우회 패턴은 [invariants.md](invariants.md) RT-3 참조.

## Controller-owned CSV logging (`logs:` schema)

데이터 CSV (per-tick state / sensor / inference 등) 는 controller 가 직접 소유한다. CM 은 logging authority 가 아니다 (Phase C 결정). 컨트롤러 YAML 에 `topics:` sibling 으로 `logs:` 섹션을 두고, 각 항목은 `rtc_msgs/<*Log>` 메시지 타입을 schema 키로 사용한다.

```yaml
<config_key>:
  topics:
    ...
  logs:
    - msg_type: rtc_msgs/DeviceStateLog
      instance: ur5e_state    # 같은 msg_type 이 여러 번 등장할 때 disambiguator
    - msg_type: rtc_msgs/DeviceStateLog
      instance: hand_state    # state vs sensor 가 같은 device 에서 둘 다 있을 땐
    - msg_type: rtc_msgs/DeviceSensorLog
      instance: hand_sensor   # instance 를 분리해 CSV 파일 충돌 방지 (Q-MSG-3 / Option A)
```

규칙:

- `msg_type` (필수): `rtc_msgs/<*Log>` 형식 (integrated_bringup-private POD 로그 — `DeviceWbcLog`/`WbcDiagLog`/`PullEstimatorLog` — 은 `integrated_bringup/<*Log>`). 컨트롤러 `LoadConfig` 의 closed-set 검증에 매핑되어 오타는 parse 시 hard fail
- `instance` (필수): 컨트롤러 코드의 `RegisterLog<MsgT>("<instance>", ...)` 호출과 1:1 매칭. 빈 `instance` 또는 같은 LogSet 내 중복 등록은 hard fail (`RegisterLog` unbound handle); 코드가 등록하지 않은 instance (optional·gated 채널이 비활성일 때, 예: `pull_estimator` wiring disabled) 는 silently skip. CSV 파일 stem 으로도 사용됨 → `<session>/controllers/<config_key>/<instance>.csv`. **instance 는 `(msg_type, instance)` 가 unique 하도록 작성하지 말고, *경로 단위로 unique* 하도록 작성한다** — 같은 device 의 state/sensor 가 있을 때는 `hand_state` / `hand_sensor` 로 분리. 같은 LogSet 안에서 동일 instance 를 두 번 등록하면 `RegisterLog` 가 unbound handle 을 반환한다 (코드 enforcement)
- `topic` (옵션, 현재는 unused): 향후 `rtc_msgs/*Log` 를 DDS 로 publish 하는 옵션을 위해 예약. Q-MSG-1(a) lock 으로 현재는 schema-only POD→SPSC→CSV
- POD 미러 정의 위치 (Q-MSG-2(d)): `<robot>_bringup/include/<robot>_bringup/logging/<msg>_pod.hpp`. `kMaxJoints` 등 capacity 는 *그 robot 의 hardware* 에 맞게 선정 — `rtc_base` 에 robot constant 금지 (ARCH-1)
- Push site 제약 (Q-ACTIVITY-GATING): controller 는 **`Compute()` 에서만** push 한다. parameter callback / BT bridge / 비-RT thread 에서 push 금지 — inactive controller 의 CSV 에 row 가 쌓이는 것을 방지
- Timestamp (Q-TIME): 첫 numeric column 은 `state.t_relative_s` (CM RT loop 가 `t0 - log_start_time_` 으로 채움). controller 는 `chrono::*::now()` 호출 금지

세부 결정 / 구현 phase 는 `~/.claude/plans/csv-logging-cleanup.md` (Phase C handoff) 참조.

## Documentation Requirements

- **Doxygen for public API**: `@brief`, `@param`, `@return`, `@note` on every public class/function
- **Math formulas**: LaTeX in Doxygen (`@f$..@f$`), paper reference (author, year, eq number), units + frame per parameter
- **FSM**: Document valid transitions, entry/exit conditions, timeout behaviors
- **Thread safety**: 공유 data member는 sync 메커니즘 명시 (SeqLock, SPSC, atomic, mutex)
- **서비스·메시지 계약의 소유자** (AP-DOC-1 적용): 한 서비스는 최대 **네 곳**에서 설명될 수 있다 — `.srv`/`.msg` 파일 · 구현 `.cpp` 파일 헤더 주석 · `rtc_msgs/README.md` · 소비 패키지 README. 같은 문장을 네 번 쓰지 말고 **축을 나눈다**:
  - **`.srv` / `.msg` 주석 = wire 계약 SSoT** — 필드 의미, 응답 필드의 판정 기준, 거부 조건 분류, 권한(무엇을 확인해야 호출이 성립하는가). *호출자가 알아야 하는 전부*가 여기 있고, 다른 곳은 이걸 재서술하지 않는다
  - **소비 패키지 README = 그 패키지 쪽 메커니즘** — 왜 그 수치인지의 산술, 내부 카운터·경합 차단 같은 구현 근거. 계약은 `.srv` 를 링크
  - **`rtc_msgs/README.md` = 색인** — request/response 필드 표 + "상세 계약은 `srv/X.srv` 가 SSoT" 한 줄. 근거 산문 금지
  - **`.cpp` 파일 헤더 = 구현 선택** — 왜 이렇게 구현했는지. 계약 재서술 금지, `.srv` 로 위임
  실제 위반: `/rtc_cm/clear_estop` 의 관측 창 산술이 네 곳에 동시에 존재했다 (#288 직후 정리)

## Commit Message Conventions

Follow **Conventional Commits**: `type(scope): subject` + optional body + optional footer.

**Types**: `feat` | `fix` | `docs` | `style` | `refactor` | `perf` | `test` | `chore`

**Scope**: package name (`rtc_base`, `integrated_bringup`, ...) or broad tag (`multi-pkg`, `launch`, `isolation`) when the change spans packages. One scope per commit — split unrelated changes.

**Subject**: English, imperative mood, capitalized first letter, no trailing period, <= 50 chars.

**Body / footer** (optional, blank line before each, wrap at 72):
- Explain *what* and *why*, not *how*. Omit when the subject is self-explanatory.
- Reference issues as `Closes: #N` / `Refs: #N` in the footer.
- PR merge commits keep GitHub's default `Merge pull request #N from ...` — do not rewrite.

Example with body + footer:
```
feat(multi-pkg): Surface MPC solve failures via throttled stderr

Previously a failed solve was silently dropped, masking convergence
regressions during long runs. Emit a count=0 sentinel plus a
rate-limited warning so operators see repeated failures without
flooding the console.

Closes: #92
```
