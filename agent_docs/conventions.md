# Conventions

**이 파일은 스타일 가이드다.**
- 위반 시 escalation 대상인 규칙(RT / ARCH / PROC / NUM) → [invariants.md](invariants.md)
- 재발성 실수 패턴과 탐지/복구 → [anti-patterns.md](anti-patterns.md)
- `[CONCERN]` 포맷과 Critical Thinking 자가 점검 → [../CLAUDE.md](../CLAUDE.md) §6 Escalation

## Domain Conventions

`rtc_*` 패키지 전체에 적용. `ur5e_*` 는 이를 상속하고 하드웨어 제약을 추가할 수 있다.

- **Coordinate frame**: Right-hand rule, ZYX Euler (roll-pitch-yaw)
- **Rotation**: Internal = quaternion (`Eigen::Quaterniond`, Hamilton). Euler only at API boundaries
- **Quaternion interpolation**: `slerp` only — `lerp`/`nlerp` 금지 ([invariants.md](invariants.md) RT-6)
- **Units**: SI base (m, rad, s, kg, N). Degree inputs는 radian으로 명시 변환
- **Jacobian**: Body Jacobian 기본. Spatial은 `_spatial` suffix
- **Dynamics**: $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$. Pinocchio RNEA 기반
- **Variable naming**: Paper notation — `J_b` (body Jacobian), `q_d` (desired joint), `x_e` (EE pose), `K_d` (stiffness)
- **Singularity**: Damped pseudoinverse (`damping` via YAML), near-zero division guard ([invariants.md](invariants.md) NUM-1)

## Code Conventions

- **Namespace**: `rtc` (all packages)
- **Naming**: Google C++ — `snake_case_` members, `PascalCase` types, `kConstant`
- **C++20**: `jthread` / `stop_token`, `std::span`, `string_view`, concepts, `[[likely]]/[[unlikely]]`, `constexpr`, structured bindings, `optional` / `expected`
- **RAII**: 모든 리소스 획득은 RAII. Raw `new` / `delete` 금지
- **`noexcept`** on all RT paths
- **`[[nodiscard]]`** on status-returning functions; **`static_assert`** on template params
- **Include order**: project → ROS 2 / third-party → C++ stdlib (alphabetical)
- **Eigen**: pre-allocated buffers, `noalias()`, zero heap on the RT path (any configured `control_rate`). `auto`로 Eigen expression 받지 말 것 ([invariants.md](invariants.md) RT-5)
- **Lifecycle**: 5 C++ nodes는 `rclcpp_lifecycle::LifecycleNode`. Empty constructor; `on_configure` (Tier 1) / `on_activate` (Tier 2). Safety publishers은 standalone `rclcpp::create_publisher` 사용
- **ROS 2 API**: 명시 `rclcpp::QoS`, `MutuallyExclusiveCallbackGroup`, 범위 지정 `ParameterDescriptor`
- **Formatting SSoT**: C++ 는 [`.clang-format`](../.clang-format) (Google base, ColumnLimit 100, PointerAlignment Left), Python 은 [`pyproject.toml`](../pyproject.toml) (ruff, line 99). PostToolUse hook ([.claude/hooks/format-code.sh](../.claude/hooks/format-code.sh)) 이 모든 Edit 마다 자동 적용. **`ament_uncrustify` enable 금지** — ROS 2 / Eclipse-CDT 표준 스타일 (Allman braces, `T *p`, `T &r`, `template <T>`) 이 Google base 와 호환 불가능하므로 두 lint 가 서로 영원히 충돌한다. 새 패키지 CMakeLists 에서 `ament_lint_auto` 를 사용한다면 `set(ament_cmake_uncrustify_FOUND TRUE)` 로 skip 필수. cppcheck / cpplint / xmllint / lint_cmake 는 가치 있으므로 유지.

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

- `msg_type` (필수): `rtc_msgs/<*Log>` 형식. 메시지 카탈로그 enum 으로 매핑되어 오타는 on_configure 시 hard fail
- `instance` (필수): 컨트롤러 코드의 `RegisterLog<MsgT>("<instance>", ...)` 호출과 1:1 매칭. 불일치는 hard fail. CSV 파일 stem 으로도 사용됨 → `<session>/controllers/<config_key>/<instance>.csv`. **instance 는 `(msg_type, instance)` 가 unique 하도록 작성하지 말고, *경로 단위로 unique* 하도록 작성한다** — 같은 device 의 state/sensor 가 있을 때는 `hand_state` / `hand_sensor` 로 분리. 같은 LogSet 안에서 동일 instance 를 두 번 등록하면 `RegisterLog` 가 unbound handle 을 반환한다 (코드 enforcement)
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
