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
- **Eigen**: pre-allocated buffers, `noalias()`, zero heap on 500 Hz. `auto`로 Eigen expression 받지 말 것 ([invariants.md](invariants.md) RT-5)
- **Lifecycle**: 5 C++ nodes는 `rclcpp_lifecycle::LifecycleNode`. Empty constructor; `on_configure` (Tier 1) / `on_activate` (Tier 2). Safety publishers은 standalone `rclcpp::create_publisher` 사용
- **ROS 2 API**: 명시 `rclcpp::QoS`, `MutuallyExclusiveCallbackGroup`, 범위 지정 `ParameterDescriptor`

## Logging

로그 한 줄에서 **어느 패키지** · **어느 컨트롤러**인지 즉시 읽히도록 logger 이름을 다음 규약으로 짓는다. (터미널에는 `[<logger>]: <msg>` 형태로 찍힘.)

| 계층 | Logger 이름 포맷 | 예시 |
|------|------------------|------|
| Node-owned (robot bringup의 lifecycle node) | `<exec_name>` (= ROS 노드 이름 = 실행 파일 이름) | `ur5e_rt_controller` |
| Library-level (agnostic base/framework) | `<full_package_name>` | `rtc_controller_interface` |
| Controller-level (구체 컨트롤러) | `<package>.<controller_key>` | `ur5e_bringup.demo_joint_controller`, `ur5e_bringup.demo_task_controller`, `ur5e_bringup.demo_wbc_controller` |

**구현 원칙:**
- ROS 노드 이름 = 실행 파일 이름 (예: `ur5e_rt_controller`). `node_->get_logger()`는 그 이름을 그대로 반환. `rtc_controller_manager`는 library-only (실행 파일 없음) — runtime identity는 robot-specific bringup이 소유. 자세한 원칙: [design-principles.md](design-principles.md)
- 컨트롤러 내부 로그는 `rclcpp::get_logger("<pkg>.<controller>")` 정적 logger를 멤버 캐시로 보유 (예: [ur5e_bringup/include/ur5e_bringup/bringup_logging.hpp](../ur5e_bringup/include/ur5e_bringup/bringup_logging.hpp))
- Base class (`RTControllerInterface`)에서 찍는 공통 로그는 `rclcpp::get_logger("rtc_controller_interface")` + 메시지 본문에 `[<controller_name>]` prefix 로 어느 컨트롤러에서 호출됐는지 표시
- 점(`.`) 하나만 허용. 패키지 prefix를 축약하지 말 것 (예: `bringup.demo_joint` 사용 금지 → `ur5e_bringup.demo_joint_controller`)

RT path logging 금지 규칙과 SPSC 우회 패턴은 [invariants.md](invariants.md) RT-3 참조.

## Documentation Requirements

- **Doxygen for public API**: `@brief`, `@param`, `@return`, `@note` on every public class/function
- **Math formulas**: LaTeX in Doxygen (`@f$..@f$`), paper reference (author, year, eq number), units + frame per parameter
- **FSM**: Document valid transitions, entry/exit conditions, timeout behaviors
- **Thread safety**: 공유 data member는 sync 메커니즘 명시 (SeqLock, SPSC, atomic, mutex)

## Commit Message Conventions

Follow **Conventional Commits**: `type(scope): subject` + optional body + optional footer.

**Types**: `feat` | `fix` | `docs` | `style` | `refactor` | `perf` | `test` | `chore`

**Scope**: package name (`rtc_base`, `ur5e_bringup`, ...) or broad tag (`multi-pkg`, `launch`, `isolation`) when the change spans packages. One scope per commit — split unrelated changes.

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
