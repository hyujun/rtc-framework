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
