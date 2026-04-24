# Conventions

## Domain Conventions

These apply to all `rtc_*` packages. `ur5e_*` inherit these and may add hardware-specific constraints.

- **Coordinate frame**: Right-hand rule, ZYX Euler (roll-pitch-yaw)
- **Rotation**: Internal = quaternion (`Eigen::Quaterniond`, Hamilton). Euler only at API boundaries
- **Quaternion interpolation**: `slerp` only -- `lerp`/`nlerp` prohibited
- **Units**: SI base (m, rad, s, kg, N). Degree inputs must convert to radians explicitly
- **Jacobian**: Body Jacobian by default. Spatial requires `_spatial` suffix
- **Dynamics**: M(q)q.. + C(q,q.)q. + g(q) = tau. Pinocchio RNEA-based
- **Variable naming**: Paper notation -- `J_b` (body Jacobian), `q_d` (desired joint), `x_e` (EE pose), `K_d` (stiffness)
- **Singularity**: Damped pseudoinverse required (`damping` via YAML), near-zero division protection mandatory

## Code Conventions

- **Namespace**: `rtc` (all packages)
- **Naming**: Google C++ -- `snake_case` members with `_` suffix, `PascalCase` types, `kConstant`
- **C++20**: `jthread`/`stop_token`, `std::span`, `string_view`, concepts, `[[likely]]/[[unlikely]]`, `constexpr`, structured bindings, `optional`/`expected`
- **RAII**: All resource acquisition via RAII. Raw `new`/`delete` prohibited
- **`noexcept`** on all RT paths
- **`[[nodiscard]]`** on status-returning functions; **`static_assert`** on template params
- **Include order**: project -> ROS 2 / third-party -> C++ stdlib (alphabetically)
- **Eigen**: pre-allocated buffers, `noalias()`, zero heap on 500Hz. Never `auto` for Eigen expressions
- **Lifecycle**: All 5 C++ nodes use `rclcpp_lifecycle::LifecycleNode`. Empty constructor; `on_configure` (Tier 1) / `on_activate` (Tier 2). Safety publishers use standalone `rclcpp::create_publisher`
- **ROS 2 API**: Explicit `rclcpp::QoS`, `MutuallyExclusiveCallbackGroup`, `ParameterDescriptor` with ranges

## Documentation Requirements

- **Doxygen for public API**: `@brief`, `@param`, `@return`, `@note` on every public class/function
- **Math formulas**: LaTeX in Doxygen (`@f$..@f$`), paper reference (author, year, eq number), units and frame per parameter
- **FSM**: Document valid transitions, entry/exit conditions, timeout behaviors
- **Thread safety**: Every shared data member documents its sync mechanism (SeqLock, SPSC, atomic, mutex)

## Critical Thinking Protocol

Before writing code, self-check:

**Safety**: Could this compromise RT determinism (500Hz)? Safe under singularity, joint limit, comms loss, E-STOP? Numerically stable?

**Design**: Consistent with existing patterns (Strategy controllers, SPSC offload, SeqLock)? Simpler alternative? For `rtc_*`: satisfies all 5 design principles? Any robot-specific assumption leaking in = Critical concern.

**Math**: Frames (right-hand), units (SI), rotation (Hamilton quat, ZYX Euler) correct? Jacobian/mass matrix dimensions consistent?

**Performance**: Completes within 2ms @ 500Hz? Unnecessary copies, redundant computation, heap allocations?

Report concerns as: `[CONCERN] summary / Severity: Critical|Warning|Info / Detail / Alternative`
- **Critical** (RT safety, numerical instability): Do NOT proceed without user confirmation
- **Warning** (performance, design inconsistency): Raise but follow user's decision

## Edge Case Audit

Before declaring implementation complete, verify handling of:
- **Singularity**: damped pseudoinverse with configurable `damping`
- **Joint limits**: clamp or null-space repulsion, never ignore
- **Communication loss**: E-STOP trigger, not silent stale-data
- **Array bounds**: device/channel index bounds check or `static_assert`
- **Numerical precision**: quaternion normalization, rotation orthogonality, `dt` near zero guard
- **E-STOP recovery**: re-initialize hold position, not resume from stale target
- **Empty/zero input**: graceful no-op or explicit rejection
- **Thread safety**: new shared state -> SeqLock, SPSC, or atomic; document mechanism

## Commit Message Conventions

Follow **Conventional Commits**: `type(scope): subject` + optional body + optional footer.

**Types**: `feat` | `fix` | `docs` | `style` | `refactor` | `perf` | `test` | `chore`

**Scope**: package name (`rtc_base`, `ur5e_bringup`, ...) or broad tag (`multi-pkg`, `launch`, `isolation`) when the change spans packages. One scope per commit -- split unrelated changes.

**Subject**: English, imperative mood, capitalized first letter, no trailing period, <= 50 chars.

**Body / footer** (optional, blank line before each, wrap at 72):
- Explain *what* and *why*, not *how*. Omit when the subject is self-explanatory.
- Reference issues as `Closes: #N` / `Refs: #N` in the footer.
- PR merge commits keep GitHub's default `Merge pull request #N from ...` -- do not rewrite.

Example with body + footer:
```
feat(multi-pkg): Surface MPC solve failures via throttled stderr

Previously a failed solve was silently dropped, masking convergence
regressions during long runs. Emit a count=0 sentinel plus a
rate-limited warning so operators see repeated failures without
flooding the console.

Closes: #92
```

## Code Quality Checks

- No placeholder stubs (`// TODO: implement`) unless explicitly agreed
- No hallucinated APIs -- read source first when uncertain
- No generic names (`data`, `result`) where domain names exist (`joint_positions`, `jacobian_body`)
- No redundant comments restating obvious code
- No copy-paste drift -- verify each instance uses correct index/name
- Never modify existing test assertions to pass -- fix new code instead
- Never do optimistic hedging ("it will probably work") or silently reduce scope
