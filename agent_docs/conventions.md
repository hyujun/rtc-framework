# Conventions

**이 파일은 스타일 가이드다.** 위반 시 escalation 대상인 규칙 (RT / ARCH / PROC / NUM / AP) 과 `[CONCERN]` 포맷은 [invariants.md](invariants.md) 가 갖는다. 규약의 근거·예시·세부는 헌법 밖 [conventions-rationale.md](../docs/reference/conventions-rationale.md).

## Domain Conventions

`rtc_*` 패키지 전체에 적용한다. integration 패키지는 이를 상속하고 하드웨어 제약을 추가할 수 있다.

- **Coordinate frame**: Right-hand rule, ZYX Euler (roll-pitch-yaw)
- **Rotation**: Internal = quaternion (`Eigen::Quaterniond`, Hamilton). Euler only at API boundaries. 보간은 `slerp` only (RT-6)
- **Units**: SI base (m, rad, s, kg, N). Degree 입력은 radian 으로 명시 변환
- **Jacobian**: Body Jacobian 기본. Spatial 은 `_spatial` suffix
- **Dynamics**: $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$. Pinocchio RNEA 기반
- **Variable naming**: Paper notation — `J_b` (body Jacobian), `q_d` (desired joint), `x_e` (EE pose), `K_d` (stiffness)
- **Singularity**: compliance §6.5 σ_min-적응형 damped pseudoinverse (`max_damping` = λ_max, `singularity_threshold` = σ₀ via YAML), near-zero division guard (NUM-1)
<!-- validate-docs: allow D10 -->
- **`§N.M` 표기**: 숫자 절번호는 헌법 (`AGENTS.md §6` 처럼 파일명과 함께), compliance 명세 (`compliance §6.5` 처럼 접두사 필수), 파일 내부 명명 섹션 (`§RT Path Invariants`) 세 네임스페이스에 쓰인다. 접두사 없는 맨 번호는 쓰지 않는다.

## Code Conventions

- **Namespace**: `rtc` (all packages)
- **Naming**: Google C++ — `PascalCase` methods/types/free functions, `snake_case_` private members, `snake_case` local variables/parameters, `kConstant` for constants. 기계 SSoT 는 `.clang-tidy`
- **C++20**: `jthread` / `stop_token`, `std::span`, `string_view`, concepts, `[[likely]]/[[unlikely]]`, `constexpr`, structured bindings, `optional` / `expected`
- **RAII**: 모든 리소스 획득은 RAII. Raw `new` / `delete` 금지
- **`noexcept`** on all RT paths. **`[[nodiscard]]`** on status-returning functions. **`static_assert`** on template params
- **Include order**: project → ROS 2 / third-party → C++ stdlib (alphabetical)
- **Method split 은 consumer 이름으로**: 한 method 가 여러 downstream (RT wire / CSV log / ROS publish) 을 채우고 있어 쪼갤 때는 각 조각을 그것이 섬기는 consumer 이름으로 짓는다 (`WriteJointCommand()` / `FillLogOutput()` / `FillPublishOutput()`). stage · 순서 이름 (`Phase3a`, `Step2`) 이나 데이터 source 이름은 쓰지 않는다. 두 consumer 가 같은 필드를 읽으면 공유 helper 로 빼지 말고 양쪽에서 독립적으로 write 한다. RT / wire-bound method 를 가장 앞에 가장 짧게 둔다
- **Eigen**: pre-allocated buffers, `noalias()`, zero heap on the RT path. `auto` 로 Eigen expression 을 받지 않는다 (RT-5)
- **Lifecycle**: 핵심 C++ 노드는 `rclcpp_lifecycle::LifecycleNode`. Empty constructor; `on_configure` (Tier 1) / `on_activate` (Tier 2). Safety publisher 는 standalone `rclcpp::create_publisher`
    - **Tier 0 예외**: 어떤 lifecycle transition 보다 먼저 확정돼야 하는 것만 생성자에 둘 수 있다 — ① executor 에 넘길 callback group, ② 그 스레드를 어디에 pin 할지 정하는 파라미터 declare. 판정 기준은 "configure 가 오기 전에 이미 필요한가" 다. 리소스 (publisher / subscription / timer / 디바이스 핸들) 는 여전히 금지이고, 이 예외를 쓰는 노드는 생성자에 그 이유를 한 줄로 남긴다
- **ROS 2 API**: 명시 `rclcpp::QoS` (`KEEP_LAST` depth 1 — ARCH-6), `MutuallyExclusiveCallbackGroup`, 범위 지정 `ParameterDescriptor`
- **Formatting SSoT**: C++ 는 [`.clang-format`](../.clang-format), Python 은 [`pyproject.toml`](../pyproject.toml) (ruff). **`ament_uncrustify` / `ament_lint_common` 사용 금지** (clang-format 과 영구 충돌한다; meta 패키지를 쓰면 skip 할 수 없다). 새 패키지는 개별 lint depend 만: `ament_cmake_cppcheck` / `ament_cmake_lint_cmake` / `ament_cmake_xmllint`
- **Include grouping**: `.clang-format` 의 `IncludeCategories` 가 SSoT 다 (프로젝트 → ROS 2 → third-party → stdlib). 같은 Priority 안의 blank line 은 clang-format 이 지우며 그것이 의도된 동작이다. 분리가 필요하면 `.clang-format` 패턴을 추가한다 — manual blank line 이 아니다

## Logging

| 계층 | Logger 이름 포맷 | 예시 |
|------|------------------|------|
| Node-owned (robot bringup 의 lifecycle node) | `<exec_name>` (= ROS 노드 이름 = 실행 파일 이름) | `integrated_rt_controller` |
| Library-level (agnostic base/framework) | `<full_package_name>` | `rtc_controller_interface` |
| Controller-level (구체 컨트롤러) | `<package>.<controller_key>` | `integrated_bringup.demo_joint_controller` |

- 점 (`.`) 하나만 허용한다. 패키지 prefix 를 축약하지 않는다.
- 컨트롤러 내부 로그는 `rclcpp::get_logger("<pkg>.<controller>")` 정적 logger 를 멤버로 캐시한다. base class 의 공통 로그는 library logger + 메시지 본문의 `[<controller_name>]` prefix 로 호출자를 밝힌다.
- 메시지 본문에 클래스·노드 이름을 박지 않는다 — logger 이름이 식별자다. 짧은 기능 영역 태그 (`[grasp]`) 는 허용.

| 레벨 | 용도 |
|------|------|
| `FATAL` | 프로세스를 계속 실행할 수 없는 상태 |
| `ERROR` | 복구 불가능한 실패, 사용자 개입 필요 |
| `WARN` | 복구 가능한 실패/이상 상태, 자동 재시도 중 |
| `INFO` | 사용자가 알아야 할 1 Hz 미만 상태 전환 |
| `DEBUG` | 개발자 진단용 (기본 꺼짐) — 고빈도 경로는 전용 서브-로거로 격리 |

- 주기 경로 (RT tick · 고빈도 폴링 · 센서 콜백) 에서 반복될 수 있는 로그는 `*_THROTTLE` 로만 찍고, 주기는 그 패키지 logging 헤더의 표준 상수를 쓴다. RT path 의 금지 규칙은 RT-3.
- 핫패스 로그는 포맷 인자 수를 최소화하고 상세 데이터는 상태 구조체 · CSV 로 넘긴다. non-RT 경로의 `INFO` 는 grep 한 줄로 진단 가능하도록 풍부하게 쓴다.
- 서브-로거 계층은 런타임 필터링 단위다 (`/<node>/set_logger_levels`).
- 패키지 README 는 레벨 표를 복제하지 않는다.

## Controller-owned CSV logging (`logs:` schema)

데이터 CSV 는 controller 가 직접 소유한다 — CM 은 logging authority 가 아니다. 컨트롤러 YAML 에 `topics:` 의 sibling 으로 `logs:` 섹션을 두고 각 항목은 메시지 타입을 schema 키로 쓴다.

- `msg_type` (필수): `rtc_msgs/<*Log>` 또는 integration 패키지의 private POD 로그. closed-set 검증이라 오타는 parse 시 hard fail 이다.
- `instance` (필수): 코드의 `RegisterLog<MsgT>("<instance>", ...)` 와 1:1 매칭이고 CSV 파일 stem 이다 (`<session>/controllers/<config_key>/<instance>.csv`). **경로 단위로 unique** 하게 짓는다 (같은 device 의 state / sensor 는 `hand_state` / `hand_sensor`). 빈 값·중복 등록은 hard fail, 코드가 등록하지 않은 instance 는 skip 이다.
- POD 미러는 integration 패키지의 `logging/<msg>_pod.hpp` 에 둔다. capacity 는 그 robot 의 hardware 에 맞게 정한다 (`rtc_base` 에 robot constant 금지 — ARCH-1).
- controller 는 **`Compute()` 에서만** push 한다 (parameter callback · 비-RT thread 에서 push 금지).
- 첫 numeric column 은 `state.t_relative_s` 다. controller 는 `chrono::*::now()` 를 부르지 않는다.

## Documentation Requirements

- **Doxygen for public API**: `@brief`, `@param`, `@return`, `@note` on every public class/function
- **Math formulas**: LaTeX in Doxygen, paper reference (author, year, eq number), units + frame per parameter
- **FSM**: Document valid transitions, entry/exit conditions, timeout behaviors
- **Thread safety**: 공유 data member 는 sync 메커니즘을 명시한다 (SeqLock, SPSC, atomic, mutex)
- **서비스·메시지 계약의 소유자** (AP-DOC-1 · AP-DOC-2): 같은 문장을 여러 곳에 쓰지 말고 **축을 나눈다**:
  - `.srv` / `.msg` 주석 = **wire 계약 SSoT** (필드 의미, 판정 기준, 거부 조건, 권한) — 호출자가 알아야 하는 전부
  - 소비 패키지 README = 그 패키지 쪽 메커니즘과 수치의 근거. 계약은 `.srv` 를 링크
  - `rtc_msgs/README.md` = 색인 (필드 표 + SSoT 포인터). 근거 산문 금지
  - `.cpp` 파일 헤더 = 구현 선택. 계약 재서술 금지

## Commit Message Conventions

Follow **Conventional Commits**: `type(scope): subject` + optional body + optional footer.

- **Types**: `feat` | `fix` | `docs` | `style` | `refactor` | `perf` | `test` | `chore`
- **Scope**: package name or broad tag (`multi-pkg`, `launch`) when the change spans packages. One scope per commit — split unrelated changes.
- **Subject**: English, imperative mood, capitalized first letter, no trailing period, <= 50 chars.
- **Body / footer** (optional, blank line before each, wrap at 72): explain *what* and *why*, not *how*. Reference issues as `Closes: #N` / `Refs: #N`. PR merge commits keep GitHub's default subject.
- **Merge method**: 기본은 **no-ff merge commit** (`gh pr merge --merge`) 이다 — 브랜치 커밋 본문 (PROC-6 근거 같은 감사 기록) 이 `main` 에 보존된다. squash · rebase 는 사용자가 명시 지시할 때만 쓴다. 머지할 때 `--delete-branch` 로 로컬·원격 브랜치를 함께 정리한다.
