# Conventions — 근거 · 예시 · 세부

> **이 문서는 헌법이 아니다.** 스타일·도메인·커밋 규약의 **규범** 은 [agent_docs/conventions.md](../../agent_docs/conventions.md) 가 갖고, 여기는 그 규약의 근거 · 실제 위반 사례 · 예시 · 세부만 둔다. 규칙 문장은 여기 다시 적지 않는다 — 두 곳에 있던 규칙이 어긋난 것이 두 번이다 (2026-10-16 정리). 둘이 어긋나면 agent_docs 쪽이 옳다.

## Domain Conventions

규범: [conventions.md](../../agent_docs/conventions.md) §Domain Conventions

- **compliance 접두사가 가리키는 문서** — [rtc_controllers/docs/compliance-conventions.md](../../rtc_controllers/docs/compliance-conventions.md) 가 참조하는 원명세.

## Code Conventions

규범: [conventions.md](../../agent_docs/conventions.md) §Code Conventions

- **Naming 의 기계 SSoT 위치** — `.clang-tidy` 의 `readability-identifier-naming.MethodCase: CamelCase` 가 기계 SSoT.
- **Method split 을 consumer 이름으로 하는 근거** — stage 명 (`Phase3a`, `Step2`) 은 body 의 목적을 감춰 "이 method 가 왜 존재하나" 를 다른 문서에서 찾게 만들지만, consumer-named method 아래의 모든 라인은 그 consumer 를 위한 것임이 자명하다. 같은 필드를 공유 helper 로 빼지 않고 양쪽에서 독립적으로 write 하는 것은 공유 추출이 ordering drift 를 부르고 그 비용이 중복 write 비용보다 크기 때문이다. 실례: `demo_joint_controller.hpp` 의 `WriteJointCommand` 선언 위 주석이 method 마다 어느 출력 필드를 어느 consumer 가 읽는지 적고, 다른 세 컨트롤러 헤더가 그것을 가리킨다.
- **어떤 노드가 LifecycleNode 인가** — [architecture.md](../../agent_docs/architecture.md) 참조 — 개수 박제 금지 ([invariants.md](../../agent_docs/invariants.md) AP-DOC-1)
- **Tier 0 예외의 근거** — executor 에 넘길 callback group 은 on_cleanup→on_configure 사이클에서 같은 객체여야 하므로 configure 에서 만들 수 없다. 예외의 근거는 "생성자가 편해서" 가 아니라 "configure 가 오기 전에 이미 필요해서" 이고, 후자가 아니면 Tier 1 이다. 현재 유일한 사례는 `UdpHandNode` (`cb_group_aux_` + `aux_cpu_slot`·`use_cpu_affinity`, issue #345).
- **Formatting 설정 요지와 uncrustify 금지의 근거** — C++ [`.clang-format`](../../.clang-format) 은 Google base, ColumnLimit 100, PointerAlignment Left, Python [`pyproject.toml`](../../pyproject.toml) 은 ruff, line 99. PostToolUse hook ([.claude/hooks/format-code.sh](../../.claude/hooks/format-code.sh)) 이 모든 Edit 마다 자동 적용. uncrustify 의 ROS 2 / Eclipse-CDT 표준 스타일 (Allman braces, `T *p`, `T &r`) 이 clang-format Google base 와 영구 충돌. `set(ament_cmake_uncrustify_FOUND TRUE)` 스킵 트릭은 작동하지 않는다 (extras hook `ament_cmake_uncrustify_lint_hook.cmake` 가 무조건 등록됨). 개별 lint depend 의 형태: `<test_depend>ament_cmake_{cppcheck,lint_cmake,xmllint}</test_depend>` + CMakeLists 에 `find_package(...)` + `ament_cppcheck()` / `ament_lint_cmake()` / `ament_xmllint()` 개별 호출. 기존 패키지에서 uncrustify 를 떼낸 직후엔 `build/<pkg>/test_results/<pkg>/uncrustify.xunit.xml` 이 남아있을 수 있으므로 `rm -rf build/<pkg>/test_results` 후 colcon test 로 검증.
- **Include grouping 의 세부** — `.clang-format` `IncludeBlocks: Regroup` + `IncludeCategories` 가 4 Priority 로 분류 (SSoT 는 `.clang-format`):
    - **P1** 본 저장소 프로젝트 헤더 — `"rtc_*"`/`"shape_estimation*"`/`"integrated_bringup*"`/`"ur5e_bt_coordinator*"`/`"udp_hand_driver*"`/`"robot_descriptions*"` (quote/angle 둘 다)
    - **P2** ROS 2 — `<rclcpp/*>`, `<std_msgs/*>` 등
    - **P3** third-party — Eigen, pinocchio, fmt, MuJoCo, BehaviorTree 등
    - **P4** stdlib — `<algorithm>` 등
    - **예외**: `#ifdef ... #endif` 사이에 낀 include 는 clang-format 이 grouping 못 해 blank line 이 살아 있다.

## Logging

규범: [conventions.md](../../agent_docs/conventions.md) §Logging

- **Logger 이름 규약의 근거** — 로그 한 줄에서 **어느 패키지** · **어느 컨트롤러**인지 즉시 읽히도록 한다. (터미널에는 `[<logger>]: <msg>` 형태로 찍힘.)
- **Node-owned logger = `<exec_name>` 의 근거** — ROS 노드 이름 = 실행 파일 이름 (예: `integrated_rt_controller`). `node_->get_logger()`는 그 이름을 그대로 반환. `rtc_controller_manager`는 library-only (실행 파일 없음) — runtime identity는 robot-specific bringup이 소유. 자세한 원칙: [design-principles.md](../../agent_docs/design-principles.md)
- **컨트롤러 정적 logger 캐시의 예** — [integrated_bringup/include/integrated_bringup/support/bringup_logging.hpp](../../integrated_bringup/include/integrated_bringup/support/bringup_logging.hpp)
- **Base class 공통 로그의 예** — `RTControllerInterface` 에서 찍는 공통 로그는 `rclcpp::get_logger("rtc_controller_interface")` + 메시지 본문에 `[<controller_name>]` prefix.
- **prefix 축약 금지의 예** — `bringup.demo_joint` 사용 금지 → `integrated_bringup.demo_joint_controller`
- **패키지 README 가 갖는 것** — 레벨 표 대신 *자기 예시*·서브-로거 표·THROTTLE 상수 이름만 갖는다.
- **서브-로거 런타임 필터링 명령** — `ros2 service call /<node>/set_logger_levels rcl_interfaces/srv/SetLoggerLevels "{levels: [{name: '<logger>', level: 10}]}"` (10=DEBUG, 30=WARN, 50=FATAL — 상위 이름은 하위 전체에 매칭)

RT path logging 금지 규칙과 SPSC 우회 패턴은 [invariants.md](../../agent_docs/invariants.md) RT-3 참조.

## Controller-owned CSV logging (`logs:` schema)

규범: [conventions.md](../../agent_docs/conventions.md) §Controller-owned CSV logging

데이터 CSV (per-tick state / sensor / inference 등) 를 controller 가 소유하고 CM 이 logging authority 가 아닌 것은 Phase C 결정이다. `logs:` 섹션 예시:

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

- **`msg_type` 의 세부** — integrated_bringup-private POD 로그 (`DeviceWbcLog`/`WbcDiagLog`/`PullEstimatorLog`) 는 `integrated_bringup/<*Log>`. 컨트롤러 `LoadConfig` 의 closed-set 검증에 매핑된다.
- **`instance` 의 세부** — 같은 LogSet 안에서 빈 `instance` 또는 동일 instance 를 두 번 등록하면 `RegisterLog` 가 unbound handle 을 반환한다 (코드 enforcement); 코드가 등록하지 않은 instance (optional·gated 채널이 비활성일 때, 예: `pull_estimator` wiring disabled) 는 silently skip. CSV 파일 stem 으로도 사용되므로 instance 는 `(msg_type, instance)` 가 unique 하도록 작성하지 말고, *경로 단위로 unique* 하도록 작성한다.
- `topic` (옵션, 현재는 unused): 향후 `rtc_msgs/*Log` 를 DDS 로 publish 하는 옵션을 위해 예약. Q-MSG-1(a) lock 으로 현재는 schema-only POD→SPSC→CSV
- **POD 미러 위치의 결정 (Q-MSG-2(d))** — `<robot>_bringup/include/<robot>_bringup/logging/<msg>_pod.hpp`. hardware 에 맞게 선정하는 capacity 의 예: `kMaxJoints`.
- **Push site 를 `Compute()` 로 제한하는 근거 (Q-ACTIVITY-GATING)** — parameter callback / BT bridge / 비-RT thread 에서의 push 를 막아 inactive controller 의 CSV 에 row 가 쌓이는 것을 방지.
- **Timestamp 의 출처 (Q-TIME)** — `state.t_relative_s` 는 CM RT loop 가 채움 — 실기는 `t0 - log_start_time_` (steady), sim (`use_sim_time_sync`) 은 `iteration × dt` (#566).

## Documentation Requirements

규범: [conventions.md](../../agent_docs/conventions.md) §Documentation Requirements

- **서비스·메시지 계약 축 분리의 근거 — 실제 위반** — 한 서비스는 최대 네 곳 (`.srv`/`.msg` 파일 · 구현 `.cpp` 파일 헤더 주석 · `rtc_msgs/README.md` · 소비 패키지 README) 에서 설명될 수 있다. `/rtc_cm/clear_estop` 의 관측 창 산술이 네 곳에 동시에 존재했다 (#288 직후 정리).

## Commit Message Conventions

규범 (type · scope · subject · body · issue 참조 · merge 방식) 은 [conventions.md](../../agent_docs/conventions.md) §Commit Message Conventions 가 갖는다 — 여기는 근거와 예시만 둔다.

- **subject 의 50 자는 `type(scope): ` 뒤부터 센다** — `main` 의 관행이 그렇다: 최근 300 개 중 205 개가 줄 전체로는 50 자를 넘고 subject 부분으로는 지킨다.
- **issue 참조에 closing keyword 를 쓰지 않는다** — 이 저장소의 이슈는 merge 뒤 정리 · 결과 코멘트까지 Done 기준에 넣는 경우가 많아, keyword 가 merge 순간 이슈를 닫으면 남은 일이 추적에서 빠진다. 줄을 넘겨 이어진 keyword 도, project item 을 Done 으로 옮기는 것도 이슈를 닫았다 (#716, #822, #791).
- **Merge method** — **기본은 no-ff merge commit** (`gh pr merge --merge`, 사용자 확정 2026-08-16): 머지 커밋이 PR 단위를 히스토리에 남기고, 브랜치 커밋 본문(PROC-6 spec-change 근거 같은 감사 기록)이 `main` 에 그대로 보존된다. **squash 는 커밋 본문을 PR 본문으로 덮어쓰므로** rebase 와 함께 사용자가 명시 지시할 때만 쓴다 (선례: #442 rebase · #450 no-ff; #464 까지는 squash 관례였고 `(#N)` 접미사가 그 흔적). `delete_branch_on_merge=false` 이므로 `gh pr merge --delete-branch` 로 로컬·원격 브랜치를 함께 정리한다.

Example with body + footer:
```
feat(multi-pkg): Surface MPC solve failures via throttled stderr

Previously a failed solve was silently dropped, masking convergence
regressions during long runs. Emit a count=0 sentinel plus a
rate-limited warning so operators see repeated failures without
flooding the console.

Refs: #92
```
