# AGENTS.md

이 파일은 RTC Framework에서 작업하는 에이전트의 **헌법**이다. 안정적인 원칙·게이트·지표만 둔다. 패키지 수, 로봇 목록, 의존성 버전, 명령 세부처럼 자주 바뀌는 사실은 이 문서에 복제하지 말고 README와 `agent_docs/`의 단일 출처(SSoT)를 참조한다.

## 1. Repository Snapshot

**RTC (Real-Time Control) Framework**는 URDF 기반 매니퓰레이터용 robot-agnostic real-time control framework다. 가변 DOF, 설정 가능한 `control_rate` RT loop, transport 추상화(UDP/CAN-FD/EtherCAT/RS485 등), lock-free SPSC, E-STOP을 제공한다.

- 패키지 구성과 역할: [README.md](README.md#패키지-구성), [agent_docs/architecture.md](agent_docs/architecture.md)
- 로봇 데이터: [robot_descriptions/README.md](robot_descriptions/README.md)
- 환경·의존성과 실행 명령: [README.md](README.md#빠른-시작), [repo_scripts/README.md](repo_scripts/README.md)
- 테스트와 디버그 절차: [agent_docs/testing-debug.md](agent_docs/testing-debug.md)

작업을 시작할 때는 이 문서의 **Invariants → Workflow → Escalation** 순서로 읽는다. 관련 작업이면 아래 참조 문서도 먼저 읽는다.

## 2. Non-Negotiable Invariants

전체 규칙, 탐지 패턴, 복구 절차는 [agent_docs/invariants.md](agent_docs/invariants.md)가 단일 출처다.

### RT path

정기 tick 또는 `SCHED_FIFO` RT 경로에서 금지되는 항목은 **RT-1 ~ RT-10** (RT-7 은 은퇴하여 PROC-6 으로 이동) 이다. 개별 규칙·탐지 패턴·대안은 `agent_docs/invariants.md` §RT Path Invariants 가 단일 출처이며, 목록을 여기에 복제하지 않는다 — 이전에 복제본이 7개에서 멈춰 RT-9(RT 경로의 `get_lifecycle_state()`)와 RT-10(`condition_variable`)이 이 문서를 읽는 도구에게 보이지 않았다.

이 규칙은 RT tick 경로에만 적용한다. lifecycle, auxiliary, test, initialization 코드는 해당하지 않는다. **어떤 콜백이 RT 인지는 함수 이름이 아니라 그 콜백이 붙은 executor 의 스케줄러가 결정하며**, 판정표는 `agent_docs/architecture.md` §Execution Contexts 에 있다.

### Architecture, process, and numerical rules

- `rtc_*` 패키지에 robot name, joint count, hardware ID를 하드코딩하지 않는다.
- 의존성 그래프를 거슬러 상위 계층에 의존하지 않는다.
- 두 번째 구체 구현을 추가하기 전 abstract interface 또는 concept를 정의한다. `#ifdef`나 하드코딩 switch로 우회하지 않는다.
- `robot_descriptions`는 data-only 패키지다. 소비자는 `<exec_depend>`와 ament index 런타임 lookup을 사용한다.
- 새 utility 작성 전 기존 `rtc_*`에 유사 기능이 있는지 검색하고, 맞지 않으면 fork보다 일반화를 우선 검토한다.
- 코드 변경 시 필요한 문서, YAML, `CMakeLists.txt`, `package.xml`을 함께 갱신한다.
- 기존 test assertion을 통과시키기 위해 약화하지 않는다. 테스트가 틀렸거나 spec이 바뀐 경우에는 근거와 분리된 변경 단위를 남긴다.
- `rtc_base` 또는 `rtc_msgs` 변경 시 전체 downstream 빌드·테스트를 수행한다.
- 수치 특이점은 damped pseudoinverse와 zero guard 규칙을 따른다.

## 3. Required Workflow

작업은 규모에 맞춰 다음 순서를 따른다.

**Type → Locate → Read → Edit → Build → Test → Verify**

- 먼저 수정인지, 새 기능·controller·message·device·thread 추가인지 분류한다.
- 추가 작업은 [agent_docs/design-principles.md](agent_docs/design-principles.md)와 [agent_docs/modification-guide.md](agent_docs/modification-guide.md)의 해당 "Adding a New ..." 절을 먼저 읽는다.
- 단순 오타, 포매팅, 자명한 한 줄 변경은 단계를 합칠 수 있다.
- Build, Test, Verify를 생략했다면 최종 보고에 생략한 항목과 이유를 명시한다.
- 다파일·다패키지 변경 또는 `rtc_base`/`rtc_msgs` 변경에서는 검증을 생략하지 않는다. 불가피하면 먼저 escalation 한다.
- 실패를 반복 시도만 하지 않는다. 누락된 test, lint, interface, 환경 capability를 보완하거나 escalation 한다.
- public header, launch, config, 파일 추가·삭제, dependency 변경에는 README 갱신 필요성을 확인한다. `CMakeLists.txt`와 `package.xml`의 동기화는 필수다.

변경 위치별 필수 sensor와 추가 sensor는 [agent_docs/testing-debug.md](agent_docs/testing-debug.md)의 matrix를 따른다.

## 4. Review and Validation

빌드·테스트·grep은 문법, 빌드 가능성, 기존 회귀를 확인하지만 설계 일관성까지 보장하지 않는다. 다음 변경에는 별도의 수동 코드 리뷰를 권장한다.

- `rtc_base` 또는 `rtc_msgs` 변경
- abstract interface 신설 또는 두 번째 구현 추가
- `rtc_*`에 robot-specific 코드가 들어갈 가능성이 있는 변경
- E-STOP 경로, safety publisher, lifecycle callback 변경
- 다파일·다패키지 PR, 100줄 이상 변경, 신규 패키지 디렉터리

리뷰에서는 robot-agnostic 원칙, abstract interface 필요성, 재사용성, RT 제약, public API 영향, E-STOP 안전성을 중점적으로 확인한다.

## 5. Escalate Before Changing

다음 상황에서는 코드를 작성하기 전에 `[CONCERN]`을 보고하고 사용자 결정을 기다린다.

severity 는 각 항목에 붙은 라벨을 따른다 — **Critical 은 사용자 승인 전 커밋·PR 을 금지**하므로, 라벨 없이 읽으면 무엇이 커밋을 막는지 알 수 없다. 아래 E-번호는 [CLAUDE.md](CLAUDE.md) §6 과 1:1 이며 그쪽이 SSoT 다.

- **E-1 (Critical)** — invariant를 위반하거나 예외가 필요할 가능성
- **E-2 (Critical)** — `rtc_*` 패키지에 robot-specific 값 추가
- **E-3 (Critical)** — `rtc_msgs` 또는 `shape_estimation_msgs` public ABI 변경
- **E-4 (Warning)** — abstract interface 없이 두 번째 구현 추가
- **E-5 (Warning)** — optional dependency(MuJoCo, aligator 등)의 fallback 제거
- **E-6 (Critical)** — 기존 test assertion 변경 또는 약화
- **E-7 (Critical)** — thread model(core affinity, priority) 변경
- **E-8 (Critical)** — E-STOP 경로 변경
- **E-9 (Warning)** — 문서와 코드 중 어느 쪽을 기준으로 맞출지 판단이 필요한 불일치
- **E-10 (Warning)** — `robot_descriptions`에 build-time 의존성(`find_package`, `<depend>`, `ament_target_dependencies`) 추가
- **E-11 (Warning)** — `PublishRole` enum에 controller-owned non-RT topic 추가. 새 controller-owned 토픽은 `SeqLock<T>` + `Setup*Publisher` helper 패턴을 쓴다

```text
[CONCERN] <한 줄 요약>
Severity: Critical | Warning | Info
Detail: <영향 범위, 검토한 대안>
Alternative: <우회안 한 가지 이상>
```

- **Critical**: 사용자 승인 전 커밋·PR·위험 변경을 진행하지 않는다.
- **Warning**: 사용자 판단에 따라 진행하며 결정 근거를 남긴다.
- **Info**: 기록 후 진행할 수 있다.

다단계 작업, 신규 public abstraction/controller/device/thread/message, `rtc_base`/`rtc_msgs` 변경, 기능 동등성이 중요한 리팩터링은 구현 전에 객관적으로 검증 가능한 성공 기준을 짧게 합의한다.

## 6. Build and Environment Hard Rules

명령 세부와 source 순서는 [README.md](README.md#빠른-시작) 및 [repo_scripts/README.md](repo_scripts/README.md)를 단일 출처로 삼는다.

### colcon working directory

`colcon build`와 `colcon test`는 반드시 colcon workspace root(`<rtc_ws>`, 일반적으로 `~/ros2_ws/rtc_ws`)에서 실행한다. repository root(`src/rtc-framework`)에서 실행하지 않는다. 직접 `colcon`을 실행할 때는 환경 설정 스크립트도 올바른 절대 경로 또는 workspace 기준 경로로 source한다.

repository 안에 잘못 생성된 `build/`, `install/`, `log/`는 잘못된 CWD의 신호다. **이 repo-local 트리는 재생성 가능하므로 확인 없이 삭제한다.** 절대 건드리지 않아야 하는 것은 workspace root(`<rtc_ws>/{build,install,log}`)의 정상 incremental cache 이며, 둘을 혼동하지 않는다.

### Virtual environment isolation

`.venv` 격리는 다른 control project와 공존하는 runtime PC의 의도된 설계다. venv 활성 상태에서 `colcon build`, `colcon test`, `ros2 run`, `ros2 launch`가 실패하면 `sys.path`, shebang, wrapper, dependency resolution의 근본 원인을 수정한다.

gtest binary 직접 실행, venv 비활성화, 강제 `PYTHONPATH` 설정 등으로 격리를 우회해 검증을 통과시키지 않는다.

예외는 하나다: `colcon build` 의 configure 단계에서 CMake `FindPython` 이 venv python 을 잡아 eigenpy/pinocchio 가 깨지는 경우, `deactivate` 또는 `-DPython3_EXECUTABLE=/usr/bin/python3` 를 쓸 수 있다(후자를 우선한다). 이는 빌드 시스템의 Python 탐색 문제이며 테스트 우회가 아니다. `colcon test` / `ros2 run` / `ros2 launch` 실패에 대한 비활성화는 금지가 유지된다.

## 7. Style and Conventions

상세 규칙은 [agent_docs/conventions.md](agent_docs/conventions.md)를 따른다.

- Namespace: `rtc`
- Naming: Google C++ (`PascalCase` types/methods/free functions, `snake_case_` private members, `kConstant` constants)
- Units: SI(m, rad, s, kg, N); degree는 API 경계에서만 사용
- Rotation: 내부는 Hamilton `Eigen::Quaterniond`, 경계에서만 ZYX Euler
- Paper notation: `J_b`, `q_d`, `K_d` 등
- RAII 사용, RT code에는 `noexcept`, 상태 반환에는 `[[nodiscard]]`
- 핵심 C++ node는 `rclcpp_lifecycle::LifecycleNode`를 사용하며 configure와 activate 단계를 분리
- Logger: node=`<exec_name>`, library=`<full_package_name>`, controller=`<package>.<controller_key>`; 점은 하나만 사용
- Commit: Conventional Commits (`type(scope): subject`)

## 8. Completion and Housekeeping

완료 보고에는 다음을 간단히 포함한다.

- 변경한 내용과 영향 범위
- 실행한 build/test/format/검토와 결과
- 생략한 검증과 이유
- 남은 위험, 후속 작업, 또는 사용자 판단이 필요한 사항

작업 후에는 repository root에 잘못 생성된 `build/`, `install/`, `log/`와 재생성 가능한 Python cache(`__pycache__`, `.pytest_cache`, `.ruff_cache`, `.mypy_cache`)가 있는지 확인한다. 삭제는 사용자 변경물이나 workspace root의 정상 cache를 건드리지 않는 범위에서만 수행한다.

## 9. Context Handoff

미완료 작업이 session · agent · model · 책임 경계를 넘을 때는 **handoff artifact** 를 만든다. artifact 는 받는 에이전트가 **이전 transcript 없이 재개**할 수 있어야 완료다 (Goal, Acceptance criteria, Out of scope, Current state, Next action, Decisions, Evidence, Failed approaches, Workspace(git), Pointers).

- 받는 쪽은 진행 전에 `git status` 와 핵심 evidence(build/test)가 artifact 와 일치하는지 검증하고, 불일치 시 **구현 전에 artifact 를 먼저 갱신**한다.
- 단순 오타·단일 세션 short task 는 artifact 불필요다. 다단계 작업은 [CLAUDE.md](CLAUDE.md) §6.5 Sprint Contract 를 적용한다.
- credentials · secret · raw 대용량 log · 미검증 주장은 넣지 않는다.
- **저장**: plan 파일은 repo 에 커밋하지 않는다 — 각 에이전트(Claude · Codex 등)가 자기 private 저장소에서 관리하고, tool 경계를 넘는 cross-tool 인계는 **git issue** 에 artifact 를 적어 공유한다. 완료된 plan 은 git log / issue / memory 로 복원 가능하거나 보존할 가치가 없으면 삭제 (상세: [agent_docs/handoff.md](agent_docs/handoff.md) §5).

trigger 분류표, artifact template, sender/receiver checklist, storage·retention 규칙의 단일 출처는 [agent_docs/handoff.md](agent_docs/handoff.md) 다. Claude 전용 메커니즘(`/compact`·`/clear`·fork 등)은 [CLAUDE.md](CLAUDE.md) §6.6 을 따른다.

## 10. Reference Documentation

- [agent_docs/architecture.md](agent_docs/architecture.md) — threading, data flow, core types, lock-free rules, lifecycle, E-STOP, dependency graph
- [agent_docs/controllers.md](agent_docs/controllers.md) — controller, gain, FSM, topic, config
- [agent_docs/modification-guide.md](agent_docs/modification-guide.md) — workflow와 controller/message/device/thread 추가 절차
- [agent_docs/design-principles.md](agent_docs/design-principles.md) — `rtc_*` 설계 원칙과 boundary rules
- [agent_docs/conventions.md](agent_docs/conventions.md) — domain, code, commit, documentation conventions
- [agent_docs/testing-debug.md](agent_docs/testing-debug.md) — sensor matrix, test commands, debug topics, RT permissions
- [agent_docs/invariants.md](agent_docs/invariants.md) — RT, architecture, process, numerical invariants
- [agent_docs/anti-patterns.md](agent_docs/anti-patterns.md) — 반복 실수의 탐지와 복구
- [agent_docs/handoff.md](agent_docs/handoff.md) — tool-neutral context handoff 계약(trigger 분류·artifact template·sender/receiver checklist·storage)
- [README.md](README.md) — install, build, dependency, quick start
- [repo_scripts/README.md](repo_scripts/README.md) — PREEMPT_RT, CPU shield, environment activation, isolated dependencies
