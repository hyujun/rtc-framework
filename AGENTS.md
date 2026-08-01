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

규칙 ID 를 함께 적는다 — escalation 번호와 severity 가 ID 로 연결되고 (§5), 위반 보고도 ID 로 한다.

- **ARCH-1** — `rtc_*` 패키지에 robot name, joint count, hardware ID를 하드코딩하지 않는다.
- **ARCH-2** — 의존성 그래프를 거슬러 상위 계층에 의존하지 않는다.
- **ARCH-3** — 두 번째 구체 구현을 추가하기 전 abstract interface 또는 concept를 정의한다. `#ifdef`나 하드코딩 switch로 우회하지 않는다.
- **ARCH-5** — `robot_descriptions`는 data-only 패키지다. 소비자는 `<exec_depend>`와 ament index 런타임 lookup을 사용한다. 테스트가 그 런타임 lookup 을 쓰면 설치 순서 보장 목적의 `<test_depend>` 도 허용된다 — `find_package` · `<depend>` · `<build_depend>` · `ament_target_dependencies` 는 금지.
- **ARCH-6** — 컨트롤러 토픽 QoS depth 는 1 이 기본이다. 예외는 코드에 `// ARCH-6-exempt` 마커와 사유를 남긴다.
- **ARCH-7** — `rtc_*` 는 RT 제어 루프를 구동하는 실행파일을 소유하지 않는다. 진입 *함수*만 export 하고, exec 는 downstream (integration package) 이 만든다. robot-agnostic standalone 노드·`example_*` 는 예외이며 그 목록은 [agent_docs/design-principles.md](agent_docs/design-principles.md) 가 SSoT.
- **P5 (설계원칙 5)** — 새 utility 작성 전 기존 `rtc_*`에 유사 기능이 있는지 검색하고, 맞지 않으면 fork보다 일반화를 우선 검토한다.
- **PROC-1** — 코드 변경 시 필요한 문서, YAML, `CMakeLists.txt`, `package.xml`을 함께 갱신한다.
- **PROC-3** — `rtc_base` 또는 `rtc_msgs` 변경 시 전체 downstream 빌드·테스트를 수행한다.
- **PROC-6** — 기존 test assertion을 통과시키기 위해 약화하지 않는다. 테스트가 틀렸거나 spec이 바뀐 경우에는 근거와 분리된 변경 단위를 남긴다.
- **NUM-1·NUM-2·NUM-4** — 수치 특이점은 damped pseudoinverse, `dt` zero guard, 게인 zero guard 규칙을 따른다.
- **NUM-5** — 폐쇄 체인 사영은 **residual 로 조립 분기를 판정할 수 없다**. 점 구속 loop 은 분기가 여럿이고 모두 φ=0 을 만족하므로 seed 증분 제한이 필수다. 완화 장치는 발동한 경우에만 적용하고, 그로 인한 `held` 를 자기 치유로 가정하지 않는다.

전체 규칙 (RT-1~RT-10, ARCH-1~7, PROC-1~7, NUM-1~6) 과 탐지 grep·복구 절차는 [agent_docs/invariants.md](agent_docs/invariants.md) 가 SSoT다. 위는 발현 빈도가 높은 것만 추린 요약이다.

### 반복 실수 (사전 신호)

[agent_docs/anti-patterns.md](agent_docs/anti-patterns.md) 가 실제 위반 commit 사례·탐지·복구를 담는다. 최근 빈도 상위: **AP-RT-1** (정기 tick 에서 `RCLCPP_*` 로깅) · **AP-RT-3** (`auto` + Eigen expression aliasing) · **AP-ARCH-1** (`rtc_*` 에 robot 상수) · **AP-PROC-1** ("완료" 보고 후 실제 미완료) · **AP-PROC-4** (test assertion 수정) · **AP-DOC-1** (헌법·문서에 패키지 수·테스트 수 같은 변동 사실 박제).

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

변경 위치별 필수 sensor와 추가 sensor는 [agent_docs/testing-debug.md](agent_docs/testing-debug.md)의 matrix를 따른다. 그 matrix 의 **필수 sensor 와 추가 sensor 를 모두** 실행한다.

### 커밋 전에 직접 돌려야 하는 것

이 저장소에는 Claude Code 전용 자동화(편집 후 포매팅, turn 종료 시 검증)가 배선돼 있다. **그 자동화는 다른 도구에서는 돌지 않으므로** 아래를 직접 수행한다.

- **포매팅** — C/C++ 는 `clang-format`(루트 `.clang-format`), Python 은 `ruff format` + `ruff check`(루트 `pyproject.toml`). 변경한 파일에 적용한다.
- **빌드·테스트** — 변경한 패키지를 빌드하고 테스트한다. `rtc_base`/`rtc_msgs` 를 건드렸으면 전체 downstream (PROC-3).
- **문서 검증** — `.md` 를 고쳤으면 `python3 repo_scripts/scripts/validate_docs.py --files <파일들>`.
- **YAML** — 고친 YAML 이 parse 되는지 확인하고, default 값·유효 범위·단위 주석이 코드와 맞는지 본다.
- **Doxygen** — public header 를 바꿨으면 주석을 갱신한다.

CI(`.github/workflows/`)가 문서 코퍼스 전체 검증·빌드·CodeQL 을 다시 돌리므로, 위를 건너뛴 변경은 PR 에서 막힌다.

## 4. Review and Validation

빌드·테스트·grep은 문법, 빌드 가능성, 기존 회귀를 확인하지만 설계 일관성까지 보장하지 않는다. 다음 변경에는 별도의 수동 코드 리뷰를 권장한다.

- `rtc_base` 또는 `rtc_msgs` 변경
- abstract interface 신설 또는 두 번째 구현 추가
- `rtc_*`에 robot-specific 코드가 들어갈 가능성이 있는 변경
- E-STOP 경로, safety publisher, lifecycle callback 변경
- 다파일·다패키지 PR, 100줄 이상 변경, 신규 패키지 디렉터리
- 다파일 리팩터 또는 유사 기능 중복이 의심되는 변경 — 이때는 버그가 아니라 **재사용·단순화** 관점으로 본다 (P5)

리뷰에서는 robot-agnostic 원칙, abstract interface 필요성, 재사용성, RT 제약, public API 영향, E-STOP 안전성을 중점적으로 확인한다.

## 5. Escalate Before Changing

다음 상황에서는 코드를 작성하기 전에 `[CONCERN]`을 보고하고 사용자 결정을 기다린다.

**E-1 ~ E-11 트리거 표·severity·`[CONCERN]` 포맷의 SSoT 는 [agent_docs/invariants.md](agent_docs/invariants.md) §Escalation Triggers 다** — tool-neutral 이므로 이 문서에 복제하지 않는다. 착수 전 그 표를 연다.

severity 의 효력만 여기 박는다:

- **Critical**: 사용자 승인 전 커밋·PR 을 진행하지 않는다. (E-1 invariant 일반 · E-2 ARCH-1 · E-3 msgs ABI · E-6 test assertion · E-7 thread model · E-8 E-STOP)
- **Warning**: 사용자 판단에 따라 진행하며 결정 근거를 남긴다. (E-4 · E-5 · E-9 · E-10 · E-11)
- **Info**: 기록 후 진행할 수 있다.

grep 이 정당한 코드를 invariant 위반으로 잘못 잡았다고 판단되면, **보고 없이 우회하지 않는다** — 판정 절차와 한 줄 보고 포맷 (`false-positive: <rule-id> at <file:line>, reason=...`) 은 [agent_docs/invariants.md](agent_docs/invariants.md) §False-positive 처리에 있다. RT 여부가 애매하면 비-RT 로 가정하지 말고 `[CONCERN]` 으로 보고한다.

### 착수 전 성공 기준

다단계 작업(PR 단위·다파일·다패키지·신규 디렉터리·phase 분할), 신규 abstract interface·controller·device·thread·message 추가, `rtc_base`/`rtc_msgs` 변경, 기능 동등성이 성공 조건인 리팩터링은 코드 수정 **전에** 1~3줄로 객관 검증 가능한 성공 기준을 제시하고 컨펌받는다. 포맷과 절차는 [agent_docs/modification-guide.md](agent_docs/modification-guide.md) §Sprint Contract & Spec.

면제: 단일 파일 bug fix, 오타·포매팅, 단일 함수 추가, 의도가 한 줄 메시지에서 자명한 경우.

### 반복 실패

같은 문제를 **3회** 시도해도 풀리지 않으면 더 시도하지 말고 중단한다 — 무엇을 시도했고 각각 왜 실패했는지 진단을 정리해 escalate 한다 ([agent_docs/handoff.md](agent_docs/handoff.md)).

## 6. Build and Environment Hard Rules

명령 세부와 source 순서는 [README.md](README.md#빠른-시작) 및 [repo_scripts/README.md](repo_scripts/README.md)를 단일 출처로 삼는다.

### colcon working directory

`colcon build`와 `colcon test`는 반드시 colcon workspace root(`<rtc_ws>` = `~/ros2_ws/rtc_ws`)에서 실행한다. repository root(`src/rtc-framework`)에서 실행하면 그 안에 별도 `build/`·`install/`·`log/` 트리가 생기고, 이후 호출마다 두 트리를 오가게 된다.

**표준형은 서브셸이다:**

```bash
( cd <rtc_ws> && source <rtc_ws>/src/rtc-framework/repo_scripts/scripts/setup_env.sh >/dev/null 2>&1 && colcon … )
```

ws-root cwd 와 절대경로 source 를 둘 다 만족하면서 **호출이 끝나면 cwd 를 원래 위치로 되돌린다**. `cd <rtc_ws> &&` 로 시작만 해도 규칙 자체는 지켜지지만 그 다음 호출부터 cwd 가 ws root 에 남아, 세션 인계 노트가 "cd 금지" 라는 반대 규율을 따로 만들어 내는 원인이 됐다. ws-root 로 갈 수 없으면 절대경로 `--build-base` / `--install-base` 를 지정한다. `build.sh` / `install.sh` 는 내부에서 workspace 로 이동하므로 안전하다.

이 규칙은 몰라서가 아니라 **cwd drift** 로 재발한다. 셸의 cwd 는 호출 사이에 유지되므로, grep 하려고 repo 로 이동한 뒤 *다음* 호출에서 colcon 을 치면 그게 이미 위반이다. 증상이 빌드 실패가 아니라는 점이 비싸다 — **같은 세션의 검증들이 서로 모순되는 결과를 낸다** (한 테스트는 수정된 코드로, 다른 테스트는 stale 바이너리로 돈다). 검증 결과가 설명 불가하게 엇갈리면 코드를 의심하기 전에 `ls src/rtc-framework/build` 부터 확인한다.

**`source` 를 파이프라인에 넣지 않는다.** `source setup_env.sh 2>&1 | tail -2 && colcon build …` 처럼 쓰면 source 가 subshell 에서 실행돼 환경이 부모 셸에 반영되지 않는다. 위 표준형과 상충하지 않는다 — 거기서는 `source` 와 `colcon` 이 **같은** 서브셸 안에 있고, 문제는 파이프가 만든 서브셸에 `colcon` 이 안 들어가는 것이다. 조용히 성공한 것처럼 보이는 게 함정이다 — `colcon` 자체는 PATH 에 있어 빌드가 시작되고, 한참 뒤 CMake 안에서 `ModuleNotFoundError: No module named 'ament_package'` 같은 **원인을 가리키지 않는 에러**로 죽는다. 출력을 줄이려면 리다이렉션(`source … >/dev/null 2>&1`)을 쓴다. 이 실패를 `Python3_EXECUTABLE` 탓으로 오진하기 쉬운데, 감별은 `/usr/bin/python3 -c "import ament_package"` 한 줄이면 된다.

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
- 대응 GitHub issue 갱신(구현 범위, 미충족 acceptance criteria, 후속 작업) 또는 close

대응하는 GitHub issue 가 있으면 구현 완료 시 갱신한다. issue 는 durable 결정 기록이자 cross-tool 인계면이므로 갱신 없이 닫지 않는다. criteria 를 전부 충족했으면 close 하고, 아니면 남은 범위를 코멘트로 남기고 open 으로 둔다.

작업 후에는 repository root에 잘못 생성된 `build/`, `install/`, `log/`와 재생성 가능한 Python cache(`__pycache__`, `.pytest_cache`, `.ruff_cache`, `.mypy_cache`)가 있는지 확인한다. 작업 중 만든 임시 파일(분석 스크립트, 중간 산출물, 로그 덤프)도 종료 시 삭제한다. 보존 가치가 있으면 git log, 문서, issue 로 옮긴 뒤 삭제한다. 삭제는 사용자 변경물이나 workspace root의 정상 cache를 건드리지 않는 범위에서만 수행한다.

feature branch 가 `main` 에 merge 됐으면 로컬 merged branch 를 삭제하고(`git branch -d`) stale remote-tracking ref 를 정리한다(`git fetch --prune`). 현재 checkout 된 branch·미merge branch·`main` 은 건드리지 않는다.

## 9. Context Handoff

미완료 작업이 session · agent · model · 책임 경계를 넘을 때는 **handoff artifact** 를 만든다. artifact 는 받는 에이전트가 **이전 transcript 없이 재개**할 수 있어야 완료다. **채울 섹션 목록은 [agent_docs/handoff.md](agent_docs/handoff.md) §2 가 SSoT** 이며 여기 복제하지 않는다 — 이전에 복제본이 한 섹션(`Constraints / pending human decisions`)을 잃은 채 굳었다.

- 보내는 쪽은 필수 섹션을 전부 채우고(해당 없으면 `N/A`), Evidence 는 **실제 실행한 명령과 결과만** 적으며, 시간은 상대 표현이 아니라 절대 날짜로 쓴다. `git status` 와 HEAD 를 기록한다.
- 받는 쪽은 transcript 가 아니라 artifact 를 읽고 시작한다. 진행 전에 `git status` 와 핵심 evidence(build/test)가 artifact 와 일치하는지 검증하고, 불일치 시 **구현 전에 artifact 를 먼저 갱신**한다. Acceptance criteria 와 Out of scope 를 확인한 뒤 Next action 을 집는다.
- 단순 오타·단일 세션 short task 는 artifact 불필요다. 다단계 작업은 §5 의 착수 전 성공 기준을 적용한다.
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
