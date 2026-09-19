# AGENTS.md

이 파일은 본 저장소의 **헌법 (constitution)** 이다 — tool-neutral 단일본이며, Claude Code 는 [CLAUDE.md](CLAUDE.md) 가 이 파일을 import 한 뒤 Claude 전용 절만 덧붙인다. 안정적인 원칙·게이트·지표만 둔다. 자주 변하는 사실 (패키지 수, robot 목록, 의존성 버전, 명령 detail) 은 sub-doc / README 의 SSoT 를 참조한다 (AP-DOC-1). 규칙의 *근거·사고 이력* 도 두지 않는다 (AP-DOC-2) — 그 규칙을 소유한 문서·hook 헤더·anti-patterns 사례가 갖는다.

## 1. Snapshot

**RTC (Real-Time Control) Framework** — URDF 기반 매니퓰레이터를 위한 robot-agnostic real-time control framework. 변수 DOF, 설정 가능한 RT 루프 주기 (`control_rate` YAML — rate 범위·default 는 [agent_docs/invariants.md](agent_docs/invariants.md) §RT Path 가 SSoT), transport 추상화, lock-free SPSC, E-STOP.

- 패키지 구성·count·역할: [README.md](README.md#패키지-구성) · [architecture.md](agent_docs/architecture.md)
- 로봇 데이터 (URDF/MJCF/mesh): [robot_descriptions/README.md](robot_descriptions/README.md)
- 언어·OS·의존성 버전: [README.md](README.md#빠른-시작) · [repo_scripts/README.md](repo_scripts/README.md)
- test 카운트·실측: [testing-debug.md](agent_docs/testing-debug.md)

## 2. Harness Overview

에이전트 가이드는 **Agent = Model + Harness** 프레이밍의 5구성요소로 조직된다 (근거·출처·표: [agent_docs/harness-rationale.md](agent_docs/harness-rationale.md)) — **Guides** §3·§10 · **Sensors** §5 (computational)·§5.5 (inferential) · **Orchestration** §4 · **Escalation** §6·§6.5 · **Enforcement** — Claude Code 는 hook 이 §4 중 기계 판정 가능한 부분을 차단하고, 다른 도구는 §4 를 직접 돌린다.

**첫 방문**: §3 → §4 → §6 순으로 읽는다. **수정 중**: §5 검증 + §6 escalation 확인. Invariant 위반 의심 시 즉시 §6.

## 3. Invariants (요약)

전체: [agent_docs/invariants.md](agent_docs/invariants.md). 규칙 목록을 여기 복제하지 않는다 (AP-DOC-1 본 repo 사례). 위반 보고·escalation 은 **규칙 ID** 로 한다.

### RT path 절대 금지 (정기 tick — `control_rate` YAML)

- 규칙은 **RT-1 ~ RT-10** (RT-7 은 은퇴 → PROC-6). 전문·탐지 패턴·대안은 invariants.md §RT Path Invariants (Claude Code 는 path-scoped rule 이 편집 시 요약을 자동 로드한다 — CLAUDE.md §Claude Code)
- RT tick / SCHED_FIFO 경로에만 구속되고 lifecycle·aux·test·init 코드는 면제. **RT 여부는 함수 이름이 아니라 그 콜백이 붙은 executor 의 스케줄러가 결정** (판정표: architecture.md §Execution Contexts). 수정 전 반드시 확인하고, 위반 필요시 §6 `[CONCERN]`

### Architecture / Process / Numerical

- Architecture 규칙은 **ARCH-1 ~ ARCH-7**. 전문·판정 절차는 invariants.md §Architecture Invariants. **탐지 패턴의 SSoT 는 `.claude/hooks/verify-changes.sh`** 이고 문서는 판정만 갖는다 — RT 와 반대 방향이다
- 새 utility 작성 전 기존 `rtc_*` 패키지에 유사 기능 검색 — 맞지 않으면 fork 대신 일반화 ([design-principles.md](agent_docs/design-principles.md) P5)
- 코드 변경 → 대응 문서·YAML·CMakeLists·package.xml 동기화 필수 (PROC-1)
- 기존 test assertion 을 통과시키려 **약화 금지** — 새 코드를 고치되, test 가 진짜 틀렸거나 spec 이 바뀌면 별도 commit + 근거 (PROC-6, §6 E-6)
- `rtc_base` / `rtc_msgs` 변경 시 전체 빌드·테스트 (PROC-3)
- 수치 특이점: damped pseudoinverse (NUM-1), zero guard (NUM-2, NUM-4)
- 폐쇄 체인 사영은 **residual 로 조립 분기를 판정할 수 없다** — 점 구속 loop 은 분기가 여럿이고 모두 φ=0 을 만족하므로 seed 증분 제한이 필수 (NUM-5). 완화 장치는 발동한 경우에만 적용하고, 그로 인한 `held` 를 자기 치유로 가정하지 않는다

## 4. Workflow Loop

7단계: **Type → Locate → Read → Edit → Build → Test → Verify**. 규모에 맞춰 압축한다 — 오타·포매팅·자명한 단일 라인 수정은 단계를 합쳐도 되나, **검증(Build/Test/Verify)을 생략했다면 최종 보고에 무엇을·왜 생략했는지 명시**한다. 다파일·다패키지·`rtc_base`/`rtc_msgs` 변경에서 검증 단계를 건너뛰는 것은 §6 escalation 사유. 실패 시 절대 **"try harder" 금지** — 누락된 capability (test, lint, interface) 를 엔지니어링하거나 §6 escalate.

**Type 분기**: "수정" 인가 "추가 (새 기능 / 컨트롤러 / 메시지 / 디바이스 / 스레드 / 패키지)" 인가? 추가면 착수 전에 [design-principles.md](agent_docs/design-principles.md) 5원칙 + [modification-guide.md](agent_docs/modification-guide.md) "Adding a New ..." 절을 먼저 읽는다 (rtc_* 추가는 P1·P2 + ARCH-3; integration package 또는 `shape_estimation*` 추가 시 rtc_* 일반화 가능성부터 검토).

**계획 전 분석**: 대응하는 GitHub issue 가 있으면 계획 전에 그 issue (본문 + 코멘트) 를 먼저 참고한다 — issue 는 durable 결정 기록이자 cross-tool 인계면이다 (§6.6). 단 issue 의 진단·근거는 **미검증 가설**로 취급하고 착수 전 grep/코드로 반증한다 (틀렸으면 issue 를 먼저 갱신). 완료 후 갱신 규칙은 §11.

### 커밋 전에 직접 돌려야 하는 것

단계 4·5·6 의 최소 집합이며 **다른 도구에서는 전부 직접 수행한다.** Claude Code 의 Stop hook 은 이 중 기계 판정 가능한 부분만 대신한다 (범위: hook 헤더).

- **포매팅** — C/C++ 는 `clang-format` (루트 `.clang-format`), Python 은 `ruff format` + `ruff check` (루트 `pyproject.toml`). 변경한 파일에 적용
- **빌드·테스트** — 변경한 패키지를 빌드·테스트한다 (§9 hard rule 준수). `rtc_base`/`rtc_msgs` 를 건드렸으면 전체 downstream (PROC-3)
- **문서·메타데이터** — `.md` 를 고쳤으면 `python3 repo_scripts/scripts/validate_docs.py --files <파일들>`; 고친 YAML 의 parse·default·범위·단위; public header 의 Doxygen; public surface (header/launch/config/파일 add·del/dep) 변경 시 README; `CMakeLists.txt`·`package.xml` 동기화는 필수
- CI 는 `docs-validate` (문서·생성물·셸 검사) 뿐이다 — **빌드·테스트·포매팅의 게이트는 위 로컬 검증뿐이다**

단계별 액션·grep 패턴·Completion Checklist: [modification-guide.md](agent_docs/modification-guide.md).

## 5. Sensors

**변경 위치별 sensor matrix·명령·Live Debug Topics**: [agent_docs/testing-debug.md](agent_docs/testing-debug.md) 가 단일 출처.

- 변경 패키지의 sensor 행을 testing-debug.md 에서 찾아 **필수 sensor + 추가 sensor** 모두 실행
- 실패한 채로 커밋하지 않는다
- `rtc_base` / `rtc_msgs` 변경 시 전체 downstream 검증 (PROC-3)

## 5.5 Inferential Sensors (LLM-as-judge, 수동 trigger)

§5 의 computational sensor 는 **문법·빌드·기존 테스트 통과** 만 검증하고 의미 회귀 (설계 일관성, robot-agnostic, interface 누락, 재사용성) 는 잡지 못한다 — 에이전트 자기 평가는 신뢰 불가. 다음 변경에는 별도의 code review 를 사용자에게 권한다 (표 전문: [modification-guide.md](agent_docs/modification-guide.md) §Inferential review 트리거):

- code review: `rtc_base`/`rtc_msgs` 변경 · abstract interface 신설/두 번째 구현 (ARCH-3) · `rtc_*` 에 robot-specific 의심 (ARCH-1) · 100+ 줄 또는 신규 패키지 · PR 준비 (브랜치 전체)
- security review: E-STOP 경로 / safety publisher / lifecycle 콜백 수정 (E-8)
- 재사용·단순화 review (버그 탐지가 아니다): 다파일 리팩터 / 유사 기능 중복 의심 (P5) / 변경 후 정리

리뷰 관점: robot-agnostic, abstract interface 필요성, 재사용성, RT 제약, public API 영향, E-STOP 안전성.

## 6. Escalation Triggers

다음 상황에서 코드를 쓰기 **전에** `[CONCERN]` 보고 후 사용자 컨펌 대기. **E-1 ~ E-11 트리거 표·severity·`[CONCERN]` 포맷의 SSoT 는 [agent_docs/invariants.md](agent_docs/invariants.md) §Escalation Triggers 다** — 헌법에 복제하지 않는다 (AP-DOC-1). 헌법이 박는 것은 severity 의 **효력**뿐이다:

- **Critical**: 사용자 컨펌 전까지 커밋·PR 금지
- **Warning**: 사용자 판단에 따라 진행, 결정 로그 남김
- **Info**: 기록만, 진행 가능

Critical 은 E-1(invariant 일반)·E-2(ARCH-1)·E-3(msgs ABI)·E-6(test assertion)·E-7(thread model)·E-8(E-STOP), Warning 은 E-4·E-5·E-9·E-10·E-11 이다.

grep 이 정당한 코드를 잘못 잡았다고 판단되면 **보고 없이 우회하지 않는다** — 판정 절차와 보고 포맷은 invariants.md §False-positive 처리. RT 여부가 애매하면 비-RT 로 가정하지 말고 `[CONCERN]`.

**반복 실패**: 같은 문제를 **3회** 시도해도 풀리지 않으면 더 시도하지 말고 중단한다 — 무엇을 시도했고 왜 실패했는지 진단을 정리해 escalate 한다 ([handoff.md](agent_docs/handoff.md)).

## 6.5 Sprint Contract (착수 전 성공 기준 협상)

다음 task 에서는 코드 수정 시작 *전* 1~3줄로 **객관 검증 가능한** 성공 기준 (`[SPRINT]` 포맷) 을 제시하고 컨펌받는다. 포맷·spec 절차·예시는 [modification-guide.md](agent_docs/modification-guide.md) §Sprint Contract & Spec.

- 다단계 task (PR 단위 / 다파일 / 다패키지 / 신규 디렉토리 / phase 분할)
- 신규 abstract interface · controller / device group / thread / message 추가 — 이 경우 **Sprint Contract = spec** 이며 각 도구의 private plan 파일에 박는다 (§6.6 저장 규칙)
- `rtc_base` / `rtc_msgs` 변경 · 리팩터 (기능 동등성 유지가 곧 success)

면제: 단일 파일 bug fix, 오타·포매팅, 단일 함수 추가, 사용자 의도가 1줄 메시지에서 자명한 경우.

## 6.6 Long-running task — Context handoff

미완료 작업이 session · agent · model · 책임 경계를 넘을 때는 **handoff artifact** 를 만든다. artifact 는 받는 에이전트가 **이전 transcript 없이 재개**할 수 있어야 완료다. 전문(trigger 분류·template·checklist·storage)은 [agent_docs/handoff.md](agent_docs/handoff.md) 이며 여기 복제하지 않는다.

- 단순 오타·단일 세션 short task 는 artifact 불필요. 다단계 작업은 §6.5 를 적용한다
- credentials · secret · raw 대용량 log · 미검증 주장은 넣지 않는다
- **저장**: plan 파일은 repo 에 커밋하지 않는다 — 각 에이전트가 자기 private 저장소에서 관리하고 (`## Spec` / `## Progress` / `## Handoff`), cross-tool 인계는 **git issue** 본문/코멘트에 artifact 를 적어 공유한다 (handoff.md §5). 완료된 plan 은 복원 가능하거나 보존 가치가 없으면 삭제 (§11)
- 도구별 세션 메커니즘 (compaction·clear·fork) 은 그 도구의 문서가 소유하며, handoff.md 계약과 충돌하면 **repo 계약이 우선**한다

## 7. Anti-patterns

최근 발현 빈도 Top: **AP-RT-1** (tick 에서 `RCLCPP_*`) · **AP-RT-3** (`auto` + Eigen) · **AP-ARCH-1** (`rtc_*` 에 robot 상수) · **AP-PROC-1** ("✅ complete" 후 미완료) · **AP-PROC-4** (test assertion 수정) · **AP-DOC-1** (헌법에 수치 박제). 전체 사례·복구·grep 은 [agent_docs/anti-patterns.md](agent_docs/anti-patterns.md) — invariant 과 **1:1 이 아니다** (각 AP 헤더가 위반한 invariant 를 명기; 사례가 없다고 그 룰이 약한 것은 아니다).

## 8. Where Things Live

패키지 역할·dependency graph·data flow·threading model: [agent_docs/architecture.md](agent_docs/architecture.md) 가 단일 출처 (위치 박제 금지).

## 9. Build & Run Hard Rules

명령 detail: [README.md](README.md#빠른-시작) · [repo_scripts/README.md](repo_scripts/README.md). 헌법은 두 가지 절대 규칙만 박는다. 위반의 재발 경로·증상·감별은 repo_scripts/README.md §흔한 실패와 감별 이 소유한다.

### 9.1 colcon CWD (Hard rule)

> **`colcon build` / `colcon test` 는 반드시 colcon workspace root (`<rtc_ws>` = `~/ros2_ws/rtc_ws`) 에서 실행한다.** repo 안에서 호출하면 `build/` · `install/` · `log/` 트리가 그 위치에 생겨 `.clangd` 와 ws-root incremental cache 를 오염시킨다. `build.sh` / `install.sh` 는 내부에서 `cd "$WORKSPACE"` 하므로 안전. 직접 `colcon` 을 칠 때는 **항상 `cd <rtc_ws>` 또는 절대경로 `--build-base` / `--install-base`**, 그리고 `setup_env.sh` 를 **절대경로**로 source. env 미source 상태의 `colcon`/`cmake` 는 silent fail 또는 즉시 비정상 종료다.

**표준형은 서브셸이다** — ws root cwd + 절대경로 source 를 만족하면서 **호출이 끝나면 cwd 를 원래대로 돌려준다**:

```bash
( cd <rtc_ws> && source <절대경로>/repo_scripts/scripts/setup_env.sh >/dev/null 2>&1 && colcon … )
```

- 에이전트 shell 의 **cwd 는 호출 간에 유지**된다 — 패키지 dir 로 `cd` 한 뒤 *다음* 호출의 colcon 도 위반이다. 증상은 빌드 실패가 아니라 **검증 결과의 상호 모순**이므로, 결과가 엇갈리면 코드보다 먼저 `ls src/rtc-framework/build` 를 친다
- **`source` 를 파이프라인에 넣지 말 것** — `source … | tail` 은 subshell 에서 실행돼 env 가 남지 않는다. 출력을 줄이려면 리다이렉션(`source … >/dev/null 2>&1`)
- post-incident 검증: `ls src/rtc-framework/{build,install,log}` — 존재하면 잘못된 cwd 에서 실행된 것이므로 삭제

### 9.2 `.venv` 격리 (Hard rule)

> **`.venv` 는 runtime PC 가 본 workspace 외에 다른 control project 들과 공존하는 환경에서 dependency 를 격리하기 위한 의도된 설계다.** venv 활성 상태에서 `colcon test` / `colcon build` / `ros2 run` / `ros2 launch` 가 실패하면 **반드시 근본 원인을 해결** (sys.path / shebang / wrapper / dep resolution 디버그). gtest binary 직접 실행, venv deactivate 후 colcon 호출, `PYTHONPATH` 강제 우회 등 **격리 무력화 우회 금지**.

**단 하나의 carve-out — `colcon build` 의 configure 단계**: venv 가 활성이면 CMake `FindPython` 이 venv 의 python 을 잡아 eigenpy/pinocchio configure 가 깨진다. 검증 우회가 아니라 빌드 시스템의 Python 탐색 문제이므로 이 경우에 한해 `deactivate` 가 허용된다. 다만 **인터프리터 고정 (`-DPython3_EXECUTABLE=/usr/bin/python3`) 을 우선**하고 (명령 형태는 repo_scripts/README.md "Plain `colcon build` 호환성"), `build.sh` 를 쓰면 둘 다 불필요하다. `colcon test` / `ros2 run` / `ros2 launch` 실패를 덮는 deactivate 는 **여전히 금지**.

## 10. Style Cheatsheet

상세: [agent_docs/conventions.md](agent_docs/conventions.md). 헌법은 절대 규칙만:

- **Namespace**: `rtc`
- **Naming**: Google C++ — `PascalCase` methods/types/free functions, `snake_case_` private members, `kConstant` constants
- **Units**: SI (m, rad, s, kg, N); degree 는 API 경계에서만
- **Rotation**: quaternion (`Eigen::Quaterniond`, Hamilton) internal, ZYX Euler at boundary
- **Variable naming**: paper notation — `J_b`, `q_d`, `K_d`
- **RAII**, `noexcept` on RT, `[[nodiscard]]` on status returns
- **Lifecycle**: 핵심 C++ 노드는 `rclcpp_lifecycle::LifecycleNode` — empty constructor, `on_configure` (Tier 1) + `on_activate` (Tier 2). 어떤 노드인지는 architecture.md (박제 금지)
- **Logger naming** (3-tier): node-owned = `<exec_name>` / library-level = `<full_package_name>` / controller-level = `<package>.<controller_key>`. 점 `.` 1개만 허용
- **Commits**: Conventional Commits `type(scope): subject`

## 11. Post-Task Housekeeping

Commit 완료 또는 사용자가 task 종료를 알린 후 (상세: [modification-guide.md](agent_docs/modification-guide.md) §Post-task housekeeping):

1. **완료 보고** — 변경 내용·영향 범위 · 실행한 build/test/format/review 와 결과 · 생략한 검증과 이유 · 남은 위험·후속·사용자 판단 사항. 실제 수행한 것만
2. **Issue 동기화** — 대응 issue 가 있으면 구현 내용·미충족 criteria·후속을 갱신하고, 전부 충족했을 때만 close (갱신 없이 닫지 않는다)
3. **Stale artifact·캐시 정리** — 완료된 private plan, scratch 파일, repo 안에 잘못 생긴 `build/`·`install/`·`log/` (§9.1), python 캐시는 삭제 (재생성 가능). **ws-root colcon 트리는 절대 건드리지 않는다**
4. **Branch prune** — `main` 에 merge 된 branch 만 로컬 삭제 + `git fetch --prune`; 미merge branch·`main` 은 건드리지 않는다
5. 도구별 memory·harness 정리는 그 도구의 문서가 소유한다 (Claude Code: CLAUDE.md)

## 12. Reference Docs (read when relevant)

- [architecture.md](agent_docs/architecture.md) — threading, data flow, core types, lifecycle, E-STOP, dep graph
- [controllers.md](agent_docs/controllers.md) — controller table, gains, FSM, topics, config
- [modification-guide.md](agent_docs/modification-guide.md) — workflow, adding X 절차, completion checklist, inferential 트리거, housekeeping 상세
- [design-principles.md](agent_docs/design-principles.md) — `rtc_*` 5 principles, boundary rules
- [conventions.md](agent_docs/conventions.md) — domain / code / commit conventions, doc requirements
- [testing-debug.md](agent_docs/testing-debug.md) — sensor matrix, test commands, debug topics, RT permissions
- [invariants.md](agent_docs/invariants.md) — RT / ARCH / PROC / NUM invariants, escalation triggers
- [anti-patterns.md](agent_docs/anti-patterns.md) — recurring mistakes, detection + recovery
- [handoff.md](agent_docs/handoff.md) — tool-neutral context handoff 계약
- [harness-rationale.md](agent_docs/harness-rationale.md) — 하네스 근거·출처, 5구성요소 표
- [CLAUDE.md](CLAUDE.md) — 이 파일을 import 하는 Claude Code 전용 addendum
- [README.md](README.md) — 빌드·설치 명령, deps 버전, 빠른 시작
- [repo_scripts/README.md](repo_scripts/README.md) — PREEMPT_RT, CPU shield, env, isolated deps, 흔한 실패와 감별
