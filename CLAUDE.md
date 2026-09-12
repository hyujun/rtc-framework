# CLAUDE.md

이 파일은 본 저장소의 **헌법 (constitution)** 이다. 안정적인 원칙·게이트·지표만 둔다. 자주 변하는 사실 (패키지 수, robot 목록, 의존성 버전, 명령 detail) 은 sub-doc / README 의 SSoT 를 참조한다 (AP-DOC-1). 규칙의 *근거·사고 이력* 도 두지 않는다 (AP-DOC-2) — 그 규칙을 소유한 문서·hook 헤더·anti-patterns 사례가 갖는다.

## 1. Snapshot

**RTC (Real-Time Control) Framework** — URDF 기반 매니퓰레이터를 위한 robot-agnostic real-time control framework. 변수 DOF, 설정 가능한 RT 루프 주기 (`control_rate` YAML — rate 범위·default 는 [agent_docs/invariants.md](agent_docs/invariants.md) §RT Path 가 SSoT), transport 추상화, lock-free SPSC, E-STOP.

- 패키지 구성·count·역할: [README.md](README.md#패키지-구성) · [architecture.md](agent_docs/architecture.md)
- 로봇 데이터 (URDF/MJCF/mesh): [robot_descriptions/README.md](robot_descriptions/README.md)
- 언어·OS·의존성 버전: [README.md](README.md#빠른-시작) · [repo_scripts/README.md](repo_scripts/README.md)
- test 카운트·실측: [testing-debug.md](agent_docs/testing-debug.md)

## 2. Harness Overview

에이전트 가이드는 **Agent = Model + Harness** 프레이밍의 5구성요소로 조직된다 (근거·출처·구성요소 표: [agent_docs/harness-rationale.md](agent_docs/harness-rationale.md)) — **Guides** §3·§10 · **Sensors** §5 (computational)·§5.5 (inferential) · **Orchestration** §4 · **Escalation** §6·§6.5 · **Enforcement** `.claude/hooks/` (`format-code.sh` PostToolUse · `verify-changes.sh` Stop, exit 2 차단 · `log-instructions-loaded.sh` InstructionsLoaded, 기록만).

**첫 방문 에이전트**: §3 → §4 → §6 순으로 읽고 작업 시작. **수정 작업 중**: §5 검증 + §6 escalation 확인. Invariant 위반 의심 시 즉시 §6.

## 3. Invariants (요약)

전체: [agent_docs/invariants.md](agent_docs/invariants.md). 규칙 목록을 여기 복제하지 않는다 — 사본이 원본과 갈라져 규칙이 조용히 사라진 사례가 두 번 있었다 (AP-DOC-1 본 repo 사례).

### RT path 절대 금지 (정기 tick — `control_rate` YAML)

- 규칙은 **RT-1 ~ RT-10** (RT-7 은 은퇴 → PROC-6). 전문은 invariants.md §RT Path Invariants, 편집 시 자동 로드되는 요약·예외·대안은 path-scoped rule [rt-path.md](.claude/rules/rt-path.md) (로드 조건은 그 frontmatter glob 이 SSoT)
- RT tick / SCHED_FIFO 경로에만 구속되고 lifecycle·aux·test·init 코드는 면제 (판정 절차는 rule·invariants.md). RT 코드 수정 전 반드시 확인하고, 위반 필요시 §6 `[CONCERN]`

### Architecture / Process / Numerical

- Architecture 규칙은 **ARCH-1 ~ ARCH-7**. 전문은 invariants.md §Architecture Invariants, 요약은 소스 축 [arch-source.md](.claude/rules/arch-source.md) (ARCH-1·2·3·4·6) · build metadata 축 [arch-build-meta.md](.claude/rules/arch-build-meta.md) (ARCH-2·5·7). **탐지 패턴의 SSoT 는 hook** 이고 rule 은 판정만 갖는다 — RT 와 반대 방향이다
- 새 utility 작성 전 기존 `rtc_*` 패키지에 유사 기능 검색 — 맞지 않으면 fork 대신 일반화 ([design-principles.md](agent_docs/design-principles.md) P5)
- 코드 변경 → 대응 문서·YAML·CMakeLists·package.xml 동기화 필수 (PROC-1)
- 기존 test assertion 을 통과시키려 **약화 금지** — 새 코드를 고치되, test 가 진짜 틀렸거나 spec 이 바뀌면 별도 commit + 근거 (PROC-6, §6 E-6)
- `rtc_base` / `rtc_msgs` 변경 시 전체 빌드·테스트 (PROC-3)
- 수치 특이점: damped pseudoinverse (NUM-1), zero guard (NUM-2, NUM-4)
- 폐쇄 체인 사영은 **residual 로 조립 분기를 판정할 수 없다** — 점 구속 loop 은 분기가 여럿이고 모두 φ=0 을 만족하므로 seed 증분 제한이 필수 (NUM-5). 완화 장치는 발동한 경우에만 적용하고, 그로 인한 `held` 를 자기 치유로 가정하지 않는다

**rule 채널은 두 센서로 검증한다** — glob 매칭은 `validate_claude_rules.py` (변경된 rule 한정, **0건이면 차단**), 실제 로드는 `.claude/instructions-loaded.log`. rule 을 새로 쓰거나 glob 을 고쳤으면 매칭 파일을 하나 열고 로그를 **자기 `session_id` 로 걸러** 발화를 확인한다 — rule 이 안 뜨는 실패는 증상이 "규칙이 조용히 없는 것" 뿐이다.


## 4. Workflow Loop

7단계: **Type → Locate → Read → Edit → Build → Test → Verify**. 규모에 맞춰 압축한다 — 오타·포매팅·자명한 단일 라인 수정은 단계를 합쳐도 되나, **검증(Build/Test/Verify)을 생략했다면 최종 보고에 무엇을·왜 생략했는지 명시**한다. 다파일·다패키지·`rtc_base`/`rtc_msgs` 변경에서 검증 단계를 건너뛰는 것은 §6 escalation 사유. 실패 시 절대 **"try harder" 금지** — 누락된 capability (test, lint, interface) 를 엔지니어링하거나 §6 escalate.

**Type 분기**: "수정" 인가 "추가 (새 기능 / 컨트롤러 / 메시지 / 디바이스 / 스레드 / 패키지)" 인가? **추가면 `adding-component` skill 이 진입점이다** — 절차 SSoT 는 [modification-guide.md](agent_docs/modification-guide.md) 이고 skill 은 그 앞의 판정(P5 일반화 · ARCH-3 · spec 필요 여부)과 발화하는 게이트만 갖는다. skill 이 없는 도구는 design-principles.md 5원칙 + modification-guide.md "Adding a New ..." 절을 먼저 읽는다 (rtc_* 추가는 P1·P2 + ARCH-3; integration package 또는 `shape_estimation*` 추가 시 rtc_* 일반화 가능성부터 검토).

**계획 전 분석**: 대응하는 GitHub issue 가 있으면 계획 전에 그 issue (본문 + 코멘트) 를 먼저 참고한다 — issue 는 durable 결정 기록이자 cross-tool 인계면이다 (§6.6). 단 issue 의 진단·근거는 **미검증 가설**로 취급하고 착수 전 grep/코드로 반증한다 (틀렸으면 issue 를 먼저 갱신). 완료 후 갱신 규칙은 §11.

**4·5·6 자동화**: [verify-changes.sh](.claude/hooks/verify-changes.sh) Stop hook 이 turn 종료 시 변경 패키지만 빌드·테스트하고 hard failure 시 `exit 2` 로 차단한다. 변경 집합은 **마지막으로 통과한 커밋(watermark) 기준 `git diff` ∪ untracked** 이므로 turn 안에서 commit 한 변경도 검증된다. timeout·launch 실패는 **"미검증" 으로 차단** (silent pass 아님). 변경 `.md` 는 `validate_docs.py --files`, 변경 YAML 은 parse 검사를 받고 (전체 코퍼스 스캔은 CI), Doxygen 은 에이전트가 직접 검증한다. **검사 범위·blocking/non-blocking 구분·allowlist·bound 의 SSoT 는 hook 헤더 주석이다.**

단계별 액션·grep 패턴·Completion Checklist: [modification-guide.md](agent_docs/modification-guide.md).

## 5. Sensors

**변경 위치별 sensor matrix·명령·Live Debug Topics**: [agent_docs/testing-debug.md](agent_docs/testing-debug.md) 가 단일 출처.

- 변경 패키지의 sensor 행을 testing-debug.md 에서 찾아 **필수 sensor + 추가 sensor** 모두 실행
- 실패하면 다음 turn 의 verify-changes.sh hook 이 차단
- `rtc_base` / `rtc_msgs` 변경 시 전체 downstream 검증 (PROC-3)

## 5.5 Inferential Sensors (LLM-as-judge, 수동 trigger)

§5 의 computational sensor 는 **문법·빌드·기존 테스트 통과** 만 검증하고 의미 회귀 (설계 일관성, robot-agnostic, interface 누락, 재사용성) 는 잡지 못한다 — 에이전트 자기 평가는 신뢰 불가. 다음에 해당하면 사용자에게 실행을 권한다 (미지원 툴은 동등한 수동 review):

- `/code-review`: `rtc_base`/`rtc_msgs` 변경 · abstract interface 신설/두 번째 구현 (ARCH-3) · `rtc_*` 에 robot-specific 의심 (ARCH-1) · 100+ 줄 또는 신규 패키지
- `/security-review`: E-STOP 경로 / safety publisher / lifecycle 콜백 수정 (E-8)
- `/code-review ultra` (현재 branch 또는 `<PR#>`): PR 준비 (다파일 / 다패키지); `/ultrareview` 는 deprecated alias
- `/simplify`: 다파일 리팩터 / 유사 기능 중복 의심 (P5) / 변경 후 정리 (재사용·단순화 전용 — 버그 탐지는 `/code-review`)

트리거 표 전문·수동 trigger 인 이유: [modification-guide.md](agent_docs/modification-guide.md) §Inferential review 트리거.

## 6. Escalation Triggers

다음 상황에서 코드를 쓰기 **전에** `[CONCERN]` 보고 후 사용자 컨펌 대기. **E-1 ~ E-11 트리거 표·severity·`[CONCERN]` 포맷의 SSoT 는 [agent_docs/invariants.md](agent_docs/invariants.md) §Escalation Triggers 다** — 헌법에 복제하지 않는다 (AP-DOC-1). 헌법이 박는 것은 severity 의 **효력**뿐이다:

- **Critical**: 사용자 컨펌 전까지 커밋·PR 금지
- **Warning**: 사용자 판단에 따라 진행, 결정 로그 남김
- **Info**: 기록만, 진행 가능

Critical 은 E-1(invariant 일반)·E-2(ARCH-1)·E-3(msgs ABI)·E-6(test assertion)·E-7(thread model)·E-8(E-STOP), Warning 은 E-4·E-5·E-9·E-10·E-11 이다.

## 6.5 Sprint Contract (착수 전 성공 기준 협상)

다음 task 에서는 코드 수정 시작 *전* 1~3줄로 **객관 검증 가능한** 성공 기준 (`[SPRINT]` 포맷) 을 제시하고 컨펌받는다 — 에이전트 자기 평가는 신뢰 불가하므로 평가 기준을 generation *전* 명시한다. 포맷·spec 절차·예시는 [modification-guide.md](agent_docs/modification-guide.md) §Sprint Contract & Spec.

- 다단계 task (PR 단위 / 다파일 / 다패키지 / 신규 디렉토리 / phase 분할)
- 신규 abstract interface · controller / device group / thread / message 추가 — 이 경우 **Sprint Contract = spec**, `~/.claude/plans/<slug>.md` 에 박음
- `rtc_base` / `rtc_msgs` 변경 · 리팩터 (기능 동등성 유지가 곧 success)

면제: 단일 파일 bug fix, 오타·포매팅, 단일 함수 추가, 사용자 의도가 1줄 메시지에서 자명한 경우.

## 6.6 Long-running task — Context handoff

**Handoff 계약**의 tool-neutral 단일 출처는 [agent_docs/handoff.md](agent_docs/handoff.md) 다 ([AGENTS.md](AGENTS.md) §9 는 요약). 이 절은 **Claude 전용 메커니즘**만 소유한다.

- 능동 제안 트리거·메커니즘 선택 (`/rename`+`/clear` / `/compact <focus>` / subagent / `/btw` / fork)·보존 우선순위는 user-level CLAUDE.md 가 SSoT. **handoff.md 계약과 충돌하면 repo 계약이 우선**한다.
- 반복 실패: handoff.md 의 "3회 시도 → 중단·escalate" 와 Claude 세션의 "2회 초과 교정 → `/clear`" 는 다른 축이며 공존한다.
- **RTC storage override** — plan 파일은 repo 에 커밋하지 않는다. Claude 전용 plan 은 `~/.claude/plans/<task-slug>.md` (§6.5 spec 과 동일 파일, `## Spec` / `## Progress` / `## Handoff`), cross-tool 인계는 **git issue** 본문/코멘트에 artifact 를 적어 공유한다 (handoff.md §5). 완료된 plan 은 복원 가능하면 삭제 (§11).

## 7. Anti-patterns

최근 발현 빈도 Top: **AP-RT-1** (tick 에서 `RCLCPP_*`) · **AP-RT-3** (`auto` + Eigen) · **AP-ARCH-1** (`rtc_*` 에 robot 상수) · **AP-PROC-1** ("✅ complete" 후 미완료) · **AP-PROC-4** (test assertion 수정) · **AP-DOC-1** (헌법에 수치 박제). 전체 사례·복구·grep 은 [agent_docs/anti-patterns.md](agent_docs/anti-patterns.md) — invariant 과 **1:1 이 아니다**: 각 AP 헤더가 위반한 invariant 를 명기하므로 §3 룰 → 사례 방향으로 읽되, 사례가 없다고 그 룰이 약한 것은 아니다.

## 8. Where Things Live

패키지 역할·dependency graph·data flow·threading model: [agent_docs/architecture.md](agent_docs/architecture.md) 가 단일 출처 (위치 박제 금지).

## 9. Build & Run Hard Rules

명령 detail: [README.md](README.md#빠른-시작) · [repo_scripts/README.md](repo_scripts/README.md). 본 헌법은 두 가지 절대 규칙만 박는다. 위반의 재발 경로·증상·감별은 repo_scripts/README.md §흔한 실패와 감별 이 소유한다.

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

> **`.venv` 는 runtime PC 가 본 workspace 외에 다른 control project 들과 공존하는 환경에서 dependency 를 격리하기 위한 의도된 설계다.** venv 활성 상태에서 `colcon test` / `colcon build` / `ros2 run` / `ros2 launch` 가 실패하면 **반드시 근본 원인을 해결** (sys.path / shebang / wrapper / dep resolution 디버그). gtest binary 직접 실행, venv deactivate 후 colcon 호출, `PYTHONPATH` 강제 우회 등 **격리 무력화 우회 금지** (runtime PC 의 silent breakage 경로).

**단 하나의 carve-out — `colcon build` 의 configure 단계**: venv 가 활성이면 CMake `FindPython` 이 venv 의 python 을 잡아 eigenpy/pinocchio configure 가 깨진다. 검증 우회가 아니라 빌드 시스템의 Python 탐색 문제이므로 이 경우에 한해 `deactivate` 가 허용된다. 다만 **인터프리터 고정 (`-DPython3_EXECUTABLE=/usr/bin/python3`) 을 우선**하고 (명령 형태는 repo_scripts/README.md "Plain `colcon build` 호환성"), `build.sh` 를 쓰면 둘 다 불필요하다. `colcon test` / `ros2 run` / `ros2 launch` 실패를 덮는 deactivate 는 **여전히 금지**.

## 10. Style Cheatsheet

상세: [agent_docs/conventions.md](agent_docs/conventions.md). 본 헌법은 변경 빈도가 낮은 절대 규칙만:

- **Namespace**: `rtc`
- **Naming**: Google C++ — `PascalCase` methods/types/free functions, `snake_case_` private members, `kConstant` constants
- **Units**: SI (m, rad, s, kg, N) — degree 는 API 경계에서만
- **Rotation**: quaternion (`Eigen::Quaterniond`, Hamilton) internal, ZYX Euler at boundaries
- **Variable naming**: paper notation — `J_b` (body Jacobian), `q_d` (desired joint), `K_d` (stiffness)
- **RAII**, `noexcept` on RT, `[[nodiscard]]` on status returns
- **Lifecycle**: 핵심 C++ 노드는 `rclcpp_lifecycle::LifecycleNode` — empty constructor, `on_configure` (Tier 1) + `on_activate` (Tier 2). 어떤 노드인지는 architecture.md (박제 금지)
- **Logger naming** (3-tier): node-owned = `<exec_name>` / library-level = `<full_package_name>` / controller-level = `<package>.<controller_key>`. 점 `.` 1개만 허용
- **Commits**: Conventional Commits `type(scope): subject`

## 11. Post-Task Housekeeping

Commit 완료 또는 사용자가 task 종료를 알린 후 (상세: [modification-guide.md](agent_docs/modification-guide.md) §Post-task housekeeping):

1. **Memory save / prune / Harness pruning 신호 보고** — user-level CLAUDE.md `# Post-task housekeeping` 가 SSoT. RTC 신호 카테고리: grep false-positive, hook 오차단, agent_docs 간 규칙 중복 drift
2. **Issue 동기화** — 대응 issue 가 있으면 구현 내용·미충족 criteria·후속을 갱신하고, 전부 충족했을 때만 close (갱신 없이 닫지 않는다)
3. **Stale artifact·캐시 정리** — 완료된 private plan, scratch 파일, repo 안에 잘못 생긴 `build/`·`install/`·`log/` (§9.1), python 캐시는 삭제. **ws-root colcon 트리는 절대 건드리지 않는다**
4. **Branch prune** — `main` 에 merge 된 branch 만 로컬 삭제 + `git fetch --prune`; 미merge branch·`main` 은 건드리지 않는다
5. **보고** — 실제 수행한 항목만 한 줄씩

## 12. Reference Docs (read when relevant)

- [architecture.md](agent_docs/architecture.md) — threading, data flow, core types, lifecycle, E-STOP, dependency graph
- [controllers.md](agent_docs/controllers.md) — controller table, gains, GraspController FSM, topics, config
- [modification-guide.md](agent_docs/modification-guide.md) — workflow loop, adding X 절차, completion checklist, inferential 트리거, housekeeping 상세
- [design-principles.md](agent_docs/design-principles.md) — `rtc_*` 5 principles, boundary rules
- [conventions.md](agent_docs/conventions.md) — domain / code / commit conventions, doc requirements
- [testing-debug.md](agent_docs/testing-debug.md) — sensor matrix, test commands, live debug topics, RT permissions
- [invariants.md](agent_docs/invariants.md) — RT / ARCH / PROC / NUM invariants, escalation triggers
- [anti-patterns.md](agent_docs/anti-patterns.md) — recurring mistakes, detection + recovery
- [handoff.md](agent_docs/handoff.md) — tool-neutral context handoff 계약
- [harness-rationale.md](agent_docs/harness-rationale.md) — 하네스 근거·출처, 5구성요소 표
- [AGENTS.md](AGENTS.md) — 같은 헌법의 tool-neutral 판. 규칙 변경 시 양쪽 동기화 필요
- [README.md](README.md) — 빌드·설치 명령, deps 버전, 빠른 시작
- [repo_scripts/README.md](repo_scripts/README.md) — PREEMPT_RT, CPU shield, env, isolated deps, 흔한 실패와 감별
