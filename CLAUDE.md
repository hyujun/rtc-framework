# CLAUDE.md

이 파일은 본 저장소의 **헌법 (constitution)** 이다. 안정적인 원칙·게이트·지표만 둔다. 자주 변하는 사실 (패키지 수, robot 목록, 의존성 버전, 명령 detail) 은 sub-doc / README 의 SSoT 를 참조한다 — CLAUDE.md 에 박제하지 않는다 ([agent_docs/anti-patterns.md](agent_docs/anti-patterns.md) AP-DOC-1).

## 1. Snapshot

**RTC (Real-Time Control) Framework** — URDF 기반 매니퓰레이터를 위한 robot-agnostic real-time control framework. 변수 DOF, 설정 가능한 RT 루프 주기 (`control_rate` YAML — rate 범위·default 는 [agent_docs/invariants.md](agent_docs/invariants.md) §RT Path 가 SSoT), transport 추상화 (UDP/CAN-FD/EtherCAT/RS485 등), lock-free SPSC, E-STOP.

- 패키지 구성·count·역할: [README.md](README.md#패키지-구성) · [agent_docs/architecture.md](agent_docs/architecture.md)
- 로봇 데이터 (URDF/MJCF/mesh, multi-robot data hub): [robot_descriptions/README.md](robot_descriptions/README.md)
- 언어·OS·의존성 버전: [README.md](README.md#빠른-시작) · [repo_scripts/README.md](repo_scripts/README.md)
- 최신 test 카운트·실측: 단일 출처 [agent_docs/testing-debug.md](agent_docs/testing-debug.md)

## 2. Harness Overview

이 저장소의 에이전트 가이드는 **agent-driven engineering** (harness engineering + spec-driven development + Anthropic 2026 agentic SDLC patterns) 의 5구성요소로 조직되어 있다. Agent = Model + Harness이며, 모델 바깥의 guides / sensors / orchestration / escalation / enforcement가 본 저장소의 1급 자산이다.

근거·출처 (학술 문헌·Anthropic 공식 memory 가이드): [agent_docs/harness-rationale.md](agent_docs/harness-rationale.md).

| 구성요소 | 목적 | 진입점 |
|---|---|---|
| **Guides** (feedforward) | 규칙·원칙·패턴 | §3 Invariants, §10 Style, [agent_docs/invariants.md](agent_docs/invariants.md), [agent_docs/design-principles.md](agent_docs/design-principles.md), [agent_docs/conventions.md](agent_docs/conventions.md) |
| **Sensors** (feedback, computational) | 변경 검증 (결정적·빠른) | §5, [agent_docs/testing-debug.md](agent_docs/testing-debug.md), `[CONCERN]` 포맷 (§6) |
| **Sensors** (feedback, inferential) | 의미 검증 (LLM-as-judge, on-demand) | §5.5 |
| **Orchestration** | Workflow | §4, [agent_docs/modification-guide.md](agent_docs/modification-guide.md) |
| **Escalation** | Human gate | §6, §6.5 Sprint Contract |
| **Enforcement** (자동) | 무인 실행/차단 | [.claude/hooks/format-code.sh](.claude/hooks/format-code.sh) (PostToolUse: clang-format / ruff), [.claude/hooks/verify-changes.sh](.claude/hooks/verify-changes.sh) (Stop: doc·CMake·build·test gate, exit 2 차단) |

**첫 방문 에이전트**: §3 → §4 → §6 순으로 읽고 작업 시작.
**수정 작업 중**: §5 검증 + §6 escalation 확인. Invariant 위반 의심 시 즉시 §6.

## 3. Invariants (요약)

전체: [agent_docs/invariants.md](agent_docs/invariants.md).

### RT path 절대 금지 (정기 tick — `control_rate` YAML)

RT 핫패스 절대금지 규칙은 **RT-1 ~ RT-10** (RT-7 은 은퇴 → PROC-6) 이며, 전문은 [agent_docs/invariants.md](agent_docs/invariants.md) §RT Path Invariants, 편집 시 자동 로드되는 요약은 **path-scoped rule** [.claude/rules/rt-path.md](.claude/rules/rt-path.md) 에 있다 (예외·대안 포함). 목록 자체를 여기 박제하지 않는 이유는 AP-DOC-1 이며, 실제로 과거에 여기 박힌 7개 목록이 invariants.md 의 9개와 갈라져 RT-9·RT-10 이 헌법에서 사라진 적이 있다 — 어떤 파일 편집 시 로드되는지는 그 rule 의 frontmatter glob 이 SSoT (AP-DOC-1: 여기 박제 금지). 이 규칙은 RT tick / SCHED_FIFO 경로에만 구속되고 lifecycle·aux·test·init 코드는 면제 (판정 절차는 rule 파일·invariants.md). RT 코드 수정 전 반드시 확인하고, 위반 필요시 §6 `[CONCERN]`.

### Architecture / Process / Numerical

- `rtc_*` 패키지에 robot name / joint count / HW ID 하드코딩 금지 (ARCH-1)
- 의존성 그래프 상향 의존 금지 (ARCH-2)
- 두 번째 구체 구현은 abstract interface / concept 정의 후에만 추가 (ARCH-3) — `#ifdef` / hardcoded switch 금지
- `rtc_*` 는 RT 제어 루프를 구동하는 exec 를 소유하지 않는다 (ARCH-7) — 진입 *함수* 만 export. robot-agnostic standalone 노드·example 은 예외 ([agent_docs/design-principles.md](agent_docs/design-principles.md))
- `robot_descriptions` 는 data-only 패키지 — 소비자는 `<exec_depend>` + ament_index 런타임 lookup만 (ARCH-5)
- 새 utility 작성 전 기존 `rtc_*` 패키지에 유사 기능 검색 — 맞지 않으면 fork 대신 일반화 ([agent_docs/design-principles.md](agent_docs/design-principles.md) P5)
- 코드 변경 → 대응 문서·YAML·CMakeLists·package.xml 동기화 필수 (PROC-1)
- 기존 test assertion 을 통과시키려 **약화 금지** — 새 코드를 고치되, test 가 진짜 틀렸거나 spec 이 바뀌면 별도 commit + 근거 (PROC-6, §6 E-6)
- `rtc_base` / `rtc_msgs` 변경 시 전체 빌드·테스트 (PROC-3)
- 수치 특이점: damped pseudoinverse (NUM-1), zero guard (NUM-2, NUM-4)
- 폐쇄 체인 사영은 **residual 로 조립 분기를 판정할 수 없다** — 점 구속 loop 은 분기가 여럿이고 모두 φ=0 을 만족하므로 seed 증분 제한이 필수 (NUM-5). 완화 장치를 넣을 때는 발동한 경우에만 적용하고, 그로 인한 `held` 를 자기 치유로 가정하지 않는다

세부 규칙·grep 패턴·복구 절차: [agent_docs/invariants.md](agent_docs/invariants.md). 위반 필요시 §6 Escalation 의 `[CONCERN]` 포맷 보고.

## 4. Workflow Loop

7단계: **Type → Locate → Read → Edit → Build → Test → Verify**. 규모에 맞춰 압축한다 — 오타·포매팅·자명한 단일 라인 수정은 단계를 합쳐도 되나, **검증(Build/Test/Verify)을 생략했다면 최종 보고에 무엇을·왜 생략했는지 명시**한다. 다파일·다패키지·`rtc_base`/`rtc_msgs` 변경에서 검증 단계를 건너뛰는 것은 §6 escalation 사유. 실패 시 절대 **"try harder" 금지** — 누락된 capability (test, lint, interface) 를 엔지니어링하거나 §6 escalate.

**Type 분기**: "수정" 인가 "추가 (새 기능 / 컨트롤러 / 메시지 / 디바이스 / 스레드)" 인가? 추가 task 는 단계 1 진입 전에 [agent_docs/design-principles.md](agent_docs/design-principles.md) 5원칙 + [agent_docs/modification-guide.md](agent_docs/modification-guide.md) "Adding a New ..." 절을 먼저 읽는다 (rtc_* 추가는 P1·P2 + ARCH-3 결합; integration package 또는 `shape_estimation*` 추가 시 rtc_* 일반화 가능성부터 검토).

**계획 전 분석**: 대응하는 GitHub issue 가 있으면 계획을 세우기 전에 그 issue (본문 + 코멘트) 를 먼저 참고한다 — issue 는 durable 결정 기록이자 cross-tool 인계면이므로 (§6.6, [agent_docs/handoff.md](agent_docs/handoff.md) §5) 이전 세션·다른 tool 의 acceptance criteria·결정·미완료 상태가 거기 남아 있다. 단 issue 본문의 진단·근거는 **미검증 가설**로 취급하고 착수 전 grep/코드로 반증한다 (틀렸으면 issue 를 먼저 갱신). 구현 완료 후 그 issue 를 갱신하는 규칙은 §11.

**4·5·6 자동화**: [.claude/hooks/verify-changes.sh](.claude/hooks/verify-changes.sh) Stop hook 이 turn 종료 시 자동 실행하고 hard failure 시 `exit 2` 로 다음 turn 까지 차단한다 (loop 방지는 `stop_hook_active` 가드; stop cycle 당 1회 발화). 변경 패키지만 빌드·테스트하며 **build/test 의 timeout·launch 실패는 "미검증"으로 차단** (silent pass 아님 — bound 초과 test 는 hook 의 timeout 상향). 변경 집합은 `git diff HEAD` **∪ untracked** 이며, build/test 는 **위치 축**으로 좁힌다 — untracked 라도 `<pkg>/src/`·`<pkg>/include/`·`<pkg>/<pkg>/` 아래 소스는 빌드·테스트 대상이고, 그 밖의 untracked scratch 만 제외된다 (tracked-ness 축은 신규 헤더를 검증 없이 통과시켜 폐기됨 — hook 주석이 SSoT). README co-update 는 public surface (header/launch/config/파일 add·del/dep) 변경 시 **non-blocking checklist** (내부 리팩터·bug fix 는 미요구); CMake/`package.xml` co-update 는 blocking. 변경된 `.md` 는 `validate_docs.py --files` (변경 파일 한정), 변경된 YAML 은 parse 검사를 받는다 — 전체 코퍼스 스캔은 CI 몫이다. Doxygen 은 에이전트가 직접 검증. Pure-format commit (clang-format / ruff round-trip 동치) 은 ARCH grep + doc 단계만 skip, build/test 는 그대로.

단계별 액션·grep 패턴·Completion Checklist: [agent_docs/modification-guide.md](agent_docs/modification-guide.md).

## 5. Sensors

**변경 위치별 sensor matrix·명령·Live Debug Topics**: [agent_docs/testing-debug.md](agent_docs/testing-debug.md) 가 단일 출처. 매트릭스를 CLAUDE.md 에 박제하면 두 곳 동기화 부담으로 drift 가 발생하므로, 본 헌법은 위임만 한다.

원칙:
- 변경 패키지의 sensor 행을 testing-debug.md 에서 찾아 **필수 sensor + 추가 sensor** 모두 실행
- 실패하면 다음 turn 의 verify-changes.sh hook 이 차단
- `rtc_base` / `rtc_msgs` 변경 시 전체 downstream 검증 (PROC-3)

## 5.5 Inferential Sensors (LLM-as-judge, 수동 trigger)

§5 의 computational sensor (build / test / grep) 는 **문법·빌드·기존 테스트 통과** 만 검증한다. 의미 회귀 — 설계 일관성, robot-agnostic 위반, abstract interface 누락, 재사용 가능성 — 은 잡지 못한다 (Anthropic 2026.04 *Harness design*: 에이전트의 자기 평가는 신뢰 불가).

다음 상황에서 사용자에게 inferential sensor 실행을 권한다 (`/code-review`·`/security-review` 는 Claude Code slash command — 미지원 환경/툴에서는 동등한 수동 code review 로 대체):

- `rtc_base` / `rtc_msgs` 변경 → `/code-review` (downstream 전 패키지 영향)
- Abstract interface 신설 / 두 번째 구현 추가 (ARCH-3 후보) → `/code-review` (base 누락·#ifdef 유혹 검출)
- `rtc_*` 에 robot-specific 코드 추가 의심 (ARCH-1 borderline) → `/code-review`
- E-STOP 경로 / safety publisher / lifecycle 콜백 수정 → `/security-review` (E-8)
- PR 준비 (다파일 / 다패키지 commit) → `/code-review ultra` (현재 branch) 또는 `/code-review ultra <PR#>` (GitHub PR). `/ultrareview` 는 deprecated alias
- 100+ 줄 변경 또는 신규 패키지 디렉토리 → `/code-review`
- 다파일 리팩터 / 유사 기능 중복 의심 ([agent_docs/design-principles.md](agent_docs/design-principles.md) P5) / 변경 후 정리 → `/simplify` (재사용·단순화 전용 — 버그 탐지는 `/code-review`)

수동 trigger 인 이유: inferential 은 GPU/cost/지연이 크고 non-deterministic 이므로 모든 변경에 자동 적용하면 ROI 음성. 위 trigger 는 "false-negative 비용 > inferential 비용" 인 경우만 추렸다.

## 6. Escalation Triggers

다음 상황에서 코드를 쓰기 **전에** `[CONCERN]` 보고 후 사용자 컨펌 대기.

**E-1 ~ E-11 트리거 표·severity·`[CONCERN]` 포맷의 SSoT 는 [agent_docs/invariants.md](agent_docs/invariants.md) §Escalation Triggers 다** — tool-neutral 이므로 헌법에 복제하지 않는다 (AP-DOC-1; 이전에 CLAUDE.md·AGENTS.md 가 각각 전체 목록을 들고 있었고 같은 구조가 handoff 섹션 목록에서 실제 드리프트를 냈다).

헌법이 박는 것은 severity 의 **효력**뿐이다:

- **Critical**: 사용자 컨펌 전까지 커밋·PR 금지
- **Warning**: 사용자 판단에 따라 진행, 결정 로그 남김
- **Info**: 기록만, 진행 가능

Critical 은 E-1(invariant 일반)·E-2(ARCH-1)·E-3(msgs ABI)·E-6(test assertion)·E-7(thread model)·E-8(E-STOP), Warning 은 E-4·E-5·E-9·E-10·E-11 이다. 각 트리거의 정확한 조건·관련 규칙 ID 는 위 SSoT 를 연다.

## 6.5 Sprint Contract (착수 전 성공 기준 협상)

다음 task 에서는 코드 수정 시작 *전* 1~3줄로 **객관 검증 가능한** 성공 기준 (`[SPRINT]` 포맷) 을 제시하고 컨펌받는다 — 에이전트 자기 평가는 신뢰 불가하므로 평가 기준을 generation *전* 명시한다. 포맷·spec 절차·예시는 [agent_docs/modification-guide.md](agent_docs/modification-guide.md) §Sprint Contract & Spec.

- 다단계 task (PR 단위 / 다파일 / 다패키지 / 신규 디렉토리 / phase 로 쪼개진 작업)
- 신규 abstract interface · 새 controller / device group / thread / message 추가 (이 경우 **Sprint Contract = spec**, `~/.claude/plans/<slug>.md` 에 박음)
- `rtc_base` / `rtc_msgs` 변경 (downstream broad impact)
- 리팩터 (기능 동등성 유지가 곧 success)

면제: 단일 파일 bug fix, 오타·포매팅, 단일 함수 추가, 사용자 의도가 1줄 메시지에서 자명한 경우.

## 6.6 Long-running task — Context handoff

**Handoff 계약**(무엇이 handoff 이고, artifact 가 무엇을 담고, 받는 쪽이 어떻게 재개하는가 — trigger 분류표·artifact template·sender/receiver checklist·storage)의 tool-neutral 단일 출처는 [agent_docs/handoff.md](agent_docs/handoff.md) 다. [AGENTS.md](AGENTS.md) §9 는 그 요약 계약을 담는다. 이 절은 **Claude 전용 메커니즘**만 소유한다.

- 능동 제안 트리거·메커니즘 선택 (`/rename`+`/clear` / `/compact <focus>` / subagent / `/btw` / fork)·보존 우선순위 (`# Compact instructions`)의 글로벌 기본값은 user-level CLAUDE.md `# Context handoff policy` 가 SSoT. **handoff.md 의 tool-neutral 계약과 충돌하면 repo 계약이 우선**한다.
- 반복 실패: handoff.md 는 "3회 시도 → 중단·진단·escalate"(tool-neutral), Claude 세션은 "동일 문제 2회 초과 교정 → `/clear`"(context 위생) — 다른 축, 공존.

**RTC storage override** — plan 파일은 repo 에 커밋하지 않는다. Claude 전용 plan 은 `~/.claude/plans/<task-slug>.md` (§6.5 Sprint Contract spec 과 동일 파일, `## Spec` / `## Progress` / `## Handoff` 섹션) 에서 스스로 관리하고, 다른 tool(Codex 등)로 넘어가는 cross-tool 인계는 **git issue** 본문/코멘트에 artifact 를 적어 공유한다 (storage·retention 은 handoff.md §5, template 은 §2). 완료된 plan 은 git log / issue / memory 로 복원 가능하거나 보존할 가치가 없으면 삭제 (§11).

## 7. Anti-patterns

최근 발현 빈도 Top: **AP-RT-1** (정기 tick `RCLCPP_*`) · **AP-RT-3** (`auto` + Eigen) · **AP-ARCH-1** (`rtc_*` 에 robot 상수) · **AP-PROC-1** ("✅ complete" 후 미완료) · **AP-PROC-4** (test assertion 수정) · **AP-DOC-1** (CLAUDE.md 에 패키지 수·테스트 수 박제). 전체 사례·복구·grep 은 [agent_docs/anti-patterns.md](agent_docs/anti-patterns.md) — invariant 과 **1:1 이 아니다** (사례가 없는 규칙도, 여러 규칙에 걸치거나 어느 규칙에도 안 걸리는 사례도 있다). 각 AP 항목이 자기 헤더에 위반한 invariant 를 명기하므로 그 방향으로 읽는다: §3 에서 룰을 보고 anti-patterns 에서 *그 룰을 위반한 실제 commit* 을 찾되, 없다고 해서 그 룰이 약한 것은 아니다.

## 8. Where Things Live

패키지 역할·dependency graph·data flow·threading model: [agent_docs/architecture.md](agent_docs/architecture.md) 가 단일 출처. 본 헌법은 위치 박제를 두지 않는다 (패키지 추가·rename 시 drift 방지).

## 9. Build & Run Hard Rules

명령 detail (build.sh 옵션, source 순서, deps 버전, plain colcon build 호환): [README.md](README.md#빠른-시작) · [repo_scripts/README.md](repo_scripts/README.md). 본 헌법은 두 가지 절대 규칙만 박는다.

### 9.1 colcon CWD (Hard rule)

> **`colcon build` / `colcon test` 는 반드시 colcon workspace root (`<rtc_ws>` = `~/ros2_ws/rtc_ws`) 에서 실행한다.** repo (`src/rtc-framework`) 안에서 호출하면 `build/` · `install/` · `log/` 트리가 그 위치에 생기고 — `.clangd` 의 CompilationDatabase 가 잘못된 트리를 가리키며 ws-root incremental cache 와 분리되어 추적 불가한 stale state 가 누적된다. `build.sh` / `install.sh` 는 내부에서 `cd "$WORKSPACE"` 하므로 안전. 직접 `colcon` 을 칠 때는 **항상 `cd <rtc_ws>` 또는 절대경로 `--build-base` / `--install-base` 지정**, 그리고 `source ${repo_ws}/repo_scripts/scripts/setup_env.sh` (`${repo_ws}` = `<rtc_ws>/src/rtc-framework` = `~/ros2_ws/rtc_ws/src/rtc-framework`; `repo_scripts` 는 repo 안에 있으므로 ws-root cwd 기준 상대경로 `repo_scripts/...` 는 안 풀린다 — 절대경로 또는 이 prefix 필수). env 미source 상태로 `colcon`/`cmake` 호출 시 컴파일러·ROS·deps·venv PATH 누락으로 `colcon test` 가 silent fail 하거나 build 가 즉시 비정상 종료한다.

실제 위반은 룰을 몰라서가 아니라 **cwd drift** 로 재발한다 — 편집하러 패키지 dir 로 `cd` 한 shell 에서 그대로 colcon 을 치거나, `cd src/rtc-framework && source src/rtc-framework/...` 처럼 한 줄에 체이닝해 상대경로 source 가 silent fail 한 채 빌드가 repo 안에서 도는 경로다.

**표준형은 서브셸이다** — 이 한 줄이 위 두 요구(ws root cwd + 절대경로 source)를 만족하면서 **호출이 끝나면 cwd 를 원래대로 돌려준다**:

```bash
( cd <rtc_ws> && source <절대경로>/repo_scripts/scripts/setup_env.sh >/dev/null 2>&1 && colcon … )
```

`cd <rtc_ws> &&` 로 시작하기만 하면 규칙은 지켜지지만 **그 다음 호출부터 셸 cwd 가 ws root 에 남는다**. 그 부작용이 아래층에서 "cd 금지" 라는 반대 규율을 만들어 냈고(실제로 #345 인계 노트가 그랬다), 두 규칙이 충돌하는 상태로 시작한 세션이 있었다. 서브셸은 그 충돌 자체를 없앤다. `source` 와 `colcon` 이 **같은 서브셸 안**에 있으므로 아래 "파이프라인 금지" 와도 상충하지 않는다 — 문제는 서브셸이 아니라 *파이프*가 만드는 서브셸에 colcon 이 안 들어가는 것이다.

"독립 call 로 내면 된다" 는 오해다 — **에이전트 shell 의 cwd 는 호출 간에 유지**되므로, grep 하나 하려고 repo 로 `cd` 한 뒤 *다음* 호출에서 colcon 을 치면 그게 이미 위반이다. 증상이 빌드 실패가 아니라는 점이 이 경로를 비싸게 만든다: repo 안에 별도 트리가 생기고 그 뒤로는 호출마다 두 트리를 오가므로, **같은 세션의 검증들이 서로 모순되는 결과를 낸다** (한 test 는 수정 후 코드로, 다른 test 는 stale 바이너리로 도는 식). 코드에 없는 논리 버그를 쫓게 되니, 검증 결과가 설명 불가하게 엇갈리면 코드를 의심하기 전에 `ls src/rtc-framework/build` 부터 친다.

**`source` 를 파이프라인에 넣지 말 것** — 출력을 줄이려 `source setup_env.sh 2>&1 | tail -2 && colcon build …` 처럼 쓰면 source 가 **subshell 에서 실행돼 env 가 부모 셸에 반영되지 않는다**. 조용히 성공한 것처럼 보이는 게 함정이다: `colcon` 자체는 profile PATH 에 있어 빌드가 정상 시작하고, 한참 뒤 CMake 안에서 `ModuleNotFoundError: No module named 'ament_package'` 같은 **원인을 가리키지 않는 에러**로 죽는다 (`ament_package` 는 dpkg 가 아니라 `/opt/ros/jazzy/lib/python3.12/site-packages` 에 있고 `PYTHONPATH` 로만 노출되므로). 출력을 줄이려면 리다이렉션(`source … >/dev/null 2>&1`)을 쓰고 파이프는 뒤따르는 명령에만 건다. 이 실패를 `-DPython3_EXECUTABLE` 탓으로 오진하기 쉬운데 (§9.2 carve-out 은 정상이다), 감별은 `echo $VIRTUAL_ENV` 또는 `/usr/bin/python3 -c "import ament_package"` 한 줄이면 된다.

post-incident 검증: `ls src/rtc-framework/{build,install,log}` — 존재하면 잘못된 cwd 에서 실행된 것이므로 삭제.

### 9.2 `.venv` 격리 (Hard rule)

> **`.venv` 는 runtime PC 가 본 workspace 외에 다른 control project 들과 공존하는 환경에서 dependency 를 격리하기 위한 의도된 설계다.** venv 활성 상태에서 `colcon test` / `colcon build` / `ros2 run` / `ros2 launch` 가 실패하면 **반드시 근본 원인을 해결** (sys.path / shebang / wrapper / dep resolution 디버그). gtest binary 직접 실행, venv deactivate 후 colcon 호출, `PYTHONPATH` 강제 우회 등 **격리 무력화 우회 금지** — runtime PC 에서 silent breakage 경로.

위 규칙은 self-contained 하다 — 실패 시 격리를 무력화(gtest 직접 실행 / venv deactivate / `PYTHONPATH` 우회)하지 말고 sys.path / shebang / wrapper / dep resolution 을 디버그한다. 근거·과거 위반 사례는 git log + auto-memory 참조 (머신 종속 절대경로는 박제하지 않는다).

**단 하나의 carve-out — `colcon build` 의 configure 단계**: venv 가 활성이면 CMake `FindPython` 이 venv 의 python 을 잡아 eigenpy/pinocchio configure 가 깨진다. 이건 *검증 우회* 가 아니라 빌드 시스템의 Python 탐색 문제이므로, 이 경우에 한해 `deactivate` 가 허용된다. 다만 **`--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3` 를 우선**하고 (격리를 유지한 채 해결), `build.sh` 를 쓰면 둘 다 불필요하다. `colcon test` / `ros2 run` / `ros2 launch` 실패를 덮기 위한 deactivate 는 **여전히 금지** — 그건 runtime PC 에서 재현될 결함을 숨기는 것이다. README 빠른 시작의 `deactivate 2>/dev/null` 스니펫이 가리키는 것이 바로 이 carve-out 이다.

## 10. Style Cheatsheet

상세: [agent_docs/conventions.md](agent_docs/conventions.md). 본 헌법은 변경 빈도가 낮은 절대 규칙만:

- **Namespace**: `rtc`
- **Naming**: Google C++ — `PascalCase` methods/types/free functions, `snake_case_` private members, `kConstant` constants. 상세 [agent_docs/conventions.md](agent_docs/conventions.md#code-conventions)
- **Units**: SI (m, rad, s, kg, N) — degree 는 API 경계에서만
- **Rotation**: quaternion (`Eigen::Quaterniond`, Hamilton) internal, ZYX Euler at boundaries
- **Variable naming**: paper notation — `J_b` (body Jacobian), `q_d` (desired joint), `K_d` (stiffness)
- **RAII**, `noexcept` on RT, `[[nodiscard]]` on status returns
- **Lifecycle**: 핵심 C++ 노드는 `rclcpp_lifecycle::LifecycleNode` — empty constructor, `on_configure` (Tier 1) + `on_activate` (Tier 2). 어떤 노드가 LifecycleNode 인지는 [agent_docs/architecture.md](agent_docs/architecture.md) 참조 (박제 금지)
- **Logger naming** (3-tier 원칙): node-owned = `<exec_name>` / library-level = `<full_package_name>` / controller-level = `<package>.<controller_key>`. 점 `.` 1개만 허용. 구체 예시: [agent_docs/conventions.md](agent_docs/conventions.md)
- **Commits**: Conventional Commits `type(scope): subject`

## 11. Post-Task Housekeeping

Commit 완료 또는 사용자가 task 종료를 알린 후:

1. **Memory save / Memory prune / Harness pruning 신호 보고** — user-level CLAUDE.md `# Post-task housekeeping` 가 SSoT. *Harness pruning 신호* 의 RTC 발현 카테고리는 invariant·anti-pattern grep false-positive, [.claude/hooks/verify-changes.sh](.claude/hooks/verify-changes.sh) 오차단, agent_docs 간 규칙 중복 drift
2. **Issue 동기화** — 대응 GitHub issue 가 있으면 구현 완료 시 갱신한다: 무엇이 구현됐는지, acceptance criteria 중 미충족 항목, 후속 작업. issue 는 durable 결정 기록이자 cross-tool 인계면이므로 (§4, [agent_docs/handoff.md](agent_docs/handoff.md) §5) 갱신 없이 닫지 않는다. criteria 를 전부 충족했으면 close, 아니면 남은 범위를 코멘트로 남기고 open 유지
3. **Stale artifact 정리** — 완료된 private plan (`~/.claude/plans/*.md`) 은 그 내용이 git log / issue / memory 로 복원 가능하거나 보존할 가치가 없으면 삭제 (복원 불가한데 보존 가치가 있는 결정 기록이 남아 있으면 issue 코멘트로 옮긴 뒤 삭제 — [agent_docs/handoff.md](agent_docs/handoff.md) §5). 작업 중 만든 임시 파일 (분석 스크립트, 중간 산출물, 로그 덤프) 은 scratchpad 에 만들고 task 종료 시 삭제하며, repo-root / `/tmp` scratch files 도 다른 곳 (git log, `agent_docs/*.md`, `docs/*.md`, issue) 에 보존됨을 확인 후 삭제
4. **캐시 정리** — repo (`src/rtc-framework`) 안에 잘못된 cwd 로 생긴 `build/` · `install/` · `log/` (§9.1) 및 python 캐시 (`__pycache__`, `.pytest_cache`, `.ruff_cache`, `.mypy_cache`) 가 있으면 삭제 — 모두 재생성 가능하므로 확인 없이 제거 가능. **단 colcon 정규 트리 `<rtc_ws>/{build,install,log}` 는 incremental cache 이므로 절대 건드리지 않는다** (§9.1)
5. **Branch prune (main merge 후에만)** — feature branch 가 `main` 에 merge 됐으면: 로컬 merged branch 삭제 (`git branch -d <branch>`), stale remote-tracking ref 정리 (`git fetch --prune`). 원격 branch 삭제는 merge 확인 후에만 (GitHub auto-delete 미설정 시). 현재 checkout 된 branch·미merge branch·`main` 은 건드리지 않는다
6. **보고** — 실제 수행한 항목만 한 줄씩

## 12. Reference Docs (read when relevant)

- [agent_docs/architecture.md](agent_docs/architecture.md) — Threading, data flow, core types, lock-free rules, lifecycle, E-STOP, 패키지 dependency graph
- [agent_docs/controllers.md](agent_docs/controllers.md) — Controller table, gains layout, GraspController FSM, topics, config files
- [agent_docs/modification-guide.md](agent_docs/modification-guide.md) — Workflow loop, adding controllers/messages/devices/threads, package update checklist
- [agent_docs/design-principles.md](agent_docs/design-principles.md) — `rtc_*` 5 principles, boundary rules
- [agent_docs/conventions.md](agent_docs/conventions.md) — Domain / code / commit conventions, documentation requirements
- [agent_docs/testing-debug.md](agent_docs/testing-debug.md) — Sensor matrix, test commands, live debug topics, RT permissions
- [agent_docs/invariants.md](agent_docs/invariants.md) — RT / ARCH / PROC / NUM invariants (escalation triggers detail)
- [agent_docs/anti-patterns.md](agent_docs/anti-patterns.md) — Recurring mistakes with detection + recovery
- [agent_docs/handoff.md](agent_docs/handoff.md) — Tool-neutral context handoff 계약 (trigger 분류·artifact template·sender/receiver checklist·storage)
- [AGENTS.md](AGENTS.md) — 같은 헌법의 tool-neutral 판 (Claude 외 도구용). 규칙 변경 시 양쪽 동기화 필요
- [README.md](README.md) — 빌드·설치 명령, deps 버전, 빠른 시작
- [repo_scripts/README.md](repo_scripts/README.md) — PREEMPT_RT, CPU shield, env activation, isolated deps
