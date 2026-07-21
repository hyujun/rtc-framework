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
| **Enforcement** (자동) | 무인 실행/차단 | [.claude/hooks/format-code.sh](.claude/hooks/format-code.sh) (PostToolUse: clang-format / ruff), [.claude/hooks/verify-changes.sh](.claude/hooks/verify-changes.sh) (Stop: doc corpus·CMake·build·test gate, exit 2 차단) |

**첫 방문 에이전트**: §3 → §4 → §6 순으로 읽고 작업 시작.
**수정 작업 중**: §5 검증 + §6 escalation 확인. Invariant 위반 의심 시 즉시 §6.

## 3. Invariants (요약)

전체: [agent_docs/invariants.md](agent_docs/invariants.md).

### RT path 절대 금지 (정기 tick — `control_rate` YAML)

RT 핫패스 절대금지 규칙 — **no** alloc(`new`/`malloc`/`push_back`/`resize`) · `throw`/`catch` · 직접 `RCLCPP_*` 로깅 · `mutex`/`lock_guard` · `auto`+Eigen · quaternion `lerp`/`nlerp` · `shared_ptr` 복사 — 은 **path-scoped rule** [.claude/rules/rt-path.md](.claude/rules/rt-path.md) 에 상세(예외·대안 포함) — 어떤 파일 편집 시 로드되는지는 그 rule 의 frontmatter glob 이 SSoT (AP-DOC-1: 여기 박제 금지). 이 규칙은 RT tick / SCHED_FIFO 경로에만 구속되고 lifecycle·aux·test·init 코드는 면제 (판정 절차는 rule 파일·invariants.md). RT 코드 수정 전 반드시 확인하고, 위반 필요시 §6 `[CONCERN]`.

### Architecture / Process / Numerical

- `rtc_*` 패키지에 robot name / joint count / HW ID 하드코딩 금지 (ARCH-1)
- 의존성 그래프 상향 의존 금지 (ARCH-2)
- 두 번째 구체 구현은 abstract interface / concept 정의 후에만 추가 (ARCH-3) — `#ifdef` / hardcoded switch 금지
- `robot_descriptions` 는 data-only 패키지 — 소비자는 `<exec_depend>` + ament_index 런타임 lookup만 (ARCH-5)
- 새 utility 작성 전 기존 `rtc_*` 패키지에 유사 기능 검색 — 맞지 않으면 fork 대신 일반화 ([agent_docs/design-principles.md](agent_docs/design-principles.md) P5)
- 코드 변경 → 대응 문서·YAML·CMakeLists·package.xml 동기화 필수 (PROC-1)
- 기존 test assertion 을 통과시키려 **약화 금지** — 새 코드를 고치되, test 가 진짜 틀렸거나 spec 이 바뀌면 별도 commit + 근거 (PROC-6, §6 E-6)
- `rtc_base` / `rtc_msgs` 변경 시 전체 빌드·테스트 (PROC-3)
- 수치 특이점: damped pseudoinverse (NUM-1), zero guard (NUM-2, NUM-4)

세부 규칙·grep 패턴·복구 절차: [agent_docs/invariants.md](agent_docs/invariants.md). 위반 필요시 §6 Escalation 의 `[CONCERN]` 포맷 보고.

## 4. Workflow Loop

7단계: **Type → Locate → Read → Edit → Build → Test → Verify**. 규모에 맞춰 압축한다 — 오타·포매팅·자명한 단일 라인 수정은 단계를 합쳐도 되나, **검증(Build/Test/Verify)을 생략했다면 최종 보고에 무엇을·왜 생략했는지 명시**한다. 다파일·다패키지·`rtc_base`/`rtc_msgs` 변경에서 검증 단계를 건너뛰는 것은 §6 escalation 사유. 실패 시 절대 **"try harder" 금지** — 누락된 capability (test, lint, interface) 를 엔지니어링하거나 §6 escalate.

**Type 분기**: "수정" 인가 "추가 (새 기능 / 컨트롤러 / 메시지 / 디바이스 / 스레드)" 인가? 추가 task 는 단계 1 진입 전에 [agent_docs/design-principles.md](agent_docs/design-principles.md) 5원칙 + [agent_docs/modification-guide.md](agent_docs/modification-guide.md) "Adding a New ..." 절을 먼저 읽는다 (rtc_* 추가는 P1·P2 + ARCH-3 결합; integration package 또는 `shape_estimation*` 추가 시 rtc_* 일반화 가능성부터 검토).

**계획 전 분석**: 대응하는 GitHub issue 가 있으면 계획을 세우기 전에 그 issue (본문 + 코멘트) 를 먼저 참고한다 — issue 는 durable 결정 기록이자 cross-tool 인계면이므로 (§6.6, [agent_docs/handoff.md](agent_docs/handoff.md) §5) 이전 세션·다른 tool 의 acceptance criteria·결정·미완료 상태가 거기 남아 있다. 단 issue 본문의 진단·근거는 **미검증 가설**로 취급하고 착수 전 grep/코드로 반증한다 (틀렸으면 issue 를 먼저 갱신).

**4·5·6 자동화**: [.claude/hooks/verify-changes.sh](.claude/hooks/verify-changes.sh) Stop hook 이 turn 종료 시 자동 실행하고 hard failure 시 `exit 2` 로 다음 turn 까지 차단한다 (loop 방지는 `stop_hook_active` 가드; stop cycle 당 1회 발화). 변경 패키지만 빌드·테스트하며 **build/test 의 timeout·launch 실패는 "미검증"으로 차단** (silent pass 아님 — bound 초과 test 는 hook 의 timeout 상향). README co-update 는 public surface (header/launch/config/파일 add·del/dep) 변경 시 **non-blocking checklist** (내부 리팩터·bug fix 는 미요구); CMake/`package.xml` co-update 는 blocking. **변경 파일 집합은 tracked(staged+unstaged) ∪ untracked** — `git add` 하지 않은 신규 소스도 게이트에 걸린다. **문서(`*.md`) / workflow 변경은 `repo_scripts/scripts/validate_docs.py` 로 blocking 검증** (깨진 링크·라인 앵커·machine-local 경로·깨진 탐지 패턴). YAML / Doxygen 은 에이전트가 직접 검증. Pure-format commit (clang-format / ruff round-trip 동치) 은 ARCH grep + doc 단계만 skip, build/test 는 그대로.

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

수동 trigger 인 이유: inferential 은 GPU/cost/지연이 크고 non-deterministic 이므로 모든 변경에 자동 적용하면 ROI 음성. 위 trigger 는 "false-negative 비용 > inferential 비용" 인 경우만 추렸다.

## 6. Escalation Triggers

다음 상황에서 코드를 쓰기 **전에** `[CONCERN]` 보고 후 사용자 컨펌 대기. 세부 invariant·grep·복구는 [agent_docs/invariants.md](agent_docs/invariants.md) 참조.

- **E-1** (Critical) — [agent_docs/invariants.md](agent_docs/invariants.md) 규칙을 건드려야 할 것 같음 (§3 전반)
- **E-2** (Critical) — `rtc_*` 패키지에 robot-specific 값을 넣어야 함 (ARCH-1)
- **E-3** (Critical) — `rtc_msgs` / `shape_estimation_msgs` public ABI 변경 필요
- **E-4** (Warning) — Abstract interface 없이 두 번째 구현 추가 필요 (ARCH-3)
- **E-5** (Warning) — Optional dep (MuJoCo, aligator) fallback 제거 필요
- **E-6** (Critical) — 기존 test assertion 을 약화·수정해야 할 것 같음 — 회귀 은폐 vs 정당한 spec 변경/test-bug 구분, 후자는 별도 commit + 근거 (PROC-6)
- **E-7** (Critical) — Thread model (core 배치, priority) 변경
- **E-8** (Critical) — E-STOP 경로 수정
- **E-9** (Warning) — 문서-코드 불일치를 어느 쪽에 맞출지 결정 필요
- **E-10** (Warning) — `robot_descriptions` 를 build-time 으로 의존하려는 변경 (`find_package` / `<depend>` / `ament_target_dependencies`) (ARCH-5)
- **E-11** (Warning) — `PublishRole` enum 에 controller-owned non-RT 토픽을 추가하려는 변경 — 새 controller-owned 토픽은 `SeqLock<T>` + `Setup*Publisher` helper 패턴 (Phase 4 trailing cleanup)

### `[CONCERN]` 포맷

```
[CONCERN] <한 줄 요약>
Severity: Critical | Warning | Info
Detail: <문제의 구체 내용, 영향 범위, 검토한 대안>
Alternative: <우회 안 1개 이상>
```

- **Critical**: 사용자 컨펌 전까지 커밋·PR 금지
- **Warning**: 사용자 판단에 따라 진행, 결정 로그 남김
- **Info**: 기록만, 진행 가능

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

최근 발현 빈도 Top: **AP-RT-1** (정기 tick `RCLCPP_*`) · **AP-RT-3** (`auto` + Eigen) · **AP-ARCH-1** (`rtc_*` 에 robot 상수) · **AP-PROC-1** ("✅ complete" 후 미완료) · **AP-PROC-4** (test assertion 수정) · **AP-DOC-1** (CLAUDE.md 에 패키지 수·테스트 수 박제). 전체 사례·복구·grep 은 [agent_docs/anti-patterns.md](agent_docs/anti-patterns.md) — §3 invariants 와 1:1 대응이므로 §3 에서 룰을 보고 anti-patterns 에서 *그 룰을 위반한 실제 commit* 을 찾는다.

## 8. Where Things Live

패키지 역할·dependency graph·data flow·threading model: [agent_docs/architecture.md](agent_docs/architecture.md) 가 단일 출처. 본 헌법은 위치 박제를 두지 않는다 (패키지 추가·rename 시 drift 방지).

## 9. Build & Run Hard Rules

명령 detail (build.sh 옵션, source 순서, deps 버전, plain colcon build 호환): [README.md](README.md#빠른-시작) · [repo_scripts/README.md](repo_scripts/README.md). 본 헌법은 두 가지 절대 규칙만 박는다.

### 9.1 colcon CWD (Hard rule)

> **`colcon build` / `colcon test` 는 반드시 colcon workspace root (`<rtc_ws>` = `~/ros2_ws/rtc_ws`) 에서 실행한다.** repo (`src/rtc-framework`) 안에서 호출하면 `build/` · `install/` · `log/` 트리가 그 위치에 생기고 — `.clangd` 의 CompilationDatabase 가 잘못된 트리를 가리키며 ws-root incremental cache 와 분리되어 추적 불가한 stale state 가 누적된다. `build.sh` / `install.sh` 는 내부에서 `cd "$WORKSPACE"` 하므로 안전. 직접 `colcon` 을 칠 때는 **항상 `cd <rtc_ws>` 또는 절대경로 `--build-base` / `--install-base` 지정**, 그리고 `source ${repo_ws}/repo_scripts/scripts/setup_env.sh` (`${repo_ws}` = `<rtc_ws>/src/rtc-framework` = `~/ros2_ws/rtc_ws/src/rtc-framework`; `repo_scripts` 는 repo 안에 있으므로 ws-root cwd 기준 상대경로 `repo_scripts/...` 는 안 풀린다 — 절대경로 또는 이 prefix 필수). env 미source 상태로 `colcon`/`cmake` 호출 시 컴파일러·ROS·deps·venv PATH 누락으로 `colcon test` 가 silent fail 하거나 build 가 즉시 비정상 종료한다.

post-incident 검증: `ls src/rtc-framework/{build,install,log}` — 존재하면 잘못된 cwd 에서 실행된 것이므로 삭제.

### 9.2 `.venv` 격리 (Hard rule)

> **`.venv` 는 runtime PC 가 본 workspace 외에 다른 control project 들과 공존하는 환경에서 dependency 를 격리하기 위한 의도된 설계다.** venv 활성 상태에서 `colcon test` / `colcon build` / `ros2 run` / `ros2 launch` 가 실패하면 **반드시 근본 원인을 해결** (sys.path / shebang / wrapper / dep resolution 디버그). gtest binary 직접 실행, venv deactivate 후 colcon 호출, `PYTHONPATH` 강제 우회 등 **격리 무력화 우회 금지** — runtime PC 에서 silent breakage 경로.

위 규칙은 self-contained 하다 — 실패 시 격리를 무력화(gtest 직접 실행 / venv deactivate / `PYTHONPATH` 우회)하지 말고 sys.path / shebang / wrapper / dep resolution 을 디버그한다. 근거·과거 위반 사례는 git log + auto-memory 참조 (머신 종속 절대경로는 박제하지 않는다).

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
2. **Stale artifact 정리** — 완료된 private plan (`~/.claude/plans/*.md`) 은 그 내용이 git log / issue / memory 로 복원 가능하거나 보존할 가치가 없으면 삭제 (복원 불가한데 보존 가치가 있는 결정 기록이 남아 있으면 issue 코멘트로 옮긴 뒤 삭제 — [agent_docs/handoff.md](agent_docs/handoff.md) §5). repo-root / `/tmp` scratch files 도 다른 곳 (git log, `agent_docs/*.md`, `docs/*.md`) 에 보존됨을 확인 후 삭제
3. **캐시 정리** — repo (`src/rtc-framework`) 안에 잘못된 cwd 로 생긴 `build/` · `install/` · `log/` (§9.1) 및 python 캐시 (`__pycache__`, `.pytest_cache`, `.ruff_cache`, `.mypy_cache`) 가 있으면 삭제 — 모두 재생성 가능하므로 확인 없이 제거 가능. **단 colcon 정규 트리 `<rtc_ws>/{build,install,log}` 는 incremental cache 이므로 절대 건드리지 않는다** (§9.1)
4. **Branch prune (main merge 후에만)** — feature branch 가 `main` 에 merge 됐으면: 로컬 merged branch 삭제 (`git branch -d <branch>`), stale remote-tracking ref 정리 (`git fetch --prune`). 원격 branch 삭제는 merge 확인 후에만 (GitHub auto-delete 미설정 시). 현재 checkout 된 branch·미merge branch·`main` 은 건드리지 않는다
5. **보고** — 실제 수행한 항목만 한 줄씩

## 12. Reference Docs (read when relevant)

- [agent_docs/architecture.md](agent_docs/architecture.md) — Threading, data flow, core types, lock-free rules, lifecycle, E-STOP, 패키지 dependency graph
- [agent_docs/controllers.md](agent_docs/controllers.md) — Controller table, gains layout, GraspController FSM, topics, config files
- [agent_docs/modification-guide.md](agent_docs/modification-guide.md) — Workflow loop, adding controllers/messages/devices/threads, package update checklist
- [agent_docs/design-principles.md](agent_docs/design-principles.md) — `rtc_*` 5 principles, boundary rules
- [agent_docs/conventions.md](agent_docs/conventions.md) — Domain / code / commit conventions, documentation requirements
- [agent_docs/testing-debug.md](agent_docs/testing-debug.md) — Sensor matrix, test commands, live debug topics, RT permissions
- [agent_docs/invariants.md](agent_docs/invariants.md) — RT / ARCH / PROC / NUM invariants (escalation triggers detail)
- [agent_docs/anti-patterns.md](agent_docs/anti-patterns.md) — Recurring mistakes with detection + recovery
- [README.md](README.md) — 빌드·설치 명령, deps 버전, 빠른 시작
- [repo_scripts/README.md](repo_scripts/README.md) — PREEMPT_RT, CPU shield, env activation, isolated deps
