# CLAUDE.md

이 파일은 본 저장소의 **헌법 (constitution)** 이다. 안정적인 원칙·게이트·지표만 둔다. 자주 변하는 사실 (패키지 수, robot 목록, 의존성 버전, 명령 detail) 은 sub-doc / README 의 SSoT 를 참조한다 — CLAUDE.md 에 박제하지 않는다 ([agent_docs/anti-patterns.md](agent_docs/anti-patterns.md) AP-DOC-1).

## 1. Snapshot

**RTC (Real-Time Control) Framework** — URDF 기반 매니퓰레이터를 위한 robot-agnostic real-time control framework. 변수 DOF, 설정 가능한 RT 루프 주기 (`control_rate` YAML, 100 Hz ~ 5 kHz, default 500 Hz), transport 추상화 (UDP/CAN-FD/EtherCAT/RS485), lock-free SPSC, E-STOP.

- 패키지 구성·count·역할: [README.md](README.md#패키지-구성) · [agent_docs/architecture.md](agent_docs/architecture.md)
- 로봇 데이터 (URDF/MJCF/mesh, multi-robot data hub): [robot_descriptions/README.md](robot_descriptions/README.md)
- 언어·OS·의존성 버전: [README.md](README.md#빠른-시작) · [repo_scripts/README.md](repo_scripts/README.md)
- 최신 test 카운트·실측: 단일 출처 [agent_docs/testing-debug.md](agent_docs/testing-debug.md)

## 2. Harness Overview

이 저장소의 에이전트 가이드는 **agent-driven engineering** (harness engineering + spec-driven development + Anthropic 2026 agentic SDLC patterns) 의 5구성요소로 조직되어 있다. Agent = Model + Harness이며, 모델 바깥의 guides / sensors / orchestration / escalation / enforcement가 본 저장소의 1급 자산이다.

출처: [Fowler — *Harness engineering for coding agent users* 2026.04](https://martinfowler.com/articles/harness-engineering.html) · [Anthropic — *Harness design for long-running application development* 2026.04](https://www.anthropic.com/engineering/harness-design-long-running-apps) · [Osmani — *Agent Harness Engineering* 2026](https://addyosmani.com/blog/agent-harness-engineering/) · [Anthropic — *2026 Agentic Coding Trends Report*](https://resources.anthropic.com/2026-agentic-coding-trends-report).

| 구성요소 | 목적 | 진입점 |
|---|---|---|
| **Guides** (feedforward) | 규칙·원칙·패턴 | §3 Invariants, §10 Style, [agent_docs/invariants.md](agent_docs/invariants.md), [agent_docs/design-principles.md](agent_docs/design-principles.md), [agent_docs/conventions.md](agent_docs/conventions.md) |
| **Sensors** (feedback, computational) | 변경 검증 (결정적·빠른) | §5, [agent_docs/testing-debug.md](agent_docs/testing-debug.md), `[CONCERN]` 포맷 (§6) |
| **Sensors** (feedback, inferential) | 의미 검증 (LLM-as-judge, on-demand) | §5.5 |
| **Orchestration** | Workflow | §4, [agent_docs/modification-guide.md](agent_docs/modification-guide.md) |
| **Escalation** | Human gate | §6, §6.5 Sprint Contract |
| **Enforcement** (자동) | 무인 실행/차단 | [.claude/hooks/format-code.sh](.claude/hooks/format-code.sh) (PostToolUse: clang-format / ruff), [.claude/hooks/verify-changes.sh](.claude/hooks/verify-changes.sh) (Stop: doc·CMake·build·test gate, exit 2 차단), [.claude/rules/rt-safety.md](.claude/rules/rt-safety.md) (RT path scoped inject), [.claude/rules/colcon-cwd.md](.claude/rules/colcon-cwd.md) (colcon/CMake/package.xml scoped inject) |

**첫 방문 에이전트**: §3 → §4 → §6 순으로 읽고 작업 시작.
**수정 작업 중**: §5 검증 + §6 escalation 확인. Invariant 위반 의심 시 즉시 §6.

## 3. Invariants (요약)

전체: [agent_docs/invariants.md](agent_docs/invariants.md). 아래는 RT path 에서 자주 위반되는 핵심만.

### RT path 절대 금지 (정기 tick — `control_rate` YAML, 100 Hz–5 kHz, default 500 Hz)

1. `new` / `malloc` / `push_back` / `emplace_back` / `resize` — pre-allocated fixed-size 사용
2. `throw` / `catch` — error code, `std::optional`, `std::expected`
3. `RCLCPP_INFO/WARN/ERROR/DEBUG/FATAL` 직접 호출 — SPSC → aux thread defer. **예외**: one-shot init, `RCLCPP_*_THROTTLE` with RT-safe msg (단순 format + 기본 타입만; `fmt::format` / `to_string` / string concat 금지)
4. `std::mutex::lock` / `lock_guard` / `scoped_lock` — `try_lock`, `SeqLock`, SPSC, atomic
5. `auto` with Eigen expression — aliasing 버그; 명시 타입
6. Quaternion `lerp` / `nlerp` — `slerp` only
7. 기존 test assertion 수정 — 새 코드를 고쳐라
8. `std::shared_ptr` 복사 — atomic ref-count contention; raw ref 또는 `const std::shared_ptr<T>&`

### Architecture / Process / Numerical

- `rtc_*` 패키지에 robot name / joint count / HW ID 하드코딩 금지 (ARCH-1)
- 의존성 그래프 상향 의존 금지 (ARCH-2)
- 두 번째 구체 구현은 abstract interface / concept 정의 후에만 추가 (ARCH-3) — `#ifdef` / hardcoded switch 금지
- `robot_descriptions` 는 data-only 패키지 — 소비자는 `<exec_depend>` + ament_index 런타임 lookup만 (ARCH-5)
- 새 utility 작성 전 기존 `rtc_*` 패키지에 유사 기능 검색 — 맞지 않으면 fork 대신 일반화 ([agent_docs/design-principles.md](agent_docs/design-principles.md) P5)
- 코드 변경 → 대응 문서·YAML·CMakeLists·package.xml 동기화 필수 (PROC-1)
- `rtc_base` / `rtc_msgs` 변경 시 전체 빌드·테스트 (PROC-3)
- 수치 특이점: damped pseudoinverse (NUM-1), zero guard (NUM-2, NUM-4)

세부 규칙·grep 패턴·복구 절차: [agent_docs/invariants.md](agent_docs/invariants.md). 위반 필요시 §6 Escalation 의 `[CONCERN]` 포맷 보고.

## 4. Workflow Loop

7단계: **Type → Locate → Read → Edit → Build → Test → Verify**. 단계 건너뛰기는 §6 escalation 사유. 실패 시 절대 **"try harder" 금지** — 누락된 capability (test, lint, interface) 를 엔지니어링하거나 §6 escalate.

**Type 분기**: "수정" 인가 "추가 (새 기능 / 컨트롤러 / 메시지 / 디바이스 / 스레드)" 인가? 추가 task 는 단계 1 진입 전에 [agent_docs/design-principles.md](agent_docs/design-principles.md) 5원칙 + [agent_docs/modification-guide.md](agent_docs/modification-guide.md) "Adding a New ..." 절을 먼저 읽는다 (rtc_* 추가는 P1·P2 + ARCH-3 결합; integration package 또는 `shape_estimation*` 추가 시 rtc_* 일반화 가능성부터 검토).

**4·5·6 자동화**: [.claude/hooks/verify-changes.sh](.claude/hooks/verify-changes.sh) Stop hook 이 turn 종료 시 자동 실행하고 실패 시 `exit 2` 로 다음 turn 까지 차단한다. 변경 패키지만 빌드·테스트 (60s timeout per pkg) + README/CMake co-update 검사 — `package.xml` / YAML / Doxygen 은 에이전트가 직접 검증. Pure-format commit (clang-format / ruff round-trip 동치) 은 ARCH grep + README/CMake 단계만 skip, build/test 는 그대로.

단계별 액션·grep 패턴·Completion Checklist: [agent_docs/modification-guide.md](agent_docs/modification-guide.md).

## 5. Sensors

**변경 위치별 sensor matrix·명령·Live Debug Topics**: [agent_docs/testing-debug.md](agent_docs/testing-debug.md) 가 단일 출처. 매트릭스를 CLAUDE.md 에 박제하면 두 곳 동기화 부담으로 drift 가 발생하므로, 본 헌법은 위임만 한다.

원칙:
- 변경 패키지의 sensor 행을 testing-debug.md 에서 찾아 **필수 sensor + 추가 sensor** 모두 실행
- 실패하면 다음 turn 의 verify-changes.sh hook 이 차단
- `rtc_base` / `rtc_msgs` 변경 시 전체 downstream 검증 (PROC-3)

## 5.5 Inferential Sensors (LLM-as-judge, 수동 trigger)

§5 의 computational sensor (build / test / grep) 는 **문법·빌드·기존 테스트 통과** 만 검증한다. 의미 회귀 — 설계 일관성, robot-agnostic 위반, abstract interface 누락, 재사용 가능성 — 은 잡지 못한다 (Anthropic 2026.04 *Harness design*: 에이전트의 자기 평가는 신뢰 불가).

다음 상황에서 사용자에게 inferential sensor 실행을 권한다:

- `rtc_base` / `rtc_msgs` 변경 → `/review` (downstream 전 패키지 영향)
- Abstract interface 신설 / 두 번째 구현 추가 (ARCH-3 후보) → `/review` (base 누락·#ifdef 유혹 검출)
- `rtc_*` 에 robot-specific 코드 추가 의심 (ARCH-1 borderline) → `/review`
- E-STOP 경로 / safety publisher / lifecycle 콜백 수정 → `/security-review` (E-8)
- PR 준비 (다파일 / 다패키지 commit) → `/ultrareview <PR#>`
- 100+ 줄 변경 또는 신규 패키지 디렉토리 → `/review`

수동 trigger 인 이유: inferential 은 GPU/cost/지연이 크고 non-deterministic 이므로 모든 변경에 자동 적용하면 ROI 음성. 위 trigger 는 "false-negative 비용 > inferential 비용" 인 경우만 추렸다.

## 6. Escalation Triggers

다음 상황에서 코드를 쓰기 **전에** `[CONCERN]` 보고 후 사용자 컨펌 대기. 세부 invariant·grep·복구는 [agent_docs/invariants.md](agent_docs/invariants.md) 참조.

- **E-1** (Critical) — [agent_docs/invariants.md](agent_docs/invariants.md) 규칙을 건드려야 할 것 같음 (§3 전반)
- **E-2** (Critical) — `rtc_*` 패키지에 robot-specific 값을 넣어야 함 (ARCH-1)
- **E-3** (Critical) — `rtc_msgs` / `shape_estimation_msgs` public ABI 변경 필요
- **E-4** (Warning) — Abstract interface 없이 두 번째 구현 추가 필요 (ARCH-3)
- **E-5** (Warning) — Optional dep (MuJoCo, aligator) fallback 제거 필요
- **E-6** (Critical) — 기존 test assertion 수정 필요 (RT-7)
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

근거: Anthropic 2026.04 *Harness design* — "에이전트 자기 평가는 신뢰 불가, 평가 기준은 generation 시작 *전* 명시화하라."

다음 task 에서는 코드 수정 시작 *전* 1~3줄로 성공 기준을 사용자에게 제시하고 컨펌 받는다:

- 다단계 task (PR 단위 / 다파일 / 다패키지 / 신규 디렉토리 / phase 로 쪼개진 작업)
- 신규 abstract interface · 새 controller / device group / thread / message 추가
- `rtc_base` / `rtc_msgs` 변경 (downstream broad impact)
- 리팩터 (기능 동등성 유지가 곧 success)

면제: 단일 파일 bug fix, 오타·포매팅, 단일 함수 추가, 사용자 의도가 1줄 메시지에서 자명한 경우.

※ **신규 abstract interface · 신규 controller · 신규 메시지·디바이스 추가 시 Sprint Contract = spec.** 구현 전 `~/.claude/plans/<slug>.md` 에 *왜 필요한가 · API surface · 검토한 alternatives* 를 1-paragraph spec 으로 박는다 (spec-driven development: Specify *before* Implement). 같은 파일이 이후 §6.6 handoff artifact·진행 progress 도 누적하므로 `## Spec` / `## Progress` / `## Handoff` 섹션으로 구분.

### Sprint Contract 포맷

```
[SPRINT] <task 한 줄 요약>
Done when:
  - <검증 가능 기준 1>
  - <검증 가능 기준 2>
  - <...>
Out of scope: <명시적으로 하지 않을 것 — drift 방지>
```

기준은 **객관 검증 가능** 해야 한다 (예: "test_X 통과", "rtc_* 에 ur5e grep 0건", "rtc_cm 빌드 0 warning"). "코드가 깔끔하다", "잘 작동한다" 같은 주관 기준은 금지. 이 컨트랙트는 task 종료 시 §11 보고에서 항목별 충족 여부를 체크한다.

## 6.6 Long-running task — Context reset > Compaction

근거: Anthropic 2026.04 *Harness design* — long-running task 에서 context compaction 은 "context anxiety" (조기 마무리 / 디테일 손실) 를 유발. **clean-slate reset + structured handoff artifact** 가 더 안정적.

다음 신호가 보이면 사용자에게 reset 을 제안한다:

- 동일 task 안에서 turn 50 이상, 또는 자동 compaction 이 1회 이상 발생
- 에이전트가 직전 결정·근거를 다시 묻거나, "방금 한 작업이…" 로 시작하는 hallucinated recall 발견
- §6.5 Sprint Contract 기준 중 일부가 "이미 했다" 라고 잘못 보고됨

### Reset 절차

1. **Handoff artifact 작성**: `~/.claude/plans/<task-slug>.md` 에 다음 박아 두기
   - Sprint Contract (§6.5 그대로 — Done when / Out of scope)
   - 지금까지 commit 된 SHA + 한 줄 요약
   - 미완료 항목 (체크박스)
   - 다음 fresh session 이 첫 turn 에 읽어야 할 파일 경로 5~10개
   - 알려진 함정 / `[CONCERN]` 미해결 사항
2. **사용자에게 통지**: "context reset 권장. handoff 는 `<path>` 에 저장. 새 세션에서 `Read <path>` 로 시작" 한 줄
3. **현 세션 종료**: 사용자 컨펌 후 `/clear` 또는 새 세션 시작은 사용자 결정

### 면제

- 단일 commit 으로 끝나는 task — compaction 도달 전에 종료
- Sensor-driven debug task (build/test/log 반복) — handoff 보다 직접 진행이 빠름
- 사용자가 명시적으로 "이대로 계속" 을 요청한 경우

## 7. Anti-patterns

최근 발현 빈도 Top: **AP-RT-1** (정기 tick `RCLCPP_*`) · **AP-RT-3** (`auto` + Eigen) · **AP-ARCH-1** (`rtc_*` 에 robot 상수) · **AP-PROC-1** ("✅ complete" 후 미완료) · **AP-PROC-4** (test assertion 수정) · **AP-DOC-1** (CLAUDE.md 에 패키지 수·테스트 수 박제). 전체 사례·복구·grep 은 [agent_docs/anti-patterns.md](agent_docs/anti-patterns.md) — §3 invariants 와 1:1 대응이므로 §3 에서 룰을 보고 anti-patterns 에서 *그 룰을 위반한 실제 commit* 을 찾는다.

## 8. Where Things Live

패키지 역할·dependency graph·data flow·threading model: [agent_docs/architecture.md](agent_docs/architecture.md) 가 단일 출처. 본 헌법은 위치 박제를 두지 않는다 (패키지 추가·rename 시 drift 방지).

## 9. Build & Run Hard Rules

명령 detail (build.sh 옵션, source 순서, deps 버전, plain colcon build 호환): [README.md](README.md#빠른-시작) · [repo_scripts/README.md](repo_scripts/README.md). 본 헌법은 두 가지 절대 규칙만 박는다.

### 9.1 colcon CWD (Hard rule)

> **`colcon build` / `colcon test` 는 반드시 colcon workspace root (`<rtc_ws>` = `~/ros2_ws/rtc_ws`) 에서 실행한다.** repo (`src/rtc-framework`) 안에서 호출하면 `build/` · `install/` · `log/` 트리가 그 위치에 생기고 — `.clangd` 의 CompilationDatabase 가 잘못된 트리를 가리키며 ws-root incremental cache 와 분리되어 추적 불가한 stale state 가 누적된다. `build.sh` / `install.sh` 는 내부에서 `cd "$WORKSPACE"` 하므로 안전. 직접 `colcon` 을 칠 때는 **항상 `cd <rtc_ws>` 또는 절대경로 `--build-base` / `--install-base` 지정**.

세부 검출·복구: [.claude/rules/colcon-cwd.md](.claude/rules/colcon-cwd.md).

### 9.2 `.venv` 격리 (Hard rule)

> **`.venv` 는 runtime PC 가 본 workspace 외에 다른 control project 들과 공존하는 환경에서 dependency 를 격리하기 위한 의도된 설계다.** venv 활성 상태에서 `colcon test` / `colcon build` / `ros2 run` / `ros2 launch` 가 실패하면 **반드시 근본 원인을 해결** (sys.path / shebang / wrapper / dep resolution 디버그). gtest binary 직접 실행, venv deactivate 후 colcon 호출, `PYTHONPATH` 강제 우회 등 **격리 무력화 우회 금지** — runtime PC 에서 silent breakage 경로.

상세 원칙·과거 위반 사례: [feedback_venv_isolation_intent](file:///home/junho/.claude/projects/-home-junho-ros2-ws-rtc-ws-src-rtc-framework/memory/feedback_venv_isolation_intent.md).

## 10. Style Cheatsheet

상세: [agent_docs/conventions.md](agent_docs/conventions.md). 본 헌법은 변경 빈도가 낮은 절대 규칙만:

- **Namespace**: `rtc`
- **Naming**: Google C++ — `snake_case_` members, `PascalCase` types, `kConstant`
- **Units**: SI (m, rad, s, kg, N) — degree 는 API 경계에서만
- **Rotation**: quaternion (`Eigen::Quaterniond`, Hamilton) internal, ZYX Euler at boundaries
- **Variable naming**: paper notation — `J_b` (body Jacobian), `q_d` (desired joint), `K_d` (stiffness)
- **RAII**, `noexcept` on RT, `[[nodiscard]]` on status returns
- **Lifecycle**: 핵심 C++ 노드는 `rclcpp_lifecycle::LifecycleNode` — empty constructor, `on_configure` (Tier 1) + `on_activate` (Tier 2). 어떤 노드가 LifecycleNode 인지는 [agent_docs/architecture.md](agent_docs/architecture.md) 참조 (박제 금지)
- **Logger naming** (3-tier 원칙): node-owned = `<exec_name>` / library-level = `<full_package_name>` / controller-level = `<package>.<controller_key>`. 점 `.` 1개만 허용. 구체 예시: [agent_docs/conventions.md](agent_docs/conventions.md)
- **Commits**: Conventional Commits `type(scope): subject`

## 11. Post-Task Housekeeping

Commit 완료 또는 사용자가 task 종료를 알린 후:

1. **Memory save** — *surprising / non-obvious* 학습만 auto-memory 타입 규칙 (user / feedback / project / reference) 에 따라 저장. 코드 패턴, git 추출 가능한 사실, ephemeral state 는 스킵
2. **Stale artifact 정리** — `~/.claude/plans/*.md` 완료 task 용, repo-root / `/tmp` scratch files. 내용이 다른 곳 (git log, `agent_docs/*.md`, `docs/*.md`) 에 보존됨을 확인 후 삭제
3. **Memory prune** — 이제 틀리거나 outdated 된 항목 정리
4. **Harness pruning 신호 보고** (Anthropic 2026.04: 모델 capability 향상 시 harness 일부는 stripping 대상) — 이번 task 에서 (a) invariant·anti-pattern grep 이 false-positive 로 발현 (정당한 사용을 막음), (b) hook 이 정당한 변경을 잘못 차단, (c) 문서 규칙이 두 곳 이상에서 동일 내용 반복으로 drift 유발 — 중 하나라도 관찰됐다면 한 줄로 보고. 사용자가 향후 정리 task 로 분리 결정
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
