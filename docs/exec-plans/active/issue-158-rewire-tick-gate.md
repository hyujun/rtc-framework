# issue-158: bt_coordinator rewire-gap — tick-time SIGSEGV + rewire-후 discovery 여유

## Goal

`bt_coordinator_node` 가 `on_activate` 직후 `RewireControllerTopics` 완료 전에 tick 하면서
null publisher 를 역참조해 SIGSEGV 하는 문제(#158)를 제거한다. 동일한 "rewire 직후
discovery 미완" 뿌리에서 나오는 `SetGains` 의 `kSrvTimeoutS` 여유 부족도 같은 범위에서 처리한다
(2026-07-15 사용자 결정 — 아래 Decisions D4).

## Acceptance criteria

1. **신규 회귀 테스트 (inject tier, DDS 불필요)**
   - active controller 미발행 상태에서 `on_activate` → 다수 tick: SIGSEGV 없음, 트리가 진행하지 않음
   - 이후 active controller 발행 → rewire 성립: `UR5eHoldPose` 가 arm target 을 **실제로 publish** 함
     (= gate 가 `onStart` 의 1회성 publish 를 삼키지 않음 — Decisions D2 의 함정)
2. `Publish{Arm,Hand}Target` / `PublishArmJointTarget` 3곳 모두 null 가드 존재 (grep 으로 검증 가능)
3. Release full-suite green (`ur5e_bt_coordinator`), 기존 `EXPECT`/`ASSERT` 수정·삭제 **0건** (E-6)
4. `hand_motions.xml` p1b sim smoke: 전체 스택과 bt_coordinator 동시 launch 를 반복해도 exit -11 없음

## Out of scope

- **#160** — `test_service_singlethread` e2e 시계 의미 분리 (테스트 전용, 별도 브랜치
  `fix/issue-160-e2e-clock-split`, artifact `issue-160-e2e-clock-split.md`)
- `test_helpers.hpp` e2e 픽스처 재구조화 — #154 에서 동작 중립 완료, 재개봉하지 않는다
- inject tier 픽스처 (#154 에서 30/30 무결)

## Current state

**분석 완료, 코드 미변경.** 근본 원인은 코드 인스펙션으로 확정했다 (아래 Evidence).
아직 **재현으로는 확인하지 않았다** — #158 은 1회 관측 후 ~28회 재현 실패한 타이밍 의존 버그다.

### 근본 원인 (inspection-confirmed)

`arm_target_pub_` / `hand_target_pub_` 는 `BtRosBridge` **생성자에서 만들어지지 않고**
`RewireControllerTopics` 안에서만 생성되는데, 역참조 3곳에 null 가드가 **하나도 없다**.
tick timer 는 `on_activate` 직후 rewire 여부와 무관하게 시작한다 (80 Hz → 12.5 ms 후 tick 1).
`hand_motions.xml` `FullDemo` 의 `Parallel` **첫 자식**이 `UR5eHoldPose` 이고 그 `onStart` 는
tick 1 에 곧바로 `PublishArmJointTarget` 를 호출한다.

따라서 `/rtc_cm/active_controller_name` (transient_local latched) 의 discovery + 전달이
`on_activate` + 12.5 ms 안에 끝나지 못하면 tick 1 이 null pub 를 역참조한다.

이슈의 관측 4가지가 모두 이 하나로 설명된다:

| #158 관측 | 설명 |
|---|---|
| activate 직후 즉시 SIGSEGV | tick 1 이 12.5 ms 후 발화 |
| peak DDS churn 에서만 | churn 이 latched 메시지 전달을 12.5 ms 밖으로 밀어냄 |
| gdb 에서 재현 안 됨 | ptrace 오버헤드가 discovery 에 settle 시간을 줌 |
| settle 후 28회 무사 | participant 가 이미 match → 전달이 activate 전에 끝남 |
| `5ede20a` 이전엔 미발현 | 그때는 load 시점에 죽어 tick 경로 도달 불가 (latent) |

**#158 이슈 본문의 가설은 기각한다** — "rmw/FastDDS internals 또는 cached-state read vs
callback write race" 가 아니다. 단일 스레드 불변식은 실제로 성립한다 (Evidence E4).
따라서 이슈가 제안한 next steps (sudo `kernel.core_pattern`, core dump, gdb-under-churn,
mutex 감사) 는 **모두 불필요**하다.

**독립 교차검증:** #154 구현 노트(2026-07-14 이슈 코멘트)가 같은 위험을 이미 지적했다 —
"`PublishArmTarget`/`PublishHandTarget` dereference `arm_target_pub_`/`hand_target_pub_`
without a null check, and those publishers are only created inside the rewire". 당시엔
테스트 픽스처 맥락이라 production tick 경로로 연결되지 않았다.

## Next action

1. 브랜치 `fix/issue-158-rewire-tick-gate` 생성 (from `main` @ `b152a49`)
2. **commit 1** — `bt_ros_bridge.cpp` choke-point null 가드 3곳
   (`PublishArmTarget:375`, `PublishArmJointTarget:383`, `PublishHandTarget:395`).
   `RCLCPP_WARN_THROTTLE` + early return. **RT 경로 아님** (BT tick 은 non-RT) — RT-1 무관.
3. **commit 2** — `bt_coordinator_node.cpp` `TickCallback` first-rewire latch gate (D1).
   `rewire_seen_` 멤버 추가, `on_cleanup`/`on_deactivate` 에서 리셋 여부 확인.
4. **commit 3** — inject-tier 회귀 테스트 (Acceptance 1). `BridgeStateInjector` 로
   rewire 전/후 상태를 만들 수 있으므로 DDS 불필요.
5. **commit 4** — `SetGains` discovery 여유 (D4). `kSrvTimeoutS` 의미 재검토:
   not-ready 재시도 구간을 timeout 시계에서 제외(전송 성공 시점부터 시계 시작)하는 쪽이
   상수 상향보다 정확 — `set_gains.cpp:208` 의 `stage_start_` 의미를 바꾸는 작업.
   `switch_controller.cpp:64-68` 의 cross-channel 순서 가정 주석도 같이 정정.
6. Sensor: `agent_docs/testing-debug.md` 의 `ur5e_bt_coordinator` 행 + p1b sim smoke (Acceptance 4)
7. `/code-review` — production 코드 + lifecycle 콜백 변경이므로 §5.5 trigger 해당

## Issue updates (#158)

> **도구 주의:** `gh issue view` / `gh pr edit` 는 이 repo 에서 Projects-classic GraphQL
> deprecation 으로 hard-fail 한다. `gh api repos/:owner/:repo/issues/158` 계열로 우회한다.

| 시점 | 내용 | 왜 필요한가 |
|---|---|---|
| **U1 — 착수 전 (즉시)** | 가설 기각. Evidence E1–E3(null pub) + E4(단일 스레드 불변식 성립) + E5/E6(crash 경로) 를 근거로 "race 아님, 정적 확정" 선언. 이슈가 제안한 core dump / sudo `kernel.core_pattern` / gdb-under-churn / mutex 감사는 **불필요**임을 명시 | #158 본문이 "needs a backtrace to confirm" 이라 **읽는 사람을 sudo core-dump 삽질로 보낸다**. 가장 시급한 업데이트 |
| **U2 — 착수 시** | 범위 확장 고지: D4 로 `kSrvTimeoutS` + `switch_controller.cpp:64-68` 순서 가정이 이 이슈 범위에 포함됨. 브랜치 `fix/issue-158-rewire-tick-gate` 링크 | 별건으로 보이는 변경이 왜 이 브랜치에 있는지 리뷰어가 알아야 함 |
| **U3 — commit 3 후 (결정적)** | fix **없이** inject-tier 회귀 테스트가 실제로 SIGSEGV 하는 로그. fix 후 통과 로그 | 유일하게 근본 원인을 **재현으로** 입증하는 증거. Constraints 1 의 해소 지점 |
| **U4 — 종결** | Acceptance 1–4 결과, 커밋 목록, artifact → `completed/` 이동 링크 | #154 종결 코멘트와 같은 형식 |

**미결 — 이슈 본문 편집 여부 (사용자 판단):** #158 본문의 `## Suspected cause` 와
`## Next steps to get a backtrace` 두 절은 U1 시점에 **전체가 stale** 이 된다. 코멘트만 달면
본문을 먼저 읽은 사람이 여전히 오도된다. 선택지 — (a) 본문의 두 절을 편집하고 원문을 U1
코멘트에 보존, (b) 코멘트만 달고 본문은 이력으로 남김. 단일 라인이 아니라 **절 전체**가
뒤집히는 경우이므로 컨펌 없이 편집하지 않는다.

## Handoff plan

이 artifact 는 handoff.md §2 template 을 채운 상태이므로 **지금 이 시점에서 이미 독립 재개
가능**하다 — 새 세션이 transcript 없이 `Next action` 1번부터 시작할 수 있다. 사용자가 지금
handoff 를 택하면 추가 준비 작업은 없다.

| 분기점 | 판단 | 메커니즘 |
|---|---|---|
| **HP-0 — 착수 전 (현재)** | #158 과 #160 은 코드 교집합 0 (production vs 테스트 전용, 둘 다 `main` 에서 분기) → **병렬 세션/에이전트 가능**. 단 아래 "추론 결합" 주의 | 두 세션으로 분리, 또는 한 세션에서 #158 → #160 순차 |
| **HP-1 — commit 3 후 (권장 분기점)** | commits 1–3 은 "null pub → gate → 회귀 테스트" 하나의 추론 사슬이고, commit 4 (`kSrvTimeoutS`) 는 **별개 사슬**(discovery 여유). context 압박 시 여기가 가장 깨끗한 절단면 | `/compact "issue-158 commit 4: kSrvTimeoutS discovery 여유"` 또는 artifact 갱신 후 `/clear` |
| **HP-2 — sim smoke 직전** | Acceptance 4 의 p1b 전체 스택 launch 는 대용량 log — 메인 세션에 넣지 않는다 (user CLAUDE.md: verbose sub-task → subagent 기본값) | subagent 위임, 요약만 회수. **실패 시에만** 메인으로 raw log 회수 |
| **HP-3 — 반복 실패** | 회귀 테스트가 fix 없이도 **안 죽으면** 근본 원인 가설이 틀린 것 (Constraints 1). 3회 시도 후 중단 | 재시도 금지 → artifact `Current state` 를 반증으로 갱신 + §6 escalate. #158 재개봉 |

**추론 결합 (병렬 시 주의):** 코드는 안 겹치지만 commit 4 가 `set_gains.cpp:208` 의 stage
시계 의미를 바꾸면, #160 artifact 의 E4 논증(`RUNNING`≠`FAILURE` → "첫 tick 이 6초 후")이
**그 이후 발생하는 실패에는 더 이상 성립하지 않는다**. 진단력의 문제이지 정합성 문제는 아니다
(commit 4 는 T1 을 건드리지 않으므로 #160 의 관측 signature 자체는 불변). #158 이 먼저
merge 되면 #160 artifact 의 E4 에 "pre-#158-commit-4 기준" 단서를 달 것.

**Artifact 갱신 의무 (sender checklist):** 현재 Evidence 절은 **"정적 인스펙션만 — 빌드·테스트·
재현 미실행"** 이라고 명시돼 있다. 각 commit 후 실제 build/test 결과로 갱신하지 않으면 다음
receiver 가 "검증됐다"고 오독한다. handoff.md §3 — `done` 을 쓰기 전에 반드시 sensor 로 검증.

## Decisions and rationale

- **D1 — tick gate 는 first-rewire latch, 매 tick gate 아님** (2026-07-15 사용자 결정).
  `rewire_seen_` 이 한 번 true 가 되면 다시 검사하지 않는다. 매 tick gate 는 실행 중
  컨트롤러가 빠질 때 트리를 통째로 얼려 기존 트리/테스트 동작을 바꿀 위험이 있다.
  폐기한 대안: 매 tick `GetActiveController().empty()` 검사.
- **D2 — null 가드만으로는 불충분** (gate 가 주(主), 가드가 보조).
  `UR5eHoldPose::onStart` 는 **한 번만** publish 한다 (`ur5e_hold_pose.cpp:36`, 이후 `RUNNING` 반환).
  가드만 넣으면 tick 1 의 publish 가 조용히 삼켜지고 팔이 영원히 안 움직이며 `RUNNING` 으로
  매달린다 — crash 가 silent hang 으로 바뀔 뿐이다. gate 가 `onStart` 자체를 rewire 이후로
  미뤄야 맞다. 가드는 `hand_motions.xml` 밖의 나머지 경로(`move_to_pose`, `move_to_joints`,
  `track_trajectory`, `grasp_control`)에 대한 defense-in-depth 로 유지한다.
  폐기한 대안: 가드만 넣는 최소 변경.
- **D3 — 가드는 action node 가 아니라 bridge choke point 에.** 역참조는 bridge 3곳이지만
  호출부는 ~10곳이다. bridge 에서 막으면 한 곳, 노드마다 막으면 열 곳 + 신규 노드마다 재발.
  `SetGains::BuildParams` (`set_gains.cpp:79-87`) 가 이미 같은 방어 패턴(빈 controller →
  깔끔한 `FAILURE`)을 쓰고 있어 일관적이다.
- **D4 — `kSrvTimeoutS` 는 이번 범위 포함** (2026-07-15 사용자 결정).
  별도 이슈 등록 대신 #158 브랜치에서 처리 — 같은 "rewire 직후 discovery 미완" 뿌리다.
  폐기한 대안: 새 이슈로 분리, exec-plan 에 기록만.

## Evidence

모두 2026-07-15, `main` @ `b152a49` 기준. **정적 인스펙션만 — 빌드·테스트·재현 미실행.**

- **E1** — 생성자에 publisher 생성 없음:
  `sed -n '19,104p' src/bt_ros_bridge.cpp | grep 'arm_target_pub_\|hand_target_pub_'` → **0 hit**
- **E2** — 유일한 할당처가 `RewireControllerTopics` 내부:
  `grep -n 'arm_target_pub_ *=\|hand_target_pub_ *='` → `:742`, `:744` (둘 다 rewire 안)
- **E3** — 역참조 3곳, null 가드 0곳:
  `grep -n 'arm_target_pub_->\|hand_target_pub_->\|if (!arm_target_pub_\|if (!hand_target_pub_'`
  → `:375`, `:383`, `:395` (전부 `->publish`), 가드 hit **0**
- **E4** — 단일 스레드 불변식 성립 확인 (`ros2-concurrency-reviewer` 감사):
  `main.cpp:16` `rclcpp::spin(...)` → 단일 `SingleThreadedExecutor`. production `src/`·`include/`
  전체에 `create_callback_group` / `Reentrant` / `MultiThreadedExecutor` / 추가 `std::thread` **없음**.
  `bt_ros_bridge.cpp:672-676` THREADING 주석의 가정은 실제로 유효 → **race 아님**.
  `controller_topics_mutex_` / `state_mutex_` / `health_mutex_` 는 writer/reader 대칭으로 정상.
- **E5** — crash 경로 확정: `trees/hand_motions.xml:242-244` `FullDemo` 의 `Parallel` 첫 자식이
  `<UR5eHoldPose pose="demo_pose"/>`; `src/nodes/ur5e_hold_pose.cpp:36` 이 `onStart` 에서
  무조건 `PublishArmJointTarget` 호출 (`SwitchController` 완료와 무관하게 동시 tick)
- **E6** — tick timer 무조건 시작: `bt_coordinator_node.cpp:144-146` (`on_activate`),
  `TickCallback:354-370` 의 가드는 `!tree_` / `paused_` / `IsEstopped()` 뿐 — rewire 검사 없음.
  `config/bt_coordinator.yaml:37` `tick_rate_hz: 80.0` → 첫 tick 12.5 ms

## Failed approaches

N/A — 구현 미착수. **단, #158 이슈 본문이 제안한 조사 방향은 착수 전에 기각했다**:
core dump / sudo `kernel.core_pattern` / gdb-under-churn / mutex 감사는 모두 "race 가설"
전제인데 E4 가 그 전제를 반증한다. 재현 없이 정적 확정이 가능하므로 이 경로들에 시간을 쓰지 않는다.

## Constraints / pending human decisions

- **재현 검증의 한계** — #158 은 1회 관측 / ~28회 재현 실패다. Acceptance 4 (sim smoke)
  는 **음성 증거**일 뿐 fix 를 증명하지 못한다. fix 의 실증은 Acceptance 1 의 결정적
  inject-tier 테스트가 담당한다 (rewire 전 tick 을 강제 → 현재 코드에서 SIGSEGV 재현,
  fix 후 통과). **먼저 fix 없이 그 테스트가 실제로 죽는지 확인**해야 회귀 테스트로서 유효하다.
- **D1 latch 리셋 범위 미결** — `on_deactivate` → `on_activate` 재순환 시 `rewire_seen_` 을
  리셋할지. bridge 가 살아있으면 publisher 도 살아있으므로 리셋 불필요할 수 있다.
  `ReleaseAllResources` / `on_cleanup` 이 bridge 를 reset 하는지 확인 후 결정.
- **D1 이 기존 테스트를 깨뜨릴 가능성** — active controller 없이 activate → tick 을 기대하는
  기존 테스트가 있으면 E-6 충돌. 구현 전 `test_lifecycle_config.cpp` + step-mode 경로 grep 필요.
  **assertion 을 약화시키지 말고**, 충돌 시 §6 E-6 으로 escalate.

## Workspace

- Branch: **`fix/issue-158-rewire-tick-gate`** (분기점 `main` @ `b152a49`) — 새 세션이 재개할 브랜치
- 이 브랜치의 첫 커밋 = **본 artifact 뿐** (docs-only). **코드 변경은 아직 0건.**
  #154 관례를 따랐다 — exec-plan 은 feature 브랜치에서 작업과 함께 살다가 merge 로 main 에 들어간다
  (`bf45d91` 이 main 의 first-parent 가 아님을 확인)
- 미커밋 변경: 없음
- `?? docs/WBC_CONTROLLER_IMPLEMENTATION.md` — **본 작업과 무관, 별도 소유.** 커밋하지 말 것
- **자매 artifact 는 이 브랜치에 없다** — `issue-160-e2e-clock-split.md` 는
  `fix/issue-160-e2e-clock-split` 브랜치에 있다. Pointers 의 해당 경로는 두 브랜치가 main 에
  merge 된 뒤에야 풀린다 (의도된 것 — 각 artifact 는 자기 브랜치 소유)

## Pointers

- Issue #158 — tick-time SIGSEGV (본문 가설은 E4 로 기각됨)
- Issue #154 / `docs/exec-plans/completed/issue-154-test-fixture-split.md` — 2026-07-14 코멘트가
  null-pub 위험을 독립 지적 (교차검증)
- `docs/exec-plans/active/issue-160-e2e-clock-split.md` — 자매 작업 (테스트 전용)
- `src/bt_ros_bridge.cpp:375,383,395` — 역참조 3곳 (commit 1 대상)
- `src/bt_ros_bridge.cpp:742-745` — 유일한 publisher 할당처 (rewire 내부)
- `src/bt_ros_bridge.cpp:672-676` — THREADING 주석 (E4 로 유효성 확인됨)
- `src/bt_coordinator_node.cpp:144-146` — tick timer 무조건 시작 (commit 2 대상)
- `src/bt_coordinator_node.cpp:354-370` — `TickCallback` 가드 (commit 2 대상)
- `src/nodes/ur5e_hold_pose.cpp:36` — tick 1 무조건 publish (crash trigger)
- `trees/hand_motions.xml:242-254` — `FullDemo` Parallel (crash 경로)
- `src/nodes/set_gains.cpp:35,208` — `kSrvTimeoutS = 2.0` + stage 시계 (commit 4 대상)
- `src/nodes/switch_controller.cpp:64-68` — DDS 가 보장 않는 cross-channel 순서 가정 (commit 4)
- `src/nodes/set_gains.cpp:79-87` — `BuildParams` 의 모범 방어 패턴 (D3 근거)
- 잠재 동일 위험 (범위 밖, 가드로 커버): `src/nodes/move_to_pose.cpp:47`,
  `move_to_joints.cpp:64`, `track_trajectory.cpp:43,69`, `grasp_control.cpp:78,153`
- `CLAUDE.md` §5.5 (`/code-review` trigger), §6 E-6 (assertion 약화 금지), §9.1 (colcon CWD)
