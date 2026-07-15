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

> **종결 (2026-07-15).** `main` 에 no-ff merge (`e71ff6e`), post-merge 재검증 green
> (23 바이너리 / 212 tests / 0 failure). 본 artifact 는 `completed/` 로 이동됐다.
> **미완 1건 — Acceptance 4 (p1b sim smoke) 는 실행하지 않았다.** 사용자 판단으로 merge 를
> 진행했다: sim smoke 는 음성 증거이고 실증은 E7(결정적 재현)이 담당한다. 실기에서 #158
> signature (`exit -11`) 가 재발하면 이 artifact 를 먼저 읽을 것 — E7 이 반증된다는 뜻이므로
> 근본 원인 가설 자체를 재개봉해야 한다 (HP-3).

**commits 1–10 완료 (2026-07-15). Acceptance 1·2·3 충족. Acceptance 4 미실행 (위 참조).**

근본 원인은 **재현으로 확정**했고 (E7), `/code-review` (high) 가 찾은 7건을 commits 5–10 이
모두 처리했다. 리뷰는 **기존 결정 두 개를 뒤집었다** — D5 의 근거가 사실과 달랐고 (→ D5-정정),
D4-정정 이 닫았다고 본 gap 이 실은 열려 있었다 (→ D7). 상세는 Decisions.

| commit | 내용 | sensor |
|---|---|---|
| `ef09b06` | bridge null 가드 3곳 (Acceptance 2 grep 통과) | build clean, 226/226 green |
| `e826fe4` | `TickCallback` first-rewire gate (D5 설계) | build clean, 226/226 green |
| `e75878c` | `test_rewire_gate.cpp` 회귀 테스트 3개 + node test seam | build clean, **230/230 green** |
| `216ccbb` | `SetGains` 예산 분리 (D4-정정) + `switch_controller` 주석 정정 | build clean, 230/230 green |
| `27e336a` | **C1/리뷰#2** — `SwitchController` 가 rewire 까지 SUCCESS 보류 (D7) | 208/208 green, E11 |
| `8d3bbf8` | **C2/리뷰#1** — `~/step` 도 게이트, 공유 `CanTick` (D8) | 210/210 green, E10 |
| `4740d1c` | **C3/리뷰#4** — 컨트롤러 대기 상한 진단 (D9) | 211/211 green, E12 |
| `94a8989` | **C4/리뷰#3** — latch 계약 정정 + rewire 커밋 순서 (D5-정정) | 211/211 green |
| `6f8f845` | **C5/리뷰#5** — target pub → `LifecyclePublisher` + 명시 활성 (D10) | 212/212 green, E8·E9 |
| `bc6cf49` | **C6/리뷰#6·#7** — temp 파일 고유화 + CMake 중복 제거 | 212/212 green, 23 바이너리 |

카운트 추이: 226 → 230 (회귀 테스트 3 + ctest 항목 1) → 208. **208 로 줄어든 것은 회귀가 아니라
집계 기준 차이**다 — 위 4개 행은 `colcon test-result` 기준(ctest 항목 포함), 아래 6개 행은
gtest XML 의 testcase 합계다. 최종 = 23 gtest 바이너리 / 212 testcase / 0 failure.

10 commit 전부 기존 `EXPECT`/`ASSERT` 수정·삭제 **0건** (Acceptance 3 유지). C1 이 mock 2곳을
고쳤으나 **assertion 이 아니라 mock 이 프로덕션 계약을 덜 구현한 것**이었다 (E11) — E-6 무해.

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

1. ~~브랜치 `fix/issue-158-rewire-tick-gate` 생성~~ — done
2. ~~**commit 1** — bridge choke-point null 가드 3곳~~ — done (`ef09b06`)
3. ~~**commit 2** — `TickCallback` first-rewire latch gate (D1)~~ — done (`e826fe4`, D5 로 설계 변경)
4. ~~**commit 3** — 회귀 테스트 (Acceptance 1)~~ — done (`e75878c`). fix 없이 SIGSEGV 재현
   확인 완료 (E7). D6 으로 tier 결정 변경 (node seam 추가, 2026-07-15 사용자 결정).
5. ~~**commit 4** — `SetGains` discovery 여유 (D4)~~ — done (`216ccbb`). **단 이 항목의 전제는
   틀렸었다 — D4-정정 참조.** 실제로는 예산 분리(`kReadyTimeoutS` 5s / `kSrvTimeoutS` 2s)를 했고,
   `switch_controller.cpp` 순서 가정 주석 정정은 지시대로 수행.
6. ~~`/code-review` — §5.5 trigger~~ — done (2026-07-15, high effort). 7건 발견,
   **전건 수정 = commits 5–10** (C1–C6). 리뷰가 D5·D4-정정을 뒤집었다 (D5-정정, D7).
7. **Acceptance 4 — p1b sim smoke** — 남음. HP-2 대로 subagent 위임.
8. **U4 종결 코멘트** + artifact → `completed/` 이동 (§11-2)
9. 이슈 본문 편집 여부 — 사용자 컨펌 대기 (아래 `## Issue updates` 미결)

**리뷰 후 재-`/code-review` 는 하지 않기로** (2026-07-15). commits 5–10 은 리뷰 지적을 좁게
반영한 것이고 각 항목이 "고치기 전 실패하는 테스트"로 실증됐다 (E8–E12). PR 직전
`/code-review ultra` 는 §5.5 상 여전히 유효한 선택지.

## Issue updates (#158)

> **도구 주의:** `gh issue view` / `gh pr edit` 는 이 repo 에서 Projects-classic GraphQL
> deprecation 으로 hard-fail 한다. `gh api repos/:owner/:repo/issues/158` 계열로 우회한다.

| 시점 | 내용 | 상태 |
|---|---|---|
| **U1 — 착수 전 (즉시)** | 가설 기각. E1–E6 근거로 "race 아님" 선언. core dump / sudo `kernel.core_pattern` / gdb-under-churn / mutex 감사 **불필요** 명시 | ✅ **done** (2026-07-15) |
| **U2 — 착수 시** | 범위 확장 고지 (D4 = `SetGains` 예산 분리 + `switch_controller` 주석), 브랜치 링크 | ✅ **done** — U1 과 통합 |
| **U3 — commit 3 후 (결정적)** | fix 없이 SIGSEGV(-11) 하는 로그 + fix 후 230/230 | ✅ **done** — U1 과 통합 |
| **U4 — 종결** | merge `e71ff6e`, 커밋 10개, `/code-review` 7건 (그중 본문 가설과 무관한 별건 2개 — step 경로, lifecycle publisher), Acceptance 4 미실행 명시, artifact → `completed/` | ✅ **done** (2026-07-15) |

**U1–U3 은 하나의 코멘트로 통합 게시** ([#158 comment-4976488580](https://github.com/hyujun/rtc-framework/issues/158#issuecomment-4976488580),
2026-07-15). 세 시점이 모두 지난 뒤 한꺼번에 쓰게 되어 3연속 코멘트는 노이즈였다.
본문 언어에 맞춰 **영어**로 작성 (artifact 는 한글, 이슈는 영어 — 혼용이 아니라 각 매체의 관례).

**미결 — 이슈 본문 편집 여부 (사용자 판단, 여전히 열림):** #158 본문의 `## Suspected cause` 와
`## Next steps to get a backtrace` 두 절은 이제 **전체가 stale** 이다. 코멘트만 달면
본문을 먼저 읽은 사람이 여전히 오도된다. 선택지 — (a) 본문의 두 절을 편집하고 원문을 U1
코멘트에 보존, (b) 코멘트만 달고 본문은 이력으로 남김. 단일 라인이 아니라 **절 전체**가
뒤집히는 경우이므로 컨펌 없이 편집하지 않는다. **현재 상태 = (b) 잠정** — U1 코멘트가 두 절이
stale 임을 명시하고 편집 의사를 물어둔 상태.

## Handoff plan

**현재 = 코드 완료 + 리뷰 완료 (2026-07-15).** 10 commit 전부 로컬에 있고 미커밋 변경 없음.
**push 는 아직** — `origin` 은 `4ae2ded` 까지만 안다. 남은 것은 sim smoke 와 이슈 종결뿐이다.

리뷰어가 볼 곳으로 지목했던 세 가지의 결말: **D5 는 뒤집혔고** (근거가 거짓 — D5-정정),
**D4-정정도 뒤집혔으며** (gap 이 안 닫혀 있었음 — D7), D6 seam 은 문제 없었다.
`kReadyTimeoutS` 가 판단값이라는 지적은 유효하게 남아 Constraints 로 이동했다.

### 재개 진입점 — **모두 소진됨** (2026-07-15 종결)

1. ~~push~~ — done (`96d7e96` 까지 push 후 `e71ff6e` 로 merge)
2. **Acceptance 4 — p1b sim smoke** — **실행하지 않고 종결**. 사용자 판단 (2026-07-15).
   재개하려면 HP-2 대로 **subagent 위임** (대용량 log). 이슈 본문 `## Repro` 명령 그대로:
   전체 스택 launch **직후** bt_coordinator 동시 launch, 반복, exit -11 부재 확인.
   관측 가치가 남아 있다: 실제 CM 기동 시간 = Constraints 의 두 판단값(`kReadyTimeoutS` 5.0,
   `controller_wait_timeout_s` 10.0) 근거. 실기 투입 시 이 둘을 재검토할 것.
3. ~~U4 종결 코멘트 + artifact → `completed/`~~ — done
4. ~~이슈 본문 편집 여부~~ — 종결 코멘트에서 처리 (아래 `## Issue updates`)
5. ~~`/code-review ultra`~~ — high-effort `/code-review` 로 갈음, 7건 전건 수정 후 merge

### 원래 계획된 분기점 (이력)

| 분기점 | 판단 | 메커니즘 |
|---|---|---|
| ~~**HP-0 — 착수 전**~~ (지남) | #158 과 #160 은 코드 교집합 0 (production vs 테스트 전용, 둘 다 `main` 에서 분기) → **병렬 세션/에이전트 가능**. 단 아래 "추론 결합" 주의 | 두 세션으로 분리, 또는 한 세션에서 #158 → #160 순차 |
| ~~**HP-1 — commit 3 후**~~ (지남 — 사용자가 같은 세션에서 commit 4 속행 결정) | commits 1–3 은 "null pub → gate → 회귀 테스트" 하나의 추론 사슬이고, commit 4 는 **별개 사슬**(discovery 여유) | 실제 절단면은 **commit 4 후 = 현재**가 됐다 (코드 완료 → 리뷰/smoke 만 남음) |
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
- **D4-정정 — commit 4 의 지시는 전제가 stale 이었다** (2026-07-15 구현 중 발견, 사용자 결정으로
  예산 분리 채택). Next action 5 는 "전송 성공 시점부터 시계 시작" 을 지시했으나 그 동작은
  `1b301c3` (`// response clock starts at send`) 로 **이미 구현돼 있었다** — plan 이 코드를
  잘못 읽었다 (`set_gains.cpp` 두 stage 모두 send 시 `stage_start_` 재스탬프).
  **실제 gap**: readiness(discovery) 창과 response 창이 같은 `kSrvTimeoutS = 2.0` 예산을
  공유해, rewire 직후 param service discovery 가 2초를 넘기면 멀쩡한 controller 에 대해
  `SetGains` 가 FAILURE. → `kReadyTimeoutS = 5.0` (stage 진입→send) / `kSrvTimeoutS = 2.0`
  (send→response) 로 분리. 둘 다 bounded 유지 (unbounded RUNNING 방지).
  폐기한 대안: 상수 단순 상향(response 창까지 같이 늘어남), 주석만 정정(실 gap 방치).
  **교훈** — 이슈 본문뿐 아니라 **exec-plan 자신의 "Next action" 도 미검증 가설**일 수 있다.
  #158·#160 에 이어 세 번째 사례. 착수 전 grep 반증은 plan 지시에도 적용한다.
  **↑ 이 정정 자체가 또 불충분했다 — D7 참조 (네 번째 사례).** 예산 분리는 맞지만, 그것이
  닫는다고 본 gap 은 `kReadyTimeoutS` 로 닫히지 않는다.
- **D5 — gate 는 node 의 `rewire_seen_` 이 아니라 bridge 소유 latch** (2026-07-15 구현 중 결정,
  D1 의 semantics 는 그대로). `BtRosBridge::IsControllerWired()` = 두 target pub 이 non-null.
  근거: ~~pub 은 `RewireControllerTopics` 안에서만 생성되고 **절대 null 로 돌아가지 않는다**
  (rewire 가 빈 이름에 early-return — `bt_ros_bridge.cpp:695-696`)~~ **← 사실이 아니다,
  D5-정정 참조.** 따라서 pub-nullness 자체가
  이미 bridge lifetime 에 묶인 monotone latch 다. 이로써 **Constraints 의 "D1 latch 리셋 범위
  미결" 이 소멸**한다 — `on_cleanup` → `ReleaseAllResources` 가 `bridge_.reset()` 하므로
  재-configure 시 gate 가 자동 재무장하고, `on_deactivate` 는 bridge 를 남기므로 (publisher 도
  살아있음) latch 가 true 로 유지되는 것이 정확하다. 별도 bool 은 리셋을 손으로 관리해야 하고
  (누락 시 cleanup→activate 경로에서 gate 가 죽은 채 버그 재발), gate 가 막아야 할 대상(null pub)
  과 다른 것을 추적한다. 폐기한 대안: node 멤버 `rewire_seen_` + 수동 리셋.
- **D6 — 회귀 테스트는 node test seam 을 추가해 실제 tick 경로를 구동** (2026-07-15 사용자 결정).
  Acceptance 1 의 "`on_activate` → 다수 tick" 은 원안(inject tier, 프로덕션 변경 0)으로는
  **불가능**했다 — `TickCallback` 과 `bridge_` 가 private 이고 seam 이 없어, 실제 tick 경로에서
  rewire 상태를 통제하려면 real DDS(=범위 밖 e2e tier)뿐이었다. 따라서 `BtCoordinatorNode` 에
  `friend struct test::CoordinatorTickInjector` 를 추가했다 (#154 의 `BridgeStateInjector`
  선례와 동형 — 관찰 전용, 필드 poke·private 전이 호출 없음). 테스트는 임시 단일-leaf
  `UR5eHoldPose` 트리(LoadTree 가 절대경로 허용)로 configure→activate→timer tick 을 실제
  구동하고, active controller 는 bridge 핸들러로 주입해 **DDS 없이** 유지한다. arm target 관찰은
  intra-process. 폐기한 대안: bridge 단위 테스트만 (gate 3줄이 미커버 → E7 재현이 불가능했을 것).

### `/code-review` (2026-07-15) 이후 — D5-정정 및 D7–D10

- **D5-정정 — latch 가 monotone 인 이유는 "pub 이 안 죽어서"가 아니다** (리뷰 #3 → `94a8989`).
  D5 의 결론(bridge 소유 latch)은 옳지만 **근거가 사실과 달랐다**. `RewireControllerTopics` 는
  컨트롤러 스위치마다 두 pub 을 `reset()` 하고 재생성한다 — "절대 null 로 돌아가지 않는다" 는
  거짓이다. 실제로 latch 를 지탱하는 것은 **rewire 와 reader 가 같은 single-threaded executor 의
  mutually-exclusive 콜백 그룹에서 돈다**는 사실뿐이다 (E4 가 확인한 그 불변식). reset→recreate
  창을 관측할 수 있는 주체가 없다. → rewire 에 전용 콜백 그룹을 주거나 multi-threaded executor 로
  가거나 rewire 를 콜백 여러 개로 쪼개면 **#158 이 그대로 재발**하며, 기존 테스트
  (`WiredLatchSurvivesControllerSwitch`) 는 `OnActiveController` 반환 *후*에만 샘플링하므로
  못 잡는다. 헤더에 이 조건을 명시했다. 덤: `rewired_controller_` 커밋을 rewire 끝으로 이동 —
  기존엔 이름을 먼저 커밋해 create 가 throw 하면 같은 이름 재전달이 중복으로 skip 되고 pub 은
  영원히 null (게이트 영구 폐쇄, 복구 불가) 였다.
  **교훈**: D5 는 "결론이 맞아서" 검증을 통과했다. 근거의 오류는 결론이 맞을 때 더 오래 산다.
- **D7 — `SwitchController` 가 rewire 를 기다린다** (리뷰 #2 → `27e336a`, 사용자 결정).
  D4-정정이 닫았다고 본 gap 이 **실제로는 열려 있었다**. `SwitchController` 는 srv `ok` 즉시
  SUCCESS 했고, 뒤따르는 `SetGains::onStart` → `BuildParams` 가 아직 갱신 안 된
  `GetActiveController()` (= 이전 이름, 알려진 이름이라 거부도 안 됨) 로 params 를 만들어
  아직 rebind 안 된 param client 로 보냈다. 이전 컨트롤러의 param service 는 살아있으므로
  (deactivate 된 LifecycleNode 도 param 을 서빙) **엉뚱한 컨트롤러에 gains 가 적용되고 SUCCESS**.
  `kReadyTimeoutS` 는 이 경로에서 **발동조차 안 한다** — 예산은 send 재시도 중에만 흐르는데
  stale 경로는 첫 시도에 send 한다. 실제 트리 4개(`vision_approach` / `shape_inspect_simple` /
  `pick_and_place_force_pi` / `search_motion`)가 `SwitchController` 직후 `SetGains` 를 둔다.
  → `onRunning` 에 stage 2 추가: `ok` 후 `GetActiveController()` 가 target 과 일치할 때까지
  `timeout_s_` 예산으로 대기. `OnActiveController` 가 rewire 를 먼저 하고 이름을 나중에 쓰므로
  일치 관측 = 바인딩 완료 보장. **SUCCESS 의 의미가 "CM 이 스왑했다" → "트리가 이 컨트롤러와
  대화할 수 있다"** 로 바뀌고, switch 뒤에 오는 모든 노드가 그 보장을 물려받는다.
  폐기한 대안: `SetGains` 에 `controller_name` 포트 추가 (트리 XML 4곳에 이름 중복, 두 값이
  어긋나면 조용히 틀림, switch 뒤 다른 노드는 여전히 무방비), 둘 다 (예산 상호작용으로 진단 복잡).
- **D8 — 게이트는 두 tick 경로가 공유, `paused_` 는 제외** (리뷰 #1 → `8d3bbf8`).
  게이트가 `TickCallback` 에만 있어 `~/step` 서비스로 같은 gap 도달 가능했다. `TickBlockedBy`
  (당초 `CanTick`) 가 세 선행조건(tree/E-STOP/wired)을 소유하고 두 경로가 호출한다.
  **`paused_` 는 의도적으로 제외** — auto-tick 만 막는 개념이고 "일시정지 중 수동 step" 이
  서비스의 존재 이유다. 이 비대칭은 원래도 동작이었으나 "각 경로가 어떤 가드를 복사했는가" 에
  우연히 의존했고, 이제 명시된다.
- **D9 — 컨트롤러 대기 상한은 진단 전용, 대기는 무제한 유지** (리뷰 #4 → `4740d1c`, 사용자 결정).
  `controller_wait_timeout_s` (기본 10.0, `<=0` 비활성) 초과 시 ERROR 1회 후 계속 대기.
  폐기한 대안: lifecycle error 전이 (늦게 뜨는 CM 의 자동 복구가 사라지고 노드 재시작 필요),
  현상 유지 (오설정이 정상 startup 과 구분 불가한 무한 정지). `CanTick` → `TickBlockedBy` +
  `TickBlocker` enum 으로 진화한 이유: E-STOP 은 정상 상태라 ERROR 를 내면 안 되는데, 사유
  **문자열 비교로 구분하는 것은 fragile**하다.
- **D10 — target pub 은 `LifecyclePublisher` 타입 + 생성 시 명시 활성** (리뷰 #5 → `6f8f845`,
  사용자 결정). pub 들이 `rclcpp::Publisher` base 핸들에 담겨 있어 `publish()` 가 base 의
  non-virtual 오버로드로 가고 `is_activated()` 검사를 통째로 건너뛰었다 — **INACTIVE 노드가
  명령을 발행**했다 (E8). 동시에 그 slicing 이 발행이 동작하던 **유일한 이유**였다: pub 은
  rewire 에서 태어나는데 rewire 는 보통 `on_activate` 이후에 돌므로 managed-entity sweep 을
  놓쳐 영원히 비활성 상태였다 (E9). 따라서 타입 정정만 하면 모든 target 이 조용히 드롭된다 —
  둘은 **한 커밋에서 같이** 가야 한다. `shape_trigger_pub_` 는 base-handle 결함만 있고
  (생성자 출생 → sweep 커버) 함께 정정했다 — 하나만 남기면 "왜 얘만 자기 게이트를 우회하나".
  폐기한 대안: 주석만 추가 (게이트는 여전히 무효, 함정은 그대로).

## Evidence

E1–E6 은 2026-07-15, `main` @ `b152a49` 기준 **정적 인스펙션** (당시 빌드·테스트·재현 미실행).
**E7 이 2026-07-15 재현·검증으로 이를 승격했다** — E1–E6 의 인스펙션 결론이 실측으로 확인됐다.

- **E7 — 재현 확정 (결정적, #158 최초 재현)**: `e826fe4`+`ef09b06` 의 gate·가드를 작업 트리에서
  임시로 제거하고 `test_rewire_gate` 실행 → `RewireGateTest.TicksBeforeRewireDoNotCrashOrAdvance`
  가 `run_test.py: return code -11` (**SIGSEGV**) 로 사망. #158 이 보고한 `exit -11` 과 동일
  signature 이며, tick 1 에서 즉시 발생. fix 복원 후 230/230 green.
  → **Constraints 1 (재현 검증의 한계) 해소**: 이제 fix 는 음성 증거(sim smoke)가 아니라
  결정적 재현 테스트로 실증된다. #158 은 "1회 관측 후 ~28회 재현 실패" 였으나 rewire 전 tick 을
  강제하면 100% 재현된다 — 타이밍이 아니라 **순서**가 원인임을 확증.

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
  `TickCallback` 의 가드는 `!tree_` / `paused_` / `IsEstopped()` 뿐 — rewire 검사 없음.
  `config/bt_coordinator.yaml:37` `tick_rate_hz: 80.0` → 첫 tick 12.5 ms.
  **(라인 번호는 pre-fix 기준. 현재는 `TickBlockedBy` 가 rewire 를 검사하고 `StepCallback` 도
  같은 것을 쓴다 — E6 이 놓쳤던 두 번째 tick 경로가 리뷰 #1 = E10 이다.)**

### `/code-review` 후속 실증 (2026-07-15) — 각 fix 는 "고치기 전 실패"로 확인했다

- **E8 — lifecycle 게이트 우회 실증 (리뷰 #5)**: fix 전 `DeactivatedNodePublishesNoArmTarget`
  실패 — 노드를 `deactivate()` 한 뒤 `PublishArmTarget` 호출했는데 구독자가 메시지를 **받았다**
  (`arm_targets_seen_` 1 → 2). base-handle slicing 이 `is_activated()` 를 건너뛴다는 직접 증거.
- **E9 — "타입만 정정" 이 위험하다는 실증 (리뷰 #5)**: 타입을 `LifecyclePublisher` 로 바꾸고
  명시적 `on_activate()` 를 뺀 상태로 실행 → arm target 을 기대하는 **4개 테스트 전부 실패**
  (`TickPublishesArmTargetOnceWired`, `StepTicksOnceWired`, `WaitTimeoutDiagnosesButKeepsWaiting`,
  `DeactivatedNodePublishesNoArmTarget` 의 baseline). 리뷰가 예측한 함정이 실재함을 확인.
- **E10 — step 경로 gap 실증 (리뷰 #1)**: `StepCallback` 의 게이트만 무력화 →
  `StepRefusesToTickBeforeRewire` **만** 실패하고 timer 경로 3개는 통과. 새 테스트가 정확히
  두 번째 tick 경로만 겨냥함을 확인.
- **E11 — mock 이 CM 계약을 덜 구현 (리뷰 #2)**: D7 적용 직후 `SrvSwitchSucceedsImmediately` 가
  2.22s (= `timeout_s=2.0` 초과) 후 FAILURE. 원인은 회귀가 아니라 mock srv 가 `resp->ok=true` 만
  하고 name latch 를 안 한 것 — **`test_switch_controller` 와 `test_service_singlethread` 두 곳**.
  후자는 known-flake 스위트(#160)라 flake 로 오인할 뻔했으나 결정적 실패였다. 둘 다 CM 순서
  (latch → ok)대로 non-blocking publish 하도록 보강; **assertion 무변경** (E-6 무해).
- **E12 — 대기 상한 진단 발화 확인 (리뷰 #4)**: `controller_wait_timeout_s=0.05` 로
  `[ERROR] ... No active controller after 0.1s — ... Still waiting; check that a controller is
  active and publishing /rtc_cm/active_controller_name.` 1회 발화 후, 늦게 주입한 컨트롤러가
  게이트를 열고 트리가 RUNNING 도달 — 진단은 하되 포기하지 않음을 로그로 확인.

## Failed approaches

N/A — 구현 미착수. **단, #158 이슈 본문이 제안한 조사 방향은 착수 전에 기각했다**:
core dump / sudo `kernel.core_pattern` / gdb-under-churn / mutex 감사는 모두 "race 가설"
전제인데 E4 가 그 전제를 반증한다. 재현 없이 정적 확정이 가능하므로 이 경로들에 시간을 쓰지 않는다.

## Constraints / pending human decisions

- ~~**재현 검증의 한계**~~ — **해소 (2026-07-15, E7)**. fix 없이 SIGSEGV(-11) 재현 확인,
  fix 후 230/230. Acceptance 4 sim smoke 는 여전히 음성 증거지만, 이제 실증은 E7 이 담당하므로
  smoke 는 보조 확인으로 격하된다.
- ~~**D1 latch 리셋 범위 미결**~~ — **해소 (2026-07-15, D5)**. 리셋 로직 자체가 불필요해졌다:
  latch 가 bridge 소유이고 `on_cleanup` 만 bridge 를 파괴한다 (`bt_coordinator_node.cpp:185`).
- ~~**D1 이 기존 테스트를 깨뜨릴 가능성**~~ — **해소 (2026-07-15)**. ~~`BtCoordinatorNode` 를
  구동하는 테스트가 **하나도 없다**~~ — commit 3 이후로는 `test_rewire_gate` 가 구동한다
  (D6 의 seam). 판단 자체는 유효했다: 당시 `TickCallback` gate 는 어떤 테스트 경로에도 닿지
  않았고 226/226 green 으로 확인됐다. 이후 게이트를 덮는 테스트는 **의도적으로 추가**한 것이다.
- **`IsControllerWired()` latch 는 콜백 그룹 불변식에 종속** (신규, D5-정정). E4 가 확인한
  단일 스레드 불변식이 깨지면 latch 가 조용히 무너진다. rewire 에 콜백 그룹을 주거나
  multi-threaded executor 로 가는 변경은 **#158 재발 검토를 동반**해야 한다. 현재 테스트로는
  못 잡는다 (헤더에 명시).
- **`kReadyTimeoutS = 5.0` / `controller_wait_timeout_s = 10.0` 은 실측이 아니라 판단**
  (미해소). 전자는 D4-정정에서, 후자는 D9 에서 도입했고 둘 다 근거가 "충분히 넉넉해 보임" 이다.
  실기 CM 기동 시간 실측이 있으면 조정 대상 — sim smoke (Acceptance 4) 때 관측 가치 있음.

## Workspace

- Branch: **`fix/issue-158-rewire-tick-gate`** (분기점 `main` @ `b152a49`) — 새 세션이 재개할 브랜치
- 커밋: `2175758` (artifact) → `ef09b06` → `e826fe4` → `5c56af0` (artifact) → `e75878c` →
  `9935670` (artifact) → `216ccbb` → `4ae2ded` (artifact) → `7e4235e` (artifact) →
  **`27e336a` → `8d3bbf8` → `4740d1c` → `94a8989` → `6f8f845` → `bc6cf49`** (리뷰 C1–C6) →
  `96d7e96` (artifact).
- **merge: `e71ff6e` (no-ff, `main`, 2026-07-15)** — post-merge 재검증 green.
  `main` 의 first-parent 에 보인다 (#154 와 같은 관례). 브랜치
  `fix/issue-158-rewire-tick-gate` 는 merge 후 로컬 삭제 (§11-4).
- 신규 파일: `ur5e_bt_coordinator/test/test_rewire_gate.cpp` (+ CMake `test_rewire_gate` 타깃 —
  node 가 라이브러리가 아니라 실행파일이라 `src/bt_coordinator_node.cpp` TU 를 함께 컴파일)
- ~~이 브랜치의 첫 커밋 = 본 artifact 뿐 (docs-only). 코드 변경은 아직 0건.~~
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
**라인 번호는 commits 1–10 으로 대부분 밀렸다 — 심볼로 찾을 것** (박제하면 재-stale).

- `BtRosBridge::Publish{ArmTarget,ArmJointTarget,HandTarget}` — 역참조 3곳, 이제 null 가드 있음
- `BtRosBridge::RewireControllerTopics` — 유일한 pub 할당처이자 **reset 처** (D5-정정의 핵심).
  끝에서 `rewired_controller_` 커밋 + 조건부 `on_activate()` (D10)
- `BtRosBridge::RewireControllerTopics` 위의 THREADING 주석 — E4 로 유효성 확인, D5-정정이
  latch 를 여기에 **명시적으로 종속**시킴
- `BtCoordinatorNode::on_activate` — tick timer 무조건 시작 (+ D9 의 `activate_time_` 스탬프)
- `BtCoordinatorNode::TickBlockedBy` — 두 tick 경로 공유 게이트 (D8). 호출부 = `TickCallback`,
  `StepCallback`
- `src/nodes/ur5e_hold_pose.cpp` `onStart` — tick 1 무조건 publish (crash trigger, 1회성)
- `trees/hand_motions.xml` `FullDemo` Parallel — crash 경로
- `src/nodes/set_gains.cpp` `kReadyTimeoutS` / `kSrvTimeoutS` — 예산 분리 (D4-정정).
  **`BuildParams` 가 stale 이름을 읽는 것이 D7 의 출발점**
- `src/nodes/switch_controller.cpp` `onRunning` stage 2 (`awaiting_rewire_`) — D7
- 잠재 동일 위험 (범위 밖, 가드로 커버): `src/nodes/move_to_pose.cpp:47`,
  `move_to_joints.cpp:64`, `track_trajectory.cpp:43,69`, `grasp_control.cpp:78,153`
- `CLAUDE.md` §5.5 (`/code-review` trigger), §6 E-6 (assertion 약화 금지), §9.1 (colcon CWD)
