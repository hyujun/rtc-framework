# issue-160: test_service_singlethread — discovery 시계와 deadlock 시계 분리

## Goal

`ServiceSingleThread.SetGainsGraspCompletesUnderSharedExecutor` 의 잔존 flake (#160,
30회 중 1회) 를 제거한다. **테스트 하네스의 시계 의미만** 고친다 — production 코드와
assertion 은 건드리지 않는다.

## Acceptance criteria

1. Release full-suite 10/10 연속 green (`ur5e_bt_coordinator`), 특히 `test_service_singlethread`
2. 기존 `EXPECT`/`ASSERT` 수정·삭제 **0건** (E-6) — diff 는 `SpinTickToCompletion` 하네스와
   `ready` 술어에만 닿는다
3. Deadlock 검출력 보존: 옛 blocking `SwitchController`/`SetGains` 구조를 흉내내면
   테스트가 여전히 실패해야 한다 (하네스가 deadlock 을 못 잡게 되면 회귀 테스트가 죽은 것)
4. 실패 시 진단 가능: discovery 초과와 deadlock 초과가 **서로 다른 메시지**로 구분되어야 함

## Out of scope

- **#158** — production rewire gap + null pub (별도 브랜치 `fix/issue-158-rewire-tick-gate`,
  artifact `issue-158-rewire-tick-gate.md`)
- `test_helpers.hpp` 픽스처 — **이 테스트는 그걸 include 하지 않는다** (Current state 참조)
- inject tier (#154 에서 30/30 무결)
- production `kSrvTimeoutS` — #158 범위 (D4 of `issue-158-rewire-tick-gate.md`)

## Current state

**구현 완료, 4개 Acceptance 전부 실증 (2026-07-15).** `main` 에 rebase 됨 (#158 merge 후).
diff 는 `SpinTickToCompletion` 하네스 한 곳 (`test_service_singlethread.cpp`, +50/-7).
production 무변경, assertion 무변경. 상세는 Evidence E7-E10.

> **#158 이 이 계획을 부분적으로 선점했다.** `216ccbb` 가 production 의 `kSrvTimeoutS` 를
> discovery(`kReadyTimeoutS = 5.0`) 와 response(`kSrvTimeoutS = 2.0`) 로 쪼갰다. 그 결과
> 아래 "Next action" 3번(T2 제거용 probe client)이 **불필요해져 드롭**됐다 (D4).

### #160 이슈 본문의 "Direction 4" 는 이 테스트에 적용되지 않는다 (기각)

#160 은 #154 의 Direction 4 ("e2e 픽스처 executor 교체 — background `spin_some` 폴링 루프
→ 전용 executor `SingleThreadedExecutor::spin` + `cancel()`") 를 범위로 잡았다. 그러나:

- `test_service_singlethread.cpp` 는 **`test_helpers.hpp` 를 include 하지 않는다** (Evidence E1).
  `RosTestFixture` 의 background `spin_some` 루프를 쓰지 않는다.
- 이 테스트는 **이미** 전용 `SingleThreadedExecutor::spin` + thread-safe `cancel()` 구조다
  (`test_service_singlethread.cpp:46-77`) — 즉 4개 e2e 바이너리 중 Direction 4 가 **이미 적용된
  유일한 바이너리**이며, 그럼에도 flake 하는 바이너리다.

따라서 Direction 4 는 no-op 이다. #160 이 남긴 후보 중 살아있는 것은 **"service ready-wait 의
discovery-aware 재시도"** 뿐이고, 실제 원인은 그보다 앞단이다.

### 실제 근본 원인 — 8초 guard budget 이 discovery 를 삼킨다

`SpinTickToCompletion` 의 `budget = 8s` 는 `exec.spin()` 시점부터 도는 **wall-clock** 이다
(`test_service_singlethread.cpp:62-71`). 여기엔 tick 이 시작되기도 전의 DDS 시간이 전부 포함된다:

- **T1** — latched `/rtc_cm/active_controller_name` (transient_local) 의 discovery + 전달,
  그리고 `OnActiveController` → `RewireControllerTopics` 완료
- **T2** — rewire 가 방금 만든 grasp client (`ns + "/grasp_command"`) 와 mock 서버의 매칭

`ready` 술어(`bridge->GetActiveController() == ctrl`)가 성립할 때까지 tick 이 아예 안 도는데,
guard 시계는 그동안에도 계속 흐른다.

### 관측 증상이 T1 지배를 확정한다

> **⚠ 이 논증은 pre-#158-commit-4 (`216ccbb`) 기준이다.** 당시엔 stage 전체가 단일
> `kSrvTimeoutS = 2.0` 을 썼다. `216ccbb` 이후 send **이전** 대기는 `kReadyTimeoutS = 5.0`
> 이므로, 같은 논증을 현재 코드에 적용하면 함의가 "첫 tick 이 t > 6s" → **"t > 3s"** 로
> 느슨해진다. 관측 signature(`RUNNING` + `grasp_calls = 0`) 자체와 **결론(T1 지배)은 불변** —
> `216ccbb` 는 T1 을 건드리지 않는다. 아래 숫자는 원 관측의 재현이므로 그대로 둔다.

실패 로그는 `status` = **`RUNNING`** (`FAILURE` 아님) + `grasp_calls` = 0 이다. 이게 결정적이다:

- `SetGains` 의 자체 stage timeout 은 `kSrvTimeoutS = 2.0` 이었다 (당시 `set_gains.cpp:35,208`;
  현재는 `:51-52` 의 `kReadyTimeoutS` / `kSrvTimeoutS` 2분할).
- grasp-only 트리(`<SetGains grasp_command="2"/>`)는 port 가 없어 `onStart` 에서 곧장
  `Stage::kGrasp` 로 진입하고 `stage_start_` 를 찍는다.
- 따라서 **grasp 단계에 진입만 했다면** T2 > 2s 일 때 노드가 `FAILURE` 를 냈어야 한다.
- `RUNNING` 이 나왔다는 건 8초 guard 가 먼저 터졌다는 뜻 → **첫 tick 이 t > 6s 에야 일어났다**
  → T1 이 지배항이다. churn 하에서 latched 메시지 전달이 6초 넘게 걸린 것.

`OnActiveController` 는 이름보다 rewire 를 **먼저** 한다 (`bt_ros_bridge.cpp:171-178`) — 즉
`ready()` 성립은 grasp client 의 **존재**를 보장하지만 **discovery 완료는 보장하지 않는다**.
그래서 T2 도 별도로 다뤄야 한다.

## Next action

1. ~~브랜치 생성~~ ✅ — `main` 에 rebase 완료 (충돌 0, 파일 교집합 0)
2. ~~`SpinTickToCompletion` budget 2분할~~ ✅ — `discovery_budget = 30s` (`exec.spin()` ~
   `ready()` 최초 성립) + `deadlock_budget = 8s` (`ready()` 성립 시점부터). timer 콜백이
   `ready()` 최초 true 에서 `ready_seen` 을 세우고 guard 가 5ms 폴링으로 시계를 갈아탄다.
   두 초과는 각자의 `ADD_FAILURE()` 메시지 (Evidence E8/E9).
3. ~~T2 제거 (probe client)~~ ❌ **드롭 — D4 참조.** #158 `216ccbb` 가 production 에
   discovery 5s grace 를 이미 넣었고, 5s < 8s deadlock budget 이므로 T2 초과 시 guard 보다
   `SetGains` 자신의 discovery timeout 이 먼저 터져 명확한 메시지를 낸다.
4. ~~Sensor: 10-run 루프~~ ✅ — full suite 10/10 green (Evidence E7)
5. **남음** — #160 에 U1/U2 코멘트 (아래 Issue updates 표), 이슈 본문 편집 방침 확정 후 close

## Issue updates (#160)

> **도구 주의:** `gh issue view` 는 이 repo 에서 Projects-classic GraphQL deprecation 으로
> hard-fail 한다. `gh api repos/:owner/:repo/issues/160` 계열로 우회한다.

| 시점 | 내용 | 왜 필요한가 |
|---|---|---|
| **U1 — 착수 전 (즉시)** | Direction 4 기각. Evidence E1(`test_helpers.hpp` 미include) + E2(이미 전용 executor 구조)를 근거로 "적용 대상이 아니어서 구현해도 no-op". 실제 원인(guard 시계가 discovery 를 삼킴, E3/E4)과 D1 의 어긋남 경위 — #154 의 Direction 4 는 `test_helpers.hpp` 기반 3개 바이너리를 겨냥했는데 정작 flake 하는 바이너리는 그 픽스처를 안 씀 | 이슈 **본문의 "방향" 절 전체가 실행 가능한 지시**로 적혀 있다. 착수하는 사람이 그대로 no-op 을 구현하게 됨 |
| **U2 — 종결** | 10-run 결과 + **통계적 한계 명시** (원 빈도 1/30 → 10-run green 은 fix 를 증명 못 함; 실질 근거는 시계 의미의 구조적 논증 D2). Acceptance 3(deadlock 검출력 보존) 확인 방법과 결과 | 한계를 안 적으면 다음 사람이 10-run green 을 "증명" 으로 오독. #154 가 10/10 을 근거로 종결했으나 이 flake 가 살아남은 전례가 있음 |

**미결 — 이슈 본문 편집 여부 (사용자 판단):** #160 본문 `## 방향` 절이 U1 시점에 뒤집힌다.
#158 과 동일한 성격의 판단이므로 두 이슈를 같은 방침으로 처리할 것 —
(a) 본문 편집 + 원문을 U1 코멘트에 보존, (b) 코멘트만.

**#154 역참조 불필요:** #154 는 종결됐고 그 Direction 4 는 당시 맥락(3개 e2e 바이너리)에서
틀리지 않았다. 어긋남은 #160 이 범위를 옮겨 적는 과정에서 생겼으므로 #160 코멘트로 충분하다.

## Handoff plan

이 artifact 는 handoff.md §2 template 을 채운 상태이므로 **지금 이미 독립 재개 가능**하다.

| 분기점 | 판단 | 메커니즘 |
|---|---|---|
| **HP-0 — 착수 전 (현재)** | #158 과 코드 교집합 0 → 병렬 가능. 범위가 파일 1개(`test_service_singlethread.cpp`)로 작아 **단일 세션에서 끝날 가능성이 높다** → handoff 불필요할 공산이 큼 | 착수 후 재평가 |
| **HP-1 — 10-run / 30-run 루프** | 대용량 test 출력. 메인 세션에 넣지 않는다 (user CLAUDE.md: verbose sub-task → subagent 기본값). **이 작업의 유일한 실질 handoff 지점** | subagent 위임, pass/fail 카운트 + 실패 시 해당 테스트 로그만 회수 |
| **HP-2 — 반복 실패** | 시계 분리 후에도 flake 하면 T1 지배 가설(E4)이 틀린 것 → T2 또는 제3 요인. 3회 시도 후 중단 | 재시도 금지 → artifact `Current state` 갱신 + escalate. probe client 근사(Constraints 3)를 test-only readiness accessor 로 교체할지 재검토 |

**#158 과의 순서 — 해소됨 (2026-07-15).** #158 이 먼저 merge 됐고 이 브랜치를 `main` 에
rebase 했다 (충돌 0, 예고대로 파일 교집합 0). 예고된 두 결합도 처리됐다:
E4 에 "pre-#158-commit-4 기준" 단서를 달았고 (진단력 문제이지 정합성 문제 아님 — `216ccbb` 는
T1 을 안 건드려 관측 signature 불변), **예고되지 않았던 결합**으로 `216ccbb` 의
`kReadyTimeoutS` 가 원 계획 3번을 불필요하게 만들어 드롭했다 (D4).

**Artifact 갱신 의무 — 이행됨.** Evidence E7-E10 이 실측 결과로 갱신됐다 (handoff.md §3).

## Decisions and rationale

- **D1 — Direction 4 (executor 교체) 폐기.** 이 테스트는 이미 그 구조다 (Evidence E1, E2).
  #154 owner 코멘트의 Direction 4 는 `test_helpers.hpp` 기반 3개 바이너리를 겨냥한 것인데,
  정작 flake 하는 바이너리는 그 픽스처를 안 쓴다. #160 이 범위를 옮겨 적으면서 어긋났다.
- **D2 — 시계 분리는 assertion 약화(E-6)가 아니다.** `EXPECT_EQ(status, SUCCESS)` 와
  `EXPECT_GE(grasp_calls, 1)` 은 그대로다. 바뀌는 건 "8초"가 **무엇을 재는 시간인가"**뿐이다.
  이 테스트의 목적은 deadlock 회귀 검출인데(파일 헤더 주석 1-15행), 현재 8초는 deadlock 이
  아니라 discovery 를 재고 있다. 분리하면 오히려 신호가 **더** 정확해진다 —
  budget 상향 같은 무딘 우회와 달리 deadlock 검출 임계는 8초 그대로 유지된다.
  폐기한 대안: budget 을 8s → 30s 로 상향 (deadlock 검출까지 30초로 둔해지고, 진짜
  deadlock 회귀 시 suite 가 30초씩 매달림).
- **D3 — grasp 서비스 pre-wait 이 deadlock 을 가리지 않는다.** deadlock 은 tick 안에서
  future 를 blocking wait 할 때 같은 executor 가 응답을 못 dispatch 하는 구조적 문제다.
  ticking **전에** discovery 를 기다리는 건 그 구조를 건드리지 않는다 — 옛 blocking 코드는
  discovery 가 끝난 뒤에도 여전히 매달린다. Acceptance 3 가 이걸 명시적으로 검증한다.
  **E8 이 이를 실증했다** — blocking 구조에서 하네스가 정확히 deadlock 으로 분류해 실패.
- **D4 — probe client (원 계획 3번) 드롭.** #158 `216ccbb` 가 production 의 send-이전 대기에
  `kReadyTimeoutS = 5.0` grace 를 줬다. deadlock budget(8s)이 그보다 크므로 T2 가 늘어지면
  guard 가 아니라 `SetGains` 자신이 `timeout (5.0s) waiting for service discovery in
  grasp_command stage` 로 먼저 실패한다 — 이미 명확한 진단이라 Acceptance 4 가 probe 없이
  충족된다. 드롭으로 Constraints 의 "probe client 가 관측을 바꿀 가능성" 도 함께 해소:
  probe 자체가 discovery 트래픽이고 bridge 의 client 와 다른 엔티티라 근사 프록시였다.
  대가: T2 초과가 여전히 테스트 실패로 남는다 (숨기지 않음). 그건 production 의 5s discovery
  예산을 실제로 초과했다는 뜻이므로 **보고할 가치가 있는 신호**이지 하네스 결함이 아니다.
- **D5 — Acceptance 3 은 일시적 수동 확인 (a) 으로.** 영구 negative-control 테스트 (b) 는
  의도적으로 8s+ 를 태우는 테스트를 스위트에 상주시킨다. 증거 휘발은 E8 에 결과·재현 절차를
  박아 상쇄한다 (#158 이 E8-E12 에서 쓴 방식과 동일).

## Evidence

E1-E6 은 2026-07-15, `main` @ `b152a49` 기준 **정적 인스펙션**. E7-E10 은 구현 후 실측
(`main` rebase 후, Release, ws-root + `setup_env.sh`).

- **E1** — `test_service_singlethread.cpp` 는 `test_helpers.hpp` 미include:
  `grep -n 'include "' test/test_service_singlethread.cpp` → `set_gains.hpp`,
  `switch_controller.hpp`, `bt_ros_bridge.hpp` 뿐. 반면 `test_set_gains` / `test_grasp_control` /
  `test_switch_controller` 는 셋 다 `test_helpers.hpp` 를 include → **Direction 4 대상 아님**
- **E2** — 이미 전용 executor 구조: `test_service_singlethread.cpp:46-77` `SpinTickToCompletion`
  = `SingleThreadedExecutor exec` + `exec.spin()` + guard 스레드의 `exec.cancel()`.
  background `spin_some` 폴링 없음 (`test_helpers.hpp:156-162` 의 그 루프를 안 씀)
- **E3** — guard 시계가 spin 시작부터: `test_service_singlethread.cpp:62-71`,
  `const auto start = steady_clock::now()` 가 `exec.spin()` **이전**에 찍힘
- **E4** — 노드 자체 timeout 이 2.0s: `src/nodes/set_gains.cpp:35` `kSrvTimeoutS = 2.0`,
  `:208` `if (ElapsedSeconds(stage_start_) > kSrvTimeoutS) return FAILURE`
  → 관측된 `RUNNING`(≠`FAILURE`)이 "첫 tick 이 6초 후" 를 함의.
  **⚠ pre-#158-commit-4 (`216ccbb`) 기준** — 현재 코드는 send-이전이 `kReadyTimeoutS = 5.0`
  (`set_gains.cpp:51-52`) 이라 함의가 "t > 3s" 로 느슨해진다. 결론(T1 지배)은 불변
- **E5** — rewire-before-name 순서: `src/bt_ros_bridge.cpp:171-178` `OnActiveController` 가
  `RewireControllerTopics(msg->data)` 를 먼저 호출 → `ready()` 는 client **존재**만 보장,
  discovery 완료는 미보장 (T2 가 별도로 남는 이유)
- **E6** — 기존 flake 증거: `docs/exec-plans/completed/issue-154-test-fixture-split.md`
  (30-run 중 1회, 2026-07-15) + #160 본문의 2026-07-14 동일 테스트 실패 이력
- **E7 — Acceptance 1**: full suite **10/10 green**, `test_service_singlethread` 단독도 10/10
  (각 0.33s). **이건 fix 를 증명하지 않는다** — 아래 통계적 한계 참조
- **E8 — Acceptance 3 (deadlock 검출력 보존)**: `set_gains.cpp` kGrasp 의 send 직후에
  `grasp_future_.wait_for(30s)` 를 **일시 삽입** (= 옛 blocking 구조: 보낸 tick 안에서 대기)
  → `SetGainsGrasp...` **FAILED (30058 ms)**, 하네스 메시지 = `deadlock budget (8000ms)
  expired: the tree was being ticked but never reached a terminal status — a service wait
  that only this executor could fulfil`. **정확히 deadlock 으로 분류** (discovery 아님).
  패치 되돌림 확인: `git diff -- src/nodes/set_gains.cpp` 비어 있음
- **E9 — Acceptance 4 (반대쪽 시계)**: gains 테스트의 latched `active_pub->publish()` 를
  **일시 제거** → `ready()` 영구 false → **FAILED (30018 ms)**, 메시지 = `discovery budget
  (30000ms) expired: 'ready' never held, so the tree was never ticked — DDS discovery/rewire,
  not a deadlock`. 같은 `RUNNING` 결과가 원인에 따라 다른 메시지로 갈림을 실증
- **E10 — Acceptance 2 (E-6)**: `git diff -U0 | grep -E '^[-+].*(EXPECT_|ASSERT_)'` → **0건**.
  assertion 추가·삭제·수정 전무. 30s discovery budget 이 CTest 기본 60s timeout 에 들어옴도
  E8/E9 가 실측 (최악 30.3s)

## Failed approaches

- **#160 본문의 Direction 4** — 착수 전에 기각 (D1, Evidence E1/E2). 적용 대상이 아니어서
  구현해도 no-op. 재시도 금지.
- **negative control 1차 시도 — 실패한 재현 (E8 의 전 단계)**: `wait_for(0)` → `wait_for(30s)`
  로만 바꿔 blocking 을 흉내내려 했으나 **테스트가 통과**했다. send 와 wait 사이에 tick 이
  한 번 끊기므로 그 틈에 executor 가 응답을 이미 dispatch 해 future 가 ready 였다. 옛 blocking
  구조의 본질은 "**같은 tick 안에서** send 후 대기" 이며, 그렇게 고쳐야 재현된다 (E8).
  → deadlock 재현을 시도하는 다음 사람은 send 와 wait 사이에 tick 경계를 두지 말 것.

## Constraints / pending human decisions

- **flake 재현의 통계적 한계 (미해소, 구조적)** — 원 빈도가 1/30 이므로 10-run green(E7)은
  fix 를 증명하지 못한다 (fix 없이도 10/30 확률로 통과). Acceptance 1 은 필요조건일 뿐이고,
  실질 근거는 시계 의미의 구조적 논증(D2) + 두 시계가 각자 발화함을 보인 E8/E9 다.
  더 강한 증거가 필요하면 30-run 을 요청할 것.
- ~~**Acceptance 3 의 검증 방법 미결**~~ — **해소**: (a) 채택 (D5), E8 에서 실증.
- ~~**probe client 가 관측을 바꿀 가능성**~~ — **해소**: probe 자체를 드롭 (D4).

## Workspace

- Branch: **`fix/issue-160-e2e-clock-split`** — 2026-07-15 `main` 에 **rebase 완료**
  (#158 merge 후. 충돌 0 — 파일 교집합이 실제로 0이었음)
- 커밋: artifact + 하네스 fix (`test_service_singlethread.cpp`). production 무변경
- `?? docs/WBC_CONTROLLER_IMPLEMENTATION.md` — **본 작업과 무관, 별도 소유.** 커밋하지 말 것
- **남은 일**: #160 U1/U2 코멘트 + 본문 편집 방침 확정 → close, main merge
- **자매 artifact 는 `main` 에 있다** — #158 종결로 `docs/exec-plans/completed/issue-158-rewire-tick-gate.md`

## Pointers

- Issue #160 — e2e 잔존 flake (본문의 Direction 4 는 D1 로 기각됨)
- Issue #154 / `docs/exec-plans/completed/issue-154-test-fixture-split.md` — 픽스처 분리 완료,
  Evidence 절에 10-run 루프 절차
- `docs/exec-plans/completed/issue-158-rewire-tick-gate.md` — 자매 작업 (production, 종결).
  그 D4 가 `kReadyTimeoutS` 분할의 근거
- `test/test_service_singlethread.cpp:1-15` — 이 테스트의 목적(deadlock 회귀) 헤더 주석
- `test/test_service_singlethread.cpp:40-120` — `SpinTickToCompletion` (두 시계 + 분기 메시지)
- `test/test_service_singlethread.cpp:181-186` — 원 flake 테스트 `SetGainsGrasp...` 와 `ready` 술어
- `test/test_helpers.hpp:156-162` — Direction 4 가 겨냥했던 background `spin_some` 루프 (범위 밖)
- `src/bt_ros_bridge.cpp:173-183` — `OnActiveController` rewire-before-name
- `src/nodes/set_gains.cpp:51-52` — `kReadyTimeoutS`(5.0) / `kSrvTimeoutS`(2.0) 2분할 (#158)
- `src/nodes/set_gains.cpp:225-227` — `stage_sent_` 기반 budget 선택
- `CLAUDE.md` §6 E-6 (assertion 약화 금지), §9.1 (colcon CWD)
