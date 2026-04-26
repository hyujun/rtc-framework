# rtc_cm Lifecycle & Switch Service Plan

## Context

`rtc_controller_manager`의 controller 선택은 현재 `/<robot_ns>/controller_type` (String request) + `/<robot_ns>/active_controller_name` (latched String confirm) topic 한 쌍으로 이루어진다. 모든 controller는 부팅 시 일괄 `on_activate` 되고 — 즉 **controller-level lifecycle 상태가 active/inactive로 분리되어 있지 않다**. 클라이언트(BT)는 fire-and-forget publish 후 latched topic을 polling해 close-loop 확인한다.

이 문서는 ros2_control 패턴을 참고해 다음을 도입하기 위한 design lockdown이다:

1. `/rtc_cm/list_controllers` srv — 현재 controller 상태 query (sync)
2. `/rtc_cm/switch_controller` srv — activate/deactivate 요청 (sync)
3. controller별 lifecycle state 분리 — active 1개 외 나머지는 inactive 상태로 대기

비교 분석은 본 문서의 [Reference](#reference) 섹션 인용 참조 (대화 history 분석본).

## Decisions (locked)

| ID | 결정 | 비고 |
|---|---|---|
| **D-A1** | **Single-active 유지** | RT dispatch는 `active_controller_idx_` atomic int 1개 (현재 그대로). multi-active는 별도 phase로 분리 |
| **D-A2** | **Namespace `/rtc_cm/...` (global, robot-agnostic)** | `rtc_cm`이 주관하는 영역. multi-CM 시 service 이름 충돌은 single-CM 가정으로 회피. ARCH-1 위반 아님 (robot_ns prefix가 오히려 robot-specific) |
| **D-A3** | **`rtc_msgs`에 srv 자체 정의** | `controller_manager_msgs` 의존 없음. 미래 ros2_control bridge가 필요해지면 변환 layer 추가 |
| **D-A4** | **Sync 응답** | srv 응답은 새 active set이 RT loop에 visible해지고 latched confirm topic publish 완료 후 반환. timeout = `Request.timeout` (default 1 s) |
| **D-A5** | **Hold init은 controller에 전가** | `RTControllerInterface::on_activate(prev_state, const ControllerState& devices)` 시그니처 확장. base 구현은 device snapshot으로 `InitializeHoldPosition()` 호출 |
| **D-A6** | **기존 topic 병행 후 제거** | Phase 3 srv 도입 + Phase 4 BT 마이그레이션 + Phase 5 legacy 제거. `/active_controller_name` latched topic은 영구 유지 (외부 노드 rewire trigger) |
| **D-A7** | **State enum: `unconfigured` / `inactive` / `active` / `finalized`** | ros2_control 표준과 동일. RTC에서는 inactive↔active만 switch_controller로 전환, 나머지는 CM 자체 lifecycle에 의존 |
| **D-B6** | **E-STOP 활성 중 switch 거부** | srv는 `ok=false, message="E-STOP active"`. 안전 책임 명확화 |
| **D-OQ6** | **MPC thread는 옵션 Z (Pause/Resume)** | `rtc::mpc::MPCThread`에 cv-기반 Pause/Resume 추가. DemoWbc on_activate/on_deactivate에서 토글. factory/solver workspace 재사용 |

## Pre-flight Findings (from B-1 audit)

각 controller의 lifecycle hook(`on_activate`/`on_deactivate`)이 switch마다 호출돼도 안전한지 점검 결과.

### F-1. Safe (no action)

- **rtc_controllers** (4): JointPD, OSC, CLIK, P → lifecycle override 없음, base의 SUCCESS no-op 사용. **완전 안전**
- **DemoJoint / DemoTask**: `Activate/DeactivateOwnedTopics()` 만 호출 — `LifecyclePublisher::on_activate/on_deactivate()` 토글이 전부, heap/throw/RCLCPP/mutex 없음 ([owned_topics.cpp:138-172](../ur5e_bringup/src/controllers/owned_topics.cpp#L138-L172)). **안전**

### F-2. DemoWbc — prereq 필요

`DemoWbcController::on_activate` ([demo_wbc_controller.cpp:1816-1840](../ur5e_bringup/src/controllers/demo_wbc_controller.cpp#L1816-L1840)) 가 매번 호출되면:

- `mpc_timing_logger_.Open("demo_wbc_controller")` — switch마다 CSV path truncate / 데이터 손실 가능
- `node->create_wall_timer(...)` — switch마다 새 timer 생성, on_deactivate에서 reset → executor register/unregister 비용 누적
- `RCLCPP_INFO/WARN` — aux thread라 RT-3 비위반이지만 noisy

→ **Phase 1.5 prereq P-1으로 처리** (아래 참조).

### F-3. PublishLoop ↔ on_deactivate race

`DeactivateOwnedTopics`가 호출되는 시점 publish thread가 같은 publisher에 publish 시도 가능. `LifecyclePublisher`가 inactive면 internally drop — **safe**. `mpc_timing_timer_.reset()`은 별도 cb_group이라 RT/publish thread와 무관.

### F-4. DemoWbc 결론

P-1 처리 후 lifecycle 분리 작업 안전. owned_topics는 활성/비활성 토글만 수행.

### F-5. DemoWbc의 MPC thread lifetime — 분석

MPC thread (`rtc::mpc::MPCThread` 베이스, `HandlerMPCThread`/`MockMPCThread`)는 다음 패턴:

| 단계 | 동작 |
|---|---|
| **Spawn** | `InitializeHoldPosition()`에서 lazy 1회. `mpc_thread_ == nullptr`이면 factory/handler/manager `Configure` → `Init` → `Start` → `mpc_manager_.SetEnabled(true)` ([demo_wbc_controller.cpp:1606-1687](../ur5e_bringup/src/controllers/demo_wbc_controller.cpp#L1606-L1687)). 두 번째 호출은 spawn 스킵, 기존 thread 그대로 |
| **RunMain** | `stop_token`까지 20 Hz `Solve() → manager_->PublishSolution()` ([mpc_thread.cpp:71-99](../rtc_mpc/src/thread/mpc_thread.cpp#L71-L99)). enabled flag와 무관하게 항상 동작 |
| **RT 결합** | RT loop가 매 tick `manager.WriteState(q,v)` → MPC가 다음 cycle에 `manager.ReadState()`. `manager.ComputeReference()`는 RT가 호출, `enabled_=false`면 즉시 invalid 반환 ([mpc_solution_manager.cpp:166-168](../rtc_mpc/src/manager/mpc_solution_manager.cpp#L166-L168)) |
| **Stop** | 명시적 stop 없음. controller dtor → `unique_ptr<MPCThread>` 해제 → MPCThread dtor → `Join()` |

**Lifecycle 분리 후 inactive 상태에서의 거동** (현재 코드 그대로 적용 시):

- DemoWbc가 inactive: RT loop가 `Compute()` 미호출 → `WriteState()` 미호출 → MPC thread가 **stale state로 Solve() 20 Hz 지속** → Aligator solver가 CPU/cache 점유. 의미 없는 작업 (publish solution을 RT가 ComputeReference로 안 받음)
- Re-activate: RT가 `WriteState` 재개 → fresh state 도착 → `stale_count_` 0으로 reset → 정상 복구. **자가 복구는 OK**, 데이터 정합성 문제 없음
- TripleBuffer 기반이라 race 없음

**즉, lifecycle 분리는 functional하게 안전하지만 inactive 동안 ~1 core CPU 낭비.** 정책 결정 필요 (OQ-6).

## Phase Plan

| Phase | 내용 | Output | Escalation |
|---|---|---|---|
| **0** | Design lockdown | 이 문서 (`rtc_cm_lifecycle_plan.md`) | E-9, E-10 |
| **1** | `rtc_msgs` srv/msg 정의 추가 | `ControllerState.msg`, `ListControllers.srv`, `SwitchController.srv` | E-3 (rtc_msgs ABI) |
| **1.5** | DemoWbc lifecycle hook idempotent화 (P-1) + (OQ-6=Z) MPCThread Pause/Resume (P-2) | DemoWbc activate/deactivate 안전화 + MPC thread CPU 토글 | E-1 (P-2 — RT-adjacent thread 동작 변경) |
| **2** | CM 내부 controller-level lifecycle state 분리 | `controller_states_` 추가, RT loop dispatch는 atomic load 유지 | **E-1, E-7** (RT path / thread model) |
| **3** | `/rtc_cm/list_controllers` + `/rtc_cm/switch_controller` srv 도입 | sync srv with E-STOP guard, 기존 topic 병행 | none (Phase 2 derivative) |
| **4** | BT `SwitchController` 노드 srv 마이그레이션 | `BtRosBridge` srv client + BT node `use_service` 포트 | none |
| **5** | Legacy `/<robot_ns>/controller_type` topic 제거 | CM/BT/문서 정리. `/active_controller_name` latched는 유지 | E-9 (호환성 결정) |
| **6** | 문서 + 메모리 동기화 | architecture.md, controllers.md, README, MEMORY.md | none |

### Phase 1 — `rtc_msgs` schema (사전 컨펌 후)

```
# rtc_msgs/msg/ControllerState.msg
string   name              # controller 인스턴스 이름 (Name())
string   state             # "unconfigured" | "inactive" | "active" | "finalized"
string   type              # registry plugin name (RTC_REGISTER_CONTROLLER 키)
bool     is_active         # state == "active" 와 동치, 호환 편의
string[] claimed_groups    # controller_topic_configs_[i].groups 키 전체

# rtc_msgs/srv/ListControllers.srv
---
ControllerState[] controllers

# rtc_msgs/srv/SwitchController.srv
string[] activate_controllers
string[] deactivate_controllers
int32    strictness        # STRICT=1 (default), BEST_EFFORT=2
int32    STRICT=1
int32    BEST_EFFORT=2
builtin_interfaces/Duration timeout    # default 1.0s, zero=infinite
---
bool   ok
string message
```

**Single-active 강제 (D-A1)**: srv 콜백에서 `activate.size() <= 1 && deactivate.size() <= 1` 검증. 위반 시 STRICT는 `ok=false`, BEST_EFFORT는 첫 항목만 사용 + warning.

**ros2_control 호환 의식**: 필드 이름과 strictness 의미를 맞춰 미래 변환 layer 작성 시 1:1 매핑 가능.

### Phase 1.5 — P-1: DemoWbc idempotent activation

`DemoWbcController` 의 MPC timing logger / wall_timer를 controller 인스턴스의 첫 activation에서만 setup, 이후 activation은 owned_topics activation만. on_cleanup에서만 close/reset.

**구현**:
- 새 멤버 `bool mpc_timing_initialized_{false}`
- `on_activate`:
  ```cpp
  ActivateOwnedTopics(...);
  if (mpc_enabled_ && !mpc_timing_initialized_) {
    mpc_timing_logger_.Open(...);
    mpc_timing_timer_ = node->create_wall_timer(...);
    mpc_timing_initialized_ = true;
  }
  return SUCCESS;
  ```
- `on_deactivate`: timer reset 제거 — timer는 controller 종료 시까지 살아있고, deactivate 동안에는 `LogMpcSolveTimingTick()`이 active controller가 아니라 stats를 못 받으면 silent skip (이미 `if (!stats) return;` 패턴 — [demo_wbc_controller.cpp:1858-1864](../ur5e_bringup/src/controllers/demo_wbc_controller.cpp#L1858-L1864)). owned_topics deactivate만 유지
- `on_cleanup`: 기존대로 timer reset + handles reset

**Test**: `test_demo_wbc_controller`에 N=20회 activate/deactivate cycle 추가, mpc_timing.csv가 1번만 생성됨을 검증.

**P-2 (OQ-6 = Z 채택 시 추가)**: `rtc::mpc::MPCThread`에 `Pause()` / `Resume()` 추가.

- `RunMain()` 루프 진입부에서 `paused_` flag 체크 → `cv_.wait(lock, [this]{ return !paused_ || stop_token_.stop_requested(); })`
- `Pause()`: `paused_ = true; cv_.notify_all();` (현재 cycle 끝나면 wait에 진입)
- `Resume()`: `paused_ = false; cv_.notify_all();`
- DemoWbc on_activate에서 `mpc_thread_->Resume()`, on_deactivate에서 `mpc_thread_->Pause()` (단, `mpc_thread_` null 가드 — 첫 activation은 InitializeHoldPosition에서 spawn하므로 그 이후부터 valid)
- Test: `MPCThread` unit test에 Pause/Resume cycle 검증 (solve_count_ 증가가 멈췄다 재개되는지)
- **prereq**: `MPCSolutionManager::ComputeReference`가 paused 동안 stale_count overflow되지 않게 — 이미 RT가 deactivate면 ComputeReference도 호출 안 됨, OK

### Phase 2 — Controller-level lifecycle state in CM

**자료구조 변경**:
- `RtControllerNode`에 `std::vector<std::atomic<int>> controller_states_` 추가 (값: 0=Inactive, 1=Active). `controllers_`와 같은 인덱스
- `active_controller_idx_` atomic int은 그대로 유지 (single-active 보장 + RT dispatch 변경 없음)

**Lifecycle 흐름 변경**:
- CM `on_configure` 끝: 모든 controller `on_configure` 호출 → state = Inactive
- CM `on_activate`: RT loop / publish loop 시작. **controller `on_activate`는 부르지 않음**. 대신 `initial_controller`만 `ActivateController(idx, snapshot)` 호출
- CM `on_deactivate`: 활성 controller만 `DeactivateController()` 호출, 그다음 RT/publish loop stop

**새 helper (CM 내부, aux thread only)**:
```cpp
// auto_hold용 device snapshot 생성 → ctrl->on_activate(prev, devices)
// state store(Active, release). atomic active_controller_idx_ 갱신
CallbackReturn ActivateController(std::size_t idx);

// state store(Inactive, release). active_controller_idx_는 호출자 책임
CallbackReturn DeactivateController(std::size_t idx);
```

**Switch sequence (sync)** — Phase 3에서 srv 콜백이 호출:
```
1. (precondition) E-STOP idle, name resolve OK, single-active 위반 없음
2. snapshot = build_device_snapshot()         // aux thread
3. activate target's on_activate (snapshot은 prev_state arg 확장으로 전달)
4. atomic store: active_controller_idx_ = target_idx (release)
5. wait one RT tick (cv + atomic flag 또는 단순 sleep_for(1.5 * dt_us))
6. previous controller's on_deactivate
7. publish /active_controller_name (latched)
8. return ok
```

**RT path 영향**:
- `rt_controller_node_rt_loop.cpp:125` 의 `active_controller_idx_.load(acquire)` 그대로
- `rt_controller_node_rt_loop.cpp:61` 의 idx load 후 `controllers_[idx]->Compute(...)`도 그대로
- 즉 **RT loop 코드는 변경 없음** — single-active 유지의 가장 큰 이득

**`RTControllerInterface::on_activate` 시그니처 확장 (D-A5)**:
```cpp
virtual CallbackReturn on_activate(
    const rclcpp_lifecycle::State &previous_state,
    const ControllerState &device_snapshot) noexcept;
```
base 구현은 `if (auto_hold) InitializeHoldPosition(snapshot);` 호출. 기존 override들은 snapshot 인자 추가 + 무시.

### Phase 3 — srv 도입

**Service 등록**:
```cpp
// 모두 cb_group_aux_ 사용 (RT sensor callback과 분리)
auto list_srv  = create_service<rtc_msgs::srv::ListControllers>(
    "/rtc_cm/list_controllers", ...);
auto switch_srv = create_service<rtc_msgs::srv::SwitchController>(
    "/rtc_cm/switch_controller", ...);
```

**`ListControllers` callback**:
- `controllers_` 순회 → `ControllerState{name, state, type, is_active, claimed_groups, update_rate_hz}` 생성
- `state` 결정: `controller_states_[i].load(acquire)` → "active"/"inactive"
- `claimed_groups`: `controller_topic_configs_[i].groups`의 키 + ownership=kController 또는 kManager 구분 없이 group 이름들

**`SwitchController` callback** (Phase 2의 sequence 그대로):
- 검증 실패 / E-STOP / single-active 위반 → `ok=false`
- timeout 초과 → `ok=false`
- 성공 → `ok=true`, `message`에 from→to 요약

**기존 topic 호환성**: `/<robot_ns>/controller_type` 구독은 그대로 유지. 내부적으로는 srv path와 동일한 helper (`SwitchActiveController(name)`) 호출하도록 refactor.

### Phase 4 — BT 마이그레이션

**`BtRosBridge`**:
- `switch_controller_client_` (`rclcpp::Client<rtc_msgs::srv::SwitchController>`) 추가
- `list_controllers_client_` 추가
- 기존 `select_ctrl_pub_` 유지 (rollback용, deprecated 표시)
- 새 헬퍼 `bool RequestSwitchController(name, timeout, message)` — sync 호출, timeout 내 응답 없으면 false

**`SwitchController` BT 노드 ([nodes/switch_controller.cpp](../ur5e_bt_coordinator/src/nodes/switch_controller.cpp))**:
- 새 input port `BT::InputPort<bool>("use_service", true, "...")`
- `use_service=true` 분기: `bridge_->RequestSwitchController(...)` 한 번 호출, 성공 시 SUCCESS (load_gains 분기는 그대로)
- `use_service=false` 분기: 기존 publish + active_controller_name polling 유지
- `onRunning()`의 polling 로직은 service 분기에서 불필요 (sync 응답)

**Test**:
- `ur5e_bt_coordinator/test/test_switch_controller.cpp`에 service 분기 케이스 추가
- E-STOP 활성 시나리오에서 `ok=false` 받는 시나리오

### Phase 5 — Legacy 제거 (D-A6)

**제거 대상**:
- `RtControllerNode::controller_selector_sub_` (`/<robot_ns>/controller_type`)
- `BtRosBridge::select_ctrl_pub_`
- `SwitchController` BT 노드의 `use_service` 포트 (강제 srv path)

**유지 대상**:
- `/<robot_ns>/active_controller_name` latched topic — shape_estimation, digital_twin, BT bridge가 rewire trigger로 사용 (`bt_ros_bridge.cpp:64-72`, `shape_estimation_node.cpp:83`)
- `/<robot_ns>/controller_gains`, `/<robot_ns>/request_gains`, `/<robot_ns>/current_gains` — switch와 무관한 gain 채널, 그대로

**Sensor**: 전 downstream — ur5e_bringup launch + shape_estimation + digital_twin smoke + BT scenario 풀 패스.

### Phase 6 — 문서 동기화 + plan doc 정리

- `agent_docs/architecture.md`: lifecycle 다이어그램에 controller-level state 추가
- `agent_docs/controllers.md`: switch flow를 srv 기반으로 갱신
- `agent_docs/testing-debug.md`: Sensor matrix에 `/rtc_cm/list_controllers` smoke 추가
- `rtc_controller_manager/README.md`: srv 인터페이스 + 예시 (`ros2 service call /rtc_cm/switch_controller ...`)
- `CLAUDE.md` §9 Common Commands에 srv 호출 예시 추가
- `controller-safety-improvements.md`에 본 plan 결과 cross-link

**Plan doc 정리** (사용자 지시 반영):
- 모든 phase 완료 + commit 후 본 문서 (`agent_docs/rtc_cm_lifecycle_plan.md`) 제거 또는 1-page summary로 축약
- `CLAUDE.md` §12 reference docs 항목 제거, §6 escalation E-11 제거
- `MEMORY.md`의 `project_rtc_cm_lifecycle.md` 항목은 "complete (Phase 0~6 done)" 한 줄로 축약 후 유지 (향후 retrospective 참조용)
- Plan 자체는 git history에 남으므로 archive 의미 유지

## Sensor Matrix

| Phase | 필수 Sensor | 추가 Sensor |
|---|---|---|
| 0 | none (doc only) | 사용자 컨펌 |
| 1 | `colcon test --packages-select rtc_msgs` + `./build.sh full` | downstream 빌드 통과 |
| 1.5 | `test_demo_wbc_controller` (N회 activate/deactivate cycle 신규 케이스) + (OQ-6=Z) `rtc_mpc` MPCThread Pause/Resume gtest | mpc_timing CSV 1회 생성 검증 + Pause 동안 solve_count_ 정지 검증 |
| 2 | `test_core_controllers` + 신규 `test_controller_lifecycle` (CM 단위) | RT timing log p99 회귀 (`mpc_solve_timing.csv`, `timing_log.csv`) |
| 3 | `ros2 service call /rtc_cm/list_controllers` smoke + 신규 `test_switch_service` gtest | E-STOP 활성 시 거부 검증 |
| 4 | `test_switch_controller` (BT, srv 분기) + 실제 grasp 시나리오 smoke | latency 실측 (M-1) |
| 5 | 전 downstream smoke (ur5e_bringup launch + shape_estimation + digital_twin) | BT full scenario |
| 6 | 문서 빌드 (mkdocs/없음) — 링크 깨짐 grep | — |

## Monitoring (M)

| ID | 항목 | 측정 시점 |
|---|---|---|
| M-1 | switch service 응답 latency (base controller, DemoWbc 각각) | Phase 3 통합 테스트 |
| M-2 | N=20회 연속 switch ASan 빌드 — 누수/race | Phase 4 통합 테스트 |
| M-3 | RT loop deadline miss 비율 (`overrun_count_` 회귀) | Phase 2/3 양쪽 |

## Open Questions

| ID | 질문 | 결정 시점 |
|---|---|---|
| OQ-1 | `controller_states_`를 `std::vector<std::atomic<int>>` vs `std::atomic<uint32_t>` bitmask로 둘지 (single-active만 보장하는데 bitmask가 multi-active 미래 확장에 유리) | Phase 2 진입 전 |
| OQ-2 | Phase 2 step 5 (RT tick 1회 대기)를 cv/atomic flag로 정확 동기화할지, `sleep_for(1.5/freq)`로 단순 처리할지 | Phase 2 |
| ~~OQ-3~~ | ~~`claimed_groups` 의미~~ | **closed** — `controller_topic_configs_[i].groups`의 모든 키 (ownership 구분 없음). 단순성 우선 |
| ~~OQ-4~~ | ~~`update_rate_hz` 필드~~ | **closed** — 제외. CM 단일 rate이므로 `ros2 param get /<cm_node> control_rate`로 query |
| ~~OQ-5~~ | ~~`activate_asap` 필드~~ | **closed** — 제외. RTC에서 의미 없음. ros2_control bridge가 필요해질 때 변환 layer가 무시 처리 |
| ~~OQ-6~~ | ~~DemoWbc inactive 시 MPC thread 정책 (F-5 참조). 옵션 X/Y/Z/W 중 택일~~ | **closed → D-OQ6 = 옵션 Z** |

### OQ-6 옵션 (참고용 — D-OQ6에서 Z 채택)

| Option | 동작 | 변경 범위 | Trade-off |
|---|---|---|---|
| **X** (현재 패턴 유지) | MPC thread는 첫 activation 후 controller dtor까지 평생. inactive 동안 stale state로 Solve() 지속 | 0 (no-op) | inactive 동안 1 core CPU 낭비 (Aligator). switch latency는 0 추가 |
| **Y** (deactivate에서 stop, activate에서 재spawn) | on_deactivate에서 `RequestStop+Join`, on_activate에서 매번 새 인스턴스 spawn (factory + handler 재생성) | DemoWbc on_activate/on_deactivate 재작성. `MPCThread::Start()`의 `running_.exchange(true)` 가드 회피 위해 매번 새 unique_ptr | switch latency가 factory cost로 ↑ (~수십 ms~수백 ms 가능). 깨끗한 lifecycle |
| **Z** (Pause/Resume — **추천**) | `MPCThread`에 `Pause()/Resume()` 추가 — `std::condition_variable`로 RunMain 루프 일시정지. on_activate에서 Resume, on_deactivate에서 Pause | `rtc_mpc/include/rtc_mpc/thread/mpc_thread.hpp` + `mpc_thread.cpp`에 Pause/Resume 추가. DemoWbc는 on_activate/on_deactivate에서 호출만 | factory/solver workspace 재사용 (init cost 회피). switch latency ≪ Y. CPU 부담 없음 |
| **W** (Manager.SetEnabled만 토글) | thread는 평생, `mpc_manager_.SetEnabled(false)`만 호출 | DemoWbc on_activate/on_deactivate 1줄 추가 | RT consumer는 즉시 끊김 (이미 그래서 사실상 X와 동일). MPC thread CPU는 그대로 X와 같음 — **부분 해결만** |

**제 추천: Z**. Phase 1.5에 `MPCThread::Pause()/Resume()` 추가를 prereq로 포함. Pause/Resume 자체는 작은 PR (~50 줄), unit test 1개면 검증 가능.

만약 사용자가 **X**를 선택하면 (가장 단순) — Phase plan 변경 없이 진행, 다만 inactive 동안 CPU 낭비를 수용한다는 결정 로그.

## Risks

| Risk | Severity | 완화 |
|---|---|---|
| Phase 2 RT 동기화 결함 — RT가 옛 idx로 dispatch하는 마지막 tick과 새 controller의 on_activate 시점 race | Critical | OQ-2 cv 동기화 채택 + ASan 통합 테스트 (M-2) |
| `on_activate(prev, snapshot)` 시그니처 변경으로 모든 override 동시 수정 필요 (rtc_controllers 4개 base만 → ur5e_bringup demo 3개) | Warning | base default 구현이 `InitializeHoldPosition(snapshot)` 호출, override는 `(void)snapshot`만 추가하면 호환 |
| ros2_control 호환을 위해 schema 1:1로 갔다가 나중에 변환 layer 부담 | Info | strictness/timeout 의미 맞춰두면 변환 layer가 직선적 |
| BT가 srv migration 도중 timeout 잘못 잡아 grasp 시나리오 deadlock | Warning | Phase 4 단계적 — `use_service` 포트로 rollback 가능 |
| E-STOP 중 switch 거부가 복구 시나리오 차단 가능 (예: PD로 fallback 의도) | Warning | 사용자 검토 — 향후 `force` 플래그 도입 여지 |
| Inactive DemoWbc의 MPC thread CPU 낭비 (F-5) | Warning | OQ-6 결정 — 옵션 Z (Pause/Resume) 추천. 옵션 X 선택 시 inactive 시간 길어지면 thermal/contention 영향 |
| 옵션 Z 채택 시 MPCThread Pause/Resume 구현 결함 → solve가 멈추지 않거나 deadlock | Warning | unit test (P-2)로 Pause→Resume cycle 회귀 + ASan |

## Reference

- ros2_control srv: `/opt/ros/jazzy/share/controller_manager_msgs/srv/{ListControllers,SwitchController,LoadController,ConfigureController,UnloadController}.srv`
- ros2_control CM 헤더: `/opt/ros/jazzy/include/controller_manager/controller_manager/controller_manager.hpp` (`RTControllerListWrapper`, `SwitchParams`)
- 현재 RTC switch 흐름: [rtc_controller_node_topics.cpp:137-208](../rtc_controller_manager/src/rt_controller_node_topics.cpp#L137-L208), [bt_ros_bridge.cpp:64-72](../ur5e_bt_coordinator/src/bt_ros_bridge.cpp#L64-L72), [nodes/switch_controller.cpp](../ur5e_bt_coordinator/src/nodes/switch_controller.cpp)
- RTControllerInterface lifecycle 인터페이스: [rt_controller_interface.hpp:60-88](../rtc_controller_interface/include/rtc_controller_interface/rt_controller_interface.hpp#L60-L88)
- 관련 invariants: [invariants.md](invariants.md) RT-1/3/4 (lifecycle hook은 aux thread에서만 호출 → 형식적 비위반이지만 cost 모니터링 필요), ARCH-1 (rtc_*에 robot-specific 금지), PROC-1 (코드↔문서 동기화), PROC-3 (rtc_msgs 변경 시 전체 빌드)
