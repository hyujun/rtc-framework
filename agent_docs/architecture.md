# Architecture

구조에 대한 **규범**이다 — 어떤 코드가 RT 인가, 누가 무엇을 소유하는가, 어느 방향으로 의존하는가. layout 이력 · 결정 근거 · 기록 시점의 priority / core 값은 헌법 밖 [architecture-rationale.md](../docs/reference/architecture-rationale.md) 가 갖는다.

## Core Data Types

- `rtc_base/types/types.hpp` 가 framework-wide RT POD (`DeviceState` / `ControllerState` / `ControllerOutput`) 의 SSoT 다. 필드 목록·capacity 값은 코드가 진실이며 문서에 적지 않는다 (AP-DOC-1).
- **controller 고유 state 는 `ControllerOutput` 에 넣지 않는다** — 그 state 를 쓰는 컨트롤러가 자기 POD 와 controller-owned SeqLock 을 소유한다 (`GraspStateData`, `WbcStateData`, `ToFSnapshotData`).
- **도메인 상수는 그 도메인 패키지가 소유한다** — hand 상수는 `udp_hand_driver/udp_hand_constants.hpp` 다. `rtc_base` 에 두면 ARCH-1 위반이다.

**`efforts` lane 계약**: `DeviceState::efforts` 는 **관절 토크 [N·m]** 다. 실기 backend 가 모터 전류를 받으면 backend 경계에서 변환해 넣고 raw 는 `motor_efforts` lane 에 남긴다. effort 의미가 backend 별로 갈려 보일 때의 처방은 둘이다:

- **같은 로봇이 mode 에 따라 갈리면 producer 결함이다** — producer 를 고친다. 소비자를 command mode 로 게이팅하지 않는다 (sim 결함을 실기 제약으로 굳힌다).
- **로봇 자체가 토크 lane 을 주지 않으면 capability 경계다** — 그 프로필에서 소비자를 끈다. 축은 sim / 실기가 아니라 **로봇** 이고, 근거는 그 프로필의 config 가 소유한다.
- lane 이 모든 일반화력을 담지 않는 포함범위 위반은 `rtc::IsLaneReadable` 로 탐지되지 않는다 (lane 은 fresh · 정단위인 채다).

## Threading Model

- **Thread roster · core · priority 의 SSoT 는 `repo_scripts/config/thread_layout.yaml`** 이다. C++ tier 상수 · shell 헬퍼 · Python launch 미러는 거기서 **생성**되며 손으로 고치지 않는다 — `gen_thread_layout.py --check` 가 드리프트를 차단한다. core 번호 · priority 값을 문서에 적지 않는다.
- **RT thread** = controller ↔ hardware / sim 경계의 결정적 tick 뿐이다: `rt_control` (정기 tick + inline actuator `WriteCommand`) 과 `rt_callback` (backend state sub 처리). `mpc_main` 은 별도 RT 그룹이다 (controller 가 producer / consumer 양쪽). 그 밖 (`nrt_callback`, `nrt_logging`, `nrt_publish`, `arm_driver`, `hand_driver`, `sim_thread`, `viewer`) 은 RT 가 아니다.
- **priority 순서는 `rt_control` > `rt_callback` > `mpc_main`** 이다 — sensor callback 이 긴 MPC solve 를 항상 선점한다.
- **Core 0 는 OS / DDS / IRQ 전용이다.** 구체 cpu 집합은 머신 · tier · profile 종속이므로 `get_cm_shield_cpus <profile>` 출력이 SSoT 다 ([repo_scripts/README.md](../repo_scripts/README.md) "RT/MPC 코어 레이아웃 함수").
- **actuator command 는 `rt_control` 이 tick 안에서 `DeviceBackend.WriteCommand` 를 inline 호출해 내보낸다** (RT-safe contract). 별도 outbound thread 를 두지 않는다.
- `rt_callback` 은 DDS receive thread (CFS) 와 같은 코어를 쓴다 — launch 가 controller process 의 비-RT thread 만 그 코어로 다시 핀한다.

**MPC**:

- **RT 루프에 해를 공급하는 solver 스레드는 `mpc` role 을 공유한다** — layout role 을 새로 만들지 않는다 (`MPCThread`, `CatchingPlannerThread`). 각 컨트롤러는 `on_deactivate` 에서 자기 solver 를 `Pause` 하므로 그 코어에서 도는 것은 하나다. 같은 이름의 스레드가 여럿일 수 있으므로 판정은 검증기의 name→TID 맵이 아니라 `/proc/<pid>/task/*` 로 한다.
- **MPC solve 는 단일 스레드다.** solver 에 외부 `std::jthread` 를 넘기지 않는다 — 병렬화가 필요하면 경로는 solver 의 OpenMP (`setNumThreads(n)` + 그 풀의 affinity) 이고, 그때 manifest 에 슬롯을 넣는다 (E-7).
- **MPC 강등 축은 launch profile 하나다.** solver 축의 강등 상태를 enum 으로 다시 선언하지 않는다 — 돌아온다면 solver 의 thread budget 옆에서 파생된다 (재선언이 아니라 재구현).

**Launch profile** — tier 가 "어느 role 이 어느 슬롯에" 를 정한다면 profile 은 "이번 실행에서 어느 role 이 도는가" 를 정한다:

- profile 은 launch 단계의 **명시적 opt-out** 이다 (런타임 자동 감지 없음). launch 가 정한 값이 `rt_layout_profile` 파라미터로 controller 에, `--profile` 로 shield / 검증기에 **같은 값** 으로 전달된다. shield 계산 · 검증기의 기대 / 금지 표 · controller activation gate 셋이 이 축을 따른다.
- profile 이 뺀 role 을 controller config 가 요구하면 `on_activate` 가 **첫 side effect 전에** FAILURE 를 낸다.
- GRUB `nohz_full` / `rcu_nocbs` 는 profile 을 타지 않는다 (boot-static).

**프로세스 · 패키지 로컬 스레드**:

- 한 패키지 안에서만 쓰는 thread config 는 package-local 로 둔다 — `SystemThreadConfigs` (`rtc_base`) 확장은 PROC-3 전면 rebuild 를 부른다. 일반 `rtc_communication::Transceiver` 는 기본이 no-pin 이고 caller 가 명시적으로 핀한다.
- `arm_driver` / `hand_driver` / `sim_thread` / `viewer` 는 process-level 배치다. `taskset -a` 전-스레드 스윕을 쓰지 않는다 — 프로세스가 의도적으로 다른 slot 에 둔 스레드를 도로 끌어온다. 제어 루프가 main thread 가 아닌 프로세스는 그 프로세스의 affinity 파라미터로 그 루프만 핀한다.

### Per-thread timing CSV infrastructure

CM RT loop · MPC thread · hand UDP receiver · rt_callback 이 **같은** generic transport 와 **같은** `RtTickTimingPayload` 를 공유한다. loop · lifecycle · cadence · overrun detection · per-tick capture 는 `rtc::PeriodicRtThread` base 가 갖고 channel 은 hook override 만 더한다. **새 per-tick timing channel 은 같은 base + payload alias 를 재사용한다** — `RTControllerInterface` virtual 추가 / 새 SPSC class / 새 logger class 금지. percentile 은 post-process 다 (aggregate 는 INFO summary 만).

## Lock-Free Rules

- **SeqLock<T>**: single-writer/multi-reader, requires `is_trivially_copyable_v<T>`
- **SpscQueue<T,N> / SpscPublishBuffer<N>**: wait-free push (drops on full), power-of-2
- **try_lock only** on RT path (never block); `lock_guard` 는 lifecycle 콜백 / nrt_callback thread / 파라미터 콜백 등 non-RT 경로에서만
- **jthread + stop_token** for cooperative cancellation
- **Separate mutexes**: `state_mutex_`, `target_mutex_`, `hand_mutex_` -- never hold more than one

## RtControllerNode

`RtControllerNode` inherits from `rclcpp_lifecycle::LifecycleNode`. The constructor is empty; all initialization happens in lifecycle callbacks.

| Callback | Tier | Resources |
|----------|------|-----------|
| `on_configure` | 1 | Callback groups, parameters, controllers, publishers/subscribers, timers, eventfd |
| `on_activate` | 2 | `SelectThreadConfigs()` -> `StartRtLoop()` + `StartNrtPublishLoop()` |
| `on_deactivate` | -- | Stop RT / nrt_publish threads, clear E-STOP, reset init state |
| `on_cleanup` | -- | Reverse of `on_configure`, with one exception: the eventfds are closed **after** the device backends (their state-lane subs outlive `on_deactivate` and write those fds) |
| `on_error` | -- | `TriggerGlobalEstop("lifecycle_error")`, stop threads, full cleanup -> SUCCESS |

**Safety publishers** (`estop_pub_`, `active_ctrl_name_pub_`) use standalone `rclcpp::create_publisher` -- active regardless of lifecycle state.

**callback_group → executor binding**:

| Executor | Scheduler | Callback groups |
|---|---|---|
| `rt_callback_executor` | SCHED_FIFO (`cfgs.rt_callback`) | `cb_group_rt_callback_` — DeviceBackend state subs. MutuallyExclusive (SeqLock single-writer 보호). **state-ready 콜백은 mailbox 전용** — slot 별 dirty bit + eventfd write 만 한다 |
| `nrt_logging_executor` | SCHED_OTHER (`cfgs.nrt_logging`) | `cb_group_nrt_logging_` — timing CSV drain + deferred E-STOP log 와 `/system/estop_status` publish (RT loop 에서 도달 가능한 `TriggerGlobalEstop` / `ClearGlobalEstop` 은 atomic flag 만 세운다) |
| `nrt_callback_executor` | SCHED_OTHER (`cfgs.nrt_callback`) | `cb_group_nrt_callback_` (lifecycle services) + 모든 controller LifecycleNode 의 default group (controller-owned RobotTarget subs, `grasp_command` services) |

`nrt_publish` 는 executor 콜백이 아니라 별도 `std::jthread` + eventfd (`NrtPublishLoopEntry`) 이고, 검증기 기대표에서 자기 이름의 행을 갖는다.

### Execution Contexts (RT 판정 SSoT)

**어떤 코드가 RT 규칙에 구속되는지는 함수 이름이 아니라 그것이 실행되는 execution context 의 스케줄러가 결정한다.** "구독 콜백" 이라는 사실만으로는 판정할 수 없다 — 아래 3·5행이 반대 결론이다. RT 금지 목록 자체는 [invariants.md](invariants.md) §RT Path Invariants, 편집 중 판정 절차는 [.claude/rules/rt-path.md](../.claude/rules/rt-path.md).

| Execution context | Scheduler | RT? | 허용 연산 |
|---|---|---|---|
| `rt_control` loop (`ControlLoop`, `Compute`, mailbox drain, inline `WriteCommand`) | SCHED_FIFO | **RT** | RT-1~10 전면 구속. alloc/throw/log/lock 금지 |
| MPC thread (`MPCThread::OnTick` → `HandlerMPCThread::Solve`), 포구 계획기 (`CatchingPlannerThread::OnTick` → `PlannerCycle::Run`, 같은 `mpc` role), `UdpHandController::RunCommCycle` | SCHED_FIFO, dedicated core | **RT** | 동일. 계획기의 대기는 eventfd `poll` |
| DeviceBackend state/motor/sensor 구독 콜백 (`cb_group_rt_callback_`) | SCHED_FIFO | **RT** | **mailbox-only** — SeqLock/atomic store, memcpy, steady_clock 캡처까지 |
| `nrt_publish_thread` (`NrtPublishLoopEntry` → `PublishNonRtSnapshot`) | SCHED_OTHER | 비-RT | ROS publish 포함 자유. executor 콜백이 **아님** (std::jthread + eventfd) |
| Controller-owned RobotTarget 구독, `grasp_command` 서비스, **`SetDeviceTarget` marshal** (base `DeliverTargetMessage` 경유) (controller LifecycleNode default group) | SCHED_OTHER | 비-RT | 자유. 단 RT loop 와 공유하는 상태는 SeqLock/SPSC 경유 — target 은 base mailbox (`PushPendingTarget`) 가 그 경유를 소유하고, RT tick 의 `DrainPendingTargets()` 가 유일한 소비자다 |
| Lifecycle 콜백 (`on_configure`/`on_activate`/`on_deactivate`/`on_cleanup`), 파라미터 콜백 | SCHED_OTHER | 비-RT | 자유 — 여기서의 `push_back`·`new`·로깅은 정상이며 RT-1 위반이 아니다 |
| `DrainLog()` / CSV drain / 1 Hz aux 타이머 (`cb_group_nrt_logging_`) | SCHED_OTHER | 비-RT | 자유. RT 가 SPSC 로 넘긴 것을 여기서 포맷·기록 |

- **controller-owned target sub 은 default group 에 붙는다** — 의도된 계약이고 테스트가 잠근다. `SubscriptionOptions.callback_group` 을 명시하면 이 lane 이 조용히 옮겨간다.
- **DeviceBackend cb_group injection 의무**: 모든 backend 구현은 `Configure(node, cfg, state_cb_group)` 가 받은 `state_cb_group` 을 자신이 만드는 모든 state/motor/sensor subscription 에 적용한다 (default-group fallback 금지 — integration test 가 assert). Reentrant cb_group 금지 — SeqLock writer 가 단일 thread 여야 한다.

**ControlLoop** (rate 는 `control_rate`): device-readiness gate → assemble `ControllerState` → `Compute()` → output validation → E-STOP substitution → inline `DeviceBackend.WriteCommand` + SPSC push (nrt-publish lane) + log.

- **E-STOP latch 가 서면 controller output 은 `BuildHoldOutput()` 으로 치환돼 backend 에 도달하지 않는다** — actuator 안전이 controller 의 E-STOP hook 구현에 의존하지 않게 하는 manager 측 방어선이다. 치환은 validation 을 우회하지 않고 그 뒤에 합성된다.
- **CheckTimeouts** (50 Hz): per-group device timeout → `TriggerGlobalEstop("{group}_timeout")`
- **E-STOP triggers**: group timeout, init timeout, >= 10 consecutive RT overruns, sim sync timeout
- **TriggerGlobalEstop**: idempotent (`compare_exchange_strong`, PROC-4), propagates to all controllers

## Data Flow

```
[Robot HW / MuJoCo Sim] --JointState--> [rt_callback (FIFO)] --SeqLock--> [rt_control: RT loop @ control_rate]
    |                                          +--dirty bit + eventfd--> [nrt_publish_thread]
    +--inline--> backend.WriteCommand (actuator command, RT-safe)
    +--SPSC--> [nrt_publish_thread (CFS)] --> controller.PublishNonRtSnapshot
    |                                          (Transforms / grasp_state / wbc_state / tof_snapshot)
    |                                      +-> /rtc_cm/{group}/joint_states (digital twin)
    +--SPSC--> [nrt_logging_executor (CFS)] --> CSV (timing + per-device state + sensor)
    +--E-STOP latch--> [nrt_logging (CFS)] --> /system/estop_status + RCLCPP log (deferred, RT-10)

[Hand HW] <--UDP--> [udp_hand_driver] <--SeqLock--> [ControlLoop]
[rtc_digital_twin]: merge /rtc_cm/{group}/joint_states --> RViz2
[BT coordinator]: subscribes grasp_state + /rtc_cm/<group>/joint_states + `<config_key>/transforms` (self-feed), publishes goals
```

## RT vs non-RT Topic Ownership

토픽 소유는 3개 lane 이다.

- **Controller-owned** (controller YAML `topics:` entry 전부) — per-controller `LifecycleNode` 가 소유한다. 소유 규칙과 두 형태 (PublishRole / private SeqLock) 는 [design-principles.md](design-principles.md) §Controller-YAML Topics Are Controller-Owned. Subscribe role 은 `target` (alias `goal`) 이고 CM 은 controller-YAML target sub 을 만들지 않는다.
- **DeviceBackend-owned** — device-wire state/motor/sensor sub + command pub. `devices.<group>.backend:` 에서 선언한다.
- **CM fixed publishers** — `RtControllerNode` 가 YAML 과 무관하게 소유한다: per-group digital-twin `/rtc_cm/<group>/joint_states`, safety pub (`/system/estop_status`, `/rtc_cm/active_controller_name`). 모두 lifecycle 과 무관한 standalone publisher 다.

RT loop 가 per-tick 으로 controller 의 SeqLock writer 에 push 하고 non-RT `nrt_publish_thread` 가 read + ROS publish 한다. **actuator 송출 lane (inline) 과 nrt publish lane 은 분리를 유지한다** — 긴 non-RT publish 가 actuator latency 를 막지 못하게 하는 두 lane 이다.

외부 도구 (BT, GUI, digital_twin, shape_estimation) 는 `/rtc_cm/active_controller_name` (TRANSIENT_LOCAL) 을 구독해 switch 시 active controller 의 `/<config_key>/...` 토픽으로 rewire 한다 (pull-based — CM 은 현재 선택을 노출할 뿐 어느 namespace 가 권위인지 정하지 않는다).

**TF `_actual` 프레임 — `/tf` publisher 없음.** 컨트롤러는 arm-tip / fingertip `_actual` 프레임을 `/tf` 로 발행하지 않고 controller-owned `/<config_key>/transforms` 로만 노출한다. bare `tf2_ros::TransformListener` 는 이 프레임을 받지 못한다. **새 tf 소비자는 둘 중 하나를 반드시 적용한다**: (a) self-feed — `/<config_key>/transforms` 를 직접 구독해 buffer 에 `setTransform` (active controller 전환 시 rewire), (b) `rtc_digital_twin` 의 `/tf` 재발행에 의존.

**Session logs**: 세션 디렉토리 아래 위치는 **producer thread 의 소유자** 기준으로 정한다 — per-controller log 는 `controllers/<config_key>/`, per-tick 스레드 타이밍은 `timing/`. Session subdir 목록은 `rtc_base/logging/session_dir.hpp` (`kSubdirs`) 와 `rtc_tools.utils.session_dir` (`_SESSION_SUBDIRS`) 가 mirror 다 — 한쪽을 바꾸면 반드시 함께 바꾼다 (PROC-5).

## Dependency Graph

**이 그래프는 ARCH-2 (상향 의존 금지) 판정에 필요한 층위 요약이다 — 전체 엣지의 SSoT 는 각 패키지의 `package.xml`** 이며 여기 전수 적지 않는다 (AP-DOC-1).

```
rtc_msgs, rtc_base (independent)
  +-- rtc_communication, rtc_inference <-- rtc_base
  +-- rtc_controller_interface <-- rtc_base, rtc_msgs, rtc_urdf_bridge
  +-- rtc_controllers <-- rtc_base, rtc_msgs, rtc_math, rtc_urdf_bridge, rtc_tsid
  |     (sibling of rtc_controller_interface -- does NOT depend on it)
  +-- rtc_controller_manager <-- rtc_controller_interface, rtc_controllers,
  |         rtc_base, rtc_msgs, rtc_communication, rtc_urdf_bridge
  +-- rtc_tsid <-- rtc_math, rtc_urdf_bridge, Pinocchio, ProxSuite, Eigen3, yaml-cpp
  +-- rtc_mpc  <-- rtc_base, Eigen3, yaml-cpp, Pinocchio (+ CMake-only: fmt, aligator)
  +-- rtc_mujoco_sim <-- rtc_base, rtc_msgs, MuJoCo 3.x (optional)
rtc_math (independent) <-- Eigen3 (Pinocchio adapter optional, test-only rtc_base)
rtc_urdf_bridge <-- Pinocchio, tinyxml2, yaml-cpp
udp_hand_driver <-- rtc_communication, rtc_inference, rtc_base, rtc_msgs
robot_descriptions (data-only, no code deps)
integrated_bringup <-- rtc_controller_manager, rtc_controller_interface, rtc_controllers,
                 rtc_tsid, rtc_mpc, rtc_base, rtc_msgs, rtc_math, rtc_urdf_bridge
                 + <exec_depend> udp_hand_driver, robot_descriptions, rtc_tools, repo_scripts
rtc_tools (analysis tools, top of stack) <-- <exec_depend> rtc_msgs, pinocchio, xacro,
                 rtc_controllers (exec 전용 — rtc_controllers 는 rtc_tools 를 의존하지 않는다)
```
