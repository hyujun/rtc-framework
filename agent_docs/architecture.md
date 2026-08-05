# Architecture

## Core Data Types

`rtc_base/types/types.hpp` 가 framework-wide POD 의 SSoT. Robot/hand 별 capacity 상수와 필드 list 는 *코드 자체가 진실* — 문서엔 owner 패키지·도메인 경계만 박제한다 (AP-DOC-1).

**Domain ownership** (어느 패키지가 어느 POD 의 owner 인가):

| POD | Owner header | 의미 |
|---|---|---|
| `DeviceState` / `ControllerState` / `ControllerOutput` | `rtc_base/types/types.hpp` | Framework-wide RT trivially-copyable POD. ControllerOutput 에 `grasp_state`/`wbc_state`/`tof_snapshot` 필드 없음 — controller-owned SeqLock 으로 이관 |
| `rtc::grasp::GraspStateData` | `rtc_controllers/grasp/grasp_state.hpp` | Force-PI 데모 (DemoJoint/DemoTask) 전용. 각 controller 가 자체 `SeqLock<GraspStateData>` 소유 |
| `integrated_bringup::WbcStateData` | `integrated_bringup/controllers/wbc/wbc_state.hpp` | TSID 데모 (DemoWbc) 전용. Controller 자체 `SeqLock<WbcStateData>` |
| `integrated_bringup::ToFSnapshotData` | `integrated_bringup/controllers/tof_snapshot.hpp` | ToF 거리 + tip pose snapshot |
| Hand 도메인 상수 (`kNumHandMotors`, `kMaxFingertips`) | `udp_hand_driver/udp_hand_constants.hpp` | rtc_base 에 두면 ARCH-1 위반이므로 hand 도메인 소유 |

필드 list / capacity 값은 위 헤더 직접 참조 (`grep -n 'struct.*Data' <header>`).

## Threading Model

Thread roster·core·priority 의 SSoT 는 **`repo_scripts/config/thread_layout.yaml`** (선언형 manifest) 다. C++ tier 상수 + `SelectThreadConfigsForCoreCount()` (`thread_config_generated.hpp`), shell 헬퍼 (`repo_scripts/scripts/lib/thread_layout_generated.sh`), Python launch 미러 (`rtc_tools/rtc_tools/launch/thread_layout_generated.py`) 가 전부 거기서 **생성**되며, `gen_thread_layout.py --check` 가 드리프트를 CI 에서 차단한다 (issue #153 M1 — 그 전에는 같은 표가 실행 코드 6곳에 손으로 인코딩돼 있었고 그중 5곳에 직접 테스트가 없었다). `SystemThreadConfigs` 구조체 정의와 런타임 wrapper `SelectThreadConfigs()` 는 각각 `thread_config.hpp` / `thread_utils.hpp` 에 남는다. 4/6/8/10/12/14/16-core 레이아웃을 자동 선택. 문서엔 *불변 원칙*만 박는다.

**RT thread 정의 (layout v4)**: "RT thread" = controller ↔ hardware/sim 경계의 결정적 tick 만. 즉 `rt_control` (정기 tick + inline actuator WriteCommand) 과 `rt_callback` (backend state sub 처리) — 둘이 SCHED_FIFO 로 묶이는 그룹이다. 다른 thread (`nrt_callback`, `nrt_logging`, `arm_driver`, `hand_driver`, `sim_thread`, `viewer`) 는 RT 가 아니다 (mpc_main/workers 는 별도 RT 그룹 — controller 가 producer/consumer 양쪽).

**RT priority hierarchy**:

```
90 rt_control (Core 1)  >  70 rt_callback (Core 2 + DDS recv co-pin)  >  60 mpc_main (Core 3)  >  55 mpc_workers (Core 4 on 10c, Core 4-5 on ≥ 12c)
```

- **Core 0 reserved** for OS / DDS / IRQ only — ≥ 6-core 모든 tier 에서 nrt_logging / nrt_callback 이 Core 0 와 분리 (v4.1)
- **rt_callback + DDS co-pin on Core 2 (v4.1)**: v4 의 핵심 — `rt_callback` thread (FIFO 70) 이 DDS receive thread (CFS) 와 같은 코어를 공유. launch-time taskset 이 controller process 의 비-RT thread (DDS / aux) 만 `rt_callback` core 로 다시 핀해서 cache locality 확보. SCHED_FIFO 가 CFS 를 무조건 선점하므로 RT 결정성은 영향 없음. core 번호는 tier-aware (`rtc_tools.launch.thread_layout.get_rt_callback_core()`; 현재 모든 tier 에서 Core 2)
- **Actuator command publish inline**: `rt_control` thread (Core 1 FIFO 90, v4.1) 가 rt_loop tick 종료 시점에 `DeviceBackend.WriteCommand` 를 직접 호출 (RT-safe contract). v3 의 별도 `rt_outbound` jthread + `publish_buffer_` SPSC + eventfd 는 제거
- **MPC main < rt_callback**: sensor callback (rt_callback) 이 long MPC solve 를 항상 preempt
- **hand-private UDP receive thread** (`hand_udp_recv`, FIFO 65, hand_driver 프로세스 내부) 는 **프로세스 self-pin** 으로 affinity 상속 — `SystemThreadConfigs` 에 필드 없음 (package-local `kHandUdpRecvConfig`). 같은 프로세스는 blocking 파일 I/O 전용 `hand_aux_io` executor 스레드를 **aux slot(OS slot)** 에 따로 두며, 그 slot 은 ROS param `aux_cpu_slot`(기본 0, shell SSoT `get_os_cores()`)이다 — `rtc_base` 확장은 PROC-3 전면 rebuild 를 부르므로 의도적으로 package-local 이다 (issue #345). 일반 `rtc_communication::Transceiver` 는 `kRtUdpRecvConfig` (cpu_core=-1) 기본값으로 caller 가 명시 핀
- **arm_driver / hand_driver / sim_thread / viewer** 는 process-level pin (SCHED_OTHER, priority 0) — launch script 가 적용. sim_thread/viewer 의 cpu_core=-1 sentinel 은 모든 tier 에서 "no pin" (v4.1, cpu_shield --sim 모드에서 격리 해제된 코어 사용). 단 `arm_driver`·`hand_driver` 는 taskset 이 아니다. `hand_driver` 는 프로세스가 스스로 main 스레드를 핀하고(`use_cpu_affinity` param 이 그것까지 끈다) launch 는 `rclcpp::init()` 이 노드 생성 전에 만드는 DDS 스레드만 co-pin 한다 — 옛 `taskset -a` 전-스레드 스윕은 `hand_aux_io` 를 도로 끌어오므로 제거됐다 (issue #345). `arm_driver` 는 그 프로세스(`ros2_control_node`)의 제어 루프는 main thread 가 아닌 별도 스레드라 taskset 이 닿지 않으므로, upstream `controller_manager` 의 `cpu_affinity`/`thread_priority` 파라미터로 그 루프만 FIFO 50 + 코어에 핀한다 (issue #343). 이 값은 `SystemThreadConfigs.arm_driver` 가 아니라 launch 가 생성하는 CM 파라미터 파일이 나른다

세부 thread 종류·core 번호·priority 값은 위 header + `cpu_topology.hpp` 참조. Hybrid-CPU 감지 + BIOS 체크리스트는 [`docs/NUC_HYBRID_SUPPORT.md`](../docs/NUC_HYBRID_SUPPORT.md) (layout 분기는 v4.1 `physical_core_slots` 추상화가 처리 — 별도 hybrid config 없음).

### Per-thread timing CSV infrastructure

CM RT loop · MPC thread · hand UDP receiver 가 *동일* generic transport + *동일* `RtTickTimingPayload` 를 공유한다. Fixed-frequency loop·lifecycle·`clock_nanosleep(TIMER_ABSTIME)` cadence·overrun detection·per-tick t0~t3 capture 는 모두 `rtc::PeriodicRtThread` base 에 박혀 있고, channel 은 hook override 만 추가한다 (sim-CV wakeup, E-STOP escalation). 새 per-tick timing channel 추가 시 같은 base + payload alias 재사용 — `RTControllerInterface` virtual 추가 / 새 SPSC class / 새 logger class 금지.

SSoT 파일:
- `rtc_base/threading/periodic_rt_thread.hpp` — loop/lifecycle base
- `rtc_base/timing/rt_tick_timing_sample.hpp` — unified `RtTickTimingPayload`
- `rtc_base/timing/thread_timing_{sample,producer,csv_logger}.hpp` — generic transport

CSV consumer / drop counter / 출력 경로는 channel 별로 다르고 (`cm_timing_log.csv` / `mpc_timing_log.csv` / `hand_udp_timing_log.csv` / `rt_callback_timing_log.csv`), `<session>/timing/` 아래 저장. Aggregate stats 는 INFO summary 만, percentile 은 post-process.

## Lock-Free Rules

- **SeqLock<T>**: single-writer/multi-reader, requires `is_trivially_copyable_v<T>`
- **SpscQueue<T,N> / SpscPublishBuffer<512>**: wait-free push (drops on full), power-of-2 (controller data CSVs use `ThreadCsvProducer<Pod, N>` which wraps `SpscQueue` — Phase C)
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
| `on_cleanup` | -- | Reverse of `on_configure` (all `.reset()` / `.clear()`), with one deliberate exception: the eventfds are closed **after** the device backends, not before — the backends' state-lane subs survive `on_deactivate` and their state-ready callback writes those fds (issue #224) |
| `on_error` | -- | `TriggerGlobalEstop("lifecycle_error")`, stop threads, full cleanup -> SUCCESS |

**Safety publishers** (`estop_pub_`, `active_ctrl_name_pub_`) use standalone `rclcpp::create_publisher` -- active regardless of lifecycle state.

**RtControllerMain** uses a 3-phase executor: (1) lifecycle_executor spins for configure/activate, (2) polls until Active, (3) switches to dedicated rt_callback / nrt_logging / nrt_callback executors per the matrix below.

**callback_group → executor binding** (see [rt_controller_main_impl.cpp](../rtc_controller_manager/src/rt_controller_main_impl.cpp)):

| Executor | Thread config | Callback groups |
|---|---|---|
| `rt_callback_executor` | `cfgs.rt_callback` (SCHED_FIFO 70, Core 2 in v4.1) | `cb_group_rt_callback_` — DeviceBackend state subs (`/joint_states`, hand state/motor/sensor); injected via `DeviceBackend::Configure(node, cfg, state_cb_group)` (MutuallyExclusive contract — SeqLock single-writer 보호). DDS receive thread co-pinned to the same core via launch taskset. **state-ready 콜백은 mailbox 전용** (issue #198 Phase 2) — slot 별 dirty bit + eventfd write 만 하고, digital-twin republish 는 `nrt_publish_thread` 의 `DrainDigitalTwin()` 이 수행 |
| `nrt_logging_executor` | `cfgs.nrt_logging` (SCHED_OTHER nice -5, tier-aware core) | `cb_group_nrt_logging_` — `cm_timing_log.csv` + `rt_callback_timing_log.csv` drain + deferred E-STOP log **and `/system/estop_status` publish** (both raised as atomic flags by `TriggerGlobalEstop`/`ClearGlobalEstop`, which are reachable from the RT loop — #198 Phase 3) |
| `nrt_callback_executor` | `cfgs.nrt_callback` (SCHED_OTHER nice 0, tier-aware core — Core 0 만 4-core fallback, 그 외 tier 는 dedicated core) | `cb_group_nrt_callback_` (lifecycle services; E-STOP status 는 lifecycle 콜백의 `FlushEstopStatus()` 를 통해 간접적으로만 — 실제 publish 는 위 logging 행의 `DrainLog()`) + every controller LifecycleNode default group (controller-owned RobotTarget subs, `grasp_command` services). CM 은 RobotTarget sub 을 만들지 않는다 (issue #138). `nrt_publish` 는 별도 std::jthread + eventfd 로 같은 코어를 공유하지만 executor callback 이 아니다 — `cfgs.nrt_publish` 로 **이름을 분리**해 두 스레드가 verifier 기대표에서 각각의 행을 갖는다 (#349 D15; 이전에는 둘 다 `nrt_callback` 이라 `verify_rt_runtime.sh` 의 name→TID 맵이 하나만 보관했다) |


### Execution Contexts (RT 판정 SSoT)

**어떤 코드가 RT 규칙에 구속되는지는 함수 이름이 아니라 그것이 실행되는 execution context 의 스케줄러가 결정한다.** "구독 콜백" 이라는 사실만으로는 판정할 수 없다 — 아래 3·5행이 반대 결론이다. RT 금지 목록 자체는 [invariants.md](invariants.md) §RT Path Invariants, 편집 중 판정 절차는 [.claude/rules/rt-path.md](../.claude/rules/rt-path.md).

| Execution context | Scheduler | RT? | 허용 연산 |
|---|---|---|---|
| `rt_control` loop (`ControlLoop`, `Compute`, mailbox drain, inline `WriteCommand`) | SCHED_FIFO 90, Core 1 | **RT** | RT-1~10 전면 구속. alloc/throw/log/lock 금지 |
| MPC thread (`MPCThread::OnTick` → `HandlerMPCThread::Solve`), `UdpHandController::RunCommCycle` | SCHED_FIFO, dedicated core | **RT** | 동일 |
| DeviceBackend state/motor/sensor 구독 콜백 (`cb_group_rt_callback_`) | SCHED_FIFO 70, Core 2 | **RT** | **mailbox-only** — SeqLock/atomic store, memcpy, steady_clock 캡처까지 |
| `nrt_publish_thread` (`NrtPublishLoopEntry` → `PublishNonRtSnapshot`) | SCHED_OTHER 0 | 비-RT | ROS publish 포함 자유. executor 콜백이 **아님** (std::jthread + eventfd) |
| Controller-owned RobotTarget 구독, `grasp_command` 서비스, **`SetDeviceTarget` marshal** (base `DeliverTargetMessage` 경유) (controller LifecycleNode default group) | SCHED_OTHER 0 | 비-RT | 자유. 단 RT loop 와 공유하는 상태는 SeqLock/SPSC 경유 — target 은 base mailbox (`PushPendingTarget`) 가 그 경유를 소유하고, RT tick 의 `DrainPendingTargets()` 가 유일한 소비자다 |
| Lifecycle 콜백 (`on_configure`/`on_activate`/`on_deactivate`/`on_cleanup`), 파라미터 콜백 | SCHED_OTHER 0 | 비-RT | 자유 — 여기서의 `push_back`·`new`·로깅은 정상이며 RT-1 위반이 아니다 |
| `DrainLog()` / CSV drain / 1 Hz aux 타이머 (`cb_group_nrt_logging_`) | SCHED_OTHER nice -5 | 비-RT | 자유. RT 가 SPSC 로 넘긴 것을 여기서 포맷·기록 |

controller-owned target sub 이 **default group** 에 붙는다는 점은 의도된 계약이고 `integrated_bringup/test/test_controller_target_cb_group_invariant.cpp` 가 잠근다 — `SubscriptionOptions.callback_group` 을 명시하면 이 lane 이 조용히 옮겨가므로 그 테스트가 회귀를 잡는다.

**DeviceBackend cb_group injection 의무**: 모든 backend 구현은 `Configure(node, cfg, state_cb_group)` 가 받은 `state_cb_group` 을 자신이 만드는 모든 state/motor/sensor subscription 의 `SubscriptionOptions.callback_group` 에 적용해야 한다. ARCH-3 두 번째 구현 이후 silent default-group fallback 회귀를 막기 위해 backend integration test 가 `get_actual_callback_group() != nullptr` 을 assert 한다. Reentrant cb_group 금지 — SeqLock writer 가 단일 thread 임을 보장해야 함.

- **ControlLoop** (configurable rate, default 500 Hz): device-readiness gate -> assemble ControllerState -> `Compute()` -> output validation (#196 Phase 2b) -> E-STOP substitution (#198 Phase 3) -> inline `DeviceBackend.WriteCommand` (actuator publish) + SPSC push (nrt-publish lane) + log. **E-STOP latch 가 서면 controller output 은 `BuildHoldOutput()` 으로 치환돼 backend 에 도달하지 않는다** — actuator 안전이 controller 의 E-STOP hook 구현에 의존하지 않게 하는 manager 측 방어선 (치환은 validation 을 우회하지 않고 그 뒤에 합성된다; 두 가드는 카운터를 따로 둔다)
- **CheckTimeouts** (50Hz): per-group device timeout -> `TriggerGlobalEstop("{group}_timeout")`
- **E-STOP triggers**: group timeout, init timeout, >= 10 consecutive RT overruns, sim sync timeout
- **TriggerGlobalEstop**: idempotent (`compare_exchange_strong`), propagates to all controllers

## Data Flow

```
[Robot HW / MuJoCo Sim] --JointState--> [rt_callback (FIFO 70)] --SeqLock--> [rt_control: RT loop @ control_rate]
    |                                          +--dirty bit + eventfd--> [nrt_publish_thread]
    +--inline--> backend.WriteCommand (actuator command, RT-safe)
    +--SPSC (cap 16)--> [nrt_publish_thread (CFS)] --> controller.PublishNonRtSnapshot
    |                                                  (Transforms / grasp_state / wbc_state / tof_snapshot)
    |                                              +-> /rtc_cm/{group}/joint_states (digital twin)
    +--SPSC--> [nrt_logging_executor (CFS -5)] --> CSV (timing + per-device state + sensor)
    +--E-STOP latch--> [nrt_logging (CFS -5)] --> /system/estop_status + RCLCPP log (deferred, RT-10)

[Hand HW] <--UDP--> [udp_hand_driver] <--SeqLock--> [ControlLoop]
[rtc_digital_twin]: merge /rtc_cm/{group}/joint_states --> RViz2
[ur5e_bt_coordinator]: subscribes grasp_state + /rtc_cm/<group>/joint_states + tf2 buffer fed by a `<config_key>/transforms` sub (self-feed — see TF invariant below), publishes goals; tunes gains via per-controller ROS 2 parameters
```

## RT vs non-RT Topic Ownership

토픽 소유는 3개 lane 으로 나뉜다 (issue #138: controller YAML 에는 `ownership:` field 가 없다 — controller-YAML entry 는 전부 controller-owned):

- **Controller-owned** (controller YAML `topics:` entry 전부) — Per-controller `LifecycleNode` (namespace `/<config_key>/`, `nrt_callback_executor` 에 add_node) 가 외부 facing snapshot 소유. Subscribe (role `target`, alias `goal` — `joint_goal`/`ee_pose` 는 role 이 아니라 토픽 이름이다), publish (transforms via PublishRole; grasp_state/wbc_state/tof_snapshot 는 controller-owned SeqLock + Setup*Publisher 헬퍼 — PublishRole 없음)
- **DeviceBackend-owned** — device-wire state/motor/sensor sub + joint/ros2 command pub, `devices.<group>.backend:` (sim.yaml/robot.yaml) 에서 선언
- **CM fixed publishers** — `RtControllerNode` 가 hardcode 로 소유 (YAML 무관): per-group digital-twin `/rtc_cm/<group>/joint_states`, safety pub (`/system/estop_status`, `/rtc_cm/active_controller_name` latched rewire trigger). 모두 lifecycle 무관 standalone publisher 로 active

RT loop 가 per-tick 으로 controller 의 SeqLock writer 에 push → non-RT `nrt_publish_thread` 가 read + ROS publish.

외부 도구 (BT, GUIs, digital_twin, shape_estimation) 는 `/rtc_cm/active_controller_name` (TRANSIENT_LOCAL) 구독 → switch 시 active controller 의 `/<config_key>/...` 토픽으로 rewire.

**TF `_actual` 프레임 — `/tf` publisher 없음 (framework invariant).** 컨트롤러는 arm-tip / fingertip `_actual` 프레임 (`base → tool0_actual`, `<link>_actual`) 을 `/tf` 로 발행하지 **않는다** — 오직 controller-owned `/<config_key>/transforms` (`tf2_msgs/TFMessage`, PublishRole) 로만 노출한다. 따라서 bare `tf2_ros::TransformListener` (`/tf`·`/tf_static` 만 청취) 는 이 프레임을 **절대 받지 못한다**. tf 소비자는 반드시 둘 중 하나: **(a) self-feed** — `/<config_key>/transforms` 를 직접 구독해 buffer 에 `setTransform` (ur5e_bt_coordinator `transforms_sub_`, demo_gui `_transforms_cb`; active controller 전환 시 rewire); **(b) `/tf` 재발행 의존** — `rtc_digital_twin` 의 `controller_tf` 재발행 (`<active>/transforms` → restamp → `/tf`, RViz TF 디스플레이·bare-listener 소비자용, default on). 이 함정은 digital_twin tcp_viz·bt_coordinator 두 곳에서 각각 silent-fail 버그로 발현했다 — **새 tf 소비자 추가 시 (a)/(b) 중 하나를 반드시 적용**하고, bare listener 만 두지 말 것.

구현: controller YAML `topics:` entry (`SubscribeTopicEntry` / `PublishTopicEntry`, `rtc_base/types/types.hpp`) 는 모두 controller-owned 이며 `integrated_bringup/src/support/owned_topics.cpp` 가 controller LifecycleNode 에 sub/pub 을 생성한다. CM 은 controller-YAML target sub 을 만들지 않는다 (manager-target 경로 폐기, issue #138); `nrt_publish_thread` (cap 16 SPSC drain, `nrt_callback` 와 동일 core, CFS) 가 `RTControllerInterface::PublishNonRtSnapshot(snap)` 로 controller-owned publisher 에 위임하고, 같은 루프에서 CM 소유 digital-twin republish (`DrainDigitalTwin()`) 도 드레인한다 — 후자는 RT tick 이 아니라 device state 콜백이 신호하므로 매 pass 무조건 확인한다. RT path 의 actuator 송출은 `rt_control` thread 가 rt_loop tick 안에서 `DeviceBackend.WriteCommand` 를 inline 호출 — long non-RT publish 가 actuator latency 를 막지 못하도록 두 lane 분리 유지.

Session logs: `logging_data/YYMMDD_HHMM/{timing,monitor,device,sim,plots,motions,tracing}/`. Per-controller logs 는 `controllers/<config_key>/` (controller LifecycleNode 가 owner, 예: `demo_wbc_controller/mpc_solve_timing.csv`), per-tick 스레드 타이밍 CSV 는 `timing/` (`cm_timing_log` / `mpc_timing_log` / `hand_udp_timing_log` / `rt_callback_timing_log`). 레거시 singular `controller/` (CM RT-loop DataLogger) 는 Phase C 에서 제거됨 — 더 이상 생성하지 않는다. 새 logger 추가 시 producer thread 의 소유자 기준으로 위치 선택. Session subdir 목록은 `rtc_base/logging/session_dir.hpp` (`kSubdirs`) + `rtc_tools.utils.session_dir` (`_SESSION_SUBDIRS`) 가 mirror SSoT — 한쪽 변경 시 반드시 동기화.

## Dependency Graph

**이 그래프는 ARCH-2 (상향 의존 금지) 판정에 필요한 층위 요약이다 — 전체 엣지의 SSoT 는 각 패키지의 `package.xml`** 이며 여기 전수 박제하지 않는다 (AP-DOC-1). 아래는 층위를 가르는 `rtc_*` 엣지와 외부 의존만 담는다.

```
rtc_msgs, rtc_base (independent)
  +-- rtc_communication, rtc_inference <-- rtc_base
  +-- rtc_controller_interface <-- rtc_base, rtc_msgs, rtc_urdf_bridge
  +-- rtc_controllers <-- rtc_base, rtc_msgs, rtc_math, rtc_urdf_bridge
  |     (sibling of rtc_controller_interface -- does NOT depend on it, #236 S7c)
  +-- rtc_controller_manager <-- rtc_controller_interface, rtc_controllers,
  |         rtc_base, rtc_msgs, rtc_communication, rtc_urdf_bridge
  +-- rtc_tsid <-- rtc_math, rtc_urdf_bridge, Pinocchio, ProxSuite, Eigen3, yaml-cpp
  +-- rtc_mpc  <-- rtc_base, Eigen3, yaml-cpp, Pinocchio
  |         (+ CMake-only: fmt >= 10, aligator -- source-installed, package.xml 미선언)
  +-- rtc_mujoco_sim <-- rtc_base, rtc_msgs, MuJoCo 3.x (optional)
rtc_math (independent) <-- Eigen3 (Pinocchio adapter optional)
rtc_urdf_bridge <-- Pinocchio, tinyxml2, yaml-cpp
udp_hand_driver <-- rtc_communication, rtc_inference, rtc_base
robot_descriptions (data-only, no code deps)
integrated_bringup <-- rtc_controller_manager, rtc_controller_interface, rtc_controllers,
                 rtc_tsid, rtc_mpc, rtc_base, rtc_msgs, rtc_math, rtc_urdf_bridge
                 + <exec_depend> udp_hand_driver, robot_descriptions, rtc_tools, repo_scripts
```
