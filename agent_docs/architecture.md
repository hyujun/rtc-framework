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

Thread roster·core·priority 의 SSoT 는 `rtc_base/threading/thread_config.hpp` (`SystemThreadConfigs` 정의 + `SelectThreadConfigs()` core-tier 분기). 4/6/8/10/12/16-core 레이아웃을 자동 선택. 문서엔 *불변 원칙*만 박는다:

- **Core 0-1 reserved** for OS/DDS/IRQ across all tiers
- **RT loop**: SCHED_FIFO, 최고 priority, dedicated core
- **Sensor executor**: SCHED_FIFO, RT loop 보다 낮음, 항상 MPC 보다 높음 (sensor callback 이 long MPC solve 를 선점할 수 있어야 함)
- **MPC main**: 8+ core tier 에서 dedicated core, priority < sensor_io
- **Log / publish thread**: SCHED_OTHER, nice 음수

세부 thread 종류·core 번호·priority 값은 위 header + `cpu_topology.hpp` 참조. Hybrid-CPU detection 은 `docs/NUC_HYBRID_SUPPORT.md`.

### Per-thread timing CSV infrastructure

CM RT loop · MPC thread · hand UDP receiver 가 *동일* generic transport + *동일* `RtTickTimingPayload` 를 공유한다. Fixed-frequency loop·lifecycle·`clock_nanosleep(TIMER_ABSTIME)` cadence·overrun detection·per-tick t0~t3 capture 는 모두 `rtc::PeriodicRtThread` base 에 박혀 있고, channel 은 hook override 만 추가한다 (sim-CV wakeup, E-STOP escalation). 새 per-tick timing channel 추가 시 같은 base + payload alias 재사용 — `RTControllerInterface` virtual 추가 / 새 SPSC class / 새 logger class 금지.

SSoT 파일:
- `rtc_base/threading/periodic_rt_thread.hpp` — loop/lifecycle base
- `rtc_base/timing/rt_tick_timing_sample.hpp` — unified `RtTickTimingPayload`
- `rtc_base/timing/thread_timing_{sample,producer,csv_logger}.hpp` — generic transport

CSV consumer / drop counter / 출력 경로는 channel 별로 다르고 (`cm_timing_log.csv` / `mpc_timing_log.csv` / `hand_udp_timing_log.csv`), `<session>/timing/` 아래 저장. Aggregate stats 는 INFO summary 만, percentile 은 post-process.

## Lock-Free Rules

- **SeqLock<T>**: single-writer/multi-reader, requires `is_trivially_copyable_v<T>`
- **SpscQueue<T,N> / SpscPublishBuffer<512>**: wait-free push (drops on full), power-of-2 (controller data CSVs use `ThreadCsvProducer<Pod, N>` which wraps `SpscQueue` — Phase C)
- **try_lock only** on RT path (never block); `lock_guard` 는 lifecycle 콜백 / aux thread / 파라미터 콜백 등 non-RT 경로에서만
- **jthread + stop_token** for cooperative cancellation
- **Separate mutexes**: `state_mutex_`, `target_mutex_`, `hand_mutex_` -- never hold more than one

## RtControllerNode

`RtControllerNode` inherits from `rclcpp_lifecycle::LifecycleNode`. The constructor is empty; all initialization happens in lifecycle callbacks.

| Callback | Tier | Resources |
|----------|------|-----------|
| `on_configure` | 1 | Callback groups, parameters, controllers, publishers/subscribers, timers, eventfd |
| `on_activate` | 2 | `SelectThreadConfigs()` -> `StartRtLoop()` + `StartPublishLoop()` |
| `on_deactivate` | -- | Stop RT/publish threads, clear E-STOP, reset init state |
| `on_cleanup` | -- | Reverse of `on_configure` (all `.reset()` / `.clear()`) |
| `on_error` | -- | `TriggerGlobalEstop("lifecycle_error")`, stop threads, full cleanup -> SUCCESS |

**Safety publishers** (`estop_pub_`, `active_ctrl_name_pub_`) use standalone `rclcpp::create_publisher` -- active regardless of lifecycle state.

**RtControllerMain** uses a 3-phase executor: (1) lifecycle_executor spins for configure/activate, (2) polls until Active, (3) switches to sensor/log/aux dedicated executors.

- **ControlLoop** (configurable rate, default 500 Hz): E-STOP check -> assemble ControllerState -> `Compute()` -> SPSC publish + log
- **CheckTimeouts** (50Hz): per-group device timeout -> `TriggerGlobalEstop("{group}_timeout")`
- **E-STOP triggers**: group timeout, init timeout, >= 10 consecutive RT overruns, sim sync timeout
- **TriggerGlobalEstop**: idempotent (`compare_exchange_strong`), propagates to all controllers

## Data Flow

```
[Robot HW / MuJoCo Sim] --JointState--> [RtControllerNode: RT loop @ control_rate]
    +--SPSC--> [publish_thread] --> /forward_position_controller/commands
    |                           +-> /rtc_cm/{group}/joint_states
    +--SPSC--> [log_executor]   --> CSV (timing + per-device state + sensor)
    +--E-STOP--> /system/estop_status

[Hand HW] <--UDP--> [udp_hand_driver] <--SeqLock--> [ControlLoop]
[rtc_digital_twin]: merge /rtc_cm/{group}/joint_states --> RViz2
[ur5e_bt_coordinator]: subscribes grasp_state + /rtc_cm/<group>/joint_states + tf2 listener (`<config_key>/transforms`), publishes goals; tunes gains via per-controller ROS 2 parameters
```

## RT vs non-RT Topic Ownership

YAML `ownership:` field (per `<topic>` entry in controller config) drives 2-tier split:

- **Manager-owned** (default, `ownership: manager`) — RT-adjacent traffic 가 `RtControllerNode` (CM, exec process) 에서. RT loop sub (state/motor/sensor), RT loop pub (commands / per-group joint_states / device logs), safety pub (`/system/estop_status`, `/rtc_cm/active_controller_name` latched rewire trigger) 모두 lifecycle 무관 standalone publisher 로 active
- **Controller-owned** (`ownership: controller`) — Per-controller `LifecycleNode` (namespace `/<config_key>/`, aux executor) 가 외부 facing snapshot 소유. Subscribe (target/joint_goal/ee_pose), publish (transforms via PublishRole; grasp_state/wbc_state/tof_snapshot 는 controller-owned SeqLock + Setup*Publisher 헬퍼 — PublishRole 없음)

RT loop 가 per-tick 으로 controller 의 SeqLock writer 에 push → non-RT aux thread 가 read + ROS publish.

외부 도구 (BT, GUIs, digital_twin, shape_estimation) 는 `/rtc_cm/active_controller_name` (TRANSIENT_LOCAL) 구독 → switch 시 active controller 의 `/<config_key>/...` 토픽으로 rewire.

구현: `rtc::TopicOwnership` enum (`rtc_controllers/topic_config.hpp`). CM 은 controller-owned sub/pub 을 configure 시 skip; publish thread 는 `RTControllerInterface::PublishNonRtSnapshot(snap)` 로 controller-owned publisher 에 위임.

Session logs: `logging_data/YYMMDD_HHMM/{controller,monitor,device,sim,plots,motions}/`. Per-controller logs 는 `controllers/<config_key>/` (plural), CM RT loop logs 는 `controller/` (singular). See `feedback_session_log_dir_convention` memory. Session root resolution (4-tier chain) 는 `rtc_base/logging/session_dir.hpp` + `rtc_tools.utils.session_dir` 가 SSoT.

## Dependency Graph

```
rtc_msgs, rtc_base (independent)
  +-- rtc_communication, rtc_inference <-- rtc_base
  +-- rtc_controller_interface <-- rtc_base, rtc_msgs, rtc_urdf_bridge
  |     +-- rtc_controllers <-- rtc_controller_interface, rtc_urdf_bridge
  |           +-- rtc_controller_manager <-- rtc_controllers, rtc_communication
  +-- rtc_tsid <-- Pinocchio, ProxSuite, Eigen3, yaml-cpp
  +-- rtc_mpc  <-- rtc_base, Eigen3, yaml-cpp, Pinocchio, fmt ≥ 10
  +-- rtc_mujoco_sim <-- MuJoCo 3.x (optional)
rtc_urdf_bridge <-- Pinocchio, tinyxml2, yaml-cpp
udp_hand_driver <-- rtc_communication, rtc_inference, rtc_base
robot_descriptions (data-only, no code deps)
integrated_bringup <-- rtc_controller_manager, udp_hand_driver, robot_descriptions,
                 rtc_tsid, rtc_mpc
```
