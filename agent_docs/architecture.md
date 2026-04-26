# Architecture

## Core Data Types (`rtc_base/types/types.hpp`)

Key constants: `kNumRobotJoints=6`, `kMaxDeviceChannels=64`, `kMaxSensorChannels=128`, `kMaxFingertips=8`, `kNumHandMotors=10`.

Key types (all trivially copyable, RT-safe):
- **DeviceState**: positions/velocities/efforts[64], motor_*/sensor_data/inference_data arrays
- **ControllerState**: devices[4], num_devices, dt, iteration
- **ControllerOutput**: devices[4], task positions, valid, command_type, grasp_state, wbc_state
- **GraspStateData**: force_magnitude/contact_flag/inference_valid[8], grasp_phase, finger_s/filtered_force/force_error[8], grasp_target_force (Force-PI 데모 전용 — DemoJoint/DemoTask)
- **WbcStateData** (2026-04-26, `6c8d70a`): wbc_phase (8-state enum), per-fingertip force/contact/displacement[8], num_active_contacts, max_force, grasp_target_force, grasp_detected, min_fingertips, tsid_solve_us, tsid_solver_ok, qp_fail_count (TSID-기반 데모 전용 — DemoWbc). RT 루프가 `output.wbc_state` → `PublishSnapshot::GroupCommandSlot::wbc_state` 로 단일 struct 복사.

## Threading Model (6-core example)

| Thread | Core | Sched | Prio | Role |
|--------|------|-------|------|------|
| rt_loop | 2 | FIFO | 90 | 500Hz ControlLoop + 50Hz CheckTimeouts |
| sensor_executor | 3 | FIFO | 70 | JointState/MotorState/SensorState/Target subs |
| log_executor | 4 | OTHER | nice -5 | CSV logging (SPSC drain) |
| mpc_main | 4 | FIFO | 60 | 20Hz MPC solve, publishes via `TripleBuffer`. Concrete driver is `rtc::mpc::HandlerMPCThread` (Phase 6): per-tick FK → `PhaseManagerBase::Update` → `MPCHandlerBase::Solve` → `MPCSolutionManager::PublishSolution`; cross-mode swap via `MPCFactory::Create` + `SeedWarmStart`. `MockMPCThread` remains the no-solver baseline. |
| publish_thread | 5 | OTHER | nice -3 | SPSC -> ROS2 publish |
| udp_recv | 5 | FIFO | 65 | Hand UDP receiver |

Core 0-1: OS/DDS/IRQ. Auto-selects 4/6/8/10/12/16-core layouts via `SelectThreadConfigs()`.
MPC gets a dedicated core on 8+ core tiers (Core 4 on 8-core; Core 9 on 12/16-core with 1-2 workers).
MPC priority is always below `sensor_io` so sensor callbacks preempt long MPC solves.

Hybrid-CPU detection (Stage A, 2026-04-22): `rtc_base/threading/cpu_topology.hpp::GetCpuTopology()` returns a cached `CpuTopology` with `{num_p_physical, p_core_{physical,sibling}_ids, {e,lpe}_core_ids, generation ∈ {NOT_NUC_HYBRID, RAPTOR_LAKE_P, METEOR_LAKE, ARROW_LAKE_H, RAPTOR_LAKE_P_HT_OFF}}`. `SelectThreadConfigs()` still dispatches on `GetPhysicalCpuCount()` — the topology layer is dormant until Stage B consumes it. Shell mirror: `rt_common.sh::detect_hybrid_capability`; sysfs-root override via `$RTC_SYSFS_ROOT` for tests. See `docs/NUC_HYBRID_SUPPORT.md`.

MPC solve-timing observability: `RTControllerInterface::GetMpcSolveStats()` returns `std::optional<rtc::MpcSolveStats>` (default `nullopt`; `DemoWbcController` overrides). Each MPC-using controller's own LifecycleNode owns a 1 Hz aux timer (spawned in `on_activate` when `mpc_enabled_`), polls itself, and appends to `<session>/controllers/<config_key>/mpc_solve_timing.csv` (9 columns, `t_wall_ns,count,window,last_ns,min_ns,p50_ns,p99_ns,max_ns,mean_ns`), logs `p50/p99/max` every 10 s via `RCLCPP_INFO`. Writer lives in `rtc_mpc/logging/mpc_solve_timing_logger.hpp`. Underlying ring is `MPCSolutionManager::GetSolveStats()` (256-sample window ≈ 12 s @ 20 Hz).

## Lock-Free Rules

- **SeqLock<T>**: single-writer/multi-reader, requires `is_trivially_copyable_v<T>`
- **SpscLogBuffer/SpscPublishBuffer<512>**: wait-free push (drops on full), power-of-2
- **try_lock only** on RT path (never block), `lock_guard` on non-RT `SetDeviceTarget()`
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

**Safety publishers** (`estop_pub_`, `active_ctrl_name_pub_`, `current_gains_pub_`) use standalone `rclcpp::create_publisher` -- active regardless of lifecycle state.

**RtControllerMain** uses a 3-phase executor: (1) lifecycle_executor spins for configure/activate, (2) polls until Active, (3) switches to sensor/log/aux dedicated executors.

- **ControlLoop** (500Hz): E-STOP check -> assemble ControllerState -> `Compute()` -> SPSC publish + log
- **CheckTimeouts** (50Hz): per-group device timeout -> `TriggerGlobalEstop("{group}_timeout")`
- **E-STOP triggers**: group timeout, init timeout, >= 10 consecutive RT overruns, sim sync timeout
- **TriggerGlobalEstop**: idempotent (`compare_exchange_strong`), propagates to all controllers

## Data Flow

```
[Robot HW / MuJoCo Sim] --JointState--> [RtControllerNode: 500Hz RT loop]
    +--SPSC--> [publish_thread] --> /forward_position_controller/commands
    |                           +-> /{group}/digital_twin/joint_states
    +--SPSC--> [log_executor]   --> CSV (timing + per-device state + sensor)
    +--E-STOP--> /system/estop_status

[Hand HW] <--UDP--> [ur5e_hand_driver] <--SeqLock--> [ControlLoop]
[rtc_digital_twin]: merge digital_twin topics --> RViz2
[ur5e_bt_coordinator]: subscribes grasp_state/gui_position, publishes goals/gains
```

## RT vs non-RT Topic Ownership

YAML `ownership:` field (per `<topic>` entry in controller config) drives a 2-tier split. CM (`RtControllerNode`) owns RT-adjacent traffic; per-controller `LifecycleNode` owns external-facing snapshot traffic.

```
                            ┌────────────────────────────────────────────────┐
                            │       RtControllerNode (CM, exec process)      │
                            │  ┌────────────────────────────────────────┐    │
ownership: "manager"  ──►   │  │ RT loop (500 Hz, SCHED_FIFO)           │    │
(default)                   │  │   sub: state / motor_state / sensor    │    │
                            │  │   pub: ros2_command, joint_command,    │    │
                            │  │        device_state_log, sensor_log,   │    │
                            │  │        digital_twin/joint_states       │    │
                            │  │   safety pubs (standalone, non-life-   │    │
                            │  │     cycle): /system/estop_status,      │    │
                            │  │     /active_controller_name (latched), │    │
                            │  │     /current_gains                     │    │
                            │  └────────────────────────────────────────┘    │
                            │                                                │
                            │  per-controller LifecycleNode  (aux executor)  │
                            │  ┌────────────────────────────────────────┐    │
ownership: "controller"     │  │ namespace = /<config_key>/             │    │
                       ──►  │  │   sub: target  (joint_goal, ee_pose)   │    │
                            │  │   pub: gui_position, grasp_state,      │    │
                            │  │        wbc_state, tof_snapshot         │    │
                            │  │ RT loop never touches this node — all  │    │
                            │  │ data flows via SPSC PublishSnapshot →  │    │
                            │  │ publish_thread → Node->publish()       │    │
                            │  └────────────────────────────────────────┘    │
                            └────────────────────────────────────────────────┘

External tools (BT, GUIs, digital_twin, shape_estimation) sub
  /active_controller_name (TRANSIENT_LOCAL, CM-owned) → rewire to
  the active controller's /<config_key>/... topics on switch.
```

Implementation: `rtc::TopicOwnership` enum (`kManager` | `kController`) read by `rtc_controllers/topic_config.hpp`. CM skips controller-owned sub/pub during configure. Publish thread routes via `RTControllerInterface::PublishNonRtSnapshot(snap)` → controller-owned `LifecyclePublisher` instances.

Session logs: `logging_data/YYMMDD_HHMM/{controller,monitor,device,sim,plots,motions}/`. Per-controller logs live under `controllers/<config_key>/` (plural), CM RT loop logs under `controller/` (singular). See `feedback_session_log_dir_convention` memory for the singular vs plural distinction.

Session/logging root resolution (4-tier chain, shared between `rtc_base/logging/session_dir.hpp` and `rtc_tools.utils.session_dir`):
1. `$RTC_SESSION_DIR` -> `$UR5E_SESSION_DIR` (legacy fallback) -- used as-is if set.
2. `$COLCON_PREFIX_PATH` first entry's parent + `/logging_data` (requires write access).
3. Walk up from `cwd` looking for `install/` + `src/` siblings -> that dir + `/logging_data`.
4. Final fallback: `$PWD/logging_data`.

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
ur5e_hand_driver <-- rtc_communication, rtc_inference, rtc_base
ur5e_bringup <-- rtc_controller_manager, ur5e_hand_driver, ur5e_description,
                 rtc_tsid, rtc_mpc
```
