# Architecture

## Core Data Types (`rtc_base/types/types.hpp`)

Key constants: `kNumRobotJoints=6`, `kMaxDeviceChannels=64`, `kMaxSensorChannels=128`, `kMaxFingertips=8`, `kNumHandMotors=10`.

Key types (all trivially copyable, RT-safe):
- **DeviceState**: positions/velocities/efforts[64], motor_*/sensor_data/inference_data arrays
- **ControllerState**: devices[4], num_devices, dt, iteration
- **ControllerOutput**: devices[4], task positions, valid, command_type, grasp_state
- **GraspStateData**: force_magnitude/contact_flag/inference_valid[8], grasp_phase, finger_s/filtered_force/force_error[8], grasp_target_force

## Threading Model (6-core example)

| Thread | Core | Sched | Prio | Role |
|--------|------|-------|------|------|
| rt_loop | 2 | FIFO | 90 | 500Hz ControlLoop + 50Hz CheckTimeouts |
| sensor_executor | 3 | FIFO | 70 | JointState/MotorState/SensorState/Target subs |
| log_executor | 4 | OTHER | nice -5 | CSV logging (SPSC drain) |
| mpc_main | 4 | FIFO | 60 | 20Hz MPC solve, publishes via `TripleBuffer` |
| publish_thread | 5 | OTHER | nice -3 | SPSC -> ROS2 publish |
| udp_recv | 5 | FIFO | 65 | Hand UDP receiver |

Core 0-1: OS/DDS/IRQ. Auto-selects 4/6/8/10/12/16-core layouts via `SelectThreadConfigs()`.
MPC gets a dedicated core on 8+ core tiers (Core 4 on 8-core; Core 9 on 12/16-core with 1-2 workers).
MPC priority is always below `sensor_io` so sensor callbacks preempt long MPC solves.

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

Session logs: `logging_data/YYMMDD_HHMM/{controller,monitor,device,sim,plots,motions}/`

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
