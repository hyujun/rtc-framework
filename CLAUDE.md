# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

**Prerequisites**: Ubuntu 22.04 (ROS2 Humble) or 24.04 (ROS2 Jazzy), `realtime` group with `rtprio 99` / `memlock unlimited`.

```bash
# Build
./build.sh sim            # simulation packages
./build.sh robot          # real robot (excludes MuJoCo)
./build.sh full           # all packages
./build.sh -p rtc_base    # single package

# Run
ros2 launch ur5e_bringup sim.launch.py                                        # MuJoCo sim
ros2 launch ur5e_bringup robot.launch.py robot_ip:=192.168.1.10              # real robot
ros2 launch ur5e_hand_driver hand_udp.launch.py target_ip:=192.168.1.2      # hand only

# Monitor
ros2 topic hz /forward_position_controller/commands   # ~500Hz
ros2 topic echo /system/estop_status                  # true = E-STOP active
```

---

## Repository Structure

18 ROS2 packages at repo root (no `src/`). 12 `rtc_*` (robot-agnostic) + 2 `shape_estimation_*` + 4 `ur5e_*` (robot-specific). Each has its own `README.md` with detailed API and configuration.

| Package | Type | Key Content |
|---------|------|-------------|
| `rtc_base` | Header-only | Types, SeqLock, SPSC buffers, threading (4/6/8/10/12/16-core), Bessel/Kalman filters, DataLogger |
| `rtc_communication` | Header-only | `TransportInterface`, `UdpSocket` RAII, `PacketCodec` concept, `Transceiver<T,C>` |
| `rtc_controller_interface` | Library | `RTControllerInterface` abstract base, `ControllerRegistry` singleton, `RTC_REGISTER_CONTROLLER` macro |
| `rtc_controllers` | Library | PController, JointPDController, ClikController, OSC, GraspController (adaptive PI force) |
| `rtc_controller_manager` | Executable | `RtControllerNode`: 500Hz RT loop, SPSC publish offload, CSV logging, E-STOP, digital twin |
| `rtc_inference` | Header-only | `InferenceEngine` abstract, `OnnxEngine` (IoBinding, pre-allocated buffers) |
| `rtc_msgs` | Messages | JointCommand, HandSensorState, GraspState, RobotTarget, SimSensor/SimSensorState, etc. |
| `rtc_mujoco_sim` | Executable | MuJoCo 3.x wrapper: sync-step, GLFW viewer, multi-group, position servo |
| `rtc_tools` | Python | controller_gui, plot_rtc_log, compare_mjcf_urdf, urdf_to_mjcf |
| `rtc_scripts` | Shell | PREEMPT_RT build, CPU shield, IRQ affinity, UDP optimization |
| `rtc_digital_twin` | Python | RViz2 multi-source JointState merge, URDF mimic auto-compute |
| `rtc_urdf_bridge` | Library | Robot-agnostic URDF parser + Pinocchio model builder |
| `shape_estimation` | Executable | ToF-based voxel point cloud + primitive fitting |
| `ur5e_description` | Data | URDF + MJCF + meshes (DAE/STL/OBJ) |
| `ur5e_hand_driver` | Executable | UDP event-driven driver, SeqLock state, ONNX F/T inference |
| `ur5e_bt_coordinator` | Executable | BehaviorTree.CPP v4 task coordinator (20 Hz) |
| `ur5e_bringup` | Launch/Config | Launch files + DemoJoint/DemoTask controllers + GUI tools |

### Dependency Graph

```
rtc_msgs, rtc_base (independent)
  +-- rtc_communication, rtc_inference <-- rtc_base
  +-- rtc_controller_interface <-- rtc_base, rtc_msgs, rtc_urdf_bridge
  |     +-- rtc_controllers <-- rtc_controller_interface, rtc_urdf_bridge
  |           +-- rtc_controller_manager <-- rtc_controllers, rtc_communication
  +-- rtc_mujoco_sim <-- MuJoCo 3.x (optional)
rtc_urdf_bridge <-- Pinocchio, tinyxml2, yaml-cpp
ur5e_hand_driver <-- rtc_communication, rtc_inference, rtc_base
ur5e_bringup <-- rtc_controller_manager, ur5e_hand_driver, ur5e_description
```

---

## Architecture

### Core Data Types (`rtc_base/types/types.hpp`)

Key constants: `kNumRobotJoints=6`, `kMaxDeviceChannels=64`, `kMaxSensorChannels=128`, `kMaxFingertips=8`, `kNumHandMotors=10`.

Key types (all trivially copyable, RT-safe):
- **DeviceState**: positions/velocities/efforts[64], motor_*/sensor_data/inference_data arrays
- **ControllerState**: devices[4], num_devices, dt, iteration
- **ControllerOutput**: devices[4], task positions, valid, command_type, grasp_state
- **GraspStateData**: force_magnitude/contact_flag/inference_valid[8], grasp_phase, finger_s/filtered_force/force_error[8], grasp_target_force

### Threading Model (6-core example)

| Thread | Core | Sched | Prio | Role |
|--------|------|-------|------|------|
| rt_loop | 2 | FIFO | 90 | 500Hz ControlLoop + 50Hz CheckTimeouts |
| sensor_executor | 3 | FIFO | 70 | JointState/MotorState/SensorState/Target subs |
| log_executor | 4 | OTHER | nice -5 | CSV logging (SPSC drain) |
| publish_thread | 5 | OTHER | nice -3 | SPSC -> ROS2 publish |
| udp_recv | 5 | FIFO | 65 | Hand UDP receiver |

Core 0-1: OS/DDS/IRQ. Auto-selects 4/6/8/10/12/16-core layouts via `SelectThreadConfigs()`.

### Lock-Free Rules

- **SeqLock<T>**: single-writer/multi-reader, requires `is_trivially_copyable_v<T>`
- **SpscLogBuffer/SpscPublishBuffer<512>**: wait-free push (drops on full), power-of-2
- **try_lock only** on RT path (never block), `lock_guard` on non-RT `SetDeviceTarget()`
- **jthread + stop_token** for cooperative cancellation
- **Separate mutexes**: `state_mutex_`, `target_mutex_`, `hand_mutex_` -- never hold more than one

---

## Controllers

| Controller | Type | Space | Key Feature |
|------------|------|-------|-------------|
| PController | Position | Joint | `q + kp*error*dt` incremental |
| JointPDController | Torque | Joint | PD + Pinocchio RNEA + quintic trajectory |
| ClikController | Position | Cartesian 3/6-DOF | Damped Jacobian pseudoinverse + null-space |
| OSC | Torque | Cartesian 6-DOF | Full pose PD + SE3 quintic trajectory |
| GraspController | Internal | Hand 3x3-DOF | Adaptive PI force, 6-state FSM, per-finger stiffness EMA |
| DemoJointController | Position | Joint + Hand | Quintic trajectory, `grasp_controller_type: "contact_stop"\|"force_pi"` |
| DemoTaskController | Position | Cartesian + Hand | CLIK + trajectory, `grasp_controller_type: "contact_stop"\|"force_pi"` |

### Gains Layout (via `~/controller_gains` topic)

| Controller | Layout | Count |
|------------|--------|-------|
| PController | `[kp x 6]` | 6 |
| JointPDController | `[kp x 6, kd x 6, gravity(0/1), coriolis(0/1), traj_speed]` | 15 |
| ClikController | `[kp_trans x 3, kp_rot x 3, damping, null_kp, null_space(0/1), 6dof(0/1), traj_speed, traj_ang_speed, max_vel, max_ang_vel]` | 14 |
| OSC | `[kp_pos x 3, kd_pos x 3, kp_rot x 3, kd_rot x 3, damping, gravity(0/1), traj_speed, traj_ang_speed, max_vel, max_ang_vel]` | 18 |
| DemoJoint | `[robot_traj_speed, hand_traj_speed, robot_max_vel, hand_max_vel, grasp_contact_thresh, grasp_force_thresh, grasp_min_fingertips, grasp_cmd(0/1/2), grasp_target_force]` | 9 |
| DemoTask | `[kp_trans x 3, kp_rot x 3, damping, null_kp, null_space(0/1), 6dof(0/1), traj_speed, traj_ang_speed, hand_traj_speed, max_vel, max_ang_vel, hand_max_vel, grasp_contact_thresh, grasp_force_thresh, grasp_min_fingertips, grasp_cmd(0/1/2), grasp_target_force]` | 21 |

### GraspController (Force-PI, internal only)

Selected via `grasp_controller_type: "force_pi"` in demo controller YAML (default: `"contact_stop"`).

**FSM**: Idle -> Approaching (position ramp) -> Contact (settle) -> ForceControl (PI + force ramp) -> Holding (anomaly monitor) -> Releasing

Key params in `grasp_types.hpp`: `Kp_base=0.02`, `Ki_base=0.002`, `f_target=2.0N`, `f_contact=0.2N`, `ds_max=0.05/s`, `delta_s_max=0.15` deformation guard, slip detection at `5.0 N/s`.

---

## RtControllerNode

- **ControlLoop** (500Hz): E-STOP check -> assemble ControllerState -> `Compute()` -> SPSC publish + log
- **CheckTimeouts** (50Hz): per-group device timeout -> `TriggerGlobalEstop("{group}_timeout")`
- **E-STOP triggers**: group timeout, init timeout, >= 10 consecutive RT overruns, sim sync timeout
- **TriggerGlobalEstop**: idempotent (`compare_exchange_strong`), propagates to all controllers

---

## ROS2 Topics (Key)

**Controller Manager** (`/{ns}/`): `controller_type` (Sub, switch), `controller_gains` (Sub), `active_controller_name` (Pub), `/system/estop_status` (Pub)

**Dynamic** (per controller TopicConfig): Subscribe `kState`/`kMotorState`/`kSensorState`/`kTarget`; Publish `kJointCommand`/`kRos2Command`/`kGuiPosition`/`kGraspState`/`kDeviceStateLog`/`kDeviceSensorLog`

**Digital Twin**: `/{group}/digital_twin/joint_states` (RELIABLE) -> `rtc_digital_twin` merges -> RViz2

**Hand Driver**: `/hand/joint_states`, `/hand/motor_states`, `/hand/sensor_states` (Pub); `/hand/joint_command` (Sub)

**MuJoCo**: `<group.state_topic>` (Pub), `<group.command_topic>` (Sub), `/sim/status` (1Hz)

---

## Configuration Files

Key config files (see each file for full parameter documentation):

| Config | Path | Key Parameters |
|--------|------|----------------|
| RT controller manager | `ur5e_bringup/config/ur5e_robot.yaml` / `ur5e_sim.yaml` | `control_rate`, `initial_controller`, `devices`, `urdf`, `device_timeout_*` |
| MuJoCo simulator | `rtc_mujoco_sim/config/mujoco_simulator.yaml` | `model_path`, `physics_timestep`, `n_substeps`, `robot_response.groups` |
| MuJoCo solver | `rtc_mujoco_sim/config/solver_param.yaml` | `solver` (Newton/CG/PGS), `cone`, `integrator`, `noslip_iterations`, `contact_override` |
| Hand UDP driver | `ur5e_hand_driver/config/hand_udp_node.yaml` | `target_ip`, `recv_timeout_ms`, `communication_mode` (bulk/individual) |
| Digital twin | `rtc_digital_twin/config/digital_twin.yaml` | `display_rate`, `num_sources`, `auto_compute_mimic` |
| Controller YAMLs | `rtc_controllers/config/controllers/{direct\|indirect}/*.yaml` | Per-controller gains + `topics:` section for device-group routing |

---

## Common Modification Patterns

### Adding a New Controller

1. Header in `rtc_controllers/include/rtc_controllers/{direct|indirect}/` -- inherit `RTControllerInterface`, implement `Compute()`, `SetDeviceTarget()`, `InitializeHoldPosition()`, `Name()` (all `noexcept`)
2. Source in `rtc_controllers/src/controllers/{direct|indirect}/` -- `LoadConfig()` for YAML, `UpdateGainsFromMsg()` for runtime gains
3. YAML in `rtc_controllers/config/controllers/` -- must include `topics:` section
4. Register via `RTC_REGISTER_CONTROLLER()` macro. Robot-specific controllers go in `ur5e_bringup/` with registration in `ur5e_bringup/src/controllers/controller_registration.cpp`

### Adding a New Message Type

1. Create `rtc_msgs/msg/MyMessage.msg`, add to `CMakeLists.txt` `rosidl_generate_interfaces()`
2. If used in publish offload: add `PublishRole` enum in `rtc_base/types/types.hpp` + YAML mapping in `rtc_controller_interface/src/rt_controller_interface.cpp`

### Adding a New Device Group

1. Add device config in YAML under `devices:` + timeout entry in `device_timeout_names`/`values`
2. Add topic routing in each controller's YAML `topics:` section
3. If kinematics needed: add `sub_models` or `tree_models` entry under `urdf:`
4. Handle new device index in controller `Compute()` / `SetDeviceTarget()`

### Adding a New Thread

1. Define `ThreadConfig` for all core tiers in `rtc_base/threading/thread_config.hpp`
2. Add to `SystemThreadConfigs`, update `ValidateSystemThreadConfigs()` + `SelectThreadConfigs()`
3. Call `ApplyThreadConfig()` at thread entry; use SCHED_FIFO for RT threads

---

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

Session logs: `logging_data/YYMMDD_HHMM/{controller,monitor,hand,sim,plots,motions}/`
Session dir env: `RTC_SESSION_DIR` (or `UR5E_SESSION_DIR` fallback).

---

## Code Conventions

- **Namespace**: `rtc` (all packages)
- **Naming**: Google C++ Style -- `snake_case` members with trailing `_`
- **`noexcept`** on all RT paths (exceptions = process termination, intentional)
- **C++20**: jthread, stop_token, span, string_view, concepts, `[[likely]]/[[unlikely]]`, constexpr
- **`[[nodiscard]]`** on status-returning functions; **`static_assert`** on template params
- **Include order**: project -> ROS2/third-party -> C++ stdlib
- **Eigen**: pre-allocated buffers, `noalias()`, zero heap on 500Hz path
- **Compiler flags**: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`
- **ROS2 Node**: Prefer `rclcpp_lifecycle::LifecycleNode` over `rclcpp::Node` for managed state transitions (unconfigured → inactive → active → finalized)

---

## RT Permissions

```bash
sudo groupadd realtime && sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# Re-login required. Optional: isolcpus, nohz_full, or cpu_shield.sh
```

---

## Debugging

| Symptom | Fix |
|---------|-----|
| `ApplyThreadConfig()` warns | `sudo usermod -aG realtime $USER` + re-login |
| E-STOP on startup | Set `init_timeout_sec: 0.0` for sim |
| High jitter (>200us) | Check `taskset` pinning, verify `isolcpus` |
| Hand timeout E-STOP | Check UDP link, `recv_timeout_ms: 0.4` |
| Controller not found | Use config_key (e.g. "p_controller") or Name() |

```bash
PID=$(pgrep -f rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
ros2 topic hz /forward_position_controller/commands
ros2 topic echo /system/estop_status
./rtc_scripts/scripts/check_rt_setup.sh --summary
```

---

## Testing

```bash
# All tests
colcon test --event-handlers console_direct+
colcon test-result --verbose

# Single package
colcon test --packages-select ur5e_bt_coordinator --event-handlers console_direct+

# Single test (C++)
colcon test --packages-select rtc_controllers --ctest-args -R test_grasp_controller

# Single test (Python)
colcon test --packages-select rtc_digital_twin --pytest-args -k test_urdf_parser
```

Test files by package (30 total):

| Package | Tests | Framework |
|---------|-------|-----------|
| `ur5e_bt_coordinator` | 14 C++ tests (`test/test_*.cpp`) | GTest |
| `rtc_controllers` | 6 C++ tests (trajectory + grasp) | GTest |
| `rtc_urdf_bridge` | 5 C++ tests (URDF/model parsing) | GTest |
| `shape_estimation` | 3 C++ tests (ToF + exploration) | GTest |
| `rtc_digital_twin` | 1 Python test | pytest |
| `rtc_tools` | 1 Python test | pytest |

---

## Key File Locations

| What | Path |
|------|------|
| Core types & constants | `rtc_base/include/rtc_base/types/types.hpp` |
| Thread configs | `rtc_base/include/rtc_base/threading/thread_config.hpp` |
| Controller abstract base | `rtc_controller_interface/include/rtc_controller_interface/rt_controller_interface.hpp` |
| Controller registry | `rtc_controller_interface/include/rtc_controller_interface/controller_registry.hpp` |
| RT control loop | `rtc_controller_manager/src/rt_controller_node.cpp` |
| Controller registration | `rtc_controllers/src/controller_registration.cpp` |
| Demo controller registration | `ur5e_bringup/src/controllers/controller_registration.cpp` |
| Grasp controller | `rtc_controllers/include/rtc_controllers/grasp/grasp_controller.hpp` |
| Grasp types/params | `rtc_controllers/include/rtc_controllers/grasp/grasp_types.hpp` |
| Main YAML (robot/sim) | `ur5e_bringup/config/ur5e_robot.yaml` / `ur5e_sim.yaml` |
| MuJoCo configs | `rtc_mujoco_sim/config/mujoco_simulator.yaml`, `solver_param.yaml` |
| Hand driver config | `ur5e_hand_driver/config/hand_udp_node.yaml` |
| UR5e URDF / MJCF | `ur5e_description/robots/ur5e/{urdf,mjcf}/` |
| BT trees + launch | `ur5e_bt_coordinator/trees/*.xml`, `launch/bt_coordinator.launch.py` |
| Arm poses | `ur5e_bt_coordinator/config/poses.yaml` |
| CycloneDDS config | `rtc_controller_manager/config/cyclone_dds.xml` |
| Supplementary docs | `docs/` (RT_OPTIMIZATION.md, SHELL_SCRIPTS.md, VSCODE_DEBUGGING.md) |
