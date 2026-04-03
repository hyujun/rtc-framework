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
ros2 launch ur5e_bringup robot.launch.py use_mock_hardware:=true             # mock HW (Jazzy)
ros2 launch ur5e_bringup robot.launch.py use_fake_hardware:=true             # fake HW (Humble)
ros2 launch ur5e_hand_driver hand_udp.launch.py target_ip:=192.168.1.2      # hand only

# Monitor
ros2 topic hz /forward_position_controller/commands   # ~500Hz
ros2 topic echo /system/estop_status                  # true = E-STOP active
PID=$(pgrep -f rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
```

---

## Repository Structure

18 ROS2 packages at repo root (no `src/`). 12 `rtc_*` (robot-agnostic) + 2 shape estimation (`shape_estimation_*`) + 4 `ur5e_*` (robot-specific). Each has its own `README.md` with detailed API and configuration.

### Package Summary

| Package | Type | Key Content |
|---------|------|-------------|
| `rtc_base` | Header-only | Types (`ControllerState`, `DeviceState`, `ControllerOutput`), SeqLock, SPSC buffers, threading (4/6/8/10/12/16-core), Bessel/Kalman filters, DataLogger, session_dir |
| `rtc_communication` | Header-only | `TransportInterface` (abstract), `UdpSocket` RAII, `PacketCodec` concept, `Transceiver<T,C>` template |
| `rtc_controller_interface` | Library | `RTControllerInterface` abstract base (Compute/SetDeviceTarget/InitializeHoldPosition/Name -- all noexcept), `ControllerRegistry` singleton, `RTC_REGISTER_CONTROLLER` macro |
| `rtc_controllers` | Library | PController, JointPDController (Pinocchio RNEA), ClikController (Jacobian IK), OSC (6-DOF Cartesian PD) + quintic trajectory |
| `rtc_controller_manager` | Executable | `RtControllerNode`: clock_nanosleep RT loop, SPSC publish offload, CSV logging, global E-STOP, controller lifecycle, digital twin auto-republish |
| `rtc_inference` | Header-only | `InferenceEngine` abstract, `OnnxEngine` (IoBinding, pre-allocated buffers), `RunModels()` batch helper |
| `rtc_msgs` | Messages | 8 types: JointCommand, FingertipSensor, HandSensorState, GraspState, GuiPosition, RobotTarget, DeviceStateLog, DeviceSensorLog |
| `rtc_mujoco_sim` | Executable | MuJoCo 3.x wrapper: sync-step loop, GLFW viewer (40+ shortcuts), multi-group architecture (robot_response + fake_response), position servo gains |
| `rtc_tools` | Python | controller_gui, plot_rtc_log, compare_mjcf_urdf, urdf_to_mjcf, hand_udp_sender, hand_data_plot, session_dir |
| `rtc_scripts` | Shell | PREEMPT_RT kernel build, CPU shield (cset), IRQ affinity, UDP optimization, NVIDIA RT coexistence |
| `rtc_digital_twin` | Python | RViz2 visualization (multi-source JointState merge, URDF mimic auto-compute, fingertip sensor MarkerArray) |
| `rtc_urdf_bridge` | Library | Robot-agnostic URDF parser + Pinocchio model builder, YAML-based chain extraction config |
| `shape_estimation_msgs` | Messages | 4 types: ToFReadings, TipPoses, ToFSnapshot, ShapeEstimate |
| `shape_estimation` | Executable | ToF-based shape estimation: voxel point cloud, least-squares primitive fitting (sphere/cylinder/plane/box) |
| `ur5e_description` | Data | URDF + MJCF + meshes (DAE/STL/OBJ). Pinocchio/RViz/MuJoCo compatible |
| `ur5e_hand_driver` | Executable | UDP event-driven driver (SeqLock state, ppoll sub-ms timeout, dual motor+joint read, ONNX F/T inference) |
| `ur5e_bt_coordinator` | Executable | BehaviorTree.CPP v4 non-RT task coordinator (20 Hz, pick-and-place / towel unfold / hand motions) |
| `ur5e_bringup` | Launch/Config | robot.launch.py, sim.launch.py, hand.launch.py + DemoJoint/DemoTask controllers + GUI tools |

### Dependency Graph

```
rtc_msgs, rtc_base (independent)
  +-- rtc_communication <-- rtc_base
  +-- rtc_inference <-- rtc_base
  +-- rtc_controller_interface <-- rtc_base, rtc_msgs, rtc_urdf_bridge
  |     +-- rtc_controllers <-- rtc_controller_interface, rtc_urdf_bridge
  |           +-- rtc_controller_manager <-- rtc_controllers, rtc_communication
  +-- rtc_mujoco_sim <-- MuJoCo 3.x (optional)
  +-- rtc_digital_twin (independent, Python)
  +-- rtc_tools (independent, Python)
  +-- rtc_scripts (independent, shell)

rtc_urdf_bridge <-- Pinocchio, tinyxml2, yaml-cpp

shape_estimation_msgs (independent)
  +-- shape_estimation <-- shape_estimation_msgs, Eigen3

ur5e_description (independent)
  +-- ur5e_hand_driver <-- rtc_communication, rtc_inference, rtc_base
  +-- ur5e_bt_coordinator <-- rtc_msgs, BehaviorTree.CPP v4
  +-- ur5e_bringup <-- rtc_controller_manager, ur5e_hand_driver, ur5e_description
```

---

## Architecture

### Core Data Types (`rtc_base/types/types.hpp`)

```cpp
// Key constants
kCacheLineSize = 64;
kNumRobotJoints = 6;  kMaxRobotDOF = 12;  kMaxDeviceChannels = 64;  kMaxSensorChannels = 128;
kNumHandMotors = 10;  kDefaultNumFingertips = 4;  kMaxFingertips = 8;
kBarometerCount = 8;  kTofCount = 3;  kSensorValuesPerFingertip = 11;
kMaxHandSensors = 88;  kFTValuesPerFingertip = 7;  // contact(1)+F(3)+u(3)
kDefaultMaxJointVelocity = 2.0;  kDefaultMaxJointTorque = 150.0;

enum class CommandType { kPosition, kTorque };
enum class GoalType : uint8_t { kJoint, kTask };

// Generalized device types (variable DOF)
struct DeviceState     { positions[64], velocities[64], efforts[64],
                         motor_positions[64], motor_velocities[64], motor_efforts[64],
                         sensor_data[128], sensor_data_raw[128],
                         inference_data[64], inference_enable[8] };
struct ControllerState { devices[4], num_devices, dt, iteration };
struct DeviceOutput    { commands[64], goal_positions[64], target_positions[64],
                         target_velocities[64], trajectory_positions[64], trajectory_velocities[64], goal_type };
struct ControllerOutput { devices[4], actual_task_positions[6], task_goal_positions[6],
                          trajectory_task_positions[6], trajectory_task_velocities[6],
                          valid, command_type, grasp_state };
struct GraspStateData  { force_magnitude[8], contact_flag[8], inference_valid[8],
                         num_active_contacts, max_force, grasp_detected,
                         force_threshold, min_fingertips_for_grasp };

// Legacy types (still present, UR5e-specific — will be removed in PR3)
struct RobotState     { positions[6], velocities[6], torques[6], tcp_position[3], dt, iteration };
struct HandState      { motor_positions[10], motor_velocities[10], motor_currents[10],
                        joint_positions[10], joint_velocities[10], joint_currents[10],
                        sensor_data[88], sensor_data_raw[88],
                        num_fingertips, valid, received_joint_mode };
```

### Threading Model (6-core)

| Thread | Core | Scheduler | Priority | Role |
|--------|------|-----------|----------|------|
| rt_loop | 2 | SCHED_FIFO | 90 | clock_nanosleep 500Hz ControlLoop + 50Hz CheckTimeouts |
| sensor_executor | 3 | SCHED_FIFO | 70 | JointState, MotorState, SensorState, Target subscribers |
| log_executor | 4 | SCHED_OTHER | nice -5 | CSV 3-file logging (SpscLogBuffer drain) |
| publish_thread | 5 | SCHED_OTHER | nice -3 | SPSC -> ROS2 publish offload |
| aux_executor | 5 | SCHED_OTHER | 0 | Controller switching, gain updates, E-STOP publisher |
| udp_recv | 5 | SCHED_FIFO | 65 | Hand UDP receiver (ur5e_hand_driver) |

Core 0-1: OS/DDS/IRQ (isolcpus=2-5). DDS threads pinned to Core 0-1 via taskset.
Auto-selects 4/6/8/10/12/16-core layouts via `SelectThreadConfigs()`.

### Lock-Free Primitives

| Pattern | Where | Rule |
|---------|-------|------|
| **SeqLock<T>** | HandState sharing | Single-writer/multi-reader. `Store()` wait-free, `Load()` lock-free with retry. Requires `is_trivially_copyable_v<T>`. |
| **SpscLogBuffer<512> / SpscPublishBuffer<512>** | RT->log/publish offload | Power-of-2 size, bitwise AND modulus, local index caching. Push is wait-free noexcept; drops on full. |
| **atomic<bool>** | E-STOP flags | `seq_cst` default, no mutex on RT path |
| **jthread + stop_token** | MuJoCo sim, UDP receiver, RT loop | Cooperative cancellation |
| **try_lock** | viz_mutex, target_mutex, device_state_mutex | Never block RT thread |
| **acquire/release atomics** | SPSC head/tail, RT status flags | Producer-consumer sync |

---

## Controller Implementations

| Controller | Type | Space | Key Feature |
|------------|------|-------|-------------|
| PController | Indirect (position) | Joint | `q + kp*error*dt` incremental step |
| JointPDController | Direct (torque) | Joint | PD + Pinocchio RNEA gravity/Coriolis + feedforward velocity + JointSpaceTrajectory quintic |
| ClikController | Indirect (position) | Cartesian 3/6-DOF | Damped Jacobian pseudoinverse (LDLT) + null-space + TaskSpaceTrajectory SE3 quintic |
| OSC | Direct (torque) | Cartesian 6-DOF | Full pose (pos + SO(3) log3) + TaskSpaceTrajectory SE3 quintic + PartialPivLU |
| DemoJointController | Indirect | Joint + Hand | Quintic rest-to-rest trajectory (arm 6-DOF + hand 10-DOF), ContactStopHand |
| DemoTaskController | Indirect | Cartesian + Hand | CLIK arm + Quintic trajectory + Hand trajectory + E-STOP |

### Gains Layout (via `~/controller_gains` topic)

| Controller | Layout | Count |
|------------|--------|-------|
| **PController** | `[kp x 6]` | 6 |
| **JointPDController** | `[kp x 6, kd x 6, gravity(0/1), coriolis(0/1), trajectory_speed]` | 15 |
| **ClikController** | `[kp x 6, damping, null_kp, enable_null_space(0/1), control_6dof(0/1)]` | 10 |
| **OSC** | `[kp_pos x 3, kd_pos x 3, kp_rot x 3, kd_rot x 3, damping, gravity(0/1), traj_speed, traj_ang_speed]` | 16 |
| **DemoJoint** | `[robot_trajectory_speed, hand_trajectory_speed, robot_max_traj_velocity, hand_max_traj_velocity]` | 4 |
| **DemoTask** | `[kp_trans x 3, kp_rot x 3, damping, null_kp, enable_null_space(0/1), control_6dof(0/1), traj_speed, traj_angular_speed, hand_traj_speed, max_vel, max_angular_vel, hand_max_vel]` | 16 |

---

## RtControllerNode Key Methods

- `ControlLoop()` (500Hz): global_estop check -> assemble ControllerState from cached device states -> `Compute()` -> push PublishSnapshot to SPSC -> push LogEntry to log buffer
- `CheckTimeouts()` (50Hz): per-group device state timeout checks (dynamic, based on `device_timeout_names`/`device_timeout_values`) -> `TriggerGlobalEstop("{group}_timeout")`
- `TriggerGlobalEstop(reason)`: idempotent via `compare_exchange_strong`, propagates to all controllers + hand E-Stop
- `RtLoopEntry()`: clock_nanosleep + overrun recovery (skip missed ticks, consecutive overrun >= 10 -> E-STOP)
- `PublishLoopEntry()`: drains ControlPublishBuffer -> ROS2 publish (all DDS serialization off RT path)

### E-STOP Triggers

| Source | Condition | Action |
|--------|-----------|--------|
| CheckTimeouts (50Hz) | `{group}` state topic exceeds configured ms | `TriggerGlobalEstop("{group}_timeout")` |
| Init timeout | No data within `init_timeout_sec` | `TriggerGlobalEstop("init_timeout")` + shutdown |
| Consecutive overrun | >= 10 consecutive RT loop overruns | `TriggerGlobalEstop("consecutive_overrun")` |
| Sim sync timeout | CV-based wakeup timeout (sim mode) | `TriggerGlobalEstop("sim_sync_timeout")` + shutdown |

---

## ROS2 Topics

### Fixed Topics (RtControllerNode)

| Topic | Type | Dir | Description |
|-------|------|-----|-------------|
| `/{ns}/controller_type` | **String** | Sub | Runtime switch by controller name (e.g. "p_controller", "clik_controller") |
| `/{ns}/controller_gains` | Float64MultiArray | Sub | Dynamic gain update (layout per controller) |
| `/{ns}/request_gains` | Bool | Sub | Request current gains |
| `/{ns}/active_controller_name` | String | Pub | Active controller (TRANSIENT_LOCAL) |
| `/{ns}/current_gains` | Float64MultiArray | Pub | Current gains response |
| `/system/estop_status` | Bool | Pub | true = E-STOP active |

### Dynamic Topics (per controller TopicConfig)

| Subscribe Role | Message Type | Description |
|----------------|-------------|-------------|
| `kState` | JointState | Device joint-space state |
| `kMotorState` | JointState | Motor-space state |
| `kSensorState` | HandSensorState | Tactile sensor state |
| `kTarget` | RobotTarget | Joint/task space goal |

| Publish Role | Message Type | Description |
|-------------|-------------|-------------|
| `kJointCommand` | JointCommand | Joint command (position/torque) |
| `kRos2Command` | Float64MultiArray | ros2_control compatible command |
| `kGuiPosition` | GuiPosition | GUI current position display |
| `kRobotTarget` | RobotTarget | Target position publish |
| `kDeviceStateLog` | DeviceStateLog | State + command + trajectory log |
| `kDeviceSensorLog` | DeviceSensorLog | Sensor + inference log |
| `kGraspState` | GraspState | Grasp detection state (500Hz) |

### Digital Twin Auto-Republish

Per device group: `/{group}/digital_twin/joint_states` (JointState, RELIABLE/10) -- reordered joint data for RViz2 visualization.

### Hand UDP Driver Topics

| Topic | Type | Dir | Description |
|-------|------|-----|-------------|
| `/hand/joint_states` | JointState | Pub | Joint-space positions/velocities (kJoint read) |
| `/hand/motor_states` | JointState | Pub | Motor-space positions/velocities/currents (kMotor read) |
| `/hand/sensor_states` | HandSensorState | Pub | Fingertip sensors + F/T inference (BEST_EFFORT) |
| `/hand/sensor_states/monitor` | HandSensorState | Pub | Same data, RELIABLE QoS for non-RT subscribers |
| `/hand/link_status` | Bool | Pub | UDP link status |
| `/hand/joint_command` | JointCommand | Sub | 10 normalized motor commands |

### MuJoCo Simulator Topics

| Topic | Type | Dir | Description |
|-------|------|-----|-------------|
| `<group.state_topic>` | JointState | Pub | Per-group state (robot: every physics step, fake: 100Hz) |
| `<group.command_topic>` | JointCommand | Sub | Per-group command (auto position/torque mode switching) |
| `/sim/status` | Float64MultiArray | Pub | `[step_count, sim_time, rtf, paused]` (1Hz) |

### `/target_joint_positions` Interpretation (legacy)

- P/JointPD: joint angles (rad)
- CLIK (`control_6dof=false`): `[x,y,z, null_q3,null_q4,null_q5]`
- CLIK (`control_6dof=true`): `[x,y,z, roll,pitch,yaw]`
- OSC: `[x,y,z, roll,pitch,yaw]`
- DemoJoint: `data[0-5]` robot joints, `data[6-15]` hand motors (optional)

---

## Configuration Quick Reference

### rt_controller_manager.yaml

```yaml
robot_namespace: "ur5e"
control_rate: 500.0
initial_controller: "joint_pd_controller"    # name or config_key
init_timeout_sec: 5.0
auto_hold_position: true
use_sim_time_sync: false       # true for MuJoCo CV-based wakeup
sim_sync_timeout_sec: 5.0      # sim sync CV timeout

enable_estop: true
device_timeout_names: ["ur5e"]      # dynamic device-group based
device_timeout_values: [100.0]      # ms per group

enable_logging: true
enable_timing_log: true
enable_device_log: true
log_dir: ""
max_log_sessions: 10

# System URDF + model topology (shared by all controllers)
urdf:
  package: "ur5e_description"
  path: "robots/ur5e/urdf/ur5e_with_hand.urdf"
  root_joint_type: "fixed"
  sub_models:                          # name = device group name
    - { name: "ur5e", root_link: "base", tip_link: "tool0" }
  # tree_models:                       # enable when hand FK needed
  #   - { name: "hand", root_link: "hand_base_link",
  #       tip_links: [thumb_tip_link, index_tip_link, ...] }
  passive_joints: [thumb_cmc_aa, ..., ring_mcp_fe]

devices:
  ur5e:
    joint_state_names: [shoulder_pan_joint, ..., wrist_3_joint]
    safe_position: [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
    # root_link/tip_link auto-resolved from urdf.sub_models by device name
    joint_limits:
      max_velocity: [2.0, 2.0, 3.0, 3.0, 3.0, 3.0]
      max_torque: [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]
```

### mujoco_simulator.yaml

```yaml
model_path: "robots/ur5e/mjcf/scene_with_hand.xml"
sync_timeout_ms: 50.0           # command wait timeout (ms)
max_rtf: 1.0                    # 0.0 = unlimited
enable_viewer: true
physics_timestep: 0.002         # control period (constant), validates against XML
n_substeps: 1                   # substeps per control cycle (substep_dt = physics_timestep / n_substeps)
use_yaml_servo_gains: false     # true = apply YAML servo_kp/kd

robot_response:
  groups: ["ur5e", "hand"]
  ur5e: { command_joint_names: [...], state_topic: "/joint_states", command_topic: "/ur5e/joint_command" }
  hand: { command_joint_names: [...], state_topic: "/hand/joint_states", command_topic: "/hand/joint_command" }
# fake_response:   # alternative: LPF echo-back for devices not in MuJoCo XML
```

### hand_udp_node.yaml

```yaml
target_ip: "192.168.1.2"
target_port: 55151
recv_timeout_ms: 0.4             # ppoll sub-ms timeout (default 0.4)
communication_mode: "bulk"       # "bulk" (default) or "individual"
enable_failure_detector: true
# Note: sensor_decimation is fixed at 1 in code. joint_mode param is unused.
```

### digital_twin.yaml

```yaml
display_rate: 60.0
output_topic: "/digital_twin/joint_states"
auto_compute_mimic: true          # URDF mimic joints auto-calculated
num_sources: 2
source_0.topic: "/ur5e/digital_twin/joint_states"
source_1.topic: "/hand/digital_twin/joint_states"
# sensor_viz block enables fingertip MarkerArray visualization
```

---

## Hand UDP Protocol

Event-driven on port 55151 (jthread, Core 5, SCHED_FIFO/65):

**Bulk mode** (default, `communication_mode: "bulk"`):
1. WritePosition (0x01, kJoint) -> 43B send + 43B echo recv
2. ReadAllMotors (0x10, kMotor) -> 3B send -> 123B recv (pos+vel+cur x10)
3. ReadAllMotors (0x10, kJoint) -> 3B send -> 123B recv (pos+vel+cur x10)
4. ReadAllSensors (0x19) -> 3B send -> 259B recv (4 fingertips x 16 int32)

**Individual mode** (`communication_mode: "individual"`):
1. WritePosition (0x01, kJoint) -> 43B send + 43B echo recv
2. ReadPosition (0x11, kMotor) -> 3B send -> 43B recv
3. ReadPosition (0x11, kJoint) -> 3B send -> 43B recv
4. ReadVelocity (0x12, kMotor) -> 3B send -> 43B recv
5. ReadSensor0-3 (0x14-0x17) -> 3B send -> 67B recv x 4

ONNX F/T inference runs per sensor cycle when calibrated: input `float32[1, 12, 16]` -> output contact(1) + F(3) + u(3) per fingertip.

---

## Message Types (`rtc_msgs`) -- 8 types

| Message | Key Fields | Usage |
|---------|-----------|-------|
| `JointCommand` | joint_names[], values[], command_type ("position"/"torque") | Robot arm commands |
| `FingertipSensor` | barometer[8], tof[3], barometer_raw[8], tof_raw[3], f[3], u[3], contact_flag, inference_enable | Single fingertip sensor + inference |
| `HandSensorState` | header, fingertips[] (FingertipSensor array) | All fingertips aggregated |
| `GraspState` | fingertip_names[], force_magnitude[], contact_flag[], num_active_contacts, max_force, grasp_detected | 500Hz grasp detection for BT coordinator |
| `GuiPosition` | joint_names[], joint_positions[], task_positions[6] | GUI display |
| `RobotTarget` | goal_type ("joint"/"task"), joint_target[], task_target[6] | Goal commands |
| `DeviceStateLog` | actual_positions[], commands[], trajectory_positions[], motor_positions[] | CSV logging |
| `DeviceSensorLog` | sensor_data_raw[], sensor_data[], inference_output[] | CSV logging |

---

## Common Modification Patterns

### Adding a New Controller

1. **Header**: `rtc_controllers/include/rtc_controllers/{direct|indirect}/my_controller.hpp`
   - Inherit `RTControllerInterface`, implement `Compute()`, `SetDeviceTarget()`, `InitializeHoldPosition()`, `Name()` -- all `noexcept`
2. **Implementation**: `rtc_controllers/src/controllers/{direct|indirect}/my_controller.cpp`
   - `LoadConfig()` for YAML, `UpdateGainsFromMsg()` for runtime gains
3. **YAML**: `rtc_controllers/config/controllers/{direct|indirect}/my_controller.yaml`
   - Must include `topics:` section for device-group topic routing
4. **Register**: Add `RTC_REGISTER_CONTROLLER(my_controller, "indirect/", "rtc_controllers", ...)` in a source file linked into the controller library

For robot-specific controllers (e.g. DemoJoint/DemoTask), place in `ur5e_bringup/` instead and register in `ur5e_bringup/src/controllers/controller_registration.cpp`.

### Adding a New Message Type

1. Create `rtc_msgs/msg/MyMessage.msg`
2. Add to `CMakeLists.txt` in `rosidl_generate_interfaces()`
3. Build: `colcon build --packages-select rtc_msgs`
4. If used in publish offload: add `PublishRole` enum value in `rtc_base/types/types.hpp`, add YAML role string mapping in `rtc_controller_interface/src/rt_controller_interface.cpp`

### Adding a New Device Group

1. Add device config block in `rt_controller_manager.yaml` under `devices:`
2. Add timeout entry in `device_timeout_names` / `device_timeout_values`
3. Add topic routing in each controller's YAML `topics:` section
4. If the device has kinematics: add matching `sub_models` or `tree_models` entry under `urdf:` (name = device group name). `root_link`/`tip_link` will auto-resolve from the model config.
5. Controller must handle the new device index in `Compute()` / `SetDeviceTarget()`

### Adding a New Thread

1. Define `ThreadConfig` constants for all core tiers (4/6/8/10/12/16) in `rtc_base/threading/thread_config.hpp`
2. Add field to `SystemThreadConfigs` struct
3. Update `ValidateSystemThreadConfigs()` and `SelectThreadConfigs()`
4. Call `ApplyThreadConfig()` at thread entry
5. Use SCHED_FIFO + appropriate priority for RT threads

---

## Data Flow Diagram

```
[Robot HW / MuJoCo Sim]
    |  /joint_states (JointState, BEST_EFFORT/2)
    v
[rtc_controller_manager: RtControllerNode]
    |  RT loop (clock_nanosleep 500Hz, or CV-based sim sync)
    |  Controller: rtc_controllers (P / JointPD / CLIK / OSC / DemoJoint / DemoTask)
    |  State acquisition: try_lock on cached device states
    +----> SPSC -----> [publish_thread] -----> /forward_position_controller/commands
    |                                    +---> /{group}/digital_twin/joint_states (RELIABLE)
    |                                    +---> /hand/grasp_state (GraspState)
    +----> SPSC -----> [log_executor] -------> CSV 3-file (timing, per-device state, per-device sensor)
    +----> E-STOP ---> /system/estop_status

[Hand HW] <--UDP event-driven--> [ur5e_hand_driver] <--SeqLock--> [ControlLoop]
                                    +---> /hand/joint_states, /hand/motor_states, /hand/sensor_states

[rtc_digital_twin]
    /{group}/digital_twin/joint_states (RELIABLE) --merge--> /digital_twin/joint_states --> RViz2

[ur5e_bt_coordinator]
    subscribes: /ur5e/gui_position, /hand/grasp_state, /system/estop_status
    publishes: /ur5e/joint_goal, /hand/joint_goal, /ur5e/select_controller, /ur5e/gains
```

---

## Session Logging Structure

```
logging_data/YYMMDD_HHMM/
+-- controller/
|   +-- timing_log.csv        (timestamp, t_state_acquire_us, t_compute_us, t_publish_us, t_total_us, jitter_us)
|   +-- {device}_state_log.csv (per-device: goal, actual, command, trajectory, motor)
|   +-- {device}_sensor_log.csv (per-device: raw/filtered sensor + inference)
+-- monitor/                   (failure logs, controller_stats.json)
+-- hand/                      (hand_udp_stats.json)
+-- sim/                       (screenshot_*.ppm)
+-- plots/                     (rtc_tools output)
+-- motions/                   (motion editor output)
```

Session dir propagated via `RTC_SESSION_DIR` (or `UR5E_SESSION_DIR` fallback) env var.

---

## Debugging & Troubleshooting

### Common Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `ApplyThreadConfig()` warns | Missing `realtime` group or RT privileges | `sudo usermod -aG realtime $USER` + re-login |
| E-STOP fires immediately on startup | `init_timeout_sec` too short or no data source | Set `init_timeout_sec: 0.0` for sim, increase for real robot |
| High jitter (>200us) | DDS threads on RT cores | Check `taskset` pinning in launch, verify `isolcpus` |
| Hand timeout E-STOP | UDP link down or `recv_timeout_ms` too low | Check network, set `recv_timeout_ms: 0.4` |
| Controller not found | Name mismatch | Use config_key (e.g. "p_controller") or Name() (e.g. "PController") |
| MuJoCo XML joint mismatch | `command_joint_names` != XML actuator joints | Bidirectional exact match required for robot_response groups |

### Useful Debug Commands

```bash
# Check RT thread layout
PID=$(pgrep -f rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID

# Check topic rates
ros2 topic hz /joint_states
ros2 topic hz /forward_position_controller/commands

# Check controller parameters (introspection)
ros2 param list /rt_controller

# Check E-STOP
ros2 topic echo /system/estop_status

# MuJoCo sim status
ros2 topic echo /sim/status

# Verify RT setup
./rtc_scripts/scripts/check_rt_setup.sh --summary

# Verify runtime threads
./rtc_scripts/scripts/verify_rt_runtime.sh --summary
```

---

## Code Conventions

- **Namespace**: `rtc` (all packages)
- **Naming**: Google C++ Style -- `snake_case` members with trailing `_`
- **`noexcept` on all RT paths**: exceptions in 500Hz loop terminate the process (intentional)
- **C++20**: `std::jthread`, `std::stop_token`, `std::span`, `std::string_view`, `std::concepts`, designated initializers, `[[likely]]/[[unlikely]]`, `constexpr`
- **`[[nodiscard]]`** on functions returning status/error (`ApplyThreadConfig()`, `IsRunning()`, `Compute()`)
- **`static_assert`** on template parameters (`BesselFilterN<N>`, `KalmanFilterN<N>`, `JointSpaceTrajectory<N>` -- all require `N > 0`)
- **Include order**: project -> ROS2/third-party -> C++ stdlib
- **Separate mutexes**: `state_mutex_`, `target_mutex_`, `hand_mutex_` -- never hold more than one
- **Trajectory race fix**: `SetDeviceTarget()` uses `lock_guard`, `Compute()` uses `try_to_lock` (never blocks RT)
- **Eigen**: all buffers pre-allocated in constructor, `noalias()` to avoid temporaries, zero heap on 500Hz path
- **Pinocchio headers**: `#pragma GCC diagnostic push/pop` to suppress warnings
- **Compiler flags**: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion` (C++20 standard)

---

## RT Permissions

```bash
sudo groupadd realtime && sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# Re-login required. Verify: ulimit -r (99), ulimit -l (unlimited)
```

Optional CPU isolation:
```bash
# GRUB_CMDLINE_LINUX_DEFAULT (6-core): nohz_full=2-5 rcu_nocbs=2-5
# For dynamic isolation without reboot: sudo cpu_shield.sh on --robot
```

If `ApplyThreadConfig()` fails, the node continues at SCHED_OTHER with a `[WARN]` log (increased jitter).

---

## Key File Locations

| What | Path |
|------|------|
| Core types & constants | `rtc_base/include/rtc_base/types/types.hpp` |
| Thread configs (all tiers) | `rtc_base/include/rtc_base/threading/thread_config.hpp` |
| Controller abstract base | `rtc_controller_interface/include/rtc_controller_interface/rt_controller_interface.hpp` |
| Controller registry | `rtc_controller_interface/include/rtc_controller_interface/controller_registry.hpp` |
| RT control loop | `rtc_controller_manager/src/rt_controller_node.cpp` |
| Built-in controller registration | `rtc_controllers/src/controller_registration.cpp` |
| Demo controller registration | `ur5e_bringup/src/controllers/controller_registration.cpp` |
| Main YAML config (robot) | `ur5e_bringup/config/ur5e_robot.yaml` |
| Main YAML config (sim) | `ur5e_bringup/config/ur5e_sim.yaml` |
| MuJoCo sim config | `rtc_mujoco_sim/config/mujoco_simulator.yaml` |
| Hand driver config | `ur5e_hand_driver/config/hand_udp_node.yaml` |
| CycloneDDS config | `rtc_controller_manager/config/cyclone_dds.xml` |
| UR5e URDF | `ur5e_description/robots/ur5e/urdf/ur5e.urdf` |
| MuJoCo scene (with hand) | `ur5e_description/robots/ur5e/mjcf/scene_with_hand.xml` |
| BT trees | `ur5e_bt_coordinator/trees/*.xml` |
| Supplementary docs | `docs/` (RT_OPTIMIZATION.md, SHELL_SCRIPTS.md, VSCODE_DEBUGGING.md) |
