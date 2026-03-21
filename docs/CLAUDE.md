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
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py                              # MuJoCo sim
ros2 launch ur5e_bringup robot.launch.py robot_ip:=192.168.1.10              # real robot
ros2 launch ur5e_bringup robot.launch.py use_fake_hardware:=true             # fake HW
ros2 launch ur5e_hand_driver hand_udp.launch.py target_ip:=192.168.1.2      # hand only

# Monitor
ros2 topic hz /forward_position_controller/commands   # ~500Hz
ros2 topic echo /system/estop_status                  # true = E-STOP active
PID=$(pgrep -f rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
```

---

## Repository Structure

16 ROS2 packages at repo root (no `src/`). 13 `rtc_*` (robot-agnostic) + 3 `ur5e_*` (robot-specific). Each has its own `README.md` with detailed API and configuration.

### Package Summary

| Package | Type | Key Content |
|---------|------|-------------|
| `rtc_base` | Header-only | Types (`RobotState`, `ControllerOutput`), SeqLock, SPSC buffers, threading (4/6/8/10/12/16-core), Bessel/Kalman filters, DataLogger, session_dir |
| `rtc_communication` | Header-only | `TransportInterface` (abstract), `UdpSocket` RAII, `PacketCodec` concept, `Transceiver<T,C>` template |
| `rtc_controller_interface` | Library | `RTControllerInterface` abstract base (Init/Compute/SetTarget — all noexcept), `ControllerRegistry` singleton |
| `rtc_controllers` | Library | PController, JointPDController (Pinocchio RNEA), ClikController (Jacobian IK), OSC (6-DOF Cartesian PD) + quintic trajectory |
| `rtc_controller_manager` | Executable | `RtControllerNode`: clock_nanosleep RT loop, SPSC publish offload, 3-CSV logging, global E-STOP, controller lifecycle |
| `rtc_inference` | Header-only | `InferenceEngine` abstract, `OnnxEngine` (IoBinding, pre-allocated buffers), `RunModels()` batch helper |
| `rtc_status_monitor` | Shared lib | 10Hz monitor: robot mode, safety mode, tracking error, joint limits. Lock-free RT accessors (`isReady()`, `getFailure()`) |
| `rtc_msgs` | Messages | 7 types: JointCommand, HandMotorCommand, HandMotorFeedback, FingertipSensors, FingertipForceTorque, SystemStatus, ControllerDiagnostics |
| `rtc_mujoco_sim` | Executable | MuJoCo 3.x wrapper: FreeRun/SyncStep, GLFW viewer (40+ shortcuts), fake_hand 1st-order filter, position servo gains |
| `rtc_tools` | Python | controller_gui, plot_rtc_log, compare_mjcf_urdf, urdf_to_mjcf, hand_udp_sender, hand_data_plot |
| `rtc_scripts` | Shell | PREEMPT_RT kernel build, CPU shield (cset), IRQ affinity, UDP optimization, NVIDIA RT coexistence |
| `rtc_digital_twin` | Python | RViz2 visualization (joint_state → robot model) |
| `ur5e_description` | Data | URDF + MJCF + meshes (DAE/STL). Pinocchio/RViz/MuJoCo compatible |
| `ur5e_hand_driver` | Executable | UDP request-response driver (SeqLock state, ppoll sub-ms timeout, 44ch tactile sensors) |
| `ur5e_bringup` | Launch/Config | robot.launch.py, sim.launch.py + DemoJoint/DemoTask controllers |

### Dependency Graph

```
rtc_msgs, rtc_base (independent)
  ├── rtc_communication ← rtc_base
  ├── rtc_inference ← rtc_base
  ├── rtc_controller_interface ← rtc_base, rtc_msgs
  │   └── rtc_controllers ← rtc_controller_interface, Pinocchio
  │       └── rtc_controller_manager ← rtc_controllers, rtc_communication, rtc_status_monitor
  ├── rtc_status_monitor ← rtc_base, rtc_msgs
  └── rtc_mujoco_sim ← MuJoCo 3.x (optional)

ur5e_hand_driver ← rtc_communication, rtc_inference, rtc_base
ur5e_bringup ← rtc_controller_manager, ur5e_hand_driver, ur5e_description
```

---

## Architecture

### Core Data Types (`rtc_base/types/types.hpp`)

```cpp
// Key constants
kNumRobotJoints = 6;  kMaxRobotDOF = 12;
kNumHandMotors = 10;  kDefaultNumFingertips = 4;
kSensorValuesPerFingertip = 11;  // 8 barometer + 3 ToF
kMaxHandSensors = 88;  kMaxDeviceChannels = 64;  kMaxSensorChannels = 128;

enum class CommandType { kPosition, kTorque };

struct RobotState     { positions[6], velocities[6], torques[6], tcp_position[3], dt, iteration };
struct HandState      { motor_positions[10], motor_velocities[10], sensor_data[88], sensor_data_raw[88], num_fingertips, valid };
struct ControllerState { RobotState robot; HandState hand; double dt; uint64_t iteration; };
struct ControllerOutput { robot_commands[6], hand_commands[10], actual_target_positions[6], actual_task_positions[6], valid, command_type, goal_positions[6], target_velocities[6], hand_goal_positions[10] };
```

### Threading Model (v5.16.0, 6-core)

| Thread | Core | Scheduler | Priority | Role |
|--------|------|-----------|----------|------|
| rt_loop | 2 | SCHED_FIFO | 90 | clock_nanosleep 500Hz ControlLoop + 50Hz CheckTimeouts |
| sensor_executor | 3 | SCHED_FIFO | 70 | /joint_states, /target_joint_positions subscribers |
| log_executor | 4 | SCHED_OTHER | nice -5 | CSV 3-file logging (SpscLogBuffer drain) |
| publish_thread | 5 | SCHED_OTHER | nice -3 | SPSC → ROS2 publish offload |
| aux_executor | 5 | SCHED_OTHER | 0 | E-STOP status publisher |
| udp_recv | 5 | SCHED_FIFO | 65 | Hand UDP receiver |
| status_monitor | 4 | SCHED_OTHER | nice -2 | 10Hz status monitor |
| hand_failure | 4 | SCHED_OTHER | nice -2 | 50Hz hand failure detector |

Core 0-1: OS/DDS/IRQ (isolcpus=2-5). Auto-selects 4/6/8/10/12/16-core layouts via `SelectThreadConfigs()`.

### Lock-Free Primitives

- **SeqLock<T>**: Single-writer/multi-reader. `Store()` wait-free, `Load()` lock-free with retry. Requires `is_trivially_copyable_v<T>`.
- **SpscLogBuffer<512>** / **SpscPublishBuffer<512>**: SPSC ring buffers. Cache-line aligned (`alignas(64)`), bitwise AND modulus, local index caching. Push is wait-free noexcept; drops on full.
- **Atomic E-STOP**: `std::atomic<bool> global_estop_` — seq_cst by default for cross-thread visibility.

### Controller Implementations

| Controller | Type | Space | Key Feature |
|------------|------|-------|-------------|
| PController (idx 0) | Indirect (position) | Joint | `q + kp*error*dt` incremental step |
| JointPDController (idx 1) | Direct (torque) | Joint | PD + Pinocchio RNEA gravity/Coriolis + JointSpaceTrajectory quintic |
| ClikController (idx 2) | Indirect (position) | Cartesian 3/6-DOF | Damped Jacobian pseudoinverse + null-space + TaskSpaceTrajectory SE3 quintic |
| OSC (idx 3) | Direct (torque) | Cartesian 6-DOF | Full pose (pos + SO(3)) + TaskSpaceTrajectory SE3 quintic |
| DemoJointController (idx 4) | Indirect | Joint + Hand | Arm P(6-DOF) + Hand P(10-DOF) |
| DemoTaskController (idx 5) | Indirect | Cartesian + Hand | CLIK arm + Hand P + E-STOP |

**Gains layout** (via `~/controller_gains` topic):
- PController: `[kp×6]` (6 values)
- JointPD: `[kp×6, kd×6, gravity(0/1), coriolis(0/1), trajectory_speed]` (15)
- CLIK: `[kp×6, damping, null_kp, enable_null_space(0/1), control_6dof(0/1)]` (10)
- OSC: `[kp_pos×3, kd_pos×3, kp_rot×3, kd_rot×3, damping, gravity(0/1), traj_speed, traj_ang_speed]` (16)
- DemoJoint: `[robot_kp×6, hand_kp×10]` (16)
- DemoTask: `[kp×6, damping, null_kp, enable_null_space(0/1), control_6dof(0/1), hand_kp×10]` (20)

### RtControllerNode Key Methods

- `ControlLoop()` (500Hz): global_estop check → assemble ControllerState → Compute() → push PublishSnapshot to SPSC → push LogEntry to log buffer
- `CheckTimeouts()` (50Hz): `/joint_states` >100ms → robot_timeout E-STOP, `/hand/joint_states` >200ms → hand_timeout E-STOP
- `TriggerGlobalEstop(reason)`: atomic flag + controller E-Stop + hand E-Stop + log
- `RtLoopEntry()`: clock_nanosleep + overrun recovery (skip missed ticks, consecutive overrun → E-STOP)
- `PublishLoopEntry()`: drains ControlPublishBuffer → ROS2 publish

### E-STOP Triggers

| Source | Condition | Action |
|--------|-----------|--------|
| CheckTimeouts (50Hz) | /joint_states >100ms | `TriggerGlobalEstop("robot_timeout")` |
| CheckTimeouts (50Hz) | /hand/joint_states >200ms | `TriggerGlobalEstop("hand_timeout")` |
| UR5eStatusMonitor (10Hz) | Safety violation, tracking error, joint limit | `TriggerGlobalEstop(failure_type)` |
| HandFailureDetector (50Hz) | Zero/duplicate data | `TriggerGlobalEstop("hand_failure")` |
| Init timeout | No data within init_timeout_sec | `TriggerGlobalEstop("init_timeout")` + shutdown |

### Hand UDP Protocol

Request-response polling on port 55151 (jthread, Core 5, SCHED_FIFO/65):
1. WritePosition (43B send + 43B echo) → motor positions
2. ReadVelocity (43B send + 43B recv) → motor velocities
3. ReadSensor0-3 (3B send + 67B recv × 4) → 44 sensor values (decimated every N cycles)

`/hand/joint_states` layout: `[positions:10][velocities:10][sensors:44]` (64 doubles at 100Hz)

---

## ROS2 Topics

| Topic | Type | Dir | Description |
|-------|------|-----|-------------|
| `/joint_states` | JointState | Sub | 6-DOF positions + velocities |
| `/target_joint_positions` | Float64MultiArray | Sub | Interpretation varies by controller |
| `/hand/joint_states` | Float64MultiArray | Sub | 64 doubles: [pos:10][vel:10][sensors:44] |
| `/hand/command` | Float64MultiArray | Sub | 10 normalized motor commands (0.0–1.0) |
| `/forward_position_controller/commands` | Float64MultiArray | Pub | 6 position commands (rad) |
| `/forward_torque_controller/commands` | Float64MultiArray | Pub | 6 torque commands (Nm) |
| `/rt_controller/trajectory_state` | Float64MultiArray | Pub | 18: goal[6]+traj_pos[6]+traj_vel[6] |
| `/rt_controller/controller_state` | Float64MultiArray | Pub | 18: actual_pos[6]+vel[6]+cmd[6] |
| `/system/estop_status` | Bool | Pub | true = E-STOP active |
| `~/controller_type` | Int32 | Sub | Runtime switch: 0=P, 1=PD, 2=CLIK, 3=OSC, 4=Hand |
| `~/controller_gains` | Float64MultiArray | Sub | Dynamic gain update (layout per controller) |
| `/sim/status` | Float64MultiArray | Pub | [step_count, sim_time, rtf, paused] |

**`/target_joint_positions` interpretation**:
- P/JointPD: joint angles (rad)
- CLIK: `[x,y,z, null_q3,null_q4,null_q5]`
- OSC: `[x,y,z, roll,pitch,yaw]`
- DemoJoint: `data[0-5]` robot joints, `data[6-15]` hand motors (optional)

---

## Configuration

### rt_controller_manager.yaml (key params)

```yaml
control_rate: 500.0          # Hz
enable_logging: true
enable_timing_log: true
enable_robot_log: true
enable_device_log: true
max_log_sessions: 10
init_timeout_sec: 5.0
enable_status_monitor: false  # true for real robot
estop:
  enable_estop: true
  robot_timeout_ms: 100.0
  hand_timeout_ms: 200.0     # 0 to disable hand E-STOP
  safe_position: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
```

### mujoco_simulator.yaml (key params)

```yaml
sim_mode: "sync_step"        # "free_run" or "sync_step"
max_rtf: 1.0                 # 0.0 = unlimited
enable_viewer: true
enable_hand_sim: true
physics_timestep: 0.002      # validates against XML
use_yaml_servo_gains: false   # true = apply YAML servo_kp/kd
```

### hand_udp_node.yaml (key params)

```yaml
target_ip: "192.168.1.2"
target_port: 55151
recv_timeout_ms: 0.4         # ppoll sub-ms timeout
publish_rate: 100.0
enable_failure_detector: true
```

---

## Adding a Custom Controller (4 steps)

1. **Header**: `rtc_controllers/include/rtc_controllers/indirect/my_controller.hpp`
   - Inherit `RTControllerInterface`, implement `Compute()`, `SetRobotTarget()`, `Name()` — all `noexcept`
2. **Implementation**: `rtc_controllers/src/indirect/my_controller.cpp`
   - `LoadConfig()` for YAML, `UpdateGainsFromMsg()` for runtime gains
3. **YAML**: `rtc_controllers/config/controllers/indirect/my_controller.yaml`
4. **Register**: Add one entry to `MakeControllerEntries()` in `rtc_controller_manager/src/rt_controller_node.cpp`

---

## Code Conventions

- **Namespace**: `rtc` (all packages)
- **Naming**: Google C++ Style — `snake_case` members with trailing `_`
- **`noexcept` on all RT paths**: exceptions in 500Hz loop terminate the process (intentional)
- **C++20**: `std::jthread`, `std::stop_token`, `std::span`, `std::string_view`, `std::concepts`, designated initializers
- **Include order**: project → ROS2/third-party → C++ stdlib
- **Separate mutexes**: `state_mutex_`, `target_mutex_`, `hand_mutex_` — never hold more than one
- **Trajectory race fix**: `SetRobotTarget()` uses `lock_guard`, `Compute()` uses `try_to_lock` (never blocks RT)
- **Eigen**: all buffers pre-allocated in constructor, `noalias()` to avoid temporaries, zero heap on 500Hz path
- **Pinocchio headers**: `#pragma GCC diagnostic push/pop` to suppress warnings
- **Compiler flags**: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`

### Concurrency Patterns

| Pattern | Where | Rule |
|---------|-------|------|
| SPSC Ring Buffer | SpscLogBuffer, SpscPublishBuffer | power-of-2 size, bitwise AND modulus, local index caching |
| SeqLock | HandState sharing | trivially copyable T, wait-free write, lock-free read |
| atomic<bool> | E-STOP flags | seq_cst default, no mutex on RT path |
| jthread + stop_token | MuJoCo sim, UDP receiver, RT loop | cooperative cancellation |
| try_lock | viz_mutex, target_mutex, ref_mutex | never block RT/SimLoop thread |
| acquire/release atomics | SPSC head/tail, RT status flags | producer-consumer sync |

### Adding a New Thread

1. Define `ThreadConfig` constants for all core tiers (4/6/8/10/12/16)
2. Add field to `SystemThreadConfigs` struct
3. Update `ValidateSystemThreadConfigs()` and `SelectThreadConfigs()`
4. Call `ApplyThreadConfig()` at thread entry
5. Use SCHED_FIFO + appropriate priority for RT threads

---

## Session Logging Structure

```
logging_data/YYMMDD_HHMM/
├── controller/
│   ├── timing_log.csv     (7 cols: timestamp, t_state_acquire_us, t_compute_us, t_publish_us, t_total_us, jitter_us)
│   ├── robot_log.csv      (49 cols: timestamp, goal_pos, actual_pos, actual_vel, torque, task_pos, command, traj_pos, traj_vel)
│   └── device_log.csv     (87 cols: timestamp, device_valid, goal, cmd, actual, vel, sensors per fingertip)
├── monitor/               (failure logs, controller_stats.json)
├── device/                (hand_udp_stats.json)
├── sim/                   (screenshot_*.ppm)
├── plots/                 (rtc_tools output)
└── motions/               (motion editor output)
```

CSV column ordering follows 4-category taxonomy: **Goal → Current State → Command → Trajectory**.

---

## RT Permissions

```bash
sudo groupadd realtime && sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# Re-login required. Verify: ulimit -r (99), ulimit -l (unlimited)
```

Optional CPU isolation for max RT performance:
```bash
# GRUB_CMDLINE_LINUX_DEFAULT (6-core): isolcpus=2-5 nohz_full=2-5 rcu_nocbs=2-5
```

If `ApplyThreadConfig()` fails, the node continues at SCHED_OTHER with a `[WARN]` log (increased jitter).
