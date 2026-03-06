# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run Commands

**Prerequisites**: Ubuntu 22.04 (ROS2 Humble) **or** Ubuntu 24.04 (ROS2 Jazzy), `realtime` group membership with `rtprio 99` / `memlock unlimited` in `/etc/security/limits.conf`.

```bash
# Automated setup (installs deps, builds, sets RT permissions)
chmod +x install.sh && ./install.sh

# Manual build (from workspace root, not repo root)
cd ~/ur_ws
colcon build --packages-select ur5e_rt_controller --symlink-install
source install/setup.bash

# Run full system (real robot)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=192.168.1.10

# Run with fake hardware (no robot needed)
ros2 launch ur5e_rt_controller ur_control.launch.py use_fake_hardware:=true

# Run MuJoCo simulation (requires MuJoCo 3.x installed)
ros2 launch ur5e_rt_controller mujoco_sim.launch.py
ros2 launch ur5e_rt_controller mujoco_sim.launch.py sim_mode:=sync_step
ros2 launch ur5e_rt_controller mujoco_sim.launch.py enable_viewer:=false

# Run UDP hand nodes only
ros2 launch ur5e_rt_controller hand_udp.launch.py udp_port:=50001 target_ip:=192.168.1.100 target_port:=50002
```

**Monitoring**:
```bash
ros2 topic hz /forward_position_controller/commands   # should be ~500Hz
ros2 topic echo /system/estop_status                  # true = E-STOP active
ros2 topic echo /sim/status                           # MuJoCo: steps + sim_time
ros2 control list_controllers -v
PID=$(pgrep -f custom_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
```

**Manually publish a target pose**:
```bash
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray "data: [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]"
```

**Analyze compute-time logs (after sync_step run)**:
```python
import pandas as pd
df = pd.read_csv('/tmp/ur5e_control_log.csv')
print(df['compute_time_us'].describe())
print(f'P95: {df["compute_time_us"].quantile(0.95):.1f} us')
print(f'P99: {df["compute_time_us"].quantile(0.99):.1f} us')
print(f'Over 2ms: {(df["compute_time_us"] > 2000).mean()*100:.2f}%')
```

---

## Repository Layout

v5.0.0+: 5개 독립 ROS2 패키지가 레포지토리 **루트 직하**에 위치합니다 (`src/` 없음).

```
ur5e-rt-controller/
├── install.sh                              # Automated setup (IRQ affinity included)
├── docs/
│   ├── CHANGELOG.md                       # Version history (Korean)
│   ├── RT_OPTIMIZATION.md                 # v4.2.0 RT tuning guide (Korean)
│   └── CORE_ALLOCATION_PLAN.md           # CPU core allocation optimization plan
│
├── ur5e_rt_base/                          # Shared header-only package
│   └── include/ur5e_rt_base/
│       ├── types.hpp                      # RobotState, HandState, ControllerState, ControllerOutput
│       ├── thread_config.hpp              # ThreadConfig + 4/6/8-core predefined RT constants
│       ├── thread_utils.hpp               # ApplyThreadConfig(), SelectThreadConfigs()
│       ├── log_buffer.hpp                 # SPSC ring buffer (lock-free, 512 entries)
│       └── data_logger.hpp                # Non-RT CSV logger
│
├── ur5e_rt_controller/                    # 500Hz RT controller package
│   ├── include/ur5e_rt_controller/
│   │   ├── rt_controller_interface.hpp    # Abstract base + all data structures
│   │   ├── controller_timing_profiler.hpp # Lock-free Compute() timing profiler
│   │   └── controllers/
│   │       ├── pd_controller.hpp          # PD + E-STOP (active default)
│   │       ├── p_controller.hpp           # Simple P controller
│   │       ├── pinocchio_controller.hpp   # Model-based PD + gravity/Coriolis
│   │       ├── clik_controller.hpp        # Closed-Loop IK (3-DOF Cartesian)
│   │       └── operational_space_controller.hpp # Full 6-DOF Cartesian PD
│   ├── src/custom_controller.cpp          # Main 500Hz node — 4 executors, 4 threads
│   ├── config/
│   │   ├── ur5e_rt_controller.yaml        # Controller gains, E-STOP, joint limits
│   │   └── cyclone_dds.xml                # CycloneDDS thread affinity (Core 0-1)
│   ├── scripts/
│   │   └── setup_irq_affinity.sh          # Pin NIC IRQs to Core 0-1
│   └── launch/
│       ├── ur_control.launch.py           # Full system (+ use_cpu_affinity, CYCLONEDDS_URI)
│       └── hand_udp.launch.py             # Hand UDP nodes only (via ur5e_hand_udp)
│
├── ur5e_hand_udp/                         # UDP hand bridge package
│   ├── include/ur5e_hand_udp/
│   │   ├── hand_udp_receiver.hpp          # UDP receiver (C++20 jthread, Core 5 FIFO 65)
│   │   └── hand_udp_sender.hpp            # UDP sender (little-endian doubles)
│   ├── src/
│   │   ├── hand_udp_receiver_node.cpp
│   │   └── hand_udp_sender_node.cpp
│   ├── config/hand_udp_receiver.yaml
│   └── launch/hand_udp.launch.py
│
├── ur5e_mujoco_sim/                       # MuJoCo 3.x simulator package (optional)
│   ├── include/ur5e_mujoco_sim/
│   │   └── mujoco_simulator.hpp           # Thread-safe MuJoCo physics wrapper
│   ├── src/mujoco_simulator_node.cpp
│   ├── models/ur5e/{scene.xml,ur5e.xml}
│   ├── config/mujoco_simulator.yaml
│   └── launch/mujoco_sim.launch.py
│
└── ur5e_tools/                            # Python utilities package
    └── scripts/
        ├── motion_editor_gui.py            # Qt5 50-pose motion editor
        ├── monitor_data_health.py          # Data health monitor + JSON stats
        ├── plot_ur_trajectory.py           # Matplotlib trajectory visualization
        └── hand_udp_sender_example.py      # Synthetic UDP hand data generator
```

---

## Architecture

This is a **ROS2 Humble** package (`ament_cmake`, C++20) implementing a 500 Hz real-time position controller for a UR5e robot arm with an 11-DOF custom hand attached via UDP.

### Core Design: Strategy Pattern + Multi-threaded Executors

`RTControllerInterface` (`include/ur5e_rt_controller/rt_controller_interface.hpp`) is the abstract base for all controllers. All virtual methods are `noexcept` — a hard RT safety requirement (an exception in a 500 Hz loop terminates the process). The key data types defined here are used throughout the codebase:

```cpp
// Compile-time constants
kNumRobotJoints = 6
kNumHandJoints  = 11
kNumHandSensors = 44  // 4 sensors × 11 joints

struct RobotState {
  std::array<double, 6>  positions{}, velocities{};
  std::array<double, 3>  tcp_position{};
  double dt{0.002}; uint64_t iteration{0};
};
struct HandState {
  std::array<double, 11> motor_positions{}, motor_velocities{}, motor_currents{};
  std::array<double, 44> sensor_data{};
  bool valid{false};
};
struct ControllerState { RobotState robot{}; HandState hand{}; double dt; uint64_t iteration; };
struct ControllerOutput { std::array<double,6> robot_commands{}; std::array<double,11> hand_commands{}; bool valid{true}; };
```

### Controller Implementations

| Controller | Header | Control Space | Notes |
|---|---|---|---|
| `PDController` | `controllers/pd_controller.hpp` | Joint-space PD | **Active default.** E-STOP drives to `kSafePosition`. |
| `PController` | `controllers/p_controller.hpp` | Joint-space P | Proportional-only, no E-STOP. |
| `PinocchioController` | `controllers/pinocchio_controller.hpp` | Joint-space PD + dynamics | Gravity + optional Coriolis compensation via Pinocchio RNEA. |
| `ClikController` | `controllers/clik_controller.hpp` | Cartesian 3-DOF | Damped Jacobian pseudoinverse + null-space joint centering. |
| `OperationalSpaceController` | `controllers/operational_space_controller.hpp` | Cartesian 6-DOF | Full pose (position + SO(3) orientation) control. |

**`PDController`** on E-STOP drives toward `kSafePosition = [0, -1.57, 1.57, -1.57, -1.57, 0]` rad. Output clamped to `kMaxJointVelocity = 2.0 rad/s`.

**`PinocchioController`** control law:
```
command[i] = Kp * e[i] + Kd * ė[i] + g(q)[i] [+ C(q,v)·v[i]]
```
All Eigen buffers pre-allocated in constructor — zero heap allocation on the 500 Hz path.

**`ClikController`** target convention (via `/target_joint_positions`): `[x, y, z, null_q3, null_q4, null_q5]` — first 3 are TCP position in metres, last 3 are null-space reference joints 3–5 in radians.

**`OperationalSpaceController`** target convention: `[x, y, z, roll, pitch, yaw]` — TCP position (m) + orientation (rad, ZYX Euler). Uses Pinocchio `log3()` for SO(3) orientation error.

### Main Node: `CustomController` (`src/custom_controller.cpp`)

The entire executable lives in this one file. It creates **4 `SingleThreadedExecutor`s**, each running in a dedicated `std::thread` with RT scheduling applied via `ApplyThreadConfig()`:

| Executor / Thread | Callback Group | CPU Core | Scheduler | Priority | What runs here |
|---|---|---|---|---|---|
| `rt_executor` / `t_rt` | `cb_group_rt_` | Core 2 | SCHED_FIFO | 90 | `ControlLoop()` (500Hz), `CheckTimeouts()` (50Hz E-STOP watchdog) |
| `sensor_executor` / `t_sensor` | `cb_group_sensor_` | Core 3 | SCHED_FIFO | 70 | `/joint_states`, `/target_joint_positions`, `/hand/joint_states` subscribers |
| `log_executor` / `t_log` | `cb_group_log_` | Core 4 | SCHED_OTHER | nice -5 | `DataLogger` CSV writes (drains `SpscLogBuffer`) |
| `aux_executor` / `t_aux` | `cb_group_aux_` | Core 5 | SCHED_OTHER | 0 | E-STOP status publisher |

`mlockall(MCL_CURRENT | MCL_FUTURE)` is called at startup to prevent page faults. Shared state between threads is protected by three separate mutexes (`state_mutex_`, `target_mutex_`, `hand_mutex_`).

**Key methods in `CustomController`:**
- `DeclareAndLoadParameters()`: loads `control_rate`, `kp`, `kd`, `enable_estop`, `robot_timeout_ms`, `hand_timeout_ms`, `enable_logging`
- `CreateCallbackGroups()`: creates 4 `MutuallyExclusive` groups
- `JointStateCallback()`: stores positions/velocities under `state_mutex_`; updates `last_robot_update_` timestamp
- `TargetCallback()`: stores target positions under `target_mutex_`; calls `controller_->SetRobotTarget()`
- `HandStateCallback()`: records timestamp under `hand_mutex_` (data itself is not buffered here)
- `CheckTimeouts()` (50Hz): triggers E-STOP if data gaps exceed configured thresholds
- `ControlLoop()` (500Hz): assembles `ControllerState`, calls `Compute()`, publishes to `/forward_position_controller/commands`, pushes to `SpscLogBuffer`

### Lock-Free Logging Infrastructure

**`SpscLogBuffer`** (`include/ur5e_rt_controller/log_buffer.hpp`): single-producer / single-consumer ring buffer (512 entries, power-of-2). The RT thread calls `Push()` without ever blocking or allocating; the log thread drains via `Pop()`. Each `LogEntry` now includes `compute_time_us` from `ControllerTimingProfiler`.

**`ControllerTimingProfiler`** (`include/ur5e_rt_controller/controller_timing_profiler.hpp`): wraps `RTControllerInterface::Compute()` with `steady_clock` timing. Maintains a lock-free histogram (0–2000 µs, 100 µs buckets) + min/max/mean/stddev/p95/p99 using relaxed atomics. Budget threshold: 2000 µs (500 Hz period). Call `MeasuredCompute()` instead of `Compute()` directly; call `Summary()` every 1000 iterations for a log line.

### MuJoCo Simulation

**`MuJoCoSimulator`** (`include/ur5e_rt_controller/mujoco_simulator.hpp`): thread-safe wrapper around a MuJoCo 3.x physics model. Optional — only built when `mujoco` CMake package is found.

Two simulation modes:
| Mode | `SimMode` | Description |
|---|---|---|
| Free-run | `kFreeRun` | Advances `mj_step()` as fast as possible. Best for algorithm validation. |
| Sync-step | `kSyncStep` | Publish state → wait for one command → step. Latency ≈ `Compute()` time. |

Both modes support `max_rtf` (maximum Real-Time Factor, 0.0 = unlimited) and RTF measurement displayed in the GLFW viewer overlay.

Threading model:
- `SimLoop` thread: runs `SimLoopFreeRun` or `SimLoopSyncStep` via `std::jthread`.
- `ViewerLoop` thread: renders scene at ~60 Hz via GLFW (optional, compiled-in with `-DMUJOCO_HAVE_GLFW`).
- Caller thread (ROS2 node): calls `SetCommand()`, `GetPositions()`, `GetVelocities()`.

Synchronisation:
- `cmd_mutex_` + `cmd_pending_` atomic — command transfer (lock-free fast path in FreeRun).
- `sync_cv_` — wakes `SimLoopSyncStep` when a command arrives.
- `state_mutex_` — protects latest state snapshot.
- `viz_mutex_` — `try_lock` only; never blocks `SimLoop`.

**`mujoco_simulator_node`** (`src/mujoco_simulator_node.cpp`): ROS2 node that wraps `MuJoCoSimulator`. Publishes `/joint_states` (at physics rate or decimated), `/hand/joint_states` (100 Hz, simulated via 1st-order filter), and `/sim/status`. Subscribes to `/forward_position_controller/commands` and `/hand/command`.

**MuJoCo model files** (`models/ur5e/`):
- `ur5e.xml` — MJCF robot model
- `scene.xml` — includes `ur5e.xml` with a ground plane

To use MuJoCo Menagerie instead:
```bash
ros2 launch ur5e_rt_controller mujoco_sim.launch.py \
    model_path:=/path/to/mujoco_menagerie/universal_robots_ur5e/scene.xml
```

**Install MuJoCo 3.x** (to enable `mujoco_simulator_node`):
```bash
wget https://github.com/google-deepmind/mujoco/releases/download/3.x.x/mujoco-3.x.x-linux-x86_64.tar.gz
sudo tar -xzf mujoco-*.tar.gz -C /opt/
# Then rebuild with:
cmake -Dmujoco_DIR=/opt/mujoco-3.x.x/lib/cmake/mujoco ...
```

### UDP Hand Protocol

`HandUdpReceiver` (`include/ur5e_rt_controller/hand_udp_receiver.hpp`) uses `std::jthread` (C++20 cooperative cancellation) to receive packets on port 50001.

**Receive packet format — 77 `double`s (616 bytes total):**
- 11 motor positions + 11 motor velocities + 11 motor currents + 44 sensor values (4 per joint)

`HandUdpSender` encodes 11 `double`s as little-endian bytes and sends to port 50002 (normalized 0.0–1.0 motor commands).

`HandUdpReceiverNode` (`src/hand_udp_receiver_node.cpp`) bridges the receiver to ROS2: it latches incoming data via callback and re-publishes at 100Hz on `/hand/joint_states`.

`HandUdpSenderNode` (`src/hand_udp_sender_node.cpp`) bridges the other direction: subscribes to `/hand/command` and calls `HandUdpSender::SendCommand()`.

### E-STOP System

`CheckTimeouts()` runs at 50 Hz. If `/joint_states` is not received for >100ms, `PDController::TriggerEstop()` is called (sets `estopped_` atomic flag). If `/hand/joint_states` is not received for >200ms, `SetHandEstop(true)` is called separately. Both flags use `std::atomic<bool>` for safe cross-thread access between the RT thread and the 50 Hz watchdog.

To disable hand E-STOP (when no hand is connected): set `enable_estop: false` or `hand_timeout_ms: 0` in `config/ur5e_rt_controller.yaml`. For MuJoCo simulation, `mujoco_simulator.yaml` already sets `enable_estop: false`.

### DataLogger (`include/ur5e_rt_controller/data_logger.hpp`)

Non-copyable (move-only) CSV logger. Writes one row per control step:
`timestamp, current_pos_0..5, target_pos_0..5, command_0..5, compute_time_us`

Default path: `/tmp/ur5e_control_log.csv`. The 500 Hz RT thread pushes entries to `SpscLogBuffer`; the `log_executor` thread (Core 4) drains and writes CSV — never blocking the RT path.

### Thread Configuration (`ur5e_rt_base/include/ur5e_rt_base/thread_config.hpp`)

`SelectThreadConfigs()` in `thread_utils.hpp` auto-selects the right config set at runtime based on `GetOnlineCpuCount()`.

**6-core layout** (≥6 CPUs):

| Constant | Core | Policy | Priority | Note |
|---|---|---|---|---|
| `kRtControlConfig` | 2 | SCHED_FIFO | 90 | 500Hz control + 50Hz E-STOP watchdog |
| `kSensorConfig` | 3 | SCHED_FIFO | 70 | Dedicated — no udp_recv contention |
| `kUdpRecvConfig` | 5 | SCHED_FIFO | 65 | Moved from Core 3 → Core 5 (v5.1.0) |
| `kLoggingConfig` | 4 | SCHED_OTHER | nice -5 | |
| `kAuxConfig` | 5 | SCHED_OTHER | 0 | Shares Core 5 with udp_recv (light) |

**8-core layout** (≥8 CPUs):

| Constant | Core | Policy | Priority |
|---|---|---|---|
| `kRtControlConfig8Core` | 2 | SCHED_FIFO | 90 |
| `kSensorConfig8Core` | 3 | SCHED_FIFO | 70 |
| `kUdpRecvConfig8Core` | 4 | SCHED_FIFO | 65 |
| `kLoggingConfig8Core` | 5 | SCHED_OTHER | nice -5 |
| `kAuxConfig8Core` | 6 | SCHED_OTHER | 0 |

**4-core fallback** (<6 CPUs): Cores 1–3; udp_recv shares Core 2 with sensor_io (`kUdpRecvConfig4Core`).

`ApplyThreadConfig()` applies CPU affinity (`pthread_setaffinity_np`), scheduler policy (`pthread_setschedparam`), and thread name (`pthread_setname_np`). Returns `false` on permission failure — the node continues at SCHED_OTHER with a `[WARN]` log.

`SystemThreadConfigs` struct (returned by `SelectThreadConfigs()`):
```cpp
struct SystemThreadConfigs {
  ThreadConfig rt_control, sensor, udp_recv, logging, aux;
};
```

---

## ROS2 Interface

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Subscribe | 6-DOF positions + velocities from UR driver or MuJoCo sim |
| `/target_joint_positions` | `std_msgs/Float64MultiArray` | Subscribe | 6 values: joint angles (rad) for PD/Pinocchio, TCP pose for CLIK/OSC |
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | Subscribe | 11 hand motor values from UDP receiver or MuJoCo sim |
| `/hand/command` | `std_msgs/Float64MultiArray` | Subscribe | 11 normalized hand commands (0.0–1.0) |
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | Publish | 6 robot position commands (rad) |
| `/system/estop_status` | `std_msgs/Bool` | Publish | `true` = E-STOP active |
| `/sim/status` | `std_msgs/Float64MultiArray` | Publish | MuJoCo only: `[step_count, sim_time_sec, rtf]` |

**Note on `/target_joint_positions` interpretation**: The 6 values are interpreted differently by each controller:
- `PDController` / `PinocchioController`: joint angles in radians
- `ClikController`: `[x, y, z, null_q3, null_q4, null_q5]` (TCP position + null-space reference)
- `OperationalSpaceController`: `[x, y, z, roll, pitch, yaw]` (full TCP pose)

---

## Configuration Reference

### `config/ur5e_rt_controller.yaml`

```yaml
controller:
  control_rate: 500.0        # Hz — timer period = 1e6/rate µs
  kp: 5.0                    # PD proportional gain
  kd: 0.5                    # PD derivative gain
  enable_logging: true       # Write CSV to log_path
  log_path: "/tmp/ur5e_control_log.csv"

joint_limits:
  max_velocity: 2.0          # rad/s — enforced in PDController::ClampCommands()
  max_acceleration: 5.0      # rad/s² (informational; not enforced in code)
  position_limits:           # Per-joint soft limits (informational)
    joint_0: [-6.28, 6.28]   # Base
    joint_1: [-6.28, 6.28]   # Shoulder
    joint_2: [-3.14, 3.14]   # Elbow
    joint_3: [-6.28, 6.28]   # Wrist 1
    joint_4: [-6.28, 6.28]   # Wrist 2
    joint_5: [-6.28, 6.28]   # Wrist 3

estop:
  enable_estop: true
  robot_timeout_ms: 100.0    # Trigger if /joint_states gap exceeds this
  hand_timeout_ms: 200.0     # Trigger if /hand/joint_states gap exceeds this; set 0 to disable
  safe_position: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # Recovery position (rad)

logging:
  log_frequency: 100.0       # Hz (subsampling intent; not currently enforced)
  max_log_size_mb: 100
  log_directory: "/tmp/ur5e_logs"
```

### `config/hand_udp_receiver.yaml`

```yaml
udp:
  port: 50001                # Listening port (override at launch with udp_port:=)
  buffer_size: 1024
  timeout_ms: 1000

publishing:
  rate: 100.0                # /hand/joint_states publish rate
  topic: "/hand/joint_states"

monitoring:
  enable_statistics: true
  statistics_period: 5.0    # seconds
```

### `config/mujoco_simulator.yaml`

```yaml
mujoco_simulator:
  ros__parameters:
    model_path: ""             # Empty → <package>/models/ur5e/scene.xml
    sim_mode: "free_run"       # "free_run" or "sync_step"
    publish_decimation: 1      # free_run: publish every N steps
    sync_timeout_ms: 50.0      # sync_step: command wait timeout
    max_rtf: 0.0               # Max Real-Time Factor (0.0 = unlimited)
    enable_viewer: true        # GLFW 3D viewer (false for headless)
    initial_joint_positions: [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
    enable_hand_sim: true      # Simulate hand with 1st-order filter
    hand_filter_alpha: 0.1     # Filter coefficient per 10ms tick

# Overrides custom_controller params when launched via mujoco_sim.launch.py
custom_controller:
  ros__parameters:
    enable_estop: false        # Prevents false E-STOPs in free_run mode
    robot_timeout_ms: 10000.0
    hand_timeout_ms:  10000.0
```

---

## Launch Files

### `launch/ur_control.launch.py` — Full System (Real Robot)

| Argument | Default | Description |
|---|---|---|
| `robot_ip` | `192.168.1.10` | UR robot IP |
| `use_fake_hardware` | `false` | Simulation mode (no physical robot) |
| `use_cpu_affinity` | `true` | Pin ur_ros2_driver to Core 0-1 via taskset (3s after launch) |

Launches: UR robot driver (ur5e), `custom_controller` node (params from `ur5e_rt_controller.yaml`), `data_health_monitor` node (10Hz check rate, 0.2s timeout threshold).

Also sets environment variables automatically:
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI=file://<pkg>/config/cyclone_dds.xml` — restricts DDS threads to Core 0-1

### `launch/mujoco_sim.launch.py` — MuJoCo Simulation

| Argument | Default | Description |
|---|---|---|
| `model_path` | `""` | Absolute path to scene.xml; empty → package default |
| `sim_mode` | `free_run` | `free_run` (max speed) or `sync_step` (1:1 sync) |
| `enable_viewer` | `true` | Open GLFW 3D viewer window |
| `publish_decimation` | `1` | free_run: publish every N steps |
| `sync_timeout_ms` | `50.0` | sync_step: command wait timeout (ms) |
| `max_rtf` | `0.0` | Max Real-Time Factor (0.0 = unlimited) |
| `kp` | `5.0` | PD controller proportional gain |
| `kd` | `0.5` | PD controller derivative gain |

Launches: `mujoco_simulator_node`, `custom_controller`, `data_health_monitor`.

### `launch/hand_udp.launch.py` — Hand UDP Only

| Argument | Default | Description |
|---|---|---|
| `udp_port` | `50001` | UDP receive port |
| `target_ip` | `192.168.1.100` | Hand controller IP |
| `target_port` | `50002` | UDP send port |

Launches: `hand_udp_receiver_node`, `hand_udp_sender_node`.

---

## Python Utilities

### `scripts/motion_editor_gui.py`

Qt5 50-pose motion editor GUI. Subscribes to `/joint_states` (current angles), publishes to `/target_joint_positions` (execute poses). Supports JSON save/load and sequential playback with 2s inter-pose delay. Requires `PyQt5` system package.

### `scripts/monitor_data_health.py`

`DataHealthMonitor` ROS2 node. Tracks packet rates and timeouts across all 4 topics. Saves JSON stats to `/tmp/ur5e_stats/` on shutdown. Parameters: `check_rate` (default 10Hz), `timeout_threshold` (default 0.2s), `stats_output_dir`, `enable_stats`.

### `scripts/plot_ur_trajectory.py`

Matplotlib visualization of CSV control logs. Plots positions, targets, and commands per joint.

```bash
ros2 run ur5e_rt_controller plot_ur_trajectory.py /tmp/ur5e_control_log.csv
ros2 run ur5e_rt_controller plot_ur_trajectory.py /tmp/ur5e_control_log.csv --joint 2
```

### `scripts/hand_udp_sender_example.py`

Synthetic hand data generator for development/testing. Sends sinusoidal or static UDP packets to the receiver on port 50001.

---

## Adding a Custom Controller

Inherit from `RTControllerInterface`, implement `Compute()`, `SetRobotTarget()`, `SetHandTarget()`, and `Name()` — all must be `noexcept`. Then replace `PDController` in `custom_controller.cpp` at the constructor:

```cpp
// Before (line ~40 of custom_controller.cpp):
controller_(std::make_unique<urtc::PDController>())

// After (PinocchioController example):
controller_(std::make_unique<urtc::PinocchioController>(
    "$(ros2 pkg prefix ur5e_description)/share/ur5e_description/robots/ur5e/urdf/ur5e.urdf",
    urtc::PinocchioController::Gains{
        .kp = 5.0,
        .kd = 0.5,
        .enable_gravity_compensation  = true,
        .enable_coriolis_compensation = false}))

// After (ClikController example — target = [x, y, z, null_q3, null_q4, null_q5]):
controller_(std::make_unique<urtc::ClikController>(
    "$(ros2 pkg prefix ur5e_description)/share/ur5e_description/robots/ur5e/urdf/ur5e.urdf",
    urtc::ClikController::Gains{.kp = 1.0, .damping = 0.01, .null_kp = 0.5}))

// After (OperationalSpaceController — target = [x, y, z, roll, pitch, yaw]):
controller_(std::make_unique<urtc::OperationalSpaceController>(
    "$(ros2 pkg prefix ur5e_description)/share/ur5e_description/robots/ur5e/urdf/ur5e.urdf",
    urtc::OperationalSpaceController::Gains{
        .kp_pos = 1.0, .kd_pos = 0.1, .kp_rot = 0.5, .kd_rot = 0.05}))
```

When switching to Pinocchio-based controllers, remove the `controller_->set_gains(kp, kd)` call in `DeclareAndLoadParameters()` — gains are passed through the constructor instead.

No CMakeLists changes needed for header-only controllers. `PinocchioController`, `ClikController`, and `OperationalSpaceController` require Pinocchio (already linked via `target_link_libraries(custom_controller pinocchio::pinocchio)`).

---

## Code Conventions

- **Include order**: project headers first, then ROS2/third-party, then C++ stdlib (see `custom_controller.cpp` line 1 comment)
- **Namespace**: `ur5e_rt_controller` (aliased as `urtc` in `.cpp` files)
- **Naming**: Google C++ Style — `snake_case` members with trailing `_`, getters match member name without trailing `_`
- **`noexcept` on all RT paths**: exceptions in 500Hz callbacks terminate the process; this is intentional and required
- **C++20 features in use**: `std::jthread`, `std::stop_token`, designated initializers (`.field = value`), `std::concepts` (`NonNegativeFloat`), `std::span`, `std::string_view`
- **`std::atomic<bool>` for cross-thread flags**: E-STOP flags avoid mutex overhead on the RT path
- **Separate mutexes per domain**: `state_mutex_`, `target_mutex_`, `hand_mutex_` — never hold more than one simultaneously
- **`[[nodiscard]]`** on all functions returning status or computed values
- **Compiler warnings**: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion` — must compile warning-free
- **Pinocchio headers**: always wrapped in `#pragma GCC diagnostic push/pop` to suppress Pinocchio's own `-Wconversion` / `-Wshadow` warnings
- **Eigen heap policy**: all Eigen work buffers pre-allocated in constructors; `noalias()` used to avoid temporaries. No `new` / heap allocations on the 500 Hz path.

---

## Key Constants (from `rt_controller_interface.hpp`)

- `kNumRobotJoints = 6`
- `kNumHandJoints = 11`
- `kNumHandSensors = 44` (4 sensors × 11 joints)

---

## RT Permissions (required for SCHED_FIFO)

```bash
sudo groupadd realtime
sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# Must log out and log back in
ulimit -r  # should print 99
ulimit -l  # should print unlimited
```

If `ApplyThreadConfig()` fails, the node logs `[WARN] Thread config failed` and continues without RT scheduling (increased jitter). The controller still functions but 500 Hz timing is not guaranteed.

### Optional: CPU Isolation for Maximum RT Performance

```bash
# Add to GRUB_CMDLINE_LINUX_DEFAULT in /etc/default/grub (6-core system)
# isolcpus=2-5 nohz_full=2-5 rcu_nocbs=2-5
sudo update-grub && sudo reboot

# Verify
cat /sys/devices/system/cpu/isolated  # should show: 2-5
```

---

## Performance Characteristics

| Metric | Before v4.2.0 | v4.2.0+ | Improvement |
|---|---|---|---|
| Control jitter | ~500μs | <50μs | 10x |
| E-STOP response | ~100ms | <20ms | 5x |
| CPU usage | ~30% | ~25% | -17% |
| Context switches | ~5000/s | ~1000/s | -80% |
| Priority inversion | Present | Eliminated | — |

### Verify Jitter with cyclictest

```bash
sudo apt install rt-tests
sudo cyclictest --mlockall --smp --priority=90 --policy=fifo \
    --interval=2000 --loops=100000 --affinity=2 --histogram=200
# Target: Max jitter < 50μs
```

For detailed RT tuning (CPU isolation, kernel parameters, DDS configuration, IRQ affinity), see `docs/RT_OPTIMIZATION.md`.

---

## Version History Summary

| Version | Key Changes |
|---|---|
| v4.4.0 | MuJoCo simulator integration (`mujoco_simulator.hpp`, `mujoco_simulator_node.cpp`, `mujoco_sim.launch.py`), RTF measurement + viewer overlay, `ControllerTimingProfiler`, `SpscLogBuffer` |
| v4.3.0 | Pinocchio model-based controllers: `PinocchioController`, `ClikController`, `OperationalSpaceController` |
| v4.2.3 | RT safety fixes: 9 hardening items from `planning_rt.md` |
| v4.2.0 | Multi-executor architecture, `mlockall`, CPU affinity; 10x jitter reduction |
