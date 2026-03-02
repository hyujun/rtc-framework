# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run Commands

**Prerequisites**: Ubuntu 22.04, ROS2 Humble, `realtime` group membership with `rtprio 99` / `memlock unlimited` in `/etc/security/limits.conf`.

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

# Run UDP hand nodes only
ros2 launch ur5e_rt_controller hand_udp.launch.py udp_port:=50001 target_ip:=192.168.1.100 target_port:=50002
```

**Monitoring**:
```bash
ros2 topic hz /forward_position_controller/commands   # should be ~500Hz
ros2 topic echo /system/estop_status                  # true = E-STOP active
ros2 control list_controllers -v
PID=$(pgrep -f custom_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
```

**Manually publish a target pose**:
```bash
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray "data: [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]"
```

---

## Repository Layout

```
ur5e_rt_controller/
├── CMakeLists.txt                          # Build config — v4.2.2, C++20, 3 executables
├── package.xml                             # ROS2 package metadata
├── install.sh                              # One-shot installation script
├── requirements.txt                        # Python deps: matplotlib, pandas, numpy, scipy
│
├── config/
│   ├── ur5e_rt_controller.yaml            # Controller gains, E-STOP, joint limits
│   └── hand_udp_receiver.yaml             # UDP hand receiver settings
│
├── docs/
│   ├── CHANGELOG.md                       # Version history (Korean)
│   └── RT_OPTIMIZATION.md                 # v4.2.0 RT tuning guide (Korean)
│
├── include/ur5e_rt_controller/
│   ├── rt_controller_interface.hpp        # Abstract base + all data structures
│   ├── data_logger.hpp                    # Non-RT CSV logger
│   ├── thread_config.hpp                  # ThreadConfig struct + predefined configs
│   ├── thread_utils.hpp                   # ApplyThreadConfig(), VerifyThreadConfig()
│   ├── hand_udp_receiver.hpp              # UDP receiver (C++20 jthread)
│   ├── hand_udp_sender.hpp                # UDP sender (little-endian doubles)
│   └── controllers/
│       ├── pd_controller.hpp              # PD + E-STOP (active implementation)
│       └── p_controller.hpp               # Simple P controller (alternative)
│
├── src/
│   ├── custom_controller.cpp              # Main 500Hz node — 4 executors, 4 threads
│   ├── hand_udp_receiver_node.cpp         # Bridges HandUdpReceiver → /hand/joint_states
│   └── hand_udp_sender_node.cpp           # Bridges /hand/command → UDP packets
│
├── launch/
│   ├── ur_control.launch.py               # Full system (UR driver + controller + monitor)
│   └── hand_udp.launch.py                 # Hand UDP nodes only
│
└── scripts/                               # Python utilities (installed to lib/)
    ├── motion_editor_gui.py               # Qt5 50-pose motion editor
    ├── monitor_data_health.py             # Data health monitor + JSON stats export
    ├── plot_ur_trajectory.py              # Matplotlib trajectory visualization
    └── hand_udp_sender_example.py         # Synthetic UDP hand data generator
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

`PDController` (`include/ur5e_rt_controller/controllers/pd_controller.hpp`) is the active implementation. On E-STOP it drives toward `kSafePosition = [0, -1.57, 1.57, -1.57, -1.57, 0]` rad. Output is clamped to `kMaxJointVelocity = 2.0 rad/s`. `PController` exists as a simpler alternative (proportional-only, no E-STOP).

### Main Node: `CustomController` (`src/custom_controller.cpp`)

The entire executable lives in this one file. It creates **4 `SingleThreadedExecutor`s**, each running in a dedicated `std::thread` with RT scheduling applied via `ApplyThreadConfig()`:

| Executor / Thread | Callback Group | CPU Core | Scheduler | Priority | What runs here |
|---|---|---|---|---|---|
| `rt_executor` / `t_rt` | `cb_group_rt_` | Core 2 | SCHED_FIFO | 90 | `ControlLoop()` (500Hz), `CheckTimeouts()` (50Hz E-STOP watchdog) |
| `sensor_executor` / `t_sensor` | `cb_group_sensor_` | Core 3 | SCHED_FIFO | 70 | `/joint_states`, `/target_joint_positions`, `/hand/joint_states` subscribers |
| `log_executor` / `t_log` | `cb_group_log_` | Core 4 | SCHED_OTHER | nice -5 | `DataLogger` CSV writes |
| `aux_executor` / `t_aux` | `cb_group_aux_` | Core 5 | SCHED_OTHER | 0 | E-STOP status publisher |

`mlockall(MCL_CURRENT | MCL_FUTURE)` is called at startup to prevent page faults. Shared state between threads is protected by three separate mutexes (`state_mutex_`, `target_mutex_`, `hand_mutex_`).

**Key methods in `CustomController`:**
- `DeclareAndLoadParameters()`: loads `control_rate`, `kp`, `kd`, `enable_estop`, `robot_timeout_ms`, `hand_timeout_ms`, `enable_logging`
- `CreateCallbackGroups()`: creates 4 `MutuallyExclusive` groups
- `JointStateCallback()`: stores positions/velocities under `state_mutex_`; updates `last_robot_update_` timestamp
- `TargetCallback()`: stores target positions under `target_mutex_`; calls `controller_->SetRobotTarget()`
- `HandStateCallback()`: records timestamp under `hand_mutex_` (data itself is not buffered here)
- `CheckTimeouts()` (50Hz): triggers E-STOP if data gaps exceed configured thresholds
- `ControlLoop()` (500Hz): assembles `ControllerState`, calls `Compute()`, publishes to `/forward_position_controller/commands`, logs to CSV

### UDP Hand Protocol

`HandUdpReceiver` (`include/ur5e_rt_controller/hand_udp_receiver.hpp`) uses `std::jthread` (C++20 cooperative cancellation) to receive packets on port 50001.

**Receive packet format — 77 `double`s (616 bytes total):**
- 11 motor positions + 11 motor velocities + 11 motor currents + 44 sensor values (4 per joint)

`HandUdpSender` encodes 11 `double`s as little-endian bytes and sends to port 50002 (normalized 0.0–1.0 motor commands).

`HandUdpReceiverNode` (`src/hand_udp_receiver_node.cpp`) bridges the receiver to ROS2: it latches incoming data via callback and re-publishes at 100Hz on `/hand/joint_states`.

`HandUdpSenderNode` (`src/hand_udp_sender_node.cpp`) bridges the other direction: subscribes to `/hand/command` and calls `HandUdpSender::SendCommand()`.

### E-STOP System

`CheckTimeouts()` runs at 50 Hz. If `/joint_states` is not received for >100ms, `PDController::TriggerEstop()` is called (sets `estopped_` atomic flag). If `/hand/joint_states` is not received for >200ms, `SetHandEstop(true)` is called separately. Both flags use `std::atomic<bool>` for safe cross-thread access between the RT thread and the 50 Hz watchdog.

To disable hand E-STOP (when no hand is connected): set `enable_estop: false` or `hand_timeout_ms: 0` in `config/ur5e_rt_controller.yaml`.

### DataLogger (`include/ur5e_rt_controller/data_logger.hpp`)

Non-copyable (move-only) CSV logger. Writes one row per control step:
`timestamp, current_pos_0..5, target_pos_0..5, command_0..5`

Default path: `/tmp/ur5e_control_log.csv`. Writes happen from the `log_executor` thread (Core 4), never from the 500Hz RT thread.

### Thread Configuration (`include/ur5e_rt_controller/thread_config.hpp`)

Predefined `ThreadConfig` constants for 6-core systems:

| Constant | Core | Policy | Priority |
|---|---|---|---|
| `kRtControlConfig` | 2 | SCHED_FIFO | 90 |
| `kSensorConfig` | 3 | SCHED_FIFO | 70 |
| `kUdpRecvConfig` | 3 | SCHED_FIFO | 65 |
| `kLoggingConfig` | 4 | SCHED_OTHER | nice -5 |
| `kAuxConfig` | 5 | SCHED_OTHER | 0 |

4-core fallback variants (`kRtControlConfig4Core`, `kSensorConfig4Core`, `kLoggingConfig4Core`) exist using Cores 1–3.

`ApplyThreadConfig()` in `thread_utils.hpp` applies CPU affinity (`pthread_setaffinity_np`), scheduler policy (`pthread_setschedparam`), and thread name (`pthread_setname_np`). Returns `false` on permission failure — the node continues at SCHED_OTHER with a `[WARN]` log.

---

## ROS2 Interface

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Subscribe | 6-DOF positions + velocities from UR driver |
| `/target_joint_positions` | `std_msgs/Float64MultiArray` | Subscribe | 6 target positions in radians |
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | Subscribe | 11 hand motor values from UDP receiver |
| `/hand/command` | `std_msgs/Float64MultiArray` | Subscribe | 11 normalized hand commands (0.0–1.0) |
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | Publish | 6 robot position commands (rad) |
| `/system/estop_status` | `std_msgs/Bool` | Publish | `true` = E-STOP active |

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

---

## Launch Files

### `launch/ur_control.launch.py` — Full System

| Argument | Default | Description |
|---|---|---|
| `robot_ip` | `192.168.1.10` | UR robot IP |
| `use_fake_hardware` | `false` | Simulation mode (no physical robot) |

Launches: UR robot driver (ur5e), `custom_controller` node (params from `ur5e_rt_controller.yaml`), `data_health_monitor` node (10Hz check rate, 0.2s timeout threshold).

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

Inherit from `RTControllerInterface`, implement `Compute()`, `SetRobotTarget()`, `SetHandTarget()`, and `Name()` — all must be `noexcept`. Then replace `PDController` in `custom_controller.cpp` at the constructor (line ~40):

```cpp
// Before:
controller_(std::make_unique<urtc::PDController>())

// After (example):
controller_(std::make_unique<urtc::MyController>())
```

No CMakeLists changes needed — controllers are header-only or compiled into the same executable.

---

## Code Conventions

- **Include order**: project headers first, then ROS2, then C++ stdlib (see `custom_controller.cpp` line 1 comment)
- **Namespace**: `ur5e_rt_controller` (aliased as `urtc` in `.cpp` files)
- **Naming**: Google C++ Style — `snake_case` members with trailing `_`, getters match member name without trailing `_`
- **`noexcept` on all RT paths**: exceptions in 500Hz callbacks terminate the process; this is intentional and required
- **C++20 features in use**: `std::jthread`, `std::stop_token`, designated initializers (`.field = value`), `std::concepts` (`NonNegativeFloat`), `std::span`, `std::string_view`
- **`std::atomic<bool>` for cross-thread flags**: E-STOP flags avoid mutex overhead on the RT path
- **Separate mutexes per domain**: `state_mutex_`, `target_mutex_`, `hand_mutex_` — never hold more than one simultaneously
- **`[[nodiscard]]`** on all functions returning status or computed values
- **Compiler warnings**: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion` — must compile warning-free

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
