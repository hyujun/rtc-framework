# ur5e_status_monitor

**Non-RT safety & status monitor for UR5e servoJ real-time control (v5.10.0)**

![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-blue)
![C++](https://img.shields.io/badge/C%2B%2B-20-brightgreen)
![License](https://img.shields.io/badge/License-MIT-yellow)

---

## Overview

`ur5e_status_monitor` is a shared library that provides the `UR5eStatusMonitor` class — a non-real-time monitor designed to be composed into an existing ROS2 node (e.g., `RtControllerNode`). It runs a 10 Hz `std::jthread` that monitors robot status, joint tracking errors, joint limit proximity, and topic watchdogs. It exposes lock-free atomic flags that the 500 Hz RT control loop can read without blocking.

### Architecture

```
  ┌─────────────────────────────────────────────┐
  │          RT Control Thread (500 Hz)          │
  │                                              │
  │  setJointReference(q_ref, qd_ref)  ──────┐  │
  │                                           │  │
  │  if (!monitor.isReady()) return;          │  │
  │  if (monitor.getFailure() != kNone) ...   │  │
  │  if (monitor.isJointLimitWarning()) ...   │  │
  └───────────────────────────────────────────│──┘
              ↕ atomic reads (lock-free)      │
  ┌───────────────────────────────────────────│──┐
  │      UR5eStatusMonitor (10 Hz jthread)    │  │
  │                                           ↓  │
  │  [subscribe callbacks] → [shared state]      │
  │         ↓                                    │
  │  [check all conditions]                      │
  │         ↓                                    │
  │  [publish diagnostics]                       │
  │  [invoke callbacks if needed]                │
  │  [write failure log if needed]               │
  └──────────────────────────────────────────────┘
              ↕ ROS2 subscriptions / services
  ┌──────────────────────────────────────────────┐
  │    ur_robot_driver / controller_manager       │
  └──────────────────────────────────────────────┘
```

---

## Monitored Conditions

| Condition | Threshold | Action | FailureType |
|-----------|-----------|--------|-------------|
| RobotMode != RUNNING | — | Set failure flag | `kEstop` / varies |
| SafetyMode == PROTECTIVE_STOP | — | Set failure flag | `kProtectiveStop` |
| SafetyMode == VIOLATION/FAULT | — | Set failure flag | `kSafetyViolation` |
| SafetyMode == E-STOP variants | — | Set failure flag | `kEstop` |
| Program not running | — | Set failure flag | `kProgramDisconnected` |
| /joint_states timeout | `watchdog_timeout_sec` (1.0 s) | Set failure flag | `kWatchdogTimeout` |
| Target controller inactive | poll every 5 s | Set failure flag | `kControllerInactive` |
| Position tracking error | warn: 0.05 rad, fault: 0.15 rad | Warn / failure | `kTrackingError` |
| Velocity tracking error | warn: 0.1 rad/s, fault: 0.3 rad/s | Warn / failure | `kTrackingError` |
| Joint limit proximity | warn: 5.0 deg, fault: 1.0 deg | Warn / failure | `kJointLimitViolation` |

### Message Statistics (v5.9.0)

Joint state 메시지의 패킷 수, 타임아웃 횟수, 현재 수신 rate를 추적합니다.

```cpp
struct MessageStats {
  uint64_t total_count{0};     // 총 수신 패킷 수
  uint64_t timeout_count{0};   // watchdog 타임아웃 횟수
  double current_rate_hz{0.0}; // 현재 수신 rate (1초 윈도우)
};

// RT-safe accessor
auto stats = monitor->getJointStateStats();
```

### Per-Controller Statistics (v5.9.0)

컨트롤러별 활성 시간, 패킷 수, 타임아웃, 장애 횟수를 독립적으로 추적합니다. `/rt_controller/active_controller_name` 토픽을 구독하여 컨트롤러 전환을 감지합니다.

```cpp
struct ControllerStats {
  std::string name;
  double total_active_sec{0.0};
  uint64_t js_packets{0};
  uint64_t js_timeouts{0};
  uint64_t failure_count{0};
};
```

**JSON 통계 내보내기**: `stop()` 호출 시 `{log_output_dir}/controller_stats_YYYYMMDDTHHMMSS.json`에 저장됩니다. v5.10.0부터 기본 경로는 `UR5E_SESSION_DIR/monitor/`입니다.

---

## Dependencies

| Package | Purpose |
|---------|---------|
| `rclcpp` | ROS2 C++ client library |
| `std_msgs` | Int32, Bool message types for UR driver topics |
| `sensor_msgs` | JointState message for joint positions/velocities |
| `diagnostic_msgs` | DiagnosticArray for status publishing |
| `controller_manager_msgs` | ListControllers service for controller state |
| `ur5e_rt_base` | Shared constants (kNumRobotJoints) and types |
| `ur_dashboard_msgs` (optional) | Dashboard services for auto-recovery |

---

## Build & Install

```bash
# Ensure workspace is set up
cd ~/ros2_ws/ur5e_ws/src

# Build (using project build script — includes ur5e_status_monitor)
cd ~/ros2_ws/ur5e_ws/src/ur5e-rt-controller
./build.sh sim    # or: ./build.sh robot, ./build.sh full

# Or build individually
cd ~/ros2_ws/ur5e_ws
colcon build --packages-select ur5e_status_monitor \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Integration Guide

### RtControllerNode에서의 통합 (v5.8.0)

`ur5e_rt_controller` 패키지의 launch 파일이 본 패키지의 설정을 자동으로 로드합니다:

```python
# ur_control.launch.py
status_monitor_config = PathJoinSubstitution([
    FindPackageShare('ur5e_status_monitor'),
    'config', 'ur5e_status_monitor.yaml'
])

rt_controller_node = Node(
    parameters=[ur_control_config, status_monitor_config, {
        'log_dir': session_dir,
        'status_monitor.log_output_dir': os.path.join(session_dir, 'monitor'),
    }],
    ...
)
```

**설정 구조:**
- `enable_status_monitor` — `ur5e_rt_controller.yaml`에서 활성화/비활성화 제어
- `status_monitor.*` — `ur5e_status_monitor.yaml`에서 모든 파라미터 관리 (단일 소스)

### 커스텀 노드에 직접 통합

```cpp
#include "ur5e_status_monitor/ur5e_status_monitor.hpp"

// In your existing node (after construction, when shared_ptr is valid)
auto node = std::make_shared<YourNode>();
auto monitor = std::make_unique<ur5e_status_monitor::UR5eStatusMonitor>(node);

// Register callbacks
monitor->registerOnFailure([&](auto type, auto& ctx) {
  RCLCPP_ERROR(node->get_logger(), "Failure: %s",
               std::string(ur5e_status_monitor::FailureTypeToString(type)).c_str());
  stopRTControlLoop();
});

monitor->registerOnReady([&]() {
  RCLCPP_INFO(node->get_logger(), "Robot ready — starting control loop");
});

// Add callback group to an executor
aux_executor.add_callback_group(
    monitor->GetCallbackGroup(), node->get_node_base_interface());

// Start monitoring
monitor->start();
monitor->waitForReady(10.0);
startRTControlLoop();

// In RT control loop (500 Hz)
if (!monitor->isReady()) return;
if (monitor->getFailure() != ur5e_status_monitor::FailureType::kNone) {
  triggerEstop();
  return;
}
monitor->setJointReference(q_ref, qd_ref);
```

---

## Parameters Reference

All parameters are declared under the `status_monitor.` prefix.
설정 파일: `ur5e_status_monitor/config/ur5e_status_monitor.yaml` (단일 소스)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `joint_states_topic` | string | `/joint_states` | Joint state subscription topic |
| `robot_mode_topic` | string | `/io_and_status_controller/robot_mode` | Robot mode topic (Int32) |
| `safety_mode_topic` | string | `/io_and_status_controller/safety_mode` | Safety mode topic (Int32) |
| `program_running_topic` | string | `/io_and_status_controller/robot_program_running` | Program running topic (Bool) |
| `watchdog_timeout_sec` | double | 1.0 | /joint_states receive timeout |
| `controller_poll_interval_sec` | double | 5.0 | Controller manager poll interval |
| `target_controller` | string | `scaled_joint_trajectory_controller` | Controller to monitor |
| `tracking_error_pos_warn_rad` | double | 0.05 | Position tracking error warning |
| `tracking_error_pos_fault_rad` | double | 0.15 | Position tracking error fault |
| `tracking_error_vel_warn_rad` | double | 0.1 | Velocity tracking error warning |
| `tracking_error_vel_fault_rad` | double | 0.3 | Velocity tracking error fault |
| `joint_limit_warn_margin_deg` | double | 5.0 | Joint limit warning margin |
| `joint_limit_fault_margin_deg` | double | 1.0 | Joint limit fault margin |
| `auto_recovery` | bool | false | Enable auto-recovery |
| `max_recovery_attempts` | int | 3 | Max recovery attempts |
| `recovery_interval_sec` | double | 5.0 | Recovery cooldown |
| `log_output_dir` | string | `""` | 실패 로그 출력 디렉토리 (빈값: `UR5E_SESSION_DIR/monitor/` → `~/.ros` 폴백) |
| `enable_controller_stats` | bool | true | Per-controller statistics tracking |

---

## Topics & Services

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | Robot status diagnostics (includes `joint_state_rate`, `joint_state_total`, `joint_state_timeouts`, `active_controller` key-values) |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Joint positions and velocities |
| `/io_and_status_controller/robot_mode` | `std_msgs/Int32` | UR robot mode |
| `/io_and_status_controller/safety_mode` | `std_msgs/Int32` | UR safety mode |
| `/io_and_status_controller/robot_program_running` | `std_msgs/Bool` | Program running state |
| `/rt_controller/active_controller_name` | `std_msgs/String` | Active controller name (transient_local QoS) |

### Used Services

| Service | Type | When |
|---------|------|------|
| `/controller_manager/list_controllers` | `controller_manager_msgs/ListControllers` | Every 5 s (configurable) |

---

## Failure Types Reference

| FailureType | Trigger Condition | Auto-Recovery | Log Generated |
|-------------|-------------------|---------------|---------------|
| `kEstop` | Emergency stop detected | No | Yes |
| `kProtectiveStop` | SafetyMode == PROTECTIVE_STOP | Yes (if enabled) | Yes |
| `kSafetyViolation` | SafetyMode == VIOLATION/FAULT | No | Yes |
| `kProgramDisconnected` | Program stopped running | Yes (if enabled) | Yes |
| `kWatchdogTimeout` | /joint_states timeout > threshold | No | Yes |
| `kControllerInactive` | Target controller not active | No | Yes |
| `kHardwareFault` | Hardware error detected | No | Yes |
| `kTrackingError` | Position/velocity error > fault threshold | No | Yes |
| `kJointLimitViolation` | Joint within fault margin of limit | No | Yes |

---

## Failure Log Format

When a failure is detected, a log file is written to `{log_output_dir}/ur5e_failure_<timestamp>.log`. v5.10.0부터 기본 경로는 `UR5E_SESSION_DIR/monitor/`입니다 (launch 파일이 세션 디렉토리를 자동 설정).

### Example

```
[HEADER]
Timestamp:        20260313T143022_512
FailureType:      TRACKING_ERROR
Description:      Joint 2 position tracking error: 0.162341 rad (threshold: 0.15 rad)
RobotMode:        7 (RUNNING)
SafetyMode:       1 (NORMAL)

[STATE AT FAILURE]
q_actual:         [-0.123456, 0.234567, 1.345678, -0.456789, 0.567890, -0.678901] rad
qd_actual:        [0.001234, -0.002345, 0.003456, -0.004567, 0.005678, -0.006789] rad/s
q_reference:      [-0.123456, 0.234567, 1.508019, -0.456789, 0.567890, -0.678901] rad
tracking_error:   [0.000000, 0.000000, 0.162341, 0.000000, 0.000000, 0.000000] rad

[STATE HISTORY — last 10 seconds at 10Hz]
timestamp_ms, robot_mode, safety_mode, q[0], q[1], q[2], q[3], q[4], q[5], qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], tracking_err[0], tracking_err[1], tracking_err[2], tracking_err[3], tracking_err[4], tracking_err[5], joint_limit_warning
1710336622512, 7, 1, -0.123456, 0.234567, 1.345678, -0.456789, 0.567890, -0.678901, 0.001234, -0.002345, 0.003456, -0.004567, 0.005678, -0.006789, 0.000000, 0.000000, 0.162341, 0.000000, 0.000000, 0.000000, 0
...

[CONTROLLER STATE AT FAILURE]
scaled_joint_trajectory_controller: active
```

### Sections

- **[HEADER]**: Failure metadata — timestamp, type, description, robot/safety mode
- **[STATE AT FAILURE]**: Joint positions, velocities, references, and tracking errors at failure time
- **[STATE HISTORY]**: CSV of the last 10 seconds (100 entries at 10 Hz) of state snapshots
- **[CONTROLLER STATE AT FAILURE]**: Status of the monitored controller

---

## License

MIT License. See [LICENSE](LICENSE) for details.
