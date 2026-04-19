# Testing & Debugging

## Test Commands

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

## Test Table (239 total)

| Package | Tests | Framework |
|---------|-------|-----------|
| `rtc_mujoco_sim` | 77 C++ tests (parse helpers, simulator lifecycle, solver priority, command/state I/O, runtime controls) | GTest |
| `rtc_mpc` | 69 C++ tests (types, TripleBuffer stress, interpolation, Riccati, solution manager, thread skeleton, RobotModelHandler, PhaseCostConfig YAML factory) | GTest |
| `ur5e_bringup` | 29 C++ tests (virtual_tcp, shared_config, demo_wbc FSM/integration/output, MPC binding) | GTest |
| `rtc_base` | 24 C++ tests (SeqLock, SPSC, Bessel/Kalman filters, session dir, thread-config tiers) | GTest |
| `rtc_tsid` | 17 C++ tests (QP solver, tasks, constraints, formulations, performance) | GTest |
| `ur5e_bt_coordinator` | 14 C++ tests | GTest |
| `rtc_communication` | 9 C++ tests (UDP loopback, Transceiver lifecycle/decode/callback) | GTest |
| `rtc_controllers` | 6 C++ tests (trajectory + grasp) | GTest |
| `rtc_urdf_bridge` | 5 C++ tests (URDF/model parsing) | GTest |
| `shape_estimation` | 3 C++ tests (ToF + exploration) | GTest |
| `rtc_digital_twin` | 1 Python test | pytest |
| `rtc_tools` | 1 Python test | pytest |

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

## RT Permissions

```bash
sudo groupadd realtime && sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# Re-login required. Optional: isolcpus, nohz_full, or cpu_shield.sh
```
