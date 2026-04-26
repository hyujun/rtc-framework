# Testing & Debugging

## Sensor Matrix (변경 유형별 필수 검증)

[CLAUDE.md](../CLAUDE.md#L87) §5의 상세판. 변경 위치에 따라 필수 sensor + 추가 sensor를 실행한다.

| 변경 위치 | 필수 Sensor | 추가 Sensor |
|----------|------------|------------|
| `rtc_base/` | `colcon test --packages-select rtc_base` | 전체 downstream ([invariants.md](invariants.md) PROC-3) |
| `rtc_msgs/` | 위 + `./build.sh full` (msg gen 전파) | downstream pub/sub 테스트 |
| `rtc_controller_interface/` | `test_core_controllers` + controller registry tests | downstream controller 빌드 |
| `rtc_controllers/` RT path | `test_core_controllers` + grasp 관련 gtest | RT scheduling 확인 (`ps -eLo cls,rtprio`) |
| `rtc_controllers/` gains/config | 위 + 해당 controller YAML 로드 smoke | `ros2 topic echo /active_controller_name` |
| `rtc_controller_manager/` | RT loop timing (`/system/estop_status`) | (MPC CSV는 `<session>/controllers/<config_key>/...` 경로로 컨트롤러가 자체 기록) |
| `rtc_tsid/` | QP/task/constraint gtest | TSID performance tests |
| `rtc_mpc/` | gtest (types, TripleBuffer, Riccati, SolutionManager) | `mpc_solve_timing.csv` p50/p99/max 회귀 |
| `rtc_mujoco_sim/` | gtest (parse, lifecycle, solver, I/O) | `ros2 launch ur5e_bringup sim.launch.py` smoke |
| `rtc_urdf_bridge/` | gtest (URDF/model parsing, xacro, chain extractor) | 실제 URDF 파싱 smoke |
| `rtc_inference/` | ONNX engine unit test | 실제 모델 로드 smoke |
| `rtc_communication/` | UDP loopback + Transceiver lifecycle/decode/callback | 실제 HW UDP 테스트 (선택) |
| `rtc_digital_twin/` | pytest + RViz2 smoke | `/{group}/digital_twin/joint_states` hz |
| `rtc_tools/` (pytest) | pytest | GUI/plot 수동 smoke |
| `rtc_scripts/` | `test_rt_common` + shell unit test | `check_rt_setup.sh --summary` |
| `shape_estimation*/` | ToF + exploration gtest | `/shape_estimation/snapshot` topic echo |
| `ur5e_bringup/` demo FSM | demo_wbc FSM/integration/output + grasp_phase_manager + virtual_tcp | BT coordinator 통합 |
| `ur5e_hand_driver/` | 단위 gtest (hand_packets, codec, FT, failure detector) + UDP loopback | `ros2 topic hz /hand/joint_states` |
| `ur5e_bt_coordinator/` | BT gtest (tree_validation, condition_nodes, hand_nodes 등) | 실제 grasp 시나리오 smoke |
| Launch / YAML | `ros2 launch ... --print` + 짧은 smoke | config 로드 검증 |
| Threading (`ApplyThreadConfig`) | `rtc_base` thread-config gtest + RT perms | `check_rt_setup.sh --summary` |

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

## Test Table (최근 실측: 2026-04-24, gtest CASES 단위)

| Package | Tests | Framework |
|---------|-------|-----------|
| `ur5e_hand_driver` | 179 C++ (hand_packets 46, hand_controller 33, hand_udp_codec 24, fingertip_ft 19, hand_udp_transport 18, hand_timing_profiler 15, hand_sensor_processor 13, hand_failure_detector 11) | GTest |
| `ur5e_bt_coordinator` | 157 C++ (condition_nodes 21, bt_utils 19, bt_types 16, hand_nodes 15, compute_offset_pose 12, shape_nodes 11, tree_validation 10, set_gains 9, set_pose_z 8, switch_controller 7 (4 base + 3 srv), grasp_control 7, compute_sweep_trajectory 6, move_to_joints 6, wait_duration 5, move_to_pose 5) | GTest |
| `rtc_mpc` | 136 C++ (types, TripleBuffer, TrajectoryInterpolator, Riccati, SolutionManager, thread skeleton, RobotModelHandler, PhaseCostConfig) | GTest |
| `rtc_urdf_bridge` | 89 C++ (urdf_analyzer 25, rt_model_handle 16, model_builder 15, chain_extractor 12, xacro_processor 11, joint_classification 10) | GTest |
| `ur5e_bringup` | 89 C++ (demo_wbc_controller 41, demo_shared_config 16, grasp_phase_manager 13, virtual_tcp 10, demo_wbc_mpc_integration 6, grasp_pipeline 3) | GTest |
| `rtc_controllers` | 87 C++ (test_core_controllers + grasp related) | GTest |
| `rtc_base` | 54 C++ (SeqLock, SPSC, Bessel/Kalman filters, session dir, thread-config tiers) | GTest |
| `rtc_controller_interface` | 51 C++ | GTest |
| `rtc_controller_manager` | 35 C++ (controller_lifecycle 9, switch_service 9, controller_timing_profiler 17) | GTest |
| `rtc_tools` | 247 Python (pytest parameterized) | pytest |

총 **1104 gtest cases + 247 pytest cases**. `rtc_tsid` · `rtc_mujoco_sim` · `rtc_communication` · `rtc_digital_twin` · `shape_estimation` 등은 등록되어 있으나 최근 실측 시점 `ament_cmake_test` 환경 이슈로 xUnit 미산출 — 이전 실행분 기준 17/77/9/1/3 C++ tests (검증 필요).

## Live Debug Topics

런타임 문제 탐지용 토픽. `ros2 topic echo` / `ros2 topic hz` / `ros2 bag record` 대상.

| Topic | 발행 주체 | 언제 보나 |
|-------|----------|----------|
| `/system/estop_status` | `rtc_controller_manager` | E-STOP 원인 파악 (timeout name / trigger thread) |
| `/active_controller_name` | 동일 (TRANSIENT_LOCAL) | Controller switch 확인. BT / GUI / digital_twin은 이 토픽으로 리와이어 |
| `/current_gains` | 동일 | Runtime gain update 검증 |
| `/forward_position_controller/commands` | 동일 | RT loop 건강성 — `ros2 topic hz` 로 ~500 Hz 확인 |
| `<session>/controllers/<config_key>/mpc_solve_timing.csv` | per-controller LifecycleNode 1 Hz aux (e.g. `DemoWbcController`) | MPC solve p50/p99/max 회귀 — 9 cols `t_wall_ns,count,window,last_ns,min_ns,p50_ns,p99_ns,max_ns,mean_ns` |
| `/{group}/digital_twin/joint_states` | controllers (per-group, RELIABLE) | Device 그룹별 건강성; `rtc_digital_twin`이 merge |
| `/sim/status` | `rtc_mujoco_sim` 1 Hz | Sim 건강성 — 중단 시 sim sync timeout E-STOP |
| `/hand/joint_states`, `/hand/motor_states`, `/hand/sensor_states` | `ur5e_hand_driver` | Hand UDP 건강성 |
| `/shape_estimation/snapshot` (action feedback) | `shape_estimation` | ToF 기반 추정 진행 상황 |

## Debugging

| Symptom | Fix |
|---------|-----|
| `ApplyThreadConfig()` warns | `sudo usermod -aG realtime $USER` + re-login |
| E-STOP on startup | Set `init_timeout_sec: 0.0` for sim |
| High jitter (>200us) | Check `taskset` pinning, verify `isolcpus`, `check_rt_setup.sh --summary` |
| Hand timeout E-STOP | Check UDP link, `recv_timeout_ms: 0.4` |
| Controller not found | Use config_key (e.g. "p_controller") or Name() |
| `ament_cmake_test` missing on `colcon test` | `.venv` overlay가 system site-packages를 가림. 재활성화 + ROS 2 환경 재로드 |

```bash
# exec name = ROS node name = "ur5e_rt_controller" (only exec from ur5e_bringup;
# rtc_controller_manager is library-only). Use the same name for pgrep and
# lifecycle calls:
#   ros2 lifecycle list /ur5e_rt_controller
PID=$(pgrep -f ur5e_rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
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
