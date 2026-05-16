# Testing & Debugging

## Sensor Matrix (변경 유형별 필수 검증)

[CLAUDE.md](../CLAUDE.md) §5의 상세판. 변경 위치에 따라 필수 sensor + 추가 sensor를 실행한다.

| 변경 위치 | 필수 Sensor | 추가 Sensor |
|----------|------------|------------|
| `rtc_base/` | `colcon test --packages-select rtc_base` | 전체 downstream ([invariants.md](invariants.md) PROC-3) |
| `rtc_msgs/` | 위 + `./build.sh full` (msg gen 전파) | downstream pub/sub 테스트 |
| `rtc_controller_interface/` | `test_core_controllers` + controller registry tests | downstream controller 빌드 |
| `rtc_controllers/` RT path | `test_core_controllers` + grasp 관련 gtest | RT scheduling 확인 (`ps -eLo cls,rtprio`) |
| `rtc_controllers/` gains/config | 위 + 해당 controller YAML 로드 smoke | `ros2 topic echo /rtc_cm/active_controller_name` |
| `rtc_controller_manager/` | RT loop timing (`/system/estop_status`) | (MPC CSV는 `<session>/controllers/<config_key>/...` 경로로 컨트롤러가 자체 기록) |
| `rtc_tsid/` | QP/task/constraint gtest | TSID performance tests |
| `rtc_mpc/` | gtest (types, TripleBuffer, Riccati, SolutionManager) | `mpc_timing_log.csv` p50/p99/max 회귀 |
| `rtc_mujoco_sim/` | gtest (parse, lifecycle, solver, I/O) | `ros2 launch integrated_bringup sim.launch.py` smoke |
| `rtc_urdf_bridge/` | gtest (URDF/model parsing, xacro, chain extractor) | 실제 URDF 파싱 smoke |
| `rtc_inference/` | ONNX engine unit test | 실제 모델 로드 smoke |
| `rtc_communication/` | UDP loopback + Transceiver lifecycle/decode/callback | 실제 HW UDP 테스트 (선택) |
| `rtc_digital_twin/` | pytest + RViz2 smoke | `/rtc_cm/{group}/joint_states` hz |
| `rtc_tools/` (pytest) | pytest | GUI/plot 수동 smoke |
| `repo_scripts/` | `test_rt_common` + shell unit test | `check_rt_setup.sh --summary` |
| `shape_estimation*/` | ToF + exploration gtest | `/shape_estimation/snapshot` topic echo |
| `integrated_bringup/` demo FSM | demo_wbc FSM/integration/output + grasp_phase_manager + virtual_tcp | BT coordinator 통합 |
| `udp_hand_driver/` | 단위 gtest (hand_packets, codec, FT, failure detector) + UDP loopback | `ros2 topic hz /hand/joint_states` |
| `ur5e_bt_coordinator/` | BT gtest (tree_validation, condition_nodes, hand_nodes 등) | 실제 grasp 시나리오 smoke |
| Launch / YAML | `ros2 launch ... --print` + 짧은 smoke | config 로드 검증 |
| Threading (`ApplyThreadConfig`) | `rtc_base` thread-config gtest + RT perms | `check_rt_setup.sh --summary` |
| RT 회귀 의심 / hot path 미상 | `mpc_timing_log.csv`·`cm_timing_log.csv` p99 검사 | `enable_perf:=true` + Hotspot 분석 (§Profiling) |

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

## Test 측정

테스트 카운트·suite 목록은 박제하지 않는다 ([anti-patterns.md](anti-patterns.md) AP-DOC-1). 최신 카운트·suite 명은 직접 측정:

```bash
colcon test --packages-select <pkg> --event-handlers console_direct+
colcon test-result --verbose
```

대표 suite 명은 `<pkg>/CMakeLists.txt` 에서 `ament_add_gtest()` / `ament_add_pytest_test()` grep — 코드 자체가 SSoT 이므로 문서 박제 불필요.

**`.venv` 격리 원칙 (Hard rule)**: `.venv`는 runtime PC가 본 workspace 외에 다른 control project들과 공존하는 환경에서 workspace dependency를 격리하기 위한 **의도된 설계**다. `colcon test` / `colcon build` / `ros2 run` / `ros2 launch`가 venv 활성 상태에서 실패하면 **반드시 근본 원인을 해결**한다 (sys.path / shebang / wrapper / dep resolution 디버그). gtest 바이너리 직접 실행, venv deactivate 후 colcon 호출, `PYTHONPATH` 강제 우회 등은 **금지** — 격리를 무력화해 runtime PC에서 다른 project의 site-packages가 끼어들면 silent breakage. 신호 (`Testing/Temporary/LastTest.log` Start/End 동일 초)가 재발하면 `env -i` 깨끗한 셸에서 `setup_env.sh` source 후 `sys.path` 순서 점검부터.

## Live Debug Topics

런타임 문제 탐지용 토픽. `ros2 topic echo` / `ros2 topic hz` / `ros2 bag record` 대상.

| Topic | 발행 주체 | 언제 보나 |
|-------|----------|----------|
| `/system/estop_status` | `rtc_controller_manager` | E-STOP 원인 파악 (timeout name / trigger thread) |
| `/rtc_cm/active_controller_name` | 동일 (TRANSIENT_LOCAL) | Controller switch 확인. BT / GUI / digital_twin은 이 토픽으로 리와이어 |
| `/<config_key>/<config_key>/get_parameters` (srv) | active 데모 컨트롤러의 LifecycleNode | Runtime gain 값 조회 (`ros2 param get`) |
| `/forward_position_controller/commands` | 동일 | RT loop 건강성 — `ros2 topic hz` 로 설정된 `control_rate` (default ~500 Hz) 매칭 확인 |
| `<session>/timing/cm_timing_log.csv` | CM RT loop @ `control_rate` (`rtc::ThreadTimingProducer<RtTickTimingPayload>`) drained by `DrainLog()` log thread | RT loop per-tick timing — 7 cols `t_wall_ns,tick_count,t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us`. p50/p99 등은 post-process 계산. **Sim 모드 (`use_sim_time_sync=true`) 에서는 `jitter_us` 컬럼이 항상 0.0** — CV wakeup 이라 `\|actual_period − budget\|`이 sim cadence 잡음일 뿐 RT 지표가 아니기 때문 (`PeriodicRtThread::JitterMeaningful()` override). 다른 6개 컬럼은 robot/sim 동일 의미 |
| `<session>/timing/mpc_timing_log.csv` | per-controller LifecycleNode 1 Hz aux drains `MPCThread::TimingProducer()` | **Per-MPC-tick raw 샘플** — CM과 동일한 7-col 스키마 (RtTickTimingPayload). 한 row = 한 main-loop iteration. p50/p99/max는 post-process로 계산 (예: `awk` / pandas). aggregate INFO 라인은 controller 로그에 10 s마다 출력 (handler self-report `solve_duration_ns` 256-sample 윈도우). 두 CSV 모두 같은 generic infra + 동일 payload (`rtc_base/timing/rt_tick_timing_sample.hpp`) — 새 thread 추가 시 payload 재사용 |
| `/rtc_cm/{group}/joint_states` | CM (per-group, RELIABLE) | Device 그룹별 건강성; `rtc_digital_twin`이 merge |
| `/sim/status` | `rtc_mujoco_sim` 1 Hz | Sim 건강성 — 중단 시 sim sync timeout E-STOP |
| `/hand/joint_states`, `/hand/motor_states`, `/hand/sensor_states` | `udp_hand_driver` | Hand UDP 건강성 |
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
# exec name = ROS node name = "integrated_rt_controller" (only exec from integrated_bringup;
# rtc_controller_manager is library-only). Use the same name for pgrep and
# lifecycle calls:
#   ros2 lifecycle list /integrated_rt_controller
PID=$(pgrep -f integrated_rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
ros2 topic hz /forward_position_controller/commands
ros2 topic echo /system/estop_status
./repo_scripts/scripts/check_rt_setup.sh --summary
```

## RT Permissions

```bash
sudo groupadd realtime && sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# Re-login required. Optional: isolcpus, nohz_full, or cpu_shield.sh
```

## Profiling

CSV timing logs (`cm_timing_log.csv` / `mpc_timing_log.csv` / `hand_udp_timing_log.csv`) 은 **per-tick 총 시간** 만 기록한다. 어떤 함수가 시간을 쓰는지 알아내려면 sampling profile 을 캡처해 분석한다.

세부 명령·뷰어 사용법·sampling event 선택·permission 분기는 [../docs/profiling.md](../docs/profiling.md) 참조 (perf / Hotspot / Perfetto operational guide).

핵심 진입점만:

```bash
ros2 launch integrated_bringup sim.launch.py enable_perf:=true   # 캡처
./repo_scripts/scripts/flame.sh                                  # flame graph (권장, 1-2초)
./repo_scripts/scripts/timeline.sh                               # Perfetto timeline (thread/CPU swimlane)
```
