# Testing & Debugging

## Sensor Matrix (변경 유형별 필수 검증)

[CLAUDE.md](../CLAUDE.md) §5의 상세판. 변경 위치에 따라 필수 sensor + 추가 sensor를 실행한다.

| 변경 위치 | 필수 Sensor | 추가 Sensor |
|----------|------------|------------|
| `rtc_base/` | `colcon test --packages-select rtc_base` | 전체 downstream ([invariants.md](invariants.md) PROC-3) |
| `rtc_math/` | `colcon test --packages-select rtc_math` (Pinocchio 발견 시 `test_se3_module` log6/Jlog6 교차검증 + 유한차분) | `se3_error_compare` S1–S5 실험 + `plot_se3_compare.py` (선택) |
| `rtc_msgs/` | 위 + `./build.sh full` (msg gen 전파) | downstream pub/sub 테스트 |
| `rtc_controller_interface/` | `test_core_controllers` + controller registry tests | downstream controller 빌드 |
| `rtc_controllers/` RT path | `test_core_controllers` + grasp 관련 gtest | RT scheduling 확인 (`ps -eLo cls,rtprio`) |
| `rtc_controllers/` gains/config | 위 + 해당 controller YAML 로드 smoke | `ros2 topic echo /rtc_cm/active_controller_name` |
| `rtc_controller_manager/` | RT loop timing (`/system/estop_status`) | (MPC CSV는 `<session>/controllers/<config_key>/...` 경로로 컨트롤러가 자체 기록) |
| `rtc_tsid/` | QP/task/constraint gtest | TSID performance tests |
| `rtc_mpc/` | gtest (types, TripleBuffer, Riccati, SolutionManager) | `mpc_timing_log.csv` p50/p99/max 회귀 |
| `rtc_mujoco_sim/` | gtest (parse, lifecycle, solver, I/O, contact_wrench) | `ros2 launch integrated_bringup sim_ur5e_p1a.launch.py` smoke. Contact wrench: `ros2 topic hz /<prefix>/<target>/contact_wrench` 후 fingertip 으로 객체 접촉 → magnitude 가시화 |
| `rtc_urdf_bridge/` | gtest (URDF/model parsing, xacro, chain extractor) | 실제 URDF 파싱 smoke |
| `rtc_inference/` | ONNX engine unit test | 실제 모델 로드 smoke |
| `rtc_communication/` | UDP loopback + CAN/CANFD loopback (vcan0 없으면 skip) + RS485 serial loopback (PTY, 상시 실행) + Transceiver lifecycle/decode/callback | vcan0 셋업 후 CAN 테스트 실행 (`sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0`), RS485는 PTY라 셋업 불필요, 실제 HW UDP/CAN/RS485(USB-RS485+Dynamixel) 테스트 (선택) |
| `rtc_digital_twin/` | pytest + RViz2 smoke | `/rtc_cm/{group}/joint_states` hz |
| `rtc_tools/` (pytest) | pytest | GUI/plot 수동 smoke |
| `repo_scripts/` | `test_rt_common` + shell unit test | `check_rt_setup.sh --summary` |
| `shape_estimation*/` | ToF + exploration gtest | `/shape_estimation/snapshot` topic echo |
| `integrated_bringup/` demo FSM | demo_wbc FSM/integration/output + grasp_phase_manager + virtual_tcp | BT coordinator 통합 |
| `udp_hand_driver/` | 단위 gtest (hand_packets, codec, FT, failure detector) + UDP loopback | `ros2 topic hz /p1a/joint_states` (ur5e_p1a; 드라이버 standalone 기본은 `/hand/`) |
| `ur5e_bt_coordinator/` | BT gtest (tree_validation, condition_nodes, hand_nodes 등) | 실제 grasp 시나리오 smoke |
| Launch / YAML | `ros2 launch ... --print` + 짧은 smoke | config 로드 검증 |
| Threading (`ApplyThreadConfig`) | `rtc_base` thread-config gtest + RT perms | `check_rt_setup.sh --summary` |
| RT 회귀 의심 / RT path 미상 | `mpc_timing_log.csv`·`cm_timing_log.csv` p99 검사 | `enable_tracing:=true` + Perfetto 분석 (§Tracing) |
| RT host 환경 검증 | `check_rt_setup.sh --summary` + `verify_rt_runtime.sh` | `cyclictest --mlockall --smp -p 80 -i 200` / `rtla osnoise top` — [invariants.md](invariants.md) §RT Host / Runtime Preconditions RT-HOST-1~3 |

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

## Test fixtures — robot URDF 해석

robot 모델이 필요한 gtest fixture 는 URDF 를 **`robot_descriptions/robots/<name>/`** (repo 체크인) 에서 해석한다. **`deps/src/...` 나 `/usr/local/...` 경로를 박지 말 것** — `build-isolated-deps` 가 CI 아티팩트에서 `deps/src` 를 삭제하고 (`deps/install` 만 업로드), `/usr/local` 은 deps 격리 정책상 부재라, 두 경로 모두 CI 에서 fixture `buildModel` throw 또는 `GTEST_SKIP` → 0 coverage 를 유발한다 (rtc_tsid/rtc_mpc/integrated_bringup 에서 실제 발현, panda.urdf vendor 로 해소). 해석 방식: compile macro (`RTC_PANDA_URDF_PATH`, CMake 가 repo-상대 `robot_descriptions` 경로 주입, env/`-D` override) 또는 `ament_index_cpp::get_package_share_directory("robot_descriptions")`.

## Test 측정

테스트 카운트·suite 목록은 박제하지 않는다 ([anti-patterns.md](anti-patterns.md) AP-DOC-1). 최신 카운트·suite 명은 직접 측정:

```bash
colcon test --packages-select <pkg> --event-handlers console_direct+
colcon test-result --verbose
```

대표 suite 명은 `<pkg>/CMakeLists.txt` 에서 `ament_add_gtest()` / `ament_add_pytest_test()` grep — 코드 자체가 SSoT 이므로 문서 박제 불필요.

### Coverage 측정 (gcov/gcovr)

build.sh wrapper 는 없고, runtime PC 에 `lcov`/`gcovr` 가 없을 수 있다 (`gcov` 만 존재). gcovr 은 venv 에 설치 (`pip` 부재 — venv 는 `uv` 기반):

```bash
source repo_scripts/scripts/setup_env.sh
uv pip install gcovr                           # 분석 전용 도구 — runtime 에 영향 없음
# coverage 빌드: 패키지 CMakeLists 의 ENABLE_COVERAGE 옵션 사용 (--coverage 플래그 주입)
colcon build --packages-select <pkg> --cmake-args -DENABLE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug
find build/<pkg> -name '*.gcda' -delete         # 누적 카운트 초기화 (정확한 측정)
colcon test  --packages-select <pkg>
SRC=src/rtc-framework/<pkg>
gcovr --root "$SRC" --object-directory build/<pkg> --filter "$SRC/src/" --print-summary
# 측정 후: clean Release 재빌드로 install tree 원복 (coverage 빌드가 install 을 덮음 →
# downstream 이 gcov 심볼을 링크하게 됨)
rm -rf build/<pkg> install/<pkg> && colcon build --packages-select <pkg>
```

> `ENABLE_COVERAGE` 옵션은 패키지 CMakeLists 에 개별 정의한다 (보유 패키지: `grep -rl ENABLE_COVERAGE src/rtc-framework/*/CMakeLists.txt`). 미보유 패키지 측정 시 동일 `option(ENABLE_COVERAGE ... OFF)` + `add_compile_options(--coverage ...)` 블록을 추가한다. header-only 패키지는 `--filter "$SRC/include/"`, src 기반은 `--filter "$SRC/src/"`.

> **Python (ament_python) 패키지는 gcov/gcovr 가 아니라 `coverage`** (venv 설치, `uv pip install coverage`): `gcovr` 은 C++ 전용이라 `.py` 를 계측하지 못한다. CMakeLists/`ENABLE_COVERAGE` 도 무관 (ament_python 은 CMake 없음 — colcon 이 `test/test_*.py` 자동 발견). 측정: `cd <pkg> && python3 -m coverage run --source=<pkg_module> -m pytest test/ -q && python3 -m coverage report -m`. 측정 후 `.coverage` 아티팩트 삭제. 보유 Python 패키지: `rtc_digital_twin`, `rtc_tools`.

> **`rtc_mpc` 측정 함정**: 위 recipe 의 `-DCMAKE_BUILD_TYPE=Debug` 를 `rtc_mpc` 에 그대로 쓰면 안 된다. (1) Debug 빌드가 proxsuite all-zero-C assert 를 표면화한다 (Release 는 NDEBUG 로 숨김 — `feedback_proxsuite_zero_c_rejection`). (2) aligator 0.19.0 의 contact_rich MPC 는 `LD_PRELOAD=$DEPS/install/lib/libmimalloc.so` 없이는 `free(): invalid pointer` 로 죽는다 (`feedback_aligator_requires_mimalloc`). `ENABLE_COVERAGE` 옵션 추가 자체(default OFF)는 무해하나, 실제 측정은 두 우회를 모두 적용해야 한다.

**LifecycleNode 노드(예: `rtc_controller_manager`) 유닛 커버리지 패턴**: `on_configure` 파이프라인 (params 로딩·device backend·publisher 생성) 과 private RT 헬퍼(`CheckTimeouts`/`CreateDeviceBackends`)는 friend accessor (`rtc::ControllerLifecycleTestAccess`, 헤더의 `friend` 선언으로 이름 고정) 로 private 멤버를 주입·호출해 **실 robot/RT 권한 없이** 구동한다 — `CreateDeviceBackends` 등은 `group_slot_map_`/`device_name_configs_` 주입 + registry fake backend 만으로 동작하고, `on_activate` 의 RT 루프는 `ApplyThreadConfig` 반환값을 버리므로 (SCHED_OTHER fallback) 테스트 샌드박스에서도 안전하다.

**`.venv` 격리 원칙 (Hard rule)**: `.venv`는 runtime PC가 본 workspace 외에 다른 control project들과 공존하는 환경에서 workspace dependency를 격리하기 위한 **의도된 설계**다. `colcon test` / `colcon build` / `ros2 run` / `ros2 launch`가 venv 활성 상태에서 실패하면 **반드시 근본 원인을 해결**한다 (sys.path / shebang / wrapper / dep resolution 디버그). gtest 바이너리 직접 실행, venv deactivate 후 colcon 호출, `PYTHONPATH` 강제 우회 등은 **금지** — 격리를 무력화해 runtime PC에서 다른 project의 site-packages가 끼어들면 silent breakage. 신호 (`Testing/Temporary/LastTest.log` Start/End 동일 초)가 재발하면 `env -i` 깨끗한 셸에서 `setup_env.sh` source 후 `sys.path` 순서 점검부터.

## Live Debug Topics

런타임 문제 탐지용 토픽. `ros2 topic echo` / `ros2 topic hz` / `ros2 bag record` 대상.

| Topic | 발행 주체 | 언제 보나 |
|-------|----------|----------|
| `/system/estop_status` | `rtc_controller_manager` | E-STOP 원인 파악 (timeout name / trigger thread) |
| `/rtc_cm/active_controller_name` | 동일 (TRANSIENT_LOCAL) | Controller switch 확인. BT / GUI / digital_twin은 이 토픽으로 리와이어 |
| `/<config_key>/<config_key>/get_parameters` (srv) | active 데모 컨트롤러의 LifecycleNode | Runtime gain 값 조회 (`ros2 param get`) |
| `/forward_position_controller/commands` | 동일 | RT loop 건강성 — `ros2 topic hz` 로 설정된 `control_rate` (default 500 Hz) 매칭 확인 |
| `<session>/timing/cm_timing_log.csv` | CM RT loop @ `control_rate` (`rtc::ThreadTimingProducer<RtTickTimingPayload>`) drained by `DrainLog()` log thread | RT loop per-tick timing — 7 cols `t_wall_ns,tick_count,t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us`. p50/p99 등은 post-process 계산. **Sim 모드 (`use_sim_time_sync=true`) 에서는 `jitter_us` 컬럼이 항상 0.0** — CV wakeup 이라 `\|actual_period − budget\|`이 sim cadence 잡음일 뿐 RT 지표가 아니기 때문 (`PeriodicRtThread::JitterMeaningful()` override). 다른 6개 컬럼은 robot/sim 동일 의미 |
| `<session>/timing/mpc_timing_log.csv` | per-controller LifecycleNode 1 Hz aux drains `MPCThread::TimingProducer()` | **Per-MPC-tick raw 샘플** — CM과 동일한 7-col 스키마 (RtTickTimingPayload). 한 row = 한 main-loop iteration. p50/p99/max는 post-process로 계산 (예: `awk` / pandas). aggregate INFO 라인은 controller 로그에 10 s마다 출력 (handler self-report `solve_duration_ns` 256-sample 윈도우). 두 CSV 모두 같은 generic infra + 동일 payload (`rtc_base/timing/rt_tick_timing_sample.hpp`) — 새 thread 추가 시 payload 재사용 |
| `/rtc_cm/{group}/joint_states` | CM (per-group, RELIABLE) | Device 그룹별 건강성; `rtc_digital_twin`이 merge |
| `/sim/status` | `rtc_mujoco_sim` 1 Hz | Sim 건강성 — 중단 시 sim sync timeout E-STOP |
| `<contact_wrench.topic_prefix>/<target>/contact_wrench` | `rtc_mujoco_sim` per-target (`mjSENS_CONTACT` netforce + world→link transform, env-on-link sign convention) | Fingertip 접촉 force/torque 확인. `RViz2 → WrenchStamped` display 또는 `ros2 topic echo`. 비접촉 시 0 발행. 활성화 조건: 그룹 YAML `contact_wrench.enabled: true` + MJCF 에 `<sensor><contact>` (data=`found force torque dist pos normal tangent` num=1 reduce=netforce) |
| `/p1a/joint_states`, `/p1a/motor_states`, `/p1a/sensor_states` | `udp_hand_driver` (ur5e_p1a; generic driver default `/hand/`) | Hand UDP 건강성 |
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

## Tracing

CSV timing logs (`cm_timing_log.csv` / `mpc_timing_log.csv` / `hand_udp_timing_log.csv`) 은 **per-tick 총 시간** 만 기록한다. 어느 thread 가 어느 core 에서 언제 run 했는지 / 어떤 callback 이 시간을 쓰는지 알아내려면 LTTng 트레이스를 캡처해 분석한다.

세부 명령·event 선택·permission 분기·뷰어 사용법은 [../docs/tracing.md](../docs/tracing.md) 참조 (ros2_tracing / babeltrace2 / Perfetto operational guide).

핵심 진입점만:

```bash
./install.sh --tracing                                              # 1회 setup
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true   # 캡처
./repo_scripts/scripts/timeline.sh                                  # Perfetto JSON 변환
```
