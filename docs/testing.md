# Testing & Debugging

이 문서는 테스트를 **돌리는 법**과 런타임에서 **살펴보는 곳**을 모은 실무 가이드다. 무엇을 반드시 돌리고 결과를 어떻게 판정하는지(규범)는 [agent_docs/testing-debug.md](../agent_docs/testing-debug.md) 가 갖고, 함정이 실제로 발현한 사례와 근거는 [testing-rationale.md](reference/testing-rationale.md) 에 있다.

## Test Commands

테스트는 기본으로 빌드되지 않는다. 먼저 `./build.sh --tests [-p <pkg>]` 로 빌드한다 (테스트 없이 빌드한 패키지는 실패가 아니라 "0 tests" 를 보고한다). 직접 `colcon` 을 칠 때는 workspace root 에서 환경을 source 한 뒤 실행한다 ([AGENTS.md](../AGENTS.md) §9.1).

```bash
colcon test --event-handlers console_direct+                      # 전체
colcon test-result --verbose
colcon test --packages-select ur5e_bt_coordinator --event-handlers console_direct+   # 패키지 하나
colcon test --packages-select rtc_controllers --ctest-args -R test_grasp_controller  # C++ 테스트 하나
colcon test --packages-select rtc_digital_twin --pytest-args -k test_urdf_parser     # Python 테스트 하나
PYTHONPATH=rtc_tools:$PYTHONPATH <rtc_ws>/.venv/bin/python -m pytest <pkg>/test/<file>.py  # venv 전용 pytest 직접 (repo 루트에서)
```

colcon 의 pytest 는 시스템 python 이라 `.venv` 에만 있는 의존성 (mujoco 등) 의 테스트는 skip 된다 — 그 파일은 마지막 줄로 repo 루트에서 직접 돌린다. `PYTHONPATH` 는 덮어쓰지 말고 이어붙이고, `rtc_tools` 를 앞에 두는 이유는 [testing-rationale.md](reference/testing-rationale.md#venv-격리의-false-green) 에 있다.

`colcon.pkg` 로 패키지 안 병렬 (`ctest-args: ["-j", n]`) 을 켠 패키지는 CLI 의 `-j 1` 로 직렬이 되지 않는다 (`colcon.pkg` 의 `-j` 가 뒤에 붙는다). 직렬로 돌리려면 `COLCON_EXTENSION_BLOCKLIST=colcon_core.package_augmentation.colcon_pkg colcon test …` 로 `colcon.pkg` 를 읽지 않게 한다. `ament_python` 패키지의 병렬 (pytest-xdist) 과 xdist 가 없는 호스트의 처리는 `rtc_tools/colcon.pkg` 주석이 갖는다.

## Sensor Matrix

변경 위치를 찾아 **필수 + 추가** 를 모두 돌린다 ([AGENTS.md](../AGENTS.md) §5). 기본형은 `colcon test --packages-select <pkg>` 이고, 표의 "필수" 는 그 안에서 반드시 봐야 하는 테스트와 추가 명령이다. 각 테스트가 무엇을 판별하는지는 그 테스트 파일의 머리 주석이 갖고, 행에 걸친 설계 원칙과 sanitizer 빌드의 예외는 [testing-rationale.md](reference/testing-rationale.md#sensor-matrix-보충) 에 있다.

| 변경 위치 | 필수 | 추가 |
|---|---|---|
| `rtc_base/` | `rtc_base` 전체 (`test_rt_heap` 은 mallopt 가 프로세스 전역이라 별도 바이너리) | 전체 downstream |
| `rtc_math/` | `rtc_math` 전체 (Pinocchio 가 있으면 `test_se3_module` 교차검증 포함) | `se3_error_compare` 실험 + `plot_se3_compare.py` (선택) |
| `rtc_msgs/` | 위 + `./build.sh --tests full` (msg 생성 전파) | downstream pub/sub 테스트 |
| `rtc_controller_interface/` | 패키지 전체 (registry · interface · mailbox · output-validation) | downstream controller 빌드 |
| `rtc_controllers/` RT path | `test_*_core` + `test_dls_convergence` (할당 게이트 무장) + grasp 관련 gtest | `ps -eLo cls,rtprio` 로 RT scheduling 확인 |
| `rtc_controllers/` gains · config | 위 + controller YAML 로드 smoke | `ros2 topic echo /rtc_cm/active_controller_name` |
| `rtc_controllers/` catching core | `--ctest-args -R test_catching` | ASan/UBSan 은 별도 build base (rationale 참조) |
| `rtc_controller_manager/` | RT loop timing (`/system/estop_status`) · teardown (`test_eventfd_teardown_order` · `test_state_lane_teardown`) | `cm_timing_log.csv` p99 |
| `rtc_tsid/` · `rtc_mpc/` | QP · task · constraint gtest · MPC gtest (Riccati · SolutionManager) | TSID performance · `mpc_timing_log.csv` p99 회귀 |
| `rtc_mujoco_sim/` | 패키지 전체 + `test_motor_servo_gains` · `test_torque_mode_ctrl_range` | `ros2 launch integrated_bringup sim_ur5e_p1a.launch.py` smoke |
| `rtc_urdf_bridge/` | 패키지 전체 + `test_frame_jacobian_fd_oracle` · `test_inertial_validation` · `test_real_model_inertial_gate` | 실제 robot 모델 로드 smoke |
| `rtc_communication/` · `rtc_inference/` · `rtc_digital_twin/` | UDP · CAN (vcan0 없으면 skip) · RS485 PTY · ONNX · pytest | vcan0 설정 후 CAN, 실제 모델 · RViz2 smoke |
| `rtc_tools/` | pytest. MJCF/URDF 를 고쳤으면 `test_real_model_pairs.py` 가 게이트 (colcon 에서 skip 될 수 있음) | venv 직접 실행 ([Test Commands](#test-commands)) |
| `repo_scripts/` · `.claude/hooks/` | `test_rt_common` · `test_install_deps` · `test_install_python` · `test_build_jobs` · `test_can_setup` + shell unit test. hook · `validate_test_*.py` · 그 import 를 고쳤으면 `bash repo_scripts/test/test_verify_changes.sh` (CI 전용 — `colcon test` 도 `--run` 도 돌리지 않는다) | `check_rt_setup.sh --summary` |
| `shape_estimation*/` | ToF + exploration gtest | `/shape/estimate` echo, `/shape/explore` 액션 |
| `integrated_bringup/` demo FSM | demo_wbc · demo_joint · demo_task · grasp_phase_manager · virtual_tcp gtest | BT coordinator 통합 |
| `integrated_bringup/` demo_joint 군 0 · dualarm | `test_joint_tree_primary_group` · `test_demo_dualarm_controller` · `test_demo_dualarm_alloc` | sim_g1_p1b 로 launch 후 GUI 확인 |
| `integrated_bringup/` demo GUI · profile | `test_demo_gui_dualarm` · `test_robot_profiles` | `demo_controller_gui --robot g1_p1b` (Tk 배치는 pytest 가 못 본다) |
| `integrated_bringup/` FK · arm tip · hand sensor_layout | `test_hand_fk_wiring` · `test_arm_tip_resolution` · `test_hand_sensor_layout_gate` | `iiwa7_leap` sim 에서 손 목표 smoke |
| `integrated_bringup/` compliance · catching | `test_compliance_admittance_coupling` · `test_demo_catching_controller` (configure — 사슬 탐색 · 관절군 · D9′) · `test_demo_catching_alloc` · `test_catching_search_segment_g1` (G1 탐색 · 구간 · 전신 항 — G1 + 합성 손 `robots/p1b_surrogate` 위) | sim 외부 렌치 → `compliance_diag.csv`; 17 관절 시간 기록은 `RTC_WHOLEBODY_TIMING=1` 로 단독 실행 |
| `integrated_bringup/` inference · observer | `test_demo_inference_controller` · `test_momentum_observer_wiring` · `test_payload_estimator` | `test_demo_inference_real_model` (로컬) |
| `integrated_bringup/` device backend (state lane) | `test_mujoco_native_backend` · `test_udp_hand_native_backend` · `test_state_lane_close` (`CloseStateLane()` 뒤 6 lane 모두 불변 — lane 을 추가하면 여기에 케이스를 더한다) | sim 에서 `ros2 lifecycle set /integrated_rt_controller cleanup` 뒤 CM 로그에 `#941` ERROR 없음 |
| `udp_hand_driver/` | hand_packets · codec · FT · failure detector gtest + UDP loopback · CAN FD comm (`test_canfd_hand_comm`, vcan0 없으면 skip — skip 은 미검증) · lifecycle (proto2 configure 거부 · cleanup / on_error 뒤 재configure · stats 키 fixture · teardown 뒤 `failure_detected` 유지 · E-Stop 토픽 → zero-write 1회: `sil_mode=off` + 127.0.0.1 무응답 소켓 · 두 손 `hand_sides`: 토픽 접두 · E-Stop 하나 · 거부 · activate 롤백) · `test_hand_comm_loop` (손 순서 · 손 사이 E-Stop) | `ros2 topic hz /p1a/joint_states`; proto2 SIL: `ros2 launch udp_hand_driver udp_hand.launch.py sil_mode:=firmware protocol_version:=2` → `/hand/link_status` true · `/hand/sensor_states` 의 `contact_flag`; 두 손 SIL 은 `hand_sides:=right,left loop_rate_hz:=250 recv_timeout_ms:=0.9` → `/hand/right/link_status` · `/hand/left/link_status` true; 실기 proto2 는 `recv_timeout_ms:=0.9 hand_side:=<side>` (H1-F06 실측값, 출하 0.4 는 매 사이클 타임아웃). **vcan0 에서 proto2 SIL 과 `colcon test` 를 동시에 돌리지 않는다** (`test_canfd_hand_comm` 이 같은 노드 id 공간을 쓴다). fixture 가 케이스마다 REQ probe 로 듣고, SIL (`fake_hand_firmware` 의 응답이나 송신 중인 노드) 이 보이면 vcan 케이스를 전부 FAIL 시킨다 ("proto2 hand traffic on vcan0") |
| `ur5e_bt_coordinator/` | BT gtest | 실제 grasp smoke |
| Launch · YAML config | `--ctest-args -R 'test_launch_'` + 변경한 YAML 의 parse · 범위 | 짧은 sim/실기 smoke |
| Threading (`ApplyThreadConfig`) | `rtc_base` thread-config gtest + RT 권한 | `check_rt_setup.sh --summary` |
| RT 회귀 의심 | `mpc_timing_log.csv` · `cm_timing_log.csv` p99 | `enable_tracing:=true` + Perfetto ([tracing.md](tracing.md)) |
| RT host 환경 | `check_rt_setup.sh --summary` + `verify_rt_runtime.sh` | `cyclictest --mlockall --smp -p 80 -i 200` · `rtla osnoise top` |

새 test 파일은 패키지 `CMakeLists.txt` 에 등록해야 `colcon test` 가 돌린다 (`repo_scripts/scripts/validate_test_registration.py --list` 로 확인). ROS 노드를 만드는 테스트는 패키지당 전용 `ROS_DOMAIN_ID` 가 필요하다 (`repo_scripts/scripts/validate_test_domains.py --list` 가 현재 배정을 보여 준다). robot 모델이 필요한 fixture 는 repo 안의 `robot_descriptions/robots/<name>/` 만 참조한다 (`repo_scripts/scripts/validate_test_fixtures.py --list`).

## Live Debug Topics

런타임 문제 탐지용. `ros2 topic echo` / `ros2 topic hz` / `ros2 bag record` 대상이다.

| Topic | 발행 주체 | 언제 보나 |
|---|---|---|
| `/system/estop_status` | `rtc_controller_manager` | E-STOP 원인 (timeout name / trigger thread) |
| `/rtc_cm/active_controller_name` | 동일 (TRANSIENT_LOCAL) | 컨트롤러 전환 확인. BT · GUI · digital_twin 이 이 토픽으로 다시 연결한다 |
| `/forward_position_controller/commands` | robot 모드 + `ur_driver_native` 전용 | `ros2 topic hz` 로 `control_rate` 확인. **sim 에는 없다** — sim 은 `devices.<group>.backend.command_topic` (예: `/ur5e/joint_command`) 을 본다 |
| `<session>/timing/cm_timing_log.csv` | CM RT loop | tick 별 8열 timing (`run_id`, `t_state_us` … `jitter_us`). p50/p99 는 후처리. `run_id` 로 먼저 묶는다. sim 에서는 `jitter_us` 가 항상 0. **RT 예산 판정은 이 CSV 로 한다** — `plot_rtc_log --stats` 의 `Overruns (> budget µs)` 행. 종료 요약 line 의 `overruns=` 는 sim 에서 센서가 아니다 (sim 동기화 경로가 overrun 판정을 우회해 `n/a(sim)` 으로 찍힌다) |
| `<session>/timing/mpc_timing_log.csv` · `rt_callback_timing_log.csv` | MPC thread · rt_callback thread | 같은 8열. MPC tick 당 1행 · device state 콜백당 1행. `rt_callback` 은 열 의미가 lane 고유다 (`t_state_us` = decode, `t_publish_us` = mailbox hand-off, `t_total_us` = slot 2 duty 분자) — dispatch 간격은 연속 행의 `t_wall_ns` 차분 |
| `/rtc_cm/{group}/joint_states` | CM (그룹별) | 그룹별 건강성 |
| `/sim/status` | `rtc_mujoco_sim` 1 Hz | sim 건강성 |
| `<contact_wrench.topic_prefix>/<target>/contact_wrench` | `rtc_mujoco_sim` | 손끝 접촉 힘/토크 (그룹 YAML `contact_wrench.enabled: true` + MJCF contact sensor 필요) |
| `/p1a/joint_states` 등 | `udp_hand_driver` | hand link 건강성 |
| `/shape/estimate` · `/shape/explore` (action) | `shape_estimation` | 형상 추정 결과 · 탐색 진행. 입력은 `<ns>/tof/snapshot` (`<ns>` = active 컨트롤러 이름) |

timing CSV 는 `ros2 run rtc_tools plot_rtc_log <csv> --stats` 로 읽는다.

## Debugging

| 증상 | 처방 |
|---|---|
| `ApplyThreadConfig()` 경고 | `sudo usermod -aG realtime $USER` 후 재로그인 |
| 시작하자마자 E-STOP | sim 이면 `init_timeout_sec: 0.0` |
| Jitter 큼 (> 200 us) | `check_rt_setup.sh --summary` → `cpu_shield.sh status` (격리는 `isolcpus` 가 아니라 cset shield). 단계별 점검은 [rt-optimization.md](rt-optimization.md#5-문제-해결) |
| Hand timeout E-STOP | UDP 링크 확인. `recv_timeout_ms` 는 로봇별 값이니 `integrated_bringup/config/<robot>/udp_hand_node_*.yaml` 을 본다 |
| TSID (dynamic) QP 가 거의 매 tick 실패 (`[wbc] Dynamic QP failed` 반복, solve 수 ms) | `wbc_diag.csv` 를 `plot_rtc_log --stats` 로 읽는다 — `QP converged` 비율, `qp_status` 이름별 히스토그램 (MAX_ITER / PRIMAL_INFEASIBLE 구분), `joint_box_inverted` 비율. inverted > 0 이면 관절이 TSID `joint_limit.position_margin` 띠 안에 있는 것이다 — 그 margin 은 position 백본의 `integration.position_margin` 보다 작아야 한다 (#551) |
| 관절 궤적이 "정착" 인지 "밴드에 pin" 인지 모르겠다 | 컨트롤러 로그의 `joint band engaged:` WARN 구간은 pin 이다. Ctrl-C 로 끝내면 요약이 없다 |
| `ament_cmake_test` missing | `.venv` overlay 가 system site-packages 를 가림. 재활성화 후 ROS 환경 재로드 |
| 0.01 초 만의 configure 실패 `Failed to find .../package.sh`, 빌드 직후 테스트 바이너리 없음 | 잘못된 cwd 로 colcon 을 돌린 것. repo 안 `build/`·`install/`·`log/` 삭제 후 workspace root 에서 재실행 |
| env 미source 로 전 바이너리 일괄 실패 | 회귀 아님. AGENTS.md §9.1 서브셸 표준형으로 재실행 |
| 지운 config · launch 를 테스트나 `ros2 launch` 가 아직 찾는다 | `--symlink-install` 이 끊긴 링크를 남긴다. `./build.sh -p <pkg>` 는 빌드 뒤 지운다 (`find install/<pkg> -xtype l` 로 확인) |
| `No rule to make target '/opt/ros/<distro>/lib/lib<X>.so.<옛 버전>'` | ROS apt 업그레이드 뒤 옛 경로가 남은 것. `colcon build --cmake-force-configure` ([repo_scripts/README.md](../repo_scripts/README.md)) |
| sim 을 다시 띄웠는데 옛 씬이 응답한다 | launch 프로세스만 죽이면 `mujoco_simulator_node` · `integrated_rt_controller` 가 남아 계속 발행한다. 재기동은 세 단계다: launch 를 죽이고 → 남은 노드를 `pidof` 로 찾아 PID 로 죽이고 → SIGKILL 이 남긴 `/dev/shm/fastrtps_*` 를 지운다 (Fast DDS 는 정상 종료할 때만 자기 세그먼트를 지운다) |
| 세션 CSV 의 끝이 잘렸다, 스크립트에서 `ros2 launch` 에 보낸 SIGINT 가 아무 일도 안 한다 | SIGINT 로 끝낸다 — `-9` 는 CSV flush 를 날린다. 비대화형 셸 (스크립트 · 에이전트) 이 `&` 로 띄운 job 은 SIGINT 를 SIG_IGN 으로 물려받고 `ros2 launch` 는 그대로 둔다. 노드는 자기 핸들러를 설치하므로 노드에 직접 `kill -INT $(pidof integrated_rt_controller mujoco_simulator_node)` (손 SIL: `udp_hand_node fake_hand_firmware` 둘 다) — launch 는 자식이 끝나면 따라 끝난다. 또는 스크립트에서 `set -m` 으로 띄우고 `kill -INT -- -<launch pid>` |
| `pkill -f <노드>` 가 자기 셸을 죽인다 (exit 144), `pgrep -f` 가 엉뚱한 PID 를 섞는다 | 명령을 `bash -c '…'` 로 감싸는 셸은 자기 커맨드라인에 그 패턴을 담는다. 노드는 `pidof <실행 파일 이름>` 으로 찾고 (`pgrep -x` 는 15 자에서 잘려 0 건이 된다), 직접 띄운 것은 띄울 때의 `$!` 로 죽인다 — `ros2 launch` · `ros2 topic pub` 은 실행 파일이 python 이라 `$!` 뿐이다 |
| 남겨 둔 평가 세션이 사라졌다 | launch 는 기동할 때 `logging_data/` 를 `max_log_sessions` (출하 10) 로 잘라 오래된 세션을 지운다. 세션을 모으는 동안에는 모든 기동에 `max_log_sessions:=<현재 개수 + 여유>` 를 주고, 판정에 쓸 세션은 끝나는 즉시 다른 곳으로 복사한다 |
| 한 세션 CSV 안에서 열 수가 중간에 바뀐다 (pandas `Expected N fields … saw M`) | 세션 이름은 `YYMMDD_HHMM` 이라 **같은 분에 띄운 두 기동은 한 세션 폴더를 공유**하고, 로거는 있던 CSV 에 헤더 없이 이어 쓴다 (다른 로봇이면 λ 열 수가 다르다). 기동 사이를 1 분 이상 띄우거나, 끝난 세션을 다음 기동 전에 복사해 둔다 |
| 켜 둔 sim 이 디스크를 채운다 | 활성 컨트롤러는 대기 중에도 tick 마다 CSV 를 쓴다 (무장한 catching 컨트롤러는 약 3.4 GB/h). 확인이 끝나면 바로 내린다. 정리할 때는 `YYMMDD_HHMM` 세션 폴더만 지운다 — 옆의 `*.json` · `stats/` · `ur_plot/` 은 사용자 파일이다 |
| sim 의 `<group>/joint_states` 구독이 0 메시지, "incompatible QoS" 경고 | sim 은 상태를 BEST_EFFORT depth 1 로 발행한다. 구독도 BEST_EFFORT (sensor-data QoS) 로 한다 |

```bash
# exec 이름 = 노드 이름 = integrated_rt_controller (integrated_bringup 의 유일한 실행파일)
PID=$(pidof -s integrated_rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
ros2 lifecycle list /integrated_rt_controller
ros2 topic echo /system/estop_status
./repo_scripts/scripts/check_rt_setup.sh --summary
```

## 실기 RT 점검

### RT 권한

```bash
sudo groupadd realtime && sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# 재로그인 필요. 나머지 RT 설정 (GRUB nohz_full · cpu_shield · IRQ) 은 ./install.sh robot --rt
```

### CPU Shield 검증

`cpu_shield.sh` 는 cpuset 을 만들기만 하고 런치가 CM 을 그 안으로 옮긴다. 격리가 실제로 서는지는 **실기** 에서만 확인된다. 기대값은 [repo_scripts/README.md](../repo_scripts/README.md) 의 코어 레이아웃 함수 (`get_cm_shield_cpus` · `get_role_slot` 등) 가 SSoT 이고 여기에 숫자를 적지 않는다.

```bash
sudo ./repo_scripts/scripts/cpu_shield.sh on --robot
cset shield -s                                   # user 집합 == get_cm_shield_cpus 출력
CM=$(pidof -s integrated_rt_controller)          # 런치(shield-on) 후
grep Cpus_allowed_list /proc/$CM/status          # 위와 동일해야 함
ps -eLo comm,psr,cls,rtprio -p $CM | grep -E "rt_control|rt_callback|nrt_"   # pin · FIFO (nrt_* 는 TS 가 정상)
grep -rE "rc=22|setaffinity failed|Thread config failed" ~/.ros/log/<run>/   # 결과 없어야 (EINVAL 회귀)
```

두 번째 런치 로그에 `CPU shield already active (cset user cpuset present)` 가 보이면 게이트가 활성 shield 를 다시 만들지 않은 것이다.

### CAN FD 손 게이트 (G-5 두 손)

한 run 은 [run_hand_canfd_gate.sh](../repo_scripts/scripts/run_hand_canfd_gate.sh) 하나다: 버스 기록 → 노드 기동 (`hand_sides`) → 손마다 `link_status` true 대기 → `--duration` 초 → `/hand/estop` true (run 은 E-Stop 으로 끝난다) → 노드에 SIGINT → 손마다 `analyze_hand_canfd_gate` → 요약 한 줄. 산출물은 `--out` (기본 `./hand_gate_<시각>`) 아래 `bus.log` · `node.log` · `session/` · `gate_<side>.md` · `.json` 이다.

```bash
# 개발 PC, vcan0 SIL (fake 둘) — colcon test 와 동시에 돌리지 않는다
./repo_scripts/scripts/run_hand_canfd_gate.sh --sil --duration 60
# 실기 (테스트 PC): can0 1 M / 8 M FD, 손 node id 1 · 2
./repo_scripts/scripts/run_hand_canfd_gate.sh --iface can0 --duration 300 --rate 250 --recv-timeout 0.9
```

- 실기 전에 두 손이 각각 REQ 에 답하는지 본다: `candump can0` 을 띄우고 `cansend can0 00404000##1` (오른손) · `cansend can0 00408000##1` (왼손) 에 RESP 3 프레임 (`0080004x` · `0080008x`) 이 오는지
- PASS: 손마다 (a)–(e) PASS, `x.no_overlap` 0, link down 0. 250 Hz 에서 (c) 가 미달이면 `--rate 200` 으로 다시 잰다 (D77). 두 손 zero-write 간격 행은 보고만 한다 (D78)
- 종료 코드 0 PASS · 1 FAIL · 2 UNDECIDED (한 번 재측정) · 3 run 불가. candump 이 없는 PC 는 같은 형식의 python 기록기로 대신한다

### Tracing

thread 별 core 사용이나 callback 소요 시간은 CSV 로 알 수 없다. LTTng 트레이스를 캡처해 본다. 명령 전체는 [tracing.md](tracing.md).

```bash
./install.sh --tracing                                                       # 1회 설정
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true   # 캡처
./repo_scripts/scripts/timeline.sh                                           # Perfetto JSON 변환
```

## Test 측정

테스트 카운트는 문서에 적지 않고 직접 잰다.

```bash
rm -rf build/<pkg>/test_results                                    # 누적 XML 제거
colcon test --packages-select <pkg> --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose --test-result-base build/<pkg>       # 패키지 하나 (--packages-select 는 없다)
( cd build/<pkg> && ctest -N )                                     # 등록 확인 — 새 테스트는 재빌드 뒤에 보인다
```

결과를 어떻게 판정하는지 (누적 XML · 종료 코드 · 범위와 단위 · timeout 과 crash) 는 [testing-debug.md](../agent_docs/testing-debug.md#test-측정) 가 갖고, 각 항이 실제로 잘못된 보고를 낳은 사례는 [testing-rationale.md](reference/testing-rationale.md) §계측 함정 에 있다.

### 커버리지

C++ 는 `gcovr`, Python 은 `coverage` 를 쓴다. 정규 build/install 트리를 오염시키지 않도록 별도 base 를 쓴다.

```bash
uv pip install gcovr                                             # venv 안, 분석 전용
colcon build --packages-select <pkg> --build-base <scratch>/cov/build --install-base <scratch>/cov/install \
  --cmake-args -DENABLE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
colcon test --packages-select <pkg> --build-base <scratch>/cov/build --install-base <scratch>/cov/install
SRC=src/rtc-framework/<pkg>
gcovr --root "$SRC" --object-directory <scratch>/cov/build/<pkg> --filter "$SRC/src/" --print-summary
# Python: cd <pkg> && python3 -m coverage run --source=<module> -m pytest test/ -q && python3 -m coverage report -m
```

`ENABLE_COVERAGE` 는 패키지별 옵션이다. `rtc_mpc` · `integrated_bringup` 은 Debug 에서 일부 바이너리가 죽으니 [testing-rationale.md](reference/testing-rationale.md) §Coverage 함정 을 먼저 본다.

