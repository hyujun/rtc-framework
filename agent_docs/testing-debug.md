# Testing & Debugging

## Sensor Matrix (변경 유형별 필수 검증)

[CLAUDE.md](../CLAUDE.md#L87) §5의 상세판. 변경 위치에 따라 필수 sensor + 추가 sensor를 실행한다.

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
| `rtc_digital_twin/` | pytest + RViz2 smoke | `/{group}/digital_twin/joint_states` hz |
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

## Test Table (최근 실측: 2026-04-26, gtest CASES 단위)

| Package | Tests | Framework |
|---------|-------|-----------|
| `udp_hand_driver` | 179 C++ (hand_packets 46, hand_controller 33, hand_udp_codec 24, fingertip_ft 19, hand_udp_transport 18, hand_timing_profiler 15, hand_sensor_processor 13, hand_failure_detector 11) | GTest |
| `ur5e_bt_coordinator` | 157 C++ (condition_nodes 21, bt_utils 19, bt_types 16, hand_nodes 15, compute_offset_pose 12, shape_nodes 11, tree_validation 10, set_gains 9, set_pose_z 8, switch_controller 7, grasp_control 7, compute_sweep_trajectory 6, move_to_joints 6, wait_duration 5, move_to_pose 5) | GTest |
| `rtc_mpc` | 136 C++ (types, TripleBuffer, TrajectoryInterpolator, Riccati, SolutionManager, thread skeleton, RobotModelHandler, PhaseCostConfig) | GTest |
| `integrated_bringup` | 92 C++ (demo_wbc_controller 44, demo_shared_config 16, grasp_phase_manager 13, virtual_tcp 10, demo_wbc_mpc_integration 6, grasp_pipeline 3) | GTest |
| `rtc_tsid` | 90 C++ (wbc_types 11, se3_task 10, qp_solver_wrapper 8, momentum_task 8, tsid_wqp 7, force_task 6, com_task 6, posture_task 5, phase3_integration 5, joint_limit 5, tsid_hqp 4, wqp_hqp_compare 3, performance 3, eom 3, torque_limit 2, friction_cone 2, contact 2) | GTest |
| `rtc_urdf_bridge` | 89 C++ (urdf_analyzer 25, rt_model_handle 16, model_builder 15, chain_extractor 12, xacro_processor 11, joint_classification 10) | GTest |
| `rtc_controllers` | 87 C++ (test_core_controllers + grasp related) | GTest |
| `shape_estimation` | 82 C++ (exploration_motion 29, tof_shape 27, protuberance_detector 26) | GTest |
| `rtc_mujoco_sim` | 77 C++ (simulator_init 15, pure_helpers 15, command_state_io 12, runtime_controls 11, lifecycle 10, solver_config 9, data_flow 5) | GTest |
| `rtc_base` | 54 C++ (SeqLock, SPSC, Bessel/Kalman filters, session dir, thread-config tiers) | GTest |
| `rtc_controller_interface` | 52 C++ (registry 8 incl. duplicate-shadow + rt_controller_interface) | GTest |
| `rtc_controller_manager` | 35 C++ (controller_lifecycle 9, switch_service 9, controller_timing_profiler 17) | GTest |
| `rtc_communication` | 9 C++ (udp_loopback 5, transceiver 4) | GTest |
| `rtc_tools` | 247 Python (pytest parameterized) | pytest |
| `rtc_digital_twin` | 34 Python (test_urdf_parser; pytest 자동 discovery 미통과 — `pytest test/` 직접 실행 필요) | pytest |

총 **1377 gtest cases + 281 pytest cases**. 5개 신규 측정 패키지 (`rtc_tsid`/`rtc_mujoco_sim`/`rtc_communication`/`shape_estimation`/`rtc_digital_twin`) 2026-04-26 실측 추가. Q-6 추가 테스트로 `rtc_controller_interface` 51→52. **참고**: 메모리에 기록된 `.venv` overlay → `ament_cmake_test` 이슈는 현재 PC 환경에서 재현되지 않음. `rtc_digital_twin` pytest discovery 실패는 별개 이슈 (pytest config 미존재 추정).

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
| `/{group}/digital_twin/joint_states` | controllers (per-group, RELIABLE) | Device 그룹별 건강성; `rtc_digital_twin`이 merge |
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

## Profiling — Hotspot + perf (function attribution per thread)

CSV timing logs (`cm_timing_log.csv` / `mpc_timing_log.csv` / `hand_udp_timing_log.csv`)
은 **per-tick 총 시간**만 기록한다. RT 회귀가 보이지만 어떤 함수가 시간을 쓰는지 모를 때
샘플링 프로파일을 캡처해 [Hotspot](https://github.com/KDAB/hotspot) 으로 본다.

### One-time setup (Fresh Ubuntu 24.04)

```bash
./install.sh --perf       # apt: linux-tools + hotspot, sysctl: perf_event_paranoid=1
# 또는 수동 동등 명령:
sudo apt install linux-tools-generic linux-tools-$(uname -r) hotspot
echo 'kernel.perf_event_paranoid = 1' | sudo tee /etc/sysctl.d/99-perf.conf
sudo sysctl --system
```

검증: `./repo_scripts/scripts/check_rt_setup.sh --summary` → `Perf Profiling [PASS]`.

### Capture (sim 또는 robot)

```bash
ros2 launch integrated_bringup sim.launch.py enable_perf:=true
# 또는
ros2 launch integrated_bringup robot.launch.py enable_perf:=true

# 출력: <ws>/logging_data/<YYMMDD_HHMM>/perf/perf.data
# Ctrl+C 로 launch 종료 시 perf 가 SIGINT 받아 정상 flush.
```

선택 인자:

* `perf_targets:='regex|of|process|names'` — 기본은 `integrated_rt_controller|mujoco_simulator_node`
  (sim) / `integrated_rt_controller|udp_hand_node|ur_ros2_control_node` (robot).
  `pgrep -f` 로 매칭됨. shell wrapper(bash/sh/dash/zsh) 는 자동 제외.
* `perf_duration:=30` — 초 단위. 빈 값(기본)이면 launch SIGINT 까지 record.
* `perf_start_delay:=N` — perf 시작까지 대기. **기본 `0` = 시작부터 캡처**
  (lifecycle bring-up 포함). steady-state 만 보고 싶으면 `5` 이상.
* `perf_stack_size:=N` — DWARF unwind stack 크기 (바이트). **기본 `4096` =
  Hotspot 로딩 빠름**. 깊은 C++ 템플릿 스택이 잘리면 `8192` / `16384` 로 상향.
  trace 파일 크기는 stack 크기에 선형 비례 (4096 → 8192 = 2× 큼).
* `perf_frequency:=N` — 샘플링 주파수 Hz, **기본 `999`**. RT tick (2 ms) /
  MPC tick (8 ms) 분석에 충분한 sub-ms 분해능. 가벼운 탐색용으로 `99` 로
  하향하면 trace 가 ~10× 작아짐.
* `perf_event:=<event>` — 샘플링 이벤트, **기본 `cycles:P`** (CPU cycle 기반).
  burst-then-sleep RT 루프 (`rt_control` 30 µs/2 ms) 분석 시 cycles:P 는
  CPU 점유율에 비례 샘플링이라 **자고 있는 동안의 주기성을 못 잡는다**. 그런
  RT 타이밍을 보고 싶으면 **`perf_event:=task-clock`** 으로 변경 — wall-clock
  CPU 시간 기반 샘플링이라 짧은 burst 마다 균일하게 잡힘.

### Sampling event 선택 가이드

`cycles:P` 와 `task-clock` 은 같은 perf 가 다른 시각을 제공한다:

| 시나리오 | 추천 event | 근거 |
|---|---|---|
| MPC / mujoco 같은 CPU-heavy thread 의 hot path | `cycles:P` (default) | CPU 사용 시간에 비례 — hot 함수가 많이 잡힘 |
| `rt_control` 의 정기 tick 주기성 검증 (default 500 Hz, `control_rate`로 가변) | **`task-clock`** | 30 µs work × 1.5 % CPU 라 cycles:P 로는 거의 못 잡음 |
| timeline 에서 thread 간 주기성 비교 | **`task-clock`** | 모든 thread 가 active 시간 기반으로 균일 샘플링 |
| Hot function flame graph (집계 통계) | `cycles:P` | CPU cycle 기준 % 가 더 직관적 |

```bash
# RT 주기성 분석 — task-clock 사용
ros2 launch integrated_bringup sim.launch.py enable_perf:=true perf_event:=task-clock

# 결과를 timeline 으로
./repo_scripts/scripts/timeline.sh
```

### Examples

```bash
# 기본 (시작부터 캡처, stack 4 KB, 999 Hz)
ros2 launch integrated_bringup sim.launch.py enable_perf:=true

# 30 초만 캡처 + steady-state 만 (lifecycle 5 초 후 시작)
ros2 launch integrated_bringup sim.launch.py enable_perf:=true \
    perf_start_delay:=5 perf_duration:=30

# 가벼운 탐색 (낮은 빈도 + 작은 stack)
ros2 launch integrated_bringup sim.launch.py enable_perf:=true \
    perf_frequency:=99

# deep template 스택 보존 (큰 trace)
ros2 launch integrated_bringup sim.launch.py enable_perf:=true \
    perf_stack_size:=16384
```

### View — 두 가지 옵션

#### A. **flame.sh** (권장 — 빠름, Hotspot 우회)

```bash
./repo_scripts/scripts/flame.sh                # 가장 최근 perf.data, mode=mixed
./repo_scripts/scripts/flame.sh --by thread    # 스레드별 분리 (top frame = comm-pid/tid)
./repo_scripts/scripts/flame.sh --by cpu       # CPU 코어별 분리 (top frame ends with [cpu_NNN])
./repo_scripts/scripts/flame.sh <perf.data>    # 명시 입력
./repo_scripts/scripts/flame.sh --by cpu <perf.data> [output.svg]
```

* 출력: `<session>/perf/flame.svg` (mixed) / `flame-thread.svg` / `flame-cpu.svg`
  — 인터랙티브 SVG, 브라우저 자동 열림
* **로딩 시간: 1–2 초** (Hotspot 1.3.0 perfparser 병목 우회)
* 모드 선택 가이드:
  - `mixed` (default) — "전체에서 어디가 hot?" — 함수별 self %를 한 화면
  - `--by thread` — "MPC 가 mpc_main 안에서 어디 쓰나?" / "rt_control 의 WBC tick
    안 분포는?" — 각 thread 의 분포를 분리해서 봄
  - `--by cpu` — "CPU 4 (MPC core) 가 정말 mpc_main 만 쓰나?" / "RT core 2 가
    의도대로 rt_control 전용인가?" — pinning 검증, IRQ leak 확인
* 기능:
  - **Search** (우상단) → `^rtc::|^integrated_bringup::` 입력 시 워크스페이스 함수 highlight
  - 프레임 클릭 → 해당 subtree zoom (thread/CPU 모드에서는 top row 클릭으로 한
    스레드/코어 안으로 drill-down)
  - 스택 hover → self / total samples
* 한계: timeline / off-CPU 같은 Hotspot 전용 뷰는 없음 (flame graph 만)

#### B. **timeline.sh + Perfetto** (시간순 — 누가 언제 어느 코어에서 돌았나)

`flame.sh` 가 "어디가 hot 한가" 를 본다면, `timeline.sh` 는 **"언제 어느 순서로
구동했나"** 를 본다. x축 시간, y축 thread/CPU swimlane.

두 가지 모드:

* **`--mode flamechart`** (default) — **함수 이름이 박힌 가로 bar**. 인접 샘플의
  공통 callstack prefix 가 유지되는 동안 그 함수가 실행 중이었다고 추정해
  begin/end pair 로 변환. width = 함수가 stack 에 머문 시간. 사용자가 보고 싶은
  "thread별로 어떤 함수가 어떤 순서로 구동했나" 가 이 형태.
* **`--mode instant`** — 각 sample 이 vertical marker. hover 시 callstack 노출.
  bar 가 없으니 시각화는 듬성하지만 timestamp 정확도가 sample 단위.

```bash
./repo_scripts/scripts/timeline.sh                       # 기본: flamechart, max-depth=10
./repo_scripts/scripts/timeline.sh --mode instant        # 마커 모드
./repo_scripts/scripts/timeline.sh --max-depth 5         # 얕은 stack → 작은 JSON
./repo_scripts/scripts/timeline.sh <perf.data>           # 명시 입력
```

**Sampling 한계 (flamechart 모드)**: perf 는 99–999 Hz snapshot 이라 진짜
begin/end 가 아니다. 인접 샘플의 stack 비교로 함수 지속 구간을 추정하므로
**한 sample period (1–10 ms) 이내 오차** 가 있을 수 있다. 예: 0.5 ms 만 실행된
함수가 아예 누락되거나, 인접 두 sample 모두에 같은 함수가 잡혀서 실제보다
긴 bar 로 그려질 수 있음. WBC tick 30 µs / MPC tick 8 ms 같은 큰 단위에는
충분히 정확하나, sub-ms 함수는 신뢰하지 말 것. 정확한 begin/end 가 필요하면
별도 uprobe / USDT instrumentation 도입이 필요.

* 출력: `<session>/perf/trace.json` (Chrome Trace JSON)
* **변환 시간: 1–2 초** (1k–10k samples 기준). JSON 크기는 sample 수 × max-depth 에 비례.
* 보기: https://ui.perfetto.dev 열고 JSON drag-drop. 설치 0, 브라우저만 있으면 됨.
* 두 가지 swimlane 그룹이 동시에 보임:
  - **Threads (by TID)** — `mpc_main-NNNN`, `rt_control-NNNN`, `mujoco_simulato-NNNN` 등
    각 thread 의 sample 시퀀스. WBC tick 의 주기성, MPC 8 ms 주기 등을 시간축에서 직접 확인.
  - **Cpus** — `Cpu 000` ~ `Cpu 011` 코어별. ApplyThreadConfig pinning 의 의도대로
    rt_control 이 Core 2 에 고정됐는지, mpc_main 이 MPC 코어에 머물렀는지 검증.
* Perfetto UI tip: 좌측 swimlane 헤더 클릭 → 그 thread 만 펼침. 마우스 휠로 zoom,
  shift+드래그로 범위 측정. 검색 (Ctrl+F) 으로 callstack 안 함수 이름 찾기.
* 한계:
  - perf 는 sampling 이라 begin/end pair 없음 — slice 폭은 1 µs marker, **gap 이
    실제 작업 시간**.
  - default capture 에는 sched_switch / off-CPU 가 없음. mutex 대기 등 off-CPU 분석은
    perfetto traced_perf 또는 perf record `--off-cpu` 가 필요 (별도 워크플로).

#### C. **Hotspot** (멀티-뷰, 느림)

```bash
hotspot <ws>/logging_data/<session>/perf/perf.data
```

* Ubuntu 24.04 apt 의 hotspot 1.3.0 은 **큰 trace에서 perfparser 가 5–10 분
  걸리는 알려진 버그** 가 있다. trace 가 작거나(< 5 MB) 시간 여유 있을 때만 권장.
  최신 1.5.x AppImage (https://github.com/KDAB/hotspot/releases) 가 훨씬 빠름.
* 탭 별 용도:
  - **Timeline** — 스레드별 가로 막대로 어느 코어에서 언제 활성/대기였는지. RT 스레드가
    off-CPU 로 빠지는 구간이 mutex/syscall 후보.
  - **Bottom-Up / Top-Down** — 스레드 필터 후 함수별 self/inclusive %. WBC tick 안에서
    Pinocchio FK / ProxSuite QP / SPSC push 의 비율 측정.
  - **Flame Graph** — 같은 데이터를 stack 단위로 본다 (= flame.sh 와 동일 정보).

### Permissions

스크립트 (`repo_scripts/scripts/perf_record.sh`) 가 자동 분기:

| `perf_event_paranoid` | 동작 |
|---|---|
| ≤ 1 | user 권한으로 perf record (`./install.sh --perf` 적용 후 기본 상태) |
| > 1, passwordless sudo OK | sudo perf record + 결과 파일 user-owned 로 chown |
| > 1, sudo 패스워드 필요 | warn 후 캡처 skip — launch 는 정상 진행 |

### Limits

* perf 는 sampling profiler — tick 안의 통계 분포는 보이지만 **특정 spike 의 ground truth**
  는 아니다. 단발 jitter 원인 추적은 향후 Perfetto/RTLA 도입 시 보강.
* dwarf call-graph 는 디스크 IO 가 크다. 30 s 캡처 = 약 50–500 MB. 장기간 캡처는
  `perf_duration:=N` 으로 제한 권장.
