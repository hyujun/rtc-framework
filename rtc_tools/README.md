# rtc_tools

RTC 로그 시각화, 포구 sim 평가 도구, MJCF/URDF 변환·비교, launch 헬퍼를 모은 Python 개발 유틸리티 패키지다. **유형**: 도구 패키지 (`ament_python`)

## 개요

- 제어 로그 CSV 를 그리고 (`plot_rtc_log`), 동적 포구 sim 의 시행을 오프라인으로 평가하며 (`catching_*` · `catch_*`), MJCF/URDF 를 변환·비교한다.
- 실행 파일은 `setup.py` 의 `console_scripts` 29 개이고 `ros2 run rtc_tools <명령>` 으로 부른다. 여기 없는 모듈은 `python3 -m` 으로 부른다.
- `launch/` 와 `utils/` 의 모듈은 다른 패키지의 launch 파일과 도구가 import 한다 (아래 [launch 헬퍼](#launch-헬퍼와-utils-모듈)).
- 분석 CLI 의 판정 규칙은 [catching-analysis-rules.md](../docs/catching-analysis-rules.md), 열의 뜻은 각 도구의 머리 docstring 이 갖는다. 이 README 는 실행 방법만 둔다.
- 컨트롤러 데모 GUI 는 이 패키지에 없다: [integrated_bringup/scripts/demo_controller_gui.py](../integrated_bringup/scripts/demo_controller_gui.py).

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다. 이 패키지만 다시 빌드하려면 `./build.sh -p rtc_tools` 다.

```bash
# 가장 최근 세션의 CM 타이밍 로그를 그린다 (세션 CSV 는 logging_data/YYMMDD_HHMM/ 아래)
ros2 run rtc_tools plot_rtc_log logging_data/<YYMMDD_HHMM>/timing/cm_timing_log.csv
# 통계만 (창 없이)
ros2 run rtc_tools plot_rtc_log logging_data/<YYMMDD_HHMM>/timing/cm_timing_log.csv --stats
```

도구 선택 요약:

| 하고 싶은 일 | 쓰는 도구 |
|---|---|
| 세션 로그를 그린다 | `plot_rtc_log` |
| MJCF 가 URDF 와 같은 모델인지 확인한다 | `compare_mjcf_urdf` |
| URDF/xacro 로 MJCF 를 만든다 | `urdf_to_mjcf` |
| LTTng trace 를 Perfetto 로 본다 | `ctf_to_chrome_trace` (`python3 -m`) |
| 포구 sim 시행을 평가한다 | `catching_trials` → `catching_pool` 외 |
| 포구 가능 영역을 오프라인으로 본다 | `catchability_map` → `catch_gate_map` → `catch_search_map` |

## 인터페이스

### console_scripts

| 명령 | 하는 일 | 예 |
|---|---|---|
| `plot_rtc_log` | CSV 로그를 종류별로 그린다 (`plot_ur_log` · `plot_ur_trajectory` 는 별칭) | `plot_rtc_log cm_timing_log.csv --stats` |
| `hand_udp_sender_example` | 10-DOF 손 UDP 프로토콜 라이브러리 + 대화형 예제 | `hand_udp_sender_example` |
| `compare_mjcf_urdf` | MJCF 와 URDF 의 물리 파라미터 비교 | `compare_mjcf_urdf --robot-pkg robot_descriptions --robot-name ur5e` |
| `urdf_to_mjcf` | URDF/xacro → MJCF 변환 | `urdf_to_mjcf --input robot.urdf --output robot.xml` |
| `derive_accel_limits` | 토크 한계에서 관절 가속 box 를 도출 | `derive_accel_limits --robot-config … --group ur5e --eta-tau 0.8` |
| `analyze_clock_phase` | sim clock lane 의 시행별 위상 오차 | `analyze_clock_phase clock_lane.csv --plot out.png` |
| `run_clock_phase_trials` | 지정 발사 N 회 러너 (clock 위상 시행 생성) | `run_clock_phase_trials --trials 200` |
| `analyze_hand_close` | 손 폐쇄 시간 $T_{close,e2e}$ 분석 | `analyze_hand_close <hand>_state.csv --profile run.json` |
| `run_hand_close_trials` | 손 preshape↔closed 계단 N 회 러너 | `run_hand_close_trials --group … --joint-names … --out run.json` |
| `analyze_hand_canfd_gate` | CAN FD 손 실기 run 의 G-3 게이트 판정 (D55) | `analyze_hand_canfd_gate <session> g3.log --node-log node.log` |
| `analyze_vision_lane` | 예측 lane 프로브 CSV 요약 | `analyze_vision_lane <prefix>` |
| `vision_lane_probe` | 예측 · 카메라 · truth lane 을 CSV 로 기록 | `vision_lane_probe <prefix> --dump` |
| `camera_relay` | 카메라 lane 릴레이 + 드롭·지연 주입 | `camera_relay --drop-after-s 0.4` |
| `catching_trials` | sim 한 run 의 시행별 평가표 | `catching_trials <session> <trials_dir> --config-dir <cfg>` |
| `catching_pool` | arm 별 unit 합산 판정 | `catching_pool --arm A ct/1 ct/2 --floor 0.35 --out pool/` |
| `catching_hand_near` | 손 근처 투척의 commit/catch 2 단계 분석 | `catching_hand_near units/u1 --out hn/` |
| `catching_arm_budget` | 팔 예산 네 층의 귀속 | `catching_arm_budget units/u1 --config-dir <cfg> --out ab/` |
| `catching_decel` | DECEL 정지 구간 지표 · paired 비열등 검정 | `catching_decel units/u1 --config-dir <cfg> --out decel/` |
| `catching_grid_sweep` | 예측 격자 조건별 성공률 비교 | `catching_grid_sweep --arm A units/A_* --ref A --out sweep/` |
| `catching_wait_pose_search` | 대기 자세 탐색 | `catching_wait_pose_search --config-dir <cfg> --radius-m 0.15 --out o/` |
| `catchability_map` | 투척 grid → C++ judge → catchability 지도 | 아래 절 |
| `catch_speed_budget` | 수락 후보별 팔 속도·토크 한계 가속 예산 | 아래 절 |
| `catch_gate_map` | 나머지 게이트를 건 gate-catchable 지도 | 아래 절 |
| `catch_search_map` | 계획기 탐색의 투척별 수용 지도 | 아래 절 |
| `catch_throw_design` | 통과점 투척 설계 · 목록 · 표본 (`design` · `select` · `plan-set` · `sample`) | 아래 절 |
| `catch_population` | 투척 모집단 1 단계 (설정 파일 하나로 위 도구를 순서대로) | `catch_population --config … --out <result>` |
| `catch_population_report` | 모집단 결과 폴더의 통계 · 3D plot | `catch_population_report <result>` |

`<cfg>` 는 로봇 profile 디렉토리다 (예: `integrated_bringup/config/ur5e_p1b`). 아래 예시의 `$CFG` 도 같다.

그 밖에 `console_scripts` 가 아닌 모듈은 `python3 -m rtc_tools.<경로>` 로 부른다: `conversion.ctf_to_chrome_trace` · `utils.hand_data_plot`. 분석 모듈 `planner_solves` · `catching_capture_set` 은 CLI 없이 import 용 라이브러리다.

### plot_rtc_log

분리된 CSV 제어 로그를 Matplotlib 으로 그린다. 파일 이름과 열 지문으로 로그 종류를 자동 감지한다.

```bash
ros2 run rtc_tools plot_rtc_log <device>_state_log.csv            # DeviceStateLog
ros2 run rtc_tools plot_rtc_log <device>_sensor_log.csv           # DeviceSensorLog
ros2 run rtc_tools plot_rtc_log cm_timing_log.csv                 # CM RT loop 타이밍
ros2 run rtc_tools plot_rtc_log mpc_timing_log.csv                # MPC main loop 타이밍
# 컨트롤러 소유 진단 CSV (<session>/controllers/<config_key>/ 아래)
ros2 run rtc_tools plot_rtc_log momentum_observer.csv --stats
ros2 run rtc_tools plot_rtc_log planner_events.csv
ros2 run rtc_tools plot_rtc_log dualarm_diag.csv
# 같은 분에 재기동해 한 파일에 런이 둘 이상이면 (기본: 마지막 런) 특정 런 선택
ros2 run rtc_tools plot_rtc_log cm_timing_log.csv --run-id 260808143052
# 창 없이 PNG 만 저장
ros2 run rtc_tools plot_rtc_log <device>_state_log.csv --save-dir /tmp/plots --no-show
ros2 run rtc_tools plot_rtc_log <device>_state_log.csv --all      # 모든 figure
```

- `--save-dir` 를 안 주면 **입력 CSV 가 속한 세션의 `plots/`** 에 저장한다. CSV 경로에서 위로 올라가며 만나는 `YYMMDD_HHMM` 디렉토리가 기준이라, 지난 세션의 CSV 를 다시 그려도 figure 가 그 세션에 남는다. 세션 트리 밖의 CSV 는 `RTC_SESSION_DIR/plots/` → 현재 ws `logging_data` 의 최신 세션 순으로 폴백하고, 그것도 없으면 저장 없이 창만 띄운다.
- 창은 기본으로 뜬다 (`--show`). `--save-dir` 만 주면 PNG 를 쓴 뒤 **창을 닫을 때까지 종료하지 않으므로** 스크립트·백그라운드에서는 `--no-show` 를 함께 준다 (그때 Agg backend).
- 타이밍 CSV 에 `run_id` 가 여러 개면 마지막 런만 그리고 버린 런을 stdout 에 적는다. `--run-id` 에 없는 값을 주면 사용 가능한 목록과 함께 에러로 끝난다.

#### 파일 이름 자동 감지

| 패턴 | 모드 |
|---|---|
| `*_state_log.csv` | state_log |
| `*_sensor_log.csv` | sensor_log (헤더보다 데이터 열이 많으면 inference 열을 복구) |
| `cm_timing_log*.csv` · `mpc_timing_log*.csv` | cm_timing · mpc_timing (같은 8 열 스키마) |
| `<dev>_state.csv` (`accel_*` 열이 있는 것) | wbc_log (state_log 에 TSID 가속도 · SE3 · fingertip force 추가) |
| `wbc_diag.csv` | wbc_diag (TSID/QP tick 진단) |
| `pull_estimator.csv` | pull_estimator (in-plane pull force 추정) |
| `grasp_diag.csv` | grasp_diag (Force-PI 서보 + 강성 추정) |
| `compliance_diag.csv` | compliance_diag (task-admittance 진단, 통계 전용) |
| `momentum_observer.csv` | momentum_observer (운동량 관측기 잔차 · payload) |
| `catching_diag.csv` | catching_diag (동적 포구 tick 레코드 — 지문 열 `track_err_rad` + `ref_gamma`) |
| `planner_events.csv` | planner_events (계획기 wake 당 한 행 — 지문 열 `n_in_window` + `rank_error_budget`) |
| `dualarm_diag.csv` | dualarm_diag (다중 frame CLIK tick 레코드 — 지문 열 `clik_ran` + `<task>_err_lin`) |

진단 CSV 를 읽는 규칙은 [catching-analysis-rules.md](../docs/catching-analysis-rules.md#plot_rtc_log) 에 있다.

state/sensor/timing 로그의 figure 는 플래그로 고른다:

| 모드 | 기본 | 플래그로 추가 |
|---|---|---|
| robot (state_log) | 관절 위치 · 속도 | `--command` 제어 명령 · `--torque` 실제 토크 · `--task-pos` TCP 위치 · `--error` 추적 오차 |
| device/hand (state/sensor) | 모터 위치 · 속도 · 센서 | `--raw` pre-LPF 센서 · `--ft` F/T 추론 출력 · `--sensor-compare` raw 대 filtered |
| timing | 루프 타이밍 분해 · 총 시간과 지터 · 히스토그램 | — |

기압/ToF lane 이 없는 힘 전용 손은 센서 figure 대신 `fingertip_force.png` 한 장을 만든다.

#### 우클릭 확대 — 정확한 x/y 범위 입력

GUI 로 뜬 figure 의 **subplot 을 우클릭**하면 x/y 범위를 숫자로 입력하는 대화상자가 열린다 (`--show` 가 기본이라 그냥 실행하면 된다).

| 항목 | 동작 |
|---|---|
| `x min` / `x max` | 시간 창. 기본으로 그 figure 의 모든 subplot 에 적용 (다관절 그리드는 subplot 이 서로 독립이라 드래그 확대는 칸 하나만 바꾼다) |
| `y min` / `y max` | 기본은 우클릭한 칸만 (한 그리드 안에서도 단위가 갈린다). 체크박스로 figure 전체 |
| `y: fit to the x window` | y 를 창 안 데이터로만 재적합 (스파이크가 y 를 차지하지 않게) |
| `also save a zoomed PNG` | `<figure>_zoom_<xmin>-<xmax>.png` 를 같은 `plots/` 에 새로 쓴다 (전 구간 PNG 는 덮지 않는다) |
| `Reset` | 대화상자를 붙인 시점의 범위로 복원 |

- figure 여백(패널 밖)을 우클릭하면 x 범위만 묻는다.
- toolbar 의 pan / zoom 이 켜져 있으면 우클릭이 matplotlib 자신의 제스처라 대화상자가 안 뜬다. 도구를 끄고 다시 우클릭한다.
- `--no-show` 에서는 이벤트 루프가 없어 확대가 불가능하다.

### compare_mjcf_urdf

MJCF 와 URDF 를 파싱해 link 질량 · COM · 관성 · 관절 한계 · 축 · 위치 · armature 의 동일성을 검증한다. mismatch 가 0 이면 exit `0`, 아니면 `1` 이다.

```bash
ros2 run rtc_tools compare_mjcf_urdf                                   # ament_index 로 자동 탐색
ros2 run rtc_tools compare_mjcf_urdf --mjcf ur5e.xml --urdf ur5e.urdf  # 경로 지정
ros2 run rtc_tools compare_mjcf_urdf --robot-pkg robot_descriptions --robot-name ur5e
# world frame 이 다르면 공통 기준 프레임을 선언한다
ros2 run rtc_tools compare_mjcf_urdf --align-frames world base --mjcf ur5e.xml --urdf ur5e.urdf
# 구조 비교 (body 수 · 전체 질량) 까지 — workspace env 에서 모듈로
source repo_scripts/scripts/setup_env.sh
python3 -m rtc_tools.validation.compare_mjcf_urdf --mjcf robot.xml --urdf robot.urdf --fail-on-unverified
```

- **`ros2 run` 으로는 구조 비교가 돌지 않는다.** console script 의 shebang 이 `/usr/bin/python3` 이고 `mujoco` 는 `.venv` 에 있다. 이때 도구는 구조 검사를 건너뛰고 `UNVERIFIED: 1` 을 찍는다 (통과가 아니다). 게이트에는 위 `python3 -m … --fail-on-unverified` 를 쓴다.
- 주요 옵션: `--link-map` (이름이 엇갈리는 link 쌍 · 접힌 link `fuse:`) · `--tip-frames` (tool 프레임 직접 비교) · `--joints` · `--tolerance` (기본 1e-4).
- `robot_descriptions/robots/model_pairs.yaml` 의 쌍은 `test/test_real_model_pairs.py` 가 자동 검사한다.
- 비교 항목 · 허용오차 · MJCF 를 읽는 규칙 · 읽지 않는 것: [docs/mjcf-urdf-compare.md](docs/mjcf-urdf-compare.md).

### urdf_to_mjcf

URDF 또는 xacro 를 MuJoCo MJCF 로 변환한다. 관절을 active / passive mimic / closed-chain / fixed 로 분류하고, closed-chain 관절은 제거한 뒤 `<equality>` 제약으로 넣는다. active 관절마다 gain 없는 `<general>` actuator 를 만든다 (토크 motor 로 컴파일되며 URDF `effort` 가 `forcerange` 가 된다).

```bash
# 디렉토리 규약 (<robot-dir>/urdf, mjcf, meshes 를 자동 탐색)
ros2 run rtc_tools urdf_to_mjcf --robot-dir robot_descriptions/robots/ur5e_assm_v1 \
    --urdf-file ur5e_with_hand.urdf.xacro
# 명시적 입출력 + xacro 인자
ros2 run rtc_tools urdf_to_mjcf --input robot.xacro --output robot.xml --xacro-args name:=ur5e
# scene.xml 생성 + 변환 후 MJCF/URDF 검증
ros2 run rtc_tools urdf_to_mjcf --robot-dir robot_descriptions/robots/ur5e_assm_v1 --scene --validate
```

`--robot-dir` 은 `urdf/` (`<robot_name>.urdf` 우선) · `mjcf/` (출력) · `meshes/` (`assets/` 의 OBJ 우선) 를 가진 디렉토리다. 출력은 그 `mjcf/` 에 쓰이므로 저장소 안의 로봇 디렉토리에 돌릴 때는 `--output` 으로 경로를 바꾼다.

### ctf_to_chrome_trace

`enable_tracing:=true` 로 수집한 LTTng CTF trace 를 [Perfetto UI](https://ui.perfetto.dev) 에 드래그-드롭할 수 있는 Chrome Trace JSON 으로 바꾼다. `console_scripts` 가 아니므로 모듈로 부른다.

```bash
python3 -m rtc_tools.conversion.ctf_to_chrome_trace \
    --input logging_data/<YYMMDD_HHMM>/tracing/<session_name> --output trace.json
babeltrace2 <trace_dir> | python3 -m rtc_tools.conversion.ctf_to_chrome_trace --stdin --output trace.json
```

- Perfetto 에 **Threads (by TID)** 와 **Cpus** 두 swimlane 그룹이 생긴다 (스레드별 실행 구간, core 별 스케줄링 — taskset 핀 검증 · migration/IRQ 탐지).
- 처리 이벤트: `ros2:callback_start/end` · `rtc:span_begin/end` (`RTC_TRACE_SCOPE`, `-DRTC_ENABLE_TRACING=ON` 빌드에만) · `sched_switch` · `irq_handler_entry/exit`. 그 외는 버리고 `--keep-events a,b` 로 고르거나 `--keep-all` 로 전부 복원한다. `--max-duration-s N` 은 앞 N 초만 변환한다.
- callback 이름은 `ros2:rclcpp_callback_register` 의 주소→symbol 매핑으로 푼다. 그 이벤트가 없는 캡처는 `callback@0x…` 로 남는다.
- 파서는 `python3-bt2` 를 우선 쓰고 없으면 `babeltrace2` CLI 텍스트로 폴백한다 (느림).
- trace 를 수집하는 법: [docs/tracing.md](../docs/tracing.md).

### hand_udp_sender_example

개발·테스트용 UDP 손 통신 라이브러리 + 합성 데이터 생성기. `HandUDPSender` 가 request-response 통신, CSV 로깅, 장애 감지를 맡는다.

```bash
ros2 run rtc_tools hand_udp_sender_example
```

대화형 프롬프트에서 대상 IP, 센서 수 (0~4), CSV 저장 여부, 실행 모드를 고른다. 실행 모드는 6 가지다: WriteOnly · PollCycle · StaticPose · ReadOnly · BulkPollCycle · BulkReadOnly.

| 패킷 | 크기 | 구조 |
|---|---|---|
| 모터 패킷 | 43 B | `[ID:1][CMD:1][MODE:1][10 × float32]` |
| 모터 일괄 응답 | 123 B | `[ID][CMD][MODE][30 × float32]` (pos + vel + cur) |
| 센서 요청 | 3 B | `[ID][CMD][MODE]` |
| 센서 응답 | 67 B | `[ID][CMD][MODE][16 × uint32]` (유효 11: 기압 8 + ToF 3) |
| 센서 일괄 응답 | 259 B | `[ID][CMD][MODE][64 × uint32]` (4 핑거) |

명령 코드: `WritePosition 0x01` · `SetSensorMode 0x04` · `ReadAllMotors 0x10` · `ReadPosition 0x11` · `ReadVelocity 0x12` · `ReadSensor0-3 0x14-0x17` · `ReadAllSensors 0x19`. joint 관련 명령의 `MODE` 는 `JOINT_MODE_MOTOR 0x00` (raw motor encoder, 기본) 또는 `JOINT_MODE_JOINT 0x01` (기어비 적용 joint-space) 이다. 프로토콜의 SSoT 는 이 모듈이고 `HandDataFailureDetector` 는 연속 0 데이터나 중복 데이터가 5 회를 넘으면 자동 종료한다.

### hand_data_plot

`hand_udp_sender_example` 이 저장한 CSV 를 그린다 (센서 수와 bulk/legacy 모드 자동 감지). `console_scripts` 가 아니다.

```bash
python3 -m rtc_tools.utils.hand_data_plot <csv_file>                     # 모터 + 센서 + 타이밍
python3 -m rtc_tools.utils.hand_data_plot <csv_file> --motors 0 1 2      # 특정 모터만
python3 -m rtc_tools.utils.hand_data_plot <csv_file> --sensors-only      # 또는 --motors-only · --timing-only
```

### derive_accel_limits

토크 한계 $\tau_{max}$ 에서 관절 가속 상수 box 를 구한다 (`catching/search_grid.yaml` 의 `robot.arm.qdd_max` 를 정할 때 쓰는 분석 도구). 출력은 컨트롤러 입력이 아니다.

```bash
ros2 run rtc_tools derive_accel_limits \
  --robot-config integrated_bringup/config/ur5e_p1b/_base.yaml --group ur5e \
  --eta-tau 0.8 --samples 20000 --check rnea --out /tmp/ur5e_p1b_accel_limits.yaml
```

`--eta-tau` 는 기본값이 없다. 종료 코드: 퇴화 (`adopted: false`) 면 `2`, 교차 검증 실패면 `3`. `--check mujoco --mjcf <xml>` 은 venv 의 `mujoco` 가 필요하므로 `.venv/bin/python -m rtc_tools.analysis.derive_accel_limits` 로 부른다. 판정식과 해석: [규칙](../docs/catching-analysis-rules.md#derive_accel_limits).

### clock_phase · hand_close · vision_lane

sim 의 측정 lane 에서 clock 위상, 손 폐쇄 시간, 예측 lane 을 잰다.

```bash
ros2 run rtc_tools run_clock_phase_trials --trials 200          # /sim/launch_ball_at 지정 발사 + 회수 반복
ros2 run rtc_tools analyze_clock_phase clock_lane.csv --plot out.png

ros2 run rtc_tools run_hand_close_trials --group <hand_group> --joint-names "<j1,j2,…>" --trials 200 --out run.json
ros2 run rtc_tools analyze_hand_close <session>/controllers/<controller>/<hand>_state.csv \
    --profile run.json --plot out.png

ros2 run rtc_tools vision_lane_probe <prefix>                   # <prefix>_{prediction,camera,truth,diag}.csv
ros2 run rtc_tools vision_lane_probe <prefix> --dump            # + 전 지평 점 · innovation · nis
ros2 run rtc_tools camera_relay --drop-after-s 0.4              # 또는 --drop-prob p / --delay-s d
ros2 run rtc_tools analyze_vision_lane <prefix>
```

`sim_estimator_node` 는 `debug.enabled_topics` 에 `prediction/trajectory` 만 있으면 발행이 0 건이다. 다른 debug 토픽을 하나 더 켠다. [판정 규칙](../docs/catching-analysis-rules.md#clock_phase).

### hand_canfd_gate

`udp_hand.launch.py protocol_version:=2` 를 실제 손에 대고 돌린 한 run 의 세션 디렉토리 (`device/hand_stats*.json` · `timing/hand_timing_log*.csv`) 와 `candump -L` 로그로 G-3 의 다섯 항목을 PASS / FAIL / UNDECIDED 로 판정한다 (문턱은 모두 옵션, 기본값이 D55).

```bash
candump -L -d -t a 'can0,0:0,#FFFFFFFF' > g3.log &          # 노드 기동 전부터 E-Stop 뒤까지
ros2 run rtc_tools analyze_hand_canfd_gate <session> g3.log --node-log node.log \
    --link-status link.log --hand-side right [--explicit-side] [--json-out g3.json]
```

- (a) 기간 ≥ 300 s · run_id 1 · tick 누락 0 — (b) link down 0: JSON `link_ok`, 노드 로그 "Hand link DOWN" 0, 완전한 RESP 세트 없는 연속 REQ < `--link-fail-timeout-ms` × 주기 수 (500 Hz 50 · 250 Hz 25), `link_status` false 0 — (c) `t_total_us` p99 < 0.8 × 주기, 제외 없음 — (d) RESP 세트 타임아웃 / 사이클 과 echo 타임아웃 / write 시도 가 **각각** < 0.1 % (집계 `recv_timeout` 은 상한만) — (e) candump 의 Frame 1 zeros → Frame 0 zeros (정상 write 와 반대 순서) → echo 2 개 가 2 ms 안, 그 뒤 명령 없음.
- `cmd_ack` (JSON `canfd.cmd_ack`, 없으면 `echo`): `none` 이면 echo 타임아웃은 `NOT_EVALUATED`, (e) 는 두 zeros 의 순서 · 간격 < 2 ms · 이후 명령 0 으로 판정한다.
- 두 손 (G-5): 버스에 두 손이 보이면 (x) `x.no_overlap` 이 붙는다 — 한 손의 트랜잭션 (첫 호스트 프레임 … RESP F2) 안에 다른 손의 echo · RESP 가 0. 첫 E-Stop 프레임부터는 보지 않고, 두 손 zero-write 간격은 보고만 한다. 손마다 `--hand-side <side> --explicit-side --timing-suffix ""` 로 한 번씩 돌린다. run 전체는 `repo_scripts/scripts/run_hand_canfd_gate.sh` ([testing.md](../docs/testing.md)). candump 없는 PC 는 `python3 -m rtc_tools.analysis.candump_log <iface> <out.log>`.
- `--req-mode auto|per_hand|broadcast` (`auto` = `canfd.req_mode`, 없으면 `per_hand`): `broadcast` 면 0xFF REQ 가 두 손의 REQ 이고 (x) 는 `x.broadcast_sets` (RESP 중복 · 버스트 중 CMD · 한 손만 빠진 창 비율).
- (d) 는 문턱 건수 ±1.65 σ 안이면 UNDECIDED (한 번 재측정). 못 본 항목은 `NOT_EVALUATED(<사유>)` 이고 통과 아님. 종료 코드 0 / 1 (FAIL) / 2 (UNDECIDED) / 3 (입력 불가 · `req_mode` 불일치).

### catching_trials

`catching_sim_trials` 한 run 의 세션 CSV · trials dir · (`sim_lanes:=true` 의) clock·접촉 lane 을 시행당 한 행으로 잇는다.

```bash
ros2 run rtc_tools catching_trials <session> <trials_dir> --config-dir $CFG \
    --urdf <xacro 로 전개한 urdf> --v-max 3.85 --eps-mm 12 22.8 41.8 49.8 --hold-window-s 0.1 \
    --floor 0.35 --n-valid-target 200 \
    --eval-samples <unit>/eval_samples.csv --eval-report <unit>/eval_report.json \
    --probe-dump <unit>/probe/lane_prediction_dump.csv
# → <trials_dir>/catching_trials/{catching_trials.csv, catching_trials_summary.json, hand_hold_window.csv}
```

- 필수는 `session` · `trials_dir` · `--config-dir` 이고 나머지는 선택이다. `--v-max` 는 기본값이 없다 (없으면 clock 위상 공변량이 `NOT_EVALUATED`).
- 무효 시행 (rig 실패) · truth 성공 · $t_c$ 분해의 규칙: [catching_trials](../docs/catching-analysis-rules.md#catching_trials).

### catching_pool

arm 마다 여러 unit 의 `catching_trials` 출력을 합쳐 truth 성공 판정 · ITT · McNemar 를 낸다.

```bash
ros2 run rtc_tools catching_pool \
    --arm tennis  ct/601 ct/602 ct/603 ct/604 ct/605 \
    --arm beanbag ct_bb/601 ct_bb/602 ct_bb/603 ct_bb/604 \
    --floor 0.35 --n-valid-target 200 --out pool/ --extra-d3 s8b_tuning/catching_trials_summary.json
# → pool/{pool_summary.json, pool_trials.csv} + stdout 보고
```

CT_DIR 은 **준 순서대로** (사전 선언 seed 순, 보충 unit 은 마지막) 읽는다. [규칙](../docs/catching-analysis-rules.md#catching_pool).

### catching_hand_near

`catching_sim_trials --dist hand_*` 로 던진 unit 에서 plan 이 있었는가 (COMMIT) 와 있을 때 잡았는가 (CATCH) 를 따로 적합한다. unit 은 `<unit>/trials` + `<unit>/ct` 이고 `<unit>:<planner_events.csv>` 로 계획기 기록을 붙일 수 있다.

```bash
ros2 run rtc_tools catching_hand_near \
    units/cliff_rf units/lhs_rf:$SES/controllers/<controller>/planner_events.csv units/lhs_ss \
    --ab <arm_a> <arm_b> --out hn/
# → hn/{hand_near_summary.json, hand_near_trials.csv, v50_map.csv, wilson_cells.csv}; rc 1 이면 조준 오차 초과
```

`--ab` 의 두 이름은 unit 의 `run_meta.json` 에 남은 arm 라벨이다. [규칙](../docs/catching-analysis-rules.md#catching_hand_near).

### catching_arm_budget

"팔이 못 간다" 를 플랜트 · 참조 · CLIK · 계획기 네 층으로 나눠 귀속시킨다. `<unit>[:<session>]` (세션 기본 `<unit>/session_copy`).

```bash
ros2 run rtc_tools catching_arm_budget units/w10_a21 units/w15_a30 --config-dir $CFG --out ab/ \
    [--overlay <overlay.yaml>] [--write-envelope-box <sim_override.yaml>]
# → ab/{arm_budget_summary.json, arm_budget_trials.csv}; 리포트는 stdout
```

`--write-envelope-box` 는 실행된 가속 envelope 를 sim override 조각 (`robot.arm.qdd_max`) 으로 쓴다 (sim 전용). [규칙](../docs/catching-analysis-rules.md#catching_arm_budget).

### catching_decel

포구 뒤 DECEL 정지 구간의 가속 · jerk · 정지 거리 · 한계 여유를 잰다. `--a` / `--b` 로 같은 투척의 두 arm 을 짝지어 2×2 표 · 불일치율 · paired 비열등 검정을 낸다.

```bash
ros2 run rtc_tools catching_decel units/p1b_601 units/p1b_602 --config-dir $CFG --overlay <overlay.yaml> --out decel/
ros2 run rtc_tools catching_decel --a units/*_a --b units/*_b --config-dir $CFG --overlay <overlay.yaml> --out decel/
ros2 run rtc_tools catching_decel --a units/*_a --b units/*_b --same-arm --config-dir $CFG --out decel/   # 같은 arm 의 복제
# → decel/{decel_summary.json, decel_trials.csv}; 리포트는 stdout
```

규칙 (창의 정의 · `--success hold` · Tango 검정): [catching_decel](../docs/catching-analysis-rules.md#catching_decel).

### catching_grid_sweep

vision 예측 격자 조건을 바꿔 같은 투척을 던진 unit 들을 조건 = arm 으로 비교한다. 로봇마다 따로 돌린다.

```bash
ros2 run rtc_tools catching_grid_sweep --arm L-50 units/L-50_* --arm L-25 units/L-25_* \
    --ref L-50 --pair L-50:L-25 --budget-s 0.020 --out sweep/
# → sweep/{grid_sweep_summary.json, grid_sweep_trials.csv}; 리포트는 stdout
```

[규칙](../docs/catching-analysis-rules.md#catching_grid_sweep).

### catching_wait_pose_search

공이 보이기 전에 팔이 서 있는 대기 자세 (`planner.wait_pose`) 를 바꾸면 방향 속력 상한이 얼마나 오르는지 탐색한다.

```bash
ros2 run rtc_tools catching_wait_pose_search --config-dir $CFG \
    --radius-m 0.15 --axis-tol-deg 2 --samples 20000 --seed 20260928 --out <out>
ros2 run rtc_tools catching_wait_pose_search --config-dir $CFG --box '-1.1 -1.1 0.15 1.1 1.1 1.2' --axis-tol-deg 10 --out <out>
# 자세 하나만 확인 (탐색 없음, --out 불필요)
ros2 run rtc_tools catching_wait_pose_search --config-dir $CFG --evaluate-pose '0.212 -1.376 1.107 -1.978 -3.296 0.121'
```

`--objective` 는 `dls` (기본) · `lp` · `robust` 다. 출력은 `wait_pose_candidates.csv` 와 `wait_pose_search_summary.yaml` (`overlay_snippet` 포함). 검증이 하나라도 실패하면 아무것도 쓰지 않는다. [규칙](../docs/catching-analysis-rules.md#catching_wait_pose_search).

### catchability_map · catch_speed_budget · catch_gate_map · catch_search_map

투척 grid 에서 시작해 계획기의 판정을 오프라인으로 재현하는 파이프라인이다. 판정은 python 이 재구현하지 않고 `rtc_controllers` 의 C++ 실행파일 (`catch_pose_ik_batch` · `catch_gate_batch` · `catch_search_batch`) 을 CSV 로 호출한다 — 이 실행파일들이 빌드·설치되어 있어야 한다.

```bash
# 1. kinematic 지도: 투척 grid → 항력 비행 → 포구 후보 → judge
ros2 run rtc_tools catchability_map \
  --robot-config $CFG/_base.yaml $CFG/sim.yaml --ball-config $CFG/mujoco_simulator.yaml \
  --drag-coefficient 0.55 --drag-coefficient-source 'rtc_mujoco_sim/src/projectile_ball.cpp:30' \
  --air-density 1.204 --air-density-source 'rtc_mujoco_sim/include/rtc_mujoco_sim/projectile_ball.hpp:95' \
  --distances-m 4.0 --azimuths-deg '180 190 200' --release-heights-m 1.8 \
  --aim-deviations-deg 0.0 --speeds-m-s '5.4 6.0' --elevations-deg '40 48' \
  --window-s 1.0 1.8 --stride-s 0.05 --min-flight-time-s 1.0 \
  --max-reach-m 1.1 --seed '0.0 -1.4 0.9 -1.9 -1.5708 0.0' --workers 6 --epsilon-m 0.01 --out-dir <map>

# 2. 수락 후보의 팔 속도 · 토크 한계 가속 예산 → γ 창이 열리는 투척 표
ros2 run rtc_tools catch_speed_budget --robot-config $CFG/_base.yaml --group <arm_group> \
  --map-dir <map> --out-dir <budget> --velocity-source model --eta-v 0.9 --eta-tau 0.8 \
  --rotor-inertia '0.1 0.1 0.1 0.1 0.1 0.1' --rotor-inertia-source '<mjcf>:<line> armature' \
  --max-reach-m 1.1 --floor-world-z-m 0.1 \
  --detection-s 0.10 --latency-s 0.14 --close-total-s 0.279 --arm-delay-s 0.05 --time-margin-s 0.02 \
  --relative-speed-m-s '0.34 1.0'

# 3. 도달시간 · γ 창 · 정지점 게이트를 건 gate-catchable 지도
ros2 run rtc_tools catch_gate_map --robot-config $CFG/_base.yaml --group <arm_group> \
  --map-dir <map> --out-dir <gate> --velocity-source model \
  --controller-config $CFG/controllers/demo_catching_controller.yaml --overlay $CFG/sim.yaml \
  --eta-v 0.9 --eta-tau 0.8 --rotor-inertia '0.1 0.1 0.1 0.1 0.1 0.1' --rotor-inertia-source '<mjcf>:<line> armature' \
  --v-max-m-s derived --d-eff-m 0.279 --d-eff-source 'planner.search.grid.hand.d_eff' \
  --close-total-s 0.279 --gamma-margin-m-s 0.1 --a-dec-m-s2 10.0 --a-dec-source 'provisional' \
  --detection-s 0.10 --latency-s 0.14 --arm-delay-s 0.05 --time-margin-s 0.03 \
  --max-reach-m 1.1 --floor-world-z-m 0.1

# 4. 계획기 탐색 자체의 투척별 수용 지도 (grid · nlp)
ros2 run rtc_tools catch_search_map --config-dir $CFG --search grid nlp --out-dir <search> \
  --ball-config $CFG/mujoco_simulator.yaml \
  --drag-coefficient 0.55 --drag-coefficient-source 'rtc_mujoco_sim/src/projectile_ball.cpp:30' \
  --air-density 1.204 --air-density-source 'rtc_mujoco_sim/include/rtc_mujoco_sim/projectile_ball.hpp:98' \
  --distances-m '3.5 4.0' --azimuths-deg '-10 0 10' --release-heights-m 1.8 --aim-deviations-deg 0 \
  --speeds-m-s '6.0 6.5' --elevations-deg '40 46' --grid-origin wait_catch_point \
  --detection-delay-s 0.15 --vision-period-s 0.0333333 --prediction-horizon-s 1.0 \
  [--overlay <file>] [--sample 50 --sample-seed <seed>] [--gate-map <gate>] [--sim-trials <csv>] \
  [--throws-file <list.json> [--judge-screened]] [--judge-segments] [--jobs 6] [--estimator-profile <json>]
```

- 항력계수 · 공기밀도 · 반지름 · 질량은 기본값이 없다. 값과 출처 라벨 (`file:line`) 을 함께 준다 (C++ preset 에만 있고 YAML 로 노출되지 않아, python 에 박으면 preset 이 바뀌어도 아무것도 실패하지 않는다).
- 같은 투척 목록으로 세 도구를 잇는 법: `catchability_map … --throws-file <list.json> --out-dir <map>` → `catch_gate_map --map-dir <map> …` → `catch_search_map --throws-file <list.json> --gate-map <gate> …`.
- `catchability_map` 출력: `candidates.csv` · `throw_summary.csv` · `reason_histogram.csv` · `throw_region.yaml` · `provenance.yaml` · `azimuth_coverage.png` · `manipulability.png`. `catch_search_map` 출력: `verdicts_<search>.csv` · `search_map_summary.json` 외. headline 규약 (seed 하나의 수치, 분모는 전체 격자): [규칙](../docs/catching-analysis-rules.md#catchability_map).
- `catchability_map` 은 python 에서 import 해서도 쓸 수 있다 (`rtc_tools.analysis.catchability_map`: `integrate_flight` · `generate_throw_grid` · `run_judge_batch` · `summarize_throws` 등).

### catch_throw_design · catch_population

통과점을 지나는 투척을 설계하고 (`design`), 부분집합을 고르고 (`select`), 지도가 수용한 투척의 plan 집단을 만들어 (`plan-set`) 층화 추출한 던질 묶음을 쓴다 (`sample`). `catch_population` 은 이 과정을 설정 파일 하나로 순서대로 돌리고 `catch_population_report` 가 결과를 요약한다.

```bash
ros2 run rtc_tools catch_throw_design design --config-dir $CFG --out-dir <design> [--overlay <file>] \
  --ball-config $CFG/mujoco_simulator.yaml \
  --drag-coefficient 0.55 --drag-coefficient-source '<file:line>' --air-density 1.204 --air-density-source '<file:line>' \
  --distances-m 1.5:4.0:0.25 --azimuths-deg=-40:40:10 --release-heights-m 1.0:1.7:0.1 \
  --pass-dx-m=-0.4:0.4:0.1 --pass-dy-m=-0.4:0.4:0.1 \
  --max-elevation-deg 70 --min-elevation-deg 5 --elevation-step-deg 5 [--min-pass-time-s 0.5] \
  --speed-max-m-s 12 --detection-delay-s 0.15 [--segment-mode mpc_docking] [--jobs 6]
ros2 run rtc_tools catch_throw_design select --design-dir <design> --out <list.json> \
  --subset 'distance_m=1.5 2.0 2.5 3.0 3.5 4.0' --subset 'azimuth_deg=-40 -20 0 20 40'
ros2 run rtc_tools catch_search_map --throws-file <list.json> --judge-screened --jobs 6 …
ros2 run rtc_tools catch_throw_design plan-set --throws-file <list.json> --verdicts <map>/verdicts_grid.csv --out <plan_set.json>
ros2 run rtc_tools catch_throw_design sample --plan-set <plan_set.json> --out-dir <sample> \
  --strata distance_m elevation_deg --seed 1 --max-throws 2500 --chunk 250

ros2 run rtc_tools catch_population --config $CFG/catch_population.yaml --out <result> \
  [--jobs 10] [--judge <catch_search_batch>] [--arms grid_mpc …] [--report-only] [--show] [--max-flights 400]
ros2 run rtc_tools catch_population_report <result> [--show] [--max-flights 400]
```

- 모집단 파이프라인: 설계 → 제한 → 모집단 → 탐색 → search 집단 → 구간 → segment 집단 → 뽑기 → 던질 묶음 → (2 단계: `catching_sim_trials` 로 sim) → 쌍 게시 → 포구.
- 결과 폴더는 설정 하나의 것이다. 이어 돌리면 끝난 단계를 건너뛰고, 설정·입력 YAML 이 바뀌었으면 바뀐 파일을 대며 거부한다 (새 `--out` 을 쓴다). 설정 파일의 형식은 `rtc_tools/utils/catch_population.py` 의 docstring 이다.
- 던질 묶음 (`<조합>/throws/chunk_NNN.json`) 은 `catching_sim_trials --throws-file` 이 읽는다 ([integrated_bringup](../integrated_bringup/README.md)).
- 설계 열 · 사전 판정 · 집단 규칙: [catch_throw_design](../docs/catching-analysis-rules.md#catch_throw_design) · [catch_population](../docs/catching-analysis-rules.md#catch_population).

### launch 헬퍼와 utils 모듈

다른 패키지의 launch 파일·도구가 import 하는 라이브러리다 (CLI 없음).

| 모듈 | 하는 일 |
|---|---|
| `launch/session.py` | launch context 안에서 세션 디렉토리를 열고 보존 정책을 읽는다 (`open_session`) |
| `launch/thread_layout.py` | 코어 배치 SSoT 의 Python mirror (`select_thread_layout` · `get_*_core`). 표는 [thread_layout.yaml](../repo_scripts/config/thread_layout.yaml) 에서 생성된 `thread_layout_generated.py` 다 |
| `launch/pinning.py` | slot index → logical CPU 로 풀어 `taskset` 핀 · DDS co-pin · shield adopt 액션을 만든다 |
| `launch/cpu_shield.py` | cset shield 감지 + adopt → ACTIVATE lifecycle 체인 (bringup launch 공용, fail-closed) |
| `launch/cm_rt_params.py` | controller_manager RT 파라미터 파일 생성 (UR arm 루프 pin) |
| `launch/trace_action.py` | `make_trace_action()` — `enable_tracing` 계열 launch 인자로 ros2_tracing capture 액션을 만든다. 미설치면 `./install.sh --tracing` 을 안내하고 no-op |
| `utils/session_dir.py` | 세션 디렉토리 규칙 (아래) |
| `utils/controller_config.py` | 컨트롤러 YAML 의 `include:` 합성 (아래) |
| `utils/smoothing.py` · `utils/rotations.py` · `utils/catching_keys.py` | 분석 모듈이 공유하는 box 평활 · RPY→회전행렬 · 옛 열 이름 별칭 |

```python
from rtc_tools.launch.thread_layout import select_thread_layout
from rtc_tools.launch.trace_action import make_trace_action

layout = select_thread_layout()          # physical core 자동 감지
arm_core = layout.arm_driver_core        # -1 = pinning 생략
actions = make_trace_action(context, session_dir=session_dir)   # [] 가능
```

**`controller_config`**: 컨트롤러 YAML 은 `include:` 로 여러 파일에 나뉠 수 있다 (규칙: [rtc_controller_manager](../rtc_controller_manager/README.md#컨트롤러-yaml-을-여러-파일로-나누기--include)). 경로로 컨트롤러 YAML 을 읽는 도구와 테스트는 `yaml.safe_load` 대신 `load_controller_config(path, *, config_key=None)` 를 쓴다. 주 파일만 읽으면 조각의 키가 조용히 빠진 트리를 본다. C++ 로더의 mirror 이므로 규칙을 바꾸면 `rtc_controller_manager/src/controller_config_loader.cpp` 와 함께 바꾼다 (PROC-5). `controller_config_leaf_lines` · `unread_leaves` 등 보조 함수는 모듈 docstring 을 본다.

**`session_dir`**: C++ `rtc_base/logging/session_dir.hpp` 와 같은 결정 로직이다. 로깅 루트 (`resolve_logging_root`) 는 3 단 체인이다.

1. 로깅 루트: `$COLCON_PREFIX_PATH` 첫 entry 가 쓰기 가능하면 그 `parent/logging_data` → cwd 에서 위로 올라가며 `install/` + `src/` 쌍을 찾으면 그 디렉토리의 `logging_data` → 최종 폴백 `$PWD/logging_data`.
2. 세션 디렉토리: `$RTC_SESSION_DIR` → 없으면 로깅 루트 아래 `YYMMDD_HHMM` (프로세스당 한 이름, `create_session_dir` 는 매번 새로). 런 ID (`generate_run_id`, `YYMMDDHHMMSS`) 는 launch 가 `RTC_RUN_ID` 로 전파한다.

```python
from rtc_tools.utils.session_dir import (create_session_dir, cleanup_old_sessions,
                                         get_or_create_session_dir, get_session_subdir)
root = resolve_logging_root();  session = create_session_dir(root);  cleanup_old_sessions(root, max_sessions=10)
session = get_or_create_session_dir()      # CLI 도구: 실행 중 세션을 쓰거나 없으면 새로 만든다
plots = get_session_subdir("plots")        # 환경변수만 읽는다, 없으면 None
```

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| `plot_rtc_log --save-dir …` 가 끝나지 않는다 | PNG 를 쓴 뒤 GUI 창을 닫을 때까지 기다린다 | `--no-show` 를 함께 준다 |
| `compare_mjcf_urdf` 가 `UNVERIFIED: 1` 을 찍고 `Mismatches: 0` | `ros2 run` 의 `/usr/bin/python3` 에 `mujoco` 가 없어 구조 비교가 빠졌다 (통과가 아니다) | `setup_env.sh` 를 source 한 뒤 `python3 -m rtc_tools.validation.compare_mjcf_urdf … --fail-on-unverified` |
| `catchability_map` 이 judge 실행파일을 못 찾는다 | `rtc_controllers` 의 `catch_*_batch` 가 빌드·설치되지 않았다 | `rtc_controllers` 를 빌드하거나 `--judge <경로>` 를 준다 |
| `catching_*` 가 "다시 만들라" 며 멈춘다 | `decel_*` 이름의 옛 출력 또는 `invalid_reason` 열이 없는 CSV | 그 unit 에 `catching_trials` 를 다시 돌린다 |
| `vision_lane_probe` 가 예측을 한 건도 못 받는다 | `debug.enabled_topics` 에 `prediction/trajectory` 만 켜져 있다 | 다른 debug 토픽을 하나 더 켠다 |
| `colcon test` 에서 `test_real_model_pairs` 가 skip | colcon 이 `/usr/bin/python3` 로 pytest 를 돌려 `mujoco` 를 못 찾는다 | [robot_descriptions](../robot_descriptions/README.md) 의 문제 해결 절 |

## 더 읽기

- 분석 CLI 의 판정 규칙: [catching-analysis-rules.md](../docs/catching-analysis-rules.md) · 시나리오: [catching-sim-trials.md](../docs/catching-sim-trials.md)
- MJCF/URDF 비교 레퍼런스: [docs/mjcf-urdf-compare.md](docs/mjcf-urdf-compare.md)
- trace 수집: [docs/tracing.md](../docs/tracing.md) · 테스트: [docs/testing.md](../docs/testing.md)

## 의존성

[package.xml](package.xml) 이 ROS 쪽 (`rclpy` · `rtc_msgs` · `ament_index_python` 외) 을 갖는다. numpy · scipy · matplotlib · pandas · PyQt5 · `mujoco` 는 cross-workspace 격리 정책상 `package.xml` 이 아니라 venv ([requirements.in](../requirements.in) → [requirements.lock](../requirements.lock)) 책임이다. `rclpy` 만 ROS Jazzy 가 제공한다. MIT.

테스트는 `./build.sh --tests -p rtc_tools` 로 빌드한 뒤 `colcon test --packages-select rtc_tools` 다. `colcon.pkg` 가 pytest 에 `-n 4` 를 넘긴다 ([testing-debug.md](../agent_docs/testing-debug.md) "패키지 안 병렬").
