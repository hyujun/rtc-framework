# rtc_tools


> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md)

RTC 프레임워크의 **Python 개발 유틸리티 패키지**입니다. 로그 시각화, UDP 손 통신, MJCF/URDF 모델 변환 및 검증, 트레이싱/스레드 배치 launch 헬퍼 도구를 포함합니다 (레거시 컨트롤러 GUI는 아래 참고 참조).

## 개요

```
rtc_tools/
├── rtc_tools/
│   ├── gui/
│   │   └── (empty)                       ← controller_gui.py 없음 (아래 참고 참조)
│   ├── monitoring/
│   │   └── __init__.py
│   ├── plotting/
│   │   └── plot_rtc_log.py              ← Matplotlib 로그 시각화 (v5, 4-카테고리) —
│   │                                       thin orchestration layer; 실제 구현은
│   │                                       columns/, io/, pipelines/, plotters/,
│   │                                       layout.py 서브모듈로 분리되어 있음
│   │   └── zoom_dialog.py                 ← 우클릭 → x/y 범위 입력 확대 (GUI 전용)
│   ├── validation/
│   │   └── compare_mjcf_urdf.py         ← MJCF vs URDF 파라미터 비교 검증
│   ├── analysis/
│   │   ├── derive_accel_limits.py       ← 토크 한계 → 관절 가속 상수 box 도출 (dynamic_catching D-16)
│   │   ├── clock_phase.py               ← sim↔steady 시계 위상 오차 δ·pause 분석 (D-3 / S3.1a)
│   │   ├── clock_phase_trials.py        ← 지정 발사 N회 러너 (D-3 시행 생성)
│   │   ├── catchability_map.py          ← catchability 지도: 공 비행 모델·투척 grid·프레임 변환
│   │   │                                   + C++ judge 배치 드라이버·집계·CLI (S3.5a)
│   │   ├── catch_speed_budget.py        ← 수락 후보별 팔 속도·토크 한계 방향 가속 → γ 창 판정표 (S4.4)
│   │   ├── catch_gate_map.py            ← kinematic 지도 위의 나머지 게이트 (도달시간·γ 창·정지점) → gate-catchable 지도 (S3.5b)
│   │   ├── vision_lane.py               ← ball_perception 예측 lane 디코더·요약 (D-4 / S3.4)
│   │   ├── vision_lane_probe.py         ← 예측·카메라·truth·diagnostics 를 CSV 로 기록 (S3.4) · `--dump` 전 지평·공분산·innovation·nis (S8-A)
│   │   ├── camera_relay.py              ← 카메라 lane 릴레이 + 드롭·지연 주입 (S3.4)
│   │   └── catching_trials.py           ← 포구 sim 시행 오프라인 평가: τ̂·t_c 분해·truth 성공·Wilson·D-3 공변량·접촉 (S8-A)
│   ├── conversion/
│   │   ├── urdf_to_mjcf.py             ← URDF/XACRO → MJCF 변환 (관절 분류 + 후처리)
│   │   └── ctf_to_chrome_trace.py      ← LTTng CTF trace → Chrome Trace JSON (Perfetto UI)
│   ├── launch/
│   │   ├── thread_layout.py            ← 스레드 코어 배치 SSoT의 Python mirror
│   │   │                                  (generated from thread_layout.yaml)
│   │   ├── pinning.py                  ← taskset pin / DDS co-pin / shield adopt 액션
│   │   ├── cpu_shield.py               ← cset shield 감지 + adopt→ACTIVATE 체인
│   │   │                                  (다섯 bringup launch 공유, fail-closed)
│   │   ├── cm_rt_params.py             ← controller_manager RT 파라미터 파일 생성
│   │   │                                  (UR arm 루프 pin)
│   │   └── trace_action.py             ← ros2_tracing (LTTng) capture 액션 헬퍼
│   └── utils/
│       ├── hand_udp_sender_example.py   ← 10-DOF 손 UDP 프로토콜 라이브러리 + 예제
│       ├── session_dir.py               ← 세션 디렉토리 유틸리티 (RTC_SESSION_DIR / RTC_RUN_ID 관리)
│       └── hand_data_plot.py            ← 손 CSV 데이터 시각화
├── test/                                 ← pytest 유닛 테스트 (Testing 섹션 참조)
├── resource/
│   └── rtc_tools
├── package.xml
├── setup.py
└── setup.cfg
```

**빌드 타입**: `ament_python` (`setup.py`의 `entry_points` 사용)

**Entry points** (18개 — SSoT 는 `setup.py` 의 `console_scripts`. 아래 표는 주요 항목이고,
`analysis/` 의 나머지 CLI 는 각 스크립트 절에서 호출 형태를 준다):

| 실행 명령 | 모듈 | 설명 |
|-----------|------|------|
| `ros2 run rtc_tools plot_rtc_log` | `plotting.plot_rtc_log` | CSV 로그 시각화 |
| `ros2 run rtc_tools plot_ur_log` | `plotting.plot_rtc_log` | plot_rtc_log의 별칭 |
| `ros2 run rtc_tools plot_ur_trajectory` | `plotting.plot_rtc_log` | legacy 별칭 |
| `ros2 run rtc_tools hand_udp_sender_example` | `utils.hand_udp_sender_example` | 핸드 UDP 테스트 (대화형) |
| `ros2 run rtc_tools compare_mjcf_urdf` | `validation.compare_mjcf_urdf` | MJCF/URDF 파라미터 비교 |
| `ros2 run rtc_tools urdf_to_mjcf` | `conversion.urdf_to_mjcf` | URDF/XACRO → MJCF 변환 |
| `ros2 run rtc_tools derive_accel_limits` | `analysis.derive_accel_limits` | 토크 한계에서 관절 가속 상수 box 도출 (provenance YAML) |
| `ros2 run rtc_tools catchability_map` | `analysis.catchability_map` | catchability 지도 (grid → 비행 → C++ judge → 집계·플롯) |
| `ros2 run rtc_tools catch_speed_budget` | `analysis.catch_speed_budget` | 지도의 수락 후보별 v_dir,max (LP·DLS)·토크 한계 방향 가속·stroke → γ 창이 열리는 투척 표 |
| `ros2 run rtc_tools catch_gate_map` | `analysis.catch_gate_map` | kinematic 지도의 수락 후보를 `catch_gate_batch` (런타임 게이트 함수) 로 판정 + 토크 검사 도달시간 층 → 두 층의 gate-catchable 지도·탈락 사유·대기 자세 제안 |
| `ros2 run rtc_tools catching_trials` | `analysis.catching_trials` | `catching_sim_trials` 한 run (세션 CSV + trials dir + sim lane) → 시행별 표·요약 JSON (S8-A) |

**Python 의존성**: `rclpy`, `std_msgs`, `sensor_msgs`, `rtc_msgs`, `numpy`, `matplotlib`, `pandas`, `scipy`, `mujoco`

---

## 스크립트 설명

> **Note:** `controller_gui.py`는 없습니다 — 세 데모 컨트롤러(DemoJoint/DemoTask/DemoWbc)용 GUI는
> [integrated_bringup/scripts/demo_controller_gui.py](../integrated_bringup/scripts/demo_controller_gui.py)에 있습니다.

### `plot_rtc_log.py` — 로그 시각화 (v5, 4-카테고리)

분리된 CSV 제어 로그를 Matplotlib으로 시각화합니다. 파일 이름 패턴으로 로그 타입을 자동 감지합니다.

```bash
# State 로그 (DeviceStateLog) 시각화
ros2 run rtc_tools plot_rtc_log <device>_state_log.csv

# Sensor 로그 (DeviceSensorLog) 시각화
ros2 run rtc_tools plot_rtc_log <device>_sensor_log.csv

# 타이밍 로그 시각화 (CM RT loop / MPC main loop — 동일 8-col 스키마)
ros2 run rtc_tools plot_rtc_log cm_timing_log.csv
ros2 run rtc_tools plot_rtc_log mpc_timing_log.csv

# 컨트롤러 소유 진단 CSV (<session>/controllers/<config_key>/ 아래)
ros2 run rtc_tools plot_rtc_log momentum_observer.csv
ros2 run rtc_tools plot_rtc_log momentum_observer.csv --stats   # ‖r‖∞ 통계만
ros2 run rtc_tools plot_rtc_log planner_events.csv              # 계획기 search 기록

# 한 파일에 두 런이 있을 때 (같은 분 재기동) 특정 런 선택 — 기본은 마지막 런
ros2 run rtc_tools plot_rtc_log cm_timing_log.csv --run-id 260808143052

# 플롯 파일로만 저장 (창 없이 — --no-show 가 없으면 저장 뒤 창을 띄우고 기다린다)
ros2 run rtc_tools plot_rtc_log <device>_state_log.csv --save-dir /tmp/plots --no-show

# 통계만 출력 (플롯 없이)
ros2 run rtc_tools plot_rtc_log <device>_state_log.csv --stats

# 모든 Figure 한 번에 생성
ros2 run rtc_tools plot_rtc_log <device>_state_log.csv --all
```

> `--save-dir` 미지정 시 **입력 CSV 를 담고 있는 세션의 `plots/`** 에 저장됩니다 —
> CSV 경로에서 상위로 올라가며 만나는 `YYMMDD_HHMM` 디렉토리가 기준이므로,
> 과거 세션 CSV 를 다시 그려도 figure 가 그 세션에 남습니다 (최신 세션으로
> 흩어지지 않음). CSV 가 세션 트리 밖이면 `RTC_SESSION_DIR/plots/` →
> 현재 ws `logging_data` 의 최신 세션 순으로 폴백하고, 그것도 없으면
> 저장 없이 GUI 표시만 합니다.
>
> GUI 창은 기본으로 뜬다 (`--show`). `--save-dir` 는 저장 위치만 정하므로, 그것만 주면
> PNG 를 쓴 뒤 창을 띄우고 **창을 닫을 때까지 종료하지 않는다** — 스크립트·백그라운드에서
> 부르면 멈춘 것처럼 보인다. 창 없이 PNG 만 쓰려면 `--no-show` 를 준다 (그때 Agg backend).
>
> 타이밍 CSV 가 `run_id` 를 여러 개 담고 있으면 (세션 디렉토리는 분 해상도라
> 같은 분의 재기동이 같은 파일에 append 된다) **마지막 런만** 그리고 무엇을
> 버렸는지 stdout 에 출력합니다 — 파일 전체로 `n / span` 을 내면 어느 런에도
> 없던 레이트가 나오기 때문입니다. 다른 런은 `--run-id <값>` 으로
> 선택하고, 없는 값이면 사용 가능한 목록과 함께 에러로 죽습니다.

#### 우클릭 확대 — 정확한 x/y 범위 입력

GUI 로 뜬 figure 의 **subplot 을 우클릭**하면 x/y 범위를 숫자로 입력하는 대화상자가
열립니다. "18.93 초부터 34.55 초까지" 처럼 드래그로는 맞출 수 없는 창을 그대로
타이핑할 수 있습니다 (`--show` 가 기본값이므로 그냥 실행하면 됩니다).

| 항목 | 동작 |
|---|---|
| `x min` / `x max` | 시간 창. **기본으로 그 figure 의 모든 subplot 에 적용** — 이 패키지가 만드는 figure 중 `sharex` 를 쓰는 것은 소수라, 다관절 그리드 (`robot_positions`, `motor_*`, `sensor_*`) 는 subplot 이 서로 독립이고 드래그 확대는 6칸 중 1칸만 바꿉니다 |
| `y min` / `y max` | 기본은 **우클릭한 칸에만** 적용 — 한 그리드 안에서도 단위가 갈릴 수 있습니다 (`wbc_task_trajectory` 는 x/y/z 가 m, roll/pitch/yaw 가 rad). 체크박스로 figure 전체 적용 |
| `y: fit to the x window` | y 를 **창 안의 데이터로만** 재적합. `Axes.autoscale()` 은 전 구간을 보므로 x 만 좁히면 창 밖 스파이크가 y 를 계속 차지합니다. `sharey` 그룹은 union 으로 계산해 열의 나머지 패널이 잘리지 않습니다 |
| `also save a zoomed PNG` | `<figure>_zoom_<xmin>-<xmax>.png` 를 같은 `plots/` 에 **새로** 씁니다 — 전 구간 `<figure>.png` 는 세션의 원본 기록이므로 절대 덮지 않습니다 |
| `Reset` | 대화상자를 붙인 시점의 범위로 복원 (`relim()` 은 collection 을 무시해 stackplot figure 가 안 돌아오므로 스냅샷을 되돌립니다) |

- **figure 여백**(패널 밖)을 우클릭하면 figure 스코프 — x 범위만 묻습니다.
- toolbar 의 pan / zoom 도구가 활성인 동안에는 우클릭이 matplotlib 자신의 제스처라 대화상자가 뜨지 않습니다. 도구를 해제하고 다시 우클릭하세요.
- 대화상자는 Tk → Qt → matplotlib 위젯 순으로 폴백하므로 `MPLBACKEND` 를 바꿔도 동작합니다.
- `--no-show` 에서는 이벤트 루프가 없어 확대가 불가능합니다 (PNG 만 생성).

**파일 이름 자동 감지:**

| 패턴 | 모드 |
|------|------|
| `*_state_log.csv` | state_log (DeviceStateLog 필드) |
| `*_sensor_log.csv` | sensor_log (DeviceSensorLog 필드, 컬럼 수 불일치 자동 복구) |
| `cm_timing_log*.csv` | cm_timing (CM RT loop) |
| `mpc_timing_log*.csv` | mpc_timing (MPC main loop) |
| `<dev>_state.csv` (WBC, `accel_*` 컬럼) | wbc_log (DeviceWbcLog — state_log superset: TSID a_opt 가속도 + SE3 trajectory(arm) / fingertip force(hand)) |
| `wbc_diag.csv` | wbc_diag (WbcDiagLog — per-tick TSID/QP 진단: solve time / λ / 수렴 / grasp) |
| `pull_estimator.csv` | pull_estimator (PullEstimatorLog — in-plane pull-force estimate: raw+filtered force / in-plane·magnitude·directional / friction util·leakage / validity 플래그 / 관측된 파지 형태 `opposing_mask`. 4×1 sharex 단일 figure + 통계) |
| `grasp_diag.csv` | grasp_diag (GraspDiagLog — per-tick Force-PI 서보 + 강성 추정 진단) |
| `compliance_diag.csv` | compliance_diag (ComplianceDiagLog — §7 task-admittance 진단: 소비된 wrench / source verdict / FSM·α / x̃·ν_c / 파라미터 스냅샷. 컬럼 지문은 `x_tilde_`. **통계 전용, figure 없음** — envelope·freshness·bias 숫자가 산출물이라 `grasp_diag` 와 같은 판단) |
| `momentum_observer.csv` | momentum_observer (MomentumObserverLog — 일반화 운동량 관측기 잔차 `r_<joint>`·‖r‖∞·게이트, Layer 2A payload wrench/질량, Layer 2B 관성 회귀. `momentum_observer.png` + (2A/2B 가 구성된 run 에서만) `momentum_payload.png` + 통계) |
| `catching_diag.csv` | catching_diag (CatchingDiagLog — 동적 포구 tick 레코드: 슈퍼바이저 mode/reason, 입력 스냅샷 token·나이·지평, plan, L4 기준(실현 가속도와 포화 전 요구값 둘 다), CLIK status/solve_us/conflict, q_c vs q. 컬럼 지문은 `track_err_rad` + `ref_gamma`. `catching_diag.png` + 통계 — **모든 tick 이 한 행**이라 tick 간극은 드롭된 행을 뜻한다) |
| `planner_events.csv` | planner_events (계획기 스레드의 non-idle wake 당 한 행 — 후보 funnel `n_in_window`/`n_ik`/`n_pass`, judgement-reject 히스토그램, 선택 후보의 rank-gate 실패 비트마스크(§4.8 rollout gate 포함), 스위칭 `decision`, `recv_to_publish_ms` (publish 된 행만). 컬럼 지문은 `n_in_window` + `rank_error_budget`. 시간축은 `wake_ns` 기준 첫 행 이후 경과초(이 채널에 `t_relative_s`/`t_wall_ns` 가 없음). `planner_events.png` (search/IK/rollout 시간·latency·funnel·reject 히스토그램·outcome/decision 이벤트·rank-gate 실패 6단) + 통계) |

> WBC `<dev>_state.csv` 는 파일명만으로 generic state_log 와 구분 불가 (둘 다 `_state`)
> → `accel_*` 컬럼 fingerprint 로 컬럼 fallback 단계에서 wbc_log 로 분류된다. wbc_log
> 는 robot/motor 플롯을 그대로 재사용하고 가속도·SE3·fingertip force 플롯을 추가한다.

**Robot 모드 플롯:**

| Figure | 플래그 | 내용 |
|--------|--------|------|
| Figure 1 | (기본) | 관절별 위치 (Goal / Trajectory / Actual) |
| Figure 2 | (기본) | 관절별 속도 (Trajectory / Actual) |
| Figure 3 | `--command` | 관절별 제어 명령 (Position/Torque) |
| Figure 4 | `--torque` | 관절별 실제 토크 |
| Figure 5 | `--task-pos` | TCP 태스크 위치 (X/Y/Z) |
| Figure 6 | `--error` | 위치/속도 추적 오차 |

**Device/Hand 모드 플롯:**

| Figure | 플래그 | 내용 |
|--------|--------|------|
| Figure 1 | (기본) | 모터별 위치 (Goal / Command / Actual) |
| Figure 2 | (기본) | 모터별 속도 (Actual) |
| Figure 3 | (기본) | 센서 (기압 + ToF per fingertip) |
| Figure 4 | `--raw` | Raw 센서 (pre-LPF, 기압 + ToF) |
| Figure 5 | `--ft` | F/T 추론 출력 (Force + Torque per fingertip) |
| Figure 6 | `--sensor-compare` | Raw vs Filtered 센서 오버레이 비교 |

**Force-only hand (`sensor_layout.values_per_group: 0`).** 기압/ToF lane 이 없는 핸드는 CSV 에 `<name>_raw_*` / `<name>_filt_*` 블록이 **아예 생성되지 않으며**, 그 부재를 gate 로 삼아 위 Figure 3·4 와 `device_ft_output` 대신 단일 `fingertip_force.png` 하나만 생성됩니다 (`device_ft_output` 은 4 행 중 3 행이 없는 채널이므로).

| Figure | 레이아웃 | 내용 |
|--------|---------|------|
| `fingertip_force.png` | N행(핑거팁) × 2열 | 좌: `Fx/Fy/Fz` raw(옅음) + LPF(진함) + guard hold 구간(파선), 우: `‖F‖` raw + LPF + guard hold tick 표시 |

- **Delta-spike guard 오버레이**: `ft_*_fx_guarded` / `ft_*_force_guard_rejected` 컬럼을 가진 세션에서만 그려집니다. 파선은 guard 가 raw 를 대체한 tick 구간만 (`np.where(rejected, guarded, nan)`) 표시하고, 우측 패널은 같은 tick 을 vline 으로 찍으며 좌측 제목에 hold tick 수를 적습니다 — guarded 는 거부 tick 을 뺀 나머지에서 raw 와 동일하므로 전 구간을 그리면 raw 위에 겹친 선 3개가 될 뿐입니다. guard 이전에 녹화된 세션은 컬럼 부재로 오버레이가 생략됩니다.
- `‖F‖` 는 컬럼이 아니라 성분에서 계산되며, **필터된 성분의 norm** 입니다 — 즉 컨트롤러가 실제로 임계와 비교하는 값과 같습니다 (`‖F_raw‖` 를 필터한 값이 아님).
- **확대 동기화**: 시간축은 8개 subplot 전체 공유 (`sharex="all"`), y축은 열 단위 공유 (`sharey="col"`) — 좌측은 부호 있는 성분, 우측은 비음수 크기라 하나로 묶으면 좌측 범위가 낭비됩니다.
- 확대는 **인터랙티브 백엔드에서만** 가능합니다 — `--show` 가 기본값이므로 그냥 실행하면 됩니다. `--no-show` 를 주면 Agg 로 전환되어 PNG 만 나옵니다. 드래그 말고 정확한 수치로 확대하려면 subplot 우클릭 (위 [우클릭 확대](#우클릭-확대--정확한-xy-범위-입력)).
- 이 스트라이드를 존중하기 전에 녹화된 세션은 0으로 채워진 블록을 그대로 갖고 있으므로 여기 매칭되지 않고 기존 기압/ToF figure 로 갑니다.

**Timing 모드 플롯:**

| Figure | 내용 |
|--------|------|
| Figure 1 | 제어 루프 타이밍 브레이크다운 |
| Figure 2 | 전체 루프 시간 + 지터 |
| Figure 3 | 타이밍 히스토그램 |

**v5 개선사항:**
- sensor_log CSV 컬럼 수 불일치 자동 복구 (헤더 < 데이터 행 시 inference 컬럼 재구성)
- 가변 DOF 자동 감지 (6-DOF 로봇 외에도 지원)
- 서브플롯 그리드 자동 계산

---

### `derive_accel_limits.py` — 토크 한계 → 관절 가속 box (dynamic_catching D-16)

가속 데이터는 없고 토크 한계는 있다는 전제에서, 팔 상태 (q, q̇) 표본마다
`Σ_j |M_ij| a_j ≤ η_τ τ_max,i − |g_i| − |c_i|` 를 만족하는 최대 `a = s·w` 를 구하고
(최악 부호 결합 — 2~4 배 보수적인 충분조건), 전 표본 최소값을 상수 box 로 낸다.
표본 최소값은 표본 수에 따라 계속 내려가므로 최악 표본 10개에서 박스 안 국소 최소화
(Powell) 로 정제한다. 판정식·절차는 `docs/dynamic_catching/IMPLEMENTATION_PLAN.md` §9.

```bash
ros2 run rtc_tools derive_accel_limits \
  --robot-config integrated_bringup/config/ur5e_p1b/_base.yaml --group ur5e \
  --eta-tau 0.8 --samples 20000 --check rnea \
  --out integrated_bringup/config/ur5e_p1b/derived_accel_limits.yaml
```

- 입력: 로봇 config 의 `devices.<group>.joint_state_names`·`joint_limits.{max_torque,max_velocity,position_*}`, `urdf.package/path` (xacro 확장). `--eta-tau` 는 기본값이 없다 (결정 사항)
- 표본 범위 기본값은 관절 한계 box 전체 (URDF ∩ config) 와 ±max_velocity. `--q-center/--q-halfwidth` 로 좁힌다
- 손 관절은 URDF 중립 자세·속도 0 으로 고정하고, 손 가속 결합 `max|M_arm,hand|` 는 provenance 에만 기록
- 퇴화 (τ_dyn ≤ 0 인 표본, 또는 s* < `--min-accel`) 면 `adopted: false` 로 쓰고 exit 2. 교차 검증 실패는 exit 3
- 교차 검증: `--check rnea` (부호 패턴 전부에 대해 RNEA 토크 ≤ η_τ τ_max), `--check mujoco --mjcf <xml>` (`mj_inverse` — MJCF armature·damping 포함, 접촉·관절 한계 구속은 끔 — 가 actuator forcerange 이하). mujoco 는 venv 에만 있으므로 `.venv/bin/python -m rtc_tools.analysis.derive_accel_limits` 로 실행한다
- 테스트 `test/test_derive_accel_limits.py`: 진자 닫힌해, 2-link 최악 부호 패턴에서의 등호 (RNEA), 정제, 퇴화, 가중치, provenance, config 병합, MuJoCo 진자 (venv 에서만)

---

### `clock_phase.py` · `clock_phase_trials.py` — D-3 시계 위상 오차 (dynamic_catching S3.1a)

D-3 는 비율(RTF)이 아니라 **시행별 clock 위상 오차**로 판정한다 (계획 §5). `rtc_mujoco_sim` 의
`clock_lane` CSV 에서 발사 시각을 원점으로 `δ(t) = (steady − steady₀) − (sim − sim₀)` 를 쌓아 시행별
`δ_max = max|δ|` 와 max pause (한 step 의 Δwall − Δsim 최대) 를 낸다. 창은 **비행 구간**이다 — lane 의
`launch_seq`·`ball_active` 로 자르므로 투척 사이 대기 시간이 오차로 잡히지 않는다. **δ 는 판정이 아니라
공변량이다** (D-S8-4 (c), plan §5): 분포 (δ_max·max pause 의 p50/p95/max) 를 내고, `--eps-mm` 을 주면
그 ε 에서 §5 유효 조건을 만족하는 시행 비율을 "valid under eps — COVARIATE" 로 병기한다. 95 % 를 통과시키는
ε 는 여전히 **제안값**으로만 낸다 (측정의 역산이지 예산이 아니다).

```bash
ros2 run rtc_tools run_clock_phase_trials --trials 200         # /sim/launch_ball_at 지정 발사 + 회수 반복
ros2 run rtc_tools analyze_clock_phase clock_lane.csv --plot out.png
```

- 지정 발사라 시드 RNG 를 소비하지 않고 모든 시행이 같은 방출 상태다 — 시행 간 차이는 투척 분산이 아니라 기계의 차이
- lane 의 누적 drop 을 그대로 보고한다. drop 이 있으면 꼬리(큰 δ·긴 pause)가 정확히 빠진 채 분포가 멀쩡해 보인다
- 테스트 `test/test_clock_phase.py`: 합성 lane 에 4 ms 스톨을 주입해 δ_max·max pause 로 복원, 대기 시간 무시, 세그먼트 열 부재 거부

### `hand_close.py` · `hand_close_trials.py` — 손 폐쇄 시간 T_close,e2e (dynamic_catching S4.2)

`L6_hand.md` §4.2 의 ρ(t) = min_{i∈C}((q_i−q_i^pre)·s_i / |q_i^cls−q_i^pre|) 와
T_close,e2e(η) = inf{t−t_cmd : ρ≥η} 를 **포구 컨트롤러의 `<hand>_state.csv`** 에서 낸다. 러너는
`demo_catching_controller` 의 손 그룹에 preshape↔closed 계단을 N 회 쏘고, 분석기는 command lane 으로
시행을 자른다 (`command_*` 가 바뀐 첫 행이 t_cmd).

```bash
ros2 run rtc_tools run_hand_close_trials --group p1b --joint-names "$(...)" --trials 200 --out run.json
ros2 run rtc_tools analyze_hand_close <session>/controllers/demo_catching_controller/p1b_state.csv \
    --profile run.json --plot out.png
```

- **프로파일은 컨트롤러의 읽기 전용 파라미터에서 읽는다** (출하 YAML 이 아니라). 러너가 그것을 JSON 으로
  떨궈 분석기에 넘기므로, 그 run 의 컨트롤러가 실제로 읽은 값으로 분석된다
- **시간축 2개를 모두 보고한다**: `steady` (`t_relative_s`, L6 정의 — lock-step sim 에서는 호스트 스톨 포함)
  와 `tick × dt` (sim 시간, 결정적). 둘의 차이가 곧 스톨이다. CSV 행이 하나라도 드롭되면 tick 축이 어긋나므로
  샘플 간격을 검사해 신뢰할 수 없으면 그렇게 말한다
- **`dt` 도 컨트롤러에서 읽는다** (`control.dt` 미러 = 1/`control_rate`). tick 축과 드롭 판정이 **둘 다** dt 로
  스케일되므로 추정한 dt 는 자기를 검사하지 못한다 — 1 kHz 에서 2 ms 드롭 간격을 3 ms 임계와 비교해 깨끗하다고
  말한다. sidecar 에 `dt` 가 없으면 기본값을 쓰되 tick 축을 **신뢰 불가**로 찍는다
- **시행 구간은 세 갈래로 분류한다** (`pre` / `close` / `other`). 두 자세 중 가까운 쪽을 고르는 투표는 "둘 다
  아니다" 를 말할 수 없어서, 활성화 시점의 hold 자세나 `q_open` 계단이 근접만으로 `close` 가 되면 유령 시행이
  열린다. 출하 p1b 에서 그 투표는 **0.107 % 차이**로 갈린다. 자기 travel 의 25 % 밖이면 `other` 이고, `other` 는
  시행을 끝낼 수는 있어도 시작하지는 못한다
- ρ 는 **최소**다. 한 손가락만 늦어도 손 전체가 못 감싼 것이고 평균은 그것을 지운다. caging 집합은 프로파일이
  정한다 — p1b 출하 자세는 닫힐 때 index DIP 가 오히려 펴지므로 그 관절을 넣으면 진행으로 오독한다
- p99 는 **순서통계량**이다. 성공 시행이 100 미만이면 p99 는 곧 최댓값이고 도구가 그렇게 말한다
- 테스트 `test/test_hand_close.py`: 1차 응답의 해석해 t = −τ·ln(1−η) 복원, ρ 의 min 거동, mask 제외, 역방향
  관절 부호, 드롭 행 탐지, 순서통계량, `other` 자세가 시행을 열지 않음, 추정 dt 의 tick 축 불신

### `vision_lane.py` · `vision_lane_probe.py` · `camera_relay.py` — 예측 lane 실측 (dynamic_catching S3.4)

`ball_perception` `sim_estimator_node` 의 `prediction/trajectory` (PointCloud2, D-4 레이아웃) 를
**필드 이름으로** 디코딩하고 (이름·offset·datatype·count·`point_step`·endianness 가 하나라도 다르면
**거부**), 프로브가 기록한 CSV 에서 S3.4 의 질문 — 발행 주기·N·지평 (TBD-VIS-04), `frame_id` (VIS-06),
측정 손실 뒤 `validity` (VIS-07), best_effort vs reliable 구독 손실 (VIS-08) — 에 답한다. 여기에
**T_det** (발사 → 첫 VALID 예측, plan §7.3 · S3.6) 이 더해진다. **측정만** 한다; 정책은 S5.2 몫이다.

```bash
ros2 run rtc_tools vision_lane_probe <prefix>            # <prefix>_{prediction,camera,truth,diag}.csv
ros2 run rtc_tools vision_lane_probe <prefix> --dump     # + <prefix>_{prediction_dump,innovation,nis}.csv (S8-A)
ros2 run rtc_tools camera_relay --drop-after-s 0.4       # 또는 --drop-prob p / --delay-s d
ros2 run rtc_tools analyze_vision_lane <prefix>
```

- 프로브는 예측 토픽을 best_effort·reliable **둘 다** KEEP_LAST(1) 로 구독해 각각 받은 것을 identity 로 비교한다 — 개수가 같아도 다른 메시지일 수 있다
- 릴레이는 stamp 를 건드리지 않는다 (지연은 전송 지연으로 보이게). estimator 프로파일의 `input.topic` 을 릴레이 출력으로 돌린다
- ⚠️ `sim_estimator_node` 는 `debug.enabled_topics` 에 `prediction/trajectory` **만** 있으면 샘플을 기록하지 않아 토픽만 있고 **발행이 0건**이다 (`needs_samples()` 가 그 토픽을 빼놓는다). 다른 debug 토픽을 하나 이상 같이 켠다
- **T_det (`detection_latencies`)**: 비행은 **truth lane 의 침묵**이 가른다 (`--flight-gap-s`, 기본 0.3 s — 시뮬레이터는 공이 park 상태면 아무것도 발행하지 않는다). 발사 시각은 그 비행의 첫 ground-truth 샘플이고 양자화는 공 발행 주기 하나다. 두 축을 **각각 한 시계 안에서** 낸다 (D-2): `recv` (프로브 steady, 전송 포함 = 소비자 체감) 와 `stamp` (발행자 stamp, 전송 제외). **stamp 축은 sim rig 전제 위에서만 한 시계다** — 시뮬레이터가 truth·camera 를 같은 문장에서 stamp 하고 estimator 가 그 capture stamp 를 예측에 복사하는 경우 (S3.4 실측). 발사 전부터 VALID 이던 트랙 (유령, VIS-07) 은 **generation 이 그때 것**이라 검출로 세지 않는다. 검출이 없던 비행은 버리지 않고 그대로 보고한다 (NaN)
- **`--dump` (S8-A, 기본 off — 위 네 CSV 는 그대로)**: 예측 메시지마다 **모든 지평 점**을 한 행씩 (`point_index`·`generation`·`snapshot_sequence`·점별 `validity`·6×6 공분산 행 우선 `cov_rc`, `vision_lane.dump_prediction_rows`) 과, 추정기 `debug/innovation` (`geometry_msgs/Vector3Stamped`, capture stamp) · `debug/nis` (`std_msgs/Float64`, **stamp 없음** — 프로브의 steady·wall 수신 시각만) 을 각자의 CSV 로. G8-C2 (A⊥B) 의 입력이고, ν̄ 생산자가 아니다 (D-S8-7 (a)). 테스트 `test/test_vision_lane_probe.py` (7 케이스, rclpy 없음)
- 테스트 `test/test_vision_lane.py` (26 케이스): near-miss 레이아웃 거부 (필드 이동·타입·count·누락·초과·point_step·endian), uint64 재조립, 빈 INVALID 스냅샷, 요약 (주기·되감김·identity 비교·유령 트랙), T_det (비행 분리·두 축·유령 배제·미검출 비행·`--flight-gap-s` 가 `--loss-gap-s` 와 별개임)

### `catching_trials.py` — 포구 sim 시행 평가 (dynamic_catching S8-A)

`integrated_bringup` 의 `catching_sim_trials` 한 run 이 남긴 세 가지 — 컨트롤러 세션 CSV, 러너의 trials
dir (`trial_results.json` + `run_meta.json` + 시행별 truth CSV), `sim_lanes:=true` 로 켠 clock·접촉 lane —
를 시행당 한 행으로 잇는다.

```bash
ros2 run rtc_tools catching_trials <session> <trials_dir> --config-dir <install>/share/integrated_bringup/config/ur5e_p1b \
    --urdf <xacro 전개한 urdf> --v-max 3.85 --eps-mm 12 22.8 41.8 49.8 --hold-window-s 0.1
# → <trials_dir>/catching_trials/{catching_trials.csv, catching_trials_summary.json, hand_hold_window.csv}
```

- **서보 지연 τ̂**: 관절별 1차 지연 최소제곱 (`q_cmd − q_meas ≈ τ q̇_meas`, 움직이는 tick) + 시행 클러스터 부트스트랩 CI·R². 속도 상호상관은 1차 지연의 τ 를 주지 않으므로 제공하지 않는다
- **t_c 분해** (첫 COMMITTED tick 에서 `t + plan_t_c_s`): CLIK `‖FK(q_cmd) − ref‖` · 서보 `‖FK(q_meas(t_c)) − FK(q_cmd(t_c − T_lead))‖` · 예측 `‖p_c − p_true(t_c)‖` · `ref_vs_true` · 합계, 공 도착 − t_c, 계획 γ_f (`plan_gamma_f` — `ref_gamma` 는 DECEL 진입 tick 에 1.0 으로 뛴다), 첫 plan 지연, 시행 순환 안의 APPROACH 교체. 공이 t_c 전에 로봇에 맞으면 truth 를 첫 접촉에서 자르고 직전 0.25 s 이차 적합을 t_c 로 외삽한다 (`truth_extrapolated_ms` 로 거리 기록)
- **lead (S8-B)**: `joint_cmd.lag.lead_enable` 이 켜지면 RT 가 참조를 `now + T_arm` 에서 샘플하므로 tick t 의 `q_cmd`·`ref` 는 t + T_arm 을 겨냥한다. 그래서 명령 쪽 (서보·CLIK·`ref_vs_true`) 은 `t_c − T_lead` tick 에서 읽는다. T_lead 는 diag `t_arm_s` 열 → 없으면 러너 미러 (`joint_cmd.lag.*`) → 둘 다 없으면 0 이고, 열과 미러가 다르면 다른 세션의 trials dir 로 보고 거부한다. 같은 tick 의 `‖FK(q_meas) − FK(q_cmd)‖` 는 `cmd_meas_gap_mm` 로 기록만 한다 — lead on 에서 의도된 선행을 잔여에 더해 서보가 나빠진 것처럼 읽히기 때문이다. lead off 에서는 두 값이 같다
- **truth 성공** (G8-D, plan §1a): HOLD 끝 (첫 RETREAT tick) 부터 대기 자세 release (손 위상 RELEASE) 까지 모든 truth 샘플이 catch frame 에서 `--hold-radius-m` (기본 프로파일의 공 지름) 안. 슈퍼바이저 판정 대비 혼동행렬, Wilson 구간
- **D-3 공변량** (`clock_phase` 재사용): δ_max·max pause·δ(t_commit)·δ(t_c), `--eps-mm` 별 유효 수. lane 의 발사는 순서가 아니라 **하나의 시계 오프셋**으로 시행과 짝짓는다 — 한 세션에 러너를 두 번 돌리거나 GUI 로 던진 발사가 섞여도 그 발사는 무시되고, 짝이 없는 시행은 공변량 없이 `unpaired_trials` 로 남는다 (절반 미만이 짝지어지면 다른 run 으로 보고 거부). `--v-max` 는 기본값이 없다 (목표 분포의 최대 포구 속력 — 로봇 상수라 CLI 로 받는다); 없으면 `NOT_EVALUATED(v_max not given)`
- **첫 손–공 접촉 episode** (접촉 lane): 충격량·최대 접촉력·지속·접촉 속력. 손 토크는 sim forcerange 클램프라 판정하지 않는다
- **손-관절 캡처 witness** (#537 S8-C, D-S8-8 (b)): diag 의 `hand_stalled_n`/`hand_effort_frac`/`hand_blocked_s`(연속 불통 지속 시간 [s], 0 = 안 걸림)/`outcome_source` (컬럼이 없는 구 세션은 NaN) 에서 시행당 `outcome_source`(0 none·1 fingertip·2 hand·3 both) — HOLD 끝 tick 에 판정되지만 그 tick 은 이미 mode RETREAT 로 기록되므로 **첫 RETREAT 행**에서 읽고, `hand_blocked_s_judge`/`hand_stalled_n_judge`/`hand_effort_frac_judge` — 판정이 참조한 손 스테이지 값이므로 그 한 tick 전인 **마지막 HOLD 행**에서 읽는다 (서로 다른 행). RETREAT 가 HOLD 가 아니라 ABORT_SAFE 등에서 들어왔으면 (판정도 witness 도 없었던 시행) 모두 NaN — RETREAT 진입 직전 행이 HOLD 일 때만 유효. `tips_only_verdict` 는 손 witness 없이 손끝만 있었다면 어땠을지 재구성 (`CAPTURED`+`outcome_source==2` → `MISSED`, 그 외 그대로; witness 없으면 빈 문자열). `t_release_to_pre_ms` 는 release tick (`truth_success` 의 `t_release`) 부터 손이 q_pre 로 돌아온 첫 PRESHAPE tick 까지. **`hand_blocked_s_judge` 는 판정이 실제로 비교한 값보다 정확히 한 tick 짧다** — 컨트롤러의 `JudgeOutcome` 은 `hand_blocked_s_judge` 가 읽힌 tick 의 **다음** tick(첫 RETREAT tick)에서 `t_persist` 와 비교하므로, 오프라인 재현은 `hand_blocked_s_judge + dt >= t_persist` 를 써야 한다 (`dt` 는 모듈이 이미 구하는 tick 주기, `recorded_dt`). `hand_persist_met(hand_blocked_s_judge, dt, t_persist_s)` 가 그 재현이고, 시행별 `hand_persist_met` 컬럼 (1.0/0.0/NaN) 으로 나간다 — `t_persist_s` 는 `hand_capture_t_persist_s` 가 `catching.robot.hand.capture.t_persist` 에서 읽으며 (TBD·없음이면 NaN), 프로파일이 이 키를 갖고 있을 필요는 없다
- ρ_min/ρ_max/qd_tol/effort_frac_min 캘리브레이션: `hand_hold_window_extremes` 가 `catching.robot.hand.{q_pre,q_close,caging_mask}` (부호 규약은 `hand_close.joint_progress` 재사용) + 손 device 의 `joint_limits.max_torque` 로 매 시행의 `[t_hold_end − --hold-window-s, t_hold_end)` 구간에서 caging 관절별 극값 (`rho_lo`/`rho_hi`/`qd_absmax`/`frac_lo` = s·τ/τ_max 최솟값·`n`) 을 내고, HOLD 끝에서 RETREAT 로 들어간 시행에 한해 (ABORT_SAFE 경유 제외) `<trials_dir>/catching_trials/hand_hold_window.csv` (long-format, `idx,joint,supervisor,rho_lo,rho_hi,qd_absmax,frac_lo,n` — `supervisor` 는 그 시행의 판정 문자열) 로 쓴다. `--hold-window-s` 기본값 0.1 은 `Settings.hold_window_s` (`DEFAULT_HOLD_WINDOW_S`) 하나가 SSoT. `capture_would_fire(rows, rho_min, rho_max, qd_tol, effort_frac_min, min_joints)` 는 그 극값이 임계를 (경계 포함) 만족하는 관절이 `min_joints` 개 이상인지 — `EvaluateHandCapture` (hand_capture.hpp) 의 오프라인 미러. **캘리브레이션은 선택 사항이라 아무 것도 세션 전체를 막지 않는다** (`None` + stderr 한 줄, `analyse_session` 은 계속됨): `q_pre`/`q_close` 가 아직 TBD/없는 프로파일, `q_pre`/`q_close` 원소 하나가 아직 숫자가 아닌 (개별 TBD) 경우, `caging_mask` 길이가 안 맞는 경우, 손 device 의 `joint_state_names` 길이가 `q_pre` 와 안 맞는 경우, `joint_limits.max_torque` 가 없거나 짧은 경우, 손 device CSV 에 `actual_vel_*`/`effort_*` 열이 없는 (구 세션) 경우. `q_pre`/`q_close` 끼리 길이가 다르거나 비어 있으면 (같은 YAML 블록의 내부 모순) 은 여전히 `SystemExit`
- `ref_saturated` 시행별 max streak (G8-C3)
- **G3-D (i) plan validity 비율** (#537 S8-D, `--clock-lane` 필요): `planner_events.csv` 의 각 행 (`wake_ns`, `plan_valid`) 을 시행의 `[발사, 첫 COMMITTED tick]` (커밋 못 하면 `[발사, 시행 끝]`) 창에 넣어 `planner_cycles`(창 안 행 수)·`plan_valid_cycles`·`plan_valid_ratio`(0 행이면 NaN)·`plan_valid_at_commit`(커밋 시점 또는 그 이전 마지막 행의 `plan_valid`, 커밋 안 하면 NaN) 을 낸다. **시간축**: `wake_ns` 는 steady clock (`rtc::SteadyNowNs`, `std::chrono::steady_clock`) 인데, `use_sim_time_sync: true` 인 sim 프로파일의 `t_relative_s` 는 iteration × dt 라 steady/wall 시계와 고정 오프셋이 없다 (`rt_controller_node_rt_loop.cpp:457-459`) — 그래서 clock lane 이 시행마다 짝지은 발사에서 얻은 **그 시행의 오프셋** `ClockLane.trial_offsets[idx]` (발사 시점의 `steady_s − t_relative_s`) 로 `t_relative = wake_ns·1e-9 − trial_offsets[idx]` 로 옮긴다 (`_planner_cycle_times`). 세션 중앙값 `steady_offset` 은 쓰지 않는다 — RTF < 1 이면 오프셋이 세션에 걸쳐 드리프트해 뒤쪽 시행의 창이 밀린다. lane 이 짝짓지 못한 시행은 G3-D 필드가 없다 (D-3 clock covariate 와 같은 규칙). lane 이 없으면 (`--clock-lane` 미지정) G3-D 필드는 전부 비어 있고 요약에 `g3d` 키가 없다. 요약 `g3d`: `n_trials_with_cycles`(≥1 cycle 시행 수)·`plan_valid_ratio_p50_p05_p95`·`approach_plan_switches_distribution`(값별 개수, 기존 `approach_plan_switches` 옆)
- **`--gate-map DIR` (선택, S8-D)**: `catch_gate_map` 출력 dir — `gate_map_summary.yaml` (`map_dir`: 원본 `catchability_map` dir — `catch_gate_map` 이 절대경로로 적는다, `seed_id`) · `gate_map.csv` (`reason_torque`) · `<map_dir>/throw_summary.csv` (grid 축 값) 를 읽어 `--dist` box 시행 (`distance_m`/`azimuth_deg`/`release_height_m`/`aim_deviation_deg`/`speed_m_s`/`elevation_deg` 을 갖는 것 — reference/varied 시행은 축이 없어 검정 `None`) 마다 가장 가까운 grid 투척을 찾는다 (각 축을 그 축 grid 간격의 중앙값으로 정규화한 유클리드 거리 — 단일 값 축은 정규화에서 제외; 동률은 `throw_index` 가 작은 쪽). 시행별 `map_open`(`seed_id` 에서 `reason_torque=="none"` 후보가 하나라도 있는 grid 투척인지)·`map_throw_index`·`map_distance`. 요약 `gate_map`: `map_dir`·`seed_id`·`verdicted`/`open`/`open_fraction`, 그리고 (hold radius 가 있을 때) `truth_whole`/`truth_open` — 검정받은 (`map_open` 이 `None` 이 아닌) 시행 전체와 그중 `map_open` 부분집합 각각의 truth 성공 수/n/Wilson 95 % (hold radius 없으면 `"NOT_EVALUATED(no hold radius)"`)
- 라이브러리 함수: `wilson_interval`·S0.9 검정력/필요 n, A⊥B 백색화 교차공분산 (시행 클러스터 부트스트랩, G8-C2), NEES 요약 (raw·centered·coverage, 양측, G8-B) — 둘 다 합성 데이터 positive control 로 테스트. CLI 는 아직 부르지 않는다 (probe 덤프가 있는 세션부터)
- **로봇 상수 없음** (ARCH-1): catch frame 은 `_base.yaml` (없으면 sim 전용 프로파일의 `sim.yaml`) `urdf.extra_frames`, sim world ↔ model world 는 `catching.io.arm_base_frame`·`base_T_world` 로 `frame_placement_in_model_world` (컨트롤러와 같은 합성), 관절은 diag 의 `q_cmd_*` 열, device·로그 이름은 컨트롤러 `topics`/`logs`, dt 는 러너가 기록한 미러 `control.dt`
- 테스트 `test/test_catching_trials.py` (81 케이스) — **골든**: 파일럿 세션 `260924_1218` 에서 자른 fixture (`test/data/catching_pilot_260924_1218/`, 2.6 MB, 재생성 스크립트 `make_fixture.py`) 로 τ̂ 6 관절 200 ± 5 ms · 서보 중앙값 122 ± 5 mm · CLIK 2 ± 1 mm · 25/25 Missed · ε 12 mm 유효 7/25 (v_max 3.85 m/s) 재현 (lead off 라 서보 = `cmd_meas_gap_mm`). lead 는 성분을 아는 합성 시행 (선행 0.2 s, ref 10·CLIK 2·서보 3 mm) 에서 복원하고, 기록된 lead 를 무시하면 거짓 FAIL 로 돌아가는 것을 positive control 로 둔다. 손-관절 witness: HOLD/RETREAT 두 행이 섞인 합성 tick 으로 판정 컬럼이 마지막 HOLD 행을, `outcome_source` 는 첫 RETREAT 행을 읽는지 (두 값을 다르게 둬 어느 쪽을 잘못 읽어도 실패하게 함), ABORT_SAFE → RETREAT (HOLD 를 거치지 않은 시행) 는 전부 NaN, `tips_only_verdict` 4 소스 조합, `t_release_to_pre_ms` (복귀 없음 NaN 포함), `hand_persist_met` 의 +dt 보정 (보정 없이는 경계에서 틀리는 값으로 pin) 과 `hand_capture_t_persist_s` 의 TBD/부재 NaN, 음의 방향 관절·non-caging 관절 제외를 포함한 window 극값, `capture_would_fire` 경계 (포함) 와 `min_joints`, 캘리브레이션의 다섯 완화 경로 (손 device 에 `joint_limits` 전무, `q_pre`/`q_close` 원소 개별 TBD, 손 device CSV 에 `actual_vel_*`/`effort_*` 열 없음 — 각각 `analyse_session` 이 행을 계속 내는지까지), `--hold-window-s` 기본값이 `Settings` 와 한 값인지. 마지막 HOLD 행 대신 첫 RETREAT 행을 읽게 하거나 ρ 의 부호 `s_i` 를 빼는 변형이 실제로 테스트를 red 로 돌리는 것을 수동으로 확인함 (영구 테스트로 넣지 않음). G3-D: `plan_validity_window` 순수 함수 (창 경계·커밋 없음·0 cycle NaN) + 골든 fixture 로 비율 범위·양수 cycle 수·첫 시행 pin, `--clock-lane` 없이는 `g3d` 키 자체가 없는지, 세션 중앙값 오프셋을 10 s 틀어도 시행별 창이 그대로인지 (짝 없는 시행은 G3-D 없음). 게이트맵: `load_gate_map`/`grid_axis_steps`/`nearest_grid_throw`/`gate_map_verdict` 순수 함수를 tmp_path 합성 CSV/YAML 로 (축 자기 step 정규화 positive control — 같은 오프셋이 어느 축에 있느냐로 판정이 갈림, 동률 시 최소 `throw_index`, azimuth 만 다른 격자에서 azimuth 로 매칭), `analyse_session`/CLI 배선 (fixture 는 reference 시행뿐이라 검정이 전부 `None` 인지)

### `catchability_map.py` — catchability 지도 (dynamic_catching S3.5a)

투척 grid → 항력 비행 → 포구 후보 → **C++ judge** → 집계·제안·플롯. 판정은 재구현하지 않고
`rtc_controllers` 의 오프라인 실행파일 `catch_pose_ik_batch` (= 런타임 계획기와 같은
`CatchPoseIk::Solve`, 같은 YAML 키 — plan §11 / S1.9) 를 CSV 로 호출한다. import 전용 순수
함수와 CLI 를 함께 갖는다.

```bash
ros2 run rtc_tools catchability_map \
  --robot-config <config>/ur5e_p1b/_base.yaml \
  --ball-config  <config>/ur5e_p1b/mujoco_simulator.yaml \
  --arm-base-frame base \
  --drag-coefficient 0.55 --drag-coefficient-source 'rtc_mujoco_sim/src/projectile_ball.cpp:30' \
  --air-density 1.204 --air-density-source 'rtc_mujoco_sim/include/rtc_mujoco_sim/projectile_ball.hpp:95' \
  --distances-m 4.0 --azimuths-deg '180 190 200' --release-heights-m 1.8 \
  --aim-deviations-deg 0.0 --speeds-m-s '5.4 6.0' --elevations-deg '40 48' \
  --window-s 1.0 1.8 --stride-s 0.05 --min-flight-time-s 1.0 \
  --max-reach-m 1.1 --seed '0.0 -1.4 0.9 -1.9 -1.5708 0.0' \
  --workers 6 --epsilon-m 0.01 --out-dir <out>
```

출력: `candidates.csv` (후보별 verdict·q\*, **모든 seed**), `throw_summary.csv` (**격자 throw 당 1행** —
후보가 0 개인 throw 도 `candidates=0`·`accepted=0` 으로 나온다; `wait_pose_seed_id` 열이 어느 대기 자세의
표인지 말한다), `reason_histogram.csv`, `throw_region.yaml` (`sim.throw_region` 제안 + provenance),
`provenance.yaml`, `azimuth_coverage.png` (방위별 포구 가능 구간), `manipulability.png`,
그리고 `model_config.yaml` / `model.urdf` / `seeds.csv` / `shards/` (재개용; shard 마다
`*_fingerprint.json` sidecar).

**headline 은 seed 하나의 수치다.** `--seed` 를 여러 개 줘도 로봇은 **한 자세**에서 기다리므로,
`accepted_throws`·`throw_region.yaml`·`throw_summary.csv`·사유 히스토그램·w/θ 분포·ε 경계·플롯·stderr
요약은 전부 seed 순위 1위 (`rank_wait_pose_seeds` best) **하나**를 기술하고, 그 seed 의 id 와 관절 벡터를
`results.headline_seed` · `throw_region.wait_pose_seed_id`/`wait_pose_q` 에 적는다. "어느 seed 든 하나라도
받으면 accepted" 인 합집합은 `accepted_throws_any_seed` 라는 **이름으로만** 남는다 — coverage 가 아니다
(어떤 단일 자세도 그 수를 내지 못한다). 비율의 분모는 어디서나 **전체 격자** 이고
(`accepted_fraction_denominator`, `seed_ranking[].denominator`), 후보가 judge 까지 간 throw 수는
`throws_with_candidates` 로 따로 적는다.

```python
from rtc_tools.analysis import catchability_map as cm

shape  = cm.ball_shape_from_config([Path(".../mujoco_simulator.yaml")])   # radius_m, mass_kg
params = cm.ball_params_from_shape(shape, drag_coefficient=..., drag_coefficient_source="<file:line>",
                                   air_density_kg_m3=..., air_density_source="<file:line>")
traj   = cm.integrate_flight(throw.position_m, throw.velocity_m_s, params, horizon_s=1.5, step_s=0.002)
throws = cm.generate_throw_grid(distances_m=(4.0,), speeds_m_s=(5.0, 7.0, 9.0))
p_b, v_b = cm.world_to_base(base_T_world, traj.position_m[i], traj.velocity_m_s[i])

art  = cm.write_model_config([robot_cfg], out_dir)        # 출하 스키마 → LoadModelConfig 스키마
mwTb = cm.frame_placement_in_model_world(art.urdf_text, "base")   # model world ← arm base
cand = cm.sample_catch_candidates(traj, window_s=(1.0, 1.8), stride_s=0.05,
                                  min_flight_time_s=1.0,
                                  reach_filter=cm.max_distance_filter(base_xyz, 1.1))
rows = cm.run_judge_batch(inv, cm.to_judge_candidates(cand, model_world_t_world=mwTb @ bTw),
                          work_dir=out_dir / "shards", shard_size=200, workers=6)
judged = cm.join_results(candidates, rows)
best = cm.rank_wait_pose_seeds(judged, throw_count=len(throws)).best        # 분모 = 전체 격자
outc = cm.summarize_throws(judged, seed_id=best.seed_id, throw_count=len(throws))  # seed 하나
```

- ⚠️ **judge 의 프레임은 arm base 가 아니라 Pinocchio MODEL WORLD (= URDF 모델 root) 다.**
  `ur5e_p1b` 에서 모델 root 는 `base_link` 이고 CLIK `base_frame` 은 `base` 로 z 축 180° 차이다
  (plan §11). world→`base` 는 항등이므로 **world 좌표를 judge 에 그대로 넣으면 x·y 가 뒤집혀**
  "공이 등 뒤에서 온다". 그래서 CLI 는 `base_T_world` 를 인자로 받고 (`--world-yaw-deg` /
  `--world-translation-m`, 기본 항등 — 쓰인 값은 provenance 에 기록), `model_world_T_base` 는
  `--arm-base-frame` 으로 지정된 프레임의 FK 에서 읽어 **합성**한다. 어느 쪽도 박제하지 않는다.
  `iiwa7_leap` 은 모델 root = `link_0` = base 라 합성이 항등이다
- ⚠️ **출하 `urdf.sub_models.<arm>` 은 flange (`tool0`/`ee_link`) 에서 끝나 catch frame 이 그 sub-model
  에 없다.** `write_model_config` 가 arm root → `urdf.extra_frames.<catch_frame>.parent` 까지의
  sub-model `arm_catch` 를 **추가로** 선언한다. 그 뒤(손 전체)는 `buildReducedModel` 이 잠그는데,
  손바닥이 손의 폐쇄 루프 상류라 catch frame 의 FK·팔 관절 Jacobian 은 **정확하다**. 스키마 차이도
  여기서 번역한다 (`urdf_path`, `sub_models` 는 map 이 아니라 **sequence**; `extra_frames` 만 같은 map)
  - **S6-B (2026-09-23)**: 로봇 config 가 `urdf.sub_models.<arm>_catch` 를 이미 선언하면 (출하 두 로봇 —
    런타임 계획기의 `planner.sub_model`) 그 항목을 **이름 그대로** 쓴다. 그 root/tip 이 arm root →
    catch frame 부모가 아니면 거부한다 — 지도와 런타임 계획기가 다른 모델에서 판정하게 되기 때문이다 (G3-I)
- **샤딩·재개**: 후보를 chunk 로 나눠 최대 6 프로세스 (이 머신 상한) 로 돌리고 자식 env 에
  `OMP_NUM_THREADS=1` 을 준다. shard 출력은 **행/ID 가 맞고 + 입력 fingerprint sidecar 가 일치할 때만**
  재사용한다 (`shard_is_reusable`). 후보 id 는 실행마다 `0..N-1` 이라 ID 대조만으로는 같은 `--out-dir` 에
  `--params`·`--seed`·robot config·격자 **값**을 바꿔 재실행해도 옛 verdict 가 그대로 재생된다. fingerprint
  는 그 shard 의 후보 CSV · model config · 그것이 가리키는 URDF/closure YAML · seeds · params (없으면
  `none`) 의 sha256, sub-model·catch frame 이름, judge 실행파일의 **정체** (경로+크기+mtime — 내용 해시가
  아니다; 둘을 보존하는 재빌드나 judge 가 링크한 공유 라이브러리 변경은 못 잡으므로 그때는 `shards*/` 를
  지운다) 로 만든다. sidecar 가 없거나 (fingerprint 이전의 출력 디렉토리) 다르면 재판정·덮어쓰기이고, 무엇이
  바뀌었는지 stderr 에 적는다. `shards_eps/` 도 같다. provenance 는 params 의 경로와 **sha256** 을 함께
  기록한다. 비정상 종료·짧은 CSV 는 그 shard 의 stderr 를 달아 raise 한다 — 조용히 짧은 shard 는
  "그 투척들은 못 잡는다" 로 읽혀 나중에 반증되지 않는다
- **`q*` 열이 비는 것은 세 번째 상태다** (pose 없음 ≠ q = 0). judge 는 `none`/`below_manip_min`/
  `rank_deficient` 에만 pose 를 쓰므로 나머지 사유는 `q*` 가 공란이고, 파서는 이를 `None` 으로
  구분하고 절반만 찬 행은 거부한다
- **w₅·w₆ 는 따로 보고한다** (같은 q\* 의 두 측정이지만 차원이 달라 pooling 하지 않는다, plan §11 C-3).
  θ 분포는 `fraction_below` 와 `fraction_near_limit` 을 **둘 다** 낸다 — "α_max 의 몇 % 안"이
  양쪽으로 읽히고 결론이 뒤집히기 때문. 접근축 cone 이 binding 하는지는 `fraction_near_limit` 이 답한다
- **θ 보고의 `alpha_max` 는 judge 가 실제로 적용한 값이다** (`resolve_alpha_max`). `--params` 가
  `planner.ik.alpha_max` 를 주면 그 값을 쓴다. 파일 형태는 judge 의 loader 와 같은 세 가지만 받는다 —
  root 의 `catching:` map / root 에 `planner:` 가 있는 catching tree 자체 / 출하 controller config 형태
  `<controller_name>: {catching: ...}` (유일한 top-level 항목일 때) — 그 밖은 기본값으로 떨어지지 않고
  **에러**다. `"TBD"`·키 부재는 "미지정" 이다. `--alpha-max-rad` 는 **기본값이 없는** 선택 인자로, params 가
  값을 주는데 다른 값을 넘기면 에러 (진실의 출처가 둘), 같으면 허용, params 가 미지정일 때만 단독으로 쓰인다
  (그때 judge 는 in-code 기본값으로 돌았으므로 이 인자는 그 기본값에 대한 호출자의 진술이다). 둘 다 없으면
  judge 의 in-code 기본값 0.26 의 **미러**를 쓰고 provenance 에 미러임과 출처를 적는다. judge 가
  `--print-options` 를 제공하면 그 출력의 `planner.ik.alpha_max` 와 대조해 **다르면 sweep 전에 에러**이고,
  제공하지 않으면 provenance 에 "not cross-checked" 로 남는다
- **seed 비교**는 포구 가능 throw 비율 내림차순, 동률이면 평균 `log w5` 로 가른다. 서로 다른 throw
  집합을 비교하려 하면 거부한다. runner-up 의 coverage 차이가 대기 자세 민감도 수치다. 비율의 분모는
  `throw_count` (전체 격자) 이며 headline 과 같다 — 주지 않으면 judge 까지 간 throw 수로 떨어지고 결과의
  `denominator` 가 그렇게 적는다. `summarize_throws` 는 여러 seed 의 행을 `seed_id` 없이 받으면 **거부**한다
  (합집합을 실수로 만들 수 없게); `propose_throw_region` 도 같다 (`flight_time_s` 가 합집합이 된다)
- **ε 경계 개수**의 "경계" 는 후보 생성 단계로 정의된다: 받아들여진 후보의 p_c 를 축별 ±ε 로 옮긴
  6 사본을 **다시 judge 에 넣어** 하나라도 거부되면 그 후보는 경계 위다 (w₅ 여유 같은 대용값 추정 아님)
- **힘 법칙은 sim 과 같다**: 중력 + 이차 항력 `-½ρC_dA|v|v` (`ComputeProjectileBallAeroForce`).
  **각속도 0 가정**이므로 Magnus 항은 소멸하며 구현하지 않는다 — spin 을 실어 발사한 궤적은 이 모델과 다르다
- **공기밀도·항력계수·반지름·질량은 기본값이 없다.** `C_d`·ρ 는 C++ `constexpr` preset 에만 있고 YAML 로
  노출되지 않으므로, python 에 박아두면 preset 이 바뀌어도 아무것도 실패하지 않는다. 호출자가 값과
  **출처 라벨 (`file:line`)** 을 함께 넘겨야 하고 provenance 가 그 라벨을 기록한다. 반지름·질량은 노출되어
  있으므로 `projectile_ball.radius_m` / `.mass_kg` 에서 읽는다
- 스칼라 `k = ρC_dA/(2m)` [1/m] 는 **참고용 파생값**으로만 기록한다 — 문서의 대표값과 수치가 다르고
  (출하 tennis preset → 0.02048, L0 §4.1 대표값 0.0229) L0 §7 이 둘을 환산할 수 없다고 명시한다
- **프레임 변환은 인자로만 받는다.** URDF 가 `base` / `base_link` 를 같은 원점에 z 180° 로 두는 경우
  잘못 고르면 downrange 부호만 뒤집혀 "공이 등 뒤에서 온다" — 그런데 수치는 전부 그럴듯하다.
  로봇별 값을 모듈에 박지 않으며, 호출자가 (같은 q 에서 MuJoCo FK ↔ Pinocchio FK 로 확정한) 변환을 넘긴다
- 각 throw 는 world 기준 release position·velocity 를 들고 있어 `rtc_msgs/srv/LaunchBall` 요청 필드로
  1:1 대응된다 (`throw_to_launch_request`)
- 테스트 `test/test_catchability_map.py` (66 케이스): 무항력 닫힌해 + **적분 차수** (스텝 절반 → 오차
  1/16; Euler 2, substage 속도를 고정한 RK4 2.1 로 실측 반증), 항력 부호·상승 중 속력 단조감소·종단속도
  √(g/k), 프레임 변환 longhand oracle·transpose·Rz(180°) 함정 pin·round trip, grid 개수·속력/고도각
  역산·downrange 부호, provenance 출처 라벨·파생 k, 손으로 계산한 후보 샘플링 (비행시간 하한이 제거 +
  재-anchor 하는 것까지)·reach prefilter, **`q*` 공란 행과 정상 행이 섞인 결과 CSV**·절반만 찬 pose 거부,
  짧은 shard 탐지, 출하 스키마 → ModelConfig 번역 (`arm_catch` tip = catch frame 부모), 모델 world 변환
  rigid 검사, seed 순위 tie-break (두 기준이 어긋나게 구성), `throw_region` 박스 (초과 포함 비율까지 손
  계산), 사유 히스토그램, ε 경계 개수. **대역 judge** (같은 CLI·CSV, verdict 가 입력의 순수 함수라 stale 재사용이
  *틀린 verdict* 로 드러난다) 로 shard 재개 — 무변경 시 재사용 (같은 bytes 재기록 포함) · params 내용 / seed
  값 / 같은 id 의 좌표 / URDF / judge 정체 변경·sidecar 부재 시 재판정 —, **서로소인 두 seed 가 headline 에
  합산되지 않음**, 후보 0 개 throw 의 요약 행, 두 비율의 분모 일치, `alpha_max` 해석 6 경우 (params / 인자 /
  일치 / 불일치 에러 / 둘 다 없음 / controller-config 형태) + judge 보고와의 대조, CLI 전 구간. **실제 `catch_pose_ik_batch` 왕복** (출하 ur5e_p1b config →
  `--dump-frame` nv 6 · `base` 가 Rz(180°) 임을 pin → 2-shard 실행 → poseless 행 · 재개) 은 바이너리·출하
  config·URDF 툴체인이 없으면 이유를 적고 skip 한다

### `catch_speed_budget.py` — 포구 속력 예산 (dynamic_catching S4.4)

`catchability_map` 이 "팔이 그 자세에 **갈 수 있는가**" 를 답한다면 이 도구는 그 수락 후보에 대해
"거기서 공과 **함께 움직일 수 있는가**" 를 답한다. 계획기 γ 창 (L3 §4.5) 은
`‖v‖ + margin ≤ v_arm + v_rel` 일 때 열린다 — `v_arm` 은 포구 자세 q\* 에서 공 진행 방향 v̂ 로 낼 수
있는 팔 속력, `v_rel` 은 손이 흡수하는 상대속도 (공식 `d_eff / T_close,tot` 또는 실측값) 다.

후보별로 (1) `v_dir_max_lp` — 접근축을 유지한 채 관절 속도 box 안에서 v̂ 로 낼 수 있는 최대 속력
(선형계획, 물리 상한), (2) `v_dir_max_dls` — L3 §4.5 의 최소노름 추정 (런타임 `DirectionalSpeedMax` 가
받는 값의 거울; LP 이하이며 그 차이가 최소노름 해가 버리는 여유 자유도다), (3) **토크 한계 방향 가속**
`max a  s.t.  J₅q̈ + J̇₅q̇ = [a v̂; 0 0],  |M q̈ + h| ≤ η_τ τ_max` 를 속력 ramp 를 따라 풀고
stroke·시간을 적분한 `v_arm` 을 낸다. 전부 **낙관적 상한**이므로 (bang-bang, 자세 고정, 도달 구)
창이 비면 결론은 확정적이고 열리면 provisional 이다.

```bash
ros2 run rtc_tools catch_speed_budget \
  --robot-config <config>/<robot>/_base.yaml --group <arm_group> \
  --map-dir <catchability_map out-dir> --out-dir <out> \
  --velocity-source model --eta-v 0.9 --eta-tau 0.8 \
  --rotor-inertia '0.1 0.1 0.1 0.1 0.1 0.1' --rotor-inertia-source '<mjcf>:<line> armature' \
  --arm-base-frame <base_frame> --max-reach-m 1.1 --floor-world-z-m 0.1 \
  --detection-s 0.10 --latency-s 0.14 --close-total-s 0.2815 --arm-delay-s 0.05 --time-margin-s 0.02 \
  --relative-speed-m-s '0.34 1.0'
```

- 출력: `speed_budget.csv` (후보별), `drop_table.csv` (행 = **낙차** `z_catch − z_release` 구간, 열 =
  `v_rel`, 셀 = 창이 열리는 후보 비율), `cell_table.csv` (행 = (릴리스 높이, 거리) 셀, 값 = **투척 수**;
  분모는 항상 `throw_summary.csv` 의 전체 격자), `speed_budget_summary.yaml` (provenance)
- **기본값이 없는 인자는 전부 결정이다.** `--velocity-source` (`config` = 실행이 강제하는
  `joint_limits.max_velocity`, `model` = URDF `<limit velocity>` 정격), `--rotor-inertia` (URDF 에는
  회전자 반사관성이 없다 — 빼면 모든 가속이 과대평가되므로 0 을 주려면 **명시**해야 한다), 선행시간
  다섯 항. 최소 비행시간 `T_det + L + T_close,tot + T_arm + T_margin` (plan S0.7 R1) 에는 **손 폐쇄
  시간이 들어간다**; ramp 에 쓸 수 있는 시간은 `t_c − (T_det + L)` 다
- **프레임.** judge 의 `p_model`·`v_model` 은 모델 world (URDF root) 이고 `LOCAL_WORLD_ALIGNED`
  Jacobian 이 쓰는 프레임과 같아 그대로 쓴다. 바닥 높이만 world 열을 쓴다
- **자체 검증 (fail-closed).** 모든 수락 q\* 에서 catch frame FK 가 `p_model` 과
  `--fk-tolerance-m` (기본 2.5 mm = judge `eps_pos` + 여유) 안에 들어와야 한다. 벗어나면 관절 순서·
  frame·모델 중 하나가 judge 와 다른 것이므로 **보고하지 않고 종료**한다. 최소노름 속력이 LP 를 넘어도 종료
- 테스트 `test/test_catch_speed_budget.py` (23 케이스): 2-관절 LP 닫힌해, LP ≥ 최소노름 (6·7 축),
  투영 분자, 한계 무효 플래그, 속력 해의 유한차분 FK 대조 (접근축 고정 포함), **프레임 비대칭**
  (base 가 모델 root 에서 반 바퀴 돈 fixture), **관절 이름 순서 ≠ 모델 순서**, 가속 LP 의 RNEA·2차 FK
  재대입 (한계가 어딘가에서 tight), 회전자 관성 효과, 중력만으로 한계 초과 시 NaN, stroke·ramp 닫힌해,
  전체 격자 분모, CLI end-to-end 와 FK 불일치 거부. pinocchio·scipy 가 없으면 skip

### `catch_gate_map.py` — gate-catchable 지도 (dynamic_catching S3.5b)

`catchability_map` 의 수락 후보 (포구 자세가 있는 후보) 에 계획기의 나머지 게이트를 건다 — 도달시간
(L3 §4.3), γ 창 (§4.5), 정지점 (§4.9), 그리고 그 앞의 commit 선행 (§4.11: $t_c-t_{plan}\ge T_{close,tot}+T_{arm}+T_{margin}$ — 런타임
함수가 없어 python 이 건다). **나머지 판정은 python 이 하지 않는다**: `rtc_controllers` 의
`catch_gate_batch` 가 `time_feasibility.hpp` 의 런타임 함수를 그대로 부르고 (G3-I), python 은 그 입력과
출력만 맡는다.

- **python 이 만드는 입력**: q̇ᵘ (L3 §4.5 의 DLS 단위 속도 — 이 도구는 python 으로 직접 계산한다. 런타임 생산자는 S6.2 의
  `rtc_controllers` `UnitSpeedSolver` 이고, 두 식의 일치는 `PlannerUnitSpeed` 테스트가 고정한다 — G3-I) 와 그것이
  내는 속도 `J_p q̇ᵘ`. `reference.v_max` 는 `--v-max-m-s derived` 면 수락 후보의 LP v_dir,max 최대 / η_v
  (S4.4 결정: TCP 항은 관절 정격 안에서 구속하지 않는다)
- **python 이 거는 경계**: p_stop 이 도달 구·바닥 안인가 (`planner.workspace.catch_box` 가 TBD 라 지도와
  같은 경계를 쓴다)
- **도달시간은 두 층**을 항상 같이 낸다. `box` = 출하 가속 box (`--accel-limits`, plan §9) 로 C++ 가
  판정. `torque` = **그 이동**이 토크 한계 안에 드는가 — 전 관절이 하나의 bang-bang/사다리꼴 경로
  프로파일을 따라 대기 자세 → q\* 로 가고, 경로 전체에서 |M q̈ + h| ≤ η_τ τ_max (회전자 관성 포함) 인
  최대 경로 가속을 이분 탐색한다. 충분조건이고 런타임 대응물이 아직 없어 **provisional** 이다 (D-16 개정)
- **대기 자세는 하나다.** 지도가 여러 seed 로 판정됐으면 `--seed-id` 가 필수다 (합집합은 과대평가).
  `proposed_wait_pose` 는 γ·정지 게이트를 통과한 q\* 의 관절별 midrange — 새 seed 로 `catchability_map`
  부터 다시 돌리는 고정점 반복의 입력이다 (q\* 가 seed 에 의존한다)
- rollout (L3 §4.8) 은 판정하지 않는다 (런타임 rollout 은 S6.3 의 `gamma_rollout.hpp` 에 있으나 이
  도구에는 배선하지 않았다) — 열린 셀은 `PASS(provisional)`,
  빈 지도는 확정적이다

```bash
ros2 run rtc_tools catch_gate_map \
  --robot-config <config>/<robot>/_base.yaml --group <arm_group> \
  --map-dir <catchability_map out-dir> --out-dir <out> --velocity-source model \
  --accel-limits <config>/<robot>/derived_accel_limits.yaml --eta-v 0.9 --eta-tau 0.8 \
  --rotor-inertia '0.1 0.1 0.1 0.1 0.1 0.1' --rotor-inertia-source '<mjcf>:<line> armature' \
  --v-max-m-s derived --d-eff-m 0.095 --d-eff-source 'planner.hand.d_eff' \
  --close-total-s 0.2815 --gamma-margin-m-s 0.1 \
  --a-dec-m-s2 10.0 --a-dec-source 'provisional' \
  --detection-s 0.10 --latency-s 0.14 --arm-delay-s 0.05 --time-margin-s 0.03 \
  --arm-base-frame base --max-reach-m 1.1 --floor-world-z-m 0.1
```

출력: `gate_candidates.csv` (judge 입력) · `gate_judged.csv` (judge 출력 그대로) · `gate_map.csv` (후보별 두
층의 사유·t_min·γ 창·q\*) · `gate_cell_table.csv` (릴리스 높이 × 거리, 분모는 전체 격자) ·
`gate_map_summary.yaml` (인자 provenance, 층별 열린 투척 수·사유 분포, **각 게이트가 단독으로 거르는 후보
수**, 층별 **열린 후보의 t_c(= 비행시간)·포구 속력과 투척별 포구 창 [min t_c, max t_c] 의 분포**
(`open_candidates` — `catchability_map` 요약과 같은 n/min/p05/…/median/…/max/mean, S3.6 지평 요구의 입력;
층이 아무것도 열지 않으면 `null`), 대기 자세 제안). FK(q\*) 가 `p_model` 과 어긋나면 보고하지 않고 종료한다 (`catch_speed_budget` 와 같은 검사).

- 테스트 `test/test_catch_gate_map.py` (22 케이스): 경로 프로파일, 토크 도달시간의 단일 관절 닫힌해
  (삼각·사다리꼴), **움직이는 관절의 한계만** 결과를 바꾸는지, 회전자 관성, 찾은 이동의 RNEA 재검사와
  더 빠른 이동의 위반, 중력만으로 한계 초과 시 NaN, 층별 사유의 순서, 전체 격자 분모, 열린 후보 통계
  (열린 행만·투척별 창·비유한값 제외), 그리고 **실제
  `catch_gate_batch` 를 부르는** CLI end-to-end — 상수 하나가 자기 게이트만 뒤집는지, 가속 box 가 box
  층만 구속하는지, seed 합집합 거부, FK 불일치 거부, judge 에 넘기는 `J_p q̇ᵘ` 가 요청값 v̂ 이 아니라
  달성값인지 (큰 damping 에서), 속도 0 인 수락 행 거부. 변이 18 종 전부 검출


### `urdf_to_mjcf.py` — URDF/XACRO → MJCF 변환

URDF 또는 XACRO 파일을 MuJoCo MJCF XML로 변환합니다. 관절을 자동 분류(active/passive mimic/closed-chain)하고 후처리를 수행합니다.

```bash
# 디렉토리 규약 기반 (urdf/, mjcf/, meshes/ 자동 탐색)
ros2 run rtc_tools urdf_to_mjcf --robot-dir robots/ur5e

# 디렉토리 내 특정 URDF 지정
ros2 run rtc_tools urdf_to_mjcf --robot-dir robots/ur5e --urdf-file ur5e_with_hand.urdf.xacro

# 명시적 입출력 경로 지정
ros2 run rtc_tools urdf_to_mjcf --input robot.urdf --output robot.xml

# XACRO 인자 전달
ros2 run rtc_tools urdf_to_mjcf --input robot.xacro --xacro-args name:=ur5e

# 씬 파일 생성 + 변환 후 검증
ros2 run rtc_tools urdf_to_mjcf --robot-dir robots/ur5e --scene --validate
```

**변환 파이프라인:**

| 단계 | 설명 |
|------|------|
| 1 | XACRO 처리 (자동 감지) 및 `package://` URI 해석 |
| 2 | 관절 분류: active / passive mimic / closed-chain / fixed |
| 3 | closed-chain 관절 제거 (MuJoCo 트리 토폴로지 요구) |
| 4 | MuJoCo로 URDF 컴파일 및 raw MJCF 저장 |
| 5 | 후처리: compiler 수정, `<option>` 추가, 메시 경로 정리 |
| 6 | `<equality>` 제약조건 (mimic/connect) 삽입 |
| 7 | `<actuator>` 생성 (active 관절만) |
| 8 | scene.xml 생성 (`--scene` 옵션) |
| 9 | MJCF/URDF 검증 (`--validate` 옵션) |

**디렉토리 규약:**
```
robot_dir/
├── urdf/     ← URDF 파일 (자동 탐색: <robot_name>.urdf 우선)
├── mjcf/     ← MJCF 출력
└── meshes/   ← 메시 파일
    └── assets/  (OBJ 파일 우선 탐색)
```

---

### `ctf_to_chrome_trace.py` — LTTng CTF trace → Chrome Trace JSON

`ros2 launch ... enable_tracing:=true` (`rtc_tools.launch.trace_action`) 로 수집한 LTTng CTF trace 를 [Perfetto UI](https://ui.perfetto.dev)에 드래그-드롭 가능한 Chrome Trace JSON 으로 변환합니다. `console_scripts` entry point 로 등록되어 있지 않으므로 `python3 -m`으로 직접 실행합니다.

```bash
python3 -m rtc_tools.conversion.ctf_to_chrome_trace \
    --input logging_data/260520_1430/tracing/trace --output trace.json

# 또는 babeltrace2 CLI 출력을 직접 파이프
babeltrace2 logging_data/260520_1430/tracing/trace \
    | python3 -m rtc_tools.conversion.ctf_to_chrome_trace --stdin --output trace.json
```

- Perfetto 에 **Threads (by TID)** / **Cpus** 2개 swimlane 그룹을 동시에 생성 — 스레드별 실행 구간과 core별 스케줄링(taskset 핀 검증, migration/IRQ 탐지)을 모두 확인 가능
- 처리 이벤트: `ros2:callback_start/end` (B/E 슬라이스), `rtc:span_begin/end` (RT-tick 내부 `RTC_TRACE_SCOPE` 중첩 span — Threads 레인, `-DRTC_ENABLE_TRACING=ON` 빌드에만 존재), `sched_switch` (Cpu 레인), `irq_handler_entry/exit` (Cpu/IRQ 레인). 그 외 이벤트는 기본 drop — `--keep-events name[,...]` 로 개별 opt-in, `--keep-all` 로 전체 복원
- Callback 슬라이스 이름: `ros2:rclcpp_callback_register` 이벤트의 주소→symbol 매핑으로 해석 (기본 UST 캡처에 포함). register 이벤트가 없는 캡처 (노드 기동 후 시작한 수동 trace, 좁힌 `trace_events_ust`) 는 `callback@0x...` 주소로 fallback
- 파서: `python3-bt2` (LTTng Python binding) 우선, 미설치 시 `babeltrace2` CLI 텍스트 출력 파싱으로 폴백 (느림)

---

### `compare_mjcf_urdf.py` — MJCF vs URDF 파라미터 비교 검증

`robot_descriptions` 패키지의 MJCF와 URDF를 파싱하여 물리 파라미터 동일성을 검증합니다.

```bash
# 자동 경로 탐색 (ament_index 또는 상대 경로)
ros2 run rtc_tools compare_mjcf_urdf

# 수동 경로 지정
ros2 run rtc_tools compare_mjcf_urdf \
    --mjcf /path/to/ur5e.xml --urdf /path/to/ur5e.urdf

# 패키지 레이아웃으로 경로 해석 (<robot-pkg>/robots/<robot-name>/...)
ros2 run rtc_tools compare_mjcf_urdf --robot-pkg robot_descriptions --robot-name ur5e

# 비교 범위 좁히기: MJCF default class 루트 · 관절 목록 (생략 시 자동 탐지, MJCF ∩ URDF)
ros2 run rtc_tools compare_mjcf_urdf --robot-pkg robot_descriptions --robot-name ur5e \
    --mjcf-class ur5e --joints shoulder_pan_joint shoulder_lift_joint

# tolerance 조정 (기본: 1e-4)
ros2 run rtc_tools compare_mjcf_urdf --tolerance 0.01

# 두 파일의 world frame 이 다를 때 공통 기준 프레임 선언 (아래 참조)
ros2 run rtc_tools compare_mjcf_urdf --align-frames world base \
    --mjcf /path/to/ur5e.xml --urdf /path/to/ur5e.urdf
```

**`--align-frames <MJCF_FRAME> <URDF_FRAME>`** — MJCF 는 로봇 루트 body 를 씬 작성자가 정한 자리에 mount 하고 URDF 의 world 는 루트 링크다. 두 world 가 다르면 world-frame FK 비교가 **로봇 전체 오프셋**을 뿜는데, 그건 모델 발산이 아니라 mounting 규약이다 (ur5e: MJCF world = UR "Base"(DH) 프레임, URDF world = REP-103 `base_link`). 물리적으로 같은 프레임을 **양쪽에서 하나씩 선언**하면 그 갭이 닫힌다. **이름이 엇갈리는 데 주의** — ur5e 의 MJCF body `base` 는 URDF 링크 `base` 가 아니라 `base_link` 에 대응한다. 미지정 시 두 world 가 일치한다고 가정한다.

> 추정이 아니라 선언인 이유: 변환을 데이터에 최소자승으로 맞추면 **진짜 발산이 그 fit 에 흡수된다** — 이 센서가 잡으려는 바로 그 실패다.

**massless URDF 프레임은 mismatch 로 세지 않는다.** 질량 0 의 링크는 순수 좌표 프레임이라 MuJoCo 가 body 를 안 만드는 것이 정상이므로 `[NOTE]` 로만 알린다. 단 면제는 **MuJoCo 가 실제로 만들지 않은 것에 한정**된다 — iiwa7 은 1개, leap_hand 는 5개의 massless 프레임을 실제 body 로 갖고 있어서 일괄 제외하면 그쪽 body count 가 깨진다. **질량을 가진 링크의 소실은 여전히 mismatch** 다 (fusestatic 이 질량을 부모로 흡수한 경우).

**링크 존재 판정은 `--link-map` 을 거친다** — 같은 이름이 서로 다른 것을 가리킬 수 있기 때문이다. ur5e 의 URDF 에는 massless `base` 프레임과 4 kg `base_link_inertia` 가 둘 다 있고 MJCF 의 `base` body 는 후자다. 이름만으로 맺으면 massless 프레임이 무거운 body 를 차지해 진짜 링크가 "lost" 로 보고된다.

**`--link-map` 은 예외 목록이지 작업 목록이 아니다.** 파일에 적힌 것은 *이름이 엇갈리는 쌍*뿐이고, 나머지 동명 쌍은 그대로 전부 비교된다 — 선언이 비교 범위를 **좁히지 않는다**. 충돌 시 선언이 이긴다 — 어떤 URDF 링크를 명시 항목이 이미 가리키면 동명 body 가 그것을 다시 채가지 못한다. 리포트는 `Link pairs compared: N` 과 짝을 못 찾은 body/link 목록을 찍으므로, 좁아졌다는 사실 자체가 관측된다.

**병합된 링크는 `fuse:` 로 선언한다.** MJCF 가 fixed joint 자식을 부모 body 에 접는 것은 정당한 모델링인데, 선언 수단이 없으면 도구가 이를 **2중 오탐**한다 — 자식이 "mass lost" 로, 부모가 "MASS MISMATCH" 로. 그러면 그런 모델은 게이트에 못 넣는다. link_map 파일의 structured form 이 이를 표현한다:

```yaml
links:                                   # 이름이 엇갈리는 쌍 (생략 가능)
  base: base_link_inertia
fuse:                                    # MJCF body ← 접혀 들어간 URDF 링크들
  index_dip_fe_link: [index_tip_link]
```

flat form (`base: base_link_inertia`) 은 그대로 동작한다 — **값이 mapping 인지**로 두 형식을 구분하므로 `links` · `fuse` 라는 이름의 링크를 가진 로봇도 안전하다.

선언된 자식은 비교 **전에 합성**된다: 질량 합, 질량가중 COM, 평행축 정리로 옮긴 관성 텐서 합. 합성은 URDF world frame (zero configuration) 에서 수행한 뒤 부모 링크 프레임으로 되돌리므로 fixed joint 의 `rpy` 와 임의 깊이의 체인이 추가 코드 없이 처리된다. **선언이 검사를 무력화하지 않는다** — 합성된 질량·COM·주모멘트가 전부 그대로 비교되고, 자식으로 향하는 경로에 fixed 가 아닌 관절이 하나라도 있으면 선언 자체가 mismatch 다 (그게 없으면 "접었다고 선언" 이 임의의 발산을 지우는 수단이 된다). 합성 body 에 대해서는 collision-geometry plausibility 추정이 부모 링크의 형상만 보므로 검사를 돌리지 않고 그 사실을 `[NOTE]` 로 알린다.

**링크 COM 은 world frame 에서 비교한다.** 로컬 프레임 COM 은 비교 대상이 아니다 — MJCF 는 body 원점을 visual mesh 기준, URDF 는 DH 기준으로 두므로 같은 물리적 COM 이 두 프레임에서 다르게 읽힌다 (ur5e `forearm_link` 실측 0.242 m 차이, 전부 규약). zero configuration 기준 world 로 올리면 그 차이가 사라진다 (같은 3쌍 실측: 최대 5e-7). 관절을 축 *직선* 으로 비교하는 것과 같은 이유다. `--align-frames` 는 COM 비교에도 적용된다. 이 검사가 없으면 **질량과 주모멘트가 그대로인 채 COM 만 옮겨진 발산이 조용히 통과**한다 — 주모멘트는 COM 기준 회전불변량이라 평행이동에 반응하지 않는다.

**`--tip-frames <MJCF_FRAME> <URDF_FRAME>`** — tool 프레임을 직접 비교한다. 관절 비교가 축 *직선* 기준이라 **마지막 관절 이후의 오프셋(DH `d6`)을 원리적으로 못 본다** — 그 오프셋은 마지막 관절 자신의 축과 평행하고, 링크 COM 도 움직이지 않는다(실측 확인). ur5e 는 `--tip-frames attachment_site tool0`. MJCF 쪽은 body 또는 **site** 이름을 받는다.

**`--fail-on-unverified`** — 아예 실행되지 못한 검사가 있으면 exit 1. 없으면 mujoco 를 import 못 해도 구조 비교가 통째로 빠진 채 `Mismatches: 0` / exit 0 이 나온다 — **게이트에는 필수**다. 이 플래그 없이도 요약은 `UNVERIFIED: N` 과 "this is NOT a clean pass" 를 찍는다 (warning 과 합치지 않는다 — warning 은 "봤는데 괜찮다", unverified 는 "안 봤다").

**관절 위치는 축 *직선* 으로 비교한다.** Revolute 관절의 원점은 자기 축 위 어디에 놓든 물리가 안 바뀌고, MJCF 는 visual-mesh 기준·URDF 는 DH 기준으로 원점을 다르게 놓는 것이 정상이다. 따라서 두 축 직선의 **수직 거리**만 mismatch 로 세고 축 방향 성분은 `[NOTE]` 로 알린다 (ur5e 실측: 축 방향 성분 최대 138 mm, 수직 성분 전부 0.8 mm 미만). Prismatic 관절은 원점이 곧 zero position 이므로 **점 비교를 유지**한다.

**비교 항목:**

| 항목 | MJCF 소스 | URDF 소스 |
|------|-----------|-----------|
| Link mass | `<inertial mass>` | `<mass value>` |
| Link COM (**world frame**) | `<inertial pos>` + body FK | `<inertial><origin xyz>` + link FK |
| Diagonal inertia | `diaginertia` | `ixx, iyy, izz` |
| Off-diagonal inertia | 없음 (0 가정) | `ixy, ixz, iyz` (비정상 시 경고) |
| Inertial frame rotation | quaternion | rpy (회전 시 경고) |
| Joint position limits | `range` (default class 상속) | `<limit lower/upper>` |
| Joint effort limits | `forcerange` (default class 상속) | `<limit effort>` |
| Joint axis (world frame) | `axis` (FK 변환) | `<axis xyz>` (FK 변환), 평행성 검사 (anti-parallel 허용) |
| Joint position (world frame) | `<body pos/quat>` (FK 누적) | `<joint origin xyz/rpy>` (FK 누적) |
| Armature | `<joint armature>` | N/A (MJCF 전용, 참고 표시) |

**MJCF default class 해석**: `ur5e` → `size3` → `size3_limited` / `size1` 상속 체인 자동 해석

**종료 코드**: mismatch가 0이면 `0`, 아니면 `1` (CI 통합 가능)

---

### `hand_udp_sender_example.py` — 10-DOF 손 UDP 프로토콜 라이브러리 + 예제

개발/테스트용 UDP 손 통신 라이브러리 및 합성 데이터 생성기입니다. `HandUDPSender` 클래스로 request-response 통신, CSV 로깅, 장애 감지를 지원합니다.

```bash
ros2 run rtc_tools hand_udp_sender_example
```

대화형 프롬프트에서 대상 IP, 센서 수(0~4), CSV 저장 여부, 실행 모드를 선택합니다.

**패킷 프로토콜:**

| 패킷 | 크기 | 구조 |
|------|------|------|
| 모터 패킷 | 43B | `[ID:1B][CMD:1B][MODE:1B][10 x float32]` |
| 모터 일괄 응답 | 123B | `[ID:1B][CMD:1B][MODE:1B][30 x float32]` (pos+vel+cur) |
| 센서 요청 | 3B | `[ID:1B][CMD:1B][MODE:1B]` (헤더만) |
| 센서 응답 | 67B | `[ID:1B][CMD:1B][MODE:1B][16 x uint32]` → 유효 11개 (기압 x8 + ToF x3) |
| 센서 일괄 응답 | 259B | `[ID:1B][CMD:1B][MODE:1B][64 x uint32]` (4핑거) |

**MODE 필드**: joint 관련 명령(`WritePosition` 0x01, `ReadAllMotors` 0x10, `ReadPosition` 0x11, `ReadVelocity` 0x12)에서 motor-space 대 joint-space 를 구분합니다:

| 상수 | 값 | 의미 |
|------|-----|------|
| `JOINT_MODE_MOTOR` | `0x00` | raw motor encoder position (기본값, 하위 호환) |
| `JOINT_MODE_JOINT` | `0x01` | joint-space position (기어비 적용, 펌웨어 변환) |

**명령 코드:**

| 명령 | 코드 | 방향 | 설명 |
|------|------|------|------|
| WritePosition | `0x01` | → | 10개 모터 목표 위치 전송 |
| SetSensorMode | `0x04` | → | 센서 모드 설정 (RAW/NN) |
| ReadAllMotors | `0x10` | ↔ | 10개 모터 pos+vel+cur 일괄 요청 (3B → 123B) |
| ReadPosition | `0x11` | ↔ | 현재 모터 위치 요청 |
| ReadVelocity | `0x12` | ↔ | 현재 모터 속도 요청 |
| ReadSensor0-3 | `0x14-0x17` | ↔ | 손가락별 센서 데이터 요청 (최대 4개) |
| ReadAllSensors | `0x19` | ↔ | 센서 4개 일괄 요청 (3B → 259B) |

**실행 모드 (6가지):**

| 모드 | 설명 |
|------|------|
| 1. WriteOnly | 정현파 모터 명령 전송 (피드백 없음) |
| 2. PollCycle | 전체 사이클: WritePos + ReadPos + ReadVel + ReadSensor x4 |
| 3. StaticPose | 고정 모터 위치 전송 |
| 4. ReadOnly | 쓰기 없음; ReadPos + ReadVel + ReadSensor x4만 수행 |
| 5. BulkPollCycle | WritePos + ReadAllMotors(0x10) + ReadAllSensors(0x19) |
| 6. BulkReadOnly | ReadAllMotors(0x10) + ReadAllSensors(0x19), 쓰기 없음 |

**주요 클래스:**

| 클래스 | 설명 |
|--------|------|
| `HandUDPSender` | UDP request-response 통신 (모터 커맨드 43B, 센서 요청 3B) |
| `UdpTimingStats` | 통신 타이밍 측정 (cycle, write, read 구간별 ms 단위) |
| `HandDataCsvLogger` | 타임스탬프, 모터 위치/속도/전류, 센서 데이터 CSV 자동 기록 |
| `HandDataFailureDetector` | 연속 0-데이터 또는 중복 데이터 5회 초과 시 자동 종료 |

---

### `hand_data_plot.py` — 손 CSV 데이터 시각화

`hand_udp_sender_example.py`에서 저장한 CSV 로그를 Matplotlib으로 시각화합니다. 센서 수 및 bulk/legacy 모드를 자동 감지합니다.

> **참고**: entry_point 미등록 — `python3`으로 직접 실행합니다.

```bash
# 모터 + 센서 + 타이밍 전체 플롯
python3 rtc_tools/utils/hand_data_plot.py <csv_file>

# 특정 모터만 플롯
python3 rtc_tools/utils/hand_data_plot.py <csv_file> --motors 0 1 2

# 센서만 플롯
python3 rtc_tools/utils/hand_data_plot.py <csv_file> --sensors-only

# 모터만 플롯
python3 rtc_tools/utils/hand_data_plot.py <csv_file> --motors-only

# 타이밍만 플롯
python3 rtc_tools/utils/hand_data_plot.py <csv_file> --timing-only
```

**생성 플롯:**

| 플롯 | 내용 |
|------|------|
| 모터 위치 | pos_0..9 (10개 트레이스) vs 시간 |
| 모터 속도 | vel_0..9 (10개 트레이스) vs 시간 |
| 모터 전류 | cur_0..9 (bulk 모드 전용) vs 시간 |
| 기압 센서 | 손가락별 (최대 4개), 기압 채널 8개씩 (uint32) |
| ToF 센서 | 손가락별 (최대 4개), ToF 채널 3개씩 (uint32) |
| 타이밍 시계열 | cycle/write/read 구간별 ms (평균선 포함) |
| 타이밍 히스토그램 | 구간별 분포 (avg/std/min/max/p99 표시) |
| 타임아웃 | 사이클당 타임아웃 횟수 (존재 시) |

---

### `session_dir.py` — 세션 디렉토리 유틸리티

C++ `rtc_base/logging/session_dir.hpp` 와 **동일한 4단 체인**으로 세션
디렉토리를 결정합니다. launch 파일과 CLI 툴이 모두 이 모듈을 사용해 같은
경로에서 세션을 생성·재사용하도록 하는 것이 목적입니다.

**로깅 루트 결정 (`resolve_logging_root`)**:

1. `$COLCON_PREFIX_PATH` 첫 entry 가 쓰기 가능한 디렉토리이면 그 `parent / "logging_data"`
2. cwd 에서 상위로 올라가며 `install/` + `src/` 쌍 발견 시 그 디렉토리 `/ "logging_data"`
3. 최종 폴백: `$PWD / "logging_data"`

**세션 디렉토리 결정 (`get_session_dir` / `create_session_dir`)**:

1. `$RTC_SESSION_DIR`
2. `resolve_logging_root() / "YYMMDD_HHMM"` 을 새로 생성

```python
from rtc_tools.utils.session_dir import (
    resolve_logging_root,
    create_session_dir,
    cleanup_old_sessions,
    get_session_dir,
    get_or_create_session_dir,
    get_session_subdir,
)

# launch 파일에서 신규 세션 생성
root = resolve_logging_root()
session = create_session_dir(root)
cleanup_old_sessions(root, max_sessions=10)

# CLI 툴에서 현재 실행 중인 세션에 쓰거나 없으면 새로 만들기
session = get_or_create_session_dir()
plots = get_session_subdir('plots')  # 환경변수 읽기 전용, None 반환 가능
```

| 함수 | 설명 |
|------|------|
| `resolve_logging_root()` | 3단 체인으로 `logging_data` 루트 경로 결정 |
| `create_session_dir(root=None)` | `YYMMDD_HHMM` 세션과 6개 서브디렉토리 생성 |
| `cleanup_old_sessions(root, max)` | `YYMMDD_HHMM` 패턴 세션만 대상으로 개수 제한 |
| `generate_run_id()` | 이번 launch 의 런 ID (`YYMMDDHHMMSS`). launch 가 `RTC_RUN_ID` 로 전파하고 C++ `rtc::ResolveRunId()` 가 소비 |
| `get_session_dir()` | `RTC_SESSION_DIR` 읽기 (없으면 `None`) |
| `get_or_create_session_dir()` | env 우선, 없으면 새 세션 생성 |
| `get_session_subdir(name)` | 현재 세션 하위 폴더 경로 반환 (자동 생성, 세션 미설정 시 `None`) |

---

### `thread_layout.py` — 스레드 코어 배치 SSoT의 Python mirror

코어 티어 breakpoint 의 Python 미러입니다. **재인코딩이 아니라 생성물**이며, 표 자체는 `thread_layout_generated.py` 에 선언형 manifest [repo_scripts/config/thread_layout.yaml](../repo_scripts/config/thread_layout.yaml) 로부터 생성됩니다 — C++ tier 상수·shell 헬퍼와 같은 출처입니다. Launch 파일(Python)이 외부 driver/simulator 프로세스에 `taskset` 핀을 적용할 때 C++ RT 루프와 동일한 코어 배치 결정을 내리기 위해 사용됩니다.

```python
from rtc_tools.launch.thread_layout import select_thread_layout, get_physical_cpu_count

layout = select_thread_layout()          # physical core 자동 감지
arm_core = layout.arm_driver_core        # -1 = pinning 생략 (no-op)
```

| 함수 | 설명 |
|------|------|
| `get_physical_cpu_count()` | `lscpu -p=Core,Socket` 기반 physical(non-SMT) 코어 수 (실패 시 `os.cpu_count()` 폴백) |
| `select_thread_layout(physical_cores=None)` | 코어 수 → `ThreadLayout` (arm/hand/sim/viewer/rt_callback 코어) |
| `get_arm_driver_core()` / `get_hand_driver_core()` / `get_sim_core()` / `get_viewer_core()` / `get_rt_callback_core()` | 개별 코어 인덱스 accessor |

C++ SSoT와의 drift는 `test/test_thread_layout.py`가 계약 형태로 고정해 검증합니다 — 어느 한쪽의 티어 breakpoint 가 바뀌면 테스트가 실패합니다.

---

### `trace_action.py` — ros2_tracing (LTTng) capture 액션 헬퍼

Bringup launch 파일이 `enable_tracing` / `trace_session_name` / `trace_events_ust` / `trace_events_kernel` LaunchArgument 를 선언하고, 세션 디렉토리가 결정된 `OpaqueFunction` 안에서 `make_trace_action()`을 호출하는 패턴을 위한 헬퍼입니다.

```python
from rtc_tools.launch.trace_action import make_trace_action

def launch_setup(context):
    actions = make_trace_action(context, session_dir=session_dir)  # [] 가능
    return actions
```

- ros2_tracing 심볼 (`tracetools_launch.action.Trace` + `tracetools_trace.tools.names.DEFAULT_EVENTS_ROS`) 을 **하나의 guard 안에서** lazy import — ros2_tracing 미설치 환경에서도 launch 파싱은 가능하며, `enable_tracing:=true`인데 미설치 (부분 설치 포함) 면 `./install.sh --tracing` 안내 메시지 후 no-op
- Trace 결과물은 `<session_dir>/tracing/<session_name>/` (CSV timing log 와 같은 세션 트리) 에 저장 — `~/.ros/tracing/`의 ros2_tracing 기본 경로가 아님
- `enable_tracing:=false` 시 빈 리스트 반환 (no-op)

---

## Testing

`rtc_tools/test/` 에 pytest 기반 유닛 테스트가 있습니다 (validation / conversion / launch / plotting / utils 서브모듈 커버). 개수는 여기 박제하지 않음 — 실측은 아래 명령으로 확인:

```bash
colcon test --packages-select rtc_tools --event-handlers console_direct+
colcon test-result --verbose
```

---

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select rtc_tools --symlink-install
source install/setup.bash
```

**Python 의존성 설치:** `install.sh` 가 `uv pip sync requirements.lock` 으로 자동 처리. 수동 시:
```bash
# venv lock — numpy / scipy / matplotlib / pandas / PyQt5 + mujoco + cython + ruff
cd ~/ros2_ws/rtc_ws && uv pip sync src/rtc-framework/requirements.lock
```

---

## 의존성

**package.xml 기준 (rosdep 해결):**

| 타입 | 패키지 |
|------|--------|
| build_type (export) | `ament_python` |
| exec | `rclpy`, `std_msgs`, `sensor_msgs`, `rtc_msgs`, `ament_index_python` |
| test | (개별 lint — `ament_lint_common` meta + `ament_uncrustify` 는 워크스페이스 정책상 사용 금지; 자세한 사유: [agent_docs/conventions.md](../agent_docs/conventions.md)) |

> Python scientific stack (`numpy` / `scipy` / `matplotlib` / `pandas` / `PyQt5`) 과 `mujoco` 는 package.xml 에 두지 않는다 — cross-workspace isolation 정책으로 모두 venv `requirements.lock` 책임. `rclpy` 만 ROS Jazzy 가 책임.

**venv lock 기준** ([requirements.in](../requirements.in) → [requirements.lock](../requirements.lock)):

| 패키지 | 버전 | 비고 |
|--------|------|------|
| numpy | <2 (1.26.4) | ros-jazzy-rclpy ABI 호환 핀 |
| scipy / matplotlib / pandas / PyQt5 | latest | scientific stack + GUI |
| mujoco | 3.7.0 | urdf_to_mjcf / compare_mjcf_urdf 런타임 |
| Cython | 3.2.4 | mujoco wheel build 등 |
| ruff | 0.7.4 | formatter / linter (`pyproject.toml` SSoT) |
| setuptools | <80 | colcon-core 0.20.1 호환 |
| wheel | latest | sdist build |

venv 는 `--system-site-packages` 로 만들어져 ROS `rclpy` / `ament_*` / `python3-bt2` 등 시스템 책임 모듈을 상속하지만, scientific stack 5종은 venv 안 pinned 버전이 sys.path 에서 우선.

---

## 라이선스

MIT License
