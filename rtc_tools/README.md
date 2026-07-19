# rtc_tools


> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md)

RTC 프레임워크의 **Python 개발 유틸리티 패키지**입니다. 로그 시각화, UDP 손 통신, MJCF/URDF 모델 변환 및 검증, 트레이싱/스레드 배치 launch 헬퍼 도구를 포함합니다 (레거시 컨트롤러 GUI는 아래 참고 참조).

## 개요

```
rtc_tools/
├── rtc_tools/
│   ├── gui/
│   │   └── (empty)                       ← controller_gui.py removed in
│   │                                       Phase F-2 (2026-04-26); the
│   │                                       supported demo GUI lives under
│   │                                       integrated_bringup/scripts/
│   │                                       demo_controller_gui.py
│   ├── monitoring/
│   │   └── __init__.py
│   ├── plotting/
│   │   └── plot_rtc_log.py              ← Matplotlib 로그 시각화 (v5, 4-카테고리) —
│   │                                       thin orchestration layer; 실제 구현은
│   │                                       columns/, io/, pipelines/, plotters/,
│   │                                       layout.py 서브모듈로 분리되어 있음
│   ├── validation/
│   │   └── compare_mjcf_urdf.py         ← MJCF vs URDF 파라미터 비교 검증
│   ├── conversion/
│   │   ├── urdf_to_mjcf.py             ← URDF/XACRO → MJCF 변환 (관절 분류 + 후처리)
│   │   └── ctf_to_chrome_trace.py      ← LTTng CTF trace → Chrome Trace JSON (Perfetto UI)
│   ├── launch/
│   │   ├── thread_layout.py            ← 스레드 코어 배치 SSoT의 Python mirror
│   │   │                                  (rtc_base SelectThreadConfigs, drift-tested)
│   │   └── trace_action.py             ← ros2_tracing (LTTng) capture 액션 헬퍼
│   └── utils/
│       ├── hand_udp_sender_example.py   ← 10-DOF 손 UDP 프로토콜 라이브러리 + 예제
│       ├── session_dir.py               ← 세션 디렉토리 유틸리티 (RTC_SESSION_DIR 관리)
│       └── hand_data_plot.py            ← 손 CSV 데이터 시각화
├── test/                                 ← pytest 유닛 테스트 (Testing 섹션 참조)
├── resource/
│   └── rtc_tools
├── package.xml
├── setup.py
└── setup.cfg
```

**빌드 타입**: `ament_python` (`setup.py`의 `entry_points` 사용)

**Entry points** (6개):

| 실행 명령 | 모듈 | 설명 |
|-----------|------|------|
| `ros2 run rtc_tools plot_rtc_log` | `plotting.plot_rtc_log` | CSV 로그 시각화 |
| `ros2 run rtc_tools plot_ur_log` | `plotting.plot_rtc_log` | plot_rtc_log의 별칭 |
| `ros2 run rtc_tools plot_ur_trajectory` | `plotting.plot_rtc_log` | legacy 별칭 |
| `ros2 run rtc_tools hand_udp_sender_example` | `utils.hand_udp_sender_example` | 핸드 UDP 테스트 (대화형) |
| `ros2 run rtc_tools compare_mjcf_urdf` | `validation.compare_mjcf_urdf` | MJCF/URDF 파라미터 비교 |
| `ros2 run rtc_tools urdf_to_mjcf` | `conversion.urdf_to_mjcf` | URDF/XACRO → MJCF 변환 |

**Python 의존성**: `rclpy`, `std_msgs`, `sensor_msgs`, `rtc_msgs`, `numpy`, `matplotlib`, `pandas`, `scipy`, `mujoco`

---

## 스크립트 설명

> **Note:** The legacy `controller_gui.py` was removed in Phase F-2 (2026-04-26)
> together with the gain → ROS 2 parameter migration. It targeted the four
> core controllers (P / JointPD / CLIK / OSC) which never exposed a
> runtime-tunable gain channel. The supported demo GUI for the
> three demo controllers (DemoJoint / DemoTask / DemoWbc) lives at
> [integrated_bringup/scripts/demo_controller_gui.py](../integrated_bringup/scripts/demo_controller_gui.py).

### `plot_rtc_log.py` — 로그 시각화 (v5, 4-카테고리)

분리된 CSV 제어 로그를 Matplotlib으로 시각화합니다. 파일 이름 패턴으로 로그 타입을 자동 감지합니다.

```bash
# State 로그 (DeviceStateLog) 시각화
ros2 run rtc_tools plot_rtc_log <device>_state_log.csv

# Sensor 로그 (DeviceSensorLog) 시각화
ros2 run rtc_tools plot_rtc_log <device>_sensor_log.csv

# 타이밍 로그 시각화 (CM RT loop / MPC main loop — 동일 7-col 스키마)
ros2 run rtc_tools plot_rtc_log cm_timing_log.csv
ros2 run rtc_tools plot_rtc_log mpc_timing_log.csv

# 플롯 파일로 저장
ros2 run rtc_tools plot_rtc_log <device>_state_log.csv --save-dir /tmp/plots

# 통계만 출력 (플롯 없이)
ros2 run rtc_tools plot_rtc_log <device>_state_log.csv --stats

# 모든 Figure 한 번에 생성
ros2 run rtc_tools plot_rtc_log <device>_state_log.csv --all
```

> `--save-dir` 미지정 시 `RTC_SESSION_DIR/plots/`에 자동 저장됩니다.
>
> `--save-dir` 지정 시 Agg backend 자동 사용 (GUI 없이 렌더링).

**파일 이름 자동 감지:**

| 패턴 | 모드 |
|------|------|
| `*_state_log.csv` | state_log (DeviceStateLog 필드) |
| `*_sensor_log.csv` | sensor_log (DeviceSensorLog 필드, 컬럼 수 불일치 자동 복구) |
| `cm_timing_log*.csv` | cm_timing (CM RT loop) |
| `mpc_timing_log*.csv` | mpc_timing (MPC main loop) |
| `<dev>_state.csv` (WBC, `accel_*` 컬럼) | wbc_log (DeviceWbcLog — state_log superset: TSID a_opt 가속도 + SE3 trajectory(arm) / fingertip force(hand)) |
| `wbc_diag.csv` | wbc_diag (WbcDiagLog — per-tick TSID/QP 진단: solve time / λ / 수렴 / grasp) |
| `pull_estimator.csv` | pull_estimator (PullEstimatorLog #167 — in-plane pull-force estimate: raw+filtered force / in-plane·magnitude·directional / friction util·leakage / validity 플래그. 4×1 sharex 단일 figure + 통계) |

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
| Figure 6 | (자동, `ft_*` 존재 시) | Per-finger 실측 force Fx/Fy/Fz (N×1, sharex — WBC fingertip force figure 와 동일 레이아웃, #167) |
| Figure 7 | `--sensor-compare` | Raw vs Filtered 센서 오버레이 비교 |

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

# tolerance 조정 (기본: 1e-4)
ros2 run rtc_tools compare_mjcf_urdf --tolerance 0.01
```

**비교 항목:**

| 항목 | MJCF 소스 | URDF 소스 |
|------|-----------|-----------|
| Link mass | `<inertial mass>` | `<mass value>` |
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
| `get_session_dir()` | `RTC_SESSION_DIR` 읽기 (없으면 `None`) |
| `get_or_create_session_dir()` | env 우선, 없으면 새 세션 생성 |
| `get_session_subdir(name)` | 현재 세션 하위 폴더 경로 반환 (자동 생성, 세션 미설정 시 `None`) |

---

### `thread_layout.py` — 스레드 코어 배치 SSoT의 Python mirror

C++ SSoT (`rtc_base/include/rtc_base/threading/thread_config.hpp` + `thread_utils.hpp::SelectThreadConfigs`)의 코어 티어 breakpoint 를 Python 으로 재인코딩합니다. Launch 파일(Python)이 외부 driver/simulator 프로세스에 `taskset` 핀을 적용할 때 C++ RT 루프와 동일한 코어 배치 결정을 내리기 위해 사용됩니다.

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

- `tracetools_launch.action.Trace` 를 lazy import — ros2_tracing 미설치 환경에서도 launch 파싱은 가능하며, `enable_tracing:=true`인데 미설치면 `./install.sh --tracing` 안내 메시지 후 no-op
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
| test | (개별 lint — `ament_lint_common` meta + `ament_uncrustify` 는 워크스페이스 정책 `bdedac7` 으로 사용 금지; 자세한 사유: [agent_docs/conventions.md](../agent_docs/conventions.md)) |

> Python scientific stack (`numpy` / `scipy` / `matplotlib` / `pandas` / `PyQt5`) 과 `mujoco` 는 package.xml 에 두지 않는다 — cross-workspace isolation 정책 (2026-05-23) 으로 모두 venv `requirements.lock` 책임. `rclpy` 만 ROS Jazzy 가 책임.

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
