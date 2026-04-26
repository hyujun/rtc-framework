# rtc_tools

![version](https://img.shields.io/badge/version-v5.17.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md)

RTC 프레임워크의 **Python 개발 유틸리티 패키지**입니다. 컨트롤러 GUI, 로그 시각화, UDP 손 통신, MJCF/URDF 모델 변환 및 검증 도구를 포함합니다.

## 개요

```
rtc_tools/
├── rtc_tools/
│   ├── gui/
│   │   └── (empty)                       ← controller_gui.py removed in
│   │                                       Phase F-2 (2026-04-26); the
│   │                                       supported demo GUI lives under
│   │                                       ur5e_bringup/scripts/
│   │                                       demo_controller_gui.py
│   ├── monitoring/
│   │   └── __init__.py
│   ├── plotting/
│   │   └── plot_rtc_log.py              ← Matplotlib 로그 시각화 (v5, 4-카테고리)
│   ├── validation/
│   │   └── compare_mjcf_urdf.py         ← MJCF vs URDF 파라미터 비교 검증
│   ├── conversion/
│   │   └── urdf_to_mjcf.py             ← URDF/XACRO → MJCF 변환 (관절 분류 + 후처리)
│   └── utils/
│       ├── hand_udp_sender_example.py   ← 10-DOF 손 UDP 프로토콜 라이브러리 + 예제
│       ├── session_dir.py               ← 세션 디렉토리 유틸리티 (RTC_SESSION_DIR 관리)
│       └── hand_data_plot.py            ← 손 CSV 데이터 시각화
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
> [ur5e_bringup/scripts/demo_controller_gui.py](../ur5e_bringup/scripts/demo_controller_gui.py).

### `plot_rtc_log.py` — 로그 시각화 (v5, 4-카테고리)

분리된 CSV 제어 로그를 Matplotlib으로 시각화합니다. 파일 이름 패턴으로 로그 타입을 자동 감지합니다.

```bash
# Robot 로그 시각화
ros2 run rtc_tools plot_rtc_log robot_log.csv

# Device/Hand 로그 시각화
ros2 run rtc_tools plot_rtc_log device_log.csv

# 타이밍 로그 시각화
ros2 run rtc_tools plot_rtc_log timing_log.csv

# 플롯 파일로 저장
ros2 run rtc_tools plot_rtc_log robot_log.csv --save-dir /tmp/plots

# 통계만 출력 (플롯 없이)
ros2 run rtc_tools plot_rtc_log robot_log.csv --stats

# 모든 Figure 한 번에 생성
ros2 run rtc_tools plot_rtc_log robot_log.csv --all
```

> `--save-dir` 미지정 시 `RTC_SESSION_DIR/plots/`에 자동 저장됩니다.
>
> `--save-dir` 지정 시 Agg backend 자동 사용 (GUI 없이 렌더링).

**파일 이름 자동 감지:**

| 패턴 | 모드 |
|------|------|
| `*_state_log.csv` | state_log (DeviceStateLog 필드) |
| `*_sensor_log.csv` | sensor_log (DeviceSensorLog 필드, 컬럼 수 불일치 자동 복구) |
| `robot_log*.csv` | robot |
| `device_log*.csv` / `hand_log*.csv` | device |
| `timing_log*.csv` | timing |

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

### `compare_mjcf_urdf.py` — MJCF vs URDF 파라미터 비교 검증

`ur5e_description` 패키지의 MJCF와 URDF를 파싱하여 물리 파라미터 동일성을 검증합니다.

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
| Joint axis | `axis` | `<axis xyz>` |
| Link origin offset | `<body pos>` | `<joint origin xyz>` (컨벤션 차이 경고) |
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

1. `$RTC_SESSION_DIR` → `$UR5E_SESSION_DIR` (하위 호환)
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
| `get_session_dir()` | `RTC_SESSION_DIR` / `UR5E_SESSION_DIR` 읽기 (없으면 `None`) |
| `get_or_create_session_dir()` | env 우선, 없으면 새 세션 생성 |
| `get_session_subdir(name)` | 현재 세션 하위 폴더 경로 반환 (자동 생성, 세션 미설정 시 `None`) |

---

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select rtc_tools --symlink-install
source install/setup.bash
```

**Python 의존성 설치:**
```bash
pip install matplotlib pandas numpy scipy mujoco
```

---

## 의존성

**package.xml 기준:**

| 타입 | 패키지 |
|------|--------|
| buildtool | `ament_python` |
| exec | `rclpy`, `std_msgs`, `sensor_msgs`, `rtc_msgs`, `python3-numpy`, `python3-mujoco` |
| test | `ament_lint_auto`, `ament_lint_common` |

**requirements.txt 기준:**

| 패키지 | 최소 버전 |
|--------|-----------|
| matplotlib | >= 3.5.3 |
| pandas | >= 1.5.3 |
| numpy | >= 1.24.3 |
| scipy | >= 1.10.1 |
| rclpy | >= 0.20.0 |
| mujoco | >= 3.0.0 |

---

## 라이선스

MIT License
