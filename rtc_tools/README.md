# rtc_tools

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스 (v5.16.0)의 일부입니다.
> 설치/빌드: [Root README](../README.md)

UR5e RT Controller 스택의 **Python 개발 유틸리티 패키지**입니다. 컨트롤러 GUI, 모션 편집기, 궤적 시각화, UDP 손 통신, 모델 검증 도구를 포함합니다.

## 개요

```
rtc_tools/
├── rtc_tools/
│   ├── gui/
│   │   ├── controller_gui.py            ← tkinter 다크 테마 컨트롤러 GUI (v5.3.0+)
│   │   └── motion_editor_gui.py         ← Qt5 모션 편집기 GUI (탭/Diff/타이밍)
│   ├── monitoring/
│   │   └── __init__.py
│   ├── plotting/
│   │   └── plot_rtc_log.py               ← Matplotlib 로그 시각화 (v4)
│   ├── validation/
│   │   └── compare_mjcf_urdf.py         ← MJCF vs URDF 파라미터 비교 검증 (v5.7.0+)
│   └── utils/
│       ├── hand_udp_sender_example.py   ← 10-DOF 손 UDP 프로토콜 라이브러리 + 예제
│       ├── session_dir.py               ← 세션 디렉토리 유틸리티 (v5.10.0, RTC_SESSION_DIR 관리)
│       └── hand_data_plot.py            ← 손 CSV 데이터 시각화
├── resource/
│   └── rtc_tools
├── package.xml
├── setup.py
└── setup.cfg
```

**빌드 타입**: `ament_python` (`setup.py`의 `entry_points` 사용)

**Entry points** (7개):
| 실행 명령 | 모듈 | 설명 |
|-----------|------|------|
| `ros2 run rtc_tools controller_gui` | `gui.controller_gui` | 컨트롤러 GUI (tkinter) |
| `ros2 run rtc_tools plot_rtc_log` | `plotting.plot_rtc_log` | CSV 로그 시각화 |
| `ros2 run rtc_tools plot_ur_log` | `plotting.plot_rtc_log` | plot_rtc_log의 별칭 |
| `ros2 run rtc_tools plot_ur_trajectory` | `plotting.plot_rtc_log` | legacy 별칭 |
| `ros2 run rtc_tools hand_udp_sender_example` | `utils.hand_udp_sender_example` | 핸드 UDP 테스트 (대화형) |
| `ros2 run rtc_tools compare_mjcf_urdf` | `validation.compare_mjcf_urdf` | MJCF/URDF 파라미터 비교 |
| `ros2 run rtc_tools urdf_to_mjcf` | `conversion.urdf_to_mjcf` | URDF/XACRO → MJCF 변환 |

> `motion_editor_gui`는 `ur5e_bringup` 패키지로 이동되었습니다.

**Python 의존성**: `rclpy`, `numpy`, `matplotlib`, `pandas`, `scipy`, `mujoco` (urdf_to_mjcf만 필요)

---

## 스크립트 설명

### `controller_gui.py` — 컨트롤러 GUI (v5.3.0+)

tkinter 기반 다크 테마 GUI. 컨트롤러 선택, 게인 튜닝, 관절/태스크 타겟 전송, E-STOP 상태 표시를 단일 창에서 수행합니다.

```bash
# tkinter는 Python 표준 라이브러리 — 별도 설치 불필요
ros2 run rtc_tools controller_gui
```

**기능:**

| 섹션 | 설명 |
|------|------|
| 컨트롤러 선택 | P / Joint PD / CLIK / OSC 라디오 버튼 + "Switch Controller" |
| 게인 패널 | 컨트롤러별 동적 입력 폼 (float 입력 또는 bool 체크박스) |
| Joint Target | 6개 관절 각도 입력 (deg 표시, rad 전송) + ±step 버튼 |
| Task Target | XYZ (m) + RPY (deg) 입력 + ±step 버튼 (CLIK/OSC용) |
| 현재 상태 | 실시간 관절 위치 (rad + deg) + 태스크 위치 (XYZ m, RPY rad + deg) |
| E-STOP 표시 | 실시간 상태 배지 (초록=정상, 빨강=활성) |
| Load Gain | 활성 컨트롤러에서 현재 게인을 요청하여 입력 필드에 자동 채움 |
| Apply Gains | "Currently Applied" 읽기 전용 섹션에 마지막 전송 게인 표시 |

**ROS2 토픽:**

| 방향 | 토픽 | 타입 | 설명 |
|------|------|------|------|
| 발행 | `/target_joint_positions` | `Float64MultiArray` | 관절/태스크 타겟 위치 |
| 발행 | `/rt_controller/controller_type` | `Int32` | 컨트롤러 전환 (0=P, 1=PD, 2=CLIK, 3=OSC) |
| 발행 | `/rt_controller/controller_gains` | `Float64MultiArray` | 게인 업데이트 |
| 발행 | `/rt_controller/request_gains` | `Bool` | 활성 컨트롤러에 현재 게인 요청 |
| 구독 | `/joint_states` | `JointState` | 현재 관절 상태 |
| 구독 | `/rt_controller/current_task_position` | `Float64MultiArray` | 현재 EE 태스크 위치 (6D) |
| 구독 | `/system/estop_status` | `Bool` | E-STOP 상태 |
| 구독 | `/rt_controller/current_gains` | `Float64MultiArray` | 현재 게인 피드백 (Load Gain 응답) |

**컨트롤러별 게인 입력 형식:**

| 컨트롤러 | 인덱스 | 게인 | 총 값 개수 |
|----------|--------|------|-----------|
| P | 0 | kp×6 | 6 |
| Joint PD | 1 | kp×6, kd×6, gravity_comp(bool), coriolis_comp(bool), traj_speed | 15 |
| CLIK | 2 | kp×6, damping, null_kp, null_space(bool), control_6dof(bool) | 10 |
| OSC | 3 | kp_pos×3, kd_pos×3, kp_rot×3, kd_rot×3, damping, gravity_comp(bool), traj_speed, traj_ang_speed | 16 |

---

### `urdf_to_mjcf.py` — URDF/XACRO → MJCF 변환

URDF 또는 XACRO 파일을 MuJoCo MJCF XML로 변환합니다.

```bash
# 기본 변환
ros2 run rtc_tools urdf_to_mjcf --input robot.urdf --output robot.xml

# XACRO 파일 변환 (자동 감지)
ros2 run rtc_tools urdf_to_mjcf --input robot.urdf.xacro --output robot.xml

# 메시 디렉토리 지정
ros2 run rtc_tools urdf_to_mjcf --input robot.urdf --output robot.xml \
    --meshdir /path/to/meshes

# XACRO 인자 전달
ros2 run rtc_tools urdf_to_mjcf --input robot.urdf.xacro --output robot.xml \
    --xacro-args name:=ur5e use_hand:=true
```

**기능:**
- `package://` URI 자동 해석 (ament_index 연동)
- XACRO 파일 자동 감지 및 처리
- 메시 디렉토리 경로 커스텀 지정
- MuJoCo Python 패키지 (`mujoco`) 필요

> **참고:** `motion_editor_gui`는 `ur5e_bringup` 패키지로 이동되었습니다 (`ros2 run ur5e_bringup motion_editor_gui`).

---

### `plot_rtc_log.py` — 로그 시각화 (v4, 4-카테고리)

> **v4**: `plot_ur_trajectory.py`에서 이름 변경. `plot_ur_trajectory`는 legacy alias로 유지됩니다.

분리된 CSV 제어 로그(`robot_log_*.csv`, `hand_log_*.csv`)를 Matplotlib으로 시각화합니다. 파일 이름으로 로그 타입을 자동 감지합니다.

```bash
# Robot 로그 시각화 (위치 + 속도 2개 Figure)
ros2 run rtc_tools plot_rtc_log logging_data/250314_1530/controller/robot_log.csv

# Hand 로그 시각화 (위치 + 속도 + 센서 3개 Figure)
ros2 run rtc_tools plot_rtc_log logging_data/250314_1530/controller/hand_log.csv

# 플롯 파일로 저장
ros2 run rtc_tools plot_rtc_log logging_data/250314_1530/controller/robot_log.csv --save-dir /tmp/plots

# 통계만 출력 (플롯 없이)
ros2 run rtc_tools plot_rtc_log logging_data/250314_1530/controller/robot_log.csv --stats

# 추적 오차 플롯 추가 (robot only)
ros2 run rtc_tools plot_rtc_log logging_data/250314_1530/controller/robot_log.csv --error

# Hand raw sensor + F/T + 비교 플롯
ros2 run rtc_tools plot_rtc_log hand_log.csv --raw --ft --sensor-compare
```

> **v5.10.0**: `--save-dir` 미지정 시 `RTC_SESSION_DIR/plots/`에 자동 저장됩니다.
>
> **v4 최적화**: `--save-dir` 지정 시 Agg backend 자동 사용 (GUI 없이 렌더링).

**파일 이름 자동 감지:**
- `robot_log_*.csv` → Robot 모드
- `hand_log_*.csv` → Hand 모드

**Robot 모드 플롯:**

| Figure | 플래그 | 레이아웃 | 내용 |
|--------|--------|----------|------|
| Figure 1 | (기본) | 3×2 서브플롯 | 관절별 위치 (Goal / Trajectory / Actual) |
| Figure 2 | (기본) | 3×2 서브플롯 | 관절별 속도 (Trajectory / Actual) |
| Figure 3 | `--command` | 3×2 서브플롯 | 관절별 제어 명령 (Position/Torque) |
| Figure 4 | `--torque` | 3×2 서브플롯 | 관절별 실제 토크 |
| Figure 5 | `--task-pos` | 3×1 | TCP 태스크 위치 (X/Y/Z) |
| Figure 6 | `--error` | 2×1 | 위치/속도 추적 오차 |

```bash
# 모든 Figure 한 번에 생성
ros2 run rtc_tools plot_rtc_log robot_log.csv --all
```

**Hand 모드 플롯:**

| Figure | 플래그 | 레이아웃 | 내용 |
|--------|--------|----------|------|
| Figure 1 | (기본) | 2×5 서브플롯 | 모터별 위치 (Goal / Command / Actual) |
| Figure 2 | (기본) | 2×5 서브플롯 | 모터별 속도 (Actual) |
| Figure 3 | (기본) | 2×N 서브플롯 | 센서 (기압 8ch + ToF 3ch per fingertip) |
| Figure 4 | `--raw` | 2×N 서브플롯 | Raw 센서 (pre-LPF, 기압 + ToF) |
| Figure 5 | `--ft` | N×2 서브플롯 | F/T 추론 출력 (Force + Torque per fingertip) |
| Figure 6 | `--sensor-compare` | 2×N 서브플롯 | Raw vs Filtered 센서 오버레이 비교 |

**통계 출력**: 구간 길이(s), 샘플 수, 평균 샘플링 속도(Hz), 관절/모터별 RMS 추적 오차, F/T 추론 통계 (mean/std/min/max)

---

### `compare_mjcf_urdf.py` — MJCF vs URDF 파라미터 비교 검증 (v5.7.0+)

`ur5e_description` 패키지의 `ur5e.xml` (MJCF)과 `ur5e.urdf` (URDF)를 파싱하여 물리 파라미터 동일성을 검증합니다.

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
# 기본 실행 (PollCycle 모드)
ros2 run rtc_tools hand_udp_sender_example

# 특정 IP/포트로 전송
ros2 run rtc_tools hand_udp_sender_example \
    --target-ip 127.0.0.1 --target-port 50001
```

**패킷 프로토콜:**

| 패킷 | 크기 | 구조 |
|------|------|------|
| 모터 패킷 | 43B | `[ID:1B][CMD:1B][MODE:1B][10×float32]` |
| 센서 요청 | 3B | `[ID:1B][CMD:1B][MODE:1B]` (헤더만) |
| 센서 응답 | 67B | `[ID:1B][CMD:1B][MODE:1B][16×uint32]` → 유효 11개 (기압×8 + ToF×3) |

**명령 코드:**

| 명령 | 코드 | 방향 | 설명 |
|------|------|------|------|
| WritePosition | `0x01` | → | 10개 모터 목표 위치 전송 |
| SetSensorMode | `0x04` | → | 센서 모드 설정 (RAW/NN) |
| ReadPosition | `0x11` | ↔ | 현재 모터 위치 요청 |
| ReadVelocity | `0x12` | ↔ | 현재 모터 속도 요청 |
| ReadSensor0-3 | `0x14-0x17` | ↔ | 손가락별 센서 데이터 요청 (최대 4개) |

**실행 모드:**

| 모드 | 설명 |
|------|------|
| WriteOnly | 정현파 모터 명령 전송 (피드백 없음) |
| PollCycle | 전체 사이클: WritePos + ReadPos + ReadVel + ReadSensor×4 |
| StaticPose | 고정 모터 위치 전송 |
| ReadOnly | 쓰기 없음; ReadPos + ReadVel + ReadSensor×4만 수행 |

**CSV 로깅**: 타임스탬프, 모터 위치/속도, 센서별 기압/ToF 데이터 자동 기록 (v5.10.0: `RTC_SESSION_DIR/hand/` 기본 경로)

**장애 감지**: 연속 0-데이터 또는 중복 데이터 5회 초과 시 자동 종료

---

### `hand_data_plot.py` — 손 CSV 데이터 시각화

`hand_udp_sender_example.py`에서 저장한 CSV 로그를 Matplotlib으로 시각화합니다. 센서 수를 자동 감지하여 플롯합니다.

> **참고**: entry_point 미등록 — `python3`으로 직접 실행합니다.

```bash
# 모터 + 센서 전체 플롯
python3 rtc_tools/utils/hand_data_plot.py <csv_file>

# 특정 모터만 플롯
python3 rtc_tools/utils/hand_data_plot.py <csv_file> --motors 0 1 2

# 센서만 플롯
python3 rtc_tools/utils/hand_data_plot.py <csv_file> --sensors-only

# 모터만 플롯
python3 rtc_tools/utils/hand_data_plot.py <csv_file> --motors-only
```

**생성 플롯:**

| 플롯 | 내용 |
|------|------|
| 모터 위치 | pos_0..9 (10개 트레이스) vs 시간 |
| 모터 속도 | vel_0..9 (10개 트레이스) vs 시간 |
| 기압 센서 | 손가락별 (최대 4개), 기압 채널 8개씩 |
| ToF 센서 | 손가락별 (최대 4개), ToF 채널 3개씩 |

---

## 빌드

```bash
cd ~/ros2_ws/ur5e_ws
colcon build --packages-select rtc_tools --symlink-install
source install/setup.bash
```

**Python 의존성 설치:**
```bash
pip install matplotlib pandas numpy scipy
sudo apt install python3-pyqt5   # motion_editor_gui만 필요
```

---

## 세션 디렉토리 유틸리티 (`session_dir.py`, v5.10.0)

`RTC_SESSION_DIR` 환경변수를 통해 세션 디렉토리 경로를 관리하는 유틸리티입니다. 모든 패키지에서 세션 디렉토리 경로를 통일하기 위해 사용됩니다.

- `--save-dir` 미지정 시 `RTC_SESSION_DIR/plots/`에 자동 저장
- `UR5E_SESSION_DIR` 환경변수 폴백 지원

---

## 모니터링 기능 이전 (v5.9.0)

기존 `monitor_data_health` 기능은 `rtc_status_monitor` 및 `ur5e_hand_driver` 패키지에 통합되었습니다. 데이터 건강 모니터링은 해당 패키지를 참조하세요.

---

## 최적화 내역 (v5.16.1)

| 영역 | 변경 내용 |
|------|----------|
| **코드 검증** | Python 패키지 — 종속성 및 스크립트 구조 확인 완료 |

---

## 라이선스

MIT License
