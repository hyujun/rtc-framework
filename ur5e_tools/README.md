# ur5e_tools

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스 (v5.7.0)의 일부입니다.
> 설치/빌드: [Root README](../README.md)

UR5e RT Controller 스택의 **Python 개발 유틸리티 패키지**입니다. 컨트롤러 GUI, 모션 편집기, 데이터 건강 모니터링, 궤적 시각화, UDP 손 통신, 모델 검증 도구를 포함합니다.

## 개요

```
ur5e_tools/
├── ur5e_tools/
│   ├── gui/
│   │   ├── controller_gui.py            ← tkinter 다크 테마 컨트롤러 GUI (v5.3.0+)
│   │   └── motion_editor_gui.py         ← Qt5 모션 편집기 GUI (탭/Diff/타이밍)
│   ├── monitoring/
│   │   └── monitor_data_health.py       ← 데이터 건강 모니터 + 컨트롤러별 JSON 통계
│   ├── plotting/
│   │   └── plot_ur_trajectory.py        ← Matplotlib 궤적 시각화
│   ├── validation/
│   │   └── compare_mjcf_urdf.py         ← MJCF vs URDF 파라미터 비교 검증 (v5.7.0+)
│   └── utils/
│       ├── hand_udp_sender_example.py   ← 11-DOF 손 UDP 프로토콜 라이브러리 + 예제
│       └── hand_data_plot.py            ← 손 CSV 데이터 시각화
├── resource/
│   └── ur5e_tools
├── package.xml
├── setup.py
└── setup.cfg
```

**빌드 타입**: `ament_python` (`setup.py`의 `entry_points` 사용)

**Entry points** (6개):
| 실행 명령 | 모듈 |
|-----------|------|
| `ros2 run ur5e_tools controller_gui` | `gui.controller_gui` |
| `ros2 run ur5e_tools motion_editor_gui` | `gui.motion_editor_gui` |
| `ros2 run ur5e_tools monitor_data_health` | `monitoring.monitor_data_health` |
| `ros2 run ur5e_tools plot_ur_trajectory` | `plotting.plot_ur_trajectory` |
| `ros2 run ur5e_tools hand_udp_sender_example` | `utils.hand_udp_sender_example` |
| `ros2 run ur5e_tools compare_mjcf_urdf` | `validation.compare_mjcf_urdf` |

**Python 의존성**: `rclpy`, `numpy`, `matplotlib`, `pandas`, `scipy`, `PyQt5` (motion_editor_gui만 필요)

---

## 스크립트 설명

### `controller_gui.py` — 컨트롤러 GUI (v5.3.0+)

tkinter 기반 다크 테마 GUI. 컨트롤러 선택, 게인 튜닝, 관절/태스크 타겟 전송, E-STOP 상태 표시를 단일 창에서 수행합니다.

```bash
# tkinter는 Python 표준 라이브러리 — 별도 설치 불필요
ros2 run ur5e_tools controller_gui
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

### `motion_editor_gui.py` — 모션 편집기 GUI

Qt5 기반 모션 시퀀스 편집기입니다. 현재 관절 상태를 캡처하여 포즈를 시퀀스로 저장하고, 포즈별 궤적/대기 시간을 지정하여 순차 재생할 수 있습니다.

```bash
# PyQt5 필요
ros2 run ur5e_tools motion_editor_gui
```

**기능:**

| 기능 | 설명 |
|------|------|
| 탭 인터페이스 | 멀티 모션 지원; 탭별 독립 포즈 시퀀스 관리 |
| 테이블 레이아웃 | 7 컬럼: 체크박스, 이름, 상태(✅/Empty), 미리보기, Traj(s), Wait(s), 설명 |
| 포즈 저장 | 행 수 커스터마이징 (1-500, 기본 50) |
| 타이밍 제어 | 포즈별 Traj(s) 스핀박스 (0.1-60s, 기본 2.0s) + Wait(s) 스핀박스 (0.0-60s, 기본 0.0s) |
| 행 조작 | Save, Load, Insert Empty, Delete Selected |
| 재생 | 체크된 포즈 순차 재생; Traj + Wait 시간 기반 자동 진행 |
| Copy/Paste | 클립보드 기반 행 복제 (탭 간 이동 가능) |
| Select All / Deselect All | 저장된 포즈 일괄 선택/해제 |
| Diff 하이라이트 | 흰색=변경없음, 노랑=파일 대비 수정됨, 초록=신규 추가 |
| JSON 저장/불러오기 | 포즈, 이름, 설명, 타이밍 정보 포함 |
| 현재 상태 표시 | 실시간 관절 각도 (rad + deg) |

**ROS2 토픽:**

| 방향 | 토픽 | 타입 | 설명 |
|------|------|------|------|
| 구독 | `/joint_states` | `JointState` | 현재 관절 상태 캡처 |
| 발행 | `/target_joint_positions` | `Float64MultiArray` | 포즈 재생/수동 로드 시 전송 |

**전제 조건:**
```bash
sudo apt install python3-pyqt5
```

---

### `monitor_data_health.py` — 데이터 건강 모니터

`DataHealthMonitor` ROS2 노드. 5개 토픽의 패킷 속도와 타임아웃을 추적하며, 컨트롤러별 누적 통계를 JSON으로 저장합니다.

```bash
# 기본 실행 (10Hz 확인, 0.2s 타임아웃)
ros2 run ur5e_tools monitor_data_health

# 파라미터 지정
ros2 run ur5e_tools monitor_data_health \
    --ros-args -p check_rate:=10.0 -p timeout_threshold:=0.2
```

**모니터링 토픽:**

| 토픽 | 타입 | 기대 주기 | 설명 |
|------|------|----------|------|
| `/joint_states` | `JointState` | ~500Hz | UR 드라이버 상태 피드백 |
| `/target_joint_positions` | `Float64MultiArray` | 사용자 정의 | 타겟 위치 명령 |
| `/hand/joint_states` | `Float64MultiArray` | ~100Hz | 손 센서/위치 피드백 |
| `/forward_position_controller/commands` | `Float64MultiArray` | ~500Hz | 액추에이터 명령 스트림 |
| `/system/estop_status` | `Bool` | 이벤트 | E-STOP 상태 변경 |
| `/rt_controller/active_controller_name` | `String` | Latched | 현재 활성 컨트롤러 이름 (transient_local QoS) |

**파라미터:**

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `check_rate` | `10.0` | 확인 주기 (Hz) |
| `timeout_threshold` | `0.2` | 타임아웃 임계값 (초) |
| `stats_output_dir` | `~/ros2_ws/ur5e_ws/logging_data/stats/` | JSON 통계 저장 경로 (ament_index 경유 자동 탐색) |
| `enable_stats` | `true` | 통계 수집 활성화 |

**통계 출력:**
- 컨트롤러별 독립 JSON 파일: `logging_data/stats/<controller_name>/` 하위에 저장
- 항목: 총 활성 시간, robot/hand/command 패킷 수·손실률·평균 주파수, E-STOP 트리거 횟수
- 매 100 패킷마다 상태 요약 로그 출력
- 종료 시 최종 통계 저장

---

### `plot_ur_trajectory.py` — 궤적 시각화

CSV 제어 로그를 Matplotlib으로 시각화합니다. 관절별 위치, 목표, 명령값을 플롯합니다.

```bash
# 전체 관절 플롯
ros2 run ur5e_tools plot_ur_trajectory /tmp/ur5e_control_log.csv

# 특정 관절만 플롯
ros2 run ur5e_tools plot_ur_trajectory /tmp/ur5e_control_log.csv --joint 2

# 플롯 파일로 저장 (화면 표시 대신)
ros2 run ur5e_tools plot_ur_trajectory /tmp/ur5e_control_log.csv --save-dir /tmp/plots

# 통계만 출력 (플롯 없이)
ros2 run ur5e_tools plot_ur_trajectory /tmp/ur5e_control_log.csv --stats
```

**입력 CSV 형식** (DataLogger 출력):
```
timestamp, current_pos_0..5, target_pos_0..5, command_0..5, compute_time_us
```

**생성 플롯:**

| 플롯 | 내용 |
|------|------|
| 단일 관절 | 현재 위치, 목표 위치, 명령값 (3개 라인) |
| 전체 관절 (3×2 그리드) | 6개 서브플롯, 현재 vs 목표 위치 |
| 추적 오차 | error = target - current, y=0 기준선 |

**통계 출력**: 구간 길이(s), 샘플 수, 평균 샘플링 속도(Hz), 관절별 RMS 추적 오차(rad + deg)

---

### `compare_mjcf_urdf.py` — MJCF vs URDF 파라미터 비교 검증 (v5.7.0+)

`ur5e_description` 패키지의 `ur5e.xml` (MJCF)과 `ur5e.urdf` (URDF)를 파싱하여 물리 파라미터 동일성을 검증합니다.

```bash
# 자동 경로 탐색 (ament_index 또는 상대 경로)
ros2 run ur5e_tools compare_mjcf_urdf

# 수동 경로 지정
ros2 run ur5e_tools compare_mjcf_urdf \
    --mjcf /path/to/ur5e.xml --urdf /path/to/ur5e.urdf

# tolerance 조정 (기본: 1e-4)
ros2 run ur5e_tools compare_mjcf_urdf --tolerance 0.01
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

### `hand_udp_sender_example.py` — 11-DOF 손 UDP 프로토콜 라이브러리 + 예제

개발/테스트용 UDP 손 통신 라이브러리 및 합성 데이터 생성기입니다. `HandUDPSender` 클래스로 request-response 통신, CSV 로깅, 장애 감지를 지원합니다.

```bash
# 기본 실행 (PollCycle 모드)
ros2 run ur5e_tools hand_udp_sender_example

# 특정 IP/포트로 전송
ros2 run ur5e_tools hand_udp_sender_example \
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

**CSV 로깅**: 타임스탬프, 모터 위치/속도, 센서별 기압/ToF 데이터 자동 기록

**장애 감지**: 연속 0-데이터 또는 중복 데이터 5회 초과 시 자동 종료

---

### `hand_data_plot.py` — 손 CSV 데이터 시각화

`hand_udp_sender_example.py`에서 저장한 CSV 로그를 Matplotlib으로 시각화합니다. 센서 수를 자동 감지하여 플롯합니다.

> **참고**: entry_point 미등록 — `python3`으로 직접 실행합니다.

```bash
# 모터 + 센서 전체 플롯
python3 ur5e_tools/utils/hand_data_plot.py <csv_file>

# 특정 모터만 플롯
python3 ur5e_tools/utils/hand_data_plot.py <csv_file> --motors 0 1 2

# 센서만 플롯
python3 ur5e_tools/utils/hand_data_plot.py <csv_file> --sensors-only

# 모터만 플롯
python3 ur5e_tools/utils/hand_data_plot.py <csv_file> --motors-only
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
colcon build --packages-select ur5e_tools --symlink-install
source install/setup.bash
```

**Python 의존성 설치:**
```bash
pip install matplotlib pandas numpy scipy
sudo apt install python3-pyqt5   # motion_editor_gui만 필요
```

---

## launch 파일에서의 사용

`ur5e_rt_controller`와 `ur5e_mujoco_sim`의 launch 파일은 `monitor_data_health`를 이 패키지에서 실행합니다:

```python
# ur_control.launch.py, mujoco_sim.launch.py 내
monitor_node = Node(
    package='ur5e_tools',
    executable='monitor_data_health',
    name='data_health_monitor',
    parameters=[{'check_rate': 10.0, 'timeout_threshold': 0.2}]
)
```

---

## 라이선스

MIT License
