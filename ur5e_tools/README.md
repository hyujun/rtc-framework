# ur5e_tools

> **Note:** This package is part of the UR5e RT Controller workspace (v5.2.2). For full architecture details, installation instructions, and ROS 2 Jazzy compatibility, please refer to the [Root README](../README.md) and [Root CLAUDE.md](../CLAUDE.md).
UR5e RT Controller 스택의 **Python 개발 유틸리티 패키지**입니다. 궤적 시각화, 데이터 건강 모니터링, 모션 편집 GUI, UDP 손 데이터 생성기를 포함합니다.

## 개요

```
ur5e_tools/
├── scripts/
│   ├── plot_ur_trajectory.py        ← Matplotlib 궤적 시각화
│   ├── monitor_data_health.py       ← 데이터 건강 모니터 + JSON 통계 출력
│   ├── motion_editor_gui.py         ← Qt5 50-포즈 모션 편집기 GUI
│   └── hand_udp_sender_example.py   ← 합성 UDP 손 데이터 생성기
├── ur5e_tools/
│   └── __init__.py
├── resource/
│   └── ur5e_tools
├── package.xml
├── setup.py
└── setup.cfg
```

**빌드 타입**: `ament_cmake` (Python 스크립트를 `scripts/` 경유 설치, `setup.py` 포함)

**Python 의존성**: `rclpy`, `numpy`, `matplotlib`, `pandas`, `scipy`, `PyQt5`

---

## 스크립트 설명

### `plot_ur_trajectory.py` — 궤적 시각화

CSV 제어 로그를 Matplotlib으로 시각화합니다. 관절별 위치, 목표, 명령값을 플롯합니다.

```bash
# 전체 관절 플롯
ros2 run ur5e_tools plot_ur_trajectory.py /tmp/ur5e_control_log.csv

# 특정 관절만 플롯
ros2 run ur5e_tools plot_ur_trajectory.py /tmp/ur5e_control_log.csv --joint 2
```

**입력 CSV 형식** (DataLogger 출력):
```
timestamp, current_pos_0..5, target_pos_0..5, command_0..5, compute_time_us
```

---

### `monitor_data_health.py` — 데이터 건강 모니터

`DataHealthMonitor` ROS2 노드. 4개 토픽의 패킷 속도와 타임아웃을 추적하며, 종료 시 JSON 통계를 파일로 저장합니다.

```bash
# 기본 실행 (10Hz 확인, 0.2s 타임아웃)
ros2 run ur5e_tools monitor_data_health.py

# 파라미터 지정
ros2 run ur5e_tools monitor_data_health.py \
    --ros-args -p check_rate:=10.0 -p timeout_threshold:=0.2
```

**모니터링 토픽:**

| 토픽 | 기대 주기 |
|------|----------|
| `/joint_states` | ~500Hz (UR 드라이버) |
| `/target_joint_positions` | 사용자 정의 |
| `/hand/joint_states` | ~100Hz |
| `/forward_position_controller/commands` | ~500Hz |

**파라미터:**

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `check_rate` | `10.0` | 확인 주기 (Hz) |
| `timeout_threshold` | `0.2` | 타임아웃 임계값 (초) |
| `stats_output_dir` | `/tmp/ur5e_stats/` | JSON 통계 저장 경로 |
| `enable_stats` | `true` | 통계 수집 활성화 |

---

### `motion_editor_gui.py` — 모션 편집기 GUI

Qt5 기반 50-포즈 모션 편집기입니다. 현재 관절 상태를 캡처하고 포즈를 시퀀스로 저장하여 재생할 수 있습니다.

```bash
# 실행 (PyQt5 필요)
ros2 run ur5e_tools motion_editor_gui.py
```

**기능:**
- `/joint_states` 구독 → 현재 각도 표시
- `/target_joint_positions` 퍼블리시 → 포즈 실행
- 최대 50개 포즈 저장/편집
- JSON 파일로 저장/불러오기
- 순차 재생 (포즈 간 2초 딜레이)

**전제 조건:**
```bash
sudo apt install python3-pyqt5
```

---

### `hand_udp_sender_example.py` — UDP 손 데이터 생성기

개발/테스트용 합성 UDP 손 데이터 생성기입니다. 정현파 또는 정적 데이터를 `ur5e_hand_udp`의 수신 노드로 전송합니다.

```bash
# 정현파 패턴 전송 (기본)
ros2 run ur5e_tools hand_udp_sender_example.py

# 특정 IP/포트로 전송
ros2 run ur5e_tools hand_udp_sender_example.py \
    --target-ip 127.0.0.1 --target-port 50001
```

**전송 패킷**: 77개 `double` (616바이트) — 위치/속도/전류/센서 데이터

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select ur5e_tools --symlink-install
source install/setup.bash
```

**Python 의존성 설치:**
```bash
pip install matplotlib pandas numpy scipy
sudo apt install python3-pyqt5
```

---

## launch 파일에서의 사용

`ur5e_rt_controller`와 `ur5e_mujoco_sim`의 launch 파일은 `monitor_data_health.py`를 이 패키지에서 실행합니다:

```python
# ur_control.launch.py, mujoco_sim.launch.py 내
monitor_node = Node(
    package='ur5e_tools',
    executable='monitor_data_health.py',
    name='data_health_monitor',
    parameters=[{'check_rate': 10.0, 'timeout_threshold': 0.2}]
)
```

---

## 라이선스

MIT License
