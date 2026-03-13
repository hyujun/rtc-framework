# UR5e RT Controller

![CI](https://github.com/hyujun/ur5e-rt-controller/actions/workflows/ros2-advanced-ci.yml/badge.svg)
[![codecov](https://codecov.io/gh/hyujun/ur5e-rt-controller/branch/master/graph/badge.svg)](https://codecov.io/gh/hyujun/ur5e-rt-controller)
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-green)

**Ubuntu 22.04 (ROS 2 Humble) / Ubuntu 24.04 (ROS 2 Jazzy) | 실시간 UR5e 제어기 + 커스텀 핸드 통합 (v5.7.0)**

E-STOP 안전 시스템, 전략 패턴 기반 다중 제어기(P/JointPD/CLIK/OSC/Hand), MuJoCo 3.x 물리 시뮬레이터, UDP 핸드 인터페이스, CSV 데이터 로깅, GUI 도구를 포함한 완전한 실시간 제어 솔루션입니다.

---

## 패키지 구성

6개 독립 ROS2 패키지로 구성되어 있으며, 각 패키지는 자체 `README.md`와 `CHANGELOG.md`를 포함합니다.

| 패키지 | 설명 | 빌드 시스템 |
|--------|------|-------------|
| [`ur5e_description`](ur5e_description/) | 로봇 모델 파일 (MJCF, URDF, 메시) | ament_cmake |
| [`ur5e_rt_base`](ur5e_rt_base/) | 공유 헤더-전용 라이브러리 (타입, 스레딩, 로깅, 필터) | ament_cmake |
| [`ur5e_rt_controller`](ur5e_rt_controller/) | 500Hz 실시간 제어기 — 위치/토크 (P/JointPD/CLIK/OSC/Hand) | ament_cmake |
| [`ur5e_hand_udp`](ur5e_hand_udp/) | 10-DOF 커스텀 핸드 UDP 브리지 (44 촉각 센서) | ament_cmake |
| [`ur5e_mujoco_sim`](ur5e_mujoco_sim/) | MuJoCo 3.x 물리 시뮬레이터 (선택적) | ament_cmake |
| [`ur5e_tools`](ur5e_tools/) | Python 개발 유틸리티 (GUI, 시각화, 검증) | ament_python |

### 의존성 그래프

```
ur5e_description   ← 독립 (모델 파일 제공)

ur5e_rt_base       ← 독립 (공유 기반, 헤더-전용)
    ↑
    ├── ur5e_rt_controller  ← ur5e_rt_base, ur5e_description
    └── ur5e_hand_udp       ← ur5e_rt_base

ur5e_mujoco_sim    ← ur5e_description (런타임 모델 참조)
ur5e_tools         ← 독립 (Python 전용, rclpy)
```

---

## 주요 기능

- **500Hz 실시간 제어**: SCHED_FIFO 멀티스레드 executor, CPU 코어 할당 (4/6/8코어 자동 선택)
- **전략 패턴 제어기**: PController, JointPDController, ClikController, OperationalSpaceController, UrFiveEHandController
- **런타임 컨트롤러 전환**: ROS2 토픽으로 제어기 간 즉시 전환 + 동적 게인 업데이트
- **E-STOP 안전 시스템**: 로봇/핸드 데이터 타임아웃 자동 감지 및 비상 정지
- **MuJoCo 시뮬레이터**: FreeRun/SyncStep 모드, GLFW 인터랙티브 뷰어, RTF 측정
- **UDP 핸드 통합**: 10-DOF 커스텀 핸드 (10 모터 + 44 촉각 센서) 요청-응답 프로토콜
- **잠금-없는 로깅**: SPSC 링 버퍼 기반 CSV 실시간 기록
- **RT-안전 신호 필터**: Bessel LPF + Kalman 필터 (N채널, noexcept)
- **GUI 도구**: Qt5 모션 편집기, tkinter 컨트롤러 GUI, Matplotlib 궤적 시각화

---

## 빠른 시작

### 설치

```bash
# 자동 설치
chmod +x install.sh
./install.sh sim          # 시뮬레이션 전용
./install.sh robot        # 실제 로봇 전용
./install.sh full         # 전체 설치
```

### 빌드

```bash
# build.sh 사용 (권장)
chmod +x build.sh
./build.sh sim            # 시뮬레이션 패키지
./build.sh robot          # 로봇 패키지 (MuJoCo 제외)
./build.sh full           # 전체 패키지

# 수동 빌드
cd ~/ur_ws
colcon build --symlink-install
source install/setup.bash
```

### 실행

```bash
# MuJoCo 시뮬레이션
ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py

# 실제 로봇
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=192.168.1.10

# 가상 하드웨어 (로봇 불필요)
ros2 launch ur5e_rt_controller ur_control.launch.py use_fake_hardware:=true

# UDP 핸드 노드 (통합, 권장)
ros2 launch ur5e_hand_udp hand_udp_unified.launch.py

# UDP 핸드 노드 (레거시 이중 노드)
ros2 launch ur5e_hand_udp hand_udp.launch.py
```

### 모니터링

```bash
ros2 topic hz /forward_position_controller/commands   # ~500Hz
ros2 topic echo /system/estop_status                  # E-STOP 상태
ros2 topic echo /sim/status                           # MuJoCo 시뮬 상태
```

---

## 아키텍처 개요

```
[UR5e 로봇 / MuJoCo 시뮬레이터]
    │  /joint_states
    ▼
[RtControllerNode]  ←  /target_joint_positions
    │  Strategy Pattern: P / JointPD / CLIK / OSC / Hand
    │  4 Executor 스레드 (RT Core 2 / Sensor Core 3 / Log Core 4 / Aux Core 5)
    ├──→ /forward_position_controller/commands → [UR 드라이버 / MuJoCo]  (indirect 컨트롤러)
    └──→ /forward_torque_controller/commands  → [UR 드라이버 / MuJoCo]  (direct 컨트롤러)

[10-DOF 핸드] ←UDP req/resp→ [HandUdpNode] ←ROS2→ [RtControllerNode]
```

각 패키지의 상세 아키텍처는 해당 패키지의 README.md를 참조하세요.

---

## RT 권한 요구사항

```bash
sudo groupadd realtime
sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# 로그아웃 후 재로그인 필요
```

---

## 문서

| 문서 | 설명 |
|------|------|
| [docs/RT_OPTIMIZATION.md](docs/RT_OPTIMIZATION.md) | 실시간 최적화 가이드 (CPU 코어 할당, 커널 설정) |
| [docs/VSCODE_DEBUGGING.md](docs/VSCODE_DEBUGGING.md) | VS Code + GDB 디버깅 가이드 |
| [docs/CLAUDE.md](docs/CLAUDE.md) | AI 어시스턴트 컨텍스트 문서 |
| [ur5e_rt_controller/docs/ADDING_CONTROLLER.md](ur5e_rt_controller/docs/ADDING_CONTROLLER.md) | 새 컨트롤러 추가 단계별 가이드 |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.
