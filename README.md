# UR5e RT Controller

![CI](https://github.com/hyujun/ur5e-rt-controller/actions/workflows/ros2-advanced-ci.yml/badge.svg)
[![codecov](https://codecov.io/gh/hyujun/ur5e-rt-controller/branch/master/graph/badge.svg)](https://codecov.io/gh/hyujun/ur5e-rt-controller)
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-green)

**Ubuntu 22.04 (ROS 2 Humble) / Ubuntu 24.04 (ROS 2 Jazzy) | 실시간 UR5e 제어기 + 커스텀 핸드 통합 (v5.13.0)**

E-STOP 안전 시스템, 전략 패턴 기반 다중 제어기(P/JointPD/CLIK/OSC/Hand), MuJoCo 3.x 물리 시뮬레이터, UDP 핸드 인터페이스, CSV 데이터 로깅, GUI 도구를 포함한 완전한 실시간 제어 솔루션입니다.

---

## 패키지 구성

7개 독립 ROS2 패키지로 구성되어 있으며, 각 패키지는 자체 `README.md`와 `CHANGELOG.md`를 포함합니다.

| 패키지 | 설명 | 빌드 시스템 |
|--------|------|-------------|
| [`ur5e_description`](ur5e_description/) | 로봇 모델 파일 (MJCF, URDF, 메시) | ament_cmake |
| [`ur5e_rt_base`](ur5e_rt_base/) | 공유 헤더-전용 라이브러리 (타입, 스레딩, 로깅, 필터) | ament_cmake |
| [`ur5e_status_monitor`](ur5e_status_monitor/) | 비-RT 상태 모니터 라이브러리 (10Hz 안전 감시) | ament_cmake |
| [`ur5e_rt_controller`](ur5e_rt_controller/) | 500Hz 실시간 제어기 — 위치/토크 (P/JointPD/CLIK/OSC/Hand) | ament_cmake |
| [`ur5e_hand_udp`](ur5e_hand_udp/) | 10-DOF 커스텀 핸드 UDP 브리지 (44 촉각 센서) | ament_cmake |
| [`ur5e_mujoco_sim`](ur5e_mujoco_sim/) | MuJoCo 3.x 물리 시뮬레이터 (선택적) | ament_cmake |
| [`ur5e_tools`](ur5e_tools/) | Python 개발 유틸리티 (GUI, 시각화, 검증) | ament_python |

### 의존성 그래프

```
ur5e_description      ← 독립 (모델 파일 제공)

ur5e_rt_base          ← 독립 (공유 기반, 헤더-전용)
    ↑
    ├── ur5e_status_monitor  ← ur5e_rt_base (비-RT 상태 모니터 라이브러리)
    ├── ur5e_rt_controller   ← ur5e_rt_base, ur5e_description, ur5e_status_monitor
    └── ur5e_hand_udp        ← ur5e_rt_base

ur5e_mujoco_sim       ← ur5e_description (런타임 모델 참조)
ur5e_tools            ← 독립 (Python 전용, rclpy)
```

---

## 주요 기능

- **500Hz 실시간 제어**: SCHED_FIFO 멀티스레드 executor, CPU 코어 할당 (4/6/8코어 자동 선택, 7스레드)
- **전략 패턴 제어기**: PController, JointPDController, ClikController, OperationalSpaceController, UrFiveEHandController
- **런타임 컨트롤러 전환**: ROS2 토픽으로 제어기 간 즉시 전환 + 동적 게인 업데이트
- **글로벌 E-STOP 안전 시스템**: 로봇/핸드 타임아웃, 상태 모니터, 핸드 실패 감지 → 통합 비상 정지 (v5.8.0)
- **상태 모니터**: 10Hz 비-RT 안전 감시 (로봇 모드, 추적 오차, 관절 한계) + 실패 시 글로벌 E-Stop (v5.8.0)
- **핸드 실패 감지기**: 50Hz C++ 모니터 (영점/중복 데이터 감지) + E-Stop 트리거 (v5.8.0)
- **초기화 타임아웃**: 설정 가능한 시간 내 데이터 미수신 시 E-Stop + 종료 (v5.8.0)
- **MuJoCo 시뮬레이터**: FreeRun/SyncStep 모드, GLFW 인터랙티브 뷰어, RTF 측정
- **UDP 핸드 통합**: 10-DOF 커스텀 핸드 (10 모터 + 44 촉각 센서) event-driven 프로토콜, sensor decimation + busy skip 보호 (v5.11.0)
- **세션 기반 통합 로깅**: `logging_data/YYMMDD_HHMM/` 세션 디렉토리에 모든 패키지 로그 통합 — controller/, monitor/, hand/, sim/, plots/, motions/ 서브디렉토리 (v5.10.0)
- **RT-안전 신호 필터**: Bessel LPF + Kalman 필터 (N채널, noexcept)
- **통신 통계 모니터링**: Status Monitor MessageStats + 컨트롤러별 통계, Hand UDP CommStats + rate 모니터링 (v5.9.0)
- **궤적 시각화 v3**: 4-카테고리 CSV 지원, command/torque/task-pos 신규 Figure, `--all` 플래그, 레거시 호환 (v5.12.0)
- **ROS2 파라미터 토픽 인트로스펙션**: 컨트롤러별 토픽 매핑을 읽기 전용 ROS2 파라미터로 노출 — `ros2 param list/get`으로 확인 가능, `--ros-args -r`로 토픽 이름 변경 지원 (v5.13.0)
- **GUI 도구**: Qt5 모션 편집기, tkinter 컨트롤러 GUI, Matplotlib 궤적 시각화
- **세션 디렉토리 전파**: `UR5E_SESSION_DIR` 환경변수로 모든 노드에 세션 경로 자동 전파 (v5.10.0)

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

# UDP 핸드 노드
ros2 launch ur5e_hand_udp hand_udp.launch.py
```

### 모니터링

```bash
ros2 topic hz /forward_position_controller/commands   # ~500Hz
ros2 topic echo /system/estop_status                  # E-STOP 상태
ros2 topic echo /sim/status                           # MuJoCo 시뮬 상태
ros2 topic echo /rt_controller/trajectory_state   # 궤적 보간 상태 (18값)
ros2 topic echo /rt_controller/controller_state    # 제어기 상태 (18값)
ros2 param list /rt_controller | grep controllers   # 토픽 파라미터 확인
ros2 param get /rt_controller controllers.PController.subscribe.goal  # 토픽 이름 조회
```

---

## 아키텍처 개요

```
[UR5e 로봇 / MuJoCo 시뮬레이터]
    │  /joint_states
    ▼
[RtControllerNode]  ←  /target_joint_positions (goal)
    │  Strategy Pattern: P / JointPD / CLIK / OSC / Hand
    │  4 Executor 스레드 (RT Core 2 / Sensor Core 3 / Log Core 4 / Aux Core 5)
    ├──→ /forward_position_controller/commands → [UR 드라이버 / MuJoCo]  (indirect 컨트롤러)
    ├──→ /forward_torque_controller/commands  → [UR 드라이버 / MuJoCo]  (direct 컨트롤러)
    ├──→ /rt_controller/trajectory_state            → [모니터링/rqt_plot]
    └──→ /rt_controller/controller_state            → [모니터링/rqt_plot]

[10-DOF 핸드] ←UDP event-driven→ [HandController] ←직접 소유→ [RtControllerNode]
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
