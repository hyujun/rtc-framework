# UR5e RT Controller

![CI](https://github.com/hyujun/ur5e-rt-controller/actions/workflows/ros2-advanced-ci.yml/badge.svg)
[![codecov](https://codecov.io/gh/hyujun/ur5e-rt-controller/branch/master/graph/badge.svg)](https://codecov.io/gh/hyujun/ur5e-rt-controller)
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-green)

**Ubuntu 22.04 (ROS 2 Humble) / Ubuntu 24.04 (ROS 2 Jazzy) | RTC (Real-Time Controller) 모듈형 rtc_* 프레임워크 기반 UR5e 실시간 제어 시스템**

로봇 비의존적(robot-agnostic) RTC 프레임워크 위에 구축된 실시간 제어 솔루션입니다. 가변 DOF, 설정 가능한 제어 주기(500Hz-2kHz), 전략 패턴 기반 다중 제어기(P/JointPD/CLIK/OSC), 전송 계층 추상화(UDP/CAN-FD/EtherCAT/RS485), RT-안전 ONNX 추론 엔진, MuJoCo 3.x 물리 시뮬레이터, E-STOP 안전 시스템, CSV 데이터 로깅, GUI 도구를 포함합니다.

---

## 패키지 구성

16개 ROS2 패키지로 구성되어 있으며, 로봇 비의존적 프레임워크(`rtc_*`)와 로봇 고유 패키지(`ur5e_*`)로 분리됩니다. 각 패키지는 자체 `README.md`와 `CHANGELOG.md`를 포함합니다.

### 로봇 비의존적 프레임워크 (rtc_*)

| 패키지 | 설명 | 빌드 시스템 |
|--------|------|-------------|
| [`rtc_msgs`](rtc_msgs/) | 커스텀 ROS2 메시지 정의 (7 types) | ament_cmake |
| [`rtc_base`](rtc_base/) | 헤더-전용 RT 인프라 (타입, 스레딩, 로깅, 필터) | ament_cmake |
| [`rtc_communication`](rtc_communication/) | 전송 계층 추상화 (UDP, 계획: CAN-FD, EtherCAT, RS485) | ament_cmake |
| [`rtc_controller_interface`](rtc_controller_interface/) | 추상 컨트롤러 인터페이스 (가변 DOF) | ament_cmake |
| [`rtc_controllers`](rtc_controllers/) | 범용 매니퓰레이터 제어기 (P, JointPD, CLIK, OSC) | ament_cmake |
| [`rtc_controller_manager`](rtc_controller_manager/) | RT 루프 + 컨트롤러 라이프사이클 관리 | ament_cmake |
| [`rtc_status_monitor`](rtc_status_monitor/) | 비-RT 안전 감시 모니터 (10Hz) | ament_cmake |
| [`rtc_inference`](rtc_inference/) | RT-안전 추론 엔진 (ONNX Runtime) | ament_cmake |
| [`rtc_mujoco_sim`](rtc_mujoco_sim/) | MuJoCo 3.x 물리 시뮬레이터 | ament_cmake |
| [`rtc_digital_twin`](rtc_digital_twin/) | RViz2 시각화 | ament_python |
| [`rtc_tools`](rtc_tools/) | Python 개발 유틸리티 (GUI, 플로팅, 검증) | ament_python |
| [`rtc_scripts`](rtc_scripts/) | RT 시스템 설정 스크립트 (커널, CPU, IRQ, 네트워크) | ament_cmake |

### 로봇 고유 패키지 (ur5e_*)

| 패키지 | 설명 | 빌드 시스템 |
|--------|------|-------------|
| [`ur5e_description`](ur5e_description/) | UR5e URDF/MJCF/메시 | ament_cmake |
| [`ur5e_hand_driver`](ur5e_hand_driver/) | 10-DOF 커스텀 핸드 드라이버 (촉각 센서) | ament_cmake |
| [`ur5e_bringup`](ur5e_bringup/) | UR5e launch/config + 데모 컨트롤러 | ament_cmake |

### 의존성 그래프

```
rtc_msgs, rtc_base (독립)
  ├── rtc_communication ← rtc_base
  ├── rtc_inference ← rtc_base
  ├── rtc_controller_interface ← rtc_base, rtc_msgs
  │   └── rtc_controllers ← rtc_controller_interface
  │       └── rtc_controller_manager ← rtc_controllers, rtc_communication, rtc_status_monitor
  ├── rtc_status_monitor ← rtc_base, rtc_msgs
  ├── rtc_mujoco_sim (독립)
  ├── rtc_digital_twin (독립, Python)
  ├── rtc_tools (독립, Python)
  └── rtc_scripts (독립, shell)

ur5e_description (독립)
  ├── ur5e_hand_driver ← rtc_communication, rtc_inference, rtc_base
  └── ur5e_bringup ← rtc_controller_manager, ur5e_hand_driver, ur5e_description
```

---

## 주요 기능

- **가변 DOF 실시간 제어**: 설정 가능한 제어 주기(500Hz-2kHz), clock_nanosleep RT 루프 + SPSC publish offload, CPU 코어 할당 (4/6/8코어 자동 선택)
- **컨트롤러 계층 분리**: `rtc_controller_interface`(추상 인터페이스) → `rtc_controllers`(범용 제어기) → `ur5e_bringup`(로봇 전용 데모 제어기)
- **전략 패턴 제어기**: 범용 4종 (PController, JointPDController, ClikController, OperationalSpaceController) + 데모 2종 (DemoJoint, DemoTask in ur5e_bringup)
- **전송 계층 추상화**: UDP/CAN-FD/EtherCAT/RS485 통합 인터페이스 (`rtc_communication`)
- **RT-안전 추론 엔진**: ONNX Runtime 기반 실시간 안전 추론 (`rtc_inference`)
- **런타임 컨트롤러 전환**: ROS2 토픽으로 제어기 간 즉시 전환 + 동적 게인 업데이트
- **글로벌 E-STOP 안전 시스템**: 로봇/핸드 타임아웃, 상태 모니터, 핸드 실패 감지 → 통합 비상 정지
- **상태 모니터**: 10Hz 비-RT 안전 감시 (로봇 모드, 추적 오차, 관절 한계) + 실패 시 글로벌 E-Stop
- **MuJoCo 시뮬레이터**: FreeRun/SyncStep 모드, GLFW 인터랙티브 뷰어, RTF 측정
- **세션 기반 통합 로깅**: `logging_data/YYMMDD_HHMM/` 세션 디렉토리에 모든 패키지 로그 통합 — controller/, monitor/, device_log/, sim/, plots/, motions/ 서브디렉토리
- **RT-안전 신호 필터**: Bessel LPF + Kalman 필터 (N채널, noexcept)
- **궤적 시각화**: CSV 지원, command/torque/task-pos Figure, 레거시 호환
- **ROS2 파라미터 토픽 인트로스펙션**: 컨트롤러별 토픽 매핑을 읽기 전용 ROS2 파라미터로 노출
- **GUI 도구**: Qt5 모션 편집기, tkinter 컨트롤러 GUI, Matplotlib 궤적 시각화
- **세션 디렉토리 전파**: `RTC_SESSION_DIR` 환경변수로 모든 노드에 세션 경로 자동 전파 (`UR5E_SESSION_DIR` 폴백 지원)
- **Lock-Free SPSC 아키텍처**: RT 스레드에서 로깅/퍼블리시를 SPSC 버퍼로 오프로드하여 블로킹 없는 500Hz 루프 보장

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
# build.sh 사용 (권장, MuJoCo 경로 및 병렬성 자동 처리)
chmod +x build.sh
./build.sh sim            # 시뮬레이션 패키지
./build.sh robot          # 로봇 패키지 (MuJoCo 제외)
./build.sh full           # 전체 패키지
./build.sh sim -c -j 4    # 클린 빌드 + 4 워커
./build.sh -p rtc_base    # 특정 패키지만 빌드

# 수동 빌드
cd ~/ur_ws
colcon build --symlink-install
source install/setup.bash
```

### 실행

```bash
# MuJoCo 시뮬레이션
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py

# 실제 로봇
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ur5e_bringup robot.launch.py robot_ip:=192.168.1.10

# 가상 하드웨어 (로봇 불필요)
ros2 launch ur5e_bringup robot.launch.py use_fake_hardware:=true

# 핸드 드라이버 노드
ros2 launch ur5e_hand_driver hand_driver.launch.py
```

### 모니터링

```bash
ros2 topic hz /forward_position_controller/commands   # RT 루프 주기 (~500Hz)
ros2 topic echo /system/estop_status                  # E-STOP 상태 (true = 활성)
ros2 topic echo /sim/status                           # MuJoCo 시뮬 상태
ros2 topic echo /rt_controller/trajectory_state       # 궤적 보간 상태
ros2 topic echo /rt_controller/controller_state       # 제어기 상태
ros2 param list /rt_controller | grep controllers     # 토픽 파라미터 확인
ros2 control list_controllers -v                      # ros2_control 컨트롤러 목록

# RT 스레드 상태 확인
PID=$(pgrep -f rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID

# 수동 타겟 포즈 전송
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \
    "data: [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]"
```

---

## 아키텍처 개요

**ROS2 Humble / Jazzy** 멀티 패키지 리포지토리 (`ament_cmake`, C++20). 로봇 비의존적 실시간 제어기 프레임워크(RTC)로, `rtc_*` 패키지는 URDF 매니퓰레이터에 범용 적용 가능합니다.

```
[로봇 하드웨어 / MuJoCo 시뮬레이터]
    │  /joint_states
    ▼
[rtc_controller_manager]  ←  /target_joint_positions (goal)
    │  RT 루프 (clock_nanosleep 500Hz) + 컨트롤러 라이프사이클
    │  제어기: rtc_controllers (P / JointPD / CLIK / OSC)
    │  전송: rtc_communication (UDP / CAN-FD / EtherCAT / RS485)
    ├──→ /forward_position_controller/commands → [UR 드라이버 / MuJoCo]  (indirect 컨트롤러)
    ├──→ /forward_torque_controller/commands  → [UR 드라이버 / MuJoCo]  (direct 컨트롤러)
    ├──→ /rt_controller/trajectory_state      → [모니터링/rqt_plot]
    └──→ /rt_controller/controller_state      → [모니터링/rqt_plot]

[핸드 HW] ←UDP (직접 소유)→ [ur5e_hand_driver] ←rtc_controller_manager 직접 소유→ [ControlLoop]

[ur5e_bringup]  ← UR5e 전용 launch, config, 데모 컨트롤러 (DemoJoint, DemoTask)
[rtc_inference]  ← RT-안전 ONNX 추론 엔진
[rtc_digital_twin]  ← RViz2 시각화
```

### 스레딩 모델 (v5.16.0)

| 스레드 / Executor | 타입 | CPU 코어 | 스케줄러 | 우선순위 | 역할 |
|---|---|---|---|---|---|
| `rt_loop` | `std::jthread` (clock_nanosleep) | Core 2 | SCHED_FIFO | 90 | `ControlLoop()` (500Hz) + `CheckTimeouts()` (50Hz) |
| `publish_thread` | `std::jthread` (SPSC drain) | Core 5 | SCHED_OTHER | nice -3 | 모든 `publish()` 호출 (SPSC 버퍼 경유) |
| `sensor_executor` | ROS2 Executor | Core 3 | SCHED_FIFO | 70 | `/joint_states`, `/target_joint_positions` 구독 |
| `log_executor` | ROS2 Executor | Core 4 | SCHED_OTHER | nice -5 | CSV 3-파일 로깅 (SpscLogBuffer drain) |
| `aux_executor` | ROS2 Executor | Core 5 | SCHED_OTHER | 0 | E-STOP 상태 퍼블리시 |

각 패키지의 상세 아키텍처는 해당 패키지의 README.md를 참조하세요.

---

## 세션 기반 로깅

로그 출력은 `~/ros2_ws/ur5e_ws/logging_data/YYMMDD_HHMM/` 세션 디렉토리에 자동 저장됩니다 (`max_log_sessions: 10`):

| 서브디렉토리 | 내용 |
|---|---|
| `controller/` | timing_log.csv, robot_log.csv, device_log.csv |
| `monitor/` | ur5e_failure_*.log, controller_stats.json |
| `device/` | hand_udp_stats.json |
| `sim/` | screenshot_*.ppm (MuJoCo 전용) |
| `plots/`, `motions/` | rtc_tools 출력 |

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
| [docs/SHELL_SCRIPTS.md](docs/SHELL_SCRIPTS.md) | RT 설정 쉘 스크립트 가이드 (커널 빌드, CPU 격리, IRQ, 네트워크) |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.
