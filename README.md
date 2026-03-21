# RTC (Real-Time Control) Framework

![CI](https://github.com/hyujun/ur5e-rt-controller/actions/workflows/ros2-advanced-ci.yml/badge.svg)
[![codecov](https://codecov.io/gh/hyujun/ur5e-rt-controller/branch/master/graph/badge.svg)](https://codecov.io/gh/hyujun/ur5e-rt-controller)
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-green)

**Ubuntu 22.04 (ROS 2 Humble) / Ubuntu 24.04 (ROS 2 Jazzy) | 모듈형 rtc_* 프레임워크 기반 UR5e 실시간 제어 시스템**

로봇 비의존적(robot-agnostic) RTC 프레임워크 위에 구축된 실시간 제어 솔루션입니다. 가변 DOF, 설정 가능한 제어 주기(500Hz–2kHz), 전략 패턴 기반 다중 제어기(P/JointPD/CLIK/OSC), 전송 계층 추상화(UDP/CAN-FD/EtherCAT/RS485), RT-안전 ONNX 추론 엔진, MuJoCo 3.x 물리 시뮬레이터, E-STOP 안전 시스템, CSV 데이터 로깅, GUI 도구를 포함합니다.

---

## 패키지 구성

16개 ROS2 패키지로 구성되어 있으며, 로봇 비의존적 프레임워크(`rtc_*`)와 로봇 고유 패키지(`ur5e_*`)로 분리됩니다. 각 패키지는 자체 `README.md`를 포함합니다.

### 로봇 비의존적 프레임워크 (rtc_*)

| 패키지 | 버전 | 설명 | 빌드 |
|--------|------|------|------|
| [`rtc_msgs`](rtc_msgs/) | 5.16.0 | 커스텀 ROS2 메시지 7종 (JointCommand, HandMotorCommand, HandMotorFeedback, FingertipSensors, FingertipForceTorque, SystemStatus, ControllerDiagnostics) | ament_cmake |
| [`rtc_base`](rtc_base/) | 5.16.0 | 헤더-전용 RT 인프라: 타입, SeqLock, SPSC 버퍼, 스레딩(4/6/8/10/12/16코어), Bessel/Kalman 필터, CSV 로깅 | ament_cmake |
| [`rtc_communication`](rtc_communication/) | 5.16.0 | 헤더-전용 전송 계층 추상화: TransportInterface, UdpSocket RAII, PacketCodec concept, Transceiver 템플릿 | ament_cmake |
| [`rtc_controller_interface`](rtc_controller_interface/) | 0.1.0 | 추상 컨트롤러 인터페이스 (Strategy 패턴) + Singleton 레지스트리 (가변 DOF) | ament_cmake |
| [`rtc_controllers`](rtc_controllers/) | 0.1.0 | 범용 제어기 4종 (P, JointPD, CLIK, OSC) + 퀸틱 궤적 생성기 | ament_cmake |
| [`rtc_controller_manager`](rtc_controller_manager/) | 0.1.0 | 500Hz RT 루프 (clock_nanosleep) + 컨트롤러 라이프사이클 + SPSC publish offload + E-STOP | ament_cmake |
| [`rtc_status_monitor`](rtc_status_monitor/) | 5.16.0 | 비-RT 10Hz 안전 감시 (로봇 모드, 추적 오차, 관절 한계) + lock-free RT 접근자 | ament_cmake |
| [`rtc_inference`](rtc_inference/) | 5.16.0 | 헤더-전용 RT-안전 추론 엔진: ONNX Runtime IoBinding, 사전 할당 버퍼, 배치/다중 모델 | ament_cmake |
| [`rtc_mujoco_sim`](rtc_mujoco_sim/) | 5.16.0 | MuJoCo 3.x 물리 시뮬레이터: FreeRun/SyncStep, GLFW 뷰어, fake_hand 1차 필터 | ament_cmake |
| [`rtc_digital_twin`](rtc_digital_twin/) | — | RViz2 디지털 트윈 시각화 | ament_python |
| [`rtc_tools`](rtc_tools/) | 5.16.0 | Python 유틸리티: controller_gui, plot_rtc_log, compare_mjcf_urdf, urdf_to_mjcf, hand_udp_sender | ament_python |
| [`rtc_scripts`](rtc_scripts/) | 5.16.0 | RT 시스템 설정 스크립트 (PREEMPT_RT 커널, CPU 격리, IRQ 어피니티, 네트워크 최적화) | ament_cmake |

### 로봇 고유 패키지 (ur5e_*)

| 패키지 | 버전 | 설명 | 빌드 |
|--------|------|------|------|
| [`ur5e_description`](ur5e_description/) | 5.16.0 | UR5e URDF/MJCF/메시 — Pinocchio/RViz/MuJoCo 겸용 | ament_cmake |
| [`ur5e_hand_driver`](ur5e_hand_driver/) | 5.16.0 | 10-DOF 핸드 UDP 드라이버: SeqLock 상태, ppoll sub-ms 타임아웃, 촉각 센서 44ch | ament_cmake |
| [`ur5e_bringup`](ur5e_bringup/) | 5.16.0 | UR5e launch/config + 데모 컨트롤러 (DemoJoint, DemoTask) + CPU 격리/DDS 핀닝 | ament_cmake |

### 의존성 그래프

```
rtc_msgs, rtc_base (독립)
  ├── rtc_communication ← rtc_base
  ├── rtc_inference ← rtc_base
  ├── rtc_controller_interface ← rtc_base, rtc_msgs
  │   └── rtc_controllers ← rtc_controller_interface, Pinocchio
  │       └── rtc_controller_manager ← rtc_controllers, rtc_communication, rtc_status_monitor
  ├── rtc_status_monitor ← rtc_base, rtc_msgs
  ├── rtc_mujoco_sim ← MuJoCo 3.x (optional)
  ├── rtc_digital_twin (독립, Python)
  ├── rtc_tools (독립, Python)
  └── rtc_scripts (독립, shell)

ur5e_description (독립)
  ├── ur5e_hand_driver ← rtc_communication, rtc_inference, rtc_base
  └── ur5e_bringup ← rtc_controller_manager, ur5e_hand_driver, ur5e_description
```

---

## 주요 기능

### RT 제어 코어
- **가변 DOF 실시간 제어**: `clock_nanosleep(TIMER_ABSTIME)` 기반 500Hz–2kHz RT 루프, CPU 코어 자동 할당 (4/6/8/10/12/16코어)
- **Lock-Free SPSC 아키텍처**: RT 스레드 → SPSC 버퍼 → 비-RT 퍼블리시/로깅 (wait-free push, cache-line 정렬)
- **SeqLock 동기화**: 단일 Writer / 다중 Reader lock-free 상태 공유 (trivially copyable 타입 전용)
- **컨트롤러 계층 분리**: `rtc_controller_interface` (추상) → `rtc_controllers` (범용 4종) → `ur5e_bringup` (데모 2종)

### 제어 알고리즘
- **PController**: Joint-space 비례 제어 (증분 스텝 `q + kp*error*dt`)
- **JointPDController**: PD + Pinocchio RNEA 중력/코리올리 보상, JointSpaceTrajectory 퀸틱 보간
- **ClikController**: Damped Jacobian 역운동학 (3/6-DOF), 영공간 제어, TaskSpaceTrajectory SE3 퀸틱
- **OperationalSpaceController**: 6-DOF Cartesian PD + SO(3) 회전 제어, Pinocchio log3 오차

### 안전 시스템
- **글로벌 E-STOP**: 로봇/핸드 타임아웃 + 상태 모니터 + 핸드 실패 감지 → 통합 비상 정지
- **상태 모니터**: 10Hz 비-RT 감시 (로봇 모드, 추적 오차, 관절 한계, watchdog)
- **Init 타임아웃**: 데이터 미수신 시 자동 셧다운

### 시뮬레이션 & 추론
- **MuJoCo 3.x 시뮬레이터**: FreeRun/SyncStep 모드, GLFW 인터랙티브 뷰어 (40+ 키보드 단축키), fake_hand 시뮬레이션
- **RT-안전 ONNX 추론**: IoBinding + 사전 할당 버퍼로 RT 경로 힙 할당 제거, 배치/다중 모델 지원

### 통신 & 로깅
- **전송 계층 추상화**: UDP 구현 완료, CAN-FD/EtherCAT/RS485 확장 가능
- **세션 기반 CSV 로깅**: `logging_data/YYMMDD_HHMM/` — timing, robot, device 3파일 분리
- **ROS2 파라미터 인트로스펙션**: 컨트롤러별 토픽 매핑을 읽기 전용 파라미터로 노출

### 도구
- **controller_gui**: tkinter 기반 실시간 게인 튜닝 GUI
- **plot_rtc_log**: Matplotlib CSV 궤적 시각화 (robot/device 자동 감지)
- **compare_mjcf_urdf**: MJCF/URDF 물리 파라미터 교차 검증
- **urdf_to_mjcf**: URDF → MJCF 자동 변환

---

## 빠른 시작

### 설치

```bash
chmod +x install.sh
./install.sh sim          # 시뮬레이션 전용
./install.sh robot        # 실제 로봇 전용
./install.sh full         # 전체 설치
```

### 빌드

```bash
chmod +x build.sh
./build.sh sim            # 시뮬레이션 패키지
./build.sh robot          # 로봇 패키지 (MuJoCo 제외)
./build.sh full           # 전체 패키지
./build.sh sim -c -j 4    # 클린 빌드 + 4 워커
./build.sh -p rtc_base    # 특정 패키지만 빌드

# 수동 빌드
cd ~/ur_ws && colcon build --symlink-install && source install/setup.bash
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
ros2 launch ur5e_hand_driver hand_udp.launch.py target_ip:=192.168.1.2
```

### 모니터링

```bash
ros2 topic hz /forward_position_controller/commands   # RT 루프 주기 (~500Hz)
ros2 topic echo /system/estop_status                  # E-STOP 상태 (true = 활성)
ros2 topic echo /sim/status                           # MuJoCo: [step, time, rtf, paused]
ros2 topic echo /rt_controller/trajectory_state       # 궤적 보간 상태
ros2 param list /rt_controller | grep controllers     # 토픽 파라미터 확인

# RT 스레드 상태 확인
PID=$(pgrep -f rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
```

---

## 아키텍처 개요

**ROS2 Humble / Jazzy** 멀티 패키지 리포지토리 (`ament_cmake`, C++20). `rtc_*` 패키지는 URDF 매니퓰레이터에 범용 적용 가능합니다.

```
[로봇 하드웨어 / MuJoCo 시뮬레이터]
    │  /joint_states (sensor_msgs/JointState)
    ▼
[rtc_controller_manager]  ←  /target_joint_positions (goal)
    │  RT 루프 (clock_nanosleep 500Hz)
    │  제어기: rtc_controllers (P / JointPD / CLIK / OSC)
    │  전송: rtc_communication (UDP)
    ├──→ SPSC ──→ [publish_thread] ──→ /forward_position_controller/commands
    ├──→ SPSC ──→ [log_thread] ──→ CSV 3-파일 (timing, robot, device)
    └──→ E-STOP ──→ /system/estop_status

[핸드 HW] ←UDP 직접 소유→ [ur5e_hand_driver] ← SeqLock ← [ControlLoop]

[rtc_inference]   RT-안전 ONNX 추론 (IoBinding, 사전 할당)
[rtc_status_monitor]  10Hz 비-RT 안전 감시 → 글로벌 E-STOP
```

### 스레딩 모델 (v5.16.0, 6코어 기준)

| 스레드 | 타입 | 코어 | 스케줄러 | 우선순위 | 역할 |
|--------|------|------|----------|----------|------|
| `rt_loop` | jthread (clock_nanosleep) | 2 | SCHED_FIFO | 90 | ControlLoop 500Hz + CheckTimeouts 50Hz |
| `sensor_executor` | ROS2 Executor | 3 | SCHED_FIFO | 70 | /joint_states, /target_joint_positions 구독 |
| `log_executor` | ROS2 Executor | 4 | SCHED_OTHER | nice -5 | CSV 3-파일 로깅 (SpscLogBuffer drain) |
| `publish_thread` | jthread (SPSC drain) | 5 | SCHED_OTHER | nice -3 | ROS2 publish offload (ControlPublishBuffer) |
| `aux_executor` | ROS2 Executor | 5 | SCHED_OTHER | 0 | E-STOP 상태 퍼블리시 |
| `udp_recv` | jthread | 5 | SCHED_FIFO | 65 | 핸드 UDP 수신 (ur5e_hand_driver) |
| `status_monitor` | jthread | 4 | SCHED_OTHER | nice -2 | 10Hz 상태 감시 |
| `hand_failure` | jthread | 4 | SCHED_OTHER | nice -2 | 50Hz 핸드 실패 감지 |

> Core 0–1: OS, DDS, NIC IRQ (isolcpus=2-5 권장). 8/10/12/16코어 레이아웃은 `rtc_base` README 참조.

---

## 세션 기반 로깅

로그는 `logging_data/YYMMDD_HHMM/` 세션 디렉토리에 자동 저장됩니다 (`max_log_sessions: 10`):

| 서브디렉토리 | 내용 |
|---|---|
| `controller/` | timing_log.csv (6열: 타이밍), robot_log.csv (49열: 관절 상태), device_log.csv (87열: 핸드/센서) |
| `monitor/` | ur5e_failure_*.log, controller_stats.json |
| `device/` | hand_udp_stats.json |
| `sim/` | screenshot_*.ppm (MuJoCo 전용) |
| `plots/`, `motions/` | rtc_tools 출력 |

환경변수 `RTC_SESSION_DIR` (또는 `UR5E_SESSION_DIR` 폴백)로 모든 노드에 세션 경로 자동 전파.

---

## RT 권한 요구사항

```bash
sudo groupadd realtime
sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# 로그아웃 후 재로그인 필요
```

최대 RT 성능을 위한 CPU 격리:
```bash
# /etc/default/grub의 GRUB_CMDLINE_LINUX_DEFAULT에 추가 (6코어 기준)
# isolcpus=2-5 nohz_full=2-5 rcu_nocbs=2-5
sudo update-grub && sudo reboot
```

---

## 문서

| 문서 | 설명 |
|------|------|
| [docs/RT_OPTIMIZATION.md](docs/RT_OPTIMIZATION.md) | 실시간 최적화 가이드 (CPU 코어 할당, 커널 설정) |
| [docs/VSCODE_DEBUGGING.md](docs/VSCODE_DEBUGGING.md) | VS Code + GDB 디버깅 가이드 |
| [docs/CLAUDE.md](docs/CLAUDE.md) | AI 어시스턴트 컨텍스트 문서 |
| [docs/SHELL_SCRIPTS.md](docs/SHELL_SCRIPTS.md) | RT 설정 쉘 스크립트 가이드 |

각 패키지의 상세 API, 설정, 아키텍처는 해당 패키지의 `README.md`를 참조하세요.

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.
