# UR5e RT Controller

**Ubuntu 22.04 + ROS2 Humble | 실시간 UR5e 제어기 + 커스텀 핸드 통합 (v4.0.0)**

E-STOP 안전 시스템, PD 제어기, UDP 핸드 인터페이스, CSV 데이터 로깅, Qt GUI 모션 에디터를 포함한 완전한 실시간 제어 솔루션입니다.

---

## 목차

- [기능 요약](#기능-요약)
- [프로젝트 구조](#프로젝트-구조)
- [아키텍처 개요](#아키텍처-개요)
- [설치 방법](#설치-방법)
- [사용 방법](#사용-방법)
- [설정 (YAML)](#설정-yaml)
- [ROS2 토픽 인터페이스](#ros2-토픽-인터페이스)
- [UDP 핸드 프로토콜](#udp-핸드-프로토콜)
- [성능 지표](#성능-지표)
- [문제 해결](#문제-해결)
- [모니터링](#모니터링)
- [고급 사용법](#고급-사용법)

---

## 기능 요약

| 기능 | 설명 |
|------|------|
| 실시간 제어 | 500Hz PD 제어 루프 (`custom_controller`) |
| E-STOP 시스템 | 로봇/핸드 데이터 타임아웃 자동 감지 및 비상 정지 |
| 커스텀 핸드 통합 | UDP 기반 4-DOF 핸드 데이터 수신/송신 |
| 데이터 로깅 | CSV 형식의 제어 데이터 실시간 기록 (`DataLogger`) |
| 데이터 시각화 | Matplotlib 기반 관절 궤적 플롯 (`plot_ur_trajectory.py`) |
| 데이터 헬스 모니터 | 패킷 손실/타임아웃 통계 수집 및 JSON 내보내기 |
| Qt GUI 에디터 | 50개 포즈 저장/로드/재생 모션 에디터 |
| Strategy Pattern | `RTControllerInterface`를 상속하는 교체 가능한 제어기 구조 |

---

## 프로젝트 구조

```
ur5e_rt_controller/
├── CMakeLists.txt                         # 빌드 설정 (C++17, ament_cmake)
├── package.xml                            # ROS2 패키지 메타데이터 (v4.0.0)
├── install.sh                             # 자동 설치 스크립트
├── organize_files.sh                      # 파일 정리 스크립트
├── setup.py                               # Python 패키지 설정
├── requirements.txt                       # Python 의존성 목록
│
├── config/                                # YAML 설정 파일
│   ├── ur5e_rt_controller.yaml            # 제어기 파라미터 + E-STOP 설정
│   └── hand_udp_receiver.yaml             # UDP 핸드 수신기 설정
│
├── launch/                                # ROS2 런치 파일
│   ├── ur_control.launch.py               # 전체 시스템 실행 (UR 드라이버 + 제어기)
│   └── hand_udp.launch.py                 # UDP 핸드 노드 단독 실행
│
├── include/ur5e_rt_controller/            # C++ 헤더 파일
│   ├── rt_controller_interface.hpp        # 제어기 기반 인터페이스 (Strategy Pattern)
│   ├── data_logger.hpp                    # CSV 데이터 로거
│   ├── hand_udp_receiver.hpp              # UDP 핸드 수신기 클래스
│   ├── hand_udp_sender.hpp                # UDP 핸드 송신기 클래스
│   └── controllers/                       # 제어기 구현체
│       ├── p_controller.hpp               # P 제어기 (비례 제어)
│       └── pd_controller.hpp              # PD 제어기 + E-STOP 지원
│
├── src/                                   # C++ 소스 파일
│   ├── custom_controller.cpp              # 메인 제어 노드 (500Hz, E-STOP)
│   ├── hand_udp_receiver_node.cpp         # 핸드 UDP 수신 ROS2 노드
│   └── hand_udp_sender_node.cpp           # 핸드 UDP 송신 ROS2 노드
│
├── scripts/                               # Python 스크립트
│   ├── monitor_data_health.py             # 데이터 헬스 모니터 + 통계
│   ├── plot_ur_trajectory.py              # CSV 로그 시각화
│   ├── motion_editor_gui.py               # Qt5 50포즈 모션 에디터
│   └── hand_udp_sender_example.py         # UDP 핸드 송신 예제
│
├── docs/                                  # 문서 (추가 예정)
├── rviz/                                  # RViz 설정 파일 (추가 예정)
├── test/                                  # 테스트 코드 (추가 예정)
└── resources/                             # 리소스 파일 (추가 예정)
```

---

## 아키텍처 개요

### 제어 흐름

```
[UR5e 로봇]
    │  /joint_states (sensor_msgs/JointState)
    ▼
[custom_controller]  ←──  /target_joint_positions (std_msgs/Float64MultiArray)
    │  PDController::compute_command()
    │  DataLogger::log_control_data()
    │  E-STOP 감시 (check_timeouts @ 50Hz)
    ▼
/forward_position_controller/commands ──► [UR 드라이버]
    │
    ▼
/system/estop_status (std_msgs/Bool) ──► [모니터링]

[커스텀 핸드]
    │  UDP 패킷 (포트 50001)
    ▼
[hand_udp_receiver_node]
    │  /hand/joint_states (std_msgs/Float64MultiArray)
    ▼
[custom_controller]  ──►  E-STOP 핸드 감시

[hand_udp_sender_node]
    │  /hand/command 구독
    ▼
[커스텀 핸드]  ◄── UDP 패킷 (포트 50002)
```

### 주요 클래스

#### `CustomController` (`src/custom_controller.cpp`)
500Hz 제어 루프를 실행하는 메인 ROS2 노드.

| 멤버 | 타입 | 역할 |
|------|------|------|
| `controller_` | `PDController` | PD 제어 계산 |
| `logger_` | `DataLogger` | CSV 로깅 |
| `control_timer_` | `rclcpp::TimerBase` | 500Hz 제어 루프 |
| `timeout_timer_` | `rclcpp::TimerBase` | 50Hz E-STOP 감시 |

파라미터:

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `control_rate` | `500.0` | 제어 주파수 (Hz) |
| `kp` | `5.0` | P 게인 |
| `kd` | `0.5` | D 게인 |
| `enable_logging` | `true` | CSV 로깅 활성화 |
| `robot_timeout_ms` | `100.0` | 로봇 데이터 타임아웃 (ms) |
| `hand_timeout_ms` | `200.0` | 핸드 데이터 타임아웃 (ms) |
| `enable_estop` | `true` | E-STOP 활성화 |

#### `RTControllerInterface` (`include/ur5e_rt_controller/rt_controller_interface.hpp`)
제어기 Strategy Pattern의 추상 기반 클래스. 6-DOF 로봇 + 11-DOF 핸드 통합 상태 관리.

```cpp
namespace ur5e_rt_controller {
  struct RobotState { Eigen::VectorXd q{6}, qd{6}; Eigen::Vector3d tcp_pos; };
  struct HandState  { Eigen::VectorXd motor_pos{11}, motor_vel{11}, motor_current{11}, sensor_data{44}; };
  struct ControllerOutput { Eigen::VectorXd robot_cmd{6}, hand_cmd{11}; };

  class RTControllerInterface {
    virtual ControllerOutput compute(const ControllerState& state) noexcept = 0;
  };
}
```

#### `PDController` (`include/ur5e_rt_controller/controllers/pd_controller.hpp`)
비례-미분 제어기. E-STOP 발생 시 안전 위치 `[0, -1.57, 1.57, -1.57, -1.57, 0]`로 이동.

#### `DataLogger` (`include/ur5e_rt_controller/data_logger.hpp`)
이동 불가 복사 비허용 CSV 로거. 타임스탬프, 현재/목표 위치, 명령값 기록.

---

## 설치 방법

### 1. 사전 요구사항

- Ubuntu 22.04 LTS
- ROS2 Humble
- (권장) LowLatency 또는 PREEMPT_RT 커널

```bash
# LowLatency 커널 설치 (실시간 성능 향상)
sudo apt install linux-lowlatency-hwe-22.04
sudo reboot

# 확인
uname -v  # "lowlatency" 또는 "PREEMPT_RT" 확인
```

### 2. ROS2 Humble 설치

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 3. 자동 설치 (권장)

```bash
chmod +x install.sh
./install.sh
```

### 4. 수동 설치

```bash
# C++ 의존성
sudo apt install -y \
    ros-humble-ur-robot-driver \
    ros-humble-ur-msgs \
    ros-humble-ur-description \
    ros-humble-control-msgs \
    ros-humble-industrial-msgs \
    ros-humble-ament-cmake \
    python3-colcon-common-extensions

# Python 의존성
pip3 install --user matplotlib pandas numpy scipy

# RT 권한 설정 (선택)
sudo groupadd realtime
sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
```

### 5. 빌드

```bash
mkdir -p ~/ur_ws/src
cd ~/ur_ws/src
# 이 저장소를 클론 또는 복사

cd ~/ur_ws
colcon build --packages-select ur5e_rt_controller --symlink-install
source install/setup.bash

# 환경변수 영구 추가
echo "source ~/ur_ws/install/setup.bash" >> ~/.bashrc
```

---

## 사용 방법

### 전체 시스템 실행

```bash
# 환경 설정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# UR5e 로봇 IP 확인 후 실행 (Teach Pendant → Settings → Network)
ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=192.168.1.10
```

런치 파일이 시작하는 노드:
1. `ur_robot_driver` - UR5e 드라이버 (ur_type: ur5e)
2. `custom_controller` - 500Hz PD 제어 노드 + E-STOP
3. `data_health_monitor` - 데이터 헬스 모니터 (10Hz)

시뮬레이션(fake hardware) 테스트:
```bash
ros2 launch ur5e_rt_controller ur_control.launch.py use_fake_hardware:=true
```

### UDP 핸드 노드만 실행

```bash
ros2 launch ur5e_rt_controller hand_udp.launch.py \
    udp_port:=50001 \
    target_ip:=192.168.1.100 \
    target_port:=50002
```

### Qt GUI 모션 에디터

```bash
# PyQt5 설치 (없는 경우)
sudo apt install python3-pyqt5

# GUI 실행
python3 scripts/motion_editor_gui.py
```

GUI 사용법:
1. **관절 각도 확인**: 상단 패널에서 J1~J6 실시간 표시
2. **포즈 저장**: 테이블에서 행 선택 → "Save Current Pose" 클릭
3. **포즈 로드**: 저장된 행 선택 → "Load Selected Pose" 클릭 → 로봇 이동
4. **모션 재생**: 여러 행 선택 (Ctrl+클릭) → "Play Motion Sequence" 클릭 (2초 간격)
5. **파일 저장**: File → Save Motion to JSON (50개 포즈 JSON 백업)
6. **파일 로드**: File → Load Motion from JSON

### 데이터 시각화

```bash
# 모든 관절 플롯
python3 scripts/plot_ur_trajectory.py /tmp/ur5e_control_log.csv

# 특정 관절만 (0~5)
python3 scripts/plot_ur_trajectory.py /tmp/ur5e_control_log.csv --joint 2

# 이미지 파일로 저장
python3 scripts/plot_ur_trajectory.py /tmp/ur5e_control_log.csv --save-dir ~/ur_plots

# 통계만 출력
python3 scripts/plot_ur_trajectory.py /tmp/ur5e_control_log.csv --stats
```

### 핸드 UDP 테스트 (예제)

```bash
# 사인파 테스트 데이터 전송 (500Hz)
python3 scripts/hand_udp_sender_example.py
# → 1) 사인파 (동적) / 2) 고정 포즈 (정적) 선택
```

---

## 설정 (YAML)

### `config/ur5e_rt_controller.yaml`

```yaml
controller:
  control_rate: 500.0        # 제어 주파수 (Hz)
  kp: 5.0                    # P 게인
  kd: 0.5                    # D 게인
  enable_logging: true        # CSV 로깅 활성화
  log_path: "/tmp/ur5e_control_log.csv"

joint_limits:
  max_velocity: 2.0          # rad/s
  max_acceleration: 5.0      # rad/s^2
  position_limits:
    joint_0: [-6.28, 6.28]   # Base
    joint_1: [-6.28, 6.28]   # Shoulder
    joint_2: [-3.14, 3.14]   # Elbow
    joint_3: [-6.28, 6.28]   # Wrist 1
    joint_4: [-6.28, 6.28]   # Wrist 2
    joint_5: [-6.28, 6.28]   # Wrist 3

estop:
  enable_estop: true
  robot_timeout_ms: 100.0    # 로봇 데이터 100ms 미수신 시 E-STOP
  hand_timeout_ms: 200.0     # 핸드 데이터 200ms 미수신 시 핸드 E-STOP
  safe_position:             # E-STOP 복구 안전 위치 (rad)
    - 0.0   # Base
    - -1.57 # Shoulder
    - 1.57  # Elbow
    - -1.57 # Wrist 1
    - -1.57 # Wrist 2
    - 0.0   # Wrist 3

logging:
  enable_logging: true
  log_frequency: 100.0       # Hz (제어율에서 서브샘플링)
  max_log_size_mb: 100
  log_directory: "/tmp/ur5e_logs"
```

### `config/hand_udp_receiver.yaml`

```yaml
udp:
  port: 50001                # UDP 수신 포트
  buffer_size: 1024          # 바이트
  timeout_ms: 1000           # 소켓 타임아웃

publishing:
  rate: 100.0                # ROS2 퍼블리시 주파수 (Hz)
  topic: "/hand/joint_states"

hand:
  num_joints: 4
  joint_names: [finger_1, finger_2, finger_3, finger_4]
  min_position: 0.0
  max_position: 1.0
```

---

## ROS2 토픽 인터페이스

### 구독 토픽

| 토픽 | 타입 | 발행자 | 설명 |
|------|------|--------|------|
| `/joint_states` | `sensor_msgs/JointState` | UR 드라이버 | 6-DOF 관절 위치/속도/힘 |
| `/target_joint_positions` | `std_msgs/Float64MultiArray` | 외부 노드 | 목표 관절 위치 (6개 값, rad) |
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | `hand_udp_receiver_node` | 핸드 상태 (4개 값) |
| `/hand/command` | `std_msgs/Float64MultiArray` | 외부 노드 | 핸드 명령 (4개 값) |

### 발행 토픽

| 토픽 | 타입 | 발행자 | 설명 |
|------|------|--------|------|
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | `custom_controller` | UR 위치 명령 (6개 값, rad) |
| `/system/estop_status` | `std_msgs/Bool` | `custom_controller` | E-STOP 상태 (true=활성) |

---

## UDP 핸드 프로토콜

핸드 시스템은 UDP로 77개의 `double` 값(616 bytes)을 전송합니다.

### 패킷 형식 (송신: 핸드 → ROS2)

```
오프셋    크기         필드
0         11 doubles   motor_pos[11]      (모터 위치)
88        11 doubles   motor_vel[11]      (모터 속도)
176       11 doubles   motor_current[11]  (모터 전류)
264       44 doubles   sensor_data[44]    (4 센서 × 11 데이터)
총계: 77 doubles = 616 bytes
```

수신 포트: **50001** (기본값, `hand_udp_receiver.yaml`에서 변경 가능)

### 패킷 형식 (수신: ROS2 → 핸드)

4개 `double` 값 → 모터 위치 명령 (정규화 0.0~1.0)

송신 포트: **50002** (기본값, `hand_udp.launch.py`에서 변경 가능)

### Python 예제 (핸드 시뮬레이터)

```python
import socket, struct, numpy as np

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target = ("127.0.0.1", 50001)

motor_pos     = [0.5] * 11
motor_vel     = [0.0] * 11
motor_current = [0.5] * 11
sensor_data   = [0.0] * 44

packet = struct.pack('77d', *(motor_pos + motor_vel + motor_current + sensor_data))
sock.sendto(packet, target)
```

---

## 성능 지표

| 항목 | 값 |
|------|-----|
| 제어 주파수 | 500Hz |
| E-STOP 감시 주기 | 50Hz (20ms) |
| 핸드 데이터 퍼블리시 | 100Hz |
| GUI 업데이트 | 100Hz (Qt 타이머 10ms) |
| 로봇 E-STOP 타임아웃 | 100ms |
| 핸드 E-STOP 타임아웃 | 200ms |
| CSV 로그 경로 | `/tmp/ur5e_control_log.csv` |
| 통계 저장 경로 | `/tmp/ur5e_stats/` |

---

## 문제 해결

### E-STOP이 계속 활성화됨

```bash
# 로봇 데이터 확인
ros2 topic hz /joint_states          # 500Hz여야 함
ros2 topic echo /system/estop_status  # E-STOP 상태 확인

# 핸드 데이터 확인 (핸드 없는 환경)
# hand_timeout_ms를 0으로 설정하거나 E-STOP 비활성화
# config/ur5e_rt_controller.yaml:
# estop:
#   enable_estop: false
```

### 500Hz 미달성

```bash
# 1) RT 커널 확인
uname -v  # "lowlatency" 또는 "PREEMPT_RT"

# 2) CPU 성능 모드 설정
sudo cpupower frequency-set -g performance

# 3) RMW 변경
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### UR 드라이버 연결 실패

```bash
# 네트워크 확인
ping 192.168.1.10

# Teach Pendant에서 External Control 프로그램 실행:
# Program → External Control → Run

# 포트 확인 (UR 기본: 50001, 30001~30004)
ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=<실제IP>
```

### `forward_position_controller` 활성화 실패

```bash
# 수동 전환
ros2 control switch_controllers \
    --deactivate scaled_joint_trajectory_controller \
    --activate forward_position_controller

# 상태 확인
ros2 control list_controllers
```

### GUI 실행 오류

```bash
# PyQt5 설치
sudo apt install python3-pyqt5
# 또는
pip3 install pyqt5
```

### 목표 위치 전송

```bash
# 목표 위치 수동 퍼블리시 (홈 포즈)
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \
    "data: [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]"
```

---

## 모니터링

```bash
# 제어 주파수 확인 (목표: 500Hz)
ros2 topic hz /forward_position_controller/commands

# 관절 상태 확인
ros2 topic echo /joint_states

# E-STOP 상태 확인
ros2 topic echo /system/estop_status

# 핸드 데이터 확인
ros2 topic hz /hand/joint_states

# 컨트롤러 목록
ros2 control list_controllers -v

# 데이터 헬스 모니터 단독 실행
python3 scripts/monitor_data_health.py

# 시스템 지터 측정 (RT 커널)
sudo cyclictest -l100000 -m -n -p99 -t1 -i200
```

---

## 고급 사용법

### 커스텀 제어기 추가

1. `include/ur5e_rt_controller/controllers/my_controller.hpp` 생성:

```cpp
#pragma once
#include "ur5e_rt_controller/rt_controller_interface.hpp"

namespace ur5e_rt_controller {

class MyController : public RTControllerInterface {
public:
  ControllerOutput compute(const ControllerState& state) noexcept override {
    ControllerOutput output;
    // 제어 로직 구현
    output.robot_cmd = robot_target_;
    output.valid = true;
    return output;
  }
  std::string name() const noexcept override { return "MyController"; }
};

}  // namespace ur5e_rt_controller
```

2. `src/custom_controller.cpp`에서 `PDController` 대신 사용:

```cpp
controller_(std::make_unique<ur5e_rt_controller::MyController>())
```

3. 재빌드:

```bash
colcon build --packages-select ur5e_rt_controller
```

### CSV 로그 형식

```
timestamp, current_pos_0, current_pos_1, ..., current_pos_5,
           target_pos_0, ..., target_pos_5,
           command_0, ..., command_5
```

예시:
```
0.000, 0.000, -1.570, 0.000, 0.000, 0.000, 0.000, 0.000, ...
0.002, 0.001, -1.569, 0.001, 0.001, 0.001, 0.001, 0.001, ...
```

### 모션 JSON 형식

```json
{
  "num_poses": 50,
  "poses": {
    "pose_0": [0.0, -1.57, 0.0, 0.0, 0.0, 0.0],
    "pose_1": [-0.5, -1.8, 1.2, 1.5, 0.3, 0.0]
  },
  "names": ["Pose 1", "Pose 2", ...]
}
```

---

## 의존성

### C++ 빌드 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS2 C++ 클라이언트 라이브러리 |
| `std_msgs`, `sensor_msgs` | 표준 메시지 타입 |
| `ur_msgs` | UR 전용 메시지 타입 |
| `controller_manager`, `controller_interface` | ros2_control 프레임워크 |
| `hardware_interface` | 하드웨어 추상화 레이어 |
| `realtime_tools` | 실시간 퍼블리셔/버퍼 |
| `Eigen3` | 선형대수 연산 (헤더 전용) |

### Python 의존성 (`requirements.txt`)

| 패키지 | 버전 | 용도 |
|--------|------|------|
| `matplotlib` | >=3.5.3 | 궤적 시각화 |
| `pandas` | >=1.5.3 | CSV 데이터 처리 |
| `numpy` | >=1.24.3 | 수치 연산 |
| `scipy` | >=1.10.1 | 신호 처리 |
| `PyQt5` | (시스템 패키지) | 모션 에디터 GUI |

---

## 참고 자료

- [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [ROS2 Control Framework](https://control.ros.org/)
- [PREEMPT_RT 실시간 리눅스](https://wiki.linuxfoundation.org/realtime/start)
- [Eigen3 문서](https://eigen.tuxfamily.org/dox/)

---

## 라이선스

MIT License

---

## 버전 이력

| 버전 | 변경사항 |
|------|----------|
| v4.0.0 | E-STOP 시스템 추가, 핸드/로봇 타임아웃 감시, 안전 위치 복귀 |
| v3.0.0 | PD 제어기 E-STOP 지원, 안전 위치 설정 |
| v2.0.0 | DataLogger CSV 로깅, 핸드 UDP 통합 |
| v1.0.0 | 초기 릴리스, P/PD 제어기, 기본 ROS2 노드 |

**최종 업데이트**: 2026-03-02
