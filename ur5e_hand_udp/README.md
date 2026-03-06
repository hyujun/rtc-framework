# ur5e_hand_udp

> **Note:** This package is part of the UR5e RT Controller workspace (v5.2.2). For full architecture details, installation instructions, and ROS 2 Jazzy compatibility, please refer to the [Root README](../README.md) and [Root CLAUDE.md](../CLAUDE.md).
UR5e RT Controller 스택의 **11-DOF 손 UDP 브리지 패키지**입니다. 외부 손 컨트롤러(하드웨어 또는 시뮬레이터)와 ROS2 토픽 사이의 UDP 통신을 담당합니다.

## 개요

```
ur5e_hand_udp/
├── include/ur5e_hand_udp/
│   ├── hand_udp_receiver.hpp    ← UDP 수신 (jthread, 포트 50001)
│   └── hand_udp_sender.hpp      ← UDP 송신 (리틀 엔디언 double, 포트 50002)
├── src/
│   ├── hand_udp_receiver_node.cpp ← UDP → /hand/joint_states (100Hz)
│   └── hand_udp_sender_node.cpp   ← /hand/command → UDP
├── config/
│   └── hand_udp_receiver.yaml
└── launch/
    └── hand_udp.launch.py
```

**의존성 그래프 내 위치:**

```
ur5e_rt_base ← ur5e_hand_udp   (ur5e_rt_controller에 의존하지 않음)
```

---

## UDP 프로토콜

### 수신 (손 상태, 포트 50001)

패킷 크기: **77 × `double` = 616 바이트**

| 오프셋 | 필드 | 개수 | 설명 |
|--------|------|------|------|
| 0–10 | `motor_positions` | 11 | 모터 위치 |
| 11–21 | `motor_velocities` | 11 | 모터 속도 |
| 22–32 | `motor_currents` | 11 | 모터 전류 |
| 33–76 | `sensor_data` | 44 | 촉각 센서 (관절당 4개) |

모든 값: 호스트 바이트 순서(little-endian) `double`

### 송신 (손 명령, 포트 50002)

패킷 크기: **11 × `double` = 88 바이트**

- 11개 모터 명령 (정규화 0.0–1.0)
- 리틀 엔디언 `double` 인코딩

---

## 노드 설명

### `hand_udp_receiver_node`

UDP 소켓에서 손 상태 패킷을 수신하여 ROS2 토픽으로 중계합니다.

- **수신**: UDP 포트 50001에서 616바이트 패킷
- **퍼블리시**: `/hand/joint_states` (`std_msgs/Float64MultiArray`, 100Hz)
  - **11개 `double` 값** — `motor_positions[11]` (모터 위치)
  - 속도/전류/센서 데이터는 UDP로 수신하지만 `HandUdpReceiver` 콜백이 11개만 전달함
- `std::jthread` (C++20 협동 취소) 사용으로 깔끔한 종료 보장
- RT 스레드 설정: **Core 5**, SCHED_FIFO/65 (`kUdpRecvConfig`, v1.1.0: Core 3→5로 변경)
- 패킷 간격 통계 로깅 (5초마다)

### `hand_udp_sender_node`

ROS2 토픽 구독하여 손 명령을 UDP로 전송합니다.

- **구독**: `/hand/command` (`std_msgs/Float64MultiArray`)
  - 11개 정규화된 모터 명령 (0.0–1.0)
- **송신**: UDP `target_ip:target_port` (기본값: `192.168.1.100:50002`)

---

## ROS2 인터페이스

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | 퍼블리시 | **11개** 모터 위치값 (100Hz) |
| `/hand/command` | `std_msgs/Float64MultiArray` | 구독 | 11개 정규화된 손 명령 (0.0–1.0) |

---

## 설정

### `config/hand_udp_receiver.yaml`

```yaml
udp:
  port: 50001          # 수신 포트
  buffer_size: 1024    # 버퍼 크기 (바이트)
  timeout_ms: 1000     # 소켓 타임아웃

publishing:
  rate: 100.0          # /hand/joint_states 퍼블리시 주기 (Hz)
  topic: "/hand/joint_states"

monitoring:
  enable_statistics: true
  statistics_period: 5.0  # 통계 출력 주기 (초)
```

---

## 실행

### 손 UDP 노드만 실행

```bash
ros2 launch ur5e_hand_udp hand_udp.launch.py \
    udp_port:=50001 \
    target_ip:=192.168.1.100 \
    target_port:=50002
```

### Launch 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `udp_port` | `50001` | UDP 수신 포트 |
| `target_ip` | `192.168.1.100` | 손 컨트롤러 IP |
| `target_port` | `50002` | UDP 송신 포트 |

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select ur5e_rt_base ur5e_hand_udp --symlink-install
source install/setup.bash
```

---

## 개발/테스트

### 합성 UDP 데이터 전송 (테스트용)

`ur5e_tools` 패키지의 `hand_udp_sender_example.py`를 사용합니다:

```bash
ros2 run ur5e_tools hand_udp_sender_example.py
```

정현파 또는 정적 손 데이터를 포트 50001로 전송합니다.

### 수신 확인

```bash
ros2 topic echo /hand/joint_states
ros2 topic hz /hand/joint_states  # 약 100Hz
```

---

## 손 E-STOP 설정

`ur5e_rt_controller`의 `config/ur5e_rt_controller.yaml`에서:

```yaml
estop:
  hand_timeout_ms: 200.0  # /hand/joint_states 갭이 200ms 초과 시 E-STOP
                           # 0으로 설정 시 손 E-STOP 비활성화
```

손이 연결되지 않은 경우: `hand_timeout_ms: 0` 설정.

---

## `HandUdpReceiver` 사용 예시 (C++)

```cpp
#include "ur5e_hand_udp/hand_udp_receiver.hpp"

ur5e_rt_controller::HandUdpReceiver receiver(/*port=*/50001);

// 콜백 시그니처: std::span<const double, kNumHandJoints> (11개 모터 위치)
receiver.SetCallback([](std::span<const double, ur5e_rt_controller::kNumHandJoints> data) {
  // data[0..10] = motor_positions[11]
  for (std::size_t i = 0; i < data.size(); ++i) {
    printf("motor[%zu] = %.3f\n", i, data[i]);
  }
});

receiver.Start();  // UDP 수신 jthread 시작

// ... (노드 실행 중)

receiver.Stop();  // jthread 협동 취소
```

---

## 라이선스

MIT License
