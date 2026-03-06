# CLAUDE.md — ur5e_hand_udp

11-DOF 커스텀 핸드와 ROS2 사이의 **UDP 브리지 패키지**.
ROS2 노드 2개(`hand_udp_receiver_node`, `hand_udp_sender_node`)와
비-ROS2 헤더(`HandUdpReceiver`, `HandUdpSender`)로 구성된다.

---

## 파일 구조 및 역할

```
include/ur5e_hand_udp/
├── hand_udp_receiver.hpp   ← UDP 수신 + jthread + 콜백
└── hand_udp_sender.hpp     ← UDP 송신 (little-endian double 인코딩)

src/
├── hand_udp_receiver_node.cpp  ← ROS2 래퍼: UDP → /hand/joint_states (100Hz)
└── hand_udp_sender_node.cpp    ← ROS2 래퍼: /hand/command → UDP

config/
└── hand_udp_receiver.yaml

launch/
└── hand_udp.launch.py       ← 두 노드 동시 실행
```

---

## UDP 패킷 프로토콜

### 수신 (핸드 → ROS2) — 포트 50001

총 **77 double = 616 bytes** (little-endian):

| 오프셋 | 크기 | 필드 |
|--------|------|------|
| 0 | 11 doubles (88B) | `motor_positions[11]` |
| 88 | 11 doubles (88B) | `motor_velocities[11]` |
| 176 | 11 doubles (88B) | `motor_currents[11]` |
| 264 | 44 doubles (352B) | `sensor_data[44]` (4센서 × 11관절) |

`ParsePacket()`: `buffer.size() == 77 * sizeof(double)` 검증 후 `latest_data_` 업데이트.

### 송신 (ROS2 → 핸드) — 포트 50002

11 double (88 bytes, little-endian): 정규화 모터 명령 (0.0–1.0).
`EncodePacket()`: `std::vector<uint8_t>` 반환 — `sendto()`로 전송.

---

## HandUdpReceiver

```cpp
HandUdpReceiver receiver(50001);              // 기본: kUdpRecvConfig (Core 5, FIFO 65)
receiver.SetCallback([](std::span<const double,11> d) { /* 콜백 */ });
receiver.Start();   // socket() + bind() + jthread 시작
receiver.Stop();    // stop_token 요청 + join (소멸자도 자동 호출)
```

- **jthread (C++20)**: `stop_token`으로 협력적 취소. `ReceiveLoop(stop_token)` 내부에서
  `recvfrom()` timeout(1000ms) 후 `stop_token.stop_requested()` 확인.
- `ApplyThreadConfig(thread_cfg_)` — jthread 진입 직후 RT 스케줄링 적용.
- `GetLatestData()` — `data_mutex_` 하에 `latest_data_` 복사본 반환 (스레드 안전).

---

## HandUdpSender

```cpp
HandUdpSender sender("192.168.1.100", 50002);
sender.Initialize();
sender.SendCommand(commands_span);  // noexcept, 실패 시 false 반환
```

- `socket(AF_INET, SOCK_DGRAM, 0)` — 비연결 UDP.
- `EncodePacket()`: `memcpy`로 double → uint8_t 리틀-엔디안 직렬화.

---

## ROS2 노드

### hand_udp_receiver_node

- `HandUdpReceiver::SetCallback()`으로 수신 콜백 등록.
- 콜백은 `latest_hand_data_`에 저장 + `last_recv_time_` 갱신.
- `publish_timer_` (100Hz): `latest_hand_data_`를 `/hand/joint_states`로 퍼블리시.
  → **11개 motor_positions만** Float64MultiArray로 발행.
- `main()` 진입 시 `mlockall(MCL_CURRENT | MCL_FUTURE)` 호출.

### hand_udp_sender_node

- `/hand/command` (Float64MultiArray, 11개 값) 구독.
- 콜백에서 `sender_.SendCommand()` 직접 호출.
- `Initialize()` 실패 시 `RCLCPP_ERROR` 후 노드 종료.

---

## 실행

```bash
# 두 노드 동시 실행 (기본 포트 사용)
ros2 launch ur5e_hand_udp hand_udp.launch.py

# 커스텀 포트/IP
ros2 launch ur5e_hand_udp hand_udp.launch.py \
    udp_port:=50001 target_ip:=192.168.1.100 target_port:=50002
```

---

## 이 패키지를 수정할 때 주의사항

1. **패킷 크기 변경** (`kNumHandJoints` 변경 포함) → `ParsePacket()`의 size 검증,
   `EncodePacket()`의 span 크기, `hand_udp_sender_example.py`의 struct format 모두 변경 필요.

2. **수신 콜백** — jthread 컨텍스트(Core 5, FIFO 65)에서 실행됨.
   콜백 내에서 블로킹 작업 금지. 데이터 복사 후 즉시 반환.

3. **포트 충돌** — `ur5e_mujoco_sim`은 시뮬 환경에서 `/hand/joint_states`를 직접 퍼블리시.
   실제 핸드와 MuJoCo 시뮬을 동시에 실행하면 토픽이 충돌한다.

4. **4코어 시스템** — `HandUdpReceiver` 생성 시 `kUdpRecvConfig4Core` 명시적 전달:
   ```cpp
   HandUdpReceiver receiver(port, kUdpRecvConfig4Core);
   ```
   `hand_udp_receiver_node`는 `SelectThreadConfigs().udp_recv`를 사용해 자동 선택.
