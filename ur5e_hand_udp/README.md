# ur5e_hand_udp

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스 (v5.8.0)의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

UR5e RT Controller 스택의 **10-DOF 손 UDP 브리지 패키지**입니다. 외부 손 컨트롤러(하드웨어)와 ROS2 토픽 사이의 UDP request-response 통신을 담당합니다.

## 개요

```
ur5e_hand_udp/
├── include/ur5e_hand_udp/
│   ├── hand_packets.hpp          ← 와이어 포맷 구조체, 인코딩/디코딩 헬퍼
│   ├── hand_udp_codec.hpp        ← 공개 코덱 API (allocation-free, noexcept)
│   ├── hand_controller.hpp       ← 핵심 드라이버 (request-response 폴링)
│   └── hand_failure_detector.hpp ← 손 통신 장애 감지기 (v5.8.0)
├── src/
│   └── hand_udp_node.cpp         ← ROS2 노드 (HandController + FailureDetector)
├── config/
│   └── hand_udp_node.yaml        ← 노드 파라미터 설정 (ros__parameters)
├── launch/
│   └── hand_udp.launch.py        ← Hand UDP 런치
├── CMakeLists.txt
├── package.xml
├── README.md
└── CHANGELOG.md
```

**의존성 그래프 내 위치:**

```
ur5e_rt_base ← ur5e_hand_udp   (ur5e_rt_controller에 의존하지 않음)
```

---

## 손 구조 (10-DOF + 44 센서)

| 항목 | 값 | 상수 (ur5e_rt_base/types.hpp) |
|------|-----|-------------------------------|
| 모터 수 | **10** | `kNumHandMotors` |
| 핑거팁 수 | **4** | `kNumFingertips` |
| 핑거팁당 기압 센서 | 8 | `kBarometerCount` |
| 핑거팁당 ToF 센서 | 3 | `kTofCount` |
| 핑거팁당 센서 합계 | **11** (8+3) | `kSensorValuesPerFingertip` |
| 총 센서 값 | **44** (4×11) | `kNumHandSensors` |

> 와이어 포맷에는 핑거팁당 `reserved[5]` 필드가 추가로 포함되나, 디코딩 시 폐기됩니다.

---

## UDP 프로토콜 (Request-Response)

단일 UDP 소켓으로 **요청-응답** 방식 통신합니다 (포트: **55151**).

### 패킷 형식

**모터 패킷 (43 바이트, little-endian):**

```
오프셋  필드          타입              개수    설명
0       id            uint8_t           1       디바이스 ID (0x01)
1       cmd           uint8_t           1       커맨드 (아래 표 참조)
2       mode          uint8_t           1       모드 (기본 0x00)
3–42    data          uint32_t[10]      10      모터 데이터 (float → uint32 reinterpret)
```

**센서 요청 패킷 (3 바이트):**

```
오프셋  필드          타입              설명
0       id            uint8_t           0x01
1       cmd           uint8_t           0x14–0x17 (핑거팁 0–3)
2       mode          uint8_t           센서 모드 (0=Raw, 1=NN)
```

**센서 응답 패킷 (67 바이트):**

```
오프셋  필드           타입              설명
0–2     헤더           3B               id/cmd/mode 에코
3–34    barometer[8]   uint32_t[8]      기압 센서 (float)
35–54   reserved[5]    uint32_t[5]      예약 (디코딩 시 폐기)
55–66   tof[3]         uint32_t[3]      ToF 센서 (float)
```

### 커맨드 정의

| 커맨드 | 코드 | 송신 | 수신 | 설명 |
|--------|------|------|------|------|
| `kWritePosition` | `0x01` | 43B | — | 10개 모터 위치 명령 (응답 없음) |
| `kSetSensorMode` | `0x04` | 3B | 3B | 센서 모드 초기화 (Raw/NN) |
| `kReadPosition` | `0x11` | 43B | 43B | 10개 모터 위치 읽기 |
| `kReadVelocity` | `0x12` | 43B | 43B | 10개 모터 속도 읽기 |
| `kReadSensor0–3` | `0x14–0x17` | 3B | 67B | 핑거팁 센서 읽기 (각 11개 값) |

### 폴링 사이클

`HandController`가 매 사이클마다 아래 순서로 통신합니다:

```
1. WritePosition  (0x01) → 43B 송신 (새 명령이 있을 때만)
2. ReadPosition   (0x11) → 43B 송신 → 43B 수신 → 10 float (위치)
3. ReadVelocity   (0x12) → 43B 송신 → 43B 수신 → 10 float (속도)
4. ReadSensor0    (0x14) →  3B 송신 → 67B 수신 → 11 float (핑거팁 0)
5. ReadSensor1    (0x15) →  3B 송신 → 67B 수신 → 11 float (핑거팁 1)
6. ReadSensor2    (0x16) →  3B 송신 → 67B 수신 → 11 float (핑거팁 2)
7. ReadSensor3    (0x17) →  3B 송신 → 67B 수신 → 11 float (핑거팁 3)
```

---

## 노드 설명

### `hand_udp_node`

`HandController`를 사용하는 ROS2 노드입니다. 단일 프로세스에서 송수신을 모두 처리합니다.

- **내부**: `HandController` — request-response 폴링 (jthread, Core 5, SCHED_FIFO/65)
  - `recv_timeout_ms` 생성자 파라미터: YAML에서 구동되는 `SO_RCVTIMEO` 소켓 타임아웃 설정
  - `recv_error_count_` (`std::atomic<uint64_t>`): recv 실패 횟수를 추적하는 원자적 카운터
  - `SetEstopFlag(std::atomic<bool>*)`: 글로벌 E-Stop 플래그 전파를 위한 설정 메서드
  - `enable_write_ack` 플래그: 커맨드 ACK 메커니즘 활성화 (WritePosition 후 응답 대기)
- **퍼블리시**: `/hand/joint_states` (`std_msgs/Float64MultiArray`, 100Hz)
  - **64개 `double` 값**: `[positions:10] + [velocities:10] + [sensors:44]`
- **구독**: `/hand/command` (`std_msgs/Float64MultiArray`)
  - **10개** 정규화된 모터 명령 (0.0–1.0)
- `mlockall(MCL_CURRENT | MCL_FUTURE)` — 페이지 폴트 방지

---

## ROS2 인터페이스

| 토픽 | 타입 | 방향 | 크기 | 설명 |
|------|------|------|------|------|
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | 퍼블리시 | **64** | [0–9] 위치, [10–19] 속도, [20–63] 센서 |
| `/hand/command` | `std_msgs/Float64MultiArray` | 구독 | **10** | 정규화된 모터 명령 (0.0–1.0) |

### `/hand/joint_states` 데이터 레이아웃

```
인덱스   내용                    개수
[0–9]    motor_positions         10    모터 위치
[10–19]  motor_velocities        10    모터 속도
[20–30]  fingertip_0 sensors     11    기압[8] + ToF[3]
[31–41]  fingertip_1 sensors     11    기압[8] + ToF[3]
[42–52]  fingertip_2 sensors     11    기압[8] + ToF[3]
[53–63]  fingertip_3 sensors     11    기압[8] + ToF[3]
                                ────
                                 64
```

---

## 설정

### `config/hand_udp_node.yaml`

```yaml
/**:
  ros__parameters:
    # UDP 통신
    target_ip: "192.168.1.2"          # 핸드 컨트롤러 IP
    target_port: 55151                # 핸드 컨트롤러 포트
    recv_timeout_ms: 10               # SO_RCVTIMEO (ms)
    enable_write_ack: false           # 핸드 하드웨어 ACK 지원 시 true

    # ROS2 퍼블리시
    publish_rate: 100.0               # /hand/joint_states 주기 (Hz)

    # Failure Detector (50Hz non-RT jthread)
    enable_failure_detector: true     # 장애 감지기 활성화
    failure_threshold: 5              # 연속 장애 판정 횟수
    check_motor: true                 # 모터 위치 검사 활성화
    check_sensor: true                # 센서 데이터 검사 활성화
```

---

## 실행

```bash
ros2 launch ur5e_hand_udp hand_udp.launch.py \
    target_ip:=192.168.1.2 \
    target_port:=55151 \
    publish_rate:=100.0
```

### Launch 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `target_ip` | `192.168.1.2` | 손 컨트롤러 IP |
| `target_port` | `55151` | 손 컨트롤러 포트 |
| `publish_rate` | `100.0` | `/hand/joint_states` 퍼블리시 주기 (Hz) |

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select ur5e_rt_base ur5e_hand_udp --symlink-install
source install/setup.bash
```

---

## RT/안전 기능

### 실시간 스레드

| 항목 | 값 |
|------|-----|
| CPU 코어 | **Core 5** (`kUdpRecvConfig`) |
| 스케줄러 | `SCHED_FIFO` |
| 우선순위 | **65** |
| 메모리 잠금 | `mlockall(MCL_CURRENT \| MCL_FUTURE)` |

> v5.1.0에서 Core 3 → Core 5로 이동 (sensor_io 스레드와의 경합 방지).

### Allocation-free 설계

- 모든 코덱 함수: `noexcept`, 힙 할당 없음
- 폴링 루프: 고정 크기 배열만 사용 (`std::array`)
- `trivially_copyable` 패킷 구조체 (`#pragma pack`, `static_assert` 검증)

### E-STOP 연동

`ur5e_rt_controller`의 `config/ur5e_rt_controller.yaml`에서:

```yaml
estop:
  hand_timeout_ms: 200.0  # /hand/joint_states 갭이 200ms 초과 시 E-STOP
                           # 0으로 설정 시 손 E-STOP 비활성화
```

손이 연결되지 않은 경우: `hand_timeout_ms: 0` 설정.

### HandFailureDetector (v5.8.0)

`hand_failure_detector.hpp`에 정의된 C++ 클래스로, 손 통신 데이터의 이상 상태를 감지합니다.

- **50Hz `std::jthread`** 비-RT 모니터링 스레드로 동작
- **두 가지 장애 조건**:
  1. **All-zero 데이터**: 모든 값이 0인 프레임이 N회 연속 감지
  2. **Duplicate 데이터**: 동일한 값이 N회 연속 반복 감지
- **독립적 검사 채널**: 모터 위치(`check_motor`)와 센서 데이터(`check_sensor`)를 각각 독립적으로 검사
- **장애 콜백 등록**: 장애 감지 시 등록된 콜백을 호출하여 글로벌 E-Stop을 트리거
- **YAML 설정**:
  - `enable_failure_detector`: 장애 감지기 활성화 여부
  - `failure_threshold`: 연속 장애 판정 횟수 (N)
  - `check_motor`: 모터 위치 검사 활성화
  - `check_sensor`: 센서 데이터 검사 활성화

---

## `HandController` 사용 예시 (C++)

```cpp
#include "ur5e_hand_udp/hand_controller.hpp"

namespace urtc = ur5e_rt_controller;

// HandController 생성 (IP, 포트)
urtc::HandController controller("192.168.1.2", 55151);

// 상태 콜백 등록
controller.SetCallback([](const urtc::HandState& state) {
  // state.motor_positions[10]  — 모터 위치
  // state.motor_velocities[10] — 모터 속도
  // state.sensor_data[44]      — 촉각 센서 (4 핑거팁 × 11)
  printf("pos[0]=%.3f vel[0]=%.3f sensor[0]=%.3f\n",
         state.motor_positions[0],
         state.motor_velocities[0],
         state.sensor_data[0]);
});

controller.Start();  // 폴링 jthread 시작 (Core 5, SCHED_FIFO/65)

// 모터 명령 전송 (스레드 안전)
std::array<float, urtc::kNumHandMotors> cmd{};
cmd.fill(0.5f);
controller.SetTargetPositions(cmd);

// ... (노드 실행 중)

controller.Stop();  // jthread 협동 취소
```

---

## 개발/테스트

### 수신 확인

```bash
ros2 topic echo /hand/joint_states
ros2 topic hz /hand/joint_states  # 약 100Hz
```

### 명령 전송 테스트

```bash
# 10개 모터에 0.5 값 전송
ros2 topic pub /hand/command std_msgs/msg/Float64MultiArray \
    "{data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"
```

---

## 라이선스

MIT License
