# ur5e_hand_driver

![version](https://img.shields.io/badge/version-v5.16.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

UR5e RT Controller 스택의 **10-DOF 손 UDP 브리지 패키지**입니다. 외부 손 컨트롤러(하드웨어)와 ROS2 토픽 사이의 UDP request-response 통신을 담당합니다.

## 개요

```
ur5e_hand_driver/
├── include/ur5e_hand_driver/
│   ├── hand_packets.hpp          ← 와이어 포맷 구조체, 인코딩/디코딩 헬퍼
│   ├── hand_udp_codec.hpp        ← 공개 코덱 API (allocation-free, noexcept)
│   ├── hand_controller.hpp       ← 핵심 드라이버 (event-driven, busy skip, sensor decimation)
│   ├── hand_failure_detector.hpp ← 손 통신 장애 감지기 (v5.8.0)
│   └── fingertip_ft_inferencer.hpp ← ONNX 기반 핑거팁 F/T 추론 (v5.15.0)
├── src/
│   └── hand_udp_node.cpp         ← ROS2 노드 (HandController + FailureDetector)
├── config/
│   ├── hand_udp_node.yaml        ← 노드 파라미터 설정 (ros__parameters)
│   └── fingertip_ft_inferencer.yaml ← ONNX 모델 경로 + 캘리브레이션 설정
├── launch/
│   └── hand_udp.launch.py        ← Hand UDP 런치
├── CMakeLists.txt
├── package.xml
├── README.md
└── CHANGELOG.md
```

**의존성 그래프 내 위치:**

```
rtc_base, rtc_communication ← ur5e_hand_driver   (rtc_controller_manager에 의존하지 않음)
```

---

## 손 구조 (10-DOF + 44 센서)

| 항목 | 값 | 상수 (rtc_base/types/types.hpp) |
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
| `kWritePosition` | `0x01` | 43B | 43B | 10개 모터 위치 명령 (echo 응답 = 적용된 position) |
| `kSetSensorMode` | `0x04` | 3B | 3B | 센서 모드 초기화 (Raw/NN) |
| `kReadPosition` | `0x11` | 43B | 43B | 10개 모터 위치 읽기 (Individual 모드에서 echo로 대체) |
| `kReadVelocity` | `0x12` | 43B | 43B | 10개 모터 속도 읽기 |
| `kReadSensor0–3` | `0x14–0x17` | 3B | 67B | 핑거팁 센서 읽기 (각 11개 값) |

### Event-Driven 통신 사이클

`HandController`는 ControlLoop의 `SendCommandAndRequestStates()` 호출에 의해 event-driven으로 구동됩니다:

```
[Core 2] ControlLoop 500Hz
             │
             │ Phase 3.5: SendCommandAndRequestStates(cmd)
             │            → busy_ 체크 → condvar notify (non-blocking, ~1µs)
             │            → busy_ 시 skip + event_skip_count 증가
             ▼
[Core 5] EventLoop — condvar wait → wake
                     → WritePosition(cmd) + recv echo (= position)
                     → ReadVelocity (Individual) 또는 ReadAllMotors (Bulk)
                     → ReadSensors (sensor_decimation cycle마다)
                     → state_seqlock_ 갱신 (lock-free)
```

매 사이클 통신 순서 (Individual 모드):

```
1. WritePosition  (0x01) → 43B 송신 + 43B echo 수신 → position 획득 (ReadPosition 대체)
2. ReadVelocity   (0x12) → 43B 송신 → 43B 수신 → 10 float (속도)
3. ReadSensor0-3  (0x14–0x17) → sensor_decimation cycle마다 수행
   → 3B 송신 → 67B 수신 × 4 핑거팁 → 44 float (센서)
   → skip 시 이전 캐시 데이터 유지
```

매 사이클 통신 순서 (Bulk 모드):

```
1. WritePosition  (0x01) → 43B 송신 + 43B echo 수신 → 소켓 버퍼 정리
2. ReadAllMotors  (0x10) → 3B 송신 → 123B 수신 → pos[10]+vel[10]+cur[10]
3. ReadAllSensors (0x19) → sensor_decimation cycle마다 수행
   → 3B 송신 → 259B 수신 → 4 fingertips × 16 uint32
```

**Sensor Decimation** (`sensor_decimation: 3` 기본값):
- Individual: cycle 1–2: write+echo + vel = 2 round-trips (~0.8ms) ✓ < 2ms
- Individual: cycle 3: write+echo + vel + sensor×4 = 6 round-trips (~2.8ms)
- Bulk: cycle 1–2: write+echo + all_motors = 2 round-trips (~0.6ms) ✓ < 2ms
- Bulk: cycle 3: write+echo + all_motors + all_sensors = 3 round-trips (~1.2ms)

---

## 노드 설명

### `hand_udp_node`

`HandController`를 사용하는 ROS2 노드입니다. 단일 프로세스에서 송수신을 모두 처리합니다.

- **내부**: `HandController` — event-driven 드라이버 (jthread, Core 5, SCHED_FIFO/65)
  - `recv_timeout_ms` 생성자 파라미터: YAML에서 구동되는 `ppoll()` 기반 수신 타임아웃 (sub-ms 지원, hrtimer on PREEMPT_RT)
  - `recv_error_count_` (`std::atomic<uint64_t>`): recv 실패 횟수를 추적하는 원자적 카운터
  - `SetEstopFlag(std::atomic<bool>*)`: 글로벌 E-Stop 플래그 전파를 위한 설정 메서드
  - Write echo 항상 수신: 소켓 버퍼 오염 방지 + Individual 모드에서 position으로 활용 (ReadPosition 대체)
  - `busy_` flag: EventLoop busy 중 이벤트 skip 보호 (v5.11.0)
  - `sensor_decimation`: N cycle마다 센서 읽기 — 기본 3 (v5.11.0)
  - SeqLock 기반 lock-free 상태 공유 — priority inversion 방지 (v5.15.0)
  - `RtControllerNode`에서 직접 소유 (v5.11.0) — ControlLoop Phase 3.5에서 `SendCommandAndRequestStates()` 호출
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
    recv_timeout_ms: 0.4              # ppoll 수신 타임아웃 (ms, sub-ms 지원)
    sensor_decimation: 3              # N cycle마다 센서 읽기 (1=매번, 3=3cycle마다)

    # ROS2 퍼블리시
    publish_rate: 100.0               # /hand/joint_states 주기 (Hz)

    # Failure Detector (50Hz non-RT jthread)
    enable_failure_detector: true     # 장애 감지기 활성화
    failure_threshold: 5              # 연속 장애 판정 횟수
    check_motor: true                 # 모터 위치 검사 활성화
    check_sensor: true                # 센서 데이터 검사 활성화

    # Rate monitoring (failure detector에서 사용)
    min_rate_hz: 30.0               # 최소 허용 polling rate
    rate_fail_threshold: 5          # 연속 N회 미달 시 failure
```

---

## 실행

```bash
ros2 launch ur5e_hand_driver hand_udp.launch.py \
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
colcon build --packages-select rtc_base rtc_communication ur5e_hand_driver --symlink-install
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

### RT 안전 설계 (v5.15.0)

- 모든 코덱 함수: `noexcept`, 힙 할당 없음
- 폴링 루프: 고정 크기 배열만 사용 (`std::array`)
- `trivially_copyable` 패킷 구조체 (`#pragma pack`, `static_assert` 검증)
- **SeqLock** 기반 lock-free 상태 공유 — `state_mutex_` 제거, priority inversion 방지
- **printf 제거** — EventLoop (SCHED_FIFO) 스레드에서 stdout 출력 완전 제거
- **Write echo 항상 수신** — 소켓 버퍼 오염 방지, Individual 모드에서 echo를 motor position으로 활용 (ReadPosition 대체, 6→5 round-trips)
- **응답 cmd 검증** — `RequestMotorRead()`, `RequestAllMotorRead()`에 cmd 필드 검증 추가, stale 패킷 거부
- **`DrainStaleResponses()` 제거** — echo 항상 수신으로 불필요

### Sub-ms 수신 타임아웃 (v5.15.1)

- `SO_RCVTIMEO` → `ppoll()` 기반 `RecvWithTimeout()` (hrtimer, µs 정밀도)
- 기본값: `recv_timeout_ms: 0.4` (400µs)
- PREEMPT_RT 커널에서 hrtimer 사용으로 정확한 타이밍 보장

### E-STOP 연동

`rtc_controller_manager`의 `config/rt_controller_manager.yaml` (또는 `ur5e_bringup/config/ur5e_robot.yaml`)에서:

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
  3. **Low rate**: PollLoop rate가 `min_rate_hz` 미만인 상태가 N회 연속 감지
- **독립적 검사 채널**: 모터 위치(`check_motor`)와 센서 데이터(`check_sensor`)를 각각 독립적으로 검사
- **장애 콜백 등록**: 장애 감지 시 등록된 콜백을 호출하여 글로벌 E-Stop을 트리거
- **YAML 설정**:
  - `enable_failure_detector`: 장애 감지기 활성화 여부
  - `failure_threshold`: 연속 장애 판정 횟수 (N)
  - `check_motor`: 모터 위치 검사 활성화
  - `check_sensor`: 센서 데이터 검사 활성화

### HandCommStats (v5.9.0, v5.11.0 확장)

`HandController`에 추가된 통신 통계 구조체입니다. EventLoop 스레드에서만 쓰기, 외부에서 struct copy로 읽기.

```cpp
struct HandCommStats {
  uint64_t recv_ok{0};            // 수신 성공 횟수
  uint64_t recv_timeout{0};       // ppoll 수신 타임아웃 횟수
  uint64_t recv_error{0};         // 기타 수신 에러 횟수
  uint64_t total_cycles{0};       // 총 EventLoop 사이클 수
  uint64_t event_skip_count{0};   // EventLoop busy 중 skip된 이벤트 수 (v5.11.0)
  uint64_t cmd_mismatch{0};       // stale 패킷 거부 횟수 (cmd 불일치)
  uint64_t mode_mismatch{0};      // 센서 모드 검증 실패 (Raw/NN 불일치)
};

// 스냅샷 조회 (relaxed read, struct copy)
auto stats = controller.comm_stats();
```

### 센서 저역통과 필터링 (v5.14.0)

EventLoop에서 Bessel 4차 LPF를 적용하여 센서 노이즈를 제거합니다.

```yaml
baro_lpf_enabled: true
baro_lpf_cutoff_hz: 30.0    # 기압 센서 차단 주파수
tof_lpf_enabled: true
tof_lpf_cutoff_hz: 15.0     # ToF 센서 차단 주파수
```

- 실효 샘플링 레이트 = 500 Hz / `sensor_decimation`
- 필터링된 데이터: `HandState.sensor_data[]`, 원시 데이터: `HandState.sensor_data_raw[]`
- `ApplySensorFilters()`는 `noexcept`로 RT 루프에서 안전

---

### FingertipFTInferencer (F/T 추론, v5.15.0)

`OnnxEngine`을 상속하여 핑거팁별 ONNX 모델 기반 힘/토크 추론을 수행합니다.

```yaml
ft_inferencer:
  enabled: true
  num_fingertips: 4
  model_paths: ["/path/thumb.onnx", "/path/index.onnx", ...]
  calibration_enabled: true
  calibration_samples: 500     # 500 샘플 @500Hz = 1초 캘리브레이션
  thumb_max: [...]             # 16채널 정규화 최댓값 (핑거팁별)
  index_max: [...]
  middle_max: [...]
  ring_max: [...]
```

**3-Phase 추론 파이프라인 (RT-safe, noexcept):**

1. **전처리:** 센서 정규화 + 델타 계산 + FIFO 히스토리 시프트
2. **배치 추론:** `RunModels(indices, count)` (할당 없음)
3. **결과 복사:** → `FingertipFTState` (contact, force, direction, normal_force)

**캘리브레이션:** `FeedCalibration()`으로 기준선 오프셋 자동 측정 (첫 N 사이클)

---

### HandTimingProfiler

EventLoop의 각 단계별 소요시간을 추적합니다 (Individual/Bulk 모드별):

**Individual 모드:**

```cpp
struct HandTimingStats {
  double write_us{0.0};       // WritePosition 소요시간
  double read_pos_us{0.0};    // ReadPosition 소요시간
  double read_vel_us{0.0};    // ReadVelocity 소요시간
  double read_sensor_us{0.0}; // ReadSensor×4 소요시간
  double total_us{0.0};       // 전체 사이클 소요시간
};
```

**Bulk 모드:**
- `read_all_motor_us`: ReadAllMotors (0x10) 소요시간
- `read_all_sensor_us`: ReadAllSensors (0x19) 소요시간

히스토그램 기반 p95/p99 백분위수, 예산(2000µs) 초과 카운트 제공

### JSON 통계 내보내기 (v5.9.0, 경로 변경 v5.10.0)

노드 종료 시 `RTC_SESSION_DIR/hand/hand_udp_stats.json`에 통신 통계를 저장합니다 (세션 미설정 시 `/tmp/` 폴백):

```json
{
  "recv_timeout_ms": 0.400,
  "total_cycles": 150000,
  "recv_ok": 148500,
  "recv_timeout": 1200,
  "recv_error": 300,
  "event_skip_count": 42,
  "avg_rate_hz": 99.50,
  "elapsed_sec": 1500.00,
  "failure_detected": false,
  "timing_stats": {
    "write_us": {"mean": 120.5, "max": 450.2},
    "read_pos_us": {"mean": 180.3, "max": 520.1},
    "total_cycle_us": {"mean": 1350.0, "max": 3200.5}
  }
}
```

---

## `HandController` 사용 예시 (C++)

```cpp
#include "ur5e_hand_driver/hand_controller.hpp"

namespace urtc = rtc;

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

## 최적화 내역 (v5.16.1)

| 영역 | 변경 내용 |
|------|----------|
| **코드 검증** | C++20, `[[nodiscard]]`, `noexcept`, `static_assert` 확인 완료 — 이미 적용됨 |

---

## 라이선스

MIT License
