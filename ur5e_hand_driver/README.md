# ur5e_hand_driver

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

RTC 프레임워크의 **10-DOF 손 UDP 브리지 패키지**입니다. 외부 손 컨트롤러(하드웨어)와 ROS2 토픽 사이의 UDP request-response 통신을 담당합니다.

## 개요

```
ur5e_hand_driver/
├── include/ur5e_hand_driver/
│   ├── hand_packets.hpp          -- 와이어 포맷 구조체, 인코딩/디코딩 헬퍼
│   ├── hand_udp_codec.hpp        -- 공개 코덱 API (allocation-free, noexcept)
│   ├── hand_udp_transport.hpp    -- 저수준 UDP 소켓 관리 + 프로토콜 요청
│   ├── hand_controller.hpp       -- 핵심 드라이버 (event-driven, busy skip, sensor decimation)
│   ├── hand_sensor_processor.hpp -- 센서 후처리 (Bessel LPF, rate estimation, drift)
│   ├── hand_failure_detector.hpp -- 손 통신 장애 감지기 (50Hz non-RT jthread)
│   ├── hand_timing_profiler.hpp  -- EventLoop 단계별 타이밍 프로파일러
│   └── fingertip_ft_inferencer.hpp -- ONNX 기반 핑거팁 F/T 추론
├── src/
│   └── hand_udp_node.cpp         -- ROS2 노드 (HandController + FailureDetector)
├── config/
│   ├── hand_udp_node.yaml        -- 노드 파라미터 설정 (ros__parameters)
│   └── fingertip_ft_inferencer.yaml -- ONNX 모델 경로 + 캘리브레이션 설정
├── launch/
│   └── hand_udp.launch.py        -- Hand UDP 런치
├── CMakeLists.txt
└── package.xml
```

**의존성:**

```
rtc_base, rtc_communication, rtc_inference, rtc_msgs  <--  ur5e_hand_driver
```

---

## 아키텍처

### Event-Driven 통신

`HandController`는 `ControlLoop`의 `SendCommandAndRequestStates()` 호출에 의해 event-driven으로 구동됩니다. 단독 실행(standalone) 시에는 `/hand/joint_command` 구독으로 명령을 수신합니다.

```
[Core 2] ControlLoop 500Hz
             |
             | Phase 4: SendCommandAndRequestStates(cmd)
             |           -> busy_ 체크 -> condvar notify (non-blocking)
             |           -> busy_ 시 skip + event_skip_count 증가
             v
[Core 5] EventLoop -- condvar wait -> wake
                    -> WritePosition(cmd, kJoint) + recv echo
                    -> ReadAllMotors(kMotor) -- motor pos/vel/cur
                    -> ReadAllMotors(kJoint) -- joint pos/vel/cur
                    -> ReadSensors (sensor_decimation cycle마다)
                    -> FT Inference (sensor cycle + calibrated)
                    -> state_seqlock_ 갱신 (lock-free)
                    -> ROS2 직접 publish (timer 없음)
```

### 통신 모드

**Bulk 모드** (기본값: `communication_mode: "bulk"`):

```
1. WritePosition  (0x01, kJoint) -> 43B 송신 + 43B echo 수신
2. ReadAllMotors  (0x10, kMotor) -> 3B 송신 -> 123B 수신 -> pos[10]+vel[10]+cur[10]
3. ReadAllMotors  (0x10, kJoint) -> 3B 송신 -> 123B 수신 -> pos[10]+vel[10]+cur[10]
4. ReadAllSensors (0x19) -> sensor_decimation cycle마다
   -> 3B 송신 -> 259B 수신 -> 4 fingertips x 16 int32
```

**Individual 모드** (`communication_mode: "individual"`):

```
1. WritePosition  (0x01, kJoint)  -> 43B 송신 + 43B echo 수신
2. ReadPosition   (0x11, kMotor)  -> 3B 송신 -> 43B 수신
3. ReadPosition   (0x11, kJoint)  -> 3B 송신 -> 43B 수신
4. ReadVelocity   (0x12, kMotor)  -> 3B 송신 -> 43B 수신
5. ReadSensor0-3  (0x14-0x17) -> sensor_decimation cycle마다
   -> 3B 송신 -> 67B 수신 x 4 fingertips
```

### Dual Read (Motor + Joint 공간)

매 사이클 모터 상태를 두 번 읽습니다:
- **kMotor (0x00)**: 모터 엔코더 값 -> `motor_positions/velocities/currents`
- **kJoint (0x01)**: 펌웨어 기어비 변환 -> `joint_positions/velocities/currents`

Write 명령은 항상 `kJoint` 모드로 전송됩니다.

### Sensor Decimation

`sensor_decimation` 값에 따라 N cycle마다 센서를 읽습니다. 현재 노드 코드에서는 `sensor_decimation=1`로 고정되어 있어 매 사이클 센서를 읽습니다.

### ONNX F/T 추론

센서 사이클에서 `FingertipFTInferencer`를 통해 핑거팁별 접촉/힘 추론을 수행합니다:

1. **캘리브레이션**: 시작 시 N 샘플로 barometer baseline offset 자동 측정
2. **전처리**: barometer 정규화 + delta 계산 + FIFO history shift (12 row)
3. **추론**: per-fingertip ONNX 모델 (IoBinding, zero-alloc)
4. **출력**: contact probability (sigmoid), force vector (3), direction vector (3)

---

## 주요 컴포넌트

### HandController (`hand_controller.hpp`)

핵심 드라이버 클래스. Event-driven jthread(Core 5, SCHED_FIFO/65)로 동작합니다.

- **SeqLock** 기반 lock-free 상태 공유 (priority inversion 방지)
- **busy_ flag**: EventLoop 실행 중 이벤트 skip 보호
- **Write echo 항상 수신**: 소켓 버퍼 오염 방지
- **첫 사이클 read-only**: 초기 상태를 모르는 상태에서 zero 명령 전송 방지
- **Fake hand 모드**: `use_fake_hand=true` 시 UDP 소켓 없이 echo-back mock 동작

### HandUdpTransport (`hand_udp_transport.hpp`)

저수준 UDP 소켓 관리. `ppoll()` 기반 sub-ms 수신 타임아웃 (hrtimer on PREEMPT_RT).

**Mode 검증**: 모든 request-response 메서드(`RequestMotorRead`, `RequestAllMotorRead`, `RequestSensorRead`, `RequestAllSensorRead`)는 응답 패킷의 mode 필드가 요청한 mode와 일치하는지 검증합니다. 불일치 시 `comm_stats_.mode_mismatch` 카운터를 증가시키고 `false`를 반환합니다.

### HandSensorProcessor (`hand_sensor_processor.hpp`)

센서 후처리 파이프라인 (noexcept):
- `PreFilter()`: Rate estimator tick + 지연 필터 재초기화
- `ApplyFilters()`: Bessel 4차 LPF in-place (barometer + ToF)
- `DetectDrift()`: OLS 기반 원-샷 드리프트 감지 (1Hz 스로틀 경고)

### FingertipFTInferencer (`fingertip_ft_inferencer.hpp`)

Per-fingertip ONNX 모델 기반 힘/토크 추론 (3-head output):
- Input: `float32[1, H, 16]` (barometer 8ch + delta 8ch)
- Output: contact logit(1) + F(3) + u(3)
- `HAS_ONNXRUNTIME` 미정의 시 stub 구현 (추론 비활성)

### HandFailureDetector (`hand_failure_detector.hpp`)

50Hz non-RT jthread로 동작하는 장애 감지기:
1. All-zero 데이터: N회 연속 감지
2. Duplicate 데이터: N회 연속 반복
3. Low rate: polling rate가 `min_rate_hz` 미만
4. Link down: `consecutive_recv_failures` >= `link_fail_threshold`

### HandTimingProfiler (`hand_timing_profiler.hpp`)

EventLoop 단계별 소요시간 추적. 히스토그램 기반 p95/p99 백분위수, 예산(2000us) 초과 카운트.

---

## ROS2 인터페이스

### 퍼블리시

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/hand/joint_states` | `sensor_msgs/JointState` | BEST_EFFORT/1 | 관절 위치/속도/전류 (kJoint read) |
| `/hand/motor_states` | `sensor_msgs/JointState` | BEST_EFFORT/1 | 모터 위치/속도/전류 (kMotor read) |
| `/hand/sensor_states` | `rtc_msgs/HandSensorState` | BEST_EFFORT/1 | 핑거팁 센서 + F/T 추론 결과 (RT 전용) |
| `/hand/sensor_states/monitor` | `rtc_msgs/HandSensorState` | RELIABLE/10 | 동일 데이터, 비-RT 구독자용 (BT, GUI 등) |
| `/hand/link_status` | `std_msgs/Bool` | RELIABLE/TRANSIENT_LOCAL/1 | UDP 링크 상태 (publish_rate 기반 decimation) |

### 구독

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/hand/joint_command` | `rtc_msgs/JointCommand` | BEST_EFFORT/1 | 모터 명령 (values[10], kJoint 모드) |

토픽 이름은 YAML 파라미터(`command_topic`, `joint_state_topic` 등)로 변경 가능합니다.

---

## 설정

### `config/hand_udp_node.yaml`

노드가 선언하고 읽는 파라미터:

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `target_ip` | `"192.168.1.2"` | 핸드 컨트롤러 IP |
| `target_port` | `55151` | 핸드 컨트롤러 포트 |
| `recv_timeout_ms` | `10.0` | ppoll 수신 타임아웃 (ms, sub-ms 지원) |
| `publish_rate` | `100.0` | link_status decimation 기준 (Hz) |
| `communication_mode` | `"individual"` | `"individual"` 또는 `"bulk"` |
| `baro_lpf_enabled` | `false` | Barometer LPF 활성화 |
| `baro_lpf_cutoff_hz` | `30.0` | Barometer LPF 차단 주파수 |
| `tof_lpf_enabled` | `false` | ToF LPF 활성화 |
| `tof_lpf_cutoff_hz` | `15.0` | ToF LPF 차단 주파수 |
| `drift_detection_enabled` | `false` | OLS 드리프트 감지 |
| `drift_threshold` | `5.0` | 드리프트 기울기 임계값 |
| `drift_window_size` | `2500` | 드리프트 윈도우 크기 |
| `joint_state_names` | `[]` (기본 이름 사용) | 관절 이름 배열 (10개) |
| `motor_state_names` | `[]` (기본 이름 사용) | 모터 이름 배열 (10개) |
| `hand_fingertip_names` | `[]` (기본 이름 사용) | 핑거팁 이름 배열 (4개) |
| `command_topic` | `"/hand/joint_command"` | 명령 구독 토픽 |
| `joint_state_topic` | `"/hand/joint_states"` | 관절 상태 퍼블리시 토픽 |
| `motor_state_topic` | `"/hand/motor_states"` | 모터 상태 퍼블리시 토픽 |
| `sensor_topic` | `"/hand/sensor_states"` | 센서 상태 퍼블리시 토픽 |
| `link_status_topic` | `"/hand/link_status"` | 링크 상태 퍼블리시 토픽 |
| `enable_failure_detector` | `true` | 장애 감지기 활성화 |
| `failure_threshold` | `5` | 연속 장애 판정 횟수 |
| `check_motor` | `true` | 모터 데이터 검사 |
| `check_sensor` | `true` | 센서 데이터 검사 |
| `min_rate_hz` | `30.0` | 최소 허용 polling rate |
| `rate_fail_threshold` | `5` | 연속 N회 미달 시 failure |
| `check_link` | `true` | UDP 링크 검사 |
| `link_fail_threshold` | `10` | 연속 recv 실패 임계값 |

> 참고: `sensor_decimation`은 현재 노드 코드에서 `1`로 고정되어 있습니다. `joint_mode`는 사용되지 않습니다 (코드에서 항상 write=kJoint, read=kMotor+kJoint dual read). `enable_write_ack`는 deprecated (echo 항상 수신).

### `config/fingertip_ft_inferencer.yaml`

F/T 추론 설정. `hand_udp_node`에서 `ft_inferencer.*` 네임스페이스로 읽습니다.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `ft_inferencer.enabled` | `false` | ONNX 추론 활성화 |
| `ft_inferencer.num_fingertips` | `4` | 추론 대상 핑거팁 수 |
| `ft_inferencer.history_length` | `12` | FIFO history 길이 |
| `ft_inferencer.model_paths` | `["", "", "", ""]` | per-fingertip ONNX 모델 경로 |
| `ft_inferencer.calibration_enabled` | `true` | baseline offset 자동 보정 |
| `ft_inferencer.calibration_samples` | `500` | 캘리브레이션 샘플 수 |
| `ft_inferencer.thumb_max` | `[40000.0 x 16]` | thumb 정규화 최댓값 |
| `ft_inferencer.index_max` | `[40000.0 x 16]` | index 정규화 최댓값 |
| `ft_inferencer.middle_max` | `[40000.0 x 16]` | middle 정규화 최댓값 |
| `ft_inferencer.ring_max` | `[40000.0 x 16]` | ring 정규화 최댓값 |

모델 경로가 상대 경로인 경우 `ur5e_hand_driver` 패키지의 `models/` 디렉토리 기준으로 해석됩니다.

---

## 실행

### 단독 실행 (standalone)

```bash
ros2 launch ur5e_hand_driver hand_udp.launch.py \
    target_ip:=192.168.1.2 \
    target_port:=55151 \
    communication_mode:=bulk \
    recv_timeout_ms:=0.4
```

### Launch 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `target_ip` | `192.168.1.2` | 손 컨트롤러 IP |
| `target_port` | `55151` | 손 컨트롤러 포트 |
| `publish_rate` | `100.0` | link_status decimation 기준 (Hz) |
| `communication_mode` | `bulk` | `"individual"` 또는 `"bulk"` |
| `recv_timeout_ms` | `0.4` | ppoll 수신 타임아웃 (ms) |
| `use_fake_hand` | `false` | Fake hand echo-back mock |

### 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select rtc_base rtc_communication rtc_inference rtc_msgs ur5e_hand_driver --symlink-install
source install/setup.bash
```

---

## 의존성

- `rclcpp` -- ROS2 C++ 클라이언트
- `sensor_msgs` -- JointState 메시지
- `std_msgs` -- Bool 메시지
- `rtc_base` -- types, threading (SeqLock, ThreadConfig), filters (BesselFilter, SensorRateEstimator, SlidingTrendDetector), timing
- `rtc_communication` -- (의존성 선언)
- `rtc_inference` -- OnnxEngine, HAS_ONNXRUNTIME 전파
- `rtc_msgs` -- JointCommand, HandSensorState, FingertipSensor 메시지
- `ament_index_cpp` -- 패키지 share 디렉토리 경로 조회

---

## RT/안전 설계

| 항목 | 값 |
|------|-----|
| CPU 코어 | **Core 5** (`kUdpRecvConfig`) |
| 스케줄러 | `SCHED_FIFO` |
| 우선순위 | **65** |
| 메모리 잠금 | `mlockall(MCL_CURRENT \| MCL_FUTURE)` |

- 모든 코덱 함수: `noexcept`, 힙 할당 없음
- 고정 크기 배열만 사용 (`std::array`)
- `trivially_copyable` 패킷 구조체 (`#pragma pack`, `static_assert` 검증)
- **SeqLock** 기반 lock-free 상태 공유
- **printf 제거** -- EventLoop (SCHED_FIFO) 스레드에서 stdout 출력 없음
- **ppoll** 기반 sub-ms 수신 타임아웃 (hrtimer on PREEMPT_RT)
- Main thread: Core 0-1로 affinity 설정 (DDS 스레드가 RT 코어에 배치되는 것 방지)

---

## 라이선스

MIT License -- [LICENSE](../LICENSE)
