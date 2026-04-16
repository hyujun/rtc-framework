# ur5e_hand_driver

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

RTC 프레임워크의 **10-DOF 손 UDP 브리지 패키지**입니다. 외부 손 컨트롤러(하드웨어)와 ROS2 토픽 사이의 UDP request-response 통신을 담당합니다. LifecycleNode 기반으로 관리된 상태 전환을 지원합니다.

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
│   └── hand_udp_node.cpp         -- ROS2 LifecycleNode (HandController + FailureDetector)
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
| `/hand/calibration/status` | `rtc_msgs/CalibrationStatus` | RELIABLE/TRANSIENT_LOCAL/1 | 센서 캘리브레이션 진행/완료 상태 (센서 타입당 1메시지) |

### 구독

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/hand/joint_command` | `rtc_msgs/JointCommand` | BEST_EFFORT/1 | 모터 명령 (values[10], kJoint 모드) |
| `/hand/calibration/command` | `rtc_msgs/CalibrationCommand` | RELIABLE/1 | 센서 캘리브레이션 트리거 (현재 barometer 지원) |

토픽 이름은 YAML 파라미터(`command_topic`, `joint_state_topic`, `calibration_command_topic` 등)로 변경 가능합니다.

### 센서 캘리브레이션 재트리거

`baseline_offset` 을 런타임에 재측정하고 싶을 때 `/hand/calibration/command` 에
`CalibrationCommand` 를 publish 합니다. 드라이버는 EventLoop 스레드에서 요청을
consume 하여 `FingertipFTInferencer::ResetCalibration()` 을 호출한 뒤 다음
500 샘플(기본값) 동안 재누적합니다. 재누적 중에는 `baseline_offset_` 이 0 으로
초기화되어 FT inference 가 일시적으로 중지됩니다 (`is_calibrated()==false`).

예시:

```bash
# Barometer bias 재측정 (기본 샘플 수 사용)
ros2 topic pub --once /hand/calibration/command rtc_msgs/msg/CalibrationCommand \
    '{sensor_type: 0, action: 0, sample_count: 0}'

# 진행률 관찰
ros2 topic echo /hand/calibration/status
```

또는 `ur5e_bringup/scripts/demo_controller_gui.py` 의 **Control 탭 → Sensor
Calibration** 패널에서 `Calibrate` 버튼 클릭으로 동일하게 트리거할 수 있습니다.

**새 센서 추가 시** (확장성):
1. `rtc_msgs/msg/CalibrationCommand.msg` 의 `SENSOR_*` enum 에 값 추가
2. `HandController::DispatchCalibrationRequest()` switch 에 case 추가
3. `HandController::GetCalibrationStatus()` 에 분기 추가
4. `demo_controller_gui.py` 의 `SENSOR_CALIBRATIONS` 리스트에 dict 한 줄 추가

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
| `calibration_command_topic` | `"/hand/calibration/command"` | 캘리브레이션 명령 구독 토픽 |
| `calibration_status_topic` | `"/hand/calibration/status"` | 캘리브레이션 상태 퍼블리시 토픽 |
| `calibration_status_rate_hz` | `5.0` | 캘리브레이션 상태 publish 주기 (Hz) |
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

## 로깅 (Logging)

### 분류 독트린

| 레벨 | 용도 | 예시 |
|------|------|------|
| `FATAL` | 프로세스를 계속 실행할 수 없는 상태 | UDP 소켓 생성 실패 (포트 점유), 잘못된 설정 |
| `ERROR` | 복구 불가능한 실패, 사용자 개입 필요 | FT 모델 로드 실패, FailureDetector trigger, 센서모드 전환 실패 |
| `WARN` | 복구 가능한 실패/이상 상태, 자동 재시도 중 | UDP recv 실패 누적, 링크 복구 중, fake 모드 안내 |
| `INFO` | 사용자가 알아야 할 1 Hz 미만 상태 전환 | 노드 시작/종료, 캘리브레이션 START/COMPLETE, 링크 복구 |
| `DEBUG` | 개발자 진단용 (기본 꺼짐) | cycle counter, FailureDetector 스레드 lifecycle |

**핵심 규칙**:

- `HandController::EventLoop`, `HandUdpTransport::Send/Recv`, `HandSensorProcessor::PreFilter/ApplyFilters`, `FingertipFTInferencer::Infer` 는 모두 **500 Hz UDP 폴링 hot path** 다. 정상 경로의 `INFO`/`WARN` 직접 호출은 **금지** — 반복될 수 있는 메시지는 반드시 `*_THROTTLE` 매크로를 사용한다.
- **`RCLCPP_*_ONCE` 금지**. `_ONCE` 도 첫 호출에서는 동일한 fmt 포맷 할당을 수행하므로 RT 안전이 아니며, 조건이 다시 참이 될 때 침묵해 버린다. 대신 `*_THROTTLE` 을 `kThrottleIdleMs` 와 함께 사용한다 (예: `HandSensorProcessor::PreFilter` 의 BesselFilter 재초기화 실패 경고).
- **루프 기반 per-element 로그 금지.** RT 스레드에서 `for` 로 돌며 per-channel 로그를 내보내는 패턴은 1초 주기 throttle 을 만족하더라도 집계 1회로 축소해야 한다. 예: `HandSensorProcessor::ThrottledDriftWarning` 은 플래그된 채널 수 + 첫 위반 id/slope 를 **단 1 개의 `WARN_THROTTLE`** 로만 내보낸다 (최대 길이 고정 → 절단 없음).
- **RT 핫패스의 포맷 인자 수를 최소화한다.** 엣지 트리거되는 링크 UP/DOWN 조차 arg 0~1개로 유지. 상세 데이터는 `SeqLock` 상태나 `CommStats`/`drift_result_` 구조체 쪽에 쌓아 두고, 필요한 쪽(`SaveCommStats`, 비-RT consumer)이 끌어가도록 한다.
- THROTTLE 주기는 매직넘버 대신 `hand_logging.hpp` 의 표준 상수를 사용한다.
- Non-RT 경로(init/shutdown, `HandFailureDetector`, `hand_udp_node` 콜백, `SaveCommStats`) 의 INFO/WARN 은 *최대한 풍부하게* 작성한다. 그렙 한 줄로 세션 결과를 진단할 수 있어야 한다: cycle/rate/ok%/timeout%/err% 등 비율과 실패 원인을 한 줄에 담는다 (`SaveCommStats` 요약 참고).
- 메시지 본문에 클래스 이름을 박아넣지 않는다. 서브-로거 이름이 곧 식별자다 (`hand.ctrl`).

### 서브-로거 네임스페이스

| 서브-로거 | 사용처 |
|-----------|--------|
| `hand.node` | `HandUdpNode` (ROS2 노드 lifecycle, 토픽 구독/발행, link 상태 전이) |
| `hand.ctrl` | `HandController` (lifecycle + EventLoop, 캘리브레이션 dispatch) |
| `hand.udp` | `HandUdpTransport` (소켓 open/close, 센서모드 전환) |
| `hand.sensor` | `HandSensorProcessor` (LPF init, drift detection, BesselFilter 재초기화) |
| `hand.fail` | `HandFailureDetector` (50 Hz 워치독 스레드) |
| `hand.ft` | `FingertipFTInferencer` (ONNX 모델 로드, 캘리브레이션, inference 예외) |

### THROTTLE 주기 표준

`ur5e_hand_driver::logging` 네임스페이스에 정의된 상수만 사용한다 (`hand_logging.hpp`):

| 상수 | 값 [ms] | 용도 |
|------|---------|------|
| `kThrottleFastMs` | 500 | UDP recv 실패 누적 등 빠른 진행 표시 |
| `kThrottleSlowMs` | 2000 | 캘리브레이션 dispatch, 일반 반복 경고 |
| `kThrottleIdleMs` | 10000 | BesselFilter 재초기화 실패, one-shot 전이 안전 그물 |
| `kThrottleHotMs` | 5000 | RT 핫패스 예외 경로 (FT inference exception) |

### 실시간 필터링 예시

```bash
# UDP 트랜스포트 레이어만 DEBUG 활성화
ros2 service call /hand_udp_node/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'hand.udp', level: 10}]}"

# 모든 hand.* 서브로거 동시에 끄기 (계층 매칭)
ros2 service call /hand_udp_node/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'hand', level: 50}]}"

# Failure detector 만 끄기 (false-positive 노이즈 제거)
ros2 service call /hand_udp_node/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'hand.fail', level: 50}]}"
```

콘솔 출력에 로거 이름을 표시하려면:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
```

---

## 라이선스

MIT License -- [LICENSE](../LICENSE)
