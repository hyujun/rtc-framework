# udp_hand_driver

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

RTC 프레임워크의 **10-DOF 손 UDP 브리지 패키지**입니다. 외부 손 컨트롤러(하드웨어)와 ROS2 토픽 사이의 UDP request-response 통신을 담당합니다. LifecycleNode 기반으로 관리된 상태 전환을 지원합니다.

## 개요

```
udp_hand_driver/
├── include/udp_hand_driver/
│   ├── udp_hand_packets.hpp          -- 와이어 포맷 구조체, 인코딩/디코딩 헬퍼
│   ├── udp_hand_codec.hpp        -- 공개 코덱 API (allocation-free, noexcept)
│   ├── udp_hand_transport.hpp    -- 저수준 UDP 소켓 관리 + 프로토콜 요청
│   ├── udp_hand_controller.hpp       -- 핵심 드라이버 (event-driven, busy skip, sensor decimation)
│   ├── udp_hand_sensor_processor.hpp -- 센서 후처리 (Bessel LPF, rate estimation, drift)
│   ├── udp_hand_failure_detector.hpp -- 손 통신 장애 감지기 (50Hz non-RT jthread)
│   ├── udp_hand_timing_profiler.hpp  -- EventLoop 단계별 타이밍 프로파일러
│   ├── udp_hand_timing_logger.hpp -- per-tick CSV writer (mpc_timing_log 패턴)
│   ├── udp_hand_node.hpp         -- UdpHandNode 클래스 선언 (LifecycleNode)
│   ├── udp_hand_constants.hpp    -- hand 전용 상수/타입 (kNumHandMotors 등), 필터 alias
│   ├── udp_hand_state.hpp        -- UdpHandState 구조체 (SeqLock 공유 상태)
│   ├── udp_hand_logging.hpp      -- 계층적 sub-logger 팩토리 + THROTTLE 상수
│   ├── protocol/
│   │   └── sensor_protocol.hpp   -- SensorProtocol version seam (1a/1b bulk-sensor 추상화)
│   ├── fake_hand_firmware.hpp    -- full-SIL 루프백 firmware 시뮬레이터 (device-side, header-only)
│   ├── fake_hand_lpf.hpp         -- fake-hand 1차 LPF joint step (controller/firmware 공유)
│   └── fingertip_ft_inferencer.hpp -- ONNX 기반 핑거팁 F/T 추론
├── src/
│   ├── udp_hand_node.cpp           -- main() (mlockall, CPU affinity, spin)
│   ├── udp_hand_node_lifecycle.cpp -- 6 lifecycle 콜백 + dtor + Drain + teardown helper
│   ├── udp_hand_node_publish.cpp   -- Preallocate / PollAndPublish / PublishState / PublishCalibrationStatus
│   ├── udp_hand_node_stats.cpp     -- SaveCommStats (hand_udp_stats.json writer)
│   ├── fake_hand_firmware_main.cpp -- fake_hand_firmware 실행파일 (socket/param/log 껍데기)
│   └── protocol/
│       └── sensor_protocol.cpp     -- SensorProtocol 구현 (정적 lib `udp_hand_protocol`)
├── config/
│   ├── udp_hand_node.yaml        -- 노드 파라미터 설정 (ros__parameters)
│   ├── fake_hand_firmware.yaml   -- SIL firmware 시뮬레이터 설정
│   └── fingertip_ft_inferencer.yaml -- ONNX 모델 경로 + 캘리브레이션 설정
├── launch/
│   └── udp_hand.launch.py        -- Hand UDP 런치 (sil_mode: off/loopmodel/firmware 단일 진입점)
├── CMakeLists.txt
└── package.xml
```

**의존성:**

```
rtc_base, rtc_communication, rtc_inference, rtc_msgs  <--  udp_hand_driver
```

---

## 아키텍처

### Self-Clocked 통신

`UdpHandController`의 `CommLoop`(`rtc::PeriodicRtThread` 서브클래스)는 `loop_rate_hz`(기본 500Hz)로 **자율 tick**한다. read + state publish 는 명령 도착과 **무관하게** 매 주기 수행되고, write UDP 는 `SendCommandAndRequestStates()`로 명령이 stage 됐을 때만 실행된다. 통합 모드에서는 `rtc_controller_manager`의 `ControlLoop`가 active controller 있을 때만 `/hand/joint_command`를 publish → write 는 control_rate 로, deactivate 시 정지(read/publish 는 계속). 단독 실행(standalone) 시에는 `/hand/joint_command` 구독으로 명령을 수신한다. CM clock 과 hand clock 은 독립(topic 경유).

```
[명령 소스] ControlLoop(통합) 또는 /hand/joint_command sub(standalone)
             |
             | SendCommandAndRequestStates(cmd)
             |   -> staged_cmd_seqlock_ store + event_pending_ set (release)
             |      (RT-safe, lock-free, 별도 wake 없음)
             v
[hand_driver core] CommLoop -- clock_nanosleep(loop_rate_hz) 자율 tick
                    -> [event_pending_ latch 시] WritePosition + recv echo (명령 있을 때만)
                    -> ReadAllMotors(kMotor) -- motor pos/vel/cur
                    -> ReadAllMotors(joint_io_mode) -- joint pos/vel/cur
                    -> ReadSensors (sensor_decimation cycle마다)
                    -> FT Inference (sensor cycle + calibrated)
                    -> state_seqlock_ 갱신 (lock-free) — tick 종료

[hand_driver core] main executor (CFS) -- publish_timer_ 폴링 (loop_rate_hz × 2)
                    -> state_sequence() 가 전진했으면 snapshot 을 publish
                    -> now() · 메시지 작성 · joint/motor/sensor/link publish
                    -> joint/calib command 구독, calib status 5 Hz

[aux core (OS slot)] hand_aux_io executor (CFS)
                    -> 타이밍 CSV drain (1 Hz, 버스트당 최대 512행)
                    -> stats JSON 저장 (10 s)
```

### 스레드 모델 (issue #345)

프로세스는 RTC 스레드 3개 + DDS 스레드를 가진다. 분리 축은 core 소속이 아니라 **연산량**이다 — 가벼운 CFS 레인은 CommLoop 과 같은 코어에 둬도 FIFO 65 가 항상 선점하므로 문제가 없고, blocking 파일 I/O 만 다른 코어로 뺀다.

| 스레드 | core | 스케줄러 | 하는 일 |
|---|---|---|---|
| `hand_udp_recv` (CommLoop) | `hand_driver` slot | **SCHED_FIFO 65** | UDP I/O · 센서 처리 · ONNX 추론 · SeqLock store |
| main = executor | `hand_driver` slot | CFS | NRT publish 소비자, command/calib 구독, calib status |
| failure detector | `hand_driver` slot (상속) | CFS nice −5 | 50 Hz SeqLock 폴링 감시 |
| `hand_aux_io` | **aux slot** (`aux_cpu_slot`, 기본 0) | CFS | 타이밍 CSV drain, stats JSON — blocking 파일 쓰기 |
| DDS | `hand_driver` slot | CFS | launch `pin_dds_threads_to_slot` 이 co-pin |

`hand_udp_recv` 와 detector 는 `cpu_core = -1` 로 **프로세스 affinity 를 상속**한다. `hand_udp_recv` 에 명시 slot 을 주지 않는 것은 의도적이다 — `ApplyThreadConfig` 는 affinity → 정책 → 이름 순서로 진행하고 affinity 실패 시 조기 반환하므로, 명시 pin 은 배치를 바꾸지 않으면서 **affinity 실패가 FIFO 65 와 스레드 이름을 함께 날리는 경로만** 새로 만든다 (이름 없는 스레드는 `verify_rt_runtime.sh` 가 찾지 못한다).

`use_cpu_affinity:=false` 는 프로세스 안의 두 pin(main → `hand_driver`, `hand_aux_io` → aux)을 건너뛴다. **스케줄링 정책은 유지되므로** CommLoop 은 그대로 SCHED_FIFO 65 다 — 코어 confinement 만 사라진다.

### NRT publish mailbox

publish 는 CommLoop 이 호출하는 콜백(push)이 아니라 executor 가 당겨가는 **pull** 이다. RT tick 은 `state_seqlock_` / `ft_seqlock_` 에 고정 크기 POD 를 store 만 하고, `now()` · 메시지 작성 · publish 는 전부 NRT 레인에서 일어난다.

- **latest-wins / overwrite**: mailbox 는 큐가 아니다. 폴 한 번은 publish 한 번이며, 그 사이 여러 tick 이 지났다면 **가장 최신 snapshot 만** 나가고 중간 것은 덮인다. 손실은 카운터로 보이지 않는다 — 관측하려면 `state_sequence()` 증가분과 publish 수를 대조한다.
- **폴링 주기 = `loop_rate_hz × kPublishPollOversampling(2.0)`**. 생산과 같은 레이트로 폴링하면 두 tick 이 두 폴 사이에 들어갈 때마다 한 샘플이 영구 손실돼 토픽 레이트가 `loop_rate_hz` 아래로 내려앉는다. 오버샘플링은 그 손실 꼬리를 wakeup 비용(새 데이터가 없으면 atomic load 1회)과 맞바꾼다.
- **카운터는 호출 횟수가 아니라 cycle 수**. `SeqLock` 은 Store 당 sequence 를 2 증가시키므로 완료된 tick 수는 `(현재 − 직전) / 2` 이고, 홀수 값은 쓰기 진행 중이라는 뜻이라 baseline 으로 삼으면 이후 모든 차분이 한 스텝 어긋난다 (`LastCompletedWrite` 로 내림). `link_status` 감쇠와 주기 로그는 이 cycle 수를 누적하고 **나머지를 carry** 하므로, 폴이 밀린 구간에서도 유효 레이트가 떨어지지 않는다.

**Command-gated write**: write(`WritePosition`)는 **per-command** 게이트다 — `event_pending_`가 set 된 cycle 에서 `pending_cmd_`를 latch 하고 `has_pending_write_`를 세운 뒤, 1회 write 시도 후 성공/실패 무관 clear 한다. 명령이 끊긴 구간은 read-only cycle 만 돌고 펌웨어는 마지막 위치를 hold 한다(이전의 영구 latch → stale 재전송 문제 제거).

**Startup write-gate**: write 는 `state_read_once_`(첫 상태 read 완료) **및** `has_pending_write_`(pending 명령) 가 **둘 다** 참이어야 실행된다. 첫 상태 read 전에는 명령을 latch 만 하고 write 는 보류 → lifecycle activate 직후 hold 명령 publish 전 창에서 손을 q=0 으로 끌어내리지 않는다(no-jump 계약).

**E-Stop self-exit**: E-Stop 은 매 tick(감쇠 skip 여부와 무관) 검사한다. 활성 시 CommLoop 은 RT 핫패스에서 **순수 atomic** `RequestLoopExit()`(mutex/condition_variable 미사용, RT-10 준수)로 자기 종료를 예약하고 `running_` 을 내린 뒤 return 한다. zero-write 는 핫패스가 아니라 loop unwind 시 base 가 1회 호출하는 `OnLoopAborted()`(→`OnCommLoopAborted()`)에서 수행된다. 재시작(`deactivate→activate`) 시 `Start()` 가 `exit_from_loop_` 및 link/telemetry 카운터(`consecutive_recv_failures_`, `cycle_count_`, `comm_skip_count_`, transport comm-stats)를 리셋하므로 정상 링크에서 오탐 link-down/rate E-STOP 이 발생하지 않는다.

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

### 센서 프로토콜 버전 (`protocol_version`)

펌웨어 센서 응답의 wire 포맷은 버전마다 다르며, 이 차이는 abstract
`SensorProtocol` (`protocol/sensor_protocol.hpp`) 로만 격리된다 — EventLoop 는
버전 문자열이 아닌 polymorphic capability (`HasMotorSpaceRead`, `JointIoMode`,
`RunsSensorPostProcess`, `VerifiesResponseMode`, `VerifiesBulkSensorResponseMode`) 로 분기한다 (ARCH-3). read·write 경로 골격은
버전 공통이나, joint-space I/O 의 **MODE byte** 는 `JointIoMode()` 로 갈린다 (1a=kJoint, 1b=kMotor —
1b 펌웨어는 kMotor 로만 joint state/command 를 서비스한다). bulk sensor **요청**(0x19)은 공통이고
**응답 크기·디코드, 그리고 응답 MODE byte 검증 여부**가 갈린다 — 1b 펌웨어는 bulk sensor 응답의
MODE byte 를 임의값으로 echo 하므로 그 경로만 검증을 끈다 (`VerifiesBulkSensorResponseMode()`).

| | `"1a"` (기본) | `"1b"` |
|---|---|---|
| bulk sensor 응답 | 259B (4 × 16 int32 baro/reserved/tof) | 99B (3B 헤더 + 4 × 6 float32) |
| 핑거당 데이터 | barometer[8] + tof[3] | [fx, fy, fz, Lx, Ly, Temp] |
| 후처리 | LPF / drift / F-T 추론 | 없음 (force 는 firmware 계산) |
| joint I/O MODE (`JointIoMode`, write + joint read) | `kJoint` (기어비 매핑) | `kMotor` (kJoint 요청 시 joint 데이터 미응답) |
| joint/set-mode 응답 MODE 검증 (`VerifiesResponseMode`) | strict | **off** (모든 응답의 MODE byte 가 임의값) |
| bulk sensor 응답 MODE 검증 (`VerifiesBulkSensorResponseMode`) | strict | **off** (`VerifiesResponseMode` 상속) |
| motor-space read (0x10 kMotor) | 수행 → `motor_states` 발행 | 스킵 → `motor_states` 미발행 |
| `sensor_states` 채움 | baro/tof + F-T `f`/`u` | `f`={fx,fy,fz}만 (Lx/Ly/Temp 는 디코드만·발행 보류) · `inference_enable`=`true` 고정 |
| 지원 통신 모드 | individual / bulk | bulk 전용 |

1b 는 `communication_mode: "bulk"` 를 요구하며, individual 과 조합 시 `on_configure`
가 FAILURE 를 반환한다. Lx/Ly/Temp 는 현재 펌웨어에서 placeholder 이므로 디코드는
하되(`UdpHandState::sensor_force`) 토픽 발행은 보류한다 — 실제 데이터 도착 시
후속 PR 에서 `u`/신규 필드로 매핑.

### Dual Read (Motor + Joint 공간)

1a 는 매 사이클 모터 상태를 두 번 읽습니다:
- **kMotor (0x00)**: 모터 엔코더 값 -> `motor_positions/velocities/currents`
- **kJoint (0x01)**: 펌웨어 기어비 변환 -> `joint_positions/velocities/currents`

1a 의 write 명령은 `kJoint` 모드로 전송됩니다.

**1b**: motor-space read (kMotor 스텝) 를 스킵하고 joint-space read 를 **kMotor** 로 한 번만
수행합니다 (1b 펌웨어의 유일한 joint-serving 모드). write 도 kMotor 로 보냅니다. joint I/O MODE 는
하드코딩이 아니라 `SensorProtocol::JointIoMode()` 로 주입되며, `Start()` 가 이를 한 번 hoist 해
write / joint read / E-Stop zero-write 에 사용합니다.

### Sensor Decimation

`sensor_decimation` 값(ROS param, 기본 `1`)에 따라 comm 사이클 N번 중 1번만 센서를 읽습니다. motor/joint read 는 매 사이클 유지되며, 유효 센서 레이트 = `loop_rate_hz / (comm_decimation * sensor_decimation)` 입니다.

### 통신 Decimation (`comm_decimation`)

UDP 통신 부하를 강제로 줄여 테스트하기 위한 knob (ROS param, 기본 `1`). `sensor_decimation` 이 **센서 read 만** 감쇠하는 것과 달리, `comm_decimation` 은 CommLoop 사이클의 **전체 UDP 트랜잭션 (write + 모든 read)** 을 감쇠한다:

- `1`: 매 사이클 통신 (기존 동작과 bit-identical, 회귀 0)
- `2`: 1 사이클 통신, 1 사이클 skip (통신 안 함)
- `3`: 1 사이클 통신, 2 사이클 skip

skip 사이클의 동작:
- UDP send/recv 없음, 상태 publish 없음, cycle 카운트에 포함 안 됨 (진짜 no-op tick). stale 상태를 재발행하지 않으므로 downstream 이 감쇠된 rate 를 실제로 관측한다 (부하 테스트의 목적).
- **E-Stop 은 skip 여부와 무관하게 매 사이클 검사** (안전 불변).
- **첫 사이클은 항상 통신** (초기 상태 read 보장 — `comm_cycle_counter_` 를 `comm_decimation_ - 1` 로 seed).
- 최신 명령은 `event_pending_` 에 persist → skip 사이클이 삼키지 않고 다음 통신 사이클이 latch·전송 (명령 유실 없음, last-wins).
- `any_recv_ok` / `consecutive_recv_failures` 를 건드리지 않아 link-down false-positive 없음.

> ℹ️ **failure detector 상호작용** — 유효 통신 rate = `loop_rate_hz` / `comm_decimation` (rate 검사는 `cycle_count` 증가분을 보며, skip 사이클은 카운트되지 않는다). `on_activate` 가 `min_rate_hz` 를 `comm_decimation` 으로 **자동 스케일**(`min_rate_hz /= comm_decimation`)하므로, 높은 감쇠에서도 rate failure 오탐 → E-STOP 이 발생하지 않는다 (별도 수동 조정 불필요). 마찬가지로 `link_status` publish 감쇠도 유효 state-publish rate = `loop_rate_hz` / `comm_decimation` 기준으로 계산된다.

감쇠량은 `hand_udp_stats.json` 의 `comm_decimation` / `comm_decimation_skip_count` 로 확인한다.

### ONNX F/T 추론

센서 사이클에서 `FingertipFTInferencer`를 통해 핑거팁별 접촉/힘 추론을 수행합니다:

1. **캘리브레이션**: 시작 시 N 샘플로 barometer baseline offset 자동 측정
2. **전처리**: barometer 정규화 + delta 계산 + FIFO history shift (12 row)
3. **추론**: per-fingertip ONNX 모델. ONNX 실행은 `rtc::OnnxEngine` (single-input / 3-output) 에 위임, zero-alloc
4. **출력**: contact probability (sigmoid), force vector (3), direction vector (3)

> ⚠️ **컨트롤러 capability 일치 의무** — 컨슈머 (integrated_bringup) 의 device YAML
> `devices.<hand_group>.sensor_layout.has_native_contact` / `has_native_displacement`
> 두 bool 은 본 inferencer 의 `ft_inferencer.enabled` 와 lockstep 유지해야 한다.
> Inferencer 가 꺼진 채 capability=true 로 두면 컨트롤러가 항상 0 인 contact
> probability 를 native signal 로 오인하여 grasp detection 이 silently 무력화된다.

---

## 주요 컴포넌트

### UdpHandController (`udp_hand_controller.hpp`)

핵심 드라이버 클래스. Event-driven jthread (hand_driver core, SCHED_FIFO/65) 로 동작합니다. 코어 번호는 tier-aware (`rtc_base/threading/thread_config.hpp::SelectThreadConfigs().hand_driver.cpu_core` — 6-core 에서 Core 1, ≥ 8-core 에서 dedicated; SSoT 참조). 프로세스가 **스스로** main 스레드를 그 코어에 pin 하고 (issue #345 — launch 의 `taskset -a` 스윕은 제거됐다) 내부 receive thread (priority 65) 가 affinity 상속. launch 는 `rclcpp::init()` 이 노드 생성 전에 만드는 DDS 스레드만 co-pin 한다.

- **SeqLock** 기반 lock-free 상태 공유 (priority inversion 방지)
- **per-command write gate**: `event_pending_`(release) → CommLoop latch(acquire); 1회 write 후 clear (stale 재전송 없음)
- **Write echo 항상 수신**: 소켓 버퍼 오염 방지
- **첫 사이클 read-only**: 초기 상태를 모르는 상태에서 zero 명령 전송 방지
- **Fake hand 모드**: `sil_mode=loopmodel` (노드가 파생하는 내부 플래그 `use_fake_hand=true`) 시 UDP 소켓 없이 **CommLoop RT 스레드는 실제 모드와 동일하게** self-clock(`loop_rate_hz`)하고, command pose 를 1차 LPF 모델(`RunFakeCommCycle`/`StepFakeModel`)에 통과시켜 read joint state(pos/vel/effort)를 생성한다 (ros2_control `use_fake_hardware:=true` 등가 — 제어 PC 의 thread 할당/RT 구조 검증용). LPF/effort 는 `fake_lpf_time_constant_s`·`fake_effort_stiffness`·`fake_effort_damping` 로 튜닝

### UdpHandTransport (`udp_hand_transport.hpp`)

저수준 UDP 소켓 관리. `ppoll()` 기반 sub-ms 수신 타임아웃 (hrtimer on PREEMPT_RT). ppoll timeout timespec 은 `recv_timeout_ms_` 가 소켓 수명 동안 고정이므로 `Open()` 에서 1회 계산(`recv_timeout_ts_`)해 hot path `RecvWithTimeout` 의 per-call 연산을 없앤다.

**Connected socket (sub-ms RTT 최적화)**: `Open()` 은 소켓을 단일 hand peer 로 `connect()` 한다. 이로써 (1) 전송이 `sendto`→`send` 로 바뀌어 커널이 라우트를 캐시하고 per-send destination lookup·주소 복사를 생략하며, (2) recv 큐가 해당 peer 발 datagram 으로 커널 레벨 필터링되어 잡음 source 가 `cmd_mismatch` 로 집계되지 않고, (3) dead peer 의 ICMP port-unreachable 이 다음 send/recv 에서 `ECONNREFUSED` 로 표면화되어 recv timeout 을 기다리지 않는 결정적 link-down 신호가 된다 (`CountRecvFail` 이 EAGAIN 이 아닌 이 errno 를 `recv_error` 로 분류 → timeout 과 구분). UDP `connect()` 는 handshake 없는 non-blocking 이라 off-RT `Open()` 에서 안전. 추가로 sub-ms 루프용 저지연 sockopt 을 best-effort(WARN-continue, `SO_RCVBUF` 와 동일 정책)로 설정: `SO_BUSY_POLL`(+`SO_PREFER_BUSY_POLL`) 로 recv 시 NIC busy-poll(IRQ/softirq+wakeup latency 제거, `CAP_NET_ADMIN` 필요), `IP_TOS=IPTOS_LOWDELAY`·`SO_PRIORITY` 로 공유 NIC egress 우선.

**Mode 검증**: request-response 메서드(`RequestMotorRead`, `RequestAllMotorRead`, `RequestSensorRead`, `RequestBulkSensorRaw`)는 응답 패킷의 mode 필드가 요청한 mode와 일치하는지 검증합니다. 불일치 시 `comm_stats_.mode_mismatch` 카운터를 증가시키고 `false`(bulk raw 는 `-1`)를 반환합니다. 이 검증은 **두 개의 독립 gate** 로 나뉩니다: joint / motor / set-mode 경로는 `verify_response_mode_`(컨트롤러가 `SensorProtocol::VerifiesResponseMode()` 로 주입), bulk sensor(0x19) 경로는 `verify_bulk_sensor_mode_`(`SensorProtocol::VerifiesBulkSensorResponseMode()` 로 주입)로 gate 됩니다. 1a 는 두 gate 모두 strict(true). 1b 는 **두 gate 모두 off** — 1b 펌웨어가 bulk sensor 응답뿐 아니라 motor/joint read(`ReadAllMotors`) 응답에서도 MODE byte 를 임의값으로 echo 하기 때문입니다. joint read 만 strict 로 두면 kMotor joint read 가 매 사이클 mode_mismatch 로 폐기되어 `state.joint_valid=false` → `/hand/joint_states` 미발행 → 컨트롤러 device 1 이 valid 가 못 되고 InitPositionHold 의 deferred hand-seed 가 발화하지 못해 손가락이 0 으로 붕괴합니다. `VerifiesBulkSensorResponseMode` 는 default(`VerifiesResponseMode`)로 off 를 상속합니다. gate 와 무관하게 모든 read 경로는 cmd·length 검증이 유지됩니다(stale/wrong-command 패킷은 계속 거부). `RequestSetSensorMode` 도 MODE gate 와 무관하게 항상 cmd echo(`kSetSensorMode`)를 검증합니다 (cmd floor).

**Per-request-kind 통계**: `UdpHandCommStats.per_kind[]` 가 request kind (`RequestKind` enum: write_echo / motor_read / joint_read / sensor_read / bulk_sensor / set_mode) 별 `{ok, timeout, error, cmd_mismatch, mode_mismatch, short_or_decode}` 를 기록한다. joint/motor read 는 같은 메서드·cmd byte 를 쓰므로 caller(controller) 가 `RequestKind` 파라미터로 명시한다. 거부된 최신 패킷의 `last_unexpected_cmd`(CMD byte)/`last_unexpected_len` 도 보존 — link-down 시 timeout vs stale-desync 판별용. hot path 에는 카운터 증가만 추가 (로깅/alloc 없음). 스키마 상세는 아래 "통계 저장" 절.

**Cycle-start stale drain**: `DrainStaleDatagrams()` 는 매 comm cycle 첫 request 직전에 소켓에 남은 datagram 을 비운다. 1-cycle desync (직전 request 의 응답이 그 `RecvWithTimeout` 만료 후 도착) 로 큐에 남은 stale 패킷을 다음 cycle 첫 read 가 `cmd_mismatch` 로 소비하고 retry 를 소모하는 문제를 제거한다. RT hot path 이므로 `kMaxDrainPerCall`(8) 회 non-blocking `recv(MSG_DONTWAIT)` 로 **bounded** — 잔여 backlog 은 다음 cycle 이 회수한다. drain 개수는 `UdpHandCommStats.stale_drained` (aggregate — pre-request 단계라 `per_kind` 귀속 불가) 에만 누적되고 hot path 로깅은 없다. fake mode(소켓 미오픈) → no-op(0). controller `RunCommCycle` 은 decimation skip return 직후·command latch 직전에 호출한다.

**Per-request cmd-mismatch blocking retry**: drain 이후에도 request 응답이 stale echo 로 밀려 도착할 수 있으므로, `RequestWithRetry`(및 `WritePositionWithEcho`)는 응답 CMD 가 요청 CMD 와 불일치(`cmd_mismatch`, = stale echo/1-cycle desync)하면 **1회 blocking `RecvWithTimeout` 을 더 수행**해 이 request 자신의 응답을 기다린 뒤, 그래도 불일치면 fail 한다 (`ValidateResult::kRetryBlocking`, `kMaxBlockingRetries=1`). stale 소비 → 신규 대기로 파이프라인 slip 이 1회에 재정렬된다. `short_or_decode`(부분 수신·codec 실패)는 desync 가 아니라 손상이므로 기존대로 `MSG_DONTWAIT` non-blocking drain(`kRetry`)만, MODE mismatch 는 즉시 fail(`kFail`, 재시도 없음). worst-case 는 mismatch 발생 request 당 recv_timeout 1회 추가(0.4ms @p1b)로 bounded — 펌웨어 응답 지연 < recv_timeout 인 정상 조건에서만 회복하고 그 이상이면 correctly fail 한다.

### UdpHandSensorProcessor (`udp_hand_sensor_processor.hpp`)

센서 후처리 파이프라인 (noexcept):
- `PreFilter()`: Rate estimator tick + 지연 필터 재초기화
- `ApplyFilters()`: Bessel 4차 LPF in-place (barometer + ToF)
- `DetectDrift()`: OLS 기반 원-샷 드리프트 감지 (1Hz 스로틀 경고)

### FingertipFTInferencer (`fingertip_ft_inferencer.hpp`)

Per-fingertip ONNX 모델 기반 힘/토크 추론 (3-head output):
- Input: `float32[1, H, 16]` (barometer 8ch + delta 8ch)
- Output: contact logit(1) + F(3) + u(3)
- ONNX session/IoBinding/tensor/warmup 은 `rtc::OnnxEngine` 이 소유 (이 클래스는 history/정규화/calibration/post-proc 만 담당)
- `HAS_ONNXRUNTIME` 미정의 시 stub 구현 (추론 비활성)

### UdpHandFailureDetector (`udp_hand_failure_detector.hpp`)

50Hz non-RT jthread로 동작하는 장애 감지기:
1. All-zero 데이터: N회 연속 감지
2. Duplicate 데이터: N회 연속 반복
3. Low rate: polling rate가 `min_rate_hz` 미만
4. Link down: `consecutive_recv_failures` >= `link_fail_timeout_ms` 환산 cycle 수 (`ms/1000 × loop_rate_hz / comm_decimation`) — threshold 도달 시 **1회 forensic dump** (per-request-kind 통계 테이블 + 최근 64 cycle 의 attempted/ok mask ring 디코드) 를 WARN 으로 출력 후 RaiseFailure. 기동 직후 `startup_grace_ms`(ROS param, 기본 1000 ms) 구간에는 rate/link 판정을 유예해 ARP/부팅/첫 왕복 지연이 오탐 E-STOP 을 내지 않게 한다 (motor/sensor data-validity 검사는 유예 대상 아님)

센서 검사 대상은 프로토콜 레이아웃에 따른다 (`sensor_force_layout` capability, 컨트롤러가 `sensor_uses_force_layout_` 주입). 1a 는 int32 `sensor_data` (barometer/ToF) 전체를, 1b 는 `sensor_force` 의 **fx,fy,fz 만** 검사한다 — Lx/Ly/Temp placeholder 는 제외해 상수 placeholder 가 all-zero 를 가리거나 duplicate 를 왜곡하지 못하게 한다. 펌웨어가 rest 에서 ADC 노이즈로 non-zero 를 내므로 exact-0 force = dead/단선 센서, bit-frozen fx,fy,fz = stalled feed 로 정상 무접촉과 구별된다. 모터 검사는 `state.motor_valid` 로 gate 되어 motor-space read 가 없는 1b (`motor_valid` 항상 false) 에서는 no-op 이다 (`check_motor: false` 권장).

### UdpHandTimingProfiler (`udp_hand_timing_profiler.hpp`)

EventLoop 단계별 소요시간 추적. 히스토그램 기반 p95/p99 백분위수, 예산(2000us) 초과 카운트. `TimingProfilerBase<250, 20, 2000>` 상속 — 250개 버킷 × 20 µs (= [0, 5000) µs 범위, 20 µs 백분위 해상도). 보간 결과는 `max_us`로 clamp되어 p95, p99 ≤ max를 보장.

### UdpHandTimingLogger (`udp_hand_timing_logger.hpp`)

CommLoop per-tick 타이밍을 `<session>/timing/hand_udp_timing_log.csv` 로 기록한다. CM (`cm_timing_log.csv`) 및 MPC (`mpc_timing_log.csv`) 와 동일한 통합 스키마 (`t_wall_ns, tick_count, t_state_us, t_compute_us, t_publish_us, t_total_us, jitter_us`) 를 사용 — `rtc_base/timing/rt_tick_timing_sample.hpp` 의 `RtTickTimingPayload` 직접 재사용.

데이터 흐름:
- producer: `RunCommCycle` 이 UDP read 직후 `MarkState()`, sensor 후처리+FT 직후 `MarkCompute()` 를 호출하면 `rtc::PeriodicRtThread` (CommLoop) 기반이 매 tick 1개 `RtTickTimingPayload` 를 `HandUdpTimingBuffer` 에 push (RT-safe, wait-free)
- drain: `udp_hand_node` 가 1 Hz `wall_timer` 로 SPSC 버퍼를 비우고 `ThreadTimingCsvLogger<RtTickTimingPayload>` 에 row 추가
- jitter: CommLoop 이 `loop_rate_hz` period budget 대비 `|actual_period − budget|` 을 자동 계산

phase 매핑 (hand UDP loop, `MarkState()`/`MarkCompute()` 브레이크포인트):
- bulk: `t_state = write+read (t0→t3)`, `t_compute = sensor 후처리+FT`, `t_publish = state store+callback`
- individual: `t_state = write+read (t0→t4)`, `t_compute = sensor 후처리+FT`, `t_publish = state store+callback`
- 내부 phase breakdown (`UdpHandTimingProfiler`) 은 별도 local t0~t5/t6 timestamps 로 유지 (write/read 세부 항목별).

`UdpHandTimingProfiler` 와 공존 — Profiler 는 in-process p95/p99 텔레메트리, Logger 는 raw per-tick CSV 로그.

### 통계 저장 (`hand_udp_stats.json`)

`<session>/device/hand_udp_stats.json` 에 통신·타이밍 통계를 저장한다. 저장 시점:

- **주기 저장**: activate 후 10 s 고정 상수 wall timer (ROS param 아님) — SIGKILL/전원 유실에도 최근 10 s 이내 상태 유지 (quiet, INFO 로그 없음)
- **failure 시점**: failure detector 콜백에서 즉시 1회 (E-STOP teardown 전에 forensic 카운터 보존)
- **정상 teardown**: deactivate/cleanup/error/소멸자

스키마 (`comm_stats` 섹션):

- 집계 필드: `total_cycles`, `recv_ok`, `recv_timeout`, `recv_error`, `cmd_mismatch`, `mode_mismatch`, `comm_decimation`, `comm_decimation_skip_count`, `stale_drained` (cycle-start drain 이 버린 datagram 누계), `avg_rate_hz`, `consecutive_recv_failures`, `link_ok`, `failure_detected` 등 (`total_cycles`/`avg_rate_hz` 는 통신 사이클만 카운트 — comm_decimation skip 은 제외)
- **`per_request`**: request kind (`write_echo` / `motor_read` / `joint_read` / `sensor_read` / `bulk_sensor` / `set_mode`) 별 `{ok, timeout, error, cmd_mismatch, mode_mismatch, short_or_decode}`. `ok` 는 request-level 성공 (검증 통과), `short_or_decode` 는 short packet + codec decode 실패. joint/motor read 는 wire format 이 동일하므로 controller 가 `RequestKind` 파라미터로 명시 attribution
- **`last_unexpected_cmd` / `last_unexpected_len`**: 가장 최근 거부된 패킷의 CMD byte / 수신 길이 — 직전 request 의 cmd echo 가 찍히면 timeout 이 아니라 1-cycle stale desync 시그니처

`timing_stats` 섹션은 `UdpHandTimingProfiler` 요약 (mean/min/max/p95/p99, phase 별). **성공한 패킷의 latency 만 집계한다** — 어떤 phase (`write` / `read_pos` / `read_vel` / `read_sensor` / bulk 의 `read_all_*`) 의 request 가 timeout/error/mismatch 로 실패하면 그 phase 의 sample 은 제외되고 (recv-timeout 값이 분포를 오염시키지 않도록), `count` / `over_budget` / percentile 을 좌우하는 `total_us` 는 그 cycle 에서 **attempt 한 모든 채널이 성공했을 때만** 기록된다. 시도하지 않은 phase (pending write 없음, 1b motor-space read, 비-sensor cycle) 도 제외. 순수 연산 phase (`sensor_proc` / `ft_infer`) 는 packet 과 무관하므로 항상 집계. 제외된 실패 자체의 카운트는 위 `comm_stats.per_request` (RequestKind 단위) 에서 확인한다 (`timing_stats.count` < `comm_stats.total_cycles` 는 실패 cycle 존재를 의미).

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
| `/hand/joint_command` | `rtc_msgs/JointCommand` | BEST_EFFORT/1 | 모터 명령 (values[10]). 발행측 `joint_names` 순서로 수신해 firmware 슬롯 순서(`joint_state_names`)로 재정렬 후 적용 — 이름 누락/미매칭 시 positional fallback |
| `/hand/calibration/command` | `rtc_msgs/CalibrationCommand` | RELIABLE/1 | 센서 캘리브레이션 트리거 (현재 barometer 지원) |

토픽 이름은 YAML 파라미터(`command_topic`, `joint_state_topic`, `calibration_command_topic` 등)로 변경 가능합니다.

### 센서 캘리브레이션 재트리거

`baseline_offset` 을 런타임에 재측정하고 싶을 때 `/hand/calibration/command` 에
`CalibrationCommand` 를 publish 합니다. 드라이버는 CommLoop 스레드에서 요청을
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

또는 `integrated_bringup/scripts/demo_controller_gui.py` 의 **Control 탭 → Sensor
Calibration** 패널에서 `Calibrate` 버튼 클릭으로 동일하게 트리거할 수 있습니다.

**새 센서 추가 시** (확장성):
1. `rtc_msgs/msg/CalibrationCommand.msg` 의 `SENSOR_*` enum 에 값 추가
2. `UdpHandController::DispatchCalibrationRequest()` switch 에 case 추가
3. `UdpHandController::GetCalibrationStatus()` 에 분기 추가
4. `demo_controller_gui.py` 의 `SENSOR_CALIBRATIONS` 리스트에 dict 한 줄 추가

---

## 설정

### `config/udp_hand_node.yaml`

노드가 선언하고 읽는 파라미터:

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `target_ip` | `"192.168.1.2"` | 핸드 컨트롤러 IP |
| `target_port` | `55151` | 핸드 컨트롤러 포트 |
| `recv_timeout_ms` | `10.0` | ppoll 수신 타임아웃 (ms, sub-ms 지원) |
| `loop_rate_hz` | `500.0` | self-clocked CommLoop 주기 (Hz) — read/state publish 자율 tick rate |
| `publish_rate` | `100.0` | link_status decimation 기준 (Hz, `loop_rate_hz / publish_rate` 비율) |
| `communication_mode` | `"individual"` | `"individual"` 또는 `"bulk"` |
| `comm_decimation` | `1` | 전체 UDP 트랜잭션 감쇠 (위 "통신 Decimation" 참조) |
| `sensor_decimation` | `1` | 센서 read 감쇠 (위 "Sensor Decimation" 참조) |
| `protocol_version` | `"1a"` | 센서 프로토콜 버전 (`"1a"`/`"1b"`, 위 "센서 프로토콜 버전" 참조) |
| `baro_lpf_enabled` | `false` | Barometer LPF 활성화 |
| `baro_lpf_cutoff_hz` | `30.0` | Barometer LPF 차단 주파수 |
| `tof_lpf_enabled` | `false` | ToF LPF 활성화 |
| `tof_lpf_cutoff_hz` | `15.0` | ToF LPF 차단 주파수 |
| `drift_detection_enabled` | `false` | OLS 드리프트 감지 |
| `drift_threshold` | `5.0` | 드리프트 기울기 임계값 |
| `drift_window_size` | `2500` | 드리프트 윈도우 크기 |
| `joint_state_names` | `[]` (기본 이름 사용) | 관절 이름 배열 (10개) |
| `joint_position_offsets_deg` | `[]` (offset 0) | 관절 position offset (degree, `joint_state_names` 순서 10개). 수신 시 `+offset`(펌웨어→controller), 송신 시 `−offset`(controller→펌웨어). 크기 불일치 시 무시(0). position 전용 |
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
| `link_fail_timeout_ms` | `100.0` | link-down 판정 시간 budget. cycles = ms/1000 × loop_rate_hz / comm_decimation 로 on_configure 에서 환산 |
| `startup_grace_ms` | `1000.0` | 기동 직후 rate/link 판정 유예 시간 (ms, "UdpHandFailureDetector" 절 참조) |
| `link_startup_grace_ms` | `100.0` | 기동 직후 link-down 검사 유예 (ms) — ARP/부팅 transient 만 넘기는 짧은 유예 |
| `sil_mode` | `off` | SIL 진입점 (아래 "SIL 모드"). `off`=실 HW / `loopmodel`=controller-side in-process LPF (소켓 미오픈) / `firmware`=device-side loopback (노드가 `target_ip`→127.0.0.1 강제). 노드 `on_configure` 가 해석해 `use_fake_hand`·`target_ip` 파생; launch `sil_mode:=` 가 override |
| `fake_lpf_time_constant_s` | `0.1` | (fake) 1차 LPF 시정수 τ [s] — read-back position 이 command 를 추종하는 지연 |
| `fake_effort_stiffness` | `1.0` | (fake) effort kp: `kp·(cmd−pos)` |
| `fake_effort_damping` | `0.1` | (fake) effort kd: `−kd·vel` (PD-torque placeholder) |

> 참고: `joint_mode`는 사용되지 않습니다 (코드에서 항상 write=kJoint, read=kMotor+kJoint dual read).

### `config/fingertip_ft_inferencer.yaml`

F/T 추론 설정. `udp_hand_node`에서 `ft_inferencer.*` 네임스페이스로 읽습니다.
이 파일은 standalone `udp_hand.launch.py` 의 **예시(enabled:false)** 이며, per-robot
튜닝본은 `integrated_bringup/config/ur5e_p1a/fingertip_ft_inferencer_p1a.yaml` 에 둔다.
p1b 는 이 파일을 넘기지 않아 노드의 `ft_inferencer.enabled=false` 기본값으로 추론이
자동 비활성된다.

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

모델 경로가 상대 경로인 경우 `udp_hand_driver` 패키지의 `models/` 디렉토리 기준으로 해석됩니다.

---

## 실행

### 단독 실행 (standalone)

```bash
ros2 launch udp_hand_driver udp_hand.launch.py \
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
| `loop_rate_hz` | `500.0` | self-clocked CommLoop 주기 (Hz) — read/state publish 자율 tick rate |
| `publish_rate` | `100.0` | link_status decimation 기준 (Hz, `loop_rate_hz / publish_rate` 비율) |
| `communication_mode` | `bulk` | `"individual"` 또는 `"bulk"` |
| `recv_timeout_ms` | `0.4` | ppoll 수신 타임아웃 (ms) |
| `protocol_version` | `1a` | `"1a"` (int32 baro/ToF, bulk 259B) 또는 `"1b"` (float force, bulk 99B). 노드+firmware 양쪽 전파 |
| `sil_mode` | `off` | SIL 진입점 (아래 "SIL 모드" 참조): `off` (실 HW) / `loopmodel` (controller-side LPF) / `firmware` (device-side loopback) |

> launch 인자는 모두 yaml 값의 **override** 다 — 값을 주지 않으면(빈 문자열) 해당
> yaml 파라미터를 그대로 쓴다(입력 config 는 `config/udp_hand_node.yaml` 1개 + 예시
> ft yaml). 위 '기본값' 은 generic yaml / 노드 C++ declare 기본값이다.

### SIL 모드 (`sil_mode`)

하드웨어 없이 두 상보적 SIL 계층을 **`sil_mode` 하나**로 선택한다. `sil_mode` 는
각 hand yaml(`udp_hand_node*.yaml`)의 정식 노드 파라미터(SSoT)이고, 노드
`on_configure` 가 이를 해석해 아래 표의 `use_fake_hand`(컨트롤러 config 내부 플래그)
와 실효 `target_ip` 를 파생한다. launch 인자 `sil_mode:=` 는 yaml 값을 override(우선).
단, `firmware` 프로세스 spawn 은 노드가 못 하므로 launch 가 `sil_mode`(yaml+CLI)를
읽어 `fake_hand_firmware` 노드를 조건부로 띄운다.

standalone `udp_hand.launch.py` 뿐 아니라 `integrated_bringup` 의
`robot_ur5e_p1a.launch.py` / `robot_ur5e_p1b.launch.py` 도 동일하게 `sil_mode:=` 를
지원한다 (각자의 per-robot yaml 이 default).

| `sil_mode` | 계층 | `use_fake_hand` (파생) | firmware | `target_ip` (파생) | 검증 대상 |
|------------|------|:---:|:---:|------|-----------|
| `off` (기본) | 실 하드웨어 | false | 미실행 | yaml 값 | — |
| `loopmodel` | Mode A (controller-side loop-model SIL) | true | 미실행 | 무관 (소켓 미오픈) | thread/RT 구조 (실 모드와 동일 self-clock) |
| `firmware` | Mode B (device-side network-level SIL) | false | 실행 | `127.0.0.1` (노드가 강제) | 전체 UDP transport (send/recv, framing, echo/MODE 검증, decode, publish) |

**`loopmodel`** — UDP 소켓을 우회하고 command 를 노드 in-process 1차 LPF 모델에
직접 통과시켜 read state(pos/vel/effort)를 생성한다. CommLoop RT 스레드는 실 모드와
동일하게 self-clock 하므로 제어 PC 의 thread 할당/RT 구조를 검증한다.

```bash
ros2 launch udp_hand_driver udp_hand.launch.py sil_mode:=loopmodel
ros2 topic echo /hand/joint_states --once     # LPF 추종 joint state 발행
```

**`firmware`** — `fake_hand_firmware` 실행파일이 hand 프로토콜의 *디바이스 측*
(펌웨어)을 시뮬레이션하고, 수정하지 않은 실제 `udp_hand_node`(`use_fake_hand=false`,
`target_ip=127.0.0.1`)가 loopback UDP 소켓으로 통신한다. firmware bind 포트는
`target_port`, wire 포맷은 `protocol_version` 으로 노드와 공유된다.

```bash
ros2 launch udp_hand_driver udp_hand.launch.py sil_mode:=firmware protocol_version:=1a
# 별도 터미널에서 링크/상태 확인
ros2 topic echo /hand/link_status --once      # data: true 면 전체 경로 정상
ros2 topic echo /hand/joint_states --once
```

firmware 합성 튜닝(LPF τ, effort 게인, 센서 base 대역/노이즈, seed)은
`config/fake_hand_firmware.yaml` 참조. 핵심 로직은 헤더 전용
`FakeHandFirmware` 클래스(`fake_hand_firmware.hpp`, 단위 test 대상)이고,
`fake_hand_firmware_main.cpp` 는 socket/param/log 껍데기다.

**한계:** loopback latency ~µs 라 `firmware` 모드는 timing 검증엔 부적합하다.
firmware 는 driver decoder 의 거울상(self-consistency)이라 driver↔실제 firmware
일치는 보장 못 하며 실기 Phase-5 가 ground truth. commander(controller/CM) 없이
단독 실행하면 firmware 모터가 LPF rest(0)에 머물러 failure detector 의
all-zero/duplicate 경고가 뜨는데, 이는 정상 동작(명령이 오면 해소)이다.

### 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select rtc_base rtc_communication rtc_inference rtc_msgs udp_hand_driver --symlink-install
source install/setup.bash
```

---

## 의존성

- `rclcpp` -- ROS2 C++ 클라이언트
- `rclcpp_lifecycle` -- LifecycleNode 기반 관리 상태 전환
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
| CPU 코어 | hand_driver core (tier-aware, `SelectThreadConfigs().hand_driver.cpu_core`); 내부 receive thread 는 process taskset 으로 affinity 상속 (`kHandUdpRecvConfig`, cpu_core=-1 sentinel) |
| 스케줄러 | `SCHED_FIFO` |
| 우선순위 | **65** (rt_callback 의 70 보다 한 단계 낮게 두어 hand 수신이 controller state 수신을 절대 선점하지 않도록 설계) |
| 메모리 잠금 | `mlockall(MCL_CURRENT \| MCL_FUTURE)` |

- 모든 코덱 함수: `noexcept`, 힙 할당 없음
- 고정 크기 배열만 사용 (`std::array`)
- `trivially_copyable` 패킷 구조체 (`#pragma pack`, `static_assert` 검증)
- **SeqLock** 기반 lock-free 상태 공유
- **printf 제거** -- CommLoop (SCHED_FIFO) 스레드에서 stdout 출력 없음
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

- `UdpHandController::RunCommCycle`, `UdpHandTransport::Send/Recv`, `UdpHandSensorProcessor::PreFilter/ApplyFilters`, `FingertipFTInferencer::Infer` 는 모두 **`loop_rate_hz` (기본 500 Hz) UDP 폴링 hot path** 다. 정상 경로의 `INFO`/`WARN` 직접 호출은 **금지** — 반복될 수 있는 메시지는 반드시 `*_THROTTLE` 매크로를 사용한다.
- **`RCLCPP_*_ONCE` 금지**. `_ONCE` 도 첫 호출에서는 동일한 fmt 포맷 할당을 수행하므로 RT 안전이 아니며, 조건이 다시 참이 될 때 침묵해 버린다. 대신 `*_THROTTLE` 을 `kThrottleIdleMs` 와 함께 사용한다 (예: `UdpHandSensorProcessor::PreFilter` 의 BesselFilter 재초기화 실패 경고).
- **루프 기반 per-element 로그 금지.** RT 스레드에서 `for` 로 돌며 per-channel 로그를 내보내는 패턴은 1초 주기 throttle 을 만족하더라도 집계 1회로 축소해야 한다. 예: `UdpHandSensorProcessor::ThrottledDriftWarning` 은 플래그된 채널 수 + 첫 위반 id/slope 를 **단 1 개의 `WARN_THROTTLE`** 로만 내보낸다 (최대 길이 고정 → 절단 없음).
- **RT 핫패스의 포맷 인자 수를 최소화한다.** 엣지 트리거되는 링크 UP/DOWN 조차 arg 0~1개로 유지. 상세 데이터는 `SeqLock` 상태나 `CommStats`/`drift_result_` 구조체 쪽에 쌓아 두고, 필요한 쪽(`SaveCommStats`, 비-RT consumer)이 끌어가도록 한다.
- THROTTLE 주기는 매직넘버 대신 `udp_hand_logging.hpp` 의 표준 상수를 사용한다.
- Non-RT 경로(init/shutdown, `UdpHandFailureDetector`, `udp_hand_node` 콜백, `SaveCommStats`) 의 INFO/WARN 은 *최대한 풍부하게* 작성한다. 그렙 한 줄로 세션 결과를 진단할 수 있어야 한다: cycle/rate/ok%/timeout%/err% 등 비율과 실패 원인을 한 줄에 담는다 (`SaveCommStats` 요약 참고).
- 메시지 본문에 클래스 이름을 박아넣지 않는다. 서브-로거 이름이 곧 식별자다 (`hand.ctrl`).

### 서브-로거 네임스페이스

| 서브-로거 | 사용처 |
|-----------|--------|
| `hand.node` | `UdpHandNode` (ROS2 노드 lifecycle, 토픽 구독/발행, link 상태 전이) |
| `hand.ctrl` | `UdpHandController` (lifecycle + CommLoop, 캘리브레이션 dispatch) |
| `hand.udp` | `UdpHandTransport` (소켓 open/close, 센서모드 전환) |
| `hand.sensor` | `UdpHandSensorProcessor` (LPF init, drift detection, BesselFilter 재초기화) |
| `hand.fail` | `UdpHandFailureDetector` (50 Hz 워치독 스레드) |
| `hand.ft` | `FingertipFTInferencer` (ONNX 모델 로드, 캘리브레이션, inference 예외) |

### THROTTLE 주기 표준

`udp_hand_driver::logging` 네임스페이스에 정의된 상수만 사용한다 (`udp_hand_logging.hpp`):

| 상수 | 값 [ms] | 용도 |
|------|---------|------|
| `kThrottleFastMs` | 500 | UDP recv 실패 누적 등 빠른 진행 표시 |
| `kThrottleSlowMs` | 2000 | 캘리브레이션 dispatch, 일반 반복 경고 |
| `kThrottleIdleMs` | 10000 | BesselFilter 재초기화 실패, one-shot 전이 안전 그물 |
| `kThrottleHotMs` | 5000 | RT 핫패스 예외 경로 (FT inference exception) |

### 실시간 필터링 예시

```bash
# UDP 트랜스포트 레이어만 DEBUG 활성화
ros2 service call /udp_hand_node/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'hand.udp', level: 10}]}"

# 모든 hand.* 서브로거 동시에 끄기 (계층 매칭)
ros2 service call /udp_hand_node/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'hand', level: 50}]}"

# Failure detector 만 끄기 (false-positive 노이즈 제거)
ros2 service call /udp_hand_node/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'hand.fail', level: 50}]}"
```

콘솔 출력에 로거 이름을 표시하려면:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
```

---

## 라이선스

MIT License -- [LICENSE](../LICENSE)
