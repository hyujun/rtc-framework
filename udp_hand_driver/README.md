# udp_hand_driver

외부 10-DOF 손 컨트롤러와 ROS 2 토픽 사이의 UDP request-response 브리지 (lifecycle 노드) + SIL 용 `fake_hand_firmware`. **유형**: 노드 패키지

## 개요

`udp_hand_node` (`rclcpp_lifecycle::LifecycleNode`) 는 `CommLoop` (SCHED_FIFO 65) 가 `loop_rate_hz` 로 **자율 tick** 하며 손 펌웨어와 UDP 로 통신한다. read + state publish 는 명령과 무관하게 매 주기 수행하고, write 는 `/hand/joint_command` 가 도착한 주기에만 한다. 통합 구성에서는 [rtc_controller_manager](../rtc_controller_manager/README.md) 의 RT loop 가 active controller 가 있을 때만 명령을 publish 하므로 write 는 `control_rate` 로 돌고, deactivate 하면 write 만 멈춘다 (CM clock 과 hand clock 은 독립). 단독 실행에서는 토픽을 직접 구독한다.

- 소비자: [integrated_bringup](../integrated_bringup/README.md) (robot launch), 센서 → grasp 컨트롤러
- 첫 상태 read 가 끝나기 전에는 명령을 latch 만 하고 write 하지 않는다. activate 직후 손이 q=0 으로 끌려가지 않는다
- E-Stop 은 매 tick 검사하며 활성이면 CommLoop 이 스스로 종료하고 loop unwind 시 zero-write 를 1회 보낸다. 재시작 (`deactivate→activate`) 은 링크 · 통계 카운터를 리셋한다

### 스레드 모델

| 스레드 | 스케줄러 | 하는 일 |
|---|---|---|
| `hand_udp_recv` (CommLoop) | **SCHED_FIFO 65**, `hand_driver` slot | UDP I/O · 센서 처리 · ONNX 추론 · SeqLock store |
| main (executor) | CFS, `hand_driver` slot | publish (SeqLock 을 당겨서 발행), 명령 · calib 구독 |
| failure detector | CFS nice −5, 상속 | 50 Hz 감시 |
| `hand_aux_io` | CFS, aux slot (`aux_cpu_slot`) | 타이밍 CSV drain, stats JSON — blocking 파일 쓰기 |
| DDS | CFS, `hand_driver` slot | launch `pin_dds_threads_to_slot` 이 co-pin (robot launch 전용) |

publish 는 RT tick 이 호출하는 push 가 아니라 executor 가 `loop_rate_hz × 2` 로 SeqLock 을 폴링하는 pull 이다. mailbox 는 latest-wins 라 밀린 구간의 중간 샘플은 덮인다. `hand_udp_recv` 와 detector 는 `cpu_core=-1` 로 프로세스 affinity 를 상속하며, main 과 `hand_aux_io` 는 노드가 스스로 pin 한다.

> ⚠️ **실기 RT 배포에서는 `use_cpu_affinity:=false` 를 쓰지 말 것.** 이 스위치는 프로세스 내부 pin 2개만 끄고 스케줄링 정책은 그대로 둔다. pin 없는 FIFO 65 CommLoop 은 아무 코어로나 이주할 수 있고, `mpc_main` (FIFO 60) 이 핀된 코어에 올라가면 우선순위가 더 높아 **그 solve 를 선점**한다 (`rt_control` 90 · `rt_callback` 70 은 영향 없음). 개발 노트북 · 컨테이너 · cpuset 없는 CI 처럼 격리가 아예 없는 환경 전용이다.

이 패키지의 단독 launch 는 프로세스 내부 pin 만 하고 cset shield · DDS co-pin 은 하지 않는다. 실기 RT 구성은 `integrated_bringup` 의 robot launch 를 쓴다. 코어 배치 매트릭스는 [rtc_base/README.md](../rtc_base/README.md#스레딩-threading), 튜닝은 [rt-optimization.md](../docs/rt-optimization.md).

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다. 이 패키지와 직접 의존 패키지 (`rtc_base` · `rtc_communication` · `rtc_inference` · `rtc_msgs`) 를 `--tests` 로 빌드한 뒤 env 를 source 한다.

```bash
# 하드웨어 없이 (controller-side LPF 모델, 소켓 미오픈)
ros2 launch udp_hand_driver udp_hand.launch.py sil_mode:=loopmodel
ros2 topic echo /hand/joint_states --once

# 실제 손 (IP 는 필요한 것만 override)
ros2 launch udp_hand_driver udp_hand.launch.py target_ip:=192.168.0.100
ros2 topic echo /hand/link_status --once        # data: true 면 링크 정상
```

launch 는 노드를 띄운 뒤 **자동으로 configure → activate** 한다. 수동으로 다루려면 `ros2 lifecycle set /udp_hand_node deactivate` · `activate` 를 쓴다. 실제 손에 처음 붙일 때는 `/hand/link_status` 가 `true` 이고 `/hand/joint_states` 가 나오는지 먼저 확인하고, 안 되면 [문제 해결](#문제-해결) 의 표를 본다.

## 인터페이스

### launch 인자 (`udp_hand.launch.py`)

모든 인자는 기본값이 빈 문자열인 **override 전용** 이다. 주지 않으면 [config/udp_hand_node.yaml](config/udp_hand_node.yaml) 값을 쓴다 (아래 표의 "YAML 값").

| 인자 | YAML 값 | 설명 |
|---|---|---|
| `target_ip` | `192.168.0.100` | 손 컨트롤러 IP |
| `target_port` | `55151` | 손 컨트롤러 포트 |
| `local_ip` / `local_interface` | `""` / `""` | 제어 PC source address (bind) / egress NIC (`SO_BINDTODEVICE`) |
| `communication_mode` | `bulk` | `bulk` \| `individual` |
| `recv_timeout_ms` | `0.4` | ppoll 수신 타임아웃 [ms] |
| `publish_rate` | `100.0` | `link_status` decimation 기준 [Hz]. `loop_rate_hz` 는 launch 인자가 아니다 |
| `protocol_version` | `1a` | `1a` \| `1b`. 노드와 `fake_hand_firmware` 양쪽에 전파 |
| `sil_mode` | `off` | `off` \| `loopmodel` \| `firmware` ([SIL 모드](#sil-모드-sil_mode)) |
| `use_cpu_affinity` | `true` | 프로세스 내부 pin on/off ([스레드 모델](#스레드-모델) 의 경고). `aux_cpu_slot` 은 YAML 전용 |

### 토픽

| 방향 | 토픽 | 타입 | QoS | 설명 |
|---|---|---|---|---|
| pub | `/hand/joint_states` | `sensor_msgs/JointState` | BEST_EFFORT/1 | 관절 pos / vel / current (kJoint read) |
| pub | `/hand/motor_states` | `sensor_msgs/JointState` | BEST_EFFORT/1 | 모터 pos / vel / current (kMotor read, `1a` 만) |
| pub | `/hand/sensor_states` | `rtc_msgs/HandSensorState` | BEST_EFFORT/1 | 핑거팁 센서 + F/T 추론 (RT 구독자용) |
| pub | `/hand/sensor_states/monitor` | `rtc_msgs/HandSensorState` | RELIABLE/10 | 같은 데이터, 비-RT 구독자용 (BT · GUI) |
| pub | `/hand/link_status` | `std_msgs/Bool` | RELIABLE, TRANSIENT_LOCAL/1 | UDP 링크 상태 |
| pub | `/hand/calibration/status` | `rtc_msgs/CalibrationStatus` | RELIABLE, TRANSIENT_LOCAL/1 | 캘리브레이션 진행 · 완료 (5 Hz) |
| sub | `/hand/joint_command` | `rtc_msgs/JointCommand` | BEST_EFFORT/1 | 모터 명령 `values[10]`. 발행측 `joint_names` 순서를 firmware 슬롯 순서로 재정렬하고, 이름이 없거나 안 맞으면 positional |
| sub | `/hand/calibration/command` | `rtc_msgs/CalibrationCommand` | RELIABLE/1 | 센서 캘리브레이션 트리거 (현재 barometer) |

토픽 이름은 YAML 의 `command_topic` · `joint_state_topic` · `calibration_command_topic` 등으로 바꾼다. `sensor_states` 한 건의 센서 값과 F/T 값은 **같은 tick 이 보장되지 않는다** (F/T 가 최대 1 comm cycle 앞설 수 있음). 소비자는 `1 / loop_rate_hz` 의 pairing 허용오차를 둔다. `sensor_decimation > 1` 이면 F/T 가 마지막 센서 cycle 값이라 더 오래될 수 있다.

#### 센서 캘리브레이션 재트리거

```bash
ros2 topic pub --once /hand/calibration/command rtc_msgs/msg/CalibrationCommand \
    '{sensor_type: 0, action: 0, sample_count: 0}'    # barometer baseline 재측정 (기본 샘플 수)
ros2 topic echo /hand/calibration/status
```

재측정 중에는 `baseline_offset` 이 0 으로 초기화돼 F/T 추론이 일시 중지된다. `demo_controller_gui.py` 의 Control 탭 → Sensor Calibration 에서도 트리거한다.

### 통신 모드

`communication_mode` 가 한 cycle 의 UDP 트랜잭션을 정한다 (괄호는 cmd byte).

| 모드 | 순서 |
|---|---|
| `bulk` | write (0x01, kJoint) + echo → `ReadAllMotors` (0x10, kMotor) → `ReadAllMotors` (0x10, kJoint) → `ReadAllSensors` (0x19, `sensor_decimation` cycle 마다) |
| `individual` | write + echo → `ReadPosition` (0x11, kMotor · kJoint) → `ReadVelocity` (0x12, kMotor) → `ReadSensor0-3` (0x14-0x17, `sensor_decimation` cycle 마다) |

`1a` 는 motor 와 joint 공간을 모두 읽고 (`motor_states` + `joint_states`), write 는 kJoint 로 보낸다.

### 센서 프로토콜 버전 (`protocol_version`)

펌웨어 센서 응답의 wire 포맷 차이는 `SensorProtocol` (`protocol/sensor_protocol.hpp`) 로 격리된다. 사용자가 고를 것은 펌웨어에 맞는 버전이다.

| | `1a` (기본) | `1b` |
|---|---|---|
| bulk sensor 응답 | 259B (4 × 16 int32: barometer 8 + ToF 3 + reserved) | 99B (3B 헤더 + 4 × 6 float32) |
| 핑거당 데이터 | barometer[8] + ToF[3] | `[fx, fy, fz, Lx, Ly, Temp]` |
| 후처리 | LPF · drift · F/T 추론 | 없음 (force 는 firmware 계산) |
| joint I/O MODE | kJoint | **kMotor** (kJoint 요청에 joint 데이터 미응답) |
| 응답 MODE byte 검증 | strict | off (펌웨어가 임의값을 echo) |
| `motor_states` | 발행 | 미발행 (motor-space read 스킵) |
| `sensor_states` | baro · tof + F/T `f` / `u` | `f={fx,fy,fz}` 만 (Lx · Ly · Temp 는 디코드만) |
| 통신 모드 | individual / bulk | **bulk 전용** |

`1b` + `individual` 조합은 `on_configure` 가 FAILURE 를 반환한다.

### 통신 decimation (`comm_decimation`)

UDP 부하를 강제로 줄여 보는 테스트용 knob 이다 (기본 `1`). `sensor_decimation` 이 센서 read 만 감쇠하는 것과 달리 `comm_decimation = N` 은 CommLoop N 사이클 중 1번만 **전체 트랜잭션 (write + 모든 read)** 을 수행한다. skip 사이클은 UDP 도 publish 도 없는 no-op 이라 downstream 이 감쇠된 rate 를 실제로 본다.

- 첫 사이클은 항상 통신하고, E-Stop 은 skip 과 무관하게 매 사이클 검사한다. 명령은 유실 없이 다음 통신 사이클에 전송된다 (last-wins)
- 유효 rate: 통신 `loop_rate_hz / comm_decimation`, 센서 `loop_rate_hz / (comm_decimation × sensor_decimation)`. `on_activate` 가 `min_rate_hz` 를 `comm_decimation` 으로 자동 스케일하므로 rate failure 오탐은 없다
- 감쇠량은 `hand_udp_stats.json` 의 `comm_decimation` · `comm_decimation_skip_count`

### ONNX F/T 추론

`1a` 센서 cycle 에서 `FingertipFTInferencer` 가 핑거팁별 접촉 / 힘 추론을 한다: 시작 시 N 샘플로 barometer baseline 보정 → 정규화 + delta + FIFO history → per-fingertip ONNX (`rtc::OnnxEngine`) → contact probability, force(3), direction(3). 설정은 [config/fingertip_ft_inferencer.yaml](config/fingertip_ft_inferencer.yaml) (`ft_inferencer.*`). ORT `Run()` 의 heap 할당은 RT-1 조건부 수용이다 ([invariants-rationale.md](../docs/reference/invariants-rationale.md)).

> ⚠️ **capability lockstep.** 컨슈머의 device YAML `devices.<hand_group>.sensor_layout.has_native_contact` · `has_native_displacement` 는 `ft_inferencer.enabled` 와 함께 켜고 꺼야 한다. 추론이 꺼진 채 capability=true 이면 컨트롤러가 항상 0 인 contact probability 를 native signal 로 오인해 grasp detection 이 조용히 무력화된다.

> ⚠️ **텐서는 positional 로 바인딩된다.** `F` 와 `u` 는 둘 다 `[1,3]` 이라 head 순서가 바뀐 `.onnx` 를 검출하지 못한다 (힘과 방향이 조용히 뒤바뀜). 실모델을 받으면 `ros2 run rtc_inference rtc_inference_check <model>.onnx` 로 텐서 이름을 확인해 `fingertip_ft_inferencer.hpp` 의 `model_config.inputs` / `.outputs` 에 채운다 ([rtc_inference](../rtc_inference/README.md#텐서를-이름으로-바인딩하는-이유)). 현재 두 config 의 `model_paths` 는 전부 `""` 다.

### SIL 모드 (`sil_mode`)

하드웨어 없이 두 상보적 SIL 계층을 **`sil_mode` 하나**로 고른다. `sil_mode` 는 노드 파라미터 (YAML) 이고 launch 인자 `sil_mode:=` 가 override 한다. 노드 `on_configure` 가 이를 해석해 내부 플래그 `use_fake_hand` 와 실효 endpoint 를 파생한다. `fake_hand_firmware` 프로세스는 노드가 못 띄우므로 launch 가 `sil_mode` 를 읽어 조건부로 띄운다. `integrated_bringup` 의 `robot_ur5e_p1a.launch.py` · `robot_ur5e_p1b.launch.py` 도 같은 `sil_mode:=` 를 지원한다.

| `sil_mode` | 계층 | `use_fake_hand` | firmware | 네트워크 endpoint | 검증 대상 |
|---|---|:---:|:---:|---|---|
| `off` (기본) | 실 하드웨어 | false | 미실행 | YAML 그대로 | - |
| `loopmodel` | controller-side loop-model | true | 미실행 | 소켓 미오픈 | thread · RT 구조 (실 모드와 같은 self-clock) |
| `firmware` | device-side network SIL | false | 실행 | `target_ip`→`127.0.0.1`, `local_ip` · `local_interface` 해제 | 전체 UDP transport (framing · echo · MODE 검증 · decode · publish) |

**`loopmodel`**: UDP 소켓을 우회하고 command 를 in-process 1차 LPF 로 통과시켜 read state 를 만든다. 튜닝은 `fake_lpf_time_constant_s` · `fake_effort_stiffness` · `fake_effort_damping`.

**`firmware`**: `fake_hand_firmware` 가 hand 프로토콜의 디바이스 측을 흉내 내고, 수정하지 않은 `udp_hand_node` 가 loopback UDP 로 통신한다. firmware 의 bind 포트는 `target_port`, wire 포맷은 `protocol_version` 이다. 합성 튜닝 (LPF τ · effort 게인 · 센서 대역 · seed) 은 [config/fake_hand_firmware.yaml](config/fake_hand_firmware.yaml).

```bash
ros2 launch udp_hand_driver udp_hand.launch.py sil_mode:=firmware protocol_version:=1a
ros2 topic echo /hand/link_status --once     # 별도 터미널. data: true 면 전체 경로 정상
```

한계: loopback latency 가 µs 라 timing 검증에는 부적합하다. firmware 는 driver decoder 의 거울상이라 실제 펌웨어와의 일치는 보장하지 못한다. commander 없이 단독 실행하면 모터가 LPF rest(0) 에 머물러 failure detector 의 all-zero / duplicate 경고가 뜨는데, 명령이 오면 해소되는 정상 동작이다.

## 설정

[config/udp_hand_node.yaml](config/udp_hand_node.yaml) 이 launch 가 읽는 파일이고 키 설명은 파일 주석이 갖는다. 아래는 **이 YAML 에 실제로 들어 있는 값** 이다 (YAML 에 없는 키는 노드 기본값).

| 파라미터 | YAML 값 | 설명 |
|---|---|---|
| `target_ip` / `target_port` | `"192.168.0.100"` / `55151` | 핸드 컨트롤러 endpoint |
| `local_ip` / `local_interface` | `""` / `""` | `""` = 커널 선택 / 라우팅 테이블에 위임. 같은 subnet 에 NIC 이 둘 이상일 때만 설정 |
| `recv_timeout_ms` | `0.4` | ppoll 타임아웃. sub-ms 지원 |
| `communication_mode` | `"bulk"` | `"individual"` 도 가능. 노드 C++ 기본값은 `individual` 이므로 YAML 없이 띄우면 달라진다 |
| `loop_rate_hz` | `500.0` | CommLoop 주기 |
| `publish_rate` | `100.0` | `link_status` decimation 기준 (`loop_rate_hz / publish_rate`) |
| `comm_decimation` / `sensor_decimation` | `1` / `1` | 위 [통신 decimation](#통신-decimation-comm_decimation) · 센서 read 감쇠 |
| `protocol_version` | `"1a"` | `"1a"` \| `"1b"` |
| `baro_lpf_enabled` / `baro_lpf_cutoff_hz` | `true` / `100.0` | barometer 4차 Bessel LPF |
| `tof_lpf_enabled` / `tof_lpf_cutoff_hz` | `true` / `15.0` | ToF LPF |
| `drift_detection_enabled` | `false` | OLS 드리프트 감지 (`drift_threshold` 5.0, `drift_window_size` 2500) |
| `joint_state_names` / `motor_state_names` / `hand_fingertip_names` | YAML 에 10 / 10 / 4개 | 비우면 기본 이름 |
| `joint_position_offsets_deg` | (YAML 에 없음, 빈 배열 = 0) | `joint_state_names` 순서 10개 [deg]. 수신 시 `+offset`, 송신 시 `−offset`. 크기가 안 맞으면 무시 |
| `enable_failure_detector` | `true` | `failure_threshold` 5, `check_motor` · `check_sensor` · `check_link` true |
| `min_rate_hz` / `rate_fail_threshold` | `30.0` / `5` | polling rate 하한 / 연속 미달 횟수 |
| `link_fail_timeout_ms` | `100.0` | link-down 시간 budget. cycle 수는 `ms/1000 × loop_rate_hz / comm_decimation` (센서는 `/ sensor_decimation` 추가) |
| `startup_grace_ms` / `link_startup_grace_ms` / `sensor_startup_grace_ms` | `1000.0` / `100.0` / `0.0` | 기동 직후 rate / link / 센서 검사 유예 [ms] |
| `sil_mode` | `"off"` | [SIL 모드](#sil-모드-sil_mode) |
| `fake_lpf_time_constant_s` · `fake_effort_stiffness` · `fake_effort_damping` | `0.1` · `1.0` · `0.1` | `loopmodel` 전용 |
| `aux_cpu_slot` / `use_cpu_affinity` | `0` / `true` | `hand_aux_io` 가 붙을 slot (`-1` = pin 안 함) / 프로세스 내부 pin on-off. 이 둘만 노드 생성자에서 declare 된다 |

### `fingertip_ft_inferencer.yaml`

`ft_inferencer.*` 네임스페이스로 읽는 F/T 추론 설정이다. 이 파일은 standalone launch 의 **예시 (`enabled: false`)** 이고 per-robot 튜닝본은 `integrated_bringup/config/ur5e_p1a/fingertip_ft_inferencer_p1a.yaml` 에 있다. p1b 는 이 파일을 넘기지 않아 `ft_inferencer.enabled=false` 기본값으로 비활성이다.

| 파라미터 | 값 | 설명 |
|---|---|---|
| `ft_inferencer.enabled` | `false` | ONNX 추론 |
| `ft_inferencer.num_fingertips` / `history_length` | `4` / `12` | 핑거팁 수 / FIFO history |
| `ft_inferencer.model_paths` | `["","","",""]` | per-fingertip ONNX 경로. 상대 경로는 설치된 `share/udp_hand_driver/models/` 기준인데 현재 이 디렉토리는 저장소에 없으므로 **절대경로** 를 준다 |
| `ft_inferencer.calibration_enabled` / `calibration_samples` | `true` / `500` | baseline 자동 보정 (센서 cycle 기준) |
| `ft_inferencer.{thumb,index,middle,ring}_max` | `40000.0` × 16 | 핑거 정규화 최댓값 |

## 디렉토리

```
udp_hand_driver/
├── include/udp_hand_driver/
├── src/
├── config/
├── launch/
└── test/
```

include 의 각 헤더는 상단 주석에 역할과 동작 계약이 있다. `protocol/` 은 `SensorProtocol` 구현. 컨트롤러는 링크를 `HandComm` (`hand_comm.hpp`) 으로만 보며 기본 구현 `UdpHandComm` 이 UDP 를 감싼다 — 다른 링크 (CAN FD) 는 `SetHandComm()` 으로 주입한다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| `/hand/link_status` 가 `false`, recv timeout | IP · NIC 불일치, 포트 차단 | `target_ip` 확인 (YAML 은 `192.168.0.100`). `hand_udp_stats.json` 의 `recv_timeout` · `recv_error` 확인 |
| `sil_mode:=firmware` 인데 timeout 만 반복 | YAML 의 `local_interface` 가 실 NIC 에 소켓을 고정 | 노드가 `firmware` 모드에서 binding 을 해제하고 INFO 로 원래 값을 남긴다. 해당 NIC 이 없는 PC 에서는 `Open()` 이 ENODEV 로 실패한다 |
| failure detector 가 all-zero / duplicate 경고 | commander 없이 단독 실행 (SIL 모터가 0 에 머묾) | 명령을 보내면 해소된다 |
| `1b` 에서 `on_configure` FAILURE | `communication_mode: individual` 과 조합 | `communication_mode:=bulk` |
| F/T 가 항상 0, grasp 이 안 잡힘 | `ft_inferencer.enabled=false` 인데 capability=true | [capability lockstep](#onnx-ft-추론) |
| `recv_error` 가 증가 | 손 쪽 포트 닫힘 (ICMP port-unreachable → `ECONNREFUSED`) | 펌웨어 · 전원 · 케이블 확인. timeout 과 구분되는 결정적 link-down 신호다 |

### 통계 (`hand_udp_stats.json`)

`<session>/device/hand_udp_stats.json` 에 통신 · 타이밍 통계를 쓴다. 10 s 주기 저장 (SIGKILL 대비), failure 발생 즉시 1회, 정상 teardown 때 저장한다. `comm_stats` 에 cycle · recv ok / timeout / error · mismatch · `stale_drained` · `comm_decimation` 이 있고, `per_request` 는 request kind (`write_echo` · `motor_read` · `joint_read` · `sensor_read` · `bulk_sensor` · `set_mode`) 별 집계다.

`last_unexpected_cmd` 에 직전 request 의 cmd echo 가 찍히면 timeout 이 아니라 1-cycle stale desync 시그니처다. `timing_stats` 는 성공한 패킷의 latency 만 집계한다. per-tick 타이밍은 `<session>/timing/hand_udp_timing_log.csv`.

### 로깅 (Logging)

레벨 분류 · 공통 규칙은 [conventions.md §Logging](../agent_docs/conventions.md#logging). 패키지 규칙은 hot path (`RunCommCycle` · `UdpHandTransport::Send/Recv` · `PreFilter/ApplyFilters` · `FingertipFTInferencer::Infer`, 모두 `loop_rate_hz` 폴링) 에서 정상 경로의 직접 `INFO`/`WARN` 과 `RCLCPP_*_ONCE` 를 금지하고 `*_THROTTLE` 만 쓰는 것이다. THROTTLE 상수는 `udp_hand_logging.hpp` 가 SSoT.

| 서브-로거 | 사용처 |
|---|---|
| `hand.node` | `UdpHandNode` (lifecycle · 토픽 · link 상태 전이) |
| `hand.ctrl` | `UdpHandController` (CommLoop · 캘리브레이션 dispatch) |
| `hand.udp` | `UdpHandTransport` (소켓 open/close · 센서모드 전환) |
| `hand.sensor` | `UdpHandSensorProcessor` (LPF · drift) |
| `hand.fail` | `UdpHandFailureDetector` (50 Hz 워치독) |
| `hand.ft` | `FingertipFTInferencer` (모델 로드 · 캘리브레이션) |

```bash
ros2 service call /udp_hand_node/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'hand.udp', level: 10}]}"
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"   # 콘솔에 logger 이름 표시
```

## 더 읽기

- UDP 소켓 · drain · mode 검증 · retry: [udp_hand_transport.hpp](include/udp_hand_driver/udp_hand_transport.hpp) 주석
- mailbox · write gate · E-Stop self-exit: [udp_hand_controller.hpp](include/udp_hand_driver/udp_hand_controller.hpp) · [udp_hand_node.hpp](include/udp_hand_driver/udp_hand_node.hpp) 주석
- RT 튜닝: [rt-optimization.md](../docs/rt-optimization.md)

## 의존성

[package.xml](package.xml) · MIT. ROS 2 (rclcpp · lifecycle · sensor/std msgs) · rtc_base · rtc_communication · rtc_inference · rtc_msgs.
