# udp_hand_driver

외부 10-DOF 손 컨트롤러와 ROS 2 토픽 사이의 UDP request-response 브리지 (lifecycle 노드) + SIL 용 `fake_hand_firmware`. **유형**: 노드 패키지

## 개요

`udp_hand_node` (`rclcpp_lifecycle::LifecycleNode`) 는 `CommLoop` (SCHED_FIFO 65) 가 `loop_rate_hz` 로 **자율 tick** 하며 손 펌웨어와 UDP 로 통신한다. read + state publish 는 명령과 무관하게 매 주기 수행하고, write 는 `/hand/joint_command` 가 도착한 주기에만 한다. 통합 구성에서는 [rtc_controller_manager](../rtc_controller_manager/README.md) 의 RT loop 가 active controller 가 있을 때만 명령을 publish 하므로 write 는 `control_rate` 로 돌고, deactivate 하면 write 만 멈춘다 (CM clock 과 hand clock 은 독립). 단독 실행에서는 토픽을 직접 구독한다.

- 소비자: [integrated_bringup](../integrated_bringup/README.md) (robot launch), 센서 → grasp 컨트롤러
- 첫 상태 read 가 끝나기 전에는 명령을 latch 만 하고 write 하지 않는다. activate 직후 손이 q=0 으로 끌려가지 않는다
- E-Stop 은 `/hand/estop` (`true`) 로 건다. 매 tick 검사하며 활성이면 CommLoop 이 스스로 종료하고 loop unwind 시 zero-write (위치 0 목표, 정지 아님) 를 1회 보낸다. 노드는 ACTIVE 에 남고, 재시작 (`deactivate→activate`) 이 latch · 링크 · 통계 카운터를 리셋한다
- 두 손은 노드 하나로 돌린다 ([두 손](#두-손-hand_sides), `2` 전용)

### 스레드 모델

| 스레드 | 스케줄러 | 하는 일 |
|---|---|---|
| `hand_comm` (CommLoop) | **SCHED_FIFO 65**, `hand_driver` slot | 링크 I/O (손이 둘이면 tick 마다 목록 순서로) · 센서 처리 · ONNX 추론 · SeqLock store |
| main (executor) | CFS, `hand_driver` slot | publish (SeqLock 을 당겨서 발행), 명령 · calib 구독 |
| failure detector | CFS nice −5, 상속 | 50 Hz 감시 |
| `hand_aux_io` | CFS, aux slot (`aux_cpu_slot`) | 타이밍 CSV drain, stats JSON — blocking 파일 쓰기 |
| DDS | CFS, `hand_driver` slot | launch `pin_dds_threads_to_slot` 이 co-pin (robot launch 전용) |

publish 는 RT tick 이 호출하는 push 가 아니라 executor 가 `loop_rate_hz × 2` 로 SeqLock 을 폴링하는 pull 이다. mailbox 는 latest-wins 라 밀린 구간의 중간 샘플은 덮인다. `hand_comm` 와 detector 는 `cpu_core=-1` 로 프로세스 affinity 를 상속하며, main 과 `hand_aux_io` 는 노드가 스스로 pin 한다.

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
# 실제 손, CAN FD (실측 확정값 — can0 은 repo_scripts/scripts/setup_can_interface.sh; echo 펌웨어는 cmd_ack:=echo)
ros2 launch udp_hand_driver udp_hand.launch.py protocol_version:=2 hand_side:=right recv_timeout_ms:=0.9
ros2 topic echo /hand/link_status --once        # data: true 면 링크 정상
# 두 손, CAN FD (노드 하나 — /hand/right/* · /hand/left/*)
ros2 launch udp_hand_driver udp_hand.launch.py protocol_version:=2 hand_sides:=right,left \
    loop_rate_hz:=400 recv_timeout_ms:=1.2
```

launch 는 노드를 띄운 뒤 **자동으로 configure → activate** 한다. 수동으로 다루려면 `ros2 lifecycle set /hand/udp_hand_node deactivate` · `activate` 를 쓴다. 실제 손에 처음 붙일 때는 `/hand/link_status` 가 `true` 이고 `/hand/joint_states` 가 나오는지 먼저 확인하고, 안 되면 [문제 해결](#문제-해결) 의 표를 본다.

## 인터페이스

### launch 인자 (`udp_hand.launch.py`)

`namespace` 외의 인자는 기본값이 빈 문자열인 **override 전용** 이다. 주지 않으면 [config/udp_hand_node.yaml](config/udp_hand_node.yaml) 값을 쓴다 (아래 표의 "YAML 값").

| 인자 | YAML 값 | 설명 |
|---|---|---|
| `namespace` | — (launch 기본 `/hand`) | 노드 FQN · 상대 토픽 이름의 접두. 빈 값은 거부 |
| `target_ip` | `192.168.0.100` | 손 컨트롤러 IP |
| `target_port` | `55151` | 손 컨트롤러 포트 |
| `local_ip` / `local_interface` | `""` / `""` | 제어 PC source address (bind) / egress NIC (`SO_BINDTODEVICE`) |
| `communication_mode` | `bulk` | `bulk` \| `individual` |
| `recv_timeout_ms` | `0.4` | 수신 타임아웃 [ms] (UDP: ppoll · CAN FD: stage 별 deadline, 체인 최악 2 ×). CAN FD 실기는 `0.9` (H1-F06 실측) |
| `publish_rate` | `100.0` | `link_status` decimation 기준 [Hz] |
| `loop_rate_hz` | `500.0` | CommLoop 주기 [Hz]. 두 손은 `400` (`per_hand` 는 `250`) |
| `protocol_version` | `1b` | `1a` \| `1b` \| `2`. 노드와 `fake_hand_firmware` 양쪽에 전파 |
| `can_interface` | `can0` | `2` 의 SocketCAN 장치 (`firmware` 는 `vcan0` 로 강제) |
| `hand_side` | — (노드 기본 `right`) | `right` \| `left`. 명시하면 통계 · 타이밍 파일명에 `_<side>` 접미사. firmware 에도 전파 |
| `hand_sides` | — | 두 손: `right,left` (쉼표 목록). `hand_side` 와 함께 주면 거부. `firmware` 는 fake 를 손마다 (`fake_hand_firmware_<side>`) |
| `cmd_ack` | — (노드 기본 `none`) | `echo` \| `none`: `2` 펌웨어의 CMD echo 여부. firmware 에도 전파 |
| `req_mode` | `auto` | `auto` (두 손 · `cmd_ack none` 이면 `broadcast`, 그 밖 `per_hand`) \| `per_hand` \| `broadcast` ([두 손](#두-손-hand_sides)) |
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
| sub | `/hand/estop` | `std_msgs/Bool` | RELIABLE/1 | `true` 만 latch (`false` 무시). Inactive 중 수신분은 다음 activate 가 버린다. 통합 robot overlay 는 `/<robot>/estop` 을 명시하고, 시스템 E-STOP 에는 일부러 잇지 않는다 (근거: overlay 주석) |

토픽 이름은 YAML 의 `*_topic` 키로 바꾼다. `sensor_states` 한 건의 센서 값과 F/T 값은 **같은 tick 이 보장되지 않는다** (F/T 가 최대 1 comm cycle 앞설 수 있음). 소비자는 `1 / loop_rate_hz` 의 pairing 허용오차를 둔다. `sensor_decimation > 1` 이면 F/T 가 마지막 센서 cycle 값이라 더 오래될 수 있다.

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

| | `1a` | `1b` (기본) | `2` |
|---|---|---|---|
| transport | UDP | UDP | **CAN FD** (`can_interface` · `hand_side`) |
| bulk sensor 응답 | 259B (4 × 16 int32: barometer 8 + ToF 3 + reserved) | 99B (3B 헤더 + 4 × 6 float32) | RESP 3 프레임 (192 B set, joint 와 공용) |
| 핑거당 데이터 | barometer[8] + ToF[3] | `[fx, fy, fz, Lx, Ly, Temp]` | `[fx, fy, fz, px, py, Temp]` + contact |
| 후처리 | LPF · drift · F/T 추론 | 없음 (force 는 firmware 계산) | 없음 |
| joint I/O MODE | kJoint | **kMotor** (kJoint 요청에 joint 데이터 미응답) | kMotor (MODE byte 없음) |
| 응답 MODE byte 검증 | strict | off (펌웨어가 임의값을 echo) | 없음 |
| `motor_states` | 발행 | 미발행 (motor-space read 스킵) | 미발행 |
| `sensor_states` | baro · tof + F/T `f` / `u` | `f={fx,fy,fz}` 만 (Lx · Ly · Temp 는 디코드만) | `f={fx,fy,fz}` + `contact_flag` |
| `feedforward` | 무시 | 무시 | 전송 (Frame 1, `feedforward_kt` 로 Nm → A) |
| 통신 모드 | individual / bulk | **bulk 전용** | **bulk 전용** |

`1b` · `2` + `individual` 조합은 `on_configure` 가 FAILURE 를 반환한다. `2` 는 그 밖에도 `can_interface` 가 없을 때 (`loopmodel` 제외), `feedforward_kt` 가 10개가 아니거나 양수가 아닐 때, `cmd_ack` 가 `echo` · `none` 이 아닐 때, 명시한 `req_mode broadcast` 가 `cmd_ack echo` 와 함께일 때, `손 수 × 2 × recv_timeout_ms` (`broadcast` 는 `2 ×`) 가 comm 슬롯 (`1000 / loop_rate_hz × comm_decimation` ms) 을 넘을 때 거부한다 (남는 여유가 슬롯의 10 % 미만이면 WARN). YAML 의 값은 반드시 `"2"` 로 따옴표를 친다 (정수는 파라미터 선언에서 예외).

`2` 의 `JointCommand.feedforward` 는 `values` 와 크기가 같을 때만 같은 이름 맵으로 재정렬되어 전송되고 (joint offset 은 적용하지 않음), 비어 있으면 0, 그 외 크기면 0 + 경고 + `feedforward_size_mismatch` 카운터, NaN · ±inf 항목은 0 + `feedforward_nonfinite` 카운터다. `command_type` · `kp` · `kd` 는 소비하지 않는다. wire 포맷의 SSoT 는 [hand_canfd_packets.hpp](include/udp_hand_driver/hand_canfd_packets.hpp) 주석이다.

### 두 손 (`hand_sides`)

`hand_sides: [right, left]` (`2` 전용) 면 노드 하나가 손마다 컨트롤러 · CAN 소켓 · failure detector 를 두고, `hand_comm` 스레드 하나가 tick 마다 목록 순서로 한 손씩 트랜잭션을 끝낸다. 이 순서가 버스 직렬화라 두 손의 프레임은 겹치지 않는다. 비어 있으면 위의 한 손 경로와 바이트 동일하다.

`req_mode: broadcast` (두 손의 기본) 면 tick 이 손마다 CMD → REQ 하나 (노드 0xFF, 모든 손이 답한다) → 손마다 그 REQ 기준 수신 (REQ + T, write tick 은 2T) → 손마다 꼬리 순서로 돈다. 순서와 E-Stop 재확인 자리는 `hand_comm_loop.hpp` 헤더.

- 토픽: 상대 이름에 `<side>/` 가 붙는다 (`/hand/right/joint_states` …). 손이 둘이면 손별 토픽에 절대 이름을 주면 configure 가 거부한다 (두 손이 한 토픽이 된다)
- E-Stop: `/hand/estop` 하나. latch 는 손마다 cycle 머리에서, loop 가 손 사이에서 다시 본다. 두 손 모두 멈춘 뒤 손 순서대로 zero-write 를 1회씩 보낸다. 최악 지연은 주기 + 한 손의 수신 창이다 (400 Hz · 1.2 ms: 4.9 ms)
- 손별 키: `<side>.joint_state_names` · `motor_state_names` · `hand_fingertip_names` · `joint_position_offsets_deg` · `feedforward_kt`. 없거나 비면 flat 키를 쓴다
- 파일: `hand_stats_<side>.json` 이 손마다, `hand_timing_log.csv` 는 하나다. CSV 의 `t_total_us` 는 두 트랜잭션을 담은 tick 이고 `t_state_us` 는 마지막 손의 read 끝까지다. `broadcast` 의 손별 `total_us` 는 앞 손의 수신 창을 포함한다
- link down 은 그 손의 detector 만 판정하고 다른 손을 멈추지 않는다
- rate · T 는 호스트별 실측값이다: IXXAT USB 400 · 1.2, 네이티브 CAN 450 · 1.0 (D87, [docs/testing.md](../docs/testing.md) G-5)
- activate 중 한 손의 시작이 실패하면 앞 손도 닫고 FAILURE 다 (loop 는 시작하지 않는다)

### 통신 decimation (`comm_decimation`)

UDP 부하를 강제로 줄여 보는 테스트용 knob 이다 (기본 `1`). `sensor_decimation` 이 센서 read 만 감쇠하는 것과 달리 `comm_decimation = N` 은 CommLoop N 사이클 중 1번만 **전체 트랜잭션 (write + 모든 read)** 을 수행한다. skip 사이클은 UDP 도 publish 도 없는 no-op 이라 downstream 이 감쇠된 rate 를 실제로 본다.

- 첫 사이클은 항상 통신하고, E-Stop 은 skip 과 무관하게 매 사이클 검사한다. 명령은 유실 없이 다음 통신 사이클에 전송된다 (last-wins)
- 유효 rate: 통신 `loop_rate_hz / comm_decimation`, 센서 `loop_rate_hz / (comm_decimation × sensor_decimation)`. `on_activate` 가 `min_rate_hz` 를 `comm_decimation` 으로 자동 스케일하므로 rate failure 오탐은 없다
- 감쇠량은 `hand_stats.json` 의 `comm_decimation` · `comm_decimation_skip_count`

### ONNX F/T 추론

`1a` 센서 cycle 에서 `FingertipFTInferencer` 가 핑거팁별 접촉 / 힘 추론을 한다: 시작 시 N 샘플로 barometer baseline 보정 → 정규화 + delta + FIFO history → per-fingertip ONNX (`rtc::OnnxEngine`) → contact probability, force(3), direction(3). 설정은 [config/fingertip_ft_inferencer.yaml](config/fingertip_ft_inferencer.yaml) (`ft_inferencer.*`). ORT `Run()` 의 heap 할당은 RT-1 조건부 수용이다 ([invariants-rationale.md](../docs/reference/invariants-rationale.md)).

> ⚠️ **capability lockstep.** 컨슈머의 device YAML `devices.<hand_group>.sensor_layout.has_native_contact` · `has_native_displacement` 는 `ft_inferencer.enabled` 와 함께 켜고 꺼야 한다. 추론이 꺼진 채 capability=true 이면 컨트롤러가 항상 0 인 contact probability 를 native signal 로 오인해 grasp detection 이 조용히 무력화된다.

> ⚠️ **텐서는 positional 로 바인딩된다.** `F` 와 `u` 는 둘 다 `[1,3]` 이라 head 순서가 바뀐 `.onnx` 를 검출하지 못한다 (힘과 방향이 조용히 뒤바뀜). 실모델을 받으면 `ros2 run rtc_inference rtc_inference_check <model>.onnx` 로 텐서 이름을 확인해 `fingertip_ft_inferencer.hpp` 의 `model_config.inputs` / `.outputs` 에 채운다 ([rtc_inference](../rtc_inference/README.md#텐서를-이름으로-바인딩하는-이유)). 현재 두 config 의 `model_paths` 는 전부 `""` 다.

### SIL 모드 (`sil_mode`)

하드웨어 없이 두 상보적 SIL 계층을 **`sil_mode` 하나**로 고른다. `sil_mode` 는 노드 파라미터 (YAML) 이고 launch 인자 `sil_mode:=` 가 override 한다. 노드 `on_configure` 가 `use_fake_hand` 와 실효 endpoint 를 파생하고, `fake_hand_firmware` 는 launch 가 `sil_mode` 를 읽어 조건부로 띄운다. `integrated_bringup` 의 `robot_ur5e_p1a.launch.py` · `robot_ur5e_p1b.launch.py` 도 같은 `sil_mode:=` 를 지원한다.

| `sil_mode` | 계층 | `use_fake_hand` | firmware | 네트워크 endpoint | 검증 대상 |
|---|---|:---:|:---:|---|---|
| `off` (기본) | 실 하드웨어 | false | 미실행 | YAML 그대로 | - |
| `loopmodel` | controller-side loop-model | true | 미실행 | 소켓 미오픈 | thread · RT 구조 (실 모드와 같은 self-clock) |
| `firmware` | device-side network SIL | false | 실행 | `target_ip`→`127.0.0.1`, `local_ip` · `local_interface` 해제 (`2`: `can_interface`→`vcan0`) | 전체 transport (framing · echo · MODE 검증 · decode · publish) |

**`loopmodel`**: UDP 소켓을 우회하고 command 를 in-process 1차 LPF 로 통과시켜 read state 를 만든다. 튜닝은 `fake_lpf_time_constant_s` · `fake_effort_stiffness` · `fake_effort_damping`.

**`firmware`**: `fake_hand_firmware` 가 hand 프로토콜의 디바이스 측을 흉내 내고, 수정하지 않은 `udp_hand_node` 가 loopback UDP 로 통신한다. firmware 의 bind 포트는 `target_port`, wire 포맷은 `protocol_version` 이다. `"2"` 는 UDP 대신 `can_interface` (기본 `vcan0`) 의 CAN FD 에 `hand_side` 노드로 붙는다. 합성 튜닝 (LPF τ · effort 게인 · 센서 대역 · seed) 은 [config/fake_hand_firmware.yaml](config/fake_hand_firmware.yaml).

```bash
ros2 launch udp_hand_driver udp_hand.launch.py sil_mode:=firmware protocol_version:=1a
ros2 launch udp_hand_driver udp_hand.launch.py sil_mode:=firmware protocol_version:=2   # vcan0 필요
ros2 topic echo /hand/link_status --once     # 별도 터미널
```

`protocol_version:=2` 는 FD vcan0 (MTU 72, `repo_scripts/scripts/setup_vcan.sh`) 이 없으면 launch 가 거부한다. `test_canfd_hand_comm` 이 같은 vcan0 · 같은 노드 id 를 쓰므로 proto2 SIL 과 `colcon test` 를 동시에 돌리지 않는다.

한계: µs loopback latency 라 timing 검증엔 부적합하고, driver decoder 의 거울상이라 실제 펌웨어와의 일치는 보장하지 못하며, commander 없이 돌리면 모터가 rest(0) 에 머물러 failure detector 의 all-zero / duplicate 경고가 뜬다.

## 설정

[config/udp_hand_node.yaml](config/udp_hand_node.yaml) 이 launch 가 읽는 파일이고 키 설명은 파일 주석이 갖는다. 아래는 **이 YAML 에 실제로 들어 있는 값** 이다 (YAML 에 없는 키는 노드 기본값).

| 파라미터 | YAML 값 | 설명 |
|---|---|---|
| `target_ip` / `target_port` | `"192.168.0.100"` / `55151` | 핸드 컨트롤러 endpoint |
| `local_ip` / `local_interface` | `""` / `""` | `""` = 커널 선택 / 라우팅 테이블에 위임. 같은 subnet 에 NIC 이 둘 이상일 때만 설정 |
| `recv_timeout_ms` | `0.4` | ppoll 타임아웃 (sub-ms). `"2"` 의 deadline 체인은 [launch 인자](#launch-인자-udp_handlaunchpy) 표 |
| `communication_mode` | `"bulk"` | `"individual"` 도 가능. 노드 C++ 기본값은 `individual` 이므로 YAML 없이 띄우면 달라진다 |
| `loop_rate_hz` | `500.0` | CommLoop 주기 |
| `publish_rate` | `100.0` | `link_status` decimation 기준 (`loop_rate_hz / publish_rate`) |
| `comm_decimation` / `sensor_decimation` | `1` / `1` | 위 [통신 decimation](#통신-decimation-comm_decimation) · 센서 read 감쇠 |
| `protocol_version` | `"1b"` | `"1a"` \| `"1b"` \| `"2"` (따옴표 필수) |
| `can_interface` / `feedforward_kt` / `cmd_ack` | `"can0"` / 10 × `1.0` / `"none"` | `"2"` 전용: SocketCAN 장치 / 모터별 토크 상수 [Nm/A] / CMD echo 여부. `hand_side` 는 YAML 에 주석 (기본 `right`) |
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

include 의 각 헤더는 상단 주석에 역할과 동작 계약이 있다. `protocol/` 은 `SensorProtocol` 구현. 컨트롤러는 링크를 `HandComm` (`hand_comm.hpp`) 으로만 보며 기본 구현 `UdpHandComm` 이 UDP 를 감싼다 — 다른 링크 (CAN FD) 는 `SetHandComm()` 으로 주입한다. `hand_comm_loop.hpp` 의 `HandCommLoop` 이 모든 손을 돌리는 comm 스레드다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| `/hand/link_status` 가 `false`, recv timeout | IP · NIC 불일치, 포트 차단 | `target_ip` 확인 (YAML 은 `192.168.0.100`). `hand_stats.json` 의 `recv_timeout` · `recv_error` 확인 |
| `sil_mode:=firmware` 인데 timeout 만 반복 | YAML 의 `local_interface` 가 실 NIC 에 소켓을 고정 | 노드가 `firmware` 모드에서 binding 을 해제하고 INFO 로 원래 값을 남긴다. 해당 NIC 이 없는 PC 에서는 `Open()` 이 ENODEV 로 실패한다 |
| failure detector 가 all-zero / duplicate 경고 | commander 없이 단독 실행 (SIL 모터가 0 에 머묾) · 센서 없는 손 | 명령을 보내면 해소된다 |
| `1b` · `2` 에서 `on_configure` FAILURE | [프로토콜](#센서-프로토콜-버전-protocol_version) 절의 거부 조건 | ERROR 한 줄이 원인을 적는다. 값을 고쳐 `ros2 param set` 뒤 다시 configure 하면 된다 |
| `2` 의 write 가 전부 `write_echo.timeout`, 또는 `echo_unexpected` 증가 | `cmd_ack` 가 펌웨어와 불일치 | 노드 WARN 이 맞는 값을 적는다 |
| `broadcast` 에서 한 손만 세트 timeout | 중재에 진 RESP 를 펌웨어가 버림, 또는 T 가 뒤 손 버스트보다 짧음 | `per_hand` (250 Hz · T 0.9) 로 돌리고 펌웨어 · T 확인 |
| F/T 가 항상 0, grasp 이 안 잡힘 | `ft_inferencer.enabled=false` 인데 capability=true | [capability lockstep](#onnx-ft-추론) |
| `recv_error` 가 증가 | 손 쪽 포트 닫힘 (ICMP port-unreachable → `ECONNREFUSED`) | 펌웨어 · 전원 · 케이블 확인. timeout 과 구분되는 결정적 link-down 신호다 |

### 통계 (`hand_stats.json`)

`<session>/device/hand_stats.json` 에 통신 · 타이밍 통계를 쓴다. 10 s 주기 저장 (SIGKILL 대비), failure 발생 즉시 1회, 정상 teardown 때 저장한다. `comm_stats` 에 cycle · recv ok / timeout / error · mismatch · `stale_drained` · `comm_decimation` 이 있고, `per_request` 는 request kind (`write_echo` · `motor_read` · `joint_read` · `sensor_read` · `bulk_sensor` · `set_mode`) 별 집계다.

`last_unexpected_cmd` 에 직전 request 의 cmd echo 가 찍히면 timeout 이 아니라 1-cycle stale desync 시그니처다. `timing_stats` 는 성공한 패킷의 latency 만 집계한다. per-tick 타이밍은 `<session>/timing/hand_timing_log.csv`. `2` 에서 `read_all_motor` 는 항상 0, write tick 의 `read_all_joint_motor` 는 echo 끝 (`cmd_ack none`: CMD 송신) → 세트 완료, broadcast tick 의 `read_all_sensor` 는 그 손의 decode 만이고 `total` 은 tick 전체라 두 손이 같다 (profiler 헤더). 로그 `Hand timing` 의 칸은 모두 평균 µs 다.

`protocol_version` · `transport` (`udp` \| `canfd`, 설치된 링크 — `loopmodel` 은 기록하지 않음) · `feedforward_size_mismatch` · `feedforward_nonfinite` 는 모든 프로토콜에 있고, `"2"` 는 `canfd` 블록 (`cmd_ack` · `req_mode` · tx errno · error frame · `echo_payload_mismatch` · `echo_unexpected` · assembler 카운터) 을 더한다. `hand_side` 를 명시하면 두 파일이 `hand_stats_<side>.json` · `hand_timing_log_<side>.csv` 가 된다 (`hand_sides` 는 [두 손](#두-손-hand_sides)). 키 집합은 `test/fixtures/hand_stats_keys*.txt` 가 고정한다.

### 로깅 (Logging)

레벨 분류 · 공통 규칙은 [conventions.md §Logging](../agent_docs/conventions.md#logging). 패키지 규칙은 hot path (`RunCommCycle` · `*SharedReq*` · `UdpHandTransport::Send/Recv` · `PreFilter/ApplyFilters` · `FingertipFTInferencer::Infer`, 모두 `loop_rate_hz` 폴링) 에서 정상 경로의 직접 `INFO`/`WARN` 과 `RCLCPP_*_ONCE` 를 금지하고 `*_THROTTLE` 만 쓰는 것이다. THROTTLE 상수는 `udp_hand_logging.hpp` 가 SSoT.

| 서브-로거 | 사용처 |
|---|---|
| `hand.node` | `UdpHandNode` (lifecycle · 토픽 · link 상태 전이) |
| `hand.ctrl` | `UdpHandController` (CommLoop · 캘리브레이션 dispatch) |
| `hand.link` | `UdpHandTransport` (소켓 open/close · 센서모드 전환) |
| `hand.sensor` | `UdpHandSensorProcessor` (LPF · drift) |
| `hand.fail` | `UdpHandFailureDetector` (50 Hz 워치독) |
| `hand.ft` | `FingertipFTInferencer` (모델 로드 · 캘리브레이션) |

```bash
ros2 service call /hand/udp_hand_node/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'hand.link', level: 10}]}"
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
```

## 더 읽기

- UDP 소켓 · drain · mode 검증 · retry: [udp_hand_transport.hpp](include/udp_hand_driver/udp_hand_transport.hpp) 주석
- CAN FD 링크 (수신 창 · deadline 체인 · E-Stop 순서): [canfd_hand_comm.hpp](include/udp_hand_driver/canfd_hand_comm.hpp) 주석
- mailbox · write gate · E-Stop self-exit: [udp_hand_controller.hpp](include/udp_hand_driver/udp_hand_controller.hpp) · [udp_hand_node.hpp](include/udp_hand_driver/udp_hand_node.hpp) 주석
- RT 튜닝: [rt-optimization.md](../docs/rt-optimization.md)

## 의존성

[package.xml](package.xml) · MIT. ROS 2 (rclcpp · lifecycle · sensor/std msgs) · rtc_base · rtc_communication · rtc_inference · rtc_msgs.
