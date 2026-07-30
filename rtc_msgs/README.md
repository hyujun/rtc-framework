# rtc_msgs


> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크를 위한 **커스텀 ROS2 메시지 + 서비스 정의** 패키지입니다. 로봇 암 관절 커맨드, 핑거팁 센서 데이터 (원시/필터링), 추론 결과 (힘/변위/접촉), 파지 상태 판정, 컨트롤러 lifecycle 조회/전환, ToF 스냅샷, 디바이스 상태/센서 로깅을 위한 메시지·서비스 타입을 정의합니다.

**핵심 특징:**
- 로봇 비의존적(robot-agnostic) 메시지 설계
- 관절/태스크 공간 목표 지원 (JointCommand, RobotTarget)
- 핑거팁 센서: 원시(raw) + 필터링(filtered) 데이터 + ONNX 추론 출력 통합
- 파지 상태: 컨트롤러 주기 (RT 정기 tick @ `control_rate`, default 500 Hz) 에서 계산된 접촉/힘/파지 판정 (Force-PI: GraspState, TSID-based WBC: WbcState)
- 컨트롤러 관리: lifecycle 상태 조회(ListControllers) + activate/deactivate(SwitchController), `/rtc_cm/*` API
- CSV 로깅 전용 메시지: 상태/커맨드/궤적 통합 (DeviceStateLog, DeviceSensorLog)
- C++ 및 Python 바인딩 자동 생성 (rosidl)

---

## 패키지 구조

```
rtc_msgs/
├── CMakeLists.txt
├── package.xml
├── README.md
├── msg/
│   ├── JointCommand.msg       <- 로봇 암 관절 커맨드 (position/torque/pd_feedforward)
│   ├── FingertipSensor.msg    <- 단일 핑거팁 센서 + 추론 결과
│   ├── HandSensorState.msg    <- 전체 핸드 센서 상태 (핑거팁 집계)
│   ├── GraspState.msg         <- 파지 상태 판정 (접촉/힘/grasp 감지)
│   ├── WbcState.msg           <- TSID 기반 WBC 컨트롤러 파지 상태 (GraspState의 WBC 대응)
│   ├── RobotTarget.msg        <- 관절/태스크 공간 목표
│   ├── ControllerState.msg    <- 컨트롤러 lifecycle 상태 스냅샷 (list_controllers 응답용)
│   ├── DeviceStateLog.msg     <- 디바이스 상태 종합 로그
│   ├── DeviceSensorLog.msg    <- 디바이스 센서 로그
│   ├── SimSensor.msg          <- MuJoCo 단일 센서 출력 (로봇 비의존적)
│   ├── SimSensorState.msg     <- MuJoCo 센서 데이터 집계 (로봇 비의존적)
│   ├── ToFSnapshot.msg        <- ToF 센서 + 핑거팁 SE3 자세 통합 스냅샷
│   ├── CalibrationCommand.msg <- 센서 캘리브레이션 명령 (확장 가능 enum)
│   └── CalibrationStatus.msg  <- 센서 캘리브레이션 진행/완료 상태
└── srv/
    ├── GraspCommand.srv       <- Force-PI 그래스프 one-shot 이벤트 (start/release)
    ├── ListControllers.srv    <- 등록된 컨트롤러 lifecycle 상태 조회 (/rtc_cm/list_controllers)
    ├── ResetFault.srv         <- latched controller-local fault 해제 (/rtc_cm/reset_fault)
    └── SwitchController.srv   <- 컨트롤러 activate/deactivate 요청 (/rtc_cm/switch_controller)
```

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `std_msgs` | `Header` 메시지 타입 (타임스탬프 + 프레임 ID) |
| `geometry_msgs` | `Pose` 메시지 타입 (ToFSnapshot 핑거팁 자세) |
| `builtin_interfaces` | `Time` 타입 (ToFSnapshot 타임스탬프) |
| `ament_cmake` | 빌드 시스템 |
| `rosidl_default_generators` | 메시지 C++/Python 코드 생성 |

---

## 메시지 정의

### `JointCommand.msg`

로봇 암 관절 커맨드 메시지 -- position, torque, pd_feedforward 모드를 지원합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `joint_names` | `string[]` | 관절 이름 배열 (비어있으면 기본 순서 fallback) |
| `values` | `float64[]` | 커맨드 값 배열 -- position(rad) \| torque(Nm) \| pd_feedforward: PD position target(rad) |
| `command_type` | `string` | `"position"` \| `"torque"` \| `"pd_feedforward"` |
| `feedforward` | `float64[]` | `pd_feedforward` 전용 -- per-joint feedforward 토크 (Nm, gravity 포함). 다른 모드에선 무시. 비면 0 |
| `kp` | `float64[]` | `pd_feedforward` 전용 -- PD 위치 게인 (>= 0). sticky (비면 현재 게인 유지) |
| `kd` | `float64[]` | `pd_feedforward` 전용 -- PD 속도 게인 (>= 0). sticky (비면 현재 게인 유지) |

- `joint_names`와 `values`는 1:1 대응 관계입니다.
- `command_type == "pd_feedforward"`: PD position-servo backbone(`values`) + per-joint feedforward 토크(`feedforward[]`) 오버레이. `rtc::CommandType::kPdFeedforward`로 컨트롤러 스택이 발행 가능하지만, `mujoco_sim`만 feedforward를 `qfrc_applied`로 주입하고 실 하드웨어(`udp_hand_driver` 등 position-only)는 feedforward를 무시한다.
- `kp`/`kd`는 all-or-nothing: 둘 중 하나만 채우거나 그룹 조인트 전체를 덮지 못하면 무시된다.

---

### `FingertipSensor.msg`

단일 핑거팁 센서 데이터 메시지 -- 8개 기압(barometer) 센서, 3개 ToF 센서의 원시/필터링 값과 ONNX 추론 결과를 포함합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `name` | `string` | 핑거팁 이름 (예: `"index"`, `"thumb"`) |
| `barometer` | `float32[8]` | 기압 센서 값 -- 필터링됨 (post-LPF) |
| `tof` | `float32[3]` | ToF 거리 센서 값 -- 필터링됨 (post-LPF) |
| `barometer_raw` | `float32[8]` | 기압 센서 값 -- 원시 (pre-LPF) |
| `tof_raw` | `float32[3]` | ToF 거리 센서 값 -- 원시 (pre-LPF) |
| `f` | `float32[3]` | 추정 힘 벡터 [Fx, Fy, Fz] (ONNX 모델 출력) |
| `u` | `float32[3]` | 추정 변위 벡터 [ux, uy, uz] (ONNX 모델 출력) |
| `contact_flag` | `float32` | 접촉 감지 플래그 (0.0 = 비접촉, 1.0 = 접촉) |
| `inference_enable` | `bool` | 캘리브레이션 완료 여부 (`true` = 추론 결과 유효) |

- 핑거팁 당 8 barometer + 3 ToF는 하드웨어 사양에 의해 고정 (`kSensorValuesPerFingertip = 11`)
- 대역폭 최적화를 위해 `Header` 필드를 포함하지 않으며, 상위 `HandSensorState`의 header를 사용합니다
- 원시(raw)와 필터링(filtered) 데이터를 동시에 전달하여 진단/디버깅 지원

---

### `HandSensorState.msg`

전체 핸드 센서 상태 메시지 -- 모든 핑거팁의 센서 데이터를 집계합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `fingertips` | `FingertipSensor[]` | 핑거팁 센서 배열 (YAML 설정에 의해 크기 결정) |

---

### `GraspState.msg`

컨트롤러의 RT 정기 tick (`control_rate`, default 500Hz) 에서 계산된 파지(grasp) 상태 메시지입니다. BT coordinator가 구독하여 파지 판정에 사용합니다.

| 카테고리 | 필드 | 타입 | 설명 |
|---------|------|------|------|
| **헤더** | `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| **핑거팁별** | `fingertip_names` | `string[]` | 핑거팁 이름 배열 (예: `["thumb", "index", "middle", "ring"]`) |
| | `force_magnitude` | `float32[]` | 핑거팁별 힘 크기 \|F\| [N] |
| | `contact_flag` | `float32[]` | 핑거팁별 접촉 확률 (0.0~1.0) |
| | `inference_valid` | `bool[]` | 핑거팁별 추론 유효 여부 |
| **집계** | `num_active_contacts` | `int32` | 활성 접촉 핑거팁 수 |
| | `max_force` | `float32` | 전체 핑거팁 중 최대 힘 크기 |
| | `grasp_detected` | `bool` | 파지 감지 여부 (`num_active_contacts >= min_fingertips`) |
| | `force_threshold` | `float32` | 감지에 사용된 힘 임계값 [N] |
| | `min_fingertips` | `int32` | 파지 판정 최소 핑거팁 수 |

- 핑거팁별 배열(`fingertip_names`, `force_magnitude`, `contact_flag`, `inference_valid`)은 모두 동일한 크기입니다.
- 집계 필드는 컨트롤러 내부에서 계산되어 BT coordinator에서 바로 사용할 수 있습니다.

**Force-PI 그래스프 컨트롤러 상태** (`grasp_controller_type == "force_pi"` 일 때만 유효):

| 필드 | 타입 | 설명 |
|------|------|------|
| `grasp_phase` | `uint8` | GraspPhase enum (0=Idle, 1=Approaching, 2=Contact, 3=ForceControl, 4=Holding, 5=Releasing) |
| `finger_s` | `float32[]` | 핑거별 그래스프 파라미터 [0,1] |
| `finger_filtered_force` | `float32[]` | 핑거별 필터링된 힘 [N] |
| `finger_force_error` | `float32[]` | 핑거별 힘 오차 [N] |
| `grasp_target_force` | `float32` | 현재 목표 힘 [N] |

**In-plane pull-force estimate** (#167) — `pull` 필드는 `PullEstimate.msg` 하위 메시지이며 `WbcState.msg` 도 같은 타입을 embed 한다 (소비자 저장 코드 공유 목적). `rtc::grasp::PullForceEstimator` 출력.

| 필드 | 타입 | 설명 |
|------|------|------|
| `pull` | `PullEstimate` | In-plane pull-force estimate — 하위 필드는 아래 `PullEstimate.msg` 표 참조 |

#### `PullEstimate.msg`

| 필드 | 타입 | 설명 |
|------|------|------|
| `force` | `float32[3]` | 필터링된 추정 힘 F̂, reference frame [N] |
| `force_inplane` | `float32[2]` | Bᵀ·F̂ 평면 좌표 [N] |
| `magnitude` | `float32` | \|F̂\| [N] |
| `directional` | `float32` | dᵀ·F̂ (direction 미설정 시 0) [N] |
| `friction_utilization` | `float32` | max_i \|f_t,i\| / (μ_i·f_n,i) |
| `leakage_bound` | `float32` | grip→in-plane leakage bound [N] |
| `valid_contact_count` | `int32` | 유효 접촉 수 |
| `valid` | `bool` | 추정 유효 여부 (required-role gate 통과) |
| `slip_risk` | `bool` | slip ratio 임계 초과 여부 |
| `any_saturated` | `bool` | 접촉 중 saturation 발생 여부 |
| `baseline_applied` | `bool` | baseline subtraction 적용 여부 |

---

### `WbcState.msg`

TSID 기반 whole-body controller (예: `DemoWbcController`)가 publish하는 파지 상태 메시지입니다. `GraspState`의 역할을 WBC 컨트롤러용으로 대응하되, 필드 구성은 WBC의 TSID 기반 파지 알고리즘을 반영합니다 (phase enum이 WBC FSM, force/contact는 raw 핑거팁 센서 파싱에서 옴; Force-PI 전용 필드인 `finger_s`/`finger_filtered_force`/`finger_force_error`는 없음). BT coordinator는 활성 컨트롤러 이름에 따라 `grasp_state` 또는 `wbc_state`를 구독합니다.

| 카테고리 | 필드 | 타입 | 설명 |
|---------|------|------|------|
| **헤더** | `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| **phase** | `phase` | `uint8` | `PHASE_IDLE=0` / `PHASE_APPROACH=1` / `PHASE_PRE_GRASP=2` (deprecated, APPROACH 로 병합 — ABI 호환용 예약) / `PHASE_CLOSURE=3` / `PHASE_HOLD=4` / `PHASE_RETREAT=5` (deprecated, 더 이상 publish 안 됨 — ABI 호환용 예약) / `PHASE_RELEASE=6` / `PHASE_FALLBACK=7` |
| **핑거팁별** | `fingertip_names` | `string[]` | 핑거팁 이름 배열 (크기 = num_fingertips, 보통 4) |
| | `force_magnitude` | `float32[]` | 핑거팁별 힘 크기 \|F\| [N] |
| | `contact_flag` | `float32[]` | 접촉 판정 -- backend capability (`has_native_contact`)에 따라 native sigmoid 확률([0,1]) 또는 controller-derived binary(1.0/0.0). 두 경우 모두 `> 0.5f`가 in-contact 판정 |
| | `displacement` | `float32[]` | 핑거팁 변위 [m] -- native 지원 시에만 값 존재, 그 외 0 |
| **집계** | `num_active_contacts` | `int32` | 활성 접촉 핑거팁 수 |
| | `max_force` | `float32` | 전체 핑거팁 중 최대 힘 크기 [N] |
| | `grasp_target_force` | `float32` | 현재 목표 힘 [N] (gain에서 유래) |
| | `grasp_detected` | `bool` | 파지 감지 여부 (`num_active_contacts >= min_fingertips`) |
| | `min_fingertips` | `int32` | 파지 판정 최소 핑거팁 수 |
| **TSID 진단** | `tsid_solve_us` | `float32` | 마지막 solver 계산 시간 [us] (informational) |
| | `tsid_solver_ok` | `bool` | 마지막 QP 수렴 여부 |
| | `qp_fail_count` | `int32` | 활성화 이후 누적 QP 실패 횟수 |
| **Pull estimate** | `pull` | `PullEstimate` | In-plane pull-force estimate (#167) — 하위 필드는 위 `GraspState.msg` 의 `PullEstimate.msg` 표와 동일 (measured R_i·f_i 기반, TSID λ_opt 아님) |

- Per-controller 토픽: `/<config_key>/hand/wbc_state` (500 Hz 컨트롤러, ~50 Hz publish thread).

---

### `RobotTarget.msg`

관절 공간 또는 태스크 공간의 목표 위치입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `goal_type` | `string` | `"joint"` 또는 `"task"` |
| `joint_names` | `string[]` | 관절 이름 배열 |
| `joint_target` | `float64[]` | 관절 공간 목표 (rad) |
| `task_target` | `float64[6]` | 태스크 공간 목표 [x, y, z, roll, pitch, yaw] |

- `joint_names`와 `joint_target`은 1:1 대응, `task_target`은 항상 6개 고정값입니다.

---

### `ControllerState.msg`

`/rtc_cm/list_controllers` 가 반환하는, 단일 컨트롤러의 메타데이터 + lifecycle 상태 스냅샷입니다. ros2_control의 `ControllerState` 의미를 따르되 RTC가 실제로 추적하는 필드만 남겼습니다 (chaining/동적 로드 없음).

| 필드 | 타입 | 설명 |
|------|------|------|
| `name` | `string` | `RTC_REGISTER_CONTROLLER`로 등록된 컨트롤러 인스턴스 이름 (`RTControllerInterface::Name()`) |
| `state` | `string` | lifecycle 상태 -- `"unconfigured"` (on_configure 이전) \| `"inactive"` (configure 완료, 비활성) \| `"active"` (RT 루프가 디스패치 중) \| `"finalized"` (on_cleanup 완료) |
| `type` | `string` | 컨트롤러 타입 -- `RTC_REGISTER_CONTROLLER`의 registry plugin 이름 키 |
| `is_active` | `bool` | `state == "active"`와 동일한 정보의 편의 플래그 |
| `claimed_groups` | `string[]` | 이 컨트롤러가 인지하는 디바이스 토픽 그룹 이름 (`controller_topic_configs_[i].groups` 키, ownership 무관). 진단용 -- 독점 소유를 의미하지 않음 |

---

### `DeviceStateLog.msg`

디바이스 상태, 커맨드, 목표, 궤적을 통합한 로깅 메시지입니다. CSV 세션 로깅에 사용됩니다.

| 카테고리 | 필드 | 타입 | 설명 |
|---------|------|------|------|
| **헤더** | `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| | `joint_names` | `string[]` | 관절 이름 배열 (N개) |
| **상태** | `actual_positions` | `float64[]` | 현재 관절 위치 (N) |
| | `actual_velocities` | `float64[]` | 현재 관절 속도 (N) |
| | `efforts` | `float64[]` | 현재 관절 토크 (N) |
| **커맨드** | `commands` | `float64[]` | 전송된 커맨드 (N) |
| | `command_type` | `string` | `"position"` 또는 `"torque"` |
| **목표** | `goal_type` | `string` | `"joint"` 또는 `"task"` |
| | `joint_goal` | `float64[]` | 관절 공간 목표 (N) |
| | `task_goal` | `float64[6]` | 태스크 공간 목표 |
| **궤적** | `trajectory_positions` | `float64[]` | 궤적 레퍼런스 위치 (N) |
| | `trajectory_velocities` | `float64[]` | 궤적 레퍼런스 속도 (N) |
| **FK** | `actual_task_positions` | `float64[6]` | 순기구학 TCP 위치 |
| **모터** | `motor_names` | `string[]` | 모터 이름 (M개, 선택적) |
| | `motor_positions` | `float64[]` | 모터 위치 (M) |
| | `motor_velocities` | `float64[]` | 모터 속도 (M) |
| | `motor_efforts` | `float64[]` | 모터 전류 (M) |

- 모든 동적 배열은 `joint_names`와 동일한 크기(N)를 갖습니다 (모터 필드는 `motor_names` 크기 M).
- CSV 열 순서: **Goal -> Current State -> Command -> Trajectory** 분류법을 따릅니다.

---

### `DeviceSensorLog.msg`

원시/필터링 센서 데이터와 추론 출력을 포함하는 로깅 메시지입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `sensor_names` | `string[]` | 핑거팁 이름 (예: `["thumb", "index", "middle", "ring"]`) |
| `sensor_data_raw` | `int32[]` | 원시 센서 값 (핑거팁당 M개) |
| `sensor_data` | `int32[]` | 필터링된 센서 값 (핑거팁당 M개) |
| `inference_valid` | `bool` | 추론 출력 유효성 플래그 |
| `inference_output` | `float32[]` | 추론 결과 (핑거팁당 F/u/contact 값) |

> 이 메시지는 컨트롤러 `logs:` 블록의 `msg_type:` 키로만 쓰이고 실제로 publish 되지는 않습니다. CSV 컬럼 집합의 SSoT 는 `integrated_bringup/logging/device_sensor_log_pod.hpp` 의 POD mirror 이며, 그쪽은 여기 없는 컬럼(LPF 된 축별 힘 등)을 추가로 갖는 **superset** 입니다.

---

### `ToFSnapshot.msg`

ToF 측정값 + 핑거팁 자세를 하나의 메시지로 통합합니다. RT 컨트롤러에서 동일 제어 사이클의 데이터를 묶어 publish합니다. shape_estimation 노드의 주 입력 데이터입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 타임스탬프 |
| `distances` | `float64[6]` | ToF 센서 거리 [m] (thumb_A, thumb_B, index_A, index_B, middle_A, middle_B) |
| `valid` | `bool[6]` | ToF 센서 유효성 플래그 |
| `tip_poses` | `geometry_msgs/Pose[3]` | 핑거팁 SE3 자세 (thumb, index, middle, 월드 프레임) |

- 컨트롤러 YAML의 `topics:` 섹션에서 `role: "tof_snapshot"`으로 등록하여 사용합니다.
- `shape_estimation_msgs`에서 로봇 독립성을 위해 이동된 메시지입니다.

---

### `SimSensor.msg`

MuJoCo 시뮬레이션에서 단일 센서 출력을 나타내는 로봇 비의존적 메시지입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `name` | `string` | XML 센서 이름 (예: `"ft_sensor"`, `"thumb_touch"`) |
| `sensor_type` | `int32` | mjtSensor enum 값 (예: `mjSENS_TOUCH=0`, `mjSENS_FORCE=3`) |
| `values` | `float64[]` | 센서 출력 (길이는 센서 차원에 따라 결정) |

---

### `SimSensorState.msg`

MuJoCo 시뮬레이션에서 디바이스 그룹별 센서 데이터를 집계한 메시지입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `sensors` | `SimSensor[]` | 해당 디바이스 그룹에 설정된 센서 배열 |

- `rtc_mujoco_sim`에서 물리 스텝마다 publish됩니다.

---

### `CalibrationCommand.msg`

센서 캘리브레이션 트리거 메시지입니다. `sensor_type` enum 을 통해 하나의 토픽으로
여러 센서 종류(barometer, F/T zero, ToF dark 등)의 캘리브레이션을 요청할 수 있습니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `sensor_type` | `uint8` | `SENSOR_BAROMETER` / `SENSOR_FT_ZERO` / `SENSOR_TOF_DARK` (enum) |
| `action` | `uint8` | `ACTION_START` / `ACTION_ABORT` (enum) |
| `sample_count` | `uint16` | 0 = YAML 기본값 사용, >0 = 이번 요청에 한해 override |

Publisher: `integrated_bringup/scripts/demo_controller_gui.py` 의 Control 탭 또는
`ros2 topic pub`.

Subscriber: `udp_hand_driver` 의 `udp_hand_node` (RELIABLE/1 QoS).

### `CalibrationStatus.msg`

센서 캘리브레이션 진행/완료 상태 메시지입니다. 센서 타입당 1개 메시지가 publish됩니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `sensor_type` | `uint8` | `CalibrationCommand.SENSOR_*` 와 동일 enum |
| `state` | `uint8` | `STATE_IDLE` / `STATE_RUNNING` / `STATE_COMPLETE` / `STATE_FAILED` |
| `progress_count` | `uint16` | 현재까지 누적된 샘플 수 |
| `target_count` | `uint16` | 목표 샘플 수 (진행률 = progress/target) |
| `message` | `string` | 사람-읽기용 (실패 원인 등, 선택) |

Publisher: `udp_hand_driver` 가 `calibration_status_rate_hz` (기본 5 Hz) 주기로
publish (QoS: RELIABLE + TRANSIENT_LOCAL + depth 1, late-join 구독자가 최신 상태
즉시 수신).

---

## 서비스 정의

### `GraspCommand.srv`

Force-PI 그래스프의 **one-shot 이벤트** 채널입니다. State가 아닌 transition이라
ROS 2 parameter로 표현하기에 부적절하므로 별도 srv로 분리되어 있습니다.

**Request:**

| 필드 | 타입 | 설명 |
|------|------|------|
| `command` | `uint8` | `NONE=0` (rejected) / `GRASP=1` / `RELEASE=2` |
| `target_force` | `float64` | 목표 grip force [N] (`GRASP`일 때만 사용, `> 0`) |

**Response:**

| 필드 | 타입 | 설명 |
|------|------|------|
| `ok` | `bool` | 활성 컨트롤러 grasp FSM에 적용 여부 |
| `message` | `string` | 사람-읽기용 결과 ("grasp started @ 2.0 N", "E-STOP active", ...) |

**Single-active 모델:** 활성 데모 컨트롤러만 `~/grasp_command` server를 advertise
합니다. 호출자(예: BT)는 `/{active_config_key}/grasp_command` 로 호출.

- `demo_joint_controller` / `demo_task_controller`: `grasp_controller_` (Force-PI
  FSM) 가 있을 때만 적용. 없으면 `ok=false, message="grasp_controller unavailable"`.
- `demo_wbc_controller`: `grasp_cmd_` atomic + `grasp_target_force` gain 갱신 →
  WBC 6-state FSM (slots 2 & 5 reserved) 이 다음 tick 의 최상단 preempt guard 에서
  `kApproach` (GRASP) / `kRelease` (RELEASE) 로 전이. RELEASE 는 active grasp
  phase (`kApproach`/`kClosure`/`kHold`) 어디서든 즉시 preempt;
  `kIdle` (no-op) / `kRelease` (이미 release 중) / `kFallback` (수동 복구 필요) 면제.
- E-STOP 활성 시 모든 호출이 `ok=false, message="E-STOP active"`.

Caller: grasp coordinator (예: BehaviorTree action node) 또는 `ros2 service call`.
Server: 활성 데모 컨트롤러의 LifecycleNode aux thread.

---

### `ListControllers.srv`

`/rtc_cm/list_controllers` -- `rtc_controller_manager`에 등록된 모든 컨트롤러의 lifecycle 상태를 조회합니다.

**Request:** 없음 (empty).

**Response:**

| 필드 | 타입 | 설명 |
|------|------|------|
| `controllers` | `rtc_msgs/ControllerState[]` | 로드된 컨트롤러마다 1개씩 |

---

### `SwitchController.srv`

`/rtc_cm/switch_controller` -- 컨트롤러 activate/deactivate를 요청합니다. Single-active 모델(D-A1)이므로 `activate_controllers`/`deactivate_controllers` 각각 최대 1개 항목만 허용됩니다.

**Request:**

| 필드 | 타입 | 설명 |
|------|------|------|
| `activate_controllers` | `string[]` | 활성화할 컨트롤러 이름 (≤ 1개) |
| `deactivate_controllers` | `string[]` | 비활성화할 컨트롤러 이름 (≤ 1개) |
| `strictness` | `int32` | `STRICT=1` (위반 시 무조건 `ok=false`) \| `BEST_EFFORT=2` (첫 유효 항목만 적용, 나머지는 warn) |
| `timeout` | `builtin_interfaces/Duration` | RT 루프의 swap 커밋을 기다릴 최대 시간 (0 = 무제한, 기본 호출자는 1.0s 권장) |

**Response:**

| 필드 | 타입 | 설명 |
|------|------|------|
| `ok` | `bool` | 선택된 strictness 하에서 전환이 커밋됐는지 여부 |
| `message` | `string` | 사람-읽기용 결과 ("switched <prev> -> <next>", "no-op (already active)", 실패 사유) |

- 응답은 동기(sync)입니다 -- `ok=true`가 반환되기 전에 새 활성 컨트롤러가 RT 루프에 반영되고 `/<robot_ns>/active_controller_name` (latched) 이 publish됩니다.
- E-STOP 활성 시 모든 호출이 `ok=false, message="E-STOP active"`.

---

### `ResetFault.srv`

`/rtc_cm/reset_fault` -- latched **controller-local** fault 를 해제합니다 (#260). compliance 계열은 critical fault 에서 `SAFE_STOP` 을 래치하고 자동 복귀하지 않으므로 (§10.6), 이 서비스가 유일한 외부 탈출구입니다. CM global E-STOP 과는 **의도적으로 분리**돼 있어 서로를 풀지 않습니다 (E-8).

**Request:**

| 필드 | 타입 | 설명 |
|------|------|------|
| `controller_name` | `string` | 대상 컨트롤러. **필수**이며 현재 active 컨트롤러와 일치해야 함 (이름 명시 = 오퍼레이터 확인) |

**Response:**

| 필드 | 타입 | 설명 |
|------|------|------|
| `ok` | `bool` | latch 가 실제로 사라졌음이 확인됐는지 (원래 latch 가 없던 no-op 도 `true`) |
| `message` | `string` | 결과 설명. global E-STOP 이 아직 래치돼 있으면 그 사실을 함께 알림 |

- 응답은 동기(sync)이고 **"전달됨"이 아니라 실제 결과**를 보고합니다 -- 요청 후 `1.5 × dt` 대기하고 latch 를 다시 읽어 해제 / no-op / 재래치(원인 잔존)를 구분합니다.
- 빈 `controller_name`, active 가 아닌 이름, 알 수 없는 이름은 모두 `ok=false` 로 거부되며 latch 는 그대로입니다. wildcard 는 없습니다.
- global E-STOP 활성 상태에서도 호출은 성립합니다 (RT 루프가 E-STOP 중에도 `Compute()` 를 계속 호출하므로) -- 다만 팔은 global latch 가 따로 풀릴 때까지 유지됩니다.

---

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select rtc_msgs
source install/setup.bash
```

빌드 후 생성된 메시지는 C++ (`rtc_msgs/msg/joint_command.hpp` 등) 및 Python (`rtc_msgs.msg.JointCommand` 등)에서 사용 가능합니다.

---

## 메시지 계층 구조

```
커맨드 (로봇으로 송신)                센서 상태 (하드웨어에서 수신)
└── JointCommand                    └── HandSensorState
    (position/torque/pd_feedforward)    └── FingertipSensor[]
                                            ├── barometer[8] + tof[3] (filtered)
                                            ├── barometer_raw[8] + tof_raw[3] (raw)
                                            └── f[3] + u[3] + contact_flag (추론)

파지 판정 (컨트롤러에서 계산)        목표
├── GraspState (Force-PI)           └── RobotTarget
│   ├── 핑거팁별: force/contact/valid    (관절/태스크 공간 목표)
│   └── 집계: grasp_detected/max_force
└── WbcState (TSID-based WBC)
    ├── phase: WBC FSM (PHASE_*)
    ├── 핑거팁별: force/contact/displacement
    └── TSID 진단: tsid_solve_us/tsid_solver_ok/qp_fail_count

(GUI/외부 도구는 sensor_msgs/JointState `/rtc_cm/<group>/joint_states` +
 tf2_msgs/TFMessage `<config_key>/transforms` 표준 토픽으로 위치 정보를 받음.)

컨트롤러 관리 (서비스)
├── ControllerState                 (list_controllers 응답 요소: name/state/type/is_active)
├── ListControllers.srv             /rtc_cm/list_controllers  → ControllerState[]
├── SwitchController.srv            /rtc_cm/switch_controller → activate/deactivate
└── ResetFault.srv                  /rtc_cm/reset_fault       → controller-local fault 해제

로깅 (CSV 세션 기록)                ToF 스냅샷 (형상 추정용)
├── DeviceStateLog                  └── ToFSnapshot
│   (상태 + 커맨드 + 목표 + 궤적)      (ToF 거리[6] + 핑거팁 SE3[3])
└── DeviceSensorLog
    (원시/필터 센서 + 추론 출력)

시뮬레이션 (MuJoCo 센서)
└── SimSensorState
    └── SimSensor[]
        (name, sensor_type, values[])
```

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
