# rtc_msgs

![version](https://img.shields.io/badge/version-v5.16.0-blue)

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RT Controller 스택을 위한 **커스텀 ROS2 메시지 정의** 패키지입니다. 로봇 암 관절 커맨드, 핸드 모터 커맨드/피드백, 핑거팁 센서 데이터, 핑거팁 힘/토크 추론 결과, GUI 표시, 디바이스 로깅을 위한 12종 메시지 타입을 정의합니다.

---

## 패키지 구조

```
rtc_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── JointCommand.msg          ← 로봇 암 관절 커맨드 (position/torque)
    ├── HandCommand.msg           ← 핸드 모터 커맨드
    ├── HandMotorState.msg        ← 핸드 모터 피드백 (위치/속도)
    ├── FingertipSensor.msg       ← 단일 핑거팁 센서 (기압 + ToF)
    ├── HandSensorState.msg       ← 전체 핸드 센서 상태
    ├── FingertipForceTorque.msg  ← 단일 핑거팁 힘/토크 추론 결과
    ├── HandForceTorqueState.msg  ← 전체 핸드 힘/토크 상태
    ├── GuiPosition.msg           ← GUI 표시용 관절/태스크 위치
    ├── DeviceJointState.msg      ← 디바이스 관절 상태 (위치/속도/토크)
    ├── JointGoal.msg             ← 관절/태스크 공간 목표
    ├── DeviceStateLog.msg        ← 디바이스 상태 종합 로그
    └── DeviceSensorLog.msg       ← 디바이스 센서 로그
```

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `std_msgs` | `Header` 메시지 타입 |
| `ament_cmake` | 빌드 시스템 |
| `rosidl_default_generators` | 메시지 C++/Python 코드 생성 |

---

## 메시지 정의

### `JointCommand.msg`

로봇 암 관절 커맨드 메시지 — position 또는 torque 모드를 지원합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `joint_names` | `string[]` | 관절 이름 배열 |
| `values` | `float64[]` | 커맨드 값 배열 (라디안 또는 토크) |
| `command_type` | `string` | `"position"` 또는 `"torque"` |

- `joint_names`와 `values`는 1:1 대응 관계입니다.
- `joint_names`가 비어있으면 기본 관절 순서로 fallback합니다.

### `HandCommand.msg`

핸드 모터 커맨드 메시지입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `motor_names` | `string[]` | 모터 이름 배열 |
| `values` | `float32[]` | 커맨드 값 배열 |

- `motor_names`와 `values`는 1:1 대응 관계입니다.

### `HandMotorState.msg`

핸드 모터 피드백 메시지 — 각 모터의 위치와 속도를 포함합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `motor_names` | `string[]` | 모터 이름 배열 |
| `positions` | `float32[]` | 모터 위치 배열 |
| `velocities` | `float32[]` | 모터 속도 배열 |

### `FingertipSensor.msg`

단일 핑거팁 센서 데이터 메시지 — 8개 기압(barometer) 센서, 3개 ToF 센서, 추론 결과를 포함합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `name` | `string` | 핑거팁 이름 (예: `"index"`, `"thumb"`) |
| `barometer` | `float32[8]` | 기압 센서 값 (하드웨어 고정 8개) |
| `tof` | `float32[3]` | ToF 거리 센서 값 (하드웨어 고정 3개) |
| `F` | `float32[3]` | 추정 힘 벡터 [Fx, Fy, Fz] (추론 모델 출력) |
| `u` | `float32[3]` | 추정 변위 벡터 [ux, uy, uz] (추론 모델 출력) |
| `contact_flag` | `float32` | 접촉 감지 플래그 (0.0 = 비접촉, 1.0 = 접촉) |

- 핑거팁 당 8개 barometer + 3개 ToF는 하드웨어 사양에 의해 고정되어 있습니다 (`kSensorValuesPerFingertip = 11`).
- `F`, `u`, `contact_flag`는 ONNX 추론 모델의 출력으로, 원시 센서 데이터와 함께 전달됩니다.
- 대역폭 최적화를 위해 `Header` 필드를 포함하지 않으며, 상위 `HandSensorState.msg`의 header를 사용합니다.

### `HandSensorState.msg`

전체 핸드 센서 상태 메시지 — 모든 핑거팁의 센서 데이터를 집계합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `fingertips` | `FingertipSensor[]` | 핑거팁 센서 배열 |

### `FingertipForceTorque.msg`

단일 핑거팁의 ONNX 모델 기반 힘/토크 추론 결과입니다 (`kFTValuesPerFingertip = 13`개 출력값에 대응).

| 필드 | 타입 | 설명 |
|------|------|------|
| `name` | `string` | 핑거팁 이름 (예: `"index"`, `"thumb"`) |
| `contact` | `bool` | 접촉 감지 플래그 |
| `force` | `float32[3]` | 힘 벡터 (Fx, Fy, Fz) [N] |
| `direction` | `float32[3]` | 단위 방향 벡터 (ux, uy, uz) |
| `normal_force` | `float32[3]` | 수직 힘 벡터 (Fnx, Fny, Fnz) [N] |
| `force_x` | `float32` | Fx 스칼라 성분 [N] |
| `force_y` | `float32` | Fy 스칼라 성분 [N] |
| `force_z` | `float32` | Fz 스칼라 성분 [N] |

**ONNX 모델 출력 레이아웃** (13 values):
```
[contact(1), F(3), u(3), Fn(3), Fx(1), Fy(1), Fz(1)]
```

> 대역폭 최적화를 위해 `Header` 필드를 포함하지 않습니다. 타임스탬프는 상위 `HandForceTorqueState.msg`의 header를 사용합니다.
> 벡터(`force`, `direction`, `normal_force`)와 스칼라(`force_x/y/z`) 이중 표현으로 다양한 사용처를 지원합니다.

### `HandForceTorqueState.msg`

전체 핸드의 힘/토크 추론 상태를 집계합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `fingertips` | `FingertipForceTorque[]` | 핑거팁 힘/토크 추론 배열 |

### `GuiPosition.msg`

GUI 디스플레이를 위한 현재 관절 및 태스크 공간 위치입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `joint_names` | `string[]` | 관절 이름 배열 |
| `joint_positions` | `float64[]` | 현재 관절 위치 (rad) |
| `task_positions` | `float64[6]` | TCP 위치 [x, y, z, roll, pitch, yaw] (FK) |

### `DeviceJointState.msg`

디바이스별 관절 상태 — 위치, 속도, 토크를 포함합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `joint_names` | `string[]` | 관절 이름 배열 |
| `positions` | `float64[]` | 관절 위치 (rad) |
| `velocities` | `float64[]` | 관절 속도 (rad/s) |
| `efforts` | `float64[]` | 관절 토크 (N·m) |

### `JointGoal.msg`

관절 공간 또는 태스크 공간의 목표 위치입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `goal_type` | `string` | `"joint"` 또는 `"task"` |
| `joint_names` | `string[]` | 관절 이름 배열 |
| `joint_goal` | `float64[]` | 관절 공간 목표 (rad) |
| `task_goal` | `float64[6]` | 태스크 공간 목표 [x, y, z, roll, pitch, yaw] |

### `DeviceStateLog.msg`

디바이스 상태, 커맨드, 목표, 궤적을 통합한 로깅 메시지입니다.

| 카테고리 | 필드 | 타입 | 설명 |
|---------|------|------|------|
| **상태** | `actual_positions` | `float64[]` | 현재 관절 위치 |
| | `actual_velocities` | `float64[]` | 현재 관절 속도 |
| | `efforts` | `float64[]` | 현재 관절 토크 |
| **커맨드** | `commands` | `float64[]` | 전송된 커맨드 |
| | `command_type` | `string` | `"position"` 또는 `"torque"` |
| **목표** | `joint_goal` | `float64[]` | 관절 공간 목표 |
| | `task_goal` | `float64[6]` | 태스크 공간 목표 |
| **궤적** | `trajectory_positions` | `float64[]` | 궤적 레퍼런스 위치 |
| | `trajectory_velocities` | `float64[]` | 궤적 레퍼런스 속도 |
| **FK** | `actual_task_positions` | `float64[6]` | 순기구학 TCP 위치 |

### `DeviceSensorLog.msg`

원시/필터링 센서 데이터와 추론 출력을 포함하는 로깅 메시지입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `sensor_names` | `string[]` | 센서 이름 배열 |
| `sensor_data_raw` | `int32[]` | 원시 센서 값 |
| `sensor_data` | `int32[]` | 필터링된 센서 값 |
| `inference_valid` | `bool` | 추론 출력 유효성 |
| `inference_output` | `float32[]` | 추론 결과 (F/T/contact) |

---

## 빌드

```bash
# 워크스페이스 루트에서
cd ~/ur_ws
colcon build --packages-select rtc_msgs
source install/setup.bash
```

빌드 후 생성된 메시지는 C++ (`rtc_msgs/msg/joint_command.hpp` 등) 및 Python (`rtc_msgs.msg.JointCommand` 등)에서 사용 가능합니다.

---

## 메시지 계층 구조

```
커맨드 (로봇으로 송신)                상태 (로봇에서 수신 — 원시 센서)
├── JointCommand (float64, 고정밀)    ├── HandMotorState (위치 + 속도)
└── HandCommand  (float32, 경량)      ├── DeviceJointState (위치 + 속도 + 토크)
                                      └── HandSensorState
                                          └── FingertipSensor[] (barometer + ToF + 추론)

목표 & GUI                            상태 (추론 결과)
├── JointGoal (관절/태스크 목표)        └── HandForceTorqueState
└── GuiPosition (관절 + TCP 위치)          └── FingertipForceTorque[] (접촉 + 힘/토크)

로깅
├── DeviceStateLog (상태 + 커맨드 + 목표 + 궤적 통합)
└── DeviceSensorLog (원시/필터 센서 + 추론 출력)
```

---

## 의존성 그래프 내 위치

**독립 패키지** — ROS2 패키지 의존성은 `std_msgs`만 존재합니다.

```
rtc_msgs   ← std_msgs (ROS2 기본 메시지)
    ↑
    ├── rtc_controller_interface (컨트롤러 타입 정의)
    ├── rtc_controller_manager   (JointCommand 퍼블리시)
    ├── rtc_status_monitor       (상태 모니터링)
    ├── rtc_mujoco_sim           (JointCommand 구독 → 시뮬레이션)
    └── ur5e_hand_driver         (핸드 센서/F·T 퍼블리시)
```

### 사용처

| 패키지 | 메시지 | 사용 방식 |
|--------|--------|----------|
| `rtc_controller_manager` | `JointCommand` | MuJoCo/외부 시뮬레이터에 관절 커맨드 퍼블리시 |
| `rtc_mujoco_sim` | `JointCommand` | 관절 커맨드 구독 → 물리 시뮬레이션 적용 |
| `ur5e_hand_driver` | `HandForceTorqueState` | ONNX F/T 추론 결과 퍼블리시 (`/hand/ft_state`) |
| `ur5e_hand_driver` | `HandSensorState` | 핑거팁 센서 데이터 퍼블리시 |
| `ur5e_bringup` | `HandCommand` | 데모 컨트롤러에서 핸드 모터 커맨드 |
| `rtc_controller_interface` | 전체 | 컨트롤러 타입 정의에서 메시지 타입 참조 |

---

## 최적화 내역 (v5.16.1)

| 영역 | 변경 내용 |
|------|----------|
| **코드 검증** | 메시지 정의 패키지 — rosidl 생성기 설정 확인 완료 |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
