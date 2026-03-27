# rtc_msgs

![version](https://img.shields.io/badge/version-v5.17.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크를 위한 **커스텀 ROS2 메시지 정의** 패키지입니다. 로봇 암 관절 커맨드, 핑거팁 센서 데이터 (원시/필터링), 추론 결과 (힘/변위/접촉), GUI 표시, 디바이스 상태/센서 로깅을 위한 **7종** 메시지 타입을 정의합니다.

**핵심 특징:**
- 로봇 비의존적(robot-agnostic) 메시지 설계
- 관절/태스크 공간 목표 지원 (JointCommand, RobotTarget)
- 핑거팁 센서: 원시(raw) + 필터링(filtered) 데이터 + ONNX 추론 출력 통합
- CSV 로깅 전용 메시지: 상태/커맨드/궤적 통합 (DeviceStateLog, DeviceSensorLog)
- C++ 및 Python 바인딩 자동 생성 (rosidl)

---

## 패키지 구조

```
rtc_msgs/
├── CMakeLists.txt
├── package.xml
├── CHANGELOG.md
└── msg/
    ├── JointCommand.msg       ← 로봇 암 관절 커맨드 (position/torque)
    ├── FingertipSensor.msg    ← 단일 핑거팁 센서 + 추론 결과
    ├── HandSensorState.msg    ← 전체 핸드 센서 상태 (핑거팁 집계)
    ├── GuiPosition.msg        ← GUI 표시용 관절/태스크 위치
    ├── RobotTarget.msg        ← 관절/태스크 공간 목표
    ├── DeviceStateLog.msg     ← 디바이스 상태 종합 로그
    └── DeviceSensorLog.msg    ← 디바이스 센서 로그
```

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `std_msgs` | `Header` 메시지 타입 (타임스탬프 + 프레임 ID) |
| `ament_cmake` | 빌드 시스템 |
| `rosidl_default_generators` | 메시지 C++/Python 코드 생성 |

---

## 메시지 정의

### `JointCommand.msg`

로봇 암 관절 커맨드 메시지 — position 또는 torque 모드를 지원합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `joint_names` | `string[]` | 관절 이름 배열 (비어있으면 기본 순서 fallback) |
| `values` | `float64[]` | 커맨드 값 배열 (라디안 또는 N·m) |
| `command_type` | `string` | `"position"` 또는 `"torque"` |

- `joint_names`와 `values`는 1:1 대응 관계입니다.

---

### `FingertipSensor.msg`

단일 핑거팁 센서 데이터 메시지 — 8개 기압(barometer) 센서, 3개 ToF 센서의 원시/필터링 값과 ONNX 추론 결과를 포함합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `name` | `string` | 핑거팁 이름 (예: `"index"`, `"thumb"`) |
| `barometer` | `float32[8]` | 기압 센서 값 — 필터링됨 (post-LPF) |
| `tof` | `float32[3]` | ToF 거리 센서 값 — 필터링됨 (post-LPF) |
| `barometer_raw` | `float32[8]` | 기압 센서 값 — 원시 (pre-LPF) |
| `tof_raw` | `float32[3]` | ToF 거리 센서 값 — 원시 (pre-LPF) |
| `f` | `float32[3]` | 추정 힘 벡터 [Fx, Fy, Fz] (ONNX 모델 출력) |
| `u` | `float32[3]` | 추정 변위 벡터 [ux, uy, uz] (ONNX 모델 출력) |
| `contact_flag` | `float32` | 접촉 감지 플래그 (0.0 = 비접촉, 1.0 = 접촉) |
| `inference_enable` | `bool` | 캘리브레이션 완료 여부 (`true` = 추론 결과 유효) |

- 핑거팁 당 8 barometer + 3 ToF는 하드웨어 사양에 의해 고정 (`kSensorValuesPerFingertip = 11`)
- 대역폭 최적화를 위해 `Header` 필드를 포함하지 않으며, 상위 `HandSensorState`의 header를 사용합니다
- 원시(raw)와 필터링(filtered) 데이터를 동시에 전달하여 진단/디버깅 지원

---

### `HandSensorState.msg`

전체 핸드 센서 상태 메시지 — 모든 핑거팁의 센서 데이터를 집계합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `fingertips` | `FingertipSensor[]` | 핑거팁 센서 배열 (YAML 설정에 의해 크기 결정) |

---

### `GuiPosition.msg`

GUI 디스플레이를 위한 현재 관절 및 태스크 공간 위치입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `joint_names` | `string[]` | 관절 이름 배열 |
| `joint_positions` | `float64[]` | 현재 관절 위치 (rad) |
| `task_positions` | `float64[6]` | TCP 위치 [x, y, z, roll, pitch, yaw] (FK 결과) |

- `joint_names`와 `joint_positions`는 1:1 대응, `task_positions`는 항상 6개 고정값입니다.

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
- CSV 열 순서: **Goal → Current State → Command → Trajectory** 분류법을 따릅니다.

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

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select rtc_msgs
source install/setup.bash
```

빌드 후 생성된 메시지는 C++ (`rtc_msgs/msg/joint_command.hpp` 등) 및 Python (`rtc_msgs.msg.JointCommand` 등)에서 사용 가능합니다.

---

## 메시지 계층 구조

```
커맨드 (로봇으로 송신)                센서 상태 (하드웨어에서 수신)
└── JointCommand                    └── HandSensorState
    (float64, position/torque)          └── FingertipSensor[]
                                            ├── barometer[8] + tof[3] (filtered)
                                            ├── barometer_raw[8] + tof_raw[3] (raw)
                                            └── f[3] + u[3] + contact_flag (추론)

목표 & GUI                          로깅 (CSV 세션 기록)
├── RobotTarget                     ├── DeviceStateLog
│   (관절/태스크 공간 목표)              │   (상태 + 커맨드 + 목표 + 궤적 통합)
└── GuiPosition                     └── DeviceSensorLog
    (관절 + TCP 위치)                   (원시/필터 센서 + 추론 출력)
```

---

## 의존성 그래프 내 위치

**독립 패키지** — ROS2 패키지 의존성은 `std_msgs`만 존재합니다.

```
rtc_msgs   ← std_msgs (ROS2 기본 메시지)
    ↑
    ├── rtc_controller_interface (컨트롤러 타입 정의)
    ├── rtc_controller_manager   (JointCommand/DeviceStateLog/DeviceSensorLog 퍼블리시)
    ├── rtc_status_monitor       (상태 모니터링)
    ├── rtc_mujoco_sim           (JointCommand 구독 → 시뮬레이션)
    ├── rtc_tools                (Python 유틸리티)
    ├── ur5e_hand_driver         (HandSensorState 퍼블리시)
    ├── ur5e_hand_status_monitor (핸드 상태 모니터링)
    ├── ur5e_bringup             (데모 컨트롤러)
    └── rtc_controllers          (컨트롤러 구현)
```

### 사용처

| 패키지 | 메시지 | 사용 방식 |
|--------|--------|----------|
| `rtc_controller_manager` | `JointCommand` | 관절 커맨드 퍼블리시 |
| `rtc_controller_manager` | `DeviceStateLog`, `DeviceSensorLog` | CSV 로깅 데이터 퍼블리시 |
| `rtc_controller_manager` | `GuiPosition`, `RobotTarget` | GUI 위치/목표 퍼블리시 |
| `rtc_mujoco_sim` | `JointCommand` | 관절 커맨드 구독 → 물리 시뮬레이션 |
| `ur5e_hand_driver` | `HandSensorState` | 핑거팁 센서 데이터 퍼블리시 |
| `rtc_controller_interface` | 전체 | 컨트롤러 타입 정의에서 메시지 타입 참조 |
| `ur5e_bringup` | `JointCommand`, `RobotTarget` | 데모 컨트롤러 |

---

## 변경 내역

### v5.17.0

| 영역 | 변경 내용 |
|------|----------|
| **DeviceStateLog** | 모터 공간 필드 추가 (`motor_names`, `motor_positions`, `motor_velocities`, `motor_efforts`) |
| **DeviceSensorLog** | 핑거팁 센서 로깅 + 추론 출력 통합 메시지 신규 추가 |
| **GuiPosition** | 관절/태스크 위치 통합 GUI 표시 메시지 신규 추가 |
| **RobotTarget** | 관절/태스크 공간 목표 메시지 신규 추가 |

### v5.16.1

| 영역 | 변경 내용 |
|------|----------|
| **코드 검증** | 메시지 정의 패키지 — rosidl 생성기 설정 확인 완료 |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
