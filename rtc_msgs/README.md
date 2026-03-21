# rtc_msgs

![version](https://img.shields.io/badge/version-v5.16.0-blue)

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RT Controller 스택을 위한 **커스텀 ROS2 메시지 정의** 패키지입니다. 로봇 암 관절 커맨드, 핸드 모터 커맨드/피드백, 핑거팁 센서 데이터, 핑거팁 힘/토크 추론 결과를 위한 메시지 타입을 정의합니다.

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
    └── HandForceTorqueState.msg  ← 전체 핸드 힘/토크 상태
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

단일 핑거팁 센서 데이터 메시지 — 8개 기압(barometer) 센서와 3개 ToF 센서로 구성됩니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `name` | `string` | 핑거팁 이름 |
| `barometer` | `float32[8]` | 기압 센서 값 (하드웨어 고정 8개) |
| `tof` | `float32[3]` | ToF 거리 센서 값 (하드웨어 고정 3개) |

- 핑거팁 당 8개 barometer + 3개 ToF는 하드웨어 사양에 의해 고정되어 있습니다.

### `HandSensorState.msg`

전체 핸드 센서 상태 메시지 — 모든 핑거팁의 센서 데이터를 집계합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `fingertips` | `FingertipSensor[]` | 핑거팁 센서 배열 |

### `FingertipForceTorque.msg`

단일 핑거팁의 ONNX 모델 기반 힘/토크 추론 결과입니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `name` | `string` | 핑거팁 이름 |
| `force` | `float32[3]` | 추론된 힘 (Fx, Fy, Fz) |
| `torque` | `float32[3]` | 추론된 토크 (Tx, Ty, Tz) |

### `HandForceTorqueState.msg`

전체 핸드의 힘/토크 추론 상태를 집계합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header` | `std_msgs/Header` | 타임스탬프 및 프레임 ID |
| `fingertips` | `FingertipForceTorque[]` | 핑거팁 힘/토크 추론 배열 |

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

## 의존성 그래프 내 위치

**독립 패키지** — ROS2 패키지 의존성은 `std_msgs`만 존재합니다.

```
rtc_msgs   ← std_msgs (ROS2 기본 메시지)
    ↑
    ├── rtc_controller_interface (컨트롤러 타입 정의)
    ├── rtc_controller_manager   (JointCommand 구독)
    ├── rtc_status_monitor       (상태 모니터링)
    ├── rtc_mujoco_sim           (시뮬레이션 연동)
    └── ur5e_hand_driver         (핸드 커맨드/피드백)
```

### 사용처

| 패키지 | 사용 방식 |
|--------|----------|
| `rtc_controller_manager` | `JointCommand` 구독을 통한 관절 명령 수신 |
| `rtc_controller_interface` | 컨트롤러 타입 정의에서 메시지 타입 참조 |
| `rtc_status_monitor` | 상태 모니터링에서 핸드 센서/모터 상태 참조 |
| `rtc_mujoco_sim` | 시뮬레이션 연동 |
| `ur5e_hand_driver` | 핸드 모터 커맨드/피드백, 센서 데이터 |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
