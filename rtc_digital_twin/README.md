# rtc_digital_twin

![version](https://img.shields.io/badge/version-v5.17.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **일반화된 RViz2 디지털 트윈 시각화** 패키지입니다. YAML로 정의된 JointState 토픽을 구독하고, URDF/xacro 기반으로 **어떤 로봇이든** RViz2에서 실시간 시각화합니다.

**핵심 기능:**
- YAML 설정 기반 다중 JointState 소스 구독 (RELIABLE, depth=10)
- URDF 조인트 검증 — fixed/mimic 제외 필수 조인트 커버리지 자동 확인
- 다중 소스 병합 → 단일 `JointState` 퍼블리시 (robot_state_publisher용)
- URDF 기반 TF 트리 자동 구성 (`robot_state_publisher`)
- 센서 시각화 선택적 활성화 (YAML에 `sensor_viz` 블록 존재 시)
- 설정 가능한 디스플레이 레이트 (기본 60 Hz)

---

## 패키지 구조

```
rtc_digital_twin/
├── package.xml
├── setup.py
├── setup.cfg
├── rtc_digital_twin/
│   ├── __init__.py
│   ├── digital_twin_node.py      ← 메인 노드 (다중 소스 병합 + URDF 검증)
│   ├── urdf_validator.py         ← URDF 파싱 + 조인트 검증 유틸리티
│   └── sensor_visualizer.py      ← 핑거팁 센서 → MarkerArray 변환 (선택적)
├── launch/
│   └── digital_twin.launch.py    ← 일반화된 launch (URDF/xacro + config 인자)
└── config/
    ├── digital_twin.yaml         ← 파라미터 설정 (source_N.* 구조)
    └── digital_twin.rviz         ← RViz2 디스플레이 설정
```

---

## 데이터 흐름

```
[rt_controller (C++, 500Hz)]
  /joint_states (BE/2) → DeviceJointStateCallback
       ├── device_states_ 업데이트 (기존)
       └── forward → /{group}/digital_twin/joint_states (RELIABLE/10)

[digital_twin_node (Python)]
  /{group}/digital_twin/joint_states (RELIABLE/10) ──┐
                                                      ├→ merge → /digital_twin/joint_states
  /{group}/digital_twin/joint_states (RELIABLE/10) ──┘     │
                                                            ├→ URDF 검증 (로그)
                                                            └→ (선택) sensor_viz → MarkerArray

[robot_state_publisher]
  /digital_twin/joint_states → TF tree → RViz2
```

---

## ROS2 인터페이스

### 구독

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `source_N.topic` (YAML 정의) | `JointState` | RELIABLE, depth=10 | rt_controller가 republish한 조인트 상태 |
| `sensor_viz.sensor_topic` (선택) | `HandSensorState` | RELIABLE, depth=10 | 핑거팁 센서 데이터 (rtc_msgs) |

### 퍼블리시

| 토픽 | 타입 | 주파수 | 설명 |
|------|------|--------|------|
| `/digital_twin/joint_states` | `JointState` | 60 Hz | 병합된 관절 상태 → robot_state_publisher |
| `sensor_viz.marker_topic` (선택) | `MarkerArray` | 60 Hz | 센서 시각화 마커 |

### URDF 조인트 검증

- 노드 시작 3초 후 첫 검증, 이후 10초 주기
- URDF에서 `fixed`, `mimic` 조인트를 자동 제외한 필수 조인트 목록 생성
- 수신된 JointState의 joint_names와 비교하여 누락 시 WARN 로그 출력

---

## YAML 설정

```yaml
/**:
  ros__parameters:
    display_rate: 60.0                          # Hz
    output_topic: "/digital_twin/joint_states"  # RSP가 구독할 토픽
    num_sources: 1

    # source_N.topic — 구독할 JointState 토픽
    # source_N.joint_names — 빈 배열이면 토픽의 모든 조인트 수용
    source_0.topic: "/ur5e/digital_twin/joint_states"
    source_0.joint_names: []

    # (선택) 센서 시각화 — 이 블록이 없으면 비활성화
    # sensor_viz.sensor_topic: "/hand/sensor_states"
    # sensor_viz.marker_topic: "/digital_twin/fingertip_markers"
    # sensor_viz.fingertip_names: ["thumb", "index", "middle", "ring"]
    # sensor_viz.barometer_min: 0.0
    # sensor_viz.barometer_max: 1000.0
    # sensor_viz.barometer_sphere_min: 0.002
    # sensor_viz.barometer_sphere_max: 0.008
    # sensor_viz.tof_max_distance: 0.2
    # sensor_viz.tof_arrow_scale: 0.003
```

---

## 실행

```bash
# UR5e + 핸드 (패키지 기반 xacro)
ros2 launch rtc_digital_twin digital_twin.launch.py \
    robot_description_package:=ur5e_description \
    robot_description_path:=robots/ur5e/urdf/ur5e_with_hand.urdf.xacro

# 임의의 로봇 (절대 경로 URDF)
ros2 launch rtc_digital_twin digital_twin.launch.py \
    robot_description_file:=/path/to/robot.urdf

# 커스텀 설정 + RViz 비활성화
ros2 launch rtc_digital_twin digital_twin.launch.py \
    robot_description_package:=ur5e_description \
    robot_description_path:=robots/ur5e/urdf/ur5e.urdf \
    config_file:=/path/to/my_config.yaml \
    use_rviz:=false

# 디스플레이 레이트 변경
ros2 launch rtc_digital_twin digital_twin.launch.py \
    robot_description_package:=ur5e_description \
    robot_description_path:=robots/ur5e/urdf/ur5e.urdf \
    display_rate:=30.0
```

### Launch Arguments

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `robot_description_package` | `''` | URDF 포함 패키지명 |
| `robot_description_path` | `''` | 패키지 내 상대 경로 |
| `robot_description_file` | `''` | 절대 경로 (패키지 대안) |
| `config_file` | 내장 기본값 | digital_twin YAML 경로 |
| `use_rviz` | `true` | RViz2 실행 여부 |
| `rviz_config` | 내장 기본값 | RViz 설정 파일 경로 |
| `display_rate` | `''` | YAML override |

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `rclpy` | ROS2 Python 클라이언트 |
| `sensor_msgs` | JointState 메시지 |
| `std_msgs` | Float64MultiArray |
| `visualization_msgs` | MarkerArray |
| `geometry_msgs` | Point, Vector3, Quaternion |
| `robot_state_publisher` | URDF → TF 변환 |
| `rviz2` | 3D 시각화 |
| `xacro` | URDF 매크로 처리 |

---

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select rtc_digital_twin
source install/setup.bash
```

---

## 주요 컴포넌트

### DigitalTwinNode (`digital_twin_node.py`)

- YAML의 `source_N.*` 설정으로 다중 JointState 토픽 구독 (RELIABLE, depth=10)
- `JointStateCache` 데이터클래스로 소스별 상태 캐싱 및 이름→인덱스 매핑
- `joint_names` 빈 배열 시 dynamic 모드: 수신 메시지의 모든 조인트를 자동 수용
- 타이머 콜백에서 모든 소스를 병합하여 단일 `JointState` 퍼블리시
- `robot_description` 파라미터에서 URDF를 파싱하여 필수 조인트 검증 수행

### URDFValidator (`urdf_validator.py`)

- `classify_joints(urdf_xml)`: URDF XML을 파싱하여 `active`, `passive_mimic`, `passive_closed_chain`, `fixed` 관절로 분류
- `parse_required_joints(urdf_xml)`: active 관절 이름 set 추출 (fixed/mimic/closed-chain 제외)
- `compute_mimic_positions(classification, positions)`: mimic 관절 위치 자동 계산 (multiplier × master + offset)
- `validate_joints(required, received)`: 커버리지 비교 → (covered, missing) 반환

### SensorVisualizer (`sensor_visualizer.py`) — 선택적

- YAML에 `sensor_viz` 블록이 있을 때만 활성화
- 핑거팁별 8개 기압 센서 → 크기/색상 비례 Sphere 마커 (2×4 격자 배치)
- 핑거팁별 3개 ToF 센서 → 길이/색상 비례 Arrow 마커 (Z축 방향)
- 마커 프레임: `{fingertip_name}_tip_link`, 수명: 100 ms

---

## rt_controller 연동 (Digital Twin Republish)

`rtc_controller_manager`의 `rt_controller` 노드는 각 디바이스 그룹의 JointState를 RELIABLE QoS로 자동 republish합니다:

| 원본 토픽 (BEST_EFFORT/2) | Republish 토픽 (RELIABLE/10) |
|---|---|
| `/joint_states` | `/{group}/digital_twin/joint_states` |
| `/hand/joint_states` | `/{group}/digital_twin/joint_states` |

- `DeviceJointStateCallback()`에서 수신 즉시 forward (최소 레이턴시)
- sensor executor 스레드에서 실행 (RT 루프 영향 없음)

---

## 센서 시각화 (선택적)

핑거팁 당 11개 센서 (8 barometer + 3 ToF):

| 센서 | 마커 타입 | 스케일링 | 범위 |
|------|----------|----------|------|
| Barometer (×8) | Sphere | 크기+색상 ∝ 압력 | 0–1000 Pa |
| ToF (×3) | Arrow | 길이 ∝ 거리 | 0–0.2 m |

색상: 파랑(낮음) → 초록 → 빨강(높음) 히트맵

YAML에 `sensor_viz` 블록을 추가하면 활성화, 생략하면 JointState 시각화만 수행합니다.

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
