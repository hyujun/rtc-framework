# rtc_digital_twin

![version](https://img.shields.io/badge/version-v5.17.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **일반화된 RViz2 디지털 트윈 시각화** 패키지입니다. YAML로 정의된 다중 JointState 토픽을 구독하여 병합하고, URDF/xacro 기반으로 **어떤 로봇이든** RViz2에서 실시간 시각화합니다.

**핵심 기능:**
- YAML 설정 기반 다중 JointState 소스 구독 및 병합 (RELIABLE, depth=10)
- URDF 조인트 자동 분류 (active / passive_mimic / passive_closed_chain / fixed)
- Mimic 조인트 위치 자동 계산 (`multiplier * master + offset`)
- 필수 조인트 커버리지 주기적 검증 (3초 후 첫 검증, 이후 10초 주기)
- 병합된 단일 `JointState` 퍼블리시 (`robot_state_publisher`용)
- 핑거팁 센서 시각화 (barometer, ToF, Force, Displacement, Contact) -- YAML에 `sensor_viz` 블록 존재 시 활성화
- 설정 가능한 디스플레이 레이트 (기본 60 Hz)
- **Joint State Publisher GUI** — active 조인트를 degree 단위로 수동 조작 (slider + 직접 입력), URDF joint limit 반영

---

## 패키지 구조

```
rtc_digital_twin/
├── package.xml
├── setup.py
├── setup.cfg
├── rtc_digital_twin/
│   ├── __init__.py
│   ├── digital_twin_node.py      <- 메인 노드 (다중 소스 병합 + URDF 검증 + 센서 시각화)
│   ├── urdf_validator.py         <- URDF 파싱 + 조인트 분류/검증 유틸리티
│   ├── sensor_visualizer.py      <- 핑거팁 센서 -> MarkerArray 변환 (선택적)
│   └── joint_gui.py              <- Joint State Publisher GUI (Qt, 선택적)
├── launch/
│   └── digital_twin.launch.py    <- 일반화된 launch (URDF/xacro + config 인자)
├── config/
│   ├── digital_twin.yaml         <- 파라미터 설정 (source_N.* 구조)
│   └── digital_twin.rviz         <- RViz2 디스플레이 설정
└── test/
    └── test_urdf_validator.py    <- URDF 분류/검증 단위 테스트
```

---

## 데이터 흐름

```
[rt_controller (C++, 500Hz)]
  /joint_states (BE/2) -> DeviceJointStateCallback
       ├── device_states_ 업데이트 (기존)
       └── forward -> /{group}/digital_twin/joint_states (RELIABLE/10)

[digital_twin_node (Python)]
  /{group}/digital_twin/joint_states (RELIABLE/10) ──┐
                                                      ├-> merge -> /digital_twin/joint_states
  /{group}/digital_twin/joint_states (RELIABLE/10) ──┘     │
                                                            ├-> mimic 조인트 자동 계산
                                                            ├-> URDF 검증 (로그)
                                                            └-> (선택) sensor_viz -> MarkerArray

[robot_state_publisher]
  /digital_twin/joint_states -> TF tree -> RViz2
```

---

## ROS2 인터페이스

### 구독

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `source_N.topic` (YAML 정의) | `sensor_msgs/JointState` | RELIABLE, depth=10 | rt_controller가 republish한 조인트 상태 |
| `sensor_viz.sensor_topic` (선택) | `rtc_msgs/HandSensorState` | RELIABLE, depth=10 | 핑거팁 센서 데이터 |

### 퍼블리시

| 토픽 | 타입 | 주파수 | 설명 |
|------|------|--------|------|
| `/digital_twin/joint_states` (기본값) | `sensor_msgs/JointState` | 60 Hz (설정 가능) | 병합된 관절 상태 -> robot_state_publisher |
| `sensor_viz.marker_topic` (선택) | `visualization_msgs/MarkerArray` | 60 Hz (설정 가능) | 핑거팁 센서 시각화 마커 |

---

## 주요 컴포넌트

### DigitalTwinNode (`digital_twin_node.py`)

- YAML의 `source_N.*` 설정으로 다중 JointState 토픽 구독 (RELIABLE, depth=10)
- `JointStateCache` 데이터클래스로 소스별 상태 캐싱 및 이름->인덱스 매핑
- **Static 모드**: `joint_names`에 조인트 이름을 명시하면 해당 이름만 수신/매핑
- **Dynamic 모드**: `joint_names`가 빈 배열이면 수신 메시지의 모든 조인트를 자동 수용
- 타이머 콜백에서 모든 소스를 병합하여 단일 `JointState` 퍼블리시
- `robot_description` 파라미터에서 URDF를 파싱하여 조인트 분류 수행
- `auto_compute_mimic` 활성 시 mimic 조인트 위치를 자동 계산하여 JointState에 추가
- 데이터 미수신 시에도 URDF의 모든 조인트를 position=0으로 퍼블리시 (TF 트리 유지)

### URDFValidator (`urdf_validator.py`)

- `classify_joints(urdf_xml)`: URDF XML을 파싱하여 `active`, `passive_mimic`, `passive_closed_chain`, `fixed` 관절로 분류
  - `active`: 외부 JointState 데이터가 필요한 관절
  - `passive_mimic`: `<mimic>` 태그가 있는 관절 (마스터 조인트로부터 자동 계산)
  - `passive_closed_chain`: 동일 child_link를 공유하는 폐쇄 루프 관절
  - `fixed`: 고정 관절 (분류 대상에서 제외)
- `parse_required_joints(urdf_xml)`: active 관절 이름 set 추출
- `compute_mimic_positions(classification, positions)`: mimic 관절 위치 자동 계산 (`multiplier * master + offset`)
- `validate_joints(required, received)`: 커버리지 비교 -> `(covered, missing)` 반환

### SensorVisualizer (`sensor_visualizer.py`) -- 선택적

YAML에 `sensor_viz` 블록이 있을 때만 활성화됩니다. 핑거팁별로 다음 마커를 생성합니다:

| 센서 | 마커 타입 | 개수 | 스케일링 | 색상 |
|------|----------|------|----------|------|
| Barometer | Arrow (+Z) | 8개 (2x4 격자, 1mm 간격) | 길이 = 압력 비례 (최대 15mm) | 히트맵 (파랑->초록->빨강) |
| ToF | Arrow (+Z) | 3개 (X축 배치) | 길이 = 거리 비례 (최대 0.2m) | 거리 비례 색상 |
| Force (F) | Arrow (3D 방향) | 1개 | 길이 = 힘 크기 * scale | 빨간색 |
| Displacement (u) | Arrow (3D 방향) | 1개 | 길이 = 변위 크기 * scale | 시안색 |
| Contact | Sphere | 1개 | 고정 반지름 (기본 5mm) | 빨강(접촉)/초록(비접촉)/회색(미보정) |

- F/u Arrow는 접촉이 감지될 때만 표시, 비접촉 시 DELETE 처리
- 접촉 판정: `inference_enable == True`이고 `contact_flag >= 0.1` (threshold)
- 마커 프레임: `{fingertip_name}_tip_link`, 수명: 100ms

### JointGuiNode (`joint_gui.py`) -- 선택적

YAML의 `joint_gui.enabled: true` 또는 launch arg `use_joint_gui:=true` 시 활성화됩니다. 기존 ROS2 `joint_state_publisher_gui`와 유사한 Qt 기반 GUI를 제공합니다.

**기존 joint_state_publisher_gui와의 차이점:**

| 항목 | joint_state_publisher_gui | JointGuiNode |
|------|--------------------------|-------------|
| 값 단위 | radian | **degree** |
| 값 입력 | read-only 표시 | **editable** (Enter로 적용) |
| Joint limit | 있으면 적용 | 있으면 적용 + **limit 범위 표시** |
| Joint 필터 | 모든 non-fixed | **active만** (mimic/closed-chain 제외) |

**GUI 레이아웃:**

```
┌──────────────────────────────────────────────────────────┐
│ [Zero All]  [Center All]                                 │
│                                                          │
│ shoulder_pan_joint              [-180.0 ~ 180.0]  deg    │
│ [══════════●══════════════]     [___30.1___]              │
│                                                          │
│ shoulder_lift_joint             [-180.0 ~ 0.0]    deg    │
│ [══════════●══════════════]     [__-45.0___]              │
└──────────────────────────────────────────────────────────┘
```

- **Slider**: URDF joint limit 범위로 매핑 (revolute: rad→deg 변환, continuous: ±360 deg, prismatic: m 단위)
- **Entry**: 직접 숫자 입력, Enter 키로 적용, limit 범위 내로 clamp
- **Slider↔Entry**: 양방향 동기화
- **Zero All**: 모든 조인트를 0으로 설정
- **Center All**: 모든 조인트를 limit 중간값으로 설정
- **Publish**: `sensor_msgs/JointState` (deg→rad 변환하여 퍼블리시)

---

## YAML 설정 (`config/digital_twin.yaml`)

```yaml
/**:
  ros__parameters:
    # -- 로봇 기술 파일 (launch에서 사용) --
    robot_description_package: ""               # 패키지명 (예: "ur5e_description")
    robot_description_path: ""                  # 패키지 내 상대 경로
    robot_description_file: ""                  # 또는 절대 경로

    # -- 노드 파라미터 --
    display_rate: 60.0                          # Hz — 퍼블리시 주기
    output_topic: "/digital_twin/joint_states"  # 병합 JointState 출력 토픽
    auto_compute_mimic: true                    # URDF mimic 조인트 자동 계산

    # -- JointState 소스 --
    num_sources: 2
    source_0.topic: "/ur5e/digital_twin/joint_states"
    source_0.joint_names: [""]                  # 빈 문자열 = dynamic 모드 (모든 조인트 수용)
    source_1.topic: "/hand/digital_twin/joint_states"
    source_1.joint_names: [""]

    # -- Joint State Publisher GUI (선택) --
    joint_gui.enabled: false                    # true = launch GUI node
    joint_gui.output_topic: "/joint_gui/joint_states"   # JointState 출력 토픽
    joint_gui.publish_rate: 10.0                # Hz — GUI 퍼블리시 주기

    # -- 센서 시각화 (선택) --
    sensor_viz.sensor_topic: "/hand/sensor_states/monitor"
    sensor_viz.marker_topic: "/digital_twin/fingertip_markers"
    sensor_viz.fingertip_names: ["thumb", "index", "middle", "ring"]

    # Barometer: +Z Arrow (2x4 격자, 1mm 간격)
    sensor_viz.barometer_min: 0.0               # 최소 압력값
    sensor_viz.barometer_max: 1000.0            # 최대 압력값
    sensor_viz.barometer_arrow_max_length: 0.015  # 최대 화살표 길이 (m)
    sensor_viz.barometer_arrow_scale: 0.0008    # 화살표 shaft 직경 (m)

    # ToF: +Z Arrow (기본 비활성)
    sensor_viz.tof_enabled: false
    sensor_viz.tof_max_distance: 0.2            # 최대 거리 (m)
    sensor_viz.tof_arrow_scale: 0.003           # shaft 직경 (m)

    # Force (F): 3D Arrow
    sensor_viz.force_arrow_scale: 0.01          # 길이 배율 (m/N)
    sensor_viz.force_arrow_shaft: 0.003         # shaft 직경 (m)

    # Displacement (u): 3D Arrow
    sensor_viz.displacement_arrow_scale: 0.05   # 길이 배율 (m/unit)
    sensor_viz.displacement_arrow_shaft: 0.002  # shaft 직경 (m)

    # Contact: Sphere
    sensor_viz.contact_sphere_radius: 0.005     # 구 반지름 (m)
```

`sensor_viz.sensor_topic`을 빈 문자열로 설정하거나 해당 블록을 생략하면 센서 시각화가 비활성화됩니다.

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

# Joint State Publisher GUI 활성화
ros2 launch rtc_digital_twin digital_twin.launch.py \
    robot_description_package:=ur5e_description \
    robot_description_path:=robots/ur5e/urdf/ur5e_with_hand.urdf.xacro \
    use_joint_gui:=true
```

### Launch Arguments

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `robot_description_package` | `''` | URDF 포함 패키지명 |
| `robot_description_path` | `''` | 패키지 내 상대 경로 |
| `robot_description_file` | `''` | 절대 경로 (패키지 방식 대안) |
| `config_file` | 내장 기본값 | digital_twin YAML 경로 |
| `use_rviz` | `true` | RViz2 실행 여부 |
| `rviz_config` | 내장 기본값 | RViz 설정 파일 경로 |
| `display_rate` | `''` (YAML 값 사용) | YAML의 display_rate 오버라이드 |
| `use_joint_gui` | `false` | Joint State Publisher GUI 실행 여부 |

Launch 파일은 URDF/xacro를 처리하여 `robot_description` 문자열을 `robot_state_publisher`와 `digital_twin_node` 모두에 전달합니다. YAML에 `robot_description_package`/`robot_description_path`/`robot_description_file`을 설정해두면 launch 인자를 생략할 수 있습니다.

### Launch 노드 구성

1. **robot_state_publisher** -- URDF 로드, `/digital_twin/joint_states` 구독, TF 트리 퍼블리시 (namespace: `digital_twin`)
2. **digital_twin_node** -- 다중 JointState 소스 병합, URDF 검증, 센서 시각화
3. **rviz2** (조건부) -- 사전 설정된 디스플레이 (`use_rviz:=true` 시)
4. **joint_gui_node** (조건부) -- Joint State Publisher GUI (`use_joint_gui:=true` 시)

---

## URDF 조인트 검증

- 노드 시작 3초 후 첫 검증, 이후 10초 주기로 반복
- URDF에서 `fixed`, `mimic`, `closed-chain` 조인트를 자동 제외한 필수(active) 조인트 목록 생성
- 수신된 JointState의 joint_names와 비교하여 누락 시 WARN 로그 출력
- 모든 필수 조인트가 커버되면 INFO 로그 1회 출력 후 이후 누락 시에만 경고

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

## 의존성

| 의존성 | 용도 |
|--------|------|
| `rclpy` | ROS2 Python 클라이언트 |
| `sensor_msgs` | `JointState` 메시지 |
| `std_msgs` | `ColorRGBA` (센서 마커 색상) |
| `visualization_msgs` | `Marker`, `MarkerArray` (센서 시각화) |
| `geometry_msgs` | `Point`, `Vector3` (마커 좌표/스케일) |
| `builtin_interfaces` | ROS2 기본 인터페이스 |
| `rtc_msgs` | `HandSensorState`, `FingertipSensor` (센서 데이터) |
| `robot_state_publisher` | URDF -> TF 변환 (launch에서 사용) |
| `rviz2` | 3D 시각화 (launch에서 사용) |
| `xacro` | URDF 매크로 처리 (launch에서 사용) |
| `python_qt_binding` | Qt GUI 프레임워크 (Joint GUI) |

---

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select rtc_digital_twin
source install/setup.bash
```

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
