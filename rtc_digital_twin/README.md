# rtc_digital_twin


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
│   ├── urdf_parser.py            <- URDF 파싱 + 조인트 분류/검증 유틸리티
│   ├── sensor_visualizer.py      <- 핑거팁 센서 -> MarkerArray 변환 (선택적)
│   ├── tcp_visualizer.py         <- TCP 위치/자세 MarkerArray + TF 시각화 (선택적)
│   └── joint_gui.py              <- Joint State Publisher GUI (Qt, 선택적)
├── launch/
│   └── digital_twin.launch.py    <- 일반화된 launch (URDF/xacro + config 인자)
├── config/
│   ├── digital_twin.yaml         <- robot-agnostic 기본값 (display_rate,
│   │                                  sensor_viz/tcp_viz scale 등)
│   │                                  ※ ARCH-1: robot-specific 값(URDF, source
│   │                                  topics, fingertip names)은 robot bringup
│   │                                  yaml에서 layered overlay로 주입 — 예:
│   │                                  integrated_bringup/config/ur5e_p1a/digital_twin.yaml
│   └── digital_twin.rviz         <- RViz2 디스플레이 설정
└── test/
    ├── test_urdf_parser.py         <- URDF 분류/검증 단위 테스트
    ├── test_sensor_visualizer.py   <- FingertipSensor → MarkerArray 변환
    ├── test_tcp_visualizer.py      <- TCP pose ↔ marker/TF, 회전 변환 수학
    ├── test_joint_state_cache.py   <- JointStateCache static/dynamic 모드
    └── test_node_construction.py   <- DigitalTwinNode 생성/파라미터 선언 스모크 테스트
```

---

## 데이터 흐름

```
[rtc_controller_manager (C++, RT @ control_rate; default 500Hz)]
  /joint_states (BE/2) -> DeviceJointStateCallback
       ├── device_states_ 업데이트 (기존)
       └── forward -> /rtc_cm/{group}/joint_states (RELIABLE/10)

[digital_twin_node (Python)]
  /rtc_cm/{group}/joint_states (RELIABLE/10) ──┐
                                                ├-> merge -> /digital_twin/joint_states
  /rtc_cm/{group}/joint_states (RELIABLE/10) ──┘     │
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
| `source_N.topic` (YAML 정의) | `sensor_msgs/JointState` | RELIABLE, depth=10 | rtc_controller_manager가 republish한 조인트 상태 |
| `sensor_viz.sensor_topic` (선택) | `rtc_msgs/HandSensorState` | RELIABLE, depth=10 | 핑거팁 센서 데이터 |
| `controller_tf.active_controller_topic` (기본 `/rtc_cm/active_controller_name`) | `std_msgs/String` | TRANSIENT_LOCAL, RELIABLE, depth=1 | 능동 컨트롤러 이름 추종 -> `<active>/transforms` 구독을 rewire |
| `/<active>/transforms` (rewire됨) | `tf2_msgs/TFMessage` | RELIABLE, depth=10 | 능동 컨트롤러의 FK `_actual` 프레임. restamp 후 `/tf`로 재발행 |
| `/tf`, `/tf_static` (TF) | `tf2_msgs/TFMessage` | tf2 기본 | TCP 시각화용 `tcp_viz.frame_id` -> `tcp_viz.source_topic`(child frame) transform. 토픽 구독이 아닌 `lookup_transform()`. 이 `/tf`는 아래 controller_tf 재발행이 채운다 |

### 퍼블리시

| 토픽 | 타입 | 주파수 | 설명 |
|------|------|--------|------|
| `/digital_twin/joint_states` (기본값) | `sensor_msgs/JointState` | 60 Hz (설정 가능) | 병합된 관절 상태 -> robot_state_publisher |
| `/tf` (controller_tf 재발행) | `tf2_msgs/TFMessage` | 능동 컨트롤러 발행률 | `<active>/transforms`의 `_actual` 프레임을 node clock으로 restamp 후 재발행 (RViz TF 디스플레이 + tcp_viz lookup 해결). `controller_tf.rebroadcast_enable=false`로 비활성 |
| `sensor_viz.marker_topic` (선택) | `visualization_msgs/MarkerArray` | 60 Hz (설정 가능) | 핑거팁 센서 시각화 마커 |
| `tcp_viz.marker_topic` (선택) | `visualization_msgs/MarkerArray` | 60 Hz (설정 가능) | TCP 위치 구체 + RGB 자세 축 마커 |

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
- **Controller TF 재발행** (`controller_tf.rebroadcast_enable`, 기본 활성): `/rtc_cm/active_controller_name`을 추종해 능동 컨트롤러의 `<config_key>/transforms`(FK `_actual` 프레임)를 구독하고, 각 transform을 node clock으로 restamp 후 `/tf`로 재발행한다. 컨트롤러는 전용 `/tf` publisher 없이 이 토픽으로만 transform을 노출하므로, 재발행 없이는 RViz(`/tf`만 구독)와 tcp_viz의 bare `TransformListener`가 `_actual` 프레임을 보지 못한다. 컨트롤러 전환 시 구독을 destroy+recreate로 rewire (robot-agnostic — 컨트롤러 키 하드코딩 없음, ARCH-1). 비활성화하면 순수 URDF twin

### UrdfParser (`urdf_parser.py`)

`UrdfParser` 클래스가 URDF/xacro 파일 경로(`UrdfParser.from_file(path)`, xacro 자동 감지) 또는 XML 문자열(`UrdfParser.from_xml(xml)`)로부터 파싱 파이프라인(링크 수집 -> 조인트 파싱 -> adjacency graph -> 분류)을 실행합니다.

- `.classification` (property): `JointClassification` 반환 — `active`, `passive_mimic`, `passive_closed_chain`, `fixed` 관절로 분류
  - `active`: 외부 JointState 데이터가 필요한 관절
  - `passive_mimic`: `<mimic>` 태그가 있는 관절 (마스터 조인트로부터 자동 계산)
  - `passive_closed_chain`: 동일 child_link를 공유하는 폐쇄 루프 관절
  - `fixed`: 고정 관절 (분류 대상에서 제외)
  - `.active_names` / `.passive_names`: 이름 `set` 프로퍼티
- `.compute_mimic_positions(joint_positions)`: mimic 관절 위치 자동 계산 (`multiplier * master + offset`)
- `.validate_joints(received)`: 커버리지 비교 -> `(covered, missing)` set 튜플 반환
- `.urdf_xml` / `.robot_name` / `.link_nodes` / `.get_active_joint_names()` 등 추가 접근자

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
- 접촉 판정: `inference_enable == True`이고 (`contact_flag >= 0.1` (threshold) **또는** force fallback)
- **Force fallback** (`sensor_viz.force_display_threshold`, 기본 0.0 = 비활성): contact classifier가 없어 `contact_flag ≡ 0`인 로봇(예: proto_1b)에서 `|F| >= threshold`이면 F Arrow를 표시한다. 이때 displacement `u ≡ 0`이면 zero-length arrow 대신 DELETE, contact Sphere는 적색으로 강제한다. `> 0`은 bringup yaml에서만 설정(ARCH-1)
- 마커 프레임: `{fingertip_name}_tip_link`, 수명: 100ms — `fingertip_names`가 실제 URDF fingertip link 접두사(예: proto_1b는 `l_`)와 일치해야 RViz가 마커를 렌더한다

### TcpVisualizer (`tcp_visualizer.py`) -- 선택적

**Phase 4에서 `GuiPosition` 메시지 구독 방식이 제거되고 tf2 조회로 대체되었습니다** (`GuiPosition` 메시지 자체도 폐기됨). YAML에 `tcp_viz.source_topic`이 비어있지 않을 때 활성화되며, 매 표시 주기마다 `tf2_ros.Buffer.lookup_transform(tcp_viz.frame_id, tcp_viz.source_topic, ...)`로 TCP pose를 조회해 RViz2 마커로 변환합니다. 이 lookup이 해결되려면 `_actual` 프레임이 `/tf`에 있어야 하는데, 컨트롤러는 전용 `/tf` publisher 없이 `<config_key>/transforms` (`tf2_msgs/TFMessage`)로만 노출하므로, DigitalTwinNode의 **controller_tf 재발행**(기본 활성)이 그 프레임을 `/tf`로 올려줘야 tcp_viz의 bare `TransformListener`가 받는다 (상세: [agent_docs/architecture.md](../agent_docs/architecture.md)).

**`tcp_viz.source_topic`은 더 이상 토픽 이름이 아니라 TF child frame 이름**입니다 (예: `"tool0_actual"`). `/`가 포함되지 않은 값은 그대로 child frame으로 쓰이고, `/`가 포함된 legacy 값(과거 절대 토픽 경로 형태)은 활성화 플래그로만 취급되어 child frame은 기본값 `"tool0_actual"`로 폴백합니다.

**시각화 마커:**

| 마커 | 타입 | 설명 |
|------|------|------|
| TCP Sphere | Sphere | 현재 TCP 위치 (파란색, 불투명) |
| Goal Sphere | Sphere | 목표 TCP 위치 (반투명, `show_goal: true` 시) — `TcpVisualizer.create_markers_from_tf`는 `goal_positions` 인자를 받지만 `digital_twin_node`가 현재 이를 채우는 소스를 연결하지 않아 항상 미표시 |
| RGB Axes | Arrow × 3 | TCP 자세 방향 (R=X, G=Y, B=Z) |

- 선택적 TF 재브로드캐스팅: `broadcast_tf: true` 시 조회한 pose를 `tf_child_frame` (기본 `virtual_tcp`) 프레임으로 재발행
- 마커 `header.frame_id` 및 tf2 lookup의 parent frame은 모두 `tcp_viz.frame_id` (기본 `"base"`)

```yaml
# YAML tcp_viz 설정 예시
tcp_viz.source_topic: "tool0_actual"   # TF child frame 이름 (토픽 아님)
tcp_viz.frame_id: "base"               # TF parent frame + 마커 frame_id
tcp_viz.marker_topic: "/digital_twin/tcp_markers"
tcp_viz.broadcast_tf: true
tcp_viz.tf_child_frame: "virtual_tcp"  # 재브로드캐스팅 시 child frame 이름
tcp_viz.sphere_radius: 0.012
tcp_viz.axes_length: 0.05
tcp_viz.axes_shaft: 0.004
tcp_viz.show_goal: true
tcp_viz.goal_sphere_radius: 0.010
```

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

> **ARCH-1**: 이 패키지의 default yaml 은 robot-agnostic 값만 보유합니다 (display_rate, sensor_viz/tcp_viz scale 등). robot identity 부분(URDF 패키지/경로, 소스 토픽, 센서 토픽, 핑거팁 이름, TCP 소스)은 `<robot>_bringup/config/digital_twin_<robot>.yaml` 에 두고 launch 의 `config_file:=` 인자로 layered overlay 합니다.
>
> UR5e+hand 풀 셋업 예시: `integrated_bringup/config/ur5e_p1a/digital_twin.yaml`
>
> ```bash
> ros2 launch rtc_digital_twin digital_twin.launch.py \
>     config_file:=$(ros2 pkg prefix --share integrated_bringup)/config/ur5e_p1a/digital_twin.yaml
> ```

기본 yaml 의 주요 키 (robot bringup yaml 에서 override):

```yaml
/**:
  ros__parameters:
    # -- 로봇 기술 파일 (robot bringup yaml이 채움) --
    robot_description_package: ""               # 예: "robot_descriptions"
    robot_description_path: ""                  # 패키지 내 상대 경로
    robot_description_file: ""                  # 또는 절대 경로

    # -- 노드 파라미터 (robot-agnostic) --
    display_rate: 60.0                          # Hz — 퍼블리시 주기
    output_topic: "/digital_twin/joint_states"  # 병합 JointState 출력 토픽
    auto_compute_mimic: true                    # URDF mimic 조인트 자동 계산
    closure_path: ""                            # Extended-URDF 폐쇄 체인 (loop-closed 핸드 전용)

    # -- Controller TF 재발행 (active-controller -> /tf) --
    controller_tf.rebroadcast_enable: true      # <active>/transforms 를 /tf 로 재발행
    controller_tf.active_controller_topic: "/rtc_cm/active_controller_name"

    # -- JointState 소스 (robot bringup yaml이 채움) --
    num_sources: 0                              # 기본 0; bringup이 1+ 로 override
    # source_N.topic / source_N.joint_names

    # -- Joint State Publisher GUI (선택) --
    joint_gui.enabled: false                    # true = launch GUI node
    joint_gui.output_topic: "/joint_gui/joint_states"   # JointState 출력 토픽
    joint_gui.publish_rate: 10.0                # Hz — GUI 퍼블리시 주기

    # -- 센서 시각화 (robot bringup yaml이 채움) --
    sensor_viz.sensor_topic: ""                 # 예: "/<hand>/sensor_states/monitor"
    sensor_viz.marker_topic: "/digital_twin/fingertip_markers"
    sensor_viz.fingertip_names: []              # 예: ["thumb","index","middle","ring"]

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

    # Force fallback: contact_flag 없는 로봇용 |F| 게이트
    sensor_viz.force_display_threshold: 0.0     # N — 0=비활성; >0은 bringup yaml (ARCH-1)
```

`sensor_viz.sensor_topic`을 빈 문자열로 설정하거나 해당 블록을 생략하면 센서 시각화가 비활성화됩니다.

---

## 실행

```bash
# UR5e + 핸드 (패키지 기반 xacro)
ros2 launch rtc_digital_twin digital_twin.launch.py \
    robot_description_package:=robot_descriptions \
    robot_description_path:=robots/ur5e/urdf/ur5e_with_hand.urdf.xacro

# 임의의 로봇 (절대 경로 URDF)
ros2 launch rtc_digital_twin digital_twin.launch.py \
    robot_description_file:=/path/to/robot.urdf

# 커스텀 설정 + RViz 비활성화
ros2 launch rtc_digital_twin digital_twin.launch.py \
    robot_description_package:=robot_descriptions \
    robot_description_path:=robots/ur5e/urdf/ur5e.urdf \
    config_file:=/path/to/my_config.yaml \
    use_rviz:=false

# 디스플레이 레이트 변경
ros2 launch rtc_digital_twin digital_twin.launch.py \
    robot_description_package:=robot_descriptions \
    robot_description_path:=robots/ur5e/urdf/ur5e.urdf \
    display_rate:=30.0

# Joint State Publisher GUI 활성화
ros2 launch rtc_digital_twin digital_twin.launch.py \
    robot_description_package:=robot_descriptions \
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
| `closure_path` | `''` (비활성) | Extended-URDF `<stem>.closure.yaml` 경로 (YAML `closure_path` 키로도 설정 가능) |

Launch 파일은 URDF/xacro를 처리하여 `robot_description` 문자열을 `robot_state_publisher`와 `digital_twin_node` 모두에 전달합니다. YAML에 `robot_description_package`/`robot_description_path`/`robot_description_file`을 설정해두면 launch 인자를 생략할 수 있습니다.

### Launch 노드 구성

1. **robot_state_publisher** -- URDF 로드, `/digital_twin/joint_states` 구독, TF 트리 퍼블리시 (namespace: `digital_twin`)
2. **digital_twin_node** -- 다중 JointState 소스 병합, URDF 검증, 센서 시각화
3. **rviz2** (조건부) -- 사전 설정된 디스플레이 (`use_rviz:=true` 시)
4. **joint_gui_node** (조건부) -- Joint State Publisher GUI (`use_joint_gui:=true` 시)
5. **closure_state_publisher** (조건부) -- Extended-URDF 폐쇄 체인 시각화 (`closure_path` 설정 시, [rtc_urdf_bridge](../rtc_urdf_bridge/README.md#nodes) 제공)

#### Extended-URDF 폐쇄 체인 시각화 (opt-in)

Loop-closed 핸드(예: linkage 핑거)는 spanning-tree URDF + `<stem>.closure.yaml` 짝으로 표현된다.
`closure_path` (launch arg 또는 bringup YAML 키) 설정 시:

- `digital_twin_node` 출력 → `/digital_twin/actuated_joint_states` (actuated 만 forward)
- `closure_state_publisher` (rtc_urdf_bridge) 추가 -- 측정된 actuated q 고정 + passive loop q 를
  closure 구속으로 풀어 **loop-consistent full q** 를 `/digital_twin/joint_states` 로 publish
- `robot_state_publisher` 가 이를 TF 로 전개 → RViz 에 loop 가 닫힌 채 렌더링

결합은 **토픽 전용**(build-time 의존 없음, ARCH-2). 미설정 시 현행 그대로 `digital_twin_node` 가
`/digital_twin/joint_states` 로 직접 publish 한다. Mimic-coupled 핸드(예: LEAP)는 미설정 유지.
경로가 절대경로가 아니면 `robot_description_package` share 기준으로 해석된다.

---

## URDF 조인트 검증

- 노드 시작 3초 후 첫 검증, 이후 10초 주기로 반복
- URDF에서 `fixed`, `mimic`, `closed-chain` 조인트를 자동 제외한 필수(active) 조인트 목록 생성
- 수신된 JointState의 joint_names와 비교하여 누락 시 WARN 로그 출력
- 모든 필수 조인트가 커버되면 INFO 로그 1회 출력 후 이후 누락 시에만 경고
- **Closure 모드**(`closure_path` 설정): loop-passive 조인트는 spanning-tree URDF 에서 active 로
  분류되지만 downstream `closure_state_publisher` 가 채우므로, 이 노드의 소스 책임이 아니다 →
  누락 WARN 을 억제하고 1회 INFO 로만 보고

---

## rtc_controller_manager 연동 (per-group JointState republish)

`rtc_controller_manager` 패키지의 `rtc_controller_manager` 노드는 각 디바이스 그룹의 JointState를 RELIABLE QoS로 자동 republish합니다:

| 원본 토픽 (BEST_EFFORT/2) | Republish 토픽 (RELIABLE/10) |
|---|---|
| `/joint_states` | `/rtc_cm/{group}/joint_states` |
| `/hand/joint_states` | `/rtc_cm/{group}/joint_states` |

- `DeviceJointStateCallback()`에서 수신 즉시 forward (최소 레이턴시)
- sensor executor 스레드에서 실행 (RT 루프 영향 없음)

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `rclpy` | ROS2 Python 클라이언트 |
| `sensor_msgs` | `JointState` 메시지 |
| `std_msgs` | `ColorRGBA` (센서 마커 색상), `String` (active_controller_name) |
| `visualization_msgs` | `Marker`, `MarkerArray` (센서 시각화) |
| `geometry_msgs` | `Point`, `Vector3` (마커 좌표/스케일) |
| `builtin_interfaces` | ROS2 기본 인터페이스 |
| `rtc_msgs` | `HandSensorState` (`FingertipSensor` 필드 포함, 센서 데이터). TCP pose는 Phase 4부터 tf2 lookup으로 대체 (`GuiPosition` 폐기) |
| `tf2_msgs` | `TFMessage` (`<active>/transforms` 구독 + `/tf` 재발행) |
| `tf2_ros` | TCP pose 조회 (`lookup_transform`) + TF 브로드캐스팅 (`TransformBroadcaster` — controller_tf 재발행 / 선택적 virtual_tcp) |
| `robot_state_publisher` | URDF -> TF 변환 (launch에서 사용) |
| `rviz2` | 3D 시각화 (launch에서 사용) |
| `xacro` | URDF 매크로 처리 (launch에서 사용) |
| `python_qt_binding` | Qt GUI 프레임워크 (Joint GUI) |
| `python3-yaml` | YAML 설정 파일 파싱 (launch에서 사용) |

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
