# Digital Twin RViz 구현 계획 (v2)

## 개요
실제 로봇(UR5e + Custom Hand) 구동 시 URDF 기반 Digital Twin을 RViz2로 60Hz 시각화.
**독립 패키지 `ur5e_digital_twin`**으로 구현하여 기존 제어기(`ur5e_rt_controller`)와 **별도로 실행** 가능.

- **Robot**: `/joint_states` → 6 joint positions → URDF TF 업데이트
- **Hand**: `/hand/joint_states` → 10 motor positions + 44 fingertip sensor → URDF TF + Marker 시각화
- **Display rate**: 60Hz (configurable)

---

## 설계 원칙

1. **독립 실행**: 제어기와 완전히 분리. 토픽만 구독하므로 제어기가 먼저 떠 있으면 언제든 실행/종료 가능
2. **별도 패키지**: `ur5e_digital_twin/` — 독립 빌드, 독립 launch
3. **기존 코드 무수정**: `ur5e_rt_controller`, `ur5e_hand_udp`, `ur5e_tools` 등 기존 패키지 변경 없음
4. **URDF는 ur5e_description에 추가**: Hand URDF + Combined URDF는 description 패키지의 역할

---

## 현재 상태 분석

| 항목 | 상태 |
|------|------|
| UR5e URDF | ✅ `ur5e_description/robots/ur5e/urdf/ur5e.urdf` (6 revolute joints) |
| Hand URDF | ❌ 미존재 (MJCF만 존재, mesh 없음) |
| RViz 설정 | ❌ 미존재 |
| Digital Twin 노드 | ❌ 미존재 |
| Digital Twin 패키지 | ❌ 미존재 |
| 토픽 구조 | ✅ `/joint_states` (500Hz), `/hand/joint_states` (Float64MultiArray: 64 doubles) |

---

## 패키지 구조

```
ur5e-rt-controller/
├── ur5e_msgs/                    # (기존) 메시지 정의
├── ur5e_description/             # (기존) + Hand URDF 추가
│   └── robots/ur5e/urdf/
│       ├── ur5e.urdf             # (기존)
│       ├── hand.urdf.xacro       # ★ 신규: Hand 모델
│       └── ur5e_with_hand.urdf.xacro  # ★ 신규: 통합 URDF
├── ur5e_rt_controller/           # (기존, 변경 없음)
├── ur5e_hand_udp/                # (기존, 변경 없음)
├── ur5e_mujoco_sim/              # (기존, 변경 없음)
├── ur5e_tools/                   # (기존, 변경 없음)
├── ur5e_rt_base/                 # (기존, 변경 없음)
├── ur5e_status_monitor/          # (기존, 변경 없음)
└── ur5e_digital_twin/            # ★ 신규 독립 패키지
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── ur5e_digital_twin
    ├── config/
    │   ├── digital_twin.yaml     # 노드 파라미터
    │   └── digital_twin.rviz     # RViz2 설정
    ├── launch/
    │   └── digital_twin.launch.py
    └── ur5e_digital_twin/
        ├── __init__.py
        ├── digital_twin_node.py  # 핵심 노드 (60Hz publisher)
        └── sensor_visualizer.py  # Fingertip sensor → MarkerArray
```

---

## Phase 1: Hand URDF 생성 (ur5e_description 패키지에 추가)

### 1-1. Hand 모델 URDF 작성
**파일**: `ur5e_description/robots/ur5e/urdf/hand.urdf.xacro`

핸드 메시가 없으므로 **기본 형상(cylinder/box)으로 간략 모델링**:

```
hand_base_link (tool0에 부착)
├── thumb_cmc_aa_link  ─ thumb_cmc_fe_link  ─ thumb_mcp_fe_link  ─ thumb_tip_link
├── index_mcp_aa_link  ─ index_mcp_fe_link  ─ index_dip_fe_link  ─ index_tip_link
├── middle_mcp_aa_link ─ middle_mcp_fe_link ─ middle_dip_fe_link ─ middle_tip_link
└── ring_mcp_fe_link   ─ ring_tip_link
```

- 10개 revolute joint → motor 이름과 1:1 매핑:
  `thumb_cmc_aa`, `thumb_cmc_fe`, `thumb_mcp_fe`,
  `index_mcp_aa`, `index_mcp_fe`, `index_dip_fe`,
  `middle_mcp_aa`, `middle_mcp_fe`, `middle_dip_fe`,
  `ring_mcp_fe`
- 4개 fingertip frame (`thumb_tip_link`, `index_tip_link`, `middle_tip_link`, `ring_tip_link`)
- 각 링크: cylinder (r=0.008, l=0.03~0.04) visual + collision
- palm(hand_base_link): box (0.08 × 0.10 × 0.025)

### 1-2. Combined URDF 생성
**파일**: `ur5e_description/robots/ur5e/urdf/ur5e_with_hand.urdf.xacro`

```xml
<robot xmlns:xacro="..." name="ur5e_with_hand">
  <xacro:include filename="$(find ur5e_description)/robots/ur5e/urdf/ur5e.urdf"/>
  <xacro:include filename="$(find ur5e_description)/robots/ur5e/urdf/hand.urdf.xacro"/>
  <joint name="tool0_to_hand" type="fixed">
    <parent link="tool0"/>
    <child link="hand_base_link"/>
  </joint>
</robot>
```

---

## Phase 2: 독립 패키지 `ur5e_digital_twin` 생성

### 2-1. 패키지 뼈대
- `package.xml` — ament_python 빌드, 의존성: rclpy, sensor_msgs, visualization_msgs, geometry_msgs, ur5e_msgs, ur5e_description, robot_state_publisher, rviz2, xacro
- `setup.py` — entry_points, data_files (config/, launch/)
- `setup.cfg` — install scripts 경로
- `resource/ur5e_digital_twin` — ament index marker

### 2-2. 설정 파일
**파일**: `ur5e_digital_twin/config/digital_twin.yaml`

```yaml
/**:
  ros__parameters:
    display_rate: 60.0          # Hz — RViz 갱신 주기
    enable_hand: true           # 핸드 시각화 활성화

    # 구독 토픽
    robot_joint_states_topic: "/joint_states"
    hand_joint_states_topic: "/hand/joint_states"

    # 로봇 관절 이름 (URDF joint 순서)
    robot_joint_names:
      - "shoulder_pan_joint"
      - "shoulder_lift_joint"
      - "elbow_joint"
      - "wrist_1_joint"
      - "wrist_2_joint"
      - "wrist_3_joint"

    # 핸드 모터 이름 (URDF joint 순서, Float64MultiArray 인덱스와 1:1)
    hand_motor_names:
      - "thumb_cmc_aa"
      - "thumb_cmc_fe"
      - "thumb_mcp_fe"
      - "index_mcp_aa"
      - "index_mcp_fe"
      - "index_dip_fe"
      - "middle_mcp_aa"
      - "middle_mcp_fe"
      - "middle_dip_fe"
      - "ring_mcp_fe"

    # 핑거팁 이름 + 센서 시각화
    fingertip_names:
      - "thumb"
      - "index"
      - "middle"
      - "ring"

    # 센서 시각화 설정
    sensor_viz:
      barometer_min: 0.0        # 최소 압력값 (정규화 하한)
      barometer_max: 1000.0     # 최대 압력값 (정규화 상한)
      barometer_sphere_min: 0.002  # 최소 sphere 반지름 (m)
      barometer_sphere_max: 0.008  # 최대 sphere 반지름 (m)
      tof_max_distance: 0.2     # ToF 최대 표시 거리 (m)
      tof_arrow_scale: 0.003    # 화살표 굵기 (m)
```

---

## Phase 3: Digital Twin Node 구현

### 3-1. `digital_twin_node.py` — 핵심 노드
**역할**: 500Hz 토픽 → 캐시 업데이트, 60Hz 타이머 → RViz용 데이터 발행

#### 구독 토픽 (Input)
| 토픽 | 메시지 타입 | 용도 |
|------|-------------|------|
| `/joint_states` | `sensor_msgs/JointState` | UR5e 6 joint positions/velocities |
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | Hand 10 positions + 10 velocities + 44 sensors = 64 doubles |

#### 발행 토픽 (Output, 60Hz)
| 토픽 | 메시지 타입 | 용도 |
|------|-------------|------|
| `/digital_twin/joint_states` | `sensor_msgs/JointState` | UR5e + Hand 통합 16 joint states → robot_state_publisher 입력 |
| `/digital_twin/fingertip_markers` | `visualization_msgs/MarkerArray` | Fingertip sensor 시각화 |

#### 핵심 로직

```python
class DigitalTwinNode(Node):
    def __init__(self):
        super().__init__('digital_twin_node')

        # Parameters
        self.display_rate = ...  # 60.0 Hz
        self.enable_hand = ...   # true

        # State cache (콜백에서 갱신, 타이머에서 읽기)
        self._robot_positions = [0.0] * 6
        self._robot_velocities = [0.0] * 6
        self._hand_positions = [0.0] * 10
        self._hand_velocities = [0.0] * 10
        self._fingertip_sensors = [[0.0]*11 for _ in range(4)]  # 4 tips × 11 sensors
        self._robot_data_received = False
        self._hand_data_received = False

        # Subscribers — 항상 최신 값만 캐시 (QoS: best_effort, depth=1)
        self.create_subscription(JointState, robot_topic, self._robot_cb, qos)
        if self.enable_hand:
            self.create_subscription(Float64MultiArray, hand_topic, self._hand_cb, qos)

        # Publisher
        self._joint_pub = self.create_publisher(JointState, '/digital_twin/joint_states', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/digital_twin/fingertip_markers', 10)

        # Sensor visualizer (별도 클래스)
        self._sensor_viz = SensorVisualizer(fingertip_names, sensor_viz_config)

        # 60Hz display timer
        self.create_timer(1.0 / self.display_rate, self._publish_display)

    def _robot_cb(self, msg: JointState):
        """500Hz → 캐시 업데이트만. 발행하지 않음."""
        # joint name → index 매핑으로 reorder
        for i, name in enumerate(msg.name):
            if name in self._robot_name_to_idx:
                idx = self._robot_name_to_idx[name]
                self._robot_positions[idx] = msg.position[i]
                if msg.velocity:
                    self._robot_velocities[idx] = msg.velocity[i]
        self._robot_data_received = True

    def _hand_cb(self, msg: Float64MultiArray):
        """Hand state 캐시 업데이트. Float64MultiArray: [pos×10, vel×10, sensor×44]"""
        data = msg.data
        if len(data) >= 64:
            self._hand_positions = list(data[0:10])
            self._hand_velocities = list(data[10:20])
            for i in range(4):
                offset = 20 + i * 11
                self._fingertip_sensors[i] = list(data[offset:offset+11])
            self._hand_data_received = True

    def _publish_display(self):
        """60Hz 타이머 콜백 — RViz용 데이터 발행"""
        if not self._robot_data_received:
            return

        now = self.get_clock().now().to_msg()

        # 1. Combined JointState 발행
        js = JointState()
        js.header.stamp = now
        js.header.frame_id = ''
        js.name = self._robot_joint_names[:]
        js.position = self._robot_positions[:]
        js.velocity = self._robot_velocities[:]

        if self.enable_hand and self._hand_data_received:
            js.name += self._hand_motor_names
            js.position += self._hand_positions
            js.velocity += self._hand_velocities

        self._joint_pub.publish(js)

        # 2. Fingertip sensor MarkerArray 발행
        if self.enable_hand and self._hand_data_received:
            markers = self._sensor_viz.create_markers(
                self._fingertip_sensors, now
            )
            self._marker_pub.publish(markers)
```

### 3-2. `sensor_visualizer.py` — 센서 시각화 모듈

```python
class SensorVisualizer:
    """Fingertip sensor data → visualization_msgs/MarkerArray 변환"""

    def create_markers(self, fingertip_sensors, stamp):
        """
        fingertip_sensors: list[4][11] — [baro×8, tof×3] per fingertip
        Returns: MarkerArray
        """
        markers = MarkerArray()

        for i, (name, sensors) in enumerate(zip(self.fingertip_names, fingertip_sensors)):
            baro = sensors[0:8]   # 8 barometer values
            tof = sensors[8:11]   # 3 ToF values

            # Barometer → Sphere markers (8개)
            for j, pressure in enumerate(baro):
                marker = self._make_baro_sphere(
                    name, i, j, pressure, stamp
                )
                markers.markers.append(marker)

            # ToF → Arrow markers (3개)
            for j, distance in enumerate(tof):
                marker = self._make_tof_arrow(
                    name, i, j, distance, stamp
                )
                markers.markers.append(marker)

        return markers

    def _make_baro_sphere(self, fingertip_name, finger_idx, baro_idx, value, stamp):
        """압력값 → 크기+색상 변환한 Sphere marker"""
        # frame_id: "{fingertip_name}_tip_link"
        # 크기: pressure에 비례 (min_r ~ max_r)
        # 색상: heatmap (blue → green → red)
        # 위치: 2×4 그리드 배치
        ...

    def _make_tof_arrow(self, fingertip_name, finger_idx, tof_idx, distance, stamp):
        """거리값 → Arrow marker"""
        # frame_id: "{fingertip_name}_tip_link"
        # 길이: distance에 비례
        # 색상: 거리에 따라 (빨강 → 초록)
        ...

    @staticmethod
    def _heatmap_color(ratio):
        """0.0~1.0 → RGBA heatmap (blue→green→red)"""
        ...
```

---

## Phase 4: robot_state_publisher 설정 (Launch에서 구성)

Combined URDF를 로드하고, `/digital_twin/joint_states`를 구독하여 TF tree 발행:

```python
# launch 파일 내에서
robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    namespace='digital_twin',
    parameters=[{
        'robot_description': combined_urdf_string,
        'publish_frequency': 60.0,
    }],
    remappings=[
        ('joint_states', '/digital_twin/joint_states'),
    ],
)
```

### TF Tree 구조
```
world
└── base_link
    └── base_link_inertia
        └── shoulder_link (shoulder_pan_joint)
            └── upper_arm_link (shoulder_lift_joint)
                └── forearm_link (elbow_joint)
                    └── wrist_1_link (wrist_1_joint)
                        └── wrist_2_link (wrist_2_joint)
                            └── wrist_3_link (wrist_3_joint)
                                └── flange → tool0
                                    └── hand_base_link (fixed)
                                        ├── thumb chain (3 joints + tip)
                                        ├── index chain (3 joints + tip)
                                        ├── middle chain (3 joints + tip)
                                        └── ring chain (1 joint + tip)
```

**주의**: 기존 UR driver도 robot_state_publisher를 실행할 수 있으므로, digital twin은 `/digital_twin` namespace로 TF를 분리하거나, 기존 robot_state_publisher가 없는 환경에서만 실행. Launch argument로 선택.

---

## Phase 5: RViz2 설정

**파일**: `ur5e_digital_twin/config/digital_twin.rviz`

Display 항목:
1. **RobotModel** — combined URDF 시각화 (`/digital_twin/robot_description` 파라미터)
2. **TF** — TF tree 표시 (선택적 on/off)
3. **MarkerArray** — `/digital_twin/fingertip_markers` (센서 시각화)
4. **Axes** — `tool0` frame TCP 좌표계
5. **Grid** — 바닥면 참조 (1m 간격)

설정:
- Fixed Frame: `world`
- Background: dark gray (48, 48, 48)
- RobotModel alpha: 0.9
- Target Frame: `base_link`

---

## Phase 6: Launch 파일

**파일**: `ur5e_digital_twin/launch/digital_twin.launch.py`

```python
def generate_launch_description():
    # Arguments
    use_rviz = LaunchArgument('use_rviz', default='true')
    display_rate = LaunchArgument('display_rate', default='60.0')
    enable_hand = LaunchArgument('enable_hand', default='true')

    # 1. xacro → URDF string (build time)
    urdf_content = Command(['xacro', urdf_xacro_path])

    # 2. robot_state_publisher
    #    - combined URDF 로드
    #    - /digital_twin/joint_states 구독 → TF 발행
    robot_state_publisher_node = Node(...)

    # 3. digital_twin_node
    #    - /joint_states, /hand/joint_states 구독
    #    - 60Hz로 /digital_twin/joint_states + /digital_twin/fingertip_markers 발행
    digital_twin_node = Node(
        package='ur5e_digital_twin',
        executable='digital_twin_node',
        parameters=[config_yaml, {'display_rate': display_rate, 'enable_hand': enable_hand}],
    )

    # 4. rviz2 (조건부 실행)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([...])
```

### 실행 방법

```bash
# 제어기와 별도로 실행 (제어기가 이미 구동 중일 때)
ros2 launch ur5e_digital_twin digital_twin.launch.py

# 핸드 없이 로봇만
ros2 launch ur5e_digital_twin digital_twin.launch.py enable_hand:=false

# Display rate 변경
ros2 launch ur5e_digital_twin digital_twin.launch.py display_rate:=30.0

# RViz 없이 TF만 발행 (다른 시각화 도구 사용 시)
ros2 launch ur5e_digital_twin digital_twin.launch.py use_rviz:=false
```

### 제어기와의 독립 실행 시나리오

```bash
# 터미널 1: 실제 로봇 제어기 실행
ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=192.168.1.100

# 터미널 2: Digital Twin 실행 (제어기와 완전 독립)
ros2 launch ur5e_digital_twin digital_twin.launch.py

# 터미널 2: 언제든 종료/재시작 가능 (제어기에 영향 없음)
# Ctrl+C → 재시작
ros2 launch ur5e_digital_twin digital_twin.launch.py display_rate:=30.0
```

---

## 파일 생성/수정 목록

### 새로 생성할 파일
| # | 파일 | 설명 |
|---|------|------|
| 1 | `ur5e_description/robots/ur5e/urdf/hand.urdf.xacro` | Hand URDF 모델 (10 joints, 4 fingertip frames) |
| 2 | `ur5e_description/robots/ur5e/urdf/ur5e_with_hand.urdf.xacro` | Combined URDF (UR5e + Hand) |
| 3 | `ur5e_digital_twin/package.xml` | 패키지 선언 (ament_python) |
| 4 | `ur5e_digital_twin/setup.py` | Python 패키지 설정 |
| 5 | `ur5e_digital_twin/setup.cfg` | install 경로 설정 |
| 6 | `ur5e_digital_twin/resource/ur5e_digital_twin` | ament index marker |
| 7 | `ur5e_digital_twin/ur5e_digital_twin/__init__.py` | 패키지 init |
| 8 | `ur5e_digital_twin/ur5e_digital_twin/digital_twin_node.py` | 핵심 노드 (60Hz decimation + publish) |
| 9 | `ur5e_digital_twin/ur5e_digital_twin/sensor_visualizer.py` | Fingertip sensor → MarkerArray |
| 10 | `ur5e_digital_twin/config/digital_twin.yaml` | 노드 파라미터 |
| 11 | `ur5e_digital_twin/config/digital_twin.rviz` | RViz2 설정 |
| 12 | `ur5e_digital_twin/launch/digital_twin.launch.py` | Launch 파일 |

### 수정할 파일
| # | 파일 | 변경 내용 |
|---|------|----------|
| - | 없음 | 기존 패키지 변경 없음 |

---

## 데이터 흐름 요약

```
[실제 로봇 UR5e]                    [Custom Hand]
   │ 500Hz                              │ ~167-500Hz
   ▼                                    ▼
/joint_states                    /hand/joint_states
(sensor_msgs/JointState)        (Float64MultiArray: 64 doubles)
   │                                    │
   └──────────┐          ┌──────────────┘
              ▼          ▼
   ┌────────────────────────────────┐
   │     ur5e_digital_twin 패키지   │  ← 독립 실행
   │  ┌──────────────────────────┐  │
   │  │   digital_twin_node     │  │
   │  │   (캐시 + 60Hz timer)   │  │
   │  └──────────┬──────────────┘  │
   │             │ 60Hz             │
   │   ┌─────────┼────────────┐    │
   │   ▼         ▼            │    │
   │ /digital_twin  /digital_twin  │
   │ /joint_states  /fingertip_    │
   │ (16 joints)    markers        │
   │   │         (MarkerArray)     │
   │   ▼              │            │
   │ robot_state_     │            │
   │ publisher         │            │
   │   │              │            │
   │   ▼              ▼            │
   │ [TF Tree]   [RViz MarkerArray]│
   │   │              │            │
   │   └──────┬───────┘            │
   │          ▼                    │
   │   ┌────────────┐             │
   │   │   RViz2    │ 60Hz render │
   │   └────────────┘             │
   └────────────────────────────────┘
```

---

## 구현 순서 (권장)

1. **Phase 1** — Hand URDF + Combined URDF (`ur5e_description`에 추가)
2. **Phase 2** — `ur5e_digital_twin` 패키지 뼈대 생성
3. **Phase 3** — Digital Twin Node 구현 (`digital_twin_node.py` + `sensor_visualizer.py`)
4. **Phase 4** — robot_state_publisher 설정 (launch 파일 내)
5. **Phase 5** — RViz2 설정 파일
6. **Phase 6** — Launch 파일 통합

예상 신규 코드: ~900 lines (Python ~650, URDF ~150, RViz config ~50, package scaffolding ~50)
