# Digital Twin RViz 구현 계획

## 개요
실제 로봇(UR5e + Custom Hand) 구동 시 URDF 기반 Digital Twin을 RViz2로 60Hz 시각화.
- **Robot**: `/joint_states` → 6 joint positions → URDF TF 업데이트
- **Hand**: `/hand/joint_states` → 10 motor positions + 44 fingertip sensor → URDF TF + Marker 시각화

---

## 현재 상태 분석

| 항목 | 상태 |
|------|------|
| UR5e URDF | ✅ `ur5e_description/robots/ur5e/urdf/ur5e.urdf` (6 revolute joints) |
| Hand URDF | ❌ 미존재 (MJCF만 존재, mesh 없음) |
| RViz 설정 | ❌ 미존재 |
| Digital Twin 노드 | ❌ 미존재 |
| 토픽 구조 | ✅ `/joint_states` (500Hz), `/hand/joint_states` (hand motor+sensor) |

---

## Phase 1: Hand URDF 생성 (ur5e_description 패키지)

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

- 10개 revolute joint (motor 1:1 매핑)
- 4개 fingertip frame (센서 시각화 기준점)
- 각 링크: cylinder (r=0.008, l=0.03~0.04) visual + collision
- joint limit: YAML config 참조하여 적절 범위 설정

### 1-2. Combined URDF 생성
**파일**: `ur5e_description/robots/ur5e/urdf/ur5e_with_hand.urdf.xacro`

```xml
<xacro:include filename="ur5e.urdf"/>
<xacro:include filename="hand.urdf.xacro"/>
<!-- hand_base_link를 tool0에 fixed joint로 연결 -->
<joint name="tool0_to_hand" type="fixed">
  <parent link="tool0"/>
  <child link="hand_base_link"/>
</joint>
```

---

## Phase 2: Digital Twin Node 구현 (Python, ur5e_tools 패키지)

### 2-1. 새 노드: `digital_twin_node.py`
**파일**: `ur5e_tools/ur5e_tools/digital_twin/digital_twin_node.py`

**역할**: 500Hz 토픽 데이터를 수신하여 60Hz로 decimate해서 RViz용 데이터 발행

#### 구독 토픽 (Input)
| 토픽 | 메시지 타입 | 용도 |
|------|-------------|------|
| `/joint_states` | `sensor_msgs/JointState` | UR5e 6 joint positions |
| `/hand/motor_states` | `ur5e_msgs/HandMotorState` | Hand 10 motor positions |
| `/hand/sensor_states` | `ur5e_msgs/HandSensorState` | Fingertip sensor data (4×11) |

> 참고: 현재 `/hand/joint_states`가 Float64MultiArray로 positions(10) + velocities(10) + sensors(44)를 묶어서 보내는 구조이므로, 이 토픽도 대안으로 구독 가능. 실제 토픽 구조에 맞춰 조정.

#### 발행 토픽 (Output, 60Hz)
| 토픽 | 메시지 타입 | 용도 |
|------|-------------|------|
| `/digital_twin/joint_states` | `sensor_msgs/JointState` | UR5e + Hand 통합 16 joint states (robot_state_publisher 입력) |
| `/digital_twin/fingertip_markers` | `visualization_msgs/MarkerArray` | Fingertip sensor 시각화 |
| `/digital_twin/tcp_pose` | `geometry_msgs/PoseStamped` | TCP 위치 시각화 (선택) |

#### 핵심 로직

```python
class DigitalTwinNode(Node):
    def __init__(self):
        # 60Hz display timer
        self.display_timer = self.create_timer(1.0/60.0, self.publish_display)

        # Latest state cache (lock-free update)
        self.robot_positions = [0.0] * 6   # UR5e joints
        self.hand_positions = [0.0] * 10   # Hand motors
        self.fingertip_sensors = {}        # {name: {barometer: [8], tof: [3]}}

    def joint_state_callback(self, msg):
        # 500Hz → cache update (no publish here)
        self.robot_positions = reorder(msg)

    def hand_state_callback(self, msg):
        # cache update
        self.hand_positions = msg.positions
        self.fingertip_sensors = parse_sensors(msg)

    def publish_display(self):
        # 60Hz timer callback
        # 1. Publish combined JointState (6 robot + 10 hand)
        # 2. Publish fingertip sensor MarkerArray
```

### 2-2. Fingertip Sensor 시각화 전략

각 fingertip별 (thumb, index, middle, ring):

#### Barometer (압력) → Sphere Marker
- 각 fingertip의 `*_tip_link` frame에 8개 sphere 배치
- **크기**: 압력값에 비례 (min 0.002m ~ max 0.008m)
- **색상**: 압력 강도에 따른 heatmap (파랑 → 초록 → 빨강)
  - 0% → blue (0,0,1), 50% → green (0,1,0), 100% → red (1,0,0)
- 배치: fingertip 표면에 2×4 그리드

#### ToF (거리) → Arrow Marker
- 각 fingertip의 `*_tip_link` frame에서 3개 화살표
- **길이**: ToF 측정 거리에 비례
- **색상**: 거리에 따라 (가까움=빨강, 멀리=초록)
- 방향: fingertip 법선 방향 (Z축)

---

## Phase 3: robot_state_publisher 설정

### 3-1. Combined URDF → TF Tree
`robot_state_publisher` 노드가 combined URDF를 로드하고, `/digital_twin/joint_states` 구독:

```python
robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'robot_description': combined_urdf_content,
        'publish_frequency': 60.0,  # TF 발행 주기
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
                                        ├── thumb chain (3 joints)
                                        ├── index chain (3 joints)
                                        ├── middle chain (3 joints)
                                        └── ring chain (1 joint)
```

---

## Phase 4: RViz2 설정

### 4-1. RViz config 파일
**파일**: `ur5e_description/config/digital_twin.rviz`

Display 항목:
1. **RobotModel** — combined URDF 시각화 (robot_description 파라미터)
2. **TF** — TF tree 표시 (선택적 on/off)
3. **MarkerArray** — `/digital_twin/fingertip_markers` (센서 시각화)
4. **Axes** — tool0 frame의 TCP 좌표계 표시
5. **Grid** — 바닥면 참조 그리드

설정:
- Fixed Frame: `world`
- Background: dark gray
- RobotModel alpha: 0.9

---

## Phase 5: Launch 파일

### 5-1. Digital Twin Launch
**파일**: `ur5e_tools/launch/digital_twin.launch.py`

```python
# 노드 구성:
# 1. robot_state_publisher (combined URDF, /digital_twin/joint_states 구독)
# 2. digital_twin_node (500Hz→60Hz decimation + sensor marker 발행)
# 3. rviz2 (digital_twin.rviz config 로드)
```

Arguments:
- `use_rviz` (default: true) — RViz 자동 실행 여부
- `display_rate` (default: 60.0) — Digital twin 갱신 주기
- `robot_description_file` — URDF 파일 경로 (기본: ur5e_with_hand)
- `enable_hand` (default: true) — 핸드 시각화 활성/비활성
- `sensor_viz_mode` (default: "heatmap") — 센서 시각화 모드

---

## Phase 6: 패키지 설정 업데이트

### 6-1. ur5e_tools/setup.py 업데이트
- `digital_twin_node` entry point 추가
- launch 파일 install 경로 추가

### 6-2. ur5e_tools/package.xml 업데이트
- `robot_state_publisher` 의존성 추가
- `rviz2` exec_depend 추가
- `visualization_msgs` 의존성 추가

### 6-3. ur5e_description/package.xml 업데이트
- xacro 의존성 확인

---

## 파일 생성/수정 목록

### 새로 생성할 파일
| # | 파일 | 설명 |
|---|------|------|
| 1 | `ur5e_description/robots/ur5e/urdf/hand.urdf.xacro` | Hand URDF 모델 |
| 2 | `ur5e_description/robots/ur5e/urdf/ur5e_with_hand.urdf.xacro` | Combined URDF |
| 3 | `ur5e_tools/ur5e_tools/digital_twin/__init__.py` | 패키지 init |
| 4 | `ur5e_tools/ur5e_tools/digital_twin/digital_twin_node.py` | 핵심 노드 |
| 5 | `ur5e_tools/ur5e_tools/digital_twin/sensor_visualizer.py` | 센서 Marker 생성 |
| 6 | `ur5e_description/config/digital_twin.rviz` | RViz2 설정 |
| 7 | `ur5e_tools/launch/digital_twin.launch.py` | Launch 파일 |

### 수정할 파일
| # | 파일 | 변경 내용 |
|---|------|----------|
| 1 | `ur5e_tools/setup.py` | entry_points + data_files |
| 2 | `ur5e_tools/package.xml` | 의존성 추가 |

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
        ┌─────────────────────┐
        │  digital_twin_node  │  ← 캐시 업데이트 (수신 즉시)
        │  (60Hz timer)       │
        └──────┬──────────────┘
               │ 60Hz
     ┌─────────┼──────────────┐
     ▼         ▼              ▼
/digital_twin  /digital_twin  /digital_twin
/joint_states  /fingertip_    /tcp_pose
(16 joints)    markers        (PoseStamped)
     │         (MarkerArray)       │
     ▼              │              ▼
robot_state_        │         [RViz Axes]
publisher           │
     │              │
     ▼              ▼
  [TF Tree]    [RViz MarkerArray]
     │              │
     └──────┬───────┘
            ▼
     ┌────────────┐
     │   RViz2    │  60Hz rendering
     └────────────┘
```

---

## 실행 방법 (완성 후)

```bash
# Digital Twin만 실행 (실제 로봇 + hand가 이미 구동 중일 때)
ros2 launch ur5e_tools digital_twin.launch.py

# 핸드 없이 로봇만
ros2 launch ur5e_tools digital_twin.launch.py enable_hand:=false

# Display rate 변경
ros2 launch ur5e_tools digital_twin.launch.py display_rate:=30.0
```

---

## 구현 순서 (권장)

1. **Phase 1** — Hand URDF 생성 + Combined URDF (시각화의 기반)
2. **Phase 2** — Digital Twin Node 구현 (데이터 파이프라인)
3. **Phase 3** — robot_state_publisher 설정 (TF 발행)
4. **Phase 4** — RViz2 설정 파일 (시각화 화면)
5. **Phase 5** — Launch 파일 (통합 실행)
6. **Phase 6** — 패키지 설정 업데이트 (빌드/인스톨)

예상 신규 코드: ~800 lines (Python ~600, URDF ~150, RViz config ~50)
