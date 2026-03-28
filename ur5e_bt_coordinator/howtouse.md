# ur5e_bt_coordinator 사용 가이드

![version](https://img.shields.io/badge/version-v5.17.0-blue)

BehaviorTree 기반 UR5e + Hand 비실시간 태스크 코디네이터.
500Hz RT 제어 루프 외부에서 20Hz로 동작하며, ROS2 토픽을 통해 RT Controller와 통신한다.

---

## 1. 사전 요구사항

### 시스템 의존성

| 패키지 | 설명 |
|--------|------|
| `rclcpp` | ROS2 C++ 클라이언트 |
| `behaviortree_cpp` | BehaviorTree.CPP v4 (`ros-jazzy-behaviortree-cpp`) |
| `std_msgs`, `geometry_msgs` | ROS2 표준 메시지 |
| `rtc_msgs` | 커스텀 메시지 (GuiPosition, GraspState) |
| `tf2` | 쿼터니언 → RPY 변환 |

### 사전 실행 필요 노드

BT coordinator를 실행하기 전에 다음 노드들이 먼저 실행되어 있어야 한다:

1. **UR5e RT Controller** — `/ur5e/gui_position`, `/hand/gui_position`, `/hand/grasp_state`, `/ur5e/active_controller_name` 토픽 발행
2. **Vision Node** (선택) — `/vision/object_pose` 토픽 발행 (물체 감지 사용 시)
3. **E-STOP Node** (선택) — `/system/estop_status` 토픽 발행

---

## 2. 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select ur5e_bt_coordinator
source install/setup.bash
```

---

## 3. 실행 방법

### 기본 실행 (Pick and Place)

```bash
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/bt_coordinator.yaml
```

### 트리 파일 지정 실행

```bash
# Pick and Place
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=pick_and_place.xml

# Towel Unfold
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=towel_unfold.xml

# Hand Motions Demo (UR5e 자세 유지 + 가감속 opposition/wave)
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=hand_motions.xml
```

### Blackboard 초기값과 함께 실행

```bash
# Pick and Place — place_pose 지정 필수
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args \
  -p tree_file:=pick_and_place.xml \
  -p place_pose:="0.3;-0.3;0.15;3.14;0.0;0.0"

# Towel Unfold — sweep 파라미터 지정 필수
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args \
  -p tree_file:=towel_unfold.xml \
  -p sweep_direction_x:=1.0 \
  -p sweep_direction_y:=0.0 \
  -p sweep_distance:=0.3
```

### 반복 실행

`repeat` 파라미터를 `true`로 설정하면 트리가 SUCCESS로 완료될 때마다 자동으로 리셋되어 반복 실행된다.
FAILURE로 완료되면 안전을 위해 반복하지 않고 정지한다.

```bash
# Pick and Place 반복 (2초 간격)
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args \
  -p tree_file:=pick_and_place.xml \
  -p repeat:=true \
  -p repeat_delay_s:=2.0 \
  -p place_pose:="0.3;-0.3;0.15;3.14;0.0;0.0"
```

반복 시 동작:
1. 트리 SUCCESS 완료 → `repeat_delay_s`만큼 대기
2. `haltTree()`로 모든 노드 상태 리셋
3. 비전 관련 blackboard 변수(`object_pose`) 초기화하여 재감지 유도
4. 트리를 처음부터 다시 실행

### 런타임 파라미터 변경

트리 시작 전에 Blackboard 값을 외부에서 설정할 수 있다:

```bash
ros2 param set /bt_coordinator place_pose "0.3;-0.3;0.15;3.14;0.0;0.0"
```

---

## 4. 설정 파일

`config/bt_coordinator.yaml`:

```yaml
bt_coordinator:
  ros__parameters:
    tree_file: "pick_and_place.xml"   # trees/ 디렉토리 내 XML 파일명
    tick_rate_hz: 20.0                # BT tick 주기 [Hz]
    repeat: false                     # true면 트리 완료 후 자동 반복
    repeat_delay_s: 1.0              # 반복 시 대기 시간 [s]

    # Blackboard 초기값 (트리에서 필요한 변수를 여기서 설정)
    # place_pose: "0.3;-0.3;0.15;3.14;0.0;0.0"
    # sweep_direction_x: 1.0
    # sweep_direction_y: 0.0
    # sweep_distance: 0.3
```

---

## 5. ROS2 토픽 인터페이스

### 구독 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/ur5e/gui_position` | `rtc_msgs/GuiPosition` | 팔 TCP 포즈 + 관절 위치 |
| `/hand/gui_position` | `rtc_msgs/GuiPosition` | 손 관절 위치 (10 DOF) |
| `/hand/grasp_state` | `rtc_msgs/GraspState` | 500Hz 사전 계산된 grasp 상태 (per-fingertip force + aggregate) |
| `/vision/object_pose` | `geometry_msgs/PoseStamped` | 비전으로 감지된 물체 포즈 |
| `/ur5e/active_controller_name` | `std_msgs/String` | 현재 활성 컨트롤러 이름 |
| `/system/estop_status` | `std_msgs/Bool` | E-STOP 상태 (true면 트리 일시정지) |

### 발행 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/ur5e/joint_goal` | `std_msgs/Float64MultiArray` | 팔 태스크 공간 목표 [x,y,z,roll,pitch,yaw] |
| `/hand/joint_goal` | `std_msgs/Float64MultiArray` | 손 모터 목표 [m0..m9] |
| `/ur5e/gains` | `std_msgs/Float64MultiArray` | 게인 업데이트 (16개 요소) |
| `/ur5e/select_controller` | `std_msgs/String` | 컨트롤러 전환 명령 |

---

## 6. BT 노드 레퍼런스

### 데이터 포맷

BT Blackboard에서 사용하는 문자열 포맷:

| 타입 | 포맷 | 예시 |
|------|------|------|
| `Pose6D` | `"x;y;z;roll;pitch;yaw"` | `"0.3;-0.3;0.15;3.14;0.0;0.0"` |
| `vector<double>` | `"v0;v1;v2;..."` | `"0.0;0.0;0.0;0.0;0.0;0.0"` |
| `vector<Pose6D>` | 파이프(`\|`)로 구분된 Pose6D | `"0.1;0.2;0.3;0;0;0\|0.4;0.5;0.6;0;0;0"` |

### Action 노드

#### MoveToPose

태스크 공간에서 목표 포즈로 팔을 이동한다.

```xml
<MoveToPose target="{pose}"
            position_tolerance="0.005"
            orientation_tolerance="0.05"
            timeout_s="10.0"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `target` | Pose6D | (필수) | 목표 6D 포즈 |
| `position_tolerance` | double | 0.005 m | 위치 수렴 허용오차 |
| `orientation_tolerance` | double | 0.05 rad | 자세 수렴 허용오차 |
| `timeout_s` | double | 10.0 s | 타임아웃 |

#### MoveToJoints

관절 공간에서 목표 관절값으로 팔을 이동한다.

```xml
<MoveToJoints target="0.0;-1.57;1.57;-1.57;-1.57;0.0"
              tolerance="0.01"
              timeout_s="10.0"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `target` | vector\<double\> | (필수) | 6개 관절 목표 [rad] |
| `tolerance` | double | 0.01 rad | 관절별 허용오차 |
| `timeout_s` | double | 10.0 s | 타임아웃 |

#### GraspControl

손(Hand)의 파지 동작을 제어한다. 4가지 모드를 지원한다.

```xml
<!-- 열기 -->
<GraspControl mode="open"
              target_positions="0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0"
              timeout_s="3.0"/>

<!-- 닫기 (점진적으로 힘 기반 파지와 함께 사용) -->
<GraspControl mode="close"
              close_speed="0.3"
              max_position="1.4"
              timeout_s="10.0"/>

<!-- 핀치 파지 (특정 모터만 닫기) -->
<GraspControl mode="pinch"
              close_speed="0.2"
              max_position="0.8"
              pinch_motors="0,1,2,3"
              timeout_s="8.0"/>

<!-- 프리셋 (지정 위치로 이동) -->
<GraspControl mode="preset"
              target_positions="0.3;0.0;0.0;0.3;0.0;0.0;0.0;0.0;0.0;0.0"
              timeout_s="3.0"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `mode` | string | (필수) | `"open"` / `"close"` / `"pinch"` / `"preset"` |
| `target_positions` | vector\<double\> | - | 10개 모터 목표 위치 [rad] |
| `close_speed` | double | 0.3 rad/s | close/pinch 모드에서 증가 속도 |
| `max_position` | double | 1.4 rad | close/pinch 모드 최대 위치 |
| `pinch_motors` | string | "0,1,2,3" | pinch 모드에서 사용할 모터 인덱스 (쉼표 구분) |
| `timeout_s` | double | 8.0 s | 타임아웃 |

#### TrackTrajectory

경유점(waypoint) 시퀀스를 순서대로 추적한다.

```xml
<TrackTrajectory waypoints="{sweep_waypoints}"
                 position_tolerance="0.01"
                 timeout_s="20.0"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `waypoints` | vector\<Pose6D\> | (필수) | 경유점 목록 |
| `position_tolerance` | double | 0.01 m | 위치 수렴 허용오차 |
| `timeout_s` | double | 30.0 s | 전체 타임아웃 |

#### SetGains

RT Controller의 게인을 동적으로 변경한다. 설정하지 않은 필드는 현재 값을 유지한다.

```xml
<SetGains trajectory_speed="0.05"
          max_traj_velocity="0.1"/>

<SetGains kp_translation="5.0,5.0,5.0"
          kp_rotation="3.0,3.0,3.0"
          trajectory_speed="0.08"
          max_traj_velocity="0.2"/>
```

| 포트 | 타입 | 설명 |
|------|------|------|
| `kp_translation` | string | 위치 비례 게인 `"kx,ky,kz"` (3개 값, 쉼표 구분) |
| `kp_rotation` | string | 자세 비례 게인 `"kx,ky,kz"` |
| `damping` | double | 감쇠 계수 |
| `null_kp` | double | Null-space 비례 게인 |
| `enable_null_space` | int | Null-space 활성화 (0/1) |
| `control_6dof` | int | 6DOF 제어 활성화 (0/1) |
| `trajectory_speed` | double | 궤적 생성 속도 [m/s] |
| `trajectory_angular_speed` | double | 궤적 각속도 |
| `hand_trajectory_speed` | double | 손 궤적 속도 |
| `max_traj_velocity` | double | 최대 궤적 속도 [m/s] |
| `max_traj_angular_velocity` | double | 최대 궤적 각속도 |
| `hand_max_traj_velocity` | double | 손 최대 궤적 속도 |
| `full_gains` | vector\<double\> | 16개 요소 전체 게인 직접 지정 |

**게인 배열 레이아웃 (16개 요소):**
`[kp_trans×3, kp_rot×3, damping, null_kp, enable_null, control_6dof, traj_speed, traj_angular_speed, hand_traj_speed, max_vel, max_angular_vel, hand_max_vel]`

#### SwitchController

활성 컨트롤러를 전환한다.

```xml
<SwitchController controller_name="impedance_controller"
                  timeout_s="3.0"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `controller_name` | string | (필수) | 전환할 컨트롤러 이름 |
| `timeout_s` | double | 3.0 s | 확인 타임아웃 |

#### ComputeOffsetPose

입력 포즈에 XYZ 오프셋을 적용한 새 포즈를 계산한다 (자세는 유지).

```xml
<ComputeOffsetPose input_pose="{object_pose}"
                   offset_z="0.04"
                   output_pose="{approach_pose}"/>
```

| 포트 | 방향 | 타입 | 기본값 | 설명 |
|------|------|------|--------|------|
| `input_pose` | input | Pose6D | (필수) | 기준 포즈 |
| `offset_x` | input | double | 0.0 | X 오프셋 [m] |
| `offset_y` | input | double | 0.0 | Y 오프셋 [m] |
| `offset_z` | input | double | 0.0 | Z 오프셋 [m] |
| `output_pose` | output | Pose6D | - | 계산된 포즈 |

#### ComputeSweepTrajectory

시작 포즈에서 지정된 방향으로 호(arc) 형태 경로를 생성한다.

```xml
<ComputeSweepTrajectory start_pose="{lift_pose}"
                        direction_x="{sweep_direction_x}"
                        direction_y="{sweep_direction_y}"
                        distance="{sweep_distance}"
                        arc_height="0.05"
                        num_waypoints="8"
                        waypoints="{sweep_waypoints}"/>
```

| 포트 | 방향 | 타입 | 기본값 | 설명 |
|------|------|------|--------|------|
| `start_pose` | input | Pose6D | (필수) | 시작 포즈 |
| `direction_x` | input | double | (필수) | 스윕 방향 X 성분 |
| `direction_y` | input | double | (필수) | 스윕 방향 Y 성분 |
| `distance` | input | double | (필수) | 총 스윕 거리 [m] |
| `arc_height` | input | double | (필수) | 호 최대 높이 [m] |
| `num_waypoints` | input | int | 8 | 생성할 경유점 수 |
| `waypoints` | output | vector\<Pose6D\> | - | 생성된 경유점 |

#### WaitDuration

지정된 시간 동안 대기한다.

```xml
<WaitDuration duration_s="0.5"/>
```

#### MoveFinger

특정 손가락을 명명된 포즈로 이동한다. 하위 컨트롤러가 quintic trajectory를 수행하므로, 목표 관절각과 duration만 전달하고 시간 기반으로 완료를 판정한다.

```xml
<MoveFinger finger_name="thumb"
            pose="thumb_index_oppose"
            duration="{op_duration}"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `finger_name` | string | (필수) | 손가락 이름 (`"thumb"` / `"index"` / `"middle"` / `"ring"`) |
| `pose` | string | (필수) | 명명된 타겟 포즈 (`hand_pose_config.hpp`에서 lookup) |
| `duration` | double | 1.0 s | Trajectory 실행 시간 |

#### FlexExtendFinger

특정 손가락의 flex → extend 1회 cycle을 수행한다. duration/2 동안 flex 포즈로 이동, 이후 duration/2 동안 home(extend)으로 복귀한다.

```xml
<FlexExtendFinger finger_name="index"
                  duration="{wave_duration}"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `finger_name` | string | (필수) | 손가락 이름 |
| `duration` | double | 1.0 s | Flex+Extend 전체 시간 (min: 0.3s) |

#### SetHandPose

Hand 전체 10-DoF를 명명된 포즈로 이동한다.

```xml
<SetHandPose pose="home" duration="{op_duration}"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `pose` | string | (필수) | 명명된 Hand 포즈 (예: `"home"`, `"full_flex"`) |
| `duration` | double | 1.0 s | Trajectory 실행 시간 |

#### UR5eHoldPose

UR5e 팔을 목표 자세로 이동 후, Parallel 부모에 의해 halt될 때까지 영구 RUNNING을 반환한다. SUCCESS를 반환하지 않으므로 반드시 `Parallel`과 함께 사용해야 한다.

```xml
<Parallel success_count="1" failure_count="1">
  <UR5eHoldPose pose="demo_pose"/>
  <Sequence>
    <!-- Hand 동작 시퀀스 -->
  </Sequence>
</Parallel>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `pose` | string | (필수) | 명명된 UR5e 포즈 (예: `"demo_pose"`) |

### Condition 노드

#### IsForceAbove

핑거팁 힘이 임계값을 초과하는지 검사한다. 컨트롤러에서 500Hz로 사전 계산된 `GraspState`를 활용하며, threshold/min_fingertips가 컨트롤러 기본값(1.0N, 2개)과 일치하면 aggregate 결과를 직접 사용한다.

```xml
<IsForceAbove threshold_N="1.5"
              min_fingertips="2"
              sustained_ms="200"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `threshold_N` | double | 1.5 N | 힘 임계값 |
| `min_fingertips` | int | 2 | 최소 감지 핑거팁 수 |
| `sustained_ms` | int | 0 ms | 유지 시간 (히스테리시스) |

#### IsGrasped

힘 + 접촉 상태를 검사하여 파지 성공 여부를 판단한다. 컨트롤러에서 500Hz로 사전 계산된 `GraspState`를 활용한다.

```xml
<IsGrasped force_threshold_N="1.0"
           min_fingertips="2"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `force_threshold_N` | double | 1.0 N | 힘 임계값 |
| `min_fingertips` | int | 2 | 최소 감지 핑거팁 수 |

#### IsObjectDetected

비전 노드에서 물체가 감지되었는지 확인하고, 포즈를 Blackboard에 출력한다.

```xml
<IsObjectDetected pose="{object_pose}"/>
```

| 포트 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `pose` | output | Pose6D | 감지된 물체 포즈 |

---

## 7. 기본 제공 BT 트리

### Pick and Place (`pick_and_place.xml`)

물체를 감지하여 집어 올린 후 목표 위치에 놓는 전체 시퀀스.

**필수 Blackboard 변수:**
- `place_pose` — 목표 배치 포즈 (예: `"0.3;-0.3;0.15;3.14;0.0;0.0"`)

**동작 순서:**

| 단계 | 동작 | 주요 파라미터 |
|------|------|--------------|
| 1 | 비전으로 물체 감지 | 최대 10회 재시도 |
| 2 | 손 열기 + 물체 상방 4cm 접근 | position_tolerance: 3mm |
| 3 | 저속 하강 | trajectory_speed: 0.05 m/s |
| 4 | 힘 기반 파지 (Parallel) | 1.5N, 2+ 핑거팁, 200ms 유지 |
| 5 | 파지 검증 + 12cm 들어올리기 | 들어올린 후 재검증 |
| 6 | 목표 위치로 이송 | trajectory_speed: 0.1 m/s |
| 7 | 5cm 하강 + 손 열기 | 저속 하강 |
| 8 | 8cm 상방 후퇴 | - |

### Towel Unfold (`towel_unfold.xml`)

수건 가장자리를 핀치 파지하여 들어올린 후 호 형태로 스윕하여 펼치는 시퀀스.

**필수 Blackboard 변수:**
- `sweep_direction_x` — 스윕 방향 X 성분 (예: `1.0`)
- `sweep_direction_y` — 스윕 방향 Y 성분 (예: `0.0`)
- `sweep_distance` — 스윕 거리 [m] (예: `0.3`)

**동작 순서:**

| 단계 | 동작 | 주요 파라미터 |
|------|------|--------------|
| 1 | 비전으로 수건 가장자리 감지 | 최대 10회 재시도 |
| 2 | 핀치 프리셋 + 가장자리 상방 5cm 접근 | 엄지+검지 open |
| 3 | 저속 하강 + 핀치 파지 | 0.5N, 1+ 핑거팁, 모터 0,1,2,3 |
| 4 | 25cm 들어올리기 | 파지 검증 포함 |
| 5 | 컴플라이언트 게인으로 전환 + 호 스윕 | kp_trans: 5.0, arc_height: 5cm |
| 6 | 20cm 하강 + 손 열기 | 강성 게인 복원 |
| 7 | 10cm 상방 후퇴 | - |

### Hand Motions Demo (`hand_motions.xml`)

UR5e가 고정 자세를 유지하는 동안, Hand가 가감속 opposition과 flex/extend wave 데모를 순차 수행한다.

**필수 Blackboard 변수:** 없음 (모두 트리 내부에서 초기화)

**내부 Blackboard 변수:**

| 변수 | 타입 | 용도 |
|------|------|------|
| `op_duration` | double | Opposition 동작의 tempo (초기값 1.5s) |
| `use_home_return` | bool | Opposition 후 home 복귀 여부 |
| `wave_duration` | double | FlexExtend wave의 tempo (초기값 2.0s) |

**BT 구조:**

```
FullDemo (Parallel, success_count=1)
├── UR5eHoldPose(demo_pose)           ← 영구 RUNNING
└── HandDemoSequence (Sequence)
    ├── OppositionDemo (SubTree)
    │   ├── Phase 1: 1.5s, home 경유 (1회 순회)
    │   ├── Phase 2: 가속 ×0.7, home 생략 (3회 순회)
    │   └── Phase 3: 감속 ×1.4, home 경유 (3회 순회)
    └── WaveDemo (SubTree)
        ├── Phase 1: 순차, 가속 ×0.7 (3회)
        ├── Phase 2: 동시(Parallel), 최고 속도 (3회)
        └── Phase 3: 순차, 감속 ×1.4 (3회)
```

**Tempo 변화:**

| Phase | Opposition duration | Wave duration |
|-------|-------------------|---------------|
| Phase 1 | 1.50s (home 경유) | 2.00 → 1.40 → 0.98 |
| Phase 2 | 1.50 → 1.05 → 0.74 (home 생략) | 0.69 → 0.69 → 0.69 (동시) |
| Phase 3 | 0.74 → 1.03 → 1.44 (home 경유) | 0.69 → 0.96 → 1.35 |

**Hand 관절 매핑 (10-DoF):**

| 손가락 | 관절 | DoF | 인덱스 |
|--------|------|-----|--------|
| Thumb | CMC abd/add, CMC flex/ext, MCP flex/ext | 3 | 0–2 |
| Index | MCP abd/add, MCP flex/ext, DIP flex/ext | 3 | 3–5 |
| Middle | MCP abd/add, MCP flex/ext, DIP flex/ext | 3 | 6–8 |
| Ring | MCP flex/ext | 1 | 9 |

포즈 값은 `hand_pose_config.hpp`에 placeholder로 정의되어 있으며, 하드웨어 캘리브레이션 후 교체해야 한다.

---

## 8. 커스텀 BT 트리 작성

### 기본 구조

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MyTask">
    <Sequence name="main">
      <!-- 노드 배치 -->
    </Sequence>
  </BehaviorTree>
</root>
```

### 파일 위치

`trees/` 디렉토리에 XML 파일을 추가하면 자동으로 install에 포함된다.

### 힘 기반 파지 패턴

가장 많이 사용되는 패턴으로, 손을 닫으면서 동시에 힘을 모니터링한다:

```xml
<Parallel success_count="1" failure_count="1">
  <GraspControl mode="close"
                close_speed="0.3"
                max_position="1.4"
                timeout_s="10.0"/>
  <RetryUntilSuccessful num_attempts="200">
    <Sequence>
      <WaitDuration duration_s="0.05"/>
      <IsForceAbove threshold_N="1.5"
                    min_fingertips="2"
                    sustained_ms="200"/>
    </Sequence>
  </RetryUntilSuccessful>
</Parallel>
```

`Parallel`에서 `success_count="1"`이므로, 힘 조건이 충족되면 즉시 GraspControl이 중단되고 성공을 반환한다.

### 접근-하강 패턴

물체 위에서 접근 후 저속으로 하강하는 일반적인 패턴:

```xml
<!-- 상방 접근 -->
<ComputeOffsetPose input_pose="{target}"
                   offset_z="0.04"
                   output_pose="{approach}"/>
<MoveToPose target="{approach}"
            position_tolerance="0.003"
            timeout_s="10.0"/>

<!-- 저속 하강 -->
<SetGains trajectory_speed="0.05"
          max_traj_velocity="0.1"/>
<MoveToPose target="{target}"
            position_tolerance="0.002"
            timeout_s="8.0"/>
```

---

## 9. 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| 트리가 즉시 FAILURE | 비전 토픽 미발행 | 비전 노드 실행 확인 |
| 팔이 움직이지 않음 | RT Controller 미실행 또는 E-STOP 활성 | 컨트롤러 상태 및 E-STOP 확인 |
| 파지 타임아웃 | 힘 임계값이 너무 높거나 센서 미연결 | `threshold_N` 조정, `/hand/grasp_state` 확인 |
| 게인 변경이 반영 안 됨 | `/ur5e/gains` 토픽 QoS 불일치 | RELIABLE QoS 확인 |
| "Tree completed with FAILURE" 로그 | 시퀀스 중 하나의 노드가 실패 | Groot2로 트리 실행 추적하여 실패 노드 확인 |

### 디버깅 유용 명령

```bash
# 토픽 모니터링
ros2 topic echo /ur5e/gui_position
ros2 topic echo /hand/grasp_state
ros2 topic echo /vision/object_pose

# 파라미터 확인
ros2 param list /bt_coordinator
ros2 param get /bt_coordinator tree_file

# 활성 컨트롤러 확인
ros2 topic echo /ur5e/active_controller_name
```
