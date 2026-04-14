# ur5e_bt_coordinator 사용 가이드

BehaviorTree 기반 UR5e + Hand 비실시간 태스크 코디네이터.
500Hz RT 제어 루프 외부에서 80Hz로 동작하며, ROS2 토픽을 통해 RT Controller와 통신한다.

---

## 1. 사전 요구사항

### 시스템 의존성

| 패키지 | 설명 |
|--------|------|
| `rclcpp` | ROS2 C++ 클라이언트 |
| `behaviortree_cpp` | BehaviorTree.CPP v4 (`ros-jazzy-behaviortree-cpp`) |
| `std_msgs`, `std_srvs`, `geometry_msgs` | ROS2 표준 메시지 및 서비스 |
| `rtc_msgs` | 커스텀 메시지 (GuiPosition, GraspState, RobotTarget, ToFSnapshot) |
| `shape_estimation_msgs` | ShapeEstimate 메시지 (shape_inspect 트리용) |
| `Eigen3` | 쿼터니언 ↔ RPY 변환 (ComputeOffsetPose quat 모드) |
| `ament_index_cpp` | 패키지 share 디렉토리 탐색 |

### 사전 실행 필요 노드

BT coordinator를 실행하기 전에 다음 노드들이 먼저 실행되어 있어야 한다:

1. **UR5e RT Controller** — `/ur5e/gui_position`, `/hand/gui_position`, `/hand/grasp_state`, `/ur5e/active_controller_name`, `/tof/snapshot` 토픽 발행
2. **Vision Node** (선택) — `/world_target_info` 토픽 발행 (물체 감지 사용 시)
3. **Shape Estimation Node** (선택) — `/shape/estimate` 토픽 발행, `/shape/trigger` 수신, `/shape/clear` 서비스 (shape_inspect 트리용)
4. **E-STOP Node** (선택) — `/system/estop_status` 토픽 발행

---

## 2. 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select ur5e_bt_coordinator
source install/setup.bash
```

---

## 3. 실행 방법

### Launch 파일 (권장)

```bash
# 기본 실행 (YAML 기본 트리, YAML + 포즈 자동 로드)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py

# ── Pick and Place (pose-based grasp, grasp controller 미사용) ──
# 기본 (medium grip)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place.xml

# soft grip (부드러운 물체용)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place.xml grip:=soft

# hard grip (단단한 물체용)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place.xml grip:=hard

# ── Pick and Place (force-based grasp, grasp controller 필요) ──
# Contact-stop grasp (단일 시도)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place_contact_stop.xml

# Force-PI adaptive grasp (retry 지원)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place_force_pi.xml

# ── Shape Inspection ──
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=shape_inspect.xml               # ToF shape estimation
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=shape_inspect_simple.xml         # search move + ToF data collection

# ── 기타 트리 ──
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=towel_unfold.xml
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=hand_motions.xml repeat:=true
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=vision_approach.xml
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=search_motion.xml

# ── 옵션 ──
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place.xml groot2_port:=1667
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place.xml paused:=true
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place.xml repeat:=true repeat_delay:=2.0
```

**Launch arguments:**

| Argument | 기본값 | 설명 |
|----------|--------|------|
| `tree` | (YAML 기본값) | BT tree XML 파일명 |
| `tick_rate` | 0 (=YAML 80Hz) | BT tick 주기 [Hz] |
| `repeat` | (YAML 기본값) | SUCCESS 시 자동 반복 |
| `repeat_delay` | 0 (=YAML 1.0s) | 반복 전 대기 시간 [s] |
| `paused` | (YAML 기본값) | 일시정지 상태로 시작 |
| `groot2_port` | 0 (비활성) | Groot2 ZMQ 포트 |
| `grip` | (YAML 기본값) | Pose-based grasp grip 강도: `soft`, `medium`, `hard` |

### ros2 run (직접 실행)

```bash
# 기본 실행 (YAML config + poses 직접 지정)
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/bt_coordinator.yaml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses.yaml

# Pick and Place (pose-based, soft grip)
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=pick_and_place.xml \
  -p bb.hand_close_pose:=hand_close_soft \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/bt_coordinator.yaml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses.yaml

# Pick and Place (force-based contact_stop)
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=pick_and_place_contact_stop.xml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/bt_coordinator.yaml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses.yaml

# Towel Unfold (Blackboard 변수 지정)
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=towel_unfold.xml \
  -p bb.sweep_direction_x:=1.0 \
  -p bb.sweep_direction_y:=0.0 \
  -p bb.sweep_distance:=0.3 \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/bt_coordinator.yaml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses.yaml
```

### 반복 실행

`repeat` 파라미터를 `true`로 설정하면 트리가 SUCCESS로 완료될 때마다 자동으로 리셋되어 반복 실행된다.
FAILURE로 완료되면 안전을 위해 반복하지 않고 정지한다.

```bash
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py \
  tree:=pick_and_place.xml grip:=medium repeat:=true repeat_delay:=2.0
```

반복 시 동작:
1. 트리 SUCCESS 완료 → `repeat_delay_s`만큼 대기
2. `haltTree()`로 모든 노드 상태 리셋
3. 비전 관련 blackboard 변수(`object_pose`) 초기화하여 재감지 유도
4. 트리를 처음부터 다시 실행

### 런타임 제어

```bash
# 일시 정지 / 재개
ros2 param set /bt_coordinator paused true
ros2 param set /bt_coordinator paused false

# 트리 핫스왑 (재시작 없이 다른 트리로 전환)
ros2 param set /bt_coordinator tree_file "towel_unfold.xml"

# 반복 모드 활성화
ros2 param set /bt_coordinator repeat true

# Step 모드: 한 틱씩 수동 진행
ros2 param set /bt_coordinator step_mode true
ros2 service call /bt_coordinator/step std_srvs/srv/Trigger

# Blackboard 변수 런타임 설정
ros2 param set /bt_coordinator bb.place_pose "0.3;-0.3;0.15;3.14;0.0;0.0"
```

### 오프라인 트리 검증

ROS 실행 없이 BT XML의 구문 및 포트 정합성을 검증할 수 있다:

```bash
ros2 run ur5e_bt_coordinator validate_tree pick_and_place.xml
# Exit code: 0=valid, 1=invalid, 2=file not found
```

---

## 4. 설정 파일

### `config/bt_coordinator.yaml`

```yaml
bt_coordinator:
  ros__parameters:
    # ── BT 시스템 설정 ──
    tree_file: "pick_and_place_force_pi.xml"
    tick_rate_hz: 80.0
    repeat: false
    repeat_delay_s: 1.0
    paused: false
    step_mode: false
    groot2_port: 0                    # 0=비활성, 1667=Groot2 기본 포트
    watchdog_timeout_s: 2.0
    watchdog_interval_s: 5.0

    # ── Blackboard: pick_and_place 공용 (contact_stop / force_pi / pose-based) ──
    bb.place_pose: "0.3;-0.3;0.15;3.14;0.0;0.0"
    bb.object_final_z: 0.06
    bb.grasp_target_force_N: 1.5
    bb.grasp_verify_force_N: 0.8
    bb.grasp_sustained_ms: 200
    bb.grasp_min_fingertips: 2
    bb.grasp_abort_retreat_z: 0.05
    bb.grasp_in_transit_min_N: 0.5
    bb.grasp_max_attempts: 2          # force_pi only
    bb.grasp_retry_settle_s: 0.3
    bb.tcp_rpy_offset_r_deg: 0.0      # _deg → rad 자동 변환
    bb.tcp_rpy_offset_p_deg: 0.0
    bb.tcp_rpy_offset_y_deg: 0.0
    bb.tcp_rotation_mode: "add"       # "add" / "quat_body" / "quat_world"

    # ── Blackboard: pick_and_place.xml (pose-based grasp) ──
    bb.hand_close_pose: "hand_close_medium"
    bb.hand_close_settle_s: 0.5

    # ── Blackboard: shape_inspect 공용 (shape_inspect / shape_inspect_simple) ──
    bb.inspect_offset_x: 0.0
    bb.inspect_offset_y: 0.0
    bb.inspect_approach_z: 0.15

    # ── Blackboard: shape_inspect_simple.xml ──
    bb.inspect_constant_z: 0.06       # 고정 Z (비전 Z 대신 사용)
    bb.search_offset_x: -0.1          # -x 방향 search 거리 [m]
    bb.search_speed: 0.02             # search trajectory speed [m/s]

    # ── Blackboard: towel_unfold.xml ──
    # bb.sweep_direction_x: 1.0
    # bb.sweep_direction_y: 0.0
    # bb.sweep_distance: 0.3
```

### `config/poses.yaml`

Hand/UR5e 포즈를 재컴파일 없이 튜닝할 수 있다. 값은 **도(deg) 단위**로 작성하고, 로드 시 자동으로 radian 변환된다.

```yaml
bt_coordinator:
  ros__parameters:
    hand_pose.home: [0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0]
    hand_pose.thumb_index_oppose: [15.0, 45.0, 35.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0]
    arm_pose.demo_pose: [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    arm_pose.ready: [0.0, -90.0, 0.0, -90.0, -90.0, 0.0]
    arm_pose.table_top: [0.0, -70.0, 110.0, -130.0, -90.0, 0.0]
    # ... (10개 추가 포즈: ready, table_top, front_reach, side_reach,
    #      handover, stow, look_up, look_down, pick_ready, elevated)
```

런타임에도 변경 가능:
```bash
ros2 param set /bt_coordinator hand_pose.home "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

---

## 5. ROS2 토픽 인터페이스

### 구독 토픽

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/ur5e/gui_position` | `rtc_msgs/GuiPosition` | RELIABLE/10 | 팔 TCP 포즈 + 관절 위치 |
| `/hand/gui_position` | `rtc_msgs/GuiPosition` | RELIABLE/10 | 손 관절 위치 (10 DOF) |
| `/hand/grasp_state` | `rtc_msgs/GraspState` | RELIABLE/10 | 500Hz 사전 계산된 grasp 상태 |
| `/world_target_info` | `geometry_msgs/Polygon` | RELIABLE/10 | 비전 물체 위치 (points[0] = x,y,z) |
| `/ur5e/active_controller_name` | `std_msgs/String` | RELIABLE/1 transient_local | 현재 활성 컨트롤러 이름 |
| `/system/estop_status` | `std_msgs/Bool` | RELIABLE/10 | E-STOP 상태 (true면 트리 일시정지) |
| `/ur5e/current_gains` | `std_msgs/Float64MultiArray` | RELIABLE/10 | SwitchController가 요청한 현재 게인 응답 |
| `/shape/estimate` | `shape_estimation_msgs/ShapeEstimate` | RELIABLE/10 | shape estimation 결과 |
| `/tof/snapshot` | `rtc_msgs/ToFSnapshot` | BEST_EFFORT/100 | 500Hz ToF 센서 데이터 (ToF 수집 모드 시 buffer) |

### 발행 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/ur5e/joint_goal` | `rtc_msgs/RobotTarget` | 팔 task-space 또는 joint-space 목표 |
| `/hand/joint_goal` | `rtc_msgs/RobotTarget` | 손 10-DoF 모터 목표 |
| `/ur5e/controller_gains` | `std_msgs/Float64MultiArray` | 게인 업데이트 (DemoTask 21개 / DemoJoint 9개 요소) |
| `/ur5e/controller_type` | `std_msgs/String` | 컨트롤러 전환 명령 |
| `/ur5e/request_gains` | `std_msgs/Bool` | 현재 게인 요청 (SwitchController에서 사용) |
| `/shape/trigger` | `std_msgs/String` | shape estimation 제어 (start/stop/pause/resume) |

### 서비스 클라이언트

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/shape/clear` | `std_srvs/Trigger` | shape estimation 누적 데이터 초기화 |

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
<SetGains trajectory_speed="0.05"/>

<SetGains kp_translation="5.0,5.0,5.0"
          kp_rotation="3.0,3.0,3.0"
          trajectory_speed="0.08"/>
```

> **Note:** `max_traj_velocity`, `max_traj_angular_velocity`, `hand_max_traj_velocity` 포트는 제거되었다. 이 값들은 SwitchController가 설정한 `current_gains`에서 자동으로 로드된다. BT XML에서 직접 설정할 수 없다.

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
| `full_gains` | vector\<double\> | 전체 게인 직접 지정 (DemoTask 21개 / DemoJoint 9개 요소) |
| `grasp_command` | int | Force-PI 명령 (0=none, 1=grasp, 2=release) |
| `grasp_target_force` | double | Force-PI 목표력 [N] (기본 2.0) |

**게인 배열 레이아웃 (DemoTaskController 21개 요소):**
`[kp_trans×3, kp_rot×3, damping, null_kp, enable_null, control_6dof, traj_speed, traj_angular_speed, hand_traj_speed, max_vel (locked), max_angular_vel (locked), hand_max_vel (locked), grasp_contact_thresh, grasp_force_thresh, grasp_min_fingertips, grasp_cmd, grasp_target_force]`

**게인 배열 레이아웃 (DemoJointController 9개 요소):**
`[robot_traj_speed, hand_traj_speed, robot_max_traj_vel (locked), hand_max_traj_vel (locked), grasp_contact_thresh, grasp_force_thresh, grasp_min_fingertips, grasp_cmd, grasp_target_force]`

> `(locked)` 필드는 `current_gains`에서 자동으로 로드되며 BT에서 변경할 수 없다.

#### SwitchController

활성 컨트롤러를 전환한다.

```xml
<SwitchController controller_name="demo_task_controller"
                  timeout_s="3.0"
                  current_gains="{current_gains}"/>
```

| 포트 | 방향 | 타입 | 기본값 | 설명 |
|------|------|------|--------|------|
| `controller_name` | input | string | (필수) | 전환할 컨트롤러 이름 |
| `timeout_s` | input | double | 3.0 s | 확인 타임아웃 |
| `load_gains` | input | bool | true | 전환 후 현재 게인 자동 로드 |
| `current_gains` | output | vector\<double\> | - | 로드된 게인 (load_gains=true 시) |

#### ComputeOffsetPose

입력 포즈에 6-DoF 오프셋을 적용한 새 포즈를 계산한다.
Translation은 항상 가산. Rotation은 3가지 모드 지원.

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
| `offset_roll` | input | double | 0.0 | Roll 오프셋 [rad] |
| `offset_pitch` | input | double | 0.0 | Pitch 오프셋 [rad] |
| `offset_yaw` | input | double | 0.0 | Yaw 오프셋 [rad] |
| `rotation_mode` | input | string | "add" | "add" / "quat_body" / "quat_world" |
| `output_pose` | output | Pose6D | - | 계산된 포즈 |

#### SetPoseZ

입력 포즈의 Z 좌표를 상수 값으로 덮어쓴다. NaN이면 pass-through.

```xml
<SetPoseZ input_pose="{pose_xy}" z="{inspect_constant_z}" output_pose="{pose}"/>
```

| 포트 | 방향 | 타입 | 기본값 | 설명 |
|------|------|------|--------|------|
| `input_pose` | input | Pose6D | (필수) | 기준 포즈 |
| `z` | input | double | NaN | 절대 Z [m] (NaN = 비활성) |
| `output_pose` | output | Pose6D | - | Z가 덮어쓰인 포즈 |

#### GetCurrentPose

현재 TCP 포즈를 Blackboard에 출력한다.

```xml
<GetCurrentPose pose="{current_pose}"/>
```

| 포트 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `pose` | output | Pose6D | 현재 TCP 포즈 |

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

특정 손가락을 명명된 포즈로 이동한다. RT 컨트롤러와 동일한 quintic trajectory duration 공식으로 소요 시간을 추정하여 완료를 판정한다.

```xml
<MoveFinger finger_name="thumb"
            pose="thumb_index_oppose"
            hand_trajectory_speed="{hand_speed}"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `finger_name` | string | (필수) | 손가락 이름 (`"thumb"` / `"thumb_mcp"` / `"index"` / `"index_dip"` / `"middle"` / `"middle_dip"` / `"ring"`) |
| `pose` | string | (필수) | 명명된 타겟 포즈 (`hand_pose_config.hpp`에서 lookup) |
| `hand_trajectory_speed` | double | 1.0 rad/s | RT 컨트롤러 trajectory speed (duration 추정에 사용) |
| `current_gains` | vector\<double\> | (자동) | Blackboard `{current_gains}`에서 자동 읽기 (max trajectory velocity 등 포함) |

#### FlexExtendFinger

특정 손가락의 flex → extend 1회 cycle을 수행한다. flex 포즈로 이동 후 trajectory 완료 시 home(extend)으로 복귀한다. 각 phase의 소요 시간은 RT 컨트롤러와 동일한 공식으로 추정한다.

```xml
<FlexExtendFinger finger_name="index"
                  hand_trajectory_speed="{hand_speed}"
                  hand_max_traj_velocity="{hand_max_vel}"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `finger_name` | string | (필수) | 손가락 이름 (`"thumb"` / `"thumb_mcp"` / `"index"` / `"index_dip"` / `"middle"` / `"middle_dip"` / `"ring"`) |
| `hand_trajectory_speed` | double | 1.0 rad/s | RT 컨트롤러 trajectory speed |
| `hand_max_traj_velocity` | double | 2.0 rad/s | RT 컨트롤러 max trajectory velocity |

#### SetHandPose

Hand 전체 10-DoF를 명명된 포즈로 이동한다. RT 컨트롤러와 동일한 quintic trajectory duration 공식으로 소요 시간을 추정하여 완료를 판정한다.

```xml
<SetHandPose pose="home"
             hand_trajectory_speed="{hand_speed}"
             hand_max_traj_velocity="{hand_max_vel}"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `pose` | string | (필수) | 명명된 Hand 포즈 (예: `"home"`, `"full_flex"`) |
| `hand_trajectory_speed` | double | 1.0 rad/s | RT 컨트롤러 trajectory speed |
| `hand_max_traj_velocity` | double | 2.0 rad/s | RT 컨트롤러 max trajectory velocity |

#### MoveOpposition

엄지 + 대상 손가락 opposition 동작을 수행한다. 비-target 손가락은 자동으로 home으로 리셋되어 잔류 문제를 방지한다.

```xml
<MoveOpposition thumb_pose="thumb_index_oppose"
                target_finger="index"
                target_pose="index_oppose"
                hand_trajectory_speed="{hand_speed}"
                hand_max_traj_velocity="{hand_max_vel}"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `thumb_pose` | string | (필수) | 엄지 포즈 이름 |
| `target_finger` | string | (필수) | 대상 손가락 이름 (`"index"` / `"middle"` / `"ring"`) |
| `target_pose` | string | (필수) | 대상 손가락 포즈 이름 |
| `hand_trajectory_speed` | double | 1.0 rad/s | RT 컨트롤러 trajectory speed |
| `hand_max_traj_velocity` | double | 2.0 rad/s | RT 컨트롤러 max trajectory velocity |

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

#### IsObjectDetected / IsVisionTargetReady

`/world_target_info`에서 물체가 감지되었는지 확인하고, 포즈를 Blackboard에 출력한다.
Position은 vision에서, orientation은 현재 TCP에서 가져온다.

```xml
<IsVisionTargetReady pose="{object_pose}"/>
```

| 포트 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `pose` | output | Pose6D | 감지된 물체 포즈 (position: vision, orientation: TCP) |

#### IsGraspPhase

Force-PI grasp controller의 FSM 단계를 확인한다.

```xml
<IsGraspPhase phase="holding"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `phase` | string | "holding" | 확인할 FSM 단계 (`idle`/`approaching`/`contact`/`force_control`/`holding`/`releasing`) |

#### CheckShapeType

ShapeEstimate에서 shape type/name/confidence를 추출한다. `expected_type` 지정 시 매칭도 수행.

```xml
<CheckShapeType estimate="{shape_estimate}"
                shape_type="{shape_type}"
                shape_name="{shape_name}"
                confidence="{shape_confidence}"/>
```

| 포트 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `estimate` | input | ShapeEstimate | WaitShapeResult의 출력 |
| `expected_type` | input | string | (선택) 매칭할 shape 이름 |
| `shape_type` | output | uint8 | 숫자 타입 (0-4) |
| `shape_name` | output | string | 이름 (plane/sphere/cylinder/box) |
| `confidence` | output | double | 추정 신뢰도 [0,1] |

### Shape Estimation / ToF 노드

#### TriggerShapeEstimation

Shape estimation 파이프라인을 제어한다. `"start"` 시 자동으로 `/shape/clear` 서비스 호출 후 `/shape/trigger` 발행.

```xml
<TriggerShapeEstimation command="start"/>
```

| 포트 | 타입 | 설명 |
|------|------|------|
| `command` | string | `"start"` / `"stop"` / `"pause"` / `"resume"` / `"single"` |

#### WaitShapeResult

Shape estimation 결과가 confidence threshold를 넘을 때까지 대기한다.

```xml
<WaitShapeResult confidence_threshold="0.7" timeout_s="10.0" estimate="{shape_estimate}"/>
```

| 포트 | 방향 | 타입 | 기본값 | 설명 |
|------|------|------|--------|------|
| `confidence_threshold` | input | double | 0.7 | 최소 신뢰도 |
| `timeout_s` | input | double | 10.0 s | 타임아웃 |
| `estimate` | output | ShapeEstimate | - | shape estimation 결과 메시지 |

#### StartToFCollection

`/tof/snapshot` (500 Hz) 메시지 버퍼링을 시작한다. 이전 수집 데이터를 항상 초기화.

```xml
<StartToFCollection/>
```

#### StopToFCollection

ToF 버퍼링을 중지하고 수집된 snapshot 수를 출력한다.

```xml
<StopToFCollection count="{tof_count}"/>
```

| 포트 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `count` | output | int | 수집된 ToFSnapshot 수 |

#### ProcessSearchData

수집된 ToF 데이터를 처리하여 목표 포즈를 출력한다. **현재 processing logic은 미구현 (stub).**

```xml
<ProcessSearchData output_pose="{final_target}"/>
```

| 포트 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `output_pose` | output | Pose6D | task controller 목표 (x,y,z = 처리 결과, rpy = 현재 TCP) |

**데이터 흐름:**
- Input: `bridge->GetCollectedToFData()` — `std::vector<rtc_msgs::msg::ToFSnapshot>`
- Index finger ToF: `distances[2]` (A), `distances[3]` (B), `tip_poses[1]` (SE3)
- Output: Pose6D (stub은 현재 TCP pose 반환)

---

## 7. 기본 제공 BT 트리

### Common Motions (`common_motions.xml`)

모든 pick-and-place / manipulation 트리에서 공통으로 사용하는 재사용 가능한 SubTree 라이브러리.
`<include path="common_motions.xml"/>`로 다른 트리 파일에서 참조한다.

| SubTree | 설명 | 사용 트리 |
|---------|------|----------|
| `DetectObject` | 재시도 기반 물체 감지 | 모든 pick 트리 |
| `ApproachFromAbove` | 목표 위 오프셋으로 접근 | 모든 pick 트리, towel_unfold |
| `SlowDescend` | 저속 게인 설정 후 목표로 이동 (optional Z override) | 모든 pick 트리, towel_unfold |
| `ForceGrasp` | 병렬 GraspControl close + 힘 감지 + 파지 검증 | pick_and_place_contact_stop |
| `LiftAndVerify` | 리프트 + 파지 유지 검증 | contact_stop, force_pi |
| `ReleaseAndRetreat` | GraspControl open + 상방 후퇴 | contact_stop |
| `ContactStopOpen` | GraspControl open + settle | contact_stop |
| `ContactStopEmergencyAbort` | ContactStopOpen + retreat (실패 시 안전 종착) | contact_stop |
| `ForcePIGrasp` | Force-PI FSM 기반 adaptive grasp | force_pi |
| `ForcePIRelease` | Force-PI FSM release (grasp_command=2) | force_pi |
| `ForcePIGraspWithRetry` | ForcePIGrasp retry wrapper (hand-only 재시도) | force_pi |
| `ForcePIGraspDiagnose` | 실패 시 FSM phase 진단 로깅 | force_pi |
| `ForcePIEmergencyAbort` | ForcePIRelease + retreat (실패 시 안전 종착) | force_pi |
| `PoseBasedEmergencyAbort` | SetHandPose open + retreat (실패 시 안전 종착) | pick_and_place (pose-based) |

### Pick and Place — Pose-based (`pick_and_place.xml`)

**Grasp controller 미사용.** 미리 정의된 hand pose(soft/medium/hard)로 물체를 집는 가장 단순한 pick-and-place.
힘 피드백이 없으므로 in-transit drop guard나 파지 검증 단계가 없다.

**필수 Blackboard 변수:**
- `place_pose` — 목표 배치 포즈 (예: `"0.3;-0.3;0.15;3.14;0.0;0.0"`)
- `hand_close_pose` — grip 포즈 이름 (기본: `"hand_close_medium"`, launch arg `grip:=` 로 설정)

**동작 순서:**

| Phase | 동작 | 주요 파라미터 |
|-------|------|--------------|
| 1 | MoveToJoints `table_top` | joint controller |
| 2 | DetectObject (비전 감지) | 최대 10회 재시도 |
| 3 | SwitchController → task controller | |
| 3.5 | TCP re-orientation | `{tcp_rpy_offset_r/p/y}` (all 0 = no-op) |
| 4 | SetHandPose `hand_open` + ApproachFromAbove | +4cm Z |
| 5 | SlowDescend | traj_speed: 0.05 |
| 6 | SetHandPose `{hand_close_pose}` + settle | `{hand_close_settle_s}` |
| 7 | Lift (+12cm Z) | 힘 검증 없음 |
| 8 | Transport to `{place_pose}` | traj_speed: 0.1 |
| 9 | Lower (-5cm Z) | |
| 10 | SetHandPose `hand_open` + retreat (+8cm Z) | |

**실패 처리:** `PoseBasedEmergencyAbort` — hand open + retreat

### Pick and Place — Contact Stop (`pick_and_place_contact_stop.xml`)

**GraspControl force 기반 파지 (단일 시도, retry 없음).** GraspControl `close` 모드로 점진적으로
손을 닫으면서 IsForceAbove로 힘 감지. Transport/Lower 단계에서 IsGrasped drop guard 포함.

**필수 Blackboard 변수:**
- `place_pose` — 목표 배치 포즈
- `grasp_controller_type: "contact_stop"` in demo controller YAML

**동작 순서:**

| Phase | 동작 | 주요 파라미터 |
|-------|------|--------------|
| 1 | MoveToJoints `table_top` | joint controller |
| 2 | DetectObject | 최대 10회 재시도 |
| 3 | SwitchController → task controller | |
| 4 | ContactStopOpen + ApproachFromAbove | +4cm Z |
| 5 | SlowDescend | traj_speed: 0.05 |
| 6 | ForceGrasp (parallel close + force detect) | 1.5N, 2+ tips, 200ms |
| 7 | LiftAndVerify (+12cm Z) | 리프트 후 재검증 |
| 8 | Transport + IsGrasped guard | drop guard: 0.5N |
| 9 | Lower + IsGrasped guard | |
| 10 | ReleaseAndRetreat | retreat: +8cm Z |

**실패 처리:** `ContactStopEmergencyAbort` — GraspControl open + retreat

### Pick and Place — Force-PI (`pick_and_place_force_pi.xml`)

**Force-PI adaptive grasp (retry 지원).** 컨트롤러 내부 FSM
(Approaching → Contact → ForceControl → Holding)으로 force control 수행.
ForcePIGraspWithRetry로 hand-only 재시도 가능. TCP re-orientation 지원.

**필수 Blackboard 변수:**
- `place_pose` — 목표 배치 포즈
- `grasp_controller_type: "force_pi"` in demo controller YAML

**동작 순서:**

| Phase | 동작 | 주요 파라미터 |
|-------|------|--------------|
| 1 | MoveToJoints `table_top` | joint controller |
| 2 | DetectObject | 최대 10회 재시도 |
| 3 | SwitchController → task controller | |
| 3.5 | TCP re-orientation | `{tcp_rpy_offset_r/p/y}`, `{tcp_rotation_mode}` |
| 4 | ForcePIRelease (pre-open) + ApproachFromAbove | +4cm Z |
| 5 | SlowDescend | traj_speed: 0.05 |
| 6 | ForcePIGraspWithRetry | max_attempts, retry settle |
| 7 | LiftAndVerify (+12cm Z) | 리프트 후 재검증 |
| 8 | Transport + IsGrasped guard | drop guard: 0.5N |
| 9 | Lower + IsGrasped guard | |
| 10 | ForcePIRelease + retreat | retreat: +8cm Z |

**실패 처리:** `ForcePIEmergencyAbort` — FSM release + retreat

### Shape Inspect (`shape_inspect.xml`)

**ToF 기반 shape estimation 파이프라인.** Ready pose에서 vision 감지 → inspection 위치 접근 → shape estimation start → 결과 대기 → shape type 추출.

**필수 Blackboard 변수:**
- `inspect_offset_x/y` — object_pose 기준 inspect 위치 오프셋 [m]
- `inspect_approach_z` — 접근/후퇴 clearance [m]

**동작 순서:**

| Phase | 동작 | 주요 파라미터 |
|-------|------|--------------|
| 1 | MoveToJoints `ready` | joint controller |
| 2 | IsVisionTargetReady (비전 감지) | 최대 60회 재시도 × 0.5s |
| 3 | SwitchController → task controller | |
| 4 | 2-step 접근 (x/y lateral + z descend) | traj_speed: 0.1 → 0.05 |
| 5 | TriggerShapeEstimation `start` | /shape/clear + /shape/trigger |
| 6 | WaitShapeResult | confidence ≥ 0.7, timeout 10s |
| 7 | TriggerShapeEstimation `stop` + CheckShapeType | shape_type, shape_name 추출 |
| N | 2-step 복귀 (+z ascend + joint-space ready) | |

### Shape Inspect Simple (`shape_inspect_simple.xml`)

**Search move + 500 Hz ToF 데이터 수집.** Vision 기반 접근 후, -x 방향 선형 search를 수행하면서 `/tof/snapshot` 데이터를 ring buffer에 축적. 수집 후 ProcessSearchData stub에서 처리 (미구현).

**필수 Blackboard 변수:**
- `inspect_offset_x/y` — object_pose 기준 x/y 오프셋 [m]
- `inspect_constant_z` — 고정 Z (비전 Z 대신 사용) [m]
- `inspect_approach_z` — 접근/후퇴 clearance [m]
- `search_offset_x` — -x 방향 search 거리 [m] (음수)
- `search_speed` — search 시 trajectory speed [m/s]

**동작 순서:**

| Phase | 동작 | 주요 파라미터 |
|-------|------|--------------|
| 1 | MoveToJoints `ready` | joint controller |
| 2 | IsVisionTargetReady (비전 감지) | 최대 60회 재시도 × 0.5s |
| 3 | SwitchController → task controller | |
| 4 | 2-step 접근 (x/y + constant z) | SetPoseZ로 Z 고정 |
| 5 | StartToFCollection + search move + StopToFCollection | 0.02 m/s → ~2500 samples |
| 6 | ProcessSearchData (stub) | input: ToF buffer, output: goal x,y,z |
| 7 | MoveToPose to computed target | |
| N | 2-step 복귀 (+z ascend + joint-space ready) | |

**ToF 수집 사양:**
- 토픽: `/tof/snapshot` (BEST_EFFORT, 500 Hz)
- 버퍼: 최대 8192 snapshots (~16초분)
- 수집 예: search 10cm @ 0.02 m/s = 5초 → ~2500 samples
- Index finger ToF: `distances[2]` (index_A), `distances[3]` (index_B)

### Towel Unfold (`towel_unfold.xml`)

수건 가장자리를 핀치 파지하여 들어올린 후 호 형태로 스윕하여 펼치는 시퀀스.
`common_motions.xml`의 SubTree를 활용하며, 핀치 파지는 `pinch_motors` 파라미터 때문에 인라인으로 구현한다.

**필수 Blackboard 변수:**
- `sweep_direction_x` — 스윕 방향 X 성분 (예: `1.0`)
- `sweep_direction_y` — 스윕 방향 Y 성분 (예: `0.0`)
- `sweep_distance` — 스윕 거리 [m] (예: `0.3`)

**동작 순서:**

| 단계 | 동작 | 사용 SubTree | 주요 파라미터 |
|------|------|-------------|--------------|
| 1 | 비전으로 수건 가장자리 감지 | `DetectObject` | 최대 10회 재시도 |
| 2 | 핀치 프리셋 + 가장자리 상방 5cm 접근 | `ApproachFromAbove` | 엄지+검지 open |
| 3 | 저속 하강 + 핀치 파지 | `SlowDescend` + 인라인 | 0.5N, 1+ 핑거팁 |
| 4 | 25cm 들어올리기 | `LiftAndVerify` | 파지 검증 포함 |
| 5 | 컴플라이언트 게인 + 호 스윕 | (인라인) | kp_trans: 5.0 |
| 6 | 20cm 하강 | (인라인) | 강성 게인 복원 |
| 7 | 손 열기 + 10cm 후퇴 | `ReleaseAndRetreat` | retreat_speed: 0.1 m/s |

### Hand Motions Demo (`hand_motions.xml`)

UR5e가 고정 자세를 유지하는 동안, Hand가 가감속 opposition → wave(관절별 flex/extend 포함) 데모를 순차 수행한다.

**필수 Blackboard 변수:** 없음 (모두 트리 내부에서 초기화)

**내부 Blackboard 변수:**

| 변수 | 타입 | 용도 |
|------|------|------|
| `hand_speed` | double | SetGains hand_trajectory_speed (가감속 제어) |
| `hand_max_vel` | double | hand_max_traj_velocity (기본 2.0) |
| `use_home_return` | bool | Opposition 후 home 복귀 여부 |

**BT 구조:**

```
FullDemo (Parallel, success_count=1)
├── UR5eHoldPose(demo_pose)                ← 영구 RUNNING
└── HandDemoSequence (Sequence)
    ├── OppositionDemo (SubTree)
    │   ├── Phase 1: speed=0.5, home 경유 (1회 순회)
    │   ├── Phase 2: 가속 ×1.43, home 생략 (num_cycles회 순회)
    │   └── Phase 3: 감속 ×0.7, home 경유 (num_cycles회 순회)
    └── WaveDemo (SubTree)
        ├── Phase 1: 순차 flex/extend, 가속 ×1.43 (num_cycles회)
        ├── Phase 2: 동시(Parallel) flex/extend, 최고 속도 (num_cycles회)
        └── Phase 3: 순차 flex/extend, 감속 ×0.7 (num_cycles회)
```

**가감속 제어:**
- `SetGains hand_trajectory_speed`로 RT 컨트롤러와 동기화
- 가속 배율: `×1.42857` (= 1/0.7), 감속 배율: `×0.7` (대칭)
- 속도 범위: 0.5 ~ 3.0 rad/s, 종료 시 1.0으로 복원

**Hand 관절 매핑 (10-DoF):**

| 이름 | 관절 | DoF | 인덱스 |
|------|------|-----|--------|
| `thumb` | CMC abd/add, CMC flex/ext, MCP flex/ext | 3 | 0–2 |
| `thumb_mcp` | MCP flex/ext | 1 | 2 |
| `index` | MCP abd/add, MCP flex/ext, DIP flex/ext | 3 | 3–5 |
| `index_dip` | DIP flex/ext | 1 | 5 |
| `middle` | MCP abd/add, MCP flex/ext, DIP flex/ext | 3 | 6–8 |
| `middle_dip` | DIP flex/ext | 1 | 8 |
| `ring` | MCP flex/ext | 1 | 9 |

포즈 값은 `hand_pose_config.hpp`에 **도(°) 단위**로 정의하고, `DegToRad()` 래퍼로 자동 rad 변환된다.
하드웨어 캘리브레이션 후 placeholder 값을 실제 값으로 교체해야 한다.

---

## 8. 커스텀 BT 트리 작성

### 기본 구조

```xml
<root BTCPP_format="4" main_tree_to_execute="MyTask">

  <!-- 공통 모션 SubTree 라이브러리 포함 -->
  <include path="common_motions.xml"/>

  <BehaviorTree ID="MyTask">
    <Sequence name="main">
      <!-- 노드 배치 -->
    </Sequence>
  </BehaviorTree>

</root>
```

### 파일 위치

`trees/` 디렉토리에 XML 파일을 추가하면 자동으로 install에 포함된다.
`<include path="common_motions.xml"/>`를 사용하면 공통 SubTree를 바로 활용할 수 있다.

### 공통 SubTree 활용 예시

`common_motions.xml`에 정의된 SubTree를 호출하여 반복 패턴을 간결하게 작성할 수 있다:

```xml
<root BTCPP_format="4" main_tree_to_execute="MyPickTask">
  <include path="common_motions.xml"/>

  <BehaviorTree ID="MyPickTask">
    <Sequence>
      <!-- 1. 물체 감지 -->
      <SubTree ID="DetectObject"
               pose="{object_pose}" num_attempts="10" wait_s="0.5"/>

      <!-- 2. 상방 접근 -->
      <SubTree ID="ApproachFromAbove"
               target_pose="{object_pose}" offset_z="0.05"
               pos_tol="0.003" ori_tol="0.05" timeout_s="10.0"
               approach_pose="{approach_pose}"/>

      <!-- 3. 저속 하강 -->
      <SubTree ID="SlowDescend"
               target_pose="{object_pose}"
               traj_speed="0.05" max_traj_vel="0.1"
               pos_tol="0.002" ori_tol="0.05" timeout_s="8.0"/>

      <!-- 4. 힘 기반 파지 -->
      <SubTree ID="ForceGrasp"
               grasp_mode="close" close_speed="0.3" max_position="1.4"
               grasp_timeout_s="10.0"
               threshold_N="1.5" min_fingertips="2" sustained_ms="200"
               verify_force_N="1.0" verify_min_tips="2"/>

      <!-- 5. 리프트 + 파지 검증 -->
      <SubTree ID="LiftAndVerify"
               base_pose="{object_pose}" offset_z="0.10"
               traj_speed="0.08" max_traj_vel="0.2"
               pos_tol="0.005" timeout_s="6.0"
               verify_force_N="0.8" verify_min_tips="2"
               lift_pose="{lift_pose}"/>

      <!-- 6. 이동 + 놓기 + 후퇴 -->
      <MoveToPose target="{place_pose}" position_tolerance="0.005" timeout_s="12.0"/>

      <SubTree ID="ReleaseAndRetreat"
               base_pose="{place_pose}" retreat_z="0.08"
               retreat_speed="0.1" timeout_s="6.0"
               retreat_pose="{retreat_pose}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

### 공통 SubTree 포트 레퍼런스

#### DetectObject

```xml
<SubTree ID="DetectObject"
         pose="{output_pose}" num_attempts="10" wait_s="0.5"/>
```

| 포트 | 방향 | 설명 |
|------|------|------|
| `pose` | output | 감지된 물체 포즈 |
| `num_attempts` | input | 최대 재시도 횟수 |
| `wait_s` | input | 재시도 간 대기 시간 [s] |

#### ApproachFromAbove

```xml
<SubTree ID="ApproachFromAbove"
         target_pose="{pose}" offset_z="0.04"
         pos_tol="0.003" ori_tol="0.05" timeout_s="10.0"
         approach_pose="{approach}"/>
```

| 포트 | 방향 | 설명 |
|------|------|------|
| `target_pose` | input | 목표 포즈 |
| `offset_z` | input | Z축 오프셋 [m] |
| `pos_tol` | input | 위치 허용오차 [m] |
| `ori_tol` | input | 자세 허용오차 [rad] |
| `timeout_s` | input | 타임아웃 [s] |
| `approach_pose` | output | 계산된 접근 포즈 |

#### SlowDescend

```xml
<SubTree ID="SlowDescend"
         target_pose="{pose}" traj_speed="0.05" max_traj_vel="0.1"
         pos_tol="0.002" ori_tol="0.05" timeout_s="8.0"/>
```

| 포트 | 방향 | 설명 |
|------|------|------|
| `target_pose` | input | 하강 목표 포즈 |
| `traj_speed` | input | 궤적 속도 [m/s] |
| `max_traj_vel` | input | 최대 궤적 속도 [m/s] |
| `pos_tol` | input | 위치 허용오차 [m] |
| `ori_tol` | input | 자세 허용오차 [rad] |
| `timeout_s` | input | 타임아웃 [s] |

#### ForceGrasp

```xml
<SubTree ID="ForceGrasp"
         grasp_mode="close" close_speed="0.3" max_position="1.4"
         grasp_timeout_s="10.0"
         threshold_N="1.5" min_fingertips="2" sustained_ms="200"
         verify_force_N="1.0" verify_min_tips="2"/>
```

| 포트 | 방향 | 설명 |
|------|------|------|
| `grasp_mode` | input | `"close"` 또는 다른 모드 |
| `close_speed` | input | 닫기 속도 [rad/s] |
| `max_position` | input | 최대 닫기 위치 [rad] |
| `grasp_timeout_s` | input | 파지 타임아웃 [s] |
| `threshold_N` | input | 힘 임계값 [N] |
| `min_fingertips` | input | 최소 감지 핑거팁 수 |
| `sustained_ms` | input | 힘 유지 시간 [ms] |
| `verify_force_N` | input | 검증용 힘 임계값 [N] |
| `verify_min_tips` | input | 검증용 최소 핑거팁 수 |

#### LiftAndVerify

```xml
<SubTree ID="LiftAndVerify"
         base_pose="{object_pose}" offset_z="0.12"
         traj_speed="0.08" max_traj_vel="0.2"
         pos_tol="0.005" timeout_s="6.0"
         verify_force_N="0.8" verify_min_tips="2"
         lift_pose="{lift_pose}"/>
```

| 포트 | 방향 | 설명 |
|------|------|------|
| `base_pose` | input | 기준 포즈 (오프셋 적용 대상) |
| `offset_z` | input | Z축 리프트 거리 [m] |
| `traj_speed` | input | 궤적 속도 [m/s] |
| `max_traj_vel` | input | 최대 궤적 속도 [m/s] |
| `pos_tol` | input | 위치 허용오차 [m] |
| `timeout_s` | input | 타임아웃 [s] |
| `verify_force_N` | input | 파지 검증 힘 임계값 [N] |
| `verify_min_tips` | input | 파지 검증 최소 핑거팁 수 |
| `lift_pose` | output | 계산된 리프트 포즈 |

#### ReleaseAndRetreat

```xml
<SubTree ID="ReleaseAndRetreat"
         base_pose="{place_pose}" retreat_z="0.08"
         retreat_speed="0.1" timeout_s="6.0"
         retreat_pose="{retreat_pose}"/>
```

| 포트 | 방향 | 설명 |
|------|------|------|
| `base_pose` | input | 기준 포즈 (오프셋 적용 대상) |
| `retreat_z` | input | Z축 후퇴 거리 [m] |
| `retreat_speed` | input | 후퇴 궤적 속도 [m/s] |
| `timeout_s` | input | 타임아웃 [s] |
| `retreat_pose` | output | 계산된 후퇴 포즈 |

#### PoseBasedEmergencyAbort

```xml
<SubTree ID="PoseBasedEmergencyAbort"
         retreat_z="{grasp_abort_retreat_z}"
         current_gains="{current_gains}"/>
```

| 포트 | 방향 | 설명 |
|------|------|------|
| `retreat_z` | input | Z축 후퇴 거리 [m] |
| `current_gains` | input | 현재 컨트롤러 gains |

#### ContactStopEmergencyAbort

```xml
<SubTree ID="ContactStopEmergencyAbort"
         retreat_z="{grasp_abort_retreat_z}"
         current_gains="{current_gains}"/>
```

| 포트 | 방향 | 설명 |
|------|------|------|
| `retreat_z` | input | Z축 후퇴 거리 [m] |
| `current_gains` | input | 현재 컨트롤러 gains |

#### ForcePIEmergencyAbort

```xml
<SubTree ID="ForcePIEmergencyAbort"
         retreat_z="{grasp_abort_retreat_z}"
         current_gains="{current_gains}"/>
```

| 포트 | 방향 | 설명 |
|------|------|------|
| `retreat_z` | input | Z축 후퇴 거리 [m] |
| `current_gains` | input | 현재 컨트롤러 gains |

### 임의의 Hand 모션 추가 방법

새로운 Hand 포즈나 모션을 추가하는 절차:

#### Step 1: 포즈 정의 (`hand_pose_config.hpp`)

`kHandPoses` 맵에 새 포즈를 **도(°) 단위**로 추가한다:

```cpp
//                        Thumb              Index              Middle           Ring
//                        CMCab CMCfe MCPfe  MCPab MCPfe DIPfe  MCPab MCPfe DIPfe MCPfe
{"my_grasp",  DegToRad(HandPose{20.0, 50.0, 40.0,   0.0, 55.0, 35.0,   0.0, 55.0, 35.0,  50.0})},
```

단일 관절만 움직이는 포즈도 정의할 수 있다:

```cpp
// 특정 관절만 움직이는 flex 타겟
{"index_mcp_flex", DegToRad(HandPose{0.0, 0.0, 0.0,  0.0, 60.0, 0.0,  0.0, 0.0, 0.0,  0.0})},
```

#### Step 2 (선택): 관절 그룹 정의 (`kFingerJointIndices`)

단일 관절이나 관절 서브셋을 `FlexExtendFinger`에서 사용하려면 인덱스 매핑을 추가한다:

```cpp
{"index_mcp", {4}},        // index MCP flex/ext만
{"thumb_cmc", {0, 1}},     // thumb CMC 2관절만
```

`FlexExtendFinger`는 `finger_name + "_flex"`로 포즈를 자동 lookup하므로,
`kFingerJointIndices`에 `"index_mcp"` → `kHandPoses`에 `"index_mcp_flex"` 짝을 맞춘다.

#### Step 3: BT XML에서 사용

```xml
<!-- 방법 1: SetHandPose로 전체 Hand 이동 -->
<SetHandPose pose="my_grasp"
             hand_trajectory_speed="1.0"
             hand_max_traj_velocity="2.0"/>

<!-- 방법 2: MoveFinger로 특정 손가락만 이동 -->
<MoveFinger finger_name="index"
            pose="my_grasp"
            hand_trajectory_speed="1.0"
            hand_max_traj_velocity="2.0"/>

<!-- 방법 3: FlexExtendFinger로 flex/extend 1 cycle -->
<FlexExtendFinger finger_name="index_mcp"
                  hand_trajectory_speed="1.0"
                  hand_max_traj_velocity="2.0"/>

<!-- 방법 4: MoveOpposition으로 opposition 동작 -->
<MoveOpposition thumb_pose="thumb_index_oppose"
                target_finger="index"
                target_pose="index_oppose"
                hand_trajectory_speed="1.0"
                hand_max_traj_velocity="2.0"/>
```

#### Step 4: 커스텀 SubTree 구성

자주 사용하는 모션 시퀀스를 SubTree로 정의하면 재사용할 수 있다:

```xml
<root BTCPP_format="4" main_tree_to_execute="MyDemo">
  <include path="common_motions.xml"/>

  <!-- 재사용 가능한 커스텀 모션 SubTree -->
  <BehaviorTree ID="PinchSequence">
    <Sequence>
      <FlexExtendFinger finger_name="index_dip"
                        hand_trajectory_speed="{speed}"
                        hand_max_traj_velocity="{max_vel}"/>
      <FlexExtendFinger finger_name="index"
                        hand_trajectory_speed="{speed}"
                        hand_max_traj_velocity="{max_vel}"/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="MyDemo">
    <Parallel success_count="1" failure_count="1">
      <UR5eHoldPose pose="demo_pose"/>
      <Sequence>
        <!-- 커스텀 SubTree를 여러 번 호출 -->
        <Script code="speed := 0.5; max_vel := 2.0"/>
        <Repeat num_cycles="3">
          <SubTree ID="PinchSequence"
                   speed="{speed}" max_vel="{max_vel}"/>
        </Repeat>
      </Sequence>
    </Parallel>
  </BehaviorTree>
</root>
```

#### 요약: 파일 수정 체크리스트

| 변경 내용 | 수정 파일 | 빌드 필요 |
|-----------|----------|----------|
| 기존 포즈 튜닝 | `config/poses.yaml` | X (런타임 오버라이드) |
| 새 포즈 추가 (컴파일타임) | `hand_pose_config.hpp` | O (헤더 변경) |
| 새 포즈 추가 (런타임) | `config/poses.yaml` | X (기존 이름 덮어쓰기만 가능) |
| 새 관절 그룹 추가 | `hand_pose_config.hpp` | O |
| 새 BT 트리 추가 | `trees/*.xml` | X (XML은 런타임 로드) |
| 공통 SubTree 추가 | `trees/common_motions.xml` | X |
| 새 UR5e 포즈 추가 (컴파일타임) | `hand_pose_config.hpp` | O |
| 새 UR5e 포즈 추가 (런타임) | `config/poses.yaml` | X |

---

## 9. 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| 트리가 즉시 FAILURE | 비전 토픽 미발행 | `/world_target_info` 발행 확인 |
| 팔이 움직이지 않음 | RT Controller 미실행 또는 E-STOP 활성 | 컨트롤러 상태 및 E-STOP 확인 |
| 파지 타임아웃 | 힘 임계값이 너무 높거나 센서 미연결 | `threshold_N` 조정, `/hand/grasp_state` 확인 |
| 게인 변경이 반영 안 됨 | `/ur5e/controller_gains` 토픽 QoS 불일치 | RELIABLE QoS 확인 |
| "Tree completed with FAILURE" 로그 | 시퀀스 중 하나의 노드가 실패 | Groot2로 트리 실행 추적하여 실패 노드 확인 |

### 디버깅 유용 명령

```bash
# 토픽 모니터링
ros2 topic echo /ur5e/gui_position
ros2 topic echo /hand/grasp_state
ros2 topic echo /world_target_info

# 파라미터 확인
ros2 param list /bt_coordinator
ros2 param get /bt_coordinator tree_file

# 활성 컨트롤러 확인
ros2 topic echo /ur5e/active_controller_name

# Step 모드로 한 틱씩 디버깅
ros2 param set /bt_coordinator step_mode true
ros2 service call /bt_coordinator/step std_srvs/srv/Trigger

# Groot2 시각화 (포트 1667)
# 노드 시작 시 -p groot2_port:=1667 옵션 사용, Groot2 GUI에서 localhost:1667 연결

# 오프라인 트리 검증 (ROS 실행 불필요)
ros2 run ur5e_bt_coordinator validate_tree pick_and_place.xml
ros2 run ur5e_bt_coordinator validate_tree pick_and_place_contact_stop.xml
ros2 run ur5e_bt_coordinator validate_tree pick_and_place_force_pi.xml
ros2 run ur5e_bt_coordinator validate_tree shape_inspect.xml
ros2 run ur5e_bt_coordinator validate_tree shape_inspect_simple.xml
ros2 run ur5e_bt_coordinator validate_tree towel_unfold.xml
```
