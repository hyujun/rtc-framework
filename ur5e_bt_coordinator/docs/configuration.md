# 설정 및 실행 가이드

BehaviorTree 기반 UR5e + Hand 비실시간 태스크 코디네이터.
500Hz RT 제어 루프 외부에서 80Hz로 동작하며, ROS2 토픽을 통해 RT Controller와 통신한다.

---

## 사전 요구사항

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

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select ur5e_bt_coordinator
source install/setup.bash
```

---

## 실행 방법

### Launch 파일 (권장)

```bash
# 기본 실행 (YAML 기본 트리, YAML + 포즈 자동 로드)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py

# ── Pick and Place (pose-based grasp, grasp controller 미사용) ──
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place.xml
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place.xml grip:=soft
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place.xml grip:=hard

# ── Pick and Place (force-based grasp, grasp controller 필요) ──
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place_contact_stop.xml
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place_force_pi.xml

# ── Shape Inspection ──
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=shape_inspect.xml
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=shape_inspect_simple.xml

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

## 설정 파일

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
    # ... (추가 포즈: front_reach, side_reach, handover, stow,
    #      look_up, look_down, pick_ready, elevated, vision_ready)
```

런타임에도 변경 가능:
```bash
ros2 param set /bt_coordinator hand_pose.home "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```
