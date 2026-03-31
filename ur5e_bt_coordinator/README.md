# ur5e_bt_coordinator

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.

UR5e + 10-DoF Hand 시스템을 위한 BehaviorTree 기반 task coordinator.

500 Hz RT 제어 루프 밖에서 실행되는 non-RT 노드로, BehaviorTree.CPP v4를 사용하여
고수준 작업 시퀀스를 XML로 정의하고 실행한다.

## 개요

`bt_coordinator` 노드는 설정된 BT XML 트리를 로드하고, 지정된 주기(기본 100 Hz)로
tick하면서 BT 노드들을 실행한다. 각 BT 노드는 `BtRosBridge`를 통해 ROS2 topic으로
RT 제어 레이어와 통신하며, 기존 컨트롤러 코드를 수정하지 않는다.

E-STOP이 활성화되면 트리 tick이 자동으로 일시 정지된다.

## 아키텍처

```
bt_coordinator (non-RT, 100 Hz)
  │
  │ publish                          subscribe
  ├─ /ur5e/joint_goal ──────────►  RtControllerNode (500 Hz RT)
  ├─ /hand/joint_goal ──────────►    └─ DemoTaskController
  ├─ /ur5e/gains                       └─ DemoJointController
  ├─ /ur5e/select_controller
  │
  │ subscribe
  ├─ /ur5e/gui_position  ◄──────  RtControllerNode
  ├─ /hand/gui_position  ◄──────  RtControllerNode
  ├─ /hand/grasp_state   ◄──────  RtControllerNode (500Hz grasp detection)
  ├─ /vision/object_pose ◄──────  Vision 노드 (외부)
  ├─ /ur5e/active_controller_name
  └─ /system/estop_status
```

## QoS 정책

BT coordinator는 RT Controller 파이프라인의 **RELIABLE QoS topic만 subscribe**한다.
BEST_EFFORT topic (`/hand/sensor_states`, `/joint_states` 등)은 RT 제어 전용이므로
BT에서 subscribe하지 않는다.

Grasp 상태 데이터는 RT Controller가 500Hz로 계산하여 publish하는
`/hand/grasp_state` (`rtc_msgs/GraspState`, depth 10) topic을 사용한다.
Fingertip별 force magnitude와 aggregate grasp detection 결과가 포함되어 있어
BT 노드에서 별도 계산 없이 직접 활용 가능하다.

## Topic 인터페이스

### 발행 (Publish)

| Topic | 메시지 타입 | 설명 |
|-------|------------|------|
| `/ur5e/joint_goal` | `rtc_msgs/RobotTarget` | Arm task-space 또는 joint-space 목표 |
| `/hand/joint_goal` | `rtc_msgs/RobotTarget` | Hand 10-DoF 모터 목표 |
| `/ur5e/gains` | `std_msgs/Float64MultiArray` | 컨트롤러 gain 업데이트 (16개 요소) |
| `/ur5e/select_controller` | `std_msgs/String` | 컨트롤러 전환 명령 |

### 구독 (Subscribe)

| Topic | 메시지 타입 | QoS | 설명 |
|-------|------------|-----|------|
| `/ur5e/gui_position` | `rtc_msgs/GuiPosition` | RELIABLE, depth 10 | TCP 포즈 + 관절 위치 |
| `/hand/gui_position` | `rtc_msgs/GuiPosition` | RELIABLE, depth 10 | Hand 관절 위치 |
| `/hand/grasp_state` | `rtc_msgs/GraspState` | RELIABLE, depth 10 | 500Hz 사전 계산된 grasp 상태 |
| `/vision/object_pose` | `geometry_msgs/PoseStamped` | RELIABLE, depth 10 | 물체 위치 (쿼터니언 → RPY 변환) |
| `/ur5e/active_controller_name` | `std_msgs/String` | TRANSIENT_LOCAL, depth 1 | 현재 활성 컨트롤러 이름 |
| `/system/estop_status` | `std_msgs/Bool` | RELIABLE, depth 10 | E-STOP 상태 |

## BT 트리

| 트리 | 파일 | 설명 |
|------|------|------|
| Pick and Place | `trees/pick_and_place.xml` | Vision 기반 물체 감지 → approach → force-based grasp → lift → transport → lower/release → retreat |
| Towel Unfold | `trees/towel_unfold.xml` | 수건 edge 감지 → pinch pre-shape → approach → pinch grasp → lift → compliant sweep → lower/release → retreat |
| Hand Motions | `trees/hand_motions.xml` | UR5e 자세 유지(UR5eHoldPose) + Hand 가감속 opposition/wave 데모 (OppositionDemo → WaveDemo) |

## BT 노드

### Action 노드

| 노드 | 타입 | 설명 | 입력 포트 |
|------|------|------|----------|
| `MoveToPose` | StatefulAction | Task-space 6D 목표 이동, position/orientation tolerance 도달 판정 | `target`, `position_tolerance`(0.005), `orientation_tolerance`(0.05), `timeout_s`(10.0) |
| `MoveToJoints` | StatefulAction | Joint-space 목표 이동, per-joint tolerance 도달 판정 | `target`, `tolerance`(0.01), `timeout_s`(10.0) |
| `GraspControl` | StatefulAction | Hand open/close/pinch/preset 제어, 점진적 닫기 지원 | `mode`(close), `target_positions`, `close_speed`(0.3), `max_position`(1.4), `pinch_motors`("0,1,2,3"), `timeout_s`(8.0) |
| `TrackTrajectory` | StatefulAction | Waypoint 시퀀스 순차 추적 (sweep motion 등) | `waypoints`, `position_tolerance`(0.01), `timeout_s`(30.0) |
| `SetGains` | SyncAction | 컨트롤러 gain 동적 변경 (16개 요소 배열) | `kp_translation`, `kp_rotation`, `trajectory_speed`, `trajectory_angular_speed`, `max_traj_velocity`, `max_traj_angular_velocity`, `hand_trajectory_speed`, `hand_max_traj_velocity`, `full_gains` |
| `SwitchController` | StatefulAction | 활성 컨트롤러 전환 (joint ↔ task) | `controller_name`, `timeout_s`(3.0) |
| `ComputeOffsetPose` | SyncAction | Pose에 XYZ offset 적용 (approach, lift, retreat 계산) | `input_pose`, `offset_x`(0.0), `offset_y`(0.0), `offset_z`(0.0) → 출력: `output_pose` |
| `ComputeSweepTrajectory` | SyncAction | Arc sweep 경로 waypoint 생성 (towel unfold용, sinusoidal arc 프로파일) | `start_pose`, `direction_x`(1.0), `direction_y`(0.0), `distance`(0.3), `arc_height`(0.05), `num_waypoints`(8) → 출력: `waypoints` |
| `WaitDuration` | StatefulAction | 지정 시간 대기 | `duration_s`(0.5) |
| `MoveFinger` | StatefulAction | 특정 손가락을 명명된 포즈로 이동 (trajectory duration 추정 기반 완료, partial hand update) | `finger_name`, `pose`, `hand_trajectory_speed`(1.0), `hand_max_traj_velocity`(2.0) |
| `FlexExtendFinger` | StatefulAction | 손가락 flex→extend 1 cycle (2-phase, phase별 trajectory duration 추정) | `finger_name`, `hand_trajectory_speed`(1.0), `hand_max_traj_velocity`(2.0) |
| `SetHandPose` | StatefulAction | 전체 Hand 10-DoF를 명명된 포즈로 이동 (trajectory duration 추정 기반 완료) | `pose`, `hand_trajectory_speed`(1.0), `hand_max_traj_velocity`(2.0) |
| `UR5eHoldPose` | StatefulAction | UR5e 목표 자세 도달 후 영구 RUNNING (halt까지 유지) | `pose` |
| `MoveOpposition` | StatefulAction | Opposition 동작 (thumb+target 포즈, 비-target home 리셋, trajectory duration 추정 완료) | `thumb_pose`, `target_finger`, `target_pose`, `hand_trajectory_speed`(1.0), `hand_max_traj_velocity`(2.0) |

### Condition 노드

| 노드 | 설명 | 입력 포트 |
|------|------|----------|
| `IsForceAbove` | Fingertip force가 threshold 초과 여부 확인 (500Hz 사전 계산 활용, sustained 판정 지원) | `threshold_N`(1.5), `min_fingertips`(2), `sustained_ms`(0) |
| `IsGrasped` | 물체 파지 상태 확인 (500Hz 사전 계산된 grasp_detected 활용) | `force_threshold_N`(1.0), `min_fingertips`(2) |
| `IsObjectDetected` | Vision 결과 수신 여부 확인 | 출력: `pose` |

## 설정 파일

### ROS2 파라미터 (`config/bt_coordinator.yaml`)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `tree_file` | `"pick_and_place.xml"` | BT XML 파일명 (`trees/` 디렉토리 기준) |
| `tick_rate_hz` | `100.0` | BT tick 주기 [Hz] |
| `repeat` | `false` | `true`면 트리 SUCCESS 완료 후 자동 반복 (FAILURE 시 정지) |
| `repeat_delay_s` | `1.0` | 반복 시 재시작 전 대기 시간 [s] |

반복 모드에서 트리 재시작 시 `object_pose` blackboard 변수가 자동으로 초기화된다.

### Blackboard 변수

트리 실행 전 Blackboard에 설정해야 하는 변수:

**Pick and Place (`pick_and_place.xml`):**
- `place_pose`: 물체를 놓을 목표 pose (형식: `"x;y;z;roll;pitch;yaw"`)

**Towel Unfold (`towel_unfold.xml`):**
- `sweep_direction_x`, `sweep_direction_y`: sweep 방향 벡터
- `sweep_distance`: sweep 거리 [m]

### Hand/UR5e 포즈 설정 (`hand_pose_config.hpp`)

코드 내 `kHandPoses` 맵에 정의된 명명 포즈 (10-DoF):

| 포즈 이름 | 용도 |
|-----------|------|
| `home` | 기본 포즈 (전체 0) |
| `full_flex` | 전체 손가락 flexion |
| `thumb_index_oppose` / `index_oppose` | 엄지-검지 opposition |
| `thumb_middle_oppose` / `middle_oppose` | 엄지-중지 opposition |
| `thumb_ring_oppose` / `ring_oppose` | 엄지-약지 opposition |
| `thumb_flex` / `index_flex` / `middle_flex` / `ring_flex` | FlexExtendFinger용 flex 타겟 |

`kUR5ePoses` 맵에 정의된 UR5e 포즈 (6-DoF):

| 포즈 이름 | 용도 |
|-----------|------|
| `home_pose` | 기본 자세 |
| `demo_pose` | 데모 자세 |

손가락-관절 인덱스 매핑 (`kFingerJointIndices`):

| 손가락 | 관절 | DoF | 인덱스 |
|--------|------|-----|--------|
| thumb | CMC abd/add, CMC flex/ext, MCP flex/ext | 3 | 0-2 |
| index | MCP abd/add, MCP flex/ext, DIP flex/ext | 3 | 3-5 |
| middle | MCP abd/add, MCP flex/ext, DIP flex/ext | 3 | 6-8 |
| ring | MCP flex/ext | 1 | 9 |

## SetGains 배열 레이아웃

`SetGains` 노드가 발행하는 16개 요소 gain 배열 (DemoTaskController 기준):

| 인덱스 | 필드 | 기본값 |
|--------|------|--------|
| 0-2 | `kp_translation` (X, Y, Z) | 15.0, 15.0, 15.0 |
| 3-5 | `kp_rotation` (R, P, Y) | 5.0, 5.0, 5.0 |
| 6 | `damping` | 0.01 |
| 7 | `null_kp` | 0.5 |
| 8 | `enable_null_space` | 0.0 |
| 9 | `control_6dof` | 1.0 |
| 10 | `trajectory_speed` | 0.1 |
| 11 | `trajectory_angular_speed` | 0.5 |
| 12 | `hand_trajectory_speed` | 1.0 |
| 13 | `max_traj_velocity` | 0.5 |
| 14 | `max_traj_angular_velocity` | 1.0 |
| 15 | `hand_max_traj_velocity` | 2.0 |

## 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS2 C++ 클라이언트 |
| `behaviortree_cpp` | BehaviorTree.CPP v4 (`ros-jazzy-behaviortree-cpp`) |
| `std_msgs` | Float64MultiArray, String, Bool |
| `geometry_msgs` | PoseStamped (vision 인터페이스) |
| `rtc_msgs` | GuiPosition, GraspState, RobotTarget |
| `tf2` | 쿼터니언 → RPY 변환 |
| `ament_index_cpp` | 패키지 share 디렉토리 탐색 (빌드 의존성) |

## 빌드

```bash
# BehaviorTree.CPP 설치 (최초 1회)
sudo apt install ros-jazzy-behaviortree-cpp

# 빌드
cd ~/ros2_ws/rtc_ws
colcon build --packages-select ur5e_bt_coordinator
```

또는 `./build.sh` 실행 시 자동으로 빌드됨.

## 실행

```bash
# Pick and Place (1회 실행)
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=pick_and_place.xml -p tick_rate_hz:=100.0

# Towel Unfold
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=towel_unfold.xml

# Hand Motions Demo (UR5e 자세 유지 + 가감속 opposition/wave)
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=hand_motions.xml

# Pick and Place 반복 실행
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=pick_and_place.xml -p repeat:=true -p repeat_delay_s:=2.0

# YAML 설정 파일 사용
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args --params-file config/bt_coordinator.yaml
```

## 파일 구조

```
ur5e_bt_coordinator/
├── config/bt_coordinator.yaml       # ROS2 파라미터
├── trees/
│   ├── pick_and_place.xml           # 물체 파지 시나리오
│   ├── towel_unfold.xml             # 수건 펼치기 시나리오
│   └── hand_motions.xml             # Hand 민첩성 데모 시나리오
├── include/ur5e_bt_coordinator/
│   ├── bt_types.hpp                 # Pose6D, CachedGraspState, BT 타입 변환
│   ├── bt_utils.hpp                 # 유틸리티 함수 (시간, map lookup, CSV 파싱, partial hand update)
│   ├── bt_ros_bridge.hpp            # ROS topic ↔ BT bridge 헤더
│   ├── bt_coordinator_node.hpp      # 메인 노드 헤더
│   ├── hand_pose_config.hpp         # Hand/UR5e 포즈 lookup map, 손가락-관절 인덱스 매핑
│   ├── action_nodes/                # 14개 action 노드 헤더
│   └── condition_nodes/             # 3개 condition 노드 헤더
└── src/
    ├── main.cpp                     # 진입점
    ├── bt_coordinator_node.cpp      # 노드 초기화, BT tick 루프
    ├── bt_ros_bridge.cpp            # Topic 구독 및 발행
    └── nodes/                       # 17개 노드 구현체
```

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
