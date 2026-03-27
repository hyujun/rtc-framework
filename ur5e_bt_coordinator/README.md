# ur5e_bt_coordinator

BehaviorTree 기반 task coordinator for UR5e + Hand system.

500 Hz RT 제어 루프 밖에서 실행되는 non-RT 노드로, BehaviorTree.CPP v4를 사용하여
고수준 작업 시퀀스(FSM)를 XML로 정의하고 실행한다.

## Architecture

```
ur5e_bt_coordinator (non-RT, 20 Hz)
  │
  │ publish                          subscribe
  ├─ /ur5e/joint_goal ──────────►  RtControllerNode (500 Hz RT)
  ├─ /hand/joint_goal ──────────►    └─ DemoTaskController
  ├─ /ur5e/gains                       └─ DemoJointController
  ├─ /ur5e/select_controller
  │
  │ subscribe
  ├─ /ur5e/gui_position  ◄──────  RtControllerNode
  ├─ /hand/gui_position   ◄──────  RtControllerNode
  ├─ /hand/sensor_states/monitor ◄── hand_udp_node (RELIABLE QoS)
  ├─ /vision/object_pose  ◄──────  Vision node (external)
  ├─ /ur5e/active_controller_name
  └─ /system/estop_status
```

BT 노드는 topic만으로 RT 제어 레이어와 통신하며, 기존 컨트롤러 코드를 수정하지 않는다.

## QoS Policy

BT coordinator는 **RELIABLE QoS topic만 subscribe**한다.
BEST_EFFORT topic (`/hand/sensor_states`, `/joint_states` 등)은 RT 제어 전용이므로
BT에서 subscribe하지 않는다.

Fingertip force 데이터는 `hand_udp_node`가 publish하는
`/hand/sensor_states/monitor` (RELIABLE, depth 10) topic을 사용한다.

## BT Trees

| Tree | File | Description |
|------|------|-------------|
| Pick and Place | `trees/pick_and_place.xml` | Vision 기반 물체 감지, approach, force-based grasp, transport, release |
| Towel Unfold | `trees/towel_unfold.xml` | 수건 edge 감지, pinch grasp, lift, compliance sweep, release |

## BT Nodes

### Action Nodes

| Node | Type | Description |
|------|------|-------------|
| `MoveToPose` | StatefulAction | Task-space 6D 목표 이동, position/orientation tolerance 도달 판정 |
| `MoveToJoints` | StatefulAction | Joint-space 목표 이동, per-joint tolerance 도달 판정 |
| `GraspControl` | StatefulAction | Hand open/close/pinch/preset 제어, 점진적 닫기 지원 |
| `TrackTrajectory` | StatefulAction | Waypoint 시퀀스 순차 추적 (sweep motion 등) |
| `SetGains` | SyncAction | 컨트롤러 gain 동적 변경 (kp, trajectory speed 등) |
| `SwitchController` | StatefulAction | 활성 컨트롤러 전환 (joint ↔ task) |
| `ComputeOffsetPose` | SyncAction | Pose에 XYZ offset 적용 (approach, lift, retreat 계산) |
| `ComputeSweepTrajectory` | SyncAction | Arc sweep 경로 waypoint 생성 (towel unfold용) |
| `WaitDuration` | StatefulAction | 지정 시간 대기 |

### Condition Nodes

| Node | Description |
|------|-------------|
| `IsForceAbove` | Fingertip force가 threshold 초과 (sustained 판정 지원) |
| `IsGrasped` | 물체 파지 상태 확인 (force + contact flag) |
| `IsObjectDetected` | Vision 결과 수신 여부 확인, pose 출력 |

## Dependencies

- `ros-jazzy-behaviortree-cpp` (BehaviorTree.CPP v4)
- `rtc_msgs` (GuiPosition, HandSensorState, FingertipSensor)
- `geometry_msgs` (PoseStamped — vision interface)
- `tf2` (quaternion → RPY 변환)

## Build

```bash
# BehaviorTree.CPP 설치 (최초 1회)
sudo apt install ros-jazzy-behaviortree-cpp

# 빌드
cd ~/ros2_ws/rtc_ws
colcon build --packages-select ur5e_bt_coordinator
```

또는 `./build.sh` 실행 시 자동으로 빌드됨.

## Usage

```bash
# Pick and Place
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=pick_and_place.xml -p tick_rate_hz:=20.0

# Towel Unfold
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=towel_unfold.xml
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tree_file` | `pick_and_place.xml` | BT XML 파일명 (`trees/` 디렉토리 기준) |
| `tick_rate_hz` | `20.0` | BT tick 주기 [Hz] |

### Blackboard Variables

Tree 실행 전 Blackboard에 설정해야 하는 변수:

**Pick and Place:**
- `place_pose`: 물체를 놓을 목표 pose (format: `"x;y;z;roll;pitch;yaw"`)

**Towel Unfold:**
- `sweep_direction_x`, `sweep_direction_y`: sweep 방향 벡터
- `sweep_distance`: sweep 거리 [m]

## File Structure

```
ur5e_bt_coordinator/
├── config/bt_coordinator.yaml       # ROS2 parameters
├── trees/
│   ├── pick_and_place.xml           # Scene A: object grasping
│   └── towel_unfold.xml             # Scene B: towel unfolding
├── include/ur5e_bt_coordinator/
│   ├── bt_types.hpp                 # Pose6D, FingertipForce, BT conversions
│   ├── bt_ros_bridge.hpp            # ROS topic ↔ BT bridge
│   ├── bt_coordinator_node.hpp      # Main node
│   ├── action_nodes/                # 9 action node headers
│   └── condition_nodes/             # 3 condition node headers
└── src/
    ├── main.cpp                     # Entry point
    ├── bt_coordinator_node.cpp      # Node initialization, BT tick loop
    ├── bt_ros_bridge.cpp            # Topic subscriptions and publishers
    └── nodes/                       # 12 node implementations
```
