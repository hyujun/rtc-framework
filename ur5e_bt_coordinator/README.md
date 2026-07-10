# ur5e_bt_coordinator

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.

UR5e + 10-DoF Hand 시스템을 위한 BehaviorTree 기반 task coordinator.

500 Hz RT 제어 루프 밖에서 실행되는 non-RT 노드로, BehaviorTree.CPP v4를 사용하여
고수준 작업 시퀀스를 XML로 정의하고 실행한다.

## 개요

`bt_coordinator` LifecycleNode는 설정된 BT XML 트리를 로드하고, 지정된 주기(기본 80 Hz)로
tick하면서 BT 노드들을 실행한다. 각 BT 노드는 `BtRosBridge`를 통해 ROS2 topic으로
RT 제어 레이어와 통신하며, 기존 컨트롤러 코드를 수정하지 않는다.

E-STOP이 활성화되면 트리 tick이 자동으로 일시 정지된다.

## 아키텍처

```
bt_coordinator (non-RT, 80 Hz)
  │
  │ publish                              subscribe
  ├─ <ns>/<arm_group>/joint_goal ──►  RtControllerNode (500 Hz RT)
  ├─ <ns>/<hand_group>/joint_goal ─►    └─ DemoTaskController
  │                                      └─ DemoJointController
  │ subscribe
  ├─ /rtc_cm/<arm_group>/joint_states  ◄──  RtControllerNode (fixed path, sensor_msgs/JointState)
  ├─ /rtc_cm/<hand_group>/joint_states ◄──  RtControllerNode (fixed path, sensor_msgs/JointState)
  ├─ base → tool0_actual (tf2 lookup)  ◄──  TCP pose (controller publishes transforms)
  ├─ <ns>/<hand_group>/grasp_state ◄─  Force-PI grasp 컨트롤러 (500Hz)
  ├─ <ns>/<hand_group>/wbc_state   ◄─  WBC 컨트롤러 (500Hz, TSID FSM phase + 진단)
  ├─ /world_target_info ◄───────────  Vision 노드 (외부)
  ├─ /rtc_cm/active_controller_name
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

Phase 4~: `<ns>`는 active controller namespace (`/demo_joint_controller`, `/demo_task_controller`, `/demo_wbc_controller` 등). `/rtc_cm/active_controller_name`이 수신될 때마다 `RewireControllerTopics()`가 sub/pub을 재바인딩합니다. 아래 표의 `ur5e`/`hand` 세그먼트는 `arm_group`/`hand_group` 파라미터 값 (default `ur5e`/`hand`) — robot-agnostic (Seam A).

| Topic | 메시지 타입 | 설명 |
|-------|------------|------|
| `<ns>/<arm_group>/joint_goal` | `rtc_msgs/RobotTarget` | Arm task-space 또는 joint-space 목표 (controller-owned) |
| `<ns>/<hand_group>/joint_goal` | `rtc_msgs/RobotTarget` | Hand 모터 목표 (controller-owned) |

게인 변경은 토픽이 아닌 active controller LifecycleNode의 ROS 2 parameter (`SetGains` BT node가 `set_parameters_atomically`로 호출). 컨트롤러 전환은 `/rtc_cm/switch_controller` srv (`SwitchController` BT node).

### 구독 (Subscribe)

| Topic | 메시지 타입 | QoS | 설명 |
|-------|------------|-----|------|
| `/rtc_cm/<arm_group>/joint_states` | `sensor_msgs/JointState` | depth 10 | Arm 관절 위치 (fixed path, active controller 와 무관 — controller-agnostic) |
| `/rtc_cm/<hand_group>/joint_states` | `sensor_msgs/JointState` | depth 10 | Hand 관절 위치 (fixed path, controller-agnostic) |
| `<ns>/<hand_group>/grasp_state` | `rtc_msgs/GraspState` | RELIABLE, depth 10 | 500Hz 사전 계산된 grasp 상태 (Force-PI grasp 컨트롤러 전용; controller-owned) |
| `<ns>/<hand_group>/wbc_state` | `rtc_msgs/WbcState` | RELIABLE, depth 10 | 500Hz WBC FSM phase + 핑거팁 raw + TSID 진단 (TSID-based WBC 컨트롤러 전용; controller-owned). BT 는 grasp_state 와 함께 항상 subscribe — active controller 가 발행하는 쪽이 캐시 채움 |
| `<ns>/tof/snapshot` | `rtc_msgs/ToFSnapshot` | BEST_EFFORT, depth 100 | ToF + 핑거팁 pose snapshot (controller-owned) |
| `<ns>/transforms` | `tf2_msgs/TFMessage` | RELIABLE, depth 10 | active controller의 FK `base → tool0_actual` (controller-owned, rewire). `tf_buffer_`에 직접 feed → TCP pose lookup 소스 |
| `/world_target_info` | `geometry_msgs/Polygon` | depth 10 | 비전 물체 위치 (`points[0]` = x,y,z; orientation 없음 — roll/pitch/yaw는 0으로 채움) |
| `/rtc_cm/active_controller_name` | `std_msgs/String` | TRANSIENT_LOCAL, depth 1 | 현재 활성 컨트롤러 이름 — rewire 트리거 |
| `/system/estop_status` | `std_msgs/Bool` | RELIABLE, depth 10 | E-STOP 상태 |

### TF

TCP 포즈는 토픽이 아닌 `tf2_ros::Buffer` lookup으로 얻는다: `base` → `tool0_actual`.
이 프레임은 active controller가 `<ns>/transforms` (전용 `/tf` publisher 없음) 로만 발행하므로,
`transforms_sub_`가 controller 전환마다 rewire 되어 그 `TFMessage`를 `tf_buffer_`에 직접 feed 한다
(bare `TransformListener`는 `/tf`만 듣기 때문에 이 프레임을 못 받는다 — 외부 `/tf` 재발행 불필요).

## BT 트리

| 트리 | 파일 | 설명 |
|------|------|------|
| Common Motions | `trees/common_motions.xml` | 재사용 가능한 공통 모션 SubTree 라이브러리 (DetectObject, ForceGrasp, LiftAndVerify, PoseBasedEmergencyAbort 등) |
| Pick and Place | `trees/pick_and_place.xml` | Pose-based grasp: vision 감지 → approach → SetHandPose 기반 grip (soft/medium/hard) → lift → transport → release. Grasp controller 미사용 |
| Pick and Place (Contact Stop) | `trees/pick_and_place_contact_stop.xml` | Force-based grasp (contact_stop): vision 감지 → approach → force-based grasp → lift → transport → release |
| Pick and Place (Force-PI) | `trees/pick_and_place_force_pi.xml` | Force-PI adaptive grasp: vision 감지 → approach → force-PI grasp (retry 지원) → lift → transport → release |
| Towel Unfold | `trees/towel_unfold.xml` | 수건 edge 감지 → pinch pre-shape → approach → pinch grasp → lift → compliant sweep → lower/release → retreat |
| Hand Motions | `trees/hand_motions.xml` | UR5e 자세 유지 + Hand 데모 (OppositionDemo → WaveDemo) |
| Vision Approach | `trees/vision_approach.xml` | Vision 기반 approach 데모 (arm-only, 핸드 미사용) |
| Shape Inspect | `trees/shape_inspect.xml` | ToF 센서 기반 shape estimation 워크플로우 (start → wait → stop → evaluate) |
| Shape Inspect Simple | `trees/shape_inspect_simple.xml` | Vision 기반 inspect 위치 이동 → -x 방향 linear search move + ToF 500Hz 데이터 수집 → 데이터 처리 → 목표 이동. 서비스 기반 shape estimation 미사용 |
| Search Motion | `trees/search_motion.xml` | 팔 sweep + tilt scan 탐색 모션 |

## BT 노드

### Action 노드

| 노드 | 타입 | 설명 | 입력 포트 |
|------|------|------|----------|
| `MoveToPose` | StatefulAction | Task-space 6D 목표 이동, position/orientation tolerance 도달 판정 | `target`, `position_tolerance`(0.005), `orientation_tolerance`(0.05), `timeout_s`(10.0) |
| `MoveToJoints` | StatefulAction | Joint-space 목표 이동, per-joint tolerance 도달 판정 | `target`, `tolerance`(0.01), `timeout_s`(10.0) |
| `GraspControl` | StatefulAction | Hand open/close/pinch/preset 제어, 점진적 닫기 지원 | `mode`(close), `target_positions`, `close_speed`(0.3), `max_position`(1.4), `pinch_motors`("0,1,2,3"), `timeout_s`(8.0) |
| `TrackTrajectory` | StatefulAction | Waypoint 시퀀스 순차 추적 (sweep motion 등) | `waypoints`, `position_tolerance`(0.01), `timeout_s`(30.0) |
| `SetGains` | SyncAction | active controller LifecycleNode의 ROS 2 parameter 동적 변경 (`set_parameters_atomically`). 입력 포트 중 채워진 것만 dispatch — 컨트롤러별 매핑은 `set_gains.cpp` 참조 (예: `trajectory_speed` → DemoJoint면 `robot_trajectory_speed`, DemoWbc면 `arm_trajectory_speed`). `grasp_command`/`grasp_target_force`는 ROS 2 srv (`rtc_msgs/srv/GraspCommand`)로 분기. read-only 파라미터 (`*_max_traj_velocity`)는 변경 불가 (rejected) | `trajectory_speed`, `trajectory_angular_speed`, `hand_trajectory_speed`, `kp_translation`, `kp_rotation`, `damping`, `null_kp`, `enable_null_space`, `control_6dof`, `grasp_contact_threshold`, `grasp_force_threshold`, `grasp_min_fingertips`, `se3_weight`, `force_weight`, `posture_weight`, `mpc_enable`, `riccati_gain_scale`, `grasp_command`, `grasp_target_force` |
| `SwitchController` | StatefulAction | 활성 컨트롤러 전환 (joint ↔ task) | `controller_name`, `timeout_s`(3.0) |
| `ComputeOffsetPose` | SyncAction | Pose에 XYZ offset 적용 (approach, lift, retreat 계산) | `input_pose`, `offset_x`(0.0), `offset_y`(0.0), `offset_z`(0.0) → 출력: `output_pose` |
| `SetPoseZ` | SyncAction | Pose의 Z좌표를 절대값으로 덮어씀 (X, Y, 방향 유지). `z`가 NaN(기본값)이면 pass-through. Object final goal의 Z를 고정하는 용도 | `input_pose`, `z`(NaN) → 출력: `output_pose` |
| `ComputeSweepTrajectory` | SyncAction | Arc sweep 경로 waypoint 생성 (towel unfold용, sinusoidal arc 프로파일) | `start_pose`, `direction_x`(1.0), `direction_y`(0.0), `distance`(0.3), `arc_height`(0.05), `num_waypoints`(8) → 출력: `waypoints` |
| `ComputeTiltSequence` | SyncAction | Roll/pitch를 번갈아 오실레이션하는 tilt-scan waypoint 시퀀스 생성 (position 고정, ExplorationMotionGenerator tilt phase 모사) | `base_pose`, `amplitude_deg`(15.0), `num_steps`(6) → 출력: `waypoints` |
| `GetCurrentPose` | SyncAction | ROS bridge에서 현재 TCP pose를 읽어 Blackboard에 기록 (sweep/tilt 시퀀스 시작점 캡처용) | 출력: `pose` |
| `WaitDuration` | StatefulAction | 지정 시간 대기 | `duration_s`(0.5) |
| `MoveFinger` | StatefulAction | 특정 손가락을 명명된 포즈로 이동 (trajectory duration 추정 기반 완료, partial hand update) | `finger_name`, `pose`, `hand_trajectory_speed`(1.0) |
| `FlexExtendFinger` | StatefulAction | 손가락 flex→extend 1 cycle (2-phase, phase별 trajectory duration 추정) | `finger_name`, `hand_trajectory_speed`(1.0) |
| `SetHandPose` | StatefulAction | 전체 Hand 10-DoF를 명명된 포즈로 이동 (trajectory duration 추정 기반 완료) | `pose`, `hand_trajectory_speed`(1.0) |
| `UR5eHoldPose` | StatefulAction | UR5e 목표 자세 도달 후 영구 RUNNING (halt까지 유지) | `pose` |
| `MoveOpposition` | StatefulAction | Opposition 동작 (thumb+target 포즈, 비-target home 리셋, trajectory duration 추정 완료) | `thumb_pose`, `target_finger`, `target_pose`, `hand_trajectory_speed`(1.0) |
| `TriggerShapeEstimation` | SyncAction | Shape estimation 시작/정지 제어 (서비스 호출) | `action`(start/stop) |
| `WaitShapeResult` | StatefulAction | Shape estimation 결과 대기 (confidence 임계값 도달까지) | `min_confidence`(0.8), `timeout_s`(10.0) |
| `StartToFCollection` | SyncAction | `<ns>/tof/snapshot` 메시지 버퍼링 시작 (기존 데이터 초기화) | — |
| `StopToFCollection` | SyncAction | ToF 버퍼링 중지, 수집된 스냅샷 수 반환 | 출력: `count` |
| `ProcessSearchData` | SyncAction | 수집된 ToF 데이터로부터 목표 pose 계산 (현재 processing 로직 미구현 — 현재 TCP pose를 그대로 반환하는 stub) | 출력: `output_pose` |

### Condition 노드

| 노드 | 설명 | 입력 포트 |
|------|------|----------|
| `IsForceAbove` | Fingertip force가 threshold 초과 여부 확인 (500Hz 사전 계산 활용, sustained 판정 지원) | `threshold_N`(1.5), `min_fingertips`(2), `sustained_ms`(0) |
| `IsGrasped` | 물체 파지 상태 확인 (500Hz 사전 계산된 grasp_detected 활용) | `force_threshold_N`(1.0), `min_fingertips`(2) |
| `IsObjectDetected` | Vision 결과 수신 여부 확인 | 출력: `pose` |
| `IsGraspPhase` | Force-PI grasp phase 상태 확인 (GraspState.grasp_phase 비교) | `expected_phase` |
| `IsVisionTargetReady` | Vision target 데이터 유효성 확인 (최신 데이터 존재 여부) | — |
| `CheckShapeType` | ShapeEstimate 결과에서 shape 타입 추출 및 비교 | `expected_type`, 출력: `shape_type` |

## 설정 파일

### ROS2 파라미터 (`config/bt_coordinator.yaml`)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `tree_file` | `"pick_and_place.xml"` (코드 기본값), `"pick_and_place_force_pi.xml"` (배포 `config/bt_coordinator.yaml`이 덮어씀) | BT XML 파일명 (`trees/` 디렉토리 기준, 절대 경로도 지원) |
| `arm_group` | `"ur5e"` | Arm device group — 토픽 네임스페이스 세그먼트 (`/rtc_cm/<arm_group>/joint_states`, `<ns>/<arm_group>/joint_goal`). 빈 문자열이면 `on_configure` FAILURE (`//` 잘못된 토픽 방지) |
| `hand_group` | `"hand"` | Hand device group — 토픽 네임스페이스 세그먼트 (`/rtc_cm/<hand_group>/joint_states`, `<ns>/<hand_group>/{joint_goal,grasp_state,wbc_state}`). 예: `p1b`. 빈 문자열이면 `on_configure` FAILURE |
| `arm_dof` | `6` | Arm 관절 폭 (arm pose 길이 검증 + 패딩). DoF 는 컴파일타임 상수가 아닌 이 파라미터가 SSoT. `<= 0`이면 `on_configure` FAILURE |
| `hand_dof` | `10` | Hand 관절 폭 (hand pose 길이 검증 + 패딩). assm_v1·proto_1b 모두 10. `<= 0`이면 `on_configure` FAILURE |
| `has_grasp_sensing` | `true` | Grasp-force 센싱 노드군 (`GraspControl`/`IsForceAbove`/`IsGraspPhase`/`IsGrasped`) 등록 여부 (Seam D) |
| `has_tof` | `true` | ToF 근접센싱 노드군 (`StartToFCollection`/`StopToFCollection`/`ProcessSearchData`) 등록 여부 (Seam D) |
| `has_shape` | `true` | Shape estimation 노드군 (`TriggerShapeEstimation`/`WaitShapeResult`/`CheckShapeType`) 등록 여부 (Seam D) |
| `tick_rate_hz` | `80.0` | BT tick 주기 [Hz] |
| `repeat` | `false` | `true`면 트리 SUCCESS 완료 후 자동 반복 (FAILURE 시 정지) |
| `repeat_delay_s` | `1.0` | 반복 시 재시작 전 대기 시간 [s] |
| `paused` | `false` | `true`면 BT tick 일시 정지 |
| `step_mode` | `false` | `true`면 자동 tick 비활성, `~/step` 서비스로 수동 tick |
| `groot2_port` | `0` | Groot2 ZMQ 포트 (0 = 비활성, 1667 = Groot2 기본 포트) |
| `watchdog_timeout_s` | `2.0` | 토픽 수신 타임아웃 [s] (이 시간 동안 메시지 없으면 경고) |
| `watchdog_interval_s` | `5.0` | 헬스 체크 주기 [s] (0 = 비활성) |

반복 모드에서 트리 재시작 시 `object_pose` blackboard 변수가 자동으로 초기화된다.

`has_*` capability 를 `false` 로 두면 해당 센서를 읽는 BT 노드가 factory 에 등록되지 않는다. 그 노드를 참조하는 트리는 로드 시 BehaviorTree.CPP 의 "unknown node" 에러로 실패하며, `on_configure` 가 이를 잡아 깨끗한 FAILURE 로 전환한다 (센서 없는 로봇이 데이터 없는 노드를 tick 하는 것을 설정 시점에 차단).

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
```

## 로깅 (Logging)

### 분류 독트린

| 레벨 | 용도 | 예시 |
|------|------|------|
| `FATAL` | 프로세스를 계속 실행할 수 없는 상태 | 트리 파일 없음, 치명적 초기화 실패 |
| `ERROR` | 복구 불가능한 실패, 사용자 개입 필요 | 필수 포트 누락, 컨트롤러 전환 실패, BT leaf 실패 (FailureLogger) |
| `WARN` | 복구 가능한 실패/이상 상태, 자동 재시도 중 | 액션 타임아웃, 토픽 stale, gimbal lock 근접 |
| `INFO` | 사용자가 알아야 할 1 Hz 미만 상태 전환 | 액션 시작/완료/halted, 트리 로드, 컨트롤러 전환 개시 |
| `DEBUG` | 개발자 진단용 (기본 꺼짐), 20 Hz까지 허용 | 틱 추적, 진행률, watchdog healthy 핑 |

**핵심 규칙**:

- `INFO`/`WARN`/`ERROR`는 **20 Hz 핫패스에서 직접 호출 금지**. 폴링 안에서 반복될 수 있는 메시지는 `*_THROTTLE` 매크로 사용.
- `DEBUG`는 20 Hz 루프 안에서도 사용 가능. 단, 전용 서브-로거 이름으로 격리되어 기본 꺼져 있어야 한다.
- 노드 내부 실패 로그에는 `"FAILURE:"` 접두사를 붙이지 않는다. 실패 사실은 `FailureLogger`가 자동으로 `[BT FAIL]` 라인을 찍어주므로, 노드는 실패 *원인의 진단 정보만* 남긴다 (예: `timeout 10.0s pos_err=0.012`).
- 메시지 본문에 노드 이름을 수동으로 박아넣지 않는다. 서브-로거 이름이 곧 식별자다 (`bt.action.move_to_pose`).
- THROTTLE 주기는 매직넘버 대신 `bt_logging.hpp`의 표준 상수를 사용한다.

### 서브-로거 네임스페이스

모든 ROS2 로그는 단일 `bt` 로거가 아닌 계층적 서브-로거를 사용한다. 이로써 런타임에 부분 필터링이 가능하다.

| 서브-로거 | 사용처 |
|-----------|--------|
| `bt.coord` | `BtCoordinatorNode` (틱 루프, 트리 로드/전환, blackboard, step 모드) |
| `bt.bridge` | `BtRosBridge` (퍼블리셔/서브스크라이버, 게인 캐시, shape 서비스) |
| `bt.fail` | `FailureLogger` (모든 BT 노드의 FAILURE 전환) |
| `bt.watchdog` | 토픽 헬스 워치독 |
| `bt.poses` | 포즈 라이브러리 로드 |
| `bt.action.<snake_case>` | Action 노드 (`bt.action.move_to_pose`, `bt.action.grasp_control`, ...) |
| `bt.cond.<snake_case>` | Condition 노드 (`bt.cond.is_grasped`, `bt.cond.is_force_above`, ...) |

### THROTTLE 주기 표준

`rtc_bt::logging` 네임스페이스에 정의된 상수만 사용한다 (`bt_logging.hpp`):

| 상수 | 값 [ms] | 용도 |
|------|---------|------|
| `kThrottleFastMs` | 500 | 고빈도 폴링 노드의 진행 상태 표시 |
| `kThrottleSlowMs` | 2000 | 일반 반복 경고 (vision target stale 등) |
| `kThrottleIdleMs` | 10000 | 장기 유휴 상태 (watchdog, paused 핑 등) |

### 실시간 필터링 예시

```bash
# 특정 액션 노드만 DEBUG 활성화
ros2 service call /bt_coordinator/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'bt.action.move_to_pose', level: 10}]}"

# 모든 액션 노드 DEBUG (계층 매칭)
ros2 service call /bt_coordinator/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'bt.action', level: 10}]}"

# 워치독만 끄기
ros2 service call /bt_coordinator/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'bt.watchdog', level: 50}]}"
```

콘솔 출력에 로거 이름을 표시하려면 환경변수 설정:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
```

### FailureLogger 동작

트리 실행 중 어떤 노드가 `FAILURE` 상태로 전이되면 **실시간으로** 다음과 같은
로그가 `bt.fail` 서브-로거의 `RCLCPP_ERROR` 레벨로 출력된다:

```
[BT FAIL] <node_name> (type=<RegistrationName> uid=<N>) <prev_status> -> FAILURE
```

이는 `BT::StatusChangeLogger` 를 상속한 `FailureLogger` 클래스가 트리 로드 시
모든 노드에 자동 구독되어 동작한다. 틱 주기 사이에도 즉시 출력되므로, 트리가
멈춘 시점에 어느 노드가 먼저 실패했는지 곧바로 확인할 수 있다.

추가로 트리가 최종 `FAILURE` 로 종료되면 `LogFailureDiagnosis()` 가 트리
전체를 순회하며 실패 경로를 들여쓰기 포맷으로 덤프한다. 가장 안쪽의 실패
노드에는 `[FAIL-LEAF]` 태그가 붙어 원인 노드를 빠르게 찾을 수 있다:

```
──── Failure Diagnosis ────
[FAIL] main (type=Sequence uid=1)
  [FAIL] <anon> (type=SubTree uid=25)
    [FAIL-LEAF] <anon> (type=IsGraspPhase uid=42)
──── End Diagnosis (1 leaf failures) ────
```

개별 Action/Condition 노드는 실패 시 진단에 필요한 수치(타임아웃, 위치/힘
오차, 현재 phase 등)를 `RCLCPP_WARN` 또는 `RCLCPP_ERROR` 로 함께 출력한다.
이때 메시지에는 `"FAILURE:"` 접두사를 붙이지 않으며, 실패 사실은 `FailureLogger`
가 자동으로 표시한다. 폴링 노드(RetryUntilSuccessful 안에서 동작)의 경우
로그 플러딩을 막기 위해 `kThrottleFastMs` 또는 `kThrottleSlowMs` 주기로
throttle 된다.

### Blackboard 변수 (`bb.*` 파라미터)

YAML의 `bb.<key>` 형식으로 선언하면 트리 로드 후 Blackboard에 자동 주입된다.
타입은 YAML 값에서 자동 추론 (string, double, int, bool).

**Pick and Place 공용 (`pick_and_place*.xml`):**
- `bb.place_pose`: 물체를 놓을 목표 pose (형식: `"x;y;z;roll;pitch;yaw"`)
- `bb.object_final_z` (double, 기본 `.nan`): Phase 5 (`SlowDescend`)에서 object
  final goal의 Z를 이 절대값으로 덮어씀. `.nan`이면 비전 감지 Z를 그대로 사용
  (pass-through). 테이블 표면이 알려진 경우처럼, 고정 Z로 최종 접근하고 싶을 때
  실제 값(예: `0.085`)을 지정. `SetPoseZ` 노드가 `SlowDescend` 내부에서 Z만
  교체하며 X, Y, 방향은 감지된 object pose를 그대로 유지한다. Phase 4
  (ApproachFromAbove)와 Phase 7 (LiftAndVerify)는 원본 `{object_pose}` 기준으로
  동작하므로 영향이 없다.
- `bb.grasp_target_force_N` (기본 `1.5`): 주 검출 / Force-PI 목표력 [N] (contact_stop, force_pi 공유)
- `bb.grasp_verify_force_N` (기본 `0.8`): 닫힘 후 / 리프트 후 검증 floor [N]
- `bb.grasp_sustained_ms` (기본 `200`): `IsForceAbove` sustained 윈도우 [ms]
- `bb.grasp_min_fingertips` (기본 `2`): 최소 접촉 손가락 수
- `bb.grasp_abort_retreat_z` (기본 `0.05`): 실패 시 EmergencyAbort 후퇴 거리 [m] (`0`=skip)
- `bb.grasp_in_transit_min_N` (기본 `0.5`): transport/lower 가드 임계 [N] (verify 보다 느슨)
- `bb.grasp_max_attempts` (기본 `2`): ForcePIGrasp retry 횟수 (`1`=비활성화, force_pi only)
- `bb.grasp_retry_settle_s` (기본 `0.3`): retry release 후 sensor settling [s]
- `bb.tcp_rpy_offset_r_deg` / `bb.tcp_rpy_offset_p_deg` / `bb.tcp_rpy_offset_y_deg` (기본 `0.0`):
  task controller 전환 직후 현재 pose 기준 roll/pitch/yaw offset [deg]
- `bb.tcp_rotation_mode` (기본 `"add"`): offset 적용 방식 — `"add"` (단순 산술합),
  `"quat_body"` (TCP 프레임 회전, intrinsic), `"quat_world"` (World 프레임 회전, extrinsic)

**Pose-based grasp (`pick_and_place.xml` 전용):**
- `bb.hand_close_pose` (string, 기본 `"hand_close_medium"`): grip 강도별 hand 포즈 이름.
  `"hand_close_soft"`, `"hand_close_medium"`, `"hand_close_hard"` 중 선택.
  Launch arg `grip:=soft/medium/hard`로 설정 가능. 향후 vision topic 기반
  `ResolveGripFromVision` 노드가 blackboard에서 덮어쓸 수 있도록 설계됨.
- `bb.hand_close_settle_s` (double, 기본 `0.5`): hand close 후 안정 대기 시간 [s]

**Shape Inspect 공용 (`shape_inspect.xml` / `shape_inspect_simple.xml`):**
- `bb.inspect_offset_x`, `bb.inspect_offset_y` (기본 `0.0`): object_pose 기준 inspect 위치 x/y 오프셋 [m]
- `bb.inspect_approach_z` (기본 `0.15`): inspect_pose 위 접근/후퇴 클리어런스 [m]

**Shape Inspect Simple (`shape_inspect_simple.xml` 전용):**
- `bb.inspect_constant_z` (기본 `0.06`): inspect 위치 고정 Z [m] (비전 Z 노이즈 회피용, `object_pose.z` 대체)
- `bb.search_offset_x` (기본 `-0.1`): -x 방향 search 이동 거리 [m]
- `bb.search_speed` (기본 `0.02`): search 시 trajectory speed [m/s]

**Towel Unfold (`towel_unfold.xml`):**
- `bb.sweep_direction_x`, `bb.sweep_direction_y`: sweep 방향 벡터
- `bb.sweep_distance`: sweep 거리 [m]

### Hand/UR5e 포즈 설정

포즈는 두 곳에서 정의할 수 있다:

1. **컴파일타임 기본값** (`hand_pose_config.hpp`) — 코드 내 `kHandPoses`, `kUR5ePoses` 맵
2. **런타임 오버라이드** (`config/poses.yaml`) — 재컴파일 없이 포즈 튜닝 가능

`poses.yaml`에서 `hand_pose.<이름>` / `arm_pose.<이름>` 형식으로 선언하면 컴파일타임 기본값을 덮어쓴다.
값은 **도(deg) 단위**로 작성하고, 로드 시 자동으로 radian 변환된다.

```yaml
# 예: 엄지-검지 opposition 포즈 조정
hand_pose.thumb_index_oppose: [15.0, 45.0, 35.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0]
arm_pose.demo_pose: [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
```

Variant 별 포즈 파일 (`poses.yaml` / `poses_p1b.yaml`) 선택과 pose-name 계약은
[configuration.md](docs/configuration.md) Robot variant 절 참고. proto_1b 는 핑거 분할이
달라(thumb4/index3/middle2/ring1) 같은 포즈 이름이라도 joint 순서가 다르다.

#### 컴파일타임 포즈 (`hand_pose_config.hpp`)

포즈 값은 **도(°) 단위**로 작성하고, `DegToRad()` 래퍼로 rad 변환된다. `HandPose`/`ArmPose` 는
`std::vector<double>` (DoF 폭 런타임화, Seam B) 이므로 초기화 리스트 길이가 곧 DoF 다:

```cpp
{"my_pose", DegToRad(HandPose{30.0, 60.0, 45.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0})},
```

`kHandPoses` 맵에 정의된 명명 포즈 (기본 hand, 10-DoF):

| 포즈 이름 | 용도 |
|-----------|------|
| `home` | 기본 포즈 (전체 0°) |
| `full_flex` | 전체 손가락 flexion |
| `thumb_index_oppose` / `index_oppose` | 엄지-검지 opposition |
| `thumb_middle_oppose` / `middle_oppose` | 엄지-중지 opposition |
| `thumb_ring_oppose` / `ring_oppose` | 엄지-약지 opposition |
| `thumb_flex` / `index_flex` / `middle_flex` / `ring_flex` | FlexExtendFinger용 flex 타겟 (전체 손가락) |
| `thumb_mcp_flex` / `index_dip_flex` / `middle_dip_flex` | FlexExtendFinger용 flex 타겟 (단일 관절) |
| `hand_open` | Pose-based grasp 완전 개방 (home과 동일, 의미적 구분) |
| `hand_close_soft` | Pose-based grasp 소프트 (~40% full_flex) |
| `hand_close_medium` | Pose-based grasp 미디엄 (~70% full_flex) |
| `hand_close_hard` | Pose-based grasp 하드 (~100% full_flex) |

`kUR5ePoses` 맵에 정의된 UR5e 포즈 (6-DoF, 컴파일타임):

| 포즈 이름 | 용도 |
|-----------|------|
| `home_pose` | 기본 자세 (전체 0°) |
| `demo_pose` | 데모 자세 (0, -90, 90, -90, -90, 0°) |

`poses.yaml`에 정의된 UR5e 런타임 포즈 (6-DoF):

| 포즈 이름 | 용도 |
|-----------|------|
| `ready` | 작업 준비 자세 (팔을 세운 상태에서 약간 앞으로) |
| `table_top` | 테이블 위 작업 자세 |
| `front_reach` | 전방 수평 도달 |
| `side_reach` | 측면 도달 (왼쪽) |
| `handover` | 핸드오버 자세 (사람에게 물체 전달) |
| `stow` | 컴팩트 수납 자세 (로봇 몸쪽으로 접기) |
| `look_up` | 상향 관찰 (end-effector가 위를 향함) |
| `look_down` | 하향 관찰 (end-effector가 아래를 향함) |
| `pick_ready` | 픽업 대기 (테이블 위 물체 집기 직전) |
| `elevated` | 높은 위치 (물체를 들어올린 상태) |

손가락-관절 인덱스 매핑 (Seam C — `FingerResolver` 전략, 가변 DoF):

finger→joint index 는 상수가 아니라 `BtRosBridge::GetFingerJointIndices(key)` 가
런타임에 구성하며, 전략은 두 가지다 ([finger_resolver.hpp](include/ur5e_bt_coordinator/finger_resolver.hpp)):

- **`PrefixFingerResolver`** (기본) — `/rtc_cm/<hand_group>/joint_states` 의 name 을
  `<key>_` 로 접두사 매칭. 손가락별 DoF 가 달라도 (예: proto_1b thumb:4/index:3/middle:2/ring:1)
  joint 이름만 맞으면 코드 변경 없이 동작한다. key 는 whole-finger(`thumb`) 또는 sub-group
  (`thumb_mcp`, `index_dip`) 모두 지원 — sub-group 은 `<finger>_<segment>_` 접두사로 매칭.
- **`ExplicitFingerResolver`** — joint 이름에 finger prefix 가 없는 hand (예: LEAP 숫자 이름
  `"0".."15"`) 용. `config` 에 `finger_map.<finger>: [idx...]` 를 하나라도 선언하면 자동 전환되며,
  map 이 authoritative 다 (sub-group 도 명시 필요). 선언 방법은 `config/bt_coordinator.yaml` 참조.
  index 는 `on_configure` 에서 `[0, hand_dof)` 범위 검증 — 범위 밖이면 FAILURE (런타임 OOB 대신
  설정 시점 fail-fast). 주의: `finger_map.*` 를 하나라도 선언하면 prefix 매칭이 전면 비활성화되므로,
  Explicit 로 전환 시 사용하는 모든 finger/sub-group key 를 빠짐없이 선언해야 한다.

assm_v1 hand (joint_states 순서 thumb:3/index:3/middle:3/ring:1) 예시:

| key | 매칭되는 joint 이름 | DoF | 인덱스 |
|------|------|-----|--------|
| `thumb` | thumb_cmc_aa, thumb_cmc_fe, thumb_mcp_fe | 3 | 0-2 |
| `thumb_mcp` | thumb_mcp_fe | 1 | 2 |
| `index` | index_mcp_aa, index_mcp_fe, index_dip_fe | 3 | 3-5 |
| `index_dip` | index_dip_fe | 1 | 5 |
| `middle` | middle_mcp_aa, middle_mcp_fe, middle_dip_fe | 3 | 6-8 |
| `middle_dip` | middle_dip_fe | 1 | 8 |
| `ring` | ring_mcp_fe | 1 | 9 |

## SetGains 노드 (ROS 2 parameter API)

`SetGains` BT 노드는 *active* 컨트롤러의 LifecycleNode (`/<config_key>`) 에 대해 `set_parameters_atomically`를 호출한다 (Phase A~E 마이그레이션, 2026-04-26). BT 입력 포트로 채워진 값들만 dispatch하며, 입력 키와 실제 parameter 이름의 매핑은 active controller에 따라 다르다 ([src/nodes/set_gains.cpp](src/nodes/set_gains.cpp) 참조).

| BT 입력 포트 | DemoJoint | DemoTask | DemoWbc |
|-------------|-----------|----------|---------|
| `trajectory_speed` | `robot_trajectory_speed` | `trajectory_speed` | `arm_trajectory_speed` |
| `trajectory_angular_speed` | — | `trajectory_angular_speed` | — |
| `hand_trajectory_speed` | `hand_trajectory_speed` | `hand_trajectory_speed` | `hand_trajectory_speed` |
| `kp_translation`, `kp_rotation`, `damping`, `null_kp`, `enable_null_space`, `control_6dof` | — | (CLIK 게인) | — |
| `grasp_contact_threshold`, `grasp_force_threshold`, `grasp_min_fingertips` | (Force-PI grasp) | (Force-PI grasp) | — |
| `se3_weight`, `force_weight`, `posture_weight`, `mpc_enable`, `riccati_gain_scale` | — | — | (TSID + MPC) |

`grasp_command` / `grasp_target_force` 입력 포트는 parameter가 아닌 srv 채널 (`/<active>/grasp_command`, `rtc_msgs/srv/GraspCommand`) 로 분기된다. one-shot transition은 state가 아니므로 parameter로 표현하지 않는다.

Read-only 파라미터 (`max_traj_velocity` / `max_traj_angular_velocity` / `hand_max_traj_velocity` / `*_max_traj_velocity`) 는 `ParameterDescriptor::read_only=true` 로 선언되어 BT에서 set 시 컨트롤러가 거절한다. 이들 값은 컨트롤러 YAML로만 설정 가능.

## 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS2 C++ 클라이언트 |
| `rclcpp_lifecycle` | `BtCoordinatorNode` (LifecycleNode) |
| `behaviortree_cpp` | BehaviorTree.CPP v4 (`ros-jazzy-behaviortree-cpp`) |
| `std_msgs` | Float64MultiArray, String, Bool |
| `std_srvs` | Trigger (step 모드 서비스) |
| `geometry_msgs` | Polygon (vision 인터페이스), Pose6D 변환 |
| `rtc_msgs` | GraspState, WbcState, RobotTarget, ToFSnapshot |
| `shape_estimation_msgs` | ShapeEstimate, ExploreShape (shape estimation 노드군) |
| `sensor_msgs` | JointState (`/rtc_cm/<group>/joint_states`) |
| `tf2` / `tf2_ros` | TCP 포즈 lookup (base → tool0_actual), 쿼터니언 → RPY 변환 |
| `eigen` | 선형대수 |
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

RT 컨트롤러와 시뮬레이터(또는 실제 로봇)가 먼저 실행되어 있어야 한다:

```bash
# 사전 실행: MuJoCo 시뮬레이션
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py
```

### Launch 파일 (권장)

```bash
# 기본 실행 (hand_motions.xml, YAML 설정 + 포즈 자동 로드)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py

# proto_1b hand variant (hand_group=p1b, poses_p1b.yaml)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py variant:=ur5e_p1b

# Pick and Place (pose-based grasp, 기본 medium grip)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place.xml

# Pick and Place (soft grip — 부드러운 물체용)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place.xml grip:=soft

# Pick and Place (hard grip — 단단한 물체용)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place.xml grip:=hard

# Pick and Place (force-based grasp — contact_stop / force_pi)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place_contact_stop.xml
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place_force_pi.xml

# Towel Unfold
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=towel_unfold.xml

# 반복 실행
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py tree:=pick_and_place.xml repeat:=true repeat_delay:=2.0

# Groot2 시각화 연결
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py groot2_port:=1667

# 일시정지 상태로 시작 (step 모드 디버깅용)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py paused:=true
```

**Launch arguments:**

| Argument | 기본값 | 설명 |
|----------|--------|------|
| `variant` | `ur5e_p1a` | Robot variant → config/poses 선택: `ur5e_p1a` / `ur5e_p1b` ([configuration.md](docs/configuration.md) Robot variant) |
| `tree` | (YAML 기본값) | BT tree XML 파일명 |
| `tick_rate` | 0 (=YAML 80Hz) | BT tick 주기 [Hz] |
| `repeat` | (YAML 기본값) | SUCCESS 시 자동 반복 |
| `repeat_delay` | 0 (=YAML 1.0s) | 반복 전 대기 시간 [s] |
| `paused` | (YAML 기본값) | 일시정지 상태로 시작 |
| `groot2_port` | 0 (비활성) | Groot2 ZMQ 포트 |
| `grip` | (YAML 기본값) | Pose-based grasp grip 강도: `soft`, `medium`, `hard` |

### 직접 실행 (ros2 run)

```bash
# YAML 설정 + 포즈 파일을 직접 지정
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/bt_coordinator.yaml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses.yaml

# Towel Unfold (Blackboard 변수 지정)
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=towel_unfold.xml \
  -p bb.sweep_direction_x:=1.0 \
  -p bb.sweep_direction_y:=0.0 \
  -p bb.sweep_distance:=0.3 \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses.yaml

# 오프라인 트리 검증 (ROS 실행 불필요)
ros2 run ur5e_bt_coordinator validate_tree pick_and_place.xml
```

## 파일 구조

```
ur5e_bt_coordinator/
├── config/
│   ├── bt_coordinator.yaml          # ROS2 파라미터 (트리, tick rate, 런타임 제어, bb.*)
│   ├── bt_coordinator_p1b.yaml      # ur5e_p1b variant delta (hand_group=p1b)
│   ├── poses.yaml                   # Hand/UR5e 포즈 오버라이드 (deg 단위, 재컴파일 불필요)
│   └── poses_p1b.yaml               # proto_1b hand 포즈만 (poses.yaml 위 override; arm 은 공용)
├── launch/
│   └── bt_coordinator.launch.py     # Launch 파일 (YAML + poses 자동 로드, launch arg 지원)
├── trees/
│   ├── common_motions.xml           # 재사용 가능 공통 모션 SubTree
│   ├── pick_and_place.xml           # Pose-based grasp 물체 파지 (grip:=soft/medium/hard)
│   ├── pick_and_place_contact_stop.xml  # Force-based grasp (contact_stop)
│   ├── pick_and_place_force_pi.xml      # Force-PI adaptive grasp (retry 지원)
│   ├── towel_unfold.xml             # 수건 펼치기 시나리오
│   ├── hand_motions.xml             # Hand 민첩성 데모 시나리오
│   ├── vision_approach.xml          # Vision approach 데모 (arm-only)
│   ├── shape_inspect.xml            # ToF shape estimation 워크플로우
│   ├── shape_inspect_simple.xml    # Vision + linear ToF search 간소화 inspection
│   └── search_motion.xml            # 팔 sweep + tilt scan 탐색
├── include/ur5e_bt_coordinator/
│   ├── bt_types.hpp                 # Pose6D, CachedGraspState/CachedWbcState (contact_flag 은 sensor-A native prob 또는 sensor-B derived binary; derived binary 는 컨트롤러 grasp_force_threshold 를 내장하므로 IsGrasped/IsForceAbove 의 custom-threshold recount 는 force_magnitude 단독으로 판정 — floor 밑 BT 임계값도 도달 가능), BT 타입 변환
│   ├── bt_utils.hpp                 # 유틸리티 함수 (시간, map lookup, CSV 파싱, partial hand update)
│   ├── bt_ros_bridge.hpp            # ROS topic ↔ BT bridge 헤더
│   ├── bt_coordinator_node.hpp      # 메인 노드 헤더
│   ├── hand_pose_config.hpp         # Hand/UR5e 포즈 lookup map, 손가락-관절 인덱스 매핑
│   ├── bt_node_registration.hpp     # BT 노드 factory 등록 헤더 (capability 기반)
│   ├── action_nodes/                # Action 노드 헤더 (§BT 노드 참조)
│   └── condition_nodes/             # Condition 노드 헤더 (§BT 노드 참조)
└── src/
    ├── main.cpp                     # 진입점
    ├── bt_coordinator_node.cpp      # 노드 초기화, BT tick 루프, 런타임 트리 전환
    ├── bt_ros_bridge.cpp            # Topic 구독/발행, 포즈 라이브러리, 토픽 헬스 모니터링
    ├── bt_node_registration.cpp     # BT 노드 factory 등록 (capability 기반, §BT 노드 참조)
    ├── validate_tree.cpp            # 오프라인 트리 XML 검증 도구
    └── nodes/                       # 노드 구현체 (§BT 노드 참조)
```

노드/헤더 개수는 `include/ur5e_bt_coordinator/{action_nodes,condition_nodes}/` 및
`src/nodes/`를 직접 확인하거나 위 [BT 노드](#bt-노드) 절 표를 참조 (등록된 노드 목록의
단일 출처는 [src/bt_node_registration.cpp](src/bt_node_registration.cpp)).

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
