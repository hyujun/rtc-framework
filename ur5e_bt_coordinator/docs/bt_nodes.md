# BT 노드 레퍼런스

등록된 BT 노드의 입력 포트와 기본값이다. 등록 목록의 단일 출처는 [src/bt_node_registration.cpp](../src/bt_node_registration.cpp), 포트 선언의 단일 출처는 각 노드 헤더([action_nodes/](../include/ur5e_bt_coordinator/action_nodes) · [condition_nodes/](../include/ur5e_bt_coordinator/condition_nodes))다. 괄호 안 숫자는 포트 기본값이다. `has_*` capability 가 false 이면 해당 노드군은 등록되지 않는다 ([configuration.md](configuration.md)).

## Blackboard 문자열 포맷

Blackboard 문자열은 `BT::convertFromString` 특수화로 파싱한다 ([bt_types.hpp](../include/ur5e_bt_coordinator/bt_types.hpp)).

| 타입 | 포맷 | 예시 |
|------|------|------|
| `Pose6D` | `"x;y;z;roll;pitch;yaw"` | `"0.3;-0.3;0.15;3.14;0.0;0.0"` |
| `vector<double>` | `"v0;v1;v2;..."` | `"0.0;0.0;0.0;0.0;0.0;0.0"` |
| `vector<Pose6D>` | 파이프(`\|`)로 구분된 Pose6D | `"0.1;0.2;0.3;0;0;0\|0.4;0.5;0.6;0;0;0"` |

## Action 노드

| 노드 | 타입 | 설명 | 입력 포트 |
|------|------|------|----------|
| `MoveToPose` | StatefulAction | Task-space 6D 목표 이동, position/orientation tolerance 도달 판정 | `target`, `position_tolerance`(0.005), `orientation_tolerance`(0.05), `timeout_s`(10.0) |
| `MoveToJoints` | StatefulAction | Joint-space 목표 이동, per-joint tolerance 도달 판정 | `target`, `pose_name`(poses.yaml named arm pose, target 대체), `tolerance`(0.01), `timeout_s`(10.0) |
| `GraspControl` | StatefulAction | Hand open/close/pinch/preset 제어, 점진적 닫기 지원 | `mode`(close), `target_positions`, `close_speed`(0.3), `max_position`(1.4), `pinch_fingers`("thumb,index"), `timeout_s`(8.0) |
| `TrackTrajectory` | StatefulAction | Waypoint 시퀀스 순차 추적 (sweep motion 등) | `waypoints`, `position_tolerance`(0.01), `timeout_s`(30.0) |
| `SetGains` | StatefulAction | active controller 의 ROS 2 parameter 를 동적으로 바꾼다. 채워진 포트만 dispatch — 매핑은 아래 [SetGains](#setgains) | `trajectory_speed`, `trajectory_angular_speed`, `hand_trajectory_speed`, `kp_translation`, `kp_rotation`, `singularity_threshold`, `max_damping`, `null_kp`, `enable_null_space`, `control_6dof`, `grasp_contact_threshold`, `grasp_force_threshold`, `grasp_min_fingertips`, `se3_weight`, `force_weight`, `posture_weight`, `mpc_enable`, `riccati_gain_scale`, `grasp_command`, `grasp_target_force` |
| `SwitchController` | StatefulAction | 활성 컨트롤러 전환 (joint ↔ task). 서비스 요청을 보내고 tick 마다 결과를 폴링한다 | `controller_name`, `timeout_s`(3.0) |
| `ComputeOffsetPose` | SyncAction | Pose에 XYZ offset 적용 (approach, lift, retreat 계산) | `input_pose`, `offset_x`(0.0), `offset_y`(0.0), `offset_z`(0.0) → 출력: `output_pose` |
| `SetPoseZ` | SyncAction | Pose의 Z좌표를 절대값으로 덮어씀 (X, Y, 방향 유지). `z`가 NaN(기본값)이면 pass-through. Object final goal의 Z를 고정하는 용도 | `input_pose`, `z`(NaN) → 출력: `output_pose` |
| `ComputeSweepTrajectory` | SyncAction | Arc sweep 경로 waypoint 생성 (towel unfold용, sinusoidal arc 프로파일) | `start_pose`, `direction_x`(1.0), `direction_y`(0.0), `distance`(0.3), `arc_height`(0.05), `num_waypoints`(8) → 출력: `waypoints` |
| `ComputeTiltSequence` | SyncAction | Roll/pitch를 번갈아 오실레이션하는 tilt-scan waypoint 시퀀스 생성 (position 고정, ExplorationMotionGenerator tilt phase 모사) | `base_pose`, `amplitude_deg`(15.0), `num_steps`(6) → 출력: `waypoints` |
| `GetCurrentPose` | SyncAction | ROS bridge에서 현재 TCP pose를 읽어 Blackboard에 기록 (sweep/tilt 시퀀스 시작점 캡처용) | 출력: `pose` |
| `WaitDuration` | StatefulAction | 지정 시간 대기 | `duration_s`(0.5) |
| `MoveFinger` | StatefulAction | 특정 손가락을 명명된 포즈로 이동 (trajectory duration 추정 + 수렴 게이트, partial hand update) | `finger_name`, `pose`, `hand_trajectory_speed`(1.0), `tolerance`(0.15), `timeout_s`(10.0) |
| `FlexExtendFinger` | StatefulAction | 손가락 flex→extend 1 cycle (2-phase, phase별 trajectory duration 추정 + 수렴 게이트) | `finger_name`, `hand_trajectory_speed`(1.0), `tolerance`(0.15), `timeout_s`(10.0) |
| `SetHandPose` | StatefulAction | 전체 Hand 10-DoF를 명명된 포즈로 이동 (trajectory duration 추정 + 수렴 게이트) | `pose`, `hand_trajectory_speed`(1.0), `tolerance`(0.15), `timeout_s`(10.0) |
| `UR5eHoldPose` | StatefulAction | UR5e 목표 자세 도달 후 영구 RUNNING (halt까지 유지) | `pose` |
| `MoveOpposition` | StatefulAction | Opposition 동작 (thumb+target 포즈, 비-target home 리셋, trajectory duration 추정 + 수렴 게이트) | `thumb_pose`, `target_finger`, `target_pose`, `hand_trajectory_speed`(1.0), `tolerance`(0.15), `timeout_s`(10.0) |
| `TriggerShapeEstimation` | SyncAction | Shape estimation 시작/정지 제어 (서비스 호출) | `command`(start; start\|stop\|pause\|resume\|single) |
| `WaitShapeResult` | StatefulAction | Shape estimation 결과 대기 (confidence 임계값 도달까지) | `confidence_threshold`(0.7), `timeout_s`(10.0), 출력: `estimate` |
| `StartToFCollection` | SyncAction | `<ns>/tof/snapshot` 메시지 버퍼링 시작 (기존 데이터 초기화) | — |
| `StopToFCollection` | SyncAction | ToF 버퍼링 중지, 수집된 스냅샷 수 반환 | 출력: `count` |
| `ProcessSearchData` | SyncAction | 수집된 ToF 데이터로 목표 pose 를 계산한다. 처리 로직은 아직 없고 현재 TCP pose 를 그대로 반환한다 | 출력: `output_pose` |

## Condition 노드

| 노드 | 설명 | 입력 포트 |
|------|------|----------|
| `IsForceAbove` | Fingertip force가 threshold 초과 여부 확인 (500Hz 사전 계산 활용, sustained 판정 지원) | `threshold_N`(1.5), `min_fingertips`(2), `sustained_ms`(0) |
| `IsGrasped` | 물체 파지 상태 확인 (500Hz 사전 계산된 grasp_detected 활용) | `force_threshold_N`(1.0), `min_fingertips`(2) |
| `IsObjectDetected` | Vision 결과 수신 여부 확인 | 출력: `pose` |
| `IsGraspPhase` | Force-PI grasp phase 상태 확인 (GraspState.grasp_phase 비교) | `phase`(holding) |
| `IsVisionTargetReady` | Vision target 데이터 유효성 확인 (최신 데이터 존재 여부) | 출력: `pose` |
| `CheckShapeType` | ShapeEstimate 결과에서 shape 타입 추출 및 비교 | `estimate`, `expected_type`(생략 시 추출만), 출력: `shape_type`/`shape_name`/`confidence` |

## Common Motions SubTree 포트

| SubTree | 주요 Input 포트 | Output 포트 |
|---------|----------------|-------------|
| `DetectObject` | `num_attempts`, `wait_s` | `pose` |
| `ApproachFromAbove` | `target_pose`, `offset_z`, `pos_tol`, `ori_tol`, `timeout_s` | `approach_pose` |
| `SlowDescend` | `target_pose`, `traj_speed`, `max_traj_vel`, `pos_tol`, `ori_tol`, `timeout_s` | - |
| `ForceGrasp` | `grasp_mode`, `close_speed`, `max_position`, `threshold_N`, `min_fingertips`, `sustained_ms`, `verify_force_N`, `verify_min_tips` | - |
| `LiftAndVerify` | `base_pose`, `offset_z`, `traj_speed`, `verify_force_N`, `verify_min_tips` | `lift_pose` |
| `ReleaseAndRetreat` | `base_pose`, `retreat_z`, `retreat_speed`, `timeout_s` | `retreat_pose` |
| `*EmergencyAbort` | `retreat_z` | - |


## SetGains

`SetGains` 는 *active* 컨트롤러의 LifecycleNode (`/<active_ctrl>/<active_ctrl>`) 에 `set_parameters_atomically` 를 호출한다. 포트 이름과 실제 parameter 이름의 매핑은 컨트롤러마다 다르다 ([src/nodes/set_gains.cpp](../src/nodes/set_gains.cpp)).

| BT 입력 포트 | DemoJoint | DemoTask | DemoWbc |
|-------------|-----------|----------|---------|
| `trajectory_speed` | `robot_trajectory_speed` | `trajectory_speed` | `arm_trajectory_speed` |
| `trajectory_angular_speed` | — | `trajectory_angular_speed` | — |
| `hand_trajectory_speed` | `hand_trajectory_speed` | `hand_trajectory_speed` | `hand_trajectory_speed` |
| `kp_translation`, `kp_rotation`, `singularity_threshold`, `max_damping`, `null_kp`, `enable_null_space`, `control_6dof` | — | (CLIK 게인) | — |
| `grasp_contact_threshold`, `grasp_force_threshold`, `grasp_min_fingertips` | (Force-PI grasp) | (Force-PI grasp) | — |
| `se3_weight`, `force_weight`, `posture_weight`, `mpc_enable`, `riccati_gain_scale` | — | — | (TSID + MPC) |

- `grasp_command` / `grasp_target_force` 는 parameter 가 아니라 서비스(`/<active>/grasp_command`, `rtc_msgs/srv/GraspCommand`)로 보낸다. parameter 커밋이 성공한 뒤에만 전송한다 — 거절된 게인 위에서 grasp 하지 않는다.
- `*_max_traj_velocity` 류 read-only 파라미터는 컨트롤러가 거절한다. 컨트롤러 YAML 로만 설정한다.

## 손가락 이름 해석

`MoveFinger` · `FlexExtendFinger` · `MoveOpposition` 의 finger key 는 `/rtc_cm/<hand_group>/joint_states` 의 joint 이름으로 해석한다 ([finger_resolver.hpp](../include/ur5e_bt_coordinator/finger_resolver.hpp)).

- **Prefix** (기본) — key 를 `<key>_` 접두사로 매칭한다. whole-finger (`thumb`) 와 sub-group (`thumb_mcp`, `index_dip`) 모두 된다. 손가락별 DoF 가 달라도 joint 이름만 맞으면 동작한다.
- **Explicit** — joint 이름에 finger prefix 가 없는 hand 용. `finger_map.<finger>: [idx...]` 를 하나라도 선언하면 자동 전환되고, 쓰는 모든 finger/sub-group key 를 빠짐없이 선언해야 한다. index 가 `[0, hand_dof)` 밖이면 `on_configure` 가 FAILURE 로 끝난다.
