# BT 노드 레퍼런스

30개 커스텀 BT 노드의 포트 및 동작 문서.

---

## 데이터 포맷

BT Blackboard에서 사용하는 문자열 포맷:

| 타입 | 포맷 | 예시 |
|------|------|------|
| `Pose6D` | `"x;y;z;roll;pitch;yaw"` | `"0.3;-0.3;0.15;3.14;0.0;0.0"` |
| `vector<double>` | `"v0;v1;v2;..."` | `"0.0;0.0;0.0;0.0;0.0;0.0"` |
| `vector<Pose6D>` | 파이프(`\|`)로 구분된 Pose6D | `"0.1;0.2;0.3;0;0;0\|0.4;0.5;0.6;0;0;0"` |

---

## Action 노드

### Arm Motion

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

#### UR5eHoldPose

UR5e 팔을 목표 자세로 이동 후, halt될 때까지 영구 RUNNING. 반드시 `Parallel`과 함께 사용.

```xml
<Parallel success_count="1" failure_count="1">
  <UR5eHoldPose pose="demo_pose"/>
  <Sequence><!-- Hand 동작 --></Sequence>
</Parallel>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `pose` | string | (필수) | 명명된 UR5e 포즈 (예: `"demo_pose"`) |

### Controller / Gains

#### SwitchController

활성 컨트롤러를 전환한다 (`/rtc_cm/switch_controller` srv 동기 호출).

```xml
<SwitchController controller_name="demo_task_controller"
                  timeout_s="3.0"/>
```

| 포트 | 방향 | 타입 | 기본값 | 설명 |
|------|------|------|--------|------|
| `controller_name` | input | string | (필수) | 전환할 컨트롤러 이름 |
| `timeout_s` | input | double | 3.0 s | srv 응답 타임아웃 |

#### SetGains

active 컨트롤러 LifecycleNode의 ROS 2 parameter를 atomic 변경 (`set_parameters_atomically`). 설정하지 않은 입력 포트는 dispatch에서 제외되어 현재 값 유지.

```xml
<SetGains trajectory_speed="0.05"/>

<SetGains kp_translation="5.0,5.0,5.0"
          kp_rotation="3.0,3.0,3.0"
          trajectory_speed="0.08"/>

<SetGains grasp_command="1" grasp_target_force="2.5"/>
```

> **Note:** `*_max_traj_velocity` 시리즈는 `ParameterDescriptor::read_only=true`로 선언되어 BT가 set 시 컨트롤러가 거절. YAML로만 설정 가능.

| 포트 | 타입 | 설명 |
|------|------|------|
| `kp_translation` | string | 위치 비례 게인 `"kx,ky,kz"` (DemoTask) |
| `kp_rotation` | string | 자세 비례 게인 `"kx,ky,kz"` (DemoTask) |
| `damping` | double | 감쇠 계수 (DemoTask) |
| `null_kp` | double | Null-space 비례 게인 (DemoTask) |
| `enable_null_space` | bool | Null-space 활성화 (DemoTask) |
| `control_6dof` | bool | 6DOF 제어 활성화 (DemoTask) |
| `trajectory_speed` | double | 궤적 속도 — active controller에 따라 `robot_trajectory_speed` (DemoJoint) / `trajectory_speed` (DemoTask) / `arm_trajectory_speed` (DemoWbc) 로 dispatch |
| `trajectory_angular_speed` | double | 궤적 각속도 (DemoTask) |
| `hand_trajectory_speed` | double | 손 궤적 속도 (모든 demo) |
| `grasp_contact_threshold`, `grasp_force_threshold`, `grasp_min_fingertips` | float / int | Force-PI grasp 감지 임계값 (DemoJoint, DemoTask) |
| `se3_weight`, `force_weight`, `posture_weight` | double | TSID task 가중치 (DemoWbc) |
| `mpc_enable` | bool | MPC output gate (DemoWbc, 빌드타임 mpc_enabled_와 AND) |
| `riccati_gain_scale` | double | Riccati 피드백 스케일 [0,1] (DemoWbc) |
| `grasp_command` | int | Force-PI one-shot transition (0=none/skip, 1=grasp, 2=release) — srv `grasp_command`로 분기 |
| `grasp_target_force` | double | grasp 목표 힘 [N] (`grasp_command=1` 일 때만) |

### Computation

#### ComputeOffsetPose

입력 포즈에 6-DoF 오프셋을 적용. Translation은 항상 가산, Rotation은 3가지 모드 지원.

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

### Hand Control

#### GraspControl

손(Hand)의 파지 동작을 제어한다. 4가지 모드 지원.

```xml
<GraspControl mode="close" close_speed="0.3" max_position="1.4" timeout_s="10.0"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `mode` | string | (필수) | `"open"` / `"close"` / `"pinch"` / `"preset"` |
| `target_positions` | vector\<double\> | - | 10개 모터 목표 위치 [rad] |
| `close_speed` | double | 0.3 rad/s | close/pinch 모드에서 증가 속도 |
| `max_position` | double | 1.4 rad | close/pinch 모드 최대 위치 |
| `pinch_motors` | string | "0,1,2,3" | pinch 모드에서 사용할 모터 인덱스 |
| `timeout_s` | double | 8.0 s | 타임아웃 |

#### SetHandPose

Hand 전체 10-DoF를 명명된 포즈로 이동한다.

```xml
<SetHandPose pose="home" hand_trajectory_speed="1.0"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `pose` | string | (필수) | 명명된 Hand 포즈 (예: `"home"`, `"full_flex"`) |
| `hand_trajectory_speed` | double | 1.0 rad/s | RT 컨트롤러 trajectory speed |

#### MoveFinger

특정 손가락을 명명된 포즈로 이동한다.

```xml
<MoveFinger finger_name="thumb" pose="thumb_index_oppose" hand_trajectory_speed="1.0"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `finger_name` | string | (필수) | 손가락 이름 (`"thumb"` / `"index"` / `"middle"` / `"ring"` 등) |
| `pose` | string | (필수) | 명명된 타겟 포즈 |
| `hand_trajectory_speed` | double | 1.0 rad/s | RT 컨트롤러 trajectory speed |

#### FlexExtendFinger

특정 손가락의 flex → extend 1회 cycle을 수행한다.

```xml
<FlexExtendFinger finger_name="index" hand_trajectory_speed="1.0"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `finger_name` | string | (필수) | 손가락 이름 |
| `hand_trajectory_speed` | double | 1.0 rad/s | RT 컨트롤러 trajectory speed |

#### MoveOpposition

엄지 + 대상 손가락 opposition 동작. 비-target 손가락은 자동으로 home 리셋.

```xml
<MoveOpposition thumb_pose="thumb_index_oppose"
                target_finger="index"
                target_pose="index_oppose"
                hand_trajectory_speed="1.0"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `thumb_pose` | string | (필수) | 엄지 포즈 이름 |
| `target_finger` | string | (필수) | 대상 손가락 (`"index"` / `"middle"` / `"ring"`) |
| `target_pose` | string | (필수) | 대상 손가락 포즈 이름 |
| `hand_trajectory_speed` | double | 1.0 rad/s | RT 컨트롤러 trajectory speed |

### Shape Estimation / ToF

#### TriggerShapeEstimation

Shape estimation 파이프라인을 제어한다. `"start"` 시 자동으로 `/shape/clear` 호출 후 `/shape/trigger` 발행.

```xml
<TriggerShapeEstimation command="start"/>
```

| 포트 | 타입 | 설명 |
|------|------|------|
| `command` | string | `"start"` / `"stop"` / `"pause"` / `"resume"` / `"single"` |

#### WaitShapeResult

Shape estimation 결과가 confidence threshold를 넘을 때까지 대기.

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

## Condition 노드

#### IsForceAbove

핑거팁 힘이 임계값을 초과하는지 검사한다.

```xml
<IsForceAbove threshold_N="1.5" min_fingertips="2" sustained_ms="200"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `threshold_N` | double | 1.5 N | 힘 임계값 |
| `min_fingertips` | int | 2 | 최소 감지 핑거팁 수 |
| `sustained_ms` | int | 0 ms | 유지 시간 (히스테리시스) |

#### IsGrasped

힘 + 접촉 상태를 검사하여 파지 성공 여부를 판단한다.

```xml
<IsGrasped force_threshold_N="1.0" min_fingertips="2"/>
```

| 포트 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `force_threshold_N` | double | 1.0 N | 힘 임계값 |
| `min_fingertips` | int | 2 | 최소 감지 핑거팁 수 |

#### IsObjectDetected / IsVisionTargetReady

`/world_target_info`에서 물체 감지를 확인한다. Position은 vision에서, orientation은 TCP에서 가져온다.

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
| `phase` | string | "holding" | FSM 단계 (`idle`/`approaching`/`contact`/`force_control`/`holding`/`releasing`) |

#### CheckShapeType

ShapeEstimate에서 shape type/name/confidence를 추출한다.

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
