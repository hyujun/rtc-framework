# ur5e_bt_coordinator

UR5e + Hand 시스템용 BehaviorTree.CPP v4 task coordinator. **유형**: 노드 패키지 (non-RT)

## 개요

`bt_coordinator` LifecycleNode 는 XML 로 정의한 BT 트리를 로드해 80 Hz 로 tick 한다. 500 Hz RT 제어 루프 밖에서 돌고, RT 컨트롤러와는 토픽 · 서비스 · parameter 로만 통신하므로 컨트롤러 코드를 고치지 않는다. E-STOP 이 활성이면 tick 을 멈추고 해제되면 재개한다. 통신 상대는 active 컨트롤러와 서비스를 제공하는 [rtc_controller_manager](../rtc_controller_manager/README.md), 그리고 비전 노드와 [shape_estimation](../shape_estimation/README.md) 이다.

```
bt_coordinator (non-RT, 80 Hz)
  ├─ pub  <ns>/<arm_group>/joint_goal, <ns>/<hand_group>/joint_goal  ─► active 컨트롤러 (500 Hz)
  ├─ sub  /rtc_cm/<group>/joint_states, <ns>/<hand_group>/{grasp_state,wbc_state}
  ├─ tf2  base → (virtual_tcp_actual | tool0_actual)   ◄─ <ns>/transforms
  ├─ sub  /world_target_info (vision), /system/estop_status
  └─ srv  /rtc_cm/switch_controller, <ns> parameter API, /shape/clear
```

`<ns>` 는 active 컨트롤러 namespace 이고 `/rtc_cm/active_controller_name` 이 바뀔 때마다 pub/sub 를 다시 묶는다. `<arm_group>` · `<hand_group>` 은 같은 이름의 파라미터 값이다.

## 빠른 시작

먼저 RT 컨트롤러와 sim(또는 실제 로봇)이 떠 있어야 한다. 빌드는 [루트 README](../README.md#빠른-시작) 를 따른다.

```bash
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py          # 사전 실행
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py       # YAML 기본 트리
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py \
  tree:=pick_and_place.xml grip:=soft repeat:=true repeat_delay:=2.0
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py variant:=ur5e_p1b
ros2 run ur5e_bt_coordinator validate_tree pick_and_place.xml   # 오프라인 검증
```

- 기본 트리는 `config/bt_coordinator.yaml` 의 `tree_file` 이고 값은 `pick_and_place_force_pi.xml` 이다 (force grasp 컨트롤러 필요). `launch` 의 `tree` 인자를 비우면 이 값을 쓴다.
- `validate_tree` 의 종료 코드는 0 = 유효, 1 = 무효, 2 = 파일 없음이다.
- `ros2 run` 으로 직접 띄우는 법, 사전 실행 노드, 반복 실행, 런타임 제어: [docs/configuration.md](docs/configuration.md).

## 인터페이스

### 실행파일

| 실행파일 | 역할 |
|---|---|
| `bt_coordinator_node` | BT 코디네이터 LifecycleNode (`/bt_coordinator`) |
| `validate_tree` | ROS 없이 트리 XML 의 구문 · 포트 정합성을 검사한다 |

### Launch 인자

[launch/bt_coordinator.launch.py](launch/bt_coordinator.launch.py) 가 `config/bt_coordinator.yaml` + `poses.yaml` 과 variant delta 두 개를 순서대로 로드한다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `variant` | `ur5e_p1a` | 로봇 variant. `ur5e_p1a` (assm_v1 hand) · `ur5e_p1b` (proto_1b hand) |
| `tree` | (YAML) | BT tree XML 파일명 |
| `tick_rate` | `0.0` (=YAML 80 Hz) | BT tick 주기 [Hz] |
| `repeat` | (YAML) | SUCCESS 시 자동 반복 |
| `repeat_delay` | `0.0` (=YAML 1.0 s) | 반복 전 대기 [s] |
| `paused` | (YAML) | 일시정지 상태로 시작 |
| `groot2_port` | `0` (비활성) | Groot2 ZMQ 포트 (기본 포트 1667) |
| `grip` | (YAML) | pose-based grasp 강도: `soft` · `medium` · `hard` |

### 발행

| Topic | 타입 | 설명 |
|---|---|---|
| `<ns>/<arm_group>/joint_goal` | `rtc_msgs/RobotTarget` | arm task-space 또는 joint-space 목표 |
| `<ns>/<hand_group>/joint_goal` | `rtc_msgs/RobotTarget` | hand 모터 목표 |
| `/shape/trigger` | `std_msgs/String` | shape estimation 시작 · 정지 트리거 |

게인 변경 · 컨트롤러 전환 · grasp 명령은 토픽이 아니라 아래 서비스 · parameter API 로 보낸다.

### 구독

QoS 는 별도 표기가 없으면 RELIABLE, depth 1 이다. BT coordinator 는 RT 전용 BEST_EFFORT 토픽을 구독하지 않는다 (예외는 `<ns>/tof/snapshot`).

| Topic | 타입 | 설명 |
|---|---|---|
| `/rtc_cm/<arm_group>/joint_states` | `sensor_msgs/JointState` | arm 관절 위치 (고정 경로, 컨트롤러와 무관) |
| `/rtc_cm/<hand_group>/joint_states` | `sensor_msgs/JointState` | hand 관절 위치 (고정 경로) |
| `<ns>/<hand_group>/grasp_state` | `rtc_msgs/GraspState` | 500 Hz 사전 계산 grasp 상태 (Force-PI grasp 컨트롤러 전용) |
| `<ns>/<hand_group>/wbc_state` | `rtc_msgs/WbcState` | WBC FSM phase + 핑거팁 + TSID 진단 (WBC 컨트롤러 전용) |
| `<ns>/tof/snapshot` | `rtc_msgs/ToFSnapshot` | ToF + 핑거팁 pose. BEST_EFFORT, depth 100 |
| `<ns>/transforms` | `tf2_msgs/TFMessage` | active 컨트롤러 FK 의 `base → tool0_actual` · `virtual_tcp_actual` |
| `/world_target_info` | `geometry_msgs/Polygon` | 비전 물체 위치 (`points[0]` = x,y,z, orientation 없음). frame 은 `world_target_frame` |
| `/rtc_cm/active_controller_name` | `std_msgs/String` | 현재 활성 컨트롤러 (TRANSIENT_LOCAL). rewire 트리거 |
| `/system/estop_status` | `std_msgs/Bool` | E-STOP 상태 (transient_local) |
| `/shape/estimate` | `shape_estimation_msgs/ShapeEstimate` | shape estimation 결과 |

`world_target_frame` 은 비우면 `base_frame` 이다. 다른 값이면 기동 때 ERROR 를 내고 이 토픽을 무시한다.

### 서비스 client

| 서비스 | 타입 | 설명 |
|---|---|---|
| `/rtc_cm/switch_controller` | `rtc_msgs/srv/SwitchController` | 활성 컨트롤러 전환 (`SwitchController` 노드) |
| `/rtc_cm/list_controllers` | `rtc_msgs/srv/ListControllers` | 컨트롤러 목록 조회 (bridge 가 client 를 갖지만 호출하는 BT 노드는 없다) |
| `/<active_ctrl>/<active_ctrl>/set_parameters_atomically` | parameter API | active 컨트롤러 게인 설정 (`SetGains`) |
| `/<active_ctrl>/grasp_command` | `rtc_msgs/srv/GraspCommand` | Force-PI one-shot grasp · release (`SetGains`) |
| `/shape/clear` | `std_srvs/srv/Trigger` | shape estimation 누적 데이터 초기화 (`TriggerShapeEstimation` start 시 자동 호출) |

노드 자신이 제공하는 서비스는 `~/step` (`std_srvs/srv/Trigger`, step 모드) 하나다.

### TCP pose 와 task frame

TCP pose 는 토픽이 아니라 `tf2_ros::Buffer` lookup 으로 읽는다: `base` → active 컨트롤러의 task frame. 컨트롤러가 `<ns>/transforms` 로만 발행하므로 bridge 가 그 `TFMessage` 를 `tf_buffer_` 에 직접 넣는다 (외부 `/tf` 재발행 불필요).

- child frame 은 고정이 아니다. `virtual_tcp_mode` 컨트롤러는 `virtual_tcp_actual` 을 발행하고, 아니면 `tool0_actual` 이다. 선택은 컨트롤러 이름이 아니라 어느 frame 이 관측되는지로만 한다.
- rewire 직후의 캐시는 *이전* 컨트롤러의 pose 다. `MoveToPose` · `TrackTrajectory` 는 pose 가 live 가 될 때까지 수렴 판정을 보류하고 RUNNING 을 유지한다 — 안 그러면 움직이지 않은 동작이 SUCCESS 로 보고된다.
- 트리의 `MoveToPose` target 은 `{blackboard_var}` 이고, 값은 `GetCurrentPose → ComputeOffsetPose` 상대 체인이나 `{object_pose}` 에서 온다. 둘 다 active 컨트롤러 frame 기준이라 절대 waypoint 를 변환할 필요가 없다.

### BT 트리

| 트리 | 파일 | 설명 | 실패 처리 |
|------|------|------|----------|
| Common Motions | `trees/common_motions.xml` | 재사용 가능한 공통 모션 SubTree 라이브러리 (아래 표) | - |
| Pick and Place | `trees/pick_and_place.xml` | Pose-based grasp: vision 감지 → approach → SetHandPose 기반 grip (soft/medium/hard) → lift → transport → release. Grasp controller 미사용 | `PoseBasedEmergencyAbort` |
| Pick and Place (Contact Stop) | `trees/pick_and_place_contact_stop.xml` | Force-based grasp (contact_stop): vision 감지 → approach → force-based grasp → lift → transport → release | `ContactStopEmergencyAbort` |
| Pick and Place (Force-PI) | `trees/pick_and_place_force_pi.xml` | Force-PI adaptive grasp: vision 감지 → approach → force-PI grasp (retry 지원) → lift → transport → release | `ForcePIEmergencyAbort` |
| Towel Unfold | `trees/towel_unfold.xml` | 수건 edge 감지 → pinch pre-shape → approach → pinch grasp → lift → compliant sweep → lower/release → retreat | `ReleaseAndRetreat` (정상 종료 경로 재사용) |
| Hand Motions | `trees/hand_motions.xml` | UR5e 자세 유지 + Hand 데모 (OppositionDemo → WaveDemo) | - |
| Vision Approach | `trees/vision_approach.xml` | Vision 기반 approach 데모 (arm-only, 핸드 미사용) | 없음 (단순 데모) |
| Shape Inspect | `trees/shape_inspect.xml` | ToF 센서 기반 shape estimation 워크플로우 (start → wait → stop → evaluate) | 없음 |
| Shape Inspect Simple | `trees/shape_inspect_simple.xml` | Vision 기반 inspect 위치 이동 → -x 방향 linear search move + ToF 500Hz 데이터 수집 → 데이터 처리 → 목표 이동. 서비스 기반 shape estimation 미사용 | 없음 |
| Search Motion | `trees/search_motion.xml` | 팔 sweep + tilt scan 탐색 모션 | 없음 (단순 데모) |

### Common Motions SubTree

`<include path="common_motions.xml"/>` 로 참조한다. 포트는 [docs/bt_nodes.md](docs/bt_nodes.md#common-motions-subtree-포트).

| SubTree | 설명 | 사용 트리 |
|---------|------|----------|
| `DetectObject` | 재시도 기반 물체 감지 | 모든 pick 트리, towel_unfold |
| `ApproachFromAbove` | 목표 위 오프셋으로 접근 | 모든 pick 트리, towel_unfold |
| `SlowDescend` | 저속 게인 설정 후 목표로 이동 (optional Z override) | 모든 pick 트리, towel_unfold |
| `ForceGrasp` | 병렬 GraspControl close + 힘 감지 + 파지 검증 | pick_and_place_contact_stop |
| `LiftAndVerify` | 리프트 + 파지 유지 검증 | contact_stop, force_pi, towel_unfold |
| `ReleaseAndRetreat` | GraspControl open + 상방 후퇴 | contact_stop, towel_unfold |
| `ContactStopOpen` | GraspControl open + settle | contact_stop |
| `ContactStopEmergencyAbort` | ContactStopOpen + retreat (실패 시 안전 종착) | contact_stop |
| `ForcePIGrasp` | Force-PI FSM 기반 adaptive grasp | force_pi |
| `ForcePIRelease` | Force-PI FSM release (grasp_command=2) | force_pi |
| `ForcePIGraspWithRetry` | ForcePIGrasp retry wrapper (hand-only 재시도) | force_pi |
| `ForcePIGraspDiagnose` | 실패 시 FSM phase 진단 로깅 | force_pi |
| `ForcePIEmergencyAbort` | ForcePIRelease + retreat (실패 시 안전 종착) | force_pi |
| `PoseBasedEmergencyAbort` | SetHandPose open + retreat (실패 시 안전 종착) | pick_and_place (pose-based) |

**SubTree 에 숫자를 넘길 때 (BehaviorTree.CPP 4.10+)**: `<SubTree ... timeout_s="10.0"/>` 처럼 리터럴로 넘긴 숫자는 4.10 부터 subtree blackboard 에 `int` (소수점 없음) 또는 `double` 로 저장된다 (4.9 까지는 `string`).

- 그 키를 읽는 강타입 포트는 타입이 정확히 같아야 트리가 만들어진다. `double` 포트에는 `"10.0"`, `int` 포트에는 `"10"` 처럼 모양을 맞춘다.
- `unsigned` 포트 (`Timeout.msec` · `Delay.delay_msec` · `Sleep.msec`) 로 가는 인자는 `common_motions.xml` 의 `ForcePIGrasp` · `ForcePIRelease` 처럼 `Script` 로 새 키에 옮겨 잇는다.
- 이 오류는 트리를 실제로 만들 때만 난다. 트리 파일을 추가하면 [test/test_tree_validation.cpp](test/test_tree_validation.cpp) 에 그 파일을 인스턴스화하는 테스트도 추가한다.

### 커스텀 트리 작성

`trees/*.xml` 에 파일을 추가하면 재빌드 없이 install 에 포함된다. 기본 골격:

```xml
<root BTCPP_format="4" main_tree_to_execute="MyTask">
  <include path="common_motions.xml"/>
  <BehaviorTree ID="MyTask">
    <Sequence name="main"><!-- 노드 배치 --></Sequence>
  </BehaviorTree>
</root>
```

| 변경 | 수정 파일 | 재컴파일 |
|---|---|---|
| 포즈 값 튜닝 · 새 포즈 (런타임) | `config/poses*.yaml` | 불필요 |
| 새 BT 트리 · 공통 SubTree | `trees/*.xml` | 불필요 |
| 컴파일타임 기본 포즈 · 새 관절 그룹 | `hand_pose_config.hpp` | 필요 |

### BT 노드

28개 노드가 등록된다. 포트와 기본값은 [docs/bt_nodes.md](docs/bt_nodes.md).

- Action: `MoveToPose` · `MoveToJoints` · `GraspControl` · `TrackTrajectory` · `SetGains` · `SwitchController` · `ComputeOffsetPose` · `SetPoseZ` · `ComputeSweepTrajectory` · `ComputeTiltSequence` · `GetCurrentPose` · `WaitDuration` · `MoveFinger` · `FlexExtendFinger` · `SetHandPose` · `UR5eHoldPose` · `MoveOpposition` · `TriggerShapeEstimation` · `WaitShapeResult` · `StartToFCollection` · `StopToFCollection` · `ProcessSearchData`
- Condition: `IsForceAbove` · `IsGrasped` · `IsObjectDetected` · `IsGraspPhase` · `IsVisionTargetReady` · `CheckShapeType`

등록 목록의 단일 출처는 [src/bt_node_registration.cpp](src/bt_node_registration.cpp) 다.

## 설정

설정은 `config/` 의 YAML 이 갖는다. launch 는 base 두 개(`bt_coordinator.yaml` · `poses.yaml`) 위에 variant delta 두 개(`bt_coordinator_<p1a|p1b>.yaml` · `poses_<p1a|p1b>.yaml`)를 얹는다.

| 키 | 기본 | 의미 |
|---|---|---|
| `tree_file` | `pick_and_place_force_pi.xml` | 로드할 트리 (`trees/` 기준, 절대 경로 가능) |
| `tick_rate_hz` | `80.0` | BT tick 주기 [Hz]. 코드 declare 기본은 20.0 이라 YAML 없이 띄우면 20 Hz 다 |
| `arm_group` · `hand_group` | `ur5e` · (variant 가 공급) | 토픽 세그먼트. 비어 있으면 `on_configure` 가 FAILURE |
| `arm_dof` · `hand_dof` | `6` · `10` | pose 길이 검증 폭 |
| `has_grasp_sensing` · `has_tof` · `has_shape` | `true` | 센서 노드군 등록 여부. `ur5e_p1b` 는 `has_tof` · `has_shape` 가 false |
| `repeat` · `repeat_delay_s` | `false` · `1.0` | SUCCESS 시 자동 반복 (FAILURE 면 정지) |
| `paused` · `step_mode` | `false` | 일시정지 · 수동 tick (`~/step`) |
| `groot2_port` | `0` | Groot2 ZMQ 포트 |
| `bb.*` | — | 트리에 주입되는 Blackboard 변수 |

전체 파라미터 표, `bb.*` 변수, 포즈 파일, variant 계약(포즈 이름 집합 · capability)은 [docs/configuration.md](docs/configuration.md) 가 갖는다.

런타임 제어:

```bash
ros2 param set /bt_coordinator paused true                     # 일시정지 (false 로 재개)
ros2 param set /bt_coordinator tree_file "towel_unfold.xml"    # 트리 핫스왑
ros2 param set /bt_coordinator step_mode true                  # 수동 tick
ros2 service call /bt_coordinator/step std_srvs/srv/Trigger    # 한 tick 진행
```

### 로깅 (Logging)

레벨 분류 · 로거 네이밍 공통 규칙은 [conventions.md](../agent_docs/conventions.md#logging) 를 따른다. 이 패키지 고유 규칙:

- tick 핫패스에서 `INFO`/`WARN`/`ERROR` 를 직접 부르지 않는다. 폴링 안에서 반복될 수 있는 메시지는 `*_THROTTLE` 을 쓰고, 주기 상수는 [bt_logging.hpp](include/ur5e_bt_coordinator/bt_logging.hpp) 의 `kThrottle*Ms` 만 쓴다.
- 노드의 실패 로그에 `"FAILURE:"` 접두사를 붙이지 않는다. 실패 사실은 `FailureLogger` 가 `[BT FAIL]` 로 찍으므로 노드는 원인 진단 값만 남긴다.
- 메시지 본문에 노드 이름을 박지 않는다. 서브-로거 이름이 식별자다.

서브-로거는 `bt.coord` · `bt.bridge` · `bt.fail` · `bt.watchdog` · `bt.poses` · `bt.action.<snake_case>` · `bt.cond.<snake_case>` 다. `ros2 run ... --ros-args --log-level bt.action.move_to_pose:=debug` 처럼 이름 단위로 레벨을 정한다 (계층 매칭이라 `bt.action` 만 주면 전체 action 노드). 콘솔에 로거 이름을 보이려면:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
```

트리 실행 중 노드가 FAILURE 로 바뀌면 `FailureLogger` 가 `bt.fail` 에 즉시 `[BT FAIL] <node_name> (type=<Registration> uid=<N>) <prev> -> FAILURE` 를 ERROR 로 찍는다. 트리가 최종 FAILURE 면 실패 경로를 들여쓰기로 덤프하고 가장 안쪽 노드에 `[FAIL-LEAF]` 를 붙인다.

```
──── Failure Diagnosis ────
[FAIL] main (type=Sequence uid=1)
  [FAIL] <anon> (type=SubTree uid=25)
    [FAIL-LEAF] <anon> (type=IsGraspPhase uid=42)
──── End Diagnosis (1 leaf failures) ────
```

## 디렉토리

```
ur5e_bt_coordinator/
├── config/    base · variant YAML, poses YAML
├── docs/      configuration.md, bt_nodes.md
├── include/   공개 헤더 (action_nodes, condition_nodes)
├── launch/    bt_coordinator.launch.py
├── src/       노드 구현 (nodes/)
├── test/      gtest (tier 1/2/3)
└── trees/     BT XML (common_motions.xml 포함)
```

테스트 suite 목록의 단일 출처는 [CMakeLists.txt](CMakeLists.txt) 다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| `[Watchdog] ...: no messages received yet` | RT 컨트롤러 미실행 | `sim_ur5e_p1a.launch.py` 또는 `robot_ur5e_p1a.launch.py` 를 먼저 띄운다 |
| 트리가 즉시 FAILURE, `[FAILED] IsObjectDetected` | 비전 미감지 | `/world_target_info` 발행을 확인한다 |
| `[FAILED] MoveToPose` (timeout) | 목표 도달 실패 | tolerance 를 완화하거나 게인을 조정한다 |
| `[FAILED] IsForceAbove`, 파지 타임아웃 | 힘 임계가 높거나 센서 미연결 | `threshold_N` 조정, `/<active_ctrl>/<hand_group>/grasp_state` 확인 |
| 팔이 움직이지 않는다 | RT 컨트롤러 미실행 또는 E-STOP | 컨트롤러 상태와 E-STOP 을 확인한다 |
| 게인 변경이 반영되지 않는다 | parameter 서비스 미준비, 또는 read-only 파라미터에 set | `ros2 param list /<active_ctrl>` 로 확인. 거절 사유는 `SetGains` 응답 message |
| 트리가 FAILURE 로 종료 | 시퀀스 중 한 노드 실패 | `bt.fail` 로그의 Failure Diagnosis 로 실패 노드를 찾거나 Groot2 로 추적한다 |
| `Tree file not found` | 경로 오류 | 절대 경로를 쓰거나 `trees/` 를 확인한다 |
| tick 이 멈추고 재개 안 됨 | E-STOP 활성 (`reason = "E-STOP active"`) | 원인 확인 후 해제하면 자동 재개된다 |
| `Tick skipped: Hand joint_states not received yet` | finger 트리가 hand joint_states 도착 전에 tick (동시 launch 시 startup race) | joint_states 가 흐르면 자동 재개된다. 계속되면 hand 드라이버와 `/rtc_cm/<hand_group>/joint_states` 를 확인한다 |
| `Tick aborted by exception: ...` | 노드가 tick 중 throw (필수 입력 누락, 해석 불가한 finger key) | 프로세스는 죽지 않는다. `no hand joints for finger '<name>'` 이면 트리의 finger 이름과 joint 이름 접두사(또는 `finger_map.*`)가 어긋난 것이다 |

디버깅:

```bash
ros2 topic echo /rtc_cm/active_controller_name        # 활성 컨트롤러
ros2 run tf2_ros tf2_echo base tool0_actual           # TCP pose (tf2 lookup)
ros2 param get /bt_coordinator tree_file
```

## 더 읽기

- [docs/configuration.md](docs/configuration.md) — 실행 · 런타임 제어 · 파라미터 · `bb.*` · 포즈 · variant 계약
- [docs/bt_nodes.md](docs/bt_nodes.md) — BT 노드 포트 · SetGains 매핑 · 손가락 이름 해석
- [conventions.md](../agent_docs/conventions.md#logging) — 로깅 공통 규칙

## 의존성

[package.xml](package.xml) — `behaviortree_cpp` (`ros-jazzy-behaviortree-cpp`) 는 사전 설치가 필요하다. MIT License ([LICENSE](../LICENSE)).
