# 설정 및 실행 가이드

BehaviorTree 기반 UR5e + Hand 비실시간 태스크 코디네이터.
500Hz RT 제어 루프 외부에서 80Hz로 동작하며, ROS2 토픽을 통해 RT Controller와 통신한다.

---

## 사전 요구사항

### 사전 실행 필요 노드

BT coordinator를 실행하기 전에 다음 노드들이 먼저 실행되어 있어야 한다:

1. **UR5e RT Controller** — `/rtc_cm/{<arm_group>,<hand_group>}/joint_states` (fixed path), `base → tool0_actual` tf2 transform, `/<active_ctrl>/<hand_group>/{grasp_state,wbc_state}` (ur5e_p1a: `/<active_ctrl>/p1a/...`), `/rtc_cm/active_controller_name`, `/<active_ctrl>/tof/snapshot` 토픽 발행
2. **Vision Node** (선택) — `/world_target_info` 토픽 발행 (물체 감지 사용 시)
3. **Shape Estimation Node** (선택) — `/shape/estimate` 토픽 발행, `/shape/trigger` 수신, `/shape/clear` 서비스 (shape_inspect 트리용)
4. **E-STOP Node** (선택) — `/system/estop_status` 토픽 발행

---

## 실행 방법

### Launch 파일 (권장)

```bash
# 기본 실행 (YAML 기본 트리, YAML + 포즈 자동 로드)
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py

# ── Robot variant (device group + poses 선택) ──
# base(bt_coordinator.yaml + poses.yaml) 위에 variant delta 를 얹음:
# ur5e_p1a (default): + bt_coordinator_p1a.yaml(hand_group=p1a) + poses_p1a.yaml
# ur5e_p1b: + bt_coordinator_p1b.yaml(hand_group=p1b) + poses_p1b.yaml
ros2 launch ur5e_bt_coordinator bt_coordinator.launch.py variant:=ur5e_p1b

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

Launch 인자 표는 [README 인터페이스](../README.md#launch-인자) 에 있다.

### ros2 run (직접 실행)

```bash
# 기본 실행 (ur5e_p1a: base + p1a delta 를 순서대로 직접 지정)
# launch 는 이 4개 params-file 로드를 자동화한다 (variant:=ur5e_p1a).
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/bt_coordinator.yaml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/bt_coordinator_p1a.yaml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses.yaml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses_p1a.yaml

# Pick and Place (pose-based, soft grip)
ros2 run ur5e_bt_coordinator bt_coordinator_node \
  --ros-args -p tree_file:=pick_and_place.xml \
  -p bb.hand_close_pose:=hand_close_soft \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/bt_coordinator.yaml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/bt_coordinator_p1a.yaml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses.yaml \
  --params-file $(ros2 pkg prefix ur5e_bt_coordinator)/share/ur5e_bt_coordinator/config/poses_p1a.yaml
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

값의 단일 출처는 [config/bt_coordinator.yaml](../config/bt_coordinator.yaml) 이다. `<variant>` delta 가 `hand_group` 을 공급한다 (아래 Robot variant).

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `tree_file` | `"pick_and_place_force_pi.xml"` | BT XML 파일명 (`trees/` 디렉토리 기준, 절대 경로도 지원) |
| `arm_group` | `"ur5e"` | Arm device group — 토픽 네임스페이스 세그먼트 (`/rtc_cm/<arm_group>/joint_states`, `<ns>/<arm_group>/joint_goal`). 빈 문자열이면 `on_configure` FAILURE (`//` 잘못된 토픽 방지) |
| `hand_group` | *(default 없음)* | Hand device group — 토픽 네임스페이스 세그먼트 (`/rtc_cm/<hand_group>/joint_states`, `<ns>/<hand_group>/{joint_goal,grasp_state,wbc_state}`). variant delta 가 공급 (p1a→`p1a`, p1b→`p1b`); 미공급(빈 문자열)이면 `on_configure` FAILURE — 존재하지 않는 legacy `hand` 그룹 silent 구독 방지 |
| `arm_dof` | `6` | Arm 관절 폭 (arm pose 길이 검증 + 패딩). DoF 는 컴파일타임 상수가 아닌 이 파라미터가 SSoT. `<= 0`이면 `on_configure` FAILURE |
| `hand_dof` | `10` | Hand 관절 폭 (hand pose 길이 검증 + 패딩). assm_v1·proto_1b 모두 10. `<= 0`이면 `on_configure` FAILURE |
| `base_frame` | `"base"` | 팔의 base frame = 컨트롤러의 `root_link` . TCP pose 를 읽는 TF 부모이자 모든 task 목표의 `header.frame_id`. 둘에 같은 값을 쓰므로 `root_link` 와 다르면 목표가 거부로 드러난다 |
| `world_target_frame` | `""` | `/world_target_info` 위치의 frame. 비우면 `base_frame`. 다른 값이면 ERROR 후 토픽 무시 |
| `has_grasp_sensing` | `true` | Grasp-force 센싱 노드군 (`GraspControl`/`IsForceAbove`/`IsGraspPhase`/`IsGrasped`) 등록 여부 (Seam D) |
| `has_tof` | `true` | ToF 근접센싱 노드군 (`StartToFCollection`/`StopToFCollection`/`ProcessSearchData`) 등록 여부 (Seam D) |
| `has_shape` | `true` | Shape estimation 노드군 (`TriggerShapeEstimation`/`WaitShapeResult`/`CheckShapeType`) 등록 여부 (Seam D) |
| `tick_rate_hz` | `80.0` | BT tick 주기 [Hz]  |
| `repeat` | `false` | `true`면 트리 SUCCESS 완료 후 자동 반복 (FAILURE 시 정지) |
| `repeat_delay_s` | `1.0` | 반복 시 재시작 전 대기 시간 [s] |
| `paused` | `false` | `true`면 BT tick 일시 정지 |
| `step_mode` | `false` | `true`면 자동 tick 비활성, `~/step` 서비스로 수동 tick |
| `groot2_port` | `0` | Groot2 ZMQ 포트 (0 = 비활성, 1667 = Groot2 기본 포트) |
| `watchdog_timeout_s` | `2.0` | 토픽 수신 타임아웃 [s] (이 시간 동안 메시지 없으면 경고) |
| `watchdog_interval_s` | `5.0` | 헬스 체크 주기 [s] (0 = 비활성) |
| `controller_wait_timeout_s` | `10.0` | 첫 active controller 대기가 이 시간을 넘기면 ERROR 1회 (`<= 0` = 미발생). 대기 자체는 무제한 — 진단 전용 |

`has_*` capability 를 `false` 로 두면 해당 센서를 읽는 BT 노드가 등록되지 않는다. 그 노드를 참조하는 트리는 로드 시 "unknown node" 에러로 실패하고, `on_configure` 가 이를 FAILURE 로 전환한다. 반복 모드에서 트리를 다시 시작하면 `object_pose` blackboard 변수가 초기화된다.

### Blackboard 변수 (`bb.*`)

YAML 의 `bb.<key>` 로 선언하면 트리 로드 후 Blackboard 에 주입된다. 타입은 YAML 값에서 추론한다 (string, double, int, bool). 런타임에는 `ros2 param set /bt_coordinator bb.<key> <값>` 으로 바꾼다. 기본값은 YAML 값이다.

| 키 | 기본 | 의미 |
|---|---|---|
| `bb.place_pose` | `"0.3;-0.3;0.15;3.14;0.0;0.0"` | 물체를 놓을 pose (`x;y;z;roll;pitch;yaw`) |
| `bb.object_final_z` | `0.06` | `SlowDescend` 의 최종 Z 를 이 절대값으로 덮어쓴다. `.nan` 이면 비전 Z 를 그대로 쓴다 (`SetPoseZ`) |
| `bb.grasp_target_force_N` | `1.5` | 주 검출 · Force-PI 목표력 [N] |
| `bb.grasp_verify_force_N` | `0.8` | 닫힘 후 · 리프트 후 검증 하한 [N] |
| `bb.grasp_sustained_ms` | `200` | `IsForceAbove` sustained 윈도우 [ms] |
| `bb.grasp_min_fingertips` | `2` | 최소 접촉 손가락 수 |
| `bb.grasp_abort_retreat_z` | `0.05` | 실패 시 EmergencyAbort 후퇴 거리 [m] (`0` = 생략) |
| `bb.grasp_in_transit_min_N` | `0.5` | transport · lower 가드 임계 [N] (verify 보다 느슨) |
| `bb.grasp_max_attempts` | `2` | ForcePIGrasp retry 횟수 (`1` = 비활성, force_pi 전용) |
| `bb.grasp_retry_settle_s` | `0.3` | retry release 후 센서 안정 대기 [s] |
| `bb.tcp_rpy_offset_{r,p,y}_deg` | `0.0` | task controller 전환 직후 현재 pose 기준 roll/pitch/yaw offset [deg] |
| `bb.tcp_rotation_mode` | `"add"` | offset 적용 방식: `"add"` (산술합) · `"quat_body"` (TCP 프레임) · `"quat_world"` (world 프레임) |
| `bb.hand_close_pose` | `"hand_close_medium"` | pose-based grasp 의 hand 포즈 (`hand_close_soft` / `_medium` / `_hard`). launch 인자 `grip` 이 설정한다 |
| `bb.hand_close_settle_s` | `0.5` | hand close 후 안정 대기 [s] |
| `bb.inspect_offset_x` · `_y` | `0.0` | shape_inspect: object_pose 기준 inspect 위치 offset [m] |
| `bb.inspect_approach_z` | `0.15` | shape_inspect: inspect 위치 위 접근 · 후퇴 여유 [m] |
| `bb.inspect_constant_z` | `0.06` | shape_inspect_simple: 고정 Z [m] (비전 Z 대신) |
| `bb.search_offset_x` | `-0.1` | shape_inspect_simple: -x 방향 search 거리 [m] |
| `bb.search_speed` | `0.02` | shape_inspect_simple: search trajectory speed [m/s] |
| `bb.sweep_direction_x` · `_y` · `bb.sweep_distance` | (주석 처리) | towel_unfold: sweep 방향 · 거리 [m] |

### `config/poses.yaml` (base, arm 전용) + `config/poses_<variant>.yaml` (hand)

포즈 이름의 컴파일타임 기본값은 [hand_pose_config.hpp](../include/ur5e_bt_coordinator/hand_pose_config.hpp) 의 `kHandPoses` · `kUR5ePoses` 이고, YAML 에 같은 이름을 선언하면 덮어쓴다. 새 이름은 YAML 에만 추가해도 되어 재컴파일이 필요 없다. 이름 목록은 [poses.yaml](../config/poses.yaml) · [poses_p1a.yaml](../config/poses_p1a.yaml) 이 갖는다.

포즈를 재컴파일 없이 튜닝할 수 있다. 값은 **도(deg) 단위**로 작성하고, 로드 시 자동으로 radian
변환된다. base `poses.yaml` 은 UR5e `arm_pose.*` 만 담고(두 variant 공용), `hand_pose.*` 는
variant delta(`poses_p1a.yaml`=assm_v1, `poses_p1b.yaml`=proto_1b)가 공급한다.

```yaml
# poses.yaml (base) — arm 전용
bt_coordinator:
  ros__parameters:
    arm_pose.demo_pose: [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    arm_pose.ready: [0.0, -90.0, 0.0, -90.0, -90.0, 0.0]
    arm_pose.table_top: [0.0, -70.0, 110.0, -130.0, -90.0, 0.0]
    # ... (추가 포즈: front_reach, side_reach, handover, stow,
    #      look_up, look_down, pick_ready, elevated, vision_ready)

# poses_p1a.yaml (delta) — hand 전용 (assm_v1 레이아웃)
bt_coordinator:
  ros__parameters:
    hand_pose.home: [0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0]
    hand_pose.thumb_index_oppose: [15.0, 45.0, 35.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0]
```

### Robot variant (`ur5e_p1a` / `ur5e_p1b`)

`variant` launch arg 로 device group + poses 파일을 통째로 선택한다. base(`bt_coordinator.yaml`
+ `poses.yaml`)는 항상 먼저 로드되어 공통 설정(bb.* / tick_rate / arm_pose.*)을 한 곳에서만
관리하고, 모든 variant 는 대칭적으로 delta 쌍(device group + hand poses)을 얹는다.

| variant | 로드되는 파일 (순서) | hand_group | hand 분할 | capabilities (`has_grasp_sensing`/`has_tof`/`has_shape`) |
|---------|--------------|-----------|-----------|-----------|
| `ur5e_p1a` (default) | base + `bt_coordinator_p1a.yaml` + `poses_p1a.yaml` | `p1a` | thumb3/index3/middle3/ring1 | true / true / true |
| `ur5e_p1b` | base + `bt_coordinator_p1b.yaml` + `poses_p1b.yaml` | `p1b` | thumb4/index3/middle2/ring1 | true / **false / false** (grasp-force 만; ToF/shape 하드웨어 없음) |

두 variant 모두 `poses.yaml`(arm)을 먼저 로드한 뒤 `poses_<variant>.yaml` 로 `hand_pose.*` 를
공급한다. 따라서 **arm_pose.\* 는 `poses.yaml` 한 곳에서만 관리**되고(UR5e 팔 공용), 각 delta
에는 hand 포즈만 둔다. base `poses.yaml` 에는 `hand_pose.*` 가 없다 — hand joint 레이아웃이
variant 마다 다르기 때문.

**Pose-name 계약**: `poses_p1a.yaml` 과 `poses_p1b.yaml` 은 **동일한 `hand_pose.*` 이름 집합**
을 선언해야 한다 (값은 레이아웃마다 다르되 이름은 같아야 트리가 두 variant 모두에서 해석된다).
base `poses.yaml` 은 `hand_pose.*` 를 정의해선 안 되고(variant 별 레이아웃 누출 방지), 두 delta
모두 `arm_pose.*` 를 재정의해선 안 된다(단일 소스 drift 방지). 이 세 조건을 `test_tree_validation`
의 `PoseNameParityP1aVsP1b` 가 강제한다. 이래야 트리를 **무수정 재사용**할 수 있다.

**Capability 계약**: `has_*` 가 false 인 센서의 노드군은 등록되지 않으므로, 그 노드를 참조하는
트리는 로드 시 실패한다(`on_configure` FAILURE). position-only variant 설계 시 사용. `ur5e_p1b`
는 grasp-force 만 갖고 ToF/shape 하드웨어가 없어(`_base.yaml` sensor_layout `values_per_group:0`;
shape estimation 은 ToF snapshot 이 유일 입력) `has_tof`/`has_shape` 를 false 로 둔다 — 따라서
p1b 에서 shape/ToF 트리(`shape_inspect*`)는 로드 시 깨끗한 FAILURE 가 된다.

런타임에도 변경 가능:
```bash
ros2 param set /bt_coordinator hand_pose.home "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```
