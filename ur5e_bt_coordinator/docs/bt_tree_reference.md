# BT 트리 레퍼런스

기본 제공 BT 트리 10종 + 커스텀 트리 작성 가이드.

---

## 기본 제공 트리

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

---

### Pick and Place — Pose-based (`pick_and_place.xml`)

**Grasp controller 미사용.** 미리 정의된 hand pose(soft/medium/hard)로 물체를 집는 가장 단순한 pick-and-place.

**필수 Blackboard 변수:**
- `place_pose` — 목표 배치 포즈 (예: `"0.3;-0.3;0.15;3.14;0.0;0.0"`)
- `hand_close_pose` — grip 포즈 이름 (기본: `"hand_close_medium"`, launch arg `grip:=` 로 설정)

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

---

### Pick and Place — Contact Stop (`pick_and_place_contact_stop.xml`)

**GraspControl force 기반 파지 (단일 시도, retry 없음).**

**필수 Blackboard 변수:**
- `place_pose` — 목표 배치 포즈
- `grasp_controller_type: "contact_stop"` in demo controller YAML

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

---

### Pick and Place — Force-PI (`pick_and_place_force_pi.xml`)

**Force-PI adaptive grasp (retry 지원).** 컨트롤러 내부 FSM (Approaching → Contact → ForceControl → Holding)으로 force control 수행.

**필수 Blackboard 변수:**
- `place_pose` — 목표 배치 포즈
- `grasp_controller_type: "force_pi"` in demo controller YAML

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

---

### Shape Inspect (`shape_inspect.xml`)

**ToF 기반 shape estimation 파이프라인.** Ready pose에서 vision 감지 → inspection 위치 접근 → shape estimation → shape type 추출.

**필수 Blackboard 변수:**
- `inspect_offset_x/y` — object_pose 기준 inspect 위치 오프셋 [m]
- `inspect_approach_z` — 접근/후퇴 clearance [m]

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

---

### Shape Inspect Simple (`shape_inspect_simple.xml`)

**Search move + 500 Hz ToF 데이터 수집.** Vision 기반 접근 후, -x 방향 선형 search를 수행하면서 `/tof/snapshot` 데이터를 ring buffer에 축적.

**필수 Blackboard 변수:**
- `inspect_offset_x/y` — object_pose 기준 x/y 오프셋 [m]
- `inspect_constant_z` — 고정 Z (비전 Z 대신 사용) [m]
- `inspect_approach_z` — 접근/후퇴 clearance [m]
- `search_offset_x` — -x 방향 search 거리 [m] (음수)
- `search_speed` — search 시 trajectory speed [m/s]

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

---

### Towel Unfold (`towel_unfold.xml`)

수건 가장자리를 핀치 파지하여 들어올린 후 호 형태로 스윕하여 펼치는 시퀀스.

**필수 Blackboard 변수:**
- `sweep_direction_x` — 스윕 방향 X 성분 (예: `1.0`)
- `sweep_direction_y` — 스윕 방향 Y 성분 (예: `0.0`)
- `sweep_distance` — 스윕 거리 [m] (예: `0.3`)

| 단계 | 동작 | 사용 SubTree | 주요 파라미터 |
|------|------|-------------|--------------|
| 1 | 비전으로 수건 가장자리 감지 | `DetectObject` | 최대 10회 재시도 |
| 2 | 핀치 프리셋 + 가장자리 상방 5cm 접근 | `ApproachFromAbove` | 엄지+검지 open |
| 3 | 저속 하강 + 핀치 파지 | `SlowDescend` + 인라인 | 0.5N, 1+ 핑거팁 |
| 4 | 25cm 들어올리기 | `LiftAndVerify` | 파지 검증 포함 |
| 5 | 컴플라이언트 게인 + 호 스윕 | (인라인) | kp_trans: 5.0 |
| 6 | 20cm 하강 | (인라인) | 강성 게인 복원 |
| 7 | 손 열기 + 10cm 후퇴 | `ReleaseAndRetreat` | retreat_speed: 0.1 m/s |

---

### Hand Motions Demo (`hand_motions.xml`)

UR5e가 고정 자세를 유지하는 동안, Hand가 가감속 opposition → wave 데모를 순차 수행한다.

**필수 Blackboard 변수:** 없음 (모두 트리 내부에서 초기화)

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

**Hand 관절 매핑 (10-DoF):**

| 이름 | 관절 | DoF | 인덱스 |
|------|------|-----|--------|
| `thumb` | CMC abd/add, CMC flex/ext, MCP flex/ext | 3 | 0-2 |
| `thumb_mcp` | MCP flex/ext | 1 | 2 |
| `index` | MCP abd/add, MCP flex/ext, DIP flex/ext | 3 | 3-5 |
| `index_dip` | DIP flex/ext | 1 | 5 |
| `middle` | MCP abd/add, MCP flex/ext, DIP flex/ext | 3 | 6-8 |
| `middle_dip` | DIP flex/ext | 1 | 8 |
| `ring` | MCP flex/ext | 1 | 9 |

---

## 커스텀 BT 트리 작성

### 기본 구조

```xml
<root BTCPP_format="4" main_tree_to_execute="MyTask">
  <include path="common_motions.xml"/>
  <BehaviorTree ID="MyTask">
    <Sequence name="main">
      <!-- 노드 배치 -->
    </Sequence>
  </BehaviorTree>
</root>
```

`trees/` 디렉토리에 XML 파일을 추가하면 자동으로 install에 포함된다.

### 공통 SubTree 활용 예시

```xml
<root BTCPP_format="4" main_tree_to_execute="MyPickTask">
  <include path="common_motions.xml"/>
  <BehaviorTree ID="MyPickTask">
    <Sequence>
      <SubTree ID="DetectObject"
               pose="{object_pose}" num_attempts="10" wait_s="0.5"/>
      <SubTree ID="ApproachFromAbove"
               target_pose="{object_pose}" offset_z="0.05"
               pos_tol="0.003" ori_tol="0.05" timeout_s="10.0"
               approach_pose="{approach_pose}"/>
      <SubTree ID="SlowDescend"
               target_pose="{object_pose}"
               traj_speed="0.05" max_traj_vel="0.1"
               pos_tol="0.002" ori_tol="0.05" timeout_s="8.0"/>
      <SubTree ID="ForceGrasp"
               grasp_mode="close" close_speed="0.3" max_position="1.4"
               grasp_timeout_s="10.0"
               threshold_N="1.5" min_fingertips="2" sustained_ms="200"
               verify_force_N="1.0" verify_min_tips="2"/>
      <SubTree ID="LiftAndVerify"
               base_pose="{object_pose}" offset_z="0.10"
               traj_speed="0.08" max_traj_vel="0.2"
               pos_tol="0.005" timeout_s="6.0"
               verify_force_N="0.8" verify_min_tips="2"
               lift_pose="{lift_pose}"/>
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

| SubTree | 주요 Input 포트 | Output 포트 |
|---------|----------------|-------------|
| `DetectObject` | `num_attempts`, `wait_s` | `pose` |
| `ApproachFromAbove` | `target_pose`, `offset_z`, `pos_tol`, `ori_tol`, `timeout_s` | `approach_pose` |
| `SlowDescend` | `target_pose`, `traj_speed`, `max_traj_vel`, `pos_tol`, `ori_tol`, `timeout_s` | - |
| `ForceGrasp` | `grasp_mode`, `close_speed`, `max_position`, `threshold_N`, `min_fingertips`, `sustained_ms`, `verify_force_N`, `verify_min_tips` | - |
| `LiftAndVerify` | `base_pose`, `offset_z`, `traj_speed`, `verify_force_N`, `verify_min_tips` | `lift_pose` |
| `ReleaseAndRetreat` | `base_pose`, `retreat_z`, `retreat_speed`, `timeout_s` | `retreat_pose` |
| `*EmergencyAbort` | `retreat_z`, `current_gains` | - |

### Hand 포즈/모션 추가 방법

#### Step 1: 포즈 정의 (`hand_pose_config.hpp`)

```cpp
//                        Thumb              Index              Middle           Ring
//                        CMCab CMCfe MCPfe  MCPab MCPfe DIPfe  MCPab MCPfe DIPfe MCPfe
{"my_grasp",  DegToRad(HandPose{20.0, 50.0, 40.0,   0.0, 55.0, 35.0,   0.0, 55.0, 35.0,  50.0})},
```

#### Step 2 (선택): 관절 그룹 정의 (`kFingerJointIndices`)

```cpp
{"index_mcp", {4}},        // index MCP flex/ext만
{"thumb_cmc", {0, 1}},     // thumb CMC 2관절만
```

#### Step 3: BT XML에서 사용

```xml
<SetHandPose pose="my_grasp" hand_trajectory_speed="1.0"/>
<MoveFinger finger_name="index" pose="my_grasp" hand_trajectory_speed="1.0"/>
<FlexExtendFinger finger_name="index_mcp" hand_trajectory_speed="1.0"/>
```

### 파일 수정 체크리스트

| 변경 내용 | 수정 파일 | 빌드 필요 |
|-----------|----------|----------|
| 기존 포즈 튜닝 | `config/poses.yaml` | X (런타임 오버라이드) |
| 새 포즈 추가 (컴파일타임) | `hand_pose_config.hpp` | O |
| 새 포즈 추가 (런타임) | `config/poses.yaml` | X |
| 새 관절 그룹 추가 | `hand_pose_config.hpp` | O |
| 새 BT 트리 추가 | `trees/*.xml` | X (런타임 로드) |
| 공통 SubTree 추가 | `trees/common_motions.xml` | X |
