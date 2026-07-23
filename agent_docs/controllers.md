# Controllers

## Controller Table

| Controller | Type | Space | Key Feature |
|------------|------|-------|-------------|
| PController | Position | Joint | `q + kp*error*dt` incremental |
| JointPDController | Torque | Joint | PD + Pinocchio RNEA + quintic trajectory |
| ClikController | Position | Cartesian 3/6-DOF | Damped Jacobian pseudoinverse + null-space |
| OSC | Torque | Cartesian 6-DOF | Full pose PD + SE3 quintic trajectory |
| GraspController | Internal | Hand 3x3-DOF | Adaptive PI force, 6-state FSM, per-finger stiffness EMA |
| DemoJointController | Position | Joint + Hand | Quintic trajectory, `grasp_controller_type: "contact_stop"\|"force_pi"\|"none"` |
| DemoTaskController | Position | Cartesian + Hand | CLIK + trajectory, `grasp_controller_type: "contact_stop"\|"force_pi"\|"none"` |
| DemoWbcController | Position | TSID QP + Hand | `initial_controller` default — **`ur5e_p1a`·`iiwa7_leap` 만**. `ur5e_p1b` 는 sim·robot 둘 다 `demo_joint_controller` 다 (robot profile 별 `{sim,robot}.yaml` 이 SSoT). 6-phase FSM (Idle->Approach->Closure->Hold->Release; slots 2 & 5 reserved, RELEASE preempts from any non-terminal phase), TSID QP -> accel -> position integration across all phases, contact-aware ForceTask + FrictionCone, sensor-driven contact / slip / deformation guards, combined 16-DoF model. MPC default: `engine: "handler"` + `enabled: false` (structural gate; MPC thread inert and TSID self-holds until YAML `mpc.enabled: true` AND runtime `mpc_enable` — see line below) |

### Base controller joint order & submodel selection (#172 Phase 3)

Base `rtc_controllers`(P/JointPD/CLIK/OSC)는 데모 컨트롤러의 `CombinedModelCache` 와 별개로, 자체
`RtModelHandle` 위에서 **device joint order** 와 **primary-device submodel** 를 처리한다.

- **Joint reorder (A2)**: `OnDeviceConfigsSet` 에서 `js==nv` 이면 `handle_->SetJointOrder(joint_state_names)`.
  device 순서 == URDF 순서면 `HasJointReorder()==false` → memcpy fallback(zero-overhead, 기존 로봇 불변).
  다르면 모델은 device 순서 입력을 correct 하게 소비하고, model 파생 항을 device channel 순서로 되돌린다:
  JointPD `g`/`C·v`, OSC `τ`(주경로)+null-space `tau0`, CLIK `dq`/`traj_dq`+null-space `null_err`.
  device 순서로 형성한 항(null-space·Coriolis)을 Pinocchio 순서 행렬과 곱하기 직전 `RtModelHandle::ReorderInput`
  (device→Pinocchio scatter, `ReorderOutput` 의 역방향)으로 1회 gather 한다. P 는 FK 입력만 reorder(joint
  command 은 device-order native, task 출력은 order-invariant → 출력 reorder 불요).
- **Submodel selection (A1)**: `LoadConfig`(base LoadConfig 가 `topic_config_` 채운 직후)에서
  `MaybeSelectSubModel` — `GetSystemModelConfig().sub_models` 중 `GetPrimaryDeviceName()` 과 이름이 일치하는
  sub_model 이 있으면 `GetReducedModel(primary)` 로 handle_ 교체(`InitFromModel` 이 nv-크기 버퍼 재할당). system
  config 없음/매칭 없음 → ctor 의 full model 유지(무회귀). hook 순서상 `OnSystemModelConfigSet` 시점엔
  `topic_config_` 미준비라 선택 불가 → `LoadConfig` 배치(DemoJointController 레퍼런스와 동일).

### Unified arm kin&dyn (joint / task / wbc)

세 데모 컨트롤러(DemoJoint/DemoTask/DemoWbc)는 arm kinematics(FK·Jacobian)를 **결합(arm+hand) actuated
모델 위의 `rtc_urdf_bridge::PinocchioCache`** 에서 획득한다 — non-E-STOP tick당 `Update(q,v)` 1회
(`GetActuatedModel` → reduced tree `"wbc"` → full model 순으로 선택). arm-only `RtModelHandle`(`arm_handle_`)은
E-STOP TF 경로(`ComputeEstop`)와 메타데이터(`nv()`/`GetFrameId`)로만 잔존. closed-chain 손에서는 접촉
프레임 J·oMf·dJv가 loop-consistent로 격상된다(dJv는 `RtClosedChainHandle` drift API 로 L2-exact, issue #173 종결).

이 cache 배선(모델 선택 + `PinocchioCache.Init`, ext(device joint-state)→Pinocchio q/v reorder map, 매 tick
state scatter, arm-TCP FK)은 **공유 타입 `integrated_bringup::CombinedModelCache`(support/) 한 곳으로 통합**됐다
(issue #174, 이전 삼중 복제 해소). 세 컨트롤러는 멤버 `combined_cache_` 를 소유·위임한다(`InitModel`/`BuildReorderMap`/
`ExtractFullState`/`Update`/`ArmTcpPoseFromCache` + `cache()`/`model()`/`reorder_valid()`/`q()`/`v()` accessor).
WBC 는 superset 소비자로, TSID contact 프레임과 reduced-dynamics provider 는 WBC-local 로 유지하고, 모델선택은
`SelectModel`(contact frame id 파싱이 선택된 모델을 요구해 cache Init 을 뒤로 미룸) + 이후 `cache().Init` 로 분리
호출한다.

RT 진입점 `Compute(state)` 는 세 컨트롤러 공통으로 **`ReadState`(순수 raw read + `ExtractFullState`) → compute
model(`combined_cache_.Update`) → compute control law → `WriteJointCommand`(순수 command write)** 순서를 지킨다
(로그/발행용 oMf 소비는 `FillLogOutput`/`FillPublishOutput` 로 분리). cache.Update 텍스트 위치는 joint·wbc = `ComputeControl`
Stage 1, task = Compute-scope Stage 1(DrainTargetSlot self-init 이 fresh cache 의존). WBC 상세 구현:
[docs/WBC_CONTROLLER_IMPLEMENTATION.md](../docs/WBC_CONTROLLER_IMPLEMENTATION.md).

## Gains (per-controller ROS 2 parameters)

게인 채널은 ROS 2 parameter API로 노출된다 ([rtc_msgs/srv/GraspCommand.srv](../rtc_msgs/srv/GraspCommand.srv)). Legacy `~/controller_gains` / `~/request_gains` / `~/current_gains` 토픽 + `UpdateGainsFromMsg`/`GetCurrentGains` 가상 메서드는 모두 제거.

각 데모 컨트롤러는 자기 LifecycleNode (`/<config_key>`) 에서 `declare_parameter`로 게인을 노출하고, `add_on_set_parameters_callback`이 SeqLock writer 측으로 mutate→Store. RT 경로는 `gains_lock_.Load()` 스냅샷만 본다.

각 데모 컨트롤러가 노출하는 정확한 parameter list 는 코드/YAML 이 SSoT:

```bash
# 활성화 후 actual parameter list
ros2 param list /demo_wbc_controller
ros2 param describe /demo_wbc_controller <param>
```

또는 컨트롤러 source 의 `DeclareGainParameters()` 멤버 + bringup config (`integrated_bringup/config/<robot>/controllers/<config_key>.yaml`).

읽기 전용 cap parameter (`*_max_traj_velocity` 등) 는 `ParameterDescriptor::read_only=true` 로 선언. One-shot 이벤트 (Force-PI grasp) 는 `~/grasp_command` srv (`rtc_msgs/srv/GraspCommand`) — active controller 만 server 를 띄움.

`mpc_enable`은 빌드타임 `mpc_enabled_` (YAML `mpc.enabled`) 와 AND 결합 — YAML이 false면 런타임 1은 무시된다. `riccati_gain_scale`은 `[0,1]`로 자동 clamp.

PController / JointPDController / ClikController / OSC 등 핵심 `rtc_controllers`는 게인 채널을 노출하지 않는다 (게인은 controller-specific YAML로 로드 후 `LoadConfig` 시점에 고정).

런타임 튜닝 예:

```bash
ros2 param set /demo_wbc_controller se3_weight 150.0
ros2 param set /demo_task_controller kp_translation '[20.0, 20.0, 30.0]'

# Force-PI grasp (one-shot event)
ros2 service call /demo_task_controller/grasp_command \
    rtc_msgs/srv/GraspCommand "{command: 1, target_force: 2.0}"
```

## DemoWbcController — Kinematic WBC / Dynamic WBC

DemoWbcController의 position-mode tick은 `ComputeTSIDPosition()`이 한 gains 스냅샷으로
**`ComputeWbcCommon`**(공유 stage: state/cache/reference, no solve) → **`ComputeKinematicWbc`**
(arm/hand position) → **`ComputeDynamicWbc`**(hand τ_ff)를 순차 orchestrate한다
(`src/controllers/wbc/compute.cpp`). 두 QP는 같은 Common reference를 독립 소비한다 (decision 5).

- **Kinematic WBC** = CLIK-QP 경로 (`rtc::tsid::ClikReferenceGenerator`). arm SE3 추종(L1) + arm
  redundancy/nullspace posture(L2) + hand posture(L3)를 velocity-level box-QP로 풀어
  `q_ref = q + v_ref·dt`를 낸다. **유일한 position backbone** — CLIK 실패는 critical
  (hold-last → N회 연속 실패 시 `kFallback`). CLIK 계약은 `nq==nv` (reduced wbc 모델)이며,
  비충족 시 `on_configure`가 lifecycle transition을 FAIL시킨다 (DEC-1: integrator fallback 없음).
- **Dynamic WBC** = hand 전용 τ_ff 경로 (`hand_tauff_*` → `kPdFeedforward`). PD position
  backbone 위에 model-based feedforward torque(gravity_comp | tsid_tau)를 overlay. closure/hold
  에서만, `±hand_tauff_max` clamp, opt-in `hand_tauff_enable`.

**Commanded SE3 target** — `RobotTarget{goal_type:"task"}`(arm, device 0)은 base interface의
`SetDeviceTaskTarget`을 거쳐 commanded SE3 slot으로 들어가고, `kRelease`/`kFallback`을 제외한
**모든 phase에서 live override**로 `tcp_goal_`을 그 pose로 갱신한다 (joint posture slot과 독립; CLIK가
추종). 단 `kClosure`/`kHold`/`kRelease`/`kFallback` **진입 시점**에는 직전의 stale
commanded SE3를 clear하므로 (idle 복귀 시 grasp 명령으로 jog되는 것 방지), 해당 phase에서는
**진입 후 새로 도착한** SE3만 재적용된다. `kRelease`/`kFallback`은 자신의 SE3 goal을 소유하여 live
override를 받지 않는다. `goal_type:"joint"`(arm)은 arm posture로 들어간다.
(x,y,z,r,p,y)→SE3 변환은 ZYX(yaw·pitch·roll) 규약.

**Hand joint target** — `RobotTarget{goal_type:"joint"}`(hand, device 1)은 `kRelease`(finger-open
ramp가 hand 점유)/`kFallback`(safety hold)을 제외한 모든 phase에서 매 tick `BuildHandTargetPosture`로
`q_des_target_full_` hand block에 fold되어 CLIK posture term(`clik_kh`)이 추종한다 — phase-entry
edge를 기다리지 않는 live command. (FSM 의 hold/non-hold 판정은 hand τ_ff overlay만 게이트.)

## GraspController (Force-PI, internal only)

Selected via `grasp_controller_type: "force_pi"` in demo controller YAML (default: `"contact_stop"`). Whitelist `{force_pi, contact_stop, none}` — `none` disables all hand intervention (trajectory passes through) while GraspState aggregation/publishing continues; any other string fails `on_configure`.

**FSM**: Idle -> Approaching (position ramp) -> Contact (settle) -> ForceControl (PI + force ramp) -> Holding (anomaly monitor) -> Releasing

PI gain / threshold / slip detection 상수 default 값은 `rtc_controllers/include/.../grasp_types.hpp` 가 SSoT — controller-specific YAML 로 override 가능.

**Grasp detection thresholds (capability-aware).** `grasp_contact_threshold` · `grasp_force_threshold` · `grasp_min_fingertips` 는 데모 컨트롤러 (위 §Controller Table 의 `Demo*`) 전반에서 동일한 키로 노출되며, `devices.<hand>.sensor_layout.has_native_contact` (robot/sim yaml) 에 따라 분기한다 — sensor A (native 접촉 확률 보유) 는 두 threshold 의 AND, sensor B (force-only) 는 force_threshold 단독. 전체 표는 [integrated_bringup/README.md](../integrated_bringup/README.md) 의 "Grasp threshold capability matrix" 섹션.

## ROS2 Topics

**Controller Manager**: switch via `/rtc_cm/switch_controller` (srv, sync, single-active), query via `/rtc_cm/list_controllers` (srv); `/rtc_cm/active_controller_name` (Pub, latched — rewire trigger for downstream nodes; namespace 템플릿이 아니라 `RtControllerNode` 가 박아 쓰는 절대 토픽명이다), `/system/estop_status` (Pub). Gain 채널은 더 이상 manager가 소유하지 않으며 컨트롤러별 LifecycleNode parameter로 이관 (위 §Gains 참조).

**Dynamic** (per controller TopicConfig):
- **DeviceBackend-owned (HW/sim ↔ controller boundary, Phase 4 SSoT)** — `state_topic` / `motor_topic` / `sensor_topic` (HW→controller) and `command_topic` (controller→HW/sim) are declared in `devices.<group>.backend:` (sim.yaml/robot.yaml) and owned by `DeviceBackend` impls (`mujoco_native` / `ur_driver_native` / `udp_hand_native`). CM no longer reads device-wire roles from controller YAML.
- **Controller-owned (`<config_key>/` namespace, `PublishNonRtSnapshot`)** — YAML role-mapped: `kRobotTransforms` 하나 (issue #196 Phase 5: publisher 가 없던 `kRobotTarget` / `kDigitalTwinState` 제거 — 선언하면 조용히 죽은 토픽이 됐다). CM은 SPSC snapshot 운반만 담당하며 controller YAML 토픽 퍼블리셔를 만들지 않는다 (issue #138: controller YAML entry 는 전부 controller-owned, `ownership:` field 없음). Phase 4: `kGuiPosition` 폐기 — `/rtc_cm/<group>/joint_states` + `<config_key>/transforms` 로 대체.
- **Controller-owned (no YAML role)** — `GraspState` / `WbcState` / `ToFSnapshot` 은 각 컨트롤러가 `Setup{Grasp,Wbc,ToF}*Publisher` 헬퍼로 직접 생성하고 자체 `SeqLock<T>` 로 RT compute → publish thread 전달. `PublishSnapshot` 에서 완전히 분리되어 CM 은 의미를 모름.
- **매 tick Store 계약 (PROC-7, issue #234 P-1)** — CM 은 tick 마다 새 `stamp_ns` 로 snapshot 을 push 하고 publish thread 는 그때마다 SeqLock 을 다시 Load 한다. 따라서 **Store 를 건너뛴 tick 은 "발행 안 함" 이 아니라 "직전 body 를 현재 stamp 로 재발행"** 이다. E-STOP 처럼 제어 법칙을 건너뛰는 tick 도 반드시 Store 하되, 이번 tick 에 계산하지 않은 필드는 명시적으로 무효화한다 — 각 컨트롤러의 `FillEstopPublishState()` 가 그 경로다 (센서 유래 집계는 갱신, Force-PI per-finger 진단은 0, TSID health 는 not-solved, pull 추정은 `StageEstopPullTick` 으로 `valid=false`+감쇠). `pull_estimator.csv` 도 같은 이유로 gap 이 아니라 `valid=0` 행을 남긴다.
- **상호 배타**: `GraspState` 와 `WbcState` — Force-PI 데모(DemoJoint/Task)만 grasp_state, TSID 데모(DemoWbc)만 wbc_state. DemoWbcController는 `<config_key>/<secondary>/wbc_state` (RELIABLE/1) 로 발행 (secondary = `p1a` for ur5e_p1a, `leap` for iiwa7_leap).

**CM per-group JointState**: `/rtc_cm/{group}/joint_states` (RELIABLE, depth 1) -> `rtc_digital_twin` merges -> RViz2

**Hand Driver** (ur5e_p1a instance; generic driver default is `/hand/`, remapped per variant): `/p1a/joint_states`, `/p1a/motor_states`, `/p1a/sensor_states` (Pub); `/p1a/joint_command` (Sub)

**MuJoCo**: `<group.state_topic>` (Pub), `<group.command_topic>` (Sub), `/sim/status` (1Hz)

## Configuration Files

YAML config 트리:

- **Robot-specific bringup** (`integrated_bringup/config/<robot>/`) — `{sim,robot}.yaml` (CM-level: `control_rate`, `initial_controller`, `devices.<group>.backend`, `urdf`, `device_timeout_*`), `mujoco_simulator.yaml` (per-robot overlay), `digital_twin.yaml` (per-robot overlay), `controllers/<config_key>.yaml` (production controller params — controller 당 한 파일의 **flat** 배치. 유일한 하위 디렉토리는 `controllers/mpc/` (`phase_config`·`contact_light`·`contact_rich`) 이며 `demo_wbc_controller.yaml` 이 참조한다. `direct/`·`indirect/` 하위 디렉토리는 `rtc_controllers` 예제 레이아웃이지 여기가 아니다)
- **Agnostic defaults** — `rtc_mujoco_sim/config/solver_param.yaml` (MuJoCo solver SSoT), `rtc_digital_twin/config/digital_twin.yaml` (robot-agnostic display defaults), `udp_hand_driver/config/udp_hand_node.yaml` (UDP transport)

**컨트롤러 YAML 이 어디서 로드되는가**: `RTC_REGISTER_CONTROLLER(config_key, config_subdir, config_package, ...)` 의 2번째 인자가 lookup 경로의 하위 디렉토리를 정한다. `integrated_bringup` 의 데모 컨트롤러 3종은 `config_subdir` 로 빈 문자열을 넘기므로 `config/<robot>/controllers/<config_key>.yaml` 이 되고, `rtc_controllers` 예제만 `"direct"` / `"indirect"` 를 넘겨 하위 디렉토리를 갖는다. 문서가 예제 레이아웃을 production 레이아웃으로 서술하는 오류가 반복됐으므로 (#213) 새 컨트롤러 추가 시 등록 매크로의 인자를 먼저 확인한다.

- **Reference only** — `rtc_controllers/examples/controllers/{direct,indirect}/*.yaml` (`<robot>` placeholder, production 은 위 robot-specific path 가 owner)

각 YAML 의 정확한 key list 는 파일 자체 + 해당 노드의 `declare_parameter` 호출이 SSoT — 코드에서 grep.
