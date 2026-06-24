# Controllers

## Controller Table

| Controller | Type | Space | Key Feature |
|------------|------|-------|-------------|
| PController | Position | Joint | `q + kp*error*dt` incremental |
| JointPDController | Torque | Joint | PD + Pinocchio RNEA + quintic trajectory |
| ClikController | Position | Cartesian 3/6-DOF | Damped Jacobian pseudoinverse + null-space |
| OSC | Torque | Cartesian 6-DOF | Full pose PD + SE3 quintic trajectory |
| GraspController | Internal | Hand 3x3-DOF | Adaptive PI force, 6-state FSM, per-finger stiffness EMA |
| DemoJointController | Position | Joint + Hand | Quintic trajectory, `grasp_controller_type: "contact_stop"\|"force_pi"` |
| DemoTaskController | Position | Cartesian + Hand | CLIK + trajectory, `grasp_controller_type: "contact_stop"\|"force_pi"` |
| DemoWbcController | Position | TSID QP + Hand | **Default `initial_controller`** (sim+robot). 7-phase FSM (Idle->Approach->PreGrasp->Closure->Hold->Release; slot 5 reserved, RELEASE preempts from any non-terminal phase), TSID QP -> accel -> position integration across all phases, contact-aware ForceTask + FrictionCone, sensor-driven contact / slip / deformation guards, combined 16-DoF model. MPC default: `engine: "handler"` + `enabled: true` (Aligator HandlerMPCThread; runtime-togglable via `mpc_enable` parameter) |

## Gains (per-controller ROS 2 parameters)

게인 채널은 ROS 2 parameter API로 노출된다 ([rtc_msgs/srv/GraspCommand.srv](../rtc_msgs/srv/GraspCommand.srv)). Legacy `~/controller_gains` / `~/request_gains` / `~/current_gains` 토픽 + `UpdateGainsFromMsg`/`GetCurrentGains` 가상 메서드는 모두 제거.

각 데모 컨트롤러는 자기 LifecycleNode (`/<config_key>`) 에서 `declare_parameter`로 게인을 노출하고, `add_on_set_parameters_callback`이 SeqLock writer 측으로 mutate→Store. RT 경로는 `gains_lock_.Load()` 스냅샷만 본다.

각 데모 컨트롤러가 노출하는 정확한 parameter list 는 코드/YAML 이 SSoT:

```bash
# 활성화 후 actual parameter list
ros2 param list /demo_wbc_controller
ros2 param describe /demo_wbc_controller <param>
```

또는 컨트롤러 source 의 `DeclareGainParameters()` 멤버 + bringup config (`<robot>_bringup/config/<robot>/controllers/{direct,indirect}/<config_key>.yaml`).

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
추종). 단 `kPreGrasp`/`kClosure`/`kHold`/`kRelease`/`kFallback` **진입 시점**에는 직전의 stale
commanded SE3를 clear하므로 (idle 복귀 시 pre-grasp 명령으로 jog되는 것 방지), 해당 phase에서는
**진입 후 새로 도착한** SE3만 재적용된다. `kRelease`/`kFallback`은 자신의 SE3 goal을 소유하여 live
override를 받지 않는다. `goal_type:"joint"`(arm)은 arm posture로 들어간다.
(x,y,z,r,p,y)→SE3 변환은 ZYX(yaw·pitch·roll) 규약.

**Hand joint target** — `RobotTarget{goal_type:"joint"}`(hand, device 1)은 `kRelease`(finger-open
ramp가 hand 점유)/`kFallback`(safety hold)을 제외한 모든 phase에서 매 tick `BuildHandTargetPosture`로
`q_des_target_full_` hand block에 fold되어 CLIK posture term(`clik_kh`)이 추종한다 — phase-entry
edge를 기다리지 않는 live command. (FSM 의 hold/non-hold 판정은 hand τ_ff overlay만 게이트.)

## GraspController (Force-PI, internal only)

Selected via `grasp_controller_type: "force_pi"` in demo controller YAML (default: `"contact_stop"`).

**FSM**: Idle -> Approaching (position ramp) -> Contact (settle) -> ForceControl (PI + force ramp) -> Holding (anomaly monitor) -> Releasing

PI gain / threshold / slip detection 상수 default 값은 `rtc_controllers/include/.../grasp_types.hpp` 가 SSoT — controller-specific YAML 로 override 가능.

**Grasp detection thresholds (capability-aware).** `grasp_contact_threshold` · `grasp_force_threshold` · `grasp_min_fingertips` 는 데모 컨트롤러 (위 §Controller Table 의 `Demo*`) 전반에서 동일한 키로 노출되며, `devices.<hand>.sensor_layout.has_native_contact` (robot/sim yaml) 에 따라 분기한다 — sensor A (native 접촉 확률 보유) 는 두 threshold 의 AND, sensor B (force-only) 는 force_threshold 단독. 전체 표는 [integrated_bringup/README.md](../integrated_bringup/README.md) 의 "Grasp threshold capability matrix" 섹션.

## ROS2 Topics

**Controller Manager**: switch via `/rtc_cm/switch_controller` (srv, sync, single-active), query via `/rtc_cm/list_controllers` (srv); `/{ns}/active_controller_name` (Pub, latched — rewire trigger for downstream nodes), `/system/estop_status` (Pub). Gain 채널은 더 이상 manager가 소유하지 않으며 컨트롤러별 LifecycleNode parameter로 이관 (위 §Gains 참조).

**Dynamic** (per controller TopicConfig):
- **DeviceBackend-owned (HW/sim ↔ controller boundary, Phase 4 SSoT)** — `state_topic` / `motor_topic` / `sensor_topic` (HW→controller) and `command_topic` (controller→HW/sim) are declared in `devices.<group>.backend:` (sim.yaml/robot.yaml) and owned by `DeviceBackend` impls (`mujoco_native` / `ur_driver_native` / `udp_hand_native`). CM no longer reads device-wire roles from controller YAML.
- **CM-owned (controller YAML)** — Subscribe: `kTarget` (외부 RobotTarget→controller).
- **Controller-owned (`<config_key>/` namespace, `PublishNonRtSnapshot`)** — YAML role-mapped: `kRobotTarget`, `kRobotTransforms`, `kDigitalTwinState`. CM은 SPSC snapshot 운반만 담당하며 퍼블리셔를 만들지 않는다 (YAML `ownership: manager` 라도 CM은 무시). Phase 4: `kGuiPosition` 폐기 — `/rtc_cm/<group>/joint_states` + `<config_key>/transforms` 로 대체.
- **Controller-owned (no YAML role)** — `GraspState` / `WbcState` / `ToFSnapshot` 은 각 컨트롤러가 `Setup{Grasp,Wbc,ToF}*Publisher` 헬퍼로 직접 생성하고 자체 `SeqLock<T>` 로 RT compute → publish thread 전달. `PublishSnapshot` 에서 완전히 분리되어 CM 은 의미를 모름.
- **상호 배타**: `GraspState` 와 `WbcState` — Force-PI 데모(DemoJoint/Task)만 grasp_state, TSID 데모(DemoWbc)만 wbc_state. DemoWbcController는 `<config_key>/<secondary>/wbc_state` (RELIABLE/10) 로 발행 (secondary = `hand` for ur5e_hand, `leap` for iiwa7_leap).

**CM per-group JointState**: `/rtc_cm/{group}/joint_states` (RELIABLE) -> `rtc_digital_twin` merges -> RViz2

**Hand Driver**: `/hand/joint_states`, `/hand/motor_states`, `/hand/sensor_states` (Pub); `/hand/joint_command` (Sub)

**MuJoCo**: `<group.state_topic>` (Pub), `<group.command_topic>` (Sub), `/sim/status` (1Hz)

## Configuration Files

YAML config 트리:

- **Robot-specific bringup** (`integrated_bringup/config/<robot>/`) — `{sim,robot}.yaml` (CM-level: `control_rate`, `initial_controller`, `devices.<group>.backend`, `urdf`, `device_timeout_*`), `mujoco_simulator.yaml` (per-robot overlay), `digital_twin.yaml` (per-robot overlay), `controllers/{direct,indirect}/<config_key>.yaml` (production controller params)
- **Agnostic defaults** — `rtc_mujoco_sim/config/solver_param.yaml` (MuJoCo solver SSoT), `rtc_digital_twin/config/digital_twin.yaml` (robot-agnostic display defaults), `udp_hand_driver/config/udp_hand_node.yaml` (UDP transport)
- **Reference only** — `rtc_controllers/examples/controllers/{direct,indirect}/*.yaml` (`<robot>` placeholder, production 은 위 robot-specific path 가 owner)

각 YAML 의 정확한 key list 는 파일 자체 + 해당 노드의 `declare_parameter` 호출이 SSoT — 코드에서 grep.
