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
| DemoWbcController | Position | TSID QP + Hand | **Default `initial_controller`** (sim+robot). 8-phase FSM (Idle->Approach->PreGrasp->Closure->Hold->Retreat->Release), TSID QP -> accel -> position integration, contact-aware ForceTask + FrictionCone, sensor-driven contact / slip / deformation guards, combined 16-DoF model. MPC default: `engine: "handler"` + `enabled: true` (Aligator HandlerMPCThread; runtime-togglable via `mpc_enable` parameter) |

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

## GraspController (Force-PI, internal only)

Selected via `grasp_controller_type: "force_pi"` in demo controller YAML (default: `"contact_stop"`).

**FSM**: Idle -> Approaching (position ramp) -> Contact (settle) -> ForceControl (PI + force ramp) -> Holding (anomaly monitor) -> Releasing

PI gain / threshold / slip detection 상수 default 값은 `rtc_controllers/include/.../grasp_types.hpp` 가 SSoT — controller-specific YAML 로 override 가능.

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
