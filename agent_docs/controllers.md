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
| DemoWbcController | Position | TSID QP + Hand | **Default `initial_controller` (sim+robot, `5118f67`)**. 8-phase FSM (Idle->Approach->PreGrasp->Closure->Hold->Retreat->Release), TSID QP -> accel -> position integration, contact-aware ForceTask + FrictionCone, sensor-driven contact / slip / deformation guards, combined 16-DoF model. MPC default: `engine: "handler"` + `enabled: true` (Aligator HandlerMPCThread; runtime-togglable via `gains[7]`) |

## Gains (per-controller ROS 2 parameters)

게인 채널은 ROS 2 parameter API로 노출된다 ([rtc_msgs/srv/GraspCommand.srv](../rtc_msgs/srv/GraspCommand.srv) Phase A~E migration, 2026-04-26). Legacy `~/controller_gains` / `~/request_gains` / `~/current_gains` 토픽 + `UpdateGainsFromMsg`/`GetCurrentGains` 가상 메서드는 **모두 제거**됨.

각 데모 컨트롤러는 자기 LifecycleNode (`/<config_key>`) 에서 `declare_parameter`로 게인을 노출하고, `add_on_set_parameters_callback`이 SeqLock writer 측으로 mutate→Store. RT 경로는 `gains_lock_.Load()` 스냅샷만 본다.

| Controller (config_key) | Tunable parameters | Read-only | One-shot srv |
|------------------------|--------------------|-----------|--------------|
| `demo_joint_controller` | `robot_trajectory_speed`, `hand_trajectory_speed`, `grasp_contact_threshold`, `grasp_force_threshold`, `grasp_min_fingertips` | `robot_max_traj_velocity`, `hand_max_traj_velocity` | `~/grasp_command` ([rtc_msgs/srv/GraspCommand](../rtc_msgs/srv/GraspCommand.srv); Force-PI start/release) |
| `demo_task_controller` | `kp_translation[3]`, `kp_rotation[3]`, `damping`, `null_kp`, `enable_null_space`, `control_6dof`, `trajectory_speed`, `trajectory_angular_speed`, `hand_trajectory_speed`, `grasp_contact_threshold`, `grasp_force_threshold`, `grasp_min_fingertips` | `max_traj_velocity`, `max_traj_angular_velocity`, `hand_max_traj_velocity` | `~/grasp_command` |
| `demo_wbc_controller` | `arm_trajectory_speed`, `hand_trajectory_speed`, `se3_weight`, `force_weight`, `posture_weight`, `mpc_enable`, `riccati_gain_scale` | `arm_max_traj_velocity`, `hand_max_traj_velocity` | `~/grasp_command` (updates `grasp_cmd_` atomic; WBC FSM consumes) |

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

Key params in `grasp_types.hpp`: `Kp_base=0.02`, `Ki_base=0.002`, `f_target=2.0N`, `f_contact=0.2N`, `ds_max=0.05/s`, `delta_s_max=0.15` deformation guard, slip detection at `5.0 N/s`.

## ROS2 Topics

**Controller Manager**: switch via `/rtc_cm/switch_controller` (srv, sync, single-active), query via `/rtc_cm/list_controllers` (srv); `/{ns}/active_controller_name` (Pub, latched — rewire trigger for downstream nodes), `/system/estop_status` (Pub). Gain 채널은 더 이상 manager가 소유하지 않으며 컨트롤러별 LifecycleNode parameter로 이관 (위 §Gains 참조).

**Dynamic** (per controller TopicConfig): Subscribe `kState`/`kMotorState`/`kSensorState`/`kTarget`; Publish `kJointCommand`/`kRos2Command`/`kGuiPosition`/`kGraspState`/`kWbcState`/`kDeviceStateLog`/`kDeviceSensorLog`. `kWbcState` (controller-owned, RELIABLE/10) — DemoWbcController가 `<config_key>/hand/wbc_state` 로 발행. `kGraspState` 와 상호 배타: Force-PI 데모(DemoJoint/Task)만 grasp_state, TSID 데모(DemoWbc)만 wbc_state.

**Digital Twin**: `/{group}/digital_twin/joint_states` (RELIABLE) -> `rtc_digital_twin` merges -> RViz2

**Hand Driver**: `/hand/joint_states`, `/hand/motor_states`, `/hand/sensor_states` (Pub); `/hand/joint_command` (Sub)

**MuJoCo**: `<group.state_topic>` (Pub), `<group.command_topic>` (Sub), `/sim/status` (1Hz)

## Configuration Files

| Config | Path | Key Parameters |
|--------|------|----------------|
| RT controller manager | `ur5e_bringup/config/ur5e_robot.yaml` / `ur5e_sim.yaml` | `control_rate`, `initial_controller`, `devices`, `urdf`, `device_timeout_*` |
| MuJoCo simulator (agnostic 기본값) | `rtc_mujoco_sim/config/mujoco_default.yaml` | `physics_timestep`, `n_substeps`, `sync_timeout_ms`, viewer 설정 |
| MuJoCo simulator (UR5e robot 오버레이) | `ur5e_bringup/config/mujoco_simulator.yaml` | `model_path`, `robot_response.groups` (`ur5e`, `hand`), joint names, command/state 토픽 |
| MuJoCo solver | `rtc_mujoco_sim/config/solver_param.yaml` | `solver` (Newton/CG/PGS), `cone`, `integrator`, `noslip_iterations`, `contact_override` |
| Hand UDP driver | `ur5e_hand_driver/config/hand_udp_node.yaml` | `target_ip`, `recv_timeout_ms`, `communication_mode` (bulk/individual) |
| Digital twin | `rtc_digital_twin/config/digital_twin.yaml` | `display_rate`, `num_sources`, `auto_compute_mimic` |
| Controller YAMLs | `rtc_controllers/config/controllers/{direct\|indirect}/*.yaml` | Per-controller gains + `topics:` section for device-group routing |
