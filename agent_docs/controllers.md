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
| DemoWbcController | Position | TSID QP + Hand | 8-phase FSM (Idle->Approach->PreGrasp->Closure->Hold->Retreat->Release), TSID QP -> accel -> position integration, contact-aware ForceTask + FrictionCone, sensor-driven contact / slip / deformation guards, combined 16-DoF model |

## Gains Layout (via `~/controller_gains` topic)

| Controller | Layout | Count |
|------------|--------|-------|
| PController | `[kp x 6]` | 6 |
| JointPDController | `[kp x 6, kd x 6, gravity(0/1), coriolis(0/1), traj_speed]` | 15 |
| ClikController | `[kp_trans x 3, kp_rot x 3, damping, null_kp, null_space(0/1), 6dof(0/1), traj_speed, traj_ang_speed, max_vel, max_ang_vel]` | 14 |
| OSC | `[kp_pos x 3, kd_pos x 3, kp_rot x 3, kd_rot x 3, damping, gravity(0/1), traj_speed, traj_ang_speed, max_vel, max_ang_vel]` | 18 |
| DemoJoint | `[robot_traj_speed, hand_traj_speed, robot_max_vel, hand_max_vel, grasp_contact_thresh, grasp_force_thresh, grasp_min_fingertips, grasp_cmd(0/1/2), grasp_target_force]` | 9 |
| DemoTask | `[kp_trans x 3, kp_rot x 3, damping, null_kp, null_space(0/1), 6dof(0/1), traj_speed, traj_ang_speed, hand_traj_speed, max_vel, max_ang_vel, hand_max_vel, grasp_contact_thresh, grasp_force_thresh, grasp_min_fingertips, grasp_cmd(0/1/2), grasp_target_force]` | 21 |
| DemoWbc | `[grasp_cmd(0/1/2), grasp_target_force, arm_traj_speed, hand_traj_speed, se3_weight, force_weight, posture_weight, mpc_enable(0/1), riccati_gain_scale(0..1)]` | 9 |

## GraspController (Force-PI, internal only)

Selected via `grasp_controller_type: "force_pi"` in demo controller YAML (default: `"contact_stop"`).

**FSM**: Idle -> Approaching (position ramp) -> Contact (settle) -> ForceControl (PI + force ramp) -> Holding (anomaly monitor) -> Releasing

Key params in `grasp_types.hpp`: `Kp_base=0.02`, `Ki_base=0.002`, `f_target=2.0N`, `f_contact=0.2N`, `ds_max=0.05/s`, `delta_s_max=0.15` deformation guard, slip detection at `5.0 N/s`.

## ROS2 Topics

**Controller Manager** (`/{ns}/`): `controller_type` (Sub, switch), `controller_gains` (Sub), `active_controller_name` (Pub), `/system/estop_status` (Pub)

**Dynamic** (per controller TopicConfig): Subscribe `kState`/`kMotorState`/`kSensorState`/`kTarget`; Publish `kJointCommand`/`kRos2Command`/`kGuiPosition`/`kGraspState`/`kDeviceStateLog`/`kDeviceSensorLog`

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
