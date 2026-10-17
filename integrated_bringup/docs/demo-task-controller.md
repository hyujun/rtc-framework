# DemoTaskController

태스크 공간 CLIK + 손 quintic 궤적이다. 감쇠 의사역행렬과 영공간 보조 태스크로 TCP 목표를 추종한다. config_key 는 `demo_task_controller`, YAML 은 `ur5e_p1a` · `ur5e_p1b` · `iiwa7_leap` 이 싣는다.

공통 파라미터 · 팔 끝 frame · 손끝 FK 배선 · grasp mode 전환은 [demo-shared-params.md](demo-shared-params.md) 가 갖는다. 같은 CLIK 에 외부 렌치 admittance 를 더한 컨트롤러는 [demo-compliance-controller.md](demo-compliance-controller.md) 다.

## 팔 제어 법칙

```
pos_error = x_traj(t) - FK(q)
J^# = J^T (J J^T + lambda^2 I)^{-1}
N   = I - J^# J
dq  = kp * J^# * pos_error + traj_velocity + null_kp * N * (q_null - q)
q_des += clamp(dq, +/-v_max) * dt          (trajectory 갱신 시 q_des = q_actual 로 초기화)
q_cmd  = q_des
```

## 목표 입력

목표는 `/demo_task_controller/<group>/joint_goal` (`rtc_msgs/RobotTarget`) 로 보낸다. 팔은 task 목표만, 손은 joint 목표만 받는다 — 반대 종류는 거부하고 센다 ([Task 목표의 기준 frame](../README.md#task-목표의-기준-frame-779)).

| 모드 | `target[0:3]` | `target[3:6]` |
|---|---|---|
| `control_6dof=false` | TCP 위치 (x, y, z) | 영공간 관절 레퍼런스 (task 목표로는 줄 수 없다 — 6 값에 자리가 없다) |
| `control_6dof=true` | TCP 위치 (x, y, z) | 자세 (roll, pitch, yaw) |

## 파라미터

값은 `ur5e_p1a` YAML 기준이고 세 로봇이 같다. 범위를 벗어난 값은 로더 · `ros2 param` 콜백 · tick 사용 지점에서 floor 된다.

| 파라미터 | YAML 값 | 설명 |
|---|---|---|
| `kp_translation` | `[5.0, 5.0, 5.0]` | 위치 비례 게인 [1/s]. 정확히 3 원소여야 하고 아니면 configure 실패 |
| `kp_rotation` | `[2.0, 2.0, 2.0]` | 자세 비례 게인 [1/s]. 같은 길이 계약 |
| `singularity_threshold` | `0.02` | σ₀. `σ_min(J) < σ₀` 에서만 감쇠가 붙는다. 6-DOF (6×n) 기준 값이다 |
| `max_damping` | `0.05` | λ_max, 감쇠 램프의 상한 |
| ~~`damping`~~ | — | 은퇴한 키. 남아 있으면 경고 후 무시되며 `max_damping` 으로 매핑되지 않는다. 지우고 위 두 키를 쓴다 |
| `null_kp` | `0.5` | 영공간 관절 센터링 게인 (음수는 floor) |
| `enable_null_space` | `false` | 영공간 활성화 (3-DOF 모드에서만 동작) |
| `control_6dof` | `true` | 6-DOF 제어 |
| `trajectory_speed` | `0.01` m/s | TCP 병진 궤적 속도 |
| `trajectory_angular_speed` | `0.08727` rad/s | TCP 회전 궤적 속도 (6-DOF) |
| `hand_trajectory_speed` | `3.14` rad/s | 손 궤적 속도 |
| `max_traj_velocity` | `0.1` m/s | TCP 병진 속도 상한 |
| `max_traj_angular_velocity` | `0.7854` rad/s | TCP 각속도 상한 |
| `hand_max_traj_velocity` | `6.28` rad/s | 손 속도 상한 |
| `virtual_tcp_mode` / `virtual_tcp_offset` / `virtual_tcp_orientation` | `demo_shared.yaml` | Virtual TCP (아래) |
| `fsm.pi_rotation_margin` | `0.15` rad | π 근처 quintic 분할 임계, 범위 [0, π/2] (필수 키) |
| `fsm.contact_stop_*` | | contact_stop 의 필터 · guard · release 파라미터 ([demo-joint-controller.md](demo-joint-controller.md#contact_stop-손-모드)) |
| `command_type` | `"position"` | `"position"` · `"torque"` · `"pd_feedforward"` (PD 위치 + τ_ff, mujoco_sim 은 `qfrc_applied` 주입 · 중력보상 off) |

게인 업데이트 레이아웃 (17 원소): `[kp_trans*3, kp_rot*3, singularity_threshold, max_damping, null_kp, enable_null(0/1), control_6dof(0/1), traj_speed, traj_angular_speed, hand_traj_speed, max_vel, max_angular_vel, hand_max_vel]`.

## Virtual TCP

기본 제어점은 tool0 이다. Virtual TCP 를 켜면 핑거팁 기구학에서 계산한 가상 frame 을 제어점으로 쓴다. 손 관절은 arm CLIK 에서 상수로 취급되고, 손이 움직이면 제어점이 매 tick 갱신된다.

| `virtual_tcp_mode` | 설명 |
|---|---|
| `"disabled"` | tool0 을 제어점으로 쓴다 (기본) |
| `"centroid"` | 4 개 핑거팁 위치의 평균 |
| `"weighted"` | 접촉력 기반 가중 평균 (접촉 중인 핑거팁에 가중) |
| `"constant"` | `virtual_tcp_offset` 의 고정 오프셋 (TCP frame 기준) |

`centroid` · `weighted` 는 hand FK 가 필요하므로 `tree_models` 가 활성이어야 한다. `virtual_tcp_orientation` (roll, pitch, yaw [rad], TCP frame) 은 `disabled` 외 모든 모드에 적용된다. 계산 결과나 TCP pose 에 비유한 값이 섞이면 그 tick 의 갱신을 건너뛰고 직전 제어점을 유지한다.

### 제어 frame 계약

hand FK 가 핑거팁 pose 를 못 내는 tick (closed-chain walk-in, hand device invalid) 에는 제어점이 tool0 로 떨어진다. 이때 목표를 그대로 두면 두 frame 의 차이가 task error 가 되어 팔이 튄다. 그래서 목표에는 authoring 된 제어 frame 이 태깅되고, CLIK 는 그 frame 이 이번 tick 의 제어 frame 과 같을 때만 돈다.

| 상황 | 동작 |
|---|---|
| frame 일치 | 기존 CLIK 그대로 (`disabled` 는 항상 여기) |
| frame 전환 + 외부 목표 없음 | hold seed 를 새 제어점으로 다시 잡는다 — 오차 0, 팔 무동작 |
| frame 전환 + 외부 목표 | 목표를 보존한 채 팔 hold, frame 이 돌아온 tick 에 실행 |
| 외부 목표가 1000 tick 넘게 대기 | 목표 만료 + WARN, 현재 frame 으로 hold 복귀 |
| 참여 핑거팁 집합 변화 (centroid · weighted) | hold seed 재-seed, 외부 목표는 새 제어점에서 재계획 |

demo GUI 는 frame 이 확정되기 전에는 task target 입력을 비활성화한다.

### 런타임 변경 (ROS 파라미터)

세 키는 Joint · Task · Compliance 의 ROS 파라미터로 선언되고 초기값은 `demo_shared.yaml` 이다 (검증 규칙: [virtual_tcp_params.hpp](../include/integrated_bringup/support/virtual_tcp_params.hpp)).

- **Inactive 일 때만 set 된다.** Active 컨트롤러는 `successful=false` 로 거부한다. Inactive 에서 바꾼 값은 활성화 첫 tick 의 hold seed 에 반영돼 팔이 움직이지 않는다.
- **검증.** 모드는 `disabled` · `centroid` · `weighted` · `constant` 만 받고 (모르는 이름은 거부), 배열은 길이 3 의 유한값이어야 한다. hand 모델이 없는 컨트롤러는 `disabled` 가 아닌 모드로의 변경을 거부한다 (현재 모드를 다시 보내는 것은 받는다).
- 셋을 함께 바꿀 때는 `set_parameters_atomically` 로 한 번에 보낸다. GUI 의 Virtual TCP 패널이 그렇게 한다.
- CLI 의 배열은 double 로 쓴다 (`"[0, 0, 1]"` 은 정수 배열이라 거부된다).

```bash
ros2 param set /demo_task_controller/demo_task_controller virtual_tcp_offset "[0.0, 0.0, 0.15]"
```

- set 한 값은 노드 수명 동안 유지된다. 재configure 해도 남고, `demo_shared.yaml` 은 프로세스 재시작 때만 다시 읽는다.

## Closed-chain hand FK

손 URDF 가 loop closure 를 가지면 (`urdf.extended: true` + `<stem>.closure.yaml`, 예: 4-bar 링키지) loop-passive 관절 하류의 핑거팁을 tree 모델로 FK 하면 운영점을 벗어날 때 큰 오차가 난다. 이 경우 `ClosedChainHandFk` ([support/closed_chain_hand_fk.hpp](../include/integrated_bringup/support/closed_chain_hand_fk.hpp)) 가 closed-chain 에 일관된 핑거팁 pose 를 계산한다.

- `on_configure` 에서 closure 구속이 있고 핑거팁이 loop-passive 관절 하류일 때만 활성이다. 그 외 로봇은 일반 serial 경로를 쓴다. 출하 `ur5e_p1a` · `iiwa7_leap` 은 `extended` 가 없어 비활성이다.
- closure 가 있는데 closed-chain FK 가 서지 못하면 (hand-root frame 이 안 풀림 · closure ill-posed · 독립 관절이 device 에 없음) serial 경로로 넘어가고, 그 손의 device 관절이 serial tree 에 다 있지 않으면 `on_configure` 가 FAILURE 다. WARN 한 줄이 이유를 먼저 찍는다.
- 활성 중 사영이 수렴하지 않거나 특이한 tick 에는 하류 핑거팁의 직전 유효 pose 를 유지한다.

## E-STOP

안전 위치 `estop.arm_safe_position` (UR5e 는 `[0, -1.57, 1.57, -1.57, -1.57, 0]` rad) 으로 이동하고 손은 현재 위치를 유지한다.

## 로그

`task_diag.csv` 를 포함한 파일 목록은 [controller-logs.md](controller-logs.md) 를 본다.
