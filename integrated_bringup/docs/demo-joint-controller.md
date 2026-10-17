# DemoJointController

관절 공간 quintic 궤적 생성기다. 팔과 손의 목표 관절각까지 rest-to-rest quintic 으로 보간해 위치 명령으로 직접 낸다 (비례 게인 없음). 새 로봇의 bring-up 은 이 컨트롤러로 한다 — 모든 출하 로봇이 `config/<robot>/controllers/demo_joint_controller.yaml` 을 싣는다. config_key 는 `demo_joint_controller`.

공통 파라미터 · 팔 끝 frame · 손끝 FK 배선 · grasp mode 전환은 [demo-shared-params.md](demo-shared-params.md) 가 갖는다.

## 목표 입력

목표는 컨트롤러가 소유한 토픽 `/demo_joint_controller/<group>/joint_goal` (`rtc_msgs/RobotTarget`, `goal_type: joint`) 로 보낸다. task 목표 (`goal_type: task`) 는 거부한다 — x,y,z,r,p,y 를 관절 rad 로 해석하는 사고를 막는다 ([Task 목표의 기준 frame](../README.md#task-목표의-기준-frame-779)). 군은 둘 (팔 계열 하나 + 손 하나) 까지다.

## 궤적

```
duration = max(0.01, T_speed, T_velocity)
  T_speed    = max_joint_dist / trajectory_speed
  T_velocity = (15/8) * max_joint_dist / max_traj_velocity
position_output = quintic(t, q_start, q_goal, duration)
```

| 파라미터 | 출하 YAML 값 | 설명 |
|---|---|---|
| `robot_trajectory_speed` | `0.78` rad/s | 팔 궤적 속도 (duration 계산용) |
| `hand_trajectory_speed` | `3.14` rad/s | 손 궤적 속도 |
| `robot_max_traj_velocity` | `3.14` rad/s | 팔 최대 관절 속도 (peak 제한). `iiwa7_leap` 은 `1.71` |
| `hand_max_traj_velocity` | `6.28` rad/s | 손 최대 관절 속도 |
| `command_type` | `"position"` | 출력 타입 (`"position"` 또는 `"torque"`) |

게인 업데이트 레이아웃 (4 원소): `[robot_trajectory_speed, hand_trajectory_speed, robot_max_traj_velocity, hand_max_traj_velocity]`. 값은 로봇별 YAML 이 정한다 (표는 `ur5e_p1a` 기준).

## contact_stop 손 모드

핑거팁 힘이 임계를 넘으면 손 궤적 출력을 현재 위치로 동결해 과도한 closure 를 막는다 (`grasp_controller_type: "contact_stop"`).

- **동결 위치.** 측정 위치를 Bessel LPF (`fsm.contact_stop_lpf_cutoff_hz`, 20 Hz) 로 거른 값이다.
- **힘 입력.** 판정에 쓰는 `|F|` 는 핑거팁별 `fx/fy/fz` 를 축별로 Bessel LPF (`fsm.contact_stop_force_lpf_cutoff_hz`, 50 Hz) 에 통과시킨 뒤 계산한다. 앞단의 delta-spike guard 는 축별 변화가 `fsm.contact_stop_force_guard_delta_n` (4.0 N) 을 넘거나 NaN/Inf 인 샘플을 마지막 수용값으로 대체한다. 연속 hold 상한은 `fsm.contact_stop_force_guard_max_hold_ticks` (2). 근거: [controllers-rationale.md](../../docs/reference/controllers-rationale.md#힘-피드백-필터).
- **raw 는 불변.** 발행되는 `GraspState` 의 `force_magnitude` / `max_force` 와 CSV 의 `ft_<f>_fx|fy|fz` 는 driver 값 그대로다. `<device>_sensor.csv` 는 raw → `_guarded` (LPF 입력) → `_filt` (LPF 출력) → `_force_guard_rejected` 를 모두 남긴다.
- **Hold latch.** 접촉이 한 번 성립하면 동결이 latch 된다. 이후 접촉이 사라져도 release 게이트나 E-STOP 이 풀 때까지 유지한다. 접촉 중에는 LPF 출력을 따라가고, 접촉이 사라지면 마지막 LPF 출력을 고정한다.
- **Release 게이트.** 손을 여는 방향의 goal 이 내려오면 latch 를 풀고 동결을 건너뛴다. 세 조건이 모두 성립해야 한다 (ε = `fsm.contact_stop_release_eps`, 0.005 rad): `thumb_cmc_fe` 는 `target > actual + ε`, `index_mcp_fe` 와 `middle_mcp_fe` 는 `target < actual − ε`.
- **로그 태그.** `[contact_stop] FREEZE ...` (접촉 중 동결), `HOLD ...` (접촉 없이 latch 유지), `SKIP (release) ...` (게이트 통과). 판정에는 필터된 `fmax=` 를 쓰고 `fmax_raw=` 를 함께 찍는다.

## E-STOP

팔은 YAML 의 `estop.arm_safe_position` (길이가 `arm_dof` 와 일치해야 하는 필수 키) 으로 이동하고, 손은 현재 위치를 유지한다.

## 로그

`<session>/controllers/demo_joint_controller/` 아래의 파일은 [controller-logs.md](controller-logs.md) 를 본다.
