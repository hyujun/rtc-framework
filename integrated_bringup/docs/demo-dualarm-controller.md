# DemoDualArmController

두 손 frame 과제와 자세 과제를 **QP 하나**로 푸는 바인딩입니다. 첫 device group 이 tree (몸통 + 양팔) 인 로봇에서 돌고, 코어는 `rtc_tsid` 의 다중 frame `ClikReferenceGenerator` 입니다. 식은 [`clik_reference.hpp`](../../rtc_tsid/include/rtc_tsid/kinematics/clik_reference.hpp) 가 갖고, 여기에는 운용 표면만 적습니다. 동작 계약은 [헤더 주석](../include/integrated_bringup/controllers/demo_dualarm_controller.hpp) 입니다.

- **`g1_p1b` 만 YAML 을 싣습니다.** `RTC_REGISTER_CONTROLLER_REQUIRING_CONFIG` 라 다른 profile 에서는 인스턴스화되지 않습니다. `initial_controller` 는 그대로 `demo_joint_controller` 이고, 이 컨트롤러는 `/rtc_cm/switch_controller` 나 `initial_controller:=demo_dualarm_controller` 로 고릅니다 (명령은 [README](../README.md) 의 `g1_p1b` launch 절).
- **군 `g1` (waist 3 + 팔 7 × 2) 은 QP 가 position 명령으로 내고, 군 `p1b` (손) 는 관절 공간 quintic** 으로 따라갑니다. 손 관절은 QP 변수에는 있되 속도 box 1e-9 로 잠깁니다.
- **과제 둘**: `right_hand` (`catch_frame`, base `pelvis`) 와 `left_hand` (`left_rubber_hand`, base `torso_link`). 왼손 과제는 몸통 기준이라 waist 열이 0 입니다. 자세군 셋 (`waist` · `left_arm` · `right_arm`) 이 나머지를 붙듭니다.

## 설정 (`config/g1_p1b/controllers/demo_dualarm_controller.yaml`)

**모든 키가 필수**입니다 — 이 파일을 본 적 없는 로봇에 맞는 기본값이 없습니다. 값은 전부 첫 실행 전에 정한 출발값이고 튜닝하지 않았습니다.

| 키 | 단위 | 범위 · 뜻 | runtime |
|---|---|---|---|
| `command_type` | – | `position` 만 받습니다 (다른 값은 configure 거부) | YAML 전용 |
| `clik.damping_sq` · `clik.w_smooth` | – | QP 감쇠 $\mu^2$ (> 0) · 직전 속도와의 평활 가중 (>= 0) | YAML 전용 |
| `clik.qp.max_iter` | – | ProxQP 반복, 정수 [1, 1000] | YAML 전용 |
| `clik.accel_constraint` | – | `dynamic` 만 받습니다 (상대 과제는 kinematic 가속 행과 못 씁니다) | YAML 전용 |
| `clik.eta_tau` | – | 토크 행이 쓰는 `max_torque` 의 비율, (0, 1.2]. 코어에는 `tau_max` 의 곱으로 넘깁니다 | YAML 전용 |
| `clik.limit_margin` | rad | 몸통 군 위치 box 를 device 한계에서 좁히는 폭 (>= 0). 범위가 뒤집히면 configure 거부. 손은 한계 그대로. 이 띠 안에 있는 관절에서는 풀이를 시작하지 않습니다 (아래 "시드") | YAML 전용 |
| `clik.joint_velocity_max` | rad/s | device 정격 위의 상한 (> 0). 속도 box = 둘 중 작은 값 | YAML 전용 |
| `clik.brake.{enabled,margin}` | – | 토크에서 얻은 감속도로 한계 앞 속도를 묶습니다. margin (0, 1] | YAML 전용 |
| `clik.target_frames` | – | 목표가 **표현될** 수 있는 frame 이름 (<= 8). 아래 "목표의 frame" | YAML 전용 |
| `clik.tasks[]` `name` | – | 과제 이름 = 토픽 토큰 `[A-Za-z][A-Za-z0-9_]*`. <= 4 개 | YAML 전용 |
| `…` `frame` · `base_frame` · `kind` | – | 움직이는 frame · 목표 / 오차 / 행을 잴 frame (다른 frame 이어야 하고 둘 다 이름이 있어야 합니다) · `se3` 만 | YAML 전용 |
| `…` `gain_linear` · `gain_angular` | 1/s | [0, 1/dt]. 넘으면 configure 거부 | `tasks.<name>.gain_linear` · `.gain_angular` |
| `…` `weight` · `fb_lin_max` · `fb_ang_max` | – · m/s · rad/s | 스칼라 가중 (> 0) · 되먹임 상한 (>= 0, 0 = 끔) | YAML 전용 |
| `clik.posture_groups[]` | – | `name` (파라미터 토큰) · `joints` (몸통 군의 관절, 군끼리 서로소) · `weight` (>= 0). <= 8 개 | YAML 전용 |
| `…` `gain` | 1/s | [0, 1/dt] | `posture.<group>.gain` |
| `trajectory.{linear,angular,hand}_speed` | m/s · rad/s | 목표까지의 quintic 평균 속도 (> 0) | 같은 이름 |
| `trajectory.*_speed_max` | m/s · rad/s | quintic 정점 속도의 상한 (평균의 15/8 배가 이 값 안에 들 때까지 늘립니다). 병진은 직선 거리가 아니라 **기준 원점이 실제로 지나는 경로** (나선, log6 의 병진 성분) 의 길이로 잽니다 — 회전이 크면 직선보다 최대 1.5 배 깁니다 | read-only |
| `fault.max_qp_fail_ticks` | tick | 연속 풀이 실패가 이만큼이면 fault latch (정수 >= 1) | YAML 전용 |
| `fault.track_err_max` | rad | max \|q_meas − q_cmd\| 가 넘으면 같은 latch. 0 = 끔 (출하값 — 아직 잰 값이 없습니다) | YAML 전용 |

## 토픽

노드 namespace 는 `/demo_dualarm_controller/` 입니다.

| 토픽 | 내용 |
|---|---|
| `right_hand/task_goal` · `left_hand/task_goal` | `rtc_msgs/RobotTarget`, `goal_type: task`, `task_target` = [x y z roll pitch yaw] (m · rad, ZYX). 과제마다 하나 (`<name>/task_goal`). `header.frame_id` 가 목표가 표현된 frame |
| `g1/joint_goal` | 17 관절 **전부** = 자세 목표 |
| `p1b/joint_goal` | 10 관절 **전부** = 손의 quintic 목표 (device 한계로 자릅니다) |
| `transforms` | `tf2_msgs/TFMessage`, 부모 `pelvis`, **측정** pose: `base_adapter_actual` · `l_<finger>_tip_bracket_actual` 넷 · `catch_frame_actual` · `left_rubber_hand_actual`. 같은 결합 모델의 두 번째 `pinocchio::Data` 를 측정 관절 상태에서 갱신해 구합니다. 명령 pose 는 CSV 에만 있습니다 |

새 메시지 · 상태 토픽 · 스레드는 없습니다. 과제 목표는 노드의 기본 callback group (non-RT) 구독이고 depth 1 입니다.

**목표의 frame.** `header.frame_id` 는 목표를 **적은** frame 입니다. 비우면 그 과제의 base frame, `clik.target_frames` 의 이름 (`world` · `pelvis` · `torso_link`) 이면 목표를 적용하는 tick 에 base frame 으로 **한 번** 바꾸고 (`T_base,des = T_base,ref(q_c) · T_ref,des`), 그 뒤로는 base 에 고정된 pose 입니다 — base 가 움직이면 목표가 따라갑니다. 목록에 없는 이름은 base frame 으로 읽지 않고 거부하며 셉니다. 모델에서 `world` 와 `pelvis` 는 **다른 frame** 입니다 (`pelvis` 가 `world` 위 0.79186375 m, 축은 같음) — 오른손 과제의 base 가 `pelvis` 인 이유이고, 같은 물리 목표도 두 frame 에서는 z 가 다른 숫자입니다.

**거부 · 버림** (전부 `dualarm_diag.csv` 의 계수로 남습니다): `goal_type` 이 `task` 가 아니거나 6 값이 유한하지 않거나 frame 이름이 모르는 것이면 ingress 가 거부합니다. 기준을 만들 수 없는 목표는 적용 tick 에 버립니다 (`<task>_drop_unusable`): 현재 기준 pose 에서 π − 0.15 rad 를 넘게 도는 것 (log6 궤적이 불연속), 경로 길이가 overflow 해 유한하지 않은 것, 이 runtime 에 없는 frame slot. 비활성 중에 보낸 목표는 활성화 때, 풀이를 못 도는 tick (E-STOP · fault · 판독 불가 · 시드 전) 에 온 목표는 그 tick 에 버립니다 — 나중에 뒤늦게 움직이지 않도록 매 tick 소비합니다. `g1/joint_goal` 로 온 task 목표나 군의 관절을 다 싣지 않은 관절 목표도 거부하고 `group_goal_rejects` 로 셉니다.

## 풀이를 안 도는 tick

- **E-STOP**: 풀이도 궤적도 돌지 않고 두 군이 **측정 위치**를 명령합니다 (선로에는 CM 이 자기 hold 를 대신 냅니다). 해제 tick 에 측정값에서 명령을 다시 시드하고 모든 목표를 "프레임이 지금 있는 자리" 로 되돌립니다 — 이어서 가지 않습니다.
- **풀이 실패**: 직전 명령을 유지합니다 (그 tick 의 $\dot q_c$ = 0). `fault.max_qp_fail_ticks` 번 연속이면 controller-local **fault latch** — 마지막 명령을 유지하고 (손 군도 궤적을 멈추고 유지합니다) 목표를 거부하며, **`/rtc_cm/reset_fault` 로만** 풀립니다 (`ros2 service call /rtc_cm/reset_fault rtc_msgs/srv/ResetFault "{controller_name: demo_dualarm_controller}"`). E-STOP 해제나 비활성 → 활성으로는 풀리지 않고, E-STOP 중에도 reset 할 수 있습니다. 풀이가 호출 자체를 거부한 tick (입력이 유한하지 않음 등) 도 실패로 셉니다. latch 중에 활성화하거나 E-STOP 이 오가면 두 군 모두 측정값에서 다시 시드해 **그 자리를 유지**합니다 (침묵하지 않습니다). 어느 tick 도 소비하지 못한 reset 요청은 다음 활성화로 넘어가지 않습니다 — 그 활성화에서 다시 요청합니다.
- **시드** (활성화 · E-STOP 해제 · fault reset): 명령을 측정값에서 다시 잡습니다. 이때 몸통 군의 관절이 풀이의 위치 box (device 한계 − `clik.limit_margin`) **밖**에 있으면 — 다른 컨트롤러가 그 띠 안에 세워 둔 경우입니다 — 거기서는 풀이를 시작하지 않습니다. box 밖에서 시작한 풀이는 그 관절을 정지 상태에서 한 tick 만에 속도 상한으로 되돌리려 하기 때문입니다. 넘은 양이 **그 관절이 한 tick 에 갈 수 있는 거리** (`속도 box × dt`, 출하값에서 6.3 mrad) 이하면 명령을 box 경계에서 시작하고 (풀이가 경계에 세워 둔 관절이 서보 오차만큼 넘어 읽힌 경우), 그보다 크면 측정 위치를 유지한 채 **fault latch** (`fault_cause` 3) 를 걸고 ERROR 한 줄로 관절 번호와 범위를 알립니다. 관절을 `demo_joint_controller` 로 box 안으로 옮긴 뒤 reset 합니다 — 그대로 reset 하면 같은 tick 에 다시 걸립니다.
- **판독 불가**: `g1` 을 못 읽으면 풀이와 기준을 멈추고 `g1` 만 침묵시킵니다. `p1b` 를 못 읽으면 손만 침묵하고 (손 궤적은 멈춥니다) 풀이는 계속 돕니다 — 모델에 들어가는 손 자세는 마지막으로 읽힌 값에 멈춥니다. 첫 판독 가능 tick 에 시드합니다. 관절 위치 (손은 속도도) 에 유한하지 않은 값이 있는 군도 판독 불가로 봅니다 — 그 값에서 시드하거나, 그 값을 명령으로 내거나, 풀이의 모델에 넣지 않습니다. 침묵한 tick 의 `<군>_state.csv` 행에서도 goal 열은 요청된 목표 (자세 목표 · 손 목표) 입니다.
- 토크 한계가 중력 부하보다 낮은 관절은 **풀이를 실패시키지 않습니다** — 그 관절이 처지는 채로 풀리고, `brake_static_infeasible` 의 bit 가 알립니다.

## Runtime 파라미터

노드 FQN 은 `/demo_dualarm_controller/demo_dualarm_controller` 입니다. `tasks.<name>.gain_linear` · `.gain_angular` 와 `posture.<group>.gain` 은 [0, 1/dt] 밖이면 **거부**합니다 (값을 잘라 받아도 원하던 값이 아닙니다). `trajectory.linear_speed` · `angular_speed` · `hand_speed` 는 나눗수라 1e-6 에서 **하한**합니다. 어느 쪽이든 유한하지 않은 값은 거부하고, 요청 안의 하나라도 걸리면 전부 적용하지 않습니다. `trajectory.*_max` 는 read-only 입니다. cleanup → configure 를 지나도 파라미터 값은 남습니다 (이 패키지의 다른 컨트롤러와 같습니다). cleanup 중에는 검사가 없으므로 그때 들어온 값은 configure 가 다시 판정하고, 거부한 값은 설정값으로 **되써서** 노드가 돌지 않는 숫자를 보이지 않게 합니다. read-only 상한은 되쓸 수 없어, 설정이 바뀐 채 다시 configure 하면 WARN 으로 알립니다.

```bash
ros2 param set /demo_dualarm_controller/demo_dualarm_controller tasks.right_hand.gain_linear 20.0
```

## 로그와 TF 판독

`<session>/controllers/demo_dualarm_controller/{g1_state,p1b_state,dualarm_diag}.csv` — 앞 둘은 기존 `DeviceStateLog`, `dualarm_diag.csv` (`integrated_bringup/DualArmDiagLog`) 는 **매 tick 한 행**입니다. 풀이를 안 돈 tick 도 행을 쓰고 (`clik_ran = 0`, 계산 안 한 칸은 0) 그래서 이 컨트롤러가 활성인 동안의 구멍은 떨어진 행뿐입니다. `tick` 은 CM 의 loop counter 라 다른 컨트롤러가 활성이던 구간도 `tick` 의 간극으로 남습니다 — 다시 활성화된 뒤의 첫 행은 시드한 행 (`reseeded` = 1) 이거나, 아직 읽을 수 없으면 시드 전 hold (`hold` = 4) 입니다.

| 열 묶음 | 열 |
|---|---|
| 행동 | `t_relative_s` · `tick` · `hold` (0 풀이 돎, 1 E-STOP, 2 fault, 3 몸통 판독 불가, 4 시드 전, 5 모델 없음) · `clik_ran` · `estop` · `fault_latched` · `fault_cause` (0 없음, 1 연속 실패, 2 추종 오차, 3 시드가 box 밖) · `body_readable` · `hand_readable` · `reseeded` |
| 풀이 (안 돈 tick 은 0) | `reached_solve` · `converged` · `rejected_input` · `non_finite` · `command_mismatch` · `accel_rows_violated` · `brake_box_empty` · `status` · `iterations` · `solve_time_us` · `accel_rows` · `accel_rows_binding` · `fb_saturated` (bit 2k = 과제 k 병진, 2k+1 = 회전) · `rot_near_pi` (bit k) · `brake_active` · `brake_static_infeasible` (bit = 모델 속도 index) |
| 감시 | `qp_fail_streak` · `track_err` (max \|q_meas − q_cmd\|, 몸통 관절) · `group_goal_rejects` |
| 과제마다 `<task>_` | `valid` · `traj_active` · `goal_sequence` (0 = 시작 자세 유지) · `err_lin` · `err_ang` · `ref_{x,y,z,qw,qx,qy,qz}` · `cmd_*` · `meas_valid` · `meas_*` · `goals_accepted` · `reject_goal_type` · `reject_non_finite` · `reject_unknown_frame` · `drop_stale` · `drop_unusable` · `drop_held` |
| 몸통 관절 | `q_cmd_<joint>` (device 순서) |

세 pose 는 모두 **그 과제의 base frame** 입니다. `ref` = 이 tick 에 풀이가 받은 기준 $T^d(t)$, `cmd` = 풀이가 평가된 **명령 상태** (직전 tick 이 남긴 명령) 의 frame, `meas` = 이 tick 의 **측정** 관절 상태의 frame. `err_*` 는 `ref` 대 `cmd` — 풀이가 되먹임한 오차 그대로이고 **서보 지연은 들어 있지 않습니다**. 지연은 `cmd` 대 `meas` 를 시간을 밀어 맞대어 봅니다. 거부 · 버림 계수 (`reject_*` · `drop_*`) 는 누적입니다. 그림과 통계는 `ros2 run rtc_tools plot_rtc_log <그 CSV>` (`--stats` 로 통계만) — 과제 · 관절은 열 이름에서 읽으므로 그 run 의 YAML 이 필요 없습니다.

## 코어 기본형과 다른 점

식은 [`clik_reference.hpp`](../../rtc_tsid/include/rtc_tsid/kinematics/clik_reference.hpp) 와 [`mpc_segment_core.hpp`](../../rtc_controllers/include/rtc_controllers/catching/mpc_segment_core.hpp) 가 갖는다. 이 바인딩이 다르게 한 곳:

| 항목 | 이 컨트롤러 | 이유 |
|---|---|---|
| QP 변수 | 27 개 (몸통 17 + 손 10), 손은 속도 box 1e-9 로 잠금 | 다른 데모 컨트롤러와 같은 모델 cache 배선을 쓰기 위해서 |
| 오른손 base | `world` 가 아니라 `pelvis` | 축이 world 와 같아 행은 같고 목표의 z 만 0.79 m 다름 |
| 왼손 base | `torso_link` | 몸통 기준 과제는 waist 열이 0 이라 몸통을 밀지 않음 |
| 기준 | 외부 정지 pose 로 가는 task-space quintic (MPC 없음) | 관절 기준을 만드는 MPC 가 이 컨트롤러에 없음 |
| 자세 기준 | 정지한 목표 ($\dot q_{ref}$ 없음) | 자세군은 움직이는 궤적이 아니라 붙들기 용도 |
| 가중 | 두 손 1.0 / 1.0, 자세 1e-3 · 1e-4 (튜닝하지 않은 출발값) | 두 과제가 열을 공유하지 않아 손 사이 우선순위는 결과에 무관 |
| 목표 frame | `frame_id` 가 base 와 다르면 적용하는 tick 에 한 번 변환 | 이후 base 가 움직이면 목표가 따라가도록 |
| 이득 상한 | $1/h$ 를 YAML · runtime 파라미터 · tick 에서 각각 검사 | 진동 없는 조건 $Kh\le1$ |
| supervisor · 충돌 damper | 없음 (QP 실패 연속 시 controller-local fault 만) | 이 단계의 범위 밖 |
| 자세 항 | $K_q>0$ 이라 정상점에 과제 오차가 남음 ($K_q=0$ 에서만 거의 0) | 자세 항이 과제를 끌 수 있음 |

E-STOP 은 sim 에서 걸 수단이 없어 gtest 로만 확인한다 ([docs/testing.md](../../docs/testing.md)).
