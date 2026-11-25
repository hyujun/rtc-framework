# LocomotionController — 부동 베이스 전신 QP + MPC reference, `pd_feedforward` + 자기 PD 게인

부동 베이스 휴머노이드의 전신 컨트롤러다. 매 tick 측정 상태 → full 모델 kinematics (손 포함) → reduced 모델 dynamics (손을 neutral 에 잠근 질량) → 전신 QP (`rtc_tsid` `LocomotionFormulationBuilder`) → 본체 군에 `pd_feedforward` (위치 목표 + QP 토크) 를 낸다 — 위치 목표는 `pd_feedforward.target` 이 고른다: `plan` (reference 자세, 출하) 또는 `wbc` (QP 의 관절 가속도를 적분한 상태, 헤더 "The servo target"). 손 군은 `kPosition` hold 한다. reference 는 `mpc.enabled` 면 **MPC (`rtc_mpc` kinodynamic) 의 plan** 을 tick 마다 샘플한 것이고, 아니면 측정 자세를 latch 한 고정 reference 다. 속도 명령 (Twist) 은 MPC 의 planner 로 들어가고, 상태는 `DiagnosticStatus` 토픽으로 나온다 (아래). 보행 자체의 게이트 · 평가는 이후 feature 다. 계약 · 상태 전이 · fault 원인의 SSoT 는 헤더 `include/integrated_bringup/controllers/locomotion_controller.hpp` 다.

## 쓰는 법 (`g1_loco` · `b7_loco`)

시작 컨트롤러는 `demo_joint_controller` 다. 로봇이 서 있는 상태에서 전환한다:

```bash
ros2 launch integrated_bringup sim_g1_loco.launch.py enable_viewer:=false              # latch 자세
ros2 launch integrated_bringup sim_g1_loco.launch.py enable_viewer:=false enable_mpc:=true  # MPC
ros2 service call /rtc_cm/switch_controller rtc_msgs/srv/SwitchController \
  "{activate_controllers: ['locomotion_controller'], deactivate_controllers: [], strictness: 1}"
ros2 topic echo --once /rtc_cm/active_controller_name      # locomotion_controller
ros2 service call /rtc_cm/reset_fault rtc_msgs/srv/ResetFault "{controller_name: 'locomotion_controller'}"
```

- 활성화 뒤 첫 유효 tick 의 측정 자세가 reference 다 (속도 · 가속 0, 접촉 wrench 는 그 자세의 정적 분배). MPC 가 켜져 있으면 쓸 수 있는 plan 이 오는 대로 stand 로 넘어간다 (아래).
- `enable_mpc` (기본 `false`) 가 CPU layout 과 `locomotion_controller.mpc.enabled` 를 함께 고른다. `enable_mpc:=false` layout 에서 `mpc.enabled: true` 면 활성화를 거부한다 (MPC 코어가 shield 밖).
- 이 컨트롤러는 joint goal 을 받지 않는다 — 실행 중 보낸 goal 은 거부된다.
- PD 게인은 wire 로 sim 에 실리고 **sticky** 다: 다른 컨트롤러로 돌아가도 그 게인이 남는다 (`rtc_mujoco_sim` README `pd_feedforward`).

## 속도 명령과 상태 토픽 (`mpc.enabled`)

```bash
ros2 topic pub -r 20 /locomotion_controller/g1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {z: 0.0}}"       # v_x · v_y · 높이 rate · yaw rate
ros2 topic echo /locomotion_controller/g1/locomotion_status      # level · source · plan · 명령

# 게임패드로 (sim 을 command_input:=joy 로 띄운 뒤): LB 를 누른 동안만 움직인다
ros2 launch rtc_teleop_input teleop_input.launch.py joy_topic:=/locomotion_controller/g1/joy \
  controller_name:=locomotion_controller status_topics:=/locomotion_controller/g1/locomotion_status
```

- `runtime.yaml` 의 `velocity_command: {topic}` 가 Twist 토픽이다 (출하 `g1/cmd_vel`, 컨트롤러 네임스페이스 아래). `linear.x` / `linear.y` / `angular.z` = v_x / v_y / yaw rate, `linear.z` = pelvis 높이 **rate** — `command.yaml` 의 필터 (clamp · Bessel · 적분, `rtc_mpc` `reference/command_filter.hpp`) 를 지나 planner 의 stage reference 가 된다. 구독은 depth 1 · reliability 기본 (reliable) 이라 best_effort 발행자는 연결되지 않는다. 키가 없으면 명령 경로가 없고 MPC 는 제자리에 선다; `mpc.enabled` 가 아니면 키는 받되 비활성이다.
- `velocity_command.type: joy` (기본 `twist`, configure 때 하나) 면 같은 키의 `topic` 이 `sensor_msgs/Joy` (표준 게임패드 배치, [rtc_teleop_input](../../rtc_teleop_input/README.md)) 이고 `joy: {linear_x, linear_y, angular_z, height_rate?, deadman}` 가 배치 이름으로 축을 고른다 — 축 [−1, 1] × `command.v_max` (높이는 `hdot_max`). deadman 버튼이 정확히 1 일 때만 움직이고, 놓으면 0 (stale 이 아닌 새 명령), 배열이 짧거나 매핑한 축이 비유한이면 메시지 전체가 0 이고 `cmd_joy_rejects` 가 는다. 그 뒤는 Twist 와 같은 필터 · stale 경로다. g1_loco 의 출하 매핑은 sim launch `command_input:=joy` (`sim_overlays/joy_command.yaml`): 왼스틱 = v_x · v_y, 오른스틱 X = yaw, LB = deadman, 높이 없음. 스키마: `controllers/locomotion/joy_command.hpp`.
- 명령은 `command.t_stale` (0.5 s) 동안 새 메시지가 없으면 0 으로 돌아간다 (높이는 유지) — 발행자는 **주기적으로** 보낸다. 비유한 축은 직전 값을 유지한다. 활성화마다 필터가 리셋되어 높이는 `command.base_height` 로 돌아간다. 필터는 활성 동안만 돈다 (`command.sample_rate_hz`).
- 지금의 FSM 은 **모든 접촉이 지지일 때만** stand 로 들어간다 — 속도 명령으로 gait 가 걸음으로 바뀌면 plan 의 mask 에 swing 이 생겨 walk 로 표시된다. 출하값의 보행은 sim 판정을 아직 통과하지 못했다 — 첫 걸음 전후에 plan 을 잃고, 로봇이 움직이는 중이라 plan-loss fault 로 멈춘다 (아래 평가 도구의 `walk` arm 이 재현한다). 명령은 sim 에서만 쓴다.
- `<body group>/locomotion_status` (`diagnostic_msgs/DiagnosticStatus`, ≈ 50 Hz, MPC 없이도 발행) — `level`: OK (stand · walk, 또는 MPC 없이 nominal) · WARN (waiting · degraded · MPC 가 있는데 idle) · ERROR (fault · E-STOP); `message` = reference 출처; `values` 는 고정 키 (순서 · 이름은 `controllers/locomotion/locomotion_status.hpp` 의 `LocomotionStatusKey`): source · mode · fault_cause · estop · tick_count · plan_id · policy_age_ms · converged · contact_mask · schedule_epoch · state_epoch · plausibility_rejects · plan_holds · solves · failed_solves · deadline_miss · withheld · overrun · wbc_tick_us · qp_iterations · cmd_vx · cmd_vy · cmd_wz · cmd_height · cmd_valid · cmd_fresh · cmd_stale_ticks · cmd_nonfinite_ticks · cmd_ignored_axis_ticks (필터 tick 수) · source_status (상태 source 판정 이름) · estimator_resets · estimator_reset_reason · estimator_contact_mask (아래 §상태 source; truth 에서는 0) · cmd_input (0 twist · 1 joy) · cmd_deadman (joy 1 / 0 / 읽지 못함 −1, twist −1) · cmd_joy_rejects. MPC 카운터는 1 Hz 로 갱신된다. GUI (W8) 는 키로 읽는다.

## 모드

| 모드 | 언제 | 출력 |
|---|---|---|
| waiting | latch 된 reference 가 없음 — 활성화 뒤 아직 유효 상태가 없거나, **E-STOP 중**이거나, 자세 가드가 측정 자세를 거부하는 동안 | 감쇠 (아래); QP · plan · MPC 상태 쓰기 없음, 로그 · 상태 토픽은 매 tick |
| nominal | QP 가 풀림 | 위치 = reference, feedforward = QP τ |
| degraded | QP 실패 또는 짧은 상태 불량 | 직전 유효 τ 유지 |
| fault (latch) | QP health Fault · reference 가 latch 된 채 상태 불량이 `fault.max_source_bad_ticks` 초과, latch 된 채 추정이 다시 시작 (state epoch 변화), 또는 추정이 `init.timeout_s` 안에 시작하지 못함 (모두 `source`) · 출력 비유한 · 모델 epoch 불일치 · 정지 양발 지지가 아닌 채 plan 상실 (`plan_loss`) · 자세 가드가 0.5 s 내내 거부 (`pose_guard`) | 감쇠: feedforward = −`damping_kd`·v (토크 한계로 clamp), 위치 = 측정 q, 게인 불변 |

## reference 출처 (`mpc.enabled` 일 때)

| 출처 | 언제 | reference |
|---|---|---|
| idle | MPC 없음 · 쓸 수 있는 plan 없음 | latch 자세 (위) |
| stand | plan 이 쓸 수 있고 (유한 · 현재 model / state epoch · `max_policy_age_s` 안 · 타당성 통과) 모든 접촉이 지지 | plan 의 q · v · a · 접촉 wrench 를 블록 그대로 |
| walk | stand 와 같고 접촉 하나가 swing | 같은 복사 + 그 접촉을 QP 에서 뺀다 |
| fault | 아래 latch | 없음 |

- 미수렴 해도 publish 됐으면 stand 한다 (publish 규칙이 잔차를 묶는다 — cold solve 는 수렴해야 publish, 그 다음 solve 는 소비 node 잔차로). stand · walk → idle 은 plan 을 **잃었을 때만** 이다 (나이 · epoch · 타당성). 그때 모든 접촉 frame 이 최저점 + 15 mm 안이고 base 가 0.1 m/s 아래면 측정 자세를 다시 latch 하고, 아니면 fault (`plan_loss`) — 떠 있는 발에 하중을 주지 않기 위해서다.
- 타당성 검사 (`mpc.sanity`) 에 걸린 plan 은 fault 가 아니라 idle 이고, 그 plan 은 끝까지 다시 쓰지 않는다.
- 단, stand · walk 중에 걸린 plan 은 **그 앞에 따르던 plan** 에 자리를 내준다 — 그 plan 이 아직 살아 있고 (epoch · `max_policy_age_s`) 그 tick 의 검사도 통과하면 계속 따른다. 다음 타당한 plan 이 오면 넘어가고, 앞 plan 이 나이로 빠지거나 다시 걸리면 위의 plan 상실 규칙대로다 (되돌아간 plan 의 앞은 없다). plan 이 늦게 올 때 허용하는 범위와 같다. idle → stand 의 조건은 바뀌지 않는다 (idle 은 아무 plan 도 따르지 않으므로 잡을 앞 plan 이 없고, idle 이 표본만 한 plan 은 잡지 않는다). 횟수는 `plan_holds` (로그 열 · 상태 키 · 10 s INFO 의 `held`).
- reset · 활성화 · E-STOP 두 edge 는 MPC 의 상태 stream (state epoch) 과 gait 를 다시 시작한다 — 옛 stream 의 plan 은 다시 샘플되지 않는다. E-STOP 동안은 stand 로 넘어가지 않는다.
- 모든 solve 는 한 예산이다 (standing 전용 예산 없음). 첫 (cold) solve 는 보통 publish 되지 않고 (미수렴), 그 다음 solve 가 첫 plan 이 된다 — 그때까지 idle (latch 자세). publish deadline 을 못 맞추는 호스트에서는 stand 에 이르지 못한다 — 예산은 제어 PC 의 soak 가 정한다.

latch 는 `/rtc_cm/reset_fault` 나 **재활성화**로 풀린다 (다음 tick 에 reference 를 다시 잡는다). 원인이 남아 있으면 다시 latch 된다. E-STOP 의 두 edge 도 reference 를 다시 잡는다.

## E-STOP 과 자세 가드

- **E-STOP 중** (CM 의 글로벌 래치 — 검출기 또는 `/rtc_cm/trigger_estop`): tick 은 상태를 읽고 모델을 갱신하고 로그 · 상태 토픽을 쓰지만 reference 를 latch 하지 않고, QP 도 plan 샘플도 MPC 로의 상태 쓰기도 하지 않는다 — 모드 `waiting`, 출력은 감쇠 (wire 에는 CM 의 hold 가 나간다). CM hold 는 게인을 싣지 않으므로 sim 은 이 컨트롤러의 **저게인**을 그대로 유지한다 — hold 아래 로봇이 천천히 주저앉는 것은 의도된 동작이다 (`run_arms.py --arms estop` 이 잰다). 해제 뒤 어느 컨트롤러가 뛰는지는 CM 의 `estop_on_clear` 가 정한다 (`rtc_controller_manager` README "운영자 E-STOP"); 두 humanoid sim 은 `demo_joint_controller` 로 돌아가고, `resume` 구성이면 이 컨트롤러가 아래 가드를 지나 재진입한다.
- E-STOP 전에 latch 된 fault 는 E-STOP 을 지나도 남는다 (clear 는 reset 이 아니다); E-STOP 중의 reset 은 latch 를 풀고 `waiting` 을 유지하며, 가드는 clear 뒤 첫 유효 tick 에 돈다. 비활성 중 온 E-STOP · reset 요청은 활성화가 소비한다 (거짓 재시작 없음).
- **자세 가드** (`fault.pose_guard`): reference 를 latch 하는 모든 입구 — 활성화 · reset · E-STOP 해제 · plan 상실 — 에서 측정 자세를 먼저 판정한다: base lane 이 유효 샘플이고, base 쿼터니언 · 속도 · 접촉 높이가 유한하며, 쿼터니언이 단위이고, 몸통 기울기 (R(2,2) = 1 − 2(x² + y²)) 가 `base_tilt_rad` 안. 통과 → 그 tick 에 latch (활성화 첫 tick 이 nominal 인 것은 그대로). 실패 → `waiting` (감쇠) 으로 매 유효 tick 다시 묻고, **0.5 s** (코드 상수, tick 수 = round(0.5 · control_rate)) 의 거부 뒤 `pose_guard` fault — 넘어진 로봇은 세우지 않고 잡아 둔다. 시계는 입구마다 0 에서 시작하고 E-STOP 중에는 돌지 않는다. 유효 상태가 없는 tick 은 판정이 아니다 (시계 없이 `waiting`). plan 상실 입구는 여기에 기존 상수 둘 (접촉 높이 퍼짐 ≤ 15 mm · base 속도 < 0.1 m/s) 을 더해 **즉시** `plan_loss` 다 (떠 있는 발은 기다릴 수 없다). 판정 비트는 `pose_guard_mask` (로그 열 · 상태 키; 0 = 통과) — `logging/locomotion_diag_log_pod.hpp` 가 가리키는 `controllers/locomotion/pose_guard.hpp` 의 비트. 출하값 30° (`0.5236`) 는 두 profile 공통 (b7 은 잠정); 각속도 · 높이 · 속도는 판정하지 않는다 (무릎 꿇은 자세는 빈틈 — sim 결과로 추가를 판단).
- 손 hold 는 몸의 reference 와 무관하게 **첫 readable tick** 에 latch 된다 (E-STOP · `waiting` 중에도 손은 밀린 자리를 따라가지 않는다); 몸의 latch 가 손 목표를 옮기지 않는다. 입구 (활성화 · reset · E-STOP edge) 마다 손도 다시 latch 한다.
- Walk 중 plan 상실은 감쇠 fault 뿐이다 (soft collapse) — 착지 fallback 은 이후 feature.

## 상태 source (`estimator.source`)

configure 때 정하고 실행 중에는 바꾸지 않는다.

- `truth` (출하 기본) — sim 의 base · 관절 lane 을 그대로 읽는다.
- `inekf` — base 를 본체 군의 `model.imu.primary_site` IMU 와 다리 기구학으로 추정한다 (접촉 보조 InEKF + 접촉 FSM). sim 의 base lane 은 읽지 않는다. 관절은 truth 와 같이 encoder 값이다. 규칙 전부 (시작 · cadence · skew · reset · 출력 사상) 는 `controllers/locomotion/inekf_state_source.hpp` 머리 주석에 있다.
  - **시작** — 정지 샘플 (`‖ω‖` · `|‖a‖ − g|` · 관절 속도가 tolerance 안, 계획 접촉 ≥ 1) 이 `init.window_ticks` 개 연속이면 시작한다. 출하값 1 이면 활성화 tick 에 시작하고 그 tick 에 latch 한다. 시작 전에는 `waiting` (`source_status = not_initialized`) 이고, `init.timeout_s` 안에 시작하지 못하면 `source` fault 다. 이 시계는 E-STOP 중에는 돌지 않고 reset · E-STOP edge 마다 0 에서 다시 시작한다.
  - **cadence** — IMU sequence 가 같으면 base 를 그대로 두고 (fresh bit 꺼짐) `policy.max_hold_ticks` 까지만 유효하다. 한 번에 여러 샘플을 놓치면 공칭 주기로 k 배 propagate 한다. 20 ms 를 넘는 공백이나 sequence 역행은 reset 이다. joint · IMU 수신 시각 차가 `max_joint_imu_skew_ticks` 를 넘으면 그 tick 접촉 update 를 생략하고, `max_skew_invalid_ticks` 를 넘으면 상태가 무효다.
  - **reset** — 공백 · FSM / 필터 불일치 · 연속 거부 · 연속 gating · 확정 접촉 없음 중 하나 (우선순위 그 순서) 가 일어나면 추정을 다시 시작하고 state epoch 를 +1 한다. 컨트롤러는 epoch 가 바뀌면 MPC 상태 stream 을 다시 시작한다 (옛 상태의 plan 은 다시 샘플하지 않는다). reference 가 latch 돼 있었으면 그 tick 에 `source` fault 를 건다 — 따라가던 base 가 점프했기 때문이다. latch 전이면 다시 시작하기를 기다린다.
  - **추정의 접촉 prior** 는 직전 tick 의 접촉 mask 다 (활성화 · MPC stream 재시작 직후는 모든 접촉). 추정한 접촉 mode 는 telemetry 뿐이며, WBC 의 접촉 활성 · plan 을 바꾸지 않는다.
  - **gauge** — 시작 순간 지지 접촉점들의 평균이 world 원점이고, heading 은 `init.yaw` 다. 이 둘은 이후 관측되지 않는다.

## YAML (`config/<robot>/controllers/locomotion_controller.yaml` + `locomotion/`)

| 조각 | 키 |
|---|---|
| `model.yaml` | `model.*` — 본체 군 · 손 군 · 접촉 · 몸통 · 팔 끝 frame |
| `runtime.yaml` | `command_type: pd_feedforward` · `pd_feedforward {kp, kd}` (본체 roster 순) · `fault {max_source_bad_ticks, damping_kd, pose_guard {base_tilt_rad}}` ((0, π/2], 필수) · `estimator {source: truth | inekf, body_max_age_ms, hand_max_age_ms, inekf {imu_frame, imu, core, fsm, init, policy}}` (`inekf` 는 `source: inekf` 면 필수이고, 있으면 source 와 무관하게 검사한다. `imu.sample_period_s` = 1 / control_rate 이어야 하고, `imu_frame` 은 root 관절에 붙은 frame 이어야 한다. schema 는 `controllers/locomotion/inekf_source_config.hpp`) · `velocity_command {topic}` (선택 — Twist 명령) · `topics` (본체 → 손 순) · `logs` |
| `whole_body_qp.yaml` | `whole_body_qp.*` — task · 제약 · `health {max_invalid_ticks, recover_valid_ticks}` · `com` task (수평 CoM 고정) |
| `mpc.yaml` | `mpc.*` — 소비자 키 `enabled` · `period_nodes` · `max_policy_age_s` · `riccati {enabled, …}` · `sanity {joint_delta_rad, base_speed, wrench_scale}` (`mpc:` 가 있으면 전부 필수, 의미는 헤더) · `sync {mode: free_run \| lockstep, delivery_ticks, wait_timeout_s}` (선택; 아래) + OCP (`kinodynamic`) · `solver` |
| `command.yaml` · `gait.yaml` · `swing.yaml` · `reference.yaml` | `mpc.enabled` 일 때 MPC 의 stage reference 로 조립 — `reference.q_default` 의 손 관절은 MPC 동안 손의 hold 자세 |

`b7_loco` 의 값은 `g1_loco` 의 것을 옮긴 잠정값이다 (`mpc:` 없음 = MPC 없음). `estimator.inekf` 절은 `g1_loco` 에만 있다.

**MPC lock-step** (`mpc.sync.mode: lockstep`, sim 전용 — `sim_g1_loco` launch 의 **기본** (`mpc_sync`, `free_run` 으로 끈다; 출하 YAML 은 실기용 free_run)): MPC thread 가 벽시계로 혼자 돌지 않고 RT tick 이 N = `period_nodes`·dt / tick 마다 solve 를 요청하고 (N 은 정수여야 한다) L = `delivery_ticks` tick 뒤 (0 = N) 그 solve 를 기다린 뒤 plan 을 채택한다 — plan 의 나이가 호스트 속도와 무관하게 정확히 L tick 이고, sim 은 tick 을 기다리므로 rtf 가 solve 를 따라 내려간다. controller manager 가 sim lock-step 이 아니면 (`cm_sim_time_sync` 없음 · false) configure 거부 (실기는 항상 free_run). 대기가 `wait_timeout_s` 를 넘으면 fault `mpc_wait`. free_run 에서는 publish deadline 이 막던 solve 를 lock-step 에서는 세기만 한다. 명령 필터도 벽시계 timer 가 아니라 RT tick 이 K = tick rate / `command.sample_rate_hz` tick 마다 step 한다 (K 정수 아니면 거부; stale 판정도 tick 시계) — planner 는 trigger tick 에 그 출력을 읽는다. diag 꼬리 열 `mpc_sync_lockstep` · `mpc_trigger_tick` · `mpc_wait_us` (`wbc_tick_us` 는 이 대기를 뺀 값) · `mpc_wait_count` (활성화마다 0 부터) · `mpc_pending_superseded`. sim 쪽은 `sync_timeout_armed_ms` (launch 가 4 s 로 두고 `wait_timeout_s` 가 그 아래인지 검사).

## 로그와 판정

`<session>/controllers/locomotion_controller/` 에 `locomotion_diag.csv` (모드 · fault 원인 · `pose_guard_mask` · 상태 판정 · QP 결과 · margin · base · CoM · reference 출처 · plan id · policy age · 수렴 · 접촉 mask · 타당성 거부 수 · state epoch · 발 · 접촉력 · 관절별 servo 몫, 열 계약은 `logging/locomotion_diag_log_pod.hpp`) · `joint_ff.csv` · 군별 `<group>_state.csv` 가 쓰인다.

```bash
ros2 run rtc_tools plot_locomotion_diag <session> --z-ref 0.764431 --stance-csv stance.csv
```

그림 7 장과 `locomotion_diag_summary.json` (MPC 열이 있으면 stand 비율 · `time_to_stand_s` · policy age 분포 · plan 의 base z 평균) 은 `rtc_tools` README §locomotion_diag 를 본다. MPC thread 의 tick 시간은 `<session>/timing/mpc_timing_log.csv`, solve · 거부 카운터는 10 s 마다 INFO 한 줄 (누적) 이다.

보행 판정 열 (끝에 붙는다):

- `cmd_vx` · `cmd_vy` · `cmd_wz` · `cmd_h` · `cmd_valid` — 명령 필터의 출력을 timer 가 마지막으로 넘긴 값 (planner 에 준 명령; 표본 plan 이 풀린 명령보다 필터 주기 + MPC 지연만큼 새롭다). `cmd_valid` 는 그 step 의 입력이 신선했는지 (명령 경로가 없거나 stale 이면 0).
- `gait_latest` — timer 가 읽은 phase manager 의 gait (`gait.sequence` 의 위치, −1 = MPC 없음). 스케줄러가 앞서 나간 gait 라 표본 plan 의 `contact_mask` 보다 앞선다 — 이벤트 정렬은 `contact_mask` 로 한다.
- `ref_base_vx/y/z` — 표본 plan 의 base 선속도 (base frame, plan 이 없으면 NaN).
- `plan_holds` — 걸려서 앞에 따르던 plan 에 자리를 내준 plan 수 (누적).
- `sample_*` — 그 tick 에 표본한 plan, **타당성 판정 전** (걸린 plan 도 남는다; `plan_id` 는 따른 plan 이라 다를 수 있다): `sample_plan_id` (0 = 표본 없음) · `sample_converged` · `sample_iterations` · `sample_prim_inf` · `sample_dual_inf` · `sample_prim_head` (앞 node 3 개의 잔차 max) · `sample_prim_node` (잔차가 가장 큰 node) · `sample_mask` · `sample_progress` · `sample_a_joint_max` · `sample_a_joint_idx` (body 관절 \|a_ref\| 의 max 와 그 관절, body 순서) · `sample_check` (걸린 검사 번호, 0 = 통과 또는 검사 안 함).
- `est_*` · `imu_*` — 상태 source 의 telemetry 다 (`controllers/locomotion/estimator_telemetry.hpp`). 두 부분으로 나뉜다.
  - **입력 열**: `imu_*` · `est_imu_sequence` · `est_skew_us` · `est_planned_mask` · 끝에 붙인 `est_imu_valid` · `est_imu_age_us` (아래 Log tiers). 두 source 가 같은 읽기 (`ReadImuSite` · `RecordEstimatorInputs`) 로 채운다. 그래서 truth 로 돌린 run 도 offline 재생이 된다.
  - **나머지**: 추정기 자신의 기록이다. truth 에서는 기본값이다.

  열은 다음과 같다.
  - 시작 · reset: `est_initialized` · `est_reset_reason` (1 요청 · 2 공백 · 3 불일치 · 4 거부 · 5 gating · 6 접촉 없음) · `est_reset_count`
  - cadence: `est_imu_sequence` · `est_periods` (k; 0 = hold, −1 = step 없음) · `est_skew_us` (관절 수신 − IMU 수신, IMU 표본이 유효할 때만, 아니면 0) · `est_hold_ticks`
  - step 결과: `est_step_status` · `est_core_status` · `est_report_valid`
  - 접촉 mask: `est_planned_mask` (그 tick 의 prior, 모델 접촉 bit 만) · `est_mask` (stance · unknown 인 접촉)
  - 계수기: `est_no_anchor_ticks` · `est_gated_ticks` · `est_refused_ticks` · `est_unobservable`
  - 재생용 raw IMU: `imu_gx … imu_az` (primary site 의 원 표본, 무효여도 그대로; site 가 없으면 0)
  - 접촉별: `est_mode_<c>` · `est_reason_<c>` · `est_update_<c>` · `est_evidence_<c>` · `est_speed_<c>` · `est_m2_<c>`
  - 공분산 대각: `est_p_<i>` (15 + 3·접촉 수, 오차 순서 φ · v · p · d · b_g · b_a)
- `meas_contact_valid` — bit c 는 본체 군 inference lane 의 group c (접촉 순서) 에 대한 backend 자신의 신선도 판정 (`inference_enable` — sim 은 연속 누락 tick 수) 이다. group 이 stale 이거나 group 수 ≠ 접촉 수거나 `sensor_layout.inference_values_per_group` < 4 면 0. 수신 나이 (`inference_recv_steady_ns`) 는 보지 않는다. 그 lane 의 힘 값은 이 파일에 없다 — 출하 profile 에서 그 lane 은 sim 의 발바닥 센서라 참값 파일 (`truth_grf_<c>_*`) 에 있다. lane 의 소스 순서는 profile 이 접촉 순서로 적는다 (`sim.yaml` 의 `fingertip_wrench_topics`).

### Log tiers

실기에서도 얻는 값과 sim 에서만 얻는 참값은 **다른 파일**이다.

- **R tier** — `locomotion_diag.csv` (위). sim 에서만 얻는 값은 이 파일에 없다. 끝에 붙인 열 (기존 열은 `test_locomotion_diag_contract` 이 고정한다):
  - `state_source` — 0 = truth lane 복사, 1 = 추정.
  - `wrench_des_<c>_{fx,fy,fz,tx,ty,tz}` — 그 tick 에 WBC QP 가 받은 접촉 wrench 목표 (`lambda_des`: stand · walk 는 plan 의 MPC wrench, idle 은 지렛대 규칙). world 축 · contact frame 원점 · 바닥→발. QP 가 돌지 않은 tick (waiting · E-STOP · 이미 latch 된 fault · degraded hold) 은 NaN — QP 가 돌고 실패한 tick 은 받은 목표를 남긴다. 점 접촉 (dim 3) 의 모멘트도 NaN.
  - `servo_kp_<j>` · `servo_kd_<j>` — `tau_servo_<j>` 의 두 항 kp (q_cmd − q) 와 −kd q̇ (합은 반올림 차이까지 `tau_servo`). damping tick 의 kp 항은 0.
  - `mpc_last_*` — MPC thread 가 마지막으로 끝낸 solve 시도 (`rtc_mpc` `thread/solve_record.hpp`): `seq` (이 thread run 의 기록 수, 0 = 없음) · `outcome` (−1 없음 · 0 published · 1 withheld · 2 deadline · 3 cold withheld · 4 failed) · `err` · `iterations` · `converged` · `initial` · `cold` · `prim_inf` · `consumed_prim` (publish 규칙이 보는 소비 구간 잔차) · `compute_us` · `state_seq` · `state_epoch` · `plan_id` (published 일 때). WBC 가 따르는 plan (`plan_id` · `sample_*`) 보다 새로울 수 있다. 두 시도가 한 tick 사이에 끝날 수 있어 `seq` 는 건너뛸 수 있다. 재활성화 (thread 는 pause 됐다 이어진다) 직후에는 새 시도가 끝날 때까지 앞 활성화의 마지막 기록이 남는다 — `state_epoch` · `plan_id` 로 가른다.
  - `est_imu_valid` · `est_imu_age_us` — 재생 입력의 나머지 (위 `est_*` 의 입력 열).
    - `est_imu_valid`: site 가 있고 producer 가 유효로 표시했고 수신 stamp 가 있고 6 값이 유한하면 1 이다. sequence 역행은 추정기 자신의 판정이라 여기에 들어가지 않는다.
    - `est_imu_age_us`: now − 수신이다. `now` 뒤에 찍힌 수신은 0, 받은 적 없거나 site 가 없으면 −1 이다.
    - 재생은 IMU 수신 = now − `est_imu_age_us` 와 관절 수신 = IMU 수신 + `est_skew_us` 로 lane 시각을 되살린다. truth 의 `source_age_us` 는 base 시각과 섞여 있어 이 용도로 쓸 수 없다.
- **T tier** — `locomotion_truth.csv` (`LocomotionTruthLog`, 열 계약은 `logging/locomotion_truth_log_pod.hpp`, 채움은 `controllers/locomotion/truth_log.hpp`). 같은 tick 에 쓰고 `tick` 열로 잇는다.
  - 행은 참값 lane 이 하나라도 신선한 tick 에만 쓰고 (`truth_mask` bit 0 = base, bit 1 + c = 접촉 c), 파일은 첫 행에서 열린다 (`rtc::LogOpen::kOnFirstRow`) — 참값 lane 이 없는 run (실기) 에는 파일이 없다. 0 으로 채운 열은 없고, 신선하지 않은 lane 은 NaN 이다.
  - 신선도: base 는 `IsBaseFresh`, 접촉은 `IsSensorGroupFresh` (backend 판정 + 수신 나이), 둘 다 `estimator.body_max_age_ms`. 나이 (`truth_base_age_us` · `truth_age_<c>_us`, −1 = 수신 없음) 와 sequence 는 늘 쓴다.
  - `truth_base_*` — base lane 의 **raw** 값 (quaternion 을 정규화하지 않는다; R 의 `base_q*` 는 상태 source 의 정규화 사본이고 inekf 에서는 추정값이다). twist 는 base LOCAL.
  - `truth_grf_<c>_*` — 몸통 군 접촉 lane 의 wrench 를 **바닥→발** 로 뒤집은 값 (diag 의 `meas_contact_valid` 는 backend 판정만, T 의 bit 는 나이도 본다), sole sensor site 축 · site 원점 모멘트. 모멘트는 몸통 군 `sensor_layout.has_native_moment` (sim.yaml) 가 없으면 NaN. G1 sim 의 sole sensor 는 world body 접촉만 센다 (`body2="world"`) — 다른 발 · 다리 위에만 얹힌 발은 0 이다.
  - `truth_cop_<c>_x/y` — site 축의 CoP (x = −τ_y / f_z, y = τ_x / f_z). f_z 가 체중의 3 % 이하면 NaN.
  - `truth_ext_{fx,fy,fz,tx,ty,tz}` · `truth_ext_age_us` — sim 이 로봇의 한 body 에 걸고 있다고 보고한 외력 wrench (push). 컨트롤러의 `external_wrench.topic` 이 빈 값 (출하) 이면 구독도 이 열들도 없다. world 축 · 요청이 준 점 기준 · 환경이 로봇에 주는 방향, 받은 값 그대로다. `truth_mask` 의 bit 5 (접촉 bit 들 위, 접촉 수와 무관한 자리) 는 마지막 보고가 `external_wrench.max_age_ms` 보다 젊을 때 선다 — 걸린 힘이 없으면 0 과 bit 1, 아는 것이 없으면 NaN 과 bit 0 이다. 제어 법칙은 이 값을 읽지 않는다. 발행 쪽은 `rtc_mujoco_sim` 의 `external_wrench_report.*` (body 하나, 50 Hz + 서비스 호출 즉시). 적용점 (body 좌표) 은 보고에 없으므로 시나리오를 돌린 쪽이 기록한다.
  - 몸통 군 접촉 lane 을 참값으로 보는 근거는 "G1 · b7 실기에는 발 F/T 가 없다" 는 규칙이다 — 실기 발 센서가 생기면 이 분류를 다시 정한다.

## 평가 도구 (`tools/standing_eval/`)

- `gain_step.py` — 매달린 로봇 (base 고정) 에서 관절마다 0.2 rad step: 2 s 도달률 · overshoot 로 PD 게인 후보를 거른다 (venv python, mujoco).
- `run_arms.py` — sim 을 run 마다 새로 띄워 같은 시간축 (기동 → settle → 전환 단계 → lead → recorder) 으로 `servo` (MJCF servo) · `loco` (전환) · `lowpd` (demo_joint 에 같은 PD 게인, feedforward 없음) · `mpc` (`enable_mpc:=true` 로 전환) · `mpc_fb` (+ `mpc_riccati` overlay — `g1_loco` 는 Riccati 를 출하에서 켜므로 `mpc` 와 같다) · `reentry` (실기의 복귀 절차: loco 정지 창 → `/rtc_cm/trigger_estop` → hold → `/rtc_cm/clear_estop` (demo_joint 복귀 확인) → demo_joint 에 서는 자세 goal (`estop.arm_safe_position`) → `/sim/reset` 으로 로봇을 세움 → loco 재진입 창, 녹음 하나에 창 둘) · `estop` (loco 중 `/rtc_cm/trigger_estop` → hold → `/rtc_cm/clear_estop` → demo_joint 복귀 확인 → loco 전환, 이벤트 시각 · 창 지표는 `run.json`; 두 arm 모두 rtf 1 로 — clear 의 검증 기한이 wall 이라 느린 rtf 에서는 unverified 로 끝난다) · `walk` (MPC 를 켠 채 전환 → 명령 스크립트 `twist_script.py` (`--script`, 기본 `walk` = ramp 0 → 0.5 m/s 2 s · hold 8 s · yaw 0.3 rad/s 4 s · hold 2 s · 정지 10 s, sim 초; `turn` · `curve` 와 그 `_slow` 는 `walk_metrics.COMMAND_SCRIPTS` 의 제자리 회전 · 좌우 회전 보행, `back` · `side` 는 후진 0.3 m/s · 왼쪽 횡보행 0.2 m/s 로 길이 · 정지가 같고 `walk_gate.py` 는 그 run 을 거부한다 (판정은 `walk` 만); `--cmd-topic` 에 20 Hz wall, 컨트롤러의 구독이 붙은 뒤 시작) → 마지막 `--stop-window` s 가 정지 창) · `walk_estop` (같은 스크립트의 `--estop-at` s 에 `/rtc_cm/trigger_estop` → hold 창, clear 없음) 를 반복하고 (`--mpc-layout` 이면 모든 arm 이 같은 MPC layout, `loco` · `reentry` · `estop` 은 `mpc_off` overlay, `walk` · `walk_estop` 은 언제나 MPC; `--mpc-threads N` 이면 MPC layout 으로 뜨는 arm 이 `layout_profile:=mpc_par_sim mpc_threads:=N` 으로 N thread solve; `--overlay ARM=…` · `--launch-arg` 로 바꾼다 — 한 arm 에 `--overlay` 를 여러 번 주면 `<out>/overlays/<arm>.yaml` 한 파일로 합친다; 외란은 `--push T DUR FX FY FZ --push-body <body>` (명령 스크립트의 T sim 초에 DUR sim 초 동안 world 축 힘, 스크립트가 자기 시계로 `/sim/set_external_wrench` 를 부른다 — 참값 로그에 찍으려면 `push_report` overlay 를 함께) 와 `--friction MU` (모든 접촉의 미끄럼 마찰, simulator 의 contact override 조각을 각 arm 의 overlay 에 합침) 이고 `run.json` 의 `scenario` 에 남는다; `--source inekf` 면 모든 arm 이 `estimator_inekf` overlay 로 추정 base 위에서 돈다 — overlay 를 이미 쓰는 arm 은 둘을 `<out>/overlays/<arm>_inekf.yaml` 한 파일로 합쳐 돈다 (같은 키를 다른 값으로 쓰면 시작 전에 거부)) recorder CSV/JSON · 로그 세션을 모은다. `repo_scripts/scripts/with_verify_hold.sh` 아래에서 돌린다. `run.json` 의 `provenance` 는 그 run 의 출처다: checkout 의 commit · dirty, profile config 의 hash 둘 (launch 가 읽은 설치본과 checkout — 다르면 빌드가 checkout 보다 옛것), overlay 파일의 경로 · hash 와 그 옆 `<overlay>.meta.json`, 제어 URDF 대 sim 이 실제로 읽은 MJCF 의 `model_match` (`rtc_tools` — 총질량 · CoM · 합성 관성의 차). profile 은 `--launch` 이름에서 읽고 (`sim_<profile>.launch.py`) `--profile` 로 바꾼다. `--mpc-sync` (기본 `lockstep` = sim launch 의 기본; 모든 arm 에 `mpc_sync:=` 로 전달, `run.json` 의 `mpc_sync`) 가 `lockstep` 이면 settle · lead · 이벤트 sleep · 녹음 길이를 wall × rtf 가 아니라 sim 시간 (`/sim/time`) 으로 세고 (명령 스크립트 `--clock sim`, recorder `--sim-time-topic` 과 `sim_t` 열) — rtf 는 solve 시간을 따라가므로 `--rtf` 의 고정 환산은 쓰지 않는다; 대기의 wall 상한은 sim 초 / `--min-rtf` (기본 0.1) 다. `free_run` 이 옛 wall × rtf 방식.
- `walk_gate.py` — `run_arms.py` 의 `--out` 디렉토리에서 `walk_<k>` · `walk_estop_<k>` 를 판정한다 (`walk_gate.json`): 보행 지표는 `rtc_tools.analysis.walk_metrics` (스케줄러와 같은 yaw 정렬 속도 · 구간은 로그의 `cmd_vx` 열로 · 접촉 에지 1:1 짝짓기 · 체중은 그 run 의 stand 구간 — 접촉 항목은 참값 파일 `locomotion_truth.csv` 의 접촉력으로 계산하므로 그 파일이 없는 run 은 접촉 항목에서 실패한다), 정지 창은 `mpc_<k>` (대조군) 의 같은 길이 꼬리와 비교, `walk_estop` 은 edge (E-STOP 열 유지 · waiting · fault 불변 · CM 거부 줄 0) 를 판정하고 hold 아래 낙하를 보고한다. 문턱은 파일 머리의 상수고, rtf · `z_ref` · 정지 창 길이는 각 run 의 `run.json` 에서 읽는다 (run 끼리 다르면 거부). 추종 · heading 창은 `--gait-yaml` 의 walk 주기 (마지막 switching time) 의 정수배 (`walk_metrics` 의 `*_CYCLES`) 라 gait 를 바꾸면 따라간다 — 창이 명령 스크립트의 구간보다 길어지는 gait 는 판정 전에 거부한다. `--probe` 는 `free_run` run 에서만 뜻이 있는 옛 rtf 선택 기준 (나이 거부 0 · stand 뒤 plan 없는 row 0 · policy age p99 ≤ 135 ms — 시간 예산 3 node + 1 주기 (0.175 s) 에서 40 ms 뺀 값으로, 보류 허용치인 출하 `max_policy_age_s` (0.35 s) 와는 따로다) 만 본다 — 그 기준이 고른 rtf 0.7 은 wall 70 ms 주기의 MPC 가 sim 초당 1.43 배 풀게 해 실시간의 plan 상실을 가렸다 (#1168 S4). 성능 (solve 시간 · overrun · `wbc_tick_us`) 은 보고만 한다 — 판정은 제어 PC 다. `run.json` 의 `mpc_sync` 가 `lockstep` 이면 정지 창은 recorder 의 `sim_t` 로 자르고, rtf 가 solve 를 따라가므로 `--probe` 는 적용되지 않는다 (출력 후 0 으로 끝남). `walk_eval` 의 run 유효성은 그 run 에서 rtf 허용폭 검사 대신 lock-step 불변식 셋 — 새 plan 첫 행의 `policy_age_us` 가 정확히 L × tick (G1 500 Hz 에서 70000), `mpc_trigger_tick` 간격이 정확히 N = 35 tick, 컨트롤러 카운터의 `refused_age` 0 — 을 본다.
- 튜닝 batch 의 판정은 `rtc_tools` `walk_eval` 이다 — `run_arms.py` 출력 디렉토리들을 arm 으로 묶는 spec (run glob · arm 의 overlay · 대조 arm · 유효성 기준) 을 받아 tier 선언 지표 (`<이름>_truth` 는 참값 파일, `<이름>_est` 는 컨트롤러가 쓴 상태, 접미사 없는 것은 명령 · 계획 · solve 열) · run 유효성 (recorder · rtf · 명령 모양 · 한 발 지지 시간 · overlay · `provenance` 의 세 항목 — arm 마다 `validity` 로 덮어쓸 수 있어, `rtc_tools` `perturb_mjcf` 로 만든 섭동 plant 의 arm 은 모델 차 허용폭을 그 arm 에만 선언한다) · arm 요약 · 완주 수의 악화 금지 검정 · 순위 비용을 낸다. 정의는 모듈 docstring.
  - 발 간격 (`foot_gap_min_est` · `foot_gap_narrow_fraction_est`) 은 heading 축 좌우의 두 접촉 frame 간격이다. G1 sim 의 sole sensor 가 바닥 접촉만 세므로 발끼리 닿은 것은 참값 lane 에 보이지 않고, 이 지표가 그것을 대신 본다. 비율의 문턱은 spec 의 `foot_width_m` (sole 폭) 이고, 없으면 비율은 거부된다.
- `rtc_tools` 의 `weight_overlay` — MPC 가중치 sweep 용 `sim_overlay:=` 파일을 쓴다: group 정의 (가중치 leaf 묶음, 관절 이름으로 지정) 와 group 별 theta 로 `shipped × 10**theta` 를 쓰고 (theta 0 인 group 은 키를 쓰지 않는다 — 전부 0 이면 출하 설정 그대로), 옆에 `<overlay>.meta.json` 을 남긴다. 없는 키 · 관절, 두 group 에 속한 leaf, gauge group 의 theta ≠ 0 은 거부하고, `--merge` 로 다른 overlay 와 병합한다 (같은 leaf 의 다른 값은 거부). 검증은 `test_locomotion_mpc_weight_overlay` 다.

- `estimator_replay` (설치 실행 파일, `controllers/locomotion/estimator_replay.hpp`) — 세션의 diag · `joint_ff` 를 `tick` 으로 잇고 행마다 source 가 읽은 `ControllerState` 를 다시 만든다. 그것을 컨트롤러와 같은 `InekfStateSource::Update` 에 넣는다.
  - 정책은 cadence · hold · gap · skew · 시작 gate · reset 까지 그대로 탄다. 모델도 `BuildBodyModel` 로 컨트롤러의 것과 bit 동일하다.
  - 시계는 합성한다: now = 상수 + tick · T. 재생에 닿는 것은 stamp 의 차이뿐이다.
  - IMU 수신은 now − `est_imu_age_us` 로 되살린다.
  - 관절 수신은 IMU 가 유효하면 IMU 수신 + `est_skew_us` (정확) 로, 아니면 now − `source_age_us` (근사, 수를 센다) 로 되살린다.
  - `tick_count` = 1 인 행 (활성화) 과 첫 행에서 `Reset()` 한다.
  - 로그가 9 자리라 실로그 재생은 online 과 bit 동일하지 않다.
    - double 입력의 재생은 bit 동일하다 (`test_estimator_replay`).
    - rig 의 CSV 왕복은 base 차 ≤ 1e-6 이다.
  - `--joint-sigma σ [--joint-seed n]` 은 source 가 읽는 관절 위치에 seed 고정 white noise N(0, σ²) 를 더한다 (sim 의 encoder 는 정확하다). truth 접촉 속도는 기록된 관절로 계산한다. 기본 0 은 bit 동일이고, 값은 sidecar 에 남는다. 그 재생의 NIS 는 보고만 한다.
  - 지표는 `rtc_tools` `estimator_metrics` 가 계산한다.
  - `g1_loco` 의 `estimator.inekf` noise 값은 이 재생으로 맞춘 것이다. `imu.accel_sigma` 와 `core.contact_noise_density` 는 sim 이 넣는 noise 가 아니라 기록된 보행에서 상태 공분산이 참값 오차와 맞도록 고른 값이다 (W8-F05 #1155 재적합, 근거는 `runtime.yaml` 의 주석).
    - 맞춘 녹음의 sim 은 IMU 의 step 평균을 낸다 (`imu_rate_export: tick_mean`). 순간값 export (`sample`) 의 녹음이면 가속도계 오차가 보행 중 주입 noise 의 수십 배라 같은 값이 맞지 않는다.
    - 값을 바꿀 때는 `--overlay` 로 같은 세션들을 다시 재생해 truth run 의 NEES 를 본다. 정지에서는 이 값이 보수적이다 (NEES 가 1 보다 작다).
  - `estimator_metrics` 는 run 을 첫 계획 lift-off 에서 보행 · 정지 부분으로 나눠 NEES 를 따로 평균하고, 보행 부분의 z gyro bias 추정을 그 run 의 측정 offset 과 비교한다. 활성화 뒤 reset 수 (활성화 자신의 reset 은 빼고) 와 발마다의 slip 순환도 센다. 순환은 0.5 s 창의 slip 진입이 12 회 이상인 상태가 1.0 s 이상 이어진 것이다 (8 회 이상의 길이는 보고). 모두 fault · 낙하 (cut) 전 행만 센다. inekf 로 돈 run 은 기록된 online 추정의 같은 수 (`online_source`) 도 낸다.
- `estimator_gate.py` — 한 batch 의 상태 추정기 판정이다. truth arm (대조) 과 inekf arm (후보) 의 보행 · 정지 run 에, 각 run 디렉토리의 `replay.csv` (드라이버가 `estimator_replay` 로 만든다) 를 함께 읽는다. spec 은 script 마다 `walk_eval` spec (arm 이름의 `.` 앞이 `truth` · `inekf`) 과 정지 arm 이다. 결과는 `estimator_gate.json` 이고 PASS 일 때만 exit 0 이다. 정의는 모듈 docstring 이다 (WE-F06, WD-356 ~ 362).
  - open loop: 규칙은 파일 머리에 있고, 수치는 baseline (truth 재생) 에서 규칙으로 낸다 (`--derive`). 연속 문턱은 ⌈1.5 × baseline 통계⌉ (1e-3 단위) 이다. 같은 조건의 뒤 batch 를 위한 회귀 막대라 baseline 자신이 지나는 것은 정의상 자명하다. baseline 에서는 보행 부분 ANEES 의 χ² 구간 (기록만, 밖이면 뒤 G1 = inekf 재생 ANEES ≤ 1.5 × baseline) · 정지 ANEES 와 ANIS 의 상한 · run 마다 tilt · yaw · 속도 rms 와 걸음당 drift 의 cap 0.05 · 보행 run 의 gyro bias 오차 ≤ 3e-3 rad/s · 원거리 recall ≥ 0.90 · precision ≥ 0.99 · slip false positive ≤ 0.005 · 순환 0 을 판정한다. WE-F06 baseline (#1150, truth 재생 보행 41 · 정지 5 run) 의 등록값은 `walk_vel_rms_median` 0.006 · `walk_vel_rms_max` 0.016 · `walk_tilt_rms_max` 0.012 · `walk_yaw_rms_max` 0.03 · `walk_drift_per_step_max` 0.004 · `stand_vel_rms_max` 0.002 (m/s · rad · m) 이다. 보행 ANEES baseline 1.591 은 χ² 구간 밖이라 기록만 하고, 뒤 G1 상한은 2.3865 이다 (`anees_walk_g1`, 등록값으로 판정할 때만). 등록 뒤에는 `--derive` 없이 판정하고, 결과의 `registered_equals_derived` 가 등록값이 그 batch 의 규칙 값과 같은지 (baseline 에서 참) 알린다.
  - (b) 정지 (inekf run 마다): 카운터 0 (낙하 · fault 행 · source 비정상 행 · 활성화 뒤 reset · QP infeasible 행 · failed solve · epoch / dim 거부 · Walk 행), Stand 비율 ≥ 0.9. source 비정상 중 lane 이 늦게 온 상태 (unreadable · missing · stale) 는 host 의 센서 정지 (dev PC 에서 ≈ 0.34 s) 로 truth run 에도 있어 (대조 68 run 중 12, 각 1 행) 따로 세어 보고한다. 연속값은 recorder 의 slip · drift · tilt 와 참값 높이의 plan 대비 (log 의 마지막 20 s 의 Stand 행) 를 `T ≤ C + tol` 과 상한 (`walk_gate.py` 의 정지 창 표, C = truth run 최댓값) 으로 본다. online = 재생 (아래), 재생 속도 rms ≤ 정지 문턱, cap, 순환 0 도 본다.
  - (c) 보행 (`walk` script): 검정 셋이 각 α 0.05 / 3 이다 — 완주의 단측 Fisher, 첫 lift-off 부터의 생존 (script 끝에서 절단, 적분 끝은 가장 이른 절단 시각) 의 RMST, `v_ratio_truth` 중앙값. 뒤 둘은 permutation 10 000 회 (seed 고정, `rtc_tools` `permutation_stats`) 이고, 추종비는 창이 있는 run 이 arm 마다 12 이상이어야 판정한다 (못 미치면 UNDECIDABLE). 하나라도 기각이면 FAIL 이다. inekf run 마다 결정론 항목도 본다 — cut 전 활성화 뒤 reset 0 · source fault 0 · online 추정의 slip 순환 0 · online = 재생 (WD-305 허용폭, 비교가 cut 전 행을 다 덮을 것 — 9 자리 로그 때문에 cut 뒤의 이산 분기는 세지 않는다) · 창이 있는 run 의 |Δ추종비| ≤ 0.02 · |Δ횡속도| ≤ 0.01 m/s.
  - 보고만: 검정의 검출력 · fault 원인 · 착지 수 · cut 전 1 s 의 online 속도 오차 · inekf 재생 지표와 cap · estimator Update 시간 · 다른 script 의 완주.

dev PC 의 보행 평가는 출하 MPC 값 그대로 (overlay 없이) 2 thread 로, sim launch 의 기본인 **MPC lock-step** (`mpc_sync:=lockstep`, 위 설정 절) 에서 돈다: RT tick 이 solve 를 trigger 하고 L tick 뒤에 그 plan 을 쓰므로 plan 나이는 PC 속도와 무관하게 L × tick 이고, sim 의 rtf 는 solve 시간을 따라 내려간다 (`--rtf` 는 상한일 뿐이라 고를 필요가 없다 — 옛 `walk_gate.py --probe` 의 rtf 선택은 은퇴). 그래서 sim 결과는 알고리즘의 것이고 solve 시간 · 주기 여유는 제어 PC 의 soak 에서 따로 판정한다. `--mpc-sync free_run` 은 실기의 시계 (wall clock MPC thread) 를 sim 에 그대로 두는 옛 방식으로, 그때는 rtf < 1 이 같은 wall 지연을 R 배 작게 보이게 하고 느린 PC 에서는 withheld solve 가 `max_policy_age_s` 를 넘어 plan 이 끊긴다:

```bash
repo_scripts/scripts/with_verify_hold.sh python3 integrated_bringup/tools/standing_eval/run_arms.py \
  --out /tmp/walk --arms mpc walk walk_estop -k 3 --mpc-layout --mpc-threads 2 \
  --lead 6 --duration 26 --runtime-yaml integrated_bringup/config/g1_loco/controllers/locomotion/runtime.yaml \
  --logging-root ~/ros2_ws/rtc_ws/logging_data --launch-arg max_log_sessions:=200
python3 integrated_bringup/tools/standing_eval/walk_gate.py /tmp/walk \
  --gait-yaml integrated_bringup/config/g1_loco/controllers/locomotion/gait.yaml
```
