# LocomotionController — 부동 베이스 전신 QP + MPC reference, `pd_feedforward` + 자기 PD 게인

부동 베이스 휴머노이드의 전신 컨트롤러다. 매 tick 측정 상태 → full 모델 kinematics (손 포함) → reduced 모델 dynamics (손을 neutral 에 잠근 질량) → 전신 QP (`rtc_tsid` `LocomotionFormulationBuilder`) → 본체 군에 `pd_feedforward` (위치 목표 + QP 토크) 를 낸다. 손 군은 `kPosition` hold 한다. reference 는 `mpc.enabled` 면 **MPC (`rtc_mpc` kinodynamic) 의 plan** 을 tick 마다 샘플한 것이고, 아니면 측정 자세를 latch 한 고정 reference 다. 속도 명령 (Twist) 은 MPC 의 planner 로 들어가고, 상태는 `DiagnosticStatus` 토픽으로 나온다 (아래). 보행 자체의 게이트 · 평가는 이후 feature 다. 계약 · 상태 전이 · fault 원인의 SSoT 는 헤더 `include/integrated_bringup/controllers/locomotion_controller.hpp` 다.

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
```

- `runtime.yaml` 의 `velocity_command: {topic}` 가 Twist 토픽이다 (출하 `g1/cmd_vel`, 컨트롤러 네임스페이스 아래). `linear.x` / `linear.y` / `angular.z` = v_x / v_y / yaw rate, `linear.z` = pelvis 높이 **rate** — `command.yaml` 의 필터 (clamp · Bessel · 적분, `rtc_mpc` `reference/command_filter.hpp`) 를 지나 planner 의 stage reference 가 된다. 구독은 depth 1 · reliability 기본 (reliable) 이라 best_effort 발행자는 연결되지 않는다. 키가 없으면 명령 경로가 없고 MPC 는 제자리에 선다; `mpc.enabled` 가 아니면 키는 받되 비활성이다.
- 명령은 `command.t_stale` (0.5 s) 동안 새 메시지가 없으면 0 으로 돌아간다 (높이는 유지) — 발행자는 **주기적으로** 보낸다. 비유한 축은 직전 값을 유지한다. 활성화마다 필터가 리셋되어 높이는 `command.base_height` 로 돌아간다. 필터는 활성 동안만 돈다 (`command.sample_rate_hz`).
- 지금의 FSM 은 **모든 접촉이 지지일 때만** stand 로 들어간다 — 속도 명령으로 gait 가 걸음으로 바뀌면 plan 의 mask 에 swing 이 생겨 walk 로 표시된다. 출하값의 보행은 sim 판정을 아직 통과하지 못했다 — 첫 걸음 전후에 plan 을 잃고, 로봇이 움직이는 중이라 plan-loss fault 로 멈춘다 (아래 평가 도구의 `walk` arm 이 재현한다). 명령은 sim 에서만 쓴다.
- `<body group>/locomotion_status` (`diagnostic_msgs/DiagnosticStatus`, ≈ 50 Hz, MPC 없이도 발행) — `level`: OK (stand · walk, 또는 MPC 없이 nominal) · WARN (waiting · degraded · MPC 가 있는데 idle) · ERROR (fault · E-STOP); `message` = reference 출처; `values` 는 고정 키 (순서 · 이름은 `controllers/locomotion/locomotion_status.hpp` 의 `LocomotionStatusKey`): source · mode · fault_cause · estop · tick_count · plan_id · policy_age_ms · converged · initial_solve · contact_mask · schedule_epoch · state_epoch · plausibility_rejects · plan_holds · solves · failed_solves · deadline_miss · withheld · overrun · wbc_tick_us · qp_iterations · cmd_vx · cmd_vy · cmd_wz · cmd_height · cmd_valid · cmd_fresh · cmd_stale_ticks · cmd_nonfinite_ticks · cmd_ignored_axis_ticks (필터 tick 수) · source_status (상태 source 판정 이름) · estimator_resets · estimator_reset_reason · estimator_contact_mask (아래 §상태 source; truth 에서는 0). MPC 카운터는 1 Hz 로 갱신된다. GUI (W8) 는 키로 읽는다.

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
| stand | plan 이 쓸 수 있고 (유한 · 현재 model / state epoch · `max_policy_age_s` 안 · 타당성 통과) 수렴하지 않은 standing 해가 아니며 모든 접촉이 지지 | plan 의 q · v · a · 접촉 wrench 를 블록 그대로 |
| walk | stand 와 같고 접촉 하나가 swing | 같은 복사 + 그 접촉을 QP 에서 뺀다 |
| fault | 아래 latch | 없음 |

- online 해는 미수렴이어도 stand 를 유지한다 (publish 규칙이 잔차를 묶는다). stand · walk → idle 은 plan 을 **잃었을 때만** 이다 (나이 · epoch · 타당성). 그때 모든 접촉 frame 이 최저점 + 15 mm 안이고 base 가 0.1 m/s 아래면 측정 자세를 다시 latch 하고, 아니면 fault (`plan_loss`) — 떠 있는 발에 하중을 주지 않기 위해서다.
- 타당성 검사 (`mpc.sanity`) 에 걸린 plan 은 fault 가 아니라 idle 이고, 그 plan 은 끝까지 다시 쓰지 않는다.
- 단, stand · walk 중에 걸린 plan 은 **그 앞에 따르던 plan** 에 자리를 내준다 — 그 plan 이 아직 살아 있고 (epoch · `max_policy_age_s`) 그 tick 의 검사도 통과하면 계속 따른다. 다음 타당한 plan 이 오면 넘어가고, 앞 plan 이 나이로 빠지거나 다시 걸리면 위의 plan 상실 규칙대로다 (되돌아간 plan 의 앞은 없다). plan 이 늦게 올 때 허용하는 범위와 같다. idle → stand 의 조건은 바뀌지 않는다 (idle 은 아무 plan 도 따르지 않으므로 잡을 앞 plan 이 없고, idle 이 표본만 한 plan 은 잡지 않는다). 횟수는 `plan_holds` (로그 열 · 상태 키 · 10 s INFO 의 `held`).
- reset · 활성화 · E-STOP 두 edge 는 MPC 의 상태 stream (state epoch) 과 gait 를 다시 시작한다 — 옛 stream 의 plan 은 다시 샘플되지 않는다. E-STOP 동안은 stand 로 넘어가지 않는다.
- standing 해 (첫 solve) 는 수 초 걸릴 수 있고 그동안은 idle (latch 자세) 이다. online solve 가 publish deadline 을 못 맞추는 호스트에서는 stand 에 이르지 못한다 — 예산은 제어 PC 의 soak 가 정한다.

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
| `mpc.yaml` | `mpc.*` — 소비자 키 `enabled` · `period_nodes` · `max_policy_age_s` · `riccati {enabled, …}` · `sanity {joint_delta_rad, base_speed, wrench_scale}` (`mpc:` 가 있으면 전부 필수, 의미는 헤더) + OCP (`kinodynamic`) · `solver` |
| `command.yaml` · `gait.yaml` · `swing.yaml` · `reference.yaml` | `mpc.enabled` 일 때 MPC 의 stage reference 로 조립 — `reference.q_default` 의 손 관절은 MPC 동안 손의 hold 자세 |

`b7_loco` 의 값은 `g1_loco` 의 것을 옮긴 잠정값이다 (`mpc:` 없음 = MPC 없음). `estimator.inekf` 절은 `g1_loco` 에만 있다.

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
- `est_*` · `imu_*` — 상태 source 의 telemetry 다 (`controllers/locomotion/estimator_telemetry.hpp`; truth 에서는 0). 열은 다음과 같다.
  - 시작 · reset: `est_initialized` · `est_reset_reason` (1 요청 · 2 공백 · 3 불일치 · 4 거부 · 5 gating · 6 접촉 없음) · `est_reset_count`
  - cadence: `est_imu_sequence` · `est_periods` (k; 0 = hold, −1 = step 없음) · `est_skew_us` · `est_hold_ticks`
  - step 결과: `est_step_status` · `est_core_status` · `est_report_valid`
  - 접촉 mask: `est_planned_mask` · `est_mask` (stance · unknown 인 접촉)
  - 계수기: `est_no_anchor_ticks` · `est_gated_ticks` · `est_refused_ticks` · `est_unobservable`
  - 재생용 raw IMU: `imu_gx … imu_az`
  - 접촉별: `est_mode_<c>` · `est_reason_<c>` · `est_update_<c>` · `est_evidence_<c>` · `est_speed_<c>` · `est_m2_<c>`
  - 공분산 대각: `est_p_<i>` (15 + 3·접촉 수, 오차 순서 φ · v · p · d · b_g · b_a)
- `meas_contact_valid` · `meas_fz_<contact>` — 본체 군 inference lane 의 group c (접촉 순서) 의 fz (센서 frame, 지지하는 발은 음수). bit c 는 backend 자신의 신선도 판정 (`inference_enable` — sim 은 연속 누락 tick 수) 이고, 0 이면 (그 group 이 stale, group 수 ≠ 접촉 수, `sensor_layout.inference_values_per_group` < 4) NaN — backend 가 stale 로 본 유지값은 기록하지 않는다. 수신 나이 (`inference_recv_steady_ns`) 는 보지 않는다. lane 의 소스 순서는 profile 이 접촉 순서로 적는다 (`sim.yaml` 의 `fingertip_wrench_topics`).

### Log tiers

실기에서도 얻는 값과 sim 에서만 얻는 참값은 **다른 파일**이다.

- **R tier** — `locomotion_diag.csv` (위). 예외는 `meas_fz_*` 하나다: G1 · b7 에는 발 F/T 가 없어 실기에서는 NaN 인 sim 전용 열이지만, 이름 · 자리는 그대로 둔다. 끝에 붙인 열 (기존 열은 `test_locomotion_diag_golden` 이 고정한다):
  - `state_source` — 0 = truth lane 복사, 1 = 추정.
  - `wrench_des_<c>_{fx,fy,fz,tx,ty,tz}` — 그 tick 에 WBC QP 가 받은 접촉 wrench 목표 (`lambda_des`: stand · walk 는 plan 의 MPC wrench, idle 은 지렛대 규칙). world 축 · contact frame 원점 · 바닥→발. QP 가 돌지 않은 tick (waiting · E-STOP · fault · degraded hold) 은 NaN, 점 접촉 (dim 3) 의 모멘트도 NaN.
  - `servo_kp_<j>` · `servo_kd_<j>` — `tau_servo_<j>` 의 두 항 kp (q_cmd − q) 와 −kd q̇ (합은 반올림 차이까지 `tau_servo`). damping tick 의 kp 항은 0.
  - `mpc_last_*` — MPC thread 가 마지막으로 끝낸 solve 시도 (`rtc_mpc` `thread/solve_record.hpp`): `seq` (이 thread run 의 기록 수, 0 = 없음) · `outcome` (−1 없음 · 0 published · 1 withheld · 2 deadline · 3 cold withheld · 4 failed) · `err` · `iterations` · `converged` · `initial` · `cold` · `prim_inf` · `consumed_prim` (publish 규칙이 보는 소비 구간 잔차) · `compute_us` · `state_seq` · `state_epoch` · `plan_id` (published 일 때). WBC 가 따르는 plan (`plan_id` · `sample_*`) 보다 새로울 수 있다. 두 시도가 한 tick 사이에 끝날 수 있어 `seq` 는 건너뛸 수 있다.
- **T tier** — `locomotion_truth.csv` (`LocomotionTruthLog`, 열 계약은 `logging/locomotion_truth_log_pod.hpp`, 채움은 `controllers/locomotion/truth_log.hpp`). 같은 tick 에 쓰고 `tick` 열로 잇는다.
  - 행은 참값 lane 이 하나라도 신선한 tick 에만 쓰고 (`truth_mask` bit 0 = base, bit 1 + c = 접촉 c), 파일은 첫 행에서 열린다 (`rtc::LogOpen::kOnFirstRow`) — 참값 lane 이 없는 run (실기) 에는 파일이 없다. 0 으로 채운 열은 없고, 신선하지 않은 lane 은 NaN 이다.
  - 신선도: base 는 `IsBaseFresh`, 접촉은 `IsSensorGroupFresh` (backend 판정 + 수신 나이), 둘 다 `estimator.body_max_age_ms`. 나이 (`truth_base_age_us` · `truth_age_<c>_us`, −1 = 수신 없음) 와 sequence 는 늘 쓴다.
  - `truth_base_*` — base lane 의 **raw** 값 (quaternion 을 정규화하지 않는다; R 의 `base_q*` 는 상태 source 의 정규화 사본이고 inekf 에서는 추정값이다). twist 는 base LOCAL.
  - `truth_grf_<c>_*` — 몸통 군 접촉 lane 의 wrench 를 **바닥→발** 로 뒤집은 값 (두 열이 다 쓰인 tick 에서 `truth_grf_<c>_fz` = −`meas_fz_<c>` — diag 는 backend 판정만, T 는 나이도 본다), sole sensor site 축 · site 원점 모멘트. 모멘트는 몸통 군 `sensor_layout.has_native_moment` (sim.yaml) 가 없으면 NaN.
  - `truth_cop_<c>_x/y` — site 축의 CoP (x = −τ_y / f_z, y = τ_x / f_z). f_z 가 체중의 3 % 이하면 NaN.
  - 몸통 군 접촉 lane 을 참값으로 보는 근거는 "G1 · b7 실기에는 발 F/T 가 없다" 는 규칙이다 — 실기 발 센서가 생기면 이 분류를 다시 정한다.

## 평가 도구 (`tools/standing_eval/`)

- `gain_step.py` — 매달린 로봇 (base 고정) 에서 관절마다 0.2 rad step: 2 s 도달률 · overshoot 로 PD 게인 후보를 거른다 (venv python, mujoco).
- `run_arms.py` — sim 을 run 마다 새로 띄워 같은 시간축 (기동 → settle → 전환 단계 → lead → recorder) 으로 `servo` (MJCF servo) · `loco` (전환) · `lowpd` (demo_joint 에 같은 PD 게인, feedforward 없음) · `mpc` (`enable_mpc:=true` 로 전환) · `mpc_fb` (+ `mpc_riccati` overlay) · `reentry` (실기의 복귀 절차: loco 정지 창 → `/rtc_cm/trigger_estop` → hold → `/rtc_cm/clear_estop` (demo_joint 복귀 확인) → demo_joint 에 서는 자세 goal (`estop.arm_safe_position`) → `/sim/reset` 으로 로봇을 세움 → loco 재진입 창, 녹음 하나에 창 둘) · `estop` (loco 중 `/rtc_cm/trigger_estop` → hold → `/rtc_cm/clear_estop` → demo_joint 복귀 확인 → loco 전환, 이벤트 시각 · 창 지표는 `run.json`; 두 arm 모두 rtf 1 로 — clear 의 검증 기한이 wall 이라 느린 rtf 에서는 unverified 로 끝난다) · `walk` (MPC 를 켠 채 전환 → 명령 스크립트 `twist_script.py` (ramp 0 → 0.5 m/s 2 s · hold 8 s · yaw 0.3 rad/s 4 s · hold 2 s · 정지 10 s, sim 초; `--cmd-topic` 에 20 Hz wall, 컨트롤러의 구독이 붙은 뒤 시작) → 마지막 `--stop-window` s 가 정지 창) · `walk_estop` (같은 스크립트의 `--estop-at` s 에 `/rtc_cm/trigger_estop` → hold 창, clear 없음) 를 반복하고 (`--mpc-layout` 이면 모든 arm 이 같은 MPC layout, `loco` · `reentry` · `estop` 은 `mpc_off` overlay, `walk` · `walk_estop` 은 언제나 MPC; `--mpc-threads N` 이면 MPC layout 으로 뜨는 arm 이 `layout_profile:=mpc_par_sim mpc_threads:=N` 으로 N thread solve; `--overlay ARM=…` · `--launch-arg` 로 바꾼다; `--source inekf` 면 모든 arm 이 `estimator_inekf` overlay 로 추정 base 위에서 돈다 — overlay 를 이미 쓰는 arm 은 거부) recorder CSV/JSON · 로그 세션을 모은다. `repo_scripts/scripts/with_verify_hold.sh` 아래에서 돌린다.
- `walk_gate.py` — `run_arms.py` 의 `--out` 디렉토리에서 `walk_<k>` · `walk_estop_<k>` 를 판정한다 (`walk_gate.json`): 보행 지표는 `rtc_tools.analysis.walk_metrics` (스케줄러와 같은 yaw 정렬 속도 · 구간은 로그의 `cmd_vx` 열로 · 접촉 에지 1:1 짝짓기 · 체중은 그 run 의 stand 구간), 정지 창은 `mpc_<k>` (대조군) 의 같은 길이 꼬리와 비교, `walk_estop` 은 edge (E-STOP 열 유지 · waiting · fault 불변 · CM 거부 줄 0) 를 판정하고 hold 아래 낙하를 보고한다. 문턱은 파일 머리의 상수고, rtf · `z_ref` · 정지 창 길이는 각 run 의 `run.json` 에서 읽는다 (run 끼리 다르면 거부). 추종 · heading 창은 `--gait-yaml` 의 walk 주기 (마지막 switching time) 의 정수배 (`walk_metrics` 의 `*_CYCLES`) 라 gait 를 바꾸면 따라간다 — 창이 명령 스크립트의 구간보다 길어지는 gait 는 판정 전에 거부한다. `--probe` 는 dev PC rtf 를 고르는 기준 (나이 거부 0 · stand 뒤 plan 없는 row 0 · policy age p99 ≤ 135 ms — 시간 예산 3 node + 1 주기 (0.175 s) 에서 40 ms 뺀 값으로, 보류 허용치인 출하 `max_policy_age_s` (0.35 s) 와는 따로다) 만 본다. 성능 (solve 시간 · overrun · `wbc_tick_us`) 은 보고만 한다 — 판정은 제어 PC 다.

dev PC 의 보행 평가는 출하 MPC 값 그대로 (overlay 없이) 2 thread · rtf 0.7 로 돈다. 출하 online 예산 (6 / AL 1, 70 ms 주기) 은 dev PC 2 thread 에서 solve p99 가 주기 안에 들고, 0.7 은 `walk_gate.py --probe` 로 고른 값이다 (그 R 과 그보다 작은 R 이 모두 나이 기준을 통과). 서 있기 평가는 rtf 1 로도 돈다. plan 나이는 sim 시간으로 재므로 rtf < 1 에서는 같은 wall 지연이 R 배로 작게 보인다 — 실시간 여유는 rtf 1 run 으로 본다. 이 PC 보다 느린 PC 나 1 thread 에서 solve 가 70 ms (wall) 를 넘으면 그 solve 는 withheld 되고, 그것이 `max_policy_age_s` 를 넘게 이어지면 plan 이 끊긴다. 그때는 `solver.publish_deadline_ms: 0.0` overlay (repo 밖) 를 주고 rtf 를 다시 고른다:

```bash
repo_scripts/scripts/with_verify_hold.sh python3 integrated_bringup/tools/standing_eval/run_arms.py \
  --out /tmp/walk --arms mpc walk walk_estop -k 3 --mpc-layout --mpc-threads 2 --rtf 0.7 \
  --lead 6 --duration 26 --runtime-yaml integrated_bringup/config/g1_loco/controllers/locomotion/runtime.yaml \
  --logging-root ~/ros2_ws/rtc_ws/logging_data --launch-arg max_log_sessions:=200
python3 integrated_bringup/tools/standing_eval/walk_gate.py /tmp/walk \
  --gait-yaml integrated_bringup/config/g1_loco/controllers/locomotion/gait.yaml
```
