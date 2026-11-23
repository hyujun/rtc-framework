# 컨트롤러 토픽과 CSV 로그

데모 컨트롤러의 외부 토픽을 누가 소유하는지, 컨트롤러가 세션 디렉토리에 남기는 CSV 가 무엇인지를 정리한 문서다. 컬럼의 정확한 뜻 · 단위 · enum 은 `include/integrated_bringup/logging/*_log_pod.hpp` 의 필드 주석이 SSoT 다. 로그 레벨과 THROTTLE 규칙은 [agent_docs/conventions.md](../../agent_docs/conventions.md#logging) 를 본다.

## 토픽 소유권

non-RT 외부 통신 토픽은 **컨트롤러가 소유**한다. 아래 경로의 예는 `demo_wbc_controller` 가 active 일 때다.

| 토픽 역할 | 소유자 | 경로 · 메커니즘 |
|---|---|---|
| `target` (group 별 joint_goal / task_goal) | 컨트롤러 | YAML `topics:` 의 `role: target` — `/demo_wbc_controller/<group>/joint_goal` |
| `grasp_state` (joint · task · compliance) | 컨트롤러 | `SeqLock<GraspStateData>` + `SetupGraspStatePublisher()` ([owned_topics.hpp](../include/integrated_bringup/support/owned_topics.hpp)) |
| `wbc_state` (wbc) | 컨트롤러 | `SeqLock<WbcStateData>` + `SetupWbcStatePublisher()` |
| `payload_estimate` (joint · task · compliance · wbc) | 컨트롤러 | `rtc_msgs/PayloadEstimate`. 발행 조건은 payload 추정기가 아니라 momentum observer 활성이다 (잔차만으로도 메시지가 성립). `joint_names` 가 잔차의 device order 를 자기기술한다 |
| `tof_snapshot` (joint · task · compliance) | 컨트롤러 | `SeqLock<ToFSnapshotData>` + `SetupToFSnapshotPublisher()` |
| `catching_state` (catching) | 컨트롤러 | `/<config_key>/catching_state` ([demo-catching-controller.md](demo-catching-controller.md#상태-토픽)) |
| `locomotion_status` (locomotion) | 컨트롤러 | `SeqLock<LocomotionDiagLogPod>` + `SetupLocomotionStatusPublisher()` — `/<config_key>/<body group>/locomotion_status` (`diagnostic_msgs/DiagnosticStatus`, [locomotion-controller.md](locomotion-controller.md#속도-명령과-상태-토픽-mpcenabled)). 속도 명령 `<body group>/cmd_vel` (`geometry_msgs/Twist`, 또는 `velocity_command.type: joy` 면 `sensor_msgs/Joy`) 도 컨트롤러 소유 구독이다 |
| `robot_transforms` | 컨트롤러 | YAML `role: robot_transforms` — `<config_key>/transforms` (`tf2_msgs/TFMessage`). frame 목록은 아래 |
| `state` (group 별 `joint_states`) | CM | `/rtc_cm/<group>/joint_states` (`JointState`) |
| device-wire command (`joint_command`, `ros2_command`) | DeviceBackend | `devices.<group>.backend:` — backend 구현체가 직접 publish |

`robot_transforms` 의 frame: DemoJoint / DemoTask / DemoCompliance 는 `base→tool0_actual` + `base→{thumb,index,middle,ring}_tip_link_actual` + `base→virtual_tcp_actual` 6 개, DemoWbc 는 `tool0_actual` + 핑거팁 넷 + `wbc_alpha_actual` (placeholder, `valid=false`), HandJoint 는 손 tree root (`p2` 는 `palm`) → `<tip>_actual` 핑거팁 넷뿐이다 (팔 tip 없음). active 컨트롤러만 publisher 가 활성이다.

외부 도구 (BT bridge · GUI · digital_twin · shape_estimation) 는 `/active_controller_name` (TRANSIENT_LOCAL) 을 구독해 런타임에 토픽 네임스페이스를 다시 연결해야 한다. 컨트롤러를 전환하면 각 소유 토픽이 이전 네임스페이스에서 침묵하고 새 네임스페이스에서 살아난다.

## CSV 로그

세션 디렉토리 `<session>/controllers/<config_key>/` 아래에 남는다. 공통 규약은 이렇다.

- tick 당 한 행이다. 계산하지 않은 tick 은 gap 이 아니라 `valid=0` 행으로 남고 (값은 0 이거나 직전 값 동결 — 파일마다 다르니 아래를 본다), 행 사이의 `tick` 간극은 SPSC ring drop 만을 뜻한다.
- 파일이 없으면 그 컨트롤러의 `logs:` 블록에 그 채널이 없거나 해당 YAML 블록이 비활성이라는 뜻이다.
- 예외는 첫 행에서 파일을 여는 채널 (`rtc::LogOpen::kOnFirstRow`, 지금은 `locomotion_truth.csv` 하나) 이다: 쓸 값이 있는 tick 에만 행을 쓰므로 `tick` 간극은 drop 이 아니고, 행이 하나도 없으면 파일도 없다. 그 열기가 실패하면 경고 한 번과 함께 행을 버린다.
- 행은 그 컨트롤러가 active 인 동안 (`Compute()` 가 도는 tick) 에만 쌓인다. 전환해 나간 컨트롤러의 파일은 거기서 멈춘다.
- 세션 폴더 이름 `YYMMDD_HHMMSS` 는 초 단위다 (#1156 이전 세션은 분 단위 `YYMMDD_HHMM`). 같은 초 안에 다시 띄우면 두 번째 실행이 **같은 폴더의 CSV 에 이어 쓴다** — header 를 다시 쓰지 않으므로 `tick` 이 되돌아가는 행이 경계다. sim 을 연달아 띄우는 드라이버는 그 초의 폴더가 이미 있으면 다음 초까지 기다린 뒤 띄운다.
- 그림과 통계는 `ros2 run rtc_tools plot_rtc_log <csv>` (`--stats` 로 통계만).

| 파일 | 컨트롤러 | 내용 |
|---|---|---|
| `<device>_state.csv` · `<device>_sensor.csv` | 전부 | device 별 상태 · 센서. 파일 이름은 device 이름에서 파생된다 (`ur5e_state.csv` · `p1a_state.csv` · `p1a_sensor.csv`, `iiwa7_state.csv` · `leap_state.csv`). 헤더와 행 폭은 config 폭이고, 런타임 채널 수가 적으면 0 으로 채우며 실제 수는 마지막 `num_*` 컬럼이 싣는다 |
| `<device>_state.csv` (wbc) | wbc | 위의 superset (`DeviceWbcLog`) — TSID `a_opt`, SE3 trajectory (arm), 핑거팁 힘 (hand: 크기 `fingertip_force_*` 와 벡터 `fingertip_fx_/fy_/fz_*`) |
| `wbc_diag.csv` | wbc | solve time · λ · 수렴 · grasp 진단. 마지막 `num_contact_vars` 컬럼이 런타임 λ 수를 싣는다. `qp_fail_count` (dynamic) · `kin_qp_fail_count`. 실패 분류 (#551): `qp_status` 는 proxsuite `QPSolverOutput` 정수 (0 SOLVED · 1 MAX_ITER_REACHED · 2 PRIMAL_INFEASIBLE · 3 SOLVED_CLOSEST_PRIMAL_FEASIBLE · 4 DUAL_INFEASIBLE · 5 NOT_RUN, −1 = 시도 없음, −2 = 제약 행이 QP 행 예산을 넘어 풀지 않음), `qp_iterations` (HQP 는 시도한 level 합), `joint_box_inverted` 는 그 tick 에 l > u 로 비어 버린 joint_limit 가속도 box 행 수 (−1 = 제약 없음·비활성). `plot_rtc_log --stats` 가 status 를 이름별 히스토그램으로 낸다 |
| `task_diag.csv` | task | 특이점 진단. `sigma0` · `lambda_max` 는 그 tick 에 실제 적용된 값이고 `damping_active` 는 파생 컬럼이다. E-STOP · arm 미판독 tick 은 σ 가 직전 값이고 `valid=0` |
| `pull_estimator.csv` | joint · task · compliance | in-plane pull-force 추정. wire 에 없는 `force_prefilter_*` (필터 직전 값) 와 `tick` 을 더 기록한다. `contact_mask[thumb\|index\|middle]` 처럼 컬럼 이름에 비트 순서가 박혀 있다. 등록은 `pull_estimator` 블록이 활성일 때만 |
| `grasp_diag.csv` | joint · task · compliance | Force-PI 서보와 강성 추정기 진단. `f_measured_*` 는 25 Hz Force-PI 뱅크이며 `<device>_sensor.csv` 의 `force_filtered_*` (contact_stop 뱅크) 와 다른 레인이다. `k_inst_raw_*` · `delta_s_*` · `delta_f_*` 는 CSV 의 s/f 컬럼에서 복원할 수 없다. 등록 게이트는 `force_pi_grasp` 블록의 존재다. PI 가 안 돈 tick 은 `valid=0` 이고 per-finger 필드는 0 이다. Holding 구간 σ 는 `plot_rtc_log --stats` 가 낸다. 실기 절차: [grasp_tuning_guide.md](../../rtc_controllers/docs/grasp_tuning_guide.md) |
| `momentum_observer.csv` | joint · task · compliance · wbc | 운동량 관측기 잔차 `r_<joint>` (arm device order) 와 payload / inertial 추정. held · E-STOP tick 은 `valid=0` 행이고 잔차는 직전 값으로 동결된다. 파일 부재는 `momentum_observer` 블록이 없거나 disabled 라는 뜻이다. `plot_rtc_log` 가 `momentum_observer.png` (와, Layer 2 가 구성된 run 의 `momentum_payload.png`) 를 낸다 |
| `compliance_diag.csv` | compliance | task-admittance 진단. [demo-compliance-controller.md](demo-compliance-controller.md#로그) |
| `inference_diag.csv` | inference | hold 사유와 정책 입력. [demo-inference-controller.md](demo-inference-controller.md#로그) |
| `dualarm_diag.csv` | dualarm | 열 묶음은 [demo-dualarm-controller.md](demo-dualarm-controller.md#로그와-tf-판독) |
| `locomotion_diag.csv` | locomotion | 모드 · QP · margin · base · plan 열과 보행 판정 열 (필터 출력 명령 `cmd_*` · `cmd_valid` · `gait_latest` · plan base 속도 `ref_base_v*` · 접촉 lane 의 신선도 `meas_contact_valid`) · 앞 plan 유지 수 `plan_holds` · 판정 전 표본 plan `sample_*` · R tier 열 (`wrench_des_*` · `servo_kp/kd_*` · `mpc_last_*`, [Log tiers](locomotion-controller.md#log-tiers)) · 명령 입력 `cmd_input` (0 twist · 1 joy) · `cmd_deadman` (joy 1 / 0 / 읽지 못함 −1, twist −1 — 운영자가 놓음과 stale 을 가른다) · 재생 입력 `est_imu_valid` · `est_imu_age_us` (IMU 입력 열 `imu_*` · `est_imu_sequence` · `est_skew_us` · `est_planned_mask` 와 함께 source 와 무관하게 채운다) · MPC lock-step `mpc_sync_lockstep` · `mpc_trigger_tick` · `mpc_wait_us` · `mpc_wait_count` · `mpc_pending_superseded` (fault cause `mpc_wait`). [locomotion-controller.md](locomotion-controller.md#로그와-판정) |
| `locomotion_truth.csv` | locomotion | sim 참값 (base raw lane · 바닥→발 sole wrench · CoP · lane 나이). 참값 lane 이 신선한 tick 에만 행을 쓰고 첫 행에서 파일을 연다 (`LogOpen::kOnFirstRow`) — 실기 run 에는 없다. [locomotion-controller.md](locomotion-controller.md#log-tiers) |
| `joint_ff.csv` | hand_joint | tick 당 feedforward 기록: 상태 lane (`estop` · `readable` · `seeded` · `traj_active` · `ff_enabled` · `mode` (정수: 0 tracking · 1 damping · 2 damping_feedforward) · `ff_finite` · `model_fresh` · 폐쇄 체인 `held` · `held_ticks` · `singular` · `closure_error` · `clamped_mask`) 뒤에 관절별 `q_*` · `q_ref_*` · `g_*` (모델 중력항, N·m) · `tau_ff_*` (wire 의 feedforward) · `qdd_ref_*` (reference 가속도, 정지 시 0) · `tau_inertia_*` (법칙이 더한 관성항 k_M·M·q̈_ref, clamp 전) 와 상태 lane 의 `inertia_gain` (그 tick 에 적용된 k_M) — 한 행에서 `tau_ff = clamp(k_g·g + tau_inertia)` 를 읽는다. E-STOP · unreadable tick 도 행을 남기고 (`ff_finite=0` / `readable=0`), `tau_ff_*` 는 그 tick 에 wire 로 나간 값이다. `analyze_hand_gravity_ff` 가 `ff_enabled` 로 A/B 구간을 나눠 `q_ref_* − q_*` 를 판정한다 (`mode` 0 구간만 의미 있다 — 비-tracking 은 q_ref = q); `plot_rtc_log` 는 비-tracking 구간을 음영으로 칠하고 drift 를 낸다 ([hand-joint-controller.md](hand-joint-controller.md)) |
| `catching_diag.csv` · `planner_events.csv` | catching | [demo-catching-controller.md](demo-catching-controller.md#tick-레코드-csv) |

`mpc` 스레드의 타이밍은 `<session>/timing/mpc_timing_log.csv` 에, CM 의 스레드별 타이밍은 `<session>/timing/` 아래에 쌓인다 ([rtc_controller_manager README](../../rtc_controller_manager/README.md)).

## 로그 끄기

스위치는 세 단이다. 위가 꺼지면 아래는 보지 않는다.

- `_base.yaml` `enable_logging: false` — CM timing CSV 와, controller 가 세션에 여는 파일 전부 (`logs:` 채널, `planner_*` · `nlp_candidates.csv`, `mpc_timing_log.csv`) 가 열리지 않는다. 경고는 없다.
- 그 아래 `enable_timing_log` 는 CM timing CSV 2개만, 각 controller YAML 의 `logs:` 는 그 controller 의 채널만 정한다. `logs:` 를 비우려면 `logs: []` 나 항목을 지운 `logs:` (null) 모두 된다.
- 손 driver 는 별도 프로세스다: `udp_hand_node_<profile>.yaml` 의 `enable_timing_log: false` 가 `timing/hand_timing_log.csv` 를 끈다 ([udp_hand_driver README](../../udp_hand_driver/README.md)).

Proto2 처럼 손 driver 가 있는 robot launch 에서 고주기 CSV 를 모두 끄려면 위 첫째와 셋째, 두 줄이다. 꺼도 남는 것: launch 가 만드는 세션 폴더와 그 retention (`max_log_sessions`), driver 가 10 s 마다 덮어쓰는 `device/hand_stats*.json`, ROS 로그 (`~/.ros/log`).
