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
| `robot_transforms` | 컨트롤러 | YAML `role: robot_transforms` — `<config_key>/transforms` (`tf2_msgs/TFMessage`). frame 목록은 아래 |
| `state` (group 별 `joint_states`) | CM | `/rtc_cm/<group>/joint_states` (`JointState`) |
| device-wire command (`joint_command`, `ros2_command`) | DeviceBackend | `devices.<group>.backend:` — backend 구현체가 직접 publish |

`robot_transforms` 의 frame: DemoJoint / DemoTask / DemoCompliance 는 `base→tool0_actual` + `base→{thumb,index,middle,ring}_tip_link_actual` + `base→virtual_tcp_actual` 6 개, DemoWbc 는 `tool0_actual` + 핑거팁 넷 + `wbc_alpha_actual` (placeholder, `valid=false`) 이다. active 컨트롤러만 publisher 가 활성이다.

외부 도구 (BT bridge · GUI · digital_twin · shape_estimation) 는 `/active_controller_name` (TRANSIENT_LOCAL) 을 구독해 런타임에 토픽 네임스페이스를 다시 연결해야 한다. 컨트롤러를 전환하면 각 소유 토픽이 이전 네임스페이스에서 침묵하고 새 네임스페이스에서 살아난다.

## CSV 로그

세션 디렉토리 `<session>/controllers/<config_key>/` 아래에 남는다. 공통 규약은 이렇다.

- tick 당 한 행이다. 계산하지 않은 tick 은 gap 이 아니라 `valid=0` 행으로 남고 (값은 0 이거나 직전 값 동결 — 파일마다 다르니 아래를 본다), 행 사이의 `tick` 간극은 SPSC ring drop 만을 뜻한다.
- 파일이 없으면 그 컨트롤러의 `logs:` 블록에 그 채널이 없거나 해당 YAML 블록이 비활성이라는 뜻이다.
- 행은 그 컨트롤러가 active 인 동안 (`Compute()` 가 도는 tick) 에만 쌓인다. 전환해 나간 컨트롤러의 파일은 거기서 멈춘다.
- 세션 폴더 이름 `YYMMDD_HHMM` 은 분 단위다. 같은 분 안에 다시 띄우면 두 번째 실행이 **같은 폴더의 CSV 에 이어 쓴다** — header 를 다시 쓰지 않으므로 `tick` 이 되돌아가는 행이 경계다. sim 을 연달아 띄우는 드라이버는 그 분의 폴더가 이미 있으면 다음 분까지 기다린 뒤 띄운다.
- 그림과 통계는 `ros2 run rtc_tools plot_rtc_log <csv>` (`--stats` 로 통계만).

| 파일 | 컨트롤러 | 내용 |
|---|---|---|
| `<device>_state.csv` · `<device>_sensor.csv` | 전부 | device 별 상태 · 센서. 파일 이름은 device 이름에서 파생된다 (`ur5e_state.csv` · `p1a_state.csv` · `p1a_sensor.csv`, `iiwa7_state.csv` · `leap_state.csv`). 헤더와 행 폭은 config 폭이고, 런타임 채널 수가 적으면 0 으로 채우며 실제 수는 마지막 `num_*` 컬럼이 싣는다 |
| `<device>_state.csv` (wbc) | wbc | 위의 superset (`DeviceWbcLog`) — TSID `a_opt`, SE3 trajectory (arm), 핑거팁 힘 (hand: 크기 `fingertip_force_*` 와 벡터 `fingertip_fx_/fy_/fz_*`) |
| `wbc_diag.csv` | wbc | solve time · λ · 수렴 · grasp 진단. 마지막 `num_contact_vars` 컬럼이 런타임 λ 수를 싣는다. `qp_fail_count` (dynamic) · `kin_qp_fail_count`. 실패 분류 (#551): `qp_status` 는 proxsuite `QPSolverOutput` 정수 (0 SOLVED · 1 MAX_ITER_REACHED · 2 PRIMAL_INFEASIBLE · 3 SOLVED_CLOSEST_PRIMAL_FEASIBLE · 4 DUAL_INFEASIBLE · 5 NOT_RUN, −1 = 시도 없음), `qp_iterations` (HQP 는 시도한 level 합), `joint_box_inverted` 는 그 tick 에 l > u 로 비어 버린 joint_limit 가속도 box 행 수 (−1 = 제약 없음·비활성). `plot_rtc_log --stats` 가 status 를 이름별 히스토그램으로 낸다 |
| `task_diag.csv` | task | 특이점 진단. `sigma0` · `lambda_max` 는 그 tick 에 실제 적용된 값이고 `damping_active` 는 파생 컬럼이다. E-STOP · arm 미판독 tick 은 σ 가 직전 값이고 `valid=0` |
| `pull_estimator.csv` | joint · task · compliance | in-plane pull-force 추정. wire 에 없는 `force_prefilter_*` (필터 직전 값) 와 `tick` 을 더 기록한다. `contact_mask[thumb\|index\|middle]` 처럼 컬럼 이름에 비트 순서가 박혀 있다. 등록은 `pull_estimator` 블록이 활성일 때만 |
| `grasp_diag.csv` | joint · task · compliance | Force-PI 서보와 강성 추정기 진단. `f_measured_*` 는 25 Hz Force-PI 뱅크이며 `<device>_sensor.csv` 의 `force_filtered_*` (contact_stop 뱅크) 와 다른 레인이다. `k_inst_raw_*` · `delta_s_*` · `delta_f_*` 는 CSV 의 s/f 컬럼에서 복원할 수 없다. 등록 게이트는 `force_pi_grasp` 블록의 존재다. PI 가 안 돈 tick 은 `valid=0` 이고 per-finger 필드는 0 이다. Holding 구간 σ 는 `plot_rtc_log --stats` 가 낸다. 실기 절차: [grasp_tuning_guide.md](../../rtc_controllers/docs/grasp_tuning_guide.md) |
| `momentum_observer.csv` | joint · task · compliance · wbc | 운동량 관측기 잔차 `r_<joint>` (arm device order) 와 payload / inertial 추정. held · E-STOP tick 은 `valid=0` 행이고 잔차는 직전 값으로 동결된다. 파일 부재는 `momentum_observer` 블록이 없거나 disabled 라는 뜻이다. `plot_rtc_log` 가 `momentum_observer.png` (와, Layer 2 가 구성된 run 의 `momentum_payload.png`) 를 낸다 |
| `compliance_diag.csv` | compliance | task-admittance 진단. [demo-compliance-controller.md](demo-compliance-controller.md#로그) |
| `inference_diag.csv` | inference | hold 사유와 정책 입력. [demo-inference-controller.md](demo-inference-controller.md#로그) |
| `dualarm_diag.csv` | dualarm | 열 묶음은 [demo-dualarm-controller.md](demo-dualarm-controller.md#로그와-tf-판독) |
| `catching_diag.csv` · `planner_events.csv` | catching | [demo-catching-controller.md](demo-catching-controller.md#tick-레코드-csv) |

`mpc` 스레드의 타이밍은 `<session>/timing/mpc_timing_log.csv` 에, CM 의 스레드별 타이밍은 `<session>/timing/` 아래에 쌓인다 ([rtc_controller_manager README](../../rtc_controller_manager/README.md)).
