# Testing & Debugging

이 문서는 테스트를 **돌리는 법**과 런타임에서 **살펴보는 곳**을 모은 실무 가이드다. 무엇을 반드시 돌리고 결과를 어떻게 판정하는지(규범)는 [agent_docs/testing-debug.md](../agent_docs/testing-debug.md) 가 갖고, 함정이 실제로 발현한 사례와 근거는 [testing-rationale.md](reference/testing-rationale.md) 에 있다.

## Test Commands

테스트는 기본으로 빌드되지 않는다. 먼저 `./build.sh --tests [-p <pkg>]` 로 빌드한다 (테스트 없이 빌드한 패키지는 실패가 아니라 "0 tests" 를 보고한다). 직접 `colcon` 을 칠 때는 workspace root 에서 환경을 source 한 뒤 실행한다 ([AGENTS.md](../AGENTS.md) §9.1).

```bash
colcon test --event-handlers console_direct+                      # 전체
colcon test-result --verbose
colcon test --packages-select ur5e_bt_coordinator --event-handlers console_direct+   # 패키지 하나
colcon test --packages-select rtc_controllers --ctest-args -R test_grasp_controller  # C++ 테스트 하나
colcon test --packages-select rtc_digital_twin --pytest-args -k test_urdf_parser     # Python 테스트 하나
PYTHONPATH=rtc_tools:$PYTHONPATH <rtc_ws>/.venv/bin/python -m pytest <pkg>/test/<file>.py  # venv 전용 pytest 직접 (repo 루트에서)
```

colcon 의 pytest 는 시스템 python 이라 `.venv` 에만 있는 의존성 (mujoco 등) 의 테스트는 skip 된다 — 그 파일은 마지막 줄로 repo 루트에서 직접 돌린다. `PYTHONPATH` 는 덮어쓰지 말고 이어붙이고, `rtc_tools` 를 앞에 두는 이유는 [testing-rationale.md](reference/testing-rationale.md#venv-격리의-false-green) 에 있다.

`colcon.pkg` 로 패키지 안 병렬 (`ctest-args: ["-j", n]`) 을 켠 패키지는 CLI 의 `-j 1` 로 직렬이 되지 않는다 (`colcon.pkg` 의 `-j` 가 뒤에 붙는다). 직렬로 돌리려면 `COLCON_EXTENSION_BLOCKLIST=colcon_core.package_augmentation.colcon_pkg colcon test …` 로 `colcon.pkg` 를 읽지 않게 한다. `ament_python` 패키지의 병렬 (pytest-xdist) 과 xdist 가 없는 호스트의 처리는 `rtc_tools/colcon.pkg` 주석이 갖는다.

## Sensor Matrix

변경 위치를 찾아 **필수 + 추가** 를 모두 돌린다 ([AGENTS.md](../AGENTS.md) §5). 기본형은 `colcon test --packages-select <pkg>` 이고, 표의 "필수" 는 그 안에서 반드시 봐야 하는 테스트와 추가 명령이다. 각 테스트가 무엇을 판별하는지는 그 테스트 파일의 머리 주석이 갖고, 행에 걸친 설계 원칙과 sanitizer 빌드의 예외는 [testing-rationale.md](reference/testing-rationale.md#sensor-matrix-보충) 에 있다.

| 변경 위치 | 필수 | 추가 |
|---|---|---|
| `rtc_base/` | `rtc_base` 전체 (`test_rt_heap` 은 mallopt 가 프로세스 전역이라 별도 바이너리) | 전체 downstream |
| `rtc_math/` | `rtc_math` 전체 (Pinocchio 가 있으면 `test_se3_module` 교차검증 포함) | `se3_error_compare` 실험 + `plot_se3_compare.py` (선택) |
| `rtc_msgs/` | 위 + `./build.sh --tests full` (msg 생성 전파) | downstream pub/sub 테스트 |
| `rtc_controller_interface/` | 패키지 전체 (registry · interface · mailbox · output-validation) | downstream controller 빌드 |
| `rtc_controllers/` RT path | `test_*_core` + `test_dls_convergence` + `test_contact_inekf_*` · `test_contact_fsm_*` (할당 게이트 무장) + grasp 관련 gtest; 접촉 FSM 은 `rtc_mujoco_sim` 의 `test_contact_fsm_sim` 도 (sim 접촉 truth) | `ps -eLo cls,rtprio` 로 RT scheduling 확인 |
| `rtc_controllers/` gains · config | 위 + controller YAML 로드 smoke · `test_yaml_read` (`params/yaml_read.hpp`) | `ros2 topic echo /rtc_cm/active_controller_name` |
| `rtc_controllers/` catching core | `--ctest-args -R test_catching` | ASan/UBSan 은 별도 build base (rationale 참조) |
| `rtc_controller_manager/` | RT loop timing · `test_rt_loop_pipeline` (선언된 PD 게인이 slot 으로, hold 는 게인 없음; IMU lane 의 raw 값 · producer orientation 이 site 별 bit 그대로, site 수로 bound) (`/system/estop_status`) · `test_trigger_estop_service` (운영자 E-STOP 응답 · 이미 래치 · 동시 trigger 1 회) · `test_estop_hold_latch` (창 중 trigger 의 hold 목표 불변 · `estop_on_clear` resume 문자 동일 / initial_controller 전환 · 실패 재래치 · 재래치 거부 · timeout 뒤 switch 없음) · `test_on_configure` (`estop_on_clear` 거부 · read-only) · teardown (`test_eventfd_teardown_order` · `test_state_lane_teardown`) | `cm_timing_log.csv` p99 |
| `rtc_tsid/` · `rtc_mpc/` | QP · task · constraint gtest (`test_floating_wqp_gates` — RNEA oracle · 해 digest · `test_floating_bounds` — Baumgarte · 가속 · torque-rate box · `tau_valid` · solver 키 · `test_locomotion_formulation` — 보행 QP 정지 · swing 마스킹 · 추종 · `ReferenceKinematics` 중앙차분 oracle · solve 보고 · `test_locomotion_gait` — 기본값 궤적 digest · reset-μ warm start + zero 비활성 λ 의 보행 궤적 게이트 (수렴 · 전환 반복 · 부호 오류 multiplier · cold 기준해 τ · 목적함수, 기본값 대조군, 잡음 run) · `test_wbc_health` — 행 slack margin 독립 재계산 · 결속 fixture · 피라미드 모서리 (원뿔 대조군) · `SolveHealth` 시퀀스 · 모순 QP 의 Fault · `health` 키 · `test_tsid_compute_malloc`) · MPC gtest (Riccati · SolutionManager · `test_command_filter` — 해석해 Bessel step oracle · `test_gait_schedule` — tiling oracle 리스트 · `test_gait_schedule_alloc_gate` · `test_swing_trajectory` — Hermite basis oracle · 등록 극값 · `test_swing_trajectory_alloc_gate` · `test_gait_phase` — tile 위상 · swing 시작 · `test_stage_reference` — 등록 knot · FK 둘 · tick 간 일관성 · `test_stage_reference_alloc_gate` · `test_mpc_handler_core_guards` — stub OCP 로 cold seed · 비유한 해 · 비유한 상태 거부 · `test_kinodynamic_dynamics` — 공개 G1 의 RNEA · M_bb · ABA oracle · mutant · masking · stage 안 mode 전환 · `test_kinodynamic_alloc_gate` · `test_kinodynamic_derivatives` — 연속 · 이산 Jacobian 중앙차분 · `test_kinodynamic_integrator` — 차수 · 관절 블록 정확 · solver 수준 mode 전환 · `test_contact_tracking_residual` — 독립 기구학 · stage ODE 가속도 · 기울기 · scale · 1/2 규약 · Jacobian 중앙차분 · `test_contact_tracking_alloc_gate` · `test_contact_constraints` — mode 별 비활성 행 값 · 마찰 / CoP 부호 fixture · gain 조합 · Jacobian 중앙차분 (3D 포함) · `test_barrier_costs` — 닫힌식 · C² · joint limit 인덱스 · 충돌 GN Hessian · relaxed log · 접촉 원뿔 barrier (독립 합 · GN Hessian) · 중앙차분 · `test_contact_constraints_alloc_gate` · `test_kinodynamic_ocp` — `mpc.kinodynamic:` 거부 · weight 행 배치 · walk 창 mode · 참조 바인딩 (비트 동일 · 거부 시 불변) · `constraints:` 거부 · stage 고정 제약 행 · 독립 cost 합 (joint barrier 포함) · 정지 / 한 발 swing (착지 속도 0 포함) / 밀린 시작 / 출하 walk 창 solve (SIE · RK2, 노드 `u_ref` seed, 등식 잔차 ≤ prim, 원뿔 안) · `test_kinodynamic_ocp_alloc_gate` — `UpdateReferences` · `ReferenceControl` 할당 0 · `ConstraintResiduals` = ODE 몫 · 2차 state / input / terminal cost (structured, 구조 판정 확인) 할당 0 · `test_structured_quadratic_costs` — structured 2차 state · input · terminal cost 가 Aligator 의 값 · gradient · Hessian 과 1e-12 · zero pattern 일치 (seeded 1000, G1 fixture · 단위 weight) · 허용 joint 9 종 chain · 구조 없는 모델 · 비대각 weight · `gauss_newton` 해제는 Aligator 와 bit 동일 · `kDense` 빌드 대조 · `test_mpc_factory` — kinodynamic dispatch · base 거부 · `limits.friction_mu` 거부 · `test_aligator_api` — 값 0 / −1 · Jacobian 0 제약 행 (12 + 10 · 접촉 mask 14 + 10) 의 해 불변 · `test_mpc_handler_core_floating` — kinodynamic stub OCP 로 `nu` · λ packing · 구간 평균 a_ff (forward 동역학 · defect bound) · node mode · schedule id · 쿼터니언 반구 · state epoch cold start · `test_kinodynamic_receding` — 한 예산의 cold start (등록 한계) · receding walk 창 · node grid shift (0 · 1 · 2 노드 비트 동일 · mode flip · 누출 0) · stage 병렬 2 – 4 thread 가 serial 과 bit 동일 (iterate · gain · 잔차 · 비용) · 품질 틀 · `test_mpc_solver_pool` — pool 이름 · 고정 · 재시작 · 거부 · 1 thread 는 pool 없음 · `test_mpc_thread_publish` — deadline · 소비 구간 잔차 · cold withhold publish 규칙 · `require_fresh_state` (stub 호출 수) · snapshot stamp 축 phase 시각 · re-anchor · `test_mpc_state_epoch_threads` — producer · consumer · bumper 3 스레드에서 bump 뒤 옛 epoch plan 샘플 0 · `test_policy_sampler` — 정수 ns 시간축 (노드 bit · seeded 1000 시각) · 부동 커널 (독립 slerp 식 · 등속 base · 관절 행 고정 경로와 bit) · wrench 구간 유지 · mode 마스크 · H = 64 마지막 구간 · 부동 Riccati (손 SE(3) log oracle · gain 노드 · 블록 cap · π fade) · 설정 거부 · 1000 tick 할당 0 · `test_kinodynamic_mpc_soak` — 12 조합 3 s smoke) | TSID performance · `mpc_timing_log.csv` p99 회귀 · `RTC_RUN_TIMING_GATES=1` 로 `test_mpc_solution_copy_cost` · `test_policy_sampler_cost` · `test_locomotion_gait` (tick p99 ≤ 500 µs 잠정) (Release, 고정 코어 — payload 복사 · 샘플러 정상 / 설치 tick 비용 판정, 평소엔 출력만) · `test_kinodynamic_receding` 의 4 thread online 시간 (같은 run, structured ≤ 1.10 × dense derivative; serial / 4 thread 비는 출력만) · `RTC_RUN_MPC_SOAK=1` 로 kinodynamic soak (손으로, `--run` 밖 — [§Kinodynamic MPC soak](#kinodynamic-mpc-soak); 연산 시간은 제어 PC 에서 판정) |
| `rtc_mujoco_sim/` | 패키지 전체 + `test_motor_servo_gains` · `test_torque_mode_ctrl_range` · `test_contact_fsm_sim` (MuJoCo 없으면 빠짐 — 실행 로그로 확인); 기동 · 리셋 자세는 `test_initial_qpos` (command joint) · `test_reset_keyframe` (`reset_keyframe` — freejoint 포함 qpos 전체 bit-exact + 미설정 대조군) | `ros2 launch integrated_bringup sim_ur5e_p1a.launch.py` smoke |
| `rtc_urdf_bridge/` | 패키지 전체 + `test_frame_jacobian_fd_oracle` · `test_inertial_validation` · `test_real_model_inertial_gate` · `test_g1_sole_frames` · `test_floating_closed_chain` (free-flyer root 의 폐쇄 체인 — fixed 빌드와 동등) | 실제 robot 모델 로드 smoke |
| `rtc_communication/` · `rtc_inference/` · `rtc_digital_twin/` | UDP · CAN (vcan0 없으면 skip) · RS485 PTY · ONNX · pytest | vcan0 설정 후 CAN, 실제 모델 · RViz2 smoke |
| `robot_descriptions/` | `rtc_tools` 의 `test_real_model_pairs.py` (venv 직접) · `test_vendored_manifests.py`, `rtc_urdf_bridge` 의 `test_real_model_inertial_gate` (G1 URDF 는 `test_g1_sole_frames` 도); G1 부동 MJCF 는 `rtc_mujoco_sim` 의 `test_g1_floating_stance` | 그 모델을 여는 launch smoke |
| `rtc_tools/` | pytest. MJCF/URDF 를 고쳤으면 `test_real_model_pairs.py` 가 게이트 (colcon 에서 skip 될 수 있음) | venv 직접 실행 ([Test Commands](#test-commands)); `joint_command_gui` 를 고쳤으면 proto2 SIL (`sil_mode:=firmware`) 에서 Enable · 포즈 재생 — 절차는 rtc_tools README joint_command_gui 절 |
| `rtc_teleop_input/` | pytest 전체 — model (키 · 포커스 · 자동 반복 · 버튼 규칙 · hold 규칙 (deadman · E-STOP · 동시 두 버튼 · 해제의 E-STOP 놓음) · 중계 · help 항목) · rclpy 노드 (주기 · 중계 · 비차단 호출 · 요청 필드 · 응답 종류 · latched 구독 QoS · help combo) · launch · offscreen Qt (help 토글 · badge · legend, skip 0 이어야 한다) | 실물 패드 smoke (`input:=pad`, LT+RT 포함, README 의 문제 해결 표) · virtual GUI 의 키 · 포커스 · help |
| `repo_scripts/` · `.claude/hooks/` | `test_rt_common` · `test_install_deps` · `test_install_python` · `test_build_jobs` · `test_can_setup` + shell unit test. hook · `validate_test_*.py` · 그 import 를 고쳤으면 `bash repo_scripts/test/test_verify_changes.sh` (CI 전용 — `colcon test` 도 `--run` 도 돌리지 않는다) | `check_rt_setup.sh --summary` |
| `shape_estimation*/` | ToF + exploration gtest | `/shape/estimate` echo, `/shape/explore` 액션 |
| `integrated_bringup/` demo FSM | demo_wbc · demo_joint · demo_task · grasp_phase_manager · virtual_tcp gtest | BT coordinator 통합 |
| `integrated_bringup/` demo_joint 군 0 · 손 2 군 · dualarm | `test_joint_tree_primary_group` · `test_joint_three_groups` (몸통 + 손 2 — 군별 target 구독 · hold · E-STOP · 4 군 거부) · `test_demo_joint_controller` 의 `JointThreeGroupTest` (두 번째 손의 지연 · 협폭 seed) · `test_demo_dualarm_controller` · `test_demo_dualarm_alloc` | sim_g1_p1b 로 launch 후 GUI 확인 |
| `integrated_bringup/` g1_loco 부동 베이스 profile | `test_g1_loco_configure` (출하 floating profile 로 configure · fixed override 와 hold 동등 · cache base neutral) · `test_g1_loco_joint_order` (모델 · roster · MJCF actuator 세 출처 관절 순서) · `test_shipped_*` · `test_launch_*` 의 profile 목록 | `sim_g1_loco` 기동 → `run_stance_metrics` 10 s (문턱은 #977) · `/sim/reset` 뒤 재현 |
| `integrated_bringup/` locomotion_controller config (`model.*` · planner 조각의 계약) | `test_humanoid_model_config` (`model.*` 거부 규칙마다 통과 쌍 · 설치된 g1_loco 조각 · `RobotModelHandler` 인계 · 설치된 `whole_body_qp.*` → 보행 QP 1 tick 수렴) · `test_g1_loco_qp_iteration_budget` (설치된 `whole_body_qp.solver` 상한: 정지 · 한 발 해제 · 복귀에 절반 여유 · miss 뒤 cold 수렴 · 모순 QP 는 Fault · 비정상 상태 300 개가 200 × 500 과 같은 수렴) · `test_original_layout_adapter` (원본 ZYX layout ↔ free-flyer · weight 순열) · `test_locomotion_reference_config` (설치된 `command.*` · `gait.*` · `swing.*` · `terrain.*` · `reference.*` 조각 → `rtc_mpc` 로더, 접촉 이름 ↔ bit, q_default 가 출하 roster 전부) · `test_locomotion_mpc_config` (설치된 `mpc.*` 조각 → 손 잠근 부동 body 모델에 로드 · Q / Q_final / R 의 길이 = 모델 차원 · 유한 · 부호, `constraints:` 의 gain · 마찰 · CoP 사각형 · barrier 가 유한 · 양수 · 원점을 감쌈 (값은 고정하지 않음), 구 16 쌍을 원본 정의로 재구성 · solver 예산 · publish 규칙이 `rtc_mpc` 측정 사본과 같음 · `MPCFactory` 생성) | — |
| `integrated_bringup/` locomotion_controller | `test_locomotion_controller` (설치된 g1_loco + CM 모델로 configure · activate, 첫 tick 정지 QP 와 독립 EOM 대조, fault latch · reset · 재활성화 · E-STOP, 거부 규칙, 두 profile LoadConfig) · `test_locomotion_mpc` (stub handler 로 stand · 블록 복사 bit 동일 · 미수렴 publish plan 도 stand · online 유지 · 타당성 넷 · 걸린 plan 의 앞 plan 유지 (stub gate 로 한 plan 씩: 도착 즉시 · 따르던 중 거부 · 재검사 · idle 이 표본만 한 plan 은 불가 · 연속 거부의 나이 소멸 · idle 불변) · plan 상실의 재latch 와 `plan_loss` · swing mask · reset · E-STOP · 재활성의 state epoch · cleanup 상한 · 손 q_default · `mpc_off` 거부 · `mpc:` 없음 = disabled bit 동일 · 스키마 · Twist 명령 경로 (구독 → 필터 timer → planner 의 horizon, stale → 0, NaN 축 유지, 키 없음 / MPC 없음 → 구독 없음, 활성화 리셋) · 보행 판정 열 (명령 · gait · plan base 속도, 측정 접촉력의 신선 · group 수 · stride 가드) · `locomotion_status` 토픽 (level · 고정 키 · plan · 명령 · fault, 활성 전 침묵), 설치 MPC smoke) · `test_locomotion_fault` (E-STOP 중 tick 은 `waiting` · QP 0 · MPC 상태 쓰기 0 · 로그 매 tick; packed E-STOP word 와 두 edge; 자세 가드 순수 함수 (경계 ±1e-9 · NaN · 영 쿼터니언 · 비단위 · 접촉 NaN · π/2) 와 입구 넷 (활성화 · reset · clear · plan 상실) 의 settle 시계 · `pose_guard` · `pose_guard_mask`; fault 와 E-STOP 의 분리; 감쇠 oracle bit 동일 (NaN · 속도 lane 미가독 · unclamped); 손 hold 선latch · 덮어쓰기 없음; lever rule 바운드 (CoM 5 · 10 · 20 cm · 발 간격); Walk 중 plan 상실 감쇠 pin; `fault.pose_guard` 거부 · 출하값 · loco < demo_joint 게인 tripwire) · `test_locomotion_diag_contract` (60 tick 고정 run 의 diag 기존 열 이름 · 순서 · 행 수 = tick 수 · inf 없음 · 성질 범위 — 값은 고정하지 않고 뒤에 붙인 열은 무시) · `test_locomotion_truth_log` (T tier: raw base · 바닥→발 wrench (lane 의 부호 반전) · 외력 보고 lane (받은 값 그대로 · 낡으면 NaN · 빈 토픽이면 열 없음) · 모멘트 flag · 합성 wrench CoP · 문턱 · 신선도 / lane fit, 참값 lane 이 없는 2500 tick 은 파일 없음 · 행 수 = 참값 tick 수 · drop 0 · T 로그 유무의 출력 bit 동일) · `test_locomotion_controller_alloc` (모든 tick 경로 할당 0 — E-STOP · 가드 거부 · clear 첫 tick · MPC 경로 · inekf source 의 시작 · hold · 재시작 fault · timeout 포함) · `test_g1_loco_configure` 의 demo_joint PD 게인 · `test_loco_estop_on_clear` (CM 수준 두 컨트롤러: loco 중 운영자 E-STOP 의 hold slot · 검증된 해제 뒤 demo_joint 복귀와 출하 게인 (`initial_controller`) 또는 loco 재진입과 자기 게인 (`resume`) · 창 tick · 인계 틈 실측) · `test_estop_on_clear_profiles` (`estop_on_clear` 값은 CM 이 받는 정책이고 `initial_controller` 는 profile 이 가진 controller) · `test_locomotion_diag_names` (rtc_tools 이름표 == C++ enum) · `test_locomotion_joy_command` (Joy 매핑 bit 동일 · 0 규칙 · 설정 거부 · 표준 배치 C++ 사본 == 설치된 `rtc_teleop_input` 표 · 출하 joy overlay 파싱; ROS 경로와 정지 두 갈래는 `test_locomotion_mpc` 의 `LocomotionJoyCommandTest`) · `test_command_input` (`sim_g1_loco` 의 parameter 층 순서 · `command_input:=joy` 자리) · `test_teleop_profiles` (`teleop:=` · `command_input` auto, 두 profile 의 `teleop_input.yaml` 이 teleop 검증 통과 · 해제 사유 == `kOperatorEstopReason` · help 축 == joy overlay 매핑 축) · `test_sim_recover` · `test_sim_recover_ros` (stand 자세 · 궤적 시간 규칙 · 거절 · sim 시간 대기 · wall 가드) · `test_launch_description_evaluates` 의 teleop 경우 | `tools/standing_eval/gain_step.py` (매달린 step) · `with_verify_hold.sh tools/standing_eval/run_arms.py` (servo · loco · lowpd · mpc · mpc_fb · reentry · estop · walk · walk_estop, `--mpc-layout` · `--rtf` · `--mpc-threads`; arm 별 launch 인자 · 이벤트 스크립트 순서 · 응답 파싱 · 명령 스크립트 프로세스는 `test_standing_eval_run_arms`) → `plot_locomotion_diag` (`pose_guard_refused_rows` 0 · `fault_latches`) · `walk_gate.py` (보행 · walk_estop 판정, 카운터 줄 · CM 거부 · E-STOP edge · walk 주기 읽기는 `test_standing_eval_walk_gate`; 지표 · 주기 단위 창은 rtc_tools `test_walk_metrics`) · `estimator_gate.py` (추정기 batch 판정 — 규칙의 올림 · baseline · 정지 · 보행 항목이 하나씩 실패 · 판정 우선순위 · A/A 합성의 family 거짓 FAIL ≤ α · 정지 run 읽기 (stale lane 분리 · 참값 높이 창) · 상수가 문서와 같음은 `test_standing_eval_estimator_gate`) |
| `integrated_bringup/` locomotion 측정 상태 source | `test_measured_state_source` (이름 → q index 맵의 거부 규칙 · `estimator` 절 · truth source 의 bit 복사 · 상태 판정 순서 · 손 stale 은 비fault · Reset epoch · 1000 tick 난수 run 의 tick 마다 상태 판정 · 수락 출력의 유한 · 단위 쿼터니언 · 거부 출력의 무효) · `test_inekf_state_source` (설치된 g1_loco 모델로 inekf source: 정지 샘플 시작 · 창 N 과 재시작 · 관절 속도 · 계획 접촉 · hold 의 base bit 동일 → stale · k = 2 · 5 · 10 이 독립 순서기와 R · v · p · P bit 동일, k = 11 reset · sequence 역행 · IMU 누락 / 비유한 · skew 경계 ±1 ns (update 생략 · 무효) · reset 넷 (접촉 없음 · gating · 불일치 · 연속 거부) 과 우선순위 · 첫 stance 까지 tick 수 = FSM dwell · 기구학 oracle (full 모델 · base 두 pose · 회전된 IMU frame · 중앙차분) · 출력 사상 oracle (임의 extrinsic · σ·√T bit 동일) · 2 × 2000 tick 결정론 · `estimator.inekf` 거부 27 종과 통과 쌍 · root 밖 frame 거부) · `test_inekf_state_source_alloc` (source `Update()` 의 모든 경로에서 operator new · C malloc 0, 양성 대조) · `test_locomotion_estimator` (inekf 로 configure: 활성화 tick 시작 · latch · 창 3 의 대기 · 시작 timeout fault 와 E-STOP 중 정지 · latch 뒤 추정 재시작 = 그 tick `source` fault + MPC stream epoch +1 (3 회) · latch 전 재시작은 stream 만 · WBC · MPC snapshot · 측정 상태의 q · v bit 동일 · source 의 prior 는 직전 tick 접촉 mask · E-STOP · fault 중 estimator 열 갱신 · frame · 주기 · source 오타 configure 거부) | `with_verify_hold.sh tools/standing_eval/run_arms.py --source inekf` (정지 · 짧은 보행의 truth 대비 오차는 보고 항목) |
| `rtc_tools/` 보행 평가 (`walk_eval` · `binomial_stats` · `model_match` · `perturb_mjcf` — `test_perturb_mjcf`: 지정한 body 만, 질량 · 관성 같은 배율) · `integrated_bringup/` `run_arms.py` 의 provenance | `test_walk_eval` (합성 run: tier 가 자기 파일을 읽는다 · 없는 tier 와 없는 열은 거부 · 창과 개수 · fault 분류 · 순위 비용 · 유효성 항목이 하나씩 실패 · arm 요약과 검정 · 발 간격의 축 · 부호 · 거부) · `test_binomial_stats` (손으로 확인한 값 · 검출력) · `test_permutation_stats` (손으로 푼 KM RMST · 동률 · 거부 · p 의 형태 · 단측 · 재현 · A/A 크기 ≤ α) · `test_model_match` (맞는 쌍 · 한쪽에만 있는 body · 출하 G1 쌍 — mujoco 가 없으면 skip 이라 `.venv` python 으로 돌린다) · `test_standing_eval_run_arms` (config hash · overlay 기록 · 모델 기록의 실패 경로) | — |
| `integrated_bringup/` · `rtc_tools/` 상태 추정기 재생 (`estimator_replay` · `estimator_metrics`) | `test_estimator_replay`: 공유 본체 모델이 컨트롤러의 동역학 모델과 bit 동일 · profile `urdf` 절 → `ModelConfig` 와 거부 · 로그의 나이 · skew → 수신 시각 · 기록 입력을 그대로 넣은 재생이 online source 와 2000 tick bit 동일 (두 번) · 관절 noise 는 source 입력의 q 에만 (truth 접촉 속도 bit 불변 · 같은 seed 재현 · σ 통계 · σ = 0 은 draw 없음 · 거부) · rig 를 inekf 로 돌려 실제 writer 가 쓴 diag · `joint_ff` 의 재생이 기록된 이산 telemetry 와 일치, base 차 ≤ 1e-6. `test_measured_state_source` · `test_inekf_state_source` 의 기록 입력 test 도 여기에 든다. `rtc_tools` `test_estimator_metrics`: 알려진 오차 · gauge 를 넣은 합성 궤적의 tilt · yaw · 속도 · 위치 drift · NEES · NIS · 접촉 지표 · accel 잔차 · online 비교가 해석값과 1e-9, χ² 분위수 표, 보행 · 정지 부분의 NEES · gyro bias · 활성화 뒤 reset (cut 전만) · slip 순환 (0.5 s 창 경계 · 12 회 1.0 s 대 0.99 s · 11 회 · cut 뒤 포화) · online 추정의 같은 수 | batch 의 각 세션에 `estimator_replay` → `estimator_metrics` (inekf run 은 `--compare-online`) — 수치는 보고 항목이고 ANEES / ANIS 는 K run 의 χ² 구간 |
| `integrated_bringup/` b7_loco 부동 베이스 profile (군 셋, 모델은 비공개 패키지) | `test_shipped_*` · `test_launch_*` · `test_robot_profiles` 의 profile 목록 (configure gtest 는 없다 — in-repo 모델이 없어 fixture 를 둘 수 없다; keyframe lane 은 skip 이 정상) | `sim_b7_loco` 기동 → `run_stance_metrics` 11 s (문턱은 #980) · `/sim/reset` 뒤 재현 · 양손 `joint_goal` 추종 — 상태만 기록 (자산은 비공개) |
| `integrated_bringup/` p2 손 단독 profile (`hand_joint_controller`, 모델은 비공개 패키지) | `test_hand_joint_controller` (in-repo 대리 손 `p1b_surrogate` 단독 sidecar 위: 폐쇄 체인 g_a 가 퍼텐셜 에너지 유한차분과 1e-4 N·m 이내 · 관성항 M_a·q̈_ref 가 사영 유한차분 G 와 full RNEA 오라클과 상대 1e-3 이내 (plain 은 tree RNEA, q̈_ref 는 reference 의 2차 차분, gain 0 은 중력만 그대로, unreadable 은 reference 동결이라 관성 0, 로그 열 `qdd_ref_*` · `tau_inertia_*`) · hold 정책 · clamp · E-STOP · 목표 lane · 진행 중 goal 의 속도 연속 retarget (전환점 속도 연속 · off 는 정지-정지와 sample 동일 · 정지 goal 불변 · 한계 옆 반전은 정지 시작 · freeze 뒤 정지 시작) · 세 `mode` (damping 은 commands = 측정 · τ 0 · goal 거부 count, damping_feedforward 는 tracking 과 같은 τ · E-STOP 겹치면 0, 비-tracking 활성화 첫 tick valid, tracking 복귀 재시드) · 출하 YAML 파싱 · lifecycle 의 `mode` 파라미터 whitelist) · `test_hand_joint_alloc` (RT-1, 관성항 on 포함) · `test_udp_hand_native_backend` 의 command lane · `test_shipped_*` · `test_launch_*` · `test_robot_profiles` 의 profile 목록; `rtc_controllers` 의 `test_dynamics_feedforward_core` (관성 slot 포함) · `test_continuous_retarget` (임의 상태 sweep 의 cap · 한계 · 목표 통과를 조밀 샘플 오라클로); `rtc_tools` 의 `test_hand_gravity_ff_gate` (`mode` 열의 `g.mode` · 이동 창의 `g.at_rest`) · `test_plot_joint_ff` (`mode` 열 음영 · drift 통계 · `mode` 없는 CSV) · `test_demo_gui_hand_joint` (패널 게이트 · Send 가드 · Gains 등록 · C++ 헤더 mode 리터럴 교차검증) | `sim_p2` 기동 (왼손은 `side:=left`, 같은 절차 · 같은 기본 관절 목록 — 왼손의 `*_pip_link02_joint` 는 목록의 `|` 대안이 받는다) → mid 자세 `joint_goal` → `dynamics_feedforward.enabled` on/off/on 각 ≥ 10 s (**`mode: tracking` 에서만** — 판정 창의 비-tracking 행은 `g.mode` FAIL) → `analyze_hand_gravity_ff <session>` PASS (servo_kp 1: 큰 5 관절 off ≥ 10 mrad · on ≤ 1 mrad 이고 off 의 5 % 이하, pip 2 + 엄지 2 on ≤ 1 mrad, `index_mcp_rotate` 제외, `ring_mcp` 가 루프 판별 — 근거 실측은 #895 스레드) · `transforms` 의 `*_virtual_axis_actual` 4 frame · `held_ticks` 0 · `mode` 를 `damping` → `damping_feedforward` → `tracking` 으로 돌려 (GUI Hand Joint 패널 또는 `ros2 param set`) 손가락을 밀었을 때 damping 은 밀린 자리에 머물고 (τ_ff 0) damping_feedforward 는 τ_ff 가 tracking 과 같은 값이며 Send 가 막힘, `plot_rtc_log joint_ff.csv` 의 음영 · drift 가 그 구간과 일치 · `continuous_retarget` on/off 로 같은 wave sequence (`Next %` < 100) 를 돌려 `joint_ff.csv` 의 `q_ref_*` 차분에서 전환점 속도가 on 은 이어지고 off 는 0 으로 끊김 · 실기는 `robot_p2 sil_mode:=firmware` (왼손 `side:=left` → fake `hand_side` left) 에서 `/p2/<side>/joint_command.feedforward` 10 개 유한값 (실기 run 은 사용자) |
| `integrated_bringup/` demo GUI · profile | `test_demo_gui_dualarm` · `test_robot_profiles` · `test_demo_gui_presets` (profile 별 기본 preset · read-only 행) · `test_demo_gui_preset_speed` (preset 의 hand 속도: 컨트롤러별 파라미터 이름을 C++ 선언과 대조 · 상한 clamp · set 응답 뒤 발행) · `test_demo_gui_sequence` (Sequence 탭: step 시간 격자 · 시작 전 전수 검사 · once/loop 전이 · 중단 사유 · 손만 발행) | `demo_controller_gui --robot g1_p1b` · `--robot g1_loco` · `--robot b7_loco` · `--robot p2` · `--robot p2_left` (Tk 배치는 pytest 가 못 본다; p2 쌍은 `Import Mirrored` 뒤 거울 preset 이 이 손 range 안인지, 다른 손 tag 파일이 `.refused` 로 비켜나는지). preset: `Send Preset` 로그의 `takes >= … s` 와 실제 이동 시간, `ros2 param get` 의 속도 값과 Gains 패널의 그 행 · `Currently Applied` 일치. sequence: `Play once` 가 마지막 자세에서 끝나고 `Play loop` 가 `Stop` 까지 돌며, 재생 중 E-STOP · `Switch Controller` 가 사유와 함께 중단 |
| `integrated_bringup/` FK · arm tip · hand sensor_layout | `test_hand_fk_wiring` · `test_arm_tip_resolution` · `test_hand_sensor_layout_gate` | `iiwa7_leap` sim 에서 손 목표 smoke |
| `integrated_bringup/` compliance · catching | `test_compliance_admittance_coupling` · `test_demo_catching_controller` (configure — 사슬 탐색 · 관절군 · D9′) · `test_demo_catching_alloc` · `test_catching_search_segment_g1` (G1 탐색 · 구간 · 전신 항 — G1 + 합성 손 `robots/p1b_surrogate` 위) | sim 외부 렌치 → `compliance_diag.csv`; 17 관절 시간 기록은 `RTC_WHOLEBODY_TIMING=1` 로 단독 실행 |
| `integrated_bringup/` inference · observer | `test_demo_inference_controller` · `test_momentum_observer_wiring` · `test_payload_estimator` | `test_demo_inference_real_model` (로컬) |
| `integrated_bringup/` device backend (state lane) | `test_mujoco_native_backend` (command lane 의 PD 게인 — pd_feedforward · 선언 · 전 관절일 때만 kp/kd, 아니면 비움 · safe 재발행; joint · wrench · 부동 베이스 · IMU lane — IMU 는 site 별 raw 값 · valid · stamp · sequence, producer orientation 은 0 · false 를 씀, 한 site 의 drop · stale 이 다른 site 와 base 에 번지지 않음, 공유 / 분리 구독, lane 별 configure 거부; joint lane 의 수신 stamp · 메시지 counter) · `test_floating_base_conventions` (MuJoCo ↔ pinocchio 변환 한 곳 — 지향 케이스 + 1000 난수 round-trip ≤ 1e-12) · `test_udp_hand_native_backend` (sensor · command lane, joint 수신 stamp · counter) · `test_state_lane_close` (`CloseStateLane()` 뒤 8 lane 모두 불변 — joint lane 의 stamp · counter 포함, lane 을 추가하면 여기에 케이스를 더한다; ur_driver_native 의 joint 수신 stamp · counter) · `test_loco_imu_lane_shipped` (설치된 g1_loco · b7_loco `sim.yaml` 로 Configure → 부동 베이스 · IMU lane claim, `imu_state.site_names` == `model.imu.sites`) | sim 에서 `ros2 lifecycle set /integrated_rt_controller cleanup` 뒤 CM 로그에 `#941` ERROR 없음 |
| `udp_hand_driver/` | hand_packets · codec · FT · failure detector gtest + UDP loopback · CAN FD comm (`test_canfd_hand_comm`, vcan0 없으면 skip — skip 은 미검증) · lifecycle (proto2 configure 거부 · cleanup / on_error 뒤 재configure · stats 키 fixture · teardown 뒤 `failure_detected` 유지 · E-Stop 토픽 → zero-write 1회: `sil_mode=off` + 127.0.0.1 무응답 소켓 · 두 손 `hand_sides`: 토픽 접두 · E-Stop 하나 · 거부 · activate 롤백 · `req_mode auto` 해석 · broadcast 예산 = 슬롯 − `broadcast_tail_guard_ms` (T 무관)) · `test_udp_hand_launch_config` (launch 의 `params_file` 해석 · yaml `hand_sides` fallback → fake 손마다 · 엄격 로드 거부) · `test_hand_comm_loop` (손 순서 · 손 사이 E-Stop · `req_mode broadcast` 의 Prepare → REQ 하나 (실패 시 다음 손) → Receive → Finish 순서와 latch 재확인 자리) · CAN FD comm 의 broadcast REQ (요청의 deadline 이 bound · 지난 bound 의 큐 프레임 · write 뒤에도 같은 bound — 지연 주입) | `ros2 topic hz /p1a/joint_states`; proto2 SIL: `ros2 launch udp_hand_driver udp_hand.launch.py sil_mode:=firmware protocol_version:=2` → `/hand/link_status` true · `/hand/sensor_states` 의 `contact_flag`; 두 손 SIL 은 `hand_sides:=right,left loop_rate_hz:=400` (기본 broadcast REQ) → `/hand/right/link_status` · `/hand/left/link_status` true; robot 손 설정 yaml 은 `params_file:=<yaml> namespace:=<ns>` (b7_p2: `params_file:=$(ros2 pkg prefix integrated_bringup)/share/integrated_bringup/config/b7_p2/udp_hand_node_p2.yaml namespace:=/p2 sil_mode:=firmware` → fake 2개 · `/p2/right/link_status` · `/p2/left/link_status` true) (per_hand 는 `req_mode:=per_hand loop_rate_hz:=250 recv_timeout_ms:=0.9`); 실기 proto2 는 `recv_timeout_ms:=0.9 hand_side:=<side>` (H1-F06 실측값, none 펌웨어 실측에서도 0.9 유지; echo 펌웨어는 `cmd_ack:=echo`, 출하 0.4 는 매 사이클 타임아웃). **vcan0 에서 proto2 SIL 과 `colcon test` 를 동시에 돌리지 않는다** (`test_canfd_hand_comm` 이 같은 노드 id 공간을 쓴다). fixture 가 케이스마다 REQ probe 로 듣고, SIL (`fake_hand_firmware` 의 응답이나 송신 중인 노드) 이 보이면 vcan 케이스를 전부 FAIL 시킨다 ("proto2 hand traffic on vcan0") |
| `ur5e_bt_coordinator/` | BT gtest | 실제 grasp smoke |
| Launch · YAML config | `--ctest-args -R 'test_launch_'` + 변경한 YAML 의 parse · 범위 | 짧은 sim/실기 smoke |
| Threading (`ApplyThreadConfig`) | `rtc_base` thread-config gtest + RT 권한 | `check_rt_setup.sh --summary` |
| RT 회귀 의심 | `mpc_timing_log.csv` · `cm_timing_log.csv` p99 | `enable_tracing:=true` + Perfetto ([tracing.md](tracing.md)) |
| RT host 환경 | `check_rt_setup.sh --summary` + `verify_rt_runtime.sh` | `cyclictest --mlockall --smp -p 80 -i 200` · `rtla osnoise top` |

새 test 파일은 패키지 `CMakeLists.txt` 에 등록해야 `colcon test` 가 돌린다 (`repo_scripts/scripts/validate_test_registration.py --list` 로 확인). ROS 노드를 만드는 테스트는 패키지당 전용 `ROS_DOMAIN_ID` 가 필요하다 (`repo_scripts/scripts/validate_test_domains.py --list` 가 현재 배정을 보여 준다). robot 모델이 필요한 fixture 는 repo 안의 `robot_descriptions/robots/<name>/` 만 참조한다 (`repo_scripts/scripts/validate_test_fixtures.py --list`).

## Live Debug Topics

런타임 문제 탐지용. `ros2 topic echo` / `ros2 topic hz` / `ros2 bag record` 대상이다.

| Topic | 발행 주체 | 언제 보나 |
|---|---|---|
| `/system/estop_status` | `rtc_controller_manager` | E-STOP 원인 (timeout name / trigger thread) |
| `/rtc_cm/active_controller_name` | 동일 (TRANSIENT_LOCAL) | 컨트롤러 전환 확인. BT · GUI · digital_twin 이 이 토픽으로 다시 연결한다 |
| `/forward_position_controller/commands` | robot 모드 + `ur_driver_native` 전용 | `ros2 topic hz` 로 `control_rate` 확인. **sim 에는 없다** — sim 은 `devices.<group>.backend.command_topic` (예: `/ur5e/joint_command`) 을 본다 |
| `<session>/timing/cm_timing_log.csv` | CM RT loop | tick 별 8열 timing (`run_id`, `t_state_us` … `jitter_us`). p50/p99 는 후처리. `run_id` 로 먼저 묶는다. sim 에서는 `jitter_us` 가 항상 0. **RT 예산 판정은 이 CSV 로 한다** — `plot_rtc_log --stats` 의 `Overruns (> budget µs)` 행. 종료 요약 line 의 `overruns=` 는 sim 에서 센서가 아니다 (sim 동기화 경로가 overrun 판정을 우회해 `n/a(sim)` 으로 찍힌다) |
| `<session>/timing/mpc_timing_log.csv` · `rt_callback_timing_log.csv` | MPC thread · rt_callback thread | 같은 8열. MPC tick 당 1행 · device state 콜백당 1행. `rt_callback` 은 열 의미가 lane 고유다 (`t_state_us` = decode, `t_publish_us` = mailbox hand-off, `t_total_us` = slot 2 duty 분자) — dispatch 간격은 연속 행의 `t_wall_ns` 차분 |
| `/rtc_cm/{group}/joint_states` | CM (그룹별) | 그룹별 건강성 |
| `/sim/status` | `rtc_mujoco_sim` 1 Hz | sim 건강성 |
| `<contact_wrench.topic_prefix>/<target>/contact_wrench` | `rtc_mujoco_sim` | 손끝 접촉 힘/토크 (그룹 YAML `contact_wrench.enabled: true` + MJCF contact sensor 필요) |
| `/p1a/joint_states` 등 | `udp_hand_driver` | hand link 건강성 |
| `/shape/estimate` · `/shape/explore` (action) | `shape_estimation` | 형상 추정 결과 · 탐색 진행. 입력은 `<ns>/tof/snapshot` (`<ns>` = active 컨트롤러 이름) |

timing CSV 는 `ros2 run rtc_tools plot_rtc_log <csv> --stats` 로 읽는다.

## Debugging

| 증상 | 처방 |
|---|---|
| `ApplyThreadConfig()` 경고 | `sudo usermod -aG realtime $USER` 후 재로그인 |
| 시작하자마자 E-STOP | sim 이면 `init_timeout_sec: 0.0` |
| Jitter 큼 (> 200 us) | `check_rt_setup.sh --summary` → `cpu_shield.sh status` (격리는 `isolcpus` 가 아니라 cset shield). 단계별 점검은 [rt-optimization.md](rt-optimization.md#5-문제-해결) |
| Hand timeout E-STOP | UDP 링크 확인. `recv_timeout_ms` 는 로봇별 값이니 `integrated_bringup/config/<robot>/udp_hand_node_*.yaml` 을 본다 |
| TSID (dynamic) QP 가 거의 매 tick 실패 (`[wbc] Dynamic QP failed` 반복, solve 수 ms) | `wbc_diag.csv` 를 `plot_rtc_log --stats` 로 읽는다 — `QP converged` 비율, `qp_status` 이름별 히스토그램 (MAX_ITER / PRIMAL_INFEASIBLE 구분), `joint_box_inverted` 비율. inverted > 0 이면 관절이 TSID `joint_limit.position_margin` 띠 안에 있는 것이다 — 그 margin 은 position 백본의 `integration.position_margin` 보다 작아야 한다 (#551) |
| 관절 궤적이 "정착" 인지 "밴드에 pin" 인지 모르겠다 | 컨트롤러 로그의 `joint band engaged:` WARN 구간은 pin 이다. Ctrl-C 로 끝내면 요약이 없다 |
| `ament_cmake_test` missing | `.venv` overlay 가 system site-packages 를 가림. 재활성화 후 ROS 환경 재로드 |
| 0.01 초 만의 configure 실패 `Failed to find .../package.sh`, 빌드 직후 테스트 바이너리 없음 | 잘못된 cwd 로 colcon 을 돌린 것. repo 안 `build/`·`install/`·`log/` 삭제 후 workspace root 에서 재실행 |
| 방금 넣은 test 가 목록에 없다, 실패 줄 번호가 소스와 어긋난다, `cmake --build build/<pkg> --target <test>` 가 출력 없이 끝난다 | `--cmake-args` 없는 plain `colcon build` 가 `BUILD_TESTING=OFF` 로 재구성해 test 타깃을 없앴고, `build/<pkg>/` 의 **옛 test 바이너리**가 새로 설치된 라이브러리에 링크된 채 돈다. `grep BUILD_TESTING build/<pkg>/CMakeCache.txt` 로 확인하고 `./build.sh -p <pkg> --tests` 로 다시 짓는다 — test 결과는 test 수 · 바이너리 mtime 으로 새것인지 본다 |
| env 미source 로 전 바이너리 일괄 실패 | 회귀 아님. AGENTS.md §9.1 서브셸 표준형으로 재실행 |
| 지운 config · launch 를 테스트나 `ros2 launch` 가 아직 찾는다 | `--symlink-install` 이 끊긴 링크를 남긴다. `./build.sh -p <pkg>` 는 빌드 뒤 지운다 (`find install/<pkg> -xtype l` 로 확인) |
| `No rule to make target '/opt/ros/<distro>/lib/lib<X>.so.<옛 버전>'` | ROS apt 업그레이드 뒤 옛 경로가 남은 것. `colcon build --cmake-force-configure` ([repo_scripts/README.md](../repo_scripts/README.md)) |
| sim 을 다시 띄웠는데 옛 씬이 응답한다 | launch 프로세스만 죽이면 `mujoco_simulator_node` · `integrated_rt_controller` 가 남아 계속 발행한다. 재기동은 세 단계다: launch 를 죽이고 → 남은 노드를 `pidof` 로 찾아 PID 로 죽이고 → SIGKILL 이 남긴 `/dev/shm/fastrtps_*` 를 지운다 (Fast DDS 는 정상 종료할 때만 자기 세그먼트를 지운다) |
| 세션 CSV 의 끝이 잘렸다, 스크립트에서 `ros2 launch` 에 보낸 SIGINT 가 아무 일도 안 한다 | SIGINT 로 끝낸다 — `-9` 는 CSV flush 를 날린다. 비대화형 셸 (스크립트 · 에이전트) 이 `&` 로 띄운 job 은 SIGINT 를 SIG_IGN 으로 물려받고 `ros2 launch` 는 그대로 둔다. 노드는 자기 핸들러를 설치하므로 노드에 직접 `kill -INT $(pidof integrated_rt_controller mujoco_simulator_node)` (손 SIL: `udp_hand_node fake_hand_firmware` 둘 다) — launch 는 자식이 끝나면 따라 끝난다. 또는 스크립트에서 `set -m` 으로 띄우고 `kill -INT -- -<launch pid>` |
| `pkill -f <노드>` 가 자기 셸을 죽인다 (exit 144), `pgrep -f` 가 엉뚱한 PID 를 섞는다 | 명령을 `bash -c '…'` 로 감싸는 셸은 자기 커맨드라인에 그 패턴을 담는다. 노드는 `pidof <실행 파일 이름>` 으로 찾고 (`pgrep -x` 는 15 자에서 잘려 0 건이 된다), 직접 띄운 것은 띄울 때의 `$!` 로 죽인다 — `ros2 launch` · `ros2 topic pub` 은 실행 파일이 python 이라 `$!` 뿐이다 |
| 남겨 둔 평가 세션이 사라졌다 | launch 는 기동할 때 `logging_data/` 를 `max_log_sessions` (출하 10) 로 잘라 오래된 세션을 지운다. 세션을 모으는 동안에는 모든 기동에 `max_log_sessions:=<현재 개수 + 계획한 기동 수 + 여유>` 를 주고 (한 번 띄워 보는 확인용 기동도 같다 — 고정 상수는 폴더가 이미 차 있으면 모자라므로 `ls logging_data \| wc -l` 로 먼저 센다), 판정에 쓸 세션은 끝나는 즉시 다른 곳으로 복사한다. `run_arms.py` 는 이 인자를 스스로 넣지 않으니 `--launch-arg max_log_sessions:=<N>` 으로 주고, 그 `--logging-root` 는 세션을 **찾는** 곳일 뿐이라 launch 는 언제나 workspace 의 `logging_data/` 에 쓴다 |
| 한 세션 CSV 안에서 열 수가 중간에 바뀐다 (pandas `Expected N fields … saw M`) | 세션 이름은 `YYMMDD_HHMMSS` 라 **같은 초에 띄운 두 기동은 한 세션 폴더를 공유**하고, 로거는 있던 CSV 에 헤더 없이 이어 쓴다 (다른 로봇이면 λ 열 수가 다르다). #1156 이전의 `YYMMDD_HHMM` 세션은 같은 분이 같은 폴더다. 그 세션이면 기동 시각을 확인하고, 끝난 세션은 다음 기동 전에 복사해 둔다 |
| 켜 둔 sim 이 디스크를 채운다 | 활성 컨트롤러는 대기 중에도 tick 마다 CSV 를 쓴다 (무장한 catching 컨트롤러는 약 3.4 GB/h). 확인이 끝나면 바로 내린다. 정리할 때는 `YYMMDD_HHMMSS` 세션 폴더만 지운다 — 옆의 `*.json` · `stats/` · `ur_plot/` 은 사용자 파일이다 |
| sim 의 `<group>/joint_states` 구독이 0 메시지, "incompatible QoS" 경고 | sim 은 상태를 BEST_EFFORT depth 1 로 발행한다. 구독도 BEST_EFFORT (sensor-data QoS) 로 한다 |

```bash
# exec 이름 = 노드 이름 = integrated_rt_controller (integrated_bringup 의 RT 노드 실행파일)
PID=$(pidof -s integrated_rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
ros2 lifecycle list /integrated_rt_controller
ros2 topic echo /system/estop_status
./repo_scripts/scripts/check_rt_setup.sh --summary
```

## 실기 RT 점검

### RT 권한

```bash
sudo groupadd realtime && sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# 재로그인 필요. 나머지 RT 설정 (GRUB nohz_full · cpu_shield · IRQ) 은 ./install.sh robot --rt
```

### CPU Shield 검증

`cpu_shield.sh` 는 cpuset 을 만들기만 하고 런치가 CM 을 그 안으로 옮긴다. 격리가 실제로 서는지는 **실기** 에서만 확인된다. 기대값은 [repo_scripts/README.md](../repo_scripts/README.md) 의 코어 레이아웃 함수 (`get_cm_shield_cpus` · `get_role_slot` 등) 가 SSoT 이고 여기에 숫자를 적지 않는다.

```bash
sudo ./repo_scripts/scripts/cpu_shield.sh on --robot
cset shield -s                                   # user 집합 == get_cm_shield_cpus 출력
CM=$(pidof -s integrated_rt_controller)          # 런치(shield-on) 후
grep Cpus_allowed_list /proc/$CM/status          # 위와 동일해야 함
ps -eLo comm,psr,cls,rtprio -p $CM | grep -E "rt_control|rt_callback|nrt_"   # pin · FIFO (nrt_* 는 TS 가 정상)
grep -rE "rc=22|setaffinity failed|Thread config failed" ~/.ros/log/<run>/   # 결과 없어야 (EINVAL 회귀)
```

두 번째 런치 로그에 `CPU shield already active (cset user cpuset present)` 가 보이면 게이트가 활성 shield 를 다시 만들지 않은 것이다.

### CAN FD 손 게이트 (G-5 두 손)

한 run 은 [run_hand_canfd_gate.sh](../repo_scripts/scripts/run_hand_canfd_gate.sh) 하나다: 버스 기록 → 노드 기동 (`hand_sides`) → 손마다 `link_status` true 대기 → `--duration` 초 → `/hand/estop` true (run 은 E-Stop 으로 끝난다) → 노드에 SIGINT → 손마다 `analyze_hand_canfd_gate` → 요약 한 줄. 산출물은 `--out` (기본 `./hand_gate_<시각>`) 아래 `bus.log` · `node.log` · `session/` · `gate_<side>.md` · `.json` 이다.

```bash
# 개발 PC, vcan0 SIL (fake 둘) — colcon test 와 동시에 돌리지 않는다
./repo_scripts/scripts/run_hand_canfd_gate.sh --sil --duration 60
# 실기 (테스트 PC): can0 1 M / 8 M FD, 손 node id 1 · 2 — broadcast · 400 Hz 가 기본 (IXXAT USB)
./repo_scripts/scripts/run_hand_canfd_gate.sh --iface can0 --duration 300
# 네이티브 CAN 호스트 (PC#3 Jetson AGX Thor, mttcan can1): 500 Hz 가 운용값 (D89; broadcast 는 T 를 쓰지 않는다)
./repo_scripts/scripts/run_hand_canfd_gate.sh --iface can1 --duration 300 --rate 500
# per_hand (기본 250 Hz · T 0.9)
./repo_scripts/scripts/run_hand_canfd_gate.sh --iface can0 --duration 300 --req-mode per_hand
```

- 실기 전에 두 손이 각각 REQ 에 답하는지 본다: `candump can0` 을 띄우고 `cansend can0 00404000##1` (오른손) · `cansend can0 00408000##1` (왼손) 에 RESP 3 프레임 (`0080004x` · `0080008x`) 이 오는지
- PASS: 손마다 (a)–(e) PASS, (x) 0, link down 0. (c) 의 문턱은 주기 / 1.1 (400 Hz 2273 µs) — 미달이면 `--rate` 를 낮춰 다시 잰다 (per_hand 250 → 200, D77). 두 손 zero-write 간격 행은 보고만 한다 (D78)
- 종료 코드 0 PASS · 1 FAIL · 2 UNDECIDED (한 번 재측정) · 3 run 불가. candump 이 없는 PC 는 같은 형식의 python 기록기로 대신한다
- broadcast REQ (기본 — `--rate` 400 은 PC#2 IXXAT 실측; 수신 마감은 tick 시작 + (슬롯 − `broadcast_tail_guard_ms`) 라 `--recv-timeout` 은 per_hand 에서만 의미가 있다. 거부는 per_hand 의 `손 수 × 2 × T ≥ 주기` 뿐): 실기 전에 `cansend can0 007FC000##1` 한 번에 두 손의 RESP 6 프레임이 오는지 본다. 손마다 (a)–(e) 에 `x.no_overlap` 대신 `x.broadcast_sets` (같은 손 RESP idx 중복 0 · 버스트 중 호스트 CMD 0 · 한 손만 빠진 창 비율이 (d) 의 문턱). 버스의 REQ 가 모드와 다르면 판정기가 exit 3. 펌웨어 쪽 W2 전제가 바뀌면 증상은 한 손만의 세트 timeout 이다. **broadcast CMD (`003FC00x`) 는 손 설정을 flash 에 쓰므로 보내지 않는다**
- broadcast 채택 run (같은 날 연속 5 회): per_hand read-only (`--recv-timeout 0.9`) → broadcast read-only → per_hand write → broadcast write → per_hand read-only (처음과 끝의 차이가 재현 폭; per_hand run 은 `--req-mode per_hand`, 넷 다 `--rate 250`). write run 은 손마다 사인파 명령 발행기를 따로 띄운다 (두 write run 의 `write_pairs / total_cycles` 차이 ≤ 5 %). 기준: broadcast tick p99 ≤ 같은 종류 per_hand p99 − 0.2 ms. 2026-10-26 run 에서 넘겨 (read-only −391 · write −321 µs) 두 손 기본이 broadcast 가 됐다 (D86). 보고표에는 `REQ → 늦은 F2` 와 `첫 CMD → 늦은 F2` 를 같이 적는다
- 호스트별 운용값 (D89): 운용 주기는 write tick (CMD 4 → REQ → RESP 6, 노드 `t_total`) p99 × 1.1 로 잡는다 — (c) 문턱과 같은 식이다. PC#2 (IXXAT USB-to-CAN FD): write tick p99 1993 µs (재측정 전) → 400 Hz. PC#3 (Jetson AGX Thor, 네이티브 mttcan `can1`, 2026-10-28 run `hand_gate_261028_081819`, broadcast 450 Hz write tick): `t_total` p50 / p99 / max 1516 / 1521 / 1600 µs → 규칙상 ≈ 597 Hz, **운용 500 Hz** (문턱 1818 µs; 이전 450 Hz · T 1.0, D87)

### Kinodynamic MPC soak

`test_kinodynamic_mpc_soak` 는 평소 12 조합 (stance 정지 · walk rollout · walk 교란 + overrun × 60 Hz · 1/dt × SIE · RK2) 을 3 s 씩 smoke 로 돈다. smoke 는 publish deadline 을 끈다 (`RTC_MPC_SOAK_DEADLINE_MS` 로 켠다) — 모든 solve 가 deadline 을 받으므로 solve 가 주기보다 긴 호스트에서는 첫 plan 이 영영 안 나온다 (예산 삭제 전에는 standing solve 가 면제라 하나는 나왔다); 무장 run 은 한 주기 (제어 PC 판정). 무장 (`RTC_RUN_MPC_SOAK=1`) 하면 조합마다 `RTC_MPC_SOAK_SECONDS` (기본 600) 를 돌아 verify hook 의 test 상한을 넘으므로 `--run` 밖에서 `with_verify_hold.sh` 로 띄운다. PASS 는 failed solve 0 · 창 전체 online tick 의 평균 node shift (grid index 의 폭 ÷ tick 수) ≥ 0.9 × 주기의 node 수 (thread 가 주기보다 빨리 돌지 않음 — 느린 호스트는 평균이 커질 뿐이다) · warm-up (기본 60 s) 뒤 heap 증가 0 (누수 게이트) · 할당 수가 후반에 늘지 않음 — 무장 run 의 30 tick 이상 묶음에서 뒤 1/3 중앙값 ≤ 앞 1/3 최대; smoke 는 보고 (같은 (shift, iteration) 묶음도 값 자체는 line search trial 수로 흔들려 분포는 보고) 뿐이고, solve p50 / p99 / max · overrun · deadline miss · withheld 는 보고다. 요약은 `build/rtc_mpc/test_results/rtc_mpc/test_kinodynamic_mpc_soak.summary.{csv,json}` 에 남는다 (각 knob 과 지표는 테스트 파일 머리 주석).

**연산 시간 (주기 · online 예산 · 적분기) 은 제어 PC 두 대 (PC#1 NUC14 · PC#3 Thor) 의 무장 soak 로 판정한다** — 같은 커밋을 설치하고, MPC thread 를 격리 코어에 SCHED_FIFO 로 고정하며, 개발 PC 의 수치는 참고로만 나란히 둔다.

```bash
# <ws> = colcon workspace root (PC#1: ~/ros2_ws/ur5e_ws, PC#3: ~/ros2_ws/demo_ws), <repo> = 그 안의 이 repo 절대경로.
# <core> = 그 호스트의 격리 코어 (PC#3 은 isolcpus 4-5 중 하나), FIFO 60 = mpc_main 우선순위 — FIFO 는 코어 고정이 필수이고
# 상태 writer (테스트 thread) 는 자동으로 다른 코어에 고정된다 (RTC_MPC_SOAK_WRITER_CPU 로 지정 가능, 같은 코어는 거부).
# 병렬 solve 는 RTC_MPC_SOAK_SLOTS=<mpc slot>,<worker slot>,… (slot 의미, thread 하나에 하나 — 길이가 thread 수)
# 로 주고, 한 slot 이면 RTC_MPC_SOAK_CPU=<core> 와 같다. MPC thread 가 OpenMP team 크기를 스스로 정하므로
# OMP_NUM_THREADS 는 무관하다 (OMP_PROC_BIND · OMP_PLACES 가 있으면 pool 을 거부). writer 는 slot 전부를 피한다.
# RTC_MPC_SOAK_COMMIT=$(git -C <repo> rev-parse --short HEAD) 를 주면 요약에 commit 이 남는다.
# realtime 권한이 없으면 FIFO 0 (SCHED_OTHER) 으로 돌고, 실제 적용된 정책 · 우선순위 · worker 배치는 요약이 다시 읽어 적는다
( cd <ws> && source <repo>/repo_scripts/scripts/setup_env.sh >/dev/null 2>&1 && \
  RTC_RUN_MPC_SOAK=1 RTC_MPC_SOAK_SECONDS=600 RTC_MPC_SOAK_CPU=<core> \
  RTC_MPC_SOAK_FIFO_PRIORITY=60 <repo>/repo_scripts/scripts/with_verify_hold.sh \
  colcon test --packages-select rtc_mpc --ctest-args -R test_kinodynamic_mpc_soak \
  --event-handlers console_direct+ --return-code-on-test-failure )
```

조합 일부만 돌리려면 `GTEST_FILTER='*walk_rollout_1_over_dt*'` 를 더한다. soak 는 test 프로세스라 cset shield 의 "user" cpuset 으로 adopt 되지 않는다 — shield 가 켜져 있으면 격리 코어로의 고정이 EINVAL 로 실패해 pool 이 거부된다. shield 를 끄고 돌리거나 (`cpu_shield.sh off`) `cset shield --exec` 로 shield 안에서 띄운다. slot 후보는 `get_mpc_worker_slots <ncpu> <profile>` ([repo_scripts/README.md](../repo_scripts/README.md)). online 예산 · publish 규칙은 `RTC_MPC_SOAK_MAX_ITERS` · `RTC_MPC_SOAK_AL_ITERS` · `RTC_MPC_SOAK_DEADLINE_MS` · `RTC_MPC_SOAK_PRIM_TOL` 로 바꾼다 (기본 10 · 1 · 한 주기 · 출하 g1_loco 의 잔차 허용 — soak 자체의 기본값이다. 출하 online 예산 (`max_iters` 6 · AL 1 · `period_nodes` 2) 은 변수로 준다). 1/dt 조합의 주기는 `RTC_MPC_SOAK_PERIOD_NODES=<N>` (1 ≤ N < horizon node 수, 기본 1 — controller 의 `period_nodes` 와 같은 뜻) 로 1 / (N · dt) 가 된다. 조합 이름은 그대로이고 N 은 요약 JSON 의 `period_nodes` 에 남으니 N 마다 `RTC_MPC_SOAK_OUT_DIR` 을 따로 준다. 60 Hz 조합은 N 과 무관해 N ≥ 2 에서는 skip 한다. publish deadline 기본은 한 주기 (35 ms × N) 다. 요약의 행마다 `mean_shift_nodes` · `period_in_nodes` 가 남는다. timeout · crash 는 gtest XML 에 남지 않으니 ctest 요약 (`colcon test-result`) 으로 판정한다.

### Tracing

thread 별 core 사용이나 callback 소요 시간은 CSV 로 알 수 없다. LTTng 트레이스를 캡처해 본다. 명령 전체는 [tracing.md](tracing.md).

```bash
./install.sh --tracing                                                       # 1회 설정
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true   # 캡처
./repo_scripts/scripts/timeline.sh                                           # Perfetto JSON 변환
```

## Test 측정

테스트 카운트는 문서에 적지 않고 직접 잰다.

```bash
rm -rf build/<pkg>/test_results                                    # 누적 XML 제거
colcon test --packages-select <pkg> --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose --test-result-base build/<pkg>       # 패키지 하나 (--packages-select 는 없다)
( cd build/<pkg> && ctest -N )                                     # 등록 확인 — 새 테스트는 재빌드 뒤에 보인다
```

결과를 어떻게 판정하는지 (누적 XML · 종료 코드 · 범위와 단위 · timeout 과 crash) 는 [testing-debug.md](../agent_docs/testing-debug.md#test-측정) 가 갖고, 각 항이 실제로 잘못된 보고를 낳은 사례는 [testing-rationale.md](reference/testing-rationale.md) §계측 함정 에 있다.

### Mutation 루프

mutant 마다 패키지 전체를 다시 짓지 않는다 — 판정할 테스트 타깃만 기존 캐시로 빌드하고 그 테스트만 돌린다.

```bash
# workspace root, setup_env.sh 를 source 한 서브셸에서 (AGENTS.md §9.1)
cmake --build build/<pkg> --target <test 타깃들>
colcon test --packages-select <pkg> --ctest-args -R '^(<test 타깃들>)$'      # 판정: test_results/<pkg>/*.gtest.xml 의 <failure
( cd build/<pkg> && LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH ctest -R '^(<test 타깃들>)$' )   # 공유 라이브러리를 바꾼 mutant
```

루프를 믿기 전에 **무변형 대조 1 회** (테스트가 실제로 돌았는가) 와 **첫 mutant 가 red 인가** 를 본다. 아래는 모두 거짓 green · 거짓 red 를 낸 경로다.

| 함정 | 처방 |
|---|---|
| 빌드를 plain `colcon build` 로 하면 `.colcon/defaults.yaml` 의 `BUILD_TESTING=OFF` 가 트리를 재구성해 test 타깃이 사라지고, colcon 은 성공을 보고한다 | 루프의 빌드는 `cmake --build build/<pkg> --target` 만 쓴다. 구성 (새 `find_package` 포함) 은 `verify-changes.sh --run` 에 맡긴다 — `cmake -S/-B` 직접 재구성은 workspace install 을 못 찾는다 |
| 테스트 바이너리는 build 트리가 아니라 **install 트리의 `lib<pkg>.so`** 를 싣는다 (`ldd` 로 확인). `colcon test` 는 `LD_LIBRARY_PATH` 앞에 install 경로를 다시 붙인다 | `.so` 를 바꾼 mutant 는 위 셋째 줄의 `ctest` 로 돌린다 |
| 정적 라이브러리 패키지 (`ls build/<pkg>/*.a`) 는 test 바이너리에 라이브러리가 박힌다 | 루프가 끝나 원복한 뒤 라이브러리만이 아니라 **test 타깃까지** 다시 빌드한다 — 아니면 마지막 mutant 가 바이너리에 남는다 |
| 원복을 `cp -p` · `shutil.copy2` · `shutil.move` 로 하면 옛 mtime 이 되살아나 make 가 다시 컴파일하지 않는다 | 원복은 내용만 쓰고 `touch` 한다 |
| `git checkout -- <file>` 원복은 그 파일의 미커밋 수정까지 HEAD 로 되돌린다 | 루프 전에 고친 사본을 따로 두고 그것으로 원복한다 |
| 빌드 출력을 ` error ` 로 거르면 gcc 의 `error:` 가 빠져 컴파일 실패가 조용히 지나가고 옛 바이너리가 돈다 | 필터는 `error` 로, 판정은 테스트 출력으로 따로 한다. commit 을 같은 파이프의 `&&` 에 잇지 않는다 |
| 루프가 소스를 바꿔 끼우는 동안 띄운 리뷰는 그때 걸린 mutant 를 결함으로 보고한다 | 리뷰는 루프가 끝나 트리를 원복한 뒤에 띄운다 |

필터로 조용했던 mutant 만 넓은 바이너리로 다시 확인하고, 루프 뒤에는 원복한 소스로 `verify-changes.sh --run` 을 돌린다. 어떤 mutant 를 만들어야 하는지 (게이트의 fail-closed 확인) 는 [testing-debug.md](../agent_docs/testing-debug.md) 가 갖는다.

### 커버리지

C++ 는 `gcovr`, Python 은 `coverage` 를 쓴다. 정규 build/install 트리를 오염시키지 않도록 별도 base 를 쓴다.

```bash
uv pip install gcovr                                             # venv 안, 분석 전용
colcon build --packages-select <pkg> --build-base <scratch>/cov/build --install-base <scratch>/cov/install \
  --cmake-args -DENABLE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
colcon test --packages-select <pkg> --build-base <scratch>/cov/build --install-base <scratch>/cov/install
SRC=src/rtc-framework/<pkg>
gcovr --root "$SRC" --object-directory <scratch>/cov/build/<pkg> --filter "$SRC/src/" --print-summary
# Python: cd <pkg> && python3 -m coverage run --source=<module> -m pytest test/ -q && python3 -m coverage report -m
```

`ENABLE_COVERAGE` 는 패키지별 옵션이다. `rtc_mpc` · `integrated_bringup` 은 Debug 에서 일부 바이너리가 죽으니 [testing-rationale.md](reference/testing-rationale.md) §Coverage 함정 을 먼저 본다.

