# DemoWbcController

UR5e + 10-DoF 손을 단일 16-DoF 모델로 묶은 whole-body controller 다. 6-단계 FSM 이 phase 마다 task 가중치와 contact 활성화를 자동으로 바꾼다. config_key 는 `demo_wbc_controller`, YAML 은 `ur5e_p1a` · `ur5e_p1b` · `iiwa7_leap` 이 싣는다 (`config/<robot>/controllers/demo_wbc_controller.yaml`).

이 문서는 제어 구조 요약과 bringup 관점 (YAML 위치 · launch 사용법 · 런타임 게인) 을 다룬다. 컨트롤러 계약은 [agent_docs/controllers.md](../../agent_docs/controllers.md), Kinematic / Dynamic 분리의 근거는 [controllers-rationale.md](../../docs/reference/controllers-rationale.md#demowbccontroller-의-kinematic--dynamic-분리) 에 있다. 공통 파라미터와 팔 끝 · 손끝 배선은 [demo-shared-params.md](demo-shared-params.md) 를 본다.

## 제어 구조

position command 의 유일한 백본은 CLIK-QP (`rtc::tsid::ClikReferenceGenerator`, [clik_reference.hpp](../../rtc_tsid/include/rtc_tsid/kinematics/clik_reference.hpp)) 다. 관절속도를 결정변수로 하는 box QP 가 TCP 추종 항과 arm/hand posture null-space 항을 함께 풀어 `v_ref` 를 만들고, `q_ref = q_anchor + v_ref·dt` 로 한 스텝 적분해 위치 명령으로 쓴다. TSID inverse-dynamics QP 는 위치 명령을 만들지 않는다 — 접촉력 · 토크 해석과 진단, 그리고 선택적 hand feedforward torque (`hand_tauff_enable`, 기본 false) 의 출처로만 쓴다.

```text
device state + fingertip inference + target
  ├─ ReadState / DrainTargetSlot (non-RT target → RT snapshot) / UpdatePhase (FSM)
  └─ 정상 phase
       ├─ Common stage: Pinocchio M/h/g/J, contact activation, grasp matrix G,
       │                posture ref (MPC 또는 phase hold), force PI → λ_des, TCP quintic ref
       ├─ Kinematic WBC (CLIK-QP) ──► q_ref, v_ref ──► position command
       └─ Dynamic WBC (TSID WQP)  ──► a_opt, λ_opt, τ ──► 진단 + 선택적 hand τ_ff
```

- 두 QP 는 직렬이 아니다. 같은 measured state · cache · reference 를 각각 소비한다.
- **CLIK 실패**: 이번 tick 은 이전 command 를 유지하고, 연속 `fsm.max_qp_fail_before_fallback` 번 실패하면 `kFallback` 으로 전환한다.
- **TSID 실패**: position 은 그대로이고 이번 tick 의 hand τ_ff 만 0 이 된다 (plain position-PD).
- hand τ_ff 는 `kClosure` / `kHold` 에서만 켜지며 source 는 `gravity_comp` 또는 `tsid_tau` 다. arm 은 torque mode 로 바뀌지 않는다.
- extended (closed-chain) 손에서는 제어 모델로 `PinocchioModelBuilder::GetActuatedModel()` 을 우선 쓰고 EOM 은 `WbcReducedDynamicsProvider` 가 loop-consistent 값으로 대체한다. 비-extended 손은 `GetTreeModel("wbc")` 경로로 이전과 같게 동작한다.

### MPC 범위

MPC 는 별도 thread 에서 돌며 RT 는 보간된 `q_ref,v_ref` 를 posture reference 로 소비한다. MPC 가 CLIK 의 위치 매핑을 대체하지는 않는다. object-level task 는 plumbing 수준이다 — 물체 pose estimator 가 없고, `object_mass` 기본값 0 이면 중력 reference 도 0 이며, internal squeeze reference 는 0 이다.

## 6-Phase FSM (slots 2 & 5 reserved)

모든 비-fallback phase 에서 위치는 CLIK-QP 가 만들고 TSID QP 는 해석과 hand τ_ff 용으로 함께 돈다. `grasp_cmd=2` (RELEASE) 는 active grasp phase (`kApproach` · `kClosure` · `kHold`) 에서 즉시 `kRelease` 로, `grasp_cmd=0` (abort) 은 `kIdle` 로 돌린다. 슬롯 2 · 5 는 reserved 라 publish 되지 않는다. GUI 의 arm SE3 target 과 hand joint target 은 `kRelease` · `kFallback` 을 뺀 모든 phase 에서 매 tick 반영된다.

| Phase | 제어 | 진입 조건 | 종료 조건 |
|---|---|---|---|
| `kIdle` (0) | SE3 hold @ 현재 TCP + posture regulate @ 초기 스냅샷. 모든 device 가 valid 상태를 스트림할 때까지 passthrough hold | 초기 / `grasp_cmd=0` / `release_done_` | `grasp_cmd=1` + `robot_new_target` |
| `kApproach` (1) | SE3 quintic ramp → grasp pose | `kIdle` 종료 | `tcp_goal_valid && ‖tcp_err‖ < epsilon_pregrasp` (TCP 와 목표 모두 `root_link` 기준) |
| `kClosure` (3) | contact + ForceTask, hand τ_ff 후보 | `kApproach` 종료 | active fingertip force ≥ N 개 (`min_contacts_for_hold`) |
| `kHold` (4) | contact + ForceTask, hand τ_ff 후보 | `kClosure` 종료 | slip 감지 시 `kFallback` (RELEASE preempt 는 top-level guard) |
| `kRelease` (6) | SE3 hold + 2-stage overlay (contact ramp `release_ramp_sec` → finger open) | active grasp phase 에서 `grasp_cmd=2` | `release_done_` (stage 1 hand trajectory 완료) |
| `kFallback` (7) | position hold | QP 연속 실패 N 회, slip | `grasp_cmd=0` (수동 복구) |

## YAML

주요 최상위 키: `tsid.tasks` · `tsid.constraints` · `tsid.contacts.*` · `tsid.force_pi` · `tsid.object_frame` · `tsid.phase_presets` · `tsid.wqp.solver`, `integration`, `clik`, `fsm`, `hand_tauff_*`, 그리고 필수 키 `arm_dof` 와 `estop.arm_safe_position` (길이가 `arm_dof` 와 일치해야 하며 불일치면 configure 에서 throw). 각 키의 의미 · 기본값 · 제약은 YAML 의 인라인 주석이 SSoT 다. `mpc.enabled: false` 가 출하값이라 MPC 는 inert 이고 posture reference 는 현재 자세를 유지한다 (self-hold).

운영상 알아 둘 것:

- phase preset 은 reset-style 이다. 이전 phase 에서 켠 task 를 끄려면 생략이 아니라 명시적 `active: false` 가 필요하다.
- `se3_tcp.name` 은 YAML key 와 같은 `se3_tcp` 여야 한다 (task lookup 과 phase preset 이 이 이름을 쓴다).
- friction face 수는 constraint 블록이 아니라 각 contact 의 `n_faces` / `friction_faces` 에서 읽는다.
- 기본 config 에서 hard `contact_constraint` 는 꺼져 있다. 강체 no-slip 을 hard 로 바꾸려면 명시적으로 추가하고 feasibility 를 확인한다.
- hold slip 감지는 `d|F|/dt` 단일 threshold 다. noisy tactile 에서는 filter · threshold 와 fallback 민감도를 함께 조정한다.
- TSID 는 `hand_tauff_enable` 이 꺼져 있어도 정상 phase 마다 풀린다. RT 예산을 평가할 때 이 비용이 들어간다.
- E-STOP 은 FSM 보다 우선하며 tick 시작에서 stale hand feedforward 활성 플래그를 지운다.

## 런타임 게인

노드 FQN 은 `/demo_wbc_controller/demo_wbc_controller` 다.

| 파라미터 | 타입 | 범위 · 비고 |
|---|---|---|
| `arm_trajectory_speed` | double | `[1e-6, ∞)` clamp. 현재는 hand quintic 외 미사용 |
| `hand_trajectory_speed` | double | `[1e-6, ∞)` clamp. `kRelease` stage 1 finger-open quintic duration |
| `arm_max_traj_velocity` / `hand_max_traj_velocity` | double (read-only) | trajectory 진행 중 최대 속도 [rad/s] |
| `tcp_trajectory_speed` | double | `[1e-6, ∞)` clamp. MPC-disabled SE3 ramp 의 TCP 병진 속도 [m/s] |
| `tcp_trajectory_angular_speed` | double | `[1e-6, ∞)` clamp. TCP 각속도 [rad/s] |
| `tcp_max_traj_velocity` / `tcp_max_traj_angular_velocity` | double | TCP 속도 cap (quintic peak = `1.875·d/T`) |
| `pi_rotation_margin` | double | π-rotation 임계 [rad]. `ang_dist > π − margin` 이면 중간 pose 로 분할 |
| `se3_weight` · `force_weight` · `posture_weight` | double | TSID `SE3Task` · `ForceTask` · `PostureTask` 가중치 (런타임 튜닝) |
| `mpc_enable` | bool | MPC 출력 소비 토글. YAML `mpc.enabled` 와 AND 라서 YAML 이 false 면 무시된다 |
| `riccati_gain_scale` | double | `[0, 1]` 자동 clamp. `RiccatiFeedback` 이 `scale · K · Δx` 로 적용 |

Force-PI grasp 는 파라미터가 아니라 one-shot 서비스 `/demo_wbc_controller/grasp_command` ([rtc_msgs/srv/GraspCommand](../../rtc_msgs/srv/GraspCommand.srv)) 로 한다 — `command=GRASP/RELEASE` + `target_force`.

## MPC 통합

`mpc.enabled: true` 면 `on_activate` 에서 aux 스레드로 MPC 스레드가 뜬다. `mpc.engine` 이 `"mock"` (선형 trajectory + identity Riccati gain, self-hold 와 bit-identical) 또는 `"handler"` (`HandlerMPCThread` + `MPCFactory` + `GraspPhaseManager`, 실제 Aligator ProxDDP solve, 초기화 실패 시 mock 으로 폴백) 를 고른다. solution 이 없거나 stale 이면 posture reference 는 현재 자세 유지로 폴백한다. per-MPC-tick 샘플은 `<session>/timing/mpc_timing_log.csv` 에 쌓인다.

`engine: "handler"` 일 때 WBC FSM 이 authoritative 이고 grasp 측 FSM 은 `GraspPhaseManager::ForcePhase` 로 동기화된다. phase 별 OCP 설정은 `config/ur5e_p1a/controllers/mpc/phase_config.yaml`, factory 설정은 `mpc/contact_light.yaml` · `mpc/contact_rich.yaml` 에서 읽는다 (두 파일의 `mpc.model:` 블록은 구조가 같아야 한다).

## 사용법 (시뮬레이션)

```bash
# MPC 비활성 (posture self-hold)
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py initial_controller:=demo_wbc_controller

# MPC 활성
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py \
    initial_controller:=demo_wbc_controller enable_mpc:=true

# 1. Pre-grasp 위치로 approach (target 토픽은 active controller 가 소유)
ros2 topic pub /demo_wbc_controller/ur5e/joint_goal rtc_msgs/RobotTarget \
  "{joint_target: [0.0, -1.2, 1.0, -1.4, -1.57, 0.0]}" -1

# 2. Hand close target
ros2 topic pub /demo_wbc_controller/p1a/joint_goal rtc_msgs/RobotTarget \
  "{joint_target: [0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6]}" -1

# 3. grasp 시작 (Force-PI one-shot) + 가중치 튜닝
ros2 service call /demo_wbc_controller/grasp_command \
  rtc_msgs/srv/GraspCommand "{command: 1, target_force: 2.0}"
ros2 param set /demo_wbc_controller/demo_wbc_controller se3_weight 100.0
ros2 param set /demo_wbc_controller/demo_wbc_controller force_weight 10.0

# 4. MPC 토글 + Riccati gain 축소
ros2 param set /demo_wbc_controller/demo_wbc_controller mpc_enable true
ros2 param set /demo_wbc_controller/demo_wbc_controller riccati_gain_scale 0.5

# 5. Release
ros2 service call /demo_wbc_controller/grasp_command \
  rtc_msgs/srv/GraspCommand "{command: 2, target_force: 0.0}"
```

`sim_ur5e_p1a.launch.py` 는 `enable_mpc` 를 `demo_wbc_controller.mpc.enabled` override 로 주입해 launch 시점에 반영한다. `robot_ur5e_p1a.launch.py` 의 `enable_mpc` 는 layout profile 선택만 하므로 실기에서는 런타임 `mpc_enable` 로 토글한다.

## Anomaly 보호

- **Slip**: EMA 로 거른 `|df/dt|` > `fsm.slip_rate_threshold` (5.0 N/s).
- **Deformation**: `‖displacement‖` > `fsm.deformation_threshold` (0.015 m).
- **QP 실패**: position 백본인 Kinematic (CLIK) QP 의 연속 실패만 critical 이다 — 연속 `fsm.max_qp_fail_before_fallback` 회 (5) 면 `kFallback`. Dynamic (TSID) QP 실패는 그 tick 의 hand τ_ff 만 버리고 `kFallback` 은 가지 않는다. 카운터는 `wbc_diag.csv` 의 `qp_fail_count` (dynamic) 와 `kin_qp_fail_count` 컬럼에 기록된다.

## E-STOP

`estop.arm_safe_position` 으로 이동하고 손은 현재 위치를 유지하며 contact 를 비활성화한다.

## 로그

`device_wbc_log` · `wbc_diag_log` 등 이 컨트롤러가 쓰는 CSV 는 [controller-logs.md](controller-logs.md) 를 본다.
