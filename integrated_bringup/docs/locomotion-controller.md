# LocomotionController — 부동 베이스 전신 QP + MPC reference, `pd_feedforward` + 자기 PD 게인

부동 베이스 휴머노이드의 전신 컨트롤러다. 매 tick 측정 상태 → full 모델 kinematics (손 포함) → reduced 모델 dynamics (손을 neutral 에 잠근 질량) → 전신 QP (`rtc_tsid` `LocomotionFormulationBuilder`) → 본체 군에 `pd_feedforward` (위치 목표 + QP 토크) 를 낸다. 손 군은 `kPosition` hold 한다. reference 는 `mpc.enabled` 면 **MPC (`rtc_mpc` kinodynamic) 의 plan** 을 tick 마다 샘플한 것이고, 아니면 측정 자세를 latch 한 고정 reference 다. 지금은 서 있기만 한다 (속도 명령 · 보행은 이후). 계약 · 상태 전이 · fault 원인의 SSoT 는 헤더 `include/integrated_bringup/controllers/locomotion_controller.hpp` 다.

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

## 모드

| 모드 | 언제 | 출력 |
|---|---|---|
| waiting | 활성화 뒤 아직 유효 상태가 없음 | 감쇠 (아래) |
| nominal | QP 가 풀림 | 위치 = reference, feedforward = QP τ |
| degraded | QP 실패 또는 짧은 상태 불량 | 직전 유효 τ 유지 |
| fault (latch) | QP health Fault · 상태 불량이 `fault.max_source_bad_ticks` 초과 · 출력 비유한 · 모델 epoch 불일치 · 정지 양발 지지가 아닌 채 plan 상실 | 감쇠: feedforward = −`damping_kd`·v (토크 한계로 clamp), 위치 = 측정 q, 게인 불변 |

## reference 출처 (`mpc.enabled` 일 때)

| 출처 | 언제 | reference |
|---|---|---|
| idle | MPC 없음 · 쓸 수 있는 plan 없음 | latch 자세 (위) |
| stand | plan 이 쓸 수 있고 (유한 · 현재 model / state epoch · `max_policy_age_s` 안 · 타당성 통과) 수렴하지 않은 standing 해가 아니며 모든 접촉이 지지 | plan 의 q · v · a · 접촉 wrench 를 블록 그대로 |
| walk | stand 와 같고 접촉 하나가 swing | 같은 복사 + 그 접촉을 QP 에서 뺀다 |
| fault | 아래 latch | 없음 |

- online 해는 미수렴이어도 stand 를 유지한다 (publish 규칙이 잔차를 묶는다). stand · walk → idle 은 plan 을 **잃었을 때만** 이다 (나이 · epoch · 타당성). 그때 모든 접촉 frame 이 최저점 + 15 mm 안이고 base 가 0.1 m/s 아래면 측정 자세를 다시 latch 하고, 아니면 fault (`plan_loss`) — 떠 있는 발에 하중을 주지 않기 위해서다.
- 타당성 검사 (`mpc.sanity`) 에 걸린 plan 은 fault 가 아니라 idle 이고, 그 plan 은 끝까지 다시 쓰지 않는다.
- reset · 활성화 · E-STOP 두 edge 는 MPC 의 상태 stream (state epoch) 과 gait 를 다시 시작한다 — 옛 stream 의 plan 은 다시 샘플되지 않는다. E-STOP 동안은 stand 로 넘어가지 않는다.
- standing 해 (첫 solve) 는 수 초 걸릴 수 있고 그동안은 idle (latch 자세) 이다. online solve 가 publish deadline 을 못 맞추는 호스트에서는 stand 에 이르지 못한다 — 예산은 제어 PC 의 soak 가 정한다.

latch 는 `/rtc_cm/reset_fault` 나 **재활성화**로 풀린다 (다음 tick 에 reference 를 다시 잡는다). 원인이 남아 있으면 다시 latch 된다. E-STOP 의 두 edge 도 reference 를 다시 잡는다.

## YAML (`config/<robot>/controllers/locomotion_controller.yaml` + `locomotion/`)

| 조각 | 키 |
|---|---|
| `model.yaml` | `model.*` — 본체 군 · 손 군 · 접촉 · 몸통 · 팔 끝 frame |
| `runtime.yaml` | `command_type: pd_feedforward` · `pd_feedforward {kp, kd}` (본체 roster 순) · `fault {max_source_bad_ticks, damping_kd}` · `estimator {source: truth, body_max_age_ms, hand_max_age_ms}` · `topics` (본체 → 손 순) · `logs` |
| `whole_body_qp.yaml` | `whole_body_qp.*` — task · 제약 · `health {max_invalid_ticks, recover_valid_ticks}` · `com` task (수평 CoM 고정) |
| `mpc.yaml` | `mpc.*` — 소비자 키 `enabled` · `period_nodes` · `max_policy_age_s` · `riccati {enabled, …}` · `sanity {joint_delta_rad, base_speed, joint_accel, wrench_scale}` (`mpc:` 가 있으면 전부 필수, 의미는 헤더) + OCP (`kinodynamic`) · `solver` |
| `command.yaml` · `gait.yaml` · `swing.yaml` · `reference.yaml` | `mpc.enabled` 일 때 MPC 의 stage reference 로 조립 — `reference.q_default` 의 손 관절은 MPC 동안 손의 hold 자세 |

`b7_loco` 의 값은 `g1_loco` 의 것을 옮긴 잠정값이다 (`mpc:` 없음 = MPC 없음).

## 로그와 판정

`<session>/controllers/locomotion_controller/` 에 `locomotion_diag.csv` (모드 · fault 원인 · 상태 판정 · QP 결과 · margin · base · CoM · reference 출처 · plan id · policy age · 수렴 · 접촉 mask · 타당성 거부 수 · state epoch · 발 · 접촉력 · 관절별 servo 몫, 열 계약은 `logging/locomotion_diag_log_pod.hpp`) · `joint_ff.csv` · 군별 `<group>_state.csv` 가 쓰인다.

```bash
ros2 run rtc_tools plot_locomotion_diag <session> --z-ref 0.764431 --stance-csv stance.csv
```

그림 7 장과 `locomotion_diag_summary.json` (MPC 열이 있으면 stand 비율 · `time_to_stand_s` · policy age 분포 · plan 의 base z 평균) 은 `rtc_tools` README §locomotion_diag 를 본다. MPC thread 의 tick 시간은 `<session>/timing/mpc_timing_log.csv`, solve · 거부 카운터는 10 s 마다 INFO 한 줄이다.

## 평가 도구 (`tools/standing_eval/`)

- `gain_step.py` — 매달린 로봇 (base 고정) 에서 관절마다 0.2 rad step: 2 s 도달률 · overshoot 로 PD 게인 후보를 거른다 (venv python, mujoco).
- `run_arms.py` — sim 을 run 마다 새로 띄워 같은 시간축 (기동 → settle → 전환 단계 → lead → recorder) 으로 `servo` (MJCF servo) · `loco` (전환) · `lowpd` (demo_joint 에 같은 PD 게인, feedforward 없음) · `mpc` (`enable_mpc:=true` 로 전환) · `mpc_fb` (+ `mpc_riccati` overlay) 를 반복하고 (`--mpc-layout` 이면 모든 arm 이 같은 MPC layout, `loco` 는 `mpc_off` overlay; `--overlay ARM=…` · `--launch-arg` 로 바꾼다) recorder CSV/JSON · 로그 세션을 모은다. `repo_scripts/scripts/with_verify_hold.sh` 아래에서 돌린다.
