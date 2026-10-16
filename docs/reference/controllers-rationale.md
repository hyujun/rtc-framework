# Controllers — 근거·실측·이력

> **이 문서는 헌법이 아니다.** 규범은 [agent_docs/controllers.md](../../agent_docs/controllers.md) 가 갖고, 사용법은 [docs/controllers.md](../controllers.md) 가 갖는다. 여기는 컨트롤러 구조가 *왜* 지금 모양인지, 무엇을 실측했는지를 기록한다. 둘이 어긋나면 agent_docs 가 옳다. 수치·이름은 **기록 시점의 것**이라 인용 전에 다시 잰다.

## 알고리즘 코어 추출 이력 (#236)

한때 `rtc_controllers` 에는 법칙과 `RTControllerInterface` 어댑터가 섞여 있었다. 어댑터는 어디에도 등록되지 않아 런타임 노출이 0 이었고 코어 추출의 cross-check oracle 로만 살았다. 그래서 법칙을 무상태 free function 으로 뽑은 뒤(S1~S6) 어댑터를 삭제했다(S7c). 지금 `rtc_controllers` 에 `RTControllerInterface` 구현은 없고, 확인은 `grep -rn "public RTControllerInterface" rtc_controllers/include` 가 비는지로 한다.

| 단계 | 한 일 |
|---|---|
| S1 | Joint PD 를 `joint/joint_pd_law.hpp` 로. 궤적 **샘플**을 받고 생성기를 소유하지 않는다. 오차 이력은 in/out span (E-STOP hold 정책과 공유) |
| S2 | OSC 의 태스크 가속도 `a = K_p·e + K_d·(ν_d−ν) + a_ff` 를 `task/task_accel_law.hpp` 로. Λ/τ/Nᵀ 는 `compliance/task_dynamics` 로 흡수 |
| S3 | CLIK 의 태스크 속도 `K_p ⊙ e + ν_ff` 를 `task/task_vel_law.hpp` 로 (6축·병진전용 두 함수 — 인라인 식이 달라 합치지 않았다). DLS `J⁺`/영공간은 `compliance/differential_ik` 로 흡수 |
| S4 | 관성 성형 `f_cmd = B·f_task + (B−I)·f_ext` (`B = Λ_S Λ_d⁻¹`) 를 `compliance/inertia_shaping.hpp` 로. `Λ_S` 는 인자로 받아 `TaskDynamics` 를 모른다 |
| S5 | Task admittance 는 새 코어 없이 compliance 코어 + `task_vel_law` 재사용 |
| S6 | 영공간 자세 법칙을 `joint/posture_law.hpp` 로 (cascade 의 인라인 루프와 문자 그대로 같았던 것이 ARCH-3 발동 조건). cascaded 의 대역폭 판정식은 `compliance/bandwidth_separation.hpp` |
| S7 | 어댑터 삭제. 공통 글루(target mailbox 등)는 `rtc_controller_interface` 로 올렸고, 배치 고유 글루는 어댑터와 함께 삭제 |

### 결정

- **D-Q1 P 컨트롤러 은퇴** — 법칙이 `q + kp·e·dt` + 클램프라 파일 하나 값어치가 없다. 관절 PD 를 `kd = 0` 으로 쓰면 같다.
- **D-Q2 Λ/τ/Nᵀ 흡수** — 다섯 컨트롤러가 같은 `TaskDynamics` 를 쓰게 해 λ 규약을 compliance-conventions.md §6.5 로 통일했다.
- **D-S2b (b)안 비-bit-identical 이관** — 이관 전후 출력이 달라지는 것을 받아들였다. λ 규약 통일이 그 목적이었기 때문이다.
- **게인 단위가 법칙마다 다르다** — CLIK 은 속도형 `[1/s]`, OSC 는 가속도형 `[1/s²]`, impedance 는 힘형(Λ 배 차이). 합치지 않는다.
- **S6b `ImpedanceParams` 중첩** — `Gains` 가 `compliance::ImpedanceParams` 를 중첩해 가진다. 기본값이 코어와 같아 중복이 실제로 사라졌고 매 tick positional 재조립도 없어졌다. YAML 키는 불변.
- **Cascaded 의 wrench 는 정확히 한 번** 소비한다 — inner 에 `f_ext` 항이 없다 (compliance-conventions.md §7.6 MUST-4 를 런타임 검사가 아니라 타입으로, D19). 대역폭 비 `ω_i/ω_a` 는 seeding tick 에 한 번 평가해 fault 가 아닌 진단 플래그 `bandwidth_ratio_low` 로만 보고한다 (D20).
- **Task impedance 는 Λ 를 쓰지 않는다** (특이점 자유, σ_min-adaptive DLS nullspace) — impedance 와 admittance 는 **반대 부호**다 (compliance-conventions.md §11.4.1 P2/P3).

## 바인딩 구조 근거

### 팔 끝 frame 과 재정렬 함정

- `urdf.tip_link` 가 풀리지 않으면 joint · task · compliance · wbc 는 `on_configure` 를 거부한다. 그대로 돌면 팔 끝이 아닌 pose(정상 tick 은 항등, E-STOP tick 은 universe frame)를 팔 끝으로 유효 표시해 내고, wbc 는 Cartesian hold 의 seed 를 universe frame 에서 읽는다. 이름이 모델 frame 이기만 하면 통과하므로 팔 끝이 아닌 link 를 적은 구성은 못 잡는다.
- device 순서로 만든 항(null-space · Coriolis · 관절속도 `q̇`)은 Pinocchio 순서 행렬과 곱하기 직전 `RtModelHandle::ReorderInput` 으로 한 번 gather 해야 한다. 어기면 모든 수가 유한한 채 토크만 틀린다. `ν = J·q̇` 의 `q̇` 를 빠뜨리면 감쇠 토크가 틀리면서 fault 도 안 뜬다 (슬라이스 4 리뷰에서 실제 발현).

### Unified arm kin&dyn

joint · task · wbc 는 arm FK/Jacobian 을 결합(arm+hand) 모델 위의 `PinocchioCache` 에서 얻는다 (tick 당 `Update(q,v)` 1회). 이 배선은 한때 컨트롤러 셋에 복제돼 있었고(#174) 지금은 공유 타입 `integrated_bringup::CombinedModelCache` 한 곳이다. closed-chain 손에서는 접촉 frame 의 J · oMf · dJv 가 loop-consistent 로 격상되고 dJv 는 `RtClosedChainHandle` drift API 로 정확하다(#173). RT 진입점은 `ReadState → 모델 갱신 → 제어 법칙 → WriteJointCommand` 순서를 지킨다. WBC 는 superset 소비자라 TSID contact frame 과 reduced-dynamics provider 를 로컬로 두고, 모델 선택(`SelectModel`)과 cache Init 을 분리 호출한다 (contact frame id 파싱이 선택된 모델을 요구하기 때문).

### DemoWbcController 의 Kinematic / Dynamic 분리

- **Kinematic WBC** = CLIK-QP(`ClikReferenceGenerator`). 유일한 position backbone 이라 실패는 critical (hold-last → N회 연속 실패 시 `kFallback`). 계약은 `nq==nv` (reduced wbc 모델)이고 어기면 `on_configure` 가 FAIL — integrator fallback 은 없다 (DEC-1).
- **Dynamic WBC** = hand 전용 τ_ff 경로(`hand_tauff_*`). PD backbone 위에 gravity_comp 또는 tsid_tau 를 overlay 한다. closure/hold 에서만, `±hand_tauff_max` clamp, opt-in.
- 두 QP 는 `ComputeWbcCommon` 의 같은 reference 를 독립 소비한다 (decision 5).
- commanded SE3 는 `kRelease`/`kFallback` 을 뺀 모든 phase 에서 live override 이고 `kClosure`/`kHold` 진입 시 직전의 stale SE3 를 clear 한다 (idle 복귀 후 grasp 명령으로 jog 되는 것을 막는다). hand joint target 은 phase 진입 edge 를 기다리지 않고 매 tick posture 로 fold 된다.
- 이상 감지는 **slip(|df/dt|) 만** 켜져 있다. deformation guard 는 강체만 다루는 현재 범위에서 꺼져 있고 키만 남아 있다.

## DemoComplianceController 근거

`demo_task` 사본에서 출발해(#469 S2) 외부 렌치 admittance 가 결합됐다(S3). 사본인 이유와 상속·추출을 기각한 근거는 `demo_compliance_controller.hpp` 배너에 있다.

- **렌치 소스는 hand 레인의 pull 추정치이고 한 tick 뒤에 발행된다** — 추정기가 같은 tick 의 grasp 판정을 필요로 하기 때문이다. 필터 시상수 대비 lag 는 무시할 만하고 어떤 토픽 소스보다 신선하다.
- **리셋은 하강 에지에서** — SAFE_STOP · E-STOP · device-invalid · cold-cache 를 매 tick 리셋하면 bias 평균이 영원히 끝나지 않는다.
- **X_d 재시드는 x̃ 도 함께 붕괴** (D-A15) — 측정 자세가 이미 X_c 를 실현하므로 남기면 두 번 더해져 최대 0.15 m 계단이 된다. 프레임 종류가 바뀐 재시드는 wrench 샘플까지 버린다 (apply point 가 다른 프레임에서 측정된 것이라 vTCP offset 전체가 레버암이 되고 `K_p^a_rot=0` 이라 안 돌아온다). 이 붕괴는 F5 게이트뿐 아니라 제어 프레임 대기·비유한 Jacobian 홀드에서도 돌아야 한다 — 그 홀드는 파이프라인 `Update()` 를 건너뛰어 렌치 age 가 정지하고, 재개 tick 이 묵은 힘을 `fade=1.0` 으로 인가하기 때문이다.
- **SAFE_STOP 은 래치** 이고 탈출구는 `/rtc_cm/reset_fault` 뿐이다. 재활성화는 래치를 풀지 않는다 (compliance-conventions.md §10.6 자동 복구 금지). 홀드 tick 의 진단 행은 FSM 의 실제 상태를 실어야 한다 — 강제로 HOLDING 을 쓰면 래치가 CSV 에서 사라진다.
- **램프 재무장 트리거는 "쓸 수 있는 wrench 가 생긴 rising edge"** (`valid && !stale`, #497) 이다. 이전 트리거(BIAS_CALIBRATING 재진입)는 실측 결함이 둘이었다. 출하 프로필은 셋 다 `bias_calibration_samples: 0` (pull baseline 이 이미 bias 를 빼므로, D-A5) 라 재진입 edge 가 안 떠서 재무장이 출하 config 에서 도달 불가였고, 평균 도중 죽은 소스는 그 edge 를 매 tick 재발화시켜 램프를 0 으로 되돌리고 FSM 을 BIAS_CALIBRATING 에 묶었다. 증상이 fault 가 아니라 침묵이었다 (p1b 실측: α=0 으로 16.8 s, 재진입 8360 tick 연속). 지금은 fresh 샘플에서만 edge 가 뜨고 램프 정지도 `!stale` 로 게이트되며 부분합은 양쪽 다 보존된다.
- **hand 레인 재시드 (#504)** — 활성화가 hand target 을 측정값에서 재시드하므로 `grasp_controller_type: none` 프로필(p1b · iiwa7_leap)에서는 파지력인 `cmd − actual` 간극이 0 이 되어 **파지한 채 전환하면 물체를 놓는다**. 계약을 고치는 대신 **순서**가 답이다 — compliance 를 먼저 활성화하고 그다음에 손을 닫는다 (절차: `.claude/skills/verify/SKILL.md` p1b). 손이 위치 지령이라 겉보기엔 닫힌 채이고 서명은 downstream(pull invalid + `quality_low`)이다.

- **설정면** — `external_wrench.source` 는 필수(현재 합법값 `pull_estimator`)이고 코어 파서와 블록을 **공유**하므로 코어가 안 읽는 잔재로 오해해 지우면 안 된다. 출하 법칙 3키(`stiffness: 0` = D-A3 · `bias_calibration_samples: 0` · `payload_mass: 0`, D-A5)와 §7 응답 키는 코어 default 가 조용히 실기를 재튜닝하지 못하게 명시 등록했다 (하드웨어 튜닝이 아니다). 검증은 `test_compliance_task_equivalence.cpp` 등이 고정한다.

## DemoInferenceController 근거

- **모델 교체에 재컴파일이 없도록** 입력 텐서의 이름·순서·폭·정규화·경로를 전부 YAML 로 뺐다. 스키마 SSoT 는 `rtc_controllers/include/rtc_controllers/params/policy_io_params.hpp`.
- **pose 기준 frame** — 모든 pose 는 `inference.policy_frame` 기준이다. ur5e_p1b 에서 정책 root 는 `base` 이고 sim world 가 곧 그 frame 이다. `base_link` 는 z 로 반바퀴 떨어져 있어, 그걸로 주면 pose 가 멀쩡해 보인 채 정책이 물체를 **반추종**한다. 물체 레인은 `object_pose.source_frame_link` 로 메시지 frame 이 URDF 의 어느 frame 인지 선언한다.
- **`decimation`** — 정책을 N tick 마다 돌리고 사이는 action hold. sim 이 lock-step 이라 `control_rate` 를 낮추는 대안보다 낫다 (후자는 `ControllerState::dt` 를 실제 sim-time 과 어긋나게 만든다).
- **손끝 힘 feature** — 손의 `sensor_layout.inference_values_per_group` 이 힘 3축을 담을 만큼 넓어야 하고 아니면 configure 거부다. 좁으면 전 그룹이 조용히 0 이 된다.
- **활성화 경계는 나이가 아니라 새 샘플이 기준** — 물체 lane 의 나이는 tick `dt` 로만 누적돼 비활성 구간이 안 실리므로, 재활성 후에는 새 TF 가 올 때까지 `kObject` hold 이고 tactile hold 래치도 리셋된다. 명령 tail 의 rate bound 는 직전 명령 기준이다 (측정 q 기준이면 서보 지연이 명령을 묶는다).
- **실패는 전 device 를 래치된 자세로 hold** (부분 적용 없음). 사유는 tick 마다 `inference_diag.csv` 에 코드로 남고, 폐쇄 체인 사영이 `closed_chain_warn_ticks` 이상 held 면 비-RT 타이머가 활성화당 1회 경고한다.

## GraspController 근거

### 모드와 capability 의 두 축 (B-3, B-4b)

`BuildGraspController` 는 `force_pi_grasp` 블록 유무만 보고 모드는 안 본다. 컨트롤러 스위칭이 re-configure 를 하지 않으므로 빌드가 모드를 봤다면 모드가 one-way door 가 된다. 따라서 `grasp_controller_ != nullptr` 은 "force_pi 가 돈다"의 대용이 아니다 — 제어 법칙 · diagnostics fill · `grasp_command` 가 각자 모드를 직접 확인한다. `ur5e_p1b` 가 `type: none` + 블록 조합을 실제로 싣는다.

### 런타임 전환은 quiet 일 때만

- 대안이던 "항상 허용 + 자동 safing" 은 RT tick 코드를 더 건드리고 재-seed 누락 자체가 hazard 가 되어 기각했다 (2026-07-30).
- quiet 는 부수 이득이 있다 — force_pi 를 떠나는 것이 `kIdle` 에서만 가능하므로 발행된 GraspState 의 PI diagnostics 는 idle-consistent 이고, 전환 시 그 필드를 청소할 필요가 없다.
- **gate 는 RT 를 멈출 수 없다** — 콜백은 RT tick 이 store 하는 atomic 미러를 읽어 판정이 한 tick 뒤처진다. 그 창은 compute 쪽 race closure 가 닫는다: 모드가 contact_stop 을 떠난 tick 에 latch 가 걸려 있으면 latch 를 떨구기 전에 hand goal 을 `hand_hold_position_` 으로 재-seed 하고 WARN 을 찍는다 (gate 가 정상이면 발화 안 하므로 발화 자체가 결함 신호). 없으면 전환 첫 tick 에 명령이 접촉 전 goal 로 튄다 — 실측 0.3 → 0.8 rad.
- **non-force_pi tick 은 FSM 을 Reset** 한다 (모드가 force_pi 가 아닌 매 tick + `on_activate`). 조건부였을 때 깨졌다: GRASP 수락 → RT 가 FSM 을 kApproaching 으로 step → 전환 Store 가 착지하면 아무도 FSM 을 step 하지 않아 phase 미러가 non-Idle 로 얼고, quiet gate 가 이후 모든 전환을 거부하고 RELEASE 도 거부돼 복구 불가였다. 미러도 분기 밖에서 매 tick store 한다. `grasp_command` 가 Inactive 에서 거부되는 것도 같은 이유 — 서비스가 deactivate 를 넘어 살아 있어 요청만 arm 되고 다음 활성화 첫 tick 에 발화했다.

### 힘 피드백 필터

- `GraspController` 는 필터를 갖지 않는다. fingertip 원시 triplet 이 delta-spike 가드를 지난 뒤 **축별** Bessel LPF 를 거치고 magnitude 는 그다음에 취한다. 순서가 뒤집히면(`LPF(‖F‖)`) 정류로 zero-mean 축 노이즈가 양의 DC(3축 가우시안이면 ≈1.6σ)로 남아 `f_contact_threshold` 밑에 영구히 깔린다. 필터가 상류에 있는 것은 취향이 아니라 강제다 — `‖LPF(F)‖` 는 3축을 봐야 하는데 `Update()` 는 이미 접힌 값만 받는다.
- **가드는 공유, cutoff 는 독립** — contact_stop 은 `fsm.contact_stop_force_lpf_cutoff_hz`(기본 50 Hz), force_pi 는 `force_pi_grasp.lpf_cutoff_hz`(기본 25 Hz). contact_stop 은 지연이 곧 손 이동 거리인 래치이고 force_pi 는 tick 당 미분(500 Hz 에서 0.01 N)으로 slip 을 보는 서보라 대역 요구가 반대다. 가드가 없을 때 실측: 2-tick 배선 스파이크(6.406/6.058 N)가 서보 피드를 **1.740 N** 으로 밀어 임계 0.8 N 을 2.2배 넘기고, 하강 df/dt ≈ −43 N/s 가 `df_slip_threshold` −5 N/s 를 넘겨 grip tightening 까지 불렀다. 발행 `GraspState.force_magnitude`/`max_force` 는 raw 그대로(BT 계약 불변), `finger_filtered_force` 가 서보가 본 값이다.

### FSM 규칙의 이유

- **Approaching 은 스스로 실패하지 않는다** — 판정 손가락(thumb + 접촉한 아무 한 손가락, #432)이 `s=1.0` 까지 닫혀도 접촉이 없으면 kIdle 로 낙하하지 않고 닫힌 자세에서 판정을 계속한다. 접촉은 시간 제한이 아니라 사건이고 힘은 늦게 들어올 수 있다(물체가 미끄러져 들어옴 · 팔이 이동 중 · fingertip lane 이 늦게 살아남). 낙하하던 이전 spec 은 호출측의 `phase() != kIdle` 게이트를 통해 손 명령을 한 tick 만에 trajectory 로 되돌렸다. 늦은 접촉이 잡히면 `s_at_contact = 1.0` 이고 `s` 상한이 deformation guard 보다 먼저 막아 이어지는 force control 은 여는 방향으로만 권한을 갖는다.
- **kApproaching 도 RELEASE 를 소비한다** (phase 함수 맨 앞, 경쟁 전이의 early return 이 플래그를 건너뛰지 않도록). 접촉 없는 grasp 를 빠져나오는 유일한 길이다. 없으면 FSM 이 영구히 kApproaching 에 남아 quiet gate 가 모드 전환을 영구 거부하고 BT `ForcePIRelease` 가 타임아웃까지 매달린다.
- **ForceControl → Holding 승급은 램프 완료를 요구한다** — dwell(`settle_time`)이 `f_desired` 가 `f_target` 에 닿은 뒤에만 돈다. 그 조건이 없던 시절 `settle_time < 2·settle_epsilon/f_ramp_rate` 인 설정이 램프 도중에 승급했고 Holding 에는 램프가 없어 기준값이 거기서 얼었다. 배포 `ur5e_p1a` 가 그 영역이었고 실측 진입 `f_desired` 는 강성과 무관하게 **0.60~0.68 N (목표 2.0 N)** 이었다. 대가는 정직해진 소요 시간이다: Holding 진입 K=200 에서 2.2 s, K=20 에서 5.5 s, K=10 에서 9.3 s. 예산 내 최대 힘(`≈ f_contact_threshold + K·delta_s_max`)이 모자라는 무른 물체(K≲7)는 거짓 Holding 대신 kForceControl 에 머물고 BT `grasp_timeout_ms` 가 실패로 처리한다.
- **grip tightening 은 rate** 다 (`grip_tightening_rate` [N/s], `grip_decay_rate` 와 같은 축). 슬립 판정의 `f_measured < f_target · f_slip_fraction` 은 사건이 아니라 상태라 참인 동안 매 tick 재발화해, per-tick 비율이던 시절 grip force 가 `control_rate` 의 함수였다 (0.15/tick → 500 Hz 에서 22 ms 만에 `f_max_multiplier` 상한). 이 분기는 integrator 동결을 건드리지 않는다 — 동결은 `ApplyDeformationGuard` 의 닫힌 계약이고, 여기서 풀면 guard 와 매 tick 교착한다 (실측: 연체에서 기준값 4.0 N 고정 · `Δs` 는 `delta_s_max` 포화 · 힘 0.85 N 정지).

## 토픽 계약 근거

- **Controller YAML 토픽은 `kRobotTransforms` 하나** — publisher 가 없던 `kRobotTarget`/`kDigitalTwinState` 는 선언하면 조용히 죽은 토픽이 돼 제거했고(#196), `kGuiPosition` 은 `/rtc_cm/<group>/joint_states` + `<config_key>/transforms` 로 대체됐다. 모든 controller YAML entry 는 controller-owned 이고 `ownership:` 필드는 없다(#138).
- **매 tick Store (PROC-7, #234 P-1)** — CM 이 tick 마다 새 `stamp_ns` 를 push 하고 publish thread 가 SeqLock 을 다시 Load 하므로, 건너뛴 Store 는 "발행 안 함"이 아니라 "직전 body 를 현재 stamp 로 재발행"이다. E-STOP 처럼 법칙을 건너뛰는 tick 의 경로가 각 컨트롤러의 `FillEstopPublishState()` 다 (센서 유래 집계는 갱신, Force-PI per-finger 진단은 0, TSID health 는 not-solved, pull 추정은 `StageEstopPullTick` 으로 `valid=false` + 감쇠). `pull_estimator.csv` 도 gap 이 아니라 `valid=0` 행을 남긴다.
- **`PullEstimate` 자기기술 (#234 P-5/P-7/P-11)** — `force_inplane[2]` 는 매 tick 관측된 pinch 축에서 재구성되는 기저의 좌표라 tick 간 비교도 역변환도 불가능하다. 그래서 같은 tick 의 `plane_normal[3]` + `basis_x[3]` (e_y = n × e_x 로 B 가 결정됨)과 `basis_source` (`BASIS_REFERENCE` 일 때만 `force_inplane[0]` 이 reference 방향 성분)를 싣는다. 기준 frame 은 `header.frame_id` — 컨트롤러가 `SetOwnedStateFrameId()` 로 arm root link 를 박는다. `invalid_reason` 은 실패한 게이트를 지목해 WBC 트레이스를 joint · task 와 비교 가능하게 한다. `valid` 는 pull 벡터 유효성 전용이다.
- **`PayloadEstimate` 자기기술 (#135 D12)** — 잔차 `r` 과 역산한 payload wrench/질량을 한 메시지에 싣는다 (토픽을 나누면 시간 정렬이 소비자 몫). `joint_names` 는 `residual` 이 device order 인데 모델 Jacobian 컬럼은 pinocchio order 라, 짝지으면 유한하고 매끄럽고 **틀린** 답이 나오기 때문이다. 프레임이 둘인 이유는 wrench 가 screw 라 작용점이 모멘트를 바꾸기 때문이다. staleness 는 `tick` 으로 판정한다 — `header.stamp` 는 publish thread 의 wall clock 이라 RT 가 멈춰도 증가한다. 발행 조건은 관측기 활성이고 추정기가 꺼져 있어도(출하 기본) 잔차 절반은 나가며 payload 블록은 `PAYLOAD_NOT_INITIALIZED` 다. **Layer 2B (#455)** 는 관성 4 파라미터(`I` 는 없다 — 준정적 게이트 아래 회귀자의 관성 6열이 항등적으로 0)를 더한다. 한 자세는 `m·c` 의 중력 방향 성분을 못 보므로 `inertial_rank` 는 자세를 바꾸기 전까지 3, `inertial_valid` 는 false 다 — 기동 시 정상이다.

## DemoJoint · DemoTask · DemoDualArm · DemoCatching 메모

- tree 모델(`urdf.tree_models`: waist + 양팔)을 구동하는 컨트롤러는 joint 와 dualarm 뿐이다. task · wbc · compliance 는 팔을 사슬 하나로 전제하므로 `g1_p1b` 에서는 YAML 이 없어 건너뛴다.
- DemoTask 의 arm 명령은 `joint_command_tail` 로 바운드된다. §7 스키마를 읽지 않으므로 δ=0 이고 밴드는 device 한계 그대로다 (#478).
- DemoCatching 은 계획기 스레드(`planner.enabled`, `mpc` layout role 을 MPC 와 공유, E-7 J) 또는 `diagnostic.oracle_plan` 이 plan box 에 쓰고 RT 가 `JudgePlan` 으로 받는다. 소비 키가 provisional/TBD 이거나 둘이 함께 켜지면 configure 는 성공하고 activate 만 거부한다 (park).
- DemoDualArm 은 시드 시 몸통 관절이 풀이의 위치 box 밖(`clik.limit_margin` 띠)이면 풀이를 시작하지 않고 `/rtc_cm/reset_fault` 로만 풀리는 latch 를 건다. 풀이 실패는 직전 명령 유지, 연속 실패는 같은 latch.
