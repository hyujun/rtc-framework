# Controllers — 근거·실측·이력

> **이 문서는 헌법이 아니다.** 규범은 [agent_docs/controllers.md](../../agent_docs/controllers.md) 가 갖고, 사용법은 [docs/controllers.md](../controllers.md) 가 갖는다. 여기는 컨트롤러 구조가 *왜* 지금 모양인지, 무엇을 실측했는지를 기록한다. 둘이 어긋나면 agent_docs 가 옳다. 수치·이름은 **기록 시점의 것**이라 인용 전에 다시 잰다.

## 알고리즘 코어 추출 이력 (#236)

어댑터는 어디에도 등록되지 않아 런타임 노출이 0 이었고 cross-check oracle 로만 살았다. 그래서 법칙을 무상태 free function 으로 뽑은 뒤(S1~S6) 어댑터를 삭제했다(S7c).

- **D-Q1 P 컨트롤러 은퇴** — 법칙이 `q + kp·e·dt` + 클램프라 파일 하나 값어치가 없다. 관절 PD 를 `kd = 0` 으로 쓰면 같다.
- **D-Q2 Λ/τ/Nᵀ 흡수** — 다섯 컨트롤러가 같은 `TaskDynamics` 를 쓰게 해 λ 규약을 compliance-conventions.md §6.5 로 통일했다. 그래서 비-bit-identical 이관(D-S2b (b)안, 허용 오차는 `differential_ik.hpp` 헤더)을 받아들였다.
- **게인 단위가 법칙마다 다르다** — CLIK 은 속도형 `[1/s]`, OSC 는 가속도형 `[1/s²]`, impedance 는 힘형(Λ 배 차이). 합치지 않는다. S3 의 6축·병진전용 두 함수도 인라인 식이 달라 합치지 않았다.
- D19/D20 은 compliance-conventions.md §7.6 · `bandwidth_separation.hpp`, impedance/admittance 의 반대 부호는 compliance-conventions.md §11.4.1 이 갖는다.

## 바인딩 구조 근거

### 팔 끝 frame 과 재정렬 함정

- `urdf.tip_link` 가 풀리지 않으면 joint · task · compliance · wbc 는 `on_configure` 를 거부한다. 그대로 돌면 팔 끝이 아닌 pose(정상 tick 은 항등, E-STOP tick 은 universe frame)를 팔 끝으로 유효 표시해 내고, wbc 는 Cartesian hold 의 seed 를 universe frame 에서 읽는다. 이름이 모델 frame 이기만 하면 통과하므로 팔 끝이 아닌 link 는 못 잡는다.
- 재정렬 규칙은 agent_docs/controllers.md §Joint order & submodel selection 이다. `ν = J·q̇` 의 `q̇` 를 빠뜨리면 감쇠 토크가 틀리면서 fault 도 안 뜬다 (슬라이스 4 리뷰에서 실제 발현).

### Unified arm kin&dyn

이 배선은 한때 컨트롤러 셋에 복제돼 있었고(#174) 지금은 공유 타입 `integrated_bringup::CombinedModelCache` 한 곳이다. closed-chain 손에서는 접촉 frame 의 J · oMf · dJv 가 loop-consistent 로 격상되고 dJv 는 `RtClosedChainHandle` drift API 로 정확하다(#173). WBC 는 superset 소비자라 TSID contact frame 과 reduced-dynamics provider 를 로컬로 두고, 모델 선택(`SelectModel`)과 cache Init 을 분리 호출한다 (contact frame id 파싱이 선택된 모델을 요구하기 때문).

### DemoWbcController 의 Kinematic / Dynamic 분리

- 두 QP 는 `ComputeWbcCommon` 의 같은 reference 를 독립 소비한다 (decision 5).
- 이상 감지는 **slip(|df/dt|) 만** 켜져 있다. deformation guard 는 강체만 다루는 현재 범위에서 꺼져 있고 키만 남아 있다.

## DemoComplianceController 근거

`demo_task` 사본에서 출발해(#469 S2) 외부 렌치 admittance 가 결합됐다(S3). 사본인 이유와 상속·추출을 기각한 근거는 `demo_compliance_controller.hpp` 배너에 있다 (배너 첫 줄의 "AT THIS COMMIT … lands in #469 S3" 는 S3 가 들어온 뒤로 낡았다).

- **렌치 소스는 hand 레인의 pull 추정치이고 한 tick 뒤에 발행된다** — 추정기가 같은 tick 의 grasp 판정을 필요로 하기 때문이다. 필터 시상수 대비 lag 는 무시할 만하고 어떤 토픽 소스보다 신선하다.
- **X_d 재시드는 x̃ 도 함께 붕괴** (D-A15) — 측정 자세가 이미 X_c 를 실현하므로 남기면 두 번 더해져 최대 0.15 m 계단이 된다. 프레임 종류가 바뀐 재시드가 wrench 샘플까지 버리는 이유는 apply point 가 다른 프레임에서 측정된 것이라 vTCP offset 전체가 레버암이 되고 `K_p^a_rot=0` 이라 안 돌아오기 때문이다.
- **램프 재무장 트리거는 "쓸 수 있는 wrench 가 생긴 rising edge"** (`valid && !stale`, #497) 이다. 이전 트리거(BIAS_CALIBRATING 재진입)는 실측 결함이 둘이었다. 출하 프로필은 셋 다 `bias_calibration_samples: 0` (pull baseline 이 이미 bias 를 빼므로, D-A5) 라 재진입 edge 가 안 떠서 재무장이 출하 config 에서 도달 불가였고, 평균 도중 죽은 소스는 그 edge 를 매 tick 재발화시켜 램프를 0 으로 되돌리고 FSM 을 BIAS_CALIBRATING 에 묶었다. 증상이 fault 가 아니라 침묵이었다 (p1b 실측: α=0 으로 16.8 s, 재진입 8360 tick 연속).
- hand 레인 재시드(#504)·설정면의 계약은 agent_docs/controllers.md §컨트롤러별 계약 이다.

## DemoInferenceController 근거

계약은 agent_docs/controllers.md §컨트롤러별 계약 이다. 아래는 계약에 없는 이유만 남긴다.

- **pose 기준 frame** — ur5e_p1b 에서 정책 root 는 `base` 이고 sim world 가 곧 그 frame 이다. `base_link` 는 z 로 반바퀴 떨어져 있어, 그걸로 주면 pose 가 멀쩡해 보인 채 정책이 물체를 **반추종**한다.
- **`decimation`** — sim 이 lock-step 이라 `control_rate` 를 낮추는 대안보다 낫다 (후자는 `ControllerState::dt` 를 실제 sim-time 과 어긋나게 만든다).
- **활성화 경계** — 물체 lane 의 나이는 tick `dt` 로만 누적돼 비활성 구간이 안 실리므로 나이를 기준으로 삼을 수 없다.

## GraspController 근거

### 모드와 capability 의 두 축 (B-3, B-4b)

컨트롤러 스위칭이 re-configure 를 하지 않으므로 빌드(`BuildGraspController`)가 모드를 봤다면 모드가 one-way door 가 된다. 계약은 agent_docs/controllers.md §GraspController 다.

### 런타임 전환은 quiet 일 때만

- 대안이던 "항상 허용 + 자동 safing" 은 RT tick 코드를 더 건드리고 재-seed 누락 자체가 hazard 가 되어 기각했다 (2026-07-30).
- **gate 는 RT 를 멈출 수 없다** — 콜백은 RT tick 이 store 하는 atomic 미러를 읽어 판정이 한 tick 뒤처진다. race closure 가 없으면 전환 첫 tick 에 명령이 접촉 전 goal 로 튄다 — 실측 0.3 → 0.8 rad.
- **non-force_pi tick 은 FSM 을 Reset** 한다. 조건부였을 때 깨졌다: GRASP 수락 → RT 가 FSM 을 kApproaching 으로 step → 전환 Store 가 착지하면 아무도 FSM 을 step 하지 않아 phase 미러가 non-Idle 로 얼고, quiet gate 가 이후 모든 전환을 거부하고 RELEASE 도 거부돼 복구 불가였다. `grasp_command` 를 Inactive 에서 거부하는 것도, 안 하면 요청만 arm 돼 다음 활성화 첫 tick 에 발화했기 때문이다.

### 힘 피드백 필터

- 필터가 상류에 있어야 하는 이유(`LPF(‖F‖)` 의 정류 DC)는 `grasp_controller.hpp` `Update()` 주석이 갖는다.
- **가드는 공유, cutoff 는 독립** — contact_stop 은 `fsm.contact_stop_force_lpf_cutoff_hz`(기본 50 Hz), force_pi 는 `force_pi_grasp.lpf_cutoff_hz`(기본 25 Hz). contact_stop 은 지연이 곧 손 이동 거리인 래치이고 force_pi 는 tick 당 미분으로 slip 을 보는 서보라 대역 요구가 반대다. 가드가 없을 때 실측: 2-tick 배선 스파이크(6.406/6.058 N)가 서보 피드를 **1.740 N** 으로 밀어 임계 0.8 N 을 2.2배 넘기고, 하강 df/dt ≈ −43 N/s 가 `df_slip_threshold` −5 N/s 를 넘겨 grip tightening 까지 불렀다.

### FSM 규칙의 이유

- **Approaching 은 스스로 실패하지 않는다** (#432) — 접촉은 시간 제한이 아니라 사건이고 힘은 늦게 들어올 수 있다(물체가 미끄러져 들어옴 · 팔이 이동 중 · fingertip lane 이 늦게 살아남). 낙하하던 이전 spec 은 호출측의 `phase() != kIdle` 게이트를 통해 손 명령을 한 tick 만에 trajectory 로 되돌렸다. kApproaching 이 RELEASE 를 소비하지 않으면 FSM 이 영구히 거기 남아 quiet gate 가 모드 전환을 영구 거부하고 BT `ForcePIRelease` 가 타임아웃까지 매달린다.
- **ForceControl → Holding 승급은 램프 완료를 요구한다.** 그 조건이 없던 시절 `settle_time < 2·settle_epsilon/f_ramp_rate` 인 설정이 램프 도중에 승급했고 Holding 에는 램프가 없어 기준값이 거기서 얼었다. 배포 `ur5e_p1a` 가 그 영역이었고 실측 진입 `f_desired` 는 강성과 무관하게 **0.60~0.68 N (목표 2.0 N)** 이었다. 대가는 정직해진 소요 시간이다: Holding 진입 K=200 에서 2.2 s, K=20 에서 5.5 s, K=10 에서 9.3 s. 예산 내 최대 힘(`≈ f_contact_threshold + K·delta_s_max`)이 모자라는 무른 물체(K≲7)는 거짓 Holding 대신 kForceControl 에 머물고 BT `grasp_timeout_ms` 가 실패로 처리한다.
- **grip tightening 은 rate** 다. 슬립 판정의 `f_measured < f_target · f_slip_fraction` 은 사건이 아니라 상태라 참인 동안 매 tick 재발화해, per-tick 비율이던 시절 grip force 가 `control_rate` 의 함수였다 (0.15/tick → 500 Hz 에서 22 ms 만에 `f_max_multiplier` 상한). 이 분기에서 integrator 동결을 풀면 `ApplyDeformationGuard` 와 매 tick 교착한다 (실측: 연체에서 기준값 4.0 N 고정 · `Δs` 는 `delta_s_max` 포화 · 힘 0.85 N 정지).

## 토픽 계약 근거

- **Controller YAML 토픽은 `kRobotTransforms` 하나** — publisher 가 없던 `kRobotTarget`/`kDigitalTwinState` 는 조용히 죽은 토픽이 돼 제거했고(#196), `kGuiPosition` 은 `<config_key>/transforms` 등으로 대체됐다. `ownership:` 필드는 없다(#138).
- 매 tick Store 는 invariants.md PROC-7 (#234 P-1), `PullEstimate`(#234 P-5/P-7/P-11)·`PayloadEstimate`(#135 D12, Layer 2B #455) 의 자기기술 근거는 `rtc_msgs/msg/PullEstimate.msg` · `PayloadEstimate.msg` 주석이 갖는다.
