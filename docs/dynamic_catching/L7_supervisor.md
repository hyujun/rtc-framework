# L7 — Supervisor: 상태 머신, 접촉 판정, 감속, abort

- 문서 버전: v0.5 (2026-09-19) — 결정·단계의 SSoT 는 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) (충돌 시 plan 우선)
- 브랜치: 단계별 `type/kebab-slug` (main 기준, 마스터 §4.2)
- 배치 `[확정 D-1]`: 순수 조각(감속 목표, 전이표 데이터, 접촉 debounce)은 rtc_controllers 의 `catching` 하위 디렉토리 (namespace `rtc::catching`), FSM 은 포구 컨트롤러(`RTControllerInterface::Compute` 안)에서 구동, YAML 은 `integrated_bringup` 바인딩. 별도 패키지를 만들지 않는다
- 단계: **S1.8** 순수 조각, **S5.1** E-STOP·fault 최소 계약(P-1, `[CONCERN] E-8` 승인 후), **S7** 손 시퀀서·FSM·접촉 판정·감속·충격량 예산·재무장. E-STOP·fault 정책 전체는 **S9 (D-13 보류)** — 그 전까지는 §4.1 의 임시 기준(P-1)만 구현
- 선행: 단계 W, L1–L6
- 산출물: 전이표(데이터), `Mode`/`Reason`/`Outcome` enum, 감속 목표, 접촉 판정기, 전이 로그 레코드

---

## 1. 범위 / 비범위

범위:
- RT 루프(`RTControllerInterface::Compute`)에서 포구 임무의 모드를 결정한다.
- plan 수락·동결, 손 시퀀서 구동, 포구 후 감속, 결과 판정(포획·실패), 안전 abort와 복귀를 맡는다.

비범위:
- 계획 계산(L3)
- 기준 생성 수식(L4)
- 관절 한계 처리(L5)
- E-STOP 시 출력 대체 — CM 이 `ValidateControllerOutput` 실패·E-STOP 시 `BuildHoldOutput` 으로 대체한다. 슈퍼바이저는 상태 정리만 한다 (§4.1)
- E-STOP·fault 정책 전체 (S9, D-13)
- UR 드라이버 자체의 보호 정지(드라이버·로봇 제어기 소관)

## 2. 코드 확인 게이트

| ID | 확인 항목 | 기록 |
|---|---|---|
| G7-1 | RTC 프레임워크의 기존 상태 머신·모드 전환 규약, 컨트롤러 활성/비활성 시 명령 유지 방식 | 닫힘 — FSM 선례는 compliance 코어의 `ComplianceState`. 명령 유지는 CM 의 `BuildHoldOutput`, 컨트롤러 fault 는 `ResetFault`/`HasLatchedFault` 래치 (E-STOP 과 분리, `/rtc_cm/reset_fault`) (W) |
| G7-2 | UR 드라이버의 speed scaling 상태 인터페이스 이름과 의미 ([R11]) | 닫힘 — repo 에 speed scaling 노출 없음 (W). 신호 출처 확보는 S10. TBD-ARM-03 은 S10 으로 이월 |
| G7-3 | 지문 센서 잡음 수준, 주기, 스탬프 (시뮬레이션·실기) | 부분 닫힘 — 실기 P1b `HandSensorState` 250 Hz, sim `rtc_mujoco_sim` 발행 모두 **finger-on-object 부호** (커밋 0fcc1d23 이후 코드 확인, §4.4). 잡음 수준은 TBD-HAND-03 |
| G7-4 | PTP 동기 상태 확인 방법 | 닫힘 — repo 에 시계 건강 신호 없음 (W). S10. TBD-NET-01 은 S10 으로 이월 |

## 3. 참고자료

[R1] 포구 직후 충격과 순응의 역할, [R3] 인터셉트 후 선형 감속(원 논문의 임시방편, 본 layer에서 정식화 — 원문 확인: "the velocity of the robot is linearly reduced during the post-interception period (0.3 s)"), [R8] 임계값 설계, [R16] 충격량 분배·강성 최적화, [R18] 충격 순간 기준 궤적 처리(reference spreading), [R19] 접선 컴플라이언스.

## 4. 수학적 이론

### 4.1 상태 머신

시각 표기는 plan §3 (D-2) 을 따른다: $t_c$, $t_{cmd}$ 는 공의 **물리 시각**(`BallTime`, 절대 steady ns), $now$ 는 매 tick steady 실측(`NowReal`), $now_{lead}=now+T_{arm}$ (`NowLead`). 비교는 타입별 오버로드로만 한다.

| 상태 | 진입 조건 | 주요 동작 | 이탈 |
|---|---|---|---|
| `IDLE` | 활성화 직후, E-STOP 해제 후, fault 리셋 후 | 파라미터 검증 전: 현재 자세 유지(비무장, latch). 검증 통과 + **무장** 후: **homing** — 관절공간 법칙(`joint_home.hpp`, QP/CLIK 비의존)으로 `wait_pose` 로 이동(팔이 이미 `pose_tol` 안이면 생략, Q13), 손 `q_open`(homing 중에만 — Q4, §4.5) | `wait_pose` 허용오차 안 + 손 `q_pre` 도달 + §4.5 조건 → `ARMED` |
| `ARMED` | 준비 완료 | 대기 자세 유지(관절공간, homing 과 같은 법칙), 손 `q_pre` | 유효 궤적 수신(형식 검사 통과 + not stale + 지평 여유) → `TRACKING` / §4.5 조건 위반 → `IDLE` |
| `TRACKING` | 유효 궤적 수신 | 기준 유지, plan 대기, 손 `q_pre` | 유효 plan → `APPROACH` / 트랙 epoch·`generation` 변경·stale → `ARMED` / plan 없음(`NO_CATCHABLE_PLAN`) → 머무름, 기록 |
| `APPROACH` | 유효 plan | L4 추종, plan 교체 허용, 손 `q_pre` | $t_c-now\le T_{freeze}$ → `COMMITTED` (R-ADMIT, §4.1) / 실패 조건 → `RETREAT` |
| `COMMITTED` | 동결 | plan 교체 금지, 손은 계속 `q_pre` (시각 기반 preshape·`T_pre` 는 쓰지 않는다 — **Q4 확정, 2026-09-23 사용자**: `q_open` 은 homing 전용이고 대기 중 손은 항상 `q_pre` 다) | $now\ge t_{cmd}$ (R-CLOSE, `HandCommandDueRounded`) → `CLOSING` / 치명 조건 → `ABORT_SAFE` |
| `CLOSING` | 폐쇄 명령 | 손 `Close` | $now_{lead}\ge t_c$ → `DECEL` / 치명 조건 → `ABORT_SAFE` |
| `DECEL` | $now_{lead}\ge t_c$ | 가상 감속 대상 추종(§4.3), 접촉 판정 | 감속 대상 정지($\tau\ge\tau_s$) → `HOLD` / 치명 조건 → `ABORT_SAFE` |
| `HOLD` | 정지 | 유지, 결과 판정 확정 | $T_{hold}$ 경과 → `RETREAT` |
| `RETREAT` | 종료·실패 | 정지 램프(`JointSpaceDecelStep`, ABORT_SAFE 경유 시 no-op) + 관절공간 `wait_pose` 복귀. 손은 결과에 따라 순서가 갈린다(§4.8 "RETREAT 순서") | `wait_pose` 도착(이미 안이면 즉시) + 손 `q_pre` 도달 → `ResetForRearm` → `ARMED` |
| `ABORT_SAFE` | 치명 조건(상태 무관) | 즉시 감속 후 정지. L5 가 정상이면 §4.3 감속 대상을 L4→L5 로, **`QP_FAILED`·`JOINT_CONFLICT` 이면 QP 비의존 관절공간 감속**(아래) | 정지 → `RETREAT` / 재차 치명 조건 → `FAULT` |
| `FAULT` | `QP_FAILED` 연속 $N_{qp}$회, 또는 `ABORT_SAFE` 중 재차 치명 조건 | QP 비의존 관절공간 감속으로 정지 후 $q_c$ 고정, 컨트롤러 fault 래치 (`HasLatchedFault()` true). **RT 에서 deactivate 를 요청하지 않는다** | `/rtc_cm/reset_fault` → `ResetFault()` → `IDLE` (P-1 reseed) |

**전이는 (상태 × 사유) 표를 데이터로 둔다 `[확정 S1.8]`.** 위 표와 §4.2 표는 사람이 읽는 형태이고, 코드는 둘을 합친 표 하나를 단일 출처로 삼는다. 기동 시 완전성을 검사한다 — 모든 상태에 진입·이탈이 최소 1개씩 있고, 모든 `Reason` 이 최소 한 칸에서 쓰이며, 미정의 칸이 없어야 한다(G7-A).

**IDLE homing `[확정 S1.8, 설계 확정 S7.2]`.** v0.4 는 `IDLE` 에서 현재 자세만 유지하면서 `ARMED` 진입 조건에 "대기 자세 허용오차 안"(§4.5-4)을 걸어, 활성화 자세가 `wait_pose` 밖이면 영원히 `ARMED` 에 못 가는 교착이 있었다. 파라미터 검증 통과 후 `wait_pose` 로 이동하는 homing 단계를 둔다. homing 은 `IDLE` 의 하위 단계다(`Reason::kNone` 이 정상 전진, homing 은 `kIdle` 안 — S1.8 전이표 헤더 해석).

**homing 은 관절공간이다 `[설계 확정 S7.2, C-13]`.** per-joint 사다리꼴 (v ≤ `supervisor.homing.v_max`, a ≤ `qdd_max`·`supervisor.homing.eta_a`), QP/CLIK 비의존 — `retreat_reference.hpp` 는 코드에 없다(v0.4 가 가정한 task-space soft-catch DS 기반 복귀는 채택하지 않는다, L4 §5.3). 도달 판정은 `supervisor.ready.pose_tol` ∧ ‖q̇‖∞ ≤ `supervisor.homing.qd_tol`. **homing 은 운동이므로 무장 latch 를 요구한다** (P-1 (e): 활성화는 무장이 아니다) — 비무장 `IDLE` 은 활성화 자세를 그대로 유지한다. **팔이 이미 `pose_tol` 안이면 homing 을 생략**하고 손만 `q_pre` 로 지시한다 (Q13, 2026-09-23 사용자 확정).

**R-IDLE.** `IDLE` tick 에서 carried `arm_qd_cmd_ ≠ 0` (RETREAT 복귀 중 disarm → IDLE, 또는 homing 중 disarm) 이면 `JointSpaceDecelStep` 으로 정지까지 램프한 뒤 유지한다 — `{RETREAT, kParamsTbd} → IDLE` 행은 그대로 두고, 헤더 근거를 "RETREAT 는 서 있다" 에서 "`IDLE` 이 이 램프를 소유한다" 로 정정한다 (§4.5 아래 문단과 같은 근거).

`ABORT_SAFE` 의 진입 조건은 "상태 무관"이다. `ARMED` 에서 준비 조건이 깨지면 `IDLE` 로 내려가 조건이 회복되기를 기다린다.

**구현 (S5.3, 2026-09-22).** `ABORT_SAFE` 는 감속이 **끝날 때까지** 머문다 (관절 속도가 전부 0). 시작한 tick 에 나가면 ABORT_SAFE 는 상태가 아니라 이름표가 되고, 다음 시행이 팔이 아직 움직이는 중에 시작한다. 래치된 fault 가 있으면 대신 `ABORT_ESCALATED` 로 FAULT 에 간다 — RETREAT 는 방금 실패한 것을 다시 시도하는 쪽으로 되돌리기 때문이다. `RETREAT` 진입은 plan 무효화 지점이다 (§4.8). E-STOP 리셋은 **모드를 건드리지 않는다**: 전이표가 `{FAULT, ESTOP} → FAULT` 를 갖고 있으므로 reset 이 IDLE 을 강제하면 P-1 (d) 가 지키려는 래치를 지운다.

**QP 비의존 관절공간 감속 `[확정 S5.3]`.** `ABORT_SAFE` 의 기본 경로(§4.3 감속 대상 → L4 → L5)는 L5 QP 가 정상일 때만 성립한다. 원인이 `QP_FAILED`·`JOINT_CONFLICT` 이면 실패한 L5 에 의존할 수 없으므로, 직전 명령 $q_c$·$\dot q_c$ 에서 관절별 가속 한계(D-16 도출값) 이내로 $\dot q_c\to0$ 까지 감속하는 관절공간 경로를 쓴다 (할당·QP 없음, `dt` 기준 적분, 관절 위치 한계 clamp). 정확한 식은 S5.3 에서 확정한다.

**명시적 면제 (C-35).** 구현(controller.cpp)은 원인과 무관하게 `ABORT_SAFE` 를 **항상** 이 관절공간 정지 경로로 처리한다 — L5 가 정상인 원인(예: `TRACK_ERR`)에서도 §4.3 의 task-space 감속 대상 경로는 쓰이지 않는다. task-space 정지의 이득이 불명확하고 QP 의존을 늘리므로 이 현행 동작을 그대로 유지하고 (S7 에서도 바꾸지 않았다), 위 문단의 "L5 가 정상이면 §4.3 감속 대상을 L4→L5 로" 서술은 설계 의도이지 현재 구현이 아님을 여기 기록한다.

**감속은 시각 기준으로 시작한다 `[확정 A-5]`.** 실기 접촉 신호가 지문 센서뿐이라, 공이 손바닥에 먼저 닿으면 손가락이 닫히기 전까지 검출이 늦을 수 있다. 따라서 `DECEL` 진입은 $now_{lead}\ge t_c$ 로 하고, 지문 센서는 결과 판정과 abort에만 쓴다.

**E-STOP·fault 임시 기준 `[확정 P-1]` (S9 전까지).**
- E-STOP 중 출력은 CM 이 `BuildHoldOutput` 으로 대체한다. 슈퍼바이저는 출력을 만들지 않고 **상태만 정리**한다 (`TriggerEstop` 에서 진행 중 시행을 `Aborted` 로 종결, plan·손 시퀀스 무효화). HOLD 끝에서 이미 판정된 시행은 진행 중이 아니다 — 복귀 중 E-STOP 이나 `ABORT_SAFE` 재진입은 그 판정을 `Aborted` 로 덮지 않는다.
- `ClearEstop` 후 **자동 재개 금지** — `IDLE` 로 가고, $q_c$ 와 CLIK 앵커를 $q_{meas}$ 로 reseed 한다. 다음 시행은 homing 부터 다시 한다.
- fault 는 E-STOP 과 분리된 컨트롤러 래치다. `ClearEstop` 은 fault 를 풀지 않고 `ResetFault` 는 E-STOP 을 풀지 않는다. RT 경로의 try/catch·deactivate 는 쓰지 않는다 (RT-2).
- 손 자세 유지로 인한 파지력 소실 등 부작용 검토와 정책 전체는 S9 (D-13).

**S5.1 최소 E-STOP 계약 (plan §4.4 S5.1, `[CONCERN] E-8` — 2026-09-22 승인, S5 에서 구현 — PR #564).** 이 계약은 위 P-1 임시 기준을 구현 수준에서 좁힌 것이다.
- (a) `TriggerEstop`·`ClearEstop`·`ResetFault`·`ResetTargetInitialization` 훅은 **atomic 요청·epoch 만 갱신**한다. reset 자체(값을 되돌리는 동작)의 **유일한 writer 는 RT tick** 이다 — 훅이 직접 상태를 되돌리지 않는다.
- (b) plan·궤적·공분산·손·FSM·타이머 무효화는 D-23 순서(activation generation 판정)로 RT tick 이 수행한다.
- (c) 해제 후 자동 재개는 하지 않는다.
- (d) `ClearEstop` 은 컨트롤러 fault 래치를 풀지 않는다 — E-STOP 경로와 fault 경로는 별개다(위 P-1 규칙과 동일).
- (e) **무장은 명시적 행위다 (A-S5-3, 2026-09-22).** 운용자 채널은 컨트롤러 노드의 파라미터 `catching.enable` (기본 false) 이고, 파라미터 콜백은 atomic 만 갱신한다. **RT tick 이 E-STOP 발동·해제 양쪽과 fault 래치에서 이 latch 를 내린다** — 그래서 (c) 의 "자동 재개 금지" 가 운용 규율이 아니라 메커니즘이다. 활성화(`on_activate`) 도 무장이 아니다: 활성화가 무장이면 E-STOP 복구가 지나가는 deactivate→activate 사이클이 곧 재개가 된다
- 구현 노트 (S5.1): 요청은 **flag 가 아니라 epoch** 이다. 두 tick 사이에 발동→해제가 모두 끝나면 flag 는 false 로 돌아와 있어 tick 이 "아무 일도 없었다" 로 읽고 $q_c$ 를 정지 너머로 이어가기 때문이다. tick 은 "지금 켜져 있는가" 가 아니라 "내가 마지막으로 처리한 값에서 움직였는가" 를 묻는다
- 이 계약은 팔 명령 경로·CLIK 앵커가 아직 없는 S4.0 에는 적용되지 않는다(S4.0 은 base 기본 동작과 CM 의 hold 방어선에 맡긴다) — E-8 대상은 그것들이 생기는 **S5.1** 부터다. 전체 물리 정책(D-13)은 S9.

**S7.2 driver 규칙 (설계 확정 #537 S7 결정 2026-09-23, 구현 PR #571).** FSM driver(`Compute` 안의 매 tick 진행)가 지켜야 할 규칙을 전이표와 별도로 둔다. 전이표(§4.1 표, §5.1)는 그대로다.

- **R-PREC (사유 우선순위).** 한 tick 에 사유는 하나이므로 순서를 고정한다: `ESTOP` > fault reset/escalation > 준비 상실(`kParamsTbd`) > 법칙 실패·치명(`kQpFailed`/`kJointConflict`/`kTrackErr`/`kBallStaleLong`/`kRefSaturated`) > 시간 전진(`kNone` — commit/close/decel 정지/T_hold 경과/복귀 완료) > 기록 전용(`kBallStaleCommitted`·`kHorizonExtrap`·`kTipStale`·`kHandTimeout`). 기록 전용 사유는 **전진이 없는 tick 에만**, 그리고 그 모드에 §4.2 행이 있을 때만 낸다(행이 없는 모드는 플래그로만 기록).
- **R-ORDER (분기 위치, C-33).** `IDLE`(homing)·`DECEL`·`HOLD`·`RETREAT` 는 vision lane **앞**(`ABORT_SAFE` 와 같은 자리)에서 판정한다. `COMMITTED`/`CLOSING` 은 vision 판정과 무관하게 항상 추종 법칙을 돌리고, 샘플러는 나이와 무관하게 마지막 스냅샷을 쓰며 지평 밖은 외삽으로 계속한다 — `kBallStaleCommitted`/`kHorizonExtrap` 은 기록만 한다.
- **R-WATCHDOG.** homing·retreat 법칙도 `track_err_` 를 계산해 `supervisor.track_err_abort` 를 본다. `RETREAT` 는 `{kRetreat, kTrackErr} → ABORT_SAFE` 행을 그대로 쓰고, `IDLE` 은 행이 없으므로 초과 시 정지 램프 + `arm_requested_` 를 내려(disarm) 기록한다.
- **R-DECEL-ENTRY.** tick 의 `now` 는 `Compute` 머리에서 **한 번** 읽어 법칙·시퀀서·판정에 같은 값을 넘긴다(`RunTrackingTick` 이 따로 시계를 읽지 않는다). `DECEL` 진입 tick 은 추종 `Step` 을 한 번 더 돌려 그 출력을 §4.3 의 $(x_s,\dot x_s)$·$t_s$ 로 삼고, 다음 tick 부터 $\tau=now_{lead}-t_s$ 로 소비한다.
- **R-ADMIT (C-31, L3 §4.11).** `JudgePlan` 에 조건 (g) `t_c-now\le T_{freeze}` 거부(`kTooLate`)를 추가한다 — 현재는 이 조건이 없어 $T_{freeze}$ 안의 $t_c$ 를 가진 plan 도 채택 다음 tick 에 commit 해 버린다. 검증기는 `T_freeze ≥ T_close_e2e + T_arm + margin` 을 강제한다.
- **R-CLOSE.** `COMMITTED→CLOSING` 은 시퀀서의 `close_issued`(규칙 `now ≥ t_cmd-h/2`)로 전이한다 — `HandCommandDue` 대신 `time_types` 에 `HandCommandDueRounded(now, t_cmd, h)` 를 추가해 시간축 표(plan §3)를 참으로 유지한다. **기록 1 tick 지연 (D-S8-10 (a), 2026-09-24 문서 기록만)**: `Compute()` 순서상 닫힘 명령이 나간 tick 의 기록에는 아직 `COMMITTED` 가 실리고 `CLOSING` 은 다음 tick 부터 기록된다 — 손 명령 시각 자체 (G6-A) 에는 영향이 없으므로 코드를 바꾸지 않는다. 오프라인 분석은 닫힘 시각을 mode 열이 아니라 손 명령 열에서 읽는다.
- **R-TRACK (Q15, 2026-09-23 사용자 확정).** 동결 후 샘플은 commit 시점 `committed_generation_` 에 고정한다 — 다른 generation 스냅샷은 stale 로 보고 동결 plan 으로 계속하며 `kBallStaleCommitted` 만 기록한다. 재무장 뒤에는 `last_trial_generation_` 을 기록해 그 generation 은 usable 로 보지 않는다(새 generation 이 올 때까지 대기). 두 멤버 모두 `ResetForRearm` 이 쓰고 `ResetTrialState` 가 지운다(§4.8).

### 4.2 abort·실패 사유

| 코드 | 조건 | 발생 가능 상태 | 처리 |
|---|---|---|---|
| `BALL_STALE` | L1 stale (나이 초과 또는 지평 소진) | `TRACKING`, `APPROACH` | `TRACKING` 이면 `ARMED`, `APPROACH` 면 `RETREAT` |
| `BALL_STALE_COMMITTED` | L1 stale | `COMMITTED`, `CLOSING` | 계속 진행 (동결 plan으로 포구 시도), 기록 `[확정 A-6]` |
| `BALL_STALE_LONG` | stale 지속 > `supervisor.stale_committed_max_s` | `COMMITTED`, `CLOSING` | `ABORT_SAFE` `[확정 A-6]` |
| `TRACK_CHANGED` | L1 트랙 변경 판정 (L1 §4.4 — 트랙 epoch. `generation` 을 어떻게 쓰는지는 L1 이 정한다, D-4) | `TRACKING`, `APPROACH` | `ARMED`/`RETREAT`. `PointCloud2`에 트랙 상태가 없어 `STATUS_LOST`를 이것으로 대체 |
| `HORIZON_EXTRAP` | L2 `after_horizon=true` (지평 **뒤**로 외삽, $now_{lead}$ 기준) | 전 구간 | `APPROACH`면 `RETREAT`, 동결 후면 기록 후 계속 |
| `PRED_INCONSISTENT` | L1 예측 일관성 지표 $\bar\nu$ 가 임계 초과 (L1 §4.5) | `TRACKING`, `APPROACH` | `RETREAT` (`TRACKING` 이면 `ARMED`). 동결 후에는 기록만. **명시 면제 — 발화 0 (2026-09-24 D-S8-7 (a))**: $\bar\nu$ 생산자를 만들지 않기로 했다 (`io.pred.nu_reg` 은퇴, L1 §6). 전이표 행 (`kPredInconsistent`) 은 남고 완전성 검사 대상이지만 어떤 tick 도 이 사유를 내지 않는다. 예측 일관성은 추정기 innovation/nis 를 오프라인으로 본다 (plan §7.3) |
| `NO_CATCHABLE_PLAN` | 계획기가 plan 없음을 게시 (catchability manipulability 미달 D-18, IK 실패, 도달 불가 — 세부 사유는 L3 plan 사유 코드) | `TRACKING` | 비치명. `TRACKING` 유지, 기록 |
| `PLAN_INVALID` | plan 무효 | `APPROACH` | `RETREAT` |
| `QP_FAILED` | L5 QP 실패 status | 전 구간 | `ABORT_SAFE` (QP 비의존 경로, §4.1). 연속 $N_{qp}$회면 `FAULT` |
| `REF_SATURATED` | L4 `ref.saturated` 가 연속 `supervisor.sat_ticks` tick (기본 **60** = 0.12 s @ 500 Hz, provisional — S7 설계 확정, D-8: γ 하향 없음. 처음 제안한 5 는 정지 상태에서 접근을 시작한 reference 의 정상 포화 (단위 fixture 실측 10·76 tick) 를 잘랐다. 100 으로 올린 뒤 sim 25 투척 (`260923_2336`) 의 정상 연속 길이가 max 44 · p99 42.5 로 나와 50 으로 내렸다. 50 재측정 (`260924_0013`) 에서 이미 빗나간 공의 CLOSING 에서 1 회 발화했고 미발화 최장이 40·33 이라 여유를 두어 60 으로 올렸다, #537 결정 2026-09-24) | `APPROACH`, `COMMITTED`, `CLOSING` | `APPROACH` 면 `RETREAT`, 동결 후면 `ABORT_SAFE` `[확정 D-8]`. `sat_ticks` 는 sim 정상 시행의 연속 길이 분포로 확인 후 확정한다 (D-S7-4, plan §7.3) |
| `GAMMA_DERATED` | v0.5 에서 삭제 — γ 하향은 v1 범위 밖 (D-8, §4.6) | – | – |
| `SAT_NEAR_TC` | v0.5 에서 삭제 — `REF_SATURATED` 로 대체 (D-8) | – | – |
| `JOINT_CONFLICT` | L5 `bound_conflict` | 전 구간 | `ABORT_SAFE` (QP 비의존 경로) |
| `TRACK_ERR` | $\Vert q-q_c(now-T_{arm})\Vert>$ 임계. ⚠️ **구현 편차 (2026-09-24 확인)**: `DemoCatchingController::UpdateTrackError` 는 지연 링 없이 $\Vert q_{meas}-q_c(now)\Vert$ 를 쓴다 (§5.2 의 링은 구현되지 않았다) — 선행을 켜도 명령이 측정보다 T_arm 앞서므로 피크가 줄지 않는다 (τ 0.2 sim 관측 ~0.77 rad → 종전 `track_err_abort` 1.54). 정의를 링으로 바꾸는 것은 RT 변경이라 S8 범위 밖. S8-B (τ 0.05 sim) 의 t_c 전 피크는 lead on·off 모두 0.21 rad 라 p1b 임계는 0.42 | 전 구간 | `ABORT_SAFE` |
| `ABORT_ESCALATED` | `ABORT_SAFE` 중 재차 치명 조건 | `ABORT_SAFE` | `FAULT` |
| `ESTOP` | E-STOP 발동·해제 (§4.1 P-1, S5.1 최소 계약) | 전 구간 | 발동: 상태 정리, 해제: `IDLE`. 단 `FAULT` 에서는 `FAULT` 유지 — 해제가 fault 래치를 풀지 않는다 (P-1, S5.1(d)) |
| `FAULT_RESET` | `ResetFault` | `FAULT` | `IDLE` |
| `SPEED_SCALING` | speed scaling ≠ 1 | 전 구간 | `ABORT_SAFE`. **repo 에 신호 출처 없음 → sim 비활성, S10** |
| `CLOCK_UNHEALTHY` | PTP 임계 초과 | 전 구간 | `IDLE`·`ARMED`에서는 진입 거부, 운행 중 `ABORT_SAFE`. **신호 출처 없음 → sim 비활성, S10** |
| `PARAMS_TBD` | L0 검증 실패 (활성 구성 TBD·provisional) | `IDLE` | 진입 거부 |
| `HAND_TIMEOUT` | L6 폐쇄 타임아웃 | `CLOSING`, `DECEL` | 기록, 계속 |
| `TIP_STALE` | 지문 센서 stale | `COMMITTED` 이후 | 판정 불가로 기록 |

**`TIP_STALE` 판정 경로는 D-24 (a) 다 (2026-09-22 사용자 확정, 배선은 S5.2e).** 지문 센서 lane 에는 수신 시각도 sequence 도 없었다 — RT backend 3종 중 관절 상태 콜백만 `last_state_ns_` (backend watchdog stamp) 를 갱신하고, 지문 센서 자체의 freshness 는 관측할 수 없다. 관절이 fresh 한 채 센서만 멈추면 옛 힘을 새 접촉으로 오판할 수 있다. 채택한 경로는 (a) `rtc_base` `DeviceState` 센서 lane 에 `recv_steady_ns`·`sequence`·`valid` 를 추가하고 backend 3종이 채워 `ControllerState` 로 전달 (PROC-3, P5 — grasp 에도 같은 gap 이라 함께 닫힌다); (b) 포구 컨트롤러 소유 mailbox 는 device 경로와 공존하는 중복 lane 이라 기각했다 (plan §7.3). `TIP_STALE` 은 그 `recv_steady_ns` 의 수신 나이가 `supervisor.contact.t_stale`(C-16, §6)을 넘으면 판정한다.

`BALL_STALE_COMMITTED` 는 A-6 으로 확정됐다. 동결 후에는 짧은 누락으로 포기하는 것보다 동결 plan으로 진행하는 편이 안전하다고 본다. stale 지속 시간 상한 `supervisor.stale_committed_max_s` 는 **0.10 s (provisional, #537 S7 결정 2026-09-23, §6)** 로 닫혔다 — 어느 물리량(공분산 성장, 포획 반경 오차 할당, abort 정지거리 $\Vert\dot x\Vert^2/(2a_{dec})$)으로 이 값을 조일지는 S8 에서 보기로 했고, **S8-B 튜닝 세트 (200 발) 에서 COMMITTED 스냅샷 age 가 최대 65 ms 로 `io.t_stale` 에도 닿지 않아 0.10 을 유지하고 두 로봇 YAML 에 명시했다** (2026-09-24). 조일 물리량은 실기 (S10) 데이터로 다시 본다.

**사유 우선순위·발화 위치 규칙(R-PREC·R-ORDER)은 §4.1 "S7.2 driver 규칙" 을 본다** — 위 표는 어떤 상태에서 어느 사유가 나올 수 있는지만 정의하고, 한 tick 에 여럿이 동시에 성립할 때 어느 것을 내는지는 그 규칙이 정한다.

### 4.3 가상 감속 대상 `[논문 외 유도]`

`DECEL` 진입 시각 $t_s$의 기준 상태 $(x_s,\dot x_s)$에서 시작한다. $\hat u_s=\dot x_s/\Vert\dot x_s\Vert$, $\tau=t-t_s$, $\tau_s=\Vert\dot x_s\Vert/a_{dec}$.

$$p_v(\tau)=x_s+\dot x_s\tau-\tfrac12a_{dec}\hat u_s\tau^2,\quad v_v(\tau)=\dot x_s-a_{dec}\hat u_s\tau,\quad a_v=-a_{dec}\hat u_s\qquad(\tau\le\tau_s)$$

$\tau>\tau_s$이면 $p_v=x_s+\tfrac12\Vert\dot x_s\Vert\tau_s\hat u_s$, $v_v=a_v=0$.

`DECEL` 진입은 $now_{lead}\ge t_c$ 이고 $t_s$ 는 그 tick 의 $now_{lead}$ 다 (plan §3 — 감속 대상 전환도 팔 명령이다). $\tau$ 는 steady 실측 차로 구하고 tick 수 × `dt` 로 세지 않는다.

L4에 이 대상을 넣고 γ를 상수 1로 바꾼다($\dot\gamma=\ddot\gamma=0$). 전환 직후 오차는

$$e=x_s-p_v(0)=0,\qquad \dot e=\dot x_s-v_v(0)=0$$

로 **정확히 0**이다. "연속"이 아니라 "0"이라는 점에 주의한다 — 전환 직전의 $e,\dot e$ 는 DS 자체의 잔여 수렴오차만큼 0이 아니므로(실측 $\Vert e^-\Vert\approx1.1$ mm, $\Vert\dot e^-\Vert\approx9.4$ mm/s, §L4 4.9의 $\epsilon_{conv}$ 와 같은 값) 오차 자체는 그만큼 점프한다. **정확히 연속인 것은 기준 상태 $(x,\dot x)$** 이고, 설계상 필요한 것은 그쪽이다. 기준 가속도는 불연속이다(jerk 무한). 필요하면 $a_{dec}$를 짧은 램프로 올린다(`supervisor.decel.ramp_time`).

**층간 제약: $a_{dec}\le$ `reference.a_max`.** $e^+=\dot e^+=0$ 이므로 전환 직후 $u_{des}=a_v$ 가 되어 크기가 정확히 $a_{dec}$ 다(실측: 0.08 → 15.0 m/s²로 도약). $a_{dec}$ 가 L4 가속 한계보다 크면 `DECEL` 첫 틱부터 포화가 걸린다. L0 파라미터 검증기가 이 관계를 검사한다.

정지거리 $\Vert\dot x_s\Vert^2/(2a_{dec})$는 L3 §4.9의 예약값($\dot x_s\approx\gamma_f v_c$)과 같다. **`a_dec` 는 단일 키** `supervisor.decel.a_dec` 이고 L3 는 이 키를 읽는다 (S0.3 — 이름이 둘인 같은 값 제거).

`ABORT_SAFE`도 L5 가 정상이면 같은 식을 현재 기준 상태에서 적용한다. `QP_FAILED`·`JOINT_CONFLICT` 로 들어온 경우는 §4.1 의 QP 비의존 관절공간 감속을 쓴다.

감속 목표 계산은 ROS 비의존 순수 조각으로 S1.8 에서 이식한다.

### 4.4 접촉 판정

**부호 규약.** sim 과 실기 두 경로 모두 **finger-on-object** 부호다 — 실기 P1b `HandSensorState` (250 Hz) 와 같게 `rtc_mujoco_sim` 이 커밋 0fcc1d23 부터 fingertip-on-environment 로 발행한다 (코드 확인. repo 규약상 sim 쪽 부호 스위치를 다시 넣지 않는다. 부호 변환 지점은 `rtc::grasp::PullContactConfig::force_sign` 한 곳). `rtc_msgs` 메시지 주석의 "sim 은 반대 부호" 서술은 stale 이다. 따라서 접촉 판정은 힘의 크기·법선 성분을 **그대로** 쓰고 별도 정규화를 두지 않는다. S7.3 구현은 바이어스를 뺀 크기 $\Vert F-b\Vert$ 만 판정하므로 (§4.4) 이 부호 규약에 의존하지 않는다. 센서 주기·스탬프는 입력단에서 기록한다 (판정 시각은 수신 steady 시각, `header.stamp` 는 staleness 판단에 쓰지 않는다).

센서 $i$의 바이어스 $b_i$는 **`ARMED`/`TRACKING` 구간(손 `q_pre` 정지 중 — Q4, §4.1)** 의 **지수이동평균(EMA, `supervisor.contact.baseline_alpha`)** 으로 추정한다 — 원형 버퍼가 아니다(C-6, §4.8 정정). 잡음 표준편차 $\hat\sigma_i$도 같은 창·같은 EMA 로 구한다. v0.2는 창을 `COMMITTED` 구간으로 잡았는데 그 길이가 $T_{freeze}-T_{close,tot}\approx T_{arm}+T_{margin}$(수십 ms)뿐이라, 센서 주기 100 Hz면 표본 2–5개로 $\hat\sigma_i$ 를 추정하게 된다. 창은 `RETREAT → ARMED`(`ResetForRearm`)에서 비운다(§4.8). **바이어스 표본 수가 `supervisor.contact.n_baseline_min`(기본 20) 미만이면 결과는 `Undetermined`** 로 기록한다 — `Missed` 로 오판하지 않는다.

**디바운서는 tick 이 아니라 샘플 단위로 먹인다 (S7 설계 확정).** 실기 지문 센서는 250 Hz, 제어 tick 은 500 Hz 이므로, `UpdateBaseline`/`UpdateContact` 는 매 tick 이 아니라 `inference_sequence[g]` 가 바뀐 tick 에만 호출한다.

$$f_i(t)=\Vert F_i(t)-b_i\Vert,\qquad c_i(t)=\mathbb 1\big[f_i>\max(f_{\min},\,k_\sigma\hat\sigma_i)\big]$$

$N_{deb}$개 연속 샘플이 참이면 센서 $i$ 접촉으로 확정한다 (debounce 는 순수 조각, S1.8).

결과 판정(`HOLD` 종료 시):
- **포획:** 판정 창 $[t_{cmd},\,t_c+T_{conf}]$ (실제 시각 $now$ 로 비교 — 공의 물리 시각과 같은 축, plan §3) 안에 접촉 센서 수가 $m_{\min}$ 이상이었고, `HOLD` 종료 시점에도 $m_{\min}$ 이상이 유지되는 경우.
- **실패:** 판정 창 안에 접촉이 없는 경우.
- **미확정:** 센서 stale, 또는 바이어스 표본 수가 `n_baseline_min` 미만.

오경보 확률은 센서 잡음 분포에 의존한다. $k_\sigma$는 시뮬레이션·실기 잡음 측정 후 정한다(가우시안 가정이면 $k_\sigma=3$에서 단측 약 0.13%, 등급 a).

### 4.5 준비(ARMED) 조건

1. L0 파라미터 검증 통과(TBD 없음)
2. 시계 건강(실기) — 신호 출처 없음, sim 비활성, S10
3. L5 `lead_enable=true`이면 `T_arm` 확정
4. 로봇이 대기 자세 허용오차 안에 있음 (`IDLE` homing 으로 도달, §4.1)
5. speed scaling = 1(실기) — 신호 출처 없음, sim 비활성, S10
6. 손 `q_pre` 도달 (Q4, 2026-09-23 사용자 확정 — `q_open` 은 homing 중에만 쓰고 대기 중 손은 항상 `q_pre` 다, §4.1)

**조건 상실의 처리 `[확정 S5, 2026-09-23 코드리뷰]`.** §4.2 에 전용 사유가 없어 전이표는 `PARAMS_TBD` 를 재사용한다(전이표 헤더가 근거를 갖는다). 어디로 가는지는 **그 모드가 운동을 싣고 있을 수 있는가**로 갈린다:

- `IDLE` 은 self-loop, `ARMED`·`RETREAT` 는 `IDLE`. `ARMED` 는 팔이 이미 서 있어 그대로 성립한다. **`RETREAT` 는 더 이상 "팔이 서 있다" 가 근거가 아니다** — S7 의 `RETREAT` 는 정지 램프 + 관절공간 복귀로 운동을 신는다(§4.1). 이 행이 여전히 성립하는 이유는 **`IDLE` 이 그 램프를 소유하기 때문**이다: `RETREAT` 중 disarm 되면 carried `arm_qd_cmd_` 를 그대로 `IDLE` 로 넘기고, `IDLE` 이 R-IDLE 규칙(§4.1)으로 정지까지 램프를 마저 돌린다 — 전이 자체는 즉시 일어나도 관절 속도가 즉시 0 이 되는 것은 아니다.
- `TRACKING`·`APPROACH`·`COMMITTED`·`CLOSING`·`DECEL`·`HOLD` 는 **`ABORT_SAFE`** 다. 운전자가 approach 중에 무장을 내리는 것이 정상 경로이고, 전이 행이 없으면 모드가 그대로 남아 드라이버가 법칙 호출을 멈춘 채 실려 있던 관절 명령이 **그 자리에서 얼어붙는다** — L5 의 감속 계약이 절대 내보내지 말라고 하는 1-tick 무한 감속이다. `ABORT_SAFE` 는 그 ramp 를 소유하고, 정지 후 `RETREAT` 로 빠지는 경로를 이미 갖고 있다.
- `ABORT_SAFE` 는 **행이 없다 (의도적)**. 그 모드의 출구는 정지 완료이고, 여기서 사유를 답하면 매 tick 그 판정을 가로채 ramp 가 끝나도 나가지 못한다. 드라이버는 이 한 모드에서만 준비 조건 검사를 통과시킨다.

이렇게 해서 무장 해제는 `APPROACH → ABORT_SAFE → (정지) → RETREAT → IDLE` 로 **종결**한다 — 팔은 ramp 로 서고, 운전자는 `IDLE` 로 돌아온 것을 본다.

### 4.6 γ 하향 (포화 대응 1차 수단) — v1 범위 밖 `[확정 D-8]`

v0.5 에서 삭제 — γ derate 는 v1 범위 밖이다 (D-8). 참조 구현 probe 에서 Frozen 분기의 γ_min 미보장, 완화 분기 무효, 기본 램프의 가속 피크 증가, 램프의 $t_c$ 초과가 확인됐다. v1 의 포화 대응은 **`COMMITTED` 전이면 `RETREAT`, 이후면 `ABORT_SAFE`** 다 (§4.2 `REF_SATURATED`). 계획 단계의 완충은 η_v 교차제약(D-9)이 맡는다. S8 에서 포화 빈도를 측정해 재설계안 도입 여부를 정하며, 이전 분석은 L4 §5.2.1 에 남아 있다.

### 4.7 충격량 예산 `[논문 외 유도]` `[TBD-IMP-01]`

v0.1에는 이 절이 없었다. 본 시스템은 UR5e를 position 명령(실기 `ur_driver_native` → vendor `forward_position_controller`)으로 구동하므로 **접촉 순간의 임피던스가 사실상 위치 서보 강성**이고, 순응 요소가 없다. [R16]의 문제의식이 그대로 적용된다 — 빠른 물체와 접근하는 로봇 사이의 속도 불일치가 큰 충격력을 만들고, 접촉 불안정과 손상으로 이어진다.

본 설계의 유일한 완화책은 상대속도를 줄이는 soft catch다.

$$\Delta p=m_{ball}\,(1-\gamma_f)\Vert v(t_c)\Vert$$

문제는 γ가 작을 수밖에 없는 구간이 넓다는 것이다(L3 §4.5, 마스터 §4.1). $\gamma_f=0.25$, $\Vert v\Vert=2$ m/s면 상대속도가 1.5 m/s이고, 이 운동량을 지문 센서만 달린 손가락과 위치 서보 팔이 받는다.

**계획·검증 단계에서 다음을 산출하고 기록한다.**

1. **충격량과 손가락 토크.** $\Delta p$ 를 접촉 시간 $\Delta t_{imp}$ 로 나눈 평균 힘 $\bar F=\Delta p/\Delta t_{imp}$ 와, 그것이 만드는 관절 토크를 P1b 모터 한계와 비교한다. 설정·모델값(YAML `max_torque` = URDF effort = MJCF forcerange)은 **3.0 N·m** 로 일치한다(L6 G6-3). 이 값을 그대로 운용 한계로 쓸 수 있는지는 **미확정** — 권위 있는 운용 한계(nominal·continuous·peak·설정값 중 어느 것)는 **D-12 대기** 이고(plan §7.3), 확정 전에는 이 게이트의 토크 비교 부분을 `NOT_EVALUATED` 로 기록한다(G7-B3). $\Delta t_{imp}$ 는 시뮬레이션 접촉 참값(S3.3 — 시각·충격량·접촉력 출력)에서 측정한다.
2. **팔 쪽 반력.** position 명령은 접촉 중에도 계속 진행하므로, 공이 손 안에서 감속되는 동안의 반력은 구조와 관절이 받는다. UR5e 보호 정지 임계 대비 어디인지 확인한다(`[HW-P1B]`, 저속부터).
3. **충격 후 CLIK 괴리.** 충격으로 $q$ 가 $q_c$ 에서 벌어지면 (D-6 으로 CLIK 은 $q_c$ 에서 평가하므로 이 괴리는 `TRACK_ERR` 로만 보인다) L7 `TRACK_ERR` 가 오동작할 수 있다. `track_err_abort` 를 정할 때 충격 구간을 제외하거나 임계를 시간 가변으로 둔다.
4. **반발.** L3 §4.5의 $d_{eff}=d(1+1/e)$ 는 법선 단일 충돌 모델이다. [R19]가 다루는 접선 컴플라이언스는 무시한다는 가정을 명시한다.

**계획에 거는 게이트.** $\Delta p\le\Delta p_{\max}$ 를 L3 후보 게이트에 추가한다(`TBD-IMP-01` 확정 후). 이 게이트가 γ 창의 하한을 실질적으로 끌어올린다. 토크 도출 가속 한계(D-16)의 여유 $\eta_\tau<1$ 도 이 충격을 위한 것이다(plan §9).

**본 설계가 하지 않는 것.** [R16]의 강성·접촉력 동시 최적화와 접촉점 선택, [R18]의 reference spreading(충격 순간 기준 궤적 불연속 처리)은 **범위 밖**이다. 둘 다 토크 또는 임피던스 인터페이스를 전제하는데 UR5e는 position으로 확정돼 있다(마스터 §1.1). 저속 구간에서 성공률이 확보되지 않으면 이 제약을 재검토해야 한다 — 그때의 선택지가 위 두 문헌이다.


### 4.8 재무장 리셋 목록 `[확정, S7.4 에서 함수로 분리]`

`RETREAT → ARMED` 전이에서 **다음을 전부 초기화한다.** 하나라도 빠지면 직전 시행의 상태가 남아 두 번째 투척이 다르게 동작한다. v0.2는 이 목록이 없었고, §9 시나리오가 전부 단발 시행이라 게이트에서도 잡히지 않았다. 아래는 소유 layer 별로 정리한 **단일 표**다.

**두 함수로 분리했다 (S7.4, C-7/C-29/C-30).** 멤버마다 어느 리셋이 되돌리는지는 컨트롤러 헤더의 리셋 표가 SSoT 이고, `test_catching_reset_table.py` (행 존재) 와 `test_catching_reset_probe.cpp` (행의 진위) 가 검사한다. `ResetForRearm()` 은 `RETREAT → ARMED` 에서만 돈다. `ResetTrialState()` = `ResetForRearm()` + activation/E-STOP 몫이고, 무장 latch 해제·E-STOP·명시적 리셋에서 돈다. 아래 표의 "함수" 열은 어느 쪽(들)이 그 대상을 리셋하는지를 가리킨다 — **면제**로 적은 대상은 어느 쪽도 손대지 않는다(의도적).

| 대상 | 소유 layer | 함수 | 리셋 내용 | 빠뜨렸을 때 |
|---|---|---|---|---|
| `PlanSnapshot` 박스 (valid flag 포함) | S6 계획기 출력 / RT 소비 | 둘 다 | `valid=false` 로 무효화하고 `last_consumed_plan_id` 기록 | 옛 plan 의 $t_c$ 가 이미 과거라 `TRACKING→…→DECEL` 을 몇 틱에 통과하며 엉뚱한 곳에서 손을 닫는다 |
| 계획기 후보 hysteresis·이전 최선 후보 | S6 계획기 (스레드 내부) | 둘 다 | 이전 시행의 최선 후보·hysteresis 상태를 지운다 | 새 시행의 첫 후보가 직전 시행의 후보와 비교돼 갱신이 지연되거나 억제된다 |
| 계획기 공분산 버퍼 | S6 계획기 (A-3, D-22 token) | 둘 다 | 버퍼·token 무효화 | 직전 시행의 옛 공분산이 새 궤적과 짝지어져(N/N−1 혼합) 게이트를 오판정한다 |
| `reset_floor_ns_` / `planner_reset_epoch_` | RT tick (D-22, D-23) | 둘 다 | **재무장에서도** `reset_floor_ns_ = SteadyNowNs()` 와 `planner_reset_epoch_` bump 를 **같은 자리에서** 갱신한다(C-7) — admission 은 `reset_floor_ns` 가 가르고, epoch 은 계획기 쪽 판정만 쓴다 | 재무장 직후 abort→재무장 fast cycle 에서 직전 시행용으로 이미 계산된 옛 plan 을 새 시행이 받아들인다 |
| 계획기 wake 잔여 신호 (eventfd) | S6 계획기 스레드 (D-7c) | **면제** | drain 하지 않는다 — **어떤 리셋에서도** (C-7, 의도적) | 비우면 리셋을 보는 동안 이미 올라온 **새 시행**의 궤적 신호까지 지워, 새 시행의 첫 plan 이 wake timeout 만큼 늦어진다. 남겨도 안전한 이유는 위 reset floor 가 옛 시행의 plan 을 걸러 주기 때문이다 |
| L4 soft-catch 기준 생성기 | L4 | 둘 다 | 측정 자세·속도 0 으로 리셋 — $p_c$ 와 γ 프로파일까지 (L4 §5.3) | 직전 포구점으로 복귀하고, 하향된 γ가 남는다 |
| L2 hint 커서 | L2 (샘플러) | 둘 다 | 0 | 정확성은 이진 탐색이 지키지만 틱 비용이 흔들린다 |
| L5 (확장 CLIK) 직전 $\dot q$ 상태·앵커 | L5 | 둘 다 | $\dot q$ 0, 앵커만 리셋 — `arm_cmd_seeded_`·`arm_q_cmd_` 자체는 **유지**한다(C-32: 재무장에서 무조건 재시딩하면 서보 오차만큼의 속도 계단이 생긴다) | 앵커를 안 지우면 첫 틱 가속 경계가 옛 속도 기준이라 `bound_conflict` 오abort. 반대로 $q_c$ 를 재시딩하면 그 자체가 계단이 된다 |
| L6 시퀀서 | L6 | 둘 다(값은 다름) | `ResetForRearm`: **`Ready`(q_pre) 위상** — `Open` 이 아니다(C-30: Q4 로 대기 중 손은 항상 `q_pre`). `ResetTrialState` 는 추가로 시퀀서를 **inactive** 로 둔다(E-STOP·activation, C-17) | `Open` 으로 잘못 리셋하면 재무장 직후 손이 열려 다음 시행의 준비 조건(§4.5-6)을 못 채운다 |
| L7 접촉 바이어스·잡음 창 | L7 | 둘 다 | EMA 상태(바이어스·분산 추정치)와 표본 카운트를 리셋 — 원형 버퍼가 아니다(C-6) | 직전 시행의 접촉력이 바이어스에 섞인다 |
| L7 접촉 debounce 카운터 | L7 | 둘 다 | $N_{deb}$ 연속 카운터를 0 으로 | 직전 시행 종료 시점의 연속 참 카운트가 남아 새 시행 초반에 한두 샘플만으로 접촉 확정된다 |
| L7 stale 타이머 | L7 | 둘 다 | `BALL_STALE_COMMITTED` 지속 시간 0 | 직전 시행의 stale 누적으로 `BALL_STALE_LONG` 오abort |
| L7 트랙 identity (`committed_generation_`, `last_trial_generation_`) | L7 (§4.1 R-TRACK, Q15) | 분리(우측 참고) | `ResetForRearm` 이 `last_trial_generation_` 을 **쓰고**(직전 시행 generation 을 거부 대상으로 기록), `ResetTrialState` 가 그 값을 **지운다** | 지우지 않으면 그 트랙이 E-STOP·fault 리셋을 넘어 계속되는 경우(track epoch 는 activation 과 별개, D-4) 여전히 유효한 그 generation 이 다음 활성화 이후에도 usable 로 보이지 않아, 새 vision generation 이 도착할 때까지 계획을 영영 못 받는다 |
| L7 `QP_FAILED` 연속 카운터 (`qp_fail_streak_`) | L7 | **`ResetTrialState` 전용 — `ResetForRearm` 은 면제(C-29)** | `ResetTrialState` 만 0 으로. 재무장에서 지우면 안 된다 | 재무장마다 지우면 연속 QP 실패가 시행 경계를 못 넘어 `FAULT` 에스컬레이션(`n_qp`)에 **영영 도달하지 못한다** — 반대 방향의 결함이다 |
| L7 결과·사유 | L7 | 둘 다 | `Outcome::None`, `Reason::None` | 진단 오염 |

**완전성 규칙.** 전이표 완전성 검사(§4.1 G7-A)와 같은 방식으로, **모든 stateful 멤버는 이 표에 있거나 명시적으로 면제되어야 한다** — 새 stateful 멤버를 추가하면서 이 표를 갱신하지 않는 것을 금지한다 (S7.4 게이트, G8-A2). 표의 주장이 참인지는 **런타임 poison 테스트**로 검증한다 — 테스트 전용 accessor 로 모든 RT 멤버에 비기본값을 채운 뒤 `ResetForRearm`/`ResetTrialState` 를 불러, 표가 주장하는 멤버만 바뀌고 면제는 그대로인지 단언한다(존재 린터만으로는 값이 실제로 리셋되는지 보장하지 못한다).

L7이 plan 을 받아들일 때 §4.1 R-ADMIT 조건(g) $t_c-now\le T_{freeze}$ 이면 `kTooLate` 로 거부한다 — 마지막 조건이 과거·너무 이른 plan 을 걸러낸다.

**RETREAT 순서 (Q13, #537 S7 결정 2026-09-23 · release 규칙은 2026-09-24 결정으로 교체).** 진입 → 정지 램프(`JointSpaceDecelStep`; `ABORT_SAFE` 경유면 no-op. 측정 팔이 정지 명령을 `track_err_abort` 안으로 따라잡을 때까지 머문다 — 안 그러면 `TRACK_ERR` abort 직후 서보 지연이 복귀 첫 tick 에 다시 `TRACK_ERR` 를 내 `ABORT_SAFE` ↔ `RETREAT` 를 돈다) → 관절공간 복귀(팔이 이미 `pose_tol` 안이면 생략) → 대기 자세 도착에서 손 Release(`q_pre`) → 손 `q_tol` 도달 → `ResetForRearm` → `ARMED`. **RETREAT 는 손을 움직이지 않는다.** 닫힌 손은 판정(Captured·Missed·Undetermined·Aborted)과 무관하게 복귀 내내 닫힌 채이고 대기 자세에서만 열린다. 아직 닫힘 명령이 나가지 않은 commit 은 진입 시 취소한다 — 손은 이미 `q_pre` 이므로 움직임은 없다. 근거: 판정은 지문만 보므로 공이 링크·손바닥에 얹힌 경우도 Missed 로 나온다 (sim `260923_2336` 25 투척 중 1건). 이전 규칙(Q12: Missed/Aborted 는 진입 즉시 Release, Q14: 접촉 확정 후 abort 만 예외)은 그 공을 포구 지점에서 떨어뜨렸다. LEAP 처럼 `q_pre` 도달로 손이 열려도 공이 남는 경우는 sim 드라이버의 `/sim/reset_ball`, 실기는 운용자가 처리한다.

**IDLE 순서.** 검증 통과 + 무장 → 팔이 `pose_tol` 밖이면 손 `q_open` → homing → 도착 → 손 `q_pre` 지시; 안이면(Q13) 손만 `q_pre` → 손 도달 + §4.5 → `ARMED`.

§9 시나리오에 **"연속 2회 투척"** 과 **"abort 직후 재투척"**, 그리고 **"RETREAT 복귀 중 disarm"** 을 넣어야 이 항목들이 게이트에서 검증된다.

## 5. C++ 구현

v0.4 의 입출력 구조체·슈퍼바이저 클래스 스케치는 삭제했다 — repo 에 없는 tick API 와 아직 정해지지 않은 타입에 기대고 있었다. 입출력 구조체의 모양은 L4·L5 의 S1·S2 타입이 정해진 뒤 **S7 에서 확정**했다. 여기서는 확정된 것만 적는다.

### 5.1 인터페이스

```cpp
// namespace rtc::catching (D-1). 전이표·enum 은 ROS 비의존 순수 조각 (S1.8)
enum class Mode : std::uint8_t { kIdle, kArmed, kTracking, kApproach, kCommitted, kClosing,
                                 kDecel, kHold, kRetreat, kAbortSafe, kFault };
enum class Reason : std::uint8_t { kNone, kBallStale, kBallStaleCommitted, kBallStaleLong,
                                   kTrackChanged, kHorizonExtrap, kPredInconsistent,
                                   kNoCatchablePlan, kPlanInvalid, kQpFailed, kRefSaturated,
                                   kJointConflict, kTrackErr, kAbortEscalated, kEstop,
                                   kFaultReset, kSpeedScaling, kClockUnhealthy, kParamsTbd,
                                   kHandTimeout, kTipStale };
enum class Outcome : std::uint8_t { kNone, kCaptured, kMissed, kUndetermined, kAborted };
```

- enum 목록은 §4.2 표와 1:1 이다 (삭제 행 제외). 비치명 사유(`kNoCatchablePlan`, `kBallStaleCommitted`, `kHorizonExtrap` 동결 후, `kHandTimeout`, `kTipStale`)는 전이 없이 기록만 한다.
- FSM 은 포구 컨트롤러의 `Compute` 안에서 매 tick 1회 진행한다. 입력은 L1 스냅샷(SeqLock 에서 읽은 POD), `PlanSnapshot`, 직전 tick 의 L4·L5 결과, `ControllerState` 의 측정값, $now$·$now_{lead}$ (§4.1) 이다. 출력은 L4 대상 선택(실제 공 / 가상 감속 / 정지 목표), 손 시퀀서 명령, 사유·결과다.

### 5.2 구현 규칙

- 전이는 (상태 × 사유) 표 데이터로 판정하고(S1.8), 한 틱에 최대 1회만 일으킨다.
- **별도 전이 로그(SPSC)는 두지 않는다 (C-27, S5.4 구현과 일치하도록 정정).** 전이만 담는 고정 크기 레코드·SPSC 는 만들지 않는다 — 대신 `Compute()` 가 단일 exit 인 매 tick 마다 `mode`·`reason` 을 포함한 전체 POD 를 기본 생성 후 채워 `catching_diag.csv` 에 싣는다(PROC-7, D-20). 전이는 이 per-tick 기록에서 mode 열이 바뀌는 행으로 **오프라인 도출**하며, 플로터가 그 행에 전이선을 그린다. 트랙 식별은 vision 의 `generation`(uint64, L1 §4.4, D-4)이다.
- **시각 비교는 plan §3 (D-2) 의 타입으로만 한다.** 내부 시각은 절대 steady ns 이고, 상대시각은 수치 코어 경계에서만 만든다. `BallTime`·`NowReal`·`NowLead` 를 서로 다른 타입으로 두어 혼용을 막는다 — 판정별 비교 대상은 §4.1 표와 plan §3 표가 같다. 매 tick 의 $now$ 는 steady 실측이며 tick 수 × `dt` 로 계산하지 않는다. **$T_{arm}\ne0$ fixture 필수** (0 이면 두 축이 같아져 버그가 숨는다).
- `TRACK_ERR` 의 $q_c(now-T_{arm})$ 는 고정 길이 지연 링에서 꺼낸다 — ⚠️ **미구현** (2026-09-24 확인, §4.2 `TRACK_ERR` 행): 현재 코드는 $q_c(now)$ 와 비교한다. 링 길이는 **가장 작은 `dt`** 기준 — `kMaxArmDof × ceil(T_arm_max × control_rate_max)` (`control_rate` 상한 5000 Hz) 로 configure 에서 할당한다. 조회는 tick 수가 아니라 저장된 steady 시각으로 한다.
- 지문 센서 바이어스·잡음 추정은 **EMA** 로 한다(`ContactDebouncer`, `supervisor.contact.baseline_alpha` — 할당 없음, 고정 길이 원형 버퍼가 아니다, C-6·§4.4).
- RT 경로에 try/catch·로깅·deactivate 요청을 두지 않는다 (RT-1~10). FAULT 는 컨트롤러 fault 래치로만 표현한다 (§4.1).
- **상태 출력.** 모드·사유·결과는 controller 소유 `rtc::SeqLock<T>` 스냅샷(trivially copyable POD) + `Setup*Publisher` 패턴의 non-RT publisher 로 낸다. `PublishRole` 에 새 토픽을 추가하지 않는다 (E-11).

## 6. YAML 파라미터

키는 **단일 원천**이다 (S0.3). 파라미터 로딩은 `LoadConfig(YAML)` + `ParseXxxParams`, 런타임 게인만 `declare_parameter`.

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `supervisor.T_sat_guard` | — | – | – | – | v0.5 에서 삭제 — `SAT_NEAR_TC` 삭제 (D-8) |
| `supervisor.gamma.*` | — | – | – | – | v0.5 에서 삭제 — γ 하향 v1 범위 밖 (D-8). `eta_sat`·`max_derates`·`min_interval`·`ramp`·`derate_step` 전부 |
| `supervisor.impact.dp_max` | double | kg·m/s | `TBD` | >0 | §4.7 `TBD-IMP-01` |
| `supervisor.stale_committed_max_s` | double | s | **0.10** (provisional, #537 S7 결정 2026-09-23; S8-B 튜닝 세트의 COMMITTED 스냅샷 age 최대 65 ms < `io.t_stale` 라 조일 근거가 없어 유지, 두 로봇 YAML 에 명시) | ≥0 | §4.2 `[확정 A-6]` |
| `supervisor.n_qp` | int | – | `TBD` | ≥1 | §4.1 `FAULT` 진입 연속 `QP_FAILED` 수 |
| `supervisor.track_err_abort` | double | rad | `TBD` (YAML: ur5e_p1b **0.42** — S8-B sim 피크 0.21 의 2 배; iiwa7_leap 0.3 placeholder) | >0 | **단일 원천.** L5 는 이 키를 참조만 한다. 실기 값은 S10 |
| `supervisor.decel.a_dec` | double | m/s² | **10.0** (provisional — 2026-09-22 사용자 확정, S3.5b gate 지도가 돌린 값; `reference.a_max` 확정 시 ≤ 재검, plan §7.3) | >0, ≤ `reference.a_max` | **단일 원천.** L3 정지거리도 이 키를 읽는다 (§4.3). 두 로봇 `demo_catching_controller.yaml` 에 기록 — 소비자는 S6 계획기의 정지점 예약 (`planner_search.cpp`) 과 S7 DECEL 이다 |
| `supervisor.decel.ramp_time` | double | s | 0.0 | 0–0.1 | §4.3 |
| `supervisor.contact.f_min` | double | N | **0.2** (provisional, 사용자 값) | >0 | G7-3. sim fingertip lane 은 잡음이 없어(C-20) 이 값만 유효하고, `k_sigma` 는 실기 전용이다 |
| `supervisor.contact.k_sigma` | double | – | 3.0 | 2–6 | §4.4. 실기 전용 (sim σ̂≈0) |
| `supervisor.contact.n_debounce` | int | – | 3 | 1–20 | 센서 주기 의존 (실기 250 Hz) |
| `supervisor.contact.m_min` | int | – | **2** (provisional) | 1–4 | 손 형상 |
| `supervisor.contact.T_confirm` | double | s | 0.2 | 0–1 | §4.4 |
| `supervisor.contact.t_stale` | double | s | **0.02** (provisional) | >0 | §4.2 `TIP_STALE` 판정 임계 (실기 250 Hz 발행 기준, D-24) |
| `supervisor.contact.baseline_alpha` | double | – | **0.02** (provisional) | 0–1 | §4.4 바이어스·잡음 EMA 비율 (C-6 — 원형 버퍼가 아니다) |
| `supervisor.contact.n_baseline_min` | int | – | **20** (provisional) | ≥1 | §4.4. 미만이면 결과 `Undetermined` |
| `supervisor.homing.v_max` | double | rad/s | **0.5** (provisional) | >0 | §4.1 homing/retreat 관절공간 사다리꼴 속도 한계 |
| `supervisor.homing.eta_a` | double | – | **0.5** (provisional) | 0–1 | §4.1. 가속 한계 = `qdd_max` × 이 값 |
| `supervisor.homing.qd_tol` | double | rad/s | **0.02** (provisional) | >0 | §4.1 homing/retreat 도착 판정(‖q̇‖∞) |
| `supervisor.ready.pose_tol` | double | rad | 0.02 | – | §4.5 |
| `supervisor.sat_ticks` | int | – | 파서 기본 **60**. YAML: ur5e_p1b **80** (S8-B — 60 이 200 발 중 3 회 발화, 그중 2 회는 잡은 공. 판정을 끈 재측정 max 61 · p99 58 · ≥ 70 0/200), iiwa7_leap 60 (S8-D 재측정). 이력: 5 는 정상 접근의 포화 구간 10–76 tick 을 잘랐고, sim 정상 시행 max 44 · p99 42.5 로 100 에서 50 으로 내렸다. 50 은 재측정에서 1 회 발화해 60 으로 올렸다 | ≥1 | §4.2 `REF_SATURATED` 연속 tick 판정 (D-S7-4, sim 시행 분포로 확인 후 확정) |

`supervisor.ready.wait_pose` 는 두지 않는다 (C-10) — `L3 §6 planner.wait_pose` 와 중복이었다(repo 에 0 hit). `IDLE` homing 목표·`ARMED` 대기 자세는 그 키를 그대로 참조한다.

## 7. 단위 기술 구현 순서

- **L7.1** (S1.8) 상태·사유·결과 enum, 전이 표를 데이터로 고정 + 완전성 검사.
- **L7.2** (S1.8) 가상 감속 대상 + 연속성 테스트.
- **L7.3** (S1.8·S7.3) 접촉 판정기(debounce, 부호 재확인) + 합성 잡음 오경보 테스트.
- **L7.4** v0.5 에서 삭제 — γ 하향 경로 v1 범위 밖 (D-8).
- **L7.5** (S7.2) 슈퍼바이저 본체 + IDLE homing + QP 비의존 abort 경로 연결 + 시나리오 테스트(§9).
- **L7.6** (S8 — G7-B3 충격량 상관과 함께 이월, #537 결정 2026-09-24) 충격량 예산(§4.7): 시뮬레이션 접촉 참값(S3.3)으로 $\Delta t_{imp}$, $\bar F$, 관절 토크 산출 → `TBD-IMP-01` 확정 → L3 게이트 연결.
- **L7.7** (S10) 실기 전용 조건 연결: speed scaling, 시계 (신호 출처 확보 후), 지문 센서 stale.
- **L7.8** (S5.1 임시 → S9) E-STOP·fault 훅: P-1 임시 기준 (S5.1, `[CONCERN] E-8`) → D-13 정책 (S9).

## 8. 디버깅 방법

- 타임라인 그래프: 모드 띠, $t_c$·$t_{cmd}$ 수직선, $\Vert e\Vert$, 포화 플래그, 손 $\rho(t)$, 센서 $f_i$와 임계.
- 예상치 못한 `RETREAT`: 전이 로그의 사유 코드로 추적한다.
- 포획했는데 `Missed`: 판정 창과 센서 수신 시각 정렬(실기 async 센서 지연), 부호 규약(§4.4)을 확인한다.
- 감속 중 흔들림: `a_dec` 값과 L5 가속 한계의 정합, 램프 적용 여부를 확인한다.
- `REF_SATURATED` 가 자주 발생: L3 rollout의 여유율(`eta_a`, η_v)이 낮거나 $T_w$ 가 짧은지 확인한다. 빈도는 S8 에서 D-8 재검토 입력으로 기록한다.
- `ARMED` 에 안 들어감: homing 목표 `wait_pose` 와 `pose_tol`, 손 `q_pre` 도달, `PARAMS_TBD` 를 확인한다.
- 접촉 직후 `TRACK_ERR` abort: 충격 구간에서 임계를 완화했는지 확인한다(§4.7-3).

## 9. 검증 방법과 합격 게이트

시나리오 테스트(모의 입력으로 RT 슈퍼바이저 단독 실행, 기대 상태열 비교, $T_{arm}\ne0$):

| 시나리오 | 기대 상태열 |
|---|---|
| 정상 | Idle(homing) → Armed → Tracking → Approach → Committed → Closing → Decel → Hold → Retreat → Armed, `Captured` |
| 활성화 자세가 대기 자세 밖 | Idle(homing) → Armed (교착 없음) |
| 공 놓침 | … → Decel → Hold → Retreat, `Missed` |
| 동결 전 stale | … → Approach → Retreat, `BallStale` |
| 동결 후 짧은 stale | … → Committed → Closing → …, `BallStaleCommitted` 기록 |
| 동결 후 긴 stale | … → Committed → AbortSafe → Retreat, `BallStaleLong` |
| catchability 탈락 | Tracking 유지, `NoCatchablePlan` 기록 |
| 동결 전 포화 | … → Approach → Retreat, `RefSaturated` |
| 동결 후 포화 | … → Committed → AbortSafe, `RefSaturated` |
| QP 실패 | 임의 상태 → AbortSafe(QP 비의존 감속), `QpFailed`. 연속 $N_{qp}$회 → Fault |
| 관절 경계 충돌 | 임의 상태 → AbortSafe(QP 비의존 감속), `JointConflict` |
| E-STOP 발동·해제 | 임의 상태 → (CM hold) → Idle, 자동 재개 없음, $q_c$·앵커 = $q_{meas}$ |
| fault 리셋 | Fault → Idle (`ResetFault`) |
| TBD 파라미터 | Idle 유지, `ParamsTbd` |

| 게이트 | 기준 | 태그 |
|---|---|---|
| G7-A | 위 시나리오 전부 기대 상태열과 일치, 전이표 완전성 검사 통과 | `[SIM-ANY]` |
| G7-B | 감속 전환 시 기준 상태 $(x,\dot x)$ 연속 (< 1e-9) | `[SIM-ANY]` |
| G7-B2 | v0.5 에서 삭제 — γ 하향 v1 범위 밖 (D-8) | – |
| G7-B3 | 충격량 $\Delta p$ 기록과 시뮬레이션 접촉 참값의 최대 접촉력 상관 확인, 손가락 관절 토크가 한계 이내 (한계 권위 출처 D-12 확정 전까지 토크 비교 부분은 `NOT_EVALUATED`) | `[SIM-P1B]` |
| G7-C | 합성 잡음에서 접촉 오경보율 기록 (임계는 사용자 결정) | `[SIM-ANY]` |
| G7-D | RT 할당 0 (`ScopedNoMalloc`·`ScopedAllocGate`), 틱 최악 실행시간 기록 | `[SIM-ANY]` |
| G7-E | `ur5e_p1b` 시뮬레이션 폐루프에서 결과 판정과 MuJoCo 참값 일치율 기록 (부호 규약 재확인 포함) | `[SIM-P1B]` |
| G7-F | 실기 speed scaling·시계·센서 stale 경로 동작 확인 (S10, 신호 출처 확보 후) | `[HW-P1B]` |
| G7-G | 관절 fresh + 지문 센서 dropout negative control 에서 `TIP_STALE` 또는 결과 `Undetermined` 발화, 옛 힘을 새 접촉으로 판정 0 (D-24) | `[SIM-ANY]` |
| G7-H | E-8 최소 계약(S5.1): (a) deactivate → 다른 컨트롤러가 팔 이동 → 재activate 첫 tick 이 옛 자세를 명령하지 않음, (b) trigger·clear·deactivate race 에서 reset writer 가 RT tick 하나, (c) 자동 재개 0, (d) `ClearEstop` 후 latched fault 유지 | `[SIM-ANY]` |

## 10. 미확정 항목

TBD-HAND-03(잡음), TBD-IMP-01(§4.7), `supervisor.stale_committed_max_s`(0.10, S8-B 로 유지·YAML 명시), `supervisor.n_qp`, `supervisor.decel.a_dec`, `supervisor.contact.*`(값은 provisional 로 닫힘, S8 튜닝), `supervisor.impact.dp_max`, `supervisor.sat_ticks`(S8-B: p1b 80, iiwa7_leap 60 — S8-D 재측정), QP 비의존 감속 식(S5.3), E-STOP·fault 정책(S9, D-13). homing 은 `IDLE` 하위 단계로 닫혔다(S1.8 헤더 해석, §4.1). S10 이월: TBD-ARM-03(speed scaling), TBD-NET-01(PTP).
