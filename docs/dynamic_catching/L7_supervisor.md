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
| `IDLE` | 활성화 직후, E-STOP 해제 후, fault 리셋 후 | 파라미터 검증 전: 현재 자세 유지. 검증 통과 후: **homing** — 복귀 기준(L4 §5.3)으로 `wait_pose` 로 이동, 손 `Open` | `wait_pose` 허용오차 안 + 손 `Open` 완료 + §4.5 조건 → `ARMED` |
| `ARMED` | 준비 완료 | 대기 자세 유지(L4 §5.3), 손 `Open` | 유효 궤적 수신(형식 검사 통과 + not stale + 지평 여유) → `TRACKING` / §4.5 조건 위반 → `IDLE` |
| `TRACKING` | 유효 궤적 수신 | 기준 유지, plan 대기 | 유효 plan → `APPROACH` / 트랙 epoch·`generation` 변경·stale → `ARMED` / plan 없음(`NO_CATCHABLE_PLAN`) → 머무름, 기록 |
| `APPROACH` | 유효 plan | L4 추종, plan 교체 허용 | $t_c-now\le T_{freeze}$ → `COMMITTED` / 실패 조건 → `RETREAT` |
| `COMMITTED` | 동결 | plan 교체 금지, $now\ge t_c-T_{pre}$ 에서 손 `Preshape` | $now\ge t_{cmd}$ → `CLOSING` / 치명 조건 → `ABORT_SAFE` |
| `CLOSING` | 폐쇄 명령 | 손 `Close` | $now_{lead}\ge t_c$ → `DECEL` / 치명 조건 → `ABORT_SAFE` |
| `DECEL` | $now_{lead}\ge t_c$ | 가상 감속 대상 추종(§4.3), 접촉 판정 | 감속 대상 정지($\tau\ge\tau_s$) → `HOLD` / 치명 조건 → `ABORT_SAFE` |
| `HOLD` | 정지 | 유지, 결과 판정 확정 | $T_{hold}$ 경과 → `RETREAT` |
| `RETREAT` | 종료·실패 | 복귀 기준(L4 §5.3), 결과에 따라 손 유지/개방 | 복귀 완료 → `ARMED` |
| `ABORT_SAFE` | 치명 조건(상태 무관) | 즉시 감속 후 정지. L5 가 정상이면 §4.3 감속 대상을 L4→L5 로, **`QP_FAILED`·`JOINT_CONFLICT` 이면 QP 비의존 관절공간 감속**(아래) | 정지 → `RETREAT` / 재차 치명 조건 → `FAULT` |
| `FAULT` | `QP_FAILED` 연속 $N_{qp}$회, 또는 `ABORT_SAFE` 중 재차 치명 조건 | QP 비의존 관절공간 감속으로 정지 후 $q_c$ 고정, 컨트롤러 fault 래치 (`HasLatchedFault()` true). **RT 에서 deactivate 를 요청하지 않는다** | `/rtc_cm/reset_fault` → `ResetFault()` → `IDLE` (P-1 reseed) |

**전이는 (상태 × 사유) 표를 데이터로 둔다 `[확정 S1.8]`.** 위 표와 §4.2 표는 사람이 읽는 형태이고, 코드는 둘을 합친 표 하나를 단일 출처로 삼는다. 기동 시 완전성을 검사한다 — 모든 상태에 진입·이탈이 최소 1개씩 있고, 모든 `Reason` 이 최소 한 칸에서 쓰이며, 미정의 칸이 없어야 한다(G7-A).

**IDLE homing `[확정 S7.2]`.** v0.4 는 `IDLE` 에서 현재 자세만 유지하면서 `ARMED` 진입 조건에 "대기 자세 허용오차 안"(§4.5-4)을 걸어, 활성화 자세가 `wait_pose` 밖이면 영원히 `ARMED` 에 못 가는 교착이 있었다. 파라미터 검증 통과 후 `wait_pose` 로 이동하는 homing 단계를 둔다. homing 을 `IDLE` 의 하위 단계로 둘지 별도 `Mode` 로 둘지는 S1.8 전이표 구현에서 정한다.

`ABORT_SAFE` 의 진입 조건은 "상태 무관"이다. `ARMED` 에서 준비 조건이 깨지면 `IDLE` 로 내려가 조건이 회복되기를 기다린다.

**QP 비의존 관절공간 감속 `[확정 S5.3]`.** `ABORT_SAFE` 의 기본 경로(§4.3 감속 대상 → L4 → L5)는 L5 QP 가 정상일 때만 성립한다. 원인이 `QP_FAILED`·`JOINT_CONFLICT` 이면 실패한 L5 에 의존할 수 없으므로, 직전 명령 $q_c$·$\dot q_c$ 에서 관절별 가속 한계(D-16 도출값) 이내로 $\dot q_c\to0$ 까지 감속하는 관절공간 경로를 쓴다 (할당·QP 없음, `dt` 기준 적분, 관절 위치 한계 clamp). 정확한 식은 S5.3 에서 확정한다.

**감속은 시각 기준으로 시작한다 `[확정 A-5]`.** 실기 접촉 신호가 지문 센서뿐이라, 공이 손바닥에 먼저 닿으면 손가락이 닫히기 전까지 검출이 늦을 수 있다. 따라서 `DECEL` 진입은 $now_{lead}\ge t_c$ 로 하고, 지문 센서는 결과 판정과 abort에만 쓴다.

**E-STOP·fault 임시 기준 `[확정 P-1]` (S9 전까지).**
- E-STOP 중 출력은 CM 이 `BuildHoldOutput` 으로 대체한다. 슈퍼바이저는 출력을 만들지 않고 **상태만 정리**한다 (`TriggerEstop` 에서 진행 중 시행을 `Aborted` 로 종결, plan·손 시퀀스 무효화).
- `ClearEstop` 후 **자동 재개 금지** — `IDLE` 로 가고, $q_c$ 와 CLIK 앵커를 $q_{meas}$ 로 reseed 한다. 다음 시행은 homing 부터 다시 한다.
- fault 는 E-STOP 과 분리된 컨트롤러 래치다. `ClearEstop` 은 fault 를 풀지 않고 `ResetFault` 는 E-STOP 을 풀지 않는다. RT 경로의 try/catch·deactivate 는 쓰지 않는다 (RT-2).
- 손 자세 유지로 인한 파지력 소실 등 부작용 검토와 정책 전체는 S9 (D-13).

**S5.1 최소 E-STOP 계약 (plan §4.4 S5.1, `[CONCERN] E-8` — S5 착수 전 승인 필요).** 이 계약은 위 P-1 임시 기준을 구현 수준에서 좁힌 것이다.
- (a) `TriggerEstop`·`ClearEstop`·`ResetFault`·`ResetTargetInitialization` 훅은 **atomic 요청·epoch 만 갱신**한다. reset 자체(값을 되돌리는 동작)의 **유일한 writer 는 RT tick** 이다 — 훅이 직접 상태를 되돌리지 않는다.
- (b) plan·궤적·공분산·손·FSM·타이머 무효화는 D-23 순서(activation generation 판정)로 RT tick 이 수행한다.
- (c) 해제 후 자동 재개는 하지 않는다.
- (d) `ClearEstop` 은 컨트롤러 fault 래치를 풀지 않는다 — E-STOP 경로와 fault 경로는 별개다(위 P-1 규칙과 동일).
- 이 계약은 팔 명령 경로·CLIK 앵커가 아직 없는 S4.0 에는 적용되지 않는다(S4.0 은 base 기본 동작과 CM 의 hold 방어선에 맡긴다) — E-8 대상은 그것들이 생기는 **S5.1** 부터다. 전체 물리 정책(D-13)은 S9.

### 4.2 abort·실패 사유

| 코드 | 조건 | 발생 가능 상태 | 처리 |
|---|---|---|---|
| `BALL_STALE` | L1 stale (나이 초과 또는 지평 소진) | `TRACKING`, `APPROACH` | `TRACKING` 이면 `ARMED`, `APPROACH` 면 `RETREAT` |
| `BALL_STALE_COMMITTED` | L1 stale | `COMMITTED`, `CLOSING` | 계속 진행 (동결 plan으로 포구 시도), 기록 `[확정 A-6]` |
| `BALL_STALE_LONG` | stale 지속 > `supervisor.stale_committed_max_s` | `COMMITTED`, `CLOSING` | `ABORT_SAFE` `[확정 A-6]` |
| `TRACK_CHANGED` | L1 트랙 변경 판정 (L1 §4.4 — 트랙 epoch. `generation` 을 어떻게 쓰는지는 L1 이 정한다, D-4) | `TRACKING`, `APPROACH` | `ARMED`/`RETREAT`. `PointCloud2`에 트랙 상태가 없어 `STATUS_LOST`를 이것으로 대체 |
| `HORIZON_EXTRAP` | L2 `after_horizon=true` (지평 **뒤**로 외삽, $now_{lead}$ 기준) | 전 구간 | `APPROACH`면 `RETREAT`, 동결 후면 기록 후 계속 |
| `PRED_INCONSISTENT` | L1 예측 일관성 지표 $\bar\nu$ 가 임계 초과 (L1 §4.5) | `TRACKING`, `APPROACH` | `RETREAT` (`TRACKING` 이면 `ARMED`). 동결 후에는 기록만 |
| `NO_CATCHABLE_PLAN` | 계획기가 plan 없음을 게시 (catchability manipulability 미달 D-18, IK 실패, 도달 불가 — 세부 사유는 L3 plan 사유 코드) | `TRACKING` | 비치명. `TRACKING` 유지, 기록 |
| `PLAN_INVALID` | plan 무효 | `APPROACH` | `RETREAT` |
| `QP_FAILED` | L5 QP 실패 status | 전 구간 | `ABORT_SAFE` (QP 비의존 경로, §4.1). 연속 $N_{qp}$회면 `FAULT` |
| `REF_SATURATED` | L4 포화 (D-8: γ 하향 없음) | `APPROACH`, `COMMITTED`, `CLOSING` | `APPROACH` 면 `RETREAT`, 동결 후면 `ABORT_SAFE` `[확정 D-8]`. 포화 판정 방식(연속 tick 수 등)은 S7 에서 확정 |
| `GAMMA_DERATED` | v0.5 에서 삭제 — γ 하향은 v1 범위 밖 (D-8, §4.6) | – | – |
| `SAT_NEAR_TC` | v0.5 에서 삭제 — `REF_SATURATED` 로 대체 (D-8) | – | – |
| `JOINT_CONFLICT` | L5 `bound_conflict` | 전 구간 | `ABORT_SAFE` (QP 비의존 경로) |
| `TRACK_ERR` | $\Vert q-q_c(now-T_{arm})\Vert>$ 임계 | 전 구간 | `ABORT_SAFE` |
| `ABORT_ESCALATED` | `ABORT_SAFE` 중 재차 치명 조건 | `ABORT_SAFE` | `FAULT` |
| `ESTOP` | E-STOP 발동·해제 (§4.1 P-1, S5.1 최소 계약) | 전 구간 | 발동: 상태 정리, 해제: `IDLE`. 단 `FAULT` 에서는 `FAULT` 유지 — 해제가 fault 래치를 풀지 않는다 (P-1, S5.1(d)) |
| `FAULT_RESET` | `ResetFault` | `FAULT` | `IDLE` |
| `SPEED_SCALING` | speed scaling ≠ 1 | 전 구간 | `ABORT_SAFE`. **repo 에 신호 출처 없음 → sim 비활성, S10** |
| `CLOCK_UNHEALTHY` | PTP 임계 초과 | 전 구간 | `IDLE`·`ARMED`에서는 진입 거부, 운행 중 `ABORT_SAFE`. **신호 출처 없음 → sim 비활성, S10** |
| `PARAMS_TBD` | L0 검증 실패 (활성 구성 TBD·provisional) | `IDLE` | 진입 거부 |
| `HAND_TIMEOUT` | L6 폐쇄 타임아웃 | `CLOSING`, `DECEL` | 기록, 계속 |
| `TIP_STALE` | 지문 센서 stale | `COMMITTED` 이후 | 판정 불가로 기록 |

**`TIP_STALE` 판정 경로는 D-24 (a) 다 (2026-09-22 사용자 확정, 배선은 S5.2e).** 지문 센서 lane 에는 수신 시각도 sequence 도 없었다 — RT backend 3종 중 관절 상태 콜백만 `last_state_ns_` (backend watchdog stamp) 를 갱신하고, 지문 센서 자체의 freshness 는 관측할 수 없다. 관절이 fresh 한 채 센서만 멈추면 옛 힘을 새 접촉으로 오판할 수 있다. 채택한 경로는 (a) `rtc_base` `DeviceState` 센서 lane 에 `recv_steady_ns`·`sequence`·`valid` 를 추가하고 backend 3종이 채워 `ControllerState` 로 전달 (PROC-3, P5 — grasp 에도 같은 gap 이라 함께 닫힌다); (b) 포구 컨트롤러 소유 mailbox 는 device 경로와 공존하는 중복 lane 이라 기각했다 (plan §7.3). `TIP_STALE` 은 그 `recv_steady_ns` 의 수신 나이로 판정한다.

`BALL_STALE_COMMITTED` 는 A-6 으로 확정됐다. 동결 후에는 짧은 누락으로 포기하는 것보다 동결 plan으로 진행하는 편이 안전하다고 본다. stale 지속 시간 상한 `supervisor.stale_committed_max_s` 의 초기 제안값 **vision 발행 주기의 3배**(예: 30 Hz 발행이면 0.1 s)는 물리적 유도가 없는 제안일 뿐이다 — 어느 물리량(공분산 성장, 포획 반경 오차 할당, abort 정지거리 $\Vert\dot x\Vert^2/(2a_{dec})$)으로 이 값을 조일지는 **S7 착수 시** 정한다(plan §7.3).

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

**부호 규약.** sim 과 실기 두 경로 모두 **finger-on-object** 부호다 — 실기 P1b `HandSensorState` (250 Hz) 와 같게 `rtc_mujoco_sim` 이 커밋 0fcc1d23 부터 fingertip-on-environment 로 발행한다 (코드 확인. repo 규약상 sim 쪽 부호 스위치를 다시 넣지 않는다. 부호 변환 지점은 `rtc::grasp::PullContactConfig::force_sign` 한 곳). `rtc_msgs` 메시지 주석의 "sim 은 반대 부호" 서술은 stale 이다. 따라서 접촉 판정은 힘의 크기·법선 성분을 **그대로** 쓰고 별도 정규화를 두지 않는다. S7.3 착수 시 부호를 다시 확인한다. 센서 주기·스탬프는 입력단에서 기록한다 (판정 시각은 수신 steady 시각, `header.stamp` 는 staleness 판단에 쓰지 않는다).

센서 $i$의 바이어스 $b_i$는 **`ARMED`/`TRACKING` 구간(손 `Open` 정지 중)** 의 이동평균으로 추정한다. 잡음 표준편차 $\hat\sigma_i$도 같은 창에서 구한다. v0.2는 창을 `COMMITTED` 구간으로 잡았는데 그 길이가 $T_{freeze}-T_{close,tot}\approx T_{arm}+T_{margin}$(수십 ms)뿐이라, 센서 주기 100 Hz면 표본 2–5개로 $\hat\sigma_i$ 를 추정하게 된다. 창은 `RETREAT → ARMED` 에서 비운다(§4.8).

$$f_i(t)=\Vert F_i(t)-b_i\Vert,\qquad c_i(t)=\mathbb 1\big[f_i>\max(f_{\min},\,k_\sigma\hat\sigma_i)\big]$$

$N_{deb}$개 연속 샘플이 참이면 센서 $i$ 접촉으로 확정한다 (debounce 는 순수 조각, S1.8).

결과 판정(`HOLD` 종료 시):
- **포획:** 판정 창 $[t_{cmd},\,t_c+T_{conf}]$ (실제 시각 $now$ 로 비교 — 공의 물리 시각과 같은 축, plan §3) 안에 접촉 센서 수가 $m_{\min}$ 이상이었고, `HOLD` 종료 시점에도 $m_{\min}$ 이상이 유지되는 경우.
- **실패:** 판정 창 안에 접촉이 없는 경우.
- **미확정:** 센서 stale.

오경보 확률은 센서 잡음 분포에 의존한다. $k_\sigma$는 시뮬레이션·실기 잡음 측정 후 정한다(가우시안 가정이면 $k_\sigma=3$에서 단측 약 0.13%, 등급 a).

### 4.5 준비(ARMED) 조건

1. L0 파라미터 검증 통과(TBD 없음)
2. 시계 건강(실기) — 신호 출처 없음, sim 비활성, S10
3. L5 `lead_enable=true`이면 `T_arm` 확정
4. 로봇이 대기 자세 허용오차 안에 있음 (`IDLE` homing 으로 도달, §4.1)
5. speed scaling = 1(실기) — 신호 출처 없음, sim 비활성, S10
6. 손 `Open` 완료

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


### 4.8 재무장 리셋 목록 `[권장]`

`RETREAT → ARMED` 전이에서 **다음을 전부 초기화한다.** 하나라도 빠지면 직전 시행의 상태가 남아 두 번째 투척이 다르게 동작한다. v0.2는 이 목록이 없었고, §9 시나리오가 전부 단발 시행이라 게이트에서도 잡히지 않았다. 아래는 소유 layer 별로 정리한 **단일 표**다.

| 대상 | 소유 layer | 리셋 내용 | 빠뜨렸을 때 |
|---|---|---|---|
| `PlanSnapshot` 박스 (valid flag 포함) | S6 계획기 출력 / RT 소비 | `valid=false` 로 무효화하고 `last_consumed_plan_id` 기록 | 옛 plan 의 $t_c$ 가 이미 과거라 `TRACKING→…→DECEL` 을 몇 틱에 통과하며 엉뚱한 곳에서 손을 닫는다 |
| 계획기 후보 hysteresis·이전 최선 후보 | S6 계획기 (스레드 내부) | 이전 시행의 최선 후보·hysteresis 상태를 지운다 | 새 시행의 첫 후보가 직전 시행의 후보와 비교돼 갱신이 지연되거나 억제된다 |
| 계획기 공분산 버퍼 | S6 계획기 (A-3, D-22 token) | 버퍼·token 무효화 | 직전 시행의 옛 공분산이 새 궤적과 짝지어져(N/N−1 혼합) 게이트를 오판정한다 |
| 계획기 wake 잔여 신호 (eventfd) | S6 계획기 스레드 (D-7c) | drain 하여 카운터를 0 으로 | 재무장 직후 첫 tick 이 옛 신호로 깨어나 있지도 않은 스냅샷을 소비한 것처럼 판정될 수 있다 |
| L4 soft-catch 기준 생성기 | L4 | 측정 자세·속도 0 으로 리셋 — $p_c$ 와 γ 프로파일까지 (L4 §5.3) | 직전 포구점으로 복귀하고, 하향된 γ가 남는다 |
| L2 hint 커서 | L2 (샘플러) | 0 | 정확성은 이진 탐색이 지키지만 틱 비용이 흔들린다 |
| L5 (확장 CLIK) 직전 $\dot q$ 상태·앵커 | L5 | $\dot q$ 0, 앵커·$q_c$ 를 현재 명령 자세로 (L5 리셋 규약) | 첫 틱 가속 경계가 옛 속도 기준이라 `bound_conflict` 오abort |
| L6 시퀀서 | L6 | `Open` 위상, 진행률 창 | 폐쇄 명령 시각이 어긋난다 |
| L7 접촉 바이어스·잡음 창 | L7 | 원형 버퍼 비우기 | 직전 시행의 접촉력이 바이어스에 섞인다 |
| L7 접촉 debounce 카운터 | L7 | $N_{deb}$ 연속 카운터를 0 으로 | 직전 시행 종료 시점의 연속 참 카운트가 남아 새 시행 초반에 한두 샘플만으로 접촉 확정된다 |
| L7 stale 타이머 | L7 | `BALL_STALE_COMMITTED` 지속 시간 0 | 직전 시행의 stale 누적으로 `BALL_STALE_LONG` 오abort |
| L7 `QP_FAILED` 연속 카운터 | L7 | 0 | 직전 시행 실패가 누적돼 `FAULT` 로 조기 진입 |
| L7 결과·사유 | L7 | `Outcome::None`, `Reason::None` | 진단 오염 |

**완전성 규칙.** 전이표 완전성 검사(§4.1 G7-A)와 같은 방식으로, **모든 stateful 멤버는 이 표에 있거나 명시적으로 면제되어야 한다** — 새 stateful 멤버를 추가하면서 이 표를 갱신하지 않는 것을 금지한다 (S7.4 게이트, G8-A2).

그리고 L7이 plan 을 받아들일 때 **세 조건을 모두** 본다: `valid`, `plan_id != last_consumed_plan_id`, $t_c > now + T_{lead,min}$ ($t_c$ 는 절대 steady 시각, D-2). 마지막 조건이 과거 plan 을 걸러낸다.

§9 시나리오에 **"연속 2회 투척"** 과 **"abort 직후 재투척"** 을 넣어야 이 항목들이 게이트에서 검증된다.

## 5. C++ 구현

v0.4 의 입출력 구조체·슈퍼바이저 클래스 스케치는 삭제했다 — repo 에 없는 tick API 와 아직 정해지지 않은 타입에 기대고 있었다. 입출력 구조체의 모양은 L4·L5 의 S1·S2 타입이 정해진 뒤 **S7 에서 확정**한다. 여기서는 확정된 것만 적는다.

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
- 전이 시 고정 크기 레코드 `{now, from, to, reason, plan_id, generation}`를 SPSC(`rtc::SpscQueue`)에 넣는다 (L8 기록). 트랙 식별은 vision 의 `generation`(uint64, L1 §4.4, D-4)이다.
- **시각 비교는 plan §3 (D-2) 의 타입으로만 한다.** 내부 시각은 절대 steady ns 이고, 상대시각은 수치 코어 경계에서만 만든다. `BallTime`·`NowReal`·`NowLead` 를 서로 다른 타입으로 두어 혼용을 막는다 — 판정별 비교 대상은 §4.1 표와 plan §3 표가 같다. 매 tick 의 $now$ 는 steady 실측이며 tick 수 × `dt` 로 계산하지 않는다. **$T_{arm}\ne0$ fixture 필수** (0 이면 두 축이 같아져 버그가 숨는다).
- `TRACK_ERR` 의 $q_c(now-T_{arm})$ 는 고정 길이 지연 링에서 꺼낸다. 링 길이는 **가장 작은 `dt`** 기준 — `kMaxArmDof × ceil(T_arm_max × control_rate_max)` (`control_rate` 상한 5000 Hz) 로 configure 에서 할당한다. 조회는 tick 수가 아니라 저장된 steady 시각으로 한다.
- 지문 센서 바이어스·잡음 추정은 고정 길이 원형 버퍼로 한다(할당 없음).
- RT 경로에 try/catch·로깅·deactivate 요청을 두지 않는다 (RT-1~10). FAULT 는 컨트롤러 fault 래치로만 표현한다 (§4.1).
- **상태 출력.** 모드·사유·결과는 controller 소유 `rtc::SeqLock<T>` 스냅샷(trivially copyable POD) + `Setup*Publisher` 패턴의 non-RT publisher 로 낸다. `PublishRole` 에 새 토픽을 추가하지 않는다 (E-11).

## 6. YAML 파라미터

키는 **단일 원천**이다 (S0.3). 파라미터 로딩은 `LoadConfig(YAML)` + `ParseXxxParams`, 런타임 게인만 `declare_parameter`.

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `supervisor.T_sat_guard` | — | – | – | – | v0.5 에서 삭제 — `SAT_NEAR_TC` 삭제 (D-8) |
| `supervisor.gamma.*` | — | – | – | – | v0.5 에서 삭제 — γ 하향 v1 범위 밖 (D-8). `eta_sat`·`max_derates`·`min_interval`·`ramp`·`derate_step` 전부 |
| `supervisor.impact.dp_max` | double | kg·m/s | `TBD` | >0 | §4.7 `TBD-IMP-01` |
| `supervisor.stale_committed_max_s` | double | s | `TBD` (제안: vision 발행 주기 × 3, 물리적 유도 없음) | ≥0 | §4.2 `[확정 A-6]`. 조일 물리량은 S7 착수 시 결정 (plan §7.3), S8 조정 |
| `supervisor.n_qp` | int | – | `TBD` | ≥1 | §4.1 `FAULT` 진입 연속 `QP_FAILED` 수 |
| `supervisor.track_err_abort` | double | rad | `TBD` | >0 | **단일 원천.** L5 는 이 키를 참조만 한다 |
| `supervisor.decel.a_dec` | double | m/s² | **10.0** (provisional — 2026-09-22 사용자 확정, S3.5b gate 지도가 돌린 값; `reference.a_max` 확정 시 ≤ 재검, plan §7.3) | >0, ≤ `reference.a_max` | **단일 원천.** L3 정지거리도 이 키를 읽는다 (§4.3). 두 로봇 `demo_catching_controller.yaml` 에 기록 — 파서는 읽지만 소비자 (S6 정지점 예약·S7 DECEL) 는 아직 없다 |
| `supervisor.decel.ramp_time` | double | s | 0.0 | 0–0.1 | §4.3 |
| `supervisor.contact.f_min` | double | N | `TBD` | >0 | G7-3 |
| `supervisor.contact.k_sigma` | double | – | 3.0 | 2–6 | §4.4 |
| `supervisor.contact.n_debounce` | int | – | 3 | 1–20 | 센서 주기 의존 (실기 250 Hz) |
| `supervisor.contact.m_min` | int | – | `TBD` | 1–4 | 손 형상 |
| `supervisor.contact.T_confirm` | double | s | 0.2 | 0–1 | §4.4 |
| `supervisor.ready.pose_tol` | double | rad | 0.02 | – | §4.5 |
| `supervisor.ready.wait_pose` | double[n] | rad | `TBD` | – | 로봇별. `IDLE` homing 목표 (§4.1). plan §11 의 catchability IK 시작 자세와 같은 값 |

## 7. 단위 기술 구현 순서

- **L7.1** (S1.8) 상태·사유·결과 enum, 전이 표를 데이터로 고정 + 완전성 검사.
- **L7.2** (S1.8) 가상 감속 대상 + 연속성 테스트.
- **L7.3** (S1.8·S7.3) 접촉 판정기(debounce, 부호 재확인) + 합성 잡음 오경보 테스트.
- **L7.4** v0.5 에서 삭제 — γ 하향 경로 v1 범위 밖 (D-8).
- **L7.5** (S7.2) 슈퍼바이저 본체 + IDLE homing + QP 비의존 abort 경로 연결 + 시나리오 테스트(§9).
- **L7.6** (S7.3) 충격량 예산(§4.7): 시뮬레이션 접촉 참값(S3.3)으로 $\Delta t_{imp}$, $\bar F$, 관절 토크 산출 → `TBD-IMP-01` 확정 → L3 게이트 연결.
- **L7.7** (S10) 실기 전용 조건 연결: speed scaling, 시계 (신호 출처 확보 후), 지문 센서 stale.
- **L7.8** (S5.1 임시 → S9) E-STOP·fault 훅: P-1 임시 기준 (S5.1, `[CONCERN] E-8`) → D-13 정책 (S9).

## 8. 디버깅 방법

- 타임라인 그래프: 모드 띠, $t_c$·$t_{cmd}$ 수직선, $\Vert e\Vert$, 포화 플래그, 손 $\rho(t)$, 센서 $f_i$와 임계.
- 예상치 못한 `RETREAT`: 전이 로그의 사유 코드로 추적한다.
- 포획했는데 `Missed`: 판정 창과 센서 수신 시각 정렬(실기 async 센서 지연), 부호 규약(§4.4)을 확인한다.
- 감속 중 흔들림: `a_dec` 값과 L5 가속 한계의 정합, 램프 적용 여부를 확인한다.
- `REF_SATURATED` 가 자주 발생: L3 rollout의 여유율(`eta_a`, η_v)이 낮거나 $T_w$ 가 짧은지 확인한다. 빈도는 S8 에서 D-8 재검토 입력으로 기록한다.
- `ARMED` 에 안 들어감: homing 목표 `wait_pose` 와 `pose_tol`, 손 `Open` 완료, `PARAMS_TBD` 를 확인한다.
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

TBD-HAND-03(잡음), TBD-IMP-01(§4.7), `supervisor.stale_committed_max_s`(값·조일 물리량 모두 S7 착수 시), `supervisor.n_qp`, `supervisor.decel.a_dec`, `supervisor.contact.*`, `supervisor.impact.dp_max`, homing 을 `IDLE` 하위 단계로 둘지 별도 `Mode` 로 둘지(S1.8), `REF_SATURATED` 판정 방식(S7), QP 비의존 감속 식(S5.3), E-STOP·fault 정책(S9, D-13). S10 이월: TBD-ARM-03(speed scaling), TBD-NET-01(PTP).
