# L7 — Supervisor: 상태 머신, 접촉 판정, 감속, abort

이 문서는 현재 구현의 L7 — 포구 임무의 상태 머신, 접촉 판정, 포구 후 감속, abort, E-STOP · fault 정책 — 을 표현한다. planner 는 `closed_form` 과 `mpc` 둘이고 (`supervisor.decel.mode`, 출하 YAML 은 `mpc`), 상태 머신과 전이표는 두 planner 에 공통이며 팔의 법칙만 갈린다. 갈리는 곳은 §4.3 (`closed_form`) 과 §4.3a (`mpc`) 에 나란히 적고, 다른 절에서는 어느 planner 의 서술인지를 밝힌다.

- 배치 `[확정 D-1]`: 순수 조각 — 전이표 데이터 (`transition_table.hpp`), 감속 대상과 관절공간 정지 (`decel_target.hpp`), 관절공간 homing (`joint_home.hpp`), 접촉 debounce (`contact_debounce.hpp`) — 은 `rtc_controllers/include/rtc_controllers/catching/` (namespace `rtc::catching`) 에 있고, FSM 드라이버는 포구 컨트롤러 (`integrated_bringup/src/controllers/catching/controller.cpp`, `RTControllerInterface::Compute` 안) 다. YAML 은 `integrated_bringup` 바인딩. 별도 패키지는 없다

---

## 1. 범위 / 비범위

범위:
- RT 루프(`RTControllerInterface::Compute`)에서 포구 임무의 모드를 결정한다.
- plan 수락·동결, 손 시퀀서 구동, 포구 후 감속, 결과 판정(포획·실패), 안전 abort와 복귀를 맡는다.
- E-STOP · fault 에 대한 컨트롤러 쪽 정책 (§4.1).

비범위:
- 계획 계산(L3) — `mpc` 에서는 `APPROACH` 부터 정지까지의 관절 구간도 계획기가 만든다 (§4.3a)
- 기준 생성 수식(L4) — `closed_form` 에서만 돈다
- 관절 한계 처리(L5)
- E-STOP 시 출력 대체 — CM 이 `ValidateControllerOutput` 실패 시 `BuildHoldOutput`, E-STOP (과 해제 검증 창) 동안 `BuildLatchedHoldOutput` 으로 대체한다. 슈퍼바이저는 상태 정리만 한다 (§4.1)
- 실기 쪽 E-STOP (드라이브 hold 반응, 실기 E-stop · 보호정지와 소프트웨어 latch 의 관계, 해제 뒤 드라이버 재개 절차 — D-S9-G)
- UR 드라이버 자체의 보호 정지(드라이버·로봇 제어기 소관)

## 2. 코드 확인 게이트

| ID | 확인 항목 |
|---|---|
| G7-1 | RTC 프레임워크의 기존 상태 머신·모드 전환 규약, 컨트롤러 활성/비활성 시 명령 유지 방식 |
| G7-2 | UR 드라이버의 speed scaling 상태 인터페이스 이름과 의미 ([R11]) |
| G7-3 | 지문 센서 잡음 수준, 주기, 스탬프 (시뮬레이션·실기) |
| G7-4 | PTP 동기 상태 확인 방법 |

G7-2 · G7-4 의 신호 (speed scaling, 시계 건강) 는 repo 에 출처가 없다 — 그래서 §4.2 의 `SPEED_SCALING` · `CLOCK_UNHEALTHY` 는 전이 행만 있고 발화하지 않는다 (TBD-ARM-03 · TBD-NET-01, §4.5, G7-F). G7-3 의 잡음 수준은 TBD-HAND-03 이다 (§4.4).

## 3. 참고자료

[R1] 포구 직후 충격과 순응의 역할, [R3] 인터셉트 후 선형 감속(원 논문의 임시방편, 본 layer에서 정식화 — 원문 확인: "the velocity of the robot is linearly reduced during the post-interception period (0.3 s)"), [R8] 임계값 설계, [R16] 충격량 분배·강성 최적화, [R18] 충격 순간 기준 궤적 처리(reference spreading), [R19] 접선 컴플라이언스.

## 4. 수학적 이론

### 4.1 상태 머신

시각 표기는 plan §3 (D-2) 을 따른다: $t_c$, $t_{cmd}$ 는 공의 **물리 시각**(`BallTime`, 절대 steady ns), $now$ 는 매 tick steady 실측(`NowReal`), $now_{lead}=now+T_{arm}$ (`NowLead`). 비교는 타입별 오버로드로만 한다.

| 상태 | 진입 조건 | 주요 동작 | 이탈 |
|---|---|---|---|
| `IDLE` | 활성화 직후, E-STOP 해제 후, fault 리셋 후 | 파라미터 검증 전: 현재 자세 유지(비무장, latch). 검증 통과 + **무장** 후: **homing** — 관절공간 법칙(`joint_home.hpp`, QP/CLIK 비의존)으로 `wait_pose` 로 이동(팔이 이미 `pose_tol` 안이면 생략), 손 `q_open`(homing 중에만, §4.5) | `wait_pose` 허용오차 안 + 손 `q_pre` 도달 + §4.5 조건 → `ARMED` |
| `ARMED` | 준비 완료 | 대기 자세 유지(관절공간, homing 과 같은 법칙), 손 `q_pre` | 유효 궤적 수신(형식 검사 통과 + not stale + 지평 여유) → `TRACKING` / §4.5 조건 위반 → `IDLE` |
| `TRACKING` | 유효 궤적 수신 | 기준 유지, plan 대기, 손 `q_pre` | 유효 plan → `APPROACH` (`mode: mpc` 에서는 plan 과 그 첫 구간을 같은 tick 에 함께 채택할 때만, §4.3a) / 트랙 epoch·`generation` 변경·stale → `ARMED` / plan 없음(`NO_CATCHABLE_PLAN`) → 머무름, 기록 |
| `APPROACH` | 유효 plan | 손 `q_pre`. `closed_form`: L4 추종 (soft-catch 기준 → CLIK), plan 교체 허용. `mpc`: 계획기의 구간 추종 (§4.3a), plan 교체 없음 | $t_c-now\le T_{freeze}$ → `COMMITTED` (R-ADMIT, §4.1) / 실패 조건 → `RETREAT` / `mode: mpc` 에서 따를 구간이 없으면 `ParamsTbd` → `ABORT_SAFE` |
| `COMMITTED` | 동결 | plan 교체 금지, 손은 계속 `q_pre` (시각 기반 preshape·`T_pre` 는 쓰지 않는다 — `q_open` 은 homing 전용이고 대기 중 손은 항상 `q_pre` 다). 팔의 법칙은 `APPROACH` 와 같다 | $now\ge t_{cmd}$ (R-CLOSE, `HandCommandDueRounded`) → `CLOSING` / 치명 조건 → `ABORT_SAFE` |
| `CLOSING` | 폐쇄 명령 | 손 `Close`. 팔의 법칙은 `APPROACH` 와 같다 | $now_{lead}\ge t_c$ → `DECEL` / 치명 조건 → `ABORT_SAFE` (`mode: mpc` 에서 따를 구간이 없으면 `ParamsTbd`, §4.3a) |
| `DECEL` | $now_{lead}\ge t_c$ | 접촉 판정. `closed_form`: 가상 감속 대상 추종 (§4.3). `mpc`: `APPROACH` 부터 따르던 구간의 연속 (§4.3a) | `closed_form`: 감속 대상 정지($\tau\ge\tau_s$) / `mpc`: 구간의 마지막 노드를 지남 → `HOLD` / 치명 조건 → `ABORT_SAFE` |
| `HOLD` | 정지 | 유지 (`closed_form`: 정지한 감속 대상, `mpc`: 구간의 정지 상태 — 새 구간은 받지 않는다), 결과 판정 확정 | $T_{hold}$ 경과 → `RETREAT` |
| `RETREAT` | 종료·실패 | 정지 램프(`JointSpaceDecelStep`, ABORT_SAFE 경유 시 no-op) + 관절공간 `wait_pose` 복귀. 손은 복귀 내내 그대로이고 대기 자세에서 연다 (§4.8 "RETREAT 순서") | `wait_pose` 도착(이미 안이면 즉시) + 손 `q_pre` 도달 → `ResetForRearm` → `ARMED` / 정지·복귀 단계가 운동 기한 초과, 또는 fault latch → `FAULT` (`ABORT_ESCALATED`, D-S9-D1) |
| `ABORT_SAFE` | 치명 조건(상태 무관) | **관절공간 정지** — 원인과 무관하게 직전 명령에서 $\dot q_c\to0$ 까지 관절별 가속 한계로 감속한다 (`JointSpaceDecelStep`, 아래 "`ABORT_SAFE` 의 정지"). L4 · L5 를 거치지 않는다 | 정지 → `RETREAT` / fault latch 또는 정지 기한 초과 → `FAULT` (`ABORT_ESCALATED`) |
| `FAULT` | CLIK 실패로 끝난 **시행**이 연속 $N_{qp}$ 회 (D-S9-D2), 또는 운동 기한 초과 — `ABORT_SAFE` 정지 램프·`RETREAT` 정지·`RETREAT` 복귀 (D-S9-D1) | `ABORT_SAFE` 와 같은 관절공간 감속으로 정지 후 $q_c$ 고정, 컨트롤러 fault 래치 (`HasLatchedFault()` true), 비무장. **RT 에서 deactivate 를 요청하지 않는다** | `/rtc_cm/reset_fault` → `ResetFault()` → 팔 정지 판정 통과 (D-S9-D3) → `IDLE` (P-1 reseed) |

**전이는 (상태 × 사유) 표를 데이터로 둔다.** 위 표와 §4.2 표는 사람이 읽는 형태이고, 코드는 둘을 합친 표 하나 (`kTransitionTable`, 95 행 — `transition_table.hpp`) 를 단일 출처로 삼는다. 드라이버 (`AdvanceMode`) 는 매 tick (현재 모드, 그 tick 의 사유) 를 `LookupTransition` 으로 찾아 행이 있으면 전이한다. 행이 없는 (모드, 사유) 는 그 모드에서 해당 없음이다 — 모드는 그대로이고 사유만 기록된다. 표의 완전성 — 모든 상태에 진입·이탈이 최소 1개씩 있고, 모든 `Reason` 이 최소 한 행에서 쓰이며, 같은 (상태, 사유) 가 서로 다른 목적지로 두 번 나오지 않는다 — 은 `CheckTransitionTableComplete` 가 판정하고, **단위 테스트가 출하 표의 완전성을 고정한다** (`rtc_controllers/test/test_catching_supervisor_core.cpp` 의 `TransitionTable.ShippedTableIsComplete`, G7-A). configure · 기동 경로는 이 함수를 부르지 않는다.

표가 정한 해석 세 가지 (근거는 `transition_table.hpp` 머리 주석):

- `Reason::kNone` 은 각 상태의 **정상 전진** 의 키다 ("실패 사유 없음 + 다음 상태의 진입 조건 성립"). 언제 `kNone` 으로 표를 찾을지는 드라이버가 정한다.
- homing 은 별도 `Mode` 가 아니라 `IDLE` 의 하위 단계다. `IDLE → ARMED` 는 homing 과 §4.5 가 모두 성립할 때 한 번 난다.
- 준비 조건 상실에는 전용 사유가 없어 `kParamsTbd` 를 재사용한다 (§4.5).

**IDLE homing.** 파라미터 검증 통과 + 무장 뒤 `wait_pose` 로 이동하는 단계가 `IDLE` 안에 있다. `ARMED` 진입 조건이 "대기 자세 허용오차 안" (§4.5-4) 이므로, 이 단계가 없으면 활성화 자세가 `wait_pose` 밖일 때 `ARMED` 에 가지 못한다. `planner.wait_pose_source: current` 면 homing 목표는 YAML 이 아니라 activation 뒤 팔이 **정지** 한 첫 판독 가능 tick 에 채택한 $q_{meas}$ 다 — 팔이 이미 거기 있으므로 homing 은 no-op 으로 끝나고, 재무장 (`RETREAT → ARMED`) 은 그 자세로 돌아온다. 채택은 activation 마다 한 번이고 E-STOP · fault 리셋은 채택한 자세를 유지한다. E-STOP 아래의 tick, 팔이 아직 움직이거나 속도 lane 을 읽을 수 없는 채 무장 요청이 온 경우, margined 관절 상자 밖 · 상자 없음은 거부이고 YAML 자세가 남는다 (사유는 publish 스레드의 WARN 한 줄; 키와 규칙의 전문은 L3 §6).

**homing 은 관절공간이다 (C-13).** per-joint 사다리꼴 (v ≤ `supervisor.homing.v_max`, a ≤ `qdd_max`·`supervisor.homing.eta_a`), QP/CLIK 비의존이다 (`joint_home.hpp` — task-space soft-catch DS 기반 복귀는 쓰지 않는다, L4 §5.3). 도달 판정은 `supervisor.ready.pose_tol` ∧ ‖q̇‖∞ ≤ `supervisor.homing.qd_tol`. **homing 은 운동이므로 무장 latch 를 요구한다** (P-1 (e): 활성화는 무장이 아니다) — 비무장 `IDLE` 은 활성화 자세를 그대로 유지한다. **팔이 이미 `pose_tol` 안이면 homing 을 생략**하고 손만 `q_pre` 로 지시한다.

**R-IDLE.** `IDLE` tick 에서 carried `arm_qd_cmd_ ≠ 0` (RETREAT 복귀 중 disarm → IDLE, 또는 homing 중 disarm) 이면 `JointSpaceDecelStep` 으로 정지까지 램프한 뒤 유지한다. `{RETREAT, kParamsTbd} → IDLE` 행이 성립하는 근거가 이것이다 — **`IDLE` 이 그 램프를 소유한다** (§4.5 아래 문단).

`ABORT_SAFE` 의 진입 조건은 "상태 무관"이다. `ARMED` 에서 준비 조건이 깨지면 `IDLE` 로 내려가 조건이 회복되기를 기다린다.

**`ABORT_SAFE` 의 정지는 항상 관절공간이다 (C-35).** `ABORT_SAFE` 는 원인과 무관하게 — CLIK 이 실패한 경우 (`QP_FAILED`·`JOINT_CONFLICT`) 든, L5 가 정상인 경우 (`TRACK_ERR`·`BALL_STALE_LONG`·`REF_SATURATED`·준비 상실·`mpc` 의 "따를 구간 없음") 든 — 정지 경로가 하나다 (`RunArmMotion` 의 `case Mode::kAbortSafe:` → `RunJointSpaceAbort` → `JointSpaceDecelStep`). 직전 명령 $(q_c,\dot q_c)$ 에서 관절마다

$$\dot q_{c,i}\leftarrow\operatorname{sign}(\dot q_{c,i})\,\max\big(\vert\dot q_{c,i}\vert-\ddot q_{\max,i}\,\Delta t,\ 0\big),\qquad q_{c,i}\leftarrow\operatorname{clamp}\big(q_{c,i}+\dot q_{c,i}\,\Delta t,\ q_{\min,i},\ q_{\max,i}\big)$$

를 매 tick 적용한다. $\ddot q_{\max}$ 는 팔의 관절별 가속 한계 (`robot.arm.qdd_max`, D-16 도출값 — homing 의 `eta_a` 축소는 걸지 않는다), 위치 상자는 CLIK 에 준 것과 같은 margin 적용 상자이고, 상자에 닿은 관절은 그 자리에서 $\dot q_{c,i}=0$ 이 된다. 할당 · QP · 모델이 없다. §4.3 의 task-space 감속 대상은 `closed_form` 의 `DECEL` · `HOLD` 에만 쓰이고 abort 에는 쓰이지 않는다. 한 경로로 두는 이유: 원인이 CLIK 이면 실패한 층을 거쳐 정지할 수 없고, 그 밖의 원인에서도 task-space 정지는 이득이 분명하지 않은 채 QP 의존을 늘린다. 한 tick 에 $\dot q_c=0$ 으로 두지 않는 이유는 그것이 무한 감속이기 때문이다 (position 인터페이스에서 드라이브의 보호 정지를 부른다). `FAULT` 도 같은 경로로 정지한다.

`ABORT_SAFE` 는 감속이 **끝날 때까지** 머문다 (명령 속도가 전부 0). 시작한 tick 에 나가면 ABORT_SAFE 는 상태가 아니라 이름표가 되고, 다음 시행이 팔이 아직 움직이는 중에 시작한다. 래치된 fault 가 있으면 대신 `ABORT_ESCALATED` 로 FAULT 에 간다 — RETREAT 는 방금 실패한 것을 다시 시도하는 쪽으로 되돌리기 때문이다. `RETREAT` 진입은 plan 무효화 지점이다 (§4.8). E-STOP 리셋은 **모드를 건드리지 않는다**: 전이표가 `{FAULT, ESTOP} → FAULT` 를 갖고 있으므로 reset 이 IDLE 을 강제하면 P-1 (d) 가 지키려는 래치를 지운다.

**감속은 시각 기준으로 시작한다 `[확정 A-5]`.** 실기 접촉 신호가 지문 센서뿐이라, 공이 손바닥에 먼저 닿으면 손가락이 닫히기 전까지 검출이 늦을 수 있다. 따라서 `DECEL` 진입은 $now_{lead}\ge t_c$ 로 하고, 지문 센서는 결과 판정과 abort에만 쓴다.

**E-STOP·fault 정책 `[확정 D-13]`.**
- **출력은 CM 이 정한다.** E-STOP latch 가 서 있는 동안 (그리고 해제 뒤 검증 창 동안) CM 이 컨트롤러 출력 전체를 버리고 모든 device 를 **정지 tick 에 latch 한 측정 위치**로 쓴다 (`BuildLatchedHoldOutput` — 그 tick 에 읽을 수 없는 채널은 마지막 판독값). 슈퍼바이저는 출력에 관여할 수 없고 **상태만 정리**한다. 그래도 컨트롤러 자신의 출력도 같은 자세다 — 정지 tick 의 리셋이 팔·손 hold latch 를 그 tick 의 측정값으로 다시 잡으므로, CM 치환이 없어도 정지가 운동이 되지 않는다.
- **손도 측정 자세로 hold (D-S9-A).** 열지도 조이지도 않는다. `HOLD`·`RETREAT` 중이면 닫힌 자세 그대로 멈추는데, position servo 는 명령과 측정의 간극으로 힘을 내므로 **간극 0 이 되어 공을 놓는다** — 확정 동작이다. "측정 자세" 는 **정지 tick 의 측정**이다 — CM 이 hold 목표를 그 tick 에 한 번 잡고 해제 (검증 통과) 까지 유지한다. 해제 뒤에도 손 latch 는 측정 자세로 재시드되어 재무장 전까지 그 자세다. 해제는 검증 창 (`watchdog_check_divisor_ + 2` tick) 동안 hold 를 유지한 뒤에만 컨트롤러 출력을 내보낸다 — 창 동안 이 컨트롤러는 이미 E-STOP 이 풀린 상태로 돌고 CSV `estop_active` 도 0 이다.
- **단계와 무관하게 한 가지 반응 (D-S9-B).** 어느 모드든 정지 tick 에 `IDLE` (사유 `ESTOP`)·비무장, plan·손 시퀀스 무효화. `FAULT` 만 예외로 유지한다 ({FAULT, ESTOP} → FAULT). 진행 중 시행은 `Aborted` 로 끝나 정지 동안 그 값을 싣고, 해제의 리셋이 `None` 으로 되돌린다 (다음 시행은 판정 없이 시작). `HOLD` 끝에서 이미 판정된 시행은 진행 중이 아니다 — 판정은 `RETREAT` 진입 tick 에 발행되고 (§4.4 결과 판정, 시행 러너가 읽는 곳), 복귀 중 E-STOP 이나 `ABORT_SAFE` 재진입은 그것을 `Aborted` 로 덮지 않는다. 감속이 필요한 조건은 E-STOP 이 아니라 컨트롤러 소유 (`ABORT_SAFE`·`FAULT`) 다.
- **해제 뒤 자동 재개 금지 (D-S9-C, P-1 (c)).** `IDLE`·비무장으로 남고, $q_c$ 와 CLIK 앵커를 $q_{meas}$ 로 reseed 한다. 운용자가 `catching.enable` 로 재무장하면 homing 부터 다시 한다. 채택된 `wait_pose` 는 유지하고 정지 자세를 새로 채택하지 않는다.
- **fault 는 별개 래치 (D-S9-E1, P-1 (d)).** `ClearEstop` 은 fault 를 풀지 않고 `ResetFault` 는 E-STOP 을 풀지 않으며, `FAULT` 를 global E-STOP 으로 승격하지 않는다. RT 경로의 try/catch·deactivate 는 쓰지 않는다 (RT-2).
- **fault 의 원인 (D-S9-D1·D2).** 둘 뿐이고, 어느 쪽이든 전이는 `ABORT_ESCALATED` → `FAULT` 다. ① **운동 기한** — `ABORT_SAFE` 의 정지 램프와 `RETREAT` 정지 단계 (명령 정지 + 측정 팔이 `track_err_abort` 안으로 따라잡음) 는 `supervisor.deadline.stop_s`, `RETREAT` 복귀 단계는 `supervisor.deadline.return_s` 안에 끝나야 한다. 시계는 각 단계의 진입 tick 에 새로 시작한다 (`ABORT_SAFE` ↔ `RETREAT` 순환마다 다시 — 순환 카운터는 두지 않는다, D-S9-I). 손을 기다리는 release 단계에는 이 기한이 걸리지 않는다 (`robot.hand.T_release_timeout` 이 따로 잰다). 판정은 escalation 계층 (R-PREC 의 두 번째) 이라 준비 상실 (disarm) 보다 앞선다 — 기한이 이미 지났으면 disarm tick 에도 `FAULT` 다. 기한 **전**의 disarm 은 `RETREAT → IDLE` 이고 `IDLE` 의 정지 램프 (R-IDLE) 에는 기한이 없다: `IDLE` 은 비무장·명령 정지라 새 운동을 명령하지 않고, 기한을 `IDLE` 까지 잇는 것은 전이표 변경 (E-8) 이다. `ABORT_SAFE` 에서는 abort 를 일으킨 `track_err_` 를 읽지 않는다 (램프 완료 `abort_stopped_` 만 본다). ② **`n_qp` 시행 연속** — CLIK 실패 (`QP_FAILED`·`JOINT_CONFLICT`) 로 끝난 시행마다 +1, `HOLD` 판정 (Captured·Missed·Undetermined) 에 이른 시행이 0 으로, 다른 사유의 abort 와 E-STOP 은 값을 바꾸지 않는다. 0 으로 되돌리는 것은 그 판정·fault reset·activation 뿐이다 (세는 것은 solve 가 아니라 시행이다 — 좋은 solve 뒤에 실패한 시행도 센다). latch 가 `RETREAT` 중에 서면 `FAULT` 로 가서 복귀를 멈추고 명령을 정지까지 램프한다. 원인은 CSV 열 `fault_cause` (latch 동안 매 tick) 와 publish 스레드의 WARN 한 줄이 남긴다. latch 는 **서는 tick 에** 비무장한다 — `n_qp` latch 는 법칙 tick 안, 그 tick 의 리셋 처리 뒤에 서므로, 다음 tick 에야 내리면 그 사이에 들어온 reset 이 무장이 살아 있는 채 latch 를 풀어 abort 가 `RETREAT` → `ARMED` → 새 시행으로 이어진다.
- **팔을 읽을 수 없을 때 (D-S9-K).** 시행 중에 팔 상태가 게이트를 통과하지 못하게 되면 (폭 부족·빈 자리 — 멈춘 stale 과 다르고 CM watchdog 이 잡지 않는다) `RETREAT` 는 정지 단계에서 복귀로 넘어가지 않고, 복귀 중이면 명령을 정지까지 램프한 뒤 유지한다 (얼리지 않는다 — latch 뒤 명령은 계속 나가므로 한 tick 속도 계단이 된다). 기한은 계속 가서 넘으면 `FAULT`, 기한 안에 다시 읽히면 정지된 명령에서 복귀를 이어 간다.
- **fault reset 은 팔이 정지해야 받는다 (D-S9-D3).** reset 요청 tick 에 ① carried 명령 속도가 0, ② 팔 속도 lane 이 판독 가능 (`IsLaneReadable`; 아니면 거부 — fail-closed), ③ 측정 |q̇| ≤ `supervisor.homing.qd_tol`. 하나라도 아니면 latch 가 남고 그 tick 의 CSV `fault_reset_refused` (1 명령 램프 중 · 2 측정 속도 · 3 lane 판독 불가) 와 WARN 한 줄로 알린다. 거부는 대기열에 쌓이지 않는다 — 운용자가 다시 부른다.
- **E-STOP 중 reset (D-S9-L).** 정지 판정을 통과하면 latch 는 그 tick 에 내려가지만 모드는 E-STOP 이 풀릴 때까지 `FAULT` 로 보이고 (ESTOP 최우선, R-PREC), 해제 tick 에 `FAULT_RESET` 으로 `IDLE`·비무장이 된다. 이 구간의 "`FAULT` 표시 + `fault_latched` 0" 은 "reset 은 받았고 해제를 기다린다" 는 뜻이다. `ABORT_SAFE` 에서 latch 가 선 tick 과 escalation tick 사이에 E-STOP 이 오면 `IDLE` 에 latch 만 남는다 — 그 상태의 reset 은 `IDLE` 에 머물고 사유 `FAULT_RESET` 을 한 tick 싣는다.
- **운용 절차.** ① E-STOP 해제 (`/rtc_cm/clear_estop` — 빈 `reason_ack` 로 한 번 호출해 거부 메시지에서 사유를 읽고, 확인 뒤 그 사유로 다시 호출; GUI 헤더의 "Clear E-STOP" 이 이 2 단계다) → ② fault 가 있으면 reset (`/rtc_cm/reset_fault`, 활성 컨트롤러의 `Name()`; GUI "Reset fault") → ③ 손에 남은 공 제거 → ④ `catching.enable` 로 재무장. `catching_diag` 플롯은 두 latch 구간을 색을 달리해 음영으로 보인다.
- **실기에서 정해야 하는 것 (D-S9-G).** 드라이브의 hold 반응, 실기 E-stop·보호정지와 소프트웨어 latch 의 관계, 해제 뒤 드라이버 재개 절차, 그리고 두 운동 기한의 실기 값 (`supervisor.deadline.provisional` 이 `true` 인 동안 실기 구성은 park, §6).

**최소 E-STOP 계약 (P-1, `[CONCERN] E-8` 승인).** 위 정책을 구현 수준에서 좁힌 것이고, 컨트롤러 헤더 (`demo_catching_controller.hpp` 머리 주석) 가 같은 계약을 갖는다.
- (a) `TriggerEstop`·`ClearEstop`·`ResetFault`·`ResetTargetInitialization` 훅은 **atomic 요청·epoch 만 갱신**한다. reset 자체(값을 되돌리는 동작)의 **유일한 writer 는 RT tick** 이다 — 훅이 직접 상태를 되돌리지 않는다.
- (b) plan·궤적·공분산·손·FSM·타이머 무효화는 D-23 순서(activation generation 판정)로 RT tick 이 수행한다.
- (c) 해제 후 자동 재개는 하지 않는다.
- (d) `ClearEstop` 은 컨트롤러 fault 래치를 풀지 않는다 — E-STOP 경로와 fault 경로는 별개다.
- (e) **무장은 명시적 행위다.** 운용자 채널은 컨트롤러 노드의 파라미터 `catching.enable` (기본 false) 이고, 파라미터 콜백은 atomic 만 갱신한다. **RT tick 이 E-STOP 발동·해제 양쪽과 fault 래치에서 이 latch 를 내린다** — 그래서 (c) 의 "자동 재개 금지" 가 운용 규율이 아니라 메커니즘이다. 활성화(`on_activate`) 도 무장이 아니다: 활성화가 무장이면 E-STOP 복구가 지나가는 deactivate→activate 사이클이 곧 재개가 된다
- 요청은 **flag 가 아니라 epoch** 이다. 두 tick 사이에 발동→해제가 모두 끝나면 flag 는 false 로 돌아와 있어 tick 이 "아무 일도 없었다" 로 읽고 $q_c$ 를 정지 너머로 이어가기 때문이다. tick 은 "지금 켜져 있는가" 가 아니라 "내가 마지막으로 처리한 값에서 움직였는가" 를 묻는다

**driver 규칙.** FSM driver(`Compute` 안의 매 tick 진행)가 지켜야 할 규칙을 전이표와 별도로 둔다.

- **R-PREC (사유 우선순위).** 한 tick 에 사유는 하나이므로 순서를 고정한다: `ESTOP` > fault reset/escalation > 준비 상실(`kParamsTbd`) > 법칙 실패·치명(`kQpFailed`/`kJointConflict`/`kTrackErr`/`kBallStaleLong`/`kRefSaturated`) > 시간 전진(`kNone` — commit/close/decel 정지/T_hold 경과/복귀 완료) > 기록 전용(`kBallStaleCommitted`·`kHorizonExtrap`·`kTipStale`·`kHandTimeout`). 기록 전용 사유는 **전진이 없는 tick 에만**, 그리고 그 모드에 §4.2 행이 있을 때만 낸다(행이 없는 모드는 플래그로만 기록).
- **R-ORDER (분기 위치, C-33).** `IDLE`(homing)·`DECEL`·`HOLD`·`RETREAT` 는 vision lane **앞**(`ABORT_SAFE` 와 같은 자리)에서 판정한다. `COMMITTED`/`CLOSING` 은 vision 판정과 무관하게 항상 추종 법칙을 돌리고 (`closed_form`: soft-catch 기준, `mpc`: 구간), 샘플러는 나이와 무관하게 마지막 스냅샷을 쓰며 지평 밖은 외삽으로 계속한다 — `kBallStaleCommitted`/`kHorizonExtrap` 은 기록만 한다.
- **R-WATCHDOG.** homing·retreat 법칙도 `track_err_` 를 계산해 `supervisor.track_err_abort` 를 본다. `RETREAT` 는 `{kRetreat, kTrackErr} → ABORT_SAFE` 행을 쓰되 **복귀 단계에서만** 판정한다 — 정지 단계에서는 같은 임계를 "측정 팔이 정지한 명령을 따라잡았는가" 로만 읽어 복귀의 시작을 미루고 (abort 하지 않는다, §4.8 "RETREAT 순서"), release 단계에는 판정이 없다. `IDLE` 은 행이 없으므로 초과 시 homing 을 멈추고 `arm_requested_` 를 내려(disarm) 사유를 기록한다 — 남은 명령 속도는 R-IDLE 이 정지까지 램프한다.
- **R-DECEL-ENTRY.** tick 의 `now` 는 `Compute` 머리에서 **한 번** 읽어 법칙·시퀀서·판정에 같은 값을 넘긴다 (추종 법칙이 따로 시계를 읽지 않는다; plan · 구간의 접수 나이만 box 를 읽은 뒤의 시각으로 잰다). `DECEL` 진입 tick 의 처리는 planner 마다 다르다. **`closed_form`**: 기준 생성기의 현재 상태 (직전 tick 의 `Step` 출력, 곧 이 tick 의 기준) 를 §4.3 의 $(x_s,\dot x_s)$ 로, 이 tick 의 $now_{lead}$ 를 $t_s$ 로 삼고, 같은 tick 에 $\tau=0$ 의 감속 step 을 돈다 (생성기는 reset 하지 않고 대상만 바꾼다). 다음 tick 부터 $\tau=now_{lead}-t_s$ 로 소비한다. **`mpc`**: 진입 상태를 잡지 않는다 — $t_c$ 에 따르던 구간을 다음 샘플로 그대로 이어 따른다 (§4.3a).
- **R-ADMIT (C-31, L3 §4.11).** `JudgePlan` 의 조건 (g) 는 $t_c-now\le T_{freeze}$ 인 plan 을 `kTooLate` 로 거부한다 — 그런 plan 은 채택 tick 에 곧바로 commit 되고 ($t_c\le now$ 면 다음 tick 에 감속해) 접근한 적 없는 포구에 묶이기 때문이다. 검증기는 $T_{freeze}\ge T_{close,e2e}+T_{arm}+h$ ($h$ 는 제어 주기 하나) 를 강제한다. 두 planner 에 공통이다 (`mpc` 는 여기에 첫 구간의 판정이 더해진다, §4.3a).
- **R-CLOSE.** `COMMITTED→CLOSING` 은 시퀀서의 `close_issued`(규칙 `now ≥ t_cmd-h/2`, `HandCommandDueRounded(now, t_cmd, h)`)로 전이한다. **기록 1 tick 지연**: `Compute()` 순서상 닫힘 명령이 나간 tick 의 기록에는 아직 `COMMITTED` 가 실리고 `CLOSING` 은 다음 tick 부터 기록된다 — 손 명령 시각 자체 (G6-A) 에는 영향이 없다. 오프라인 분석은 닫힘 시각을 mode 열이 아니라 손 명령 열에서 읽는다.
- **R-TRACK.** 동결 후 샘플은 commit 시점 `committed_generation_` 에 고정한다 — 다른 generation 스냅샷은 stale 로 보고 동결 plan 으로 계속하며 `kBallStaleCommitted` 만 기록한다. 재무장 뒤에는 `last_trial_generation_` 을 기록해 그 generation 은 usable 로 보지 않는다(새 generation 이 올 때까지 대기). 두 멤버 모두 재무장 리셋이 쓰고 `ResetTrialState` 가 지운다(§4.8).

### 4.2 abort·실패 사유

| 코드 | 조건 | 발생 가능 상태 | 처리 |
|---|---|---|---|
| `BALL_STALE` | L1 stale (나이 초과 또는 지평 소진) | `TRACKING`, `APPROACH` | `TRACKING` 이면 `ARMED`, `APPROACH` 면 `RETREAT` |
| `BALL_STALE_COMMITTED` | L1 stale | `COMMITTED`, `CLOSING` | 계속 진행 (동결 plan으로 포구 시도), 기록 `[확정 A-6]` |
| `BALL_STALE_LONG` | stale 지속 > `supervisor.stale_committed_max_s` | `COMMITTED`, `CLOSING` | `ABORT_SAFE` `[확정 A-6]` |
| `TRACK_CHANGED` | L1 트랙 변경 판정 (L1 §4.4 — 트랙 epoch. `generation` 을 어떻게 쓰는지는 L1 이 정한다, D-4) | `TRACKING`, `APPROACH` | `ARMED`/`RETREAT`. `PointCloud2`에 트랙 상태가 없어 `STATUS_LOST`를 이것으로 대체 |
| `HORIZON_EXTRAP` | L2 `after_horizon=true` (지평 **뒤**로 외삽, $now_{lead}$ 기준) | 전 구간 | `APPROACH`면 `RETREAT`, 동결 후면 기록 후 계속 |
| `PRED_INCONSISTENT` | L1 예측 일관성 지표 $\bar\nu$ 가 임계 초과 (L1 §4.5) | `TRACKING`, `APPROACH` | `RETREAT` (`TRACKING` 이면 `ARMED`). 동결 후에는 기록만. **발화하지 않는다**: $\bar\nu$ 생산자가 없다 (L1 §6). 전이표 행 (`kPredInconsistent`) 은 남고 완전성 검사 대상이지만 어떤 tick 도 이 사유를 내지 않는다 |
| `NO_CATCHABLE_PLAN` | 이 tick 에 채택할 plan 이 없다 — 계획기가 plan 없음을 게시했거나 (catchability manipulability 미달 D-18, IK 실패, 도달 불가 — 세부 사유는 L3 plan 사유 코드), RT 의 접수 판정 (`JudgePlan`) 이 box 의 plan 을 거부했거나, `mpc` 에서 그 plan 의 첫 구간이 판정을 통과하지 못했다 (§4.3a) | `TRACKING` | 비치명. `TRACKING` 유지, 기록 |
| `PLAN_INVALID` | plan 무효 | `APPROACH` | `RETREAT` |
| `QP_FAILED` | L5 QP 실패 status | 전 구간 | `ABORT_SAFE`. 이 실패로 끝난 **시행**이 연속 $N_{qp}$ 회면 `FAULT` (D-S9-D2 — solve 단위가 아니다, §4.1) |
| `REF_SATURATED` | **`closed_form` 의 사유다.** L4 `ref.saturated` 가 연속 `supervisor.sat_ticks` tick, 또는 기준 생성기가 그 tick 에 유효한 기준을 내지 못함. 세는 것은 공을 추종하는 tick (`APPROACH`·`COMMITTED`·`CLOSING`) 의 연속 포화이고, 포화가 아닌 tick 과 감속 대상을 추종하는 tick 이 0 으로 되돌린다. `mpc` 는 soft-catch 기준을 돌리지 않으므로 **발화하지 않는다** (§4.3a) | `APPROACH`, `COMMITTED`, `CLOSING` | `APPROACH` 면 `RETREAT`, 동결 후면 `ABORT_SAFE` `[확정 D-8]` |
| `JOINT_CONFLICT` | L5 `bound_conflict` | 전 구간 | `ABORT_SAFE` |
| `TRACK_ERR` | $\Vert q_{meas}-q_c(now)\Vert_2>$ `supervisor.track_err_abort` — 측정과 **같은 tick 의 명령** 의 차 (팔 관절 전체). 명령을 움직이는 tick (추종 법칙, homing, `RETREAT` 정지·복귀) 에서만 계산한다. 지연 링으로 $q_c(now-T_{arm})$ 와 비교하는 형태는 구현하지 않았다 — 선행을 켜면 명령이 측정보다 $T_{arm}$ 앞서므로 그만큼의 오차가 이 값에 들어 있다 | 전 구간 | `ABORT_SAFE` |
| `ABORT_ESCALATED` | fault latch (`n_qp` 시행 연속, D-S9-D2) 또는 운동 기한 초과 (D-S9-D1) — 원인은 CSV `fault_cause` | `ABORT_SAFE`, `RETREAT` | `FAULT` |
| `ESTOP` | E-STOP 발동·해제 (§4.1 P-1) | 전 구간 | 발동: 상태 정리, 해제: `IDLE`. 단 `FAULT` 에서는 `FAULT` 유지 — 해제가 fault 래치를 풀지 않는다 (P-1 (d)) |
| `FAULT_RESET` | `ResetFault` | `FAULT` | `IDLE` |
| `SPEED_SCALING` | speed scaling ≠ 1 | 전 구간 | `ABORT_SAFE`. **전이 행은 있으나 발화하는 코드가 없다** — repo 에 신호 출처가 없다 (TBD-ARM-03). 실기 단계가 신호를 연결한다 (G7-F) |
| `CLOCK_UNHEALTHY` | PTP 임계 초과 | 전 구간 | `IDLE`·`ARMED`에서는 진입 거부, 운행 중 `ABORT_SAFE`. **전이 행은 있으나 발화하는 코드가 없다** — 신호 출처가 없다 (TBD-NET-01, G7-F) |
| `PARAMS_TBD` | L0 검증 실패 (활성 구성 TBD·provisional) | `IDLE` | 진입 거부. 같은 사유를 준비 조건 상실 (§4.5) 과 "법칙에 필요한 값이 없음" (`mpc` 에서 따를 구간이 없음, §4.3a) 이 재사용한다 — 그때의 처리는 그 절이 적는다 |
| `HAND_TIMEOUT` | L6 폐쇄 타임아웃 (`CLOSING`·`DECEL`) · **`RETREAT` 의 release 뒤 손이 `robot.hand.T_release_timeout` 안에 `q_pre` 에 정착하지 못함** (D-S8-6 (a)) | `CLOSING`, `DECEL`, `RETREAT` | `CLOSING`·`DECEL`: 기록, 계속. `RETREAT`: **`IDLE` + 같은 tick disarm** — 정착 못 한 손이 스스로 재무장하지 않게 하고, 운전자가 다시 무장한다 (P-1 (c)). 시계는 복귀 도착 tick (`kReturn → kRelease`) 에 시작하고, 같은 tick 에 정착했으면 재무장이 이긴다. `IDLE` 에는 행이 없다 — 손을 기다리지 않는다 |
| `TIP_STALE` | 지문 센서 stale | `COMMITTED` 이후 | 판정 불가로 기록 |

**`TIP_STALE` 은 지문 센서 lane 자신의 수신 나이로 판정한다 (D-24).** `rtc_base` `DeviceState` 의 센서 lane 이 group 마다 수신 steady 시각과 sequence 를 싣고 (backend 가 채운다), 그 수신 나이가 `supervisor.contact.t_stale` 을 넘거나 backend 가 그 group 을 유효하지 않다고 표시하면 stale 이다 (`rtc::IsSensorGroupFresh`). 관절 상태의 freshness 는 지문 센서의 멈춤을 말해 주지 않는다 — 관절이 fresh 한 채 센서만 멈추면 옛 힘을 새 접촉으로 오판한다 (G7-G).

`BALL_STALE_COMMITTED` 는 A-6 이다. 동결 후에는 짧은 누락으로 포기하는 것보다 동결 plan으로 진행하는 편이 안전하다고 본다. `BALL_STALE_LONG` 의 시계는 따르는 트랙의 마지막 스냅샷의 수신 나이다 — 나이가 `io.t_stale` + `supervisor.stale_committed_max_s` 를 넘으면 발화한다. 이 상한을 어느 물리량 (공분산 성장, 포획 반경 오차 할당, abort 정지거리) 으로 조일지는 실기 데이터로 정한다.

**사유 우선순위·발화 위치 규칙(R-PREC·R-ORDER)은 §4.1 "driver 규칙" 을 본다** — 위 표는 어떤 상태에서 어느 사유가 나올 수 있는지만 정의하고, 한 tick 에 여럿이 동시에 성립할 때 어느 것을 내는지는 그 규칙이 정한다.

### 4.3 가상 감속 대상 — `closed_form` 의 `DECEL` `[논문 외 유도]`

`supervisor.decel.mode: closed_form` 에서 `DECEL` · `HOLD` 의 팔 기준이다. `mpc` 의 같은 구간은 §4.3a 다. `ABORT_SAFE` 의 정지는 어느 planner 에서도 이 절의 대상이 아니다 (관절공간, §4.1).

`DECEL` 진입 시각 $t_s$의 기준 상태 $(x_s,\dot x_s)$에서 시작한다. $\hat u_s=\dot x_s/\Vert\dot x_s\Vert$, $\tau=t-t_s$, $\tau_s=\Vert\dot x_s\Vert/a_{dec}$.

$$p_v(\tau)=x_s+\dot x_s\tau-\tfrac12a_{dec}\hat u_s\tau^2,\quad v_v(\tau)=\dot x_s-a_{dec}\hat u_s\tau,\quad a_v=-a_{dec}\hat u_s\qquad(\tau\le\tau_s)$$

$\tau>\tau_s$이면 $p_v=x_s+\tfrac12\Vert\dot x_s\Vert\tau_s\hat u_s$, $v_v=a_v=0$.

`DECEL` 진입은 $now_{lead}\ge t_c$ 이고 $t_s$ 는 그 tick 의 $now_{lead}$ 다 (plan §3 — 감속 대상 전환도 팔 명령이다). $\tau$ 는 steady 실측 차로 구하고 tick 수 × `dt` 로 세지 않는다. $(x_s,\dot x_s)$ 는 기준 생성기 자신의 상태다 (R-DECEL-ENTRY, §4.1).

L4에 이 대상을 넣고 γ를 상수 1로 바꾼다($\dot\gamma=\ddot\gamma=0$). 전환 직후 오차는

$$e=x_s-p_v(0)=0,\qquad \dot e=\dot x_s-v_v(0)=0$$

로 **정확히 0**이다. "연속"이 아니라 "0"이라는 점에 주의한다 — 전환 직전의 $e,\dot e$ 는 DS 자체의 잔여 수렴오차만큼 0이 아니므로 (L4 §4.9 의 $\epsilon_{conv}$) 오차 자체는 그만큼 점프한다. **정확히 연속인 것은 기준 상태 $(x,\dot x)$** 이고, 설계상 필요한 것은 그쪽이다 (G7-B). 기준 가속도는 불연속이다 ($\tau=0$ 과 $\tau_s$ 에서 jerk 무한) — $a_{dec}$ 를 램프로 올리는 처리는 없다.

**층간 제약: $a_{dec}\le$ `reference.a_max`.** $e^+=\dot e^+=0$ 이므로 전환 직후 $u_{des}=a_v$ 가 되어 크기가 정확히 $a_{dec}$ 다. $a_{dec}$ 가 L4 가속 한계보다 크면 `DECEL` 첫 틱부터 포화가 걸린다. L0 파라미터 검증기가 이 관계를 검사한다 (planner 와 무관하게).

정지거리 $\Vert\dot x_s\Vert^2/(2a_{dec})$는 L3 §4.9의 예약값($\dot x_s\approx\gamma_f v_c$)과 같다. **`a_dec` 는 단일 키** `supervisor.decel.a_dec` 이고 소비자는 둘이다: 계획기 탐색의 정지점 예약 (`StoppingPoint` — $p_{stop}=p_c+(\gamma_f\Vert v\Vert)^2/(2a_{dec})\,\hat v$, 두 planner 에 공통) 과 이 절의 감속 대상 (`closed_form` 만). `mpc` 에서는 RT 가 이 값을 쓰지 않는다 — 탐색의 예약에만 남는다.

`DECEL` · `HOLD` 의 감속 대상 추종은 기준 포화를 세지 않는다 (§4.2 `REF_SATURATED`) — 정지 중의 포화는 정지가 길어지는 것이지 실패한 포구가 아니다. 대상이 정지 ($\tau\ge\tau_s$) 하면 `HOLD` 이고, `HOLD` 는 정지한 대상 ($p_v$ 고정) 을 계속 추종한다.

감속 대상 계산은 ROS 비의존 순수 조각이다 (`EvaluateDecelTarget`, `decel_target.hpp`).

### 4.3a MPC 구간 추종 — `APPROACH` 부터 정지까지 (`supervisor.decel.mode: mpc`) `[MPC E1-F04 · E1-F09]`

결정의 근거는 MPC 계획 문서 MD-34 – MD-45 · MD-56 – MD-58 · MD-65 – MD-69 ([MPC_DUALARM_PLAN.md](../MPC_DUALARM_PLAN.md) §4) 에 있다. 이 절은 동작만 적는다.

`mpc` 에서 팔의 기준은 `APPROACH` 부터 `HOLD` 까지 계획기가 게시한 **관절 노드 구간** 하나의 흐름이다. RT 는 soft-catch 기준 생성기도 §4.3 의 closed-form 감속도 돌리지 않는다. 포구점 · 포구 시각의 탐색은 `closed_form` 과 같다 (L3).

- **planner 는 하나를 고른다 (MD-44 · MD-45).** configure 에서 `supervisor.decel.mode` 로 정하고 활성화 동안 바뀌지 않는다. `closed_form` (코드 기본 — 키가 없을 때. 출하 YAML 은 두 로봇 `mpc`) 은 §4.3 과 L4 의 soft-catch 기준 그대로이고, RT 는 decel box 를 읽지 않으며 계획기는 decel 코어를 만들지 않는다. `mpc` 는 구간을 따른다. 따를 구간이 없으면 `ParamsTbd` 로 `ABORT_SAFE` 다. 다른 법칙으로 넘어가는 fallback 은 없다.
- **전제 (MD-34).** `mpc` 는 catch sub-model 샘플러, `joint_cmd.K_n` > 0, `planner.gamma.eta_v` < 1, 팔 관절마다 `max_velocity` 와 CLIK 의 관절별 속도 · 위치 box, decel 계획기 (`planner.enabled` + `planner.decel_mpc.approach.n_pre_max` > 0, oracle profile 은 예외 — 켜는 키는 없고 `mode: mpc` 가 그것이다), 그리고 접수 나이 상한이 아래 "대기 구간" 의 대기 시간보다 클 것을 요구한다. 하나라도 없으면 park (`kDecelModeUnmet`). `planner.workspace.catch_box` 는 이 전제가 아니다 — 탐색의 키다. 포구 전 격자가 없으면 (`n_pre_max` 0) plan 과 함께 채택할 구간을 낼 수 없어 decel 계획기를 만들지 않는다 (MD-70).
- **쌍 채택 (MD-56 · MD-65).** 계획기는 plan 과 그 첫 구간을 쌍으로 게시한다 (구간 먼저, 같은 `publish_ns`). `TRACKING` 의 lane 은 이 tick 이 채택할 수 있는 plan 을 기준으로 box 의 구간을 **판정만** 하고 (`JudgeDecelPlan`), `TRACKING → APPROACH` edge 가 plan 과 구간을 같은 tick 에 함께 채택한다. 구간이 통과하지 못하면 plan 도 받지 않는다 (`NO_CATCHABLE_PLAN` 으로 머문다). lane 이 구간을 먼저 채택해 두면 edge 가 걸리지 않은 tick 뒤로 그 구간이 `repeat` 로 거부돼 쌍이 들어오지 못한다 — 그래서 판정과 채택을 나눈다. 채택 뒤 `APPROACH` 에서 새 plan 은 받지 않는다 (MD-57).
- **구간의 접수 판정 (MD-37 · MD-66 · MD-67).** `APPROACH` · `COMMITTED` · `CLOSING` · `DECEL` 의 매 tick 에 decel box 를 한 번 Load 하고 `JudgeDecelPlan` 으로 판정한다. 통과 조건: `valid`, 같은 activation, 따르는 plan 의 id · $t_c$ · track (구간은 **plan 의 track** 을 싣는다. 동결 뒤 RT 가 마지막으로 소비한 track 과는 다를 수 있어 그것과는 비교하지 않는다), 이미 받은 것보다 새 `decel_seq`, 나이 ≤ 50 ms (`kDecelAdmissionMaxAgeNs`, 게시 시각 기준), 게시 시각과 구간이 출발한 RT 상태 (`rt_state_ns`) 가 모두 reset floor 뒤, 관절 수가 샘플러가 묶인 팔과 같음 (평가할 수 없는 구간은 `malformed` — 채택해 두면 node 0 에서 abort 가 된다), 노드 값의 형식.
- **RT 는 구간이 어디서 정지하는지를 판정하지 않는다 (MD-73).** `catch_box` 를 보는 것은 계획기의 탐색뿐이고, 탐색은 그것을 **포구점 $p_c$ 와 closed-form 정지점 $p_{stop}$ (§4.3 의 예약)** 두 곳에만 건다 (`planner_search.cpp`). MPC 구간의 정지 위치는 관절 한계만 지키면 된다 — MPC 의 관절 행과 CLIK 의 box 가 그것을 지킨다. 그래서 정지 부분이 `catch_box` 를 벗어나는 구간도 채택되고 따라진다.
- **대기 구간 — 교체와 나이 (MD-37 · MD-58).** 대기 슬롯은 하나다. 비었으면 채운다. 차 있으면 node 0 시각이 **같은** 더 새 구간 (같은 격자점을 새 예측으로 다시 푼 것) 만 교체하고, 다른 격자점의 구간은 box 에 두고 다음 tick 에 다시 본다 (덮어쓰면 그 사이의 시각에 따를 것이 없어진다). 다음 격자점의 구간이 box 에서 기다리는 시간은 최대 `planner.decel_mpc.budget.replan_s` + 3 tick 이고, 나이 상한이 그보다 커야 한다 (configure 가 확인한다). 그 시간을 넘겨 기다린 구간은 나이로 거부된다.
- **node 0 전 (MD-68).** 첫 구간의 node 0 가 오기 전의 `APPROACH` (lead 에 따라 `COMMITTED` 초입까지) 는 채택 tick 에 seed 한 명령을 그대로 든다 — 법칙을 돌리지 않는다. 계획기가 첫 구간을 정지한 보고 자세에서 풀었으므로 그 전제와 같다.
- **전환 게이트 (MD-38 · MD-39 · MD-40).** 샘플 시각은 $s=now_{lead}+h$ 다. 대기 구간은 $s\ge$ node 0 시각이고, 따르는 plan 의 id · $t_c$ 와 맞고, 관절마다 $\vert\Delta\dot q_i\vert+K_p\vert\Delta q_i\vert\le\rho_{\max}(1-\eta_v)\dot q_{\max,i}$ 일 때 따르는 구간이 된다 ($\Delta$ 는 들고 있는 명령과 구간의 $s$ 에서의 차, $K_p$ = `joint_cmd.K_p`, $\rho_{\max}$ = `supervisor.decel.switch_margin`). `HOLD` 를 뺀 모든 따르는 모드에서 같다. 게이트를 못 지난 구간은 버린다 — 따르던 구간이 있으면 그것을 계속 따르고, 없으면 (첫 구간) `ABORT_SAFE` 다. `DECEL` 진입은 전환이 아니다: $t_c$ 에 따르던 구간을 그대로 이어 따른다.
- **추종 tick (MD-36).** 구간을 $s$ 에서 샘플해 catch frame 의 위치 · 축 · twist 를 CLIK 목표와 feedforward 로 넘기고, 자세 목표를 $q_{ref}+\dot q_{ref}/K_n$ 으로 넘긴다 ($K_n(q'-q)=K_n(q_{ref}-q)+\dot q_{ref}$). tick 을 나가는 명령은 구간의 $now_{lead}+2h$ 값이고, 계획기는 RT 의 보고를 같은 label 로 읽는다. 따르는 중에 구간이 plan 과 어긋나거나 샘플이 실패하면 `ParamsTbd` 로 `ABORT_SAFE` 다. 구간의 마지막 노드를 지나면 정지로 보고 `HOLD` 다.
- **`HOLD`.** 진입 tick 에 대기 구간을 버리고, `HOLD` 동안은 전환하지 않는다. 따르던 구간의 마지막 노드 뒤 샘플 (정지 상태) 을 계속 CLIK 에 넘긴다.
- **감독 사유는 그대로다.** 공 lane 의 사유 (`BALL_STALE` · `TRACK_CHANGED` · `HORIZON_EXTRAP` → `RETREAT`, 동결 뒤 `BALL_STALE_LONG` → `ABORT_SAFE`) 와 CLIK 의 사유 (`QP_FAILED` · `JOINT_CONFLICT` · `TRACK_ERR`) 는 `closed_form` 과 같은 전이표 행을 탄다. `DECEL` 전의 추종 tick 은 구간을 샘플하기 전에 공 궤적을 lead 시각에서 한 번 샘플해 같은 판정을 한다 — 구간은 그 표본을 읽지 않는다. 기준 포화 (`REF_SATURATED`) 는 soft-catch 기준의 사유라 `mpc` 에서는 나지 않는다.
- **보고 (MD-58 · MD-69).** RT 는 매 tick `PlannerRtState` 에 따르는 구간 (`decel_active` · `decel_seq` — `APPROACH` 부터 `HOLD` 까지) 과 대기 구간 (`decel_pending` · `decel_pending_seq`) 을 싣는다. 계획기는 재계획의 출처를 이 보고로만 정한다. 게이트가 거부했거나 시행과 함께 버린 구간은 보고에서 빠진다.
- **reset (MD-35, E-8).** 대기 · 따르는 구간과 채택 기억, 쌍 판정은 `ResetTrialState` (activation · E-STOP · fault reset) 와 `ResetTrialScope` (재무장 · `RETREAT → IDLE`), 그리고 `ABORT_SAFE` · `RETREAT` 진입에서 버린다 (한 함수 `DropDecelSegments`). `HOLD` 진입은 대기 구간만 버린다.

**두 planner 의 법칙을 나란히.** 상태 머신 · 전이표 · 손 시퀀서 · 접촉 판정 · `ABORT_SAFE` · `RETREAT` · E-STOP 정책은 공통이다.

| 구간 | `closed_form` | `mpc` |
|---|---|---|
| `TRACKING → APPROACH` | plan 접수 (`JudgePlan`) | plan 접수 + 첫 구간의 판정, 둘을 같은 tick 에 채택 |
| `APPROACH` | soft-catch 기준 (L4) → CLIK. 새 plan 으로 교체 가능 ($T_{freeze}$ 밖에서) | node 0 전: seed 명령 유지. 그 뒤: 구간 샘플 → CLIK. plan 교체 없음, 구간만 교체 |
| `COMMITTED` · `CLOSING` | soft-catch 기준 계속 | 구간 계속 (재계획 구간으로 전환 가능) |
| `DECEL` 진입 | 기준 생성기의 상태를 $(x_s,\dot x_s)$ 로 잡고 대상을 §4.3 으로 바꾼다 | 전환 없음 — 같은 구간의 다음 샘플 |
| `DECEL` | TCP 직선 등감속 대상 (§4.3) → L4 (γ ≡ 1) → CLIK | 구간의 정지 부분 → CLIK |
| `DECEL → HOLD` | $\tau\ge\tau_s$ | 구간의 마지막 노드를 지남 |
| `HOLD` | 정지한 대상을 추종 | 구간의 정지 상태를 추종, 대기 구간 폐기 · 전환 없음 |
| `REF_SATURATED` | 공 추종 tick 에서 센다 | 발화하지 않는다 |
| `supervisor.decel.a_dec` | 탐색의 정지점 예약 + `DECEL` 대상 | 탐색의 정지점 예약만 |
| 법칙에 필요한 것이 없을 때 | `ParamsTbd` (감속 대상이 유효하지 않음) | `ParamsTbd` (따를 구간 없음 · plan 불일치 · 샘플 실패) |

### 4.4 접촉 판정

**부호 규약.** sim 과 실기 두 경로 모두 **finger-on-object** 부호다 — 실기 P1b `HandSensorState` (250 Hz) 와 같게 `rtc_mujoco_sim` 이 fingertip-on-environment 로 발행한다 (repo 규약상 sim 쪽 부호 스위치를 두지 않는다. 부호 변환 지점은 `rtc::grasp::PullContactConfig::force_sign` 한 곳). 접촉 판정은 바이어스를 뺀 크기 $\Vert F-b\Vert$ 만 보므로 이 부호 규약에 의존하지 않는다. 판정 시각은 수신 steady 시각이고 `header.stamp` 는 staleness 판단에 쓰지 않는다.

센서 $i$의 바이어스 $b_i$는 **`ARMED`/`TRACKING` 구간에서 손이 `q_pre` 에 정착해 있는 동안** 의 **지수이동평균(EMA, `supervisor.contact.baseline_alpha`)** 으로 추정한다 (원형 버퍼가 아니다 — 할당 없는 O(1) 기억). 잡음 표준편차 $\hat\sigma_i$도 같은 창에서 잔차 제곱 $\Vert F_i-b_i\Vert^2$ 의 EMA 로 구한다. 창을 `COMMITTED` 구간으로 잡지 않는 이유는 그 길이가 $T_{freeze}-T_{close,tot}\approx T_{arm}+T_{margin}$ (수십 ms) 뿐이라 표본 몇 개로 $\hat\sigma_i$ 를 추정하게 되기 때문이다. 추정 상태는 재무장 리셋이 비운다 (§4.8). **바이어스 표본 수가 `supervisor.contact.n_baseline_min` 미만이면 결과는 `Undetermined`** 다 — `Missed` 로 오판하지 않는다.

**디바운서는 tick 이 아니라 샘플 단위로 먹인다.** 실기 지문 센서는 250 Hz, 제어 tick 은 500 Hz 이므로, `UpdateBaseline`/`UpdateContact` 는 매 tick 이 아니라 `inference_sequence[g]` 가 바뀐 tick 에만 호출한다.

$$f_i(t)=\Vert F_i(t)-b_i\Vert,\qquad c_i(t)=\mathbb 1\big[f_i>\max(f_{\min},\,k_\sigma\hat\sigma_i)\big]$$

$N_{deb}$개 연속 샘플이 참이면 센서 $i$ 접촉으로 확정한다 (`ContactDebouncer`, `contact_debounce.hpp`).

결과 판정(`HOLD` 종료 시):
- **포획:** 판정 창 $[t_{cmd},\,t_c+T_{conf}]$ (실제 시각 $now$ 로 비교 — 공의 물리 시각과 같은 축, plan §3) 안에 접촉 센서 수가 $m_{\min}$ 이상이었고, `HOLD` 종료 시점에도 $m_{\min}$ 이상이 유지되는 경우.
- **실패:** 판정 창 안에 접촉이 없는 경우.
- **미확정:** 판정 창 안에서 센서가 stale 이었거나, 바이어스 표본 수가 `n_baseline_min` 미만이거나, 지문 lane 이 구성되지 않았거나 지문 수가 $m_{\min}$ 미만인 경우.

오경보 확률은 센서 잡음 분포에 의존한다. $k_\sigma$는 실기 잡음 측정 후 정한다 (가우시안 가정이면 $k_\sigma=3$에서 단측 약 0.13%, 등급 a) — sim 의 fingertip lane 은 잡음이 없어 $\hat\sigma\approx0$ 이고 `f_min` 만 유효하다 (TBD-HAND-03).

**손 관절 증거 (D-S8-8 (b)).** 지문 합의만 보는 판정은 공이 지문이 아닌 손가락 링크·손바닥에 얹힌 포구를 실패로 읽는다. 그래서 두 번째 증인을 더한다: 공을 든 손은 **손가락이 공에 막혀 멈춘다**. caging 관절 $i\in C$ 마다, 같은 관절에서 세 절이 동시에 성립하면 그 관절을 *stalled* 로 본다.

$$\rho_i=\frac{(q_i-q_{pre,i})\,s_i}{|q_{close,i}-q_{pre,i}|},\quad s_i=\operatorname{sign}(q_{close,i}-q_{pre,i})$$

$$\text{stalled}_i=\big[\rho_{\min}\le\rho_i\le\rho_{\max}\big]\wedge\big[|\dot q_i|\le\dot q_{tol}\big]\wedge\big[s_i\tau_i/\tau_{\max,i}\ge\kappa\big]$$

- 손 위상이 `Hold` 이고 (hold 목표는 `q_close` 라 빈 손은 $\rho=1$ 에 닿는다 — 그래서 `hold.mode: close_target` 에서만 허용한다, L6 §6) q·q̇·effort lane 이 모두 readable 할 때, stalled 관절이 `min_joints` 개 이상이면 그 tick 은 *blocked* 다. 이 상태가 **판정 tick 까지 `t_persist` 이상 끊기지 않았으면** 손 증거가 성립한다.
- **포획 = (위 지문 조건) ∨ 손 증거.** 손 증거는 실패를 포획으로 올리는 데만 쓴다. **미확정 조건이 먼저다** — 지문 lane 이 판정할 수 없는 경우 (stale·바이어스 부족) 에는 손이 막혀 있어도 미확정이다. 손 증거는 lane 을 대신하지 않고 증거를 더할 뿐이다.
- 관절별로 보는 이유: 공에 닿는 관절은 몇 개뿐이라, min-ρ 에 전 관절 토크 통계를 짝지으면 공에 안 닿은 관절 (τ≈0) 이 통계를 좌우한다. $\rho_{\min}$ 은 닫히지 않은 채 다른 것을 미는 손가락을, 부호 있는 토크는 무언가에 밀려 열리는 손가락을 제외한다.
- $\tau_{\max,i}$ 는 손 device 의 `joint_limits.max_torque` 다 — 로봇 상수를 코드에 두지 않는다. 임계는 `robot.hand.capture.*` ($\rho_{\min}$ `rho_min` · $\rho_{\max}$ `rho_max` · $\kappa$ `effort_frac_min` · `t_persist` · `min_joints`, L6 §6) 이고 $\dot q_{tol}$ 은 `robot.hand.qd_tol` 이다. `capture` 블록이 없으면 판정은 지문만 본다. 블록이 있는데 임계가 비었거나 `max_torque` 가 없으면 시행을 park 한다 (조용히 지문만 쓰지 않는다).
- **실기 한계.** 실기 손 드라이버의 effort lane 은 관절 토크가 아니라 전류다 (`udp_hand` 는 `joint_currents` 를 발행). 그래서 `robot.hand.capture.provisional` 이 실기 구성을 막는다 — $s_i\tau_i/\tau_{\max,i}$ 절은 effort 가 관절 토크라는 전제로 쓴 것이다. 또 *blocked* 가 `t_persist` 동안 끊기지 않아야 하므로, q̇ 잡음으로 stalled 관절 수가 `min_joints` 아래로 한 tick 이라도 내려가면 지속 시간은 0 부터 다시 센다 — 실기의 q̇ 잡음에 맞춘 `qd_tol`·`t_persist` 는 실기 단계가 정한다.
- 기록: `catching_diag.csv` 의 `hand_stalled_n`·`hand_effort_frac` (C 에서 $s_i\tau_i/\tau_{\max,i}$ 의 최대)·`hand_blocked_s` (끊김 없는 막힘의 지속 시간 [s])·`outcome_source` (0 없음·1 지문·2 손·3 둘 다). 그래서 같은 투척에서 "지문만 봤을 때의 판정" 을 재구성할 수 있다.
- 이 판정의 목적은 진단 (sim truth 와의 혼동행렬) 과, truth 가 없는 실기의 판정이다. sim 의 성공률은 truth 로 판정하고 release 는 판정과 무관하다 (§4.8 "RETREAT 순서").

### 4.5 준비(ARMED) 조건

1. L0 파라미터 검증 통과(TBD 없음)
2. 시계 건강(실기) — 신호 출처가 없어 검사하지 않는다 (TBD-NET-01, G7-F)
3. L5 `lead_enable=true`이면 `T_arm` 확정
4. 로봇이 대기 자세 허용오차 안에 있음 (`IDLE` homing 으로 도달, §4.1) — 위치 `pose_tol` **과 정지** (|q̇| ≤ `homing.qd_tol`). 팔 속도 lane 을 읽을 수 없으면 (hole) 대기 자세가 아니다: homing 도착 판정도 같은 검사라 끝나지 않고 대기 자세를 계속 명령한다
5. speed scaling = 1(실기) — 신호 출처가 없어 검사하지 않는다 (TBD-ARM-03, G7-F)
6. 손 `q_pre` 도달 (`q_open` 은 homing 중에만 쓰고 대기 중 손은 항상 `q_pre` 다, §4.1) **과 정지** — 손 속도 lane 을 읽을 수 없으면 정착이 아니다. 손 시퀀서의 `at_target` 도 같은 규칙이다 — 속도 lane 비판독이면 `RELEASE → PRESHAPE` 가 일어나지 않아 `RETREAT` 는 재무장하지 않고 release timeout 으로 끝나며, 접촉 baseline 도 학습되지 않는다. 닫힘 판정 (ρ) 과 hold offset 은 위치만 읽으므로 그대로 동작한다

드라이버가 준비 조건으로 실제로 묻는 것은 "구성이 무장 가능한가 (검증 통과) ∧ 운용자가 무장했는가 (`catching.enable`)" 이고, 4 · 6 은 `IDLE → ARMED` 의 전진 조건이다. 2 · 5 는 실기 단계가 신호를 연결할 때 이 자리에 들어간다.

**조건 상실의 처리.** §4.2 에 전용 사유가 없어 전이표는 `PARAMS_TBD` 를 재사용한다(전이표 헤더가 근거를 갖는다). 어디로 가는지는 **그 모드가 운동을 싣고 있을 수 있는가**로 갈린다:

- `IDLE` 은 self-loop, `ARMED`·`RETREAT` 는 `IDLE`. `ARMED` 는 팔이 이미 서 있어 그대로 성립한다. `RETREAT` 는 정지 램프 + 관절공간 복귀로 운동을 싣는다(§4.1) — 이 행이 성립하는 이유는 **`IDLE` 이 그 램프를 소유하기 때문**이다: `RETREAT` 중 disarm 되면 carried `arm_qd_cmd_` 를 그대로 `IDLE` 로 넘기고, `IDLE` 이 R-IDLE 규칙(§4.1)으로 정지까지 램프를 마저 돌린다 — 전이 자체는 즉시 일어나도 관절 속도가 즉시 0 이 되는 것은 아니다.
- `TRACKING`·`APPROACH`·`COMMITTED`·`CLOSING`·`DECEL`·`HOLD` 는 **`ABORT_SAFE`** 다. 운전자가 approach 중에 무장을 내리는 것이 정상 경로이고, 전이 행이 없으면 모드가 그대로 남아 드라이버가 법칙 호출을 멈춘 채 실려 있던 관절 명령이 **그 자리에서 얼어붙는다** — L5 의 감속 계약이 절대 내보내지 말라고 하는 1-tick 무한 감속이다. `ABORT_SAFE` 는 그 ramp 를 소유하고, 정지 후 `RETREAT` 로 빠지는 경로를 이미 갖고 있다.
- `ABORT_SAFE` 는 **행이 없다 (의도적)**. 그 모드의 출구는 정지 완료이고, 여기서 사유를 답하면 매 tick 그 판정을 가로채 ramp 가 끝나도 나가지 못한다. 드라이버는 이 한 모드에서만 준비 조건 검사를 통과시킨다.

이렇게 해서 무장 해제는 `APPROACH → ABORT_SAFE → (정지) → RETREAT → IDLE` 로 **종결**한다 — 팔은 ramp 로 서고, 운전자는 `IDLE` 로 돌아온 것을 본다.

### 4.6 γ 하향 (포화 대응 1차 수단) — v1 범위 밖 `[확정 D-8]`

구현하지 않는다. 포화 대응은 §4.2 `REF_SATURATED` 하나다 (`closed_form` — `COMMITTED` 전이면 `RETREAT`, 이후면 `ABORT_SAFE`).

### 4.7 충격량 예산 `[논문 외 유도]` `[TBD-IMP-01]`

**구현하지 않았다.** 아래 식의 $\Delta p$ 를 계획 후보의 게이트로 쓰는 코드와 상한 키 ($\Delta p_{\max}$) 는 없다. 식과 확인 항목은 실기 단계의 입력으로 남긴다.

본 시스템은 UR5e를 position 명령(실기 `ur_driver_native` → vendor `forward_position_controller`)으로 구동하므로 **접촉 순간의 임피던스가 사실상 위치 서보 강성**이고, 순응 요소가 없다. [R16]의 문제의식이 그대로 적용된다 — 빠른 물체와 접근하는 로봇 사이의 속도 불일치가 큰 충격력을 만들고, 접촉 불안정과 손상으로 이어진다.

본 설계의 유일한 완화책은 상대속도를 줄이는 soft catch다.

$$\Delta p=m_{ball}\,(1-\gamma_f)\Vert v(t_c)\Vert$$

$\gamma_f$ 는 탐색이 후보에 매긴 값이다 (두 planner 에 공통). `mpc` 에서 포구 순간의 팔 속도는 MPC 구간이 정하므로 (상대속도 행, `decel_mpc.hpp`) 이 식의 $\gamma_f$ 가 실제 상대속도를 정하지는 않는다.

문제는 γ가 작을 수밖에 없는 구간이 넓다는 것이다(L3 §4.5, 마스터 §4.1). $\gamma_f=0.25$, $\Vert v\Vert=2$ m/s면 상대속도가 1.5 m/s이고, 이 운동량을 지문 센서만 달린 손가락과 위치 서보 팔이 받는다.

**계획·검증 단계에서 산출하고 기록할 것.**

1. **충격량과 손가락 토크.** $\Delta p$ 를 접촉 시간 $\Delta t_{imp}$ 로 나눈 평균 힘 $\bar F=\Delta p/\Delta t_{imp}$ 와, 그것이 만드는 관절 토크를 P1b 모터 한계와 비교한다. 설정·모델값(YAML `max_torque` = URDF effort = MJCF forcerange)은 서로 일치한다(L6 G6-3). 이 값을 그대로 운용 한계로 쓸 수 있는지는 **미확정** — 권위 있는 운용 한계(nominal·continuous·peak·설정값 중 어느 것)는 **D-12 대기** 이고(plan §7.3), 확정 전에는 이 게이트의 토크 비교 부분을 `NOT_EVALUATED` 로 기록한다(G7-B3). $\Delta t_{imp}$ 는 시뮬레이션 접촉 참값(시각·충격량·접촉력 출력)에서 측정한다.
2. **팔 쪽 반력.** position 명령은 접촉 중에도 계속 진행하므로, 공이 손 안에서 감속되는 동안의 반력은 구조와 관절이 받는다. UR5e 보호 정지 임계 대비 어디인지 확인한다(`[HW-P1B]`, 저속부터).
3. **충격 후 CLIK 괴리.** 충격으로 $q$ 가 $q_c$ 에서 벌어지면 (D-6 으로 CLIK 은 $q_c$ 에서 평가하므로 이 괴리는 `TRACK_ERR` 로만 보인다) L7 `TRACK_ERR` 가 오동작할 수 있다. `track_err_abort` 를 정할 때 충격 구간을 제외하거나 임계를 시간 가변으로 둔다 — 지금 코드의 임계는 단일 상수다.
4. **반발.** L3 §4.5의 $d_{eff}=d(1+1/e)$ 는 법선 단일 충돌 모델이다. [R19]가 다루는 접선 컴플라이언스는 무시한다는 가정을 명시한다.

**계획에 거는 게이트 (미구현).** $\Delta p\le\Delta p_{\max}$ 를 L3 후보 게이트에 추가한다(`TBD-IMP-01` 확정 후). 이 게이트가 γ 창의 하한을 실질적으로 끌어올린다. 토크 도출 가속 한계(D-16)의 여유 $\eta_\tau<1$ 도 이 충격을 위한 것이다(plan §9).

**본 설계가 하지 않는 것.** [R16]의 강성·접촉력 동시 최적화와 접촉점 선택, [R18]의 reference spreading(충격 순간 기준 궤적 불연속 처리)은 **범위 밖**이다. 둘 다 토크 또는 임피던스 인터페이스를 전제하는데 UR5e는 position으로 확정돼 있다(마스터 §1.1). 저속 구간에서 성공률이 확보되지 않으면 이 제약을 재검토해야 한다 — 그때의 선택지가 위 두 문헌이다.


### 4.8 재무장 리셋 목록 `[확정, S7.4 에서 함수로 분리]`

시행이 끝나는 **모든 경로** 에서 직전 시행의 상태를 되돌린다. 하나라도 빠지면 직전 시행의 상태가 남아 두 번째 투척이 다르게 동작하고, 단발 시행만 도는 시험은 그것을 잡지 못한다.

**멤버별 표는 컨트롤러 헤더가 정본이다.** RT tick 이 소유하는 멤버마다 어느 리셋이 되돌리는지 (또는 왜 면제인지) 는 `demo_catching_controller.hpp` 의 "Reset table" 주석이 갖고, `test_catching_reset_table.py` (모든 멤버에 행이 있는가) 와 `test_catching_reset_probe.cpp` (행의 주장이 참인가 — 멤버에 비기본값을 채운 뒤 리셋을 불러 표가 말한 멤버만 바뀌는지 본다) 가 그 표를 검사한다 (G8-A2). 이 절은 그 표가 따르는 규칙만 적는다.

**세 리셋과 그것이 도는 때.**

| 리셋 | 도는 때 | 범위 |
|---|---|---|
| `ResetForRearm` | `RETREAT → ARMED` (재무장 경계) | `ResetTrialScope` + "팔이 정지해 대기 자세에 있다" 는 가정 (명령 속도 0, homing 완료, 손 시퀀서 `Ready`) |
| `ResetTrialScope` | `RETREAT → IDLE` (복귀 중 disarm · E-STOP · release timeout) | 시행 범위의 상태. 명령 속도는 남긴다 — 팔이 아직 움직일 수 있고 `IDLE` 이 램프로 내린다 (R-IDLE) |
| `ResetTrialState` | activation · E-STOP 의 발동과 해제 각각 · 받아들인 fault reset — RT tick 이 epoch 의 변화를 보고 돈다 (P-1 (a)) | 시행 범위 + hold latch · 대기 중인 목표 · 명령의 seed (다음 판독 가능 tick 이 측정 자세로 다시 잡는다) · vision 소비 기억 · 손 시퀀서 비활성. activation 일 때만 모드를 `IDLE` 로 두고 채택한 `wait_pose` 를 YAML 로 되돌린다 — E-STOP 은 모드를 전이표에 맡긴다 (§4.1) |

**규칙.**

1. **시행 범위의 상태는 세 리셋이 모두 되돌린다.** plan 과 그 접수 기억, 동결 기억 ($t_c$ · $t_{cmd}$ · generation), 법칙이 따르는 궤적 스냅샷, 기준 생성기의 seed 여부, 샘플러 hint, 감속 진입 상태 (§4.3), MPC 구간 (§4.3a — `ABORT_SAFE` · `RETREAT` 진입에서도 버리고, `HOLD` 진입은 대기 구간만 버린다), 포화 연속 수, 추종 오차, 접촉 바이어스 · 잡음 추정과 표본 수, debounce 카운터, 판정 창의 sticky 플래그, 손 증거의 지속 시간, `RETREAT` 단계.
2. **reset floor 와 reset epoch 는 같은 자리에서 함께 움직인다 (C-7).** 재무장에서도 `reset_floor_ns_` 를 그 tick 의 시각으로 올리고 `planner_reset_epoch_` 를 올린다. 접수 (plan · 구간) 는 floor 가 가르고, 계획기 스레드는 epoch 의 변화를 보고 탐색과 decel 계획기의 시행 상태를 스스로 지우며 decel box 의 구간을 거둔다 (RT 는 그 스레드의 상태를 직접 쓰지 않는다). 한쪽만 움직이면 정지 직전에 게시된 plan 이 다음 시행에 들어온다.
3. **재무장은 명령 $q_c$ 를 다시 seed 하지 않는다 (C-32).** 실려 있는 명령이 곧 대기 자세다. 측정으로 재시딩하면 서보 오차만큼의 계단이 된다. 속도만 0 으로 두고, CLIK 앵커와 직전 속도는 다음 시행이 plan 을 채택하는 tick 이 리셋한다 (옛 속도 기준의 가속 box 가 `bound_conflict` 를 내지 않도록).
4. **손은 `Ready` (`q_pre`) 로 돌아간다 — `Open` 이 아니다 (C-30).** 대기 중 손은 항상 `q_pre` 이고, `Open` 으로 두면 다음 시행의 준비 조건 (§4.5-6) 을 채우지 못한다. E-STOP · activation 의 리셋은 시퀀서를 비활성으로 두고 손 latch 가 측정 자세를 잡는다 (C-17).
5. **트랙 기억은 방향이 갈린다 (R-TRACK).** 재무장 쪽 리셋은 `last_trial_generation_` 을 **쓴다** (방금 끝난 시행의 공을 거부). `ResetTrialState` 는 그것을 **지운다** — 그 트랙이 E-STOP · fault 리셋을 넘어 계속될 수 있고 (track epoch 는 activation 과 별개, D-4), 지우지 않으면 새 generation 이 올 때까지 plan 을 받지 못한다.
6. **면제는 이유와 함께 표에 적는다.** 대표적인 둘:
   - **계획기 wake 신호 (eventfd) 는 어떤 리셋에서도 비우지 않는다 (C-7).** 비우면 리셋을 보는 동안 이미 올라온 **새 시행** 의 궤적 신호까지 지워 첫 plan 이 wake timeout 만큼 늦는다. 남겨도 안전한 것은 reset floor 가 옛 시행의 plan 을 거르기 때문이다.
   - **CLIK 실패 시행 수 (`qp_fail_streak_`) 는 재무장 (C-29) 과 E-STOP 이 지우지 않는다 (D-S9-D2).** 재무장마다 지우면 실패가 시행 경계를 넘지 못해 `FAULT` 에스컬레이션 (`n_qp`) 에 영영 닿지 못한다. E-STOP 은 풀이 불가능성에 대해 아무것도 말하지 않는다. 0 으로 되돌리는 것은 `HOLD` 판정 · fault reset · activation 뿐이다.
   - 그 밖의 면제는 "매 tick 다시 쓰는 값", "리셋 자신의 장부 (epoch)", "관측 카운터", "마지막 시행을 보고하는 값 (`outcome_`)" 의 네 부류다.
7. **새 stateful 멤버를 더할 때.** 헤더의 `RT-OWNED BEGIN` / `RT-OWNED END` 범위 안에 선언하고, 같은 변경에서 헤더의 리셋 표에 행을 더하고 (어느 리셋이 되돌리는가, 면제면 왜인가) 해당 리셋 함수에 넣는다. 범위 밖에 선언한 멤버는 행 존재 검사가 보지 못한다 — 리뷰가 잡아야 한다. 값이 실제로 되돌려지는지는 probe 테스트가 본다 (행이 있다는 것만으로는 보장되지 않는다).

plan 접수 쪽의 방어 (reset floor · 동결 창) 는 §4.1 R-ADMIT 이 적는다.

**RETREAT 순서.** 진입 → 정지 램프(`JointSpaceDecelStep`; `ABORT_SAFE` 경유면 no-op. 측정 팔이 정지 명령을 `track_err_abort` 안으로 따라잡을 때까지 머문다 — 안 그러면 `TRACK_ERR` abort 직후 서보 지연이 복귀 첫 tick 에 다시 `TRACK_ERR` 를 내 `ABORT_SAFE` ↔ `RETREAT` 를 돈다) → 관절공간 복귀(팔이 이미 `pose_tol` 안이면 생략) → 대기 자세 도착에서 손 Release(`q_pre`) → 손 `q_tol` 도달 → `ResetForRearm` → `ARMED`. **RETREAT 는 손을 움직이지 않는다.** 닫힌 손은 판정(Captured·Missed·Undetermined·Aborted)과 무관하게 복귀 내내 닫힌 채이고 대기 자세에서만 열린다. 아직 닫힘 명령이 나가지 않은 commit 은 진입 시 취소한다 — 손은 이미 `q_pre` 이므로 움직임은 없다. 근거: 지문 판정은 공이 링크·손바닥에 얹힌 포구를 Missed 로 읽을 수 있고 (§4.4), 판정에 따라 포구 지점에서 손을 열면 그 공을 떨어뜨린다. `q_pre` 도달로 손이 열려도 공이 남는 경우는 sim 드라이버의 `/sim/reset_ball`, 실기는 운용자가 처리한다.

**IDLE 순서.** 검증 통과 + 무장 → 팔이 `pose_tol` 밖이면 손 `q_open` → homing → 도착 → 손 `q_pre` 지시; 안이면 손만 `q_pre` → 손 도달 + §4.5 → `ARMED`.

§9 시나리오의 **"연속 2회 투척"**, **"abort 직후 재투척"**, **"RETREAT 복귀 중 disarm"** 이 이 규칙들을 닫힌 루프에서 검사한다.

## 5. C++ 구현

### 5.1 인터페이스

`Mode` (11) · `Reason` (21) · `Outcome` (5) 와 전이표의 정의는 `rtc_controllers/include/rtc_controllers/catching/transition_table.hpp` 에 있다 (namespace `rtc::catching`, ROS 비의존).

- `Reason` 목록은 §4.2 표와 1:1 이다. 비치명 사유(`kNoCatchablePlan`, `kBallStaleCommitted`, `kHorizonExtrap` 동결 후, `kHandTimeout`, `kTipStale`)는 전이 없이 기록만 한다.
- FSM 은 포구 컨트롤러의 `Compute` 안에서 매 tick 1회 진행한다. 입력은 L1 스냅샷(SeqLock 에서 읽은 POD), `PlanSnapshot` (`mpc` 는 여기에 `DecelPlanSnapshot`), 직전 tick 의 법칙 결과, `ControllerState` 의 측정값, $now$·$now_{lead}$ (§4.1) 이다. 출력은 그 tick 의 팔 법칙의 선택, 손 시퀀서 명령, 사유·결과다. 팔 법칙의 선택은 planner 마다 다르다 — `closed_form` 은 L4 의 대상 (공의 표본 / §4.3 의 가상 감속 대상) 을 고르고, `mpc` 는 L4 대상이 없이 구간의 표본을 CLIK 목표로 넘긴다 (§4.3a). 정지 (`ABORT_SAFE` · `FAULT`) 와 homing · 복귀는 두 planner 모두 관절공간 법칙이다.

### 5.2 구현 규칙

- 전이는 (상태 × 사유) 표 데이터로 판정하고, 한 틱에 최대 1회만 일으킨다.
- **별도 전이 로그(SPSC)는 두지 않는다 (C-27).** `Compute()` 가 단일 exit 인 매 tick 마다 `mode`·`reason` 을 포함한 전체 POD 를 기본 생성 후 채워 `catching_diag.csv` 에 싣는다(PROC-7, D-20). 전이는 이 per-tick 기록에서 mode 열이 바뀌는 행으로 **오프라인 도출**하며, 플로터가 그 행에 전이선을 그린다. 트랙 식별은 vision 의 `generation`(uint64, L1 §4.4, D-4)이다.
- **시각 비교는 plan §3 (D-2) 의 타입으로만 한다.** 내부 시각은 절대 steady ns 이고, 상대시각은 수치 코어 경계에서만 만든다. `BallTime`·`NowReal`·`NowLead` 를 서로 다른 타입으로 두어 혼용을 막는다 — 판정별 비교 대상은 §4.1 표와 plan §3 표가 같다. 매 tick 의 $now$ 는 steady 실측이며 tick 수 × `dt` 로 계산하지 않는다. **$T_{arm}\ne0$ fixture 필수** (0 이면 두 축이 같아져 버그가 숨는다).
- `TRACK_ERR` 는 $\Vert q_{meas}-q_c(now)\Vert$ 로 계산한다 (`UpdateTrackError`). $q_c(now-T_{arm})$ 를 꺼낼 지연 링은 없다 (§4.2).
- 지문 센서 바이어스·잡음 추정은 **EMA** 로 한다(`ContactDebouncer`, `supervisor.contact.baseline_alpha` — 할당 없음, 고정 길이 원형 버퍼가 아니다, §4.4).
- RT 경로에 try/catch·로깅·deactivate 요청을 두지 않는다 (RT-1~10). FAULT 는 컨트롤러 fault 래치로만 표현한다 (§4.1).
- **상태 출력.** 모드·사유·결과는 controller 소유 `rtc::SeqLock<T>` 스냅샷(trivially copyable POD) + `Setup*Publisher` 패턴의 non-RT publisher 로 낸다. `PublishRole` 에 새 토픽을 추가하지 않는다 (E-11).

## 6. YAML 파라미터

키는 **단일 원천**이다. 파라미터 로딩은 `LoadConfig(YAML)` + `ParseCatchingParams`. 값과 그 근거는 로봇별 YAML 과 그 주석이 갖는다 — `integrated_bringup/config/<robot>/controllers/demo_catching_controller.yaml` (`supervisor.*` 대부분과 `decel.mode`), 같은 폴더의 `catching/planner_closed_form.yaml` (`decel.a_dec`) · `catching/planner_mpc.yaml` (`decel.switch_margin`). 키가 없을 때의 파서 기본값과 범위는 `rtc_controllers/include/rtc_controllers/catching/catching_params.hpp` 다.

| 키 | 단위 | 뜻 |
|---|---|---|
| `supervisor.stale_committed_max_s` | s | §4.2 `BALL_STALE_LONG` — 동결 뒤 stale 을 `io.t_stale` 너머로 얼마나 더 견디는가 |
| `supervisor.n_qp` | – | §4.1 `FAULT` 진입 — CLIK 실패 (`QP_FAILED`·`JOINT_CONFLICT`) 로 끝난 **시행**의 연속 수. 세는 것은 solve 가 아니라 시행이고, `HOLD` 판정 · fault reset · activation 이 0 으로 되돌린다 |
| `supervisor.deadline.stop_s` | s | §4.1 운동 기한: `ABORT_SAFE` 정지 램프, 그리고 `RETREAT` 정지 단계 (명령 정지 + 측정 팔이 따라잡음). 시계는 그 단계의 진입 tick 에 새로 시작한다. 넘으면 `FAULT` |
| `supervisor.deadline.return_s` | s | §4.1 운동 기한: `RETREAT` 복귀 단계. 시계는 복귀 시작 tick 에 새로 시작한다. 복귀는 대기 자세까지의 거리에 비례하므로 정지와 키를 나눈다 |
| `supervisor.deadline.provisional` | – | 두 기한의 L0 §5.3 플래그 (없으면 `true`) — sim 은 경고, 실기 구성은 park. 실기 값은 실기에서 잰다 (D-S9-G) |
| `supervisor.track_err_abort` | rad | §4.2 `TRACK_ERR` 임계. **단일 원천** — L5 는 이 키를 참조만 한다. `RETREAT` 정지 단계의 "따라잡음" 판정도 이 값이다 |
| `supervisor.decel.a_dec` | m/s² | §4.3 감속 크기. **단일 원천**, ≤ `reference.a_max` (검증기). 소비자: 탐색의 정지점 예약 (두 planner) 과 `closed_form` 의 `DECEL` |
| `supervisor.decel.mode` | – | planner 선택: `closed_form` · `mpc` (다른 값은 configure 실패, 키가 없으면 `closed_form`). `mpc` 의 전제가 빠지면 park (§4.3a). `closed_form` 은 decel MPC 키를 보지 않는다 |
| `supervisor.decel.switch_margin` | – | §4.3a 전환 게이트의 $\rho_{\max}$, > 0 (아니면 configure 실패). `mpc` 에서만 읽는다 |
| `supervisor.contact.f_min` | N | §4.4 접촉 임계의 절대 하한. sim fingertip lane 은 잡음이 없어 이 값만 유효하다 |
| `supervisor.contact.k_sigma` | – | §4.4 잡음 배수. 실기 전용 (sim $\hat\sigma\approx0$) |
| `supervisor.contact.n_debounce` | 샘플 | §4.4 $N_{deb}$ — 연속 참 샘플 수. 센서 주기에 의존한다 |
| `supervisor.contact.m_min` | – | §4.4 $m_{\min}$ — 합의에 필요한 지문 수. 손 형상에 의존한다 |
| `supervisor.contact.T_confirm` | s | §4.4 판정 창 $[t_{cmd},\,t_c+T_{conf}]$ 의 $T_{conf}$ |
| `supervisor.contact.t_stale` | s | §4.2 `TIP_STALE` 의 수신 나이 임계 (D-24) |
| `supervisor.contact.baseline_alpha` | – | §4.4 바이어스·잡음 EMA 비율 |
| `supervisor.contact.n_baseline_min` | 샘플 | §4.4. 바이어스 표본이 이보다 적으면 결과 `Undetermined` |
| `supervisor.homing.v_max` | rad/s | §4.1 homing/retreat 관절공간 사다리꼴 속도 한계 |
| `supervisor.homing.eta_a` | – | §4.1. homing/retreat 가속 한계 = `qdd_max` × 이 값 |
| `supervisor.homing.qd_tol` | rad/s | §4.1 homing/retreat 도착 판정 (‖q̇‖∞), §4.5 의 "정지", fault reset 의 정지 판정 |
| `supervisor.ready.pose_tol` | rad | §4.5 대기 자세 허용오차 |
| `supervisor.sat_ticks` | tick | §4.2 `REF_SATURATED` — 연속 포화 tick 수. `closed_form` 에서만 뜻이 있다 |

`supervisor.ready.wait_pose` 는 없다 (C-10 — 이 키가 있으면 파서가 거부한다) — `IDLE` homing 목표·`ARMED` 대기 자세는 L3 §6 `planner.wait_pose` 를 그대로 참조한다.

**실기에서 다시 정해야 하는 값.** `supervisor.contact.*` 의 출하 값은 잡음 없는 sim lane 에서 정한 잠정값이다 — 실기 지문 센서의 잡음 (TBD-HAND-03) · 주기로 `f_min` · `k_sigma` · `n_debounce` · `baseline_alpha` · `n_baseline_min` · `t_stale` 를 다시 정한다. 이 키들에는 provisional 플래그가 없다: 실기 구성을 park 하는 것은 `supervisor.deadline.provisional`, `robot.hand.provisional`, `robot.hand.capture.provisional` 이고, 접촉 임계는 그 게이트에 걸리지 않으므로 실기 진입 전 확인 항목으로 따로 본다. `supervisor.track_err_abort` · `supervisor.stale_committed_max_s` 의 출하 값도 sim 에서 정한 것이다.

## 7. 단위 기술 구현 순서

구현 순서는 이 문서가 다루지 않는다 — 이 문서는 구현된 상태를 적는다. 구현되지 않은 것은 §10 에 모았다.

## 8. 디버깅 방법

- 타임라인 그래프: 모드 띠, $t_c$·$t_{cmd}$ 수직선, $\Vert e\Vert$, 포화 플래그, 손 $\rho(t)$, 센서 $f_i$와 임계.
- 예상치 못한 `RETREAT`: per-tick 기록의 사유 코드로 추적한다.
- 포획했는데 `Missed`: 판정 창과 센서 수신 시각 정렬(실기 async 센서 지연), 부호 규약(§4.4)을 확인한다. sim 에서는 공이 링크·손바닥에 얹힌 구조적 false-Missed 가 난다 (§4.4 손 관절 증거).
- 감속 중 흔들림 (`closed_form`): `a_dec` 값과 L5 가속 한계의 정합을 확인한다.
- `REF_SATURATED` 가 자주 발생 (`closed_form`): L3 rollout의 여유율(`eta_a`, η_v)이 낮거나 $T_w$ 가 짧은지 확인한다.
- `mode: mpc` 에서 `ParamsTbd` 로 abort: `catching_diag.csv` 의 `decel_event` (구간 없음 · 게이트 거부 · plan 불일치 · 샘플 실패 · node 0 미도래) 와 게이트 열 (`decel_rho` · `decel_dq_max` · `decel_dqd_max` · `decel_gate_joint`), 접수 판정 `decel_refusal` 을 본다.
- `mode: mpc` 에서 `TRACKING` 에 머묾 (`NoCatchablePlan`): plan 의 거부 사유와 첫 구간의 `decel_refusal` 을 함께 본다 — 구간이 통과하지 못하면 plan 도 받지 않는다 (§4.3a).
- `ARMED` 에 안 들어감: homing 목표 `wait_pose` 와 `pose_tol`, 손 `q_pre` 도달, `PARAMS_TBD`, 그리고 팔·손 **속도 lane 의 판독 여부** (backend 가 속도를 싣지 않으면 §4.5 의 4·6 이 성립하지 않는다) 를 확인한다. 속도 lane 은 전이를 남기지 않으므로 로그가 유일한 표시다 — publish 스레드가 축별로 닫힐 때 WARN `<arm|hand> velocity lane UNREADABLE …` 한 줄, 열릴 때 INFO 한 줄을 낸다 (위치 gate 가 닫힌 경우는 gate 진단이 말한다 — 그동안 속도 lane 은 판정되지 않으므로 episode 는 시작도 끝도 나지 않고, 기억은 activation 마다 새로 시작한다).
- `RETREAT` 복귀가 복귀 기한 fault 로 끝나고 reset 이 `kVelocityUnreadable` 로 거부됨: 같은 WARN 을 본다 — 팔 속도 lane 이 닫혀 있으면 도착 판정이 성립하지 않는다.
- 접촉 직후 `TRACK_ERR` abort: 충격으로 벌어진 $q-q_c$ 가 임계를 넘은 것인지 본다 (§4.7-3 — 임계는 단일 상수다).

## 9. 검증 방법과 합격 게이트

시나리오 테스트(모의 입력으로 RT 슈퍼바이저를 닫힌 루프로 실행, 기대 상태열 비교, $T_{arm}\ne0$). 시나리오의 정본은 `integrated_bringup/test/test_catching_supervisor_scenarios.cpp` 다.

| 시나리오 | 기대 상태열 |
|---|---|
| 정상 | Idle(homing) → Armed → Tracking → Approach → Committed → Closing → Decel → Hold → Retreat → Armed, `Captured` |
| 활성화 자세가 대기 자세 밖 | Idle(homing) → Armed (교착 없음) |
| 공 놓침 | … → Decel → Hold → Retreat, `Missed` |
| 동결 전 stale | … → Approach → Retreat, `BallStale` |
| 동결 후 짧은 stale | … → Committed → Closing → …, `BallStaleCommitted` 기록 |
| 동결 후 긴 stale | … → Committed → AbortSafe → Retreat, `BallStaleLong` |
| catchability 탈락 | Tracking 유지, `NoCatchablePlan` 기록 |
| 동결 전 포화 (`closed_form`) | … → Approach → Retreat, `RefSaturated` |
| 동결 후 포화 (`closed_form`) | … → Committed → AbortSafe, `RefSaturated` |
| QP 실패 | 임의 상태 → AbortSafe(관절공간 정지), `QpFailed`. CLIK 실패로 끝난 시행 연속 $N_{qp}$회 → Fault (좋은 solve 뒤의 실패도 센다, `HOLD` 판정이 끼면 0 — D-S9-D2) |
| 관절 경계 충돌 | 임의 상태 → AbortSafe(관절공간 정지), `JointConflict` |
| E-STOP 발동·해제 | 임의 상태 → (CM hold) → Idle, 자동 재개 없음, $q_c$·앵커 = $q_{meas}$ |
| fault 리셋 | Fault → Idle (`ResetFault`, 팔 정지 뒤). 정지 전·속도 lane 판독 불가면 Fault 유지, `fault_reset_refused` 기록 (D-S9-D3) |
| 운동 기한 초과 (D-S9-D1) | AbortSafe 램프·Retreat 정지·Retreat 복귀 → Fault, `AbortEscalated`, `fault_cause` 가 기한을 가리킴. Fault 는 명령을 정지까지 램프 |
| 복귀 중 팔 판독 불가 (D-S9-K) | Retreat 에서 명령 정지 유지 → 기한 초과면 Fault, 기한 안에 다시 읽히면 복귀 재개 → Armed |
| E-STOP 중 fault 리셋 (D-S9-L) | Fault (latch 내려감) → 해제 tick 에 Idle, `FaultReset` |
| TBD 파라미터 | Idle 유지, `ParamsTbd` |
| 연속 2회 투척 · abort 직후 재투척 (§4.8) | 두 시행이 각각 plan 을 한 번 채택해 같은 상태열을 돈다. 같은 공 (같은 `generation`) 으로는 다음 시행이 시작되지 않고, abort 된 시행의 plan 은 다음 시행에 채택되지 않는다 |
| 복귀 중 disarm (§4.8, R-IDLE) | Retreat → Idle, 명령은 `IDLE` 에서 정지까지 램프 (얼지 않는다) |
| `mode: mpc` 정상 (§4.3a) | … → Tracking → Approach (plan 과 첫 구간을 함께 채택, node 0 까지 명령 유지 → 전환) → Committed → Closing → Decel (같은 구간의 연속, 전환 없음) → Hold → Retreat → Armed, `Captured` |
| `mode: mpc` plan 의 첫 구간이 채택되지 않음 (없음 · 나이 · plan · track 불일치 · malformed · 관절 수 불일치 · reset 전) | Tracking 에 머문다, `NoCatchablePlan` — plan 도 받지 않는다 |
| `mode: mpc` 정지 부분이 `catch_box` 를 벗어나는 구간 | 채택하고 따른다 (쌍 · 재계획 모두) — RT 는 정지 위치를 판정하지 않는다 |
| `mode: mpc` 첫 구간이 node 0 에서 게이트를 못 지남 | … → Approach → AbortSafe → Retreat → Armed, `ParamsTbd` |
| `mode: mpc` 대기 슬롯 | 같은 node 0 시각의 더 새 구간은 대기 구간을 교체. 다음 격자점의 구간은 box 에서 기다렸다가 슬롯이 비면 채택, 나이 상한을 넘기면 채택하지 않는다 |
| `mode: mpc` 재계획 | 같은 궤적의 재계획은 포구 전 · 후 모두 node 0 에서 넘겨받고, 게이트를 넘는 재계획은 버리고 따르던 구간을 계속 따른다 |
| `mode: mpc` `APPROACH` 중 다른 plan | 받지 않는다 — 따르던 plan 과 구간으로 끝까지 간다 |
| `mode: mpc` 공 stale · CLIK 실패 | `closed_form` 과 같은 사유 · 전이 (`BallStale` → Retreat, `QpFailed` → AbortSafe), 구간은 버린다 |
| `mode: mpc` 중 E-STOP (대기 중 · `APPROACH` 추종 중 · `DECEL` · tick 사이 발동+해제) | 대기 · 따르는 구간을 버리고 보고에서 뺀다. 재무장 뒤 옛 구간으로는 plan 이 채택되지 않는다 |

| 게이트 | 기준 | 태그 |
|---|---|---|
| G7-A | 위 시나리오 전부 기대 상태열과 일치, 출하 전이표가 완전성 검사 (`CheckTransitionTableComplete`, 단위 테스트) 를 통과 | `[SIM-ANY]` |
| G7-B | `closed_form`: 감속 전환 시 기준 상태 $(x,\dot x)$ 연속 (< 1e-9) | `[SIM-ANY]` |
| G7-B′ | `mode: mpc` (MPC MD-39): node 0 를 전환 tick 의 명령 $(q_c,\dot q_c)$ 로 만든 구간에서 $\Vert p_d-\mathrm{FK}(q_c)\Vert$ · $\Vert V_{ff}-J\dot q_c\Vert$ · $\Delta q$ · $\Delta\dot q$ < 1e-9 — 정지한 첫 구간의 전환과, 움직이는 팔의 포구 전 재계획 전환 둘 다. `DECEL` 진입 tick 은 전환이 아니라 같은 구간의 다음 샘플이다 | `[SIM-ANY]` |
| G7-B3 | 충격량 $\Delta p$ 기록과 시뮬레이션 접촉 참값의 최대 접촉력 상관 확인, 손가락 관절 토크가 한계 이내 (한계 권위 출처 D-12 확정 전까지 토크 비교 부분은 `NOT_EVALUATED`) | `[SIM-P1B]` |
| G7-C | 합성 잡음에서 접촉 오경보율 기록 (임계는 사용자 결정) | `[SIM-ANY]` |
| G7-D | RT 할당 0 (`ScopedNoMalloc`·`ScopedAllocGate`), 틱 최악 실행시간 기록 | `[SIM-ANY]` |
| G7-E | `ur5e_p1b` 시뮬레이션 폐루프에서 결과 판정과 MuJoCo 참값 일치율 기록 (부호 규약 재확인 포함) | `[SIM-P1B]` |
| G7-F | 실기 speed scaling·시계·센서 stale 경로 동작 확인 (신호 출처 확보 후 — `SPEED_SCALING` · `CLOCK_UNHEALTHY` 의 발화 코드를 연결하고 §4.2 의 처리대로 전이하는지) | `[HW-P1B]` |
| G7-G | 관절 fresh + 지문 센서 dropout negative control 에서 `TIP_STALE` 또는 결과 `Undetermined` 발화, 옛 힘을 새 접촉으로 판정 0 (D-24) | `[SIM-ANY]` |
| G7-H | E-8 최소 계약 (P-1): (a) deactivate → 다른 컨트롤러가 팔 이동 → 재activate 첫 tick 이 옛 자세를 명령하지 않음, (b) trigger·clear·deactivate race 에서 reset writer 가 RT tick 하나, (c) 자동 재개 0, (d) `ClearEstop` 후 latched fault 유지 | `[SIM-ANY]` |

## 10. 미확정 항목

- **TBD-HAND-03** — 실기 지문 센서의 잡음 수준. `supervisor.contact.*` 의 실기 값 (§4.4, §6).
- **TBD-IMP-01** — 충격량 예산의 상한과 계획 게이트 (§4.7, 미구현). G7-B3 의 토크 비교는 D-12 대기.
- **TBD-ARM-03** (speed scaling) · **TBD-NET-01** (PTP) — 신호 출처. 전이 행은 있고 발화 코드가 없다 (§4.2, §4.5, G7-F).
- `supervisor.deadline.stop_s` · `return_s` 의 실기 값 — `provisional` 인 동안 실기 구성은 park (D-S9-G). 실기 쪽 E-STOP 절차 (§4.1 D-13 의 마지막 항목).
- `supervisor.track_err_abort` 의 실기 값, 충격 구간의 처리 (§4.7-3), $q_c(now-T_{arm})$ 지연 링 (미구현, §4.2).
- 손 관절 증거의 실기 적용 — effort lane 이 전류라는 것, `qd_tol` · `t_persist` (§4.4 "실기 한계").
- `supervisor.stale_committed_max_s` 를 어느 물리량으로 조일지 (§4.2).
