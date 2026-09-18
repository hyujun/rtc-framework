# L7 — Supervisor: 상태 머신, 접촉 판정, 감속, abort

- 브랜치: `feat/catching-L7-supervisor`
- 패키지: `catching_supervisor`
- 선행: 단계 W, L1–L6
- 산출물: `catch_supervisor.hpp`, `contact_detector.hpp`, `decel_target.hpp`, 전이 로그 레코드

---

## 1. 범위 / 비범위

범위:
- RT 루프에서 포구 임무의 모드를 결정한다.
- plan 수락·동결, 손 시퀀서 구동, 포구 후 감속, 결과 판정(포획·실패), 안전 abort와 복귀를 맡는다.

비범위:
- 계획 계산(L3)
- 기준 생성 수식(L4)
- 관절 한계 처리(L5)
- UR 드라이버 자체의 보호 정지(드라이버·로봇 제어기 소관, 감시만 함)

## 2. 코드 확인 게이트

| ID | 확인 항목 | 기록 |
|---|---|---|
| G7-1 | RTC 프레임워크의 기존 상태 머신·모드 전환 규약, 컨트롤러 활성/비활성 시 명령 유지 방식 | TBD-RTC-18 |
| G7-2 | UR 드라이버의 speed scaling 상태 인터페이스 이름과 의미 ([R11]: 드라이버는 teach pendant 속도 슬라이더·보호 정지를 speed scaling으로 반영) | TBD-ARM-03 |
| G7-3 | 지문 센서 잡음 수준, 주기, 스탬프 (시뮬레이션·실기) | TBD-HAND-03 |
| G7-4 | PTP 동기 상태 확인 방법 | TBD-NET-01 |

## 3. 참고자료

[R1] 포구 직후 충격과 순응의 역할, [R3] 인터셉트 후 선형 감속(원 논문의 임시방편, 본 layer에서 정식화 — 원문 확인: "the velocity of the robot is linearly reduced during the post-interception period (0.3 s)"), [R8] 임계값 설계, [R16] 충격량 분배·강성 최적화, [R18] 충격 순간 기준 궤적 처리(reference spreading), [R19] 접선 컴플라이언스.

## 4. 수학적 이론

### 4.1 상태 머신

| 상태 | 진입 조건 | 주요 동작 | 이탈 |
|---|---|---|---|
| `IDLE` | 활성화 직후 | 기준 = 현재 자세 유지 | 파라미터·시계·자세 준비 완료 → `ARMED` |
| `ARMED` | 준비 완료 | 대기 자세 유지(L4 §5.3), 손 `Open` | 유효 궤적 수신(형식 검사 통과 + not stale + 지평 여유) → `TRACKING` / §4.5 조건 위반 → `IDLE` |
| `TRACKING` | 유효 궤적 수신 | 기준 유지, plan 대기 | 유효 plan → `APPROACH` / 트랙 epoch 변경·stale → `ARMED` |
| `APPROACH` | 유효 plan | L4 추종, plan 교체 허용 | $t_c-t\le T_{freeze}$ → `COMMITTED` / 실패 조건 → `RETREAT` |
| `COMMITTED` | 동결 | plan 교체 금지(γ 하향만 허용, §4.6), 손 `Preshape` | $t\ge t_{cmd}$ → `CLOSING` / 치명 조건 → `ABORT_SAFE` |
| `CLOSING` | 폐쇄 명령 | 손 `Close`, γ 하향 계속 허용 | $t_{lead}\ge t_c$ → `DECEL` / 치명 조건 → `ABORT_SAFE` |
| `DECEL` | $t\ge t_c$ | 가상 감속 대상 추종(§4.3), 접촉 판정 | 정지 → `HOLD` |
| `HOLD` | 정지 | 유지, 결과 판정 확정 | $T_{hold}$ 경과 → `RETREAT` |
| `RETREAT` | 종료·실패 | 복귀 기준(L4 §5.3), 결과에 따라 손 유지/개방 | 복귀 완료 → `ARMED` |
| `ABORT_SAFE` | 치명 조건(상태 무관) | 즉시 감속(§4.3, 현재 속도 기준) 후 복귀 | → `RETREAT` |
| `FAULT` | `QP_FAILED` 연속 $N_{qp}$회, 또는 `ABORT_SAFE` 중 재차 치명 조건 | 명령 유지, 컨트롤러 비활성 요청 | 수동 |

**전이는 (상태 × 사유) 행렬로 구현한다 `[권장]`.** 위 표와 §4.2 표는 사람이 읽는 형태이고, 코드는 둘을 합친 행렬 하나를 단일 출처로 삼는다(L7.1). 기동 시 완전성을 검사한다 — 모든 상태에 진입·이탈이 최소 1개씩 있고, 모든 `Reason` 이 최소 한 칸에서 쓰이며, 미정의 칸이 없어야 한다(G7-A). v0.2 표에는 `FAULT` 로 가는 사유가 없어 도달 불가 상태였고, `CLOSING → ABORT_SAFE` 전이가 빠져 있었다.

`ABORT_SAFE` 의 진입 조건을 "동결 후"에서 "상태 무관"으로 넓혔다. `ARMED` 에서 `SPEED_SCALING` 이 발생하면 v0.2 규칙으로는 갈 곳이 없었다(§4.5 조건 5 때문에 `ARMED` 로도 못 돌아온다). 그런 경우는 `IDLE` 로 내려가 조건이 회복되기를 기다린다.

**감속은 시각 기준으로 시작한다.** 실기 접촉 신호가 지문 센서뿐이라, 공이 손바닥에 먼저 닿으면 손가락이 닫히기 전까지 검출이 늦을 수 있다. 따라서 `DECEL` 진입은 $t_c$(선행 보상 포함)로 하고, 지문 센서는 결과 판정과 abort에만 쓴다 `[권장]`.

### 4.2 abort·실패 사유

| 코드 | 조건 | 발생 가능 상태 | 처리 |
|---|---|---|---|
| `BALL_STALE` | L1 stale (나이 초과 또는 지평 소진) | `APPROACH` | `RETREAT` |
| `BALL_STALE_COMMITTED` | L1 stale | `COMMITTED`, `CLOSING` | 계속 진행 (동결 plan으로 포구 시도), 기록 |
| `TRACK_CHANGED` | L1 트랙 epoch 변경 (§L1 4.4) | `TRACKING`, `APPROACH` | `ARMED`/`RETREAT`. `PointCloud2`에 트랙 상태가 없어 `STATUS_LOST`를 이것으로 대체 |
| `HORIZON_EXTRAP` | L2 `after_horizon=true` (지평 **뒤**로 외삽) | 전 구간 | `APPROACH`면 `RETREAT`, 동결 후면 기록 후 계속 |
| `PRED_INCONSISTENT` | L1 예측 일관성 지표 $\bar\nu$ 가 임계 초과 (§L1 4.6) | `TRACKING`, `APPROACH` | `RETREAT`. 동결 후에는 기록만 |
| `QP_FAILED` | L5 `solver_status != 0` | 전 구간 | `ABORT_SAFE`. 연속 $N_{qp}$회면 `FAULT` |
| `PLAN_INVALID` | plan 무효 | `APPROACH` | `RETREAT` |
| `GAMMA_DERATED` | L4 포화 또는 포화 임박, γ 하향 여지 있음 | `APPROACH`, `COMMITTED`, `CLOSING` | γ 하향 후 계속 진행, 기록 (§4.6) |
| `SAT_NEAR_TC` | L4 포화, $t_c-t\le T_{sat}$, **γ 하향으로도 해소 안 됨** | `APPROACH`, `COMMITTED` | `RETREAT` / `ABORT_SAFE` |
| `JOINT_CONFLICT` | L5 `bound_conflict` | 전 구간 | `ABORT_SAFE` |
| `TRACK_ERR` | $\Vert q-q_c(t-T_{arm})\Vert>$ 임계 | 전 구간 | `ABORT_SAFE` |
| `SPEED_SCALING` | speed scaling ≠ 1 (G7-2) | 전 구간 | `ABORT_SAFE` (타이밍 전제 붕괴) |
| `CLOCK_UNHEALTHY` | PTP 임계 초과 | 전 구간 | `IDLE`·`ARMED`에서는 진입 거부, 운행 중 발생 시 `ABORT_SAFE` |
| `PARAMS_TBD` | L0 검증 실패 | `IDLE` | 진입 거부 |
| `HAND_TIMEOUT` | L6 폐쇄 타임아웃 | `CLOSING`, `DECEL` | 기록, 계속 |
| `TIP_STALE` | 지문 센서 stale (실기) | `COMMITTED` 이후 | 판정 불가로 기록 |

`BALL_STALE_COMMITTED`는 설계 선택이다. 동결 후에는 짧은 누락으로 포기하는 것보다 동결 plan으로 진행하는 편이 안전하다고 본다. 다만 stale 지속 시간이 상한을 넘으면 `ABORT_SAFE`로 간다(YAML). 사용자 검토 대상이다.

### 4.3 가상 감속 대상 `[논문 외 유도]`

`DECEL` 진입 시각 $t_s$의 기준 상태 $(x_s,\dot x_s)$에서 시작한다. $\hat u_s=\dot x_s/\Vert\dot x_s\Vert$, $\tau=t-t_s$, $\tau_s=\Vert\dot x_s\Vert/a_{dec}$.

$$p_v(\tau)=x_s+\dot x_s\tau-\tfrac12a_{dec}\hat u_s\tau^2,\quad v_v(\tau)=\dot x_s-a_{dec}\hat u_s\tau,\quad a_v=-a_{dec}\hat u_s\qquad(\tau\le\tau_s)$$

$\tau>\tau_s$이면 $p_v=x_s+\tfrac12\Vert\dot x_s\Vert\tau_s\hat u_s$, $v_v=a_v=0$.

L4에 이 대상을 넣고 γ를 상수 1로 바꾼다($\dot\gamma=\ddot\gamma=0$). 전환 직후 오차는

$$e=x_s-p_v(0)=0,\qquad \dot e=\dot x_s-v_v(0)=0$$

로 **정확히 0**이다. "연속"이 아니라 "0"이라는 점에 주의한다 — 전환 직전의 $e,\dot e$ 는 DS 자체의 잔여 수렴오차만큼 0이 아니므로(실측 $\Vert e^-\Vert\approx1.1$ mm, $\Vert\dot e^-\Vert\approx9.4$ mm/s, §L4 4.9의 $\epsilon_{conv}$ 와 같은 값) 오차 자체는 그만큼 점프한다. **정확히 연속인 것은 기준 상태 $(x,\dot x)$** 이고, 설계상 필요한 것은 그쪽이다. 기준 가속도는 불연속이다(jerk 무한). 필요하면 $a_{dec}$를 짧은 램프로 올린다(YAML).

**층간 제약: $a_{dec}\le$ `reference.a_max`.** $e^+=\dot e^+=0$ 이므로 전환 직후 $u_{des}=a_v$ 가 되어 크기가 정확히 $a_{dec}$ 다(실측: 0.08 → 15.0 m/s²로 도약). $a_{dec}$ 가 L4 가속 한계보다 크면 `DECEL` 첫 틱부터 포화가 걸린다. L0 파라미터 검증기가 이 관계를 검사한다.

정지거리 $\Vert\dot x_s\Vert^2/(2a_{dec})$는 L3 §4.9의 예약값($\dot x_s\approx\gamma_f v_c$)과 같다. 두 layer가 같은 `a_dec` 키를 쓴다.

`ABORT_SAFE`도 같은 식을 현재 기준 상태에서 적용한다.

### 4.4 접촉 판정

센서 $i$의 바이어스 $b_i$는 **`ARMED`/`TRACKING` 구간(손 `Open` 정지 중)** 의 이동평균으로 추정한다. 잡음 표준편차 $\hat\sigma_i$도 같은 창에서 구한다. v0.2는 창을 `COMMITTED` 구간으로 잡았는데 그 길이가 $T_{freeze}-T_{close,tot}\approx T_{arm}+T_{margin}$(수십 ms)뿐이라, 센서 주기 100 Hz면 표본 2–5개로 $\hat\sigma_i$ 를 추정하게 된다. 창은 `RETREAT → ARMED` 에서 비운다(§4.8).

$$f_i(t)=\Vert F_i(t)-b_i\Vert,\qquad c_i(t)=\mathbb 1\big[f_i>\max(f_{\min},\,k_\sigma\hat\sigma_i)\big]$$

$N_{deb}$개 연속 샘플이 참이면 센서 $i$ 접촉으로 확정한다.

결과 판정(`HOLD` 종료 시):
- **포획:** 판정 창 $[t_{cmd},\,t_c+T_{conf}]$ 안에 접촉 센서 수가 $m_{\min}$ 이상이었고, `HOLD` 종료 시점에도 $m_{\min}$ 이상이 유지되는 경우.
- **실패:** 판정 창 안에 접촉이 없는 경우.
- **미확정:** 센서 stale.

오경보 확률은 센서 잡음 분포에 의존한다. $k_\sigma$는 시뮬레이션·실기 잡음 측정 후 정한다(가우시안 가정이면 $k_\sigma=3$에서 단측 약 0.13%, 등급 a).

### 4.5 준비(ARMED) 조건

1. L0 파라미터 검증 통과(TBD 없음)
2. 시계 건강(실기)
3. L5 `lead_enable=true`이면 `T_arm` 확정
4. 로봇이 대기 자세 허용오차 안에 있음
5. speed scaling = 1(실기)
6. 손 `Open` 완료

### 4.6 γ 하향 (포화 대응 1차 수단) `[권장]`

`APPROACH` 이후 L4가 포화 플래그를 올리면(또는 $\Vert u_{des}\Vert>\eta_{sat}a_{\max}$ 로 임박하면) **abort하기 전에 먼저 γ를 낮춘다.**

```
if (ref_out->saturated || ref_out->u_des.norm() > eta_sat * a_max) {
  if (derate_count_ >= cfg.max_derates || now - last_derate_ < cfg.min_interval) {
    /* 억제 구간: 아무것도 하지 않는다 */
  } else if (trans.derateJump(*target, t_lead) > cfg.ed_jump_max) {
    // 점프가 큰 시점(램프 중앙)이다 → 목표 하향 대신 현재 값 동결만 시도
    (void)trans.derateGamma(t_lead, current_gamma - 1e-9, cfg.ramp);   // -> Frozen
    reason = Reason::GammaDerated; ++derate_count_; last_derate_ = now;
  } else {
    const double gf_new = std::max(plan->gamma_min,
                                   trans.gamma().gf - cfg.derate_step);   // **목표 기준**
    const auto r = trans.derateGamma(t_lead, gf_new, cfg.ramp);
    if (r != ref::DerateResult::Rejected) {        // Applied 와 Frozen 모두 완화다
      reason = Reason::GammaDerated; ++derate_count_; last_derate_ = now;
    } else if (t_c - t_lead <= cfg.T_sat_guard) {  // 더 내릴 여유가 없다
      reason = Reason::SatNearTc;  -> RETREAT (APPROACH) / ABORT_SAFE (COMMITTED, CLOSING)
    }
  }
}
```

근거와 연속성 분석은 L3 §4.7, L4 §5.2.1에 있다. 요점만 다시 적는다.

- $p_c$, $t_c$ 는 동결 유지. $\gamma_f$ 만 내린다.
- **요청은 목표 $\gamma_f$ 기준이고, `Rejected` 만 "여유 없음"이다.** `Frozen`($\dot\gamma,\ddot\gamma$ 를 0으로)도 유효한 완화이며 램프 중앙에서는 이것이 $\Vert u\Vert$ 감소의 지배 요인이다. `false` 하나로 판정하면 하향 경로가 램프 상승 구간에서 통째로 죽는다(L4 §5.2.1).
- $\gamma(t)$, $e$ 는 연속. $\dot e$ 만 $\vert\dot\gamma(t)\vert\Vert\xi^O(t)\Vert$ 만큼 점프한다.
- **점프는 시간에 단조가 아니다.** 램프 중앙에서 봉우리를 이루므로(L4 §5.2.1 표, 실측 최대 2.04 at $t=0.56$) `derateJump()` 로 미리 계산해 `ed_jump_max` 와 비교하고, 초과하면 동결로 대신한다. 안전해지는 것은 $t\to t_c$ 극한이지 "늦을수록"이 아니다.
- 필요 가속도는 $\gamma_f$ 에 거의 선형(L3 §4.8 표).
- 하한은 $\gamma_{\min}$(손 폐쇄 제약, L3 §4.5). 그 아래가 필요하면 abort다.
- 하향 횟수 상한·최소 간격을 둔다(`supervisor.gamma.max_derates`, `min_interval`). **카운터는 `RETREAT → ARMED` 에서 리셋한다**(§4.8) — 안 하면 두 번째 시행부터 하향 경로가 죽는다.

이 경로가 없으면 포화 대응이 abort뿐인데, 포화는 $t_c$ 직전에 몰리므로(L4 §4.8) abort하기에 가장 나쁜 시점이다. [R3]이 매 스텝 $J(q)\dot q_{\max}$ 제약 아래 γ를 재최적화하는 것의 축소판이다.

### 4.7 충격량 예산 `[논문 외 유도]` `[TBD-IMP-01]`

v0.1에는 이 절이 없었다. 본 시스템은 UR5e를 `servoj` position 인터페이스로 구동하므로 **접촉 순간의 임피던스가 사실상 위치 서보 강성**이고, 순응 요소가 없다. [R16]의 문제의식이 그대로 적용된다 — 빠른 물체와 접근하는 로봇 사이의 속도 불일치가 큰 충격력을 만들고, 접촉 불안정과 손상으로 이어진다.

본 설계의 유일한 완화책은 상대속도를 줄이는 soft catch다.

$$\Delta p=m_{ball}\,(1-\gamma_f)\Vert v(t_c)\Vert$$

문제는 γ가 작을 수밖에 없는 구간이 넓다는 것이다(L3 §4.5, 마스터 §4.1). $\gamma_f=0.25$, $\Vert v\Vert=2$ m/s면 상대속도가 1.5 m/s이고, 이 운동량을 지문 센서만 달린 손가락과 위치 서보 팔이 받는다.

**계획·검증 단계에서 다음을 산출하고 기록한다.**

1. **충격량과 손가락 토크.** $\Delta p$ 를 접촉 시간 $\Delta t_{imp}$ 로 나눈 평균 힘 $\bar F=\Delta p/\Delta t_{imp}$ 와, 그것이 만드는 관절 토크를 P1b 모터 한계(1.5 Nm, G6-3)와 비교한다. $\Delta t_{imp}$ 는 시뮬레이션 접촉 참값에서 측정한다.
2. **팔 쪽 반력.** `servoj` 명령은 접촉 중에도 계속 진행하므로, 공이 손 안에서 감속되는 동안의 반력은 구조와 관절이 받는다. UR5e 보호 정지 임계 대비 어디인지 확인한다(`[HW-P1B]`, 저속부터).
3. **충격 후 CLIK 괴리.** 충격으로 $q$ 가 $q_c$ 에서 벌어지면 L7 `TRACK_ERR` 가 오동작할 수 있다. `track_err_abort` 를 정할 때 충격 구간을 제외하거나 임계를 시간 가변으로 둔다.
4. **반발.** L3 §4.5의 $d_{eff}=d(1+1/e)$ 는 법선 단일 충돌 모델이다. [R19]가 다루는 접선 컴플라이언스는 무시한다는 가정을 명시한다.

**계획에 거는 게이트.** $\Delta p\le\Delta p_{\max}$ 를 L3 후보 게이트에 추가한다(`TBD-IMP-01` 확정 후). 이 게이트가 γ 창의 하한을 실질적으로 끌어올린다.

**본 설계가 하지 않는 것.** [R16]의 강성·접촉력 동시 최적화와 접촉점 선택, [R18]의 reference spreading(충격 순간 기준 궤적 불연속 처리)은 **범위 밖**이다. 둘 다 토크 또는 임피던스 인터페이스를 전제하는데 UR5e는 position으로 확정돼 있다(마스터 §1.1). 저속 구간에서 성공률이 확보되지 않으면 이 제약을 재검토해야 한다 — 그때의 선택지가 위 두 문헌이다.


### 4.8 재무장 리셋 목록 `[권장]`

`RETREAT → ARMED` 전이에서 **다음을 전부 초기화한다.** 하나라도 빠지면 직전 시행의 상태가 남아 두 번째 투척이 다르게 동작한다. v0.2는 이 목록이 없었고, §9 시나리오가 전부 단발 시행이라 게이트에서도 잡히지 않았다.

| 대상 | 리셋 내용 | 빠뜨렸을 때 |
|---|---|---|
| `PlanSnapshot` 박스 | `valid=false` 로 무효화하고 `last_consumed_plan_id` 기록 | 옛 plan 의 $t_c$ 가 이미 과거라 `TRACKING→…→DECEL` 을 몇 틱에 통과하며 엉뚱한 곳에서 손을 닫는다 |
| L4 `SoftCatchTranslation` | `reset(x_meas, 0)` — $p_c$ 와 γ 프로파일까지 (L4 §5.3) | 직전 포구점으로 복귀하고, 하향된 γ가 남는다 |
| L2 hint 커서 | 0 | 정확성은 이진 탐색이 지키지만 틱 비용이 흔들린다 |
| L5 `qd_prev` | 0 (`resetState` 가 함께 수행) | 첫 틱 가속 경계가 옛 속도 기준이라 `bound_conflict` 오abort |
| L6 시퀀서 | `Open` 위상, 진행률 창 | 폐쇄 명령 시각이 어긋난다 |
| L7 접촉 바이어스·잡음 창 | 원형 버퍼 비우기 | 직전 시행의 접촉력이 바이어스에 섞인다 |
| L7 γ 하향 카운터 | `derate_count=0`, `last_derate=-∞` | 두 번째 시행부터 하향 경로가 죽는다(§4.6) |
| L7 결과·사유 | `Outcome::None`, `Reason::None` | 진단 오염 |

그리고 L7이 plan 을 받아들일 때 **세 조건을 모두** 본다: `valid`, `plan_id != last_consumed_plan_id`, `t_ref + t_c_rel > now + T_{lead,min}`. 마지막 조건이 과거 plan 을 걸러낸다.

§9 시나리오에 **"연속 2회 투척"** 과 **"abort 직후 재투척"** 을 넣어야 이 항목들이 게이트에서 검증된다.

## 5. C++ 구현

### 5.1 인터페이스

```cpp
enum class Mode : std::uint8_t { Idle, Armed, Tracking, Approach, Committed, Closing, Decel, Hold, Retreat, AbortSafe, Fault };
enum class Reason : std::uint8_t { None, BallStale, BallStaleCommitted, TrackChanged, HorizonExtrap,
                                   PlanInvalid, GammaDerated, SatNearTc,
                                   JointConflict, TrackErr, SpeedScaling, ClockUnhealthy, ParamsTbd,
                                   HandTimeout, TipStale };
enum class Outcome : std::uint8_t { None, Captured, Missed, Undetermined, Aborted };

struct SupervisorInputs {                  // RT 틱마다 구성
  Nanoseconds now;
  const TrajView* traj;                    // L1 (나이·stale·지평 소진)
  const traj::Eval* target;                // L2 샘플러 출력 (extrapolated 플래그 포함)
  const PlanSnapshot* plan;                // L1 브리지 경유 L3
  const ref::TranslationOutput* ref_out;   // L4 직전 틱
  const joint::JointCmdOutput* jc;         // L5 직전 틱 (solver_status 포함)
  const RobotSnapshot* robot;              // L1
  const joint::VecN* q_cmd_delayed;        // q_c(t - T_arm). 고정 길이 지연 링에서 (L7 §4.2 TRACK_ERR)
  double t_real, t_lead;                   // 두 시간축 (§5.2). t_lead = t_real + T_arm
  bool clock_ok, params_ok, speed_scaling_ok;
};

struct SupervisorOutputs {
  Mode mode;
  ref::TargetState target;                 // L4 대상 (실제 공 / 가상 감속 / 정지 목표)
  ref::GammaProfile gamma;                 // 모드별
  Eigen::Vector3d p_c, a_d;
  bool set_intercept;                      // 이번 틱에 L4 setIntercept 호출 여부
  bool use_retreat;
  Reason reason; Outcome outcome;
};

class CatchSupervisor {
 public:
  [[nodiscard]] SupervisorOutputs update(const SupervisorInputs& in, HandSequencer& hand) noexcept;
};
```

### 5.2 구현 규칙

- `switch`로 상태를 처리하고, 전이는 한 틱에 최대 1회만 일으킨다.
- 전이 시 고정 크기 레코드 `{now, from, to, reason, plan_id, track_id}`를 SPSC에 넣는다(L8 기록).
- **시각 비교는 두 축을 구분해 쓴다.** 모든 상대시각의 원점은 `PlanSnapshot::t_ref`(= 계획에 쓴 궤적 메시지의 `header.stamp`)로 통일하되, 축은 둘이다(마스터 §3).
  - **선행축** $t_{lead}=t_{real}+T_{arm}$: 궤적 샘플링, γ 프로파일, $t_c$, `setIntercept`/`derateGamma`, `CLOSING→DECEL` 전환.
  - **실제시각축** $t_{real}=10^{-9}(t_{now}-t_{ref})$: 손 명령 시각 $t_{cmd}$(L3 §4.11), 접촉 판정 창, stale·나이 판정.
  v0.2는 "하나로 통일한다"고 적었으나 §4.1 표 안에서 이미 두 축이 섞여 있어 실행 불가능한 요구였다. 두 값을 서로 다른 타입으로 선언해 혼용을 막는다(마스터 §3 각속도 규약과 같은 방식) `[권장]`.
- `TRACK_ERR` 의 $q_c(t-T_{arm})$ 는 고정 길이 지연 링(`kMaxArmDof × ceil(T_arm_max/h)`)에서 꺼낸다. `RobotSnapshot::q_cmd` 는 1틱 전 값이라 쓸 수 없다.
- 지문 센서 바이어스·잡음 추정은 고정 길이 원형 버퍼로 한다(할당 없음).

## 6. YAML 파라미터

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `supervisor.T_sat_guard` | double | s | 0.3 | 0–1 | §4.2 `SAT_NEAR_TC` |
| `supervisor.gamma.eta_sat` | double | – | 0.95 | 0.5–1 | §4.6 포화 임박 판정 ($\Vert u_{des}\Vert>\eta_{sat}a_{\max}$) |
| `supervisor.gamma.max_derates` | int | – | 4 | 0–20 | §4.6 시행당 하향 횟수 상한 |
| `supervisor.gamma.min_interval` | double | s | 0.05 | 0.01–0.5 | §4.6 하향 간 최소 간격 |
| `supervisor.gamma.ramp` | double | s | 0.05 | 0.01–0.3 | §4.6. L4 `reference.gamma_derate.ramp`와 같은 값 |
| `supervisor.gamma.derate_step` | double | – | 0.1 | 0.02–0.3 | §4.6 1회 하향 폭. **L3 §6 `planner.gamma.derate_step`과 같은 키** — §5의 `cfg.derate_step`이 이 값이다 |
| `supervisor.impact.dp_max` | double | kg·m/s | `TBD` | >0 | §4.7 `TBD-IMP-01` |
| `supervisor.stale_committed_max` | double | s | `TBD` | ≥0 | §4.2 설계 선택 |
| `supervisor.track_err_abort` | double | rad | `TBD` | >0 | **단일 원천.** L5 §6의 `joint_cmd.track_err_abort`는 같은 키를 가리킨다 |
| `supervisor.decel.a_dec` | double | m/s² | `TBD` | >0 | L3 `planner.stop.a_dec`와 동일 값 |
| `supervisor.decel.ramp_time` | double | s | 0.0 | 0–0.1 | §4.3 |
| `supervisor.contact.f_min` | double | N | `TBD` | >0 | G7-3 |
| `supervisor.contact.k_sigma` | double | – | 3.0 | 2–6 | §4.4 |
| `supervisor.contact.n_debounce` | int | – | 3 | 1–20 | 센서 주기 의존 |
| `supervisor.contact.m_min` | int | – | `TBD` | 1–4 | 손 형상 |
| `supervisor.contact.T_confirm` | double | s | 0.2 | 0–1 | §4.4 |
| `supervisor.ready.pose_tol` | double | rad | 0.02 | – | §4.5 |
| `supervisor.ready.wait_pose` | double[n] | rad | `TBD` | – | 로봇별 |

## 7. 단위 기술 구현 순서

- **L7.1** 상태·사유·결과 enum, 전이 표를 코드 테이블로 고정.
- **L7.2** 가상 감속 대상 + 연속성 테스트.
- **L7.3** 접촉 판정기 + 합성 잡음 오경보 테스트.
- **L7.4** γ 하향 경로(§4.6) + 연속성·상한 테스트 (L4 `derateGamma` 호출).
- **L7.5** 슈퍼바이저 본체 + 시나리오 테스트(§9).
- **L7.6** 충격량 예산(§4.7): 시뮬레이션 접촉 참값으로 $\Delta t_{imp}$, $\bar F$, 관절 토크 산출 → `TBD-IMP-01` 확정 → L3 게이트 연결.
- **L7.7** 실기 전용 조건 연결: speed scaling, 시계, 지문 센서 stale.

## 8. 디버깅 방법

- 타임라인 그래프: 모드 띠, $t_c$·$t_{cmd}$ 수직선, $\Vert e\Vert$, 포화 플래그, 손 $\rho(t)$, 센서 $f_i$와 임계.
- 예상치 못한 `RETREAT`: 전이 로그의 사유 코드로 추적한다.
- 포획했는데 `Missed`: 판정 창과 센서 스탬프 정렬(실기 async 센서 지연)을 확인한다.
- 감속 중 흔들림: `a_dec` 값과 L5 가속 한계의 정합, 램프 적용 여부를 확인한다.
- γ 하향이 자주 발생: L3 rollout의 여유율(`eta_a`, `eta_v`)이 낮거나 $T_w$ 가 짧은지 확인한다. 하향이 상한까지 가면 계획 단계 γ가 애초에 과대하다는 뜻이다.
- 접촉 직후 `TRACK_ERR` abort: 충격 구간에서 임계를 완화했는지 확인한다(§4.7-3).

## 9. 검증 방법과 합격 게이트

시나리오 테스트(모의 입력으로 RT 슈퍼바이저 단독 실행, 기대 상태열 비교):

| 시나리오 | 기대 상태열 |
|---|---|
| 정상 | Armed → Tracking → Approach → Committed → Closing → Decel → Hold → Retreat → Armed, `Captured` |
| 공 놓침 | … → Decel → Hold → Retreat, `Missed` |
| 동결 전 stale | … → Approach → Retreat, `BallStale` |
| 동결 후 짧은 stale | … → Committed → Closing → …, `BallStaleCommitted` 기록 |
| 동결 후 긴 stale | … → Committed → AbortSafe → Retreat |
| 포화, γ 하향으로 해소 | … → Committed(γ 하향 기록) → Closing → Decel → …, `GammaDerated` |
| $t_c$ 직전 포화, γ 하향 불가 | … → Committed → AbortSafe, `SatNearTc` |
| 관절 경계 충돌 | 임의 상태 → AbortSafe, `JointConflict` |
| speed scaling < 1 | 임의 상태 → AbortSafe, `SpeedScaling` |
| TBD 파라미터 | Idle 유지, `ParamsTbd` |

| 게이트 | 기준 | 태그 |
|---|---|---|
| G7-A | 위 시나리오 전부 기대 상태열과 일치 | `[SIM-ANY]` |
| G7-B | 감속 전환 시 $e,\dot e$ 연속 (< 1e-9) | `[SIM-ANY]` |
| G7-B2 | γ 하향 시 $\gamma$, $e$ 연속(< 1e-12), $\dot e$ 점프가 `ed_jump_max` 이하, 하향 횟수·간격 상한 준수 | `[SIM-ANY]` |
| G7-B3 | 충격량 $\Delta p$ 기록과 시뮬레이션 접촉 참값의 최대 접촉력 상관 확인, 손가락 관절 토크가 한계 이내 | `[SIM-P1B]` |
| G7-C | 합성 잡음에서 접촉 오경보율 기록 (임계는 사용자 결정) | `[SIM-ANY]` |
| G7-D | RT 할당 0, 틱 최악 실행시간 기록 | `[SIM-ANY]` |
| G7-E | `ur5e_p1b` 시뮬레이션 폐루프에서 결과 판정과 MuJoCo 참값 일치율 기록 | `[SIM-P1B]` |
| G7-F | 실기 speed scaling·시계·센서 stale 경로 동작 확인 | `[HW-P1B]` |

## 10. 미확정 항목

TBD-RTC-18, TBD-ARM-03, TBD-HAND-03, TBD-NET-01, TBD-IMP-01(§4.7), `supervisor.stale_committed_max`, `supervisor.decel.a_dec`, `supervisor.contact.*`, `supervisor.impact.dp_max`.
