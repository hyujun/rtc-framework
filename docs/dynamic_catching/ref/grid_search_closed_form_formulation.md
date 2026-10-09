# 격자 탐색 (`search_grid`) 과 `closed_form` 구간의 수학적 정리 — `GridCatchSearch` 와 soft-catch 법칙

- 이 문서는 **지금 구현된** 포구 후보 탐색 (`planner.search.grid.*`, `GridCatchSearch`) 과 `closed_form` planner 의 팔 기준 법칙 (soft-catch DS → 등감속 정지) 을 한 벌의 수식으로 적는다. [mpc_multiframe_clik_formulation.md](mpc_multiframe_clik_formulation.md) 가 `mpc` 구간 계획기에 대해, [ball_catching_inverse_dynamics_mpc.md](ball_catching_inverse_dynamics_mpc.md) §17 이 `mpc_docking` · NLP 탐색에 대해 하는 일을 이 둘에 대해 한다.
- 식은 코드에서 되읽은 것이다 (`rtc_controllers/{include,src}/rtc_controllers/catching/` 의 `grid_catch_search` · `catch_pose_ik` · `rank_gates` · `time_feasibility` · `unit_speed` · `gamma_rollout` · `soft_catch` · `decel_target`, 그리고 `integrated_bringup/src/controllers/catching/controller.cpp` 의 RT 경로). 설계 문서와 다른 곳은 코드의 것이 이 문서의 식이다.
- 층별 문서는 그대로 SSoT 다 — 결정 ID · YAML 키의 범위와 검증 · 게이트 · 디버깅은 [L3_planner.md](L3_planner.md) (탐색), [L4_reference.md](L4_reference.md) (기준 생성), [L7_supervisor.md](L7_supervisor.md) (감속 · 상태 머신) 이 갖는다. 이 문서는 그 셋에 흩어진 수학을 **한 문제의 순서**로 다시 세우고, 식이 코드의 어느 분기에 대응하는지를 적는다. 값 (출하 수치) 은 적지 않는다 — YAML 과 그 머리 주석이 갖는다.
- 수식 표기 규칙은 formulation 문서의 "수식 표기 규칙" 을 그대로 따른다.
- 결정 ID (`D-27` · `MD-46` …) 의 뜻은 [ID_INDEX.md](../ID_INDEX.md).

---

## 0. 범위 · 가정 · 표기

### 0.1 구현 범위

| 상태 | 내용 | 절 |
|---|---|---|
| 구현됨 | vision 샘플 격자 위의 1 차원 시각 탐색 + 후보별 5 행 IK (제약 QP 과제 스텝 + 영공간 $\log w_5$ 상승) + 닫힌식 게이트 (도달시간 · γ 창 · 오차 예산) + soft-catch DS rollout 으로 $(\gamma_f, T_w)$ 선택 + 가중 점수 | §2 |
| 구현됨 | 판정 게이트 / 순위 게이트의 분리 (D-27), IK 예산과 사전 점수 순서 (R-2), 교체 히스테리시스와 $u_{des}$ 계단 상한, 동결 | §2.2, §2.11 |
| 구현됨 | `closed_form` 의 RT 법칙 — 오차 좌표 soft-catch DS (LTI, $\zeta=1$), 5 차 γ 램프, 반암시적 Euler, 방사형 포화, DECEL 의 등감속 가상 대상, HOLD | §3 |
| 구현됨 | 모든 segment mode 가 공유하는 탐색 — `mpc` · `mpc_docking` 에서는 이 탐색의 plan 을 구간 계획기가 받고, RT 가 plan 을 따르는 동안에도 탐색은 돌지만 결과는 게시되지 않는다 (MD-46, §1.4) | §1.4, §4 |
| 구현하지 않음 | 충격량 게이트, γ derate, LPV $A_i(\theta)$, $(q^\ast, t_c)$ 동시 NLP, 격자 사이 보간 후보, $\sigma_\ell$ 로 하는 abort, $a_{dec}$ 램프, 교체 때 램프 이어 붙이기 | §6 |

### 0.2 시간축 (L0 §4.5, D-2)

모든 시각은 절대 steady ns 다. 세 타입 — $t_k$ · $t_c$ · $t_{cmd}$ 는 공의 물리 시각 `BallTime`, $now$ 는 실측 `NowReal`, $now_{lead}=now+T_{arm}$ 은 `NowLead` — 이고 판정마다 비교하는 '지금' 이 정해져 있다. 이 문서가 쓰는 것은 다음이다.

| 판정 | '지금' |
|---|---|
| 후보 창 (§2.1), commit 선행 (§2.9), 동결 · commit (§2.11, §3.8) | $now$ |
| 도달시간 (§2.5), rollout · γ 프로파일 · 궤적 샘플링 · 기준 생성 (§2.7, §3), DECEL 진입 (§3.6) | $now_{lead}$ |

상대 초는 수치 코어 경계에서만 만든다 — γ 프로파일의 $t$ 와 램프 시각 $t_0$ · $t_1$ 은 한 원점 (plan 의 램프 시작) 에서 잰 초다 (`ProfileSeconds` · `MakeGammaProfile`).

### 0.3 기호

| 기호 | 뜻 |
|---|---|
| $\hat p(t)$, $\hat v(t)$, $\hat a(t)$, $\Sigma(t)$ | vision 예측 궤적의 위치 · 속도 · 가속도와 6×6 공분산 (world). 샘플 격자 $t_k$ 위의 값이고 샘플 사이는 L2 의 5 차 Hermite 샘플러 `SampleAt` 가 보간한다 |
| $\Sigma_{pp}$, $\sigma_{\max}=\sqrt{\lambda_{\max}(\Sigma_{pp})}$ | 위치 3×3 블록 (대칭화) 과 그 최대 고윳값의 제곱근 |
| $p_c$, $v_c$, $\Vert v\Vert$, $\hat d=v_c/\Vert v_c\Vert$, $a_d=-\hat d$ | 후보의 포구점 · 공 속도 · 속력 · 진행 방향 (단위벡터) · 목표 접근축. $\hat p_k$ · $\hat v_k$ 의 hat 은 추정값 표시이고 단위벡터는 $\hat d$ 뿐이다 |
| $\kappa_\sigma$, $n_\sigma$ | 불확실성 게이트의 배율, 오차 예산의 시그마 배수 |
| $\eta_a$, $\epsilon_{term}$ | rollout 의 요구 가속 여유율, $t_c$ 잔여 오차 한계 |
| $\Delta_J$, $\eta_{jump}$ | 교체의 점수 개선 문턱, 교체 계단 상한의 비율 |
| $n_s$, $T_{bud}$, $N_{IK,\max}$ | 격자 솎기 보폭, 한 wake 의 탐색 예산, 한 wake 의 IK 후보 수 상한 |
| $q\in\mathbb R^{n}$, $q_{\min}, q_{\max}$, $\dot q_{\max}$, $\ddot q_{\max}$ | 팔 sub-model 의 관절 (모델 순서), 위치 한계, 관절 속도 정격, D-16 가속 box |
| $q_c$, $\dot q_c$ | RT 가 보고한 현재 팔 **명령** |
| $q_n$ | IK seed = 대기 자세 (`planner.wait_pose`, `wait_pose_source: current` 면 RT 가 채택한 자세) 를 한계로 clamp 한 것 |
| $p_C(q)$, $R_{WC}(q)$, $z=R_{WC}\hat e_z$ | catch frame 의 위치 · 자세 · 손바닥 바깥 법선 (D-17) |
| $J_p^{LWA}$, $J_\omega^{L}$, $S$ | catch frame 의 병진 Jacobian (LOCAL_WORLD_ALIGNED) · 각속도 Jacobian (LOCAL), LOCAL 각속도의 $x, y$ 행만 고르는 2×3 선택행렬 |
| $J_5=[J_p^{LWA};\enspace SJ_\omega^L]$, $J_6=[J_p^{LWA};\enspace J_\omega^L]$ | 5 행 · 6 행 과제 Jacobian (팔 관절 열) |
| $w_5=\sqrt{\det J_5J_5^\top}$, $w_6=\sqrt{\det J_6J_6^\top}$ | manipulability (가중하지 않은 $J$) |
| $e_a$, $\theta$ | 접근축 회전벡터 오차와 그 크기 (L4 §4.5, `AxisAlignError`) |
| $\eta_v$, $v_{\max}$, $v_{tcp}=\eta_vv_{\max}$ | 속도 여유율 (D-9), TCP 속도 한계 (탐색 복사본 `planner.search.grid.reference.v_max`), 계획 TCP 속도 |
| $\omega$, $\zeta$, $a_{\max}$ | soft-catch DS 의 고유진동수 · 감쇠비 (1) · 가속 포화 |
| $a_{dec}$ | DECEL 등감속 크기 |
| $d_{eff}$, $r_{cap}$ | 손이 흡수할 수 있는 상대속도 × $T_{close,tot}$ (포켓 깊이가 아니다, L6 §4.5), 포획 측면 반경 (공 중심 기준) |
| $T_{arm}$, $T_{close,e2e}$, $T_{close,tot}=T_{close,e2e}+h/2$, $T_{margin}$, $T_{freeze}$, $h$ | 팔 지연, 손 폐쇄 종단 간 시간, 틱 양자화 예산을 더한 것, 도달시간 여유, 동결 창, 제어 주기 |
| $\gamma(t)$, $\gamma_0$, $\gamma_f$, $T_w$ | softness 프로파일, 램프 시작값 · 끝값, 램프 길이 |
| $x$, $\dot x$, $u$ | catch frame 병진 기준의 위치 · 속도 · 가속 (DS 의 상태와 입력) |
| $o=(p_O, v_O, a_O)$ | DS 가 따르는 대상 — 공 샘플, DECEL 에서는 가상 감속 대상 |
| $\xi^O=p_O-p_c$, $\xi=x-p_c$ | 포구점 원점의 대상 · 기준 상대위치 |

---

## 1. 문제 — 입력 · 출력 · 결정 구조

### 1.1 입력

계획기 스레드의 한 wake (`PlannerCycle::Run`, L3 §5.3) 가 탐색에 넘기는 것은

- 궤적 스냅샷 $\lbrace(t_k, \hat p_k, \hat v_k, \hat a_k)\rbrace _ {k=0}^{N-1}$ 와 같은 provenance token 의 공분산 $\lbrace\Sigma_k\rbrace$ (token 이 다르면 "공분산 모름"),
- RT 상태 `PlannerRtState` — $q_c$, $\dot q_c$, 대기 자세, 기준 생성기의 상태 $(x, \dot x, \gamma, \dot\gamma, \ddot\gamma)$ 와 돌고 있는 램프, 따르는 `plan_id`,
- 실측 '지금' $now$.

### 1.2 출력 (`PlanSnapshot`)

$$
\text{plan}=\big(t_c, p_c, v_c, a_d, q^\ast, w_5, w_6, (\gamma_0,\gamma_f,t_0,t_1), \gamma_{\min}, t_{cmd}, \sigma_c, \sigma_\ell, \Delta p_{impact}, J\big)
$$

또는 "plan 없음" + 사유 (§2.14). `mpc` 에서는 유효한 plan 을 첫 MPC 구간과 쌍으로 게시한다 (L3 §5.3) — 그 구간의 수학은 formulation 이다.

### 1.3 결정 구조 (A-4, D-27)

[R1] 의 $(q^\ast, t_c)$ 동시 NLP 대신 **vision 격자 위의 1 차원 시각 탐색 + 후보별 IK** 다. 후보 $k$ 에 계산을 싼 것부터 적용한다.

$$
\underbrace{\text{입력 유한성}\to\sigma_{\max}\to\text{사전 점수}} _ {\text{모든 후보}} \Longrightarrow \underbrace{\text{IK}\to w_5\to\dot q^u\to t_{\min}\to\gamma\text{ 창}\to\text{rollout}(\gamma_f,T_w)\to\sigma_{gap}(\gamma_f)\to J} _ {\text{사전 점수 상위 }\le N_{IK,\max}\text{ 개, 예산 안}}
$$

게이트는 두 종류다.

- **판정 게이트** — 후보를 **제거**한다: 입력 유한성 (NUM-7), IK 수렴, manipulability (D-18). 포구점이 **어디에** 있는지는 판정하지 않는다 (L3 §4.9 — §2.8).
- **순위 게이트** — 제거하지 않고 실패마다 점수에 벌점 $w_{pen}$ 을 더한다: 불확실성 · 도달시간 · γ 창 · commit 선행 · 오차 예산 · rollout (비트마스크 `RankGateBit`).

판정 통과 후보 가운데 점수 $J$ 최소를 고른다. 판정 통과 후보가 없을 때만 plan 없음이다.

### 1.4 segment mode 에서의 쓰임 (MD-46)

탐색은 `closed_form` · `mpc` · `mpc_docking` 에 공통이다 (탐색의 다른 구현 `nlp` 는 formulation §17.11 이 갖는다). `closed_form` 에서는 plan 의 $p_c$ · γ 프로파일 · $a_d$ 를 RT 가 §3 의 법칙으로 그대로 실행하고, APPROACH 동안 탐색이 계속 돌며 §2.11 로 plan 을 교체한다. `mpc` 에서는 plan 의 $t_c$ (격자의 닻) · $p_c$ · $v_c$ · $a_d$ (포구 노드 목표) · $q^\ast$ (첫 선형화 기준) 만 MPC 가 읽고, $(\gamma_f, T_w)$ 는 순위에만 쓰인다 — RT 는 DS 를 돌리지 않는다. 구간 계획기 아래에서 탐색은 RT 가 plan 을 따르는 동안에도 (`planner.freeze.t_stop_plan` 까지) 돌고, 교체를 판정하면 계획기가 새 plan 을 그 첫 구간과 쌍으로 게시해 RT 가 그 구간의 node 0 에서 둘을 함께 바꾼다 (L3 §5.3, L7 §4.3a). 그 동안 도달시간의 출발 (§2.4) 은 RT 가 보고한 구간 위의 상태다. 차이의 표는 §4.3.

---

## 2. 격자 탐색 — `GridCatchSearch::Plan`

### 2.1 후보 집합과 창

후보 시각은 vision 샘플 자체다 (보간하지 않는다 — 공분산을 시각 보간할 필요가 없게). 샘플 간격 $\Delta_v=t_1-t_0$ 에 대해 격자를 솎는 보폭 $n_s=\max(1, \mathrm{round}(\Delta_{slice}/\Delta_v))$ ($\Delta_{slice}$ = `slice.dt`; 간격이 비양수 · 비유한이면 $n_s=1$) 이고,

$$
\mathcal K=\lbrace k=0, n_s, 2n_s,\dots : T_{lead,\min}\le t_k-now\le T_{lead,\max}\rbrace,\qquad
T_{lead,\min}=T_{lead,0}\enspace(\text{적혀 있으면; 아니면 }T_{freeze}),\quad T_{lead,\max}=T_{lead,1}.
$$

$T_{lead,0}$ = `slice.t_lead_min`, $T_{lead,1}$ = `slice.t_max`. 선행 $\ell_k=t_k-now$ 는 **실측 시각** 기준이다. $\mathcal K$ 가 비면 plan 없음 (`kHorizonShort`). 샘플 수와 후보 수는 `kCap` 까지다. 트랙 epoch 이 바뀐 직후 `n_settle` 개 메시지는 계획하지 않는다 (`kUncertainty`, L3 §4.4).

### 2.2 싼 항 · 사전 점수 · IK 순서와 예산

모든 $k\in\mathcal K$ 에서:

- **입력 판정.** $\hat p_k$, $\hat v_k$ 가 유한하고 $\Vert\hat v_k\Vert\ge v_{eps}$ (`ik.v_eps`) 가 아니면 제거 (`kInput` — plan 사유로는 `kInputNonFinite` 이고 속력 하한 탈락도 여기 든다). clamp 로 덮지 않는다 (NUM-7).
- **불확실성.** $\sigma_k=\sigma_{\max}(t_k)$ — 공분산이 token 불일치 · 비유한이면 모름 (NaN). 순위 게이트 실패 조건은 $\neg(\sigma_k\le\kappa_\sigma r_{cap})$ (모름 포함).
- **사전 점수.**

$$
J^{pre} _ k=\mathbf 1[\sigma_k]\enspace w_\sigma\frac{\sigma_k}{r_{cap}}+w_{late}\big(T_{lead,\max}-\ell_k\big)+\mathbf 1[\neg(\sigma_k\le\kappa_\sigma r_{cap})]\enspace w_{pen}.
$$

판정에서 살아남은 후보를 $J^{pre}$ 오름차순으로 세운다 (동률의 순서는 정해져 있지 않다 — 안정 정렬이 아니다). **따르는 plan 의 후보** — $\vert t_k-t_{c,cur}\vert\le n_s\Delta_v/2$ — 가 있으면 맨 앞으로 옮긴다 (교체 규칙이 그 후보의 판정을 필요로 하고, `max_ik` · 예산에 밀려 "불가능" 으로 읽히면 안 된다). IK 는 앞에서 $\min(\vert\text{순서}\vert, N_{IK,\max})$ 개까지만 돈다 ($N_{IK,\max}$ = `max_ik`).

**예산 (R-2).** 후보 $i\gt0$ 의 IK 를 시작하기 **전에** $\text{elapsed}+\hat c_{IK}+\hat c_{roll}\gt T_{bud}$ 이면 남은 후보를 평가하지 않고 멈춘다 (`budget_hit`). $\hat c$ 는 감쇠 최대 추정 $\hat c\leftarrow\max(c_{meas}, \hat c-\hat c/8)$ 이다 — 느린 풀이 하나가 추정을 즉시 올리고 몇 사이클에 걸쳐 풀린다. 첫 후보는 늘 돈다.

### 2.3 포구 자세 IK — `CatchPoseIk::Solve`

오프라인 catchability 지도와 런타임이 **같은 함수 · 같은 seed · 같은 키** 로 돈다 (G3-I). 호출 간 상태가 없고 QP 는 후보마다 cold start 한다 — 해가 탐색 순서에 의존하지 않게.

#### 2.3.1 목표와 오차

$$
p_c=\hat p_k,\qquad a_d=-\frac{\hat v_k}{\Vert\hat v_k\Vert},\qquad a^C=R_{WC}^\top a_d,\qquad
e_a^C=\theta\frac{\hat e_z\times a^C}{\Vert\hat e_z\times a^C\Vert},\quad \theta=\mathrm{atan2}(\Vert\hat e_z\times a^C\Vert, a^C_z).
$$

$e_a^C\perp\hat e_z$ 이므로 $z$ 성분이 0 이고 $S$ 는 정보를 버리지 않는다. $\exp([e_a^C] _ \times)\hat e_z=a^C$ 가 정확히 성립한다 (L4 §4.5). 과제 잔차와 가중은

$$
e=[p_c-p_C(q);\enspace Se_a^C]\in\mathbb R^5,\qquad W=\mathrm{diag}(1,1,1,\rho,\rho) [\rho: \text{m/rad}],
$$

$\rho$ 는 $J$ 와 $e$ **양쪽**에 곱하는 과제 가중이다 (D-25) — 잔차에만 곱하면 회전 행이 m 단위가 되어 $\rho$ 가 스텝 이득으로 작동하고 "한 번에 정렬" 이 깨진다.

#### 2.3.2 갱신 법칙 (D-25 · D-26)

반복 1 회는 두 관절속도의 합을 $\Delta t=1$ 로 적분한다.

$$
\dot q_{clik}=\arg\min_{\dot q} \tfrac12\Vert W(J_5\dot q-e)\Vert^2+\tfrac12\mu\Vert\dot q\Vert^2
\quad\text{s.t.}\quad \max(q_{\min}-q, -\Delta_{\max})\le\dot q\le\min(q_{\max}-q, \Delta_{\max})
$$

$$
\dot q_{sec}=k_w\nabla_q\log w_5(q)+K_n(q_n-q),\qquad \dot q_n=N\dot q_{sec},\quad N=I-(WJ_5)^\dagger_\lambda(WJ_5),\qquad
\boxed{\dot q_d=\dot q_{clik}+\dot q_n}
$$

$$
\dot q_d\leftarrow\dot q_d\cdot\min\Big(1, \frac{\Delta_{\max}}{\Vert\dot q_d\Vert_\infty}\Big),\qquad q\leftarrow\mathrm{clamp}(q+\dot q_d, q_{\min}, q_{\max}).
$$

- 과제 QP (ProxQP, `QPSolverWrapper`): Hessian $J_{5w}^\top J_{5w}+\mu I$, 기울기 $-J_{5w}^\top e_w$, box 는 관절 한계와 스텝 한계 $\Delta_{\max}$ (`ik.dq_step_max`) 의 교집합 (뒤집힌 행은 0 으로 닫는다). $J_5^\top J_5$ 는 rank ≤ 5 라 $\mu\gt0$ 이 없으면 해가 유일하지 않다 (`ik.mu`).
- 영공간 투영 $N$ 은 `DifferentialIk` 가 $WJ_5$ 에서 만든다 — $\sigma_{\min}$ 적응 감쇠 $\lambda^2(\sigma_{\min}; \sigma_0, \lambda_{\max})$ 는 $N$ 만 파라미터화한다.
- $\nabla\log w_5$ 는 중심차분 (간격 `ik.fd_step`, 반복당 $2n$ 회 Jacobian) 이다. $\log w_5=\tfrac12\log\det(J_5J_5^\top)$ 는 고정 크기 LDLT 피벗의 로그 합으로 계산한다 — 작은 $w$ 가 언더플로하지 않게. 탐침이 못 쓰게 나온 반복은 상승 항을 **빼고** (대체하지 않고) 그 반복을 `manip_grad_failures` 로 센다.
- 제약은 $\dot q_{clik}$ 만 묶는다. 움직이는 것은 $\dot q_d$ 이고 $\dot q_n$ 은 QP 밖이므로 $\Vert\dot q_d\Vert_\infty$ 축소 (성분별 clip 이 아니라 **방향 유지 축소**) 와 관절 한계 clamp 가 여전히 필요하다.

#### 2.3.3 수락과 종료

매 반복 처음에 현재 $q$ 를 판정한다.

$$
\text{meets}(q)\iff\Vert p_c-p_C(q)\Vert\lt\epsilon_p \wedge \theta\le\alpha_{\max}.
$$

($\theta\le\alpha_{\max}$ 는 $z^\top a_d\ge\cos\alpha_{\max}$ 와 동치이면서 큰 오차에서 수치적으로 안정하다.) 종료는 $\text{meets}\wedge\big(\Vert N\nabla\log w_5\Vert\lt\epsilon_{grad} \vee k_w=0\big)$ 또는 반복 상한 $N_{IK}$ 다 ($\epsilon_{grad}$ = `ik.manip_grad_tol`). 투영한 기울기로 판정한다 — 과제가 상쇄하는 성분은 쓸 수 없어 $\Vert\nabla\log w_5\Vert$ 로는 영원히 수렴하지 않는다.

$q^\ast$ 는 **마지막으로 meets 를 만족한 반복값** 이다 (상승이 반복 상한에 걸려도 수락은 유지, `manip_converged=false`). 한 번도 만족하지 못하면 `kNotConverged`. QP 가 수렴하지 않으면: 수락된 반복값이 있으면 그것을 돌려주고 (`qp_failures=1`), 없으면 `kQpFailed` — 감쇠 pseudo-inverse 로 대체하지 않는다 (지도가 기록한 법칙과 다른 법칙이 된다).

#### 2.3.4 catchability 게이트 (D-18, C-3)

$q^\ast$ 에서 Jacobian 을 다시 세워 (마지막 반복값이 $q^\ast$ 가 아닐 수 있다)

$$
w_5(q^\ast)=\sqrt{\det J_5J_5^\top},\qquad w_6(q^\ast)=\sqrt{\det J_6J_6^\top}
$$

를 **가중하지 않은** $J$ 에서 잰다 ($W$ 는 스텝의 것, 게이트의 것이 아니다). 게이트 정의 (`catchability.definition`, 기본 `arm_5row`) 의 값이 그 정의의 하한 미만이면 제거 (`kBelowManipMin`); 분해의 피벗이 비유한 · 비양수면 `kRankDeficient`. plan 사유로는 `kBelowManipMin` 만 `kManipulability` 이고 나머지 IK 거부 (`kNotConverged` · `kQpFailed` · `kRankDeficient` …) 는 `kIkFailed` 로 집계된다. 상승이 올리는 것은 정의와 무관하게 **항상 $w_5$** 라 $q^\ast$ 가 정의에 의존하지 않고, 두 값은 같은 자세의 두 측정이다. 판정은 `det > 0` 이 아니라 중간값 전부 유한 ∧ $w\ge$ 하한 이다 — NaN 은 탈락이다.

### 2.4 출발 상태

탐색이 후보를 팔 운동으로 판정할 때의 출발은 **현재 명령** 이다 (지도는 대기 자세 정지).

$$
q_0=q_c,\qquad w_0=\dot q_c\enspace(\text{명령이 seed 되지 않았으면 }0),\qquad
\dot q_{plan}=\eta_v\dot q_{\max},\qquad now_{lead}=now+T_{arm}.
$$

RT 가 구간 계획기의 구간을 따르는 동안에는 $(q_0,w_0)$ 가 보고 시각의 명령이 아니라, $now_{lead}$ 에 팔이 있을 보고된 구간 (대기 구간의 node 0 이 $now_{lead}$ 이전이면 그것, 아니면 추종 구간) 을 $now_{lead}$ 에서 평가한 $(q,\dot q)$ 다. 그 구간을 읽을 수 없으면 위 식 그대로다 (L3 §4.3). 아래 rollout 의 출발은 바뀌지 않는다.

rollout 의 출발 $(x_0, \dot x_0)$ 는 기준 생성기가 돌고 있으면 **그 상태** (교체는 거기서 이어진다), 아니면 $q_c$ 의 catch frame 위치에 정지다. 램프 시작값은 $\gamma_0=\gamma_{RT}$ (따르는 plan 이 있고 기준이 돌면), 아니면 0.

### 2.5 도달시간 (닫힌해, `TMinChecked`)

관절 $i$ 가 $(q_{0,i}, w_{0,i})$ 에서 $(q^\ast_i, 0)$ 까지 $\vert\dot q\vert\le\bar\omega_i=\eta_v\dot q_{\max,i}$, $\vert\ddot q\vert\le\bar a_i=\ddot q_{\max,i}$ 로 가는 최소시간. $D=\vert q^\ast_i-q_{0,i}\vert$, $w=\mathrm{sgn}(q^\ast_i-q_{0,i})w_{0,i}$ (이 절의 $w$ 는 관절 초속도이지 manipulability 가 아니다; $D\lt10^{-12}$ 이고 $\vert w_{0,i}\vert\lt10^{-12}$ 이면 $t_{\min,i}=0$, $D\lt10^{-12}$ 이면 부호는 $-\mathrm{sgn}(w_{0,i})$), 정지 상태 이동 $T_{rest}(D)=2\sqrt{D/\bar a}$ ($\sqrt{\bar aD}\le\bar\omega$), 아니면 $D/\bar\omega+\bar\omega/\bar a$:

$$
t_{\min,i}=\begin{cases}
-\dfrac{w}{\bar a}+T_{rest}\Big(D+\dfrac{w^2}{2\bar a}\Big)&w\lt0 (\text{반대 방향 이동 중})\\
\dfrac{w}{\bar a}+T_{rest}\Big(\dfrac{w^2}{2\bar a}-D\Big)&\dfrac{w^2}{2\bar a}\gt D (\text{지나침})\\
\dfrac{2\omega_p-w}{\bar a},\quad\omega_p=\sqrt{\bar aD+\tfrac12w^2}&\omega_p\le\bar\omega (\text{삼각})\\
\dfrac{\bar\omega-w}{\bar a}+\dfrac{\bar\omega}{\bar a}+\dfrac{D-\frac{\bar\omega^2-w^2}{2\bar a}-\frac{\bar\omega^2}{2\bar a}}{\bar\omega}&\text{(사다리꼴)}
\end{cases}
$$

전제 $\vert w_0\vert\le\bar\omega$ 는 검사한다 — 넘으면 clamp 하고 `w0_clamped` 를 세워 결과를 **못 쓰게** 한다 (사다리꼴 분기에 음의 구간이 생긴다). $\bar a\le0$ · $\bar\omega\le0$ · NaN 은 `limits_invalid`, 비유한 입력은 `input_invalid` 이고 $t=+\infty$ 다. 가속 box 가 없는 구성 (`accel_box=false`) 은 도달시간을 판정할 수 없다.

$$
\text{순위 게이트 }\texttt{kRankReach}:\quad \ell_k-T_{arm}-T_{margin} \ge \max_i t_{\min,i}\quad(\text{즉 }t_k-now_{lead}-T_{margin}\ge\max_it_{\min,i}),
$$

그리고 결과가 쓸 수 있어야 한다 (`Usable()`). 이 조건은 **필요조건** 이다 — 실제 운동은 과제 공간 DS 가 만들고 충분성은 rollout (§2.7) 이 본다. $\bar a$ 는 토크 한계에서 도출한 보수적 상수 box 이고 실행 (CLIK 의 토크 행) 의 가속 제약과 같은 값이 아니다 (D-16).

### 2.6 방향 속력과 γ 창

**단위 속도 관절속도** (`UnitSpeedSolver`, 접근축 각속도 0 유지; $\lambda_{dls}$ = `gamma.unit_speed_damping`):

$$
\dot q^u=J_5^\top\big(J_5J_5^\top+\lambda^2I\big)^{-1}[\hat d;\enspace0;\enspace0],\qquad \lambda=\lambda_{dls}.
$$

**방향 속력** (`DirectionalSpeedMax`):

$$
v_{dir,\max}=\frac{\max\big(0, \hat d^\top J_p\dot q^u\big)}{\max_i\dfrac{\vert\dot q^u_i\vert}{\bar\omega_i}}.
$$

분자가 1 이 아니라 투영인 이유: DLS ($\lambda\gt0$) 는 $[\hat d;0]$ 을 정확히 만족하지 않고, 노름은 $\hat d$ 와 다른 방향으로 새는 성분까지 센다. 분모 0 ($\dot q^u=0$ — DLS 풀이가 실패한 후보도 $\dot q^u=0$ 으로 두어 여기로 온다) 은 `undetermined` 이고 창을 **판정 불가** 로 만든다 (0 m/s 라는 물리량이 아니다); $\bar\omega_i\le0$ · NaN 은 `limits_invalid`. 최소노름 해만 보므로 LP 최적값보다 작거나 같다 (보수적).

**γ 창** (`ComputeGammaWindow`, `MaxCatchableSpeed`):

$$
\gamma_{\min}=\mathrm{clamp}\Big(1-\frac{d_{eff}}{\Vert v\Vert T_{close,tot}}, 0, 1\Big),\qquad
\gamma_{\max}=\mathrm{clamp}\Big(\frac{\min(v_{dir,\max}, \eta_vv_{\max})}{\Vert v\Vert}, 0, 1\Big),
$$

$$
\Vert v\Vert_{\max}=\min(v_{dir,\max}, \eta_vv_{\max})+\frac{d_{eff}}{T_{close,tot}}.
$$

하한은 "상대속도 $(1-\gamma)\Vert v\Vert$ 로 $d_{eff}$ 를 지나기 전에 손이 닫힌다", 상한은 "포구 자세에서 $\hat d$ 로 낼 수 있는 속력과 TCP 한계". clamp 는 물리적 입력에서만 무해하므로 $v_{dir,\max}\lt0$ · $\Vert v\Vert\le0$ · $T_{close,tot}\le0$ 은 먼저 `input_invalid` 로 닫는다 (음수가 0 으로 **올라가** 판정을 뒤집지 않게).

$$
\text{순위 게이트 }\texttt{kRankGamma}:\quad\text{창 판정 가능} \wedge \gamma_{\min}\le\gamma_{\max} \wedge \Vert v\Vert+m_\gamma\le\Vert v\Vert_{\max}.
$$

$\eta_v$ 는 TCP 항과 관절 한계 양쪽에 걸린다 — 한쪽에만 두면 D-9 의 완충이 사라진다. 창이 비어도 후보는 남는다 — rollout 이 팔의 한계 $\gamma_{\max}$ 에서 $\gamma_f$ 를 찾고 나머지 상대속도는 손이 받는다.

### 2.7 γ rollout — $(\gamma_f, T_w)$ 의 선택 (`ChooseGamma`)

후보마다 격자 $\Gamma\times\mathcal T$ (`gamma.grid` × `gamma.window_grid`) 의 $\gamma_f\in[\gamma_{lo}, \gamma_{\max}]$, $\gamma_{lo}=\min(\gamma_{\min}, \gamma_{\max})$ (빈 창은 $\gamma_{\max}$ 한 점) 에 대해 §3.1 의 DS 를 **포화 없이** ($a_{\max}, v_{\max}\to\infty$ 로 구성한 `SoftCatchTranslation`) 적분한다.

- 출발 $(x_0, \dot x_0)$ 는 §2.4, 시각은 $t\in[now_{lead}, t_k]$ 를 보폭 $dt$ 로, 대상은 같은 시각의 `SampleAt` (RT 가 볼 것과 같은 함수 — 외삽 샘플이 나오면 판정 불가), 램프는 $[t_0, t_k]$, $t_0=\max(now_{lead}, t_k-T_w)$, $\gamma_0\to\gamma_f$.
- 기록: 전 구간의 $\max\Vert u_{des}\Vert$, $\max\Vert\dot x\Vert$, 창 $[t_0, t_k]$ 안의 같은 두 값, $\Vert e(t_k)\Vert$ (마지막 스텝의 오차 — $t_k-now_{lead}$ 가 $dt$ 의 정수배가 아니면 $t_k$ 직전 스텝의 값). 판정에 쓰는 것은 포화 전 요구 가속 $u_{des}$ 다.

$$
\text{수락}(\gamma_f, T_w)\iff\max_{[now_{lead}, t_k]}\Vert u_{des}\Vert\le\eta_aa_{\max} \wedge \max\Vert\dot x\Vert\le\eta_vv_{\max} \wedge \Vert e(t_k)\Vert\le\epsilon_{term}.
$$

**선택 규칙.** 수락 조합 가운데 $\gamma_f$ 최대, 동률이면 최대 가속이 작은 것. 거친 단계에서 전 구간 수락도 창 수락 (아래) 도 없으면 $\gamma_{lo}$ 를 모든 $T_w$ 로 한 번 더 본다.

**coarse-to-fine.** 전 조합은 `rollout.dt_coarse` 로 거르되 최대치만 판정하고 ($\Vert e(t_k)\Vert$ 는 보지 않는다 — 거친 반암시적 Euler 가 움직이는 대상을 약 $\gamma\Vert v\Vert dt$ 만큼 뒤따라 soft catch 가 자기 이산화 오차로 탈락한다), 고른 하나만 제어 주기 $h$ 로 다시 돌려 세 조건을 확인한다. 확인에서 반증되면 판정은 실패이고 $(\gamma_f, T_w)$ 는 아래 창 규칙의 선택으로 떨어진다 (그것도 없으면 $\gamma_{lo}$ 와 가장 긴 창).

**전 구간과 창.** 접근 구간 (기준이 $x_0$ 에서 $p_c$ 로 $\gamma\approx0$ 으로 끌려가는 구간) 은 $\gamma_f$ 와 무관하게 $\approx\omega^2\Vert x_0-p_c\Vert$ 를 요구하므로 실제 이동에서는 전 구간 수락이 없고 $\gamma_f$ 선택이 정보를 잃는다. 그래서 전 구간 수락이 없으면 같은 규칙을 **창 안의 최대치** 에 적용해 $\gamma_f$ 를 고르고 (`window_only`), 판정은 "실패" 로 둔다 (순위 벌점 `kRankRollout`). 창 규칙의 선택은 거친 단계의 값이고 제어 주기 확인을 받지 않는다. 그것도 없으면 $\gamma_f=\gamma_{lo}$, $T_w=\max\mathcal T$ (손이 요구하는 최소). 창을 판정할 수 없는 후보 (§2.6) 는 rollout 을 돌리지 않고 $\gamma_f=0$, $T_w=\max\mathcal T$ 다.

`mpc` 에서도 이 rollout 이 후보의 순위와 $\gamma_f$ 를 매긴다 — 다만 그 법칙이 RT 에서 실행되지 않으므로 거기서는 실행의 예측이 아니라 순위의 기준이다.

### 2.8 정지점 (`StoppingPoint`) — 런타임은 판정하지 않는다

포구 뒤 등감속 정지에 드는 거리의 닫힌식은

$$
p_{stop}=p_c+\frac{(\gamma_f\Vert v\Vert)^2}{2a_{dec}}\hat d
$$

이다 — §3.6 의 등감속 정지거리 $\Vert\dot x_s\Vert^2/(2a_{dec})$ 에 $\dot x_s\approx\gamma_fv_c$ (§3.7) 를 넣은 것이고, $a_{dec}$ 는 탐색의 복사본 `stop.a_dec` 다.

**런타임 탐색은 이 점을 계산하지도 판정하지도 않는다** — 포구점과 정지점의 위치를 보는 게이트가 없다 (이유는 L3 §4.9). 지금 식을 쓰는 것은 오프라인 지도다 — `JudgeRankGates` 가 γ 창의 두 끝에서 $p_{stop}$ 을 내주고, 그 점이 도달 구 · 바닥 안인지는 지도의 python 이 자기 인자로 건다. $p_{stop}$ 의 IK 는 어디서도 검사하지 않는다. RT 도 구간이 어디서 정지하는지 판정하지 않는다 (MD-73).

모르는 값은 통과가 아니라는 규칙은 그대로다: $T_{lead,\min}$ 을 정할 수 없으면 (키도 $T_{freeze}$ 도 없음) §2.1 의 창이 모든 후보를 떨어뜨린다.

### 2.9 순위 게이트와 오차 예산

후보 $k$ 의 실패 비트마스크 $M_k$:

| 비트 | 실패 조건 |
|---|---|
| `kRankUncertainty` | $\sigma_k$ 모름 $\vee$ $\sigma_k\gt\kappa_\sigma r_{cap}$ |
| `kRankReach` | §2.5 의 게이트 실패 (쓸 수 없는 결과 포함) |
| `kRankGamma` | §2.6 의 게이트 실패 |
| `kRankCommitLead` | $\ell_k\lt T_{close,lead}+h/2+T_{arm}+T_{margin}$ |
| `kRankErrorBudget` | $\neg\big(n_\sigma\sigma_{gap}\le r_{cap}\big)$ ($\sigma_k$ 모름이면 NaN → 실패) |
| `kRankRollout` | §2.7 의 전 구간 수락 없음 |

**오차 예산.** commit 시 $p_c$ 가 고정되고 포구 순간 기준은 $p_c+\gamma(\hat p_{live}(t_c)-p_c)$ 에 있다. $A=p_{true}-\hat p_{live}$, $B=\hat p_{live}-p_c$ 로 두면 간극은 $A+(1-\gamma)B$ 이고, 최적 추정의 직교성 ($\sigma_c^2=\sigma_\ell^2+\sigma_B^2$) 아래

$$
\sigma_{gap}^2=(1-\gamma_f)^2\sigma_c^2+\big(2\gamma_f-\gamma_f^2\big)\sigma_\ell^2+\sigma_{trk}^2+(\Vert v\Vert\delta)^2\qquad(\texttt{CatchErrorSigma}).
$$

계획 단계는 $\sigma_\ell$ 을 모르므로 $\sigma_\ell=\sigma_c=\sigma_k$ 를 넣는다 — 그러면 식이 $\sigma_c^2+\sigma_{trk}^2+(\Vert v\Vert\delta)^2$ 로 환원되고, 이것은 직교성 없이도 ($\sigma_\ell\le\sigma_c$ 인 한) 상한이다. 따라서 **런타임 판정은 $\gamma_f$ 에 의존하지 않고**, 출하 profile ($\sigma_{trk}=\delta=0$, $\kappa_\sigma\lt1/n_\sigma$) 에서 이 게이트는 같은 $\sigma_c$ 에 대한 두 번째 임계로 동작한다 (L3 §4.6): $\kappa_\sigma r_{cap}\lt\sigma_k\le r_{cap}/n_\sigma$ 에서는 불확실성 비트만, 그 위에서는 두 비트가 켜져 벌점이 두 번 붙는다.

### 2.10 점수와 선택

$$
\boxed{
J_k=\mathbf 1[\sigma_k]\enspace w_\sigma\frac{\sigma_k}{r_{cap}}
+\mathbf 1[t_{\min}\wedge\ell_k\gt T_{arm}]\enspace w_t\frac{\max_it_{\min,i}}{\ell_k-T_{arm}}
+w_q\Vert q^\ast-q_n\Vert^2
+w_{late}\big(T_{lead,\max}-\ell_k\big)
-w_\gamma\gamma_f
+w_{pen} \mathrm{popcount}(M_k)
}
$$

판정 통과 후보 가운데 $J$ 최소 (동률은 먼저 평가된 쪽 — IK 순서) 를 고른다. $\sigma$ 를 모르거나 도달시간을 쓸 수 없는 후보는 그 항이 빠지되 해당 순위 게이트 실패로 $M_k$ 에 들어간다. $w_{late}\gt0$ 은 늦은 포구 ([R1] 의 "latest" — 예측이 정확해지는 시간을 번다), $w_\gamma\gt0$ 은 soft catch (충격량 · 오차 예산 두 근거) 를 선호한다. $\gamma_f$ 를 사전식 1 순위로 두지 않는 이유는 격자값이라 동률이 거의 없어 나머지 가중이 전부 죽기 때문이다 — 사전식은 $w_\gamma\to\infty$ 의 특수한 경우다. $w_{pen}$ (키는 `score.penalty`) 은 연속 항을 압도해야 한다. $q_n$ 항은 clamp 전 seed 와 비교한다 — seed 가 한계 안이면 같은 값이다.

### 2.11 교체 규칙 — `closed_form` 이 게시한다 (L3 §4.7)

RT 가 **이 탐색이 게시한** plan 을 따르고 있을 때 (RT 의 `plan_id` 를 최근 게시 링에서 찾는다 — RT 가 거부한 게시는 현재가 되지 않는다) 게시 여부는 다음 순서로 정해진다. 기본은 "게시하지 않음" (RT 는 가진 plan 을 지킨다).

1. **동결.** $t_{c,cur}-now\le T_{freeze}$ 이면 `held_freeze`.
2. **후보 없음.** 판정 통과 후보가 없으면 (settle 중 · 창 안 후보 0 · 입력 무효 · 미구성도 같다) `held_no_candidate` — "plan 없음" 을 게시하지 않는다 (RT 가 아직 안 읽은 교체를 덮는다).
3. **현재 plan 의 점수.** $\vert t_k-t_{c,cur}\vert\le n_s\Delta_v/2$ 인 통과 후보가 있으면 그 $J_{cur}$ (없으면 현재 plan 은 이번 검사에서 **불가능**).
4. **개선.** $\text{better}\iff\neg\text{feasible}_{cur} \vee J_{cur}-J_{best}\gt\Delta_J$.
5. **가속 계단** (구간 계획기 아래에서는 보지 않는다 — 계단이 들어갈 L4 기준이 없어 항상 통과로 둔다, `follows_segments`). 교체가 $u_{des}$ 에 넣는 계단의 상계 (`SwitchAccelStepBound`):

$$
\Delta u\le\omega^2\vert1-\gamma\vert \Vert\Delta p_c\Vert+\big(2\zeta\omega\vert\dot\gamma\vert+\vert\ddot\gamma\vert\big)\Vert o-p_c\Vert+2\vert\dot\gamma\vert \Vert v_O\Vert \le \eta_{jump}a_{\max},
$$

   $\Delta p_c=p_{c,best}-p_{c,cur}$, $p_c$ 는 **옛** 포구점. RT 는 교체 plan 을 γ 는 이어서, 램프는 다시 시작해서 ($\dot\gamma=\ddot\gamma=0$) 채택하므로 실제 계단은 $\Delta u=\omega^2(1-\gamma)\Delta p_c-(2\zeta\omega\dot\gamma+\ddot\gamma)(o-p_c)-2\dot\gamma v_O$ 이고 위 식은 그 삼각 상계다 (§3.7). 좌변은 RT 가 이 게시를 채택할 수 있는 구간 $[now_{lead}, now_{lead}+T_{bud}+2h]$ 의 `switch.samples` 개 순간에서 **최악값** 으로 잰다 — $\gamma, \dot\gamma, \ddot\gamma$ 는 RT 가 실제로 돌리는 램프를 그 순간에서 평가한 값, $(o, v_O)$ 는 같은 시각의 공 샘플이다. 기준이 돌고 있지 않으면 $\gamma=\dot\gamma=\ddot\gamma=0$ 으로 한 점, 램프 정보가 없으면 스냅샷 값으로 한 점. 램프 항이 살아 있는 ($\dot\gamma\ne0$ 또는 $\ddot\gamma\ne0$) 순간에 공을 샘플할 수 없으면 판정 불가 (거부) 이고, 램프가 멈춘 순간은 샘플하지 않는다. 램프 항은 $\Vert o-p_c\Vert$ 에 비례하므로 램프가 도는 동안의 교체는 거의 거부되고, 램프가 멈춘 구간에서는 $\Vert\Delta p_c\Vert\le\eta_{jump}a_{\max}/(\omega^2(1-\gamma))$ 다.

6. **결정.**

| 조건 | 결정 | 게시 |
|---|---|---|
| $\neg$ better, best 가 현재 후보, $\Vert\Delta p_c\Vert\gt\epsilon_{term}$, 계단 통과 | `refreshed` — 같은 후보, 움직인 예측을 새 $p_c$ 로 | 예 |
| 위에서 계단 실패 | `held_jump` | 아니오 |
| $\neg$ better, 그 밖 | `held_hysteresis` | 아니오 |
| better, 계단 실패 | `held_jump` | 아니오 |
| better, 계단 통과 | `replaced` | 예 |

히스테리시스는 "다른 후보로 바꾸는가" 의 규칙이지 "옛 예측을 붙잡는가" 가 아니다 — 붙잡으면 soft catch 가 $(1-\gamma_f)\Vert\delta\Vert$ 만큼 빗나간다. 구간 계획기 (`mpc` · `mpc_docking`) 에서도 이 절의 순서는 돈다 (5 는 항상 통과). 다만 게시는 탐색이 아니라 계획기의 한 주기가 한다: `replaced` 는 새 plan 과 그 첫 구간의 쌍으로 게시되고, `refreshed` 를 포함한 그 밖의 판정은 게시되지 않고 구간의 재계획으로 간다 (L3 §5.3).

### 2.12 게시 — plan 의 값

고른 후보 $k^\ast$ 에서

$$
t_c=t_{k^\ast},\quad p_c=\hat p_{k^\ast},\quad v_c=\hat v_{k^\ast},\quad a_d=-\frac{v_c}{\Vert v_c\Vert},\quad
t_{cmd}=t_c-T_{close,lead},
$$

$$
(\gamma_0, \gamma_f, t_0, t_1)=\big(\gamma_{RT}\text{ 또는 }0, \gamma_f^\ast, \max(now_{lead}, t_c-T_w^\ast), t_c\big),\qquad
\sigma_c=\sigma_{k^\ast} (\text{모름이면 }0),\quad\sigma_\ell=\sigma_c,\quad \Delta p_{impact}=m_b(1-\gamma_f^\ast)\Vert v_c\Vert .
$$

γ 램프는 rollout 이 돌린 것과 **같은 프로파일** 이다 (같은 $t_0$ 규칙; 창 규칙으로 고른 후보는 거친 단계의 것). $q^\ast$ 는 device 순서로 싣고, $w_5$ · $w_6$ 를 함께 기록한다. $t_{cmd}$ ($T_{close,lead}$ 가 TBD 면 $t_c$; $T_{close,lead}$ 는 손이 폐쇄를 지령하는 선행으로 잰 폐쇄 시간 $T_{close,e2e}$ 와 다른 설계값이다 — L6 §4.2) 는 기록 · 진단용이다 — 손 명령에 쓰는 값은 손 시퀀서가 동결된 $t_c$ 로 계산한 하나다 (C-14). $T_{tick}=h/2$ 는 γ 창의 예산에는 들어가고 $t_{cmd}$ 에는 들어가지 않는다 (틱 반올림 오차는 영평균). $\Delta p_{impact}$ 는 기록일 뿐 게이트가 아니다 (TBD-IMP-01).

게시 직전 cycle 이 token · 리셋 epoch · activation 을 다시 확인하고, RT 는 `JudgePlan` (유효 · activation · 트랙 · 새 `plan_id` · 나이 · reset floor · $t_c-now\gt T_{freeze}$) 으로 받는다 (L3 §5.2).

### 2.13 동결 후 감시 (`Monitor`)

COMMITTED · CLOSING 의 wake 는 따르는 plan 의 $t_c$ 의 공분산 (가장 가까운 샘플의 $6\times6$ 을 $F\Sigma F^\top$ 으로 $t_c$ 까지 전파한 것, L3 §5.3) 에서 $\sigma_\ell=\sigma_{\max}$ 를 계산해 **기록만** 한다. 이 값으로 abort 를 판단하지 않는다 — 동결 뒤의 낡은 입력은 수신 나이로 판정한다 (L7).

### 2.14 사유

plan 이 없을 때의 사유는 **가장 많이 걸린 판정 게이트** 다 (`kInputNonFinite` · `kIkFailed` · `kManipulability`). `kStoppingDistance` 는 메시지에 남은 값이고 쓰는 탐색이 없다 (L3 §4.9). 예산에 밀려 평가 못 한 후보는 예산이 실제로 잘랐을 때만 세고 (`kBudgetExceeded`), settle 중은 `kUncertainty`, 창 안에 후보가 없으면 `kHorizonShort` 다. 고른 후보의 순위 게이트 실패는 사유가 아니라 CSV 의 비트마스크다.

---

## 3. `closed_form` 구간 — RT 가 만드는 팔 기준

`planner.segment.mode: closed_form` 에서 APPROACH 부터 HOLD 까지의 팔 기준은 RT tick 이 직접 만든다. 구간 계획기는 없다.

### 3.1 오차 좌표 soft-catch DS ([R3], L4 §4.1)

포구점을 원점으로 두고 $\xi^O=p_O-p_c$, $\xi=x-p_c$. 오차와 그 동역학은

$$
e=\xi-\gamma\xi^O,\qquad \dot e=\dot\xi-\big(\gamma\dot\xi^O+\dot\gamma\xi^O\big),\qquad \ddot e=-\omega^2e-2\zeta\omega\dot e
$$

이고 $\ddot\xi=u$ 이므로 기준 가속 (포화 전 요구값) 은

$$
\boxed{u_{des}=\gamma a_O+2\dot\gamma v_O+\ddot\gamma\xi^O-\omega^2e-2\zeta\omega\dot e}.
$$

$A_1=-\omega^2I$, $A_2=-2\zeta\omega I$ 의 LTI 이고 [R3] 의 GMM LPV 는 없다. [R3] 의 plant 는 이중적분기가 아니라 식 (4) 의 $u_{[R3]}$ 에 plant 상쇄항이 붙어 있는데, 총 가속도는 위 $u_{des}$ 와 정확히 같다 — [R3] 식을 그대로 옮기면 plant 항을 두 번 뺀다. $a_O$ 는 vision 이 준 $a$ 를 샘플러가 보간한 값이다 (수치미분 금지, $C^2$ 보간이라 샘플 경계에서 튀지 않는다).

### 3.2 γ 프로파일 (`GammaProfile`)

5 차 램프, 양 끝 1 · 2 계 미분 0. $s=\mathrm{clamp}((t-t_0)/T, 0, 1)$, $T=t_1-t_0$, $d=\gamma_f-\gamma_0$:

$$
\gamma=\gamma_0+d(10s^3-15s^4+6s^5),\qquad
\dot\gamma=\frac{d}{T}(30s^2-60s^3+30s^4),\qquad
\ddot\gamma=\frac{d}{T^2}(60s-180s^2+120s^3).
$$

$T\le0$ 은 $t_0$ 에서의 계단 (미분 0) 이고, $0\lt T\lt$ `kMinRampSeconds` 는 무효다 ($\ddot\gamma\propto1/T^2$ 가 유한하지만 무의미한 값이 된다). 포구 전 $\gamma$ 는 feedforward 에만 들어가므로 오차계는 임의의 $C^2$ $\gamma(t)$ 에 대해 같고, 대신 $\Vert u\Vert$ 가 $\dot\gamma, \ddot\gamma, \Vert\xi^O\Vert$ 에 비례해 커진다 — 창과 램프 길이를 탐색이 rollout 으로 정하는 이유다.

### 3.3 이산화와 포화 (`SoftCatchTranslation::Step`)

매 tick $h$ 에

$$
u=u_{des}\cdot\min\Big(1, \frac{a_{\max}}{\Vert u_{des}\Vert}\Big),\qquad
\dot x^+=(\dot x+uh)\cdot\min\Big(1, \frac{v_{\max}}{\Vert\dot x+uh\Vert}\Big),\qquad
x^+=x+\dot x^+h,\qquad \ddot x_{real}=\frac{\dot x^+-\dot x}{h}.
$$

반암시적 Euler · 방사형 포화이고 포화 중에는 §3.1 의 오차계가 성립하지 않는다. 포화 검출은 NaN 에서도 참이 되도록 부정 비교 ($\neg(\Vert u\Vert\le a_{\max})$) 로 쓴다. 비유한 입력 · 넘친 명령은 상태를 보존한 채 무효로 돌려준다. 오차계를 같은 방식으로 이산화하면 ($\zeta=1$, $s=\omega h$) Jury 조건의 안정 경계는 $0\lt s\lt2\sqrt2-2\approx0.828$ 이고 정확도 권장은 $s\le0.05$ 다 — 검증기가 실제 $h$ 로 검사한다 (L4 §4.7). 반환값의 시간축이 섞여 있다: $x^+, \dot x^+$ 는 다음 tick 의 명령, $\ddot x_{real}$ 은 $[t, t+h]$ 의 실현 가속, $e, \dot e, u_{des}$ 는 $t$ 의 진단값이다.

### 3.4 채택과 교체 (RT 측)

- **첫 채택** (TRACKING → APPROACH 다음 tick, 법칙의 첫 호출): 기준을 현재 명령 자세의 catch frame 위치에 정지로 리셋하고 ($x\leftarrow p_C(q_c)$, $\dot x\leftarrow0$) plan 의 $(p_c, \gamma_0, \gamma_f, t_0, t_1)$ 을 건다. 첫 명령 스텝이 DS 자신의 첫 스텝이 되게 — 생성기가 놓여 있던 곳에서의 점프가 아니라. 리셋이 거부되면 `REF_SATURATED`, 프로파일이 무효면 `PLAN_INVALID` 다.
- **매 tick** (APPROACH · COMMITTED · CLOSING): 대상 $o=$ `SampleAt`$(now_{lead})$, $t=$ `ProfileSeconds`$(now_{lead}, t_0)$ 로 §3.3 을 한 번.
- **교체** (APPROACH 에서만. 아래는 `closed_form` 의 것이다 — 구간 계획기 아래의 교체는 plan 과 첫 구간의 쌍을 그 구간의 node 0 에서 받는 것이고 L7 §4.3a 가 적는다; 받은 plan 의 `plan_id` 가 따르는 것과 다르고 **옛 plan** 의 $t_c-now\gt T_{freeze}$ 일 때): 생성기를 리셋하지 않고 포구점 $p_c$ · γ 램프 · 접근축 $a_d$ 를 새 plan 의 것으로 바꾼다 (대상 $o$ 는 그대로 공이다) — 기준 상태 $(x, \dot x)$ 는 연속이고 오차만 점프한다. 새 램프는 **채택 tick 의 기준 γ** $\gamma_{now}$ 에서 ($\gamma_0\leftarrow\gamma_{now}$, plan 의 $\gamma_0$ 는 한 탐색 전의 값이라 쓰지 않는다), **그 tick 이후에** ($t_0\leftarrow\max(t_0, now_{lead})$) 시작한다. 시작이 이미 지난 램프는 첫 스텝에서 올라가 있어 γ 계단이 되고, 계단은 $\gamma(o-p_c)$ · $\ddot\gamma(o-p_c)$ 를 통해 $e$ · $u_{des}$ 의 계단이 된다.

### 3.5 CLIK 입력 (L5 §4.2)

기준 $x^+$ 를 위치 목표로, $\dot x^+$ 를 선속도 feedforward 로, plan 의 $a_d$ 를 축 목표로 넘긴다 (`PositionAxisTarget`). 각속도 feedforward 는 0 이다 — 고정된 포구점의 접근축은 돌지 않는다. CLIK 은 명령 자세 $q_c$ 에서 5 행 과제 ($J_p\dot q=\dot x^++K_p(x^+-x_C)$, $SR_{WC}^\top J_\omega\dot q=SR_{WC}^\top K_ae_a$) 와 자세 과제 (목표는 시행 시작 시의 명령 자세, 작은 가중) 를 푼다. $q^\ast$ 는 RT 의 CLIK 이 읽지 않는다.

**포화 감시.** 공을 따르는 tick 에서 기준이 `supervisor.sat_ticks` tick 연속 포화하면 (연속이 끊기면 0 부터; 생성기가 그 tick 의 스텝을 거부해 기준이 없으면 연속과 무관하게 즉시) `REF_SATURATED` — COMMITTED 이전은 RETREAT, 이후는 ABORT_SAFE (D-8). 실행 중 γ 를 낮추는 경로는 없고, 계획의 여유 ($\eta_v$, $\eta_a$) 가 유일한 완충이다. DECEL · HOLD 에서는 세지 않는다. CLIK 실패 사유가 이보다 우선한다.

### 3.6 DECEL — 등감속 가상 대상 (`EvaluateDecelTarget`), HOLD

CLOSING 에서 $now_{lead}\ge t_c$ 가 되는 tick 에 진입한다. 진입 상태는 **기준 생성기 자신의 상태** $(x_s, \dot x_s)$ 이고 $t_s$ 는 그 tick 의 $now_{lead}$ 다. $\hat u_s=\dot x_s/\Vert\dot x_s\Vert$, $\tau=now_{lead}-t_s (\ge0)$, $\tau_s=\Vert\dot x_s\Vert/a_{dec}$:

$$
p_v(\tau)=x_s+\dot x_s\tau-\tfrac12a_{dec}\hat u_s\tau^2,\qquad v_v(\tau)=\dot x_s-a_{dec}\hat u_s\tau,\qquad a_v=-a_{dec}\hat u_s\qquad(\tau\lt\tau_s)
$$

$$
p_v=x_s+\tfrac12\Vert\dot x_s\Vert\tau_s\hat u_s,\qquad v_v=a_v=0\qquad(\tau\ge\tau_s, \texttt{stopped}).
$$

이 대상을 §3.1 의 $o$ 로 넣고 γ 를 상수 1 로 바꾼다 ($\gamma_0=\gamma_f=1$ 인 램프, $\dot\gamma=\ddot\gamma=0$) — 생성기는 리셋하지 않고 대상만 바뀐다. $\Vert\dot x_s\Vert^2\le10^{-12}$ 이면 이미 정지로 보고 $x_s$ 를 든다. 가상 대상이 정지하면 ($\tau\ge\tau_s$) HOLD 이고, HOLD 는 정지한 $p_v$ 를 계속 따른다. 대상이 무효 (비유한 · $a_{dec}\le0$) 이거나 생성기가 그 tick 의 스텝을 거부하면 기준이 없는 것이라 `PARAMS_TBD` → ABORT_SAFE (관절공간 램프 `JointSpaceStopStep`: $\dot q_i\leftarrow\mathrm{sgn}(\dot q_i)\max(\vert\dot q_i\vert-\ddot q_{\max,i}h, 0)$, $q_i\leftarrow\mathrm{clamp}(q_i+\dot q_ih, q_{\min,i}, q_{\max,i})$ (CLIK 과 같은 margin 상자, 한계에 닿은 관절은 $\dot q_i=0$) — QP · 과제 공간 · 모델 비의존, 어느 planner 에서도 같다).

### 3.7 닫힌식 성질

**전환 직후 오차는 정확히 0.** γ ≡ 1 에서 $e=x-p_v$, $\dot e=\dot x-v_v$ 이므로 $\tau=0$ 에서 $e=x_s-p_v(0)=0$, $\dot e=\dot x_s-v_v(0)=0$ 이고 $u_{des}=a_v$, 크기가 정확히 $a_{dec}$ 다. 그래서 $a_{dec}\le a_{\max}$ 가 층간 제약이다 (검증기). 연속인 것은 기준 상태 $(x, \dot x)$ 이고 오차는 DS 의 잔여 수렴 오차 $\epsilon_{conv}$ 만큼 점프한다. 기준 가속은 $\tau=0$ 과 $\tau_s$ 에서 불연속이다 (jerk 무한) — $a_{dec}$ 램프는 없다.

**정지거리.** $\Vert\dot x_s\Vert^2/(2a_{dec})$ — §2.8 의 정지점 $(\gamma_f\Vert v\Vert)^2/(2a_{dec})$ 와 같은 식이다 ($\dot x_s\approx\gamma_fv_c$, 아래).

**포구 순간의 기준** (L4 §4.2). $e(t_c)=\dot e(t_c)=0$, $\xi^O(t_c)=0$ 이면 $\xi(t_c)=0$, $\dot\xi(t_c)=\gamma\dot\xi^O(t_c)$: 위치는 γ 와 무관하게 일치하고 기준 속도는 대상 속도의 γ 배, 상대속도는 $(1-\gamma)v_O(t_c)$ 다. $\gamma=0$ 은 정지 포구, $\gamma=1$ 은 완전 추종.

**임계감쇠 닫힌해** ($\zeta=1$, 축별, `CriticallyDampedError`):

$$
e(t)=\big(e_0+(\dot e_0+\omega e_0)t\big)e^{-\omega t},\qquad \dot e(t)=\big(\dot e_0-\omega t(\dot e_0+\omega e_0)\big)e^{-\omega t}.
$$

포화가 없을 때의 식이다 — 탐색은 $t_c$ 의 잔여 오차와 포화 여부를 rollout 의 적분으로 잰다. $\zeta\ne1$ 은 검증기가 막는다 (이 닫힌해와 §3.3 의 이산 경계가 $\zeta=1$ 의 것이다).

**예측 오차와 교체 점프** (L4 §4.3). 실제 대상이 $t_c$ 에 $\xi^O=\delta\ne0$ 에 있으면 간극은 $(1-\gamma)\Vert\delta\Vert$ 다. 포구점을 $p_c'=p_c+\Delta p_c$ 로 바꾸면 $\xi^{O\prime}=\xi^O-\Delta p_c$, $\xi'=\xi-\Delta p_c$ 라 프로파일을 그대로 둘 때는 $e'=e-(1-\gamma)\Delta p_c$, $\dot e'=\dot e+\dot\gamma\Delta p_c$ 다. 런타임 교체는 γ 는 잇되 램프를 새로 시작하므로 ($\dot\gamma'=\ddot\gamma'=0$) $\dot e'=\dot e+\dot\gamma\xi^O$ 이고 ($\xi^O$ 는 옛 포구점 기준) feedforward 의 $2\dot\gamma v_O+\ddot\gamma\xi^O$ 가 사라져

$$
\Delta u_{des}=\omega^2(1-\gamma)\Delta p_c-\big(2\zeta\omega\dot\gamma+\ddot\gamma\big)\xi^O-2\dot\gamma v_O
$$

— §2.11 규칙 5 의 상계가 이것을 $\eta_{jump}a_{\max}$ 로 묶는다.

**$\gamma\equiv0$ 이면 대상은 소거된다.** $u=-\omega^2(x-p_c)-2\zeta\omega\dot x$ — 끌개는 $p_c$ 뿐이다. 그래서 정지 목표는 $p_c$ 로 주고, `Reset` 은 $p_c\leftarrow x$ 까지 한다 (활성화 직후 현재 자세 유지, 재무장 시 직전 시행의 포구점이 남지 않게).

### 3.8 감독과의 접점

| 전이 | 조건 ('지금') | 법칙 |
|---|---|---|
| APPROACH → COMMITTED | $t_c-now\le T_{freeze}$ ($now$) | 이후 $p_c$ · $t_c$ · γ 프로파일 동결, 교체 없음 |
| COMMITTED → CLOSING | $now\ge t_{cmd}-h/2$ (손 시퀀서가 Close 를 낸 tick — 가장 가까운 tick 반올림) | 기준은 계속 |
| CLOSING → DECEL | $now_{lead}\ge t_c$ | §3.6 진입 |
| DECEL → HOLD | $\tau\ge\tau_s$ | 정지한 대상 추종 |
| REF_SATURATED | `sat_ticks` 연속 포화 (공 추종 tick) | COMMITTED 전 RETREAT / 후 ABORT_SAFE |

$T_{freeze}\ge T_{close,lead}+T_{arm}+h$ 를 검증기가 강제한다. 공 lane 의 사유 (`BALL_STALE` · `HORIZON_EXTRAP` · `BALL_STALE_LONG`) 와 CLIK 의 사유 (`QP_FAILED` · `JOINT_CONFLICT` · `TRACK_ERR`) 는 두 planner 공통이고, 전체 상태 머신 · 사유 · 리셋 목록은 L7 §4.1 · §4.8.

---

## 4. 탐색과 법칙의 결합

### 4.1 rollout 은 법칙의 재생이다

탐색의 rollout 은 RT 가 돌릴 **같은 코드** (`SoftCatchTranslation` · `SampleAt` · `MakeGammaProfile`) 를 같은 시간축 ($now_{lead}$) 에서 돌린다. 다른 점은 셋이다.

| | rollout (탐색) | RT (`closed_form`) |
|---|---|---|
| 포화 | 없음 — 최대치를 $\eta_aa_{\max}$ · $\eta_vv_{\max}$ 와 비교 | $a_{\max}$ · $v_{\max}$ 로 포화 |
| 파라미터 | 탐색의 복사본 `planner.search.grid.reference.*` · `stop.a_dec` | `reference.*` · `supervisor.decel.a_dec` |
| 그 뒤 | CLIK · 팔 지연 없음 ($\sigma_{trk}$ 가 그 자리) | CLIK → $T_{arm}$ 선행 보상 → 팔 |

복사본은 원본의 검증 규칙을 따르고, 계획기가 켜져 있을 때 (TBD 인 쌍은 비교하지 않는다) `closed_form` 에서 원본과 다르면 park (`kSearchCopyDiffers` — 탐색이 팔이 따르지 않는 운동으로 후보를 매기게 된다), 구간 계획기에서는 WARN 이다 (L3 §6). 복사본을 두는 이유는 탐색이 여러 segment mode 에서 돌기 때문이다.

### 4.2 탐색이 법칙에 대해 가정하는 것

- 출발: 기준 생성기의 상태 (돌고 있으면) — 교체가 거기서 이어진다는 §3.4 와 같다.
- γ 램프: 게시한 $(\gamma_0, \gamma_f, t_0, t_1)$ 이 rollout 의 것과 같다. RT 는 교체 시 $\gamma_0$ 와 $t_0$ 를 채택 tick 의 값으로 덮지만 (§3.4), 그 차이는 계단 상한 (§2.11) 이 묶는다.
- 정지: 탐색은 정지점을 판정하지 않는다 (§2.8). 지도가 쓰는 $p_{stop}$ 은 $\dot x_s=\gamma_fv_c$ 를 가정한 §3.6 의 닫힌식이고, 실제 $\dot x_s$ 는 $t_c$ 의 기준 속도라 $\epsilon_{conv}$ 만큼 다르다.
- 도달시간 (§2.5) 은 관절별 시간최적 프로파일의 필요조건이고 실제 운동은 과제 공간 DS 다 — 충분성은 rollout.

### 4.3 `mpc` · `mpc_docking` 과의 차이

| | `closed_form` | `mpc` (`mpc_docking` 도 같은 곳이 많다) |
|---|---|---|
| 탐색 | §2 전부 | §2 전부 (따르는 동안에도 돈다. 교체 §2.11 의 판정이 `replaced` 면 계획기가 plan 과 첫 구간의 쌍을 게시한다 — L3 §5.3) |
| plan 에서 실행이 읽는 것 | $p_c$, $a_d$, $(\gamma_0, \gamma_f, t_0, t_1)$, $t_c$ | $t_c$ (격자 닻), $p_c$ · $v_c$ · $a_d$ (포구 노드 목표), $q^\ast$ (첫 선형화 기준) |
| $(\gamma_f, T_w)$ | 실행된다 | 순위에만. 속도 목표 비는 `planner.segment.mpc.catch.gamma_ref` |
| APPROACH – 정지 | §3 (DS → DECEL → HOLD) | MPC 관절 노드 구간 (formulation §1.6) |
| 예측 변화의 흡수 | plan 교체 (§2.11) | 같은 포구는 구간 재계획, 다른 포구는 교체 쌍 (L7 §4.3a) |
| `REF_SATURATED` | 있음 | 없음 |

---

## 5. Sanity check

1. $\gamma_f=0$, 예측 정확: $t_c$ 의 간극 $\approx\epsilon_{conv}$, 상대속도 $\approx\Vert v_O\Vert$. $\gamma_f=0.4$: 상대속도 $\approx0.6\Vert v_O\Vert$ (L4 §4.9 표, G4-A).
2. 예측 오차 $\delta$ 의 간극 $\approx(1-\gamma_f)\Vert\delta\Vert$.
3. §3.7 의 닫힌해가 미세 스텝 적분과 일치하고, $s=\omega h$ 가 0.80 에서 수렴 · 0.85 에서 발산한다 (G4-C).
4. §2.5 의 닫힌해가 속도 · 가속 제약 LP (시간 이분) 의 해와 일치한다 (G3-A).
5. γ 창: $v_{dir,\max}=0$ 이면 $\Vert v\Vert\le d_{eff}/T_{close,tot}$ — [R1] 의 포켓 3 cm · 6 m/s 는 $T_{close}\le5$ ms (5 ms 통과, 5.1 ms 창 빔).
6. rollout: 창 안 최대 $\Vert u_{des}\Vert$ 는 $\gamma_f$ 에 거의 선형이고 창 길이가 포화를 지배한다 (L3 §4.8 표, G3-B ±5 %).
7. DECEL 진입 tick 에 $e=\dot e=0$ 이 정확히 성립하고 $\Vert u_{des}\Vert=a_{dec}$ 다 (G7-B).
8. 교체 계단: 램프 정지 · $\gamma=0$ 에서 $\Vert\Delta p_c\Vert\le\eta_{jump}a_{\max}/\omega^2$.
9. 같은 입력 · 같은 seed · 같은 옵션의 IK 는 호출 순서와 무관하게 bit-exact 로 같다 (지도 ↔ 런타임, G3-I).
10. $T_{arm}\ne0$ fixture 에서 두 시간축이 갈린다 — §0.2 의 표가 타입으로 고정된다 (G0-E).

## 6. 구현하지 않은 것과 알려진 한계

- **충격량 게이트** — $\Delta p_{impact}$ 는 기록만 (TBD-IMP-01).
- **γ derate** — 실행 중 포화에 γ 를 낮추는 경로가 없다 (D-8). 완충은 $\eta_v$ · $\eta_a$ 뿐.
- **LPV $A_i(\theta)$** — LTI 만. $\zeta\ne1$ 도 없다.
- **$(q^\ast, t_c)$ 동시 NLP** · 격자 사이의 포구 시각 — 후보는 vision 격자 점뿐이다. 그 경로는 `NlpCatchSearch` (ball_catching §17.11) 이고 계획기에 꽂혀 있지 않다.
- **$\sigma_\ell$ 로 하는 abort** — 경로 2 의 $\sigma_\ell$ 은 CSV 에만 있다. 오차 예산의 γ 의존 항은 어느 판정에도 들어가지 않는다.
- **램프 이어 붙이기** — 교체는 램프를 다시 시작한다 ($\dot\gamma$ 연속 아님). 램프가 도는 동안의 교체는 상한이 거의 다 거부한다.
- **$a_{dec}$ 램프**, **각속도 feedforward** ($a_d$ 고정), **$p_{stop}$ 의 IK**, **자세 의존 가속 한계** (D-16 은 상수 box).
- **도달시간의 보수성** — 상수 box 는 최악 부호 충분조건이라 토크 한계보다 훨씬 보수적이고, 순위 게이트라 제거하지는 않는다.
- **방향 속력의 보수성** — 최소노름 해만 본다 (7 축에서 LP 와 더 벌어진다).
- **$v_{dir,\max}$ 와 $w_5$ 의 상충** — $w_5$ 를 올린 자세가 $\hat d$ 로 빠른 자세는 아니다.
- **직교 가정** — sim truth 에서 $\mathrm{Cov}(A,B)\lt0$ 이지만 (G3-H) 런타임 판정이 $\gamma$ 에 의존하지 않아 영향이 없다 (L3 §4.6).

## 7. 기호 ↔ YAML 키 ↔ 코드

값 · 범위 · 근거는 YAML (`integrated_bringup/config/<robot>/controllers/catching/search_grid.yaml` · `planner_closed_form.yaml`, 주 파일) 과 파서가 갖는다. 전체 키 표는 L3 §6.

| 기호 | 키 | 코드 |
|---|---|---|
| $T_{lead,\min}$, $T_{lead,\max}$, $s$ | `planner.search.grid.slice.{t_lead_min, t_max, dt}` | `PlannerParams::LeadMin`, §2.1 |
| $\kappa_\sigma$, $n_\sigma$, $\sigma_{trk}$, $\delta$ | `unc.kappa_sigma`, `budget.{n_sigma, sigma_trk, clock_err}` | `SigmaMax`, `CatchErrorSigma` |
| $w_\sigma, w_t, w_q, w_{late}, w_\gamma, w_{pen}$ | `score.{w_sigma, w_t, w_q, w_late, w_gamma, penalty}` | §2.10 |
| $N_{IK,\max}$, $T_{bud}$, `n_settle` | `planner.search.grid.{max_ik, budget_s, n_settle}` | §2.2 |
| $d_{eff}$, $r_{cap}$ | `hand.{d_eff, r_cap}` | §2.6, §2.9 |
| $\epsilon_p$, $\alpha_{\max}$, $\rho$, $\sigma_0$, $\lambda_{\max}$, $\Delta_{\max}$, $\mu$, $K_n$, $k_w$, $N_{IK}$, $\epsilon_{grad}$, $v_{eps}$ | `ik.{eps_pos, alpha_max, rho, sigma0, lambda_max, dq_step_max, mu, k_null, k_manip, max_iter, manip_grad_tol, v_eps}` (중심차분 간격 `ik.fd_step`, QP 허용오차 · 반복 상한 `ik.{qp_eps_abs, qp_max_iter}`) | `CatchPoseIkOptions`, §2.3 |
| 게이트 정의 · 하한 | `catchability.{definition, manipulability_min.*}` | §2.3.4 |
| $\eta_v$, $m_\gamma$, $\lambda_{dls}$ | `gamma.{eta_v, margin, unit_speed_damping}` | `PlanningTcpSpeed`, `UnitSpeedSolver` |
| $\Gamma$, $\mathcal T$, $\eta_a$, $\epsilon_{term}$, $dt_{coarse}$ | `gamma.{grid, window_grid, eta_a, eps_term}`, `rollout.dt_coarse` | `RolloutSettings`, `ChooseGamma` |
| $T_{margin}$ | `time.margin` | §2.5, §2.9 |
| $\Delta_J$, $\eta_{jump}$, `samples` | `switch.{delta_J, eta_jump, samples}` | `SwitchStep`, §2.11 |
| $\omega, \zeta, v_{\max}, a_{\max}$ (탐색) · $a_{dec}$ (탐색) | `planner.search.grid.reference.*`, `stop.a_dec` | rollout · γ 창 · $p_{stop}$ |
| $\omega, \zeta, v_{\max}, a_{\max}$ (RT) · $a_{dec}$ (RT) | `reference.*`, `supervisor.decel.a_dec` | `SoftCatchTranslation::Params`, `EvaluateDecelTarget` |
| $\ddot q_{\max}$ | `robot.arm.qdd_max` (주 파일) | `MaxJointTMin` |
| $q_n$ | `planner.wait_pose`, `wait_pose_source` | §2.3, §2.10 |
| $T_{freeze}$, $T_{arm}$, $T_{close,lead}$, `sat_ticks` | `planner.freeze.T_freeze`, `joint_cmd.lag.T_arm`, `robot.hand.T_close_lead` (없으면 `T_close_e2e`), `supervisor.sat_ticks` | §3.8 |
| $T_{close,e2e}$ ($T_{close,tot}=T_{close,e2e}+h/2$) | `robot.hand.T_close_e2e` (잰 값) | §2.6 |

코드 대응: 후보 · 순서 · 점수 · 교체 · 게시는 `grid_catch_search.cpp`, IK 는 `catch_pose_ik.cpp`, 도달시간 · γ 창 · 정지점 · 오차 예산은 `time_feasibility.hpp` 를 `rank_gates.hpp` 의 `JudgeRankGates` 가 묶고, $\dot q^u$ 는 `unit_speed.hpp`, rollout 은 `gamma_rollout.hpp`, DS · γ 프로파일 · 닫힌해는 `soft_catch.hpp`, 감속 대상은 `decel_target.hpp`, 관절공간 정지는 `joint_stop.hpp`, 축 정렬 오차는 `rtc_math` 의 `axis_align.hpp`, RT 의 채택 · tick · DECEL 진입은 `controller.cpp` (`RunTrackingTick` · `StepReferenceAndSolve` · `EnterDecel` · `RunDecelLawTick`) 다. 테스트는 L3 §9 · L4 §9 · L7 §9.

## 8. 참고

[R1] 포구 제약과 관절 램프 (도달시간 제약의 출처, "latest" 목적), [R2] 시간 슬라이스 탐색과 허용 콘, [R3] soft catch 의 오차 좌표 DS 와 softness — 서지는 [CATCHING_MASTER.md](CATCHING_MASTER.md). 결정 ID 는 [ID_INDEX.md](../ID_INDEX.md).
