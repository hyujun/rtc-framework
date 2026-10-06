# Arm–Hand Ball Catching을 위한 Inverse-Dynamics MPC 수학적 구성 (개정판 v3)

> **구현 상태.** §0 – §16 은 설계 자료이고 고치지 않는다 — 원래 설계와 구현을 견주어 볼 수 있게 그대로 둔다. 구현한 내용은 문서 끝의 §17 에 원래 절과 대응시켜 적는다 (17.$n$ 이 §$n$ 의 구현). 지금 §17 은 mpc_docking 의 수치 코어 (§10 의 inner 문제, E1-F13 [#739](https://github.com/hyujun/rtc-framework/issues/739)) 와 NLP search 의 탐색 코어 (§11, E1-F14 [#740](https://github.com/hyujun/rtc-framework/issues/740) — 17.11 · 17.12) 를 적는다. 계획기 · RT 쪽은 아직 구현하지 않았다 (E1-F15 – F21, epic [#621](https://github.com/hyujun/rtc-framework/issues/621)).

작성일: 2026-10-02 (v1) · 개정: 2026-10-02 (v2, v3)

## 0. 개정 요약

### 0.1 v3: 30 Hz 예측 갱신의 반영

Vision이 공 예측을 30 Hz로 계속 갱신한다는 조건을 NLP와 실행 구조에 반영했다. 이 갱신은 NLP의 변수·제약 구조를 바꾸지 않고 parameter만 바꾸므로, 문제를 **parametric NLP의 연속 해법**으로 다룬다. 각 결론은 1차 출처로 검증했고(§16), 문헌에 직접 서술이 없는 식은 유도 결과로 표기했다.

| # | 변경 | 위치 |
|---|---|---|
| 1 | 메시지 stamp $s_j$, 갱신 주기 $\Delta_m=1/30$ s, 메시지 내부 보간 규칙 | §2, §3.4 |
| 2 | 정보 시각을 $\Delta_m$ 격자와 occlusion 시각 $t_{\mathrm{occ}}$ 로 제한한 anticipated covariance | §3.5 |
| 3 | $\tau_{\mathrm{react}}$ 를 지연·수정 능력·지각 한계의 세 조건으로 결정 (bang-bang 수정 한계 유도) | §3.6 |
| 4 | 연속 예측의 jump 공분산 유도와 NIS 동치성, gating 규칙 | §3.7 |
| 5 | Hand commit을 arm commit과 분리. Timing 제약의 $\sigma_s$ 를 hand 정보 시각으로 계산 | §8.4 |
| 6 | 첫 구간 길이 $h_0$ 를 갖는 비균일 격자 | §4.1, §10 |
| 7 | 절대 시각 후보 격자, 후보별 국소 연속 포획 시각, shrinking horizon과 정확한 warm start | §11.1, §11.4, §11.5 |
| 8 | 30 Hz 실행 루프: event-trigger, 동기식(P1)/pipeline(P2), reference 기반 초기 상태, RTI, 결정 변수의 고정 순서, $t_{\mathrm{cmd}}$ 갱신 | §12.7 |
| 9 | Commit 이후 구간의 open-loop 강건성 (Schill & Buss 계열) | §14 |
| 10 | 검증 항목과 참고문헌 추가 (RTI, acados, shrinking horizon, 추정 이론, catching 계열 서지 검증) | §15, §16 |

### 0.2 v2: v1 검토 반영

v1 검토에서 지적된 사항을 모두 반영했다. 가장 큰 구조 변경은 두 가지이다. 첫째, terminal을 **entrance plane 통과 사건**으로 재정의했다. 이에 따라 chance constraint를 통과 평면 위의 lateral 분포와 통과 시각 분포로 분리했다. 둘째, 손가락을 NLP에서 분리해 **preshape schedule과 closure trigger**로 다루도록 했다. 또한 실행 구조를 UR5e position 인터페이스(`servoj`)와 기존 CLIK 기준으로 다시 썼다.

| # | 변경 | 위치 |
|---|---|---|
| 1 | Terminal을 entrance-plane crossing으로 정의. Signed-distance equality를 affine equality로 대체 | §6.1, §10 |
| 2 | Chance constraint를 crossing-plane 공분산 $\Sigma_\rho$ 와 timing 분산 $\sigma_t^2$ 로 분리 (oblique projection 유도) | §8.2–8.4 |
| 3 | Closing speed 하한 $c_{\min}$ 추가. Timing chance constraint로부터 closing speed 하한, impact bound로부터 상한을 유도 | §6.5, §8.4 |
| 4 | Open-loop 공분산 대신 commit 시점 기준 anticipated covariance 사용. Mean drift 공분산 분리 | §3.3 |
| 5 | Hand를 NLP 변수에서 제거. Preshape 시작 시각과 closure trigger를 스케줄 변수로 둠 | §1, §6.2, §11.3 |
| 6 | UR5e `servoj`/CLIK 실행 구조. Torque는 nominal feasibility 판정과 reserve margin으로만 사용 | §4.2, §12 |
| 7 | 계산 예산, 필요조건 screening, 병렬 solve, 절대 시각 기준 warm start | §11.2–11.4 |
| 8 | Capture surface와 fingertip 센서 coverage 정합. Closure trigger 두 mode 정의 | §6.2, §8.3 |
| 9 | Effective mass의 모델 계층과 에너지 상한의 단조성 정리 (armature, hand lock, closed chain) | §7.2 |
| 10 | Corridor·closing envelope를 제곱형 smooth constraint로 변경 | §6.3–6.4 |
| 11 | Pinocchio frame 규약(`LOCAL_WORLD_ALIGNED` vs `LOCAL`) 명시 | §5.2 |
| 12 | 목적함수를 한 곳에서 정의. $\ell_k\ge0$ 의 hard 분류, $w_T$ 의 의미 명시 | §9, §13 |
| 13 | 참고문헌 표기 정정, catching 계열 선행연구 추가, Post-capture reference spreading | §12.6, §16 |

## 1. 목적과 모델의 범위

이 문서는 공의 미래 궤적을 입력받아 **언제, 어디에서, 어떤 자세와 속도로 잡을 것인지**를 선택하고, 그 시점까지의 arm 궤적과 hand 스케줄을 함께 생성하는 model-predictive capture planner를 정의한다. 기본 구조는 후보 포획 시점을 선택하는 outer loop와, 각 후보에 대해 arm inverse-dynamics NMPC를 푸는 inner loop이다.

Spacecraft rendezvous/docking에서 차용하는 것은 상대상태, 접근 corridor, terminal set, 시간 선택, 확률 제약의 **구조**이다. 아래 통합식은 **설계 제안**이며, 특정 논문의 MPC를 그대로 재현한 식이 아니다.

대상 시스템은 `ur5e_p1b` 이다. UR5e(6-DoF)와 자체 개발 4-finger 핸드 P1b(cross 4-bar, 10 actuated DoF)로 구성된다. 가정은 다음과 같다.

1. **(A1) Arm.** 고정 베이스, revolute 6자유도. $q\in\mathbb R^{6}$ 는 국소적으로 유클리드 좌표로 다룬다. NLP의 결정 변수는 arm 변수뿐이다.
2. **(A2) Arm 실행.** Joint position 인터페이스를 쓴다. Planner reference는 기존 CLIK를 거쳐 `servoj` 로 전달된다. Joint torque는 직접 지령하지 않는다. Inverse dynamics는 **nominal torque feasibility** 판정에만 쓴다.
3. **(A3) Hand.** P1b는 closed-chain이며 NLP 변수가 아니다. Preshape 궤적 $\eta_{\mathrm{pre}}(\cdot)$ 는 오프라인에서 설계·검증한다. 그 시작 시각 $t_{\mathrm{ps}}$ 와 closure 지령 시각 $t_{\mathrm{cmd}}$ 가 스케줄 변수이다. 따라서 계획 구간에서 hand configuration은 시간의 기지 함수 $\eta(t)$ 이다.
4. **(A4) Estimator.** Vision PC가 horizon 시각별 공의 위치·속도·가속도와 6×6 공분산을 **30 Hz로** publish한다. 각 메시지는 카메라 측정 시각 stamp $s_j$ 를 가지며, 갱신 간격 jitter는 작다고 가정한다. 메시지의 공분산은 그 시점까지의 측정에 조건부인 필터 공분산 $\Sigma_b(\cdot\mid s_j)$ 이고 calibration되어 있다고 가정한다. 제어 측은 예측 **평균**을 그대로 사용하며 재전파하지 않는다(메시지 내부 보간만 한다, §3.4). 계획에 쓰는 공분산은 §3.5의 anticipated covariance이다. 이 계산을 vision 측에서 수행해 함께 publish하는 구성도 수학적으로 동등하다. Calibration 가정이 깨지면 §3.5와 §3.7은 근사 이상의 의미를 갖지 못한다.
5. **(A5) 무접촉 terminal.** Inner-loop 모델은 포획 직전까지 무접촉 운동이다. Terminal event는 공 중심이 hand frame의 **entrance plane**을 통과하는 사건이다. Entrance plane은 첫 접촉이 시작되는 평면의 local approximation으로 선정한다(§6.1).
6. **(A6) 오프라인 식별.** Capture 영역, velocity set, closure timing window $[\delta_{\mathrm{lo}},\delta_{\mathrm{hi}}]$, closure latency $\tau_{\mathrm{cl}}$ 는 hand 형상, 공 반지름, 접촉 후 controller를 고려해 오프라인에서 식별·검증한다. 이 조건들을 만족해도 grasp 성공이나 force closure가 보장되지는 않는다.
7. **(A7) 충격 모델.** 단일 지배 접촉의 frictionless normal impulse 근사이다. Peak force, 다중 접촉, 실리콘 변형은 이 모델로 예측하지 않는다.

핵심 최적화는 다음 bilevel 형태이다.

$$
\boxed{
T^\star=\arg\min_{T\in\mathcal T_{\mathrm{valid}}}\;J^\star(T),\qquad
J^\star(T)=\min_{\mathcal Z_T}\;
\underbrace{J_{\mathrm{motion}}+J_{\mathrm{near}}+V_f+J_{\mathrm{slack}}
+J_{\mathrm{unc}}+J_{\mathrm{time}}+J_{\mathrm{switch}}}_{J(T,\mathcal Z_T)}
}
$$

각 항은 §9에서 **한 번만** 정의한다. v1의 $J_{\mathrm{capture}}$ 는 $J_{\mathrm{near}}$ 와 $V_f$ 의 위치·속도 항에 흡수했다. $J_{\mathrm{impact}}$ 는 $V_f$ 의 한 항이다.

## 2. 표기와 시간축

| 기호 | 정의 | 차원 또는 단위 |
|---|---|---|
| $W,H$ | World frame, hand capture frame | — |
| $q,v,a,\tau$ | Arm 관절각·속도·가속도·토크 | $\mathbb R^{6}$; rad, rad/s, rad/s², N·m |
| $\eta(t)$ | Hand 관절 configuration (스케줄로 주어지는 기지 함수) | $\mathbb R^{n_f}$ |
| $s_j,\ \Delta_m$ | $j$ 번째 vision 메시지의 측정 시각 stamp, 갱신 주기 ($1/30$) | s |
| $\tau_{\mathrm{vis}}$ | 측정 시각부터 제어 측 수신까지의 지연 | s |
| $t_{\mathrm{occ}}$ | 공이 손·팔에 가려져 유효 측정이 끊기기 시작하는 시각 | s |
| $t_0$ | 새 계획의 적용 기준 시각 | s |
| $h,h_0,N$ | 예측 간격, 첫 구간 길이 ($0<h_0\le h$), 후보의 구간 수 | s, s, 정수 |
| $T=h_0+(N-1)h,\ t_c=t_0+T$ | 상대 포획 시간, nominal 절대 포획 시각 (v2의 균일 격자는 $h_0=h$ 인 특수형) | s |
| $h_c$ | Arm 제어(`servoj`) 주기 | s |
| $p_h(q),R(q)$ | Capture frame 원점, $H\to W$ 회전 | m, $SO(3)$ |
| $\hat p_b,\hat v_b$ | 공 예측 평균 위치·속도 | m, m/s |
| $\Sigma_b(t\mid s)$ | 시각 $s$ 까지의 정보로 본 $t$ 의 공 상태 공분산 | 상태 단위별 |
| $r_b,m_b$ | 공 반지름, 질량 | m, kg |
| $e_3$ | Hand frame의 바깥쪽 접근축 단위벡터 | — |
| $E_\perp=[e_1\ e_2]$ | Entrance plane 접선 기저, $P_\perp=E_\perp E_\perp^\top$ | — |
| $s_{\mathrm{ent}}$ | Entrance plane의 축 좌표 | m |
| $c$ | Entrance-axis closing speed | m/s |
| $\rho$ | Entrance plane 위 공 중심의 lateral 좌표 | m ($\mathbb R^2$) |
| $\sigma_s,\ \sigma_t$ | 축방향 위치 표준편차, 통과 시각 표준편차 | m, s |
| $[\delta_{\mathrm{lo}},\delta_{\mathrm{hi}}]$ | 통과 후 closure가 유효해야 하는 시간 창, $\Delta_{\mathrm{win}}=\delta_{\mathrm{hi}}-\delta_{\mathrm{lo}}$ | s |
| $\tau_{\mathrm{cl}},\tau_{\mathrm{det}},\tau_{\mathrm{react}}$ | Closure latency, 접촉 검출 지연, arm 수정 유효 지연 | s |
| $t_{\mathrm{ps}},T_{\mathrm{ps}}$ | Preshape 시작 시각, preshape 소요 시간 | s |
| $\Delta\tau$ | Planning torque reserve | N·m |

$\|z\|_Q^2=z^\top Qz$ 이며 모든 quadratic weight는 positive semidefinite이다. Bias force는 샘플링 간격 $h$ 와 혼동하지 않도록 $b(q,v)$ 로 쓴다. 상태는 $k=0,\ldots,N$ 에, 입력과 토크는 $k=0,\ldots,N-1$ 에 정의한다.

$$
\mathcal Z_T=\{q_k,v_k\}_{k=0}^{N}\cup\{a_k,\tau_k\}_{k=0}^{N-1}\cup\{s_k\}_{k=0}^{N-1}.
$$

$a_N,\tau_N$ 은 만들지 않는다. $x_N=(q_N,v_N)$ 은 entrance plane 통과 순간이자 접촉 직전 상태로 해석한다. Terminal slack $s_f$ 는 진단용 relaxation(§10)에서만 등장한다.

## 3. 공의 예측과 uncertainty

### 3.1 비행 모델

$$
\dot p_b=v_b,\qquad
\dot v_b=g_W+\frac{1}{m_b}f_{\mathrm{aero}}(v_b,\omega_b,\vartheta).
$$

$f_{\mathrm{aero}}=0$ 인 baseline에서는

$$
\hat p_b(t_0+\Delta t)=\hat p_b(t_0)+\Delta t\,\hat v_b(t_0)+\tfrac12\Delta t^2g_W,\qquad
\hat v_b(t_0+\Delta t)=\hat v_b(t_0)+\Delta t\,g_W .
$$

### 3.2 외생 입력으로서의 공 예측

무접촉 구간에서 로봇은 공의 궤적을 바꿀 수 없다. 따라서 예측은 최적화 변수가 아니라 외생 입력이다.

$$
z_b=\begin{bmatrix}p_b\\v_b\end{bmatrix},\qquad
z_{b,k}\sim\mathcal N\!\left(\hat z_{b,k},\Sigma_{b,k}\right),\qquad
\Sigma_{b,k}=\begin{bmatrix}\Sigma_{p,k}&\Sigma_{pv,k}\\\Sigma_{vp,k}&\Sigma_{v,k}\end{bmatrix}.
$$

Gaussian 가정과 공분산 calibration이 실제 오차와 맞지 않으면 이후 chance constraint의 확률 해석도 성립하지 않는다.

### 3.3 정보 집합과 anticipated covariance

시각 $s$ 까지의 측정을 $\mathcal I_s$ 라 하고 $\Sigma_b(t\mid s):=\operatorname{Cov}[z_b(t)\mid\mathcal I_s]$ 로 정의한다. v1은 $\Sigma_b(t_c\mid t_0)$ 즉 **open-loop** 공분산을 썼다. 그러나 receding horizon에서 terminal 오차를 결정하는 것은, 마지막으로 반영 가능한 정보로 본 예측 오차이다.

**Commit 시각.** $t_{\mathrm{commit}}=t_c-\tau_{\mathrm{react}}$ 로 둔다. $\tau_{\mathrm{react}}$ 는 새 측정이 arm terminal 상태의 수정으로 실현되기까지 필요한 최소 시간이다.

$$
\tau_{\mathrm{react}}\ \ge\ \tau_{\mathrm{est}}+\tau_{\mathrm{comm}}+\tau_{\mathrm{solve}}+\tau_{\mathrm{track}},
$$

여기서 $\tau_{\mathrm{track}}$ 는 CLIK–`servoj` 경로의 유효 추종 지연이다. 각 항은 실측으로 정한다. 이 식은 지연만의 하한이며, 수정 능력과 지각 한계를 더한 최종 결정 규칙은 §3.6에 있다.

**Riccati recursion.** 측정 주기를 $\Delta_m$, 측정 모델을 $y_j=H_m z_{b,j}+\varepsilon_j$, $\varepsilon_j\sim\mathcal N(0,R_m)$ 라 하자. 공분산은 다음과 같이 진행한다.

$$
P_{j+1}^-=F_jP_jF_j^\top+Q_j,\qquad
K_{j+1}=P_{j+1}^-H_m^\top\!\left(H_mP_{j+1}^-H_m^\top+R_m\right)^{-1},
$$

$$
P_{j+1}=(I-K_{j+1}H_m)P_{j+1}^-(I-K_{j+1}H_m)^\top+K_{j+1}R_mK_{j+1}^\top .
$$

마지막 식은 Joseph form이며 수치적으로 대칭·양반정치를 유지한다. 선형 Gaussian 모델에서 이 수열은 **측정값에 의존하지 않는다** (a; Kalman filter 표준 결과. 확인한 자료는 강의노트와 기술보고서이며, Anderson & Moore 원문의 해당 절은 미확인). 따라서 미래 측정 전에 미리 계산할 수 있다. $f_{\mathrm{aero}}\neq0$ 인 EKF에서는 $F_j,H_m$ 을 nominal 예측 궤적에서 평가하므로 근사이다.

$t_0$ 에서 $t_{\mathrm{commit}}$ 까지 위 recursion을 진행한 뒤 $t_c$ 까지 측정 없이 전파한다.

$$
\boxed{\Sigma_b^{\mathrm{ant}}(t_c):=\Sigma_b(t_c\mid t_{\mathrm{commit}})
=\Phi(t_c,t_{\mathrm{commit}})\,P_{\mathrm{commit}}\,\Phi(t_c,t_{\mathrm{commit}})^\top+Q_{\mathrm{int}}(t_c,t_{\mathrm{commit}})}.
$$

**Mean drift.** 선형 Gaussian 모델에서 law of total covariance를 쓰면, 지금 시점에서 본 미래 추정 평균의 변동 공분산은 다음과 같다 (a).

$$
\Sigma_{\mathrm{drift}}(t_c):=\operatorname{Cov}\!\left[\,\mathbb E[z_b(t_c)\mid\mathcal I_{t_{\mathrm{commit}}}]\;\middle|\;\mathcal I_{t_0}\right]
=\Sigma_b(t_c\mid t_0)-\Sigma_b^{\mathrm{ant}}(t_c)\succeq0 .
$$

$\Sigma_b^{\mathrm{ant}}$ 로 tightening하는 것은 "평균이 $\Sigma_{\mathrm{drift}}$ 만큼 움직여도 replanning으로 따라간다"는 가정을 포함한다. 이 가정은 자동으로 성립하지 않는다. 따라서 §11.1의 validity에 terminal reachability margin 조건을 둔다.

$$
m_{\mathrm{reach}}(T)\ \ge\ \kappa_m\sqrt{\lambda_{\max}\!\left(E_\perp^\top R_N^\top\Sigma_{\mathrm{drift},p}R_NE_\perp\right)} .
$$

여기서 $m_{\mathrm{reach}}$ 는 terminal lateral 위치를 그만큼 옮겨도 nominal 해가 feasible로 남는 여유이다. 예를 들어 terminal lateral 위치에 대한 sensitivity 분석이나, 오프셋된 목표로 다시 solve해서 구한다.

**Sanity check.**

- $\tau_{\mathrm{react}}\ge t_c-t_0$ 이면 측정이 반영되지 않아 $\Sigma_b^{\mathrm{ant}}=\Sigma_b(t_c\mid t_0)$, $\Sigma_{\mathrm{drift}}=0$ 이다.
- $R_m\to\infty$ 이면 같은 결과로 돌아간다.

### 3.4 30 Hz 메시지의 시간 정렬

$j$ 번째 메시지는 horizon 시각 $t_i$ 마다 $(p_i,v_i,a_i,\Sigma_i)$ 를 준다. NLP 격자 시각 $t$ 의 값은 가장 가까운 점 $t_i$ 에서 국소 전파한다.

$$
\hat p_b(t\mid s_j)=p_i+(t-t_i)v_i+\tfrac12(t-t_i)^2a_i,\qquad
\hat v_b(t\mid s_j)=v_i+(t-t_i)a_i,
$$

$$
\Sigma_b(t\mid s_j)\approx F(t-t_i)\,\Sigma_i\,F(t-t_i)^\top,\qquad
F(\Delta)=\begin{bmatrix}I&\Delta I\\0&I\end{bmatrix}.
$$

평균은 구간 내 가속도가 일정하면 정확하다. 공분산은 짧은 구간에서 process noise를 무시한 근사이다. 이는 예측의 재전파가 아니라 메시지 내부 보간이다.

매 메시지의 parameter $\mathcal P_j=\{\hat z_b(\cdot\mid s_j),\Sigma_b(\cdot\mid s_j)\}$ 에 대해, 각 cycle은 구조가 같은 문제 $\mathrm{NLP}(\mathcal P_j,\hat x_0)$ 를 푼다. 실행 방식은 §12.7에서 정한다.

### 3.5 정보 시각의 이산화와 occlusion

§3.3의 commit 시각 이후의 정보는 arm에 반영되지 않는다. 또한 정보는 $\Delta_m$ 단위로만 들어오고, 공이 손이나 팔에 가려지는 시각 $t_{\mathrm{occ}}$ 이후에는 유효 측정이 없다. 따라서 arm에 실제로 반영되는 마지막 메시지 stamp는 다음과 같다.

$$
\boxed{s^{\star}_{\mathrm{arm}}(t_c)=\max\left\{s_j:\ s_j\le t_{\mathrm{occ}},\ \ s_j+\tau_{\mathrm{vis}}\le t_c-\tau_{\mathrm{react}}\right\}},\qquad
\Sigma_b^{\mathrm{ant}}(t_c)=\Sigma_b\!\left(t_c\mid s^{\star}_{\mathrm{arm}}\right).
$$

연속 시간 정의보다 최대 $\Delta_m$ 만큼 보수적이다. $s^{\star}_{\mathrm{arm}}=s_{j_0}+n\Delta_m$ 으로 쓰면, 현재 메시지 $j_0$ 이후 반영될 갱신 수는 $n$ 이다.

$t_{\mathrm{occ}}$ 는 카메라 배치와 포획 자세에 의존하는 설계 parameter이며, 오프라인 시뮬레이션이나 로그로 추정한다. 실제 포획 시스템에서도 이런 지각 한계가 보고되었다. 예를 들어 손 근처 occlusion 때문에 접촉 직전 일정 시간 이후 예측 갱신을 멈춘 사례(Kim et al., 2014)와, 포획 직전 일정 구간의 측정이 최종 포획 위치를 결정했다는 보고(Birbach et al., 2011)가 있다. 그 수치는 각 시스템 고유의 값이므로 본 설계에 그대로 옮기지 않는다.

**계산 경로.** 제어 측이 예측 평균을 재전파하지 않는다는 결정(A4)과 충돌하지 않도록, 제어 측에 **공분산만의 KF replica**를 둔다. 필요한 것은 $Q$, $R_m$, $\Delta_m$ 과 nominal 예측 궤적(EKF의 선형화용)뿐이다. Riccati 수열은 측정값과 무관하므로 평균 없이 계산할 수 있다. Replica의 정확성은 매 cycle 검사한다. 시각 $s_j$ 에서 replica가 계산한 $\Sigma(t\mid s_j)$ 는 vision이 publish한 값과 일치해야 한다. 대안은 투척 로그에서 (관측 횟수, 예측 horizon) → 공분산의 lookup을 만드는 것이다.

### 3.6 $\tau_{\mathrm{react}}$ 의 결정: 지연, 수정 능력, 지각

**지연.** 새 측정이 reference에 반영되기까지의 지연은

$$
\tau_{\mathrm{lat}}=\tau_{\mathrm{vis}}+\tau_{\mathrm{solve}}+\tau_{\mathrm{track}}
$$

이다. §12.7의 pipeline 방식(P2)에서는 $\tau_{\mathrm{solve}}$ 가 메시지 주기보다 길 수 있으며, 그 값을 그대로 더한다.

**수정 능력.** 남은 시간 $T_{\mathrm{rem}}$ 동안 hand frame lateral 가속도 여유 $a_{\mathrm{lat}}$ 로, 종단 속도를 바꾸지 않고 종단 위치를 $\delta$ 만큼 옮기는 문제를 보자. 1차원 이중적분기에서 $|a|\le a_{\mathrm{lat}}$, 시작·종단 속도 변화 0 조건 하의 최대 변위는 $T/2$ 가속 후 $T/2$ 감속하는 bang-bang 프로파일이 주며, 다음이 필요충분조건이다 (a).

$$
\delta\le\tfrac14a_{\mathrm{lat}}T_{\mathrm{rem}}^2 .
$$

한 번의 갱신이 만드는 평균 jump의 크기는 §3.7의 식으로 주어진다. 따라서 메시지 $s_j$ 의 정보를 arm이 따라갈 수 있으려면 다음이 필요하다 (c).

$$
\kappa_m\sqrt{\lambda_{\max}\!\Big(E_\perp^\top R_N^\top\big[\Sigma_b(t_c\mid s_j)-\Sigma_b(t_c\mid s_{j+1})\big]_pR_NE_\perp\Big)}
\ \le\ \tfrac14a_{\mathrm{lat}}\big(t_c-s_j-\tau_{\mathrm{lat}}\big)^2 .
$$

좌변은 남은 시간이 줄수록 빠르게 작아지고, 우변은 $T_{\mathrm{rem}}^2$ 로 작아진다. 이 조건을 만족하는 마지막 $s_j$ 가 수정 능력이 정하는 정보 한계이다. $a_{\mathrm{lat}}$ 는 §4.2의 torque reserve와 가속도 bound로부터 정하는 설계 parameter이다.

**결정.** $\tau_{\mathrm{react}}$ 는 세 조건을 모두 만족하도록 정한다.

$$
\tau_{\mathrm{react}}\ \ge\ \max\left\{\tau_{\mathrm{lat}},\ \ t_c-s_j^{\mathrm{corr}},\ \ t_c-t_{\mathrm{occ}}\right\},
$$

여기서 $s_j^{\mathrm{corr}}$ 는 위 수정 능력 조건을 만족하는 마지막 stamp이다.

### 3.7 연속 예측의 일관성: jump 공분산과 NIS gating

같은 미래 시각 $t_c$ 에 대한 연속 예측의 차이를

$$
d_j:=\hat z_b(t_c\mid s_{j+1})-\hat z_b(t_c\mid s_j)
$$

로 둔다. 두 메시지의 예측은 §3.4로 같은 $t_c$ 에 보간한다.

**유도 (c, 표준 결과로부터).**

1. $\hat z_b(t_c\mid s_j)=\mathbb E[z_b(t_c)\mid\mathcal I_{s_j}]$ 는 tower property에 의해 $j$ 에 대한 martingale이다 (a). 따라서 $\mathbb E[d_j\mid\mathcal I_{s_j}]=0$ 이고 증분들은 서로 무상관이다.
2. Law of total covariance에 의해 다음이 성립한다 (a).

   $$
   \Sigma_b(t_c\mid s_j)=\mathbb E\big[\Sigma_b(t_c\mid s_{j+1})\,\big|\,\mathcal I_{s_j}\big]+\operatorname{Cov}\big[d_j\,\big|\,\mathcal I_{s_j}\big].
   $$

3. 선형 Gaussian 모델에서는 $\Sigma_b(t_c\mid s_{j+1})$ 가 측정값과 무관한 결정적 값이므로 기대값이 사라진다.

   $$
   \boxed{\operatorname{Cov}[d_j]=\Sigma_b(t_c\mid s_j)-\Sigma_b(t_c\mid s_{j+1})\succeq0}.
   $$

이 식은 필터가 모델과 일치하고 최적일 때만 성립한다. 공기저항이나 spin 모델이 틀린 EKF에서는 근사이다. 문헌에서 이 식을 직접 서술한 출처는 찾지 못했다. 1차원 탄도 KF(30 Hz 위치 측정)의 Monte Carlo로 수치 일치를 확인했다(§15).

**NIS와의 동치 (a).** Kalman 갱신에서 미래 시각 예측의 jump는 innovation $\nu_{j+1}$ 의 선형 사상이다.

$$
d_j=A_j\nu_{j+1},\qquad A_j=\Phi(t_c,s_{j+1})K_{j+1},\qquad \operatorname{Cov}[d_j]=A_jS_{j+1}A_j^\top .
$$

Stereo처럼 3차원 위치를 측정하고 위치 블록 $A_{j,p}$ (3×3)가 가역이면

$$
d_{j,p}^\top\left(A_{j,p}S_{j+1}A_{j,p}^\top\right)^{-1}d_{j,p}=\nu_{j+1}^\top S_{j+1}^{-1}\nu_{j+1},
$$

즉 jump 기반 검사는 표준 NIS(normalized innovation squared)와 같은 값이다.

**Gating 규칙.**

1. Vision 노드가 NIS를 계산할 수 있으면 함께 publish하고, 제어 측은 그 값을 쓴다.
2. 그렇지 않으면 제어 측에서 재구성한다.

   $$
   \chi_j^2=d_{j,p}^\top\Big(\big[\Sigma_b(t_c\mid s_j)-\Sigma_b(t_c\mid s_{j+1})\big]_p+\varepsilon I\Big)^{-1}d_{j,p},\qquad
   \chi_j^2\le\chi^2_{3,1-\alpha}.
   $$

   $\varepsilon I$ 는 공분산 차이가 거의 singular할 때를 위한 regularization이다. 보간 오차와 $A_{j,p}$ 의 조건수 때문에 이 재구성은 근사이다.

3. 임계값을 넘는 갱신은 NLP에 넣지 않고 hold한다. 연속으로 넘으면 catch를 취소한다(§12.5). 원인으로는 예측 모델 불일치(spin, drag), 측정의 잘못된 결합(occlusion 후 재연결, 반사), bounce 같은 사건이 있다.

공분산 calibration이 틀리면 이 검사는 과민하거나 둔감해진다. 따라서 먼저 로그로 NEES/NIS 일관성을 확인한다(§15).

## 4. Arm prediction과 inverse dynamics

### 4.1 Acceleration-level 상태 전이

입력 $a_k$ 를 구간 $[t_k,t_{k+1})$ 에서 상수로 둔다.

$$
\boxed{q_{k+1}=q_k+hv_k+\tfrac12h^2a_k},\qquad
\boxed{v_{k+1}=v_k+ha_k},\qquad q_0=\hat q(t_0),\ v_0=\hat v(t_0).
$$

이는 관절 가속도가 정확히 실현된다는 prediction model이며, piecewise-constant acceleration에 대해서는 정확한 이산화이다.

**비균일 첫 구간 (v3).** 후보를 절대 시각 격자에 고정하면(§11.1) $t_0$ 은 격자 위에 있지 않다. 따라서 첫 구간만 길이 $h_0=t_1-t_0\in(0,h]$ 를 갖는다.

$$
q_1=q_0+h_0v_0+\tfrac12h_0^2a_0,\qquad v_1=v_0+h_0a_0,\qquad
h_k=\begin{cases}h_0,&k=0\\h,&k\ge1\end{cases}.
$$

이후 모든 식의 $h$ 는 구간별 $h_k$ 로 읽는다. Running cost의 적분 가중치, discrete jerk의 분모, 구간 내부 극값 검사가 여기에 해당한다. $h_0$ 는 cycle마다 바뀌는 parameter일 뿐 문제 구조를 바꾸지 않는다.

### 4.2 Inverse dynamics와 nominal torque feasibility

무접촉 EOM과 stage별 토크는 다음과 같다.

$$
M(q)a+b(q,v)=\tau,\qquad b(q,v)=C(q,v)v+g(q),\qquad
\boxed{\tau_k=\operatorname{RNEA}(q_k,v_k,a_k)=M(q_k)a_k+b(q_k,v_k)}.
$$

UR5e는 position 인터페이스로 구동되며 토크는 내부 제어기가 생성한다. 따라서 $\tau_k$ 는 지령이 아니라 **nominal 모델에서 그 가속도를 내는 데 필요한 토크**이다. 내부 feedback, 모델 오차, 충격 직전의 보정분을 위해 reserve $\Delta\tau\ge0$ 를 둔다.

$$
\boxed{\tau_{\min}+\Delta\tau\ \le\ M(q_k)a_k+b(q_k,v_k)\ \le\ \tau_{\max}-\Delta\tau}.
$$

이는 EOM과 torque limit을 결합한 state-dependent acceleration feasibility이다. $M$ 이 관절을 결합하므로 관절별 독립 가속도 bound로 치환하지 않는다. 별도의 box constraint $a_{\min}\le a_k\le a_{\max}$ 는 기계·controller 가속도 제한으로 함께 둔다. 마찰을 포함하려면 $b$ 와 RNEA 구현에 일관되게 추가한다.

Explicit formulation은 $\tau_k$ 를 변수로 두고 위 equality를 유지한다. Condensed formulation은 $\tau_k$ 를 RNEA로 소거한다. 두 방식은 동일한 모델과 제약에서 동등하다.

### 4.3 Node 제약과 구간 내부 검증

$$
q_{\min}\le q_k\le q_{\max},\quad
v_{\min}\le v_k\le v_{\max},\quad
a_{\min}\le a_k\le a_{\max}.
$$

Node feasibility가 구간 전체 feasibility를 보장하지는 않는다. 관절 $j$ 에서 $q_j(t_k+s)=q_{j,k}+sv_{j,k}+\tfrac12s^2a_{j,k}$ 의 내부 극값은 $s^{\star}=-v_{j,k}/a_{j,k}\in(0,h)$ 일 때 존재한다. 그 값도 bound 검사에 포함한다. Collision과 torque는 구간 내부 sampling 또는 margin으로 확인한다.

Discrete jerk(input slew rate)는

$$
j_0=\frac{a_0-a_{\mathrm{prev}}}{h},\qquad j_k=\frac{a_k-a_{k-1}}{h}\ (k=1,\ldots,N-1),\qquad
-j_{\max}\le j_k\le j_{\max}.
$$

$a_{\mathrm{prev}}$ 는 실제 적용 중인 reference의 직전 가속도이다. 연속 jerk 보장이 필요하면 jerk를 입력으로 두고 가속도를 상태로 추가한다.

### 4.4 연속 시간 reference

계획 해는 구간별 2차 다항식으로 정확히 표현된다.

$$
q^\star(t)=q_k+sv_k+\tfrac12s^2a_k,\quad v^\star(t)=v_k+sa_k,\quad a^\star(t)=a_k,\qquad
t=t_k+s,\ s\in[0,h).
$$

제어 주기 $h_c$ 로 샘플링해 §12의 실행 경로에 전달한다. 별도의 spline 재보간은 하지 않는다.

## 5. 손 좌표계의 상대 위치와 속도

### 5.1 상대 운동학

World-aligned Jacobian $J_p,J_\omega$ 로 쓰면

$$
v_h=J_p(q)v,\qquad\omega_h=J_\omega(q)v,\qquad\dot R=[\omega_h]_\times R .
$$

상대량을 다음과 같이 정의한다.

$$
r^W=\hat p_b-p_h(q),\qquad w^W=\hat v_b-J_p(q)v,\qquad \boxed{r^H=R^\top r^W}.
$$

Transport theorem을 적용한다. $\dot R^\top=-R^\top[\omega_h]_\times$ 와 $R^\top(\omega\times r)=(R^\top\omega)\times(R^\top r)$ 를 쓰면

$$
\boxed{\nu^H:=\dot r^H=R^\top w^W-[\omega_h^H]_\times r^H},\qquad \omega_h^H=R^\top\omega_h .
$$

$\nu^H$ 는 회전하는 capture frame 안에서 공 중심이 실제로 움직이는 속도이다.

### 5.2 Pinocchio frame 규약

$R=$ `data.oMf[H].rotation` 이다. 위 식의 $J_p,J_\omega$ 는 `getFrameJacobian(model, data, H, LOCAL_WORLD_ALIGNED)` 의 앞 3행(linear)과 뒤 3행(angular)이다. `LOCAL` Jacobian을 쓰면 $J^{\mathrm{L}}=\operatorname{blkdiag}(R^\top,R^\top)\,J^{\mathrm{LWA}}$ 이므로 식이 다음과 같이 바뀐다.

$$
\nu^H=R^\top\hat v_b-J_p^{\mathrm L}v-[\omega_h^H]_\times r^H,\qquad \omega_h^H=J_\omega^{\mathrm L}v .
$$

두 규약을 섞으면 회전 중 상대속도가 체계적으로 틀린다. §15의 finite-difference 검사로 확인한다.

### 5.3 접근축과 closing speed

공은 $+e_3$ 측에서 $-e_3$ 방향으로 들어온다. 축 좌표, lateral 좌표, closing speed를 다음과 같이 정의한다.

$$
s=e_3^\top r^H,\qquad \rho=E_\perp^\top r^H\in\mathbb R^2,\qquad
\boxed{c=-\dot s=-e_3^\top\nu^H}.
$$

$c>0$ 이면 접근, $c<0$ 이면 이탈이다. World 접근축은 $d=Re_3$ 이다.

**Sanity check.** 손이 정지해 있고 $r^H=se_3$, $\hat v_b^H=-Ve_3$ 이면 $c=V>0$ 이다.

## 6. Capture geometry, hand schedule, corridor, closing envelope

### 6.1 Entrance plane과 terminal event

Hand frame의 entrance plane을 $\{r^H:e_3^\top r^H=s_{\mathrm{ent}}\}$ 로 둔다. Terminal은 nominal 공 중심이 이 평면에 도달하는 사건으로 정의한다.

$$
\boxed{h_{\mathrm{ent}}(q_N):=e_3^\top R(q_N)^\top\!\left(\hat p_{b,N}-p_h(q_N)\right)-s_{\mathrm{ent}}=0}.
$$

이 식은 $q_N$ 에 대해 smooth하다. 따라서 v1의 signed-distance equality $g_{\mathrm{contact}}=0$ 을 대체한다.

**Lateral capture set.** 통과 시 lateral 좌표의 허용 영역을 ball-center 좌표로 정의한다.

$$
\mathcal C_\perp=\{\rho\in\mathbb R^2:\ \tilde a_i^\top\rho\le\tilde b_i,\ i=1,\ldots,m_\perp\}.
$$

Hand 입구의 개구부(volume) 기준으로 측정했다면 ball-radius erosion을 적용한다. 공 전체가 들어가야 하므로 $\mathcal C_\perp=\mathcal C_{\perp,\mathrm{open}}\ominus\mathcal B(r_b)$, 즉 $\tilde b_i=\tilde b_i^{\mathrm{open}}-r_b\|\tilde a_i\|_2$ 이다. 이미 ball-center 기준으로 식별했다면 반지름을 다시 빼지 않는다.

**무접촉 일관성 조건.** Entrance plane은 다음을 만족하도록 오프라인에서 선정·검증한다. Hand가 $\eta_{\mathrm{ready}}$ 에 있고 $\rho\in\mathcal C_\perp$, $\nu\in\mathcal V_{\mathrm{cap}}$ 인 모든 통과에 대해, $s>s_{\mathrm{ent}}$ 인 동안 공이 어떤 hand 표면과도 접촉하지 않아야 한다. 이 조건이 성립해야 terminal 이전 구간을 무접촉 동역학으로 다룰 수 있다.

### 6.2 Hand schedule과 closure trigger

Hand configuration은 시간의 기지 함수이다.

$$
\eta(t)=\begin{cases}
\eta_{\mathrm{open}}, & t<t_{\mathrm{ps}},\\
\eta_{\mathrm{pre}}(t-t_{\mathrm{ps}}), & t_{\mathrm{ps}}\le t<t_{\mathrm{ps}}+T_{\mathrm{ps}},\\
\eta_{\mathrm{ready}}, & t_{\mathrm{ps}}+T_{\mathrm{ps}}\le t<t_{\mathrm{cmd}},
\end{cases}\qquad
\boxed{t_{\mathrm{ps}}+T_{\mathrm{ps}}\le t_c-\tau_{\mathrm{rdy}}}.
$$

$\tau_{\mathrm{rdy}}\ge0$ 는 preshape 완료 후 정착 여유이다. 이 조건은 NLP가 아니라 outer loop에서 검사한다(§11.3).

통과 후 closure가 유효해지는 시각을 $t_{\mathrm{cl}}$, 실제 통과 시각을 $t_x$ 라 하자. Capture 조건은 다음과 같다.

$$
\delta_{\mathrm{lo}}\ \le\ t_{\mathrm{cl}}-t_x\ \le\ \delta_{\mathrm{hi}} .
$$

$\delta_{\mathrm{lo}}\ge0$ 은 공이 충분히 들어오기 전에 닫히지 않을 조건이고, $\delta_{\mathrm{hi}}$ 는 공이 튕겨 나가기 전에 닫힐 조건이다. 두 값은 (A6)에 따라 식별한다. Trigger는 두 mode 중 하나를 쓴다.

- **(M1) Contact-triggered.** $t_{\mathrm{cl}}=t_x+\tau_{\mathrm{det}}+\tau_{\mathrm{cl}}$. 이 mode는 첫 접촉이 **감지 가능한 표면**에서 일어날 때만 성립한다. 실기 접촉 신호가 fingertip 센서뿐이므로, 센서 coverage를 entrance plane에 사영한 영역을 $\mathcal C_{\mathrm{sens}}\subseteq\mathcal C_\perp$ 라 할 때 다음 두 조건이 필요하다.

  $$
  \Pr\{\rho\in\mathcal C_{\mathrm{sens}}\}\ge1-\epsilon_s,\qquad
  \delta_{\mathrm{lo}}\le\tau_{\mathrm{det}}+\tau_{\mathrm{cl}}\le\delta_{\mathrm{hi}} .
  $$

- **(M2) Time-triggered.** $t_{\mathrm{cl}}=t_{\mathrm{cmd}}+\tau_{\mathrm{cl}}$ 이며 $t_{\mathrm{cmd}}$ 는 예측으로 정한다(§8.4). 첫 접촉 표면이 센서 coverage 밖이면 이 mode가 **필수**이다. 이 경우 접촉 신호는 closure 성공 확인과 post-capture 전환에만 쓴다.

Mode 선택은 capture surface와 센서 배치로 결정한다. Planner는 선택된 mode의 조건을 validity에 포함한다.

### 6.3 유한 입구를 갖는 approach corridor (smooth form)

Gap을 $\ell=s-s_{\mathrm{ent}}$ 로 둔다. Approach stage index set $\mathcal A_T\subseteq\{0,\ldots,N-1\}$ 에서

$$
\boxed{\|\rho_k\|_2^2\le\left(r_{\mathrm{ent}}+\ell_k\tan\theta+s_{c,k}\right)^2},\qquad
\ell_k\ge0,\quad s_{c,k}\ge0 .
$$

$\ell_k\ge0$ 이면 우변 괄호가 음이 아니므로 v1의 norm 형식과 동치이다. 또한 $\rho_k=0$ 에서도 미분 가능하다. $r_{\mathrm{ent}}$ 는 공 반지름을 반영한 허용 center offset이다. $\ell_k\ge0$ 은 "terminal 이전에는 공이 입구를 통과하지 않는다"는 무접촉 가정의 일관성 조건이므로 **hard**로 둔다. Corridor는 ball–hand collision 검사를 대체하지 않는다.

$\mathcal A_T$ 는 online phase machine이 정하고 inner solve 동안 고정한다. Decision-dependent 조건 $\ell<\ell_{\mathrm{activate}}$ 를 solver 안에 직접 넣으면 disjunctive 문제가 된다.

### 6.4 Closing-speed envelope (smooth form)

$$
\boxed{c_k^2\le c_{\mathrm{ent,max}}^2+2a_{\mathrm{brake}}\ell_k+s_{v,k}},\qquad s_{v,k}\ge0,\quad k\in\mathcal A_T .
$$

Slack 단위는 m²/s²이다. v1의 $\sqrt{\cdot}$ 형식은 $c_{\mathrm{ent,max}}\to0$, $\ell\to0$ 에서 기울기가 발산하므로 사용하지 않는다. 이 envelope는 상대 운동의 유효 감속 능력을 상수 $a_{\mathrm{brake}}$ 로 근사한 soft 설계 조건이다. $\ddot s$ 는 공 가속도, 로봇 가속도, frame 회전에 모두 의존하므로 joint acceleration limit만으로 $a_{\mathrm{brake}}$ 를 정하지 않는다. 실제 도달 가능성은 dynamics optimization이 판단한다.

### 6.5 Terminal velocity set

$$
\mathcal V_{\mathrm{cap}}=\left\{\nu:\ c_{\min}\le-e_3^\top\nu\le c_{\mathrm{cap,max}},\ \ \|E_\perp^\top\nu\|_2^2\le v_{\perp,\max}^2\right\},\qquad c_{\min}>0 .
$$

하한 $c_{\min}>0$ 은 §8.2의 crossing-plane 변환이 잘 정의되기 위한 조건이다($c\to0$ 이면 발산). §8.4의 timing chance constraint는 이보다 강한, 공분산에 의존하는 하한을 준다.

## 7. Impact-aware terminal model

### 7.1 접촉점과 approaching mode

예상 첫 접촉점을 $p_c(q,\eta_{\mathrm{ready}})$, world translational contact Jacobian을 $J_c$ 라 한다. 이는 일반적으로 $J_p$ 와 다르다. World normal $n$ 은 손 표면에서 공 쪽을 향한다. 구의 frictionless radial contact에서 공의 각속도는 normal 상대속도에 기여하지 않는다.

$$
g_n=n^\top(\hat v_b-J_cv) .
$$

Terminal mode는 approaching contact로 고정한다. 즉 $g_{n,N}\le0$ 을 제약으로 두고 $c_{n}:=-g_{n}$ 으로 쓴다. 이렇게 하면 v1의 $\max(0,\cdot)$ 비평활성이 사라진다.

### 7.2 Effective mass의 모델 계층

**기본형.** Normal inverse inertia와 reduced mass는

$$
\beta_h=n^\top J_cM^{-1}J_c^\top n\ \ge0,\qquad
\boxed{m_{\mathrm{red}}=\left(\frac1{m_b}+\beta_h\right)^{-1}}.
$$

$M^{-1}$ 을 직접 만들지 않고 $My=J_c^\top n$ 을 풀어 $\beta_h=n^\top J_cy$ 를 계산한다.

**Closed-chain hand 포함.** Loop constraint Jacobian $J_\ell$ (full row rank)를 갖는 전체 모델에서 충격 동역학은 다음과 같다.

$$
M\Delta v=J_c^\top nP+J_\ell^\top\Lambda,\qquad J_\ell\Delta v=0 .
$$

$\Lambda$ 를 소거하면

$$
\Lambda=-\left(J_\ell M^{-1}J_\ell^\top\right)^{-1}J_\ell M^{-1}J_c^\top nP .
$$

따라서 normal 방향 속도 변화는 $n^\top J_c\Delta v=\beta_h^{\mathrm{cc}}P$ 이고,

$$
\beta_h^{\mathrm{cc}}=n^\top J_c\left[M^{-1}-M^{-1}J_\ell^\top\left(J_\ell M^{-1}J_\ell^\top\right)^{-1}J_\ell M^{-1}\right]J_c^\top n .
$$

**에너지 상한의 단조성 (a).** 다음 두 성질이 성립한다.

- $M_1\succeq M_2\succ0$ 이면 $M_1^{-1}\preceq M_2^{-1}$ 이다. 따라서 관성을 더하면(예: rotor reflected inertia, Pinocchio `model.armature`) $\beta_h$ 는 감소한다.
- 구속을 더하면 위 괄호 안에서 PSD 항을 빼게 되므로 $\beta_h$ 는 감소한다. 예를 들어 position 제어로 hand 관절이 사실상 고정되는 경우가 그렇다.

$\beta_h$ 가 감소하면 $m_{\mathrm{red}}$ 와 충격 에너지가 증가한다. 따라서 동일한 $c_n$ 에 대해 다음 순서가 성립한다.

$$
E^{\mathrm{free\ links}}\ \le\ E^{\mathrm{+armature}}\ \le\ E^{\mathrm{+hand\ locked}}\ \le\ \tfrac12m_bc_n^2 .
$$

마지막 식은 $\beta_h\ge0\Rightarrow m_{\mathrm{red}}\le m_b$ 에서 나오며, **모델에 무관한 상한**이다.

**모델 선택.** 실제 충격 시간 척도에서 감속기 탄성이 rotor를 분리하는지는 하드웨어에 따라 다르다. 이 효과는 로봇 충돌 안전 문헌에서 다뤄진 것으로 알고 있으나, 본 문서에서 해당 출처를 검증하지 않았다(**확인 필요**). 또한 `servoj` 의 고이득 position loop는 충격 순간 로봇이 자유 관성계라는 가정과 맞지 않는다. 따라서 식별 전에는 보수적 모델(armature 포함, hand locked)로 $E_{\max}$ 조건을 검사하고, 불확실성이 크면 $\tfrac12m_bc_n^2$ 를 쓴다.

### 7.3 충격 에너지, impulse, closing speed 상한

$$
\boxed{E_n^-=\tfrac12m_{\mathrm{red}}c_n^2},\qquad
\boxed{P_n=(1+e)m_{\mathrm{red}}c_n},\qquad e\in[0,1].
$$

손실 에너지는 $(1-e^2)E_n^-$ 이다. Peak force는 contact stiffness, damping, duration 없이 얻을 수 없다. $E_n^-\le E_{\max}$ 와 $P_n\le P_{\max}$ 는 각각 normal closing speed의 상한과 동치이다.

$$
c_n\ \le\ c_{n,\mathrm{hi}}:=\min\!\left\{\sqrt{\frac{2E_{\max}}{m_{\mathrm{red}}}},\ \frac{P_{\max}}{(1+e)m_{\mathrm{red}}}\right\}.
$$

**Sanity check.**

- $c_n=0$ 이면 $E=P=0$ 이다.
- $m_{h,\mathrm{eff}}\to\infty$ 이면 $m_{\mathrm{red}}\to m_b$, $m_{h,\mathrm{eff}}\to0$ 이면 $m_{\mathrm{red}}\to0$ 이다.
- 단위는 $E$ 가 J, $P$ 가 N·s이다.

이 bound는 충격 안전의 충분조건이 아니다. 다중 손가락 동시 접촉, servo의 impulsive response, 실리콘 compliance가 중요하면 다중 접촉/변형 모델로 확장한다.

## 8. Probabilistic capture constraints

### 8.1 Hand-frame covariance

Robot 상태를 조건부로 고정하고 §3.3의 $\Sigma_b^{\mathrm{ant}}$ 를 쓴다. 이하 $\Sigma_{b,N}:=\Sigma_b^{\mathrm{ant}}(t_c)$ 로 표기한다.

$$
y_N=\begin{bmatrix}r_N^H\\\nu_N^H\end{bmatrix},\qquad
L_N=\begin{bmatrix}R_N^\top&0\\-[\omega_{h,N}^H]_\times R_N^\top&R_N^\top\end{bmatrix},\qquad
\Sigma_{y,N}=L_N\Sigma_{b,N}L_N^\top=\begin{bmatrix}\Sigma_{r,N}^H&\Sigma_{r\nu,N}^H\\\Sigma_{\nu r,N}^H&\Sigma_{\nu,N}^H\end{bmatrix}.
$$

$L_N$ 의 블록은 $\partial\nu^H/\partial p_b=-[\omega_h^H]_\times R^\top$, $\partial\nu^H/\partial v_b=R^\top$ 에서 나온다. Robot tracking 불확실성이 무시할 수 없으면 $y=f(z_b,x_r)$ 를 선형화한다.

$$
\Sigma_y\approx F_b\Sigma_bF_b^\top+F_r\Sigma_rF_r^\top+F_b\Sigma_{br}F_r^\top+F_r\Sigma_{rb}F_b^\top .
$$

독립성이 확인된 경우에만 cross covariance를 0으로 둔다.

### 8.2 Crossing-plane 분포의 유도

**문제.** v1은 고정 시각 $T$ 에서 $\Pr\{r_N^H\in\mathcal C\}$ 를 제약했다. 그러나 terminal은 사건이므로, 실제 공은 평면을 $t_x=t_c+\delta t$ 에 통과한다. 축방향 위치 오차는 사실상 통과 시각 오차이고, lateral 상대속도를 통해 통과 위치 오차로 바뀐다.

**가정.** 통과 직전 짧은 구간에서 상대 가속도 영향을 무시한다.

$$
r^H(t_c+\delta)\approx r_N^H+\nu_N^H\,\delta .
$$

**유도.** 실제 상태를 $r_N^H=\hat r_N^H+\delta r$ 로 쓴다. Terminal equality(§6.1)에 의해 $e_3^\top\hat r_N^H=s_{\mathrm{ent}}$ 이다.

1. 통과 조건 $e_3^\top(r_N^H+\nu\,\delta t)=s_{\mathrm{ent}}$ 와 $e_3^\top\nu=-c$ 로부터

   $$
   \delta t=\frac{e_3^\top r_N^H-s_{\mathrm{ent}}}{c}=\frac{e_3^\top\delta r}{c}.
   $$

2. 통과 위치는 $r_x=r_N^H+\nu\,\delta t$ 이다. $\nu=\hat\nu+\delta\nu$ 로 두면 $\delta\nu\,\delta t$ 는 2차 항이므로 1차까지

   $$
   \delta r_x=\Pi\,\delta r,\qquad \boxed{\Pi=I+\frac{\hat\nu\,e_3^\top}{\hat c}},\qquad \hat c=-e_3^\top\hat\nu .
   $$

   Nominal 통과가 정확히 $T$ 에서 일어나도록 정했기 때문에, 속도 불확실성은 1차에서 통과 위치에 들어오지 않는다.

3. $e_3^\top\Pi=e_3^\top-e_3^\top=0$ 이므로 $\delta r_x$ 는 평면 위에 있다. $\Pi$ 는 $\hat\nu$ 방향을 따라 평면으로 내리는 **oblique projection**이다.

**결과.** 다음 세 양이 crossing-plane 분포를 결정한다.

$$
\boxed{\Sigma_\rho=E_\perp^\top\Pi\,\Sigma_{r,N}^H\,\Pi^\top E_\perp},\qquad
\boxed{\sigma_s^2=e_3^\top\Sigma_{r,N}^He_3=d_N^\top\Sigma_{p,N}d_N},\qquad
\sigma_t=\frac{\sigma_s}{\hat c},
$$

$$
\operatorname{Cov}(\delta\rho,\delta t)=\frac{1}{\hat c}E_\perp^\top\Pi\,\Sigma_{r,N}^He_3 ,
$$

여기서 $d_N=R_Ne_3$ 이다.

**Sanity check.**

- $\hat\nu\parallel e_3$ 이면 $E_\perp^\top\Pi=E_\perp^\top$ 이고, $\Sigma_\rho$ 는 고정 시각 lateral 공분산과 같다.
- $\Sigma=0$ 이면 deterministic 조건으로 돌아간다.
- $\hat c\to0$ 이면 $\Pi$ 와 $\sigma_t$ 가 발산한다. Grazing 접근에서는 통과 위치와 시각이 정의되지 않는다는 물리적 사실과 일치한다.

**선형화 유효 조건.** 무시한 2차 항의 크기는 $\tfrac12\|E_\perp^\top a_{\mathrm{rel}}\|\,\delta t^2$ 이다. $\delta t$ 를 $\kappa\sigma_t$ 로 잡아 다음을 검사한다.

$$
\tfrac12\|E_\perp^\top a_{\mathrm{rel},N}\|(\kappa\sigma_t)^2\ \le\ \varepsilon_{\mathrm{lin}}\min_i\frac{\tilde b_i-\tilde a_i^\top\hat\rho_N}{\|\tilde a_i\|_2} .
$$

$a_{\mathrm{rel}}$ 는 공 가속도(중력 포함), hand 가속도, frame 회전 항을 포함한 hand-frame 상대 가속도이다. $\varepsilon_{\mathrm{lin}}<1$ 은 설계 상수이다. 위반 시 2차 보정이나 scenario 검증을 쓴다.

### 8.3 Lateral capture, sensor coverage, velocity의 tightening

**Lateral capture.** Face별 risk budget $\epsilon_i>0$, $\sum_i\epsilon_i\le\epsilon_c$ 를 배정한다. Gaussian affine marginal과 Boole 부등식에 의해 다음은 $\Pr\{\rho\in\mathcal C_\perp\}\ge1-\epsilon_c$ 의 충분조건이다 (a).

$$
\boxed{\tilde a_i^\top\hat\rho_N+\kappa_i\sqrt{\tilde a_i^\top\Sigma_\rho\tilde a_i}\le\tilde b_i},\qquad \kappa_i=\Phi^{-1}(1-\epsilon_i),\qquad \hat\rho_N=E_\perp^\top\hat r_N^H .
$$

각 face에서는 정확한 scalar 변환이고, 보수성은 risk allocation에서만 생긴다. Face 간 독립성은 필요 없다. $\Sigma_\rho$ 가 singular일 수 있는 방향에서는 $\sqrt{\cdot}$ 의 기울기를 위해 $\sqrt{\tilde a_i^\top\Sigma_\rho\tilde a_i+\varepsilon_\sigma^2}$ 로 regularize한다.

**Sensor coverage (M1).** $\mathcal C_{\mathrm{sens}}=\{\rho:\bar a_i^\top\rho\le\bar b_i\}$ 에 같은 식을 risk budget $\epsilon_s$ 로 적용한다.

**Terminal velocity.** 축방향 성분은 affine이다.

$$
-e_3^\top\hat\nu_N\pm\kappa_\nu\sqrt{e_3^\top\Sigma_{\nu,N}^He_3}\in[c_{\min},c_{\mathrm{cap,max}}],
$$

즉 하한은 $-$ 부호, 상한은 $+$ 부호로 tightening한다. Lateral speed norm은 정 $m$ 각형 inner approximation으로 바꾼다.

$$
u_j=\begin{bmatrix}\cos(2\pi j/m)\\\sin(2\pi j/m)\end{bmatrix},\qquad
\bigcap_{j=0}^{m-1}\{x:u_j^\top x\le v_{\perp,\max}\cos(\pi/m)\}\subset\{\|x\|_2\le v_{\perp,\max}\}.
$$

각 face에 위 tightening을 적용한다. 여기서 $x=E_\perp^\top\nu_N$ 이고 공분산은 $E_\perp^\top\Sigma_{\nu,N}^HE_\perp$ 이다.

Chance constraint에 slack을 허용하면 확률 보장을 주장할 수 없다. 실행 승인에는 모든 chance constraint의 slack이 0이어야 한다.

### 8.4 Timing chance constraint와 closing speed 창

통과 시각 편차는 $\delta t\sim\mathcal N(0,\sigma_t^2)$ 이다(1차 근사). Closure latency jitter를 $\sigma_\tau$ (독립 Gaussian)라 하면 $\sigma_{\mathrm{tot}}^2=\sigma_t^2+\sigma_\tau^2$ 이다.

**(M2) Time-triggered.** $t_{\mathrm{cl}}-t_x=t_{\mathrm{cmd}}+\tau_{\mathrm{cl}}-t_c-\delta t$ 이다. 창의 중앙에 맞추는 지령 시각은

$$
\boxed{t_{\mathrm{cmd}}=t_c-\tau_{\mathrm{cl}}+\delta_{\mathrm{mid}}},\qquad \delta_{\mathrm{mid}}=\tfrac12(\delta_{\mathrm{lo}}+\delta_{\mathrm{hi}}).
$$

이때 성공 확률은 정확히

$$
\Pr\{\delta_{\mathrm{lo}}\le t_{\mathrm{cl}}-t_x\le\delta_{\mathrm{hi}}\}=2\Phi\!\left(\frac{\Delta_{\mathrm{win}}}{2\sigma_{\mathrm{tot}}}\right)-1 .
$$

따라서 $\ge1-\epsilon_t$ 조건은 $\kappa_t=\Phi^{-1}(1-\epsilon_t/2)$ 로 두었을 때 다음과 **동치**이다.

$$
\sigma_{\mathrm{tot}}\le\frac{\Delta_{\mathrm{win}}}{2\kappa_t}
\quad\Longleftrightarrow\quad
\boxed{\hat c\ \ge\ c_{t,\mathrm{lo}}:=\frac{\sigma_s}{\sqrt{\left(\Delta_{\mathrm{win}}/2\kappa_t\right)^2-\sigma_\tau^2}}},\qquad
\frac{\Delta_{\mathrm{win}}}{2\kappa_t}>\sigma_\tau .
$$

$\sigma_\tau=0$ 이면 $c_{t,\mathrm{lo}}=2\kappa_t\sigma_s/\Delta_{\mathrm{win}}$ 이다. 오른쪽 조건이 깨지면 latency jitter만으로 요구 확률을 만족할 수 없으므로 M2가 불가능하다. 우변의 $\sigma_s=\sqrt{d_N^\top\Sigma_{p,N}d_N}$ 는 $q_N$ 에 의존하는 smooth 함수이므로 NLP의 terminal 제약으로 넣는다.

**Hand 정보 시각 (v3).** Arm 궤적은 $t_c-\tau_{\mathrm{react}}$ 에서 고정되지만, M2의 closure 지령 시각 $t_{\mathrm{cmd}}$ 는 그 뒤의 메시지로도 계속 고칠 수 있다(§12.7). 손가락 지령 경로의 지연이 arm 수정 지연보다 짧기 때문이다. 따라서 timing 제약의 $\sigma_s$ 는 arm이 아니라 **hand 정보 시각**의 공분산으로 계산한다.

$$
\boxed{s^{\star}_{\mathrm{hand}}=\max\left\{s_j:\ s_j\le t_{\mathrm{occ}},\ \ s_j+\tau_{\mathrm{vis}}\le t_{\mathrm{cmd}}\right\}},\qquad
\sigma_s^2=d_N^\top\,\Sigma_p\!\left(t_c\mid s^{\star}_{\mathrm{hand}}\right)d_N .
$$

$s^{\star}_{\mathrm{hand}}\ge s^{\star}_{\mathrm{arm}}$ 이므로 $\sigma_s$ 가 작아지고, 그만큼 $c_{t,\mathrm{lo}}$ 가 낮아져 아래의 closing speed 창이 넓어진다. 다만 $t_{\mathrm{occ}}$ 이후에는 정보가 없으므로, 이 이득은 $\min(t_{\mathrm{occ}},\,t_{\mathrm{cmd}}-\tau_{\mathrm{vis}})$ 까지로 제한된다. 계획 시점에는 $t_{\mathrm{cmd}}$ 를 nominal 값 $t_c-\tau_{\mathrm{cl}}+\delta_{\mathrm{mid}}$ 로 두고 계산한다.

$$
\hat c_N\sqrt{\left(\Delta_{\mathrm{win}}/2\kappa_t\right)^2-\sigma_\tau^2}\ \ge\ \sigma_s(q_N).
$$

**(M1) Contact-triggered.** $t_{\mathrm{cl}}-t_x=\tau_{\mathrm{det}}+\tau_{\mathrm{cl}}$ 로 통과 시각 오차와 무관하다. 대신 §8.3의 coverage 조건이 확률을 담당한다. 검출·closure jitter가 있으면 $\sigma_t$ 를 0으로 둔 위 식을 쓴다.

**Closing speed 창.** 접촉 normal이 접근축과 정렬되고($n\approx d_N$), 접촉점과 capture 원점의 normal 방향 속도가 같다고 근사하면 $c_n\approx\hat c$ 이다. 그러면 M2에서 다음이 필요하다.

$$
\boxed{\max\{c_{\min},\,c_{t,\mathrm{lo}}\}\ \le\ \hat c\ \le\ \min\{c_{\mathrm{cap,max}},\,c_{n,\mathrm{hi}}\}}.
$$

Timing robustness는 빠른 접근을, impact 제한은 느린 접근을 요구한다. 이 창이 비면 arm 운동과 무관하게 후보가 불가능하다. 따라서 이 식은 outer-loop screening(§11.3)에 쓴다. NLP 안에서는 근사 대신 $c_n$ 과 $\hat c$ 에 대한 각 제약을 따로 둔다.

### 8.5 Uncertainty cost (선택)

$$
J_{\mathrm{unc}}=w_\Sigma\,\frac{\sigma_\rho^2}{\sigma_{\mathrm{ref}}^2},\qquad \sigma_\rho^2=\operatorname{tr}\Sigma_\rho .
$$

$\Sigma_\rho$ 는 $\hat\nu_N$ 과 $R_N$ 을 통해 결정 변수에 의존한다. 따라서 이 항은 inner solve에서 lateral 상대속도를 줄이는 방향, 즉 접근축 정렬 쪽으로 작용한다. Chance constraint가 이미 불확실성을 반영하므로 $w_\Sigma$ 는 선택 사항이다.

### 8.6 Ball clearance의 불확실성 margin

Capture surface 이외의 link $i$ 와 공 사이의 nominal signed clearance $d_{\mathrm{ball},i}$ 에 대해, closest-point normal $n_{i,k}$ 방향 표준편차로 tightening한다.

$$
d_{\mathrm{ball},i}(q_k,\eta(t_k),\hat p_{b,k},r_b)\ \ge\ d_{\mathrm{clear},i}+\kappa_d\sqrt{n_{i,k}^\top\Sigma_{p}(t_k\mid\cdot)\,n_{i,k}} .
$$

$\Sigma_p(t_k\mid\cdot)$ 에는 §3.3과 같은 논리로 $t_k-\tau_{\mathrm{react}}$ 기준 anticipated covariance를 쓴다. Hand 형상은 스케줄 $\eta(t_k)$ 로 평가한다.

## 9. Objective function

### 9.1 단위 normalization과 horizon 비교

서로 다른 단위의 항은 characteristic scale로 나누거나 그에 상응하는 단위의 weight를 쓴다. 후보마다 가중치와 scale을 바꾸지 않는다. Running cost에는 $h$ 를 곱해 시간 적분을 근사한다. 평균 비용이 필요하면 모든 후보에 동일하게 $1/T$ 를 적용하고 목적 변경을 명시한다.

### 9.2 Motion cost

$$
\ell_{\mathrm{motion},k}=\|\tau_k\|_{R_\tau}^2+\|a_k\|_{R_a}^2+\|j_k\|_{R_j}^2+\|q_k-q_{\mathrm{nom}}\|_{Q_q}^2+w_m\psi_m(q_k),\qquad
J_{\mathrm{motion}}=h\sum_{k=0}^{N-1}\ell_{\mathrm{motion},k}.
$$

Torque square는 effort proxy이며 에너지와 같지 않다. Manipulability regularizer는 다음과 같다.

$$
\bar J=D_x^{-1}J_{\mathrm{task}}D_q,\qquad \psi_m(q)=-\log\det(\bar J\bar J^\top+\delta I),\quad \delta>0 .
$$

6D Jacobian에서는 translation과 rotation의 scale $D_x$ 를 반드시 정한다.

### 9.3 Catch vicinity relative-state cost

$$
\rho_T(t)=\exp\!\left[-\frac{(t-T)^2}{2\sigma_T^2}\right],\qquad
J_{\mathrm{near}}=h\sum_{k=0}^{N-1}\rho_T(kh)\left(\|r_k^H-r_{\mathrm{ref},k}^H\|_{Q_p}^2+\|\nu_k^H-\nu_{\mathrm{ref}}^H\|_{Q_v}^2\right).
$$

$r_{\mathrm{ref},k}^H=r_{\mathrm{ref}}^H+(T-kh)\,(-\nu_{\mathrm{ref}}^H)$ 로 두면, terminal reference를 지나는 등속 접근선을 추종하게 된다. 이렇게 하면 terminal 이전 구간에서 위치와 속도 reference가 서로 모순되지 않는다. $\nu_{\mathrm{ref}}^H$ 는 $\mathcal V_{\mathrm{cap}}$ 안에서, §8.4의 closing speed 창 안의 축방향 성분으로 선택한다.

### 9.4 Terminal cost

$$
V_f=\|\rho_N-\rho_{\mathrm{ref}}\|_{Q_{\rho,f}}^2+\|\nu_N^H-\nu_{\mathrm{ref}}^H\|_{Q_{\nu,f}}^2
+\underbrace{w_E\,\frac{E_{n,N}^-}{E_{\mathrm{ref}}}}_{J_{\mathrm{impact}}}
\ \big[+\|e_R\|_{Q_R}^2\big],\qquad
e_R=\operatorname{Log}\!\left(R_{\mathrm{des}}^\top R(q_N)\right)^\vee .
$$

축방향 위치는 §6.1의 equality로 고정되므로 terminal 위치 cost는 lateral 좌표 $\rho$ 에만 둔다. Hand 자세는 스케줄로 정해지므로 v1의 $\eta$ 항은 제거했다. Orientation 항은 선택 사항이다. 구형 공에는 grasp 목표 orientation이 없다. v1의 $\|v_h-\alpha v_b\|^2$ 는 Galilean invariant하지 않으므로 사용하지 않는다.

### 9.5 Slack, time, switching

$$
J_{\mathrm{slack}}=h\sum_{k\in\mathcal A_T}\left(\lambda_1^\top s_k+\|s_k\|_{\Lambda_2}^2\right),\qquad s_k=\begin{bmatrix}s_{c,k}\\s_{v,k}\end{bmatrix}\ge0 .
$$

$s_c$ (m)와 $s_v$ (m²/s²)는 단위가 다르므로 성분별 scaling을 쓴다. Physical bound, EOM, collision, chance constraint에는 slack을 두지 않는다.

$$
J_{\mathrm{time}}=w_T\frac{T}{T_{\mathrm{ref}}},\qquad
J_{\mathrm{switch}}=w_{\mathrm{sw}}\left(\frac{t_0+T-t_{c,\mathrm{prev}}}{T_{\mathrm{ref}}}\right)^2 .
$$

$w_T\ge0$ 은 이른 포획을 선호한다는 뜻이다. 늦은 포획일수록 측정이 많아져 $\Sigma_b^{\mathrm{ant}}$ 가 작아지는 이점은 chance constraint가 이미 반영한다. 따라서 $w_T$ 는 workspace 경계 근처의 불필요하게 늦은 포획을 억제하는 역할로 한정한다. Switching penalty는 **절대 포획 시각**의 변화에 적용하며, deadline이나 feasibility가 바뀌면 이전 후보를 고집하지 않도록 작게 둔다.

## 10. Inner-loop NMPC의 완성된 형태

후보 $T$ 와 그에 대응하는 hand 스케줄 $\eta(\cdot)$, closure mode, 그리고 메시지 parameter $\mathcal P_j$ 가 주어졌다고 하자. Stage 제약은 $0\le k<N$, state bound는 $0\le k\le N$, approach 제약은 $k\in\mathcal A_T$ 에 적용한다.

$$
\boxed{
\begin{aligned}
J^\star(T)=\min_{\mathcal Z_T}\quad&
J_{\mathrm{motion}}+J_{\mathrm{near}}+V_f+J_{\mathrm{slack}}+J_{\mathrm{unc}}+J_{\mathrm{time}}+J_{\mathrm{switch}}\\
\mathrm{s.t.}\quad
&q_0=\hat q(t_0),\quad v_0=\hat v(t_0),\\
&q_{k+1}=q_k+h_kv_k+\tfrac12h_k^2a_k,\quad v_{k+1}=v_k+h_ka_k,\quad h_0\in(0,h],\ h_k=h\ (k\ge1),\\
&\tau_k=M(q_k)a_k+b(q_k,v_k),\quad \tau_{\min}+\Delta\tau\le\tau_k\le\tau_{\max}-\Delta\tau,\\
&q_{\min}\le q_k\le q_{\max},\quad v_{\min}\le v_k\le v_{\max},\quad a_{\min}\le a_k\le a_{\max},\quad -j_{\max}\le j_k\le j_{\max},\\
&d_i(q_k,\eta(t_k))\ge d_{\min,i}\quad\text{(self/environment)},\\
&d_{\mathrm{ball},i}(q_k,\eta(t_k),\hat p_{b,k},r_b)\ge d_{\mathrm{clear},i}+\kappa_d\sigma_{i,k}\quad\text{(capture surface 이외)},\\
&\ell_k\ge0,\quad \|\rho_k\|_2^2\le(r_{\mathrm{ent}}+\ell_k\tan\theta+s_{c,k})^2,\quad
c_k^2\le c_{\mathrm{ent,max}}^2+2a_{\mathrm{brake}}\ell_k+s_{v,k}\quad(k\in\mathcal A_T),\\
&h_{\mathrm{ent}}(q_N)=0,\\
&\tilde a_i^\top\hat\rho_N+\kappa_i\sqrt{\tilde a_i^\top\Sigma_\rho\tilde a_i}\le\tilde b_i\quad(i=1,\ldots,m_\perp),\\
&\text{(M1)}\ \ \bar a_i^\top\hat\rho_N+\kappa_{s,i}\sqrt{\bar a_i^\top\Sigma_\rho\bar a_i}\le\bar b_i,\qquad
\text{(M2)}\ \ \hat c_N\sqrt{(\Delta_{\mathrm{win}}/2\kappa_t)^2-\sigma_\tau^2}\ge\sigma_s(q_N),\\
&\nu_N^H\in\mathcal V_{\mathrm{cap}}\ \text{(§8.3의 tightened polytope)},\\
&g_{n,N}\le0,\quad \tfrac12m_{\mathrm{red}}(q_N)\,g_{n,N}^2\le E_{\max},\quad (1+e)\,m_{\mathrm{red}}(q_N)\,(-g_{n,N})\le P_{\max},\\
&s_k\ge0 .
\end{aligned}}
$$

이 문제는 nonlinear EOM, FK, 자세, collision, effective mass를 포함하는 **nonconvex NLP**이다. SQP의 각 iteration에서 QP를 풀더라도 전체가 하나의 QP가 되지는 않는다. 결정 변수는 arm 변수뿐이다. Hand 형상은 스케줄로 주어지는 데이터로서 collision과 contact geometry에만 들어간다.

Terminal chance constraint 때문에 infeasible인 후보는 실행 후보에서 제외한다. 진단 목적으로만 우변에 $+s_{f,i}$ 를 더한 relaxation을 풀 수 있으며, $s_f>0$ 인 해는 유효한 capture solution으로 선택하지 않는다.

## 11. Outer loop: 언제, 어디에서 잡을 것인가

### 11.1 후보 집합과 validity

**절대 시각 후보 격자 (v3).** 후보를 상대 시간이 아니라 절대 시각 격자에 고정한다.

$$
\mathcal T_{\mathrm{abs}}=\left\{t_c^{(i)}=t_{\mathrm{ref}}+i\,h\right\}_{i}\cap\left[t_0+T_{\min},\ t_0+T_{\max}\right],\qquad h=\Delta_m/m,\quad m\in\mathbb N .
$$

$t_{\mathrm{ref}}$ 는 한 번의 catch 시도 동안 고정한다. 그러면 cycle마다 각 후보의 남은 구간 수가 정확히 $m$ 개 줄어들고(shrinking horizon), 같은 물리적 후보가 cycle 사이에 같은 index로 유지된다. 상대 포획 시간은 $T^{(i)}=t_c^{(i)}-t_0$ 이다.

**후보별 국소 연속 포획 시각 (v3).** 문헌의 실시간 포획 시스템은 포획 시각을 NLP의 연속 결정 변수로 두고 예측 갱신마다 다시 풀었다(Bäuml et al., 2010; Abeyruwan et al., 2023). 이산 격자는 여러 local basin을 동시에 유지하는 장점이 있으므로, 두 방식을 결합한다 (c). 격자 후보는 초기값과 정체성으로만 쓰고, 각 후보 안에서 포획 시각을 국소적으로 연다.

$$
t_c\in\left[t_c^{(i)}-\tfrac h2,\ t_c^{(i)}+\tfrac h2\right],\qquad
h_{T}=\frac{t_c-t_1}{N-1}\quad(\text{첫 구간 } h_0 \text{ 이후 균등 분할}).
$$

이 경우 공 예측 $\hat z_b(t_k)$ 와 그 시간 미분이 $t_c$ 의 함수가 되며, §3.4의 국소 다항식으로 미분을 얻는다. 포획 시각을 고정하는 형태는 Diehl et al.(2005, SIAM)의 shrinking-horizon contraction 정리가 직접 다루는 설정이다. 국소 연속화를 하면 그 정리는 그대로 적용되지 않는다. 수렴이 불안정하면 국소 연속화를 끄고 고정 격자로 돌아간다.

$T_{\min}$ 은 반응·계산·통신·preshape에 필요한 시간으로 정한다. $T_{\max}$ 는 예측 신뢰 구간, workspace 체류 시간, environment collision deadline으로 정한다.

$$
\operatorname{valid}(T)=1\iff
\begin{cases}
\text{(V1) §11.3의 screening 통과},\\
\text{(V2) 계산 기한 안에 NLP 해를 얻고 모든 hard 제약을 허용 오차 안에서 만족},\\
\text{(V3) 모든 chance constraint의 slack이 0},\\
\text{(V4) §3.3의 reachability margin } m_{\mathrm{reach}}(T)\ge\kappa_m\sqrt{\lambda_{\max}(\cdot)}\ \text{만족},\\
\text{(V5) Hand 스케줄 조건 } t_{\mathrm{ps}}+T_{\mathrm{ps}}\le t_c-\tau_{\mathrm{rdy}}\ \text{만족}.
\end{cases}
$$

$$
\boxed{T^\star=\arg\min_{T\in\mathcal T_{\mathrm{valid}}}J^\star(T)},\qquad t_c^\star=t_0+T^\star .
$$

NLP가 nonconvex이므로 $J^\star$ 는 local solver가 반환한 best feasible cost이며, global optimum을 보장하지 않는다.

선택된 nominal catch point와 hand pose는 다음과 같다.

$$
\boxed{p_{\mathrm{catch}}^\star=\hat p_b(t_c^\star)},\qquad
\boxed{p_h(q_N^\star)=p_{\mathrm{catch}}^\star-R(q_N^\star)\,r_N^{H\star}},\qquad e_3^\top r_N^{H\star}=s_{\mathrm{ent}} .
$$

불확실성 하에서 실제 통과 위치는 평균 $\hat\rho_N$, 공분산 $\Sigma_\rho$ 의 분포를 갖는다.

### 11.2 계산 예산

MPC cycle 주기를 $\Delta_{\mathrm{mpc}}$ 라 하자. 30 Hz 메시지에 event-trigger하는 동기식 실행(§12.7의 P1)에서는 $\Delta_{\mathrm{mpc}}=\Delta_m$ 이다. NLP에 쓸 수 있는 시간은

$$
t_{\mathrm{sol,max}}=\Delta_{\mathrm{mpc}}-t_{\mathrm{pred}}-t_{\mathrm{screen}}-t_{\mathrm{pub}}-t_{\mathrm{margin}} .
$$

후보 NLP는 서로 독립이므로 $P$ 개 worker로 병렬 실행한다. 후보당 solve 시간의 95th percentile을 $t_{\mathrm{solve},95}$ (실측 profiling)라 하면, cycle당 NLP 개수는 다음으로 제한한다.

$$
L_{\mathrm{NLP}}\ \le\ P\left\lfloor\frac{t_{\mathrm{sol,max}}}{t_{\mathrm{solve},95}}\right\rfloor .
$$

기한을 넘긴 solve는 (V2)에 따라 invalid로 처리한다. 이 때 후보 순서는 §11.3의 순위를 따른다. $\Delta_{\mathrm{mpc}}$, $P$, $t_{\mathrm{solve},95}$ 의 수치는 구현 profiling으로 정하며 본 문서는 수치를 가정하지 않는다. 실시간 구현에서는 worker와 solver workspace를 미리 할당한다. 이 한계를 만족할 수 없으면 §12.7의 pipeline 실행(P2)을 쓰고, 그 지연을 $\tau_{\mathrm{react}}$ 에 반영한다.

### 11.3 필요조건 screening과 순위

아래 조건은 모두 **필요조건**이다. 통과가 feasibility를 보장하지 않으며, 탈락은 해당 후보의 infeasibility를 의미한다.

1. **(S1) 시간 창.** $T\in[T_{\min},T_{\max}]$.
2. **(S2) Hand 스케줄.** Preshape를 아직 시작하지 않았다면 $t_{\mathrm{ps}}\ge t_0$ 이므로 $T\ge T_{\mathrm{ps}}+\tau_{\mathrm{rdy}}$ 이다. 이미 시작했다면 고정된 $t_{\mathrm{ps}}$ 로 (V5)를 검사한다.
3. **(S3) Closing speed 창.** §8.4의 창이 비어 있지 않다. 이때 $m_{\mathrm{red}}$ 는 S4의 IK 해에서 평가한다(근사 screening).
4. **(S4) 운동학적 도달 가능성.** 목표 $p_h=\hat p_b(t_c)-R\,r_{\mathrm{ref}}^H$ 에 대해, 허용 orientation 집합에서 IK 해 $q^c$ 를 몇 개 구한다. 각 IK branch에서 관절별로 다음을 검사하고, 하나라도 통과하는 branch가 있어야 한다.

   $$
   |q_j^c-q_{0,j}|\le v_{\max,j}T,\qquad |q_j^c-q_{0,j}-v_{0,j}T|\le\tfrac12a_{\max,j}T^2 .
   $$

   첫 식은 $|v|\le v_{\max}$ 에서, 둘째 식은 $|a|\le a_{\max}$ 에서 각각 나오는 도달 집합의 필요조건이다.

5. **순위.** 통과한 후보를 $J_{\mathrm{time}}+J_{\mathrm{switch}}$ 와 IK 기반 cost proxy(예: $\|q^c-q_{\mathrm{nom}}\|$, $\psi_m(q^c)$)로 정렬한다. 상위 $L_{\mathrm{NLP}}$ 개만 NLP를 푼다.

### 11.4 Warm start: 정확한 shift와 신규 후보의 time-scaling

**기존 후보 (v3).** 절대 시각 격자에서는 한 cycle이 지나면 각 후보의 앞쪽 node $m$ 개가 이미 지난 시각이 된다. Warm start는 이 node들을 버리는 **정확한 shift**이다. 새 첫 구간 $h_0$ 은 직전 해를 $t_0$ 에서 평가해 채운다.

$$
\tilde q_0=q^\star_{\mathrm{prev}}(t_0),\qquad \tilde q_k=q_{\mathrm{prev},\,k+m}\ (k\ge1),
$$

$\tilde v_k,\tilde a_k$ 도 같은 방식이다. 이는 RTI 원논문의 shrinking horizon 처리, 즉 완료된 stage를 버리는 방식과 일치한다(Diehl et al., 2005, SIAM).

**신규 후보.** 격자 경계에서 새로 들어오는 후보만, 가장 가까운 기존 후보의 해를 affine time-scaling으로 옮긴다. 이전 cycle의 선택 해를 절대 시각의 함수 $q_{\mathrm{prev}}(t),v_{\mathrm{prev}}(t),a_{\mathrm{prev}}(t)$ ($t\in[t_{0,\mathrm{prev}},t_{c,\mathrm{prev}}]$)로 저장해 두고, 새 후보 $t_c$ 의 격자 $t_k$ 에 대해 다음을 쓴다.

$$
\sigma(t)=t_0+\alpha\,(t-t_0),\qquad \alpha=\frac{t_{c,\mathrm{prev}}-t_0}{t_c-t_0},
$$

$$
\tilde q_k=q_{\mathrm{prev}}(\sigma(t_k)),\qquad \tilde v_k=\alpha\,v_{\mathrm{prev}}(\sigma(t_k)),\qquad \tilde a_k=\alpha^2a_{\mathrm{prev}}(\sigma(t_k)) .
$$

$\tilde q_0,\tilde v_0$ 는 §12.7의 초기 상태 규칙에 따라 교체한다. $\alpha=1$ 이면 단순 시간 shift와 같다. 초기 추정치가 dynamics를 정확히 만족할 필요는 없다.

### 11.5 Continuous catch time (단일 NLP 형태)

$N$ 을 고정하고 $T$ 를 결정 변수로 두어 $h_T=T/N$ 을 쓰는 단일 NLP도 가능하다. 이 형태는 실시간 포획 문헌에 선례가 있다(Bäuml et al., 2010; Abeyruwan et al., 2023).

$$
\min_{T,\mathcal Z}J,\qquad T_{\min}\le T\le T_{\max},\qquad
q_{k+1}=q_k+h_Tv_k+\tfrac12h_T^2a_k,\quad v_{k+1}=v_k+h_Ta_k,\quad \hat z_{b,k}=\hat z_b(t_0+kT/N).
$$

Cost 적분, jerk, phase schedule, 공분산에도 $h_T$ 와 시간 의존성을 반영한다. Predictor의 시간 미분이 필요하다. 이 방식은 cycle당 NLP를 하나로 줄이지만 local minimum이 하나의 basin에 묶인다. v3의 기본 구현은 §11.1의 절대 시각 격자 후보와 후보별 국소 연속 포획 시각의 결합이다.

## 12. 실행 구조와 receding-horizon execution

### 12.1 Planner → CLIK → `servoj`

Planner는 §4.4의 연속 reference $q^\star(t),v^\star(t)$ 와 hand frame reference를 출력한다.

$$
X_h^\star(t)=\left(p_h(q^\star(t)),\,R(q^\star(t))\right),\qquad
V_h^\star(t)=J^{\mathrm{LWA}}(q^\star(t))\,v^\star(t).
$$

기존 CLIK가 이를 joint position 지령으로 바꾼다. 정확한 형태는 workspace 구현을 따른다. 인터페이스를 명확히 하기 위한 대표형은 다음과 같다.

$$
v_{\mathrm{cmd}}=J^{\#}(q)\left(V_h^\star+K_X\,e_X\right),\qquad
e_X=\operatorname{Log}_6\!\left(X_h(q)^{-1}X_h^\star\right)\ (\text{frame 일관성 유지}),\qquad
q_{\mathrm{cmd}}^{+}=q_{\mathrm{cmd}}+h_c\,v_{\mathrm{cmd}} .
$$

$q_{\mathrm{cmd}}^{+}$ 가 `servoj` 로 전달된다. Planner의 $h$ 와 제어 주기 $h_c$ 는 독립적으로 설계한다.

### 12.2 Torque와 추종 감시

토크는 지령하지 않으므로 v1의 $\tau_{\mathrm{cmd}}=Ma^\star+b+K_pe+K_d\dot e$ 경로는 존재하지 않는다. 대신 다음을 감시한다.

- 추종 오차 $e_q(t)=q^\star(t)-q(t)$ 와 hand frame 오차 $e_X(t)$.
- 가능하면 실제 관절 전류 또는 토크 추정치와 nominal $\tau^\star(t)$ 의 차이.

$\|e_X\|$ 가 lateral capture margin에 비해 커지면, 즉 $\|E_\perp^\top e_{X,p}\|>\gamma\min_i(\tilde b_i-\tilde a_i^\top\hat\rho)/\|\tilde a_i\|$ 이면 catch를 취소한다. 여기서 $e_{X,p}$ 는 위치 오차를 hand frame에서 표현한 것이고, $\gamma\in(0,1)$ 은 설계 상수이다. Reserve $\Delta\tau$ 는 이 감시 결과로 보정한다.

### 12.3 Hand 지령

Preshape 지령을 $t_{\mathrm{ps}}$ 에 보낸다. Closure는 §6.2의 mode에 따라 처리한다.

- **M1**: 접촉 검출 시 즉시 closure를 지령한다.
- **M2**: $t_{\mathrm{cmd}}=t_c^\star-\tau_{\mathrm{cl}}+\delta_{\mathrm{mid}}$ 에 지령한다. Arm 고정 전에는 $t_c^\star$ 가 cycle마다 갱신되므로 $t_{\mathrm{cmd}}$ 도 매 cycle 다시 계산한다. Arm 고정 후에는 §12.7의 통과 시각 재예측으로 갱신한다. 지령 후에는 고정한다.

### 12.4 Phase schedule

| Phase | 주요 목적 | 주요 활성 조건 | 전환 조건 |
|---|---|---|---|
| Intercept | Feasible capture 위치로 이동 | Robot bound, collision, terminal 조건 전부 | Corridor 근접 및 approach feasibility 확인 |
| Approach | 정해진 방향·속도로 진입 | Corridor, closing envelope, relative cost | Terminal까지 남은 시간이 preshape·closure 스케줄 기준 이하 |
| Capture transition | 통과 직전 상태 실현 | Lateral·timing chance, velocity set, impact bound | 실제 통과 또는 접촉 검출 |
| Post-capture | 충격 흡수, closure, 유지 | Contact dynamics, closure 성공 판정 | Stable hold 판정 |

Terminal 조건은 Intercept phase의 후보 solve에서도 항상 적용한다. Phase에 따라 바뀌는 것은 $\mathcal A_T$ 와 중간 구간 weight뿐이다.

### 12.5 Fallback

다음 경우 catch를 취소하고 미리 검증한 braking/hold/retreat policy로 전환한다.

- Valid 후보가 없을 때.
- Solver가 timeout일 때.
- 예측 갱신으로 계획이 성립하지 않을 때.
- §12.2의 감시 조건을 위반할 때.
- §3.7의 예측 일관성 gating이 연속으로 실패할 때.

직전 해를 재사용할 때는 최신 상태와 예측에 대해 남은 궤적의 feasibility를 다시 확인한다.

### 12.6 Post-capture 전환과 reference spreading

충격 시각의 불일치는 tracking 오차 peak를 만든다. 이를 줄이기 위해 reference를 양쪽으로 연장한다.

- **Ante-impact reference.** 마지막 구간의 등가속 다항식을 $t>t_c$ 로 연장한다. 즉 $q_{\mathrm{ante}}^{\mathrm{ext}}(t)=q_N+(t-t_c)v_N$ 이며, 가속도를 0으로 두는 연장도 가능하다.
- **Post-impact reference.** $q_{\mathrm{post}}^{\mathrm{ext}}(t)$ 를 $t<t_c$ 까지 연장한다.

전환 규칙은

$$
q_{\mathrm{ref}}(t)=\begin{cases}q_{\mathrm{ante}}^{\mathrm{ext}}(t),&\text{접촉 미검출 (M2에서는 } t<t_{\mathrm{cmd}})\\ q_{\mathrm{post}}^{\mathrm{ext}}(t),&\text{그 이후}\end{cases}
$$

이다. 이 아이디어는 reference spreading 계열(§16)에서 다룬 것이다. Position 제어 arm에서 post-impact reference를 어떻게 설계할지는 별도 검증이 필요하다. 예를 들어 $c_n$ 과 $m_{\mathrm{red}}$ 로 예측한 속도 jump를 반영하는 방법이 있다.

필요하면 hybrid MPC로 확장할 수 있다. 이 경우 다음 식에 contact kinematics, friction cone, complementarity 또는 고정 contact mode, impact reset map, closure dynamics를 함께 추가해야 한다.

$$
M(q)a+b(q,v)=\tau+J_{\mathrm{con}}^\top\lambda,\qquad M_b\dot v_b=h_b+G_b\lambda .
$$

### 12.7 30 Hz 예측 갱신 루프 (v3)

**Trigger.** MPC cycle은 vision 메시지 도착에 event-trigger한다. 이보다 자주 풀어도 공에 대한 새 정보는 없고 로봇 상태만 갱신되며, position 제어로 추종하는 arm에서는 그 정보 가치가 작다. 메시지마다 §3.7의 gating을 먼저 수행하고, 통과한 메시지만 NLP parameter로 쓴다.

**실행 방식.** 둘 중 하나를 선택한다.

- **(P1) 동기식.** $t_{\mathrm{solve}}<\Delta_m$ 일 때 쓴다. 메시지마다 모든 활성 후보를 풀고 §11.1로 선택한다. 계획 적용 시각은 다음과 같다.

  $$
  t_0=s_j+\tau_{\mathrm{vis}}+t_{\mathrm{sol,max}} .
  $$

- **(P2) Pipeline.** Solve 시간이 메시지 주기보다 길 때 쓴다. 메시지마다 놀고 있는 worker를 배정하고, 완료된 해 중 **가장 최신 stamp**의 해를 적용한다. Bäuml et al.(2010)은 예측마다 놀고 있는 core에 배정해 예측을 건너뛰지 않았으며, 최악 solve 시간이 예측 주기보다 길었다. P2에서는 적용 지연이 $\tau_{\mathrm{vis}}+t_{\mathrm{solve}}$ 이므로 그 값을 $\tau_{\mathrm{lat}}$ (§3.6)에 그대로 더한다.

**초기 상태.** 초기 상태는 측정값이 아니라 직전 계획의 reference로 둔다.

$$
q_0=q^\star_{\mathrm{prev}}(t_0),\qquad v_0=v^\star_{\mathrm{prev}}(t_0),\qquad a_{\mathrm{prev}}=a^\star_{\mathrm{prev}}(t_0^-),
$$

단 추종 오차가 임계값을 넘으면 측정값 $\hat q(t_0),\hat v(t_0)$ 로 교체한다. 메시지마다 reference가 측정 잡음만큼 튀는 것을 막고, §4.3의 discrete jerk 제약과 일관된다. Bäuml et al.(2010)은 예측의 jump가 지령 속도의 꺾임으로 나타났다고 보고했으며, 이 규칙과 jerk 제약의 경험적 근거가 된다. 다만 이 규칙은 RTI 문헌의 표준, 즉 측정 상태를 initial value embedding으로 쓰는 방식과 다른 설계 선택이다 (c). Position 제어 arm의 추종이 좋다는 전제에서만 정당화된다.

**Real-time iteration.** 예측 parameter는 매 cycle 조금씩만 바뀌므로, NLP를 매번 수렴까지 풀 필요가 없다. RTI는 sampling마다 Newton형 반복을 한 번만 수행하고, 연속된 cycle을 통해 해를 수렴시킨다(Diehl et al., 2005, SIAM). 각 반복은 두 단계로 나뉜다(Gros et al., 2020).

- **Preparation**: 메시지가 오기 전에, shift된 이전 해와 예측 parameter 주변에서 선형화와 condensing을 해 둔다.
- **Feedback**: 메시지가 오면 QP만 푼다.

이렇게 하면 갱신부터 reference 반영까지의 지연이 QP 한 번으로 줄어든다. acados는 `SQP_RTI` 와 `rti_phase` 옵션(1: preparation, 2: feedback)으로 이 분리를 지원한다(소스 기준). 고정 종단 시각의 shrinking horizon에서 RTI의 contraction 정리가 성립하며(Diehl et al., 2005, SIAM), receding horizon의 nominal stability는 Diehl et al.(2005, IEE Proc.)이 다룬다. 이 정리는 **상태 교란**에 대한 것이며, 공 예측 parameter의 jump에 대한 contraction 영역은 별도로 검증해야 한다.

초기 몇 cycle은 예측 jump가 크다(§15의 예시). 이 구간은 남은 시간이 길어 수정 여유도 크다. 따라서 계산 예산 안에서 초기에는 반복을 여러 번 하고, 이후 한 번으로 줄이는 방식이 합리적이다 (c).

**결정 변수의 고정 순서.** 다음 순서로 하나씩 고정한다.

1. Preshape를 시작하면 $t_{\mathrm{ps}}$ 를 고정한다.
2. $t\ge t_c^\star-\tau_{\mathrm{react}}$ 이면 outer loop를 멈추고 $t_c^\star$ 와 arm 궤적을 고정한다. 이후 arm은 open-loop로 실행한다(§14).
3. 그 뒤로는 closure 지령 시각 갱신과 gating만 계속한다.

고정되기 전에는 후보가 절대 시각 격자 위에서 유지되므로, 메시지마다 screening(§11.3)을 다시 하고 기존 후보의 warm-start된 NLP를 이어서 푼다.

**Closure 지령 시각 갱신 (M2).** Arm이 고정된 뒤에는 고정된 arm 궤적과 최신 공 평균으로 통과 시각을 다시 예측한다.

$$
\hat t_x(s_j):\ \ e_3^\top R(q^\star(t))^\top\big(\hat p_b(t\mid s_j)-p_h(q^\star(t))\big)=s_{\mathrm{ent}},\qquad
t_{\mathrm{cmd}}=\hat t_x(s_j)-\tau_{\mathrm{cl}}+\delta_{\mathrm{mid}} .
$$

$\hat t_x$ 는 $t_c^\star$ 근방에서 1차원 root-finding(예: Newton, 초기값 $t_c^\star$)으로 구한다. 이 갱신은 §8.4의 $s^{\star}_{\mathrm{hand}}$ 까지 반복하고, 지령을 보낸 뒤에는 고정한다.

## 13. Hard constraint / soft constraint / cost의 역할

| 항목 | 기본 배치 | 해석 |
|---|---|---|
| Initial condition, state transition, EOM | Hard equality | Prediction 일관성 |
| Joint·velocity·acceleration bound, torque bound(reserve 포함) | Hard inequality | Physical feasibility |
| Self/environment collision, 비-capture link의 ball clearance | Hard inequality | 허용된 접촉 이외 충돌 회피 |
| Gap 비음수 조건 (approach 구간) | Hard inequality | 무접촉 가정의 일관성 |
| Entrance-plane equality | Hard equality | Terminal event 정의 |
| Approach corridor, closing envelope | Soft path constraint | 접근 유연성, 위반 정도 표시 |
| Lateral capture chance, sensor coverage chance(M1), timing chance(M2) | Hard terminal | Valid 후보의 확률 조건 |
| Terminal velocity set (tightened) | Hard terminal | Capture transition feasibility |
| Impact energy·impulse | Cost + hard nominal bound | 충격 감소와 상한 |
| Hand schedule, closing speed 창, reachability margin | Outer-loop validity | 후보 선택 조건 |
| Relative state, posture, manipulability | Cost | Feasible 해 사이 선호 |
| Time, uncertainty, switching | Candidate cost | Catch time trade-off |

Hard constraint의 수치 허용 오차는 solver tolerance와 구현 margin으로 명시한다. 큰 slack penalty만으로 안전성이나 확률 보장이 확보된다고 해석하지 않는다.

## 14. 모델 확장과 한계

**Hand를 NLP에 포함하는 확장.** Closed-chain hand를 전체 link 좌표로 넣으면 fully actuated equation을 그대로 쓸 수 없다.

$$
M(q)a+b(q,v)=B(q)\tau_{\mathrm{act}}+J_\ell^\top\lambda_\ell,\qquad \phi(q)=0,\qquad J_\ell v=0,\qquad J_\ell a+\dot J_\ell v=0 .
$$

Independent coordinate로 reduction할 수 있다면 대응하는 reduced dynamics와 actuator mapping을 쓴다. 이 확장은 NLP 차원을 크게 늘린다. 따라서 preshape 타이밍만으로 readiness를 만족할 수 없다는 증거가 있을 때만 고려한다.

**이론적 한계.**

- **Recursive feasibility.** Moving terminal set, hybrid switch, local NLP, 예측 갱신 때문에 recursive feasibility나 stability를 자동으로 주장할 수 없다. Anticipated covariance는 공분산이 줄어드는 동안 feasibility에 유리하지만, 평균 이동에 대해서는 (V4)의 margin이 성립할 때만 의미가 있다.
- **Commit 이후의 open-loop 구간.** $t_c^\star-\tau_{\mathrm{react}}$ 이후 arm은 재계획 없이 실행된다. 이 구간의 강건성은 재계획이 아니라 계획 자체의 여유(chance tightening, reachability margin)에 의존한다. Schill & Buss(2018)는 재계획 없이 하나의 오프라인 가속도 프로파일이 초기 상태·가속도·충격의 유한한 불확실성에 대해 강건함을 보였다. 본 설계의 commit 이후 구간을 분석하는 데 그 접근을 참고할 수 있다.
- **Invariance.** Terminal capture set은 control invariant set이 아니다. 유지까지 보장하려면 post-capture controller의 viability/invariant set을 구성하거나 접촉 후 horizon을 포함해야 한다.
- **선형화.** Crossing-plane 분포는 1차 선형화와 Gaussian 가정에 의존한다. §8.2의 유효 조건과 실측 calibration으로 확인한다.
- **Effective mass.** 충격 시간 척도, 감속기 탄성, position loop의 영향은 식별 전까지 §7.2의 보수적 상한으로 다룬다.

## 15. 구현 및 수학적 검증 항목

1. **Frame과 부호.** $R$ 은 $H\to W$, $r=p_b-p_h$, $c=-\dot s$ 로 통일한다. 정지한 손으로 공이 직선 접근할 때 $c>0$ 인지 확인한다.
2. **Transport term과 Jacobian 규약.** 회전하는 손에 대해 $\nu^H$ 를 finite difference와 비교한다. `LOCAL_WORLD_ALIGNED` 와 `LOCAL` 두 경로가 같은 값을 내는지 확인한다. (v2 작성 시 임의 회전·속도에서 중심차분 대비 오차 약 $10^{-10}$ 수준을 확인했다.)
3. **Dynamics.** RNEA residual을 확인한다. Acceleration rollout과 실제 `servoj` 추종 결과의 차이를 측정해 $\Delta\tau$ 와 $\tau_{\mathrm{track}}$ 을 정한다.
4. **Entrance plane 일관성.** §6.1의 무접촉 일관성 조건을 hand mesh로 오프라인 검증한다. Erosion을 중복 적용하지 않는다. Contact normal과 $J_c$ 는 같은 geometry에서 계산한다.
5. **Crossing-plane 분포.** 선형 상대운동 Monte Carlo로 $\Sigma_\rho=E_\perp^\top\Pi\Sigma\Pi^\top E_\perp$ 를 검증한다. v2 작성 시 사용한 예시는 $\hat\nu=[0.3,-0.1,-2.0]$ m/s, $\sigma=[4,4,20]$ mm, 2×10⁵ 표본이다. 이 예시에서 MC 공분산과 해석식이 일치했고, 고정 시각 식 대비 x 방향 분산이 약 56% 컸다. 이 수치는 해당 예시 설정의 결과일 뿐 일반적인 크기가 아니다. 실제 검증은 실측 $\hat\nu,\Sigma$ 로 반복한다.
6. **Timing.** $\tau_{\mathrm{cl}}$, $\sigma_\tau$, $[\delta_{\mathrm{lo}},\delta_{\mathrm{hi}}]$ 를 식별하고 §8.4의 성공 확률식을 실험 성공률과 비교한다.
7. **Anticipated covariance.** $\tau_{\mathrm{react}}\ge T$ 에서 open-loop 공분산과 일치하는지, $\Sigma_{\mathrm{drift}}\succeq0$ 인지, Joseph form이 대칭·양반정치를 유지하는지 확인한다.
8. **Impact.** $m_{\mathrm{red}}$ 의 극한과 단위, 모델 계층별 에너지 순서(§7.2)를 수치로 확인한다. Energy proxy를 peak force와 혼동하지 않는다.
9. **Time choice.** 같은 연속 궤적을 다른 mesh로 적분했을 때 cost가 유사한지 확인한다. Absolute catch-time switching penalty와 warm-start time-scaling을 검증한다.
10. **계산 예산.** $t_{\mathrm{solve},95}$ 를 profiling하고 $L_{\mathrm{NLP}}$ 한계 안에서 deadline miss율을 측정한다.
11. **Execution.** Fallback, M1/M2 전환, reference spreading 전환을 시뮬레이션(MuJoCo)에서 먼저 검증한다.
12. **Covariance replica (v3).** 시각 $s_j$ 마다 replica의 $\Sigma(t\mid s_j)$ 와 vision이 publish한 공분산이 일치하는지 확인한다(§3.5).
13. **예측 일관성 (v3).** 투척 로그로 NEES/NIS가 $\chi^2$ 범위에 드는지 확인한 뒤 gating 임계값을 정한다(§3.7). v3 작성 시 1차원 탄도 KF로 jump 공분산 식을 Monte Carlo 검증했다. 가정은 30 Hz 위치 측정, 측정 잡음 5 mm, 초기 불확실성 50 mm와 0.5 m/s, 첫 갱신 후 0.5 s(15회 갱신) 포획, 2×10⁴ 표본이다. Jump 분산의 MC 값과 $\Sigma(t_c\mid s_j)-\Sigma(t_c\mid s_{j+1})$ 가 모든 단계에서 수 % 이내로 일치했다. 이 설정에서 $\sigma(t_c\mid s_j)$ 는 남은 갱신 15회에서 약 250 mm, 11회에서 21 mm, 7회에서 7.3 mm, 3회에서 3.6 mm였다. 이 크기는 가정한 잡음 수준의 결과일 뿐이며, 초기에 급격히 줄고 이후 완만해진다는 형태만 일반적이다.
14. **Commit 시각 (v3).** $t_{\mathrm{occ}}$ 를 카메라 배치와 포획 자세로 추정하고, §3.6의 수정 능력 조건과 함께 $\tau_{\mathrm{react}}$ 를 정한다. 시뮬레이션에서 commit 이후 들어온 예측 갱신의 크기 분포와 최종 lateral 오차를 비교한다.
15. **실시간 루프 (v3).** P1이면 deadline miss율, P2이면 적용 지연 분포를 측정한다. RTI의 cycle당 반복 수와 예측 jump 크기에 따른 해의 수렴(KKT residual)을 기록한다.

주요 미해결 사항은 다음과 같다.

- Capture window·closure latency의 식별.
- 다중·compliant contact에서 impact bound의 타당성.
- Position 제어 arm의 post-impact reference 설계.
- 예측 갱신 하의 recursive feasibility.

## 16. 참고문헌과 본 설계에서의 활용 범위

### 16.1 검증된 문헌

아래 서지 정보와 내용 대응은 publisher, Crossref, 기관 repository, arXiv에서 확인했다(2026-10-02). 통합된 수학적 구성은 본 설계의 제안이며, 각 논문이 이 arm–hand MPC 전체를 검증했다는 의미는 아니다.

| 문헌 | 확인된 내용 | 본 설계와의 대응 및 한계 |
|---|---|---|
| Hartley et al., 2012, *Control Engineering Practice* | Range 기반 phase별 MPC, finite-time 완료를 위한 variable prediction horizon, collision-avoidance 제약을 switched convex 제약으로 처리 | Phase 구성과 catch time 선택에 참고. LOS cone·corridor라는 표현은 초록에서 확인되지 않았으므로 corridor의 근거로 쓰지 않는다 |
| Gavilan et al., 2012, *Control Engineering Practice* | Gaussian disturbance, chance constraint의 deterministic algebraic 변환, online disturbance estimation, line-of-sight 제약 | Affine face의 probabilistic tightening과 approach corridor 개념의 근거. Grasp 성공 확률은 별도 검증 필요 |
| Ravikumar, Padhi, Philip, 2020, *IFAC-PapersOnLine* (ACODS 2020) | SQP로 푸는 receding-horizon NMPC, thrust 제한, line-of-sight 내 접근, debris 회피, soft docking을 위한 terminal velocity 제한 | Approach·terminal velocity 조건 설계에 참고. 논문 수식을 재현하지 않음 |
| Tassi et al., 2026, *The International Journal of Robotics Research* | 접촉 전 velocity matching과 impact force 최소화, 사람 시연에서 학습한 접촉 후 energy dissipation, hierarchical QP 2차 층의 reflected mass 최소화. Nonprehensile catching | Relative velocity와 configuration-dependent inertia의 근거. Multi-finger grasp 성공을 보장하지 않음 |

1. E. N. Hartley, P. A. Trodden, A. G. Richards, J. M. Maciejowski, "Model predictive control system design and implementation for spacecraft rendezvous," *Control Engineering Practice*, 20(7), 695–713, 2012. DOI: [10.1016/j.conengprac.2012.03.009](https://doi.org/10.1016/j.conengprac.2012.03.009). [White Rose eprint 90483](https://eprints.whiterose.ac.uk/90483/).
2. F. Gavilan, R. Vazquez, E. F. Camacho, "Chance-constrained model predictive control for spacecraft rendezvous with disturbance estimation," *Control Engineering Practice*, 20(2), 111–122, 2012. DOI: [10.1016/j.conengprac.2011.09.006](https://doi.org/10.1016/j.conengprac.2011.09.006). (Crossref 표기는 악센트 없음.)
3. L. Ravikumar, R. Padhi, N. K. Philip, "Trajectory optimization for Rendezvous and Docking using Nonlinear Model Predictive Control," *IFAC-PapersOnLine*, 53(1), 518–523, 2020 (ACODS 2020, IIT Madras). DOI: [10.1016/j.ifacol.2020.06.087](https://doi.org/10.1016/j.ifacol.2020.06.087). 저자 표기는 기관 페이지에서 "Ravi Kumar L", "Radhakant Padhi"로도 나온다.
4. F. Tassi, J. Zhao, G. J. G. Lahr, L. Gava, M. Monforte, A. Glover, C. Bartolozzi, A. Ajoudani, "IMA-catcher: An IMpact-aware nonprehensile catching framework based on combined optimization and learning," *The International Journal of Robotics Research*, 45(1), 100–127, 2026 (online first 2025-06-20). DOI: [10.1177/02783649251345851](https://doi.org/10.1177/02783649251345851). arXiv: [2506.20801](https://arxiv.org/abs/2506.20801).

### 16.2 v3에서 추가 검증한 문헌

아래 서지는 Crossref DOI 기록, publisher 페이지, 저자·기관 PDF, arXiv, acados 소스로 확인했다(2026-10-02). 내용 대응은 열람한 본문 또는 초록 범위에서만 기술한다.

| 문헌 | 확인된 내용 | 본 설계와의 대응 및 한계 |
|---|---|---|
| Diehl, Bock, Schlöder, 2005, *SIAM J. Control Optim.* | Sampling마다 Newton형 반복 1회, 새 상태가 오기 전 대부분의 계산을 끝내는 구조, initial value embedding, 고정 종단 시각 shrinking horizon에서의 contraction 정리 | §12.7의 RTI, §11.4의 완료 stage 제거. 정리는 상태 교란 기준이며 예측 parameter jump는 별도 검증 |
| Diehl, Findeisen, Allgöwer, Bock, Schlöder, 2005, *IEE Proc. Control Theory Appl.* | RTI 결합 시스템의 nominal stability (receding horizon) | §12.7. IET 페이지는 열람하지 못했고 IMA preprint 초록으로 확인 |
| Diehl et al., 2002, *J. Process Control* | Initial value embedding, Hessian·gradient·QP 사전 계산, 반복마다 feedback | RTI의 초기 형태 |
| Gros, Zanon, Quirynen, Bemporad, Diehl, 2020, *Int. J. Control* | Preparation/feedback 분리를 명시한 알고리즘, 이전 해의 shift, 온라인 입력을 갖는 NLP와 tangential predictor | §12.7의 두 단계 구조 |
| Verschueren et al., 2022, *Mathematical Programming Computation* (acados) | acados 프레임워크. `SQP_RTI` 와 `rti_phase` 는 논문이 아니라 소스 코드에서 확인 | §12.7의 구현 경로 |
| Nagy & Braatz, 2003, *AIChE Journal* | 고정 종료 시각의 batch 공정에서 shrinking horizon NMPC 정식화 | §11.1의 shrinking horizon 용어와 정식화 |
| Bäuml, Wimböck, Hirzinger, 2010, IEEE/RSJ IROS | Arm–hand 포획. 새 예측(20 ms 주기)마다 SQP를 다시 풀고, 예측마다 놀고 있는 core에 배정. 포획 시각을 결정 변수로 둠. 예측 jump가 지령 속도의 꺾임으로 나타남 | §11.1 국소 연속 포획 시각, §11.5, §12.7의 P2와 초기 상태 규칙 |
| Birbach, Frese, Bäuml, 2011, IEEE ICRA | Stereo 기반 실시간 지각, UKF 예측을 planner로 전달, 포획 직전 구간의 측정이 최종 포획 위치를 결정 | §3.5의 지각 한계. 수치는 시스템 고유값 |
| Kim, Shukla, Billard, 2014, *IEEE T-RO* | 예측 thread와 포획 configuration·시각 재최적화 thread 병렬. 손 근처 occlusion 때문에 접촉 직전 일정 시간 이후 갱신 중단 | §3.5의 $t_{\mathrm{occ}}$ |
| Salehian, Khoramshahi, Billard, 2016, *IEEE T-RO* | LPV 동역학 기반 soft catching DS controller, GMM 학습, Lyapunov 수렴 (초록 기준) | §9.3의 soft catching. 포획점의 online 갱신 여부는 본문 미열람으로 미확인 |
| Schill & Buss, 2018, *IEEE T-RO* | 재계획 없이 오프라인 가속도 프로파일 하나가 유한한 불확실성에 강건함을 증명. Visual feedback 불필요 | §14의 commit 이후 open-loop 구간 분석 |
| Abeyruwan et al., 2023, L4DC (PMLR 211) | Whole-body MPC 포획. 공 예측 parameter를 비동기로 갱신하고 SQP를 연속으로 다시 풂. 포획 시각을 결정 변수로 둠 | §11.1, §12.7 |
| Kailath, 1968, *IEEE TAC* | Innovations approach. Innovation의 백색성과 직교성 | §3.7 유도의 기반 |

5. M. Diehl, H. G. Bock, J. P. Schlöder, "A real-time iteration scheme for nonlinear optimization in optimal feedback control," *SIAM J. Control Optim.*, 43(5), 1714–1736, 2005. DOI: [10.1137/S0363012902400713](https://doi.org/10.1137/S0363012902400713).
6. M. Diehl, R. Findeisen, F. Allgöwer, H. G. Bock, J. P. Schlöder, "Nominal stability of real-time iteration scheme for nonlinear model predictive control," *IEE Proc. Control Theory Appl.*, 152(3), 296–308, 2005. DOI: [10.1049/ip-cta:20040008](https://doi.org/10.1049/ip-cta:20040008).
7. M. Diehl, H. G. Bock, J. P. Schlöder, R. Findeisen, Z. Nagy, F. Allgöwer, "Real-time optimization and nonlinear model predictive control of processes governed by differential-algebraic equations," *J. Process Control*, 12(4), 577–585, 2002. DOI: [10.1016/S0959-1524(01)00023-3](https://doi.org/10.1016/S0959-1524(01)00023-3).
8. S. Gros, M. Zanon, R. Quirynen, A. Bemporad, M. Diehl, "From linear to nonlinear MPC: bridging the gap via the real-time iteration," *Int. J. Control*, 93(1), 62–80, 2020 (online 2016). DOI: [10.1080/00207179.2016.1222553](https://doi.org/10.1080/00207179.2016.1222553).
9. R. Verschueren et al., "acados—a modular open-source framework for fast embedded optimal control," *Mathematical Programming Computation*, 14(1), 147–183, 2022 (online 2021). DOI: [10.1007/s12532-021-00208-8](https://doi.org/10.1007/s12532-021-00208-8). arXiv: [1910.13753](https://arxiv.org/abs/1910.13753).
10. Z. K. Nagy, R. D. Braatz, "Robust nonlinear model predictive control of batch processes," *AIChE Journal*, 49(7), 1776–1786, 2003. DOI: [10.1002/aic.690490715](https://doi.org/10.1002/aic.690490715).
11. B. Bäuml, T. Wimböck, G. Hirzinger, "Kinematically optimal catching a flying ball with a hand-arm-system," IEEE/RSJ IROS 2010, 2592–2599. DOI: [10.1109/IROS.2010.5651175](https://doi.org/10.1109/IROS.2010.5651175).
12. O. Birbach, U. Frese, B. Bäuml, "Realtime perception for catching a flying ball with a mobile humanoid," IEEE ICRA 2011, 5955–5962. DOI: [10.1109/ICRA.2011.5980138](https://doi.org/10.1109/ICRA.2011.5980138).
13. S. Kim, A. Shukla, A. Billard, "Catching objects in flight," *IEEE Trans. Robotics*, 30(5), 1049–1065, 2014. DOI: [10.1109/TRO.2014.2316022](https://doi.org/10.1109/TRO.2014.2316022).
14. S. S. M. Salehian, M. Khoramshahi, A. Billard, "A dynamical system approach for softly catching a flying object: Theory and experiment," *IEEE Trans. Robotics*, 32(2), 462–471, 2016. DOI: [10.1109/TRO.2016.2536749](https://doi.org/10.1109/TRO.2016.2536749).
15. M. M. Schill, M. Buss, "Robust ballistic catching: A hybrid system stabilization problem," *IEEE Trans. Robotics*, 34(6), 1502–1517, 2018. DOI: [10.1109/TRO.2018.2868857](https://doi.org/10.1109/TRO.2018.2868857).
16. S. Abeyruwan et al., "Agile catching with whole-body MPC and blackbox policy learning," L4DC 2023, PMLR 211, 851–863. arXiv: [2306.08205](https://arxiv.org/abs/2306.08205).
17. T. Kailath, "An innovations approach to least-squares estimation—Part I: Linear filtering in additive white noise," *IEEE Trans. Automatic Control*, 13(6), 646–655, 1968. DOI: [10.1109/TAC.1968.1099025](https://doi.org/10.1109/TAC.1968.1099025).

**추정 이론 교과서 (원문 미열람).** Kalman 공분산의 측정값 독립성과 NEES/NIS의 $\chi^2$ 일관성 검사는 다음 교과서의 표준 내용으로 알려져 있다. 이번 검증에서는 원문을 열람하지 못했고, 강의노트·기술보고서와 이를 인용한 논문으로만 확인했다. 출판 인용 전 해당 절을 직접 확인한다.

- B. D. O. Anderson, J. B. Moore, *Optimal Filtering*, Prentice-Hall, 1979.
- Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001. DOI: [10.1002/0471221279](https://doi.org/10.1002/0471221279).

### 16.3 관련 선행연구 (서지 세부 미검증)

아래 문헌은 Dynamic catching 문헌 조사(`55_Dynamic_Catching` 폴더) 수집 목록에서 가져왔다. 제목·권호·DOI를 다시 검증하지 않았다. 인용 전 수집 원문으로 서지를 확인한다.

| 계열 | 수집 목록의 식별 정보 | 본 설계와의 관련 |
|---|---|---|
| DLR Bäuml 계열 (시스템 개요) | Humanoids 2011 (Bäuml, Birbach, Wimböck, Frese, Dietrich, Hirzinger). 제목은 저자 업로드본으로만 확인 | 비행 중 예측이 크게 바뀌므로 카메라 주기 수준으로 재계산이 필요하다는 설계 근거 (§12.7) |
| Reference spreading | Saccon CDC 2014, Rijnen CDC 2015, van Steen ACC 2022 및 T-RO 2024 | 충격 시각 불일치 하의 reference 전환 (§12.6) |
| 충격 모델 | Jia IJRR 2013 | 다중 충격 모델로의 확장 (§7, §14) |
| 로봇 충돌 안전의 effective mass 분석 | **확인 필요** (출처 미특정) | 감속기 탄성과 rotor 분리 (§7.2) |

### 16.4 수치와 근거 등급

이 문서는 검증되지 않은 논문의 정량 성능 수치를 사용하지 않는다. §15의 수치는 v2·v3 작성 시 수행한 수치 검증의 예시 결과이다. §3.5와 §16.2에서 언급한 문헌의 시간 수치(예측 주기, 갱신 중단 시점)는 각 시스템 고유의 값이므로 설계값으로 옮기지 않았다.

- **표준 결과 (a)**: Frame 변환, transport theorem, constant-acceleration 적분, Gaussian affine chance constraint, Boole risk allocation, Riccati recursion의 측정값 독립성, effective mass와 구속·관성의 단조성, 조건부 기대값의 martingale 성질과 law of total covariance, bang-bang 최대 변위, innovation 사상을 통한 NIS 동치.
- **확립된 방법 (b)**: RTI와 preparation/feedback 분리, shrinking-horizon NMPC, 예측 갱신마다의 포획 재계획.
- **설계 제안 (c)**: Crossing-plane 분해, timing 기반 closing speed 하한, anticipated covariance와 drift margin, screening 조건 조합, jump 공분산 식(표준 결과로부터 유도), $\tau_{\mathrm{react}}$ 의 세 조건 결정, 절대 시각 격자와 국소 연속 포획 시각의 결합, reference 기반 초기 상태, arm/hand 정보 시각의 분리.
- **대상 로봇에서 식별·검증할 설계 parameter**: Corridor 형상, weight, capture window, latency, $t_{\mathrm{occ}}$, $a_{\mathrm{lat}}$, $E_{\max}$, risk budget, gating 임계값.

## 17. 구현 — mpc_docking 수치 코어

§0 – §16 은 설계이고 이 절은 구현이다. `rtc_controllers` 의 `MpcDockingSegmentCore` (`catching/mpc_docking_segment_core.hpp`) 와 그것이 부르는 함수들 (`catching/mpc_docking_relative_state.hpp`, `catching/ball_node_samples.hpp`) 이 실제로 계산하는 식을 적는다. 소절 번호는 원래 절과 대응한다 — 17.$n$ 이 §$n$ 의 구현이다. 설계와 다른 곳은 이 절의 식이 코드의 것이다. 구현하지 않은 항도 소절마다 적는다.

### 17.1 범위

- 구현한 것은 §10 의 inner 문제 하나다 — 포구 시각 $t_c$ 가 정해진 후보 하나에 대해 팔 궤적을 푼다. 궤적은 포구에서 끝나지 않고 그 뒤 정지할 때까지 이어진다. 스위치 `catch_time_variable` 을 켠 코어는 그 후보의 포구 시각도 호출자가 준 구간 안에서 정한다 (17.11). 기본은 꺼짐이고, 끈 코어에는 그 경로가 없다.
- 결정변수는 팔 관절뿐이다. 모델은 호출자가 주는, 손 관절을 잠근 팔 모델이고 `model.armature` 에 입력 armature 를 더해 쓴다 (§7.2 의 보수 쪽 모델).
- §11 의 바깥 루프 (후보 집합 · 순위 · 선택), §12 의 실행 구조, hand schedule (§6.2) 은 이 코어에 없다. $J_{\mathrm{time}}$ · $J_{\mathrm{switch}}$ 와 후보별 국소 연속 포획 시각 (§11.1) 은 포구 시각을 변수로 둔 코어에만 있다 — 그 코어는 두 항을 포구 시각의 함수로 받는다 (17.11). 폐쇄 시각은 손 시퀀서가 정하고 코어는 그 명목값 $\delta_0$ 만 받는다 (17.8).
- 코어는 ROS · 컨트롤러 · 로봇 이름을 모른다. 호출하는 것은 NLP 탐색 (17.11) 과 테스트이고, 계획기에 꽂는 일은 다른 feature 가 한다.

### 17.2 표기와 시간축

격자는 포구 시각에 닻을 내린다. 포구 전 구간이 $n_{pre}$ 개 (간격 $\Delta_a$), 정지 구간이 $n_{stop}$ 개 (간격 $\Delta_s$) 이고 $N=n_{pre}+n_{stop}$, 포구 노드는 $k_c=n_{pre}$ 다.

$$
t_k=\begin{cases}t_c-(k_c-k)\Delta_a,&k\le k_c\\ t_c+(k-k_c)\Delta_s,&k\gt k_c\end{cases}
$$

§4.1 의 비균일 첫 구간 $h_0$ 는 없다 — 노드 0 의 시각이 곧 $t_c-n_{pre}\Delta_a$ 다. 풀 수 있는 것은 $n_{pre}\ge1$ 뿐이다. 포구 시각을 변수로 둔 코어에서는 위 식의 $t_c$ 가 닻 $\hat t$ 이고, 포구 노드와 그 뒤의 노드만 $\delta t_c$ 만큼 옮겨진다 — 포구 노드 앞 구간 하나의 길이가 $\Delta_a+\delta t_c$ 다 (17.11).

입력은 구간 상수 jerk $u_k$ 이고 상태는 $x_k=(q_k,\dot q_k,\ddot q_k)$ 다.

$$
q_{k+1}=q_k+\Delta_k\dot q_k+\tfrac12\Delta_k^2\ddot q_k+\tfrac16\Delta_k^3u_k,\qquad
\dot q_{k+1}=\dot q_k+\Delta_k\ddot q_k+\tfrac12\Delta_k^2u_k,\qquad
\ddot q_{k+1}=\ddot q_k+\Delta_ku_k .
$$

$u_k$ 는 블록마다 같은 값을 쓴다 (move blocking — 블록은 포구 노드를 넘지 않고, 포구 뒤에 셋 이상). 풀이의 변수는 $z=\tilde{\mathbf u}/u_s$ ($u_s$ 는 jerk 의 scale) 이고 상태는 소거한다 — $x_k=\Phi_kx_0+\Gamma_kE\tilde{\mathbf u}$. $x_0$ 는 입력이다.

### 17.3 공 예측과 uncertainty

코어는 공을 전파하지 않는다. 입력은 노드 $k=0,\dots,k_c$ 의 공 표본 $(\hat p_{b,k},\hat v_{b,k},\hat a_{b,k})$ 과 포구 노드의 6×6 공분산 $\Sigma_b$ ($[p;v]$ 순서, model world) 다.

- **평균.** 예측 궤적의 표본 사이는 양 끝의 $p,v,a$ 를 맞추는 5 차 Hermite 로 보간한다 (다른 planner 와 같은 함수). §3.4 의 한 점 Taylor 전개가 아니다.
- **예측 밖의 시각.** 노드 시각이 예측의 마지막 표본을 지나면 평균은 외삽이고 공분산에는 process noise 가 없다. 코어는 그런 표본이 하나라도 있는 입력을 받지 않는다.
- **공분산.** 가장 가까운 표본 $i$ 에서 $\Sigma(t)=F(t-t_i)\,\Sigma_i\,F(t-t_i)^\top$, $F(\Delta)=\begin{bmatrix}I&\Delta I\\0&I\end{bmatrix}$ (§3.4 그대로). 표본을 대칭화하고 쓴다.
- **쓸 수 없는 공분산.** 원소가 비유한이거나, 예측 궤적과 다른 메시지의 것이거나, 분산이 음이면 '없음' 으로 표시한다 — 0 으로 바꾸지 않는다. 코어는 $\lambda_{\min}(\Sigma_b)\ge-10^{-9}\operatorname{tr}\Sigma_b$ 도 확인하고, 확률 제약이 켜져 있는데 공분산을 쓸 수 없으면 풀지 않고 사유를 낸다.
- **확률 제약을 끈 풀이.** $\Sigma_b=0$ 으로 놓는다. 17.8 의 행은 결정적 행에서 $\kappa\varepsilon_\sigma$ 만큼 조인 것이 되고 timing 행은 만들지 않는다.
- **포구 시각을 변수로 둔 코어.** 포구 노드의 평균은 입력 표본이 아니라 예측 궤적에서 $\hat t+\delta t_c$ 로 직접 읽는다 (같은 보간 함수). 공분산은 입력의 것 그대로이고, $\delta t_c$ 의 구간은 예측의 마지막 표본에서 끝난다 (17.11).

구현하지 않은 것: §3.1 의 비행 모델, §3.3 의 anticipated covariance 와 drift margin, §3.5 의 정보 시각 · occlusion, §3.6 의 $\tau_{\mathrm{react}}$, §3.7 의 jump 공분산 · NIS gating. 공분산은 메시지의 $\Sigma_b(t_c\mid s_j)$ 를 그대로 쓴다.

### 17.4 팔 예측과 inverse dynamics

$\tau$ 는 변수가 아니다. 노드마다 $\tau_k=\mathrm{RNEA}(q_k,\dot q_k,\ddot q_k)$ 로 계산하고, QP 에는 반복값 $\bar x_k$ 에서의 1 차 모델을 넣는다.

$$
\tau_k\approx\bar\tau_k+D_k(x_k-\bar x_k),\qquad D_k=\begin{bmatrix}\partial_q\tau&\partial_{\dot q}\tau&M(\bar q_k)\end{bmatrix}.
$$

- **토크 행** ($k=1,\dots,N$): $\tau_{lo}\le\tau_k\le\tau_{hi}$. 관절마다 $1/\tau_{\max,j}$ 로 나눠 건다. $\tau_{lo}$ · $\tau_{hi}$ 는 여유 $\Delta\tau$ 를 이미 뺀 입력값이다 (없으면 $\mp\tau_{\max}$). hard 행이고 17.10 의 elastic 으로 구현한다. 노드 0 은 $x_0$ 라 행이 없다. 끝 노드의 행은 정지 자세의 중력 토크 판정이다.
- **box** ($k=1,\dots,N$): $q_{\min}\le q_k\le q_{\max}$, $\vert\dot q_k\vert\le\dot q_{\max}$ 는 늘 건다. $\vert\ddot q_k\vert\le\ddot q_{\max}$ 와 $\vert u_k\vert\le j_{\max}$ 는 켜고 끄는 행이다. 선형 hard 행이라 elastic 이 없다 (포구 시각을 변수로 둔 코어에서는 포구 노드부터의 box 가 elastic 행 군이다 — 17.11). $x_0$ 가 box 밖이면 풀지 않는다.
- **종단:** $\dot q_N=\ddot q_N=0$ (등식; 포구 시각을 변수로 둔 코어에서는 elastic 행 군이다 — 17.11).
- 한계는 노드에서만 건다. §4.3 의 구간 내부 극값 검사는 코어에 없다.
- 연속 시간 기준 (§4.4) 은 구간별 3 차식이다 — 코어는 노드를 내고 RT 샘플러가 위 적분식으로 평가한다.

### 17.5 손 좌표계의 상대 위치와 속도

capture frame $H$ 는 팔 모델의 frame 이다. $R=R_{WH}$, $p_h$ 는 그 원점, Jacobian 은 `LOCAL_WORLD_ALIGNED` 하나만 쓴다 ($v_h=J_p\dot q$, $\omega_h=J_\omega\dot q$).

$$
r^W=\hat p_b-p_h,\qquad r^H=R^\top r^W,\qquad \nu^H=R^\top\big(\hat v_b-v_h-\omega_h\times r^W\big),
$$

$$
s=e_3^\top r^H,\qquad\rho=E_\perp^\top r^H,\qquad c=-e_3^\top\nu^H .
$$

$\nu^H$ 는 §5.1 의 식과 같은 양이다 ($R^\top(\omega\times r)=\omega^H\times r^H$). 미분은

$$
\frac{\partial r^H}{\partial q}=-R^\top J_p+[r^H]_\times R^\top J_\omega=\frac{\partial\nu^H}{\partial\dot q},
$$

$$
\frac{\partial\nu^H}{\partial q}=[\nu^H]_\times R^\top J_\omega+R^\top\Big(-\partial_qv_h+[r^W]_\times\partial_q\omega_h+[\omega_h]_\times J_p\Big).
$$

공은 world 에 고정된 점이라 마지막 항 $[\omega_h]_\times J_p$ 가 있다 (손에 고정된 점의 속도 미분에는 없는 항이다). $\partial_qv_h$ 는 pinocchio 의 점 속도 미분, $\partial_q\omega_h$ 는 frame 속도 미분의 각속도 행이다 (둘 다 `LOCAL_WORLD_ALIGNED`).

### 17.6 포획 조건

- **통과 평면** (§6.1). 포구 노드에 $\ell_{k_c}=0$, $\ell=s-s_{\mathrm{ent}}$. 포구 노드는 지평의 끝이 아니라 내부 노드다.
- **lateral capture set.** 면 $(\tilde a_i,\tilde b_i)$ 를 ball-center 좌표의 파라미터로 받는다 (최대 8 면, $\Vert\tilde a_i\Vert=1$). 반지름 erosion 은 코어가 하지 않는다. 행은 17.8.
- **접근 집합** $\mathcal A$ (§6.3). `Init` 때 정하고 풀이 동안 고정한다: $0\lt t_c-t_k\le T_{app}$ 인 노드 가운데 $k\ge1$ 인 것.
- **gap.** $k\in\mathcal A$ 에서 $\ell_k\ge0$ (hard — elastic).
- **corridor.** $g_c=\Vert\rho_k\Vert^2-\big(r_{\mathrm{ent}}+\ell_k^+\tan\theta+s_{c,k}\big)^2\le0$, $s_{c,k}\ge0$, $\ell^+=\max(\ell,0)$. $\ell^+$ 는 수치 가드다 — $\ell\ge0$ 이 그 자체로 제약이라 반복 중에는 어길 수 있고, 가드가 없으면 괄호가 0 이 되어 어떤 $s_c\ge0$ 로도 선형화한 행을 풀 수 없다. $\ell\ge0$ 에서는 §6.3 의 식과 같다.
- **closing envelope.** $c_k^2\le c_{\mathrm{ent,max}}^2+2a_{\mathrm{brake}}\ell_k+s_{v,k}$, $s_{v,k}\ge0$ (§6.4 그대로).
- **slack 의 값.** $s_c$ · $s_v$ 는 QP 의 변수다. 각 slack 의 벌점은 $\lambda_1+\lambda_2\gt0$ 이어야 한다 (0 이면 그 행이 사라진다). 반복값에서는 그 점의 최소값 $s_c=\max(0,\Vert\rho\Vert-r_{\mathrm{ent}}-\ell^+\tan\theta)$, $s_v=\max(0,c^2-c_{\mathrm{ent,max}}^2-2a_{\mathrm{brake}}\ell)$ 로 평가한다.
- **terminal velocity set** (§6.5) 은 조인 형태로 건다 — 17.8.

구현하지 않은 것: 무접촉 일관성 조건의 검증 (오프라인의 일), hand schedule 과 preshape 조건, M1 (contact-triggered).

### 17.7 충격

접촉점은 capture frame 의 고정점 $p_c=p_h+Rp_c^H$ ($p_c^H$ 는 파라미터), 법선은 $n=Re_3$ 다 (§8.4 의 근사 $n\approx d$).

$$
J_c=J_p-[Rp_c^H]_\times J_\omega,\qquad g_n=n^\top(\hat v_b-J_c\dot q),\qquad c_n=-g_n,
$$

$$
\beta_h=f^\top M^{-1}f,\quad f=J_c^\top n,\qquad m_{\mathrm{red}}=\Big(\frac1{m_b}+\beta_h\Big)^{-1},\qquad
E_n^-=\tfrac12m_{\mathrm{red}}c_n^2,\qquad P_n=(1+e)\,m_{\mathrm{red}}c_n .
$$

$\beta_h$ 는 $My=f$ 를 Cholesky 로 풀어 $f^\top y$ 로 계산한다. 기울기는

$$
\frac{\partial\beta_h}{\partial q}=2\,n^\top\partial_q(J_cy)\big\vert_y+2\,(J_cy)^\top\partial_qn-y^\top\partial_q(My)\big\vert_y,\qquad\partial_qn=-[n]_\times J_\omega,
$$

이고 $\partial_q(My)\vert_y$ 는 $\mathrm{RNEA}(q,0,y)$ 의 $q$ 미분에서 중력 미분을 뺀 것이다.

- **행** (임계가 유한할 때만): $g_n\le0$, $E_n^-/E_{\max}\le1$, $P_n/P_{\max}\le1$. 기본은 임계가 무한대 — 행이 없다.
- **비용** $w_EE_n^-/E_{\mathrm{ref}}$ 는 잔차 $\sqrt{m_{\mathrm{red}}/2}\;c_n$ 의 제곱으로 넣는다 (17.10 의 Gauss–Newton).

구현하지 않은 것: closed-chain $\beta_h^{cc}$ (손은 잠근 모델이다).

### 17.8 확률 제약

로봇 상태를 고정하고 공의 공분산만 쓴다 (§8.1 의 로봇 추종 항은 없다). $\Sigma_p$ 는 $\Sigma_b$ 의 위치 블록이다.

**통과 평면의 분포** (§8.2).

$$
\Sigma_r^H=R^\top\Sigma_pR,\qquad\Pi=I+\frac{\nu^He_3^\top}{\tilde c},\qquad\tilde c=\max(c,c_{\min}),\qquad
\Sigma_\rho=E_\perp^\top\Pi\,\Sigma_r^H\,\Pi^\top E_\perp,
$$

$$
\sigma_s=\sqrt{d^\top\Sigma_pd+\varepsilon_\sigma^2},\quad d=Re_3,\qquad\sigma_t=\sigma_s/\tilde c .
$$

$\tilde c$ 는 수치 가드다 — $c\ge c_{\min}$ 이 제약이라 반복 중에는 어길 수 있고 $\Pi$ 는 $c\to0$ 에서 발산한다. $c\gt c_{\min}$ 에서는 §8.2 의 식과 같다. 모든 표준편차는 $\sqrt{\text{분산}+\varepsilon_\sigma^2}$ 다 (분산 0 에서 제곱근은 미분이 없다 — §8.3 은 lateral 행에만 적었다).

**lateral 행** (포구 노드, 면마다).

$$
\tilde a_i^\top\rho+\kappa_i\sqrt{\tilde a_i^\top\Sigma_\rho\tilde a_i+\varepsilon_\sigma^2}\le\tilde b_i,\qquad\kappa_i=\Phi^{-1}(1-\epsilon_i),\quad0\lt\epsilon_i\le\tfrac12 .
$$

분산은 $\tilde a_i^\top\Sigma_\rho\tilde a_i=w^\top\Sigma_pw$, $w=R\,\Pi^\top E_\perp\tilde a_i$ 의 이차형식으로 직접 계산한다 (인수분해가 없어 특이한 $\Sigma_p$ 도 된다). 기울기는 $\Pi$ 가 $\nu^H$ 에, $\Sigma_r^H$ 가 $R$ 에 의존하는 것을 포함한다.

**속도 행** (§8.3). capture frame 의 방향 $m$ 에 대해 $\operatorname{Var}(m^\top\nu^H)=\ell^\top\Sigma_b\ell$, $\ell=\begin{bmatrix}\omega_h\times Rm\\Rm\end{bmatrix}$ (§8.1 의 $L_N$ 과 같다). $\sigma_c$ 는 $m=e_3$, $\sigma_j$ 는 $m=E_\perp u_j$ 의 것이다.

$$
c-\kappa_\nu\sigma_c\ge c_{\min},\qquad c+\kappa_\nu\sigma_c\le c_{\mathrm{cap,max}},\qquad
u_j^\top E_\perp^\top\nu^H+\kappa_\nu\sigma_j\le v_{\perp,\max}\cos(\pi/m)\quad(j=0,\dots,m-1).
$$

**timing 행** (§8.4, M2). 폐쇄는 통과보다 명목상 $\delta_0$ 뒤에 온다. $\delta_0$ 는 창의 중앙이 아닐 수 있다 (손 시퀀서가 정한다).

$$
c\,\sqrt{\sigma_{\max}^2-\sigma_\tau^2}\ge\sigma_s,\qquad
\Phi\Big(\frac{\delta_{hi}-\delta_0}{\sigma_{\max}}\Big)-\Phi\Big(\frac{\delta_{lo}-\delta_0}{\sigma_{\max}}\Big)=1-\epsilon_t .
$$

$\sigma_{\max}$ 는 `Init` 에서 이 식의 근으로 구한다 ($\delta_{lo}\lt\delta_0\lt\delta_{hi}$ 에서 좌변이 $\sigma$ 에 단조 감소라 근이 하나다). $\delta_0$ 가 중앙이면 $\sigma_{\max}=\Delta_{\mathrm{win}}/2\kappa_t$ 로 §8.4 의 식과 같다. $\delta_0$ 가 창 안 (strict) 이 아니거나 $\sigma_{\max}\le\sigma_\tau$ 이면 `Init` 이 거부한다.

**선형화 유효 조건** (§8.2). 제약이 아니라 풀이 뒤의 진단값이다 — $\tfrac12\Vert E_\perp^\top a_{\mathrm{rel}}\Vert(\kappa\sigma_t)^2$ 를 가장 가까운 면까지의 여유로 나눈 비. 여유가 양이 아니거나 $\tilde c$ 가 가드에 걸리면 '정의 안 됨' 으로 낸다.

구현하지 않은 것: sensor coverage (M1), hand 정보 시각의 공분산, $\operatorname{Cov}(\delta\rho,\delta t)$, $J_{\mathrm{unc}}$ (§8.5), 공–link clearance 의 margin (§8.6), 2 차 보정 · scenario 검증.

### 17.9 목적함수

$$
\begin{aligned}
J=\;&\Delta_a\sum_{k=0}^{k_c-1}\Big[\Vert\tau_k\Vert^2_{R_\tau}+\Vert\ddot q_k\Vert^2_{R_a}+\Vert u_k/u_s\Vert^2_{R_j}+\Vert q_k-q_{\mathrm{nom}}\Vert^2_{Q_q}+w_m\psi_m(q_k)\Big]\\
&+\Delta_a\sum_{k=0}^{k_c-1}\rho_T(t_k)\Big[\Vert r_k^H-r_{\mathrm{ref},k}^H\Vert^2_{Q_p}+\Vert\nu_k^H-\nu_{\mathrm{ref}}^H\Vert^2_{Q_v}\Big]\\
&+\Vert\rho_{k_c}-\rho_{\mathrm{ref}}\Vert^2_{Q_{\rho,f}}+\Vert\nu_{k_c}^H-\nu_{\mathrm{ref}}^H\Vert^2_{Q_{\nu,f}}+w_E\frac{E_n^-}{E_{\mathrm{ref}}}
+\Delta_a\sum_{k\in\mathcal A}\big(\lambda_1^\top s_k+\Vert s_k\Vert^2_{\Lambda_2}\big)\\
&+\Delta_s\sum_{k=k_c}^{N-1}\Vert u_k/u_s\Vert^2_{R_{j,stop}}+\Delta_s\,w_\perp\sum_{k=k_c}^{N}\big\Vert P_\perp\big(p_h(q_k)-p_{\mathrm{line}}\big)\big\Vert^2 .
\end{aligned}
$$

$\rho_T(t)=\exp[-(t-t_c)^2/2\sigma_T^2]$, $r_{\mathrm{ref},k}^H=r_{\mathrm{ref}}^H+(t_c-t_k)(-\nu_{\mathrm{ref}}^H)$, $r_{\mathrm{ref}}^H=(\rho_{\mathrm{ref}},s_{\mathrm{ent}})$ 다. 포구 시각을 변수로 둔 코어에서는 $r_{\mathrm{ref},k}^H$ 의 $t_c$ 가 $\hat t+\delta t_c$ 이고 $\rho_T(t_k)$ 는 $\delta t_c=0$ 의 값이다 (17.11).

- 앞의 세 줄이 §9.2 – §9.5 의 항이고 마지막 줄은 정지 구간의 항이다 (jerk 와, 가중이 0 이 아닐 때 정지 직선에서 벗어난 거리). 결과는 두 묶음을 따로 낸다.
- running cost 는 구간 길이를 곱하고, 식에 $\tfrac12$ 은 없다 (§9.1).
- **노드 0.** $x_0$ 가 고정이라 노드 0 의 $\tau$ · $\ddot q$ · $q$ · $\psi_m$ · 근방 항은 상수다. QP 에는 노드 $1,\dots,k_c-1$ 만 들어가고 노드 0 의 값은 결과의 $J$ 에 더한다 — 후보 사이에서 $J$ 를 비교할 수 있게. $u_0$ 는 변수라 jerk 항은 stage 0 부터 있다. $k_c=1$ 이면 QP 에 running 항과 접근 행이 없다.
- 입력이 jerk 라 §9.2 의 $j_k$ 는 $u_k$ 그 자체다.
- $\psi_m=-\log\det(\bar J\bar J^\top+\delta I)$, $\bar J=D_x^{-1}JD_q$ ($J$ 는 6×$n$ frame Jacobian). QP 에는 기울기만 넣는다 — $\partial\psi_m/\partial q_k=-2\operatorname{tr}\big(A^{-1}\,\partial_{q_k}\bar J\,\bar J^\top\big)$, $A=\bar J\bar J^\top+\delta I$, $\partial_{q_k}J$ 는 운동학의 Hessian. 가중이 0 이면 계산하지 않는다.

구현하지 않은 것: $J_{\mathrm{unc}}$, 자세 항 $\Vert e_R\Vert^2_{Q_R}$. $J_{\mathrm{time}}$ · $J_{\mathrm{switch}}$ 는 포구 시각이 고정인 코어에서는 한 후보 안의 상수라 바깥 루프의 것이다. 포구 시각을 변수로 둔 코어는 그 둘을 $\delta t_c$ 의 함수로 받아 같이 최소화하고, 값은 $J$ 와 따로 낸다 (17.11).

### 17.10 푸는 문제와 풀이

**문제.** 17.9 의 $J$ 를 $z$ 와 slack 에 대해 최소화한다 (포구 시각을 변수로 둔 코어는 $\delta t_c$ 에 대해서도 — 17.11). 제약은 17.4 의 종단 등식 · box · 토크 행, 17.6 의 gap · corridor · envelope · 통과 평면, 17.8 의 lateral · 속도 · timing 행, 17.7 의 충격 행 (켰을 때) 이다. §10 의 충돌 행, 공–link clearance 행, M1 행은 없다.

**SQP.** 반복값은 변수 공간의 점 $\bar z$ 다. 반복마다 $\bar x=x(\bar z)$ 에서 선형화하고 step $d$ 에 대한 QP 하나를 푼다 (ProxQP). Hessian 은 Gauss–Newton — 제곱 항마다 $2L^\top WL$ ($L$ 은 잔차의 $z$ 에 대한 Jacobian) 이고 $R_j\gt0$ 이라 양정치다.

**hard 인 비선형 행은 elastic 이다.** (행 군, 노드) 마다 변수 $e\ge0$ 하나를 그 군의 행들이 같이 쓰고 비용에 $\mu_Ge$ 를 더한다. 군과 위반의 단위는 다음과 같다.

| 군 | 행 | 단위 |
|---|---|---|
| 토크 | $\tau_{lo}\le\tau_k\le\tau_{hi}$, 노드마다 | $\tau_{\max}$ 의 비 |
| gap | $\ell_k\ge0$, 접근 노드마다 | m |
| 통과 평면 | $-e\le\ell_{k_c}\le e$ | m |
| lateral | lateral 행 전부 | m |
| timing | timing 행 | m |
| 속도 집합 | 축 방향 둘 + 다각형 $m$ 면 | m/s |
| 충격 | $g_n$, $E_n^-/E_{\max}$, $P_n/P_{\max}$ | m/s, 임계의 비 |

corridor 와 envelope 는 자기 slack 이 있어 elastic 이 없다. box 와 종단 등식은 완화하지 않는다 — 포구 시각을 변수로 둔 코어에서는 포구 전 노드의 box 와 jerk box 만 그렇고, 포구 노드부터의 box 와 종단 정지는 elastic 행 군이다 (17.11).

**실행 가능의 뜻.** 해는 hard 행 전부를 비선형 모델로 다시 평가해 위반이 허용 오차 안일 때만 실행 가능하다. elastic 은 풀리지 않는 문제의 진단이지 완화가 아니다.

**merit.** $\phi(z)=J(z)+\sum_G\mu_G\sum_{\text{노드}}\max_{i\in G}\mathrm{viol}_i(z)$ — QP 와 같은 군 · 같은 단위다. slack 은 그 점의 최소값으로 평가한다. QP 의 선형 모델이 예측하는 변화는

$$
D=\nabla J^\top d+\big[J_{\mathrm{slack}}(s^{QP})-J_{\mathrm{slack}}(\bar s)\big]+\sum_G\mu_G\Big(\sum e^{QP}-\sum\max_i\mathrm{viol}_i(\bar z)\Big)
$$

이고 QP 를 정확히 풀면 $D\le0$ 이다. step 길이 $\alpha$ 는 1 에서 시작해 줄이며

$$
\phi(\bar z+\alpha d)\le\phi(\bar z)+\alpha\,\big(D^\ast+2\,\eta_{QP}\big),\qquad D^\ast=\begin{cases}\eta D,&D\lt0\\D,&D\ge0\end{cases}
$$

을 만족하는 첫 값을 받는다. $\eta_{QP}=\sum_i\mu_{G(i)}\,(\text{행 }i\text{ 의 잔차})^+$ 는 그 QP 해가 elastic 행에 실제로 남긴 잔차의 벌점 값이다 (해에서 잰 값). QP 를 허용 오차까지만 풀면 0 이어야 할 elastic 이 남아 $D\gt0$ 이 되기도 하고 행에 잔차가 남기도 한다. 해 근처에서는 얻을 감소가 그보다 작아지므로, 이것을 허용하지 않으면 판정이 step 자신의 잡음보다 작은 감소를 요구하게 된다. 그래서 도달할 수 있는 KKT 잔차의 하한은 대략 $\mu\times$ (QP 의 절대 허용 오차) 다.

**벌점의 갱신.** 벌점이 정확하려면 $\mu_G\gt\sum_{i\in G}\vert\lambda_i\vert$ 여야 한다. QP 에 elastic 이 남아 있는 동안은 multiplier 가 $\mu$ 에 붙어 있어 필요한 크기를 알려 주지 않는다. 그래서 벌점을 기하적으로 키워 QP 를 다시 풀되, 그 증가가 선형화한 행의 실행 가능성을 살 때만 받아들이고 아니면 되돌린다 — elastic 의 합이 정해진 비율 이상 줄거나, (trust region 이 한 step 이 없앨 수 있는 양을 묶고 있을 때) 그 step 이 없애는 위반이 정해진 비율 이상 늘 때다. step 이 위반을 오히려 늘리고 있었으면 (QP 가 비용을 위해 행을 내주는 경우) 없애던 양을 0 으로 본다. 어느 쪽도 아니면 그 선형화에서 행을 만족할 수 없는 것이고 $\mu$ 를 더 키워도 QP 의 조건만 나빠진다. 키울 때는 **모든 군을 같은 배율로** 키운다 (가장 큰 것이 상한에 닿을 때까지) — 군 사이의 비는 입력 그대로이고, 풀리지 않는 문제의 잔류가 어느 군에 남는지는 그 비가 정한다.

**QP 의 설정.** 이 QP 는 구성상 늘 실행 가능하다 (시작점이 선형 행을 만족하고, 비선형 행마다 elastic 이나 slack 이 있다). 그래서 solver 의 primal infeasibility 판정은 끈다 — 그 판정은 근사 인증서를 받아들여, 벌점이 큰 실행 가능한 QP 를 실행 불가로 보고했다. 초기화 QP 는 다르다: $x_0$ 에 따라 선형 행을 만족하는 시작점이 없을 수 있고 elastic 도 없으므로 판정을 켜 둔다. '시작점 없음' 은 그 판정이 났을 때만 보고하고, 초기화 QP 가 다른 이유로 풀리지 않으면 QP 실패다.

**시작점.** 시작점은 늘 선형 행 (box · 종단 정지) 을 만족한다. 줄탐색이 그 볼록 집합 안에 머물러 반복값 전부가 선형 행을 만족하므로 QP 는 늘 풀린다. 포구 시각을 변수로 둔 코어에서는 포구 시각을 옮긴 뒤의 반복값이 포구 노드부터의 box 와 종단 정지를 어길 수 있다 — 그 두 군이 elastic 이라 QP 는 그때도 풀린다 (17.11).

- 호출자가 준 노드 궤적이 있으면 블록 jerk 로 사영한다 — 블록마다 노드 가속 차분 $(\ddot q_{k+1}-\ddot q_k)/\Delta_k$ 의 평균. 같은 격자의 이전 해는 정확히 재현된다. 사영한 궤적이 선형 행을 만족하면 그것이 시작점이다.
- 아니면 초기화 QP 를 푼다: 선형 행 아래에서 목표까지의 거리와 jerk 를 최소화한다. 목표는 호출자의 노드 궤적 (있을 때), 또는 포구 노드의 관절 자세 $q^\ast$ (탐색의 IK 해) 와 속도 $\dot q^\ast=J_p^\top(J_pJ_p^\top+\lambda^2I)^{-1}\big(\hat v_b-R\,\nu_{\mathrm{ref}}^H\big)$ 다. 목표를 속도 한계로 미리 자르지 않는다 — 한계는 QP 의 행이 건다.

**끝나는 조건.**

- 수렴: $\Vert\nabla_zL\Vert_\infty\le\epsilon_{KKT}\max(1,\Vert\nabla_zJ\Vert_\infty)$, hard 행의 위반과 상보성이 허용 오차 안. $\nabla L=g+C^\top\lambda+A^\top y$ 는 QP 의 multiplier 로 계산한다 (jerk 변수에서 $-Hd$ 와 같다). trust region 의 multiplier 는 문제의 것이 아니므로 뺀다.
- 실행 불가: QP 에 elastic 이 남고 (벌점을 키워도 줄지 않고), 반복값이 정류점이거나 hard 행의 위반이 정해진 반복 수 동안 정해진 비율만큼 줄지 않았을 때. 사유와 함께 위반이 가장 큰 군을 낸다.
- step 이 trust region 에 걸려 있는 동안 (그 multiplier 가 정류점 판정을 좌우할 만큼 클 때) 은 수렴도 실행 불가도 판정하지 않는다. 느린 진행은 그 상한 때문이지 행 때문이 아니다 — 작은 trust region 아래의 실행 가능한 문제는 반복 상한이나 기한으로 끝난다.
- 반복 상한, 기한, 줄탐색 실패, QP 실패. 기한은 반복 사이에 본다. 어느 경우든 마지막으로 수용한 반복값과 그 점의 위반 · 비용 · KKT 잔차를 낸다.
- 반복 상한이 1 이면 줄탐색 없이 full step 하나를 낸다 (real-time iteration). 수렴은 시작점이 이미 수렴 조건을 만족할 때만 보고되고, 그때는 step 을 내지 않는다.

결과는 '실행 가능' 과 '수렴' 을 따로 낸다. 포구 시각을 변수로 둔 코어가 포구 시각을 옮기는 방법과 그때의 끝나는 조건은 17.11 에 있다.

구현하지 않은 것: second-order correction, 단일 NLP 형태의 연속 포획 시각 (§11.5), 진단용 relaxation $s_f$ 를 따로 푸는 것 (elastic 이 그 역할을 한다).

### 17.11 바깥 루프 — 포구 후보의 탐색

§11 과, 17.9 가 바깥 루프의 것으로 남긴 §9.5 의 $J_{\mathrm{time}}$ · $J_{\mathrm{switch}}$ 의 구현이다. `rtc_controllers` 의 `NlpCatchSearch` (`catching/nlp_catch_search.hpp`) 가 예측 하나에 대해 한 번 도는 탐색 (`Plan`) 이고, 그것이 쓰는 격자 · 필요조건 · 바깥 비용의 식은 상태 없는 함수로 따로 있다 (`catching/nlp_catch_screening.hpp`). 후보마다의 팔 문제는 17.1 – 17.10 의 코어가 푼다.

**시작할 수 있는 시각.** 이번 탐색의 결과가 팔에 닿을 수 있는 가장 이른 시각을 $t_0$ 로 둔다.

$$
t_0=t_{\mathrm{now}}+T_{\mathrm{arm}}+T_{\mathrm{budget}}+T_{\mathrm{lead}}
$$

$T_{\mathrm{arm}}$ 은 지령이 팔에 닿는 지연, $T_{\mathrm{budget}}$ 은 탐색 한 번의 예산, $T_{\mathrm{lead}}$ 는 그 뒤의 여유다.

**후보 집합 (§11.1).** 후보는 절대 시각 격자의 점이다.

$$
\mathcal T=\{\,t_c^{(i)}=t_{\mathrm{ref}}+i\,h\,\}\cap(t_0,\ t_0+T_{\max}],\qquad T^{(i)}=t_c^{(i)}-t_0 .
$$

- $t_{\mathrm{ref}}$ 는 그 시도의 첫 탐색의 $t_{\mathrm{now}}$ 이고 시도가 끝나거나 공의 track 이 바뀌면 다시 정한다. $h$ 는 파라미터다 ($\Delta_m$ 의 약수로 묶지 않는다). 후보의 index $i$ 가 탐색 사이에서 그 후보의 정체성이다.
- $T^{(i)}\lt T_{\min}$ 인 후보는 버리되 사유를 남긴다. $T_{\min}$ 은 파라미터이고 $T_{\min}\ge n_{pre,\min}\Delta_a$, $T_{\max}\le n_{pre,\max}\Delta_a$ 가 아닌 구성은 받지 않는다 — 포구 전 격자가 후보의 창을 덮는다.
- 공은 후보의 격자 노드마다 17.3 의 방법으로 읽는다. 노드 하나라도 예측의 끝을 넘으면 그 후보는 없다.

**후보의 격자.** 17.2 의 격자를 후보의 $t_c$ 에 닻 내린다. 포구 전 구간 수는 들어가는 만큼이다.

$$
n_{pre}=\Big\lfloor\frac{t_c-t_0}{\Delta_a}\Big\rfloor,\qquad t_s=t_c-n_{pre}\Delta_a\in[t_0,\ t_0+\Delta_a).
$$

노드 0 의 시각이 $t_s$ 다. $[t_0,t_s)$ 동안 팔은 하던 운동을 계속한다 — 그 구간은 비용에 없다. 전부 정수 ns 로 계산한다. 코어는 $n_{pre}=n_{pre,\min},\dots,n_{pre,\max}$ 마다 하나씩 미리 만들고, 파라미터는 한 벌이다: 포구 전은 구간마다 블록 하나, 정지 구간의 블록은 파라미터.

**필요조건 (§11.3).** 후보마다 아래 순서로 보고, 처음 걸린 것이 그 후보의 사유다.

1. (S1) $T\ge T_{\min}$.
2. 격자의 모든 노드에서 공을 읽을 수 있고 $\Vert\hat v_b(t_c)\Vert$ 가 하한 이상이다.
3. $\hat p_b(t_c)$ 가 포구 작업 영역 (box) 안이다.
4. 확률 제약을 쓰면 포구 노드의 공분산이 있다.
5. 출발 상태를 얻을 수 있다 (17.12).
6. (S4) 포구 자세의 IK 해 $q^c$ 가 있다. 목표는 capture frame 의 원점 $\hat p_b(t_c)+s_{\mathrm{ent}}\hat v$ ($\hat v=\hat v_b/\Vert\hat v_b\Vert$) 와 접근축 $e_3=-\hat v$ 이고, seed 는 대기 자세 — 후보 · 탐색 · 팔의 상태와 무관하게 같다. 해는 하나이고 IK 의 조작성 gate 를 같이 지난다.
7. (S4) 관절마다

$$
\vert q_j^c-q_{0,j}\vert\le v_{\max,j}T_m,\qquad \vert q_j^c-q_{0,j}-v_{0,j}T_m\vert\le\tfrac12a_{\max,j}T_m^2,\qquad T_m=n_{pre}\Delta_a .
$$

   $(q_0,v_0)$ 는 출발 상태, $v_{\max}$ · $a_{\max}$ 는 코어의 box 다. 둘째 식은 코어에 가속 box 가 있을 때만 본다. $T$ 가 아니라 팔이 실제로 움직이는 시간 $T_m$ 이다.
8. (S3) closing speed 창이 비어 있지 않다.

$$
\max\{c_{\min},\,c_{t,\mathrm{lo}}\}\le\min\{c_{\mathrm{cap,max}},\,c_{n,\mathrm{hi}}\},\qquad
c_{t,\mathrm{lo}}=\frac{\sigma_s}{\sqrt{\sigma_{\max}^2-\sigma_\tau^2}},\qquad
c_{n,\mathrm{hi}}=\min\Big\{\sqrt{\tfrac{2E_{\max}}{m_{\mathrm{red}}}},\ \frac{P_{\max}}{(1+e)\,m_{\mathrm{red}}}\Big\}.
$$

   $\sigma_s=\sqrt{d^\top\Sigma_pd+\varepsilon_\sigma^2}$, $d=R(q^c)e_3$, $\sigma_{\max}$ 는 17.8 의 것, $m_{\mathrm{red}}$ 는 17.7 의 것을 $q^c$ 에서 계산한다. timing 행이나 충격 행이 꺼져 있으면 그 항은 없다.

**순위와 예산 (§11.2 · §11.3).** 필요조건을 지난 후보를

$$
J_{\mathrm{time}}+J_{\mathrm{switch}}+w_{rq}\Vert q^c-q_0\Vert^2+w_{rm}\,\psi_m(q^c)
$$

의 오름차순으로 세우고 (같으면 index 가 작은 쪽) 앞의 $L$ 개만 푼다.

$$
L=\min\Big\{L_{\max},\ \Big\lfloor\frac{T_{\mathrm{budget}}-t_{\mathrm{screen}}}{T_{\mathrm{solve}}}\Big\rfloor\Big\}
$$

$t_{\mathrm{screen}}$ 은 필요조건에 쓴 시간, $T_{\mathrm{solve}}$ 는 후보 하나의 몫이다. 풀이마다 자기 시작 시각에서 잰 자기 기한 (몫) 을 갖는다 — 다른 후보가 쓰거나 남긴 시간은 넘어오지 않는다. $L$ 은 첫 풀이 전에 정하고, 앞 풀이가 시간을 넘겼다고 뒤 풀이를 건너뛰지 않는다 (무엇을 푸는가가 푸는 순서에 달리지 않게). $T_{\mathrm{solve}}\le T_{\mathrm{budget}}$ 가 아니면 구성을 거부한다. worker 는 하나다. 코어 · 입력 · 결과 · 후보의 기억은 구성할 때 전부 만든다. RT 가 이 track 의 plan 을 따르고 있으면 그 plan 의 셀의 후보가 위 순서와 무관하게 맨 앞이다 (17.12).

**시작점 (§11.4).** 풀이의 시작점은 **앞선 탐색들의 기억**에서만 가져온다. 이번 탐색의 해는 따로 모았다가 탐색이 끝날 때 기억에 넣으므로 풀이의 순서가 결과를 바꾸지 않는다. 기억은 후보의 index 마다 하나이고, 이번에 풀지 않은 후보의 기억은 남는다.

기억하는 것은 풀이가 **끝난 점**이다 — 유효한 해뿐 아니라 기한에 잘린 것, 수렴하지 않은 것, hard 행이 어긋난 채 정류한 것도. 마지막 것은 예측이 좁혀지면 풀이가 가장 가까이 머무는 점이다. 기억하지 않는 것은 solver 가 실패한 풀이의 점 (QP 가 수렴하지 않았거나 평가가 유한하지 않음) 이다: 그 후보의 기억은 지우고, 다음 탐색은 그 후보를 다른 곳에서 시작한다.

- 같은 후보의 해가 있으면 지난 노드를 버린다. 그 해의 포구 전 구간 수가 $n_{pre}^{\mathrm{prev}}$ 이면 새 격자의 노드 $k$ 는 그 해의 노드 $k+(n_{pre}^{\mathrm{prev}}-n_{pre})$ 다 (두 격자가 같은 $t_c$ 에 닻 내려 있어 시각이 같다).
- 없으면 index 가 가장 가까운 후보의 해를 절대 시각의 함수 (구간별 3 차식) 로 보고 $t_s$ 를 중심으로 늘인다.

$$
\sigma(t)=t_s+\alpha\,(t-t_s),\qquad \alpha=\frac{t_c^{\mathrm{prev}}-t_s}{t_c-t_s},\qquad
\tilde q_k=q^{\mathrm{prev}}(\sigma(t_k)),\quad \tilde v_k=\alpha\,v^{\mathrm{prev}}(\sigma(t_k)),\quad \tilde a_k=\alpha^2a^{\mathrm{prev}}(\sigma(t_k))\quad(k\le k_c).
$$

  포구 뒤의 노드는 그 해의 정지 구간을 그대로 옮긴다. 그 해가 $t_s$ 를 덮지 않으면 (노드 0 이 $t_s$ 뒤거나 포구가 $t_s$ 앞) 쓰지 않는다.
- 어느 쪽이든 노드 0 은 출발 상태로 바꾼다. 쓸 기억이 없으면 $q^c$ 를 포구 노드의 목표로 준다 (17.10 의 시작점).

**유효한 후보와 선택 (§11.1 · §9.5).** 후보는 다음을 모두 만족할 때 유효하다: 코어가 반복값을 냈고, 풀이가 자기 몫 안에 끝났고, hard 행 전부가 허용 오차 안이고 (17.10 의 "실행 가능"), 수렴했고, RT 가 그 구간을 노드 0 부터 읽을 수 있다.

마지막 조건은 탐색이 예산을 넘겼을 때만 걸린다. 코어는 기한을 반복 사이에서만 보므로 (17.10) 풀이는 몫을 넘기고, 넘긴 것이 쌓이면 탐색이 $T_{\mathrm{budget}}$ 뒤에 끝난다. $t_0$ 는 탐색이 예산 안에 끝난다고 보고 잡은 시각이므로, 탐색이 $\delta$ 만큼 늦게 끝났으면 노드 0 이 $t_0$ 에서 $\delta$ 안쪽인 후보 ($t_s-t_0\lt\delta$) 의 구간은 RT 에 닿을 때 이미 노드 0 을 지나 있다. $\delta$ 는 마지막 풀이 뒤에 한 번 재므로 푸는 순서와 무관하다. $t_{\mathrm{now}}$ 에서 탐색이 시작될 때까지와 탐색이 끝난 뒤 RT 가 읽을 때까지의 시간은 탐색이 재지 못한다 — $T_{\mathrm{lead}}$ 가 덮어야 한다.

유효한 후보 가운데

$$
\Phi=J^\star+J_{\mathrm{time}}+J_{\mathrm{switch}},\qquad
J_{\mathrm{time}}=w_T\frac{T}{T_{\mathrm{ref}}},\qquad
J_{\mathrm{switch}}=w_{\mathrm{sw}}\Big(\frac{t_c-t_{c,\mathrm{prev}}}{T_{\mathrm{ref}}}\Big)^2
$$

이 가장 작은 것을 고른다 (같으면 index 가 작은 쪽). $J^\star$ 는 17.9 의 $J$ 에서 **정지 구간의 항 (마지막 줄) 을 뺀 것**이다 — 코어가 최소화한 것은 둘의 합이고, 고르는 값은 그 최소점에서 잰 §9 의 비용이다. 정지 구간의 값은 기록한다. $t_{c,\mathrm{prev}}$ 는 RT 가 plan 을 따르고 있으면 그 plan 의 $t_c$, 아니면 이 시도에서 탐색이 직전에 고른 $t_c$ 이고, 둘 다 없으면 $J_{\mathrm{switch}}=0$ 이다.

고른 후보에서 내는 것은 $t_c$, $p_{\mathrm{catch}}=\hat p_b(t_c)$, $\hat v_b(t_c)$, 접근축 $-\hat v$, 해의 포구 노드 자세 $q_{k_c}^\star$, 그 자세에서의 조작성 (IK 의 gate 가 쓰는 두 값 — IK 의 자세 $q^c$ 가 아니라 $q_{k_c}^\star$ 에서 다시 계산한다), $\Phi$, $\sqrt{\lambda_{\max}(\Sigma_p(t_c))}$, 예상 충격량 $m_b\,\hat c_{k_c}$ 그리고 해 전체 (노드 궤적) 다.

**RT 가 plan 을 따르고 있을 때의 보고.** 고른 $t_c$ 가 따르는 plan 의 것과 ns 단위로 같으면 "같은 포구 — 갱신", 다르면 "교체" 다. 유효한 후보가 없으면 아무것도 내지 않는다 (RT 는 가진 것을 따른다).

**사유.** 후보마다 하나, 탐색마다 하나다. 탐색이 아무것도 고르지 못했으면 가장 멀리 간 후보의 사유가 탐색의 사유다.

| 순서 | 사유 | 뜻 |
|---|---|---|
| 1 | 창 밖 | 채택 뒤의 창 (`follow_window`) 밖의 셀 — 다른 검사보다 먼저 본다 (17.12) |
| 2 | lead 부족 | $T\lt T_{\min}$ |
| 3 | 공 | 격자의 노드에서 공을 읽을 수 없거나 속력이 하한 아래 |
| 4 | 작업 영역 | $\hat p_b(t_c)$ 가 box 밖 |
| 5 | 공분산 | 확률 제약에 쓸 공분산이 없음 |
| 6 | 출발 구간 없음 | 그 후보의 $t_s$ 에 RT 가 보고한 구간이 없음 (17.12) |
| 7 | IK | 포구 자세의 IK 가 수렴하지 않음 |
| 8 | 조작성 | IK 의 조작성 gate |
| 9 | 도달 | (S4) 의 두 식 |
| 10 | 속도 창 | (S3) 의 창이 빔 |
| 11 | 순위 밖 | 필요조건은 지났으나 $L$ 안에 들지 못함 |
| 12 | 기한 | 풀이가 자기 몫을 넘김, 또는 탐색이 늦게 끝나 노드 0 부터 읽을 수 없음 |
| 13 | 풀이 거부 | 코어가 반복값 없이 거부 (17.10 — 시작점 없음 등), 또는 solver 가 실패한 점을 냄 (QP 미수렴 · 유한하지 않은 평가) |
| 14 | hard 행 | 확률 제약이 아닌 hard 행이 하나라도 어긋남 |
| 15 | 확률 제약 | 어긋난 것이 lateral · timing · 속도 집합의 행뿐 |
| 16 | 미수렴 | hard 행은 맞으나 수렴하지 않음 |

탐색에만 있는 사유: 후보 없음 (창 안에 격자점이 없음), 정지 아님 · RT 상태 못 씀 · 출발 구간 없음 (17.12).

14 와 15 는 코어가 끝난 점에서 어느 군의 행이 어긋나 있는가로 가른다. 풀리지 않는 풀이가 위반을 어느 군에 남기는가는 17.10 의 벌점 비가 정한다. 팔의 자세와 무관한 확률 제약 (공 속도의 불확실성이 큰 경우의 속도 집합) 은 15 로 끝난다. 팔의 자세로 조금 줄일 수 있는 확률 제약 (측면 위치의 불확실성이 큰 경우의 lateral 행) 은, 줄이려다 다른 군의 행까지 어긴 채 끝날 수 있고 그때는 14 다.

**셀 안의 포구 시각 — 코어 (`catch_time_variable`).** 격자의 후보 사이는 $h$ 만큼 떨어져 있고 가장 좋은 포구 시각은 대개 격자 위에 있지 않다. 스위치를 켠 코어는 변수 하나 $\delta t_c$ 를 더한다. 닻 $\hat t$ 는 호출자가 준 시각이고 (탐색에서는 후보의 격자 시각 $t_c^{(i)}$) 포구 노드가 $\hat t+\delta t_c$ 에 있다. 기본은 꺼짐이고, 끈 코어에는 아래의 어느 것도 없다 — 변수와 행의 수 · 계산 경로 · 결과의 수치가 스위치가 없던 코어와 같다.

격자는 한 곳에서만 늘어난다. 포구 노드 앞 구간의 길이를 $\tau=\Delta_a+\delta t_c$ 라 하면

$$
t_k=\begin{cases}\hat t-(k_c-k)\Delta_a,&k\lt k_c\\ \hat t+\delta t_c,&k=k_c\\ \hat t+\delta t_c+(k-k_c)\Delta_s,&k\gt k_c\end{cases}
$$

이다 — 포구 전 구간을 균일하게 늘이는 것이 아니다. 노드 0 의 시각 $t_s$ 와 $x_0$, 포구 전 노드는 $\delta t_c$ 에 의존하지 않는다. 포구 노드의 상태는 구간 $k_c-1$ 의 3 차식 (17.2) 을 $\tau$ 에서 평가한 것이고 (정확), 그 미분은

$$
\frac{\partial x_{k_c}}{\partial\delta t_c}=\bar f_c=\big(\dot q_{k_c},\ \ddot q_{k_c},\ u_{k_c-1}\big),\qquad
\frac{\partial x_k}{\partial\delta t_c}=A(\Delta_s)^{k-k_c}\,\bar f_c\quad(k\gt k_c)
$$

다. $A(\Delta)$ 는 17.2 의 전이식이 $x_k$ 에 곱하는 행렬이다. 노드 $k\ge k_c$ 의 stage gain 과 자유 응답은 평가하는 점의 $\delta t_c$ 에서 다시 만든다 — $\delta t_c=0$ 의 것을 쓰지 않는다.

- **포구 전 노드.** 포구 노드 앞 노드의 행과 비용에는 $\delta t_c$ 의 열이 없다. 예외는 비용의 근방 항 하나다: 기준선 $r_{\mathrm{ref},k}^H=r_{\mathrm{ref}}^H+(\hat t+\delta t_c-t_k)(-\nu_{\mathrm{ref}}^H)$ 가 포구 시각과 함께 움직여 잔차 $r_k^H-r_{\mathrm{ref},k}^H$ 의 $\delta t_c$ 기울기가 $+\nu_{\mathrm{ref}}^H$ 다. 상태가 상수인 노드 0 의 항도 그렇다.
- **공.** 포구 노드의 공 평균은 예측 궤적에서 $\hat t+\delta t_c$ 로 읽는다 (17.3 의 보간 함수). 포구 노드의 행과 비용은 공을 $r^H$ · $\nu^H$ 로만 읽으므로, 그 $\delta t_c$ 계수에는 공에 대한 미분에 $\dot{\hat p}_b=\hat v_b$, $\dot{\hat v}_b=\hat a_b$ 를 곱한 것이 더해진다.

$$
\frac{\partial r^H}{\partial\hat p_b}=R^\top,\qquad\frac{\partial r^H}{\partial\hat v_b}=0,\qquad
\frac{\partial\nu^H}{\partial\hat p_b}=-R^\top[\omega_h]_\times,\qquad\frac{\partial\nu^H}{\partial\hat v_b}=R^\top .
$$

- **$\delta t_c=0$ 의 값으로 두는 것.** 공분산 $\Sigma_b$ (호출자가 준 것 그대로), 근방 가중 $\rho_T(t_k)$, 접근 집합 $\mathcal A$, 늘어난 구간의 running cost 가중 $\Delta_a$. 그 구간의 jerk 는 여전히 $u_{k_c-1}$ 이고, 노드 가속에서 되찾을 때는 $\tau$ 로 나눈다.
- **elastic 인 두 군.** stage gain 을 거쳐 $\delta t_c$ 에 의존하는 선형 행은 노드 $k\ge k_c$ 의 box 와 종단 정지다. 둘은 17.10 의 hard 인 비선형 행과 같은 elastic 행 군이다 — box 는 노드마다 elastic 하나, 종단 정지는 양쪽을 합쳐 하나이고 벌점은 군마다 파라미터다. merit 에 들어가고, '실행 가능' 은 비선형 모델로 다시 평가해 정한다. 그 노드들의 trust region 은 box 와 같이 쓰던 행에서 나와 자기 행을 갖는다. 포구 전 노드의 box 와 jerk box 는 hard 인 선형 행 그대로다.
- **시각의 항.** 비용에 $c_1\,\delta t_c+c_2\,(\delta t_c-\delta_{\mathrm{ref}})^2$ 을 더한다. $c_1$, $c_2\ge0$, $\delta_{\mathrm{ref}}$ 는 입력이다. QP 와 merit 는 이 항을 갖고, 결과는 그 값을 17.9 의 $J$ 와 따로 낸다 (합에 넣지 않는다).

**풀이는 중첩이다.** $\theta=\delta t_c/\Delta_a$ 는 QP 의 변수이고 ($[\,d\mid\theta\mid s_c\mid s_v\mid e\,]$) 포구 시각과 함께 움직이는 모든 행과 비용 항에 $\theta$ 의 열이 있다. 그러나 QP 안에서 $\theta$ 는 움직이지 않는다 — $\theta$ 의 행이 0 에 묶는다.

- 안쪽: $\delta t_c$ 를 고정한 문제다. 17.10 의 SQP 그대로다.
- 바깥쪽: 안쪽 문제가 끝나면 (풀렸거나, 맞출 수 없는 행을 남긴 채 정류했으면) 묶은 행의 multiplier 가 $-\partial L/\partial\theta$ 이고, envelope 정리에 의해 이것은 안쪽 문제의 최적값을 $\theta$ 로 미분한 값이다. $\delta t_c$ 를 그 미분의 부호가 바뀌는 구간 안에서 secant 로 옮긴다 — 한 번에 `delta_t_step` 이하, 호출자가 준 구간 $[\delta_{lo},\delta_{hi}]$ 안의 정수 ns 로.

$(d,\theta)$ 를 한 QP 에서 같이 밟지 않는 까닭은 2 차 항이다. jerk 를 둔 채 포구 시각을 옮기면 종단 정지와 통과 평면이 jerk 의 변화 × 시각의 변화만큼 어긋나는데, 고정 격자의 해에서 잰 20 ms 의 step 은 목적함수를 0.02 줄이면서 종단 속도 1.3 rad/s 를 남겼고 ℓ₁ merit 는 약 0.1 µs 보다 큰 step 을 전부 거부했다.

- **끝나는 조건.** 포구 시각이 더 움직이지 않는 것은 그 미분이 KKT 허용 오차 안이거나, 구간의 끝에서 바깥을 가리키거나, 이웃한 두 ns 사이에서 부호가 바뀔 때다. 그때 안쪽 문제가 풀려 있으면 수렴이고, 17.10 의 실행 불가로 끝나 있으면 실행 불가다. KKT 잔차는 jerk 변수의 것과 $\partial L/\partial\theta$ 를 함께 본다.
- 반복 상한이 1 인 풀이 (real-time iteration) 는 포구 시각을 옮기지 못한다.
- 돌려주는 $\delta t_c$ 와 풀이가 섰던 모든 시각은 정수 ns 다 (payload 와 RT 의 분해능). 결과에 $\delta t_c$, 포구 시각을 옮긴 횟수, 끝난 점의 $\partial L/\partial\theta$ 를 낸다.

**셀 안의 포구 시각 — 탐색 (`continuous_tc`).** 기본은 꺼짐이다. 켜면 푸는 후보마다 두 번 푼다.

1. ① 격자 시각에서, 위에 적은 그대로 (같은 코어, 같은 시작점).
2. ② ① 이 반복값을 냈으면, 포구 시각을 그 후보의 **셀** 안에서 풀어 주고 다시 (포구 시각을 변수로 둔, 같은 $n_{pre}$ 의 코어).

셀은 반열린 구간이고, 예측한 포구점이 포구 작업 영역 (box) 안에 남는 범위와 예측의 마지막 표본 시각 $t_{\mathrm{pred},\max}$ 로 더 자른다 (뒤의 것은 코어가 한다).

$$
\delta t_c\in\Big[-\Big\lfloor\frac h2\Big\rfloor,\ h-\Big\lfloor\frac h2\Big\rfloor\Big),\qquad
\hat p_b(\hat t)+\hat v_b(\hat t)\,\delta t_c\in\text{box},\qquad \hat t+\delta t_c\le t_{\mathrm{pred},\max} .
$$

- 늘어나는 것은 포구 노드 앞 구간 하나라 노드 0 · 출발 상태 · 포구 전 노드는 ① 의 것이다. 후보의 index 와 격자, 다음 탐색의 기억에서의 자리가 그대로다.
- **시작점.** 앞선 탐색에서 그 후보가 남긴 연속 해가 있고 그 해가 끝난 포구 시각이 이번의 구간 안이면 그것에서 — 지난 노드를 버리고, 그 포구 시각에서 — 시작하고, 아니면 이번 탐색의 ① 의 해에서 $\delta t_c=0$ 으로 시작한다. 연속 해의 기억은 고정 격자 해의 기억과 따로 있고, 똑같이 후보의 index 마다 하나다 (포구 시각은 격자 위에 없어도 셀은 격자의 것이다).
- **시각의 항.** 코어에 $c_1=w_T/T_{\mathrm{ref}}$, $c_2=w_{\mathrm{sw}}/T_{\mathrm{ref}}^2$, $\delta_{\mathrm{ref}}=t_{c,\mathrm{prev}}-\hat t$ 를 준다 ($t_{c,\mathrm{prev}}$ 가 없으면 $c_2=0$). 그래서 ② 는 $J^\star$ + 정지 구간의 항 + $J_{\mathrm{time}}+J_{\mathrm{switch}}$ 를 궤적과 포구 시각에 대해 같이 최소화한다.
- **공분산.** ② 가 쓰는 공분산은 셀의 격자 시각의 것 — 필요조건이 본 것 — 이다. plan 에 싣는 $\sqrt{\lambda_{\max}(\Sigma_p)}$ 는 포구 시각 자신의 것이다.
- **쓰는 해.** ② 가 ① 과 같은 규칙으로 유효하고 (자기 몫 안 · hard 행 전부 · 수렴) 끝난 포구 시각에서 예측으로 읽은 포구점이 box 안이면 ② 가 그 후보의 해다. 아니면 후보는 ① 의 해와 ① 의 판정을 그대로 갖고, 기록에는 ② 가 돌았다는 것 · 쓰지 않은 사유 · 끝난 점이 남는다.
- **$\Phi$.** ② 를 쓰는 후보의 $\Phi$ 는 끝난 포구 시각 $\hat t+\delta t_c^\star$ 에서 ① 과 같은 함수로 계산한다 — $T=\hat t+\delta t_c^\star-t_0$ 이고 $J_{\mathrm{switch}}$ 도 $\Phi$ 의 항이다. 유효한 ② 는 $\Phi$ 가 ① 보다 커도 쓴다: $\Phi$ 에는 코어가 같이 최소화한 정지 구간의 항이 없다. 두 값을 다 기록한다.
- **예산.** 후보 하나가 몫 둘을 쓴다 (풀이마다 자기 몫). $n_{\mathrm{pass}}$ 를 필요조건을 지난 후보의 수라 하면

$$
L=\min\Big\{L_{\max},\ n_{\mathrm{pass}},\ \Big\lfloor\frac{T_{\mathrm{budget}}-t_{\mathrm{screen}}}{2\,T_{\mathrm{solve}}}\Big\rfloor\Big\}
$$

  이고, $2\,T_{\mathrm{solve}}\le T_{\mathrm{budget}}$ 와 $\lfloor h/2\rfloor\lt\Delta_a$ (셀의 이른 끝에서도 포구 노드 앞 구간의 길이가 양이다) 가 아니면 구성을 거부한다.
- **내는 것.** plan 과 구간의 $t_c$ 는 $\hat t+\delta t_c^\star$ 다. plan 의 `p_c` · `v_c` · `a_d` · `sigma_c` · `t_cmd_ns` 는 그 시각의 값이고 `q_star` 는 해의 포구 노드다.
- **구간 payload.** 포구 노드 앞 구간의 길이를 따로 싣는다 — `dt_catch_ns` $=\Delta_a+\delta t_c^\star$ 이고 $\delta t_c^\star=0$ 이면 0 이다. 0 이 "`dt_pre_ns` 와 같다" 의 유일한 표기라서 `dt_catch_ns` 가 `dt_pre_ns` 와 같은 payload 는 검증이 거부한다. `dt_catch_ns` 가 0 이 아닌 구간의 노드 시각은 $k\lt n_{pre}$ 에서 $t_s+k\,\Delta_a$, 포구 노드에서 $t_c$, 그 뒤에서 $t_c+(k-n_{pre})\Delta_s$ 이고 노드 0 은 $t_c-(n_{pre}-1)\Delta_a-\tau$ 다 ($\tau$ 가 `dt_catch_ns`). RT 샘플러는 그런 구간을 세 부분으로 평가한다 — 노드 $n_{pre}-1$ 까지 $\Delta_a$, 거기서 포구 노드까지 $\tau$, 그 뒤 $\Delta_s$. `MpcSegmentPlanner` 가 내는 구간은 늘 0 이다.

RT 가 따르는 plan 의 셀은 이 두 번 풀이의 예외다 (17.12).

구현하지 않은 것: 단일 NLP 형태 (§11.5), 공분산 · 근방 가중 · 접근 집합을 $\delta t_c$ 의 함수로 두는 것, 코어 안에서 $\delta t_c$ 에 commit 하한을 두는 것 (탐색을 부르는 주기의 일이다), 두 스위치의 YAML 키와 값, (V4) reachability margin — 그 재료인 $\Sigma_b^{\mathrm{ant}}$ 를 쓰지 않는다 (17.3), hand schedule 의 (S2) · (V5), 병렬 worker, pipeline 실행 (P2), 여러 IK branch.

**검증.** `rtc_controllers/test/test_catching_nlp_catch_screening.cpp` 가 격자와 (S3) · (S4) 의 식을 스칼라로 다시 써서 경계 양쪽에서 본다. `test_catching_nlp_catch_search.cpp` 는 탐색을 본다: IK 의 판정을 IK 를 직접 부른 것과, 고른 후보와 $\Phi$ 를 테스트가 코어를 직접 불러 창 안의 후보를 전부 푼 것의 최소와 (정지 출발 · 움직이는 출발), 풀이 순서를 바꾼 결과를 서로 bit 단위로 비교하고, 사유마다 그것을 내는 입력을 둔다. `integrated_bringup/test/test_catching_nlp_search_shipped.cpp` 는 출하 sub-model 에서 구성 · 필요조건 · 풀이 · 탐색 한 번의 시간을 기록한다 (판정이 아니다).

셀 안의 포구 시각은 다음이 본다. `test_catching_mpc_docking_segment_core.cpp`: $\delta t_c$ 의 열과 기울기를 중심 차분과, 0 에 닫은 구간의 풀이를 고정 격자의 풀이와 비교하고, 포구 시각이 움직여도 목적함수가 오르지 않는 것, 보고한 $\partial L/\partial\theta$ 가 최적값의 기울기인 것, 구간과 step 상한, 반복 1 회가 포구 시각을 옮기지 않는 것, 구간이 예측의 끝에서 끝나는 것을 보고, 스위치를 끈 코어의 결과를 고정해 둔 digest 와 bit 단위로 비교한다. `test_catching_mpc_docking_relative_state.cpp` 는 공에 대한 미분을 중심 차분과 비교하고, `test_catching_nlp_catch_screening.cpp` 는 셀이 반열린 구간인 것을 본다. `test_catching_node_follower.cpp` 는 `dt_catch_ns` 가 있는 payload 의 노드 시각 · 샘플러 (노드 값과 그 사이의 적분) · 거부 사례를 보고, 그 필드가 0 인 payload 가 전과 bit 단위로 같게 읽히는 것을 본다. `test_catching_nlp_catch_search.cpp` 는 두 스위치를 끄고 RT 가 plan 을 따르지 않는 탐색의 결과와 창을 끈 채택 뒤의 필요조건을 고정해 둔 digest 와 bit 단위로 비교하고, ② 를 쓰는 후보의 목적함수가 ① 보다 크지 않은 것, 수렴하지 않은 ② 가 ① 로 돌아가는 것, plan 과 구간이 끝난 포구 시각의 것인 것, 포구점이 box 안에 남는 것, 따르는 plan 의 셀 (17.12) 과 창 (17.12) 의 규칙, 구성의 거부를 본다. `test_catching_nlp_search_shipped.cpp` 는 출하 sub-model 에서 연속 풀이를 고정 풀이 옆에 기록한다 ($n_{pre}$ 별 시간 · 반복 수 · 포구 시각을 옮긴 횟수, 후보 1 · 2 · 4 개의 탐색 한 번).

### 17.12 실행 구조 — 예측이 갱신될 때의 탐색

§12.7 가운데 바깥 루프가 하는 일의 구현이다. 실행 구조의 나머지 (§12.1 – §12.6, P1 · P2 의 실행 방식, closure 지령 시각의 갱신) 는 탐색에 없다. `NlpCatchSearch` 를 부르는 것은 테스트뿐이고, 계획기의 한 주기는 그것을 아직 부르지 않는다.

**예측 하나마다.** 탐색 한 번이 §12.7 의 한 주기다: 필요조건을 전부 다시 보고, 순위에 든 후보를 17.11 의 시작점에서 다시 푼다. 풀이는 자기 몫 안에서 수렴할 때까지 돈다 — real-time iteration 이 아니다.

**출발 상태 $x_0$ (§12.7).** 측정값이 아니라 팔이 따르고 있는 기준이다. 후보마다 그 격자의 노드 0 시각 $t_s$ 에서 정한다.

- RT 가 plan 을 따르고 있지 않으면 $x_0=(q_{\mathrm{cmd}},0,0)$ — 지금의 관절 명령, 정지. 명령이 움직이고 있으면 ($\max_j\vert\dot q_{\mathrm{cmd},j}\vert$ 가 허용값을 넘으면) 탐색은 plan 을 내지 않는다.
- 따르고 있으면 RT 가 보고한 구간을 $t_s$ 에서 평가한 $(q,\dot q,\ddot q)$ 다. 구간은 둘일 수 있다 — RT 가 받아 두고 아직 노드 0 에 닿지 않은 것 (대기) 과 지금 명령을 뽑고 있는 것 (추종). 대기 구간의 노드 0 이 $t_s$ 이전이면 그것에서, 아니면 추종 구간에서 평가한다. 평가는 RT 가 구간에서 명령을 뽑는 것과 같은 식이다 (노드 사이는 jerk 일정). 보고된 구간이 없으면 탐색은 plan 을 내지 않고, 대기 구간만 있는데 그 노드 0 이 $t_s$ 뒤인 후보는 버린다.
- 평가한 $q$ · $\dot q$ 가 코어의 box 밖이면 box 안으로 사영하고 그 사실을 표시한다 (코어는 box 밖의 출발을 거부한다). (S4) 는 사영한 값을 쓴다.
- 추종 오차가 커지면 측정값으로 바꾸는 단서는 없다. 대신 RT 의 명령과 추종 구간의 차이 (위치 · 속도의 관절별 최대) 를 탐색마다 기록한다.

**따르는 후보의 재풀이.** 예측이 그대로이고 RT 가 직전 해를 따르고 있으면, 그 후보의 출발 상태는 직전 해 위에 있고 시작점은 그 해의 남은 노드다 (17.11). 코어는 그 점에서 수렴 조건을 이미 만족해 같은 궤적을 낸다.

**이 track 의 plan 과 그 셀.** RT 가 따르는 plan 은, RT 가 보고한 구간이 지금 탐색하는 예측과 같은 track 세대를 실을 때 "이 track 의 plan" 이다 (구간은 자기 plan 의 track 을 싣는다). 그 plan 의 **셀**은 포구 시각이 든 반열린 격자 셀이다 — 포구 시각이 격자 위에 없어도 후보 하나를 가리킨다.

$$
i=\Big\lfloor\frac{t_c-t_{\mathrm{ref}}+\lfloor h/2\rfloor}{h}\Big\rfloor
\quad\Longleftrightarrow\quad
t_c^{(i)}-\Big\lfloor\frac h2\Big\rfloor\le t_c\lt t_c^{(i)}-\Big\lfloor\frac h2\Big\rfloor+h .
$$

RT 가 이 track 의 plan 을 따르는 동안만 아래가 적용된다. 앞의 셋은 `continuous_tc` 와 무관하다.

- **먼저 푼다.** 그 셀의 후보가 필요조건을 지나면 순위의 맨 앞에 둔다 (17.11 의 순서와 무관하게, 늘). 예산이 풀이 하나만 담는 탐색도 팔이 하고 있는 것을 다시 풀고 "갱신" 을 낼 수 있다.
- **창 (`follow_window`).** 이 track 에서 RT 가 **처음** 따른 plan 의 셀 $i_a$ 를 기억한다. 창 $W\ge0$ (셀의 수) 이 있으면 $\vert i-i_a\vert\gt W$ 인 후보를 다른 어떤 검사보다 먼저 뺀다 (사유 "창 밖" — 공도 IK 도 보지 않는다). 따르는 plan 자신의 셀은 빼지 않는다. 그래서 plan 이 몇 번 교체되든 포구 시각은 접근을 시작한 셀에서 창보다 멀리 가지 못한다. $i_a$ 는 시도를 다시 시작할 때 (`ResetTrial`), track 이 바뀔 때, RT 가 이 track 의 plan 을 따르지 않는 탐색이 있을 때 지운다. 음수는 창 없음이고 기본이 그것이다.
- **기록.** 창이 있든 없든 탐색마다 $i_a$, 창이 뺀 후보의 수, 고른 후보의 $i-i_a$, 고른 $t_c$ 와 처음 따른 plan 의 $t_c$ 의 차 (ns), 고른 후보가 창의 끝 ($\vert i-i_a\vert=W$) 인지를 남긴다.
- **`continuous_tc` 에서 따르는 plan 의 셀.** 17.11 의 두 번 풀이의 예외다. 그 후보의 포구 시각은 격자 시각도 자유 변수도 아닌 **plan 의 것**이다: 필요조건을 그 시각에서 보고, 포구 시각을 변수로 둔 코어로 $\delta t_c$ 를 그 값에 묶은 채 한 번 푼다. 그것을 고르면 "갱신" 이다. 같은 셀의 다른 포구 시각은 후보로 존재하지 않으므로 plan 이 자기 셀 안의 다른 시각으로 교체되는 일은 없다.

**멈추는 때.** 탐색 자신은 멈추는 규칙을 갖지 않는다. $T\lt T_{\min}$ 이 된 후보가 차례로 빠질 뿐이고, 탐색을 언제까지 부를지는 부르는 쪽이 정한다.

구현하지 않은 것: $t\ge t_c^\star-\tau_{\mathrm{react}}$ 에서 바깥 루프를 멈추는 것, real-time iteration 과 preparation / feedback 의 분리, P1 · P2, closure 지령 시각의 갱신, 채택 뒤에 따르는 셀 안에서 포구 시각을 옮기는 것, commit 에 따라 포구 시각을 묶는 것, 계획기의 한 주기가 격자 밖 포구 시각의 plan 을 게시하는 것.

### 17.15 검증

§15 의 항목 가운데 이 코어의 테스트가 보는 것은 다음과 같다 (`rtc_controllers/test/test_catching_mpc_docking_*.cpp`, 공 표본은 `test_catching_ball_node_samples.cpp`).

- 1 · 2 (frame 과 부호, transport 항): $r^H$ · $\nu^H$ 와 17.6 – 17.8 의 행 전부의 기울기를 중심 차분과 비교한다. 손이 회전하고 공이 축에서 벗어난 상태에서 본다. `LOCAL` 경로가 같은 $\nu^H$ 를 내는지도 본다.
- 5 (crossing-plane 분포): §15 의 예 ($\hat\nu=[0.3,-0.1,-2.0]$ m/s, $\sigma=[4,4,20]$ mm, 2×10⁵ 표본) 로 $\Sigma_\rho$ 를 Monte Carlo 와 비교한다. 고정 시각 식은 같은 허용 오차에서 벗어나야 한다. 이 검사는 식의 대수를 본다 — 1 차 근사의 크기는 포물선 공과 가속 · 회전하는 손의 실제 통과점으로 따로 잰다.
- 8 (impact): $\beta_h$ 를 RNEA 로 만든 관성행렬 · 차분으로 만든 접촉 Jacobian 과 비교하고, armature 를 더하면 $\beta_h$ 가 줄어드는 것을 본다.
- §8.2 의 sanity check 셋 ($\Sigma=0$, $\hat\nu\parallel e_3$, 정지한 손과 축을 따라 오는 공).
- 조립: QP 의 기울기와 행을 비선형 문제의 중심 차분과 비교한다.
- 풀이: 실행 가능한 것이 알려진 합성 투척이 수렴하고, hard 행을 코어 밖에서 다시 계산해 본다. 실행 불가능한 투척은 그렇게 보고된다.
- 출하 모델: `integrated_bringup/test/test_catching_docking_core_shipped.cpp` 가 손을 잠근 출하 sub-model 과 출하 catch frame · 관절 정격으로 짧은 lead 의 격자와 실제 투척 조건을 푼다. 판정이 아니라 기록이다 — 포획 집합의 값이 합성값이라, 단언하는 것은 모델이 출하된 것이라는 점과 풀이마다 결과가 나온다는 점뿐이다.

나머지 (3 · 4 · 6 · 7 · 9 – 15) 는 이 코어의 범위가 아니다.
