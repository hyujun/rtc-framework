# L4 — Reference: soft-catch DS, 접근축 정렬, 복귀

이 문서는 현재 구현의 **기준 생성 층** 을 표현한다 — soft-catch DS (catch frame 의 병진 기준) 와 접근축 정렬 (회전 오차) 의 수학이다.

- **이 층의 soft-catch DS 는 `closed_form` planner 의 RT 기준 생성이다. `mpc` (출하 값) 에서는 탐색의 rollout (L3 §4.8) 에서만 쓰인다** — `mpc` 의 RT 기준은 MPC 구간이고 (formulation 과 L7), RT 는 이 DS 를 돌리지 않는다. planner 의 구분은 L3 §4.1
- **접근축 정렬 (§4.5) 은 두 planner 가 다 쓴다** — 탐색의 포구 자세 IK, RT 의 CLIK 5 행 과제, 그리고 `mpc` 의 MPC 코어가 같은 오차 정의를 쓴다
- 코드: 병진 기준·γ 프로파일은 `rtc_controllers/include/rtc_controllers/catching/soft_catch.hpp` (namespace `rtc::catching`), 축 정렬 오차·Jacobian 은 `rtc_math/include/rtc_math/se3/axis_align.hpp` (namespace `rtc::math::se3`)
- L3가 γ rollout에서 **이 layer의 같은 코드**를 호출한다

---

## 1. 범위 / 비범위

범위: 추종 대상(실제 공 또는 L7의 가상 감속 목표)의 상태로부터 catch frame의 병진 기준 $(x,\dot x,\ddot x)$ 를 제어 주기 $h$ = `ControllerState::dt` (= 1/`control_rate`, 100–5000 Hz) 마다 생성한다. 500 Hz 고정을 가정하지 않는다. 그리고 접근축 정렬 오차 $e_a$ 와 그 Jacobian 을 정의한다 — 각속도 기준 $K_ae_a$ 는 CLIK 이 과제 행 안에서 만든다 (L5).

비범위: 관절 공간 변환과 한계 처리(L5), γ·포구점 결정(L3), 모드 전환 판단(L7), `mpc` 의 구간 기준 (formulation), 복귀 (관절공간 — §5.3).

## 2. 코드 확인 게이트

이 층이 기대는 기존 구조는 다음과 같다.

| ID | 확인 항목 | 지금의 구조 |
|---|---|---|
| G4-1 | `rtc_tsid` / CLIK가 받는 과제 기준의 형식 | `rtc::tsid::ClikReferenceGenerator` 는 pose(SE3) 목표 외에 **위치 3 행 + LOCAL 접근축 2 행** 과제 (`PositionAxisTarget`: 위치 · 축 · 선속도 / 각속도 feedforward) 를 받는다 `[확정 D-5]`. catching 은 이 과제를 쓴다 (L5 §4.2) |
| G4-2 | 기존 SE(3)/SO(3) 오차 헬퍼(U1 공유 헬퍼)의 규약과 본 문서 §4.5 축 정렬 오차의 공존 방식 | U1 헬퍼는 `rtc_tsid` se3_error (`ComputeTaskPoseError`, LWA BodyLog6). §4.5 축 정렬 오차는 그것과 **별도 함수**로 `rtc_math` se3 에 있다 (D-1) |
| G4-3 | 두 로봇의 catch frame과 손바닥 바깥 법선 축 | D-17: 모델 빌더가 YAML 선언 frame 으로 추가 (D-10), 부모·offset·자세는 로봇 config, 접근축 = 그 frame 의 +z (L5 §11) |

## 3. 참고자료

[R3] soft catch 이론(오차 좌표 LPV DS, softness) — **본 layer의 1차 근거, 원문 대조 완료(§4.1)**. [R4] 같은 구조의 다중 팔 확장 — **전문 미보유, 식 번호 인용 금지**. [R5] 공개 코드(§4.6, **미열람**). [R7] SO(3).

## 4. 수학적 이론

### 4.1 오차 좌표 soft-catch DS ([R3], [R4])

**적용: `closed_form` 의 RT 기준, 그리고 두 planner 공통의 탐색 rollout (L3 §4.8).**

좌표 원점을 예측 포구점 $p_c$에 둔다. 대상 상대위치 $\xi^O=p_O-p_c$, 기준 상대위치 $\xi=x-p_c$. 오차

$$e=\xi-\gamma\xi^O,\qquad \dot e=\dot\xi-(\gamma\dot\xi^O+\dot\gamma\xi^O)$$

가 다음을 따르도록 기준 가속도 $u=\ddot x$를 정한다.

$$\ddot e=A_1e+A_2\dot e$$

$\ddot\xi=u$에서 $\ddot e=u-(\gamma\ddot\xi^O+2\dot\gamma\dot\xi^O+\ddot\gamma\xi^O)$이므로

$$u=\gamma\ddot\xi^O+2\dot\gamma\dot\xi^O+\ddot\gamma\xi^O+A_1e+A_2\dot e$$

본 구현은 $A_1=-\omega^2I$, $A_2=-2\zeta\omega I$ (LTI) 다. [R3]의 GMM 기반 LPV $A_i(\theta)$는 구현에 없다.

**[R3] 원문 대조.** 결론은 **동치**이나 plant 규약이 달라 겉보기 식이 다르므로 주의한다.

[R3] 식 (1)의 plant는 순수 이중적분기가 아니다.

$$\ddot\xi=A_1(\theta_{A_1})\xi+A_2(\theta_{A_2})\dot\xi+u$$

그래서 [R3] 식 (4)의 제어입력에는 대상 항을 상쇄하는 성분이 붙는다.

$$u_{[R3]}=\gamma\ddot\xi^O-A_1\gamma\xi^O-A_2(\gamma\dot\xi^O+\dot\gamma\xi^O)+2\dot\gamma\dot\xi^O+\ddot\gamma\xi^O$$

여기에 plant 항을 더하면

$$\ddot\xi=A_1\xi+A_2\dot\xi+u_{[R3]}=A_1e+A_2\dot e+\gamma\ddot\xi^O+2\dot\gamma\dot\xi^O+\ddot\gamma\xi^O$$

로, 본 문서의 $u$ 와 **총 가속도가 정확히 같다**. 본 문서는 $\ddot\xi=u$ 규약이므로 $u_{doc}=\ddot\xi\neq u_{[R3]}$ 다. 오차 좌표 정의도 [R3] 식 (5)와 문자 그대로 일치한다. [R3] 식 (4)를 그대로 코드로 옮기면 plant 항을 두 번 빼게 된다.

[R3] 식 (11)의 수치 예는 $\ddot\xi=-12\dot\xi-36\xi+u$ 로 $\omega=6$, $\zeta=1$ 이다. 출하 `reference.omega` 는 그보다 크지만 §4.7의 이산 안정 경계 안에 있다.

- $\ddot\xi^O$는 **vision이 준 $a$** 를 L2 샘플러가 보간한 값을 쓴다(L2 §4.2). 수치미분 금지. 제어 PC는 궤적을 재전파하지 않는다(마스터 §5.2). 5차 Hermite 보간이 $C^2$ 라 $\ddot\xi^O$ 가 샘플 경계에서 튀지 않는다 — 이 성질이 여기서 필요한 이유다.
- $\gamma(t)$는 5차 보간 $\gamma_0\to\gamma_f$ ($[t_0,t_1]$), 양 끝의 1·2계 미분이 0이다 (`GammaProfile`).

### 4.2 Corollary: 원점 = 포구점의 의미 ([R3], [R4])

$e(t_c)=0$, $\dot e(t_c)=0$이고 $\xi^O(t_c)=0$이면

$$\xi(t_c)=0=\xi^O(t_c),\qquad \dot\xi(t_c)=\gamma\dot\xi^O(t_c)$$

위치는 γ와 무관하게 일치하고, 기준 속도는 대상 속도와 같은 방향에 크기가 γ배다. 상대속도는 $(1-\gamma)\dot\xi^O(t_c)$다.

극한 점검: $\gamma=0$이면 고정점 수렴(정지 포구), $\gamma=1$이면 완전 추종.

### 4.3 예측 오차와 재예측에 대한 민감도 `[논문 외 유도]`

**예측 오차.** 실제 대상이 $t_c$에 $\xi^O(t_c)=\delta\neq0$에 있고 $e\approx0$이면, 기준은 $\gamma\delta$에 있으므로 간극은 $(1-\gamma)\Vert\delta\Vert$다.

**재예측 점프** (`closed_form` 의 plan 교체 — L3 §4.7). L3가 포구점을 $p_c'=p_c+\Delta p_c$로 바꾸면 $\xi^{O\prime}=\xi^O-\Delta p_c$, $\xi'=\xi-\Delta p_c$이므로

$$e'=e-(1-\gamma)\Delta p_c,\qquad \dot e'=\dot e+\dot\gamma\,\Delta p_c$$

기준 상태 $(x,\dot x)$는 연속이고 오차만 점프한다. 위 식은 γ 프로파일이 그대로일 때다. 런타임 교체는 γ 를 이어 받되 램프를 새로 시작하므로 ($\dot\gamma'=\ddot\gamma'=0$) $u_{des}$ 의 계단은

$$\Delta u_{des}=\omega^2(1-\gamma)\Delta p_c-(2\zeta\omega\dot\gamma+\ddot\gamma)\,\xi^O-2\dot\gamma\,v_o$$

이다 ($\xi^O=o-p_c$, 옛 포구점 기준). L3 의 교체 규칙 (L3 §4.7 규칙 2) 이 이 계단의 상계를 $\eta_{jump}a_{\max}$ 로 제한한다.

### 4.4 수렴 한계 `[논문 외 유도]`

**임계감쇠 닫힌해** ($\zeta=1$, 축별 독립):

$$e(t)=\big(e_0+(\dot e_0+\omega e_0)t\big)e^{-\omega t},\qquad \dot e(t)=\big(\dot e_0-\omega t(\dot e_0+\omega e_0)\big)e^{-\omega t}$$

함수는 `CriticallyDampedError` 다. 포화가 없다는 가정 아래의 식이고, 탐색은 $t_c$ 의 잔여 오차와 포화 여부를 rollout 의 적분으로 잰다 (L3 §4.8).

**LPV 확장 시 충분조건.** $z=[e;\dot e]$, 꼭짓점 $\mathcal A_k=\begin{bmatrix}0&I\\A_{1k}&A_{2k}\end{bmatrix}$에 대해

$$\exists P\succ0,\ \alpha>0:\ P\mathcal A_k+\mathcal A_k^\top P\preceq-2\alpha P\ \ \forall k\ \Rightarrow\ \Vert z(t)\Vert\le\sqrt{\kappa(P)}\,e^{-\alpha t}\Vert z(0)\Vert$$

따라서 허용오차 $\epsilon$에 대해 $T\ge\frac1\alpha\ln\frac{\sqrt{\kappa(P)}\Vert z(0)\Vert}{\epsilon}$이면 충분하다.

**이 LMI가 필요한 범위.** [R3] Theorem 1은 점근 수렴만 주장하고 수렴 속도는 다루지 않는다(원문 확인). 다만 본 구현이 쓰는 LTI $A_1=-\omega^2I$, $A_2=-2\zeta\omega I$ 에서는 수렴률이 $\omega$ 로 자명하므로, 위 LMI가 실제로 필요한 것은 **LPV 확장 시뿐이다**.

**γ가 수렴에 주는 영향.** $e(0)=\xi(0)-\gamma(0)\xi^O(0)$이다. $\gamma(0)=0$으로 시작하면 초기 오차가 대상 거리와 무관해진다. γ는 feedforward에만 들어가므로 오차계 자체는 임의의 $C^2$ $\gamma(t)$에 대해 같다. 대신 입력 크기 $\Vert u\Vert$가 $\dot\gamma,\ddot\gamma,\Vert\xi^O\Vert$에 비례해 커진다. 이 때문에 γ 창과 램프 구간은 L3가 rollout으로 정한다.

### 4.5 접근축 정렬 `[논문 외 유도]`

**적용: 공통 — 두 planner 가 다 쓴다.** 이 절의 $e_a$ (`AxisAlignError`) 를 쓰는 곳은 셋이다.

| 쓰는 곳 | planner | 무엇 |
|---|---|---|
| 탐색의 포구 자세 IK (`CatchPoseIk`, L3 §4.2) | 공통 | catch frame LOCAL 표현의 $e_a^C$ — 5 행 과제의 회전 2 행 |
| RT 의 CLIK 5 행 과제 (`ClikReferenceGenerator::Compute(…, PositionAxisTarget, …)`, L5 §4.2) | 공통 | 회전 2 행의 기준 $S R_{WC}^\top(K_ae_a+\omega_{ff})$. 축 목표는 `closed_form` 에서 `PlanSnapshot` 의 $a_d$ ($\omega_{ff}=0$), `mpc` 에서 구간의 관절 노드를 FK 한 catch frame 의 $z$ 축 (각속도 feedforward 포함) |
| MPC 코어의 포구 노드 접근축 항 (`decel_mpc.cpp`) | `mpc` | $e_a$ 와 $J_a$ (`AxisAlignJacobian`) 로 선형화한 비용 — 식은 formulation |

손바닥 바깥 법선 $z=R_{WC}\hat e_z$ (W), 목표 $a_d=-\hat v_O(t_c)$. 둘 다 단위벡터다.

**오차는 회전벡터로 정의한다.** $m=z\times a_d$, $c=z^\top a_d$ 에 대해

$$\theta=\mathrm{atan2}(\Vert m\Vert,\,c),\qquad \hat u=\frac{m}{\Vert m\Vert},\qquad \boxed{e_a=\theta\,\hat u},\qquad \omega_{ref}=K_a\,e_a$$

$\exp([e_a]_\times)z=a_d$ 를 **정확히** 만족한다. 따라서 $\Vert e_a\Vert=\theta$ 이고 $\Vert\omega_{ref}\Vert=K_a\theta$ 는 정렬 오차에 대해 연속·단조다. 접근축 주위 회전(roll)에는 기준을 주지 않는다(5-DoF). $e_a\perp z$ 이므로 L5의 LOCAL 마스크 $S=\mathrm{diag}(1,1,0)$ 가 정보를 버리지 않는다.

**CLIK 에 들어가는 회전 기준.** $\omega_{ref}$ 는 되먹임 항이고, 5 행 과제의 회전 2 행에 들어가는 것은 거기에 feedforward 를 더한 것이다.

$$\boxed{\omega_{cmd}=K_a\,e_a+\omega_{ff}}$$

$\omega_{ff}$ 는 축 목표 $a_d$ 가 움직이는 각속도다 — `mpc` 에서는 구간이 주는 catch frame 의 각속도, `closed_form` 에서는 0 ($a_d$ 가 `PlanSnapshot` 의 고정값). 아래 수렴성은 $\omega_{ff}=0$ 인 경우 (고정된 $a_d$) 의 것이다.

수렴성: $\dot z=\omega\times z$ 이고 $\omega_{ref}=K_a\theta\hat u$ 이므로 $\frac{d}{dt}(z^\top a_d)=K_a\frac{\theta}{\sin\theta}\big(1-(z^\top a_d)^2\big)\ge0$. $z^\top a_d$ 가 단조 증가하므로 반평행 평형점을 제외하면 전역 수렴한다.

$\Vert m\Vert<\epsilon_{\sin}$ 이고 $c\le0$ 이면(반평행) 축이 정의되지 않으므로 $z$에 수직인 고정 축을 골라 $e_a=\pi\hat u_\perp$ 를 쓴다. 크기는 $\pi$ 로 연속이다.

**회전벡터이고 $z\times a_d$ 가 아닌 이유.** $e_a=z\times a_d$ 로 두면 크기가 $\sin\theta$ 라 단조가 아니다 — 90° 를 넘으면 오차가 클수록 기준이 **작아져** 170° 에서 $0.17K_a$, 177° 에서 $0.05K_a$ 로 사실상 수렴이 멈춘다 (대기 자세가 공 반대쪽을 보고 있으면 여기에 걸린다). 그것을 덮으려 반평행 임계에서 분기하면 그 임계에서 각속도 기준이 계단으로 튀어 CLIK 입력에 무한 jerk 가 들어간다. 회전벡터는 $\Vert\omega_{ref}\Vert=K_a\theta$ 라 1° 격자에서의 변화가 $K_a\cdot\Delta\theta$ 로 연속이고, 반평행 분기 대신 $\Vert m\Vert$ 의 데드밴드 $\epsilon_{\sin}$ 만 둔다.

**오차 Jacobian** (W 표현 각속도 $\omega=J_\omega^W\dot q$). $f(c)=\theta/\sin\theta$ 로 두면 $e_a=f(c)\,m$ 이고, $\dot c=m^\top\omega$, $\dot m=[a_d]_\times[z]_\times\omega$ 이므로

$$\dot e_a=J_a\,\omega,\qquad \boxed{J_a=f'(c)\,mm^\top+f(c)\,[a_d]_\times[z]_\times},\qquad f'(c)=\frac{\theta c/\sin\theta-1}{\sin^2\theta}$$

$\theta\to0$ 극한은 $f\to1$, $f'\to-1/3$ 로 유한하고, 이때 $J_a\to[a_d]_\times[z]_\times$ 다. $\theta\to\pi$ 에서는 $f\to\infty$ 로 발산한다 — 축이 정의되지 않기 때문이며, 이는 정의의 결함이 아니라 문제의 성질이다. CLIK 은 마스크 방식(L5 §4.2)이라 $J_a$ 가 필요 없고, $J_a$ 를 쓰는 것은 MPC 코어의 선형화다.

**구현 주의 두 가지.**

1. 소각도 급수는 $c>0$ 일 때만 쓴다. $\sin\theta$ 는 $\theta\to0$ 과 $\theta\to\pi$ **양쪽에서** 0이므로, 부호를 보지 않으면 반평행 근처에서 발산해야 할 $f$ 가 유한한 값(≈2.645)으로 조용히 바뀐다.
2. `AxisAlignJacobian` 의 `sin_eps` 는 `AxisAlignError` 와 **같은 값**이어야 한다. 다르면 데드밴드 안에서 $e_a$ 는 상수인데 $J_a$ 가 0이 아닌 값을 돌려주어, $J_a$ 가 더 이상 $e_a$ 의 야코비안이 아니게 된다.

**함수는 항상 유한값을 돌려준다.** 위 $\theta\to\pi$ 발산은 수학의 성질이지만, RT 경로 함수가 NaN·폭주값을 내보내면 CLIK 입력이 오염된다. `rtc_math/include/rtc_math/se3/axis_align.hpp` 의 세 함수 `AxisAlignError`·`AxisAlignJacobian`·`AxisAlignOmega` 는 결과와 함께 분기 (`AxisAlignRegion`) 를 돌려준다.

| 분기 | 조건 | $e_a$ | $J_a$ |
|---|---|---|---|
| 정렬 데드밴드 | $\Vert m\Vert<\epsilon_{\sin}$, $c>0$ | 0 | 0 |
| 반평행 데드밴드 | $\Vert m\Vert<\epsilon_{\sin}$, $c\le0$ | $\pi\hat u_\perp$ ($z$ 로 정해지는 고정 축) | 0 |
| Jacobian 상한 | $c<0$, $\epsilon_{\sin}\le\Vert m\Vert<n_J$ | 정확값 | $f,f'$ 를 $\Vert m\Vert=n_J$ 에서 평가 — $\Vert J_a\Vert\lesssim\pi/n_J$, $n_J$ 에서 연속 |
| 무효 입력 | 비유한·비단위($\vert\Vert v\Vert-1\vert>10^{-6}$)·$[10^{-12},1)$ 밖의 $\epsilon_{\sin}$, $n_J$ | 0 | 0 |

$\epsilon_{\sin}$ 과 $n_J$ 는 YAML 키가 아니라 헤더의 상수다 (`kAxisAlignSinEps` $=10^{-6}$, `kAxisAlignJacobianSinFloor` $=10^{-3}$ — 함수 인자의 기본값). 상한은 약 179.94° 이상에서만 걸리므로 G4-D 의 유한차분 범위(1–170°)에 영향이 없다. 두 데드밴드 모두 $e_a$ 가 상수라 $J_a=0$ 을 돌려준다. 소각도 급수는 $c>0$, $\theta<10^{-3}$ 에서만 쓴다 ($f=1+\theta^2/6$, $f'=-1/3-2\theta^2/15$).

`AxisAlignOmega(error, k_axis, w_max)` 는 $K_ae_a$ 를 $\Vert\omega\Vert\le w_{max}$ 로 줄이고 `saturated` 를 올린다 (비유한 오차·잘못된 게인은 0 과 무효). **이 함수를 부르는 제어 코드는 없다** — CLIK 은 `AxisAlignError` 의 $e_a$ 에 이득 `joint_cmd.K_a` 를 곱해 과제 행의 기준으로 직접 쓰고, 각속도 기준에 노름 상한을 두지 않는다 (관절 속도는 CLIK 의 box 가 묶는다 — L5).

### 4.6 공개 코드 [R5]와의 차이 (이식 금지 목록)

**방침: [R5] 공개 코드는 논문 식과 다를 수 있으므로 이식하지 않고 논문 식으로 직접 구현한다.**

[R5] `bimanual_ds.cpp`의 `Update()`가 [R4]의 식과 다르다고 기록된 항목은 셋이다. **[R4] 전문과 [R5] 코드를 지금 보유하고 있지 않아 셋 다 재확인되지 않았다 (`TBD-REF-01`) — 구체적 주장으로 인용하지 않는다.**

1. γ 갱신: 논문은 노름, 코드는 제곱노름에 이득 0.1, 적분 계수 0.2.
2. feedforward: $\dot\gamma\dot\xi$ 한 개와 $\ddot\gamma\xi$ 누락, $x_\gamma^O$ 속도 성분의 $\dot\gamma\xi$ 누락.
3. 결합항: 논문의 $U_j=\dot x_R+A_R(x_{V,j}-x_R)$ 에서 코드는 $\dot x_R$ 항이 없다. 등속 대상에 대해 정상상태 추종 오차가 남는다.

`TBD-REF-01` 을 닫는 길은 둘이다: [R4] 전문과 [R5] 코드를 확보해 세 항목을 재확인하고 1차원 재현 스크립트를 남기거나, 확보하지 못하면 항목별 주장을 지우고 위 방침만 남긴다. 어느 쪽이든 방침은 바뀌지 않는다.

### 4.7 이산화

기준 상태는 반암시적 오일러로 적분한다. 오차계를 같은 방식으로 이산화하면 $s=\omega h$에 대해

$$M=\begin{bmatrix}1-s^2&1-2s\\-s^2&1-2s\end{bmatrix}\quad(\text{상태 }[e;\,h\dot e])$$

이고, Jury 조건에서 안정 조건은 $0<s<2\sqrt2-2\approx0.828$이다($s=0.83$에서 스펙트럼 반경 1.005). 정확도를 위해 $s\le0.05$를 권장한다. $h=2$ ms면 $\omega\le25$ rad/s다.

**$h$ 는 실제 제어 주기다.** $h$ = `ControllerState::dt` = 1/`control_rate` 이고 `control_rate` 는 100–5000 Hz 범위다. 500 Hz 에서 맞는 $\omega$ 가 100 Hz ($h=10$ ms) 에서는 $s=\omega h$ 가 5배가 되어 안정 경계를 넘을 수 있다 ($\omega=10$ → $s=0.1$, 권장치 초과). 파라미터 검증기 (`ValidateCatchingParams`) 가 configure 시 실제 $h$ 로 $s$ 를 검사한다: 경계 이상 → `armable=false`, 권장치 초과 → 경고 (L0 §5.3). 위 $M$ 은 $\zeta=1$ 의 식이다.

**$\zeta\neq1$ 은 검증기가 막는다.** §4.4 닫힌해와 위 이산 안정 경계가 $\zeta=1$ 의 식이므로, `reference.zeta` $\neq1$ 이면 검증기가 `armable=false` 로 막는다 (L0 §5.3). 일반 $\zeta$ 를 허용하려면 닫힌해와 이산 안정 경계를 $\zeta$ 에 대해 다시 유도해야 한다.

### 4.8 포화

$\Vert u\Vert\le a_{\max}$, $\Vert\dot x\Vert\le v_{\max}$로 포화시키고 (방사형 축소) 플래그를 올린다. 포화 중에는 §4.1의 오차계가 성립하지 않는다.

soft catch에서는 포화가 포구 직전에 몰려 치명적이다. 수치 예($t_c=0.8$ s, 예측 오차 3 cm)에서 간극은 다음과 같다.

| 조건 | 간극 |
|---|---|
| 포화 없음, $\gamma_f=0.4$ | 17.7 mm |
| $a_{\max}=15$, $\gamma_f=0.4$ | 97.4 mm |
| $a_{\max}=15$, $\gamma_f=0$ | 29.7 mm |

따라서 L3가 rollout으로 포화 없는 $(\gamma_f,T_w)$를 선택하고(L3 §4.8), 계획 γ 창이 $\eta_v$`reference.v_max` 로 여유를 남긴다 (D-9). `closed_form` 의 실행 중 기준이 `supervisor.sat_ticks` tick 연속으로 포화하면 (`REF_SATURATED`) `COMMITTED` 이전은 RETREAT, 이후는 ABORT_SAFE 다 `[확정 D-8]`. 실행 중 γ 를 낮추는 경로는 없다 (§5.2.1). `mpc` 의 RT 는 이 기준을 돌리지 않으므로 이 포화가 없다.

### 4.9 Sanity check

1. $\gamma_f=0$, 예측 정확: 간극 ≈ 0, 상대속도 ≈ $\Vert v_O\Vert$.
2. 예측 오차 $\delta$: 간극 ≈ $(1-\gamma)\Vert\delta\Vert$.
3. 상대속도 ≈ $(1-\gamma)\Vert v_O(t_c)\Vert$.
4. 닫힌해와 미세 스텝 적분 일치.
5. 축 정렬: $\Vert\omega_{ref}\Vert$ 가 정렬 오차에 대해 연속, $\exp([e_a]_\times)z=a_d$, Jacobian이 유한차분과 일치.
6. (회귀) 속도 포화 시 반환 `xdd`가 실현 가속도와 일치(§5.2).

기준 표 — G4-A 가 재현한다 (`test_catching_soft_catch`; $\omega=10$, $h=2$ ms, 포화 없음, $T_w=0.4$ s, $t_c=0.8$ s):

| 조건 | 간극 | 예측 $(1-\gamma)\delta$ | 상대속도 | 예측 $(1-\gamma)\Vert v_O\Vert$ |
|---|---|---|---|---|
| $\delta=0,\gamma_f=0$ | 1.1 mm | 0 | 5.662 | 5.671 |
| $\delta=0,\gamma_f=0.4$ | 1.1 mm | 0 | 3.395 | 3.402 |
| $\delta=3$ cm, $\gamma_f=0$ | 29.8 mm | 30.0 | 5.662 | 5.671 |
| $\delta=3$ cm, $\gamma_f=0.4$ | 17.7 mm | 18.0 | 3.395 | 3.402 |

$\delta=0$ 행의 1.1 mm는 오차가 아니라 **DS 자체의 잔여 수렴 오차** $\epsilon_{conv}$ 다($\omega=10$, 비행 0.8 s). 게이트는 이 값을 허용해야 한다 — G4-A의 허용치 $\epsilon_{conv}=2$ mm가 이 1.1 mm를 덮는다(§9 G4-A). 참값 공 궤적은 RK4 로 만든다 — 기준과 같은 semi-implicit Euler 로 만들면 검증 대상인 간극과 같은 자릿수의 계통 오차 ($\tfrac12gTh$) 가 섞인다.

## 5. C++ 구현

### 5.1 `soft_catch_reference.hpp` (S1.4·S2.1 이식 완료)

정의는 `rtc_controllers/include/rtc_controllers/catching/soft_catch.hpp` (병진 기준·γ 프로파일) 와 `rtc_math/include/rtc_math/se3/axis_align.hpp` (축 정렬) 다 — 문서에 코드를 복제하지 않는다.

- `soft_catch.hpp`: `TargetState`, `GammaProfile` (5차 램프), `TranslationOutput` (`x`·`xd`·`xdd`·`u_des`·`e`·`ed`·γ 3종·`saturated`·`valid`), `SoftCatchTranslation` (`Reset`·`SetIntercept`·`Step`·`Evaluate`), `CriticallyDampedError` (§4.4), 시각 변환 `ProfileSeconds`·`MakeGammaProfile`
- `axis_align.hpp`: `AxisAlignError`·`AxisAlignJacobian`·`AxisAlignOmega` 와 `AxisAlignRegion` (§4.5)

헤더에 걸린 규범:

- **NaN 가드.** 비유한 입력 (목표·`t`·`dt`) 이나 비유한으로 넘친 명령은 **내부 상태를 보존**하고 출력을 invalid (`valid=false`, `saturated=true`) 로 돌려준다 — 비유한 목표 하나가 `x`·`ẋ` 를 영구히 오염시키지 않는다. 다음 유한 입력은 보존된 상태에서 이어 간다. 포화 검출은 NaN 에서도 참이 되도록 부정 비교 (`!(un <= a_max)`) 로 쓴다 (G4-I)
- **dt 검증.** `dt` 는 `ControllerState::dt` (100–5000 Hz) 이고, $\omega h$ 검사는 configure 의 검증기가 한다 (§4.7). 비유한·비양수 `dt` 는 invalid 다. 적분 없이 $e$·$\dot e$·$u_{des}$ 를 읽는 것은 `Evaluate(o, t)` 다
- **γ derate 는 없다** (D-8, §5.2.1)
- **축 정렬 함수의 유한성.** 데드밴드·반평행에서 NaN·폭주 금지 (§4.5)
- RT 경로 코드이므로 할당 0·`noexcept` (G4-G)

### 5.2 사용 규약

**적용: `closed_form` 의 RT 와 탐색의 rollout.** `mpc` 의 RT 는 `SoftCatchTranslation` 을 돌리지 않는다.

- **시간축 (L0 §4.5, D-2).** γ 프로파일 평가와 대상 샘플링은 **선행 시각 $now_{lead}=now+T_{arm}$** 축이다 (팔 명령은 $T_{arm}$ 뒤 실현). `PlanSnapshot` 의 시각(γ 프로파일 `t0`·`t1`, $t_c$)은 절대 steady ns 이고, `Step(o, t, dt)` 의 `t` 와 `GammaProfile` 의 `t0`·`t1` 은 **수치 코어 경계에서** 같은 원점의 상대 초로 바꾼 값이다 (`ProfileSeconds`·`MakeGammaProfile` — 원점은 `PlanSnapshot` 의 γ 램프 시작). 대상 `o` 는 L2 샘플러로 같은 $now_{lead}$ 에서 샘플링한다. 매 tick 의 $now$ 는 steady 실측이며 tick 수 × dt 로 계산하지 않는다. T_arm ≠ 0 fixture 로 두 축을 구분해 테스트한다
- plan 에 의한 `SetIntercept()` (첫 채택과 교체) 는 `COMMITTED` 이전에만 일어난다. `COMMITTED` 이후 허용되는 계획 변경은 없고, 그 뒤의 호출은 DECEL 진입의 γ≡1 전환 하나뿐이다 (아래).
- **반환값의 시간축이 섞여 있다.** `x`, `xd`는 $t+\Delta t$ 기준(다음 틱 명령), `xdd`는 $[t,t+\Delta t]$ 구간의 실현 가속도, `e`, `ed`는 $t$ 기준 진단값이다. L8 `TickRecord`에 함께 기록할 때 1틱 오정렬을 감안한다.
- **`xdd` vs `u_des`.** 속도 포화가 걸리면 DS가 요구한 가속도 `u_des`는 실현되지 않는다. 실현값이 필요한 곳(기록)에는 `xdd`를, L3 rollout 판정과 포화 진단에는 `u_des`를 쓴다.
- **CLIK 공급.** 기준 `x` 를 위치 목표로, `xd` 를 선속도 feedforward 로, `PlanSnapshot` 의 $a_d$ 를 축 목표로 넘긴다 (`ClikReferenceGenerator::PositionAxisTarget`). 각속도 feedforward 는 주지 않는다 — 고정된 포구점의 접근축은 돌지 않는다. 어느 성분이 어느 행에 실리는지는 L5 §4.2.
- 감속 모드(L7 §4.3, `closed_form`): 대상을 가상 감속 목표로 바꾸고 γ 를 상수 1 (`GammaProfile{1,1,…}`) 로 둔다. 전환은 $now_{lead}\ge t_c$ 에서 한다 (A-5). 가상 목표는 **진입 tick 의 기준 상태** $(x_s,\dot x_s)$ 에서 시작하므로 진입 시 $e=0$, $\dot e=0$ 이 정확히 성립한다 — 기준 생성기는 리셋하지 않고 대상만 바꾼다. `mpc` 에서는 감속도 구간이 만든다.
- 회전: CLIK 의 오차·Jacobian 은 현재 **명령 자세** $q_c$ 에서 평가한다 (D-6) — 축 오차의 $z$ 도 $q_c$ 의 FK 다. 실추종 오차는 `TRACK_ERR` 로 별도 감시한다.

### 5.2.1 `derateGamma()` — 동결 후 γ 하향 `[권장]`

채택하지 않았다 — 구현에 없다 (v1 범위 밖, D-8). 실행 중 포화는 `COMMITTED` 이전 RETREAT, 이후 ABORT_SAFE 로 처리하고 (§4.8), 계획 여유 (D-9 의 $\eta_v$) 가 유일한 완충이다.

### 5.3 복귀 기준 — 채택하지 않음 (C-13, S7.2 설계 확정 2026-09-23)

task-space soft-catch DS 로 복귀하는 기준은 채택하지 않았고 구현에 없다. homing·retreat 는 관절공간 법칙 `joint_home.hpp` (per-joint 사다리꼴, QP/CLIK 비의존) 가 한다 — L7 §4.1·§4.8.

이 절에 남는 것은 DS 의 성질이다. **$\gamma\equiv0$ 이면 대상 $o$ 는 결과에 전혀 영향을 주지 않는다.**

$$u=-\omega^2(x-p_c)-2\zeta\omega\,\dot x$$

가 되어 $o.p,\,o.v,\,o.a$ 가 모두 소거되고, 끌개는 오직 $p_c$ 다. 따라서 정지 목표는 **$p_c$ 로** 지정해야 한다 (대상을 그 점으로 주는 것은 no-op 이다).

$$\texttt{SetIntercept}(p_{goal},\ \texttt{GammaProfile}\{0,0,\cdot,\cdot\})$$

같은 이유로 `Reset(x, xd)` 는 $p_c\leftarrow x$, $\gamma$ 프로파일 초기화까지 수행한다 — 그래야 활성화 직후 현재 자세 유지가 되고, 재무장 시 직전 시행의 포구점·γ 프로파일이 남지 않는다(L7 §4.8 재무장 리셋 목록).

## 6. YAML 파라미터

값 · 범위 · 근거는 YAML 과 파서가 갖는다 (`integrated_bringup/config/<robot>/controllers/`). `reference.*` 는 `catching/planner_closed_form.yaml` 에 있다 — `closed_form` 의 RT 법칙의 값이지만 **`mpc` 에서도 탐색이 읽는다** (rollout · γ 창).

| 키 | 뜻 | 단위 | 자리 |
|---|---|---|---|
| `reference.omega` | §4.1 의 $\omega$. 검증기가 실제 $h$ = `ControllerState::dt` 로 $s=\omega h$ 를 검사 (§4.7). 클수록 잔여 오차는 줄지만 예측 잡음을 따라간다 — $\omega$ 의 지연이 저역 필터 역할을 한다 | rad/s | `catching/planner_closed_form.yaml` |
| `reference.zeta` | §4.1 의 $\zeta$. 1 만 허용 — 닫힌해(§4.4)·이산 경계(§4.7)가 1에서만 유효 | – | 같은 파일 |
| `reference.a_max` | §4.8 의 가속 포화 (TBD-ARM-02 — 로봇·자세 의존). L7 의 `supervisor.decel.a_dec` 는 이 값 이하여야 한다 | m/s² | 같은 파일 |
| `reference.v_max` | §4.8 의 TCP 속도 포화. L3 `ComputeGammaWindow` 는 $\eta_v\cdot$ 이 값을 쓴다 `[확정 D-9]` (L3 §4.5). **실측이 아니라 도출값** — 수락 후보의 LP $v_{dir,\max}$ 최대 (오프라인 도출은 `rtc_tools catch_gate_map --v-max-m-s derived`): URDF·제조사 자료에는 관절 정격만 있다. 그렇게 두면 포화는 관절 정격 안에서는 발화하지 않는다. 실기 TCP 안전 한계는 이 값을 **낮출 수만** 있다 | m/s | 같은 파일 |
| `reference.provisional` | `reference` 블록 전체가 provisional — sim 경고, 실기 차단 (L0 §5.3) | – | 같은 파일 |
| `joint_cmd.K_a` | **축 정렬 이득 $K_a$ 의 키다 (옛 이름 `reference.axis.k_axis`).** $\Vert\omega_{ref}\Vert=K_a\theta$ 이므로 $\theta=\pi$ 에서 $K_a\pi$. CLIK 의 접근축 행이 쓴다 (L5 §6) | 1/s | `demo_catching_controller.yaml` |

키가 아닌 것: 축 정렬의 데드밴드 $\epsilon_{\sin}$ 과 Jacobian 하한 $n_J$ 는 `axis_align.hpp` 의 상수다 (§4.5). 각속도 기준의 노름 상한 ($w_{max}$) 과 γ derate · task-space 복귀의 키는 없다.

## 7. 단위 기술 구현 순서

구현 순서는 이 문서가 갖지 않는다 — 구현은 끝났다. 단위와 코드의 대응만 남긴다.

| 단위 | 코드 | 테스트 |
|---|---|---|
| `GammaProfile`, `SoftCatchTranslation`, 닫힌해 | `soft_catch.hpp` | `test_catching_soft_catch` |
| 축 정렬 오차·Jacobian·각속도 | `rtc_math` se3 `axis_align.hpp` | `test_axis_align` |
| CLIK 결합 (기준 → 5 행 과제) | L5, `integrated_bringup` 의 catching 컨트롤러 | `test_catching_tracking` |

## 8. 디버깅 방법

- 기록 항목(틱마다): $t$, $\gamma,\dot\gamma,\ddot\gamma$, $e,\dot e$, $\Vert u\Vert$, feedforward 성분별 크기, 포화 플래그, 포구점 교체 이벤트. `mpc` 에서는 이 기준이 돌지 않으므로 기준 열이 무효 (`ref_valid` 거짓) 다 — 구간 추종의 열 (`decel_*`) 을 본다.
- 간극이 $(1-\gamma)\delta$보다 크다: 포화 여부 → `t` 시간축 불일치(γ 프로파일과 대상 상태의 시각 차) → $\ddot\xi^O$ 부호 순으로 확인.
- 상대속도가 기대와 다르다: $t_c$에서 $\xi^O\neq0$인지(포구점과 실제 도달 시각 불일치) 확인.
- 접근축이 진동한다: `joint_cmd.K_a` 가 L5 CLIK 대역보다 큰지 확인.
- 접근축이 큰 오차에서 안 움직인다: 회전벡터 기반이면 $\Vert\omega_{ref}\Vert=K_a\theta$ 라 오차가 클수록 빠르다 (§4.5) — 축 목표가 단위벡터인지 (`LastAxisRegion()` 이 무효 입력을 보고하는지), 관절 속도 box 가 묶고 있는지 확인.
- 발산: `omega × dt`가 0.828을 넘는지 확인.
- 포화 중 기준이 이상하다: `xdd`(실현)와 `u_des`(요구)를 함께 기록해 어느 쪽을 쓰고 있는지 확인(§5.2).

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G4-A | §4.9 표 재현 (간극 오차 < $\epsilon_{conv}$ + $(1-\gamma)\delta$의 5%, 상대속도 오차 < 2%). $\epsilon_{conv}=2$ mm는 $\omega=10$, 비행 0.8 s의 DS 잔여 수렴 오차다 | `[SIM-ANY]` |
| G4-B | 재예측 점프 식 일치 (< 1e-9) | `[SIM-ANY]` |
| G4-C | 이산 안정 경계 테스트 통과 ($s=0.80$ 수렴, $s=0.85$ 발산) | `[SIM-ANY]` |
| G4-D | 축 정렬: $\exp([e_a]_\times)z=a_d$ 잔차 < 1e-12, 1° 격자 $\Vert\omega_{ref}\Vert$ 변화 < $1.5K_a\pi/180$, Jacobian 유한차분 오차 < 1e-5 (1–170°), **정렬·반평행 데드밴드와 그 근처에서 출력 전부 유한** | `[SIM-ANY]` |
| G4-E | 해당 없음 — γ derate 는 구현에 없다 (v1 범위 밖, D-8) | – |
| G4-F | 속도 포화 시 `xdd` == 실현 가속도 (< 1e-9) | `[SIM-ANY]` |
| G4-G | 할당 0, `noexcept`, 틱당 최악 실행시간 기록 | `[SIM-ANY]` |
| G4-H | MuJoCo에서 L5와 결합 후 catch frame 실제 궤적이 기준을 추종 (추종 오차 기록) | `[SIM-P1B]` |
| G4-I | NaN 가드: 비유한 목표·`t`·`dt` 입력 시 내부 상태 보존 + invalid, 다음 유한 입력에서 정상 출력, NaN 에서 포화 검출 참 | `[SIM-ANY]` |

G4-A·B·C·F·G·I 는 `test_catching_soft_catch`, G4-D 는 `rtc_math` 의 `test_axis_align` 이 고정한다.

## 10. 미확정 항목

TBD-ARM-02 (`reference.a_max` — 출하는 provisional), TBD-REF-01 (§4.6 [R4]/[R5] 재확인).
