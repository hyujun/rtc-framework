# L4 — Reference: soft-catch DS, 접근축 정렬, 복귀

- 문서 버전: v0.5 (2026-09-19) — 결정·단계의 SSoT 는 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) (충돌 시 plan 우선)
- 브랜치: 단계별 `type/kebab-slug` (main 기준, 마스터 §4.2)
- 배치 `[확정 D-1]`: soft-catch 병진 기준·γ 프로파일·복귀 기준은 rtc_controllers 의 `catching` 하위 디렉토리 (namespace `rtc::catching`), **축 정렬 오차·각속도·Jacobian 은 `rtc_math` se3** 로 옮긴다
- 단계: **S1.4** soft-catch 기준 생성기 (NaN 가드, derate 없음), **S2.1** 축 정렬 함수의 `rtc_math` se3 이식 (deadband·반평행에서 유한), CLIK 기준 공급은 **S2.2b** (CLIK twist feedforward 옵션, D-5) · **S5.3** (스트리밍 기준 → 확장 CLIK)
- 선행: 단계 W, L0 (L2 궤적 샘플러 `traj::sampleAt()` 출력을 입력으로 받음)
- 산출물: soft-catch 병진 기준 (참조: `soft_catch_reference.hpp`), 복귀 기준, 수렴 한계
- 비고: L3가 γ rollout에서 **이 layer의 같은 코드**를 호출한다.

---

## 1. 범위 / 비범위

범위: 추종 대상(실제 공 또는 L7의 가상 감속 공)의 상태로부터 catch frame의 병진 기준 $(x,\dot x,\ddot x)$와 회전 기준 $\omega_{ref}$를 제어 주기 $h$ = `ControllerState::dt` (= 1/`control_rate`, 100–5000 Hz) 마다 생성한다. 500 Hz 고정을 가정하지 않는다. 복귀(retreat) 기준도 포함한다.

비범위: 관절 공간 변환과 한계 처리(L5), γ·포구점 결정(L3), 모드 전환 판단(L7).

## 2. 코드 확인 게이트

단계 W 에서 처리했다 (2026-09-19, plan §2).

| ID | 확인 항목 | 기록 |
|---|---|---|
| G4-1 | `rtc_tsid` / CLIK가 받는 과제 기준의 형식 | 닫힘 — `rtc::tsid::ClikReferenceGenerator` 는 현재 **pose(SE3) 목표만** 받는다 (6 LWA 행 고정, feedforward·마스크 없음). twist feedforward·LOCAL 접근축 2행·가속 box 는 옵션(기본 off)으로 확장한다 `[확정 D-5]` (S2.2b) |
| G4-2 | 기존 SE(3)/SO(3) 오차 헬퍼(U1 공유 헬퍼)의 규약과 본 문서 §4.5 축 정렬 오차의 공존 방식 | 닫힘 — U1 헬퍼는 `rtc_tsid` se3_error (`ComputeTaskPoseError`, LWA BodyLog6). §4.5 축 정렬 오차는 그것과 **별도 함수**로 `rtc_math` se3 (`log3`/`exp3` 기반) 에 둔다 (D-1, S2.1) |
| G4-3 | 두 로봇의 catch frame과 손바닥 바깥 법선 축 | 닫힘 — D-17: 모델 빌더가 YAML 선언 frame 으로 추가 (D-10), 부모·offset·자세는 로봇 config, 접근축 = 그 frame 의 +z. 초기값은 S2.3a/b 제안 → 사용자 sim 확인 (plan §10) |

## 3. 참고자료

[R3] soft catch 이론(오차 좌표 LPV DS, softness) — **본 layer의 1차 근거, 원문 대조 완료(§4.1)**. [R4] 같은 구조의 다중 팔 확장 — **전문 미보유, 식 번호 인용 금지**. [R5] 공개 코드(§4.6, **미열람**). [R7] SO(3).

## 4. 수학적 이론

### 4.1 오차 좌표 soft-catch DS ([R3], [R4])

좌표 원점을 예측 포구점 $p_c$에 둔다. 대상 상대위치 $\xi^O=p_O-p_c$, 기준 상대위치 $\xi=x-p_c$. 오차

$$e=\xi-\gamma\xi^O,\qquad \dot e=\dot\xi-(\gamma\dot\xi^O+\dot\gamma\xi^O)$$

가 다음을 따르도록 기준 가속도 $u=\ddot x$를 정한다.

$$\ddot e=A_1e+A_2\dot e$$

$\ddot\xi=u$에서 $\ddot e=u-(\gamma\ddot\xi^O+2\dot\gamma\dot\xi^O+\ddot\gamma\xi^O)$이므로

$$u=\gamma\ddot\xi^O+2\dot\gamma\dot\xi^O+\ddot\gamma\xi^O+A_1e+A_2\dot e$$

본 구현은 $A_1=-\omega^2I$, $A_2=-2\zeta\omega I$ (LTI)로 시작한다. [R3]의 GMM 기반 LPV $A_i(\theta)$는 확장 항목이다.

**[R3] 원문 대조 (v0.2에서 완료).** 결론은 **동치**이나 plant 규약이 달라 겉보기 식이 다르므로 주의한다.

[R3] 식 (1)의 plant는 순수 이중적분기가 아니다.

$$\ddot\xi=A_1(\theta_{A_1})\xi+A_2(\theta_{A_2})\dot\xi+u$$

그래서 [R3] 식 (4)의 제어입력에는 대상 항을 상쇄하는 성분이 붙는다.

$$u_{[R3]}=\gamma\ddot\xi^O-A_1\gamma\xi^O-A_2(\gamma\dot\xi^O+\dot\gamma\xi^O)+2\dot\gamma\dot\xi^O+\ddot\gamma\xi^O$$

여기에 plant 항을 더하면

$$\ddot\xi=A_1\xi+A_2\dot\xi+u_{[R3]}=A_1e+A_2\dot e+\gamma\ddot\xi^O+2\dot\gamma\dot\xi^O+\ddot\gamma\xi^O$$

로, 본 문서의 $u$ 와 **총 가속도가 정확히 같다**. 본 문서는 $\ddot\xi=u$ 규약이므로 $u_{doc}=\ddot\xi\neq u_{[R3]}$ 다. 오차 좌표 정의도 [R3] 식 (5)와 문자 그대로 일치한다. 코드 이식 시 [R3] 식 (4)를 그대로 옮기면 plant 항을 두 번 빼게 되니 주의할 것.

[R3] 식 (11)의 수치 예는 $\ddot\xi=-12\dot\xi-36\xi+u$ 로 $\omega=6$, $\zeta=1$ 이다. 본 문서 기본값 $\omega=10$ 은 그보다 공격적이지만 §4.7의 이산 안정 경계 안에 있다.

- $\ddot\xi^O$는 **vision이 준 $a$** 를 L2 샘플러가 보간한 값을 쓴다(L2 §4.2). 수치미분 금지. v0.2는 제어 PC의 해석적 모델 $g-k\Vert v\Vert v$ 를 썼으나, v0.3에서 제어 PC는 궤적을 재전파하지 않는다(마스터 §5.2). 5차 Hermite 보간이 $C^2$ 라 $\ddot\xi^O$ 가 샘플 경계에서 튀지 않는다 — 이 성질이 여기서 필요한 이유다.
- $\gamma(t)$는 5차 보간 $\gamma_0\to\gamma_f$ ($[t_0,t_1]$), 양 끝의 1·2계 미분이 0이다.

### 4.2 Corollary: 원점 = 포구점의 의미 ([R3], [R4])

$e(t_c)=0$, $\dot e(t_c)=0$이고 $\xi^O(t_c)=0$이면

$$\xi(t_c)=0=\xi^O(t_c),\qquad \dot\xi(t_c)=\gamma\dot\xi^O(t_c)$$

위치는 γ와 무관하게 일치하고, 기준 속도는 대상 속도와 같은 방향에 크기가 γ배다. 상대속도는 $(1-\gamma)\dot\xi^O(t_c)$다.

극한 점검: $\gamma=0$이면 고정점 수렴(정지 포구), $\gamma=1$이면 완전 추종.

### 4.3 예측 오차와 재예측에 대한 민감도 `[논문 외 유도]`

**예측 오차.** 실제 대상이 $t_c$에 $\xi^O(t_c)=\delta\neq0$에 있고 $e\approx0$이면, 기준은 $\gamma\delta$에 있으므로 간극은 $(1-\gamma)\Vert\delta\Vert$다.

**재예측 점프.** L3가 포구점을 $p_c'=p_c+\Delta p_c$로 바꾸면 $\xi^{O\prime}=\xi^O-\Delta p_c$, $\xi'=\xi-\Delta p_c$이므로

$$e'=e-(1-\gamma)\Delta p_c,\qquad \dot e'=\dot e+\dot\gamma\,\Delta p_c$$

기준 상태 $(x,\dot x)$는 연속이고 오차만 점프한다. L3의 교체 히스테리시스(L3 §4.7)가 이 두 크기를 제한한다.

### 4.4 수렴 한계 `[논문 외 유도]`

**임계감쇠 닫힌해** ($\zeta=1$, 축별 독립):

$$e(t)=\big(e_0+(\dot e_0+\omega e_0)t\big)e^{-\omega t},\qquad \dot e(t)=\big(\dot e_0-\omega t(\dot e_0+\omega e_0)\big)e^{-\omega t}$$

L3는 이 식으로 $t_c$에서의 잔여 오차를 포화가 없다는 가정 아래 즉시 예측하고, 포화 여부는 rollout으로 확인한다.

**LPV 확장 시 충분조건.** $z=[e;\dot e]$, 꼭짓점 $\mathcal A_k=\begin{bmatrix}0&I\\A_{1k}&A_{2k}\end{bmatrix}$에 대해

$$\exists P\succ0,\ \alpha>0:\ P\mathcal A_k+\mathcal A_k^\top P\preceq-2\alpha P\ \ \forall k\ \Rightarrow\ \Vert z(t)\Vert\le\sqrt{\kappa(P)}\,e^{-\alpha t}\Vert z(0)\Vert$$

따라서 허용오차 $\epsilon$에 대해 $T\ge\frac1\alpha\ln\frac{\sqrt{\kappa(P)}\Vert z(0)\Vert}{\epsilon}$이면 충분하다.

**이 LMI가 필요한 범위.** [R3] Theorem 1은 점근 수렴만 주장하고 수렴 속도는 다루지 않는다(원문 확인). 다만 본 구현이 쓰는 LTI $A_1=-\omega^2I$, $A_2=-2\zeta\omega I$ 에서는 수렴률이 $\omega$ 로 자명하므로, 위 LMI가 실제로 필요한 것은 **LPV 확장 시뿐이다**. v0.1은 이를 "[R4]가 적은 공백"이라고 서술했으나 [R4] 전문을 보유하고 있지 않아 그 출처 주장은 철회한다(`TBD-REF-01`).

**γ가 수렴에 주는 영향.** $e(0)=\xi(0)-\gamma(0)\xi^O(0)$이다. $\gamma(0)=0$으로 시작하면 초기 오차가 대상 거리와 무관해진다. γ는 feedforward에만 들어가므로 오차계 자체는 임의의 $C^2$ $\gamma(t)$에 대해 같다. 대신 입력 크기 $\Vert u\Vert$가 $\dot\gamma,\ddot\gamma,\Vert\xi^O\Vert$에 비례해 커진다. 이 때문에 γ 창과 램프 구간은 L3가 rollout으로 정한다.

### 4.5 접근축 정렬 `[논문 외 유도]`

손바닥 바깥 법선 $z=R_{WC}\hat e_z$ (W), 목표 $a_d=-\hat v_O(t_c)$. 둘 다 단위벡터다.

**오차는 회전벡터로 정의한다.** $m=z\times a_d$, $c=z^\top a_d$ 에 대해

$$\theta=\mathrm{atan2}(\Vert m\Vert,\,c),\qquad \hat u=\frac{m}{\Vert m\Vert},\qquad \boxed{e_a=\theta\,\hat u},\qquad \omega_{ref}=K_a\,e_a$$

$\exp([e_a]_\times)z=a_d$ 를 **정확히** 만족한다(수치 확인: 잔차 $<5\times10^{-16}$, 0–180°). 따라서 $\Vert e_a\Vert=\theta$ 이고 $\Vert\omega_{ref}\Vert=K_a\theta$ 는 정렬 오차에 대해 연속·단조다. 접근축 주위 회전(roll)에는 기준을 주지 않는다(5-DoF). $e_a\perp z$ 이므로 L5의 LOCAL 마스크 $S=\mathrm{diag}(1,1,0)$ 가 정보를 버리지 않는다는 성질은 그대로다.

수렴성: $\dot z=\omega\times z$ 이고 $\omega_{ref}=K_a\theta\hat u$ 이므로 $\frac{d}{dt}(z^\top a_d)=K_a\frac{\theta}{\sin\theta}\big(1-(z^\top a_d)^2\big)\ge0$. $z^\top a_d$ 가 단조 증가하므로 반평행 평형점을 제외하면 전역 수렴한다.

$\Vert m\Vert<\epsilon_{\sin}$ 이고 $c<0$ 이면(반평행) 축이 정의되지 않으므로 $z$에 수직인 임의 축을 골라 $e_a=\pi\hat u_\perp$ 를 쓴다. 크기는 $\pi$ 로 연속이다.

**v0.1의 $e_a=z\times a_d$ 대비 무엇이 달라지는가.** 이전 정의는 크기가 $\sin\theta$ 라 두 가지 문제가 있었다(실측).

| 정렬 오차 각 | 5° | 45° | 90° | 135° | 170° | 177° | 179° |
|---|---|---|---|---|---|---|---|
| $\sin$ 기반 $\Vert\omega_{ref}\Vert$ (v0.1) | 0.70 | 5.66 | 6.00 | 5.66 | 1.39 | 0.42 | **6.00** |
| 회전벡터 기반 (v0.2, $K_a=8$, $\omega_{max}=6$) | 0.70 | 6.00 | 6.00 | 6.00 | 6.00 | 6.00 | 6.00 |

1. 반평행 임계(v0.1의 YAML 키 `reference.axis.antiparallel_cos = -0.999`, 177.4°. **v0.2에서 이 키는 폐기됐다** — 회전벡터 정식화에는 반평행 분기가 없고, 대신 `reference.axis.sin_eps` 데드밴드를 쓴다)에서 $0.36\to6.0$ rad/s로 튀었다. CLIK 입력에 무한 jerk가 들어간다.
2. 165–177° 구간에서 사실상 수렴이 멈췄다. 대기 자세가 공 반대쪽을 보고 있으면 여기에 걸린다.

회전벡터 기반은 1° 격자에서 최대 변화가 0.14 rad/s($=K_a\cdot\Delta\theta$)로 연속이다(`test_l4.cpp`).

**오차 Jacobian** (W 표현 각속도 $\omega=J_\omega^W\dot q$). $f(c)=\theta/\sin\theta$ 로 두면 $e_a=f(c)\,m$ 이고, $\dot c=m^\top\omega$, $\dot m=[a_d]_\times[z]_\times\omega$ 이므로

$$\dot e_a=J_a\,\omega,\qquad \boxed{J_a=f'(c)\,mm^\top+f(c)\,[a_d]_\times[z]_\times},\qquad f'(c)=\frac{\theta c/\sin\theta-1}{\sin^2\theta}$$

$\theta\to0$ 극한은 $f\to1$, $f'\to-1/3$ 로 유한하고, 이때 $J_a\to[a_d]_\times[z]_\times$ 라 v0.1 식과 일치한다(v0.1 식은 $\theta\to0$ 근사였다). $\theta\to\pi$ 에서는 $f\to\infty$ 로 발산한다 — 축이 정의되지 않기 때문이며, 이는 정의의 결함이 아니라 문제의 성질이다. L5가 마스크 방식(§L5 4.2)을 쓰면 $J_a$ 자체가 필요 없다.

유한차분 검증: 1–170° 구간 최대 오차 $2.4\times10^{-6}$ (`test_l4.cpp`), `verify_l3.py` [3]도 같은 식을 독립 구현으로 대조한다.

**구현 주의 두 가지.**

1. 소각도 급수는 $c>0$ 일 때만 쓴다. $\sin\theta$ 는 $\theta\to0$ 과 $\theta\to\pi$ **양쪽에서** 0이므로, 부호를 보지 않으면 반평행 근처에서 발산해야 할 $f$ 가 유한한 값(≈2.645)으로 조용히 바뀐다. 실측: $\theta=180°-5.7\times10^{-7}°$ 에서 참값 $3.16\times10^{8}$ 대신 2.645.
2. `axisAlignJacobian` 의 `sin_eps` 는 `axisAlignError` 와 **같은 값**이어야 한다. 다르면 데드밴드 안에서 $e_a$ 는 상수인데 $J_a$ 가 0이 아닌 값을 돌려주어, $J_a$ 가 더 이상 $e_a$ 의 야코비안이 아니게 된다.

**참조 구현 결함 — 함수는 항상 유한값을 돌려줘야 한다 (S2.1).** 위 $\theta\to\pi$ 발산은 수학의 성질이지만, RT 경로 함수가 NaN·폭주값을 내보내면 CLIK 입력이 오염된다. 참조 `axisAlignJacobian` 은 반평행 데드밴드($\Vert m\Vert<\epsilon_{\sin}$, $c<0$)와 그 근처에서 NaN 또는 폭주값을 낸다. `rtc_math` se3 이식 시 요구사항:

- 정렬 데드밴드: $J_a=0$ (`axisAlignError` 가 상수 0 이므로)
- 반평행 데드밴드: `axisAlignError` 가 고정 축 $\pi\hat u_\perp$ 를 돌려주는 구간이므로 그에 맞는 유한값 (0 포함) — 비유한 출력 금지
- 데드밴드 경계 근처의 $f$·$f'$ 는 유한 상한으로 제한하고, 그 선택을 유한차분 테스트로 고정한다 (G4-D)
- 입력 비단위·NaN 에도 유한값 + 무효 표시

**S2.1 구현 (`rtc_math/include/rtc_math/se3/axis_align.hpp`).** 세 함수 `AxisAlignError`·`AxisAlignJacobian`·`AxisAlignOmega` 가 결과와 함께 분기 (`AxisAlignRegion`) 를 돌려준다.

| 분기 | 조건 | $e_a$ | $J_a$ |
|---|---|---|---|
| 정렬 데드밴드 | $\Vert m\Vert<\epsilon_{\sin}$, $c>0$ | 0 | 0 |
| 반평행 데드밴드 | $\Vert m\Vert<\epsilon_{\sin}$, $c\le0$ | $\pi\hat u_\perp$ ($z$ 로 정해지는 고정 축) | 0 |
| Jacobian 상한 | $c<0$, $\epsilon_{\sin}\le\Vert m\Vert<n_J$ | 정확값 | $f,f'$ 를 $\Vert m\Vert=n_J$ 에서 평가 — $\Vert J_a\Vert\lesssim\pi/n_J$, $n_J$ 에서 연속 |
| 무효 입력 | 비유한·비단위($\vert\Vert v\Vert-1\vert>10^{-6}$)·$[10^{-12},1)$ 밖의 $\epsilon_{\sin}$, $n_J$ | 0 | 0 |

기본값은 $\epsilon_{\sin}=10^{-6}$, $n_J=10^{-3}$ 이다. 상한은 약 179.94° 이상에서만 걸리므로 G4-D 의 유한차분 범위(1–170°)에 영향이 없다. 두 데드밴드 모두 $e_a$ 가 상수라 $J_a=0$ 을 돌려준다. 소각도 급수는 $c>0$, $\theta<10^{-3}$ 에서만 쓴다 ($f=1+\theta^2/6$, $f'=-1/3-2\theta^2/15$). ω 는 $K_a e_a$ 를 $\Vert\omega\Vert\le w_{max}$ 로 줄이고 `saturated` 를 올린다. 비유한 오차·잘못된 게인은 0 과 무효를 돌려준다.

### 4.6 공개 코드 [R5]와의 차이 (이식 금지 목록)

[R5] `bimanual_ds.cpp`의 `Update()`는 [R4]의 식과 다음이 다르다. 본 구현은 논문 식을 따른다.

1. γ 갱신: 논문은 노름, 코드는 제곱노름에 이득 0.1, 적분 계수 0.2.
2. feedforward: $\dot\gamma\dot\xi$ 한 개와 $\ddot\gamma\xi$ 누락, $x_\gamma^O$ 속도 성분의 $\dot\gamma\xi$ 누락.
3. 결합항: 논문의 $U_j=\dot x_R+A_R(x_{V,j}-x_R)$ 에서 코드는 $\dot x_R$ 항이 없다. 등속 대상에 대해 정상상태 추종 오차가 남는다.

**`확인 필요` (v0.2).** v0.1은 항목 3의 크기를 "$z_{ss}=2A^{-1}\dot x^O$ ($\omega=5$, 1.5 m/s에서 −1.2 m)"로 적었는데, $A=-\omega I$ 로 두면 그 식은 $2\cdot(-1/5)\cdot1.5=-0.6$ m를 준다. **식과 수치가 서로 맞지 않는다.** 재현 스크립트도 남아 있지 않다. 그리고 세 항목 모두 [R4] 전문과 [R5] 코드를 대조해 얻은 것인데 현재 둘 다 보유하고 있지 않다(마스터 §8).

따라서 §4.6 전체를 `TBD-REF-01`로 내린다. 다음 중 하나가 될 때까지 이 절의 구체적 수치를 인용하지 않는다.

1. [R4] 전문과 [R5] 코드를 확보해 세 항목을 재확인하고 1차원 재현 스크립트를 이 폴더에 남긴다.
2. 확보하지 못하면 §4.6을 "[R5] 공개 코드는 논문 식과 다를 수 있으므로 이식하지 않고 논문 식으로 직접 구현한다"는 **방침만** 남기고 항목별 주장을 삭제한다.

어느 쪽이든 구현 방침(논문 식 기준 직접 구현, 코드 이식 금지)은 바뀌지 않는다.

### 4.7 이산화

기준 상태는 반암시적 오일러로 적분한다. 오차계를 같은 방식으로 이산화하면 $s=\omega h$에 대해

$$M=\begin{bmatrix}1-s^2&1-2s\\-s^2&1-2s\end{bmatrix}\quad(\text{상태 }[e;\,h\dot e])$$

이고, Jury 조건에서 안정 조건은 $0<s<2\sqrt2-2\approx0.828$이다(수치 확인: $s=0.83$에서 스펙트럼 반경 1.005). 정확도를 위해 $s\le0.05$를 권장한다. $h=2$ ms면 $\omega\le25$ rad/s다.

**$h$ 는 실제 제어 주기다 (v0.5).** $h$ = `ControllerState::dt` = 1/`control_rate` 이고 `control_rate` 는 100–5000 Hz 범위다. 500 Hz 에서 맞는 $\omega$ 가 100 Hz ($h=10$ ms) 에서는 $s=\omega h$ 가 5배가 되어 안정 경계를 넘을 수 있다 ($\omega=10$ → $s=0.1$, 권장치 초과). 파라미터 검증기가 configure 시 실제 $h$ 로 $s$ 를 검사한다: 경계 이상 → `armable=false`, 권장치 초과 → 경고 (L0 §5.3, S1.7). 위 $M$ 은 $\zeta=1$ 의 식이다.

**$\zeta\neq1$ 은 검증기가 다뤄야 한다.** §4.4 닫힌해 `criticallyDampedError` 는 $\zeta=1$ 을 가정하므로, `reference.zeta` $\neq1$ 이면 L3 종단 오차 예측이 틀린다. v1 검증기는 $\zeta\neq1$ 을 `armable=false` 로 막는다 (L0 §5.3). 일반 $\zeta$ 를 허용하려면 닫힌해와 이산 안정 경계를 $\zeta$ 에 대해 다시 유도해야 한다.

### 4.8 포화

$\Vert u\Vert\le a_{\max}$, $\Vert\dot x\Vert\le v_{\max}$로 포화시키고 플래그를 올린다. 포화 중에는 §4.1의 오차계가 성립하지 않는다.

soft catch에서는 포화가 포구 직전에 몰려 치명적이다. 지난 분석의 수치 예($t_c=0.8$ s, 예측 오차 3 cm)에서 간극은 다음과 같았다.

| 조건 | 간극 |
|---|---|
| 포화 없음, $\gamma_f=0.4$ | 17.7 mm |
| $a_{\max}=15$, $\gamma_f=0.4$ | 97.4 mm |
| $a_{\max}=15$, $\gamma_f=0$ | 29.7 mm |

따라서 L3가 rollout으로 포화 없는 $(\gamma_f,T_w)$를 선택하고(L3 §4.8), 계획 γ 창이 $\eta_v$`reference.v_max` 로 여유를 남긴다 (D-9). 실행 중 포화가 나면 `COMMITTED` 이전은 RETREAT, 이후는 ABORT_SAFE 다 `[확정 D-8]`. v0.4 의 "먼저 γ 하향, 그래도 안 되면 abort" 경로(`derateGamma`)는 v1 범위 밖이다 (§5.2.1).

### 4.9 Sanity check

1. $\gamma_f=0$, 예측 정확: 간극 ≈ 0, 상대속도 ≈ $\Vert v_O\Vert$.
2. 예측 오차 $\delta$: 간극 ≈ $(1-\gamma)\Vert\delta\Vert$.
3. 상대속도 ≈ $(1-\gamma)\Vert v_O(t_c)\Vert$.
4. 닫힌해와 미세 스텝 적분 일치.
5. 축 정렬: $\Vert\omega_{ref}\Vert$ 가 정렬 오차에 대해 연속, $\exp([e_a]_\times)z=a_d$, Jacobian이 유한차분과 일치.
6. (회귀) 속도 포화 시 반환 `xdd`가 실현 가속도와 일치(§5.1).
7. (v1 범위 밖, D-8) `derateGamma` 후 $\gamma$, $e$ 연속, $\dot e$ 점프 $=\vert\dot\gamma\vert\Vert\xi^O\Vert$.

`test_l4.cpp` 실행 결과($\omega=10$, $h=2$ ms, 포화 없음, $T_w=0.4$ s, $t_c=0.8$ s):

| 조건 | 간극 | 예측 $(1-\gamma)\delta$ | 상대속도 | 예측 $(1-\gamma)\Vert v_O\Vert$ |
|---|---|---|---|---|
| $\delta=0,\gamma_f=0$ | 1.1 mm | 0 | 5.662 | 5.671 |
| $\delta=0,\gamma_f=0.4$ | 1.1 mm | 0 | 3.395 | 3.402 |
| $\delta=3$ cm, $\gamma_f=0$ | 29.8 mm | 30.0 | 5.662 | 5.671 |
| $\delta=3$ cm, $\gamma_f=0.4$ | 17.7 mm | 18.0 | 3.395 | 3.402 |

$\delta=0$ 행의 1.1 mm는 오차가 아니라 **DS 자체의 잔여 수렴 오차** $\epsilon_{conv}$ 다($\omega=10$, 비행 0.8 s). 게이트는 이 값을 허용해야 한다 — G4-A의 허용치 $\epsilon_{conv}=2$ mm가 이 1.1 mm를 덮는다(§9 G4-A).

v0.1 표와의 차이: $\gamma_f=0.4,\delta=0$ 행이 0.6 → 1.1 mm, 상대속도가 3.393 → 3.395로 바뀌었다. v0.1 테스트가 참값 공 궤적을 자체 semi-implicit Euler로 만들었기 때문이다($h=2$ ms, $T=0.8$ s에서 계통 오차 $\tfrac12gTh\approx7.9$ mm — 검증 대상인 간극과 같은 자릿수다). v0.2는 L0의 `ball::propagate`(RK4)를 쓴다. 간극 결론은 바뀌지 않는다.

축 정렬: $\exp([e_a]_\times)z=a_d$ 잔차 $4.6\times10^{-16}$, 1° 격자 최대 각속도 변화 0.140 rad/s, Jacobian 유한차분 오차 $2.4\times10^{-6}$ (1–170°).

## 5. C++ 구현

### 5.1 `soft_catch_reference.hpp` (참조 구현, 검증 완료)

SSoT 는 같은 폴더의 `soft_catch_reference.hpp` (v0.4) 다 — 문서에 코드를 복제하지 않는다. 내용: `TargetState`, `GammaProfile` (5차 램프), `TranslationOutput` (`x`·`xd`·`xdd`·`u_des`·`e`·`ed`·γ 3종·`saturated`), `SoftCatchTranslation` (`reset`·`setIntercept`·`step`, 그리고 v1 범위 밖인 `derateJump`·`derateGamma`·`DerateResult`), `AxisAlignParams`·`axisAlignError`·`axisAlignOmega`·`axisAlignJacobian` (§4.5), `criticallyDampedError` (§4.4).

S1 이식 시 변경:

- **배치·명명** (D-1, S0.3): 병진 기준·γ 프로파일은 rtc_controllers `catching` (namespace `rtc::catching`, S1.4), 축 정렬 3함수는 `rtc_math` se3 (S2.1). 함수 PascalCase
- **NaN 가드 (S1.4).** 참조 `step` 은 비유한 목표(`TargetState` 의 NaN/Inf)가 한 번 들어오면 내부 상태 `x_`·`xd_` 가 NaN 으로 **영구 오염**된다. 또 `u.norm() > a_max` 비교가 NaN 에서 거짓이라 **포화 플래그도 서지 않는다**. 이식본은 비유한 입력(목표·`t`·`dt`) 시 **내부 상태를 보존**하고 출력을 invalid 로 표시하며, 포화 검출은 NaN 에서도 참이 되도록 부정 비교(`!(un <= a_max)`)로 쓴다.
- **derate 제거 (D-8).** `derateGamma`·`derateJump`·`DerateResult` 는 이식하지 않는다 (§5.2.1)
- **축 정렬 함수 유한성 (S2.1).** 데드밴드·반평행에서 NaN·폭주 금지 (§4.5)
- **dt 검증.** `dt` 는 `ControllerState::dt` (100–5000 Hz) 이고, $\omega h$ 검사는 configure 의 검증기가 한다 (§4.7). 비유한·비양수 `dt` 는 invalid. 참조 테스트가 `step(o, t, 0)` 으로 하던 "적분 없이 e·ė 읽기" 는 별도 `Evaluate(o, t)` 로 분리했다 (S1.4)
- RT 경로 코드이므로 할당 0·`noexcept` 유지 (G4-G)
- 테스트: `test_l4.cpp` → GTest (S1.1), NaN 회귀 테스트 추가 (G4-I)

### 5.2 사용 규약

- **시간축 (plan §3, D-2).** γ 프로파일 평가와 대상 샘플링은 **선행 시각 $now_{lead}=now+T_{arm}$** 축이다 (팔 명령은 $T_{arm}$ 뒤 실현). `PlanSnapshot` 의 시각(γ 프로파일 `t0`·`t1`, $t_c$)은 절대 steady ns 이고, `step(o, t, dt)` 의 `t` 와 `GammaProfile` 의 `t0`·`t1` 은 **수치 코어 경계에서** 같은 원점의 상대 초로 바꾼 값이다. 대상 `o` 는 L2 샘플러로 같은 $now_{lead}$ 에서 샘플링한다. 매 tick 의 $now$ 는 steady 실측이며 tick 수 × dt 로 계산하지 않는다. T_arm ≠ 0 fixture 로 두 축을 구분해 테스트한다
- `setIntercept()`는 L7이 `COMMITTED` 이전에만 호출한다. **v1 에서 `COMMITTED` 이후 허용되는 계획 변경은 없다** (γ 하향은 v1 범위 밖, D-8 — §5.2.1).
- **반환값의 시간축이 섞여 있다.** `x`, `xd`는 $t+\Delta t$ 기준(다음 틱 명령), `xdd`는 $[t,t+\Delta t]$ 구간의 실현 가속도, `e`, `ed`는 $t$ 기준 진단값이다. L8 `TickRecord`에 함께 기록할 때 1틱 오정렬을 감안한다.
- **`xdd` vs `u_des`.** 속도 포화가 걸리면 DS가 요구한 가속도 `u_des`는 실현되지 않는다. 실현값이 필요한 곳(CLIK 공급, 기록)에는 `xdd`를, L3 rollout 판정과 포화 진단에는 `u_des`를 쓴다. v0.1은 포화 후에도 `u_des`를 `xdd`로 돌려주어, 예를 들어 $v_{max}=0.5$ m/s 조건에서 실현 5.0 m/s² 대신 490 m/s²를 보고했다.
- **CLIK 공급 (S2.2b·S5.3).** 현 `rtc::tsid::ClikReferenceGenerator` 는 pose 목표만 받으므로, 기준 `x` 와 접근축 목표를 pose 로 넘기고 `xd`·$\omega_{ref}$ 는 twist feedforward 옵션(D-5)으로 넘긴다. 어떤 성분을 어느 행에 싣는지(LOCAL 접근축 2행, 가속 box 와의 관계)는 S2.2b CLIK 확장 설계에서 확정한다 (구조 자체는 S2.2a 에서 먼저 결정).
- 감속 모드(L7): 대상에 가상 감속 공을 넣고 `GammaProfile{1,1,…}`(상수 1)로 바꾼다. 전환은 $now_{lead}\ge t_c$ 에서 한다 (A-5). 전환 시각이 $t_c$이면 $\xi^O(t_c)\approx0$이라 오차 점프가 작다(§4.3). 가상 공의 초기 속도를 전환 시점의 기준 속도로 두면 $\dot e$도 연속이다(L7 §4.3).
- 회전: `axisAlignOmega(z_cmd, a_d, p)`의 `z_cmd`는 현재 **명령 자세** $q_c$ 의 FK 기준이다 — CLIK 오차·J 를 명령값 $q_c$ 에서 평가하는 옵션(D-6)과 같은 자세다. 실추종 오차는 `TRACK_ERR` 로 별도 감시한다.

### 5.2.1 `derateGamma()` — 동결 후 γ 하향 `[권장]`

**v0.5 에서 v1 범위 밖 (D-8) — S8 포화 빈도 측정 후 재검토.** v1 은 실행 중 포화 시 `COMMITTED` 이전 RETREAT, 이후 ABORT_SAFE 로 처리하고, 계획 여유(D-9 $\eta_v$)가 유일한 완충이다. S8 에서 γ 포화 빈도를 측정해 재도입 여부를 정한다 (plan §4 S8).

**참조 구현 probe 결과 (v1 제외 근거).**

- `Frozen` 분기가 $\gamma_{\min}$ (손 폐쇄 하한, L3 §4.5) 을 보장하지 않는다 — 램프 상승 구간에서 현재 $\gamma(t)<\gamma_{\min}$ 인 값으로 동결될 수 있고, `Applied` 도 `gf_new` 를 $\gamma_{\min}$ 으로 clamp 하지 않는다
- 완화 분기가 무효다
- 기본 램프 0.05 s (`reference.gamma_derate.ramp`) 가 가속 피크를 오히려 **20 → 70 m/s²** 로 키운다
- 램프 끝 $t+T_{ramp}$ 가 $t_c$ 를 넘을 수 있다 — 그러면 $t_c$ 에서 $\dot\gamma\neq0$ 이라 §4.2 Corollary 가 성립하지 않는다

**재도입 시 재설계 요구사항.**

1. 결과 γ 를 항상 $\gamma\ge\gamma_{\min}$ 으로 clamp (`Applied`·`Frozen` 모두)
2. 램프 끝 $\le t_c$ (남은 시간에 맞춰 램프 길이를 줄이거나 거부)
3. **forward rollout 수락** — 하향 후 궤적을 L3 §4.8 과 같은 rollout 으로 $t_c$ 까지 굴려 $\max\Vert u_{des}\Vert$ 가 실제로 줄어드는 경우에만 적용
4. 키는 단일 키로 (`derate_step`, ramp, `ed_jump_max` 공유 — S0.3)

**v0.4 분석 (재도입 검토용 기록).** 아래는 v0.4 설계와 그 연속성 분석이다. L7 §4.6 이 참조하던 내용이며, 재설계의 출발점으로만 남긴다.

**비교 기준은 현재 값 $\gamma(t)$ 가 아니라 목표 $\gamma_f$ 다.** 현재 값을 기준으로 삼으면 램프 상승 구간에서 요청이 전부 거부된다 — $\gamma(t)\ll\gamma_f$ 이므로 `gf_new = γ_f − step` 이 거의 항상 $\gamma(t)$ 보다 크기 때문이다. 실측(γ_f=0.4, 램프 [0, 0.45] s, step=0.1): $t=0.05\sim0.25$ 에서 전부 거부, $t\ge0.30$ 부터 수락.

| 결과 | 조건 | 동작 |
|---|---|---|
| `Applied` | $\gamma_f^{new}<\gamma(t)$ | 목표를 낮추고 현재 값에서 새 램프 시작 |
| `Frozen` | $\gamma_f^{new}\ge\gamma(t)$ 이지만 $\gamma_f^{new}<\gamma_f$ | **현재 값에서 동결** ($\gamma_0=\gamma_f=\gamma(t)$) |
| `Rejected` | $\gamma_f^{new}\ge\gamma_f$ (상향) | 무시 |

연속성: $\gamma(t)$ 연속 → $e$ 연속, $\dot\gamma$ 가 0으로 점프 → $\dot e$ 가 $\vert\dot\gamma(t)\vert\,\Vert\xi^O(t)\Vert$ 만큼 점프.

**이 점프는 시간에 대해 단조가 아니다.** $\dot\gamma$ 는 5차 램프의 **중앙에서 최대**($1.875\,\Delta\gamma/T$)이고 $\Vert\xi^O\Vert$ 만 단조 감소하므로, 곱은 봉우리를 만든다. 실측(램프 [0.4, 0.8] s, $\gamma_f=0.4$, $t_c=0.8$):

| $t$ [s] | 0.45 | 0.50 | **0.56** | 0.60 | 0.70 | 0.76 | 0.79 |
|---|---|---|---|---|---|---|---|
| $\vert\dot\gamma\vert$ | 0.36 | 1.06 | 1.83 | 1.88 | 1.06 | 0.29 | 0.02 |
| $\Vert\xi^O\Vert$ | 1.61 | 1.42 | 1.11 | 1.01 | 0.53 | 0.19 | 0.06 |
| 점프 | 0.58 | 1.50 | **2.04** | 1.88 | 0.56 | 0.05 | 0.001 |

맞는 것은 극한 주장뿐이다: $t\to t_c$ 에서 점프 $\to0$. 필요 가속도는 $\gamma_f$ 에 거의 선형으로 줄어든다(L3 §4.8 표: $T_w=0.30$ s에서 $\gamma_f$ 0.4→0.2가 33.0→16.4 m/s²).

### 5.3 복귀 기준 `retreat_reference.hpp`

**주의: $\gamma\equiv0$ 이면 대상 $o$ 는 결과에 전혀 영향을 주지 않는다.**

$$u=-\omega^2(x-p_c)-2\zeta\omega\,\dot x$$

가 되어 $o.p,\,o.v,\,o.a$ 가 모두 소거되고, 끌개는 오직 $p_c$ 다. 따라서 정지 목표는 **$p_c$ 로** 지정해야 한다.

$$\texttt{setIntercept}(p_{home},\ \texttt{GammaProfile}\{0,0,\cdot,\cdot\})$$

v0.2는 "대상을 홈 위치로, γ를 0으로 넣어 재사용"이라고 적었는데 이것은 **no-op** 이다. 실측하면 대상을 홈으로 주든 임의의 점으로 주든 똑같이 **직전 포구점**으로 수렴한다. 그러면 L7 §4.5 조건 4(대기 자세 허용오차)가 영원히 거짓이라 `RETREAT → ARMED` 전이가 막힌다. `test_l4.cpp` 의 `A5` 가 회귀 검사한다.

같은 이유로 `reset(x, xd)` 는 $p_c\leftarrow x$, $\gamma$ 프로파일 초기화까지 수행한다 — 그래야 활성화 직후 현재 자세 유지가 되고, 재무장 시 직전 시행의 포구점·γ 프로파일이 남지 않는다(L7 §4.8 재무장 리셋 목록). 같은 기준이 IDLE 의 wait_pose homing 에도 쓰인다 (L7).

회전은 홈 자세로의 SO(3) 오차로, 기존 U1 헬퍼 `rtc_tsid` se3_error (`ComputeTaskPoseError`) 를 쓴다(G4-2). 별도 파일로 둘지 병진 기준 코어에 함께 둘지는 S1.4 에서 정한다.

## 6. YAML 파라미터

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `reference.omega` | double | rad/s | 10.0 | 1–25 | §4.7. 검증기가 실제 $h$ = `ControllerState::dt` 로 $s=\omega h$ 를 검사 (경계 0.828 이상 armable=false, 0.05 초과 경고) |
| `reference.zeta` | double | – | 1.0 | v1: 1 만 허용 | 닫힌해(§4.4)·이산 경계(§4.7)가 1에서만 유효 — ≠1 이면 검증기가 armable=false (L0 §5.3) |
| `reference.a_max` | double | m/s² | `TBD` | >0 | TBD-ARM-02, 로봇·자세 의존 |
| `reference.v_max` | double | m/s | `TBD` | >0 | 로봇 TCP 속도 한계. L3 `gammaWindow` 는 $\eta_v\cdot$ 이 값을 쓴다 `[확정 D-9]` (L3 §4.5) |
| `reference.axis.k_axis` | double | 1/s | 8.0 | 1–30 | 튜닝. $\Vert\omega_{ref}\Vert=K_a\theta$ 이므로 $\theta=\pi$ 에서 $K_a\pi$ |
| `reference.axis.w_max` | double | rad/s | `TBD` | >0 | 손목 관절 한계에서 산정 |
| `reference.axis.sin_eps` | double | – | 1e-6 | 1e-9–1e-3 | 반평행 축 정의 하한 $\Vert z\times a_d\Vert$ (§4.5) |
| `reference.gamma_derate.ramp` | – | – | – | – | v1 범위 밖 (D-8, §5.2.1). 기본 0.05 s 는 가속 피크를 키웠다 — 재도입 시 단일 키로 다시 정한다 |
| `reference.retreat.omega` | double | rad/s | 3.0 | 0.5–10 | 튜닝 |
| `reference.retreat.home_pose` | pose | m, quat | `TBD` | – | 로봇별 |

## 7. 단위 기술 구현 순서

단계 매핑 (plan §4, §14.2): L4.1–L4.3·L4.5·L4.7 = **S1.4**, L4.4 = **S2.1** (`rtc_math` se3), CLIK 결합은 S2.2b·S5.3.

- **L4.1** `GammaProfile` + 미분 일치(유한차분) 테스트.
- **L4.2** `SoftCatchTranslation` + §4.9 1–4, 6 테스트 + NaN 가드 회귀 (§5.1).
- **L4.3** 재예측 점프 테스트: `setIntercept` 교체 직후 $e,\dot e$ 변화가 §4.3 식과 일치.
- **L4.4** `axisAlignError`/`Omega`/`Jacobian` 을 `rtc_math` se3 로 이식 + 연속성 sweep + 반평행 + Jacobian 유한차분 + 데드밴드·반평행 유한성 (L3·L5와 공유).
- **L4.5** 이산 안정 경계 테스트 ($s=0.8$ 수렴, $s=0.85$ 발산), 100·500·5000 Hz 의 $h$ 로.
- **L4.6** (v1 범위 밖, D-8) `derateGamma` + §5.2.1 연속성 테스트.
- **L4.7** 복귀 기준.

참조 구현: `soft_catch_reference.hpp`, `test_l4.cpp` (같은 폴더). S1.1 에서 GTest로 옮긴다.

## 8. 디버깅 방법

- 기록 항목(틱마다): $t$, $\gamma,\dot\gamma,\ddot\gamma$, $e,\dot e$, $\Vert u\Vert$, feedforward 성분별 크기, 포화 플래그, 포구점 교체 이벤트.
- 간극이 $(1-\gamma)\delta$보다 크다: 포화 여부 → `t` 시간축 불일치(γ 프로파일과 대상 상태의 시각 차) → $\ddot\xi^O$ 부호 순으로 확인.
- 상대속도가 기대와 다르다: $t_c$에서 $\xi^O\neq0$인지(포구점과 실제 도달 시각 불일치) 확인.
- 접근축이 진동한다: `k_axis`가 L5 CLIK 대역보다 큰지, `w_max` 포화 여부를 확인.
- 접근축이 큰 오차에서 안 움직인다: v0.1의 $\sin$ 기반 오차가 남아 있는지 확인(§4.5). 회전벡터 기반이면 $\Vert\omega_{ref}\Vert=K_a\theta$ 라 오차가 클수록 빠르다.
- 발산: `omega × dt`가 0.828을 넘는지 확인.
- 포화 중 기준이 이상하다: `xdd`(실현)와 `u_des`(요구)를 함께 기록해 어느 쪽을 쓰고 있는지 확인(§5.2).

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G4-A | §4.9 표 재현 (간극 오차 < $\epsilon_{conv}$ + $(1-\gamma)\delta$의 5%, 상대속도 오차 < 2%). $\epsilon_{conv}=2$ mm는 $\omega=10$, 비행 0.8 s의 DS 잔여 수렴 오차다 | `[SIM-ANY]` |
| G4-B | 재예측 점프 식 일치 (< 1e-9) | `[SIM-ANY]` |
| G4-C | 이산 안정 경계 테스트 통과 ($s=0.80$ 수렴, $s=0.85$ 발산) | `[SIM-ANY]` |
| G4-D | 축 정렬: $\exp([e_a]_\times)z=a_d$ 잔차 < 1e-12, 1° 격자 $\Vert\omega_{ref}\Vert$ 변화 < $1.5K_a\pi/180$, Jacobian 유한차분 오차 < 1e-5 (1–170°), **정렬·반평행 데드밴드와 그 근처에서 출력 전부 유한** (S2.1) | `[SIM-ANY]` |
| G4-E | v1 범위 밖 (D-8). 재도입 시: `derateGamma` 후 $\gamma$, $e$ 연속(< 1e-12), $\dot e$ 점프 $=\vert\dot\gamma\vert\Vert\xi^O\Vert$ (< 1e-9), 상향 거부 + §5.2.1 재설계 요구사항 (γ_min clamp, 램프 끝 ≤ $t_c$, forward rollout 수락) | – |
| G4-F | 속도 포화 시 `xdd` == 실현 가속도 (< 1e-9) | `[SIM-ANY]` |
| G4-G | 할당 0, `noexcept`, 틱당 최악 실행시간 기록 | `[SIM-ANY]` |
| G4-H | MuJoCo에서 L5와 결합 후 catch frame 실제 궤적이 기준을 추종 (추종 오차 기록) | `[SIM-P1B]` |
| G4-I | NaN 가드: 비유한 목표·`t`·`dt` 입력 시 내부 상태 보존 + invalid, 다음 유한 입력에서 정상 출력, NaN 에서 포화 검출 참 (S1.4) | `[SIM-ANY]` |

`test_l4.cpp`가 G4-A~F를 전부 돌린다(v0.2 기준 통과). S1 이식본은 G4-E 를 빼고 G4-I 와 G4-D 유한성 항목을 더한다.

## 10. 미확정 항목

TBD-ARM-02, TBD-REF-01(§4.6 [R4]/[R5] 재확인), `reference.v_max`, `reference.axis.w_max`, `reference.retreat.home_pose`, CLIK 공급 성분 배치 (S2.2b). 축 정렬 Jacobian 의 데드밴드 처리는 닫힘 (S2.1, §4.5). TBD-RTC-07·TBD-RTC-08·TBD-FRAME-01 은 닫힘 (§2), γ derate 는 v1 범위 밖 (D-8).
