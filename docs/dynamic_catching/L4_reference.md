# L4 — Reference: soft-catch DS, 접근축 정렬, 복귀

- 브랜치: `feat/catching-L4-reference`
- 패키지: `catching_reference`
- 선행: 단계 W, L0 (L2 궤적 샘플러 `traj::sampleAt()` 출력을 입력으로 받음)
- 산출물: `soft_catch_reference.hpp`, `retreat_reference.hpp`, `convergence_bound.hpp`
- 비고: L3가 γ rollout에서 **이 layer의 같은 코드**를 호출한다.

---

## 1. 범위 / 비범위

범위: 추종 대상(실제 공 또는 L7의 가상 감속 공)의 상태로부터 catch frame의 병진 기준 $(x,\dot x,\ddot x)$와 회전 기준 $\omega_{ref}$를 500 Hz로 생성한다. 복귀(retreat) 기준도 포함한다.

비범위: 관절 공간 변환과 한계 처리(L5), γ·포구점 결정(L3), 모드 전환 판단(L7).

## 2. 코드 확인 게이트

| ID | 확인 항목 | 기록 |
|---|---|---|
| G4-1 | `rtc_tsid` / CLIK가 받는 과제 기준의 형식 (위치+속도+가속도? 속도만?) | TBD-RTC-07 |
| G4-2 | 기존 SE(3)/SO(3) 오차 헬퍼(U1 공유 헬퍼)의 규약과 본 문서 §4.5 축 정렬 오차의 공존 방식 | TBD-RTC-08 |
| G4-3 | 두 로봇의 catch frame과 손바닥 바깥 법선 축 | TBD-FRAME-01 |

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

### 4.8 포화

$\Vert u\Vert\le a_{\max}$, $\Vert\dot x\Vert\le v_{\max}$로 포화시키고 플래그를 올린다. 포화 중에는 §4.1의 오차계가 성립하지 않는다.

soft catch에서는 포화가 포구 직전에 몰려 치명적이다. 지난 분석의 수치 예($t_c=0.8$ s, 예측 오차 3 cm)에서 간극은 다음과 같았다.

| 조건 | 간극 |
|---|---|
| 포화 없음, $\gamma_f=0.4$ | 17.7 mm |
| $a_{\max}=15$, $\gamma_f=0.4$ | 97.4 mm |
| $a_{\max}=15$, $\gamma_f=0$ | 29.7 mm |

따라서 L3가 rollout으로 포화 없는 $(\gamma_f,T_w)$를 선택하고(L3 §4.8), 실행 중 포화가 나면 L7이 **먼저 γ를 하향**하고(§5.1 `derateGamma`, L7 §4.6), 그래도 해소되지 않을 때만 abort한다. abort는 $t_c$ 직전에 가장 나쁜 선택이므로 마지막 수단으로 둔다.

### 4.9 Sanity check

1. $\gamma_f=0$, 예측 정확: 간극 ≈ 0, 상대속도 ≈ $\Vert v_O\Vert$.
2. 예측 오차 $\delta$: 간극 ≈ $(1-\gamma)\Vert\delta\Vert$.
3. 상대속도 ≈ $(1-\gamma)\Vert v_O(t_c)\Vert$.
4. 닫힌해와 미세 스텝 적분 일치.
5. 축 정렬: $\Vert\omega_{ref}\Vert$ 가 정렬 오차에 대해 연속, $\exp([e_a]_\times)z=a_d$, Jacobian이 유한차분과 일치.
6. (회귀) 속도 포화 시 반환 `xdd`가 실현 가속도와 일치(§5.1).
7. (회귀) `derateGamma` 후 $\gamma$, $e$ 연속, $\dot e$ 점프 $=\vert\dot\gamma\vert\Vert\xi^O\Vert$.

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

```cpp
#pragma once
// catching_reference/soft_catch_reference.hpp — RT-safe, 고정 크기, noexcept.
//
// L3(계획 rollout), L4(기준 생성), L5(접근축 과제)가 모두 이 헤더를 쓴다.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace catching::ref {

struct TargetState {            // 추종 대상 (vision 예측 보간값 또는 L7 가상 감속 공), W 기준
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};   // [m]
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};   // [m/s]
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};   // [m/s^2]
};

// gamma(t): [t0, t1]에서 g0 → gf 5차 보간. 양 끝 1·2계 미분 0 → clamp와 일관.
struct GammaProfile {
  double g0{0.0}, gf{0.0}, t0{0.0}, t1{1.0};   // t 는 선행축 상대시간 [s] (L4 §5.2)
  void eval(double t, double& g, double& gd, double& gdd) const noexcept {
    const double T = std::max(t1 - t0, 1e-6);
    const double s = std::clamp((t - t0) / T, 0.0, 1.0);
    const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
    const double d = gf - g0;
    g   = g0 + d * (10.0 * s3 - 15.0 * s4 + 6.0 * s5);
    gd  = d * (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / T;
    gdd = d * (60.0 * s - 180.0 * s2 + 120.0 * s3) / (T * T);
  }
};

// 시간축 규약 (L4 §5.2):
//   x, xd        : t + dt 기준 (다음 틱에 내보낼 기준 상태)
//   xdd          : [t, t+dt] 구간에 **실제 실현된** 평균 가속도 (속도 포화 반영)
//   u_des        : 포화 전 DS 요구 가속도 (진단, L3 rollout 판정)
//   e, ed        : t 기준 오차 (진단)
struct TranslationOutput {
  Eigen::Vector3d x{Eigen::Vector3d::Zero()}, xd{Eigen::Vector3d::Zero()},
                  xdd{Eigen::Vector3d::Zero()};
  Eigen::Vector3d u_des{Eigen::Vector3d::Zero()};
  Eigen::Vector3d e{Eigen::Vector3d::Zero()}, ed{Eigen::Vector3d::Zero()};
  double gamma{0.0}, gamma_d{0.0}, gamma_dd{0.0};   // L8 TickRecord 기록용 (L4 §8)
  bool saturated{false};
};

// derateGamma() 의 결과. L7 은 Rejected 만 "여유 없음"으로 해석한다 (L7 §4.6).
//   Applied : 목표를 gf_new 로 낮췄다.
//   Frozen  : 현재 gamma(t) 가 이미 gf_new 이하라 **현재 값에서 동결**했다.
//             gammadot, gammaddot 가 0 이 되므로 feedforward 의 지배항이 사라진다 —
//             램프 중앙에서는 이것만으로도 |u| 가 크게 준다.
//   Rejected: 상향 요청이거나 목표가 이미 그 값 이하다.
enum class DerateResult : std::uint8_t { Rejected = 0, Frozen = 1, Applied = 2 };

class SoftCatchTranslation {
 public:
  struct Params {
    double omega{10.0};   // [rad/s]  (omega*dt <= 0.828 이 이산 안정 경계, L4 §4.7)
    double zeta{1.0};
    double a_max{15.0};   // [m/s^2]  L7 supervisor.decel.a_dec <= 이 값 이어야 한다 (L7 §4.3)
    double v_max{2.0};    // [m/s]    L3 gammaWindow 의 v_tcp_max 와 같은 값이어야 한다
  };
  explicit SoftCatchTranslation(const Params& p) noexcept : prm_(p) {}

  // 활성화·재무장 시 호출. 기준 상태뿐 아니라 **포구점과 gamma 프로파일까지** 초기화한다.
  // gamma == 0 이면 끌개는 p_c_ 이므로(아래 주의), p_c_ = x 로 두어 현재 자세 유지가 된다.
  // v0.3 이전에는 p_c_ 와 gp_ 가 남아 직전 시행의 포구점으로 복귀하는 결함이 있었다.
  void reset(const Eigen::Vector3d& x, const Eigen::Vector3d& xd) noexcept {
    x_ = x; xd_ = xd; p_c_ = x; gp_ = GammaProfile{};
  }

  // **주의: gamma == 0 이면 대상 o 는 결과에 전혀 영향을 주지 않는다.**
  //   u = -w^2 (x - p_c) - 2 zeta w xd  → 끌개는 오직 p_c_ 다.
  // 따라서 정지 목표(홈 복귀, 대기 자세 유지)는 **p_c 로** 지정해야 한다 (L4 §5.3).
  void setIntercept(const Eigen::Vector3d& p_c, const GammaProfile& gp) noexcept {
    p_c_ = p_c; gp_ = gp;
  }
  [[nodiscard]] const Eigen::Vector3d& intercept() const noexcept { return p_c_; }
  [[nodiscard]] const GammaProfile& gamma() const noexcept { return gp_; }

  // 지금 하향하면 ed 가 얼마나 점프하는지 (L7 §4.6 사전 검사).
  //   ||Δed|| = |gammadot(t)| * ||xi^O(t)||
  // 이 값은 t 에 대해 **단조가 아니다** — gammadot 이 램프 중앙에서 최대이므로 봉우리가 있다.
  [[nodiscard]] double derateJump(const TargetState& o, double t) const noexcept {
    double g{}, gd{}, gdd{};
    gp_.eval(t, g, gd, gdd);
    return std::abs(gd) * (o.p - p_c_).norm();
  }

  // COMMITTED 이후에 허용되는 유일한 계획 변경 (L3 §4.7, L7 §4.6).
  // p_c 는 동결한 채 gamma 목표만 **하향**한다. 현재 gamma 값에서 다시 5차 램프를
  // 시작하므로 gamma(t) 는 연속이고 e 도 연속이다. ed 는 derateJump() 만큼 점프한다.
  //
  // 비교 기준은 **현재 값 gamma(t) 가 아니라 목표 gp_.gf** 다. 현재 값을 기준으로
  // 삼으면 램프 상승 구간(gamma(t) << gf)에서 요청이 전부 거부되는데, |u| 가 최대인
  // 곳이 바로 그 구간이다.
  DerateResult derateGamma(double t, double gf_new, double t_ramp) noexcept {
    if (!(gf_new < gp_.gf)) return DerateResult::Rejected;
    double g{}, gd{}, gdd{};
    gp_.eval(t, g, gd, gdd);
    const double T = std::max(t_ramp, 1e-3);
    if (!(gf_new < g)) {                       // 현재 값이 이미 새 목표 이하 → 동결
      gp_ = GammaProfile{g, g, t, t + T};
      return DerateResult::Frozen;
    }
    gp_ = GammaProfile{g, gf_new, t, t + T};
    return DerateResult::Applied;
  }

  // t: gamma profile 과 같은 시간축(선행축) [s], dt: 제어 주기 [s]
  [[nodiscard]] TranslationOutput step(const TargetState& o, double t, double dt) noexcept {
    const Eigen::Vector3d xo = o.p - p_c_;           // 원점 = 포구점
    double g{}, gd{}, gdd{};
    gp_.eval(t, g, gd, gdd);
    const Eigen::Vector3d e  = (x_ - p_c_) - g * xo;
    const Eigen::Vector3d ed = xd_ - (g * o.v + gd * xo);
    const double w = prm_.omega;
    const Eigen::Vector3d u_des = g * o.a + 2.0 * gd * o.v + gdd * xo      // feedforward
                                - w * w * e - 2.0 * prm_.zeta * w * ed;   // e'' = A1 e + A2 e'

    bool sat = false;
    Eigen::Vector3d u = u_des;
    if (const double un = u.norm(); un > prm_.a_max) { u *= prm_.a_max / un; sat = true; }

    const Eigen::Vector3d xd_prev = xd_;
    xd_ += u * dt;                                                        // semi-implicit Euler
    if (const double vn = xd_.norm(); vn > prm_.v_max) { xd_ *= prm_.v_max / vn; sat = true; }
    x_ += xd_ * dt;

    // 속도 포화가 걸리면 u 는 더 이상 실현 가속도가 아니다. 실제 실현값을 돌려준다.
    const Eigen::Vector3d xdd = (dt > 0.0) ? Eigen::Vector3d((xd_ - xd_prev) / dt) : u;
    return {x_, xd_, xdd, u_des, e, ed, g, gd, gdd, sat};
  }

 private:
  Params prm_;
  Eigen::Vector3d x_{Eigen::Vector3d::Zero()}, xd_{Eigen::Vector3d::Zero()},
                  p_c_{Eigen::Vector3d::Zero()};
  GammaProfile gp_{};
};

// ---------------------------------------------------------------------------
// 접근축 정렬 (L4 §4.5). L3 §4.2(IK), L5 §4.2(과제)도 같은 함수를 쓴다.
//
// 오차는 **회전벡터** e_a = theta * u_hat 이다 (u_hat = (z x a_d)/||z x a_d||).
// exp([e_a]x) z == a_d 를 정확히 만족하고, ||e_a|| = theta 라 0~pi 에서 연속·단조다.
// ---------------------------------------------------------------------------
struct AxisAlignParams {
  double k_axis{8.0};          // [1/s]
  double w_max{6.0};           // [rad/s]
  double sin_eps{1e-6};        // 축이 수치적으로 정의되는 하한 (|z x a_d|)
};

// z, a_d 는 단위벡터여야 한다.
[[nodiscard]] inline Eigen::Vector3d axisAlignError(const Eigen::Vector3d& z,
                                                    const Eigen::Vector3d& a_d,
                                                    double sin_eps = 1e-6) noexcept {
  const Eigen::Vector3d m = z.cross(a_d);
  const double n = m.norm();
  const double c = z.dot(a_d);
  if (n < sin_eps) {
    if (c > 0.0) return Eigen::Vector3d::Zero();                 // 이미 정렬 (데드밴드)
    // 반평행: 축이 정의되지 않음 → z 에 수직인 임의 축으로 pi 회전
    const Eigen::Vector3d r = (std::abs(z.x()) < 0.9) ? Eigen::Vector3d::UnitX()
                                                      : Eigen::Vector3d::UnitY();
    return M_PI * z.cross(r).normalized();
  }
  return (std::atan2(n, c) / n) * m;
}

[[nodiscard]] inline Eigen::Vector3d axisAlignOmega(const Eigen::Vector3d& z,
                                                    const Eigen::Vector3d& a_d,
                                                    const AxisAlignParams& p) noexcept {
  Eigen::Vector3d w = p.k_axis * axisAlignError(z, a_d, p.sin_eps);
  if (const double n = w.norm(); n > p.w_max) w *= p.w_max / n;
  return w;                                                       // W 기준, z 에 수직
}

// de_a/dt = J_a * omega  (omega 는 W 표현). L5 에서 J_a = axisAlignJacobian(z,a_d) * J_omega^W.
//
//   m = z x a_d,  c = z^T a_d,  theta = atan2(|m|, c),  f = theta/sin(theta)
//   J_a = f'(c) m m^T + f(c) [a_d]x [z]x ,   f'(c) = (theta*c/sin(theta) - 1) / sin^2(theta)
//
// **sin_eps 는 axisAlignError 와 같은 값을 넘겨야 한다.** 다르면 J_a 가 e_a 의 야코비안이
// 아니게 된다(데드밴드 안에서 e_a 는 상수인데 J_a 는 0 이 아닌 값을 돌려준다).
//
// 소각도 급수는 **c > 0 일 때만** 쓴다. sin(theta) 는 theta->0 과 theta->pi 양쪽에서 0 이라,
// 부호를 보지 않으면 반평행 근처에서 발산해야 할 값이 유한한 값(~2.645)으로 조용히 바뀐다.
// theta->pi 에서의 발산은 축이 정의되지 않기 때문이며, 정의의 결함이 아니라 문제의 성질이다.
[[nodiscard]] inline Eigen::Matrix3d axisAlignJacobian(const Eigen::Vector3d& z,
                                                       const Eigen::Vector3d& a_d,
                                                       double sin_eps = 1e-6) noexcept {
  const Eigen::Vector3d m = z.cross(a_d);
  const double n = m.norm();
  const double c = std::clamp(z.dot(a_d), -1.0, 1.0);
  const double th = std::atan2(n, c);
  if (n < sin_eps && c > 0.0) return Eigen::Matrix3d::Zero();     // e_a 데드밴드 → de_a = 0
  double f, fp;
  if (n < 1e-8 && c > 0.0) {            // theta -> 0 급수: f = 1 + th^2/6, f' = -1/3
    f  = 1.0 + th * th / 6.0;
    fp = -1.0 / 3.0;
  } else {
    f  = th / n;                        // c < 0 이고 n -> 0 이면 발산 — 의도된 동작
    fp = (th * c / n - 1.0) / (n * n);
  }
  const Eigen::Matrix3d Sa = (Eigen::Matrix3d() <<     0.0, -a_d.z(),  a_d.y(),
                                                   a_d.z(),      0.0, -a_d.x(),
                                                  -a_d.y(),  a_d.x(),      0.0).finished();
  const Eigen::Matrix3d Sz = (Eigen::Matrix3d() <<   0.0, -z.z(),  z.y(),
                                                   z.z(),    0.0, -z.x(),
                                                  -z.y(),  z.x(),    0.0).finished();
  return fp * (m * m.transpose()) + f * (Sa * Sz);
}

// 임계감쇠(zeta=1) 오차의 닫힌해: 계획 단계의 종단 오차 예측용 (L3)
inline void criticallyDampedError(const Eigen::Vector3d& e0, const Eigen::Vector3d& ed0,
                                  double omega, double t,
                                  Eigen::Vector3d& e, Eigen::Vector3d& ed) noexcept {
  const Eigen::Vector3d c = ed0 + omega * e0;
  const double ex = std::exp(-omega * t);
  e  = (e0 + c * t) * ex;
  ed = (ed0 - omega * t * c) * ex;
}

}  // namespace catching::ref
```

### 5.2 사용 규약

- 시간축: `t`는 계획 기준 상대시간(`PlanSnapshot`의 기준 시각에서 잰 값)을 쓴다. `GammaProfile`의 `t0`, `t1`도 같은 기준이다.
- `setIntercept()`는 L7이 `COMMITTED` 이전에만 호출한다. `COMMITTED` 이후 허용되는 유일한 계획 변경은 `derateGamma()`다(아래).
- **반환값의 시간축이 섞여 있다.** `x`, `xd`는 $t+\Delta t$ 기준(다음 틱 명령), `xdd`는 $[t,t+\Delta t]$ 구간의 실현 가속도, `e`, `ed`는 $t$ 기준 진단값이다. L8 `TickRecord`에 함께 기록할 때 1틱 오정렬을 감안한다.
- **`xdd` vs `u_des`.** 속도 포화가 걸리면 DS가 요구한 가속도 `u_des`는 실현되지 않는다. CLIK feedforward(L5)에는 `xdd`를, L3 rollout 판정과 포화 진단에는 `u_des`를 쓴다. v0.1은 포화 후에도 `u_des`를 `xdd`로 돌려주어, 예를 들어 $v_{max}=0.5$ m/s 조건에서 실현 5.0 m/s² 대신 490 m/s²를 보고했다.
- 감속 모드(L7): 대상에 가상 감속 공을 넣고 `GammaProfile{1,1,…}`(상수 1)로 바꾼다. 전환 시각이 $t_c$이면 $\xi^O(t_c)\approx0$이라 오차 점프가 작다(§4.3). 가상 공의 초기 속도를 전환 시점의 기준 속도로 두면 $\dot e$도 연속이다(L7 §4.3).
- 회전: `axisAlignOmega(z_cmd, a_d, p)`의 `z_cmd`는 현재 명령 자세(CLIK 내부 상태) 기준이다. 측정 자세 사용 여부는 G4-1 결과에 맞춘다.

### 5.2.1 `derateGamma()` — 동결 후 γ 하향 `[권장]`

`COMMITTED` 이후 포화가 예상되면 plan 전체를 버리는 대신 $\gamma_f$ 만 낮춘다(L7 §4.6). $p_c$ 와 $t_c$ 는 동결 상태를 유지한다.

**비교 기준은 현재 값 $\gamma(t)$ 가 아니라 목표 $\gamma_f$ 다.** 현재 값을 기준으로 삼으면 램프 상승 구간에서 요청이 전부 거부된다 — $\gamma(t)\ll\gamma_f$ 이므로 `gf_new = γ_f − step` 이 거의 항상 $\gamma(t)$ 보다 크기 때문이다. 실측(γ_f=0.4, 램프 [0, 0.45] s, step=0.1): $t=0.05\sim0.25$ 에서 전부 거부, $t\ge0.30$ 부터 수락. 그런데 $\Vert u\Vert$ 최대는 $\dot\gamma,\ddot\gamma$ 가 큰 **바로 그 앞 구간**에 몰린다. 즉 기준을 잘못 잡으면 하향 경로가 필요한 순간에만 정확히 무력해진다.

반환은 세 값이다.

| 결과 | 조건 | 동작 |
|---|---|---|
| `Applied` | $\gamma_f^{new}<\gamma(t)$ | 목표를 낮추고 현재 값에서 새 램프 시작 |
| `Frozen` | $\gamma_f^{new}\ge\gamma(t)$ 이지만 $\gamma_f^{new}<\gamma_f$ | **현재 값에서 동결** ($\gamma_0=\gamma_f=\gamma(t)$) |
| `Rejected` | $\gamma_f^{new}\ge\gamma_f$ (상향) | 무시 |

`Frozen` 도 유효한 완화다. $\dot\gamma,\ddot\gamma$ 가 0이 되면 feedforward 의 $2\dot\gamma\dot\xi^O+\ddot\gamma\xi^O$ 항이 사라지는데, 램프 중앙에서는 이것이 $\Vert u\Vert$ 의 지배항이다. **L7 은 `Rejected` 만 "여유 없음"으로 해석해야 한다** — `false` 하나로 뭉뚱그리면 `Frozen` 이 abort 트리거가 된다.

연속성:

- $\gamma(t)$ 연속 → $e=(x-p_c)-\gamma\xi^O$ **연속** (수치 확인: $\Delta e=0$)
- $\dot\gamma$ 가 0으로 점프 → $\dot e$ 가 $\vert\dot\gamma(t)\vert\,\Vert\xi^O(t)\Vert$ 만큼 점프

**이 점프는 시간에 대해 단조가 아니다.** v0.2는 "늦게 할수록 점프가 작아진다"고 적었으나 틀렸다. $\dot\gamma$ 는 5차 램프의 **중앙에서 최대**($1.875\,\Delta\gamma/T$)이고 $\Vert\xi^O\Vert$ 만 단조 감소하므로, 곱은 봉우리를 만든다. 실측(램프 [0.4, 0.8] s, $\gamma_f=0.4$, $t_c=0.8$):

| $t$ [s] | 0.45 | 0.50 | **0.56** | 0.60 | 0.70 | 0.76 | 0.79 |
|---|---|---|---|---|---|---|---|
| $\vert\dot\gamma\vert$ | 0.36 | 1.06 | 1.83 | 1.88 | 1.06 | 0.29 | 0.02 |
| $\Vert\xi^O\Vert$ | 1.61 | 1.42 | 1.11 | 1.01 | 0.53 | 0.19 | 0.06 |
| 점프 | 0.58 | 1.50 | **2.04** | 1.88 | 0.56 | 0.05 | 0.001 |

맞는 것은 극한 주장뿐이다: $t\to t_c$ 에서 $\dot\gamma\to0$, $\xi^O\to0$ 이므로 점프 $\to0$. 따라서 L7 은 하향 전에 `derateJump(o, t)` 로 점프를 **미리 계산해** `ed_jump_max` 와 비교하고, 초과하면 `Frozen` 으로 대신하거나 다음 틱으로 미룬다(L7 §4.6).

필요 가속도는 $\gamma_f$ 에 거의 선형으로 줄어든다(§L3 4.8 표: $T_w=0.30$ s에서 $\gamma_f$ 0.4→0.2가 33.0→16.4 m/s²).

### 5.3 복귀 기준 `retreat_reference.hpp`

**주의: $\gamma\equiv0$ 이면 대상 $o$ 는 결과에 전혀 영향을 주지 않는다.**

$$u=-\omega^2(x-p_c)-2\zeta\omega\,\dot x$$

가 되어 $o.p,\,o.v,\,o.a$ 가 모두 소거되고, 끌개는 오직 $p_c$ 다. 따라서 정지 목표는 **$p_c$ 로** 지정해야 한다.

$$\texttt{setIntercept}(p_{home},\ \texttt{GammaProfile}\{0,0,\cdot,\cdot\})$$

v0.2는 "대상을 홈 위치로, γ를 0으로 넣어 재사용"이라고 적었는데 이것은 **no-op** 이다. 실측하면 대상을 홈으로 주든 임의의 점으로 주든 똑같이 **직전 포구점**으로 수렴한다. 그러면 L7 §4.5 조건 4(대기 자세 허용오차)가 영원히 거짓이라 `RETREAT → ARMED` 전이가 막힌다. `test_l4.cpp` 의 `A5` 가 회귀 검사한다.

같은 이유로 `reset(x, xd)` 는 $p_c\leftarrow x$, $\gamma$ 프로파일 초기화까지 수행한다 — 그래야 활성화 직후 현재 자세 유지가 되고, 재무장 시 직전 시행의 포구점·하향된 γ가 남지 않는다(L7 §4.8 재무장 리셋 목록).

회전은 홈 자세로의 SO(3) 오차(기존 U1 헬퍼)를 쓴다(G4-2).

## 6. YAML 파라미터

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `reference.omega` | double | rad/s | 10.0 | 1–25 | §4.7 ($h=2$ ms에서 $s\le0.05$) |
| `reference.zeta` | double | – | 1.0 | 0.7–1.5 | 닫힌해는 1에서만 유효 |
| `reference.a_max` | double | m/s² | `TBD` | >0 | TBD-ARM-02, 로봇·자세 의존 |
| `reference.v_max` | double | m/s | `TBD` | >0 | 로봇 TCP 속도 한계. **L3 `gammaWindow`의 `v_tcp_max`와 같은 값이어야 한다** |
| `reference.axis.k_axis` | double | 1/s | 8.0 | 1–30 | 튜닝. $\Vert\omega_{ref}\Vert=K_a\theta$ 이므로 $\theta=\pi$ 에서 $K_a\pi$ |
| `reference.axis.w_max` | double | rad/s | `TBD` | >0 | 손목 관절 한계에서 산정 |
| `reference.axis.sin_eps` | double | – | 1e-6 | 1e-9–1e-3 | 반평행 축 정의 하한 $\Vert z\times a_d\Vert$ (§4.5) |
| `reference.gamma_derate.ramp` | double | s | 0.05 | 0.01–0.3 | §5.2.1 하향 램프 길이 |
| `reference.retreat.omega` | double | rad/s | 3.0 | 0.5–10 | 튜닝 |
| `reference.retreat.home_pose` | pose | m, quat | `TBD` | – | 로봇별 |

## 7. 단위 기술 구현 순서

- **L4.1** `GammaProfile` + 미분 일치(유한차분) 테스트.
- **L4.2** `SoftCatchTranslation` + §4.9 1–4, 6 테스트.
- **L4.3** 재예측 점프 테스트: `setIntercept` 교체 직후 $e,\dot e$ 변화가 §4.3 식과 일치.
- **L4.4** `axisAlignError`/`Omega`/`Jacobian` + 연속성 sweep + 반평행 + Jacobian 유한차분(L3·L5와 공유).
- **L4.5** 이산 안정 경계 테스트 ($s=0.8$ 수렴, $s=0.85$ 발산).
- **L4.6** `derateGamma` + §5.2.1 연속성 테스트.
- **L4.7** 복귀 기준.

참조 구현: `soft_catch_reference.hpp`, `test_l4.cpp` (같은 폴더). ROS 2 패키지로 이식할 때 GTest로 옮긴다.

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
| G4-D | 축 정렬: $\exp([e_a]_\times)z=a_d$ 잔차 < 1e-12, 1° 격자 $\Vert\omega_{ref}\Vert$ 변화 < $1.5K_a\pi/180$, Jacobian 유한차분 오차 < 1e-5 (1–170°) | `[SIM-ANY]` |
| G4-E | `derateGamma` 후 $\gamma$, $e$ 연속(< 1e-12), $\dot e$ 점프 $=\vert\dot\gamma\vert\Vert\xi^O\Vert$ (< 1e-9), 상향 거부 | `[SIM-ANY]` |
| G4-F | 속도 포화 시 `xdd` == 실현 가속도 (< 1e-9) | `[SIM-ANY]` |
| G4-G | 할당 0, `noexcept`, 틱당 최악 실행시간 기록 | `[SIM-ANY]` |
| G4-H | MuJoCo에서 L5와 결합 후 catch frame 실제 궤적이 기준을 추종 (추종 오차 기록) | `[SIM-P1B]` |

`test_l4.cpp`가 G4-A~F를 전부 돌린다(v0.2 기준 통과).

## 10. 미확정 항목

TBD-RTC-07, TBD-RTC-08, TBD-FRAME-01, TBD-ARM-02, TBD-REF-01(§4.6 [R4]/[R5] 재확인), `reference.v_max`, `reference.axis.w_max`, `reference.retreat.home_pose`.
