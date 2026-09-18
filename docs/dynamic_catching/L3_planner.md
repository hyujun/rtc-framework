# L3 — Planner: 포구 시각·포구점·접근축·γ 결정

- 브랜치: `feat/catching-L3-planner`
- 패키지: `catching_planner`
- 선행: 단계 W, L1, L2, L4 (γ rollout에 L4 코드 재사용), L5 (한계·`T_arm` 값)
- 산출물: `catch_planner.{hpp,cpp}`, `time_feasibility.hpp`, `catch_ik.hpp`, `directional_speed.hpp`, `PlanSnapshot`

---

## 1. 범위 / 비범위

범위: 새 예측 궤적 메시지마다(non-RT 스레드) 다음을 결정해 `PlanSnapshot`으로 발행한다. 후보 시각은 **vision이 준 샘플 격자**에서 고른다 — 제어 PC가 궤적을 만들지 않는다(마스터 §5.2).
- 포구 시각 $t_c$, 포구점 $p_c$, 목표 접근축 $a_d$
- IK 해 $q^\ast$ (L5의 posture 참고용)
- γ 프로파일 $(\gamma_f,T_w)$, 손 폐쇄 명령 시각 $t_{cmd}$
- 유효성, commit 가능 여부

비범위: 실행(L4/L5), 모드 전환(L7).

## 2. 코드 확인 게이트

단계 W에서 처리한다. **기존 kinematics 구현을 재사용한다**(마스터 §1.2) — IK도 FK·Jacobian도 새로 만들지 않는다.

| ID | 확인 항목 | 기록 |
|---|---|---|
| G3-1 | 모델 로드 경로와 FK/Jacobian API (폐쇄 체인 손 포함 시 팔 부분만 쓰는 방법) | TBD-RTC-14 (W3-1~3) |
| G3-2 | RT → non-RT 상태 전달 경로(현재 $q_c,\dot q_c$, L4 기준 상태) — SeqLock 사용 가능 여부 | TBD-RTC-15 (W2-2) |
| G3-3 | non-RT 스레드 생성·우선순위 규약 | TBD-RTC-16 (W2-1) |
| G3-4 | 손별 포켓 유효 깊이 $d_{eff}$, 포획 반경 $r_{cap}$ | TBD-HAND-04 (W7-4) |
| G3-5 | 포구 허용 작업공간, 감속 여유 공간 | TBD-BALL-02 (W7-3) |
| G3-6 | vision 샘플 간격·지평·$N$ → 후보 격자 범위 | TBD-VIS-04 (W5-6) |
| G3-7 | **독립 IK/포즈 해석기가 있는지**와 그 API. 있으면 §4.2의 DLS를 새로 짜지 않고 자세 수락 조건·접근축 마스크만 위에 얹는다 | W3-11 |

## 3. 참고자료

[R1] 포구 제약과 관절 램프(도달시간 제약의 출처), [R2] 시간 슬라이스 탐색과 예측 중단 시점, [R3]/[R4] softness, [R8] 불확실성, [R9] kinematics, [R15] 부록 SQP.

## 4. 수학적 이론

### 4.1 결정 구조

후보 시각은 vision 샘플 격자에서 고른다. 각 샘플의 $t$ 는 메시지 `header.stamp` 기준이므로, 도달시간 제약(§4.3)에 쓸 때는 **메시지 나이를 뺀다**: $t_{k,now}=s.t-\text{age}$. 이것을 빼먹으면 나이만큼 도달시간 여유를 과대평가한다.

게이트 적용 순서는 §5.3 코드가 단일 출처이며, 연산량이 싼 것부터다.

$$\text{§4.4 불확실성}\to\text{§4.2 IK}\to\text{§4.3 도달시간}\to\text{§4.5 γ 창}\to\text{§4.9 정지거리}\to\text{§4.8 rollout}\to\text{§4.6 오차 예산}\to\text{L7 §4.7 충격량}$$

앞 단계에서 탈락하면 뒤 단계는 계산하지 않는다. 통과한 후보 중 §4.10 규칙으로 하나를 고르고, **§4.7 히스테리시스**로 현재 plan과 비교한다.

[R1]은 $(q_c,t_c)$를 동시에 푸는 NLP를 썼다. 본 구현은 6축 주 타깃의 연산 결정성을 위해 1차원 시간 탐색 + IK로 분해한다 `[권장]`. 7축용 NLP는 부록 A.

### 4.2 5-DoF 포구 자세와 IK

목표: $p_c=\hat p(t_k)$, $a_d=-\hat v(t_k)/\Vert\hat v(t_k)\Vert$.

catch frame LOCAL $z$가 손바닥 바깥 법선이라 하자(TBD-FRAME-01). $a^C=R_{WC}^\top a_d$ 로 두면 접근축 오차의 LOCAL 표현은 L4 §4.5의 회전벡터다.

$$e_a^C=R_{WC}^\top e_a=\theta\,\frac{\hat e_z\times a^C}{\Vert\hat e_z\times a^C\Vert},\qquad \theta=\mathrm{atan2}(\Vert\hat e_z\times a^C\Vert,\ a^C_z)$$

$e_a^C\perp\hat e_z$ 이므로 $z$ 성분이 0이고, $S=\begin{bmatrix}1&0&0\\0&1&0\end{bmatrix}$ 가 정보를 버리지 않는다.

**갱신식은 Gauss-Newton이 아니다** — 이 점을 v0.1은 잘못 적었다. 아래 $J$ 는 잔차 $r$ 의 야코비안 $\partial r/\partial q$ 가 **아니고**, 관절속도와 과제속도를 잇는 관계일 뿐이다.

$$J(q)=\begin{bmatrix}J_p\\S\,J^L_\omega\end{bmatrix},\qquad \Delta q=J^\top(JJ^\top+\lambda^2I)^{-1}\begin{bmatrix}-(p_C-p_c)\\ \rho\,S\,e_a^C\end{bmatrix}+\big(I-J^\dagger J\big)K_n(q_n-q)$$

- 위치 행: $J_p\dot q=v_p$ 에 대한 Newton 스텝. 잔차의 부호를 뒤집어 넣는다.
- 회전 행: $SJ^L_\omega\dot q=S\omega^L$ 이므로, $\omega^L=e_a^C$ 를 단위 시간 적용하면 $\exp([e_a]_\times)z=a_d$ 에 의해 **한 번에 정확히 정렬된다**(L4 §4.5). 즉 이 행은 1차 근사가 아니라 정확한 회전 갱신이고, 부호도 그래서 양수다.

두 블록은 단위가 다르다(m vs rad). $\rho$ [m/rad]는 그 스케일을 맞추는 특성길이로, 단일 $\lambda$ 아래 두 과제의 상대 가중을 결정한다. `planner.ik.rho`로 둔다. v0.1은 이 항이 없어 상대 스케일이 임의였다.

참 야코비안이 필요하면 L4 §4.5의 $J_a$ 를 쓴다($S[\hat e_z]_\times[a^C]_\times J_\omega^L$ 형태). 본 갱신식은 그것을 쓰지 않으므로 수렴률에 대한 Gauss-Newton 보장은 없다 — 수렴은 게이트 G3-G로 실측한다.

반복마다 관절 한계로 clamp하고, 반복 상한 $N_{IK}$와 허용오차로 종료한다. 이웃 슬라이스의 해를 warm start로 쓴다.

수락 조건: 위치 오차 < $\epsilon_p$, $\theta\le\alpha_{\max}$ ([R2]의 허용 콘과 같은 취지. $\theta$ 는 위 회전벡터의 크기라 $z^\top a_d\ge\cos\alpha_{\max}$ 와 동치이면서 큰 오차에서도 수치적으로 안정하다).

### 4.3 관절 도달시간 제약 ([R1] 출처, 닫힌해는 `[논문 외 유도]`)

[R1]은 관절 램프의 실현 가능성 $t\ge t_{\min,i}(q_i)$를 제약으로 썼다. 본 구현은 현재 명령 상태 $(q_{c,i},\dot q_{c,i})$에서 $(q^\ast_i,0)$까지의 최소시간을 닫힌해로 계산한다. 속도 한계 $\bar\omega$, 가속 한계 $\bar a$, 목표 방향 속도 성분 $w=s\,\dot q_{c,i}$, $D=|q^\ast_i-q_{c,i}|$:

- $w<0$ (반대 방향 이동 중): 정지 후 $D+w^2/2\bar a$를 정지 상태에서 이동.
- $w^2/2\bar a>D$ (지나침): $w/\bar a$ 후 $w^2/2\bar a-D$ 복귀.
- 삼각: $\omega_p=\sqrt{\bar aD+w^2/2}\le\bar\omega$이면 $t=(2\omega_p-w)/\bar a$.
- 사다리꼴: $t=\dfrac{\bar\omega-w}{\bar a}+\dfrac{\bar\omega}{\bar a}+\dfrac{D-\frac{\bar\omega^2-w^2}{2\bar a}-\frac{\bar\omega^2}{2\bar a}}{\bar\omega}$
- 정지 상태 이동 $T_{rest}(D)$: $\sqrt{\bar aD}\le\bar\omega$이면 $2\sqrt{D/\bar a}$, 아니면 $D/\bar\omega+\bar\omega/\bar a$.

유도 요지: 가속 구간 이동거리 $(\omega_p^2-w^2)/2\bar a$와 감속 구간 $\omega_p^2/2\bar a$의 합이 $D$.

**전제 $|w|\le\bar\omega$ 는 검사한다.** 초기 속도가 이미 속도 한계를 넘으면 최소시간 문제 자체가 정의되지 않는다 — 사다리꼴 분기의 $(\bar\omega-w)/\bar a$ 가 음수가 되어 물리적 의미가 없는 값이 조용히 나온다(예: $w_0=6$, $\bar\omega=\pi$, $\bar a=10$, $D=2$ → 0.92374 s, 그중 첫 구간이 $-0.2858$ s). 계획용 $\dot q_{\max}$(운용 여유율 적용값)와 CLIK 내부 한계가 다르거나, L5의 경계 충돌 규칙이 발동한 직후에 일어날 수 있다. `tMinChecked`가 clamp하고 플래그를 세우며, 플래그가 서면 해당 후보를 탈락시킨다.

검증: 무작위 40개 조건에서 속도·가속 제약 선형계획(시간 이분 탐색) 해와 최대 차이 — python 거울 $8.3\times10^{-6}$ s, C++ `tMin` $9.5\times10^{-6}$ s (LP 격자 이산화 수준). `test_l3.cpp`가 `cases.txt`를 만들고 `verify_l3.py`가 대조한다.

제약:

$$t_k-t_{now}-T_{arm}-T_{margin}\ \ge\ \max_i t_{\min,i}$$

**한계.** 이 조건은 **필요조건**이다. 실제 운동은 과제 공간 DS가 만들므로 관절별 시간최적 프로파일과 다르다. 충분성은 §4.8 rollout에서 확인한다.

### 4.4 불확실성 게이트

$$\sigma_{\max}(t_k)=\sqrt{\lambda_{\max}\big(\Sigma_{pp}(t_k)\big)}\ \le\ \kappa_\sigma\,r_{cap}$$

**$\Sigma_{pp}$는 vision이 준 값이다.** 각 샘플의 6×6 공분산에서 위치 3×3 블록을 꺼내 대칭화한 뒤 최대 고윳값을 쓴다(`Eigen::SelfAdjointEigenSolver::computeDirect`, 고정 크기·무할당). v0.2처럼 제어 PC가 $\Phi P\Phi^\top+Q$ 로 전파하지 않는다.

`PointCloud2`에는 트랙 상태(`STATUS_INITIALIZING` 등)가 없으므로(마스터 §5.1), "초기화 직후 트랙 탈락"은 다음으로 대체한다.

- L1의 트랙 epoch가 막 바뀐 직후 `n_settle` 개 메시지는 계획하지 않는다(L1 §4.4).
- 그리고 위 $\sigma_{\max}$ 게이트 자체가 초기 불확실성이 큰 구간을 걸러낸다 — vision의 공분산이 정직하다면 이 편이 상태 플래그보다 낫다.

vision 공분산의 신뢰성은 시뮬레이션에서 참값 대비 NEES로 확인한다(L8 §4.4). 일관적이지 않으면 $\kappa_\sigma$ 로 보정하고 그 사실을 기록한다.

### 4.5 γ 창 `[논문 외 유도]`

**하한 (손 폐쇄).** 상대속도 $(1-\gamma)\Vert v\Vert$로 포켓 유효 깊이 $d_{eff}$를 지나기 전에 손이 닫혀야 한다.

$$\gamma\ge\gamma_{\min}=1-\frac{d_{eff}}{\Vert v(t_k)\Vert\,T_{close,tot}},\qquad T_{close,tot}=T_{close}+T_{link}+T_{tick}$$

$d_{eff}$는 손바닥 접촉 전 폐쇄를 요구하면 포켓 깊이 $d$, 반발계수 $e$로 튕겨 나오기 전까지 허용하면 $d(1+1/e)$다(TBD-HAND-04).

**상한 (팔 속도).** 포구 자세 $q^\ast$에서 방향 $\hat v$로 낼 수 있는 최대 속력 $v_{dir,\max}$와 TCP 속도 한계 $v_{\max}$(= L4 `reference.v_max`)로 제한한다.

$$\gamma\le\gamma_{\max}=\frac{\min(v_{dir,\max},\ v_{\max})}{\Vert v\Vert}$$

$v_{\max}$ 를 빠뜨리면 계획이 통과시킨 γ가 L4에서 속도 포화를 일으킨다. `gammaWindow`는 두 값을 모두 인자로 받는다(v0.1 코드는 $v_{dir,\max}$만 썼다).

**여유율은 세 곳에서 같아야 한다 `[권장]`.** γ 창은 $v_{\max}$ 전체를, rollout 수락(§4.8)은 $\eta_vv_{\max}$ 를, L4 포화는 다시 $v_{\max}$ 를 쓰면, 창은 통과했는데 rollout 에서만 탈락하는 후보가 구조적으로 생긴다. `gammaWindow` 에 $\eta_vv_{\max}$ 를 넘긴다.

**입력 방어.** $v_{dir,\max}$ 는 아래 DLS 정규화식의 결과라 수치 문제로 음수가 나올 수 있다. 음수면 clamp 가 $\gamma_{\max}$ 를 0으로 **올려** 판정을 뒤집으므로, `gammaWindow` 가 비물리적 입력을 검사해 플래그를 세운다(`tMinChecked` 가 $|w_0|>\bar\omega$ 를 검사하는 것과 같은 수준).

**방향 속력 계산.** 접근축 각속도 0을 유지하며 $\hat v$ 방향 단위 속도를 내는 관절속도 $\dot q^u$를 damped least-squares로 구하고

$$v_{dir,\max}\approx\frac{\Vert J_p\dot q^u\Vert}{\displaystyle\max_i\frac{|\dot q^u_i|}{\dot q_{\max,i}}}$$

로 근사한다. **분자가 필요한 이유:** DLS($\lambda>0$)는 $J\dot q^u=[\hat v;0]$ 을 정확히 만족하지 않는다. 특이 자세 근처에서 $\Vert J_p\dot q^u\Vert<1$ 인데 v0.1은 분자를 1로 두고 $(\max_i|\dot q^u_i|/\dot q_{\max,i})^{-1}$ 만 썼다. 그러면 **실제로 낼 수 없는 속력을 보고한다** — 문서가 주장한 "보수적"의 반대다. 달성 속력으로 정규화하면 부호가 보장된다.

남는 보수성: 최소노름 해만 보므로 여유 자유도를 최대한 쓴 LP 최적값보다 작거나 같다. 6축 5-DoF 과제에서는 여유가 1자유도라 차이가 작다.

특이 자세 자체는 조작도 $\sqrt{\det J_pJ_p^\top}$ 로 따로 게이트한다(`planner.ik.manip_min`).

**Sanity check.** $v_{dir,\max}=0$(정지 포구)이면 $\Vert v\Vert\le d/T_{close,tot}$다. [R1]의 포켓 3 cm, 6 m/s를 넣으면 $T_{close}\le5$ ms다. [R1] 각주 2 원문으로 확인했다("Assuming that the ball flies within the hand about 0.03 m with a velocity of 6 m/s the time duration of 5 ms is obtained"). `test_l3.cpp`가 5 ms는 통과, 6 ms는 창이 빔을 검사한다.

**창이 비는 조건.** 받을 수 있는 최대 공 속력은

$$\Vert v\Vert_{\max}=\min(v_{dir,\max},v_{\max})+\frac{d_{eff}}{T_{close,tot}}$$

이고 이를 넘는 후보는 탈락이다(`maxCatchableSpeed`). **이 식이 시스템 전체의 실현 가능성을 결정한다** — 마스터 §4.1을 볼 것. $v_{dir,\max}=1.5$ m/s, $d_{eff}=4$ cm, $T_{close,tot}=60$ ms이면 상한이 2.17 m/s에 불과하다.

### 4.6 포구 오차 예산 `[논문 외 유도]`

commit 시 포구점 $p_c$가 고정되고, 포구 순간 기준은 $p_c+\gamma(\hat p_{live}(t_c)-p_c)$에 있다. 실제 공 위치를 $p_{true}$라 하면

$$\text{gap}=(1-\gamma)\big(p_{true}-p_c\big)+\gamma\big(p_{true}-\hat p_{live}(t_c)\big)+\varepsilon_{trk}+\varepsilon_{clk}$$

- 첫 항: commit 시점 예측 오차, $(1-\gamma)$배로 감쇠.
- 둘째 항: 포구 순간의 실시간 추정 오차(스냅샷 나이 포함).
- $\varepsilon_{trk}$: L5 추종·지연 잔차.
- $\varepsilon_{clk}\approx\Vert v\Vert\delta$: 시계 오차.

**첫 두 항은 독립이 아니다.** $\hat p_{live}$ 는 $p_c$ 보다 나중 정보를 쓴 추정이므로 두 오차가 강하게 상관돼 있다. 직교 분해로 정리한다. $A=p_{true}-\hat p_{live}$, $B=\hat p_{live}-p_c$ 로 두면

$$\text{gap}=(1-\gamma)(A+B)+\gamma A=A+(1-\gamma)B$$

최적 추정이면 혁신 $B$ 와 잔차 $A$ 가 직교하므로 $\sigma_c^2=\sigma_\ell^2+\sigma_B^2$ 이고,

$$\boxed{\sigma_{gap}^2=\sigma_\ell^2+(1-\gamma)^2(\sigma_c^2-\sigma_\ell^2)=(1-\gamma)^2\sigma_c^2+\big(1-(1-\gamma)^2\big)\sigma_\ell^2}$$

게이트는 다음과 같다.

$$n_\sigma\sqrt{(1-\gamma)^2\sigma_c^2+(2\gamma-\gamma^2)\,\sigma_\ell^2+\sigma_{trk}^2+(\Vert v\Vert\delta)^2}\le r_{cap}$$

$\sigma_c$는 **commit 시점 메시지**의 $t_c$ 샘플 공분산에서, $\sigma_\ell$은 **포구 직전 최신 메시지**의 같은 시각 샘플 공분산에서 뽑는다. 둘 다 vision이 준 값이다(§4.4). $\sigma_{trk}$와 $\delta$는 실측값(L5, 인프라)이며 나머지와 독립으로 두는 것은 타당하다.

commit 시점에는 $\sigma_\ell$ 을 아직 모른다. 두 경로를 둔다.

1. **계획 단계**: 보수적으로 $\sigma_\ell=\sigma_c$ 로 둔다. 그러면 식이 $\sigma_c^2$ 로 환원되는데, 이것은 **직교성 가정 없이도 상한**이다 — $\mathrm{Var}(A+\lambda B)$ 는 $\lambda=1-\gamma$ 의 볼록 2차식이라 $\lambda\in[0,1]$ 에서 최대가 끝점이고, $\sigma_\ell\le\sigma_c$ 인 한 그 값이 $\sigma_c^2$ 다.
2. **동결 후 감시**(§5.3 `monitorOnly`): 최신 메시지의 $t_c$ 샘플 공분산으로 $\sigma_\ell$ 을 갱신해 `PlanSnapshot` 에 싣는다. L7이 이 값의 성장을 보고 γ 하향·abort를 판단한다. 이 경로가 없으면 §4.6의 직교 분해 이득이 실현되지 않고 `sigma_l` 은 죽은 필드가 된다.

**직교성이 깨질 때의 방향.** 정확한 오차항은 $2\gamma(1-\gamma)\mathrm{Cov}(A,B)$ 이고 $\gamma=0,1$ 에서 사라져 $\gamma=0.5$ 에서 최대다. **새 측정을 과소 반영하는(sluggish) 예측기** — 측정잡음 과대설정, 공정잡음 과소설정 같은 흔한 튜닝 실패 — 는 $\mathrm{Cov}(A,B)>0$ 을 만들어 위 식이 $\sigma_{gap}$ 을 **과소평가**하게 한다(모의 실험에서 7–8%). v0.1의 $\gamma^2$ 오류와 같은 방향이다. 반대로 과잉반응 예측기는 보수적이 된다.

NEES가 정상이어도 $\mathrm{Cov}(A,B)=0$ 은 보장되지 않으므로, **혁신 백색성(innovation whiteness) 검정**을 게이트에 넣는다(G3-H). L1 §4.5의 $\bar\nu$ 추세가 그 대용이다.

$\sigma$ 는 스칼라로 썼지만 실제는 3×3이다. §4.4의 $\lambda_{\max}$ 규약으로 읽으며, $\lambda_{\max}(\Sigma_A+\lambda^2\Sigma_B)\le\lambda_{\max}(\Sigma_A)+\lambda^2\lambda_{\max}(\Sigma_B)$ 이므로 그 경우에도 보수적 상한이다.

**v0.1과의 차이.** v0.1은 둘째 계수를 $\gamma^2$ 로 두었다(완전 독립 가정). $\gamma=0,1$ 에서만 일치하고 중간에서 **과소평가**한다. $\sigma_\ell/\sigma_c=0.5$, $\gamma=0.5$ 에서 15.5%, $\sigma_\ell/\sigma_c=0.7$, $\gamma=0.5$ 에서 22.3% 과소평가다. 통과시켜서는 안 될 후보를 통과시킨다.

**부수 함의.** $\gamma\to1$ 이면 $\sigma_{gap}\to\sigma_\ell$ 로 줄어든다. 즉 γ를 키우는 것은 충격량(L7 §4.7)뿐 아니라 **예측 오차 측면에서도 유리하다** — commit 시점의 낡은 예측 대신 실시간 추정에 가중이 실리기 때문이다. §4.10의 γ 선호는 이 근거를 함께 쓴다.

**$\sigma_{trk}$ 의 성격.** L5 CLIK은 명령 공간에서만 닫히므로(L5 §4.2) $\sigma_{trk}$ 는 실행 중 관측되지 않는 오프라인 식별값이다. $T_{arm}$ 모델의 잔차가 그대로 여기에 들어간다.

### 4.7 교체 히스테리시스

현재 plan이 유효하면 새 후보는 다음을 모두 만족할 때만 채택한다.

1. 점수 개선 $J_{cur}-J_{new}>\Delta_J$, 또는 현재 plan이 이번 검사에서 불가능 판정.
2. 오차 점프 한계 (L4 §4.3): $(1-\gamma(t))\Vert\Delta p_c\Vert\le e_{jump,\max}$, $|\dot\gamma(t)|\,\Vert\Delta p_c\Vert\le\dot e_{jump,\max}$.
3. `COMMITTED` 이후에는 포구점을 교체하지 않는다(L7).

**`COMMITTED` 이후 유일한 예외: γ 하향 `[권장]`.** $p_c$, $t_c$ 는 동결한 채 $\gamma_f$ 만 낮추는 것은 허용한다(L4 §5.2.1, L7 §4.6). 근거는 셋이다.

- [R3]은 γ를 매 스텝 $J(q)\dot q_{\max}$ 제약 아래 재최적화한다. 즉 로봇이 못 따라가면 그 자리에서 γ를 깎는다. 본 설계는 계획 시점 rollout(§4.8)으로 γ를 고정하므로, 예측이 빗나가거나 관절 한계가 예상보다 일찍 활성화되면 대응 수단이 abort뿐이었다.
- 포화는 $t_c$ 직전에 몰린다(L4 §4.8). abort하기에 가장 나쁜 시점이다.
- $p_c$ 고정 하에 γ만 낮출 때의 $\dot e$ 점프는 $|\dot\gamma(t)|\Vert\xi^O(t)\Vert$ 이고 $t\to t_c$ 에서 $\xi^O\to0$ 이므로 **늦게 할수록 안전하다.** $e$ 는 연속이다.

필요 가속도는 $\gamma_f$ 에 거의 선형이다(§4.8 표). 하한은 $\gamma_{\min}$(손 폐쇄 제약)이며, 그 아래로는 내릴 수 없다 — 내려야 한다면 그때는 abort다.

### 4.8 γ rollout (포화 검사)

후보마다 격자 $(\gamma_f,T_w)\in\Gamma\times\mathcal T$, $\gamma_f\in[\gamma_{\min},\gamma_{\max}]$에 대해 L4 `SoftCatchTranslation`을 **포화 없이** 실행한다. 초기 상태는 현재 기준 상태, 대상은 **L2 샘플러가 vision 궤적에서 보간한 $(p,v,a)$** 다 — RT 루프가 실제로 볼 것과 같은 함수를 쓴다(`traj::sampleAt`). 다음을 기록한다.

- 구간 $[t_{now},t_k]$의 $\max\Vert u\Vert$, $\max\Vert\dot x\Vert$
- $t_k$의 잔여 오차 $\Vert e\Vert$

판정에는 `TranslationOutput::u_des`(포화 전 요구 가속도)를 쓴다. `xdd`(실현값)는 포화가 없으면 같지만 의미가 다르다(L4 §5.2).

수락 조건: $\max\Vert u_{des}\Vert\le\eta_aa_{\max}$, $\max\Vert\dot x\Vert\le\eta_vv_{\max}$, $\Vert e(t_k)\Vert\le\epsilon_{term}$ ($\eta<1$ 여유율).

수락 조합 중 $\gamma_f$ 최대를 고르고, 동률이면 최대 가속이 작은 것을 고른다. 수락 조합이 없으면 $\gamma_{\min}$을 한 번 더 검사한다. 그것도 실패하면 후보를 탈락시킨다.

참고 수치(`test_l3.cpp`, 한 시나리오, 포화 없는 rollout의 창 내 최대 $\Vert u_{des}\Vert$ [m/s²]):

| $T_w$ \ $\gamma_f$ | 0.1 | 0.2 | 0.3 | 0.4 | 0.5 |
|---|---|---|---|---|---|
| 0.30 s | 8.1 | 16.4 | 24.7 | 33.0 | 41.3 |
| 0.45 s | 5.4 | 10.5 | 15.8 | 21.2 | 26.5 |
| 0.60 s | 6.3 | 8.9 | 11.6 | 15.3 | 19.2 |
| 0.75 s | 8.9 | 8.8 | 10.4 | 12.4 | 14.8 |

$\gamma_f$ 에 대해 거의 선형이라는 점이 §4.7의 γ 하향 근거다. 짧은 창($T_w=0.30$ s)에서 $\gamma_f=0.4$ 에 33 m/s²가 필요한데, L4 기본 `a_max`가 15 m/s²임을 생각하면 창 길이 선택이 포화 여부를 지배한다.

### 4.9 정지거리 예약

포구 후 감속(L7 §4.3)에 필요한 공간을 확보한다.

$$p_{stop}=p_c+\frac{(\gamma_f\Vert v\Vert)^2}{2a_{dec}}\hat v\ \in\ \mathcal W_{catch}$$

선택적으로 $p_{stop}$에서도 IK 수락 여부를 검사한다(YAML).

### 4.10 선택 규칙 `[권장]`

단일 가중 점수 최소화를 쓴다. γ도 그 안의 한 항이다.

$$J=w_\sigma\frac{\sigma_{\max}(t_k)}{r_{cap}}+w_t\frac{\max_it_{\min,i}}{t_k-t_{now}-T_{arm}}+w_q\Vert q^\ast-q_n\Vert^2+w_{late}\,(t_{k,\max}-t_k)-w_\gamma\,\gamma_f$$

$w_{late}>0$이면 늦은 포구를 선호한다([R1]의 "latest" 목적과 같은 취지로, 예측이 정확해지는 시간을 번다). $w_\gamma>0$이면 soft catch를 선호한다(충격량 L7 §4.7, 오차 예산 §4.6 두 근거). 가중치는 튜닝 대상이다.

**v0.1의 사전식(lexicographic) 선택을 버린 이유.** v0.1은 1순위가 "$\gamma_f$ 최대"였는데, $\gamma_f$ 가 연속 격자값이라 동률이 거의 나오지 않는다. 결과적으로 1순위에서 후보가 하나로 결정되고 $w_\sigma,w_t,w_q,w_{late}$ 네 파라미터가 전부 죽는다. "늦은 포구 선호" 같은 의도가 반영되지 않는다. $\gamma_f$ 를 $J$ 안의 항으로 넣으면 $w_\gamma$ 로 그 상충을 튜닝할 수 있다.

$\gamma_f$ 를 사실상 절대 우선으로 두고 싶으면 $w_\gamma$ 를 크게 잡으면 된다 — 사전식은 $w_\gamma\to\infty$ 의 특수한 경우다.

**대안 정식화 `[미채택]`.** [R17]은 포구를 하이브리드 시스템 안정화 문제로 보고, 포구 성공을 "충돌 후 상태가 포획 집합(capture set) 안에 드는가"로 정량화한다. 그러면 후보 $t_k$ 의 점수가 위처럼 손으로 정한 가중합이 아니라 **포획 집합까지의 여유**라는 하나의 물리량이 된다. 장점은 $w_\sigma,w_t,w_q,w_{late},w_\gamma$ 다섯 개 튜닝 파라미터가 사라진다는 것이고, 대가는 손–공 충돌 모델(반발계수, 접촉 기하, 마찰)과 그 집합의 사전 계산이다. P1b 손의 접촉 모델이 아직 없고(TBD-HAND-05, L6 §4.1), RT 경로에서 집합 소속 판정을 고정 시간에 끝낼 수 있는지도 미확인이라 v0.4에서는 채택하지 않는다. L8 이후 $w$ 튜닝이 실제로 문제가 되면 재검토 대상이다. (Schill & Buss, 2018, IEEE T-RO — 서지는 마스터 §8 [R17].)

### 4.11 commit과 손 명령 시각

- commit 조건: $t_c-t_{now}\le T_{freeze}$, $T_{freeze}\ge T_{close,tot}+T_{arm}+T_{margin}$.
- 손 폐쇄 명령 시각: $t_{cmd}=t_c-T_{close}-T_{link}$. 팔 지연은 L5 선행 보상으로 이미 흡수되므로 빼지 않는다.
- **$T_{tick}$은 여기에 넣지 않는다.** §4.5의 $T_{close,tot}=T_{close}+T_{link}+T_{tick}$ 에서 $T_{tick}=h/2$ 는 틱 양자화 오차의 **worst-case 예산**이다. L6 §4.3의 반올림 규칙(가장 가까운 틱)을 쓰면 오차는 $\pm h/2$ 로 영평균이라 명령 시각 자체를 당길 이유가 없다. γ 창(예산)에는 들어가고 $t_{cmd}$(명령)에는 들어가지 않는다 — 두 곳의 역할이 다르다.
- $t_{cmd}$ 와 $t_c$ 는 `PlanSnapshot::t_ref` 기준 상대시간이며, 모두 **실제 시각** 축이다(L5 선행 보상은 팔 명령 경로에만 적용된다, L5 §4.5).
- [R2]는 접촉 직전 일정 시간부터 예측 갱신을 멈췄다. 본 구현의 동결은 포구점·γ에만 적용하고, L2의 실시간 추정은 계속 사용한다(§4.6 둘째 항).

## 5. C++ 구현

### 5.1 `time_feasibility.hpp` (참조 구현, 검증 완료)

```cpp
#pragma once
// catching_planner/time_feasibility.hpp — 관절별 최소 도달시간 (속도·가속 한계, 목표 속도 0)
//                                          + gamma 창 부등식
#include <algorithm>
#include <cmath>

namespace catching::plan {

// 정지 상태에서 거리 D(>=0)를 이동해 정지하는 최소시간
[[nodiscard]] inline double tRest(double D, double w_max, double a_max) noexcept {
  if (D <= 0.0) return 0.0;
  const double w_peak = std::sqrt(a_max * D);
  return (w_peak <= w_max) ? 2.0 * std::sqrt(D / a_max) : D / w_max + w_max / a_max;
}

struct TMinResult {
  double t{0.0};
  bool   w0_clamped{false};   // |w0| > w_max 로 들어와 clamp 했음 → 반환값은 하한일 뿐
};

// (q0, w0) → (q1, 0) 최소시간.
//
// 전제: a_max > 0, w_max > 0. |w0| <= w_max 는 전제이지만 **검사한다**.
// |w0| > w_max 이면 초기 상태 자체가 속도 한계를 위반한 것이라 최소시간 문제가
// 정의되지 않는다(사다리꼴 분기에서 (w_max - w)/a < 0 인 음수 구간이 나온다).
// 이 경우 w0 를 clamp 하고 플래그를 세운다. 호출자는 플래그가 서면 해당 후보를
// 탈락시키거나(권장) 반환값을 하한으로만 쓴다.
[[nodiscard]] inline TMinResult tMinChecked(double q0, double w0, double q1,
                                            double w_max, double a_max) noexcept {
  TMinResult r{};
  if (!(a_max > 0.0) || !(w_max > 0.0)) return r;
  if (std::abs(w0) > w_max) { w0 = std::copysign(w_max, w0); r.w0_clamped = true; }

  const double d = q1 - q0;
  constexpr double eps = 1e-12;
  if (std::abs(d) < eps && std::abs(w0) < eps) return r;
  const double s = (std::abs(d) >= eps) ? std::copysign(1.0, d) : -std::copysign(1.0, w0);
  const double D = std::abs(d);
  const double w = s * w0;                                 // 목표 방향 속도 성분

  if (w < 0.0) {                                           // 반대 방향 이동 중: 먼저 정지
    r.t = -w / a_max + tRest(D + w * w / (2.0 * a_max), w_max, a_max);
    return r;
  }
  const double d_stop = w * w / (2.0 * a_max);
  if (d_stop > D) {                                        // 지나친 뒤 복귀
    r.t = w / a_max + tRest(d_stop - D, w_max, a_max);
    return r;
  }
  const double w_peak = std::sqrt(a_max * D + 0.5 * w * w);
  if (w_peak <= w_max) {                                   // 삼각
    r.t = (w_peak - w) / a_max + w_peak / a_max;
    return r;
  }
  const double cruise = D - (w_max * w_max - w * w) / (2.0 * a_max)
                          - w_max * w_max / (2.0 * a_max); // 사다리꼴
  r.t = (w_max - w) / a_max + w_max / a_max + cruise / w_max;
  return r;
}

// 편의 래퍼. 플래그를 버리므로 clamp 여부를 따로 봐야 하는 곳에서는 tMinChecked 를 쓸 것.
[[nodiscard]] inline double tMin(double q0, double w0, double q1,
                                 double w_max, double a_max) noexcept {
  return tMinChecked(q0, w0, q1, w_max, a_max).t;
}

// gamma 창: [g_min, g_max]
struct GammaWindow {
  double g_min{0.0}, g_max{0.0};
  [[nodiscard]] bool feasible() const noexcept { return g_min <= g_max; }
};

// L3 §4.5.
//   g_min = 1 - d_eff / (|v| * T_close_tot)          (손 폐쇄 하한)
//   g_max = min(v_dir_max, v_tcp_max) / |v|          (팔 속도 상한)
//
// v_tcp_max 는 L4 `reference.v_max` 와 같은 값이어야 한다. 이 인자를 빠뜨리면
// 계획이 통과시킨 gamma 가 L4 에서 속도 포화를 일으킨다.
// clamp 는 물리적 입력 범위(d_eff >= 0, v_dir_max >= 0, v_tcp_max >= 0)에서만
// 무clamp 판정과 동치다. 음수 v_dir_max 가 들어오면 clamp 가 g_max 를 0 으로 **올려**
// 판정을 뒤집으므로 검사한다. (v_dir_max 는 L3 §4.5 의 DLS 정규화식 결과라 구현·수치
// 문제로 음수가 나올 수 있다. tMinChecked 가 |w0| > w_max 를 검사하는 것과 같은 수준의 방어.)
[[nodiscard]] inline GammaWindow gammaWindow(double v_ball, double v_dir_max, double v_tcp_max,
                                             double d_eff, double t_close_total,
                                             bool* input_invalid = nullptr) noexcept {
  const bool bad = !(d_eff >= 0.0) || !(v_dir_max >= 0.0) || !(v_tcp_max >= 0.0)
                || !(t_close_total > 0.0) || !(v_ball >= 0.0);
  if (input_invalid) *input_invalid = bad;
  if (bad) return {1.0, 0.0};                         // 실현 불가로 보고 (feasible() == false)
  const double vb = std::max(v_ball, 1e-6);
  const double g_min = std::clamp(1.0 - d_eff / (vb * std::max(t_close_total, 1e-6)), 0.0, 1.0);
  const double g_max = std::clamp(std::min(v_dir_max, v_tcp_max) / vb, 0.0, 1.0);
  return {g_min, g_max};
}

// 창이 비는 임계 공 속력: |v| > v_reach + d_eff / T_close_tot 이면 어떤 gamma 도 불가.
// 경계(|v| == 상한)에서 gammaWindow().feasible() 과 1 ulp 로 엇갈릴 수 있으므로,
// 후보 게이트에는 둘 중 하나만 쓰고 여유(planner.gamma.margin)를 둔다.
[[nodiscard]] inline double maxCatchableSpeed(double v_dir_max, double v_tcp_max,
                                              double d_eff, double t_close_total) noexcept {
  return std::min(v_dir_max, v_tcp_max) + d_eff / std::max(t_close_total, 1e-6);
}

}  // namespace catching::plan
```

### 5.2 `PlanSnapshot`

```cpp
struct PlanSnapshot {
  std::uint32_t track_epoch{0}, plan_id{0};       // L1 §4.4 (PointCloud2 에 track_id 없음)
  Nanoseconds   t_ref{0};                 // **계획에 쓴 궤적 메시지의 header.stamp 와 같은 값**
                                          // (L2 §4.4 — 두 축의 원점을 일치시킨다)
  double        t_c_rel{0.0};             // t_c - t_ref [s]  — **선행축** (L2 §4.4)
  double        t_cmd_rel{0.0};           // 손 폐쇄 명령 시각 - t_ref [s] — **실제시각축**
  Eigen::Vector3d p_c{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a_d{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d v_c{Eigen::Vector3d::Zero()};   // 예측 포구 순간 공 속도 (감속 계획용)
  ref::GammaProfile gamma{};
  double        gamma_min{0.0};                   // §4.5 손 폐쇄 하한. L7 §4.6 γ 하향의 하한
  Eigen::Matrix<double, 7, 1> q_star{Eigen::Matrix<double, 7, 1>::Zero()};
  double        score{0.0}, sigma_c{0.0}, sigma_l{0.0};   // §4.6 오차 예산 두 항
  double        dp_impact{0.0};                   // §L7 4.7 예상 충격량 [kg m/s]
  PlanReject    last_reject{PlanReject::None};    // 진단: 마지막 탈락 사유
  bool          valid{false};
};
```

### 5.3 계획 루프 (non-RT)

```cpp
void CatchPlanner::onNewSnapshot() {                       // L1 신호로 깨어남
  if (!plan_buf_.read(traj_, cov_)) return;                  // L1 계획용 버퍼 (궤적 + 공분산)
  if (settling(traj_.track_epoch)) { publishInvalid(); return; }   // §4.4
  if (!rt_state_box_.tryRead(rt_)) return;                   // q_c, qd_c, 기준 상태 (G3-2)
  const Nanoseconds t0 = clock_now();
  const double age = 1e-9 * double(t0 - traj_.t_ref);        // 메시지 나이 (§4.1)
  if (isFrozen(rt_.mode)) { monitorOnly(traj_, cov_); return; }    // §4.6 — 감시는 계속한다

  Candidate best{}; bool found = false;
  q_seed_ = rt_.q_c;
  for (int k = k_first(); k <= k_last(); ++k) {              // vision 샘플 격자
    const auto& s = traj_.s[k];                              // (t, p, v, a), t 는 header.stamp 기준
    const double t_k_now = s.t - age;                        // 현재 시각 기준 (§4.1)
    if (!inWorkspace(s.p) || s.t > horizonLimit(traj_)) break;   // L2 §4.6
    Candidate c{};
    if (!gateUncertainty(s, c))          continue;           // §4.4
    if (!solveCatchIk(s, q_seed_, c))    continue;           // §4.2 (warm start 갱신)
    if (!gateTime(c, t_k_now))           continue;           // §4.3 (현재 시각 기준)
    if (!gateGammaWindow(s, c))          continue;           // §4.5
    if (!gateStopDistance(s, c))         continue;           // §4.9
    if (!gammaRollout(s, c))             continue;           // §4.8 (L4 코드)
    if (!gateErrorBudget(s, c))          continue;           // §4.6 (직교 분해)
    if (!gateImpulse(s, c))              continue;           // L7 §4.7, TBD-IMP-01 확정 후 활성
    scoreCandidate(s, c);                                    // §4.10
    if (!found || better(c, best)) { best = c; found = true; }
  }
  const double elapsed = seconds_since(t0);
  stats_.record(elapsed, found);
  if (!found) { publishInvalidKeepReason(); return; }
  if (acceptSwitch(best)) publishPlan(best, t0);             // §4.7
}
```

- 버퍼(`traj_`, `cov_`, IK 작업 공간, rollout 상태)는 configure에서 할당한다. non-RT지만 지연 편차를 줄이기 위함이다.
- `isFrozen(mode)` 는 술어 함수로 둔다. `mode >= Mode::Committed` 같은 enum 나열 순서 의존은 상태를 추가하면 조용히 깨진다.
- **동결 중에도 `monitorOnly()` 는 돈다**(§4.6): $\sigma_\ell$, σ 성장률, L1 $\bar\nu$ 를 갱신 발행해 L7이 γ 하향·abort 판단에 쓴다. v0.2는 즉시 return 해서 `PlanSnapshot::sigma_l` 이 영구히 죽은 필드였다.
- 시행 종료 시 plan 무효화는 L7이 한다(L7 §4.8).
- 후보 시각은 vision 샘플 격자를 그대로 쓴다. `planner.slice.dt`가 카메라 주기보다 크면 격자를 솎아 쓰고, 작으면 보간해 쓴다(L2 `sampleAt`). 어느 쪽이든 격자 간격은 `TBD-VIS-04` 확정 후 정한다.
- 연산 예산 초과 시 남은 슬라이스를 건너뛰고 지금까지의 최선을 쓴다(`budget_s`).

### 5.4 방향 속력 (§4.5)

```cpp
// 반환: 포구 자세에서 v_hat 방향으로 낼 수 있는 TCP 속력 [m/s] 의 보수적 추정.
// DLS 해는 [v_hat; 0] 를 정확히 달성하지 못하므로 **달성 속력으로 정규화**한다.
[[nodiscard]] inline double directionalSpeedMax(const Eigen::Matrix<double, 5, Eigen::Dynamic, 0, 5, 7>& J, // [J_p; S J_w^L]
                                                const Eigen::Vector3d& v_hat,
                                                const joint::VecN& qd_max, double lambda) noexcept {
  Eigen::Matrix<double, 5, 1> y; y << v_hat, 0.0, 0.0;
  const Eigen::Matrix<double, 5, 5> JJt = J * J.transpose() + lambda * lambda * Eigen::Matrix<double, 5, 5>::Identity();
  const joint::VecN qd_unit = J.transpose() * JJt.ldlt().solve(y);   // 고정 최대 크기
  double r = 0.0;
  for (int i = 0; i < qd_unit.size(); ++i) r = std::max(r, std::abs(qd_unit[i]) / qd_max[i]);
  if (!(r > 0.0)) return 0.0;                     // 관절 운동이 필요 없다 = 판정 불가 → 보수적으로 0
  const double achieved = (J.template topRows<3>() * qd_unit).norm();   // <= 1, 특이 자세에서 작아진다
  return achieved / r;
}
```

`achieved` 를 곱하지 않으면 특이 자세 근처에서 실제로 낼 수 없는 속력을 보고한다(§4.5).

## 6. YAML 파라미터

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `planner.budget_s` | double | s | 0.010 | 0.001–0.016 | 60 Hz 주기 내 |
| `planner.slice.dt` | double | s | `TBD` | 0.005–0.05 | vision 샘플 간격의 정수배 (TBD-VIS-04) |
| `planner.slice.t_lead_min` | double | s | `TBD` | >0 | $T_{freeze}$ 이상 |
| `planner.slice.t_max` | double | s | `TBD` | 0.2–1.5 | vision 지평 − `prediction.t_horizon_margin` 이하 |
| `planner.n_settle` | int | – | 3 | 0–20 | §4.4 트랙 epoch 변경 후 대기 메시지 수 |
| `planner.gamma.margin` | double | m/s | 0.1 | 0–1 | §4.5 `maxCatchableSpeed` 경계 여유 (1 ulp 엇갈림 방지) |
| `planner.ik.max_iter` | int | – | 20 | 1–100 | 연산 예산 |
| `planner.ik.lambda` | double | – | 0.05 | 1e-4–0.5 | DLS |
| `planner.ik.rho` | double | m/rad | 0.1 | 0.01–1 | §4.2 위치/회전 스케일 정합 (특성길이) |
| `planner.ik.eps_pos` | double | m | 0.002 | – | 수락 |
| `planner.ik.alpha_max` | double | rad | `TBD` | 0–π/2 | 손 형상 허용 콘 ($\theta\le\alpha_{\max}$) |
| `planner.ik.manip_min` | double | – | `TBD` | ≥0 | §4.5 조작도 하한 $\sqrt{\det J_pJ_p^\top}$ |
| `planner.time.margin` | double | s | 0.03 | 0–0.2 | §4.3 |
| `planner.unc.kappa_sigma` | double | – | 0.3 | 0.05–1 | §4.4 |
| `planner.hand.d_eff` | double | m | `TBD` | >0 | TBD-HAND-04 (손별) |
| `planner.hand.r_cap` | double | m | `TBD` | >0 | TBD-HAND-04 |
| `planner.gamma.grid` | double[] | – | [0.0, 0.1, …, 0.6] | 0–1 | §4.8 |
| `planner.gamma.window_grid` | double[] | s | [0.3, 0.45, 0.6] | >0 | §4.8 |
| `planner.gamma.eta_a`, `eta_v` | double | – | 0.8, 0.9 | 0–1 | 여유율 |
| `planner.gamma.eps_term` | double | m | 0.002 | – | §4.8 |
| `planner.budget.n_sigma` | double | – | 2.0 | 1–3 | §4.6 |
| `planner.budget.sigma_trk` | double | m | `TBD` | ≥0 | L5 실측 |
| `planner.budget.clock_err` | double | s | `TBD` | ≥0 | 인프라 실측 |
| `planner.stop.a_dec` | double | m/s² | `TBD` | >0 | L7과 공유 |
| `planner.stop.check_ik` | bool | – | true | – | §4.9 |
| `planner.switch.delta_J` | double | – | 0.1 | ≥0 | §4.7 |
| `planner.switch.e_jump_max` | double | m | 0.01 | >0 | §4.7 |
| `planner.switch.ed_jump_max` | double | m/s | 0.05 | >0 | §4.7. γ 하향의 $\dot e$ 점프에도 같은 임계를 쓴다 |
| `planner.gamma.derate_step` | double | – | 0.1 | 0.02–0.3 | §4.7 1회 하향 폭 (L7 §4.6) |
| `planner.freeze.T_freeze` | double | s | `TBD` | ≥ §4.11 하한 | §4.11 |
| `planner.score.w_sigma`, `w_t`, `w_q`, `w_late`, `w_gamma` | double | – | 1, 1, 0.1, 0, 5 | ≥0 | §4.10 튜닝. `w_gamma`를 크게 잡으면 사전식 선택과 같아진다 |
| `planner.workspace.catch_box` | box | m | `TBD` | – | TBD-BALL-02 |

## 7. 단위 기술 구현 순서

- **L3.1** `time_feasibility.hpp` + LP 대조 테스트(`test_l3.cpp` → `cases.txt` → `verify_l3.py`, 결과를 고정 테이블로 GTest화) + `w0_clamped` 경로 테스트.
- **L3.2** `catch_ik.hpp` (Pinocchio 4.0) + 수렴률·콘·스케일($\rho$) 테스트.
- **L3.3** `directional_speed.hpp` + γ 창 테스트([R1] 수치 sanity, `v_tcp_max` 구속, `maxCatchableSpeed` 포함).
- **L3.4** γ rollout (L4 코드 호출) + §4.8 표 재현 테스트.
- **L3.5** 오차 예산(§4.6 직교 분해)·정지거리 게이트.
- **L3.6** 선택·히스테리시스·commit + γ 하향 경로(L7 §4.6과 함께).
- **L3.7** 계획 스레드, 예산 관리, 진단 발행.
- **L3.8** (선택) 부록 A SQP, `iiwa7_leap` 전용.

L3.1·L3.3·L3.4는 `test_l3.cpp`가 참조 구현을 이미 돌리고 있다. 단, **L3 착수 전에 마스터 §4.1의 $T_{close,tot}$ 선행 측정(L6a)을 끝낼 것.** 그 값이 `planner.gamma.*`, `planner.hand.*`, `reference.a_max`를 전부 좌우한다.

## 8. 디버깅 방법

- 계획마다 기록: 후보 수, 게이트별 탈락 수(히스토그램), 선택 후보의 $(t_c,p_c,\gamma_f,T_w,\sigma_{\max},\max t_{\min})$, 실행시간, 교체 여부와 사유.
- "항상 탈락": 게이트별 탈락 히스토그램에서 첫 번째 병목을 찾는다. γ 창이 원인이면 $d_{eff}$, $T_{close}$, $v_{dir,\max}$ 값을 먼저 의심한다.
- 포구점이 자주 바뀐다: `delta_J`, 점프 한계, 예측 품질(L2 `lastJump`)을 확인한다.
- IK 수렴 실패: catch frame 축 정의, `alpha_max`, 초기값(warm start 끊김)을 확인한다.
- 시뮬레이션 시각화(RViz): vision 예측 궤적(시각화용 `nav_msgs/Path` 로 재발행), 후보 점(색 = 탈락 사유), 선택된 $p_c$와 $a_d$, $p_{stop}$.

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G3-A | `tMin` 고정 테이블 일치 (< 1e-9, 스크립트 기준값 대비) + `w0_clamped` 시 후보 탈락 | `[SIM-ANY]` |
| G3-B | γ 창 sanity ([R1] 3 cm / 6 m/s → 5 ms 통과, 6 ms 탈락), `v_tcp_max` 구속, §4.8 표 재현(±5%) | `[SIM-ANY]` |
| G3-C | 합성 투척 1000회: 계획 실행시간 99% < `budget_s`, 탈락 사유 분포 기록 | `[SIM-ANY]` |
| G3-D | `iiwa7_leap` 시뮬레이션에서 plan 유효율, 교체 빈도 기록 | `[SIM-ANY]` |
| G3-E | `ur5e_p1b` 시뮬레이션에서 선택된 plan의 L4/L5 실행 시 포화 0, 한계 활성 비율, γ 하향 발생률 기록 | `[SIM-P1B]` |
| G3-F | 실측 $T_{close}$, $T_{arm}$, $\sigma_{trk}$ 반영 후 γ 창·오차 예산 재산정. $\Vert v\Vert_{\max}$(§4.5)가 목표 투척 속도를 덮는지 확인 | `[HW-P1B]` |
| G3-G | IK 수렴률: 합성 후보 1000개에서 `max_iter` 내 수락 비율과 실패 시 잔차 분포 기록 (§4.2는 Gauss-Newton이 아니므로 수렴 보장이 없다) | `[SIM-ANY]` |
| G3-H | 오차 예산 모델 검증: L8에서 §4.6 예측 간극 분포와 실제 간극 분포 비교 (직교 분해 식이 맞는지) | `[SIM-P1B]` |

## 10. 미확정 항목

TBD-RTC-14~16, TBD-HAND-01, TBD-HAND-04, TBD-BALL-02, `planner.ik.alpha_max`, `planner.stop.a_dec`, `planner.freeze.T_freeze`.

---

## 부록 A. [R1]식 SQP (`iiwa7_leap` 확장용, 선택)

결정변수 $y=(q,t)\in\mathbb R^{n+1}$.

**등식 제약.** 위치 3개와 접근축 2개.

$$h_p=p_C(q)-\hat p(t),\qquad \frac{\partial h_p}{\partial(q,t)}=\big[J_p(q)\ \ -\hat v(t)\big]$$

$$h_o=\begin{bmatrix}\hat x_C(q)^\top\hat v(t)\\\hat y_C(q)^\top\hat v(t)\end{bmatrix},\qquad \hat z_C(q)^\top\hat v(t)\le0$$

$\hat z_C^\top\hat v=-1$을 등식으로 쓰면 해에서 기울기가 0이 되어 LICQ가 깨진다. 위 형태는 해에서 $\hat x_C\perp\hat v$이므로 기울기가 살아 있다.

**기울기.** $d\hat x_C=\omega\times\hat x_C$이므로

$$\frac{\partial(\hat x_C^\top\hat v)}{\partial q}=(\hat x_C\times\hat v)^\top J^W_\omega,\qquad \frac{\partial(\hat x_C^\top\hat v)}{\partial t}=\hat x_C^\top\frac{(I-\hat v\hat v^\top)\hat a}{\Vert\hat v\Vert}$$

**부등식.** 관절 한계, $0<t-t_{now}\le t_{\max}$, 각 관절 $t-t_{now}-T_{arm}\ge t_{\min,i}(q_i)$ (§4.3), 작업공간.

**목적.** [R1]의 세 목적(soft/latest/cool) 중 선택. cool의 $L_4$ 노름은 $\max_it_{\min,i}$의 매끄러운 근사다.

**풀이.** ProxQP 부분문제의 SQP([R15]), 직전 해 warm start, 초기해 multi-start.
