# Waist 포함 dual-arm manipulator의 오른손 단일 포구 — MPC 계획기와 다중 frame CLIK의 수학적 정리

- 작성일: 2026-09-29 (v0.4 — 문헌 대조의 "빠진 것" · "근거가 약한 것" 반영, 예측 격자 sweep 추가)
- 대상: `rtc-framework` dynamic_catching 확장 설계 (waist + dual-arm, 오른손 한 손 파지 물체)
- 전제: estimator(ball_perception PointCloud2)와 CLIK 입력 형식(pose + twist feedforward)은 고정. CLIK는 다중 frame 확장형.
- 상태 표기: `[가정]` 기구·구성 가정, `[확인 필요]` 코드 대조 전 항목, `[선택]` 옵션.
- 인용 표기: `[Heins2023]` 같은 key 는 §7 참고 문헌의 항목이다. 문헌 대조 결과는 §6 에 있다 — "고칠 것" (§6.2) 은 v0.3 에, "빠진 것" (§6.3) 과 "근거가 약한 것" (§6.4) 은 v0.4 에 반영했다.

### 개정 이력

| 버전 | 내용 |
|---|---|
| v0.1 | 활성 자유도 $q_a=(q_w,q_R)$, 왼팔 $q_L^{hold}$ 고정 |
| v0.2 | **활성 자유도를 전체 $q=(q_w,q_L,q_R)$로 확장.** 왼팔은 포구에 참여하지 않지만 MPC 결정변수에 포함 — 자세 유지는 고정이 아니라 비용, 각운동량 counter-swing 가능, 자기충돌 기울기가 두 팔 모두에 생김, waist 토크 행에 왼팔 운동의 반력이 들어감, 공–왼팔 거리가 $q_L$ 궤적의 함수가 됨. 결정변수 $nN$ 증가에 따른 move blocking 추가 |
| v0.2a | 수식 표기만 정리 (내용 불변) — GitHub 와 VSCode 미리보기 양쪽에서 렌더링되도록 고침. 규칙은 아래 "수식 표기 규칙" |
| v0.2b | 문헌 · 공개 코드 대조 추가 (§6 대조 결과, §7 참고 문헌, §8 공개 코드) 와 본문 인용. **식과 설계는 불변** |
| v0.3 | **G1 기구** ($n_w=3$, $n=17$, frame 이름), **토크 기반 제약** (가속 box 삭제, 토크 행을 활성 관절 전체로 — 계획 MD-7), **단일 팔 환원형** (§1.6), **§6.2 의 R-1 – R-9 반영**: 속도 선형화의 1차 항 포함, RTI 주기당 1회와 shift 절차, 재계획 주기 0.05 s, 종단 정지 제약의 위상 정정, 한계 여유의 성립 조건, 자기충돌 쌍의 두 부류, RT 보간기 삭제 (관절 기준에서 FK), 이산 tick 의 오차 점화식 |
| v0.4 | **§6.3 · §6.4 반영** (사용자 승인 2026-09-29): 포구 구간의 상대속도 · 경로 비용과 방향별 가중, 토크 · 충돌 행의 slack, 토크 행의 1차 선형화, 예측 공분산에 비례하는 일관성 가중, 후보 순위의 공통 가중, 손을 잠근 축소 모델, CLIK 의 제동 거리 한계 · 충돌 damper · 되먹임 상한. **각운동량 항은 유지** — 목적을 왼팔 운동 생성과 floating base 확장으로 명시 (§1.4). **예측 격자 sweep** 추가 (§1.7) — horizon 0.75 s 와 1.0 s, 조건은 기존 제약 (점 수 상한 40, horizon 이 step 의 배수) 안에서 도출. **v0.3 의 오류 정정**: 재계획 주기는 예측 메시지 주기이고 예측점 간격과 다르다 (§1.2) |

### 수식 표기 규칙

markdown 이 수식 안의 문자를 먼저 해석하므로 (GitHub 기준), 수식을 고칠 때 다음을 지킨다.

| 쓰지 않는다 | 대신 쓴다 | 이유 |
|---|---|---|
| backslash + 세로선 (norm 기호) | `\Vert` | backslash escape 로 `\` 가 사라진다. 표 안에서는 열 구분자가 된다 |
| `\,` `\!` `\ ` | 공백, `\enspace`, `\quad` | backslash escape |
| `\{` `\}` | `\lbrace` `\rbrace` | backslash escape |
| 줄 중간의 `\\` | `\\` 뒤에서 줄바꿈 | 줄 끝이 아니면 `\` 하나로 줄어든다 |
| `<` `>` | `\lt` `\gt` | HTML 로 해석된다 |
| 기호 뒤의 `_` (`}_k`, `]_w`, `)_x`) | 양쪽에 공백 (`} _ k`) | 강조 표시로 해석돼 display 수식까지 깨진다 |
| `·` 나 `/` 에 붙은 `$` | 사이에 공백 | 수식이 열리지 않는다 |

---

## 0. 가정과 표기

### 0.1 기구 가정 `[가정]`

고정 베이스 위에 waist $q_w\in\mathbb R^{n_w}$, 그 하류의 몸통 링크(frame $T$)에 좌·우 팔 $q_L\in\mathbb R^{n_L}$, $q_R\in\mathbb R^{n_R}$가 붙어 있다. 오른손에 catch frame $C_R$, 왼손에 기준 frame $C_L$이 있다. 전체 관절

$$q=(q_w,q_L,q_R)\in\mathbb R^n,\qquad n=n_w+n_L+n_R .$$

G1 (`g1_with_proto_1b_fixed_base`) 에서의 값:

| 항목 | 값 |
|---|---|
| waist | $n_w=3$ — `waist_yaw_joint` → `waist_roll_joint` → `waist_pitch_joint` 순 |
| 팔 | $n_L=n_R=7$, 따라서 $n=17$ |
| 몸통 frame $T$ | `torso_link` — waist 세 관절 모두의 하류이고 양 어깨의 부모다 |
| 왼손 기준 frame $C_L$ | `left_rubber_hand` (`left_hand_palm_joint` 의 자식, 고정) |
| 오른손 | proto_1b — 폐쇄 체인 (구동 10, 수동 10, loop closure 5). **MPC 활성 관절 밖**이고 손 시퀀서가 구동한다 |
| 고정된 것 | 다리와 `pelvis` — 움직이지 않지만 충돌 상대다 (§0.3) |

동역학 모델은 **오른손 관절을 기준 자세로 잠근 축소 모델**이다 (Pinocchio `buildReducedModel`). 손의 질량과 관성은 손목 링크에 합쳐진다. 폐쇄 체인 구속은 MPC 와 CLIK 의 $M,h$ 에 들어가지 않는다 — 손 자세에 따른 관성 변화는 무시하며, 그 크기는 토크 여유 $\eta' _ \tau$ 가 담는다. 폐쇄 체인 동역학 자체는 [Carpentier2021] 이 다룬다. 오른손 catch frame $C_R$ 의 link 이름은 `[확인 필요]`.

왼손은 포구에 참여하지 않는다. 그러나 **왼팔은 MPC 결정변수에 포함**된다 — 역할은 (i) 자세 유지(비용), (ii) 각운동량 상쇄, (iii) 공·오른팔과의 충돌 회피(제약)다.

### 0.2 기호

| 기호 | 뜻 |
|---|---|
| $W,\enspace T,\enspace C_R,\enspace C_L$ | world, 몸통, 오른손 catch, 왼손 기준 frame |
| $T_{WT}(q_w),\enspace T_{TC_R}(q_R),\enspace T_{TC_L}(q_L)$ | 체인별 FK. $T_{WC_R}=T_{WT} T_{TC_R}$ |
| ${}^WJ_{C_R}(q)\in\mathbb R^{6\times n}$ | $C_R$의 LOCAL_WORLD_ALIGNED Jacobian. 열 구조 $[ J_{R,w}\quad0\quad J_{R,R} ]$ |
| ${}^TJ_{C_L}(q)\in\mathbb R^{6\times n}$ | 몸통 frame 기준 $C_L$의 Jacobian. 열 구조 $[ 0\quad J_{L,L}\quad0 ]$ |
| $\hat p_b(t),\hat v_b(t),\Sigma_p(t)$ | estimator 예측 위치·속도·위치 공분산 (world) |
| $a(q)=R_{WC_R}e_z$, $a_d=-\hat v_b/\Vert\hat v_b\Vert$ | 접근축과 목표 접근축 |
| $e_a(q)$, $J_a(q)$ | 접근축 회전벡터 오차 ($\exp([e_a] _ \times)a=a_d$)와 그 Jacobian (`rtc_math` se3) |
| $A_G(q)\in\mathbb R^{6\times n}$, $A^\omega_G$ | 중심 운동량 행렬과 그 각운동량 3행 |
| $q_L^{rest}$ | 왼팔 대기 자세 (비용 기준값, 제약 아님) |
| $H_v(\bar q,\bar{\dot q})\in\mathbb R^{3\times n}$ | $\partial_q[J^v_{C_R}(q)\bar{\dot q}]$ 를 $\bar q$ 에서 평가한 것 — 손 속도의 관절 위치에 대한 미분 |
| $\mathcal P_A$, $\mathcal P_B$ | 자기충돌 쌍의 두 부류 (§0.3) |
| $\Delta$, $N$, $k_c$ | MPC 노드 간격, 노드 수, 포구 노드 ($t_c=t_s+k_c\Delta$) |
| $\Delta_v$ | vision 예측점 간격. 예측점은 메시지의 기준 시각에서 $\Delta_v,2\Delta_v,\dots$ 떨어져 있다. 출하값 0.05 s, sweep 대상 (§1.7) |
| $T_r$ | 재계획 주기 = 예측 메시지의 주기. sim 실측 30 Hz (v1 L2 G2-1). $\Delta$ · $\Delta_v$ 와 다른 양이다 |
| $T_{close}$, $\mathcal K_c$ | 손 폐쇄 시간과, 포구 구간의 노드 집합 $\lbrace k_c,\dots,k_c+\lceil T_{close}/\Delta\rceil\rbrace$ |
| $\hat d_k$, $P_{\perp,k}$ | 공 진행 방향 $\hat v_b(t_k)/\Vert\hat v_b(t_k)\Vert$ 와 그에 수직인 성분의 사영 $I-\hat d_k\hat d_k^\top$ |
| $t_s$ | 새 계획이 효력을 갖는 시각 ($t_k+T_{pipe}$) |
| $h$ | RT tick (`ControllerState::dt`) |
| $\mathrm{Log}:SO(3)\to\mathbb R^3$ | 회전벡터, Hamilton quaternion 규약 |
| $\Vert z\Vert^2_W$ | $z^\top Wz$ |

### 0.3 핵심 구조적 사실

몸통 $T$가 waist 하류이므로 $T_{TC_L}$는 $q_L$만의 함수다. 왼손 목표를 **몸통 frame**에서 주면 왼손 과제의 Jacobian은 waist 열이 0이 되어, waist는 오른손 과제·자세 과제·각운동량 항만이 결정한다. 아래 정식화는 전부 이 선택 위에 있다.

자기충돌 쌍은 waist 의존성으로 두 부류로 나뉜다.

| 부류 | 쌍 | 거리가 의존하는 관절 |
|---|---|---|
| $\mathcal P_A$ | 팔–팔, 팔 · 손–`torso_link` (과 그에 고정된 머리) | $(q_L,q_R)$ — waist 에 무관 |
| $\mathcal P_B$ | 팔 · 손–`pelvis`, `waist_yaw_link`, `waist_roll_link`, 고정된 다리 | $(q_w,q_L,q_R)$ |

$\mathcal P_B$ 의 상대 물체는 waist 관절 중 하나 이상의 상류에 있다. G1 은 대기 자세에서 손이 골반 옆에 있으므로 이 부류를 뺄 수 없다.

몸통 기준 과제는 dual-arm 의 상대 Jacobian 과 같은 구조다 [Lewis1990] [Jamisola2015]. 공개 구현으로는 pink · mink · placo 의 relative frame task 가 있다 (§8).

왼손을 world에 고정해야 하는 상황이면 ${}^WJ_{C_L}=[ J_{L,w}\quad J_{L,L}\quad0 ]$로 waist 열이 살아나 두 과제가 waist를 두고 경쟁한다 (§4 항목 6).

---

## 1. MPC (계획기 스레드)

### 1.1 결정변수와 예측 모델

활성 관절은 전체 $q\in\mathbb R^n$이다. 상태 $x_k=(q_k,\dot q_k,\ddot q_k)\in\mathbb R^{3n}$, 입력은 구간 상수 jerk $u_k\in\mathbb R^{n}$:

$$
x_{k+1}=A x_k+B u_k,\qquad
A=\begin{bmatrix}I&\Delta I&\tfrac12\Delta^2 I\\
0&I&\Delta I\\
0&0&I\end{bmatrix},\quad
B=\begin{bmatrix}\tfrac16\Delta^3 I\\
 \tfrac12\Delta^2 I\\
 \Delta I\end{bmatrix}.
$$

Condensing: $\mathbf x=\Phi x_0+\Gamma\mathbf u$, $\Phi=[A;A^2;\dots;A^N]$, $\Gamma_{kj}=A^{k-1-j}B$ ($j\lt k$). 결정변수는 $\mathbf u\in\mathbb R^{nN}$ 하나다 [BockPlitt1984] [Frison2016].

**Move blocking.** $n=17$, $N=20$이면 결정변수 340개이고, dense 분해 비용은 변수 수의 세제곱 규모로 는다. 입력을 블록 $\mathcal B=\lbrace[k_0,k_1),[k_1,k_2),\dots\rbrace$로 묶어 $u_k=\tilde u_b\enspace(k\in b)$로 두면 $\mathbf u=E \tilde{\mathbf u}$, $E\in\mathbb R^{nN\times n|\mathcal B|}$이고 condensing은 $\mathbf x=\Phi x_0+(\Gamma E)\tilde{\mathbf u}$로 그대로 성립한다. 권장 분할: 오른팔·waist는 $k\lt k_c$에서 1노드, $k\ge k_c$(정지 구간)에서 2–3노드; 왼팔은 전 구간 2–3노드. 관절군별로 다른 블록을 쓰려면 $E$를 관절군마다 따로 구성한다. 고정된 블록 패턴에서는 직전 해를 한 노드 민 것이 새 문제의 결정변수로 표현되지 않으므로 재귀적 실행 가능성이 보장되지 않는다 [Cagienard2007]. 이 문서는 그 보장을 주장하지 않는다 (§1.3 "종단 정지 제약의 위상").

**정지 구간의 블록 수.** 종단 등식은 관절마다 2개 ($\dot q_N=0$, $\ddot q_N=0$) 다. 포구 구간 뒤에 관절마다 자유 블록이 2개면 등식만으로 해가 정해져 한계를 피할 여지가 없다. 그래서 **포구 구간 뒤의 블록은 관절마다 3개 이상**으로 둔다. 이 조건은 이 문서의 계산이며 구현에서 rank 로 확인한다.

**문제 크기.** 변수 수만이 아니라 행 수가 비용을 정한다. condensing 뒤에는 상태 제약이 모두 dense 행이 되고, move blocking 은 행을 줄이지 않는다.

| 행 | 수 ($n=17$, $N=20$) |
|---|---|
| 위치 한계 | 340 |
| 속도 한계 | 340 |
| 토크 행 | 340 |
| trust region | 340 |
| 충돌 | 쌍의 수 × 노드 수 |
| 종단 등식 | 34 |

- 양쪽 부등식 약 1400 행에 충돌 행이 더해진다.
- condensed QP 를 만드는 비용은 horizon 의 제곱 규모이고 주기마다 든다 [Verschueren2022]. dense 분해는 세제곱 규모다 [Frison2016].
- 행을 줄이는 수단은 한계 행을 일부 노드에만 거는 것이다 `[선택]`. 거르는 기준은 직전 해에서 한계까지의 거리다.

모델 선택 근거:

1. 동역학이 선형이라 비선형은 출력(FK·거리·운동량)에만 남고, 결정변수가 $\tilde{\mathbf u}$ 하나라 기존 `QPSolverWrapper`(ProxQP dense, 고정 차원 사전 할당)에 들어간다.
2. jerk가 구간 상수이므로 RT가 노드 사이의 관절 기준을 닫힌식으로 정확히 재현할 수 있다. 손의 목표는 그 관절 기준에서 FK 로 만든다 (§1.5).
3. 가속이 연속이라 CLIK의 가속·토크 제약이 명령 계단을 보지 않는다.

같은 상태 · 입력 정의를 같은 이유 (가속 연속) 로 매니퓰레이터 MPC 에 쓴 사례가 [Heins2023] 이다. jerk 입력 모델 자체는 [Wieber2006], 관절 공간의 jerk 제한 궤적은 [Berscheid2021] 을 본다.

### 1.2 비선형 출력의 선형화 (SQP-RTI)

직전 해 $\bar x_k$ 주변에서, $\delta q=q_k-\bar q_k$:

$$
\begin{aligned}
p_{C_R}(q_{k_c})&\approx p_{C_R}(\bar q)+J^v_{C_R}(\bar q) \delta q,\\
e_a(q_{k_c})&\approx e_a(\bar q)+J_a(\bar q) \delta q,\\
J^v_{C_R}(q_{k_c})\dot q_{k_c}&\approx J^v_{C_R}(\bar q) \dot q_{k_c}
+H_v(\bar q,\bar{\dot q}) \delta q,\\
d_j(q_k)&\approx d_j(\bar q_k)+\nabla_{q_L}d_j^\top \delta q_L+\nabla_{q_R}d_j^\top \delta q_R
\quad(j\in\mathcal P_A\text{, waist 열 }0),\\
d_j(q_k)&\approx d_j(\bar q_k)+\nabla_{q_w}d_j^\top \delta q_w+\nabla_{q_L}d_j^\top \delta q_L+\nabla_{q_R}d_j^\top \delta q_R
\quad(j\in\mathcal P_B),\\
d^{ball} _ {L}(q_k,t_k)&\approx d^{ball} _ L(\bar q_k,t_k)+\nabla_{q_w}d^{ball\top} _ L\delta q_w+\nabla_{q_L}d^{ball\top} _ L\delta q_L,\\
A^\omega_G(q_k)\ddot q_k+\dot A^\omega_G(q_k,\dot q_k)\dot q_k
&\approx A^\omega_G(\bar q_k) \ddot q_k+\dot A^\omega_G(\bar q_k,\bar{\dot q} _ k) \bar{\dot q} _ k ,\\
\tau(q_k,\dot q_k,\ddot q_k)&\approx\bar\tau_k+M(\bar q_k) \delta\ddot q_k+\partial_q\tau \delta q_k+\partial_{\dot q}\tau \delta\dot q_k=:\tau^{lin} _ k .
\end{aligned}
$$

- $J^v_{C_R}$, $J_a$, $H_v$ 는 $q_L$ 열이 구조적으로 0이다.
- 손 속도의 선형화는 $\delta q$ 의 1차 항 $H_v\delta q$ 를 포함한다. 버리는 것은 $\delta q$ 와 $\delta\dot q$ 의 곱인 2차 항뿐이다. $H_v$ 는 포구 노드 한 점에서만 필요하며 Pinocchio 의 `computeForwardKinematicsDerivatives` 와 `getFrameVelocityDerivatives` 로 계산한다. LOCAL_WORLD_ALIGNED 에서 이 미분이 $J^v\dot q$ 의 미분과 같은지는 유한 차분으로 대조한다 `[확인 필요]`.
- 자기충돌 $d_j$ 는 **두 팔 모두**에 기울기를 갖는다 — 왼팔이 오른팔 경로로 휘둘러질 수 있으므로 v0.1의 "왼팔 capsule 상수" 단순화는 사라진다. $\mathcal P_B$ 의 쌍은 waist 기울기도 갖는다.
- 공–왼팔 거리 $d^{ball} _ L$은 예측 공 위치 $\hat p_b(t_k)$와 왼팔 capsule 사이 거리이며 $(q_w,q_L)$의 함수다. 왼팔이 counter-swing으로 공 경로에 들어가는 것을 막는다.
- 토크는 **1차까지 선형화**한다. $\bar\tau_k$ 는 직전 해에서의 역동역학 값이고, $\partial_q\tau$ · $\partial_{\dot q}\tau$ 는 Pinocchio 의 `computeRNEADerivatives` 가 준다 ($\partial_{\ddot q}\tau=M$). 해석적 미분의 계산 비용은 작다 [Carpentier2018].
- 종단 노드에서는 $\dot q_N=\ddot q_N=0$ 이므로 토크 행이 $g(\bar q_N)+\partial_qg \delta q_N$ 이 된다. 이것이 **정지 자세를 유지할 토크가 한계 안인지** 보는 행이다.
- 각운동량 변화율의 식은 선형화가 아니라 **계수를 고정한 근사**다. $\ddot q$ 에는 정확하지만 $q$, $\dot q$ 에 대한 1차 항을 뺐다. Jacobian 이 부정확한 SQP 는 보정 없이는 원 문제의 정류점으로 수렴하지 않는다 [Diehl2010]. 이 항은 비용에만 있고 제약에는 없으므로 실행 가능성에는 영향이 없다. 1차 항은 Pinocchio 의 `computeCentroidalDynamicsDerivatives` 로 넣을 수 있다 `[선택]`.

**반복.** 재계획 주기 $T_r$ 마다 선형화 1회와 QP 1회를 한다 (Real-Time Iteration). 수렴은 주기를 거치며 누적된다. RTI 의 정의와 전제는 [Diehl2005] [Gros2020], 구현 기준은 [Verschueren2022] 다. 한 주기에 2회 반복하면 선형화와 condensing 을 다시 해야 하므로 계산 예산은 그만큼으로 센다 `[선택]`.

**세 시간 간격.** 재계획 주기 $T_r$, MPC 노드 간격 $\Delta$, vision 예측점 간격 $\Delta_v$ 는 서로 다른 양이다.

| 양 | 정하는 것 | 현재 값 |
|---|---|---|
| $T_r$ | 예측 메시지가 오는 주기 | sim 실측 30 Hz |
| $\Delta_v$ | 포구 시각 후보의 간격 | 0.05 s (sweep 대상, §1.7) |
| $\Delta$ | MPC 의 결정변수 수 | 0.05 s |

v0.3 은 $T_r=\Delta=\Delta_v$ 로 적었다. 이것은 틀렸다 — 0.05 s 는 예측점 간격이고 메시지 주기가 아니다. v0.2 의 "30 Hz 재계획" 이 맞았다.

**격자.** MPC 격자는 $t_c$ 에 고정한다. 노드 시각은 $t_c+(k-k_c)\Delta$ 다. $\Delta$ 가 $\Delta_v$ 와 독립이므로 예측점을 촘촘히 해도 결정변수가 늘지 않는다.

$$
k_c=\Big\lfloor\frac{t_c-(t_k+T_{pipe})}{\Delta}\Big\rfloor,\qquad t_s=t_c-k_c\Delta .
$$

- $t_s$ 는 $t_k+T_{pipe}$ 이후의 첫 격자점이다 (사용자 승인 2026-09-29). 효력 시각이 최대 $\Delta$ 늦어진다.
- $t_c$ 가 격자의 기준이므로 COMMITTED 뒤에도 $t_c$ 는 격자 위에 있다.

**Shift.** 선형화 기준점 $\bar x$ 는 직전 해를 새 격자에서 다시 평가한 것이다. shift 를 빼면 RTI 의 폐루프 성능이 나빠진다 [Gros2020].

$$
\bar x_k\leftarrow x^{prev}\big(t_s+k\Delta\big)\quad(k=0,\dots,N).
$$

- $T_r$ 가 $\Delta$ 의 정수배가 아니므로 노드 번호를 미는 것으로는 안 된다. 직전 해는 구간별 3차식이라 임의 시각에서 닫힌식으로 평가된다 (§1.5 의 식).
- 직전 해의 끝을 넘는 시각은 종단 상태로 둔다. 종단이 $\dot q_N=\ddot q_N=0$ 이므로 정지가 이어진다.
- $\bar x$ 는 기준점일 뿐이라 move blocking 으로 표현되지 않아도 된다.
- 포구 시각 후보를 바꾼 주기의 초기화는 §1.3 "바깥 루프" 에 있다.

### 1.3 최적화 문제

$$
\boxed{
\begin{aligned}
\min_{\tilde{\mathbf u},s}\quad
&\sum_{k=0}^{N-1}\Vert u_k\Vert_{R}^2
+w_\Delta\sum_{k}\Vert q_{k}-q^{prev} _ {k}\Vert^2
+w_w\sum_k\Vert\dot q_{w,k}\Vert^2
+\sum_k\Vert q_{L,k}-q_L^{rest}\Vert^2_{W_L^{rest}}\\
&+\sum_k\frac{\big\Vert A^\omega_G(\bar q_k)\ddot q_k+\dot A^\omega_G(\bar q_k,\bar{\dot q} _ k)\bar{\dot q} _ k\big\Vert^2_{W_{\dot k}(k)}}{s_{\dot k}^2}\\
&+\Vert p_{C_R}(q_{k_c})-\hat p_b(t_c)\Vert^2_{W_p}
+w_a\Vert e_a(q_{k_c})\Vert^2\\
&+\sum_{k\in\mathcal K_c}\Big(\Vert\hat v_b(t_k)-v_{C_R,k}\Vert^2_{W_{v,k}}
+w_{path}\Vert P_{\perp,k}\big(p_{C_R}(q_k)-\hat p_b(t_k)\big)\Vert^2\Big)\\
&+\rho s_v+\rho_\tau\sum_k\mathbf 1^\top s_{\tau,k}+\rho_d\sum_{k,j}s_{d,kj}\\
\text{s.t.}\quad
&x_0=\hat x(t_s)\quad(\text{직전 게시 궤적을 }t_s\text{에서 평가}),\qquad \mathbf u=E\tilde{\mathbf u},\\
&q_{\min}+m_q\le q_{k}\le q_{\max}-m_q,\quad
|\dot q_{k}|\le\eta_v\dot q_{\max},\\
&\big|\tau^{lin} _ k\big|\le\eta' _ \tau\tau_{\max}+s_{\tau,k},\quad s_{\tau,k}\ge0
\quad(\text{활성 관절 전체},\enspace k=0,\dots,N),\\
&d_j(\bar q_k)+\nabla d_j^\top\delta q_k\ge d_{safe}+m_j+\tfrac12\Vert\dot{\bar q} _ k\Vert_\infty\Delta c_j-s_{d,kj},\quad s_{d,kj}\ge0
\quad(j\in\mathcal P_A\cup\mathcal P_B),\\
&d^{ball} _ L(\bar q_k,t_k)+\nabla d^{ball\top} _ L\delta q_k\ge r_b+d_{safe}^{ball}+m^{ball} _ k\quad(k\le k_c),\\
&\Vert\hat v_b(t_c)-v_{C_R,k_c}\Vert_\infty\le v_{rel,\mathrm{allow}}+s_v,\quad s_v\ge0,\\
&\dot q_{N}=0,\quad\ddot q_{N}=0,\qquad
\Vert q_{k}-\bar q_{k}\Vert_\infty\le\delta_{tr}.
\end{aligned}}
$$

여기서 손 속도는 §1.2 의 선형화 값이고, 가중은 다음과 같다.

$$
\begin{aligned}
v_{C_R,k}&=J^v_{C_R}(\bar q_k)\dot q_k+H_v(\bar q_k,\bar{\dot q} _ k)\delta q_k,\\
W_{v,k}&=w_\parallel\hat d_k\hat d_k^\top+w_\perp P_{\perp,k},\qquad w_\perp\gt w_\parallel,\\
w_\Delta&=w_{\Delta,0}\min\Big(1,\enspace\frac{\mathrm{tr}\Sigma_p(t_c)}{\sigma_{ref}^2}\Big).
\end{aligned}
$$

각 항의 역할:

| 항 | 역할 | 비고 |
|---|---|---|
| $\Vert u_k\Vert^2_R$ | jerk 평활 | $R$ 단위 $(\mathrm{rad/s^3})^{-2}$, 왼팔 블록을 더 크게 |
| $w_\Delta\Vert q-q^{prev}\Vert^2$ | 직전 계획과의 일관성 | 예측 공분산에 비례한다. 예측이 불확실할 때는 흔들림을 막고, 포구 직전 예측이 정확해지면 보정을 허용한다. $q^{prev}$ 는 shift 한 직전 해다 |
| $w_w\Vert\dot q_w\Vert^2$ | waist 운동 억제 | waist가 $v_{dir}$의 지렛대이므로 과하게 잡지 않음. 대조군: waist 고정 |
| $\Vert q_L-q_L^{rest}\Vert^2_{W_L^{rest}}$ | **왼팔 자세 유지 (비용)** | v0.1의 고정을 대체. $W_{\dot k}=0$이면 왼팔은 충돌 회피 외에는 대기 자세에 머문다 |
| $J_{\dot k}$ | **각운동량 변화율** | 왼팔 운동을 만드는 항이다 (§1.4). $W_{\dot k}(k)$는 $k\lt k_c$ 작게, $k\ge k_c$ 크게 스케줄 |
| $\Vert p_{C_R}-\hat p_b\Vert^2_{W_p}$ | 포구 위치 | $W_p=\kappa(\Sigma_p(t_c)+\sigma^2_{trk}I)^{-1}$ — 예측 공분산 가중. **이 문서의 선택**이며 같은 형태를 쓰는 포구 논문은 찾지 못했다. 읽은 시스템은 예측 평균 위에서 계획하고 빠른 재계획에 기댄다 [Bauml2010] [Dong2020] [Abeyruwan2023] |
| $w_a\Vert e_a\Vert^2$ | 접근축 | roll은 무관 |
| $\Vert\hat v_b-v_{C_R}\Vert^2_{W_v}$ | 상대속도 | **포구 구간 전체** $\mathcal K_c$ 에 건다 — 손가락이 닫히는 동안 손이 공과 함께 움직인다 [Salehian2016] [Lampariello2011]. 공 진행 방향과 수직 방향의 가중을 나눈다 [Abeyruwan2023]. 수직 성분은 손을 공 경로 밖으로 밀고, 진행 방향 성분은 손이 흡수한다. $\gamma$ · $v_{dir,\max}$는 게이트가 아니라 해의 결과 |
| $w_{path}\Vert P_\perp(\cdot)\Vert^2$ | 경로 이탈 | 포구 구간에서 손이 공 경로에서 옆으로 벗어나는 것 |
| $\rho s_v$ | 손 흡수 한계 slack | $v_{rel,\mathrm{allow}}$ 초과를 벌점. 관절 속도 한계 때문에 완전한 속도 일치는 대개 불가능하다 — [Lampariello2011] 은 손 속도가 목표 속도의 5 % 였다고 보고한다. $v_{rel,\mathrm{allow}}$ 는 손의 흡수 능력에서 정한다 |
| 토크 행 | **가속 제약** (계획 MD-7) | 활성 관절 전체에 건다. 유도 가속 box 는 쓰지 않는다. waist 행에는 두 팔의 중력과 **왼팔 counter-swing의 반력** 모멘트가 들어간다. G1 의 토크 한계는 관절마다 5–88 N·m 로 다르다 (손목이 가장 작다). armature 는 로봇 config 에서 읽고 sim 에서는 MJCF 의 값과 같게 둔다 — G1 URDF 에는 armature 가 없다 |
| 자기충돌 | $\mathcal P_A$ 는 두 팔 기울기, $\mathcal P_B$ 는 waist 기울기 추가 | 선형화 거리 제약은 [Faverjon1987] [Schulman2014]. 노드 사이 여유는 아래 |
| 공–왼팔 | $k\le k_c$ | $r_b$ 공 반지름, $m^{ball} _ k$는 $\Sigma_p(t_k)$ 기반 여유 (예: $n_\sigma\sqrt{\lambda_{\max}\Sigma_p}$) |
| $\dot q_N=\ddot q_N=0$ | 이 계획이 정지로 끝난다는 조건 | $t_c$ 이후 정지 구간 포함. 왼팔도 정지. 다음 주기의 실행 가능성은 보장하지 않는다 (아래). 종단 등식의 근거는 [Mayne2000], 포구 궤적에 쓴 사례는 [Lampariello2011] [Dong2020] |
| trust region | 선형화 유효 범위 | |

비용은 $\tilde{\mathbf u}$ 의 2차식과 slack 의 1차식이고 제약은 선형이므로 문제는 QP다.

**slack.** hard 로 두는 것과 slack 을 두는 것을 나눈다 (사용자 승인 2026-09-29).

| 제약 | 형태 | 이유 |
|---|---|---|
| 위치 · 속도 한계 | hard | 선형화가 없어 주기 사이에 움직이지 않는다 |
| 종단 등식, trust region | hard | 계획의 정의다 |
| 토크 행 | slack $s_{\tau,k}$ | 재선형화로 행이 움직인다. 실제 한계는 CLIK 이 다시 건다 |
| 자기충돌 | slack $s_{d,kj}$ | 재선형화로 행이 움직인다 |
| 공–왼팔 | hard | 왼팔은 포구에 필요하지 않으므로 물러나면 된다 |
| 상대속도 | slack $s_v$ | 완전 일치가 대개 불가능하다 |

- 벌점은 1차 (exact penalty) 다. $\rho_\tau$ · $\rho_d$ 를 크게 두면 실행 가능한 경우 slack 은 0 이다.
- slack 이 0 이 아닌 노드는 진단으로 남긴다. 자기충돌 slack 이 $m_j$ 를 넘으면 그 계획은 게시하지 않는다.
- 동역학을 뺀 모든 제약을 slack 으로 완화한 사례가 [Heins2023] 이다.

**노드 사이 여유.** 충돌은 노드에서만 검사하므로 노드 사이의 이동을 여유로 뺀다.

- $c_j$ 는 쌍 $j$ 의 최근접점까지의 lever arm 을 관절에 대해 합한 값의 상한이다 (m/rad). 관절 하나가 1 rad 돌 때 최근접점이 움직이는 거리의 상한이다.
- 양쪽 노드가 모두 제약되므로 노드 사이에서 벗어나는 양은 구간 절반의 이동이다. 그래서 계수가 $\tfrac12$ 다. 이 계수는 이 문서의 계산이다.
- 이 여유는 속도에 비례해 커진다. 팔–팔 제약이 자주 실행 불가능해지면 연속한 두 노드의 swept volume 에 제약을 거는 방법으로 바꾼다 [Schulman2014] `[선택]`.
- capsule 쌍의 거리는 두 축이 평행할 때 기울기가 연속이 아니다. 기울기가 연속인 형상은 [EscandeSTP2014].

**한계 여유.** $\eta_v,\eta' _ \tau$ 는 CLIK 한계보다 작게 둔다. 이것은 여유이지 포함 관계가 아니다 — MPC 의 실행 가능 집합이 CLIK 의 실행 가능 집합 안에 든다는 것은 $\eta\lt1$ 만으로 나오지 않는다.

- CLIK 은 feedforward 위에 오차 되먹임 $K e$ 를 더한다. 속도 쪽 조건은 관절마다 $\vert J^{+}Ke\vert\le(1-\eta_v)\dot q_{\max}$ 다.
- 토크는 상태에 의존한다. MPC 는 계획값 $(\bar q,\bar{\dot q})$ 에서, CLIK 은 명령값 $q_c$ 에서 평가한다.
- 따라서 $\eta$ 는 **실측 추종 오차**에서 정한다. v1 은 `ref_saturated` 가 tennis 시행 200 중 165 에서 발생했으므로 (계획 §3.2) CLIK 의 포화는 드문 일이 아니다.
- 외란 한계에서 제약 축소량을 유도하는 방법은 tube MPC [Mayne2005] 다. 이 문서는 그것을 쓰지 않는다.

**종단 정지 제약의 위상.** 종단 등식은 재귀적 실행 가능성의 고전적 수단이다 [Mayne2000]. 그 논증은 직전 해를 한 노드 민 것이 다음 문제의 실행 가능해라는 데 기댄다. 이 문서의 구성은 그 전제를 세 곳에서 깬다.

1. 고정된 move blocking — 민 해가 결정변수로 표현되지 않는다.
2. 재선형화된 제약 — 토크 · 충돌 행이 기준점과 함께 움직인다.
3. 매 주기 바뀌는 공 예측 — 포구 비용의 목표가 바뀐다.

그래서 종단 제약은 "게시하는 계획 하나가 정지로 끝난다" 만을 뜻한다. 제동의 안전망은 MPC 밖에 있다 (사용자 승인 2026-09-29).

| 상황 | 동작 |
|---|---|
| QP 가 실행 불가능하거나 예산을 넘김 | 새 계획을 게시하지 않는다. RT 는 직전 계획을 계속 따른다 (직전 계획도 정지로 끝난다) |
| 직전 계획이 없거나 나이 한계를 넘음 | v1 의 closed-form DECEL (`EvaluateDecelTarget`) |
| `ABORT_SAFE` | 관절 공간 정지 `JointSpaceDecelStep` — 원인과 무관, QP 독립 (v1 C-35, 불변) |

**바깥 루프.** 포구 시각 후보를 열거하고 후보마다 QP 를 푼다. 발표된 포구 시스템은 포구 시각을 연속 변수로 둔다 [Bauml2010] [Lampariello2011] [Dong2020] [Abeyruwan2023] — 열거는 이 문서의 선택이다. 볼록 QP 라 그 논문들이 보고하는 국소 최소 문제가 없다.

- **후보.** vision 예측점의 시각이다. 간격은 $\Delta_v$ 이고 v1 의 계획기와 같다. MPC 격자가 $t_c$ 에 고정돼 있어 후보 간격은 $\Delta$ 와 무관하다.
- **시각 정밀도.** 팔 궤적의 포구 시각은 후보 격자에 묶이지만, 손 폐쇄 명령 시각 $t_{cmd}$ 는 손 시퀀서가 따로 정한다. 포구 구간 $\mathcal K_c$ 의 상대속도 · 경로 비용이 시각 오차의 허용 폭을 넓힌다. 문헌이 드는 수 ms 의 정밀도 [Bauml2010] [Dong2020] 는 팔과 손을 한 시각으로 묶은 시스템의 값이다. 후보 간격의 영향은 §1.7 의 sweep 으로 잰다.
- **사전 거르기.** 후보가 늘면 후보마다 QP 를 풀 수 없다. 도달 가능성으로 미리 걸러 한 주기에 푸는 후보 수를 상한 $K_{\max}$ 로 묶는다 (v1 의 `planner.max_ik` 와 같은 역할).
- **초기화.** 새 후보의 기준점은 가장 가까운 기존 후보의 해를 그 후보의 격자에서 다시 평가한 것이다.
- **교체.** 한 주기 이상 풀린 후보만 교체 대상이다. 수렴 단계가 다른 비용을 비교하지 않기 위해서다. 교체는 비용 차에 히스테리시스를 둔다.
- **순위.** 후보의 순위는 **공통 고정 가중**으로 매긴다 — 포구 위치 항을 $W_p$ 대신 $W^{rank} _ p$ 로 다시 평가한 비용이다. $W_p$ 는 불확실성이 큰 후보일수록 작아지므로, 그대로 비교하면 같은 오차가 불확실한 후보에서 더 싸게 보인다. $W^{rank} _ p$ 는 손의 포획 범위에서 정한다 (접근축에 수직인 방향은 손 벌림 폭, 접근축 방향은 깊이).
- **COMMITTED 이후.** $t_c$ · $t_{cmd}$는 고정하고 $\hat p_b(t_c)$ 갱신에 따른 궤적 재계획만 계속한다. 고정된 $t_c$ 가 예측점 사이에 놓이면 공 예측은 v1 L2 의 보간 (`SampleAt`) 으로 얻는다.

**연속성.** $x_0$를 직전 게시 궤적의 $t_s$ 값으로 두므로 전환 시점에 전체 $(q,\dot q,\ddot q)$가 구성상 이어진다 — 왼팔 포함. RT 채택 규칙은 token 일치·게시 나이· $t_s$ 기준 전환 셋뿐이다. 이 구성에서 MPC 는 측정 상태를 되먹이지 않는다 — 기준 궤적을 다시 계획할 뿐이고, 로봇 상태의 되먹임은 CLIK 에만 있다.

### 1.4 각운동량 항의 위상 스케줄과 우선순위

중심 운동량 행렬의 정의는 [Orin2008] [Orin2013] 이다.

**이 항의 목적** (사용자 결정 2026-09-29).

1. **왼팔의 운동을 만든다.** 이 항이 없으면 왼팔은 대기 자세에 머문다. 포구에 참여하지 않는 팔에 물리적으로 뜻이 있는 운동을 주는 수단이다.
2. **floating base 로 확장한다.** 지금은 다리와 골반이 고정이지만 이후 floating base 로 넓힌다. 그때 각운동량 변화율은 균형에 직접 걸리는 양이다 [Kajita2003] [Wensing2016]. 고정 베이스 단계에서 항과 선형화, 스케줄을 먼저 검증한다.

**고정 베이스에서의 한계.** 지금 단계에서 이 항이 하지 못하는 것을 적어 둔다.

- 강체 고정 베이스에서 각운동량 변화율은 베이스가 받는다. sim 에서 손의 정확도에는 영향이 없다. 따라서 **이 항의 효과는 포구 성공률로 재지 않는다** — 각운동량 변화율의 크기와 포구 항의 변화로 잰다 (§4 항목 7).
- 이 양은 질량 중심에 대한 모멘트다. waist 나 베이스가 받는 모멘트와 같지 않다 — 그쪽에는 선운동량 변화율의 모멘트와 중력 모멘트가 더 있다. waist 의 부하는 토크 행이 맡는다.
- 몸통 반력이 실제로 문제가 되는 경로는 진동과 카메라 흔들림이다 [Bauml2011]. 강체 sim 에는 없다.
- 고정 베이스 모델의 중심 운동량 행렬은 움직이는 물체만 합한 값이다. floating base 로 가면 다리와 골반이 합에 들어오고 열이 6개 늘어난다. 항의 형태는 같다.

waist 의 부하를 직접 줄이려면 waist 토크를 비용으로도 쓴다 `[선택]`. 제약용으로 이미 계산하는 양이다.

$A^\omega_G$는 $3\times n$이라 영공간이 넓다. 오른팔이 공을 향해 뻗는 각운동량은 불가피하므로, 항이 줄일 수 있는 것은 (i) 왼팔·waist의 상쇄 운동, (ii) 오른팔 자체의 감속뿐이다. (ii)는 포구와 정면 충돌하므로 두 가지로 우선순위를 준다.

1. **시간 스케줄.** $W_{\dot k}(k)=W_{\dot k}^{-}$ ($k\lt k_c$, 작게), $W_{\dot k}^{+}$ ($k\ge k_c$, 크게). 포구 전에는 상대속도가 우선이고, 포구 후 정지 구간은 시간 제약이 느슨해 각운동량을 줄이며 멈출 여유가 있다. 공 운동량 유입 $r\times m_bv_{rel}$도 이 구간에서 흡수된다 ($m_b\approx0.05$ kg이라 작다).
2. **사전적(lexicographic) 처리 `[선택]`.** 포구 항만으로 푼 최적 비용 $J^\ast_{catch}$에 대해 $J_{catch}\le(1+\epsilon)J^\ast_{catch}$ 제약 아래 $J_{\dot k}$를 최소화한다. QP 두 번이지만 "포구를 $\epsilon$ 이상 희생하지 않는다"가 보장된다. 엄격한 계층 해법은 [Escande2014].

$t_c$ 이후 실제 실행은 기존 DECEL 법칙(L7 §4.3)이 맡으므로, 정지 구간의 각운동량 최소화를 실제로 반영하려면 DECEL 중에도 RT가 MPC 궤적의 꼬리를 따르도록 바꿔야 한다. 이 변경이 계획 E1-F04 이고, L7 전이 동작 변경이라 E-8 검토 대상이다. 단일 팔에서의 형태는 §1.6 에 있다.

Pinocchio `computeCentroidalMap`·`computeCentroidalMapTimeVariation`의 `RtModelHandle` 노출과 할당 0은 `[확인 필요]`.

### 1.5 출력 (RT로 게시)

계획기는 **관절 해만** 게시한다. 손의 pose 와 twist 는 게시하지 않는다.

$$
\begin{aligned}
\text{노드 }k=0,\dots,N\text{: }&\enspace q_k,\enspace\dot q_k,\enspace\ddot q_k\quad(\text{전체 }n),\\
\text{그 외: }&\enspace t_c,\enspace t_{cmd},\enspace\gamma_f=\hat v_b^\top J^v_{C_R}\dot q_{k_c}/\Vert\hat v_b\Vert^2,\enspace\text{token}.
\end{aligned}
$$

**RT 평가.** RT 는 now_lead 가 속한 구간 $[t_k,t_{k+1})$ 에서 $\tau=t-t_k$ 로 관절 기준을 닫힌식으로 평가한다. jerk 는 노드의 가속에서 나온다.

$$
\begin{aligned}
u_k&=(\ddot q_{k+1}-\ddot q_k)/\Delta,\\
q_{ref}(t)&=q_k+\dot q_k\tau+\tfrac12\ddot q_k\tau^2+\tfrac16u_k\tau^3,\\
\dot q_{ref}(t)&=\dot q_k+\ddot q_k\tau+\tfrac12u_k\tau^2 .
\end{aligned}
$$

손의 목표는 그 관절 기준에서 FK 로 만든다.

$$
\begin{aligned}
\text{오른손 (world): }&\enspace T^d_R=T_{WC_R}(q_{ref}),\quad V^{ff} _ R={}^WJ_{C_R}(q_{ref})\dot q_{ref},\\
\text{왼손 (몸통): }&\enspace T^d_L=T_{TC_L}(q_{L,ref}),\quad V^{ff} _ L={}^TJ_{C_L}(q_{ref})\dot q_{ref} .
\end{aligned}
$$

- CLIK 입력 형식 (pose + twist feedforward) 은 그대로다.
- 손의 목표와 자세 기준이 같은 $q_{ref}$ 에서 나오므로 서로 정확히 일치한다. 제약이 비활성이고 $q_c=q_{ref}$ 이면 $v^\ast=\dot q_{ref}$ 가 세 과제의 잔차를 모두 0 으로 만든다.
- 회전벡터 보간이 없으므로 각속도와 회전벡터 미분의 구분, world 축과 body 축의 변환이 필요 없다. v0.2 의 5차 Hermite 보간은 이 구분을 빠뜨렸다 [Sola2018] [Zefran1998].
- $V^{ff}$ 는 Jacobian 없이 속도를 포함한 FK 로 얻을 수 있다.
- RT 는 tick 마다 $q_{ref}$ 에서 FK 를 한 번 더 한다 (CLIK 은 $q_c$ 에서 이미 한다). 이 비용이 tick 예산에 드는지는 `[확인 필요]`.

여유 자유도 (전체 $n$ 대 두 과제 12행) 를 CLIK 이 계획과 같게 채우려면 $q_{ref}$ 전체가 자세 과제로 들어가야 한다 — 왼팔 counter-swing 과 waist 분배는 손의 목표만으로는 전달되지 않는다.

payload 는 노드당 관절 $3n$ double 이다. $n=17$, $N=20$ 이면 $3\cdot17\cdot21\cdot8$ byte, 약 8.6 KB 다. `PlanSnapshot` 의 기존 필드는 유지하고 노드 payload 를 추가한다 (SeqLock POD, D-21 소비 규약).

### 1.6 단일 팔 · 정지 구간 환원형 (계획 E1)

계획 E1 이 푸는 문제다. ur5e_p1b ($n=6$) 와 iiwa7_leap ($n=7$) 의 DECEL 을 v1 의 TCP 직선 등감속 대신 관절 공간 MPC 로 계획한다. §1.3 에서 다음을 빼면 얻는다.

| §1.3 의 요소 | 환원형 |
|---|---|
| 활성 관절 | 팔만 ($n_w=n_L=0$). 손은 활성 관절 밖 |
| 포구 항 (위치 · 접근축 · 상대속도 · slack) | 없음 — 정지 구간은 포구 뒤다 ($k_c=0$) |
| 왼팔 rest, waist 억제, 각운동량 | 없음 |
| 충돌 제약 | 없음 (v1 과 같음) |
| 토크 행, 위치 · 속도 한계, 종단 정지 | 유지 |

$$
\boxed{
\begin{aligned}
\min_{\tilde{\mathbf u}}\quad
&\sum_{k=0}^{N_s-1}\Vert u_k\Vert_{R}^2
+w_\Delta\sum_{k}\Vert q_{k}-q^{prev} _ {k}\Vert^2
+w_\perp\sum_k\Vert P_\perp\big(p_{C}(\bar q_k)+J^v_{C}(\bar q_k)\delta q_k-p_c\big)\Vert^2
+\rho_\tau\sum_k\mathbf 1^\top s_{\tau,k}\\
\text{s.t.}\quad
&x_0=\hat x(t_c),\qquad \mathbf u=E\tilde{\mathbf u},\\
&q_{\min}+m_q\le q_{k}\le q_{\max}-m_q,\quad
|\dot q_{k}|\le\eta_v\dot q_{\max},\\
&\big|\tau^{lin} _ k\big|\le\eta' _ \tau\tau_{\max}+s_{\tau,k},\quad s_{\tau,k}\ge0,\\
&\dot q_{N_s}=0,\quad\ddot q_{N_s}=0,\qquad
\Vert q_{k}-\bar q_{k}\Vert_\infty\le\delta_{tr}.
\end{aligned}}
$$

| 기호 | 뜻 |
|---|---|
| $N_s$, $\Delta_s$ | 정지 구간의 노드 수와 간격. 이 절에서는 $\Delta$ 자리에 $\Delta_s$ 를 쓴다 |
| $x_0=\hat x(t_c)$ | DECEL 진입 시각의 기준 상태 $(q,\dot q,\ddot q)$. 진입 직전의 기준과 이어져야 한다 (v1 L7 G7-B) |
| $p_c$, $P_\perp$ | 포구점과, 포구 순간의 손 속도 방향에 수직인 성분을 뽑는 사영 $I-\hat d\hat d^\top$ |
| $w_\perp$ | 손이 v1 의 직선 경로에서 옆으로 벗어나는 것에 대한 가중 — 선택 항이며 0 이면 경로는 자유다 (사용자 승인 2026-09-29) |

- 비선형은 토크 행과 $w_\perp$ 항의 FK 뿐이다. 토크 행은 §1.2 의 1차 선형화이고 slack 은 §1.3 과 같다.
- 손은 기준 자세로 잠근 축소 모델이다 (§0.1). p1b 는 폐쇄 체인 손이라 이 처리가 필요하다.
- v1 의 정지 시간은 포구 속도를 감속도 (`a_dec`) 로 나눈 값이라 vision 간격보다 짧을 수 있다. 그래서 $\Delta_s$ 는 $\Delta$ 와 따로 정한다 (사용자 승인 2026-09-29). $N_s$ · $\Delta_s$ 의 값은 E1-F01 의 spec 에서 정한다 `[확인 필요]`.
- 정지 구간은 포구 전에 미리 계산한다 (계획 E1-F03). $x_0$ 를 포구 전에 어떻게 예측하는지는 그 spec 에서 정한다 `[확인 필요]`.
- 출력과 RT 평가는 §1.5 와 같다. DECEL 에서는 soft-catch DS 를 건너뛰고 $q_{ref}$ 에서 만든 pose · twist 와 관절 기준을 CLIK 에 넣는다 (계획 MD-3).
- 안전망은 §1.3 의 표와 같다. v1 의 closed-form DECEL 이 기본값으로 남고 MPC 는 YAML 로 켠다.
- 비교 기준은 계획의 게이트 G-1 이다.

### 1.7 예측 격자 sweep

포구 시각 후보의 간격 $\Delta_v$ 가 성능에 주는 영향을 잰다 (사용자 지시 2026-09-29). 예측점의 수는 ball_perception 의 profile 에서 바꾼다. 조건은 기존 시스템의 제약 안에서 정한다.

horizon 은 0.75 s 와 1.0 s 둘을 시험한다 (사용자 지시 2026-09-29). horizon 을 나누는 이유는 추정기 (EKF) 의 정확도다 — 예측 오차는 horizon 이 길수록 커진다.

**기존 제약** (2026-09-29 코드 대조).

| 제약 | 값 | 출처 |
|---|---|---|
| 점 수의 상한 | 40 | rtc-framework 의 컴파일 상수 `kCap` |
| horizon 과 step | horizon 이 step 의 정확한 배수 (ns 단위 정수) | ball_perception `prediction.cpp` |
| 점 수와 `max_points` | horizon / step ≤ `max_points` | 같은 곳. profile 의 값이며 조건마다 바꾼다 |
| 지평 요구 | 첫 점에서 마지막 점까지의 길이가 `io.horizon_min` (0.51 s) 이상 | rtc-framework `traj_sampler.hpp` |
| 점 수의 하한 | $\lceil$ `io.horizon_min` $/\Delta_v\rceil+1$ | `io.n_min` 의 산식 |

예측점은 기준 시각에서 $\Delta_v$ 떨어진 곳부터 시작하므로, 첫 점에서 마지막 점까지의 길이는 horizon 이 아니라 horizon $-\Delta_v$ 다.

**현재 설정값.** profile 은 ball_perception 저장소에만 있다 (계획 MD-18, E0-F04 에서 이전 — ball_perception `9005aab`).

| 파일 (ball_perception) | horizon | step | 점 수 |
|---|---|---|---|
| `ball_perception_sim/config/sim_profile.catching.json` — sim 포구 시행이 읽는 profile | 1.0 s | 0.05 s | 20 |
| `ball_perception_sim/config/sim_profile.example.json` | 1.0 s | 0.05 s | 20 |
| `ball_perception_estimation/config/bearing_run_config.template.json` | 1.0 s | 0.05 s | 20 |

rtc-framework 에는 사본이 없다. 이전 전의 사본 (`integrated_bringup/config/<robot>/`) 은 `sim_profile.catching.json` 과 바이트가 같았다.

**조건 — horizon 1.0 s.** $10^9$ ns 를 나누어떨어지게 하는 점 수 가운데 20 이상 40 이하인 것은 넷이다.

| 조건 | 점 수 | $\Delta_v$ | 첫 점–마지막 점 | `io.n_min` |
|---|---|---|---|---|
| L-50 (rtc-framework 출하값) | 20 | 50 ms | 0.950 s | 12 |
| L-40 | 25 | 40 ms | 0.960 s | 14 |
| L-31 | 32 | 31.25 ms | 0.969 s | 18 |
| L-25 | 40 | 25 ms | 0.975 s | 22 |

**조건 — horizon 0.75 s.** $7.5\cdot10^8$ ns 를 나누어떨어지게 하는 점 수는 15, 16, 20, 24, 25, 30, 32, 40 이다. 이 가운데 1.0 s 조건과 간격이 같은 셋과 가장 촘촘한 하나를 쓴다.

| 조건 | 점 수 | $\Delta_v$ | 첫 점–마지막 점 | `io.n_min` |
|---|---|---|---|---|
| M-50 | 15 | 50 ms | 0.700 s | 12 |
| M-31 | 24 | 31.25 ms | 0.719 s | 18 |
| M-25 | 30 | 25 ms | 0.725 s | 22 |
| M-19 | 40 | 18.75 ms | 0.731 s | 29 |

- 간격 50 · 31.25 · 25 ms 는 두 horizon 에 모두 있다. 이 세 쌍이 **같은 간격에서 horizon 만 다른** 비교다.
- 40 ms 는 1.0 s 에만, 18.75 ms 는 0.75 s 에만 있다. 0.75 s 를 40 ms 로 나누면 정수가 아니고, 1.0 s 를 18.75 ms 로 나누면 점 수가 상한 40 을 넘는다.
- 두 horizon 모두 첫 점에서 마지막 점까지의 길이가 지평 요구 0.51 s 를 넘는다. **`io.horizon_min` 과 v1 의 commit 조건은 바꾸지 않는다.** 그래서 두 horizon 의 비교에 다른 요인이 섞이지 않는다.
- horizon 0.5 s 는 쓰지 않는다. 첫 점에서 마지막 점까지가 0.45 – 0.4875 s 라 지평 요구에 못 미치고, 맞추려면 commit 조건의 여유나 선행 시간을 줄여야 한다.

**조건마다 함께 바꾸는 설정.**

| 쪽 | 파일 | 키 |
|---|---|---|
| ball_perception profile | `sim_profile.catching.json` 의 사본 — 조건마다 따로 두고 출하 파일은 고치지 않는다 | `prediction.horizon_s`, `prediction.step_s`, `prediction.max_points` |
| rtc-framework | `demo_catching_controller.yaml` (로봇별) | `prediction.dt_expected`, `io.n_min`, `planner.slice.dt` |

- profile 만 촘촘하게 바꾸면 메시지는 받아들여진다. 거부 하한은 `prediction.dt_expected` 의 10 % (`kTrajSpacingFloorFraction`) 라 조건의 간격은 모두 그 위다. 대신 `planner.slice.dt` 가 남은 값이면 계획기가 후보를 그 간격으로 솎아 (`planner_search.cpp`) 경고 없이 옛 격자로 돈다. 세 키는 떠 있는 컨트롤러의 read-only 미러 파라미터로 확인한다.
- 두 쪽의 값을 맞춰 보는 자동 검사는 없다. profile 은 ball_perception 쪽이 소유하고 (계획 MD-18) rtc-framework 는 그 파일을 읽을 수 없다. 위 표의 키를 바꿀 때는 양쪽을 함께 확인한다.
- 조건별 설정은 출하값을 덮어쓰지 않고 조건마다 따로 둔다 — 두 저장소 밖의 profile 사본과 sim overlay 다 (계획 §8 E0-F04).
- 25 ms (1.0 s) · 18.75 ms (0.75 s) 보다 촘촘한 조건은 `kCap` 을 올려야 한다. `kCap` 은 스냅샷의 크기이고 스냅샷은 tick 마다 통째로 복사되므로 (v1 L2 §5), 올리면 RT 비용이 함께 는다. 이 sweep 에는 넣지 않는다 `[선택]`.

**바뀌는 것과 바뀌지 않는 것.**

| 양 | sweep 에서 |
|---|---|
| 후보 간격 $\Delta_v$ | 바뀐다 |
| 후보 수 | $\Delta_v$ 에 반비례해 늘어난다. 한 주기에 푸는 수는 $K_{\max}$ 로 묶인다 |
| MPC 노드 간격 $\Delta$, 결정변수 수 | 바뀌지 않는다 (§1.2 "격자") |
| horizon | 두 값 (0.75 s, 1.0 s) |
| 지평 요구 `io.horizon_min`, commit 조건 | 바뀌지 않는다 |
| 재계획 주기 $T_r$ | 바뀌지 않는다 |
| L2 보간의 입력 간격 | 바뀐다 |

**재는 것.**

| 지표 | 뜻 |
|---|---|
| 포구 성공률 | 조건별. 같은 투척의 paired 비교 |
| 포구 순간의 위치 · 상대속도 오차 | 후보 간격이 직접 주는 영향 |
| 선택된 $t_c$ 의 교체 횟수 | 후보가 촘촘하면 교체가 잦아질 수 있다 |
| 계획기 한 주기의 계산 시간 p99 | 사전 거르기의 비용 포함 |
| 예측 메시지의 크기와 발행 주기 | ball_perception 쪽 부하 |
| 포구 시각에서의 예측 위치 오차 | horizon 별. 추정기의 정확도가 horizon 에 따라 얼마나 달라지는지 |

**순서.**

1. v1 계획기로 여덟 조건을 먼저 돈다. v1 도 같은 예측 격자를 후보로 쓰므로 MPC 와 무관한 기준선이 생긴다.
2. MPC 계획기로 같은 투척을 돈다.
3. 시행 수와 판정 기준은 시행 전에 고정한다.

여덟 조건의 profile 은 모두 load 되고 수신 간격의 p50 은 30 Hz 로 유지되지만, 긴 간격 (≥ 0.10 s) 의 비율은 메시지 크기와 함께 는다. v1 기준선의 값은 계획 문서 §8 E0-F04 에 있다.

---

## 2. 다중 frame CLIK (RT tick)

### 2.1 오차 정의

명령값 $q_c$에서 평가한다 (q_c 평가 모드, D-6). 오른손은 world, 왼손은 몸통 frame이다.

$$
\begin{aligned}
e_{p,R}&=p^d_{R}-p_{C_R}(q_c),&
e_{o,R}&=R_{C_R}(q_c) \mathrm{Log}\big(R_{C_R}(q_c)^\top R^d_{R}\big),\\
e_{p,L}&={}^Tp^d_{L}-{}^Tp_{C_L}(q_{c,L}),&
e_{o,L}&={}^TR_{C_L} \mathrm{Log}\big({}^TR_{C_L}^\top {}^TR^d_{L}\big).
\end{aligned}
$$

목표 twist 의 구조 (feedforward + 오차 되먹임) 는 CLIK [Sciavicco1988] [Siciliano1990] 그대로다.

$e_o$는 body 회전벡터를 현재 자세로 회전한 **world-aligned(몸통-aligned)** 회전벡터라 LWA Jacobian의 각속도 행과 frame이 맞는다. 목표 twist는

$$
V^d_i=V^{ff} _ i+K_i e_i,\qquad e_i=(e_{p,i},e_{o,i}),\quad
K_i=\mathrm{diag}(K_{p,i}I_3,\enspace K_{o,i}I_3)\enspace[\mathrm{s^{-1}}].
$$

되먹임 항은 크기를 제한한다.

$$
V^d_i=V^{ff} _ i+\mathrm{sat}\big(K_ie_i,\enspace v_{fb,\max}\big).
$$

- $\mathrm{sat}$ 는 위치 블록과 자세 블록 각각의 norm 을 상한으로 줄인다. 방향은 유지한다.
- 상한에 걸린 tick 은 진단으로 남긴다. 상한이 걸린 동안 §2.3 의 오차 동역학은 성립하지 않는다.
- $\Vert e_o\Vert$ 가 $\pi$ 근처면 $\mathrm{Log}$ 의 축이 불연속이다. 이 경우 직전 tick 의 축을 유지하고 진단을 남긴다. MPC 궤적을 따르는 동안에는 생기지 않아야 하는 상태다.

왼손도 $V^{ff} _ L\ne0$이다. 두 손의 $T^d$ 와 $V^{ff}$ 는 RT 가 관절 기준에서 FK 로 만든 값이다 (§1.5).

### 2.2 QP

$$
\boxed{
\begin{aligned}
v^\ast=\arg\min_{v\in\mathbb R^n}\quad
&\big\Vert{}^WJ_{C_R}(q_c) v-V^d_R\big\Vert^2_{W_R}
+\big\Vert{}^TJ_{C_L}(q_c) v-V^d_L\big\Vert^2_{W_L}\\
&+\big\Vert v-\dot q_{n}\big\Vert^2_{W_n}
+w_s\Vert v-v_{prev}\Vert^2\\
\text{s.t.}\quad
&q_{\min}\le q_c+h v\le q_{\max},\qquad |v|\le\dot q_{\max},\\
&\Big|\frac{v-v_{prev}}{h}\Big|\le\ddot q_{\max}
\quad\text{또는}\quad
\Big|M(q_c)\frac{v-v_{prev}}{h}+h(q_c,v_{prev})\Big|\le\eta_\tau\tau_{\max}\enspace(\texttt{dynamic}),
\end{aligned}}
$$

$$
\dot q_n=\dot q_{ref}(t)+K_q\big(q_{ref}(t)-q_c\big)\quad(\text{전체 }n).
$$

해를 적분해 명령을 만든다: $q_c\leftarrow\mathrm{Integrate}(q_c,\enspace h v^\ast)$. 이것을 waist·좌팔·우팔 device slot에 position으로 싣는다.

- 가중 우선순위 $W_R\gg W_L\gg W_n$.
- 왼손 과제는 $q_L$ 열만 갖는다. 자세 과제는 정칙화 겸 영공간 해결이라 $W_n$이 DLS의 $\lambda^2 I$ 역할을 대신한다.
- `dynamic` 토크 제약은 전체 $M,h$로 걸리므로 왼팔 counter-swing의 반력이 waist 행에 자동으로 들어간다 — MPC의 토크 행과 같은 물리다.
- 엄격한 계층이 필요하면 HQP로 바꾸되 [Kanoun2011] [Escande2014], 기존 `ClikReferenceGenerator`는 가중 최소제곱 형태다.
- 위치 · 속도 · 가속 한계를 tick 마다 따로 걸면 서로 양립하지 않을 수 있다 [DelPrete2018] [Faroni2018]. 아래 "제동 거리 한계" 가 그 대응이다.
- waist 를 움직이면서 두 번째 손을 몸통 기준으로 두는 G1 용 속도 수준 QP IK 의 공개 구현은 찾지 못했다. 같은 구조의 공개 구현 pink (§8) 를 golden 회귀의 수치 기준으로 쓴다.
- `[확인 필요]` 확장 CLIK가 (i) 두 번째 frame 과제, (ii) 자세 과제 $\dot q_n$(또는 동등한 영공간 기준)을 받는지. 없으면 §2.2는 `rtc_tsid` 일반화 범위이고 게이트는 §4 항목 4의 골든 회귀다.

**제동 거리 한계** (opt-in, 사용자 승인 2026-09-29). 위치 box $q_{\min}\le q_c+hv\le q_{\max}$ 는 한 tick 앞만 본다. 한계 가까이에서 빠르게 움직이면 가속 · 토크 한계 안에서 멈출 수 없어 QP 가 실행 불가능해진다. 멈출 수 있는 속도로 묶는다 [DelPrete2018] [Flacco2015].

$$
-\sqrt{2a_{brk}(q_c-q_{\min})}\le v\le\sqrt{2a_{brk}(q_{\max}-q_c)} .
$$

- $a_{brk}$ 는 관절별 제동 감속도다. 토크 제약 아래에서는 낼 수 있는 가속이 상태에 따라 달라지므로 보수적인 값으로 둔다.
- 기본값은 꺼짐이다. 끄면 기존 `ClikReferenceGenerator` 의 동작 (1-step box, 충돌 시 `bound_conflict` 와 직전 값 유지) 그대로다. 기존 로봇의 golden 회귀는 꺼진 상태로 판정한다.
- `rtc_tsid` 에는 가속 수준의 같은 제약 `JointLimitConstraint` 가 있다. 속도 수준으로 옮겨 쓴다.

**충돌 damper** `[선택]` (기본 꺼짐). 오차 되먹임이 팔을 계획 경로 밖으로 밀 수 있으므로 tick 주기 QP 에도 충돌 제약을 둘 수 있다 [Faverjon1987] [Stasse2008]. 거리가 영향 거리 $d_i$ 아래인 쌍에 대해

$$
\nabla d_j(q_c)^\top v\ge-\xi\frac{d_j(q_c)-d_s}{d_i-d_s}\qquad(d_j\lt d_i).
$$

- $d_s$ 는 안전 거리, $\xi$ 는 접근 속도의 상한이다.
- 거리 계산이 tick 마다 든다. 충돌 코어 (계획 E3-F03) 가 선행이고, 검사할 쌍은 MPC 가 활성으로 표시한 것으로 줄인다.
- 꺼 둔 동안의 대응은 MPC 의 충돌 여유 $m_j$ 다.

### 2.3 폐루프 오차 동역학

제약이 비활성이고 $W_n\to0$, 왼손 과제가 $q_L$ 열만 쓰는 상황에서 오른손 과제만 보면 $J_Rv=V^{ff} _ R+K_Re_R$이 정확히 풀린다 ($J_R$ 행 full rank). 참조가 $\dot p^d_R=V^{ff,v} _ R$이므로

$$
\dot e_{p,R}=\dot p^d_R-J^v_Rv=V^{ff,v} _ R-(V^{ff,v} _ R+K_{p,R}e_{p,R})=-K_{p,R} e_{p,R}.
$$

왼손도 같은 식으로 $\dot e_{p,L}=-K_{p,L}e_{p,L}$이며, 두 과제의 열이 $q_L$에서만 겹치지 않고(오른손 과제 $q_L$ 열 0) 자세 과제가 나머지를 채우므로 서로 간섭하지 않는다. 

자세는 목표가 정지해 있으면 $\dot e_o=-K_oe_o$ 가 근사가 아니라 정확히 성립한다 — 되먹임 각속도가 오차 회전의 축과 나란해 오차는 그 축을 따라서만 줄어든다. 목표가 움직이면 feedforward 각속도를 현재 자세와 목표 자세 중 어느 쪽 기준으로 쓰느냐의 차이에서 잔차가 생기고, 그 크기는 $\Vert e_o\Vert \Vert\omega^d\Vert$ 규모다. 포구 운동은 $\omega^d$ 가 크므로 이 잔차를 추종 오차 예산에 넣는다. 이 문단은 이 문서의 유도이며 문헌으로 대조하지 않았다.

유효 조건은

1. $J_R$이 waist + 우팔 열에서 rank 6, $J_{L,L}$이 rank 6,
2. 제약이 활성화되지 않는 것 — MPC 의 한계 여유가 되먹임 항까지 담을 만큼 클 때다 (§1.3 "한계 여유").

제약이 활성이면 이 보장은 사라지고 `REF_SATURATED` 감시가 잡는다.

**이산 tick.** 위 식은 연속 시간 결과다. 명령은 tick $h$ 마다 적분되므로 실제 점화식은 1차 근사로

$$
e_{i+1}=(1-K h) e_i+O(h^2)
$$

이다. 수렴 조건은 $0\lt Kh\lt2$, 진동 없이 줄어드는 조건은 $Kh\le1$ 이다. 1 kHz tick 에서 $K$ 는 1000 $\mathrm{s^{-1}}$ 미만이어야 하고, 실제 이득은 이보다 훨씬 작게 둔다. 이 조건은 제약이 비활성인 경우의 유도다. 여유 자유도가 있는 CLIK 의 이산 시간 안정성은 [Falco2011] 이 다룬다.

---

## 3. MPC ↔ CLIK 계약

| 항목 | MPC 쪽 | CLIK 쪽 |
|---|---|---|
| 오른손 | 게시하지 않음 | RT 가 $q_{ref}$ 에서 FK 로 $T^d_R,V^{ff} _ R$ 를 만든다. world LWA 6행 과제 |
| 왼손 | 게시하지 않음 | RT 가 $q_{L,ref}$ 에서 FK 로 $T^d_L,V^{ff} _ L$ 를 만든다. 몸통 frame 6행 과제 — waist 열 0 |
| 관절 기준 | 노드별 $q_k,\dot q_k,\ddot q_k$ (전체 $n$) — 유일한 궤적 payload | 닫힌식으로 $q_{ref},\dot q_{ref}$ 평가. 자세 과제 $\dot q_n$ — waist 분배·왼팔 counter-swing의 유일한 전달 경로 |
| 한계 | $\eta_v,\eta' _ \tau\lt1$ — 실측 추종 오차에서 정한 여유 | 정격· $\eta_\tau=0.8$ (`dynamic`, 전체 $M,h$) |
| 시간 | 노드 $t_s+k\Delta$, 격자는 $t_c$ 에 고정, 절대 steady ns (D-2) | now_lead에서 닫힌식 평가 |
| 연속성 | $x_0$ = 직전 궤적의 $t_s$ 값 (전체 $n$) | 채택 규칙: token·나이· $t_s$ |
| 손·FSM | $t_c,t_{cmd},\gamma_f$ (기존 필드) | 손 시퀀서·슈퍼바이저 불변 (왼손은 $q_{pre}$ / $q_{open}$ 유지) |

---

## 4. Sanity check

1. **차원.** $n_w=3,\enspace n_L=n_R=7$이면 $n=17$, 과제 행 12, 영공간 5 → 자세 과제 없이는 CLIK QP가 유일해가 아니다. MPC 결정변수 $nN=340$; move blocking(오른팔·waist $k\lt k_c$ 1노드, 나머지 3노드)으로 대략 $n_wk_c+n_Rk_c+\lceil(N-k_c)/3\rceil(n_w+n_R)+\lceil N/3\rceil n_L$ ≈ 170–210. 부등식 행은 move blocking 으로 줄지 않는다 (§6 M-6).
2. **단위.** $e_p$ [m], $e_o$ [rad], $K$ [1/s], $v$ [rad/s]. $W_R,W_L$의 위치·자세 블록은 각각 $\mathrm{m}^{-2},\mathrm{rad}^{-2}$ 기준 무차원화. MPC의 $R$은 $(\mathrm{rad/s^3})^{-2}$, $W_p$는 $\mathrm{m}^{-2}$, $W_v$는 $(\mathrm{m/s})^{-2}$, $W_L^{rest}$는 $\mathrm{rad}^{-2}$, $s_{\dot k}$는 N·m (waist 허용 토크의 일정 비율).
3. **극한 — 왼팔 고정.** $W_L^{rest}\to\infty$(또는 왼팔 box 폭 0), $W_{\dot k}=0$이면 v0.1 정식화($q_L\equiv q_L^{rest}$)로 환원되고, 자기충돌은 왼팔 capsule 상수·공–왼팔 거리는 $q_w$만의 함수가 된다.
4. **극한 — 단일 frame.** 항목 3에 더해 waist 잠금, $W_L,W_n\to0$, $K_q\to0$이면 CLIK QP는 기존 `ClikReferenceGenerator`(6행, 위치∩속도 box)와 같은 문제다 — golden 회귀의 기준.
5. **정지.** $V^{ff}=0$, $e_R=e_L=0$, $q_c=q_{ref}$이면 $v^\ast=0$, 명령 불변. MPC는 $\hat v_b=0$ · $\hat p_b=p_{C_R}(x_0)$ · $q_L=q_L^{rest}$이면 $u=0$이 최적이고 $\dot k_G=\dot A_G\dot q=0$.
6. **결합 부호.** 왼손 과제를 world에 두면 ${}^WJ_{C_L}=[ J_{L,w}\quad J_{L,L}\quad0 ]$로 waist 열이 생겨, waist yaw $\omega_w$가 왼손에 $\omega_w\times r_L$의 속도 오차를 만들고 $W_L$이 그것을 지우려 waist를 끌어당긴다. 몸통 frame 선택이 이 결합을 구조적으로 0으로 만든다.
7. **각운동량 상쇄 확인.** 왼팔 고정($W_L^{rest}\to\infty$)에 $W_{\dot k}\to\infty$를 두면 오른팔 속도가 0 쪽으로 눌려 포구가 실패해야 한다. 왼팔을 풀면 counter-swing이 나타나고 $\max_k\Vert\dot k_G\Vert$가 줄면서 포구 항은 거의 그대로여야 한다. 그렇지 않으면 선형화나 $A_G$ frame이 틀린 것이다.
8. **일치.** 제약이 비활성이고 $q_c=q_{ref}$ 이면 CLIK 의 해는 $v^\ast=\dot q_{ref}$ 이고 두 손 과제와 자세 과제의 잔차가 모두 0 이어야 한다 (§1.5). 0 이 아니면 RT 의 FK frame 이나 몸통 기준 변환이 틀린 것이다.
9. **환원.** §1.6 의 환원형에서 $w_\perp=0$, $w_\Delta=0$, 토크 행과 한계 비활성, $x_0$ 의 가속 0 이면 해는 관절별로 독립인 최소 jerk 정지 궤적이다. 관절 하나의 닫힌식 해와 대조한다.
10. **충돌 회귀.** 왼팔이 counter-swing할 때 팔–팔 거리와 공–왼팔 거리 제약이 활성화되는 노드가 진단에 찍혀야 한다. 활성 0이면 왼팔이 실제로 움직이지 않은 것이다.

---

## 5. 확인 필요 항목 정리

| 항목 | 내용 | 영향 |
|---|---|---|
| 기구 | **확인됨** (G1 URDF) — 두 팔 뿌리가 모두 `torso_link` 에 있다. 충돌 단순화는 $\mathcal P_A$ 에만 성립한다 | §0.3 |
| CLIK 다중 frame | **확인됨** — 기존 `ClikReferenceGenerator` 는 frame 과제 1개다. 일반화가 필요하다 (계획 E2-F04) | §2.2 |
| CLIK 자세 과제 | $\dot q_n$ 또는 영공간 기준 입력 | §2.2, 여유 자유도 일치 — v0.2에서 필수 |
| `RtModelHandle` | 두 함수는 설치된 Pinocchio 에 있고 고정 베이스 모델에서도 정의된다. `RtModelHandle` 노출과 할당 0 은 미확인 | §1.4 |
| 속도 미분 | `getFrameVelocityDerivatives` 의 결과가 $H_v$ 와 같은지 (유한 차분 대조) | §1.2 |
| RT 의 FK | tick 마다 $q_{ref}$ 에서 하는 FK 의 비용과 할당 0 | §1.5 |
| 재계획 주기 | 예측 메시지 주기의 실측값과 흔들림 (v1 실측 30 Hz, 드롭 시 15 Hz) | §1.2 |
| 환원형 | $N_s$ · $\Delta_s$ 의 값, 포구 전 $x_0$ 예측 | §1.6 |
| sweep 조건 | 여덟 조건의 profile 이 load 되는지, 점 수가 늘 때 발행 주기가 유지되는지 | §1.7 |
| 후보 수 | 사전 거르기 뒤 한 주기에 푸는 후보 수 $K_{\max}$ 와 예산 | §1.3 바깥 루프 |
| 토크 미분 | `computeRNEADerivatives` 의 할당 0 과 계산 시간 | §1.2 |
| 축소 모델 | 손을 잠근 모델의 관성이 sim 의 손 자세 범위에서 얼마나 벗어나는가 | §0.1 |
| armature | 값의 출처 — G1 URDF 에는 없고 MJCF 에만 있다 | §1.3 토크 행 |
| DECEL 중 RT 추종 | MPC 정지 구간을 실제로 따르게 할지 (L7 전이, E-8) | §1.4 |
| `PlannerRtState` | 직전 채택 plan_id·현재 모드·전체 $q$ | §1.3 $x_0$ 계산 |
| solve time | move blocking 후 (선형화 + condensing + dense QP) × $t_c$ 후보의 p99. 반복은 주기당 1회 | `budget_s` 20 ms 게이트 |
| PlanSnapshot 크기 | 노드 payload ≈ 8.6 KB의 SeqLock 복사 비용 | tick 예산 |
| waist backend | device 종류·servo 지연 ($T_{arm}$ 관절군별) | lead 보상 |

---

## 6. 문헌 · 공개 코드 대조 (2026-09-29)

이 문서의 v0.2 를 발표된 논문과 공개 구현에 대조한 결과다. §6.2 는 v0.3 에, §6.3 과 §6.4 는 v0.4 에 반영했다. 표의 "위치" 는 지적 당시인 v0.2 기준이고, 마지막 열이 지금 문서에서의 반영 위치다. §6.5 는 계획 문서에 대한 것이라 이 문서에는 반영할 것이 없다.

근거 수준을 항목마다 적는다.

| 표기 | 뜻 |
|---|---|
| 본문 | 논문 본문 또는 소스 코드를 읽고 확인했다 |
| 초록 | 초록까지만 확인했다 |
| 서지 | 논문의 존재와 서지만 확인했다. 내용은 통설에 기댄 것이다 |
| 자체 | 문헌이 아니라 이 문서를 검토하며 한 계산 · 추론이다. 검증되지 않았다 |
| 로컬 | 이 workspace 의 파일로 확인했다 |

### 6.1 문헌이 뒷받침하는 선택

| 선택 | 위치 | 근거 | 수준 |
|---|---|---|---|
| jerk 입력 triple integrator | §1.1 | [Heins2023] 이 같은 상태 · 입력을 가속 연속을 이유로 쓴다. 노드 20 개, 계산 시간 최대 20 ms 로 이 문서의 규모 · 예산과 같다 | 본문 |
| Gauss-Newton 형태의 SQP | §1.2 | 최소제곱 비용에 출력을 선형화하는 구성은 RTI 의 통상 선택이다 [Gros2020] | 본문 |
| 선형화한 거리 제약 | §1.2 · §1.3 | velocity damper [Faverjon1987], 순차 볼록화 [Schulman2014] | 초록 · 본문 |
| 상대속도는 게이트가 아니라 해의 결과 | §1.3 | [Salehian2016] 은 한계 안에서 softness 를 최대화한다. 실측 softness 는 0.50–0.67 | 본문 |
| 종단 정지 | §1.3 | 포구 궤적 전체를 정지까지 한 문제로 푼 사례 [Lampariello2011], 종단 속도 0 [Dong2020] | 본문 |
| 가중 최소제곱 QP | §2.2 | pink · mink · Tasks · TSID 가 모두 가중 형태다 (§8) | 본문 |
| 회전 오차 정의 | §2.1 | TSID 의 world 기준 모드와 같다. 부호 · frame 오류는 찾지 못했다 | 본문 |
| 몸통 frame 과제의 waist 열 0 | §0.3 | G1 모델에서 waist 세 관절이 모두 `torso_link` 상류이고 양 어깨가 `torso_link` 의 자식이다. 상대 Jacobian [Lewis1990] [Jamisola2015] | 로컬 · 본문 |
| 중심 운동량 행렬의 계산 | §1.4 | 설치된 Pinocchio 에 `computeCentroidalMap` · `computeCentroidalMapTimeVariation` 이 있다. 고정 베이스 모델에서도 정의된다 | 로컬 |
| 단일 팔에서 성공률 상승을 기대하지 않음 | 계획 §3.2 | [Dong2020] 은 QP 계획 75 % 와 사다리꼴 계획 72.5 % (각 40 회) 로 유의한 차이가 없다고 보고한다. [Bauml2010] 은 실패의 주원인을 예측 오차로 든다 | 본문 |

### 6.2 고칠 것 — v0.3 에 반영

| ID | 위치 (v0.2) | 문제 | 근거 | 수준 | v0.3 반영 |
|---|---|---|---|---|---|
| R-1 | §1.2 | $J\dot q$ 의 선형화에서 버린 항을 "2차 항" 이라 했다. 이 항은 $\delta q$ 의 1차 항이다. 버리면 Jacobian 이 부정확한 SQP 가 되고, 각운동량의 $\dot A_G\bar{\dot q}$ 고정도 같다. 부정확한 Jacobian 을 쓰는 SQP 는 보정 없이는 원 문제의 정류점으로 수렴하지 않는다 | [Diehl2010] | 서지 · 자체 | §1.2 — $H_v\delta q$ 를 선형화와 상대속도 제약에 넣었다. 각운동량 · 토크 식은 "계수를 고정한 근사" 로 표기 |
| R-2 | §1.2 | RTI 는 주기당 Newton step 1 회다. "1–2회" 는 변형이며 2 회째는 선형화와 condensing 을 다시 해야 한다. 직전 해의 시간 shift 절차가 없다 — shift 없는 RTI 는 폐루프 성능이 나빠진다 | [Gros2020] | 본문 | §1.2 — 주기당 1회, shift 절차 추가 |
| R-3 | §0.2 · §1.2 | 노드 간격은 0.05 s 인데 재계획을 30 Hz 라 썼다. vision 간격이 0.05 s 이므로 재계획 주기를 다시 적어야 한다. 주기와 노드 간격이 다르면 직전 해가 새 격자 위에 놓이지 않는다 | L3 §6 | 로컬 · 자체 | **v0.4 에서 정정.** 이 지적은 절반이 틀렸다. 0.05 s 는 예측점 간격이고 재계획 주기는 예측 메시지의 주기 (sim 실측 30 Hz) 다 — v0.2 의 "30 Hz" 가 맞았고 v0.3 의 수정이 틀렸다. 남는 문제 (주기와 노드 간격이 달라 직전 해가 새 격자 위에 없다) 는 §1.2 에서 직전 해를 닫힌식으로 다시 평가해 푼다 |
| R-4 | §1.3 | 종단 정지 제약을 제동 가능성의 보장으로 썼다. 종단 등식이 재귀적 실행 가능성을 주는 것은 직전 해를 shift 한 것이 다음 문제의 해일 때다. 고정 move blocking, 격자 어긋남, 재선형화된 hard 제약이 이 전제를 깬다 | [Mayne2000] [Cagienard2007] | 서지 · 초록 | §1.1 · §1.3 — 보장을 주장하지 않는다. 안전망 표 추가 |
| R-5 | §1.3 | "MPC 실행 가능 집합 ⊂ CLIK 실행 가능 집합" 은 $\eta\lt1$ 만으로 나오지 않는다. 토크는 상태에 의존하고 CLIK 은 feedforward 위에 오차 되먹임을 더한다. 추종 오차와 이득의 곱이 여유 안에 들어야 성립한다. 제약 축소를 외란 한계에서 유도하는 방법은 tube MPC 다 | [Mayne2005] | 서지 · 자체 | §1.3 · §2.3 · §3 — 여유로 표기, 성립 조건 명시 |
| R-6 | §0.3 · §1.2 | 자기충돌 거리가 waist 에 무관한 것은 `torso_link` 에 고정된 물체끼리만이다. 팔 · 손과 `pelvis` · `waist_yaw_link` · `waist_roll_link` · 고정된 다리 사이의 거리는 waist 에 의존한다. 충돌 쌍을 두 부류로 나눠야 한다 | G1 URDF | 로컬 | §0.3 · §1.2 · §1.3 · §5 — $\mathcal P_A$ · $\mathcal P_B$ 로 분리 |
| R-7 | §1.5 | 회전벡터의 시간 미분은 각속도가 아니다. 구간 끝에서는 SO(3) Jacobian 으로 경계값을 바꿔야 하고, RT 도 보간한 미분을 각속도로 되돌려야 한다. 게시 twist 는 world 축, 보간 chart 는 body 축이라 변환도 필요하다. 빠뜨리면 노드마다 각속도가 끊긴다 | [Sola2018] [Zefran1998] | 초록 · 자체 | §1.5 · §3 — 보간기 삭제, RT 가 관절 기준에서 FK |
| R-8 | §2.3 | 오차 동역학이 연속 시간 식이다. 이산 tick 에서는 이득과 tick 의 곱에 대한 조건이 필요하다. 자세 오차는 목표가 정지해 있으면 근사가 아니라 정확히 성립하고, 목표가 움직일 때의 잔차는 feedforward 의 frame 차이에서 나온다 | [Falco2011] | 서지 · 자체 | §2.3 — 이산 점화식과 이득 조건, 자세 오차 문장 정정 |
| R-9 | §1.3 | 가속 box 와 waist 만의 토크 행이 남아 있다. 계획 MD-7 (토크 기반, 활성 관절 전체) 과 어긋난다. $n=16$ 가정도 G1 의 $n=17$ 과 다르다 | 계획 §4 · §5 | 로컬 | §0.1 · §1.1 · §1.3 · §3 · §4 — 가속 box 삭제, 토크 행 전체, $n=17$ |

R-7 은 보간기를 고치는 대신 없애는 쪽을 택했다. R-1 과 R-8 의 식 일부는 이 문서의 유도이며 구현 때 수치로 대조한다 (§5).

### 6.3 빠진 것 — v0.4 에 반영

| ID | 위치 (v0.2) | 빠진 것 | 근거 | 수준 | v0.4 반영 |
|---|---|---|---|---|---|
| M-1 | §1.3 | 포구 시각이 0.05 s 격자에 묶인다. 발표된 시스템이 드는 시각 정밀도는 수 ms 다. 격자 아래의 보정과, COMMITTED 뒤 $t_c$ 가 격자에서 벗어나는 경우의 처리가 없다. 수렴 전의 비용을 후보끼리 비교하는 점도 다루지 않았다 | [Bauml2010] [Dong2020] | 본문 | §1.3 바깥 루프 — 후보는 vision 예측 격자 그대로 두고 MPC 격자와 분리. 후보 초기화 · 교체 자격 추가. 간격의 영향은 §1.7 sweep 으로 잰다 |
| M-2 | §1.3 | 상대속도를 포구 노드 한 점에서만 맞춘다. 문헌은 손가락이 닫히는 동안 손이 공과 함께 움직이는 구간을 둔다 | [Salehian2016] [Lampariello2011] | 본문 | §1.3 — 상대속도와 경로 이탈 비용을 포구 구간 $\mathcal K_c$ 에 건다 |
| M-3 | §1.3 | 상대속도 가중이 등방이다. 공 진행 방향 성분과 수직 성분을 나눠 다루지 않는다 | [Abeyruwan2023] | 본문 | §1.3 — $W_{v,k}$ 를 진행 방향과 수직 방향으로 분리 |
| M-4 | §1.3 | slack 이 상대속도에만 있다. 토크 · 충돌 · trust region 행이 hard 라 재선형화 뒤 QP 가 실행 불가능해질 수 있고, 그때 계획기가 무엇을 게시하는지 정하지 않았다. [Heins2023] 은 동역학을 뺀 모든 제약을 slack 으로 완화한다 | [Heins2023] [Schulman2014] | 본문 | §1.3 "slack" — 토크 · 자기충돌 행에 slack, 나머지는 hard |
| M-5 | §1.3 | 정지 자세를 유지할 토크가 한계 안인지 보는 종단 행이 없다 | — | 자체 | §1.2 — 종단 노드의 토크 행이 정적 토크 행이다 |
| M-6 | §1.1 · §5 | 비용을 변수 수로만 따졌다. condensing 뒤에는 상태 제약이 모두 dense 행이 되고 move blocking 은 행 수를 줄이지 않는다. condensed QP 를 만드는 비용은 horizon 의 제곱 규모이고 반복마다 든다 | [Verschueren2022] [Frison2016] | 본문 | §1.1 "문제 크기" |
| M-7 | §1.1 | 정지 구간을 2–3 노드로 묶으면 종단 등식을 맞출 jerk 자유도가 남는지 확인하지 않았다 | — | 자체 | §1.1 "정지 구간의 블록 수" — 관절마다 3개 이상 |
| M-8 | §2.2 | 위치 box 와 가속 · 토크 한계가 한 tick 에서 양립하지 않을 수 있다. 문헌의 해법은 제동 거리로 속도를 묶는 viability 한계다. 기존 `ClikReferenceGenerator` 는 1-step 위치 box 를 쓰고 충돌 시 `bound_conflict` 와 실패 (직전 값 유지) 로 처리한다. `rtc_tsid` 에는 가속 수준의 viability 제약 `JointLimitConstraint` 가 따로 있다 | [DelPrete2018] [Faroni2018] [Flacco2015] | 초록 · 로컬 | §2.2 "제동 거리 한계" — opt-in, 기본 꺼짐 |
| M-9 | §2.2 | RT CLIK 에 충돌 제약이 없다. 오차 되먹임이 팔을 계획 경로 밖으로 밀어도 막을 것이 없다. 문헌과 공개 구현은 tick 주기 QP 에 velocity damper 를 둔다 | [Stasse2008] | 초록 | §2.2 "충돌 damper" — 선택 항, 기본 꺼짐 |
| M-10 | §1.3 | 노드 사이 여유의 $c_j$ 가 정의돼 있지 않다. 이 여유는 속도에 비례해 커져 팔–팔 제약을 실행 불가능하게 만들 수 있다. [Schulman2014] 는 연속한 두 노드의 swept volume 에 제약을 건다 | [Schulman2014] | 본문 | §1.3 "노드 사이 여유" — $c_j$ 정의, 계수 1/2 |
| M-11 | §1.3 · §2.2 | "armature 포함" 의 값 출처가 없다. G1 URDF 에는 armature 가 없고 MJCF 에만 있다 (0.05). 모델에 직접 넣지 않으면 관성 행렬이 sim 과 다르다 | G1 URDF · MJCF | 로컬 | §1.3 토크 행 — 로봇 config 에서 읽고 sim 은 MJCF 값 |
| M-12 | §1.3 · §2.2 | 오른손이 폐쇄 체인인데 토크 행의 $M,h$ 를 어떤 모델로 계산하는지 없다. 폐쇄 체인 동역학은 [Carpentier2021] | [Carpentier2021] | 본문 | §0.1 — 손을 잠근 축소 모델 |
| M-13 | §1.3 | 직전 계획과의 일관성 가중 $w_\Delta$ 가 상수다. 예측이 가장 정확해지는 포구 직전의 보정을 막는다 | [Bauml2010] | 본문 · 자체 | §1.3 — $w_\Delta$ 를 예측 공분산에 비례 |
| M-14 | §1.3 | 공분산 역수 가중을 후보 사이의 비교에 그대로 쓰면, 불확실성이 큰 후보일수록 같은 오차의 비용이 작아진다 | — | 자체 | §1.3 바깥 루프 "순위" — 공통 고정 가중 $W^{rank} _ p$ |
| M-15 | §1.2 · §2.1 | 자세 오차가 $\pi$ 근처일 때의 처리와 되먹임 항의 크기 제한이 없다 | — | 자체 | §2.1 — 되먹임 상한과 $\pi$ 근처의 처리 |

### 6.4 근거가 약한 것 — v0.4 에 반영

| ID | 위치 (v0.2) | 내용 | 근거 | 수준 | v0.4 반영 |
|---|---|---|---|---|---|
| W-1 | §1.4 | 고정 베이스 강체 모델에서 중심 각운동량 변화율은 베이스가 받으며 손의 정확도에 영향이 없다. 이 양은 waist 나 베이스의 모멘트와도 같지 않다 — 선운동량 변화율의 모멘트와 중력 모멘트가 빠진다. waist 토크 행이 이미 제약에 있다. 문헌에서 이 항의 동기는 균형 [Wensing2016] [Kajita2003], 유연한 베이스 [Wimbock2009] 다. [Bauml2011] 은 몸통 반력의 영향 (진동, 카메라 흔들림) 을 보고하지만 대응은 동역학 feedforward 였다 | [Orin2013] [Bauml2011] | 초록 · 본문 · 자체 | §1.4 — **항을 유지한다** (사용자 결정). 목적은 왼팔 운동 생성과 floating base 확장. 고정 베이스에서의 한계를 명시하고, 효과를 성공률로 재지 않는다 |
| W-2 | §1.3 | 공분산 역수를 종단 위치 가중으로 쓰는 포구 논문을 찾지 못했다. 읽은 시스템은 모두 예측 평균 위에서 계획하고 빠른 재계획에 기댄다. 이 가중은 이 문서의 선택이다 | [Bauml2010] [Dong2020] [Abeyruwan2023] | 본문 | §1.3 포구 위치 행 — 이 문서의 선택으로 표기 |
| W-3 | §1.3 | 토크 행은 $\ddot q$ 에만 정확하고 $q$ · $\dot q$ 에 대해서는 0차다. 적분기 모델 MPC 안에서 이 근사를 검증한 문헌을 찾지 못했다. 해석적 미분은 계산이 싸다 [Carpentier2018]. 동역학 전체를 쓰는 대안은 [Kleff2021] | [Carpentier2018] | 초록 | §1.2 — 토크를 1차까지 선형화 |
| W-4 | 구현 | ProxQP 논문의 benchmark 는 warm start 를 끈 임의 QP 다. 이 문제 크기의 warm start 된 MPC QP 를 잰 공개 benchmark 를 찾지 못했다. MPC 에서는 구조를 쓰는 solver 가 빠르다는 보고가 있다 | [Bambade2022] [FrisonDiehl2020] [Stark2025] | 본문 | 아래 |
| W-5 | §2 | waist 를 움직이면서 두 번째 손을 몸통 기준으로 두는 G1 용 속도 수준 QP IK 의 공개 구현을 찾지 못했다. Unitree 의 원격 조작 코드는 waist 를 잠그고 위치 수준 NLP 를 푼다 (§8) | — | 본문 | §2.2 — 선례 없음을 명시, pink 를 golden 회귀의 수치 기준으로 |

W-4 는 식이 아니라 구현의 선택이다. 기본 solver 는 ProxQP dense 로 둔다 (계획 MD-1). 계획기 한 주기의 계산 시간 p99 를 실측하고, 덤프한 QP 로 구조를 쓰는 solver 와 오프라인 비교하는 것을 선택 항으로 둔다 (§8 의 qpbenchmark, hpipm).

W-1 의 지적은 고정 베이스에 한정된 것이다. 이 작업은 floating base 로 확장할 예정이므로 항을 유지한다.

### 6.5 계획 문서의 게이트 G-1

| 항목 | 내용 | 근거 | 수준 |
|---|---|---|---|
| 검정 | paired 이진 결과의 단측 비열등 검정을 쓴다. McNemar 검정은 "차이 없음" 을 기각하지 못했다는 것만 말하므로 비열등의 근거가 아니다 | [Tango1998] [Liu2002] | 본문 · 서지 |
| 시행 수 | 필요한 N 은 비열등 한계와 불일치율로 정해진다. 불일치율은 같은 투척을 v1 끼리 비교해 먼저 잰다 | [Tango1998] | 본문 · 자체 |
| 성공 정의 | DECEL 은 포구 뒤에만 돌므로, 성공 정의에 "DECEL 이 끝날 때까지 공을 쥐고 있음" 이 들어가야 차이가 보인다 | — | 자체 |
| 한계의 단위 | 비열등 한계가 절대값인지 상대값인지 적는다. baseline 이 낮은 로봇에서 둘의 차이가 크다 | — | 자체 |

---

## 7. 참고 문헌

서지 (저자 · 제목 · 게재지 · 연도 · DOI) 는 모두 2026-09-29 에 Crossref 또는 arXiv 등록 정보로 확인했다. 내용을 어디까지 읽었는지는 §6 의 수준 열에 있다. 본문에서 인용하지 않은 항목은 같은 주제의 배경 문헌이다.

### 7.1 MPC 수치

- **[Wieber2006]** P.-B. Wieber. Trajectory Free Linear Model Predictive Control for Stable Walking in the Presence of Strong Perturbations. IEEE-RAS Humanoids, 137–142, 2006. [doi:10.1109/ICHR.2006.321375](https://doi.org/10.1109/ICHR.2006.321375)
- **[Berscheid2021]** L. Berscheid, T. Kröger. Jerk-limited Real-time Trajectory Generation with Arbitrary Target States. Robotics: Science and Systems, 2021. [doi:10.15607/RSS.2021.XVII.015](https://doi.org/10.15607/RSS.2021.XVII.015)
- **[Heins2023]** A. Heins, A. P. Schoellig. Keep It Upright: Model Predictive Control for Nonprehensile Object Transportation With Obstacle Avoidance on a Mobile Manipulator. IEEE RA-L 8(12), 7986–7993, 2023. [doi:10.1109/LRA.2023.3324520](https://doi.org/10.1109/LRA.2023.3324520) · [arXiv:2305.17484](https://arxiv.org/abs/2305.17484)
- **[BockPlitt1984]** H. G. Bock, K. J. Plitt. A Multiple Shooting Algorithm for Direct Solution of Optimal Control Problems. IFAC Proceedings Volumes 17(2), 1603–1608, 1984. [doi:10.1016/S1474-6670(17)61205-9](https://doi.org/10.1016/S1474-6670%2817%2961205-9)
- **[Frison2016]** G. Frison, D. Kouzoupis, J. B. Jørgensen, M. Diehl. An efficient implementation of partial condensing for Nonlinear Model Predictive Control. IEEE CDC, 4457–4462, 2016. [doi:10.1109/CDC.2016.7798946](https://doi.org/10.1109/CDC.2016.7798946)
- **[FrisonDiehl2020]** G. Frison, M. Diehl. HPIPM: a high-performance quadratic programming framework for model predictive control. IFAC-PapersOnLine 53(2), 6563–6569, 2020. [doi:10.1016/j.ifacol.2020.12.073](https://doi.org/10.1016/j.ifacol.2020.12.073)
- **[Cagienard2007]** R. Cagienard, P. Grieder, E. C. Kerrigan, M. Morari. Move blocking strategies in receding horizon control. Journal of Process Control 17(6), 563–570, 2007. [doi:10.1016/j.jprocont.2007.01.001](https://doi.org/10.1016/j.jprocont.2007.01.001)
- **[Gondhalekar2010]** R. Gondhalekar, J. Imura. Least-restrictive move-blocking model predictive control. Automatica 46(7), 1234–1240, 2010. [doi:10.1016/j.automatica.2010.04.010](https://doi.org/10.1016/j.automatica.2010.04.010)
- **[Shekhar2015]** R. C. Shekhar, C. Manzie. Optimal move blocking strategies for model predictive control. Automatica 61, 27–34, 2015. [doi:10.1016/j.automatica.2015.07.030](https://doi.org/10.1016/j.automatica.2015.07.030)
- **[Diehl2005]** M. Diehl, H. G. Bock, J. P. Schlöder. A Real-Time Iteration Scheme for Nonlinear Optimization in Optimal Feedback Control. SIAM Journal on Control and Optimization 43(5), 1714–1736, 2005. [doi:10.1137/S0363012902400713](https://doi.org/10.1137/S0363012902400713)
- **[Gros2020]** S. Gros, M. Zanon, R. Quirynen, A. Bemporad, M. Diehl. From linear to nonlinear MPC: bridging the gap via the real-time iteration. International Journal of Control 93(1), 62–80, 2020. [doi:10.1080/00207179.2016.1222553](https://doi.org/10.1080/00207179.2016.1222553)
- **[Diehl2010]** M. Diehl, A. Walther, H. G. Bock, E. Kostina. An adjoint-based SQP algorithm with quasi-Newton Jacobian updates for inequality constrained optimization. Optimization Methods and Software 25(4), 531–552, 2010. [doi:10.1080/10556780903027500](https://doi.org/10.1080/10556780903027500)
- **[Verschueren2022]** R. Verschueren, G. Frison, D. Kouzoupis, J. Frey, N. van Duijkeren, A. Zanelli, B. Novoselnik, T. Albin, R. Quirynen, M. Diehl. acados — a modular open-source framework for fast embedded optimal control. Mathematical Programming Computation 14(1), 147–183, 2022. [doi:10.1007/s12532-021-00208-8](https://doi.org/10.1007/s12532-021-00208-8)
- **[Mayne2000]** D. Q. Mayne, J. B. Rawlings, C. V. Rao, P. O. M. Scokaert. Constrained model predictive control: Stability and optimality. Automatica 36(6), 789–814, 2000. [doi:10.1016/S0005-1098(99)00214-9](https://doi.org/10.1016/S0005-1098%2899%2900214-9)
- **[Mayne2005]** D. Q. Mayne, M. M. Seron, S. V. Raković. Robust model predictive control of constrained linear systems with bounded disturbances. Automatica 41(2), 219–224, 2005. [doi:10.1016/j.automatica.2004.08.019](https://doi.org/10.1016/j.automatica.2004.08.019)
- **[Kleff2021]** S. Kleff, A. Meduri, R. Budhiraja, N. Mansard, L. Righetti. High-Frequency Nonlinear Model Predictive Control of a Manipulator. IEEE ICRA, 7330–7336, 2021. [doi:10.1109/ICRA48506.2021.9560990](https://doi.org/10.1109/ICRA48506.2021.9560990)

### 7.2 QP solver

- **[Bambade2022]** A. Bambade, S. El-Kazdadi, A. Taylor, J. Carpentier. PROX-QP: Yet another Quadratic Programming Solver for Robotics and beyond. Robotics: Science and Systems, 2022. [doi:10.15607/RSS.2022.XVIII.040](https://doi.org/10.15607/RSS.2022.XVIII.040)
- **[Bambade2025]** A. Bambade, F. Schramm, S. El-Kazdadi, S. Caron, A. Taylor, J. Carpentier. ProxQP: an Efficient and Versatile Quadratic Programming Solver for Real-Time Robotics Applications and Beyond. IEEE Transactions on Robotics, 2025. [doi:10.1109/TRO.2025.3577107](https://doi.org/10.1109/TRO.2025.3577107)
- **[Stellato2020]** B. Stellato, G. Banjac, P. Goulart, A. Bemporad, S. Boyd. OSQP: an operator splitting solver for quadratic programs. Mathematical Programming Computation 12(4), 637–672, 2020. [doi:10.1007/s12532-020-00179-2](https://doi.org/10.1007/s12532-020-00179-2)
- **[Ferreau2014]** H. J. Ferreau, C. Kirches, A. Potschka, H. G. Bock, M. Diehl. qpOASES: a parametric active-set algorithm for quadratic programming. Mathematical Programming Computation 6(4), 327–363, 2014. [doi:10.1007/s12532-014-0071-1](https://doi.org/10.1007/s12532-014-0071-1)
- **[Stark2025]** F. Stark, J. Middelberg, D. Mronga, S. Vyas, F. Kirchner. Benchmarking Different QP Formulations and Solvers for Dynamic Quadrupedal Walking. IEEE ICRA, 14412–14418, 2025. [doi:10.1109/ICRA55743.2025.11128397](https://doi.org/10.1109/ICRA55743.2025.11128397) · [arXiv:2502.01329](https://arxiv.org/abs/2502.01329)

### 7.3 포구

- **[Bauml2010]** B. Bäuml, T. Wimböck, G. Hirzinger. Kinematically optimal catching a flying ball with a hand-arm-system. IEEE/RSJ IROS, 2592–2599, 2010. [doi:10.1109/IROS.2010.5651175](https://doi.org/10.1109/IROS.2010.5651175)
- **[Bauml2011]** B. Bäuml, O. Birbach, T. Wimböck, U. Frese, A. Dietrich, G. Hirzinger. Catching flying balls with a mobile humanoid: System overview and design considerations. IEEE-RAS Humanoids, 513–520, 2011. [doi:10.1109/Humanoids.2011.6100837](https://doi.org/10.1109/Humanoids.2011.6100837)
- **[Lampariello2011]** R. Lampariello, D. Nguyen-Tuong, C. Castellini, G. Hirzinger, J. Peters. Trajectory planning for optimal robot catching in real-time. IEEE ICRA, 3719–3726, 2011. [doi:10.1109/ICRA.2011.5980114](https://doi.org/10.1109/ICRA.2011.5980114)
- **[Kim2014]** S. Kim, A. Shukla, A. Billard. Catching Objects in Flight. IEEE Transactions on Robotics 30(5), 1049–1065, 2014. [doi:10.1109/TRO.2014.2316022](https://doi.org/10.1109/TRO.2014.2316022)
- **[Salehian2016]** S. S. Mirrazavi Salehian, M. Khoramshahi, A. Billard. A Dynamical System Approach for Softly Catching a Flying Object: Theory and Experiment. IEEE Transactions on Robotics 32(2), 462–471, 2016. [doi:10.1109/TRO.2016.2536749](https://doi.org/10.1109/TRO.2016.2536749)
- **[Dong2020]** K. Dong, K. Pereida, F. Shkurti, A. P. Schoellig. Catch the Ball: Accurate High-Speed Motions for Mobile Manipulators via Inverse Dynamics Learning. IEEE/RSJ IROS, 6718–6725, 2020. [doi:10.1109/IROS45743.2020.9341134](https://doi.org/10.1109/IROS45743.2020.9341134) · [arXiv:2003.07489](https://arxiv.org/abs/2003.07489)
- **[Abeyruwan2023]** S. Abeyruwan, A. Bewley, N. M. Boffi, K. Choromanski, D. D'Ambrosio, D. Jain, P. Sanketi, A. Shankar, V. Sindhwani, S. Singh, J.-J. Slotine, S. Tu. Agile Catching with Whole-Body MPC and Blackbox Policy Learning. L4DC, 2023. [arXiv:2306.08205](https://arxiv.org/abs/2306.08205)
- **[Yan2024]** L. Yan, T. Stouraitis, J. Moura, W. Xu, M. Gienger, S. Vijayakumar. Impact-Aware Bimanual Catching of Large-Momentum Objects. IEEE Transactions on Robotics 40, 2543–2563, 2024. [doi:10.1109/TRO.2024.3381551](https://doi.org/10.1109/TRO.2024.3381551)
- **[Wimbock2009]** T. Wimböck, D. Nenchev, A. Albu-Schäffer, G. Hirzinger. Experimental study on dynamic reactionless motions with DLR's humanoid robot Justin. IEEE/RSJ IROS, 5481–5486, 2009. [doi:10.1109/IROS.2009.5354528](https://doi.org/10.1109/IROS.2009.5354528)

### 7.4 CLIK · 다중 과제 QP · 관절 한계

- **[Sciavicco1988]** L. Sciavicco, B. Siciliano. A solution algorithm to the inverse kinematic problem for redundant manipulators. IEEE Journal on Robotics and Automation 4(4), 403–410, 1988. [doi:10.1109/56.804](https://doi.org/10.1109/56.804)
- **[Siciliano1990]** B. Siciliano. A closed-loop inverse kinematic scheme for on-line joint-based robot control. Robotica 8(3), 231–243, 1990. [doi:10.1017/S0263574700000096](https://doi.org/10.1017/S0263574700000096)
- **[Nakamura1986]** Y. Nakamura, H. Hanafusa. Inverse Kinematic Solutions With Singularity Robustness for Robot Manipulator Control. Journal of Dynamic Systems, Measurement, and Control 108(3), 163–171, 1986. [doi:10.1115/1.3143764](https://doi.org/10.1115/1.3143764)
- **[Falco2011]** P. Falco, C. Natale. On the Stability of Closed-Loop Inverse Kinematics Algorithms for Redundant Robots. IEEE Transactions on Robotics 27(4), 780–784, 2011. [doi:10.1109/TRO.2011.2135210](https://doi.org/10.1109/TRO.2011.2135210)
- **[Kanoun2011]** O. Kanoun, F. Lamiraux, P.-B. Wieber. Kinematic Control of Redundant Manipulators: Generalizing the Task-Priority Framework to Inequality Task. IEEE Transactions on Robotics 27(4), 785–792, 2011. [doi:10.1109/TRO.2011.2142450](https://doi.org/10.1109/TRO.2011.2142450)
- **[Escande2014]** A. Escande, N. Mansard, P.-B. Wieber. Hierarchical quadratic programming: Fast online humanoid-robot motion generation. International Journal of Robotics Research 33(7), 1006–1028, 2014. [doi:10.1177/0278364914521306](https://doi.org/10.1177/0278364914521306)
- **[DelPrete2015]** A. Del Prete, F. Nori, G. Metta, L. Natale. Prioritized motion–force control of constrained fully-actuated robots: "Task Space Inverse Dynamics". Robotics and Autonomous Systems 63, 150–157, 2015. [doi:10.1016/j.robot.2014.08.016](https://doi.org/10.1016/j.robot.2014.08.016)
- **[Flacco2015]** F. Flacco, A. De Luca, O. Khatib. Control of Redundant Robots Under Hard Joint Constraints: Saturation in the Null Space. IEEE Transactions on Robotics 31(3), 637–654, 2015. [doi:10.1109/TRO.2015.2418582](https://doi.org/10.1109/TRO.2015.2418582)
- **[DelPrete2018]** A. Del Prete. Joint Position and Velocity Bounds in Discrete-Time Acceleration/Torque Control of Robot Manipulators. IEEE RA-L 3(1), 281–288, 2018. [doi:10.1109/LRA.2017.2738321](https://doi.org/10.1109/LRA.2017.2738321)
- **[Faroni2018]** M. Faroni, M. Beschi, N. Pedrocchi, A. Visioli. Viability and Feasibility of Constrained Kinematic Control of Manipulators. Robotics 7(3), 41, 2018. [doi:10.3390/robotics7030041](https://doi.org/10.3390/robotics7030041)
- **[Djeha2023]** M. Djeha, P. Gergondet, A. Kheddar. Robust Task-Space Quadratic Programming for Kinematic-Controlled Robots. IEEE Transactions on Robotics 39(5), 3857–3874, 2023. [doi:10.1109/TRO.2023.3286069](https://doi.org/10.1109/TRO.2023.3286069)
- **[Lewis1990]** C. L. Lewis, A. A. Maciejewski. Trajectory generation for cooperating robots. IEEE International Conference on Systems Engineering, 300–303, 1990. [doi:10.1109/ICSYSE.1990.203156](https://doi.org/10.1109/ICSYSE.1990.203156)
- **[Jamisola2015]** R. S. Jamisola, R. G. Roberts. A more compact expression of relative Jacobian based on individual manipulator Jacobians. Robotics and Autonomous Systems 63, 158–164, 2015. [doi:10.1016/j.robot.2014.08.011](https://doi.org/10.1016/j.robot.2014.08.011)

### 7.5 동역학 · 중심 운동량

- **[Carpentier2019]** J. Carpentier, G. Saurel, G. Buondonno, J. Mirabel, F. Lamiraux, O. Stasse, N. Mansard. The Pinocchio C++ library: A fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives. IEEE/SICE SII, 614–619, 2019. [doi:10.1109/SII.2019.8700380](https://doi.org/10.1109/SII.2019.8700380)
- **[Carpentier2018]** J. Carpentier, N. Mansard. Analytical Derivatives of Rigid Body Dynamics Algorithms. Robotics: Science and Systems, 2018. [doi:10.15607/RSS.2018.XIV.038](https://doi.org/10.15607/RSS.2018.XIV.038)
- **[Carpentier2021]** J. Carpentier, R. Budhiraja, N. Mansard. Proximal and Sparse Resolution of Constrained Dynamic Equations. Robotics: Science and Systems, 2021. [doi:10.15607/RSS.2021.XVII.017](https://doi.org/10.15607/RSS.2021.XVII.017)
- **[Kajita2003]** S. Kajita, F. Kanehiro, K. Kaneko, K. Fujiwara, K. Harada, K. Yokoi, H. Hirukawa. Resolved momentum control: humanoid motion planning based on the linear and angular momentum. IEEE/RSJ IROS, vol. 2, 1644–1650, 2003. [doi:10.1109/IROS.2003.1248880](https://doi.org/10.1109/IROS.2003.1248880)
- **[Orin2008]** D. E. Orin, A. Goswami. Centroidal Momentum Matrix of a humanoid robot: Structure and properties. IEEE/RSJ IROS, 653–659, 2008. [doi:10.1109/IROS.2008.4650772](https://doi.org/10.1109/IROS.2008.4650772)
- **[Orin2013]** D. E. Orin, A. Goswami, S.-H. Lee. Centroidal dynamics of a humanoid robot. Autonomous Robots 35(2-3), 161–176, 2013. [doi:10.1007/s10514-013-9341-4](https://doi.org/10.1007/s10514-013-9341-4)
- **[Wensing2016]** P. M. Wensing, D. E. Orin. Improved Computation of the Humanoid Centroidal Dynamics and Application for Whole-Body Control. International Journal of Humanoid Robotics 13(01), 1550039, 2016. [doi:10.1142/S0219843615500395](https://doi.org/10.1142/S0219843615500395)

### 7.6 충돌 거리

- **[Faverjon1987]** B. Faverjon, P. Tournassoud. A local based approach for path planning of manipulators with a high number of degrees of freedom. IEEE ICRA, vol. 4, 1152–1159, 1987. [doi:10.1109/ROBOT.1987.1087982](https://doi.org/10.1109/ROBOT.1987.1087982)
- **[Stasse2008]** O. Stasse, A. Escande, N. Mansard, S. Miossec, P. Evrard, A. Kheddar. Real-time (self)-collision avoidance task on a HRP-2 humanoid robot. IEEE ICRA, 3200–3205, 2008. [doi:10.1109/ROBOT.2008.4543698](https://doi.org/10.1109/ROBOT.2008.4543698)
- **[Schulman2014]** J. Schulman, Y. Duan, J. Ho, A. Lee, I. Awwal, H. Bradlow, J. Pan, S. Patil, K. Goldberg, P. Abbeel. Motion planning with sequential convex optimization and convex collision checking. International Journal of Robotics Research 33(9), 1251–1270, 2014. [doi:10.1177/0278364914528132](https://doi.org/10.1177/0278364914528132)
- **[EscandeSTP2014]** A. Escande, S. Miossec, M. Benallegue, A. Kheddar. A Strictly Convex Hull for Computing Proximity Distances With Continuous Gradients. IEEE Transactions on Robotics 30(3), 666–678, 2014. [doi:10.1109/TRO.2013.2296332](https://doi.org/10.1109/TRO.2013.2296332)
- **[Pan2012]** J. Pan, S. Chitta, D. Manocha. FCL: A general purpose library for collision and proximity queries. IEEE ICRA, 3859–3866, 2012. [doi:10.1109/ICRA.2012.6225337](https://doi.org/10.1109/ICRA.2012.6225337)
- **[Montaut2022]** L. Montaut, Q. Le Lidec, V. Petrik, J. Sivic, J. Carpentier. Collision Detection Accelerated: An Optimization Perspective. Robotics: Science and Systems, 2022. [doi:10.15607/RSS.2022.XVIII.039](https://doi.org/10.15607/RSS.2022.XVIII.039)

### 7.7 Lie 군 위의 오차와 보간

- **[Sola2018]** J. Solà, J. Deray, D. Atchuthan. A micro Lie theory for state estimation in robotics. arXiv preprint, 2018. [arXiv:1812.01537](https://arxiv.org/abs/1812.01537)
- **[Zefran1998]** M. Žefran, V. Kumar. Interpolation schemes for rigid body motions. Computer-Aided Design 30(3), 179–189, 1998. [doi:10.1016/S0010-4485(97)00060-2](https://doi.org/10.1016/S0010-4485%2897%2900060-2)
- **[Sommer2020]** C. Sommer, V. Usenko, D. Schubert, N. Demmel, D. Cremers. Efficient Derivative Computation for Cumulative B-Splines on Lie Groups. IEEE/CVF CVPR, 11145–11153, 2020. [doi:10.1109/CVPR42600.2020.01116](https://doi.org/10.1109/CVPR42600.2020.01116)

### 7.8 통계 (게이트 판정)

- **[Tango1998]** T. Tango. Equivalence test and confidence interval for the difference in proportions for the paired-sample design. Statistics in Medicine 17(8), 891–908, 1998. [doi 링크](https://doi.org/10.1002/%28SICI%291097-0258%2819980430%2917:8%3C891::AID-SIM780%3E3.0.CO;2-B)
- **[Liu2002]** J.-P. Liu, H.-M. Hsueh, E. Hsieh, J. J. Chen. Tests for equivalence or non-inferiority for paired binary data. Statistics in Medicine 21(2), 231–245, 2002. [doi:10.1002/sim.1012](https://doi.org/10.1002/sim.1012)

---

## 8. 공개 코드

2026-09-29 에 GitHub 등록 정보로 저장소의 존재 · license · 마지막 push 를 확인했다. license 가 "확인 필요" 인 것은 GitHub 이 license 를 자동 식별하지 못한 저장소다 — 코드를 가져다 쓰기 전에 LICENSE 파일을 직접 읽는다.

### 8.1 MPC · solver

| 저장소 | license | 마지막 push | 구현 | 대조 대상 |
|---|---|---|---|---|
| [acados/acados](https://github.com/acados/acados) | 확인 필요 | 2026-09 | SQP · SQP-RTI, condensing · partial condensing, 여러 QP solver 연결 | §1.1–§1.3 전체. 선형화 → condensing → 풀이의 단계 분리 |
| [giaf/hpipm](https://github.com/giaf/hpipm) | 확인 필요 | 2026-09 | dense · OCP 구조 QP 의 interior point, condensing 루틴 | §1.1 condensing, dense ProxQP 의 대안 |
| [Simple-Robotics/proxsuite](https://github.com/Simple-Robotics/proxsuite) | BSD-2-Clause | 2026-09 | ProxQP dense · sparse | `QPSolverWrapper` 의 solver |
| [qpsolvers/qpbenchmark](https://github.com/qpsolvers/qpbenchmark) | Apache-2.0 | 2026-07 | QP solver benchmark 틀 | solve time 비교 |
| [leggedrobotics/ocs2](https://github.com/leggedrobotics/ocs2) | BSD-3-Clause | 2026-09 | SLQ · iLQR · multiple shooting SQP, 자기충돌 제약 | §1.2 · §1.3 의 구조 |
| [learnsyslab/upright](https://github.com/learnsyslab/upright) | MIT | 2026-08 | [Heins2023] 의 코드 — 관절 공간 jerk 입력 MPC (OCS2 위) | §1.1 에 가장 가까운 공개 구현. sparse 경로를 쓴다 |
| [pantor/ruckig](https://github.com/pantor/ruckig) | MIT | 2026-09 | [Berscheid2021] 의 코드 — jerk 제한 시간 최적 궤적 | 정지 구간, 실행 불가능 시 fallback |
| [Simple-Robotics/aligator](https://github.com/Simple-Robotics/aligator) | BSD-2-Clause | 2026-09 | 제약 있는 궤적 최적화 (ProxDDP) | 계획 MD-1 이 택하지 않은 경로 |
| [loco-3d/crocoddyl](https://github.com/loco-3d/crocoddyl) | BSD-3-Clause | 2026-09 | 동역학 전체를 쓰는 DDP 계열 | 계획 MD-7 의 대안 |
| [google-deepmind/mujoco_mpc](https://github.com/google-deepmind/mujoco_mpc) | Apache-2.0 | 2026-09 | MuJoCo 위의 iLQG · sampling MPC | sim 쪽 대조군 |

### 8.2 IK · 전신 제어 · 충돌

| 저장소 | license | 마지막 push | 구현 | 대조 대상 |
|---|---|---|---|---|
| [stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio) | BSD-2-Clause | 2026-09 | 강체 동역학, 중심 운동량, Lie 군 미분, 폐쇄 체인 | 전체 |
| [stack-of-tasks/tsid](https://github.com/stack-of-tasks/tsid) | BSD-2-Clause | 2026-09 | 가속 수준 QP. SE3 과제, 두 frame 과제, viability 관절 한계, 각운동량 과제 | `rtc_tsid`, §2.1, §6 M-8 |
| [pink-kinematics/pink](https://github.com/pink-kinematics/pink) | Apache-2.0 | 2026-09 | Pinocchio 위의 속도 수준 가중 QP IK. frame · relative frame · posture 과제, 가속 한계, 자기충돌 | §2 에 가장 가까운 공개 구현. golden 회귀의 수치 기준으로 쓸 수 있다 |
| [kevinzakka/mink](https://github.com/kevinzakka/mink) | Apache-2.0 | 2026-09 | 같은 구조를 MuJoCo 위에 구현. 충돌 회피 한계 | §2, §6 M-9 |
| [Rhoban/placo](https://github.com/Rhoban/placo) | MIT | 2026-09 | C++ QP IK. relative frame, 중심 운동량, 자기충돌을 한 QP 에 | §2, §1.4 |
| [jrl-umi3218/Tasks](https://github.com/jrl-umi3218/Tasks) | BSD-2-Clause | 2026-09 | 가중 QP 전신 제어. velocity damper 형태의 관절 한계 · 충돌 제약 | §2.2 제약 |
| [jrl-umi3218/mc_rtc](https://github.com/jrl-umi3218/mc_rtc) | BSD-2-Clause | 2026-09 | Tasks 위의 실시간 제어 framework | 컨트롤러 계층 |
| [coal-library/coal](https://github.com/coal-library/coal) | 확인 필요 | 2026-09 | 거리 계산 (GJK · EPA), capsule. 이전 이름 hpp-fcl | 충돌 코어 (계획 E3-F03) |
| [tesseract-robotics/trajopt](https://github.com/tesseract-robotics/trajopt) | 확인 필요 | 2026-09 | [Schulman2014] 계열의 유지되는 구현 | §6 M-10 |
| [NVlabs/curobo](https://github.com/NVlabs/curobo) | Apache-2.0 | 2026-09 | GPU 최소 jerk 궤적 최적화, 구 기반 충돌 | §1.1 |
| [artivis/manif](https://github.com/artivis/manif) | MIT | 2026-08 | [Sola2018] 의 Lie 군 library | §6 R-7 |

### 8.3 G1 모델 · 포구

| 저장소 | license | 마지막 push | 구현 | 대조 대상 |
|---|---|---|---|---|
| [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) | BSD-3-Clause | 2026-09 | G1 URDF 의 원본 | §0.3 |
| [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) | BSD-3-Clause | 2026-09 | Unitree 의 MuJoCo simulator | G1 sim |
| [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) | 모델별 | 2026-09 | G1 MJCF (`unitree_g1/`) | G1 MJCF 교차 확인 |
| [unitreerobotics/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) | 확인 필요 | 2026-09 | G1 양팔 IK — waist 를 잠근 위치 수준 NLP (Pinocchio + CasADi) | §2 의 대조 사례 (§6 W-5) |
| [SinaMirrazavi/LPV](https://github.com/SinaMirrazavi/LPV) | LGPL-3.0 | 2019-06 | [Salehian2016] 이 쓰는 LPV 동역학계 library. 포구 계획기 전체는 아니다 | soft catch |
| [hang0610/Catch_It](https://github.com/hang0610/Catch_It) | MIT | 2025-02 | MuJoCo 포구 환경과 강화학습 | 투척 분포 · 성공 정의 참고 |

DLR 의 포구 계획기 [Bauml2010] [Lampariello2011] 와 EPFL 의 포구 계획기 [Kim2014] 의 공개 구현은 찾지 못했다.
