# L5 — Joint Command: `ClikReferenceGenerator` 와 포구 컨트롤러 팔 명령 바인딩

이 문서는 현재 구현의 **팔 명령 계층** — `rtc_tsid` 의 `ClikReferenceGenerator` 와 그것을 부르는 포구 컨트롤러의 바인딩 — 을 표현한다. 다음 일반화 (다중 frame) 의 출발점 서술이기도 하다.

- 코드 배치: CLIK 은 `rtc_tsid` (`rtc::tsid::ClikReferenceGenerator`), 컨트롤러 바인딩은 `integrated_bringup`. 새 패키지를 만들지 않는다
- API 정의: `rtc_tsid/include/rtc_tsid/kinematics/clik_reference.hpp` · `rtc_tsid/src/kinematics/clik_reference.cpp`. 필드 · 옵션의 사양은 그 헤더 주석과 `rtc_tsid/README.md` 가 갖는다 — 이 문서는 **식과 구조 결정** 을 갖는다

---

## 1. 범위 / 비범위

이 layer 는 두 부분이다.

| 부분 | 내용 |
|---|---|
| (a) CLIK | 한 호출에 **frame 하나**. 과제는 둘 — SE3 (6 행: 기존 DemoWbc 위치 백본) 과 위치 3 + 접근축 2 의 5 행 (`PositionAxisTarget`, 포구용). posture 는 arm · hand 2 군. 옵션: 관절별 속도 한계, 가속 제약 (세 형태, §4.3), 평활 항, `max_iter`, twist feedforward, 명령값 평가 모드, 진단 (`LastSolve()`). **모든 옵션은 기본 off 이고 off 에서 기존 출력과 bit-identical** (golden 회귀 `rtc_tsid/test/test_clik_golden.cpp`) |
| (b) 컨트롤러 바인딩 | 포구 컨트롤러가 기준 (§5.3) 을 CLIK 에 넣고 결과 $q_c$ 를 `ControllerOutput` device 0 (팔) 에 position 으로 쓴다. CLIK 실패 시 QP 비의존 관절공간 abort 경로 |

비범위:
- **토크 제어.** 명령은 전부 position. sim · 실기 모두 `CommandType::kPosition`.
- **명령 경로 · 드라이버 설정.** `DeviceBackend` 와 드라이버 소관. 본 구현은 감시만 한다 (L7).
- kinematics · dynamics 구현 (`PinocchioCache` 재사용), QP solver 구현 (`QPSolverWrapper` 재사용).
- 손 관절 명령 (L6 — device 1 slot).

## 2. 코드 확인 게이트

이 절의 확인 사실은 §4–§5 가 서술한다 (명령은 `devices[0].commands` position 한 곳으로 나가고, backend 는 위치 clamp · 출력 검증 · hold 만 하며 지연 보상이 없다; frame Jacobian 은 `LOCAL_WORLD_ALIGNED`).

## 3. 참고자료

[R9] Pinocchio, [R10] ProxQP, [R11] UR ROS 2 드라이버 (position 인터페이스, `servoj` 보간), [R7] 각속도 규약.

## 4. 수학적 이론

### 4.1 속도 수준 CLIK를 쓰는 이유

관절 명령이 전부 position 이므로 QP 출력 $\dot q$ 를 한 번 적분해 $q_c$ 를 만든다. 가속도 QP 출력을 두 번 적분하면 드리프트와 솔버 잡음 증폭이 생기므로 쓰지 않는다. 적분은 CLIK 한 곳에서만 한다 (§4.3).

### 4.2 과제 정의

**평가 지점.** 포구 모드에서 CLIK 은 오차 · Jacobian 을 **명령값 $q_c$** 에서 평가한다 (`evaluate_at_command`; SE3 백본의 기본값은 측정 q). 측정 q 평가는 servo 지연을 CLIK 루프에 품어 §4.5 선행 보상과 이중 보상이 된다. 호출자가 $q_c$ (직전 `QRef()` 의 팔 성분) 와 명령 속도로 `PinocchioCache` 를 갱신해서 넘기고, CLIK 은

- 적분 anchor 를 매 tick `cache.q` 에 둔다 (`reseed_anchor` 무시),
- 첫 성공 뒤로는 (실패한 호출이 있어도 `ResetAnchor()` 전까지) 팔 성분의 `cache.q` 가 직전 `QRef()` 와 다르면 false + `command_mismatch` (배선 오류 검출. 손 성분은 L6 가 명령하므로 검사하지 않는다),
- `anchor_drift_max` 를 쓰지 않는다 (`Init` 이 거부 — 측정 q 가 없다. 실추종 감시는 L7 `TRACK_ERR`).

**병진 과제 (twist feedforward)**

$$J_p(q_c)\dot q=v_p^\ast,\qquad v_p^\ast=\dot x_{ref}+K_p\big(x_{ref}-x_C(q_c)\big)$$

$J_p$ 는 캐시 `J` 의 행 0..2 (`LOCAL_WORLD_ALIGNED`). $\dot x_{ref}$ feedforward 는 SE3 경로에서는 선택 입력 (`twist_ff`, 없으면 $r=K_x\odot e_x$) 이고 위치 + 접근축 경로에서는 목표 구조체의 일부다.

**명령 공간 폐루프임을 명시한다.** $x_C$ 는 $q_c$ 의 FK 이지 측정 $q$ 의 FK 가 아니다. 따라서 $K_p(x_{ref}-x_C(q_c))$ 는 **적분 드리프트 보정 항일 뿐 외란 제거 항이 아니다.** 실제 운동은 $\dot x_{ref}$ feedforward 가 만든다. 결과로:

1. 실기 추종 오차는 이 루프 밖에 있다. 측정 $q$ 는 L7 `TRACK_ERR` 감시 ($\Vert q-q_c(t-T_{arm})\Vert$, 임계 `supervisor.track_err_abort`) 에만 쓰인다.
2. L3 §4.6 의 $\sigma_{trk}$ 는 **실행 중 관측되지 않는 오프라인 식별값**이다. $T_{arm}$ 모델의 정확도가 그대로 포구 오차로 간다.
3. §4.4 의 1차 + 지연 모델에서 1차 성분이 유의하면 선행 보상만으로 위상이 맞지 않고 (§4.5), 그 잔차가 $\sigma_{trk}$ 의 지배 성분이 된다.

**재앵커 규칙.** $q_c$ 를 측정 $q$ 로 다시 맞추는 (re-anchor) 시점은 다음으로 한정한다 (`ResetAnchor()` 가 anchor 와 $\dot q_{prev}$ 를 함께 초기화). 그 밖의 tick 에서는 carry-forward 다.

| 시점 | 동작 |
|---|---|
| 컨트롤러 activate, arm · 재무장 (L7 §4.8) | $q_c\leftarrow q_{meas}$, $\dot q_{prev}\leftarrow0$ — $\dot q_{prev}$ 를 남기면 첫 tick 의 평활 항 ($w_s$) 이 직전 시행의 속도를 향해 당긴다 |
| E-STOP 해제 | $q_c$ · CLIK anchor 를 $q_{meas}$ 로 reseed, **자동 재개 금지**. `ClearEstop` 은 컨트롤러 fault 를 풀지 않는다 (두 경로는 별개). 전체 정책은 L7 |
| CLIK 호출 실패 | 실패한 호출의 출력은 $\dot q=0$ 이고 $\dot q_{prev}$ 도 0 으로 둔다. 다음 호출은 anchor 를 다시 잡는다 — 명령값 모드에서는 cache 가 이미 $q_c$ 라 연속이다 |

**접근축 과제 (2 행).** catch frame 의 LOCAL $z$ 축이 손바닥 바깥 법선이다 (규약). roll 제거는 LOCAL 각속도의 $x,y$ 성분만 쓰는 선택 행렬 $S=\begin{bmatrix}1&0&0\\0&1&0\end{bmatrix}$ 로 표현한다. 캐시는 LWA 만 주므로 각속도 행 (캐시 `J` 의 행 3..5) 을 catch frame 으로 회전한다.

$$J_a=S\,R_{WC}^\top J_\omega^{LWA}(q_c)\in\mathbb R^{2\times n_v},\qquad r_a=S\,R_{WC}^\top\big(K_a\,e_a+\omega_{ff}\big)$$

$e_a=\theta\hat u$ 는 회전벡터 오차다 (L4 §4.5, `rtc_math` se3 의 `AxisAlignError`). $e_a\perp z_C$ 이므로 $R_{WC}^\top e_a$ 의 $z$ 성분이 0 이고 $S$ 가 정보를 버리지 않는다. 무효 · 반평행 데드밴드 분기는 `LastAxisRegion()` 으로 호출자에게 노출된다. 입력이 비유한이거나 축이 단위가 아니면 solve 전에 실패한다.

위치 + 접근축 경로는 오차 · Jacobian 을 모두 world 정렬 (LWA) 로 쓰고 base frame 은 목표를 world 로 옮기는 데만 쓴다. SE3 경로는 base 정렬 오차와 world 정렬 `rf.J` 를 곱한다 (base 의 축이 world 와 같을 때만 맞다 — #779). 다중 frame 호출 (§5.1) 의 base frame 은 뜻이 다르다: 과제를 그 frame 기준의 상대 과제로 만들고, 오차와 행이 모두 base 의 축이다.

기존 SE3 포즈 오차 헬퍼 (`ComputeTaskPoseError`) 는 전체 SO(3) 오차라 roll 기준이 없는 이 과제에 쓰지 않는다. $J_a=f'(c)mm^\top+f(c)[a_d]_\times[z]_\times$ 를 $J_\omega^{W}$ 에 곱하는 동치 표현 (L4 §4.5) 은 $\theta\to\pi$ 에서 발산하므로 채택하지 않는다.

**자세 (posture) 과제.** 6 축에서 5-DoF 과제를 풀면 1 자유도가 남는다. posture 는 두 군이다 — 팔 ($v_{post}=K_n(q_{des}-q)$ on arm indices) 과 손 (같은 형태, 손 인덱스). 손 posture 항이 존재하는 것은 CLIK 이 결합 모델 전체 $n_v$ 를 푸는 구조이기 때문이고, 자세 기준에는 속도 feedforward 를 더할 수 있다 — $v_{post}=K_n(q_{des}-q)+\dot q_{ff}$ (`qd_posture_ff`, 없으면 앞 항만). `mpc` 의 추종 tick 이 구간의 $q_{ref}$ 를 $q_{des}$ 로, $\dot q_{ref}$ 를 $\dot q_{ff}$ 로 넘긴다 (MD-36, L7 §4.3a): 자세 행이 움직이는 관절 기준을 지연 없이 따른다. `closed_form` 은 feedforward 를 넘기지 않는다. 손 device 명령은 L6 시퀀서가 쓰므로 CLIK 의 손 출력은 팔 명령에 쓰지 않는다. 컨트롤러는 손을 solve 안에서 잠근다 (속도 box 를 $10^{-9}$ — 손바닥에 달린 catch frame 의 Jacobian 에 손 관절이 들어오므로, 풀어 두면 QP 가 일어나지 않을 손 운동으로 과제를 만족시키고 팔이 그만큼 덜 간다).

### 4.3 QP

결정변수 $v\in\mathbb R^{n_v}$ (결합 모델 전체) 의 단일 가중 box-QP 다. 위치 + 접근축 경로의 비용:

$$\min_{v}\ \tfrac12 w_{task}\Vert J_pv-v_p^\ast\Vert^2+\tfrac12 w_a\Vert J_av-r_a\Vert^2+\tfrac12 w_{arm}\Vert S_{arm}v-v_{post,arm}\Vert^2+\tfrac12 w_{hand}\Vert S_{hand}v-v_{post,hand}\Vert^2+\tfrac{\mu^2}2\Vert v\Vert^2+\tfrac{w_s}2\Vert v-\dot q_{prev}\Vert^2$$

SE3 경로는 앞 두 항이 $\tfrac12 w_{task}\Vert J_{task}v-r_{task}\Vert^2$ ($J_{task}$ 는 6 행) 하나다. 가중 순서 $w_{task}\gg w_{arm},w_{hand}\gg\mu^2$ 는 선호가 아니라 필수다 (자세 가중이 과제 가중에 닿으면 컨트롤러가 자기 자세를 추종한다 — 검증기가 거부한다). 출하 YAML 은 `w_hand` 키가 없고 `w_arm` 을 손에도 쓴다.

$$\text{s.t.}\quad \ell\le v\le\upsilon,\qquad\text{(가속 제약의 행)}$$

$$\ell_i=\max\Big(-\dot q_{\max,i},\ \frac{q_{\min,i}+m_q-q_{c,i}}{\Delta t}\Big),\qquad \upsilon_i=\min\Big(\dot q_{\max,i},\ \frac{q_{\max,i}-m_q-q_{c,i}}{\Delta t}\Big)$$

- 속도 한계는 관절별 (`v_limit_per_joint`; 비면 스칼라 `v_limit`). 마진 $m_q$ 는 CLIK 에 넘기는 `q_min`/`q_max` 를 좁혀 구현한다 (`robot.arm.limit_margin`) — 새 옵션이 아니다.
- $\Delta t$ 는 `ControllerState::dt` (= 1/`control_rate`). 가속 한계는 이 box 에 접지 않고 행으로 건다 (아래).
- $H$ 는 $\mu^2>0$ 이므로 양정치이고, 차원 $n_v$ 와 제약 수가 고정이다. QP 해는 ProxQP `eps_abs` (1e-6) 안에서만 box 를 지킨다.

**가속 창 (`box` 형태) 은 포구 층에서 쓰지 않는다 (#712).** `ClikReferenceGenerator` 에는 관절별 창 $\dot q_{prev}\pm\ddot q_{\max}\Delta t$ 를 위 box 에 접는 형태와 그 경계 충돌 규칙 (`bound_conflict` · `conflict_mask`) 이 남아 있다. 포구 컨트롤러는 그 형태를 넘기지 않는다. 그래서 `bound_conflict` 가 서지 않고, L7 의 `JOINT_CONFLICT` 는 이 컨트롤러에서 발화하지 않는다 (사유 코드 · 전이 · 기록 열은 남아 있다 — 지울지는 #755). 가속 한계와 속도 ∩ 위치 box 가 양립하지 않는 tick 은 아래 행이 hard 라 solve 실패로 끝난다.

**가속 제약의 두 형태.** $\dot v\approx(v-\dot q_c)/\Delta t$ 로 두면 두 형태 모두 $v$ 에 **선형**이다. $\dot q_c$ 는 cache 가 평가된 속도 (명령값 평가 모드에서 명령 속도 — $h$ · $\dot J$ 가 평가된 같은 상태) 의 **팔 성분**이다. 손은 다른 곳에서 명령되므로 그 가속은 이 solve 의 것이 아니다. 둘 중 하나를 고른다 (`joint_cmd.accel_constraint`). **기본값이 없다** — 키가 없거나 `TBD` 면 컨트롤러가 park 하고 ERROR 가 그 키를 지목한다 (다른 형태의 키가 같이 있어도 configure 실패가 아니라 park 다). 형태를 골랐는데 다른 형태의 키를 함께 주면 파서가 거부하고 `Init` 도 같은 규칙이다. 행은 box 아래에 부등식 행으로 붙는다 ($C=[I;\,C_a]$).

- `kinematic` — 비용이 추종하는 과제 행의 가속 $J\dot v+\dot J\dot q_c$ 를 행마다 $\pm$`task_accel_max_linear` (위치 행) · `task_accel_max_angular` (회전 · 접근축 행). $\dot J\dot q_c$ 는 등록 frame 의 classical drift (`dJv`), 접근축 행은 LOCAL x, y 에서 ($\tfrac{d}{dt}(R^\top\omega)=R^\top\dot\omega$). **그 행만** 묶는다 — 영공간 운동 (5 행 과제의 접근축 roll 등) 은 속도 box 외에 가속 한계가 없다. 관절마다 묶는 것은 `dynamic` 이다
- `dynamic` — 팔 관절 $i$ 의 토크 $\tau_i=\sum_{j\in arm}M_{ij}(q_c)\,\dot v_j+h_i(q_c,\dot q_c)$ 를 $|\tau_i|\le\eta_\tau\tau_{\max,i}$ 로 묶는다 (팔 인덱스당 한 행, 손 열은 행에 들어오지 않지만 손 속도는 $h$ 를 통해 들어온다). $M,h$ 는 cache 의 값, $\tau_{\max}$ 는 팔 device 의 `joint_limits.max_torque`, $\eta_\tau\in(0,1]$ (`joint_cmd.eta_tau`). **URDF 에 회전자 관성이 없어 $M$ 에 빠져 있다** — $\eta_\tau<1$ 이 그것을 덮는다고 가정한다 (오프라인 가속 box 도출은 MuJoCo `mj_inverse` 로 armature 포함 교차 검증했다). 실기 전 (S10) 재확인

**출하는 `dynamic` 이다.** `ur5e_p1b` · `iiwa7_leap` 의 `demo_catching_controller.yaml` 은 두 로봇 모두 `accel_constraint: dynamic`, `eta_tau: 0.8` 이다. `robot.arm.qdd_max` 의 소비자는 CLIK 이 아니라 탐색의 도달 시간, QP 없는 정지 램프, homing 이다. 그 상수 가속 한계는 포구 자세에서 토크가 허락하는 가속보다 훨씬 보수적이라, CLIK 에 걸면 명령이 기준 (특히 `mpc` 의 구간) 보다 늦는다 — `dynamic` 은 자세 의존 $M,h$ 로 그 보수성을 실행층에서 없앤다. 계획기의 도달 시간 순위 항은 여전히 `qdd_max` box 층이다.

행은 **단위 norm** 으로 스케일한다 — 가능 집합은 그대로이고, $M/\Delta t$ · $J/\Delta t$ 행이 box 행보다 $10^2$–$10^5$ 배 커서 ProxQP 가 가능한 문제에 PRIMAL_INFEASIBLE 을 내는 것을 막는다. 행이 있으면 실패한 solve 와 `ResetAnchor()` 뒤에 warm start 를 버린다 (한 번 실패한 dual 에서 시작하면 infeasible 이 이어진다). **행은 hard 다.** 속도 ∩ 위치 box 와 동시에 만족할 수 없으면 (예: 한계 근처에서 중력 토크를 못 버티는 경우) 관절별로 물러서는 해소 규칙이 없다 — 행이 관절을 결합하기 때문이다. 그때 호출은 **실패**하고 (`LastSolve().accel_rows_violated` + `converged` false — status 는 SOLVED 일 수 있다, 또는 수렴 실패 status) 아래 QP 비의존 abort 경로로 간다. 행을 깨는 명령을 돌려주지 않는다. 진단: `accel_rows` (조립한 행 수) · `accel_rows_binding` (해가 경계에 닿은 행 수). abort 경로의 감속은 형태와 무관하게 `qdd_max` box 를 쓴다.

**제동 거리 한계 (`brake_from_torque`, `dynamic` 전용 · 기본 꺼짐 — 포구 YAML 에는 아직 키가 없다).** 위 box 의 위치 항은 한 tick 앞만 본다. 속도 한계로 달리던 관절은 위치 한계 직전 tick 에 한 tick 안의 정지를 요구받고, 그 감속은 토크 행이 허용하지 않아 solve 가 실패한다. 이 옵션은 팔 관절의 box 를 멈출 수 있는 속도로 더 좁힌다:

$$0\le v_i\le\frac{2a_id_i}{a_i\Delta t+\sqrt{a_i^2\Delta t^2+2a_id_i}},\qquad a_i=m\max\Big(0,\ \frac{\eta_\tau\tau_{\max,i}+h_i}{M_{ii}}\Big),\quad d_i=q_{\max,i}-m_q-q_{c,i}$$

($q_{\min}$ 쪽은 $h_i$ 의 부호와 $d_i$ 를 바꾼 대칭.) 감속도는 토크 한계가 지금 상태에서 남기는 값이라 따로 정하는 제동 상수가 없다. 식은 연속 시간의 $\sqrt{2ad}$ 가 아니라 이산 tick 의 것이다 ($v\Delta t+v^2/(2a)=d$ 의 양의 근) — 명령은 한 tick 동안 유지되고 속도는 tick 마다 $a\Delta t$ 씩만 줄어든다. 한계는 토크 행이 한 tick 에 도달할 수 있는 속도보다 좁아지지 않는다. **보장이 아니라 실행 가능성 장치**이고 `brake_margin` ($m\lt1$) 을 두고 쓴다: $M_{ii}$ 가 관성 결합과 회전자 관성을 빼고, $a_i$ 가 상태에 따라 변하므로 $m=1$ 은 그 변동을 받을 여유가 없다. hard 제약은 여전히 토크 행이다. 유도와 §2.2 의 상수 $a_{brk}$ 식과의 차이는 [mpc_multiframe_clik_formulation.md](mpc_multiframe_clik_formulation.md) §10.3.

**반복 상한과 상태 노출.** `max_iter` (기본 20, `joint_cmd.qp.max_iter`) 를 넘거나 수렴 실패하거나 비유한 결과가 나오면 `Compute` 는 false 를 돌려준다. solver status · 반복 수 · solve time 은 `LastSolve()` 로 노출된다.

**실패 경로 — QP 비의존 관절공간 abort.** CLIK 은 실패 시 `q_ref = q_meas`, `v_ref = 0`, false 를 돌려준다. 포구 컨트롤러는 **이 출력을 소비하지 않는다** — $q_{meas}$ 로 점프하면 $q_c$ 불연속이고 $v=0$ 은 가속 한계를 무시한 즉시 정지다. 대신 QP 없이 동작하는 관절공간 경로 (`RunJointSpaceAbort` → `RampArmToStop`) 로 직전 $\dot q_{prev}$ 에서 관절별로 $\ddot q_{\max}\Delta t$ 씩 0 으로 감속하며 $q_c$ 를 적분하고 (위치 한계 clamp 포함), L7 `QP_FAILED` / `JOINT_CONFLICT` → `ABORT_SAFE` 로 넘긴다. CLIK 실패로 끝난 **시행**이 연속으로 **L7 소유의 단일 키** `supervisor.n_qp` (L7 §4.1) 에 이르면 L7 FAULT 래치 — 세는 단위는 solve 가 아니라 시행이다. 카운터는 성공한 solve 가 아니라 `HOLD` 판정 · fault reset · activation 이 지운다. L5 는 이 카운터를 새로 두지 않고 L7 판정을 참조만 한다.

**RT tick 안 try/catch 금지 (RT-2).** `ClikReferenceGenerator::Compute` 는 noexcept 이고 모든 할당 · 검증 (throw) 은 non-RT `Init` 에 있다. RT 경로의 방어는 입력 검증 (비유한 · 차원) 과 false 반환으로 한다.

**적분과 출력.** $q_c\leftarrow q_c+\dot q^\ast\Delta t$ 는 CLIK 의 carry-forward anchor 가 한다 (`QRef()`). 바인딩은 다시 적분하지 않는다 — 두 곳에서 적분하면 $q_c$ 가 이원화된다. 바인딩은 `QRef()` 의 팔 성분을 `devices[0].commands` 에 쓴다.

**중복 방지.** backend 는 관절 위치 clamp 만 한다. CLIK 에 넘기는 위치 한계는 backend 가 쓰는 YAML ∩ URDF 한계보다 `limit_margin` 만큼 안쪽이어야 backend clamp 가 발동하지 않는다. 발동하면 QP 해와 실제 명령이 달라지고 그 차이가 L7 `TRACK_ERR` 로 나타난다.

### 4.4 `servoj` 지연 모델과 식별 `[논문 외 설계]`

backend 에 지연 보상이 없다. 따라서 이 절과 §4.5 가 필요하다. 실기 식별은 `[HW-P1B]` 단계 (S10) 가 한다.

`servoj` 는 현재 상태와 목표 사이를 보간하므로 명령 대비 실제 관절에 지연이 생긴다 ([R11]). 두 가지 모델을 둔다.

- 순수 지연: $q(t)\approx q_c(t-T_{arm})$
- 1차 + 지연: $G(s)=e^{-\tau s}/(T_fs+1)$. 포구 대역 주파수 $f$ 에서의 등가 지연은 $T_{eq}(f)=\big(2\pi f\tau+\arctan(2\pi fT_f)\big)/(2\pi f)$.

식별 절차 (관절별):
1. 운용과 **같은 드라이버 파라미터**로 여기 (excitation) 궤적을 명령한다. 권장은 multisine (포구 운동 대역 포함) 과 실제 포구형 궤적 두 종류다.
2. $\dot q_c$ 와 $\dot q$ 의 정규화 상호상관 최대 위치로 순수 지연 성분의 초기값 $\hat\tau$ 를 잡는다. **상호상관 피크는 1차 지연의 $\tau$ 를 주지 않는다** — 피크는 지연이 아니라 저역통과의 위상 · 대역에 걸린다. 초기값으로만 쓴다.
3. 1차 + 지연 모델을 최소제곱으로 맞추고 (부트스트랩 CI · $R^2$ 병기), 포구 대역의 $T_{eq}$ 를 계산한다.
4. $T_{arm}=\max_iT_{eq,i}$ (보수적) 와 관절별 값을 모두 기록한다.

sim 은 지연이 없는 것이 아니다 — sim 팔의 actuator 는 position-PD (`<general>`) 라 kd/kp 시정수의 1차 지연이다. 이 식별 절차는 sim 의 `catching_diag.csv` (`q_cmd_*` · `q_meas_*`) 에도 적용된다 (LS 적합 도구 `rtc_tools/analysis/catching_trials.py` 의 `servo_lag_ls`). sim 의 값은 sim 플랜트의 값이지 `servoj` 의 값이 아니다 (§4.6). 에뮬레이션 지연은 두지 않는다.

### 4.5 지연 보상: 예측 선행 `[논문 외 설계]`

순수 지연이 지배적이면 명령 궤적을 $T_{arm}$ 만큼 앞당긴다. 시간 비교는 L0 §4.5 규약을 따른다.

1. 궤적 샘플링 · γ 프로파일 · 기준 생성은 $\text{now\_lead}=\text{now}+T_{arm}$ 에서 한다 (L2 §4.4 의 $T_{lead}=T_{arm}$). vision 예측을 앞당겨 읽는 것이지 제어 PC 가 전파하는 것이 아니다. now 는 매 tick steady 실측이며 tick 수 × dt 로 계산하지 않는다.
2. CLOSING→DECEL 전환도 now_lead ≥ $t_c$ 로 판정한다 (L7). 즉 명령 경로가 $t_c-T_{arm}$ 에 포구점에 도달한다.

손 명령 시각 $t_{cmd}$ 와 Preshape 는 **실제 시각 (now)** 축이므로 이 선행을 적용하지 않는다 (L0 §4.5, L6).

1차 필터 성분이 크면 선행만으로는 위상이 완전히 맞지 않는다. 잔여 (비-순수지연분) 는 선행을 켠 것과 끈 것의 비교로 잰다 — sim 의 1차 플랜트에서는 런타임에 (L8 §9.1 G8-E), 실기 값은 S10 이 잰다.

**$T_{arm}\neq0$ fixture 는 필수다** — $T_{arm}=0$ 이면 now 와 now_lead 가 같아져 축 혼동 버그가 숨는다. 이 요구는 sim 런타임 지연과 무관하다 (`integrated_bringup/test/arm_lag_fixture.hpp` 의 순수 지연 큐).

### 4.6 시뮬레이션 동등성

MuJoCo 팔 actuator (`<general>` position-PD) 는 `servoj` 와 동특성이 다르다. 에뮬레이션 지연 옵션은 없다. 남는 사실은 **동특성이 다르다는 것 자체**다: sim 의 position-PD 응답은 `servoj` 가 아니므로 sim 에서 잰 추종 오차를 실기 예측값으로 쓰면 안 된다. 지연 주입이 필요하면 **출하 YAML 파라미터가 아니라 테스트 fixture 전용**으로 넣는다.

### 4.7 Sanity check

1. 정지 목표: 과제 오차가 지수적으로 감소하고 roll 은 posture 과제로 정해진다.
2. $R_{WC}^\top J_\omega^{LWA}$ 로 만든 접근축 행과 유한차분 FK 일치.
3. 관절 한계 접근 시 위반 0.
4. 식별 도구의 검증은 실기 여기 궤적에서만 한다 (주입 지연 회복 테스트는 없다).
5. 옵션 전부 off 에서 기존 CLIK 출력과 bit-identical (golden).

## 5. C++ 구현

### 5.1 CLIK 구조 결정 (`rtc_tsid`)

**행 선택형 확장 — formulation 클래스를 택하지 않는다.** `ClikReferenceGenerator` 한 클래스에 옵션을 더한다. `QPSolverWrapper` · se3 오차만 공유하는 formulation 클래스는 택하지 않는다. box 조립 · 위치 한계 collapse · anchor 적분 · 실패 처리를 두 벌로 유지해야 하고, 구현이 하나뿐이라 분리 이득이 없다 (P5, ARCH-3). `rtc_controllers` 의 DLS task-velocity 법칙 (`task_vel_core`, DemoTask · DemoCompliance) 도 후보가 아니다 — box 제약이 없고, 가속 제약 · 위치 한계를 QP 로 푸는 것이 이 확장의 목적이다. **다중 frame 일반화 (E2-F04) 에서 이 결정을 다시 열었고 그대로 두었다** — 과제 목록을 받는 세 번째 `Compute` 오버로드를 같은 클래스에 더했다. box · 가속 행 · solve · anchor · 실패 분기를 그대로 공유하고, 과제 하나의 극한이 기존 오버로드와 비트 단위로 같다는 것이 게이트다 (구현이 여전히 하나다).

| 항목 | 결정 |
|---|---|
| 과제 입력 | `Compute` 오버로드 셋. 단일 과제 둘: SE3 (6 행, `twist_ff` 포인터 선택) 와 `PositionAxisTarget` (base frame 의 위치 · 접근축 (단위) · 선속도 · 각속도 feedforward, 5 행, 자세 속도 feedforward 선택) — 한 호출에 frame 하나. 다중 frame: `MultiFrameInput` 의 `FrameTask` 목록 (과제마다 종류 · frame · base · 이득 · 가중 · feedforward · 되먹임 상한), 자세 목표와 자세 속도 feedforward. 포구 컨트롤러는 `PositionAxisTarget` 오버로드를 쓴다 |
| 상대 과제 | 다중 frame 호출에서 `base_frame_idx` ≥ 0 이면 행이 base 의 축에서 본 상대 Jacobian 이다 (`relative_jacobian.hpp`). 공통 상류 관절의 열이 0 이라 그 관절이 그 과제에 동원되지 않는다. `kinematic` 과는 함께 쓸 수 없다 |
| 자세 군 | `Config::posture_groups` (서로소 관절 집합마다 가중 · 이득). 비면 팔 · 손 두 군이다 |
| 공유 코드 | box 조립, 가속 행, solve, anchor 적분 · 실패 분기는 private helper 로 공유한다 (floating-point 누적 순서를 golden 이 비트 단위로 고정) |
| 접근축 행 | $J_a=S\,R_{WC}^\top J_\omega^{LWA}$, $r_a=S\,R_{WC}^\top(K_a e_a+\omega_{ff})$, $e_a$ = `rtc::math::se3::AxisAlignError` |
| 옵션 (전부 기본 off) | 관절별 속도 한계, 가속 제약 (`box` · `kinematic` · `dynamic` — 포구 컨트롤러는 뒤의 둘만 넘긴다), 평활 가중 $w_s$, `max_iter`, SE3 경로의 twist feedforward, 명령값 평가 모드, 자세 속도 feedforward, 과제별 되먹임 상한 (다중 frame 호출), 제동 거리 한계 (`dynamic` 에서만, §4.3) |
| 명령값 평가 모드 | CLIK 안에 cache 를 두지 않는다. 호출자가 $q_c$ 로 갱신한 cache 를 넘긴다 (§4.2). anchor 는 하나 (cache.q) 다 |
| 진단 | `LastSolve()`: `reached_solve` · `converged` · `non_finite` · ProxQP status · 반복 수 · solve time · `command_mismatch` · `bound_conflict` + `conflict_mask` · `accel_rows` · `accel_rows_binding` · `accel_rows_violated`. 다중 frame 호출: `tasks` · `rejected_input` · `fb_saturated` · `rot_near_pi`. 제동 한계: `brake_active` · `brake_static_infeasible` · `brake_box_empty` |

차원: 결정변수는 **결합 모델 전체 $n_v$**. CLIK 의 control model 은 actuated 축약 모델이 있으면 그것이고 없을 때만 tree/full 로 fallback 한다. 팔 열은 `Config::arm_v_idx`, 손 열은 `hand_v_idx` 로 고른다 (이 둘이 posture 의 두 군이다). 할당은 `Init` 에서만, `Compute` 는 noexcept · 할당 0. `Manipulability()` (팔 6×6 damped √det) 는 진단값이다.

### 5.2 경계 계산 (RT)

§4.3 의 box · 가속 행은 CLIK 내부 (`AssembleBox` · `AssembleAccelRows`) 에서 계산한다.

### 5.3 컨트롤러 바인딩 (`integrated_bringup`)

포구 컨트롤러는 `RTControllerInterface` 를 상속한 코어 (rtc_controllers `catching`) + 바인딩 (`integrated_bringup`, `DemoCatchingController`) 2 층이다 (L8). 팔 명령 경로의 tick 흐름 (`Compute(const ControllerState&) noexcept`):

1. cache 를 명령 상태 ($q_c$, 명령 속도) 로 갱신 (`PrepareLawTick`; 손 성분은 측정값)
2. planner 별 기준 생성 — 아래
3. `SolveClikAndCommand`: 확장 CLIK `Compute` (catch frame index, base frame index, dt = `ControllerState::dt`, `reseed_anchor=false`)
4. 성공: `QRef()` 팔 성분 → `devices[0].commands`, `command_type = kPosition`. 실패: §4.3 관절공간 abort 경로 + L7 `QP_FAILED` / `JOINT_CONFLICT`
5. L6 시퀀서 결과 → `devices[1]` (손)
6. 진단 (status · 반복 · solve time · `bound_conflict`) 은 SPSC 로 aux drain (RT-1 — tick 에서 로깅 금지)

**CLIK 이 받는 입력은 planner 가 정한다** (단계 2).

- `closed_form` (`RunTrackingTick`): soft-catch DS 의 목표 상태 (공 샘플 $p,v,a$ at now_lead) 를 γ 프로파일과 함께 기준 생성기 (`reference_->Step`) 에 넣어 얻은 $x_{ref},\dot x_{ref}$ 가 위치 목표와 선속도 ff 다. 접근축은 계획의 고정축 $a_d$, 각속도 ff 는 없다 (고정 포구점의 접근축은 돌지 않는다 — 꾸며낸 $\omega_{ff}$ 는 목표가 돈다고 스스로 말하는 것이다). 자세 목표는 시행 시작 자세다.
- `mpc` (`RunSegmentTick`): RT 는 soft-catch DS 를 돌리지 않는다. MPC 구간의 샘플러가 catch frame 의 pose · twist 를 낸다 — 위치 목표 = 샘플 pose 의 병진, 접근축 = 샘플 pose 회전의 $z$ 열, 선속도 ff = 샘플 twist 의 선속도 부분, 각속도 ff = twist 의 각속도 부분. 자세 목표는 팔 관절마다 $q_{ref}$, 자세 행의 속도 feedforward 는 $\dot q_{ref}$ 다 ($q_{ref},\dot q_{ref}$ 는 구간 샘플의 관절 위치 · 속도 — CLIK 의 `qd_posture_ff`, §4.2). 손 성분은 시행 자세를 유지하고 feedforward 가 0 이다. 자세 행의 기준은 $K_n(q_{ref}-q)+\dot q_{ref}$ 다. 구간이 비어 있거나 샘플이 비유한이면 abort 경로다.

catch frame 은 모델 빌더가 YAML 선언 (`config/<robot>/_base.yaml`) 으로 추가한 frame 이다 — 컨트롤러는 frame 이름만 참조하고 `RegisterFrame` 으로 index 를 얻는다. 파라미터는 `LoadConfig(YAML)` + `ParseXxxParams`, runtime gain 은 `declare_parameter` (L8).

### 5.4 지연 식별 도구 (non-RT, Python, S10)

실기 $T_{arm}$ 식별 도구는 §4.4 절차의 구현이다. 입력: 컨트롤러 CSV 로그 (명령 $q_c$ = `devices[0].commands`, 측정 $q$, 공통 steady 시각). 출력: 관절별 $(\tau,T_f,T_{eq})$ 와 적합 잔차. 1차 + 지연 LS 적합 부분은 `rtc_tools/analysis/catching_trials.py` 의 `servo_lag_ls` 로 있고, 상호상관 초기값 · 등가 지연 계산은 도구에 없다 (이 절의 식이 그 명세다).

## 6. YAML 파라미터

값 · 기본값 · 범위는 출하 YAML (`integrated_bringup/config/{ur5e_p1b,iiwa7_leap}/controllers/demo_catching_controller.yaml`, 가속 box 도 주 파일의 `robot.arm.qdd_*`) 과 파서 (`rtc_controllers/src/params/catching_params.cpp`) 가 갖는다. 한 기능 안의 같은 값은 한 키만 둔다 (L3 §6). 팔 관절 이름 · 위치 / 속도 한계는 device config (`devices.<arm>`) 에서 오고 이 표의 키가 아니다.

| 키 | 단위 | 뜻 |
|---|---|---|
| `robot.arm.qdd_max` | rad/s² | 팔의 상수 가속 box (관절별). 탐색의 도달 시간, QP 없는 정지 램프, homing 이 읽는다 (CLIK 은 읽지 않는다). 위치는 주 파일 `demo_catching_controller.yaml`. 없거나 길이가 다르거나 양수가 아니면 box 없음 |
| `robot.arm.qdd_provisional` | – | 위 box 를 실기에서 써도 되는가. true (또는 키 부재) 는 sim 경고 · 실기 구성 park |
| `robot.arm.limit_margin` | rad | CLIK 에 넘기는 위치 box 를 좁히는 마진 $m_q$ (§4.3) |
| `joint_cmd.K_p` | 1/s | 위치 행 게인 (CLIK 대역; L4 `k_axis` 보다 크게) |
| `joint_cmd.K_a` | 1/s | 접근축 행 게인 $K_a$ |
| `joint_cmd.K_n` | 1/s | posture 게인 (`SetPostureGains`, 팔 · 손 같은 값). `mpc` 에서는 0 보다 커야 한다 (MD-34) — 자세 행의 기준 $K_n(q_{ref}-q)+\dot q_{ref}$ 에서 $q_{ref}$ 로 당기는 항이 이것뿐이라, 0 이면 여유 자유도가 구간에서 표류한다 |
| `joint_cmd.w_task`, `w_a`, `w_arm` | – | 가중 $w_{task}$, $w_a$, $w_{arm}$ (= $w_{hand}$). 순서 $w_{task}\gg w_{arm}\gg\mu^2$ |
| `joint_cmd.damping_sq` | – | $\mu^2$ |
| `joint_cmd.w_smooth` | – | 평활 항 $w_s$ |
| `joint_cmd.qp.max_iter` | – | ProxQP 반복 상한 |
| `joint_cmd.accel_constraint` | – | `kinematic` · `dynamic` (§4.3). 기본값 없음 — 키가 없거나 `TBD` 거나 지운 값 `box` 면 park. 선택하지 않은 형태의 키가 있으면 파서가 거부한다 |
| `joint_cmd.task_accel_max_linear` | m/s² | `kinematic` 전용 — 위치 행 가속 한계 |
| `joint_cmd.task_accel_max_angular` | rad/s² | `kinematic` 전용 — 회전 · 접근축 행 가속 한계 |
| `joint_cmd.eta_tau` | – | `dynamic` 전용 — $\eta_\tau$. $\tau_{\max}$ 는 팔 device `joint_limits.max_torque` |
| `joint_cmd.lag.T_arm` | s | §4.4 식별값 $T_{arm}$. 출하 sim 은 0 — sim actuator 의 고유 지연 보상은 sim overlay 가 `T_freeze` 와 함께 켠다. `lead_enable` 이 false 여도 검증기는 이 값을 `planner.freeze.T_freeze` 하한에 넣는다 |
| `joint_cmd.lag.lead_enable` | – | §4.5 선행 보상. 식별 전에는 false |
| `joint_cmd.lag.provisional` | – | `T_arm` 이 실기 식별값인가 (fail-closed: 키가 없으면 true). sim 은 경고, 실기 구성은 park |
| `supervisor.track_err_abort` | rad | **L7 §6 단일 원천.** L5 는 참조만 한다 |

## 7. 단위 기술 구현 순서

구현 순서의 기록은 두지 않는다 (실기 식별은 §4.4 · §9 G5-F).

## 8. 디버깅 방법

- 기록: $q_c$, $q$, $\dot q^\ast$, 경계 $(\ell,\upsilon)$, 활성 관절 인덱스, 과제 잔차 $\Vert J_p\dot q-v_p^\ast\Vert$, 축 잔차, solve time, solver status · 반복 수, `bound_conflict`.
- 과제 잔차가 크다: 한계 활성 여부, 특이 자세 (`Manipulability()`), 가중치 비율 확인.
- 접근축이 roll 과 섞인다: $R_{WC}^\top$ 회전 방향 (LWA → catch frame) 과 catch frame 축 정의 (`_base.yaml`) 확인.
- 명령 대비 측정 지연이 식별값과 다르다: 드라이버 파라미터가 식별 때와 같은지, 시뮬레이션이면 sim 서보 게인 확인.
- 관절 한계 근처에서 떨림: `limit_margin`, 가속 경계와의 충돌 빈도 확인.
- $q_c$ 가 한 틱에 튄다: CLIK 실패 출력 (`q_ref = q_meas`) 을 소비했는지, 재앵커가 §4.2 표 밖 시점에 일어났는지 확인.

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G5-A | 정지 목표에서 위치 오차 < 1 mm, 축 오차 < 0.5° 수렴 | `[SIM-ANY]` |
| G5-A2 | 옵션 전부 off 에서 기존 CLIK 출력 bit-identical, 기존 테스트 assertion 무수정 green | `[SIM-ANY]` |
| G5-B | 무작위 기준 1e4 틱에서 속도 · 가속 한계 위반 0, 위치 한계 위반은 `limit_margin` 이내. "위반 0" 은 ProxQP `eps_abs` 안을 뜻한다 | `[SIM-ANY]` |
| G5-B2 | 경계 충돌 유도 시나리오 (CLIK 의 `box` 형태): `bound_conflict` 발생, $\vert\dot q^\ast-\dot q_{prev}\vert\le\ddot q_{\max}\Delta t$ 유지. CLIK 단독으로 판정한다 — 포구 컨트롤러는 그 형태를 넘기지 않으므로 L7 전이는 판정 대상이 아니다 (§4.3) | `[SIM-ANY]` |
| G5-C | RT: page fault 0, 할당 0, QP 차원 고정, solve time 분위수 < 예산. `control_rate` 500 Hz 의 tick 2000 µs 기준 **p99 ≤ 400 µs (20 %) · 최대 ≤ 1500 µs (75 %)** — 평균이 아니라 꼬리로 건다 (L8 §5). 제어 PC 판정은 G8-A | `[SIM-ANY]` |
| G5-C2 | backend 왕복: `ControllerOutput.devices[0].commands` 와 backend 가 쓴 명령 slot 이 전 틱에서 일치 (backend clamp 미발동) | `[SIM-P1B]` / `[HW-P1B]` |
| G5-C3 | `max_iter` 설정값 준수, 초과 시 status 노출 + 관절공간 abort 경로 (가속 한계 준수), L7 `QP_FAILED` 전이. RT tick 에 try/catch 없음, `Compute` noexcept | `[SIM-ANY]` |
| G5-C4 | 재무장 · E-STOP 해제 reseed 후 첫 solve 가 $\dot q_{prev}=0$ 에서 시작 (직전 시행의 속도가 평활 항에 남지 않는다), 자동 재개 없음, `ClearEstop` 후에도 latched fault 유지 | `[SIM-ANY]` |
| G5-E | 선행 보상의 방향 확인: $T_{arm}\neq0$ 순수 지연 fixture 위에서 같은 공 · 같은 plant 로 보상 off / on 을 돌려 $t_c$ 측정 자세의 위치 오차가 줄어든다. 순수 지연 모델이라 §4.4 의 1차 성분은 빠져 있다 (sim 런타임 판정은 L8 G8-E) | `[SIM-P1B]` |
| G5-F | 실기 `T_arm` 식별 및 YAML 확정 | `[HW-P1B]` |

## 10. 미확정 항목

- `joint_cmd.task_accel_max_linear` · `task_accel_max_angular` (`kinematic` 을 쓸 때만), `supervisor.track_err_abort` (L7), 실기 `joint_cmd.lag.T_arm` (G5-F)
- E-STOP · fault 전체 정책 (L7)

## 11. catch frame (`urdf.extra_frames`)

catch frame 은 포구 컨트롤러가 정렬하는 대상이며, URDF 에 없고 **모델 빌더가 YAML 선언으로 추가하는 frame** (D-10) 이다. 사용자가 sim 에서 확인하며 바꿀 수 있게 로봇 config 의 `urdf` 절에 연다. rclcpp 파라미터는 list-of-dict 를 담지 못하므로 `urdf.sub_models.<name>.*` 와 같은 **map key** 형태다 (D-17).

```yaml
urdf:
  extra_frames:
    catch_frame:             # map key = frame 이름
      parent: l_palm_link    # 부모 frame (모델에 이미 있는 link · joint · frame)
      xyz: [0.0, 0.0, 0.0]   # m, 부모 frame 좌표
      rpy: [0.0, 0.0, 0.0]   # rad, 부모 frame 기준, URDF 관례 R = Rz·Ry·Rx
      provisional: true      # 사용자가 확인하기 전 (키가 없으면 true)
```

값은 `integrated_bringup/config/{ur5e_p1b,iiwa7_leap,g1_p1b}/` 의 로봇 config (`_base.yaml` 또는 `sim.yaml` 의 `urdf` 절) 가 갖는다. 포구 컨트롤러는 frame 이름만 참조한다 (`catching.catch_frame`, 기본 `catch_frame`).

**규약.**

- **접근축은 이 frame 의 LOCAL $+z$** 다 — 손바닥 바깥 법선이 $+z$ 가 되도록 `rpy` 를 둔다 (§4.2, L4 §4.5). 손바닥 frame 의 $+z$ 가 안쪽이면 $\pi$ 회전을 준다 (`palm_lower` 가 그렇다).
- **원점은 포구점이다** — 손이 닫혀 공을 쥐는 자리 (공 중심, L6 §4.5). 손끝 중심이나 포켓 입구가 아니다. 이유: $r_{cap}$ (L3 §4.6 게이트의 우변) 은 포구점 **둘레의 측면** 허용량이고, $t_c$ 는 손이 $\eta$ 까지 닫힌 시각이라 그때 공이 있는 자리가 원점이어야 한다. 입구를 원점으로 두면 접근축 방향으로 입구까지의 거리가 γ 창에 두 번 들어간다. "손끝" body 는 여럿이라 그 정의로는 값이 정해지지도 않는다. 손끝 중심은 포구점과 같은 $+z$ 쪽에 있고 그 차이가 $d_{eff}$ 자릿수인지 보는 교차 확인에만 쓴다.
- YAML 의 `xyz` · `rpy` 는 **부모 frame** 좌표다. L6 §4.5 의 포구점은 catch frame 좌표이므로 `rpy` 가 회전인 손에서는 값을 옮겨 쓸 때 부호가 바뀐다.

**로더가 하는 일.**

1. CM 파서 (`RtControllerNode::ParseExtraFrames`) 가 `urdf.extra_frames` 아래 map key 를 열거해 `rtc_urdf_bridge::ModelConfig::extra_frames` 를 채운다. 항목마다 `parent` · `xyz[3]` · `rpy[3]` 가 필수이고 하나라도 빠지거나 숫자 목록이 아니면 **configure 를 거부한다** (fail-closed — catch frame 이 조용히 빠지면 소비자는 자기 configure 에서야 안다). yaml-cpp `LoadModelConfig` 도 같은 키를 읽고 불완전한 항목은 예외로 거부한다. 이 키를 선언한 채 공유 모델 빌드가 실패해도 configure 를 거부한다.
2. `PinocchioModelBuilder::AddExtraFrames` 가 `BuildFullModel()` 직후 **full 모델에만** frame 을 추가한다: 부모 frame 의 관절 상대 placement 에 $\mathrm{SE3}(R(rpy),\,xyz)$ 를 합성한다. 이름이 비었거나 이미 있거나, 부모가 없거나, `xyz`·`rpy` 가 비유한이면 예외다. 새 frame 은 끝에 붙어 기존 frame id 를 바꾸지 않는다.
3. sub · tree · actuated 모델은 full 모델의 `buildReducedModel` 이라 frame 을 상속한다. 부모 관절이 잠기면 같은 world placement 로 가장 가까운 유지 조상 관절에 다시 붙는다. 네 모델 모두에서 frame 이 있고 위치가 같아야 한다.
4. 모델 빌드 때 읽으므로 값을 바꾸면 컨트롤러를 다시 configure 해야 한다.
5. 포구 컨트롤러는 `ResolveCatchFrame(model, name)` 으로 이름을 해석한다 (frame 이 없거나 `universe` 이면 `std::invalid_argument`) — 철자 오류나 frame 을 갖지 않는 sub-model 이 "아무것도 잡을 수 없다" 는 정상 모양의 지도로 나오는 것을 막는 검사다. 실패하면 팔은 hold 된다 (§5.3).

**`provisional`.** 사용자가 렌더로 frame 을 확인하기 전에는 true 다. 빌더는 이 flag 를 운반만 한다 (`ExtraFrameConfig::provisional`). 소비 규칙은 L0 §5.3 의 provisional 규칙 (sim 경고 · 실기 차단) 이고 그 판정 함수는 `CheckCatchFrameProvisional` (`catching_params.hpp`) 이다. 포구 컨트롤러는 `on_configure` 에서 모델 config 의 `extra_frames` 로부터 이 flag 를 읽어 그 함수를 부르고, 그 key (`urdf.extra_frames.<catch_frame>.provisional`) 를 소비 key 로 센다 — 실기 구성은 park 되고 (configure 는 성공, activate 는 거부) sim 은 경고만 낸다. 모델이 있는데 catch frame 이 `extra_frames` 의 항목이 아니면 (URDF 고유 frame 을 이름으로 가리킨 경우) 읽을 flag 가 없으므로 provisional 로 본다. 모델이 없으면 (URDF 미지정) 팔을 구동하지 않아 frame 을 쓰지 않으므로 판정하지 않는다.

**앵커 확인.** catch frame 의 앵커는 두 엔진 (Pinocchio · MuJoCo) 에서 **같은 frame** 이어야 한다. 포구점은 MuJoCo palm **body** frame 에서 재고 YAML 은 URDF 부모 **link** frame 이며 두 관례는 갈릴 수 있다 (이름이 같아도 `wrist_3_link` 의 body 원점은 올바른 변환에서도 어긋난다). 같은 $q$ 의 무작위 팔 자세에서 두 FK 를 대조하고, 잔차를 $L=T_{mj}T_{pin}^{-1}$ (palm 이 일치할 때만 상수) 와 $R=T_{pin}^{-1}T_{mj}$ (base 가 일치할 때만 상수) 로 분해해 판정한다. `l_palm_link` · `tool0` 는 두 모델에서 일치해 앵커로 유효하다. `ur5e_p1b` 에 남는 잔차는 MJCF 와 URDF 의 치수 차이이고 frame 불일치가 아니다 — sim 으로 줄일 수 없으므로 다른 오차 항과 합치지 않는다.
