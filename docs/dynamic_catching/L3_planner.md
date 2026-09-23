# L3 — Planner: 포구 시각·포구점·접근축·γ 결정

- 문서 버전: v0.5 (2026-09-19) — 결정·단계의 SSoT 는 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) (충돌 시 plan 우선)
- 브랜치: 단계별 `type/kebab-slug` (main 기준, 마스터 §4.2)
- 배치 `[확정 D-1]`: 탐색 코어(순수 수치)는 rtc_controllers 의 `catching` 하위 디렉토리 (namespace `rtc::catching`), 계획기 스레드 소유·YAML 은 `integrated_bringup` 바인딩
- 단계: **S1.5** 순수 조각 (도달시간 `time_feasibility`, 방향 속력, 정지거리·오차 예산), **S6** 계획기 스레드·포구 자세 IK·catchability 게이트·rollout·선택/commit·D-7a 측정
- 선행: 단계 W, L1, L2, L4 (γ rollout에 L4 코드 재사용), L5 (한계·`T_arm` 값)
- 산출물: 계획기 코어(단일 진입 함수, §4.1), 도달시간·γ 창 (참조: `time_feasibility.hpp`), 방향 속력, 계획기 스레드 (D-7), `PlanSnapshot` POD

---

## 1. 범위 / 비범위

범위: 새 예측 궤적 메시지마다(계획기 스레드, D-7 — §5.3) 다음을 결정해 `PlanSnapshot`으로 발행한다. 후보 시각은 **vision이 준 샘플 격자**에서 고른다 — 제어 PC가 궤적을 만들지 않는다(마스터 §5.2).
- 포구 시각 $t_c$, 포구점 $p_c$, 목표 접근축 $a_d$
- IK 해 $q^\ast$ (L5의 posture 참고용)과 그 자세의 manipulability $w_5$·$w_6$ (catchability, D-18·C-3 — §4.2)
- γ 프로파일 $(\gamma_f,T_w)$, 손 폐쇄 명령 시각 $t_{cmd}$
- 유효성, commit 가능 여부

비범위: 실행(L4/L5), 모드 전환(L7).

## 2. 코드 확인 게이트

단계 W에서 처리했다 (2026-09-19, plan §2). **기존 kinematics 구현을 재사용한다**(마스터 §1.2) — IK도 FK·Jacobian도 새로 만들지 않는다.

| ID | 확인 항목 | 기록 |
|---|---|---|
| G3-1 | 모델 로드 경로와 FK/Jacobian API (폐쇄 체인 손 포함 시 팔 부분만 쓰는 방법) | 닫힘 — CM 이 공유하는 `PinocchioModelBuilder` 1개 + 계획기 스레드 전용 `RtModelHandle` 1개 (스레드별 1개, heap-free, LOCAL/LWA/WORLD). P1b 손바닥 frame 은 폐쇄 루프 상류라 팔 관절 열만 쓴다 (W, D-18) |
| G3-2 | RT → 계획기 상태 전달 경로(현재 $q_c,\dot q_c$, L4 기준 상태) | 닫힘 — `rtc::SeqLock` 사용. payload 는 trivially copyable POD (`std::array` 기반, Eigen 멤버 금지) (W, plan §6) |
| G3-3 | 계획기 스레드 생성·우선순위 규약 | 닫힘 — D-7: MPC 스레드와 같은 방식 (`rtc::PeriodicRtThread` 형제 subclass), 새 thread layout role, 초기 FIFO, 정책은 D-7a 측정으로 확정 (plan §6, §7) |
| G3-4 | 손별 포켓 유효 깊이 $d_{eff}$, 포획 반경 $r_{cap}$ | 닫힘(provisional) — LEAP 80 mm / 31.0 mm, P1b ≥ 95 mm / 24 mm (S4.5, L6 §4.5). P1b 는 사용자 제공 자세가 파지 불가여서 2026-09-21 에 탐색한 자세 기준. 투척 보정은 S7.1 후 (TBD-HAND-04) |
| G3-5 | 포구 허용 작업공간, 감속 여유 공간 | TBD-BALL-02 (W7-3) |
| G3-6 | vision 샘플 간격·지평·$N$ → 후보 격자 범위 | sim 실측 간격 0.05 s · 지평 0.80 s · N 16 (S3.4 2026-09-20, TBD-VIS-04). **요구 사양 (S3.6, 2026-09-22)**: 간격 0.05 s · 지평 1.0 s (설정) · `n_max` 20 (plan §4.4 S3.6 결과) |
| G3-7 | **독립 IK/포즈 해석기가 있는지**와 그 API | 닫힘 — 독립 IK 없음. `rtc::compliance::DifferentialIk` (σ_min 적응 λ, heap-free) 를 m=5 로 재사용 (D-7d). 수렴은 G3-G 로 검증 |

## 3. 참고자료

[R1] 포구 제약과 관절 램프(도달시간 제약의 출처), [R2] 시간 슬라이스 탐색과 예측 중단 시점, [R3]/[R4] softness, [R8] 불확실성, [R9] kinematics, [R15] 부록 SQP.

## 4. 수학적 이론

### 4.1 결정 구조

후보 시각은 vision 샘플 격자에서 고른다. **시간 규약은 plan §3 (D-2) 이 SSoT 다.** 각 샘플의 시각 $t_k$ 는 공의 **물리 시각** `BallTime` (절대 steady ns) 이다 — nrt 수신 시 $t_{ref,steady}=\text{recv}_{steady}-(\text{recv}_{wall}-\text{stamp})$ 로 한 번 변환해 두므로, 계획기는 메시지 나이를 따로 빼지 않고 매 판정에서 정해진 '지금'과 직접 비교한다.

- 도달시간(§4.3)·commit(§4.11): 실제 시각 $now$ (`NowReal`, steady 실측)
- rollout(§4.8)·γ 프로파일·궤적 샘플링: 선행 시각 $now_{lead}=now+T_{arm}$ (`NowLead`)

상대시각은 수치 코어 경계에서만 만든다. 비교는 타입별 오버로드로만 한다 (S1.3). v0.4의 "샘플 $t$ 는 `header.stamp` 기준 상대시간, 나이를 빼서 쓴다" 서술은 원점이 다른 상대시각끼리 비교하는 버그 원천이라 폐기한다.

게이트 적용 순서 (연산량이 싼 것부터, D-18 반영):

$$\text{§4.4 불확실성}\to\text{§4.2 IK}\to\text{§4.2 manipulability (D-18)}\to\text{§4.3 도달시간}\to\text{§4.5 γ 창}\to\text{§4.9 정지거리}\to\text{§4.8 rollout}\to\text{§4.6 오차 예산}\to\text{L7 §4.7 충격량}$$

앞 단계에서 탈락하면 뒤 단계는 계산하지 않고 탈락 사유를 기록한다. 통과한 후보 중 §4.10 규칙으로 하나를 고르고, **§4.7 히스테리시스**로 현재 plan과 비교한다. 후보가 하나도 남지 않으면 plan 없음(포기)이며, 사유 코드를 함께 기록한다 (S6.2).

**탐색 방식 `[확정 A-4]`.** [R1]은 $(q_c,t_c)$를 동시에 푸는 NLP를 썼다. 본 구현은 1차원 시간 탐색 + IK로 시작하되, NLP 전환을 염두에 둔 경계를 유지한다 (plan §8, S6.6).

- 계획기 코어는 "입력 스냅샷(궤적 + 공분산 + 로봇 상태) → `PlanSnapshot`" **단일 진입 함수**다. 스레드(§5.3), 입출력 SeqLock, RT 쪽 소비(L4·L7), 게이트 앞단(불확실성·도달시간 사전 필터)은 탐색 전략과 독립이다
- **추상 interface 는 지금 만들지 않는다** — 구현이 하나뿐인 abstract interface 는 ARCH-3 위반이다. NLP 구현이 실제로 생길 때 두 구현을 두고 도입한다
- NLP solver 가 할당·예외를 쓰면 RT-1~10 을 지킬 수 없으므로, 그때 D-7a 는 thread layout 값만 바꿔 SCHED_OTHER 로 간다 (코드 변경 없음)
- 전환 판단 신호: IK 수렴률(G3-G), 계획 성공률, 예산 초과율, 1차원 분해가 놓치는 후보(시각·자세 결합) 사례 — S6·S8 에서 기록

NLP 정식화는 부록 A에 미래 선택지로 남긴다.

### 4.2 5-DoF 포구 자세와 IK

목표: $p_c=\hat p(t_k)$, $a_d=-\hat v(t_k)/\Vert\hat v(t_k)\Vert$.

**입력 방어 (NUM-7, plan §11).** $a_d$ 계산은 $\Vert\hat v(t_k)\Vert\ge v_{eps}$ 와 `std::isfinite`($\Vert\hat v(t_k)\Vert$) 를 **둘 다** 검사한다. 하나라도 실패하면 후보를 사유 코드와 함께 탈락시킨다 — `std::max` 류 clamp 로 값을 덮어 진행하지 않는다.

catch frame LOCAL $+z$가 손바닥 바깥 법선이다 `[확정 D-17]` — catch frame 은 모델 빌더가 YAML 선언으로 추가하는 frame 이고 (D-10, S2.3a), 부모 frame·offset·자세는 로봇 config 에서 연다 (plan §10). $a^C=R_{WC}^\top a_d$ 로 두면 접근축 오차의 LOCAL 표현은 L4 §4.5의 회전벡터다.

$$e_a^C=R_{WC}^\top e_a=\theta\,\frac{\hat e_z\times a^C}{\Vert\hat e_z\times a^C\Vert},\qquad \theta=\mathrm{atan2}(\Vert\hat e_z\times a^C\Vert,\ a^C_z)$$

$e_a^C\perp\hat e_z$ 이므로 $z$ 성분이 0이고, $S=\begin{bmatrix}1&0&0\\0&1&0\end{bmatrix}$ 가 정보를 버리지 않는다.

**갱신식은 Gauss-Newton이 아니다** — 이 점을 v0.1은 잘못 적었다. 아래 $J$ 는 잔차 $r$ 의 야코비안 $\partial r/\partial q$ 가 **아니고**, 관절속도와 과제속도를 잇는 관계일 뿐이다.

$$J(q)=\begin{bmatrix}J_p\\S\,J^L_\omega\end{bmatrix},\qquad W=\mathrm{diag}(1,1,1,\rho,\rho),\qquad e=\begin{bmatrix}p_c-p_C\\ S\,e_a^C\end{bmatrix}$$

$$\dot q_{clik}=\arg\min_{\dot q}\ \tfrac12\Vert W(J\dot q-e)\Vert^2+\tfrac12\mu\Vert\dot q\Vert^2\quad\text{s.t.}\quad \max(q_{min}-q,\,-\Delta_{max})\le\dot q\le\min(q_{max}-q,\,\Delta_{max})$$

$$\dot q_n=\big(I-J^\dagger J\big)\dot q_{sec},\qquad \boxed{\ \dot q_d=\dot q_{clik}+\dot q_n\ }$$

**갱신량은 관절 속도의 합이지 하나의 $\Delta q$ 가 아니다 `[2026-09-20 사용자 지시]`.** CLIK 이 내는 것은 $\dot q_{clik}$ 이고, 2차 과제가 보태는 것은 **영공간 관절 속도** $\dot q_n$ 이다. 둘을 하나의 스텝으로 합쳐 적으면 $N$ 이 강제하는 우선순위가 표기에서 사라진다. 반복 1회는 $\dot q_d$ 를 $\Delta t=1$ 로 적분한다 — 오프라인 root-finding 반복이지 servo tick 이 아니라서 샘플 주기가 없고, `planner.ik.dq_step_max` 는 반복당 $\Vert\dot q_d\Vert_\infty$ 상한이다.

- 위치 행: $J_p\dot q=v_p$ 에 대한 Newton 스텝. 잔차의 부호를 뒤집어 넣는다.
- 회전 행: $SJ^L_\omega\dot q=S\omega^L$ 이므로, $\omega^L=e_a^C$ 를 단위 시간 적용하면 $\exp([e_a]_\times)z=a_d$ 에 의해 **한 번에 정확히 정렬된다**(L4 §4.5). 즉 이 행은 1차 근사가 아니라 정확한 회전 갱신이고, 부호도 그래서 양수다.

두 블록은 단위가 다르다(m vs rad). $\rho$ [m/rad]는 그 스케일을 맞추는 특성길이로, 단일 $\lambda$ 아래 두 과제의 상대 가중을 결정한다. `planner.ik.rho`로 둔다. v0.1은 이 항이 없어 상대 스케일이 임의였다.

**$\rho$ 는 과제 가중이지 잔차 이득이 아니다 (S1.9 정정).** v0.5 까지 이 식은 $\rho$ 를 잔차에만 곱해 $\Delta q=J^\top(JJ^\top+\lambda^2I)^{-1}[-(p_C-p_c);\ \rho Se_a^C]$ 로 적었다. 그 형태는 차원이 맞지 않는다 — $J\Delta q$ 의 회전 행은 rad 인데 잔차의 회전 행은 m 이 되고, $\rho$ 는 단위 변환이 아니라 값 0.1 짜리 **스텝 이득**으로 작동해 회전 오차가 반복마다 $(1-\rho)$ 로만 줄어든다. 그러면 바로 위의 "한 번에 정확히 정렬된다" 도 성립하지 않는다. $J$ 와 $e$ 양쪽에 $W$ 를 곱해야 $\rho$ 가 [m/rad] 특성길이로 쓰이고, $\lambda^2=0$ 인 곳에서 $W$ 가 상쇄되어 1-스텝 정렬이 복원되며, 특이점 근처에서는 단일 $\lambda$ 가 m 블록과 rad 블록에 감쇠를 어떻게 나눌지를 $\rho$ 가 결정한다 — 이 절이 $\rho$ 에 부여한 역할 그대로다. 구현은 가중형이다 (`catch_pose_ik.hpp`).

**영공간 2차 과제 $\dot q_{sec}$ (S1.9).**

$$\dot q_{sec}=k_w\,\nabla\log w_5(q)+K_n\,(q_n-q),\qquad \dot q_n=(I-J^\dagger J)\,\dot q_{sec}$$

- $q_n$ 은 **seed 를 관절 한계로 clamp 한 것** (= wait_pose) 이다. v0.5 까지 $q_n$ 의 정의가 이 문서 어디에도 없었다. clamp 가 정의의 일부인 이유는 첫 반복부터 $q$ 가 clamp 된 값이기 때문이다 — 한계 밖 seed 를 그대로 $q_n$ 으로 두면 어떤 반복도 도달할 수 없는 자세를 목표로 잡아 $K_n(q_n-q)$ 가 **영원히 감쇠하지 않고**, 매 반복 한계 쪽으로 밀면 clamp 가 되돌리는 정상 편향이 남는다 (수렴한 해가 한계에 앉아 있는 것처럼 보인다). `planner.ik.k_null` 기본값은 0 이라 이 항은 요청하지 않으면 비활성이다.
- $K_n$ 은 `planner.ik.eps_pos` 와 **함께** 골라야 한다. $N$ 이 $\dot q_n$ 을 과제에서 안 보이게 하는 것은 **1차까지**라, 크기 $K_n\Vert q_n-q\Vert$ 의 자세 스텝은 다음 과제 스텝이 되갚아야 할 2차 잔차를 남긴다. $K_n$ 을 키우면 반복이 수렴하지 않고 정상 오차에 눌러앉는다 (S1.9 실측: 6R fixture 에서 $K_n=0.5$ 는 `eps_pos` 2e-3 에서도 $N_{IK}=200$ 을 소진했고 $K_n=0.1$ 은 수렴했다).
- $\nabla\log w_5$ 는 중심차분으로 구한다 (반복당 $2n_{arm}$ 회 Jacobian, 할당 0). $w_5$ 가 아니라 $\log w_5$ 를 올리는 이유는 특이점에 가까울수록 기울기가 커져 밀어내는 방향이 강해지고, 이득 $k_w$ 가 $w_5$ 의 혼합단위 스케일에 덜 의존하기 때문이다.
- 종료는 수락 조건 **∧** ($\Vert N\nabla\log w_5\Vert<$ `planner.ik.manip_grad_tol` ∨ $N_{IK}$) 다. 투영된 기울기로 판정한다 — 과제가 상쇄하는 성분은 쓸 수 없으므로 $\Vert\nabla\log w_5\Vert$ 로는 영원히 수렴하지 않는다. 상승이 안 끝난 채 반복 상한에 걸려도 **수락은 유지**하고 `manip_converged=false` 로 기록한다 (G3-G 신호). **중심차분 탐침이 못 쓰게 나온 반복은 수렴이 아니다** — 그 반복의 $\Vert N\nabla\log w_5\Vert$ 가 0 인 것은 도달해서가 아니라 잰 것이 없어서이고, 이를 허용오차와 비교하면 특이점 근처(탐침이 깨지기 가장 쉬운 곳)에서 상승이 한 번도 안 돈 자세를 "수렴" 으로 보고해 G3-G 신호의 부호가 뒤집힌다. 그런 반복은 `manip_converged=false` 로 두고 `manip_grad_failures` 로 따로 센다.
- 매 반복 $\Vert\dot q_d\Vert_\infty\le$ `planner.ik.dq_step_max` 로 **방향을 유지한 채 축소**한다 (성분별 clip 은 과제 방향과 영공간 방향을 함께 왜곡한다).

참 야코비안이 필요하면 L4 §4.5의 $J_a$ 를 쓴다($S[\hat e_z]_\times[a^C]_\times J_\omega^L$ 형태). 본 갱신식은 그것을 쓰지 않으므로 수렴률에 대한 Gauss-Newton 보장은 없다 — 수렴은 게이트 G3-G로 실측한다.

**과제 스텝은 제약 QP 다 `[D-7d 번복, 2026-09-20 사용자 결정 → D-26]`.** v0.5 까지는 갱신식 전체를 `DifferentialIk` (감쇠 pseudo-inverse) 로 푼다고 적었다. 관절 한계·스텝 제한을 **사후 clamp 가 아니라 부등식 제약**으로 두기 위해 $\dot q_{clik}$ 은 위 QP 로 바꾼다 (ProxQP, `rtc_tsid::QPSolverWrapper`). $\mu$ 는 `planner.ik.mu` 다 — $J^\top J$ 는 rank ≤ 5 라 어떤 팔에서도 특이하므로 $\mu>0$ 이 없으면 해가 유일하지 않다.

- **측정 근거** (2 fixture × 500 후보, plan §4.4 표): $\mu=10^{-4}$ 에서 QP 가 DLS 보다 수락률(99.2% vs 98.8%, 98.8% vs 98.2%)·잔차·한계 활성 비율에서 근소하게 앞서고 호출당 시간은 약 22% 더 든다. $\mu=10^{-8}$ 에서는 Hessian 이 거의 특이해져 대부분의 반복에서 QP 가 수렴하지 않는다 — **$\mu$ 는 절벽이 있는 손잡이**라 기본값을 provisional 로 두고 검증한다
- **제약은 $\dot q_{clik}$ 만 묶는다.** 실제로 움직이는 것은 $\dot q_d=\dot q_{clik}+\dot q_n$ 이고 $\dot q_n$ 은 QP 밖에서 계산되므로, $\Vert\dot q_d\Vert_\infty$ 축소와 관절 한계 clamp 는 **여전히 필요**하다. "한계를 제약으로" 는 과제 스텝을 고르는 방식의 개선이지 최종 적용값의 경계를 대체하지 않는다
- **후보마다 cold start 한다.** `QPSolverWrapper` 는 호출 간 warm start 를 유지하는데, 연속 호출이 **서로 다른 후보**이므로 그대로 두면 답이 탐색 순서에 의존해 지도와 런타임이 어긋난다 (§11). 이를 위해 `ResetWarmStart()` 를 rtc_tsid 에 추가했다 (enum 하나만 바꾸므로 할당 없음). 한 후보 **안의** 반복 사이 warm start 는 결정적이라 유지한다
- **QP 가 안 풀리면 fail closed 인데, 닫을 것이 있을 때만 거부다.** 비수렴 QP 는 과제 스텝을 모른다는 뜻이라 감쇠 pseudo-inverse 로 대체하지 않는다 (지도가 기록한 법칙과 다른 법칙으로 자세를 만들게 된다). 다만 **이미 허용오차를 만족한 반복이 있었다면** 그 $q^\ast$ 는 같은 법칙으로 얻은 유효한 자세이므로 그것을 반환하고 `qp_failures=1` 로 조기 종료만 기록한다 — 버리면 유효한 포구 자세가 거부로 바뀐다. 아직 수락된 반복이 없을 때만 후보를 거부한다 (`kQpFailed`). 이 구분은 D-25 상승이 생기면서 **비로소 도달 가능**해졌다: `k_manip`=0 이면 루프가 수락 즉시 끝나 두 번째 QP 가 돌지 않는다
- **의존.** rtc_controllers → rtc_tsid 엣지가 새로 생긴다 (순환 없음 — rtc_tsid 는 rtc_controllers 를 모른다). `architecture.md` §Dependency Graph 에 기록했다
- **`RtModelHandle` 은 device 관절 순서가 걸려 있으면 안 된다.** `SetJointOrder` 는 **입력만** 재배열한다 — `ComputeJacobians` 에 넘긴 $q$ 를 device 순서로 읽는 반면 `GetFrameJacobian` 의 **열**, `lowerPositionLimit(i)`, 따라서 $\dot q$ 와 box 행·clamp 는 전부 Pinocchio 순서다. 이 함수는 그 경계의 양쪽을 동시에 쓰므로 순열이 걸린 핸들에서는 모든 후보가 **유한하고 수렴하며 틀린다** (다른 팔의 자세로 간다). 그래서 `HasJointReorder()` 는 우회가 아니라 거부다 (`kJointOrderMismatch`). 실기 배선은 팔 sub-model 핸들에 `SetJointOrder` 를 건다 (`momentum_observer_wiring`) 므로 호출자가 실제로 여기 걸릴 수 있다 — 그 경우 같은 모델로 재배열 없는 핸들을 하나 더 만든다. identity 순서는 매핑을 설치하지 않으므로 모델 순서로 이름을 넘기는 보통의 호출자는 영향이 없다

$N=I-J^\dagger J$ 는 **`DifferentialIk` 가 계속 만든다** — 영공간 투영은 갱신식의 일부이지 과제 solver 의 일부가 아니다. 따라서 `planner.ik.sigma0`·`lambda_max` 는 이제 $N$ 만 파라미터화한다. $J$ 는 $m=5$ (위치 3행 LOCAL_WORLD_ALIGNED + 접근축 2행 LOCAL $x,y$) 다. $J$ 는 계획기 스레드 전용 `RtModelHandle` 에서 catch frame 의 **팔 관절 열**만 꺼낸다 (G3-1).

함수는 `rtc_controllers/include/rtc_controllers/catching/catch_pose_ik.hpp` 의 `rtc::catching::CatchPoseIk` 다 (S1.9). ROS 의존 없음, `Resize()` 뒤 할당 0·`noexcept`·무로깅 (G3-K 함수 부분), 호출 간 상태 없음. 입력 $p_c$·$\hat v$ 는 **모델 world 좌표**로 받는다 — base→world 변환은 호출자(S3.5a 지도 / S6.2 계획기) 몫이다. `DifferentialIk::Compute` 의 `ok=false` 는 **비유한 J 만** 뜻하므로 (특이 자세는 `ok=true`, σ_min≈0 — #310), 랭크 결손 판정은 $w$ 계산의 LDLT 피벗에서 하고 사유 코드를 따로 둔다.

반복마다 관절 한계로 clamp하고, 반복 상한 $N_{IK}$와 허용오차로 종료한다. **seed 는 매 후보 대기 자세(wait_pose)다** `[확정 D-18]` — 6축 5행 과제는 roll 1 자유도와 IK 해 가지가 남아 해(따라서 manipulability)가 seed 에 따라 달라지므로, 오프라인 catchability 지도(S3.5a/b)와 런타임이 **같은 함수·같은 seed·같은 YAML 키**를 써야 지도와 실제 판정이 어긋나지 않는다 (plan §11). v0.4의 "이웃 슬라이스 해 warm start" 는 해를 탐색 순서에 의존하게 만들어 폐기한다.

**roll 은 manipulability 최대화로 고른다 `[D-18 일부 번복, 2026-09-20 사용자 결정]`.** v0.5 까지 이 문단은 "roll 을 manipulability 최대화로 고르는 방식은 v1 범위 밖" 이라고 적었다. S1.9 에서 위 영공간 항 $k_w\nabla\log w_5$ 로 구현했으므로 그 문장은 폐기한다. **seed 규정은 그대로다** — 상승은 seed 가 놓인 해 가지 안의 **국소 최대**일 뿐 전역 roll 탐색이 아니라서, 지도와 런타임의 동치는 여전히 같은 seed·같은 키에 의존한다. 최대화 대상은 게이트 정의(`planner.catchability.definition`)와 무관하게 **항상 $w_5$** 다 `[확정 Q3a]` — 그래야 $q^\ast$ 가 정의에 의존하지 않아 같은 자세에서 잰 $w_5$ 와 $w_6$ 를 비교할 수 있다 (C-3). `arm_6row` 로 판정할 때는 **직접 최대화하지 않은 값으로 게이트한다**는 뜻이므로 지도 해석 시 유의한다. `planner.ik.k_manip` = 0 이면 번복 전 동작(seed 가 roll 을 결정)으로 정확히 되돌아간다.

수락 조건: 위치 오차 < $\epsilon_p$, $\theta\le\alpha_{\max}$ ([R2]의 허용 콘과 같은 취지. $\theta$ 는 위 회전벡터의 크기라 $z^\top a_d\ge\cos\alpha_{\max}$ 와 동치이면서 큰 오차에서도 수치적으로 안정하다).

**manipulability 게이트 (catchability) `[확정 D-18]`.** IK 수락 **직후** 해 $q^\ast$ 에서

$$w_5(q^\ast)=\sqrt{\det\big(J_5J_5^\top\big)},\qquad J_5=\begin{bmatrix}J_p^{LWA}\\ S\,J^{L}_\omega\end{bmatrix}_{\text{팔 관절 열}}\in\mathbb R^{5\times n_{arm}}$$

를 재고, 게이트 정의(`planner.catchability.definition`, 기본 `arm_5row`)의 값이 그 정의의 threshold (`planner.catchability.manipulability_min.arm_5row` = 0.1, provisional) 미만이면 후보를 사유 코드와 함께 탈락시킨다. 검증용으로 $w_6=\sqrt{\det(J_6J_6^\top)}$ (팔 열 6×6, roll 포함) 도 함께 계산·기록한다 (C-3, plan §11). 모든 후보가 탈락하면 plan 없음(포기)이다. 정의 세부 (plan §11):

- 손바닥 법선 둘레 roll 은 포구에 무관해 행에서 뺀다. 손 관절은 손바닥 frame 에 영향이 없다 (P1b 손바닥은 폐쇄 루프 상류)
- m 와 rad 가 섞인 값이라 threshold 0.1 은 **이 정의에 대한 값**이다. 정의를 바꾸면 다시 맞춘다
- 이 게이트는 도달시간·γ 창·정지거리 게이트에 **추가되는 AND 조건**이다. manipulability 만으로 시간 안 도달은 보장되지 않는다

**fail-closed 수치 규칙 (NUM-7, NUM-1, plan §11).** $w_5$·$w_6$ 는 고정 크기 분해(사전 할당 LDLT 의 대각 곱, 또는 고정 크기 `JacobiSVD` 의 특이값 곱·log 곱)로 계산한다 — 계획기 스레드가 FIFO 라 RT-1 이 걸리므로 동적 크기 `JacobiSVD<MatrixXd>` (할당 발생) 는 쓸 수 없다. 판정은 `det > 0` 이 아니라 분해 도중의 모든 중간값이 `isfinite` 이고 `w ≥ threshold` 인지로 한다 — 특이 근처에서 반올림으로 det 가 음수가 되거나 NaN 이 나오면 탈락이다. **기존 `ClikReferenceGenerator::Manipulability` (팔 6×6 damped) 는 게이트로 재사용하지 않는다** — roll 을 포함해 같은 값이 아닐 뿐 아니라, damped(μ² > 0) 라 특이 자세에서도 $w>0$ 을 내고 `det > 0.0` 검사가 NaN 을 0 으로 세탁한다. 진단 로그에는 둘 다 남긴다.

### 4.3 관절 도달시간 제약 ([R1] 출처, 닫힌해는 `[논문 외 유도]`)

[R1]은 관절 램프의 실현 가능성 $t\ge t_{\min,i}(q_i)$를 제약으로 썼다. 본 구현은 현재 명령 상태 $(q_{c,i},\dot q_{c,i})$에서 $(q^\ast_i,0)$까지의 최소시간을 닫힌해로 계산한다. 속도 한계 $\bar\omega$, 가속 한계 $\bar a$, 목표 방향 속도 성분 $w=s\,\dot q_{c,i}$, $D=|q^\ast_i-q_{c,i}|$:

- $w<0$ (반대 방향 이동 중): 정지 후 $D+w^2/2\bar a$를 정지 상태에서 이동.
- $w^2/2\bar a>D$ (지나침): $w/\bar a$ 후 $w^2/2\bar a-D$ 복귀.
- 삼각: $\omega_p=\sqrt{\bar aD+w^2/2}\le\bar\omega$이면 $t=(2\omega_p-w)/\bar a$.
- 사다리꼴: $t=\dfrac{\bar\omega-w}{\bar a}+\dfrac{\bar\omega}{\bar a}+\dfrac{D-\frac{\bar\omega^2-w^2}{2\bar a}-\frac{\bar\omega^2}{2\bar a}}{\bar\omega}$
- 정지 상태 이동 $T_{rest}(D)$: $\sqrt{\bar aD}\le\bar\omega$이면 $2\sqrt{D/\bar a}$, 아니면 $D/\bar\omega+\bar\omega/\bar a$.

유도 요지: 가속 구간 이동거리 $(\omega_p^2-w^2)/2\bar a$와 감속 구간 $\omega_p^2/2\bar a$의 합이 $D$.

**전제 $|w|\le\bar\omega$ 는 검사한다.** 초기 속도가 이미 속도 한계를 넘으면 최소시간 문제 자체가 정의되지 않는다 — 사다리꼴 분기의 $(\bar\omega-w)/\bar a$ 가 음수가 되어 물리적 의미가 없는 값이 조용히 나온다(예: $w_0=6$, $\bar\omega=\pi$, $\bar a=10$, $D=2$ → 0.92374 s, 그중 첫 구간이 $-0.2858$ s). 계획용 $\dot q_{\max}$(운용 여유율 적용값)와 CLIK 내부 한계가 다르거나, L5의 경계 충돌 규칙이 발동한 직후에 일어날 수 있다. `tMinChecked`가 clamp하고 플래그를 세우며, 플래그가 서면 해당 후보를 탈락시킨다.

**한계 값 $\bar a$ 의 출처 `[확정 D-16]`.** 관절 가속 한계는 토크 한계(`devices.<group>.joint_limits.max_torque` = URDF `effort` = MJCF `forcerange`)에서 오프라인 도구(S2.5, plan §9)로 도출한 **보수적 상수 box** 다. 이 box 는 CLIK 가속 box (L5, S2.2) 와 **같은 값**을 공유해야 계획이 실행과 일치한다. YAML 의 기존 `max_acceleration` (출처 없는 placeholder) 은 쓰지 않는다. 자세 의존 한계는 v1 범위 밖이다.

> **S3.5b (2026-09-22).** 이 box 로는 gate 지도가 열리지 않는다 (`ur5e_p1b` 기준 투척 주변 2835 투척 중 6; 그 이동이 토크 한계 안에 드는지를 직접 검사한 층은 590). D-16 개정 (plan §7.3 결정 B, §9) 으로 오프라인 지도는 두 층을 병기하고, 런타임의 자세 의존 한계는 S6 에서 설계한다 — 위 "v1 범위 밖" 은 그때 다시 본다.

**잘못된 한계 입력은 flag 로 보고한다.** 참조 구현 `tMinChecked` 는 $\bar a\le0$ 또는 $\bar\omega\le0$ (또는 NaN) 이면 $t=0$ 을 **아무 표시 없이** 돌려준다 — 도달시간 게이트가 무조건 통과하는 결함이다. S1.5 이식 시 한계 무효 플래그를 추가하고, 플래그가 서면 후보를 탈락시킨다 (clamp 플래그와 같은 처리). 한계 값 자체의 범위 검사는 파라미터 검증(S1.7)이 한다.

검증: 무작위 40개 조건에서 속도·가속 제약 선형계획(시간 이분 탐색) 해와 최대 차이 — python 거울 $8.3\times10^{-6}$ s, C++ `tMin` $9.5\times10^{-6}$ s (LP 격자 이산화 수준). `test_l3.cpp`가 `cases.txt`를 만들고 `verify_l3.py`가 대조한다.

제약:

$$t_k-now-T_{arm}-T_{margin}\ \ge\ \max_i t_{\min,i}$$

$t_k$ 는 `BallTime`, $now$ 는 `NowReal` (plan §3). 팔 명령이 $T_{arm}$ 뒤에 실현되므로 $T_{arm}$ 을 뺀다 — 이는 $now_{lead}$ 와 비교하는 것과 같다.

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

$$\gamma\ge\gamma_{\min}=1-\frac{d_{eff}}{\Vert v(t_k)\Vert\,T_{close,tot}},\qquad T_{close,tot}=T_{close,e2e}+T_{tick}$$

$d_{eff}$는 손바닥 접촉 전 폐쇄를 요구하면 포켓 깊이 $d$, 반발계수 $e$로 튕겨 나오기 전까지 허용하면 $d(1+1/e)$다(TBD-HAND-04). **S4.5 는 전자를 쓴다** — 접촉 시뮬레이션이 공을 손바닥에 앉힌 상태에서 재므로 반발 여유를 세지 않는다 (보수적, L6 §4.5). **S3.5b (2026-09-22): 그 값으로는 `ur5e_p1b` 의 gate 지도가 비어 있다** — 열린 투척은 전부 시각 발동 fly-in 으로 잰 상대속도 1.0 m/s (등가 $d_{eff}=v_{rel}T_{close,tot}$) 위에 있다. **`[확정 2026-09-22 사용자]` `planner.hand.d_eff` 는 둘 중 어느 것도 아닌 시각 발동 fly-in 으로 잰 허용 상대속도 × $T_{close,tot}$ 다** — P1b 0.2815 · LEAP 0.1047 m (L6 §4.5). 이 절은 $d_{eff}$ 를 $d_{eff}/T_{close,tot}$ 로만 쓰므로 식·코드는 그대로고, 포켓 깊이는 접촉 물리량으로 L6 에 남는다. 유효 조건 (런타임 손 발동도 시각 발동) 과 남긴 대안 (`planner.hand.v_rel_max` [m/s], S6 배선 시 재검토) 은 plan §7.3.

**상한 (팔 속도).** 포구 자세 $q^\ast$에서 방향 $\hat v$로 낼 수 있는 최대 속력 $v_{dir,\max}$와 TCP 속도 한계 $v_{\max}$(= L4 `reference.v_max`)로 제한한다.

> **이 상한이 v1 의 구속이다 (S3.5a 지도 2026-09-21, S4.4 2026-09-22).** 사용자 결정으로 속력 격차는 투척을 좁히는 대신 **팔 궤적으로 상대속도를 줄여** 닫는다 — 즉 위 하한 식이 쓰는 그 γ 다. 4 m·$T_f\ge1.0$ s 격자에서 $\gamma_{\min}$ 은 0.88–0.96 (팔이 5.7–8.0 m/s 로 후퇴) 인데, S4.4 가 수락 후보마다 푼 $v_{dir,\max}$ (LP, 관절 정격 × $\eta_v$) 의 중앙값은 1.1–1.4 m/s 라 $\gamma_{\max}$ 는 **0.13–0.22** 다. 구속하는 것은 TCP 한계가 아니라 **포구 자세에서의 방향 속력**이고, 그것은 자세마다 다르다 — w₅ 를 올린 자세는 $\hat v$ 로 빠른 자세가 아니다. 창이 열리는 것은 낙차가 없는 투척 (포구점이 릴리스 높이 −0.25 m 이상) 뿐이다. 수치·조건부 go 의 세 조건은 plan §4.4 S4.4 결과·§11.
>
> **`[확정 2026-09-22]` (S4.4 제안 → S3.5b 결정 D).** (1) `reference.v_max` 는 실측할 양이 아니라 **도출값**으로 둔다 — 수락 후보의 LP $v_{dir,\max}$ 최대값. URDF·제조사 자료에는 관절 정격만 있고 TCP 정격은 없다. 그러면 아래 $\min$ 의 TCP 항은 관절 정격이 허용하는 범위에서는 구속하지 않고 기준이 폭주할 때만 잡는다 (D-8 의 포화 의미 유지). (2) **$\eta_v$ 를 관절 속도 한계에도 적용한다** — $v_{dir,\max}$ 를 $\eta_v\dot q_{\max}$ 로 계산. 지금 식은 $\eta_v$ 가 TCP 항에만 붙어 있어, 구속 항이 $v_{dir,\max}$ 인 v1 에서는 아래 D-9 의 "유일한 완충" 이 사라진다. (3) 최소노름 추정 (아래 식) 은 LP 최적값의 0.90 (6축) · 0.82 (7축) 배다 (중앙값) — "차이가 작다" 는 아래 문장은 6축에만 맞는다 (S6.3 입력).

$$\gamma\le\gamma_{\max}=\frac{\min(v_{dir,\max},\ \eta_vv_{\max})}{\Vert v\Vert}$$

TCP 속도 한계를 빠뜨리면 계획이 통과시킨 γ가 L4에서 속도 포화를 일으킨다. `gammaWindow`는 두 값을 모두 인자로 받는다(v0.1 코드는 $v_{dir,\max}$만 썼다).

**여유율 `[확정 D-9]`.** `gammaWindow` 의 TCP 속도 인자는 $v_{tcp}=\eta_v\cdot$`reference.v_max` ($0<\eta_v\le1$) 이다. γ 창이 $v_{\max}$ 전체를, rollout 수락(§4.8)이 $\eta_vv_{\max}$ 를 쓰면 창은 통과했는데 rollout 에서만 탈락하는 후보가 구조적으로 생기고, 계획이 한계 끝을 쓰면 실행 중 예측 변화로 L4 가 포화한다. γ derate 가 v1 에서 빠졌으므로(D-8) 이 여유가 실행 중 유일한 완충이다. 마스터 §6 교차제약도 이 식으로 고친다 (검증은 S1.7 교차제약 표).

**입력 방어.** $v_{dir,\max}$ 는 아래 DLS 정규화식의 결과라 수치 문제로 음수가 나올 수 있다. 음수면 clamp 가 $\gamma_{\max}$ 를 0으로 **올려** 판정을 뒤집으므로, `gammaWindow` 가 비물리적 입력을 검사해 플래그를 세운다(`tMinChecked` 가 $|w_0|>\bar\omega$ 를 검사하는 것과 같은 수준).

**방향 속력 계산.** 접근축 각속도 0을 유지하며 $\hat v$ 방향 단위 속도를 내는 관절속도 $\dot q^u$를 damped least-squares로 구하고

$$v_{dir,\max}\approx\frac{\max\big(0,\ \hat v^\top J_p\dot q^u\big)}{\displaystyle\max_i\frac{|\dot q^u_i|}{\dot q_{\max,i}}}$$

로 근사한다. **분자가 필요한 이유:** DLS($\lambda>0$)는 $J\dot q^u=[\hat v;0]$ 을 정확히 만족하지 않는다. 특이 자세 근처에서 달성 속력이 1보다 작은데 v0.1은 분자를 1로 두고 $(\max_i|\dot q^u_i|/\dot q_{\max,i})^{-1}$ 만 썼다. 그러면 **실제로 낼 수 없는 속력을 보고한다** — 문서가 주장한 "보수적"의 반대다.

**분자는 노름이 아니라 $\hat v$ 방향 투영이다 (v0.5).** v0.4 의 $\Vert J_p\dot q^u\Vert$ 는 $\hat v$ 와 다른 방향으로 새는 속도 성분까지 "달성 속력"으로 세어 과대평가한다. γ 상한에 필요한 것은 공 진행 방향 성분 $\hat v^\top J_p\dot q^u$ 뿐이다. 음수(역방향)는 0 으로 둔다.

**0 가드.** $\dot q_{\max,i}\le0$ (또는 NaN) 이면 분모가 0·무한이 되어 값이 조용히 무의미해진다. 한계 무효는 플래그로 보고하고 $v_{dir,\max}=0$ (보수적)으로 둔다. 분모 자체가 0 (관절 운동 불필요)도 판정 불가로 보고 0 을 돌려준다. 두 수정은 S1.5 이식 범위다.

남는 보수성: 최소노름 해만 보므로 여유 자유도를 최대한 쓴 LP 최적값보다 작거나 같다. 6축 5-DoF 과제에서는 여유가 1자유도라 차이가 작다.

특이 자세 자체는 §4.2 의 manipulability 게이트 $w_5$ (D-18) 가 거른다. $J_p$ 가 랭크를 잃으면 $J_5$ 도 랭크를 잃어 $w_5=0$ 이므로, v0.4 의 별도 $\sqrt{\det J_pJ_p^\top}$ 게이트(`planner.ik.manip_min`)는 두지 않는다.

**Sanity check.** $v_{dir,\max}=0$(정지 포구)이면 $\Vert v\Vert\le d/T_{close,tot}$다. [R1]의 포켓 3 cm, 6 m/s를 넣으면 $T_{close}\le5$ ms다. [R1] 각주 2 원문으로 확인했다("Assuming that the ball flies within the hand about 0.03 m with a velocity of 6 m/s the time duration of 5 ms is obtained"). `test_l3.cpp`가 5 ms는 통과, 6 ms는 창이 빔을 검사한다.

**창이 비는 조건.** 받을 수 있는 최대 공 속력은

$$\Vert v\Vert_{\max}=\min(v_{dir,\max},\eta_vv_{\max})+\frac{d_{eff}}{T_{close,tot}}$$

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
2. **동결 후 감시**(§5.3 `monitorOnly`): 최신 메시지의 $t_c$ 샘플 공분산으로 $\sigma_\ell$ 을 갱신해 `PlanSnapshot` 에 싣는다. L7이 이 값의 성장을 보고 abort를 판단한다 (γ 하향은 v1 범위 밖, D-8 — §4.7). 이 경로가 없으면 §4.6의 직교 분해 이득이 실현되지 않고 `sigma_l` 은 죽은 필드가 된다.

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

`COMMITTED` 진입(commit 조건은 §4.11)부터 $p_c$, $t_c$, $\gamma$ 프로파일은 동결된다. 실행 중 포화가 예상·발생하면 `COMMITTED` 이전은 RETREAT, 이후는 ABORT_SAFE 다 `[확정 D-8]`.

**`COMMITTED` 이후 γ 하향 — v0.5 에서 v1 범위 밖 (D-8) — S8 포화 빈도 측정 후 재검토.** 아래는 v0.4 의 설계 근거로, 재도입 검토 시의 입력으로만 남긴다. 참조 구현 `derateGamma` 의 결함(Frozen 분기 γ_min 미보장, 완화 분기 무효, 기본 램프가 가속 피크를 키움, 램프가 $t_c$ 초과 가능)과 재설계 요구사항은 L4 §5.2.1 에 있다. v0.4 원문: $p_c$, $t_c$ 는 동결한 채 $\gamma_f$ 만 낮추는 것은 허용한다(L4 §5.2.1, L7 §4.6). 근거는 셋이다.

- [R3]은 γ를 매 스텝 $J(q)\dot q_{\max}$ 제약 아래 재최적화한다. 즉 로봇이 못 따라가면 그 자리에서 γ를 깎는다. 본 설계는 계획 시점 rollout(§4.8)으로 γ를 고정하므로, 예측이 빗나가거나 관절 한계가 예상보다 일찍 활성화되면 대응 수단이 abort뿐이었다.
- 포화는 $t_c$ 직전에 몰린다(L4 §4.8). abort하기에 가장 나쁜 시점이다.
- $p_c$ 고정 하에 γ만 낮출 때의 $\dot e$ 점프는 $|\dot\gamma(t)|\Vert\xi^O(t)\Vert$ 이고 $t\to t_c$ 에서 $\xi^O\to0$ 이므로 **늦게 할수록 안전하다.** $e$ 는 연속이다.

필요 가속도는 $\gamma_f$ 에 거의 선형이다(§4.8 표). 하한은 $\gamma_{\min}$(손 폐쇄 제약)이며, 그 아래로는 내릴 수 없다 — 내려야 한다면 그때는 abort다. (v0.4 근거 끝. 이 가운데 "늦게 할수록 안전하다" 는 틀렸다 — 점프는 램프 중앙에서 봉우리를 이룬다, L4 §5.2.1.)

### 4.8 γ rollout (포화 검사)

후보마다 격자 $(\gamma_f,T_w)\in\Gamma\times\mathcal T$, $\gamma_f\in[\gamma_{\min},\gamma_{\max}]$에 대해 L4 `SoftCatchTranslation`을 **포화 없이** 실행한다. 초기 상태는 현재 기준 상태, 대상은 **L2 샘플러가 vision 궤적에서 보간한 $(p,v,a)$** 다 — RT 루프가 실제로 볼 것과 같은 함수를 쓴다(`traj::sampleAt`). **rollout 시간축은 $now_{lead}=now+T_{arm}$ 이다** (plan §3 — 궤적 샘플링·γ 프로파일·기준 생성은 모두 선행 시각). 다음을 기록한다.

- 구간 $[now_{lead},t_k]$의 $\max\Vert u\Vert$, $\max\Vert\dot x\Vert$
- $t_k$의 잔여 오차 $\Vert e\Vert$

판정에는 `TranslationOutput::u_des`(포화 전 요구 가속도)를 쓴다. `xdd`(실현값)는 포화가 없으면 같지만 의미가 다르다(L4 §5.2).

수락 조건: $\max\Vert u_{des}\Vert\le\eta_aa_{\max}$, $\max\Vert\dot x\Vert\le\eta_vv_{\max}$, $\Vert e(t_k)\Vert\le\epsilon_{term}$ ($\eta<1$ 여유율).

수락 조합 중 $\gamma_f$ 최대를 고르고, 동률이면 최대 가속이 작은 것을 고른다. 수락 조합이 없으면 $\gamma_{\min}$을 한 번 더 검사한다. 그것도 실패하면 후보를 탈락시킨다.

**연산 예산 (S6.3).** 참조 구현 기준 추정으로 전 격자 rollout 이 약 26 ms 로 `planner.budget_s` (10 ms) 를 넘는다. 전 격자 × 전 후보 × 매 tick 적분을 그대로 돌릴 수 없으므로 **coarse-to-fine 이 필수**다 — 거친 단계로 먼저 거르고 통과한 조합만 세밀하게 재검사한다. 구체 방식과 실측 시간은 S6.3 에서 정한다.

참고 수치(`test_l3.cpp`, 한 시나리오, 포화 없는 rollout의 창 내 최대 $\Vert u_{des}\Vert$ [m/s²]):

| $T_w$ \ $\gamma_f$ | 0.1 | 0.2 | 0.3 | 0.4 | 0.5 |
|---|---|---|---|---|---|
| 0.30 s | 8.1 | 16.4 | 24.7 | 33.0 | 41.3 |
| 0.45 s | 5.4 | 10.5 | 15.8 | 21.2 | 26.5 |
| 0.60 s | 6.3 | 8.9 | 11.6 | 15.3 | 19.2 |
| 0.75 s | 8.9 | 8.8 | 10.4 | 12.4 | 14.8 |

$\gamma_f$ 에 대해 거의 선형이라는 점이 v0.4 §4.7 γ 하향의 근거였다 (v1 범위 밖, D-8). 짧은 창($T_w=0.30$ s)에서 $\gamma_f=0.4$ 에 33 m/s²가 필요한데, L4 기본 `a_max`가 15 m/s²임을 생각하면 창 길이 선택이 포화 여부를 지배한다.

### 4.9 정지거리 예약

포구 후 감속(L7 §4.3)에 필요한 공간을 확보한다.

$$p_{stop}=p_c+\frac{(\gamma_f\Vert v\Vert)^2}{2a_{dec}}\hat v\ \in\ \mathcal W_{catch}$$

$a_{dec}$ 는 L7 감속과 **같은 단일 키**를 읽는다 (§6, S0.3 단일 키 정리).

선택적으로 $p_{stop}$에서도 IK 수락 여부를 검사한다(YAML).

### 4.10 선택 규칙 `[권장]`

단일 가중 점수 최소화를 쓴다. γ도 그 안의 한 항이다.

$$J=w_\sigma\frac{\sigma_{\max}(t_k)}{r_{cap}}+w_t\frac{\max_it_{\min,i}}{t_k-now-T_{arm}}+w_q\Vert q^\ast-q_n\Vert^2+w_{late}\,(t_{k,\max}-t_k)-w_\gamma\,\gamma_f$$

$w_{late}>0$이면 늦은 포구를 선호한다([R1]의 "latest" 목적과 같은 취지로, 예측이 정확해지는 시간을 번다). $w_\gamma>0$이면 soft catch를 선호한다(충격량 L7 §4.7, 오차 예산 §4.6 두 근거). 가중치는 튜닝 대상이다.

**v0.1의 사전식(lexicographic) 선택을 버린 이유.** v0.1은 1순위가 "$\gamma_f$ 최대"였는데, $\gamma_f$ 가 연속 격자값이라 동률이 거의 나오지 않는다. 결과적으로 1순위에서 후보가 하나로 결정되고 $w_\sigma,w_t,w_q,w_{late}$ 네 파라미터가 전부 죽는다. "늦은 포구 선호" 같은 의도가 반영되지 않는다. $\gamma_f$ 를 $J$ 안의 항으로 넣으면 $w_\gamma$ 로 그 상충을 튜닝할 수 있다.

$\gamma_f$ 를 사실상 절대 우선으로 두고 싶으면 $w_\gamma$ 를 크게 잡으면 된다 — 사전식은 $w_\gamma\to\infty$ 의 특수한 경우다.

**대안 정식화 `[미채택]`.** [R17]은 포구를 하이브리드 시스템 안정화 문제로 보고, 포구 성공을 "충돌 후 상태가 포획 집합(capture set) 안에 드는가"로 정량화한다. 그러면 후보 $t_k$ 의 점수가 위처럼 손으로 정한 가중합이 아니라 **포획 집합까지의 여유**라는 하나의 물리량이 된다. 장점은 $w_\sigma,w_t,w_q,w_{late},w_\gamma$ 다섯 개 튜닝 파라미터가 사라진다는 것이고, 대가는 손–공 충돌 모델(반발계수, 접촉 기하, 마찰)과 그 집합의 사전 계산이다. P1b 손의 접촉 모델이 아직 없고(TBD-HAND-05, L6 §4.1), RT 경로에서 집합 소속 판정을 고정 시간에 끝낼 수 있는지도 미확인이라 v0.4에서는 채택하지 않는다. L8 이후 $w$ 튜닝이 실제로 문제가 되면 재검토 대상이다. (Schill & Buss, 2018, IEEE T-RO — 서지는 마스터 §8 [R17].)

### 4.11 commit과 손 명령 시각

- commit 조건 (APPROACH→COMMITTED, plan §3): $t_c-now_{real}\le T_{freeze}$, $T_{freeze}\ge T_{close,tot}+T_{arm}+T_{margin}$. 비교 대상은 **실제 시각**이고, 팔 선행분은 $T_{freeze}$ 하한의 $T_{arm}$ 항이 흡수한다.
- 손 폐쇄 명령 시각: $t_{cmd}=t_c-T_{close,e2e}$ (종단 간 실측값, L6 §4.1 — D-11 로 $T_{link}$ 를 따로 재지 않는다). 팔 지연은 L5 선행 보상으로 이미 흡수되므로 빼지 않는다.
- **$T_{tick}$은 여기에 넣지 않는다.** §4.5의 $T_{close,tot}=T_{close,e2e}+T_{tick}$ 에서 $T_{tick}=h/2$ 는 틱 양자화 오차의 **worst-case 예산**이다. L6 §4.3의 반올림 규칙(가장 가까운 틱)을 쓰면 오차는 $\pm h/2$ 로 영평균이라 명령 시각 자체를 당길 이유가 없다. γ 창(예산)에는 들어가고 $t_{cmd}$(명령)에는 들어가지 않는다 — 두 곳의 역할이 다르다.
- **시간 규약 `[확정 D-2]`.** $t_c$ 와 $t_{cmd}$ 는 모두 공의 **물리 시각** `BallTime` (절대 steady ns) 단일 정의다. "선행축 시각" 이나 "실제시각축 시각" 이라는 별도의 $t_c$ 는 없다 — 축은 시각 값이 아니라 **판정마다 비교하는 '지금'** 에 붙는다 (plan §3): 손 명령은 $now\ge t_{cmd}$ (실제, 손은 선행 보상 없음), DECEL 진입은 $now_{lead}\ge t_c$, rollout·γ 프로파일은 $now_{lead}$. v0.4 의 §4.11 ("실제 시각 축") 과 §5.2 주석 ("$t_c$ 는 선행축, $t_{cmd}$ 는 실제시각축") 의 모순은 이 규약으로 해소한다.
- [R2]는 접촉 직전 일정 시간부터 예측 갱신을 멈췄다. 본 구현의 동결은 포구점·γ에만 적용하고, L2의 실시간 추정은 계속 사용한다(§4.6 둘째 항).

## 5. C++ 구현

### 5.1 `time_feasibility.hpp` (참조 구현, 검증 완료)

SSoT 는 같은 폴더의 `time_feasibility.hpp` (v0.4) 다 — 문서에 코드를 복제하지 않는다. 내용: `tRest` (정지→정지 최소시간), `tMinChecked`/`TMinResult` (§4.3, `w0_clamped` 플래그), 편의 래퍼 `tMin`, `GammaWindow`/`gammaWindow` (§4.5, `input_invalid` 플래그), `maxCatchableSpeed`.

S1.5 이식 시 변경:

- 배치·명명: rtc_controllers `catching` (namespace `rtc::catching`), 함수 PascalCase (D-1, S0.3). 구조체 `GammaWindow` 와 함수 이름이 겹치지 않게 정리한다
- `tMinChecked`: 한계 $\le0$·NaN 이면 $t=0$ 을 무표시로 돌려주는 결함 → 한계 무효 플래그 추가, 호출자는 후보 탈락 (§4.3)
- 가속 한계 인자는 D-16 도출 상수 box (CLIK 가속 box 와 같은 값) (§4.3)
- `gammaWindow` 의 `v_tcp_max` 인자는 호출부에서 $\eta_v\cdot$`reference.v_max` 로 넘긴다 (D-9, §4.5)
- 테스트: `test_l3.cpp` → `cases.txt` → `verify_l3.py` 대조 결과를 GTest 고정 테이블로 이식 (S1.1), 한계 무효 경로 회귀 테스트 추가

### 5.2 `PlanSnapshot`

계획기 → RT 전달은 `rtc::SeqLock<PlanSnapshot>` 이다 (plan §6). **SeqLock payload 는 trivially copyable 이어야 하므로 `PlanSnapshot` 은 POD 다** — Eigen 멤버 금지, 벡터는 `std::array<double, N>`, 시각은 절대 정수 ns. v0.4 의 `Eigen::Vector3d`·`t_ref` + 상대시간 설계는 폐기한다 (S1.2). 필드 (타입 정의는 S1.2 에서 확정):

| 필드 | 타입 | 의미 |
|---|---|---|
| `activation_generation` | `uint64` | base `ActivationGeneration()` 값 — vision ingress·계획기가 매 스냅샷에 싣는 activation 경계 (D-23). RT 는 `IsCurrentGeneration()` 이 아니면 무효로 본다 |
| `generation` | `uint64` | 계획에 쓴 궤적의 vision 트랙 epoch (L1 §4.4, D-4) |
| `snapshot_sequence` | `uint64` | provenance token 의 단조 시퀀스 (D-21, D-22). RT 는 `SeqLock::sequence()` 를 따로 읽지 않고 이 payload 내부 값만으로 새 스냅샷 여부를 판정한다 |
| `traj_recv_ns` | `int64` (절대 steady ns) | 이 plan 이 쓴 궤적 스냅샷의 수신 시각 (D-22) — source 나이 판정 입력 |
| `rt_iteration` | `uint64` | 계산 시작 시 읽은 RT 상태 스냅샷의 tick 카운터 (D-22) |
| `rt_state_ns` | `int64` (절대 steady ns) | 계산 시작 시 읽은 RT 상태 스냅샷의 시각 (D-22) — state 나이 판정 입력 |
| `publish_ns` | `int64` (절대 steady ns) | 계획기가 이 `PlanSnapshot` 을 게시한 시각 (D-22). G3-J 의 `{snapshot_sequence, recv_steady_ns, wake_ns, publish_ns}` 이벤트 레코드와 짝을 이룬다 (plan §7.2) |
| `plan_id` | `uint32` | plan 식별 |
| `t_c_ns`, `t_cmd_ns` | `int64` (절대 steady ns) | 포구 시각·손 폐쇄 명령 시각. 둘 다 `BallTime` (§4.11, D-2) — 상대시간·축 구분 없음 |
| `p_c`, `a_d`, `v_c` | `std::array<double,3>` (W) | 포구점, 목표 접근축, 예측 포구 순간 공 속도 (감속 계획용) |
| `gamma_g0`, `gamma_gf`, `gamma_t0_ns`, `gamma_t1_ns` | `double` ×2, `int64` ×2 | L4 `GammaProfile` 파라미터. 시각은 절대 ns 이고 $now_{lead}$ 와 비교해 평가한다. 상대시간 변환은 L4 수치 코어 경계에서 |
| `gamma_min` | `double` | §4.5 γ 창 하한 (진단. v0.4 의 derate 하한 용도는 v1 범위 밖, D-8) |
| `q_star`, `nv` | `std::array<double, kMaxPlanNv>`, `int` | IK 해 (L5 posture 참고). 용량 `kMaxPlanNv` 는 **계획기 control 모델 nv** 를 담는 컴파일 타임 상수이고 (S1.2: 32), configure 에서 nv ≤ `kMaxPlanNv` 를 검사한다. 궤적 점 용량 `kCap` 과는 다른 상수다 (v0.5 문서가 둘을 같은 이름으로 불렀다) |
| `w5`, `w6` | `double` | §4.2 catchability manipulability — IK·게이트 판정용 $w_5$ 와 검증용 $w_6$ 를 정의(`planner.catchability.definition`)와 무관하게 매 후보 **항상 함께** 기록한다 (D-18, C-3, plan §11) |
| `score`, `sigma_c`, `sigma_l` | `double` | §4.10 점수, §4.6 오차 예산 두 항 (`sigma_l` 은 동결 후 `monitorOnly` 가 갱신) |
| `dp_impact` | `double` | L7 §4.7 예상 충격량 [kg m/s] |
| `reason` | `uint8` enum | plan 없음·탈락 사유 (게이트별: 불확실성, IK 실패, manipulability 미달, 도달시간, 한계 무효, γ 창, 정지거리, rollout, 오차 예산, 충격량, 지평 부족(D-15), 예산 초과) |
| `valid` | `bool` | |

**RT 소비자의 fail-closed 판정 (D-21, D-22, D-23).** 매 tick `Load()` 를 무조건 한 번 하고(D-21), payload 안에서 다음을 모두 검사한다: `activation_generation` 이 `IsCurrentGeneration()` 과 일치하는가, `generation` 이 RT 가 아는 vision epoch 과 일치하는가, `snapshot_sequence` 가 이전에 소비한 값보다 단조 증가했는가, `traj_recv_ns`·`rt_state_ns` 로 계산한 source 나이·state 나이가 각각의 상한 이내인가. 하나라도 실패하면 그 tick 은 새 payload 를 쓰지 않고 이전 유효 plan(또는 무효 상태)을 유지한다.

**S6 구현 (2026-09-23).** 새 plan 판정은 `snapshot_sequence` 가 아니라 **`plan_id`** 로 한다 — `snapshot_sequence` 는 궤적의 것이라 같은 궤적에서 계산한 두 plan 을 가르지 못한다. `plan_id` 는 writer (계획기, 또는 테스트·sim 용 oracle) 의 단조 카운터이고, RT 는 (a) `valid` · (b) `activation_generation` 일치 · (c) `generation` 이 RT 가 마지막으로 받은 궤적의 트랙 epoch 과 일치 · (d) `plan_id` 가 이미 받은 값과 다름 · (e) 게시 나이 (`now − publish_ns`) ≤ `io.t_stale` · (f) `publish_ns` 가 RT 의 마지막 리셋 시각 이후 — 를 모두 만족할 때만 받는다. (f) 는 E-STOP 처럼 activation generation 을 올리지 않는 리셋을 덮는다. **token 불일치·나이 초과를 위한 새 `Reason` 은 만들지 않는다** (값 하나가 enum·`kAllReasons`·문자열·`CatchingState.msg` 네 곳에 걸리고 msg 는 D-20 동결): TRACKING 에서는 `kNoCatchablePlan` 으로 읽는다. RT 는 `plan_box_` 에 쓰지 않는다 — writer 는 하나뿐이다 (계획기와 oracle 이 함께 켜진 설정은 park).

계획기 쪽도 대칭으로 검사한다: 계산 시작 시 최신 token 을 한 번 읽고, 게시 직전에 다시 읽어 그 사이 대체됐으면 게시를 버린다. 궤적 스냅샷의 token 과 공분산 버퍼의 token 이 다르면(궤적 N ↔ 공분산 N−1 혼합 포함) 그 조합은 쓰지 않는다.

### 5.3 계획 루프 (계획기 스레드, D-7)

**스레드 `[확정 D-7]`.** 기존 MPC 스레드와 같은 생성 방식이다 (plan §6).

- 클래스: `rtc::PeriodicRtThread` 의 **형제 subclass** (`rtc::mpc::MPCThread` 를 상속하지 않는다 — PlanSnapshot 을 `MPCSolution` 에 억지로 넣게 된다). 탐색 코어는 rtc_controllers `catching`, 스레드 소유는 `integrated_bringup` 바인딩 (D-1)
- 기동 `[확정 D-7c]`: event 구동 — nrt 파서가 새 궤적을 게시하면 eventfd 로 깨운다. `WaitForNextTick` 을 eventfd 대기 + 제한 시간으로 override, `JitterMeaningful()` 은 false. eventfd 는 여러 신호를 한 번으로 합치므로(coalescing), 깨어난 신호 횟수와 무관하게 **항상 최신 스냅샷을 읽는다**
- 수명: DemoWbc 관용구 — configure 에서 전 버퍼(궤적·공분산 버퍼, IK 작업 공간, rollout 상태, 진단 큐) 할당, activate 에서 layout profile 게이트 → lazy spawn → `Resume`, deactivate 에서 `Pause`, ~~join 은 소멸자에서만~~ → **join 은 `on_cleanup` 에서** (S6-A `/code-review`: 스레드가 설정보다 오래 살면 다음 설정에서 resume 된다 — 아래 "데이터" 절). `Pause()` 는 요청 플래그만 세우고 진행 중인 iteration 을 멈추지 않으므로, `on_deactivate` 이후에도 plan 이 한 번 더 게시될 수 있다 — RT 쪽은 그 payload 의 `activation_generation` 이 `IsCurrentGeneration()` 과 다르면 소비하지 않는다 (D-23, §5.2)
- 데이터: RT → 계획기 `rtc::SeqLock` (POD: $q_c,\dot q_c$, L4 기준 상태, 모드), 궤적 스냅샷은 nrt 파서가 게시한 SeqLock, **공분산은 계획기 쪽 버퍼에만** (A-3 — NaN(모름) 처리도 계획기 한 곳에서), 출력은 `rtc::SeqLock<PlanSnapshot>` — **S6 구현**: RT → 계획기는 `PlannerRtState` (activation generation · tick · 시각 · mode · 팔 명령 $q_c,\dot q_c$ · L4 기준 상태 $x,\dot x,\gamma,\dot\gamma$ · 현재 `plan_id` · 리셋 epoch) 를 RT tick 이 **매 tick** Store 한다. 깨우는 쪽은 nrt 파서 (새 궤적 수락 시 eventfd write) 이고 RT tick 은 eventfd 에 손대지 않는다. 재무장 리셋 (L7 §4.8) 은 RT 가 리셋 epoch 을 올리고 reset floor 를 적는 것뿐이다 (box 의 writer 는 하나). 끝난 시행을 위해 올라온 wake 신호는 따로 비우지 않는다 — 깨어날 때의 read 가 이미 소비하고, 그 시행용으로 계산된 plan 은 RT 가 reset floor 로 거른다. 리셋을 본 wake 도중에 올라온 신호는 **새 시행**의 궤적이므로 남겨 둔다 (비우면 새 시행의 첫 plan 이 wake timeout 만큼 늦는다 — 2026-09-23 `/code-review`). 수명: 계획기 스레드는 **한 configuration 의 것**이다 — `on_cleanup` 에서 join 하고 다음 activation 이 새 설정으로 다시 띄운다 (DemoWbc MPC 처럼 소멸자까지 두면 oracle 로 재구성한 뒤에도 resume 돼 box writer 가 둘이 된다)
- **RT-1~10 준수 코드** (plan §7.2 결정 방식 1): 할당 0, `noexcept`, 락·블로킹 I/O 없음, 로깅 금지. 진단(후보 수, 게이트별 탈락, 실행시간, 선택 결과)은 `rtc::SpscQueue` 로 넘기고 aux 타이머가 drain 해 CSV 로 쓴다. 그러면 FIFO/OTHER 는 thread layout 값 하나로 바뀌고 코드가 바뀌지 않는다 — 단, RT 쪽 `PlanSnapshot` 읽기는 writer 가 SCHED_OTHER 여도 D-21 의 측정(최악 재시도 시간)을 통과해야 한다 (plan §7.2 결정 방식 1)
- 배치 `[확정 D-7b → E-7 결정 J 로 대체, 2026-09-23]`: ~~빈 slot 에 새 thread layout role~~ → **기존 `mpc` role 재사용** (`SelectThreadConfigs().mpc.main`, 스레드 이름 `mpc_main`, tier ≥ 6 slot 3 FIFO 60 · tier 4 OTHER). 계획기는 MPC 와 같은 역할이고 CM 이 active 컨트롤러를 하나만 두므로 같은 코어에 동시에 도는 FIFO 는 하나다 (plan §6). activate 게이트: `planner.enabled` && profile `mpc_off` 면 `on_activate` 첫 문장 FAILURE
- 스케줄러 D-7a: S6.5 에서 제어 PC 부하 상태로 FIFO·OTHER 각각 측정 (수신 → plan 게시 지연 p50·p99·최대, 예산 초과율, ≥ 1000 시행). FIFO 가 p99 를 `budget_s` 의 10% 이상 줄이거나 예산 초과율을 줄이면 FIFO 유지, 아니면 SCHED_OTHER (A-2)
- 복사하지 않을 것: 현 MPC 경로의 `MPCSolutionManager::PublishSolution` mutex·try/catch, `HandlerMPCThread` 의 `fprintf` (plan §6)

**한 번 깨어났을 때의 순서** (§4.1 단일 진입 함수의 본문):

1. 궤적 스냅샷·공분산 읽기. 트랙 epoch 가 막 바뀌었으면 `n_settle` 동안 invalid 게시 (§4.4)
2. RT 상태 읽기. 동결 모드면 `monitorOnly()` 만 수행 (아래)
3. vision 샘플 격자의 각 $k$ 에 대해: 작업공간·지평 검사 (L2 §4.6) → §4.1 게이트 순서 → 점수 (§4.10). 탈락 사유는 진단 큐로
4. 예산 `budget_s` 초과 시 남은 슬라이스를 건너뛰고 지금까지의 최선을 쓴다
5. 후보가 없으면 invalid + 사유 게시, 있으면 §4.7 히스테리시스를 거쳐 게시

- `isFrozen(mode)` 는 술어 함수로 둔다. `mode >= Mode::Committed` 같은 enum 나열 순서 의존은 상태를 추가하면 조용히 깨진다.
- **동결 중에도 `monitorOnly()` 는 돈다**(§4.6): $\sigma_\ell$, σ 성장률, L1 $\bar\nu$ 를 갱신 발행해 L7이 abort 판단에 쓴다. v0.2는 즉시 return 해서 `PlanSnapshot::sigma_l` 이 영구히 죽은 필드였다.
- 시행 종료 시 plan 무효화는 L7이 한다(L7 §4.8).
- 후보 시각은 vision 샘플 격자를 그대로 쓴다. `planner.slice.dt`가 카메라 주기보다 크면 격자를 솎아 쓰고, 작으면 보간해 쓴다(L2 `sampleAt`). 어느 쪽이든 격자 간격은 `TBD-VIS-04` 확정 후 정한다.

### 5.4 방향 속력 (§4.5)

v0.4 문서의 코드 스케치는 삭제한다 (참조 헤더에 없고, 분자가 노름이며 0 가드가 없었다). S1.5 에서 확정할 요구사항:

- 입력: §4.2 의 $J_5$ (팔 관절 열, 고정 최대 크기), $\hat v$, $\dot q_{\max}$. 출력: $v_{dir,\max}$ 와 무효 플래그
- 분자는 투영 $\max(0,\hat v^\top J_p\dot q^u)$, 분모 $\max_i|\dot q^u_i|/\dot q_{\max,i}$ (§4.5)
- $\dot q_{\max,i}\le0$·NaN 은 플래그 + 0, 분모 0 은 0 (보수적)
- 할당 0, `noexcept` (계획기 스레드가 RT-1~10 준수)

## 6. YAML 파라미터

로딩은 컨트롤러 `LoadConfig(YAML)` → `ParseXxxParams` (on_configure, non-RT), runtime 변경 gain 만 `declare_parameter` (L0 §5.3). 여러 층이 쓰는 같은 물리량은 **단일 키 하나**만 두고 다른 층은 그 키를 읽는다 (S0.3) — "같은 값" 일치 검사는 두지 않는다.

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `planner.enabled` | bool | – | false | – | 계획기 스레드를 띄운다 (S6-A). `diagnostic.oracle_plan.enabled` 와 동시 true 면 park — `plan_box_` 의 writer 는 하나다 |
| `planner.wake_timeout_s` | double | s | 0.05 | 0.005–0.5 | 새 궤적이 없어도 깨어나는 상한 (결정 H). `PeriodicRtThread` 가 양수 주기를 요구하므로 이 값이 그 주기다 |
| `planner.budget_s` | double | s | **0.020** | 0.001–0.05 | 한 사이클 계산 예산. 2026-09-23 **R-2**: 후보당 IK 가 개발 PC 6R p50 1.8 ms · 7R 2.2 ms 라 0.010 으로는 후보 5 개도 못 본다 → 0.020 + 사전 필터 (IK 후보 ≤ 8). vision 주기 0.05 s 안 |
| `planner.wait_pose` | double[] | rad | provisional | 관절 한계 안, 길이 = arm dof | IK seed 이자 대기 자세 (결정 L, arm 관절 순서). p1b `[0.212, −1.376, 1.107, −1.978, −3.296, 0.121]` · leap `[0, 1.0, 0, −1.2, 0, 1.2, 0]` (S3.5a/b 지도의 대기 자세). homing 은 S7.2 |
| `planner.slice.dt` | double | s | **0.05** | 0.005–0.05 | vision 샘플 간격의 정수배. **S3.6 이 `prediction.dt_expected` 를 0.05 s 로 정했다** (2026-09-22, provisional — L2 §6). S6 착수 시 (2026-09-23) vision 간격 그대로로 정함 — 후보는 vision 격자 그대로라 공분산 시각 보간이 필요 없다 |
| `planner.slice.t_lead_min` | double | s | = `planner.freeze.T_freeze` | >0 | $T_{freeze}$ 이상 (2026-09-23: 같은 값으로 정함) |
| `planner.slice.t_max` | double | s | = 지평 − margin | 0.2–1.5 | vision 지평 − `prediction.t_horizon_margin` 이하 (S3.6 설정 profile 1.0 s 면 ≤ 0.95 s). 2026-09-23: 그 상한 그대로로 정함 |
| `planner.n_settle` | int | – | 3 | 0–20 | §4.4 트랙 epoch 변경 후 대기 메시지 수 |
| `planner.gamma.margin` | double | m/s | 0.1 | 0–1 | §4.5 `maxCatchableSpeed` 경계 여유 (1 ulp 엇갈림 방지) |
| `planner.ik.max_iter` | int | – | 20 | 1–100 | 연산 예산 |
| `planner.ik.lambda` | – | – | – | – | v0.5 에서 삭제 — 고정 λ 대신 `DifferentialIk` 의 σ_min 적응 λ (D-7d). 그 파라미터는 아래 `sigma0`·`lambda_max` 다 |
| `planner.ik.sigma0` | double | – | 1e-3 (**provisional**) | >0 | §6.5 감쇠 shell 진입 σ_min — **영공간 투영 $N$ 만** 파라미터화한다 (과제 스텝은 QP, D-26). **S6.2 가 아니라 S1.9 에서 정한다** — S3.5a 지도가 S6.2 보다 먼저 같은 함수를 돌리고 지도와 런타임은 같은 키를 써야 한다 (plan §11) |
| `planner.ik.lambda_max` | double | – | 1e-2 (**provisional**) | ≥0 | §6.5 최대 감쇠. $N$ 전용, 위와 같은 이유로 S1.9 |
| `planner.ik.mu` | double | – | 1e-4 (**provisional**) | >0 | §4.2 과제 QP 정칙화. $J^\top J$ 가 rank ≤ 5 라 필수다. 1e-8 은 QP 비수렴, 1e-2 는 수락률 하락 — 절벽이 있으니 값을 바꾸면 재측정한다 (plan §4.4) |
| `planner.ik.qp_eps_abs` | double | – | 1e-10 (**provisional**) | >0 | 과제 QP 절대 허용오차. TSID tick 기본값 1e-6 을 그대로 쓰면 IK 잔차에 solver 바닥이 생긴다 |
| `planner.ik.qp_max_iter` | int | – | 50 | ≥1 | 과제 QP 반복 상한 |
| `planner.ik.rho` | double | m/rad | 0.1 | 0.01–1 | §4.2 위치/회전 스케일 정합 (특성길이). **$J$ 와 잔차 양쪽에 가중** (S1.9 정정) |
| `planner.ik.dq_step_max` | double | rad | 0.15 (**provisional**) | >0 | 반복당 $\Vert\dot q_d\Vert_\infty$ 상한 (방향 유지 축소, $\Delta t=1$) |
| `planner.ik.k_null` | double | 1/step | 0 | ≥0 | §4.2 영공간 자세 과제 $K_n$. $q_n$ = **한계로 clamp 한** seed. `eps_pos` 와 함께 고른다 (2차 잔차) |
| `planner.ik.k_manip` | double | – | 0 (**provisional**) | ≥0 | §4.2 영공간 $\log w_5$ 상승 이득 $k_w$. 0 이면 D-18 번복 전 동작 |
| `planner.ik.manip_grad_tol` | double | – | 1e-4 (**provisional**) | ≥0 | 상승 종료 판정 $\Vert N\nabla\log w_5\Vert$ |
| `planner.ik.v_eps` | double | m/s | 1e-6 | >0 | §4.2 NUM-7 속력 하한 — 미만이면 clamp 가 아니라 탈락 |
| `planner.ik.eps_pos` | double | m | 0.002 | – | 수락 |
| `planner.ik.alpha_max` | double | rad | **0.26 (provisional)** | 0–π/2 | 손 형상 허용 콘 ($\theta\le\alpha_{\max}$). 2026-09-21: `TBD` 로 적혀 있었으나 코드에는 provisional 기본값 0.26 rad (≈15°) 가 살아 있었다 — `ParseCatchPoseIkParams` 신설 때 드러난 불일치이고, 값은 코드 쪽으로 맞췄다. 닫는 근거는 S3.5a 지도의 `theta` 분포 (이 콘이 실제로 구속하는지) 다 |
| `planner.ik.manip_min` | – | – | – | – | v0.5 에서 삭제 — `planner.catchability.manipulability_min` 이 대체 (§4.5) |
| `planner.catchability.manipulability_min.arm_5row` / `.arm_6row` | double | – | 0.1 (**provisional**) / TBD | ≥0 | §4.2 D-18 정의별 하한 (차원이 달라 따로 둔다, C-3). `arm_6row` 값은 S3.5a/b 지도 결과로 제안. 사용자가 sim 에서 자세 확인 후 갱신. provisional 이면 실기 구성 arm 차단 (L0 §5.3). S3.5a/b 지도 도구와 **같은 키** |
| `planner.catchability.definition` | string | – | `"arm_5row"` | `arm_5row` \| `arm_6row` | §4.2 게이트에 쓸 정의. w₅·w₆ 는 정의와 무관하게 둘 다 기록 |
| `planner.time.margin` | double | s | 0.03 | 0–0.2 | §4.3 |
| `planner.unc.kappa_sigma` | double | – | 0.3 | 0.05–1 | §4.4 |
| `planner.hand.d_eff` | double | m | LEAP **0.1047** / P1b **0.2815** | >0 | **시각 발동 fly-in 허용 상대속도 1.0 m/s × $T_{close,tot}$** (2026-09-22 확정 — §4.5, L6 §4.5, plan §7.3). 포켓 깊이 (0.080 / 0.095 m, S4.5) 가 아니다. provisional. **소비자 없음** — 파서는 이 키를 읽지 않는다 (S6 의 γ 창이 첫 소비자) |
| `planner.hand.r_cap` | double | m | LEAP **0.031** / P1b **0.024** | >0 | S4.5 실측 (L6 §4.5), provisional. 측면 허용량이며 공 중심 좌표계라 공 반지름이 이미 포함돼 있다 |
| `planner.gamma.grid` | double[] | – | [0.0, 0.1, …, 0.6] | 0–1 | §4.8 |
| `planner.gamma.window_grid` | double[] | s | [0.3, 0.45, 0.6] | >0 | §4.8 |
| `planner.gamma.eta_a`, `eta_v` | double | – | 0.8, 0.9 | (0, 1] | 여유율. `eta_v` 는 D-9 의 $\eta_v$ — `gammaWindow` 와 rollout 수락이 같은 값을 쓴다 (§4.5, §4.8) |
| `planner.gamma.eps_term` | double | m | 0.002 | – | §4.8 |
| `planner.budget.n_sigma` | double | – | 2.0 | 1–3 | §4.6 |
| `planner.budget.sigma_trk` | double | m | `TBD` | ≥0 | L5 실측. ⚠️ **sim 초기값 출처가 없다** — S3.7 이 2026-09-20 결정으로 빠져 (L5 §7 L5.7) **S10 실기 식별까지 TBD 로 남는다** |
| `planner.budget.clock_err` | double | s | `TBD` | ≥0 | 인프라 실측 |
| `planner.stop.a_dec` | – | – | – | – | v0.5 에서 삭제 — 단일 키 `supervisor.decel.a_dec` (L7 §6) 를 읽는다 (§4.9) |
| `planner.stop.check_ik` | bool | – | true | – | §4.9 |
| `planner.switch.delta_J` | double | – | 0.1 | ≥0 | §4.7 |
| `planner.switch.e_jump_max` | double | m | 0.01 | >0 | §4.7 |
| `planner.switch.ed_jump_max` | double | m/s | 0.05 | >0 | §4.7 교체 시 $\dot e$ 점프 한계. 단일 키 (L7 은 γ 하향용으로 읽었으나 v1 범위 밖, D-8) |
| `planner.gamma.derate_step` | – | – | – | – | v1 범위 밖 (D-8) — γ derate 재도입 시 단일 키로 다시 정한다 |
| `planner.freeze.T_freeze` | double | s | p1b **0.36** · leap **0.19** (provisional) | ≥ §4.11 하한 | §4.11 하한식 (결정 G, 2026-09-23) |
| `planner.score.w_sigma`, `w_t`, `w_q`, `w_late`, `w_gamma` | double | – | 1, 1, 0.1, 0, 5 | ≥0 | §4.10 튜닝. `w_gamma`를 크게 잡으면 사전식 선택과 같아진다 |
| `planner.workspace.catch_box` | box | m | sim: S3.5b 외접 상자 + 0.1 m / 실기 provisional | – | TBD-BALL-02. 모양 (결정 I, 2026-09-23): base 원점 기준 축정렬 상자 `{min: [x,y,z], max: [x,y,z]}`. 판정 게이트 — $p_c$ 와 $p_{stop}$ 이 모두 안에 있어야 한다 |

## 7. 단위 기술 구현 순서

단계 매핑 (plan §4, §14.2): L3.1·L3.3·L3.5 = **S1.5**, L3.2 = **S1.9** (IK+catchability 함수 — S3.5a/b 지도 도구와 S6.2 런타임이 공유) → **S6.2** (스레드로 배선), L3.4·L3.6·L3.7 = **S6** (S6.3 γ 창·rollout, S6.4 선택·commit, S6.1 스레드, S6.5 D-7a 측정).

- **L3.1** `time_feasibility.hpp` + LP 대조 테스트(`test_l3.cpp` → `cases.txt` → `verify_l3.py`, 결과를 고정 테이블로 GTest화) + `w0_clamped` 경로 테스트.
- **L3.2** 포구 자세 IK (`DifferentialIk` m=5 + 전용 `RtModelHandle`, seed = wait_pose) + manipulability 게이트 (D-18, S3.5a/b 도구와 같은 함수) + 수렴률·콘·스케일($\rho$) 테스트.
- **L3.3** 방향 속력 (§5.4, 투영·0 가드) + γ 창 테스트([R1] 수치 sanity, `v_tcp_max` 구속, `maxCatchableSpeed` 포함).
- **L3.4** γ rollout (L4 코드 호출, $now_{lead}$ 축) + §4.8 표 재현 테스트 + coarse-to-fine (예산 초과 시, S6.3).
- **L3.5** 오차 예산(§4.6 직교 분해)·정지거리 게이트.
- **L3.6** 선택·히스테리시스·commit (γ 하향 경로는 v1 범위 밖, D-8).
- **L3.7** 계획기 스레드 (D-7, §5.3), 예산 관리, SPSC 진단, D-7a 측정 (S6.5).
- **L3.8** (미래 선택지) 부록 A NLP — 전환 신호(§4.1)가 필요를 보일 때만 (A-4).

L3.1·L3.3·L3.4는 `test_l3.cpp`가 참조 구현을 이미 돌리고 있다. **L3.1·L3.3·L3.5 (S1.5) 는 한계·γ창 값을 인자로 받는 값-매개변수 공식이라 실측값 없이도 이식·테스트할 수 있다** — `planner.gamma.*`, `planner.hand.*`, `reference.a_max` 가 provisional 이어도 S1.5 착수를 막지 않는다. 반대로 **S6 착수 전에는 마스터 §4.1의 $T_{close,tot}$ 선행 측정(S4 손 타이밍 go/no-go)을 끝낼 것** — 이 값이 실제로 필요한 것은 L3.2 의 런타임 배선(S6.2)·L3.4·L3.6·L3.7 (S6) 이다. 값이 좌우하는 파라미터는 위와 같다.

## 8. 디버깅 방법

- 계획마다 기록: 후보 수, 게이트별 탈락 수(히스토그램), 선택 후보의 $(t_c,p_c,\gamma_f,T_w,\sigma_{\max},\max t_{\min})$, 실행시간, 교체 여부와 사유.
- "항상 탈락": 게이트별 탈락 히스토그램에서 첫 번째 병목을 찾는다. γ 창이 원인이면 $d_{eff}$, $T_{close}$, $v_{dir,\max}$ 값을 먼저 의심한다.
- 포구점이 자주 바뀐다: `delta_J`, 점프 한계, 예측 품질(L2 `lastJump`)을 확인한다.
- IK 수렴 실패: catch frame 축 정의 (D-17 YAML), `alpha_max`, seed(wait_pose)와 후보 자세의 거리를 확인한다.
- manipulability 탈락이 지배적: $w_5$ 분포와 S3.5a/b 지도 결과를 대조한다. 지도와 런타임이 다른 함수·키·seed 를 쓰고 있지 않은지 먼저 본다. arm base frame 이 로봇 config 의 CLIK `base_frame` 인지 확인한다 (ur5e_p1b `base` vs `base_link` 180° — plan §11).
- 시뮬레이션 시각화(RViz): vision 예측 궤적(시각화용 `nav_msgs/Path` 로 재발행), 후보 점(색 = 탈락 사유), 선택된 $p_c$와 $a_d$, $p_{stop}$.

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G3-A | `tMin` 고정 테이블 일치 (< 1e-9, 스크립트 기준값 대비) + `w0_clamped`·한계 무효 플래그 시 후보 탈락 | `[SIM-ANY]` |
| G3-B | γ 창 sanity ([R1] 3 cm / 6 m/s → 5 ms 통과, 6 ms 탈락), `v_tcp_max` = $\eta_v$`reference.v_max` 구속 (D-9), §4.8 표 재현(±5%), 방향 속력 투영·0 가드 | `[SIM-ANY]` |
| G3-C | 합성 투척 1000회: 계획 실행시간 99% < `budget_s`, 탈락 사유 분포 기록 | `[SIM-ANY]` |
| G3-D | `iiwa7_leap` 시뮬레이션에서 plan 유효율, 교체 빈도 기록 | `[SIM-ANY]` |
| G3-E | `ur5e_p1b` 시뮬레이션에서 선택된 plan의 L4/L5 실행 시 포화 0, 한계 활성 비율, **포화 발생 빈도** 기록 (D-8 재검토 입력) | `[SIM-P1B]` |
| G3-F | 실측 $T_{close}$, $T_{arm}$, $\sigma_{trk}$ 반영 후 γ 창·오차 예산 재산정. $\Vert v\Vert_{\max}$(§4.5)가 목표 투척 속도를 덮는지 확인 | `[HW-P1B]` |
| G3-G | IK 수렴률: 합성 후보 1000개에서 `max_iter` 내 수락 비율과 실패 시 잔차 분포 기록 (§4.2는 Gauss-Newton이 아니므로 수렴 보장이 없다) | `[SIM-ANY]` |
| G3-H | 오차 예산 모델 검증: L8에서 §4.6 예측 간극 분포와 실제 간극 분포 비교 (직교 분해 식이 맞는지) | `[SIM-P1B]` |
| G3-I | catchability 게이트 (D-18): $w_5$ 가 유한차분·해석 대조와 일치, threshold 미만 후보 탈락 + 사유 코드, 전부 탈락 시 plan 없음 (함수 자체의 판정은 **S1.9**). S3.5a/b 지도 도구와 S6 런타임이 같은 입력에서 같은 판정을 내는 동치성은 **S3.5a/b·S6** 에서 판정 | `[SIM-ANY]` |
| G3-J | D-7a 측정 (S6.5): 제어 PC 부하 상태에서 FIFO·OTHER 각각 수신 → plan 게시 지연 p50·p99·최대, 예산 초과율 (≥ 1000 시행) → plan §7.2 기준으로 정책 확정. 판정은 제어 PC 에서만 — dev PC 결과는 `NOT_EVALUATED(제어 PC)` (PREEMPT_RT 아님). 각 run 은 planner 스레드 이름에 맞는 모든 TID 의 실제 policy·priority·논리 CPU·cpuset mask 를 `verify_rt_runtime.sh` 로 기록하고, `{snapshot_sequence, recv_steady_ns, wake_ns, publish_ns}` 이벤트 레코드를 SPSC 로 남긴다 (plan §7.2) | `[SIM-ANY]` |
| G3-K | RT 할당 게이트: 계획기 스레드 한 사이클(탐색·IK·rollout·게시)이 `ScopedAllocGate`·`ScopedNoMalloc` 아래 할당 0, `noexcept`, 로깅 없음 (진단은 SPSC) | `[SIM-ANY]` |
| G3-L | token·race: eventfd coalescing, 계산 중 새 스냅샷 도착, 같은 generation 의 옛 plan 게시, deactivate·Pause race 에서 대체된 plan 소비 0 (D-22, D-23) | `[SIM-ANY]` |

## 10. 미확정 항목

TBD-HAND-01, TBD-HAND-04 (투척 보정만 — 기하값은 두 손 모두 S4.5 로 provisional 닫힘), TBD-BALL-02, TBD-VIS-04, `planner.ik.alpha_max` (provisional 0.26 — 값은 위 §6 에서 코드와 일치시켰고 닫는 근거만 남았다), `planner.freeze.T_freeze`, `planner.catchability.manipulability_min` (provisional, D-18), `planner.ik` 의 S1.9 provisional 기본값 (`sigma0`, `lambda_max`, `dq_step_max`, `k_manip`, `manip_grad_tol` — 값은 S3.5a 지도 실측으로 제안하고 사용자가 확정), 점수 가중치 (S6~S8), D-7a 정책 (S6.5), NLP 전환 여부 (§4.1, S6·S8 후). TBD-RTC-14~16 은 닫힘 (§2).

---

## 부록 A. [R1]식 SQP (`iiwa7_leap` 확장용, 선택)

**v0.5 위치 `[확정 A-4]`.** v1 은 1차원 시간 탐색 + IK (§4.1) 이고, 이 부록은 NLP 전환 시의 미래 선택지로 보존한다 (plan §8). 전환 시: §4.1 단일 진입 함수의 본문만 바꾸고, 두 구현이 생기는 그때 interface 를 도입하며 (ARCH-3), solver 가 할당·예외를 쓰면 D-7a 를 SCHED_OTHER 로 옮긴다. 재사용 후보는 `rtc_mpc` 의 solver 기반·스레드 관용구 (그 시점에 조사).

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
