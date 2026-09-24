# L5 — Joint Command: `ClikReferenceGenerator` 확장 옵션과 포구 컨트롤러 팔 명령 바인딩

- 문서 버전: v0.5 (2026-09-19)
- 브랜치: 단계별 `type/kebab-slug` (main 기준, 마스터 §4.2)
- 코드 배치 `[확정 D-1]`: CLIK 확장은 `rtc_tsid` (`rtc::tsid::ClikReferenceGenerator` 옵션), 컨트롤러 바인딩은 `integrated_bringup`. 새 패키지를 만들지 않는다
- 단계: **S2.2a·S2.2b** (CLIK 옵션, D-5·D-6 — S2.2a 구조 결정+golden-vector, S2.2b 옵션 구현) · **S5.3** (스트리밍 기준 → 확장 CLIK → 팔 명령, QP 비의존 관절공간 abort 경로) · ~~**S3.7** (지연 식별 도구)~~ — **2026-09-20 결정으로 제외, S10 에 흡수** · S10 (실기 $T_{arm}$ 식별)
- 선행: 단계 W, L0, L4
- 산출물: `ClikReferenceGenerator` 옵션(기본 off) + 회귀 테스트, 포구 컨트롤러의 팔 명령 경로, 지연 식별 도구(S10)

---

## 1. 범위 / 비범위

**v0.4 까지의 "얇은 어댑터" 전제는 폐기한다** `[확정 D-5]`. 기존 CLIK 은 pose(SE3) 목표·6행 고정·측정 q 평가·위치∩속도 box 만 지원하므로(W, plan §2) 포구 과제를 "등록" 할 자리가 없다. 이 layer 는 두 부분이다.

| 부분 | 내용 | 단계 |
|---|---|---|
| (a) CLIK 확장 | `rtc::tsid::ClikReferenceGenerator` 에 옵션 추가: twist feedforward, 접근축 2행(LOCAL x·y 각속도), 가속 box + `bound_conflict`, 직전 $\dot q$ 평활 항, status·반복 수·solve time 노출, `max_iter` 설정, q_c 평가 모드. **기본 off, off 시 기존 출력 bit-identical.** 기존 assertion 을 고쳐야 하면 즉시 E-6 | S2.2a → S2.2b |
| (b) 컨트롤러 바인딩 | 포구 컨트롤러가 L4 스트리밍 기준을 확장 CLIK 에 넣고, 결과 $q_c$ 를 `ControllerOutput` device 0 (팔) 에 position 으로 쓴다. CLIK 실패 시 QP 비의존 관절공간 abort 경로 | S5.3 |

확장 구조("행 집합 선택형 확장" vs "`QPSolverWrapper`·se3 오차만 공유하는 formulation 클래스")는 **S2.2a** 에서 확정한다 — 기존 동작 golden-vector 회귀(기록한 q 열 → q_ref 해시)도 그 구조 결정의 일부다. golden 테스트가 생기기 전에는 기존 위치∩속도 box 코드를 재구조화하지 않는다 (plan §7.3, §4.4 S2).

비범위:
- **토크 제어.** 명령은 전부 position (마스터 §1.3). sim·실기 모두 `CommandType::kPosition`.
- **명령 경로·드라이버 설정.** `DeviceBackend` 와 드라이버 소관. 본 구현은 감시만 한다(L7).
- kinematics·dynamics 구현(`PinocchioCache` 재사용), QP solver 구현(`QPSolverWrapper` 재사용).
- 손 관절 명령(L6 — device 1 slot).

## 2. 코드 확인 게이트

단계 W 에서 확인 완료 (2026-09-19, plan §2 · `WORKSPACE_ANALYSIS.md`).

| ID | 확인 항목 | 기록 |
|---|---|---|
| G5-1 | 명령을 싣는 자리의 형태 | 닫힘 — `RTControllerInterface::Compute` 가 돌려주는 `ControllerOutput` 의 `devices[0].commands` (팔 device 0 관례, rad, `CommandType::kPosition`). CM 이 같은 RT tick 안에서 `ValidateControllerOutput` → (실패·E-STOP 시 `BuildHoldOutput` 대체) → `DeviceBackend::WriteCommand` 로 보낸다. ros2_control `command_interface` 아님 (W4-1) |
| G5-2 | backend 가 이미 하는 일 | 닫힘 — 관절 위치 clamp(YAML `devices.<g>.joint_limits` ∩ URDF) 와 출력 검증·hold 만 있다. **지연 보상 없음**, 컨트롤러 쪽 외의 속도·변화량 제한 없음, speed scaling 노출 없음, `ApplySafetyLayer` production 호출 없음 (W4-2, W4-3) |
| G5-3 | CLIK 입력·출력·적분 상태 | 닫힘 — `ClikReferenceGenerator::Compute` 는 **측정 q** 에서 e·J 를 평가하고, 적분 anchor 는 `reseed_anchor` 로 측정/carry-forward 를 고른다 (`anchor_drift_max` clamp). 적분 상태 소유자는 CLIK. q_c 평가는 새 옵션 `[확정 D-6]` |
| G5-4 | 마스크 지원 | 닫힘 — CLIK 은 LWA 6행 고정, LOCAL 고정축 마스크 없음. `rtc_tsid` TaskBase 계열(SE3Task mask 는 LWA 행)은 acceleration-level 이라 쓰지 않는다 → **CLIK 에 접근축 2행 옵션 신설** (S2.2b) |
| G5-5 | QP solver 사용 방식 | 닫힘 — ProxQP dense box-QP over v, 차원 고정 (n_vars = nv, n_ineq = nv), `max_iter` 20 **하드코딩**, status 미노출 → `max_iter` 설정·status·반복 수·solve time 노출 옵션 (S2.2b) |
| G5-6 | frame Jacobian 함수·reference frame | 닫힘 — `PinocchioCache` 등록 frame 의 `J` 는 `LOCAL_WORLD_ALIGNED` 고정 (`RegisterFrame` 후 `Update`). LOCAL 이 필요하면 LWA 각속도 행을 $R_{WC}^\top$ 로 회전해 만든다 (§4.2) |
| G5-7 | 실기/sim 경로 전환, MuJoCo actuator | 닫힘 — 로봇 config YAML + launch 로 backend 선택 (`ur_driver_native` / `mujoco_native`), **둘 다 position**. MuJoCo UR 팔은 `<general>` position-PD (forcerange ±150/±28 N·m) (W4-2, W6-2) |
| G5-8 | `DemoWbcController` 알려진 버그 | 닫힘 — 본 경로에 영향 있는 미해결 버그 없음 (W2-8). DemoWbc 위치 백본이 이 CLIK 이므로 S2.4 회귀(기존 assertion 무수정)가 옵션 off 의 동등성을 지킨다 |

## 3. 참고자료

[R9] Pinocchio, [R10] ProxQP, [R11] UR ROS 2 드라이버(position 인터페이스, `servoj` 보간), [R7] 각속도 규약.

## 4. 수학적 이론

### 4.1 속도 수준 CLIK를 쓰는 이유

관절 명령이 전부 position이므로 QP 출력 $\dot q$를 한 번 적분해 $q_c$를 만든다. 가속도 QP 출력을 두 번 적분하면 드리프트와 솔버 잡음 증폭이 생기므로 쓰지 않는다(기존 프로젝트 결론과 동일).

이 구조는 `rtc::tsid::ClikReferenceGenerator` 로 이미 있다 (DemoWbc 위치 백본). 본 layer 는 그 클래스를 옵션으로 확장하고(S2.2b) 포구 컨트롤러에서 호출한다(S5.3). 적분은 CLIK 한 곳에서만 한다(§4.3).

### 4.2 과제 정의

**평가 지점 `[확정 D-6]`.** 포구 모드에서 CLIK 은 오차·Jacobian 을 **명령값 $q_c$** 에서 평가한다 (기존 기본값은 측정 q). 측정 q 평가는 servo 지연을 CLIK 루프에 품어 §4.5 선행 보상과 이중 보상이 된다. $q_c$ 평가는 `PinocchioCache` 를 $q_c$ 로 `Update` 해야 하므로, 측정 q 로 갱신하는 기존 캐시와 별개의 Data 가 필요한지는 S2.2b 에서 확정한다.

**병진 과제 (twist feedforward, D-5)**

$$J_p(q_c)\dot q=v_p^\ast,\qquad v_p^\ast=\dot x_{ref}+K_p\big(x_{ref}-x_C(q_c)\big)$$

$J_p$ 는 캐시 `J` 의 행 0..2 (`LOCAL_WORLD_ALIGNED`, 마스터 §3). $\dot x_{ref}$ feedforward 는 기존 CLIK 에 없는 항이다 (기존은 $r=K_x\odot e_x$ 뿐) — S2.2b 옵션.

**명령 공간 폐루프임을 명시한다.** $x_C$ 는 $q_c$ 의 FK 이지 측정 $q$ 의 FK 가 아니다. 따라서 $K_p(x_{ref}-x_C(q_c))$ 는 **적분 드리프트 보정 항일 뿐 외란 제거 항이 아니다.** 실제 운동은 $\dot x_{ref}$ feedforward 가 만든다. 결과로:

1. 실기 추종 오차는 이 루프 밖에 있다. 측정 $q$ 는 L7 `TRACK_ERR` 감시($\Vert q-q_c(t-T_{arm})\Vert$, 임계 `supervisor.track_err_abort`)에만 쓰인다 `[확정 D-6]`.
2. L3 §4.6 의 $\sigma_{trk}$ 는 **실행 중 관측되지 않는 오프라인 식별값**이다. $T_{arm}$ 모델의 정확도가 그대로 포구 오차로 간다.
3. §4.4 의 1차+지연 모델에서 1차 성분이 유의하면 선행 보상만으로 위상이 맞지 않고(§4.5), 그 잔차가 $\sigma_{trk}$ 의 지배 성분이 된다.

**재앵커 규칙 `[확정 D-6]`.** $q_c$ 를 측정 $q$ 로 다시 맞추는(re-anchor) 시점을 다음으로 한정한다. 그 밖의 tick 에서는 carry-forward 다.

| 시점 | 동작 |
|---|---|
| 컨트롤러 activate, arm·재무장 (L7 §4.8) | $q_c\leftarrow q_{meas}$, $\dot q_{prev}\leftarrow0$ — $\dot q_{prev}$ 를 남기면 첫 tick 가속 box 가 직전 시행 속도 기준이라 `bound_conflict` 오abort 가 난다 |
| E-STOP 해제 | $q_c$·CLIK anchor 를 $q_{meas}$ 로 reseed, **자동 재개 금지** — S9 이전 임시 기준 `[확정 P-1]` (S5 E-8 최소 계약, plan §4.4 S5.1: `TriggerEstop`·`ClearEstop`·`ResetFault`·`ResetTargetInitialization` 훅은 atomic 요청·epoch 만 갱신하고 reset 의 유일 writer 는 RT tick 이며, `ClearEstop` 은 컨트롤러 fault 를 풀지 않는다 — 두 경로는 별개). 전체 정책은 D-13 (S9) |
| 기존 CLIK 실패 후 첫 호출 | 기존 동작은 측정 q 로 강제 재앵커. 포구 모드에서는 abort 경로(§4.3)가 만든 $q_c$ 와 불연속이 되므로, 재앵커 대상을 옵션으로 둘지 S2.2b 에서 확정 |

기존 `anchor_drift_max` clamp 는 $\Vert q_c-q\Vert$ 를 조용히 잘라 `TRACK_ERR` 감시와 겹친다 — 포구 모드에서의 사용 여부는 S2.2b 에서 정한다.

**접근축 과제 (2행, S2.2b 신설).** catch frame 의 LOCAL $z$ 축이 손바닥 바깥 법선이다 (규약, D-17 · plan §10). roll 제거는 LOCAL 각속도의 $x,y$ 성분만 쓰는 선택 행렬 $S=\begin{bmatrix}1&0&0\\0&1&0\end{bmatrix}$ 로 표현한다. 캐시는 LWA 만 주므로(G5-6) 각속도 행(캐시 `J` 의 행 3..5)을 catch frame 으로 회전한다.

$$J_a=S\,R_{WC}^\top J_\omega^{LWA}(q_c)\in\mathbb R^{2\times n_v},\qquad r_a=S\,R_{WC}^\top K_a\,e_a$$

$e_a=\theta\hat u$ 는 회전벡터 오차다(L4 §4.5, `rtc_math` se3 의 축 정렬 오차, S2.1). $e_a\perp z_C$ 이므로 $R_{WC}^\top e_a$ 의 $z$ 성분이 0 이고 $S$ 가 정보를 버리지 않는다. L4 가 주는 각속도 기준의 feedforward 가 있으면 $r_a$ 에 같은 방식으로 더한다 (D-5).

기존 U1 포즈 오차 헬퍼(`ComputeTaskPoseError`)는 전체 SO(3) 오차라 roll 기준이 없는 이 과제에 쓰지 않는다.

동치 표현(W 기준): $J_a=f'(c)mm^\top+f(c)[a_d]_\times[z]_\times$ 를 $J_\omega^{W}$에 곱한다(L4 §4.5). $\theta\to\pi$ 에서 발산하므로 채택하지 않는다.

**자세(posture) 과제.** 6축에서 5-DoF 과제를 풀면 1자유도가 남는다. 기존 CLIK 의 팔 posture 항(L2, $v_{post}=K(q_{des}-q)$ on arm indices)을 그대로 쓴다. 손 posture 항(L3)은 CLIK 이 결합 모델 전체 $n_v$ 를 푸는 구조라 존재하지만, 손 device 명령은 L6 시퀀서가 쓰므로 CLIK 의 손 출력은 팔 명령에 쓰지 않는다.

### 4.3 QP

기존 CLIK 의 단일 가중 box-QP 에 옵션 항을 더한다 (결정변수 $v\in\mathbb R^{n_v}$, 결합 모델 전체 — §5.1).

$$\min_{v}\ \tfrac12 w_{task}\Vert J_pv-v_p^\ast\Vert^2+\tfrac12 w_a\Vert J_av-r_a\Vert^2+\tfrac12 w_{arm}\Vert S_{arm}v-v_{post}\Vert^2+\tfrac{\mu^2}2\Vert v\Vert^2+\tfrac{w_s}2\Vert v-\dot q_{prev}\Vert^2$$

$$\text{s.t.}\quad \ell\le v\le\upsilon$$

$$\ell_i=\max\Big(-\dot q_{\max,i},\ \frac{q_{\min,i}+m_q-q_{c,i}}{\Delta t},\ \dot q_{prev,i}-\ddot q_{\max,i}\Delta t\Big),\qquad \upsilon_i=\min\Big(\dot q_{\max,i},\ \frac{q_{\max,i}-m_q-q_{c,i}}{\Delta t},\ \dot q_{prev,i}+\ddot q_{\max,i}\Delta t\Big)$$

- 위치∩속도 항은 기존 box 그대로다 (기존은 스칼라 `v_limit`, $\beta$ 없음). 마진 $m_q$ 는 CLIK 에 넘기는 `q_min`/`q_max` 를 좁혀 구현하므로 새 옵션이 필요 없다 `[권장]`. v0.4 의 한계 접근 감속 $\beta$ 는 기존 box 에 없고 S2.2b 옵션 목록에도 없어 v1 에서 쓰지 않는다 ($\beta=1$ 과 동치).
- 가속 항과 평활 항($w_s$)은 S2.2b 옵션이다. $\Delta t$ 는 `ControllerState::dt` (= 1/`control_rate`).
- **가속 한계 $\ddot q_{\max}$ 의 출처 `[확정 D-16]`:** 토크 한계에서 오프라인으로 도출한 **상수 box** (plan §9, S2.5 도구 출력 YAML, provenance 포함). URDF 에는 가속 한계가 없다. 로봇 config 의 기존 `devices.<g>.joint_limits.max_acceleration` (5.0 rad/s²) 은 어떤 컨트롤러도 쓰지 않는 placeholder 라 **쓰지 않는다**. L3 도달시간 계산과 같은 값을 써야 계획이 실행과 일치한다.

$H$ 는 $\mu^2>0$ 이므로 양정치이고, 차원 $n_v$ 와 제약 수가 고정이다.

**경계 충돌 규칙 `[권장]`.** 관절 한계 근처에서 가속 한계 때문에 $\ell_i>\upsilon_i$ 가 될 수 있다. 이때 **가속 한계를 유지하고** 위치 한계 쪽으로 가장 가까운 값을 쓴다.

$$\ell_i=\upsilon_i=\mathrm{clamp}\Big(\mathrm{proj}_{[p_{lo,i},\,p_{hi,i}]}(\dot q_{prev,i}),\ \dot q_{prev,i}-\ddot q_{\max,i}\Delta t,\ \dot q_{prev,i}+\ddot q_{\max,i}\Delta t\Big)$$

그리고 `bound_conflict` 플래그를 올린다(L7 `ABORT_SAFE` 사유). 기존 box 의 "한계 위반 시 $\ell\leftarrow\min(\ell,\upsilon)$" 복구 규칙은 가속 옵션이 off 일 때만 적용된다.

위치 한계를 1틱 더 정확히 지키려고 가속 항을 빼면 $\dot q^\ast$ 가 한 틱에 $\pm\ddot q_{\max}\Delta t$ 를 넘어 점프한다. position 인터페이스에서 $q_c$ 기울기 불연속이고 UR 제어기가 보호 정지를 걸 수 있다. 이 상황은 어차피 abort 대상이므로 **abort 경로로 안전하게 빠져나가는 것**이 우선이다. 위치 한계 침범은 `limit_margin` 이 흡수한다.

**가속 제약의 세 형태 `[확정 결정 K, S6-C2]`.** 위 가속 box 는 `joint_cmd.accel_constraint` 의 한 형태 (`box`, 기본) 다. $\dot v\approx(v-\dot q_c)/\Delta t$ 로 두면 나머지 두 형태도 $v$ 에 **선형**이라 box 아래에 부등식 행으로 붙는다 ($C=[I;\,C_a]$). $\dot q_c$ 는 cache 가 평가된 속도 (명령값 평가 모드에서 명령 속도, D-6 — $h$·$\dot J$ 가 평가된 같은 상태) 의 **팔 성분**이다. 손은 다른 곳에서 명령되므로 그 가속은 이 solve 의 것이 아니다. 셋 중 하나만 고른다 — 다른 형태의 키를 함께 주면 YAML 파서가 거부하고 (`Init` 도 같은 규칙).

- `box` — 관절별 창 $\dot q_{prev}\pm\ddot q_{\max}\Delta t$ (D-16 상수 box). 행을 더하지 않아 기존 출력과 **비트 일치** (golden)
- `kinematic` — 비용이 추종하는 과제 행의 가속 $J\dot v+\dot J\dot q_c$ 를 행마다 $\pm$`task_accel_max_linear` (위치 행) · `task_accel_max_angular` (회전·접근축 행). $\dot J\dot q_c$ 는 등록 frame 의 classical drift (`dJv`), 접근축 행은 LOCAL x, y 에서 ($\tfrac{d}{dt}(R^\top\omega)=R^\top\dot\omega$). **그 행만** 묶는다 — 영공간 운동 (5행 오버로드의 접근축 roll 등) 은 속도 box 외에 가속 한계가 없어 한 tick 에 속도 한계까지 뛸 수 있다 (2026-09-23 `/code-review`). 관절마다 묶는 것은 `dynamic` 이다
- `dynamic` — 팔 관절 토크 $M_{arm}(q)\dot v_{arm}+h(q,\dot q_c)$ 를 $\pm\eta_\tau\tau_{\max}$. $M,h$ 는 cache 의 값 (명령값 평가 모드에서 $q_c,\dot q_c$, D-6), $\tau_{\max}$ 는 팔 device 의 `joint_limits.max_torque` (D-16 도출이 쓴 같은 수), $\eta_\tau$ 기본 0.8 (D-16). **URDF 에 회전자 관성이 없어 $M$ 에 빠져 있다** — $\eta_\tau<1$ 이 그것을 덮는다고 가정한다 (D-16 도출은 MuJoCo `mj_inverse` 로 armature 포함 교차 검증했다). 실기 전 (S10) 재확인

행은 **단위 norm** 으로 스케일한다 — 가능 집합은 그대로이고, $M/\Delta t$·$J/\Delta t$ 행이 box 행보다 $10^2$–$10^5$ 배 커서 ProxQP 가 가능한 문제에 PRIMAL_INFEASIBLE 을 냈던 것을 막는다. 행이 있으면 실패한 solve 와 `ResetAnchor()` 뒤에 warm start 를 버린다 (한 번 실패한 dual 에서 시작하면 infeasible 이 이어졌다). 행은 hard 다. 속도∩위치 box 와 동시에 만족할 수 없으면 (예: 한계 근처에서 중력 토크를 못 버티는 경우) box 의 관절별 충돌 규칙 같은 해소가 없다 — 행이 관절을 결합하기 때문이다. 그때 호출은 **실패**하고 (`LastSolve().accel_rows_violated` + `converged` false — status 는 SOLVED 일 수 있다, 또는 수렴 실패 status) 아래 QP 비의존 abort 경로로 간다. 행을 깨는 명령을 돌려주지 않는다. 진단: `accel_rows` (조립한 행 수) · `accel_rows_binding` (해가 경계에 닿은 행 수). abort 경로의 감속은 형태와 무관하게 D-16 box 를 쓴다.

결정 K 의 근거 (#537 코멘트 5785720197): D-16 상수 box 는 포구 자세에서 토크가 허락하는 가속보다 약 50 배 (p1b) 보수적이라 γ 창을 닫는다 (plan §9 S4.4 판정). `dynamic` 은 자세 의존 $M,h$ 로 그 보수성을 실행층에서 없애고, L3 도달시간 게이트의 **토크 층 런타임 대응물**을 겸한다 (결정 F) — 계획기의 도달시간 순위 항은 여전히 box 층이다.

**출하 형태 (2026-09-23 사용자 결정, #537 코멘트 5789708503).** `ur5e_p1b` 는 `dynamic` ($\eta_\tau$ 0.8). 근거는 투척마다 팔을 `planner.wait_pose` 에 정렬한 뒤 잰 sim A/B (공 25회씩): APPROACH 중 $\Vert FK(q_c)-x_{ref}\Vert$ p50 / p95 가 `dynamic` 3.2 / 68 mm, `box` 163 / 624 mm 이다. `iiwa7_leap` 은 측정이 없어 파서 기본값 `box` 로 둔다.

**반복 상한과 상태 노출 (S2.2b).** 차원만 고정하면 최악 실행시간이 묶이지 않는다. 기존 하드코딩 `max_iter` 20 을 설정 가능하게 하고, solver status·반복 수·solve time 을 노출한다. 초과·수렴 실패·비유한 결과는 `Compute` 가 false 를 돌려주는 기존 경로로 합쳐진다.

**실패 경로 — QP 비의존 관절공간 abort (S5.3).** 기존 CLIK 은 실패 시 `q_ref = q_meas`, `v_ref = 0`, false 를 돌려준다. 포구 컨트롤러는 **이 출력을 소비하지 않는다** — $q_{meas}$ 로 점프하면 $q_c$ 불연속이고 $v=0$ 은 가속 한계를 무시한 즉시 정지다. v0.4 의 $\dot q^\ast=\beta_{qp}\dot q_{prev}$ 감쇠 폴백은 QP 없이 동작하는 관절공간 경로로 흡수한다: 직전 $\dot q_{prev}$ 에서 관절별로 $\ddot q_{\max}\Delta t$ 씩 0 으로 감속하며 $q_c$ 를 적분하고(위치 한계 clamp 포함), L7 `QP_FAILED` → `ABORT_SAFE` 로 넘긴다. 감속 법칙의 세부는 S5.3 에서 확정한다. 연속 `QP_FAILED` 횟수가 **L7 소유의 단일 키** `supervisor.n_qp` (L7 §4.1) 에 이르면 L7 FAULT 래치 (`HasLatchedFault`) — L5 는 이 카운터를 새로 두지 않고 L7 판정을 참조만 한다.

**RT tick 안 try/catch 금지 (RT-2).** v0.4 가 제안한 `try { … } catch (...)` 는 쓰지 않는다. `ClikReferenceGenerator::Compute` 는 이미 noexcept 이고 모든 할당·검증(throw)은 non-RT `Init` 에 있다. 예외 주입은 non-RT 테스트 빌드에서만 하며, RT 경로의 방어는 입력 검증(비유한·차원)과 false 반환으로 한다.

**적분과 출력.** $q_c\leftarrow q_c+\dot q^\ast\Delta t$ 는 CLIK 의 carry-forward anchor 가 한다(`QRef()`). 바인딩은 다시 적분하지 않는다 — 두 곳에서 적분하면 $q_c$ 가 이원화된다. 바인딩은 `QRef()` 의 팔 성분을 `devices[0].commands` 에 쓴다(G5-1).

**중복 방지.** backend 는 관절 위치 clamp 만 한다(G5-2). CLIK 에 넘기는 위치 한계는 backend 가 쓰는 YAML ∩ URDF 한계보다 `limit_margin` 만큼 안쪽이어야 backend clamp 가 발동하지 않는다. 발동하면 QP 해와 실제 명령이 달라지고 그 차이가 L7 `TRACK_ERR` 로 나타난다(G5-C2 가 감시).

### 4.4 `servoj` 지연 모델과 식별 `[논문 외 설계]`

**backend 에 지연 보상이 없음이 확인됐다 (G5-2, W4-2).** 따라서 이 절과 §4.5 는 필요하다. 실기 식별은 S10 `[HW-P1B]` 에서 한다.

`servoj` 는 현재 상태와 목표 사이를 보간하므로 명령 대비 실제 관절에 지연이 생긴다([R11]). 두 가지 모델을 둔다.

- 순수 지연: $q(t)\approx q_c(t-T_{arm})$
- 1차 + 지연: $G(s)=e^{-\tau s}/(T_fs+1)$. 포구 대역 주파수 $f$에서의 등가 지연은 $T_{eq}(f)=\big(2\pi f\tau+\arctan(2\pi fT_f)\big)/(2\pi f)$.

식별 절차(관절별):
1. 운용과 **같은 드라이버 파라미터**로 여기(excitation) 궤적을 명령한다. 권장은 multisine(포구 운동 대역 포함)과 실제 포구형 궤적 두 종류다.
2. $\dot q_c$와 $\dot q$의 정규화 상호상관 최대 위치로 $\hat\tau$를 초기 추정한다.
3. 1차 + 지연 모델을 최소제곱으로 맞추고, 포구 대역의 $T_{eq}$를 계산한다.
4. $T_{arm}=\max_iT_{eq,i}$ (보수적)와 관절별 값을 모두 기록한다.

> **2026-09-20 결정 — sim 은 지연이 없다고 보고 구현한다 (사용자 결정, plan §4.4 S3a 각주).**
> 종전의 "sim 에서는 주입 지연 회복 테스트(G5-D)로 도구를 검증한다" 는 **삭제한다** — 주입도 에뮬레이션도 하지 않고 `arm_lag` (또는 `sim.arm_lag`) 파라미터를 신설하지 않는다. 이 절의 식별 절차는 **실기 전용** (§7 L5.9, S10 `[HW-P1B]`) 이 되고 `G5-D` 는 은퇴한다 (§9).
>
> **정정 (2026-09-24, S8 준비).** 위 결정은 *에뮬레이션 지연을 넣지 않는다* 는 뜻이지 sim 에 지연이 없다는 뜻이 아니다. sim 팔의 MJCF actuator 는 §4.6 대로 `<general>` PD 이고 두 로봇 모두 kd/kp = 0.2 s 라 (p1b `size3` 2000/400 · `size1` 500/100, iiwa `proximal` 3000/600 · `distal` 750/150) 느린 극이 kp/kd = 5 rad/s 로 관성과 무관하게 고정된다 (kd² ≫ 4·I·kp) — **시정수 약 200 ms 의 1차 지연**이다 (램프 정상 지연 199 ms, step t63 200 ms). S6 의 t_c 서보 성분 ~125 mm (≈ 0.6 m/s × 0.2 s) 와 S7 포획 0/25 (공이 t_c 약 40 ms 전 도달) 의 원인이다. 그러므로 이 절의 식별 절차는 sim 에도 적용할 수 있다 (`catching_diag.csv` 의 `q_cmd_*`·`q_meas_*`) — 파일럿 (`260924_1218`) 의 1차 지연 **최소제곱 적합이 6 관절 모두 τ̂ 199–204 ms** 다. ⚠️ 위 절차 2 의 속도 상호상관은 **1차 지연에서 τ 를 주지 않는다** — 같은 데이터의 xcorr 피크는 70–98 ms 였다 (피크는 지연이 아니라 저역통과의 위상·대역에 걸린다). xcorr 는 순수 지연 성분의 초기값으로만 쓰고 τ̂ 는 LS 적합 (부트스트랩 CI·R² 병기) 으로 낸다. 보상은 **D-S8-1 (a) 확정 (2026-09-24)** — S8-B 에서 sim overlay 로 선행을 켠다 (§6 `joint_cmd.lag.*` 행, plan §4.4 S8).
>
> **개정 (2026-09-24, D-S8-13).** `ur5e_p1b` sim 은 팔 서보 게인을 `config/ur5e_p1b/mujoco_simulator.yaml` 에서 준다 (`use_yaml_servo_gains: true`, kp ×4 = 8000/2000, kd 400/100 그대로) — kd/kp = **0.05 s**, LS τ̂ 50.2–50.9 ms. τ 0.2 는 T_freeze 0.52 를 요구해 S3.5b 목표 분포를 닫았고, 0.05 는 설계가 유도된 T_arm 과 같다. 이것은 sim 의 선택이지 UR5e 의 값이 아니다 (S10 이 식별한다). `iiwa7_leap` 은 MJCF 게인 그대로 (τ 0.2).

### 4.5 지연 보상: 예측 선행 `[논문 외 설계]`

순수 지연이 지배적이면 명령 궤적을 $T_{arm}$ 만큼 앞당긴다. 시간 비교는 plan §3 규약을 따른다 `[확정 D-2]`.

1. 궤적 샘플링·γ 프로파일·기준 생성은 $\text{now\_lead}=\text{now}+T_{arm}$ 에서 한다 (L2 §4.4 의 $T_{lead}=T_{arm}$). vision 예측을 앞당겨 읽는 것이지 제어 PC 가 전파하는 것이 아니다. now 는 매 tick steady 실측이며 tick 수 × dt 로 계산하지 않는다.
2. CLOSING→DECEL 전환도 now_lead ≥ $t_c$ 로 판정한다 (L7). 즉 명령 경로가 $t_c-T_{arm}$ 에 포구점에 도달한다.

손 명령 시각 $t_{cmd}$ 와 Preshape 는 **실제 시각(now)** 축이므로 이 선행을 적용하지 않는다 (plan §3, L6).

1차 필터 성분이 크면 선행만으로는 위상이 완전히 맞지 않는다. ~~시뮬레이션 에뮬레이션(§4.6)으로 잔여 오차를 측정해 판정한다~~ — **sim 지연 0 결정 (2026-09-20) 으로 그 판정 수단이 사라졌다.** 잔여 오차 판정은 실기 (S10) 로 간다. **정정 (2026-09-24)**: sim actuator 자체가 순수 지연이 아닌 1차 지연 (MJCF 게인 τ ≈ 200 ms, `ur5e_p1b` sim 은 D-S8-13 으로 50 ms — §4.4 정정) 이므로 sim 에서 선행 보상의 잔여 (비-순수지연분) 를 **런타임에** 잴 수 있다 — S8-B 의 lead on/off 비교가 그것이고 (L8 §9.1 G8-E), 그 수치는 sim 1차 플랜트 (p1b 50 ms) 의 값이지 `servoj` 의 값이 아니다 (§4.6).

**$T_{arm}\neq0$ fixture 는 여전히 필수다** — $T_{arm}=0$ 이면 now 와 now_lead 가 같아져 축 혼동 버그가 숨는다. 이 요구는 sim **런타임** 지연과 무관하며, sim 지연 0 결정은 이것을 면제하지 않는다. (그래서 이 fixture 가 §9 G5-E·L8 G8-E 를 되살리는 가장 싼 경로이기도 하다 — S5 착수 시 결정. 2026-09-24 부터 G8-E 는 sim 런타임 lead on/off 로 판정하고 fixture 는 G5-E 에 남는다.)

### 4.6 시뮬레이션 동등성

MuJoCo UR 팔 actuator(`<general>` position-PD)는 `servoj` 와 동특성이 다르다. ~~`ur5e_p1b` 시뮬레이션에는 식별된 $G(s)$ 를 명령 경로에 넣는 에뮬레이션 옵션을 둔다(`sim.arm_lag.*`, `[SIM]` 전용)~~ — **2026-09-20 결정으로 두지 않는다 — 에뮬레이션 지연 0** (plan §4.4 S3a 각주). `iiwa7_leap` 도 같다. 단 actuator 자체가 kd/kp 의 1차 지연이라 sim 팔은 지연 0 이 아니다 (§4.4 정정 2026-09-24) — MJCF 게인은 0.2 s, `ur5e_p1b` sim 은 YAML 게인으로 0.05 s (D-S8-13).

남는 사실은 **동특성이 다르다는 것 자체**다: sim 의 position-PD 응답은 `servoj` 가 아니므로 sim 에서 잰 추종 오차를 실기 예측값으로 쓰면 안 된다. 호스트가 될 명령 경로는 `MuJoCoSimulator::ApplyCommand()` 이고, 나중에 지연 주입이 필요해지면 **출하 YAML 파라미터가 아니라 테스트 fixture 전용**으로 넣는다 (`rtc_controllers` 의 `sim.ball.*` 이 그 선례다).

### 4.7 Sanity check

1. 정지 목표: 과제 오차가 지수적으로 감소하고 roll 은 posture 과제로 정해진다.
2. $R_{WC}^\top J_\omega^{LWA}$ 로 만든 접근축 행과 유한차분 FK 일치.
3. 관절 한계 접근 시 위반 0.
4. ~~알려진 지연을 주입한 시뮬레이션에서 식별값이 주입값을 회복~~ — **삭제 (2026-09-20, sim 지연 0).** 식별 도구의 검증은 실기 여기 궤적 (S10) 에서만 한다.
5. 옵션 전부 off 에서 기존 CLIK 출력과 bit-identical (S2.4 회귀).

## 5. C++ 구현

### 5.1 CLIK 확장 옵션 (S2.2a·S2.2b, `rtc_tsid`)

v0.4 의 `CatchTaskAdapter` (`configure`/`update()`, 자체 `VecN`) 스케치는 폐기한다.

**구조 결정 (S2.2a — 행 선택형 확장, 2026-09-19 사용자 결정).** `ClikReferenceGenerator` 한 클래스에 옵션을 더한다. formulation 클래스 (`QPSolverWrapper`·se3 오차만 공유) 는 택하지 않는다. box 조립·위치 한계 collapse·anchor 적분·실패 처리를 두 벌로 유지해야 하고, 구현이 하나뿐이라 분리 이득이 없다 (P5, ARCH-3). rtc_controllers 의 DLS task-velocity 법칙 (`task_vel_core`, DemoTask·DemoCompliance) 도 후보가 아니다 — box 제약이 없고, 가속 box·위치 한계를 QP 로 푸는 것이 이 확장의 목적이다.

| 항목 | 결정 |
|---|---|
| 과제 입력 | 기존 `Compute(…, const pinocchio::SE3&, …)` 는 그대로 둔다. 포구용으로 위치 3행 + 접근축 2행을 받는 **`Compute` 오버로드**를 더한다. 목표 구조체는 base frame 의 위치·접근축 (단위)·선속도·각속도 feedforward |
| 공유 코드 | box 조립, solve, anchor 적분·실패 분기를 private helper 로 뽑는다. 이 추출은 **golden-vector 회귀 (`rtc_tsid/test/test_clik_golden.cpp`) 가 비트 일치로 통과하는 별도 커밋**으로 한다 |
| 좌표 | 새 오버로드는 오차·Jacobian 을 모두 world 정렬 (LWA) 로 쓴다. base frame 은 목표를 world 로 옮기는 데만 쓴다. 기존 SE3 경로는 base 정렬 오차와 world 정렬 `rf.J` 를 곱한다 — base 가 world 에 대해 회전하면 어긋나지만 현 로봇 구성 (root = universe 정렬) 에서는 드러나지 않는다. 기존 경로는 golden 이 고정하므로 고치지 않고 기록만 한다 |
| 접근축 행 | $J_a=S\,R_{WC}^\top J_\omega^{LWA}$, $r_a=S\,R_{WC}^\top(K_a e_a+\omega_{ff})$, $e_a$ = `rtc::math::se3::AxisAlignError` (S2.1). 무효·반평행 데드밴드 분기는 결과의 region 으로 호출자에게 노출한다 |
| 옵션 (전부 기본 off) | 관절별 속도 한계 (비면 기존 스칼라 `v_limit`), 가속 box ($\ddot q_{\max}$ 벡터, $\dot q_{prev}$ = 직전 `v_ref`) + `bound_conflict`·관절 mask, 평활 가중 $w_s$, `max_iter` (기본 20 = 기존값), 기존 SE3 경로의 twist feedforward (없으면 기존 식), 명령값 평가 모드 |
| 명령값 평가 모드 (D-6) | CLIK 안에 cache 를 두지 않는다. 호출자가 $q_c$ (= 직전 `QRef()`) 로 갱신한 cache 를 넘기고, CLIK 은 (a) 매 tick `cache.q` (= $q_c$) 에 anchor 를 두고 (`reseed_anchor` 무시), (b) 첫 성공 뒤로는 (실패한 호출이 있어도 `ResetAnchor()` 전까지) **팔 성분**의 `cache.q` 가 직전 `QRef()` 와 다를 때 false + `command_mismatch` (배선 오류 검출. 손 성분은 L6 가 명령하므로 검사하지 않는다), (c) `anchor_drift_max` 를 쓰지 않는다 (Init 이 거부) (측정 q 가 없다. 실추종 감시는 L7 `TRACK_ERR`) |
| 재앵커 | 실패한 호출은 출력이 $\dot q=0$ 이므로 $\dot q_{prev}$ 도 0 으로 둔다. `ResetAnchor()` 가 anchor 와 $\dot q_{prev}$ 를 함께 초기화한다 (activate·재무장·E-STOP 해제, §4.2 표). 실패 후 측정 q 재앵커는 기존 동작 유지가 기본이고, 명령값 모드에서는 cache 가 이미 $q_c$ 라 연속이다 |
| 진단 | `LastSolve()`: ProxQP status (`SolveResult` 에 raw status 추가), 반복 수, solve time, `bound_conflict`, 충돌 관절 mask |
| box 허용오차 | QP 해는 ProxQP `eps_abs` (1e-6) 안에서만 box 를 지킨다 (실측: `v_limit` 초과 최대 8e-7). G5-B 의 "위반 0" 은 이 허용오차 안을 뜻한다 |

S2.2b 커밋 순서: helper 추출 (golden 비트 일치) → 진단 노출 → `max_iter` → 관절별 속도 한계 → 가속 box + `bound_conflict` → 평활 항 → twist feedforward → 위치 + 접근축 오버로드 (S2.1 머지 후) → 명령값 평가 모드. 커밋마다 golden 비트 일치를 확인한다.

요구 사항:

- 모든 옵션 기본 off, off 시 기존 `Compute` 출력 bit-identical. 할당은 `Init` 에서만, `Compute` 는 noexcept·할당 0
- 추가 옵션: twist feedforward 입력, 접근축 2행 (catch frame 등록 index + $R_{WC}$ 회전), 가속 box ($\ddot q_{\max}$ 벡터, $\dot q_{prev}$) + `bound_conflict`, 평활 가중 $w_s$, `max_iter`, q_c 평가 모드와 재앵커 대상(§4.2)
- 노출: solver status, 반복 수, solve time, `bound_conflict`, 활성 관절 mask
- 차원: 결정변수는 **결합 모델 전체 $n_v$**. ur5e_p1b 결합 모델은 full tree nv **26** (UR5e 6 + P1b 20 revolute), actuated 축약 모델 nv **16** (팔 6 + 손 10) — 이전 기록의 22 는 근거가 없다(plan §2). CLIK 의 control model 은 actuated 모델이 있으면 그것(ur5e_p1b: nv 16)이고, 없을 때만 tree/full 로 fallback 한다 (DemoWbc `ConfigureReducedDynamicsProvider` 의 게이트와 같은 인스턴스). iiwa7_leap 값은 S2.3a 의 nv 고정 테스트에서 기록한다. 팔 열은 `Config::arm_v_idx` 로 고른다. v0.4 의 "최대 7 `VecN`" 가정은 틀렸다
- 기존 `Manipulability()` (팔 6×6 damped √det) 는 진단값으로 유지. D-18 의 5행 $w_5$ 와 다른 값이므로 로그에 둘 다 남긴다 (plan §11)

### 5.2 경계 계산 (RT)

§4.3 의 가속 box·충돌 규칙은 CLIK 옵션 내부에서 계산한다. v0.4 의 `jointVelocityBounds` 참조 코드는 삭제한다 — S2.2b 에서 `ClikReferenceGenerator` 의 box 조립에 넣고, 가속 옵션 off 에서는 기존 box 조립(한계 위반 시 collapse 복구 포함)을 그대로 둔다. 충돌 규칙 단위 테스트(G5-B2)는 S2.2b 에서 작성한다.

### 5.3 컨트롤러 바인딩 (S5.3, `integrated_bringup`)

포구 컨트롤러는 `RTControllerInterface` 를 상속한 코어(rtc_controllers `catching`) + 바인딩(`integrated_bringup`) 2층이다 (agent_docs/modification-guide.md "Adding a New Controller", L8). 팔 명령 경로의 tick 흐름 (`Compute(const ControllerState&) noexcept`):

1. `PinocchioCache::Update` (측정 q; q_c 평가 캐시는 §4.2)
2. plan `rtc::SeqLock` 스냅샷 읽기 → L2 샘플링(now_lead) → L4 기준 (x_ref, ẋ_ref, e_a, feedforward)
3. 확장 CLIK `Compute` (catch frame index, base frame index, dt = `ControllerState::dt`)
4. 성공: `QRef()` 팔 성분 → `devices[0].commands`, `command_type = kPosition`. 실패: §4.3 관절공간 abort 경로 + L7 `QP_FAILED`
5. L6 시퀀서 결과 → `devices[1]` (손)
6. 진단(status·반복·solve time·`bound_conflict`)은 SPSC 로 aux drain (RT-1 — tick 에서 로깅 금지)

catch frame 은 모델 빌더가 YAML 선언으로 추가한 frame 이다 `[확정 D-10, D-17]` — 컨트롤러는 frame 이름만 참조하고 `RegisterFrame` 으로 index 를 얻는다. 파라미터는 `LoadConfig(YAML)` + `ParseXxxParams`, runtime gain 은 `declare_parameter` (L8).

### 5.4 지연 식별 도구 (non-RT, Python, S10)

```python
# identify_arm_lag (요지 — 위치는 S10 에서 정한다)
def xcorr_delay(qd_cmd, qd_meas, dt, max_lag_s):
    """정규화 상호상관 최대 지연 [s] (관절별)."""
    ...
def fit_fopdt(t, q_cmd, q_meas):
    """G(s) = exp(-tau s)/(T s + 1) 최소제곱 적합 → (tau, T)."""
    ...
def equivalent_delay(tau, T, f):
    return (2*np.pi*f*tau + np.arctan(2*np.pi*f*T)) / (2*np.pi*f)
```

입력: 컨트롤러 CSV 로그(명령 $q_c$ = `devices[0].commands`, 측정 $q$, 공통 steady 시각). 출력: 관절별 $(\tau,T_f,T_{eq})$ YAML 조각과 적합 잔차 그래프.

## 6. YAML 파라미터

키 이름은 S5.1 에서 컨트롤러 YAML 스키마를 확정할 때 맞춘다. 같은 값은 한 키만 둔다.

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `robot.arm.joint_names` | string[] | – | 로봇 config | – | `devices.<arm>.joint_state_names` 를 참조 (사본 금지) |
| `catch_frame` | string | – | `catch_frame` | – | 모델 빌더 추가 frame 이름 `[확정 D-17]` (plan §10) |
| `robot.arm.q_min`, `q_max` | double[n] | rad | 로봇 config | – | YAML `joint_limits` ∩ URDF (backend 와 같은 원천, 사본 금지) |
| `robot.arm.qd_max` | double[n] | rad/s | 로봇 config `max_velocity` | ≤ 데이터시트 | 기존 CLIK 은 스칼라 `v_limit` — 관절별 적용은 S2.2b 에서 확인 |
| `robot.arm.qdd_max` | double[n] | rad/s² | S2.5 도출값 | >0 | `[확정 D-16]` 토크 한계에서 도출한 상수 box, provenance 포함. `max_acceleration` placeholder 사용 금지 |
| `robot.arm.limit_margin` | double | rad | 0.05 | 0–0.3 | CLIK 에 넘기는 위치 box 를 좁힘 (§4.3) |
| `robot.arm.q_nominal` | double[n] | rad | `TBD` | – | posture 과제 (wait_pose 와 관계는 L7) |
| `joint_cmd.K_p` | double | 1/s | 20.0 | 1–100 | CLIK 대역 (L4 `k_axis`보다 크게) |
| `joint_cmd.K_a` | double | 1/s | `TBD` | >0 | 접근축 게인 |
| `joint_cmd.w_task`, `w_a`, `w_arm` | double | – | 1.0, 0.5, 1e-2 | >0 | 기존 CLIK 가중 체계 (`w_task ≫ w_arm ≫ μ²`) |
| `joint_cmd.damping_sq` | double | – | 1e-4 | >0 | 기존 CLIK μ² |
| `joint_cmd.w_smooth` | double | – | 1e-3 | ≥0 | 평활 항 $w_s$ (S2.2b 옵션) |
| `joint_cmd.qp.max_iter` | int | – | 20 | >0 | 기존 하드코딩값을 기본으로, S2.2b 에서 설정화 |
| `joint_cmd.accel_constraint` | string | – | `box` | `box`·`kinematic`·`dynamic` | 결정 K (S6-C2, §4.3). 셋 중 하나 — 선택하지 않은 형태의 키가 있으면 파서가 거부한다 |
| `joint_cmd.task_accel_max_linear` | double | m/s² | `TBD` | 1e-3–500 | `kinematic` 전용 — 위치 행 가속 한계 |
| `joint_cmd.task_accel_max_angular` | double | rad/s² | `TBD` | 1e-3–1000 | `kinematic` 전용 — 회전·접근축 행 가속 한계 |
| `joint_cmd.eta_tau` | double | – | 0.8 | (0, 1] | `dynamic` 전용 — D-16 의 $\eta_\tau$. $\tau_{\max}$ 는 팔 device `joint_limits.max_torque` (사본 금지) |
| `joint_cmd.K_n` | double | 1/s | 1.0 | 0–10 | posture (기존 `SetPostureGains`) |
| `joint_cmd.lag.T_arm` | double | s | **0.0** (sim, S5.3) | 0–0.5 | §4.4 식별 (S10). **출하 sim 값 0** — 에뮬레이션 지연을 주입하지 않는다 (2026-09-20); sim actuator 의 고유 지연 (`ur5e_p1b` 0.05 s — YAML 서보 게인, D-S8-13) 은 **S8-B overlay 에서만** 보상한다 (D-S8-1 (a)) — overlay 는 `T_arm` 0.05 와 함께 `planner.freeze.T_freeze` 0.37 을 넣는다 (L3 §4.11 하한 T_close,tot + T_arm + margin = 0.3615 — 검증기 `CheckFreezeCoversClose` 는 margin 없이 0.3325 만 보므로 출하 0.36 도 활성은 되지만 설계 하한에 못 미친다). `io.horizon_min` 0.51 · `n_min` 12 는 출하값이 이미 T_arm 0.05 로 유도됐다. ⚠️ `lead_enable` 과 무관하게 검증기는 이 값을 T_freeze 하한에 넣는다. 0 이 아닌 값은 §4.5 의 축 혼동 fixture 와 G5-E 지연 fixture 에서만 쓴다. `lead_enable` 이 false 면 읽히지 않는다 (선행축 = 실제축) |
| `joint_cmd.lag.per_joint` | double[n] | s | `TBD` | ≥0 | §4.4 |
| `joint_cmd.lag.lead_enable` | bool | – | false | – | 식별 전에는 false. S8-B sim overlay 에서 lead on arm 만 true (off arm 은 같은 T_arm·T_freeze 로 false — 선행만 다르다). overlay 는 `integrated_bringup/config/ur5e_p1b/sim_overlays/catch_lead_{on,off}{,_gamma0}.yaml` 이고, 키 경로가 출하 YAML 에 있는지는 `test_catch_lead_overlays.py` 가 고정한다 (한 단계 얕은 경로는 경고 없이 출하값으로 돈다) |
| `supervisor.track_err_abort` | double | rad | `TBD` | >0 | **L7 §6 단일 원천.** L5 는 참조만 한다 |

v0.4 의 `joint_cmd.qp.beta_fallback` 은 삭제한다 (실패 경로는 §4.3 관절공간 abort). `robot.arm.limit_beta` 도 삭제한다 (§4.3).

## 7. 단위 기술 구현 순서

- **L5.0** 단계 W 결과 반영 — 완료 (§2).
- **L5.1** (S2.2a) 확장 구조 설계 리뷰 + 기존 동작 golden-vector 회귀(기록한 q 열 → q_ref 해시) → 옵션 추가는 S2.2b. 옵션 off bit-identical 회귀 테스트를 먼저 쓴다.
- **L5.2** (S2.2b) 가속 box + 충돌 규칙 테스트.
- **L5.3** (S2.2b) 접근축 2행, twist feedforward, q_c 평가 모드와 재앵커.
- **L5.4** (S2.2b) 정지 목표 수렴 테스트, 유한차분 Jacobian 테스트.
- **L5.5** (S2.2b·S2.4) RT 검사: QP 차원 고정, 할당 0, solve time 분포, DemoWbc 회귀.
- **L5.6** (S5.3) 컨트롤러 바인딩: `devices[0]` 쓰기, 관절공간 abort 경로. **완료 2026-09-22** — 구현에서 정해진 것 넷: (1) **손은 solve 안에서 잠근다** (속도 box 를 1e-9 로). catch frame 이 손바닥에 달려 있어 손 관절이 frame Jacobian 에 들어오는데 이 컨트롤러는 손을 명령하지 않는다 (L6 가 한다) — 풀어 두면 QP 가 일어나지 않을 운동으로 과제의 일부를 만족시키고 팔이 그만큼 덜 간다. (2) **posture 목표는 시행 시작 자세** (`robot.arm.q_nominal` 은 TBD 이고 wait_pose 는 S7 소유). (3) **가속 box 는 팔만 derived 값**이고 손 항은 잠금 속도/dt 로 파생한다 (CLIK 은 전 nv 를 요구한다). (4) QP 실패 streak 은 **성공한 solve 만** 지운다 — abort→재시도 사이클에는 solve 가 없으므로 seed 에서 지우면 래치가 영원히 안 선다
- **L5.7** ~~(S3.7)~~ → **S10 으로 이동 (2026-09-20)**, L5.9 에 흡수한다. 지연 식별 도구: 순수 지연 + 시상수 분리 식별. ~~σ_trk sim 초기값 산출~~·~~시뮬레이션 주입 지연 회복 테스트~~ 는 **sim 지연 0 결정으로 소멸** — `planner.budget.sigma_trk` (L3 §6) 는 sim 초기값 출처를 잃고 S10 까지 TBD 로 남는다.
- **L5.8** (S5.3) 예측 선행 보상(now_lead, L2 연동). ~~+ 에뮬레이션에서 효과 측정~~ — **substrate 를 잃었다** (§4.6). 보상 자체는 그대로 구현하고 효과 측정 (G5-E) 은 **fixture 전용 지연 주입** 위에서 한다 (2026-09-22 사용자 확정, §9 G5-E). sim 런타임 효과 (G8-E) 는 S8-B 에서 — sim actuator 가 1차 지연이라 substrate 가 있었다 (§4.4 정정, D-S8-1).
- **L5.9** 실기 식별 `[HW-P1B]` (S10).
- **L5.10** (S5.3) backend 왕복 확인: `ControllerOutput` 에 쓴 $q_c$ 와 backend 가 실제로 쓴 값 1:1 대조.

## 8. 디버깅 방법

- 기록: $q_c$, $q$, $\dot q^\ast$, 경계 $(\ell,\upsilon)$, 활성 관절 인덱스, 과제 잔차 $\Vert J_p\dot q-v_p^\ast\Vert$, 축 잔차, solve time, solver status·반복 수, `bound_conflict`.
- 과제 잔차가 크다: 한계 활성 여부, 특이 자세(`Manipulability()` 와 $w_5$), 가중치 비율 확인.
- 접근축이 roll 과 섞인다: $R_{WC}^\top$ 회전 방향(LWA → catch frame)과 catch frame 축 정의(plan §10 YAML) 확인.
- 명령 대비 측정 지연이 식별값과 다르다: 드라이버 파라미터가 식별 때와 같은지, 시뮬레이션이면 에뮬레이션 설정 확인.
- 관절 한계 근처에서 떨림: `limit_margin`, 가속 경계와의 충돌 빈도 확인.
- $q_c$ 가 한 틱에 튄다: CLIK 실패 출력(`q_ref = q_meas`)을 소비했는지, 재앵커가 §4.2 표 밖 시점에 일어났는지 확인.

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G5-A | 정지 목표에서 위치 오차 < 1 mm, 축 오차 < 0.5° 수렴 | `[SIM-ANY]` |
| G5-A2 | 옵션 전부 off 에서 기존 CLIK 출력 bit-identical, 기존 테스트 assertion 무수정 green (S2.4) | `[SIM-ANY]` |
| G5-B | 무작위 기준 1e4 틱에서 속도·가속 한계 위반 0, 위치 한계 위반은 `limit_margin` 이내 (§4.3 충돌 규칙) | `[SIM-ANY]` |
| G5-B2 | 경계 충돌 유도 시나리오: `bound_conflict` 발생, $\vert\dot q^\ast-\dot q_{prev}\vert\le\ddot q_{\max}\Delta t$ 유지, L7이 `ABORT_SAFE`로 전이 | `[SIM-ANY]` |
| G5-C | RT: page fault 0, 할당 0, QP 차원 고정, solve time 분위수 < 예산. **예산 확정 (2026-09-22 사용자, provisional)**: `control_rate` 500 Hz 의 tick 2000 µs 기준 **p99 ≤ 400 µs (20 %) · 최대 ≤ 1500 µs (75 %)** — 평균이 아니라 꼬리로 건다 (L8 §5). S5 실측이 훨씬 작으면 그때 조인다 (plan §4.3 S5·§7.3) | `[SIM-ANY]` |
| G5-C2 | backend 왕복: `ControllerOutput.devices[0].commands` 와 backend 가 쓴 명령 slot 이 전 틱에서 일치 (backend clamp 미발동) | `[SIM-P1B]` / `[HW-P1B]` |
| G5-C3 | `max_iter` 설정값 준수, 초과 시 status 노출 + 관절공간 abort 경로(가속 box 준수), L7 `QP_FAILED` 전이. RT tick 에 try/catch 없음, `Compute` noexcept | `[SIM-ANY]` |
| G5-C4 | 재무장·E-STOP 해제 reseed 후 첫 틱에 `bound_conflict` 미발생, 자동 재개 없음, `ClearEstop` 후에도 latched fault 유지 (P-1, S5 E-8 최소 계약) | `[SIM-ANY]` |
| ~~G5-D~~ | **은퇴 (2026-09-20)** — S3.7 이 빠지고 sim 에 주입 지연이 없어 주입할 대상이 없다 (actuator 고유 1차 지연은 있다 — §4.4 정정; 그 식별은 §4.4 LS 적합으로 S8-A 도구가 한다). 식별 오차 판정은 S10 의 G5-F 로 간다 | — |
| G5-E | (**S5 게이트**) **PASS (2026-09-22, S5.3)** — `integrated_bringup/test/arm_lag_fixture.hpp` 의 순수 지연 큐 (50 ms) 위에서 같은 공·같은 plant 로 보상 off/on 두 번 돌려 $t_c$ 측정 자세의 위치 오차를 기록: **77 mm → 71 mm**. 방향은 일치하고 크기는 작다 — 0.4 s 지평에서 0.75 m/s 로 움직이는 γ-스케일 목표를 쫓는 절대 오차가 지배하므로, 이 수치는 "보상이 작동한다" 이지 "보상으로 충분하다" 가 아니다. 순수 지연 모델이라 L5 §4.4 의 1차 성분은 빠져 있다 (S10 · sim 런타임은 S8-B G8-E — 2026-09-24 부터 L8 G8-E 는 이 fixture 가 아니라 sim 런타임 lead on/off 로 판정한다, D-S8-1). 이하 원문: ⚠️ **판정 입력이 없다** — S3.7 이 2026-09-20 결정으로 빠져 sim 에 에뮬레이션 지연이 없고, 지연 0 에서는 전후가 같아 **측정이 공허하다**. **확정 (2026-09-22 사용자): §4.5 의 fixture 전용 지연 주입으로 살린다** — 테스트 전용 지연 큐, 런타임 경로 불변 (plan §7.3 G5-E substrate 행). S5.3 에서 fixture 를 만들고 이 게이트를 그 위에서 판정한다 | `[SIM-P1B]` |
| G5-F | 실기 `T_arm` 식별 및 YAML 확정 (S10) | `[HW-P1B]` |

## 10. 미확정 항목

- 닫힘 (S2.2a, §5.1 표): 확장 구조, q_c 평가 cache 소유, 실패 후 재앵커, 명령값 모드의 `anchor_drift_max`, 관절별 속도 한계
- S5.3 관절공간 abort 감속 법칙 세부
- `joint_cmd.K_a`, `robot.arm.q_nominal`, `supervisor.track_err_abort` (L7), `joint_cmd.lag.*` (S10)
- E-STOP·fault 전체 정책 (D-13, S9)
- 닫힘: TBD-RTC-09~13, TBD-ARM-01, TBD-ARM-02 (→ D-16), TBD-SIM-01, TBD-FRAME-01 (→ D-17)
