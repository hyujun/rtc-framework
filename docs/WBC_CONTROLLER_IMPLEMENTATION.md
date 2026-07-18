# DemoWbcController 현재 구현 분석

이 문서는 `integrated_bringup`의 `DemoWbcController`가 **현재 실제로 수행하는 제어**를 설명한다. 문서의 기준은 구현 코드와 제공되는 WBC YAML이며, 과거 설계 문서에 남아 있는 "TSID 가속도를 적분하여 위치 명령을 생성"한다는 설명은 현재의 주 명령 경로를 나타내지 않는다.

현재 구조에서 position command의 유일한 백본은 **CLIK-QP**이고, **TSID inverse-dynamics QP**는 접촉력/토크 해석, 진단, 그리고 선택적인 손 feedforward torque에 사용된다.

## 범위와 주요 소스

- 컨트롤러 인터페이스·상태·버퍼: `integrated_bringup/include/integrated_bringup/controllers/demo_wbc_controller.hpp`
- RT tick·센서·공통 단계·CLIK·TSID·출력: `integrated_bringup/src/controllers/wbc/compute.cpp`, `controller.cpp`
- grasp FSM 및 phase 진입 처리: `integrated_bringup/src/controllers/wbc/phase.cpp`
- force PI: `integrated_bringup/src/controllers/wbc/force_reference_updater.cpp`
- TSID task/constraint factory: `integrated_bringup/src/controllers/wbc/controller.cpp`
- 기본 예시 설정: `integrated_bringup/config/ur5e_p1a/controllers/demo_wbc_controller.yaml`

프로파일마다 arm/hand 및 YAML 값은 달라질 수 있지만, controller의 제어 흐름은 공유한다.

## 코드 구성: 단일 클래스, 7개 translation unit

`DemoWbcController`는 `RTControllerInterface`를 상속한 **단일 클래스**를 7개 `.cpp`로 나눈 것이다. 상속 계층이 아니라 컴파일·감사 용이성을 위한 기능별 파티션이며, 모든 멤버 *상태*는 헤더에 있다. RT-critical 코드(`compute.cpp`)와 yaml-cpp·task 헤더를 끌어오는 무거운 config 코드(`controller.cpp`)를 분리하는 것이 핵심 의도다.

| TU | 책임 |
|---|---|
| `controller.cpp` | config-time 생성(ctor/dtor, 모델 빌드, TSID task/constraint factory, `LoadConfig`, MPC config), RT target marshalling(`Compute` 진입, SPSC drain), E-STOP setter |
| `compute.cpp` | RT hot path(센서 read, phase dispatch, 두 QP solve, release/fallback, 출력 조립, FK, log POD fill) |
| `phase.cpp` | grasp FSM(transition guard, phase-enter side-effect, TCP quintic trajectory) |
| `lifecycle.cpp` | lifecycle 콜백 + owned topic/log/service 셋업 |
| `parameters.cpp` | ROS 2 파라미터 선언 + set 콜백(런타임 gain) |
| `force_reference_updater.cpp` | per-contact force-PI helper (별도 클래스) |
| `grasp_phase_manager.cpp` | MPC측 grasp FSM `phase::GraspPhaseManager` (별도 클래스, WBC FSM을 mirror) |

Config는 세 단계에 분산되며 호출 순서가 중요하다.

1. **`LoadConfig`** (Tier 1, device config 도착 전) — Pinocchio 모델, TSID init, task/constraint factory, phase preset 사전 해석. **arm DoF는 `estop.arm_safe_position` 길이에서 확정**한다.
2. **`OnDeviceConfigsSet`** (device config 도착 후) — `hand_dof_`/`full_dof_` 및 joint reorder map 구성, `se3_tcp.base_frame` ↔ `urdf.root_link` 검증. 이 콜백은 throw할 수 없으므로 mismatch는 `base_frame_mismatch_` 플래그로 지연시킨다.
3. **`on_configure`** — 플래그가 서 있으면 transition FAIL. owned topic/TF slot/CSV log/service 등록, `DeclareGainParameters`, `InitClik`. **TSID가 init됐는데 CLIK enable에 실패하면 configure를 FAIL시킨다** — CLIK이 유일한 위치 backbone이기 때문이다.

`on_activate`에서 MPC thread를 spawn(heap alloc은 off-RT)하고 `target_initialized_=false`로 두어 첫 `Compute`가 self-seed하게 한다. **소멸자는 멤버 자동 소멸 전에 MPC thread를 명시적으로 join한다** — 역순 소멸로 Pinocchio 모델이 먼저 해제되면 실행 중인 MPC thread가 use-after-free(shutdown SEGV)를 낸다.

## 한 tick의 실제 데이터 흐름

```text
device state + fingertip inference + target
  │
  ├─ ReadState: |F|, native-contact, EMA d|F|/dt
  ├─ DrainTargetSlot: non-RT target → RT snapshot
  ├─ UpdatePhase: grasp FSM / phase preset
  │
  └─ ComputeTSIDPosition (정상 phase)
       ├─ Common stage
       │   ├─ device 순서 → Pinocchio q,v
       │   ├─ contact activation ramp, Pinocchio M/h/g/J/dJv
       │   ├─ grasp matrix G 및 pseudoinverse/null-space cache
       │   ├─ MPC 또는 phase posture reference
       │   ├─ force PI → lambda_des
       │   └─ MPC off 시 TCP quintic SE3 reference
       │
       ├─ Kinematic WBC (CLIK-QP) ──► q_ref,v_ref ──► position command
       └─ Dynamic WBC (TSID WQP) ────► a_opt,lambda_opt,tau
                                      └─ optional hand tau_ff
```

`DemoWbcController::Compute()`가 이 흐름의 RT 진입점이다. 매 tick 먼저 stale hand feedforward 활성 플래그를 지우고, E-STOP을 최우선으로 처리한다. measured pose로 idle hold target을 아직 만들지 못한 초기 tick은 TSID/CLIK를 실행하지 않고 hold command만 내보낸다.

외부 callback이 RT working state를 직접 변경하지 않는다. joint target과 SE3 target은 SPSC queue를 거치고, RT thread가 이를 drain해 `SeqLock` 기반 target snapshot에 반영한다. 이 때문에 target 수신과 RT tick 사이에 data race가 생기지 않는다.

## 상태·좌표계·센서 처리

### 관절 순서

device가 내보내는 arm/hand channel 순서와 Pinocchio model 순서는 다를 수 있다. `ExtractFullState()`는 `ext_to_pin_q_`, `ext_to_pin_v_` map으로 measured state를 full model의 `q_curr_full_`, `v_curr_full_`에 넣는다. CLIK가 낸 `q_ref`, `v_ref`는 같은 map의 역방향으로 `robot_computed_`, `hand_computed_`에 기록된다.

SE3 target도 root/base frame contract를 따라야 한다. YAML의 `se3_tcp.base_frame`은 arm device의 URDF root frame과 일치해야 한다. mismatch는 TCP target이 조용히 다른 좌표계에서 해석되는 위험이 있으므로 lifecycle 초기화 시 검증 대상이다.

### 손가락 inference

hand device의 inference block에서 fingertip마다 다음 값을 사용한다.

| Slot | 의미 |
|---|---|
| 0 | native contact probability (지원 sensor만) |
| 1..3 | force `(fx, fy, fz)` |
| 4..6 | displacement 예약 영역; 현재 WBC는 사용하지 않음 |

컨트롤러는 `|F| = sqrt(fx^2 + fy^2 + fz^2)`를 계산한다. native-contact 경로에서는 `probability > grasp_contact_threshold`와 `|F| > grasp_force_threshold`가 모두 참이어야 `in_contact`다. native-contact를 지원하지 않는 경로는 force threshold만 사용한다. 또한 `d|F|/dt`는 EMA로 평활하며 첫 tick의 startup spike는 버린다.

주의할 점은 FSM의 closure-to-hold 판정이 `in_contact`가 아니라 raw `|F| > force_contact_threshold` count를 사용한다는 것이다. 따라서 publish되는 grasp status와 FSM 전이가 순간적으로 다를 수 있다.

## Grasp FSM

도달 가능한 상태와 전이는 다음과 같다. enum 값 2와 5는 하위 소비자 ABI 호환을 위해 예약되어 있으며 현재 상태가 아니다.

```text
Idle(0) -- grasp_cmd=1, new arm target --> Approach(1)
Approach -- TCP error < epsilon_pregrasp --> Closure(3)
Closure -- active force contacts >= min_contacts_for_hold --> Hold(4)
Hold -- |d|F|/dt| > slip_rate_threshold --> Fallback(7)

Approach / Closure / Hold -- grasp_cmd=2 --> Release(6) -- hand open done --> Idle
Approach / Closure / Hold -- grasp_cmd=0 --> Idle
Fallback -- grasp_cmd=0 --> Idle
```

`grasp_cmd=2` release와 `grasp_cmd=0` abort는 상태별 일반 전이보다 먼저 처리된다.

- **Idle**: 현재 measured joint/TCP를 hold target으로 seed한다. commanded SE3가 있으면 그것이 measured TCP hold를 override한다.
- **Approach**: arm joint target의 FK로 TCP goal을 만든다. 명시적 SE3 command가 있으면 이를 우선한다. MPC가 꺼진 경우 현재 TCP에서 goal까지 rest-to-rest quintic SE3 trajectory를 생성한다.
- **Closure/Hold**: contact activation target을 1로 ramp한다. closure 진입 때 force PI를 reset하고 object task reference를 seed한다.
- **Release**: contact activation을 0으로 먼저 ramp한 뒤, hand joint를 zero/open pose까지 trajectory로 이동한다. arm은 현재 TCP를 hold한다.
- **Fallback**: 마지막 valid position command를 hold하고 velocity를 0으로 만든다. 동적 feedforward는 사용하지 않는다.

hold의 anomaly guard는 현재 force-rate slip만 구현돼 있다. YAML에 있는 deformation threshold는 센서 displacement를 아직 소비하지 않으므로 동작하지 않는다. 또한 CLIK QP가 `max_qp_fail_before_fallback`번 연속 실패해도 `Fallback`으로 강제 전이한다.

FSM은 사실상 두 벌이다. 위 WBC FSM이 authoritative이며, MPC가 켜진 경우 phase 진입 시 `GraspPhaseManager::ForcePhase`로 MPC측 FSM을 mirror한다. mirror mode가 latch되면 MPC FSM은 자체 guard를 억제하고 WBC edge를 그대로 추종하며, phase별 OCP type(`contact_light`/`contact_rich`)과 cost 테이블을 전환한다. grasp goal pose는 Eigen/SE3가 trivially-copyable이 아니므로 POD mirror를 거쳐 `SeqLock`으로 전달한다.

## 공통 단계: reference와 접촉 모델

정상 TSID-routing phase에서 `ComputeWbcCommon()`은 두 QP가 함께 쓸 입력을 딱 한 번 준비한다.

1. measured state를 Pinocchio order로 추출한다.
2. 각 contact activation `s_i in [0,1]`를 `contact_ramp_sec` 동안 변화시키고 active contact를 재계산한다.
3. Pinocchio cache에 질량행렬 `M`, bias `h`, gravity `g`, Jacobian `J`, `dJv`를 갱신한다.
4. active contact column으로 grasp matrix `G`를 만들고 `G+`, `(G^T)+`, null-space projector, rank를 `GraspCache`에 계산한다.
5. MPC reference가 유효하면 `q_des`, `v_des`, `a_des`에 주입한다. 그렇지 않으면 phase별 hold/target posture를 쓴다.
6. Closure/Hold에서 force PI가 ForceTask reference `lambda_des`를 만든다.
7. MPC가 꺼져 있고 SE3 task가 active이면 quintic trajectory의 pose/velocity/acceleration을 SE3Task에 매 tick 전달한다.

### 축약(closed-chain) 동역학

확장 URDF hand처럼 loop-passive joint만 lock한 actuated closed-chain model을 쓰는 경우, `computeAllTerms`가 낸 open-chain `M/h/g`는 loop closure를 무시하므로 그대로 쓰면 안 된다. `WbcReducedDynamicsProvider`가 주입돼 있으면 Pinocchio cache 갱신 직후 `M/h/g`를 **constraint-consistent 축약 동역학으로 덮어쓴다** (`wbc_reduced_dynamics_provider.cpp`, 실제 축약은 `rtc_urdf_bridge`의 `rt_closed_chain_handle.cpp`).

- 감축 사상 `G`(nv×n_a)는 종속 좌표 블록 `-A⁻¹B`(`A = Jc_Dᵀ Jc_D`, `B = Jc_Dᵀ Jc_I`, damped 정규방정식 좌-pseudoinverse)와 독립 좌표 identity 블록으로 구성한다.
- 축약 관성 `M_a = Gᵀ M G`, 축약 중력 `g_a = Gᵀ g`, 축약 비선형효과 `h_a = Gᵀ · rnea(q, G·v_a, a_drift)` (`v=0`이면 `h_a = g_a`).
- **접촉 프레임의 placement·Jacobian(`oMf`·`J`)은 loop-downstream contact에 한해 loop-consistent 값으로 격상**한다 (`FillReducedFrameKinematics` — 같은 `Update()` 안에서 축약 동역학이 이미 사영한 `RtClosedChainHandle`을 재사용하므로 재사영 없음). 비-downstream/미매핑 frame은 open-chain 정확값을 그대로 유지한다(byte-for-byte). 단 **접촉 프레임 drift `dJv`는 아직 frozen-loop 0(L2-zero)** — 핸들에 loop-consistent 가속도 API가 없어 미격상이며 후속 과제다(issue #173).
- `Jc_D`의 최소 특이값이 임계 이하이면 singular로 표시하고 **마지막 정상 축약 동역학을 유지**한다(열화된 dynamics를 절대 주입하지 않는다). provider가 실패를 반환하면 open-chain 값으로 fallback한다.

fixed-base 개곡선 arm(예: reduced tree UR5e, `nq==nv`)에는 provider가 없고 open-chain `M/h/g`를 그대로 쓴다.

### 접촉 force PI

각 contact의 측정 force magnitude에 대해 다음을 사용한다.

```text
e = f_des - |F_measured|
I_next = clamp(I + ki * e * dt, -i_max, i_max)
lambda_n = clamp(kp * e + I + f_des, lambda_min, lambda_max)
lambda_des_force = lambda_n * contact_normal
```

출력이 이미 saturation이고 적분 step이 saturation 방향을 더 밀면 그 step을 commit하지 않는 conditional anti-windup이다. invalid sensor가 들어오면 해당 contact의 integrator와 output을 즉시 0으로 reset한다.

현재 force PI는 contact index와 fingertip sensor index가 동일한 순서라고 가정한다. YAML의 `contacts` 순서와 inference sensor 순서가 달라지면 다른 손가락의 힘으로 reference를 만들게 된다. 또 피드백은 normal projection이 아닌 `|F|`를 쓰지만 output은 configured contact normal 방향이다.

## Kinematic WBC: 실제 위치 명령 경로

`ComputeKinematicWbc()`가 arm과 hand의 position command를 소유한다. 입력은 Pinocchio cache, TCP goal(또는 current quintic sample), posture reference `q_des`다.

CLIK는 TCP tracking term과 arm/hand null-space posture term을 함께 풀어 velocity reference를 만든다. 중요한 runtime tuning 값은 다음과 같다.

| Parameter | 의미 |
|---|---|
| `clik_kx_pos`, `clik_kx_rot` | TCP 병진/회전 task gain |
| `clik_ka`, `clik_kh` | arm/hand posture null-space gain |
| `clik.damping_sq` | damped inverse의 damping |
| `clik.v_limit` | joint velocity reference clamp |
| `clik.anchor_drift_max` | desired와 measured 사이 carry-forward drift 제한 |

내부적으로 CLIK은 관절속도 `v ∈ R^nv`를 결정변수로 하는 box QP(등식 없음)를 같은 ProxSuite 백엔드로 푼다.

- 태스크 항: `e_x = log6(T⁻¹ T_des)`(LWA 프레임), `r_task = kx ⊙ e_x`, arm 열만 채운 태스크 Jacobian `J_task`.
- posture 항: arm은 `ka·(q_des−q)`, hand는 `kh·(q_des−q)`.
- 비용: `H = w_task·J_taskᵀ J_task + diag(w_arm, w_hand) + μ²I`, `g = −w_task·J_taskᵀ r_task − scatter(posture)`. `μ² = damping_sq`가 damped right-inverse 정규화다.
- box 제약: `±v_limit` 관절속도 clamp와 `(q_lim−q)/dt` 위치→속도 한계의 교집합.
- 적분(semi-implicit / one-step Euler): `v_ref = x_opt` → **`q_ref = q_anchor + v_ref·dt`**.

기본 예시 값(`ur5e_p1a`): `damping_sq=1e-4`, `v_limit=1.5 rad/s`, `kx_pos=kx_rot=5.0`, `ka=kh=1.0`, `anchor_drift_max=0.5 rad`.

CLIK reference는 이전 desired에서 전진하는 carry-forward 방식이며, 새 goal 또는 idle/release phase edge에서는 measured pose로 re-anchor한다(`anchor_drift_max`가 measured와의 drift를 관절별로 제한). 성공하면 CLIK의 `QRef()`, `VRef()`를 device order로 map해 wire command에 사용한다.

CLIK failure는 position safety에 critical이다. 이번 tick은 이전 command를 hold하고, `max_qp_fail_before_fallback`번 연속 실패하면 `Fallback`으로 전환한다. 따라서 현재 구현은 TSID `a_opt`를 semi-implicit Euler로 적분해 position을 만드는 경로를 사용하지 않는다.

## Dynamic WBC: TSID inverse dynamics와 hand torque

TSID solver는 WQP(기본) 또는 설정 가능한 HQP formulation으로 다음 fixed-size 변수를 푼다.

\[
z = \begin{bmatrix}a\\lambda\end{bmatrix}
\]

여기서 `a`는 generalized acceleration, `lambda`는 point contact의 3D force 또는 surface contact의 6D wrench다. active contact 수가 달라져도 solver dimension은 max contact capacity로 고정한다. inactive `lambda` slot은 regularization으로 0 근처에 유지되어 RT 중 resize/allocation을 피한다.

기본 WQP는 active task를 weighted least-squares로 합친다.

\[
\min_z \sum_i w_i\|J_i z-r_i\|^2 + 10^{-8}\|z\|^2
\]

solver는 **ProxSuite(ProxQP) dense**이며 RT-safe하게 쓰인다. max 차원으로 **1회 할당**한 뒤 매 tick `update()` + `solve()`만 호출해 heap 재할당을 피하고, 첫 solve 이후 이전 결과로 warm-start한다. 기본 예시 값(`ur5e_p1a`, `tsid.wqp.solver`): `max_iter=20`, `eps_abs=1e-6`, `eps_rel=0`. inner iteration 상한은 infeasible QP가 500 Hz tick 안에서 빨리 실패하도록 낮게 잡혀 있다. active contact 수가 바뀌어도 dimension은 고정이고 inactive `lambda` slot은 `±inf` bound + `1e-8` 정규화로 0 근처에 유지된다.

solve 후 `a_opt = z[0:nv]`, `lambda_opt = z[nv:]`를 뽑고 EoM을 역산해 actuated torque `tau = S(M·a_opt + h − Σ Jc_iᵀ λ_i)`를 복원한다. 이 `tau`가 선택적 hand feedforward(`tsid_tau` source)의 출처다.

TSID task와 제약의 실제 구성은 YAML 및 factory에서 결정된다.

| 분류 | 구성 요소 | 현재 역할 |
|---|---|---|
| Task | posture | desired joint acceleration regularization |
| Task | `se3_tcp` | TCP 6D acceleration tracking |
| Task | force | `lambda_des` 추종; activation에 따라 scale |
| Task | contact consistency | `Jc*a + dJv ~= 0` soft no-slip |
| Task | object wrench/internal force/object SE3 | closure/hold의 grasp-level objective |
| Constraint | EoM | floating-base일 때 unactuated dynamics equality; fixed-base에는 equality 차원 0 가능 |
| Constraint | joint limit | position/velocity viability를 acceleration bound로 변환 |
| Constraint | friction cone | contact별 `mu`, `n_faces` 선형 마찰 피라미드 |
| Constraint | torque limit | `tau = S(Ma+h-Jc^T lambda)`의 effort bound |
| Optional constraint | contact constraint | hard `Jc*a + dJv = 0`; 제공 YAML에서는 off |

TSID QP 실패는 **position에 non-critical**이다. CLIK는 계속 position command를 생성하고, 이번 tick의 hand feedforward만 0으로 만든다. 이는 TSID failure가 곧바로 grasp force/torque overlay 손실을 뜻하지만, arm/hand position hold 자체가 중단되지는 않음을 의미한다.

### Optional hand `tau_ff`

기본값은 `hand_tauff_enable: false`다. 활성화된 Closure/Hold에서만 손 device command type을 `kPdFeedforward`로 바꾼다. position target은 여전히 CLIK output이고, feedforward는 다음 중 하나다.

- `gravity_comp`: Pinocchio gravity vector의 hand 관절 성분
- `tsid_tau`: TSID가 복원한 actuated torque의 hand 성분

두 source 모두 scale, closure bias, per-joint clamp를 거친다. non-finite 토크나 TSID QP failure 시 모든 hand `tau_ff`는 0이 된다. 이 분리는 arm을 torque mode로 바꾸지 않으며, hand position PD를 기본 안전 백본으로 유지한다.

## 상세 알고리즘: task·constraint 수식

앞의 Dynamic WBC 표가 각 요소의 *역할*을 요약했다면, 이 절은 두 QP가 실제로 푸는 *수식*을 정리한다. 값은 `ur5e_p1a` 예시 config 기준이며 프로파일마다 다를 수 있다. task 수식은 `rtc_tsid/src/tasks/*`, constraint는 `rtc_tsid/src/constraints/*`, grasp cache는 `rtc_tsid/src/contact/*`가 SSoT다.

### 실행 계약: 두 QP의 역할과 실패 정책

두 QP는 직렬로 서로의 출력을 전달하는 제어기가 아니다. Common stage가 같은 measured state, Pinocchio cache, contact state, posture/SE3 reference를 준비하고, 이를 각각 소비한다.

```text
Kinematic WBC (CLIK-QP) : q_ref, v_ref  ──> device position command (유일한 backbone)
Dynamic WBC (TSID-ID QP): a_opt, λ_opt, τ ──> 진단 + 선택적 hand τ_ff
```

- **CLIK 실패**: 이번 tick은 이전 position command를 유지한다. 연속 실패 수가 `max_qp_fail_before_fallback`에 도달하면 `kFallback`으로 전환하고 velocity를 0으로 만든다.
- **TSID 실패**: CLIK position command는 계속 사용한다. 이번 tick의 hand feedforward를 모두 0으로 하여 plain position-PD로 되돌린다.
- 따라서 `a_opt`는 현재 구현에서 device position command로 적분되지 않는다. TSID의 torque는 `hand_tauff_enable`이고 phase가 Closure/Hold일 때만 hand `kPdFeedforward` overlay의 후보가 된다.

이 분리는 TSID의 접촉·토크 feasibility와 CLIK의 position safety를 독립적으로 관찰할 수 있게 하지만, TSID failure가 grasp force/torque overlay의 손실을 뜻한다는 점은 운영상 별도로 모니터링해야 한다.

### Tick 알고리즘

정상 TSID-routing phase(Idle, Approach, Closure, Hold 및 Release의 TSID 구간)는 매 control tick에 아래 순서로 실행된다. 모든 QP workspace와 contact capacity는 configure 시점에 준비되어 있으며, 이 단계는 resize나 solver 재생성을 하지 않는다.

```text
1. device/inference state와 RT target snapshot을 읽고 grasp FSM을 갱신한다.
2. device 순서를 Pinocchio q, v 순서로 변환한다.
3. contact activation s_i를 갱신하고 active contact/active λ dimension을 재계산한다.
4. Pinocchio cache(M, h, g, frame/contact Jacobian, dJv)를 갱신한다.
5. active-contact grasp cache(G+, (Gᵀ)+, P_N, rank)를 준비한다.
6. 유효한 MPC reference가 있으면 posture reference로 사용하고, 없으면 phase hold/target을 사용한다.
7. Closure/Hold에서는 force PI로 λ_des를, MPC-off SE3 phase에서는 quintic TCP reference를 갱신한다.
8. CLIK-QP를 풀어 q_ref, v_ref를 만들고 device order로 map한다.
9. TSID-ID QP를 풀어 a_opt, λ_opt, τ를 만든 뒤, 조건을 만족하면 hand τ_ff를 overlay한다.
```

8과 9는 같은 Common-stage 입력을 소비한다. 호출 순서가 CLIK 뒤 TSID인 것은 hand feedforward failure가 position command를 무효화하지 않게 하기 위한 현재 구현의 안전 정책이며, 두 solver 사이에 `a_opt → q_ref` 전달은 없다.

### 결정변수와 비용

Dynamic WBC(TSID)의 결정변수는 가속도와 **최대 contact capacity 기준** 접촉력을 쌓은 fixed-size 벡터다.

```text
z = [ a ; λ_slot ] ∈ R^(nv + nλ,max)
  a  ∈ R^nv     generalized acceleration
  λ_slot,i      contact i force(point, cdim=3) 또는 wrench(surface, cdim=6)
```

`nλ,max = Σ_i cdim_i`는 YAML의 모든 configured contact를 합친 값이며 QP column offset은 contact slot에 고정된다. active contact만 packed `G`, residual row, constraint row에 기여하지만 inactive slot은 decision vector에서 제거되지 않는다. 이 설계는 activation 변화 중에도 QP 차원과 workspace를 고정해 RT 경로의 allocation을 피한다.

각 active task는 선형 잔차 `(J_i, r_i)`를 방출하고 WQP가 가중 최소자승으로 합친다. `H`는 앞서 본 대로 `Σ w_i J_iᵀJ_i + 1e-8·I`, `g = −Σ w_i J_iᵀ r_i`이며, 가중치 `w_i`는 스칼라 선형(√w 아님)이고 phase preset이 tick마다 active/weight를 바꾼다.

### TSID task 잔차

`s = s_i ∈ [0,1]`는 contact activation ramp이며 `√s`를 J·r 양쪽에 곱해 비용을 `w·s·‖…‖²`로 만들어 접촉을 fade in/out한다. 단, 현 구현은 `s >= kActivationDeadband (1e-3)`일 때에만 legacy `active` flag를 켜고 residual row를 만든다. 따라서 ramp의 최초 deadband 구간은 cost가 정확히 0이고, 그 뒤부터 `√s` scaling이 적용된다. `J_f`, `J̇_f v`는 대상 frame의 LWA Jacobian과 drift다.

| Task | J = `[ a열 \| λ열 ]` | r | 예시 gain·weight |
|---|---|---|---|
| **posture** | `[ I_nv \| 0 ]` | `Kp⊙(q_des−q) + Kd⊙(v_des−v) + a_ff` | `kp=10, kd=6`, w=1 |
| **se3_tcp** | `[ J_f(mask) \| 0 ]` | `a_des − J̇_f v`,  `a_des = a_ff + Kp⊙e_pos + Kd⊙e_vel` | `Kp=[100,100,100,50,50,50]`, `Kd=[20,20,20,10,10,10]`, w=50–100 |
| **force** | `[ 0 \| √s·I ]` | `√s · λ_des` | w=10–20 |
| **contact_consistency** | `[ √s·J_c \| 0 ]` | `−√s · (J̇_c v)` | w=100 |
| **object_wrench** | `[ 0 \| G ]` | `w_obj_des = [0,0,m·g,0,0,0]ᵀ` | w=100 (`mass=0`→no-op) |
| **internal_force** | `[ 0 \| I ]` (active λ) | `P_N · λ_squeeze_des` | w=50 (`λ_squeeze≡0`) |
| **object_se3** | `[ (Gᵀ)⁺ J_c,stack \| 0 ]` | `a_obj_des − (Gᵀ)⁺ (J̇_c v)_stack` | w=50 |

posture는 fixed-base(`nq==nv`)에서만 `q_des−q` 항을 직접 쓴다. force/object_wrench/internal_force는 λ에만, se3/contact_consistency/object_se3는 a에만 작용한다.

### SE3 pose error 정의

SE3 오차는 `base_frame`에서 해석되는 LWA body log6 오차다.

```text
e_pos = twistLocalToWorld( log6( T_tip⁻¹ · T_des ) )  ∈ R^6  [linear; angular]
e_vel = ė_pos  (Jlog6 정방향)  ≈ v_des − J_f v   (소오차 근사)
```

`T_tip`은 tip을 `base_frame`으로 `actInv`한 것이다. 회전 오차는 SO(3) log map이므로 θ→π 특이 근방에서 방향은 유지하고 크기만 clamp해 방어한다(quaternion lerp/nlerp 금지 불변식과 정합).

### TSID hard constraint

부등식은 `l ≤ C z ≤ u` 형태다.

**관절 한계 (viability, Del Prete 2018)** — `C = [ I_nv \| 0 ]`. 위치·속도 한계를 한 tick lookahead로 가속 bound로 변환하고 교집합한다.

```text
position → accel:  2(q_min − q − v·dt)/dt²  ≤ a ≤  2(q_max − q − v·dt)/dt²
velocity → accel:  (v_min − v)/dt           ≤ a ≤  (v_max − v)/dt
l = max(하한들),  u = min(상한들)
```

`position_margin`, `velocity_margin`이 한계를 안쪽으로 좁힌다.

**마찰뿔 (선형화 Coulomb)** — point contact의 world-frame force `λ_w`에 대해 face k(`θ_k = 2πk/n`)마다:

```text
[ cosθ_k, sinθ_k, −μ ] · R_cᵀ λ_w ≤ 0        (측면 n행)
             − nᵀ λ_w ≤ 0                      (unilateral 1행)
```

`R_c = [t₁ | t₂ | n]`은 런타임 contact normal에 Gram-Schmidt로 세운 contact-local→world basis다(+Z 법선은 `R_c=I` 지름길). surface contact는 CoP 사각형 + yaw-moment 6행을 추가한다. 예시 `μ=0.5, n_faces=8` → contact당 9행. `n_faces`/`μ`는 constraint block이 아니라 `contacts:`에서 읽는다.

**토크 한계** — `τ = S(M a + h − J_cᵀ λ)`를 z에 사상한다.

```text
C = [ S·M | −S·J_cᵀ ],   l = τ_min − S·h,   u = τ_max − S·h
```

**EoM 등식** — floating base에서만 `P(M a + h − J_cᵀ λ) = 0` (`P = I − SᵀS`). 이 로봇은 fixed base라 unactuated 차원이 0이므로 **등식 행이 없고**, 동역학은 torque recovery로 암묵 강제된다.

### Grasp matrix G와 GraspCache

`G`는 활성 접촉 wrench를 **world-aligned 물체 원점 wrench**로 합산하는 기하학적 사상이다. 즉 grasp matrix 자체를 만들기 위해 접촉 Jacobian을 곱하지는 않는다. 필요한 최소 입력은 다음과 같다.

| 입력 | 현재 source | G에서의 역할 |
|---|---|---|
| arm-hand 전체 관절 위치 `q`와 통합 URDF/kinematic model | device state → Pinocchio | fingertip/contact frame의 forward kinematics를 계산한다. |
| 접촉 frame의 world 위치 `p_ci` | `pinocchio_cache_.contact_frames[i].oMf.translation()` | 물체 원점까지의 lever arm `d_i = p_ci - p_o`를 만든다. |
| 물체 기준점 `p_o` | `tsid.object_frame.p_w` | 합산 wrench의 모멘트 기준점을 정한다. |
| contact type/dimension | `tsid.contacts[i].contact_dim` | point(3 force DoF) 또는 surface(6 wrench DoF) block을 선택한다. |
| activation/active state | `ContactState` | 현재 packed G에 포함할 contact와 column order를 정한다. |

`ObjectFrame::R_w`는 현재 G에 사용하지 않는다. 모든 force/moment는 Pinocchio `LOCAL_WORLD_ALIGNED` contact convention과 같은 world-aligned basis에 있으므로, body-frame wrench가 필요하면 향후 별도의 회전 변환을 추가해야 한다. 접촉 법선·마찰계수·tactile force는 friction cone 및 force reference에는 필요하지만, 위의 기하학적 G block 자체에는 직접 들어가지 않는다.

active point contact i(위치 `p_ci`, 물체 원점 `p_o`)의 힘을 물체 원점 wrench로 사상하는 블록은 다음과 같다.

```text
B_i = [ I_3 ; skew(p_ci − p_o) ]  ∈ R^{6×3}
G   = [ B_1 … B_K ]              ∈ R^{6×n_λ},   w_obj = G λ
```

surface contact(`contact_dim=6`)은 contact wrench `λ_i=[f_i; m_i]`를 포함하며, 같은 위치 변환의 dual adjoint block을 사용한다.

```text
A_i = [ I_3              0_3 ]  ∈ R^{6×6}
      [ skew(p_ci − p_o)  I_3 ]
w_obj,i = A_i λ_i
```

따라서 point contact는 `d_i × f_i`만 모멘트에 더하고, surface contact는 원래 contact moment `m_i`도 보존한다. 여러 활성 접촉의 block은 **config index 순서**로 빈틈없이 pack되며, 비활성 contact는 G의 column을 차지하지 않는다. 반면 TSID 결정변수 `λ_slot`은 fixed capacity를 유지하므로 `ObjectWrenchTask`는 packed G를 해당 active contact의 고정 slot으로 다시 펼쳐 `[0 | G]` residual Jacobian을 구성한다.

contact Jacobian `J_ci(q)`와 bias `dJ_ci·v`는 G 구성 입력은 아니지만 contact kinematics와 object SE(3) task에는 필요하다.

```text
ẋ_ci = J_ci(q) v
J_c,stack = stack_i(J_ci.topRows(contact_dim_i))
r_object_se3 = a_obj,des − (Gᵀ)⁺ (dJ_c·v)_stack
```

즉 FK는 `p_ci`를 얻기 위해 G에 직접 필요하고, Jacobian은 no-slip/contact consistency 및 object motion을 generalized velocity/acceleration에 연결하기 위해 별도로 필요하다.

`GraspCache`가 tick당 1회 LDLT로 damped pseudoinverse를 미리 계산한다(ε = max(tol, machine_eps·trace)).

```text
n_λ ≥ 6:  G⁺ = Gᵀ(GGᵀ+εI)⁻¹,   (Gᵀ)⁺ = (GGᵀ+εI)⁻¹G
n_λ < 6:  G⁺ = (GᵀG+εI)⁻¹Gᵀ,   (Gᵀ)⁺ = G(GᵀG+εI)⁻¹
null projector:  P_N = I − G⁺G
```

`InternalForceTask`와 `ObjectSE3Task`는 이 cache의 `P_N`, `(Gᵀ)+`, rank를 읽는다. 반면 현재 `ObjectWrenchTask`는 자신의 `[0 | G]` residual block을 채우기 위해 같은 tick의 Pinocchio cache와 contact state에서 `G`를 다시 구성한다. 즉 controller common stage의 cache는 pseudoinverse/rank 공유의 기준점이며, object-wrench Jacobian의 직접 source는 task 내부의 재구성 결과다. 접촉 force PI(`lambda_des` 생성)는 위 「접촉 force PI」 절 참조.

## MPC와 object-level task의 현재 범위

MPC는 별도 thread에서 동작할 수 있으며, RT common stage는 lock-free manager에 current `q,v`를 전달하고 보간된 `q_ref,v_ref,a_ff,u_fb`를 소비한다. MPC reference가 유효하면 posture reference가 이를 사용하고, 그렇지 않으면 controller의 phase-specific hold/target이 사용된다. MPC 역시 CLIK의 device position mapping을 대체하지 않는다.

object-level task wiring은 구현돼 있지만 현재 기능 범위는 제한적이다.

- object frame은 static seed 성격이며 물체 pose estimator/vision provider는 없다.
- 기본 `object_mass`가 0이면 object wrench gravity reference도 0이다.
- internal squeeze reference는 현재 0이며 별도 force planner가 아직 쓰지 않는다.

따라서 object tasks는 grasp matrix와 task plumbing을 검증하는 기반은 제공하지만, 현재 데모가 완전한 object manipulation planner를 의미하지는 않는다.

## YAML을 읽을 때의 핵심

`demo_wbc_controller.yaml`의 다음 구획을 함께 봐야 한다.

1. `tsid.tasks`, `tsid.constraints`: task/constraint factory 등록과 수학적 formulation
2. `tsid.contacts`: fingertip frame, point/surface type, friction coefficient, face count
3. `tsid.phase_presets`: 상태별로 어떤 task/constraint를 active로 둘지 결정
4. `force_pi`: per-contact force loop
5. `clik`: 실제 position backbone의 구조 파라미터
6. `hand_tauff_*`: 선택적 hand torque overlay
7. `fsm`: grasp transition/safety threshold

특히 phase preset은 reset-style이다. 이전 phase에서 켜진 task를 확실히 끄려면 omission이 아니라 명시적인 `active: false`가 필요하다.

## 운영·튜닝 시 유의사항

- `se3_tcp.name`은 YAML key와 동일하게 `se3_tcp`여야 한다. controller는 task lookup과 phase preset에서 그 이름을 사용한다.
- friction face count는 constraint block이 아니라 각 contact의 `n_faces` 또는 `friction_faces`에서 읽는다.
- 기본 config에서 hard contact constraint는 꺼져 있다. 강체 no-slip을 hard guarantee로 바꾸려면 `contact_constraint`를 명시적으로 추가하고 feasibility를 검증해야 한다.
- Hold slip detection은 `d|F|/dt` 단일 threshold 기반이다. noisy tactile signal에서는 filter/threshold와 fallback sensitivity를 함께 조정해야 한다.
- TSID는 기본 `tau_ff`가 비활성이라도 normal phase마다 풀린다. RT budget을 평가할 때 diagnostic/contact analysis의 비용까지 포함해야 한다.
- E-STOP은 FSM보다 우선하며, stale hand feedforward replay를 막기 위해 tick 시작에서 `hand_tauff_active_`를 reset한다.

## 오래된 설명과의 구분

다음 표현은 현재 controller에 적용하면 안 된다.

> "TSID `a_opt`를 semi-implicit Euler로 적분한 결과가 arm/hand position command이다."

현재 동작은 다음이 맞다.

> "CLIK-QP가 `q_ref`, `v_ref`를 만들어 arm/hand position command를 생성한다. TSID inverse-dynamics QP는 `a_opt`, `lambda`, `tau`를 계산하며, 기본적으로 진단 및 선택적 hand feedforward torque에 기여한다."
