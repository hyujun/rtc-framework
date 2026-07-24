# Compliance controller conventions (normative)

이 문서는 `rtc_controllers` 의 **compliance (impedance / admittance) 컨트롤러 계열**이
따르는 **규범 매트릭스 (normative matrix)** 다. 코드·YAML·리뷰가 참조하는 SSoT 이며,
값이 바뀌면 여기부터 갱신한다. 현재 범위는 **슬라이스 1 — `TaskImpedanceController`
(A=NONE, Jacobian-transpose)** 이고, 나머지 행은 후속 슬라이스에서 채운다.

## 1. 입력 명세 provenance

컨트롤러 설계의 원 명세 2건은 저장소에 **커밋하지 않는다** (분량·소유권). 리비전은
아래 SHA-256 으로 고정하며, 재검토 시 이 해시로 동일본을 확인한다.

| 문서 | SHA-256 |
|---|---|
| `IMPEDANCE_ADMITTANCE_CONTROLLER_SPEC.md` | `950576ad6ce91a8ae57b598b7b0abf9edbd91ff0cd569df841be30e406854109` |
| `any_urdf_compliance_controller_design.md` | `c69dadd77664f13e17187d9f9cfeaa37e8d54bd1fc5d85f34db72dabd3cb8be1` |

## 2. Convention block (frame · ordering · sign — MUST)

명세 §19 의 `CONVENTIONS.md` 블록을 그대로 채택한다. **가장 위험한 버그는 부호가 아니라
frame mismatch** 이므로 (부호는 즉시 발산, frame 불일치는 특정 자세에서만 드러남) 모든
상태/진단에 frame 규약을 기록한다.

```text
- Pose:    A_T_B maps coordinates from frame B to frame A.
- Twist ordering:  [linear; angular]  = [vx,vy,vz, wx,wy,wz]
- Wrench ordering: [force;  torque]   = [fx,fy,fz, tx,ty,tz]
- Jacobian maps dq to twist in the DECLARED reference frame (default: LOCAL_WORLD_ALIGNED).
- Position error: desired - current.
- Rotation error: Log(R_current^T R_desired), then expressed in the declared frame.
- Wrench transform: Ad^{-T}, NOT Ad^T.  (verify with power-duality test)
- Positive external wrench: wrench applied BY the environment ON the robot/control body.
- Joint torque: actuator torque applied to the robot.
- Nullspace torque projection uses N_M^T (transpose); acceleration projection uses N_M.
```

- 부호 규약 MUST (§6.2): `e` 는 "현재 → desired" 방향이므로 제어 법칙은 **`+K_p·e`**
  (음수 부호 아님). 각 제어 법칙 구현부 주석에 명시한다.
- Wrench transform 은 `rtc_math/se3/wrench.hpp` (`transformWrench = Ad^{-T}·f`) 가 SSoT 이고,
  방향은 `WrenchTransform.PowerDuality` 테스트가 고정한다 (라이브러리 `act`/`actInv` 방향
  가정 금지).

## 3. 규범 매트릭스

| 항목 | 결정 (슬라이스 1) | 근거 |
|---|---|---|
| **Controller split** | OSC 확장 아님 — **별도 클래스 + 공용 helper 추출** (`compliance/task_dynamics`). OSC 는 `F=Λ·a_task` 가속도-게인 형, impedance 는 `Jᵀ(Ke+Dė)` 강성-게인 형 | enum 합치면 게인이 조용히 무의미해짐 (§6.0 금지). P5 는 Λ·Nᵀ·DLS 공용화로 충족 |
| **Formulation** | `kJacobianTranspose` (§6.2), A=NONE. `kOperationalSpaceInverseDyn` (§6.3) 는 슬라이스 2 | Λ 미사용 → **task-space 특이점 자유** (§6.5); f_ext 측정 불필요 |
| **Pose error frame** | `SplitWorld` = `LOCAL_WORLD_ALIGNED`, `[p_d−p ; log3(R_d Rᵀ)]` | §1.3, `rtc_math/se3/pose_error.hpp` |
| **Velocity error** | `ν_d − ν`; 정적 setpoint 는 `ν_d=0` ⇒ `ė = −J·q̇` | §1.4, SplitWorld = LWA |
| **Sign** | `+K_p·e` (desired−current) | §6.2 부호 규약 MUST |
| **Selection (axis B)** | `FULL_SE3` (m=6) \| `TRANSLATION_ONLY` (m=3). `J_S = S·J`, linear rows first | §6.1 |
| **Nullspace** | dynamically-consistent `Nᵀ = I − J_Sᵀ J̄_Sᵀ`, gate **`nv > task_dim`** (NOT `nv > 6`), 토크는 `Nᵀ` 투영 (가속도는 `N`) | §6.4 — 6-DoF+`TRANSLATION_ONLY` 는 dim-3 nullspace 가 **필수** |
| **DLS (singularity)** | σ_min-adaptive: `λ²=0` (σ≥σ₀), `λ_max²(1−(σ/σ₀)²)` (σ<σ₀). OSC 의 상수 λ 와 다름 | §6.5. `singularity_threshold`=0.02, `max_damping`=0.05 |
| **Safety layer order** | joint-limit repulsive (§5.3, 스프링+**댐핑**) → 절대 saturation → **`state.dt` 기반** rate limit → non-finite. non-finite 는 saturation *전에* 캡처 (∞ 마스킹 방지) | §10.5 (순서가 load-bearing) |
| **Filter** | 슬라이스 1 미사용. 활성화는 gain-ramp (§10.7). 명세는 2차 Butterworth 이나 repo `BesselFilterN` 은 4차 — 도입 시 `Init()` 전 `Apply()` 는 계수 0 (silent) 주의 | §3.3 |
| **State machine** | 축소 §10.6: `HOLDING→RUNNING→DEGRADED→SAFE_STOP`. wrench 전이는 슬라이스 2. SAFE_STOP 은 `ResetFault()` 로만 탈출, CM global E-STOP latch 와 **분리** | §10.6 |
| **Torque E-STOP** | `τ = ĝ(q) − D·q̇`, torque 한계 clamp (`compliance/torque_estop`) | E-8 승인. #184 position-slew 미복제 |
| **Mimic 정책** | "arm 서브모델에 mimic 없음" 으로 축소 (전면 거부 아님) | repo 는 mimic hand 를 실제로 다룸 (`RtModelHandle::ComputeMimicPosition`) — 명세 V10 전면거부 부적용 |
| **최소 URDF** | 6-DoF 산업용 / 7-DoF 여유 / 3-DoF 평면 configure 성공 | §11.6 T6.1 |
| **Backend scope** | **MuJoCo backend 전용** (torque). 실기 UR 은 `DeviceBackend::AcceptsCommandType` 이 `kPosition` 만 → CM 이 configure 에서 거부 | 실기 torque 는 별도 선행 설계 |

## 4. 슬라이스 1 설계 주석 (README 필수 설명)

- **A=NONE 은 열등한 fallback 이 아니다** (§6.2). 폐루프는 `Λ(q)ë + [μ+K_d]ė + K_p e = f_ext`
  로 desired inertia 가 자연 관성 `Λ(q)` 로 남고 f_ext 측정이 전혀 불필요하다. 접촉 안정성
  (passivity) 측면에서 오히려 견고하다. "F/T 센서 없으니 반쪽" 은 오해다.
- **`TRANSLATION_ONLY` 의 회전은 nullspace posture 가 담당** (§6.1). 회전은 task 로 규제되지
  않고 joint posture 로 **간접** 결정되므로, 정확한 회전 복원은 nullspace gain·여유자유도에
  의존한다 ("회전 free-float" 도 "회전 stiff" 도 아님). 그래서 `nullspace_stiffness == 0` +
  `TRANSLATION_ONLY` 는 **configure 에러** (회전 무구속 drift = 위험).
- **`J̇·v` (task-space Coriolis) 는 생략** — quasi-static 가정. 저속 compliance 에서 무시 가능하며
  provider surface 가 repo 에 없다. 고속 task 가 필요하면 별도 설계.
- **MuJoCo-only** — 위 매트릭스 Backend scope 참조. YAML `command_type: "torque"` 이외 거부.

## 5. 슬라이스 경계 (여기서 하지 않는 것)

wrench source 일체 (A≠NONE) · admittance · virtual TCP · forward-dynamics emulation ·
cascaded outer-admittance/inner-impedance (§7.6) · 실기 backend · `PublishRole` enum 추가 ·
진단 토픽의 ROS publisher wiring (컨트롤러는 controller-private `SeqLock<POD>` 에 매 tick
store 하나 LifecyclePublisher 연결은 후속). 후속 슬라이스 표는 issue #236 참조.
