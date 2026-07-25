# Compliance controller conventions (normative)

이 문서는 `rtc_controllers` 의 **compliance (impedance / admittance) 컨트롤러 계열**이
따르는 **규범 매트릭스 (normative matrix)** 다. 코드·YAML·리뷰가 참조하는 SSoT 이며,
값이 바뀌면 여기부터 갱신한다. 현재 범위는 **슬라이스 1 — `TaskImpedanceController`
(A=NONE, Jacobian-transpose)** 과 **슬라이스 2 — external wrench 입력 계약 + 조건화 체인 +
§6.3 inertia shaping** 이고, 나머지 행은 후속 슬라이스에서 채운다.

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
| **Formulation** | `formulation:` YAML 로 명시 선택. `jacobian_transpose` (§6.2, **기본값**) \| `inertia_shaping` (§6.3). "wrench 가 설정됐으니 §6.3" 같은 암묵 추론 없음 | 두 법칙은 실패 모드와 특이점 노출이 다르다 — 센서 배선의 부작용으로 제어 법칙이 바뀌면 안 된다 |
| **Pose error frame** | `SplitWorld` = `LOCAL_WORLD_ALIGNED`, `[p_d−p ; log3(R_d Rᵀ)]` | §1.3, `rtc_math/se3/pose_error.hpp` |
| **Velocity error** | `ν_d − ν`; 정적 setpoint 는 `ν_d=0` ⇒ `ė = −J·q̇` | §1.4, SplitWorld = LWA |
| **Sign** | `+K_p·e` (desired−current) | §6.2 부호 규약 MUST |
| **Selection (axis B)** | `FULL_SE3` (m=6) \| `TRANSLATION_ONLY` (m=3). `J_S = S·J`, linear rows first | §6.1 |
| **Nullspace** | dynamically-consistent `Nᵀ = I − J_Sᵀ J̄_Sᵀ`, gate **`nv > task_dim`** (NOT `nv > 6`), 토크는 `Nᵀ` 투영 (가속도는 `N`) | §6.4 — 6-DoF+`TRANSLATION_ONLY` 는 dim-3 nullspace 가 **필수** |
| **DLS (singularity)** | σ_min-adaptive: `λ²=0` (σ≥σ₀), `λ_max²(1−(σ/σ₀)²)` (σ<σ₀). OSC 의 상수 λ 와 다름 | §6.5. `singularity_threshold`=0.02, `max_damping`=0.05 |
| **Safety layer order** | joint-limit repulsive (§5.3, 스프링+**댐핑**) → 절대 saturation → **`state.dt` 기반** rate limit → non-finite. non-finite 는 saturation *전에* 캡처 (∞ 마스킹 방지) | §10.5 (순서가 load-bearing) |
| **Filter** | **`rtc_base` `BesselFilterN<3>` 2개** (force / torque 별도 cutoff). 명세의 2차 Butterworth **미도입 — deviation (D11)**: repo 필터는 4차 Bessel 이고 P5 가 동등 기능 fork 를 금지한다. §3.3 의 위상여유 우려는 admittance 루프(S3) 대상이며 impedance 에는 해당 없음. `Init()` 전 `Apply()` 는 **계수 0 을 조용히 반환**하므로 ctor·`LoadConfig` 양쪽에서 `Configure()` 하고 활성화 시 `Seed()` | §3.3, D11 |
| **State machine** | 전체 §10.6: `BIAS_CALIBRATING→HOLDING→RUNNING ⇄ RUNNING_CONTACT→DEGRADED→SAFE_STOP`. enum 의 `kRunning` **이** spec 의 `RUNNING_FREE_SPACE` 다 (contact split 이전에 명명 — 슬라이스 1 assertion 을 그대로 두기 위해 rename 하지 않음, PROC-6). SAFE_STOP 은 `ResetFault()` 로만 탈출, CM global E-STOP latch 와 **분리**하며 `BeginBiasCalibration()` 도 latch 를 이기지 못한다 | §10.6, E-8 |
| **Torque E-STOP** | `τ = ĝ(q) − D·q̇`, torque 한계 clamp (`compliance/torque_estop`) | E-8 승인. #184 position-slew 미복제 |
| **Mimic 정책** | "arm 서브모델에 mimic 없음" 으로 축소 (전면 거부 아님) | repo 는 mimic hand 를 실제로 다룸 (`RtModelHandle::ComputeMimicPosition`) — 명세 V10 전면거부 부적용 |
| **최소 URDF** | 6-DoF 산업용 / 7-DoF 여유 / 3-DoF 평면 configure 성공 | §11.6 T6.1 |
| **Backend scope** | **MuJoCo backend 전용** (torque). 실기 UR 은 `DeviceBackend::AcceptsCommandType` 이 `kPosition` 만 → CM 이 configure 에서 거부 | 실기 torque 는 별도 선행 설계 |

## 3.1 External wrench 입력 계약 (슬라이스 2, MUST)

**컨트롤러는 노드·구독·메시지 타입을 만들지 않는다.** wrench 는 비-RT setter
`SetExternalWrench(std::span<const double,6>)` 로 들어오고, RT 와의 교환은
`SeqLock<WrenchSample>` 만 쓴다 (RT-4). 누가 센서를 읽어 넣어주는가 — 전송 계층 — 은
`rtc_controllers` 범위 밖이며 wire format 도 여기서 정하지 않는다 (issue #236 D8/D10).
따라서 `geometry_msgs` 의존이 없고 `package.xml` / `CMakeLists.txt` 는 무변경이다.

| 항목 | 계약 |
|---|---|
| **Frame** | 설정된 `sensor_frame` 의 **body frame**. `sensor_frame` 미지정이면 tip frame (= tip body frame이지 LWA 가 아니다 — `Ad^{-T}` 가 여전히 `R_tip` 만큼 회전시킨다). 없는 frame 이름은 configure 에서 즉시 throw |
| **Sign** | **"환경이 로봇에 가하는 힘"이 양수** (§2 convention block 과 동일) |
| **Ordering / units** | `[f; τ] = [fx,fy,fz, tx,ty,tz]`, SI (N, N·m) |
| **전처리** | **없음 — 호출자는 센서 값을 그대로 전달한다.** bias 제거·중력보상·frame 변환·필터링은 전부 컨트롤러가 수행한다. 근거: `Ad^{-T}` 변환과 정적 중력보상은 URDF 모델을 필요로 하므로, 호출자에게 떠넘기면 호출자가 모델을 또 들어야 한다 |
| **Thread** | `SetDeviceTarget` 과 동일 — 비-RT, **단일 writer** (SeqLock 불변식) |
| **기본 비활성** | `external_wrench.enabled: false` 가 기본. 비활성이면 `SetExternalWrench` 는 조용히 drop 되고 A=NONE 경로가 그대로 유지된다 |

**조건화 체인 순서 (§3.2.1, 순서가 load-bearing)**:

```text
bias 제거 → 정적 중력보상 → deadband → saturation → filter    [전부 sensor frame]
      ↓
Ad^{-T} 로 LOCAL_WORLD_ALIGNED(tip) 변환  →  staleness fade 곱
```

- **bias 가 중력보다 먼저**: bias 는 센서 고유 오프셋, 중력항은 모델 계산값이다. 순서를
  뒤집으면 bias 가 남거나 이중 계산된다. `BIAS_CALIBRATING` 은 `raw − f_grav` 를 N 샘플
  평균하므로, 보정 자세에서 체인 출력이 정확히 0 이 된다.
- **deadband 가 saturation 보다 먼저**: 반대로 하면 deadband 가 선언된 `|w|max` 를 깎는다.
- **filter 가 마지막**: deadband 앞에 두면 deadband 경계가 필터 임펄스 응답만큼 번져
  "비접촉 ⇒ 정확히 0" 성질이 깨지고, contact FSM 이 그 성질에 의존한다.
- **deadband 는 soft shrink** `y = sign(x)·max(0,|x|−d)` — hard zeroing 은 경계에서 크기 `d`
  의 토크 불연속을 만든다 (§10.6 이 MUST 로 금지하는 바로 그것).
- **fade 는 체인 *뒤*에 곱한다**: raw 를 fade 시키면 중력보상이 `−f_grav` 를 다시 주입한다.

**bias 캘리브레이션의 두 래치 (§3.2.1 "on_activate 시 자동 1회")**: 활성화 시 두 개의
독립 상태가 선다 — `bias_done_` 는 **FSM 게이트**("BIAS_CALIBRATING 을 더 붙잡지 마라"),
`bias_pending_` 는 **남은 작업**("이 활성화에 캘리브레이션이 아직 빚으로 남아 있다"). 둘이
갈라지는 경우가 정확히 게이트를 풀어야 하지만 작업은 안 끝난 경우다:

- **producer 가 아직 아무것도 publish 하지 않음** (컨트롤러가 F/T 드라이버보다 먼저
  활성화되는 정상 cold start): 게이트는 풀되 (안 풀면 BIAS_CALIBRATING 에 영구 정착해
  이후의 모든 wrench fault 를 가린다) 작업은 유지 → 데이터가 들어오는 첫 tick 에
  BIAS_CALIBRATING 으로 재진입해 그때 평균을 낸다. 하나의 플래그로 합치면 게이트를 푸는
  순간 작업이 폐기돼 **그 활성화 동안 bias 가 영구히 0** 으로 남았다 (진단에도 흔적 없음).
- **`bias_calibration_samples: 0`**: 작업 자체가 없다 (0 bias 가 곧 캘리브레이션 결과).
  `bias_pending_` 을 세우지 않으므로 첫 샘플이 재진입에 소모되지 않는다.
- **평균 중 producer 사망** (`stale`): 게이트를 풀어 `wrench_timeout` 이
  BIAS_CALIBRATING 에 가려지지 않고 DEGRADED 로 올라가게 하되, 부분 합은 유지하고 신선한
  샘플이 돌아오면 이어서 평균한다 (재시작하면 불안정한 producer 가 bias 를 영구히 막는다).

**`bias_calibration_samples` 의 N 은 tick 이 아니라 신규 샘플 수다.** RT 루프(500–5000 Hz)
는 어떤 F/T 소스(100–1000 Hz)보다 빠르므로 tick 으로 세면 같은 측정치를 rate 비만큼
중복 누산한다 — `N: 100` 이 실제로는 20개의 서로 다른 측정치를 평균하면서 (노이즈 감소가
`√100` 이 아니라 `√20`) 도착 jitter 가 가중치로 새어 들어온다. `WrenchRead::is_new`
(generation 변화 tick) 로 게이트한다.

`diag.bias_calibrated` 가 이 구분을 외부에 노출한다 — 이 필드가 없으면 "커밋된 캘리브레이션"
과 "한 번도 안 돈 캘리브레이션" 이 밖에서 완전히 동일하게 보인다.

**정적 중력보상**: `f_g = ᵂR_Sᵀ·(m_p·ᵂg)`, `τ_g = ˢr_c × f_g`. `ᵂg` 는 **모델**(`Model::gravity`)
에서 가져오며 9.81·−Z 하드코딩 금지 — 이 패키지는 비-Z 중력 URDF 를 이미 테스트한다.
동적 항(payload 관성×가속도)은 준정적 가정으로 생략 (§12-G1).

## 3.2 Staleness 정책 (슬라이스 2, §10.6 MUST)

| 항목 | 결정 | 근거 |
|---|---|---|
| **측정 기준** | **generation counter + tick 카운팅** — `age = ticks_since_change × state.dt`. RT 에서 `steady_clock` 을 읽지 않고, producer 의 header stamp 도 신뢰하지 않는다 | 리뷰가 요구한 "ROS header time 금지 + receipt 기준" 을 RT clock 호출 없이 충족 (D9) |
| **동일 값 재전송** | **fresh 로 판정**. 값-변화 감지 방식이 아니다 | 일정한 접촉력을 유지 중인 살아있는 센서는 동일 벡터를 매 주기 재전송한다 — 값 변화로 판정하면 그 센서를 죽은 것으로 오판한다 |
| **timeout 초과** | `wrench_fadeout_time` 동안 **0 으로 선형 fade**. **마지막 값 유지 절대 금지** | §10.6 MUST — 센서가 죽은 순간의 값이 영구 인가되면 로봇이 한 방향으로 계속 밀린다 |
| **fault 등급** | `wrench_timeout` / `quality_low` 는 **DEGRADE**, critical 아님 → SAFE_STOP 으로 승격하지 않음 | §10.6: wrench 소실은 "정상과 치명적 사이의 중간 상태". 어느 한쪽으로 뭉개면 과잉 정지 또는 위험한 무시가 된다 |
| **재활성화** | `ResetTiming()` 이 누적 age 를 지운다 | 활성화가 "돌지 않던 동안 쌓인 age" 를 물려받으면 안 된다. **대가**: 컨트롤러가 비활성/E-STOP 인 동안 죽은 producer 는 제어 재개 후 `wrench_timeout` 이내에 검출된다 (그 gap 중에는 아님) |
| **contact split** | `‖f_LWA‖` 에 히스테리시스 (`contact_threshold` / `contact_release_ratio·threshold`, ratio 는 0.99 로 clamp) | §10.6 "히스테리시스 필수" — 동일 임계값은 경계에서 chatter |
| **`contact_threshold ≤ 0`** | 접촉 감지 **비활성** (`in_contact` 항상 false) | ratio clamp 는 임계값을 *스케일* 하므로 임계값 0 에서는 enter = exit = 0 이 되어 clamp 가 막으려던 bare comparator 로 정확히 퇴화한다. 게다가 모든 실측값이 0 N 을 넘으므로 센서 노이즈에 latch 되어 영구히 RUNNING_CONTACT 가 된다 — "밴드 없는 히스테리시스" 보다 "감지 off" 가 정직하다 |

## 3.3 §6.3 Inertia shaping (슬라이스 2)

$$\tau = J^\top S^\top\left[B\left(K_p S e + K_d S\dot e\right) + (B - I)\,S f^{ext}_{LWA}\right] + \tau_{null} + \hat g,\quad B = \Lambda_S\Lambda_d^{-1}$$

| 항목 | 결정 | 근거 |
|---|---|---|
| **wrench 항의 부호** | 설계 명세는 이 항을 `(B − I)(−S f_ext)` 로 쓰지만 구현은 **`+(B − I) S f_ext`** 다 | 명세의 `f_ext` 는 *로봇이 환경에 가하는* 힘이고, 본 패키지의 입력 계약(§2 · §3.1)은 그 반대인 **환경→로봇** (F/T 센서가 실제로 읽는 부호, 조건화 체인 전체가 그 위에 서 있다) 이므로 대입 시 한 번 뒤집힌다. 규약 논쟁 없이 판정되는 기준은 `Λ_d → ∞` (`B → 0`) 극한이다 — 무한히 무거운 desired inertia 는 외력에 **부동(不動)** 이어야 하므로 `Λ ν̇ = f_cmd + f_ext` 에서 `f_cmd → −f_ext` 여야 한다. 부호를 뒤집으면 `+f_ext` 가 되어 **미보정 대비 2배로 밀린다**. `HeavyDesiredInertiaCancelsTheExternalWrench` 가 이 극한을 고정 |
| **폐루프** | `Λ_d ë + K_d ė + K_p e = −f_ext` (`e = x_d − x`) | `B = I` 에서 §6.2 (`Λ_S ë + K_d ė + K_p e = −f_ext`) 와 **정확히 일치**해야 하며, 이 일관성이 위 부호를 독립적으로 재확인한다. §4 가 §6.2 폐루프를 `= f_ext` 로 적은 것은 명세의 반대 부호 규약을 따른 표기다 |
| **기본 비활성** | `formulation: jacobian_transpose` 가 기본. §6.3 은 명시 선택 | §5.2 MUST — `Λ_d ≠ Λ_S` 는 wrench 측정 오차·모델 오차에 근본적으로 민감하고 `Λ_d ≪ Λ_S` 일수록 노이즈가 증폭된다 |
| **`Λ_d`** | `desired_inertia` (6항 대각, 전부 > 0) 또는 **미지정 시 `Λ_d := Λ_S`** ("natural", `B = I`) | natural 이 A=NONE ↔ A≠NONE 경계 그 자체다. **단락(short-circuit)하지 않고** 실제 `Λ_d` Cholesky + solve 를 태워 `B` 를 항등에 수렴시키므로 T4.1 이 `if` 가 아니라 코드 경로를 검증한다 |
| **`max_inertia_ratio` clamp** | `‖B − I‖∞ > r−1` 이면 `B ← I + s(B−I)`, `s = (r−1)/‖B−I‖∞` ⇒ `‖B‖∞ ≤ r` 정확 보장. 기본 3.0 | 편차를 clamp 하면 연속이고 **`B = I` 를 절대 건드리지 않는다** — `B` 자체를 스케일하면 중립 설정을 안전 clamp 가 흔들어 T4.1 이 깨진다. 발동 시 `diag.inertia_clamped` 로 보고 (RT 로깅 금지, RT-3) |
| **`Λ_d` 인자화 실패** | `B = I` (즉 §6.2) 로 degrade + **`inertia_solve_failed`** 보고 (`inertia_clamped` 와 **별개 플래그**) | 특이 자세에서 DLS 로도 `Λ_S` 가 정부호를 잃을 수 있다. 쓰레기 shaping 행렬을 내보내는 것보다 Λ 를 아예 안 쓰는 법칙으로 후퇴하는 편이 안전. 플래그를 clamp 와 공유하면 "튜닝 bound 가 제 일을 했다" 와 "수치 붕괴" 를 operator 가 구분할 수 없고, clamp 테스트가 인자화 실패로도 green 이 된다 |
| **`Λ_S` 게이트 확장** | `M(q)`·Cholesky·`TaskDynamics::Compute` 게이트를 `nullspace_active` → **`nullspace_active \|\| inertia_shaping`**. σ_min fault (§6.5) 도 같은 게이트 | §6.3 은 여유자유도와 무관하게 `Λ_S` 를 요구한다. **PR #241 F2 의 narrowing 을 되돌리는 것이 아니라 조건을 넓히는 것** — `jacobian_transpose` 에서는 Λ 를 아예 형성하지 않으므로 F2 축소가 그대로 유효하다 |
| **특이 arm** | `Λ_S` 를 쓰는 순간 §6.5 특이점 노출이 되살아난다. rank-deficient arm 에서는 §6.3 이 SAFE_STOP 으로 latch 되고 §6.2 는 계속 제어한다 | 설계된 동작이며 테스트로 고정 (`serial_6dof` 픽스처는 6관절이 전부 +Z 동축이라 σ_min ≡ 0) |

## 3.4 §11.4.1 부호 규약 검증 — 하드웨어 도입 전 MUST

수학적 unit test 로는 잡히지 않는 **물리적 부호 오류** 절차다. 실기 F/T 센서를 처음
연결할 때 반드시 수행하고 결과를 기록한다. **P1–P3 중 하나라도 예상과 반대면 즉시 중단** —
부호 오류는 gain 을 올릴수록 발산이 빨라진다.

| # | 절차 | 기대 |
|---|---|---|
| **P1** | 로봇을 손으로 `+x` 방향으로 민다 | `SetExternalWrench` 로 들어가는 `f_x` 가 **양수** (환경이 로봇에 가하는 힘) |
| **P2** | 동일 상황, `formulation: inertia_shaping` + `Λ_d ≠ Λ_S` | impedance 컨트롤러가 `−x` 방향(밀림에 저항)으로 토크를 낸다 |
| **P3** | 동일 상황, admittance (슬라이스 3) | 로봇이 `+x` 로 **따라간다**. 저항하면 부호가 반전된 것 |
| **P4** | `sensor_frame` 을 tip 에서 멀리 떨어진 링크로 두고 TCP 에 순수 힘을 가한다 | base 관절 토크가 `(p_S − p_tip) × f` 로 예상되는 방향 (lever arm 검증) |

P4 의 시뮬레이션 대응은 `test_task_impedance_wrench.cpp`
`LeverArmProducesExpectedLwaMoment` 가 이미 고정한다 (`Ad^{-T}` vs `Ad^{T}` 검출). P1–P3 은
실기 전용이며 수치 테스트로 대체 불가.

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

admittance (`TaskAdmittance`) · virtual TCP · forward-dynamics emulation ·
cascaded outer-admittance/inner-impedance (§7.6) · 실기 backend · `PublishRole` enum 추가 ·
진단 토픽의 ROS publisher wiring (컨트롤러는 controller-private `SeqLock<POD>` 에 매 tick
store 하나 LifecyclePublisher 연결은 후속).

슬라이스 2 가 명시적으로 **하지 않은** 것:

- **전송 계층** — 누가 센서를 읽어 `SetExternalWrench` 를 호출하는가, 어떤 메시지 타입으로
  받는가. `integrated_bringup` 배선 (`RTC_REGISTER_CONTROLLER` / YAML / launch smoke) 도
  범위 밖이다 (issue #236 범위 계약 (1)).
- **`WrenchSourceBase` 추상** (D13) — ARCH-3 은 *두 번째 구체 구현* 앞에서 발화한다. 입력이
  인자가 되면 F/T · momentum observer (#135) · `PullForceEstimator` 는 전부 **같은 setter 를
  호출하는 외부 주체**일 뿐 컨트롤러 안의 구현체가 아니므로 다형 호출 지점이 없다.
  구현체 1개짜리 추상은 소비자 없는 `PublishRole` enum 과 같은 실수다. **모델을 필요로 하는
  두 번째 알고리즘이 컨트롤러 *안*에 들어올 때 ARCH-3 재발화.**
- **`CompositeWrenchSource` / fusion policy** (§3.2.4) · **`qualityScore()`** — 소스가 하나뿐이라
  중복 합산 위험이 없다. `ComplianceFaults::quality_low` 필드는 자리만 있고 현재 아무도
  세우지 않는다 (state machine 테스트로만 구속).
- **동적 payload 보상** (관성×가속도) — 준정적 가정, 정적 중력항만.

후속 슬라이스 표는 issue #236 참조.
