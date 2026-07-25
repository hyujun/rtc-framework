# Compliance controller conventions (normative)

이 문서는 `rtc_controllers` 의 **compliance (impedance / admittance) 컨트롤러 계열**이
따르는 **규범 매트릭스 (normative matrix)** 다. 코드·YAML·리뷰가 참조하는 SSoT 이며,
값이 바뀌면 여기부터 갱신한다. 현재 범위는 **슬라이스 1 — `TaskImpedanceController`
(A=NONE, Jacobian-transpose)**, **슬라이스 2 — external wrench 입력 계약 + 조건화 체인 +
§6.3 inertia shaping**, **슬라이스 3 — `TaskAdmittanceController` (§7, 힘 입력 → 위치 출력,
§3.5)** 이고, 나머지 행은 후속 슬라이스에서 채운다.

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
| **재활성화** | `WrenchPipeline::ResetForActivation()` 은 `Invalidate()` — 누적 age 를 지우고 **슬롯에 있던 샘플 자체를 disown** 한다 (producer 가 *다른* generation 을 낼 때까지 "아직 아무것도 안 왔음"으로 읽힌다) | 활성화가 "돌지 않던 동안 쌓인 age" 를 물려받으면 안 된다. 그런데 `ResetTiming()` 만 하면 **죽은 producer 의 마지막 판독이 age 0 으로 부활**한다 — 제어 재개 첫 tick 에 그 힘이 fresh 로 인가되므로 §10.6 "만료된 wrench 는 ZERO, 절대 hold 아님" 의 정확한 역이다. producer 가 살아있으면 차이는 다음 publish 까지의 몇 tick 뿐이고, 그 구간은 §3.1 의 bias 이중 래치("gate 해제 후 데이터 도착")가 이미 1급 경로로 처리한다. **대가는 그대로**: 비활성/E-STOP 중 죽은 producer 는 제어 재개 후 `wrench_timeout` 이내에 검출된다 (gap 중에는 아님). `TaskImpedanceController` 는 자체 `WrenchInput` 사본을 쓰며 아직 `ResetTiming()` (파이프라인 이전과 함께 후속) |
| **비유한 샘플** | `WrenchInput::Set()` 진입점에서 6성분 `isfinite` 전수 검사, 하나라도 걸리면 **샘플 전체 드롭** + `rejected_samples()` 카운트 (진단 `wrench_rejected`) | 하위 어디도 이걸 못 거른다: conditioner 의 deadband·saturation 은 전부 비교연산이고 NaN 과의 비교는 모두 false 다. 그래서 garbage 1패킷이 compliant frame 까지 가서 `nan_inf` → **SAFE_STOP latch**, 그런데 그 latch 는 `ClearEstop()` 이 안 풀고 `~/reset_fault` 배선은 아직 없다 = 프로세스 재시작. 드롭하면 직전 샘플이 §10.6 정상 경로(fade → ZERO → DEGRADED)로 만료되며, 그게 garbage 를 내보내는 센서의 올바른 표현이다. 부분 채택(유한 성분만 반영)은 하지 않는다 — 조용히 다른 값이 된다 |
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

## 3.5 §7 Task admittance (슬라이스 3)

$$\Lambda_d\,\ddot{\tilde x}_c + K_d\,\dot{\tilde x}_c + K_p\,\tilde x_c = S f^{ext}_{LWA},\qquad
\dot q = J^{+}\left(\nu_c + K^{ik} e(X, X_c)\right) + (I - J^{+}J)\,\dot q_{null}$$

| 항목 | 결정 | 근거 |
|---|---|---|
| **출력** | `CommandType::kPosition` — 내부 적분 `q_cmd = q_meas + q̇·Δt`. `kVelocity` **신설 안 함** | D14. §7.3 방식 1 의 출력 자체가 position 이다. `kVelocity` 는 `rtc_base` enum + `-Wswitch` 강제 `CommandTypeToString` + backend `AcceptsCommandType` + sim 의 **별도** actuator enum 까지 3–4 패키지에 번지고 PROC-3 를 유발한다. position 출력이라 **현재 유일하게 position-only 실기 backend 가 받을 수 있는 compliance 경로** (D15) |
| **`x̃_c` 방향** | `x̃_c = e(X_d, X_c)` = **desired → compliant**. §1.3 pose error (`current → desired`) 와 **반대** | §7.2 명시. 슬라이스 2 에서 §6.3 부호 반전 결함(`da057c4`)이 실제로 난 지점이라 표현을 코드 주석·이 표 양쪽에 박는다. IK 의 `e(X, X_c)` 는 §1.3 방향(현재→compliant)이며 두 방향이 한 함수 안에 공존한다 |
| **회전 표현** | 편차의 회전 성분을 **`R̃ = R_c R_dᵀ` 행렬로 저장**하고 매 스텝 `R̃ ← exp3(ω·dt)·R̃` 로 retract. `x̃_rot = log3(R̃)` 는 매 tick 파생 | §7.2 MUST. 접공간 누적(`x_rot += ω dt`)은 비동축 회전에서 합성의 rotation vector 가 아니므로 드리프트한다 — 테스트가 naive 합과의 차이 + `‖R̃ᵀR̃−I‖<1e-10` 를 동시에 고정 |
| **`exp3` 곱하는 쪽** | **왼쪽**(`exp3(ω dt)·R̃`). 명세 스니펫의 `R_c = R_c·exp3(ω dt)` 는 오른쪽 | 스니펫은 ω 가 **body** frame 임을 가정한다. 이 패키지의 Jacobian·wrench·`K_p` 축은 전부 LWA 이므로 ω 는 **world** 각속도이고 `Ṙ_c = [ω]×R_c ⇒ Ṙ̃ = [ω]×R̃` 다 |
| **적분** | **semi-implicit(symplectic) Euler — 속도 먼저** | §7.2 MUST. explicit Euler 는 진동계에 에너지를 주입한다. 테스트는 무감쇠 자유진동에서 `E` 비증가를 걸고, **같은 파일에 explicit Euler 참조 구현을 두어 그것이 발산함을 함께 고정**한다 (안 그러면 "에너지가 안 늘었다" 가 공허하다, §11.5 T5.2) |
| **§7.5 변위 제한** | clamp 아님 — **saturating spring + 바깥 방향 반경 속도 제거**. 반경 속도는 `‖x̃ + ẋ̃·dt‖ = d_max` 가 되도록 **풀어서** 넣는다 | 단순 clamp 는 속도 불연속(§7.5 MUST). 바깥 반경 속도를 0 으로만 만들면 접선 이동이 매 tick `(v·dt)²/2d` 씩 새어 4000 step 에 `1e-5 m` 를 넘어간다 — 반경 성분만 풀어 넣으면 접선 성분은 그대로 남아 "벽을 따라 미끄러짐" 이 보존되면서 경계는 정확해진다 |
| **§7.5 속도 제한** | 선/각속도 **별도**, **norm 기준**(방향 보존) | 변위 제한만으로는 큰 임펄스의 순간 속도를 못 막는다. 성분별 clamp 는 대각 방향 밀림을 축 정렬로 **조용히 회전**시킨다 |
| **§7.5 복귀 속도 상한** | `max_return_{linear,angular}_velocity` — 속도 가드와 **별개 키**, `≤0` 이 off 가 아니라 **사용 시점 floor** (`kMinReturnVelocity*`, NUM-1) | 이미 경계 밖인 상태(런타임 `max_displacement` 축소, 반올림)를 되돌리는 **안쪽 반경 속도**의 상한이다. 이걸 속도 가드(`max_compliant_*_velocity`)와 공유하면, 그 가드의 문서화된 `≤0 = off` 관용구가 **상한이 가장 필요한 바로 그 설정**에서 상한을 지워 `−δ/dt` 임펄스(예: `(0.10−0.20)/0.002 = −50 m/s`)를 IK feedforward 로 흘린다. `≤0` 을 off 로 해석하지 않는 이유는 그러면 반경 속도가 0 에 고정돼 프레임이 경계 밖에 **영구히 갇히기** 때문 — 가드의 목적과 정반대다 |
| **§7.5 합성 상한** | 접선+반경 합성 후 `max(v_return_max, ‖u_in‖)` 로 norm clamp | 두 반쪽이 독립적으로 바운드되므로 합성하면 `√(v_max² + v_return²) = √2·v_max` 가 나오고, 그 벡터가 그대로 `ν_c` = IK feedforward twist 다. 상한을 `v_return_max` 단독이 아니라 `‖u_in‖` 과의 max 로 두는 이유: 가드는 경계 밖 상태를 되돌리는 **유일한 구동원**이라 `v_return_max` 까지 *올릴* 권한이 있어야 하고(그렇지 않으면 들어온 속도가 작을 때 자기 복구가 사라진다), 동시에 받은 것보다 큰 값을 돌려주면 안 된다 |
| **§7.4 `min_desired_inertia`** | 하한을 **사용 시점**(`Step`/`Energy`)에서 강제. 기본 translation 2.0 kg / rotation 0.05 kg·m² | 접촉 안정성은 `Λ_d` 가 작을수록 나빠진다(Colgate/Hogan 계열). `set_gains` 는 `LoadConfig` 를 우회하므로 configure 에서만 clamp 하면 구멍이 남는다. `Λ_d` 는 매 tick 역수를 취하므로 NUM-2 가드이기도 하다 |
| **IK** | 신규 공용 helper `compliance/differential_ik.hpp` (DLS + **속도공간** `N = I − J⁺J`). §6.5 σ_min-adaptive λ 규칙은 `task_dynamics.hpp` 것을 **호출**(복제 아님) | D16. CLIK 의 DLS 는 인라인 상수-λ 라 재사용 불가하고 복제하면 세 번째 변형(P5 저촉). `task_dynamics` 는 `M` 이 필요한 **동역학 일관** 역행렬이라 position 출력 컨트롤러에는 부적합. 토크는 `Nᵀ`, 속도는 `N` (§6.4) |
| **폐루프 IK 항 (명세 갭)** | §7.3 공식은 순수 feedforward(`ν = ν_c`)지만 구현은 `ν = ν_c + K^ik·e(X, X_c)` 를 쓴다 (`ik_kp_*`, 0 으로 두면 명세와 동일) | feedforward 만으로는 하위 제어기 추종 오차가 **영구히 보정되지 않아** 실제 TCP 가 `X_c` 에서 멀어진다. §12 "구현 시 발견하면 보고" 대상 갭으로 기록 |
| **`integrate_from_measured`** | 양방향 구현, 기본 `true`. `false` 일 때만 `‖q_cmd−q_meas‖ > command_divergence_limit` → **critical** (`ComplianceFaults::command_divergence` → SAFE_STOP) | §7.3 MUST. `true` 는 매 tick 재anchor 되므로 windup 자체가 불가능하다. critical 인 이유: 명령이 이미 팔에서 벗어났으므로 계속 돌수록 간격이 벌어지고, wrench 소실과 달리 **후퇴할 축소 권한 모드가 없다** |
| **관절 한계** | `q_cmd` 를 `[q_min+δ, q_max−δ]` 로 clamp | `safety_limiter.hpp` 는 **torque-domain** 이라 전이되지 않는다. 한계 밖 position 명령은 "부드러운 밀어냄" 이 아니라 backend 가 거부하거나 하드스톱으로 몰고 가는 요청이다. §7.3 방식 3(QP viability)은 범위 밖 |
| **E-STOP** | **position-hold**: 첫 held tick 의 측정 위치를 **latch** 하고 반복 출력 | `torque_estop.hpp` 의 `ĝ(q) − D·q̇` 는 토크 출력 전제라 부적용. 매 tick `q_meas` 를 재출력하면 hold 처럼 보이지만 backdrivable arm 이 밀릴 때 명령이 따라가 **저항이 사라진다**. CLIK 의 `ComputeEstop`(safe_position 슬루)도 쓰지 않는다 — compliance 컨트롤러가 E-STOP 순간에 조작자가 요청하지 않은 **이동을 시작**하면 안 된다 |
| **hold latch 수명** | 정상 tick 끝에서 무장 해제되고, **재활성화 / `ClearEstop()` / `ResetFault()`** 에서도 무효화된다 (`InvalidateEstopHold()`, RT thread 가 tick 머리에서 소비) | latch 는 "팔이 있던 자세" 다. 이 컨트롤러가 팔을 몰지 않는 동안 (deactivate 후 타 컨트롤러 이동, E-STOP 중 pendant jog) 그 자세는 의미를 잃고, 다음 held tick 이 **한 스텝으로** 옛 자세를 명령한다 — `ComputeEstop` 에는 슬루도 divergence 바운드도 없다. 반대로 **latched SAFE_STOP 이 강제하는 매 tick 재-seed 는 이겨야** 하므로 (그때 무효화하면 hold 가 backdriven arm 을 따라간다) `target_initialized_` 와 **별개 신호**다 |
| **hold 의 관절 한계·rate** | 출력 직전 `[q_min+δ, q_max−δ]` clamp, 이어서 **직전 발행 명령** 기준 `max_velocity·dt` rate 바운드 | 정지 시점에 이미 범위 밖이던 자세를 latch 해 영구 재명령하면 안 된다. 다만 그 교정은 **이동**이므로 이 컨트롤러의 다른 모든 이동과 같이 rate 바운드된다. 기준이 `q_meas` 가 아니라 **직전 발행 명령**인 것이 핵심 — `q_meas` 기준이면 hold 가 다시 팔을 따라가 latch 의 존재 이유가 사라진다 (정지 상태의 hold 는 명령이 변하지 않으므로 바운드가 물지 않는다) |
| **wrench 필수** | `external_wrench.enabled: false` 는 **configure 에러** | §7.1 축 A≠NONE. impedance 와 대칭이 아니다: 거기서는 A=NONE 이 1급 법칙이지만, 여기서는 힘이 곧 입력이라 없으면 비싼 위치 홀드로 퇴화한다 |
| **축 B (selection)** | **FULL_SE3 전용** — `TaskSelection` enum 없음 | `TRANSLATION_ONLY` 이면 어떤 task 도 규제하지 않는 회전을 토크로 적분하게 된다. §6.1 이 요구하는 nullspace posture 계약과 함께 후속 슬라이스에서 도입 |
| **wrench 파이프라인** | 신규 `compliance/wrench_pipeline.hpp` 로 추출 (read → bias 이중 래치 → 조건화 → `Ad^{-T}` → fade → contact) | 조각들은 이미 분리돼 있었지만 **순서와 bias 이중 래치**는 공유되지 않았고 거기가 바로 미묘한 결함이 사는 자리다(§3.1). `TaskImpedanceController` 마이그레이션은 **유예** — helper 추출과 shipped 컨트롤러 이전은 회귀 표면이 다르다 (`task_dynamics.hpp` 가 OSC 를 안 건드린 D16 선례) |

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

virtual TCP · forward-dynamics emulation · cascaded outer-admittance/inner-impedance (§7.6) ·
실기 backend · `PublishRole` enum 추가 · 진단 토픽의 ROS publisher wiring (컨트롤러는
controller-private `SeqLock<POD>` 에 매 tick store 하나 LifecyclePublisher 연결은 후속).

슬라이스 3 (`TaskAdmittanceController`) 이 명시적으로 **하지 않은** 것:

- **`integrated_bringup` 배선 일체** (`RTC_REGISTER_CONTROLLER` / YAML / launch smoke) —
  슬라이스 2 와 같은 범위 계약 (issue #236 범위 계약 (1)).
- **`CommandType::kVelocity` 신설** (D14 로 기각 — 별도 이슈 후보).
- **축 B `TRANSLATION_ONLY`** — §3.5 참조. `TaskSelection` enum 자체를 만들지 않았다.
- **S4 cascaded** (§7.6) · **JointAdmittance** (§5.4) · 동적 payload 보상 ·
  `CompositeWrenchSource`/fusion (§3.2.4).
- **CLIK 을 `differential_ik.hpp` 로 마이그레이션** · **`TaskImpedanceController` 를
  `wrench_pipeline.hpp` 로 마이그레이션** — 둘 다 helper 추출과는 회귀 표면이 다른 별건
  (D16 선례). 후자를 할 때 두 경로가 **byte-identical** wrench 를 내는지가 수용 조건이다.

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
