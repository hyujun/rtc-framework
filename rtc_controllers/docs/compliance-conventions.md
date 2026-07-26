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
| **§6.2 법칙 위치** | `Compute()` 인라인 아님 — 공용 helper `compliance/impedance_law.hpp` (`α·[K_p·e + K_d·(ν_d−ν)]`), `ν_d` 는 **명시 인자** | D18. §7.6 cascade 의 inner loop 가 같은 법칙을 `ν_d = ν_c` 로 쓴다 (복제하면 §6.2 사본이 셋). 추출은 **bitwise 무변경** — `ImpedanceLaw.MatchesThePreExtractionInlineFormBitwise` 가 추출 전 인라인 형태의 사본과 비트 단위로 대조하고, 재결합(`α·kp·e + α·kd·ė`)이 그 비교를 실제로 깨뜨림을 같은 파일이 함께 고정한다 |
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
| **재활성화** | `WrenchPipeline::ResetForActivation()` 은 `Invalidate()` — 누적 age 를 지우고 **슬롯에 있던 샘플 자체를 disown** 한다 (producer 가 *다른* generation 을 낼 때까지 "아직 아무것도 안 왔음"으로 읽힌다) | 활성화가 "돌지 않던 동안 쌓인 age" 를 물려받으면 안 된다. 그런데 `ResetTiming()` 만 하면 **죽은 producer 의 마지막 판독이 age 0 으로 부활**한다 — 제어 재개 첫 tick 에 그 힘이 fresh 로 인가되므로 §10.6 "만료된 wrench 는 ZERO, 절대 hold 아님" 의 정확한 역이다. producer 가 살아있으면 차이는 다음 publish 까지의 몇 tick 뿐이고, 그 구간은 §3.1 의 bias 이중 래치("gate 해제 후 데이터 도착")가 이미 1급 경로로 처리한다. **대가는 그대로**: 비활성/E-STOP 중 죽은 producer 는 제어 재개 후 `wrench_timeout` 이내에 검출된다 (gap 중에는 아님). `TaskImpedanceController` 도 슬라이스 4 에서 파이프라인으로 이전돼 같은 규칙을 따른다 (D22) — 그 전까지는 자체 사본에서 `ResetTiming()` 을 불렀으므로 이 결함이 shipped 상태였다. 이전의 관측 가능한 결과 하나: **활성화 직전에 발행된 샘플도 disown 된다**. 안에서는 1 µs 전 판독과 죽은 producer 의 마지막 판독을 구별할 수 없으므로 둘 다 신뢰하지 않으며, calibration 부채는 남아 다음 샘플이 BIAS_CALIBRATING 을 한 tick 늦게 재진입시킨다 (`BiasCalibrationSuppressesWrenchThenCommits` 가 이 순서를 고정). latched SAFE_STOP 은 매 held tick 재-seed 하므로 그 동안 wrench 는 계속 disown 되고, `ResetFault()` 이후 첫 신규 샘플부터 다시 유효하다 |
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
| **관절 한계** | `q_cmd` 를 `[q_min+δ, q_max−δ]` 로 clamp, **그 다음** 적분 base 기준 `max_velocity·dt` 로 rate 재바운드 (순서 반대 금지) | `safety_limiter.hpp` 는 **torque-domain** 이라 전이되지 않는다. 한계 밖 position 명령은 "부드러운 밀어냄" 이 아니라 backend 가 거부하거나 하드스톱으로 몰고 가는 요청이다. §7.3 방식 3(QP viability)은 범위 밖. **rate 재바운드가 필요한 이유**: clamp 는 받은 스텝을 *넓힐* 수 있다 — 팔이 margin 밴드 안에 있으면 IK 스텝이 아무리 작아도 clamp 가 한 tick 만에 밴드 경계까지 옮기므로 `joint_limit_margin: 0.08` 은 곧 0.08 rad 점프다. 이 컨트롤러는 `ApplySafetyLayer` 의 `max_torque_rate` 를 버리면서 대체물을 두지 않았고, 유일한 감시인 `faults.command_divergence` 는 `!integrate_from_measured` 게이트인데 그 기본값이 `true` 라 기본 모드에서 항상 꺼져 있다. 재바운드로 `q_cmd` 가 밴드 안에 일시적으로 머물 수 있는데, 밴드는 soft margin 이고 "바운드된 속도로 경계에 도달" 이 곧 그 밴드가 요구하는 동작이다. 보고되는 `target_velocities` 도 재계산해 실제 발행 위치와 일치시킨다 |
| **device state 무효** | `devices[0].valid == false` 또는 `num_channels < nv` → device 0 에 **길이 0 명령**(모든 backend 가 "업데이트 없음"으로 취급, CM `BuildHoldOutput` 과 같은 관용구) + `ComplianceFaults::device_state_invalid` → **DEGRADED**, 다음 tick 재-seed. 보조 device 는 passthrough 유지 | 제어 법칙 전체가 이 device 의 q 로 평가된다. 백엔드 첫 state 도착 전이면 읽히지 않은 채널이 기본구성 0 이라 FK/Jacobian 이 **zero configuration** 에서 평가되고 전 관절이 원점으로 명령된다 — 관여한 모든 수가 유한하므로 fault 도 안 뜬다. 모르는 관절 위치에 정직한 대체값은 없다 (0.0 은 "원점으로 가라" 다). DEGRADED 이고 critical 이 아닌 이유: 백엔드 복구가 정상 경로이고, 명령을 안 내는 축소 권한 모드가 존재한다 (발산한 명령에는 없는 선택지) |
| **E-STOP** | **position-hold**: 첫 held tick 의 측정 위치를 **latch** 하고 반복 출력 | `torque_estop.hpp` 의 `ĝ(q) − D·q̇` 는 토크 출력 전제라 부적용. 매 tick `q_meas` 를 재출력하면 hold 처럼 보이지만 backdrivable arm 이 밀릴 때 명령이 따라가 **저항이 사라진다**. CLIK 의 `ComputeEstop`(safe_position 슬루)도 쓰지 않는다 — compliance 컨트롤러가 E-STOP 순간에 조작자가 요청하지 않은 **이동을 시작**하면 안 된다 |
| **hold latch 수명** | 정상 tick 끝에서 무장 해제되고, **재활성화 / `ClearEstop()` / `ResetFault()`** 에서도 무효화된다 (`InvalidateEstopHold()`, RT thread 가 tick 머리에서 소비) | latch 는 "팔이 있던 자세" 다. 이 컨트롤러가 팔을 몰지 않는 동안 (deactivate 후 타 컨트롤러 이동, E-STOP 중 pendant jog) 그 자세는 의미를 잃고, 다음 held tick 이 **한 스텝으로** 옛 자세를 명령한다 — `ComputeEstop` 에는 슬루도 divergence 바운드도 없다. 반대로 **latched SAFE_STOP 이 강제하는 매 tick 재-seed 는 이겨야** 하므로 (그때 무효화하면 hold 가 backdriven arm 을 따라간다) `target_initialized_` 와 **별개 신호**다 |
| **hold 의 관절 한계·rate** | 출력 직전 `[q_min+δ, q_max−δ]` clamp, 이어서 **직전 발행 명령** 기준 `max_velocity·dt` rate 바운드 | 정지 시점에 이미 범위 밖이던 자세를 latch 해 영구 재명령하면 안 된다. 다만 그 교정은 **이동**이므로 이 컨트롤러의 다른 모든 이동과 같이 rate 바운드된다. 기준이 `q_meas` 가 아니라 **직전 발행 명령**인 것이 핵심 — `q_meas` 기준이면 hold 가 다시 팔을 따라가 latch 의 존재 이유가 사라진다 (정지 상태의 hold 는 명령이 변하지 않으므로 바운드가 물지 않는다) |
| **config 검증** | `ik_kp_pos`/`ik_kp_rot` 는 `load6` 와 **동형** — 3-entry sequence 가 아니면 throw (스칼라 축약 없음, D5). `pose_error_limit` 는 **양수 강제**, 위반 시 throw (`≤0 = 비활성` 관용구 미채택, D6) | `load3` 가 shape 불일치를 조용히 버려서, 헤더가 "§7.3 그대로 재현하려면 `ik_kp_*: 0`" 이라 안내하는데 `ik_kp_pos: 0.0` (스칼라) 를 쓰면 버려지고 기본 2.0 이 유지됐다 — 그 knob 이 존재하는 유일한 실험이 조용히 CLIK 변형으로 돌고 진단에도 안 보인다. 스칼라 축약을 넣으면 "조용히 다른 값" 의 변종이 남으므로 허용 안 함. `pose_error_limit` 은 매 tick **critical** fault 비교(`e.norm() > limit`)라 0/음수면 첫 tick 에 SAFE_STOP 이 latch 되고 `pose_error_exceeded` 에는 원인 필드가 없다 — 가드 자체이므로 끄는 수단을 주면 오설정이 정당한 설정처럼 보인다 |
| **`kMaxRobotDOF` 용량 체크** | **두지 않는다** (OSC `operational_space_controller.cpp` 와 동일 판단) | 이 컨트롤러는 `kMaxRobotDOF` 폭 저장소가 하나도 없다 — 작업 버퍼는 전부 nv 로 sizing 된 동적 Eigen 이고, 고정 배열 (`estop_hold_`, `TargetSlot::targets`) 은 device **채널** 인덱스라 `kMaxDeviceChannels`(64) 로 바운드된다. 게다가 체크가 생성자에서 **system 모델**로 돌았고 (`builder.GetFullModel()`), 서브모델 축소는 `LoadConfig` 의 `MaybeSelectSubModel()` 에서 일어나므로 nv=14 dual-arm URDF 가 축소되기 전에 throw → 팩토리 실패 → `on_configure` FAILURE. 같은 URDF 에서 `TaskImpedanceController` 는 정상 동작했다. 체크 보유가 정당한 곳은 실제 kMaxRobotDOF 폭 배열을 가진 CLIK / JointPD / P 컨트롤러다 |
| **wrench 필수** | `external_wrench.enabled: false` 는 **configure 에러** | §7.1 축 A≠NONE. impedance 와 대칭이 아니다: 거기서는 A=NONE 이 1급 법칙이지만, 여기서는 힘이 곧 입력이라 없으면 비싼 위치 홀드로 퇴화한다 |
| **task-space 텔레메트리** | `actual_task_positions` = 측정 TCP, `task_goal_positions` = **compliant frame** `X_c`. 둘 다 6-wide 전부 채운다 (translation + ZYX Euler RPY) | goal lane 이 `X_d` 가 아니라 `X_c` 인 이유: 실제로 명령하는 대상이 `X_c` 이고, `X_d` 만 보이면 지속적인 밀림 하에서 정지한 것처럼 보인다 — 조작자가 지켜보는 바로 그 상황이다. 0..2 만 채우면 FULL_SE3 컨트롤러인데 orientation 실험 로그가 "회전 없음" 으로 남는다 — "미측정" 과 구별 불가한 유일한 판독. 소비자(`device_state_log_pod` / `pod_fill`)는 6개를 전부 CSV 로 방출한다. 선례: `p_controller.cpp`, `operational_space_controller.cpp` |
| **축 B (selection)** | **FULL_SE3 전용** — `TaskSelection` enum 없음 | `TRANSLATION_ONLY` 이면 어떤 task 도 규제하지 않는 회전을 토크로 적분하게 된다. §6.1 이 요구하는 nullspace posture 계약과 함께 후속 슬라이스에서 도입 |
| **wrench 파이프라인** | 신규 `compliance/wrench_pipeline.hpp` 로 추출 (read → bias 이중 래치 → 조건화 → `Ad^{-T}` → fade → contact). **슬라이스 4 에서 `TaskImpedanceController` 도 이전 완료** (D22) | 조각들은 이미 분리돼 있었지만 **순서와 bias 이중 래치**는 공유되지 않았고 거기가 바로 미묘한 결함이 사는 자리다(§3.1). 슬라이스 3 은 이전을 유예했고(D16 선례) 그 사이 파이프라인만 F6(`Invalidate`)을 받아 §3.2 결함이 impedance 쪽에만 남았다 — 유예 비용이 실측된 셈이라 슬라이스 4 가 즉시 이전했다. 정상 경로(살아있는 producer)는 **byte-identical** 이 수용 조건이었고 τ·조건화 wrench·age·fade·상태 684 레코드로 확인했다 |

## 3.6 §7.6 Cascaded compliance (슬라이스 4)

명세가 "외력 추종의 **올바른** 구조" 로 지목한 형태. outer admittance 가 순응 거동을 정의하고
(`X_c`, `ν_c`), inner §6.2 impedance 가 그 프레임을 추종한다. §6.3(측정 외력을 impedance 법칙에
직접 가산)보다 안정적이라고 명세가 명시한 대안이다.

| 항목 | 결정 | 근거 |
|---|---|---|
| **wrench 소비 횟수** | **정확히 1회** — inner 는 `f_ext` 를 보지 않는다. `formulation` 축도 `inertia_shaping` 노브도 **만들지 않았다** | D19. §7.6 MUST-4 를 런타임 검사가 아니라 **타입 레벨**로 보장한다: outer 가 이미 `Ŵ_ext` 를 소비했으므로 inner 가 다시 쓰면 같은 힘을 두 번 세고 응답이 약 2배가 되는데, 모든 수가 유한하므로 fault 도 안 뜬다. 테스트는 단일가산·이중가산 두 후보 법칙을 **둘 다** 계산해 실제 출력이 전자와 일치하고 후자와 다름을 고정한다 (차이가 작으면 테스트 자체가 실패한다 — 공허한 단언 방지) |
| **inner 의 `ν_d`** | `ν_d = ν_c` (0 아님). 공용 helper `compliance/impedance_law.hpp` 가 `ν_d` 를 명시 인자로 받는 이유 | inner 는 compliant frame 의 **운동까지** 추종해야 한다. `ν_d=0` 이면 outer 가 만든 운동을 `K_d^i·ν_c` 만큼 거꾸로 감쇠한다 — cascade 의 존재 이유와 정반대 |
| **대역폭 분리 (MUST-1)** | `ω_i/ω_a` 를 **seeding tick 1회** `Λ_S(q₀)` 로 축별 계산 → 최악 축을 진단 `bandwidth_ratio` + 플래그 `bandwidth_ratio_low`. **fault 아님** | D20. 명세는 "on_configure 에서 경고" 라 하지만 configure 시점엔 `q` 가 없어 `Λ_S` 를 모른다. seeding tick 은 이미 `M(q)`·Cholesky 경로가 있고 사전할당이라 heap-free. RT 에서 `RCLCPP_*` 는 RT-3 위반이므로 로그가 아니라 플래그다. 낮은 비율은 두 게인 세트에 대한 **튜닝 진술**이지 런타임 실패가 아니므로 SAFE_STOP 으로 승격하지 않는다 (승격하면 "느린" 로봇을 세운다). `K_p^a = 0`(hand-guiding) 축은 분리할 대역 자체가 없으므로 비율에서 제외 — MUST-3 이 명시적으로 허용하는 설정을 플래그하면 플래그를 무시하도록 훈련시킨다 |
| **복귀 vs hand-guiding (MUST-3)** | `K_p^a > 0` → 외력 제거 후 `X_c → X_d`. `K_p^a = 0` → 그 자리 유지 | `AdmittanceIntegrator` 의 성질이지만 cascade 수준에서 각각 테스트한다. hand-guiding 쪽은 "되돌아오지 않음" 을 고정하되 감쇠로 인한 잔여 coast 는 허용 (스프링백만 금지) |
| **활성화 램프 α** | outer 로 들어가는 wrench 와 inner 에서 나오는 task force **양쪽**에 적용 | 각각 다른 불연속을 덮는다: 토크만 램프하면 팔이 부드러운 동안 `X_c` 가 정하중에서 멀어져 α=1 에 도달할 때 튀고, wrench 만 램프하면 inner 감쇠항 `−K_d^i·ν` 가 활성화 순간 토크 계단이 된다. 램프 구간에서 힘 구동 응답이 `α²` 인 것은 의도된 대가이며 `activation_ramp_time` 안에서만 단조 증가한다. 중력보상은 **절대 램프하지 않는다** |
| **YAML 구조** | `outer:` / `inner:` **중첩 맵** (다른 컨트롤러의 flat 스타일과 다름) | 두 루프가 **각각** stiffness 와 damping 을 가진다. flat `stiffness:` 는 이 파일에서 가장 결과가 큰 모호성이 된다 — outer 는 로봇이 힘에 얼마나 양보하는지를, inner 는 얼마나 단단히 추종하는지를 정한다 |
| **σ 특이점 fault** | `nullspace_active` 게이트에만 연동 | inner 는 Jacobian-transpose 라 `Λ` 를 형성하지 않아 task-space 특이점 노출이 없다(§6.5). seeding tick 의 `Λ_S` 계산은 **진단 목적**이므로 실패해도 대역폭 리포트만 잃고 활성화를 막지 않는다 |
| **E-STOP** | torque hold `ĝ(q) − D·q̇` (`torque_estop.hpp` 재사용) | 토크 출력이므로 `TaskAdmittanceController` 의 position-hold latch 는 부적용. latch 가 없으므로 무효화 규칙(F1)도 대응물이 없다 — held tick 이 강제하는 **재-seed** 가 compliant frame·램프·rate 이력을 함께 되돌린다 |
| **wrench 필수** | `external_wrench.enabled: false` 는 configure 에러 | outer 의 입력이 없으면 `X_c` 가 `X_d` 를 떠나지 않아 §6.2 impedance 컨트롤러와 동작이 같아진다 — 그건 이미 있고 더 잘 검증돼 있다 |
| **`x̃_nom`(nominal offset)** | **0 고정, 미구현** | `AdmittanceIntegrator` 가 `K_p x̃` 형태이고 offset 소비자가 없다. 명세 대비 의도적 미구현으로 기록 |
| **범위 밖** | arm-hand object-pull (virtual TCP = object/grasp frame) | §9 Virtual TCP 선행. `integrated_bringup` 배선(`RTC_REGISTER_CONTROLLER` / YAML / launch smoke)도 범위 계약 (1) 유지 — **따라서 S4 도 sim 에서 돌려볼 수 없고 단위 검증만 존재한다** |

## 3.7 F5 device-validity 게이트 · 판독 불가 시의 출력 (#236 E-8)

`ControllerState::devices[0]` 이 이번 tick 에 쓸 수 없는 상태 —`valid == false` 또는
`num_channels < nv`— 일 때 세 compliance 컨트롤러가 무엇을 내보내는가. 세 곳에 흩어지면
서로 다른 답을 내므로 여기가 SSoT 다.

| 항목 | 결정 | 근거 |
|---|---|---|
| **게이트 위치** | `Compute()` 의 조인트 상태 복사 **직전** — 세 컨트롤러 모두. `ComputeEstop()` 의 `ĝ(q)` 계산 직전 — **토크 도메인 둘** (`TaskImpedanceController`, `CascadedComplianceController`) | 미보고 채널은 `0` 으로 읽히므로 게이트 없이는 FK·Jacobian·법칙 전체가 **ZERO configuration** 에서 돌고 전 관절을 원점으로 당기는 토크가 나간다. 모든 수가 유한해 CM 의 actuator-boundary validator 도 거르지 않는다. `TaskImpedanceController` 는 `Compute()` 게이트조차 없이 출하됐었다 (#236 E-8 에서 신설) |
| **미적용 — `TaskAdmittanceController::ComputeEstop`** | position-hold latch 를 `dev0.positions` 에서 seed 하는데 **게이트가 없다**. 이번 범위에서 손대지 않음 | 판독 불가 tick 에 latch 되면 미보고 채널의 hold 가 `0` 으로 굳어 "원점으로 servo" 가 되고, `estop_hold_valid_` 가 latch 라 이후 tick 까지 남는다. 다만 이 경로는 global E-STOP 하에서만 도달하고 그때 CM 이 `BuildHoldOutput` 으로 출력을 치환하므로 현재 하드웨어에 도달하지 않는다. position 도메인 hold 의 무효화 규칙(F1)과 얽히므로 별도 E-8 판단 사안으로 남긴다 |
| **출력** | **zero-length** (`devices[0].num_channels = 0`) — 값이 아니라 "이번 tick 은 갱신 없음". secondary device passthrough 는 유지 | 알 수 없는 관절 위치의 정직한 대체값은 없다. CM 자신의 `BuildHoldOutput` 도 같은 idiom 을 쓴다. 팔 상태가 사라졌다고 손이 명령 불가가 되지는 않으므로 secondary 는 통과시킨다 |
| **`ComputeEstop` 도 동일** | 같은 zero-length. **`nc0` 길이의 0 커맨드를 내보내지 않는다** | 둘은 반대다: zero-length 는 전 백엔드가 early-return 하는 **침묵**(드라이브는 직전 setpoint 유지), `nc0` 길이의 0 은 **진짜 0 N·m** 이고 토크 모드 팔에서는 정지가 아니라 **낙하**다. `ĝ(q) − D·q̇` 는 q·q̇ 가 실측일 때만 hold 이므로 판독 불가 상태에서는 hold 자체가 성립하지 않는다 |
| **fault 등급** | `device_state_invalid` 는 **DEGRADE**, critical 아님 → SAFE_STOP 승격 없음 | 백엔드 복구가 정상 경로이고, "명령을 내지 않는다" 는 축소된 권한의 답이 존재한다 (발산한 명령과 달리) |
| **시간 상한 없음** | 지속되는 판독 불가에 타이머를 두지 **않는다**. DEGRADED 에 머문 채 ride-through | #236 E-8 에서 (a)상한+에스컬레이션 / (b)CM 이 컨트롤러발 zero-length 에도 `WriteSafeCommand()` 를 검토하고 **둘 다 기각**했다. 아래 참조 |

**왜 타이머를 두지 않는가 (실측 근거)**

- 현실적 실패인 **런중 백엔드 dropout 은 이 게이트에 도달하지 않는다.** 출하된 백엔드에서
  `valid` 는 첫 state 메시지에 `true` 로 서고 **다시 내려가지 않는다** (`joint_state_reorder.hpp`).
  dropout 은 플래그가 아니라 **stamp** 를 멈추므로, CM 의 50 Hz device watchdog 이
  `device_timeout` (실기 1000 ms) 후 **global E-STOP** 을 걸고 그때부터 CM 이 `BuildHoldOutput`
  으로 출력을 전량 치환한다. 즉 ride-through 는 이미 bounded 다.
- 게이트에 실제로 도달하는 것은 **`num_channels < nv`, 곧 기동시 설정 불일치**다. CM 의
  startup gate 가 모든 device 가 보고하기 전엔 `Compute()` 를 돌리지 않으므로 `!valid` 는
  백엔드가 아예 없는 그룹에서만 나오고 그건 init timeout 이 잡는다. 설정 불일치 시점에는
  백엔드가 아직 아무것도 publish 하지 않았으므로 **zero-length = 드라이브가 명령된 적 없음**
  이지 "직전 토크 유지" 가 아니다. 여기에 타이머를 걸면 *잘못 설정된 모든 기동이 N 초 뒤
  팔을 떨어뜨리는* 동작이 된다.
- (b) 는 **오늘 물리적으로 no-op** 이다. 출하된 세 백엔드의 `WriteSafeCommand()` 는 전부
  "마지막으로 나간 명령 재발행" 이라 침묵과 물리 결과가 같다 (`device_backend.hpp` 가 명시).
  토크 유지 시간을 줄이지 못하면서 #198 Phase 4 계약만 바꾼다.

남은 실제 구멍은 토크 정책이 아니라 **진단**이다 — `num_channels < nv` 는 낫지 않는 영구
DEGRADED 인데 이를 알리는 경로가 없다 (issue #261).

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
- **CLIK 을 `differential_ik.hpp` 로 마이그레이션** — helper 추출과는 회귀 표면이 다른 별건
  (D16 선례, 이슈 #258). 같이 유예했던 **`TaskImpedanceController` → `wrench_pipeline.hpp`**
  는 슬라이스 4 가 흡수했다 (D22).

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
