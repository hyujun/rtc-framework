# Compliance controller conventions (normative)

이 문서는 `rtc_controllers` 의 **compliance (impedance / admittance) 컨트롤러 계열**이
따르는 **규범 매트릭스 (normative matrix)** 다. 코드·YAML·리뷰가 참조하는 SSoT 이며,
값이 바뀌면 여기부터 갱신한다. 현재 범위는 **슬라이스 1 — 태스크 임피던스
(A=NONE, Jacobian-transpose)**, **슬라이스 2 — external wrench 입력 계약 + 조건화 체인 +
§6.3 inertia shaping**, **슬라이스 3 — 태스크 어드미턴스 (§7, 힘 입력 → 위치 출력,
§3.5)**, **슬라이스 4 — §7.6 캐스케이드 컴플라이언스** 이고, 나머지 행은 후속
슬라이스에서 채운다 — 잔여 행의 목록·판정은 issue #316.

**#298 S7c-2 이후 읽는 법.** 이 문서가 쓰일 당시 세 계열은 `rtc_controllers` 안의
어댑터 클래스 (`TaskImpedanceController` / `TaskAdmittanceController` /
`CascadedComplianceController`) 였고, 그 클래스들은 삭제됐다. 남은 것은 `compliance/`
· `task/` · `joint/` 의 법칙 헤더와 `params/` 스키마이며, tick 배선·E-STOP·게이트는
이제 **바인딩이 소유**한다. 따라서 아래 행들은 두 종류로 읽는다 — 법칙·스키마의
성질을 규정하는 행은 여전히 코드에 대한 규범이고, tick 배선을 규정하는 행은
**바인딩에 대한 요구사항**이다 (해당 행에 그렇게 표시했다). 삭제된 클래스명이
남아 있는 곳은 그 결정이 어디서 관측됐는지를 가리키는 **이력 참조**이지 현행 구현이
아니다.

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
| **DLS (singularity)** | σ_min-adaptive: `λ²=0` (σ≥σ₀), `λ_max²(1−(σ/σ₀)²)` (σ<σ₀). **저장소 단일 규약** — OSC·CLIK 의 상수 λ 는 #236 S2b+S3b 에서 여기로 수렴했다 (아웃라이어였던 쪽이 흡수된 것이지 새 규약이 아니다) | §6.5. `singularity_threshold`=0.02, `max_damping`=0.05 — 다섯 컨트롤러 전부 같은 키·같은 기본값 |
| **Safety layer order** | joint-limit repulsive (§5.3, 스프링+**댐핑**) → 절대 saturation → **`state.dt` 기반** rate limit → non-finite. non-finite 는 saturation *전에* 캡처 (∞ 마스킹 방지) | §10.5 (순서가 load-bearing) |
| **Filter** | **`rtc_base` `BesselFilterN<3>` 2개** (force / torque 별도 cutoff). 명세의 2차 Butterworth **미도입 — deviation (D11)**: repo 필터는 4차 Bessel 이고 P5 가 동등 기능 fork 를 금지한다. §3.3 의 위상여유 우려는 admittance 루프(S3) 대상이며 impedance 에는 해당 없음. `Init()` 전 `Apply()` 는 **계수 0 을 조용히 반환**하므로 ctor·`LoadConfig` 양쪽에서 `Configure()` 하고 활성화 시 `Seed()` | §3.3, D11 |
| **State machine** | 전체 §10.6: `BIAS_CALIBRATING→HOLDING→RUNNING ⇄ RUNNING_CONTACT→DEGRADED→SAFE_STOP`. enum 의 `kRunning` **이** spec 의 `RUNNING_FREE_SPACE` 다 (contact split 이전에 명명 — 슬라이스 1 assertion 을 그대로 두기 위해 rename 하지 않음, PROC-6). SAFE_STOP 은 `ResetFault()` 로만 탈출, CM global E-STOP latch 와 **분리**하며 `BeginBiasCalibration()` 도 latch 를 이기지 못한다. 외부 진입점은 `/rtc_cm/reset_fault` (#260) — 아래 행 | §10.6, E-8 |
| **fault 복구 진입점** (#260) | `ResetFault()` / `HasLatchedFault()` 는 `RTControllerInterface` 의 virtual 이고, CM 서비스 `/rtc_cm/reset_fault` 가 그것을 부른다. **active 컨트롤러 한정 + `controller_name` 명시 필수**, wildcard 없음. 요청은 atomic flag 이고 **`Compute()` 머리**에서 소비된다 (E-STOP early-return **앞**). `ResetTargetInitialization()` 은 미소비 요청을 **버린다**. `ComplianceStateMachine::ResetFault()` 자체는 **`SAFE_STOP` 이 아니면 no-op** 이다 — `BeginBiasCalibration()` 과 같은 가드 형태 | 컨트롤러는 여전히 노드·서비스를 만들지 않는다 (범위 계약 2) — 진입점은 `SetExternalWrench` 와 같은 비-RT setter 이고 서비스 배선은 CM 몫이다. 이름 명시가 오퍼레이터 확인 단계다. 소비 지점이 E-STOP 분기 앞인 덕분에 **global E-STOP 중에도 controller-local fault 를 독립적으로 풀 수 있다** — 그게 E-8 분리의 실제 의미다. 활성화 경계에서 요청을 버리는 이유는 `BeginBiasCalibration()` 이 latch 를 이기지 못하게 한 이유와 같다: 비활성 컨트롤러에 남은 요청이 다음 활성화 첫 tick 에 소비되면, 아무도 그 시점에 재승인하지 않은 fault 가 풀린다 | §10.6, E-8 |
| **Torque E-STOP** | `τ = ĝ(q) − D·q̇`, torque 한계 clamp (`compliance/torque_estop`) | E-8 승인. #184 position-slew 미복제 |
| **task-space 텔레메트리** | `actual_task_positions` = 측정 TCP, `task_goal_positions` = **`X_d`** (`goal_pose_`, = 드레인된 target slot). 둘 다 6-wide 전부 채운다 (translation + ZYX Euler RPY). E-STOP·hold tick 은 두 레인을 0 으로 둔다 (형제 컨트롤러와 동일 — 바꾸려면 전 컨트롤러 동시에) | goal lane 은 **실제로 명령하는 대상**을 보여야 한다. admittance/cascade 가 `X_c` 를 넣는 이유가 그것이고 (§3.5), 이 컨트롤러에는 `X_c` 가 없다 — §6.2/§6.3 이 setpoint 자체를 향해 규제하므로 `X_d` 가 그 대상이다. 0..2 만 채우면 orientation 실험 로그가 "회전 없음" 으로 남아 **"미측정" 과 구별 불가**하고, 레인 전체가 0 인 채 `goal_type = kTask` 를 주장하면 "월드 원점·단위 자세를 명령 중" 으로 읽힌다 (CM 은 이 레인을 대신 채우지 않는다). 소비자 `device_state_log_pod` / `pod_fill` 는 6개를 전부 CSV 로 방출한다. `TRANSLATION_ONLY` 에서도 회전을 보고한다 — `m_` 은 컨트롤러 설정이지 보고되는 goal 의 성질이 아니다 (#262) |
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
| **비유한 샘플** | `WrenchInput::Set()` 진입점에서 6성분 `isfinite` 전수 검사, 하나라도 걸리면 **샘플 전체 드롭** + `rejected_samples()` 카운트 (진단 `wrench_rejected`) | 하위 어디도 이걸 못 거른다: conditioner 의 deadband·saturation 은 전부 비교연산이고 NaN 과의 비교는 모두 false 다. 그래서 garbage 1패킷이 compliant frame 까지 가서 `nan_inf` → **SAFE_STOP latch**, 그런데 그 latch 는 `ClearEstop()` 이 안 푼다 (#260 이전에는 배선 자체가 없어 프로세스 재시작이 유일한 복구였고, 지금은 `/rtc_cm/reset_fault` 가 그 경로다 — 다만 **유입을 막는 것이 여전히 먼저**다). 드롭하면 직전 샘플이 §10.6 정상 경로(fade → ZERO → DEGRADED)로 만료되며, 그게 garbage 를 내보내는 센서의 올바른 표현이다. 부분 채택(유한 성분만 반영)은 하지 않는다 — 조용히 다른 값이 된다 |
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
| **§6.3 법칙 위치** | `Compute()` 인라인 아님 — 공용 helper `compliance/inertia_shaping.hpp` (`ComputeShapedTaskForce`), `Λ_S` 는 **명시 인자** | #236 S4. §6.2 가 `impedance_law.hpp` 로 나간 것과 같은 경계이며, `Λ_S` 를 인자로 받는 이유는 `TaskDynamics` 를 이름으로 아는 법칙이 그 헬퍼의 **수렴점 판정**(OSC 인라인 Λ 블록 흡수 = S2b)을 선점하기 때문이다. 추출은 **bitwise 무변경** — `InertiaShaping.MatchesThePreExtractionInlineFormBitwise` 가 추출 전 인라인 형태(멤버 `MatrixXd` 스크래치 그대로)의 사본과 비트 대조하고, 대수적으로 동치인 재결합(역행렬 materialise)이 그 비교를 실제로 깨뜨림을 같은 파일이 함께 고정한다. 스크래치는 max-size `Matrix<double,Dyn,Dyn,0,6,6>` (D-S4a): 순수 고정 6×6 은 `TRANSLATION_ONLY`(m=3)에서 LLT 의 크기 assert 에 걸리는데, 그 assert 는 **Release(`NDEBUG`, 이 저장소의 기본 빌드)에서 컴파일아웃**되므로 abort 는 운 좋은 경우고 실제로는 쓰이지 않은 행 위에서 solve 가 도는 **무증상 오답**이 된다. 같은 이유로 `Λ_S` 가 `m×m` 보다 작은 경우를 **런타임 검사**로 잡아 `solve_failed` 로 강등한다 — `eigen_assert` 는 라이브러리 TU 에서 센서가 될 수 없다. `Λ_d`·`max_inertia_ratio` 는 `Gains` 가 `compliance::InertiaShapingParams` 를 **그대로 보유**하므로 (admittance 의 `AdmittanceParams` 와 같은 형태) 기본값 정의가 한 곳이다 |
| **`max_inertia_ratio` clamp** | `‖B − I‖∞ > r−1` 이면 `B ← I + s(B−I)`, `s = (r−1)/‖B−I‖∞` ⇒ `‖B‖∞ ≤ r` 정확 보장. 기본 3.0 | 편차를 clamp 하면 연속이고 **`B = I` 를 절대 건드리지 않는다** — `B` 자체를 스케일하면 중립 설정을 안전 clamp 가 흔들어 T4.1 이 깨진다. 발동 시 `diag.inertia_clamped` 로 보고 (RT 로깅 금지, RT-3) |
| **`Λ_d` 인자화 실패** | `B = I` (즉 §6.2) 로 degrade + **`inertia_solve_failed`** 보고 (`inertia_clamped` 와 **별개 플래그**) | 특이 자세에서 DLS 로도 `Λ_S` 가 정부호를 잃을 수 있다. 쓰레기 shaping 행렬을 내보내는 것보다 Λ 를 아예 안 쓰는 법칙으로 후퇴하는 편이 안전. 플래그를 clamp 와 공유하면 "튜닝 bound 가 제 일을 했다" 와 "수치 붕괴" 를 operator 가 구분할 수 없고, clamp 테스트가 인자화 실패로도 green 이 된다 |
| **`Λ_S` 게이트 확장** | `M(q)`·Cholesky·`TaskDynamics::Compute` 게이트를 `nullspace_active` → **`nullspace_active \|\| inertia_shaping`**. σ_min fault (§6.5) 도 같은 게이트 | §6.3 은 여유자유도와 무관하게 `Λ_S` 를 요구한다. **PR #241 F2 의 narrowing 을 되돌리는 것이 아니라 조건을 넓히는 것** — `jacobian_transpose` 에서는 Λ 를 아예 형성하지 않으므로 F2 축소가 그대로 유효하다 |
| **특이 arm** | `Λ_S` 를 쓰는 순간 §6.5 특이점 노출이 되살아난다. rank-deficient arm 에서는 §6.3 이 SAFE_STOP 으로 latch 되고 §6.2 는 계속 제어한다 | 설계된 동작이며 테스트로 고정 (`serial_6dof` 픽스처는 6관절이 전부 +Z 동축이라 σ_min ≡ 0). **같은 동축성이 `ĝ(q) ≡ 0` 도 만든다** — 중력이 −Z 인데 모든 관절축이 +Z 라 중력 토크의 축 성분이 정확히 0 이다. 따라서 이 픽스처 위에서는 **중력 기반 hazard 단언이 게이트를 지워도 통과한다** (vacuous). 관측 가능한 것은 TCP 자세뿐이므로, 게이트를 검증하려면 같은 seed 컨트롤러에 partial-zero 형상을 valid 로 먹여 실제 출력 차이를 대조한다 |

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
| **`integrate_from_measured`** | 양방향 구현, 기본 `true`. `false` 일 때만 `‖q_cmd−q_meas‖ > command_divergence_limit` → **critical** (`ComplianceFaults::command_divergence` → SAFE_STOP). **관절 축 가드**이므로 task 축 바인딩에는 대응물이 따로 있다 (오른쪽 scope 註) | §7.3 MUST. `true` 는 매 tick 재anchor 되므로 windup 자체가 불가능하다. critical 인 이유: 명령이 이미 팔에서 벗어났으므로 계속 돌수록 간격이 벌어지고, wrench 소실과 달리 **후퇴할 축소 권한 모드가 없다**. **Scope (#478)**: `‖q_cmd−q_meas‖` 는 관절 공간 양이고, 이 감시가 *유일한* 관측 수단인 것은 명령이 곧 q 인 바인딩(joint compliance)에서다. task compliance 바인딩은 FK·Jacobian·`e(X, X_c)`·nullspace 오차를 **전부 측정 q 로 평가**하고 명령 상태를 지고 가는 곳이 마지막 적분기 하나뿐이므로, `q_cmd` 가 팔에서 벗어나는 사건은 **그 tick 에 곧바로 task 축 오차로 나타난다** — 그것을 잡는 것이 `pose_error_limit` → `pose_error_exceeded` 이고, 같은 critical 등급이며 파서가 양수를 강제해 **끌 수도 없다** (D6). 따라서 `demo_compliance_controller` 는 `false` 모드로 돌면서 `command_divergence` 를 **의도적으로 배선하지 않는다**; 같은 사건에 두 번째 critical 을, 그것도 이 컨트롤러가 직접 권한을 갖지 않는 공간에서 거는 것이기 때문이다. 관절 축에서 실제로 필요한 것은 divergence 감시가 아니라 **아래 "관절 한계" 행의 clamp** 다 |
| **관절 한계** | `q_cmd` 를 `[q_min+δ, q_max−δ]` 로 clamp, **그 다음** 적분 base 기준 `max_velocity·dt` 로 rate 재바운드 (순서 반대 금지). 공용 단위 `compliance/joint_command_tail.hpp`, δ 가 밴드를 뒤집으면 **configure 거부** | `safety_limiter.hpp` 는 **torque-domain** 이라 전이되지 않는다. 한계 밖 position 명령은 "부드러운 밀어냄" 이 아니라 backend 가 거부하거나 하드스톱으로 몰고 가는 요청이다. §7.3 방식 3(QP viability)은 범위 밖. **rate 재바운드가 필요한 이유**: clamp 는 받은 스텝을 *넓힐* 수 있다 — 팔이 margin 밴드 안에 있으면 IK 스텝이 아무리 작아도 clamp 가 한 tick 만에 밴드 경계까지 옮기므로 `joint_limit_margin: 0.08` 은 곧 0.08 rad 점프다. 이 컨트롤러는 `ApplySafetyLayer` 의 `max_torque_rate` 를 버렸으므로 **이 clamp 가 관절 축의 유일한 바운드**다 — `command_divergence` 는 위 행의 scope 註대로 task 축 바인딩의 감시가 아니다 (#478). 재바운드로 `q_cmd` 가 밴드 안에 일시적으로 머물 수 있는데, 밴드는 soft margin 이고 "바운드된 속도로 경계에 도달" 이 곧 그 밴드가 요구하는 동작이다. 보고되는 `target_velocities` 도 재계산해 실제 발행 위치와 일치시킨다. tail 이 돌려주는 `{position_clamped, rate_rebounded}` 는 **바인딩이 소비한다** (#484) — 밴드에 눌려 멈춘 팔과 정착한 팔이 다른 모든 출력에서 같은 평평한 궤적이기 때문이다. 채널·판독법은 `integrated_bringup/README.md` compliance §7.3 관절 밴드 판독 |
| **device state 무효** | `!IsDeviceReadable(devices[0], nv)` — **항 3개**: 보고됨 · 폭 충분 · `[0, nv)` 에 구멍 없음 (§3.7 술어표, #284) → device 0 에 **길이 0 명령**(모든 backend 가 "업데이트 없음"으로 취급, CM `BuildHoldOutput` 과 같은 관용구) + `ComplianceFaults::device_state_invalid` → **DEGRADED**, 다음 tick 재-seed. 보조 device 는 passthrough 유지 | 제어 법칙 전체가 이 device 의 q 로 평가된다. 백엔드 첫 state 도착 전이면 읽히지 않은 채널이 기본구성 0 이라 FK/Jacobian 이 **zero configuration** 에서 평가되고 전 관절이 원점으로 명령된다 — 관여한 모든 수가 유한하므로 fault 도 안 뜬다. 모르는 관절 위치에 정직한 대체값은 없다 (0.0 은 "원점으로 가라" 다). DEGRADED 이고 critical 이 아닌 이유: 백엔드 복구가 정상 경로이고, 명령을 안 내는 축소 권한 모드가 존재한다 (발산한 명령에는 없는 선택지) |
| **E-STOP** | **position-hold**: 첫 held tick 의 측정 위치를 **latch** 하고 반복 출력 | `torque_estop.hpp` 의 `ĝ(q) − D·q̇` 는 토크 출력 전제라 부적용. 매 tick `q_meas` 를 재출력하면 hold 처럼 보이지만 backdrivable arm 이 밀릴 때 명령이 따라가 **저항이 사라진다**. CLIK 의 `ComputeEstop`(safe_position 슬루)도 쓰지 않는다 — compliance 컨트롤러가 E-STOP 순간에 조작자가 요청하지 않은 **이동을 시작**하면 안 된다 |
| **hold latch 수명** | 정상 tick 끝에서 무장 해제되고, **재활성화 / `ClearEstop()` / `ResetFault()`** 에서도 무효화된다 (`InvalidateEstopHold()`, RT thread 가 tick 머리에서 소비) | latch 는 "팔이 있던 자세" 다. 이 컨트롤러가 팔을 몰지 않는 동안 (deactivate 후 타 컨트롤러 이동, E-STOP 중 pendant jog) 그 자세는 의미를 잃고, 다음 held tick 이 **한 스텝으로** 옛 자세를 명령한다 — `ComputeEstop` 에는 슬루도 divergence 바운드도 없다. 반대로 **latched SAFE_STOP 이 강제하는 매 tick 재-seed 는 이겨야** 하므로 (그때 무효화하면 hold 가 backdriven arm 을 따라간다) `target_initialized_` 와 **별개 신호**다 |
| **hold 의 관절 한계·rate** | 출력 직전 `[q_min+δ, q_max−δ]` clamp, 이어서 **직전 발행 명령** 기준 `max_velocity·dt` rate 바운드 | 정지 시점에 이미 범위 밖이던 자세를 latch 해 영구 재명령하면 안 된다. 다만 그 교정은 **이동**이므로 이 컨트롤러의 다른 모든 이동과 같이 rate 바운드된다. 기준이 `q_meas` 가 아니라 **직전 발행 명령**인 것이 핵심 — `q_meas` 기준이면 hold 가 다시 팔을 따라가 latch 의 존재 이유가 사라진다 (정지 상태의 hold 는 명령이 변하지 않으므로 바운드가 물지 않는다) |
| **config 검증** | `ik_kp_pos`/`ik_kp_rot` 는 `load6` 와 **동형** — 3-entry sequence 가 아니면 throw (스칼라 축약 없음, D5). `pose_error_limit` 는 **양수 강제**, 위반 시 throw (`≤0 = 비활성` 관용구 미채택, D6) | `load3` 가 shape 불일치를 조용히 버려서, 헤더가 "§7.3 그대로 재현하려면 `ik_kp_*: 0`" 이라 안내하는데 `ik_kp_pos: 0.0` (스칼라) 를 쓰면 버려지고 기본 2.0 이 유지됐다 — 그 knob 이 존재하는 유일한 실험이 조용히 CLIK 변형으로 돌고 진단에도 안 보인다. 스칼라 축약을 넣으면 "조용히 다른 값" 의 변종이 남으므로 허용 안 함. `pose_error_limit` 은 매 tick **critical** fault 비교(`e.norm() > limit`)라 0/음수면 첫 tick 에 SAFE_STOP 이 latch 되고 `pose_error_exceeded` 에는 원인 필드가 없다 — 가드 자체이므로 끄는 수단을 주면 오설정이 정당한 설정처럼 보인다 |
| **`kMaxRobotDOF` 용량 체크** (바인딩 요구사항) | **두지 않는다** (당시 OSC 어댑터와 동일 판단 — 그 클래스는 S7c 에서 삭제됐다) | 이 계열은 `kMaxRobotDOF` 폭 저장소가 하나도 없다 — 작업 버퍼는 전부 nv 로 sizing 된 동적 Eigen 이고, 고정 배열 (`estop_hold_`, `TargetSlot::targets`) 은 device **채널** 인덱스라 `kMaxDeviceChannels`(64) 로 바운드된다. 게다가 체크가 생성자에서 **system 모델**로 돌았고 (`builder.GetFullModel()`), 서브모델 축소는 `LoadConfig` 의 `MaybeSelectSubModel()` 에서 일어나므로 nv=14 dual-arm URDF 가 축소되기 전에 throw → 팩토리 실패 → `on_configure` FAILURE. 같은 URDF 에서 `TaskImpedanceController` 는 정상 동작했다. 체크가 정당한 곳은 실제 `kMaxRobotDOF` 폭 **고정 배열**을 든 바인딩이다 — 당시 그것은 CLIK / JointPD / P 어댑터였고, 지금은 이 기준을 새 바인딩에 적용한다 |
| **wrench 필수** | `external_wrench.enabled: false` 는 **configure 에러** | §7.1 축 A≠NONE. impedance 와 대칭이 아니다: 거기서는 A=NONE 이 1급 법칙이지만, 여기서는 힘이 곧 입력이라 없으면 비싼 위치 홀드로 퇴화한다 |
| **task-space 텔레메트리** | `actual_task_positions` = 측정 TCP, `task_goal_positions` = **compliant frame** `X_c`. 둘 다 6-wide 전부 채운다 (translation + ZYX Euler RPY) | goal lane 이 `X_d` 가 아니라 `X_c` 인 이유: 실제로 명령하는 대상이 `X_c` 이고, `X_d` 만 보이면 지속적인 밀림 하에서 정지한 것처럼 보인다 — 조작자가 지켜보는 바로 그 상황이다. 0..2 만 채우면 FULL_SE3 컨트롤러인데 orientation 실험 로그가 "회전 없음" 으로 남는다 — "미측정" 과 구별 불가한 유일한 판독. 소비자(`device_state_log_pod` / `pod_fill`)는 6개를 전부 CSV 로 방출한다. 선례는 S7c 에서 삭제된 P·OSC 어댑터가 세웠다 |
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

`ControllerState::devices[0]` 이 이번 tick 에 쓸 수 없는 상태 —`valid == false`,
`num_channels < nv`, 또는 `[0, nv)` 에 **이번 tick 에 안 써진 슬롯**이 있는 경우 (#284)— 일 때
세 compliance 컨트롤러가 무엇을 내보내는가. 세 곳에 흩어지면 서로 다른 답을 내므로 여기가 SSoT 다.

| 항목 | 결정 | 근거 |
|---|---|---|
| **바인딩 요구사항 — 게이트 위치** | `Compute()` 의 조인트 상태 복사 **직전** — 세 계열 모두. `ComputeEstop()` 의 `ĝ(q)` 계산 직전 — **토크 도메인 둘** (임피던스·캐스케이드) | 미보고 채널은 `0` 으로 읽히므로 게이트 없이는 FK·Jacobian·법칙 전체가 **ZERO configuration** 에서 돌고 전 관절을 원점으로 당기는 토크가 나간다. 모든 수가 유한해 CM 의 actuator-boundary validator 도 거르지 않는다. `TaskImpedanceController` 는 `Compute()` 게이트조차 없이 출하됐었다 (#236 E-8 에서 신설) |
| **바인딩 요구사항 — position 도메인의 E-STOP hold seed** | position-hold latch 를 `dev0.positions` 에서 seed 하는 바인딩은 **그 seed 지점에도 게이트를 걸어야 한다** | 판독 불가 tick 에 latch 되면 미보고 채널의 hold 가 `0` 으로 굳어 "원점으로 servo" 가 되고, hold-valid 플래그가 latch 라 이후 tick 까지 남는다. 토크 도메인의 `ĝ(q)` 게이트와 같은 이유이며 위치는 다르다 — 여기서는 `Compute()` 진입 게이트를 통과한 뒤 별도로 도달할 수 있다. 이 행의 유래는 §부록 A |
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

**게이트의 구현 위치 — base (#236 S7b)**

위 계약은 더 이상 컨트롤러마다 복제되지 않는다. 판정·출력 형태·꼬리 정책의 **구현**은
`rtc_controller_interface/include/rtc_controller_interface/device_readability.hpp` 가 소유하며
(3계층표의 "device 판독가능성 게이트" 행 —
[design-principles.md](../../agent_docs/design-principles.md) §3계층 배치), 노출되는 것은
**술어 3개 + 원시연산 3개**다.

> **계약 문장의 소유 층위 — 이 절이다 (#297 결정, 2026-07-30).** 같은 계약이 §3.7 · 패키지
> README · 헤더 주석 세 곳에 문장 단위로 복제돼 있었고, #291 이 한 문장의 뜻을 좁힐 때 세 파일을
> 동시에 고쳐야 했다. 더 나쁜 것은 *세 사본이 일치한다*는 사실이 S7b 종결 검증에서 **무결성
> 증거로 쓰였는데**, 정작 셋이 일치한 채로 함께 오독을 유발했다는 점이다 — 복제의 일치는 내용의
> 정확성을 보증하지 않는다. 그래서 **계약 문장은 여기 한 곳**이 소유하고, `README` 와 헤더 주석은
> 술어 이름 + 한 줄 요약 + 이 절로의 링크만 둔다. 헤더에는 "이 코드가 왜 이 모양인가"(음수 clamp,
> `model_dim <= 0` 축퇴, RT 계약)는 계속 코드 옆에 남는다 — 그건 계약이 아니라 구현 근거다.

| 이름 | 무엇을 답하는가 | 언제 적용하는가 |
|---|---|---|
| `ModelChannelBound(nc0, model_dim)` | "몇 채널까지 **인덱싱**해도 되는가" — 순수 OOB 방어, 정책 없음 | **항상**. 과다보고(`nc0 > model_dim`)는 정상 입력이다 (`num_channels` 는 wire 길이) |
| `IsDeviceReadable(dev, model_dim)` | "이 device 를 이번 tick 에 **써도 되는가**" — 게이트. **항 3개**: 보고됨 · 폭 충분 · `[0, model_dim)` 에 구멍 없음 | 조인트 상태를 읽기 **전**. false 면 아래 침묵 |
| `IsGateClosedByWidth(dev, model_dim)` | "닫힌 게이트가 **왜** 닫혔는가" — 폭 축. 진단 전용, 출력에 관여하지 않음 | 게이트 판정 **직후**. true 면 아래 진단 로그 (#307) |
| `IsGateClosedByHoles(dev, model_dim)` | 같은 질문의 **구멍 축** (#284). 폭 판정을 *통과한* device 만 대상이라 위 술어와 배타적 | 같은 자리. 둘은 `valid` 한 device 의 폐쇄를 전수 분할한다 |
| `FirstHoleSlot(dev, model_dim)` | 모델 폭 안에서 **안 써진 가장 낮은 슬롯** (없으면 -1) — 진단 메시지가 지목할 자리 | `IsGateClosedByHoles` 가 true 인 tick 의 로그 인자 |
| `IsSlotFresh(dev, slot)` | "이 **한 슬롯**을 이번 tick 에 써도 되는가" — prefix 가 아닌 특정 `(device, channel)` 을 읽는 reader 용 (#284 후속) | 명명된 관절을 임의 위치에서 뽑는 자리 (closed-chain q 브릿지 등). 게이트로 물으면 과잉거부/미검사 둘 중 하나가 된다 |
| `IsLaneReadable(dev, lane, model_dim)` | "이 **lane** 의 `[0, model_dim)` 을 이번 tick 에 써도 되는가" — 게이트와 같은 항 3개를 lane 인자로 (#446). 게이트는 이 술어의 `kPosition` 인스턴스다 | `velocities` / `efforts` 를 읽는 **모든** 자리. 게이트는 `positions` 만 판정하므로 `IsDeviceReadable` 통과 후 q̇ 를 읽는 코드는 이 술어를 **추가로** 물어야 한다 |
| `SelfReportedChannelBound(dev, cap)` | "선언된 폭이 없을 때 device **자기 보고에서 어떤 폭을 채택**해도 되는가" — 구멍 없는 prefix, `min(nc0, cap)` 이 **아니다** | 바인딩의 deferred self-init 처럼 폭을 **latch** 하는 자리. 근거는 아래 §per-slot freshness |
| `SilenceDeviceOutput(dev_out)` | 게이트가 false 일 때의 유일한 답 (`num_channels = 0`) | **그 게이트가 판정한 device 에만.** primary 게이트는 primary 만 침묵시키고 secondary 는 그대로 둔다 — secondary 가 *자기* 폭으로 판독 불가일 때는 secondary 도 침묵한다 (#291, 아래 표) |
| `HoldTelemetryAtMeasured(dev_out, nc0, measured)` | 침묵 tick 의 reference lane (`target_*` / `trajectory_*`) 을 이번 tick 측정값으로 | `SilenceDeviceOutput` 과 **항상 짝** — 아래 참조 |
| `FillCommandTail(cmds, bound, nc0, cmd, measured)` | `[bound, nc0)` 을 도메인별 중립값으로 (torque → 0.0 / position → 측정값) | 명령 조립 시. 미기입은 fresh-zero = "원점으로" |

**닫힌 게이트의 진단 — 폭 불일치만 보고한다 (#307).** 침묵한 tick 의 유일한 관측 흔적은 `/<key>/transforms`
에서 **arm-tip frame 이 사라지는 것**이고, 부재는 원인을 가리키지 않는다 (게이트 폐쇄 / 컨트롤러 비활성 /
TF slot 미설정을 구별하지 못한다). 명령 lane 은 더 조용하다 — zero-length 는 "갱신 없음" 이고
`HoldTelemetryAtMeasured` 가 로그 lane 을 측정값으로 채우므로 CSV·GUI 상 "팔이 가만히 있다" 와 같다.
그래서 바인딩은 게이트 판정 직후 `IsGateClosedByWidth` 가 참인 tick 에 **throttled WARN** 을 낸다.

보고 대상을 폭 불일치로 **좁히는 것이 이 진단의 계약**이다. 위 "왜 타이머를 두지 않는가" 가 이미
`!valid` 를 배제했다 — CM 의 startup gate 가 모든 device 보고 전에는 `Compute()` 를 돌리지 않으므로
바인딩이 보는 `!valid` 는 백엔드가 아예 없는 그룹뿐이고 그건 init timeout 이 이름을 대며 잡는다. 남는
`num_channels < nv` 는 첫 tick 부터 닫힌 채 열리지 않으므로 **지속 판정용 카운터도 시간 임계도 두지
않는다** — 두 원인이 항으로 분리되므로 첫 보고 tick 이 이미 정직하고, 시간 임계는 이 절이 거부한
타이머를 진단 축으로 되들여오는 것이다.

메시지는 **선언 폭·보고 폭·고칠 키**를 함께 싣는다. **두 축은 이 지점에서 대칭이 아니다.** `arm_dof` 는
#340 이후 required YAML 키이고 fallback 이 없으므로 arm 의 폭은 출처가 하나뿐이고 처방도 무조건이다.
반면 hand 는 출처가 **둘**이다 — device 그룹이 선언한 `joint_state_names` 의 길이, 또는 그 키가 없을 때
**그 device 가 첫 tick 에 보고한 `num_channels`** (CM 의 파서는 이 키를 optional 로 읽는다). 그래서 hand
메시지의 처방은 실제로 이긴 출처를 따라간다:

| `hand_dof_` 출처 | 폭 미달이 뜻하는 것 | 처방 |
|---|---|---|
| 선언된 `joint_state_names` | 설정 불일치 | 그 키 또는 백엔드 보고 폭을 고친다 |
| device 의 첫 보고 | **백엔드가 세션 중 폭을 줄였다** — 고칠 키가 없다 | `joint_state_names` 를 선언해 기대 폭을 고정한다 |

출처 무관하게 `joint_state_names` 를 지목하면 두 번째 경로에서 **존재하지 않는 키로 운영자를 보낸다** —
게다가 그 경로의 고장은 설정 불일치가 아예 아니다. 판정은 `hand_dof_ > 0` (config 해석 종료 시점) 이며,
RT fallback 이 `hand_dof_ == 0` 에서만 발동하므로 이 한 항이 두 출처를 정확히 가른다.

꼬리 문장은 **조립하지 않고 선택**한다 (`%s` + string literal — RT-3 의 허용 목록). 매크로는 base 헬퍼
안이 아니라 **바인딩 호출부에서 전개**하고, 그 호출부에서도 if/else 로 매크로를 둘 두지 않는다 —
`RCLCPP_*_THROTTLE` 은 상태를 매크로 전개점에 두므로 공용 헬퍼 안에 두면 세 바인딩이 창 하나를 공유해
컨트롤러 전환 직후 첫 보고가 삼켜지고, 매크로를 둘로 쪼개면 한 호출부가 창을 둘 갖게 된다.

**침묵은 wire 만 침묵시킨다 — 로그는 아니다.** device state 로그 POD 는 출력이 아니라 **device 의**
`num_channels` 로 bound 하고 (`integrated_bringup/logging/pod_fill.hpp`) 실제로 내보낸 폭을 담는 필드가
없다. 그래서 reference lane 을 fresh-zero 로 두면 침묵 tick 이 **전 채널 0 인 행**으로 기록되고, 이는
정확히 "전 관절을 원점으로 명령" 으로 읽힌다 — §3.7 이 막으려는 그 오독이, 이 게이트의 유일한 발현
조건(기동시 설정 불일치) 에서, 엔지니어가 진단하러 보는 유일한 장소에 나타난다. 따라서
`SilenceDeviceOutput` 을 부르는 모든 lane 은 `HoldTelemetryAtMeasured` 도 부른다 (E-STOP lane 포함 —
거기엔 `Fill*` 가 돌지 않으므로 `ComputeEstop` 이 직접 채운다). CM 의 `BuildHoldOutput` 이 자기
zero-length 케이스에 쓰는 정책과 같다. `goal_positions` 는 예외로 바인딩이 계속 소유한다 — goal 이
무엇을 뜻하는지는 바인딩마다 다르고 (관절 hold 타깃 / 앞 3슬롯이 TCP pose), 운영자가 준 목표는 팔이
판독 불가가 됐다고 사라지지 않는다.

**게이트는 primary lane 만 잡는다 — secondary 제어 법칙까지 멈추면 안 된다.** 판독 불가 tick 에
컨트롤러가 secondary(핸드) 궤적 계산 자체를 건너뛰면, 그 tick 의 핸드 명령은 *직전 값의 재생*이 되고
활성화 이후 판독 가능한 tick 이 한 번도 없었다면 그 값은 zero-init 이다 — 즉 **핸드를 원점으로 보내는
진짜 명령**이며, 팔에서 막은 hazard 를 device 하나 옆으로 옮긴 것에 불과하다. 그러므로 secondary
lane 은 primary 게이트와 **다른 함수/다른 분기**에 두어 항상 돌게 한다 (`DemoTaskController::
ComputeSecondary` 가 이 이유로 `ComputeControl` 에서 분리돼 있다).

**단, 위 문장은 "secondary 에는 게이트가 없다" 가 아니다 (#291).** 두 개의 다른 질문이 있다.

| 질문 | 답 | 술어 |
|---|---|---|
| *팔이* 판독 불가일 때 핸드를 죽이는가 | **아니다** — 위 문단. 핸드는 계속 명령된다 | `arm_readable_` 는 `devices[0]` 만 침묵시킨다 |
| *핸드가* 판독 불가일 때 핸드는 무엇을 내보내는가 | **팔과 같게 침묵** — `SilenceDeviceOutput(out1)` + `HoldTelemetryAtMeasured` | `hand_readable_ = IsDeviceReadable(dev1, hand_dof_)` |

두 번째 줄이 #291 이 닫은 것이다. hazard 는 팔과 **구조가 같다**: `hand_dof_` 는 YAML
`joint_state_names.size()` 에서 오고 `num_channels` 는 wire 길이라, 어긋나면 미보고 핸드 관절이
유한한 `0` 으로 읽혀 궤적·hold·grasp FSM 에 흘러든다. 따라서 게이트는 **device 마다 자기 폭으로**
서고, 한 device 의 게이트가 다른 device 를 침묵시키지 않는다. `IsDeviceReadable` 이 애초에
device 무관이라 새 추상화는 필요 없었다.

secondary 축에서 특히 주의할 세 가지:

- **latch 하는 self-init 이 가장 위험하다.** `hand_target_initialized_` / `target_initialized_` 는
  성공 시 latch 하므로, 판독 불가 상태에서 seed 하면 환영의 `0` 이 영구히 남고 재시드가 없다
  (일반 판독 지점은 tick 마다 재오염되므로 게이트가 서는 순간 회복된다). 팔의 #265 audit T1/T2 와
  같은 자리다
- **grasp FSM 은 수치가 아니라 분기가 바뀐다.** release gate 의 `d = target − dev1.positions[gi]`
  는 `gi` 가 모델 인덱스라, 미보고 채널이면 `d` 가 환영값이 되어 contact-stop/release **판정이
  뒤집힌다**. 또 hold LPF 는 IIR 이라 오염 샘플이 그 tick 을 넘어 남는다
- **모델 lane 은 all-or-nothing 이다.** `ExtractFullState` 의 핸드 절반은 bound 만으로는 부족하다
  — 지속 버퍼 `q_curr_full_` 에서 부분 갱신은 *fresh 앞부분 + stale 뒷부분* 이라는 **로봇이 취한 적
  없는 자세**가 되고, fingertip FK·폐쇄체인 구속 블록은 핸드 블록 **전체**의 함수다. 한 tick 묵은
  일관된 자세는 방어 가능하지만 이어붙인 자세는 아니다. 반대 방향의 비대칭은 유지된다: 팔이 판독
  불가면 (모델을 공유하므로) 핸드 블록도 갱신하지 않고, 핸드가 판독 불가여도 팔 scatter 는 남긴다

**두 술어의 이름을 분리한 이유** (#265 결정 B): `min(nc0, nv)` 는 좁은 device 를 안전하게
만들어 주는 것처럼 보이지만 그렇지 않다. *지속* 버퍼에 scatter 하는 경로(`ExtractFullState`,
`CopyToEigen`)에서 건너뛴 슬롯은 이전값(초기 0)을 유지하므로, 모델이 보는 configuration 은
**bound 없이 읽은 것과 수치적으로 동일한** 부분 ZERO configuration 이다. bound 는 crash 만
없애고 hazard 는 남긴다. 따라서 #172 의 `min(nc0, nv)` 는 OOB 방어로서 옳고 되돌리지 않되,
F5 답으로 쓰면 안 된다. 이 성질은 계약 테스트
`rtc_controller_interface/test/test_device_readability.cpp` 의 `GateVsBoundTest` 2건이 pin 한다.

**per-slot freshness — 폭 판정만으로는 충분조건이 아니었다 (#265 D1-a → issue #284, 닫힘)**

`!dev.valid || dev.num_channels < nv` 두 항만으로는 **필요조건이지 충분조건이 아니었다.** true 가
"슬롯 `[0, nv)` 가 이번 tick 에 갱신됐다" 를 뜻하지 않기 때문이다 — `num_channels` 는 wire
길이이고, reorder map 이 활성이면 실제로 써지는 슬롯은 *매칭된 ref 인덱스*라 `nc0 ≥ nv` 여도
구멍이 남는다. 반례는 `integrated_bringup/test/test_joint_state_reorder.cpp` 가 의도된 동작으로
pin 하고 있다 (`{j2, ghost, j1}` → `num_channels == 3` 인데 슬롯 0 은 sentinel 잔존).

`num_channels` 의 의미를 좁히는 안은 기각됐고 (**결정 B**, 재논의 금지 — 그 필드는
`ValidateControllerOutput` 의 egress bound 로 이미 쓰이고, 반례가 *의도된 동작*으로 pin 돼
있으므로 계약을 좁히면 옳은 테스트가 깨진다 → PROC-6), 대신 **`DeviceState::hole_mask`** 를
신설했다 (`rtc_base/types/types.hpp`). 계약은 넷이다.

| 항목 | 계약 |
|---|---|
| **의미** | 비트 `i` set = 슬롯 `i` 가 **직전 state 메시지에서 안 써짐**. `positions` lane 한정 |
| **극성** | set 이 나쁜 소식 — aggregate zero-init 이 "구멍 주장 없음" 이 되어, 이 필드를 채우지 않는 생산자(테스트 fixture·미래 백엔드)의 판정이 도입 전과 **동일**하다. 반대 극성이면 `DeviceState{}` 전부가 fail-closed 가 된다 |
| **생산** | ingress 공유 헬퍼 `WriteJointStateToCache` **한 곳**. 메시지마다 **대입**하며 누적(OR)하지 않는다 — 누적하면 한 번 써진 슬롯이 영구히 fresh 로 남아 갭이 다시 열린다. 백엔드 3종이 이 헬퍼를 공유하므로 한 곳이 셋을 정합시킨다 |
| **판정** | 별도 술어가 아니라 **`IsDeviceReadable` 의 세 번째 항**으로 접힌다 |

**왜 별도 술어가 아니라 게이트 안인가.** 소비자를 빠뜨리는 것을 원리적으로 불가능하게 만들기
위해서다. 이 갭의 착수 감사조차 "컨트롤러 3종" 패턴으로 훑어 support 계층 2곳
(`combined_model_cache` · `closed_chain_hand_fk`) 을 빠뜨렸다 — 게이트에 접힌 항은 모든 소비자에게
구조적으로 도달하고, 별도 술어는 *누군가 기억한* 소비자에게만 도달한다.

`positions` **lane 한정**인 것도 계약이다. velocity/effort 는 각자 메시지 길이로 복사되므로 슬롯
하나가 fresh position + stale velocity 일 수 있다. #284 는 이를 "다른 축" 으로 유예했고 **#446 이
그 축을 열었다** (아래 절). motor/sensor lane 은 여전히 범위 밖이다 — motor lane 은 reorder 를
타지 않아 이 기전이 없고, sensor lane 은 동형의 갭이 있으나 폭이 `kMaxSensorChannels`(128)이라
`uint64_t` 하나에 안 들어가고 이 게이트의 소비자도 아니다.

**닫힌 게이트의 세 번째 진단.** 게이트에 항을 늘리면서 진단을 안 늘리면 *조용한 폐쇄*가 생기고,
그건 위 "닫힌 게이트의 진단" 이 없애려던 실패 그 자체다. 그래서 `IsGateClosedByHoles` +
`FirstHoleSlot` 이 함께 신설됐고, `valid` 한 device 의 폐쇄 원인은 이제 **폭 / 구멍** 둘로
분할된다 (배타적·전수). 메시지가 지목하는 키는 `joint_command_names` 다 — 세 백엔드가
`BuildJointStateReorder` 에 넘기는 참조 리스트가 그것이고, 따라서 보고되는 **슬롯 인덱스도 그
리스트 기준**이다. CM 파서는 이 키가 없으면 `joint_state_names` 로 default 하므로 메시지는 둘 다
지목한다. 폭 축과 달리 처방은 arm/hand **대칭**이다.

**세 번째 항이 깬 것 — 자기 보고에서 폭을 채택하는 자리 (`SelfReportedChannelBound`).** 선언된
DOF 가 없는 바인딩은 device 의 첫 보고에서 폭을 latch 한다 (YAML 을 우회한 fixture, 또는
`joint_state_names` 를 선언하지 않은 hand 그룹). 그 자리들은 `min(nc0, cap)` 을 쓰면서 *"nc0 로
bound 되므로 방금 통과한 게이트를 다시 닫을 수 없다"* 를 근거로 삼았는데, 이는 **폭 항에 대해서만
참**이다 — 구멍 항은 `nc0` 가 bound 하지 않으므로 wire 길이에서 채택한 폭이 다음 tick 에 거부될 수
있다. 이 자리들은 **latch** 하므로 결과가 영구 침묵 + 안 써진 슬롯으로 굳은 seed 다 (그 seed 오염은
이 블록들이 원래 막으려던 실패이며, 새 항을 통해 되돌아온다). 그래서 채택 폭은 **구멍 없는
prefix** 로 자른다 — 구멍이 없는 device 는 예전과 같은 답을 받으므로 기존 테스트는 불변이다.
불변식 `IsDeviceReadable(dev, SelfReportedChannelBound(dev, cap))` 은 `test_device_readability.cpp`
의 `TheAdoptedWidthAlwaysPassesTheGate` 가 pin 한다.

**접은 항의 도달 범위는 "이 술어를 부르는 소비자" 까지다.** #284 는 세 번째 항을 게이트에 접으면
모든 소비자가 편집 없이 강한 답을 받는다고 적었고 그건 참이지만, **술어를 부르지 않고 본문을 손수
재구현한 reader** 는 그 밖이다. 실제로 하나 있었다 — `ClosedChainHandFk::Update`
(`integrated_bringup/src/support/closed_chain_hand_fk.cpp`) 가 소스 채널마다
`valid && channel < num_channels` 를 손수 검사해 구멍 난 슬롯의 직전 값을 측정값으로 사영했고,
같은 파일의 `IsDeviceReadable` 호출은 **serial 경로 전용**이라 closed 경로를 덮지 않았다.
그 reader 는 prefix 가 아니라 특정 `(device, channel)` 쌍을 읽으므로 맞는 짝은 게이트가 아니라
**`IsSlotFresh`** 이고, 그래서 그 형태에 이름을 줬다 (손수 만든 사본은 원본의 변경이 못 미친다).
전수 훑기 결과 그런 reader 는 저장소에 그 하나였다.

**lane 축 — 게이트에 접지 *않은* 유일한 항 (issue #446)**

`hole_mask` 가 `positions` 한정인 것은 #284 시점에 **참인 전제** 위에 서 있었다: 게이트의 소비자가
전부 q 만 인덱싱한다는 것. 그 전제는 이미 깨져 있었다 — `CombinedModelCache::ExtractFullState` 가
게이트를 통과한 **직후** `dev0.velocities` 를 읽고, 그 값이 `v_curr_full_` 을 거쳐 TSID · 자코비안
제어 법칙에 들어간다. 게이트가 판정한 적 없는 lane 이 출하된 제어 법칙에 도달해 있었던 것이다.

`sensor_msgs/JointState` 는 `velocity` · `effort` 를 **optional** 로 두고 출하 드라이버가 실제로
생략하므로, 이 상태는 예외가 아니라 정상 입력이다. lane 이 비면 배열은 zero-init 에 머무는데
소비자에겐 "정지한 관절" 과 구별되지 않는다.

| 항목 | 계약 |
|---|---|
| **필드** | `DeviceState::velocity_hole_mask` · `::effort_hole_mask` — `hole_mask` 와 **같은 극성 · 같은 생산자 · 같은 대입(누적 아님) 규칙** |
| **폭 항의 부재** | `num_channels` 는 **position** wire 길이이므로 두 lane 을 bound 하지 않는다. lane 이 짧거나 없는 것을 폭으로는 말할 수 없다는 것이 이 필드들이 필요한 이유다 |
| **판정** | `IsLaneReadable(dev, lane, model_dim)` — **별도 술어**. `IsDeviceReadable` 은 이 술어의 `kPosition` 인스턴스로 정의되어 세 항의 사본이 하나만 존재한다 |

**왜 이번엔 게이트에 접지 않는가 — #284 와 반대 방향이고, 그 규칙이 함의하는 대로다.** 접힌 항은
모든 소비자에게 구조적으로 도달한다. 그건 *강한 답이 모두에게 옳은 답일 때만* 미덕이다. q 만 읽는
소비자(대다수)에게 q̇ 의 구멍으로 게이트를 닫는 것은 과잉거부이며, 이 축에서는 강한 답이 옳은 답이
아니다. 그래서 lane 질문은 자기 술어를 갖고 게이트는 늘 묻던 positions 질문을 계속 묻는다.
**그 선택의 비용은 명시한다**: q̇ / τ 를 읽는 소비자는 **물어야** 하고, 그걸 처음 놓친 코드가 이미
트리 안에 있었다 (`ExtractFullState`).

**소비자 측 처방은 lane 도 all-or-nothing 이다.** `ExtractFullState` 는 hand 블록과 같은 이유로
velocity 를 블록 단위로 갱신한다 — `v_curr_full_` 은 persistent 이므로 구멍 난 슬롯만 건너뛰면
일부는 이번 tick, 일부는 과거인 **splice** 가 되고 `h(q, v)` 는 벡터 전체의 함수다. 다만 이것이
닫지 *않는* 것도 적어 둔다: lane 을 아예 보고하지 않는 device 는 v 가 영원히 zero-init 이고 그건
여기서 "정지" 와 구별되지 않는다. 그 구별이 필요한 소비자(momentum observer, #135)는 스스로
`IsLaneReadable` 을 묻는다 — 남을 대신해 거부하는 것은 이 헬퍼의 권한이 아니다.

**이 원인이 startup 전용은 아니다.** 마스크는 메시지마다 대입되므로 나중 메시지가 앞선 메시지의
슬롯을 구멍으로 만들 수 있다. 보통은 `num_channels` 도 함께 좁아져 **폭** 축이 발화하지만, 넓은
첫 메시지로 map 이 고정되고 모델 슬롯이 메시지의 뒤쪽 인덱스에 있으면 짧은 후속 메시지가
`nc0 ≥ nv` 를 유지한 채 `[0, nv)` 를 통째로 비운다. 그 tick 도 **hysteresis 를 두지 않는다** —
슬롯이 실제로 stale 이므로 침묵이 옳은 답이고, 디바운스는 device 가 보내지 않은 값을 제어 법칙에
먹이는 데 그 시간을 쓴다.

### 부록 A — position-domain E-STOP seed 행의 유래 (#236 S7c, E-8 결정 A)

위 표의 "바인딩 요구사항" 행은 원래 **클래스별 결함 노트**였다:
`TaskAdmittanceController::ComputeEstop` 이 `dev0.positions` 에서 hold latch 를 seed 하면서
게이트를 걸지 않는다는 지적이고, "이번 범위에서 손대지 않음" 으로 유예돼 있었다. S7c 에서 그
클래스가 삭제되면서 **주체가 사라졌으므로**, 결함 노트를 지우는 대신 남는 것 — 미래 바인딩이
지켜야 할 요구사항 — 으로 다시 썼다 (E-8 결정 A: 은퇴 + provenance, 2026-07-28 사용자 컨펌).
실코드 게이트를 추가하지도, base 로 상향하지도 않았다. 근거: 프로덕션 admittance/compliance
바인딩이 0 이고, 계약 자체는 base(`device_readability.hpp`) + 데모 3종이 이미 소유한다.

**동시에 그 행의 유예 *근거절* 을 정정했다.** 옛 문장은 "이 경로는 global E-STOP 하에서만
도달하므로 CM 이 `BuildHoldOutput` 으로 치환한다" 였는데, 코드 실측상 `ComputeEstop` 호출점은
둘이었다 — global E-STOP 경로(성립)와 **컨트롤러-로컬 SAFE_STOP 경로**다. 후자에서는
`IsGlobalEstopped()` 가 false 라 CM 이 치환하지 **않는다**. 결론(하드웨어 미도달)은 살아 있었지만
그 문장이 적지 않은 다른 이유 때문이었다: 로컬 경로는 `Compute()` 진입 F5 게이트의 하류라 그 tick
의 device 가 항상 판독 가능했고, global 경로에서 굳은 zero-latch 는 `ClearEstop()` 의
hold-invalidate 가 은퇴시켰다. 새 바인딩이 그 두 조건 중 하나라도 재현하지 않으면 위험은
되살아나므로, 요구사항 형태로 남기는 편이 정확하다.

## 3.8 §6.5 λ 규약 수렴 — OSC·CLIK 이관의 지점별 차이 (#236 S2b+S3b)

`OperationalSpaceController` 와 `ClikController` 는 저장소에 남아 있던 마지막 **상수-λ** DLS
구현이었고, 각각 `compliance/task_dynamics` 와 `compliance/differential_ik` 로 이관됐다. 이
이관은 **비트 동일이 아니다** — `## 검증 전략` 이 요구하는 "발생 지점마다 (왜 / 차이 상한 /
무해 근거)" 기록이 아래 표다. 감당 근거는 **두 어댑터 모두 bringup 배선이 0** 이라는 것이다
(`integrated_bringup` 은 `demo_joint`/`demo_task`/`demo_wbc` 만 등록한다). 배선된 여섯 번째
사본인 `demo_task_controller` 의 인라인 DLS 는 **바인딩이므로** 여기서 건드리지 않았고,
**#282 에서 같은 2-티어 형식으로 별도 이관됐다** — 그쪽은 배선이 0 이 아니라 출하 로봇 3종이므로
티어 2 상한을 인용하지 않고 다시 실측했다 (LDLT→LLT 는 여기 CLIK 의 LLT→LLT 와 다른 반올림
프로파일이다). 하네스는 `integrated_bringup/test/test_task_dls_convergence.cpp`.

차이는 **두 티어**이고 섞으면 센서가 vacuous 해진다.

| 티어 | 지점 | 왜 바뀌었나 | 차이 상한 | 무해 근거 |
|---|---|---|---|---|
| **1 (법칙)** | λ 규약: 상수 `max(1e-4, damping)` → §6.5 σ_min-적응형 | **이관의 목적 그 자체.** 상수 λ 인 두 컨트롤러가 아웃라이어였고 나머지 셋은 이미 §6.5 였다 | **없음 — O(1)** (특이점 밖에서 `λ²`: 0.0025 → 정확히 0) | 의도된 동작 변경. 배선 0. 오히려 특이점에서 멀 때 영공간 직교성 `J M⁻¹ Nᵀ = 0` 이 정확해진다 |
| **2 (구조)** | 고정 `LLT<Matrix6d>` → 동적 `LLT<MatrixXd>` (**벡터** RHS 만 해당) | 헬퍼가 m=3/m=6 을 모두 받아야 한다 (고정 6×6 은 `TRANSLATION_ONLY` 불가) | 2.5e-14 | 반올림. 행렬 RHS 는 비트 동일 (probe 0/840,000) |
| **2 (구조)** | LDLT → LLT (CLIK) | 헬퍼는 SPD 전제라 LLT 하나로 통일 | 1.7e-12 | 반올림. `J Jᵀ + λ²I` 는 SPD |
| **2 (구조)** | 역행렬 materialise → solve | `LambdaS()` 는 §6.3·§7.6 소비자가 **행렬로** 받는 public 출력이라 어차피 materialise 가 남는다 | 5.0e-14 | 반올림 |
| **2 (구조)** | CLIK 영공간 게인이 사영 **後 → 前** (`kp·(N·Δq)` → `N·(kp·Δq)`) | 그래야 `joint::ComputePostureVelocity` 를 부를 수 있고, 다섯 소비자가 한 법칙이 된다 | 4.2e-16 (비트 불일치율은 68% 지만 마지막 자리) | 스칼라 분배는 대수적 항등 |

센서는 티어별로 다르다 (`test/test_dls_convergence.cpp`):

- **티어 2** — 리터럴 pre-extraction oracle 을 유지하되 `BitsEqual` 대신 **상대오차 상한 1e-9**.
  λ² 를 oracle 에 **인자로 넘겨** 양쪽을 일치시키므로 남는 차이는 구조뿐이다.
  `RelativeBoundRejectsTheConstantLambdaLaw` 가 그 상한이 λ 변경을 실제로 **거부**함을 보여
  비-vacuity 를 고정한다 (상한을 넓혀 티어 1 을 덮으려 드는 순간 무의미해진다는 증명).
- **티어 1** — 법칙의 *성질* 로 고정한다: 셸 밖 `λ²=0` · 셸 안 단조증가 · `λ² ≤ λ_max²` ·
  σ₀ ≤ 0 이면 램프 무장해제(NUM-2). 그리고 두 이관 경로가 **그 법칙을 쓰고 있는지**를 별도로.
- **게이트·진단** — 여전히 비트 (bool 은 반올림이 없다). 수치 레인이 무뎌진 만큼 이쪽이 유일한
  배선 센서다. mutation 11종 중 2종이 관측 창 없이는 **탐지되지 않았고**, 그 둘에만 창을 냈다:
  OSC 의 `nullspace_active()` (게인 0 이면 게이트가 수치적으로 inert) 와 CLIK 의 비유한 J 강등
  경로 (`ok == false` 가 평범한 draw 에서는 도달 불가). 나머지 9종은 창 없이 발화한다.

## 3.9 F8 임계값 수렴 — 세 스키마의 비대칭 해소 (#298 S7c-2)

§10.5 `max_torque_rate` 와 CRITICAL `pose_error_limit` 은 세 계열이 **같은 이름·같은
안전 계층**을 공유하는데도 검증이 갈려 있었다. 캐스케이드는 둘 다, 어드미턴스는
`pose_error_limit` 을 거부했고, **임피던스는 둘 다 무가드**였다. 어댑터 시절에는 세
`LoadConfig` 가 1000줄 넘는 파일 셋에 흩어져 있어 나란히 읽을 수 없었고, S7c-1 이
`params/` 한 디렉토리로 모으면서 비로소 보였다 — 통합 리팩터가 아웃라이어를 드러낸
전형적인 경로다.

| 항목 | 결정 | 근거 |
|---|---|---|
| **판정식** | `!(v > 0.0) \|\| !std::isfinite(v)` — 세 스키마 공통 (어드미턴스는 #280 에서 합류; 아래 §3.10 참조) | `v <= 0.0` 은 **NaN 을 통과시킨다** (NaN 과의 모든 비교가 false). `.inf` 도 마찬가지로 통과하는데, `pose_error_limit: .inf` 는 CRITICAL 바운드를 영구 도달 불가로 만든다 — 0 과 정확히 반대 방향의 같은 결함이다 |
| **`max_torque_rate` 이 0 일 때 실제로 일어나는 일** | 슬루 리미터가 **조용히 꺼진다** (freeze 아님) | `compliance::RateLimit` 은 `max_rate <= 0.0` 에서 `tau_prev = tau; return false;` 로 early-return 한다 ("a degenerate tick must not freeze the command at tau_prev"). 캐스케이드 주석은 "명령이 rate-limit 이력에 freeze 된다" 고 적혀 있었으나 헬퍼와 모순이었고, 함께 정정했다. 결과가 다르면 완화책도 다르다 — 진짜 위험은 `SafetyStatus::rate_limited` 가 영원히 false 인 채 팔에 슬루 보호가 없는 것이고, 진단 어디에도 그 사실이 안 나온다 |
| **거부 vs 클램프** | 거부 | 클램프할 **정당한 대상값이 없다**. `≤0 = 비활성` 관용구도 미채택 (D6 과 동일 논리): 가드 자체를 끄는 수단을 주면 오설정이 정당한 설정처럼 보인다 |
| **호환성** | 파괴적 변경이나 실측 노출 0 | repo 내 `max_torque_rate: 0` / `pose_error_limit: ≤0` 설정 0건, 이 값을 *허용* 으로 pin 하던 테스트 0건 (기존 테스트는 `1.0e6` 만 쓴다). 따라서 PROC-6/E-6 해당 없음 — 기존 assertion 을 약화한 것이 아니라 가드를 추가한 것이다 |

## 3.10 §5.3 안전층 게인 수렴 — `params/` 의 마지막 비대칭 (#280)

§3.9 가 F8 임계값 둘을 수렴시킨 뒤 남은 **마지막 비수렴 검증**이다 (NUM-6b). 키가 셋인데
**처방이 갈린다** — 이것이 이 축을 별도 이슈로 만든 이유다. `std::max(0.0, ·)` 는 셋 중
**어디에도** 옳은 철자가 아니며, 그 판정은 #279 리뷰가 자세 게인에서 이미 내렸다.

| 키 | 처방 | 근거 |
|---|---|---|
| `joint_limit_stiffness` (k_lim) · `joint_limit_damping` (d_lim) | **floor** — `rtc::FloorNonNegativeGain` (유한만 floor, 비유한 통과) | 밴드 안에서 `q − lo < 0` 이라 부호가 자세 게인과 **반대로** 작동한다: `k_lim<0` 은 관절을 한계 **안쪽으로** 밀고 `d_lim<0` 은 하드스톱 앞에서 에너지를 주입한다. 출하 UR5e 는 `k_lim=0, d_lim=2` (순수 감쇠) 라 d_lim 이 유일한 방어선이다. 비유한 값을 0 으로 세탁하면 안 되는 이유는 `ApplySafetyLayer` 의 단계 순서다 — `AllFinite` 가 반발항 **뒤·saturation 앞**이라, NaN 게인은 지금 `nan_inf` SAFE_STOP 으로 잡히는데 `std::max` 는 그 fault 를 조용히 은퇴시킨다 |
| `joint_limit_margin` (δ) | **configure 거부** (`>= 0` 이고 유한) | floor 로는 **고칠 수 없다**. `lo = q_min + δ` 이므로 δ<0 은 밴드를 한계 밖에 놓고, δ=NaN 은 `q < lo` 도 `q > hi` 도 false 로 만들어 반발항이 한 번도 발동하지 않는다 — 그런데 fault 는 없다. 0 으로 clamp 하면 그 오설정이 **정상 구동되는 config** 가 된다. δ=0 자체는 합법 ("밴드 경계 = 관절 한계"; 어드미턴스 출하 기본값) 이라 `>0` 이 아니라 `>=0` 이다 |

**적용 범위** — 세 검사 모두 **키 유무와 무관하게 무조건**이고 `if (!cfg)` 조기 반환 경로도
포함한다. 키 안(`if (cfg[key])`)에만 두면 값이 생성자·`set_gains()` 에서 온 경우를 못 보는데,
POD 를 SeqLock 에 직접 쓰는 그 경로가 정확히 하한이 필요한 경우다. 캐스케이드는 #280 이전에
floor 를 갖고 있었으나 `if` **안**의 손으로 쓴 `std::max` 여서 이 경우를 놓쳤다 — "이미 옳다"
가 아니라 **양쪽 다 고칠 대상**이었다.

**법칙에는 넣지 않는다** — `compliance::AddJointLimitRepulsive` 는 자기 인자를 floor 하지
않는다. NUM-1 의 `AdaptiveDampingSquared` 와 같은 판정이다: 법칙이 인자를 몰래 고치면 리터럴
oracle 이 거짓이 되고, 기존 oracle 은 전부 양수 게인이라 **그 변경을 아무 테스트도 못 본다**.
사용 지점 하한은 λ_max 와 마찬가지로 **바인딩 요구사항**이며 in-tree 소비자는 아직 없다.

**센서** — `test_params_schema.cpp` 의 `SafetyLayerGainSchema.*` 7 케이스가 키×경로(음수 YAML ·
키 부재 · `UndefinedNode()` · 비유한 · 정상값 비-vacuity)를 pin 한다. green 만으로는 부족하므로
**11종 mutation** 으로 각 가드를 하나씩 되돌려 실측했고, 전부 검출됐다. 특히 floor 를
`std::max(0.0, ·)` 로 바꾸는 mutation 은 **비유한 통과 케이스 하나에서만** red 가 된다 — 나머지
전 케이스가 green 을 유지하므로, 그 케이스가 없으면 #279 가 폐기한 철자가 그대로 재유입된다.

**호환성** — 실측 노출 0 (`grep "joint_limit" integrated_bringup/src/` 0건, `ApplySafetyLayer`
소비자는 테스트뿐). 이 키들을 쓰는 repo YAML 0건, 음수·비유한 값을 *허용* 으로 pin 하던 테스트도
0건 — 따라서 PROC-6/E-6 해당 없음 (약화가 아니라 가드 추가). 같은 PR 에서 어드미턴스의
`pose_error_limit` 이 §3.9 의 공통 판정식 중 `isfinite` 절반을 빠뜨리고 있던 것도 함께
합류시켰다 (그 스키마에는 `num()` 이 없어 `.inf` 가 통과하고 있었다).

## 4. 슬라이스 1 설계 주석 (README 필수 설명)

- **A=NONE 은 열등한 fallback 이 아니다** (§6.2). 폐루프는 `Λ(q)ë + [μ+K_d]ė + K_p e = f_ext`
  로 desired inertia 가 자연 관성 `Λ(q)` 로 남고 f_ext 측정이 전혀 불필요하다. 접촉 안정성
  (passivity) 측면에서 오히려 견고하다. "F/T 센서 없으니 반쪽" 은 오해다.
- **`TRANSLATION_ONLY` 의 회전은 nullspace posture 가 담당** (§6.1). 회전은 task 로 규제되지
  않고 joint posture 로 **간접** 결정되므로, 정확한 회전 복원은 nullspace gain·여유자유도에
  의존한다 ("회전 free-float" 도 "회전 stiff" 도 아님). 그래서 `nullspace_stiffness ≤ 0` +
  `TRANSLATION_ONLY` 는 **configure 에러** (회전 무구속 drift = 위험). `< 0` 이 `== 0` 과 같은
  판정을 받는 이유는 #277 이 자세 게인을 로더·사용 지점 양쪽에서 `max(0.0, ·)` 로 floor 하기
  때문이다 — 음수 `K_pⁿ` 는 τ₀ 를 자세 오차가 **커지는** 방향으로 만들고 `Nᵀ` 가 그걸 task
  로부터 가리므로, 조용히 자세가 밀리는 대신 0 과 동일하게 취급돼 가드에 걸린다. floor 가
  가드 **앞**에 있는 것이 이 정합성의 전부다. configure 는 거부로 끝나지만 `set_gains()` 는
  POD 를 SeqLock 에 직접 써 configure 를 통째로 우회하므로, 런타임에 같은 조합에 도달하면
  `ComplianceFaults::posture_authority_lost` 를 **DEGRADED** 로 세워야 한다 — 여기서 SAFE_STOP
  이 아닌 이유는 병진 task 는 여전히 정상 추종 중이고 잃은 것은 회전 *권한* 이라, 알리는 것이
  멈추는 것보다 정확한 축소-권한 답이기 때문이다. 조용한 것만이 선택지가 아니었다.
  **단 이것은 아직 요구사항이지 구현이 아니다** (#301): `ComplianceStateMachine` 에 출하
  바인딩이 없어 `ComplianceFaults` 의 어떤 필드도 in-tree 기록자가 없고, `TaskSelection::`
  `kTranslationOnly` 자체가 출하 컨트롤러에 배선돼 있지 않아 조건에 도달할 경로도 없다.
  상태 머신을 배선하는 바인딩이 생길 때 이 필드 하나가 아니라 전체를 함께 세운다.
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

후속 슬라이스 표는 issue #316 참조 (#236 은 종결 — 결정 이력만 그쪽에 남는다).
