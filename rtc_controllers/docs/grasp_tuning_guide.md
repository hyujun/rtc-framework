# Force-PI Grasp Controller — 튜닝 가이드

`rtc::grasp::GraspController` 는 손가락별 스칼라 파지 파라미터 `s ∈ [0,1]` 의 미분 `ds` 를
outer-loop PI 출력으로 쓰는 **position-rate 힘 제어기**다. 관절 토크를 지령하지 않으므로 위치
제어 하드웨어에서도 힘 제어가 성립한다. 앞 두 손가락의 접촉만으로 파지 성공을 판정하는 비대칭
FSM 을 돌리고, deformation guard + anomaly-trigger grip tightening 으로 안전 한계를 건다.
핵심 튜닝 축은 (1) `Kp`/`Ki` 의 응답 대 안정성, (2) `delta_s_max` 의 파손 한계,
(3) `f_contact_threshold` 의 false-latch 마진, (4) `settle_*` 의 수렴 판정 엄격도다.

`integrated_bringup` 의 `demo_joint` / `demo_task` 컨트롤러가 `grasp_controller_type: "force_pi"`
로 동작할 때 활성화된다.

- 구현: [grasp_controller.hpp](../include/rtc_controllers/grasp/grasp_controller.hpp), [grasp_controller.cpp](../src/controllers/grasp/grasp_controller.cpp)
- 타입/파라미터: [grasp_types.hpp](../include/rtc_controllers/grasp/grasp_types.hpp)
- YAML: [ur5e_p1a/demo_shared.yaml](../../integrated_bringup/config/ur5e_p1a/controllers/demo_shared.yaml) · [ur5e_p1b/demo_shared.yaml](../../integrated_bringup/config/ur5e_p1b/controllers/demo_shared.yaml) (`force_pi_grasp:` 블록)
- 로더: [demo_shared_config.cpp](../../integrated_bringup/src/support/demo_shared_config.cpp) (`ApplyForcePiBlock`)
- 테스트: [test_grasp_controller.cpp](../test/test_grasp_controller.cpp)

> **문서 구획.** §1–§3 은 컨트롤러 자체 (robot-agnostic — `rtc_controllers` 소유). §4–§7 은 배포
> 종속 (`integrated_bringup` 의 YAML·튜닝 절차). §4 의 파라미터 표가 **코드 default 와 배포값을
> 두 열로 나눠 싣는 이유**가 이것이다 — 한 열로 합치면 어느 쪽이 실제로 도는 값인지 알 수 없고,
> 실제로 그렇게 갈라져 있었다.

---

## 1. Controller 구조 개요

- **제어 변수**: 손가락별 스칼라 grasp parameter `s ∈ [0, 1]`
  - `q(s) = (1 − s) · q_open + s · q_close` 선형 보간 (`InterpolatePosture`)
  - PI 출력은 관절 토크가 아니라 closing 진행도의 미분 `ds`
- **피드백**: fingertip force sensor → delta-spike 가드 → **축별** Bessel 4차 LPF → norm → `f_measured`.
  **필터는 이 클래스가 갖지 않는다** — 호출자가 필터해서 넘긴다. `‖LPF(F)‖` 는 3축을 봐야 하는데
  `Update()` 는 이미 magnitude 로 접힌 값만 받기 때문이다. norm 이후에 필터를 걸면 zero-mean 축
  노이즈가 양의 DC (3축 가우시안이면 ≈1.6σ) 로 정류돼 `f_contact_threshold` 바로 밑에 영구히 깔린다.
  근거 전문은 `GraspController::Update` 의 선언부 주석이 SSoT
- **호스팅**: RT 루프에서 `Update(f_filtered, dt)` 호출 — [joint/compute.cpp](../../integrated_bringup/src/controllers/joint/compute.cpp) · [task/compute.cpp](../../integrated_bringup/src/controllers/task/compute.cpp) 의 `run_force_pi` 분기.
  먹이는 값은 `fingertip_force_mag_filt_grasp_` (가드+LPF 레인) 이며, raw wire 값인
  `grasp_state_.force_magnitude` 가 **아니다**
- **활성 범위**: `phase() != kIdle` 일 때만 hand trajectory 출력의 finger 관절을 덮어씀
- **명령 인터페이스**: `CommandGrasp(target_force)` / `CommandRelease()` (cross-thread atomic flag),
  `Reset()` (§2.7), `set_params()` / `set_target_force()` (즉시 발효)
- **입력 span 계약**: `f_filtered` 의 부족한 항목은 **0 N 으로 읽고** 초과 항목은 무시한다 — 짧은
  span 에 throw 하지 않는 RT-safe 경로 (`ShortForceSpanIsZeroPadded`)

### 1.1 손가락 구성은 런타임 값이다

손가락 수와 **손가락별 DoF 는 모두 `Init(std::span<const FingerConfig>, …)` 이 정하는 런타임
값**이다. `kMaxGraspFingers` / `kMaxDoFPerFinger` (각 8) 는 per-finger 저장소를 고정 크기로
유지하기 위한 컴파일타임 상한일 뿐, 루프는 언제나 런타임 개수까지만 돈다 (RT no-alloc).
DoF 는 손가락마다 **달라도 된다** — 로더가 각 손가락의 posture 배열 길이에서 추론한다.

배포된 두 구성이 그대로 예시다:

| variant | 손가락 | DoF |
|---|---|---|
| `ur5e_p1a` | thumb, index, middle | 3 / 3 / 3 |
| `ur5e_p1b` | thumb, index, middle, ring | **4 / 3 / 2 / 1** |

`GraspControllerRaggedTest` (`HandlesHeterogeneousFingerDoF`) 가 이 경로를 못박는다.
YAML 에서 이름 목록을 바꾸는 방법은 §7.1.

### 1.2 앞 두 손가락 접촉으로 진행을 판정한다

전이 판정은 **인덱스 0·1 두 손가락**의 접촉만 요구한다 (`kPrimaryContacts = min(2, num_fingers)`).
관례상 이 둘이 thumb·index 지만 **이름이 계약은 아니다** — 계약은 순서다. 손가락이 2개 미만인
구성에서는 가용한 손가락 **전부**의 접촉을 요구한다. 나머지 손가락은 접촉 여부와 무관하게 진행을
막지 않는다.

설계 의도는 두 가지다.

1. **2-finger pinch grasp 를 1급으로 지원** — 대향 파지가 실제 조작 대부분을 차지.
2. **후행 손가락의 late contact 을 선택적 참여로** — 접촉하면 force control 에 편입, 접촉 못 해도
   grasp 실패로 판정하지 않음.

그 결과 각 phase 에서 후행 손가락은 "참여자" 또는 "관찰자" 역할을 가변적으로 수행한다 (§2 상세).
`ApproachingTransitionsWithoutMiddleContact` / `ApproachHoldsWhenThumbCannotContact` 가 양방향을
못박는다.

> ⚠ **"순서가 계약" 을 "`finger_names` 를 재정렬하면 다른 쌍으로 판정시킬 수 있다" 로 읽지 말 것.**
> `finger_names` 는 **자세 블록(`q_open`/`q_close`)만** 고른다. 힘 입력은 `f_feed[f] =
> fingertip_force_mag_filt_grasp_[f]` 로 **grasp 손가락 f ↔ 지문 센서 슬롯 f** 가 인덱스로 박혀
> 있고, 구동 관절은 `hand_finger_joint_map[f]` 로 **위치 기반 시퀀스**이며 `finger_names` 와
> 무관하게 파싱된다. 따라서 이름만 재정렬하면 **자세 · 센서 · 관절이 3중으로 어긋난다** — 컴파일도
> 되고 기동도 되며 증상은 "왜 이 손가락이 이상하게 움직이지" 뿐이다.
>
> 셋을 함께 돌리는 것도 일반적으로는 불가능하다: `ur5e_p1b` 의 센서 레인 순서는 tree-model
> `tip_links` 순서에 묶여 있다 (그 YAML 주석의 Stage A-3 — `fingertip_data_[f] ↔
> fingertip_rotations_[f]` 페어링).
>
> **실제로 무는 쌍이 인덱스 0·1 이 아니면 처방은 물체 배치를 바꾸는 것이다** (§8.1 에 p1b 실측
> 사례). 참고로 pull estimator 는 같은 문제를 매 tick `opposing_mask` 로 *관측해서* 푸는데
> (§`pull_estimator` 블록의 `pinch_geometry`), grasp FSM 에는 그 대응물이 없다.

---

## 2. 상태 머신 (`GraspPhase`)

```
                CommandGrasp              앞 두 손가락 접촉         contact_settle_time 경과
   Idle ──────────────────────► Approaching ──────────────► Contact ──────────────────────┐
     ▲                             │  ▲                                                   │
     │                             │  └── s=1.0 도달해도 머무름 (접촉 판정 계속, 실패 없음)  │
     │                             │                                                      ▼
     │                             │                                                 ForceControl
     │                             │                                                      │
     │                             │                          램프 완료 & |e_f|≤ε 를 settle_time 유지
     │                             │                                                      ▼
     │        Releasing ◄──────────┴──────────── CommandRelease ─────────────────────► Holding
     │            │                                                                       │
     └────────────┘  모든 s ≤ 0.01                     slip 또는 force drop → f_desired ↑ ─┘
                                                       (grip tightening, 상한 f_max_multiplier)

   Reset() : 어느 phase 에서든 → Idle (플래그 2개 drop + finger state reset)  — §2.7
```

### 2.1 Idle (`UpdateIdle`)
- `grasp_requested_` 플래그 set → finger state 전부 reset 후 `kApproaching` 진입
- 리셋할 필터 tail 은 없다 (상류 뱅크는 contact_stop 과 수명을 공유하며 warm 하게 유지된다) —
  즉 grasp 시작 첫 tick 의 `f_measured` 는 0 에서 램프하지 않고 실측값이다
- 그 외는 현 자세 hold. `release_requested_` 는 무조건 drop (이미 Idle)

### 2.2 Approaching (`UpdateApproaching`)
- 접촉되지 않은 finger 만 `s += approach_speed · dt`, `s ≤ 1.0` clamp
- finger 별 `f_measured > f_contact_threshold` 면 `contact_detected=true` latch,
  `s_at_contact`/`integral_error`/`K_contact_est=1.0` 초기화
- **전이 조건**: 인덱스 0·1 의 `contact_detected` (§1.2) → `kContact`
- **실패 조건 없음**: `s = 1.0` 까지 닫혔는데 미접촉이어도 **이 phase 에 머무르며 접촉 판정을
  계속 돌린다**. 접촉은 시간 제한이 아니라 사건이고 힘은 늦게 들어올 수 있기 때문 (물체가 미끄러져
  들어옴 / 팔이 이동 중 / fingertip lane 의 뒤늦은 복구). 늦은 접촉이 잡히면 `s_at_contact = 1.0`
  이 되고, 이후 force control 은 `s` 상한에 막혀 **여는 방향으로만** 권한을 갖는다 (deformation
  guard 보다 clamp 가 먼저 작용)
- **RELEASE 를 소비한다** — phase 함수 *맨 앞*에서 (`kForceControl`/`kHolding` 은 맨 뒤). 여기에는
  경쟁 전이 (`kContact`) 가 있어 뒤에 두면 접촉 성립 tick 의 early return 이 플래그를 건너뛴다.
  위 "실패 조건 없음" 과 한 세트다 — 접촉 없는 grasp 를 빠져나오는 유일한 길이며, 없으면 phase
  미러가 non-Idle 로 남아 `grasp_controller_type` 전환(quiet gate)과 BT `ForcePIRelease`
  (phase==idle 대기)가 함께 막힌다
- 후행 손가락은 전이 시점의 `s` 값 그대로 freeze 된 채 다음 phase 로 진입

### 2.3 Contact (`UpdateContact`)
- `contact_settle_timer_ ≥ contact_settle_time` 까지 자세 유지
- 만료 시 `f_desired=0`, `integral_error=0`, `integrator_frozen=false` 로 **모든 finger** 재설정 후
  `kForceControl` 진입
- 목적: 충돌 직후 force 신호 transient/ringing 을 (상류) LPF + 시간으로 안정화
- 튜닝 핸들: `contact_settle_time`

### 2.4 ForceControl (`UpdateForceControl`)
- **접촉된 finger 만** 처리 (`if (!fs.contact_detected) continue;`)
  - `f_desired = min(f_desired + f_ramp_rate · dt, active_target_force)` (force reference ramp)
  - `ds = ComputeAdaptivePI(f, dt)` → `ApplyDeformationGuard(f, ds)` → `s += ds · dt`, clamp `[0,1]`
- 접촉된 모든 finger 가 **`f_desired == active_target_force` (램프 완료) 이고**
  `|f_desired − f_measured| ≤ settle_epsilon` → `force_settle_timer_ += dt`
- `force_settle_timer_ ≥ settle_time` → `kHolding` 전이
- 어느 하나라도 미수렴이면 `force_settle_timer_ = 0` 리셋
- 도중 `release_requested_` → `kReleasing` 즉시 전이

> **램프 완료 요구가 이 판정의 핵심이다.** 없으면 판정하는 것이 "목표힘에 수렴했다" 가 아니라
> "램프가 지금 측정힘 근처를 지나간다" 가 된다. 왜 그것이 배포 파라미터에서 상시 발현했는지는
> `GraspParams::settle_epsilon` 선언부 주석과 `UpdateForceControl` 의 수렴 판정 주석이 SSoT이고,
> 운용상의 결론만 §4.5 에 싣는다. 회귀는 `HoldingIsNotEnteredWhileForceIsShortOfTarget` 가 잡는다.

> **주의**: `all_settled` 는 접촉된 손가락만 집계한다. 즉 앞 두 손가락 기준으로 판정하며, 접촉 못 한
> 손가락은 settle 검사에서 제외된다 (PI 가 돌지 않으므로 의미상 일관).

### 2.5 Holding (`UpdateHolding`)

tick 당 실행 순서가 곧 의미이므로 순서대로 적는다.

1. **감쇠 (무조건)** — `f_desired > active_target_force` 이면
   `f_desired ← max(f_desired − grip_decay_rate · dt, active_target_force)`.
   anomaly 가 해소된 뒤에만 도는 게 아니라 **매 tick 돈다**
2. **PI + deformation guard** 로 `s` 유지 (접촉된 finger 만) — ForceControl 과 동일
3. **이상감지 (per-finger)** — `df/dt < −df_slip_threshold` (슬립 의심) **또는**
   `f_measured < f_slip_fraction · active_target_force` (목표 대비 비율 이하) 중 하나라도 만족하면
   `f_desired ← min(f_desired + grip_tightening_rate · dt, active_target_force · f_max_multiplier)`

1 과 3 이 같은 tick 에 함께 돌 수 있으므로 anomaly 지속 중의 **순 상승률은
`grip_tightening_rate − grip_decay_rate`** 다 (배포값 기준 0.4 N/s).

두 이상감지 조건의 성격이 다르다: `df/dt` 는 **사건**, `f_slip_fraction` 비교는 **상태**(조건이 참인
동안 매 tick 재발화). 그래서 보정량은 반드시 rate 여야 한다 — per-tick 비율이었을 때 grip force 가
`control_rate` 의 함수였다. 근거 전문은 `GraspParams::grip_tightening_rate` 선언부 주석이 SSoT이고,
`GripTighteningIsIndependentOfControlRate` 가 rate-독립성을 못박는다.

**integrator 동결은 건드리지 않는다.** 동결의 유일한 권한은 같은 tick 앞에서 이미 돈
`ApplyDeformationGuard` 이고, 여기서 풀면 guard 가 얼리고 이 분기가 녹이는 교착이 매 tick
반복된다 (`AnomalyDoesNotThawDeformationGuardFreeze`). §3.3·§6.1 참조.

`release_requested_` → `kReleasing`.

### 2.6 Releasing (`UpdateReleasing`)
- **모든 finger** (접촉 여부 무관) `s -= release_speed · dt`, `s ≥ 0` clamp
- 모든 `s ≤ 0.01` 이면 finger state 전체 reset 후 `kIdle`

### 2.7 Reset — 이 컨트롤러가 손을 몰지 않는 tick

`Reset()` 은 phase 를 즉시 `kIdle` 로 되돌리고, GRASP·RELEASE 플래그를 **둘 다** drop 하며, finger
state 를 리셋한다. `Init()` 이 계산한 구성(손가락 배치·파라미터)은 건드리지 않고, `active_target_force`
도 유지한다 (setpoint 이지 state 가 아니며, GRASP 는 언제나 자기 목표를 들고 온다).

**왜 필요한가**: `Update()` 호출을 멈추는 쪽이 FSM 진행도 멈추므로, 제어법 전환이나 deactivate 로
중단된 grasp 는 플래그를 장전한 채 phase 중간에 얼어붙는다. 다음 `Update()` 가 — 그때 손이 무엇을
쥐고 있든 — 조이기를 재개하고, `phase()` 로 게이팅하는 관찰자는 아무도 지울 수 없는 non-Idle 값을
보게 된다. PI 법칙이 손을 몰지 않는 tick 과 활성화 시점에 `Reset()` 을 부르면 *"이 컨트롤러가 돌지
않으면 FSM 은 Idle"* 이 모든 호출 경로에 의존하는 성질이 아니라 **무조건적 성질**이 된다.

호출처는 두 컨트롤러의 lifecycle (`on_activate`) 과 compute 경로 (PI 가 비활성인 tick) 다.
RT-safe: 고정 크기 루프, 무할당, 무로깅. 상류 필터는 공유·warm 유지이므로 **필터 tail 을 버리지
않는다** — 복귀 첫 tick 이 0 에서 램프하지 않고 안정된 힘을 읽는다 (§2.1 과 같은 이유).

---

> **읽기 전에 — 적응 이득 스케줄링은 현재 배포에서 꺼져 있다.** `K_est_max` 가 추정치의 seed 와
> 같아 `gain_scale` 은 상수 `1/(1 + β)` = 0.769 다. 아래 §3.2 · §4.1 · §6.3 의 β=0.03 분석과
> 실측표는 **적응이 켜진 상태**를 다루며, 그 상태는 실기 힘 노이즈에서 아직 성립하지 않는다
> (§6.6, #426). 지금 로봇이 도는 값을 찾는다면 §4 의 "배포" 열과 §6.6 을 본다.

## 3. 제어 코어 (`ComputeAdaptivePI`)

```
e_f         = f_desired − f_measured
ΔF/Δs       = (f_measured − f_prev) / (s − s_prev)        # |Δs| > 1e-6 일 때만
K_contact   ← α · K_contact + (1−α) · (ΔF/Δs)             # EMA, α = alpha_ema
                                                          #   단 ΔF/Δs > 0 일 때만 갱신.
                                                          #   ≤0 이면 이전 추정치를 유지한다
                                                          #   (0 쪽으로 끌지 않는다)
gain_scale  = 1 / (1 + β · K_contact)                     # β = beta
Kp_eff      = Kp_base · gain_scale
Ki_eff      = Ki_base · gain_scale
∫e          ← clamp(∫e + e_f · dt, ±integral_clamp)       # frozen 일 땐 누적 안 함
ds          = clamp(Kp_eff · e_f + Ki_eff · ∫e, ±ds_max)
```

### 3.1 단위 분석
- `e_f` [N] · `Kp_base` [1/(N·s)] → `[1/s]` ✓ (`ds` 단위)
- `∫e_f dt` [N·s] · `Ki_base` [1/(N·s²)] → `[1/s]` ✓
- 즉 `Kp_base`/`Ki_base` 는 *force error 를 closing rate (s 진행률) 로 매핑*하는 이득

### 3.2 Gain scheduling — β 는 배율이 아니라 루프이득 상한이다

`K_contact_est` 는 "s 를 1만큼 닫으면 force 가 몇 N 오르는가" 의 접촉 강성 추정치다. 부드러운
물체면 작아 `gain_scale → 1` (full gain), 단단한 물체면 커져 `gain_scale` 이 줄고 진동/오버슈트가
억제된다. 실측으로 추정치는 참 강성을 **아래에서** 따라간다 — 배포 파라미터에서 참 강성 10/20/50/100/200 에
대해 9.8/19.8/49.6/99.5/199.4 (상대오차 2.0/1.0/0.8/0.5/0.3%). 저평가는 우연이 아니라 플랜트 점성의
계통 편향이다: Kelvin-Voigt 물체에서 `K_inst = K − D/τ` 이므로 절대 편향은 단단한 물체일수록 크고
(τ 가 작다) 상대 편향은 무른 물체일수록 크다. 실측값이 이 식을 세 자리까지 재현한다.

여기서 `beta` 는 흔히 오해되는 것과 달리 **이득에 곱하는 배율이 아니라 루프이득의 상한**이다.
폐루프 수렴률이

```
λ = K · Kp_base · gain_scale = K · Kp_base / (1 + β·K)
```

이므로 K → ∞ 에서 `λ → Kp_base / β` 로 **포화한다**. 즉 물체가 아무리 단단해도 수렴이 그보다
빨라지지 않고, 최단 시상수는 `τ_min = β / Kp_base` 로 β 혼자 결정한다. 배포 β=0.03 · Kp_base=0.02
에서 `τ_min = 1.5 s`. β 를 올리는 것은 "이득을 조금 낮추는" 것이 아니라 **모든 파지의 속도 상한을
낮추는** 행위다 (§6.3).

> **이력 (2026-08).** 이 스케줄링은 최초 커밋부터 한 번도 동작하지 않았다. `Update()` 가 tick 맨
> 앞 latch 루프에서 `s_prev = s` 를 쓰는데 `ComputeAdaptivePI` 는 `s += ds·dt` 보다 *먼저*
> `Δs = s − s_prev` 를 읽어, `Δs ≡ 0` 이라 `|Δs| > 1e-6` 가드가 영영 통과하지 못했다.
> `K_contact_est` 는 1.0 seed 에 고정됐고 `gain_scale` 은 상수 `1/(1+β)` 였다 — adaptive 가 아니라
> 고정 이득 de-rating. `s_prev` 를 FSM *이후*에 쓰도록 옮겨 고쳤다 (외부에서 읽는 값은 불변:
> tick 이 반환된 시점에 `s_prev` 는 여전히 직전 tick 의 `s`). 이 버그가 6개월 숨어 있었던 이유는
> 추정치를 읽는 소비자가 하나도 없었기 때문이다 — `GraspState` 에도, 로그에도 없어서 27개 테스트가
> 그 위를 통과했다. 지금은 `GraspStiffnessEstimationTest` 가 추정치의 참값 추종과 `alpha_ema` 의
> 유효성을 못박는다. 수정과 동시에 배포 `beta` 를 0.3 → 0.03 으로 재조정했다 — 추정이 살아나면
> `τ_min` 이 15 s 가 되어 BT 예산을 전부 초과하기 때문이다.

### 3.3 Anti-windup
- `integrator_frozen` 은 `ApplyDeformationGuard` 가 `remaining ≤ 0` 또는
  `remaining < 10% · delta_s_max` 일 때 set
- **해제 주체는 `ApplyDeformationGuard` 뿐이다** — `remaining ≥ 10% · delta_s_max` 로 여유가
  회복되면 스스로 푼다 (else 분기). 즉 동결은 guard 가 걸고 guard 가 푸는 닫힌 계약이다
- 그 밖에 플래그가 `false` 가 되는 곳은 **상태 자체가 리셋되는 지점**뿐이다: 접촉 latch
  (Approaching), Contact→ForceControl 진입, `ResetFingers()` (Init / `Reset()` / Idle→Approaching /
  Releasing→Idle)
- **Holding 의 slip 보정은 동결을 풀지 않는다** — 과거에 풀던 시절 연체에서 영구 교착이 났다
  (§2.5, §6.1). `AnomalyDoesNotThawDeformationGuardFreeze` 가 회귀를 잡는다

---

## 4. 파라미터 레퍼런스 (배포 종속)

**코드 default** 는 [grasp_types.hpp](../include/rtc_controllers/grasp/grasp_types.hpp) 의 `GraspParams`
필드 초기값, **배포값**은 `ur5e_p1a`·`ur5e_p1b` 의 `force_pi_grasp:` 블록이다 (두 variant 는 현재
동일). 두 열이 다른 행이 실제로 도는 값은 **배포값**이며, 굵게 표시했다. YAML 은 키가 있는 항목만
덮어쓰므로 (`ApplyForcePiBlock`), 키를 지우면 코드 default 로 되돌아간다.

### 4.1 PI 게인

| 파라미터 | 코드 default | 배포 | 단위 | 영향 | 너무 크면 | 너무 작으면 |
|---|---|---|---|---|---|---|
| `Kp_base` | 0.02 | 0.02 | 1/(N·s) | 비례 응답 | 진동/오버슈트 | 수렴 느림 |
| `Ki_base` | 0.002 | 0.002 | 1/(N·s²) | 정상상태 오차 제거 | 와인드업, 한계사이클 | steady-state offset 잔존 |
| `integral_clamp` | 0.1 | 0.1 | N·s | `∫e` saturation | wind-up | 큰 외란 회복 느림 |

> 설계 노트: 유효 비례 이득 `Kp_base · gain_scale` 은 고정값이 아니라 강성 추정을 따라 내려간다
> (§3.2). 접촉 직후에는 `K_contact_est` 가 1.0 seed 라 `gain_scale ≈ 0.971` → `Kp_eff ≈ 0.0194`
> 로 거의 full gain 이다. 다만 **이 시점의 힘 오차는 최대가 아니라 음수다** — `UpdateContact` 가
> `kForceControl` 진입 시 `f_desired` 를 0 으로 리셋하고 램프가 거기서 출발하므로, 첫 tick 의
> `e_f ≈ 0 − f_measured ≈ −0.8 N` (배포 `f_contact_threshold`) 이고 `ds ≈ −0.016` 으로 잠깐 **여는**
> 방향이다. 추정이 수렴하면 `gain_scale` 은 K=20 에서 0.628, K=200 에서 0.143 까지 내려간다.
>
> 따라서 **`ds_max` 여유가 가장 좁은 순간은 접촉 직후가 아니라 램프 중반의 무른 물체**다: 오차는
> 램프가 목표에 다가갈수록 커지는데 `gain_scale` 은 무른 물체일수록 덜 깎이기 때문이다 (K=10 에서
> 0.769). `alpha_ema=0.95` 의 추정 시상수는 500 Hz 에서 40 ms 라 그 시점에는 이미 수렴해 있다.
> `Kp_base` 를 올릴 때 확인할 지점은 접촉 순간이 아니라 그 구간이다.

### 4.2 Gain scheduling

| 파라미터 | 코드 default | 배포 | 의미 |
|---|---|---|---|
| `alpha_ema` | 0.95 | 0.95 | stiffness EMA 계수. 클수록 추정이 느리고 매끄럽다 |
| `beta` | 0.3 | 0.3 | 루프이득 상한 — `τ_min = β/Kp_base` (§3.2). **적응이 켜질 때 0.03** |
| `K_est_max` | 1.0 | 1.0 | `K_contact_est` 상한. seed 와 같으므로 **적응 비활성** (§6.6) |

튜닝:
- **`beta` 는 속도 상한 손잡이다.** 진동하면 ↑, 느리면 ↓ — 다만 올릴 때 잃는 것은 "이득 조금" 이
  아니라 *가장 단단한 물체에서의 도달 시간 전부*다. 0.03 은 `τ_min = 1.5 s` 로, K∈[10,200] 전
  구간이 BT 의 10 s 예산 안에 들어오도록 잡은 값 (§6.3). 0.1 이상으로 올리기 전에 그 예산을
  먼저 확인할 것
- **`alpha_ema` 는 추정기의 시상수**다. 힘 센서 노이즈가 커서 `K_inst = ΔF/Δs` 가 튀면 ↑ (0.98),
  물체 강성이 파지 중에 변해 추종이 늦으면 ↓ (0.9). 추정이 참값에 붙는 데 걸리는 시간과
  노이즈 민감도의 교환이다
- 두 손잡이는 **축이 다르다**: `alpha_ema` 는 추정이 얼마나 빨리 옳아지는가, `beta` 는 옳아진
  추정으로 얼마나 이득을 깎는가. 진동을 잡을 때는 `beta`, 추정이 헛도는 것 같을 때는 `alpha_ema`

> **무엇을 보고 이 둘을 움직이는가 (#424).** 추정치는 `<secondary>/grasp_state` 의
> `finger_stiffness_est` 로 나온다 — `alpha_ema` 를 조정할 근거는 이 필드의 시계열이지
> 도달 시간이 아니다. 도달 시간은 `beta` 와 뒤섞여 있어 어느 손잡이가 원인인지 못 가른다.
>
> | 관찰 | 해석 | 손잡이 |
> |---|---|---|
> | 파지 내내 정확히 `1.000` | `K_est_max` pin (현재 배포) — 적응 자체가 꺼져 있다 | 없음. §6.6 |
> | 참값 근처에서 매끄럽게 수렴 | 정상 | 없음 |
> | tick 마다 크게 요동 | 노이즈가 `ΔF/Δs` 를 지배 | `alpha_ema` ↑ (0.98) |
> | 접촉 후에도 seed 부근에 오래 머묾 | 추정이 너무 느리다 | `alpha_ema` ↓ (0.9) |
> | 단조 상승 후 안 내려옴 = 부풀어오름 | §6.6 의 흡수 상태. **튜닝으로 못 고친다** | #426 |
> | `0.000` | 이번 tick 에 계산 안 함 (E-STOP). 무른 접촉이 아니다 | — |
>
> `finger_stiffness_est` 는 `finger_filtered_force` 와 **함께** 본다: 후자의 정지 구간
> 표준편차가 §6.6 이 말하는 σ 이고, 요동의 원인이 추정기인지 센서인지를 그 둘의 대조가 가른다.

### 4.3 Force 임계/목표

| 파라미터 | 코드 default | 배포 | 단위 | 의미 |
|---|---|---|---|---|
| `f_contact_threshold` | 0.2 | **0.8** | N | Approaching → `contact_detected` latch 임계값 |
| `f_target` | 2.0 | 2.0 | N | 목표 grip force (YAML 기본 목표) |
| `f_ramp_rate` | 1.0 | **2.0** | N/s | `f_desired` ramp 속도 |
| `f_max_multiplier` | 2.0 | 2.0 | — | Holding 최대 허용 force = `active_target_force · this` |
| `f_slip_fraction` | 0.5 | 0.5 | — | `f_measured < active_target_force · this` 면 grip 상실로 판정 |
| `grip_tightening_rate` | 0.5 | 0.5 | N/s | grip 상실 동안 `f_desired` 상승 속도 |
| `grip_decay_rate` | 0.1 | 0.1 | N/s | `active_target_force` 로 되돌아오는 감쇠 속도 |
| `df_slip_threshold` | 5.0 | 5.0 | N/s | 슬립 판정 `df/dt` 임계 |

> **`f_target` 이 아니라 `active_target_force` 다.** ramp 상한, `f_max_multiplier` 곱셈 기준,
> `f_slip_fraction` 비교 기준, 감쇠 바닥 — 넷 다 `active_target_force` 를 본다. 이 값은
> `CommandGrasp(target_force)` (BT 의 `target_force_N` 포트) 나 `set_target_force()` 로 런타임에
> 바뀌므로, **YAML 의 `f_target` 은 아무도 GRASP 에 목표를 실어 보내지 않을 때의 기본값**일 뿐이다.

`grip_tightening_rate` 와 `grip_decay_rate` 는 같은 축의 짝이고 **둘의 비가 조임/풀림 비대칭**이다
(배포 5:1). anomaly 지속 중 순 상승률은 `tightening − decay` = 0.4 N/s (§2.5).

튜닝 가이드:
- `f_contact_threshold`: 센서 noise floor 의 **5σ** 위로 (false latch 방지). 너무 높으면 가벼운 물체
  인식 실패. 배포가 코드 default 의 4배인 것이 실측 반영 결과다
- `f_target`: 물체 grip-failure 한계의 50% 정도에서 시작
- `f_ramp_rate`: `settle_*` 와 독립이다 (§4.5) — 램프 속도는 도달 시간에만 영향

### 4.4 Rate / saturation

| 파라미터 | 코드 default | 배포 | 단위 | 의미 |
|---|---|---|---|---|
| `ds_max` | 0.05 | 0.05 | 1/s | PI 출력 saturation |
| `delta_s_max` | 0.15 | 0.15 | — | 접촉 후 추가 closing 허용 한계 (변형 가드) |

튜닝:
- `ds_max` 너무 작음 → 응답 느림; 너무 큼 → deformation overshoot
- `delta_s_max` 너무 작음 → stiff object 에서 force 형성 전에 한계 도달 (§6.2); 너무 큼 → 손가락이
  물체 파괴

### 4.5 FSM 타이밍

| 파라미터 | 코드 default | 배포 | 단위 | 의미 |
|---|---|---|---|---|
| `approach_speed` | 0.2 | **0.4** | 1/s | Approaching 의 `ds/dt` (배포 기준 open→close 2.5 s) |
| `release_speed` | 0.3 | 0.3 | 1/s | Releasing 의 `ds/dt` |
| `contact_settle_time` | 0.1 | 0.1 | s | Contact phase dwell |
| `settle_epsilon` | 0.1 | **0.5** | N | 수렴 판정 threshold (배포는 `f_target=2.0` 의 25%) |
| `settle_time` | 0.3 | 0.3 | s | 수렴 유지 시간 |

> **`settle_*` 는 `f_ramp_rate` 와 무관하다.** dwell 은 램프가 목표에 닿은 뒤에만 시작하므로
> (§2.4), 이 둘을 정할 때 램프 속도를 고려할 필요가 없다. 이 보장이 코드에 들어오기 *전*에는
> `settle_time < 2 · settle_epsilon / f_ramp_rate` 인 설정이 램프 도중에 승급했고, 배포값
> (0.3 s vs 0.5 s) 이 정확히 그 영역이라 실측 Holding 진입 `f_desired` 가 물체 강성과 무관하게
> 0.60~0.68 N (목표 2.0 N) 이었다. 지금은 램프 완료가 선행 조건이라 이 조합이어도 안전하다.

### 4.6 필터

| 파라미터 | 코드 default | 배포 | 단위 | 의미 |
|---|---|---|---|---|
| `lpf_cutoff_hz` | — | 25.0 | Hz | Bessel 4차 LPF cutoff |

**`GraspParams` 에 없다.** YAML 키는 `force_pi_grasp.lpf_cutoff_hz` 그대로지만, 값은
`DemoSharedConfig::force_pi_lpf_cutoff_hz` 를 거쳐 **컨트롤러의 축별 필터 뱅크**로 간다 (§1).
25 Hz 의 group delay ≈ 20 ms 로 force PI 의 outer-loop bandwidth 대비 여유가 충분하다.

이 cutoff 는 contact_stop 의 `fsm.contact_stop_force_lpf_cutoff_hz` (배포 50 Hz) 와 **독립**이다.
둘은 같은 guarded 입력을 받는 별개 뱅크이며, 요구가 반대이기 때문에 합치지 않는다 — contact_stop 은
지연이 곧 손 이동 거리인 래치라 저지연을, force_pi 는 tick 당 미분으로 slip 을 보는 서보라
저노이즈를 원한다. 가드(`fsm.contact_stop_force_guard_*`)는 반대로 **공유**한다: wire 아티팩트
거부는 신호원의 성질이라 소비자별로 다를 이유가 없다.

---

## 5. 권장 튜닝 워크플로 (배포 종속)

단계마다 `<secondary>/grasp_state` 토픽 (`grasp_phase`, `finger_filtered_force`, `finger_s`,
`finger_force_error`, `finger_stiffness_est`, `grasp_target_force`) 을 CSV 로 로깅하여 검증한다.

### Step 1 — 센서 baseline
- 자유공간에서 hand 를 여러 차례 open/close 시키며 `force_magnitude[..]` 의 noise σ 측정
- **설정**: `f_contact_threshold ≥ 5σ`. 배포값 0.8 N 이 그 결과이며, 코드 default 0.2 N 은 실기
  노이즈에 비해 낮다

### Step 2 — Approach 단독 검증
- 테스트 물체 없이 `CommandGrasp()` → `s` 가 1.0 까지 ramp 된 뒤 **kApproaching 에 머무르는지**
  확인 (kIdle 로 abort 되지 않는다). 되돌리려면 `CommandRelease()` — 이 검증이 곧 release 경로 점검
- 이어서 손 안에 물체를 넣어 늦은 접촉이 잡히는지 확인 → `s_at_contact = 1.0` 으로 Contact 전이
- 테스트 물체 있음 → Contact 전이 시점에 앞 두 손가락의 `f_measured` 가 노이즈 수준을 충분히
  넘는지 확인
- 충격이 크면 `approach_speed` ↓ (배포 0.4 → 0.2 등)

### Step 3 — Open-loop ramp 검증
- 임시로 `Kp_base=0`, `Ki_base=0` 으로 설정 (또는 테스트 YAML 분리)
- Contact → ForceControl 진입 후 `s` 가 변하지 않아야 함 (PI 출력 0)
- `f_measured` 가 (상류) LPF 통해 안정적으로 관측되는지 확인. 발행되는
  `GraspState.finger_filtered_force` 가 바로 이 값이다
- `f_desired` ramp 동작과 실제 `f_measured` 의 괴리 관찰

### Step 4 — P 이득 튜닝
- `Ki_base = 0` 유지
- `Kp_base` 를 0.005 → 0.04 로 점증하며 step response 관찰
- 진동 시작 직전 값의 **절반** 을 최종 `Kp_base` 로 채택
- 목표: 오버슈트 ≤ 5%, `0.5 s` 이내 접근 (강체 기준 — 무른 물체는 §6.3 의 시상수가 지배한다)

### Step 5 — I 이득 튜닝
- Step 4 에서 결정된 `Kp_base` 유지
- steady-state 잔존 오차를 보고 `Ki_base = Kp_base / τ_i`, `τ_i ≈ 5 · settle_time` 부터 시작
- `integral_clamp ≈ 0.5 · ds_max / Ki_base` 로 설정 (wind-up 방지)

### Step 6 — `beta` 튜닝
- 강체(금속) + 연체(스펀지) 두 종류 테스트 물체로 교차 검증
- 진동 → `beta ↑` (0.03 → 0.05 → 0.1), 응답 부족 → `beta ↓` (0.03 → 0.02)
- **한 스텝 올릴 때마다 가장 단단한 물체의 도달 시간을 다시 잰다** — `beta` 는 배율이 아니라
  속도 상한이라 `τ_min = β/Kp_base` 가 그만큼 같이 늘어난다 (§3.2). 0.1 이면 `τ_min = 5 s` 로
  BT 의 10 s 예산이 사실상 소진된다 (§6.3)
- 강성 적응이 동작하므로 한 `beta` 로 두 물체군을 덮는 것이 원칙이다. 그래도 안 되면 `beta` 보다
  `alpha_ema` 를 먼저 본다 — 물체군 간 차이가 정상상태 이득이 아니라 추정 수렴 속도에서 오는
  경우가 있다 (§6.4)

### Step 7 — Deformation guard 점검
- 대상 물체 geometry 에서 `s_at_contact` 부터 `delta_s_max` 만큼 닫혔을 때 물체가 안전한지 물리 검증
- 파손 위험 시 `delta_s_max` ↓, 반대로 stiff object 에서 force 형성 부족 시 ↑ (§6.2)

### Step 8 — Anomaly / 장시간 hold 검증
- Holding 중 물체를 살짝 빼내어 slip 유도 → `df_slip_threshold` trigger 여부, `f_desired` 상승 후
  `grip_decay_rate` 로 복귀하는지 확인
- 30 s 이상 hold 에 외란 인가 → `f_desired` 가 `f_max_multiplier` 상한에 붙어 머무르지 않는지 확인

---

## 6. 알려진 이슈 / 주의사항

### 6.1 Deformation guard 의 integrator 동결은 자기 치유한다
`ApplyDeformationGuard` 는 `remaining ≥ 10% · delta_s_max` 로 여유가 회복되면 `integrator_frozen`
을 스스로 해제한다. 동결은 guard 가 걸고 guard 가 푸는 닫힌 계약이며, 다른 어떤 코드도 이 플래그를
건드리지 않는다 — anomaly 분기가 이걸 풀던 시절에 연체에서 영구 교착이 났다 (§2.5, §3.3).
**해결된 항목이며 현재 재발 경로는 없다.**

### 6.2 물체가 물러서 목표힘에 도달할 수 없는 경우
`delta_s_max` 가 접촉 후 추가 closing 을 제한하므로 도달 가능한 최대 정적 힘은
`f_at_contact + K · delta_s_max` 다 (K = 물체 강성 [N/Δs], `f_at_contact` = latch 시점의 **정적**
힘). 순수 탄성체면 `f_at_contact ≈ f_contact_threshold` 지만, 점성이 있는 물체는 접근 속도가 임계값
통과에 기여하므로 그보다 낮다.

이보다 `settle_epsilon` 이상 모자라면 `all_settled` 가 성립하지 않아 **kForceControl 에 머문다** —
거짓 Holding 대신 정직한 미수렴이다. BT 는 `grasp_timeout_ms` 로 이를 실패로 처리한다.

- 실측 (배포 파라미터, `f_target=2.0`, Kelvin-Voigt 점성 포함): K≥10 이면 도달, K≤7 이면 미도달
- 점성 없는 물체라면 경계는 `K ≥ (f_target − settle_epsilon − f_contact_threshold) / delta_s_max`
  ≈ 4.7 로 내려간다. 즉 **경계값은 물체 모델에 따라 2배 이상 움직인다** — 위 K 수치를 다른 물체에
  그대로 옮기지 말 것
- 조정 축은 `delta_s_max` ↑ 또는 `f_target` ↓ 이며, 전자는 물체 파괴 위험과 직접 맞바꾼다

### 6.3 Holding 진입까지의 소요 시간
수렴 판정이 정직해진 뒤로 Holding 진입은 **feedforward 구간 + 폐루프 수렴**의 합이다.

feedforward 하한은 물체와 무관하게 고정이다 — `contact_settle_time` (0.1 s) + 램프
(`f_target / f_ramp_rate` = 1.0 s) + `settle_time` (0.3 s) = **1.4 s**.

폐루프 부분의 시상수는 `τ = 1/λ = (1 + β·K) / (K · Kp_base)` 다 (§3.2). K 에 단조 감소하되
`τ_min = β/Kp_base = 1.5 s` 에서 **포화한다** — 강성 추정이 살아 있으므로 단단한 물체라고 무한히
빨라지지 않는다. 배포 파라미터 실측 (Kelvin-Voigt 물체, D=1.0, 500 Hz):

| K [N/Δs] | 200 | 100 | 50 | 20 | 10 | 7 |
|---|---|---|---|---|---|---|
| τ (모델) | 1.75 s | 2.0 s | 2.5 s | 4.0 s | 6.5 s | — |
| Holding 진입 (실측) | 3.64 s | 4.03 s | 4.64 s | 6.41 s | 9.34 s | 도달 못 함 |
| `K_contact_est` (실측) | 199.4 | 99.5 | 49.6 | 19.8 | 9.8 | 6.4 |

모델 `1.4 s + τ` 는 실측의 **하한**으로 읽는다 (0.5–1.4 s 낙관적) — 램프와 폐루프가 겹치는 구간,
그리고 접촉 직후 추정이 아직 seed 근처인 구간을 세지 않기 때문이다. 경향은 정확하다:
무른 물체일수록 폐루프 항이 지배한다.

- BT `grasp_timeout_ms` (예: [pick_and_place_force_pi.xml](../../ur5e_bt_coordinator/trees/pick_and_place_force_pi.xml) 의 10000) 와 함께 봐야 한다 — K=10 의 9.34 s 는 **예산의 93%** 로,
  이보다 무른 물체는 timeout 전에 Holding 에 못 든다. 여유가 필요하면 `beta` 를 내리거나
  `settle_epsilon` 을 완화한다
- **K ≤ 7 의 미도달은 `beta` 와 무관하다.** 그 경계는 §6.2 의 도달 가능 최대 정적 힘
  `f_at_contact + K · delta_s_max` 와 수렴 판정 문턱 `f_target − settle_epsilon` (= 1.5 N) 의
  관계다. 이 rig 의 `f_at_contact` 는 **0.4 N** 이고 (임계값 0.8 N 중 `D · approach_speed` = 0.4 N
  이 점성 몫이라 정적 성분은 그 절반), K=7 이면 `0.4 + 1.05 = 1.45 N` 으로 1.5 N 에 **0.05 N**
  모자란다. 같은 식이 주는 경계는 `K ≥ (1.5 − 0.4)/0.15 = 7.33` 으로, 위 실측의 "K=7 미도달 /
  K=10 도달" 과 정확히 맞는다. `beta` 를 아무리 내려도 이 경계는 움직이지 않는다.
  > `delta_s_max` 를 키워 이 경계를 내릴 때 참고할 여유는 **0.05 N** 이지 `2.0 − K·delta_s_max`
  > 가 아니다 — 후자는 `f_at_contact` 를 빠뜨리고 목표력을 판정 문턱 대신 쓴 값이라 실제의
  > 19 배다. §7 Step 7 이 이 손잡이를 물체 파괴 위험과 맞바꾼다

### 6.4 `K_contact_est` 초기 transient
접촉 latch 시 `K_contact_est` 는 1.0 으로 reset 되므로, 실제 강성이 200 인 물체를 잡아도 **처음
몇 tick 은 `gain_scale ≈ 0.971` 의 거의 full gain 으로 돈다.** 다만 유효 이득이 가장 높은 이 구간이
힘 오차가 가장 큰 구간은 **아니다** — `f_desired` 가 0 에서 램프를 시작하므로 접촉 직후 오차는
음수이고, `ds` 가 `ds_max` 에 가장 가까워지는 것은 램프 중반의 무른 물체다 (§4.1 설계 노트).
`alpha_ema=0.95` 의 추정 시상수가 500 Hz 에서 40 ms 라 그 시점에는 추정이 이미 붙어 있다.

방어는 두 겹이다 — 첫 update 의 `Δs` 가 극히 작아 `K_inst = ΔF/Δs` 가 발산하는 것은
`kDeltaSEpsilon = 1e-6` 가드가 막고, `K_inst ≤ 0` (힘이 줄거나 s 가 후퇴) 인 표본은 EMA 에
반영하지 않아 추정치를 0 쪽으로 끌지 않는다 (§3).

`alpha_ema = 0.95` 에서 추정이 참값에 붙기까지 수십~수백 tick 이 걸리므로, 단단한 물체에서
오버슈트가 관찰되면 **정상상태 이득이 아니라 이 transient 를 먼저 의심한다**. 완화 순서:
`alpha_ema` ↓ (추정을 빨리 수렴시킨다) → `ds_max` ↓ (transient 의 진폭을 직접 자른다) →
`Kp_base` ↓ (전 구간을 느리게 만드므로 마지막). 이 구간의 실기 거동은 아직 미검증이다 —
저장소 테스트가 쓰는 Kelvin-Voigt 모델은 관성이 없어 구조적으로 오버슈트가 나지 않는다.

이 transient 를 실기에서 보는 방법은 `finger_stiffness_est` 를 접촉 latch 시각 기준으로
정렬해 보는 것이다 (#424) — seed 1.0 에서 출발해 언제 참값에 붙는지가 곧 `alpha_ema` 의
유효 시상수다. **단 현재 배포에서는 이 곡선을 볼 수 없다**: `K_est_max` 가 seed 와 같아
추정치가 1.0 을 못 벗어나므로, 이 절을 실기에서 확인하려면 벤치에서 `K_est_max` 를 먼저
올려야 한다 (§6.6 의 표 — `beta` 를 함께 내리지 않으면 예산을 초과한다).

### 6.5 앞 두 손가락 비대칭의 함의
전이가 인덱스 0·1 로 결정되므로 (§1.2):
- **후행 손가락의 spurious latch 방지가 더 중요해진다**: false latch 는 `s_at_contact = 0` 인 상태로
  deformation guard 에 진입시켜 그 손가락을 즉시 동결시킨다. `f_contact_threshold` 를 센서 noise
  floor 대비 충분한 마진으로 (§4.3)
- **후행 손가락의 `s`**: Approaching 이 전이 시점에 freeze 한 값 그대로. 기하학적으로 이상한
  posture 가 나올 수 있으나 제어 로직 자체는 영향 없음
- **비대칭 grasp 시나리오 지원**: 2-finger pinch 가 1급 use case 이므로 적극 활용 가능. 단
  "어느 두 손가락이 그 쌍인가" 는 **고정(인덱스 0·1)** 이며 재정렬로 바꿀 수 없다 — §1.2 경고

---

### 6.6 추정기는 힘 노이즈에 구조적으로 취약하다 (실기 미해결)

**이 절의 내용은 배포 전에 반드시 읽어야 한다.** §6.3 의 실측표는 노이즈가 **정확히 0** 인
플랜트에서 나왔고, 그 조건은 실기에 존재하지 않는다.

`K_inst = ΔF/Δs` 는 tick 단위 차분이다. 배포 파라미터·500 Hz 에서 램프 구간의 tick 당 힘 증분은
**0.4–1.4 mN** 인데, 25 Hz LPF 를 통과한 잔여 센서 노이즈 σ=2 mN 이 같은 차분에 기여하는 양이
**약 1.4 mN** 이다. 즉 **신호와 노이즈가 같은 크기** (SNR ≲ 1) 이고, 이것은 튜닝 문제가 아니라
차분 구조의 문제다.

실측 (배포 파라미터, K=20, 25 Hz LPF 통과 노이즈):

| σ [mN] | `K_contact_est` | `gain_scale` | Holding 진입 |
|---|---|---|---|
| 0 | 19.8 | 0.628 | 6.41 s |
| 1 | 115 | 0.224 | 9.54 s |
| 2 | 1075 | 0.030 | **51.5 s** |
| 20 | 2501 | 0.013 | 미도달 |

부풀어 오른 상태는 **자기 회복하지 않는다** — `gain_scale` 이 붕괴하면 `ds` 가 작아지고, `ds` 가
작아지면 `Δs` 가 작아져 교정 표본 자체가 생기지 않는다 (흡수 상태).

시도했다가 **기각한** 완화책 (모두 실측):

- **`Δs` 하한 게이트** — 틀린 축이다. 단단한 물체일수록 작은 변형으로 힘을 내므로, `Δs` 게이트는
  적응이 가장 필요한 물체에서 추정기를 침묵시킨다 (`0.25·ds_max·dt` 에서 참 K=200 이 seed 1.0 에
  고정)
- **`ΔF` 하한 게이트** — 신호와 노이즈가 같은 크기라 둘을 가르는 문턱이 없다. 노이즈를 막는
  값은 무노이즈 추정기도 함께 죽인다 (5 mN 에서 전 구간 seed 고정)
- **차분 윈도우 확대** — 노이즈 면역은 실제로 개선되지만 (0.1 s 윈도우에서 σ=2 mN 이 무노이즈와
  거의 동일) 물체 의존 편향이 생긴다: 고정 윈도우 0.05 s 에서 K=10 은 20.5 로 2 배 과대평가,
  K=200 은 180 으로 과소평가. K∈[10,200] 전 구간을 만족하는 단일 윈도우가 없고, 무노이즈
  K=10 조차 9.34 s → 11.14 s 로 회귀했다

### 현재 조치: 적응을 끈다

이 때문에 **배포 설정은 적응을 비활성으로 둔다** — `K_est_max = 1.0` 은 `K_contact_est` 의
seed 와 같아 추정치가 오르지 못하고, `gain_scale` 은 상수 `1/(1 + β)` 가 된다. `β = 0.3` 과
짝지어 그 상수는 **0.769** 이며, 이는 추정기가 죽어 있던 시절 이 컨트롤러가 실제로 돌던 값과
정확히 같다. 즉 이 설정은 *새로운* 거동이 아니라 **검증된 기존 거동의 보존**이다.

`β` 와 `K_est_max` 는 **반드시 함께** 움직인다:

| | `K_est_max = 1.0` (pinned) | `K_est_max = 400` (활성) |
|---|---|---|
| **`β = 0.3`** | **← 현재 배포.** `gain_scale` 0.769 | τ_min 15 s — 전 구간 예산 초과 |
| **`β = 0.03`** | `gain_scale` 0.971 — 이득만 26% 상승, 적응 없음 | #426 목표. 단 위 노이즈 문제 선결 |

pinned 거동을 노이즈에 대해 실측한 결과 (K∈{10,20,50,200}, σ∈{0,1,2,20} mN): 도달 시간이
3.3–9.8 s 로 **σ 에 거의 무관**하다. 상수 이득에는 노이즈가 개입할 지점이 없기 때문이다 —
역설적으로 고장 나 있던 추정기가 보호 장치였다. `GraspStiffnessEstimationTest.
DeployedTuningIsPinnedAndNoiseImmune` 이 이 두 성질(추정치가 seed 를 안 벗어남 + 예산 준수)을
고정한다.

**결론**: 적응을 실기에서 켜려면 tick 차분을 그대로 둔 채 상수를 조정하는 것으로는 안 되고,
추정기 자체 (여기 조건을 반영한 가변 윈도우, 최소자승 누적, RLS/칼만 등) 를 다시 설계해야 한다
— #426. 관측 수단은 #424 가 붙였다: `GraspState.finger_stiffness_est` 가 per-finger 추정치를
싣는다. pin 이 걸린 배포에서 이 필드가 계속 `1.000` 인 것이 위 `DeployedTuningIsPinnedAndNoiseImmune`
의 런타임 대응물이며, 값이 움직이기 시작하면 pin 이 풀렸다는 뜻이다. 그때까지 §3.2 · §4.1 · §6.3 의 β=0.03 분석은 **적응이
켜진 상태를 가정한 설계 문서**이며 현재 배포 거동이 아니다.

## 7. 파라미터 변경 절차

1. 해당 variant 의 `demo_shared.yaml` (`config/<variant>/controllers/`) 에서 `force_pi_grasp:`
   블록을 편집
2. `--symlink-install` 빌드(기본값)면 재빌드 불필요. `--no-symlink` 로 빌드했다면 config 가 install
   트리에 복사되므로 **재빌드가 필요하다**
3. 반영 시점: YAML 은 컨트롤러 `on_configure` 에서 읽는다 → 런타임 반영은 컨트롤러 재로드
   (controller switch). 단 `set_params()` / `set_target_force()` 경로로 들어오는 변경은 **다음
   tick 에 즉시 발효**한다 (`GraspParams` 전 필드)
4. 테스트 실행 — colcon 은 반드시 workspace root 에서 (CLAUDE.md §9.1):

```bash
./build.sh -p rtc_controllers
( cd ~/ros2_ws/rtc_ws \
  && source ~/ros2_ws/rtc_ws/src/rtc-framework/repo_scripts/scripts/setup_env.sh >/dev/null 2>&1 \
  && colcon test --packages-select rtc_controllers \
       --ctest-args -R test_grasp_controller --event-handlers console_direct+ )
```

### 7.1 Finger 구성 · 각도 단위

`fingers:` 블록이 손가락 이름 순서·자세·각도 단위를 정한다. 내부 저장은 항상 radian 이다.

```yaml
force_pi_grasp:
  fingers:
    units: "deg"                                    # "deg" | "rad" (default "rad")
    finger_names: ["thumb", "index", "middle", "ring"]   # 생략 시 [thumb, index, middle]
    thumb:
      q_open:  [0.0, 0.0, 0.0, 0.0]                 # 길이가 곧 이 손가락의 DoF
      q_close: [30.0, 60.0, 45.0, 45.0]
    index:
      q_open:  [0.0, 0.0, 0.0]
      q_close: [0.0, 60.0, 45.0]
    middle:
      q_open:  [0.0, 0.0]
      q_close: [60.0, 45.0]
    ring:
      q_open:  [0.0]
      q_close: [45.0]
```

- **`finger_names` 의 순서가 계약**이다 — 인덱스 0·1 이 전이 판정에 쓰이고 (§1.2),
  `f_filtered` span 및 `GraspState` 배열도 같은 순서로 읽힌다
- **DoF 는 `q_open`/`q_close` 배열 길이에서 추론**된다 (둘 중 긴 쪽, `kMaxDoFPerFinger`=8 상한).
  손가락마다 달라도 된다 — 위 예시가 `ur5e_p1b` 의 실제 구성 (4/3/2/1)
- 이름이 `finger_names` 에 있으나 블록이 없으면 그 손가락은 건너뛴다 (DoF 0)
- `units` 생략 또는 `"rad"`/`"radians"` → 스케일 없음. `"deg"`/`"degrees"` → `π/180` 곱해 rad 저장.
  알 수 없는 값 → 경고 후 rad fallback. 단위는 블록 전체 일괄 적용 (손가락별 지정 불가)

### 7.2 제거된 키 — `grip_tightening_ratio`

`grip_tightening_ratio` 는 제거됐고, 남아 있으면 로더가 **`std::runtime_error` 를 던진다**
(`ApplyForcePiBlock`). 조용히 무시하지 않는 이유는 값이 아니라 *단위*가 바뀌었기 때문이다 — 옛
필드는 tick 당 `f_desired *= (1 + ratio)` 로 적용돼 slip 이 만드는 grip force 가 `control_rate` 의
함수였다 (0.15 면 500 Hz 에서 22 ms 만에 상한 도달). 자동 변환이 불가능하므로 실패시키는 것이
유일하게 옳다. **`grip_tightening_rate` [N/s] 로 대체**하되 값은 새로 정해야 한다.

같은 변경에서 `grip_decay_rate` 가 YAML 로 노출됐다 — 그 전에는 키를 적어도 무시되고 default
0.1 N/s 가 돌았다. 둘은 같은 축의 짝이므로 함께 설정한다 (§4.3).

## 8. 실기 계측 runbook — 힘 노이즈 σ 와 추정기 실측 (#428)

§6.6 은 두 가지를 **가정**으로 남겼다: 실기 힘 노이즈 σ 의 값, 그리고 "단단한 물체에서 진동이
난다" 는 전제. 저장소의 Kelvin-Voigt 플랜트는 관성이 없어 후자를 재현할 수 없으므로 둘 다
실기에서만 답이 나온다. #426 (추정기 재설계) 의 수용 기준이 이 두 값에 의존하므로 **이 절이
#426 의 선행조건**이다.

계측 채널은 `grasp_diag.csv` 다 (#428). `ros2 topic echo --csv` 가 아닌 이유는 500 Hz 에서
Python echo 가 샘플을 떨구기 때문이다 — σ 는 무작위 drop 을 견디지만 추정기 transient 는 손상된다.

> ⚠ **`<device>_sensor.csv` 의 `force_filtered_*` 로 σ 를 재지 말 것.** 그것은 contact_stop
> 뱅크(배포 50 Hz)이고, Force-PI 법칙과 추정기가 읽는 것은 **별개 뱅크**(`f_measured_*`,
> 배포 25 Hz, 자체 YAML cutoff)다. 두 컬럼 이름이 모두 "force" 라서 틀린 레인에서 잰 σ 는
> 아무 증상 없이 그럴듯한 숫자를 준다. `grasp_diag.csv` 의 `f_measured_*` 가 유일하게 옳은 출처다.

### 8.0 Preflight — 배포 확인 (제어 PC)

**env 재source 는 배포가 아니다.** 이전 실기 세션에서 옛 코드를 3회 재측정한 사례가 있고 그중
한 번은 잔존 상태 때문에 `rc=0` 까지 나와 fix 가 통한 것처럼 보였다. **설치 트리를 직접 grep** 한다.

```bash
WS=~/ros2_ws/demo_ws                      # 제어 PC 워크스페이스 (rtc-framework 체크아웃)
REPO=$WS/src/ur5e-rt-controller

# 1) 같은 저장소인지 + b6c8dc00 이후인지
git -C $REPO remote -v && git -C $REPO log --oneline -3

# 2) 최신화 + 빌드 (colcon 은 반드시 ws root 에서 — CLAUDE.md §9.1)
git -C $REPO pull
( cd $WS && source $REPO/repo_scripts/scripts/setup_env.sh >/dev/null 2>&1 \
    && colcon build --packages-select rtc_msgs rtc_controllers integrated_bringup rtc_tools )

# 3) 설치 트리에 실제로 들어갔는지 — 셋 다 hit 이어야 한다
grep -rl finger_stiffness_est $WS/install/rtc_msgs/          # #424 토픽 필드
grep -rl GraspDiagLog        $WS/install/integrated_bringup/ # #428 POD + YAML
grep -rl grasp_diag          $WS/install/rtc_tools/          # #428 플로터
```

셋 중 하나라도 비면 **측정하지 말고 빌드부터 고친다** — 없는 컬럼은 조용히 안 나올 뿐이다.

기동 후 `on_configure` INFO 한 줄로 채널이 살았는지 확인한다:

```
[grasp_diag] enabled — grasp_diag.csv, 3 finger column set(s): thumb, index, middle. ...
```

`disabled` 로 뜨면 사유가 같은 줄에 있다 (`force_pi_grasp` 블록 부재 / 핑거 센서 없음).

### 8.1 접촉 성립 확인 (30초) — **긴 세션 전에 반드시**

긴 세션을 돌린 뒤 분석 단계에서야 "접촉이 한 번도 안 났다" 를 발견하면 그 세션은 통째로 버린다.
2026-08-13 첫 실기 세션(p1b, 58036 tick)이 정확히 그렇게 소모됐다. 짧게 grasp 를 한 번 걸고
`--stats` 두 줄만 본다.

```bash
# grasp 걸고 몇 초 유지 → release → 세션 종료 후
plot_rtc_log <세션>/controllers/demo_joint_controller/grasp_diag.csv --stats | head -20
```

| 기대 | 실패 시그널 |
|---|---|
| `Phase occupancy` 에 **`holding`** 이 있다 | `idle`/`approaching`/`releasing` 만 있다 → **접촉 래치 실패** |
| `Stationary hold: N ticks` 의 N > 0 | `0 ticks` → σ 를 못 잰다 |
| | `force_control` 이 지배적인데 `holding` 이 없다 → 래치는 됐으나 **settle 불가** (아래) |

**접촉 래치 실패면 손가락별 최대 힘을 `f_contact_threshold` 와 대조한다:**

```bash
python3 -c "
import pandas as pd; d=pd.read_csv('<...>/grasp_diag.csv'); v=d[d.valid==1]
print(v[[c for c in d.columns if c.startswith('f_measured_')]].max())
print(v[[c for c in d.columns if c.startswith('s_')]].max())"
```

전이 판정은 **인덱스 0·1** 두 손가락만 본다 (§1.2). 그래서 *실제로 물체를 무는 쌍*과 *판정하는
쌍*이 다르면 다른 손가락이 아무리 세게 눌려도 진행하지 않는다. 첫 세션의 실측이 그 사례다
(p1b, `f_contact_threshold` 0.8 N):

| | thumb | index | middle | ring |
|---|---|---|---|---|
| max force [N] | 0.983 ✓ | **0.446 ✗** | 1.359 ✓ | 0.125 ✗ |

`s` 는 네 손가락 모두 1.0 (완전 폐쇄) 이었다. 즉 **다 닫아도 index 가 임계의 56% 에 그쳤고**,
물체는 thumb–middle 사이에 물려 있었다. 판정 쌍은 thumb+index 이므로 래치가 안 걸린다.
**처방은 물체를 index 쪽으로 옮기는 것**이지 손가락 순서를 바꾸는 것이 아니다 (§1.2 경고 참조).

> ⚠ **`f_contact_threshold` 만 낮추지 말 것.** 임계를 내리면 래치는 되지만 그 손가락이 곧바로
> settle 검사 대상이 되고(§2.4), `|f_target − f_measured| ≤ settle_epsilon` 을 만족 못 하면
> **ForceControl 에 갇힌다**. 위 실측에서 임계만 0.3 으로 내렸다면 index 는 래치된 뒤
> `f_target` 1.0 · `settle_epsilon` 0.5 기준 0.5 N 이 필요한데 최대가 0.446 이라 영원히
> 미달이었을 것이다. 그리고 그 상태는 지금보다 **진단이 어렵다** — `force_control` 이 90% 로
> 찍혀 "돌고는 있는데 왜 안 끝나지" 가 된다. 내려야 한다면 `f_target` 을 **함께** 내린다.

> **s 가 1.0 에 포화한 채로도 σ 는 잴 수 있다** — 기계적으로 물려 정지한 유지 구간은 서보 운동이
> 분산에 안 섞이므로 오히려 깨끗하다. 다만 **S2(진동 유무)는 그 상태로 판정할 수 없다**:
> 서보가 조절하지 않으면 진동이 날 수도 안 날 수도 없다. S2 를 보려면 `s < 1.0` 에서 목표힘에
> 도달해야 한다.

### 8.2 세션 3개

세 시나리오 모두 `grasp_controller_type: force_pi` 에서 돈다. **variant 마다 기본값이 다르다** —
`ur5e_p1a` 는 `force_pi` 로 배포되지만 **`ur5e_p1b` 는 `none`** 이므로 아래 `param set` 이 선택이
아니라 **필수**다 (게인·임계는 두 variant 가 동일하고 다른 것은 배포 모드와 손가락 구성뿐이다:
p1a 3핑거 / p1b 4핑거 `[thumb, index, middle, ring]`). 아래 S3 의 YAML 경로도 실제 variant 로
바꿔 읽는다. 모드가 다르면 행은 남되
`valid=0` 이고 per-finger 값이 전부 0 이다 — 그 자체가 "PI 법칙이 안 돌았다" 는 판정이다.

```bash
ros2 param set /demo_joint_controller/demo_joint_controller grasp_controller_type force_pi
ros2 service call /demo_joint_controller/grasp_command \
  rtc_msgs/srv/GraspCommand "{command: 1, target_force: 2.0}"     # command 1 = GRASP
# ... 시나리오 수행 ...
ros2 service call /demo_joint_controller/grasp_command \
  rtc_msgs/srv/GraspCommand "{command: 2, target_force: 0.0}"     # command 2 = RELEASE
```

| # | 무엇을 | 산출물 / 판정 |
|---|---|---|
| **S2** | **단단한 물체 파지, 배포 설정 그대로** (적응 OFF). 목표힘 도달 후 ≥20 s 유지 | **이 실험의 분기점.** ① `k_est_*` 가 전 구간 `1.000` 인가 (pin 유효 = #424 실기 AC) ② `gain_scale_*` 가 상수 0.769 인가 ③ **`f_measured_*` 에 진동/오버슈트가 실제로 나는가** ← #426 의 존재 이유를 결정한다 |
| **S1** | 자유공간에서 파지 유지 (물체 없이 접촉 상태 ≥30 s), 정지 | Holding 구간 `f_measured_*` 의 표준편차 = **σ**. §6.6 표는 0/1/2/20 mN 을 가정했다 — 실제로 어디 떨어지는지가 산출물 |
| **S3** | 벤치, `K_est_max` 상향 + `beta` 동반 인하 (**둘은 한 쌍**, §6.6) | `k_est_*` 가 seed 를 벗어나 물체 강성으로 수렴하는가 (#424 AC 나머지 절반) · `k_inst_raw_*` 의 산포가 §6.6 의 SNR ≲ 1 주장과 맞는가 |

**S2 를 S1 보다 먼저 본다.** 진동이 안 나면 #426 의 범위는 재설계가 아니라 **적응 제거**가 되고,
그 경우 S3 는 돌 필요조차 없다.

S3 의 값 변경 — **ROS 파라미터가 아니라 YAML + 컨트롤러 재로드**다. `grasp_controller_type` 만
파라미터로 노출돼 있고 `force_pi_grasp` 블록의 값(`K_est_max` · `beta` · 게인 전부)은 노출되지
않는다. `set_params()` 는 C++ API 이며 ROS 표면이 없으므로, 실기에서 이 값을 바꾸는 경로는
`on_configure` 재실행뿐이다 (§7 3번).

```bash
# 1) 제어 PC 에서 YAML 편집 — 두 값은 반드시 함께 움직인다.
#    K_est_max 만 올리면 tau_min = beta/Kp_base 가 예산을 넘긴다 (§6.6 표)
$EDITOR $REPO/integrated_bringup/config/ur5e_p1a/controllers/demo_shared.yaml
#      force_pi_grasp:
#        K_est_max: 400.0
#        beta: 0.03

# 2) --symlink-install 빌드(기본값)면 재빌드 불필요. --no-symlink 였다면 재빌드 (§7 2번)

# 3) 컨트롤러를 재로드해 on_configure 를 다시 태운다 (controller switch)
```

반영 여부는 추측하지 말고 **CSV 로 확인한다** — 아래 §8.4 의 `beta` / `K_est_max` 줄이 그 run 의
실효값을 그대로 찍는다. 값이 안 바뀌었으면 S3 는 S2 를 한 번 더 돌린 것에 불과하다.

> `grasp_diag.csv` 는 `beta` · `alpha_ema` · `K_est_max` 를 **매 tick** 기록한다. 그 run 의 YAML
> 없이도 어느 설정에서 나온 데이터인지 파일만 보고 알 수 있고, 재로드 없이 편집만 한 경우가
> 데이터에서 바로 드러난다.

### 8.3 수거 — 텍스트만

```bash
S=$(ls -dt $WS/logging_data/*/ | head -1)   # 방금 세션. RTC_SESSION_DIR 를 줬으면 그 경로
tar czf ~/grasp_session_$(basename $S).tgz \
  -C "$(dirname $S)" "$(basename $S)/controllers" \
  "$(basename $S)"/*.yaml 2>/dev/null
```

`controllers/demo_joint_controller/` (task 컨트롤러를 썼으면 `demo_task_controller/`) 아래에
`grasp_diag.csv` · `<hand>_sensor.csv` · `pull_estimator.csv` 가 함께 들어간다. 이 디렉토리 이름은
variant 가 아니라 **컨트롤러별 고정 문자열**이다 (`ControllerLogSet log_set_{"demo_joint_controller"}`)
— `ur5e_p1a` 같은 variant 이름이 아니므로 찾을 때 헷갈리지 말 것. 세 파일은 `tick` 컬럼 (CM RT loop iteration) 으로 정렬되므로 **같은 run 에서
25 Hz 와 50 Hz 레인을 교차 검증**할 수 있다 — 위 ⚠ 의 함정을 데이터로 직접 확인하는 수단이다.

### 8.4 dev PC 분석

```bash
tar xzf grasp_session_*.tgz
G=<세션>/controllers/demo_joint_controller/grasp_diag.csv
plot_rtc_log $G --stats     # 1차 판정: 숫자
plot_rtc_log $G --no-show   # 그림 (세션 plots/ 로)
```

`--stats` 가 찍는 것과 각 줄이 답하는 질문:

| 출력 | 무엇을 판정하는가 |
|---|---|
| `Dropped rows (tick gaps): N` | **0 이 아니면 그 run 은 무손실이 아니다.** 행 하나 = tick 하나이므로 gap 은 SPSC drop 이며, E-STOP tick 은 gap 이 아니라 `valid=0` 행으로 남는다 |
| `valid: X% of ticks` | PI 법칙이 실제로 돈 비율. 낮으면 모드가 force_pi 가 아니었거나 E-STOP 이 걸렸다 |
| `Phase occupancy` | Holding 구간이 실제로 존재하는지. 없으면 σ 는 **정의되지 않는다** (아래) |
| `beta` / `alpha_ema` / `K_est_max` | 그 run 의 실효 설정. `CHANGED during run` 이면 구간을 나눠 봐야 한다 |
| **`sigma= … mN`** (핑거별) | **§6.6 표의 σ 열.** #426 수용 기준 1 이 가설에서 실측으로 바뀌는 지점 |
| `k_est: … [CONSTANT]` | `[CONSTANT]` 면 pin 이 살아 있다 (S2 기대값). S3 에서 이게 붙어 있으면 param set 이 안 먹은 것 |
| `samples accepted by the guards: X%` | 가드가 표본을 얼마나 기각하는지. 낮으면 추정기는 "돌고 있지만 굶고 있는" 상태다 |
| `k_inst_raw … sigma=` | 원시 표본의 산포. 이것이 `k_est` 자체 크기와 비슷하면 §6.6 의 **SNR ≲ 1** 이 실기에서 확인된 것 |

> **σ 는 Holding 구간에서만 잰다.** approach·force ramp 는 설계상 transient 이므로 전 구간
> 표준편차는 노이즈가 아니라 궤적을 재고 몇 배 크게 나온다. `--stats` 가 이 분리를 이미 하며,
> Holding 구간이 없으면 σ 를 출력하는 대신 "undefined for this run" 이라고 말한다.

별도 분석 스크립트를 새로 만들지 않는다 — 그 자리가 `print_grasp_diag_statistics` 다
([design-principles.md](../../agent_docs/design-principles.md) P5).

### 8.5 결과를 어디에 쓰는가

1. **σ 실측값** → §6.6 의 표에 실기 행을 추가하고, #426 수용 기준 1 의 "가설" 표기를 걷는다
2. **S2 의 진동 유무** → #426 의 범위 결정 (재설계 vs 적응 제거)
3. **S2/S3 의 `k_est` 거동** → #424 가 남긴 실기 AC 종결
