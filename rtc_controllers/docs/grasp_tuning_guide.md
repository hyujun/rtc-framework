# Force-PI Grasp Controller — 튜닝 가이드

본 문서는 `rtc::grasp::GraspController` (adaptive PI force controller) 의 내부 구조·FSM·설계 파라미터와 권장 튜닝 절차를 정리한다. `ur5e_bringup` 의 `demo_joint` / `demo_task` 컨트롤러가 `grasp_controller_type: "force_pi"` 로 동작할 때 활성화된다.

- 구현: [grasp_controller.hpp](../include/rtc_controllers/grasp/grasp_controller.hpp), [grasp_controller.cpp](../src/controllers/grasp/grasp_controller.cpp)
- 타입/파라미터: [grasp_types.hpp](../include/rtc_controllers/grasp/grasp_types.hpp)
- YAML: [demo_shared.yaml](../../ur5e_bringup/config/controllers/demo_shared.yaml) (`force_pi_grasp:` 블록)
- 로더: [demo_shared_config.cpp](../../ur5e_bringup/src/controllers/demo_shared_config.cpp)
- 테스트: [test_grasp_controller.cpp](../test/test_grasp_controller.cpp)

---

## 1. Controller 구조 개요

- **대상**: 3-finger hand (thumb / index / middle), 손가락당 3 DOF (MCP_AA, MCP_FE, DIP_FE)
- **제어 변수**: 손가락별 스칼라 grasp parameter `s ∈ [0, 1]`
  - `q(s) = (1 − s) · q_open + s · q_close` 선형 보간
  - PI 출력은 관절 토크가 아니라 closing 진행도의 미분 `ds` → 위치 제어 하드웨어에서도 힘 제어 가능
- **피드백**: fingertip force sensor 의 norm `f_raw[3]` → Bessel 4차 LPF → `f_measured`
- **호스팅**: 500 Hz RT 루프에서 `Update(f_raw, dt)` 호출 ([demo_joint_controller.cpp:421](../../ur5e_bringup/src/controllers/demo_joint_controller.cpp#L421), [demo_task_controller.cpp:664](../../ur5e_bringup/src/controllers/demo_task_controller.cpp#L664))
- **활성 범위**: `phase() != kIdle` 일 때만 hand trajectory 출력의 finger 관절을 덮어씀
- **명령 인터페이스**: `CommandGrasp(target_force)` / `CommandRelease()` (cross-thread atomic flag)

### 1.1 손가락 역할 비대칭 (thumb + index 중심)

현재 구현은 **thumb(0) + index(1) 두 손가락만으로 grasp 성공/실패를 판정**한다. middle(2) 은 접촉 여부와 무관하게 phase 진행을 막지 않는다. 이는 다음 두 가지 설계 의도를 반영한다.

1. **2-finger pinch grasp 를 1급으로 지원** — thumb/index 대향 파지가 실제 조작 대부분을 차지.
2. **middle 의 late contact 을 선택적 참여로** — 접촉하면 force control 에 편입, 접촉 못 해도 grasp 실패로 판정하지 않음.

그 결과 각 phase 에서 middle 은 "참여자" 또는 "관찰자" 역할을 가변적으로 수행한다 (§2 상세).

---

## 2. 상태 머신 (`GraspPhase`)

```
            CommandGrasp                 thumb+index 접촉            contact_settle_time 경과
Idle ───────────────────► Approaching ─────────────────────────► Contact ────────────────────┐
   ▲                            │                                                             │
   │                            │ thumb 또는 index 가 s≥1.0 & 미접촉                            │
   │                            └────► (실패) Idle                                              │
   │                                                                                          ▼
   │                          CommandRelease                                             ForceControl
   │              ┌──────────────────────────────────────────────────────────────────┐        │
   │              │                                                                  │        │
   │              │                                                                  ▼        │
   │        Releasing ◄── (모든 s ≤ 0.01 → Idle) ── Holding ◄── all_settled & settle_time 충족
   │                                                  │
   │                                                  │ slip 또는 force drop
   │                                                  └─► f_desired ↑ (grip tightening)
   │                                                  └── CommandRelease ──► Releasing
   └──────────── 모든 s ≤ 0.01 ────────────────────────┘
```

### 2.1 Idle (`UpdateIdle`)
- `grasp_requested_` 플래그 set → finger state 전부 reset, force LPF reset 후 `kApproaching` 진입
- 그 외는 현 자세 hold. `release_requested_` 는 consume 후 무시

### 2.2 Approaching (`UpdateApproaching`)
- 접촉되지 않은 finger 만 `s += approach_speed · dt`, `s ≤ 1.0` clamp
- finger 별 `f_measured > f_contact_threshold` 면 `contact_detected=true` latch, `s_at_contact`/`integral_error`/`K_contact_est=1.0` 초기화
- **전이 조건 (현재 구현)**: `fingers_[0].contact_detected && fingers_[1].contact_detected` → `kContact`
- **실패 조건 (현재 구현)**: thumb 또는 index 가 `s ≥ 1.0` 에 도달했는데 미접촉 → `kIdle`
  - middle 의 `s=1.0` 도달은 실패를 유발하지 않음
- middle 의 동작: 전이 시점에 middle 의 `s` 가 어떤 값이든 그대로 freeze 된 채 다음 phase 로 진입

### 2.3 Contact (`UpdateContact`)
- `contact_settle_timer_ ≥ contact_settle_time` 까지 자세 유지
- 만료 시 `f_desired=0`, `integral_error=0`, `integrator_frozen=false` 로 **모든 finger** 재설정 후 `kForceControl` 진입
- 목적: 충돌 직후 force 신호 transient/ringing 을 LPF + 시간으로 안정화
- 튜닝 핸들: `contact_settle_time`

### 2.4 ForceControl (`UpdateForceControl`)
- **접촉된 finger 만** 처리 (`if (!fs.contact_detected) continue;`)
  - `f_desired = min(f_desired + f_ramp_rate · dt, active_target_force)` (force reference ramp)
  - `ds = ComputeAdaptivePI(f, dt)` → `ApplyDeformationGuard(f, ds)` → `s += ds · dt`, clamp `[0,1]`
- 접촉된 모든 finger 가 `|f_desired − f_measured| ≤ settle_epsilon` → `force_settle_timer_ += dt`
- `force_settle_timer_ ≥ settle_time` → `kHolding` 전이
- 어느 하나라도 미수렴이면 `force_settle_timer_ = 0` 리셋
- 도중 `release_requested_` → `kReleasing` 즉시 전이

> **주의**: `all_settled` 는 접촉된 손가락만 집계한다. 즉 thumb+index 2 손가락 기준으로 판정. middle 이 접촉 못 한 경우 middle 은 settle 검사에서 제외된다 (PI 가 돌지 않으므로 의미상 일관).

### 2.5 Holding (`UpdateHolding`)
- ForceControl 와 동일한 PI + deformation guard 로 `s` 유지 (접촉된 finger 만)
- **이상감지 (per-finger)**:
  - `df/dt < -df_slip_threshold` (슬립 의심) OR
  - `f_measured < 0.5 · active_target_force` (목표 절반 이하)
  - 중 하나라도 만족하면 `f_desired ← min(f_desired · (1 + grip_tightening_ratio), f_target · f_max_multiplier)`, `integrator_frozen=false` 로 추가 closing 허용
- `release_requested_` → `kReleasing`
- ⚠️ **설계상 한계**: anomaly 가 사라져도 `f_desired` 는 자동으로 내려오지 않는다. 누적 grip-tightening 은 영구 적용.

### 2.6 Releasing (`UpdateReleasing`)
- **모든 finger** (접촉 여부 무관) `s -= release_speed · dt`, `s ≥ 0` clamp
- 모든 `s ≤ 0.01` 이면 finger state 전체 reset 후 `kIdle`

---

## 3. 제어 코어 (`ComputeAdaptivePI`)

```
e_f         = f_desired − f_measured
ΔF/Δs       = (f_measured − f_prev) / (s − s_prev)        # |Δs| > 1e-6 일 때만
K_contact   ← α · K_contact + (1−α) · max(0, ΔF/Δs)        # EMA, α = alpha_ema
gain_scale  = 1 / (1 + β · K_contact)                      # β = beta
Kp_eff      = Kp_base · gain_scale
Ki_eff      = Ki_base · gain_scale
∫e          ← clamp(∫e + e_f · dt, ±integral_clamp)        # frozen 일 땐 누적 안 함
ds          = clamp(Kp_eff · e_f + Ki_eff · ∫e, ±ds_max)
```

### 3.1 단위 분석
- `e_f` [N] · `Kp_base` [1/(N·s)] → `[1/s]` ✓ (`ds` 단위)
- `∫e_f dt` [N·s] · `Ki_base` [1/(N·s²)] → `[1/s]` ✓
- 즉 `Kp_base`/`Ki_base` 는 *force error 를 closing rate (s 진행률) 로 매핑*하는 이득

### 3.2 Adaptive gain scheduling 의미
- `K_contact_est` 는 "s 단위 1만큼 닫았을 때 force 가 얼마나 증가하는가" (N 단위) 의 접촉 강성 추정치
- 부드러운 물체 → `K_contact` 작음 → `gain_scale → 1` (full gain)
- 단단한 물체 → `K_contact` 큼 → `gain_scale` 감소 → 진동/오버슈트 억제
- **튜닝 핸들**: `beta` (강체 진동 시 ↑, soft 응답 부족 시 ↓), `alpha_ema` (EMA 시정수)

### 3.3 Anti-windup
- `integrator_frozen` 은 `ApplyDeformationGuard` 가 `remaining ≤ 0` 또는 `remaining < 10% · delta_s_max` 일 때 set
- 해제는 Approaching→Contact 진입 시, Holding 의 slip 보정 시, phase reset 시
- ⚠️ deformation guard 가 영역을 벗어나도 **자동 해제되지 않음** — Holding 에서 한번 frozen 되면 지속될 수 있음 (코드 주석 명시)

---

## 4. 파라미터 레퍼런스

기본값은 [grasp_types.hpp](../include/rtc_controllers/grasp/grasp_types.hpp) 와 [demo_shared.yaml](../../ur5e_bringup/config/controllers/demo_shared.yaml) 의 `force_pi_grasp:` 블록 기준.

### 4.1 PI 게인

| 파라미터 | 기본값 | 단위 | 영향 | 너무 크면 | 너무 작으면 |
|---|---|---|---|---|---|
| `Kp_base` | 0.02 | 1/(N·s) | 비례 응답. `e_f=2N` → `ds≈0.04` | 진동/오버슈트 | 수렴 느림 |
| `Ki_base` | 0.002 | 1/(N·s²) | 정상상태 오차 제거 | 와인드업, 한계사이클 | steady-state offset 잔존 |
| `integral_clamp` | 0.1 | N·s | `∫e` saturation | wind-up | 큰 외란 회복 느림 |

> 설계 노트: `Kp_base · f_target ≈ 0.04 ≈ ds_max = 0.05` 이므로 비례항 단독으로 거의 saturation 영역에 있다. 즉 대부분의 응답은 P 항이 주도하고 I 항은 미세 오차 보정 역할.

### 4.2 Adaptive scheduling

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `alpha_ema` | 0.95 | stiffness EMA 시정수 ≈ `dt/(1−α) = 0.002/0.05 = 40 ms` @ 500 Hz |
| `beta` | 0.3 | gain de-rating 강도 |

튜닝: 강체 진동 시 `beta ↑` (0.5–1.0), soft 물체에서 응답 부족하면 `beta ↓`. `alpha` 는 노이즈 심하면 ↑, 빠른 강성 변화 필요하면 ↓.

### 4.3 Force 임계/목표

| 파라미터 | 기본값 | 단위 | 의미 |
|---|---|---|---|
| `f_contact_threshold` | 0.2 | N | Approaching → contact_detected latch 임계값 |
| `f_target` | 2.0 | N | 목표 grip force |
| `f_ramp_rate` | 1.0 | N/s | f_desired ramp 속도 |
| `f_max_multiplier` | 2.0 | — | Holding 시 최대 허용 force = `f_target · multiplier` |
| `grip_tightening_ratio` | 0.15 | — | 슬립 감지 시 1회 force boost 비율 |
| `df_slip_threshold` | 5.0 | N/s | 슬립 판정 `df/dt` 임계 |

튜닝 가이드:
- `f_contact_threshold`: 센서 noise floor 의 3–5σ 위로 (false latch 방지). 너무 높으면 가벼운 물체 인식 실패.
- `f_target`: 물체 grip-failure 한계의 50% 정도에서 시작.
- `f_ramp_rate`: §4.5 의 settle 검사와 trade-off — 아래 참고.

### 4.4 Rate / saturation

| 파라미터 | 기본값 | 단위 | 의미 |
|---|---|---|---|
| `ds_max` | 0.05 | 1/s | PI 출력 saturation |
| `delta_s_max` | 0.15 | — | 접촉 후 추가 closing 허용 한계 (변형 가드) |

튜닝:
- `ds_max` 너무 작음 → 응답 느림; 너무 큼 → deformation overshoot
- `delta_s_max` 너무 작음 → stiff object 에서 force 형성 전에 한계 도달; 너무 큼 → 손가락이 물체 파괴

### 4.5 FSM 타이밍

| 파라미터 | 기본값 | 단위 | 의미 |
|---|---|---|---|
| `approach_speed` | 0.2 | 1/s | Approaching 의 `ds/dt` (open→close 5 s 소요) |
| `release_speed` | 0.3 | 1/s | Releasing 의 `ds/dt` |
| `contact_settle_time` | 0.1 | s | Contact phase dwell |
| `settle_epsilon` | **0.1** | N | ForceControl 수렴 판정 threshold (`f_target=2.0 N` 기준 5%) |
| `settle_time` | 0.3 | s | ForceControl 수렴 유지 시간 |

> **구조적 관찰**: `f_ramp_rate · settle_time = 1.0 · 0.3 = 0.3 N > settle_epsilon = 0.1 N`. ramp 가 진행되는 동안은 `all_settled` 가 성립하기 어려우므로, 실질적으로 **`f_desired` 가 `f_target` 에 도달한 이후** 에만 `force_settle_timer_` 가 꾸준히 누적된다. `f_ramp_rate` 를 크게 올리면 사실상 settle 검사가 ramp 끝까지 미뤄진다.

### 4.6 필터

| 파라미터 | 기본값 | 단위 | 의미 |
|---|---|---|---|
| `lpf_cutoff_hz` | 25.0 | Hz | Bessel 4차 LPF cutoff |
| `control_rate_hz` | 500.0 | Hz | `BuildGraspController` 에서 런타임에 덮어씀 |

25 Hz cutoff 의 group delay ≈ 20 ms 로 force PI 의 outer-loop bandwidth 대비 충분히 여유 있음.

---

## 5. 권장 튜닝 워크플로

아래 순서로 진행한다. 단계마다 `grasp_state` 토픽 (phase, `f_measured`, `f_desired`, `s`, `K_contact_est`) 을 CSV 로 로깅하여 검증한다.

### Step 1 — 센서 baseline
- 자유공간에서 hand 를 여러 차례 open/close 시키며 `force_magnitude[0..2]` 의 noise σ 측정
- **설정**: `f_contact_threshold ≥ 5σ`. 기본 0.2 N 은 대부분의 경우 안전하나 sensor noise 가 큰 개체에서는 상향 필요.

### Step 2 — Approach 단독 검증
- 테스트 물체 없이 `CommandGrasp()` → `s` 가 `s=1.0` 까지 ramp 되는지 확인 (실패 후 kIdle 로 abort 되어야 함)
- 테스트 물체 있음 → Contact 전이 시점에 thumb/index 의 `f_measured` 가 노이즈 수준을 충분히 넘는지 확인
- 충격이 크면 `approach_speed` ↓ (0.1 1/s 등)

### Step 3 — Open-loop ramp 검증
- 임시로 `Kp_base=0`, `Ki_base=0` 으로 설정 (또는 테스트 YAML 분리)
- Contact → ForceControl 진입 후 `s` 가 변하지 않아야 함 (PI 출력 0)
- `f_measured` 가 LPF 통해 안정적으로 관측되는지 확인
- `f_desired` ramp 동작과 실제 `f_measured` 의 괴리 관찰

### Step 4 — P 이득 튜닝
- `Ki_base = 0` 유지
- `Kp_base` 를 0.005 → 0.04 로 점증하며 step response (f_desired → f_target) 관찰
- 진동 시작 직전 값의 **절반** 을 최종 `Kp_base` 로 채택
- 목표: 오버슈트 ≤ 5%, `0.5 s` 이내 접근

### Step 5 — I 이득 튜닝
- Step 4 에서 결정된 `Kp_base` 유지
- steady-state 잔존 오차를 보고 `Ki_base = Kp_base / τ_i`, `τ_i ≈ 5 · settle_time` 부터 시작
- `integral_clamp ≈ 0.5 · ds_max / Ki_base` 로 설정 (wind-up 방지)

### Step 6 — Adaptive (`beta`) 튜닝
- 강체(금속) + 연체(스펀지) 두 종류 테스트 물체로 교차 검증
- 강체에서 진동 → `beta` ↑ (0.3 → 0.5 → 1.0)
- 연체에서 응답 부족 → `beta` ↓ (0.3 → 0.1)
- `alpha_ema` 는 노이즈 심할 때만 상향

### Step 7 — Deformation guard 점검
- 튜닝 대상 물체의 geometry 에서 `s_at_contact` 부터 `delta_s_max` 만큼 닫혔을 때 물체가 안전한지 물리 검증
- 파손 위험 시 `delta_s_max` ↓, 반대로 stiff object 에서 force 형성 부족 시 ↑

### Step 8 — Anomaly / 장시간 hold 검증
- Holding 중 물체를 살짝 빼내어 slip 유도 → `df_slip_threshold` 가 trigger 되는지, `f_desired` 상승 후 회복되는지 확인
- 30 s 이상 hold 에 외란 인가 → `integrator_frozen` 영구화 여부, `f_desired` 누적 여부 확인 (§6 잠재 이슈)

---

## 6. 알려진 이슈 / 주의사항

### 6.1 Integrator freeze 가 영구화될 수 있음
`ApplyDeformationGuard` 에서 `remaining` 이 한계에 근접하면 `integrator_frozen=true` 로 set 되지만, `remaining` 이 다시 커져도 자동 해제되지 않는다. Holding 단계에서 일시적 deformation 한계 근접 후 외란이 사라져도 I 항은 다시 누적되지 않을 수 있다.
- 증상: Holding 중 steady-state 오차가 줄어들지 않음
- 완화: slip anomaly 가 발생하면 해제되므로, 심각한 문제는 아님. 장기 hold 시나리오에서만 관찰 필요.

### 6.2 Holding 의 grip-tightening 단방향성
Anomaly 가 사라져도 `f_desired` 는 자동으로 감소하지 않는다. `f_max_multiplier` 에 의한 상한만 있음.
- 영향: 반복 slip 시 `f_desired` 가 상한까지 계속 올라가고 유지
- 완화 방향 (향후 개선): anomaly 해제 후 `f_desired` 를 `f_target` 로 decay 하는 로직 추가 고려

### 6.3 `f_ramp_rate` 와 `settle` 검사의 일관성
§4.5 에서 서술한 대로 ramp 도중에는 `all_settled` 가 false 가 되기 쉽다. 실질적으로 ForceControl 의 실제 dwell 시간은 `(f_target / f_ramp_rate) + settle_time` 에 가까움.
- 필요 시 "ramp 완료 후 settle timer 시작" 으로 semantic 변경 고려

### 6.4 `K_contact_est` 초기 transient
`Approaching → Contact` 진입 시 `K_contact_est = 1.0` 으로 reset 된다. 첫 update 의 `Δs = ds_max · dt = 1e-4` 정도로 매우 작아 `K_inst = ΔF/Δs` 가 크게 튈 수 있다. `kDeltaSEpsilon = 1e-6` 가드는 있으나 초기 PI transient 에서 EMA 가 spike 칠 가능성 존재.
- 완화: `contact_settle_time` 중 LPF 가 안정화되는 동안은 문제 없음 (실제로는 f_desired=0 이므로 ds 도 작음)

### 6.5 Thumb/Index 비대칭의 함의
Contact 전이가 thumb+index 2 손가락에 결정되므로:
- **middle spurious latch 방지가 더 중요해짐**: middle 의 false latch 는 이제 grasp 정상 진행을 이상하게 만들 수 있음 (s_at_contact=0 인 상태로 deformation guard 진입). `f_contact_threshold` 를 센서 noise floor 대비 충분한 마진으로 설정 권장.
- **middle 의 `s`**: Approaching 이 thumb+index 접촉 시점에 freeze 한 값 그대로. 기하학적으로 이상한 posture 가 나올 수 있으나 제어 로직 자체는 영향 없음.
- **비대칭 grasp 시나리오 지원**: 2-finger pinch 가 1급 use case 이므로 적극 활용 가능.

---

## 7. 파라미터 변경 절차

1. [demo_shared.yaml](../../ur5e_bringup/config/controllers/demo_shared.yaml) 의 `force_pi_grasp:` 블록을 편집
2. 재빌드 필요 없음 — YAML 은 컨트롤러 init 시 로드
3. 런타임 중 변경은 컨트롤러 재로드 (controller switch) 필요
4. 테스트 실행:
```bash
./build.sh -p rtc_controllers
colcon test --packages-select rtc_controllers --ctest-args -R test_grasp_controller --event-handlers console_direct+
```

---

## 8. 한 줄 요약

Force-PI grasp controller 는 `s ∈ [0,1]` 파라미터의 미분 `ds` 를 outer-loop PI 출력으로 사용하는 position-rate 방식 힘 제어기다. thumb+index 두 손가락만으로 grasp 성공을 판정하는 비대칭 FSM 을 사용하며, online stiffness EMA 로 gain 을 adaptive 하게 조정하고 deformation guard + anomaly-trigger grip tightening 으로 안전성을 확보한다. 핵심 튜닝 축은 (1) `Kp/Ki/β` 의 응답 vs 안정성, (2) `f_ramp_rate` ↔ `settle_*` 의 일관성, (3) `delta_s_max`/`integrator_frozen` 의 안전 한계, (4) `f_contact_threshold` 의 false-latch 마진이다.
