# L8 — Bringup: 컨트롤러 통합, launch·YAML, 시뮬레이션 기반, 로깅·지표, 시스템 검증

- 문서 버전: v0.5 (2026-09-19) — 결정·단계의 SSoT 는 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) (충돌 시 plan 우선)
- 브랜치: 단계별 `type/kebab-slug` (main 기준, 마스터 §4.2)
- 배치 `[확정 D-1]`: 포구 컨트롤러는 `integrated_bringup` 의 `RTControllerInterface` 바인딩 **1개** (`RTC_REGISTER_CONTROLLER`), 수치 코어는 rtc_controllers 의 `catching` 하위 디렉토리. sim 인프라(S3)는 `rtc_mujoco_sim` (robot-agnostic). 오프라인 도구(catchability 지도, NEES·지표 스크립트)의 정확한 위치(`integrated_bringup` scripts 또는 `rtc_mujoco_sim`)는 S3 에서 정한다. **새 패키지를 만들지 않는다**
- 단계: **S3** 시뮬레이션 기반, **S5** 컨트롤러 골격·로그·상태 publisher, **S6** 계획기 스레드, **S8** sim 통합 평가, **S10** 실기 단계 도입
- 선행: 단계 W, L0–L7
- 비고: 컨트롤러 기반 클래스·lifecycle·로깅은 workspace의 기존 구조를 따른다 ([modification-guide.md](../../agent_docs/modification-guide.md) "Adding a New Controller"). 본 layer는 L0–L7을 그 구조에 **끼워 넣는다**.
- 산출물: 포구 컨트롤러 바인딩, 기존 sim launch 확장(인자), 컨트롤러 YAML, 발사 srv (D-14), 공 접촉 truth 출력, catchability 지도 도구 (D-18), vision 요구 사양 (D-15), 지표 스크립트

---

## 1. 범위 / 비범위

범위:
- L0–L7을 하나의 `RTControllerInterface` 컨트롤러로 묶는다 (ros2_control 이 아니다). tick 은 `Compute`.
- 시뮬레이션(`iiwa7_leap`, `ur5e_p1b`)과 실기(`ur5e_p1b`) 실행 구성을 기존 `integrated_bringup` launch 에 얹는다.
- 시스템 수준 검증을 수행한다.
- `demo_controller_gui` 패널과 `plot_rtc_log` CSV 플롯을 단계마다 함께 구현·확인한다 `[확정 D-19]` — 단계별 항목·게이트는 plan §13, 포구 상태는 `rtc_msgs` 새 상태 메시지로 GUI 에 보낸다 `[확정 D-20]` (필드는 S5).

비범위: vision 노드 구현 (형제 저장소 ball_perception, 사용자가 개발). 시뮬레이션에서는 ball_perception 의 `sim_estimator_node` 를 그대로 쓰며 자체 vision 발행기를 만들지 않는다 (§4.3).

## 2. 코드 확인 게이트

단계 W에서 처리했다 (plan §2).

| ID | 확인 항목 | 기록 |
|---|---|---|
| G8-1 | 컨트롤러 기반 클래스·lifecycle 훅, non-RT 스레드 소유 규약 | 닫힘 — `RTControllerInterface` (noexcept lifecycle 훅, `Compute`, `LoadConfig`), 코어+바인딩 2층. 계획기 스레드는 `rtc::PeriodicRtThread` subclass 를 바인딩이 소유 (D-7, plan §6) (W) |
| G8-2 | MuJoCo 연동 방식, `/clock` 발행, 공(freejoint) 상태 접근 방법 | 닫힘 — `rtc_mujoco_sim` 은 ros2_control·`/clock` 없음, lock-step (명령 대기), stamp = wall. 공 상태는 `/sim/ball/ground_truth` (Odometry, world) 와 `/sim/ball/camera_position` (noise). 시간축은 D-3 (§4.5) (W) |
| G8-3 | 두 MJCF의 지문 센서 정의와 발행 경로 | 부분 닫힘 — sim·실기 모두 finger-on-object 부호 (커밋 0fcc1d23 이후, L7 §4.4). 센서 정의·잡음은 TBD-HAND-03 |
| G8-4 | 기존 로깅 도구 | 닫힘 — 기존 CSV·timing 인프라 (`rtc::ThreadCsvProducer`/`rtc::ThreadCsvLogger`, `rtc::ThreadTimingCsvLogger`) 를 쓴다. 새 기록 스레드를 만들지 않는다 (W) |
| G8-5 | 컨트롤러가 backend 명령 자리를 어떻게 잡는지, 실기/시뮬레이션 전환 | 닫힘 — `ControllerOutput.devices[]` (팔 device 0, 손 device 1), CM 이 `ValidateControllerOutput` → `DeviceBackend::WriteCommand`. 전환은 launch 가 고르는 backend (`mujoco_native` / `ur_driver_native` + `udp_hand_native`) (W) |

## 3. 참고자료

[R8] NEES/NIS, [R13] Wilson 구간, [R14] MuJoCo, [R11] UR 드라이버.

## 4. 수학적 이론

### 4.1 RT 틱 내 실행 순서

`Compute(const ControllerState&) noexcept` 한 번 안의 순서를 고정한다. 순서가 바뀌면 1틱 지연이 생긴다.

1. 시각: $now$ = steady 실측, $now_{lead}=now+T_{arm}$ (plan §3). tick 수 × `ControllerState::dt` 로 계산하지 않는다
2. 상태 읽기: `ControllerState` 의 측정값, 궤적 스냅샷·`PlanSnapshot` SeqLock 읽기 (L1)
3. 공 상태: 궤적을 $now_{lead}$ 에서 샘플링 (L2)
4. 감독: FSM 1 tick → 모드, 대상, intercept 설정 (L7)
5. 기준 생성: soft-catch 기준, 축 정렬 (L4)
6. 관절 명령: 확장 CLIK (`rtc::tsid::ClikReferenceGenerator`, D-5·D-6) → $q_c$ (L5). QP 실패 시 QP 비의존 관절공간 경로 (L7 §4.1)
7. 손: 손 시퀀서 → 손 device slot (L6, D-11)
8. 명령 쓰기: `ControllerOutput.devices[0]` (팔, `kPosition`), `devices[1]` (손). 이후 CM 이 검증·E-STOP 대체(`BuildHoldOutput`)·`WriteCommand` 를 한다
9. RT 상태 게시: 계획기용 `rtc::SeqLock<RtStatePod>` write, 상태 publisher 용 SeqLock write, 기록 레코드 SPSC push

### 4.2 투척 생성 (시뮬레이션)

**발사 API `[확정 D-14]`.** 발사는 (p0, v0, ω) 를 명시하는 srv 를 `rtc_msgs` 에 추가해 부른다 (Adding a New Message, PROC-3, S3.2). 기존 `/sim/launch_ball` (Trigger, YAML 분포 샘플)은 파라미터 설정 + Trigger 조합이 경합·재현성에 약해 평가 경로에 쓰지 않는다.

**발사 조건 `[확정 D-18]`.** 목표 포구점을 표본 추출해 역산하던 v0.4 방식(무항력 초기값 + 상태전이행렬 슈팅)은 v0.5 에서 대체됐다. 발사 조건은 plan §11 의 발사 영역에서 직접 표본 추출한다: arm base frame (CLIK `base_frame`) 기준 수평 거리 4 m 원호 위의 방위 φ, world z 1.5–2.0 m, 속도 크기·앙각, 수평 방향 = (발사점 → 겨냥점) + 편차. 표본 범위는 catchability 지도(§4.6)가 정한 `sim.throw_region` 이다.

- 항력·Magnus 는 `rtc_mujoco_sim` 이 자체 구현한다 (tennis preset). 계획기의 공 모델과 다르므로 도달점 차이는 의도된 모델 불일치로 기록한다
- iiwa7_leap 에는 projectile 설정이 없다 → S3.2 에서 추가. ur5e_p1b 의 현 스폰 위치는 손바닥 위(낙하 테스트용)라 투척용 스폰을 S3.2 에서 따로 둔다
- 재현성: srv 인자 자체를 시행 기록에 남기고, 난수는 평가 스크립트 쪽 seed 로 뽑는다

### 4.3 시뮬레이션 vision — ball_perception `sim_estimator_node`

v0.4 의 자체 vision 발행기(측정 모사 + 참조 EKF + `PointCloud2`)는 v0.5 에서 삭제했다. ball_perception 의 `sim_estimator_node` 가 `/sim/ball/camera_position` 을 구독해 `/ball_perception/debug/prediction/trajectory` (`PointCloud2`) 를 발행하므로, L1 파서는 sim 과 실기에서 **실제 발행기와 같은 레이아웃**을 탄다 (D-4).

- 연결 (S3.4, **완료 2026-09-20** — 결과는 plan §4.4 S3.4): clock domain `ros_system_time` + `use_sim_time=false` (rtc 에 `/clock` 없음, stamp = wall epoch — 2026-09-22 부터 공 lane 은 발사 기준 sim 시간축을 wall 에 얹은 값을 찍는다, `rtc_mujoco_sim` README §Projectile Ball stamp — 설정으로 닫힘), `frame_id` = `world`, 당시 profile 로 30 Hz · N 16 · 지평 0.8 s (TBD-VIS-04/06 닫힘; 현 설정은 아래 vision 요구 사양의 1.0 s / 20 점), 지연·드롭 주입은 `rtc_tools camera_relay` 로 입력 토픽 앞에서 한다. sim 재시작 시 `clock_reset` 은 wall stamp 에서는 **발동하지 않는다** — `/clock` 전환(S5/S6) 뒤에 다시 본다
- 측정 잡음은 `rtc_mujoco_sim` 의 `publish.position_noise_stddev_m` 이 준다
- 제어 경로에서 truth 토픽을 쓰지 않는다. truth 는 지표·NEES 전용
- 자체 fixture EKF 는 만들지 않는다. `ball_dynamics` 는 test fixture 전용 위치로 옮긴다 (S1.6)

**vision 요구 사양 `[확정 D-15]` — S3.6 완료 (2026-09-22).** 목표 투척 분포에서 "검출 이후 포구 창 종료까지 최대 비행 시간" → 필요 지평, L2 보간 게이트를 만족하는 간격 → 점 수 → 런타임 상한 `n_max` 를 산출했다: $H_{req}$ **0.99 s** · 간격 **0.05 s** · **n 20** → **sim profile 1.0 s / 0.05 s / 20 점 / ≤ 30 Hz — 설정됨** (`integrated_bringup/config/ur5e_p1b/ball_perception_sim_profile.json`, `sim_estimator.launch.py profile_path:=` 에 준다; 2026-09-22 사용자 결정, 공 lane stamp 수정 + T_det 재실측 후). 값·유도·한계는 plan §4.4 "S3.6 결과"·"T_det 실측" 이 SSoT 다. 수신 궤적의 지평이 요구 (`io.horizon_min` 0.51 s, L1 §6) 보다 짧으면 제어기는 계획 후보에서 제외하고 진단한다.

### 4.4 지표

**예측 일관성 (시뮬레이션, 오프라인).** ball_perception 이 발행한 **예측** 점 $k$ (예측 원점 `header.stamp` + `horizon_ns`) 와 같은 시각의 truth 로 NEES 를 계산한다.

$$\epsilon_k=(x_{true}(t_k)-\hat x_k)^\top P_k^{-1}(x_{true}(t_k)-\hat x_k),\qquad x=(p,v)\in\mathbb R^6$$

평균이 상태 차원 6 에 가까운지 지평별로 본다([R8]). `covariance` 가 NaN(모름)인 점은 제외하고 그 비율을 따로 기록한다.

- truth `publish.sample_rate_hz` (현 100 Hz) 로는 $t_k$ 보간 오차가 커질 수 있다 → S3.3 에서 상향
- truth 는 공이 활성(발사 후, park 전)인 동안만 의미가 있다 — 그 구간만 쓴다
- stamp 는 둘 다 wall 이다 (D-3). δ 가 큰 구간에서만 오차가 커지는지는 §4.5 게이트로 가른다

**포구 지표 (시행별).**
- $t_c$에서 간극: $\Vert p_{true}(t_c)-p_C(t_c)\Vert$ (catch frame 실제 위치, D-10)
- 상대속도: $\Vert v_{true}(t_c)-\dot p_C(t_c)\Vert$
- **충격량** $\Delta p=m_{ball}\Vert v_{true}(t_c)-\dot p_C(t_c)\Vert$, 접촉 지속 $\Delta t_{imp}$, 평균·최대 접촉력, 손가락 관절 최대 토크 (시뮬레이션 접촉 truth, S3.3) — L7 §4.7
- 계획 $\gamma_f$, `REF_SATURATED` 발생 (D-8 재검토 입력), 전이 사유, 결과, catchability w₅ (D-18) 와 CLIK `Manipulability()`

**성공률 신뢰구간 (Wilson, [R13]).** $n$회 중 $s$회 성공, $\hat p=s/n$, $z$는 정규 분위수:

$$\frac{\hat p+\frac{z^2}{2n}\pm z\sqrt{\frac{\hat p(1-\hat p)}{n}+\frac{z^2}{4n^2}}}{1+\frac{z^2}{n}}$$

합격 하한(floor)과 시행 수는 D-12 (사용자 제공 값).

**오차 예산 검증.** L3 §4.6 예측 간극 분포와 실제 간극 분포를 비교한다(모델 검증).

### 4.5 sim 시간축과 clock 위상 게이트 `[D-3 채택 — 검증 후 추가 검토 필수]`

sim 은 wall clock 을 유지한다 (D-3). D-2 변환이 실기와 같은 경로로 동작하게 하려는 선택이다. 기존 RTF 신호(200 step 구간 평균)는 0.5 s 비행 동안 표본이 1~2 개뿐이라 짧은 스톨을 평균이 지우므로, 판정은 **비율(RTF)이 아니라 시행별 clock 위상 오차**로 한다 (plan §5).

**측정.** S3.3 이 추가하는 per-step `(sim_time, steady_now)` 진단 lane 이 데이터 원천이다. 발사 시각을 원점으로 비행 구간 매 step 에서 δ(t) = (steady(t) − steady₀) − (sim(t) − sim₀) 를 구하고, 시행별 δ_max = max|δ| 와 max pause(한 step 의 Δwall − Δsim 최댓값)를 기록한다 (양방향).

**시행 유효 조건.** v_max·δ_max + ½·a_bound·δ_max² ≤ ε_clk,alloc **이고** max pause ≤ ε_clk,alloc / v_max. v_max 는 목표 투척 분포의 최대 공 속력(S3.5b 전에는 S0.7 가정값), a_bound 는 g + 항력 가속 상한(항력 k — **S3.8 이 2026-09-20 결정으로 빠졌으므로** L0 §4.1 의 문서 대표값 0.0229 1/m 를 쓴다). **ε_clk,alloc 은 L3 §4.6 오차 예산 중 시계 항에 할당한 몫이고, 예산 우변은 `r_cap/n_σ` 다.** r_cap 은 S4.5 가 닫았고 (LEAP 31.0 mm, P1b 24 mm — L6 §4.5) 재판정 결과는 plan §5.1 에 있다: **LEAP PASS, P1b PASS(provisional)** (각 손의 $\Vert v\Vert_{\max}$ 기준). ε_clk 는 ‖v‖δ 라 **목표 속력에 비례**하므로 판정은 S4.4 의 목표 속력과 함께 읽는다.

**시행 수·무효율.** 구성별(로봇 2종) 발사 ≥ 200 회, 무효율 ≤ 5 %. 이 두 값은 **제안값**이다(사용자 확인, plan §7.3). 어느 구성이든 무효율이 상한을 넘으면 `/clock` 방식을 포함해 D-3 을 다시 결정한다.

**이 결정은 S3.1a(무부하) · S3.1b(부하: 포구 컨트롤러 + 계획기 + `sim_estimator_node`) 검증(plan §5) 결과로 다시 검토한다** — 예측 오차가 δ 가 큰 구간에서만 커지는지, `sim_estimator_node` stamp 가 `use_sim_time=false` 에서 wall 인지도 함께 본다. 성공률 평가(S8)는 유효 시행으로 계산하되 전체 발사 수와 무효 사유를 함께 보고한다.

처리량: lock-step 에서 `max_rtf` 1.0 은 상한이므로 RTF ≤ 1 이다. 시행당 발사·비행·감속·복귀·재무장을 합쳐 수 초가 걸려 **200 시행에 약 12 분** 이상이 든다 — 평가 일정은 이를 전제로 짠다.

### 4.6 catchability 지도 도구 `[확정 D-18]` (S3.5a kinematic 지도 → S3.5b gate-catchable 지도)

plan §11 이 정의·YAML 의 SSoT 다. 요점:

- 발사 영역: arm base frame 수평 거리 √(x²+y²) = 4 m 원호 (방위 φ, 초기 탐색 ±90°), world z 1.5–2.0 m, 비행시간 T_f ≥ 1.0 s (상한은 지도 결과), 수평 방향 = (발사점 → 겨냥점) + 편차 (초기 탐색 ±10°), 속도·앙각 격자
- 판정: 궤적 위 포구 후보마다 catch frame +z = −v̂ 자세의 IK (대기 자세에서 시작) → 게이트 정의(기본 `arm_5row`)의 manipulability ≥ 정의별 threshold (w₅ 0.1, provisional). w₅·w₆ 를 모두 기록해 지도에서 두 분포를 비교한다 (C-3). 런타임 계획기(S6.2)와 **같은 함수·같은 YAML 키** (S1.9)
- **frame 함정:** ur5e_p1b 의 arm base frame 은 URDF `base` 이고 `base_link` 와 z 둘레 180° 다르다 — `base_link` 로 두면 공이 등 뒤에서 날아오는데 결과가 그럴듯해 조용히 틀린다. world ↔ base 변환은 같은 q 에서 MuJoCo FK 와 Pinocchio FK 대조로 **S3.2** 에서 확정한다 (S3.5a 의 선행, plan §4.2 DAG)
- S3.5a 는 kinematic catchability 지도(IK + w₅/w₆), S3.5b 는 S4.4 go/no-go 값(T_close, d_eff, 가속 box, η_v)으로 전체 게이트 체인을 다시 돌린 gate-catchable 지도다 — 이 지도가 목표 투척 분포가 되고, S3.6 vision 요구 사양 산출로 이어진다
- 출력: 격자별 포구 가능 여부, 최대 w 와 그 후보의 $t_c$·$p_c$·$q^*$, 탈락 사유 → `sim.throw_region` 제안 (발사 srv 설정으로 사용)
- **도구 (S3.5a 완료 2026-09-21)**: `ros2 run rtc_tools catchability_map` (python — 격자·항력 비행·집계·그림) 이 `ros2 run rtc_controllers catch_pose_ik_batch` (C++ — 판정) 을 샤딩해 부른다. 판정을 python 에서 다시 구현하지 않는 이유가 §4.6 첫 줄의 "같은 함수" 다. 두 함정: 판정기는 `p_c`·`v` 를 **모델 world** (URDF 모델 root) 로 받고 이것은 arm base frame 이 **아니다** (`ur5e_p1b` 에서 Rz(180°) 차이 — plan §11); 출하 `sub_models.<arm>` 은 flange 에서 끝나 catch frame 을 담지 않으므로 지도는 arm root → catch frame 부모까지의 sub-model 을 따로 선언한다. 결과·제안값은 plan §11

## 5. C++ 구현

### 5.1 컨트롤러 구조

```cpp
// integrated_bringup 바인딩 (D-1). 수치 코어는 rtc::catching.
class CatchingController final : public rtc::RTControllerInterface {
 public:
  // non-RT (noexcept lifecycle 훅)
  //  on_configure : LoadConfig(YAML) + ParseXxxParams + L0 검증, PinocchioCache/RtModelHandle 준비,
  //                 PointCloud2 필드 맵(L1), 전 버퍼 할당, 구독·Setup*Publisher 생성
  //  on_activate  : q_c·CLIK 앵커 = q_meas, 계획기 스레드 layout 게이트 → spawn → Resume (D-7), Mode::kIdle
  //  on_deactivate: 계획기 Pause (join 은 소멸자에서만)
  // RT
  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;  // §4.1
  // E-STOP·fault: TriggerEstop/ClearEstop/SetHandEstop, ResetFault/HasLatchedFault — P-1 임시 기준(S5.1 최소 계약, L7 §4.1), 정책은 S9.
  //   S5.1 구현: 네 훅은 atomic 요청·epoch 만 갱신하고, 되돌리는 동작의 유일 writer 는 Compute() 다.
  //   운용자 무장 채널은 파라미터 `catching.enable` (A-S5-3) — 콜백은 atomic 만 쓰고 tick 이 소비하며,
  //   tick 이 E-STOP·fault 에서 그 latch 를 내린다 (P-1 (c) 를 메커니즘으로 만든다).
};
```

- 궤적 구독은 controller 소유 구독이라 `nrt_callback_executor` (단일 스레드)에서 돈다 — 파싱·D-2 변환·SeqLock write·계획기 eventfd 신호만 하고 계산은 계획기 스레드로 넘긴다 (D-7)
- 계획기 스레드: `rtc::PeriodicRtThread` 의 형제 subclass, event 구동 (D-7c), slot·스케줄러는 D-7b·D-7a (plan §6·§7). 배치 변경은 E-7 이며 Adding a New Thread 절차를 따른다
- SeqLock payload (`RtStatePod`, `PlanSnapshotPod`, 궤적 스냅샷, 상태 스냅샷)는 trivially copyable POD (`std::array` 기반, Eigen 멤버 금지)
- 상태 publisher (L8.3, D-20): `rtc_msgs` 새 포구 상태 메시지, controller 소유 `rtc::SeqLock<T>` + `Setup*Publisher` 패턴 (`WbcState`·`GraspState` 선례). `PublishRole` 에 추가하지 않는다 (E-11). **필드는 S5 에서 S5~S9 superset 을 한 번에 동결**하고 이후 단계는 값만 채운다. `Compute()` 의 **모든 tick 에서 Store** 한다 — E-STOP·stale·generation 불일치·지평 부족·plan 없음·abort 를 포함한 **모든 early-return 분기**에서도 Store 하고, 그 tick 에 계산하지 않은 필드는 무효화한다 (PROC-7, agent_docs/invariants.md). DemoWbc 의 `!target_initialized_` early-return 이 `wbc_state_lock_.Store` 를 빠뜨리는 것은 **알려진 gap 이지 선례가 아니다** — 새 컨트롤러가 그대로 이식하지 않는다
- v0.4 의 손 명령 포트 추상화(in_loop/async)는 D-11 로 삭제 — 손은 device slot 에 직접 쓴다

### 5.2 기록 레코드

```cpp
struct TickRecord {                   // 고정 크기, POD
  std::int64_t t_steady_ns;
  std::uint8_t mode, reason;
  std::uint64_t generation;           // vision 트랙 epoch (L1 §4.4, D-4)
  std::uint32_t plan_id;
  std::uint64_t activation_generation; // provenance token (D-22) — ActivationGeneration() at consume time
  std::uint64_t snapshot_sequence;     // provenance token (D-22) — 소비한 스냅샷 sequence
  float gamma, gamma_d, gamma_dd;
  std::array<float, 3> x_ref, xd_ref, xdd_ref, u_des, e, ed, ball_p, ball_v;
  // xdd_ref = 실현 가속도, u_des = 포화 전 DS 요구값 (L4). 둘을 함께 기록해야
  // 포화 구간 해석이 된다. e, ed 는 한 틱 이전 기준이다.
  std::array<float, 7> q_cmd, q_meas;
  float hand_rho;
  std::array<float, 4> tip_force;     // 센서 수 상한 (G8-3), finger-on-object 부호 (L7 §4.4)
  std::uint8_t flags;                 // sat, bound_active, bound_conflict, stale, ...
};
```

- 기록 경로: RT 는 SPSC 에 push 만 하고, drain 과 CSV 쓰기는 기존 CSV 인프라(`rtc::ThreadCsvProducer`/`rtc::ThreadCsvLogger`)가 맡는다. **새 기록 스레드를 만들지 않는다.** 계획기 timing 은 DemoWbc 관용구대로 aux 타이머가 drain 한다 (plan §6)
- 레코드 필드 확정은 S5.4 (L4·L5 타입 확정 후)
- 토픽 기록: rosbag2로 vision `PointCloud2`, truth(시뮬레이션), 상태 토픽을 기록한다

### 5.3 launch 구성

새 launch 파일을 만들지 않고 **기존 `integrated_bringup` launch 에 인자를 더한다** (중복 launch 금지).

| launch | 로봇 | backend | 공 입력 |
|---|---|---|---|
| `sim_iiwa7_leap.launch.py` (+ 포구 인자) | `iiwa7_leap` | `mujoco_native` | `sim_estimator_node` (§4.3) |
| `sim_ur5e_p1b.launch.py` (+ 포구 인자) | `ur5e_p1b` | `mujoco_native` | `sim_estimator_node` (~~선택: position 지연 에뮬레이션~~ — 2026-09-20 결정으로 없다, L5 §4.6) |
| `robot_ur5e_p1b.launch.py` (+ 포구 인자, S10) | `ur5e_p1b` | `ur_driver_native` + `udp_hand_native` | vision PC |

컨트롤러 선택은 기존 `initial_controller` 인자를 쓴다. 추가 인자(`sim_estimator_node` 동시 기동 등)의 이름은 S5 에서 정한다.

### 5.4 YAML 파일

- 포구 컨트롤러 설정: 로봇별 `integrated_bringup/config/<robot>/controllers/` 아래 (기존 `demo_*_controller.yaml` 과 같은 자리). 파일명은 `demo_catching_controller.yaml` (config_key `demo_catching_controller`, S4.0 에서 확정 — plan §4.4 S4a)
- catch frame (`extra_frames`, D-10·D-17) 은 로봇 config 의 모델 절 (plan §10)
- 투척·catchability (`sim.throw_region`, `planner.catchability.*`) 는 plan §11 스키마
- sim 공 설정 (projectile, truth 주기)은 로봇별 `mujoco_simulator.yaml`
- 로드 후 L0 검증기를 실행한다. provisional 값(D-12·D-17)은 실기 arm 을 막는다

## 6. YAML 파라미터

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `logging.decimation` | int | – | 1 | 1–50 | 기록 부하 |
| `logging.ring_capacity` | int | – | 4096 | 256–65536 | SPSC 용량 (drain 지연 흡수) |
| `logging.dir` | — | – | – | – | v0.5 에서 삭제 — 기존 CSV 인프라의 세션 디렉토리를 쓴다 |
| `sim.throw.*` | — | – | – | – | v0.5 에서 삭제 — `sim.throw_region` (plan §11, D-18) 로 대체 |
| `sim.meas.rate` | — | – | – | – | v0.5 에서 삭제 — 측정은 `rtc_mujoco_sim` `publish.sample_rate_hz`, 예측 발행률은 ball_perception profile (D-15) |
| `sim.meas.sigma` | — | – | – | – | v0.5 에서 삭제 — `rtc_mujoco_sim` `publish.position_noise_stddev_m` |
| `sim.meas.delay`·`dropout`·`outlier_rate` | — | – | – | – | 지연·드롭 주입 위치와 키는 S3.4 에서 정한다 |
| `sim.ekf.q_acc` | — | – | – | – | v0.5 에서 삭제 — 자체 fixture EKF 없음 (§4.3) |
| `sim.vision.layout` | — | – | – | – | v0.5 에서 삭제 — 실제 발행기 레이아웃 사용 (D-4) |
| `sim.rtf_min` | — | – | – | – | v0.5 에서 삭제 — RTF 비율 판정은 §4.5 clock 위상 오차 판정으로 대체 (D-3, plan §5) |
| `sim.clock.eps_alloc_m` | double | m | `TBD` | >0 | §4.5 ε_clk,alloc — r_cap 확정 후 재판정 완료 (plan §5.1). 값은 **목표 속력에 비례**하므로 S4.4 가 속력을 확정한 뒤에 박는다 |
| `sim.clock.min_launches` | int | – | 200 | ≥1 | §4.5 구성별 최소 발사 수 (제안값, plan §7.3) |
| `sim.clock.max_invalid_rate` | double | – | 0.05 | (0, 1] | §4.5 무효율 상한 (제안값, plan §7.3) |
| `sim.trials` | int | – | D-12 | ≥30 | 신뢰구간 폭과 연동, 성공률 하한 D-12 |

**실기에서 검증 불가능한 것 `[권장]`.** 실기에는 $p_{true}(t_c)$ 가 없으므로 "예측 간극 분포"를 직접 잴 수 없다. 관측 가능한 것은 포획/실패 이진 결과와 지문 접촉 시각뿐이다. 따라서 실기 게이트(G8-G)는 다음 중 하나로 대체한다 — **선택은 S10** (plan §7.3).

1. 포획률 대 $\sigma_c$ 의 로지스틱 회귀로 유효 $r_{cap}/\kappa_\sigma$ 를 역추정.
2. 접촉 센서 조합·접촉 시각 편차를 간극의 대용 지표로 사용.
3. 저속 구간 1회 한정 외부 계측(마커 또는 고속 카메라) 캠페인.

셋 다 하지 않으면 **"실기 vision 공분산은 미검증 가정"** 임을 plan §12 위험 목록에 올린 채로 진행한다.

## 7. 단위 기술 구현 순서

- **L8.1** G8 게이트 — 닫힘 (W, §2).
- **L8.2** (S4.0 → S5.1) 컨트롤러 골격 + §4.1 순서 + lifecycle + 기존 launch 인자. S4.0 은 손 계단 진단 모드·팔 hold 뿐인 최소 골격이고 (sim 전용을 `backend.type` 가드로 강제 — S5.1 에서 E-8 승인과 함께 제거), 팔 명령 경로·CLIK 앵커·E-8 최소 계약을 포함한 완성은 S5.1.
- **L8.3** (S5.4) 기록 레코드 → 기존 CSV 인프라, 상태 publisher, 플롯 스크립트.
- **L8.4** (S3.1a·S3.2·S3.3, 부하 재검증은 S3.1b — S6 에서 계획기·`sim_estimator_node` 동시 구동으로) D-3 검증, 발사 srv, iiwa7_leap projectile, 접촉 truth·truth 주기·sim time 진단.
- **L8.5** (S3.4·S3.5a/b·S3.6) `sim_estimator_node` 연결, catchability 지도(kinematic → gate-catchable), vision 요구 사양, 오프라인 NEES.
- **L8.6** (S8) `sim_iiwa7_leap` 폐루프 → 시스템 게이트 G8-A~C.
- **L8.7** (S8) `sim_ur5e_p1b` 폐루프 → G8-D.
- **L8.8** (S10) 실기 단계적 도입 (§9.2).

## 8. 디버깅 방법

- 틱 실행시간 분해: §4.1 단계별 시간 히스토그램.
- 결과별 타임라인 자동 생성: 실패·abort 시행만 모아 L7 §8 그래프를 만든다.
- 재현성: 발사 srv 인자·난수 seed·YAML 해시를 시행 기록에 넣는다.
- 틱 예산 분해: 스냅샷 복사 바이트, SeqLock 재시도 횟수, QP 반복 수·solve time을 따로 기록한다. 예산은 `dt` (= 1/`control_rate`, 100–5000 Hz) 이고 총량이 아니라 **분산의 꼬리**가 위험하다.
- 오차 원인 분해: 간극을 L3 §4.6의 항별($A=p_{true}-\hat p_{live}$, $B=\hat p_{live}-p_c$, 추종, 시계)로 분해해 표로 만든다. 시뮬레이션은 truth 가 있으므로 각 항을 직접 계산할 수 있고, $A\perp B$ 가정도 검증할 수 있다(G8-C2).
- 충격 구간 분해: $t_c$ 전후 100 ms의 접촉력·관절 토크·$q-q_c$ 괴리를 겹쳐 그린다(L7 §4.7).
- 무효 시행 급증: §4.5 clock 위상 오차(δ_max·pause) 로그와 부하 구성을 확인한다.

## 9. 검증 방법과 합격 게이트

### 9.1 시뮬레이션

| 게이트 | 기준 | 태그 |
|---|---|---|
| G8-A | 전체 시행에서 RT 위반 0 (할당, 틱 초과 기준은 RTC 규약). 계획기 스레드가 RT 와 다른 코어에 있는지 확인. **개발 PC (PREEMPT_RT 아님) 의 RT·CPU 격리 결과는 smoke 로만 보고, 판정은 제어 PC 에서** 한다 | `[SIM-ANY]` |
| G8-A2 | **연속 2회 투척**과 **abort 직후 재투척** 시나리오에서 L7 §4.8 재무장 리셋 목록이 전부 동작 (옛 plan 재사용 0, 복귀 위치가 `wait_pose`, 첫 틱 `bound_conflict` 0) | `[SIM-ANY]` |
| G8-H | early-return 분기(E-STOP·stale·generation 불일치·지평 부족·plan 없음·abort)마다 그 tick 의 body 가 실렸는지 보는 실패 경로 테스트 (PROC-7, `EstopTickPublishesThisTicksBody*` 선례). S7 에서 늘어난 분기까지 확장 | `[SIM-ANY]` |
| G8-B | ball_perception 예측의 NEES 평균이 [R8] 구간 안 (지평별, NaN 공분산 제외, §4.4) | `[SIM-ANY]` |
| G8-B2 | L1 파서가 ball_perception 실제 레이아웃(필드 이름·datatype)을 검사하고 레이아웃 해시 진단이 변경을 감지 — sim·실기가 같은 발행기 계열이라 복제 레이아웃 대조는 불필요 (D-4, P-3) | `[SIM-ANY]` |
| G8-B3 | 실기 vision 공분산의 일관성: §6 세 수단 중 S10 에서 고른 방법 (G8-C2와 짝) | `[HW-P1B]` |
| G8-C | `iiwa7_leap`에서 $\gamma=0$ 고정 대 계획 γ ablation: 간극·상대속도·**충격량**·접촉력 분포 비교 | `[SIM-ANY]` |
| G8-C2 | 오차 예산 모델 검증: L3 §4.6 직교 분해식의 예측 간극 분포와 실제 분포 비교. truth 가 있으므로 $A=p_{true}-\hat p_{live}$, $B=\hat p_{live}-p_c$ 의 직교성도 직접 확인한다 | `[SIM-ANY]` |
| G8-C3 | v0.5 에서 삭제 — γ 하향 v1 범위 밖 (D-8). 대신 `REF_SATURATED` 빈도를 기록해 D-8 재검토 입력으로 쓴다 | `[SIM-ANY]` |
| G8-D | `ur5e_p1b`에서 성공률 Wilson 95% 하한 ≥ floor (D-12), 결과별 사유 분포. 전체 발사 수·무효 시행 수·무효 사유를 함께 보고하고, 무효율이 plan §5 상한을 넘으면 그 run 은 `NOT_EVALUATED` | `[SIM-P1B]` |
| G8-D2 | `iiwa7_leap`에서 성공률 Wilson 95% 하한 ≥ floor (D-12), 결과별 사유 분포. 전체 발사 수·무효 시행 수·무효 사유를 함께 보고하고, 무효율이 plan §5 상한을 넘으면 그 run 은 `NOT_EVALUATED`. **평가 대상 여부는 plan §1a 의 조건부 규칙이 정하고, T_det 실측 (2026-09-22) 으로 조건은 충족됐다** — 그 lead 에서 gate 지도가 11 / 2835 로 비어 있지 않다 | `[SIM-ANY]` |
| G8-E | `ur5e_p1b` + position 지연 에뮬레이션에서 선행 보상 유무 비교. ⚠️ **substrate 없음** — S3.7 이 2026-09-20 결정으로 빠져 sim 에 에뮬레이션 지연이 없다 (L5 §4.6). 지연 0 에서는 유무 비교가 공허하므로 **fixture 전용 지연 주입** 위에서 판정한다 (2026-09-22 사용자 확정 — L5 §9 G5-E, plan §7.3) | `[SIM-P1B]` |

모든 sim 게이트는 §4.5 clock 위상 게이트를 통과한 시행만 집계하고 무효 비율을 함께 보고한다. 처리량은 §4.5 (200 시행 ≈ 12 분 이상).

### 9.2 실기 단계적 도입 `[권장]` (S10)

1. **공 없이 재생.** 기록된 vision `PointCloud2` bag을 재생한다. D-2 변환은 수신 wall 시각과 stamp 차를 쓰므로 **재스탬프 도구**가 필요하다 (S10). 저속 설정(투척 속도 축소 bag)으로 로봇·손 동작과 타이밍을 확인한다.
2. **가상 공.** 제어 PC 내부 가상 궤적으로 전 구간(감속·복귀 포함)을 확인한다.
3. **실제 공, 저속 토스.** 부드러운 공, 짧은 거리, 낮은 속도로 시작한다.
4. **속도 단계 상향.** 각 단계에서 G8-D 지표를 기록하고 다음 단계로 간다.

S10 착수 전 S9 (E-STOP·fault 정책, D-13) 완료가 필수다. 각 단계 진입 전 `T_arm`, `T_close_tot`, 시계 offset, 그리고 **예측 일관성 지표 $\bar\nu$**(L1 §4.5)를 확인한다. speed scaling·PTP 감시는 신호 출처를 확보한 뒤 연결한다.

| 게이트 | 기준 | 태그 |
|---|---|---|
| G8-F | 단계 1–2 완료, abort 경로 전부 실기 동작 확인 | `[HW-P1B]` |
| G8-G | 단계 3 이후 성공률·간극 분포 기록, 시뮬레이션 대비 차이 원인 분해 — "원인 분해" 는 항목별 수치 기록(§8 오차 원인 분해 표의 항목별 값)으로 판정한다 (plan §4.4 S10) | `[HW-P1B]` |

## 10. 미확정 항목

TBD-HAND-03, ~~TBD-VIS-04/06~~ (S3.4 닫힘), D-3 재검토 (S3.1a·S3.1b), `sim.clock.eps_alloc_m`·시행 수·무효율 제안값 (S3.1a), D-12 값 (투척 속도·성공률 하한·시행 수), `sim.throw_region` 값 (S3.5a/b), 지연·드롭 주입 위치 (S3.4), 오프라인 도구 위치 (S3), 포구 launch 인자·컨트롤러 YAML 파일명 (S5), 실기 공분산 검증 수단 (S10), 재스탬프 도구 (S10).
