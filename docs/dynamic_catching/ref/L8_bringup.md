# L8 — Bringup: 컨트롤러 통합, launch·YAML, 시뮬레이션 기반, 로깅·지표, 시스템 검증

이 문서는 현재 구현의 **통합 계층** — L0–L7 을 하나의 컨트롤러로 묶는 tick 순서, 시뮬레이션 기반, 지표, 시스템 게이트, 실기 도입 절차 — 을 표현한다.

- 배치: 포구 컨트롤러는 `integrated_bringup` 의 `RTControllerInterface` 바인딩 **1개** (`DemoCatchingController`, `RTC_REGISTER_CONTROLLER` — `controller_registration.cpp`), 수치 코어는 `rtc_controllers` 의 `catching` 하위 디렉토리. sim 인프라는 `rtc_mujoco_sim` (robot-agnostic). **새 패키지를 만들지 않는다**
- 컨트롤러 기반 클래스 · lifecycle · 로깅은 workspace 의 기존 구조를 따른다 ([modification-guide.md](../../../agent_docs/modification-guide.md) "Adding a New Controller"). 본 layer 는 L0–L7 을 그 구조에 **끼워 넣는다**
- 로봇은 셋이다: `ur5e_p1b` (UR5e + proto_1b, 폐쇄 체인 손, sim + 실기), `iiwa7_leap` (iiwa7 + LEAP, sim), `g1_p1b` (Unitree G1 상체 + proto_1b 오른손, sim). **`g1_p1b` 는 포구 컨트롤러 config 가 없다** — `config/g1_p1b/controllers/` 에는 `demo_joint_controller.yaml`, `demo_dualarm_controller.yaml` (포구 컨트롤러가 아니다 — 두 손 frame 을 목표로 받는 다중 frame CLIK), `demo_shared.yaml` 이 있다

---

## 1. 범위 / 비범위

범위:
- L0–L7 을 하나의 `RTControllerInterface` 컨트롤러로 묶는다 (ros2_control 이 아니다). tick 은 `Compute`.
- 시뮬레이션 (`iiwa7_leap`, `ur5e_p1b`) 과 실기 (`ur5e_p1b`) 실행 구성을 기존 `integrated_bringup` launch 에 얹는다.
- 시스템 수준 검증을 수행한다.
- `demo_controller_gui` 패널 (Catching 탭) 과 `plot_rtc_log` CSV 플롯을 함께 구현 · 확인한다. 포구 상태는 `rtc_msgs/CatchingState` 로 GUI 에 보낸다.

비범위: vision 노드 구현 (형제 저장소 ball_perception). 시뮬레이션에서는 ball_perception 의 `sim_estimator_node` 를 그대로 쓰며 자체 vision 발행기를 만들지 않는다 (§4.3).

## 2. 코드 확인 게이트

이 절의 확인 사실은 §4–§5 가 서술한다 — 컨트롤러는 코어 + 바인딩 2 층이고, 명령은 `ControllerOutput.devices[]` (팔 device 0, 손 device 1) 로 나가 CM 이 `ValidateControllerOutput` → `DeviceBackend::WriteCommand` 한다. `rtc_mujoco_sim` 은 ros2_control · `/clock` 이 없고 lock-step (명령 대기), stamp = wall 이다. 기록은 기존 CSV · timing 인프라 (`rtc::ThreadCsvProducer`/`rtc::ThreadCsvLogger`, `rtc::ThreadTimingCsvLogger`) 를 쓰고 새 기록 스레드를 만들지 않는다.

## 3. 참고자료

[R8] NEES/NIS, [R13] Wilson 구간, [R14] MuJoCo, [R11] UR 드라이버.

## 4. 수학적 이론

### 4.1 RT 틱 내 실행 순서

`Compute(const ControllerState&) noexcept` 한 번 안의 순서를 고정한다. 순서가 바뀌면 1 틱 지연이 생긴다. `Compute` 는 **단일 exit** 이다 — 모든 분기가 마지막의 기록 발행 (13) 에 닿는다.

1. **기록 초기화 · 입력 판독.** tick 기록을 새로 생성한다 (이 tick 이 계산하지 않은 블록은 0 — PROC-7 이 구조적으로 성립한다). E-STOP latch 를 tick 당 **한 번** 읽는다. 팔 · 손 device 의 판독 가능 여부를 정한다
2. **reset 서비스.** 활성화 · 재무장 · fault reset 요청을 처리한다 (`ServiceResetRequests`). 목표 drain · 명령 쓰기보다 **앞**이다 — reset 은 대기 중인 목표를 버리고 hold latch 를 내리므로 먼저 소비한 것은 버려질 상태가 된다. reset 직후 · 대기 자세를 읽기 전에 switch 된 자세 채택 (`planner.wait_pose_source`) 을 한다
3. **목표 drain.** E-STOP 중에는 drain 하되 **버린다** (큐에 남기면 해제 뒤에 오래된 목표가 손에 간다)
4. **공 입력 lane.** 궤적 스냅샷 SeqLock 을 tick 당 한 번 `Load` 한다 (D-21). 그 뒤 tick 의 **유일한 시계 읽기** 를 한다: $now$ = steady 실측, $now_{lead}=now+T_{arm}$ (L0 §4.5 — tick 수 × `dt` 로 계산하지 않는다). 스냅샷의 새로움 · stale · track 변경을 판정하고, 따르는 트랙의 스냅샷을 보관한다
5. **접촉 lane.** 손 지문 센서 (`RunContactLane`): 손이 `q_pre` 에 정지한 ARMED · TRACKING 에서는 바이어스를 학습하고, COMMITTED · CLOSING · DECEL · HOLD 에서는 접촉을 판정한다 (L7 §4.4)
6. **plan lane.** plan 상자를 tick 당 한 번 `Load` 하고 `JudgePlan` 으로 채택 가능 여부를 판정한다 (L3 §5.2: 유효 · 현재 activation · 같은 트랙 · 새 `plan_id` · 나이 ≤ `io.t_stale` · 마지막 reset 이후 · freeze 창 밖). 계획기가 없는 profile 이면 oracle 이 같은 상자에 먼저 쓴다
7. **segment lane (`mpc` 만).** 계획기가 게시한 정지 구간 상자 (`SegmentSnapshot`) 를 판정한다 — TRACKING 에서는 채택 가능한 plan 에 대한 첫 구간을, APPROACH 부터 DECEL 까지는 다음 구간의 대기 슬롯 채택을, APPROACH 에서는 교체 plan 과 그 첫 구간의 쌍도 (L7 §4.3a). plan lane 이 방금 판정한 plan 에 대해 판정하므로 그 뒤다
8. **감독 + 모드별 법칙** (`EvaluateReason`, L7). 우선순위는 E-STOP > fault reset · 에스컬레이션 > 준비 상실 > 법칙 실패 > 시간 전진 > 기록 전용 (R-PREC) 이다. 공에 대해 아무것도 묻지 않는 모드 (ABORT_SAFE · IDLE · RETREAT · DECEL · HOLD · COMMITTED · CLOSING) 가 vision lane 보다 먼저 판정된다 — 착지 뒤 stale 한 스냅샷이 DECEL · HOLD · RETREAT 를 영원히 잡지 않도록. 팔 기준을 만드는 **법칙은 이 판정 안에서 돌고 그 verdict 가 이 tick 의 사유다**:
   - TRACKING: plan 이 채택되는 tick (`mpc` 에서는 plan 과 그 첫 구간이 함께) 에 TRACKING → APPROACH. 채택한 plan 이 없으면 `NO_CATCHABLE_PLAN` (self-loop)
   - APPROACH · COMMITTED · CLOSING, `closed_form`: `RunTrackingTick` — 공 샘플 (now_lead) → soft-catch 기준 (γ 프로파일, L4) → 확장 CLIK (L5). APPROACH 중 새 plan 은 계획기의 교체 규칙을 통과한 것만, freeze 창 밖에서만 reset 없이 재조준한다 (`SetIntercept`). COMMITTED 이후에는 **동결된 트랙** 의 마지막 스냅샷을 샘플링한다
   - APPROACH · COMMITTED · CLOSING, `mpc`: `RunSegmentTick` — RT 는 soft-catch DS 를 돌리지 않는다. 첫 구간의 node 0 을 기다리는 동안 seed 한 명령을 유지하고, 그 뒤 now_lead + $h$ 에서 plan 대조 · 전환 게이트를 통과한 구간 샘플 (pose · twist · $q_{ref}$ · $\dot q_{ref}$) 을 CLIK 목표로 넘긴다 (L5 §5.3). 공은 `closed_form` 과 같은 감독 사유 (stale · horizon) 를 위해 샘플링할 뿐 구간은 공을 읽지 않는다. APPROACH 의 plan 교체는 첫 구간과의 쌍으로만, 그 구간의 전환 tick 에 일어난다 (L7 §4.3a). COMMITTED 에서는 폐쇄 지령 전까지 손의 지령 시각을 공의 통과 시각에 다시 맞춘다 (L6 §4.3). 따를 구간이 없거나 plan 이 어긋나면 다른 법칙으로 fallback 하지 않고 `ABORT_SAFE` 다
   - COMMITTED → CLOSING: 손 시퀀서가 폐쇄 명령을 낸 tick (R-CLOSE)
   - CLOSING → DECEL: now_lead ≥ $t_c$. `closed_form` 은 reference 의 현재 상태를 진입 상태로 γ ≡ 1 의 가상 감속 대상을 따르고 (`EnterDecel` / `RunDecelLawTick`), `mpc` 는 같은 구간의 연속이다 (`RunSegmentTick` entry)
   - DECEL · HOLD: `closed_form` 은 `RunDecelLawTick`, `mpc` 는 구간 끝의 정지 샘플 (`RunSegmentTick`, 공 없이). DECEL → HOLD 는 정지 도달, HOLD 는 `T_hold` 뒤에 판정 (L7 §4.4) 하고 RETREAT
9. **모드 전이 · 관측 게시.** 판정이 전이이면 `AdvanceMode`. 모드 · 사유는 atomic 으로 게시한다
10. **관절공간 동작** (`RunArmMotion`) — 감독이 선택하므로 판정 **뒤**다: ABORT_SAFE · FAULT 의 정지 램프 (원인과 무관하게 항상 관절공간, QP 비의존), IDLE 의 homing, RETREAT 의 복귀
11. **손** (`RunHandStage`) — 이번 tick 의 edge (RETREAT 의 Release, Commit) 가 이 tick 에 wire 에 나가도록 팔 동작 뒤다: 손 시퀀서 → 손 device slot (L6)
12. **명령 쓰기.** `ControllerOutput.devices[0]` (팔, `kPosition`), `devices[1]` (손). 이후 CM 이 검증 (실패 시 `BuildHoldOutput`) · E-STOP 대체 (`BuildLatchedHoldOutput`) · `WriteDeviceCommand` 를 한다. device 상태 로그 push
13. **RT 상태 게시:** 계획기용 `PlannerRtState` SeqLock write (모드 · 기준 상태 · 따르는 구간과 대기 구간), 상태 publisher 용 기록 (`CatchingDiagLogPod`) 을 SPSC push — 이 기록이 상태 토픽과 CSV 의 **같은 한 벌**이다 (PROC-7)

### 4.2 투척 생성 (시뮬레이션)

**발사 API.** 발사는 (p0, v0, ω) 를 명시하는 srv `rtc_msgs/srv/LaunchBall` (`/sim/launch_ball_at`) 로 부른다. 기존 `/sim/launch_ball` (Trigger, YAML 분포 샘플) 은 파라미터 설정 + Trigger 조합이 경합 · 재현성에 약해 평가 경로에 쓰지 않는다.

**발사 조건.** 발사 조건은 §4.6 의 발사 영역에서 직접 표본 추출한다: arm base frame (CLIK `base_frame`) 기준 수평 거리 4 m 원호 위의 방위 φ, world z 1.5–2.0 m, 속도 크기 · 앙각, 수평 방향 = (발사점 → 겨냥점) + 편차. 표본 범위는 catchability 지도 (§4.6) 가 정한 분포다. `sim.throw_region` 이라는 YAML 키는 없다 — 투척 분포는 시행 러너 `catching_sim_trials` 의 `--dist` 가 갖는다 (`integrated_bringup/README.md` §Catching sim trials).

- 항력 · Magnus 는 `rtc_mujoco_sim` 이 자체 구현한다 (tennis preset). 계획기의 공 모델과 다르므로 도달점 차이는 의도된 모델 불일치로 기록한다
- 재현성: srv 인자 자체를 시행 기록에 남기고, 난수는 평가 스크립트 쪽 seed 로 뽑는다

### 4.3 시뮬레이션 vision — ball_perception `sim_estimator_node`

ball_perception 의 `sim_estimator_node` 가 `/sim/ball/camera_position` 을 구독해 `/ball_perception/debug/prediction/trajectory` (`PointCloud2`) 를 발행하므로, L1 파서는 sim 과 실기에서 **실제 발행기와 같은 레이아웃** 을 탄다 (D-4).

- 연결: clock domain `ros_system_time` + `use_sim_time=false` (rtc 에 `/clock` 없음, stamp = wall epoch — 공 lane 은 발사 기준 sim 시간축을 wall 에 얹은 값을 찍는다, `rtc_mujoco_sim` README §Projectile Ball stamp), `frame_id` = `world`. 지연 · 드롭 주입은 `rtc_tools camera_relay` 로 입력 토픽 앞에서 한다
- 측정 잡음은 `rtc_mujoco_sim` 의 `publish.position_noise_stddev_m` 이 준다
- 제어 경로에서 truth 토픽을 쓰지 않는다. truth 는 지표 · NEES 전용
- 자체 fixture EKF 는 만들지 않는다 (`ball_dynamics` 는 test fixture 전용)

**vision 요구 사양.** 목표 투척 분포에서 "검출 이후 포구 창 종료까지 최대 비행 시간" → 필요 지평 $H_{req}$, L2 보간 게이트를 만족하는 간격 → 점 수로 산출한다. 값은 sim profile (`sim_estimator.launch.py profile_path:=`) 과 예측 격자 (MPC 계획 · `mpc_multiframe_clik_formulation.md` §1.7) 가 갖는다. 수신 궤적의 지평이 요구 (`io.horizon_min`, L1 §6) 보다 짧으면 제어기는 계획 후보에서 제외하고 진단한다.

### 4.4 지표

**예측 일관성 (시뮬레이션, 오프라인).** ball_perception 이 발행한 **예측** 점 $k$ (예측 원점 `header.stamp` + `horizon_ns`) 와 같은 시각의 truth 로 NEES 를 계산한다.

$$\epsilon_k=(x_{true}(t_k)-\hat x_k)^\top P_k^{-1}(x_{true}(t_k)-\hat x_k),\qquad x=(p,v)\in\mathbb R^6$$

평균이 상태 차원 6 에 가까운지 지평별로 본다 ([R8]). `covariance` 가 NaN (모름) 인 점은 제외하고 그 비율을 따로 기록한다.

- truth `publish.sample_rate_hz` 가 낮으면 $t_k$ 보간 오차가 커질 수 있다
- truth 는 공이 활성 (발사 후, park 전) 인 동안만 의미가 있다 — 그 구간만 쓴다
- stamp 는 둘 다 wall 이다 (D-3). δ 가 큰 구간에서만 오차가 커지는지는 §4.5 로 가른다

**포구 지표 (시행별).**
- $t_c$ 에서 간극: $\Vert p_{true}(t_c)-p_C(t_c)\Vert$ (catch frame 실제 위치, D-10)
- 상대속도: $\Vert v_{true}(t_c)-\dot p_C(t_c)\Vert$
- **충격량** $\Delta p=m_{ball}\Vert v_{true}(t_c)-\dot p_C(t_c)\Vert$, 접촉 지속 $\Delta t_{imp}$, 평균 · 최대 접촉력, 손가락 관절 최대 토크 (시뮬레이션 접촉 truth) — L7 §4.7
- 계획 $\gamma_f$, `REF_SATURATED` 발생, 전이 사유, 결과, catchability $w_5$ (D-18) 와 CLIK `Manipulability()`

**성공률 신뢰구간 (Wilson, [R13]).** $n$ 회 중 $s$ 회 성공, $\hat p=s/n$, $z$ 는 정규 분위수:

$$\frac{\hat p+\frac{z^2}{2n}\pm z\sqrt{\frac{\hat p(1-\hat p)}{n}+\frac{z^2}{4n^2}}}{1+\frac{z^2}{n}}$$

합격 하한 (floor) 과 시행 수는 D-12 (사용자 제공 값).

**오차 예산 검증.** L3 §4.6 예측 간극 분포와 실제 간극 분포를 비교한다 (모델 검증).

### 4.5 sim 시간축과 clock 위상

sim 은 wall clock 을 유지한다 (D-3). D-2 변환이 실기와 같은 경로로 동작하게 하려는 선택이다. 기존 RTF 신호 (200 step 구간 평균) 는 0.5 s 비행 동안 표본이 1~2 개뿐이라 짧은 스톨을 평균이 지우므로, 판정은 **비율 (RTF) 이 아니라 시행별 clock 위상 오차** 로 한다.

**측정.** per-step `(sim_time, steady_now)` 진단 lane (launch 인자 `sim_lanes:=true`) 이 데이터 원천이다. 발사 시각을 원점으로 비행 구간 매 step 에서 δ(t) = (steady(t) − steady₀) − (sim(t) − sim₀) 를 구하고, 시행별 δ_max = max|δ| 와 max pause (한 step 의 Δwall − Δsim 최댓값) 를 기록한다 (양방향).

**무부하 판정 (clock 게이트).** v_max·δ_max + ½·a_bound·δ_max² ≤ ε_clk,alloc **이고** max pause ≤ ε_clk,alloc / v_max. v_max 는 목표 투척 분포의 최대 공 속력, a_bound 는 g + 항력 가속 상한 (항력 k 는 L0 §4.1 의 문서 대표값). **ε_clk,alloc 은 L3 §4.6 오차 예산 중 시계 항에 할당한 몫이고, 예산 우변은 `r_cap/n_σ` 다.** ε_clk 는 ‖v‖δ 라 **목표 속력에 비례** 하므로 판정은 목표 속력과 함께 읽는다. 구성별 (로봇 2 종) 발사 ≥ 200 회, 무효율 ≤ 5 % (제안값). 이 상한은 **무부하 판정에만** 쓴다 — 성공률 평가에서는 ε_clk,alloc 이 판정이 아니라 **공변량** 이다: 시행별 δ(t_commit) · δ(t_c) · tick overrun 수 · 최대 tick 간격을 기록하고, D-3 효과는 같은 seed 투척의 부하 A/B 로 상계한다.

host 부하로 sim 이 실시간보다 느리면 발사 기준 sim 시간축의 공 stamp 가 벽시계보다 뒤처져 컨트롤러 steady 나이 검사가 입력을 `BALL_STALE` 로 끊는다. 러너 `catching_sim_trials --host-watch off|warn|abort` 가 투척마다 truth 행의 RTF 를 판정하고 (`abort` 는 exit code 3, 같은 seed 로 unit 재실행), 분석기 `catching_trials` 는 `t_c` 열이 포구 순간에서 벗어난 행을 `tc_axis` 로 표시해 `t_c` 중앙값에서 뺀다. 성공률 평가의 무효는 **rig 실패만** (srv 거부 · lane drop · sim stall · 미발사 — 기계 판정 5 종) 이고 "plan 없음 · abort" 는 실패다 — 전체 발사 수 · 무효 사유 · 무효를 실패로 센 ITT 하한을 함께 보고한다 (§9.1 G8-D).

처리량: lock-step 에서 `max_rtf` 1.0 은 상한이므로 RTF ≤ 1 이다. 시행당 발사 · 비행 · 감속 · 복귀 · 재무장을 합쳐 수 초가 걸려 **200 시행에 약 12 분** 이상이 든다 — 평가 일정은 이를 전제로 짠다.

### 4.6 catchability 지도 도구

지도의 정의는 이 절이고 게이트 정의는 L3 §4.2 (D-18), 키는 L3 §6 의 `planner.search.grid.catchability.*` 다. 요점:

- 발사 영역: arm base frame 수평 거리 √(x²+y²) = 4 m 원호 (방위 φ), world z 1.5–2.0 m, 비행시간 T_f ≥ 1.0 s, 수평 방향 = (발사점 → 겨냥점) + 편차, 속도 · 앙각 격자
- 판정: 궤적 위 포구 후보마다 catch frame +z = −v̂ 자세의 IK (대기 자세에서 시작) → 게이트 정의 (기본 `arm_5row`) 의 manipulability ≥ 정의별 threshold ($w_5$; `search_grid.yaml` 의 `planner.search.grid.catchability`). $w_5$ · $w_6$ 를 모두 기록해 두 분포를 비교한다. 런타임 계획기와 **같은 함수 · 같은 YAML 키**
- **frame 함정:** `ur5e_p1b` 의 arm base frame 은 URDF `base` 이고 `base_link` 와 z 둘레 180° 다르다 — `base_link` 로 두면 공이 등 뒤에서 날아오는데 결과가 그럴듯해 조용히 틀린다. world ↔ base 변환은 같은 q 에서 MuJoCo FK 와 Pinocchio FK 대조로 확정한다. 새 로봇을 추가할 때마다 재발하는 함정이다
- 지도는 두 단계다: kinematic 지도 (IK + $w_5/w_6$) 와, 손 · 팔 값 ($T_{close}$, $d_{eff}$, 가속 box, $\eta_v$) 으로 전체 게이트 체인을 다시 돌린 gate-catchable 지도 — 후자가 목표 투척 분포가 되고 vision 요구 사양 산출로 이어진다
- 출력: 격자별 포구 가능 여부, 최대 $w$ 와 그 후보의 $t_c$ · $p_c$ · $q^*$, 탈락 사유 → 목표 투척 분포 제안
- 도구: `ros2 run rtc_tools catchability_map` (python — 격자 · 항력 비행 · 집계 · 그림) 이 `ros2 run rtc_controllers catch_pose_ik_batch` (C++ — 판정) 를 샤딩해 부른다. 판정을 python 에서 다시 구현하지 않는 이유가 "같은 함수" 다. 두 함정: 판정기는 `p_c` · `v` 를 **모델 world** (URDF 모델 root) 로 받고 이것은 arm base frame 이 **아니다** (`ur5e_p1b` 에서 Rz(180°) 차이); 출하 `sub_models.<arm>` 은 flange 에서 끝나 catch frame 을 담지 않으므로 지도는 arm root → catch frame 부모까지의 sub-model 을 따로 선언한다. 판정기는 `catching/search_grid.yaml` 을 직접 받는다

## 5. C++ 구현

### 5.1 컨트롤러 구조

정의는 `integrated_bringup/include/integrated_bringup/controllers/demo_catching_controller.hpp` (`DemoCatchingController`, 구현은 `src/controllers/catching/`) 다. 구조의 규범:

- 수치 코어는 `rtc::catching` (`rtc_controllers`), 바인딩은 `integrated_bringup`. lifecycle: `on_configure` (non-RT) 에서 `LoadConfig` + `ParseXxxParams` + 검증, `PinocchioCache` · 모델 핸들, `PointCloud2` 필드 맵, 전 버퍼 할당, 구독 · publisher 생성; `on_activate` 에서 $q_c$ · CLIK 앵커 = $q_{meas}$, 계획기 스레드 layout 게이트 → spawn → Resume (`Mode::kIdle`); `on_deactivate` 에서 계획기 Pause (join 은 `on_cleanup`)
- E-STOP · fault 훅 (`TriggerEstop`/`ClearEstop`, `ResetFault`/`HasLatchedFault`) 은 atomic 요청 · epoch 만 갱신하고, 되돌리는 동작의 **유일 writer 는 `Compute()`** 다. 운용자 무장 채널은 파라미터 `catching.enable` — 콜백은 atomic 만 쓰고 tick 이 소비하며, tick 이 E-STOP · fault 에서 그 latch 를 내린다. 정책은 L7 §4.1
- 궤적 구독은 controller 소유 구독이라 `nrt_callback_executor` (단일 스레드) 에서 돈다 — 파싱 · D-2 변환 · SeqLock write · 계획기 eventfd 신호만 하고 계산은 계획기 스레드로 넘긴다
- 계획기 스레드: `rtc::PeriodicRtThread` 의 형제 subclass (`CatchingPlannerThread`), event 구동, slot · 스케줄러는 L3 §5.3. 배치 변경은 E-7 이며 Adding a New Thread 절차를 따른다
- SeqLock payload 는 trivially copyable POD (`std::array` 기반, Eigen 멤버 금지): 궤적 스냅샷 (`TrajectorySnapshot`), plan 상자 (`PlanSnapshot`), 정지 구간 상자 (`SegmentSnapshot`, `mpc`), RT 상태 (`PlannerRtState`), tick 기록 (`CatchingDiagLogPod`). 정의는 `rtc_controllers/include/rtc_controllers/catching/{trajectory,planner_io}.hpp` 와 `integrated_bringup/include/integrated_bringup/logging/catching_diag_log_pod.hpp`
- 상태 publisher: `rtc_msgs/CatchingState` + `SetupCatchingStatePublisher`, controller 소유 `rtc::SeqLock<T>` 패턴 (`WbcState` · `GraspState` 선례). `PublishRole` 에 추가하지 않는다 (E-11). **필드는 한 번에 동결** 되어 있고 이후는 값만 채운다. `Compute()` 의 **모든 tick 에서 Store** 한다 — E-STOP · stale · generation 불일치 · 지평 부족 · plan 없음 · abort 를 포함한 모든 early-return 분기에서도, 계산하지 않은 필드는 무효화한다 (PROC-7, agent_docs/invariants.md). 단일 exit 와 tick 머리의 기록 재생성이 이것을 열거가 아니라 **구조적 성질** 로 만든다. DemoWbc 의 `!target_initialized_` early-return 이 Store 를 빠뜨리는 것은 알려진 gap 이지 선례가 아니다. ingress 카운터는 **메시지 도착** 에 움직이는 값이라 이 기록에 없고 별도 SeqLock (`CatchingIngressSnapshot`) 으로 발행 스레드가 읽는다
- 손은 device slot 에 직접 쓴다 (손 명령 포트 추상화 없음)

### 5.2 기록 레코드

tick 마다 한 행의 기록은 `CatchingDiagLogPod` 이다 (`integrated_bringup/include/integrated_bringup/logging/catching_diag_log_pod.hpp`). 두 가지가 규범이다. (1) 상태 메시지와 **같은 POD 한 벌** 을 쓴다 — 파일의 숫자와 화면의 숫자가 갈릴 수 없게 하려는 것이고, 그래서 float 이 아니라 double 로 싣는다. (2) `flags` 비트필드 대신 이름 있는 bool 을 쓴다 — CSV 헤더가 비트 이름을 실을 수 없어 저장된 파일이 그 run 의 헤더 없이는 해독 불가가 된다. 공 truth 는 컨트롤러에 없으므로 이 기록에 없다 (오프라인 평가 소관). CSV 열 목록의 SSoT 는 pod 헤더 (헤더 행 emit) 이고, 운용 설명은 `integrated_bringup/README.md` §DemoCatchingController "tick 레코드 CSV", 열 해석은 `rtc_tools/README.md` 다. 구간 lane 의 열은 `segment_*` 이고, 열 이름이 `decel_*` 인 옛 recording 은 도구가 별칭 표 한 벌로 읽으며 옛 이름과 새 이름이 섞인 파일은 거부한다. 기록 경로: RT 는 SPSC 에 push 만 하고, drain 과 CSV 쓰기는 기존 CSV 인프라가 맡는다 (새 기록 스레드 없음). 계획기 timing · 이벤트 (`planner_events.csv`) 는 aux 타이머가 drain 한다.

토픽 기록: rosbag2 로 vision `PointCloud2`, truth (시뮬레이션), 상태 토픽을 기록한다.

### 5.3 launch 구성

새 launch 파일을 만들지 않고 **기존 `integrated_bringup` launch 에 인자를 더한다** (중복 launch 금지). launch 파일 · 인자 표는 `integrated_bringup/README.md` §Launch 파일 와 각 launch 의 `DeclareLaunchArgument` 가 갖는다. 포구 관련 launch 는 `sim_ur5e_p1b` · `sim_iiwa7_leap` · `sim_g1_p1b` (sim, `mujoco_native`) 와 `robot_ur5e_p1b` (실기, `ur_driver_native` + `udp_hand_native`) 다. 컨트롤러 선택은 기존 `initial_controller` 인자를 쓴다.

### 5.4 YAML 파일

- 포구 컨트롤러 설정: 로봇별 `integrated_bringup/config/<robot>/controllers/demo_catching_controller.yaml` (config_key `demo_catching_controller`). 이 파일이 CM 의 `include:` 로 기능별 파일을 합친다 — `catching/search_grid.yaml` (`grid` 탐색), `catching/search_nlp.yaml` (`nlp` 탐색), `catching/planner_closed_form.yaml` (closed_form 법칙), `catching/segment_mpc.yaml` (mpc 법칙), `catching/segment_mpc_docking.yaml` (mpc_docking 법칙). 다섯 조각은 **항상 모두** include 하고 무엇이 도는지는 주 파일의 두 선택자 — `planner.search.mode` (`grid` \| `nlp`, 출하 `grid`) 와 `planner.segment.mode` (`closed_form` \| `mpc` \| `mpc_docking`, 출하 `mpc`, 코드 기본 `closed_form`) — 가 정하고 요구는 선택을 따른다 (L3 §6). 조각은 키의 전체 경로를 그대로 적고 한 키는 한 파일에만 있다. 한 기능이 쓰는 설계값은 그 기능의 조각에 있고 두 기능이 같은 수를 읽으면 key 를 각자 갖는다 (L3 §6). 키가 어느 파일의 것인지와 구조는 `integrated_bringup/README.md` ("config 는 주 파일과 조각 다섯이다"), `rtc_controller_manager` README 의 `include:` 절, 각 YAML 의 머리 주석이 갖는다
- `ur5e_p1b` 와 `iiwa7_leap` 만 이 파일 묶음이 있다. `g1_p1b` 는 없다
- catch frame (`extra_frames`) 은 로봇 config 의 모델 절 (`_base.yaml`, L5 §11)
- 투척 · catchability (`planner.search.grid.catchability.*`) 키는 L3 §6, 투척은 목록 파일 (`catching_sim_trials --throws-file`, §12) 또는 손 근처 설계 (`--dist hand_*`, §4.2)
- sim 공 설정 (projectile, truth 주기) 은 로봇별 `mujoco_simulator.yaml`
- 로드 후 L0 검증기를 실행한다. provisional 값 (D-12 · D-17) 은 실기 arm 을 막는다

## 6. YAML 파라미터

키 · 값 · 기본값은 YAML 과 파서 (`rtc_controllers/src/params/catching_params.cpp`) 가 갖는다. 이 절에 키 표는 두지 않는다. 구현되지 않은 키 (`logging.decimation`, `logging.ring_capacity`, `sim.clock.min_launches`, `sim.clock.max_invalid_rate`, `sim.throw_region`) 는 존재하지 않는다 — 기록은 기존 CSV 인프라의 세션 디렉토리를 쓰고, 시행 수 · 무효율은 러너 인자와 분석기의 일이다. sim 측 키는 `rtc_mujoco_sim` 의 `publish.sample_rate_hz` · `publish.position_noise_stddev_m`, 예측 발행률은 ball_perception profile 이다.

**실기에서 검증 불가능한 것 `[권장]`.** 실기에는 $p_{true}(t_c)$ 가 없으므로 "예측 간극 분포" 를 직접 잴 수 없다. 관측 가능한 것은 포획/실패 이진 결과와 지문 접촉 시각뿐이다. 따라서 실기 게이트 (G8-G) 는 다음 중 하나로 대체한다 — **선택은 실기 단계** 가 한다.

1. 포획률 대 $\sigma_c$ 의 로지스틱 회귀로 유효 $r_{cap}/\kappa_\sigma$ 를 역추정.
2. 접촉 센서 조합 · 접촉 시각 편차를 간극의 대용 지표로 사용.
3. 저속 구간 1 회 한정 외부 계측 (마커 또는 고속 카메라) 캠페인.

셋 다 하지 않으면 **"실기 vision 공분산은 미검증 가정"** 임을 위험으로 기록한 채로 진행한다.

## 7. 단위 기술 구현 순서

구현 순서와 단계 결과의 기록은 두지 않는다. 시스템 게이트는 §9.1, 실기 단계적 도입은 §9.2 다.

## 8. 디버깅 방법

- 틱 실행시간 분해: §4.1 단계별 시간 히스토그램.
- 결과별 타임라인 자동 생성: 실패 · abort 시행만 모아 L7 §8 그래프를 만든다.
- 재현성: 발사 srv 인자 · 난수 seed · YAML 해시를 시행 기록에 넣는다.
- 틱 예산 분해: 스냅샷 복사 바이트, SeqLock 재시도 횟수, QP 반복 수 · solve time 을 따로 기록한다. 예산은 `dt` (= 1/`control_rate`) 이고 총량이 아니라 **분산의 꼬리** 가 위험하다.
- 오차 원인 분해: 간극을 L3 §4.6 의 항별 ($A=p_{true}-\hat p_{live}$, $B=\hat p_{live}-p_c$, 추종, 시계) 로 분해해 표로 만든다. 시뮬레이션은 truth 가 있으므로 각 항을 직접 계산할 수 있고, $A\perp B$ 가정도 검증할 수 있다 (G8-C2).
- 충격 구간 분해: $t_c$ 전후 100 ms 의 접촉력 · 관절 토크 · $q-q_c$ 괴리를 겹쳐 그린다 (L7 §4.7).
- 무효 시행 급증: §4.5 clock 위상 오차 (δ_max · pause) 로그와 부하 구성을 확인한다.
- `mpc` 에서 "기준" 은 `ref_x` 가 아니라 `catching_diag.csv` 의 `segment_*` 열 (따르는 구간이 CLIK 에 준 목표 위치 · 선속도 ff) 이다.
- E-STOP · fault 구간: `catching_diag` 플롯이 `estop_active` (CM 의 global latch) 와 `fault_latched` (컨트롤러 latch) 구간을 색을 달리해 음영으로 보인다 — 두 latch 는 해제 수단이 달라 한쪽만 끝나는 구간이 있을 수 있다. 해제는 GUI 헤더의 "Clear E-STOP" (사유 조회 → 확인 뒤 해제, 2 단계) · "Reset fault", 절차는 L7 §4.1. 사유 코드 `FAULT_RESET` · `ABORT_ESCALATED` 는 그 한 tick 에만 실리므로 상태 토픽을 폴링하는 쪽은 놓치기 쉽다 — CSV 로 본다. fault 의 **원인** 은 CSV 열 `fault_cause` 로, 거부된 reset 은 `fault_reset_refused` 로 본다 (상태 메시지에는 없는 CSV 전용 열). `rtc_tools` 의 catching 요약이 원인별로 센다.

## 9. 검증 방법과 합격 게이트

### 9.1 시뮬레이션

| 게이트 | 기준 | 태그 |
|---|---|---|
| G8-A | 전체 시행에서 RT 위반 0 (할당, 틱 초과 기준은 RTC 규약). 계획기 스레드가 RT 와 다른 코어에 있는지 확인. **개발 PC (PREEMPT_RT 아님) 의 RT · CPU 격리 결과는 smoke 로만 보고, 판정은 제어 PC 에서** 한다 | `[SIM-ANY]` |
| G8-A2 | **연속 2 회 투척** 과 **abort 직후 재투척** 시나리오에서 L7 §4.8 재무장 리셋 목록이 전부 동작 (옛 plan 재사용 0, 복귀 위치가 `wait_pose`, 첫 solve 의 $\dot q_{prev}$ 가 0) | `[SIM-ANY]` |
| G8-H | early-return 분기마다 그 tick 의 body 가 실렸는지 보는 실패 경로 테스트 (PROC-7, `EstopTickPublishesThisTicksBody*` 선례). 분기: E-STOP · stale 입력 · generation 불일치 · 지평 부족 · plan 없음 · `ABORT_SAFE` · 손 단계 조기 반환 · `HAND_TIMEOUT` · 손 관절 캡처 | `[SIM-ANY]` |
| G8-B | ball_perception 예측의 NEES 평균이 [R8] 구간 안 (지평별, NaN 공분산 제외, §4.4). **판정식**: `sim_estimator.launch.py record:=true` 의 capture 를 `sim_capture_evaluate` 로 평가하고, 표본을 발사 창으로 시행에 묶은 시행별 · 지평별 평균 NEES 의 시행 부트스트랩 95 % CI 의 **상한 ≤ 3 ∧ coverage_95 ≥ 0.95** 이면 PASS (위치 차원 3). 막는 것은 과소 추정이다 — sim 공의 실제 항력 오차는 profile 이 실제 공을 위해 둔 불확실성보다 훨씬 작아 보수 쪽 (NEES < 3) 은 예상값이다; 과대 쪽은 commit 시점 예측 오차로 본다 (임계 미정, 기록). 부호 있는 편향은 보고 (추정기가 중력만 모델하므로 긴 지평 편향은 예상값). NaN 공분산 > 10 % 인 지평 bin 은 `NOT_EVALUATED`. 캡처는 평가 unit 에서 필수 | `[SIM-ANY]` |
| G8-B2 | L1 파서가 ball_perception 실제 레이아웃 (필드 이름 · datatype) 을 검사하고 레이아웃 해시 진단이 변경을 감지 — sim · 실기가 같은 발행기 계열이라 복제 레이아웃 대조는 불필요 (D-4) | `[SIM-ANY]` |
| G8-B3 | 실기 vision 공분산의 일관성: §6 세 수단 중 실기 단계에서 고른 방법 (G8-C2 와 짝) | `[HW-P1B]` |
| G8-C | `ur5e_p1b` 에서 $\gamma=0$ 고정 (`planner.search.grid.gamma.grid: [0.0]` + `planner.search.grid.hand.d_eff: 10.0` overlay — grid 만으로는 γ 창 하한 γ_min 이 대신 쓰인다) 대 계획 γ ablation: 간극 · 상대속도 · **충격량** · 접촉력 분포 비교. G8-E 와 함께 **2×2 factorial** (lead {off, on} × γ {0, 계획}, 4 arm × 4 블록 × 25 seed — overlay 는 기동 때만 읽혀 arm 마다 재기동, 블록 안 arm 순서 무작위) 의 lead on 에서의 γ 주효과로 판정하고 상호작용을 보고한다 | `[SIM-ANY]` |
| G8-C2 | 오차 예산 모델 검증: L3 §4.6 직교 분해식의 예측 간극 분포와 실제 분포 비교. truth 가 있으므로 $A=p_{true}-\hat p_{live}$, $B=\hat p_{live}-p_c$ 의 직교성도 직접 확인한다. **$\hat p_{live}$**: commit tick 의 diag `input_snapshot_sequence` 를 `vision_lane_probe --dump` 의 `snapshot_sequence` 와 정확히 결합하고, dump 에 없는 시행만 commit 직전 마지막 prediction 으로 근사해 근사 수를 보고한다; 시행 클러스터 부트스트랩, n ≥ 100 | `[SIM-ANY]` |
| G8-C3 | 삭제 — γ 하향은 v1 범위 밖 (D-8). 대신 `REF_SATURATED` 빈도를 기록해 D-8 재검토 입력으로 쓴다 | `[SIM-ANY]` |
| G8-D | `ur5e_p1b` 에서 **truth 기반 성공** (HOLD 끝부터 대기 자세 release 까지 공이 손에 있음) 의 Wilson 95 % 하한 (97.5 % 단측, z 1.96) ≥ floor (D-12 — **0.35**, 동결; n_valid 200), 결과별 사유 분포, 슈퍼바이저 판정의 truth 대비 혼동행렬. 무효 = rig 실패만 (srv 거부 · lane drop · sim stall · 미발사 · 컨트롤러 무응답 — 기계 판정 5 종, 시행마다 `invalid_reason` 하나, 우선순위 `srv_refused` (`accepted == false`) > `not_launched` (accepted 인데 truth 행 0) > `controller_silent` (러너가 컨트롤러 상태를 못 봤거나 시행 창에 diag 행 0) > `lane_drop` (clock lane 에 그 `launch_seq` 가 없거나 `dropped_total` 증가) > `sim_stall` (`sim_time_sec` 간격 > 5 × 공칭 step). 판정은 `rtc_tools` `catching_trials` 의 `record_invalid_reason` · `lane_invalid_reason`) — 그 밖의 것 (plan 없음 · abort · `HAND_TIMEOUT` · 미종결) 은 실패. host 부하 unit 은 같은 seed 로 재실행하되 재실행 규칙은 결과를 보기 전에 선언한다 (D-S8-17). beanbag arm 은 같은 seed · 같은 floor 로 병기하고 tennis 대 McNemar 쌍 비교. 전체 발사 수 · 무효 시행 수 · 무효 사유 · ITT 하한 (무효 = 실패) · δ 공변량을 함께 보고한다 | `[SIM-P1B]` |
| G8-D2 | `iiwa7_leap` 에서 G8-D 와 같은 정의 (truth 기반 성공 · Wilson 97.5 % 단측 하한 ≥ floor · 무효 = rig 실패만 · ITT 하한 · δ 공변량). 평가 대상 여부는 조건부다: 실측 선행시간에서 `iiwa7_leap` 의 gate-catchable 지도 (§4.6) 가 비어 있지 않으면 평가하고, 비어 있으면 `NOT_EVALUATED(선행시간)` 로 그 실측 선행시간과 함께 보고한다 (막는 것은 손이 아니라 선행시간이다). 새 seed 로 n_valid 200, floor 0.35 | `[SIM-ANY]` |
| G8-E | `ur5e_p1b` 에서 선행 보상 유무 비교. sim 팔 actuator 가 1차 지연이므로 (L5 §4.4) fixture 없이 **sim 런타임 lead on/off** 로 비교한다 — overlay `catch_lead_*` (`joint_cmd.lag.{T_arm, lead_enable}` + `T_freeze`, 두 arm 동일 `T_freeze`). 2×2 factorial (G8-C 행) 의 계획 γ 에서의 lead 주효과. **판정 = lead 반영 $t_c$ 서보 잔여 ‖FK(q_meas($t_c$)) − FK(q_cmd($t_c − T_{lead}$))‖ ($T_{lead}$ = diag `t_arm_s`) 중앙값이 lead on 에서 감소 (부호 일치 + 쌍 Wilcoxon)**, 1차 지연의 비-순수지연분인 잔여를 보고, 성공률 차는 기록. 같은 tick 의 ‖FK(q_meas) − FK(q_cmd)‖ 는 lead on 에서 의도된 선행을 더해 거짓 FAIL 하므로 `cmd_meas_gap_mm` 로 기록만 한다. sim G8-E 는 **sim 서보 게인의 1차 플랜트** 위 검증이며 UR5e 이득을 예측하지 않는다. $T_{arm}\neq0$ 순수 지연 fixture 판정은 L5 §9 G5-E 다 | `[SIM-P1B]` |

모든 sim 게이트는 δ (§4.5) 를 공변량으로 병기하고, 무효는 rig 실패만이다. 처리량은 §4.5 (200 시행 ≈ 12 분 이상).

### 9.2 실기 단계적 도입 `[권장]`

1. **공 없이 재생.** 기록된 vision `PointCloud2` bag 을 재생한다. D-2 변환은 수신 wall 시각과 stamp 차를 쓰므로 **재스탬프 도구** 가 필요하다. 저속 설정 (투척 속도 축소 bag) 으로 로봇 · 손 동작과 타이밍을 확인한다.
2. **가상 공.** 제어 PC 내부 가상 궤적으로 전 구간 (감속 · 복귀 포함) 을 확인한다.
3. **실제 공, 저속 토스.** 부드러운 공, 짧은 거리, 낮은 속도로 시작한다.
4. **속도 단계 상향.** 각 단계에서 G8-D 지표를 기록하고 다음 단계로 간다.

실기 단계 착수 전 E-STOP · fault 정책 (D-13) 완료가 필수다. 각 단계 진입 전 `T_arm`, `T_close_tot`, 시계 offset, 그리고 **예측 일관성 지표 $\bar\nu$** (L1 §4.5) 를 확인한다. speed scaling · PTP 감시는 신호 출처를 확보한 뒤 연결한다.

| 게이트 | 기준 | 태그 |
|---|---|---|
| G8-F | 단계 1–2 완료, abort 경로 전부 실기 동작 확인 | `[HW-P1B]` |
| G8-G | 단계 3 이후 성공률 · 간극 분포 기록, 시뮬레이션 대비 차이 원인 분해 — "원인 분해" 는 항목별 수치 기록 (§8 오차 원인 분해 표의 항목별 값) 으로 판정한다 | `[HW-P1B]` |

## 10. 미확정 항목

- TBD-HAND-03 (L6), D-3 재검토 (host 부하 시 `BALL_STALE` 는 D-S8-17, 감시는 러너 `--host-watch`), `ε_clk,alloc` (목표 속력 확정 뒤), D-12 값 (투척 속도 · 성공률 하한 — 시행 수 n_valid 200, floor 0.35 동결), 실기 공분산 검증 수단 (§6 세 수단 중 선택), 재스탬프 도구 (실기 단계), `g1_p1b` 의 포구 컨트롤러 config

## 11. GUI · plot 이 보여 주는 것

컨트롤러가 내는 상태 (`rtc_msgs/CatchingState`) 와 기록 (`catching_diag.csv`, `planner_events.csv`) 은 `demo_controller_gui` 에서 보이고 `plot_rtc_log` 로 그려져야 한다 — 새 상태 · CSV 는 둘을 함께 갖는다. GUI 패널의 상태 로직은 Tk · rclpy 에 의존하지 않는 순수 python 모듈이라 화면 없이 테스트된다 (`integrated_bringup/integrated_bringup/demo_gui/{ball_launch,hand_step,catching}.py`). 이 절은 무엇을 어떻게 보여 주는가의 계약이다.

**GUI.**

- **공 패널** (`ball_launch.py`). 발사 조건 `(p0, v0, ω)` 의 파싱은 srv `/sim/launch_ball_at` 이 거부하는 것과 **같은 집합** 을 wire 앞에서 거부하고 틀린 필드를 이름으로 가리킨다 — 패널이 srv 가 거부하는 것을 받아들이면 일어나지 않은 발사를 보고하게 된다. truth 와 예측 피드는 **never / live / stale 세 상태** 로 구분한다 (never 와 stale 은 화면에서 같아 보이지만 뜻이 반대다 — 아무것도 돌지 않는다 대 멈췄다). ball_perception 이 없어도 "never received" 로 동작한다.
- **손 step 패널** (`hand_step.py`). 손 자세는 컨트롤러가 미러한 읽기 전용 파라미터 (`hand.q_pre` · `q_close` · `caging_mask` · `eta_close`) 에서 읽고 파일에 박지 않는다 (화면의 자세와 실행의 자세가 갈리지 않게). ρ 는 `rtc_tools.analysis.hand_close.rho` 를 import 해서 쓴다 — 화면 값과 오프라인 보고서 값이 갈리지 않게 사본을 두지 않는다.
- **포구 패널** (`catching.py`). 모드 · 사유, 입력 lane, plan, 추종 오차 · CLIK 상태, 슈퍼바이저 결과, 손 위상, 접촉 센서와 freshness 를 보인다. 세 가지를 합치지 않는다. ① **관측된 무장** 을 머리에 두고 요청된 무장 (`catching.enable` 파라미터) 을 옆에 둔다 — tick 이 E-STOP · fault 에서 latch 를 스스로 내리므로 파라미터 set 의 성공은 무장의 증거가 아니다. ② **거부된 입력 lane 과 조용한 lane** 은 거부 카운터로만 구별되므로 0 이 아닌 카운터는 항상 보인다. ③ 컨트롤러가 그 tick 에 계산하지 않은 블록 (`*_valid` false, PROC-7 이 0 으로 지운다) 은 0 이 아니라 `--` 로 보인다. 팔 기준을 만드는 planner (`closed_form` | `mpc`) 는 `CatchingState` 가 동결이라 읽기 전용 파라미터 `planner.segment.mode` 로 읽고 `segment mode: <값>` 줄로 보인다 (configure 가 끝나기 전이거나 park 된 컨트롤러는 빈 문자열을 답한다). 탐색 (`planner.search.mode`) 도 같은 방식의 `search mode: <값>` 줄이다. 고른 구현에 예산 파라미터가 있으면 (`mpc` · `mpc_docking` 의 first · replan, `nlp` 의 wake · 풀이 예산과 풀이 수 상한) 한 번 읽어 그 줄에 ms 로 붙이고, sim 전용 구현 (`nlp` · `mpc_docking`) 은 그렇게 적는다. plan 을 첫 구간과 함께만 게시하는 planner (`mpc` · `mpc_docking`) 에서는 구간이 보류된 plan 이 게시되지 않아 화면에 아무 사유도 남지 않으므로, plan 이 없을 때 `planner_events.csv` 의 `segment_outcome` · `segment_core_reason` 을 가리키는 한 줄을 붙인다 — wake 마다의 계획기 상태는 GUI 에 싣지 않는다 (메시지가 동결이다). 연속 투척의 진행 카운트는 결과의 엣지를 GUI 가 센다 (새 필드 없음).
- **헤더 공용 행.** "Clear E-STOP" (사유 조회 → 확인 뒤 해제의 2 단계) 과 "Reset fault" 는 포구 패널이 아니라 공용 위치에 있다 (절차는 L7 §4.1). 해제 요청이 latch 를 내렸으나 검증되지 않았다는 응답은 거부와 구별해 보인다.
- **`g1_p1b` profile.** `demo_controller_gui --robot g1_p1b` 가 있다. 위 패널은 profile 과 무관하게 만들어지지만 그 profile 에는 포구 컨트롤러가 없어 포구 패널은 받은 것이 없는 상태로 남는다. 그 profile 의 Dual Arm 탭과 `plot_rtc_log` 의 `dualarm_diag` 는 `demo_dualarm_controller` 의 것이고 이 절의 계약 밖이다 — `integrated_bringup/README.md` §demo_controller_gui, `rtc_tools/README.md`. 그 컨트롤러는 상태 토픽이 없어 GUI 가 세션 CSV 의 꼬리를 읽는다; G1 포구 컨트롤러의 상태를 무엇으로 보일지는 아직 정하지 않았다 ([#645](https://github.com/hyujun/rtc-framework/issues/645)).

**plot.** `plot_rtc_log` 는 CSV 의 종류를 파일명과 컬럼으로 판별한다 (`catching_diag` · `planner_events`); 스레드 timing CSV 는 공통 스키마라 기존 timing plotter 를 쓴다.

- `catching_diag` figure 는 한 시간축 위에 기준 · 실현 위치 · 추종 오차 · CLIK solve · 모드를 쌓는다 — "손이 어디로 가라고 지시받았고, 어디로 갔고, 슈퍼바이저가 왜 그렇게 판단했나" 가 같은 축에서 비교돼야 한다. 모든 패널에 모드 전이선 (tick 별 `mode` 열에서 유도) 을, 모든 패널에 `estop_active` (CM 의 global latch) 와 `fault_latched` (컨트롤러 latch) 구간을 서로 다른 색의 음영으로 얹는다. 두 latch 는 해제 수단이 다르므로 한쪽만 끝나는 구간이 읽혀야 한다.
- **무효 tick 은 NaN 으로 끊는다.** PROC-7 이 계산하지 않은 tick 을 0 으로 기록하므로 그대로 그리면 손이 매 tick 원점으로 간 것처럼 읽힌다. 평균 · 통계도 `*_valid` 로 거른 행만 쓴다.
- `catching_hand` figure 는 손 시퀀서 (위상 · ρ · 타임아웃) 와 지문 lane (‖F − b‖ · debounce 된 접촉 · freshness) 을 그 시행의 판정과 함께 쌓는다.
- `planner_events` 는 계획기가 깨어난 회마다의 탐색 기록 (후보 funnel · 거부 사유 · rank-gate 비트마스크 · 교체 결정) 을 그린다. 같은 디렉토리의 `nlp_candidates.csv` (#798) 는 `nlp` 탐색이 wake 마다 푼 후보 하나하나의 행 — verdict · 위반한 행 군 (전부) · 최대 위반 · SQP 반복 · QP 수 · 풀이 시간 · 포구 노드의 raw σ · Φ — 이고 `wake_ns` 로 `planner_events` 의 행과 잇는다 (열의 뜻은 `planner_events_csv.hpp` 머리). plot 은 아직 읽지 않는다. 구간 계획기 · `nlp` 탐색이 돈 세션에는 그 열이 있는 만큼 패널이 더 붙는다 (구간 풀이의 결과와 시간, docking 의 행 그룹별 위반과 포구 노드, `nlp` 의 후보 수 · wake 사유 · 고른 후보의 비용). **코어가 기한에 끊은 풀이는 끝난 풀이와 다른 표식으로 그리고 통계의 분포에 넣지 않는다** — 그 시간은 끊긴 시각이다 (L3 §8).
- 파일에 공 truth 열은 없다 (컨트롤러가 갖지 않는다) — 판정이 맞았는가는 이 plot 이 답하지 않고 시행 분석기 (§4.4) 의 일이다.

## 12. 탐색의 판정 지도 도구

§4.6 의 지도는 포구 자세와 게이트를 후보 하나씩 판정한다. 이 도구는 **탐색 자체** — `CatchSearch::Plan` (L3 §4.1), 설정이 고른 구현 — 가 투척을 받는가를 본다. 판정은 런타임 함수이고 python 은 입력과 집계만 맡는다 (§4.6 과 같은 분담).

- **한 wake 가 받는 것.** 탐색이 따르는 plan 없는 팔을 처음 보는 wake 다. 팔은 `planner.wait_pose` 에 정지해 있고 RT 는 plan 도 구간도 보고하지 않는다. 공분산은 0 행렬이다 (유효 · 예측과 짝 — 공분산을 읽는 탐색은 평균만으로 판정한다). 시계는 멈춰 있다: 탐색이 설정된 예산에서 계산하는 것 (한 wake 의 풀이 수, 첫 노드의 자리) 은 그대로이고, 도는 시계만 만드는 거부 (예산에 끊긴 탐색, 기한을 넘긴 풀이) 는 나타나지 않는다
- **한 투척.** reset 된 탐색에서 시작해 wake 를 순서대로 부르고, plan 을 낸 첫 wake 에서 멈춘다. plan 을 따르는 동안의 탐색 (교체 · follow window) 은 구간 계획기와 RT 가 있어야 하므로 재현하지 않는다. 투척의 판정은 그 투척의 wake 만으로 정해진다 — 앞에 어떤 투척이 돌았는가와 무관하다
- **예측 snapshot.** 비행 모델 (§4.6 의 항력 법칙) 의 위치 · 속도 · 가속도를 추정기 profile 의 격자로 자른 것이다. wake 는 발사 뒤 검출 지연부터 비전 주기마다 하나이고, 둘 다 도구의 인자다
- **탐색이 받는 로봇 값.** `planner.*` 의 키는 합성된 `catching:` 트리에서 런타임 파서가 읽는다. 컨트롤러가 configure 에서 묶는 값 (장치 정격, 여유가 적용된 관절 · 토크 한계, $\eta_v$ 가 적용된 속도 한계, 손의 폐쇄 시간과 선행) 은 트리의 키가 아니어서 python 이 설정에서 계산해 binding 파일로 넘긴다. 이 사본이 런타임과 어긋나면 지도가 조용히 틀린다 — 같은 투척의 sim 판정과의 일치율로 확인한다
- **수용.** 투척의 wake 가운데 plan 을 낸 것이 하나라도 있으면 수용이다. sim 에서는 발사부터 포구 시각까지의 wake 가운데 탐색이 plan 을 낸 것이 있으면 수용이고, plan 의 게시는 그 다음 층이다 (L3 §5.3) — 둘을 따로 센다. 거부한 투척의 대표 사유는 가장 잦은 wake 사유다 (동률이면 나중 것) `grid` 에서는 순위 게이트 (L3 §4 의 D-27) 에 걸린 후보도 plan 이 되므로 수용은 그 게이트의 통과를 뜻하지 않는다 — plan 의 `rank_mask` 가 wake 의 행에 남는다
- **축의 값.** 발사 위치 · 거리는 투척 설계의 값이다. 비행시간과 종단 속도는 비행 모델의 궤적이 대기 자세의 포구점에 가장 가까워지는 시각과 그때의 속력이다 — 탐색이 고른 $t_c$ 가 아니다 (거부한 투척에도 값이 있다)
- **도구.** `ros2 run rtc_tools catch_search_map` (python — 투척 설계 · snapshot · binding · 집계) 이 `ros2 run rtc_controllers catch_search_batch` (C++ — 판정) 를 부른다. 같은 투척 목록을 sim 이 던진다 (`catching_sim_trials --throws-file`)
- **통과점 투척 설계.** `ros2 run rtc_tools catch_throw_design design` 은 발사점 · 통과점 · 발사각의 격자에서 투척을 만든다 (L3 부록 C — 속력은 풀어서 얻는다). 투척의 `throw_id` 는 여섯 축의 곱에서의 순번이고, 격자의 부분집합과 그 사이를 메우는 세분화 (`select`) 가 같은 id 를 쓴다. 통과점의 격자는 투척을 만드는 데만 쓰이고 탐색에는 전달되지 않는다 — 탐색은 wake 만 본다. 투척마다 속력 · 정점과 통과면 사이의 여유 · 포구점 최근접의 시각 · 속력 · 하강각 · 손의 접근축과의 각 · 도달 구 안의 마지막 시각을 적는다
- **사전 판정.** 설계는 어느 탐색도 받을 수 없는 투척에 `screen:*` 이름을 붙인다 (L3 부록 C). 구와 lead 하한은 판정기 (`catch_search_batch --print-reach-bound`) 와 합성 트리에서 출처와 함께 읽는다. 이름이 붙은 투척은 지도에서 그 이름으로 거부되고 탐색은 그것을 보지 않는다; `catch_search_map --judge-screened` 는 그 투척에도 탐색을 돌려 조건을 탐색 자체로 확인한다 (받는 것이 있으면 요약의 `screened.accepted` 에 나온다). 이름은 그것을 낸 구 · lead 하한 · 검출 지연에 대해서만 필요조건이므로, 목록은 그 값을 함께 싣고 지도는 자기 설정이 그보다 느슨하면 (구가 크거나 자리가 다르다 · lead 하한이 작다 · 첫 wake 가 이르다) 그 목록을 탐색 없이 거부하는 실행을 받지 않는다. 통과점이 대기 자세의 관절 원점을 이은 선에 가까운 투척은 flag 만 단다 — 로봇 몸에 맞는지는 sim 의 접촉 기록이 말한다
- **나눠 돌리기.** 지도는 투척을 shard 로 잘라 여러 프로세스에서 돌린다 (`--jobs`). 투척의 판정은 그 투척만으로 정해지므로 자르는 방식은 판정을 바꾸지 않는다. wake 마다 `Plan` 호출의 벽시계 시간이 `wall_us` 로 남는다 — 탐색의 시계 (멈춰 있다) 가 아니라 기계의 시간이라 실행마다 다르다
- **지도가 무엇의 판정인가.** 요약은 합성 트리의 sha256, 합성에 쓴 파일, checkout 의 commit, 판정기 바이너리의 sha256, 도달 구, 추정기 profile 의 예측 지평을 적는다. 설정이 바뀌면 같은 투척의 판정이 바뀐다 — 지도는 그 hash 의 것이다
- **출하 투척 세트 (#798).** 로봇마다 `integrated_bringup/config/<robot>/throw_sets/` 에 세 목록이 있다 — D-1 회귀 (130 발) · D-2 튜닝 (p1b 30 · leap 27 발) · 후보 상자 (LHS 180 발; p1b R 2.5 · leap R 1.5 — #800 의 결정) — 투척마다 두 탐색의 오프라인 라벨 (`offline_<search>_*`) 을 싣고, 로봇의 세 세트는 투척을 공유하지 않는다. sim unit 은 그 파일을 던지고 (`run_unit.sh` 의 `THROWS_FILE`, 앞 N 발은 `THROWS_LIMIT`), 드라이버에 자기 투척 계열은 없다 (옛 배치의 `s35b` · `reference` 는 지웠다). 두 unit 의 시행은 **(목록의 sha256, `throw_id`)** 로 짝짓는다 (`rtc_tools.analysis.catching_throw_list.throw_key`; `catching_trials.csv` 의 `throws_file_sha256` 열, 요약의 `throws_file`) — 같은 launch 를 다시 쓴 파일은 다른 모집단이다. seed 는 추정기에만 간다 (sim 은 seed 를 받지 않는다)
- **sim 투척의 끝.** 시행 드라이버는 사이클이 닫히거나 (RETREAT 뒤 ARMED · IDLE · FAULT) 기록 창 (`--record-s`) 이 끝나면 투척을 끝낸다. plan 이 게시되지 않은 투척은 RETREAT 에 들어가지 않으므로 창을 끝까지 쓴다. `--end-on-ball-low` 는 **사이클이 열리지 않은** 투척 (APPROACH 이후의 mode 를 본 적 없음) 을 공의 ground truth 가 바닥 + 여유 아래로 내려간 뒤 유예 (sim 시간) 가 지나면 끝낸다. 판정은 ground truth 의 stamp 로 하고 벽시계는 창의 상한에만 쓴다. 열린 사이클은 닫히거나 창이 끝나야 끝난다. 투척마다 끝난 사유 (`end_reason`) 를 적는다
