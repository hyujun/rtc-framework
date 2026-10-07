# L3 — Planner: 포구 시각·포구점·접근축·γ 결정

이 문서는 현재 구현의 **계획기 층** 을 표현한다 — 포구 후보의 탐색 (포구 시각 $t_c$ · 포구점 $p_c$ · 접근축 $a_d$ · 포구 자세 $q^\ast$ · γ 프로파일) 과 그 결과의 게시다.

- **선택 키는 둘이다**: 탐색 `planner.search.mode` (`grid` 기본 \| `nlp`) 와 구간 `planner.segment.mode` (`closed_form` 기본 \| `mpc` \| `mpc_docking`). 조합은 다섯이고 (`nlp` × `closed_form` 은 park) 출하는 두 로봇 모두 `grid` × `mpc` 다. 이 문서가 적는 것은 `grid` 탐색 (`GridCatchSearch`) 과 그 뒤의 `closed_form` · `mpc` 이고, `nlp` 탐색과 `mpc_docking` 구간은 formulation ([ball_catching_inverse_dynamics_mpc.md](ball_catching_inverse_dynamics_mpc.md) §17.11 · §17.12) 이 갖는다. `closed_form` 과 구간 계획기가 있는 모드는 탐색 뒤에 구간 층이 붙느냐로 갈린다 (§4.1). 각 절의 머리에 적용 범위 — **공통 / closed_form 전용 / 구간 계획기 전용** — 를 적는다
- `mpc` 의 구간 계획 (APPROACH–정지 MPC) 의 수학은 [mpc_multiframe_clik_formulation.md](mpc_multiframe_clik_formulation.md) 가 갖는다. 이 문서는 탐색이 낸 후보가 거기서 어떻게 쓰이는지만 가리킨다 (§4.1)
- 탐색의 수식을 L4 · L7 의 `closed_form` 법칙과 한 문제의 순서로 이어 적은 통합본은 [grid_search_closed_form_formulation.md](grid_search_closed_form_formulation.md) 다. 결정 · YAML 키의 범위 · 게이트는 이 문서가 그대로 SSoT 다
- 코드: 탐색 코어 (순수 수치, ROS 의존 없음) 는 `rtc_controllers/{include,src}/…/catching/` (namespace `rtc::catching`), 계획기 스레드의 소유와 YAML 은 `integrated_bringup` 의 `controllers/catching/`. 한 wake (`PlannerCycle`) 는 탐색과 구간 계획기를 추상 interface `CatchSearch` · `SegmentPlanner` 로만 부른다 (§4.1)

---

## 1. 범위 / 비범위

범위: 새 예측 궤적 메시지마다 (계획기 스레드 — §5.3) 다음을 결정해 `PlanSnapshot` 으로 게시한다. 후보 시각은 **vision이 준 샘플 격자**에서 고른다 — 제어 PC가 궤적을 만들지 않는다(마스터 §5.2).
- 포구 시각 $t_c$, 포구점 $p_c$, 목표 접근축 $a_d$
- IK 해 $q^\ast$ 와 그 자세의 manipulability $w_5$·$w_6$ (catchability, D-18·C-3 — §4.2). $q^\ast$ 는 탐색의 도달시간 · 방향 속력 · 점수와, `mpc` 첫 풀이의 선형화 기준이 쓴다 — RT 의 CLIK posture 는 이 값을 읽지 않는다
- γ 프로파일 $(\gamma_f,T_w)$, 손 폐쇄 명령 시각 $t_{cmd}$
- 유효성과, plan 이 없을 때의 사유

`mpc` 에서는 유효한 plan 을 그 plan 의 첫 MPC 구간 (`SegmentSnapshot`) 과 **한 쌍으로** 게시한다 (§5.3).

비범위: 실행 (`closed_form`: L4/L5, `mpc`: MPC 구간의 추종 — formulation · L7), 모드 전환 (L7), MPC 구간 계획기의 수학 (formulation).

## 2. 코드 확인 게이트

**기존 kinematics 구현을 재사용한다**(마스터 §1.2) — FK·Jacobian 을 새로 만들지 않는다. 계획기가 기대는 기존 구조는 다음과 같다.

| ID | 확인 항목 | 지금의 구조 |
|---|---|---|
| G3-1 | 모델 로드 경로와 FK/Jacobian API (폐쇄 체인 손 포함 시 팔 부분만 쓰는 방법) | CM 이 공유하는 `PinocchioModelBuilder` 1개 + 계획기 스레드 전용 `RtModelHandle` 1개 (스레드별 1개, heap-free, LOCAL/LWA/WORLD). 계획기 모델은 `planner.sub_model` 이 이름으로 고르는 팔 sub-model 이다 — P1b 손바닥 frame 은 폐쇄 루프 상류라 팔 관절 열만 쓴다 (D-18) |
| G3-2 | RT → 계획기 상태 전달 경로(현재 $q_c,\dot q_c$, L4 기준 상태) | `rtc::SeqLock`. payload 는 trivially copyable POD (`std::array` 기반, Eigen 멤버 금지) — `PlannerRtState` (`planner_io.hpp`) |
| G3-3 | 계획기 스레드 생성·우선순위 규약 | MPC 스레드와 같은 방식 (`rtc::PeriodicRtThread` 형제 subclass), thread layout 의 `mpc` role 을 쓴다 (§5.3) |
| G3-4 | 손별 포켓 유효 깊이 $d_{eff}$, 포획 반경 $r_{cap}$ | `planner.search.grid.hand.d_eff` · `r_cap` (provisional, L6 §4.5). 투척 보정은 실기에서만 한다 (TBD-HAND-04) |
| G3-5 | 포구 허용 작업공간, 감속 여유 공간 | `planner.search.grid.workspace.catch_box` (provisional, TBD-BALL-02) |
| G3-6 | vision 샘플 간격·지평·$N$ → 후보 격자 범위 | 후보는 vision 격자 그대로다 — 간격 `prediction.dt_expected`, 지평은 `planner.search.grid.slice.t_max` 가 상한 (TBD-VIS-04) |
| G3-7 | **독립 IK/포즈 해석기가 있는지**와 그 API | 독립 IK 는 없다. 포구 자세 IK 는 `CatchPoseIk` (§4.2) 가 하고, `rtc::compliance::DifferentialIk` (σ_min 적응 λ, heap-free) 는 그 안에서 영공간 투영 $N$ 을 만드는 데만 쓴다 |

## 3. 참고자료

[R1] 포구 제약과 관절 램프(도달시간 제약의 출처), [R2] 시간 슬라이스 탐색과 예측 중단 시점, [R3]/[R4] softness, [R8] 불확실성, [R9] kinematics, [R15] 부록 SQP.

## 4. 수학적 이론

### 4.1 결정 구조

**적용: 공통** (탐색). 두 planner 가 갈리는 곳은 이 절 끝의 "두 planner" 다.

후보 시각은 vision 샘플 격자에서 고른다. **시간 규약은 L0 §4.5 (D-2) 가 SSoT 다.** 각 샘플의 시각 $t_k$ 는 공의 **물리 시각** `BallTime` (절대 steady ns) 이다 — nrt 수신 시 $t_{ref,steady}=\text{recv}_{steady}-(\text{recv}_{wall}-\text{stamp})$ 로 한 번 변환해 두므로, 계획기는 메시지 나이를 따로 빼지 않고 매 판정에서 정해진 '지금'과 직접 비교한다.

- 도달시간(§4.3)·commit(§4.11): 실제 시각 $now$ (`NowReal`, steady 실측)
- rollout(§4.8)·γ 프로파일·궤적 샘플링: 선행 시각 $now_{lead}=now+T_{arm}$ (`NowLead`)

상대시각은 수치 코어 경계에서만 만든다. 비교는 타입별 오버로드로만 한다 (`time_types.hpp`) — 원점이 다른 상대시각끼리 비교하지 않기 위해서다.

후보 하나에 계산을 적용하는 순서 (런타임 `GridCatchSearch::Plan`, 연산량이 싼 것부터):

$$\text{입력 · §4.9 작업공간}(p_c)\ \cdot\ \text{§4.4 불확실성}\to\text{§4.2 IK · manipulability (D-18)}\to\text{§4.3 도달시간}\to\text{§4.5 γ 창}\to\text{§4.8 rollout}\ (\gamma_f)\to\text{§4.9 정지점}(\gamma_f)\to\text{§4.6 오차 예산}(\gamma_f)$$

rollout 이 정지점과 오차 예산보다 **먼저** 다 — 둘 다 rollout 이 고른 $\gamma_f$ 로 계산한다 (rollout 을 돌릴 수 없으면 $\gamma_f=0$). 오프라인 지도 (`JudgeGates`) 는 포구 자세가 이미 있는 후보에 도달시간 · γ 창 · 정지점 (γ 창의 두 끝에서) 만 계산하고, rollout 과 `catch_box` 는 보지 않는다.

**구현하지 않은 항 — 충격량 게이트.** 설계의 순서는 끝에 충격량 게이트 (L7 §4.7) 를 둔다. 임계가 정해지지 않아 (TBD-IMP-01) 탐색에 이 게이트는 없고, `PlanReason::kImpulse` 는 쓰이지 않는다. 충격량 $\Delta p=m(1-\gamma_f)\Vert v\Vert$ 은 `PlanSnapshot::dp_impact` 로 **기록만** 한다.

판정 게이트에서 탈락한 후보는 뒤 단계를 계산하지 않고 탈락 사유를 기록한다 (아래 "런타임 판정/순위 분리"). 통과한 후보 중 §4.10 규칙으로 하나를 고르고, `closed_form` 에서는 **§4.7 히스테리시스**로 현재 plan과 비교한다. 후보가 하나도 남지 않으면 plan 없음(포기)이며, 사유 코드를 함께 기록한다.

**런타임 판정/순위 분리 (D-27).** 오프라인 지도는 자기가 계산하는 게이트를 **엄격한** 필터로 쓴다. 런타임 계획기 (`GridCatchSearch`) 는 둘로 나눈다. **판정 게이트** — 입력 유한성 (NUM-7) · IK 수렴 · manipulability (D-18, IK 안의 게이트) · `planner.search.grid.workspace.catch_box` 안의 $p_c$ 와 $p_{stop}$ — 는 후보를 **제거**한다 (팔을 거기 둘 수 있는가, 어디서 멈추는가의 문제). **순위 게이트** — 불확실성 (§4.4) · 도달시간 (§4.3) · γ 창 (§4.5) · commit 선행 (§4.11) · 오차 예산 (§4.6) · rollout (§4.8) — 는 제거하지 않고 실패마다 `planner.search.grid.score.penalty` 를 점수에 더한다 (비트마스크 `RankGateBit`, `grid_catch_search.hpp`). 판정 통과 후보가 0 일 때만 plan 없음이고 `PlanSnapshot::reason` = 가장 많이 걸린 판정 게이트, 선택 후보의 순위 게이트 실패는 계획기 CSV 의 비트마스크로 남는다. **IK 예산 (R-2)**: 싼 항 (입력·작업공간·불확실성·늦음) 으로 전 후보의 사전 점수를 먼저 매기고 IK 는 상위 `planner.search.grid.max_ik` 개에만, `budget_s` 가 남는 동안 돈다 (다음 후보의 비용을 최근 값으로 추정해 IK 전에 확인한다. 첫 후보는 항상 돈다). 도달시간·γ 창·정지점은 지도와 **같은 함수** (`JudgeRankGates`, `rank_gates.hpp`) 이고, 출발 상태만 다르다 (지도: 대기 자세 정지 / 런타임: 현재 명령 상태 — 구간을 따르는 중이면 그 구간 위의 상태, §4.3).

런타임의 한 사이클은 이 순서로 돈다: 창 안의 전 후보에 싼 항 → 사전 점수 상위 후보마다 IK (+ manipulability) → $\dot q^u$ 와 `JudgeRankGates` (도달시간 · γ 창) → rollout 이 $(\gamma_f,T_w)$ 를 고름 → 그 $\gamma_f$ 로 $p_{stop}$ 판정 → 오차 예산 → 점수.

**탐색 방식 `[확정 A-4]`.** [R1]은 $(q_c,t_c)$를 동시에 푸는 NLP를 썼다. 본 구현은 1차원 시간 탐색 + IK 다.

- 계획기 코어는 "입력 스냅샷(궤적 + 공분산 + 로봇 상태) → `PlanSnapshot`" **단일 진입 함수**다 (`CatchSearch::Plan` — 구현은 `GridCatchSearch::Plan`. `PlannerCycle::PlanOnce` 가 부른다). 스레드(§5.3), 입출력 SeqLock, RT 쪽 소비(L4·L7)는 탐색 전략과 독립이다
- 탐색 구현은 둘이다 — 이 문서가 적는 `GridCatchSearch` 와, 후보마다 팔 궤적 문제를 풀어 그 비용으로 고르는 `NlpCatchSearch` (`nlp_catch_search.hpp`; 식은 [ball_catching_inverse_dynamics_mpc.md](ball_catching_inverse_dynamics_mpc.md) §17.11 · §17.12). 뒤의 것에는 기본이 꺼진 스위치가 둘 있다 — 후보의 격자 셀 안에서 포구 시각까지 푸는 `continuous_tc`, RT 가 plan 을 채택한 뒤 후보를 처음 따른 plan 의 셀 둘레로 묶는 `follow_window`. `PlannerCycle` 은 탐색을 추상 interface `CatchSearch` (`catch_search.hpp` — `Plan` · `Monitor` · `NotePublished` · `ResetTrial` · `Solution`) 의 포인터로 소유하고, wake 는 interface 만 부른다. `PlannerCycle` 은 두 구현의 헤더를 include 하지 않고, 구체 타입을 아는 것은 configure 경로뿐이다 — `planner.search.mode` 가 `grid` 면 `MakeGridCatchSearch` 가 만든 `GridCatchSearch` 를, `nlp` 면 컨트롤러의 `SetupNlpCatchSearch` 가 만든 `NlpCatchSearch` 를 꽂는다
- **탐색과 구간 계획기가 주고받는 것 둘.** 둘 다 `PlannerCycle` 을 거치고 어느 interface 도 상대의 구현을 모른다. (1) 구간 계획기 → 탐색: RT 가 보고한 대기 · 추종 구간의 **복사** (`SegmentPlanner::Reported` → `ReportedSegments` → `CatchSearch::Plan`). 후보를 팔 운동으로 판정하는 탐색은 그 운동을 팔이 있을 구간 위에서 출발시킨다. "노드 0 이 $t_{eff}$ 인 풀이는 어느 구간에서 출발하나" 의 규칙은 한 곳 (`SourceSegmentAt`, `planner_io.hpp`) 에 있고 구간 계획기의 재계획과 교체 쌍의 첫 구간 (§5.3) 도 그것을 쓴다. (2) 탐색 → 구간 계획기: 직전 `Plan` 이 고른 후보의 팔 궤적 (`CatchSearch::Solution` → `CatchSolution` → `SegmentPlanner::PlanFirst`). 같은 문제를 다시 풀 계획기는 그것을 게시할 수 있다. `GridCatchSearch` 는 구간을 도달시간 게이트의 출발 상태로만 읽고 (§4.3) 해를 내지 않으며 (null), `MpcSegmentPlanner` 는 해를 읽지 않는다 (`MpcDockingSegmentPlanner` 는 `NlpCatchSearch` 의 해를 읽는다 — 그 해가 RT 가 보고한 구간에서 출발했을 때만 그대로 게시한다). RT 가 plan 을 따르는 동안에도 탐색은 `planner.freeze.t_stop_plan` 전까지 돌므로 (§5.3) 그 wake 에 넘어가는 구간은 따르는 중이면 차 있다 — 두 탐색 모두 읽는다

**구간 (`closed_form` / `mpc` / `mpc_docking`).** 구간은 `planner.segment.mode` 가 고르는데, 코드에서는 모드마다 클래스가 있는 것이 아니라 `PlannerCycle` 에 **구간 계획기가 꽂혔는가** (`SegmentActive()` — segment box 가 묶여 있고 추상 interface `SegmentPlanner` 의 구현이 꽂혀 있다) 로 갈린다. `SegmentPlanner` (`segment_planner.hpp`) 의 구현은 `MpcSegmentPlanner` (`mpc`) 와 `MpcDockingSegmentPlanner` (`mpc_docking`, formulation §17.12) 이고, 아래의 `MpcSegmentPlanner::…` 호출은 모두 그 interface 를 거친다. 아래 `mpc` 의 서술은 둘이 같은 곳이 많다 — 다른 곳은 `mpc_docking` 이라 적는다.

- **꽂힌 것이 구성된 것이다.** 팩토리 (`MakeGridCatchSearch` · `MakeMpcSegmentPlanner` · `MakeMpcDockingSegmentPlanner`) 는 새 객체를 만들어 구성에 성공했을 때만 돌려주고 (실패는 null), 컨트롤러가 그것을 `InstallSearch` · `InstallSegmentPlanner` 로 꽂는다 (configure 에서만 — 계획기 스레드가 멈춰 있을 때). `ClearSearch` · `ClearSegmentPlanner` 은 비운다 — cycle 은 구성되지 않은 구현을 들고 있지 않다. 꽂을 때와 `SetClock` 마다 cycle 의 시계가 구현에 전달된다 (`CatchSearch::SetClock` — `NlpCatchSearch` 는 코어에 전달). 이 규칙은 `Configure` 가 전체 reset 이라는 데 기댄다 — 한 번 쓴 객체를 다시 구성한 것이 새 객체와 같은 답을 낸다 (`AReconfiguredSearchIsANewOne` · `AReconfiguredPlannerIsANewOne`)
- **공의 예측은 view 로 건넨다.** 구간 계획기는 $t_c$ 의 공 한 점이 아니라 그 wake 가 읽은 궤적 · 공분산과 둘의 짝 여부 (`BallPrediction`) 를 받는다. 거기서 무엇을 뽑는가 — `mpc` 는 $t_c$ 의 공 (`MakeMpcSegmentBallTarget`) — 는 구현의 일이다. cycle 이 정하는 것은 그 wake 에 **따르는 plan 의 공이 있는가** 뿐이다: box 의 궤적이 그 plan 의 track 이 아니면, 그리고 포구 뒤에는, 빈 view 를 넘긴다
- 스레드 · RT 계약 (계획기 스레드에서만 불린다, 할당 · 잠금 · 로그 · 예외 없음) 과 cycle 이 기록에서 되읽는 필드 (`SearchStats::publish`, 재계획의 `SegmentRecord::source_seq`) 는 두 interface 헤더의 머리 주석이 갖는다

- **`closed_form`**: 탐색이 낸 plan 을 그대로 게시한다. RT 는 그 plan 의 $p_c$ 와 γ 프로파일로 soft-catch DS (L4) 를 돌려 접근하고, $t_c$ 뒤에는 상수 감속의 가상 목표를 따른다 (L7 §4.3). APPROACH 동안에도 탐색이 계속 돌며 §4.7 의 교체 규칙으로 plan 을 바꾼다
- **`mpc`**: 탐색이 유효한 plan 을 내면 같은 wake 에서 그 plan 의 첫 구간을 풀어 (`MpcSegmentPlanner::PlanFirst`) 둘을 한 쌍으로 게시한다. 구간이 보류되면 plan 도 게시하지 않는다. RT 는 plan 을 첫 구간과 함께만 채택하고, APPROACH 부터 정지 끝까지 관절 노드 구간을 따른다 — **RT 는 soft-catch DS 를 돌리지 않는다**. RT 가 plan 을 따르는 동안에도 탐색은 `planner.freeze.t_stop_plan` 까지 계속 돈다 — 그 wake 는 탐색을 먼저 하고, 탐색이 **다른 plan** 을 고르면 그 plan 과 첫 구간을 **교체 쌍**으로 게시하고 (RT 는 그 구간의 node 0 에서 plan 과 구간을 함께 바꾼다 — L7 §4.3a), 아니면 따르는 plan 의 구간을 재계획한다 (`MpcSegmentPlanner::Replan`). 탐색이 돌지 않는 wake 는 재계획뿐이다 (§5.3)
- **탐색이 낸 후보가 `mpc` 에서 쓰이는 방식** (`mpc_docking` 은 plan 의 $t_c$ 를 공이 입구 평면을 지나는 순간으로 읽고 첫 풀이의 방식도 다르다 — formulation §17.12). plan 의 $t_c$ 는 구간 격자의 닻이다 (포구 전 $t_c-j\Delta_{pre}$, 정지 $t_c+k\Delta_s$). 첫 풀이는 plan 의 $p_c$ · $v_c$ · $a_d$ 를 포구 노드의 목표 (공 위치 · 속도 · 접근축) 로, $q^\ast$ 를 선형화 기준의 목표로 쓴다 — 첫 쌍에서는 대기 자세에서 $q^\ast$ 로 가는 관절별 최소 jerk 도달이고, 교체 쌍에서는 팔이 움직이고 있으므로 RT 가 보고한 구간 위의 $(q_0,\dot q_0,\ddot q_0)$ 에서 $(q^\ast,0,0)$ 로 가는 관절별 5 차 다항식이다 ([ball_catching_inverse_dynamics_mpc.md](ball_catching_inverse_dynamics_mpc.md) §17.12). 포구 전의 재계획은 plan 이 아니라 그 wake 의 최신 예측에서 $t_c$ 의 공을 다시 읽고, 포구 뒤의 재계획은 공을 읽지 않는다. plan 의 γ 프로파일 $(\gamma_f,T_w)$ 은 `mpc` 의 실행에 쓰이지 않는다 — 탐색의 순위와 게이트에만 쓰이고, MPC 의 속도 목표 비는 `planner.segment.mpc.catch.gamma_ref` 다. 식은 formulation 이 갖는다

### 4.2 5-DoF 포구 자세와 IK

**적용: 공통** (탐색의 IK — `CatchPoseIk`. 오프라인 catchability 지도도 같은 함수를 돈다).

목표: $p_c=\hat p(t_k)$, $a_d=-\hat v(t_k)/\Vert\hat v(t_k)\Vert$.

**입력 방어 (NUM-7).** $a_d$ 계산은 $\Vert\hat v(t_k)\Vert\ge v_{eps}$ 와 `std::isfinite`($\Vert\hat v(t_k)\Vert$) 를 **둘 다** 검사한다. 하나라도 실패하면 후보를 사유 코드와 함께 탈락시킨다 — `std::max` 류 clamp 로 값을 덮어 진행하지 않는다.

catch frame LOCAL $+z$가 손바닥 바깥 법선이다 `[확정 D-17]` — catch frame 은 모델 빌더가 YAML 선언으로 추가하는 frame 이고 (D-10), 부모 frame·offset·자세는 로봇 config 에서 연다 (스키마와 초기값은 L5 의 catch frame 절). $a^C=R_{WC}^\top a_d$ 로 두면 접근축 오차의 LOCAL 표현은 L4 §4.5의 회전벡터다.

$$e_a^C=R_{WC}^\top e_a=\theta\,\frac{\hat e_z\times a^C}{\Vert\hat e_z\times a^C\Vert},\qquad \theta=\mathrm{atan2}(\Vert\hat e_z\times a^C\Vert,\ a^C_z)$$

$e_a^C\perp\hat e_z$ 이므로 $z$ 성분이 0이고, $S=\begin{bmatrix}1&0&0\\0&1&0\end{bmatrix}$ 가 정보를 버리지 않는다.

**갱신식은 Gauss-Newton이 아니다.** 아래 $J$ 는 잔차 $r$ 의 야코비안 $\partial r/\partial q$ 가 **아니고**, 관절속도와 과제속도를 잇는 관계일 뿐이다.

$$J(q)=\begin{bmatrix}J_p\\S\,J^L_\omega\end{bmatrix},\qquad W=\mathrm{diag}(1,1,1,\rho,\rho),\qquad e=\begin{bmatrix}p_c-p_C\\ S\,e_a^C\end{bmatrix}$$

$$\dot q_{clik}=\arg\min_{\dot q}\ \tfrac12\Vert W(J\dot q-e)\Vert^2+\tfrac12\mu\Vert\dot q\Vert^2\quad\text{s.t.}\quad \max(q_{min}-q,\,-\Delta_{max})\le\dot q\le\min(q_{max}-q,\,\Delta_{max})$$

$$\dot q_n=\big(I-J^\dagger J\big)\dot q_{sec},\qquad \boxed{\ \dot q_d=\dot q_{clik}+\dot q_n\ }$$

**갱신량은 관절 속도의 합이지 하나의 $\Delta q$ 가 아니다.** CLIK 이 내는 것은 $\dot q_{clik}$ 이고, 2차 과제가 보태는 것은 **영공간 관절 속도** $\dot q_n$ 이다. 둘을 하나의 스텝으로 합쳐 적으면 $N$ 이 강제하는 우선순위가 표기에서 사라진다. 반복 1회는 $\dot q_d$ 를 $\Delta t=1$ 로 적분한다 — 오프라인 root-finding 반복이지 servo tick 이 아니라서 샘플 주기가 없고, `planner.search.grid.ik.dq_step_max` 는 반복당 $\Vert\dot q_d\Vert_\infty$ 상한이다.

- 위치 행: $J_p\dot q=v_p$ 에 대한 Newton 스텝. 잔차의 부호를 뒤집어 넣는다.
- 회전 행: $SJ^L_\omega\dot q=S\omega^L$ 이므로, $\omega^L=e_a^C$ 를 단위 시간 적용하면 $\exp([e_a]_\times)z=a_d$ 에 의해 **한 번에 정확히 정렬된다**(L4 §4.5). 즉 이 행은 1차 근사가 아니라 정확한 회전 갱신이고, 부호도 그래서 양수다.

두 블록은 단위가 다르다(m vs rad). $\rho$ [m/rad]는 그 스케일을 맞추는 특성길이로, 단일 $\lambda$ 아래 두 과제의 상대 가중을 결정한다. `planner.search.grid.ik.rho`로 둔다.

**$\rho$ 는 과제 가중이지 잔차 이득이 아니다.** $\rho$ 를 잔차에만 곱한 형태 $\Delta q=J^\top(JJ^\top+\lambda^2I)^{-1}[-(p_C-p_c);\ \rho Se_a^C]$ 는 차원이 맞지 않는다 — $J\Delta q$ 의 회전 행은 rad 인데 잔차의 회전 행은 m 이 되고, $\rho$ 는 단위 변환이 아니라 **스텝 이득**으로 작동해 회전 오차가 반복마다 $(1-\rho)$ 로만 줄어든다. 그러면 바로 위의 "한 번에 정확히 정렬된다" 도 성립하지 않는다. $J$ 와 $e$ 양쪽에 $W$ 를 곱해야 $\rho$ 가 [m/rad] 특성길이로 쓰이고, $\lambda^2=0$ 인 곳에서 $W$ 가 상쇄되어 1-스텝 정렬이 복원되며, 특이점 근처에서는 단일 $\lambda$ 가 m 블록과 rad 블록에 감쇠를 어떻게 나눌지를 $\rho$ 가 결정한다. 구현은 가중형이다 (`catch_pose_ik.hpp`).

**영공간 2차 과제 $\dot q_{sec}$.**

$$\dot q_{sec}=k_w\,\nabla\log w_5(q)+K_n\,(q_n-q),\qquad \dot q_n=(I-J^\dagger J)\,\dot q_{sec}$$

- $q_n$ 은 **seed 를 관절 한계로 clamp 한 것** (= wait_pose) 이다. clamp 가 정의의 일부인 이유는 첫 반복부터 $q$ 가 clamp 된 값이기 때문이다 — 한계 밖 seed 를 그대로 $q_n$ 으로 두면 어떤 반복도 도달할 수 없는 자세를 목표로 잡아 $K_n(q_n-q)$ 가 **영원히 감쇠하지 않고**, 매 반복 한계 쪽으로 밀면 clamp 가 되돌리는 정상 편향이 남는다 (수렴한 해가 한계에 앉아 있는 것처럼 보인다). `planner.search.grid.ik.k_null` 이 0 이면 이 항은 비활성이다.
- $K_n$ 은 `planner.search.grid.ik.eps_pos` 와 **함께** 골라야 한다. $N$ 이 $\dot q_n$ 을 과제에서 안 보이게 하는 것은 **1차까지**라, 크기 $K_n\Vert q_n-q\Vert$ 의 자세 스텝은 다음 과제 스텝이 되갚아야 할 2차 잔차를 남긴다. $K_n$ 을 키우면 반복이 수렴하지 않고 정상 오차에 눌러앉는다.
- $\nabla\log w_5$ 는 중심차분으로 구한다 (반복당 $2n_{arm}$ 회 Jacobian, 할당 0, 간격 `planner.search.grid.ik.fd_step`). $w_5$ 가 아니라 $\log w_5$ 를 올리는 이유는 특이점에 가까울수록 기울기가 커져 밀어내는 방향이 강해지고, 이득 $k_w$ 가 $w_5$ 의 혼합단위 스케일에 덜 의존하기 때문이다.
- 종료는 수락 조건 **∧** ($\Vert N\nabla\log w_5\Vert<$ `planner.search.grid.ik.manip_grad_tol` ∨ $N_{IK}$) 다. 투영된 기울기로 판정한다 — 과제가 상쇄하는 성분은 쓸 수 없으므로 $\Vert\nabla\log w_5\Vert$ 로는 영원히 수렴하지 않는다. 상승이 안 끝난 채 반복 상한에 걸려도 **수락은 유지**하고 `manip_converged=false` 로 기록한다 (G3-G 신호). **중심차분 탐침이 못 쓰게 나온 반복은 수렴이 아니다** — 그 반복의 $\Vert N\nabla\log w_5\Vert$ 가 0 인 것은 도달해서가 아니라 잰 것이 없어서이고, 이를 허용오차와 비교하면 특이점 근처(탐침이 깨지기 가장 쉬운 곳)에서 상승이 한 번도 안 돈 자세를 "수렴" 으로 보고해 G3-G 신호의 부호가 뒤집힌다. 그런 반복은 `manip_converged=false` 로 두고 `manip_grad_failures` 로 따로 센다.
- 매 반복 $\Vert\dot q_d\Vert_\infty\le$ `planner.search.grid.ik.dq_step_max` 로 **방향을 유지한 채 축소**한다 (성분별 clip 은 과제 방향과 영공간 방향을 함께 왜곡한다).

참 야코비안이 필요하면 L4 §4.5의 $J_a$ 를 쓴다($S[\hat e_z]_\times[a^C]_\times J_\omega^L$ 형태). 본 갱신식은 그것을 쓰지 않으므로 수렴률에 대한 Gauss-Newton 보장은 없다 — 수렴은 게이트 G3-G로 실측한다.

**과제 스텝은 제약 QP 다 (D-26).** 관절 한계·스텝 제한을 **사후 clamp 가 아니라 부등식 제약**으로 두기 위해 $\dot q_{clik}$ 은 위 QP 로 푼다 (ProxQP, `rtc_tsid::QPSolverWrapper`). $\mu$ 는 `planner.search.grid.ik.mu` 다 — $J^\top J$ 는 rank ≤ 5 라 어떤 팔에서도 특이하므로 $\mu>0$ 이 없으면 해가 유일하지 않다.

- **$\mu$ 는 절벽이 있는 손잡이다.** 너무 작으면 Hessian 이 거의 특이해져 대부분의 반복에서 QP 가 수렴하지 않고, 너무 크면 수락률이 떨어진다 — 값을 바꾸면 다시 잰다
- **제약은 $\dot q_{clik}$ 만 묶는다.** 실제로 움직이는 것은 $\dot q_d=\dot q_{clik}+\dot q_n$ 이고 $\dot q_n$ 은 QP 밖에서 계산되므로, $\Vert\dot q_d\Vert_\infty$ 축소와 관절 한계 clamp 는 **여전히 필요**하다. "한계를 제약으로" 는 과제 스텝을 고르는 방식이지 최종 적용값의 경계를 대체하지 않는다
- **후보마다 cold start 한다.** `QPSolverWrapper` 는 호출 간 warm start 를 유지하는데, 연속 호출이 **서로 다른 후보**이므로 그대로 두면 답이 탐색 순서에 의존해 지도와 런타임이 어긋난다. `Solve()` 마다 `ResetWarmStart()` 를 한 번 부른다 (enum 하나만 바꾸므로 할당 없음). 한 후보 **안의** 반복 사이 warm start 는 결정적이라 유지한다
- **QP 가 안 풀리면 fail closed 인데, 닫을 것이 있을 때만 거부다.** 비수렴 QP 는 과제 스텝을 모른다는 뜻이라 감쇠 pseudo-inverse 로 대체하지 않는다 (지도가 기록한 법칙과 다른 법칙으로 자세를 만들게 된다). 다만 **이미 허용오차를 만족한 반복이 있었다면** 그 $q^\ast$ 는 같은 법칙으로 얻은 유효한 자세이므로 그것을 반환하고 `qp_failures=1` 로 조기 종료만 기록한다 — 버리면 유효한 포구 자세가 거부로 바뀐다. 아직 수락된 반복이 없을 때만 후보를 거부한다 (`kQpFailed`). 이 구분은 영공간 상승이 있을 때만 도달 가능하다: `k_manip`=0 이면 루프가 수락 즉시 끝나 두 번째 QP 가 돌지 않는다
- **의존.** rtc_controllers → rtc_tsid 엣지가 있다 (순환 없음 — rtc_tsid 는 rtc_controllers 를 모른다)
- **`RtModelHandle` 은 device 관절 순서가 걸려 있으면 안 된다.** `SetJointOrder` 는 **입력만** 재배열하고 Jacobian 의 열 · 관절 한계 · $\dot q$ 는 Pinocchio 순서라서, 순열이 걸린 핸들에서는 모든 후보가 **유한하고 수렴하며 틀린다**. 그래서 `HasJointReorder()` 는 우회가 아니라 거부다 (`kJointOrderMismatch`) — 그런 핸들을 가진 호출자는 같은 모델로 재배열 없는 핸들을 하나 더 만든다 (근거: `catch_pose_ik.hpp` 머리 주석 7)

$N=I-J^\dagger J$ 는 **`DifferentialIk` 가 만든다** — 영공간 투영은 갱신식의 일부이지 과제 solver 의 일부가 아니다. 따라서 `planner.search.grid.ik.sigma0`·`lambda_max` 는 $N$ 만 파라미터화한다. $J$ 는 $m=5$ (위치 3행 LOCAL_WORLD_ALIGNED + 접근축 2행 LOCAL $x,y$) 다. $J$ 는 계획기 스레드 전용 `RtModelHandle` 에서 catch frame 의 **팔 관절 열**만 꺼낸다 (G3-1).

함수는 `rtc_controllers/include/rtc_controllers/catching/catch_pose_ik.hpp` 의 `rtc::catching::CatchPoseIk` 다. ROS 의존 없음, `Resize()` 뒤 할당 0·`noexcept`·무로깅 (G3-K 함수 부분), 호출 간 상태 없음. 입력 $p_c$·$\hat v$ 는 **모델 world 좌표**로 받는다 — base→world 변환은 호출자(오프라인 지도 / 런타임 계획기) 몫이다. `DifferentialIk::Compute` 의 `ok=false` 는 **비유한 J 만** 뜻하므로 (특이 자세는 `ok=true`, σ_min≈0 — #310), 랭크 결손 판정은 $w$ 계산의 LDLT 피벗에서 하고 사유 코드를 따로 둔다.

반복마다 관절 한계로 clamp하고, 반복 상한 $N_{IK}$와 허용오차로 종료한다. **seed 는 매 후보 대기 자세(wait_pose)다** `[확정 D-18]` — 6축 5행 과제는 roll 1 자유도와 IK 해 가지가 남아 해(따라서 manipulability)가 seed 에 따라 달라지므로, 오프라인 catchability 지도와 런타임이 **같은 함수·같은 seed·같은 YAML 키**를 써야 지도와 실제 판정이 어긋나지 않는다. 이웃 슬라이스의 해로 warm start 하지 않는다 — 해가 탐색 순서에 의존하게 된다. seed 는 `planner.wait_pose` 이고, `planner.wait_pose_source: current` 에서는 RT 가 채택한 자세가 그것을 덮는다 (§6).

**roll 은 manipulability 최대화로 고른다.** 위 영공간 항 $k_w\nabla\log w_5$ 가 그것이다. **seed 규정은 그대로다** — 상승은 seed 가 놓인 해 가지 안의 **국소 최대**일 뿐 전역 roll 탐색이 아니라서, 지도와 런타임의 동치는 여전히 같은 seed·같은 키에 의존한다. 최대화 대상은 게이트 정의(`planner.search.grid.catchability.definition`)와 무관하게 **항상 $w_5$** 다 `[확정 Q3a]` — 그래야 $q^\ast$ 가 정의에 의존하지 않아 같은 자세에서 잰 $w_5$ 와 $w_6$ 를 비교할 수 있다 (C-3). `arm_6row` 로 판정할 때는 **직접 최대화하지 않은 값으로 게이트한다**는 뜻이므로 지도 해석 시 유의한다. `planner.search.grid.ik.k_manip` = 0 이면 seed 가 roll 을 결정한다.

수락 조건: 위치 오차 < $\epsilon_p$, $\theta\le\alpha_{\max}$ ([R2]의 허용 콘과 같은 취지. $\theta$ 는 위 회전벡터의 크기라 $z^\top a_d\ge\cos\alpha_{\max}$ 와 동치이면서 큰 오차에서도 수치적으로 안정하다).

**manipulability 게이트 (catchability) `[확정 D-18]`.** IK 수락 **직후** 해 $q^\ast$ 에서

$$w_5(q^\ast)=\sqrt{\det\big(J_5J_5^\top\big)},\qquad J_5=\begin{bmatrix}J_p^{LWA}\\ S\,J^{L}_\omega\end{bmatrix}_{\text{팔 관절 열}}\in\mathbb R^{5\times n_{arm}}$$

를 재고, 게이트 정의(`planner.search.grid.catchability.definition`, 기본 `arm_5row`)의 값이 그 정의의 threshold (`planner.search.grid.catchability.manipulability_min.arm_5row` — 로봇별 값) 미만이면 후보를 사유 코드와 함께 탈락시킨다. $w_5$ 는 **가중하지 않은** $J_5$ 에서 잰다 ($W$ 는 스텝의 것이고 게이트의 것이 아니다). 검증용으로 $w_6=\sqrt{\det(J_6J_6^\top)}$ (팔 열 6×6, roll 포함) 도 함께 계산·기록한다 (C-3). 모든 후보가 탈락하면 plan 없음(포기)이다. 정의 세부:

- 손바닥 법선 둘레 roll 은 포구에 무관해 행에서 뺀다. 손 관절은 손바닥 frame 에 영향이 없다 (P1b 손바닥은 폐쇄 루프 상류)
- m 와 rad 가 섞인 값이라 threshold 는 **이 정의에 대한 값**이다. 정의를 바꾸면 다시 맞춘다
- 이 게이트는 도달시간·γ 창·정지거리 게이트에 **추가되는 AND 조건**이다. manipulability 만으로 시간 안 도달은 보장되지 않는다

**fail-closed 수치 규칙 (NUM-7, NUM-1).** $w_5$·$w_6$ 는 고정 크기 분해(사전 할당 LDLT 의 대각 곱 — 구현은 피벗의 log 합)로 계산한다 — 계획기 스레드가 FIFO 라 RT-1 이 걸리므로 동적 크기 `JacobiSVD<MatrixXd>` (할당 발생) 는 쓸 수 없다. 판정은 `det > 0` 이 아니라 분해 도중의 모든 중간값이 `isfinite` 이고 `w ≥ threshold` 인지로 한다 — 특이 근처에서 반올림으로 det 가 음수가 되거나 NaN 이 나오면 탈락이다. **기존 `ClikReferenceGenerator::Manipulability` (팔 6×6 damped) 는 게이트로 재사용하지 않는다** — roll 을 포함해 같은 값이 아닐 뿐 아니라, damped(μ² > 0) 라 특이 자세에서도 $w>0$ 을 내고 `det > 0.0` 검사가 NaN 을 0 으로 세탁한다. 진단 로그에는 둘 다 남긴다.

**w₅ · w₆ 게이트의 의미 (`[확정 D-18]`).** 이 판정은 기존 게이트 (IK 수렴, 도달시간, γ 창, 정지거리) 에 **추가되는 AND 조건**이다 — manipulability 만으로 시간 안에 도달할 수 있다는 보장은 없다. 그래서 지도도 kinematic 지도 (`catchability_map` 과 `catch_pose_ik_batch`) 와 전체 게이트 지도 (`catch_gate_map` 과 `catch_gate_batch`) 를 따로 낸다. 게이트 정의 · threshold 는 서로 바꿀 수 없다 — $w_5$ 는 병진 (m) 과 접근축 (rad) 이 섞이고 $w_6$ 는 roll 을 포함해 손목 특이점에서 0 으로 내려가는데 그 자세가 포구에는 멀쩡하므로, 정의는 `arm_5row` 를 기본으로 두고 `arm_6row` 는 기록 · 선택 용도다 (threshold 는 정의별로 따로 둔다). `planner.search.grid.ik.k_manip` = 0 이면 영공간 상승이 꺼져 대기 자세 seed 가 roll 을 정하는데, 그럴듯한 대기 자세에서도 $w_5$ 가 문턱 아래에 머물 수 있다 — 게이트가 통과 가능한 것은 상승 때문이므로 "문턱 + 상승 꺼짐" 조합은 출하 config 에 두지 않는다.

**frame 규약 (함정).**

- "arm base frame" 은 로봇 config `urdf.sub_models.<arm>.root_link` (`ur5e_p1b` 는 `base`, `iiwa7_leap` 은 `link_0`) 이고 컨트롤러 config 의 CLIK `base_frame` 이 이름으로 참조한다.
- `ur5e_p1b` 에서 URDF `base` 와 `base_link` 는 원점이 같고 z 둘레 180° 다르다. `base_link` 로 두면 +x 가 반대가 되어 공이 등 뒤에서 날아오는데도 그럴듯한 결과가 나오므로 조용히 틀린다.
- **변환이 둘이다.** `CatchPoseIk::Solve` 는 $p_c,\hat v$ 를 **모델 world** (Pinocchio universe = URDF 모델 root) 로 받는다. world ↔ arm base frame 변환 (`io.base_T_world`) 과 arm base frame ↔ 모델 root 변환 (모델에서 읽는다 — `ur5e_p1b` 는 모델 root 가 `base_link` 라 `base` = root·Rz(180°)) 은 별개이고, 지도 도구는 두 변환을 분리해 받는다 (`--arm-base-frame`, `--world-yaw-deg`, `--world-translation-m`). 도구 모듈에는 로봇 값을 박지 않는다 — 변환은 항상 인자이고 180° 는 테스트가 고정한다. 런타임의 같은 변환은 L1 §1 이 갖는다. 발사 높이는 world 기준, 거리는 arm base 기준이다.
- world ↔ base 대조는 body · link 원점이 아니라 **관절 축선** 의 FK (MuJoCo `xanchor`/`xaxis` 대 Pinocchio `oMi`) 로 한다 — 파일마다 body 관례가 달라 원점 비교는 파일 관례를 잰다. 단일 링크의 "implied transform 이 상수" 는 증거가 못 된다 (pan 축 둘레 회전 + z 이동은 q 와 무관하게 상수로 나온다).
- **`ur5e_p1b` 는 sim 모델 불일치 항을 분리해 센다.** sim MJCF 가 URDF 와 wrist 부터 치수가 다른 사본이라 catch frame 지점의 모델 불일치 ($\varepsilon_{model,p1b}$) 가 계통 편향으로 남는다. 이 항은 오차 예산 (§4.6) 과 catchability 지도에서 **분리된 계통 항** 으로 세고 (합쳐 평균내면 안 줄어드는 항이 줄어드는 것처럼 보인다), **sim 실측으로 검증하지 않는다** (sim 이 편향의 출처다 — 실기에서만 갈린다). `iiwa7_leap` 에는 이 항이 없으므로 두 로봇의 sim 포구 정확도 차이를 로봇 차이로 읽지 않는다. 새 로봇 프로파일이 추가되거나 손 MJCF 가 고쳐질 때 축선 대조를 다시 돌린다.

**오프라인 catchability 지도의 구성.** 판정은 런타임과 같은 `CatchPoseIk::Solve` 여야 하고 격자 · 비행 · 집계는 python 관행이라, C++ 배치 실행파일과 python 오케스트레이터로 잇는다 (python 재구현은 "같은 함수" 를 어긴다).

- **옵션 파서** `rtc::catching::ParseCatchPoseIkParams` 가 `planner.search.grid.ik.*` · `planner.search.grid.catchability.*` 를 `CatchPoseIkOptions` 로 바꾼다 — 런타임 계획기와 지도가 읽는 **같은 YAML 키** 의 실체다. `planner.search.grid.ik` 아래 미지 키는 거부한다. 활성 정의의 `manipulability_min` 이 TBD 면 비유한으로 남겨 `kOptionsInvalid` 로 fail-closed 한다 (다른 차원의 $w_5$ 문턱이 $w_6$ 게이트의 대체값이 될 수 없다).
- **배치 실행파일** `catch_pose_ik_batch` (kinematic) · `catch_gate_batch` (도달시간 · γ 창 · 정지점의 런타임 함수를 호출). ARCH-7 의 오프라인 검사 도구 예외다. 입력은 ModelConfig · sub-model · catch frame · 옵션 YAML · seed · 후보 CSV, 출력은 후보별 사유 · $q^\ast$ · $w_5$ · $w_6$ · 반복 수 · $\sigma_{min}$ · QP 진단 CSV. 테스트가 고정하는 계약은 둘이다: **열이 solver 의 double 을 bit-exact 로 싣는다** (G3-I 비교용), **후보 순서를 바꿔도 판정이 같다** (python 이 샤딩 · 재개한다).
- **python 순수 모듈** `rtc_tools.analysis.catchability_map` (격자, 항력 비행, base ↔ world 변환, provenance) 과 `catch_gate_map` (전체 게이트 지도). pybind 는 쓰지 않는다 (venv `FindPython` 함정, AGENTS.md §9.2).
- **catch frame 이 있는 sub-model 이 필요하다.** 출하 `urdf.sub_models.<arm>` 은 flange (`tool0`/`ee_link`) 에서 끝나 catch frame 이 거기에 없다 — 지도 (와 런타임 계획기, `planner.sub_model`) 는 arm root 에서 catch frame 부모 link 까지의 sub-model 을 따로 선언해 쓴다 (손 관절은 `buildReducedModel` 이 잠그고 손바닥은 루프 상류라 FK · Jacobian 은 정확하다). `LoadModelConfig` 스키마는 출하 robot config 와 다르다 (`urdf_path`, `sub_models` 가 sequence) — 번역은 python 쪽이 한다.
- **항력.** sim 항력은 무차원 $C_d$ preset 이고 스칼라 $k$ [1/m] 는 YAML 에 없다 (preset 에서 환산해 쓸 수 없다). 지도는 sim 의 힘 법칙 자체를 적분하고 ($\omega=0$ 이라 Magnus 는 소멸), $\rho$ · $C_d$ 는 기본값 없는 필수 인자로 받아 출처를 provenance 에 남긴다.
- **지도의 출력.** 격자 셀마다 (a) 포구 가능 여부, (b) 최대 $w$ 와 그 후보의 $t_c$ · $p_c$ · $q^\ast$, (c) 탈락 사유 (IK 실패 · $w$ 미달 · 비유한 입력 · 도달 불가 · γ 창 없음). 수락 투척 수는 로봇이 한 자세에서 기다리므로 **최선 seed 하나** 기준이고, 상자 수락률의 분모는 상자 안 전체 격자 투척이다. 발사 조건은 방위 · 높이 · 속력 · 앙각 · 수평 방향 (발사점 → 겨냥점 방향 + 편차) 으로 정한다. 포구 후보는 비행시간이 하한을 넘는 것만 남긴다 (짧은 직선 투척 제외, D-18 — 하한은 도구 인자). 거리는 vision 지평 요구와 가속 box 를 동시에 가장 어렵게 하는 인자라 도구의 인자다. threshold 를 사용자가 갱신하면 지도를 다시 돌린다.

### 4.3 관절 도달시간 제약 ([R1] 출처, 닫힌해는 `[논문 외 유도]`)

**적용: 공통** (순위 게이트 `kRankReach`).

[R1]은 관절 램프의 실현 가능성 $t\ge t_{\min,i}(q_i)$를 제약으로 썼다. 본 구현은 현재 명령 상태 $(q_{c,i},\dot q_{c,i})$에서 $(q^\ast_i,0)$까지의 최소시간을 닫힌해로 계산한다 (`TMinChecked`, `time_feasibility.hpp`). 속도 한계 $\bar\omega$ (계획 한계 $\eta_v\dot q_{\max}$ — §4.5), 가속 한계 $\bar a$, 목표 방향 속도 성분 $w=s\,\dot q_{c,i}$, $D=|q^\ast_i-q_{c,i}|$:

- $w<0$ (반대 방향 이동 중): 정지 후 $D+w^2/2\bar a$를 정지 상태에서 이동.
- $w^2/2\bar a>D$ (지나침): $w/\bar a$ 후 $w^2/2\bar a-D$ 복귀.
- 삼각: $\omega_p=\sqrt{\bar aD+w^2/2}\le\bar\omega$이면 $t=(2\omega_p-w)/\bar a$.
- 사다리꼴: $t=\dfrac{\bar\omega-w}{\bar a}+\dfrac{\bar\omega}{\bar a}+\dfrac{D-\frac{\bar\omega^2-w^2}{2\bar a}-\frac{\bar\omega^2}{2\bar a}}{\bar\omega}$
- 정지 상태 이동 $T_{rest}(D)$: $\sqrt{\bar aD}\le\bar\omega$이면 $2\sqrt{D/\bar a}$, 아니면 $D/\bar\omega+\bar\omega/\bar a$.

유도 요지: 가속 구간 이동거리 $(\omega_p^2-w^2)/2\bar a$와 감속 구간 $\omega_p^2/2\bar a$의 합이 $D$.

**출발 상태 $(q_c,\dot q_c)$ (런타임).** RT 가 보고한 명령이다 (명령이 seed 되기 전의 보고는 측정 자세와 영속도). RT 가 구간 계획기의 구간을 따르는 동안에는 보고 시각의 명령이 아니라 **$now_{lead}$ 에 팔이 있을 구간** — 대기 구간의 node 0 이 $now_{lead}$ 이전이면 그것, 아니면 추종 구간 (`SourceSegmentAt`) — 을 $now_{lead}$ 에서 평가한 $(q,\dot q)$ 다. 평가는 RT 가 구간에서 명령을 뽑는 것과 같은 함수다 (`NodeTrajectoryFollower::SampleJoints`). 보고된 구간이 없거나 그 시각에 읽을 수 없으면 (관절 수가 모델과 다름 · node 0 전 · 비유한 값) 보고된 명령을 쓴다. 바뀌는 것은 이 게이트의 출발뿐이다 — rollout 의 기준이 출발하는 곳 (§4.8) 과 교체 규칙 (§4.7) 은 그대로다.

**전제 $|w|\le\bar\omega$ 는 검사한다.** 초기 속도가 이미 속도 한계를 넘으면 최소시간 문제 자체가 정의되지 않는다 — 사다리꼴 분기의 $(\bar\omega-w)/\bar a$ 가 음수가 되어 물리적 의미가 없는 값이 조용히 나온다(예: $w_0=6$, $\bar\omega=\pi$, $\bar a=10$, $D=2$ → 0.92374 s, 그중 첫 구간이 $-0.2858$ s). 계획용 $\dot q_{\max}$(운용 여유율 적용값)와 CLIK 내부 한계가 다르거나, L5의 경계 충돌 규칙이 발동한 직후에 일어날 수 있다. `TMinChecked`가 clamp하고 `w0_clamped` 플래그를 세우며, 플래그가 서면 그 결과는 쓸 수 없다 (`Usable()` 거짓) — 도달시간 게이트 실패다 (지도: 탈락 / 런타임: 순위 실패).

**한계 값 $\bar a$ 의 출처 `[확정 D-16]`.** 관절 가속 한계는 토크 한계(`devices.<group>.joint_limits.max_torque` = URDF `effort` = MJCF `forcerange`)에서 오프라인 도구로 도출한 **보수적 상수 box** 이고, 키는 `robot.arm.qdd_max` (`catching/search_grid.yaml`) 다. 자세 의존 한계는 쓰지 않는다.

**이 box 는 탐색의 도달시간에 쓰는 값이고, 실행의 가속 제약과 같은 값이 아니다.** 출하 CLIK 의 가속 제약은 `joint_cmd.accel_constraint: dynamic` — 토크 행 ($M\dot v+h$ 를 $\pm\eta_\tau\tau_{\max}$ 안에) — 이고, CLIK 은 이 box 를 읽지 않는다. 같은 키를 QP 비의존 정지 램프 (ABORT) 와 homing 램프도 읽는다. 따라서 이 절의 도달시간은 실행이 실제로 내는 가속과 다른 한계로 잰 값이다 — 순위 게이트라 후보를 제거하지는 않는다. `ur5e_p1b` 의 sim profile 은 이 키를 실행 envelope 값으로 덮는다 (`config/ur5e_p1b/sim.yaml`).

**상수 box 의 도출 `[확정 D-16]`.** 가속 데이터는 없고 토크 한계는 있다. 오프라인 도구 `derive_accel_limits` (`rtc_tools/rtc_tools/analysis/derive_accel_limits.py`) 가 다음으로 box 를 만든다.

1. 포구 작업공간 · 대기 자세 주변에서 관절 자세 $q$ 와 속도 $\dot q$ 를 표본 추출한다. $\dot q$ 는 계획이 쓰는 속도 집합 (관절 속도 한계 이내) 으로 제한한다. 표본 범위의 기본값은 관절 한계 상자 전체 (가장 보수적) 이고 `--q-center` · `--q-halfwidth` 로 좁힌다.
2. 각 표본에서 Pinocchio 로 $M(q)$, 중력 $g(q)$, 속도항 $c(q,\dot q)$ 를 구한다. 모델은 손을 포함한 전체 URDF 이고, 손 관절은 고정 자세 · 속도 0 · 가속 0 으로 두어 손의 질량이 팔 행의 $M,g,c$ 에 들어가게 한다 (손 가속과의 결합 블록은 묶지 않고 최대 크기를 보고한다). 회전자 반사관성은 URDF 에 없으므로 MJCF `armature` 를 인자로 넣는다.
3. 중력과 속도항은 별도 헤드룸으로 뺀다: 팔 관절 $i$ 의 동적 토크 여유 $\tau_{dyn,i}=\eta_\tau\tau_{\max,i}-|g_i(q)|-|c_i(q,\dot q)|$ ($\tau_{\max}$ 는 `devices.<group>.joint_limits.max_torque` = URDF `effort` = MJCF `forcerange`). 가속 box $a=s\,w$ 가 그 표본에서 허용되는 충분조건은 모든 팔 행 $i$ 에서

$$\sum_j |M_{ij}(q)|\,w_j\,s\ \le\ \tau_{dyn,i}\ \Longrightarrow\ s(q,\dot q)=\min_i\frac{\tau_{dyn,i}}{\big(|M|\,w\big)_i}$$

 이다 (가속 부호의 최악 조합을 본 충분조건이라 box 표현에는 정확한 조건이고 조일 수 없다). 스칼라 $s$ 의 LP 라 닫힌해가 위와 같다.
4. 관절 가중 $w$ 의 기본은 균일 ($w_j=1$) 이다. 다른 가중 (`--weights tau_max` 등) 을 쓸 때는 도달시간 식이 쓰는 한계와 같은 벡터여야 한다.
5. 전 표본의 최소 $s^\ast=\min s(q,\dot q)$ 로 box $s^\ast w$ 를 채택하고, 표본 범위 · $\eta_\tau$ · $w$ · 모델 버전 · 일자 · $\tau_{dyn}\le0$ 인 표본 비율 · 관절별 binding 제약 (최소를 만든 행) 을 provenance 로 YAML 에 기록한다. $\eta_\tau<1$ 은 접촉 충격과 모델 오차를 위한 여유다 (값은 도구 인자와 YAML provenance).
6. **퇴화 시 채택하지 않는다.** $\tau_{dyn,i}\le0$ 인 표본이 있거나 $s^\ast$ 가 `--min-accel` 아래면 YAML 에 `adopted: false` 를 적고 도구가 rc 2 로 끝난다 — 표본 범위 축소, $\eta_\tau$ 조정, 자세 의존 한계 검토는 사용자 판단이다.

교차 검증은 두 가지다. `--check rnea` 는 새 표본에서 모든 부호 패턴 $\sigma$ 에 대해 $\tau=\mathrm{RNEA}(q,\dot q,\sigma a)$ 가 $|\tau_i|\le\eta_\tau\tau_{\max,i}$ 인지 (위 $|M|$ 경계와 독립인 경로), `--check mujoco` 는 `mj_inverse` 로 MJCF 의 armature · damping 항까지 포함해 actuator `forcerange` 와 비교한다 ("도출값 ≤ 달성 가능"). MJCF 가 모델 자기정합 밖의 사본인 로봇 (`ur5e_p1b`) 에서는 `armature` · `damping` · `frictionloss` 가 Pinocchio $M,h$ 에 없어 도출값을 낙관적으로 확인해 줄 수 있으므로 RNEA 자기정합으로 검증하고, `mujoco` 교차는 `model_pairs.yaml` 게이트 안의 로봇으로 한다. 이 box 는 최악 부호 충분조건과 자세와 무관한 단일 상수라는 두 원인으로 자세 · 방향 의존의 토크 한계보다 훨씬 보수적이다 — 실행의 가속 제약이 토크 행 (`dynamic`) 인 이유다 (L5 §4.3 의 가속 제약). 실기에서는 UR 컨트롤러가 자체 가속 · 보호 정지 기준을 가질 수 있어 (repo 밖) 실기 식별로 확인한다.

**잘못된 한계 입력은 flag 로 보고한다.** `TMinChecked` 는 $\bar a\le0$ 또는 $\bar\omega\le0$ (또는 NaN) 이면 `limits_invalid` 를 세우고 $t=+\infty$ 를 돌려준다 (비유한 $q$·$w$ 는 `input_invalid`) — 플래그를 보지 않는 호출자도 후보를 통과시키지 못한다. 가속 box 가 없는 구성에서는 도달시간을 판정할 수 없다 (게이트 실패). 한계 값 자체의 범위 검사는 파라미터 검증기가 한다.

검증: 닫힌해는 속도·가속 제약 선형계획(시간 이분 탐색) 해와의 대조로 고정돼 있다 (`test_catching_time_feasibility` 의 고정 테이블, G3-A).

제약:

$$t_k-now-T_{arm}-T_{margin}\ \ge\ \max_i t_{\min,i}$$

$t_k$ 는 `BallTime`, $now$ 는 `NowReal` (L0 §4.5). 팔 명령이 $T_{arm}$ 뒤에 실현되므로 $T_{arm}$ 을 뺀다 — 이는 $now_{lead}$ 와 비교하는 것과 같다 (`ReachTimeFeasible`).

**한계.** 이 조건은 **필요조건**이다. 실제 운동은 관절별 시간최적 프로파일이 아니다 — `closed_form` 에서는 과제 공간 DS 가, `mpc` 에서는 MPC 구간이 만든다. DS 에 대한 충분성은 §4.8 rollout에서 확인한다.

### 4.4 불확실성 게이트

**적용: 공통** (순위 게이트 `kRankUncertainty`).

$$\sigma_{\max}(t_k)=\sqrt{\lambda_{\max}\big(\Sigma_{pp}(t_k)\big)}\ \le\ \kappa_\sigma\,r_{cap}$$

**$\Sigma_{pp}$는 vision이 준 값이다.** 각 샘플의 6×6 공분산에서 위치 3×3 블록을 꺼내 대칭화한 뒤 최대 고윳값을 쓴다(`Eigen::SelfAdjointEigenSolver::computeDirect`, 고정 크기·무할당 — `GridCatchSearch::SigmaMax`). 제어 PC가 공분산을 전파하지 않는다. 공분산을 모르는 후보 (비유한 값, 또는 궤적과 token 이 다른 공분산) 는 이 게이트의 실패다.

`PointCloud2`에는 트랙 상태(`STATUS_INITIALIZING` 등)가 없으므로(마스터 §5.1), "초기화 직후 트랙 탈락"은 다음으로 대체한다.

- L1의 트랙 epoch가 막 바뀐 직후 `n_settle` 개 메시지는 계획하지 않는다(L1 §4.4).
- 그리고 위 $\sigma_{\max}$ 게이트 자체가 초기 불확실성이 큰 구간을 걸러낸다 — vision의 공분산이 정직하다면 이 편이 상태 플래그보다 낫다.

vision 공분산의 신뢰성은 시뮬레이션에서 참값 대비 NEES로 확인한다(L8 §4.4). 일관적이지 않으면 $\kappa_\sigma$ 로 보정하고 그 사실을 기록한다.

### 4.5 γ 창 `[논문 외 유도]`

**적용: 공통** (순위 게이트 `kRankGamma`). `closed_form` 에서는 여기서 정한 창 안의 $\gamma_f$ 가 실행되고, `mpc` 에서는 창이 **순위와 게이트에만** 쓰인다 — MPC 가 실행하는 속도 목표 비는 `planner.segment.mpc.catch.gamma_ref` 이고 이 창과 다른 양이다 (창 밖의 값일 수 있다).

**하한 (손 폐쇄).** 상대속도 $(1-\gamma)\Vert v\Vert$로 포켓 유효 깊이 $d_{eff}$를 지나기 전에 손이 닫혀야 한다.

$$\gamma\ge\gamma_{\min}=\mathrm{clamp}\!\left(1-\frac{d_{eff}}{\Vert v(t_k)\Vert\,T_{close,tot}},\ 0,\ 1\right),\qquad T_{close,tot}=T_{close,e2e}+T_{tick}$$

**`planner.search.grid.hand.d_eff` 는 포켓 깊이가 아니다** — 시각 발동 fly-in 으로 잰 허용 상대속도 × $T_{close,tot}$ 다 (L6 §4.5). 이 절은 $d_{eff}$ 를 $d_{eff}/T_{close,tot}$ (손이 흡수할 수 있는 상대속도) 로만 쓰므로 식은 그대로고, 포켓 깊이는 접촉 물리량으로 L6 에 있다 (TBD-HAND-04). 유효 조건은 런타임 손 발동도 시각 발동이라는 것이다.

**상한 (팔 속도).** 포구 자세 $q^\ast$에서 방향 $\hat v$로 낼 수 있는 최대 속력 $v_{dir,\max}$와 TCP 속도 한계 $v_{\max}$(= `planner.search.grid.reference.v_max`, L4 `reference.v_max` 의 탐색용 복사본 — §6)로 제한한다.

$$\gamma\le\gamma_{\max}=\mathrm{clamp}\!\left(\frac{\min(v_{dir,\max},\ \eta_vv_{\max})}{\Vert v\Vert},\ 0,\ 1\right)$$

TCP 속도 한계를 빠뜨리면 계획이 통과시킨 γ가 L4에서 속도 포화를 일으킨다. `ComputeGammaWindow`는 두 값을 모두 인자로 받는다.

**게이트.** 순위 게이트 `kRankGamma` 는 창이 비지 않고 공이 속력 여유 안에 있을 때 통과한다 (`JudgeRankGates`):

$$\gamma_{\min}\le\gamma_{\max}\quad\text{and}\quad\Vert v\Vert+m_\gamma\ \le\ \Vert v\Vert_{\max}=\min(v_{dir,\max},\ \eta_vv_{\max})+\frac{d_{eff}}{T_{close,tot}}$$

$m_\gamma$ 는 `planner.search.grid.gamma.margin` 이고 $\Vert v\Vert_{\max}$ 는 `MaxCatchableSpeed` 다. clamp 가 없는 두 식에서 $\gamma_{\min}\le\gamma_{\max}$ 는 $\Vert v\Vert\le\Vert v\Vert_{\max}$ 와 같은 조건이고, 둘째 조건은 거기에 여유 $m_\gamma$ 를 얹는다.

- **$\eta_v$ 는 관절 속도 한계에도 적용한다** — $v_{dir,\max}$ 를 $\eta_v\dot q_{\max}$ 로 계산한다 (`RankGateInputs::qdot_plan`). 구속 항이 $v_{dir,\max}$ 일 때 TCP 항에만 여유를 두면 아래 D-9 의 완충이 사라진다
- `reference.v_max` (탐색의 복사본도 같다) 는 실측값이 아니라 **도출값**이다 (L4 §6) — 그러면 위 $\min$ 의 TCP 항은 관절 정격이 허용하는 범위에서는 구속하지 않고 기준이 폭주할 때만 잡는다
- 구속하는 것은 대개 **포구 자세에서의 방향 속력**이고 그것은 자세마다 다르다 — $w_5$ 를 올린 자세가 $\hat v$ 로 빠른 자세는 아니다

**여유율 `[확정 D-9]`.** `ComputeGammaWindow` 의 TCP 속도 인자는 $v_{tcp}=\eta_v\cdot$`planner.search.grid.reference.v_max` ($0<\eta_v\le1$, `PlanningTcpSpeed`) 이다. γ 창이 $v_{\max}$ 전체를, rollout 수락(§4.8)이 $\eta_vv_{\max}$ 를 쓰면 창은 통과했는데 rollout 에서만 탈락하는 후보가 구조적으로 생기고, 계획이 한계 끝을 쓰면 실행 중 예측 변화로 L4 가 포화한다. 실행 중 γ 를 낮추는 경로가 없으므로 (D-8) 이 여유가 실행 중 유일한 완충이다.

**입력 방어.** $v_{dir,\max}$ 는 아래 DLS 정규화식의 결과라 수치 문제로 음수가 나올 수 있다. 음수면 clamp 가 $\gamma_{\max}$ 를 0으로 **올려** 판정을 뒤집으므로, `ComputeGammaWindow` 가 비물리적 입력을 검사해 플래그를 세운다(`TMinChecked` 가 $|w_0|>\bar\omega$ 를 검사하는 것과 같은 수준). 공 속력이 0 이하이면 창이 정의되지 않는다 (`input_invalid`).

**방향 속력 계산.** 접근축 각속도 0을 유지하며 $\hat v$ 방향 단위 속도를 내는 관절속도 $\dot q^u=J_5^\top(J_5J_5^\top+\lambda^2I)^{-1}[\hat v;0;0]$ 를 damped least-squares로 구하고 (`UnitSpeedSolver`, $\lambda$ = `planner.search.grid.gamma.unit_speed_damping`)

$$v_{dir,\max}\approx\frac{\max\big(0,\ \hat v^\top J_p\dot q^u\big)}{\displaystyle\max_i\frac{|\dot q^u_i|}{\dot q_{\max,i}}}$$

로 근사한다 (`DirectionalSpeedMax`). **분자가 필요한 이유:** DLS($\lambda>0$)는 $J\dot q^u=[\hat v;0]$ 을 정확히 만족하지 않는다. 특이 자세 근처에서 달성 속력이 1보다 작은데 분자를 1로 두면 **실제로 낼 수 없는 속력을 보고한다** — "보수적"의 반대다.

**분자는 노름이 아니라 $\hat v$ 방향 투영이다.** $\Vert J_p\dot q^u\Vert$ 는 $\hat v$ 와 다른 방향으로 새는 속도 성분까지 "달성 속력"으로 세어 과대평가한다. γ 상한에 필요한 것은 공 진행 방향 성분 $\hat v^\top J_p\dot q^u$ 뿐이다. 음수(역방향)는 0 으로 둔다.

**0 가드.** $\dot q_{\max,i}\le0$ (또는 NaN) 이면 분모가 0·무한이 되어 값이 조용히 무의미해진다. 한계 무효는 플래그로 보고하고 $v_{dir,\max}=0$ (보수적)으로 둔다. 분모 자체가 0 (관절 운동 불필요)은 판정 불가 (`undetermined`) 로 보고 0 을 돌려준다 — 이 값은 창에 물리량으로 들어가지 않고 γ 창을 판정 불가로 만든다.

남는 보수성: 최소노름 해만 보므로 여유 자유도를 최대한 쓴 LP 최적값보다 작거나 같다. 6축 5-DoF 과제에서는 여유가 1자유도라 차이가 작고, 7축에서는 더 벌어진다.

특이 자세 자체는 §4.2 의 manipulability 게이트 $w_5$ (D-18) 가 거른다. $J_p$ 가 랭크를 잃으면 $J_5$ 도 랭크를 잃어 $w_5=0$ 이므로, 별도의 $\sqrt{\det J_pJ_p^\top}$ 게이트는 두지 않는다.

**Sanity check.** $v_{dir,\max}=0$(정지 포구)이면 $\Vert v\Vert\le d/T_{close,tot}$다. [R1]의 포켓 3 cm, 6 m/s를 넣으면 $T_{close}\le5$ ms다. [R1] 각주 2 원문으로 확인했다("Assuming that the ball flies within the hand about 0.03 m with a velocity of 6 m/s the time duration of 5 ms is obtained"). `test_catching_time_feasibility` 가 5 ms는 통과, 5.1 ms는 창이 빔을 검사한다.

**창이 비는 조건.** 받을 수 있는 최대 공 속력은

$$\Vert v\Vert_{\max}=\min(v_{dir,\max},\eta_vv_{\max})+\frac{d_{eff}}{T_{close,tot}}$$

이고 이를 넘는 후보는 게이트 실패다(`MaxCatchableSpeed`. 경계에서 1 ulp 엇갈리지 않게 `planner.search.grid.gamma.margin` 만큼의 여유를 두고 $\Vert v\Vert+$ margin $\le\Vert v\Vert_{\max}$ 로 비교한다). **이 식이 시스템 전체의 실현 가능성을 결정한다** — 마스터 §4.1을 볼 것. $v_{dir,\max}=1.5$ m/s, $d_{eff}=4$ cm, $T_{close,tot}=60$ ms이면 상한이 2.17 m/s에 불과하다. 창이 비어도 ($\gamma_{\min}>\gamma_{\max}$) 후보는 남는다 — rollout 은 그때 팔의 한계 $\gamma_{\max}$ 에서 $\gamma_f$ 를 찾고 (§4.8), 나머지 상대속도는 손이 받는다.

### 4.6 포구 오차 예산 `[논문 외 유도]`

**적용: 공통** (순위 게이트 `kRankErrorBudget`).

commit 시 포구점 $p_c$가 고정되고, 포구 순간 기준은 $p_c+\gamma(\hat p_{live}(t_c)-p_c)$에 있다. 실제 공 위치를 $p_{true}$라 하면

$$\text{gap}=(1-\gamma)\big(p_{true}-p_c\big)+\gamma\big(p_{true}-\hat p_{live}(t_c)\big)+\varepsilon_{trk}+\varepsilon_{clk}$$

- 첫 항: commit 시점 예측 오차, $(1-\gamma)$배로 감쇠.
- 둘째 항: 포구 순간의 실시간 추정 오차(스냅샷 나이 포함).
- $\varepsilon_{trk}$: L5 추종·지연 잔차.
- $\varepsilon_{clk}\approx\Vert v\Vert\delta$: 시계 오차.

**첫 두 항은 독립이 아니다.** $\hat p_{live}$ 는 $p_c$ 보다 나중 정보를 쓴 추정이므로 두 오차가 강하게 상관돼 있다. 직교 분해로 정리한다. $A=p_{true}-\hat p_{live}$, $B=\hat p_{live}-p_c$ 로 두면

$$\text{gap}=(1-\gamma)(A+B)+\gamma A=A+(1-\gamma)B$$

최적 추정이면 혁신 $B$ 와 잔차 $A$ 가 직교하므로 $\sigma_c^2=\sigma_\ell^2+\sigma_B^2$ 이고,

$$\boxed{\sigma_{gap}^2=\sigma_\ell^2+(1-\gamma)^2(\sigma_c^2-\sigma_\ell^2)=(1-\gamma)^2\sigma_c^2+\big(1-(1-\gamma)^2\big)\sigma_\ell^2}$$

게이트는 다음과 같다 (`CatchErrorSigma`, `ErrorBudgetOk`).

$$n_\sigma\sqrt{(1-\gamma)^2\sigma_c^2+(2\gamma-\gamma^2)\,\sigma_\ell^2+\sigma_{trk}^2+(\Vert v\Vert\delta)^2}\le r_{cap}$$

$\sigma_c$는 **commit 시점 메시지**의 $t_c$ 샘플 공분산에서, $\sigma_\ell$은 **포구 직전 최신 메시지**의 같은 시각 샘플 공분산에서 뽑는다. 둘 다 vision이 준 값이다(§4.4). $\sigma_{trk}$와 $\delta$는 실측값(L5, 인프라)이며 나머지와 독립으로 두는 것은 타당하다.

commit 시점에는 $\sigma_\ell$ 을 아직 모른다. 경로는 둘이고, **판정에 쓰는 것은 경로 1 뿐이다**.

1. **계획 단계**: 보수적으로 $\sigma_\ell=\sigma_c$ 로 둔다 (`GridCatchSearch` 의 `CatchErrorSigma(γ_f, σ, σ, …)`). 그러면 식이 $\sigma_c^2$ 로 환원되는데, 이것은 **직교성 가정 없이도 상한**이다 — $\mathrm{Var}(A+\lambda B)$ 는 $\lambda=1-\gamma$ 의 볼록 2차식이라 $\lambda\in[0,1]$ 에서 최대가 끝점이고, $\sigma_\ell\le\sigma_c$ 인 한 그 값이 $\sigma_c^2$ 다.
2. **동결 후 감시**(§5.3 `monitorOnly`): 최신 메시지에서 $t_c$ 의 공분산 (가장 가까운 표본의 $6\times6$ 을 $F\Sigma F^\top$ 으로 $t_c$ 까지 전파한 것 — `SampleBallNode`, §5.3) 으로 $\sigma_\ell$ 을 갱신한다 (`GridCatchSearch::Monitor`). **이 값으로 abort 를 판단하지 않는다** — $\sigma_\ell$ 은 `planner_events.csv` 의 `sigma_l` 열로 기록만 하고, 동결 뒤의 낡은 입력은 steady 수신 나이로 판정한다 (L7). 따라서 직교 분해의 $\gamma$ 의존 이득은 어느 판정에도 쓰이지 않는다. 다시 보는 것은 실기 공분산 검증 수단을 정할 때다 (#613).

**직교성이 깨질 때의 방향.** 정확한 오차항은 $2\gamma(1-\gamma)\mathrm{Cov}(A,B)$ 이고 $\gamma=0,1$ 에서 사라져 $\gamma=0.5$ 에서 최대다. **새 측정을 과소 반영하는(sluggish) 예측기** — 측정잡음 과대설정, 공정잡음 과소설정 같은 흔한 튜닝 실패 — 는 $\mathrm{Cov}(A,B)>0$ 을 만들어 위 식이 $\sigma_{gap}$ 을 **과소평가**하게 한다. 반대로 $\mathrm{Cov}(A,B)<0$ 이면 위 식은 $\sigma_{gap}$ 을 **과대평가**한다 (보수적).

NEES가 정상이어도 $\mathrm{Cov}(A,B)=0$ 은 보장되지 않으므로, **혁신 백색성(innovation whiteness) 검정**을 게이트에 넣는다(G3-H). L1 §4.5의 $\bar\nu$ 추세가 그 대용이다.

**직교 가정은 sim 에서 성립하지 않는다 — 식은 유지한다.** sim truth 로 $A$·$B$ 를 직접 잰 상관은 음이다 (G3-H = L8 의 G8-C2). 그래도 식을 유지하는 근거는 셋이다.

1. **런타임 검사는 직교 가정을 쓰지 않는다.** 계획기는 위 경로 1 대로 $\sigma_\ell=\sigma_c$ 를 넣으므로 검사는 $\gamma$ 와 무관하게 $n_\sigma\sqrt{\sigma_c^2+\sigma_{trk}^2+(\Vert v\Vert\delta)^2}\le r_{cap}$ 이다. 직교 분해의 $\gamma$ 의존 항은 어느 판정에도 들어가지 않는다
2. **어긋남의 방향이 보수적이다.** 음의 상관은 식이 간극을 크게 잡게 할 뿐이고, 이 검사는 순위 게이트라 (§4.1, D-27) 초과해도 후보가 탈락하지 않는다
3. **식을 바꿔 얻을 것이 지금은 없다.** $\gamma$ 의존 예산을 판정에 쓰려면 $\sigma_\ell$ 의 commit 시점 예측이 필요한데 그 경로가 없다

상관의 크기는 항등식 $\sigma_c^2=\sigma_\ell^2+\sigma_B^2$ 가 깨진 정도이지 $\sigma_{gap}^2$ 의 과대추정 배수가 아니다 — 예산식의 오차는 위의 $2\gamma(1-\gamma)\mathrm{Cov}(A,B)$ 로, $\gamma$ 에 따라 0 에서 $\tfrac12\vert\mathrm{Cov}\vert$ 사이다.

식을 다시 볼 조건: 경로 2 를 판정에 써서 $\gamma$ 의존 예산을 판정이나 $\gamma$ 선택에 쓰기로 할 때. 그때는 교차항을 경험적으로 보정하거나 직교 가정 없는 상한으로 바꾼다. **경로 1 의 상한은 $\sigma_\ell\le\sigma_c$ 를 전제한다** — 음의 상관이 커서 $2\vert\mathrm{Cov}(A,B)\vert>\sigma_B^2$ 이면 이 전제가 깨지므로, 그 재검토는 $E\Vert A\Vert^2$ 대 $E\Vert A+B\Vert^2$ 를 먼저 본다.

**지금의 구현.** 게시되는 `PlanSnapshot::sigma_l` 은 $\sigma_c$ 와 같은 값이다 (경로 2 의 $\sigma_\ell$ 은 CSV 에만 있고 읽는 소비자가 없다). 출하 profile 은 $\sigma_{trk}=0$, $\delta=0$ 이고 $\kappa_\sigma<1/n_\sigma$ 라서 이 검사가 실패하는 후보는 §4.4 불확실성 게이트도 이미 실패한다 — 이 검사는 독립된 정보가 아니라 같은 $\sigma_c$ 에 대한 두 번째 임계로 동작한다.

$\sigma$ 는 스칼라로 썼지만 실제는 3×3이다. §4.4의 $\lambda_{\max}$ 규약으로 읽으며, $\lambda_{\max}(\Sigma_A+\lambda^2\Sigma_B)\le\lambda_{\max}(\Sigma_A)+\lambda^2\lambda_{\max}(\Sigma_B)$ 이므로 그 경우에도 보수적 상한이다.

**부수 함의.** $\gamma\to1$ 이면 $\sigma_{gap}\to\sigma_\ell$ 로 줄어든다. 즉 γ를 키우는 것은 충격량(L7 §4.7)뿐 아니라 **예측 오차 측면에서도 유리하다** — commit 시점의 낡은 예측 대신 실시간 추정에 가중이 실리기 때문이다. §4.10의 γ 선호는 이 근거를 함께 쓴다.

**$\sigma_{trk}$ 의 성격.** L5 CLIK은 명령 공간에서만 닫히므로(L5 §4.2) $\sigma_{trk}$ 는 실행 중 관측되지 않는 오프라인 식별값이다. $T_{arm}$ 모델의 잔차가 그대로 여기에 들어간다.

### 4.7 교체 히스테리시스

**적용.** 아래의 규칙 전부와 "런타임" 의 게시 방식은 `closed_form` 의 것이다. 구간 계획기 (`mpc` · `mpc_docking`) 에서도 `grid` 탐색은 RT 가 plan 을 따르는 동안 돌며 같은 판정을 내고, 계획기의 한 주기가 그 판정을 이렇게 쓴다 (§5.3): 교체 (`replaced`) 면 새 plan 을 그 첫 구간과 **쌍으로** 게시하고 RT 가 그 구간의 node 0 에서 plan 과 구간을 함께 바꾼다 (L7 §4.3a). 그 밖 — 갱신 (`refreshed`) · 보류 · 후보 없음 — 은 게시하지 않고 따르는 plan 의 구간을 재계획한다. 그 판정에서 2 (η_jump 가속 계단) 는 보지 않는다 — 계단이 들어갈 L4 기준 $u_{des}$ 가 구간 계획기 아래에는 없다 (`GridCatchSearchConstants::follows_segments`). 교체는 1 의 $\Delta J$ (또는 현재 plan 의 불가능 판정) 만으로 정해지고, 3 (동결) 은 그대로다. 따라서 `planner.search.grid.switch.eta_jump` · `samples` 는 `closed_form` 에서만 쓰이고 `switch.delta_J` 는 둘 다에서 쓰인다. 구간 계획기에서 같은 포구의 예측 변화는 구간의 재계획이 받는다 (formulation).

현재 plan이 유효하면 새 후보는 다음을 모두 만족할 때만 채택한다.

1. 점수 개선 $J_{cur}-J_{new}>\Delta_J$, 또는 현재 plan이 이번 검사에서 불가능 판정.
2. 가속 예산 (L4 §4.3): 교체가 $u_{des}$ 에 넣는 계단의 상계가 $\eta_{jump}\,a_{\max}$ 이하.

$$\omega^2(1-\gamma)\Vert\Delta p_c\Vert+\bigl(2\zeta\omega|\dot\gamma|+|\ddot\gamma|\bigr)\Vert o-p_c\Vert+2|\dot\gamma|\,\Vert v_o\Vert\;\le\;\eta_{jump}\,a_{\max}$$

   $p_c$ 는 **옛** 포구점, $\omega,\zeta,a_{\max}$ 는 탐색이 가진 L4 기준의 값 (`planner.search.grid.reference.*`) 이다 (`SwitchAccelStepBound`). 좌변은 **RT 가 이 사이클의 게시를 채택할 수 있는 구간** $[now_{lead},\,now_{lead}+$`budget_s`$+2h]$ 의 최악값으로 판정한다 (`planner.search.grid.switch.samples` 개의 순간에서 평가 — `GridCatchSearch::SwitchStep`): $\gamma,\dot\gamma,\ddot\gamma$ 는 RT 가 실제로 돌리는 램프 (`PlannerRtState.ramp_*` — RT 가 채택하며 바꾼 g0·t0 포함) 를 그 구간에서 평가한 값, $(o,v_o)$ 는 같은 시각의 공 대상 (rollout 과 같은 샘플러) 이다. 스냅샷 tick 의 $\dot\gamma=\ddot\gamma=0$ 은 램프 시작 직전일 수 있어 채택 시점의 값이 아니다. 램프 정보가 없으면 스냅샷 값을 쓴다. RT 는 교체 plan 을 γ 는 이어서, 5차 램프는 **다시 시작해서** ($\dot\gamma=\ddot\gamma=0$) 채택하므로 실제 계단은 $\Delta u=\omega^2(1-\gamma)\Delta p_c-(2\zeta\omega\dot\gamma+\ddot\gamma)(o-p_c)-2\dot\gamma v_o$ 이고, 위 식은 그 삼각 상계다. 램프 항은 $\Vert\Delta p_c\Vert$ 가 아니라 $\Vert o-p_c\Vert$ 에 비례하므로 **램프가 도는 동안의 교체는 거의 다 거부된다** — 램프를 이어 붙이는 채택 ($\dot\gamma,\ddot\gamma$ 연속) 은 구현에 없다. 램프가 멈춘 구간 ($\dot\gamma=\ddot\gamma=0$) 에서는 $\Vert\Delta p_c\Vert\le\eta_{jump}a_{\max}/(\omega^2(1-\gamma))$ 다. 공 대상을 샘플할 수 없는 순간이 있으면 판정 불가 (거부) 다.
3. `COMMITTED` 이후에는 포구점을 교체하지 않는다(L7). 그 앞에서도 따르는 plan 의 $t_c$ 까지 `planner.freeze.T_freeze` 이내면 교체하지 않는다 (`held_freeze`).

**런타임.**

- **현재 plan 은 RT 가 따르는 plan 이다.** 계획기는 최근 게시 몇 개를 기억하고, RT 가 `PlannerRtState` 로 보고한 `plan_id` 로 "현재" 를 찾는다. RT 가 거부한 게시 (freeze·나이·더 새 게시) 는 현재가 되지 않는다.
- **현재 plan 의 후보를 먼저 평가한다.** 1 의 "불가능 판정" 이 `max_ik`·예산에 밀려 평가 안 된 것을 뜻하지 않도록, 따라가는 plan 의 $t_c$ (반 slice 안) 후보를 IK 순서 맨 앞에 둔다.
- **갱신 (`refreshed`).** 따라가는 후보가 여전히 최선인데 예측된 $p_c$ 가 `planner.search.grid.gamma.eps_term` 을 넘게 움직였으면 새 $p_c$ 로 재게시한다 (2·freeze 적용 — 2 가 거부하면 `held_jump`). 히스테리시스는 "다른 후보로 바꾸는가" 의 규칙이지 "옛 예측을 붙잡는가" 가 아니다 — 붙잡으면 soft catch 가 $(1-\gamma_f)\Vert\delta\Vert$ 만큼 빗나간다.
- **후보가 없는 사이클은 게시하지 않는다.** 따라가는 중 settle·후보 0·입력 무효로 끝난 사이클은 "plan 없음" 을 게시하지 않는다 (`held_no_candidate`) — 게시하면 RT 가 아직 안 읽은 교체를 덮는다.
- **교체의 γ 는 연속이다.** RT 는 교체 plan 의 γ 램프를 **채택 tick 의 기준 γ** 에서, **그 tick 이후에** 시작한다. 계획 시점의 γ 나 이미 지난 램프 시작을 쓰면 γ 계단이 $\gamma\,(o-p_c)$·$\ddot\gamma\,(o-p_c)$ 를 통해 $e$·$u_{des}$ 의 계단이 된다.

`COMMITTED` 진입(commit 조건은 §4.11)부터 $p_c$, $t_c$, $\gamma$ 프로파일은 동결된다. 실행 중 기준이 포화한 채로 남으면 (`REF_SATURATED`) `COMMITTED` 이전은 RETREAT, 이후는 ABORT_SAFE 다 `[확정 D-8]`. `COMMITTED` 뒤에 γ 를 낮추는 경로는 없다 (v1 범위 밖).

### 4.8 γ rollout (포화 검사)

**적용: 공통** (순위 게이트 `kRankRollout`, 그리고 $\gamma_f$ · $T_w$ 의 선택). rollout 이 굴리는 것은 `closed_form` 의 RT 법칙 (L4 soft-catch DS) 이다. `mpc` 에서도 탐색은 이 rollout 으로 후보의 순위와 $\gamma_f$ 를 매긴다 — 다만 그 법칙이 RT 에서 실행되지는 않으므로, `mpc` 에서 이 판정은 실행의 예측이 아니라 후보 순위의 기준이다.

후보마다 격자 $(\gamma_f,T_w)\in\Gamma\times\mathcal T$, $\gamma_f\in[\gamma_{\min},\gamma_{\max}]$에 대해 L4 `SoftCatchTranslation`을 **포화 없이** 실행한다 (`gamma_rollout.hpp`). 기준의 값 ($\omega,\zeta,v_{\max},a_{\max}$) 은 탐색의 `planner.search.grid.reference.*` 다 — `closed_form` 의 `reference.*` 가 아니다. 초기 상태는 현재 기준 상태 (기준이 돌고 있지 않으면 현재 명령 자세의 catch frame 위치, 정지), 대상은 **L2 샘플러가 vision 궤적에서 보간한 $(p,v,a)$** 다 — RT 루프가 실제로 볼 것과 같은 함수를 쓴다(`SampleAt`). **rollout 시간축은 $now_{lead}=now+T_{arm}$ 이다** (L0 §4.5 — 궤적 샘플링·γ 프로파일·기준 생성은 모두 선행 시각). γ 램프는 $[t_k-T_w,\,t_k]$ (시작은 $now_{lead}$ 로 자름) 에서 돈다. 다음을 기록한다.

- 구간 $[now_{lead},t_k]$의 $\max\Vert u\Vert$, $\max\Vert\dot x\Vert$ (그리고 γ 창 $[t_k-T_w,t_k]$ 안에서의 같은 두 값)
- $t_k$의 잔여 오차 $\Vert e\Vert$

판정에는 `TranslationOutput::u_des`(포화 전 요구 가속도)를 쓴다. `xdd`(실현값)는 포화가 없으면 같지만 의미가 다르다(L4 §5.2).

수락 조건: $\max\Vert u_{des}\Vert\le\eta_aa_{\max}$, $\max\Vert\dot x\Vert\le\eta_vv_{\max}$, $\Vert e(t_k)\Vert\le\epsilon_{term}$ ($\eta<1$ 여유율).

수락 조합 중 $\gamma_f$ 최대를 고르고, 동률이면 최대 가속이 작은 것을 고른다. 수락 조합이 없으면 $\gamma_{\min}$을 한 번 더 검사한다. 그것도 실패하면 rollout 게이트 실패다 (런타임에서는 순위 실패 — §4.1).

**전 구간과 창.** 판정은 전 구간 $[now_{lead},t_k]$ 의 것이다. 그런데 접근 구간 — 기준이 팔이 있는 곳에서 $p_c$ 로 γ = 0 으로 끌려가는 구간 — 은 $\gamma_f$ 와 무관하게 $\omega^2\Vert x_0-p_c\Vert$ 를 요구하므로, 실제 이동에서는 모든 조합이 전 구간 판정에 실패하고 $\gamma_f$ 의 선택이 정보를 잃는다. 그래서 전 구간 수락이 없을 때 $\gamma_f$ 는 같은 규칙을 **창 안의 최대치**에 적용해 고르고, 판정은 "실패" 로 둔다 (접근 구간이 포화한다는 뜻 — 순위 벌점). 그 선택도 없으면 $\gamma_{\min}$ 과 가장 긴 창이다.

**연산 예산.** 전 격자 × 전 후보를 제어 주기로 적분하면 `planner.search.grid.budget_s` 를 넘으므로 **coarse-to-fine** 이다 — 격자 전체를 `planner.search.grid.rollout.dt_coarse` 로 거르고, 고른 조합 하나만 제어 주기로 확인한다. **거친 단계는 최대치 ($u$, $\dot x$) 만 판정하고 $\Vert e(t_k)\Vert$ 는 확인 단계만 판정한다** — semi-implicit Euler 가 거친 간격에서 움직이는 대상을 약 $\gamma\Vert v\Vert\,dt$ 만큼 뒤따르므로, 거친 단계에서 $\epsilon_{term}$ 을 판정하면 soft catch 가 자기 이산화 오차로 탈락한다. 확인 단계에서 반증되면 판정은 "실패" 이고 $\gamma_f$ 는 창 규칙으로 떨어진다. 사이클 시간은 G3-C (`test_catching_planner_g3c`) 가 본다.

기준 수치 — G3-B 가 런타임 경로로 ±5 % 안에서 재현한다 (`test_catching_gamma_rollout`; 한 시나리오, $\omega=10$, 포화 없는 rollout 의 **창 내** 최대 $\Vert u_{des}\Vert$ [m/s²]):

| $T_w$ \ $\gamma_f$ | 0.1 | 0.2 | 0.3 | 0.4 | 0.5 |
|---|---|---|---|---|---|
| 0.30 s | 8.1 | 16.4 | 24.7 | 33.0 | 41.3 |
| 0.45 s | 5.4 | 10.5 | 15.8 | 21.2 | 26.5 |
| 0.60 s | 6.3 | 8.9 | 11.6 | 15.3 | 19.2 |
| 0.75 s | 8.9 | 8.8 | 10.4 | 12.4 | 14.8 |

필요 가속도는 $\gamma_f$ 에 대해 거의 선형이고, 창 길이의 선택이 포화 여부를 지배한다 (짧은 창 $T_w=0.30$ s 에서 $\gamma_f=0.4$ 에 33 m/s²).

### 4.9 정지거리 예약

**적용: 공통** (판정 게이트 — `catch_box`). `mpc` 에서 실제 정지는 MPC 구간이 만들지만, 탐색은 같은 닫힌 형태의 정지점으로 후보를 거른다. RT 는 `catch_box` 를 검사하지 않는다 — 상자는 탐색의 것이다.

포구 후 감속(L7 §4.3)에 필요한 공간을 확보한다.

$$p_{stop}=p_c+\frac{(\gamma_f\Vert v\Vert)^2}{2a_{dec}}\hat v\ \in\ \mathcal W_{catch}$$

$a_{dec}$ 는 탐색의 `planner.search.grid.stop.a_dec` 다 — L7 감속의 `supervisor.decel.a_dec` 의 복사본이고 `closed_form` 에서는 둘이 같아야 한다 (§6). 함수는 `StoppingPoint` 다. 런타임은 rollout 이 고른 $\gamma_f$ 에서, 오프라인 지도는 γ 창의 양 끝에서 평가한다. $p_{stop}$ 의 IK 는 검사하지 않는다.

### 4.10 선택 규칙 `[권장]`

**적용: 공통.**

단일 가중 점수 최소화를 쓴다. γ도 그 안의 한 항이다.

$$J=w_\sigma\frac{\sigma_{\max}(t_k)}{r_{cap}}+w_t\frac{\max_it_{\min,i}}{t_k-now-T_{arm}}+w_q\Vert q^\ast-q_n\Vert^2+w_{late}\,(t_{k,\max}-t_k)-w_\gamma\,\gamma_f+w_{pen}\,n_{fail}$$

$n_{fail}$ 은 그 후보가 실패한 순위 게이트의 수 (§4.1 의 여섯 가운데, `RankGateBit` 의 켜진 비트 수), $w_{pen}$ 은 `planner.search.grid.score.penalty` 다. $\sigma$ 를 모르는 후보는 첫 항이, 도달시간을 쓸 수 없는 후보는 둘째 항이 빠진다 — 그 후보는 해당 순위 게이트에 실패한 것이라 $n_{fail}$ 에 들어간다. $q_n$ 은 대기 자세 (IK 의 seed, §4.2) 다.

$w_{late}>0$이면 늦은 포구를 선호한다([R1]의 "latest" 목적과 같은 취지로, 예측이 정확해지는 시간을 번다). $w_\gamma>0$이면 soft catch를 선호한다(충격량 L7 §4.7, 오차 예산 §4.6 두 근거). 가중치는 튜닝 대상이다 (`planner.search.grid.score.*`).

$\gamma_f$ 를 사전식 (lexicographic) 1순위로 두지 않는 이유: $\gamma_f$ 가 연속 격자값이라 동률이 거의 나오지 않아 1순위에서 후보가 결정되고 나머지 가중이 전부 죽는다. $J$ 안의 항으로 두면 $w_\gamma$ 로 그 상충을 튜닝할 수 있고, $\gamma_f$ 를 사실상 절대 우선으로 두고 싶으면 $w_\gamma$ 를 크게 잡으면 된다 — 사전식은 $w_\gamma\to\infty$ 의 특수한 경우다.

### 4.11 commit과 손 명령 시각

**적용: 공통** (commit 은 supervisor 의 것이고 두 planner 에서 같다).

- commit 조건 (APPROACH→COMMITTED, L0 §4.5): $t_c-now_{real}\le T_{freeze}$. 비교 대상은 **실제 시각**이고, 팔 선행분은 $T_{freeze}$ 하한의 $T_{arm}$ 항이 흡수한다. **$T_{freeze}$ 의 하한**: 검증기 `CheckFreezeCoversClose` 가 $T_{freeze}\ge T_{close,lead}+T_{arm}+h$ ($h$ = 제어 주기) 를 강제한다 (`lead_enable` 과 무관하게 $T_{arm}$ 을 읽는다). 계획기 후보 창 하한 (`planner.search.grid.slice.t_lead_min` 이 없을 때) 과 RT 채택 거부 (g) 도 $T_{freeze}$ 다. 순위 게이트 `kRankCommitLead` 는 후보의 선행 시간이 $t_k-now\ge T_{close,lead}+h/2+T_{arm}+T_{margin}$ ($T_{margin}$ = `planner.search.grid.time.margin`) 인지를 본다 — 이쪽은 $T_{freeze}$ 의 조건이 아니라 후보의 순위 조건이다 (아래 두 시간의 구분). $T_{arm}$ 을 올리면 $T_{freeze}$ 하한도 함께 올라가고 그만큼 후보 창이 짧아진다
- **손의 두 시간.** `robot.hand.T_close_e2e` 는 **잰** 폐쇄 시간 (명령 → 폐쇄 완료, 종단 간 — L6 §4.1, D-11 로 $T_{link}$ 를 따로 재지 않는다) 이고 `robot.hand.T_close_lead` 는 포구점에 공이 닿기 **얼마 전에** 폐쇄를 지령하는가의 **설계값**이다 (L6 §4.2). γ 창과 $d_{eff}$ 의 $T_{close,tot}=T_{close,e2e}+h/2$ (§4.5, 시간 가능성 게이트) 는 잰 값을 쓰고, commit 게이트 · $T_{freeze}$ 하한 · $t_{cmd}$ 는 lead 를 쓴다. 두 값이 같다는 가정은 없다.
- 손 폐쇄 명령 시각: $t_{cmd}=t_c-T_{close,lead}$. $t_c$ 가 팔의 운동에서 어느 순간인가와 lead 가 어느 축의 값으로 실행되는가 (`mpc_docking` 아래의 환산) 는 L6 §4.3 이 갖는다. 팔 지연은 L5 선행 보상으로 이미 흡수되므로 빼지 않는다. **단일 출처 (C-14).** 계획기(`grid_catch_search.cpp`)와 oracle(`StoreOraclePlan`)은 `PlanSnapshot::t_cmd_ns` 를 각자 채운다 (oracle 은 $t_{cmd}=t_c$). 손 명령에 쓰는 값은 **손 시퀀서**가 COMMITTED 에서 동결된 $t_c$ 와 손 프로파일로 계산한 것 하나다 (`HandSequencer::Commit`, 접촉 판정 창도 같은 값) — `PlanSnapshot::t_cmd_ns` 는 계획기 기록·진단용일 뿐 손 명령에 쓰이지 않는다. 구간 계획기 아래에서는 RT 가 그 값을 폐쇄 지령 전까지 공의 통과 시각에 다시 맞춘다 (L6 §4.3) — 계획기는 관여하지 않는다. 손 명령 발동은 L6 §4.3/§5.3 의 `HandCommandDueRounded`(R-CLOSE, L7 §4.1) 로 한다.
- **$T_{tick}$은 여기에 넣지 않는다.** §4.5의 $T_{close,tot}=T_{close,e2e}+T_{tick}$ 에서 $T_{tick}=h/2$ 는 틱 양자화 오차의 **worst-case 예산**이다. L6 §4.3의 반올림 규칙(가장 가까운 틱)을 쓰면 오차는 $\pm h/2$ 로 영평균이라 명령 시각 자체를 당길 이유가 없다. γ 창(예산)에는 들어가고 $t_{cmd}$(명령)에는 들어가지 않는다 — 두 곳의 역할이 다르다.
- **시간 규약 `[확정 D-2]`.** $t_c$ 와 $t_{cmd}$ 는 모두 공의 **물리 시각** `BallTime` (절대 steady ns) 단일 정의다. "선행축 시각" 이나 "실제시각축 시각" 이라는 별도의 $t_c$ 는 없다 — 축은 시각 값이 아니라 **판정마다 비교하는 '지금'** 에 붙는다 (L0 §4.5): 손 명령은 $now\ge t_{cmd}$ (실제, 손은 선행 보상 없음), DECEL 진입은 $now_{lead}\ge t_c$, rollout·γ 프로파일은 $now_{lead}$.
- [R2]는 접촉 직전 일정 시간부터 예측 갱신을 멈췄다. 본 구현의 동결은 포구점·γ에만 적용하고, L2의 실시간 추정은 계속 사용한다(§4.6 둘째 항).

## 5. C++ 구현

### 5.1 `time_feasibility.hpp` (S1.5 이식 완료)

**적용: 공통.** 정의는 `rtc_controllers/include/rtc_controllers/catching/time_feasibility.hpp` 다 — 문서에 코드를 복제하지 않는다. 함수와 이 문서의 절:

| 함수 | 절 |
|---|---|
| `TRest` · `TMinChecked` (`TMinResult`) · `TMin` · `MaxJointTMin` · `ReachTimeFeasible` | §4.3 |
| `PlanningTcpSpeed` · `ComputeGammaWindow` (`GammaWindow`) · `DirectionalSpeedMax` · `MaxCatchableSpeed` | §4.5 |
| `StoppingPoint` | §4.9 |
| `CatchErrorSigma` · `ErrorBudgetOk` | §4.6 |

한 후보의 도달시간 · γ 창 · 정지점을 묶어 판정하는 것은 `rank_gates.hpp` 의 `JudgeRankGates` 이고 (오프라인 지도와 런타임이 같이 부른다 — G3-I), 방향 속력의 $\dot q^u$ 는 `unit_speed.hpp`, rollout 은 `gamma_rollout.hpp` 다. 전부 할당 0 · `noexcept` 다 (계획기 스레드가 SCHED_FIFO 일 수 있다). 무효 입력은 통과하는 값이 아니라 플래그와 $+\infty$ · 0 · NaN 으로 닫힌다 (fail-closed, NUM-2/NUM-4).

### 5.2 `PlanSnapshot`

**적용: 공통.** `mpc` 는 여기에 `SegmentSnapshot` (별도 SeqLock) 이 더해진다.

계획기 → RT 전달은 `rtc::SeqLock<PlanSnapshot>` 이다. 정의는 `rtc_controllers/include/rtc_controllers/catching/trajectory.hpp` (`PlanSnapshot` · `ProvenanceToken` · `PlanReason` · `SegmentSnapshot`), RT 쪽 수락 규칙은 `planner_io.hpp` (`JudgePlan` · `JudgeSegment`) 다. **SeqLock payload 는 trivially copyable 이어야 하므로 `PlanSnapshot` 은 POD 다** — Eigen 멤버 금지, 벡터는 `std::array<double, N>`, 시각은 절대 정수 ns. 필드의 뜻 가운데 헤더에 없는 규범:

- provenance (`activation_generation` · `generation` · `snapshot_sequence` · `traj_recv_ns`) 는 중첩 `ProvenanceToken token` 안에 있다. 궤적 스냅샷 · 공분산 버퍼 · plan 이 같은 네 값을 싣는다 (D-22, D-23)
- `t_c_ns` · `t_cmd_ns` 는 둘 다 `BallTime` 이다 (§4.11, D-2) — 상대시간·축 구분이 없다. `t_cmd_ns` 는 기록·진단용이고 손 명령에 쓰이지 않는다 (§4.11)
- `gamma_g0` · `gamma_gf` · `gamma_t0_ns` · `gamma_t1_ns` 는 L4 `GammaProfile` 의 파라미터다. 시각은 절대 ns 이고 $now_{lead}$ 와 비교해 평가한다. 상대시간 변환은 L4 수치 코어 경계에서 한다. `gamma_min` 은 §4.5 γ 창 하한 (진단)
- `q_star` 는 팔 **device 순서**다. 용량 `kMaxPlanNv` 는 **계획기 control 모델 nv** 를 담는 컴파일 타임 상수이고 configure 에서 nv ≤ `kMaxPlanNv` 를 검사한다. 궤적 점 용량 `kCap` 과는 다른 상수다
- `w5` · `w6` 는 정의 (`planner.search.grid.catchability.definition`) 와 무관하게 **항상 함께** 기록한다 (D-18, C-3)
- `sigma_l` 은 `sigma_c` 와 같은 값이다 (§4.6). `dp_impact` 는 예상 충격량 [kg m/s] 의 기록이다 (§4.1)
- `reason` 은 plan 이 없을 때 가장 많이 걸린 판정 게이트다. 작업공간 탈락 ($p_c$ 든 $p_{stop}$ 이든) 은 `kStoppingDistance` 로, 예산에 밀려 평가 못 한 것은 `kBudgetExceeded` 로, settle 중은 `kUncertainty` 로, 창 안에 후보가 없으면 `kHorizonShort` 로 읽힌다 — 어느 쪽이 상자를 벗어났는지는 `planner_events.csv` 가 갖는다

**RT 소비자의 fail-closed 판정 (D-21, D-22, D-23).** 매 tick `Load()` 를 무조건 한 번 하고(D-21), payload 안의 값만으로 판정한다 (`SeqLock::sequence()` 를 따로 읽지 않는다). 새 plan 판정은 `snapshot_sequence` 가 아니라 **`plan_id`** 로 한다 — `snapshot_sequence` 는 궤적의 것이라 같은 궤적에서 계산한 두 plan 을 가르지 못한다. `plan_id` 는 writer (계획기, 또는 테스트·sim 용 oracle) 의 단조 카운터이고, RT 는 (a) `valid` · (b) `activation_generation` 일치 · (c) `generation` 이 RT 가 마지막으로 받은 궤적의 트랙 epoch 과 일치 · (d) `plan_id` 가 이미 받은 값과 다름 · (e) 게시 나이 (`now − publish_ns`) 가 0 이상이고 `io.t_stale` 이하 · (f) `publish_ns` 가 RT 의 마지막 리셋 시각 이후 · (g) 아래 R-ADMIT — 를 모두 만족할 때만 받는다. (f) 는 E-STOP 처럼 activation generation 을 올리지 않는 리셋을 덮는다. 하나라도 실패하면 그 tick 은 새 payload 를 쓰지 않고 이전 유효 plan(또는 무효 상태)을 유지한다. **token 불일치·나이 초과를 위한 새 `Reason` 은 만들지 않는다** (값 하나가 enum·`kAllReasons`·문자열·`CatchingState.msg` 네 곳에 걸리고 msg 는 D-20 동결): TRACKING 에서는 `kNoCatchablePlan` 으로 읽는다 (거부 사유 자체는 `PlanRefusal` 로 관측된다). RT 는 plan box 에 쓰지 않는다 — writer 는 하나뿐이다 (계획기와 oracle 이 함께 켜진 설정은 park).

**R-ADMIT (C-31).** **(g)** `t_c-now\le T_{freeze}` 이면 거부한다(`kTooLate`) — 이 조건이 없으면 $T_{freeze}$ 안의 $t_c$ 를 가진 plan 도 채택 다음 tick 에 곧바로 commit 하고, $t_c\le now$ 인 plan 은 몇 tick 만에 `DECEL` 까지 통과해 버린다. 파라미터 검증기는 `T_freeze ≥ T_close_e2e + T_arm + h` 를 강제한다(§4.11 의 코드 하한). fixture 는 `planner.freeze`(`T_freeze`)를 명시해야 한다 — 미지정이면 이 조건이 꺼진다.

**`mpc` 의 쌍.** RT 는 TRACKING 에서 plan 을 그 첫 구간 (`SegmentSnapshot`) 과 **같은 tick 에 함께** 채택한다 — 구간이 수락 (`JudgeSegment`) 되지 않으면 plan 도 받지 않는다 ("plan 없음" 과 같다). APPROACH 중에 온 다른 plan 도 그 첫 구간과 쌍일 때만 받고, 받은 tick 이 아니라 그 구간의 node 0 에서 바꾼다 (L7 §4.3a).

계획기 쪽도 대칭으로 검사한다: 계산 시작 시 최신 token 을 한 번 읽고, 게시 직전에 다시 읽어 그 사이 대체됐으면 게시를 버린다. 궤적 스냅샷의 token 과 공분산 버퍼의 token 이 다르면(궤적 N ↔ 공분산 N−1 혼합 포함) 그 조합은 쓰지 않는다.

### 5.3 계획 루프 (계획기 스레드, D-7)

**적용: 공통** (스레드 · 데이터). "한 번 깨어났을 때의 순서" 는 planner 마다 다르다 — 아래에 둘 다 적는다.

**스레드 `[확정 D-7]`.** 기존 MPC 스레드와 같은 생성 방식이다. 정의는 `integrated_bringup/include/integrated_bringup/controllers/catching/planner_thread.hpp` (`CatchingPlannerThread`), 한 번의 wake 의 본문은 `rtc_controllers` 의 `PlannerCycle::Run` (`planner_cycle.hpp`) 이다.

- 클래스: `rtc::PeriodicRtThread` 의 **형제 subclass** (`rtc::mpc::MPCThread` 를 상속하지 않는다 — PlanSnapshot 을 `MPCSolution` 에 억지로 넣게 된다). 탐색 코어는 rtc_controllers `catching`, 스레드 소유는 `integrated_bringup` 바인딩 (D-1)
- 기동 `[확정 D-7c]`: event 구동 — nrt 파서가 새 궤적을 수락하면 eventfd 로 깨운다. `WaitForNextTick` 은 eventfd 대기 + 제한 시간 (`planner.wake_timeout_s`) 이라, vision 이 조용해도 움직이는 RT 상태에 대해 다시 계획한다. eventfd 는 여러 신호를 한 번으로 합치므로(coalescing), 깨어난 신호 횟수와 무관하게 **항상 최신 스냅샷을 읽는다**. RT tick 은 eventfd 에 손대지 않는다
- 수명: configure 에서 전 버퍼(궤적·공분산 버퍼, IK 작업 공간, rollout 상태, 진단 큐)를 할당하고, activate 에서 layout profile 게이트 → lazy spawn → `Resume`, deactivate 에서 `Pause`, **join 은 `on_cleanup` 에서** 한다. 계획기 스레드는 **한 configuration 의 것**이다 — 스레드가 설정보다 오래 살면 다음 설정 (예: oracle 로 재구성) 에서 resume 돼 box writer 가 둘이 된다. `Pause()` 는 요청 플래그만 세우고 진행 중인 iteration 을 멈추지 않으므로, `on_deactivate` 이후에도 plan 이 한 번 더 게시될 수 있다 — RT 쪽은 그 payload 의 `activation_generation` 이 지금 것과 다르면 소비하지 않는다 (D-23, §5.2 (b))
- 데이터: RT → 계획기는 `rtc::SeqLock<PlannerRtState>` (activation generation · tick · 시각 · mode · 팔 명령 $q_c,\dot q_c$ · 대기 자세 · L4 기준 상태 $x,\dot x,\gamma,\dot\gamma,\ddot\gamma$ 와 돌고 있는 γ 램프 · 따르는 `plan_id` 와 $t_c$ · 따르는 / 대기 중인 구간 · 들고 있는 교체 plan 의 id 와 $t_c$ · 리셋 epoch) 이고 RT tick 이 **매 tick** Store 한다. 궤적 스냅샷은 nrt 파서가 게시한 SeqLock, **공분산은 계획기 쪽 버퍼에만** (A-3 — NaN(모름) 처리도 계획기 한 곳에서), 출력은 `rtc::SeqLock<PlanSnapshot>` 와 (`mpc`) `rtc::SeqLock<SegmentSnapshot>` 이다. box 마다 writer 는 하나다. 재무장 리셋 (L7 §4.8) 은 RT 가 리셋 epoch 을 올리고 reset floor 를 적는 것뿐이다. 끝난 시행을 위해 올라온 wake 신호는 따로 비우지 않는다 — 깨어날 때의 read 가 이미 소비하고, 그 시행용으로 계산된 plan 은 RT 가 reset floor 로 거른다. 리셋을 본 wake 도중에 올라온 신호는 **새 시행**의 궤적이므로 남겨 둔다 (비우면 새 시행의 첫 plan 이 wake timeout 만큼 늦는다)
- **RT-1~10 준수 코드:** 할당 0, `noexcept`, 락·블로킹 I/O 없음, 로깅 금지. 진단(후보 수, 게이트별 탈락, 실행시간, 선택 결과)은 `rtc::SpscQueue` 로 넘기고 aux 타이머가 drain 해 CSV (`planner_events.csv`) 로 쓴다. 그러면 FIFO/OTHER 는 thread layout 값 하나로 바뀌고 코드가 바뀌지 않는다
- 배치: thread layout 의 **`mpc` role** 을 쓴다 (`SelectThreadConfigs().mpc.main`, 스레드 이름 `mpc_main`). 계획기는 MPC 와 같은 역할이고 CM 이 active 컨트롤러를 하나만 두므로 같은 코어에 동시에 도는 FIFO 는 하나다. activate 게이트: `planner.enabled` 이고 layout profile 이 `mpc` role 의 코어를 돌려준 것 (`mpc_off`) 이면 `on_activate` 가 FAILURE 다
- 스케줄러: `mpc` role 의 설정을 따른다 (D-7a). 제어 PC 에서의 FIFO · OTHER 비교 측정 (G3-J) 은 하지 않았다
- **스케줄러 판정 기준 (D-7a).** 계획기는 RT 루프와 다른 코어에서 SeqLock 으로만 결과를 넘기므로 스케줄링 클래스는 RT 루프 결정성과 무관하다. 영향을 받는 것은 **vision 도착 → plan 게시 지연과 그 꼬리** (commit 시점의 plan 신선도) 뿐이다. plan 의 시각은 절대 steady 시각이라 늦은 plan 이 틀린 시각을 쓰지는 않는다 — 남은 시간만 준다. DDS 수신 → 파싱 → eventfd 는 `nrt_callback_executor` (SCHED_OTHER) 라 FIFO 와 무관하고, FIFO 가 줄이는 것은 깨어남 지연과 선점으로 늘어나는 탐색 꼬리뿐이다. 초기값은 FIFO (`rt_callback` 보다 낮은 우선순위) 이고 코드는 스케줄링 클래스와 무관하게 RT-1~10 을 지키므로 FIFO/OTHER 는 `thread_layout.yaml` 값만 바꾼다. 확정 절차 (G3-J): ① 제어 PC 에 부하 (포구 컨트롤러 + sim 또는 실기 드라이버 + vision) 를 건 채 두 정책을 각각 측정 (수신 → 게시 지연 p50 · p99 · 최대, 예산 초과율, 표본 ≥ 1000), ② 설정값은 증거가 아니므로 각 run 에서 planner 스레드 이름의 모든 TID 의 policy · priority · 논리 CPU · cpuset mask 를 `/proc` 에서 기록하고 기대와 다르면 그 run 은 `NOT_EVALUATED`, ③ 판정은 FIFO 가 p99 지연을 `planner.search.grid.budget_s` 의 10 % 이상 줄이지도 예산 초과율을 줄이지도 못하면 **SCHED_OTHER 로 전환**, ④ 상류 구간 (nrt 수신) 이 지배적이면 수신 경로를 따로 검토한다. 개발 PC 는 PREEMPT_RT 가 아니라 판정은 제어 PC 에서만 한다.
- **`PeriodicRtThread` 의 성질.** 기반 클래스는 진입 시 `ApplyThreadConfigVerbose` 가 **실패해도 무시하고 계속 실행**한다 — 그래서 배치의 증거는 설정값이 아니라 `/proc` 이다. 계획기는 `MPCThread` / `MPCHandlerBase` 가 아니라 `PeriodicRtThread` 의 형제이고 (같은 기반의 네 번째 소비자라 P5 · ARCH-3 을 만족한다), 스레드의 `Pause()` 뒤 한 번 더 게시된 plan 을 RT 가 activation generation 으로 거르는 장치 (D-23) 는 MPC 해에는 없는 것이다.
- **role 공유의 범위.** 계획기가 `mpc` role 을 쓰므로 D-7a 의 `mpc_main` 값 변경은 MPC 에도 적용된다. 둘의 요구가 갈리면 그때 role 을 분리한다 (layout 변경 · 별도 E-7). 이 role 선택은 layout 표 · generator · oracle · 검증기에 변경을 만들지 않는다.
- **복사하지 않는 것.** MPC 스레드의 cross-mode swap 은 phase 전환 때 그 스레드에서 handler 를 새로 만든다 (heap · YAML · try/catch) — invariants.md §RT Path 의 알려진 위반이라 계획기의 템플릿이 아니다. 통계 mutex 와 `fprintf` 도 두지 않는다 (진단은 SPSC).

**한 번 깨어났을 때의 순서** (`PlannerCycle::Run`):

1. RT 상태 (`PlannerRtState`) 읽기 — 이것이 계획할지를 정한다. 리셋 epoch 이 바뀌었으면 시행 상태를 버린다 (탐색의 현재 plan · settle 카운트, `mpc` 는 구간 상태와 segment box 의 구간까지)
2. 모드가 정하는 일 (`ActivityFor` — 모드의 술어이지 enum 순서 비교가 아니다. `mode >= Committed` 같은 나열 순서 의존은 상태를 추가하면 조용히 깨진다):

| RT 모드 | `closed_form` | `mpc` · `mpc_docking` |
|---|---|---|
| TRACKING · APPROACH (탐색) | 아래 "탐색 wake" — APPROACH 에서도 매 wake 탐색하고 §4.7 로 교체를 판정 | RT 가 plan 을 **따르는 중**이면 "따르는 중의 탐색" (아래) — `planner.freeze.t_stop_plan` 까지는 탐색 → 교체 쌍 또는 재계획, 그 뒤에는 재계획만. 아니면 "탐색 wake". 어느 쪽이든 쌍 게시 직후에는 RT 상태가 그 게시를 반영할 때까지 (게시 뒤 3 tick, 또는 보고가 그 plan 을 따를 때까지) 기다린다 |
| COMMITTED · CLOSING (동결) | `monitorOnly()` — $\sigma_\ell$ 기록 (§4.6) | `monitorOnly()` 뒤 구간 재계획 (아직 $t_c$ 앞이라 공을 읽는다) |
| DECEL | 없음 | 포구 뒤 재계획만 |
| 그 밖 (IDLE · ARMED · HOLD · RETREAT · ABORT_SAFE · FAULT) | 없음 | 없음 |

**탐색 wake** (두 planner 공통, 마지막 단계만 다르다):

1. 궤적 스냅샷 읽기 — 지금 activation 의 것이 아니면 끝 (`no_input`). 이어서 공분산 읽기: token 이 궤적과 다르면 한 번 더 읽고, 그래도 다르면 그 조합은 쓰지 않는다 (공분산 모름으로 계획)
2. 탐색 (`GridCatchSearch::Plan`, §4.1 단일 진입 함수): 트랙 epoch 가 막 바뀌었으면 `n_settle` 동안 plan 없음 (§4.4) → vision 샘플 격자의 후보마다 창·작업공간 검사와 사전 점수 → IK 예산 안에서 §4.1 의 순서 → 점수 (§4.10). 예산 `budget_s` 가 다하면 남은 후보를 건너뛰고 지금까지의 최선을 쓴다. `closed_form` 에서 RT 가 plan 을 따르는 중이면 §4.7 이 게시 여부를 정한다
3. 게시 직전 재확인: 궤적 token 이 그대로이고 리셋 epoch · activation 이 그대로여야 한다 — 아니면 버린다 (`superseded`). 교체 규칙이 보류를 정했으면 게시하지 않는다 (`held`). RT 가 구간 계획기의 구간을 따르는 중이면 이 단계와 다음 단계 대신 아래 "따르는 중의 탐색" 이 정한다
4. 게시:
   - **`closed_form`**: `PlanSnapshot` 을 저장한다 — 유효한 plan 이든 "plan 없음" + 사유든
   - **`mpc`**: plan 이 유효하면 첫 구간을 푼다 (`MpcSegmentPlanner::PlanFirst`). 보류되면 plan 도 게시하지 않는다 (`held`). 풀리면 쌍을 다시 확인한다 — 같은 트랙 (같은 스냅샷이 아니다: 첫 풀이가 궤적 주기보다 길 수 있다), 리셋 epoch · activation 그대로, RT 가 그 사이 다른 plan 을 따르지 않음, $t_c-$ 게시 시각 $>T_{freeze}$, 구간이 읽히기 전에 시작하지 않음 — 그리고 **구간을 먼저, plan 을 다음에** 저장한다 (RT 가 한 tick 에 쌍을 판정하므로 그때 구간이 있어야 한다). "plan 없음" 은 `mpc` 에서도 plan 만 게시한다

- **따르는 중의 탐색 (구간 계획기).** APPROACH 에서 RT 가 plan 을 따르는 wake 다. 식과 조건의 전문은 [ball_catching_inverse_dynamics_mpc.md](ball_catching_inverse_dynamics_mpc.md) §17.12 이고, 탐색과 구간 계획기의 종류와 무관하다.
  - **RT 가 교체를 들고 있으면 쉰다.** RT 의 보고가 `plan_pending` 이면 그 wake 는 탐색도 재계획도 하지 않는다 (`held`) — RT 의 슬롯이 차 있어 무엇을 게시해도 받지 않는다.
  - **탐색이 도는 때.** RT 가 따르는 plan 중 **처음 따른 것** (리셋 뒤 — 교체돼도 바뀌지 않는다) 의 $t_c$ 까지 `planner.freeze.t_stop_plan` 보다 많이 남았을 때, 그리고 모드가 APPROACH 일 때만 돈다. 그 밖의 wake 는 재계획뿐이다 (COMMITTED 부터도).
  - **순서는 탐색 → (교체 쌍 또는 재계획) 이다.** 탐색이 wake 의 첫 일이고, RT 가 보고한 구간 (`arm`) 을 받는다.
  - **탐색의 예산에 상한이 걸린다.** 예측 한 주기 안에 탐색과 그 뒤의 재계획이 들어가야 하므로, 탐색은 자기 예산과 `prediction.dt_expected` − 구간 계획기의 `budget.replan_s` 가운데 작은 쪽으로 돈다 (`CatchSearch::Plan` 의 `budget_cap_ns`; 새 키는 없다). `prediction.dt_expected` 가 없거나 그 차가 양수가 아니면 상한은 없다. 출하값에서 `nlp` 탐색은 이 상한 아래 한 wake 에 후보를 하나까지만 풀고, 그 하나는 따르는 plan 의 셀이 필요조건을 지나는 동안 그 셀의 것이다 — 그래서 `nlp` 의 교체는 그 셀이 필요조건에서 빠진 wake 에서만 나온다 ([ball_catching_inverse_dynamics_mpc.md](ball_catching_inverse_dynamics_mpc.md) §17.12). RT 가 plan 을 따르지 않는 wake 의 탐색은 자기 예산 그대로다. 이 상한이 한 주기 안에 묶는 것은 탐색과 그 뒤의 재계획뿐이다 — 교체 쌍으로 가는 wake 는 탐색 위에 첫 풀이를 그 풀이의 예산 (`budget.first_s`) 으로 돌리고, 그 합을 묶는 것은 없다.
  - **교체 쌍.** 탐색의 판정이 교체 (`replaced`) 이고 plan 이 유효하면 새 plan 의 첫 구간을 푼다 (`SegmentPlanner::PlanFirst` — RT 가 보고한 구간 위의 움직이는 상태에서 출발한다) 그리고 첫 쌍과 같은 방식으로 게시한다 (구간 먼저, 같은 `publish_ns`, 새 plan id). 구간 계획기는 따르는 plan 의 구간을 버리지 않고 새 구간을 그 옆에 둔다 — 팔은 전환까지 옛 구간 위에 있다. 새 구간의 node 0 는 두 곳에서 묶인다 — RT 가 전환 tick 에 하는 일 때문이다 (L7 §4.3a): **따르는 plan 의 동결 ($t_c-T_{freeze}$) 앞**이어야 하고 (RT 는 동결 뒤에 교체를 받지 않는다), **새 plan 의 $t_c$ 보다 $T_{freeze}$ 넘게 앞**이어야 한다 (RT 는 전환 tick 에 새 $t_c$ 가 $T_{freeze}$ 안이면 들고 있던 쌍을 버린다). 풀기 전에, 첫 구간이 설 수 있는 가장 이른 node 0 가 이미 둘 가운데 하나를 어기면 풀지 않는다 (그 wake 는 재계획한다).
  - **쌍의 재확인.** 게시 직전에 다음을 모두 확인하고 하나라도 아니면 버린다 (`superseded` — 그 wake 는 재계획도 하지 않는다): 같은 트랙 · 리셋 epoch · activation, RT 가 여전히 **같은 plan** 을 APPROACH 에서 따르고 들고 있는 교체가 없음, 풀이가 출발한 구간이 RT 의 새 보고에서도 그 node 0 의 출처임, 새 $t_c-$ 게시 시각 $>T_{freeze}$, 풀린 구간의 node 0 가 위의 두 경계 안 (따르는 plan 의 동결 앞 · 새 $t_c$ 보다 $T_{freeze}$ 넘게 앞), 구간이 읽히기 전에 시작하지 않음.
  - **재계획.** 그 밖의 모든 경우 (갱신 · 보류 · 후보 없음 · 궤적 없음 · 교체의 첫 구간을 풀지 않았거나 보류) 는 따르는 plan 의 구간을 재계획한다 — 탐색이 시간을 썼으므로 RT 의 보고를 다시 읽어 그것에서 출발한다. 그 wake 의 `outcome` 은 `held` 다 (궤적이 없으면 `no_input`). 보류된 교체의 첫 풀이는 기록의 별도 필드에 남고 (`PlannerCycleRecord::replacement`) `planner_events.csv` 의 `segment_*` 열은 재계획의 것이다. 재계획한 구간은 게시 직전에 RT 의 보고를 한 번 더 읽어 — RT 가 여전히 그 plan 을 따르고, 새 node 0 의 출처가 같은 구간이고, **교체를 들고 있지 않을** 때만 — 저장한다 (탐색이 돌지 않는 wake 의 재계획도 같다). 풀이 사이에 RT 가 쌍을 받았으면 버린다 (`segment_outcome` `superseded`).
  - **RT 가 쌍을 버리면** (전환 게이트 · 동결 · COMMITTED 진입, L7 §4.3a) 보고는 옛 plan · 교체 없음으로 돌아가고 계획기는 그것을 따로 알지 못한다. 다음 wake 의 탐색이 다시 교체를 고르면 새 id 로 쌍을 낸다. 횟수의 제한은 없다 — `t_stop_plan` 이 끝이다.
  - 구간 재계획만 한 wake (탐색이 돌지 않는 wake) 가 idle 로 남는 것 (MD-29) 은 그대로다.
- **동결 중에도 `monitorOnly()` 는 돈다**(§4.6): $\sigma_\ell$ 을 계산해 `planner_events.csv` 에 **기록만** 한다. 이 값이나 σ 성장률·L1 $\bar\nu$ 로 abort 를 판단하지 않는다 (§4.6 경로 2).
- 시행 종료 시 plan 무효화는 L7이 한다(L7 §4.8).
- 후보 시각은 vision 샘플 격자를 그대로 쓴다. `planner.search.grid.slice.dt` 가 vision 간격보다 크면 격자를 솎아 쓴다. 보간해서 후보를 만들지 않는다 — 후보는 표본 자체이므로 후보의 공분산은 그 표본의 것이고 전파가 필요 없다.
- **표본 사이의 시각에서 공분산은 하나의 규칙으로 읽는다.** $t$ 의 공 (`SampleBallNode`) 은 평균을 `SampleAt` 으로, 공분산을 **가장 가까운 표본**의 $6\times6$ 을 등속 전이 $F(t-t_i)$ 로 $F\Sigma_iF^\top$ 전파한 것으로 읽는다. 쓰는 곳은 `GridCatchSearch::Monitor` (동결 뒤 $t_c$), `MakeMpcSegmentBallTarget` ($\Sigma_p$ 는 이 $6\times6$ 의 위치 블록), NLP 탐색 · docking 코어이고, 어느 것도 두 표본을 보간하지 않는다. 예측 밖의 시각은 호출하는 쪽이 거부한다 (`SampleBallNode` 는 `after_horizon` 만 표시한다)
- 구간 재계획의 판정 (어느 격자점에서 다시 푸는가, 게시 조건, RT 의 구간 전환) 은 `mpc` 는 formulation 과 L7, `mpc_docking` 은 formulation §17.12 가 갖고, 코드는 `mpc_segment_planner.hpp` · `mpc_docking_segment_planner.hpp` 의 머리 주석이 갖는다. 구간을 담는 ring (`SegmentRing` — 구간 8 개. 구간을 넣을 때 RT 가 따르는 plan, RT 가 들고 있는 교체, 넣는 구간의 plan 의 것만 남긴다) 은 둘이 공유한다.

### 5.4 방향 속력 (§4.5)

**적용: 공통.** 구현은 `unit_speed.hpp` 의 `UnitSpeedSolver` ($\dot q^u$) 와 `time_feasibility.hpp` 의 `DirectionalSpeedMax` 다. 규범:

- 입력: §4.2 의 $J_5$ (팔 관절 열, 고정 최대 크기), $\hat v$, $\dot q_{\max}$ (계획 한계 $\eta_v\dot q_{\max}$). 출력: $v_{dir,\max}$ 와 무효 플래그
- 분자는 투영 $\max(0,\hat v^\top J_p\dot q^u)$, 분모 $\max_i|\dot q^u_i|/\dot q_{\max,i}$ (§4.5)
- $\dot q_{\max,i}\le0$·NaN 은 플래그 + 0, 분모 0 은 0 (보수적)
- 할당 0, `noexcept` (계획기 스레드가 RT-1~10 준수)
- 감쇠 $\lambda$ 는 오프라인 지도와 **같은 키** (`planner.search.grid.gamma.unit_speed_damping`) 에서 읽는다

## 6. YAML 파라미터

로딩은 컨트롤러 `LoadConfig(YAML)` → `ParsePlannerParams` · `ParseCatchPoseIkParams` · `ParseCatchingParams` (on_configure, non-RT), runtime 변경 gain 만 `declare_parameter` (L0 §5.3). **키는 기능이 갖는다.** 한 기능 (탐색 `grid` · `nlp`, 구간 `closed_form` · `mpc` · `mpc_docking`) 이 쓰는 설계값은 전부 그 기능의 파일에 있고, 두 기능이 같은 수를 읽으면 각자 key 를 갖는다. **요구는 선택을 따른다** — 선택된 기능의 키만 읽고 요구하며 (`ParsePlannerParams` 는 `grid` 일 때만 `planner.search.grid` 를, `mpc` 일 때만 `planner.segment.mpc` 를 연다), 선택되지 않은 기능의 조각은 없거나 틀려도 컨트롤러가 알아채지 않는다. 선택된 기능의 조각이 없으면 park 하고 로그가 없는 키를 적는다. 선택 키를 모르는 철자로 적으면 파서가 거부한다. 다른 층의 사실 (`joint_cmd.*` · `robot.hand.*` · `planner.freeze.T_freeze` …) 은 한 곳에 그대로 둔다.

**복사본은 일치 검사를 받는다.** 탐색의 `planner.search.grid.reference.{v_max, omega, zeta, a_max}` · `stop.a_dec` 는 `reference.*` · `supervisor.decel.a_dec` 의 복사본이고 원본의 검증 규칙 (`ValidateCatchingParams`) 을 따른다. 이 검사는 `grid` 탐색에만 있다. 계획기가 켜져 있고 복사본이 원본과 다르면 `planner.segment.mode: closed_form` 은 park 하고 (`CatchingParkReason::kSearchCopyDiffers` — 탐색이 팔이 따르지 않는 움직임으로 후보를 매기게 된다) 구간 계획기 (`mpc` · `mpc_docking`) 는 WARN 만 한다. `planner.search.grid.gamma.eta_v` 와 `planner.segment.mpc.eta_v` 가 다를 때도 `mpc` 에서 WARN 이다. 옮긴 옛 key 18 개 (아래 "옛 key") 가 있으면 park 한다 (`kRemovedKey`).

**값 · 범위 · 근거는 YAML 과 파서가 갖는다.** 키는 로봇 config (`integrated_bringup/config/<robot>/controllers/`) 의 주 파일과 조각 다섯에 나뉘어 있고 전부 `demo_catching_controller.catching` 아래의 전체 경로를 갖는다. 어느 키가 어느 planner 의 것인지는 각 파일의 머리 주석이 정확하다.

| 자리 (표의 약어) | 파일 | 무엇 |
|---|---|---|
| 주 | `demo_catching_controller.yaml` | 스레드 · 대기 자세 · 동결 · segment mode 의 선택 (`planner.segment.mode`) · 관절 가속 box (`robot.arm.qdd_*`) |
| 탐색 | `catching/search_grid.yaml` | 탐색 `planner.search.grid.*` — **두 segment mode 공통**. 탐색이 읽는 `reference` · `stop.a_dec` 의 복사본과 `switch.*` 도 여기다 |
| CF | `catching/planner_closed_form.yaml` | `closed_form` 의 법칙 — `reference.*` · `supervisor.decel.a_dec`. 탐색은 읽지 않고 자기 복사본을 읽는다 |
| MPC | `catching/segment_mpc.yaml` | `mpc` 구간 계획기 `planner.segment.mpc.*` — `planner.segment.mode: mpc` 일 때만 읽는다 |
| NLP | `catching/search_nlp.yaml` | `nlp` 탐색 `planner.search.nlp.*` — `planner.search.mode: nlp` 일 때만 읽는다 (구간 계획기 `mpc` 또는 `mpc_docking` 과 `robot.hand.docking` 이 함께 필요하다) |
| DOCK | `catching/segment_mpc_docking.yaml` | `mpc_docking` 구간 계획기 `planner.segment.mpc_docking.*` — `planner.segment.mode: mpc_docking` 일 때만 읽는다 |

**mpc 구간 계획기의 키 (`planner.segment.mpc.*`) 는 `catching/segment_mpc.yaml` 과 formulation 이 갖는다** — 이 표의 범위가 아니다 (정의는 `planner_params.hpp` 의 `MpcSegmentPlannerParams`). 탐색의 값과 맞물리는 `eta_v` · `v_eps` 만 아래 표에 둔다.

| 키 | 뜻 | 단위 | 자리 |
|---|---|---|---|
| `planner.enabled` | 계획기 스레드를 띄운다. `diagnostic.oracle_plan.enabled` 와 동시 true 면 park — plan box 의 writer 는 하나다 | – | 주 |
| `planner.segment.mode` | 팔이 APPROACH 부터 정지 끝까지 따르는 것 (`closed_form` \| `mpc` \| `mpc_docking`, 코드 기본 `closed_form`, 출하 `mpc`). `closed_form` 은 segment planner 가 아니다 — RT tick 이 기준을 직접 만든다. 읽기 전용 mirror 가 같은 이름으로 있다 | – | 주 |
| `planner.search.mode` | 탐색 (`grid` \| `nlp`, 코드 기본 · 출하 `grid`). `nlp` × `closed_form` 은 park (`kSearchSegmentCombination`, 로그가 두 키를 적는다). `nlp` 와 `mpc_docking` 은 `robot.hand.docking` 을 요구하고 (없으면 `kMpcDockingInvalid`) sim 전용이다 (#654) — 실기 configuration 이 둘 중 하나를 고르면 park 한다 (`kMpcDockingInvalid`) | – | 주 |
| `planner.wake_timeout_s` | 새 궤적이 없어도 깨어나는 상한. 스레드의 주기이기도 하다 | s | 주 |
| `planner.search.grid.budget_s` | 한 사이클의 탐색 계산 예산 (§4.1 R-2). 구간 계획기 아래에서 RT 가 plan 을 따르는 wake 에는 상한이 더 걸린다 (§5.3) | s | 탐색 |
| `planner.sub_model` | 계획기 모델 (R-3): 로봇 config `urdf.sub_models` 의 이름 — arm root → catch frame 부모. 오프라인 지도도 같은 항목을 이름으로 쓴다 (G3-I). 결정값이라 비었으면 park | – | 주 |
| `planner.search.grid.max_ik` | 한 cycle 의 IK 후보 수 (사전 점수 상위, R-2) | – | 탐색 |
| `planner.provisional` | `planner` 블록 전체가 provisional — sim 경고, 실기 park. `planner` 의 provisional 플래그는 이것 하나다 (`planner.search.grid.hand.provisional` 같은 키는 없다) | – | 주 |
| `planner.wait_pose` | IK seed 이자 대기 자세 (arm 관절 순서, 길이 = arm dof, 관절 한계 안) | rad | 주 |
| `planner.wait_pose_source` | 대기 자세의 출처: `yaml` = 위 목록, `current` = 이 컨트롤러가 활성화된 뒤 **첫 팔 판독 가능 tick** 의 $q_{meas}$ (팔이 정지한 tick 에 채택). 채택 자세는 homing 목표 · `ARMED` 도착 판정 · 계획기 IK seed (`PlannerRtState.wait_pose`) 에 같이 쓰인다. 채택이 거부되면 `yaml` 값이 유지된다 — 계획기는 매 cycle YAML seed 에서 시작해 채택 자세를 덮으므로 앞선 activation 의 채택 자세를 이어받지 않는다. 채택 · 거부의 조건은 L7 §4.1. read-only 미러 `planner.wait_pose` 는 **YAML 값**이고, `workspace.catch_box` 는 자세를 따라가지 않는다 | – | 주 |
| `planner.search.grid.slice.dt` | 후보 간격 — vision 샘플 간격 (`prediction.dt_expected`) 의 정수배. 격자를 솎을 뿐 보간하지 않는다 | s | 탐색 |
| `planner.search.grid.slice.t_lead_min` | 후보 창의 하한. 없으면 `planner.freeze.T_freeze` 다 (출하 YAML 은 적지 않는다) | s | 탐색 |
| `planner.search.grid.slice.t_max` | 후보 창의 상한 — vision 지평에서 여유를 뺀 값 이하 | s | 탐색 |
| `planner.search.grid.n_settle` | §4.4 트랙 epoch 변경 후 건너뛰는 메시지 수 | – | 탐색 |
| `planner.search.grid.time.margin` | §4.3 $T_{margin}$. §4.11 의 commit 선행 순위 게이트도 같은 값을 쓴다 | s | 탐색 |
| `planner.search.grid.unc.kappa_sigma` | §4.4 $\kappa_\sigma$ | – | 탐색 |
| `planner.search.grid.gamma.eta_v` | D-9 의 $\eta_v$ — `ComputeGammaWindow` · 관절 속도 계획 한계 · rollout 수락이 같은 값을 쓴다 (§4.5, §4.8). `mpc` 구간 계획기는 자기 `planner.segment.mpc.eta_v` 를 갖는다 | – | 탐색 |
| `planner.segment.mpc.eta_v` | mpc 구간 계획기의 속도 행과 RT 의 구간 전환 게이트가 읽는 $\eta_v$ (0 < η_v ≤ 1 — `mpc` 일 때만 범위를 검사하고 < 1 은 MD-34 전제). 탐색의 `planner.search.grid.gamma.eta_v` 와 다르면 `mpc` 에서 WARN | – | MPC |
| `planner.segment.mpc.v_eps` | mpc 구간 계획기의 속력 하한 (> 0, 파서가 검사). 탐색 IK 의 `planner.search.grid.ik.v_eps` 와 별개다 | m/s | MPC |
| `planner.search.grid.gamma.margin` | §4.5 `MaxCatchableSpeed` 경계 여유 | m/s | 탐색 |
| `planner.search.grid.gamma.unit_speed_damping` | §4.5 $v_{dir,\max}$ 의 DLS 단위속도의 $\lambda$. 오프라인 지도 `catch_gate_map` 이 **같은 키** 를 profile 에서 읽는다 | – | 탐색 |
| `planner.search.grid.gamma.grid` | §4.8 $\gamma_f$ 후보 | – | 탐색 |
| `planner.search.grid.gamma.window_grid` | §4.8 $T_w$ 후보 | s | 탐색 |
| `planner.search.grid.gamma.eta_a` | §4.8 요구 가속도의 여유율 (`planner.search.grid.reference.a_max` 대비) | – | 탐색 |
| `planner.search.grid.gamma.eps_term` | §4.8 $t_c$ 의 잔여 오차 한계. §4.7 의 갱신 (`refreshed`) 임계이기도 하다 | m | 탐색 |
| `planner.search.grid.rollout.dt_coarse` | §4.8 rollout 의 거친 단계 간격. 확인 단계는 제어 주기 | s | 탐색 |
| `planner.search.grid.budget.n_sigma` | §4.6 $n_\sigma$ | – | 탐색 |
| `planner.search.grid.budget.sigma_trk` | §4.6 $\sigma_{trk}$ — L5 실측 (실기 식별 전에는 0) | m | 탐색 |
| `planner.search.grid.budget.clock_err` | §4.6 $\delta$ — 인프라 실측 (실기 식별 전에는 0) | s | 탐색 |
| `planner.search.grid.score.w_sigma`, `w_t`, `w_q`, `w_late`, `w_gamma` | §4.10 의 가중. `w_gamma`를 크게 잡으면 사전식 선택과 같아진다 | – | 탐색 |
| `planner.search.grid.score.penalty` | 실패한 **순위** 게이트 하나당 점수에 더한다. 연속 항을 압도해야 한다 (§4.1) | – | 탐색 |
| `planner.search.grid.workspace.catch_box` | 모델 world 축정렬 상자 `{min: [x,y,z], max: [x,y,z]}`. 판정 게이트 — $p_c$ 와 $p_{stop}$ 이 모두 안에 있어야 한다 (TBD-BALL-02). 결정값이라 없으면 park | m | 탐색 |
| `planner.search.grid.hand.d_eff` | **시각 발동 fly-in 허용 상대속도 × $T_{close,tot}$** (§4.5, L6 §4.5). 포켓 깊이가 아니다. 결정값이라 비었거나 TBD 면 park | m | 탐색 |
| `planner.search.grid.hand.r_cap` | 측면 허용량 (L6 §4.5). 공 중심 좌표계라 공 반지름이 이미 포함돼 있다. 불확실성 · 오차 예산이 쓴다 | m | 탐색 |
| `planner.search.grid.ik.max_iter` | IK 반복 상한 $N_{IK}$ | – | 탐색 |
| `planner.search.grid.ik.eps_pos` | 수락: 위치 오차 $\epsilon_p$ | m | 탐색 |
| `planner.search.grid.ik.alpha_max` | 수락: 손 형상 허용 콘 ($\theta\le\alpha_{\max}$) | rad | 탐색 |
| `planner.search.grid.ik.rho` | §4.2 위치/회전 스케일 정합 (특성길이). **$J$ 와 잔차 양쪽에 가중** | m/rad | 탐색 |
| `planner.search.grid.ik.sigma0` | 감쇠 shell 진입 σ_min — **영공간 투영 $N$ 만** 파라미터화한다 (과제 스텝은 QP) | – | 탐색 |
| `planner.search.grid.ik.lambda_max` | 최대 감쇠. $N$ 전용 | – | 탐색 |
| `planner.search.grid.ik.dq_step_max` | 반복당 $\Vert\dot q_d\Vert_\infty$ 상한 (방향 유지 축소, $\Delta t=1$) | rad | 탐색 |
| `planner.search.grid.ik.mu` | §4.2 과제 QP 정칙화. $J^\top J$ 가 rank ≤ 5 라 필수다. 절벽이 있으니 값을 바꾸면 재측정한다 | – | 탐색 |
| `planner.search.grid.ik.qp_eps_abs` | 과제 QP 절대 허용오차. TSID tick 의 값을 그대로 쓰면 IK 잔차에 solver 바닥이 생긴다 | – | 탐색 |
| `planner.search.grid.ik.qp_max_iter` | 과제 QP 반복 상한 | – | 탐색 |
| `planner.search.grid.ik.k_null` | §4.2 영공간 자세 과제 $K_n$. $q_n$ = **한계로 clamp 한** seed. `eps_pos` 와 함께 고른다 (2차 잔차) | 1/step | 탐색 |
| `planner.search.grid.ik.k_manip` | §4.2 영공간 $\log w_5$ 상승 이득 $k_w$. 0 이면 seed 가 roll 을 정한다 | – | 탐색 |
| `planner.search.grid.ik.manip_grad_tol` | 상승 종료 판정 $\Vert N\nabla\log w_5\Vert$ | – | 탐색 |
| `planner.search.grid.ik.fd_step` | $\nabla\log w_5$ 중심차분의 간격 $h$ | rad | 탐색 |
| `planner.search.grid.ik.v_eps` | §4.2 NUM-7 속력 하한 — 미만이면 clamp 가 아니라 탈락 | m/s | 탐색 |
| `planner.search.grid.catchability.definition` | §4.2 게이트에 쓸 정의 (`arm_5row` \| `arm_6row`). $w_5$·$w_6$ 는 정의와 무관하게 둘 다 기록 | – | 탐색 |
| `planner.search.grid.catchability.manipulability_min.arm_5row` / `.arm_6row` | §4.2 D-18 정의별 하한 (차원이 달라 따로 둔다, C-3). 로봇별 값이다. TBD 인 정의로 판정하면 fail closed. 오프라인 지도 도구와 **같은 키** | – | 탐색 |
| `planner.search.grid.catchability.manipulability_min.provisional` | 위 하한이 provisional — 실기 구성 차단 (L0 §5.3) | – | 탐색 |
| `planner.freeze.T_freeze` | §4.11 의 동결 창. 하한은 §4.11 의 코드 하한. 결정값이라 없으면 park | s | 주 |
| `planner.freeze.t_stop_plan` | 구간 계획기에서 RT 가 plan 을 따르는 동안 탐색이 도는 마지막 시점 — 처음 따른 plan 의 $t_c$ 까지 이만큼 남으면 탐색을 멈춘다 (§5.3). 없으면 `T_freeze`, `T_freeze` 보다 작으면 park (`kPlannerUnset`) | s | 주 |
| `robot.arm.qdd_max` | §4.3 의 관절 가속 box (arm 관절 순서). 탐색의 도달시간, QP 비의존 정지 램프, homing 램프가 읽는다 (CLIK 은 읽지 않는다). 여러 기능이 읽으므로 주 파일에 있다 | rad/s² | 주 |
| `robot.arm.qdd_provisional` | 위 box 가 실기에 승인됐는가 — true 면 실기 구성 park. `qdd_max` 를 덮는 쪽이 함께 적는다 | – | 주 |
| `planner.search.grid.reference.omega` · `zeta` · `v_max` · `a_max` | 탐색이 rollout (§4.8) 과 γ 창 (§4.5) 에서 읽는 기준 — `reference.*` (L4 §6) 의 복사본. 원본의 검증 규칙을 따르고 `closed_form` 에서 원본과 다르면 park | – | 탐색 |
| `planner.search.grid.stop.a_dec` | §4.9 정지점의 감속 — `supervisor.decel.a_dec` 의 복사본. 같은 규칙 ($0<a_{dec}\le a_{\max}$) | m/s² | 탐색 |
| `reference.omega` · `zeta` · `v_max` · `a_max` | L4 §6. `closed_form` 의 기준. 탐색은 읽지 않는다 (위 복사본) | – | CF |
| `supervisor.decel.a_dec` | L7 §6 의 감속 법칙. 탐색의 정지점은 위 복사본을 읽는다 | m/s² | CF |
| `planner.search.grid.switch.delta_J` | §4.7 규칙 1 의 $\Delta_J$ — `closed_form` 에서는 plan 의 재게시를, 구간 계획기에서는 교체 쌍의 게시를 정한다 (§5.3) | – | 탐색 |
| `planner.search.grid.switch.eta_jump` | §4.7 규칙 2: 교체가 $u_{des}$ 에 넣는 계단 ≤ `eta_jump` × `reference.a_max` — **closed_form 전용** (구간 계획기에서는 보지 않는다, §4.7) | – | 탐색 |
| `planner.search.grid.switch.samples` | §4.7 규칙 2 의 계단 상한을 평가하는 순간의 수 (2 ~ `kSwitchSamplesMax`) — **closed_form 전용** | – | 탐색 |

옛 key: 아래 18 개가 병합된 트리에 있으면 컨트롤러는 park 하고 (`kRemovedKey`) ERROR 가 옛 경로와 새 경로를 적는다 — `supervisor.decel.mode` → `planner.segment.mode`, `supervisor.decel.switch_margin` → `planner.segment.mpc.switch_margin`, `planner.decel_mpc` → `planner.segment.mpc`, 그리고 `planner` 바로 아래의 탐색 key (`budget_s` · `max_ik` · `n_settle` · `slice` · `time` · `unc` · `gamma` · `rollout` · `budget` · `score` · `workspace` · `hand` · `ik` · `catchability` · `switch`) → `planner.search.grid.<같은 이름>`. `supervisor.decel` 은 leaf 마다 판정해 `a_dec` 는 남는다 (`rtc::catching::kRenamedCatchingKeys`).

읽지 않는 키: `planner.search.grid.ik.lambda` · `planner.search.grid.ik.manip_min` 은 파서가 읽지 않고 보고만 한다 (`CatchPoseIkRetiredKeys` — 감쇠는 `sigma0` · `lambda_max`, 하한은 `planner.search.grid.catchability.manipulability_min`). `planner.search.grid.switch.e_jump_max` · `ed_jump_max` 는 파서가 **거부**한다 (`eta_jump` 가 그 자리다). 그 밖의 모르는 `planner.search.grid.ik.*` 키도 거부한다.

## 7. 단위 기술 구현 순서

구현 순서는 이 문서가 갖지 않는다 — 구현은 끝났다. 단위와 코드의 대응만 남긴다.

| 단위 | 코드 |
|---|---|
| 도달시간 · γ 창 · 정지점 · 오차 예산 | `time_feasibility.hpp`, `rank_gates.hpp` |
| 포구 자세 IK + manipulability 게이트 | `catch_pose_ik.hpp` (+ `catch_pose_ik_params.hpp`) |
| 방향 속력 | `unit_speed.hpp` |
| γ rollout | `gamma_rollout.hpp` |
| 탐색 · 선택 · 히스테리시스 | `grid_catch_search.hpp` |
| 후보별 팔 궤적 풀이로 고르는 탐색 (꽂는 경로 없음) | `nlp_catch_search.hpp`, `nlp_catch_screening.hpp` |
| 탐색 · 구간 계획기의 추상 interface | `catch_search.hpp`, `segment_planner.hpp` |
| 탐색 wake 의 기록 (`SearchStats`) | `search_stats.hpp` |
| 한 번의 wake · 게시 | `planner_cycle.hpp` |
| 계획기 스레드 | `integrated_bringup` 의 `planner_thread.hpp` |

## 8. 디버깅 방법

- 계획마다 기록 (`planner_events.csv`): 후보 수, 게이트별 탈락 수(히스토그램), 선택 후보의 $(t_c,\gamma_f,T_w)$ 와 γ 창 ($\gamma_{\min}$ · $\gamma_{\max}$ · $v_{dir,\max}$ · $\Vert v\Vert_{\max}$), 순위 게이트 비트마스크, 실행시간, 교체 여부와 사유. 구간 계획기 (`mpc` · `mpc_docking`) 는 구간 풀이의 결과를 `segment_*` 열에 더한다. `nlp` 탐색은 자기 기록을 `nlp_*` 열에 낸다 — wake 의 사유 (plan 을 못 냈으면 가장 멀리 간 후보의 사유), 후보 수 (격자 · screening 통과 · 푼 것 · 유효), 검사별로 탈락한 후보 수, 고른 후보의 비용 ($J^\star$ · $\Phi$). `mpc_docking` 은 iterate 가 있는 풀이마다 코어의 기록을 더한다 — QP 의 수와 반복, 단계별 시간, KKT 잔차, 행 그룹별 위반과 불능으로 끝났을 때의 그룹, 포구 노드의 $c_N$ · $\sigma_s$ · $\sigma_t$ 와 lateral · timing 행의 부호 있는 여유 (음수가 위반). 교체를 시도한 wake 는 그 시도가 어디서 끝났는지를 `replace_step` 에 남긴다 (풀지 않음 둘 · 보류 · re-check 탈락 · 게시).
- **기한에 끊긴 풀이의 시간은 풀이 시간이 아니다.** 코어가 기한에 끊은 풀이 (`segment_core_reason` 이 `deadline`) 의 `segment_solve_us` 는 끊긴 시각이고, 풀이에 걸리는 시간은 그 이상이라는 것만 안다. outcome `budget` 은 끊겼다는 뜻이 아니다 — 기한이 없는 코어 (`mpc`) 나 다른 사유로 끝난 풀이가 예산을 넘긴 것도 `budget` 이고 그 시간은 끝까지 푼 시간이다. 도구 (`rtc_tools.analysis.planner_solves`) 는 둘을 나눠 센다.
- "항상 탈락": 게이트별 탈락 히스토그램에서 첫 번째 병목을 찾는다. γ 창이 원인이면 $d_{eff}$, $T_{close}$, $v_{dir,\max}$ 값을 먼저 의심한다.
- 포구점이 자주 바뀐다 (`closed_form`): `delta_J`, 점프 한계, 예측 품질(L2 `lastJump`)을 확인한다.
- IK 수렴 실패: catch frame 축 정의 (D-17 YAML), `alpha_max`, seed(wait_pose)와 후보 자세의 거리를 확인한다.
- manipulability 탈락이 지배적: $w_5$ 분포와 오프라인 지도 결과를 대조한다. 지도와 런타임이 다른 함수·키·seed 를 쓰고 있지 않은지 먼저 본다. arm base frame 이 로봇 config 의 CLIK `base_frame` 인지 확인한다 (ur5e_p1b `base` vs `base_link` 180° — §4.2 의 frame 규약).
- `mpc` 에서 plan 이 유효한데 게시되지 않는다 (`held`): 첫 구간이 보류된 것이다 — `segment_outcome` 을 본다.
- 구간 계획기에서 RT 가 plan 을 따르는 동안 탐색이 교체를 판정했는데 (`decision` `replaced`) `outcome` 이 `held` 다: 교체의 첫 구간이 따르는 plan 의 동결 앞에 설 수 없었거나 그 풀이가 보류된 것이다 (그 행의 `segment_*` 는 대신 돈 재계획의 것이다). `superseded` 면 쌍의 재확인이 버렸다 (§5.3). 게시됐는데 (`published`) plan 이 바뀌지 않았으면 RT 쪽이다 — L7 §8.
- 시뮬레이션 시각화(RViz): vision 예측 궤적(시각화용 `nav_msgs/Path` 로 재발행), 후보 점(색 = 탈락 사유), 선택된 $p_c$와 $a_d$, $p_{stop}$.

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G3-A | `TMin` 고정 테이블 일치 (< 1e-9, 스크립트 기준값 대비) + `w0_clamped`·한계 무효 플래그 시 후보 탈락 | `[SIM-ANY]` |
| G3-B | γ 창 sanity ([R1] 3 cm / 6 m/s → 5 ms 통과, 6 ms 탈락), `v_tcp_max` = $\eta_v$`reference.v_max` 구속 (D-9), §4.8 표 재현(±5%), 방향 속력 투영·0 가드 | `[SIM-ANY]` |
| G3-C | 합성 투척 1000회: 계획 실행시간 99% < `budget_s`, 탈락 사유 분포 기록 | `[SIM-ANY]` |
| G3-D | `iiwa7_leap` 시뮬레이션에서 plan 유효율, 교체 빈도 기록 | `[SIM-ANY]` |
| G3-E | `ur5e_p1b` 시뮬레이션에서 선택된 plan의 L4/L5 실행 시 포화 0, 한계 활성 비율, **포화 발생 빈도** 기록 | `[SIM-P1B]` |
| G3-F | 실측 $T_{close}$, $T_{arm}$, $\sigma_{trk}$ 반영 후 γ 창·오차 예산 재산정. $\Vert v\Vert_{\max}$(§4.5)가 목표 투척 속도를 덮는지 확인 | `[HW-P1B]` |
| G3-G | IK 수렴률: 합성 후보 1000개에서 `max_iter` 내 수락 비율과 실패 시 잔차 분포 기록 (§4.2는 Gauss-Newton이 아니므로 수렴 보장이 없다) | `[SIM-ANY]` |
| G3-H | 오차 예산 모델 검증: L8에서 §4.6 예측 간극 분포와 실제 간극 분포 비교 (직교 분해 식이 맞는지) — 판정은 L8 의 G8-C2 (L8 §9.1) | `[SIM-P1B]` |
| G3-I | catchability 게이트 (D-18): $w_5$ 가 유한차분·해석 대조와 일치, threshold 미만 후보 탈락 + 사유 코드, 전부 탈락 시 plan 없음. 오프라인 지도 도구와 런타임이 같은 입력에서 같은 판정을 내는 동치성 | `[SIM-ANY]` |
| G3-J | D-7a 측정: 제어 PC 부하 상태에서 FIFO·OTHER 각각 수신 → plan 게시 지연 p50·p99·최대, 예산 초과율 (≥ 1000 시행) → §5.3 "스케줄러" 의 판정 기준으로 정책 확정. 판정은 제어 PC 에서만 — dev PC 결과는 `NOT_EVALUATED(제어 PC)` (PREEMPT_RT 아님). 각 run 은 planner 스레드 이름에 맞는 모든 TID 의 실제 policy·priority·논리 CPU·cpuset mask 를 `verify_rt_runtime.sh` 로 기록하고, `{snapshot_sequence, recv_steady_ns, wake_ns, publish_ns}` 이벤트 레코드를 SPSC 로 남긴다 | `[SIM-ANY]` |
| G3-K | RT 할당 게이트: 계획기 스레드 한 사이클(탐색·IK·rollout·게시)이 `ScopedAllocGate`·`ScopedNoMalloc` 아래 할당 0, `noexcept`, 로깅 없음 (진단은 SPSC) | `[SIM-ANY]` |
| G3-L | token·race: eventfd coalescing, 계산 중 새 스냅샷 도착, 같은 generation 의 옛 plan 게시, deactivate·Pause race 에서 대체된 plan 소비 0 (D-22, D-23) | `[SIM-ANY]` |

G3-D 의 "교체 빈도" 와 G3-E 의 "L4/L5 실행 시 포화" 는 `closed_form` 의 양이다 (`mpc` 의 교체는 교체 쌍이고 `catching_diag.csv` 의 `segment_event` 로 센다 — L7 §8; RT 는 L4 기준을 돌리지 않는다).

## 10. 미확정 항목

- TBD-HAND-01, TBD-HAND-04 (투척 보정 — 실기), TBD-BALL-02 (`planner.search.grid.workspace.catch_box`), TBD-VIS-04
- provisional 로 남은 값: `planner.search.grid.ik.alpha_max`, `planner.freeze.T_freeze`, `planner.search.grid.catchability.manipulability_min`, `planner.search.grid.ik` 의 `sigma0` · `lambda_max` · `dq_step_max` · `k_manip` · `manip_grad_tol` · `mu` · `qp_eps_abs`, 점수 가중치, `robot.arm.qdd_max` (실기 envelope)
- `planner.search.grid.budget.sigma_trk` · `clock_err` (실기 식별), 충격량 임계 (TBD-IMP-01, L7 §4.7)
- 계획기 스레드의 스케줄러 정책 판정 (G3-J — 제어 PC)

---

## 부록 A. [R1]식 SQP (`iiwa7_leap` 확장용, 선택)

채택하지 않았다 — 탐색은 1차원 시간 탐색 + IK (§4.1) 이고, $(q,t)$ 를 함께 푸는 SQP 는 구현에 없다.
