# L2 — Prediction: 궤적 샘플러 (vision 예측의 시각 정렬·보간)

이 문서는 현재 구현의 예측 샘플러 층 (공용 궤적 타입 · 5차 Hermite 샘플러 · 지평 감시) 을 표현한다.

- 배치 `[확정 D-1]`: rtc_controllers 의 `catching` 하위 디렉토리 (namespace `rtc::catching`, ROS 비의존 순수 코드)
- 구성: 공용 궤적 타입 (스냅샷 POD, `trajectory.hpp`), 샘플러 (`traj_sampler.hpp`)
- 의존: L0 (시간 타입·POD 규칙)

---

## 1. 범위 / 비범위

vision 노드가 이미 예측 궤적을 발행하고(마스터 §5, D-4), 제어 PC는 그것을 **재전파하지 않고 그대로 신뢰**한다(마스터 §5.2).

범위:
1. **공용 궤적 타입** — L1 파서가 채우고 RT·계획기가 읽는 스냅샷 POD. L1 이 L2 타입에 의존하던 역전을 이 타입을 공용으로 두어 해소한다.
2. vision이 준 $(p,v,a)$ 샘플 열을 임의의 제어 시각으로 **보간**한다 (RT 루프, `control_rate` 100–5000 Hz).
3. 지평 밖 요청을 감지하고 외삽 플래그를 올린다.
4. 샘플 열의 형식·일관성을 검사한다.

비범위:
- 공 상태 추정과 궤적 **예측** (vision 노드).
- 제어 PC 자체 동역학 전파 (`RtBallPropagator`, `rollout()`, `propagateWithCov()` 같은 것은 없다).
- 공분산 전파·보관. vision이 점마다 6×6을 주고, 공분산은 계획기 버퍼에만 둔다(A-3, §4.5).
- 포구점 선택(L3).

## 2. 코드 확인 게이트

본 layer에 직접 걸리는 항목:

| ID | 확인 항목 | 확인된 사실 |
|---|---|---|
| G2-1 | 발행 주기, $N$ 범위, 지평 길이 → `kCap`(컴파일타임)·런타임 점 수 상한과 L3 슬라이스 범위 | `[확정 D-15]` — vision 사양은 **제어기가 요구를 정하고** sim profile 을 맞춘다. 예측점은 `step, 2·step, …, horizon` 이라 t = 0 이 없다. `kCap` 은 궤적 타입이 단독 소유하는 컴파일 상수이고 (provisional) 구현의 런타임 상한은 `kCap` 이다 — `n_max` 키는 없다 (§5.1). 점 수 요구는 목표 분포의 지평 요구에서 산출하며 (D-15, `ceil(H_req/step)`), 요구가 `kCap` 을 넘으면 `kCap` 을 올리고 L0 · L1 게이트를 재실행한다. 발행은 입력 step 마다 나오므로 입력이 드롭되면 발행이 얇아진다 (TBD-VIS-04) |
| G2-2 | 점 시각 필드 타입·기준 → 시각 정렬 식 | `horizon_ns` UINT32 (`header.stamp` 기준 상대 ns). L1 이 수신 시 절대 `BallTime` 으로 변환한다(D-2, L1 §4.1). 샘플러는 절대 시각만 받는다 |
| G2-3 | `ax,ay,az`가 상수 $g$인지 항력 포함 총 가속도인지 | profile 이 정한다 — 출하 sim profile 은 이차 항력 모델이라 **그 점의 총 가속도 $g - k\lVert v\rVert v$**, 항력 절이 없는 profile 은 상수 $g$ |
| G2-4 | 공분산을 RT까지 넘길지 | `[확정 A-3]` — RT 스냅샷에서 분리, 계획기 버퍼에만 |
| G2-5 | 바닥 높이·작업셀 경계의 `W` 좌표 | 열림 (TBD-WS-01). 포구점의 위치를 제한하는 것은 없다 — 탐색은 IK 가 닿는 점을 어디든 후보로 삼는다 (L3 의 L3 §4.9). 바닥 높이 · 작업셀 경계 키는 구현하지 않았다 |

## 3. 참고자료

5차 Hermite 보간은 수치해석 표준 내용(등급 a). [R8]은 L1·L3의 공분산 **사용**에만 관련되고, 전파에는 관련되지 않는다.

## 4. 수학적 이론

### 4.1 왜 재전파하지 않는가 `[확정]`

vision의 예측기와 제어 PC가 각자 전파하면 두 모델이 어긋날 때 예측이 갈라진다. 어긋남은 항력계수, 중력 벡터, 적분 스텝, 추정 시점 어디서든 생긴다. 한쪽을 단일 진리원으로 두는 편이 낫고, 예측기를 가진 쪽은 vision이다.

부수 효과로 다음이 실시간 경로에서 사라진다.

- L0의 이차 항력 모델과 RK4 (→ test fixture 전용)
- $k$ 최소제곱 식별 (→ fixture 전용)
- 변분방정식·상태전이행렬 (`Rk4WithStm`)
- 공정잡음 $Q$ 설정과 vision과의 값 합의

남는 것은 **보간**뿐이다.

### 4.2 5차 Hermite 보간 `[논문 외 유도]`

vision 샘플 간격(`prediction.dt_expected`)은 RT 틱 $h$ = `dt` (0.2–10 ms)보다 훨씬 길다 — 500 Hz 면 구간 하나에 25 틱이 들어가므로 보간 방식이 중요하다.

각 샘플이 $(p_j,v_j,a_j)$ 를 모두 주므로, 구간 $[t_j,t_{j+1}]$ ($h_j=t_{j+1}-t_j$, $s=(t-t_j)/h_j$)에서 **양 끝의 위치·속도·가속도를 모두 맞추는** 5차 Hermite를 쓴다.

$$p(s)=H_0p_j+H_1h_j\,v_j+H_2h_j^2a_j+H_3p_{j+1}+H_4h_j\,v_{j+1}+H_5h_j^2a_{j+1}$$

$$
\begin{aligned}
H_0&=1-10s^3+15s^4-6s^5, & H_1&=s-6s^3+8s^4-3s^5, & H_2&=\tfrac12s^2-\tfrac32s^3+\tfrac32s^4-\tfrac12s^5\\
H_3&=10s^3-15s^4+6s^5, & H_4&=-4s^3+7s^4-3s^5, & H_5&=\tfrac12s^3-s^4+\tfrac12s^5
\end{aligned}
$$

속도·가속도는 $s$로 미분해 $h_j$, $h_j^2$로 나눈다(`traj_sampler.hpp` 의 `Hermite5`, `Interpolate`).

**왜 $C^2$인가.** 샘플러는 양 끝 $a$ 를 맞추므로 경계에서 $C^2$ 다. 샘플 경계에서 $a$ 가 튀는 보간 (한 샘플만 쓰는 Taylor 전개 $p_j+v_j\Delta+\tfrac12a_j\Delta^2$ 는 구간마다 $a$ 가 계단으로 바뀐다) 은 가속도를 읽는 소비자에게 계단을 준다.
- `closed_form`: L4 의 feedforward 가 $\ddot\xi^O$ 를 직접 쓴다(L4 §4.1). 가속도가 샘플 경계에서 튀면 기준 가속도 $u$ 에 그대로 계단이 생기고, 그것이 CLIK을 거쳐 관절 명령의 jerk가 된다. $C^2$ 는 이 경로의 요구다.
- `mpc`: RT 는 샘플을 추종 대상으로 쓰지 않는다 (§5.2). 샘플이 쓰이는 곳은 포구 전의 감독 (지평 · stale 판정) 이고, 계획기가 읽는 것은 슬라이스의 샘플점 (보간하지 않은 vision 샘플) 이다. $C^2$ 는 `mpc` 에서는 소비자가 없는 성질이다.

### 4.3 정확도

보간은 **vision의 예측을 재현하는 것**이 목표이지 참 궤적을 맞히는 것이 아니다. 항력 절이 없는 profile 은 $a$ 가 상수 $g$ 라 점별 $(p,v,a)$ 가 한 포물선 위에 있고, 2차 궤적은 5차 기저에 정확히 포함되므로 보간은 간격과 무관하게 정확하다. 항력 profile 은 $a = g - k\lVert v\rVert v$ 를 주므로 $(p,v,a)$ 가 서로 맞고 보간 오차는 간격이 길수록 커진다. 보간 게이트(G2-C)를 만족하는 간격이 점 수 산출의 입력이다(D-15).

### 4.4 시각 정렬 `[확정 D-2]`

시간 규약의 SSoT 는 L0 §4.5 이다. 샘플러가 따르는 부분:

- **샘플 시각은 절대 `BallTime`** (steady ns). L1 이 수신 시 `header.stamp`·`horizon_ns` 를 한 번 변환해 싣는다(L1 §4.1). 원점이 다른 상대시각(메시지 스탬프 기준, 세션 기준, 계획 시각 기준)을 섞지 않는다.
- **샘플링·지평 경고는 now_lead** 로 한다: `NowLead` = 매 tick steady 실측 now + $T_{arm}$. γ 프로파일, $t_c$ 판정, `CLOSING→DECEL` 진입도 같은 축이다.
- **stale 판정은 steady 수신 나이**(now_steady − recv_steady)이며 샘플러가 아니라 L1 이 한다(L1 §5.3). 샘플러는 stale 을 판정하지 않는다.
- 수치 코어 경계에서만 double 초 상대값을 만든다: 구간 안 $s=(t-t_j)/h_j$ 는 같은 스냅샷의 두 `BallTime` 차로 계산하므로 원점 혼합이 생기지 않는다.
- `PlanSnapshot` 은 $t_c$ 등을 절대 `BallTime` 으로 싣는다(L3). 계획 스레드의 소요 시간이 시각을 틀리게 만들지 않고 남은 시간만 줄인다.

선행 보상량 $T_{arm}$ 은 L5 의 것이다(L5 §4.5). 샘플러의 선행축은 `NowLead` 의 $T_{arm}$ 이고, 선행 보상을 켜는 스위치는 `joint_cmd.lag.lead_enable` 이다 (꺼져 있으면 $T_{arm}$ 을 더하지 않는다). 값은 실기 식별로 정한다(L5 §6). 테스트는 $T_{arm}\ne0$ fixture 필수(L0 §4.5).

### 4.5 공분산을 어디까지 넘기는가 `[확정 A-3]`

**분리.** RT 스냅샷에는 $(t,p,v,a)$ 만 담고, 공분산은 계획기 버퍼에만 둔다. NaN(모름) 처리도 계획기 한 곳에서 한다. RT 경로(L4·L5·L7)에 공분산 소비자가 없다.

### 4.6 종료 조건과 지평 감시

- now_lead 가 마지막 샘플 시각을 넘으면 Taylor 외삽하고 `after_horizon=true` 를 올린다. **L7이 감시하는 것은 이 플래그뿐이다.** 포구 전에는 이 플래그가 `HORIZON_EXTRAP` 사유이고, 동결 (COMMITTED · CLOSING) 이후에는 기록만 하고 따르지 않는다 (`SampleBallForLaw`).
- 앞쪽(`before_horizon`, now_lead < 첫 샘플 시각)은 감시하지 않는다. vision 첫 점의 `horizon_ns` 가 0 이 아니거나 수신 지연이 있으면 정상 동작 중에도 발생할 수 있기 때문이다.
- 지평 끝 정확히(= 마지막 샘플 시각)는 외삽이 아니다.
- 지평 밖 외삽에 의존해 포구하는 것은 금지한다. L3는 마지막 샘플 시각에서 여유를 뺀 범위 안에서만 후보를 고른다 — 그 범위가 `planner.search.grid.slice.t_max` 이다 (vision 지평 − 여유).
- 수신 궤적 지평이 요구(`io.horizon_min`, D-15)보다 짧으면 L1 이 진단하고 계획 후보에서 제외한다(L1 §4.1).
- $p_z<z_{floor}$ 이거나 작업셀 밖인 샘플을 후보에서 제외하는 것은 구현하지 않았다 (G2-5). 탐색에도 포구점의 위치 제한은 없다 (L3 의 L3 §4.9).

### 4.7 Sanity check

1. 샘플점에서 $(p,v,a)$ 를 정확히 복원.
2. 샘플 경계에서 $a$ 연속 ($C^2$).
3. 순수 중력 데이터에 대해 참 궤적과 일치.
4. 지평 앞/뒤 외삽 플래그 구분, 지평 끝 정확히는 외삽 아님.
5. hint 커서 결과가 이진 탐색 결과와 동일.
6. 형식 검사기(단조 시각, 유한값, 개수 `[n_min, kCap]`, `dt_min` 미만 거부).
7. (회귀) `n > kCap`·NaN 시각에서 범위 밖 읽기 없이 invalid (ASan).

## 5. C++ 구현

### 5.1 `traj_sampler.hpp`

SSoT 는 `rtc_controllers/include/rtc_controllers/catching/traj_sampler.hpp` (`Hermite5`, `Interpolate`, `Extrapolate`, `SampleAt`, `Check`) 와 궤적 타입 `trajectory.hpp` 다. 계약:

- **점 개수 경계.** `n` 을 `[n_min, kCap]` 로 `Check`·`SampleAt`·RT 읽기 모두에서 **먼저** 검사한다 (범위 밖 읽기 방지). 런타임 점 수 상한은 `kCap` 이고 `n_max` 키는 없다 — 점 수는 vision profile 의 속성이라, profile 이 바뀌면 거부가 아니라 진단의 점 수로 드러난다
- **NaN 거부.** NaN 시각·값은 `Check` 에서 거부, `SampleAt(NaN)` 은 invalid 를 반환한다
- **간격 하한 거부.** 최소 샘플 간격 (`TrajLimits` 의 `dt_min`, 구성에서는 `TrajInputConfig::dt_min_ns` 상수) 미만 구간은 경고가 아니라 거부한다 — `Interpolate` 가 극소 $h$ 를 받아 $1/h^2$ 로 폭주하는 것을 막는다
- **POD 스냅샷.** 궤적 스냅샷은 `rtc::SeqLock` payload 이므로 trivially copyable 이어야 한다 — 벡터는 `std::array<double, 3>`, 계산은 `Eigen::Map` 으로 한다(L0 §5.2). `static_assert(std::is_trivially_copyable_v<…>)`
- **시간 타입.** 샘플 시각은 `BallTime`(절대 steady ns), 샘플링 인자는 `NowLead` (L0 §4.5). 스냅샷 필드: provenance `token` (`generation`·`snapshot_sequence`·`traj_recv_ns`·`activation_generation`), `n`, `valid`
- **공용 타입.** 궤적 타입은 L1·L2·L3 공용 헤더로 둔다(L1 → L2 의존 역전 해소). `kCap` 은 이 타입이 단독 소유한다 (L0 §5.2)

RT 규칙: 고정 크기, 할당 없음, `noexcept`, ROS 의존 없음. `SampleAt()`은 hint 커서로 평균 $O(1)$ 이고, 커서가 어긋나면 이진 탐색으로 복구한다($O(\log N)$ 상한).

### 5.2 L1·L4와의 연결

- L1이 `PointCloud2`를 파싱해 공용 궤적 스냅샷(`generation` 포함)을 채우고 `rtc::SeqLock` 에 쓴다(L1 §5.2).
- RT 루프(`RTControllerInterface::Compute`)는 **매 tick 무조건 `Load`** 하고(D-21, 재시도 상한 없음), payload 의 `snapshot_sequence` 로 새 스냅샷 여부를 판정한 뒤 `SampleAt(tr, now_lead, hint_)` 을 부른다 (`SampleBallForLaw`, `controller.cpp`).
- **샘플이 무엇에 쓰이는가는 planner 가 정한다.**
  - `closed_form`: 샘플 $(p,v,a)$ 를 L4 추종 대상 상태로 넘긴다 (`StepReferenceAndSolve` 의 `target`; L8 §4.1 순서 2).
  - `mpc`: `RunSegmentTick` 이 포구 전에 `SampleBallForLaw` 를 부르되 **구간(segment)은 샘플을 읽지 않는다** — 구간은 계획기가 낸 MPC 노드열을 따른다. 샘플은 감독에만 쓰인다: 샘플할 수 없으면 `BALL_STALE`, 지평 밖이면 `HORIZON_EXTRAP` (동결 전에만) — 사유가 남는다.
- `hint_`는 컨트롤러 멤버 (`traj_hint_`) 로 유지하고, **`snapshot_sequence` 가 바뀌면 0으로 초기화**한다. 정확성은 이진 탐색이 지키지만 틱 비용이 흔들린다.
- `Interpolate()` 가 `valid=false` 를 돌려주면(비단조 샘플 쌍 등) 그 틱은 invalid 로 처리한다.

**스냅샷 복사 비용.** 스냅샷은 `kCap` 고정이라 실제 $n$ 과 무관하게 전체를 **매 tick** 복사한다 — `SeqLock::sequence()` 로 새 메시지 도착 tick 에만 복사를 한정하는 최적화는 D-21 이 금지한다(payload 안 token 으로만 새 스냅샷을 판정). 점당 시각 + 9 double ≈ 80 B 이므로 복사 크기는 `kCap` × 80 B 다. `rtc::SeqLock::Load` 는 재시도 상한이 없으므로(L1 G1-8) 복사 시간이 곧 writer 와의 경합 창이며, 최악 재시도 시간은 G1-C 로 측정한다. 유일한 대응은 `kCap` 을 요구 $N$ 상한에 여유를 둔 값으로 줄이는 것이다.

## 6. YAML 파라미터

값은 `integrated_bringup/config/<robot>/controllers/demo_catching_controller.yaml` 와 `catching/search_grid.yaml`.

| 키 | 타입 | 단위 | 뜻 |
|---|---|---|---|
| `prediction.dt_expected` | double | s | vision 점 간격. 검사용이며 `io.n_min` 이 이 값에서 유도된다 (L1 §6). ball_perception 은 `horizon % step == 0` 을 요구한다 |

`prediction.*` 로는 이 키 하나만 있다. 나머지 역할은 다른 곳이 한다.

| 역할 | 하는 것 |
|---|---|
| 점 수 상한 (`prediction.max_samples`) | 컴파일 상수 `kCap` (`trajectory.hpp`) |
| 지평 끝 여유 (`prediction.t_horizon_margin`) | `planner.search.grid.slice.t_max` (`search_grid.yaml`) = vision 지평 − 여유 |
| 최소 간격 (`prediction.dt_min`) | `TrajInputConfig::dt_min_ns` 상수 (구조적 하한; 실제 간격 gate 는 `io.n_min`) |
| 선행 보상 (`prediction.lead`) | `joint_cmd.lag.lead_enable` · `joint_cmd.lag.T_arm` (L5 §6) — 선행량은 $T_{arm}\times$ `lead_enable` |
| 바닥 · 작업셀 (`prediction.z_floor`, `prediction.workcell`) | 구현하지 않았다 (G2-5). 포구점의 위치 제한도 없다 (L3 §4.9) |

## 7. 단위 기술 구현 순서

구현한 것: 공용 궤적 타입 (POD, `BallTime` 시각) 과 `Check` (개수 경계 선검사, NaN, 단조, 간격 하한 거부), Hermite 샘플러, `NowLead` 샘플링, 지평 감시 플래그의 L7 입력 연결. 테스트는 §4.7 의 7종이 `test_catching_traj_sampler.cpp` 에 있다.

## 8. 디버깅 방법

- 보간 결과와 원 샘플을 같은 그래프(steady 절대 시각 축)에 그린다. 샘플점을 지나지 않으면 L1 의 시각 변환(L1 §4.1) 또는 `horizon_ns` 해석을 의심한다.
- (`closed_form`) 기준 가속도 $u$ 에 주기적 계단이 보이면 보간이 Taylor로 떨어졌는지 확인한다(§4.2).
- 외삽 플래그가 자주 서면 vision 지평이 요구(D-15)보다 짧거나 $T_{arm}$ 이 과대한 것이다.
- 샘플 간격이 `dt_expected`와 다르면 vision 측 profile 설정 또는 리샘플링을 의심한다.
- 궤적이 틱마다 크게 점프하면 vision 예측 갱신 품질 또는 시계 오차를 본다(L1 §4.4 점프 진단: 연속 두 스냅샷의 같은 절대시각 위치 차).

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G2-A | 샘플점 복원 오차 < 1e-12 (p, v, a) | `[SIM-ANY]` |
| G2-B | 샘플 경계 $\Vert\Delta a\Vert$ < 1e-6 m/s² | `[SIM-ANY]` |
| G2-C | 순수 중력 데이터에서 참 궤적 대비 위치 오차 < 1e-10 m | `[SIM-ANY]` |
| G2-D | 외삽 플래그(앞/뒤 구분), hint 커서, 형식 검사기 동작 | `[SIM-ANY]` |
| G2-E | RT 샘플링 경로 할당 0 (`ScopedNoMalloc`·`ScopedAllocGate`), 최악 실행시간·**복사 바이트 수** 기록. 임계는 L8 틱 예산에서 역산 | `[SIM-ANY]` |
| G2-G | `Interpolate()` 가 비단조 샘플 쌍에 `valid=false` 반환 | `[SIM-ANY]` |
| G2-H | (회귀) `n > kCap`·NaN 입력에서 ASan 무오류 + invalid, `dt_min` 미만 거부, 스냅샷 타입 trivially copyable | `[SIM-ANY]` |
| G2-F | sim(ball_perception) 재생에서 외삽 발생률·샘플 간격 분포 기록, 실기는 S10 | `[SIM-ANY]` |

G2-A~D 는 `test_catching_traj_sampler.cpp` 가 돌린다.

## 10. 미확정 항목

TBD-WS-01 (바닥 높이·작업셀 경계 — G2-5), `kCap` (provisional), 선행 보상의 $T_{arm}$ (실기 식별). 점 수 요구 · `prediction.dt_expected` · `io.n_min` 은 provisional 이다 (L1 §6). `io.n_min` 이 걸린 선행시간 $L$ 의 실기 확정은 실기 단계 (#613) 에서 한다.
