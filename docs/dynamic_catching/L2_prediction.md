# L2 — Prediction: 궤적 샘플러 (vision 예측의 시각 정렬·보간)

- 문서 버전: v0.5 (2026-09-19) — 결정·단계의 SSoT 는 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) (충돌 시 plan 우선)
- 브랜치: 단계별 `type/kebab-slug` (main 기준, 마스터 §4.2)
- 배치 `[확정 D-1]`: rtc_controllers 의 `catching` 하위 디렉토리 (namespace `rtc::catching`, ROS 비의존 순수 코드)
- 단계: **S1** — S1.2 공용 궤적 타입(POD) + Hermite 샘플러, S1.3 시간 타입 적용
- 선행: 단계 W (완료, plan §2), L0 (시간 타입·POD 규칙)
- 산출물: 공용 궤적 타입(스냅샷 POD), 샘플러 (참조: `traj_sampler.hpp`)

---

## 1. 범위 / 비범위

vision 노드가 이미 예측 궤적을 발행하고(마스터 §5, D-4), 제어 PC는 그것을 **재전파하지 않고 그대로 신뢰**한다(마스터 §5.2).

범위:
1. **공용 궤적 타입** — L1 파서가 채우고 RT·계획기가 읽는 스냅샷 POD (S1.2). L1 이 L2 타입에 의존하던 역전을 이 타입을 공용으로 두어 해소한다.
2. vision이 준 $(p,v,a)$ 샘플 열을 임의의 제어 시각으로 **보간**한다 (RT 루프, `control_rate` 100–5000 Hz).
3. 지평 밖 요청을 감지하고 외삽 플래그를 올린다.
4. 샘플 열의 형식·일관성을 검사한다.

비범위:
- 공 상태 추정과 궤적 **예측** (vision 노드).
- 제어 PC 자체 동역학 전파 — v0.2의 `RtBallPropagator`, `rollout()`, `propagateWithCov()`는 **전부 삭제**했다.
- 공분산 전파·보관. vision이 점마다 6×6을 주고, 공분산은 계획기 버퍼에만 둔다(A-3, §4.5).
- 포구점 선택(L3).

## 2. 코드 확인 게이트

단계 W에서 처리했다(plan §2). 본 layer에 직접 걸리는 항목:

| ID | 확인 항목 | 기록 |
|---|---|---|
| G2-1 | 발행 주기, $N$ 범위, 지평 길이 → `kCap`(컴파일타임)·`n_max`(런타임)와 L3 슬라이스 범위 | 전환 `[확정 D-15]` — vision 사양은 **제어기가 요구를 정하고** sim profile 을 맞춘다. 현 예시 profile: 지평 0.5 s, 간격 0.05 s, 최대 10 점, ≤ 30 Hz. `kCap` 은 S0.7 손계산 제안값으로 S1.2 가 정해 provisional 로 두고, 런타임 상한 `n_max ≤ kCap` 은 S3.6 이 요구 사양에서 산출한다 — `n_max > kCap` 이면 S1.2 backfill 후 S1 게이트 재실행 (plan §4.2) (W5-6, TBD-VIS-04) |
| G2-2 | 점 시각 필드 타입·기준 → 시각 정렬 식 | 닫힘 — `horizon_ns` UINT32 (`header.stamp` 기준 상대 ns). L1 이 수신 시 절대 `BallTime` 으로 변환한다(D-2, L1 §4.1). 샘플러는 절대 시각만 받는다 (W5-3, TBD-VIS-03) |
| G2-3 | `ax,ay,az`가 상수 $g$인지 항력 포함 총 가속도인지 | 닫힘 — 상수 $g$ (W5-4, TBD-VIS-05) |
| G2-4 | 공분산을 RT까지 넘길지 | 닫힘 `[확정 A-3]` — RT 스냅샷에서 분리, 계획기 버퍼에만 (TBD-COV-01) |
| G2-5 | 바닥 높이·작업셀 경계의 `W` 좌표 | TBD-WS-01 유지 (W7-1). 닫는 단계는 plan 에 지정되지 않음 |

## 3. 참고자료

5차 Hermite 보간은 수치해석 표준 내용(등급 a). [R8]은 L1·L3의 공분산 **사용**에만 관련되고, 전파에는 관련되지 않는다.

## 4. 수학적 이론

### 4.1 왜 재전파하지 않는가 `[확정]`

vision의 예측기와 제어 PC가 각자 전파하면 두 모델이 어긋날 때 예측이 갈라진다. 어긋남은 항력계수, 중력 벡터, 적분 스텝, 추정 시점 어디서든 생긴다. 한쪽을 단일 진리원으로 두는 편이 낫고, 예측기를 가진 쪽은 vision이다.

부수 효과로 다음이 실시간 경로에서 사라진다.

- L0의 이차 항력 모델과 RK4 (→ test fixture 전용, S1.6)
- $k$ 최소제곱 식별 (→ fixture 전용)
- 변분방정식·상태전이행렬 (`rk4WithStm`)
- 공정잡음 $Q$ 설정과 vision과의 값 합의 (v0.2의 `TBD-PRED-01` 폐기)

남는 것은 **보간**뿐이다.

### 4.2 5차 Hermite 보간 `[논문 외 유도]`

vision 샘플 간격(예시 profile 0.05 s)은 RT 틱 $h$ = `dt` (0.2–10 ms)보다 훨씬 길다 — 500 Hz 면 구간 하나에 25 틱이 들어가므로 보간 방식이 중요하다.

각 샘플이 $(p_j,v_j,a_j)$ 를 모두 주므로, 구간 $[t_j,t_{j+1}]$ ($h_j=t_{j+1}-t_j$, $s=(t-t_j)/h_j$)에서 **양 끝의 위치·속도·가속도를 모두 맞추는** 5차 Hermite를 쓴다.

$$p(s)=H_0p_j+H_1h_j\,v_j+H_2h_j^2a_j+H_3p_{j+1}+H_4h_j\,v_{j+1}+H_5h_j^2a_{j+1}$$

$$
\begin{aligned}
H_0&=1-10s^3+15s^4-6s^5, & H_1&=s-6s^3+8s^4-3s^5, & H_2&=\tfrac12s^2-\tfrac32s^3+\tfrac32s^4-\tfrac12s^5\\
H_3&=10s^3-15s^4+6s^5, & H_4&=-4s^3+7s^4-3s^5, & H_5&=\tfrac12s^3-s^4+\tfrac12s^5
\end{aligned}
$$

속도·가속도는 $s$로 미분해 $h_j$, $h_j^2$로 나눈다(참조 헤더 `interpolate`).

**왜 $C^2$가 필요한가.** L4의 feedforward가 $\ddot\xi^O$ 를 직접 쓴다(L4 §4.1). 가속도가 샘플 경계에서 튀면 기준 가속도 $u$ 에 그대로 계단이 생기고, 그것이 CLIK을 거쳐 관절 명령의 jerk가 된다. 5차 Hermite는 양 끝 $a$ 를 맞추므로 경계에서 $C^2$ 다.

**비교 (실측, `test_l2.cpp`).** 한 샘플만 쓰는 Taylor 전개 $p_j+v_j\Delta+\tfrac12a_j\Delta^2$ 를 쓰면 구간마다 $a$ 가 계단으로 바뀐다.

| 방식 | 샘플 경계에서의 $\Vert\Delta a\Vert$ |
|---|---|
| 5차 Hermite | $4.8\times10^{-7}$ m/s² |
| Taylor (1-sample) | $3.99\times10^{-2}$ m/s² |

$10^{4}$배 이상 차이가 난다.

### 4.3 정확도

보간은 **vision의 예측을 재현하는 것**이 목표이지 참 궤적을 맞히는 것이 아니다. 두 모델이 얼마나 가까운지는 알아둘 필요가 있다(`test_l2.cpp`, 60 Hz 간격 기준 실측).

| vision 모델 | 60 Hz 간격에서 보간 오차 (위치) |
|---|---|
| 순수 중력 ($a$ 상수) | $7.0\times10^{-13}$ m — 2차 궤적은 5차 기저에 정확히 포함된다 |
| 이차 항력 ($k=0.0229$) | $4.2\times10^{-14}$ m (가속도 $2.4\times10^{-9}$ m/s²) |

ball_perception 은 $a$ 를 상수 $g$ 로 준다(G2-3). 그러면 점별 $(p,v,a)$ 가 한 포물선 위에 있는 한 보간은 간격과 무관하게 정확하다. 예시 profile 의 0.05 s 간격에서 점들이 한 포물선이 아닌 경우(예측 내부에 항력이 있는데 $a$ 만 상수로 보고하는 경우)의 오차는 S1.2 이식 시 0.05 s 간격으로 다시 잰다. L2 보간 게이트를 만족하는 간격이 S3.6 점 수 산출의 입력이다(D-15).

### 4.4 시각 정렬 `[확정 D-2]`

시간 규약의 SSoT 는 plan §3 이다. 샘플러가 따르는 부분:

- **샘플 시각은 절대 `BallTime`** (steady ns). L1 이 수신 시 `header.stamp`·`horizon_ns` 를 한 번 변환해 싣는다(L1 §4.1). 원점이 다른 상대시각(메시지 스탬프 기준, 세션 기준, 계획 시각 기준)을 섞지 않는다 — v0.4 의 "모든 상대시각의 단일 원점 = `header.stamp`" 규약은 절대 시각으로 대체했다.
- **샘플링·지평 경고는 now_lead** 로 한다: `NowLead` = 매 tick steady 실측 now + $T_{arm}$. γ 프로파일, $t_c$ 판정, `CLOSING→DECEL` 진입도 같은 축이다.
- **stale 판정은 steady 수신 나이**(now_steady − recv_steady)이며 샘플러가 아니라 L1 이 한다(L1 §5.3). 샘플러는 stale 을 판정하지 않는다.
- 수치 코어 경계에서만 double 초 상대값을 만든다: 구간 안 $s=(t-t_j)/h_j$ 는 같은 스냅샷의 두 `BallTime` 차로 계산하므로 원점 혼합이 생기지 않는다.
- `PlanSnapshot` 은 $t_c$ 등을 절대 `BallTime` 으로 싣는다(L3). v0.4 가 `PlanSnapshot::t_ref` 를 메시지 `t_ref` 와 맞추던 장치는 필요 없어졌다 — 계획 스레드의 소요 시간이 시각을 틀리게 만들지 않고 남은 시간만 줄인다(plan §7.2).

`prediction.lead` = $T_{arm}$ 은 L5 의 선행 보상량이다(L5 §4.5). 도출 관계(§6)는 v0.4 와 같다. backend 에는 지연 보상이 없으므로(W4-2) 끌 이유가 backend 쪽에는 없고, $T_{arm}$ 값은 S10 식별로 정한다(L5 §6). 테스트는 $T_{arm}\ne0$ fixture 필수(plan §3).

### 4.5 공분산을 어디까지 넘기는가 `[확정 A-3]`

닫힘 — **분리.** RT 스냅샷에는 $(t,p,v,a)$ 만 담고, 공분산은 계획기 버퍼에만 둔다. NaN(모름) 처리도 계획기 한 곳에서 한다. RT 경로(L4·L5·L7)에 공분산 소비자가 없다. v0.4 의 2안(일체)은 폐기했다.

### 4.6 종료 조건과 지평 감시

- now_lead 가 마지막 샘플 시각을 넘으면 Taylor 외삽하고 `after_horizon=true` 를 올린다. **L7이 감시하는 것은 이 플래그뿐이다.**
- 앞쪽(`before_horizon`, now_lead < 첫 샘플 시각)은 감시하지 않는다. vision 첫 점의 `horizon_ns` 가 0 이 아니거나 수신 지연이 있으면 정상 동작 중에도 발생할 수 있기 때문이다.
- 지평 끝 정확히(= 마지막 샘플 시각)는 외삽이 아니다.
- 지평 밖 외삽에 의존해 포구하는 것은 금지한다. L3는 마지막 샘플 시각에서 여유(`t_horizon_margin`)를 뺀 범위 안에서만 후보를 고른다.
- 수신 궤적 지평이 요구(`io.horizon_min`, D-15)보다 짧으면 L1 이 진단하고 계획 후보에서 제외한다(L1 §4.1).
- $p_z<z_{floor}$ 이거나 작업셀 밖인 샘플은 L3가 후보에서 제외한다(G2-5).

### 4.7 Sanity check

1. 샘플점에서 $(p,v,a)$ 를 정확히 복원.
2. 샘플 경계에서 $a$ 연속 ($C^2$).
3. 순수 중력 데이터에 대해 참 궤적과 일치.
4. 지평 앞/뒤 외삽 플래그 구분, 지평 끝 정확히는 외삽 아님.
5. hint 커서 결과가 이진 탐색 결과와 동일.
6. 형식 검사기(단조 시각, 유한값, 개수 `[n_min, kCap]`, `dt_min` 미만 거부).
7. (회귀) `n > kCap`·NaN 시각에서 범위 밖 읽기 없이 invalid (ASan).

## 5. C++ 구현

### 5.1 `traj_sampler.hpp` (참조 구현, 검증 완료)

v0.5 에서 코드 복사본(v0.2 그대로였음)을 삭제했다. **SSoT 는 같은 폴더의 `traj_sampler.hpp` (v0.4)** 다 — `before_horizon`/`after_horizon` 분리, `track_epoch` 필드 등 v0.4 변경은 헤더에만 있다. S1.2 이식 시 변경:

- **점 개수 경계.** `n` 을 `[n_min, kCap]` 로 `check`·`sampleAt`·RT 읽기 모두에서 **먼저** 검사한다(참조 구현은 `n > kMaxSamples` 에서 범위 밖 읽기가 있었다, ASan 확인). 런타임 상한 `n_max`(S3.6, ≤ `kCap`)이 정해지면 그 값으로 더 좁혀 검사한다
- **NaN 거부.** NaN 시각·값은 `check` 에서 거부, `sampleAt(NaN)` 은 invalid 를 반환한다(참조 구현은 valid 반환)
- **`dt_min` 거부.** 최소 샘플 간격 미만 구간은 경고가 아니라 거부한다 — `interpolate` 가 극소 $h$ 를 받아 $1/h^2$ 로 폭주하는 것을 막는다
- **POD 스냅샷.** 궤적 스냅샷은 `rtc::SeqLock` payload 이므로 trivially copyable 이어야 한다 — `Sample` 의 `Eigen::Vector3d` 멤버를 `std::array<double, 3>` 으로 바꾸고, 계산은 `Eigen::Map` 으로 한다(L0 §5.2, plan §6). `static_assert(std::is_trivially_copyable_v<…>)`
- **시간 타입.** 샘플 시각은 `BallTime`(절대 steady ns), 샘플링 인자는 `NowLead` (L0 §4.5). 스냅샷 필드: `generation`·`snapshot_sequence` (uint64, `track_epoch`·`seq` 대체), `recv_steady_ns`, `n`, `valid`
- **공용 타입.** 궤적 타입은 L1·L2·L3 공용 헤더로 둔다(L1 → L2 의존 역전 해소). `kCap` 은 이 타입이 단독 소유하고(L0 의 중복 상수·`static_assert` 짝맞춤 삭제), 값은 S0.7 제안값으로 S1.2 가 정한다(provisional). 런타임 상한 `n_max ≤ kCap` 은 S3.6 이 정하고, 넘으면 S1.2 backfill (plan §4.2)
- **명명.** namespace `rtc::catching`, 함수 PascalCase (`hermite5`/`interpolate`/`extrapolate`/`sampleAt`/`check` → `Hermite5`/`Interpolate`/`Extrapolate`/`SampleAt`/`Check`)

RT 규칙: 고정 크기, 할당 없음, `noexcept`, ROS 의존 없음. `SampleAt()`은 hint 커서로 평균 $O(1)$ 이고, 커서가 어긋나면 이진 탐색으로 복구한다($O(\log N)$ 상한).

### 5.2 L1·L4와의 연결

- L1이 `PointCloud2`를 파싱해 공용 궤적 스냅샷(`generation` 포함)을 채우고 `rtc::SeqLock` 에 쓴다(L1 §5.2).
- RT 루프(`RTControllerInterface::Compute`)는 **매 tick 무조건 `Load`** 하고(D-21, 재시도 상한 없음), payload 의 `snapshot_sequence` 로 새 스냅샷 여부를 판정한다 → 매 tick `SampleAt(tr, now_lead, hint_)` → 결과를 L4 추종 대상 상태 $(p,v,a)$ 로 넘긴다(L8 §4.1 순서 2).
- `hint_`는 컨트롤러 멤버로 유지하고, **`snapshot_sequence` 가 바뀌면 0으로 초기화**한다. 정확성은 이진 탐색이 지키지만 틱 비용이 흔들린다.
- `Interpolate()` 가 `valid=false` 를 돌려주면(비단조 샘플 쌍 등) 그 틱은 invalid 로 처리한다.

**스냅샷 복사 비용.** 스냅샷은 `kCap` 고정이라 실제 $n$ 과 무관하게 전체를 **매 tick** 복사한다 — `SeqLock::sequence()` 로 새 메시지 도착 tick 에만 복사를 한정하는 최적화는 D-21 이 금지한다(payload 안 token 으로만 새 스냅샷을 판정). 점당 시각 + 9 double ≈ 80 B, 512 샘플이면 약 41 KB. `rtc::SeqLock::Load` 는 재시도 상한이 없으므로(L1 G1-8) 복사 시간이 곧 writer 와의 경합 창이며, 최악 재시도 시간은 G1-C 로 측정한다. 유일한 대응은 `kCap` 을 S3.6 요구 $N$ 상한(`n_max`)에 여유를 둔 값으로 줄이는 것이다(예시 profile 은 최대 10 점).

## 6. YAML 파라미터

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `prediction.max_samples` | int | – | 512 (provisional) | 16–512 | `kCap` (컴파일 상수와 일치 검사, S0.7 제안값). 런타임 상한 `n_max ≤ kCap` 은 S3.6 요구 사양으로 정한다 (D-15) |
| `prediction.n_min` | – | – | – | – | v0.5 삭제 — 단일 키 `io.n_min` (L1 §6) 을 쓴다 (plan S0.3) |
| `prediction.t_horizon_margin` | double | s | 0.05 | 0–0.3 | §4.6 지평 끝 여유 |
| `prediction.dt_expected` | double | s | `TBD` | >0 | vision 점 간격. 검사용. 예시 profile 0.05 (S3.6 에서 확정) |
| `prediction.dt_tol` | double | – | 0.2 | 0–1 | 샘플 간격 허용 상대편차 (경고) |
| `prediction.dt_min` | double | s | `TBD` | >0 | 이 미만 간격은 **거부** (S1.2) |
| `prediction.z_floor` | double | m | `TBD` | – | G2-5 |
| `prediction.workcell` | box | m | `TBD` | – | G2-5 |
| `prediction.lead` | double | s | 0.0 | ≥0 | §4.4. **`joint_cmd.lag.T_arm`(L5 §6)에서 파생한다** — `lead = T_arm × lead_enable`(마스터 §6). backend 에 지연 보상 없음 (W4-2), $T_{arm}$ 은 S10 식별 |

v0.2의 `prediction.rt.*`, `prediction.rollout.*`, `q_acc`, `q_k`는 전부 삭제했다.

## 7. 단위 기술 구현 순서

- **S1.2a** 공용 궤적 타입(POD, `BallTime` 시각) + `Check` (개수 경계 선검사, NaN, 단조, `dt_min` 거부).
- **S1.2b** Hermite 샘플러 이식 + §4.7 테스트 7종 (참조: `test_l2.cpp`, GTest), 0.05 s 간격 정확도 재측정(§4.3).
- **S1.3** `NowLead` 샘플링 + $T_{arm}\ne0$ fixture 로 지평 플래그 축 검증.
- **S5** 지평 감시·외삽 플래그를 L7 입력으로 연결, sim 재생 테스트: 샘플 간격 분포, $N$ 분포, 지평 길이, 외삽 발생률 기록.

## 8. 디버깅 방법

- 보간 결과와 원 샘플을 같은 그래프(steady 절대 시각 축)에 그린다. 샘플점을 지나지 않으면 L1 의 시각 변환(L1 §4.1) 또는 `horizon_ns` 해석을 의심한다.
- 기준 가속도 $u$ 에 주기적 계단이 보이면 보간이 Taylor로 떨어졌는지 확인한다(§4.2).
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

`test_l2.cpp`가 G2-A~D를 돌린다(v0.4 기준 통과).

## 10. 미확정 항목

TBD-WS-01 (닫는 단계 미지정), `kCap` 제안값 (S0.7 → S1.2)·런타임 `n_max`·`prediction.dt_expected` (S3.6, D-15), `prediction.dt_min` (S1.2), `io.n_min` (L1), `prediction.lead` 의 $T_{arm}$ (S10).
