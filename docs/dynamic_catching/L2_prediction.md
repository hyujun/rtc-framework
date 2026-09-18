# L2 — Prediction: 궤적 샘플러 (vision 예측의 시각 정렬·보간)

- 브랜치: `feat/catching-L2-prediction`
- 패키지: `catching_prediction` (가칭, `TBD-WS-02`)
- 선행: 단계 W, L0
- 산출물: `traj_sampler.hpp`, `PredictedTrajectory` 고정 버퍼

---

## 1. 범위 / 비범위

**v0.3에서 이 layer의 역할이 크게 줄었다.** vision 노드가 이미 예측 궤적을 발행하고(마스터 §5), 제어 PC는 그것을 **재전파하지 않고 그대로 신뢰**하기로 확정했다(마스터 §5.2).

범위:
1. vision이 준 $(p,v,a)$ 샘플 열을 임의의 제어 시각으로 **보간**한다 (500 Hz RT 루프).
2. 지평 밖 요청을 감지하고 외삽 플래그를 올린다.
3. 샘플 열의 형식·일관성을 검사한다.

비범위:
- 공 상태 추정과 궤적 **예측** (vision 노드).
- 제어 PC 자체 동역학 전파 — v0.2의 `RtBallPropagator`, `rollout()`, `propagateWithCov()`는 **전부 삭제**했다.
- 공분산 전파. vision이 점마다 6×6을 주므로 제어 PC가 계산하지 않는다.
- 포구점 선택(L3).

## 2. 코드 확인 게이트

단계 W에서 처리한다. 본 layer에 직접 걸리는 항목:

| ID | 확인 항목 | 기록 |
|---|---|---|
| G2-1 | 발행 주기, $N$(width) 범위, 지평 길이 → `kMaxSamples`와 L3 슬라이스 범위 결정 | TBD-VIS-04 (W5-6) |
| G2-2 | `t` 필드 타입·기준 → 시각 정렬 식 | TBD-VIS-03 (W5-3) |
| G2-3 | `ax,ay,az`가 상수 $g$인지 항력 포함 총 가속도인지 | TBD-VIS-05 (W5-4) |
| G2-4 | 공분산을 RT까지 넘길지 결정 (§4.5) | TBD-COV-01 (W2-2) |
| G2-5 | 바닥 높이·작업셀 경계의 `W` 좌표 | TBD-WS-01 (W7-1) |

## 3. 참고자료

5차 Hermite 보간은 수치해석 표준 내용(등급 a). [R8]은 이제 L1·L3의 공분산 **사용**에만 관련되고, 전파에는 관련되지 않는다.

## 4. 수학적 이론

### 4.1 왜 재전파하지 않는가 `[확정]`

vision의 예측기와 제어 PC가 각자 전파하면 두 모델이 어긋날 때 예측이 갈라진다. 어긋남은 항력계수, 중력 벡터, 적분 스텝, 추정 시점 어디서든 생긴다. 한쪽을 단일 진리원으로 두는 편이 낫고, 예측기를 가진 쪽은 vision이다.

부수 효과로 다음이 실시간 경로에서 사라진다.

- L0의 이차 항력 모델과 RK4 (→ 시뮬레이션 fixture 전용)
- $k$ 최소제곱 식별 (L0.3 → fixture 전용)
- 변분방정식·상태전이행렬 (`rk4WithStm`)
- 공정잡음 $Q$ 설정과 vision과의 값 합의 (v0.2의 `TBD-PRED-01` 폐기)

남는 것은 **보간**뿐이다.

### 4.2 5차 Hermite 보간 `[논문 외 유도]`

vision 샘플 간격은 카메라 주기(예: 60 Hz → 16.7 ms)이고 RT 틱은 2 ms다. 샘플 하나당 RT 틱 8개가 들어가므로 보간 방식이 중요하다.

각 샘플이 $(p_j,v_j,a_j)$ 를 모두 주므로, 구간 $[t_j,t_{j+1}]$ ($h=t_{j+1}-t_j$, $s=(t-t_j)/h$)에서 **양 끝의 위치·속도·가속도를 모두 맞추는** 5차 Hermite를 쓴다.

$$p(s)=H_0p_j+H_1h\,v_j+H_2h^2a_j+H_3p_{j+1}+H_4h\,v_{j+1}+H_5h^2a_{j+1}$$

$$
\begin{aligned}
H_0&=1-10s^3+15s^4-6s^5, & H_1&=s-6s^3+8s^4-3s^5, & H_2&=\tfrac12s^2-\tfrac32s^3+\tfrac32s^4-\tfrac12s^5\\
H_3&=10s^3-15s^4+6s^5, & H_4&=-4s^3+7s^4-3s^5, & H_5&=\tfrac12s^3-s^4+\tfrac12s^5
\end{aligned}
$$

속도·가속도는 $s$로 미분해 $h$, $h^2$로 나눈다(구현은 §5.1).

**왜 $C^2$가 필요한가.** L4의 feedforward가 $\ddot\xi^O$ 를 직접 쓴다(L4 §4.1). 가속도가 샘플 경계에서 튀면 기준 가속도 $u$ 에 그대로 계단이 생기고, 그것이 CLIK을 거쳐 관절 명령의 jerk가 된다. 5차 Hermite는 양 끝 $a$ 를 맞추므로 경계에서 $C^2$ 다.

**비교 (실측, `test_l2.cpp`).** 한 샘플만 쓰는 Taylor 전개 $p_j+v_j\Delta+\tfrac12a_j\Delta^2$ 를 쓰면 구간마다 $a$ 가 계단으로 바뀐다.

| 방식 | 샘플 경계에서의 $\Vert\Delta a\Vert$ |
|---|---|
| 5차 Hermite | $4.8\times10^{-7}$ m/s² |
| Taylor (1-sample) | $3.99\times10^{-2}$ m/s² |

$10^{4}$배 이상 차이가 난다.

### 4.3 정확도

보간은 **vision의 예측을 재현하는 것**이 목표이지 참 궤적을 맞히는 것이 아니다. 그래도 두 모델이 얼마나 가까운지는 알아둘 필요가 있다(`test_l2.cpp`).

| vision 모델 | 60 Hz 간격에서 보간 오차 (위치) |
|---|---|
| 순수 중력 ($a$ 상수) | $7.0\times10^{-13}$ m — 2차 궤적은 5차 기저에 정확히 포함된다 |
| 이차 항력 ($k=0.0229$) | $4.2\times10^{-14}$ m (가속도 $2.4\times10^{-9}$ m/s²) |

즉 16.7 ms 간격에서는 두 경우 모두 사실상 정확하다. 보간이 오차원이 되지 않는다. `TBD-VIS-05`(항력 포함 여부)가 보간 방식 선택에 영향을 주지 않는 이유다 — 다만 L3의 $\Vert v\Vert$·$a$ 해석에는 영향을 준다.

### 4.4 시각 정렬

**모든 상대시각의 단일 원점은 메시지의 `header.stamp`($t_{ref}$)다.** 그 위에 두 축을 둔다(마스터 §3).

$$t_{real}=10^{-9}(t_{now}-t_{ref}),\qquad t_{lead}=t_{real}+T_{arm}$$

- **선행축 $t_{lead}$**: 궤적 샘플링(`sampleAt`), γ 프로파일, $t_c$, `setIntercept`/`derateGamma`, `CLOSING→DECEL` 전환.
- **실제시각축 $t_{real}$**: 손 명령 시각 $t_{cmd}$(L3 §4.11), 접촉 판정 창, stale·나이 판정.

$T_{arm}$ 은 L5의 선행 보상량이다(L5 §4.5). backend가 지연을 이미 보상하고 있으면 0이다(W4-2).

**`PlanSnapshot::t_ref` 는 계획에 쓴 궤적 메시지의 `t_ref` 와 같은 값으로 둔다.** 그래야 γ 프로파일의 $t_0,t_1$ 과 샘플러의 $t_{lead}$ 가 같은 원점을 쓴다. v0.2는 계획 스레드의 `clock_now()` 를 `PlanSnapshot::t_ref` 로 삼았는데, 그러면 두 축의 원점이 메시지 나이 + 계획 소요(통상 10–50 ms)만큼 어긋난다. $\dot\gamma_{\max}\approx1.67\,\mathrm{s^{-1}}$ 에서 30 ms면 γ 오차 0.05이고, γ가 $\gamma_f$ 에 도달하는 시각이 $t_c$ 에서 벗어나 L4 §4.2 Corollary 가 성립하지 않는다.

두 시계가 같아야 한다(`use_sim_time` 또는 PTP, L1 게이트). `t` 필드가 uint32 ns면 파서에서 초로 바꾼다(`TBD-VIS-03`).

### 4.5 공분산을 어디까지 넘기는가 `[TBD-COV-01]`

점당 6×6 float64 = 288 B다. $N=300$ 이면 공분산만 86 KB이고, SeqLock으로 매 틱 복사하기에는 크다.

두 가지 안을 두고 단계 W(W2-2, RTC의 SeqLock·버퍼 관례)에서 고른다.

1. **분리 `[권장]`.** RT 스냅샷에는 $(t,p,v,a)$ 만 담고(점당 80 B), 공분산은 non-RT 계획 버퍼에만 둔다. L3는 non-RT이므로 전체를 본다. RT 경로에서 공분산을 쓰는 곳이 현재 없다.
2. **일체.** 단일 스냅샷이라 구조는 단순하지만 틱당 복사량과 최악 지연이 커진다.

판단 기준: RT 경로에서 공분산을 읽는 소비자가 생기는지. 현재 L4·L5·L7 어디도 읽지 않으므로 1안이 맞다.

### 4.6 종료 조건과 지평 감시

- $t_{lead}$ 가 마지막 샘플을 넘으면 Taylor 외삽하고 `after_horizon=true` 를 올린다. **L7이 감시하는 것은 이 플래그뿐이다.**
- 앞쪽(`before_horizon`, $t<s_0.t$)은 감시하지 않는다. vision이 $s_0.t>0$ 으로 발행하면(`TBD-VIS-03/04` 미확정) 정상 동작 중에도 상시 발생하기 때문이다. v0.2는 단일 `extrapolated` 플래그라 이 경우 `HORIZON_EXTRAP` 오abort가 상시 났다.
- 지평 끝 정확히($t=s_{n-1}.t$)는 외삽이 아니다.
- 지평 밖 외삽에 의존해 포구하는 것은 금지한다. L3는 마지막 샘플 시각에서 여유(`t_horizon_margin`)를 뺀 범위 안에서만 후보를 고른다.
- $p_z<z_{floor}$ 이거나 작업셀 밖인 샘플은 L3가 후보에서 제외한다(G2-5).

### 4.7 Sanity check

1. 샘플점에서 $(p,v,a)$ 를 정확히 복원.
2. 샘플 경계에서 $a$ 연속 ($C^2$).
3. 순수 중력 데이터에 대해 참 궤적과 일치.
4. 지평 안/밖 외삽 플래그.
5. hint 커서 결과가 이진 탐색 결과와 동일.
6. 형식 검사기(단조 $t$, 유한값, 개수).

## 5. C++ 구현

### 5.1 `traj_sampler.hpp` (참조 구현, 검증 완료)

```cpp
#pragma once
// catching_prediction/traj_sampler.hpp — vision 예측 궤적(sensor_msgs/PointCloud2)을
// RT 제어 시각으로 샘플링한다. RT-safe: 고정 크기, 할당 없음, noexcept, ROS 의존 없음.
//
// 제어 PC 는 공 궤적을 **다시 전파하지 않는다**. vision 이 준 (p, v, a) 샘플 열을
// 그대로 믿고, 샘플 사이만 보간한다 (L2 §4.1).
//
// 보간은 5차 Hermite 다. 양 끝점의 p, v, a 를 모두 맞추므로 결과가 C^2 이고,
// 따라서 L4 feedforward 에 들어가는 xi_ddot^O 가 샘플 경계에서 튀지 않는다.
// (한 샘플만 쓰는 Taylor 전개 p + v*dt + a*dt^2/2 는 a 가 샘플마다 계단으로 바뀌어
//  u 에 불연속을 만든다 — L2 §4.2 의 수치 비교 참조.)
#include <Eigen/Core>
#include <algorithm>
#include <cstdint>

namespace catching::traj {

inline constexpr int kMaxSamples = 512;
// 이식 시 L0 types.hpp 와 함께 컴파일되면 다음을 켠다 (CovBuffer 길이 일치, L0 §5.2):
//   static_assert(kMaxSamples == catching::kMaxTrajSamples);
// 참조 구현은 L0 에 의존하지 않으려고 주석으로 둔다.

// vision PointCloud2 의 한 점 중 RT 경로가 쓰는 부분.
// cov(6x6) 는 non-RT 계획 버퍼에만 둔다 (L2 §4.5, TBD-VIS-04).
struct Sample {
  double t{0.0};                                  // [s] header.stamp 기준 상대 시각
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};     // [m]
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};     // [m/s]
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};     // [m/s^2]
};

struct PredictedTrajectory {
  std::int64_t t_ref{0};                          // [ns] header.stamp
  std::uint32_t seq{0};
  int  n{0};
  bool valid{false};
  std::array<Sample, kMaxSamples> s{};
};

struct Eval {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  bool extrapolated{false};                       // 지평 밖 → Taylor 외삽 (L7 감시 대상)
  bool valid{false};
};

// 5차 Hermite 기저. 구간 길이 h, 국소 좌표 s = dt/h in [0,1].
struct HermiteBasis {
  double H0, H1, H2, H3, H4, H5;                  // 값
  double D0, D1, D2, D3, D4, D5;                  // d/ds
  double S0, S1, S2, S3, S4, S5;                  // d2/ds2
};

[[nodiscard]] inline HermiteBasis hermite5(double s) noexcept {
  const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
  HermiteBasis b{};
  b.H0 = 1.0 - 10.0 * s3 + 15.0 * s4 -  6.0 * s5;
  b.H1 =   s -  6.0 * s3 +  8.0 * s4 -  3.0 * s5;
  b.H2 = 0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5;
  b.H3 =        10.0 * s3 - 15.0 * s4 +  6.0 * s5;
  b.H4 =        -4.0 * s3 +  7.0 * s4 -  3.0 * s5;
  b.H5 =         0.5 * s3 -        s4 +  0.5 * s5;
  b.D0 = -30.0 * s2 +  60.0 * s3 - 30.0 * s4;
  b.D1 =  1.0 - 18.0 * s2 + 32.0 * s3 - 15.0 * s4;
  b.D2 =    s -  4.5 * s2 +  6.0 * s3 -  2.5 * s4;
  b.D3 =  30.0 * s2 -  60.0 * s3 + 30.0 * s4;
  b.D4 = -12.0 * s2 +  28.0 * s3 - 15.0 * s4;
  b.D5 =   1.5 * s2 -   4.0 * s3 +  2.5 * s4;
  b.S0 = -60.0 * s + 180.0 * s2 - 120.0 * s3;
  b.S1 = -36.0 * s +  96.0 * s2 -  60.0 * s3;
  b.S2 =  1.0 - 9.0 * s + 18.0 * s2 - 10.0 * s3;
  b.S3 =  60.0 * s - 180.0 * s2 + 120.0 * s3;
  b.S4 = -24.0 * s +  84.0 * s2 -  60.0 * s3;
  b.S5 =   3.0 * s -  12.0 * s2 +  10.0 * s3;
  return b;
}

// 구간 [A, B] 안에서의 보간. h = B.t - A.t > 0 을 전제한다.
[[nodiscard]] inline Eval interpolate(const Sample& A, const Sample& B, double t) noexcept {
  const double h = B.t - A.t;
  if (!(h > 0.0)) return {A.p, A.v, A.a, false, true};
  const double s = std::clamp((t - A.t) / h, 0.0, 1.0);
  const HermiteBasis b = hermite5(s);
  Eval e{};
  e.p = b.H0 * A.p + (b.H1 * h) * A.v + (b.H2 * h * h) * A.a
      + b.H3 * B.p + (b.H4 * h) * B.v + (b.H5 * h * h) * B.a;
  e.v = (b.D0 * A.p + b.D3 * B.p) / h + b.D1 * A.v + b.D4 * B.v
      + (b.D2 * h) * A.a + (b.D5 * h) * B.a;
  e.a = (b.S0 * A.p + b.S3 * B.p) / (h * h) + (b.S1 * A.v + b.S4 * B.v) / h
      + b.S2 * A.a + b.S5 * B.a;
  e.valid = true;
  return e;
}

// 지평 밖 Taylor 외삽. 짧은 dt 에만 의미가 있다.
[[nodiscard]] inline Eval extrapolate(const Sample& S, double t) noexcept {
  const double d = t - S.t;
  return {S.p + S.v * d + 0.5 * S.a * d * d, S.v + S.a * d, S.a, true, true};
}

// 상대 시각 t [s] 에서 샘플링. hint 는 직전 호출의 구간 인덱스(단조 증가 가정) —
// RT 틱이 시간순이라 평균 O(1) 이다. 범위를 벗어나면 이진 탐색으로 복구한다.
[[nodiscard]] inline Eval sampleAt(const PredictedTrajectory& tr, double t, int& hint) noexcept {
  if (!tr.valid || tr.n <= 0) return {};
  if (tr.n == 1) return extrapolate(tr.s[0], t);
  if (t <= tr.s[0].t)          return extrapolate(tr.s[0], t);
  if (t >= tr.s[tr.n - 1].t)   return extrapolate(tr.s[tr.n - 1], t);

  int i = std::clamp(hint, 0, tr.n - 2);
  if (!(tr.s[i].t <= t && t < tr.s[i + 1].t)) {
    if (i + 1 <= tr.n - 2 && tr.s[i + 1].t <= t && t < tr.s[i + 2].t) {
      ++i;                                        // 가장 흔한 경우: 다음 구간
    } else {
      int lo = 0, hi = tr.n - 1;                  // 이진 탐색 (재동기)
      while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        (tr.s[mid].t <= t) ? lo = mid : hi = mid;
      }
      i = lo;
    }
  }
  hint = i;
  return interpolate(tr.s[i], tr.s[i + 1], t);
}

[[nodiscard]] inline Eval sampleAt(const PredictedTrajectory& tr, double t) noexcept {
  int hint = 0;
  return sampleAt(tr, t, hint);
}

// 샘플 열의 형식 검사 (수신 직후 1회, non-RT). t 단조 증가, 유한값, 개수.
struct TrajCheck {
  bool ok{false};
  bool t_monotonic{true}, finite{true}, count_ok{true};
  double dt_min{0.0}, dt_max{0.0}, horizon{0.0};
};

[[nodiscard]] inline TrajCheck check(const PredictedTrajectory& tr, int n_min) noexcept {
  TrajCheck c{};
  c.count_ok = (tr.n >= n_min && tr.n <= kMaxSamples);
  if (tr.n <= 0) { c.count_ok = false; return c; }
  c.dt_min = 1e300; c.dt_max = 0.0;
  for (int i = 0; i < tr.n; ++i) {
    const auto& s = tr.s[i];
    if (!std::isfinite(s.t) || !s.p.allFinite() || !s.v.allFinite() || !s.a.allFinite()) c.finite = false;
    if (i > 0) {
      const double d = s.t - tr.s[i - 1].t;
      if (!(d > 0.0)) c.t_monotonic = false;
      c.dt_min = std::min(c.dt_min, d);
      c.dt_max = std::max(c.dt_max, d);
    }
  }
  c.horizon = tr.s[tr.n - 1].t - tr.s[0].t;
  c.ok = c.count_ok && c.finite && c.t_monotonic;
  return c;
}

}  // namespace catching::traj
```

RT 규칙: 고정 크기, 할당 없음, `noexcept`, ROS 의존 없음. `sampleAt()`은 hint 커서로 평균 $O(1)$ 이고, 커서가 어긋나면 이진 탐색으로 복구한다($O(\log N)$ 상한).

`Sample`에 공분산이 없는 것은 §4.5의 1안을 전제한 것이다. 2안으로 가면 별도 배열을 병렬로 둔다(구조체를 키우면 RT 복사량이 늘어난다).

### 5.2 L1·L4와의 연결

- L1이 `PointCloud2`를 파싱해 `PredictedTrajectory`(`track_epoch` 포함)를 채우고 SeqLock에 쓴다(L1 §5.1).
- RT 루프는 매 틱 `tryRead` → `sampleAt(tr, t_lead, hint_)` → 결과를 `ref::TargetState{p, v, a}` 로 L4에 넘긴다(L8 §4.1 순서 2).
- `hint_`는 컨트롤러 멤버로 유지하고, **`buf.seq != last_seq_` 이면 0으로 초기화**한다. 정확성은 이진 탐색이 지키지만 틱 비용이 흔들린다.
- `interpolate()` 가 `valid=false` 를 돌려주면(비단조 샘플 쌍, 찢어진 읽기) 그 틱은 stale 로 처리한다.

**스냅샷 복사 비용.** `PredictedTrajectory` 는 `kMaxSamples` 고정이라 실제 $n$ 과 무관하게 전체를 복사한다 — 512 샘플이면 41 KB, 캐시 hot 복사 약 1.1 µs, `seqlock_max_retries=4` 면 최악 5.5 µs에 L1d 전면 축출까지 따른다. 대응은 둘 중 하나다(W2-2 결과에 따라 결정): SeqLock wrapper 가 `n` 까지만 복사하도록 가변 길이 경로를 두거나, `kMaxSamples` 를 G2-1 실측 $N$ 상한으로 줄인다(예: 512 → 128). §4.5의 2안(공분산 일체)은 같은 이유로 **배제한다** — 구조체가 고정 크기라 실제 복사량이 188 KB이고 틱당 최악 940 KB다.

## 6. YAML 파라미터

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `prediction.max_samples` | int | – | 512 | 16–512 | `kMaxSamples`. vision의 $N$ 상한 확인 후 축소 가능 (G2-1) |
| `prediction.n_min` | int | – | `TBD` | ≥2 | `traj::check` 하한. **`io.n_min`(L1 §6)과 같은 키를 쓴다** — L1이 읽어 L2에 넘기며, 두 곳에 따로 두지 않는다 (G2-1) |
| `prediction.t_horizon_margin` | double | s | 0.05 | 0–0.3 | §4.6 지평 끝 여유 |
| `prediction.dt_expected` | double | s | `TBD` | >0 | 카메라 주기. 검사용 (G2-1) |
| `prediction.dt_tol` | double | – | 0.2 | 0–1 | 샘플 간격 허용 상대편차 |
| `prediction.z_floor` | double | m | `TBD` | – | G2-5 |
| `prediction.workcell` | box | m | `TBD` | – | G2-5 |
| `prediction.lead` | double | s | 0.0 | ≥0 | §4.4. **`joint_cmd.lag.T_arm`(L5 §6)에서 파생한다** — `lead = T_arm × lead_enable`(마스터 §6). backend가 이미 보상하면 `lead_enable=0` (W4-2) |

v0.2의 `prediction.rt.*`, `prediction.rollout.*`, `q_acc`, `q_k`는 전부 삭제했다.

## 7. 단위 기술 구현 순서

- **L2.1** `traj_sampler.hpp` + §4.7 테스트 6종 (참조: `test_l2.cpp`, 이식 시 GTest).
- **L2.2** 시각 정렬(§4.4) + `t` 필드 타입 분기 (G2-2 확정 후).
- **L2.3** 지평 감시·외삽 플래그를 L7 입력으로 연결.
- **L2.4** 실제 vision bag으로 재생 테스트: 샘플 간격 분포, $N$ 분포, 지평 길이, 외삽 발생률 기록.

## 8. 디버깅 방법

- 보간 결과와 원 샘플을 같은 그래프에 그린다. 샘플점을 지나지 않으면 시각 정렬(§4.4) 또는 `t` 필드 해석을 의심한다.
- 기준 가속도 $u$ 에 주기적 계단이 보이면 보간이 Taylor로 떨어졌는지 확인한다(§4.2).
- 외삽 플래그가 자주 서면 vision 지평이 짧거나 $T_{lead}$ 가 과대한 것이다.
- 샘플 간격이 `dt_expected`와 다르면 카메라 프레임 드롭 또는 vision 측 리샘플링을 의심한다.
- 궤적이 틱마다 크게 점프하면 vision 예측 갱신 품질 또는 시계 오차를 본다(v0.2 `lastJump()`의 대체 지표: 연속 두 스냅샷의 같은 절대시각 위치 차).

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G2-A | 샘플점 복원 오차 < 1e-12 (p, v, a) | `[SIM-ANY]` |
| G2-B | 샘플 경계 $\Vert\Delta a\Vert$ < 1e-6 m/s² | `[SIM-ANY]` |
| G2-C | 순수 중력 데이터에서 참 궤적 대비 위치 오차 < 1e-10 m | `[SIM-ANY]` |
| G2-D | 외삽 플래그, hint 커서, 형식 검사기 동작 | `[SIM-ANY]` |
| G2-E | RT 샘플링 경로 할당 0, 최악 실행시간·**복사 바이트 수**·L1d miss 기록. 임계는 L8 틱 예산에서 역산 | `[SIM-ANY]` |
| G2-G | `interpolate()` 가 비단조 샘플 쌍에 `valid=false` 반환 | `[SIM-ANY]` |
| G2-F | 실제 vision bag 재생에서 외삽 발생률·샘플 간격 분포 기록 | `[HW-P1B]` |

`test_l2.cpp`가 G2-A~D를 돌린다(v0.3 기준 통과).

## 10. 미확정 항목

TBD-VIS-03, TBD-VIS-04, TBD-VIS-05, TBD-COV-01, TBD-WS-01, `prediction.n_min`, `prediction.dt_expected`, `prediction.lead`.
