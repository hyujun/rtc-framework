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
#include <array>
#include <cmath>
#include <cstdint>

namespace catching::traj {

inline constexpr int kMaxSamples = 512;

// 이식 시 L0 types.hpp 와 함께 컴파일되면 다음을 켠다 (CovBuffer 길이 일치, L0 §5.2):
//   static_assert(kMaxSamples == catching::kMaxTrajSamples);
// 참조 구현은 L0 에 의존하지 않으려고 주석으로 둔다.

// vision PointCloud2 의 한 점 중 RT 경로가 쓰는 부분.
// cov(6x6) 는 non-RT 계획 버퍼에만 둔다 (L2 §4.5, TBD-VIS-04).
struct Sample {
  double t{0.0};                               // [s] header.stamp 기준 상대 시각
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};  // [m]
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};  // [m/s]
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};  // [m/s^2]
};

// 모든 상대시각의 단일 원점은 t_ref (= header.stamp) 다 (L2 §4.4).
struct PredictedTrajectory {
  std::int64_t t_ref{0};  // [ns] header.stamp
  std::uint32_t seq{0};
  std::uint32_t track_epoch{0};  // L1 §4.4. PointCloud2 에 track_id 가 없어
                                 // 제어 PC 가 만들어내는 트랙 식별자.
  int n{0};
  bool valid{false};
  std::array<Sample, kMaxSamples> s{};
};

struct Eval {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  // 지평 앞뒤를 구분한다. vision 이 s[0].t > 0 으로 발행하면 정상 동작 중에도 앞쪽
  // 외삽이 상시 발생하므로, L7 이 감시하는 것은 after_horizon 뿐이다 (L7 §4.2).
  bool before_horizon{false};  // t < s[0].t
  bool after_horizon{false};   // t > s[n-1].t  ← 포구 금지 조건
  bool extrapolated{false};    // = before_horizon || after_horizon
  bool valid{false};
};

// 5차 Hermite 기저. 구간 길이 h, 국소 좌표 s = dt/h in [0,1].
struct HermiteBasis {
  double H0, H1, H2, H3, H4, H5;  // 값
  double D0, D1, D2, D3, D4, D5;  // d/ds
  double S0, S1, S2, S3, S4, S5;  // d2/ds2
};

[[nodiscard]] inline HermiteBasis hermite5(double s) noexcept {
  const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
  HermiteBasis b{};
  b.H0 = 1.0 - 10.0 * s3 + 15.0 * s4 - 6.0 * s5;
  b.H1 = s - 6.0 * s3 + 8.0 * s4 - 3.0 * s5;
  b.H2 = 0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5;
  b.H3 = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
  b.H4 = -4.0 * s3 + 7.0 * s4 - 3.0 * s5;
  b.H5 = 0.5 * s3 - s4 + 0.5 * s5;
  b.D0 = -30.0 * s2 + 60.0 * s3 - 30.0 * s4;
  b.D1 = 1.0 - 18.0 * s2 + 32.0 * s3 - 15.0 * s4;
  b.D2 = s - 4.5 * s2 + 6.0 * s3 - 2.5 * s4;
  b.D3 = 30.0 * s2 - 60.0 * s3 + 30.0 * s4;
  b.D4 = -12.0 * s2 + 28.0 * s3 - 15.0 * s4;
  b.D5 = 1.5 * s2 - 4.0 * s3 + 2.5 * s4;
  b.S0 = -60.0 * s + 180.0 * s2 - 120.0 * s3;
  b.S1 = -36.0 * s + 96.0 * s2 - 60.0 * s3;
  b.S2 = 1.0 - 9.0 * s + 18.0 * s2 - 10.0 * s3;
  b.S3 = 60.0 * s - 180.0 * s2 + 120.0 * s3;
  b.S4 = -24.0 * s + 84.0 * s2 - 60.0 * s3;
  b.S5 = 3.0 * s - 12.0 * s2 + 10.0 * s3;
  return b;
}

// 구간 [A, B] 안에서의 보간. h = B.t - A.t > 0 을 전제한다.
[[nodiscard]] inline Eval interpolate(const Sample& A, const Sample& B, double t) noexcept {
  const double h = B.t - A.t;
  // 단조성은 non-RT 의 check() 가 보장하지만 RT 는 그것을 다시 돌리지 않는다.
  // 찢어진 SeqLock 읽기가 재시도 상한을 넘기면 여기로 들어올 수 있으므로 invalid 로 알린다.
  if (!(h > 0.0))
    return {A.p, A.v, A.a, false, false, false, false};
  const double s = std::clamp((t - A.t) / h, 0.0, 1.0);
  const HermiteBasis b = hermite5(s);
  Eval e{};
  e.p = b.H0 * A.p + (b.H1 * h) * A.v + (b.H2 * h * h) * A.a + b.H3 * B.p + (b.H4 * h) * B.v +
        (b.H5 * h * h) * B.a;
  e.v =
      (b.D0 * A.p + b.D3 * B.p) / h + b.D1 * A.v + b.D4 * B.v + (b.D2 * h) * A.a + (b.D5 * h) * B.a;
  e.a =
      (b.S0 * A.p + b.S3 * B.p) / (h * h) + (b.S1 * A.v + b.S4 * B.v) / h + b.S2 * A.a + b.S5 * B.a;
  e.valid = true;
  return e;
}

// 지평 밖 Taylor 외삽. 짧은 dt 에만 의미가 있다.
[[nodiscard]] inline Eval extrapolate(const Sample& S, double t, bool after) noexcept {
  const double d = t - S.t;
  return {S.p + S.v * d + 0.5 * S.a * d * d, S.v + S.a * d, S.a, !after, after, true, true};
}

// 상대 시각 t [s] 에서 샘플링. hint 는 직전 호출의 구간 인덱스(단조 증가 가정) —
// RT 틱이 시간순이라 평균 O(1) 이다. 범위를 벗어나면 이진 탐색으로 복구한다.
[[nodiscard]] inline Eval sampleAt(const PredictedTrajectory& tr, double t, int& hint) noexcept {
  if (!tr.valid || tr.n <= 0)
    return {};
  if (tr.n == 1)
    return extrapolate(tr.s[0], t, t > tr.s[0].t);
  if (t < tr.s[0].t)
    return extrapolate(tr.s[0], t, false);
  if (t > tr.s[tr.n - 1].t)
    return extrapolate(tr.s[tr.n - 1], t, true);
  if (t == tr.s[tr.n - 1].t)
    return {tr.s[tr.n - 1].p,
            tr.s[tr.n - 1].v,
            tr.s[tr.n - 1].a,
            false,
            false,
            false,
            true};  // 지평 끝은 외삽이 아니다

  int i = std::clamp(hint, 0, tr.n - 2);
  if (!(tr.s[i].t <= t && t < tr.s[i + 1].t)) {
    if (i + 1 <= tr.n - 2 && tr.s[i + 1].t <= t && t < tr.s[i + 2].t) {
      ++i;  // 가장 흔한 경우: 다음 구간
    } else {
      int lo = 0, hi = tr.n - 1;  // 이진 탐색 (재동기)
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
  if (tr.n <= 0) {
    c.count_ok = false;
    return c;
  }
  c.dt_min = 1e300;
  c.dt_max = 0.0;
  for (int i = 0; i < tr.n; ++i) {
    const auto& s = tr.s[i];
    if (!std::isfinite(s.t) || !s.p.allFinite() || !s.v.allFinite() || !s.a.allFinite())
      c.finite = false;
    if (i > 0) {
      const double d = s.t - tr.s[i - 1].t;
      if (!(d > 0.0))
        c.t_monotonic = false;
      c.dt_min = std::min(c.dt_min, d);
      c.dt_max = std::max(c.dt_max, d);
    }
  }
  c.horizon = tr.s[tr.n - 1].t - tr.s[0].t;
  c.ok = c.count_ok && c.finite && c.t_monotonic;
  return c;
}

}  // namespace catching::traj
