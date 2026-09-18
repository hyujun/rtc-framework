#pragma once
// catching_reference/soft_catch_reference.hpp — RT-safe, 고정 크기, noexcept.
//
// L3(계획 rollout), L4(기준 생성), L5(접근축 과제)가 모두 이 헤더를 쓴다.
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace catching::ref {

struct TargetState {  // 추종 대상 (vision 예측 보간값 또는 L7 가상 감속 공), W 기준
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};  // [m]
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};  // [m/s]
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};  // [m/s^2]
};

// gamma(t): [t0, t1]에서 g0 → gf 5차 보간. 양 끝 1·2계 미분 0 → clamp와 일관.
struct GammaProfile {
  double g0{0.0}, gf{0.0}, t0{0.0}, t1{1.0};  // t 는 선행축 상대시간 [s] (L4 §5.2)

  void eval(double t, double& g, double& gd, double& gdd) const noexcept {
    const double T = std::max(t1 - t0, 1e-6);
    const double s = std::clamp((t - t0) / T, 0.0, 1.0);
    const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
    const double d = gf - g0;
    g = g0 + d * (10.0 * s3 - 15.0 * s4 + 6.0 * s5);
    gd = d * (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / T;
    gdd = d * (60.0 * s - 180.0 * s2 + 120.0 * s3) / (T * T);
  }
};

// 시간축 규약 (L4 §5.2):
//   x, xd        : t + dt 기준 (다음 틱에 내보낼 기준 상태)
//   xdd          : [t, t+dt] 구간에 **실제 실현된** 평균 가속도 (속도 포화 반영)
//   u_des        : 포화 전 DS 요구 가속도 (진단, L3 rollout 판정)
//   e, ed        : t 기준 오차 (진단)
struct TranslationOutput {
  Eigen::Vector3d x{Eigen::Vector3d::Zero()}, xd{Eigen::Vector3d::Zero()},
      xdd{Eigen::Vector3d::Zero()};
  Eigen::Vector3d u_des{Eigen::Vector3d::Zero()};
  Eigen::Vector3d e{Eigen::Vector3d::Zero()}, ed{Eigen::Vector3d::Zero()};
  double gamma{0.0}, gamma_d{0.0}, gamma_dd{0.0};  // L8 TickRecord 기록용 (L4 §8)
  bool saturated{false};
};

// derateGamma() 의 결과. L7 은 Rejected 만 "여유 없음"으로 해석한다 (L7 §4.6).
//   Applied : 목표를 gf_new 로 낮췄다.
//   Frozen  : 현재 gamma(t) 가 이미 gf_new 이하라 **현재 값에서 동결**했다.
//             gammadot, gammaddot 가 0 이 되므로 feedforward 의 지배항이 사라진다 —
//             램프 중앙에서는 이것만으로도 |u| 가 크게 준다.
//   Rejected: 상향 요청이거나 목표가 이미 그 값 이하다.
enum class DerateResult : std::uint8_t { Rejected = 0, Frozen = 1, Applied = 2 };

class SoftCatchTranslation {
 public:
  struct Params {
    double omega{10.0};  // [rad/s]  (omega*dt <= 0.828 이 이산 안정 경계, L4 §4.7)
    double zeta{1.0};
    double a_max{15.0};  // [m/s^2]  L7 supervisor.decel.a_dec <= 이 값 이어야 한다 (L7 §4.3)
    double v_max{2.0};   // [m/s]    L3 gammaWindow 의 v_tcp_max 와 같은 값이어야 한다
  };

  explicit SoftCatchTranslation(const Params& p) noexcept : prm_(p) {}

  // 활성화·재무장 시 호출. 기준 상태뿐 아니라 **포구점과 gamma 프로파일까지** 초기화한다.
  // gamma == 0 이면 끌개는 p_c_ 이므로(아래 주의), p_c_ = x 로 두어 현재 자세 유지가 된다.
  // v0.3 이전에는 p_c_ 와 gp_ 가 남아 직전 시행의 포구점으로 복귀하는 결함이 있었다.
  void reset(const Eigen::Vector3d& x, const Eigen::Vector3d& xd) noexcept {
    x_ = x;
    xd_ = xd;
    p_c_ = x;
    gp_ = GammaProfile{};
  }

  // **주의: gamma == 0 이면 대상 o 는 결과에 전혀 영향을 주지 않는다.**
  //   u = -w^2 (x - p_c) - 2 zeta w xd  → 끌개는 오직 p_c_ 다.
  // 따라서 정지 목표(홈 복귀, 대기 자세 유지)는 **p_c 로** 지정해야 한다 (L4 §5.3).
  void setIntercept(const Eigen::Vector3d& p_c, const GammaProfile& gp) noexcept {
    p_c_ = p_c;
    gp_ = gp;
  }

  [[nodiscard]] const Eigen::Vector3d& intercept() const noexcept { return p_c_; }

  [[nodiscard]] const GammaProfile& gamma() const noexcept { return gp_; }

  // 지금 하향하면 ed 가 얼마나 점프하는지 (L7 §4.6 사전 검사).
  //   ||Δed|| = |gammadot(t)| * ||xi^O(t)||
  // 이 값은 t 에 대해 **단조가 아니다** — gammadot 이 램프 중앙에서 최대이므로 봉우리가 있다.
  [[nodiscard]] double derateJump(const TargetState& o, double t) const noexcept {
    double g{}, gd{}, gdd{};
    gp_.eval(t, g, gd, gdd);
    return std::abs(gd) * (o.p - p_c_).norm();
  }

  // COMMITTED 이후에 허용되는 유일한 계획 변경 (L3 §4.7, L7 §4.6).
  // p_c 는 동결한 채 gamma 목표만 **하향**한다. 현재 gamma 값에서 다시 5차 램프를
  // 시작하므로 gamma(t) 는 연속이고 e 도 연속이다. ed 는 derateJump() 만큼 점프한다.
  //
  // 비교 기준은 **현재 값 gamma(t) 가 아니라 목표 gp_.gf** 다. 현재 값을 기준으로
  // 삼으면 램프 상승 구간(gamma(t) << gf)에서 요청이 전부 거부되는데, |u| 가 최대인
  // 곳이 바로 그 구간이다.
  DerateResult derateGamma(double t, double gf_new, double t_ramp) noexcept {
    if (!(gf_new < gp_.gf))
      return DerateResult::Rejected;
    double g{}, gd{}, gdd{};
    gp_.eval(t, g, gd, gdd);
    const double T = std::max(t_ramp, 1e-3);
    if (!(gf_new < g)) {  // 현재 값이 이미 새 목표 이하 → 동결
      gp_ = GammaProfile{g, g, t, t + T};
      return DerateResult::Frozen;
    }
    gp_ = GammaProfile{g, gf_new, t, t + T};
    return DerateResult::Applied;
  }

  // t: gamma profile 과 같은 시간축(선행축) [s], dt: 제어 주기 [s]
  [[nodiscard]] TranslationOutput step(const TargetState& o, double t, double dt) noexcept {
    const Eigen::Vector3d xo = o.p - p_c_;  // 원점 = 포구점
    double g{}, gd{}, gdd{};
    gp_.eval(t, g, gd, gdd);
    const Eigen::Vector3d e = (x_ - p_c_) - g * xo;
    const Eigen::Vector3d ed = xd_ - (g * o.v + gd * xo);
    const double w = prm_.omega;
    const Eigen::Vector3d u_des = g * o.a + 2.0 * gd * o.v + gdd * xo      // feedforward
                                  - w * w * e - 2.0 * prm_.zeta * w * ed;  // e'' = A1 e + A2 e'

    bool sat = false;
    Eigen::Vector3d u = u_des;
    if (const double un = u.norm(); un > prm_.a_max) {
      u *= prm_.a_max / un;
      sat = true;
    }

    const Eigen::Vector3d xd_prev = xd_;
    xd_ += u * dt;  // semi-implicit Euler
    if (const double vn = xd_.norm(); vn > prm_.v_max) {
      xd_ *= prm_.v_max / vn;
      sat = true;
    }
    x_ += xd_ * dt;

    // 속도 포화가 걸리면 u 는 더 이상 실현 가속도가 아니다. 실제 실현값을 돌려준다.
    const Eigen::Vector3d xdd = (dt > 0.0) ? Eigen::Vector3d((xd_ - xd_prev) / dt) : u;
    return {x_, xd_, xdd, u_des, e, ed, g, gd, gdd, sat};
  }

 private:
  Params prm_;
  Eigen::Vector3d x_{Eigen::Vector3d::Zero()}, xd_{Eigen::Vector3d::Zero()},
      p_c_{Eigen::Vector3d::Zero()};
  GammaProfile gp_{};
};

// ---------------------------------------------------------------------------
// 접근축 정렬 (L4 §4.5). L3 §4.2(IK), L5 §4.2(과제)도 같은 함수를 쓴다.
//
// 오차는 **회전벡터** e_a = theta * u_hat 이다 (u_hat = (z x a_d)/||z x a_d||).
// exp([e_a]x) z == a_d 를 정확히 만족하고, ||e_a|| = theta 라 0~pi 에서 연속·단조다.
// ---------------------------------------------------------------------------
struct AxisAlignParams {
  double k_axis{8.0};    // [1/s]
  double w_max{6.0};     // [rad/s]
  double sin_eps{1e-6};  // 축이 수치적으로 정의되는 하한 (|z x a_d|)
};

// z, a_d 는 단위벡터여야 한다.
[[nodiscard]] inline Eigen::Vector3d axisAlignError(const Eigen::Vector3d& z,
                                                    const Eigen::Vector3d& a_d,
                                                    double sin_eps = 1e-6) noexcept {
  const Eigen::Vector3d m = z.cross(a_d);
  const double n = m.norm();
  const double c = z.dot(a_d);
  if (n < sin_eps) {
    if (c > 0.0)
      return Eigen::Vector3d::Zero();  // 이미 정렬 (데드밴드)
    // 반평행: 축이 정의되지 않음 → z 에 수직인 임의 축으로 pi 회전
    const Eigen::Vector3d r =
        (std::abs(z.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    return M_PI * z.cross(r).normalized();
  }
  return (std::atan2(n, c) / n) * m;
}

[[nodiscard]] inline Eigen::Vector3d axisAlignOmega(const Eigen::Vector3d& z,
                                                    const Eigen::Vector3d& a_d,
                                                    const AxisAlignParams& p) noexcept {
  Eigen::Vector3d w = p.k_axis * axisAlignError(z, a_d, p.sin_eps);
  if (const double n = w.norm(); n > p.w_max)
    w *= p.w_max / n;
  return w;  // W 기준, z 에 수직
}

// de_a/dt = J_a * omega  (omega 는 W 표현). L5 에서 J_a = axisAlignJacobian(z,a_d) * J_omega^W.
//
//   m = z x a_d,  c = z^T a_d,  theta = atan2(|m|, c),  f = theta/sin(theta)
//   J_a = f'(c) m m^T + f(c) [a_d]x [z]x ,   f'(c) = (theta*c/sin(theta) - 1) / sin^2(theta)
//
// **sin_eps 는 axisAlignError 와 같은 값을 넘겨야 한다.** 다르면 J_a 가 e_a 의 야코비안이
// 아니게 된다(데드밴드 안에서 e_a 는 상수인데 J_a 는 0 이 아닌 값을 돌려준다).
//
// 소각도 급수는 **c > 0 일 때만** 쓴다. sin(theta) 는 theta->0 과 theta->pi 양쪽에서 0 이라,
// 부호를 보지 않으면 반평행 근처에서 발산해야 할 값이 유한한 값(~2.645)으로 조용히 바뀐다.
// theta->pi 에서의 발산은 축이 정의되지 않기 때문이며, 정의의 결함이 아니라 문제의 성질이다.
[[nodiscard]] inline Eigen::Matrix3d axisAlignJacobian(const Eigen::Vector3d& z,
                                                       const Eigen::Vector3d& a_d,
                                                       double sin_eps = 1e-6) noexcept {
  const Eigen::Vector3d m = z.cross(a_d);
  const double n = m.norm();
  const double c = std::clamp(z.dot(a_d), -1.0, 1.0);
  const double th = std::atan2(n, c);
  if (n < sin_eps && c > 0.0)
    return Eigen::Matrix3d::Zero();  // e_a 데드밴드 → de_a = 0
  double f, fp;
  if (n < 1e-8 && c > 0.0) {  // theta -> 0 급수: f = 1 + th^2/6, f' = -1/3
    f = 1.0 + th * th / 6.0;
    fp = -1.0 / 3.0;
  } else {
    f = th / n;  // c < 0 이고 n -> 0 이면 발산 — 의도된 동작
    fp = (th * c / n - 1.0) / (n * n);
  }
  const Eigen::Matrix3d Sa =
      (Eigen::Matrix3d() << 0.0, -a_d.z(), a_d.y(), a_d.z(), 0.0, -a_d.x(), -a_d.y(), a_d.x(), 0.0)
          .finished();
  const Eigen::Matrix3d Sz =
      (Eigen::Matrix3d() << 0.0, -z.z(), z.y(), z.z(), 0.0, -z.x(), -z.y(), z.x(), 0.0).finished();
  return fp * (m * m.transpose()) + f * (Sa * Sz);
}

// 임계감쇠(zeta=1) 오차의 닫힌해: 계획 단계의 종단 오차 예측용 (L3)
inline void criticallyDampedError(const Eigen::Vector3d& e0, const Eigen::Vector3d& ed0,
                                  double omega, double t, Eigen::Vector3d& e,
                                  Eigen::Vector3d& ed) noexcept {
  const Eigen::Vector3d c = ed0 + omega * e0;
  const double ex = std::exp(-omega * t);
  e = (e0 + c * t) * ex;
  ed = (ed0 - omega * t * c) * ex;
}

}  // namespace catching::ref
