// test_l4.cpp — L4 §4.3/§4.5/§4.7/§4.9 재현 + 회귀 테스트
//   g++ -std=c++20 -O2 -Wall -Wextra -I/usr/include/eigen3 test_l4.cpp -o test_l4 && ./test_l4
//
// 참값 공 궤적은 L0 의 RK4(`ball::propagate`)로 만든다. (자체 Euler 적분을 쓰면
// h=2ms, T=0.8s 에서 계통 오차가 약 7.9 mm 로, 검증 대상인 간극과 같은 자릿수다.)
#include "ball_dynamics.hpp"
#include "soft_catch_reference.hpp"
#include <cstdio>
#include <cmath>

using namespace catching;

static int g_fail = 0;
static void check(bool ok, const char* name, const char* detail) {
  std::printf("  [%s] %-46s %s\n", ok ? "PASS" : "FAIL", name, detail);
  if (!ok) ++g_fail;
}

namespace {

constexpr double kDt = 2e-3, kTc = 0.8, kK = 0.0229;

ball::Model model() { ball::Model m; return m; }

ball::State ballAt(double t) {
  ball::State x; x << -2.5, 0.2, 0.8, 4.0, -0.2, 3.5, kK;
  if (t > 0.0) ball::propagate(model(), x, t, 1e-4, 1 << 20);
  return x;
}

ref::TargetState toTarget(const ball::State& x) {
  return {x.head<3>(), x.segment<3>(3), ball::f(model(), x).segment<3>(3)};
}

struct RunOut { double gap, relv, peak_a, peak_v, gamma; };

// pred_offset: 계획이 믿은 포구점 - 참 포구점 (예측 오차 delta)
RunOut run(const Eigen::Vector3d& pred_offset, double gf, double Tw,
           double a_max = 1e9, double v_max = 1e9) {
  const ball::State xc = ballAt(kTc);
  ref::SoftCatchTranslation::Params prm;
  prm.a_max = a_max; prm.v_max = v_max;
  ref::SoftCatchTranslation ds(prm);
  ds.reset({0.3, 0.0, 0.6}, Eigen::Vector3d::Zero());
  ds.setIntercept(xc.head<3>() + pred_offset, ref::GammaProfile{0.0, gf, kTc - Tw, kTc});

  const int N = static_cast<int>(kTc / kDt + 0.5);
  RunOut r{0, 0, 0, 0, 0};
  ref::TranslationOutput o{};
  for (int k = 0; k < N; ++k) {
    const double t = k * kDt;
    o = ds.step(toTarget(ballAt(t)), t, kDt);
    r.peak_a = std::max(r.peak_a, o.u_des.norm());
    r.peak_v = std::max(r.peak_v, o.xd.norm());
  }
  const ball::State xe = ballAt(kTc);
  r.gap  = (o.x - xe.head<3>()).norm();
  r.relv = (xe.segment<3>(3) - o.xd).norm();
  r.gamma = o.gamma;
  return r;
}

}  // namespace

int main() {
  std::printf("L4 soft_catch_reference\n\n§4.9 표 재현 (omega=10, h=2ms, 포화 없음)\n");
  std::printf("  %-22s %10s %10s %12s %12s\n", "조건", "간극[mm]", "예측[mm]", "상대속도", "예측");
  struct Row { const char* name; double off; double gf; };
  const Row rows[] = {{"delta=0,    gf=0",   0.00, 0.0},
                      {"delta=0,    gf=0.4", 0.00, 0.4},
                      {"delta=3cm,  gf=0",   0.03, 0.0},
                      {"delta=3cm,  gf=0.4", 0.03, 0.4}};
  const double v_tc = ballAt(kTc).segment<3>(3).norm();
  constexpr double kEpsConv = 2e-3;   // [m] omega=10, t_c=0.8s 에서 DS 자체의 잔여 수렴 오차
  bool tab_ok = true;
  for (const auto& r : rows) {
    const auto o = run({0.0, r.off, 0.0}, r.gf, 0.4);
    const double gap_pred = (1.0 - r.gf) * r.off, relv_pred = (1.0 - r.gf) * v_tc;
    std::printf("  %-22s %10.1f %10.1f %12.3f %12.3f\n",
                r.name, 1e3 * o.gap, 1e3 * gap_pred, o.relv, relv_pred);
    tab_ok = tab_ok && (std::abs(o.gap - gap_pred) < kEpsConv + 0.05 * gap_pred)
                    && (std::abs(o.relv - relv_pred) < 0.02 * relv_pred);
  }
  std::printf("\n");
  check(tab_ok, "§4.9 gap/relv within gate", "간극 <2mm+5%, 상대속도 <2%");

  // §4.3 재예측 점프: e' = e - (1-g) dp,  ed' = ed + gdot dp
  {
    ref::SoftCatchTranslation ds(ref::SoftCatchTranslation::Params{10.0, 1.0, 1e9, 1e9});
    ds.reset({0.3, 0.0, 0.6}, Eigen::Vector3d::Zero());
    const Eigen::Vector3d pc(1.0, 0.2, 1.1), dp(0.05, -0.02, 0.01);
    const ref::GammaProfile gp{0.0, 0.4, 0.2, 0.8};
    ds.setIntercept(pc, gp);
    const double t = 0.5;
    const auto tgt = toTarget(ballAt(t));
    for (int k = 0; k < 250; ++k) (void)ds.step(toTarget(ballAt(k * kDt)), k * kDt, kDt);
    const auto a = ds.step(tgt, t, 0.0);          // dt=0: 상태 불변, e/ed 만 읽음
    ds.setIntercept(pc + dp, gp);
    const auto b = ds.step(tgt, t, 0.0);
    double g{}, gd{}, gdd{}; gp.eval(t, g, gd, gdd);
    const double e_err  = (b.e  - (a.e  - (1.0 - g) * dp)).norm();
    const double ed_err = (b.ed - (a.ed + gd * dp)).norm();
    char s[96]; std::snprintf(s, sizeof s, "|de|=%.1e |ded|=%.1e  (< 1e-9)", e_err, ed_err);
    check(e_err < 1e-9 && ed_err < 1e-9, "§4.3 re-plan jump formula", s);
  }

  // §4.7 이산 안정 경계 s = omega*h < 2*sqrt(2)-2
  {
    auto diverges = [](double s) {
      const double h = 2e-3, w = s / h;
      ref::SoftCatchTranslation ds(ref::SoftCatchTranslation::Params{w, 1.0, 1e12, 1e12});
      ds.reset({1.0, 0.0, 0.0}, Eigen::Vector3d::Zero());
      ds.setIntercept(Eigen::Vector3d::Zero(), ref::GammaProfile{0, 0, 0, 1});
      const ref::TargetState o{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
      double last = 1.0;
      for (int k = 0; k < 4000; ++k) last = ds.step(o, k * h, h).x.norm();
      return !(last < 1.0);
    };
    // 경계 2sqrt(2)-2 = 0.828427 을 실제로 고정하려면 0.80/0.85 로는 부족하다
    // (분해능 +-2.5%. 감쇠를 2.0*zeta*w -> 1.96*zeta*w 로 바꿔도 통과한다).
    char s[128]; std::snprintf(s, sizeof s, "s=0.827 conv=%d, s=0.830 div=%d, bound=%.6f",
                              int(!diverges(0.827)), int(diverges(0.830)), 2 * std::sqrt(2.0) - 2);
    check(!diverges(0.827) && diverges(0.830), "§4.7 discrete stability boundary (+-0.2%)", s);
  }

  // §4.5 축 정렬: 회전벡터 오차의 연속성·단조성 + 정확성 + Jacobian
  {
    ref::AxisAlignParams ap;
    std::printf("\n§4.5 접근축 정렬 (k_axis=%.0f, w_max=%.0f)\n  angle[deg] :", ap.k_axis, ap.w_max);
    double prev = -1.0, max_jump = 0.0, max_exact = 0.0;
    for (int i = 0; i <= 180; ++i) {
      const double th = i * M_PI / 180.0;
      const Eigen::Vector3d z(0, 0, 1), ad(std::sin(th), 0, std::cos(th));
      const double n = ref::axisAlignOmega(z, ad, ap).norm();
      if (prev >= 0.0) max_jump = std::max(max_jump, std::abs(n - prev));
      prev = n;
      const Eigen::Vector3d e = ref::axisAlignError(z, ad);
      const double a = e.norm();
      const Eigen::Matrix3d R = (a < 1e-12) ? Eigen::Matrix3d::Identity()
                              : Eigen::Matrix3d(Eigen::AngleAxisd(a, e / a));
      max_exact = std::max(max_exact, (R * z - ad).norm());
      if (i % 30 == 0 || i == 179) std::printf(" %d:%.2f", i, n);
    }
    std::printf("\n");
    char s[128]; std::snprintf(s, sizeof s, "max 1deg jump = %.3f rad/s (sin 기반은 5.6)", max_jump);
    check(max_jump < 0.2, "§4.5 omega continuous in misalignment", s);
    std::snprintf(s, sizeof s, "max |exp(e_a) z - a_d| = %.1e", max_exact);
    check(max_exact < 1e-12, "§4.5 e_a is the exact rotation vector", s);

    // Jacobian vs 유한차분
    double worst = 0.0;
    for (int i = 1; i <= 170; ++i) {
      const double th = i * M_PI / 180.0, h = 1e-7;
      const Eigen::Vector3d z(0, 0, 1), ad(std::sin(th), 0, std::cos(th)), om(0.3, -0.7, 0.5);
      const Eigen::Matrix3d dR(Eigen::AngleAxisd(om.norm() * h, om.normalized()));
      const Eigen::Vector3d fd = (ref::axisAlignError(dR * z, ad) - ref::axisAlignError(z, ad)) / h;
      worst = std::max(worst, (fd - ref::axisAlignJacobian(z, ad) * om).cwiseAbs().maxCoeff());
    }
    std::snprintf(s, sizeof s, "max err = %.1e  (1-170 deg)", worst);
    check(worst < 1e-5, "§4.5 axis Jacobian vs finite difference", s);
  }

  // 회귀(C3): 속도 포화 시 반환 xdd 가 실현 가속도와 일치
  {
    ref::SoftCatchTranslation ds(ref::SoftCatchTranslation::Params{10.0, 1.0, 1e9, 0.5});
    ds.reset({0, 0, 0}, {0.49, 0, 0});
    ds.setIntercept({5, 0, 0}, ref::GammaProfile{0, 0, 0, 1});
    const auto o = ds.step({{5, 0, 0}, {0, 0, 0}, {0, 0, 0}}, 0.0, kDt);
    const double realized = (o.xd.x() - 0.49) / kDt;
    char s[112]; std::snprintf(s, sizeof s, "xdd=%.3f realized=%.3f u_des=%.1f sat=%d",
                               o.xdd.x(), realized, o.u_des.x(), int(o.saturated));
    check(std::abs(o.xdd.x() - realized) < 1e-9 && o.saturated,
          "C3 xdd == realized accel under v-saturation", s);
  }

  // D3: gamma 하향 — 세 분기(Applied / Frozen / Rejected)와 연속성
  {
    const ball::State xc = ballAt(kTc);
    auto make = [&](double t_stop) {
      ref::SoftCatchTranslation ds(ref::SoftCatchTranslation::Params{10, 1, 1e9, 1e9});
      ds.reset({0.3, 0.0, 0.6}, Eigen::Vector3d::Zero());
      ds.setIntercept(xc.head<3>(), ref::GammaProfile{0.0, 0.4, 0.0, 0.45});
      for (int k = 0; k * kDt < t_stop; ++k) (void)ds.step(toTarget(ballAt(k * kDt)), k * kDt, kDt);
      return ds;
    };
    // 램프 상승 구간(gamma(t) << gf): v0.3 이전에는 여기서 전부 거부됐다.
    auto d1 = make(0.20);
    const auto r1 = d1.derateGamma(0.20, 0.30, 0.05);           // gf 0.4 -> 0.3, gamma(0.2)=0.159
    auto d2 = make(0.40);
    const auto r2 = d2.derateGamma(0.40, 0.30, 0.05);           // gamma(0.4)=0.385 > 0.3
    auto d3 = make(0.40);
    const auto r3 = d3.derateGamma(0.40, 0.50, 0.05);           // 상향 → 거부
    char s[192];
    std::snprintf(s, sizeof s, "ramp-up(g=0.159)->%s  late(g=0.385)->%s  up->%s",
                  r1 == ref::DerateResult::Frozen ? "Frozen" : (r1 == ref::DerateResult::Applied ? "Applied" : "Rejected"),
                  r2 == ref::DerateResult::Applied ? "Applied" : (r2 == ref::DerateResult::Frozen ? "Frozen" : "Rejected"),
                  r3 == ref::DerateResult::Rejected ? "Rejected" : "?");
    check(r1 == ref::DerateResult::Frozen && r2 == ref::DerateResult::Applied
          && r3 == ref::DerateResult::Rejected, "D3a derateGamma three branches", s);
  }

  // D3b: gamma, e 연속 + ed 점프 = derateJump() 예측값
  {
    ref::SoftCatchTranslation ds(ref::SoftCatchTranslation::Params{10, 1, 1e9, 1e9});
    ds.reset({0.3, 0.0, 0.6}, Eigen::Vector3d::Zero());
    const ball::State xc = ballAt(kTc);
    ds.setIntercept(xc.head<3>(), ref::GammaProfile{0.0, 0.4, 0.4, kTc});
    const double t = 0.70;
    for (int k = 0; k * kDt < t; ++k) (void)ds.step(toTarget(ballAt(k * kDt)), k * kDt, kDt);
    const auto tgt = toTarget(ballAt(t));
    const auto a = ds.step(tgt, t, 0.0);
    const double pred = ds.derateJump(tgt, t);                  // 사전 예측 (L7 §4.6)
    ds.derateGamma(t, 0.15, 0.05);
    const auto b = ds.step(tgt, t, 0.0);
    char s[176];
    std::snprintf(s, sizeof s, "dgamma=%.1e de=%.1e |ded|=%.4f pred=%.4f",
                  std::abs(b.gamma - a.gamma), (b.e - a.e).norm(), (b.ed - a.ed).norm(), pred);
    check(std::abs(b.gamma - a.gamma) < 1e-12 && (b.e - a.e).norm() < 1e-12
          && std::abs((b.ed - a.ed).norm() - pred) < 1e-9,
          "D3b derate: gamma/e continuous, ed jump == derateJump()", s);
  }

  // D3c: ed 점프는 시간에 **단조가 아니다** — 램프 중앙에 봉우리가 있다 (L7 §4.6)
  {
    ref::SoftCatchTranslation ds(ref::SoftCatchTranslation::Params{10, 1, 1e9, 1e9});
    const ball::State xc = ballAt(kTc);
    ds.setIntercept(xc.head<3>(), ref::GammaProfile{0.0, 0.4, 0.4, kTc});
    double peak = 0.0, peak_t = 0.0, at_early = 0.0, at_late = 0.0;
    for (double t = 0.41; t < kTc; t += 0.01) {
      const double j = ds.derateJump(toTarget(ballAt(t)), t);
      if (j > peak) { peak = j; peak_t = t; }
      if (std::abs(t - 0.45) < 5e-3) at_early = j;
      if (std::abs(t - 0.76) < 5e-3) at_late  = j;
    }
    char s[176];
    std::snprintf(s, sizeof s, "peak %.3f at t=%.2f ; t=0.45:%.3f t=0.76:%.3f (단조 아님)",
                  peak, peak_t, at_early, at_late);
    check(peak > at_early && peak > at_late && peak_t > 0.45 && peak_t < 0.72,
          "D3c derate jump is NOT monotone in t", s);
  }

  // A5: retreat — gamma=0 이면 대상이 아니라 p_c 가 끌개다 (L4 §5.3)
  {
    auto converge = [](Eigen::Vector3d target, Eigen::Vector3d p_c) {
      ref::SoftCatchTranslation ds(ref::SoftCatchTranslation::Params{3, 1, 1e9, 1e9});
      ds.reset({0.5, 0.3, 0.9}, Eigen::Vector3d::Zero());
      ds.setIntercept(p_c, ref::GammaProfile{0, 0, 0, 1});
      ref::TranslationOutput o{};
      for (int k = 0; k < 3000; ++k) o = ds.step({target, {0, 0, 0}, {0, 0, 0}}, k * kDt, kDt);
      return o.x;
    };
    const Eigen::Vector3d home(0, 0, 1.2), stale_pc(1.5, 0.2, 0.8);
    const auto wrong = converge(home, stale_pc);      // 문서 v0.2 방식: 대상만 홈으로
    const auto right = converge(home, home);          // v0.3 방식: p_c 를 홈으로
    char s[176];
    std::snprintf(s, sizeof s, "target-only: (%.2f %.2f %.2f) != home ; p_c=home: err %.1e",
                  wrong.x(), wrong.y(), wrong.z(), (right - home).norm());
    check((wrong - home).norm() > 1.0 && (right - home).norm() < 1e-6,
          "A5 retreat attractor is p_c, not the target", s);
  }

  // reset() 이 p_c, gamma 프로파일까지 지우는가 (재무장 오염 방지)
  {
    ref::SoftCatchTranslation ds(ref::SoftCatchTranslation::Params{3, 1, 1e9, 1e9});
    ds.setIntercept({9, 9, 9}, ref::GammaProfile{0.0, 0.8, 0.0, 1.0});
    const Eigen::Vector3d hold(0.1, -0.2, 1.0);
    ds.reset(hold, Eigen::Vector3d::Zero());
    ref::TranslationOutput o{};
    for (int k = 0; k < 2000; ++k) o = ds.step({{5, 5, 5}, {1, 1, 1}, {0, 0, 0}}, k * kDt, kDt);
    char s[160]; std::snprintf(s, sizeof s, "hold err = %.1e m, gamma = %.3f",
                               (o.x - hold).norm(), o.gamma);
    check((o.x - hold).norm() < 1e-9 && o.gamma == 0.0,
          "reset() clears p_c and gamma profile", s);
  }

  std::printf(g_fail ? "\nL4: %d FAILED\n" : "\nL4: all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
