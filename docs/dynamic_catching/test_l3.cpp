// test_l3.cpp — L3 §4.3/§4.5/§4.8 재현 + 회귀 테스트, verify_l3.py 용 cases.txt 생성
//   g++ -std=c++20 -O2 -Wall -Wextra -I/usr/include/eigen3 test_l3.cpp -o test_l3 && ./test_l3
//   python3 verify_l3.py            # 같은 디렉터리에서 (numpy, scipy 필요)
#include "ball_dynamics.hpp"
#include "soft_catch_reference.hpp"
#include "time_feasibility.hpp"

#include <cmath>
#include <cstdio>
#include <random>

using namespace catching;

static int g_fail = 0;

static void check(bool ok, const char* name, const char* detail) {
  std::printf("  [%s] %-46s %s\n", ok ? "PASS" : "FAIL", name, detail);
  if (!ok)
    ++g_fail;
}

namespace {

constexpr double kDt = 2e-3, kTc = 0.8, kK = 0.0229;

ball::State ballAt(double t) {
  ball::State x;
  x << -2.5, 0.2, 0.8, 4.0, -0.2, 3.5, kK;
  if (t > 0.0)
    ball::propagate(ball::Model{}, x, t, 1e-4, 1 << 20);
  return x;
}

ref::TargetState toTarget(const ball::State& x) {
  return {x.head<3>(), x.segment<3>(3), ball::f(ball::Model{}, x).segment<3>(3)};
}

// §4.8 rollout: 창 [t_c - T_w, t_c] 에서 포화 없는 요구 가속도·속도의 최대값
struct Peak {
  double a, v;
};

Peak gammaRollout(double gf, double Tw) {
  ref::SoftCatchTranslation ds(ref::SoftCatchTranslation::Params{10.0, 1.0, 1e9, 1e9});
  ds.reset({0.3, 0.0, 0.6}, Eigen::Vector3d::Zero());
  ds.setIntercept(ballAt(kTc).head<3>(), ref::GammaProfile{0.0, gf, kTc - Tw, kTc});
  Peak pk{0.0, 0.0};
  const int N = static_cast<int>(kTc / kDt + 0.5);
  for (int k = 0; k < N; ++k) {
    const double t = k * kDt;
    const auto o = ds.step(toTarget(ballAt(t)), t, kDt);
    if (t >= kTc - Tw) {
      pk.a = std::max(pk.a, o.u_des.norm());
      pk.v = std::max(pk.v, o.xd.norm());
    }
  }
  return pk;
}

}  // namespace

int main() {
  std::printf("L3 time_feasibility / gamma window / gamma rollout\n");

  // §4.3 tMin: 고정 케이스 + verify_l3.py 용 cases.txt
  {
    std::mt19937_64 rng(12345);
    std::uniform_real_distribution<double> uq(-2.0, 2.0), ua(5.0, 20.0), uw(-M_PI, M_PI);
    FILE* fp = std::fopen("cases.txt", "w");
    for (int i = 0; i < 40; ++i) {
      const double q0 = uq(rng), q1 = uq(rng), a = ua(rng), w0 = uw(rng), wm = M_PI;
      if (fp)
        std::fprintf(fp, "%.17g %.17g %.17g %.17g %.17g %.17g\n", q0, w0, q1, wm, a,
                     plan::tMin(q0, w0, q1, wm, a));
    }
    if (fp)
      std::fclose(fp);

    // 손으로 검산 가능한 네 분기 전부. w0 = 0 만 쓰면 reverse/overshoot 분기와
    // tRest() 가 어느 assertion 도 통과하지 않는다.
    const double a = 8.0;
    // (1) 삼각 (w0=0): 2 sqrt(D/a)
    const double t_tri = plan::tMin(0.0, 0.0, 1.0, 100.0, a);
    const double e_tri = std::abs(t_tri - 2.0 * std::sqrt(1.0 / a));
    // (2) 사다리꼴 (w0=0): D/wm + wm/a
    const double t_trap = plan::tMin(0.0, 0.0, 4.0, 2.0, a);
    const double e_trap = std::abs(t_trap - (4.0 / 2.0 + 2.0 / a));
    // (3) reverse (w0 < 0, 목표는 +): |w0|/a 로 제동 후 D + w0^2/2a 를 정지상태 이동
    const double w0 = -4.0, D3 = 1.0;
    const double t_rev = plan::tMin(0.0, w0, D3, 100.0, a);
    const double e_rev = std::abs(t_rev - (4.0 / a + 2.0 * std::sqrt((D3 + 16.0 / (2 * a)) / a)));
    // (4) overshoot (d_stop > D): w0/a 로 제동 후 (d_stop - D) 만큼 복귀
    const double w4 = 5.0, D4 = 0.5, ds4 = w4 * w4 / (2 * a);  // 1.5625 > 0.5
    const double t_ovr = plan::tMin(0.0, w4, D4, 100.0, a);
    const double e_ovr = std::abs(t_ovr - (w4 / a + 2.0 * std::sqrt((ds4 - D4) / a)));
    // (5) tRest 의 사다리꼴 가지 (속도 한계에 걸리는 정지→정지 이동)
    const double t_rest = plan::tRest(4.0, 2.0, a);
    const double e_rest = std::abs(t_rest - (4.0 / 2.0 + 2.0 / a));
    char s[176];
    std::snprintf(s, sizeof s,
                  "tri=%.1e trap=%.1e rev=%.1e ovr=%.1e tRest=%.1e ; cases.txt 40 rows", e_tri,
                  e_trap, e_rev, e_ovr, e_rest);
    check(e_tri < 1e-12 && e_trap < 1e-12 && e_rev < 1e-12 && e_ovr < 1e-12 && e_rest < 1e-12,
          "§4.3 tMin closed form (4 branches + tRest)", s);
  }

  // 회귀(C9): |w0| > w_max 는 clamp 되고 플래그가 선다 (이전에는 음수 구간을 포함한 값을 반환)
  {
    const auto r = plan::tMinChecked(0.0, 6.0, 2.0, M_PI, 10.0);
    const auto ok = plan::tMinChecked(0.0, 3.0, 2.0, M_PI, 10.0);
    char s[112];
    std::snprintf(s, sizeof s, "w0=6>pi -> clamped=%d t=%.4f ; w0=3 -> clamped=%d",
                  int(r.w0_clamped), r.t, int(ok.w0_clamped));
    check(r.w0_clamped && !ok.w0_clamped && r.t > 0.0, "C9 tMin rejects |w0| > w_max", s);
  }

  // §4.5 g_min 값 고정. boolean 만 보면 1 - d/(v T) 를 0.9 - ... 로 바꿔도 통과한다
  // (clamp 가 음수를 0 으로 가려준다).
  {
    const auto w1 = plan::gammaWindow(5.0, 10.0, 10.0, 0.04, 0.02);  // 1 - 0.04/(5*0.02) = 0.6
    const auto w2 = plan::gammaWindow(4.0, 10.0, 10.0, 0.05, 0.05);  // 1 - 0.05/(4*0.05) = 0.75
    char s[128];
    std::snprintf(s, sizeof s, "g_min: %.6f (기대 0.6) , %.6f (기대 0.75)", w1.g_min, w2.g_min);
    check(std::abs(w1.g_min - 0.6) < 1e-12 && std::abs(w2.g_min - 0.75) < 1e-12,
          "§4.5 g_min value fixed", s);
  }

  // [R1] sanity: 정지 포구, 포켓 3cm, 6 m/s -> T_close <= 5 ms.
  // 5 ms 정확히는 g_min == g_max == 0 인 knife-edge 라 여유를 두고 양옆에서 본다.
  {
    const double v = 6.0, d = 0.03;
    const auto w_ok = plan::gammaWindow(v, 0.0, 10.0, d, 0.0049);   // 5 ms 보다 약간 빠름
    const auto w_bad = plan::gammaWindow(v, 0.0, 10.0, d, 0.0051);  // 약간 느림 -> 창 없음
    const double t_crit = d / v;                                    // = 0.005 s
    char s[160];
    std::snprintf(s, sizeof s, "4.9ms ok=%d(g_min=%.4f) ; 5.1ms ok=%d(g_min=%.4f) ; 임계 %.4f ms",
                  int(w_ok.feasible()), w_ok.g_min, int(w_bad.feasible()), w_bad.g_min,
                  1e3 * t_crit);
    check(w_ok.feasible() && !w_bad.feasible() && std::abs(t_crit - 0.005) < 1e-12,
          "§4.5 [R1] 3cm / 6 m/s -> T_close <= 5 ms", s);
  }

  // 회귀: 비물리적 입력(음수 v_dir_max)에 clamp 가 판정을 뒤집지 않는가
  {
    bool bad1 = false, bad2 = false;
    const auto w_neg = plan::gammaWindow(1.0, -1.0, 5.0, 0.04, 0.06, &bad1);
    const auto w_pos = plan::gammaWindow(1.0, 1.0, 5.0, 0.04, 0.06, &bad2);
    char s[144];
    std::snprintf(s, sizeof s, "v_dir=-1 -> invalid=%d feasible=%d ; v_dir=+1 -> invalid=%d",
                  int(bad1), int(w_neg.feasible()), int(bad2));
    check(bad1 && !w_neg.feasible() && !bad2 && w_pos.feasible(),
          "C-guard gammaWindow rejects non-physical input", s);
  }

  // 회귀(C6): v_tcp_max 가 g_max 를 실제로 구속하는가
  {
    const auto a = plan::gammaWindow(4.0, 3.0, 3.0, 0.04, 0.06);  // v_dir 3.0, v_tcp 3.0
    const auto b = plan::gammaWindow(4.0, 3.0, 1.0, 0.04, 0.06);  // v_tcp 1.0 이 구속
    char s[112];
    std::snprintf(s, sizeof s, "g_max: v_tcp=3 -> %.3f ; v_tcp=1 -> %.3f", a.g_max, b.g_max);
    check(std::abs(a.g_max - 0.75) < 1e-12 && std::abs(b.g_max - 0.25) < 1e-12,
          "C6 gammaWindow honours v_tcp_max", s);
  }

  // 창이 비는 임계 속력
  {
    const double v_lim = plan::maxCatchableSpeed(1.5, 2.0, 0.04, 0.06);
    const bool at6 = plan::gammaWindow(6.0, 1.5, 2.0, 0.04, 0.06).feasible();
    const bool at2 = plan::gammaWindow(2.0, 1.5, 2.0, 0.04, 0.06).feasible();
    char s[128];
    std::snprintf(s, sizeof s, "v_lim=%.3f m/s (v_dir=1.5, d=4cm, T=60ms) ; 6:%d 2:%d", v_lim,
                  int(at6), int(at2));
    check(std::abs(v_lim - (1.5 + 0.04 / 0.06)) < 1e-12 && !at6 && at2, "§4.5 max catchable speed",
          s);
  }

  // §4.8 gamma rollout 표
  std::printf("\n§4.8 gamma rollout — 창 내 최대 |u_des| [m/s^2] / 최대 |v| [m/s], 포화 없음\n");
  std::printf("  %-8s", "T_w\\gf");
  const double gfs[] = {0.1, 0.2, 0.3, 0.4, 0.5};
  for (double gf : gfs)
    std::printf("%13.1f", gf);
  std::printf("\n");
  for (double Tw : {0.30, 0.45, 0.60, 0.75}) {
    std::printf("  %-8.2f", Tw);
    for (double gf : gfs) {
      const auto p = gammaRollout(gf, Tw);
      std::printf("%9.1f/%.2f", p.a, p.v);
    }
    std::printf("\n");
  }
  {
    const auto p = gammaRollout(0.4, 0.30);
    char s[112];
    std::snprintf(s, sizeof s, "(T_w=0.30, gf=0.4) -> %.1f m/s^2 (문서 33.0 +-5%%)", p.a);
    check(std::abs(p.a - 33.0) < 0.05 * 33.0, "§4.8 table reproducible", s);
  }

  std::printf(g_fail ? "\nL3: %d FAILED\n" : "\nL3: all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
