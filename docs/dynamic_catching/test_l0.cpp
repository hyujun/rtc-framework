// test_l0.cpp — L0 §4.4 sanity check + 회귀 테스트
//   g++ -std=c++20 -O2 -Wall -Wextra -I/usr/include/eigen3 test_l0.cpp -o test_l0 && ./test_l0
#include "ball_dynamics.hpp"

#include <cmath>
#include <cstdio>

using namespace catching::ball;

static int g_fail = 0;

static void check(bool ok, const char* name, const char* detail) {
  std::printf("  [%s] %-46s %s\n", ok ? "PASS" : "FAIL", name, detail);
  if (!ok)
    ++g_fail;
}

int main() {
  std::printf("L0 ball_dynamics\n");

  // 1) k = 0 이면 해석해와 일치
  {
    Model m;
    State x;
    x << 0, 0, 0, 3, -1, 6, 0.0;
    const double T = 0.8;
    State y = x;
    propagate(m, y, T, 2e-3, 1000);
    const Eigen::Vector3d p_an = x.head<3>() + x.segment<3>(3) * T + 0.5 * m.g * T * T;
    const double err = (y.head<3>() - p_an).norm();
    char b[96];
    std::snprintf(b, sizeof b, "err = %.2e m  (< 1e-9)", err);
    check(err < 1e-9, "1. analytic solution (k=0)", b);
  }

  // 2) 역학적 에너지 비증가: Edot = -k |v|^3
  {
    Model m;
    State x;
    x << 0, 0, 0, 4, 2, 7, 0.0229;
    auto energy = [&](const State& s) {
      return 0.5 * s.segment<3>(3).squaredNorm() - m.g.dot(s.head<3>());
    };
    double E = energy(x), worst = 0.0;
    for (int i = 0; i < 500; ++i) {
      x = rk4(m, x, 2e-3);
      const double E2 = energy(x);
      worst = std::max(worst, E2 - E);
      E = E2;
    }
    char b[96];
    std::snprintf(b, sizeof b, "max dE = %.2e  (<= 0 + rounding)", worst);
    check(worst < 1e-9, "2. mechanical energy non-increasing", b);
  }

  // 3) 스텝 절반 → 전역 오차 약 1/16 (4차 수렴)
  {
    Model m;
    State x0;
    x0 << 0, 0, 0, 5, 1, 8, 0.0229;
    const double T = 0.5;
    auto run = [&](double h) {
      State y = x0;
      propagate(m, y, T, h, 1 << 20);
      return y;
    };
    const State ref = run(1e-5);
    const double e1 = (run(8e-3).head<3>() - ref.head<3>()).norm();
    const double e2 = (run(4e-3).head<3>() - ref.head<3>()).norm();
    const double ratio = e1 / e2;
    char b[96];
    std::snprintf(b, sizeof b, "ratio = %.2f  (12-20)", ratio);
    check(ratio > 12.0 && ratio < 20.0, "3. RK4 4th-order convergence", b);
  }

  // 4) STM vs 중심 유한차분
  {
    Model m;
    State x0;
    x0 << 0.2, -0.1, 1.0, 4, 2, 7, 0.0229;
    const double T = 0.5, h = 2e-3;
    const int n = static_cast<int>(T / h + 0.5);
    auto prop = [&](State s) {
      for (int i = 0; i < n; ++i)
        s = rk4(m, s, h);
      return s;
    };
    State x = x0;
    Mat7 Phi = Mat7::Identity();
    for (int i = 0; i < n; ++i)
      rk4WithStm(m, x, Phi, h);
    Mat7 Fd;
    const double d = 1e-6;
    for (int j = 0; j < 7; ++j) {
      State xp = x0, xm = x0;
      xp(j) += d;
      xm(j) -= d;
      Fd.col(j) = (prop(xp) - prop(xm)) / (2.0 * d);
    }
    const double err = (Phi - Fd).cwiseAbs().maxCoeff();
    char b[96];
    std::snprintf(b, sizeof b, "max|Phi - FD| = %.2e  (< 1e-6)", err);
    check(err < 1e-6, "4. STM vs finite difference", b);
  }

  // 5) 회귀(C8): |v| < v_eps 에서 d(vdot)/dk 가 f() 와 같은 |v| 를 쓰는가.
  //    유한차분은 g=9.81 에 묻혀 자릿수 손실이 나므로 닫힌식 -|v| v 와 직접 비교한다.
  {
    Model m;
    State x;
    x << 0, 0, 0, 2e-4, 0.0, -1e-4, 0.05;  // |v| = 2.24e-4 << v_eps
    const Eigen::Vector3d v = x.segment<3>(3);
    const Eigen::Vector3d exact = -v.norm() * v;
    const Eigen::Vector3d got = jacobian(m, x).block<3, 1>(3, 6);
    const double rel = (got - exact).norm() / exact.norm();
    char b[112];
    std::snprintf(b, sizeof b, "rel err = %.2e  (clamp bug -> v_eps/|v| = %.2f)", rel,
                  m.v_eps / v.norm());
    check(rel < 1e-12, "5. dA/dk consistent with f() at low |v|", b);
  }

  // 6) 회귀(C11): propagate() 의 truncated 계약
  {
    Model m;
    State x;
    x << 0, 0, 0, 4, 0, 6, 0.0229;
    State a = x, b_ = x;
    const auto r1 = propagate(m, a, 0.2, 2e-3, 100);   // 정확히 100 스텝, 포화 아님
    const auto r2 = propagate(m, b_, 0.5, 2e-3, 100);  // 250 요구 → 100 으로 잘림
    char b[128];
    std::snprintf(b, sizeof b, "T=0.2:{n=%d,tr=%d}  T=0.5:{n=%d,tr=%d,h=%.1fms}", r1.steps,
                  int(r1.truncated), r2.steps, int(r2.truncated), 1e3 * r2.h_used);
    check(r1.steps == 100 && !r1.truncated && r2.steps == 100 && r2.truncated,
          "6. propagate() truncated flag", b);
  }

  std::printf(g_fail ? "\nL0: %d FAILED\n" : "\nL0: all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
