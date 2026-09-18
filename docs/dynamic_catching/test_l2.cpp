// test_l2.cpp — L2 궤적 샘플러(5차 Hermite) 검증
//   g++ -std=c++20 -O2 -Wall -Wextra -I/usr/include/eigen3 test_l2.cpp -o test_l2 && ./test_l2
#include "ball_dynamics.hpp"
#include "traj_sampler.hpp"
#include <cstdio>
#include <cmath>

using namespace catching;

static int g_fail = 0;
static void check(bool ok, const char* name, const char* detail) {
  std::printf("  [%s] %-48s %s\n", ok ? "PASS" : "FAIL", name, detail);
  if (!ok) ++g_fail;
}

namespace {

constexpr double kCam = 1.0 / 60.0;     // vision 샘플 간격 [s]
constexpr double kRt  = 2e-3;           // RT 틱 [s]

// vision 예측 궤적을 흉내낸다. drag_k = 0 이면 순수 중력(2차) 모델.
traj::PredictedTrajectory makeTraj(int n, double drag_k) {
  ball::Model m;
  ball::State x; x << -2.5, 0.2, 0.8, 4.0, -0.2, 3.5, drag_k;
  traj::PredictedTrajectory tr{};
  tr.n = n; tr.valid = true; tr.t_ref = 0;
  for (int i = 0; i < n; ++i) {
    tr.s[i].t = i * kCam;
    tr.s[i].p = x.head<3>();
    tr.s[i].v = x.segment<3>(3);
    tr.s[i].a = ball::f(m, x).segment<3>(3);
    ball::propagate(m, x, kCam, 1e-4, 1 << 20);
  }
  return tr;
}

}  // namespace

int main() {
  std::printf("L2 traj_sampler (5차 Hermite)\n");

  // 1) 샘플점에서 p, v, a 를 정확히 복원 — 각 구간의 **양 끝** 모두.
  //    s=0 만 보면 D1(0)=1 외에 기저 미분 계수가 전혀 구속되지 않는다.
  {
    const auto tr = makeTraj(60, 0.0229);
    double ep = 0, ev = 0, ea = 0;
    for (int i = 0; i + 1 < tr.n; ++i) {
      const auto& A = tr.s[i]; const auto& B = tr.s[i + 1];
      const auto e0 = traj::interpolate(A, B, A.t);      // s = 0
      const auto e1 = traj::interpolate(A, B, B.t);      // s = 1  ← v0.3 이전 미검증
      ep = std::max({ep, (e0.p - A.p).norm(), (e1.p - B.p).norm()});
      ev = std::max({ev, (e0.v - A.v).norm(), (e1.v - B.v).norm()});
      ea = std::max({ea, (e0.a - A.a).norm(), (e1.a - B.a).norm()});
    }
    char b[128]; std::snprintf(b, sizeof b, "p=%.1e v=%.1e a=%.1e (s=0 and s=1)", ep, ev, ea);
    check(ep < 1e-12 && ev < 1e-12 && ea < 1e-12, "1. interpolant matches samples at both ends", b);
  }

  // 1b) 구간 **내부**의 v, a 가 p 의 실제 미분인가 (h 차수 포함 검증).
  //     이것이 없으면 D0..D5 / S0..S5 계수와 h 거듭제곱이 사실상 무검증이다.
  {
    const auto tr = makeTraj(60, 0.0229);
    const double d = 1e-6;
    double ev = 0, ea = 0;
    for (int i = 0; i + 1 < tr.n; i += 7) {
      const auto& A = tr.s[i]; const auto& B = tr.s[i + 1];
      for (double s : {0.15, 0.37, 0.5, 0.63, 0.88}) {
        const double t = A.t + s * (B.t - A.t);
        const auto c = traj::interpolate(A, B, t);
        const auto m = traj::interpolate(A, B, t - d);
        const auto p = traj::interpolate(A, B, t + d);
        ev = std::max(ev, (c.v - (p.p - m.p) / (2 * d)).norm());
        ea = std::max(ea, (c.a - (p.p - 2 * c.p + m.p) / (d * d)).norm());
      }
    }
    char b[128]; std::snprintf(b, sizeof b, "|v - dp/dt|=%.1e  |a - d2p/dt2|=%.1e", ev, ea);
    check(ev < 1e-6 && ea < 1e-2, "1b. v, a are true derivatives of p (interior)", b);
  }

  // 2) C^2: 샘플 경계 좌우에서 a 가 이어진다
  {
    const auto tr = makeTraj(60, 0.0229);
    const double d = 1e-7;
    double jump_h = 0.0;
    for (int i = 1; i < tr.n - 1; ++i) {
      const double t = tr.s[i].t;
      jump_h = std::max(jump_h, (traj::sampleAt(tr, t + d).a - traj::sampleAt(tr, t - d).a).norm());
    }
    // 비교군: 한 샘플만 쓰는 Taylor 전개 (구간마다 a 가 계단)
    double jump_t = 0.0;
    for (int i = 1; i < tr.n - 1; ++i)
      jump_t = std::max(jump_t, (tr.s[i].a - tr.s[i - 1].a).norm());
    char b[128]; std::snprintf(b, sizeof b, "Hermite %.1e m/s^2 vs Taylor(1-sample) %.3e m/s^2", jump_h, jump_t);
    check(jump_h < 1e-6, "2. C2 across sample boundaries", b);
  }

  // 3) 순수 중력(a 상수)이면 5차 Hermite 가 참 궤적을 정확히 재현
  {
    const auto tr = makeTraj(60, 0.0);
    ball::Model m; ball::State x; x << -2.5, 0.2, 0.8, 4.0, -0.2, 3.5, 0.0;
    double worst = 0.0;
    for (double t = 0.0; t < 0.9; t += kRt) {
      ball::State y = x;
      if (t > 0) ball::propagate(m, y, t, 1e-4, 1 << 20);
      worst = std::max(worst, (traj::sampleAt(tr, t).p - y.head<3>()).norm());
    }
    char b[128]; std::snprintf(b, sizeof b, "max pos err = %.2e m (2차 궤적은 5차 기저에 포함)", worst);
    check(worst < 1e-10, "3. exact for pure-gravity (drag-free) model", b);
  }

  // 4) 항력이 있으면 보간 오차가 남는다 — 얼마나 남는지 기록
  {
    const auto tr = makeTraj(60, 0.0229);
    ball::Model m; ball::State x; x << -2.5, 0.2, 0.8, 4.0, -0.2, 3.5, 0.0229;
    double wp = 0.0, wa = 0.0;
    for (double t = 0.0; t < 0.9; t += kRt) {
      ball::State y = x;
      if (t > 0) ball::propagate(m, y, t, 1e-4, 1 << 20);
      const auto e = traj::sampleAt(tr, t);
      wp = std::max(wp, (e.p - y.head<3>()).norm());
      wa = std::max(wa, (e.a - ball::f(m, y).segment<3>(3)).norm());
    }
    char b[128]; std::snprintf(b, sizeof b, "pos %.2e m, accel %.2e m/s^2 (dt_cam=%.1f ms)", wp, wa, 1e3 * kCam);
    check(wp < 1e-9, "4. drag model: interpolation error at 60 Hz spacing", b);
  }

  // 5) 지평 밖은 외삽 플래그
  {
    const auto tr = makeTraj(20, 0.0229);                // 지평 = 19/60 = 0.3167 s
    const auto in  = traj::sampleAt(tr, 0.15);
    const auto out = traj::sampleAt(tr, 0.50);
    const auto pre = traj::sampleAt(tr, -0.01);
    const auto end = traj::sampleAt(tr, tr.s[tr.n - 1].t);      // 지평 끝 정확히
    char b[160];
    std::snprintf(b, sizeof b, "in=%d/%d out=%d/%d pre=%d/%d end=%d/%d (horizon %.4f s)",
                  int(in.before_horizon), int(in.after_horizon),
                  int(out.before_horizon), int(out.after_horizon),
                  int(pre.before_horizon), int(pre.after_horizon),
                  int(end.before_horizon), int(end.after_horizon), tr.s[tr.n - 1].t);
    check(!in.extrapolated && out.after_horizon && !out.before_horizon
          && pre.before_horizon && !pre.after_horizon
          && !end.extrapolated && end.valid,
          "5. horizon flags distinguish before/after", b);
  }

  // 6) hint 커서: 순차 접근 결과가 이진 탐색 결과와 동일
  {
    const auto tr = makeTraj(60, 0.0229);
    int hint = 0; double worst = 0.0;
    for (double t = 0.0; t < 0.95; t += kRt)
      worst = std::max(worst, (traj::sampleAt(tr, t, hint).p - traj::sampleAt(tr, t).p).norm());
    char b[96]; std::snprintf(b, sizeof b, "max diff = %.1e m", worst);
    check(worst == 0.0, "6. hint cursor == fresh binary search", b);
  }

  // 7) 형식 검사기
  {
    auto tr = makeTraj(60, 0.0229);
    const auto ok = traj::check(tr, 10);
    tr.s[30].t = tr.s[29].t;                              // 단조성 위반 주입
    const auto bad = traj::check(tr, 10);
    char b[128]; std::snprintf(b, sizeof b, "ok=%d dt=[%.2f,%.2f]ms horizon=%.3fs ; 위반주입 ok=%d mono=%d",
                               int(ok.ok), 1e3 * ok.dt_min, 1e3 * ok.dt_max, ok.horizon,
                               int(bad.ok), int(bad.t_monotonic));
    check(ok.ok && !bad.ok && !bad.t_monotonic, "7. format check (monotonic t, finite, count)", b);
  }

  // 8) 회귀: 찢어진 읽기로 t 가 비단조가 되면 invalid 로 알린다 (조용히 A 를 돌려주지 않는다)
  {
    traj::Sample A{}, B{};
    A.t = 0.10; A.p = {1, 2, 3};
    B.t = 0.10;                                   // h == 0
    const auto e = traj::interpolate(A, B, 0.10);
    char b[96]; std::snprintf(b, sizeof b, "h=0 -> valid=%d", int(e.valid));
    check(!e.valid, "8. non-monotonic sample pair reported invalid", b);
  }

  std::printf(g_fail ? "\nL2: %d FAILED\n" : "\nL2: all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
