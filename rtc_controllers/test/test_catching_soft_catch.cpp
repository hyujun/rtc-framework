// L4 soft-catch translational reference — gates G4-A, G4-B, G4-C, G4-F, G4-G,
// G4-I (docs/dynamic_catching/L4_reference.md §9) plus the reference's A5 and
// reset() regressions.
//
// Ported from docs/dynamic_catching/test_l4.cpp with thresholds unchanged.
// Deliberately NOT ported:
//  • D3a/D3b/D3c (γ derate) — the feature is out of v1 (D-8, G4-E); the tests
//    go with it, they are not weakened (S1 sub-plan F-4);
//  • the three §4.5 axis-alignment cases (G4-D) — the functions move to
//    rtc_math se3 in S2.1 and are tested there (sub-plan F-2).
// One mechanical change: the reference read e/ė with step(…, dt = 0); dt ≤ 0 is
// invalid now (L4 §5.1), so those reads use Evaluate(), which is the same
// computation without the integration.
//
// Include order: the Eigen allocation tripwire must precede every Eigen header.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/catching/soft_catch.hpp"
#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/catching_ball_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>

namespace {

using rtc::catching::BallTime;
using rtc::catching::CriticallyDampedError;
using rtc::catching::GammaProfile;
using rtc::catching::MakeGammaProfile;
using rtc::catching::MakeNowLead;
using rtc::catching::NowReal;
using rtc::catching::ProfileSeconds;
using rtc::catching::SoftCatchTranslation;
using rtc::catching::TargetState;
using rtc::catching::TranslationOutput;
namespace fx = rtc::catching::fixture;

constexpr double kDt = 2e-3;
constexpr double kTc = 0.8;
constexpr double kK = 0.0229;

fx::BallState BallAt(double t) {
  fx::BallState x;
  x << -2.5, 0.2, 0.8, 4.0, -0.2, 3.5, kK;
  if (t > 0.0)
    (void)fx::Propagate(fx::BallModel{}, x, t, 1e-4, 1 << 20);
  return x;
}

TargetState ToTarget(const fx::BallState& x) {
  return {x.head<3>(), x.segment<3>(3), fx::F(fx::BallModel{}, x).segment<3>(3)};
}

SoftCatchTranslation::Params Prm(double omega, double zeta, double a_max, double v_max) {
  SoftCatchTranslation::Params p;
  p.omega = omega;
  p.zeta = zeta;
  p.a_max = a_max;
  p.v_max = v_max;
  return p;
}

struct RunOut {
  double gap;
  double relv;
};

// pred_offset: catch point the plan believed − true catch point (prediction error δ).
RunOut RunCatch(const Eigen::Vector3d& pred_offset, double gf, double Tw) {
  const fx::BallState xc = BallAt(kTc);
  SoftCatchTranslation ds(Prm(10.0, 1.0, 1e9, 1e9));
  EXPECT_TRUE(ds.Reset({0.3, 0.0, 0.6}, Eigen::Vector3d::Zero()));
  EXPECT_TRUE(ds.SetIntercept(xc.head<3>() + pred_offset, GammaProfile{0.0, gf, kTc - Tw, kTc}));
  const int N = static_cast<int>(kTc / kDt + 0.5);
  TranslationOutput o{};
  for (int k = 0; k < N; ++k) {
    const double t = k * kDt;
    o = ds.Step(ToTarget(BallAt(t)), t, kDt);
    EXPECT_TRUE(o.valid);
  }
  const fx::BallState xe = BallAt(kTc);
  return {(o.x - xe.head<3>()).norm(), (xe.segment<3>(3) - o.xd).norm()};
}

// G4-A: §4.9 table — terminal gap (1−γ_f)·δ and relative speed (1−γ_f)·‖v‖.
TEST(CatchingSoftCatch, G4ATerminalGapAndRelativeSpeedTable) {
  const double v_tc = BallAt(kTc).segment<3>(3).norm();
  constexpr double kEpsConv = 2e-3;  // [m] DS residual convergence at ω=10, t_c=0.8 s

  struct Row {
    double off;
    double gf;
  };

  for (const Row r : {Row{0.00, 0.0}, Row{0.00, 0.4}, Row{0.03, 0.0}, Row{0.03, 0.4}}) {
    const RunOut o = RunCatch({0.0, r.off, 0.0}, r.gf, 0.4);
    const double gap_pred = (1.0 - r.gf) * r.off;
    const double relv_pred = (1.0 - r.gf) * v_tc;
    EXPECT_LT(std::abs(o.gap - gap_pred), kEpsConv + 0.05 * gap_pred) << r.off << " " << r.gf;
    EXPECT_LT(std::abs(o.relv - relv_pred), 0.02 * relv_pred) << r.off << " " << r.gf;
  }
}

// G4-B: re-prediction jump  e' = e − (1−γ)Δp,  ė' = ė + γ̇Δp.
TEST(CatchingSoftCatch, G4BReplanJumpFormula) {
  SoftCatchTranslation ds(Prm(10.0, 1.0, 1e9, 1e9));
  ASSERT_TRUE(ds.Reset({0.3, 0.0, 0.6}, Eigen::Vector3d::Zero()));
  const Eigen::Vector3d pc(1.0, 0.2, 1.1);
  const Eigen::Vector3d dp(0.05, -0.02, 0.01);
  const GammaProfile gp{0.0, 0.4, 0.2, 0.8};
  ASSERT_TRUE(ds.SetIntercept(pc, gp));
  const double t = 0.5;
  const TargetState tgt = ToTarget(BallAt(t));
  for (int k = 0; k < 250; ++k)
    ASSERT_TRUE(ds.Step(ToTarget(BallAt(k * kDt)), k * kDt, kDt).valid);
  const TranslationOutput a = ds.Evaluate(tgt, t);
  ASSERT_TRUE(ds.SetIntercept(pc + dp, gp));
  const TranslationOutput b = ds.Evaluate(tgt, t);
  ASSERT_TRUE(a.valid && b.valid);
  double g{};
  double gd{};
  double gdd{};
  gp.Eval(t, g, gd, gdd);
  EXPECT_LT((b.e - (a.e - (1.0 - g) * dp)).norm(), 1e-9);
  EXPECT_LT((b.ed - (a.ed + gd * dp)).norm(), 1e-9);
}

// G4-C: discrete stability boundary s = ω·h < 2√2 − 2 = 0.828427. The doc's
// 0.80 / 0.85 pair has ±2.5 % resolution (damping 2.0ζω → 1.96ζω would still
// pass), so the reference pins it at ±0.2 %.
TEST(CatchingSoftCatch, G4CDiscreteStabilityBoundary) {
  const auto diverges = [](double s) {
    const double h = 2e-3;
    SoftCatchTranslation ds(Prm(s / h, 1.0, 1e12, 1e12));
    EXPECT_TRUE(ds.Reset({1.0, 0.0, 0.0}, Eigen::Vector3d::Zero()));
    EXPECT_TRUE(ds.SetIntercept(Eigen::Vector3d::Zero(), GammaProfile{0, 0, 0, 1}));
    const TargetState o{};
    double last = 1.0;
    for (int k = 0; k < 4000; ++k)
      last = ds.Step(o, k * h, h).x.norm();
    return !(last < 1.0);
  };
  EXPECT_FALSE(diverges(0.827));
  EXPECT_TRUE(diverges(0.830));
}

// G4-F (regression C3): under velocity saturation xdd is the realised
// acceleration, not the demand.
TEST(CatchingSoftCatch, G4FXddIsRealisedAccelUnderVelocitySaturation) {
  SoftCatchTranslation ds(Prm(10.0, 1.0, 1e9, 0.5));
  ASSERT_TRUE(ds.Reset({0, 0, 0}, {0.49, 0, 0}));
  ASSERT_TRUE(ds.SetIntercept({5, 0, 0}, GammaProfile{0, 0, 0, 1}));
  const TranslationOutput o = ds.Step({{5, 0, 0}, {0, 0, 0}, {0, 0, 0}}, 0.0, kDt);
  const double realized = (o.xd.x() - 0.49) / kDt;
  EXPECT_LT(std::abs(o.xdd.x() - realized), 1e-9);
  EXPECT_TRUE(o.saturated);
  EXPECT_TRUE(o.valid);
}

// A5: with γ = 0 the attractor is p_c, not the target (L4 §5.3) — a retreat
// must move p_c.
TEST(CatchingSoftCatch, RetreatAttractorIsInterceptNotTarget) {
  const auto converge = [](const Eigen::Vector3d& target, const Eigen::Vector3d& p_c) {
    SoftCatchTranslation ds(Prm(3, 1, 1e9, 1e9));
    EXPECT_TRUE(ds.Reset({0.5, 0.3, 0.9}, Eigen::Vector3d::Zero()));
    EXPECT_TRUE(ds.SetIntercept(p_c, GammaProfile{0, 0, 0, 1}));
    TranslationOutput o{};
    for (int k = 0; k < 3000; ++k)
      o = ds.Step({target, {0, 0, 0}, {0, 0, 0}}, k * kDt, kDt);
    return Eigen::Vector3d(o.x);
  };
  const Eigen::Vector3d home(0, 0, 1.2);
  const Eigen::Vector3d stale_pc(1.5, 0.2, 0.8);
  EXPECT_GT((converge(home, stale_pc) - home).norm(), 1.0);  // target-only: wrong
  EXPECT_LT((converge(home, home) - home).norm(), 1e-6);     // p_c = home: right
}

// Reset() clears the intercept and γ profile too (re-arm contamination).
TEST(CatchingSoftCatch, ResetClearsInterceptAndGammaProfile) {
  SoftCatchTranslation ds(Prm(3, 1, 1e9, 1e9));
  ASSERT_TRUE(ds.SetIntercept({9, 9, 9}, GammaProfile{0.0, 0.8, 0.0, 1.0}));
  const Eigen::Vector3d hold(0.1, -0.2, 1.0);
  ASSERT_TRUE(ds.Reset(hold, Eigen::Vector3d::Zero()));
  TranslationOutput o{};
  for (int k = 0; k < 2000; ++k)
    o = ds.Step({{5, 5, 5}, {1, 1, 1}, {0, 0, 0}}, k * kDt, kDt);
  EXPECT_LT((o.x - hold).norm(), 1e-9);
  EXPECT_EQ(o.gamma, 0.0);
}

// ── G4-I: NaN guard ─────────────────────────────────────────────────────────

// Twin runs: one sees a burst of bad inputs mid-flight, the other does not.
// Bad ticks must be invalid, leave the state bit-identical, report saturated;
// the next finite tick must continue exactly as the twin that never saw them.
TEST(CatchingSoftCatch, G4INonFiniteInputsPreserveStateAndRecover) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const fx::BallState xc = BallAt(kTc);
  const auto make = [&] {
    SoftCatchTranslation ds(Prm(10.0, 1.0, 15.0, 2.0));
    EXPECT_TRUE(ds.Reset({0.3, 0.0, 0.6}, Eigen::Vector3d::Zero()));
    EXPECT_TRUE(ds.SetIntercept(xc.head<3>(), GammaProfile{0.0, 0.4, kTc - 0.4, kTc}));
    return ds;
  };
  SoftCatchTranslation clean = make();
  SoftCatchTranslation hit = make();
  for (int k = 0; k < 150; ++k) {
    const double t = k * kDt;
    (void)clean.Step(ToTarget(BallAt(t)), t, kDt);
    (void)hit.Step(ToTarget(BallAt(t)), t, kDt);
  }
  const Eigen::Vector3d x_before = hit.Position();
  const Eigen::Vector3d xd_before = hit.Velocity();
  const double t_bad = 150 * kDt;
  const TargetState good = ToTarget(BallAt(t_bad));

  int case_id = 0;
  const auto expect_rejected = [&](const TranslationOutput& o) {
    EXPECT_FALSE(o.valid) << case_id;
    EXPECT_TRUE(o.saturated) << case_id;  // saturation detection true on non-finite
    EXPECT_EQ(hit.Position(), x_before) << case_id;
    EXPECT_EQ(hit.Velocity(), xd_before) << case_id;
    EXPECT_TRUE(o.x.allFinite() && o.xd.allFinite()) << case_id;
    ++case_id;
  };
  for (double bad : {nan, inf, -inf}) {
    for (int field = 0; field < 3; ++field) {
      TargetState o = good;
      Eigen::Vector3d& f = field == 0 ? o.p : (field == 1 ? o.v : o.a);
      f.y() = bad;
      expect_rejected(hit.Step(o, t_bad, kDt));
    }
    expect_rejected(hit.Step(good, bad, kDt));    // non-finite t
    expect_rejected(hit.Step(good, t_bad, bad));  // non-finite dt
  }
  expect_rejected(hit.Step(good, t_bad, 0.0));   // dt = 0
  expect_rejected(hit.Step(good, t_bad, -kDt));  // dt < 0
  // Finite inputs whose demand overflows: ‖u‖ is non-finite, so the old
  // `un > a_max` test would be false — the negated comparison catches it.
  // Evaluated inside the γ ramp: at t_bad γ is still 0, and with γ = 0 the
  // target has no influence at all, so a huge target there is (correctly) harmless.
  const double t_ramp = kTc - 0.2;
  TargetState huge = good;
  huge.p = Eigen::Vector3d::Constant(1e300);
  expect_rejected(hit.Step(huge, t_ramp, kDt));
  EXPECT_FALSE(hit.Evaluate(huge, t_ramp).valid);
  EXPECT_TRUE(hit.Evaluate(huge, t_bad).valid);

  // Recovery: continue both runs; the hit run must match the clean one exactly.
  for (int k = 150; k < 400; ++k) {
    const double t = k * kDt;
    const TranslationOutput a = clean.Step(ToTarget(BallAt(t)), t, kDt);
    const TranslationOutput b = hit.Step(ToTarget(BallAt(t)), t, kDt);
    ASSERT_TRUE(b.valid);
    ASSERT_EQ(a.x, b.x) << k;
    ASSERT_EQ(a.xd, b.xd) << k;
  }
}

TEST(CatchingSoftCatch, G4IRejectsNonFiniteResetAndIntercept) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  SoftCatchTranslation ds(Prm(10.0, 1.0, 15.0, 2.0));
  ASSERT_TRUE(ds.Reset({0.1, 0.2, 0.3}, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(ds.Reset({nan, 0, 0}, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(ds.Reset(Eigen::Vector3d::Zero(), {0, nan, 0}));
  EXPECT_EQ(ds.Position(), Eigen::Vector3d(0.1, 0.2, 0.3));
  EXPECT_FALSE(ds.SetIntercept({nan, 0, 0}, GammaProfile{}));
  EXPECT_FALSE(ds.SetIntercept({1, 1, 1}, GammaProfile{0, nan, 0, 1}));
  EXPECT_EQ(ds.Intercept(), Eigen::Vector3d(0.1, 0.2, 0.3));  // Reset set p_c = x
}

// A degenerate ramp is a step with zero derivatives, not a γ̈ spike.
TEST(CatchingSoftCatch, DegenerateGammaRampIsFiniteStep) {
  const GammaProfile gp{0.0, 0.4, 0.5, 0.5};
  double g{};
  double gd{};
  double gdd{};
  gp.Eval(0.49, g, gd, gdd);
  EXPECT_EQ(g, 0.0);
  gp.Eval(0.5, g, gd, gdd);
  EXPECT_EQ(g, 0.4);
  EXPECT_EQ(gd, 0.0);
  EXPECT_EQ(gdd, 0.0);
}

// Time axis (plan §3, L4 §5.2): γ is evaluated at now + T_arm. With the
// profile built from absolute instants and T_arm ≠ 0, the γ seen at real
// instant `now` is the profile value at now + T_arm, not at now.
TEST(CatchingSoftCatch, GammaEvaluatedOnLeadAxis) {
  constexpr std::int64_t kMs = 1'000'000;
  const BallTime origin{10'000 * kMs};
  const BallTime t0{10'200 * kMs};
  const BallTime t1{10'600 * kMs};
  const GammaProfile gp = MakeGammaProfile(0.0, 0.4, t0, t1, origin);
  EXPECT_DOUBLE_EQ(gp.t0, 0.2);
  EXPECT_DOUBLE_EQ(gp.t1, 0.6);

  const std::int64_t t_arm = 50 * kMs;
  const NowReal now{10'350 * kMs};
  double g_lead{};
  double g_real{};
  double gd{};
  double gdd{};
  gp.Eval(ProfileSeconds(MakeNowLead(now, t_arm), origin), g_lead, gd, gdd);
  gp.Eval(0.35, g_real, gd, gdd);
  double g_expected{};
  gp.Eval(0.40, g_expected, gd, gdd);
  EXPECT_DOUBLE_EQ(g_lead, g_expected);
  EXPECT_GT(std::abs(g_lead - g_real), 0.05);  // the two axes really differ here
}

// L4 §4.4: the closed form is the ζ = 1 error dynamics. Checked against a
// fine integration of e'' = −ω²e − 2ωė.
TEST(CatchingSoftCatch, CriticallyDampedClosedFormMatchesIntegration) {
  const Eigen::Vector3d e0(0.05, -0.02, 0.01);
  const Eigen::Vector3d ed0(-0.3, 0.1, 0.4);
  const double w = 10.0;
  const double T = 0.4;
  Eigen::Vector3d e = e0;
  Eigen::Vector3d ed = ed0;
  const int n = 400'000;
  const double h = T / n;
  for (int i = 0; i < n; ++i) {  // semi-implicit Euler, h = 1 µs
    ed += (-w * w * e - 2.0 * w * ed) * h;
    e += ed * h;
  }
  Eigen::Vector3d ec;
  Eigen::Vector3d edc;
  CriticallyDampedError(e0, ed0, w, T, ec, edc);
  EXPECT_LT((ec - e).norm(), 1e-6);
  EXPECT_LT((edc - ed).norm(), 1e-5);
}

// G4-G: Step() / Evaluate() allocate nothing, worst tick time recorded.
TEST(CatchingSoftCatch, G4GStepAllocationFreeAndRecorded) {
  static_assert(noexcept(
      std::declval<SoftCatchTranslation&>().Step(std::declval<const TargetState&>(), 0.0, 0.0)));
  static_assert(noexcept(std::declval<const SoftCatchTranslation&>().Evaluate(
      std::declval<const TargetState&>(), 0.0)));
  const fx::BallState xc = BallAt(kTc);
  std::array<TargetState, 400> targets{};
  for (std::size_t k = 0; k < targets.size(); ++k)
    targets[k] = ToTarget(BallAt(static_cast<double>(k) * kDt));
  SoftCatchTranslation ds(Prm(10.0, 1.0, 15.0, 2.0));
  ASSERT_TRUE(ds.Reset({0.3, 0.0, 0.6}, Eigen::Vector3d::Zero()));
  ASSERT_TRUE(ds.SetIntercept(xc.head<3>(), GammaProfile{0.0, 0.4, kTc - 0.4, kTc}));
  double sink = 0.0;
  std::int64_t worst_ns = 0;
  std::size_t heap = 0;
  std::uint64_t eigen = 0;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (std::size_t k = 0; k < targets.size(); ++k) {
      const double t = static_cast<double>(k) * kDt;
      const auto t0 = std::chrono::steady_clock::now();
      const TranslationOutput o = ds.Step(targets[k], t, kDt);
      const auto t1 = std::chrono::steady_clock::now();
      sink += o.x.x() + ds.Evaluate(targets[k], t).e.x();
      worst_ns = std::max<std::int64_t>(
          worst_ns, std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    }
    heap = heap_gate.count();
    eigen = eigen_gate.violations();
  }
  EXPECT_EQ(heap, 0u);
  EXPECT_EQ(eigen, 0u);
  EXPECT_TRUE(std::isfinite(sink));
  RecordProperty("worst_step_ns", std::to_string(worst_ns));
  std::printf("[ record ] worst SoftCatchTranslation::Step %lld ns\n",
              static_cast<long long>(worst_ns));
}

}  // namespace
