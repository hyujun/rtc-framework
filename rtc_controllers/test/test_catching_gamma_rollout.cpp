// ── γ rollout (L3 §4.8, dynamic_catching S6-C) ──────────────────────────────
//
//   1. G3-B: the L3 §4.8 table is reproduced (±5 %) through the RUNTIME path —
//      a vision-style snapshot sampled by SampleAt, not the reference's exact
//      ball — so the planner's rollout and the table measure the same thing.
//   2. The choice rule: the largest accepted γ_f wins; with no accepted grid
//      combination γ_min is tried once more; with nothing accepted the choice
//      reports failure and carries γ_min (a RANK failure, decision C).
//   3. Coarse-to-fine: the winner is re-run at the control period, and only
//      that run judges the terminal error (the coarse step's own lag is not a
//      verdict).
//   4. G3-K: a full choice allocates nothing.
//
// Include order: the Eigen allocation tripwire must precede every Eigen header.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/catching/gamma_rollout.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/catching_ball_fixture.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>

namespace {

namespace fx = rtc::catching::fixture;
using rtc::catching::BallTime;
using rtc::catching::ChooseGamma;
using rtc::catching::NowLead;
using rtc::catching::RolloutSettings;
using rtc::catching::RunGammaRollout;
using rtc::catching::SoftCatchTranslation;
using rtc::catching::TrajectorySnapshot;

constexpr double kTc = 0.8;
constexpr double kK = 0.0229;

// The reference scenario of docs/dynamic_catching/test_l3.cpp.
fx::BallState BallAt(double t) {
  fx::BallState x;
  x << -2.5, 0.2, 0.8, 4.0, -0.2, 3.5, kK;
  if (t > 0.0) {
    static_cast<void>(fx::Propagate(fx::BallModel{}, x, t, 1e-4, 1 << 20));
  }
  return x;
}

/// kCap samples covering [0, 0.82 s] — what vision would hand the planner.
std::unique_ptr<TrajectorySnapshot> Snapshot() {
  auto t = std::make_unique<TrajectorySnapshot>();
  t->valid = true;
  t->n = static_cast<std::int32_t>(rtc::catching::kCap);
  const double span = 0.82;
  for (int k = 0; k < t->n; ++k) {
    const double tk = span * k / (t->n - 1);
    const fx::BallState x = BallAt(tk);
    const fx::BallState dx = fx::F(fx::BallModel{}, x);
    auto& s = t->s[static_cast<std::size_t>(k)];
    s.t_ns = static_cast<std::int64_t>(std::llround(tk * 1e9));
    s.p = {x(0), x(1), x(2)};
    s.v = {x(3), x(4), x(5)};
    s.a = {dx(3), dx(4), dx(5)};
  }
  return t;
}

SoftCatchTranslation Unsaturated() {
  return SoftCatchTranslation(SoftCatchTranslation::Params{10.0, 1.0, 1e9, 1e9});
}

const Eigen::Vector3d kX0{0.3, 0.0, 0.6};
constexpr std::int64_t kTcNs = 800'000'000;

TEST(GammaRollout, ReproducesTheL3Table) {
  // L3 §4.8: (T_w 0.30, γ_f 0.4) → 33.0 m/s² inside the window, ±5 %.
  const auto traj = Snapshot();
  auto ds = Unsaturated();
  const Eigen::Vector3d p_c = BallAt(kTc).head<3>();
  const auto p = RunGammaRollout(ds, *traj, kX0, Eigen::Vector3d::Zero(), p_c, NowLead{0},
                                 BallTime{kTcNs}, 0.0, 0.4, 0.30, 2e-3);
  ASSERT_TRUE(p.valid);
  EXPECT_NEAR(p.u_max_window, 33.0, 0.05 * 33.0);
  // And the table's monotone shape in γ_f at a fixed window (γ_f dominates).
  double prev = 0.0;
  for (const double gf : {0.1, 0.2, 0.3, 0.4, 0.5}) {
    const auto q = RunGammaRollout(ds, *traj, kX0, Eigen::Vector3d::Zero(), p_c, NowLead{0},
                                   BallTime{kTcNs}, 0.0, gf, 0.30, 2e-3);
    ASSERT_TRUE(q.valid);
    EXPECT_GT(q.u_max_window, prev) << "γ_f " << gf;
    prev = q.u_max_window;
  }
}

RolloutSettings Settings(const std::array<double, 7>& gammas, const std::array<double, 3>& windows,
                         double a_max) {
  RolloutSettings s;
  s.a_max = a_max;
  s.v_max = 50.0;  // not the binding limit in these cases
  s.eta_a = 0.8;
  s.eta_v = 0.9;
  s.eps_term = 1.0;  // the terminal error is not what these cases are about
  s.dt_coarse = 0.01;
  s.dt_fine = 2e-3;
  s.gamma_grid = gammas;
  s.window_grid = windows;
  return s;
}

constexpr std::array<double, 7> kGammas{0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
constexpr std::array<double, 3> kWindows{0.30, 0.45, 0.60};

TEST(GammaRollout, ChoosesTheLargestAcceptedGammaAndConfirmsItFine) {
  const auto traj = Snapshot();
  auto ds = Unsaturated();
  const Eigen::Vector3d p_c = BallAt(kTc).head<3>();
  // Starting AT the catch point, so the approach phase demands nothing and the
  // whole-interval peak is the γ window's. η_a a_max = 16 m/s²: from the table,
  // γ_f 0.4 fits at T_w 0.6 (15.3) and 0.5 does not (19.2) — the choice lands
  // at 0.4.
  const auto s = Settings(kGammas, kWindows, 20.0);
  const auto c = ChooseGamma(ds, *traj, p_c, Eigen::Vector3d::Zero(), p_c, NowLead{0},
                             BallTime{kTcNs}, 0.0, 0.0, 1.0, s);
  ASSERT_TRUE(c.accepted);
  EXPECT_FALSE(c.window_only);
  EXPECT_FALSE(c.tried_min);
  EXPECT_DOUBLE_EQ(c.gamma_f, 0.4);
  EXPECT_LE(c.peaks.u_max, s.eta_a * s.a_max);
  EXPECT_GT(c.peaks.steps, 300) << "the confirmation was not the fine run";
  EXPECT_EQ(c.rollouts, 7 * 3 + 1);
}

TEST(GammaRollout, AFarStartFailsTheWholeIntervalButStillChoosesGammaOnTheWindow) {
  // The reference's own start (0.3, 0, 0.6), ~0.5 m from p_c: the approach
  // demands ω²·0.5 ≈ 50 m/s² whatever γ_f is. The verdict is "failed" (a rank
  // penalty); γ_f is still chosen by the γ window's peaks, where it matters.
  const auto traj = Snapshot();
  auto ds = Unsaturated();
  const Eigen::Vector3d p_c = BallAt(kTc).head<3>();
  const auto s = Settings(kGammas, kWindows, 20.0);
  const auto c = ChooseGamma(ds, *traj, kX0, Eigen::Vector3d::Zero(), p_c, NowLead{0},
                             BallTime{kTcNs}, 0.0, 0.0, 1.0, s);
  EXPECT_FALSE(c.accepted);
  EXPECT_TRUE(c.window_only);
  EXPECT_GT(c.gamma_f, 0.0);
  EXPECT_LE(c.peaks.u_max_window, s.eta_a * s.a_max);
  EXPECT_GT(c.peaks.u_max, s.eta_a * s.a_max) << "the approach should be what fails";
}

TEST(GammaRollout, TheWindowBoundsTheGridAndAnEmptyAcceptanceIsAFailureAtGammaMin) {
  const auto traj = Snapshot();
  auto ds = Unsaturated();
  const Eigen::Vector3d p_c = BallAt(kTc).head<3>();
  // Grid restricted to [0.25, 0.35]: only 0.3 is eligible.
  const auto s = Settings(kGammas, kWindows, 20.0);
  const auto c = ChooseGamma(ds, *traj, p_c, Eigen::Vector3d::Zero(), p_c, NowLead{0},
                             BallTime{kTcNs}, 0.0, 0.25, 0.35, s);
  ASSERT_TRUE(c.accepted);
  EXPECT_DOUBLE_EQ(c.gamma_f, 0.3);
  // An a_max nothing fits under (η_a a_max = 0.8 m/s²): the grid fails, γ_min
  // is tried, it fails too — reported, carrying γ_min.
  const auto tight = Settings(kGammas, kWindows, 1.0);
  const auto f = ChooseGamma(ds, *traj, p_c, Eigen::Vector3d::Zero(), p_c, NowLead{0},
                             BallTime{kTcNs}, 0.0, 0.25, 0.35, tight);
  EXPECT_FALSE(f.accepted);
  EXPECT_FALSE(f.window_only);
  EXPECT_TRUE(f.tried_min);
  EXPECT_DOUBLE_EQ(f.gamma_f, 0.25);
}

TEST(GammaRollout, TheTerminalErrorIsJudgedAtTheControlPeriodNotTheScreen) {
  // A soft catch the RT would land within ε_term = 2 mm, whose coarse screen
  // lags the moving target by more than that (≈ γ‖v‖·dt_coarse). Judged at the
  // screen, the choice refused every soft catch in G3-C.
  const auto traj = Snapshot();
  auto ds = Unsaturated();
  const Eigen::Vector3d p_c = BallAt(kTc).head<3>();
  auto s = Settings(kGammas, kWindows, 20.0);
  s.eps_term = 0.002;
  const auto coarse = RunGammaRollout(ds, *traj, p_c, Eigen::Vector3d::Zero(), p_c, NowLead{0},
                                      BallTime{kTcNs}, 0.0, 0.4, 0.60, s.dt_coarse);
  const auto fine = RunGammaRollout(ds, *traj, p_c, Eigen::Vector3d::Zero(), p_c, NowLead{0},
                                    BallTime{kTcNs}, 0.0, 0.4, 0.60, s.dt_fine);
  ASSERT_TRUE(coarse.valid && fine.valid);
  ASSERT_GT(coarse.e_term, s.eps_term) << "premise: the screen's lag exceeds ε_term";
  ASSERT_LE(fine.e_term, s.eps_term) << "premise: the control-period run meets it";
  const auto c = ChooseGamma(ds, *traj, p_c, Eigen::Vector3d::Zero(), p_c, NowLead{0},
                             BallTime{kTcNs}, 0.0, 0.0, 1.0, s);
  ASSERT_TRUE(c.accepted);
  EXPECT_DOUBLE_EQ(c.gamma_f, 0.4);
  EXPECT_LE(c.peaks.e_term, s.eps_term);
}

TEST(GammaRollout, APredictionThatEndsBeforeTheCatchIsUnjudgeable) {
  auto traj = Snapshot();
  traj->n = 10;  // ends at ~0.19 s, t_c is 0.8 s
  auto ds = Unsaturated();
  const auto p = RunGammaRollout(ds, *traj, kX0, Eigen::Vector3d::Zero(), BallAt(kTc).head<3>(),
                                 NowLead{0}, BallTime{kTcNs}, 0.0, 0.3, 0.3, 1e-2);
  EXPECT_FALSE(p.valid);
}

TEST(GammaRollout, AFullChoiceAllocatesNothing) {
  const auto traj = Snapshot();
  auto ds = Unsaturated();
  const Eigen::Vector3d p_c = BallAt(kTc).head<3>();
  const auto s = Settings(kGammas, kWindows, 20.0);
  std::size_t heap = 0;
  std::uint64_t eigen = 0;
  bool accepted = false;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    accepted = ChooseGamma(ds, *traj, p_c, Eigen::Vector3d::Zero(), p_c, NowLead{0},
                           BallTime{kTcNs}, 0.0, 0.0, 1.0, s)
                   .accepted;
    heap = heap_gate.count();
    eigen = eigen_gate.violations();
  }
  EXPECT_TRUE(accepted);
  EXPECT_EQ(heap, 0U);
  EXPECT_EQ(eigen, 0U);
}

}  // namespace
