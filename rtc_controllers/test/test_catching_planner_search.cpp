// ── The planner's search on a real 6R arm (dynamic_catching S6-B) ────────────
//
//   1. q̇ᵘ (G3-I's runtime half): the fixed-capacity producer equals an
//      independent dynamic-size DLS built from the fixture's own Jacobian
//      stack — the formula the offline map's python uses.
//   2. Judgement vs rank (decisions C/D): a judgement gate removes a candidate
//      and names itself in the no-plan reason; a failed rank gate only sets
//      its bit and adds the penalty — the candidate can still be chosen.
//   3. Settle (§4.4), the IK budget (R-2 pre-filter / budget_s).
//   4. Switching (§4.7) and freeze (decision G): hold on no improvement, on
//      the freeze window, and replace when the current plan is infeasible.
//   5. G3-K: a full search (IK + q̇ᵘ + gates) allocates nothing.
//   6. R-2 / G3-G: timing and IK acceptance recorded as integer µs / counts
//      (RecordProperty's to_string squashes small doubles).
//
// Include order: the Eigen allocation tripwire must precede every Eigen header.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/catching/planner_search.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/catch_arm_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <vector>

namespace {

using rtc::catching::CatchPoseIkOptions;
using rtc::catching::CovarianceSnapshot;
using rtc::catching::JudgeReject;
using rtc::catching::NowReal;
using rtc::catching::PlannerConstants;
using rtc::catching::PlannerModel;
using rtc::catching::PlannerParams;
using rtc::catching::PlannerRtState;
using rtc::catching::PlannerSearch;
using rtc::catching::PlanReason;
using rtc::catching::PlanSnapshot;
using rtc::catching::SearchStats;
using rtc::catching::SwitchDecision;
using rtc::catching::TrajectorySnapshot;
using rtc::catching::UnitSpeedSolver;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kNow = 10'000 * kMs;

std::int64_t SteadyClock() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

/// The 6R wrist arm, its reference catch configuration, and a trajectory that
/// passes through that configuration's catch point at t = now + 0.5 s.
struct Rig {
  rtc::testing::Arm arm = rtc::testing::Arm6R();
  Eigen::VectorXd q_ref;
  rtc::testing::Target target;
  PlannerModel model{};
  PlannerConstants constants{};
  PlannerParams params{};
  CatchPoseIkOptions ik{};
  PlannerSearch search;

  Rig() {
    q_ref = Eigen::VectorXd::Zero(arm.nv);
    for (int i = 0; i < arm.nv; ++i) {
      q_ref(i) = 0.3 * ((i % 2 == 0) ? 1.0 : -1.0) + 0.1 * i;
    }
    target = rtc::testing::TargetAt(arm, q_ref, /*speed=*/3.0);

    model.handle = arm.handle.get();
    model.catch_frame = arm.frame;
    model.nv = arm.nv;
    for (int j = 0; j < arm.nv; ++j) {
      model.device_of_model[static_cast<std::size_t>(j)] = j;
      model.qdot_max[static_cast<std::size_t>(j)] = 3.14;
      model.qddot_max[static_cast<std::size_t>(j)] = 20.0;
    }
    model.accel_box = true;

    constants.eta_v = 0.9;
    constants.v_max = 3.0;
    constants.a_dec = 10.0;
    constants.t_arm_s = 0.0;
    constants.t_close_e2e = 0.1;
    constants.t_close_total = 0.101;
    constants.ball_mass = 0.057;
    constants.ref_omega = 10.0;
    constants.ref_zeta = 1.0;
    constants.ref_a_max = 21.0;
    constants.control_dt = 0.002;

    params.enabled = true;
    params.wait_pose_n = arm.nv;
    for (int j = 0; j < arm.nv; ++j) {
      // Seed near the reference so the IK has real but short work.
      params.wait_pose[static_cast<std::size_t>(j)] = q_ref(j) + 0.15;
    }
    params.t_freeze = 0.2;
    params.slice_t_max = 0.95;
    params.n_settle = 0;
    params.d_eff = 0.2;
    params.r_cap = 0.03;
    params.max_ik = 8;
    params.budget_s = 0.05;
    params.catch_box.set = true;
    params.catch_box.min = {-5.0, -5.0, -5.0};
    params.catch_box.max = {5.0, 5.0, 5.0};

    ik.max_iter = 60;
    ik.manipulability_min = 0.0;
  }

  bool Configure(PlannerSearch::ClockFn clock = &SteadyClock) {
    return search.Configure(model, constants, params, ik, clock);
  }

  /// 20 samples, 50 ms apart from now; sample 10 (t = 0.5 s) is the target.
  [[nodiscard]] TrajectorySnapshot Traj(std::uint64_t seq = 1, std::uint64_t gen = 7) const {
    TrajectorySnapshot t{};
    t.valid = true;
    t.n = 20;
    t.token.activation_generation = 3;
    t.token.generation = gen;
    t.token.snapshot_sequence = seq;
    t.token.traj_recv_ns = kNow - 5 * kMs;
    for (int k = 0; k < t.n; ++k) {
      auto& s = t.s[static_cast<std::size_t>(k)];
      s.t_ns = kNow + k * 50 * kMs;
      const double dt = (k - 10) * 0.05;
      const Eigen::Vector3d p = target.p_c + target.v_ball * dt;
      s.p = {p.x(), p.y(), p.z()};
      s.v = {target.v_ball.x(), target.v_ball.y(), target.v_ball.z()};
    }
    return t;
  }

  /// A matched covariance, isotropic σ per sample.
  [[nodiscard]] static CovarianceSnapshot Cov(const TrajectorySnapshot& t, double sigma) {
    CovarianceSnapshot c{};
    c.valid = true;
    c.n = t.n;
    c.token = t.token;
    for (int k = 0; k < t.n; ++k) {
      auto& e = c.c[static_cast<std::size_t>(k)];
      e.fill(0.0);
      for (int d = 0; d < 6; ++d) {
        e[static_cast<std::size_t>(d * 6 + d)] = sigma * sigma;
      }
    }
    return c;
  }

  [[nodiscard]] PlannerRtState Rt() const {
    PlannerRtState rt{};
    rt.valid = true;
    rt.activation_generation = 3;
    rt.mode = static_cast<std::uint8_t>(rtc::catching::Mode::kTracking);
    rt.nv = arm.nv;
    rt.cmd_seeded = true;
    for (int j = 0; j < arm.nv; ++j) {
      rt.q_cmd[static_cast<std::size_t>(j)] = params.wait_pose[static_cast<std::size_t>(j)];
    }
    return rt;
  }
};

// ── 1. q̇ᵘ ───────────────────────────────────────────────────────────────────

TEST(PlannerUnitSpeed, MatchesAnIndependentDynamicSizeDls) {
  auto rig = std::make_unique<Rig>();
  UnitSpeedSolver us;
  us.Resize(rig->arm.nv);
  const Eigen::Vector3d v_hat = rig->target.v_ball.normalized();
  std::vector<double> qdu(static_cast<std::size_t>(rig->arm.nv), 0.0);
  const auto r =
      us.Compute(*rig->arm.handle, rig->arm.frame, rtc::testing::AsSpan(rig->q_ref), v_hat, qdu);
  ASSERT_TRUE(r.valid);

  // The map's python: j5 = [jp; jw], q̇ᵘ = j5ᵀ (j5 j5ᵀ + λ² I)⁻¹ [v̂; 0; 0].
  const Eigen::MatrixXd j6 = rtc::testing::StackJacobianRef(rig->arm, rig->q_ref);
  Eigen::MatrixXd j5(5, rig->arm.nv);
  j5.topRows(3) = j6.topRows(3);
  j5.bottomRows(2) = j6.middleRows(3, 2);
  Eigen::VectorXd rhs(5);
  rhs << v_hat, 0.0, 0.0;
  const double lam = rtc::catching::kUnitSpeedDamping;
  const Eigen::MatrixXd a = j5 * j5.transpose() + lam * lam * Eigen::MatrixXd::Identity(5, 5);
  const Eigen::VectorXd ref = j5.transpose() * a.partialPivLu().solve(rhs);
  for (int j = 0; j < rig->arm.nv; ++j) {
    EXPECT_NEAR(qdu[static_cast<std::size_t>(j)], ref(j), 1e-10) << "joint " << j;
  }
  EXPECT_LT((r.jp_qdot_u - j5.topRows(3) * ref).norm(), 1e-10);
  // And it does what it is for: unit speed along v̂ (DLS, so nearly).
  EXPECT_NEAR(v_hat.dot(r.jp_qdot_u), 1.0, 1e-3);
}

// ── 2. Judgement vs rank ─────────────────────────────────────────────────────

TEST(PlannerSearchPlan, FindsTheReachableCatchPointOnTheTrajectory) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  const PlanSnapshot plan =
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats);
  ASSERT_TRUE(plan.valid) << "reason " << static_cast<int>(plan.reason);
  EXPECT_TRUE(stats.publish);
  EXPECT_EQ(stats.decision, SwitchDecision::kNoCurrent);
  EXPECT_GT(stats.n_ik, 0);
  EXPECT_GT(stats.n_pass, 0);
  // t_c is a vision sample inside [T_freeze, t_max].
  const double lead = static_cast<double>(plan.t_c_ns - kNow) * 1e-9;
  EXPECT_GE(lead, rig->params.t_freeze);
  EXPECT_LE(lead, rig->params.slice_t_max);
  EXPECT_EQ(plan.nv, rig->arm.nv);
  for (int j = 0; j < plan.nv; ++j) {
    EXPECT_TRUE(std::isfinite(plan.q_star[static_cast<std::size_t>(j)]));
  }
  // The approach axis opposes the ball.
  const Eigen::Vector3d a_d(plan.a_d[0], plan.a_d[1], plan.a_d[2]);
  EXPECT_NEAR(a_d.dot(rig->target.v_ball.normalized()), -1.0, 1e-12);
  EXPECT_GE(plan.gamma_gf, 0.0);
  EXPECT_LE(plan.gamma_gf, 1.0);
  EXPECT_EQ(plan.gamma_t1_ns, plan.t_c_ns);
  // S6-C: the rollout ran and chose the γ window the plan carries.
  EXPECT_GT(stats.n_rollouts, 0);
  EXPECT_GT(stats.chosen_t_w, 0.0);
  EXPECT_EQ(plan.gamma_t0_ns,
            std::max<std::int64_t>(kNow, plan.t_c_ns - static_cast<std::int64_t>(
                                                           std::llround(stats.chosen_t_w * 1e9))));
}

TEST(PlannerSearchPlan, AJudgementGateRemovesEveryCandidateAndNamesItself) {
  auto rig = std::make_unique<Rig>();
  // A box nowhere near the trajectory: every p_c fails the workspace gate.
  rig->params.catch_box.min = {100.0, 100.0, 100.0};
  rig->params.catch_box.max = {101.0, 101.0, 101.0};
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  const PlanSnapshot plan =
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats);
  EXPECT_FALSE(plan.valid);
  EXPECT_EQ(plan.reason, PlanReason::kStoppingDistance);
  EXPECT_EQ(stats.n_pass, 0);
  EXPECT_EQ(stats.n_ik, 0) << "IK ran on candidates the cheap gate had already removed";
  EXPECT_EQ(stats.judge_rejects[static_cast<std::size_t>(JudgeReject::kWorkspace)],
            stats.n_in_window);
}

TEST(PlannerSearchPlan, AFailedRankGatePenalisesButDoesNotRemove) {
  // An unknown covariance fails the uncertainty rank gate for EVERY candidate:
  // decision D says the plan still exists, carries the bit, and costs the
  // penalty; decision C says only judgement gates remove.
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats known;
  const PlanSnapshot with_cov =
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, known);
  ASSERT_TRUE(with_cov.valid);
  rig->search.ResetTrial();
  SearchStats unknown;
  const PlanSnapshot without = rig->search.Plan(traj, CovarianceSnapshot{}, /*cov_matched=*/false,
                                                rig->Rt(), NowReal{kNow}, unknown);
  ASSERT_TRUE(without.valid) << "a rank gate removed the candidates";
  EXPECT_NE(unknown.chosen_rank_mask & rtc::catching::kRankUncertainty, 0);
  EXPECT_EQ(known.chosen_rank_mask & rtc::catching::kRankUncertainty, 0);
  EXPECT_GE(unknown.chosen_score, known.chosen_score + rig->params.score.penalty -
                                      rig->params.score.w_sigma * 0.002 / rig->params.r_cap - 1e-9);
}

TEST(PlannerSearchPlan, AnUnsetDecisionKeepsEveryCandidateOut) {
  // T_freeze unset (NaN) and no explicit t_lead_min: no lead is admissible,
  // so the planner plans nothing rather than guessing a freeze window.
  auto rig = std::make_unique<Rig>();
  rig->params.t_freeze = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  const PlanSnapshot plan =
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats);
  EXPECT_FALSE(plan.valid);
  EXPECT_EQ(plan.reason, PlanReason::kHorizonShort);
  EXPECT_EQ(stats.n_in_window, 0);
}

// ── 3. Settle and budget ─────────────────────────────────────────────────────

TEST(PlannerSearchPlan, SettlesForNSnapshotsAfterATrackChange) {
  auto rig = std::make_unique<Rig>();
  rig->params.n_settle = 2;
  ASSERT_TRUE(rig->Configure());
  SearchStats stats;
  for (std::uint64_t seq = 1; seq <= 2; ++seq) {
    const auto traj = rig->Traj(seq);
    const PlanSnapshot p =
        rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats);
    EXPECT_FALSE(p.valid) << "seq " << seq;
    EXPECT_TRUE(stats.settling);
  }
  const auto third = rig->Traj(3);
  EXPECT_TRUE(
      rig->search.Plan(third, Rig::Cov(third, 0.002), true, rig->Rt(), NowReal{kNow}, stats).valid);
  // The same snapshot again does not count: settling is about NEW information.
  const auto changed = rig->Traj(4, /*gen=*/8);
  EXPECT_FALSE(
      rig->search.Plan(changed, Rig::Cov(changed, 0.002), true, rig->Rt(), NowReal{kNow}, stats)
          .valid);
  EXPECT_TRUE(stats.settling);
}

std::int64_t g_fake_ns = 0;

std::int64_t FakeClock() noexcept {
  g_fake_ns += 3 * kMs;  // every read costs 3 ms
  return g_fake_ns;
}

TEST(PlannerSearchPlan, TheBudgetStopsTheIkAndSaysSo) {
  auto rig = std::make_unique<Rig>();
  rig->params.budget_s = 0.010;
  ASSERT_TRUE(rig->Configure(&FakeClock));
  const auto traj = rig->Traj();
  SearchStats stats;
  static_cast<void>(
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats));
  EXPECT_TRUE(stats.budget_hit);
  EXPECT_LT(stats.n_ik, rig->params.max_ik);
  EXPECT_GT(stats.judge_rejects[static_cast<std::size_t>(JudgeReject::kNotEvaluated)], 0);
}

// ── 4. Switching and freeze ──────────────────────────────────────────────────

TEST(PlannerSearchSwitch, HoldsTheCurrentPlanWhenNothingIsBetter) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  PlanSnapshot first =
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats);
  ASSERT_TRUE(first.valid);
  first.plan_id = 11;
  rig->search.NotePublished(first);
  auto rt = rig->Rt();
  rt.plan_active = true;
  rt.plan_id = 11;
  static_cast<void>(rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rt, NowReal{kNow}, stats));
  EXPECT_EQ(stats.decision, SwitchDecision::kHeldHysteresis);
  EXPECT_FALSE(stats.publish);
}

TEST(PlannerSearchSwitch, NothingReplacesAPlanInsideTheFreezeWindow) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  PlanSnapshot first =
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats);
  ASSERT_TRUE(first.valid);
  first.plan_id = 12;
  rig->search.NotePublished(first);
  auto rt = rig->Rt();
  rt.plan_active = true;
  rt.plan_id = 12;
  // 'now' moved to within T_freeze of the committed catch instant.
  const NowReal late{first.t_c_ns - static_cast<std::int64_t>(0.5 * rig->params.t_freeze * 1e9)};
  static_cast<void>(rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rt, late, stats));
  EXPECT_EQ(stats.decision, SwitchDecision::kHeldFreeze);
  EXPECT_FALSE(stats.publish);
}

TEST(PlannerSearchSwitch, AnInfeasibleCurrentPlanIsReplacedWhenTheJumpIsSmall) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  PlanSnapshot first =
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats);
  ASSERT_TRUE(first.valid);
  // Pretend the current plan was for an instant no candidate matches now:
  // further than half a slice (25 ms) from every sample. Beyond the whole
  // prediction, not +30 ms: on a 50 ms grid +30 ms is 20 ms from the next
  // sample, which the planner now evaluates first (the followed candidate
  // leads the IK order, 2026-09-23 /code-review) and finds feasible.
  first.plan_id = 13;
  first.t_c_ns = kNow + 5000 * kMs;
  rig->search.NotePublished(first);
  auto rt = rig->Rt();
  rt.plan_active = true;
  rt.plan_id = 13;
  rt.ref_valid = true;
  rt.gamma = 1.0;  // (1-γ)‖Δp‖ = 0 and γ̇ = 0: the jump limits cannot refuse
  rt.gamma_d = 0.0;
  const PlanSnapshot next =
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rt, NowReal{kNow}, stats);
  EXPECT_EQ(stats.decision, SwitchDecision::kReplaced);
  EXPECT_TRUE(stats.publish);
  EXPECT_TRUE(next.valid);
  EXPECT_DOUBLE_EQ(next.gamma_g0, 1.0) << "γ must continue from the reference, not restart";

  // Same, but the reference is mid-ramp and moving: the jump limit refuses.
  rt.gamma = 0.0;
  rt.gamma_d = 5.0;
  first.p_c[0] += 1.0;  // a 1 m catch-point jump
  rig->search.NotePublished(first);
  static_cast<void>(rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rt, NowReal{kNow}, stats));
  EXPECT_EQ(stats.decision, SwitchDecision::kHeldJump);
  EXPECT_FALSE(stats.publish);
}

// ── 4a. The switch's acceleration budget (§4.7, decision ⑥) ─────────────────

/// Δu_des the RT's adoption actually produces: the L4 law evaluated before and
/// after re-targeting at the same state and instant, the new ramp restarted
/// from the current γ (what controller.cpp does on adoption).
Eigen::Vector3d AdoptionStep(const Eigen::Vector3d& p_c, const Eigen::Vector3d& dp,
                             const rtc::catching::TargetState& o, double t,
                             rtc::catching::GammaProfile ramp) {
  rtc::catching::SoftCatchTranslation law({10.0, 1.0, 1e9, 1e9});
  EXPECT_TRUE(law.Reset(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0.4, 0.1, -0.2)));
  EXPECT_TRUE(law.SetIntercept(p_c, ramp));
  const auto before = law.Evaluate(o, t);
  ramp.g0 = before.gamma;
  ramp.t0 = t;
  EXPECT_TRUE(law.SetIntercept(p_c + dp, ramp));
  const auto after = law.Evaluate(o, t);
  EXPECT_EQ(after.gamma, before.gamma) << "γ must be continuous across the adoption";
  return after.u_des - before.u_des;
}

TEST(PlannerSwitchBudget, TheBoundIsTheLawsOwnStepWhenTheTermsAlign) {
  // Every term along +x: Δp_c along +x, o − p_c and v_o along −x, γ̇, γ̈ > 0
  // (a third of the way up the ramp). The triangle bound is then exact, so
  // dropping or mis-weighting any term moves it off the law's own step.
  const rtc::catching::GammaProfile ramp{0.0, 0.6, 0.0, 0.6};
  const double t = 0.2;
  double g = 0.0;
  double gd = 0.0;
  double gdd = 0.0;
  ramp.Eval(t, g, gd, gdd);
  ASSERT_GT(gd, 0.0);
  ASSERT_GT(gdd, 0.0);
  const Eigen::Vector3d p_c(0.5, 0.0, 0.4);
  const Eigen::Vector3d dp(0.03, 0.0, 0.0);
  rtc::catching::TargetState o;
  o.p = p_c + Eigen::Vector3d(-0.8, 0.0, 0.0);
  o.v = Eigen::Vector3d(-3.0, 0.0, 0.0);
  o.a = Eigen::Vector3d(0.0, 0.0, -9.81);
  const Eigen::Vector3d du = AdoptionStep(p_c, dp, o, t, ramp);
  const double bound = rtc::catching::SwitchAccelStepBound(10.0, 1.0, g, gd, gdd, dp.norm(),
                                                           (o.p - p_c).norm(), o.v.norm());
  EXPECT_NEAR(du.norm(), bound, 1e-9);
  EXPECT_NEAR(du.y(), 0.0, 1e-12);
  EXPECT_NEAR(du.z(), 0.0, 1e-12);
  // The ramp terms dominate here: a switch that moves nothing still steps u.
  const Eigen::Vector3d du0 = AdoptionStep(p_c, Eigen::Vector3d::Zero(), o, t, ramp);
  EXPECT_GT(du0.norm(), 0.25 * 21.0);
}

TEST(PlannerSwitchBudget, TheBoundCoversTheLawsStepInAnyDirection) {
  std::mt19937 rng(537);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  std::uniform_real_distribution<double> when(0.0, 0.7);
  const rtc::catching::GammaProfile ramp{0.05, 0.5, 0.1, 0.55};
  for (int trial = 0; trial < 200; ++trial) {
    const double t = when(rng);
    double g = 0.0;
    double gd = 0.0;
    double gdd = 0.0;
    ramp.Eval(t, g, gd, gdd);
    const Eigen::Vector3d p_c(u(rng), u(rng), 0.5 + u(rng));
    const Eigen::Vector3d dp = 0.1 * Eigen::Vector3d(u(rng), u(rng), u(rng));
    rtc::catching::TargetState o;
    o.p = p_c + Eigen::Vector3d(u(rng), u(rng), u(rng));
    o.v = 4.0 * Eigen::Vector3d(u(rng), u(rng), u(rng));
    o.a = Eigen::Vector3d(0.0, 0.0, -9.81);
    const double step = AdoptionStep(p_c, dp, o, t, ramp).norm();
    const double bound = rtc::catching::SwitchAccelStepBound(10.0, 1.0, g, gd, gdd, dp.norm(),
                                                             (o.p - p_c).norm(), o.v.norm());
    EXPECT_LE(step, bound + 1e-9) << "trial " << trial << " t " << t;
  }
}

TEST(PlannerSwitchBudget, AnUnknownTargetFailsTheBudgetClosed) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double b = rtc::catching::SwitchAccelStepBound(10.0, 1.0, 0.1, 0.5, 0.0, 0.001, nan, 3.0);
  EXPECT_FALSE(b <= 0.25 * 21.0);
}

/// A followed plan (γ, γ̇ given) and the same ball predicted `dy` further
/// along y; returns the second cycle's decision. `ramp_t0_ns` ≠ 0 also reports
/// the ramp the RT runs: 0 → 0.7 over 0.45 s from that instant.
SwitchDecision RefreshDecision(double dy, double gamma, double gamma_d,
                               std::int64_t ramp_t0_ns = 0) {
  auto rig = std::make_unique<Rig>();
  EXPECT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  PlanSnapshot first =
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats);
  EXPECT_TRUE(first.valid);
  first.plan_id = 41;
  rig->search.NotePublished(first);
  auto rt = rig->Rt();
  rt.plan_active = true;
  rt.plan_id = 41;
  rt.ref_valid = true;
  rt.gamma = gamma;
  rt.gamma_d = gamma_d;
  if (ramp_t0_ns != 0) {
    rt.ramp_valid = true;
    rt.ramp_g0 = 0.0;
    rt.ramp_gf = 0.7;
    rt.ramp_t0_ns = ramp_t0_ns;
    rt.ramp_t1_ns = ramp_t0_ns + 450 * kMs;
  }
  auto moved = rig->Traj(2);
  for (int k = 0; k < moved.n; ++k) {
    moved.s[static_cast<std::size_t>(k)].p[1] += dy;
  }
  const PlanSnapshot next =
      rig->search.Plan(moved, Rig::Cov(moved, 0.002), true, rt, NowReal{kNow}, stats);
  if (stats.decision == SwitchDecision::kRefreshed) {
    EXPECT_EQ(next.t_c_ns, first.t_c_ns) << "the same candidate, not a different one";
    EXPECT_NEAR(next.p_c[1] - first.p_c[1], dy, 1e-12);
  }
  EXPECT_EQ(stats.publish, stats.decision == SwitchDecision::kRefreshed);
  return stats.decision;
}

TEST(PlannerSwitchBudget, AtRestTheRefreshLimitIsEtaJumpAMaxOverOmegaSquared) {
  // γ = 0, ramp at rest: ω²‖Δp‖ ≤ η_jump a_max → ‖Δp‖ ≤ 0.25·21/100 = 52.5 mm.
  // The old 10 mm distance limit refused all of these (refreshed 0 in sim).
  const double limit = 0.25 * 21.0 / 100.0;
  EXPECT_EQ(RefreshDecision(0.9 * limit, 0.0, 0.0), SwitchDecision::kRefreshed);
  EXPECT_EQ(RefreshDecision(1.1 * limit, 0.0, 0.0), SwitchDecision::kHeldJump);
}

TEST(PlannerSwitchBudget, AMovingRampRefusesEvenASmallRefresh) {
  // 5 mm at γ 0.2 is 0.4 m/s² of Δp term — far inside 5.25. But with γ̇ = 0.5
  // the restarted ramp adds 2ζωγ̇‖o − p_c‖ + 2γ̇‖v_o‖: the ball is at least
  // T_freeze·3 m/s = 0.6 m from p_c, so ≥ 6 + 3 m/s².
  EXPECT_EQ(RefreshDecision(0.005, 0.2, 0.0), SwitchDecision::kRefreshed);
  EXPECT_EQ(RefreshDecision(0.005, 0.2, 0.5), SwitchDecision::kHeldJump);
}

TEST(PlannerSwitchBudget, ARampStartingBeforeAdoptionIsJudgedAtAdoption) {
  // The snapshot's tick is just before the followed ramp starts (γ = γ̇ = γ̈
  // = 0), but the RT adopts this cycle's publish up to budget_s + 2 ticks
  // (54 ms) later, 44 ms into the ramp: γ̈ ≈ 15 s⁻² against a ball ≥ 0.47 m
  // from p_c is ≥ 7 m/s² on its own (2026-09-23 /code-review).
  EXPECT_EQ(RefreshDecision(0.005, 0.0, 0.0, kNow + 10 * kMs), SwitchDecision::kHeldJump);
  // The same ramp a second away never starts inside the window.
  EXPECT_EQ(RefreshDecision(0.005, 0.0, 0.0, kNow + 1000 * kMs), SwitchDecision::kRefreshed);
}

// ── 4b. /code-review 2026-09-23 regressions ─────────────────────────────────

std::int64_t g_slow_ns = 0;
int g_slow_calls = 0;

/// 1 µs per read, except that the FOURTH read of the first cycle lands 30 ms
/// later — the read that closes the first IK (t_start, the budget check, the
/// IK start, the IK end). One slow solve, longer than the whole budget.
std::int64_t OneSlowIkClock() noexcept {
  ++g_slow_calls;
  g_slow_ns += (g_slow_calls == 4) ? 30 * kMs : 1'000;
  return g_slow_ns;
}

TEST(PlannerSearchReview, OneSlowSolveDoesNotStopThePlannerForGood) {
  auto rig = std::make_unique<Rig>();
  rig->params.budget_s = 0.020;  // the shipped budget; the spike is 1.5× it
  g_slow_ns = 0;
  g_slow_calls = 0;
  ASSERT_TRUE(rig->Configure(&OneSlowIkClock));
  const auto traj = rig->Traj();
  SearchStats stats;
  static_cast<void>(
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats));
  ASSERT_GE(stats.ik_ns_max, 30 * kMs) << "premise: the first cycle saw one 30 ms solve";
  // Every later cycle still runs IK, and the search widens again as the
  // estimate relaxes — it used to stop at the first check forever.
  int widest = 0;
  for (int cycle = 0; cycle < 40; ++cycle) {
    static_cast<void>(
        rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats));
    EXPECT_GE(stats.n_ik, 1) << "cycle " << cycle;
    widest = std::max(widest, static_cast<int>(stats.n_ik));
  }
  EXPECT_GT(widest, 1) << "the estimate never relaxed";
}

TEST(PlannerSearchReview, AMovedPredictionOfTheFollowedCandidateIsRefreshed) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  PlanSnapshot first =
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats);
  ASSERT_TRUE(first.valid);
  first.plan_id = 21;
  rig->search.NotePublished(first);
  auto rt = rig->Rt();
  rt.plan_active = true;
  rt.plan_id = 21;
  rt.ref_valid = true;
  rt.gamma = 1.0;  // the jump limits cannot refuse (see the replacement case)
  rt.gamma_d = 0.0;
  // The same ball, predicted 5 mm further along y — more than eps_term (2 mm).
  auto moved = rig->Traj(2);
  for (int k = 0; k < moved.n; ++k) {
    moved.s[static_cast<std::size_t>(k)].p[1] += 0.005;
  }
  const PlanSnapshot next =
      rig->search.Plan(moved, Rig::Cov(moved, 0.002), true, rt, NowReal{kNow}, stats);
  EXPECT_EQ(stats.decision, SwitchDecision::kRefreshed);
  EXPECT_TRUE(stats.publish);
  ASSERT_TRUE(next.valid);
  EXPECT_EQ(next.t_c_ns, first.t_c_ns) << "the same candidate, not a different one";
  EXPECT_NEAR(next.p_c[1] - first.p_c[1], 0.005, 1e-12);
}

TEST(PlannerSearchReview, TheCurrentPlanIsWhatTheRtFollowsNotTheLastPublish) {
  // P1 is followed; P2 was published but the RT refused it (freeze, age). The
  // planner must keep treating P1 as current, not fall back to "no current"
  // and republish every cycle without hysteresis.
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  PlanSnapshot p1 =
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats);
  ASSERT_TRUE(p1.valid);
  p1.plan_id = 31;
  rig->search.NotePublished(p1);
  PlanSnapshot p2 = p1;
  p2.plan_id = 32;
  p2.t_c_ns += 50 * kMs;
  rig->search.NotePublished(p2);
  auto rt = rig->Rt();
  rt.plan_active = true;
  rt.plan_id = 31;
  static_cast<void>(rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rt, NowReal{kNow}, stats));
  EXPECT_NE(stats.decision, SwitchDecision::kNoCurrent);
  EXPECT_EQ(stats.decision, SwitchDecision::kHeldHysteresis);
  EXPECT_FALSE(stats.publish);
}

TEST(PlannerSearchReview, SettlingWhileFollowingHoldsInsteadOfPublishingNoPlan) {
  auto rig = std::make_unique<Rig>();
  rig->params.n_settle = 1;
  ASSERT_TRUE(rig->Configure());
  SearchStats stats;
  const auto settle = rig->Traj(1);
  static_cast<void>(
      rig->search.Plan(settle, Rig::Cov(settle, 0.002), true, rig->Rt(), NowReal{kNow}, stats));
  ASSERT_TRUE(stats.settling) << "premise: the first snapshot of a track settles";
  const auto traj = rig->Traj(2);
  PlanSnapshot first =
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats);
  ASSERT_TRUE(first.valid);
  first.plan_id = 41;
  rig->search.NotePublished(first);
  auto rt = rig->Rt();
  rt.plan_active = true;
  rt.plan_id = 41;
  // A new track generation restarts the settle count.
  const auto other = rig->Traj(1, 8);
  const PlanSnapshot next =
      rig->search.Plan(other, Rig::Cov(other, 0.002), true, rt, NowReal{kNow}, stats);
  ASSERT_TRUE(stats.settling);
  EXPECT_FALSE(next.valid);
  EXPECT_FALSE(stats.publish) << "a no-plan publish would overwrite the followed plan's box";
  EXPECT_EQ(stats.decision, SwitchDecision::kHeldNoCandidate);
}

// ── 5. G3-K ──────────────────────────────────────────────────────────────────

TEST(PlannerSearchPlan, AFullSearchAllocatesNothing) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  const auto cov = Rig::Cov(traj, 0.002);
  const auto rt = rig->Rt();
  SearchStats stats;
  ASSERT_TRUE(rig->search.Plan(traj, cov, true, rt, NowReal{kNow}, stats).valid);  // warm
  std::size_t heap = 0;
  std::uint64_t eigen = 0;
  bool valid = false;
  int n_ik = 0;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (int i = 0; i < 5; ++i) {
      rig->search.ResetTrial();
      valid = rig->search.Plan(traj, cov, true, rt, NowReal{kNow}, stats).valid;
      n_ik += stats.n_ik;
    }
    heap = heap_gate.count();
    eigen = eigen_gate.violations();
  }
  EXPECT_TRUE(valid);
  EXPECT_GT(n_ik, 0) << "the gated loop ran no IK — the expensive half was not measured";
  EXPECT_EQ(heap, 0U);
  EXPECT_EQ(eigen, 0U);
}

// ── 6. R-2 timing and G3-G acceptance (recorded) ─────────────────────────────

TEST(PlannerSearchTiming, RecordsIkAndCycleTimesOnThisHost) {
  // Not a pass/fail on time — dev-PC numbers are NOT the control PC's (S6-D).
  // Recorded so the report and #537 cite a measurement, not an estimate.
  auto rig = std::make_unique<Rig>();
  rig->params.budget_s = 0.05;  // let every IK run: this measures cost, not policy
  ASSERT_TRUE(rig->Configure());
  std::mt19937 rng(20260923);
  std::vector<std::int64_t> cycle_us;
  std::vector<std::int64_t> ik_us;
  int ik_total = 0;
  int passed_total = 0;
  constexpr int kCycles = 200;
  for (int c = 0; c < kCycles; ++c) {
    const Eigen::VectorXd q = rtc::testing::SampleQ(rig->arm, rng);
    rig->target = rtc::testing::TargetAt(rig->arm, q, 3.0);
    for (int j = 0; j < rig->arm.nv; ++j) {
      rig->params.wait_pose[static_cast<std::size_t>(j)] = q(j) + 0.15;
    }
    ASSERT_TRUE(rig->Configure());
    const auto traj = rig->Traj(static_cast<std::uint64_t>(c + 1));
    SearchStats stats;
    static_cast<void>(
        rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), NowReal{kNow}, stats));
    cycle_us.push_back(stats.search_ns / 1000);
    ik_us.push_back(stats.ik_ns_max / 1000);
    ik_total += stats.n_ik;
    passed_total += stats.n_pass;
  }
  std::sort(cycle_us.begin(), cycle_us.end());
  std::sort(ik_us.begin(), ik_us.end());
  const auto pct = [](const std::vector<std::int64_t>& v, double p) {
    return v[static_cast<std::size_t>(p * static_cast<double>(v.size() - 1))];
  };
  RecordProperty("cycle_us_p50", static_cast<int>(pct(cycle_us, 0.5)));
  RecordProperty("cycle_us_p99", static_cast<int>(pct(cycle_us, 0.99)));
  RecordProperty("cycle_us_max", static_cast<int>(cycle_us.back()));
  RecordProperty("ik_worst_per_cycle_us_p50", static_cast<int>(pct(ik_us, 0.5)));
  RecordProperty("ik_worst_per_cycle_us_p99", static_cast<int>(pct(ik_us, 0.99)));
  RecordProperty("ik_solves", ik_total);
  RecordProperty("ik_accepted_and_passed", passed_total);
  EXPECT_GT(ik_total, 0);
}

}  // namespace
