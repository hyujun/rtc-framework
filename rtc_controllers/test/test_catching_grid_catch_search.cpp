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
//   5a. The arm (E1-F17): while the RT follows a segment the reach starts on
//      that segment at now_lead, bit for bit; with none readable, on the
//      reported command.
//   6. R-2 / G3-G: timing and IK acceptance recorded as integer µs / counts
//      (RecordProperty's to_string squashes small doubles).
//
// Include order: the Eigen allocation tripwire must precede every Eigen header.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/catching/ball_node_samples.hpp"
#include "rtc_controllers/catching/grid_catch_search.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/time_feasibility.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/testing/catch_arm_fixture.hpp"
#include "rtc_controllers/testing/grid_catch_search_fixture.hpp"
#include "rtc_controllers/testing/planner_trace_digest.hpp"

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <span>
#include <vector>

namespace {

using rtc::catching::CatchPoseIkOptions;
using rtc::catching::CovarianceSnapshot;
using rtc::catching::GridCatchSearch;
using rtc::catching::GridCatchSearchConstants;
using rtc::catching::GridCatchSearchModel;
using rtc::catching::JudgeReject;
using rtc::catching::kMaxSegmentNv;
using rtc::catching::NodeTrajectoryFollower;
using rtc::catching::NowReal;
using rtc::catching::PlannerParams;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanReason;
using rtc::catching::PlanSnapshot;
using rtc::catching::ReportedSegments;
using rtc::catching::SearchStats;
using rtc::catching::SegmentSnapshot;
using rtc::catching::SwitchDecision;
using rtc::catching::TrajectorySnapshot;
using rtc::catching::UnitSpeedSolver;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kNow = 10'000 * kMs;

// What a search is handed when the RT reports no segment: the reach then
// starts on the reported command.
const rtc::catching::ReportedSegments kNoSegments{};

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
  GridCatchSearchModel model{};
  GridCatchSearchConstants constants{};
  PlannerParams params{};
  CatchPoseIkOptions ik{};
  GridCatchSearch search;

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
    constants.t_close_lead = 0.1;
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

    ik.max_iter = 60;
    ik.manipulability_min = 0.0;
  }

  bool Configure(GridCatchSearch::ClockFn clock = &SteadyClock) {
    return search.Configure(model, constants, params, ik, clock);
  }

  /// 20 samples, 50 ms apart from now; sample 10 (t = 0.5 s) is the target.
  [[nodiscard]] TrajectorySnapshot Traj(std::uint64_t seq = 1, std::uint64_t gen = 7) const {
    return rtc::testing::LineTrajectory(target.p_c, target.v_ball, kNow, 50 * kMs, 20, 10, seq, gen,
                                        3, kNow - 5 * kMs);
  }

  /// A matched covariance, isotropic σ per sample.
  [[nodiscard]] static CovarianceSnapshot Cov(const TrajectorySnapshot& t, double sigma) {
    return rtc::testing::IsotropicCovariance(t, sigma);
  }

  [[nodiscard]] PlannerRtState Rt() const {
    return rtc::testing::TrackingRtState(
        3, std::span<const double>(params.wait_pose.data(), static_cast<std::size_t>(arm.nv)));
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

// `planner.search.grid.gamma.unit_speed_damping` reaches the search's unit-speed
// solve. The chosen candidate's v_dir,max (L3 §4.5) is recomputed here, from
// its own posture and ball direction, with the damping the profile gave — and
// the two dampings give different numbers, so a search that kept the constant
// would match only one of them.
TEST(PlannerUnitSpeed, TheSearchUsesTheProfilesDamping) {
  std::vector<double> v_dir;
  for (const double damping : {rtc::catching::kUnitSpeedDamping, 0.3}) {
    auto rig = std::make_unique<Rig>();
    rig->params.unit_speed_damping = damping;
    ASSERT_TRUE(rig->Configure());
    const auto traj = rig->Traj();
    SearchStats stats;
    const PlanSnapshot plan = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(),
                                               kNoSegments, NowReal{kNow}, 0, stats);
    ASSERT_TRUE(plan.valid);
    const Eigen::Vector3d v(plan.v_c[0], plan.v_c[1], plan.v_c[2]);
    UnitSpeedSolver us;
    us.Resize(rig->arm.nv);
    std::vector<double> qdu(static_cast<std::size_t>(rig->arm.nv), 0.0);
    const auto r = us.Compute(
        *rig->arm.handle, rig->arm.frame,
        std::span<const double>(plan.q_star.data(), static_cast<std::size_t>(rig->arm.nv)),
        v.normalized(), qdu, damping);
    ASSERT_TRUE(r.valid);
    std::vector<double> qdot_plan(static_cast<std::size_t>(rig->arm.nv),
                                  rig->constants.eta_v * 3.14);
    const auto expected =
        rtc::catching::DirectionalSpeedMax(v.normalized(), r.jp_qdot_u, qdu, qdot_plan);
    EXPECT_NEAR(stats.chosen_v_dir_max, expected.v_dir_max, 1e-9) << damping;
    v_dir.push_back(stats.chosen_v_dir_max);
  }
  ASSERT_EQ(v_dir.size(), 2U);
  EXPECT_GT(std::fabs(v_dir[0] - v_dir[1]), 1e-6);
}

// The two keys the search divides or damps by cannot be unusable: a hand-built
// PlannerParams (no parser in front) is refused at configure.
TEST(GridCatchSearchPlan, ConfigureRefusesAnUnusableSwitchSamplesOrDamping) {
  {
    auto rig = std::make_unique<Rig>();
    rig->params.switch_samples = 1;  // the bound divides by samples - 1
    EXPECT_FALSE(rig->Configure());
    rig->params.switch_samples = 2;
    EXPECT_TRUE(rig->Configure());
  }
  for (const double bad : {0.0, -1e-3, std::numeric_limits<double>::quiet_NaN()}) {
    auto rig = std::make_unique<Rig>();
    rig->params.unit_speed_damping = bad;
    EXPECT_FALSE(rig->Configure()) << bad;
  }
}

// ── 2. Judgement vs rank ─────────────────────────────────────────────────────

TEST(GridCatchSearchPlan, FindsTheReachableCatchPointOnTheTrajectory) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  const PlanSnapshot plan = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(),
                                             kNoSegments, NowReal{kNow}, 0, stats);
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
  // S8-I: the chosen candidate's γ window is reported as judged (L3 §4.5),
  // so an analysis reads it instead of rebuilding it from FK.
  const double speed = Eigen::Vector3d(plan.v_c[0], plan.v_c[1], plan.v_c[2]).norm();
  ASSERT_TRUE(std::isfinite(stats.chosen_v_dir_max));
  EXPECT_GT(stats.chosen_v_dir_max, 0.0);
  const double v_tcp = rig->constants.eta_v * rig->constants.v_max;
  const double v_arm = std::min(stats.chosen_v_dir_max, v_tcp);
  EXPECT_NEAR(stats.chosen_g_max, std::clamp(v_arm / speed, 0.0, 1.0), 1e-12);
  EXPECT_NEAR(
      stats.chosen_g_min,
      std::clamp(1.0 - rig->params.d_eff / (speed * rig->constants.t_close_total), 0.0, 1.0),
      1e-12);
  EXPECT_NEAR(stats.chosen_max_catchable, v_arm + rig->params.d_eff / rig->constants.t_close_total,
              1e-12);
  EXPECT_NEAR(plan.gamma_min, stats.chosen_g_min, 1e-12);
}

TEST(GridCatchSearchPlan, AnAdoptedWaitPoseInTheRtStateBecomesTheIkSeed) {
  // S8-I (`planner.wait_pose_source: current`): the RT hands the adopted pose
  // over in PlannerRtState; the search seeds its IK from it. Handing over the
  // configured pose changes nothing; a different pose still yields a plan.
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  const PlanSnapshot base = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(),
                                             kNoSegments, NowReal{kNow}, 0, stats);
  ASSERT_TRUE(base.valid);

  PlannerRtState same = rig->Rt();
  same.wait_pose_adopted = true;
  for (int j = 0; j < rig->arm.nv; ++j) {
    same.wait_pose[static_cast<std::size_t>(j)] =
        rig->params.wait_pose[static_cast<std::size_t>(j)];
  }
  const PlanSnapshot again = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, same, kNoSegments,
                                              NowReal{kNow}, 0, stats);
  ASSERT_TRUE(again.valid);
  for (int j = 0; j < base.nv; ++j) {
    EXPECT_DOUBLE_EQ(again.q_star[static_cast<std::size_t>(j)],
                     base.q_star[static_cast<std::size_t>(j)]);
  }
  EXPECT_DOUBLE_EQ(again.score, base.score);

  PlannerRtState moved = same;
  for (int j = 0; j < rig->arm.nv; ++j) {
    moved.wait_pose[static_cast<std::size_t>(j)] += 0.3;
  }
  const PlanSnapshot shifted = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, moved,
                                                kNoSegments, NowReal{kNow}, 0, stats);
  EXPECT_TRUE(shifted.valid) << "a plan from a different seed, reason "
                             << static_cast<int>(shifted.reason);
  // The handover is not a no-op: the seed in force IS the handed pose, and the
  // score (w_q·|q* − seed|²) or the IK solution moved with it.
  for (int j = 0; j < rig->arm.nv; ++j) {
    EXPECT_DOUBLE_EQ(rig->search.IkSeedForTesting()[j],
                     moved.wait_pose[static_cast<std::size_t>(j)]);
  }
  bool differs = shifted.score != base.score;
  for (int j = 0; j < base.nv; ++j) {
    differs = differs || shifted.q_star[static_cast<std::size_t>(j)] !=
                             base.q_star[static_cast<std::size_t>(j)];
  }
  EXPECT_TRUE(differs) << "a seed 0.3 rad away changed neither q* nor the score";

  // A later cycle WITHOUT an adopted pose (a refused switch-in, source yaml)
  // is back on the configure-time seed — not on the pose adopted before.
  const PlanSnapshot back = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(),
                                             kNoSegments, NowReal{kNow}, 0, stats);
  ASSERT_TRUE(back.valid);
  for (int j = 0; j < rig->arm.nv; ++j) {
    EXPECT_DOUBLE_EQ(rig->search.IkSeedForTesting()[j],
                     rig->params.wait_pose[static_cast<std::size_t>(j)]);
  }
  for (int j = 0; j < base.nv; ++j) {
    EXPECT_DOUBLE_EQ(back.q_star[static_cast<std::size_t>(j)],
                     base.q_star[static_cast<std::size_t>(j)]);
  }
  EXPECT_DOUBLE_EQ(back.score, base.score);
}

TEST(GridCatchSearchPlan, TheAdoptedWaitPoseIsReadInDeviceOrder) {
  // The RT hands the pose over in DEVICE order; the seed is in model order.
  // A rig whose model order is the device order reversed tells the two apart.
  auto rig = std::make_unique<Rig>();
  for (int j = 0; j < rig->arm.nv; ++j) {
    rig->model.device_of_model[static_cast<std::size_t>(j)] = rig->arm.nv - 1 - j;
  }
  ASSERT_TRUE(rig->Configure());
  PlannerRtState rt = rig->Rt();
  rt.wait_pose_adopted = true;
  for (int d = 0; d < rig->arm.nv; ++d) {
    rt.wait_pose[static_cast<std::size_t>(d)] = 0.1 * (d + 1);  // distinct per device joint
  }
  const auto traj = rig->Traj();
  SearchStats stats;
  (void)rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rt, kNoSegments, NowReal{kNow}, 0,
                         stats);
  for (int j = 0; j < rig->arm.nv; ++j) {
    EXPECT_DOUBLE_EQ(rig->search.IkSeedForTesting()[j], 0.1 * (rig->arm.nv - j))
        << "model joint " << j << " must read device joint " << rig->arm.nv - 1 - j;
  }
}

TEST(GridCatchSearchPlan, AJudgementGateRemovesEveryCandidateAndNamesItself) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  // The same line 100 m off: no candidate's catch point is within the arm's
  // reach, so the IK gate removes every one it runs on. (Until that gate was removed — L3 §4.9 —
  // this test used the catch box, a gate that no longer exists.)
  auto traj = rig->Traj();
  for (int k = 0; k < traj.n; ++k) {
    traj.s[static_cast<std::size_t>(k)].p[0] += 100.0;
  }
  SearchStats stats;
  const PlanSnapshot plan = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(),
                                             kNoSegments, NowReal{kNow}, 0, stats);
  EXPECT_FALSE(plan.valid);
  EXPECT_EQ(plan.reason, PlanReason::kIkFailed);
  EXPECT_EQ(stats.n_pass, 0);
  ASSERT_GT(stats.n_ik, 0);
  EXPECT_EQ(stats.judge_rejects[static_cast<std::size_t>(JudgeReject::kIk)], stats.n_ik);
}

TEST(GridCatchSearchPlan, NothingButTheInputRemovesACandidateBeforeTheIk) {
  // L3 §4.9: no gate judges WHERE a candidate's catch point is. Every candidate
  // in the lead window with a finite, moving ball either reaches the IK or is
  // left outside its budget — the two counts add up to the window.
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  const PlanSnapshot plan = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(),
                                             kNoSegments, NowReal{kNow}, 0, stats);
  EXPECT_TRUE(plan.valid);
  ASSERT_GT(stats.n_in_window, 0);
  EXPECT_EQ(stats.judge_rejects[static_cast<std::size_t>(JudgeReject::kInput)], 0);
  EXPECT_EQ(stats.n_ik + stats.judge_rejects[static_cast<std::size_t>(JudgeReject::kNotEvaluated)],
            stats.n_in_window);
}

TEST(GridCatchSearchPlan, AFailedRankGatePenalisesButDoesNotRemove) {
  // An unknown covariance fails the uncertainty rank gate for EVERY candidate:
  // decision D says the plan still exists, carries the bit, and costs the
  // penalty; decision C says only judgement gates remove.
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats known;
  const PlanSnapshot with_cov = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(),
                                                 kNoSegments, NowReal{kNow}, 0, known);
  ASSERT_TRUE(with_cov.valid);
  rig->search.ResetTrial();
  SearchStats unknown;
  const PlanSnapshot without = rig->search.Plan(traj, CovarianceSnapshot{}, /*cov_matched=*/false,
                                                rig->Rt(), kNoSegments, NowReal{kNow}, 0, unknown);
  ASSERT_TRUE(without.valid) << "a rank gate removed the candidates";
  EXPECT_NE(unknown.chosen_rank_mask & rtc::catching::kRankUncertainty, 0);
  EXPECT_EQ(known.chosen_rank_mask & rtc::catching::kRankUncertainty, 0);
  EXPECT_GE(unknown.chosen_score, known.chosen_score + rig->params.score.penalty -
                                      rig->params.score.w_sigma * 0.002 / rig->params.r_cap - 1e-9);
}

TEST(GridCatchSearchPlan, AnUnsetDecisionKeepsEveryCandidateOut) {
  // T_freeze unset (NaN) and no explicit t_lead_min: no lead is admissible,
  // so the planner plans nothing rather than guessing a freeze window.
  auto rig = std::make_unique<Rig>();
  rig->params.t_freeze = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  const PlanSnapshot plan = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(),
                                             kNoSegments, NowReal{kNow}, 0, stats);
  EXPECT_FALSE(plan.valid);
  EXPECT_EQ(plan.reason, PlanReason::kHorizonShort);
  EXPECT_EQ(stats.n_in_window, 0);
}

// ── 2a. The hand-close lead (E1-F16) ─────────────────────────────────────────

// `robot.hand.T_close_lead` is what the hand sequencer subtracts from t_c; the
// MEASURED closure time (`t_close_total`) only feeds the γ window. The command
// instant and the commit gate follow the lead, not the closure time.
TEST(GridCatchSearchPlan, TheCloseLeadDrivesTheCommandInstantAndTheCommitGate) {
  const auto run = [](double lead, double total, SearchStats& stats) {
    auto rig = std::make_unique<Rig>();
    rig->constants.t_close_lead = lead;
    rig->constants.t_close_total = total;
    EXPECT_TRUE(rig->Configure());
    const auto traj = rig->Traj();
    return rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                            NowReal{kNow}, 0, stats);
  };
  // A short lead beside a long measured closure (e2e 0.10 s ≠ lead 0.03 s; the
  // total here is far above every lead of the slice): the gate reads
  // lead + h/2 + T_arm + margin, so no candidate fails it. A gate reading the
  // closure time would fail them all.
  SearchStats short_lead;
  const PlanSnapshot a = run(0.03, 5.0, short_lead);
  ASSERT_TRUE(a.valid) << "reason " << static_cast<int>(a.reason);
  EXPECT_EQ(a.t_cmd_ns, a.t_c_ns - rtc::catching::SecondsToNs(0.03));
  EXPECT_EQ(short_lead.chosen_rank_mask & rtc::catching::kRankCommitLead, 0);
  // A lead longer than any candidate's: every one fails the gate (a rank gate —
  // the plan stays), whatever the short measured closure time says.
  SearchStats long_lead;
  const PlanSnapshot b = run(10.0, 0.101, long_lead);
  ASSERT_TRUE(b.valid) << "a rank gate removed the candidates";
  EXPECT_EQ(b.t_cmd_ns, b.t_c_ns - rtc::catching::SecondsToNs(10.0));
  EXPECT_NE(long_lead.chosen_rank_mask & rtc::catching::kRankCommitLead, 0);
  // An unknown lead: the close instant is the catch instant, and the gate fails.
  SearchStats unknown;
  const PlanSnapshot c = run(std::numeric_limits<double>::quiet_NaN(), 0.101, unknown);
  ASSERT_TRUE(c.valid);
  EXPECT_EQ(c.t_cmd_ns, c.t_c_ns);
  EXPECT_NE(unknown.chosen_rank_mask & rtc::catching::kRankCommitLead, 0);
}

// ── 2b. The monitored covariance (E1-F16) ────────────────────────────────────

// The followed plan's σ is read by the rule every planner reads the prediction
// by: the nearest sample's covariance propagated to t_c, F Σ Fᵀ — not that
// sample's block as it stands.
TEST(GridCatchSearchMonitor, SigmaIsThePropagatedPositionBlockAtTheCatchInstant) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  auto cov = Rig::Cov(traj, 0.002);
  for (int k = 0; k < traj.n; ++k) {
    auto& e = cov.c[static_cast<std::size_t>(k)];
    e[14] = 3e-4;          // p_z p_z
    e[35] = 1e-2;          // v_z v_z
    e[17] = e[32] = 1e-3;  // p_z v_z (|rho| < 1: the whole matrix stays PSD)
    e[0] = 1e-4;           // p_x p_x
    e[7] = 2e-4;           // p_y p_y
  }
  // t_c between samples 10 and 11, not at the midpoint: the nearest is sample 10.
  const std::int64_t t_c = traj.s[10].t_ns + 13 * kMs;
  PlanSnapshot followed{};
  followed.valid = true;
  followed.plan_id = 31;
  followed.t_c_ns = t_c;
  rig->search.NotePublished(followed);
  PlannerRtState rt = rig->Rt();
  rt.plan_active = true;
  rt.plan_id = 31;
  SearchStats stats;
  rig->search.Monitor(traj, cov, true, rt, stats);
  ASSERT_TRUE(std::isfinite(stats.sigma_l));

  const auto sigma_of = [](const Eigen::Matrix3d& pp) {
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(pp);
    return std::sqrt(std::max(es.eigenvalues().maxCoeff(), 0.0));
  };
  // The same statistic from SampleBallNode's propagated block ...
  int hint = 0;
  const rtc::catching::BallNodeSample node =
      rtc::catching::SampleBallNode(traj, &cov, true, rtc::catching::BallTime{t_c}, hint);
  ASSERT_TRUE(node.cov_valid);
  EXPECT_NEAR(stats.sigma_l, sigma_of(node.cov.topLeftCorner<3, 3>()), 1e-12);
  // ... and from F Sigma_10 F^T written out.
  Eigen::Matrix<double, 6, 6> sigma;
  for (int r = 0; r < 6; ++r) {
    for (int q = 0; q < 6; ++q) {
      sigma(r, q) = cov.c[10][static_cast<std::size_t>(r * 6 + q)];
    }
  }
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
  F.topRightCorner<3, 3>() = 0.013 * Eigen::Matrix3d::Identity();
  const Eigen::Matrix<double, 6, 6> moved = F * sigma * F.transpose();
  EXPECT_NEAR(stats.sigma_l, sigma_of(moved.topLeftCorner<3, 3>()), 1e-12);
  // It is not the sample's own block, which the old reading gave.
  EXPECT_GT(std::abs(stats.sigma_l - GridCatchSearch::SigmaMax(cov, 10)), 1e-5);
}

// ── 3. Settle and budget ─────────────────────────────────────────────────────

TEST(GridCatchSearchPlan, SettlesForNSnapshotsAfterATrackChange) {
  auto rig = std::make_unique<Rig>();
  rig->params.n_settle = 2;
  ASSERT_TRUE(rig->Configure());
  SearchStats stats;
  for (std::uint64_t seq = 1; seq <= 2; ++seq) {
    const auto traj = rig->Traj(seq);
    const PlanSnapshot p = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(),
                                            kNoSegments, NowReal{kNow}, 0, stats);
    EXPECT_FALSE(p.valid) << "seq " << seq;
    EXPECT_TRUE(stats.settling);
  }
  const auto third = rig->Traj(3);
  EXPECT_TRUE(rig->search
                  .Plan(third, Rig::Cov(third, 0.002), true, rig->Rt(), kNoSegments, NowReal{kNow},
                        0, stats)
                  .valid);
  // The same snapshot again does not count: settling is about NEW information.
  const auto changed = rig->Traj(4, /*gen=*/8);
  EXPECT_FALSE(rig->search
                   .Plan(changed, Rig::Cov(changed, 0.002), true, rig->Rt(), kNoSegments,
                         NowReal{kNow}, 0, stats)
                   .valid);
  EXPECT_TRUE(stats.settling);
}

std::int64_t PinnedClock() noexcept {
  return kNow;
}

// One fixed sequence — a track settling, a plan published and followed, a
// monitor — with every plan and record it produces digested
// (planner_trace_digest.hpp).
std::uint64_t SearchSequenceDigest(Rig& rig) {
  rtc::testing::ValueDigest h;
  SearchStats stats;
  PlanSnapshot last{};
  for (std::uint64_t seq = 1; seq <= 4; ++seq) {
    const auto traj = rig.Traj(seq);
    last = rig.search.Plan(traj, Rig::Cov(traj, 0.002), true, rig.Rt(), kNoSegments, NowReal{kNow},
                           0, stats);
    rtc::testing::AddPlan(h, last);
    rtc::testing::AddSearchStats(h, stats);
  }
  last.plan_id = 31;
  rig.search.NotePublished(last);
  PlannerRtState rt = rig.Rt();
  rt.plan_active = true;
  rt.plan_id = 31;
  const auto traj = rig.Traj(5);
  const auto cov = Rig::Cov(traj, 0.002);
  rtc::testing::AddPlan(h,
                        rig.search.Plan(traj, cov, true, rt, kNoSegments, NowReal{kNow}, 0, stats));
  rtc::testing::AddSearchStats(h, stats);
  rig.search.Monitor(traj, cov, true, rt, stats);
  rtc::testing::AddSearchStats(h, stats);
  return h.Value();
}

TEST(GridCatchSearchPlan, AReconfiguredSearchIsANewOne) {
  // Configure is a full reset: a search that has run and is configured again
  // answers exactly as one built and configured now. PlannerCycle relies on
  // it — a configure installs a NEW search (MakeGridCatchSearch) every time (E1-F12
  // #738) where it once configured the one in place again, and the two are
  // the same thing only if nothing a search did before survives its
  // Configure. n_settle 2 puts the per-trial part of that in view: a search
  // that kept its settle count would plan on the first snapshot.
  const auto make = [] {
    auto rig = std::make_unique<Rig>();
    rig->params.n_settle = 2;
    EXPECT_TRUE(rig->Configure(&PinnedClock));
    return rig;
  };
  auto fresh = make();
  const std::uint64_t expected = SearchSequenceDigest(*fresh);

  // Non-vacuity: run again WITHOUT a Configure in between, the same sequence
  // answers differently — it does leave state behind for Configure to clear.
  auto dirty = make();
  static_cast<void>(SearchSequenceDigest(*dirty));
  EXPECT_NE(SearchSequenceDigest(*dirty), expected);

  auto used = make();
  static_cast<void>(SearchSequenceDigest(*used));
  ASSERT_TRUE(used->Configure(&PinnedClock));
  EXPECT_EQ(SearchSequenceDigest(*used), expected);
}

std::int64_t g_fake_ns = 0;

std::int64_t FakeClock() noexcept {
  g_fake_ns += 3 * kMs;  // every read costs 3 ms
  return g_fake_ns;
}

TEST(GridCatchSearchPlan, TheBudgetStopsTheIkAndSaysSo) {
  auto rig = std::make_unique<Rig>();
  rig->params.budget_s = 0.010;
  ASSERT_TRUE(rig->Configure(&FakeClock));
  const auto traj = rig->Traj();
  SearchStats stats;
  static_cast<void>(rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                                     NowReal{kNow}, 0, stats));
  EXPECT_TRUE(stats.budget_hit);
  EXPECT_LT(stats.n_ik, rig->params.max_ik);
  EXPECT_GT(stats.judge_rejects[static_cast<std::size_t>(JudgeReject::kNotEvaluated)], 0);
}

TEST(GridCatchSearchPlan, ACallersCapBelowItsBudgetIsTheBudgetItRunsOn) {
  // CatchSearch::Plan's budget cap (E1-F17): a wake that has a replan to run
  // behind the search gives the search less than its own budget. On the fake
  // clock (3 ms a read) the cap decides how many candidates get their IK.
  const auto run = [](double budget_s, std::int64_t cap_ns) {
    auto rig = std::make_unique<Rig>();
    rig->params.budget_s = budget_s;
    EXPECT_TRUE(rig->Configure(&FakeClock));
    const auto traj = rig->Traj();
    SearchStats stats;
    static_cast<void>(rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                                       NowReal{kNow}, cap_ns, stats));
    return stats;
  };
  // Its own budget holds every candidate.
  const SearchStats own = run(1.0, 0);
  EXPECT_FALSE(own.budget_hit);
  ASSERT_GT(own.n_ik, 2);
  // A cap below it is the budget: fewer IKs, and the record says the budget
  // cut them — exactly as a search whose OWN budget is that small.
  const SearchStats capped = run(1.0, 10 * kMs);
  EXPECT_TRUE(capped.budget_hit);
  EXPECT_LT(capped.n_ik, own.n_ik);
  EXPECT_GE(capped.n_ik, 1) << "the first candidate always runs";
  const SearchStats small = run(0.010, 0);
  EXPECT_EQ(capped.n_ik, small.n_ik);
  EXPECT_EQ(capped.budget_hit, small.budget_hit);
  // The smaller of the two decides, whichever it is.
  const SearchStats cap_above = run(0.010, 1000 * kMs);
  EXPECT_EQ(cap_above.n_ik, small.n_ik);
  EXPECT_TRUE(cap_above.budget_hit);
  // No cap is 0; a cap that is not positive is no cap either.
  const SearchStats negative = run(1.0, -10 * kMs);
  EXPECT_EQ(negative.n_ik, own.n_ik);
  EXPECT_FALSE(negative.budget_hit);
}

// ── 4. Switching and freeze ──────────────────────────────────────────────────

TEST(GridCatchSearchSwitch, HoldsTheCurrentPlanWhenNothingIsBetter) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  PlanSnapshot first = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                                        NowReal{kNow}, 0, stats);
  ASSERT_TRUE(first.valid);
  first.plan_id = 11;
  rig->search.NotePublished(first);
  auto rt = rig->Rt();
  rt.plan_active = true;
  rt.plan_id = 11;
  static_cast<void>(rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rt, kNoSegments,
                                     NowReal{kNow}, 0, stats));
  EXPECT_EQ(stats.decision, SwitchDecision::kHeldHysteresis);
  EXPECT_FALSE(stats.publish);
}

TEST(GridCatchSearchSwitch, NothingReplacesAPlanInsideTheFreezeWindow) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  PlanSnapshot first = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                                        NowReal{kNow}, 0, stats);
  ASSERT_TRUE(first.valid);
  first.plan_id = 12;
  rig->search.NotePublished(first);
  auto rt = rig->Rt();
  rt.plan_active = true;
  rt.plan_id = 12;
  // 'now' moved to within T_freeze of the committed catch instant.
  const NowReal late{first.t_c_ns - static_cast<std::int64_t>(0.5 * rig->params.t_freeze * 1e9)};
  static_cast<void>(
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rt, kNoSegments, late, 0, stats));
  EXPECT_EQ(stats.decision, SwitchDecision::kHeldFreeze);
  EXPECT_FALSE(stats.publish);
}

TEST(GridCatchSearchSwitch, AnInfeasibleCurrentPlanIsReplacedWhenTheJumpIsSmall) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  PlanSnapshot first = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                                        NowReal{kNow}, 0, stats);
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
      rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rt, kNoSegments, NowReal{kNow}, 0, stats);
  EXPECT_EQ(stats.decision, SwitchDecision::kReplaced);
  EXPECT_TRUE(stats.publish);
  EXPECT_TRUE(next.valid);
  EXPECT_DOUBLE_EQ(next.gamma_g0, 1.0) << "γ must continue from the reference, not restart";

  // Same, but the reference is mid-ramp and moving: the jump limit refuses.
  rt.gamma = 0.0;
  rt.gamma_d = 5.0;
  first.p_c[0] += 1.0;  // a 1 m catch-point jump
  rig->search.NotePublished(first);
  static_cast<void>(rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rt, kNoSegments,
                                     NowReal{kNow}, 0, stats));
  EXPECT_EQ(stats.decision, SwitchDecision::kHeldJump);
  EXPECT_FALSE(stats.publish);
}

TEST(GridCatchSearchSwitch, UnderASegmentModeTheJumpLimitIsNotJudged) {
  // E1-F16: the η_jump bound is the step a switch puts into the L4 reference's
  // u_des. An arm that follows a segment planner's segments has no such
  // reference, so the 1 m switch the closed_form law holds (kHeldJump, the case
  // above) is decided by ΔJ alone there. Same search, same wake, one flag.
  for (const bool follows : {false, true}) {
    SCOPED_TRACE(follows ? "the arm follows segments" : "closed_form");
    auto rig = std::make_unique<Rig>();
    rig->constants.follows_segments = follows;
    ASSERT_TRUE(rig->Configure());
    const auto traj = rig->Traj();
    SearchStats stats;
    PlanSnapshot first = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                                          NowReal{kNow}, 0, stats);
    ASSERT_TRUE(first.valid);
    first.plan_id = 13;
    first.t_c_ns = kNow + 5000 * kMs;  // no candidate is at it: the current plan is infeasible
    first.p_c[0] += 1.0;               // a 1 m catch-point jump
    rig->search.NotePublished(first);
    auto rt = rig->Rt();
    rt.plan_active = true;
    rt.plan_id = 13;
    rt.ref_valid = true;
    rt.gamma = 0.0;  // mid-ramp and moving: the jump limit refuses under closed_form
    rt.gamma_d = 5.0;
    static_cast<void>(rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rt, kNoSegments,
                                       NowReal{kNow}, 0, stats));
    EXPECT_EQ(stats.decision, follows ? SwitchDecision::kReplaced : SwitchDecision::kHeldJump);
    EXPECT_EQ(stats.publish, follows);
  }
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
  PlanSnapshot first = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                                        NowReal{kNow}, 0, stats);
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
  const PlanSnapshot next = rig->search.Plan(moved, Rig::Cov(moved, 0.002), true, rt, kNoSegments,
                                             NowReal{kNow}, 0, stats);
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

TEST(GridCatchSearchReview, OneSlowSolveDoesNotStopThePlannerForGood) {
  auto rig = std::make_unique<Rig>();
  rig->params.budget_s = 0.020;  // the shipped budget; the spike is 1.5× it
  g_slow_ns = 0;
  g_slow_calls = 0;
  ASSERT_TRUE(rig->Configure(&OneSlowIkClock));
  const auto traj = rig->Traj();
  SearchStats stats;
  static_cast<void>(rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                                     NowReal{kNow}, 0, stats));
  ASSERT_GE(stats.ik_ns_max, 30 * kMs) << "premise: the first cycle saw one 30 ms solve";
  // Every later cycle still runs IK, and the search widens again as the
  // estimate relaxes — it used to stop at the first check forever.
  int widest = 0;
  for (int cycle = 0; cycle < 40; ++cycle) {
    static_cast<void>(rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                                       NowReal{kNow}, 0, stats));
    EXPECT_GE(stats.n_ik, 1) << "cycle " << cycle;
    widest = std::max(widest, static_cast<int>(stats.n_ik));
  }
  EXPECT_GT(widest, 1) << "the estimate never relaxed";
}

TEST(GridCatchSearchReview, AMovedPredictionOfTheFollowedCandidateIsRefreshed) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  PlanSnapshot first = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                                        NowReal{kNow}, 0, stats);
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
  const PlanSnapshot next = rig->search.Plan(moved, Rig::Cov(moved, 0.002), true, rt, kNoSegments,
                                             NowReal{kNow}, 0, stats);
  EXPECT_EQ(stats.decision, SwitchDecision::kRefreshed);
  EXPECT_TRUE(stats.publish);
  ASSERT_TRUE(next.valid);
  EXPECT_EQ(next.t_c_ns, first.t_c_ns) << "the same candidate, not a different one";
  EXPECT_NEAR(next.p_c[1] - first.p_c[1], 0.005, 1e-12);
}

TEST(GridCatchSearchReview, TheCurrentPlanIsWhatTheRtFollowsNotTheLastPublish) {
  // P1 is followed; P2 was published but the RT refused it (freeze, age). The
  // planner must keep treating P1 as current, not fall back to "no current"
  // and republish every cycle without hysteresis.
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  SearchStats stats;
  PlanSnapshot p1 = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                                     NowReal{kNow}, 0, stats);
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
  static_cast<void>(rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rt, kNoSegments,
                                     NowReal{kNow}, 0, stats));
  EXPECT_NE(stats.decision, SwitchDecision::kNoCurrent);
  EXPECT_EQ(stats.decision, SwitchDecision::kHeldHysteresis);
  EXPECT_FALSE(stats.publish);
}

TEST(GridCatchSearchReview, SettlingWhileFollowingHoldsInsteadOfPublishingNoPlan) {
  auto rig = std::make_unique<Rig>();
  rig->params.n_settle = 1;
  ASSERT_TRUE(rig->Configure());
  SearchStats stats;
  const auto settle = rig->Traj(1);
  static_cast<void>(rig->search.Plan(settle, Rig::Cov(settle, 0.002), true, rig->Rt(), kNoSegments,
                                     NowReal{kNow}, 0, stats));
  ASSERT_TRUE(stats.settling) << "premise: the first snapshot of a track settles";
  const auto traj = rig->Traj(2);
  PlanSnapshot first = rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                                        NowReal{kNow}, 0, stats);
  ASSERT_TRUE(first.valid);
  first.plan_id = 41;
  rig->search.NotePublished(first);
  auto rt = rig->Rt();
  rt.plan_active = true;
  rt.plan_id = 41;
  // A new track generation restarts the settle count.
  const auto other = rig->Traj(1, 8);
  const PlanSnapshot next = rig->search.Plan(other, Rig::Cov(other, 0.002), true, rt, kNoSegments,
                                             NowReal{kNow}, 0, stats);
  ASSERT_TRUE(stats.settling);
  EXPECT_FALSE(next.valid);
  EXPECT_FALSE(stats.publish) << "a no-plan publish would overwrite the followed plan's box";
  EXPECT_EQ(stats.decision, SwitchDecision::kHeldNoCandidate);
}

// ── 5. G3-K ──────────────────────────────────────────────────────────────────

TEST(GridCatchSearchPlan, AFullSearchAllocatesNothing) {
  auto rig = std::make_unique<Rig>();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  const auto cov = Rig::Cov(traj, 0.002);
  const auto rt = rig->Rt();
  SearchStats stats;
  ASSERT_TRUE(
      rig->search.Plan(traj, cov, true, rt, kNoSegments, NowReal{kNow}, 0, stats).valid);  // warm
  std::size_t heap = 0;
  std::uint64_t eigen = 0;
  bool valid = false;
  int n_ik = 0;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (int i = 0; i < 5; ++i) {
      rig->search.ResetTrial();
      valid = rig->search.Plan(traj, cov, true, rt, kNoSegments, NowReal{kNow}, 0, stats).valid;
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

// ── 5a. The arm: where the reach starts (E1-F17 #743) ────────────────────────

// A segment of `nv` joints that is moving at every instant, with node values
// that tell joint, node and derivative apart. Four 50 ms intervals from
// `t0_ns`; nothing but what the sampler reads is filled.
[[nodiscard]] SegmentSnapshot MovingSegment(int nv, std::uint32_t seq, std::int64_t t0_ns,
                                            double bias) {
  SegmentSnapshot s{};
  s.valid = true;
  s.segment_seq = seq;
  s.nv = nv;
  s.n_nodes = 4;
  s.n_pre = 0;
  s.dt_ns = 50 * kMs;
  s.t0_ns = t0_ns;
  s.t_c_ns = t0_ns;
  for (int k = 0; k <= s.n_nodes; ++k) {
    for (int d = 0; d < nv; ++d) {
      const auto e = static_cast<std::size_t>(k * kMaxSegmentNv + d);
      s.q[e] = bias + 0.05 * d + 0.02 * k;
      s.qd[e] = 0.3 - 0.04 * d + 0.01 * k;
      s.qdd[e] = -0.2 + 0.03 * d - 0.05 * k;
    }
  }
  return s;
}

// T_arm ≠ 0 and a model order that is the device order reversed: the start is
// read at now + T_arm, in DEVICE order.
constexpr std::int64_t kArmLagNs = 30 * kMs;
constexpr std::int64_t kNowLead = kNow + kArmLagNs;

[[nodiscard]] std::unique_ptr<Rig> ArmRig() {
  auto rig = std::make_unique<Rig>();
  for (int j = 0; j < rig->arm.nv; ++j) {
    rig->model.device_of_model[static_cast<std::size_t>(j)] = rig->arm.nv - 1 - j;
  }
  rig->constants.t_arm_s = static_cast<double>(kArmLagNs) * 1e-9;
  return rig;
}

// The RT's report with a seeded, moving command — distinct per device joint,
// and nowhere near a segment built by MovingSegment.
[[nodiscard]] PlannerRtState MovingCommandRt(const Rig& rig) {
  PlannerRtState rt = rig.Rt();
  rt.cmd_seeded = true;
  for (int d = 0; d < rig.arm.nv; ++d) {
    rt.qd_cmd[static_cast<std::size_t>(d)] = 0.011 * (d + 1);
  }
  return rt;
}

// One search on `arm`; the candidates were judged, so the start was built.
void SearchOn(Rig& rig, const PlannerRtState& rt, const ReportedSegments& arm) {
  const auto traj = rig.Traj();
  SearchStats stats;
  static_cast<void>(
      rig.search.Plan(traj, Rig::Cov(traj, 0.002), true, rt, arm, NowReal{kNow}, 0, stats));
  ASSERT_GT(stats.n_ik, 0) << "no candidate was judged: the reach start was not built";
}

// The start the last search used is `segment` read at now_lead by the RT's
// own evaluator, device → model order, to the bit.
void ExpectStartOnSegment(const Rig& rig, const SegmentSnapshot& segment) {
  std::array<double, kMaxSegmentNv> q{};
  std::array<double, kMaxSegmentNv> qd{};
  std::array<double, kMaxSegmentNv> qdd{};
  ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(segment, kNowLead, q, qd, qdd));
  const std::span<const double> q0 = rig.search.ReachStartPositionForTesting();
  const std::span<const double> w0 = rig.search.ReachStartVelocityForTesting();
  ASSERT_EQ(q0.size(), static_cast<std::size_t>(rig.arm.nv));
  ASSERT_EQ(w0.size(), static_cast<std::size_t>(rig.arm.nv));
  for (int j = 0; j < rig.arm.nv; ++j) {
    const auto m = static_cast<std::size_t>(j);
    const auto d = static_cast<std::size_t>(rig.arm.nv - 1 - j);
    EXPECT_TRUE(rtc::testing::BitsEqual(q0[m], q[d])) << "q, model joint " << j;
    EXPECT_TRUE(rtc::testing::BitsEqual(w0[m], qd[d])) << "q̇, model joint " << j;
  }
}

// The start the last search used is the reported command, device → model
// order, to the bit — the velocity zero while the command is not seeded.
void ExpectStartOnCommand(const Rig& rig, const PlannerRtState& rt) {
  const std::span<const double> q0 = rig.search.ReachStartPositionForTesting();
  const std::span<const double> w0 = rig.search.ReachStartVelocityForTesting();
  ASSERT_EQ(q0.size(), static_cast<std::size_t>(rig.arm.nv));
  ASSERT_EQ(w0.size(), static_cast<std::size_t>(rig.arm.nv));
  for (int j = 0; j < rig.arm.nv; ++j) {
    const auto m = static_cast<std::size_t>(j);
    const auto d = static_cast<std::size_t>(rig.arm.nv - 1 - j);
    EXPECT_TRUE(rtc::testing::BitsEqual(q0[m], rt.q_cmd[d])) << "q, model joint " << j;
    EXPECT_TRUE(rtc::testing::BitsEqual(w0[m], rt.cmd_seeded ? rt.qd_cmd[d] : 0.0))
        << "q̇, model joint " << j;
  }
}

TEST(GridCatchSearchArm, WhileASegmentIsFollowedTheReachStartsOnItAtNowLead) {
  auto rig = ArmRig();
  ASSERT_TRUE(rig->Configure());
  const PlannerRtState rt = MovingCommandRt(*rig);
  auto arm = std::make_unique<ReportedSegments>();
  arm->has_following = true;
  // now_lead is 20 ms into the segment's first interval: between two nodes.
  arm->following = MovingSegment(rig->arm.nv, 5, kNowLead - 20 * kMs, 0.1);
  ASSERT_NO_FATAL_FAILURE(SearchOn(*rig, rt, *arm));
  ASSERT_NO_FATAL_FAILURE(ExpectStartOnSegment(*rig, arm->following));
  // Non-vacuous: that state is neither the reported command nor the segment
  // read at the wake instant (T_arm earlier).
  std::array<double, kMaxSegmentNv> q{};
  std::array<double, kMaxSegmentNv> qd{};
  std::array<double, kMaxSegmentNv> qdd{};
  ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(arm->following, kNowLead, q, qd, qdd));
  std::array<double, kMaxSegmentNv> q_wake{};
  ASSERT_TRUE(
      NodeTrajectoryFollower::SampleJoints(arm->following, kNowLead - 10 * kMs, q_wake, qd, qdd));
  for (int d = 0; d < rig->arm.nv; ++d) {
    const auto u = static_cast<std::size_t>(d);
    EXPECT_NE(q[u], rt.q_cmd[u]) << d;
    EXPECT_NE(q[u], q_wake[u]) << d;
  }
  // The same wake with nothing reported starts on the command again: the
  // start is this call's, not a remembered one.
  ASSERT_NO_FATAL_FAILURE(SearchOn(*rig, rt, kNoSegments));
  ASSERT_NO_FATAL_FAILURE(ExpectStartOnCommand(*rig, rt));
}

TEST(GridCatchSearchArm, ThePendingSegmentIsTheStartFromItsOwnNodeZeroOn) {
  // SourceSegmentAt: the pending segment once now_lead has reached its node 0,
  // the followed one before that — one nanosecond apart.
  for (const bool due : {false, true}) {
    SCOPED_TRACE(due ? "pending, node 0 at now_lead" : "pending, node 0 one ns after now_lead");
    auto rig = ArmRig();
    ASSERT_TRUE(rig->Configure());
    const PlannerRtState rt = MovingCommandRt(*rig);
    auto arm = std::make_unique<ReportedSegments>();
    arm->has_following = true;
    arm->following = MovingSegment(rig->arm.nv, 5, kNowLead - 20 * kMs, 0.1);
    arm->has_pending = true;
    arm->pending = MovingSegment(rig->arm.nv, 6, due ? kNowLead : kNowLead + 1, -0.3);
    const SegmentSnapshot& want = due ? arm->pending : arm->following;
    ASSERT_EQ(rtc::catching::SourceSegmentAt(*arm, kNowLead), &want);
    ASSERT_NO_FATAL_FAILURE(SearchOn(*rig, rt, *arm));
    ASSERT_NO_FATAL_FAILURE(ExpectStartOnSegment(*rig, want));
  }
}

TEST(GridCatchSearchArm, WithNoReadableSegmentTheReachStartsOnTheReportedCommand) {
  const char* const cases[] = {"nothing reported",
                               "a pending segment that is not due, nothing followed",
                               "a followed segment of another joint count",
                               "a followed segment whose node 0 is after now_lead",
                               "a followed segment with a NaN node",
                               "a followed segment with a NaN node velocity"};
  for (int i = 0; i < 6; ++i) {
    SCOPED_TRACE(cases[i]);
    auto rig = ArmRig();
    ASSERT_TRUE(rig->Configure());
    auto arm = std::make_unique<ReportedSegments>();
    const SegmentSnapshot moving = MovingSegment(rig->arm.nv, 5, kNowLead - 20 * kMs, 0.1);
    switch (i) {
      case 1:
        arm->has_pending = true;
        arm->pending = MovingSegment(rig->arm.nv, 6, kNowLead + 1, -0.3);
        break;
      case 2:
        arm->has_following = true;
        arm->following = moving;
        arm->following.nv = rig->arm.nv - 1;
        break;
      case 3:
        arm->has_following = true;
        arm->following = MovingSegment(rig->arm.nv, 5, kNowLead + 1, 0.1);
        break;
      case 4:
        arm->has_following = true;
        arm->following = moving;
        arm->following.q[2] = std::numeric_limits<double>::quiet_NaN();
        break;
      case 5:
        arm->has_following = true;
        arm->following = moving;
        arm->following.qd[3] = std::numeric_limits<double>::quiet_NaN();
        break;
      default:
        break;
    }
    // Seeded and moving, then unseeded: the command's own two forms.
    PlannerRtState rt = MovingCommandRt(*rig);
    ASSERT_NO_FATAL_FAILURE(SearchOn(*rig, rt, *arm));
    ASSERT_NO_FATAL_FAILURE(ExpectStartOnCommand(*rig, rt));
    rt.cmd_seeded = false;
    ASSERT_NO_FATAL_FAILURE(SearchOn(*rig, rt, *arm));
    ASSERT_NO_FATAL_FAILURE(ExpectStartOnCommand(*rig, rt));
  }
}

TEST(GridCatchSearchArm, ASearchFromAFollowedSegmentAllocatesNothing) {
  auto rig = ArmRig();
  ASSERT_TRUE(rig->Configure());
  const auto traj = rig->Traj();
  const auto cov = Rig::Cov(traj, 0.002);
  const PlannerRtState rt = MovingCommandRt(*rig);
  auto arm = std::make_unique<ReportedSegments>();
  arm->has_following = true;
  arm->following = MovingSegment(rig->arm.nv, 5, kNowLead - 20 * kMs, 0.1);
  arm->has_pending = true;
  arm->pending = MovingSegment(rig->arm.nv, 6, kNowLead - 5 * kMs, -0.3);
  SearchStats stats;
  static_cast<void>(rig->search.Plan(traj, cov, true, rt, *arm, NowReal{kNow}, 0, stats));  // warm
  std::size_t heap = 0;
  std::uint64_t eigen = 0;
  int n_ik = 0;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (int i = 0; i < 5; ++i) {
      rig->search.ResetTrial();
      static_cast<void>(rig->search.Plan(traj, cov, true, rt, *arm, NowReal{kNow}, 0, stats));
      n_ik += stats.n_ik;
    }
    heap = heap_gate.count();
    eigen = eigen_gate.violations();
  }
  EXPECT_GT(n_ik, 0) << "the gated loop judged no candidate — the start was not built";
  ASSERT_NO_FATAL_FAILURE(ExpectStartOnSegment(*rig, arm->pending));
  EXPECT_EQ(heap, 0U);
  EXPECT_EQ(eigen, 0U);
}

// ── 6. R-2 timing and G3-G acceptance (recorded) ─────────────────────────────

TEST(GridCatchSearchTiming, RecordsIkAndCycleTimesOnThisHost) {
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
    static_cast<void>(rig->search.Plan(traj, Rig::Cov(traj, 0.002), true, rig->Rt(), kNoSegments,
                                       NowReal{kNow}, 0, stats));
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
