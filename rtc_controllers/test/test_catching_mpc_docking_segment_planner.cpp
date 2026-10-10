// The docking segment planner (E1-F16 #742; mpc_docking_segment_planner.hpp).
//
// The planner is the second implementation of the SegmentPlanner interface:
// the joint-node segments the RT follows, solved by MpcDockingSegmentCore on a
// grid anchored at the catch instant. What is tested is the contract its
// header states, on the rig of test_catching_nlp_catch_search.cpp:
//   1. configure — one initialised core per pre-catch count, warmed up, and
//      what Configure refuses.
//   2. the first segment, adopted — the search's own solution, when it is this
//      plan's, of this RT report and on this grid, is re-evaluated and
//      published bit for bit; convergence is the search's statement.
//   3. the first segment, solved — from an arm at rest, toward the plan's catch
//      pose; and every refusal with its named outcome.
//   4. replans — a later segment of the followed plan, started from the
//      segment the RT reports; the queries of the interface on what was
//      published; and the decision this class exists to pin: NOTHING IS
//      PUBLISHED AFTER THE CATCH.
//   4a. the first segment of a REPLACEMENT (E1-F17 #743) — the RT follows a
//      plan and the search replaces it: the new plan's first segment starts
//      on the segment the RT reports, a search's solution is published only
//      when it started there, and the followed plan's segments stay.
//   5. allocation — a C-level malloc gate over PlanFirst and Replan, the QP
//      solver bracketed out and the core's own stages gated again inside it.
//
// THE SCENE. The search of the nlp test is built on the same arm, the same
// limits and the same core parameters, and run on a ball coming down the wait
// pose's capture axis: it produces a plan and the arm trajectory it solved
// (Solution()), which are known to be valid. The planner under test is
// configured on that grid (DockingGridMismatch's condition) so that the
// search's solution is on its grid.
//
// THE CLOCK steps on every read, and a call's FIRST read is the instant the
// test names (SetClockAt): a solve's start, its age check and its lead are
// then exact numbers, and the budget tests make the step large.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controllers/catching/catch_pose_ik.hpp"
#include "rtc_controllers/catching/catch_search.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_planner.hpp"
#include "rtc_controllers/catching/nlp_catch_search.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "rtc_controllers/catching/search_stats.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/testing/catch_arm_fixture.hpp"
#include "rtc_controllers/testing/grid_catch_search_fixture.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/multibody/data.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <utility>

namespace {

using rtc::catching::BallPrediction;
using rtc::catching::CatchSolution;
using rtc::catching::CovarianceSnapshot;
using rtc::catching::kMaxSegmentNodes;
using rtc::catching::kMaxSegmentNv;
using rtc::catching::kMpcDockingMaxRtStateAgeNs;
using rtc::catching::MakeMpcDockingSegmentPlanner;
using rtc::catching::Mode;
using rtc::catching::MpcDockingReason;
using rtc::catching::MpcDockingReasonName;
using rtc::catching::MpcDockingSegmentCoreInput;
using rtc::catching::MpcDockingSegmentPlanner;
using rtc::catching::MpcDockingSegmentPlannerConstants;
using rtc::catching::MpcDockingSegmentPlannerModel;
using rtc::catching::MpcDockingSegmentPlannerParams;
using rtc::catching::MpcDockingStage;
using rtc::catching::NlpCatchSearch;
using rtc::catching::NlpCatchSearchConstants;
using rtc::catching::NlpCatchSearchModel;
using rtc::catching::NlpCatchSearchParams;
using rtc::catching::NlpRejectName;
using rtc::catching::NodeTrajectoryFollower;
using rtc::catching::NowReal;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanSnapshot;
using rtc::catching::ReportedSegments;
using rtc::catching::SearchStats;
using rtc::catching::SegmentKind;
using rtc::catching::SegmentOutcome;
using rtc::catching::SegmentPlanner;
using rtc::catching::SegmentRecord;
using rtc::catching::SegmentSnapshot;
using rtc::catching::TrajectorySnapshot;
using rtc::catching::ValidateSegmentNodes;
using rtc::testing::BitsEqual;
namespace fx = rtc::testing::mpc_segment_core;
namespace dk = rtc::testing::mpc_docking;

static_assert(noexcept(std::declval<MpcDockingSegmentPlanner&>().PlanFirst(
    std::declval<const PlannerRtState&>(), std::declval<const PlanSnapshot&>(),
    std::declval<const BallPrediction&>(), std::declval<const CatchSolution*>(),
    std::declval<SegmentSnapshot&>(), std::declval<SegmentRecord&>())));
static_assert(noexcept(std::declval<MpcDockingSegmentPlanner&>().Replan(
    std::declval<const PlannerRtState&>(), std::declval<const BallPrediction&>(),
    std::declval<SegmentSnapshot&>(), std::declval<SegmentRecord&>())));

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kNow = 10'000 * kMs;
constexpr std::uint64_t kActivation = 3;
constexpr std::uint64_t kTrack = 7;
constexpr std::uint32_t kPlanId = 4;
// The rig's grid: 50 ms before and after the catch node.
constexpr std::int64_t kDtPreNs = 50 * kMs;
// A replan at first-read instant `now` reaches the grid point t_c − n·Δ_pre
// when t_c − now − (T_arm + budget.replan_s + 2h) = n·Δ_pre + a margin below Δ_pre:
// 25 ms + 2 × 2 ms of lead, plus 5 ms of margin.
constexpr std::int64_t kReplanLeadNs = 34 * kMs;
constexpr std::int64_t kReplanBudgetNs = 25 * kMs;

// ── The clock ────────────────────────────────────────────────────────────────

std::atomic<std::int64_t> g_clock{0};
std::atomic<std::int64_t> g_step{1000};  // ns per read

std::int64_t StepClock() noexcept {
  const std::int64_t step = g_step.load(std::memory_order_relaxed);
  return g_clock.fetch_add(step, std::memory_order_relaxed) + step;
}

void SetClockStep(std::int64_t step_ns) {
  g_clock.store(0);
  g_step.store(step_ns);
}

// The next read of the clock returns `first_read_ns`; every read after it is
// `step_ns` later.
void SetClockAt(std::int64_t first_read_ns, std::int64_t step_ns = 1000) {
  g_step.store(step_ns);
  g_clock.store(first_read_ns - step_ns);
}

[[nodiscard]] std::int64_t Ns(double s) {
  return static_cast<std::int64_t>(std::llround(s * 1e9));
}

// ── The rig: the nlp search's, and a planner configured on the same grid ─────

// model joint m is the arm device's joint kDeviceOfModel[m].
constexpr std::array<int, 6> kDeviceOfModel{2, 0, 1, 5, 3, 4};

template <typename V>
[[nodiscard]] std::array<double, 6> ToDevice(const V& q_model) {
  std::array<double, 6> out{};
  for (std::size_t m = 0; m < 6; ++m) {
    out[static_cast<std::size_t>(kDeviceOfModel[m])] = q_model[static_cast<Eigen::Index>(m)];
  }
  return out;
}

struct Rig {
  rtc::testing::Arm arm = rtc::testing::Arm6R();
  dk::Rig dock = dk::MakeRig(fx::Synthetic6R());
  NlpCatchSearchModel model{};
  NlpCatchSearchConstants constants{};
  NlpCatchSearchParams params{};
  rtc::catching::CatchPoseIkOptions ik{};
  NlpCatchSearch search;
  Eigen::VectorXd q_wait;  // model order

  Rig() {
    q_wait = dock.arm.q_nominal;
    model.arm = arm.model;
    model.handle = arm.handle.get();
    model.catch_frame = arm.frame;
    model.nv = arm.nv;
    for (std::size_t m = 0; m < 6; ++m) {
      model.device_of_model[m] = kDeviceOfModel[m];
    }
    constants.t_arm_s = 0.0;
    constants.control_dt = 0.002;
    constants.t_close_lead = 0.1;

    params.cand_dt = 0.04;
    params.t_lead_min = 0.1;
    params.t_max = 0.4;
    params.cand_capacity = 16;
    params.wait_pose_n = arm.nv;
    const std::array<double, 6> wait_dev = ToDevice(q_wait);
    std::copy(wait_dev.begin(), wait_dev.end(), params.wait_pose.begin());
    params.n_pre_min = 2;
    params.n_pre_max = 8;
    params.dt_pre = 0.05;
    params.n_stop = 7;
    params.dt_stop = 0.05;
    params.n_stop_blocks = 4;
    params.stop_block_sizes = {1, 1, 2, 3};
    params.budget_s = 0.04;
    params.solve_budget_s = 0.004;
    params.start_lead_s = 0.004;
    params.max_solves = 8;
    params.w_time = 0.0;
    params.w_switch = 0.0;
    params.t_ref_s = 0.5;
    params.rank_w_q = 1.0;
    params.core = dock.params;
    params.limits = dock.limits;
    ik.max_iter = 60;
    ik.manipulability_min = 0.0;
  }

  [[nodiscard]] bool Configure(std::string* error = nullptr) {
    SetClockStep(1000);
    return search.Configure(model, constants, params, ik, &StepClock, error);
  }

  // The planner's model: the search's arm, device order and limits; the
  // ratings are `rating_scale` times the cores' own velocity box.
  [[nodiscard]] MpcDockingSegmentPlannerModel PlannerModel(double rating_scale = 2.0) const {
    MpcDockingSegmentPlannerModel m;
    m.arm = arm.model;
    m.catch_frame = arm.frame;
    m.nv = arm.nv;
    m.device_of_model = model.device_of_model;
    m.limits = params.limits;
    for (Eigen::Index j = 0; j < arm.nv; ++j) {
      m.qd_rating[static_cast<std::size_t>(j)] = rating_scale * params.limits.qd_max[j];
    }
    m.warm_pose = params.wait_pose;
    return m;
  }

  [[nodiscard]] MpcDockingSegmentPlannerConstants PlannerConstants() const {
    MpcDockingSegmentPlannerConstants c;
    c.t_arm_s = constants.t_arm_s;
    c.control_dt = constants.control_dt;
    return c;
  }

  // The search's grid, and its core parameters.
  [[nodiscard]] MpcDockingSegmentPlannerParams PlannerParams() const {
    MpcDockingSegmentPlannerParams p;
    p.n_pre_max = params.n_pre_max;
    p.dt_pre_s = params.dt_pre;
    p.n_stop = params.n_stop;
    p.dt_stop_s = params.dt_stop;
    p.n_stop_blocks = params.n_stop_blocks;
    p.stop_block_sizes = params.stop_block_sizes;
    p.core = params.core;
    return p;
  }

  // The arm at rest on its wait pose, following no plan.
  [[nodiscard]] PlannerRtState RestingRt(std::int64_t now) const {
    PlannerRtState rt = rtc::testing::TrackingRtState(kActivation, ToDevice(q_wait));
    rt.rt_iteration = 500 + static_cast<std::uint64_t>((now - kNow) / kMs);
    rt.rt_state_ns = now - kMs;
    return rt;
  }

  // The arm following plan `t_c_ns`.
  [[nodiscard]] PlannerRtState FollowingRt(std::int64_t now, std::int64_t t_c_ns) const {
    PlannerRtState rt = RestingRt(now);
    rt.mode = static_cast<std::uint8_t>(Mode::kApproach);
    rt.plan_active = true;
    rt.plan_id = kPlanId;
    rt.plan_t_c_ns = t_c_ns;
    return rt;
  }
};

// ── The throw: a ball down the wait pose's capture axis ─────────────────────

struct Throw {
  TrajectorySnapshot traj;
  CovarianceSnapshot cov;
};

// On the wait pose's entrance plane `t_star_s` after kNow. 20 samples 50 ms
// apart from kNow.
[[nodiscard]] Throw AxisThrow(const Rig& rig, double t_star_s, double speed = 0.8,
                              double sigma = 0.002) {
  const dk::HandState h = dk::HandAt(rig.dock, rig.q_wait, Eigen::VectorXd::Zero(rig.arm.nv));
  const Eigen::Vector3d v_b = -speed * h.R.col(2);
  const Eigen::Vector3d p_e = h.p + h.R * Eigen::Vector3d(0.0, 0.0, rig.params.core.s_ent);
  Throw t;
  t.traj = rtc::testing::LineTrajectory(p_e - v_b * t_star_s, v_b, kNow, 50 * kMs, 20, 0,
                                        /*seq=*/1, kTrack, kActivation, kNow - 5 * kMs);
  t.cov = rtc::testing::IsotropicCovariance(t.traj, sigma);
  return t;
}

const ReportedSegments& NoSegments() {
  static const ReportedSegments none{};
  return none;
}

// ── One scene: the search's plan and solution, and a configured planner ──────

struct Scene {
  Rig rig;
  Throw ball;
  PlannerRtState rt;
  PlanSnapshot plan;
  CatchSolution solution;
  std::unique_ptr<MpcDockingSegmentPlanner> planner;

  // gtest ASSERTs: call as ASSERT_NO_FATAL_FAILURE(scene->Setup()).
  void Setup(double t_star_s = 0.28) {
    std::string err;
    ASSERT_TRUE(rig.Configure(&err)) << err;
    ball = AxisThrow(rig, t_star_s);
    rt = rig.RestingRt(kNow);
    SearchStats stats;
    plan = rig.search.Plan(ball.traj, ball.cov, true, rt, NoSegments(), NowReal{kNow}, 0, stats);
    ASSERT_TRUE(plan.valid) << "the search chose nothing: " << NlpRejectName(stats.nlp.reason);
    const CatchSolution* sol = rig.search.Solution();
    ASSERT_NE(sol, nullptr);
    solution = *sol;
    ASSERT_TRUE(solution.converged);
    ASSERT_EQ(solution.seg.dt_catch_ns, 0) << "a fixed-grid solution is what the planner adopts";
    ASSERT_EQ(solution.seg.rt_iteration, rt.rt_iteration);
    // The cycle gives the plan its id.
    plan.plan_id = kPlanId;
    planner = MakeMpcDockingSegmentPlanner(rig.PlannerModel(), rig.PlannerConstants(),
                                           rig.PlannerParams(), &StepClock, &err);
    ASSERT_NE(planner, nullptr) << err;
  }

  [[nodiscard]] BallPrediction Ball() const { return BallPrediction{&ball.traj, &ball.cov, true}; }
};

// ── Calling the planner ──────────────────────────────────────────────────────

struct Result {
  SegmentSnapshot out{};
  SegmentRecord rec{};
  bool ok{false};
};

[[nodiscard]] std::string Describe(const SegmentRecord& r) {
  std::ostringstream os;
  os << "\noutcome " << SegmentOutcomeName(r.outcome) << ", kind " << SegmentKindName(r.kind)
     << ", core " << r.core_reason_name << ", from_search " << r.from_search << ", iterations "
     << r.iterations << ", solve " << r.solve_ns << " ns, slack_c " << r.slack_max << ", slack_v "
     << r.slack_v << ", speed ratio " << r.speed_ratio_max << ", x0_speed " << r.x0_speed
     << ", source_seq " << r.source_seq;
  return os.str();
}

[[nodiscard]] std::unique_ptr<Result> RunFirst(MpcDockingSegmentPlanner& planner,
                                               const PlannerRtState& rt, const PlanSnapshot& plan,
                                               const BallPrediction& ball,
                                               const CatchSolution* solution, std::int64_t now,
                                               std::int64_t step_ns = 1000) {
  auto r = std::make_unique<Result>();
  SetClockAt(now, step_ns);
  r->ok = planner.PlanFirst(rt, plan, ball, solution, r->out, r->rec);
  return r;
}

[[nodiscard]] std::unique_ptr<Result> RunReplan(MpcDockingSegmentPlanner& planner,
                                                const PlannerRtState& rt,
                                                const BallPrediction& ball, std::int64_t now,
                                                std::int64_t step_ns = 1000) {
  auto r = std::make_unique<Result>();
  SetClockAt(now, step_ns);
  r->ok = planner.Replan(rt, ball, r->out, r->rec);
  return r;
}

using Nodes = decltype(SegmentSnapshot::q);

// The index of the first entry whose bits differ, or −1.
[[nodiscard]] int FirstDifference(const Nodes& a, const Nodes& b) {
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!BitsEqual(a[i], b[i])) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

// ── 1. Configure ─────────────────────────────────────────────────────────────

TEST(MpcDockingPlannerConfigure, BuildsAWarmedCoreForEveryPreCatchCountOnTheSearchsGrid) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  const MpcDockingSegmentPlanner& planner = *s->planner;
  const int n_pre_max = s->rig.params.n_pre_max;
  EXPECT_TRUE(planner.Configured());
  EXPECT_EQ(planner.Nv(), 6);
  for (int n_pre = 1; n_pre <= n_pre_max; ++n_pre) {
    SCOPED_TRACE(n_pre);
    ASSERT_NE(planner.Core(n_pre), nullptr);
    const rtc::catching::MpcDockingSegmentCore& core = *planner.Core(n_pre);
    EXPECT_TRUE(core.IsInitialized());
    EXPECT_EQ(core.CatchNode(), n_pre);
    EXPECT_EQ(core.NumNodes(), n_pre + s->rig.params.n_stop);
    EXPECT_NE(planner.LastResult(n_pre), nullptr);
  }
  // No core outside 1..n_pre_max.
  EXPECT_EQ(planner.Core(0), nullptr);
  EXPECT_EQ(planner.Core(-1), nullptr);
  EXPECT_EQ(planner.Core(n_pre_max + 1), nullptr);
  EXPECT_EQ(planner.LastResult(0), nullptr);
  EXPECT_EQ(planner.LastResult(n_pre_max + 1), nullptr);
  // The warm-up solves took time on the clock: a step each, at least.
  EXPECT_GT(planner.WarmUpMaxNs(), 0);
  EXPECT_GE(planner.WarmUpTotalNs(), planner.WarmUpMaxNs());
  // The grid of the planner is the search's.
  EXPECT_EQ(planner.ControlDtNs(), 2 * kMs);
  EXPECT_TRUE(planner.StartsInTime(0, 2 * 2 * kMs + 1));
  EXPECT_FALSE(planner.StartsInTime(0, 2 * 2 * kMs)) << "publish + T_arm + 2 ticks < t0 is strict";
  // Never configured: no core.
  const MpcDockingSegmentPlanner never{};
  EXPECT_FALSE(never.Configured());
  EXPECT_EQ(never.Core(1), nullptr);
}

TEST(MpcDockingPlannerConfigure, RefusesWhatItCannotBuildAndSaysWhy) {
  Rig rig;

  struct Cfg {
    MpcDockingSegmentPlannerModel model;
    MpcDockingSegmentPlannerConstants consts;
    MpcDockingSegmentPlannerParams params;
    SegmentPlanner::ClockFn clock;
  };

  const auto refuses = [&rig](const char* label, auto&& edit, const char* fragment = nullptr) {
    SCOPED_TRACE(label);
    Cfg cfg{rig.PlannerModel(), rig.PlannerConstants(), rig.PlannerParams(), &StepClock};
    edit(cfg);
    MpcDockingSegmentPlanner planner;
    std::string err;
    SetClockStep(1000);
    EXPECT_FALSE(planner.Configure(cfg.model, cfg.consts, cfg.params, cfg.clock, &err));
    EXPECT_FALSE(planner.Configured());
    EXPECT_FALSE(err.empty());
    if (fragment != nullptr) {
      EXPECT_NE(err.find(fragment), std::string::npos)
          << "'" << err << "' lacks '" << fragment << "'";
    }
    // A refused Configure leaves no core behind.
    EXPECT_EQ(planner.Core(1), nullptr);
  };
  refuses("null clock", [](Cfg& c) { c.clock = nullptr; });
  refuses("n_pre_max = 0", [](Cfg& c) { c.params.n_pre_max = 0; });
  // More nodes than a segment holds: 18 + 7 stop nodes = 25.
  static_assert(18 + 7 > kMaxSegmentNodes);
  refuses("n_pre_max + n_stop over the node capacity", [](Cfg& c) { c.params.n_pre_max = 18; });
  refuses("too few stop blocks", [](Cfg& c) { c.params.n_stop_blocks = 2; });
  refuses("device_of_model above the joint count", [](Cfg& c) { c.model.device_of_model[1] = 6; });
  refuses("device_of_model negative", [](Cfg& c) { c.model.device_of_model[1] = -1; });
  refuses("zero rating", [](Cfg& c) { c.model.qd_rating[2] = 0.0; });
  refuses("negative rating", [](Cfg& c) { c.model.qd_rating[2] = -1.0; });
  refuses("NaN rating",
          [](Cfg& c) { c.model.qd_rating[2] = std::numeric_limits<double>::quiet_NaN(); });
  refuses("no arm model", [](Cfg& c) { c.model.arm = nullptr; });
  // The timing row is on (chance and timing_row default true in the rig's core
  // parameters), and a nominal closure instant outside (delta_lo, delta_hi) is
  // what the core's Init answers kTimingWindowInvalid to.
  refuses(
      "delta_0 above the closure window",
      [](Cfg& c) { c.params.core.delta_0 = c.params.core.delta_hi + 0.1; },
      MpcDockingReasonName(MpcDockingReason::kTimingWindowInvalid));
  refuses(
      "delta_0 below the closure window",
      [](Cfg& c) { c.params.core.delta_0 = c.params.core.delta_lo - 0.1; },
      MpcDockingReasonName(MpcDockingReason::kTimingWindowInvalid));

  // A Configure that fails on a configured planner leaves it unconfigured.
  MpcDockingSegmentPlanner planner;
  std::string err;
  SetClockStep(1000);
  ASSERT_TRUE(planner.Configure(rig.PlannerModel(), rig.PlannerConstants(), rig.PlannerParams(),
                                &StepClock, &err))
      << err;
  EXPECT_TRUE(planner.Configured());
  EXPECT_FALSE(planner.Configure(rig.PlannerModel(), rig.PlannerConstants(), rig.PlannerParams(),
                                 nullptr, &err));
  EXPECT_FALSE(planner.Configured());
  EXPECT_EQ(planner.Core(1), nullptr);
}

// ── 2. The first segment: the search's solution, adopted ─────────────────────

TEST(MpcDockingPlannerFirstAdopted, PublishesTheSearchsNodesBitForBit) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  const SegmentSnapshot& seg = s->solution.seg;
  auto r = RunFirst(*s->planner, s->rt, s->plan, s->Ball(), &s->solution, kNow);
  ASSERT_TRUE(r->ok) << Describe(r->rec);
  const SegmentSnapshot& out = r->out;
  EXPECT_EQ(r->rec.outcome, SegmentOutcome::kReady) << Describe(r->rec);
  EXPECT_EQ(r->rec.kind, SegmentKind::kFirst);
  EXPECT_TRUE(r->rec.from_search);
  EXPECT_EQ(r->rec.source_seq, s->solution.source_seq);
  EXPECT_EQ(r->rec.k, -seg.n_pre);
  EXPECT_EQ(r->rec.n_nodes, seg.n_nodes);
  // The cycle's to fill: the plan's id, no stamp, no seq.
  EXPECT_EQ(seg.plan_id, 0U);
  EXPECT_EQ(out.plan_id, s->plan.plan_id);
  EXPECT_EQ(out.publish_ns, 0);
  EXPECT_EQ(out.segment_seq, 0U);
  // Everything else is the solution's, to the bit.
  EXPECT_EQ(FirstDifference(out.q, seg.q), -1);
  EXPECT_EQ(FirstDifference(out.qd, seg.qd), -1);
  EXPECT_EQ(FirstDifference(out.qdd, seg.qdd), -1);
  EXPECT_EQ(out.t_c_ns, seg.t_c_ns);
  EXPECT_EQ(out.t0_ns, seg.t0_ns);
  EXPECT_EQ(out.n_pre, seg.n_pre);
  EXPECT_EQ(out.n_nodes, seg.n_nodes);
  EXPECT_EQ(out.dt_ns, seg.dt_ns);
  EXPECT_EQ(out.dt_pre_ns, seg.dt_pre_ns);
  EXPECT_EQ(out.nv, seg.nv);
  EXPECT_EQ(out.rt_iteration, seg.rt_iteration);
  EXPECT_EQ(out.rt_state_ns, seg.rt_state_ns);
  EXPECT_EQ(out.token.activation_generation, seg.token.activation_generation);
  EXPECT_EQ(out.token.generation, seg.token.generation);
  EXPECT_EQ(out.token.snapshot_sequence, seg.token.snapshot_sequence);
  EXPECT_EQ(out.token.traj_recv_ns, seg.token.traj_recv_ns);
  EXPECT_TRUE(ValidateSegmentNodes(out));
  // Nothing was published by asking: the ring is the cycle's NotePublished.
  std::uint64_t generation = 0;
  EXPECT_FALSE(s->planner->FollowedTrack(s->rig.FollowingRt(kNow, out.t_c_ns), generation));
}

TEST(MpcDockingPlannerFirstAdopted, ConvergenceIsTheSearchsStatementNotTheEvaluations) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  CatchSolution sol = s->solution;
  sol.converged = false;
  auto r = RunFirst(*s->planner, s->rt, s->plan, s->Ball(), &sol, kNow);
  EXPECT_FALSE(r->ok);
  EXPECT_EQ(r->rec.outcome, SegmentOutcome::kSolveFailed) << Describe(r->rec);
  // Nothing was solved instead.
  EXPECT_TRUE(r->rec.from_search);
  EXPECT_FALSE(r->out.valid);
}

TEST(MpcDockingPlannerFirstAdopted, ASolutionThatIsNotThisPlansIsNotAdoptedTheCoreSolvesItself) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  // Each case is a solution the planner must not republish; the call is then
  // the one of `solution == nullptr` (the same plan, report and ball), which
  // the next section shows publishes.
  const char* const names[] = {"no solution", "another catch instant", "another RT report",
                               "a catch interval of its own length", "another pre-catch spacing"};
  for (int i = 0; i < 5; ++i) {
    SCOPED_TRACE(names[i]);
    CatchSolution sol = s->solution;
    const CatchSolution* use = &sol;
    switch (i) {
      case 0:
        use = nullptr;
        break;
      case 1:
        sol.seg.t_c_ns += kMs;
        break;
      case 2:
        sol.seg.rt_iteration += 1;
        break;
      case 3:
        sol.seg.dt_catch_ns = 10 * kMs;
        break;
      case 4:
        sol.seg.dt_pre_ns += kMs;
        break;
      default:
        break;
    }
    auto r = RunFirst(*s->planner, s->rt, s->plan, s->Ball(), use, kNow);
    EXPECT_FALSE(r->rec.from_search);
    // The same input as the solve from rest of the next section.
    ASSERT_TRUE(r->ok) << Describe(r->rec);
    EXPECT_EQ(r->out.t_c_ns, s->plan.t_c_ns);
    EXPECT_EQ(r->out.plan_id, s->plan.plan_id);
    EXPECT_TRUE(ValidateSegmentNodes(r->out));
  }
}

TEST(MpcDockingPlannerFirstAdopted, AnEvaluationPastTheBudgetIsWithheldAsBudget) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  // A 40 ms clock step: the call's end is at least one step after its start,
  // past the 35 ms first-solve budget, whatever the core reads in between.
  auto r = RunFirst(*s->planner, s->rt, s->plan, s->Ball(), &s->solution, kNow, 40 * kMs);
  EXPECT_FALSE(r->ok);
  EXPECT_EQ(r->rec.outcome, SegmentOutcome::kBudget) << Describe(r->rec);
  EXPECT_GT(r->rec.solve_ns, Ns(0.035));
  EXPECT_TRUE(r->rec.from_search);
}

TEST(MpcDockingPlannerFirstAdopted, TheBetweenNodeSpeedIsHeldToTheRatingNotTheCoresBox) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  auto good = RunFirst(*s->planner, s->rt, s->plan, s->Ball(), &s->solution, kNow);
  ASSERT_TRUE(good->ok) << Describe(good->rec);
  EXPECT_LE(good->rec.speed_ratio_max, 1.0);
  EXPECT_GT(good->rec.speed_ratio_max, 0.0);
  // The same cores and the same call on a planner that rates every joint at
  // 0.1 % of its velocity box: the solution's motion is far over that.
  std::string err;
  auto slow = MakeMpcDockingSegmentPlanner(s->rig.PlannerModel(1e-3), s->rig.PlannerConstants(),
                                           s->rig.PlannerParams(), &StepClock, &err);
  ASSERT_NE(slow, nullptr) << err;
  auto r = RunFirst(*slow, s->rt, s->plan, s->Ball(), &s->solution, kNow);
  EXPECT_FALSE(r->ok);
  EXPECT_EQ(r->rec.outcome, SegmentOutcome::kSpeed) << Describe(r->rec);
  EXPECT_GT(r->rec.speed_ratio_max, 1.0);
  // The rating is 2000 times smaller than the first planner's: the ratio is
  // far above the first one's (a loose bound — both read the same nodes).
  EXPECT_GT(r->rec.speed_ratio_max, 100.0 * good->rec.speed_ratio_max);
  EXPECT_FALSE(r->out.valid);
}

// ── 3. The first segment, solved from rest ───────────────────────────────────

TEST(MpcDockingPlannerFirstSolved, SolvesFromTheCommandToTheCatchPoseOnTheGridItsBudgetLeaves) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  auto r = RunFirst(*s->planner, s->rt, s->plan, s->Ball(), nullptr, kNow);
  ASSERT_TRUE(r->ok) << Describe(r->rec);
  const SegmentSnapshot& out = r->out;
  EXPECT_EQ(r->rec.outcome, SegmentOutcome::kReady) << Describe(r->rec);
  EXPECT_EQ(r->rec.kind, SegmentKind::kFirst);
  EXPECT_FALSE(r->rec.from_search);
  EXPECT_EQ(r->rec.x0_speed, 0.0);
  EXPECT_FALSE(r->rec.x0_clamped);
  EXPECT_EQ(out.t_c_ns, s->plan.t_c_ns);
  EXPECT_EQ(out.plan_id, s->plan.plan_id);
  EXPECT_EQ(out.token.generation, s->plan.token.generation);
  EXPECT_EQ(out.token.activation_generation, s->rt.activation_generation);
  EXPECT_EQ(out.rt_iteration, s->rt.rt_iteration);
  EXPECT_EQ(out.rt_state_ns, s->rt.rt_state_ns);
  EXPECT_EQ(out.publish_ns, 0);
  EXPECT_EQ(out.segment_seq, 0U);
  // The number of pre-catch intervals the header's rule gives: the call's first
  // read is kNow, T_arm is 0, and the lead is the first budget and two ticks.
  const std::int64_t earliest = kNow + Ns(0.035) + 2 * Ns(0.002);
  const std::int64_t expected_n_pre =
      std::min<std::int64_t>(s->rig.params.n_pre_max, (s->plan.t_c_ns - earliest) / kDtPreNs);
  ASSERT_GE(expected_n_pre, 1);
  EXPECT_EQ(out.n_pre, expected_n_pre);
  EXPECT_GE(out.n_pre, 1);
  EXPECT_LE(out.n_pre, s->rig.params.n_pre_max);
  EXPECT_EQ(out.t0_ns, out.t_c_ns - out.n_pre * kDtPreNs);
  EXPECT_EQ(out.n_nodes, out.n_pre + s->rig.params.n_stop);
  EXPECT_EQ(out.dt_pre_ns, kDtPreNs);
  EXPECT_EQ(out.dt_ns, 50 * kMs);
  EXPECT_EQ(out.dt_catch_ns, 0);
  EXPECT_EQ(out.nv, 6);
  EXPECT_EQ(r->rec.k, -out.n_pre);
  EXPECT_EQ(r->rec.n_nodes, out.n_nodes);
  // The RT can still read node 0 when the call ends (the clock stands at the
  // call's last read).
  EXPECT_TRUE(s->planner->StartsInTime(g_clock.load(), out.t0_ns));
  // Node 0 is the command, in DEVICE order, at rest.
  for (int d = 0; d < 6; ++d) {
    const auto i = static_cast<std::size_t>(d);
    EXPECT_NEAR(out.q[i], s->rt.q_cmd[i], 1e-9) << d;
    EXPECT_NEAR(out.qd[i], 0.0, 1e-9) << d;
    EXPECT_NEAR(out.qdd[i], 0.0, 1e-9) << d;
  }
  EXPECT_TRUE(ValidateSegmentNodes(out));
}

TEST(MpcDockingPlannerFirstSolved, RefusesWhatItCannotStartFromWithTheNamedOutcome) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  MpcDockingSegmentPlanner& planner = *s->planner;
  const double rest_tol = planner.Params().rest_tol;
  const auto refused = [&](const char* what, const PlannerRtState& rt, const PlanSnapshot& plan,
                           const BallPrediction& ball, SegmentOutcome want,
                           std::int64_t now = kNow) {
    SCOPED_TRACE(what);
    auto r = RunFirst(planner, rt, plan, ball, nullptr, now);
    EXPECT_FALSE(r->ok);
    EXPECT_EQ(r->rec.outcome, want) << Describe(r->rec);
    // No segment was written, and none claimed.
    EXPECT_FALSE(r->out.valid);
    std::uint64_t generation = 0;
    EXPECT_FALSE(planner.FollowedTrack(s->rig.FollowingRt(now, plan.t_c_ns), generation));
    return r;
  };

  // Not at rest: the speed is recorded, by magnitude.
  {
    PlannerRtState rt = s->rt;
    rt.qd_cmd[0] = 2 * rest_tol;
    auto r = refused("moving", rt, s->plan, s->Ball(), SegmentOutcome::kNotAtRest);
    EXPECT_DOUBLE_EQ(r->rec.x0_speed, 2 * rest_tol);
    rt.qd_cmd[0] = 0.0;
    rt.qd_cmd[3] = -2 * rest_tol;
    r = refused("moving backwards", rt, s->plan, s->Ball(), SegmentOutcome::kNotAtRest);
    EXPECT_DOUBLE_EQ(r->rec.x0_speed, 2 * rest_tol);
    // Exactly the tolerance is at rest.
    rt.qd_cmd[3] = rest_tol;
    auto at = RunFirst(planner, rt, s->plan, s->Ball(), nullptr, kNow);
    EXPECT_NE(at->rec.outcome, SegmentOutcome::kNotAtRest) << Describe(at->rec);
    // A speed that is not a number is not "at rest" (a max would drop it).
    rt = s->rt;
    rt.qd_cmd[1] = std::numeric_limits<double>::quiet_NaN();
    r = refused("a NaN commanded speed", rt, s->plan, s->Ball(), SegmentOutcome::kNotAtRest);
    EXPECT_TRUE(std::isnan(r->rec.x0_speed));
  }
  // No ball.
  refused("empty view", s->rt, s->plan, BallPrediction{}, SegmentOutcome::kNoBall);
  {
    TrajectorySnapshot gone = s->ball.traj;
    gone.valid = false;
    refused("invalid trajectory", s->rt, s->plan, BallPrediction{&gone, &s->ball.cov, true},
            SegmentOutcome::kNoBall);
  }
  // A stale (or future) report of the RT.
  {
    PlannerRtState rt = s->rt;
    rt.rt_state_ns = kNow - kMpcDockingMaxRtStateAgeNs - kMs;
    refused("stale", rt, s->plan, s->Ball(), SegmentOutcome::kStaleState);
    rt.rt_state_ns = kNow + 1;
    refused("from the future", rt, s->plan, s->Ball(), SegmentOutcome::kStaleState);
    // Exactly the limit is not stale.
    rt.rt_state_ns = kNow - kMpcDockingMaxRtStateAgeNs;
    auto at = RunFirst(planner, rt, s->plan, s->Ball(), nullptr, kNow);
    EXPECT_NE(at->rec.outcome, SegmentOutcome::kStaleState) << Describe(at->rec);
  }
  // No usable state.
  {
    PlannerRtState rt = s->rt;
    rt.valid = false;
    refused("rt invalid", rt, s->plan, s->Ball(), SegmentOutcome::kNoState);
    rt = s->rt;
    rt.nv = 5;
    refused("another joint count", rt, s->plan, s->Ball(), SegmentOutcome::kNoState);
    PlanSnapshot plan = s->plan;
    plan.valid = false;
    refused("plan invalid", s->rt, plan, s->Ball(), SegmentOutcome::kNoState);
    plan = s->plan;
    plan.nv = 5;
    refused("plan of another joint count", s->rt, plan, s->Ball(), SegmentOutcome::kNoState);
  }
  // A catch instant that leaves less than one pre-catch interval after the
  // lead (T_arm + first budget + two ticks): both sides of the boundary.
  {
    const std::int64_t earliest = kNow + Ns(0.035) + 2 * Ns(0.002);
    PlanSnapshot plan = s->plan;
    plan.t_c_ns = kNow + kDtPreNs / 2;
    refused("half an interval away", s->rt, plan, s->Ball(), SegmentOutcome::kTooLate);
    plan.t_c_ns = earliest + kDtPreNs - 1;
    refused("one nanosecond short of an interval", s->rt, plan, s->Ball(),
            SegmentOutcome::kTooLate);
    plan.t_c_ns = earliest + kDtPreNs;
    auto exact = RunFirst(planner, s->rt, plan, s->Ball(), nullptr, kNow);
    EXPECT_NE(exact->rec.outcome, SegmentOutcome::kTooLate) << Describe(exact->rec);
  }
}

// Before it takes a plan the RT has seeded no command: it reports the measured
// pose at zero velocity, which is where it seeds the command when it takes the
// pair. A first segment has to be planned from that report — a planner that
// asked for a seeded command would never publish the first pair, and the RT
// would never have a plan to seed on.
TEST(MpcDockingPlannerFirstSolved, AFirstSegmentIsPlannedBeforeTheRtSeededItsCommand) {
  auto seeded = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(seeded->Setup());
  auto base = RunFirst(*seeded->planner, seeded->rt, seeded->plan, seeded->Ball(), nullptr, kNow);
  ASSERT_TRUE(base->ok) << Describe(base->rec);

  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  PlannerRtState rt = s->rt;
  rt.cmd_seeded = false;
  auto r = RunFirst(*s->planner, rt, s->plan, s->Ball(), nullptr, kNow);
  ASSERT_TRUE(r->ok) << Describe(r->rec);
  // The same report but for the flag: the same segment, bit for bit.
  ASSERT_EQ(r->out.n_nodes, base->out.n_nodes);
  EXPECT_EQ(r->out.t0_ns, base->out.t0_ns);
  EXPECT_EQ(r->out.q, base->out.q);
  EXPECT_EQ(r->out.qd, base->out.qd);
  EXPECT_EQ(r->out.qdd, base->out.qdd);
}

TEST(MpcDockingPlannerFirstSolved, ASolveThatTakesLongerThanItsBudgetIsWithheldAsBudget) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  // The control: the same call on a 1 µs step is inside the 35 ms budget.
  auto fast = RunFirst(*s->planner, s->rt, s->plan, s->Ball(), nullptr, kNow);
  ASSERT_TRUE(fast->ok) << Describe(fast->rec);
  EXPECT_LT(fast->rec.solve_ns, Ns(0.035));
  // With a 40 ms step the call ends at least one step after it started.
  auto r = RunFirst(*s->planner, s->rt, s->plan, s->Ball(), nullptr, kNow, 40 * kMs);
  EXPECT_FALSE(r->ok);
  EXPECT_EQ(r->rec.outcome, SegmentOutcome::kBudget) << Describe(r->rec);
  EXPECT_GT(r->rec.solve_ns, Ns(0.035));
  EXPECT_FALSE(r->out.valid);
}

// The record's docking block is the core's account of the solve, for EVERY
// solve that reached an iterate — the one that is published and the one that is
// cut at its deadline alike, slacks included: a solve that did not make it is
// the one whose account is wanted.
TEST(MpcDockingPlannerFirstSolved, EverySolveThatReachedAnIterateLeavesTheCoresAccount) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  auto fast = RunFirst(*s->planner, s->rt, s->plan, s->Ball(), nullptr, kNow);
  ASSERT_TRUE(fast->ok) << Describe(fast->rec);
  const rtc::catching::DockingSolveStats& d = fast->rec.docking;
  EXPECT_TRUE(d.ran);
  EXPECT_GE(d.qp_solves, fast->rec.iterations);
  EXPECT_GT(fast->rec.iterations, 0);
  EXPECT_GE(d.qp_iterations, d.qp_solves);
  EXPECT_GT(d.qp_us, 0.0);
  EXPECT_GT(d.linearize_us, 0.0);
  EXPECT_TRUE(std::isfinite(d.kkt_residual));
  EXPECT_STREQ(d.infeasible_group_name, "none");
  for (const double v : d.violation) {
    EXPECT_LE(v, s->rig.params.core.tol_violation);
  }
  EXPECT_GT(d.c_catch, 0.0);
  EXPECT_TRUE(std::isfinite(d.lateral_margin));
  EXPECT_GE(d.lateral_margin, -s->rig.params.core.tol_violation);
  EXPECT_TRUE(std::isfinite(d.cost_reference));
  EXPECT_TRUE(std::isfinite(fast->rec.slack_max));
  EXPECT_TRUE(std::isfinite(fast->rec.slack_v));

  // Cut at the deadline after two iterations (a 10 ms clock step against the
  // 35 ms budget: the core reads the clock before its initialisation QP and
  // before each iteration's QP, and the fourth of those reads is 40 ms after
  // the call's own): the core hands back its last accepted iterate, and its
  // account with it. The solve above was not published, so the planner would
  // start this one from its solution (MpcDockingPlannerFirstMemory) and it
  // would have nothing left to be cut at: a new trial forgets it.
  ASSERT_GT(fast->rec.iterations, 2);
  s->planner->ResetTrial();
  auto cut = RunFirst(*s->planner, s->rt, s->plan, s->Ball(), nullptr, kNow, 10 * kMs);
  ASSERT_FALSE(cut->ok);
  ASSERT_EQ(cut->rec.outcome, SegmentOutcome::kBudget) << Describe(cut->rec);
  EXPECT_STREQ(cut->rec.core_reason_name, "deadline");
  EXPECT_EQ(cut->rec.iterations, 2);
  EXPECT_TRUE(cut->rec.docking.ran);
  EXPECT_GT(cut->rec.docking.qp_solves, 0);
  EXPECT_TRUE(std::isfinite(cut->rec.slack_max)) << "recorded for a solve that was not published";
  EXPECT_TRUE(std::isfinite(cut->rec.slack_v));
  EXPECT_STREQ(cut->rec.docking.infeasible_group_name, "none");

  // Nothing was solved: the block stays at its default.
  PlannerRtState stale = s->rt;
  stale.valid = false;
  auto none = RunFirst(*s->planner, stale, s->plan, s->Ball(), nullptr, kNow);
  EXPECT_FALSE(none->ok);
  EXPECT_FALSE(none->rec.docking.ran);
}

// Before which QP the core's deadline cut a solve is in the record for every
// docking solve — the one cut before its initialisation QP included, which
// has no iterate and so nothing else in the block.
TEST(MpcDockingPlannerFirstSolved, TheRecordNamesTheQpTheDeadlineKeptFromStarting) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  MpcDockingSegmentPlanner& planner = *s->planner;
  auto adopted = RunFirst(planner, s->rt, s->plan, s->Ball(), &s->solution, kNow);
  ASSERT_TRUE(adopted->ok) << Describe(adopted->rec);
  EXPECT_STREQ(adopted->rec.docking.cut_site_name, "none");
  // An evaluation judges no QP's step.
  EXPECT_TRUE(adopted->rec.docking.ran);
  EXPECT_TRUE(std::isnan(adopted->rec.docking.kkt_residual));
  auto whole = RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow);
  ASSERT_TRUE(whole->ok) << Describe(whole->rec);
  EXPECT_STREQ(whole->rec.docking.cut_site_name, "none");
  EXPECT_TRUE(std::isfinite(whole->rec.docking.kkt_residual));
  // One clock step past the 35 ms budget: the deadline has passed before the
  // initialisation QP.
  planner.ResetTrial();
  auto at_once = RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow, 40 * kMs);
  EXPECT_EQ(at_once->rec.outcome, SegmentOutcome::kBudget) << Describe(at_once->rec);
  EXPECT_STREQ(at_once->rec.core_reason_name, "deadline");
  EXPECT_FALSE(at_once->rec.docking.ran);
  EXPECT_STREQ(at_once->rec.docking.cut_site_name, "init_qp");
  EXPECT_EQ(at_once->rec.docking.qp_solves, 0);
  // … and after two iterations.
  auto cut = RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow, 10 * kMs);
  EXPECT_EQ(cut->rec.outcome, SegmentOutcome::kBudget) << Describe(cut->rec);
  EXPECT_TRUE(cut->rec.docking.ran);
  EXPECT_STREQ(cut->rec.docking.cut_site_name, "iteration");
  // A call that solved nothing names none.
  PlannerRtState stale = s->rt;
  stale.valid = false;
  EXPECT_STREQ(
      RunFirst(planner, stale, s->plan, s->Ball(), nullptr, kNow)->rec.docking.cut_site_name,
      "none");
}

TEST(MpcDockingPlannerFirstSolved, AnUnconfiguredPlannerPlansNothingAndSaysOff) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  MpcDockingSegmentPlanner raw;
  auto r = RunFirst(raw, s->rt, s->plan, s->Ball(), &s->solution, kNow);
  EXPECT_FALSE(r->ok);
  EXPECT_EQ(r->rec.outcome, SegmentOutcome::kOff) << Describe(r->rec);
  EXPECT_EQ(r->rec.kind, SegmentKind::kNone);
  EXPECT_FALSE(r->out.valid);
  auto q = RunReplan(raw, s->rig.FollowingRt(kNow, s->plan.t_c_ns), s->Ball(), kNow);
  EXPECT_FALSE(q->ok);
  EXPECT_EQ(q->rec.outcome, SegmentOutcome::kOff) << Describe(q->rec);
  EXPECT_FALSE(q->out.valid);
}

// ── 4. Replan ────────────────────────────────────────────────────────────────

// A planner that solved and "published" a first segment (seq 1), and the RT
// report of an arm following it.
struct Followed {
  std::unique_ptr<Scene> scene;
  SegmentSnapshot first{};
  int n0{0};
  std::int64_t t_c{0};

  void Setup() {
    scene = std::make_unique<Scene>();
    ASSERT_NO_FATAL_FAILURE(scene->Setup());
    auto r = RunFirst(*scene->planner, scene->rt, scene->plan, scene->Ball(), nullptr, kNow);
    ASSERT_TRUE(r->ok) << Describe(r->rec);
    first = r->out;
    // What the cycle does on publishing.
    first.segment_seq = 1;
    first.publish_ns = kNow + 100'000;
    scene->planner->NotePublished(first);
    n0 = first.n_pre;
    t_c = first.t_c_ns;
    // The catch is 144 ms or more after the first wake's start, the lead is
    // 39 ms: two pre-catch intervals at least, so a later grid point exists.
    ASSERT_GE(n0, 2);
  }

  // The first-read instant at which a replan's grid point is n_pre intervals
  // before the catch.
  [[nodiscard]] std::int64_t NowFor(int n_pre) const {
    return t_c - n_pre * kDtPreNs - kReplanLeadNs;
  }

  // The RT following the first segment, reported at `now`.
  [[nodiscard]] PlannerRtState RtAt(std::int64_t now) const {
    PlannerRtState rt = scene->rig.FollowingRt(now, t_c);
    rt.plan_id = first.plan_id;
    rt.segment_active = true;
    rt.segment_seq = 1;
    rt.segment_pending = false;
    rt.segment_pending_seq = 0;
    return rt;
  }
};

TEST(MpcDockingPlannerReplan,
     StartsFromTheReportedSegmentAtTheNextGridPointAndKeepsTheCatchInstant) {
  Followed f;
  ASSERT_NO_FATAL_FAILURE(f.Setup());
  const int n1 = f.n0 - 1;
  const std::int64_t now = f.NowFor(n1);
  const PlannerRtState rt = f.RtAt(now);
  auto r = RunReplan(*f.scene->planner, rt, f.scene->Ball(), now);
  ASSERT_TRUE(r->ok) << Describe(r->rec);
  const SegmentSnapshot& out = r->out;
  EXPECT_EQ(r->rec.outcome, SegmentOutcome::kReady) << Describe(r->rec);
  EXPECT_TRUE(r->rec.kind == SegmentKind::kAdvance || r->rec.kind == SegmentKind::kSame)
      << Describe(r->rec);
  // The publish gate: the cycle asks SourceSeq again and compares.
  EXPECT_EQ(r->rec.source_seq, 1U);
  EXPECT_EQ(r->rec.source_seq, f.scene->planner->SourceSeq(rt, out.t0_ns));
  EXPECT_TRUE(r->rec.x0_from_segment);
  EXPECT_FALSE(r->rec.from_search);
  // The catch instant does not move; the plan's id and track are the source's.
  EXPECT_EQ(out.t_c_ns, f.first.t_c_ns);
  EXPECT_EQ(out.plan_id, rt.plan_id);
  EXPECT_EQ(out.token.generation, f.first.token.generation);
  EXPECT_EQ(out.token.activation_generation, rt.activation_generation);
  EXPECT_EQ(out.rt_iteration, rt.rt_iteration);
  EXPECT_EQ(out.publish_ns, 0);
  EXPECT_EQ(out.segment_seq, 0U);
  // On the grid, at the latest point the budget reaches.
  EXPECT_GE(out.t0_ns, f.first.t0_ns);
  EXPECT_EQ((out.t_c_ns - out.t0_ns) % kDtPreNs, 0);
  EXPECT_EQ(out.n_pre, (out.t_c_ns - out.t0_ns) / kDtPreNs);
  EXPECT_EQ(out.n_pre, n1);
  EXPECT_EQ(out.t0_ns, f.t_c - n1 * kDtPreNs);
  EXPECT_GT(out.t0_ns, f.first.t0_ns);
  EXPECT_EQ(out.n_nodes, out.n_pre + f.scene->rig.params.n_stop);
  EXPECT_EQ(out.dt_ns, f.first.dt_ns);
  EXPECT_EQ(out.dt_pre_ns, f.first.dt_pre_ns);
  EXPECT_EQ(out.nv, f.first.nv);
  EXPECT_EQ(r->rec.k, -out.n_pre);
  EXPECT_TRUE(ValidateSegmentNodes(out));
  EXPECT_TRUE(f.scene->planner->StartsInTime(g_clock.load(), out.t0_ns));
  // Node 0 is the source segment read at node 0's instant, in device order.
  if (!r->rec.x0_clamped) {
    std::array<double, kMaxSegmentNv> q{};
    std::array<double, kMaxSegmentNv> qd{};
    std::array<double, kMaxSegmentNv> qdd{};
    ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(f.first, out.t0_ns, q, qd, qdd));
    for (int d = 0; d < 6; ++d) {
      const auto i = static_cast<std::size_t>(d);
      EXPECT_DOUBLE_EQ(out.q[i], q[i]) << d;
      EXPECT_DOUBLE_EQ(out.qd[i], qd[i]) << d;
      EXPECT_DOUBLE_EQ(out.qdd[i], qdd[i]) << d;
    }
  }
}

TEST(MpcDockingPlannerReplan, TheInterfaceQueriesAnswerFromWhatWasPublished) {
  Followed f;
  ASSERT_NO_FATAL_FAILURE(f.Setup());
  MpcDockingSegmentPlanner& planner = *f.scene->planner;
  const PlannerRtState rt = f.RtAt(f.NowFor(f.n0 - 1));

  std::uint64_t generation = 0;
  ASSERT_TRUE(planner.FollowedTrack(rt, generation));
  EXPECT_EQ(generation, f.scene->plan.token.generation);
  EXPECT_EQ(planner.SourceSeq(rt, f.first.t0_ns), 1U);
  EXPECT_EQ(planner.SourceSeq(rt, f.first.t0_ns + 1'000 * kMs), 1U);
  ReportedSegments reported;
  reported.has_pending = true;
  planner.Reported(rt, reported);
  EXPECT_TRUE(reported.has_following);
  EXPECT_FALSE(reported.has_pending);
  EXPECT_EQ(reported.following.segment_seq, 1U);
  EXPECT_EQ(reported.following.plan_id, f.first.plan_id);
  EXPECT_EQ(reported.following.t0_ns, f.first.t0_ns);
  EXPECT_EQ(reported.following.publish_ns, f.first.publish_ns);
  EXPECT_EQ(FirstDifference(reported.following.q, f.first.q), -1);
  EXPECT_EQ(FirstDifference(reported.following.qd, f.first.qd), -1);
  EXPECT_EQ(FirstDifference(reported.following.qdd, f.first.qdd), -1);

  // A second published segment, pending: the source from its node 0 on.
  SegmentSnapshot second = f.first;
  second.segment_seq = 2;
  second.publish_ns = kNow + 200'000;
  second.t0_ns = f.first.t0_ns + 2 * kDtPreNs;
  planner.NotePublished(second);
  PlannerRtState with_pending = rt;
  with_pending.segment_pending = true;
  with_pending.segment_pending_seq = 2;
  planner.Reported(with_pending, reported);
  EXPECT_TRUE(reported.has_pending);
  EXPECT_TRUE(reported.has_following);
  EXPECT_EQ(reported.pending.segment_seq, 2U);
  EXPECT_EQ(reported.following.segment_seq, 1U);
  EXPECT_EQ(planner.SourceSeq(with_pending, second.t0_ns - 1), 1U);
  EXPECT_EQ(planner.SourceSeq(with_pending, second.t0_ns), 2U);

  // A report of another plan finds nothing.
  PlannerRtState other = with_pending;
  other.plan_id += 1;
  EXPECT_FALSE(planner.FollowedTrack(other, generation));
  EXPECT_EQ(planner.SourceSeq(other, second.t0_ns), 0U);
  planner.Reported(other, reported);
  EXPECT_FALSE(reported.has_pending);
  EXPECT_FALSE(reported.has_following);

  // A trial reset forgets every segment: nothing is followed any more.
  planner.ResetTrial();
  EXPECT_FALSE(planner.FollowedTrack(rt, generation));
  EXPECT_EQ(planner.SourceSeq(rt, f.first.t0_ns), 0U);
  planner.Reported(rt, reported);
  EXPECT_FALSE(reported.has_pending);
  EXPECT_FALSE(reported.has_following);
  const std::int64_t now = f.NowFor(f.n0 - 1);
  auto r = RunReplan(planner, rt, f.scene->Ball(), now);
  EXPECT_FALSE(r->ok);
  EXPECT_EQ(r->rec.outcome, SegmentOutcome::kNotFollowed) << Describe(r->rec);
}

TEST(MpcDockingPlannerReplan, RefusesWhatItCannotStartFromWithTheNamedOutcome) {
  Followed f;
  ASSERT_NO_FATAL_FAILURE(f.Setup());
  MpcDockingSegmentPlanner& planner = *f.scene->planner;
  const std::int64_t now = f.NowFor(f.n0 - 1);
  const PlannerRtState rt = f.RtAt(now);
  const auto refused = [&](const char* what, const PlannerRtState& report,
                           const BallPrediction& ball, SegmentOutcome want) {
    SCOPED_TRACE(what);
    auto r = RunReplan(planner, report, ball, now);
    EXPECT_FALSE(r->ok);
    EXPECT_EQ(r->rec.outcome, want) << Describe(r->rec);
    EXPECT_FALSE(r->out.valid);
    return r;
  };
  {
    PlannerRtState report = rt;
    report.segment_active = false;
    refused("the RT reports no segment", report, f.scene->Ball(), SegmentOutcome::kNotFollowed);
    report = rt;
    report.segment_seq = 9;  // a seq of ours the ring never held
    refused("the RT reports a seq that is not ours", report, f.scene->Ball(),
            SegmentOutcome::kNotFollowed);
    report = rt;
    report.plan_id += 1;
    refused("the RT follows another plan than the ring's", report, f.scene->Ball(),
            SegmentOutcome::kNotFollowed);
  }
  {
    PlannerRtState report = rt;
    report.plan_active = false;
    refused("no plan followed", report, f.scene->Ball(), SegmentOutcome::kNoState);
    report = rt;
    report.valid = false;
    refused("report invalid", report, f.scene->Ball(), SegmentOutcome::kNoState);
    // A replan starts on the command the RT runs: it has to be seeded.
    report = rt;
    report.cmd_seeded = false;
    refused("command not seeded", report, f.scene->Ball(), SegmentOutcome::kNoState);
    report = rt;
    report.rt_state_ns = now - kMpcDockingMaxRtStateAgeNs - kMs;
    refused("stale report", report, f.scene->Ball(), SegmentOutcome::kStaleState);
  }
  refused("no ball", rt, BallPrediction{}, SegmentOutcome::kNoBall);

  // The grid point of an EARLIER first-read instant is before the source's
  // node 0: the published segment already starts later.
  if (f.n0 < f.scene->rig.params.n_pre_max) {
    const std::int64_t early = f.NowFor(f.n0 + 1);
    auto r = RunReplan(planner, f.RtAt(early), f.scene->Ball(), early);
    EXPECT_FALSE(r->ok);
    EXPECT_EQ(r->rec.outcome, SegmentOutcome::kUpToDate) << Describe(r->rec);
    EXPECT_EQ(r->rec.source_seq, 1U);
  }

  // The grid point that IS the source's node 0: a re-solve of it with
  // `replan.same_point`, up to date without. (The outcome of the re-solve is
  // not what is pinned here: its kind is, and that nothing else decides.)
  const std::int64_t same = f.NowFor(f.n0);
  auto resolved = RunReplan(planner, f.RtAt(same), f.scene->Ball(), same);
  EXPECT_EQ(resolved->rec.kind, SegmentKind::kSame) << Describe(resolved->rec);
  EXPECT_EQ(resolved->rec.source_seq, 1U);
  MpcDockingSegmentPlannerParams params = f.scene->rig.PlannerParams();
  params.replan_same_point = false;
  std::string err;
  auto strict = MakeMpcDockingSegmentPlanner(
      f.scene->rig.PlannerModel(), f.scene->rig.PlannerConstants(), params, &StepClock, &err);
  ASSERT_NE(strict, nullptr) << err;
  strict->NotePublished(f.first);
  auto same_off = RunReplan(*strict, f.RtAt(same), f.scene->Ball(), same);
  EXPECT_FALSE(same_off->ok);
  EXPECT_EQ(same_off->rec.outcome, SegmentOutcome::kUpToDate) << Describe(same_off->rec);
  EXPECT_EQ(same_off->rec.source_seq, 1U);
  EXPECT_FALSE(same_off->out.valid);
  // The same planner is not "up to date" at a later grid point.
  auto later = RunReplan(*strict, f.RtAt(f.NowFor(f.n0 - 1)), f.scene->Ball(), f.NowFor(f.n0 - 1));
  EXPECT_NE(later->rec.outcome, SegmentOutcome::kUpToDate) << Describe(later->rec);
}

TEST(MpcDockingPlannerReplan, NothingIsPublishedAfterTheCatchOrWithinOneIntervalOfIt) {
  Followed f;
  ASSERT_NO_FATAL_FAILURE(f.Setup());
  MpcDockingSegmentPlanner& planner = *f.scene->planner;
  // A replan reaches a grid point with at least one pre-catch interval when
  // t_c − now − (T_arm + replan budget + 2 ticks) >= Δ_pre.
  const std::int64_t last_reachable = f.t_c - kDtPreNs - kReplanBudgetNs - 4 * kMs;
  const auto past = [&](const char* what, std::int64_t now, const PlannerRtState& rt,
                        const BallPrediction& ball) {
    SCOPED_TRACE(what);
    auto r = RunReplan(planner, rt, ball, now);
    EXPECT_FALSE(r->ok);
    EXPECT_EQ(r->rec.outcome, SegmentOutcome::kPastReplanWindow) << Describe(r->rec);
    EXPECT_FALSE(r->out.valid);
  };
  // The boundary: one nanosecond inside the window and one past it.
  {
    auto at = RunReplan(planner, f.RtAt(last_reachable), f.scene->Ball(), last_reachable);
    EXPECT_NE(at->rec.outcome, SegmentOutcome::kPastReplanWindow) << Describe(at->rec);
  }
  past("one nanosecond past the window", last_reachable + 1, f.RtAt(last_reachable + 1),
       f.scene->Ball());
  // Inside the last interval, at the catch, and after it, whatever the RT
  // reports and whether or not there is a ball.
  for (const std::int64_t now :
       {f.t_c - 60 * kMs, f.t_c - kMs, f.t_c, f.t_c + 20 * kMs, f.t_c + 1'000 * kMs}) {
    SCOPED_TRACE(now - f.t_c);
    past("following", now, f.RtAt(now), f.scene->Ball());
    past("no ball", now, f.RtAt(now), BallPrediction{});
    PlannerRtState none = f.RtAt(now);
    none.segment_active = false;
    past("nothing reported", now, none, f.scene->Ball());
    PlannerRtState pending = f.RtAt(now);
    pending.segment_pending = true;
    pending.segment_pending_seq = 5;
    past("a pending segment not ours", now, pending, f.scene->Ball());
  }
}

// ── 4a. The first segment of a REPLACEMENT (E1-F17 #743) ─────────────────────

// A followed plan (Followed: seq 1) and the search's replacement of it: the
// same throw 30 ms later, to be caught 30 ms later — the first problem shifted
// in time, so it is as solvable as the first one, but the arm is on seq 1 by
// then. The new grid is anchored at the new catch instant: its points fall
// BETWEEN the nodes of seq 1.
struct Replaced {
  Followed f;
  Throw ball;           // the replacement's prediction
  PlanSnapshot plan;    // the replacement
  int n1{0};            // its pre-catch intervals at `now`
  std::int64_t now{0};  // the wake's first clock read
  std::int64_t t0{0};   // node 0 of its first segment

  void Setup() {
    ASSERT_NO_FATAL_FAILURE(f.Setup());
    constexpr std::int64_t kShiftNs = 30 * kMs;
    ball = AxisThrow(f.scene->rig, 0.28 + 0.03);
    plan = f.scene->plan;
    plan.plan_id = kPlanId + 1;
    plan.t_c_ns = f.t_c + kShiftNs;
    // One interval fewer than the followed segment has: node 0 is 80 ms after
    // that segment's. The wake is the first-solve lead (35 ms + 2 × 2 ms)
    // and a 10 ms margin before it — after the followed segment's node 0, so
    // the RT is on it.
    n1 = f.n0 - 1;
    t0 = plan.t_c_ns - n1 * kDtPreNs;
    now = t0 - 39 * kMs - 10 * kMs;
    ASSERT_EQ(t0, f.first.t0_ns + 80 * kMs);
    ASSERT_GT(now, f.first.t0_ns);
  }

  [[nodiscard]] BallPrediction Ball() const { return BallPrediction{&ball.traj, &ball.cov, true}; }

  // The RT following seq 1 of the old plan, reported at the wake.
  [[nodiscard]] PlannerRtState Rt() const { return f.RtAt(now); }

  // The same RT holding the replacement, whose first segment `seq` waits.
  [[nodiscard]] PlannerRtState RtHolding(std::uint32_t seq) const {
    PlannerRtState rt = Rt();
    rt.segment_pending = true;
    rt.segment_pending_seq = seq;
    rt.plan_pending = true;
    rt.plan_pending_id = plan.plan_id;
    rt.plan_pending_t_c_ns = plan.t_c_ns;
    return rt;
  }
};

// The start state the core for `n_pre` was last handed is `seg` read at
// `t_ns` by the RT's evaluator (device → model order), to the bit: q̈ always,
// q and q̇ when the start was not projected into the box.
void ExpectStartOnSegment(const MpcDockingSegmentPlanner& planner, int n_pre,
                          const SegmentSnapshot& seg, std::int64_t t_ns, bool clamped) {
  std::array<double, kMaxSegmentNv> q{};
  std::array<double, kMaxSegmentNv> qd{};
  std::array<double, kMaxSegmentNv> qdd{};
  ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(seg, t_ns, q, qd, qdd));
  const MpcDockingSegmentCoreInput* in = planner.LastInputForTesting(n_pre);
  ASSERT_NE(in, nullptr);
  ASSERT_EQ(in->q0.size(), 6);
  for (std::size_t m = 0; m < 6; ++m) {
    const auto d = static_cast<std::size_t>(kDeviceOfModel[m]);
    const auto e = static_cast<Eigen::Index>(m);
    if (!clamped) {
      EXPECT_TRUE(BitsEqual(in->q0[e], q[d])) << "q, model joint " << m;
      EXPECT_TRUE(BitsEqual(in->qd0[e], qd[d])) << "q̇, model joint " << m;
    }
    EXPECT_TRUE(BitsEqual(in->qdd0[e], qdd[d])) << "q̈, model joint " << m;
  }
}

TEST(MpcDockingPlannerReplacement, StartsOnTheReportedSegmentAtNodeZeroOfTheNewPlansGrid) {
  Replaced x;
  ASSERT_NO_FATAL_FAILURE(x.Setup());
  MpcDockingSegmentPlanner& planner = *x.f.scene->planner;
  const PlannerRtState rt = x.Rt();
  auto r = RunFirst(planner, rt, x.plan, x.Ball(), nullptr, x.now);

  // Before the solve: the new plan's grid, the source, and the start the core
  // was handed.
  EXPECT_EQ(r->rec.kind, SegmentKind::kFirst);
  EXPECT_FALSE(r->rec.from_search);
  EXPECT_NE(r->rec.outcome, SegmentOutcome::kNotAtRest) << Describe(r->rec);
  ASSERT_EQ(r->rec.k, -x.n1) << Describe(r->rec);
  ASSERT_TRUE(r->rec.x0_from_segment) << Describe(r->rec);
  EXPECT_EQ(r->rec.source_seq, 1U);
  EXPECT_EQ(r->rec.source_seq, planner.SourceSeq(rt, x.t0)) << "the cycle's publish gate";
  EXPECT_FALSE(r->rec.x0_clamped) << "the q and q̇ comparisons below are skipped";
  ASSERT_NO_FATAL_FAILURE(ExpectStartOnSegment(planner, x.n1, x.f.first, x.t0, r->rec.x0_clamped));
  const MpcDockingSegmentCoreInput* in = planner.LastInputForTesting(x.n1);
  ASSERT_NE(in, nullptr);
  // The same cold path as a solve from rest: toward the plan's catch pose.
  EXPECT_FALSE(in->initial_valid);
  EXPECT_TRUE(in->catch_target_valid);
  std::array<double, kMaxSegmentNv> q{};
  std::array<double, kMaxSegmentNv> qd{};
  std::array<double, kMaxSegmentNv> qdd{};
  ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(x.f.first, x.t0, q, qd, qdd));
  double speed = 0.0;
  for (std::size_t m = 0; m < 6; ++m) {
    const auto d = static_cast<std::size_t>(kDeviceOfModel[m]);
    speed = std::max(speed, std::fabs(qd[d]));
    EXPECT_TRUE(BitsEqual(in->q_catch_target[static_cast<Eigen::Index>(m)], x.plan.q_star[d])) << m;
  }
  EXPECT_EQ(r->rec.x0_speed, speed);

  // The solve, and the segment it leaves: the NEW plan's, on its grid.
  ASSERT_TRUE(r->ok) << Describe(r->rec);
  const SegmentSnapshot& out = r->out;
  EXPECT_EQ(out.plan_id, x.plan.plan_id);
  EXPECT_EQ(out.t_c_ns, x.plan.t_c_ns);
  EXPECT_EQ(out.token.generation, x.plan.token.generation);
  EXPECT_EQ(out.token.activation_generation, rt.activation_generation);
  EXPECT_EQ(out.rt_iteration, rt.rt_iteration);
  EXPECT_EQ(out.rt_state_ns, rt.rt_state_ns);
  EXPECT_EQ(out.n_pre, x.n1);
  EXPECT_EQ(out.t0_ns, x.t0);
  EXPECT_EQ(out.n_nodes, x.n1 + x.f.scene->rig.params.n_stop);
  EXPECT_EQ(out.publish_ns, 0);
  EXPECT_EQ(out.segment_seq, 0U);
  EXPECT_TRUE(ValidateSegmentNodes(out));
  EXPECT_TRUE(planner.StartsInTime(g_clock.load(), out.t0_ns));
  // Node 0 is the source read at node 0's instant, in device order.
  if (!r->rec.x0_clamped) {
    for (int d = 0; d < 6; ++d) {
      const auto i = static_cast<std::size_t>(d);
      EXPECT_DOUBLE_EQ(out.q[i], q[i]) << d;
      EXPECT_DOUBLE_EQ(out.qd[i], qd[i]) << d;
      EXPECT_DOUBLE_EQ(out.qdd[i], qdd[i]) << d;
    }
  }

  // Published, it sits BESIDE the followed plan's segment.
  SegmentSnapshot second = out;
  second.segment_seq = 2;
  second.publish_ns = x.now + 100'000;
  planner.NotePublished(second);
  std::uint64_t generation = 0;
  ASSERT_TRUE(planner.FollowedTrack(rt, generation));
  EXPECT_EQ(generation, x.f.first.token.generation);
  EXPECT_EQ(planner.SourceSeq(rt, x.f.t_c), 1U) << "the old plan's report still has its source";
  // The RT holding the pair: the followed segment is the old plan's, the
  // pending one the replacement's.
  const PlannerRtState held = x.RtHolding(2);
  EXPECT_EQ(planner.SourceSeq(held, x.t0 - 1), 1U);
  EXPECT_EQ(planner.SourceSeq(held, x.t0), 2U);
  auto reported = std::make_unique<ReportedSegments>();
  planner.Reported(held, *reported);
  ASSERT_TRUE(reported->has_following);
  ASSERT_TRUE(reported->has_pending);
  EXPECT_EQ(reported->following.segment_seq, 1U);
  EXPECT_EQ(reported->following.plan_id, x.f.first.plan_id);
  EXPECT_EQ(reported->pending.segment_seq, 2U);
  EXPECT_EQ(reported->pending.plan_id, x.plan.plan_id);
  EXPECT_EQ(reported->pending.t_c_ns, x.plan.t_c_ns);
  // Without the RT saying it holds the replacement, seq 2 is not pending.
  PlannerRtState unheld = held;
  unheld.plan_pending = false;
  EXPECT_EQ(planner.SourceSeq(unheld, x.t0), 1U);
  // After the switch the RT follows the replacement on that segment.
  PlannerRtState switched = x.f.scene->rig.FollowingRt(x.now, x.plan.t_c_ns);
  switched.plan_id = x.plan.plan_id;
  switched.segment_active = true;
  switched.segment_seq = 2;
  EXPECT_EQ(planner.SourceSeq(switched, x.plan.t_c_ns), 2U);
  EXPECT_TRUE(planner.FollowedTrack(switched, generation));
  // Should the RT drop the pair instead, a replan of the OLD plan still finds
  // its source (whatever the solve then makes of it).
  const std::int64_t replan_at = x.f.NowFor(x.f.n0 - 1);
  auto again = RunReplan(planner, x.f.RtAt(replan_at), x.f.scene->Ball(), replan_at);
  EXPECT_NE(again->rec.outcome, SegmentOutcome::kNotFollowed) << Describe(again->rec);
  EXPECT_EQ(again->rec.source_seq, 1U);
}

TEST(MpcDockingPlannerReplacement, RefusesWhatItCannotStartFromWithTheNamedOutcome) {
  Replaced x;
  ASSERT_NO_FATAL_FAILURE(x.Setup());
  MpcDockingSegmentPlanner& planner = *x.f.scene->planner;
  const PlannerRtState rt = x.Rt();
  BallPrediction ball = x.Ball();
  const auto refused = [&](const char* what, const PlannerRtState& report, const PlanSnapshot& plan,
                           SegmentOutcome want) {
    SCOPED_TRACE(what);
    auto r = RunFirst(planner, report, plan, ball, nullptr, x.now);
    EXPECT_FALSE(r->ok);
    EXPECT_EQ(r->rec.outcome, want) << Describe(r->rec);
    EXPECT_EQ(r->rec.kind, SegmentKind::kFirst);
    EXPECT_FALSE(r->out.valid);
    return r;
  };
  // No segment of ours the RT reports: never "it follows a plan, so it must be
  // on its last segment".
  {
    PlannerRtState report = rt;
    report.segment_active = false;
    auto r = refused("the RT reports no segment", report, x.plan, SegmentOutcome::kNotFollowed);
    EXPECT_EQ(r->rec.k, -x.n1) << "the grid point is recorded even when withheld";
    EXPECT_EQ(r->rec.source_seq, 0U);
    EXPECT_FALSE(r->rec.x0_from_segment);
    report = rt;
    report.segment_seq = 9;
    refused("the RT reports a seq that is not ours", report, x.plan, SegmentOutcome::kNotFollowed);
    report = rt;
    report.plan_id += 5;
    refused("the RT follows a plan the ring holds nothing of", report, x.plan,
            SegmentOutcome::kNotFollowed);
    report = rt;
    report.plan_t_c_ns += 1;
    refused("the RT follows another catch instant", report, x.plan, SegmentOutcome::kNotFollowed);
  }
  // A replacement starts on the command the RT runs: it has to be seeded.
  {
    PlannerRtState report = rt;
    report.cmd_seeded = false;
    refused("command not seeded", report, x.plan, SegmentOutcome::kNoState);
  }
  // Not even one pre-catch interval before the new catch.
  {
    PlanSnapshot late = x.plan;
    late.t_c_ns = x.now + 39 * kMs + kDtPreNs - 1;
    refused("one nanosecond short of an interval", rt, late, SegmentOutcome::kTooLate);
  }
  // The rest check is the resting arm's. A command that moves — it does, the
  // arm follows a plan — withholds nothing here: the start is the segment's.
  {
    PlannerRtState report = rt;
    report.qd_cmd[0] = 2 * planner.Params().rest_tol;
    auto r = RunFirst(planner, report, x.plan, ball, nullptr, x.now);
    EXPECT_NE(r->rec.outcome, SegmentOutcome::kNotAtRest) << Describe(r->rec);
    EXPECT_TRUE(r->rec.x0_from_segment);
    ASSERT_NO_FATAL_FAILURE(
        ExpectStartOnSegment(planner, x.n1, x.f.first, x.t0, r->rec.x0_clamped));
    report.qd_cmd[0] = std::numeric_limits<double>::quiet_NaN();
    r = RunFirst(planner, report, x.plan, ball, nullptr, x.now);
    EXPECT_NE(r->rec.outcome, SegmentOutcome::kNotAtRest) << Describe(r->rec);
    EXPECT_TRUE(r->rec.x0_from_segment);
    // With no plan followed the same command is refused as it always was.
    PlannerRtState free_arm = x.f.scene->rig.RestingRt(x.now);
    free_arm.qd_cmd[0] = 2 * planner.Params().rest_tol;
    refused("no plan followed, a moving command", free_arm, x.plan, SegmentOutcome::kNotAtRest);
  }
  // No ball is no ball, followed or not.
  ball = BallPrediction{};
  refused("empty view", rt, x.plan, SegmentOutcome::kNoBall);
}

TEST(MpcDockingPlannerReplacement, TheSourceIsThePendingSegmentFromItsNodeZeroOnElseTheFollowed) {
  // SourceSegmentAt on what the RT reports: a pending segment of the followed
  // plan is the source once the new node 0 has reached its own — one
  // nanosecond decides.
  for (const bool due : {false, true}) {
    SCOPED_TRACE(due ? "pending, node 0 at the new node 0" : "pending, node 0 one ns after it");
    Replaced x;
    ASSERT_NO_FATAL_FAILURE(x.Setup());
    MpcDockingSegmentPlanner& planner = *x.f.scene->planner;
    // A second segment of the followed plan the RT holds pending: seq 1's
    // nodes a little off, so the two read differently everywhere.
    SegmentSnapshot second = x.f.first;
    second.segment_seq = 2;
    second.publish_ns = kNow + 200'000;
    second.t0_ns = due ? x.t0 : x.t0 + 1;
    for (double& v : second.q) {
      v += 0.01;
    }
    planner.NotePublished(second);
    PlannerRtState rt = x.Rt();
    rt.segment_pending = true;
    rt.segment_pending_seq = 2;
    auto r = RunFirst(planner, rt, x.plan, x.Ball(), nullptr, x.now);
    ASSERT_EQ(r->rec.k, -x.n1) << Describe(r->rec);
    ASSERT_TRUE(r->rec.x0_from_segment) << Describe(r->rec);
    EXPECT_EQ(r->rec.source_seq, due ? 2U : 1U);
    EXPECT_EQ(r->rec.source_seq, planner.SourceSeq(rt, x.t0));
    ASSERT_NO_FATAL_FAILURE(
        ExpectStartOnSegment(planner, x.n1, due ? second : x.f.first, x.t0, r->rec.x0_clamped));
  }
}

// A solution of the replacement's problem on this planner's grid, of the RT
// report `rt`, that claims to have started on segment `source_seq`: the
// followed plan's first segment moved onto the new catch instant. Whether it
// would pass the evaluation is not what the tests below ask — they ask which
// PATH the planner takes, and SegmentRecord::from_search says that before
// anything is evaluated.
[[nodiscard]] std::unique_ptr<CatchSolution> ClaimedSolution(const Replaced& x,
                                                             const PlannerRtState& rt,
                                                             std::uint32_t source_seq) {
  auto sol = std::make_unique<CatchSolution>();
  sol->seg = x.f.first;
  sol->seg.plan_id = 0;
  sol->seg.segment_seq = 0;
  sol->seg.publish_ns = 0;
  sol->seg.t_c_ns = x.plan.t_c_ns;
  sol->seg.t0_ns = x.plan.t_c_ns - static_cast<std::int64_t>(sol->seg.n_pre) * kDtPreNs;
  sol->seg.rt_iteration = rt.rt_iteration;
  sol->seg.rt_state_ns = rt.rt_state_ns;
  sol->source_seq = source_seq;
  sol->feasible = true;
  sol->converged = true;
  return sol;
}

TEST(MpcDockingPlannerReplacement,
     ASearchsSolutionIsPublishedOnlyWhenItStartedOnTheReportedSegment) {
  Replaced x;
  ASSERT_NO_FATAL_FAILURE(x.Setup());
  MpcDockingSegmentPlanner& planner = *x.f.scene->planner;
  const PlannerRtState rt = x.Rt();
  // The other three conditions hold for every solution below: this plan's
  // catch instant, this RT report, this planner's grid.
  const auto run = [&](const char* what, const PlannerRtState& report, std::uint32_t claimed) {
    SCOPED_TRACE(what);
    const std::unique_ptr<CatchSolution> sol = ClaimedSolution(x, report, claimed);
    EXPECT_EQ(sol->seg.t_c_ns, x.plan.t_c_ns);
    EXPECT_EQ(sol->seg.rt_iteration, report.rt_iteration);
    return RunFirst(planner, report, x.plan, x.Ball(), sol.get(), x.now);
  };
  const std::int64_t t0_sol = x.plan.t_c_ns - static_cast<std::int64_t>(x.f.n0) * kDtPreNs;
  ASSERT_EQ(planner.SourceSeq(rt, t0_sol), 1U);

  // It started on the segment the RT reports for its node 0: the search's
  // nodes are what the planner takes (re-evaluated, not solved again).
  auto r = run("started on the followed segment", rt, 1);
  EXPECT_TRUE(r->rec.from_search) << Describe(r->rec);
  EXPECT_EQ(r->rec.source_seq, 1U);
  EXPECT_TRUE(r->rec.x0_from_segment);
  EXPECT_EQ(r->rec.k, -x.f.n0);
  EXPECT_NE(r->rec.outcome, SegmentOutcome::kNotAtRest);

  // It started anywhere else: the planner solves from the reported segment
  // itself, on its own grid point.
  r = run("started at rest on the command", rt, 0);
  EXPECT_FALSE(r->rec.from_search) << Describe(r->rec);
  EXPECT_EQ(r->rec.source_seq, 1U);
  EXPECT_TRUE(r->rec.x0_from_segment);
  EXPECT_EQ(r->rec.k, -x.n1);
  r = run("started on a segment the RT does not report", rt, 7);
  EXPECT_FALSE(r->rec.from_search) << Describe(r->rec);
  EXPECT_EQ(r->rec.source_seq, 1U);

  // The RT's report moved since the search read it: a pending segment is due
  // at the solution's node 0, so that is where a first segment starts now.
  SegmentSnapshot second = x.f.first;
  second.segment_seq = 2;
  second.publish_ns = kNow + 200'000;
  second.t0_ns = t0_sol;
  planner.NotePublished(second);
  PlannerRtState moved = rt;
  moved.segment_pending = true;
  moved.segment_pending_seq = 2;
  ASSERT_EQ(planner.SourceSeq(moved, t0_sol), 2U);
  r = run("started on the followed segment, a pending one is due", moved, 1);
  EXPECT_FALSE(r->rec.from_search) << Describe(r->rec);
  EXPECT_EQ(r->rec.source_seq, 2U);
  r = run("started on the pending segment that is due", moved, 2);
  EXPECT_TRUE(r->rec.from_search) << Describe(r->rec);
  EXPECT_EQ(r->rec.source_seq, 2U);

  // An arm that follows a plan starts on a segment: with none reported, a
  // solution that started on none is not published either — nothing is.
  PlannerRtState silent = rt;
  silent.segment_active = false;
  r = run("nothing reported, a solution that started at rest", silent, 0);
  EXPECT_FALSE(r->rec.from_search) << Describe(r->rec);
  EXPECT_FALSE(r->ok);
  EXPECT_EQ(r->rec.outcome, SegmentOutcome::kNotFollowed) << Describe(r->rec);

  // With no plan followed the rule is the one it was: a solution that started
  // at rest is the search's own, one that claims a segment is not.
  const PlannerRtState free_arm = x.f.scene->rig.RestingRt(x.now);
  r = run("no plan followed, started at rest", free_arm, 0);
  EXPECT_TRUE(r->rec.from_search) << Describe(r->rec);
  EXPECT_FALSE(r->rec.x0_from_segment);
  r = run("no plan followed, claims a segment", free_arm, 1);
  EXPECT_FALSE(r->rec.from_search) << Describe(r->rec);
  EXPECT_FALSE(r->rec.x0_from_segment);
}

// ── 5. Allocation ────────────────────────────────────────────────────────────

struct GateLog {
  int solver_calls{0};
  int stages{0};
};

// The gate is armed over the whole call (depth 1). A QP-solving call suspends
// it; inside a core's Solve each stage that is NOT the QP arms it again.
void SolverHook(bool begin, void* user) noexcept {
  auto* log = static_cast<GateLog*>(user);
  if (begin) {
    --rtc::testing::detail::MallocGateDepth();
    ++log->solver_calls;
  } else {
    ++rtc::testing::detail::MallocGateDepth();
  }
}

void StageHook(MpcDockingStage, bool begin, void* user) noexcept {
  auto* log = static_cast<GateLog*>(user);
  if (begin) {
    ++rtc::testing::detail::MallocGateDepth();
    ++log->stages;
  } else {
    --rtc::testing::detail::MallocGateDepth();
  }
}

// Runs `f` with the C-level malloc gate armed on this thread; returns the
// allocations it made (the solver hook steps the gate around the QP solver).
template <typename F>
[[nodiscard]] std::size_t Gated(F&& f) {
  rtc::testing::detail::MallocGateCount() = 0;
  ++rtc::testing::detail::MallocGateDepth();
  f();
  --rtc::testing::detail::MallocGateDepth();
  return rtc::testing::detail::MallocGateCount();
}

TEST(MpcDockingPlannerAllocation, PlanFirstAndReplanAllocateNothingOutsideTheQpSolver) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  // Positive control: the gate counts an allocation made inside a library,
  // through the same wrapper the calls below run in.
  EXPECT_GT(Gated([&] { const pinocchio::Data probe(*s->rig.arm.model); }), 0U)
      << "the malloc gate does not see library allocations";
  {
    const rtc::testing::ScopedMallocGate gate;
    const pinocchio::Data probe(*s->rig.arm.model);
    ASSERT_GT(gate.count(), 0U);
  }

  MpcDockingSegmentPlanner& planner = *s->planner;
  GateLog log;
  planner.SetSolverHookForTesting(&SolverHook, &log);
  planner.SetCoreStageHookForTesting(&StageHook, &log);

  // Everything a call reads and writes is built before the gate.
  auto adopt = std::make_unique<Result>();
  auto solve = std::make_unique<Result>();
  auto replan = std::make_unique<Result>();
  ReportedSegments reported;
  const BallPrediction ball = s->Ball();
  std::size_t counted = 0;

  SetClockAt(kNow);
  counted += Gated([&] {
    adopt->ok = planner.PlanFirst(s->rt, s->plan, ball, &s->solution, adopt->out, adopt->rec);
  });
  ASSERT_EQ(rtc::testing::detail::MallocGateDepth(), 0) << "the hooks are not balanced";
  SetClockAt(kNow);
  counted += Gated([&] {
    solve->ok = planner.PlanFirst(s->rt, s->plan, ball, nullptr, solve->out, solve->rec);
  });
  ASSERT_EQ(rtc::testing::detail::MallocGateDepth(), 0) << "the hooks are not balanced";
  // The gated calls did the work the claim is about.
  ASSERT_TRUE(adopt->ok) << Describe(adopt->rec);
  EXPECT_TRUE(adopt->rec.from_search);
  ASSERT_TRUE(solve->ok) << Describe(solve->rec);
  EXPECT_FALSE(solve->rec.from_search);
  ASSERT_GE(solve->out.n_pre, 2);

  SegmentSnapshot published = solve->out;
  published.segment_seq = 1;
  published.publish_ns = kNow + 100'000;
  counted += Gated([&] { planner.NotePublished(published); });
  const int n1 = published.n_pre - 1;
  const std::int64_t now = published.t_c_ns - n1 * kDtPreNs - kReplanLeadNs;
  PlannerRtState rt = s->rig.FollowingRt(now, published.t_c_ns);
  rt.plan_id = published.plan_id;
  rt.segment_active = true;
  rt.segment_seq = 1;
  SetClockAt(now);
  counted += Gated([&] { replan->ok = planner.Replan(rt, ball, replan->out, replan->rec); });
  ASSERT_EQ(rtc::testing::detail::MallocGateDepth(), 0) << "the hooks are not balanced";
  ASSERT_TRUE(replan->ok) << Describe(replan->rec);

  // The queries and the reset.
  std::uint64_t generation = 0;
  bool followed = false;
  std::uint32_t source = 0;
  counted += Gated([&] {
    followed = planner.FollowedTrack(rt, generation);
    source = planner.SourceSeq(rt, replan->out.t0_ns);
    planner.Reported(rt, reported);
    planner.ResetTrial();
  });
  EXPECT_TRUE(followed);
  EXPECT_EQ(source, 1U);
  EXPECT_TRUE(reported.has_following);

  EXPECT_EQ(counted, 0U) << "PlanFirst, Replan or a query allocated outside the QP solver";
  // The solver and the stages were what the bracket stepped around.
  EXPECT_GE(log.solver_calls, 3);
  EXPECT_GT(log.stages, 3);
}

TEST(MpcDockingPlannerAllocation, AReplacementsFirstSegmentAllocatesNothingOutsideTheQpSolver) {
  // The paths a replacement adds to PlanFirst, under the same gate: the solve
  // from the reported segment, the search's solution taken when it started
  // there, the withhold with nothing reported, and the replacement's segment
  // stored beside the followed plan's.
  Replaced x;
  ASSERT_NO_FATAL_FAILURE(x.Setup());
  MpcDockingSegmentPlanner& planner = *x.f.scene->planner;
  GateLog log;
  planner.SetSolverHookForTesting(&SolverHook, &log);
  planner.SetCoreStageHookForTesting(&StageHook, &log);

  // Everything a call reads and writes is built before the gate.
  auto solve = std::make_unique<Result>();
  auto adopt = std::make_unique<Result>();
  auto silent = std::make_unique<Result>();
  auto reported = std::make_unique<ReportedSegments>();
  const BallPrediction ball = x.Ball();
  const PlannerRtState rt = x.Rt();
  PlannerRtState nothing = rt;
  nothing.segment_active = false;
  const std::unique_ptr<CatchSolution> sol = ClaimedSolution(x, rt, 1);
  std::size_t counted = 0;

  SetClockAt(x.now);
  counted += Gated(
      [&] { solve->ok = planner.PlanFirst(rt, x.plan, ball, nullptr, solve->out, solve->rec); });
  ASSERT_EQ(rtc::testing::detail::MallocGateDepth(), 0) << "the hooks are not balanced";
  SetClockAt(x.now);
  counted += Gated(
      [&] { adopt->ok = planner.PlanFirst(rt, x.plan, ball, sol.get(), adopt->out, adopt->rec); });
  ASSERT_EQ(rtc::testing::detail::MallocGateDepth(), 0) << "the hooks are not balanced";
  SetClockAt(x.now);
  counted += Gated([&] {
    silent->ok = planner.PlanFirst(nothing, x.plan, ball, nullptr, silent->out, silent->rec);
  });
  ASSERT_EQ(rtc::testing::detail::MallocGateDepth(), 0) << "the hooks are not balanced";
  // The gated calls took the paths the claim is about.
  EXPECT_TRUE(solve->rec.x0_from_segment) << Describe(solve->rec);
  EXPECT_FALSE(solve->rec.from_search);
  EXPECT_EQ(solve->rec.source_seq, 1U);
  EXPECT_TRUE(adopt->rec.from_search) << Describe(adopt->rec);
  EXPECT_EQ(silent->rec.outcome, SegmentOutcome::kNotFollowed) << Describe(silent->rec);

  // The replacement's segment beside the followed plan's, and the queries on
  // the report of an RT that holds the pair. (The cycle stores a segment only
  // after the PlanFirst that returned true for it.)
  ASSERT_TRUE(solve->ok) << Describe(solve->rec);
  SegmentSnapshot second = solve->out;
  second.segment_seq = 2;
  second.publish_ns = x.now + 100'000;
  const PlannerRtState held = x.RtHolding(2);
  std::uint64_t generation = 0;
  bool followed = false;
  std::uint32_t source = 0;
  counted += Gated([&] {
    planner.NotePublished(second);
    followed = planner.FollowedTrack(held, generation);
    source = planner.SourceSeq(held, x.t0);
    planner.Reported(held, *reported);
  });
  EXPECT_TRUE(followed);
  EXPECT_EQ(source, 2U);
  EXPECT_TRUE(reported->has_following);
  EXPECT_TRUE(reported->has_pending);

  EXPECT_EQ(counted, 0U) << "a replacement's PlanFirst or a query allocated outside the QP solver";
  // The solver and the stages were what the bracket stepped around.
  EXPECT_GE(log.solver_calls, 2);
  EXPECT_GT(log.stages, 0);
}

// ── 5b. The first solve's memory ─────────────────────────────────────────────
//
// A first solve that is withheld leaves its iterate for the next wake's first
// solve of the same plan. The solve these tests withhold is one the core cuts
// after two iterations: a 10 ms clock step against the 35 ms budget
// (EverySolveThatReachedAnIterateLeavesTheCoresAccount).

constexpr std::int64_t kCutStepNs = 10 * kMs;

// What the planner remembers, copied (the planner's own changes with a call).
[[nodiscard]] std::unique_ptr<SegmentSnapshot> Remembered(const MpcDockingSegmentPlanner& planner) {
  const SegmentSnapshot* mem = planner.FirstMemoryForTesting();
  return mem != nullptr ? std::make_unique<SegmentSnapshot>(*mem) : nullptr;
}

// The start trajectory the core for `n_pre` was last handed is `mem` from the
// node `n_pre` leaves it at (device → model order), to the bit — and node 0
// is the start state the core was handed, whatever `mem` had there.
void ExpectStartFromMemory(const MpcDockingSegmentPlanner& planner, int n_pre,
                           const SegmentSnapshot& mem) {
  const MpcDockingSegmentCoreInput* in = planner.LastInputForTesting(n_pre);
  ASSERT_NE(in, nullptr);
  ASSERT_TRUE(in->initial_valid);
  const int shift = mem.n_pre - n_pre;
  ASSERT_GE(shift, 0);
  const int n_total = n_pre + (mem.n_nodes - mem.n_pre);
  for (int k = 1; k <= n_total; ++k) {
    for (std::size_t m = 0; m < 6; ++m) {
      const auto e = static_cast<std::size_t>((k + shift) * kMaxSegmentNv + kDeviceOfModel[m]);
      const auto row = static_cast<Eigen::Index>(m);
      EXPECT_TRUE(BitsEqual(in->q_init(row, k), mem.q[e])) << "q, node " << k << " joint " << m;
      EXPECT_TRUE(BitsEqual(in->qd_init(row, k), mem.qd[e])) << "q̇, node " << k << " joint " << m;
      EXPECT_TRUE(BitsEqual(in->qdd_init(row, k), mem.qdd[e])) << "q̈, node " << k << " joint " << m;
    }
  }
  for (Eigen::Index m = 0; m < 6; ++m) {
    EXPECT_TRUE(BitsEqual(in->q_init(m, 0), in->q0[m])) << m;
    EXPECT_TRUE(BitsEqual(in->qd_init(m, 0), in->qd0[m])) << m;
    EXPECT_TRUE(BitsEqual(in->qdd_init(m, 0), in->qdd0[m])) << m;
  }
}

TEST(MpcDockingPlannerFirstMemory, TheNextWakeStartsFromTheIterateTheWithheldSolveEndedOn) {
  // The reference: the same first solve run to its end on a planner that
  // remembers nothing.
  auto ref = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(ref->Setup());
  auto whole = RunFirst(*ref->planner, ref->rt, ref->plan, ref->Ball(), nullptr, kNow);
  ASSERT_TRUE(whole->ok) << Describe(whole->rec);
  EXPECT_FALSE(whole->rec.start_from_memory);
  ASSERT_GT(whole->rec.iterations, 2);

  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  MpcDockingSegmentPlanner& planner = *s->planner;
  ASSERT_EQ(planner.FirstMemoryForTesting(), nullptr) << "the warm-up solves leave nothing";
  auto cut = RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow, kCutStepNs);
  ASSERT_FALSE(cut->ok);
  ASSERT_EQ(cut->rec.outcome, SegmentOutcome::kBudget) << Describe(cut->rec);
  ASSERT_EQ(cut->rec.iterations, 2);
  EXPECT_FALSE(cut->rec.start_from_memory);
  const int n_pre = -cut->rec.k;
  // What is remembered is that iterate, packed as a segment of the plan.
  const std::unique_ptr<SegmentSnapshot> mem = Remembered(planner);
  ASSERT_NE(mem, nullptr);
  EXPECT_EQ(mem->plan_id, s->plan.plan_id);
  EXPECT_EQ(mem->t_c_ns, s->plan.t_c_ns);
  EXPECT_EQ(mem->token.generation, s->plan.token.generation);
  EXPECT_EQ(mem->token.activation_generation, s->rt.activation_generation);
  EXPECT_EQ(mem->n_pre, n_pre);
  EXPECT_EQ(mem->n_nodes, n_pre + s->rig.params.n_stop);
  const rtc::catching::MpcDockingSegmentCoreResult* res = planner.LastResult(n_pre);
  ASSERT_NE(res, nullptr);
  for (int k = 0; k <= mem->n_nodes; ++k) {
    for (std::size_t m = 0; m < 6; ++m) {
      const auto e = static_cast<std::size_t>(k * kMaxSegmentNv + kDeviceOfModel[m]);
      ASSERT_TRUE(BitsEqual(mem->q[e], res->q(static_cast<Eigen::Index>(m), k))) << k << " " << m;
    }
  }

  // The next wake, on the same report and plan: from there, with no
  // initialisation QP (the arm has not moved, so the start holds every linear
  // row as it stands), to the solution the uninterrupted solve reached.
  auto next = RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow);
  EXPECT_TRUE(next->rec.start_from_memory);
  ASSERT_NO_FATAL_FAILURE(ExpectStartFromMemory(planner, n_pre, *mem));
  EXPECT_FALSE(planner.LastResult(n_pre)->init_qp_used);
  ASSERT_TRUE(next->ok) << Describe(next->rec);
  EXPECT_LT(next->rec.iterations, whole->rec.iterations);
  EXPECT_EQ(next->out.n_pre, whole->out.n_pre);
  double apart = 0.0;
  for (std::size_t i = 0; i < next->out.q.size(); ++i) {
    apart = std::max(apart, std::fabs(next->out.q[i] - whole->out.q[i]));
  }
  EXPECT_LT(apart, 1e-5) << "another solution than the uninterrupted solve's";
  // Not published, it is still what a further wake would start from: the
  // cycle may drop a pair at its re-check.
  ASSERT_NE(planner.FirstMemoryForTesting(), nullptr);
  auto again = RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow);
  EXPECT_TRUE(again->rec.start_from_memory);
  ASSERT_TRUE(again->ok) << Describe(again->rec);
  EXPECT_LE(again->rec.iterations, 2) << "from a solution there is nothing left to do";
}

TEST(MpcDockingPlannerFirstMemory, ALaterWakeDropsTheNodesThatHavePassed) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  MpcDockingSegmentPlanner& planner = *s->planner;
  auto cut = RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow, kCutStepNs);
  ASSERT_EQ(cut->rec.outcome, SegmentOutcome::kBudget) << Describe(cut->rec);
  const int n_pre = -cut->rec.k;
  ASSERT_GE(n_pre, 2);
  const std::unique_ptr<SegmentSnapshot> mem = Remembered(planner);
  ASSERT_NE(mem, nullptr);
  // One pre-catch interval later the budget reaches one grid point fewer: the
  // same catch, the same nodes from the second on.
  const std::int64_t later = kNow + kDtPreNs;
  auto next = RunFirst(planner, s->rig.RestingRt(later), s->plan, s->Ball(), nullptr, later);
  ASSERT_EQ(next->rec.k, -(n_pre - 1)) << Describe(next->rec);
  EXPECT_TRUE(next->rec.start_from_memory);
  ASSERT_NO_FATAL_FAILURE(ExpectStartFromMemory(planner, n_pre - 1, *mem));
  EXPECT_TRUE(next->rec.docking.ran) << Describe(next->rec);

  // A wake that reaches MORE grid points than the memory has cannot start
  // from it: the nodes before its first are not there.
  planner.ResetTrial();
  cut = RunFirst(planner, s->rig.RestingRt(later), s->plan, s->Ball(), nullptr, later, kCutStepNs);
  ASSERT_EQ(cut->rec.k, -(n_pre - 1)) << Describe(cut->rec);
  ASSERT_NE(planner.FirstMemoryForTesting(), nullptr) << Describe(cut->rec);
  auto earlier = RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow);
  ASSERT_EQ(earlier->rec.k, -n_pre);
  EXPECT_FALSE(earlier->rec.start_from_memory);
  EXPECT_FALSE(planner.LastInputForTesting(n_pre)->initial_valid);
}

TEST(MpcDockingPlannerFirstMemory, AnotherPlanCatchInstantTrackOrActivationStartsFromTheCatchPose) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  MpcDockingSegmentPlanner& planner = *s->planner;

  struct Other {
    const char* what;
    PlannerRtState rt;
    PlanSnapshot plan;
  };

  std::vector<Other> others;
  others.push_back({"another plan id", s->rt, s->plan});
  others.back().plan.plan_id += 1;
  // One nanosecond is another grid: the nodes are not the remembered ones'.
  others.push_back({"another catch instant", s->rt, s->plan});
  others.back().plan.t_c_ns += 1;
  others.push_back({"another track", s->rt, s->plan});
  others.back().plan.token.generation += 1;
  others.push_back({"another activation", s->rt, s->plan});
  others.back().rt.activation_generation += 1;
  for (const Other& o : others) {
    SCOPED_TRACE(o.what);
    planner.ResetTrial();
    auto cut = RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow, kCutStepNs);
    ASSERT_EQ(cut->rec.outcome, SegmentOutcome::kBudget) << Describe(cut->rec);
    ASSERT_NE(planner.FirstMemoryForTesting(), nullptr);
    auto r = RunFirst(planner, o.rt, o.plan, s->Ball(), nullptr, kNow);
    ASSERT_TRUE(r->rec.docking.ran) << Describe(r->rec);
    EXPECT_FALSE(r->rec.start_from_memory);
    EXPECT_FALSE(planner.LastInputForTesting(-r->rec.k)->initial_valid);
    EXPECT_TRUE(planner.LastResult(-r->rec.k)->init_qp_used);
    // … and the control: the very plan the memory is of does start from it.
    planner.ResetTrial();
    cut = RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow, kCutStepNs);
    ASSERT_EQ(cut->rec.outcome, SegmentOutcome::kBudget);
    EXPECT_TRUE(RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow)->rec.start_from_memory);
  }
}

TEST(MpcDockingPlannerFirstMemory, ItEndsWithThePlansPublicationTheTrialAndASolveThatLeftNothing) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  MpcDockingSegmentPlanner& planner = *s->planner;
  const auto remember = [&] {
    planner.ResetTrial();
    auto cut = RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow, kCutStepNs);
    EXPECT_EQ(cut->rec.outcome, SegmentOutcome::kBudget) << Describe(cut->rec);
    EXPECT_NE(planner.FirstMemoryForTesting(), nullptr);
  };
  const auto starts_cold = [&] {
    auto r = RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow);
    return !r->rec.start_from_memory && !planner.LastInputForTesting(-r->rec.k)->initial_valid;
  };

  // A segment of ANOTHER plan published — the followed plan's, while its
  // replacement is being solved for — leaves it.
  remember();
  SegmentSnapshot other{};
  other.valid = true;
  other.plan_id = s->plan.plan_id + 5;
  planner.NotePublished(other);
  EXPECT_NE(planner.FirstMemoryForTesting(), nullptr);
  // The plan's own publication ends it: what follows are replans.
  other.plan_id = s->plan.plan_id;
  planner.NotePublished(other);
  EXPECT_EQ(planner.FirstMemoryForTesting(), nullptr);
  planner.ResetTrial();  // the segment stored above is not a real one
  EXPECT_TRUE(starts_cold());

  remember();
  planner.ResetTrial();
  EXPECT_EQ(planner.FirstMemoryForTesting(), nullptr);
  EXPECT_TRUE(starts_cold());

  // A call the planner refuses before its core runs says nothing of the
  // memory …
  remember();
  PlannerRtState stale = s->rt;
  stale.valid = false;
  EXPECT_FALSE(RunFirst(planner, stale, s->plan, s->Ball(), nullptr, kNow)->ok);
  EXPECT_NE(planner.FirstMemoryForTesting(), nullptr);
  // … one the CORE refuses for anything but its deadline ends it: here a
  // prediction that stops before the catch.
  TrajectorySnapshot brief = s->ball.traj;
  brief.n = 3;  // 100 ms of it
  const BallPrediction short_ball{&brief, &s->ball.cov, true};
  // (A refused solve that did NOT start from it — another catch instant —
  // says nothing of it either.)
  PlanSnapshot elsewhere = s->plan;
  elsewhere.t_c_ns += 1;
  auto unrelated = RunFirst(planner, s->rt, elsewhere, short_ball, nullptr, kNow);
  EXPECT_FALSE(unrelated->ok);
  EXPECT_FALSE(unrelated->rec.start_from_memory);
  EXPECT_STREQ(unrelated->rec.core_reason_name, "ball_invalid") << Describe(unrelated->rec);
  EXPECT_NE(planner.FirstMemoryForTesting(), nullptr);
  auto refused = RunFirst(planner, s->rt, s->plan, short_ball, nullptr, kNow);
  EXPECT_FALSE(refused->ok);
  EXPECT_TRUE(refused->rec.start_from_memory);
  EXPECT_STREQ(refused->rec.core_reason_name, "ball_invalid") << Describe(refused->rec);
  EXPECT_FALSE(refused->rec.docking.ran);
  EXPECT_EQ(planner.FirstMemoryForTesting(), nullptr);
  EXPECT_TRUE(starts_cold());

  // A solve cut again keeps what it stood at: started from the memory, cut
  // before its first QP, the same iterate is still the next wake's start.
  remember();
  const std::unique_ptr<SegmentSnapshot> before = Remembered(planner);
  ASSERT_NE(before, nullptr);
  auto cut_at_once = RunFirst(planner, s->rt, s->plan, s->Ball(), nullptr, kNow, 40 * kMs);
  EXPECT_EQ(cut_at_once->rec.outcome, SegmentOutcome::kBudget) << Describe(cut_at_once->rec);
  EXPECT_TRUE(cut_at_once->rec.start_from_memory);
  EXPECT_EQ(cut_at_once->rec.iterations, 0);
  // An iterate, and no QP's step judged: the last-QP numbers are not numbers.
  EXPECT_TRUE(cut_at_once->rec.docking.ran);
  EXPECT_EQ(cut_at_once->rec.docking.qp_solves, 0);
  EXPECT_TRUE(std::isnan(cut_at_once->rec.docking.kkt_residual));
  EXPECT_TRUE(std::isnan(cut_at_once->rec.docking.grad_norm));
  EXPECT_TRUE(std::isnan(cut_at_once->rec.docking.complementarity));
  for (const double e : cut_at_once->rec.docking.elastic) {
    EXPECT_TRUE(std::isnan(e));
  }
  EXPECT_TRUE(std::isfinite(cut_at_once->rec.docking.cost_reference));
  const std::unique_ptr<SegmentSnapshot> after = Remembered(planner);
  ASSERT_NE(after, nullptr);
  // To rounding: the core rebuilds a start from its block jerk.
  double moved = 0.0;
  for (std::size_t i = 0; i < after->q.size(); ++i) {
    moved = std::max(
        {moved, std::fabs(after->q[i] - before->q[i]), std::fabs(after->qd[i] - before->qd[i])});
  }
  EXPECT_LT(moved, 1e-12);
  RecordProperty("memory_restart_max_abs_change", std::to_string(moved));
  std::printf("[ record ] a memory start handed back: max |change| %.3e\n", moved);
}

TEST(MpcDockingPlannerFirstMemory, AReplacementsFirstSolveIsRememberedBesideTheFollowedPlan) {
  Replaced x;
  ASSERT_NO_FATAL_FAILURE(x.Setup());
  MpcDockingSegmentPlanner& planner = *x.f.scene->planner;
  const PlannerRtState rt = x.Rt();
  // The followed plan's own first solve was published (Followed::Setup).
  ASSERT_EQ(planner.FirstMemoryForTesting(), nullptr);
  auto cut = RunFirst(planner, rt, x.plan, x.Ball(), nullptr, x.now, kCutStepNs);
  ASSERT_EQ(cut->rec.outcome, SegmentOutcome::kBudget) << Describe(cut->rec);
  ASSERT_TRUE(cut->rec.docking.ran) << Describe(cut->rec);
  ASSERT_EQ(cut->rec.k, -x.n1);
  const std::unique_ptr<SegmentSnapshot> mem = Remembered(planner);
  ASSERT_NE(mem, nullptr);
  EXPECT_EQ(mem->plan_id, x.plan.plan_id);
  // A replan of the followed plan goes out meanwhile: another plan's segment.
  SegmentSnapshot replanned = x.f.first;
  replanned.segment_seq = 2;
  planner.NotePublished(replanned);
  ASSERT_NE(planner.FirstMemoryForTesting(), nullptr);
  // The next wake's replacement solve: from the memory, on the reported
  // segment at node 0.
  auto next = RunFirst(planner, rt, x.plan, x.Ball(), nullptr, x.now);
  EXPECT_TRUE(next->rec.start_from_memory) << Describe(next->rec);
  EXPECT_TRUE(next->rec.x0_from_segment);
  ASSERT_NO_FATAL_FAILURE(ExpectStartFromMemory(planner, x.n1, *mem));
  ASSERT_NO_FATAL_FAILURE(
      ExpectStartOnSegment(planner, x.n1, x.f.first, x.t0, next->rec.x0_clamped));
}

TEST(MpcDockingPlannerFirstMemory, AFirstSolveFromItAllocatesNothingOutsideTheQpSolver) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  MpcDockingSegmentPlanner& planner = *s->planner;
  GateLog log;
  planner.SetSolverHookForTesting(&SolverHook, &log);
  planner.SetCoreStageHookForTesting(&StageHook, &log);
  auto cut = std::make_unique<Result>();
  auto next = std::make_unique<Result>();
  const BallPrediction ball = s->Ball();
  std::size_t counted = 0;
  // Remembering it and starting from it, both under the gate.
  SetClockAt(kNow, kCutStepNs);
  counted += Gated(
      [&] { cut->ok = planner.PlanFirst(s->rt, s->plan, ball, nullptr, cut->out, cut->rec); });
  ASSERT_EQ(rtc::testing::detail::MallocGateDepth(), 0) << "the hooks are not balanced";
  SetClockAt(kNow);
  counted += Gated(
      [&] { next->ok = planner.PlanFirst(s->rt, s->plan, ball, nullptr, next->out, next->rec); });
  ASSERT_EQ(rtc::testing::detail::MallocGateDepth(), 0) << "the hooks are not balanced";
  ASSERT_EQ(cut->rec.outcome, SegmentOutcome::kBudget) << Describe(cut->rec);
  ASSERT_TRUE(next->rec.start_from_memory) << Describe(next->rec);
  ASSERT_TRUE(next->ok) << Describe(next->rec);
  EXPECT_EQ(counted, 0U) << "a first solve allocated while remembering or starting from memory";
  EXPECT_GE(log.solver_calls, 2);
}

// ── 6. In the cycle, behind the nlp search ───────────────────────────────────
//
// The real search and the real planner behind PlannerCycle's two interfaces,
// on the scene above: the pair of a first wake, and a wake while the RT
// follows that pair's plan.

struct CycleScene {
  Rig rig;  // the model, limits and parameters (its own search is not the cycle's)
  Throw ball;
  rtc::SeqLock<TrajectorySnapshot> traj{};
  rtc::SeqLock<CovarianceSnapshot> cov{};
  rtc::SeqLock<PlannerRtState> rt{};
  rtc::SeqLock<PlanSnapshot> plan{};
  rtc::SeqLock<SegmentSnapshot> segment{};
  rtc::catching::PlannerCycle cycle;
  NlpCatchSearch* search{nullptr};             // owned by the cycle
  MpcDockingSegmentPlanner* planner{nullptr};  // owned by the cycle

  // gtest ASSERTs: call as ASSERT_NO_FATAL_FAILURE(scene->Setup()).
  void Setup(double t_stop_plan_s = 0.1) {
    std::string err;
    SetClockStep(1000);
    auto s = std::make_unique<NlpCatchSearch>();
    ASSERT_TRUE(s->Configure(rig.model, rig.constants, rig.params, rig.ik, &StepClock, &err))
        << err;
    auto p = MakeMpcDockingSegmentPlanner(rig.PlannerModel(), rig.PlannerConstants(),
                                          rig.PlannerParams(), &StepClock, &err);
    ASSERT_NE(p, nullptr) << err;
    search = s.get();
    planner = p.get();
    rtc::catching::PlannerCycleIo io;
    io.traj = &traj;
    io.cov = &cov;
    io.rt = &rt;
    io.plan = &plan;
    io.segment = &segment;
    ASSERT_TRUE(cycle.Bind(io));
    rtc::catching::PlannerParams params;
    params.t_freeze = 0.05;
    params.t_stop_plan = t_stop_plan_s;
    cycle.Configure(params);
    cycle.SetClock(&StepClock);
    cycle.InstallSearch(std::move(s));
    cycle.InstallSegmentPlanner(std::move(p));
    ball = AxisThrow(rig, 0.28);
    traj.Store(ball.traj);
    cov.Store(ball.cov);
  }

  // One wake at `now`, the clock's first read there.
  [[nodiscard]] rtc::catching::PlannerCycleRecord Wake(std::int64_t now) {
    SetClockAt(now);
    return cycle.Run(NowReal{now});
  }

  // The first wake: the arm at rest, the pair published.
  void PublishPair(rtc::catching::PlannerCycleRecord& rec) {
    rt.Store(rig.RestingRt(kNow));
    rec = Wake(kNow);
    ASSERT_EQ(rec.outcome, rtc::catching::CycleOutcome::kPublished)
        << NlpRejectName(rec.search.nlp.reason) << Describe(rec.segment);
    ASSERT_TRUE(rec.plan_valid);
    ASSERT_EQ(rec.segment.outcome, SegmentOutcome::kPublished) << Describe(rec.segment);
  }

  // The RT following the published pair at `now`.
  [[nodiscard]] PlannerRtState FollowingPair(std::int64_t now) const {
    const PlanSnapshot p = plan.Load();
    const SegmentSnapshot seg = segment.Load();
    PlannerRtState out = rig.FollowingRt(now, p.t_c_ns);
    out.plan_id = p.plan_id;
    out.segment_active = true;
    out.segment_seq = seg.segment_seq;
    return out;
  }
};

TEST(MpcDockingPlannerCycle, TheSearchsSolutionGoesOutAsThePairsSegmentBitForBit) {
  auto s = std::make_unique<CycleScene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  rtc::catching::PlannerCycleRecord rec;
  ASSERT_NO_FATAL_FAILURE(s->PublishPair(rec));
  const PlanSnapshot plan = s->plan.Load();
  const SegmentSnapshot seg = s->segment.Load();
  // A pair: one stamp, the segment naming the plan — the id the cycle gave it,
  // which the planner wrote into the segment.
  EXPECT_TRUE(plan.valid);
  EXPECT_EQ(plan.plan_id, rec.plan_id);
  EXPECT_NE(plan.plan_id, 0U);
  EXPECT_EQ(seg.plan_id, plan.plan_id);
  EXPECT_EQ(seg.publish_ns, plan.publish_ns);
  EXPECT_EQ(seg.t_c_ns, plan.t_c_ns);
  EXPECT_NE(seg.segment_seq, 0U);
  EXPECT_TRUE(ValidateSegmentNodes(seg));
  // Not solved again: the nodes are the search's own, and the record says so.
  EXPECT_TRUE(rec.segment.from_search);
  EXPECT_EQ(rec.segment.kind, SegmentKind::kFirst);
  const CatchSolution* solution = s->search->Solution();
  ASSERT_NE(solution, nullptr);
  EXPECT_TRUE(solution->converged);
  EXPECT_EQ(FirstDifference(seg.q, solution->seg.q), -1);
  EXPECT_EQ(FirstDifference(seg.qd, solution->seg.qd), -1);
  EXPECT_EQ(FirstDifference(seg.qdd, solution->seg.qdd), -1);
  EXPECT_EQ(seg.n_pre, solution->seg.n_pre);
  EXPECT_EQ(seg.t0_ns, solution->seg.t0_ns);
}

TEST(MpcDockingPlannerCycle, WhileThePairIsFollowedTheWakeSearchesFromTheSegmentThenReplans) {
  auto s = std::make_unique<CycleScene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  rtc::catching::PlannerCycleRecord first;
  ASSERT_NO_FATAL_FAILURE(s->PublishPair(first));
  const PlanSnapshot plan = s->plan.Load();
  const SegmentSnapshot followed = s->segment.Load();
  const std::uint64_t plan_stores = s->plan.sequence();
  // 50 ms on: the catch is still further than t_stop_plan away.
  const std::int64_t now = kNow + 50 * kMs;
  ASSERT_GT(plan.t_c_ns - now, 100 * kMs) << "precondition: outside t_stop_plan";
  s->rt.Store(s->FollowingPair(now));
  const rtc::catching::PlannerCycleRecord rec = s->Wake(now);
  // Behind the search a replan from the followed segment was solved (and, on
  // this throw, published) …
  EXPECT_NE(rec.segment.outcome, SegmentOutcome::kOff) << Describe(rec.segment);
  EXPECT_EQ(rec.segment.source_seq, followed.segment_seq) << Describe(rec.segment);
  // … and the search ran, on candidates whose arm motion starts ON the
  // followed segment: node 0 of each solved one is that segment evaluated at
  // the candidate's own start instant, bit for bit (unless the box moved it).
  ASSERT_TRUE(rec.search.nlp.ran);
  ASSERT_GT(rec.search.nlp.n_solved, 0U) << NlpRejectName(rec.search.nlp.reason);
  int checked = 0;
  for (const NlpCatchSearch::CandidateRecord& c : s->search->Candidates()) {
    if (!c.solved) {
      continue;
    }
    EXPECT_EQ(c.source_seq, followed.segment_seq);
    if (c.x0_clamped) {
      continue;
    }
    std::array<double, kMaxSegmentNv> q{};
    std::array<double, kMaxSegmentNv> qd{};
    std::array<double, kMaxSegmentNv> qdd{};
    ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(followed, c.t_s_ns, q, qd, qdd));
    for (std::size_t m = 0; m < 6; ++m) {
      const auto d = static_cast<std::size_t>(kDeviceOfModel[m]);
      EXPECT_TRUE(BitsEqual(c.q0[m], q[d])) << "joint " << m;
      EXPECT_TRUE(BitsEqual(c.qd0[m], qd[d])) << "joint " << m;
      EXPECT_TRUE(BitsEqual(c.qdd0[m], qdd[d])) << "joint " << m;
    }
    ++checked;
  }
  EXPECT_GT(checked, 0) << "no solved candidate started inside the box";
  // On this throw the search does not choose another plan: nothing of one is
  // published, and the RT's plan is the one it took.
  EXPECT_EQ(rec.outcome, rtc::catching::CycleOutcome::kHeld)
      << rtc::catching::CycleOutcomeName(rec.outcome);
  EXPECT_EQ(s->plan.sequence(), plan_stores);
  EXPECT_EQ(s->plan.Load().plan_id, plan.plan_id);
  EXPECT_EQ(s->segment.Load().plan_id, plan.plan_id);

  // Inside t_stop_plan the wake is the replan alone.
  const std::int64_t late = plan.t_c_ns - 100 * kMs;
  s->rt.Store(s->FollowingPair(late));
  const rtc::catching::PlannerCycleRecord stopped = s->Wake(late);
  EXPECT_FALSE(stopped.search.nlp.ran);
  EXPECT_EQ(stopped.outcome, rtc::catching::CycleOutcome::kIdle);
}

TEST(MpcDockingPlannerCycle, AWholeWakeAllocatesNothingOutsideTheQpSolvers) {
  auto s = std::make_unique<CycleScene>();
  ASSERT_NO_FATAL_FAILURE(s->Setup());
  GateLog log;
  s->planner->SetSolverHookForTesting(&SolverHook, &log);
  s->planner->SetCoreStageHookForTesting(&StageHook, &log);
  s->search->SetSolverHookForTesting(&SolverHook, &log);
  s->search->SetCoreStageHookForTesting(&StageHook, &log);
  // Positive control, through the wrapper the wakes run in.
  EXPECT_GT(Gated([&] { const pinocchio::Data probe(*s->rig.arm.model); }), 0U)
      << "the malloc gate does not see library allocations";

  // The pair's wake: the search, the planner's evaluation, both stores.
  auto first = std::make_unique<rtc::catching::PlannerCycleRecord>();
  s->rt.Store(s->rig.RestingRt(kNow));
  SetClockAt(kNow);
  std::size_t counted = Gated([&] { *first = s->cycle.Run(NowReal{kNow}); });
  ASSERT_EQ(rtc::testing::detail::MallocGateDepth(), 0) << "the hooks are not balanced";
  ASSERT_EQ(first->outcome, rtc::catching::CycleOutcome::kPublished) << Describe(first->segment);
  ASSERT_TRUE(first->segment.from_search);

  // A wake while that pair is followed: the replan, then the search.
  const std::int64_t now = kNow + 50 * kMs;
  s->rt.Store(s->FollowingPair(now));
  auto second = std::make_unique<rtc::catching::PlannerCycleRecord>();
  SetClockAt(now);
  counted += Gated([&] { *second = s->cycle.Run(NowReal{now}); });
  ASSERT_EQ(rtc::testing::detail::MallocGateDepth(), 0) << "the hooks are not balanced";
  ASSERT_NE(second->segment.outcome, SegmentOutcome::kOff) << Describe(second->segment);
  ASSERT_TRUE(second->search.nlp.ran);
  ASSERT_GT(second->search.nlp.n_solved, 0U);

  EXPECT_EQ(counted, 0U) << "a wake allocated outside the QP solvers";
  EXPECT_GT(log.solver_calls, 2);
  EXPECT_GT(log.stages, 3);
}

}  // namespace
