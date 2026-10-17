// E3-F02 (#641): the whole-body terms through whole catches of the planner
// cycle — the search on a chain (a trunk and one arm, 10 joints), the mpc
// segment planner on the tree it is part of (the other arm too, 17), an RT
// stand-in that takes every segment it is given, on a pinned clock. Judged
// over the first segment and its replans (Done when "연속 wake"):
//   • the angular-momentum rate stays near the first wake's: every published
//     segment's max ‖k̇_G‖ within (1 + ε) of the first's;
//   • the other arm — the joints a plan leaves out — ends each segment near
//     its rest posture;
//   • the negative control: without its rest weight the arm drifts, further
//     with every wake (what D9′ refuses at configure).
// k̇_G is evaluated here on the published nodes (pinocchio's centroidal map and
// its central difference along q̇), none of the core's.
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controllers/catching/grid_catch_search.hpp"
#include "rtc_controllers/catching/joint_groups.hpp"
#include "rtc_controllers/catching/mpc_segment_planner.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/transition_table.hpp"
#include "rtc_controllers/testing/grid_catch_search_fixture.hpp"
#include "rtc_controllers/testing/synthetic_tree_fixture.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

using rtc::catching::CycleOutcome;
using rtc::catching::Mode;
using rtc::catching::PlannerCycleRecord;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanSnapshot;
using rtc::catching::SegmentOutcome;
using rtc::catching::SegmentOutcomeName;
using rtc::catching::SegmentSnapshot;
namespace fx = rtc::testing::mpc_segment_core;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kTArm = 50 * kMs;
constexpr std::int64_t kH = 2 * kMs;
constexpr std::uint64_t kActivation = 3;
constexpr std::uint64_t kTrack = 7;
constexpr int kTrunk = 3;
constexpr int kArm = 7;
constexpr int kChainNv = kTrunk + kArm;     // 10
constexpr int kTreeNv = kTrunk + 2 * kArm;  // 17
constexpr int kSegNv = rtc::catching::kMaxSegmentNv;

// The judgement's values (#641 stage 0, D15): W⁻ / W⁺ = 0.001 / 0.1 at
// s = 10 N·m, the left-out arm's rest weight 1; ε = 0.1, the end node within
// 1e-2 rad of rest.
constexpr double kAngPre = 0.001;
constexpr double kAngPost = 0.1;
constexpr double kRest = 1.0;
constexpr double kEps = 0.1;
constexpr double kEndTol = 1e-2;

std::atomic<std::int64_t> g_now{0};

std::int64_t Clock() noexcept {
  return g_now.load();
}

// The RT under `mode: mpc`: adopt the first plan, hold each newer segment of
// it pending until its node 0, then follow it — and report it (the planner
// replans from what the RT reports). Mode by time.
struct RtStandIn {
  std::vector<double> wait_pose;  // device order
  bool following{false};
  std::uint32_t plan_id{0};
  std::int64_t t_c{0};
  std::uint32_t last_seq{0};
  bool pending{false};
  std::uint32_t pending_seq{0};
  std::int64_t pending_t0{0};
  bool active{false};
  std::uint32_t active_seq{0};

  void Tick(rtc::SeqLock<PlannerRtState>& rt_box, const rtc::SeqLock<PlanSnapshot>& plan_box,
            const rtc::SeqLock<SegmentSnapshot>& segment_box, std::int64_t now) {
    if (!following) {
      const PlanSnapshot p = plan_box.Load();
      if (p.valid) {
        following = true;
        plan_id = p.plan_id;
        t_c = p.t_c_ns;
      }
    }
    if (following) {
      const SegmentSnapshot d = segment_box.Load();
      if (d.valid && d.plan_id == plan_id && d.segment_seq > last_seq) {
        last_seq = d.segment_seq;
        pending = true;
        pending_seq = d.segment_seq;
        pending_t0 = d.t0_ns;
      }
      if (pending && now + kTArm + kH >= pending_t0) {
        active = true;
        active_seq = pending_seq;
        pending = false;
      }
    }
    PlannerRtState s = rtc::testing::TrackingRtState(kActivation, wait_pose);
    s.rt_iteration = static_cast<std::uint64_t>(now / kH);
    s.rt_state_ns = now;
    s.reset_epoch = 1;
    s.track_seen = true;
    s.track_generation = kTrack;
    if (following) {
      const std::int64_t lead = t_c - now;
      s.mode = static_cast<std::uint8_t>(lead <= 0           ? Mode::kDecel
                                         : lead <= 200 * kMs ? Mode::kCommitted
                                                             : Mode::kApproach);
      s.plan_active = true;
      s.plan_id = plan_id;
      s.plan_t_c_ns = t_c;
      s.segment_pending = pending;
      s.segment_pending_seq = pending ? pending_seq : 0;
      s.segment_active = active;
      s.segment_seq = active ? active_seq : 0;
    }
    rt_box.Store(s);
  }
};

struct TreeCycle {
  fx::ArmModel chain = rtc::testing::SyntheticTree(kTrunk, kArm, 1);
  fx::ArmModel tree = rtc::testing::SyntheticTree(kTrunk, kArm, 2);
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle;
  rtc::SeqLock<rtc::catching::TrajectorySnapshot> traj;
  rtc::SeqLock<rtc::catching::CovarianceSnapshot> cov;
  rtc::SeqLock<PlannerRtState> rt_box;
  rtc::SeqLock<PlanSnapshot> plan_box;
  rtc::SeqLock<SegmentSnapshot> segment_box;
  rtc::catching::PlannerCycle cycle;
  RtStandIn rt;
  rtc::catching::PlannerParams params;
  Eigen::Vector3d p_c, v_ball;
  std::int64_t first_ns{0};

  TreeCycle(double w_rest, double w_pre, double w_post) {
    g_now.store(1'727'000'000'000'000'000LL);
    handle = std::make_unique<rtc_urdf_bridge::RtModelHandle>(chain.model);
    // The catch posture: the trunk and the catch arm off the nominal one; the
    // ball comes down the catch frame's −z there. The wait pose is a little
    // off it on those joints; the other arm waits on its rest posture.
    Eigen::VectorXd q_ref = tree.q_nominal;
    for (int m = 0; m < kChainNv; ++m) {
      q_ref[m] += m % 2 == 0 ? 0.06 : -0.05;
    }
    pinocchio::Data data(*tree.model);
    pinocchio::framesForwardKinematics(*tree.model, data, q_ref);
    p_c = data.oMf[tree.frame].translation();
    v_ball = -1.0 * data.oMf[tree.frame].rotation().col(2);
    for (int m = 0; m < kTreeNv; ++m) {
      rt.wait_pose.push_back(m < kChainNv ? q_ref[m] + (m % 2 == 0 ? 0.03 : -0.03)
                                          : tree.q_nominal[m]);
      params.wait_pose[static_cast<std::size_t>(m)] = rt.wait_pose.back();
    }
    params.enabled = true;
    params.wait_pose_n = kTreeNv;
    params.t_freeze = 0.2;
    params.t_stop_plan = params.t_freeze;
    params.slice_t_lead_min = 0.6;
    params.slice_t_max = 0.8;
    params.n_settle = 0;
    params.d_eff = 0.2;
    params.r_cap = 0.03;
    params.max_ik = 8;
    params.budget_s = 0.05;
    auto& d = params.mpc_segment;
    d.n_nodes = 7;
    d.dt_s = 0.05;
    d.blocks = {1, 1, 2, 3};
    d.n_blocks = 4;
    d.k_max = 2;
    d.n_pre_max = 6;
    d.dt_pre_s = 0.1;
    d.horizon_explicit = true;
    d.budget_first_s = 0.1;
    d.budget_replan_s = 0.1;
    // The whole-body terms: three groups (the trunk, the catch arm, the other
    // arm), the other's rest weight and the angular rate.
    d.n_groups = 3;
    d.groups[2].w_rest = w_rest;
    d.ang_w_pre = w_pre;
    d.ang_w_post = w_post;
    d.ang_scale = 10.0;
    cycle.Configure(params);
    cycle.SetClock(&Clock);
    EXPECT_TRUE(cycle.Bind({&traj, &cov, &rt_box, &plan_box, &segment_box}));

    // The search: the chain, as device joints 0..9 of the tree.
    rtc::catching::GridCatchSearchModel pm;
    pm.handle = handle.get();
    pm.catch_frame = chain.frame;
    pm.nv = kChainNv;
    pm.n_dev = kTreeNv;
    pm.accel_box = true;
    for (int m = 0; m < kChainNv; ++m) {
      pm.device_of_model[static_cast<std::size_t>(m)] = m;
      pm.qdot_max[static_cast<std::size_t>(m)] = 3.0;
      pm.qddot_max[static_cast<std::size_t>(m)] = 20.0;
    }
    rtc::catching::GridCatchSearchConstants pc;
    pc.eta_v = 0.9;
    pc.v_max = 3.0;
    pc.a_dec = 10.0;
    pc.t_arm_s = static_cast<double>(kTArm) * 1e-9;
    pc.t_close_lead = 0.1;
    pc.t_close_total = 0.101;
    pc.ball_mass = 0.057;
    pc.ref_omega = 10.0;
    pc.ref_zeta = 1.0;
    pc.ref_a_max = 21.0;
    pc.control_dt = 0.002;
    rtc::catching::CatchPoseIkOptions ik;
    ik.max_iter = 60;
    ik.manipulability_min = 0.0;
    auto search = rtc::catching::MakeGridCatchSearch(pm, pc, params, ik, &Clock);
    EXPECT_NE(search, nullptr);
    cycle.InstallSearch(std::move(search));

    // The segment planner: the tree.
    rtc::catching::MpcSegmentPlannerModel dm;
    dm.arm = tree.model;
    dm.catch_frame = tree.frame;
    dm.nv = kTreeNv;
    dm.groups.n_groups = 3;
    for (int m = 0; m < kTreeNv; ++m) {
      const auto u = static_cast<std::size_t>(m);
      dm.device_of_model[u] = m;
      dm.q_min[u] = tree.model->lowerPositionLimit[m];
      dm.q_max[u] = tree.model->upperPositionLimit[m];
      dm.qdot_max[u] = 3.0;
      dm.tau_max[u] = 300.0;
      dm.groups.group_of_device[u] =
          static_cast<std::int8_t>(m < kTrunk ? 0 : (m < kChainNv ? 1 : 2));
      dm.rest_pose[u] = rt.wait_pose[u];
    }
    dm.rest_pose_n = kTreeNv;
    rtc::catching::MpcSegmentPlannerConstants dc;
    dc.eta_v = 0.9;
    dc.t_arm_s = pc.t_arm_s;
    dc.control_dt = 0.002;
    dc.v_eps = 1e-6;
    std::string err;
    auto planner = rtc::catching::MakeMpcSegmentPlanner(dm, dc, params.mpc_segment, &Clock, &err);
    EXPECT_NE(planner, nullptr) << err;
    cycle.InstallSegmentPlanner(std::move(planner));
  }

  void StoreTrajectory(std::uint64_t seq) {
    const auto t = rtc::testing::LineTrajectory(p_c, v_ball, first_ns, 50 * kMs, 20, 14, seq,
                                                kTrack, kActivation, Clock() - 5 * kMs);
    cov.Store(rtc::testing::IsotropicCovariance(t, 0.004));
    traj.Store(t);
  }

  PlannerCycleRecord Wake() {
    rt.Tick(rt_box, plan_box, segment_box, Clock());
    return cycle.Run(rtc::catching::NowReal{Clock()});
  }
};

// What a published segment says about the whole-body terms: its largest
// angular-momentum rate over nodes 1..N and how far the other arm ends from
// its rest posture.
struct SegmentLook {
  double rate_max{0.0};
  double other_end{0.0};
  rtc::catching::SegmentKind kind{rtc::catching::SegmentKind::kFirst};
};

SegmentLook Look(const TreeCycle& c, const SegmentSnapshot& s) {
  const pinocchio::Model& model = *c.tree.model;
  pinocchio::Data data(model);
  SegmentLook out;
  const auto at = [&](int k, int m) { return static_cast<std::size_t>(k * kSegNv + m); };
  for (int k = 1; k <= s.n_nodes; ++k) {
    Eigen::VectorXd q(kTreeNv), v(kTreeNv), a(kTreeNv);
    for (int m = 0; m < kTreeNv; ++m) {
      q[m] = s.q[at(k, m)];
      v[m] = s.qd[at(k, m)];
      a[m] = s.qdd[at(k, m)];
    }
    const Eigen::MatrixXd ag = pinocchio::computeCentroidalMap(model, data, q).bottomRows<3>();
    const double eps = 1e-6;
    const Eigen::MatrixXd ap =
        pinocchio::computeCentroidalMap(model, data, q + eps * v).bottomRows<3>();
    const Eigen::MatrixXd am =
        pinocchio::computeCentroidalMap(model, data, q - eps * v).bottomRows<3>();
    const Eigen::Vector3d kd = ag * a + (ap - am) / (2.0 * eps) * v;
    out.rate_max = std::max(out.rate_max, kd.norm());
  }
  for (int m = kChainNv; m < kTreeNv; ++m) {
    out.other_end = std::max(out.other_end, std::abs(s.q[at(s.n_nodes, m)] -
                                                     c.rt.wait_pose[static_cast<std::size_t>(m)]));
  }
  return out;
}

struct WakeTrace {
  std::vector<SegmentLook> segments;  // every published segment, in order
  int cold_retried{0};
  int x0_clamped{0};
};

// A catch: the first wake publishes the pair, then a wake every 15 ms until
// the catch instant, the RT taking each segment.
WakeTrace RunCatch(TreeCycle& c) {
  WakeTrace t;
  c.first_ns = Clock();
  c.StoreTrajectory(1);
  const PlannerCycleRecord first = c.Wake();
  EXPECT_EQ(first.outcome, CycleOutcome::kPublished)
      << rtc::catching::CycleOutcomeName(first.outcome) << " / "
      << SegmentOutcomeName(first.segment.outcome) << " / " << first.segment.core_reason_name;
  if (first.outcome != CycleOutcome::kPublished) {
    return t;
  }
  const std::int64_t t_c = c.plan_box.Load().t_c_ns;
  t.segments.push_back(Look(c, c.segment_box.Load()));
  std::uint64_t seq = 1;
  while (Clock() < t_c) {
    g_now.fetch_add(15 * kMs);
    c.StoreTrajectory(++seq);
    const PlannerCycleRecord rec = c.Wake();
    if (rec.segment.outcome == SegmentOutcome::kPublished) {
      t.segments.push_back(Look(c, c.segment_box.Load()));
      t.segments.back().kind = rec.segment.kind;
      t.cold_retried += rec.segment.solver_retried ? 1 : 0;
      t.x0_clamped += rec.segment.x0_clamped ? 1 : 0;
    }
  }
  return t;
}

TEST(WholeBodyCycle, TheRateStaysNearTheFirstWakesAndTheOtherArmEndsAtRest) {
  auto c = std::make_unique<TreeCycle>(kRest, kAngPre, kAngPost);
  const WakeTrace t = RunCatch(*c);
  ASSERT_GE(t.segments.size(), 6U) << "the first segment and five replans";
  RecordProperty("segments", static_cast<int>(t.segments.size()));
  RecordProperty("cold_retried", t.cold_retried);
  RecordProperty("x0_clamped", t.x0_clamped);
  const double first = t.segments.front().rate_max;
  ASSERT_GT(first, 0.0);
  for (std::size_t i = 0; i < t.segments.size(); ++i) {
    SCOPED_TRACE("segment " + std::to_string(i));
    EXPECT_LE(t.segments[i].rate_max, (1.0 + kEps) * first);
  }
  EXPECT_LE(t.segments.back().other_end, kEndTol);
}

// The replans of the followed plan start where the segment the RT follows has
// taken the arm (a new plan the search pairs — kFirst — starts again from the
// wait pose, and the RT here does not take it): with nothing to bring the
// other arm back, no replan returns it, and it ends far from rest.
TEST(WholeBodyCycle, WithoutItsRestWeightTheOtherArmDriftsFurtherEveryWake) {
  auto c = std::make_unique<TreeCycle>(0.0, kAngPre, kAngPost);
  const WakeTrace t = RunCatch(*c);
  ASSERT_GE(t.segments.size(), 6U);
  double last = 0.0;
  int replans = 0;
  for (std::size_t i = 0; i < t.segments.size(); ++i) {
    if (i > 0 && t.segments[i].kind == rtc::catching::SegmentKind::kFirst) {
      continue;
    }
    SCOPED_TRACE("segment " + std::to_string(i));
    // Near where the drift saturates (the jerk cost against the angular term)
    // the end node wobbles by micro-radians; a return is more than 1 % of the
    // positive case's tolerance.
    EXPECT_GE(t.segments[i].other_end, last - 0.01 * kEndTol) << "a replan brought it back";
    last = t.segments[i].other_end;
    ++replans;
  }
  EXPECT_GE(replans, 6);
  EXPECT_GT(t.segments.back().other_end, t.segments.front().other_end);
  EXPECT_GT(t.segments.back().other_end, 10.0 * kEndTol);
  RecordProperty("other_arm_end_first", std::to_string(t.segments.front().other_end));
  RecordProperty("other_arm_end_last", std::to_string(t.segments.back().other_end));
}

}  // namespace
