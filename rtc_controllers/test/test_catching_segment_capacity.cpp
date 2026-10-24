// ── The segment payload's joint capacity (#827) ──────────────────────────────
// kMaxSegmentNv is the widest model a segment may be planned on. It was sized
// for one arm; a segment model that is a trunk and two arms needs more. This
// suite puts a model of 17 joints — wider than an arm, and no robot's — through
// every place the capacity is checked:
//   1. the payload and the RT's sampler: 17 joints × (kMaxSegmentNodes + 1)
//      nodes validate, and Sample() reads every joint of every node back and
//      runs FK on the 17-joint model;
//   2. both segment planners configure on it (cores built and warmed up), and
//      refuse a model one joint past the capacity;
//   3. the mpc planner PRODUCES a 17-joint segment — a first solve from rest
//      and a replan from the published one — that validates and that the RT's
//      sampler reads back (the writer's side of the capacity: Pack and the
//      start state read from a reported segment);
//   4. both searches configure on the chain the catch is searched on (the
//      trunk and one arm, 10 joints — wider than the old capacity), and
//      refuse a chain their catch-pose IK cannot be sized for.
// Not here: a first segment of the docking planner at 17 joints. It adopts a
// search's solution or is solved from rest, and what it starts from when the
// search ran on a narrower model is not defined here.
//
// Mutation: with kMaxSegmentNv at 16 every "17 joints" case here fails.
#include "rtc_base/types/types.hpp"
#include "rtc_controllers/catching/catch_pose_ik.hpp"
#include "rtc_controllers/catching/grid_catch_search.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_planner.hpp"
#include "rtc_controllers/catching/mpc_segment_planner.hpp"
#include "rtc_controllers/catching/nlp_catch_search.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/segment_planner.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_controllers/catching/transition_table.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"
#include "rtc_controllers/testing/mpc_segment_core_fixture.hpp"
#include "rtc_controllers/testing/mpc_warm_up_fixture.hpp"
#include "rtc_controllers/testing/synthetic_tree_fixture.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace {

using rtc::catching::GridCatchSearch;
using rtc::catching::GridCatchSearchConstants;
using rtc::catching::GridCatchSearchModel;
using rtc::catching::kMaxPlanNv;
using rtc::catching::kMaxSegmentNodes;
using rtc::catching::kMaxSegmentNv;
using rtc::catching::kNlpMaxSearchNv;
using rtc::catching::MpcDockingSegmentPlanner;
using rtc::catching::MpcDockingSegmentPlannerConstants;
using rtc::catching::MpcDockingSegmentPlannerModel;
using rtc::catching::MpcDockingSegmentPlannerParams;
using rtc::catching::MpcSegmentBallTarget;
using rtc::catching::MpcSegmentPlanner;
using rtc::catching::MpcSegmentPlannerConstants;
using rtc::catching::MpcSegmentPlannerModel;
using rtc::catching::MpcSegmentPlannerParams;
using rtc::catching::NlpCatchSearch;
using rtc::catching::NlpCatchSearchConstants;
using rtc::catching::NlpCatchSearchModel;
using rtc::catching::NlpCatchSearchParams;
using rtc::catching::NodeTrajectoryFollower;
using rtc::catching::PlannerParams;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanSnapshot;
using rtc::catching::SegmentNodeSample;
using rtc::catching::SegmentOutcomeName;
using rtc::catching::SegmentRecord;
using rtc::catching::SegmentSnapshot;
using rtc::catching::ValidateSegmentNodes;
using rtc::testing::SyntheticTree;
namespace fx = rtc::testing::mpc_segment_core;
namespace dk = rtc::testing::mpc_docking;

// A trunk of 3 and arms of 7: the chain a catch is searched on is the trunk and
// one arm, the segment model is the trunk and both.
constexpr int kTrunk = 3;
constexpr int kArm = 7;
constexpr int kChainNv = kTrunk + kArm;     // 10
constexpr int kTreeNv = kTrunk + 2 * kArm;  // 17
// The chain is a model the searches take: a suite built on another chain
// would have to say so here, at compile time.
static_assert(kChainNv <= kNlpMaxSearchNv);
static_assert(kChainNv <= rtc::kMaxRobotDOF);

constexpr std::int64_t kDtNs = 25'000'000;
constexpr std::int64_t kTc = 1'727'000'000'123'456'789LL;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kT0 = 1'727'000'000'000'000'000LL;
constexpr std::int64_t kTArm = 30 * kMs;  // T_arm
constexpr std::int64_t kH = 2 * kMs;      // control_dt

std::atomic<std::int64_t> g_clock{0};

std::int64_t StepClock() noexcept {
  return g_clock.fetch_add(1000, std::memory_order_relaxed) + 1000;
}

// A clock that stands where a test put it (the planners' deadlines read it).
std::atomic<std::int64_t> g_now{kT0};

std::int64_t FixedClock() noexcept {
  return g_now.load(std::memory_order_relaxed);
}

std::size_t U(int i) {
  return static_cast<std::size_t>(i);
}

// device_of_model[m] = nv − 1 − m: a non-identity permutation at every width,
// so a reader that skipped the mapping would show.
int DeviceOf(int m, int nv) {
  return nv - 1 - m;
}

// A tree one joint past the payload's capacity (the trunk takes the excess).
fx::ArmModel TreeOverCapacity() {
  return SyntheticTree(kMaxSegmentNv + 1 - 2 * kArm, kArm, 2);
}

// ── 1. The payload and the RT's sampler ──────────────────────────────────────

// Node (k, device joint d): distinct for every pair, so a stride or a mapping
// that is off by one reads another entry's value.
double NodeValue(int k, int d) {
  return 0.01 * (k + 1) + 0.001 * (d + 1);
}

TEST(SegmentCapacity, ASeventeenJointSegmentIsValidatedAndSampledWhole) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  ASSERT_EQ(tree.model->nv, kTreeNv);
  ASSERT_GE(kMaxSegmentNv, kTreeNv) << "the payload cannot carry the segment model";
  std::array<int, kMaxPlanNv> device_of_model{};
  for (int m = 0; m < kTreeNv; ++m) {
    device_of_model[U(m)] = DeviceOf(m, kTreeNv);
  }
  NodeTrajectoryFollower follower;
  ASSERT_TRUE(follower.Init(tree.model, tree.frame,
                            std::span<const int>(device_of_model.data(), U(kTreeNv))));
  EXPECT_EQ(follower.Nv(), kTreeNv);

  SegmentSnapshot p{};
  p.valid = true;
  p.nv = kTreeNv;
  p.n_nodes = kMaxSegmentNodes;
  p.dt_ns = kDtNs;
  p.t_c_ns = kTc;
  p.t0_ns = kTc;
  p.plan_id = 1;
  p.segment_seq = 1;
  for (int k = 0; k <= p.n_nodes; ++k) {
    for (int d = 0; d < kTreeNv; ++d) {
      const auto e = U(k * kMaxSegmentNv + d);
      p.q[e] = NodeValue(k, d);
      // Node N at rest (the terminal equality ValidateSegmentNodes requires).
      p.qd[e] = k == p.n_nodes ? 0.0 : 0.1 * NodeValue(k, d);
      p.qdd[e] = k == p.n_nodes ? 0.0 : -0.2 * NodeValue(k, d);
    }
  }
  ASSERT_TRUE(ValidateSegmentNodes(p));

  pinocchio::Data data(*tree.model);
  Eigen::VectorXd q_model(kTreeNv);
  SegmentNodeSample s{};
  std::array<double, kMaxSegmentNv> q{};
  std::array<double, kMaxSegmentNv> qd{};
  std::array<double, kMaxSegmentNv> qdd{};
  for (int k = 0; k <= p.n_nodes; ++k) {
    const std::int64_t t = p.t0_ns + k * kDtNs;
    ASSERT_TRUE(follower.Sample(p, t, s)) << "node " << k;
    ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(p, t, q, qd, qdd)) << "node " << k;
    for (int d = 0; d < kTreeNv; ++d) {
      const auto e = U(k * kMaxSegmentNv + d);
      EXPECT_NEAR(s.q[U(d)], p.q[e], 1e-12) << k << " " << d;
      EXPECT_NEAR(s.qd[U(d)], p.qd[e], 1e-12) << k << " " << d;
      EXPECT_NEAR(s.qdd[U(d)], p.qdd[e], 1e-12) << k << " " << d;
      EXPECT_EQ(q[U(d)], s.q[U(d)]) << k << " " << d;
      EXPECT_EQ(qd[U(d)], s.qd[U(d)]) << k << " " << d;
      EXPECT_EQ(qdd[U(d)], s.qdd[U(d)]) << k << " " << d;
    }
    // FK ran on all 17 joints, each read through the device mapping.
    for (int m = 0; m < kTreeNv; ++m) {
      q_model[m] = NodeValue(k, DeviceOf(m, kTreeNv));
    }
    pinocchio::forwardKinematics(*tree.model, data, q_model);
    pinocchio::updateFramePlacement(*tree.model, data, tree.frame);
    EXPECT_LT((s.placement.translation() - data.oMf[tree.frame].translation()).norm(), 1e-12)
        << "node " << k;
    EXPECT_LT((s.placement.rotation() - data.oMf[tree.frame].rotation()).norm(), 1e-12)
        << "node " << k;
  }

  // One joint past the capacity is not a payload, and not a model the sampler binds.
  SegmentSnapshot over = p;
  over.nv = kMaxSegmentNv + 1;
  EXPECT_FALSE(ValidateSegmentNodes(over));
  const fx::ArmModel wide = TreeOverCapacity();
  std::array<int, kMaxPlanNv> wide_map{};
  for (int m = 0; m < wide.model->nv; ++m) {
    wide_map[U(m)] = DeviceOf(m, wide.model->nv);
  }
  NodeTrajectoryFollower unbound;
  EXPECT_FALSE(unbound.Init(wide.model, wide.frame,
                            std::span<const int>(wide_map.data(), U(wide.model->nv))));
}

// ── 2. The segment planners ──────────────────────────────────────────────────

MpcSegmentPlannerModel MpcModelOf(const fx::ArmModel& a) {
  MpcSegmentPlannerModel pm;
  pm.arm = a.model;
  pm.catch_frame = a.frame;
  pm.nv = a.model->nv;
  for (int m = 0; m < pm.nv; ++m) {
    pm.device_of_model[U(m)] = DeviceOf(m, pm.nv);
    pm.q_min[U(m)] = a.model->lowerPositionLimit[m];
    pm.q_max[U(m)] = a.model->upperPositionLimit[m];
    pm.qdot_max[U(m)] = a.model->upperVelocityLimit[m];
    pm.tau_max[U(m)] = a.model->upperEffortLimit[m];
  }
  return pm;
}

// The shipped approach profile's shape (7 × 0.05 s [1, 1, 2, 3] after the
// catch, up to 6 × 0.1 s before it, k_max 2).
MpcSegmentPlannerParams MpcParams() {
  MpcSegmentPlannerParams p;
  p.n_nodes = 7;
  p.dt_s = 0.05;
  p.blocks = {1, 1, 2, 3};
  p.n_blocks = 4;
  p.k_max = 2;
  p.n_pre_max = 6;
  p.dt_pre_s = 0.1;
  p.horizon_explicit = true;
  return p;
}

MpcSegmentPlannerConstants MpcConsts() {
  MpcSegmentPlannerConstants c;
  c.eta_v = 0.9;
  c.t_arm_s = static_cast<double>(kTArm) * 1e-9;
  c.control_dt = static_cast<double>(kH) * 1e-9;
  c.v_eps = 1e-6;
  return c;
}

TEST(SegmentCapacity, TheMpcPlannerConfiguresOnSeventeenJoints) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  ASSERT_EQ(tree.model->nv, kTreeNv);
  MpcSegmentPlanner planner;
  std::string err;
  EXPECT_TRUE(planner.Configure(MpcModelOf(tree), MpcConsts(), MpcParams(), &StepClock, &err))
      << err;
  EXPECT_TRUE(planner.Configured());
  EXPECT_EQ(planner.Nv(), kTreeNv);
}

// #880: the warm-ups at the middle of the tree's box (q = 0 on its
// symmetric limits), per infeasibility threshold — recorded, and none fails at
// the cores' own threshold; the replica is the planner's own warm-up at the
// planner's pose.
TEST(SegmentCapacity, TheSeventeenJointWarmUpsAtTheBoxMiddlePerInfeasibilityThreshold) {
  namespace wu = rtc::testing::mpc_warm_up;
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  const MpcSegmentPlannerModel model = MpcModelOf(tree);
  MpcSegmentPlanner planner;
  std::string err;
  ASSERT_TRUE(planner.Configure(model, MpcConsts(), MpcParams(), &StepClock, &err)) << err;
  for (const wu::WarmUpSolve& s :
       wu::WarmUpSolvesAt(planner, model, wu::PlannerWarmUpPose(planner),
                          planner.StopCoreParams(0).solver.eps_primal_inf)) {
    EXPECT_TRUE(s.ok && s.same_as_planner) << s.core;
  }
  const Eigen::VectorXd mid = wu::BoxMiddle(planner);
  for (const double eps : wu::EpsGrid()) {
    const std::vector<wu::WarmUpSolve> solves = wu::WarmUpSolvesAt(planner, model, mid, eps);
    wu::PrintWarmUpSolves("tree17", eps, solves);
    int failed = 0;
    for (const wu::WarmUpSolve& s : solves) {
      failed += s.ok ? 0 : 1;
    }
    RecordProperty("tree17.eps_" + wu::EpsKey(eps) + ".failed", failed);
    if (eps == rtc::catching::MpcSegmentCoreParams{}.solver.eps_primal_inf) {
      EXPECT_EQ(failed, 0) << "at the cores' own threshold (#880)";
    }
  }
}

TEST(SegmentCapacity, TheMpcPlannerRefusesAModelPastTheCapacity) {
  const fx::ArmModel wide = TreeOverCapacity();
  ASSERT_EQ(wide.model->nv, kMaxSegmentNv + 1);
  MpcSegmentPlanner planner;
  std::string err;
  EXPECT_FALSE(planner.Configure(MpcModelOf(wide), MpcConsts(), MpcParams(), &StepClock, &err));
  EXPECT_NE(err.find("kMaxSegmentNv"), std::string::npos) << err;
  EXPECT_FALSE(planner.Configured());
}

// The frame's position at node `k` of a planner's segment (device → model).
Eigen::Vector3d NodeFramePos(const fx::ArmModel& a, const SegmentSnapshot& p, int k) {
  const int nv = a.model->nv;
  Eigen::VectorXd q(nv);
  for (int m = 0; m < nv; ++m) {
    q[m] = p.q[U(k * kMaxSegmentNv + DeviceOf(m, nv))];
  }
  pinocchio::Data data(*a.model);
  pinocchio::forwardKinematics(*a.model, data, q);
  pinocchio::updateFramePlacement(*a.model, data, a.frame);
  return data.oMf[a.frame].translation();
}

TEST(SegmentCapacity, TheMpcPlannerProducesASeventeenJointSegmentAndReplansFromIt) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  ASSERT_EQ(tree.model->nv, kTreeNv);
  ASSERT_GE(kMaxSegmentNv, kTreeNv) << "the payload cannot carry the segment model";
  MpcSegmentPlanner planner;
  std::string err;
  g_now.store(kT0);
  ASSERT_TRUE(planner.Configure(MpcModelOf(tree), MpcConsts(), MpcParams(), &FixedClock, &err))
      << err;

  // A catch the tree reaches a little away from where it rests: the ball
  // passes the catch frame's position along the frame's −z.
  Eigen::VectorXd q_catch = tree.q_nominal;
  for (int m = 0; m < kTreeNv; ++m) {
    q_catch[m] += m % 2 == 0 ? 0.04 : -0.04;
  }
  pinocchio::Data data(*tree.model);
  pinocchio::forwardKinematics(*tree.model, data, q_catch);
  pinocchio::updateFramePlacement(*tree.model, data, tree.frame);
  const Eigen::Vector3d p_c = data.oMf[tree.frame].translation();
  const Eigen::Vector3d a_d = data.oMf[tree.frame].rotation().col(2);
  const Eigen::Vector3d v_c = -5.0 * a_d;
  const std::int64_t t_c = kT0 + kTArm + 800 * kMs;

  PlanSnapshot plan{};
  plan.valid = true;
  plan.plan_id = 7;
  plan.t_c_ns = t_c;
  plan.token.activation_generation = 3;
  plan.token.generation = 5;
  plan.nv = kTreeNv;
  plan.joint_set = rtc::catching::FullJointSet(kTreeNv);  // a plan names its joints (#640)
  PlannerRtState rt{};
  rt.valid = true;
  rt.activation_generation = 3;
  rt.rt_iteration = 1000;
  rt.rt_state_ns = kT0 - kH;
  rt.reset_epoch = 1;
  rt.mode = static_cast<std::uint8_t>(rtc::catching::Mode::kTracking);
  rt.nv = kTreeNv;
  rt.cmd_seeded = true;
  rt.track_seen = true;
  rt.track_generation = 5;
  for (int m = 0; m < kTreeNv; ++m) {
    plan.q_star[U(DeviceOf(m, kTreeNv))] = q_catch[m];
    rt.q_cmd[U(DeviceOf(m, kTreeNv))] = tree.q_nominal[m];
  }
  for (int i = 0; i < 3; ++i) {
    plan.p_c[U(i)] = p_c[i];
    plan.v_c[U(i)] = v_c[i];
    plan.a_d[U(i)] = a_d[i];
  }
  MpcSegmentBallTarget ball;
  ball.valid = true;
  ball.p_b = p_c;
  ball.v_b = v_c;
  ball.a_d = a_d;
  ball.sigma_p = 1e-4 * Eigen::Matrix3d::Identity();
  ball.sigma_valid = true;

  // ── The first segment, from rest ──
  SegmentSnapshot first{};
  SegmentRecord rec{};
  ASSERT_TRUE(planner.PlanFirst(rt, plan, ball, first, rec))
      << SegmentOutcomeName(rec.outcome) << " / " << rec.core_reason_name;
  first.segment_seq = 1;
  first.publish_ns = kT0;
  EXPECT_EQ(first.nv, kTreeNv);
  ASSERT_TRUE(ValidateSegmentNodes(first));
  ASSERT_GE(first.n_pre, 1);
  // Node 0 is the command of all 17 joints, in DEVICE order; node N rests.
  for (int m = 0; m < kTreeNv; ++m) {
    const int d = DeviceOf(m, kTreeNv);
    EXPECT_NEAR(first.q[U(d)], tree.q_nominal[m], 1e-12) << m;
    EXPECT_NEAR(first.qd[U(d)], 0.0, 1e-9) << m;
    EXPECT_LE(std::fabs(first.qd[U(first.n_nodes * kMaxSegmentNv + d)]), 1e-4) << m;
  }
  // The catch node reaches the ball (the planner's own gate).
  EXPECT_LE((NodeFramePos(tree, first, first.n_pre) - p_c).norm(), 0.02);
  // The RT's sampler reads the planner's payload at every node.
  std::array<int, kMaxPlanNv> device_of_model{};
  for (int m = 0; m < kTreeNv; ++m) {
    device_of_model[U(m)] = DeviceOf(m, kTreeNv);
  }
  NodeTrajectoryFollower follower;
  ASSERT_TRUE(follower.Init(tree.model, tree.frame,
                            std::span<const int>(device_of_model.data(), U(kTreeNv))));
  SegmentNodeSample s{};
  for (int k = 0; k <= first.n_nodes; ++k) {
    ASSERT_TRUE(follower.Sample(first, rtc::catching::SegmentNodeTimeNs(first, k), s)) << k;
    for (int d = 0; d < kTreeNv; ++d) {
      EXPECT_NEAR(s.q[U(d)], first.q[U(k * kMaxSegmentNv + d)], 1e-9) << k << " " << d;
    }
  }

  // ── A replan from the published segment: its start is that segment, read
  //    back at 17 joints ──
  planner.NotePublished(first);
  const std::int64_t now = kT0 + 20 * kMs;
  g_now.store(now);
  PlannerRtState following = rt;
  following.rt_state_ns = now - kH;
  following.mode = static_cast<std::uint8_t>(rtc::catching::Mode::kApproach);
  following.plan_active = true;
  following.plan_id = plan.plan_id;
  following.plan_t_c_ns = t_c;
  following.segment_pending = true;
  following.segment_pending_seq = first.segment_seq;
  SegmentSnapshot second{};
  ASSERT_TRUE(planner.Replan(following, ball, second, rec))
      << SegmentOutcomeName(rec.outcome) << " / " << rec.core_reason_name;
  EXPECT_EQ(second.nv, kTreeNv);
  second.segment_seq = 2;
  second.publish_ns = now;
  ASSERT_TRUE(ValidateSegmentNodes(second));
  std::array<double, kMaxSegmentNv> q{};
  std::array<double, kMaxSegmentNv> qd{};
  std::array<double, kMaxSegmentNv> qdd{};
  ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(first, second.t0_ns, q, qd, qdd));
  for (int d = 0; d < kTreeNv; ++d) {
    EXPECT_NEAR(second.q[U(d)], q[U(d)], 1e-9) << d;
    EXPECT_NEAR(second.qd[U(d)], qd[U(d)], 1e-9) << d;
    EXPECT_NEAR(second.qdd[U(d)], qdd[U(d)], 1e-9) << d;
  }
  EXPECT_LE((NodeFramePos(tree, second, second.n_pre) - p_c).norm(), 0.02);
}

// The docking planner's inputs for a model: the cores' box and parameters are
// the docking fixture's, the grid the suites' (8 × 0.05 s before the catch, a
// 7-node stop in four blocks).
struct DockingConfig {
  MpcDockingSegmentPlannerModel model;
  MpcDockingSegmentPlannerConstants consts;
  MpcDockingSegmentPlannerParams params;
};

DockingConfig DockingConfigOf(const fx::ArmModel& a) {
  const dk::Rig rig = dk::MakeRig(a);
  DockingConfig c;
  c.model.arm = a.model;
  c.model.catch_frame = a.frame;
  c.model.nv = a.model->nv;
  c.model.limits = rig.limits;
  for (int m = 0; m < c.model.nv; ++m) {
    c.model.device_of_model[U(m)] = DeviceOf(m, c.model.nv);
    c.model.qd_rating[U(m)] = 2.0 * rig.limits.qd_max[m];
    c.model.warm_pose[U(DeviceOf(m, c.model.nv))] = a.q_nominal[m];
  }
  c.consts.t_arm_s = 0.0;
  c.consts.control_dt = 0.002;
  c.params.n_pre_max = 8;
  c.params.dt_pre_s = 0.05;
  c.params.n_stop = 7;
  c.params.dt_stop_s = 0.05;
  c.params.n_stop_blocks = 4;
  c.params.stop_block_sizes = {1, 1, 2, 3};
  c.params.core = rig.params;
  return c;
}

TEST(SegmentCapacity, TheDockingPlannerConfiguresOnSeventeenJoints) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  ASSERT_EQ(tree.model->nv, kTreeNv);
  const DockingConfig c = DockingConfigOf(tree);
  MpcDockingSegmentPlanner planner;
  std::string err;
  EXPECT_TRUE(planner.Configure(c.model, c.consts, c.params, &StepClock, &err)) << err;
  EXPECT_TRUE(planner.Configured());
}

TEST(SegmentCapacity, TheDockingPlannerRefusesAModelPastTheCapacity) {
  const fx::ArmModel wide = TreeOverCapacity();
  ASSERT_EQ(wide.model->nv, kMaxSegmentNv + 1);
  const DockingConfig c = DockingConfigOf(wide);
  MpcDockingSegmentPlanner planner;
  std::string err;
  EXPECT_FALSE(planner.Configure(c.model, c.consts, c.params, &StepClock, &err));
  EXPECT_NE(err.find("capacity"), std::string::npos) << err;
  EXPECT_FALSE(planner.Configured());
}

// ── 3. The searches, on the chain a catch is searched on ─────────────────────

// The search's inputs for a chain: the docking fixture's cores, the suites'
// lattice and grid.
struct SearchConfig {
  std::shared_ptr<const pinocchio::Model> arm;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle;
  NlpCatchSearchModel model{};
  NlpCatchSearchConstants constants{};
  NlpCatchSearchParams params{};
  rtc::catching::CatchPoseIkOptions ik{};
};

SearchConfig SearchConfigOf(const fx::ArmModel& a) {
  const dk::Rig rig = dk::MakeRig(a);
  SearchConfig c;
  c.arm = a.model;
  c.handle = std::make_unique<rtc_urdf_bridge::RtModelHandle>(a.model);
  const int nv = a.model->nv;
  c.model.arm = a.model;
  c.model.handle = c.handle.get();
  c.model.catch_frame = a.frame;
  c.model.nv = nv;
  c.params.wait_pose_n = nv;
  for (int m = 0; m < nv; ++m) {
    c.model.device_of_model[U(m)] = DeviceOf(m, nv);
    c.params.wait_pose[U(DeviceOf(m, nv))] = a.q_nominal[m];
  }
  c.constants.t_arm_s = 0.0;
  c.constants.control_dt = 0.002;
  c.constants.t_close_lead = 0.1;
  c.params.cand_dt = 0.04;
  c.params.t_lead_min = 0.1;
  c.params.t_max = 0.4;
  c.params.cand_capacity = 16;
  c.params.n_pre_min = 2;
  c.params.n_pre_max = 8;
  c.params.dt_pre = 0.05;
  c.params.n_stop = 7;
  c.params.dt_stop = 0.05;
  c.params.n_stop_blocks = 4;
  c.params.stop_block_sizes = {1, 1, 2, 3};
  c.params.budget_s = 0.04;
  c.params.solve_budget_s = 0.004;
  c.params.start_lead_s = 0.004;
  c.params.max_solves = 8;
  c.params.w_time = 0.0;
  c.params.w_switch = 0.0;
  c.params.t_ref_s = 0.5;
  c.params.rank_w_q = 1.0;
  c.params.core = rig.params;
  c.params.limits = rig.limits;
  c.ik.max_iter = 60;
  c.ik.manipulability_min = 0.0;
  return c;
}

TEST(SegmentCapacity, TheNlpSearchConfiguresOnATenJointChain) {
  // The trunk and one arm: wider than the capacity the search used to refuse
  // above (8), within its catch-pose IK's.
  const fx::ArmModel chain = SyntheticTree(kTrunk, kArm, 1);
  ASSERT_EQ(chain.model->nv, kChainNv);
  const SearchConfig c = SearchConfigOf(chain);
  NlpCatchSearch search;
  std::string err;
  EXPECT_TRUE(search.Configure(c.model, c.constants, c.params, c.ik, &StepClock, &err)) << err;
  EXPECT_TRUE(search.Configured());
}

TEST(SegmentCapacity, TheNlpSearchRefusesAChainPastItsLimit) {
  // One joint past kNlpMaxSearchNv — the smaller of the payload's capacity
  // and the catch-pose IK's. Past the IK's, CatchPoseIk would stay unsized and
  // answer kModelInvalid to every candidate: the search refuses at configure,
  // naming its limit, instead of never finding a catch.
  const int nv = kNlpMaxSearchNv + 1;
  const fx::ArmModel chain = SyntheticTree(nv - kArm, kArm, 1);
  ASSERT_EQ(chain.model->nv, nv);
  const SearchConfig c = SearchConfigOf(chain);
  NlpCatchSearch search;
  std::string err;
  EXPECT_FALSE(search.Configure(c.model, c.constants, c.params, c.ik, &StepClock, &err));
  EXPECT_NE(err.find("kNlpMaxSearchNv"), std::string::npos) << err;
  EXPECT_NE(err.find(std::to_string(kNlpMaxSearchNv)), std::string::npos) << err;
  EXPECT_FALSE(search.Configured());
}

// The grid search's inputs for a chain (the suites' values).
struct GridConfig {
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle;
  GridCatchSearchModel model{};
  GridCatchSearchConstants constants{};
  PlannerParams params{};
  rtc::catching::CatchPoseIkOptions ik{};
};

GridConfig GridConfigOf(const fx::ArmModel& a) {
  GridConfig c;
  c.handle = std::make_unique<rtc_urdf_bridge::RtModelHandle>(a.model);
  const int nv = a.model->nv;
  c.model.handle = c.handle.get();
  c.model.catch_frame = a.frame;
  c.model.nv = nv;
  c.model.accel_box = true;
  c.params.enabled = true;
  c.params.wait_pose_n = nv;
  for (int m = 0; m < nv; ++m) {
    c.model.device_of_model[U(m)] = DeviceOf(m, nv);
    c.model.qdot_max[U(m)] = 3.14;
    c.model.qddot_max[U(m)] = 20.0;
    c.params.wait_pose[U(DeviceOf(m, nv))] = a.q_nominal[m];
  }
  c.constants.eta_v = 0.9;
  c.constants.v_max = 3.0;
  c.constants.a_dec = 10.0;
  c.constants.t_arm_s = 0.0;
  c.constants.t_close_lead = 0.1;
  c.constants.t_close_total = 0.101;
  c.constants.ball_mass = 0.057;
  c.constants.ref_omega = 10.0;
  c.constants.ref_zeta = 1.0;
  c.constants.ref_a_max = 21.0;
  c.constants.control_dt = 0.002;
  c.params.t_freeze = 0.2;
  c.params.slice_t_max = 0.95;
  c.params.n_settle = 0;
  c.params.d_eff = 0.2;
  c.params.r_cap = 0.03;
  c.params.max_ik = 8;
  c.params.budget_s = 0.05;
  c.ik.max_iter = 60;
  c.ik.manipulability_min = 0.0;
  return c;
}

TEST(SegmentCapacity, TheGridSearchConfiguresOnATenJointChain) {
  const fx::ArmModel chain = SyntheticTree(kTrunk, kArm, 1);
  ASSERT_EQ(chain.model->nv, kChainNv);
  const GridConfig c = GridConfigOf(chain);
  GridCatchSearch search;
  EXPECT_TRUE(search.Configure(c.model, c.constants, c.params, c.ik, &StepClock));
  EXPECT_TRUE(search.Configured());
}

TEST(SegmentCapacity, TheGridSearchRefusesAChainItsIkCannotBeSizedFor) {
  // The grid search's own bound is kMaxPlanNv; its catch-pose IK's is lower.
  // A chain between the two used to configure and then fail every candidate's
  // IK — it is refused at configure.
  const int nv = rtc::kMaxRobotDOF + 1;
  if (nv > kMaxPlanNv) {
    GTEST_SKIP() << "the IK takes every model the search does: nothing to refuse";
  }
  const fx::ArmModel chain = SyntheticTree(nv - kArm, kArm, 1);
  ASSERT_EQ(chain.model->nv, nv);
  const GridConfig c = GridConfigOf(chain);
  GridCatchSearch search;
  EXPECT_FALSE(search.Configure(c.model, c.constants, c.params, c.ik, &StepClock));
  EXPECT_FALSE(search.Configured());
}

}  // namespace
