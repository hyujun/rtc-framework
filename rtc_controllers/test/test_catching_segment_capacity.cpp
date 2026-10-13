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
//   3. the nlp search configures on the chain the catch is searched on (the
//      trunk and one arm, 10 joints — wider than the old capacity), and
//      refuses a chain its catch-pose IK cannot be sized for.
// The grid search has no check against this constant (its model is bounded by
// kMaxPlanNv and its IK), so it has no case here.
//
// Mutation: with kMaxSegmentNv at 16 every "17 joints" case here fails.
#include "rtc_base/types/types.hpp"
#include "rtc_controllers/catching/catch_pose_ik.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_planner.hpp"
#include "rtc_controllers/catching/mpc_segment_planner.hpp"
#include "rtc_controllers/catching/nlp_catch_search.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"
#include "rtc_controllers/testing/mpc_segment_core_fixture.hpp"
#include "rtc_controllers/testing/synthetic_tree_fixture.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace {

using rtc::catching::kMaxPlanNv;
using rtc::catching::kMaxSegmentNodes;
using rtc::catching::kMaxSegmentNv;
using rtc::catching::MpcDockingSegmentPlanner;
using rtc::catching::MpcDockingSegmentPlannerConstants;
using rtc::catching::MpcDockingSegmentPlannerModel;
using rtc::catching::MpcDockingSegmentPlannerParams;
using rtc::catching::MpcSegmentPlanner;
using rtc::catching::MpcSegmentPlannerConstants;
using rtc::catching::MpcSegmentPlannerModel;
using rtc::catching::MpcSegmentPlannerParams;
using rtc::catching::NlpCatchSearch;
using rtc::catching::NlpCatchSearchConstants;
using rtc::catching::NlpCatchSearchModel;
using rtc::catching::NlpCatchSearchParams;
using rtc::catching::NodeTrajectoryFollower;
using rtc::catching::SegmentNodeSample;
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

constexpr std::int64_t kDtNs = 25'000'000;
constexpr std::int64_t kTc = 1'727'000'000'123'456'789LL;

std::atomic<std::int64_t> g_clock{0};

std::int64_t StepClock() noexcept {
  return g_clock.fetch_add(1000, std::memory_order_relaxed) + 1000;
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
    pm.qdot_max[U(m)] = a.model->velocityLimit[m];
    pm.tau_max[U(m)] = a.model->effortLimit[m];
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
  c.t_arm_s = 0.02;
  c.control_dt = 0.002;
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

TEST(SegmentCapacity, TheMpcPlannerRefusesAModelPastTheCapacity) {
  const fx::ArmModel wide = TreeOverCapacity();
  ASSERT_EQ(wide.model->nv, kMaxSegmentNv + 1);
  MpcSegmentPlanner planner;
  std::string err;
  EXPECT_FALSE(planner.Configure(MpcModelOf(wide), MpcConsts(), MpcParams(), &StepClock, &err));
  EXPECT_NE(err.find("kMaxSegmentNv"), std::string::npos) << err;
  EXPECT_FALSE(planner.Configured());
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

// ── 3. The nlp search, on the chain a catch is searched on ───────────────────

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
  ASSERT_LE(kChainNv, rtc::kMaxRobotDOF);
  const SearchConfig c = SearchConfigOf(chain);
  NlpCatchSearch search;
  std::string err;
  EXPECT_TRUE(search.Configure(c.model, c.constants, c.params, c.ik, &StepClock, &err)) << err;
  EXPECT_TRUE(search.Configured());
}

TEST(SegmentCapacity, TheNlpSearchRefusesAChainItsIkCannotBeSizedFor) {
  // Within the payload's capacity but past the catch-pose IK's: CatchPoseIk
  // would stay unsized and answer kModelInvalid to every candidate, so the
  // search refuses at configure, by name, instead of never finding a catch.
  const int nv = rtc::kMaxRobotDOF + 1;
  ASSERT_LE(nv, kMaxSegmentNv) << "this case needs a payload wider than the IK";
  const fx::ArmModel chain = SyntheticTree(nv - kArm, kArm, 1);
  ASSERT_EQ(chain.model->nv, nv);
  const SearchConfig c = SearchConfigOf(chain);
  NlpCatchSearch search;
  std::string err;
  EXPECT_FALSE(search.Configure(c.model, c.constants, c.params, c.ik, &StepClock, &err));
  EXPECT_NE(err.find("kMaxRobotDOF"), std::string::npos) << err;
  EXPECT_FALSE(search.Configured());
}

}  // namespace
