// The segment planners with joint groups (`planner.groups`) on
// a synthetic tree whose device order is the REVERSE of the model's — a reader
// that skipped device_of_model would show on every joint but the middle one.
// Judged: each model joint's group, the partition each core is built with, the
// fold of groups without keys into the problem without groups, the refusals,
// and a first segment planned with per-group partitions.
#include "rtc_controllers/catching/docking_params.hpp"
#include "rtc_controllers/catching/joint_groups.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_planner.hpp"
#include "rtc_controllers/catching/mpc_segment_planner.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_controllers/catching/transition_table.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"
#include "rtc_controllers/testing/mpc_segment_core_fixture.hpp"
#include "rtc_controllers/testing/synthetic_tree_fixture.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace {

namespace fx = rtc::testing::mpc_segment_core;
using rtc::catching::BlockPattern;
using rtc::catching::JointGroups;
using rtc::catching::JointGroupSpec;
using rtc::catching::kMaxSegmentNv;
using rtc::catching::MpcSegmentBallTarget;
using rtc::catching::MpcSegmentPlanner;
using rtc::catching::MpcSegmentPlannerConstants;
using rtc::catching::MpcSegmentPlannerModel;
using rtc::catching::MpcSegmentPlannerParams;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanSnapshot;
using rtc::catching::SegmentOutcomeName;
using rtc::catching::SegmentRecord;
using rtc::catching::SegmentSnapshot;
using rtc::catching::ValidateSegmentNodes;
using rtc::testing::SyntheticTree;

constexpr int kTrunk = 3;
constexpr int kArm = 7;
constexpr int kNv = kTrunk + 2 * kArm;  // 17

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kT0 = 1'727'000'000'000'000'000LL;
constexpr std::int64_t kTArm = 30 * kMs;
constexpr std::int64_t kH = 2 * kMs;

std::atomic<std::int64_t> g_now{kT0};

std::int64_t FixedClock() noexcept {
  return g_now.load(std::memory_order_relaxed);
}

std::size_t U(int i) {
  return static_cast<std::size_t>(i);
}

int DeviceOf(int m) {
  return kNv - 1 - m;
}

// The model's parts: trunk 0..2, the catch arm 3..9, the other arm 10..16.
int PartOf(int m) {
  return m < kTrunk ? 0 : (m < kTrunk + kArm ? 1 : 2);
}

// The device's joint names: "d<device index>".
std::vector<std::string> DeviceJoints() {
  std::vector<std::string> names;
  for (int d = 0; d < kNv; ++d) {
    names.push_back("d" + std::to_string(d));
  }
  return names;
}

// Groups declared on DEVICE names, in an order that is none of the parts':
// "other" (the other arm), "trunk", "reach" (the catch arm).
std::vector<JointGroupSpec> Specs() {
  std::vector<JointGroupSpec> specs = {{"other", {}}, {"trunk", {}}, {"reach", {}}};
  for (int m = 0; m < kNv; ++m) {
    const int part = PartOf(m);
    specs[U(part == 2 ? 0 : (part == 0 ? 1 : 2))].joints.push_back("d" +
                                                                   std::to_string(DeviceOf(m)));
  }
  return specs;
}

// Group index (in Specs()) of model joint m.
int GroupOfModelJoint(int m) {
  const int part = PartOf(m);
  return part == 2 ? 0 : (part == 0 ? 1 : 2);
}

MpcSegmentPlannerModel ModelOf(const fx::ArmModel& a, bool groups) {
  MpcSegmentPlannerModel pm;
  pm.arm = a.model;
  pm.catch_frame = a.frame;
  pm.nv = a.model->nv;
  for (int m = 0; m < pm.nv; ++m) {
    pm.device_of_model[U(m)] = DeviceOf(m);
    pm.q_min[U(m)] = a.model->lowerPositionLimit[m];
    pm.q_max[U(m)] = a.model->upperPositionLimit[m];
    pm.qdot_max[U(m)] = a.model->upperVelocityLimit[m];
    pm.tau_max[U(m)] = a.model->upperEffortLimit[m];
  }
  if (groups) {
    std::string why;
    const std::vector<JointGroupSpec> specs = Specs();
    const std::vector<std::string> joints = DeviceJoints();
    EXPECT_TRUE(rtc::catching::ResolveJointGroups(specs, joints, pm.groups, &why)) << why;
  }
  return pm;
}

// The shipped approach profile's shape (7 × 0.05 s [1, 1, 2, 3] after the
// catch, up to 6 × 0.1 s before it, k_max 2).
MpcSegmentPlannerParams Params() {
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

// Groups with keys: the other arm coarse before the catch and after it, the
// trunk coarse before it; the catch arm keeps the block's own partition.
MpcSegmentPlannerParams GroupedParams() {
  MpcSegmentPlannerParams p = Params();
  p.n_groups = 3;
  p.groups[0].pre_block = 3;
  p.groups[0].stop_blocks = {2, 2, 3};
  p.groups[0].n_stop_blocks = 3;
  p.groups[1].pre_block = 2;
  return p;
}

MpcSegmentPlannerConstants Consts() {
  MpcSegmentPlannerConstants c;
  c.eta_v = 0.9;
  c.t_arm_s = static_cast<double>(kTArm) * 1e-9;
  c.control_dt = static_cast<double>(kH) * 1e-9;
  c.v_eps = 1e-6;
  return c;
}

// The rule written out here (not the planner's code): n_pre nodes in blocks
// of `per`, the remainder joining the last; then `stop` with its largest
// trailing block shrunk k times.
BlockPattern Expected(int n_pre, int per, std::vector<int> stop, int k) {
  BlockPattern b;
  if (n_pre > 0) {
    const int count = std::max(1, n_pre / per);
    for (int i = 0; i < count; ++i) {
      b.block_sizes[U(b.n_blocks++)] = i + 1 < count ? per : n_pre - per * (count - 1);
    }
  }
  for (int step = 0; step < k; ++step) {
    std::size_t largest = 0;
    for (std::size_t i = 1; i < stop.size(); ++i) {
      if (stop[i] >= stop[largest]) {
        largest = i;
      }
    }
    if (--stop[largest] == 0) {
      stop.erase(stop.begin() + static_cast<std::ptrdiff_t>(largest));
    }
  }
  for (const int s : stop) {
    b.block_sizes[U(b.n_blocks++)] = s;
  }
  return b;
}

TEST(SegmentGroupsMpc, EachModelJointsGroupIsReadThroughDeviceOfModel) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  MpcSegmentPlanner planner;
  std::string err;
  ASSERT_TRUE(planner.Configure(ModelOf(tree, true), Consts(), GroupedParams(), &FixedClock, &err))
      << err;
  for (int m = 0; m < kNv; ++m) {
    EXPECT_EQ(planner.GroupOfModel(m), GroupOfModelJoint(m)) << "model joint " << m;
    for (int j = 1; j <= 6; ++j) {
      EXPECT_EQ(planner.ApproachCoreParams(j).joint_group[U(m)], GroupOfModelJoint(m));
    }
  }
}

TEST(SegmentGroupsMpc, EveryCoreIsBuiltWithEachGroupsOwnPartition) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  MpcSegmentPlanner planner;
  std::string err;
  ASSERT_TRUE(planner.Configure(ModelOf(tree, true), Consts(), GroupedParams(), &FixedClock, &err))
      << err;
  const std::array<int, 3> per = {3, 2, 1};
  const std::array<std::vector<int>, 3> stop = {
      std::vector<int>{2, 2, 3}, std::vector<int>{1, 1, 2, 3}, std::vector<int>{1, 1, 2, 3}};
  for (int g = 0; g < 3; ++g) {
    SCOPED_TRACE("group " + std::to_string(g));
    for (int j = 1; j <= 6; ++j) {
      const rtc::catching::MpcSegmentCoreParams& cp = planner.ApproachCoreParams(j);
      EXPECT_EQ(cp.n_groups, 3);
      EXPECT_TRUE(cp.group_blocks[U(g)] == Expected(j, per[U(g)], stop[U(g)], 0))
          << "catch core n_pre = " << j;
    }
    for (int k = 0; k <= 2; ++k) {
      EXPECT_TRUE(planner.StopCoreParams(k).group_blocks[U(g)] ==
                  Expected(0, per[U(g)], stop[U(g)], k))
          << "stop core k = " << k;
    }
  }
  // Three partitions are three block classes on the catch core of 6 nodes
  // (the trunk's [2, 2, 2] and the catch arm's [1 × 6] differ before the
  // catch, the other arm's after it too).
  EXPECT_EQ(planner.ApproachCore(6).Layout().n_classes, 3);
  // n_pre = 1: every group's pre-catch part is one block, so the trunk's and
  // the catch arm's partitions coincide and share a class.
  EXPECT_EQ(planner.ApproachCore(1).Layout().n_classes, 2);
}

TEST(SegmentGroupsMpc, GroupsWithoutKeysAreTheProblemWithoutGroups) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  MpcSegmentPlannerParams p = Params();
  p.n_groups = 3;  // declared, no key of their own
  MpcSegmentPlanner planner;
  std::string err;
  ASSERT_TRUE(planner.Configure(ModelOf(tree, true), Consts(), p, &FixedClock, &err)) << err;
  for (int j = 1; j <= 6; ++j) {
    EXPECT_TRUE(planner.ApproachCore(j).Layout().Single()) << j;
  }
  for (int k = 0; k <= 2; ++k) {
    EXPECT_TRUE(planner.Core(k).Layout().Single()) << k;
  }
}

TEST(SegmentGroupsMpc, RefusesGroupsTheModelAndTheProfileDisagreeOn) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  std::string err;
  {
    MpcSegmentPlanner planner;
    EXPECT_FALSE(planner.Configure(ModelOf(tree, true), Consts(), Params(), &FixedClock, &err));
    EXPECT_NE(err.find("joint groups (3) are not the ones planner.segment.mpc was parsed with (0)"),
              std::string::npos)
        << err;
  }
  {
    MpcSegmentPlanner planner;
    EXPECT_FALSE(
        planner.Configure(ModelOf(tree, false), Consts(), GroupedParams(), &FixedClock, &err));
    EXPECT_NE(err.find("joint groups (0)"), std::string::npos) << err;
  }
  // The control.
  MpcSegmentPlanner planner;
  EXPECT_TRUE(planner.Configure(ModelOf(tree, false), Consts(), Params(), &FixedClock, &err))
      << err;
}

TEST(SegmentGroupsMpc, PlansASeventeenJointSegmentWithPerGroupPartitions) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  MpcSegmentPlanner planner;
  std::string err;
  g_now.store(kT0);
  ASSERT_TRUE(planner.Configure(ModelOf(tree, true), Consts(), GroupedParams(), &FixedClock, &err))
      << err;
  // A catch the trunk and the catch arm reach a little away from where the
  // tree rests: the ball passes the catch frame along its −z.
  Eigen::VectorXd q_catch = tree.q_nominal;
  for (int m = 0; m < kTrunk + kArm; ++m) {
    q_catch[m] += m % 2 == 0 ? 0.04 : -0.04;
  }
  pinocchio::Data data(*tree.model);
  pinocchio::forwardKinematics(*tree.model, data, q_catch);
  pinocchio::updateFramePlacement(*tree.model, data, tree.frame);
  const Eigen::Vector3d p_c = data.oMf[tree.frame].translation();
  const Eigen::Vector3d a_d = data.oMf[tree.frame].rotation().col(2);
  const Eigen::Vector3d v_c = -5.0 * a_d;
  PlanSnapshot plan{};
  plan.valid = true;
  plan.plan_id = 7;
  plan.t_c_ns = kT0 + kTArm + 800 * kMs;
  plan.token.activation_generation = 3;
  plan.token.generation = 5;
  plan.nv = kNv;
  plan.joint_set = rtc::catching::FullJointSet(kNv);
  PlannerRtState rt{};
  rt.valid = true;
  rt.activation_generation = 3;
  rt.rt_iteration = 1000;
  rt.rt_state_ns = kT0 - kH;
  rt.reset_epoch = 1;
  rt.mode = static_cast<std::uint8_t>(rtc::catching::Mode::kTracking);
  rt.nv = kNv;
  rt.cmd_seeded = true;
  rt.track_seen = true;
  rt.track_generation = 5;
  for (int m = 0; m < kNv; ++m) {
    plan.q_star[U(DeviceOf(m))] = q_catch[m];
    rt.q_cmd[U(DeviceOf(m))] = tree.q_nominal[m];
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
  SegmentSnapshot first{};
  SegmentRecord rec{};
  ASSERT_TRUE(planner.PlanFirst(rt, plan, ball, first, rec))
      << SegmentOutcomeName(rec.outcome) << " / " << rec.core_reason_name;
  ASSERT_TRUE(ValidateSegmentNodes(first));
  ASSERT_GE(first.n_pre, 2) << "a pre-catch part the coarse blocks act on";
  EXPECT_GE(planner.ApproachCore(first.n_pre).Layout().n_classes, 2);
  // The catch node reaches the ball: the planner's own gate.
  Eigen::VectorXd q(kNv);
  for (int m = 0; m < kNv; ++m) {
    q[m] = first.q[U(first.n_pre * kMaxSegmentNv + DeviceOf(m))];
  }
  pinocchio::forwardKinematics(*tree.model, data, q);
  pinocchio::updateFramePlacement(*tree.model, data, tree.frame);
  EXPECT_LE((data.oMf[tree.frame].translation() - p_c).norm(), 0.02);
}

// ── The velocity and rest weights, spread over the groups' joints ───────────

TEST(SegmentGroupsMpc, EachJointTakesItsGroupsWeightsAndTheWaitPosesEntry) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  MpcSegmentPlannerModel model = ModelOf(tree, true);
  for (int d = 0; d < kNv; ++d) {
    model.rest_pose[U(d)] = 0.3 + 0.01 * d;  // per DEVICE joint, all different
  }
  model.rest_pose_n = kNv;
  MpcSegmentPlannerParams p = GroupedParams();
  p.groups[0].w_rest = 2.0;  // the other arm
  p.groups[1].w_vel = 0.3;   // the trunk
  p.groups[2].w_vel = 0.05;  // the catch arm
  MpcSegmentPlanner planner;
  std::string err;
  ASSERT_TRUE(planner.Configure(model, Consts(), p, &FixedClock, &err)) << err;
  const std::array<double, 3> w_vel = {0.0, 0.3, 0.05};
  const std::array<double, 3> w_rest = {2.0, 0.0, 0.0};
  for (const rtc::catching::MpcSegmentCoreParams* cp :
       {&planner.ApproachCoreParams(1), &planner.ApproachCoreParams(6),
        &planner.StopCoreParams(2)}) {
    ASSERT_EQ(cp->w_vel.size(), kNv);
    ASSERT_EQ(cp->w_rest.size(), kNv);
    ASSERT_EQ(cp->q_rest.size(), kNv);
    for (int m = 0; m < kNv; ++m) {
      const auto g = U(GroupOfModelJoint(m));
      EXPECT_EQ(cp->w_vel[m], w_vel[g]) << "model joint " << m;
      EXPECT_EQ(cp->w_rest[m], w_rest[g]) << "model joint " << m;
      EXPECT_EQ(cp->q_rest[m], model.rest_pose[U(DeviceOf(m))]) << "model joint " << m;
    }
  }
  EXPECT_TRUE(planner.ApproachCore(3).RestTermActive());
  EXPECT_TRUE(planner.ApproachCore(3).VelocityTermActive());
}

TEST(SegmentGroupsMpc, NoWeightLeavesTheCoresWithoutTheTermsAndARestWeightNeedsTheWaitPose) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  std::string err;
  {
    MpcSegmentPlanner planner;
    ASSERT_TRUE(
        planner.Configure(ModelOf(tree, true), Consts(), GroupedParams(), &FixedClock, &err))
        << err;
    EXPECT_EQ(planner.ApproachCoreParams(2).w_vel.size(), 0);
    EXPECT_EQ(planner.ApproachCoreParams(2).w_rest.size(), 0);
    EXPECT_FALSE(planner.ApproachCore(2).RestTermActive());
    EXPECT_FALSE(planner.ApproachCore(2).VelocityTermActive());
  }
  MpcSegmentPlannerParams p = GroupedParams();
  p.groups[0].w_rest = 1.0;
  MpcSegmentPlanner planner;
  EXPECT_FALSE(planner.Configure(ModelOf(tree, true), Consts(), p, &FixedClock, &err));
  EXPECT_NE(err.find("w_rest > 0 needs planner.wait_pose (0 entries, the arm has 17)"),
            std::string::npos)
      << err;
}

// ── The docking planner ──────────────────────────────────────────────────────

namespace dk = rtc::testing::mpc_docking;
using rtc::catching::MpcDockingSegmentCoreParams;
using rtc::catching::MpcDockingSegmentPlanner;
using rtc::catching::MpcDockingSegmentPlannerConstants;
using rtc::catching::MpcDockingSegmentPlannerModel;
using rtc::catching::MpcDockingSegmentPlannerParams;

std::int64_t StepClock() noexcept {
  static std::atomic<std::int64_t> clock{0};
  return clock.fetch_add(1000, std::memory_order_relaxed) + 1000;
}

struct DockingConfig {
  MpcDockingSegmentPlannerModel model;
  MpcDockingSegmentPlannerConstants consts;
  MpcDockingSegmentPlannerParams params;
};

// The capacity suite's docking inputs (the docking fixture's cores, 8 × 0.05 s
// before the catch, a 7-node stop in four blocks) on the reversed device.
DockingConfig DockingConfigOf(const fx::ArmModel& a, bool groups) {
  const dk::Rig rig = dk::MakeRig(a);
  DockingConfig c;
  c.model.arm = a.model;
  c.model.catch_frame = a.frame;
  c.model.nv = a.model->nv;
  c.model.limits = rig.limits;
  for (int m = 0; m < c.model.nv; ++m) {
    c.model.device_of_model[U(m)] = DeviceOf(m);
    c.model.qd_rating[U(m)] = 2.0 * rig.limits.qd_max[m];
    c.model.warm_pose[U(DeviceOf(m))] = a.q_nominal[m];
  }
  if (groups) {
    std::string why;
    const std::vector<JointGroupSpec> specs = Specs();
    const std::vector<std::string> joints = DeviceJoints();
    EXPECT_TRUE(rtc::catching::ResolveJointGroups(specs, joints, c.model.groups, &why)) << why;
    c.params.n_groups = 3;
  }
  c.consts.t_arm_s = 0.0;
  c.consts.control_dt = 0.002;
  c.params.n_pre_max = 3;
  c.params.dt_pre_s = 0.05;
  c.params.n_stop = 7;
  c.params.dt_stop_s = 0.05;
  c.params.n_stop_blocks = 4;
  c.params.stop_block_sizes = {1, 1, 2, 3};
  c.params.core = rig.params;
  return c;
}

bool SameVector(const Eigen::VectorXd& x, const Eigen::VectorXd& y) {
  return x.size() == y.size() && (x.size() == 0 || x == y);
}

TEST(SegmentGroupsDocking, GroupsWithoutKeysLeaveTheCoresAsTheyWere) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  const DockingConfig plain = DockingConfigOf(tree, false);
  const DockingConfig grouped = DockingConfigOf(tree, true);
  MpcDockingSegmentPlanner a, b;
  std::string err;
  ASSERT_TRUE(a.Configure(plain.model, plain.consts, plain.params, &StepClock, &err)) << err;
  ASSERT_TRUE(b.Configure(grouped.model, grouped.consts, grouped.params, &StepClock, &err)) << err;
  for (int j = 1; j <= 3; ++j) {
    const MpcDockingSegmentCoreParams& pa = a.CoreParamsFor(j);
    const MpcDockingSegmentCoreParams& pb = b.CoreParamsFor(j);
    EXPECT_TRUE(SameVector(pa.r_tau, pb.r_tau));
    EXPECT_TRUE(SameVector(pa.r_jerk, pb.r_jerk));
    EXPECT_TRUE(SameVector(pa.r_jerk_stop, pb.r_jerk_stop));
    EXPECT_TRUE(SameVector(pa.w_q_nom, pb.w_q_nom));
    EXPECT_EQ(pb.w_vel.size(), 0);
    EXPECT_EQ(pb.w_rest_stop.size(), 0);
    EXPECT_FALSE(b.CoreFor(j).VelocityTermActive());
  }
  for (int m = 0; m < kNv; ++m) {
    EXPECT_EQ(b.GroupOfModel(m), GroupOfModelJoint(m)) << m;
  }
}

TEST(SegmentGroupsDocking, EachJointTakesItsGroupsWeights) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  DockingConfig c = DockingConfigOf(tree, true);
  // "other" (0): held at the stop and before it, no torque cost; "trunk" (1):
  // damped, its own jerk weights; "reach" (2): nothing of its own.
  c.params.groups[0].w_rest_stop = 2.0;
  c.params.groups[0].w_q_nom = 3.0;
  c.params.groups[0].r_tau = 0.0;
  c.params.groups[1].w_vel = 0.4;
  c.params.groups[1].r_jerk = 5.0;
  c.params.groups[1].r_jerk_stop = 6.0;
  MpcDockingSegmentPlanner planner;
  std::string err;
  ASSERT_TRUE(planner.Configure(c.model, c.consts, c.params, &StepClock, &err)) << err;
  const MpcDockingSegmentCoreParams& base = c.params.core;  // uniform per joint
  for (int j = 1; j <= 3; ++j) {
    const MpcDockingSegmentCoreParams& cp = planner.CoreParamsFor(j);
    ASSERT_EQ(cp.w_vel.size(), kNv);
    ASSERT_EQ(cp.w_rest_stop.size(), kNv);
    ASSERT_EQ(cp.r_tau.size(), kNv);
    for (int m = 0; m < kNv; ++m) {
      SCOPED_TRACE("model joint " + std::to_string(m));
      const int g = GroupOfModelJoint(m);
      EXPECT_EQ(cp.w_vel[m], g == 1 ? 0.4 : 0.0);
      EXPECT_EQ(cp.w_rest_stop[m], g == 0 ? 2.0 : 0.0);
      EXPECT_EQ(cp.w_q_nom[m], g == 0 ? 3.0 : base.w_q_nom[m]);
      EXPECT_EQ(cp.r_tau[m], g == 0 ? 0.0 : base.r_tau[m]);
      EXPECT_EQ(cp.r_jerk[m], g == 1 ? 5.0 : base.r_jerk[m]);
      EXPECT_EQ(cp.r_jerk_stop[m], g == 1 ? 6.0 : base.r_jerk[m]) << "empty = r_jerk";
    }
    EXPECT_TRUE(planner.CoreFor(j).VelocityTermActive());
    EXPECT_TRUE(planner.CoreFor(j).StopRestTermActive());
  }
  // Every group's torque weight 0: the core's "off", an empty vector.
  for (auto& gp : c.params.groups) {
    gp.r_tau = 0.0;
  }
  MpcDockingSegmentPlanner off;
  ASSERT_TRUE(off.Configure(c.model, c.consts, c.params, &StepClock, &err)) << err;
  EXPECT_EQ(off.CoreParamsFor(1).r_tau.size(), 0);
}

TEST(SegmentGroupsDocking, RefusesGroupsTheModelAndTheProfileDisagreeOn) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  DockingConfig c = DockingConfigOf(tree, true);
  c.params.n_groups = 0;
  MpcDockingSegmentPlanner planner;
  std::string err;
  EXPECT_FALSE(planner.Configure(c.model, c.consts, c.params, &StepClock, &err));
  EXPECT_NE(err.find("joint groups (3) are not the ones planner.segment.mpc_docking was parsed "
                     "with (0)"),
            std::string::npos)
      << err;
}

TEST(SegmentGroupsDocking, EveryCoreIsBuiltWithEachGroupsOwnPartition) {
  const fx::ArmModel tree = SyntheticTree(kTrunk, kArm, 2);
  DockingConfig c = DockingConfigOf(tree, true);
  c.params.groups[0].pre_block = 2;  // the other arm: coarse before the catch and after it
  c.params.groups[0].stop_blocks = {2, 2, 3};
  c.params.groups[0].n_stop_blocks = 3;
  MpcDockingSegmentPlanner planner;
  std::string err;
  ASSERT_TRUE(planner.Configure(c.model, c.consts, c.params, &StepClock, &err)) << err;
  for (int j = 1; j <= 3; ++j) {
    const MpcDockingSegmentCoreParams& cp = planner.CoreParamsFor(j);
    EXPECT_EQ(cp.n_groups, 3);
    EXPECT_TRUE(cp.group_blocks[0] == Expected(j, 2, {2, 2, 3}, 0)) << "n_pre = " << j;
    EXPECT_TRUE(cp.group_blocks[1] == Expected(j, 1, {1, 1, 2, 3}, 0)) << "n_pre = " << j;
    EXPECT_TRUE(cp.group_blocks[2] == Expected(j, 1, {1, 1, 2, 3}, 0)) << "n_pre = " << j;
    for (int m = 0; m < kNv; ++m) {
      EXPECT_EQ(cp.joint_group[U(m)], GroupOfModelJoint(m));
    }
    EXPECT_EQ(planner.CoreFor(j).Layout().n_classes, 2) << "trunk and catch arm share one";
  }
}

}  // namespace
