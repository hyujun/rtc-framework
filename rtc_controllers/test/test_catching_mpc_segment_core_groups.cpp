// MpcSegmentCore with joint groups — per-group move blocking on a synthetic tree (a trunk and two
// arms off it, 17 joints, the catch frame on the first arm). The judged properties:
//   • groups that all have one partition are the problem without groups, to
//     the bit (the layout folds them into one block class);
//   • with different partitions the structured assembly matches the dense
//     oracle, the stage gains match the integral, the terminal equality keeps
//     its rank and every joint's jerk is constant on ITS group's blocks;
//   • a partition is checked per group (catch node, three blocks after it);
//   • the per-joint velocity and rest terms: off is the QP without them, their
//     constant Hessian is on every path of H (w_delta_scale 0, between, 1),
//     their reported value is the direct sum at the solution, they leave an
//     exact stop at zero and do not depend on what was solved before;
//   • the angular-momentum rate: its linearisation against pinocchio's other
//     centroidal algorithm and a finite difference (after a second
//     linearisation of the same core), its schedule and reported value
//     against a direct sum, its rate against the time derivative of the
//     solution's angular momentum, the condition that linearises for it
//     alone, and no allocation.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_controllers/catching/mpc_block_grid.hpp"
#include "rtc_controllers/catching/mpc_segment_core.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/mpc_segment_core_fixture.hpp"
#include "rtc_controllers/testing/synthetic_tree_fixture.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace {

using rtc::catching::BlockPattern;
using rtc::catching::MpcSegmentCore;
using rtc::catching::MpcSegmentCoreInput;
using rtc::catching::MpcSegmentCoreParams;
using rtc::catching::MpcSegmentCoreReason;
using rtc::catching::MpcSegmentCoreReasonName;
using rtc::catching::MpcSegmentCoreResult;
using rtc::testing::SyntheticTree;
using rtc::testing::mpc_segment_core::ArmModel;
using rtc::testing::mpc_segment_core::LimitsFromModel;
using rtc::testing::mpc_segment_core::RestInput;
using rtc::testing::mpc_segment_core::UseAsReference;

constexpr int kTrunk = 3;
constexpr int kArm = 7;
constexpr int kNv = kTrunk + 2 * kArm;  // 17

// Group of model joint m: the trunk, the catch arm, the other arm.
constexpr int GroupOf(int m) {
  return m < kTrunk ? 0 : (m < kTrunk + kArm ? 1 : 2);
}

// The grid: two spacings, the catch node strictly inside.
constexpr int kNPre = 4;
constexpr double kDtPre = 0.05;
constexpr int kNStop = 6;
constexpr double kDt = 0.025;

BlockPattern Pattern(std::initializer_list<int> sizes) {
  BlockPattern p;
  for (const int s : sizes) {
    p.block_sizes[static_cast<std::size_t>(p.n_blocks++)] = s;
  }
  return p;
}

// One block per pre-catch node, then {1, 2, 3}: the shipped shape.
const BlockPattern kFine = Pattern({1, 1, 1, 1, 1, 2, 3});
// Each its own: coarser before the catch, or after it.
const BlockPattern kPreCoarse = Pattern({1, 1, 2, 1, 2, 3});
const BlockPattern kBothCoarse = Pattern({2, 2, 2, 2, 2});

MpcSegmentCoreParams BaseParams() {
  MpcSegmentCoreParams p;
  p.n_pre = kNPre;
  p.dt_pre = kDtPre;
  p.n_nodes = kNStop;
  p.dt = kDt;
  p.n_blocks = kFine.n_blocks;
  p.block_sizes = kFine.block_sizes;
  p.catch_terms = true;
  p.w_axis = 50.0;
  p.w_v_par = 2.0;
  p.w_v_perp = 20.0;
  p.rho_v = 2.0;
  p.v_rel_allow = 0.3;
  p.w_perp = 10.0;
  return p;
}

void SetGroups(MpcSegmentCoreParams& p, const std::array<BlockPattern, 3>& patterns) {
  p.n_groups = 3;
  for (int m = 0; m < kNv; ++m) {
    p.joint_group[static_cast<std::size_t>(m)] = static_cast<std::int8_t>(GroupOf(m));
  }
  for (std::size_t g = 0; g < 3; ++g) {
    p.group_blocks[g] = patterns[g];
  }
}

struct Fixture {
  ArmModel arm = SyntheticTree(kTrunk, kArm, 2);
  rtc::catching::MpcSegmentCoreLimits limits = LimitsFromModel(*arm.model);
  Eigen::VectorXd q_target;
  MpcSegmentCoreInput input;
};

// A reach of the trunk and the catch arm toward a pose the catch frame is
// asked to be at; the other arm is not asked to move (nothing here pulls it).
Fixture MakeFixture() {
  Fixture f;
  f.q_target = f.arm.q_nominal;
  for (int m = 0; m < kTrunk + kArm; ++m) {
    f.q_target[m] += (m % 2 == 0 ? 0.08 : -0.06) + 0.004 * m;
  }
  pinocchio::Data data(*f.arm.model);
  pinocchio::framesForwardKinematics(*f.arm.model, data, f.q_target);
  const Eigen::Vector3d p = data.oMf[f.arm.frame].translation();
  const Eigen::Vector3d z = data.oMf[f.arm.frame].rotation().col(2);
  f.input = RestInput(f.arm.q_nominal);
  f.input.p_b = p + Eigen::Vector3d(0.01, -0.01, 0.015);
  f.input.a_d = (z + Eigen::Vector3d(0.05, -0.04, 0.0)).normalized();
  f.input.v_b = Eigen::Vector3d(0.9, -0.4, -0.6);
  f.input.gamma_ref = 0.6;
  f.input.w_p = 2000.0 * Eigen::Matrix3d::Identity();
  f.input.w_delta_scale = 0.0;
  f.input.cold_start = true;
  f.input.p_c = f.input.p_b;
  f.input.d_hat = Eigen::Vector3d(1.0, 2.0, -0.5).normalized();
  return f;
}

// Rest-to-rest minimum-jerk reach arriving at the catch node, held after it.
void ReachReference(const MpcSegmentCore& mpc, const Eigen::VectorXd& q0, const Eigen::VectorXd& q1,
                    MpcSegmentCoreInput& in) {
  const int N = mpc.NumNodes();
  const double T = mpc.NodeTime(mpc.CatchNode());
  const Eigen::Index n = q0.size();
  const Eigen::VectorXd d = q1 - q0;
  in.q_ref.resize(n, N + 1);
  in.qd_ref.resize(n, N + 1);
  in.qdd_ref.resize(n, N + 1);
  for (int k = 0; k <= N; ++k) {
    const double s = std::min(mpc.NodeTime(k) / T, 1.0);
    const double s2 = s * s;
    in.q_ref.col(k) = q0 + d * (10.0 * s2 * s - 15.0 * s2 * s2 + 6.0 * s2 * s2 * s);
    in.qd_ref.col(k) = d * ((30.0 * s2 - 60.0 * s2 * s + 30.0 * s2 * s2) / T);
    in.qdd_ref.col(k) = d * ((60.0 * s - 180.0 * s2 + 120.0 * s2 * s) / (T * T));
  }
  in.reference_valid = true;
}

bool SameBits(const Eigen::MatrixXd& x, const Eigen::MatrixXd& y) {
  return x.rows() == y.rows() && x.cols() == y.cols() &&
         std::equal(x.data(), x.data() + x.size(), y.data(),
                    [](double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; });
}

bool MatricesClose(const Eigen::MatrixXd& x, const Eigen::MatrixXd& y) {
  if (x.rows() != y.rows() || x.cols() != y.cols()) {
    return false;
  }
  double scale = 1.0;
  double diff = 0.0;
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    const double xi = x.data()[i];
    const double yi = y.data()[i];
    if (!std::isfinite(xi) || !std::isfinite(yi)) {
      if (xi != yi) {
        return false;
      }
      continue;
    }
    scale = std::max(scale, std::abs(xi));
    diff = std::max(diff, std::abs(xi - yi));
  }
  return diff <= 1e-9 * scale;
}

// Two cycles of one core: the plan's first solve (scale 0), then a warm one on
// its own solution (scale 0.6).
struct TwoCycles {
  std::array<MpcSegmentCoreResult, 2> result;
  std::array<rtc::tsid::QPData, 2> qp;
};

TwoCycles SolveTwice(const Fixture& f, const MpcSegmentCoreParams& p) {
  TwoCycles out;
  MpcSegmentCore core;
  EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits), MpcSegmentCoreReason::kNone);
  if (!core.IsInitialized()) {
    return out;
  }
  MpcSegmentCoreInput in = f.input;
  ReachReference(core, f.arm.q_nominal, f.q_target, in);
  for (int cycle = 0; cycle < 2; ++cycle) {
    MpcSegmentCoreResult& r = out.result[static_cast<std::size_t>(cycle)];
    core.ResizeResult(r);
    if (cycle == 1) {
      UseAsReference(out.result[0], in);
      in.w_delta_scale = 0.6;
      in.cold_start = false;
    }
    EXPECT_TRUE(core.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason) << " cycle " << cycle;
    out.qp[static_cast<std::size_t>(cycle)] = core.MainQp();
  }
  return out;
}

// ── 1. One partition: the problem without groups ─────────────────────────────

TEST(MpcSegmentCoreGroups, GroupsOfOnePartitionAreTheProblemWithoutGroupsToTheBit) {
  const Fixture f = MakeFixture();
  const MpcSegmentCoreParams none = BaseParams();
  MpcSegmentCoreParams one = none;
  one.n_groups = 1;  // one group = no group
  MpcSegmentCoreParams three = none;
  SetGroups(three, {kFine, kFine, kFine});
  const TwoCycles a = SolveTwice(f, none);
  for (const MpcSegmentCoreParams* p : {&one, &three}) {
    SCOPED_TRACE(p->n_groups);
    const TwoCycles b = SolveTwice(f, *p);
    for (std::size_t c = 0; c < 2; ++c) {
      SCOPED_TRACE(c);
      EXPECT_TRUE(SameBits(a.qp[c].H, b.qp[c].H));
      EXPECT_TRUE(SameBits(a.qp[c].g, b.qp[c].g));
      EXPECT_TRUE(SameBits(a.qp[c].A, b.qp[c].A));
      EXPECT_TRUE(SameBits(a.qp[c].b, b.qp[c].b));
      EXPECT_TRUE(SameBits(a.qp[c].C, b.qp[c].C));
      EXPECT_TRUE(SameBits(a.qp[c].l, b.qp[c].l));
      EXPECT_TRUE(SameBits(a.qp[c].u, b.qp[c].u));
      EXPECT_TRUE(SameBits(a.result[c].q, b.result[c].q));
      EXPECT_TRUE(SameBits(a.result[c].u, b.result[c].u));
      EXPECT_EQ(a.result[c].iterations, b.result[c].iterations);
    }
  }
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, three, f.limits), MpcSegmentCoreReason::kNone);
  EXPECT_TRUE(core.Layout().Single());
  EXPECT_EQ(core.Layout().nu, kNv * kFine.n_blocks);
}

// ── 2. Different partitions ──────────────────────────────────────────────────

MpcSegmentCoreParams ThreePartitions() {
  MpcSegmentCoreParams p = BaseParams();
  SetGroups(p, {kPreCoarse, kFine, kBothCoarse});
  return p;
}

TEST(MpcSegmentCoreGroups, TheLayoutHoldsEachClassBlockMajorInModelOrder) {
  const Fixture f = MakeFixture();
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, ThreePartitions(), f.limits),
            MpcSegmentCoreReason::kNone);
  const rtc::catching::BlockLayout& l = core.Layout();
  ASSERT_EQ(l.n_classes, 3);
  EXPECT_EQ(l.nu,
            kTrunk * kPreCoarse.n_blocks + kArm * kFine.n_blocks + kArm * kBothCoarse.n_blocks);
  EXPECT_EQ(core.MainQp().H.rows(), l.nu + (kNPre + kNStop) + 1);  // a slack per node (#857)
  // Every (joint, block) has its own column.
  std::vector<int> seen(static_cast<std::size_t>(l.nu), 0);
  for (int j = 0; j < kNv; ++j) {
    EXPECT_EQ(l.class_of[static_cast<std::size_t>(j)], GroupOf(j)) << j;
    for (int b = 0; b < l.pattern[static_cast<std::size_t>(GroupOf(j))].n_blocks; ++b) {
      ++seen[static_cast<std::size_t>(l.Col(j, b))];
    }
  }
  EXPECT_TRUE(std::all_of(seen.begin(), seen.end(), [](int s) { return s == 1; }));
}

// The two groups that share a partition share a class; the third has its own.
TEST(MpcSegmentCoreGroups, GroupsWithOnePartitionShareAClass) {
  const Fixture f = MakeFixture();
  MpcSegmentCoreParams p = BaseParams();
  SetGroups(p, {kBothCoarse, kFine, kBothCoarse});
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits), MpcSegmentCoreReason::kNone);
  const rtc::catching::BlockLayout& l = core.Layout();
  ASSERT_EQ(l.n_classes, 2);
  EXPECT_EQ(l.n_joints[0], kTrunk + kArm);
  EXPECT_EQ(l.n_joints[1], kArm);
  for (int j = 0; j < kNv; ++j) {
    EXPECT_EQ(l.class_of[static_cast<std::size_t>(j)], GroupOf(j) == 1 ? 1 : 0) << j;
  }
}

// Response of the scalar triple integrator at instant t to unit jerk on [t0, t1].
double UnitJerkResponseAt(int m, double t, double t0, double t1) {
  if (t0 >= t) {
    return 0.0;
  }
  const double a = t - t0;
  const double b = t - std::min(t1, t);
  switch (m) {
    case 0:
      return (a * a * a - b * b * b) / 6.0;
    case 1:
      return (a * a - b * b) / 2.0;
    default:
      return std::min(t1, t) - t0;
  }
}

double NodeTime(int k) {
  return k <= kNPre ? k * kDtPre : kNPre * kDtPre + (k - kNPre) * kDt;
}

TEST(MpcSegmentCoreGroups, EachClassesStageGainsMatchTheIntegralOracle) {
  const Fixture f = MakeFixture();
  const MpcSegmentCoreParams p = ThreePartitions();
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits), MpcSegmentCoreReason::kNone);
  for (int c = 0; c < 3; ++c) {
    const BlockPattern& pat = p.group_blocks[static_cast<std::size_t>(c)];  // class c = group c
    int start = 0;
    for (int b = 0; b < pat.n_blocks; ++b) {
      const int end = start + pat.block_sizes[static_cast<std::size_t>(b)];
      for (int k = 0; k <= kNPre + kNStop; ++k) {
        for (int m = 0; m < 3; ++m) {
          const double expected =
              p.u_scale * UnitJerkResponseAt(m, NodeTime(k), NodeTime(start), NodeTime(end));
          EXPECT_NEAR(core.StageGain(m, k, b, c), expected, 1e-9 * (1.0 + std::abs(expected)))
              << "class " << c << " block " << b << " node " << k << " order " << m;
        }
      }
      start = end;
    }
    EXPECT_TRUE(std::isnan(core.StageGain(0, 1, pat.n_blocks, c))) << "past the class's blocks";
  }
  EXPECT_TRUE(std::isnan(core.StageGain(0, 1, 0, 3))) << "no such class";
  EXPECT_EQ(core.TerminalRank(), 2 * kNv);
}

TEST(MpcSegmentCoreGroups, StructuredAssemblyMatchesDenseWithThreePartitions) {
  const Fixture f = MakeFixture();
  const MpcSegmentCoreParams fast = ThreePartitions();
  MpcSegmentCoreParams dense = fast;
  dense.reference_assembly = true;
  const TwoCycles a = SolveTwice(f, fast);
  const TwoCycles b = SolveTwice(f, dense);
  for (std::size_t c = 0; c < 2; ++c) {
    SCOPED_TRACE(c);
    EXPECT_TRUE(MatricesClose(a.qp[c].H, b.qp[c].H));
    EXPECT_TRUE(MatricesClose(a.qp[c].g, b.qp[c].g));
    EXPECT_TRUE(MatricesClose(a.qp[c].A, b.qp[c].A));
    EXPECT_TRUE(MatricesClose(a.qp[c].b, b.qp[c].b));
    EXPECT_TRUE(MatricesClose(a.qp[c].C, b.qp[c].C));
    EXPECT_TRUE(MatricesClose(a.qp[c].l, b.qp[c].l));
    EXPECT_TRUE(MatricesClose(a.qp[c].u, b.qp[c].u));
    EXPECT_EQ((a.qp[c].H - a.qp[c].H.transpose()).cwiseAbs().maxCoeff(), 0.0) << "H symmetric";
    EXPECT_LE((a.result[c].u - b.result[c].u).cwiseAbs().maxCoeff(),
              1e-6 * (1.0 + b.result[c].u.cwiseAbs().maxCoeff()));
  }
}

// The interval → block map of a partition, computed here.
std::vector<int> BlockOfInterval(const BlockPattern& p) {
  std::vector<int> out;
  for (int b = 0; b < p.n_blocks; ++b) {
    for (int i = 0; i < p.block_sizes[static_cast<std::size_t>(b)]; ++i) {
      out.push_back(b);
    }
  }
  return out;
}

TEST(MpcSegmentCoreGroups, EveryJointsJerkIsConstantOnItsGroupsBlocksAndIntegratesToItsNodes) {
  const Fixture f = MakeFixture();
  const MpcSegmentCoreParams p = ThreePartitions();
  const TwoCycles run = SolveTwice(f, p);
  const MpcSegmentCoreResult& r = run.result[1];
  const int N = kNPre + kNStop;
  ASSERT_EQ(r.u.cols(), N);
  // A boundary of one group that is not one of another: some joint of the
  // group whose block continues there must keep its jerk while some joint of
  // the other changes — otherwise this fixture would not tell them apart.
  bool kept_across_another_groups_boundary = false;
  bool changed_somewhere = false;
  for (int j = 0; j < kNv; ++j) {
    const std::vector<int> blocks =
        BlockOfInterval(p.group_blocks[static_cast<std::size_t>(GroupOf(j))]);
    for (int k = 0; k + 1 < N; ++k) {
      if (blocks[static_cast<std::size_t>(k)] == blocks[static_cast<std::size_t>(k + 1)]) {
        EXPECT_EQ(r.u(j, k), r.u(j, k + 1)) << "joint " << j << " interval " << k;
        kept_across_another_groups_boundary =
            kept_across_another_groups_boundary || (GroupOf(j) == 2 && k == 0);
      } else if (r.u(j, k) != r.u(j, k + 1)) {
        changed_somewhere = true;
      }
    }
  }
  EXPECT_TRUE(kept_across_another_groups_boundary);
  EXPECT_TRUE(changed_somewhere);
  // The nodes are the triple integral of that jerk from x_0 — read with each
  // joint's own blocks, nothing of the core's gains.
  Eigen::VectorXd q = r.q.col(0), v = r.qd.col(0), a = r.qdd.col(0);
  for (int k = 0; k < N; ++k) {
    const double dt = NodeTime(k + 1) - NodeTime(k);
    const Eigen::VectorXd u = r.u.col(k);
    q += dt * v + 0.5 * dt * dt * a + dt * dt * dt / 6.0 * u;
    v += dt * a + 0.5 * dt * dt * u;
    a += dt * u;
    EXPECT_LE((q - r.q.col(k + 1)).cwiseAbs().maxCoeff(), 1e-9) << "node " << k + 1;
    EXPECT_LE((v - r.qd.col(k + 1)).cwiseAbs().maxCoeff(), 1e-9) << "node " << k + 1;
    EXPECT_LE((a - r.qdd.col(k + 1)).cwiseAbs().maxCoeff(), 1e-9) << "node " << k + 1;
  }
  EXPECT_LE(r.qd.col(N).cwiseAbs().maxCoeff(), 1e-6) << "terminal rest";
  EXPECT_LE(r.qdd.col(N).cwiseAbs().maxCoeff(), 1e-6) << "terminal rest";
}

// ── 3. Each partition checked ────────────────────────────────────────────────

TEST(MpcSegmentCoreGroups, InitChecksEveryGroupsPartition) {
  const Fixture f = MakeFixture();

  struct Case {
    BlockPattern pattern;
    MpcSegmentCoreReason why;
  };

  for (const Case& c : {
           Case{Pattern({1, 1, 3, 2, 3}), MpcSegmentCoreReason::kBlocksAcrossCatch},
           Case{Pattern({2, 2, 3, 3}), MpcSegmentCoreReason::kBlocksTooFew},
           Case{Pattern({2, 2, 2, 2, 1}), MpcSegmentCoreReason::kParamsInvalid},  // Σ ≠ N
       }) {
    SCOPED_TRACE(MpcSegmentCoreReasonName(c.why));
    MpcSegmentCoreParams p = BaseParams();
    SetGroups(p, {kFine, kFine, c.pattern});
    MpcSegmentCore core;
    EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits), c.why);
    // The same partition on a group no joint is in is not read.
    p.joint_group.fill(0);
    EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits), MpcSegmentCoreReason::kNone);
  }
  MpcSegmentCoreParams p = ThreePartitions();
  p.joint_group[5] = 3;  // a group that is not declared
  MpcSegmentCore core;
  EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits),
            MpcSegmentCoreReason::kParamsInvalid);
  p = ThreePartitions();
  p.n_groups = rtc::catching::kMaxJointGroups + 1;
  EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits),
            MpcSegmentCoreReason::kParamsInvalid);
}

// ── 4. Velocity and rest terms ───────────────────────────────────────────────

// Per joint and distinct (an index slip changes the answer): the trunk's
// velocities damped, the other arm held near its rest posture — the nominal
// posture, offset per joint.
void SetWholeBody(MpcSegmentCoreParams& p, double w_vel, double w_rest, const Eigen::VectorXd& q) {
  p.w_vel = Eigen::VectorXd::Zero(kNv);
  p.w_rest = Eigen::VectorXd::Zero(kNv);
  p.q_rest = Eigen::VectorXd::Constant(kNv, std::numeric_limits<double>::quiet_NaN());
  for (int m = 0; m < kNv; ++m) {
    if (GroupOf(m) == 0) {
      p.w_vel[m] = w_vel * (1.0 + 0.1 * m);
    }
    if (GroupOf(m) == 2) {
      p.w_rest[m] = w_rest * (1.0 + 0.05 * (m - kTrunk - kArm));
      p.q_rest[m] = q[m] + 0.002 * (m - kTrunk - kArm);
    }
  }
}

TEST(MpcSegmentCoreWholeBody, OffIsTheQpWithoutTheTermsAndReadsNoRestPosture) {
  const Fixture f = MakeFixture();
  const MpcSegmentCoreParams without = ThreePartitions();
  MpcSegmentCoreParams zero = without;
  SetWholeBody(zero, 0.0, 0.0, f.arm.q_nominal);
  zero.q_rest.setConstant(std::numeric_limits<double>::quiet_NaN());  // never read
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, zero, f.limits), MpcSegmentCoreReason::kNone);
  EXPECT_FALSE(core.VelocityTermActive());
  EXPECT_FALSE(core.RestTermActive());
  const TwoCycles a = SolveTwice(f, without);
  const TwoCycles b = SolveTwice(f, zero);
  for (std::size_t c = 0; c < 2; ++c) {
    SCOPED_TRACE(c);
    EXPECT_TRUE(SameBits(a.qp[c].H, b.qp[c].H));
    EXPECT_TRUE(SameBits(a.qp[c].g, b.qp[c].g));
    EXPECT_TRUE(SameBits(a.result[c].u, b.result[c].u));
    EXPECT_EQ(b.result[c].whole_body.vel, 0.0);
    EXPECT_EQ(b.result[c].whole_body.rest, 0.0);
  }
  // On: the flags, and the rest posture read where the weight is positive.
  MpcSegmentCoreParams on = without;
  SetWholeBody(on, 0.1, 1.0, f.arm.q_nominal);
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, on, f.limits), MpcSegmentCoreReason::kNone)
      << "NaN where the rest weight is 0 is not read";
  EXPECT_TRUE(core.VelocityTermActive());
  EXPECT_TRUE(core.RestTermActive());
  MpcSegmentCoreParams bad = on;
  bad.q_rest[kNv - 1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, bad, f.limits),
            MpcSegmentCoreReason::kParamsInvalid);
  bad = on;
  bad.q_rest.resize(0);
  EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, bad, f.limits),
            MpcSegmentCoreReason::kParamsInvalid);
  bad = on;
  bad.w_vel[0] = -1e-9;
  EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, bad, f.limits),
            MpcSegmentCoreReason::kParamsInvalid);
  bad = on;
  bad.w_rest[kNv - 1] = std::nextafter(rtc::catching::kWholeBodyWeightMax, 1e300);
  EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, bad, f.limits),
            MpcSegmentCoreReason::kParamsInvalid);
  bad = on;
  bad.w_vel.resize(kNv - 1);
  EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, bad, f.limits),
            MpcSegmentCoreReason::kParamsInvalid);
  bad = on;
  bad.w_rest[kNv - 1] = rtc::catching::kWholeBodyWeightMax;  // the bound itself is taken
  EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, bad, f.limits), MpcSegmentCoreReason::kNone);
}

// One cold first solve of the fixture's reach at a given w_delta_scale.
rtc::tsid::QPData FirstSolveQp(const Fixture& f, const MpcSegmentCoreParams& p, double scale,
                               MpcSegmentCoreResult* result = nullptr) {
  MpcSegmentCore core;
  EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreInput in = f.input;
  ReachReference(core, f.arm.q_nominal, f.q_target, in);
  in.w_delta_scale = scale;
  MpcSegmentCoreResult r;
  core.ResizeResult(r);
  EXPECT_TRUE(core.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason);
  if (result != nullptr) {
    *result = r;
  }
  return core.MainQp();
}

TEST(MpcSegmentCoreWholeBody, TheTermsAreOnEveryPathOfTheHessian) {
  const Fixture f = MakeFixture();
  const MpcSegmentCoreParams off = ThreePartitions();
  MpcSegmentCoreParams on = off;
  SetWholeBody(on, 0.1, 1.0, f.arm.q_nominal);
  MpcSegmentCoreParams dense = on;
  dense.reference_assembly = true;
  Eigen::MatrixXd first_difference;
  for (const double scale : {0.0, 0.37, 1.0}) {
    SCOPED_TRACE(scale);
    const rtc::tsid::QPData h_on = FirstSolveQp(f, on, scale);
    const rtc::tsid::QPData h_dense = FirstSolveQp(f, dense, scale);
    const rtc::tsid::QPData h_off = FirstSolveQp(f, off, scale);
    EXPECT_TRUE(MatricesClose(h_on.H, h_dense.H));
    EXPECT_TRUE(MatricesClose(h_on.g, h_dense.g));
    // The terms' Hessian is constant: the same on every path, whatever the
    // scale — a path that drops it shows here.
    const Eigen::MatrixXd difference = h_on.H - h_off.H;
    EXPECT_GT(difference.cwiseAbs().maxCoeff(), 0.0);
    if (first_difference.size() == 0) {
      first_difference = difference;
    } else {
      EXPECT_LE((difference - first_difference).cwiseAbs().maxCoeff(),
                1e-12 * first_difference.cwiseAbs().maxCoeff());
    }
  }
}

TEST(MpcSegmentCoreWholeBody, TheReportedValuesAreTheDirectSumsAtTheSolution) {
  const Fixture f = MakeFixture();
  MpcSegmentCoreParams p = ThreePartitions();
  SetWholeBody(p, 0.1, 1.0, f.arm.q_nominal);
  MpcSegmentCoreResult r;
  (void)FirstSolveQp(f, p, 0.0, &r);
  // ½ Σ_{k=1..N} — node 0 is x_0, a constant (no variable to act on).
  std::array<double, 3> vel{}, rest{};
  for (int k = 1; k <= kNPre + kNStop; ++k) {
    for (int m = 0; m < kNv; ++m) {
      const double v = r.qd(m, k);
      vel[static_cast<std::size_t>(GroupOf(m))] += 0.5 * p.w_vel[m] * v * v;
      if (p.w_rest[m] > 0.0) {
        const double dq = r.q(m, k) - p.q_rest[m];
        rest[static_cast<std::size_t>(GroupOf(m))] += 0.5 * p.w_rest[m] * dq * dq;
      }
    }
  }
  const auto near = [](double a, double b) {
    return std::abs(a - b) <= 1e-12 * (1.0 + std::abs(b));
  };
  EXPECT_GT(vel[0], 0.0);
  EXPECT_GT(rest[2], 0.0);
  for (std::size_t g = 0; g < 3; ++g) {
    EXPECT_TRUE(near(r.whole_body.vel_group[g], vel[g])) << g;
    EXPECT_TRUE(near(r.whole_body.rest_group[g], rest[g])) << g;
  }
  EXPECT_TRUE(near(r.whole_body.vel, vel[0] + vel[1] + vel[2]));
  EXPECT_TRUE(near(r.whole_body.rest, rest[0] + rest[1] + rest[2]));
}

// At rest, the ball where the catch frame is and at
// rest, the held group at its rest posture — nothing to do, with every term
// on, at both ends of the w_Δ schedule.
TEST(MpcSegmentCoreWholeBody, AnExactStopStaysAtZeroWithTheTermsOn) {
  Fixture f = MakeFixture();
  MpcSegmentCoreParams p = ThreePartitions();
  SetWholeBody(p, 0.1, 1.0, f.arm.q_nominal);
  p.q_rest = f.arm.q_nominal;
  p.w_ang_pre = 0.01;
  p.w_ang_post = 1.0;
  p.ang_scale = 10.0;
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits), MpcSegmentCoreReason::kNone);
  pinocchio::Data data(*f.arm.model);
  pinocchio::framesForwardKinematics(*f.arm.model, data, f.arm.q_nominal);
  MpcSegmentCoreInput in = f.input;
  in.q_ref = f.arm.q_nominal.replicate(1, core.NumNodes() + 1);
  in.qd_ref.setZero(kNv, core.NumNodes() + 1);
  in.qdd_ref.setZero(kNv, core.NumNodes() + 1);
  in.reference_valid = true;
  in.p_b = data.oMf[f.arm.frame].translation();
  in.a_d = data.oMf[f.arm.frame].rotation().col(2);
  in.v_b.setZero();
  in.p_c = in.p_b;
  for (const double scale : {0.0, 1.0}) {
    SCOPED_TRACE(scale);
    in.w_delta_scale = scale;
    MpcSegmentCoreResult r;
    core.ResizeResult(r);
    ASSERT_TRUE(core.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason);
    EXPECT_LE(r.u.cwiseAbs().maxCoeff(), 10.0 * p.solver.eps_abs * p.u_scale);
    EXPECT_LE(r.whole_body.vel, 1e-12);
    EXPECT_LE(r.whole_body.rest, 1e-12);
    // Zero to the QP's tolerance, like u: the rate is A·q̈ of that u.
    EXPECT_LE(r.whole_body.ang_rate_max, 1e-6);
  }
}

TEST(MpcSegmentCoreWholeBody, TheRestTermPullsItsGroupBackAndTheVelocityTermSlowsItsGroup) {
  Fixture f = MakeFixture();
  // The other arm starts 0.05 rad off its rest posture, at rest.
  for (int m = kTrunk + kArm; m < kNv; ++m) {
    f.input.q0[m] += 0.05;
  }
  const auto run = [&f](double w_vel, double w_rest) {
    MpcSegmentCoreParams p = ThreePartitions();
    SetWholeBody(p, w_vel, w_rest, f.arm.q_nominal);
    p.q_rest = f.arm.q_nominal;
    MpcSegmentCore core;
    EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits), MpcSegmentCoreReason::kNone);
    MpcSegmentCoreInput in = f.input;
    ReachReference(core, in.q0, f.q_target, in);
    for (int m = kTrunk + kArm; m < kNv; ++m) {
      in.q_ref.row(m).setConstant(in.q0[m]);
    }
    MpcSegmentCoreResult r;
    core.ResizeResult(r);
    EXPECT_TRUE(core.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason);
    return r;
  };
  const MpcSegmentCoreResult off = run(0.0, 0.0);
  const MpcSegmentCoreResult held = run(0.0, 100.0);
  const MpcSegmentCoreResult damped = run(0.1, 0.0);
  const int N = kNPre + kNStop;
  const auto off_rest = [&f, N](const MpcSegmentCoreResult& r) {
    double d = 0.0;
    for (int m = kTrunk + kArm; m < kNv; ++m) {
      d = std::max(d, std::abs(r.q(m, N) - f.arm.q_nominal[m]));
    }
    return d;
  };
  EXPECT_GE(off_rest(off), 0.05 - 1e-9) << "nothing moves the other arm without the term";
  EXPECT_LE(off_rest(held), 0.5 * 0.05);
  const auto trunk_speed = [N](const MpcSegmentCoreResult& r) {
    double s = 0.0;
    for (int k = 1; k <= N; ++k) {
      s += r.qd.block(0, k, kTrunk, 1).squaredNorm();
    }
    return s;
  };
  EXPECT_LT(trunk_speed(damped), trunk_speed(off));
}

// The same input gives the same answer whatever the core solved before it —
// every per-solve part of H is rebuilt (or rightly kept).
TEST(MpcSegmentCoreWholeBody, SameInputGivesTheSameAnswerWhateverWasSolvedBefore) {
  const Fixture f = MakeFixture();
  MpcSegmentCoreParams p = ThreePartitions();
  SetWholeBody(p, 0.1, 1.0, f.arm.q_nominal);
  p.w_ang_pre = 0.01;
  p.w_ang_post = 1.0;
  p.ang_scale = 10.0;
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreInput a = f.input;
  ReachReference(core, f.arm.q_nominal, f.q_target, a);
  a.w_delta_scale = 1.0;
  MpcSegmentCoreInput b = a;
  b.w_delta_scale = 0.4;
  b.p_b += Eigen::Vector3d(0.02, 0.01, -0.01);
  for (int m = kTrunk + kArm; m < kNv; ++m) {
    b.q0[m] += 0.03;
    b.q_ref.row(m).array() += 0.03;
  }
  MpcSegmentCoreResult first, middle, again;
  core.ResizeResult(first);
  core.ResizeResult(middle);
  core.ResizeResult(again);
  ASSERT_TRUE(core.Solve(a, first)) << MpcSegmentCoreReasonName(first.reason);
  const rtc::tsid::QPData qp_first = core.MainQp();
  ASSERT_TRUE(core.Solve(b, middle)) << MpcSegmentCoreReasonName(middle.reason);
  ASSERT_TRUE(core.Solve(a, again)) << MpcSegmentCoreReasonName(again.reason);
  EXPECT_TRUE(SameBits(qp_first.H, core.MainQp().H));
  EXPECT_TRUE(SameBits(qp_first.g, core.MainQp().g));
  EXPECT_TRUE(SameBits(qp_first.C, core.MainQp().C));
  EXPECT_TRUE(SameBits(qp_first.l, core.MainQp().l));
  EXPECT_TRUE(SameBits(qp_first.u, core.MainQp().u));
  EXPECT_TRUE(SameBits(first.u, again.u));
  EXPECT_EQ(first.whole_body.rest, again.whole_body.rest);
  EXPECT_EQ(first.whole_body.ang, again.whole_body.ang);
  EXPECT_GT(first.whole_body.ang, 0.0);
}

// ── 5. Angular-momentum rate ─────────────────────────────────────────────────

// The fixture's values: s = 10 N·m, W⁻ / W⁺ = 0.01 / 1.
constexpr double kAngScale = 10.0;
constexpr double kAngPre = 0.01;
constexpr double kAngPost = 1.0;

void SetAngular(MpcSegmentCoreParams& p, double pre = kAngPre, double post = kAngPost) {
  p.w_ang_pre = pre;
  p.w_ang_post = post;
  p.ang_scale = kAngScale;
}

TEST(MpcSegmentCoreAngularMomentum, OffIsTheQpWithoutTheTermAndInitChecksTheKeys) {
  const Fixture f = MakeFixture();
  const MpcSegmentCoreParams without = ThreePartitions();
  MpcSegmentCoreParams zero = without;
  zero.ang_scale = 0.0;  // not read while both weights are 0
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, zero, f.limits), MpcSegmentCoreReason::kNone);
  EXPECT_FALSE(core.AngularMomentumTermActive());
  const TwoCycles a = SolveTwice(f, without);
  const TwoCycles b = SolveTwice(f, zero);
  for (std::size_t c = 0; c < 2; ++c) {
    EXPECT_TRUE(SameBits(a.qp[c].H, b.qp[c].H)) << c;
    EXPECT_TRUE(SameBits(a.qp[c].g, b.qp[c].g)) << c;
    EXPECT_EQ(b.result[c].whole_body.ang, 0.0);
    EXPECT_EQ(b.result[c].whole_body.ang_rate_max, 0.0);
  }
  MpcSegmentCoreParams on = without;
  SetAngular(on);
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, on, f.limits), MpcSegmentCoreReason::kNone);
  EXPECT_TRUE(core.AngularMomentumTermActive());
  for (const auto& bad : std::vector<std::function<void(MpcSegmentCoreParams&)>>{
           [](MpcSegmentCoreParams& p) { p.ang_scale = 0.0; },
           [](MpcSegmentCoreParams& p) { p.ang_scale = -1.0; },
           [](MpcSegmentCoreParams& p) { p.ang_scale = std::numeric_limits<double>::infinity(); },
           [](MpcSegmentCoreParams& p) { p.ang_scale = 1e-160; },  // W/s² = ∞
           [](MpcSegmentCoreParams& p) { p.ang_scale = 1e160; },   // W/s² = 0
           [](MpcSegmentCoreParams& p) { p.w_ang_pre = -1e-9; },
           [](MpcSegmentCoreParams& p) { p.w_ang_post = std::numeric_limits<double>::quiet_NaN(); },
           [](MpcSegmentCoreParams& p) {
             p.w_ang_post = std::nextafter(rtc::catching::kWholeBodyWeightMax, 1e300);
           },
       }) {
    MpcSegmentCoreParams p = on;
    bad(p);
    EXPECT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits),
              MpcSegmentCoreReason::kParamsInvalid);
  }
}

// What the term is at reference x̄, from pinocchio's centroidal map (not the
// core's dccrba) and a central difference of it along q̄̇ for Ȧ q̄̇.
struct AngularOracle {
  Eigen::Matrix<double, 3, Eigen::Dynamic> a;  // A^ω_G(q̄)
  Eigen::Vector3d d;                           // Ȧ^ω_G(q̄, q̄̇) q̄̇
};

AngularOracle AngularAt(const pinocchio::Model& model, const Eigen::VectorXd& q,
                        const Eigen::VectorXd& v) {
  pinocchio::Data data(model);
  AngularOracle o;
  o.a = pinocchio::computeCentroidalMap(model, data, q).bottomRows<3>();
  const double eps = 1e-6;
  const Eigen::MatrixXd ap =
      pinocchio::computeCentroidalMap(model, data, q + eps * v).bottomRows<3>();
  const Eigen::MatrixXd am =
      pinocchio::computeCentroidalMap(model, data, q - eps * v).bottomRows<3>();
  o.d = (ap - am) / (2.0 * eps) * v;
  return o;
}

// The term's z-space operator at node k, from the core's stage gains and
// layout (not its assembly): L(:, Col(j, b)) = ĝ_a[b] A(:, j).
Eigen::MatrixXd StageOperator(const MpcSegmentCore& core, int k,
                              const Eigen::Matrix<double, 3, Eigen::Dynamic>& a, int nz) {
  const rtc::catching::BlockLayout& l = core.Layout();
  Eigen::MatrixXd op = Eigen::MatrixXd::Zero(3, nz);
  for (int j = 0; j < kNv; ++j) {
    const int c = l.class_of[static_cast<std::size_t>(j)];
    for (int b = 0; b < l.pattern[static_cast<std::size_t>(c)].n_blocks; ++b) {
      op.col(l.Col(j, b)) = core.StageGain(2, k, b, c) * a.col(j);
    }
  }
  return op;
}

double WeightAt(int k, double pre, double post) {
  return (k < kNPre ? pre : post) / (kAngScale * kAngScale);
}

TEST(MpcSegmentCoreAngularMomentum, TheAssembledTermIsTheCentroidalMapsAfterASecondLinearisation) {
  const Fixture f = MakeFixture();
  // Both assemblies: the one block class (no groups) and three.
  for (const MpcSegmentCoreParams& off : {BaseParams(), ThreePartitions()}) {
    SCOPED_TRACE(off.n_groups);
    MpcSegmentCoreParams on = off;
    SetAngular(on);
    MpcSegmentCore core_off, core_on;
    ASSERT_EQ(core_off.Init(*f.arm.model, f.arm.frame, off, f.limits), MpcSegmentCoreReason::kNone);
    ASSERT_EQ(core_on.Init(*f.arm.model, f.arm.frame, on, f.limits), MpcSegmentCoreReason::kNone);
    MpcSegmentCoreResult r_off, r_on;
    core_off.ResizeResult(r_off);
    core_on.ResizeResult(r_on);
    // First at the reach, then both at ONE other x̄ (the off core's solution):
    // the second linearisation of the same core, so a stale Data or a stale
    // A_k shows; the two QPs differ by the term alone.
    MpcSegmentCoreInput in = f.input;
    ReachReference(core_off, f.arm.q_nominal, f.q_target, in);
    ASSERT_TRUE(core_off.Solve(in, r_off)) << MpcSegmentCoreReasonName(r_off.reason);
    ASSERT_TRUE(core_on.Solve(in, r_on)) << MpcSegmentCoreReasonName(r_on.reason);
    const MpcSegmentCoreResult ref = r_off;
    UseAsReference(ref, in);
    in.w_delta_scale = 0.6;
    ASSERT_TRUE(core_off.Solve(in, r_off)) << MpcSegmentCoreReasonName(r_off.reason);
    ASSERT_TRUE(core_on.Solve(in, r_on)) << MpcSegmentCoreReasonName(r_on.reason);
    const int nz = static_cast<int>(core_on.MainQp().H.rows());
    Eigen::MatrixXd dh = Eigen::MatrixXd::Zero(nz, nz);
    Eigen::VectorXd dg = Eigen::VectorXd::Zero(nz);
    for (int k = 1; k <= kNPre + kNStop; ++k) {
      const AngularOracle o = AngularAt(*f.arm.model, ref.q.col(k), ref.qd.col(k));
      const Eigen::MatrixXd op = StageOperator(core_on, k, o.a, nz);
      // x_0 rests, so the free response has no acceleration: r_k = d_k.
      const Eigen::Vector3d r = o.a * in.qdd0 + o.d;
      const double w = WeightAt(k, kAngPre, kAngPost);
      dh += w * op.transpose() * op;
      dg += w * op.transpose() * r;
    }
    const Eigen::MatrixXd got_h = core_on.MainQp().H - core_off.MainQp().H;
    const Eigen::VectorXd got_g = core_on.MainQp().g - core_off.MainQp().g;
    EXPECT_GT(dh.cwiseAbs().maxCoeff(), 0.0);
    EXPECT_GT(dg.cwiseAbs().maxCoeff(), 0.0) << "the reference moves: the drift is not zero";
    EXPECT_LE((got_h - dh).cwiseAbs().maxCoeff(), 1e-6 * dh.cwiseAbs().maxCoeff());
    EXPECT_LE((got_g - dg).cwiseAbs().maxCoeff(), 1e-6 * dg.cwiseAbs().maxCoeff());
    const Eigen::MatrixXd& h = core_on.MainQp().H;
    EXPECT_EQ((h - h.transpose()).cwiseAbs().maxCoeff(), 0.0) << "H symmetric";
  }
}

// The value and the schedule: ½ Σ_k W(k)/s² ‖A(q̄_k) q̈_k + d_k‖² over the
// solution's accelerations, W⁻ before the catch node and W⁺ from it on — and
// the largest rate over the nodes the term weights.
TEST(MpcSegmentCoreAngularMomentum, TheReportedValueFollowsTheScheduleOnTheSolution) {
  const Fixture f = MakeFixture();
  for (const std::array<double, 2>& w :
       {std::array<double, 2>{kAngPre, kAngPost}, std::array<double, 2>{0.0, kAngPost},
        std::array<double, 2>{kAngPost, 0.0}}) {
    SCOPED_TRACE(w[0]);
    MpcSegmentCoreParams p = ThreePartitions();
    SetAngular(p, w[0], w[1]);
    MpcSegmentCore core;
    ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits), MpcSegmentCoreReason::kNone);
    MpcSegmentCoreInput in = f.input;
    ReachReference(core, f.arm.q_nominal, f.q_target, in);
    MpcSegmentCoreResult r;
    core.ResizeResult(r);
    ASSERT_TRUE(core.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason);
    double value = 0.0;
    double rate_max = 0.0;
    for (int k = 1; k <= kNPre + kNStop; ++k) {
      const double wk = WeightAt(k, w[0], w[1]);
      if (wk == 0.0) {
        continue;
      }
      const AngularOracle o = AngularAt(*f.arm.model, in.q_ref.col(k), in.qd_ref.col(k));
      const Eigen::Vector3d kd = o.a * r.qdd.col(k) + o.d;
      value += 0.5 * wk * kd.squaredNorm();
      rate_max = std::max(rate_max, kd.norm());
    }
    EXPECT_GT(value, 0.0);
    EXPECT_NEAR(r.whole_body.ang, value, 1e-6 * value);
    EXPECT_NEAR(r.whole_body.ang_rate_max, rate_max, 1e-6 * rate_max);
  }
}

// h^ω(t) = A^ω_G(q(t)) q̇(t) along the solution's cubic pieces, nothing of the core.
Eigen::Vector3d AngularMomentumAt(const pinocchio::Model& model, const MpcSegmentCoreResult& r,
                                  int k, double tau) {
  // Within interval [k, k+1] (τ ≥ 0) or [k−1, k] (τ < 0), from node k.
  const int i = tau >= 0.0 ? k : k - 1;
  const Eigen::VectorXd u = r.u.col(i);
  const Eigen::VectorXd q =
      r.q.col(k) + tau * r.qd.col(k) + 0.5 * tau * tau * r.qdd.col(k) + tau * tau * tau / 6.0 * u;
  const Eigen::VectorXd v = r.qd.col(k) + tau * r.qdd.col(k) + 0.5 * tau * tau * u;
  pinocchio::Data data(model);
  return pinocchio::computeCentroidalMap(model, data, q).bottomRows<3>() * v;
}

// At an RTI fixed point (the reference is the solution) the linear model is
// the trajectory's own: the reported largest rate is the time derivative of
// the solution's angular momentum.
TEST(MpcSegmentCoreAngularMomentum, AtAFixedPointTheRateIsTheSolutionsTimeDerivative) {
  const Fixture f = MakeFixture();
  MpcSegmentCoreParams p = ThreePartitions();
  SetAngular(p, kAngPost, kAngPost);  // every node weighted
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreInput in = f.input;
  ReachReference(core, f.arm.q_nominal, f.q_target, in);
  in.w_delta_scale = 1.0;
  MpcSegmentCoreResult r;
  core.ResizeResult(r);
  // Settled to the QP's own tolerance (the iterates stop moving at ~eps_abs).
  double step = 1.0;
  for (int it = 0; it < 30 && step > 1e-6; ++it) {
    ASSERT_TRUE(core.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason) << " iteration " << it;
    step = (r.q - in.q_ref).cwiseAbs().maxCoeff();
    UseAsReference(r, in);
    in.cold_start = false;
  }
  ASSERT_LE(step, 1e-6) << "the RTI did not settle";
  const int N = kNPre + kNStop;
  // The jerk jumps at a node, so h is smooth only within an interval: a
  // second-order difference on ONE side of the node (forward, backward at N)
  // — a central one would carry δ/4 times the jump of ḧ.
  const double delta = 1e-5;
  double rate_max = 0.0;
  for (int k = 1; k <= N; ++k) {
    const double sgn = k < N ? 1.0 : -1.0;
    const Eigen::Vector3d h0 = AngularMomentumAt(*f.arm.model, r, k, 0.0);
    const Eigen::Vector3d h1 = AngularMomentumAt(*f.arm.model, r, k, sgn * delta);
    const Eigen::Vector3d h2 = AngularMomentumAt(*f.arm.model, r, k, sgn * 2.0 * delta);
    const Eigen::Vector3d rate = sgn * (-3.0 * h0 + 4.0 * h1 - h2) / (2.0 * delta);
    rate_max = std::max(rate_max, rate.norm());
  }
  EXPECT_GT(rate_max, 0.0);
  EXPECT_NEAR(r.whole_body.ang_rate_max, rate_max, 1e-4 * rate_max);
}

// The term alone must linearise: no torque rows, no line, no catch node.
TEST(MpcSegmentCoreAngularMomentum, TheTermAloneLinearisesAndAllocatesNothing) {
  const Fixture f = MakeFixture();
  MpcSegmentCoreParams off;
  off.n_nodes = kNStop;
  off.dt = kDt;
  off.n_blocks = 3;
  off.block_sizes = {1, 2, 3};
  off.rho_tau = 0.0;
  off.w_perp = 0.0;
  MpcSegmentCoreParams on = off;
  SetAngular(on, 0.0, kAngPost);  // W⁺ covers every node without a pre-catch grid
  // A moving entry state: the stop has something to do and the drift is not 0.
  MpcSegmentCoreInput in = RestInput(f.arm.q_nominal);
  for (int m = 0; m < kNv; ++m) {
    in.qd0[m] = 0.2 * std::sin(0.7 * m + 0.3);
  }
  rtc::tsid::QPData qp_off, qp_on;
  MpcSegmentCoreResult r;
  for (const bool term : {false, true}) {
    MpcSegmentCore core;
    ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, term ? on : off, f.limits),
              MpcSegmentCoreReason::kNone);
    core.ResizeResult(r);
    ASSERT_TRUE(core.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason);  // the pre-solve path
    MpcSegmentCoreInput warm = in;
    UseAsReference(r, warm);
    ASSERT_TRUE(core.Solve(warm, r)) << MpcSegmentCoreReasonName(r.reason);  // warm-up
    (term ? qp_on : qp_off) = core.MainQp();
    if (!term) {
      continue;
    }
    EXPECT_GT(r.whole_body.ang_rate_max, 0.0);
    // No allocation: the whole Solve by operator new; up to the QP — the
    // linearisation (dccrba) and the condensing — by the C-level gate, on a
    // trust-region rejection that comes after both.
    {
      rtc::testing::ScopedAllocGate new_gate;
      rtc::testing::ScopedMallocGate malloc_gate;
      const bool ok = core.Solve(warm, r);
      EXPECT_TRUE(ok);
      EXPECT_EQ(new_gate.count(), 0U) << "operator new inside Solve";
      (void)malloc_gate.count();  // ProxQP's own C mallocs (#654) — counted, not asserted
    }
    // The same input again, with no other term that rebuilds H: the angular
    // term is rebuilt on h_main_, not added to the last solve's.
    EXPECT_TRUE(SameBits(core.MainQp().H, qp_on.H));
    MpcSegmentCoreInput outside = warm;
    for (int k = 2; k <= kNStop; ++k) {
      outside.q_ref(kNv - 1, k) = f.limits.q_max[kNv - 1] + 3.0 * on.delta_tr;
    }
    {
      rtc::testing::ScopedAllocGate new_gate;
      rtc::testing::ScopedMallocGate malloc_gate;
      const bool ok = core.Solve(outside, r);
      EXPECT_FALSE(ok);
      EXPECT_EQ(r.reason, MpcSegmentCoreReason::kTrustRegionConflict);
      EXPECT_GT(r.linearize_us, 0.0) << "the rejection must come after the linearisation";
      EXPECT_EQ(new_gate.count(), 0U);
      EXPECT_EQ(malloc_gate.count(), 0U) << "C-level allocation up to the QP";
    }
  }
  EXPECT_GT((qp_on.H - qp_off.H).cwiseAbs().maxCoeff(), 0.0)
      << "the term must be in H with no other linearised term on";
}

// ── 6. Record: the QP's iterations with and without the terms ────────────────
// Stop rule: the terms on may not more than double the ProxQP iterations of
// the same input. Recorded here; judged in the issue.
TEST(MpcSegmentCoreWholeBodyRecord, IterationsWithAndWithoutTheTerms) {
  const Fixture f = MakeFixture();
  const MpcSegmentCoreParams off = ThreePartitions();
  MpcSegmentCoreParams on = off;
  SetWholeBody(on, 0.1, 1.0, f.arm.q_nominal);
  SetAngular(on);
  const TwoCycles a = SolveTwice(f, off);
  const TwoCycles b = SolveTwice(f, on);
  for (std::size_t c = 0; c < 2; ++c) {
    RecordProperty("iterations_off_" + std::to_string(c), a.result[c].iterations);
    RecordProperty("iterations_on_" + std::to_string(c), b.result[c].iterations);
    std::printf("[iterations] cycle %zu: off %d, on %d\n", c, a.result[c].iterations,
                b.result[c].iterations);
  }
}

// ── 7. Judgement ────────────────────────────────────────────────────────────
// The other arm (group 2) is the group a plan leaves out: held at its rest
// posture by w_rest, moved only by the angular-momentum term. Its x_0 is its
// rest posture, at rest.

// The judgement's values: W⁻ / W⁺ = 0.001 / 0.1, W_big = 100·W⁺ for the negative
// control; the rest weight 1 for the arm a plan leaves out, 100 for the
// locked limit's comparison.
constexpr double kJudgePre = 0.001;
constexpr double kJudgePost = 0.1;
constexpr double kWBig = 100.0 * kJudgePost;
constexpr double kRestWeight = 1.0;
constexpr double kRestBig = 100.0;

bool OtherArm(int m) {
  return GroupOf(m) == 2;
}

struct JudgeRun {
  MpcSegmentCoreResult r;
  bool ok{false};
};

// One cold first solve of the fixture's reach (the other arm's reference held
// at its rest posture) with the other arm's rest weight, the angular weights
// and, when `locked`, the other arm's box of width 0 at its x_0.
JudgeRun Judge(const Fixture& f, double w_rest, double w_pre, double w_post, bool locked) {
  MpcSegmentCoreParams p = ThreePartitions();
  p.w_rest = Eigen::VectorXd::Zero(kNv);
  p.q_rest = f.arm.q_nominal;
  for (int m = 0; m < kNv; ++m) {
    if (OtherArm(m)) {
      p.w_rest[m] = w_rest;
    }
  }
  if (w_rest == 0.0) {
    p.w_rest.resize(0);
  }
  if (w_pre > 0.0 || w_post > 0.0) {
    SetAngular(p, w_pre, w_post);
  }
  rtc::catching::MpcSegmentCoreLimits limits = f.limits;
  if (locked) {
    for (int m = 0; m < kNv; ++m) {
      if (OtherArm(m)) {
        limits.q_min[m] = limits.q_max[m] = f.arm.q_nominal[m];
      }
    }
  }
  JudgeRun out;
  MpcSegmentCore core;
  if (core.Init(*f.arm.model, f.arm.frame, p, limits) != MpcSegmentCoreReason::kNone) {
    return out;
  }
  MpcSegmentCoreInput in = f.input;
  ReachReference(core, f.arm.q_nominal, f.q_target, in);
  for (int m = 0; m < kNv; ++m) {
    if (OtherArm(m)) {
      in.q_ref.row(m).setConstant(f.arm.q_nominal[m]);
      in.qd_ref.row(m).setZero();
      in.qdd_ref.row(m).setZero();
    }
  }
  core.ResizeResult(out.r);
  out.ok = core.Solve(in, out.r);
  return out;
}

double OtherArmOffRest(const Fixture& f, const MpcSegmentCoreResult& r) {
  double d = 0.0;
  for (int k = 0; k < r.q.cols(); ++k) {
    for (int m = 0; m < kNv; ++m) {
      if (OtherArm(m)) {
        d = std::max(d, std::abs(r.q(m, k) - f.arm.q_nominal[m]));
      }
    }
  }
  return d;
}

double ChainDifference(const MpcSegmentCoreResult& a, const MpcSegmentCoreResult& b) {
  double d = 0.0;
  for (int m = 0; m < kNv; ++m) {
    if (!OtherArm(m)) {
      d = std::max(d, (a.q.row(m) - b.q.row(m)).cwiseAbs().maxCoeff());
    }
  }
  return d;
}

struct CatchNode {
  Eigen::Vector3d pos_err, v_rel;
  double axis_err{0.0};
};

CatchNode CatchOf(const MpcSegmentCoreResult& r) {
  return {r.catch_pos_err, r.catch_v_rel, r.catch_axis_err};
}

// The tolerance on the catch-node values against the solve without the
// whole-body terms.
bool WithinD2(const CatchNode& a, const CatchNode& b, std::string* why) {
  const double dp = (a.pos_err - b.pos_err).norm();
  const double da = std::abs(a.axis_err - b.axis_err);
  const double dv = (a.v_rel - b.v_rel).norm();
  *why = "position " + std::to_string(dp) + " m, axis " + std::to_string(da) + " rad, v_rel " +
         std::to_string(dv) + " m/s";
  return dp <= 1e-3 && da <= 0.0175 && dv <= 0.02;
}

// The torque rows already span the whole model: a trunk joint's row carries
// the other arm's motion (its reaction), seen against the condensed map's own
// finite difference — z → nodes → RNEA.
TEST(MpcSegmentCoreJudgement, TheTrunksTorqueRowsCarryTheOtherArmsReaction) {
  const Fixture f = MakeFixture();
  MpcSegmentCoreParams p = ThreePartitions();
  MpcSegmentCore core;
  ASSERT_EQ(core.Init(*f.arm.model, f.arm.frame, p, f.limits), MpcSegmentCoreReason::kNone);
  // The reference is x_0 at rest, so it is the trajectory of z = 0 and the
  // linearisation point of the rows is where the difference is taken.
  MpcSegmentCoreInput in = RestInput(f.arm.q_nominal);
  in.q_ref = f.arm.q_nominal.replicate(1, core.NumNodes() + 1);
  in.qd_ref.setZero(kNv, core.NumNodes() + 1);
  in.qdd_ref.setZero(kNv, core.NumNodes() + 1);
  in.reference_valid = true;
  in.p_b = f.input.p_b;
  in.a_d = f.input.a_d;
  in.v_b = f.input.v_b;
  in.w_p = f.input.w_p;
  in.p_c = in.p_b;
  in.d_hat = f.input.d_hat;
  MpcSegmentCoreResult r;
  core.ResizeResult(r);
  ASSERT_TRUE(core.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason);
  const rtc::tsid::QPData& qp = core.MainQp();
  const rtc::catching::BlockLayout& l = core.Layout();
  const int N = core.NumNodes();
  const int nN = kNv * N;
  pinocchio::Data data(*f.arm.model);
  // Nodes of z, from x_0 at rest, through the stage gains and the layout.
  const auto nodes = [&](const Eigen::VectorXd& z, int k, Eigen::VectorXd& q, Eigen::VectorXd& v,
                         Eigen::VectorXd& a) {
    q = f.arm.q_nominal;
    v.setZero(kNv);
    a.setZero(kNv);
    for (int j = 0; j < kNv; ++j) {
      const int c = l.class_of[static_cast<std::size_t>(j)];
      for (int b = 0; b < l.pattern[static_cast<std::size_t>(c)].n_blocks; ++b) {
        const double zb = z[l.Col(j, b)];
        q[j] += core.StageGain(0, k, b, c) * zb;
        v[j] += core.StageGain(1, k, b, c) * zb;
        a[j] += core.StageGain(2, k, b, c) * zb;
      }
    }
  };
  const double eps = 1e-6;
  double worst = 0.0;
  double largest = 0.0;
  for (int k = 1; k <= N; ++k) {
    for (int m = kTrunk + kArm; m < kNv; ++m) {  // the other arm's variables
      const int c = l.class_of[static_cast<std::size_t>(m)];
      for (int b = 0; b < l.pattern[static_cast<std::size_t>(c)].n_blocks; ++b) {
        const int col = l.Col(m, b);
        Eigen::VectorXd zp = Eigen::VectorXd::Zero(l.nu);
        Eigen::VectorXd zm = zp;
        zp[col] = eps;
        zm[col] = -eps;
        Eigen::VectorXd q, v, a;
        nodes(zp, k, q, v, a);
        const Eigen::VectorXd tp = pinocchio::rnea(*f.arm.model, data, q, v, a);
        nodes(zm, k, q, v, a);
        const Eigen::VectorXd tm = pinocchio::rnea(*f.arm.model, data, q, v, a);
        for (int t = 0; t < kTrunk; ++t) {
          const double fd = (tp[t] - tm[t]) / (2.0 * eps) / f.limits.tau_max[t];
          const double row = qp.C(2 * nN + (k - 1) * kNv + t, col);
          worst = std::max(worst, std::abs(row - fd));
          largest = std::max(largest, std::abs(fd));
        }
      }
    }
  }
  EXPECT_GT(largest, 1e-6) << "the reaction must be there";
  EXPECT_LE(worst, 1e-6 * std::max(1.0, largest));
}

// A large rest weight is the arm locked.
TEST(MpcSegmentCoreJudgement, ALargeRestWeightIsTheOtherArmLocked) {
  const Fixture f = MakeFixture();
  ASSERT_TRUE(f.input.qd0.isZero(0.0));
  const JudgeRun held = Judge(f, kRestBig, kJudgePre, kJudgePost, false);
  const JudgeRun locked = Judge(f, 0.0, kJudgePre, kJudgePost, true);
  const JudgeRun free = Judge(f, 0.0, kJudgePre, kJudgePost, false);
  ASSERT_TRUE(held.ok && locked.ok && free.ok);
  // The same chain solution, to the chain solve's tolerance …
  EXPECT_LE(ChainDifference(held.r, locked.r), 1e-4);
  EXPECT_LE((held.r.catch_pos_err - locked.r.catch_pos_err).norm(), 1e-4);
  EXPECT_LE(OtherArmOffRest(f, locked.r), 1e-6) << "locked: to the QP's tolerance";
  // … the arm converging on the locked one as its weight grows (the
  // limit, measured as a monotone fall to ≤ 1e-4 rad at the weight bound) …
  double last = std::numeric_limits<double>::infinity();
  for (const double w : {1.0, 10.0, 100.0, 1e3, rtc::catching::kWholeBodyWeightMax}) {
    const JudgeRun r = Judge(f, w, kJudgePre, kJudgePost, false);
    ASSERT_TRUE(r.ok) << w;
    const double off = OtherArmOffRest(f, r.r);
    RecordProperty("other_arm_off_rest_at_" + std::to_string(static_cast<int>(w)),
                   std::to_string(off));
    EXPECT_LT(off, last) << "w_rest " << w;
    last = off;
  }
  EXPECT_LE(last, 1e-4) << "at the weight bound";
  // … and without the weight the term does move it: the case can tell.
  EXPECT_GT(OtherArmOffRest(f, free.r), 1e-3);
}

// With the other arm free the angular term swings it
// against the reach, and the catch hardly notices; locked and weighted
// heavily, the reach itself has to give.
TEST(MpcSegmentCoreJudgement, TheOtherArmSwingsAgainstTheReachAndTheCatchHardlyNotices) {
  const Fixture f = MakeFixture();
  ASSERT_TRUE(f.input.qd0.isZero(0.0));  // the premise: the other arm at rest at its rest posture
  const JudgeRun off = Judge(f, kRestWeight, 0.0, 0.0, false);
  const JudgeRun swing = Judge(f, kRestWeight, kJudgePre, kJudgePost, false);
  const JudgeRun locked = Judge(f, kRestWeight, kJudgePre, kJudgePost, true);
  const JudgeRun locked_big = Judge(f, kRestWeight, kWBig, kWBig, true);
  ASSERT_TRUE(off.ok && swing.ok && locked.ok && locked_big.ok);
  std::string why;
  // The control: no angular term, nothing moves the other arm.
  EXPECT_LE(OtherArmOffRest(f, off.r), 1e-6);
  // Positive: it swings, within the trust region per node …
  EXPECT_GT(OtherArmOffRest(f, swing.r), 1e-3);
  EXPECT_LT(swing.r.whole_body.ang, locked.r.whole_body.ang)
      << "free, the arm must lower the term the core minimises";
  RecordProperty("swing_ang_rate_max", std::to_string(swing.r.whole_body.ang_rate_max));
  RecordProperty("locked_ang_rate_max", std::to_string(locked.r.whole_body.ang_rate_max));
  // … and the catch node is within tolerance of the solve without it.
  EXPECT_TRUE(WithinD2(CatchOf(swing.r), CatchOf(off.r), &why)) << why;
  RecordProperty("swing_catch_difference", why);
  // Negative: locked, with the term weighted heavily on both phases, the
  // reach gives — the tolerance is not met.
  EXPECT_FALSE(WithinD2(CatchOf(locked_big.r), CatchOf(off.r), &why)) << why;
  RecordProperty("locked_big_catch_difference", why);
}

}  // namespace
