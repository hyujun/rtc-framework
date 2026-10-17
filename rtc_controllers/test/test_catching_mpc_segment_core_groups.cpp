// E3-F02 (#641): MpcSegmentCore with joint groups — per-group move blocking
// (formulation §1.1, MD-49; D11) on a synthetic tree (a trunk and two arms off
// it, 17 joints, the catch frame on the first arm). The judged properties:
//   • groups that all have one partition are the problem without groups, to
//     the bit (the layout folds them into one block class);
//   • with different partitions the structured assembly matches the dense
//     oracle, the stage gains match the integral, the terminal equality keeps
//     its rank and every joint's jerk is constant on ITS group's blocks;
//   • a partition is checked per group (catch node, three blocks after it).
#include "rtc_controllers/catching/mpc_block_grid.hpp"
#include "rtc_controllers/catching/mpc_segment_core.hpp"
#include "rtc_controllers/testing/mpc_segment_core_fixture.hpp"
#include "rtc_controllers/testing/synthetic_tree_fixture.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
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
  EXPECT_EQ(core.MainQp().H.rows(), l.nu + kNv * (kNPre + kNStop) + 1);
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

}  // namespace
