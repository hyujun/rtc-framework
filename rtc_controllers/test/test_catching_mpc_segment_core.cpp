// MPC segment core — jerk-input condensed QP with first-order torque rows and
// slack (mpc_segment_core.hpp). The criteria map to named tests here:
//   1 ZeroSolutionAtRest                 5 TheAllocationGatesAreArmed,
//   2 ReducesToMinJerkClosedForm,          CoreAllocatesNothingOutsideTheQpSolver,
//     PresolveMatchesClosedForm            PresolveAndRejectionPathsAllocateNothing
//   3 SlackIsZeroWhenFeasible            6 Rejects* / *FailsClosed
//   4 TerminalEqualityIsFullRank         7 ReevaluatedTorqueWithinLimits
//                                        8 TimingDistribution*
// plus the seam checks (derivatives, offset sign, armature) and the
// structured-vs-dense assembly oracle for the optimisation.
//
// The allocation gates: this binary links THREE sensors (CMakeLists note).
// rtc_base's malloc_gate.hpp is the one that can see pinocchio's and ProxQP's own
// allocations; its positive control allocates inside pinocchio.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_controllers/catching/jerk_segment.hpp"
#include "rtc_controllers/catching/mpc_segment_core.hpp"
#include "rtc_controllers/catching/mpc_segment_core_torque.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/mpc_segment_core_fixture.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {

using rtc::catching::MpcSegmentCore;
using rtc::catching::MpcSegmentCoreInput;
using rtc::catching::MpcSegmentCoreLimits;
using rtc::catching::MpcSegmentCoreParams;
using rtc::catching::MpcSegmentCoreReason;
using rtc::catching::MpcSegmentCoreReasonName;
using rtc::catching::MpcSegmentCoreResult;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// ── Fixtures ─────────────────────────────────────────────────────────────────

// Arms, limits and the rest input live in mpc_segment_core_fixture.hpp, shared with
// the approach suite.
using rtc::testing::mpc_segment_core::ArmModel;
using rtc::testing::mpc_segment_core::LimitsFromModel;
using rtc::testing::mpc_segment_core::Percentile;
using rtc::testing::mpc_segment_core::Rank;
using rtc::testing::mpc_segment_core::RealArm6;
using rtc::testing::mpc_segment_core::RealArm7;
using rtc::testing::mpc_segment_core::RecordMicros;
using rtc::testing::mpc_segment_core::RestInput;
using rtc::testing::mpc_segment_core::Synthetic6R;
using rtc::testing::mpc_segment_core::UseAsReference;

// A solved trajectory shifted by `t_shift` onto the same grid (the warm start): node k of the new
// reference is the old trajectory at t_shift+kΔ.
void ShiftReference(const MpcSegmentCoreResult& r, double dt, double t_shift,
                    MpcSegmentCoreInput& in) {
  const Eigen::Index n = r.q.rows();
  const Eigen::Index cols = r.q.cols();
  in.q_ref.resize(n, cols);
  in.qd_ref.resize(n, cols);
  in.qdd_ref.resize(n, cols);
  Eigen::VectorXd q(n), qd(n), qdd(n);
  for (Eigen::Index k = 0; k < cols; ++k) {
    const bool ok = rtc::catching::SampleJerkTrajectory(
        r.q, r.qd, r.qdd, dt, t_shift + static_cast<double>(k) * dt, q, qd, qdd);
    EXPECT_TRUE(ok);
    in.q_ref.col(k) = q;
    in.qd_ref.col(k) = qd;
    in.qdd_ref.col(k) = qdd;
  }
  in.reference_valid = true;
  // x_0 is where the old plan says the arm is now.
  in.q0 = in.q_ref.col(0);
  in.qd0 = in.qd_ref.col(0);
  in.qdd0 = in.qdd_ref.col(0);
}

Eigen::VectorXd Rnea(const pinocchio::Model& m, pinocchio::Data& d, const Eigen::VectorXd& q,
                     const Eigen::VectorXd& v, const Eigen::VectorXd& a) {
  return pinocchio::rnea(m, d, q, v, a);
}

// ── 1. Zero solution at rest ─────────────────────────────────────────────────

TEST(MpcSegmentCore, ZeroSolutionAtRest) {
  const ArmModel arm = Synthetic6R();
  MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model);
  // Precondition: gravity at q0 is inside η'τ_max, otherwise the torque rows
  // would (correctly) demand slack and the answer is not zero.
  pinocchio::Data data(*arm.model);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(arm.model->nv);
  const Eigen::VectorXd g = Rnea(*arm.model, data, arm.q_nominal, zero, zero);
  MpcSegmentCoreParams p;
  lim.tau_max = lim.tau_max.cwiseMax(3.0 * g.cwiseAbs() / p.eta_tau);

  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);

  const double z_tol = 10.0 * p.solver.eps_abs;
  for (const bool with_reference : {false, true}) {
    MpcSegmentCoreInput in = RestInput(arm.q_nominal);
    if (with_reference) {
      in.q_ref = arm.q_nominal.replicate(1, p.n_nodes + 1);
      in.qd_ref.setZero(arm.model->nv, p.n_nodes + 1);
      in.qdd_ref.setZero(arm.model->nv, p.n_nodes + 1);
      in.reference_valid = true;
    }
    ASSERT_TRUE(mpc.Solve(in, res)) << MpcSegmentCoreReasonName(res.reason);
    EXPECT_EQ(res.presolved, !with_reference);
    EXPECT_LE(res.u.cwiseAbs().maxCoeff(), z_tol * p.u_scale) << "reference=" << with_reference;
    EXPECT_LE(res.slack.cwiseAbs().maxCoeff(), z_tol);
    for (int k = 0; k <= p.n_nodes; ++k) {
      EXPECT_LE((res.q.col(k) - arm.q_nominal).cwiseAbs().maxCoeff(), 1e-6);
    }
    EXPECT_LE(res.qd.cwiseAbs().maxCoeff(), 1e-6);
  }
}

// ── 2. Reduction to the per-joint min-jerk closed form ───────────────────────
// With every limit inactive, ρ_τ = 0, w_Δ = w_⊥ = 0, the stationary point of
// ½ Σ_b R n_b u_b² subject to a_N = 0, v_N = 0 is u_b = α + β·m_b with
// m_b = mean_{k∈b}(N − k − ½); (α, β) solve the 2×2 system below. The n_b
// weighting is what distinguishes EᵀRE from an unweighted Σ u_b².

Eigen::MatrixXd ClosedFormJerk(const MpcSegmentCoreParams& p, const Eigen::VectorXd& v0,
                               const Eigen::VectorXd& a0) {
  const int N = p.n_nodes;
  const double dt = p.dt;
  std::vector<double> nb, mb;
  int k0 = 0;
  for (int b = 0; b < p.n_blocks; ++b) {
    const int size = p.block_sizes[static_cast<std::size_t>(b)];
    double sum = 0.0;
    for (int k = k0; k < k0 + size; ++k) {
      sum += N - k - 0.5;
    }
    nb.push_back(size);
    mb.push_back(sum / size);
    k0 += size;
  }
  double s0 = 0.0, s1 = 0.0, s2 = 0.0;
  for (std::size_t b = 0; b < nb.size(); ++b) {
    s0 += nb[b];
    s1 += nb[b] * mb[b];
    s2 += nb[b] * mb[b] * mb[b];
  }
  Eigen::MatrixXd u(v0.size(), N);
  for (Eigen::Index j = 0; j < v0.size(); ++j) {
    Eigen::Matrix2d m;
    m << s0, s1, s1, s2;
    Eigen::Vector2d rhs(-a0[j] / dt, -(v0[j] + N * dt * a0[j]) / (dt * dt));
    const Eigen::Vector2d ab = m.fullPivLu().solve(rhs);
    int k = 0;
    for (std::size_t b = 0; b < nb.size(); ++b) {
      for (int i = 0; i < static_cast<int>(nb[b]); ++i) {
        u(j, k++) = ab[0] + ab[1] * mb[b];
      }
    }
  }
  return u;
}

MpcSegmentCoreParams KinematicOnlyParams() {
  MpcSegmentCoreParams p;
  p.rho_tau = 0.0;
  p.w_delta = 0.0;
  p.w_perp = 0.0;
  p.delta_tr = kInf;
  return p;
}

TEST(MpcSegmentCore, ReducesToMinJerkClosedForm) {
  const ArmModel arm = Synthetic6R();
  const MpcSegmentCoreParams p = KinematicOnlyParams();
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)),
            MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);

  MpcSegmentCoreInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.30, -0.25, 0.20, -0.15, 0.35, -0.30;
  in.qdd0 << 1.0, -0.5, 0.8, 0.0, -1.2, 0.4;
  // A resting reference that is NOT the answer: the main solve (not the
  // pre-solve) has to produce the closed form.
  in.q_ref = arm.q_nominal.replicate(1, p.n_nodes + 1);
  in.qd_ref.setZero(6, p.n_nodes + 1);
  in.qdd_ref.setZero(6, p.n_nodes + 1);
  in.reference_valid = true;

  ASSERT_TRUE(mpc.Solve(in, res)) << MpcSegmentCoreReasonName(res.reason);
  EXPECT_FALSE(res.presolved);
  const Eigen::MatrixXd u_ref = ClosedFormJerk(p, in.qd0, in.qdd0);
  const double scale = u_ref.cwiseAbs().maxCoeff();
  EXPECT_LE((res.u - u_ref).cwiseAbs().maxCoeff(), 1e-6 * scale) << "MPC\n"
                                                                 << res.u << "\nclosed form\n"
                                                                 << u_ref;
  EXPECT_LE(res.qd.col(p.n_nodes).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_LE(res.qdd.col(p.n_nodes).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(MpcSegmentCore, PresolveMatchesClosedForm) {
  const ArmModel arm = Synthetic6R();
  const MpcSegmentCoreParams p = KinematicOnlyParams();
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)),
            MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  MpcSegmentCoreInput in = RestInput(arm.q_nominal);
  in.qd0 << -0.2, 0.3, -0.1, 0.25, -0.3, 0.15;
  ASSERT_TRUE(mpc.Solve(in, res)) << MpcSegmentCoreReasonName(res.reason);
  EXPECT_TRUE(res.presolved);
  const Eigen::MatrixXd u_ref = ClosedFormJerk(p, in.qd0, in.qdd0);
  EXPECT_LE((res.u - u_ref).cwiseAbs().maxCoeff(), 1e-6 * u_ref.cwiseAbs().maxCoeff());
}

// ── 4. Terminal equality rank; stage gains ───────────────────────────────────

// Response of the scalar triple integrator at t = kΔ to unit jerk on
// [iΔ, (i+1)Δ] — the integral form, independent of the core's recursion.
double UnitJerkResponse(int m, int k, int i, double dt) {
  if (i >= k) {
    return 0.0;
  }
  const double t0 = (k - i) * dt;
  const double t1 = (k - i - 1) * dt;
  switch (m) {
    case 0:
      return (t0 * t0 * t0 - t1 * t1 * t1) / 6.0;
    case 1:
      return (t0 * t0 - t1 * t1) / 2.0;
    default:
      return dt;
  }
}

// [G_q̇,N; G_q̈,N] for an arbitrary block pattern, from the integral oracle.
Eigen::MatrixXd TerminalMatrixOracle(int n, int n_nodes, const std::vector<int>& blocks,
                                     double dt) {
  const int nb = static_cast<int>(blocks.size());
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(2 * n, n * nb);
  int k0 = 0;
  for (int b = 0; b < nb; ++b) {
    double gv = 0.0, ga = 0.0;
    for (int i = k0; i < k0 + blocks[static_cast<std::size_t>(b)]; ++i) {
      gv += UnitJerkResponse(1, n_nodes, i, dt);
      ga += UnitJerkResponse(2, n_nodes, i, dt);
    }
    for (int j = 0; j < n; ++j) {
      a(j, b * n + j) = gv;
      a(n + j, b * n + j) = ga;
    }
    k0 += blocks[static_cast<std::size_t>(b)];
  }
  return a;
}

// The same per joint (joint groups): joint j on its own group's blocks,
// its columns where `col(j, b)` puts them.
Eigen::MatrixXd TerminalMatrixOracle(const std::vector<std::vector<int>>& blocks_of_joint,
                                     int n_nodes, double dt,
                                     const std::function<Eigen::Index(int, int)>& col,
                                     Eigen::Index nu) {
  const int n = static_cast<int>(blocks_of_joint.size());
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(2 * n, nu);
  for (int j = 0; j < n; ++j) {
    const std::vector<int>& blocks = blocks_of_joint[static_cast<std::size_t>(j)];
    const Eigen::MatrixXd one = TerminalMatrixOracle(1, n_nodes, blocks, dt);
    for (int b = 0; b < static_cast<int>(blocks.size()); ++b) {
      a(j, col(j, b)) = one(0, b);
      a(n + j, col(j, b)) = one(1, b);
    }
  }
  return a;
}

TEST(MpcSegmentCore, StageGainsMatchIntegralOracle) {
  const ArmModel arm = Synthetic6R();
  const MpcSegmentCoreParams p;
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)),
            MpcSegmentCoreReason::kNone);
  for (int m = 0; m < 3; ++m) {
    for (int k = 0; k <= p.n_nodes; ++k) {
      int k0 = 0;
      for (int b = 0; b < p.n_blocks; ++b) {
        double oracle = 0.0;
        for (int i = k0; i < k0 + p.block_sizes[static_cast<std::size_t>(b)]; ++i) {
          oracle += UnitJerkResponse(m, k, i, p.dt);
        }
        k0 += p.block_sizes[static_cast<std::size_t>(b)];
        EXPECT_NEAR(mpc.StageGain(m, k, b), p.u_scale * oracle,
                    1e-12 * p.u_scale + 1e-9 * std::abs(p.u_scale * oracle))
            << "m=" << m << " k=" << k << " b=" << b;
      }
    }
  }
}

TEST(MpcSegmentCore, TerminalEqualityIsFullRank) {
  const ArmModel arm = Synthetic6R();
  const int n = arm.model->nv;
  // The rank helper itself: one block → rank n; two blocks → rank 2n and no
  // freedom left (nullity 0) — that is why B ≥ 3 is required, not the rank.
  EXPECT_EQ(Rank(TerminalMatrixOracle(n, 12, {12}, 0.05)), n);
  EXPECT_EQ(Rank(TerminalMatrixOracle(n, 12, {6, 6}, 0.05)), 2 * n);
  EXPECT_EQ(TerminalMatrixOracle(n, 12, {6, 6}, 0.05).cols() - 2 * n, 0);

  for (const std::vector<int>& blocks :
       {std::vector<int>{4, 4, 4}, std::vector<int>{1, 1, 2, 2, 3, 3}}) {
    MpcSegmentCoreParams p;
    p.n_blocks = static_cast<int>(blocks.size());
    std::copy(blocks.begin(), blocks.end(), p.block_sizes.begin());
    MpcSegmentCore mpc;
    ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)),
              MpcSegmentCoreReason::kNone);
    EXPECT_EQ(mpc.TerminalRank(), 2 * n);
    const Eigen::MatrixXd a = mpc.MainQp().A;
    EXPECT_EQ(Rank(a), 2 * n) << "B=" << blocks.size();
    // Assembled matrix = oracle (scaled by u_scale).
    EXPECT_LE(
        (a - p.u_scale * TerminalMatrixOracle(n, p.n_nodes, blocks, p.dt)).cwiseAbs().maxCoeff(),
        1e-9 * p.u_scale);
  }
  // Two joint groups on partitions of their own: each joint's rows hold its
  // group's integrals, in the columns the layout gives that joint.
  {
    const std::vector<int> fine{1, 1, 2, 2, 3, 3};
    const std::vector<int> coarse{4, 4, 4};
    MpcSegmentCoreParams p;
    p.n_groups = 2;
    std::vector<std::vector<int>> blocks_of_joint;
    for (int j = 0; j < n; ++j) {
      p.joint_group[static_cast<std::size_t>(j)] = static_cast<std::int8_t>(j < 2 ? 0 : 1);
      blocks_of_joint.push_back(j < 2 ? coarse : fine);
    }
    for (std::size_t g = 0; g < 2; ++g) {
      const std::vector<int>& sizes = g == 0 ? coarse : fine;
      p.group_blocks[g].n_blocks = static_cast<int>(sizes.size());
      std::copy(sizes.begin(), sizes.end(), p.group_blocks[g].block_sizes.begin());
    }
    MpcSegmentCore mpc;
    ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)),
              MpcSegmentCoreReason::kNone);
    ASSERT_FALSE(mpc.Layout().Single());
    EXPECT_EQ(mpc.TerminalRank(), 2 * n);
    const Eigen::MatrixXd a = mpc.MainQp().A;
    const Eigen::Index nu = mpc.Layout().nu;
    ASSERT_EQ(nu, 2 * 3 + (n - 2) * 6);
    const Eigen::MatrixXd oracle = TerminalMatrixOracle(
        blocks_of_joint, p.n_nodes, p.dt,
        [&](int j, int b) { return static_cast<Eigen::Index>(mpc.Layout().Col(j, b)); }, nu);
    EXPECT_LE((a.leftCols(nu) - p.u_scale * oracle).cwiseAbs().maxCoeff(), 1e-9 * p.u_scale);
    EXPECT_EQ(a.rightCols(a.cols() - nu).cwiseAbs().maxCoeff(), 0.0);
  }
  MpcSegmentCoreParams two;
  two.n_blocks = 2;
  two.block_sizes[0] = 6;
  two.block_sizes[1] = 6;
  MpcSegmentCore mpc;
  EXPECT_EQ(mpc.Init(*arm.model, arm.frame, two, LimitsFromModel(*arm.model)),
            MpcSegmentCoreReason::kBlocksTooFew);
}

// ── Torque fixture: rows ACTIVE and the hard problem feasible ────────────────
// The real 6-dof arm's first joint has a vertical axis, so its torque is purely
// inertial: the stop's peak torque on it can be shaved by redistributing jerk
// over the fixed horizon. τ_max(0) is set 5 % below the unconstrained peak so
// its row must bind; every other joint keeps a loose limit.

struct TorqueFixture {
  ArmModel arm;
  MpcSegmentCoreParams params;
  MpcSegmentCoreLimits limits;
  MpcSegmentCoreInput input;
};

TorqueFixture MakeTorqueFixture(double armature = 0.0) {
  TorqueFixture f{RealArm6(), MpcSegmentCoreParams{}, {}, {}};
  const pinocchio::Model& m = *f.arm.model;
  f.limits = LimitsFromModel(m, armature);
  f.limits.tau_max *= 10.0;
  f.input = RestInput(f.arm.q_nominal);
  f.input.qd0 << 1.2, -0.6, 0.8, 0.5, 0.4, 0.4;

  MpcSegmentCore loose;
  EXPECT_EQ(loose.Init(m, f.arm.frame, f.params, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult r;
  loose.ResizeResult(r);
  EXPECT_TRUE(loose.Solve(f.input, r)) << MpcSegmentCoreReasonName(r.reason);
  double peak = 0.0;
  for (int k = 1; k <= f.params.n_nodes; ++k) {
    peak = std::max(peak, std::abs(r.tau_ratio(0, k - 1)) * f.limits.tau_max[0]);
  }
  f.limits.tau_max[0] = peak / (f.params.eta_tau * 1.05);
  return f;
}

// Solve the fixture's last main QP with the slack pinned to 0 (the hard
// problem). Returns false when it is infeasible. The N slack variables (#857)
// follow the jerk ones and their s ≥ 0 rows are the last rows — true only
// without a velocity slack, whose column and rows would be pinned instead.
bool SolveHard(const MpcSegmentCore& mpc, const MpcSegmentCoreParams& p, Eigen::VectorXd& z) {
  rtc::tsid::QPData hard = mpc.MainQp();
  const Eigen::Index n_in = hard.C.rows();
  const Eigen::Index n_s = hard.H.rows() - mpc.Layout().nu;
  EXPECT_EQ(n_s, mpc.NumNodes()) << "SolveHard assumes no velocity slack";
  if (n_s != mpc.NumNodes()) {
    return false;
  }
  hard.u.tail(n_s).setZero();
  hard.g.tail(n_s).setZero();
  rtc::tsid::QPSolverWrapper solver;
  solver.Init(static_cast<int>(hard.H.rows()), static_cast<int>(hard.A.rows()),
              static_cast<int>(n_in), p.solver);
  const rtc::tsid::SolveResult& res = solver.Solve(hard);
  z = res.x_opt;
  return res.converged;
}

Eigen::MatrixXd JerkFromZ(const Eigen::VectorXd& z, const MpcSegmentCoreParams& p, int n) {
  Eigen::MatrixXd u(n, p.n_nodes);
  int k = 0;
  for (int b = 0; b < p.n_blocks; ++b) {
    for (int i = 0; i < p.block_sizes[static_cast<std::size_t>(b)]; ++i, ++k) {
      for (int j = 0; j < n; ++j) {
        u(j, k) = p.u_scale * z[b * n + j];
      }
    }
  }
  return u;
}

// ── 3. Feasible ⇒ zero slack ─────────────────────────────────────────────────

TEST(MpcSegmentCore, SlackIsZeroWhenFeasible) {
  const TorqueFixture f = MakeTorqueFixture();
  const int n = f.arm.model->nv;
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*f.arm.model, f.arm.frame, f.params, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  ASSERT_TRUE(mpc.Solve(f.input, res)) << MpcSegmentCoreReasonName(res.reason);

  // The row binds (otherwise "slack is zero" would be vacuous) …
  EXPECT_GE(res.tau_ratio_max, f.params.eta_tau - 1e-4);
  // … the slack is zero …
  EXPECT_LE(res.slack_max, 10.0 * f.params.solver.eps_abs);
  // … and the answer is the hard problem's.
  Eigen::VectorXd z_hard;
  ASSERT_TRUE(SolveHard(mpc, f.params, z_hard));
  const Eigen::MatrixXd u_hard = JerkFromZ(z_hard, f.params, n);
  EXPECT_LE((res.u - u_hard).cwiseAbs().maxCoeff(), 1e-3 * u_hard.cwiseAbs().maxCoeff());

  // Mutation: a penalty below the row multiplier makes slack cheaper than
  // obeying the limit — this test must see it.
  {
    MpcSegmentCoreParams cheap = f.params;
    cheap.rho_tau = 1e-8;
    MpcSegmentCore m2;
    ASSERT_EQ(m2.Init(*f.arm.model, f.arm.frame, cheap, f.limits), MpcSegmentCoreReason::kNone);
    MpcSegmentCoreResult r2;
    m2.ResizeResult(r2);
    ASSERT_TRUE(m2.Solve(f.input, r2)) << MpcSegmentCoreReasonName(r2.reason);
    EXPECT_GT(r2.slack_max, 1e-3) << "ρ_τ too small must leave slack";
  }
  // Positive control: a static-torque limit below gravity at the stop is
  // infeasible for the hard problem; the soft one reports terminal slack.
  {
    MpcSegmentCoreLimits tight = f.limits;
    pinocchio::Data data(*f.arm.model);
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n);
    const Eigen::VectorXd g = Rnea(*f.arm.model, data, f.arm.q_nominal, zero, zero);
    tight.tau_max[1] = 0.5 * std::abs(g[1]) / f.params.eta_tau;
    MpcSegmentCore m3;
    ASSERT_EQ(m3.Init(*f.arm.model, f.arm.frame, f.params, tight), MpcSegmentCoreReason::kNone);
    MpcSegmentCoreResult r3;
    m3.ResizeResult(r3);
    ASSERT_TRUE(m3.Solve(f.input, r3)) << MpcSegmentCoreReasonName(r3.reason);
    EXPECT_GT(r3.slack_terminal_max, 0.1);
    Eigen::VectorXd z3;
    EXPECT_FALSE(SolveHard(m3, f.params, z3));
  }
}

// ── #857: one torque slack per node, against a per-joint oracle ─────────────
// The oracle lays a core QP's torque slack out again — one per joint and node,
// or one per node — around the SAME jerk block (H, g, A, b and the position,
// velocity and torque rows' jerk columns with their l, u copied), and solves it
// with its own QPSolverWrapper to kOracleEps. Every count is filled in:
// QPData::Init leaves n_ineq at 0, and the wrapper then swaps the rows for ±∞
// placeholders — an unconstrained QP that agrees with anything. These fixtures
// have no velocity slack.

constexpr double kOracleEps = 1e-9;

rtc::tsid::QPData RelaySlack(const MpcSegmentCore& mpc, const rtc::tsid::QPData& src, bool per_node,
                             double rho) {
  const Eigen::Index n = mpc.Nv();
  const Eigen::Index N = mpc.NumNodes();
  const Eigen::Index nN = n * N;
  const Eigen::Index nu = mpc.Layout().nu;
  const Eigen::Index n_s = per_node ? N : nN;
  const Eigen::Index nz = nu + n_s;
  const Eigen::Index n_eq = src.A.rows();
  const Eigen::Index n_in = 4 * nN + n_s;
  // src is either layout; a velocity slack would be one column more.
  EXPECT_TRUE(src.H.rows() == nu + N || src.H.rows() == nu + nN)
      << "RelaySlack assumes no velocity slack";
  rtc::tsid::QPData qp;
  qp.Init(static_cast<int>(nz), static_cast<int>(n_eq), static_cast<int>(n_in));
  qp.n_vars = static_cast<int>(nz);
  qp.n_eq = static_cast<int>(n_eq);
  qp.n_ineq = static_cast<int>(n_in);
  qp.H.topLeftCorner(nu, nu) = src.H.topLeftCorner(nu, nu);
  qp.g.head(nu) = src.g.head(nu);
  qp.g.tail(n_s).setConstant(rho);
  qp.A.leftCols(nu) = src.A.leftCols(nu);
  qp.b = src.b;
  qp.C.topLeftCorner(4 * nN, nu) = src.C.topLeftCorner(4 * nN, nu);
  for (Eigen::Index k0 = 0; k0 < N; ++k0) {
    for (Eigen::Index j = 0; j < n; ++j) {
      const Eigen::Index col = nu + (per_node ? k0 : k0 * n + j);
      qp.C(2 * nN + k0 * n + j, col) = -1.0;
      qp.C(3 * nN + k0 * n + j, col) = 1.0;
    }
  }
  qp.C.bottomRightCorner(n_s, n_s).setIdentity();
  qp.l.head(4 * nN) = src.l.head(4 * nN);
  qp.u.head(4 * nN) = src.u.head(4 * nN);
  qp.l.tail(n_s).setZero();
  qp.u.tail(n_s).setConstant(kInf);
  return qp;
}

struct OracleSolution {
  bool converged{false};
  int status{-1};
  Eigen::VectorXd x;
  Eigen::VectorXd z_in;  // inequality multipliers (QPSolverWrapper sign convention)
  int iterations{0};
};

// A slack QP is feasible by construction, so ProxQP's approximate
// infeasibility test is off for it (eps_primal_inf 0, QPSolverConfig); a hard
// QP (slack pinned) may be infeasible and keeps the test.
OracleSolution SolveOracle(const rtc::tsid::QPData& qp, bool may_be_infeasible = false) {
  rtc::tsid::QPSolverConfig cfg;
  cfg.max_iter = 5000;
  cfg.max_iter_in = 1500;
  cfg.eps_abs = kOracleEps;
  cfg.eps_rel = 0.0;
  cfg.eps_primal_inf = may_be_infeasible ? 1e-4 : 0.0;
  cfg.update_preconditioner = true;
  cfg.dense_backend = proxsuite::proxqp::DenseBackend::PrimalDualLDLT;
  rtc::tsid::QPSolverWrapper solver;
  solver.Init(qp.n_vars, qp.n_eq, qp.n_ineq, cfg);
  const rtc::tsid::SolveResult& r = solver.Solve(qp);
  OracleSolution o;
  o.converged = r.converged;
  o.status = r.status;
  o.x = r.x_opt.head(qp.n_vars);
  o.z_in = solver.InequalityDual().head(qp.n_ineq);
  o.iterations = r.iterations;
  return o;
}

// Torque row r of [2nN, 4nN) at one of its bounds at x, to tol.
bool TorqueRowActive(const rtc::tsid::QPData& qp, const Eigen::VectorXd& x, Eigen::Index row,
                     double tol) {
  const double cx = qp.C.row(row).dot(x);
  return (std::isfinite(qp.l[row]) && std::abs(cx - qp.l[row]) <= tol) ||
         (std::isfinite(qp.u[row]) && std::abs(cx - qp.u[row]) <= tol);
}

int ActiveTorqueRows(const rtc::tsid::QPData& qp, const Eigen::VectorXd& x, Eigen::Index nN,
                     double tol) {
  int active = 0;
  for (Eigen::Index r = 2 * nN; r < 4 * nN; ++r) {
    active += TorqueRowActive(qp, x, r, tol) ? 1 : 0;
  }
  return active;
}

// τ^lin/τ_max (signed, nN) at x, read off the upper torque rows:
// C_ũ ũ − s ≤ η − c̃, so the ratio C_ũ ũ + c̃ is C_ũ ũ + η − u.
Eigen::VectorXd TorqueRatio(const rtc::tsid::QPData& qp, const Eigen::VectorXd& x, Eigen::Index nN,
                            Eigen::Index nu, double eta) {
  Eigen::VectorXd ratio = qp.C.block(2 * nN, 0, nN, nu) * x.head(nu);
  ratio.array() += eta - qp.u.segment(2 * nN, nN).array();
  return ratio;
}

bool SameQp(const rtc::tsid::QPData& a, const rtc::tsid::QPData& b) {
  return a.n_vars == b.n_vars && a.n_eq == b.n_eq && a.n_ineq == b.n_ineq && a.H == b.H &&
         a.g == b.g && a.A == b.A && a.b == b.b && a.C == b.C && a.l == b.l && a.u == b.u;
}

// The oracle's relay of the core's layout IS the core's QP, bit for bit: the
// slack block is −1 / +1 on the torque rows of its node, identity on the
// s ≥ 0 rows, ρ_τ in g, [0, ∞) bounds; nothing else touches it.
TEST(MpcSegmentCoreSlackLayout, TheOracleRelayReproducesTheCoreQp) {
  const TorqueFixture f = MakeTorqueFixture();
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*f.arm.model, f.arm.frame, f.params, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  ASSERT_TRUE(mpc.Solve(f.input, res)) << MpcSegmentCoreReasonName(res.reason);
  const Eigen::Index N = mpc.NumNodes();
  const Eigen::Index nN = static_cast<Eigen::Index>(mpc.Nv()) * N;
  EXPECT_EQ(mpc.MainQp().H.rows(), mpc.Layout().nu + N);
  EXPECT_EQ(mpc.MainQp().C.rows(), 4 * nN + N);
  EXPECT_TRUE(SameQp(RelaySlack(mpc, mpc.MainQp(), true, f.params.rho_tau), mpc.MainQp()));
  ASSERT_EQ(res.slack.size(), N);
  for (Eigen::Index k = 0; k < N; ++k) {
    EXPECT_EQ(res.slack[k], mpc.LastSolution()[mpc.Layout().nu + k]) << "node " << k + 1;
  }
}

// (b) The hard problem feasible and its torque row binding: both layouts give
// the same jerk — ρ_τ = 10 is exact for both here.
TEST(MpcSegmentCoreSlackLayout, PerNodeAgreesWithPerJointWhenTheSlackIsZero) {
  const TorqueFixture f = MakeTorqueFixture();
  MpcSegmentCoreParams p = f.params;
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*f.arm.model, f.arm.frame, p, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  ASSERT_TRUE(mpc.Solve(f.input, res)) << MpcSegmentCoreReasonName(res.reason);
  const Eigen::Index N = mpc.NumNodes();
  const Eigen::Index nN = static_cast<Eigen::Index>(mpc.Nv()) * N;
  const Eigen::Index nu = mpc.Layout().nu;
  const rtc::tsid::QPData joint = RelaySlack(mpc, mpc.MainQp(), false, p.rho_tau);
  const rtc::tsid::QPData node = RelaySlack(mpc, mpc.MainQp(), true, p.rho_tau);
  const OracleSolution oj = SolveOracle(joint);
  const OracleSolution on = SolveOracle(node);
  ASSERT_TRUE(oj.converged);
  ASSERT_TRUE(on.converged);
  // The premise: torque rows bind (else the agreement says nothing) and no
  // slack is needed.
  EXPECT_GT(ActiveTorqueRows(joint, oj.x, nN, 10.0 * kOracleEps), 0);
  EXPECT_GT(ActiveTorqueRows(node, on.x, nN, 10.0 * kOracleEps), 0);
  EXPECT_LE(oj.x.tail(nN).maxCoeff(), 10.0 * kOracleEps);
  EXPECT_LE(on.x.tail(N).maxCoeff(), 10.0 * kOracleEps);
  const double scale = oj.x.head(nu).cwiseAbs().maxCoeff();
  ASSERT_GT(scale, 0.0);
  EXPECT_LE((oj.x.head(nu) - on.x.head(nu)).cwiseAbs().maxCoeff(), 1e-6 * scale);
  // The core's own answer (eps_abs 1e-6) against the oracle — measured.
  const double core_err =
      (mpc.LastSolution().head(nu) - on.x.head(nu)).cwiseAbs().maxCoeff() / scale;
  RecordProperty("core_vs_oracle_rel_ppb", static_cast<int>(std::lround(1e9 * core_err)));
  std::printf("[slack layout] core vs oracle |dz|/|z| = %.3g (oracle it %d / %d)\n", core_err,
              oj.iterations, on.iterations);
  EXPECT_LE(core_err, 1e-4);  // measured 7.8e-6 (2026-10-18)
}

// (c) Past the limit — static torques the stop posture cannot hold on two
// joints — the node's slack is its worst excess, max_j(|τ_kj|/τ_max,j − η)⁺:
// an ℓ∞ penalty within the node.
TEST(MpcSegmentCoreSlackLayout, PerNodeSlackIsTheWorstExcessOfItsNode) {
  const TorqueFixture f = MakeTorqueFixture();
  const int n = f.arm.model->nv;
  MpcSegmentCoreLimits tight = f.limits;
  pinocchio::Data data(*f.arm.model);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n);
  const Eigen::VectorXd g = Rnea(*f.arm.model, data, f.arm.q_nominal, zero, zero);
  tight.tau_max[1] = 0.5 * std::abs(g[1]) / f.params.eta_tau;
  tight.tau_max[2] = 0.7 * std::abs(g[2]) / f.params.eta_tau;
  ASSERT_GT(std::abs(g[2]), 1.0) << "the second joint carries no static torque here";
  MpcSegmentCoreParams p = f.params;
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*f.arm.model, f.arm.frame, p, tight), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  ASSERT_TRUE(mpc.Solve(f.input, res)) << MpcSegmentCoreReasonName(res.reason);
  const Eigen::Index N = mpc.NumNodes();
  const Eigen::Index nN = static_cast<Eigen::Index>(n) * N;
  const Eigen::Index nu = mpc.Layout().nu;
  const rtc::tsid::QPData node = RelaySlack(mpc, mpc.MainQp(), true, p.rho_tau);
  const OracleSolution on = SolveOracle(node);
  ASSERT_TRUE(on.converged);
  const Eigen::VectorXd ratio = TorqueRatio(node, on.x, nN, nu, p.eta_tau);
  int two_over = 0;
  for (Eigen::Index k0 = 0; k0 < N; ++k0) {
    double worst = 0.0;
    int over = 0;
    for (Eigen::Index j = 0; j < n; ++j) {
      const double excess = std::abs(ratio[k0 * n + j]) - p.eta_tau;
      worst = std::max(worst, excess);
      over += excess > 1e-6 ? 1 : 0;
    }
    const double s_k = on.x[nu + k0];
    EXPECT_NEAR(s_k, worst, 1e-6) << "node " << k0 + 1;
    for (Eigen::Index j = 0; j < n; ++j) {
      EXPECT_LE(std::abs(ratio[k0 * n + j]), p.eta_tau + s_k + 1e-6)
          << "node " << k0 + 1 << " joint " << j;
    }
    two_over += over >= 2 ? 1 : 0;
  }
  EXPECT_GT(on.x.tail(N).maxCoeff(), 0.1) << "the premise: the slack is active";
  EXPECT_GT(two_over, 0) << "no node has two joints past η — the max is not exercised";
  // The core agrees with its oracle on the terminal (static-torque) node.
  EXPECT_NEAR(res.slack_terminal_max, on.x[nu + N - 1], 1e-4);
}

// (d) The exactness condition the law changes: per joint ρ_τ must exceed each
// torque row's multiplier, per node it must exceed the SUM over the node's
// rows. Built so that the answer is known: at the loose stop's optimum x*
// (no torque row active), m rows of one inner node get their bound at their
// value at x*, and the cost a pull −μ·c_r along each — x* stays optimal, with
// multiplier μ on each of those rows (KKT) and m·μ on their node. With ρ*
// between the two, the per-joint penalty stays exact and the per-node one
// does not. (Tightening τ_max instead makes this stop's hard problem
// infeasible: the joints that carry torque on one node carry the arm's
// weight, which the jerk cannot shed.)
TEST(MpcSegmentCoreSlackLayout, PerNodeNeedsTheSumOfItsNodesMultipliers) {
  const TorqueFixture f = MakeTorqueFixture();
  const int n = f.arm.model->nv;
  MpcSegmentCoreParams p = f.params;
  MpcSegmentCoreLimits lim = f.limits;
  lim.tau_max[0] *= 10.0;  // loose: no torque row active at the optimum
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*f.arm.model, f.arm.frame, p, lim), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  ASSERT_TRUE(mpc.Solve(f.input, res)) << MpcSegmentCoreReasonName(res.reason);
  const Eigen::Index N = mpc.NumNodes();
  const Eigen::Index nN = static_cast<Eigen::Index>(n) * N;
  const Eigen::Index nu = mpc.Layout().nu;
  rtc::tsid::QPData base = RelaySlack(mpc, mpc.MainQp(), false, 0.0);
  base.u.tail(nN).setZero();  // the hard problem: slack pinned at 0
  const OracleSolution o0 = SolveOracle(base, true);
  ASSERT_TRUE(o0.converged) << "status " << o0.status;
  ASSERT_EQ(ActiveTorqueRows(base, o0.x, nN, 10.0 * kOracleEps), 0) << "the loose stop binds";
  // The inner node (2..N−1) with the most torque; its three largest joints.
  const Eigen::VectorXd ratio = TorqueRatio(base, o0.x, nN, nu, p.eta_tau);
  Eigen::Index k1 = 1;
  double most = -1.0;
  for (Eigen::Index k0 = 1; k0 + 1 < N; ++k0) {
    const double t = ratio.segment(k0 * n, n).cwiseAbs().sum();
    if (t > most) {
      most = t;
      k1 = k0;
    }
  }
  std::vector<int> order(static_cast<std::size_t>(n));
  for (int j = 0; j < n; ++j) {
    order[static_cast<std::size_t>(j)] = j;
  }
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return std::abs(ratio[k1 * n + a]) > std::abs(ratio[k1 * n + b]);
  });
  constexpr int kBound = 3;
  constexpr double kMu = 1.0;
  for (int i = 0; i < kBound; ++i) {
    const Eigen::Index r = k1 * n + order[static_cast<std::size_t>(i)];
    const bool upper = ratio[r] >= 0.0;
    const Eigen::Index row = (upper ? 2 * nN : 3 * nN) + r;
    const double at = base.C.row(row).dot(o0.x);
    if (upper) {
      base.u[row] = at;
      base.g.head(nu) -= kMu * base.C.row(row).head(nu).transpose();
    } else {
      base.l[row] = at;
      base.g.head(nu) += kMu * base.C.row(row).head(nu).transpose();
    }
  }
  const OracleSolution oh = SolveOracle(base, true);
  ASSERT_TRUE(oh.converged) << "status " << oh.status;
  EXPECT_LE((oh.x.head(nu) - o0.x.head(nu)).cwiseAbs().maxCoeff(),
            1e-6 * o0.x.head(nu).cwiseAbs().maxCoeff())
      << "x* is no longer the optimum";
  std::vector<double> node_sum(static_cast<std::size_t>(N), 0.0);
  double lambda_max = 0.0;
  for (Eigen::Index r0 = 0; r0 < 2 * nN; ++r0) {
    const Eigen::Index row = 2 * nN + r0;
    if (!TorqueRowActive(base, oh.x, row, 10.0 * kOracleEps)) {
      continue;
    }
    const double lambda = std::abs(oh.z_in[row]);
    lambda_max = std::max(lambda_max, lambda);
    node_sum[static_cast<std::size_t>((r0 % nN) / n)] += lambda;
  }
  const auto it = std::max_element(node_sum.begin(), node_sum.end());
  EXPECT_EQ(std::distance(node_sum.begin(), it), k1);
  EXPECT_NEAR(lambda_max, kMu, 1e-6);
  EXPECT_NEAR(*it, kBound * kMu, 1e-6);
  const double rho_star = 0.5 * (lambda_max + *it);
  std::printf("[slack layout] node %ld: sum of multipliers %.6g, largest %.6g, rho* %.4g\n",
              static_cast<long>(k1 + 1), *it, lambda_max, rho_star);
  const OracleSolution oj = SolveOracle(RelaySlack(mpc, base, false, rho_star));
  const OracleSolution on = SolveOracle(RelaySlack(mpc, base, true, rho_star));
  ASSERT_TRUE(oj.converged) << "status " << oj.status;
  ASSERT_TRUE(on.converged) << "status " << on.status;
  RecordProperty("per_node_slack_at_rho_star_ppb",
                 static_cast<int>(std::lround(1e9 * on.x[nu + k1])));
  EXPECT_LE(oj.x.tail(nN).maxCoeff(), 10.0 * kOracleEps) << "per joint: exact at rho*";
  EXPECT_GE(on.x[nu + k1], 100.0 * kOracleEps) << "per node: not exact at rho*";
}

// ── 7. Re-evaluated torque inside the limit ──────────────────────────────────

TEST(MpcSegmentCore, ReevaluatedTorqueWithinLimits) {
  // Armature on (the model copy inside the core carries it; the oracle below
  // must too). One RTI step: the pre-solve path linearises exactly once.
  const double armature = 0.1;
  const TorqueFixture f = MakeTorqueFixture(armature);
  const int n = f.arm.model->nv;
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*f.arm.model, f.arm.frame, f.params, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  ASSERT_TRUE(mpc.Solve(f.input, res)) << MpcSegmentCoreReasonName(res.reason);
  ASSERT_TRUE(res.presolved);
  ASSERT_LE(res.slack_max, 10.0 * f.params.solver.eps_abs);

  pinocchio::Model m = *f.arm.model;
  m.armature = Eigen::VectorXd::Constant(n, armature);
  pinocchio::Data data(m);
  Eigen::VectorXd q(n), qd(n), qdd(n);
  double ratio_max = 0.0;
  const double t_end = f.params.n_nodes * f.params.dt;
  for (double t = 0.0; t <= t_end + 1e-12; t += 1e-3) {
    ASSERT_TRUE(
        rtc::catching::SampleJerkTrajectory(res.q, res.qd, res.qdd, f.params.dt, t, q, qd, qdd));
    const Eigen::VectorXd tau = Rnea(m, data, q, qd, qdd);
    ratio_max = std::max(ratio_max, (tau.cwiseAbs().cwiseQuotient(f.limits.tau_max)).maxCoeff());
  }
  EXPECT_LE(ratio_max, 1.0);
  RecordProperty("tau_ratio_max_permille", static_cast<int>(std::lround(1000.0 * ratio_max)));
  RecordProperty("tau_ratio_over_eta_permille",
                 static_cast<int>(std::lround(1000.0 * ratio_max / f.params.eta_tau)));
  std::printf("[reevaluated torque] max |tau|/tau_max = %.4f (eta' = %.2f, ratio/eta' = %.4f)\n",
              ratio_max, f.params.eta_tau, ratio_max / f.params.eta_tau);
}

// ── Torque seam: derivative, offset sign, armature ───────────────────────────

TEST(MpcSegmentCore, TorqueLinearizationMatchesFiniteDifference) {
  const ArmModel arm = RealArm7();
  pinocchio::Model m = *arm.model;
  const int n = m.nv;
  m.armature = Eigen::VectorXd::Constant(n, 0.15);
  pinocchio::Data data(m);
  pinocchio::Data data_fd(m);
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  const Eigen::VectorXd q =
      arm.q_nominal + 0.3 * Eigen::VectorXd::NullaryExpr(n, [&] { return uni(rng); });
  const Eigen::VectorXd v = Eigen::VectorXd::NullaryExpr(n, [&] { return uni(rng); });
  const Eigen::VectorXd a = 3.0 * Eigen::VectorXd::NullaryExpr(n, [&] { return uni(rng); });
  Eigen::VectorXd tau(n);
  Eigen::MatrixXd d(n, 3 * n);
  ASSERT_TRUE(rtc::catching::LinearizeTorqueAt(m, data, q, v, a, tau, d));
  EXPECT_LE((tau - Rnea(m, data_fd, q, v, a)).cwiseAbs().maxCoeff(), 1e-10);
  const double h = 1e-6;
  for (int i = 0; i < 3 * n; ++i) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(3 * n);
    x[i] = h;
    const Eigen::VectorXd tp = Rnea(m, data_fd, q + x.head(n), v + x.segment(n, n), a + x.tail(n));
    const Eigen::VectorXd tm = Rnea(m, data_fd, q - x.head(n), v - x.segment(n, n), a - x.tail(n));
    const Eigen::VectorXd col = (tp - tm) / (2.0 * h);
    EXPECT_LE((d.col(i) - col).cwiseAbs().maxCoeff(), 1e-4 * (1.0 + col.cwiseAbs().maxCoeff()))
        << "column " << i;
  }
  // M block is symmetric (pinocchio fills only the upper triangle).
  const Eigen::MatrixXd mm = d.rightCols(n);
  EXPECT_LE((mm - mm.transpose()).cwiseAbs().maxCoeff(), 1e-12);
  // Stale output must not leak in (the armature is ADDED by pinocchio).
  d.setConstant(123.0);
  ASSERT_TRUE(rtc::catching::LinearizeTorqueAt(m, data, q, v, a, tau, d));
  EXPECT_LE((d.rightCols(n) - mm).cwiseAbs().maxCoeff(), 1e-12);
  // Size mismatch is refused.
  Eigen::MatrixXd bad(n, 3 * n - 1);
  EXPECT_FALSE(rtc::catching::LinearizeTorqueAt(m, data, q, v, a, tau, bad));
}

TEST(MpcSegmentCore, ArmatureEntersTorqueRows) {
  const ArmModel arm = RealArm6();
  const int n = arm.model->nv;
  pinocchio::Model m0 = *arm.model;
  m0.armature.setZero();
  pinocchio::Model m1 = m0;
  m1.armature = Eigen::VectorXd::Constant(n, 0.2);
  pinocchio::Data d0(m0), d1(m1);
  const Eigen::VectorXd v = Eigen::VectorXd::Constant(n, 0.3);
  const Eigen::VectorXd a = Eigen::VectorXd::LinSpaced(n, -2.0, 2.0);
  Eigen::VectorXd t0(n), t1(n);
  Eigen::MatrixXd dd0(n, 3 * n), dd1(n, 3 * n);
  ASSERT_TRUE(rtc::catching::LinearizeTorqueAt(m0, d0, arm.q_nominal, v, a, t0, dd0));
  ASSERT_TRUE(rtc::catching::LinearizeTorqueAt(m1, d1, arm.q_nominal, v, a, t1, dd1));
  const Eigen::MatrixXd dm = dd1.rightCols(n) - dd0.rightCols(n);
  EXPECT_LE((dm - 0.2 * Eigen::MatrixXd::Identity(n, n)).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LE((dd1.leftCols(2 * n) - dd0.leftCols(2 * n)).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LE((t1 - t0 - 0.2 * a).cwiseAbs().maxCoeff(), 1e-12);
}

// The offset c_k = τ̄_k + D_k(Φ_k x_0 − x̄_k): with the right sign the solved
// τ^lin differs from the true torque at the solution by O(‖x − x̄‖²), so
// halving the reference perturbation quarters the error. A sign error in the
// offset makes it first order (ratio ≈ 2).
TEST(MpcSegmentCore, TorqueLinearizationErrorIsSecondOrder) {
  const ArmModel arm = RealArm6();
  const int n = arm.model->nv;
  MpcSegmentCoreParams p;
  p.delta_tr = kInf;
  MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model);
  lim.tau_max *= 10.0;  // rows present, not binding: the error is the model's alone
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult base;
  mpc.ResizeResult(base);
  MpcSegmentCoreInput in = RestInput(arm.q_nominal);
  in.qd0 << 1.0, -0.8, 0.9, 0.6, 0.5, 0.5;
  ASSERT_TRUE(mpc.Solve(in, base));

  pinocchio::Data data(*arm.model);
  const auto error_at = [&](double delta) {
    MpcSegmentCoreInput pin = in;
    UseAsReference(base, pin);
    for (int k = 1; k < p.n_nodes; ++k) {
      for (int j = 0; j < n; ++j) {
        const double s = std::sin(1.3 * k + 0.7 * j);
        pin.q_ref(j, k) += delta * s;
        pin.qd_ref(j, k) += 4.0 * delta * s;
        pin.qdd_ref(j, k) += 20.0 * delta * s;
      }
    }
    MpcSegmentCoreResult r;
    mpc.ResizeResult(r);
    EXPECT_TRUE(mpc.Solve(pin, r)) << MpcSegmentCoreReasonName(r.reason);
    double err = 0.0;
    for (int k = 1; k <= p.n_nodes; ++k) {
      const Eigen::VectorXd tau = Rnea(*arm.model, data, r.q.col(k), r.qd.col(k), r.qdd.col(k));
      err = std::max(
          err, (r.tau_ratio.col(k - 1) - tau.cwiseQuotient(lim.tau_max)).cwiseAbs().maxCoeff());
    }
    return err;
  };
  const double e1 = error_at(0.04);
  const double e2 = error_at(0.02);
  ASSERT_GT(e1, 1e-7) << "perturbation too small to measure";
  const double ratio = e1 / e2;
  EXPECT_GT(ratio, 3.0) << "e(δ)=" << e1 << " e(δ/2)=" << e2;
  EXPECT_LT(ratio, 5.5) << "e(δ)=" << e1 << " e(δ/2)=" << e2;
}

// ── w_⊥ (implemented, default off) ──────────────────────────────────────────

TEST(MpcSegmentCore, PerpendicularTermReducesOffLineError) {
  const ArmModel arm = RealArm6();
  const int n = arm.model->nv;
  pinocchio::Data data(*arm.model);
  MpcSegmentCoreInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.8, -0.6, 0.9, 0.4, 0.5, 0.3;
  Eigen::MatrixXd j6 = Eigen::MatrixXd::Zero(6, n);
  pinocchio::computeFrameJacobian(*arm.model, data, arm.q_nominal, arm.frame,
                                  pinocchio::LOCAL_WORLD_ALIGNED, j6);
  in.p_c = data.oMf[arm.frame].translation();
  const Eigen::Vector3d v_tcp = j6.topRows(3) * in.qd0;
  in.d_hat = v_tcp.normalized();

  const auto off_line = [&](double w) {
    MpcSegmentCoreParams p;
    p.w_perp = w;
    MpcSegmentCore mpc;
    EXPECT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)),
              MpcSegmentCoreReason::kNone);
    MpcSegmentCoreResult r;
    mpc.ResizeResult(r);
    EXPECT_TRUE(mpc.Solve(in, r)) << MpcSegmentCoreReasonName(r.reason);
    const Eigen::Matrix3d perp = Eigen::Matrix3d::Identity() - in.d_hat * in.d_hat.transpose();
    double sum = 0.0;
    for (int k = 1; k <= p.n_nodes; ++k) {
      pinocchio::framesForwardKinematics(*arm.model, data, r.q.col(k));
      sum += (perp * (data.oMf[arm.frame].translation() - in.p_c)).squaredNorm();
    }
    return sum;
  };
  const double e_off = off_line(0.0);
  const double e_on = off_line(1e4);
  RecordProperty("off_line_sq_sum_off_um2", static_cast<int>(std::lround(1e12 * e_off)));
  RecordProperty("off_line_sq_sum_on_um2", static_cast<int>(std::lround(1e12 * e_on)));
  EXPECT_LT(e_on, 0.5 * e_off) << "off=" << e_off << " on=" << e_on;
}

// ── Optimisation oracle: structured assembly == dense assembly ──────────────

TEST(MpcSegmentCore, StructuredAssemblyMatchesDense) {
  const TorqueFixture f = MakeTorqueFixture(0.1);
  MpcSegmentCoreParams p = f.params;
  p.w_perp = 10.0;
  MpcSegmentCoreInput in = f.input;
  pinocchio::Data data(*f.arm.model);
  pinocchio::framesForwardKinematics(*f.arm.model, data, f.arm.q_nominal);
  in.p_c = data.oMf[f.arm.frame].translation();
  in.d_hat = Eigen::Vector3d(1.0, 2.0, -0.5).normalized();

  MpcSegmentCore fast, dense;
  MpcSegmentCoreParams pd = p;
  pd.reference_assembly = true;
  ASSERT_EQ(fast.Init(*f.arm.model, f.arm.frame, p, f.limits), MpcSegmentCoreReason::kNone);
  ASSERT_EQ(dense.Init(*f.arm.model, f.arm.frame, pd, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult rf, rd;
  fast.ResizeResult(rf);
  dense.ResizeResult(rd);
  // Two cycles: pre-solve path, then a shifted warm cycle.
  for (int cycle = 0; cycle < 2; ++cycle) {
    MpcSegmentCoreInput ci = in;
    if (cycle == 1) {
      ShiftReference(rf, p.dt, 0.02, ci);
    }
    ASSERT_TRUE(fast.Solve(ci, rf)) << MpcSegmentCoreReasonName(rf.reason);
    ASSERT_TRUE(dense.Solve(ci, rd)) << MpcSegmentCoreReasonName(rd.reason);
    const rtc::tsid::QPData& a = fast.MainQp();
    const rtc::tsid::QPData& b = dense.MainQp();
    // Equal to rounding; infinite bounds must match exactly.
    const auto close = [](const Eigen::MatrixXd& x, const Eigen::MatrixXd& y) {
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
    };
    EXPECT_TRUE(close(a.H, b.H)) << "H cycle " << cycle;
    EXPECT_TRUE(close(a.g, b.g)) << "g cycle " << cycle;
    EXPECT_TRUE(close(a.A, b.A)) << "A cycle " << cycle;
    EXPECT_TRUE(close(a.b, b.b)) << "b cycle " << cycle;
    EXPECT_TRUE(close(a.C, b.C)) << "C cycle " << cycle;
    EXPECT_TRUE(close(a.l, b.l)) << "l cycle " << cycle;
    EXPECT_TRUE(close(a.u, b.u)) << "u cycle " << cycle;
    EXPECT_LE((rf.u - rd.u).cwiseAbs().maxCoeff(), 1e-6 * (1.0 + rd.u.cwiseAbs().maxCoeff()));
  }
}

// ── 5. Allocation gates ──────────────────────────────────────────────────────

TEST(MpcSegmentCore, TheAllocationGatesAreArmed) {
  // operator-new gate sees std containers.
  {
    rtc::testing::ScopedAllocGate gate;
    std::vector<int> v(16);
    v[3] = 1;
    EXPECT_GT(gate.count(), 0U);
  }
  // malloc gate sees an Eigen allocation that the operator-new gate cannot.
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    Eigen::VectorXd v(257);
    v.setZero();
    EXPECT_GT(malloc_gate.count(), 0U);
    EXPECT_EQ(new_gate.count(), 0U) << "operator new saw Eigen — the premise changed";
  }
  // malloc gate sees allocations made INSIDE a shared library (pinocchio's
  // explicitly instantiated Data constructor).
  {
    const ArmModel arm = RealArm7();
    rtc::testing::ScopedMallocGate gate;
    pinocchio::Data data(*arm.model);
    EXPECT_GT(gate.count(), 0U);
  }
}

struct AllocCounts {
  std::size_t op_new{0};
  std::size_t c_malloc{0};
};

AllocCounts GatedSolve(MpcSegmentCore& mpc, const MpcSegmentCoreInput& in,
                       MpcSegmentCoreResult& res, bool& ok) {
  AllocCounts c;
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    ok = mpc.Solve(in, res);
    c.op_new = new_gate.count();
    c.c_malloc = malloc_gate.count();
  }
  return c;
}

MpcSegmentCoreInput PerpInput(const ArmModel& arm, const MpcSegmentCoreInput& base) {
  MpcSegmentCoreInput in = base;
  pinocchio::Data data(*arm.model);
  pinocchio::framesForwardKinematics(*arm.model, data, arm.q_nominal);
  in.p_c = data.oMf[arm.frame].translation();
  in.d_hat = Eigen::Vector3d(0.3, -0.4, 0.2).normalized();
  return in;
}

// Decision B (user, 2026-09-30): the core's own path must allocate nothing;
// ProxQP's internal allocations (public update() copies its vector arguments,
// and solve() allocates a few times per call) are a KNOWN limitation shared
// with every QPSolverWrapper user, tracked as a follow-up issue. So:
//   • operator new: zero over the WHOLE Solve (ProxQP included);
//   • C malloc: zero over every path that stops before the QP — the
//     reference-outside-box rejection runs linearisation (RNEA derivatives +
//     frame Jacobian) and the complete condensing (torque rows, bounds,
//     gradient, w_⊥) and fails only at the conflict check;
//   • C malloc over a full Solve is recorded (property + stdout), not asserted,
//     so the follow-up has a baseline.
TEST(MpcSegmentCore, CoreAllocatesNothingOutsideTheQpSolver) {
  const ArmModel arm = RealArm7();
  MpcSegmentCoreParams p;
  p.w_perp = 10.0;  // the FK / frame-Jacobian path is measured too
  const MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model, 0.2);
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  MpcSegmentCoreInput in = PerpInput(arm, RestInput(arm.q_nominal));
  in.qd0 << 0.6, -0.4, 0.5, 0.7, -0.3, 0.4, 0.2;
  ASSERT_TRUE(mpc.Solve(in, res)) << MpcSegmentCoreReasonName(res.reason);
  MpcSegmentCoreInput warm = in;
  ShiftReference(res, p.dt, 0.02, warm);
  ASSERT_TRUE(mpc.Solve(warm, res)) << MpcSegmentCoreReasonName(res.reason);  // warm-up
  MpcSegmentCoreInput next = warm;
  ShiftReference(res, p.dt, 0.02, next);  // prepared outside the gate
  MpcSegmentCoreInput outside = next;     // reference leaves the box by > δ
  for (int k = 1; k <= p.n_nodes; ++k) {
    outside.q_ref(2, k) = lim.q_max[2] + 3.0 * p.delta_tr;
  }

  bool ok = false;
  const AllocCounts full = GatedSolve(mpc, next, res, ok);
  EXPECT_TRUE(ok) << MpcSegmentCoreReasonName(res.reason);
  EXPECT_EQ(full.op_new, 0U) << "operator new inside Solve";
  RecordProperty("warm_solve_qp_solver_mallocs", static_cast<int>(full.c_malloc));
  std::printf("[alloc] warm Solve: %zu C mallocs (ProxQP, known limitation)\n", full.c_malloc);

  const AllocCounts core = GatedSolve(mpc, outside, res, ok);
  EXPECT_FALSE(ok);
  ASSERT_EQ(res.reason, MpcSegmentCoreReason::kTrustRegionConflict);
  EXPECT_GT(res.linearize_us, 0.0) << "the rejection must come after linearisation";
  EXPECT_GT(res.condense_us, 0.0) << "the rejection must come after condensing";
  EXPECT_EQ(core.op_new, 0U);
  EXPECT_EQ(core.c_malloc, 0U) << "C-level allocation in linearisation / condensing";
}

TEST(MpcSegmentCore, PresolveAndRejectionPathsAllocateNothing) {
  const ArmModel arm = RealArm7();
  MpcSegmentCoreParams p;
  p.w_perp = 10.0;
  MpcSegmentCore mpc;
  const MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model, 0.2);
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  MpcSegmentCoreInput in = PerpInput(arm, RestInput(arm.q_nominal));
  in.qd0 << 0.6, -0.4, 0.5, 0.7, -0.3, 0.4, 0.2;
  ASSERT_TRUE(mpc.Solve(in, res));  // warm-up of the pre-solve path

  bool ok = false;
  AllocCounts c = GatedSolve(mpc, in, res, ok);
  EXPECT_TRUE(ok);
  EXPECT_EQ(c.op_new, 0U) << "pre-solve path";
  RecordProperty("presolve_path_qp_solver_mallocs", static_cast<int>(c.c_malloc));

  MpcSegmentCoreInput nan_in = in;
  nan_in.qd0[2] = kNan;
  c = GatedSolve(mpc, nan_in, res, ok);
  EXPECT_FALSE(ok);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "non-finite rejection";

  MpcSegmentCoreInput drift = in;
  UseAsReference(res, drift);
  drift.q0[0] += 2.0 * p.delta_tr;
  c = GatedSolve(mpc, drift, res, ok);
  EXPECT_FALSE(ok);
  EXPECT_EQ(res.reason, MpcSegmentCoreReason::kTrustRegionConflict);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "trust-region rejection";

  // QP failure: at the upper edge of the box, moving outward at the velocity
  // limit — node 1 cannot stay inside.
  MpcSegmentCoreInput edge = RestInput(arm.q_nominal);
  edge.q0[3] = lim.q_max[3] - p.m_q - 1e-9;
  edge.qd0[3] = p.eta_v * lim.qd_max[3];
  edge.p_c = in.p_c;
  edge.d_hat = in.d_hat;
  c = GatedSolve(mpc, edge, res, ok);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(res.reason == MpcSegmentCoreReason::kPresolveFailed ||
              res.reason == MpcSegmentCoreReason::kQpFailed)
      << MpcSegmentCoreReasonName(res.reason);
  EXPECT_EQ(c.op_new, 0U) << "QP failure path";
  RecordProperty("qp_failure_path_qp_solver_mallocs", static_cast<int>(c.c_malloc));
}

// ── 6. Fail-closed ───────────────────────────────────────────────────────────

struct Snapshot {
  Eigen::MatrixXd q, qd, qdd, u;
  Eigen::VectorXd slack;  // one per node (#857)
  Eigen::MatrixXd tau_ratio;

  explicit Snapshot(const MpcSegmentCoreResult& r)
      : q(r.q), qd(r.qd), qdd(r.qdd), u(r.u), slack(r.slack), tau_ratio(r.tau_ratio) {}

  [[nodiscard]] bool Same(const MpcSegmentCoreResult& r) const {
    return q == r.q && qd == r.qd && qdd == r.qdd && u == r.u && slack == r.slack &&
           tau_ratio == r.tau_ratio;
  }
};

class MpcSegmentCoreFailClosed : public ::testing::Test {
 protected:
  void SetUp() override {
    arm_ = Synthetic6R();
    params_.w_perp = 1.0;
    limits_ = LimitsFromModel(*arm_.model);
    ASSERT_EQ(mpc_.Init(*arm_.model, arm_.frame, params_, limits_), MpcSegmentCoreReason::kNone);
    mpc_.ResizeResult(res_);
    good_ = PerpInput(arm_, RestInput(arm_.q_nominal));
    good_.qd0 << 0.3, -0.2, 0.25, 0.1, -0.2, 0.3;
    ASSERT_TRUE(mpc_.Solve(good_, res_));
  }

  void ExpectRejected(const MpcSegmentCoreInput& in, MpcSegmentCoreReason why, const char* what) {
    const Snapshot before(res_);
    EXPECT_FALSE(mpc_.Solve(in, res_)) << what;
    EXPECT_EQ(res_.reason, why) << what << ": got " << MpcSegmentCoreReasonName(res_.reason);
    EXPECT_FALSE(res_.valid) << what;
    EXPECT_TRUE(before.Same(res_)) << what << ": outputs changed on failure";
  }

  ArmModel arm_;
  MpcSegmentCoreParams params_;
  MpcSegmentCoreLimits limits_;
  MpcSegmentCore mpc_;
  MpcSegmentCoreResult res_;
  MpcSegmentCoreInput good_;
};

TEST_F(MpcSegmentCoreFailClosed, RejectsNonFiniteInput) {
  for (const double bad : {kNan, kInf, -kInf}) {
    MpcSegmentCoreInput in = good_;
    in.q0[1] = bad;
    ExpectRejected(in, MpcSegmentCoreReason::kNonFinite, "q0");
    in = good_;
    in.qd0[4] = bad;
    ExpectRejected(in, MpcSegmentCoreReason::kNonFinite, "qd0");
    in = good_;
    in.qdd0[0] = bad;
    ExpectRejected(in, MpcSegmentCoreReason::kNonFinite, "qdd0");
    in = good_;
    UseAsReference(res_, in);
    in.q_ref(2, 5) = bad;
    ExpectRejected(in, MpcSegmentCoreReason::kNonFinite, "q_ref");
    in = good_;
    UseAsReference(res_, in);
    in.qdd_ref(0, 3) = bad;
    ExpectRejected(in, MpcSegmentCoreReason::kNonFinite, "qdd_ref");
    in = good_;
    in.p_c[1] = bad;
    ExpectRejected(in, MpcSegmentCoreReason::kNonFinite, "p_c");
    in = good_;
    in.d_hat[2] = bad;
    ExpectRejected(in, MpcSegmentCoreReason::kNonFinite, "d_hat");
  }
  MpcSegmentCoreInput in = good_;
  in.d_hat = Eigen::Vector3d(1.0, 1.0, 0.0);
  ExpectRejected(in, MpcSegmentCoreReason::kDirectionNotUnit, "d_hat not unit");
}

TEST_F(MpcSegmentCoreFailClosed, RejectsDimensionMismatch) {
  MpcSegmentCoreInput in = good_;
  in.q0.resize(5);
  in.q0.setZero();
  ExpectRejected(in, MpcSegmentCoreReason::kDimMismatch, "q0 size");
  in = good_;
  UseAsReference(res_, in);
  in.q_ref.conservativeResize(Eigen::NoChange, params_.n_nodes);
  ExpectRejected(in, MpcSegmentCoreReason::kDimMismatch, "reference columns");
  MpcSegmentCoreResult wrong;  // never resized
  EXPECT_FALSE(mpc_.Solve(good_, wrong));
  EXPECT_EQ(wrong.reason, MpcSegmentCoreReason::kDimMismatch);
  EXPECT_EQ(wrong.q.size(), 0) << "Solve must not resize";
}

TEST_F(MpcSegmentCoreFailClosed, RejectsInitialStateOutsideBox) {
  MpcSegmentCoreInput in = good_;
  in.q0[2] = limits_.q_min[2] + 0.5 * params_.m_q;
  ExpectRejected(in, MpcSegmentCoreReason::kInitialStateOutsideBox, "q0 in margin");
  in = good_;
  in.qd0[1] = -1.01 * params_.eta_v * limits_.qd_max[1];
  ExpectRejected(in, MpcSegmentCoreReason::kInitialStateOutsideBox, "qd0 over eta_v");
}

TEST_F(MpcSegmentCoreFailClosed, RejectsReferenceNotAtRest) {
  MpcSegmentCoreInput in = good_;
  UseAsReference(res_, in);
  in.qd_ref(3, params_.n_nodes) = 0.1;
  ExpectRejected(in, MpcSegmentCoreReason::kReferenceNotAtRest, "terminal velocity");
  in = good_;
  UseAsReference(res_, in);
  in.qdd_ref(0, params_.n_nodes) = -0.1;
  ExpectRejected(in, MpcSegmentCoreReason::kReferenceNotAtRest, "terminal acceleration");
}

TEST_F(MpcSegmentCoreFailClosed, TrustRegionConflictFailsClosed) {
  MpcSegmentCoreInput in = good_;
  UseAsReference(res_, in);
  in.q0[5] += 1.5 * params_.delta_tr;
  ExpectRejected(in, MpcSegmentCoreReason::kTrustRegionConflict, "x0 drifted");
  // A reference outside the box by more than δ: lower > upper on its row.
  in = good_;
  UseAsReference(res_, in);
  for (int k = 1; k <= params_.n_nodes; ++k) {
    in.q_ref(2, k) = limits_.q_max[2] + 3.0 * params_.delta_tr;
  }
  ExpectRejected(in, MpcSegmentCoreReason::kTrustRegionConflict, "reference outside box");
}

// ── 6b. Deadline and warm cap (#887) ─────────────────────────────────────────

// A clock that returns its time, then advances it by a step, and counts its
// reads — which QP a deadline cuts is then fixed by the step, not by how long
// a solve takes.
std::int64_t g_clock_now = 0;
std::int64_t g_clock_step = 0;
int g_clock_reads = 0;

std::int64_t StepClock() noexcept {
  ++g_clock_reads;
  const std::int64_t t = g_clock_now;
  g_clock_now += g_clock_step;
  return t;
}

void StartStepClock(std::int64_t now, std::int64_t step) {
  g_clock_now = now;
  g_clock_step = step;
  g_clock_reads = 0;
}

// The cut site's fields, and the fields a cut must leave alone.
void ExpectCut(const MpcSegmentCoreResult& r, rtc::catching::MpcSegmentCutSite site,
               const char* what) {
  EXPECT_FALSE(r.valid) << what;
  EXPECT_EQ(r.reason, MpcSegmentCoreReason::kDeadline)
      << what << ": got " << MpcSegmentCoreReasonName(r.reason);
  EXPECT_EQ(r.cut_site, site) << what << ": got "
                              << rtc::catching::MpcSegmentCutSiteName(r.cut_site);
  EXPECT_FALSE(r.cold_retried) << what;
}

class MpcSegmentCoreDeadline : public ::testing::Test {
 protected:
  void SetUp() override {
    arm_ = RealArm7();
    limits_ = LimitsFromModel(*arm_.model, 0.2);
    ASSERT_EQ(mpc_.Init(*arm_.model, arm_.frame, params_, limits_), MpcSegmentCoreReason::kNone);
    mpc_.ResizeResult(res_);
    moving_ = RestInput(arm_.q_nominal);
    moving_.qd0 << 0.6, -0.4, 0.5, 0.7, -0.3, 0.4, 0.2;
    ASSERT_TRUE(mpc_.Solve(moving_, res_));
    referenced_ = moving_;
    UseAsReference(res_, referenced_);
    // Really infeasible (the approach suite's case): at the upper edge of the
    // box, moving outward at the velocity limit — node 1 cannot stay inside.
    // With a reference, so a warm solver runs the main QP twice.
    edge_ = RestInput(arm_.q_nominal);
    edge_.q0[3] = limits_.q_max[3] - params_.m_q - 1e-9;
    edge_.qd0[3] = params_.eta_v * limits_.qd_max[3];
    const int n = static_cast<int>(arm_.model->nv);
    edge_.q_ref = edge_.q0.replicate(1, mpc_.NumNodes() + 1);
    edge_.qd_ref.setZero(n, mpc_.NumNodes() + 1);
    edge_.qdd_ref.setZero(n, mpc_.NumNodes() + 1);
    edge_.reference_valid = true;
    mpc_.SetClock(&StepClock);
  }

  // A converged solve with a reference: the solver holds its iterates.
  void Warm(MpcSegmentCore& mpc) {
    MpcSegmentCoreInput in = referenced_;
    in.deadline_ns = 0;
    ASSERT_TRUE(mpc.Solve(in, res_)) << MpcSegmentCoreReasonName(res_.reason);
  }

  ArmModel arm_;
  MpcSegmentCoreParams params_;
  MpcSegmentCoreLimits limits_;
  MpcSegmentCore mpc_;
  MpcSegmentCoreResult res_;
  MpcSegmentCoreInput moving_, referenced_, edge_;
};

TEST_F(MpcSegmentCoreDeadline, WithoutADeadlineOrAClockTheClockIsNotRead) {
  StartStepClock(1000, 0);
  ASSERT_TRUE(mpc_.Solve(moving_, res_));  // pre-solve + main, deadline 0
  EXPECT_EQ(g_clock_reads, 0);
  MpcSegmentCoreInput past = referenced_;
  past.deadline_ns = -5;  // ≤ 0 is no deadline
  ASSERT_TRUE(mpc_.Solve(past, res_));
  EXPECT_EQ(g_clock_reads, 0);
  EXPECT_EQ(res_.cut_site, rtc::catching::MpcSegmentCutSite::kNone);
  // A core with no clock ignores a deadline long past.
  MpcSegmentCore no_clock;
  ASSERT_EQ(no_clock.Init(*arm_.model, arm_.frame, params_, limits_), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult r;
  no_clock.ResizeResult(r);
  past.deadline_ns = 1;
  EXPECT_TRUE(no_clock.Solve(past, r)) << MpcSegmentCoreReasonName(r.reason);
  EXPECT_EQ(g_clock_reads, 0);
}

TEST_F(MpcSegmentCoreDeadline, PastItThePresolveIsNotStarted) {
  const Snapshot before(res_);
  StartStepClock(1000, 0);
  MpcSegmentCoreInput in = moving_;  // no reference: the pre-solve runs first
  in.deadline_ns = 500;
  EXPECT_FALSE(mpc_.Solve(in, res_));
  ExpectCut(res_, rtc::catching::MpcSegmentCutSite::kPresolve, "pre-solve");
  EXPECT_FALSE(res_.presolved);
  EXPECT_EQ(res_.presolve_iterations, 0);
  EXPECT_EQ(res_.iterations, 0);
  EXPECT_EQ(res_.qp_status, -1);
  EXPECT_EQ(res_.solve_us, 0.0);
  EXPECT_EQ(g_clock_reads, 1);
  EXPECT_TRUE(before.Same(res_)) << "outputs changed on a cut";
}

TEST_F(MpcSegmentCoreDeadline, PastItTheMainQpIsNotStarted) {
  const Snapshot before(res_);
  StartStepClock(1000, 0);
  MpcSegmentCoreInput in = referenced_;
  in.deadline_ns = 500;
  EXPECT_FALSE(mpc_.Solve(in, res_));
  ExpectCut(res_, rtc::catching::MpcSegmentCutSite::kMain, "main");
  EXPECT_EQ(res_.iterations, 0);
  EXPECT_EQ(res_.outer_iterations, 0);
  EXPECT_EQ(res_.qp_status, -1);
  EXPECT_EQ(res_.solve_us, 0.0);
  EXPECT_EQ(g_clock_reads, 1);
  EXPECT_TRUE(before.Same(res_)) << "outputs changed on a cut";
  // The pre-solve ran inside the deadline, the main QP did not: its fields
  // are the pre-solve's, the cut is the main QP's.
  in = moving_;
  StartStepClock(0, 10);
  in.deadline_ns = 5;  // read 1 (pre-solve) at 0, read 2 (main) at 10
  EXPECT_FALSE(mpc_.Solve(in, res_));
  ExpectCut(res_, rtc::catching::MpcSegmentCutSite::kMain, "main after the pre-solve");
  EXPECT_TRUE(res_.presolved);
  EXPECT_GT(res_.presolve_iterations, 0);
  EXPECT_EQ(res_.iterations, 0);
  EXPECT_EQ(g_clock_reads, 2);
  EXPECT_TRUE(before.Same(res_)) << "outputs changed on a cut";
}

TEST_F(MpcSegmentCoreDeadline, PastItAfterAFailedWarmRunTheColdRunIsNotStarted) {
  // The control: inside the deadline the warm run fails and the cold run
  // follows — two reads, both runs.
  Warm(mpc_);
  StartStepClock(0, 10);
  MpcSegmentCoreInput in = edge_;
  in.deadline_ns = 1000;
  EXPECT_FALSE(mpc_.Solve(in, res_));
  EXPECT_EQ(res_.reason, MpcSegmentCoreReason::kQpFailed) << MpcSegmentCoreReasonName(res_.reason);
  EXPECT_TRUE(res_.cold_retried);
  EXPECT_EQ(g_clock_reads, 2);

  // Read 1 (main) at 0 is inside, read 2 (cold run) at 10 is past.
  Warm(mpc_);
  const Snapshot before(res_);
  StartStepClock(0, 10);
  in.deadline_ns = 5;
  EXPECT_FALSE(mpc_.Solve(in, res_));
  ExpectCut(res_, rtc::catching::MpcSegmentCutSite::kColdRetry, "cold run");
  EXPECT_EQ(g_clock_reads, 2);
  // The warm run's record.
  EXPECT_NE(res_.qp_status, 0);
  EXPECT_NE(res_.qp_status, -1);
  EXPECT_GT(res_.iterations, 0);
  EXPECT_GE(res_.outer_iterations, 1);
  EXPECT_LE(res_.outer_iterations, params_.solver.max_iter);
  EXPECT_GT(res_.solve_us, 0.0);
  EXPECT_TRUE(before.Same(res_)) << "outputs changed on a cut";
  // The failed warm run reset the solver: the same input now runs once, cold.
  StartStepClock(0, 0);
  in.deadline_ns = 0;
  EXPECT_FALSE(mpc_.Solve(in, res_));
  EXPECT_EQ(res_.reason, MpcSegmentCoreReason::kQpFailed);
  EXPECT_FALSE(res_.cold_retried);
}

TEST_F(MpcSegmentCoreDeadline, AFarDeadlineChangesNoResult) {
  MpcSegmentCore plain;
  ASSERT_EQ(plain.Init(*arm_.model, arm_.frame, params_, limits_), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult r_plain;
  plain.ResizeResult(r_plain);
  ASSERT_TRUE(plain.Solve(moving_, r_plain));  // the same history as mpc_
  StartStepClock(0, 1);
  for (const MpcSegmentCoreInput* base : {&referenced_, &edge_, &moving_, &referenced_}) {
    MpcSegmentCoreInput far = *base;
    far.deadline_ns = std::numeric_limits<std::int64_t>::max();
    const bool ok = mpc_.Solve(far, res_);
    EXPECT_EQ(plain.Solve(*base, r_plain), ok);
    EXPECT_EQ(res_.reason, r_plain.reason);
    EXPECT_EQ(res_.iterations, r_plain.iterations);
    EXPECT_EQ(res_.outer_iterations, r_plain.outer_iterations);
    EXPECT_EQ(res_.presolve_iterations, r_plain.presolve_iterations);
    EXPECT_EQ(res_.qp_status, r_plain.qp_status);
    EXPECT_EQ(res_.cold_retried, r_plain.cold_retried);
    EXPECT_TRUE(Snapshot(r_plain).Same(res_)) << "bit for bit";
  }
  EXPECT_GT(g_clock_reads, 0);
}

TEST_F(MpcSegmentCoreDeadline, ACutAllocatesNothing) {
  StartStepClock(1000, 0);
  for (MpcSegmentCoreInput in : {moving_, referenced_}) {
    in.deadline_ns = 500;
    bool ok = true;
    const AllocCounts c = GatedSolve(mpc_, in, res_, ok);
    EXPECT_FALSE(ok);
    EXPECT_EQ(res_.reason, MpcSegmentCoreReason::kDeadline);
    EXPECT_EQ(c.op_new + c.c_malloc, 0U) << rtc::catching::MpcSegmentCutSiteName(res_.cut_site);
  }
}

// solver_max_iter_warm caps the outer iterations of a main QP that starts
// warm; the cold run after it keeps solver.max_iter.
TEST_F(MpcSegmentCoreDeadline, TheWarmCapEndsAFailingWarmRunAndNotTheColdOne) {
  constexpr int kCap = 5;
  MpcSegmentCoreParams capped_params = params_;
  capped_params.solver_max_iter_warm = kCap;
  MpcSegmentCore capped;
  ASSERT_EQ(capped.Init(*arm_.model, arm_.frame, capped_params, limits_),
            MpcSegmentCoreReason::kNone);
  capped.SetClock(&StepClock);
  ASSERT_TRUE(capped.Solve(moving_, res_));
  // The warm run, read where the cold run is cut: capped, and the control
  // (no cap) past it.
  for (MpcSegmentCore* core : {&capped, &mpc_}) {
    Warm(*core);
    StartStepClock(0, 10);
    MpcSegmentCoreInput in = edge_;
    in.deadline_ns = 5;
    EXPECT_FALSE(core->Solve(in, res_));
    ExpectCut(res_, rtc::catching::MpcSegmentCutSite::kColdRetry, "warm run");
    if (core == &capped) {
      EXPECT_LE(res_.outer_iterations, kCap);
    } else {
      EXPECT_GT(res_.outer_iterations, kCap) << "the control does not reach past the cap";
    }
  }
  // Uncut, the cold run after the capped warm one runs under max_iter.
  Warm(capped);
  StartStepClock(0, 0);
  EXPECT_FALSE(capped.Solve(edge_, res_));
  EXPECT_EQ(res_.reason, MpcSegmentCoreReason::kQpFailed);
  EXPECT_TRUE(res_.cold_retried);
  EXPECT_GT(res_.outer_iterations, kCap);
  // And it is a cap on max_iter, not a setting left behind: a solve from zero
  // on the capped core matches the uncapped one.
  MpcSegmentCoreInput cold = referenced_;
  cold.cold_start = true;
  MpcSegmentCoreResult r_plain;
  mpc_.ResizeResult(r_plain);
  ASSERT_TRUE(capped.Solve(cold, res_));
  ASSERT_TRUE(mpc_.Solve(cold, r_plain));
  EXPECT_EQ(res_.iterations, r_plain.iterations);
  EXPECT_TRUE(Snapshot(r_plain).Same(res_));
  // A main QP warm-started by the solve's own pre-solve is not capped: a
  // cap of 1 changes nothing there, and the control shows it would bite.
  capped_params.solver_max_iter_warm = 1;
  MpcSegmentCore one, none;
  ASSERT_EQ(one.Init(*arm_.model, arm_.frame, capped_params, limits_), MpcSegmentCoreReason::kNone);
  ASSERT_EQ(none.Init(*arm_.model, arm_.frame, params_, limits_), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult r_one, r_none;
  one.ResizeResult(r_one);
  none.ResizeResult(r_none);
  ASSERT_TRUE(one.Solve(moving_, r_one));
  ASSERT_TRUE(none.Solve(moving_, r_none));
  ASSERT_TRUE(r_none.presolved);
  EXPECT_GT(r_none.outer_iterations, 1) << "the control does not reach past the cap";
  EXPECT_EQ(r_one.outer_iterations, r_none.outer_iterations);
  EXPECT_EQ(r_one.iterations, r_none.iterations);
  EXPECT_FALSE(r_one.cold_retried);
  EXPECT_TRUE(Snapshot(r_none).Same(r_one));
  // Negative is refused.
  capped_params.solver_max_iter_warm = -1;
  MpcSegmentCore bad;
  EXPECT_EQ(bad.Init(*arm_.model, arm_.frame, capped_params, limits_),
            MpcSegmentCoreReason::kParamsInvalid);
}

TEST(MpcSegmentCore, RejectsBeforeInit) {
  MpcSegmentCore mpc;
  MpcSegmentCoreResult res;
  res.q.setZero(6, 13);
  EXPECT_FALSE(mpc.IsInitialized());
  EXPECT_FALSE(mpc.Solve(RestInput(Eigen::VectorXd::Zero(6)), res));
  EXPECT_EQ(res.reason, MpcSegmentCoreReason::kNotInitialized);
}

TEST(MpcSegmentCore, InitValidatesParamsAndLimits) {
  const ArmModel arm = Synthetic6R();
  const MpcSegmentCoreLimits good = LimitsFromModel(*arm.model);
  const auto init = [&](const MpcSegmentCoreParams& p, const MpcSegmentCoreLimits& l) {
    MpcSegmentCore mpc;
    const MpcSegmentCoreReason r = mpc.Init(*arm.model, arm.frame, p, l);
    EXPECT_EQ(mpc.IsInitialized(), r == MpcSegmentCoreReason::kNone);
    return r;
  };
  MpcSegmentCoreParams p;
  EXPECT_EQ(init(p, good), MpcSegmentCoreReason::kNone);
  p.block_sizes[5] = 4;  // Σ = 13 ≠ N
  EXPECT_EQ(init(p, good), MpcSegmentCoreReason::kParamsInvalid);
  p = MpcSegmentCoreParams{};
  p.n_nodes = rtc::catching::kMaxSegmentNodes + 1;
  EXPECT_EQ(init(p, good), MpcSegmentCoreReason::kParamsInvalid);
  for (const double bad : {0.0, -1.0, kNan}) {
    p = MpcSegmentCoreParams{};
    p.dt = bad;
    EXPECT_EQ(init(p, good), MpcSegmentCoreReason::kParamsInvalid) << "dt " << bad;
    p = MpcSegmentCoreParams{};
    p.eta_tau = bad;
    EXPECT_EQ(init(p, good), MpcSegmentCoreReason::kParamsInvalid) << "eta_tau " << bad;
    p = MpcSegmentCoreParams{};
    p.delta_tr = bad;
    EXPECT_EQ(init(p, good), MpcSegmentCoreReason::kParamsInvalid) << "delta_tr " << bad;
  }
  p = MpcSegmentCoreParams{};
  p.jerk_weight = Eigen::VectorXd::Ones(5);
  EXPECT_EQ(init(p, good), MpcSegmentCoreReason::kParamsInvalid);
  // A rest tolerance at or below eps_abs would reject every shifted reference.
  p = MpcSegmentCoreParams{};
  p.reference_rest_tol = p.solver.eps_abs;
  EXPECT_EQ(init(p, good), MpcSegmentCoreReason::kParamsInvalid);
  p.reference_rest_tol = 2.0 * p.solver.eps_abs;
  EXPECT_EQ(init(p, good), MpcSegmentCoreReason::kNone);
  // The infeasibility threshold (#880): 0 is valid (an exact certificate only),
  // a negative or non-finite one is not.
  for (const double bad : {-1e-6, kNan, std::numeric_limits<double>::infinity()}) {
    p = MpcSegmentCoreParams{};
    p.solver.eps_primal_inf = bad;
    EXPECT_EQ(init(p, good), MpcSegmentCoreReason::kParamsInvalid) << "eps_primal_inf " << bad;
  }
  p = MpcSegmentCoreParams{};
  p.solver.eps_primal_inf = 0.0;
  EXPECT_EQ(init(p, good), MpcSegmentCoreReason::kNone);

  p = MpcSegmentCoreParams{};
  MpcSegmentCoreLimits l = good;
  l.tau_max[2] = kNan;
  EXPECT_EQ(init(p, l), MpcSegmentCoreReason::kLimitsInvalid);
  l = good;
  l.q_min[1] = l.q_max[1] + 0.1;
  EXPECT_EQ(init(p, l), MpcSegmentCoreReason::kLimitsInvalid);
  l = good;
  l.qd_max[0] = 0.0;
  EXPECT_EQ(init(p, l), MpcSegmentCoreReason::kLimitsInvalid);
  l = good;
  l.armature.resize(3);
  EXPECT_EQ(init(p, l), MpcSegmentCoreReason::kLimitsInvalid);

  MpcSegmentCore mpc;
  EXPECT_EQ(mpc.Init(*arm.model, static_cast<pinocchio::FrameIndex>(arm.model->nframes), p, good),
            MpcSegmentCoreReason::kFrameUnknown);
}

TEST(MpcSegmentCore, LockedJointIsAllowed) {
  // q_min == q_max is a legitimate locked joint (NUM-7 availability case):
  // Init accepts it and the joint stays put.
  const ArmModel arm = Synthetic6R();
  MpcSegmentCoreLimits l = LimitsFromModel(*arm.model);
  l.q_min[4] = arm.q_nominal[4];
  l.q_max[4] = arm.q_nominal[4];
  MpcSegmentCoreParams p;
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, l), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  MpcSegmentCoreInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.3, -0.2, 0.25, 0.1, 0.0, 0.3;
  ASSERT_TRUE(mpc.Solve(in, res)) << MpcSegmentCoreReasonName(res.reason);
  EXPECT_LE((res.q.row(4).array() - arm.q_nominal[4]).abs().maxCoeff(), 1e-5);
}

// ── 8. Timing distribution (informational; F03 owns the budget) ─────────────

struct Samples {
  std::vector<double> presolve, linearize, condense, solve, total, iters, pre_iters;
  int failures{0};
  std::string failure_log;

  void Fail(const MpcSegmentCoreResult& r) {
    ++failures;
    if (failures <= 5) {
      failure_log += std::string(" ") + MpcSegmentCoreReasonName(r.reason) + "/status" +
                     std::to_string(r.qp_status) + "/it" + std::to_string(r.iterations);
    }
  }

  void Add(const MpcSegmentCoreResult& r) {
    presolve.push_back(r.presolve_us);
    linearize.push_back(r.linearize_us);
    condense.push_back(r.condense_us);
    solve.push_back(r.solve_us);
    total.push_back(r.presolve_us + r.linearize_us + r.condense_us + r.solve_us);
    iters.push_back(r.iterations);
    pre_iters.push_back(r.presolve_iterations);
  }
};

void Report(const std::string& key, const Samples& s) {
  const auto rec = [&](const std::string& name, const std::vector<double>& v) {
    RecordMicros(key + "." + name + ".p50", Percentile(v, 0.50));
    RecordMicros(key + "." + name + ".p99", Percentile(v, 0.99));
    RecordMicros(key + "." + name + ".max", Percentile(v, 1.0));
  };
  rec("presolve_us", s.presolve);
  rec("linearize_us", s.linearize);
  rec("condense_us", s.condense);
  rec("solve_us", s.solve);
  rec("total_us", s.total);
  ::testing::Test::RecordProperty(key + ".iterations.p50",
                                  static_cast<int>(Percentile(s.iters, 0.5)));
  ::testing::Test::RecordProperty(key + ".iterations.p99",
                                  static_cast<int>(Percentile(s.iters, 0.99)));
  ::testing::Test::RecordProperty(key + ".failures", s.failures);
  if (s.failures > 0) {
    std::printf("[timing] %s failures:%s\n", key.c_str(), s.failure_log.c_str());
  }
  std::printf(
      "[timing] %-34s n=%3zu fail=%2d | pre p50 %6.0f p99 %6.0f | lin p50 %5.0f p99 %5.0f | "
      "cond p50 %5.0f p99 %5.0f | solve p50 %6.0f p99 %6.0f max %6.0f | total p99 %6.0f | "
      "iter p50 %3.0f p99 %3.0f (pre %3.0f)\n",
      key.c_str(), s.total.size(), s.failures, Percentile(s.presolve, 0.5),
      Percentile(s.presolve, 0.99), Percentile(s.linearize, 0.5), Percentile(s.linearize, 0.99),
      Percentile(s.condense, 0.5), Percentile(s.condense, 0.99), Percentile(s.solve, 0.5),
      Percentile(s.solve, 0.99), Percentile(s.solve, 1.0), Percentile(s.total, 0.99),
      Percentile(s.iters, 0.5), Percentile(s.iters, 0.99), Percentile(s.pre_iters, 0.5));
}

// Cold = the first cycle (reference_valid = false → pre-solve + full solve).
// Warm = the next cycle, reference shifted by one planner period, x_0 on it.
// The distribution is only meaningful in an optimised build: elsewhere the
// suite is run small enough not to trip ctest's timeout, and the failure
// counts are recorded rather than asserted (an unoptimised ProxQP also
// reaches max_iter differently).
[[nodiscard]] bool OptimisedBuild() {
  const std::string bt = RTC_TEST_BUILD_TYPE;
  return bt == "Release" || bt == "RelWithDebInfo" || bt == "MinSizeRel";
}

void RunTiming(const ArmModel& arm, const MpcSegmentCoreParams& p, const MpcSegmentCoreLimits& lim,
               const std::string& key, int n_samples = 200, double* max_dev = nullptr,
               const MpcSegmentCoreParams* dev_ref_params = nullptr,
               bool require_no_failures = true) {
  if (!OptimisedBuild()) {
    n_samples = std::min(n_samples, 10);
    require_no_failures = false;
  }
  const int n = arm.model->nv;
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), MpcSegmentCoreReason::kNone);
  MpcSegmentCore ref;
  if (dev_ref_params != nullptr) {
    ASSERT_EQ(ref.Init(*arm.model, arm.frame, *dev_ref_params, lim), MpcSegmentCoreReason::kNone);
  }
  MpcSegmentCoreResult res, rref;
  mpc.ResizeResult(res);
  ref.ResizeResult(rref);
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  Samples cold, warm;
  double dev = 0.0;
  pinocchio::Data fk_data(*arm.model);
  for (int i = 0; i < n_samples; ++i) {
    MpcSegmentCoreInput in = RestInput(arm.q_nominal);
    for (int j = 0; j < n; ++j) {
      in.q0[j] += 0.2 * uni(rng);
      in.qd0[j] = 0.6 * p.eta_v * lim.qd_max[j] * uni(rng);
      in.qdd0[j] = 2.0 * uni(rng);
    }
    if (p.w_perp > 0.0) {
      // What the planner would pass: the catch point at the TCP and the line
      // along the TCP velocity at x_0.
      Eigen::MatrixXd j6 = Eigen::MatrixXd::Zero(6, n);
      pinocchio::computeFrameJacobian(*arm.model, fk_data, in.q0, arm.frame,
                                      pinocchio::LOCAL_WORLD_ALIGNED, j6);
      in.p_c = fk_data.oMf[arm.frame].translation();
      const Eigen::Vector3d v_tcp = j6.topRows(3) * in.qd0;
      in.d_hat =
          v_tcp.norm() > 1e-9 ? Eigen::Vector3d(v_tcp.normalized()) : Eigen::Vector3d::UnitX();
    }
    if (mpc.Solve(in, res)) {
      cold.Add(res);
    } else {
      cold.Fail(res);
      continue;
    }
    MpcSegmentCoreInput next = in;
    ShiftReference(res, p.dt, 0.02, next);
    if (mpc.Solve(next, res)) {
      warm.Add(res);
      if (dev_ref_params != nullptr && ref.Solve(next, rref)) {
        dev = std::max(dev, (res.q - rref.q).cwiseAbs().maxCoeff());
      }
    } else {
      warm.Fail(res);
    }
  }
  Report(key + ".cold", cold);
  Report(key + ".warm", warm);
  ::testing::Test::RecordProperty("build_type", RTC_TEST_BUILD_TYPE);
  if (max_dev != nullptr) {
    *max_dev = dev;
  }
  if (require_no_failures) {
    EXPECT_EQ(cold.failures, 0) << key;
    EXPECT_EQ(warm.failures, 0) << key;
  }
}

TEST(MpcSegmentCoreTiming, TimingDistribution6R) {
  const ArmModel arm = RealArm6();
  MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model, 0.1);
  MpcSegmentCoreParams p;
  RunTiming(arm, p, lim, "real_6dof");
  MpcSegmentCoreParams dense = p;
  dense.reference_assembly = true;
  RunTiming(arm, dense, lim, "real_6dof.dense_assembly");
  // Torque rows binding (the fixture's tight first joint).
  const TorqueFixture f = MakeTorqueFixture(0.1);
  // Informational: with a binding torque row ProxQP still returns a few
  // false PRIMAL_INFEASIBLE verdicts on warm cycles (~3 % at ρ_τ = 10); the
  // count is recorded, and the caller's fallback covers them.
  RunTiming(f.arm, f.params, f.limits, "real_6dof.torque_active", 200, nullptr, nullptr, false);
}

TEST(MpcSegmentCoreTiming, TimingDistribution7R) {
  const ArmModel arm = RealArm7();
  MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model, 0.2);
  MpcSegmentCoreParams p;
  RunTiming(arm, p, lim, "real_7dof");
  MpcSegmentCoreParams dense = p;
  dense.reference_assembly = true;
  RunTiming(arm, dense, lim, "real_7dof.dense_assembly");
  MpcSegmentCoreParams perp = p;
  perp.w_perp = 10.0;
  RunTiming(arm, perp, lim, "real_7dof.w_perp_on");
}

// O-4: solver tolerance sweep — warm solve time and iterations against the
// trajectory deviation from a tight-tolerance solve of the same inputs.
TEST(MpcSegmentCoreTiming, SolverToleranceSweep7R) {
  const ArmModel arm = RealArm7();
  const MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model, 0.2);
  MpcSegmentCoreParams tight;
  tight.solver.eps_abs = 1e-9;
  tight.solver.max_iter = 1000;
  for (const double eps : {1e-4, 1e-5, 1e-6, 1e-7}) {
    MpcSegmentCoreParams p;
    p.solver.eps_abs = eps;
    p.reference_rest_tol = 10.0 * eps;  // a shifted reference rests only to eps_abs
    double dev = 0.0;
    char key[64];
    std::snprintf(key, sizeof(key), "real_7dof.eps_%.0e", eps);
    RunTiming(arm, p, lim, key, 100, &dev, &tight);
    RecordProperty(std::string(key) + ".max_q_dev_urad", static_cast<int>(std::lround(1e6 * dev)));
    std::printf("[tolerance] eps_abs %.0e: max |q - q_tight| = %.3g rad\n", eps, dev);
  }
}

// A velocity drift between cycles (x_0 off the reference's node 0 in q̇, not
// q) does NOT make the trust region infeasible: jerk is unbounded in the QP,
// so node 1 is correctable and the remaining blocks absorb the rest. Pinned
// because a review predicted a slow infeasibility verdict here; the solve
// stays a few iterations.
TEST(MpcSegmentCore, VelocityDriftIsAbsorbedByJerk) {
  const ArmModel arm = RealArm7();
  MpcSegmentCoreParams p;
  const MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model, 0.2);
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  MpcSegmentCoreInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.6, -0.4, 0.5, 0.7, -0.3, 0.4, 0.2;
  ASSERT_TRUE(mpc.Solve(in, res));
  for (const double dv : {0.5, 1.0, 2.5}) {
    MpcSegmentCoreInput drift = in;
    UseAsReference(res, drift);
    drift.qd0[3] = std::min(drift.qd0[3] + dv, p.eta_v * lim.qd_max[3]);
    MpcSegmentCoreResult r;
    mpc.ResizeResult(r);
    EXPECT_TRUE(mpc.Solve(drift, r)) << "dv=" << dv << " " << MpcSegmentCoreReasonName(r.reason);
    EXPECT_LE(r.iterations, 20) << "dv=" << dv;
    EXPECT_LE(std::abs(r.q(3, 1) - drift.q_ref(3, 1)), p.delta_tr + 1e-6) << "dv=" << dv;
  }
}

// MpcSegmentCoreLimits::armature is ADDED to the armature the model carries — the
// same torque rows come out of (model 0.1 + limits 0.1) and (model 0 +
// limits 0.2).
TEST(MpcSegmentCore, LimitsArmatureAddsToModelArmature) {
  const TorqueFixture f = MakeTorqueFixture();
  const int n = f.arm.model->nv;
  pinocchio::Model carrying = *f.arm.model;
  carrying.armature = Eigen::VectorXd::Constant(n, 0.1);
  MpcSegmentCoreLimits half = f.limits;
  half.armature = Eigen::VectorXd::Constant(n, 0.1);
  MpcSegmentCoreLimits full = f.limits;
  full.armature = Eigen::VectorXd::Constant(n, 0.2);
  MpcSegmentCoreLimits none = f.limits;
  none.armature.setZero();

  const auto ratios = [&](const pinocchio::Model& m, const MpcSegmentCoreLimits& l) {
    MpcSegmentCore mpc;
    EXPECT_EQ(mpc.Init(m, f.arm.frame, f.params, l), MpcSegmentCoreReason::kNone);
    MpcSegmentCoreResult r;
    mpc.ResizeResult(r);
    EXPECT_TRUE(mpc.Solve(f.input, r)) << MpcSegmentCoreReasonName(r.reason);
    return Eigen::MatrixXd(r.tau_ratio);
  };
  const Eigen::MatrixXd a = ratios(carrying, half);
  const Eigen::MatrixXd b = ratios(*f.arm.model, full);
  const Eigen::MatrixXd c = ratios(carrying, none);
  EXPECT_LE((a - b).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_GT((a - c).cwiseAbs().maxCoeff(), 1e-3) << "the model's own armature must not be dropped";
}

}  // namespace
