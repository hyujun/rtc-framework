// E1-F01 (#627): decel MPC core — jerk-input condensed QP with first-order
// torque rows and slack (decel_mpc.hpp). Each #627 "Done when" item maps to a
// named test here (plan §4):
//   1 ZeroSolutionAtRest                 5 TheAllocationGatesAreArmed,
//   2 ReducesToMinJerkClosedForm,          CoreAllocatesNothingOutsideTheQpSolver,
//     PresolveMatchesClosedForm            PresolveAndRejectionPathsAllocateNothing
//   3 SlackIsZeroWhenFeasible            6 Rejects* / *FailsClosed
//   4 TerminalEqualityIsFullRank         7 ReevaluatedTorqueWithinLimits
//                                        8 TimingDistribution*
// plus the seam checks (derivatives, offset sign, armature) and the
// structured-vs-dense assembly oracle for the optimisation (plan §9).
//
// The allocation gates: this binary links THREE sensors (CMakeLists note).
// malloc_gate.hpp is the one that can see pinocchio's and ProxQP's own
// allocations; its positive control allocates inside pinocchio.
#include "rtc_controllers/catching/decel_mpc.hpp"
#include "rtc_controllers/catching/decel_mpc_torque.hpp"
#include "rtc_controllers/catching/jerk_segment.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/decel_mpc_fixture.hpp"
#include "rtc_controllers/testing/malloc_gate.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {

using rtc::catching::DecelMpc;
using rtc::catching::DecelMpcInput;
using rtc::catching::DecelMpcLimits;
using rtc::catching::DecelMpcParams;
using rtc::catching::DecelMpcReason;
using rtc::catching::DecelMpcReasonName;
using rtc::catching::DecelMpcResult;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// ── Fixtures ─────────────────────────────────────────────────────────────────

// Arms, limits and the rest input live in decel_mpc_fixture.hpp, shared with
// the E1-F07 approach suite.
using rtc::testing::decel::ArmModel;
using rtc::testing::decel::LimitsFromModel;
using rtc::testing::decel::Percentile;
using rtc::testing::decel::RealArm6;
using rtc::testing::decel::RealArm7;
using rtc::testing::decel::RecordMicros;
using rtc::testing::decel::RestInput;
using rtc::testing::decel::Synthetic6R;

void UseAsReference(const DecelMpcResult& r, DecelMpcInput& in) {
  in.q_ref = r.q;
  in.qd_ref = r.qd;
  in.qdd_ref = r.qdd;
  in.reference_valid = true;
}

// A solved trajectory shifted by `t_shift` onto the same grid (the E1-F03
// warm start): node k of the new reference is the old trajectory at t_shift+kΔ.
void ShiftReference(const DecelMpcResult& r, double dt, double t_shift, DecelMpcInput& in) {
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

int Rank(const Eigen::MatrixXd& a) {
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(a);
  const Eigen::VectorXd& s = svd.singularValues();
  const double tol = 1e-10 * std::max(1.0, s.size() > 0 ? s[0] : 0.0);
  return static_cast<int>((s.array() > tol).count());
}

// ── 1. Zero solution at rest ─────────────────────────────────────────────────

TEST(DecelMpc, ZeroSolutionAtRest) {
  const ArmModel arm = Synthetic6R();
  DecelMpcLimits lim = LimitsFromModel(*arm.model);
  // Precondition: gravity at q0 is inside η'τ_max, otherwise the torque rows
  // would (correctly) demand slack and the answer is not zero.
  pinocchio::Data data(*arm.model);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(arm.model->nv);
  const Eigen::VectorXd g = Rnea(*arm.model, data, arm.q_nominal, zero, zero);
  DecelMpcParams p;
  lim.tau_max = lim.tau_max.cwiseMax(3.0 * g.cwiseAbs() / p.eta_tau);

  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), DecelMpcReason::kNone);
  DecelMpcResult res;
  mpc.ResizeResult(res);

  const double z_tol = 10.0 * p.solver.eps_abs;
  for (const bool with_reference : {false, true}) {
    DecelMpcInput in = RestInput(arm.q_nominal);
    if (with_reference) {
      in.q_ref = arm.q_nominal.replicate(1, p.n_nodes + 1);
      in.qd_ref.setZero(arm.model->nv, p.n_nodes + 1);
      in.qdd_ref.setZero(arm.model->nv, p.n_nodes + 1);
      in.reference_valid = true;
    }
    ASSERT_TRUE(mpc.Solve(in, res)) << DecelMpcReasonName(res.reason);
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

Eigen::MatrixXd ClosedFormJerk(const DecelMpcParams& p, const Eigen::VectorXd& v0,
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

DecelMpcParams KinematicOnlyParams() {
  DecelMpcParams p;
  p.rho_tau = 0.0;
  p.w_delta = 0.0;
  p.w_perp = 0.0;
  p.delta_tr = kInf;
  return p;
}

TEST(DecelMpc, ReducesToMinJerkClosedForm) {
  const ArmModel arm = Synthetic6R();
  const DecelMpcParams p = KinematicOnlyParams();
  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)), DecelMpcReason::kNone);
  DecelMpcResult res;
  mpc.ResizeResult(res);

  DecelMpcInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.30, -0.25, 0.20, -0.15, 0.35, -0.30;
  in.qdd0 << 1.0, -0.5, 0.8, 0.0, -1.2, 0.4;
  // A resting reference that is NOT the answer: the main solve (not the
  // pre-solve) has to produce the closed form.
  in.q_ref = arm.q_nominal.replicate(1, p.n_nodes + 1);
  in.qd_ref.setZero(6, p.n_nodes + 1);
  in.qdd_ref.setZero(6, p.n_nodes + 1);
  in.reference_valid = true;

  ASSERT_TRUE(mpc.Solve(in, res)) << DecelMpcReasonName(res.reason);
  EXPECT_FALSE(res.presolved);
  const Eigen::MatrixXd u_ref = ClosedFormJerk(p, in.qd0, in.qdd0);
  const double scale = u_ref.cwiseAbs().maxCoeff();
  EXPECT_LE((res.u - u_ref).cwiseAbs().maxCoeff(), 1e-6 * scale) << "MPC\n"
                                                                 << res.u << "\nclosed form\n"
                                                                 << u_ref;
  EXPECT_LE(res.qd.col(p.n_nodes).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_LE(res.qdd.col(p.n_nodes).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(DecelMpc, PresolveMatchesClosedForm) {
  const ArmModel arm = Synthetic6R();
  const DecelMpcParams p = KinematicOnlyParams();
  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)), DecelMpcReason::kNone);
  DecelMpcResult res;
  mpc.ResizeResult(res);
  DecelMpcInput in = RestInput(arm.q_nominal);
  in.qd0 << -0.2, 0.3, -0.1, 0.25, -0.3, 0.15;
  ASSERT_TRUE(mpc.Solve(in, res)) << DecelMpcReasonName(res.reason);
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

TEST(DecelMpc, StageGainsMatchIntegralOracle) {
  const ArmModel arm = Synthetic6R();
  const DecelMpcParams p;
  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)), DecelMpcReason::kNone);
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

TEST(DecelMpc, TerminalEqualityIsFullRank) {
  const ArmModel arm = Synthetic6R();
  const int n = arm.model->nv;
  // The rank helper itself: one block → rank n; two blocks → rank 2n and no
  // freedom left (nullity 0) — that is why B ≥ 3 is required, not the rank.
  EXPECT_EQ(Rank(TerminalMatrixOracle(n, 12, {12}, 0.05)), n);
  EXPECT_EQ(Rank(TerminalMatrixOracle(n, 12, {6, 6}, 0.05)), 2 * n);
  EXPECT_EQ(TerminalMatrixOracle(n, 12, {6, 6}, 0.05).cols() - 2 * n, 0);

  for (const std::vector<int>& blocks :
       {std::vector<int>{4, 4, 4}, std::vector<int>{1, 1, 2, 2, 3, 3}}) {
    DecelMpcParams p;
    p.n_blocks = static_cast<int>(blocks.size());
    std::copy(blocks.begin(), blocks.end(), p.block_sizes.begin());
    DecelMpc mpc;
    ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)),
              DecelMpcReason::kNone);
    EXPECT_EQ(mpc.TerminalRank(), 2 * n);
    const Eigen::MatrixXd a = mpc.MainQp().A;
    EXPECT_EQ(Rank(a), 2 * n) << "B=" << blocks.size();
    // Assembled matrix = oracle (scaled by u_scale).
    EXPECT_LE(
        (a - p.u_scale * TerminalMatrixOracle(n, p.n_nodes, blocks, p.dt)).cwiseAbs().maxCoeff(),
        1e-9 * p.u_scale);
  }
  DecelMpcParams two;
  two.n_blocks = 2;
  two.block_sizes[0] = 6;
  two.block_sizes[1] = 6;
  DecelMpc mpc;
  EXPECT_EQ(mpc.Init(*arm.model, arm.frame, two, LimitsFromModel(*arm.model)),
            DecelMpcReason::kBlocksTooFew);
}

// ── Torque fixture: rows ACTIVE and the hard problem feasible ────────────────
// The real 6-dof arm's first joint has a vertical axis, so its torque is purely
// inertial: the stop's peak torque on it can be shaved by redistributing jerk
// over the fixed horizon. τ_max(0) is set 5 % below the unconstrained peak so
// its row must bind; every other joint keeps a loose limit.

struct TorqueFixture {
  ArmModel arm;
  DecelMpcParams params;
  DecelMpcLimits limits;
  DecelMpcInput input;
};

TorqueFixture MakeTorqueFixture(double armature = 0.0) {
  TorqueFixture f{RealArm6(), DecelMpcParams{}, {}, {}};
  const pinocchio::Model& m = *f.arm.model;
  f.limits = LimitsFromModel(m, armature);
  f.limits.tau_max *= 10.0;
  f.input = RestInput(f.arm.q_nominal);
  f.input.qd0 << 1.2, -0.6, 0.8, 0.5, 0.4, 0.4;

  DecelMpc loose;
  EXPECT_EQ(loose.Init(m, f.arm.frame, f.params, f.limits), DecelMpcReason::kNone);
  DecelMpcResult r;
  loose.ResizeResult(r);
  EXPECT_TRUE(loose.Solve(f.input, r)) << DecelMpcReasonName(r.reason);
  double peak = 0.0;
  for (int k = 1; k <= f.params.n_nodes; ++k) {
    peak = std::max(peak, std::abs(r.tau_ratio(0, k - 1)) * f.limits.tau_max[0]);
  }
  f.limits.tau_max[0] = peak / (f.params.eta_tau * 1.05);
  return f;
}

// Solve the fixture's last main QP with the slack pinned to 0 (the hard
// problem). Returns false when it is infeasible.
bool SolveHard(const DecelMpc& mpc, const DecelMpcParams& p, Eigen::VectorXd& z) {
  rtc::tsid::QPData hard = mpc.MainQp();
  const Eigen::Index n_in = hard.C.rows();
  const Eigen::Index nN = n_in / 5;
  hard.u.tail(nN).setZero();
  hard.g.tail(nN).setZero();
  rtc::tsid::QPSolverWrapper solver;
  solver.Init(static_cast<int>(hard.H.rows()), static_cast<int>(hard.A.rows()),
              static_cast<int>(n_in), p.solver);
  const rtc::tsid::SolveResult& res = solver.Solve(hard);
  z = res.x_opt;
  return res.converged;
}

Eigen::MatrixXd JerkFromZ(const Eigen::VectorXd& z, const DecelMpcParams& p, int n) {
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

TEST(DecelMpc, SlackIsZeroWhenFeasible) {
  const TorqueFixture f = MakeTorqueFixture();
  const int n = f.arm.model->nv;
  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*f.arm.model, f.arm.frame, f.params, f.limits), DecelMpcReason::kNone);
  DecelMpcResult res;
  mpc.ResizeResult(res);
  ASSERT_TRUE(mpc.Solve(f.input, res)) << DecelMpcReasonName(res.reason);

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
    DecelMpcParams cheap = f.params;
    cheap.rho_tau = 1e-8;
    DecelMpc m2;
    ASSERT_EQ(m2.Init(*f.arm.model, f.arm.frame, cheap, f.limits), DecelMpcReason::kNone);
    DecelMpcResult r2;
    m2.ResizeResult(r2);
    ASSERT_TRUE(m2.Solve(f.input, r2)) << DecelMpcReasonName(r2.reason);
    EXPECT_GT(r2.slack_max, 1e-3) << "ρ_τ too small must leave slack";
  }
  // Positive control: a static-torque limit below gravity at the stop is
  // infeasible for the hard problem; the soft one reports terminal slack.
  {
    DecelMpcLimits tight = f.limits;
    pinocchio::Data data(*f.arm.model);
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n);
    const Eigen::VectorXd g = Rnea(*f.arm.model, data, f.arm.q_nominal, zero, zero);
    tight.tau_max[1] = 0.5 * std::abs(g[1]) / f.params.eta_tau;
    DecelMpc m3;
    ASSERT_EQ(m3.Init(*f.arm.model, f.arm.frame, f.params, tight), DecelMpcReason::kNone);
    DecelMpcResult r3;
    m3.ResizeResult(r3);
    ASSERT_TRUE(m3.Solve(f.input, r3)) << DecelMpcReasonName(r3.reason);
    EXPECT_GT(r3.slack_terminal_max, 0.1);
    Eigen::VectorXd z3;
    EXPECT_FALSE(SolveHard(m3, f.params, z3));
  }
}

// ── 7. Re-evaluated torque inside the limit ──────────────────────────────────

TEST(DecelMpc, ReevaluatedTorqueWithinLimits) {
  // Armature on (the model copy inside the core carries it; the oracle below
  // must too). One RTI step: the pre-solve path linearises exactly once.
  const double armature = 0.1;
  const TorqueFixture f = MakeTorqueFixture(armature);
  const int n = f.arm.model->nv;
  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*f.arm.model, f.arm.frame, f.params, f.limits), DecelMpcReason::kNone);
  DecelMpcResult res;
  mpc.ResizeResult(res);
  ASSERT_TRUE(mpc.Solve(f.input, res)) << DecelMpcReasonName(res.reason);
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

TEST(DecelMpc, TorqueLinearizationMatchesFiniteDifference) {
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

TEST(DecelMpc, ArmatureEntersTorqueRows) {
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
TEST(DecelMpc, TorqueLinearizationErrorIsSecondOrder) {
  const ArmModel arm = RealArm6();
  const int n = arm.model->nv;
  DecelMpcParams p;
  p.delta_tr = kInf;
  DecelMpcLimits lim = LimitsFromModel(*arm.model);
  lim.tau_max *= 10.0;  // rows present, not binding: the error is the model's alone
  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), DecelMpcReason::kNone);
  DecelMpcResult base;
  mpc.ResizeResult(base);
  DecelMpcInput in = RestInput(arm.q_nominal);
  in.qd0 << 1.0, -0.8, 0.9, 0.6, 0.5, 0.5;
  ASSERT_TRUE(mpc.Solve(in, base));

  pinocchio::Data data(*arm.model);
  const auto error_at = [&](double delta) {
    DecelMpcInput pin = in;
    UseAsReference(base, pin);
    for (int k = 1; k < p.n_nodes; ++k) {
      for (int j = 0; j < n; ++j) {
        const double s = std::sin(1.3 * k + 0.7 * j);
        pin.q_ref(j, k) += delta * s;
        pin.qd_ref(j, k) += 4.0 * delta * s;
        pin.qdd_ref(j, k) += 20.0 * delta * s;
      }
    }
    DecelMpcResult r;
    mpc.ResizeResult(r);
    EXPECT_TRUE(mpc.Solve(pin, r)) << DecelMpcReasonName(r.reason);
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

// ── w_⊥ (D-5: implemented, default off) ─────────────────────────────────────

TEST(DecelMpc, PerpendicularTermReducesOffLineError) {
  const ArmModel arm = RealArm6();
  const int n = arm.model->nv;
  pinocchio::Data data(*arm.model);
  DecelMpcInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.8, -0.6, 0.9, 0.4, 0.5, 0.3;
  Eigen::MatrixXd j6 = Eigen::MatrixXd::Zero(6, n);
  pinocchio::computeFrameJacobian(*arm.model, data, arm.q_nominal, arm.frame,
                                  pinocchio::LOCAL_WORLD_ALIGNED, j6);
  in.p_c = data.oMf[arm.frame].translation();
  const Eigen::Vector3d v_tcp = j6.topRows(3) * in.qd0;
  in.d_hat = v_tcp.normalized();

  const auto off_line = [&](double w) {
    DecelMpcParams p;
    p.w_perp = w;
    DecelMpc mpc;
    EXPECT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)),
              DecelMpcReason::kNone);
    DecelMpcResult r;
    mpc.ResizeResult(r);
    EXPECT_TRUE(mpc.Solve(in, r)) << DecelMpcReasonName(r.reason);
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

TEST(DecelMpc, StructuredAssemblyMatchesDense) {
  const TorqueFixture f = MakeTorqueFixture(0.1);
  DecelMpcParams p = f.params;
  p.w_perp = 10.0;
  DecelMpcInput in = f.input;
  pinocchio::Data data(*f.arm.model);
  pinocchio::framesForwardKinematics(*f.arm.model, data, f.arm.q_nominal);
  in.p_c = data.oMf[f.arm.frame].translation();
  in.d_hat = Eigen::Vector3d(1.0, 2.0, -0.5).normalized();

  DecelMpc fast, dense;
  DecelMpcParams pd = p;
  pd.reference_assembly = true;
  ASSERT_EQ(fast.Init(*f.arm.model, f.arm.frame, p, f.limits), DecelMpcReason::kNone);
  ASSERT_EQ(dense.Init(*f.arm.model, f.arm.frame, pd, f.limits), DecelMpcReason::kNone);
  DecelMpcResult rf, rd;
  fast.ResizeResult(rf);
  dense.ResizeResult(rd);
  // Two cycles: pre-solve path, then a shifted warm cycle.
  for (int cycle = 0; cycle < 2; ++cycle) {
    DecelMpcInput ci = in;
    if (cycle == 1) {
      ShiftReference(rf, p.dt, 0.02, ci);
    }
    ASSERT_TRUE(fast.Solve(ci, rf)) << DecelMpcReasonName(rf.reason);
    ASSERT_TRUE(dense.Solve(ci, rd)) << DecelMpcReasonName(rd.reason);
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

TEST(DecelMpc, TheAllocationGatesAreArmed) {
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

AllocCounts GatedSolve(DecelMpc& mpc, const DecelMpcInput& in, DecelMpcResult& res, bool& ok) {
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

DecelMpcInput PerpInput(const ArmModel& arm, const DecelMpcInput& base) {
  DecelMpcInput in = base;
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
TEST(DecelMpc, CoreAllocatesNothingOutsideTheQpSolver) {
  const ArmModel arm = RealArm7();
  DecelMpcParams p;
  p.w_perp = 10.0;  // the FK / frame-Jacobian path is measured too
  const DecelMpcLimits lim = LimitsFromModel(*arm.model, 0.2);
  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), DecelMpcReason::kNone);
  DecelMpcResult res;
  mpc.ResizeResult(res);
  DecelMpcInput in = PerpInput(arm, RestInput(arm.q_nominal));
  in.qd0 << 0.6, -0.4, 0.5, 0.7, -0.3, 0.4, 0.2;
  ASSERT_TRUE(mpc.Solve(in, res)) << DecelMpcReasonName(res.reason);
  DecelMpcInput warm = in;
  ShiftReference(res, p.dt, 0.02, warm);
  ASSERT_TRUE(mpc.Solve(warm, res)) << DecelMpcReasonName(res.reason);  // warm-up
  DecelMpcInput next = warm;
  ShiftReference(res, p.dt, 0.02, next);  // prepared outside the gate
  DecelMpcInput outside = next;           // reference leaves the box by > δ
  for (int k = 1; k <= p.n_nodes; ++k) {
    outside.q_ref(2, k) = lim.q_max[2] + 3.0 * p.delta_tr;
  }

  bool ok = false;
  const AllocCounts full = GatedSolve(mpc, next, res, ok);
  EXPECT_TRUE(ok) << DecelMpcReasonName(res.reason);
  EXPECT_EQ(full.op_new, 0U) << "operator new inside Solve";
  RecordProperty("warm_solve_qp_solver_mallocs", static_cast<int>(full.c_malloc));
  std::printf("[alloc] warm Solve: %zu C mallocs (ProxQP, known limitation)\n", full.c_malloc);

  const AllocCounts core = GatedSolve(mpc, outside, res, ok);
  EXPECT_FALSE(ok);
  ASSERT_EQ(res.reason, DecelMpcReason::kTrustRegionConflict);
  EXPECT_GT(res.linearize_us, 0.0) << "the rejection must come after linearisation";
  EXPECT_GT(res.condense_us, 0.0) << "the rejection must come after condensing";
  EXPECT_EQ(core.op_new, 0U);
  EXPECT_EQ(core.c_malloc, 0U) << "C-level allocation in linearisation / condensing";
}

TEST(DecelMpc, PresolveAndRejectionPathsAllocateNothing) {
  const ArmModel arm = RealArm7();
  DecelMpcParams p;
  p.w_perp = 10.0;
  DecelMpc mpc;
  const DecelMpcLimits lim = LimitsFromModel(*arm.model, 0.2);
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), DecelMpcReason::kNone);
  DecelMpcResult res;
  mpc.ResizeResult(res);
  DecelMpcInput in = PerpInput(arm, RestInput(arm.q_nominal));
  in.qd0 << 0.6, -0.4, 0.5, 0.7, -0.3, 0.4, 0.2;
  ASSERT_TRUE(mpc.Solve(in, res));  // warm-up of the pre-solve path

  bool ok = false;
  AllocCounts c = GatedSolve(mpc, in, res, ok);
  EXPECT_TRUE(ok);
  EXPECT_EQ(c.op_new, 0U) << "pre-solve path";
  RecordProperty("presolve_path_qp_solver_mallocs", static_cast<int>(c.c_malloc));

  DecelMpcInput nan_in = in;
  nan_in.qd0[2] = kNan;
  c = GatedSolve(mpc, nan_in, res, ok);
  EXPECT_FALSE(ok);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "non-finite rejection";

  DecelMpcInput drift = in;
  UseAsReference(res, drift);
  drift.q0[0] += 2.0 * p.delta_tr;
  c = GatedSolve(mpc, drift, res, ok);
  EXPECT_FALSE(ok);
  EXPECT_EQ(res.reason, DecelMpcReason::kTrustRegionConflict);
  EXPECT_EQ(c.op_new + c.c_malloc, 0U) << "trust-region rejection";

  // QP failure: at the upper edge of the box, moving outward at the velocity
  // limit — node 1 cannot stay inside.
  DecelMpcInput edge = RestInput(arm.q_nominal);
  edge.q0[3] = lim.q_max[3] - p.m_q - 1e-9;
  edge.qd0[3] = p.eta_v * lim.qd_max[3];
  edge.p_c = in.p_c;
  edge.d_hat = in.d_hat;
  c = GatedSolve(mpc, edge, res, ok);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(res.reason == DecelMpcReason::kPresolveFailed ||
              res.reason == DecelMpcReason::kQpFailed)
      << DecelMpcReasonName(res.reason);
  EXPECT_EQ(c.op_new, 0U) << "QP failure path";
  RecordProperty("qp_failure_path_qp_solver_mallocs", static_cast<int>(c.c_malloc));
}

// ── 6. Fail-closed ───────────────────────────────────────────────────────────

struct Snapshot {
  Eigen::MatrixXd q, qd, qdd, u, slack, tau_ratio;

  explicit Snapshot(const DecelMpcResult& r)
      : q(r.q), qd(r.qd), qdd(r.qdd), u(r.u), slack(r.slack), tau_ratio(r.tau_ratio) {}

  [[nodiscard]] bool Same(const DecelMpcResult& r) const {
    return q == r.q && qd == r.qd && qdd == r.qdd && u == r.u && slack == r.slack &&
           tau_ratio == r.tau_ratio;
  }
};

class DecelMpcFailClosed : public ::testing::Test {
 protected:
  void SetUp() override {
    arm_ = Synthetic6R();
    params_.w_perp = 1.0;
    limits_ = LimitsFromModel(*arm_.model);
    ASSERT_EQ(mpc_.Init(*arm_.model, arm_.frame, params_, limits_), DecelMpcReason::kNone);
    mpc_.ResizeResult(res_);
    good_ = PerpInput(arm_, RestInput(arm_.q_nominal));
    good_.qd0 << 0.3, -0.2, 0.25, 0.1, -0.2, 0.3;
    ASSERT_TRUE(mpc_.Solve(good_, res_));
  }

  void ExpectRejected(const DecelMpcInput& in, DecelMpcReason why, const char* what) {
    const Snapshot before(res_);
    EXPECT_FALSE(mpc_.Solve(in, res_)) << what;
    EXPECT_EQ(res_.reason, why) << what << ": got " << DecelMpcReasonName(res_.reason);
    EXPECT_FALSE(res_.valid) << what;
    EXPECT_TRUE(before.Same(res_)) << what << ": outputs changed on failure";
  }

  ArmModel arm_;
  DecelMpcParams params_;
  DecelMpcLimits limits_;
  DecelMpc mpc_;
  DecelMpcResult res_;
  DecelMpcInput good_;
};

TEST_F(DecelMpcFailClosed, RejectsNonFiniteInput) {
  for (const double bad : {kNan, kInf, -kInf}) {
    DecelMpcInput in = good_;
    in.q0[1] = bad;
    ExpectRejected(in, DecelMpcReason::kNonFinite, "q0");
    in = good_;
    in.qd0[4] = bad;
    ExpectRejected(in, DecelMpcReason::kNonFinite, "qd0");
    in = good_;
    in.qdd0[0] = bad;
    ExpectRejected(in, DecelMpcReason::kNonFinite, "qdd0");
    in = good_;
    UseAsReference(res_, in);
    in.q_ref(2, 5) = bad;
    ExpectRejected(in, DecelMpcReason::kNonFinite, "q_ref");
    in = good_;
    UseAsReference(res_, in);
    in.qdd_ref(0, 3) = bad;
    ExpectRejected(in, DecelMpcReason::kNonFinite, "qdd_ref");
    in = good_;
    in.p_c[1] = bad;
    ExpectRejected(in, DecelMpcReason::kNonFinite, "p_c");
    in = good_;
    in.d_hat[2] = bad;
    ExpectRejected(in, DecelMpcReason::kNonFinite, "d_hat");
  }
  DecelMpcInput in = good_;
  in.d_hat = Eigen::Vector3d(1.0, 1.0, 0.0);
  ExpectRejected(in, DecelMpcReason::kDirectionNotUnit, "d_hat not unit");
}

TEST_F(DecelMpcFailClosed, RejectsDimensionMismatch) {
  DecelMpcInput in = good_;
  in.q0.resize(5);
  in.q0.setZero();
  ExpectRejected(in, DecelMpcReason::kDimMismatch, "q0 size");
  in = good_;
  UseAsReference(res_, in);
  in.q_ref.conservativeResize(Eigen::NoChange, params_.n_nodes);
  ExpectRejected(in, DecelMpcReason::kDimMismatch, "reference columns");
  DecelMpcResult wrong;  // never resized
  EXPECT_FALSE(mpc_.Solve(good_, wrong));
  EXPECT_EQ(wrong.reason, DecelMpcReason::kDimMismatch);
  EXPECT_EQ(wrong.q.size(), 0) << "Solve must not resize";
}

TEST_F(DecelMpcFailClosed, RejectsInitialStateOutsideBox) {
  DecelMpcInput in = good_;
  in.q0[2] = limits_.q_min[2] + 0.5 * params_.m_q;
  ExpectRejected(in, DecelMpcReason::kInitialStateOutsideBox, "q0 in margin");
  in = good_;
  in.qd0[1] = -1.01 * params_.eta_v * limits_.qd_max[1];
  ExpectRejected(in, DecelMpcReason::kInitialStateOutsideBox, "qd0 over eta_v");
}

TEST_F(DecelMpcFailClosed, RejectsReferenceNotAtRest) {
  DecelMpcInput in = good_;
  UseAsReference(res_, in);
  in.qd_ref(3, params_.n_nodes) = 0.1;
  ExpectRejected(in, DecelMpcReason::kReferenceNotAtRest, "terminal velocity");
  in = good_;
  UseAsReference(res_, in);
  in.qdd_ref(0, params_.n_nodes) = -0.1;
  ExpectRejected(in, DecelMpcReason::kReferenceNotAtRest, "terminal acceleration");
}

TEST_F(DecelMpcFailClosed, TrustRegionConflictFailsClosed) {
  DecelMpcInput in = good_;
  UseAsReference(res_, in);
  in.q0[5] += 1.5 * params_.delta_tr;
  ExpectRejected(in, DecelMpcReason::kTrustRegionConflict, "x0 drifted");
  // A reference outside the box by more than δ: lower > upper on its row.
  in = good_;
  UseAsReference(res_, in);
  for (int k = 1; k <= params_.n_nodes; ++k) {
    in.q_ref(2, k) = limits_.q_max[2] + 3.0 * params_.delta_tr;
  }
  ExpectRejected(in, DecelMpcReason::kTrustRegionConflict, "reference outside box");
}

TEST(DecelMpc, RejectsBeforeInit) {
  DecelMpc mpc;
  DecelMpcResult res;
  res.q.setZero(6, 13);
  EXPECT_FALSE(mpc.IsInitialized());
  EXPECT_FALSE(mpc.Solve(RestInput(Eigen::VectorXd::Zero(6)), res));
  EXPECT_EQ(res.reason, DecelMpcReason::kNotInitialized);
}

TEST(DecelMpc, InitValidatesParamsAndLimits) {
  const ArmModel arm = Synthetic6R();
  const DecelMpcLimits good = LimitsFromModel(*arm.model);
  const auto init = [&](const DecelMpcParams& p, const DecelMpcLimits& l) {
    DecelMpc mpc;
    const DecelMpcReason r = mpc.Init(*arm.model, arm.frame, p, l);
    EXPECT_EQ(mpc.IsInitialized(), r == DecelMpcReason::kNone);
    return r;
  };
  DecelMpcParams p;
  EXPECT_EQ(init(p, good), DecelMpcReason::kNone);
  p.block_sizes[5] = 4;  // Σ = 13 ≠ N
  EXPECT_EQ(init(p, good), DecelMpcReason::kParamsInvalid);
  p = DecelMpcParams{};
  p.n_nodes = rtc::catching::kMaxDecelNodes + 1;
  EXPECT_EQ(init(p, good), DecelMpcReason::kParamsInvalid);
  for (const double bad : {0.0, -1.0, kNan}) {
    p = DecelMpcParams{};
    p.dt = bad;
    EXPECT_EQ(init(p, good), DecelMpcReason::kParamsInvalid) << "dt " << bad;
    p = DecelMpcParams{};
    p.eta_tau = bad;
    EXPECT_EQ(init(p, good), DecelMpcReason::kParamsInvalid) << "eta_tau " << bad;
    p = DecelMpcParams{};
    p.delta_tr = bad;
    EXPECT_EQ(init(p, good), DecelMpcReason::kParamsInvalid) << "delta_tr " << bad;
  }
  p = DecelMpcParams{};
  p.jerk_weight = Eigen::VectorXd::Ones(5);
  EXPECT_EQ(init(p, good), DecelMpcReason::kParamsInvalid);
  // A rest tolerance at or below eps_abs would reject every shifted reference.
  p = DecelMpcParams{};
  p.reference_rest_tol = p.solver.eps_abs;
  EXPECT_EQ(init(p, good), DecelMpcReason::kParamsInvalid);
  p.reference_rest_tol = 2.0 * p.solver.eps_abs;
  EXPECT_EQ(init(p, good), DecelMpcReason::kNone);

  p = DecelMpcParams{};
  DecelMpcLimits l = good;
  l.tau_max[2] = kNan;
  EXPECT_EQ(init(p, l), DecelMpcReason::kLimitsInvalid);
  l = good;
  l.q_min[1] = l.q_max[1] + 0.1;
  EXPECT_EQ(init(p, l), DecelMpcReason::kLimitsInvalid);
  l = good;
  l.qd_max[0] = 0.0;
  EXPECT_EQ(init(p, l), DecelMpcReason::kLimitsInvalid);
  l = good;
  l.armature.resize(3);
  EXPECT_EQ(init(p, l), DecelMpcReason::kLimitsInvalid);

  DecelMpc mpc;
  EXPECT_EQ(mpc.Init(*arm.model, static_cast<pinocchio::FrameIndex>(arm.model->nframes), p, good),
            DecelMpcReason::kFrameUnknown);
}

TEST(DecelMpc, LockedJointIsAllowed) {
  // q_min == q_max is a legitimate locked joint (NUM-7 availability case):
  // Init accepts it and the joint stays put.
  const ArmModel arm = Synthetic6R();
  DecelMpcLimits l = LimitsFromModel(*arm.model);
  l.q_min[4] = arm.q_nominal[4];
  l.q_max[4] = arm.q_nominal[4];
  DecelMpcParams p;
  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, l), DecelMpcReason::kNone);
  DecelMpcResult res;
  mpc.ResizeResult(res);
  DecelMpcInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.3, -0.2, 0.25, 0.1, 0.0, 0.3;
  ASSERT_TRUE(mpc.Solve(in, res)) << DecelMpcReasonName(res.reason);
  EXPECT_LE((res.q.row(4).array() - arm.q_nominal[4]).abs().maxCoeff(), 1e-5);
}

// ── 8. Timing distribution (informational; F03 owns the budget) ─────────────

struct Samples {
  std::vector<double> presolve, linearize, condense, solve, total, iters, pre_iters;
  int failures{0};
  std::string failure_log;

  void Fail(const DecelMpcResult& r) {
    ++failures;
    if (failures <= 5) {
      failure_log += std::string(" ") + DecelMpcReasonName(r.reason) + "/status" +
                     std::to_string(r.qp_status) + "/it" + std::to_string(r.iterations);
    }
  }

  void Add(const DecelMpcResult& r) {
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

void RunTiming(const ArmModel& arm, const DecelMpcParams& p, const DecelMpcLimits& lim,
               const std::string& key, int n_samples = 200, double* max_dev = nullptr,
               const DecelMpcParams* dev_ref_params = nullptr, bool require_no_failures = true) {
  if (!OptimisedBuild()) {
    n_samples = std::min(n_samples, 10);
    require_no_failures = false;
  }
  const int n = arm.model->nv;
  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), DecelMpcReason::kNone);
  DecelMpc ref;
  if (dev_ref_params != nullptr) {
    ASSERT_EQ(ref.Init(*arm.model, arm.frame, *dev_ref_params, lim), DecelMpcReason::kNone);
  }
  DecelMpcResult res, rref;
  mpc.ResizeResult(res);
  ref.ResizeResult(rref);
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  Samples cold, warm;
  double dev = 0.0;
  pinocchio::Data fk_data(*arm.model);
  for (int i = 0; i < n_samples; ++i) {
    DecelMpcInput in = RestInput(arm.q_nominal);
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
    DecelMpcInput next = in;
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

TEST(DecelMpcTiming, TimingDistribution6R) {
  const ArmModel arm = RealArm6();
  DecelMpcLimits lim = LimitsFromModel(*arm.model, 0.1);
  DecelMpcParams p;
  RunTiming(arm, p, lim, "real_6dof");
  DecelMpcParams dense = p;
  dense.reference_assembly = true;
  RunTiming(arm, dense, lim, "real_6dof.dense_assembly");
  // Torque rows binding (the fixture's tight first joint).
  const TorqueFixture f = MakeTorqueFixture(0.1);
  // Informational: with a binding torque row ProxQP still returns a few
  // false PRIMAL_INFEASIBLE verdicts on warm cycles (~3 % at ρ_τ = 10); the
  // count is recorded, and the caller's fallback (MD-11) covers them.
  RunTiming(f.arm, f.params, f.limits, "real_6dof.torque_active", 200, nullptr, nullptr, false);
}

TEST(DecelMpcTiming, TimingDistribution7R) {
  const ArmModel arm = RealArm7();
  DecelMpcLimits lim = LimitsFromModel(*arm.model, 0.2);
  DecelMpcParams p;
  RunTiming(arm, p, lim, "real_7dof");
  DecelMpcParams dense = p;
  dense.reference_assembly = true;
  RunTiming(arm, dense, lim, "real_7dof.dense_assembly");
  DecelMpcParams perp = p;
  perp.w_perp = 10.0;
  RunTiming(arm, perp, lim, "real_7dof.w_perp_on");
}

// O-4: solver tolerance sweep — warm solve time and iterations against the
// trajectory deviation from a tight-tolerance solve of the same inputs.
TEST(DecelMpcTiming, SolverToleranceSweep7R) {
  const ArmModel arm = RealArm7();
  const DecelMpcLimits lim = LimitsFromModel(*arm.model, 0.2);
  DecelMpcParams tight;
  tight.solver.eps_abs = 1e-9;
  tight.solver.max_iter = 1000;
  for (const double eps : {1e-4, 1e-5, 1e-6, 1e-7}) {
    DecelMpcParams p;
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
TEST(DecelMpc, VelocityDriftIsAbsorbedByJerk) {
  const ArmModel arm = RealArm7();
  DecelMpcParams p;
  const DecelMpcLimits lim = LimitsFromModel(*arm.model, 0.2);
  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), DecelMpcReason::kNone);
  DecelMpcResult res;
  mpc.ResizeResult(res);
  DecelMpcInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.6, -0.4, 0.5, 0.7, -0.3, 0.4, 0.2;
  ASSERT_TRUE(mpc.Solve(in, res));
  for (const double dv : {0.5, 1.0, 2.5}) {
    DecelMpcInput drift = in;
    UseAsReference(res, drift);
    drift.qd0[3] = std::min(drift.qd0[3] + dv, p.eta_v * lim.qd_max[3]);
    DecelMpcResult r;
    mpc.ResizeResult(r);
    EXPECT_TRUE(mpc.Solve(drift, r)) << "dv=" << dv << " " << DecelMpcReasonName(r.reason);
    EXPECT_LE(r.iterations, 20) << "dv=" << dv;
    EXPECT_LE(std::abs(r.q(3, 1) - drift.q_ref(3, 1)), p.delta_tr + 1e-6) << "dv=" << dv;
  }
}

// DecelMpcLimits::armature is ADDED to the armature the model carries — the
// same torque rows come out of (model 0.1 + limits 0.1) and (model 0 +
// limits 0.2).
TEST(DecelMpc, LimitsArmatureAddsToModelArmature) {
  const TorqueFixture f = MakeTorqueFixture();
  const int n = f.arm.model->nv;
  pinocchio::Model carrying = *f.arm.model;
  carrying.armature = Eigen::VectorXd::Constant(n, 0.1);
  DecelMpcLimits half = f.limits;
  half.armature = Eigen::VectorXd::Constant(n, 0.1);
  DecelMpcLimits full = f.limits;
  full.armature = Eigen::VectorXd::Constant(n, 0.2);
  DecelMpcLimits none = f.limits;
  none.armature.setZero();

  const auto ratios = [&](const pinocchio::Model& m, const DecelMpcLimits& l) {
    DecelMpc mpc;
    EXPECT_EQ(mpc.Init(m, f.arm.frame, f.params, l), DecelMpcReason::kNone);
    DecelMpcResult r;
    mpc.ResizeResult(r);
    EXPECT_TRUE(mpc.Solve(f.input, r)) << DecelMpcReasonName(r.reason);
    return Eigen::MatrixXd(r.tau_ratio);
  };
  const Eigen::MatrixXd a = ratios(carrying, half);
  const Eigen::MatrixXd b = ratios(*f.arm.model, full);
  const Eigen::MatrixXd c = ratios(carrying, none);
  EXPECT_LE((a - b).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_GT((a - c).cwiseAbs().maxCoeff(), 1e-3) << "the model's own armature must not be dropped";
}

}  // namespace
