// E3-F02 (#641): the whole-body terms of MpcDockingSegmentCore — the velocity
// weight on both running sums, the stop-node rest toward q_nom and the
// angular-momentum rate with its coefficients held from the start point on
// (D12). Judged:
//   • off is the problem without them, to the bit; Init refuses what it cannot
//     solve (with the catch instant a variable among it);
//   • the solve's convergence contract keeps its meaning with them on: the
//     assembled gradient is the merit's (finite differences, 2e-6), and
//     restarts from its own solution contract to the coefficients' fixed
//     point, where one converges at once (D14);
//   • the reported values are the direct sums — the angular one on the START
//     point's coefficients after a Solve, on its own before any;
//   • the same input gives the same answer whatever was solved before, and no
//     stage outside the QP solver allocates;
//   • per-group move blocking (D11): groups of one partition are the problem
//     without groups to the bit; with several the gradient is the merit's,
//     a restart from its own solution converges at once, every joint's jerk
//     is constant on its group's blocks and the nodes are its integral.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_core.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"
#include "rtc_controllers/testing/synthetic_tree_fixture.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/centroidal.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

using rtc::catching::kWholeBodyWeightMax;
using rtc::catching::MpcDockingReason;
using rtc::catching::MpcDockingReasonName;
using rtc::catching::MpcDockingSegmentCore;
using rtc::catching::MpcDockingSegmentCoreInput;
using rtc::catching::MpcDockingSegmentCoreParams;
using rtc::catching::MpcDockingSegmentCoreResult;
using rtc::catching::MpcDockingStage;
namespace fx = rtc::testing::mpc_segment_core;
namespace dk = rtc::testing::mpc_docking;

std::int64_t NoClock() noexcept {
  return 0;
}

// The whole-body weights on the 7-joint arm, per joint and distinct (an
// index slip changes the answer): velocity on the first two, the stop rest
// on the last three, the angular rate on both running sums.
void SetWholeBody(MpcDockingSegmentCoreParams& p, int n) {
  p.w_vel = Eigen::VectorXd::Zero(n);
  p.w_rest_stop = Eigen::VectorXd::Zero(n);
  p.w_vel[0] = 0.1;
  p.w_vel[1] = 0.15;
  for (int j = n - 3; j < n; ++j) {
    p.w_rest_stop[j] = 1.0 + 0.2 * (j - n + 3);
  }
  p.w_ang_pre = 0.01;
  p.w_ang_post = 1.0;
  p.ang_scale = 10.0;
}

bool SameBits(const Eigen::MatrixXd& x, const Eigen::MatrixXd& y) {
  return x.rows() == y.rows() && x.cols() == y.cols() &&
         std::equal(x.data(), x.data() + x.size(), y.data(),
                    [](double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; });
}

struct Case {
  dk::Throw th;
  MpcDockingSegmentCoreInput in;
};

std::vector<Case> FeasibleCases(const dk::Rig& rig, const MpcDockingSegmentCore& core, int count) {
  std::vector<Case> cases;
  for (unsigned seed = 1; static_cast<int>(cases.size()) < count && seed < 2000; ++seed) {
    Case c;
    if (dk::MakeThrow(rig, core, seed, 1e-3, c.th, c.in)) {
      dk::PerturbTarget(c.in, seed);
      cases.push_back(std::move(c));
    }
  }
  return cases;
}

std::string Describe(const MpcDockingSegmentCoreResult& r) {
  char buf[256];
  std::snprintf(buf, sizeof(buf), "reason=%s feasible=%d converged=%d it=%d J=%.6f wb=%.6f",
                MpcDockingReasonName(r.reason), r.feasible, r.converged, r.iterations, r.cost.total,
                r.cost.whole_body);
  return buf;
}

// ── 1. Off ───────────────────────────────────────────────────────────────────

TEST(MpcDockingWholeBody, OffIsTheProblemWithoutTheTermsToTheBit) {
  const dk::Rig rig = dk::MakeRig(fx::RealArm7());
  const int n = rig.model.nv;
  MpcDockingSegmentCoreParams zero = rig.params;
  zero.w_vel = Eigen::VectorXd::Zero(n);
  zero.w_rest_stop = Eigen::VectorXd::Zero(n);
  zero.ang_scale = 0.0;  // not read while both angular weights are 0
  MpcDockingSegmentCore a, b;
  ASSERT_EQ(a.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  ASSERT_EQ(b.Init(rig.model, rig.arm.frame, zero, rig.limits, &NoClock), MpcDockingReason::kNone);
  EXPECT_FALSE(b.VelocityTermActive());
  EXPECT_FALSE(b.StopRestTermActive());
  EXPECT_FALSE(b.AngularMomentumTermActive());
  const std::vector<Case> cases = FeasibleCases(rig, a, 2);
  ASSERT_EQ(cases.size(), 2U);
  for (const Case& c : cases) {
    MpcDockingSegmentCoreResult ra, rb;
    a.ResizeResult(ra);
    b.ResizeResult(rb);
    ASSERT_TRUE(a.Solve(c.in, ra)) << Describe(ra);
    ASSERT_TRUE(b.Solve(c.in, rb)) << Describe(rb);
    EXPECT_TRUE(SameBits(ra.q, rb.q));
    EXPECT_TRUE(SameBits(ra.u, rb.u));
    EXPECT_EQ(ra.cost.total, rb.cost.total);
    EXPECT_EQ(ra.iterations, rb.iterations);
    EXPECT_EQ(rb.cost.whole_body, 0.0);
    EXPECT_TRUE(SameBits(a.LastQp().H, b.LastQp().H));
    EXPECT_TRUE(SameBits(a.LastQp().g, b.LastQp().g));
  }
}

TEST(MpcDockingWholeBody, InitRefusesWhatItCannotSolve) {
  const dk::Rig rig = dk::MakeRig(fx::RealArm7());
  const int n = rig.model.nv;
  MpcDockingSegmentCoreParams on = rig.params;
  SetWholeBody(on, n);
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, on, rig.limits, &NoClock), MpcDockingReason::kNone);
  EXPECT_TRUE(core.VelocityTermActive());
  EXPECT_TRUE(core.StopRestTermActive());
  EXPECT_TRUE(core.AngularMomentumTermActive());
  const auto refused = [&](MpcDockingSegmentCoreParams p) {
    return core.Init(rig.model, rig.arm.frame, p, rig.limits, &NoClock) ==
           MpcDockingReason::kParamsInvalid;
  };
  MpcDockingSegmentCoreParams p = on;
  p.catch_time_variable = true;  // the search's, which plans without them (D10′)
  EXPECT_TRUE(refused(p));
  p = on;
  p.w_ang_pre = p.w_ang_post = 0.0;
  p.w_rest_stop.setZero();
  p.catch_time_variable = true;
  EXPECT_TRUE(refused(p)) << "the velocity term alone";
  p = on;
  p.q_nom.resize(0);
  p.w_q_nom.resize(0);
  EXPECT_TRUE(refused(p)) << "the stop rest needs q_nom";
  p = on;
  p.ang_scale = 0.0;
  EXPECT_TRUE(refused(p));
  p = on;
  p.ang_scale = 1e-160;
  EXPECT_TRUE(refused(p)) << "W/s² = inf";
  p = on;
  p.ang_scale = 1e160;
  EXPECT_TRUE(refused(p)) << "W/s² = 0";
  p = on;
  p.w_ang_post = std::nextafter(kWholeBodyWeightMax, 1e300);
  EXPECT_TRUE(refused(p));
  p = on;
  p.w_vel[0] = -1e-9;
  EXPECT_TRUE(refused(p));
  p = on;
  p.w_rest_stop[n - 1] = std::nextafter(kWholeBodyWeightMax, 1e300);
  EXPECT_TRUE(refused(p));
  p = on;
  p.w_vel.resize(n - 1);
  EXPECT_TRUE(refused(p));
}

// ── 2. The convergence contract (D12) ────────────────────────────────────────

// Every reference term on as in the core suite's gradient test, and the
// whole-body terms; one real-time iteration from a start taken as given.
dk::Rig GradientRig() {
  dk::Rig rig = dk::MakeRig(fx::RealArm7());
  rig.params.w_manip = 0.02;
  rig.params.manip_d_lin = 0.5;
  rig.params.w_impact = 0.5;
  rig.params.e_ref = 0.05;
  rig.params.e_max = 0.2;
  rig.params.p_max = 0.3;
  rig.params.contact_point_hand = Eigen::Vector3d(0.01, -0.01, 0.02);
  rig.params.w_perp = 30.0;
  rig.params.accel_box = true;
  rig.limits.qdd_max = Eigen::VectorXd::Constant(rig.model.nv, 80.0);
  rig.params.max_iterations = 1;
  rig.params.tol_linear = 1e-2;
  SetWholeBody(rig.params, rig.model.nv);
  return rig;
}

// A start off the optimum, at rest at node N (the push stays in the null space
// of the terminal equality), on the core's own grid.
struct Start {
  Eigen::VectorXd z0;
  MpcDockingSegmentCoreInput in;
  dk::Throw th;
};

Start PushedStart(const dk::Rig& rig, const MpcDockingSegmentCore& core) {
  Start s;
  (void)dk::MakeThrow(rig, core, 3, 0.0, s.th, s.in);
  s.in.p_line = s.th.p_b + Eigen::Vector3d(0.02, -0.03, 0.01);
  s.in.d_line = Eigen::Vector3d(0.3, -0.5, 0.8).normalized();
  s.in.initial_valid = true;
  const Eigen::Index n = core.Nv();
  s.z0 = dk::JerkThrough(core, s.th.q0, s.th.q_c, s.th.v_c);
  const int nb = core.NumBlocks();
  const int N = core.NumNodes();
  Eigen::MatrixXd m(2, nb);
  for (int b = 0; b < nb; ++b) {
    m(0, b) = core.StageGain(1, N, b);
    m(1, b) = core.StageGain(2, N, b);
  }
  const Eigen::MatrixXd null_proj =
      Eigen::MatrixXd::Identity(nb, nb) - m.transpose() * (m * m.transpose()).inverse() * m;
  for (Eigen::Index j = 0; j < n; ++j) {
    Eigen::VectorXd push(nb);
    for (int b = 0; b < nb; ++b) {
      push[b] = 0.02 * std::sin(1.3 * static_cast<double>(b * n + j) + 0.4);
    }
    const Eigen::VectorXd kept = null_proj * push;
    for (int b = 0; b < nb; ++b) {
      s.z0[b * n + j] += kept[b];
    }
  }
  return s;
}

void SetStart(const MpcDockingSegmentCore& core, const dk::Throw& th, const Eigen::VectorXd& z,
              MpcDockingSegmentCoreInput& in) {
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(th.q0.size());
  const dk::Nodes nodes = dk::NodesFromJerk(core, th.q0, zero, zero, z);
  in.q_init = nodes.q;
  in.qd_init = nodes.qd;
  in.qdd_init = nodes.qdd;
}

TEST(MpcDockingWholeBody, TheAssembledGradientIsTheMeritsWithTheTermsOn) {
  const dk::Rig rig = GradientRig();
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  Start s = PushedStart(rig, core);
  SetStart(core, s.th, s.z0, s.in);
  ASSERT_TRUE(core.Solve(s.in, out)) << Describe(out);
  ASSERT_FALSE(out.init_qp_used) << "the start must be taken as given";
  const rtc::tsid::QPData base = core.LastQp();
  EXPECT_EQ((base.H - base.H.transpose()).cwiseAbs().maxCoeff(), 0.0) << "H symmetric";
  const Eigen::Index nu = core.NumJerkVariables();
  // The merit's smooth part, on the coefficients the Solve held (Evaluate
  // reads the last Solve's).
  const auto smooth = [&](const Eigen::VectorXd& z, double* whole_body) {
    SetStart(core, s.th, z, s.in);
    EXPECT_TRUE(core.Evaluate(s.in, out));
    if (whole_body != nullptr) {
      *whole_body = out.cost.whole_body;
    }
    return out.cost.total - out.cost.slack;
  };
  const double h = 1e-6;
  double wb_slope = 0.0;
  for (Eigen::Index i = 0; i < nu; ++i) {
    Eigen::VectorXd zp = s.z0;
    Eigen::VectorXd zm = s.z0;
    zp[i] += h;
    zm[i] -= h;
    double wp = 0.0;
    double wm = 0.0;
    const double fd = (smooth(zp, &wp) - smooth(zm, &wm)) / (2.0 * h);
    EXPECT_NEAR(base.g[i], fd, 2e-6 * std::max(1.0, std::abs(fd))) << "variable " << i;
    wb_slope = std::max(wb_slope, std::abs(wp - wm) / (2.0 * h));
  }
  EXPECT_GT(wb_slope, 1e-3) << "the whole-body terms must carry part of the gradient";
}

// The velocity and stop-rest sums are quadratic in z, so the QP's Hessian
// carries them exactly: on − off at one start is their second difference, on
// every node either sum weights (the catch node's rest among them).
TEST(MpcDockingWholeBody, TheAssembledHessianIsTheQuadraticTermsSecondDifference) {
  dk::Rig off_rig = GradientRig();
  const int n = off_rig.model.nv;
  off_rig.params.w_vel = Eigen::VectorXd::Zero(n);
  off_rig.params.w_rest_stop = Eigen::VectorXd::Zero(n);
  off_rig.params.w_ang_pre = off_rig.params.w_ang_post = 0.0;
  dk::Rig on_rig = GradientRig();
  on_rig.params.w_ang_pre = on_rig.params.w_ang_post = 0.0;
  MpcDockingSegmentCore off, on;
  ASSERT_EQ(off.Init(off_rig.model, off_rig.arm.frame, off_rig.params, off_rig.limits, &NoClock),
            MpcDockingReason::kNone);
  ASSERT_EQ(on.Init(on_rig.model, on_rig.arm.frame, on_rig.params, on_rig.limits, &NoClock),
            MpcDockingReason::kNone);
  ASSERT_TRUE(on.VelocityTermActive() && on.StopRestTermActive());
  ASSERT_FALSE(on.AngularMomentumTermActive());
  MpcDockingSegmentCoreResult out;
  on.ResizeResult(out);
  Start s = PushedStart(on_rig, on);
  SetStart(on, s.th, s.z0, s.in);
  ASSERT_TRUE(on.Solve(s.in, out)) << Describe(out);
  ASSERT_FALSE(out.init_qp_used);
  const Eigen::MatrixXd h_on = on.LastQp().H;
  ASSERT_TRUE(off.Solve(s.in, out)) << Describe(out);
  ASSERT_FALSE(out.init_qp_used);
  const Eigen::MatrixXd dh = h_on - off.LastQp().H;
  const Eigen::Index nu = on.NumJerkVariables();
  EXPECT_EQ(dh.bottomRows(dh.rows() - nu).cwiseAbs().maxCoeff(), 0.0) << "the slacks hold none";
  const auto sums = [&](const Eigen::VectorXd& z) {
    SetStart(on, s.th, z, s.in);
    EXPECT_TRUE(on.Evaluate(s.in, out));
    return out.cost.vel + out.cost.rest_stop;
  };
  const double h = 1e-2;
  const double f0 = sums(s.z0);
  double largest = 0.0;
  for (Eigen::Index i = 0; i < nu; ++i) {
    Eigen::VectorXd zp = s.z0;
    Eigen::VectorXd zm = s.z0;
    zp[i] += h;
    zm[i] -= h;
    const double second = (sums(zp) - 2.0 * f0 + sums(zm)) / (h * h);
    EXPECT_NEAR(dh(i, i), second, 1e-6 * std::max(1.0, std::abs(second))) << "variable " << i;
    largest = std::max(largest, std::abs(second));
  }
  // Off the diagonal, along a few directions mixing every variable.
  for (int r = 0; r < 4; ++r) {
    Eigen::VectorXd d(nu);
    for (Eigen::Index i = 0; i < nu; ++i) {
      d[i] = std::sin(0.7 * static_cast<double>(i) + 1.9 * r + 0.3);
    }
    const double second = (sums(s.z0 + h * d) - 2.0 * f0 + sums(s.z0 - h * d)) / (h * h);
    const double form = d.dot(dh.topLeftCorner(nu, nu) * d);
    EXPECT_NEAR(form, second, 1e-6 * std::max(1.0, std::abs(second))) << "direction " << r;
  }
  EXPECT_GT(largest, 1e-3) << "the terms must carry part of the Hessian";
}

// D14 (#641, 2026-10-17): a Solve takes the angular coefficients at ITS start
// point (D12), so a restart from a solution solves the problem held at that
// solution — another one than the solve that produced it. Restarted on its
// own solution, the core therefore runs a fixed-point iteration of the
// coefficients: each restart moves the solution by a tenth or less of the
// previous move, and at the fixed point a restart converges at once.
TEST(MpcDockingWholeBody, RestartsContractToTheCoefficientsFixedPointWhereOneConvergesAtOnce) {
  dk::Rig rig = dk::MakeRig(fx::RealArm7());
  SetWholeBody(rig.params, rig.model.nv);
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  const std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  MpcDockingSegmentCoreResult prev;
  core.ResizeResult(prev);
  ASSERT_TRUE(core.Solve(cases[0].in, prev));
  ASSERT_TRUE(prev.feasible && prev.converged) << Describe(prev);
  double last_move = std::numeric_limits<double>::infinity();
  bool at_fixed_point = false;
  for (int r = 0; r < 6 && !at_fixed_point; ++r) {
    SCOPED_TRACE("restart " + std::to_string(r));
    MpcDockingSegmentCoreInput again = cases[0].in;
    again.initial_valid = true;
    again.q_init = prev.q;
    again.qd_init = prev.qd;
    again.qdd_init = prev.qdd;
    MpcDockingSegmentCoreResult next;
    core.ResizeResult(next);
    ASSERT_TRUE(core.Solve(again, next));
    EXPECT_FALSE(next.init_qp_used);
    EXPECT_TRUE(next.feasible && next.converged) << Describe(next);
    const double move = (next.q - prev.q).cwiseAbs().maxCoeff();
    if (move < 1e-9) {
      EXPECT_EQ(next.iterations, 1) << "at the fixed point a restart converges at once";
      at_fixed_point = true;
    } else {
      EXPECT_LE(move, 0.1 * last_move) << "the restarts must contract";
    }
    last_move = move;
    prev = next;
  }
  EXPECT_TRUE(at_fixed_point) << "no fixed point within six restarts";
}

TEST(MpcDockingWholeBody, SameInputGivesTheSameAnswerWhateverWasSolvedBefore) {
  dk::Rig rig = dk::MakeRig(fx::RealArm7());
  SetWholeBody(rig.params, rig.model.nv);
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult a, b, other;
  core.ResizeResult(a);
  core.ResizeResult(b);
  core.ResizeResult(other);
  const std::vector<Case> cases = FeasibleCases(rig, core, 2);
  ASSERT_EQ(cases.size(), 2U);
  ASSERT_TRUE(core.Solve(cases[0].in, a));
  ASSERT_TRUE(core.Solve(cases[1].in, other));
  ASSERT_TRUE(core.Solve(cases[0].in, b));
  EXPECT_TRUE(SameBits(a.q, b.q));
  EXPECT_TRUE(SameBits(a.u, b.u));
  EXPECT_EQ(a.iterations, b.iterations);
  EXPECT_EQ(a.cost.total, b.cost.total);
  EXPECT_EQ(a.cost.ang, b.cost.ang);
  EXPECT_GT((a.q - other.q).cwiseAbs().maxCoeff(), 1e-3) << "the two cases must differ";
}

// ── 3. The reported values ───────────────────────────────────────────────────

struct Angular {
  Eigen::Matrix<double, 3, Eigen::Dynamic> a;
  Eigen::Vector3d d;
};

// A^ω_G and Ȧ^ω_G q̇ at (q, q̇) by pinocchio's centroidal map and a central
// difference of it along q̇ — not the core's dccrba.
Angular AngularAt(const pinocchio::Model& model, const Eigen::VectorXd& q,
                  const Eigen::VectorXd& v) {
  pinocchio::Data data(model);
  Angular o;
  o.a = pinocchio::computeCentroidalMap(model, data, q).bottomRows<3>();
  const double eps = 1e-6;
  const Eigen::MatrixXd ap =
      pinocchio::computeCentroidalMap(model, data, q + eps * v).bottomRows<3>();
  const Eigen::MatrixXd am =
      pinocchio::computeCentroidalMap(model, data, q - eps * v).bottomRows<3>();
  o.d = (ap - am) / (2.0 * eps) * v;
  return o;
}

struct Sums {
  double vel{0.0}, rest_stop{0.0}, ang{0.0}, rate_max{0.0};
};

// The reference's convention: no ½, each running sum times its interval
// length — stages 0..k_c−1 at Δ_a, nodes k_c..N at Δ_s; the angular one on
// the coefficients at (q_c, q̇_c) and the accelerations of (q, q̇, q̈).
Sums DirectSums(const dk::Rig& rig, const MpcDockingSegmentCoreParams& p, int kc,
                const Eigen::MatrixXd& q, const Eigen::MatrixXd& qd, const Eigen::MatrixXd& qdd,
                const Eigen::MatrixXd& q_c, const Eigen::MatrixXd& qd_c) {
  Sums s;
  const int N = static_cast<int>(q.cols()) - 1;
  for (int k = 0; k <= N; ++k) {
    const bool pre = k < kc;
    const double dt = pre ? p.dt_pre : p.dt_stop;
    for (int j = 0; j < q.rows(); ++j) {
      s.vel += dt * p.w_vel[j] * qd(j, k) * qd(j, k);
      if (!pre) {
        const double dq = q(j, k) - p.q_nom[j];
        s.rest_stop += p.dt_stop * p.w_rest_stop[j] * dq * dq;
      }
    }
    const double w = (pre ? p.w_ang_pre : p.w_ang_post) * dt / (p.ang_scale * p.ang_scale);
    if (w > 0.0) {
      const Angular o = AngularAt(rig.model, q_c.col(k), qd_c.col(k));
      const Eigen::Vector3d kd = o.a * qdd.col(k) + o.d;
      s.ang += w * kd.squaredNorm();
      s.rate_max = std::max(s.rate_max, kd.norm());
    }
  }
  return s;
}

TEST(MpcDockingWholeBody, TheValuesAreTheDirectSumsOnTheStartPointsCoefficients) {
  dk::Rig rig = dk::MakeRig(fx::RealArm7());
  SetWholeBody(rig.params, rig.model.nv);
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  Start s = PushedStart(rig, core);
  SetStart(core, s.th, s.z0, s.in);
  const Eigen::MatrixXd q_start = s.in.q_init;
  const Eigen::MatrixXd qd_start = s.in.qd_init;
  ASSERT_TRUE(core.Solve(s.in, out)) << Describe(out);
  ASSERT_FALSE(out.init_qp_used);
  ASSERT_GE(out.iterations, 2) << "the solution must have moved off the start";
  const int kc = core.CatchNode();
  const auto near = [](double x, double y, double rel) {
    return std::abs(x - y) <= rel * std::max(1e-12, std::abs(y));
  };
  // After a Solve: the angular term on the START point's coefficients (D12).
  const Sums held = DirectSums(rig, rig.params, kc, out.q, out.qd, out.qdd, q_start, qd_start);
  EXPECT_TRUE(near(out.cost.vel, held.vel, 1e-9)) << out.cost.vel << " vs " << held.vel;
  EXPECT_TRUE(near(out.cost.rest_stop, held.rest_stop, 1e-9));
  EXPECT_TRUE(near(out.cost.ang, held.ang, 1e-6)) << out.cost.ang << " vs " << held.ang;
  EXPECT_TRUE(near(out.cost.ang_rate_max, held.rate_max, 1e-6));
  EXPECT_TRUE(near(out.cost.whole_body, out.cost.vel + out.cost.rest_stop + out.cost.ang, 1e-12));
  EXPECT_TRUE(
      near(out.cost.total, out.cost.reference + out.cost.stop + out.cost.whole_body, 1e-12));
  // A core that has not solved evaluates on the trajectory's own — another
  // number here: the hold is real.
  MpcDockingSegmentCore fresh;
  ASSERT_EQ(fresh.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreInput at = s.in;
  at.q_init = out.q;
  at.qd_init = out.qd;
  at.qdd_init = out.qdd;
  MpcDockingSegmentCoreResult ev;
  fresh.ResizeResult(ev);
  ASSERT_TRUE(fresh.Evaluate(at, ev));
  const Sums own = DirectSums(rig, rig.params, kc, out.q, out.qd, out.qdd, out.q, out.qd);
  EXPECT_TRUE(near(ev.cost.ang, own.ang, 1e-6)) << ev.cost.ang << " vs " << own.ang;
  EXPECT_GT(std::abs(own.ang - held.ang), 1e-6 * held.ang) << "the fixture must tell them apart";
}

TEST(MpcDockingWholeBody, TheRestHoldsItsJointsAndTheVelocityWeightSlowsItsJoints) {
  dk::Rig rig = dk::MakeRig(fx::RealArm7());
  const int n = rig.model.nv;
  MpcDockingSegmentCore probe;
  ASSERT_EQ(probe.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  const std::vector<Case> cases = FeasibleCases(rig, probe, 1);
  ASSERT_EQ(cases.size(), 1U);
  const auto solve = [&](double w_vel, double w_rest) {
    MpcDockingSegmentCoreParams p = rig.params;
    p.w_vel = Eigen::VectorXd::Zero(n);
    p.w_rest_stop = Eigen::VectorXd::Zero(n);
    p.w_vel.head(2).setConstant(w_vel);
    p.w_rest_stop.tail(3).setConstant(w_rest);
    MpcDockingSegmentCore core;
    EXPECT_EQ(core.Init(rig.model, rig.arm.frame, p, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult r;
    core.ResizeResult(r);
    EXPECT_TRUE(core.Solve(cases[0].in, r));
    EXPECT_TRUE(r.feasible && r.converged) << Describe(r);
    return r;
  };
  const MpcDockingSegmentCoreResult off = solve(0.0, 0.0);
  const MpcDockingSegmentCoreResult held = solve(0.0, 100.0);
  const MpcDockingSegmentCoreResult damped = solve(1.0, 0.0);
  const int kc = probe.CatchNode();
  const auto stop_off_rest = [&](const MpcDockingSegmentCoreResult& r) {
    double s = 0.0;
    for (int k = kc; k < r.q.cols(); ++k) {
      s += (r.q.col(k).tail(3) - rig.params.q_nom.tail(3)).squaredNorm();
    }
    return s;
  };
  EXPECT_LT(stop_off_rest(held), 0.5 * stop_off_rest(off));
  const auto speed = [](const MpcDockingSegmentCoreResult& r) {
    return r.qd.topRows(2).squaredNorm();
  };
  EXPECT_LT(speed(damped), speed(off));
}

// ── 4. Allocation ────────────────────────────────────────────────────────────

struct StageRecord {
  std::array<std::size_t, 6> mallocs{};
  std::array<int, 6> entries{};
};

void CountingHook(MpcDockingStage stage, bool begin, void* user) noexcept {
  auto* rec = static_cast<StageRecord*>(user);
  const auto s = static_cast<std::size_t>(stage);
  if (begin) {
    rtc::testing::detail::MallocGateCount() = 0;
    ++rtc::testing::detail::MallocGateDepth();
    ++rec->entries[s];
  } else {
    --rtc::testing::detail::MallocGateDepth();
    rec->mallocs[s] += rtc::testing::detail::MallocGateCount();
  }
}

TEST(MpcDockingWholeBody, NoStageOutsideTheSolverAllocatesWithTheTermsOn) {
  dk::Rig rig = dk::MakeRig(fx::RealArm7());
  SetWholeBody(rig.params, rig.model.nv);
  for (const bool fresh_evaluate : {false, true}) {
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult out;
    core.ResizeResult(out);
    const std::vector<Case> cases = FeasibleCases(rig, core, 2);
    ASSERT_EQ(cases.size(), 2U);
    StageRecord rec;
    core.SetStageHook(&CountingHook, &rec);
    std::size_t news = 0;
    for (const Case& c : cases) {
      if (fresh_evaluate) {
        // The first call an Evaluate: it takes the coefficients itself.
        MpcDockingSegmentCoreInput at = c.in;
        at.initial_valid = true;
        at.q_init = c.th.known.q;
        at.qd_init = c.th.known.qd;
        at.qdd_init = c.th.known.qdd;
        const rtc::testing::ScopedAllocGate new_gate;
        ASSERT_TRUE(core.Evaluate(at, out));
        news += new_gate.count();
      }
      const rtc::testing::ScopedAllocGate new_gate;
      ASSERT_TRUE(core.Solve(c.in, out)) << Describe(out);
      news += new_gate.count();
    }
    core.SetStageHook(nullptr, nullptr);
    EXPECT_EQ(news, 0U) << "operator new inside Solve / Evaluate";
    for (std::size_t s = 0; s < rec.mallocs.size(); ++s) {
      EXPECT_EQ(rec.mallocs[s], 0U) << "stage " << s;
    }
    EXPECT_GT(rec.entries[static_cast<std::size_t>(MpcDockingStage::kLinearize)], 2)
        << "the hold and the iterations ran";
  }
}

// ── 5. Per-group move blocking ───────────────────────────────────────────────

using rtc::catching::BlockPattern;

BlockPattern Pattern(std::initializer_list<int> sizes) {
  BlockPattern p;
  for (const int v : sizes) {
    p.block_sizes[static_cast<std::size_t>(p.n_blocks++)] = v;
  }
  return p;
}

// The 7-joint arm in three groups (0–1, 2–4, 5–6). BaseParams' grid is 6 ×
// 0.05 s before the catch and 8 after, {1 × 6 | 1, 1, 2, 2, 2}.
const BlockPattern kBase = Pattern({1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2});
const BlockPattern kPreCoarse = Pattern({2, 2, 2, 1, 1, 2, 2, 2});
const BlockPattern kBothCoarse = Pattern({3, 3, 2, 3, 3});

void SetGroups(MpcDockingSegmentCoreParams& p, const std::array<BlockPattern, 3>& patterns) {
  p.n_groups = 3;
  const std::array<int, 7> group = {0, 0, 1, 1, 1, 2, 2};
  for (std::size_t j = 0; j < 7; ++j) {
    p.joint_group[j] = static_cast<std::int8_t>(group[j]);
  }
  for (std::size_t g = 0; g < 3; ++g) {
    p.group_blocks[g] = patterns[g];
  }
}

TEST(MpcDockingGroups, GroupsOfOnePartitionAreTheProblemWithoutGroupsToTheBit) {
  dk::Rig rig = dk::MakeRig(fx::RealArm7());
  SetWholeBody(rig.params, rig.model.nv);
  rig.params.w_ang_pre = rig.params.w_ang_post = 0.0;  // D14 aside: every other term
  MpcDockingSegmentCoreParams grouped = rig.params;
  SetGroups(grouped, {kBase, kBase, kBase});
  MpcDockingSegmentCore a, b;
  ASSERT_EQ(a.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  ASSERT_EQ(b.Init(rig.model, rig.arm.frame, grouped, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  EXPECT_TRUE(b.Layout().Single());
  const std::vector<Case> cases = FeasibleCases(rig, a, 2);
  ASSERT_EQ(cases.size(), 2U);
  for (const Case& c : cases) {
    MpcDockingSegmentCoreResult ra, rb;
    a.ResizeResult(ra);
    b.ResizeResult(rb);
    ASSERT_TRUE(a.Solve(c.in, ra));
    ASSERT_TRUE(b.Solve(c.in, rb));
    EXPECT_TRUE(SameBits(ra.q, rb.q));
    EXPECT_TRUE(SameBits(ra.u, rb.u));
    EXPECT_EQ(ra.cost.total, rb.cost.total);
    EXPECT_EQ(ra.iterations, rb.iterations);
    EXPECT_TRUE(SameBits(a.LastQp().H, b.LastQp().H));
    EXPECT_TRUE(SameBits(a.LastQp().g, b.LastQp().g));
    EXPECT_TRUE(SameBits(a.LastQp().C, b.LastQp().C));
  }
}

TEST(MpcDockingGroups, InitChecksEveryGroupsPartitionAndRefusesThemWithTheCatchInstantFree) {
  const dk::Rig rig = dk::MakeRig(fx::RealArm7());
  MpcDockingSegmentCore core;

  struct Case2 {
    BlockPattern pattern;
    MpcDockingReason why;
  };

  for (const Case2& c : {
           Case2{Pattern({1, 1, 1, 1, 3, 1, 2, 2, 2}), MpcDockingReason::kBlocksAcrossCatch},
           Case2{Pattern({3, 3, 4, 4}), MpcDockingReason::kBlocksTooFew},
           Case2{Pattern({3, 3, 2, 2, 2}), MpcDockingReason::kParamsInvalid},  // Σ ≠ N
       }) {
    MpcDockingSegmentCoreParams p = rig.params;
    SetGroups(p, {kBase, kPreCoarse, c.pattern});
    EXPECT_EQ(core.Init(rig.model, rig.arm.frame, p, rig.limits, &NoClock), c.why)
        << MpcDockingReasonName(c.why);
  }
  MpcDockingSegmentCoreParams p = rig.params;
  SetGroups(p, {kBase, kPreCoarse, kBothCoarse});
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, p, rig.limits, &NoClock), MpcDockingReason::kNone);
  EXPECT_EQ(core.Layout().n_classes, 3);
  EXPECT_EQ(core.NumJerkVariables(),
            2 * kBase.n_blocks + 3 * kPreCoarse.n_blocks + 2 * kBothCoarse.n_blocks);
  p.catch_time_variable = true;
  EXPECT_EQ(core.Init(rig.model, rig.arm.frame, p, rig.limits, &NoClock),
            MpcDockingReason::kParamsInvalid);
  // Groups of one partition stay available with it: they are no groups.
  SetGroups(p, {kBase, kBase, kBase});
  EXPECT_EQ(core.Init(rig.model, rig.arm.frame, p, rig.limits, &NoClock), MpcDockingReason::kNone);
}

// PushedStart on any layout: each joint's push in the null space of its own
// class's terminal rows.
Start PushedStartOf(const dk::Rig& rig, const MpcDockingSegmentCore& core) {
  Start s;
  (void)dk::MakeThrow(rig, core, 3, 0.0, s.th, s.in);
  s.in.p_line = s.th.p_b + Eigen::Vector3d(0.02, -0.03, 0.01);
  s.in.d_line = Eigen::Vector3d(0.3, -0.5, 0.8).normalized();
  s.in.initial_valid = true;
  s.z0 = dk::JerkThrough(core, s.th.q0, s.th.q_c, s.th.v_c);
  const rtc::catching::BlockLayout& l = core.Layout();
  const int N = core.NumNodes();
  for (int j = 0; j < core.Nv(); ++j) {
    const int c = l.class_of[static_cast<std::size_t>(j)];
    const int nb = l.pattern[static_cast<std::size_t>(c)].n_blocks;
    Eigen::MatrixXd m(2, nb);
    for (int b = 0; b < nb; ++b) {
      m(0, b) = core.StageGain(1, N, b, c);
      m(1, b) = core.StageGain(2, N, b, c);
    }
    const Eigen::MatrixXd null_proj =
        Eigen::MatrixXd::Identity(nb, nb) - m.transpose() * (m * m.transpose()).inverse() * m;
    Eigen::VectorXd push(nb);
    for (int b = 0; b < nb; ++b) {
      push[b] = 0.02 * std::sin(1.3 * static_cast<double>(b * 7 + j) + 0.4);
    }
    const Eigen::VectorXd kept = null_proj * push;
    for (int b = 0; b < nb; ++b) {
      s.z0[l.Col(j, b)] += kept[b];
    }
  }
  return s;
}

TEST(MpcDockingGroups, WithThreePartitionsTheAssembledGradientIsTheMerits) {
  dk::Rig rig = GradientRig();
  SetGroups(rig.params, {kBase, kPreCoarse, kBothCoarse});
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  ASSERT_EQ(core.Layout().n_classes, 3);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  Start s = PushedStartOf(rig, core);
  SetStart(core, s.th, s.z0, s.in);
  ASSERT_TRUE(core.Solve(s.in, out)) << Describe(out);
  ASSERT_FALSE(out.init_qp_used) << "the start must be taken as given";
  const rtc::tsid::QPData base = core.LastQp();
  EXPECT_EQ((base.H - base.H.transpose()).cwiseAbs().maxCoeff(), 0.0) << "H exactly symmetric";
  const auto smooth = [&](const Eigen::VectorXd& z) {
    SetStart(core, s.th, z, s.in);
    EXPECT_TRUE(core.Evaluate(s.in, out));
    return out.cost.total - out.cost.slack;
  };
  const double h = 1e-6;
  for (Eigen::Index i = 0; i < core.NumJerkVariables(); ++i) {
    Eigen::VectorXd zp = s.z0;
    Eigen::VectorXd zm = s.z0;
    zp[i] += h;
    zm[i] -= h;
    const double fd = (smooth(zp) - smooth(zm)) / (2.0 * h);
    EXPECT_NEAR(base.g[i], fd, 2e-6 * std::max(1.0, std::abs(fd))) << "variable " << i;
  }
}

TEST(MpcDockingGroups, WithThreePartitionsARestartConvergesAtOnceAndTheJerkKeepsEachGroupsBlocks) {
  dk::Rig rig = dk::MakeRig(fx::RealArm7());
  SetGroups(rig.params, {kBase, kPreCoarse, kBothCoarse});
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult first, second;
  core.ResizeResult(first);
  core.ResizeResult(second);
  const std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  ASSERT_TRUE(core.Solve(cases[0].in, first));
  ASSERT_TRUE(first.feasible && first.converged) << Describe(first);
  // The start-point projection reproduces its own solution exactly.
  MpcDockingSegmentCoreInput again = cases[0].in;
  again.initial_valid = true;
  again.q_init = first.q;
  again.qd_init = first.qd;
  again.qdd_init = first.qdd;
  ASSERT_TRUE(core.Solve(again, second));
  EXPECT_FALSE(second.init_qp_used);
  EXPECT_TRUE(second.converged) << Describe(second);
  EXPECT_EQ(second.iterations, 1);
  EXPECT_LT((second.q - first.q).cwiseAbs().maxCoeff(), 1e-9);
  // Each joint's jerk is constant on its own group's blocks, and the nodes
  // are its integral (what the payload carries and the RT samples).
  const std::array<BlockPattern, 3> pat = {kBase, kPreCoarse, kBothCoarse};
  const std::array<int, 7> group = {0, 0, 1, 1, 1, 2, 2};
  const int N = core.NumNodes();
  for (int j = 0; j < 7; ++j) {
    const BlockPattern& p = pat[static_cast<std::size_t>(group[static_cast<std::size_t>(j)])];
    int k = 0;
    for (int b = 0; b < p.n_blocks; ++b) {
      for (int i = 1; i < p.block_sizes[static_cast<std::size_t>(b)]; ++i) {
        EXPECT_EQ(first.u(j, k + i), first.u(j, k)) << "joint " << j << " block " << b;
      }
      k += p.block_sizes[static_cast<std::size_t>(b)];
    }
  }
  for (int k = 0; k < N; ++k) {
    const double dt = core.NodeTime(k + 1) - core.NodeTime(k);
    const Eigen::VectorXd u = first.u.col(k);
    const Eigen::VectorXd q = first.q.col(k) + dt * first.qd.col(k) +
                              0.5 * dt * dt * first.qdd.col(k) + dt * dt * dt / 6.0 * u;
    EXPECT_LE((q - first.q.col(k + 1)).cwiseAbs().maxCoeff(), 1e-9) << "node " << k + 1;
    const Eigen::VectorXd a = first.qdd.col(k) + dt * u;
    EXPECT_LE((a - first.qdd.col(k + 1)).cwiseAbs().maxCoeff(), 1e-9) << "node " << k + 1;
  }
}

TEST(MpcDockingGroups, NoStageAllocatesWithThreePartitionsAndTheTermsOn) {
  dk::Rig rig = dk::MakeRig(fx::RealArm7());
  SetWholeBody(rig.params, rig.model.nv);
  SetGroups(rig.params, {kBase, kPreCoarse, kBothCoarse});
  rig.params.jerk_box = true;
  rig.limits.jerk_max = Eigen::VectorXd::Constant(rig.model.nv, 5e3);
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  const std::vector<Case> cases = FeasibleCases(rig, core, 2);
  ASSERT_EQ(cases.size(), 2U);
  StageRecord rec;
  core.SetStageHook(&CountingHook, &rec);
  std::size_t news = 0;
  for (const Case& c : cases) {
    const rtc::testing::ScopedAllocGate new_gate;
    ASSERT_TRUE(core.Solve(c.in, out)) << Describe(out);
    news += new_gate.count();
  }
  core.SetStageHook(nullptr, nullptr);
  EXPECT_EQ(news, 0U);
  for (std::size_t st = 0; st < rec.mallocs.size(); ++st) {
    EXPECT_EQ(rec.mallocs[st], 0U) << "stage " << st;
  }
}

// ── 6. Judgement: counter-swing on the tree (#641 D2, D15) ──────────────────
// A trunk and two arms; the second arm (model joints 10..16) is the group a
// plan leaves out, at rest on its rest posture q_nom at x_0. Values: W⁻ / W⁺ =
// 0.001 / 0.1, W_big = 100·W⁺, the stop rest 1 on that arm. No torque cost:
// through the trunk's reaction it moves the other arm too (E3-F01's
// observation, #641 D9′), and the control below must see the angular term
// alone.

constexpr int kTreeOther = 10;

struct TreeJudge {
  fx::ArmModel tree = rtc::testing::SyntheticTree(3, 7, 2);
  dk::Rig rig = dk::MakeRig(tree);
  MpcDockingSegmentCoreInput in;
  int kc{0};
};

std::unique_ptr<TreeJudge> MakeTreeJudge() {
  auto t = std::make_unique<TreeJudge>();
  MpcDockingSegmentCore probe;
  EXPECT_EQ(probe.Init(t->rig.model, t->rig.arm.frame, t->rig.params, t->rig.limits, &NoClock),
            MpcDockingReason::kNone);
  t->kc = probe.CatchNode();
  for (unsigned seed = 1; seed < 2000; ++seed) {
    Case c;
    if (dk::MakeThrow(t->rig, probe, seed, 1e-3, c.th, c.in)) {
      t->in = c.in;
      break;
    }
  }
  for (int m = kTreeOther; m < t->rig.model.nv; ++m) {
    t->in.q0[m] = t->tree.q_nominal[m];
    t->in.q_catch_target[m] = t->tree.q_nominal[m];
  }
  return t;
}

MpcDockingSegmentCoreResult SolveTree(const TreeJudge& t, double pre, double post, bool locked) {
  const int n = t.rig.model.nv;
  MpcDockingSegmentCoreParams p = t.rig.params;
  p.r_tau.resize(0);
  p.q_nom = t.tree.q_nominal;
  p.w_rest_stop = Eigen::VectorXd::Zero(n);
  p.w_rest_stop.tail(n - kTreeOther).setConstant(1.0);
  p.w_ang_pre = pre;
  p.w_ang_post = post;
  p.ang_scale = 10.0;
  auto lim = t.rig.limits;
  if (locked) {
    for (int m = kTreeOther; m < n; ++m) {
      lim.q_min[m] = lim.q_max[m] = t.tree.q_nominal[m];
    }
  }
  MpcDockingSegmentCore core;
  EXPECT_EQ(core.Init(t.rig.model, t.rig.arm.frame, p, lim, &NoClock), MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult r;
  core.ResizeResult(r);
  EXPECT_TRUE(core.Solve(t.in, r));
  return r;
}

double TreeOtherOffRest(const TreeJudge& t, const MpcDockingSegmentCoreResult& r) {
  return (r.q.bottomRows(t.rig.model.nv - kTreeOther).colwise() -
          t.tree.q_nominal.tail(t.rig.model.nv - kTreeOther))
      .cwiseAbs()
      .maxCoeff();
}

// D2: the catch node's frame position, approach axis and velocity against
// the solve without the angular term (the test's own FK).
bool TreeWithinD2(const TreeJudge& t, const MpcDockingSegmentCoreResult& a,
                  const MpcDockingSegmentCoreResult& b, std::string* why) {
  const dk::HandState ha = dk::HandAt(t.rig, a.q.col(t.kc), a.qd.col(t.kc));
  const dk::HandState hb = dk::HandAt(t.rig, b.q.col(t.kc), b.qd.col(t.kc));
  const double dp = (ha.p - hb.p).norm();
  const double da = std::acos(std::clamp(ha.R.col(2).dot(hb.R.col(2)), -1.0, 1.0));
  const double dv = (ha.R * ha.v_local - hb.R * hb.v_local).norm();
  *why = "position " + std::to_string(dp) + " m, axis " + std::to_string(da) + " rad, velocity " +
         std::to_string(dv) + " m/s";
  return dp <= 1e-3 && da <= 0.0175 && dv <= 0.02;
}

TEST(MpcDockingJudgement, TheOtherArmSwingsAgainstTheReachAndTheCatchHardlyNotices) {
  const auto t = MakeTreeJudge();
  ASSERT_EQ(t->in.q0.size(), t->rig.model.nv);
  ASSERT_TRUE(t->in.qd0.size() == 0 || t->in.qd0.isZero(0.0)) << "the premise: at rest";
  const MpcDockingSegmentCoreResult off = SolveTree(*t, 0.0, 0.0, false);
  const MpcDockingSegmentCoreResult swing = SolveTree(*t, 0.001, 0.1, false);
  const MpcDockingSegmentCoreResult locked = SolveTree(*t, 0.001, 0.1, true);
  const MpcDockingSegmentCoreResult locked_big = SolveTree(*t, 10.0, 10.0, true);
  for (const MpcDockingSegmentCoreResult* r : {&off, &swing, &locked, &locked_big}) {
    EXPECT_TRUE(r->feasible && r->converged) << Describe(*r);
  }
  std::string why;
  // The control: without the angular term nothing moves the other arm.
  EXPECT_LE(TreeOtherOffRest(*t, off), 1e-6);
  // Positive: the free arm moves and lowers the term the core minimises …
  EXPECT_GT(TreeOtherOffRest(*t, swing), 1e-3);
  EXPECT_LT(swing.cost.ang, locked.cost.ang);
  RecordProperty("swing_ang_rate_max", std::to_string(swing.cost.ang_rate_max));
  RecordProperty("locked_ang_rate_max", std::to_string(locked.cost.ang_rate_max));
  // … and the catch node is within D2's tolerance of the solve without it.
  EXPECT_TRUE(TreeWithinD2(*t, swing, off, &why)) << why;
  RecordProperty("swing_catch_difference", why);
  // Negative: locked, with the term weighted heavily on both phases, the
  // reach gives.
  EXPECT_FALSE(TreeWithinD2(*t, locked_big, off, &why)) << why;
  RecordProperty("locked_big_catch_difference", why);
  EXPECT_LE(TreeOtherOffRest(*t, locked), 1e-9);
}

}  // namespace
