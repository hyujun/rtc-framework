// E3-F02 (#641): the whole-body terms of MpcDockingSegmentCore — the velocity
// weight on both running sums, the stop-node rest toward q_nom and the
// angular-momentum rate with its coefficients held from the start point on
// (D12). Judged:
//   • off is the problem without them, to the bit; Init refuses what it cannot
//     solve (with the catch instant a variable among it);
//   • the solve's convergence contract keeps its meaning with them on: the
//     assembled gradient is the merit's (finite differences, 2e-6);
//   • the reported values are the direct sums — the angular one on the START
//     point's coefficients after a Solve, on its own before any;
//   • the same input gives the same answer whatever was solved before, and no
//     stage outside the QP solver allocates.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_core.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/centroidal.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
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

}  // namespace
