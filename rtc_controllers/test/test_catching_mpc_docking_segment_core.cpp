// mpc_docking numeric core (E1-F13, #739) — the SQP on synthetic throws.
//
// Feasible cases are CONSTRUCTED (mpc_docking_fixture.hpp): a trajectory inside
// every limit exists by construction, so "the solver did not converge" can
// never be excused by "perhaps the problem was infeasible". Every hard row of
// the returned solution is re-evaluated outside the core, by a route that
// shares no code with it.
//
// The C-level malloc gate is defined in this TU (one per binary); the core
// header comes first because it pulls in <malloc.h> (through ProxQP), whose
// noexcept declarations must be seen before the gate defines the replacements.
#include "rtc_controllers/catching/mpc_docking_segment_core.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/malloc_gate.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <limits>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

using rtc::catching::BallCovariance;
using rtc::catching::DockingRowGroup;
using rtc::catching::DockingRowGroupName;
using rtc::catching::kMaxSegmentNv;
using rtc::catching::kNumDockingElasticGroups;
using rtc::catching::kNumDockingRowGroups;
using rtc::catching::MpcDockingReason;
using rtc::catching::MpcDockingReasonName;
using rtc::catching::MpcDockingSegmentCore;
using rtc::catching::MpcDockingSegmentCoreInput;
using rtc::catching::MpcDockingSegmentCoreParams;
using rtc::catching::MpcDockingSegmentCoreResult;
using rtc::catching::MpcDockingStage;
using rtc::catching::MpcSegmentCore;
using rtc::catching::MpcSegmentCoreParams;
using rtc::catching::NodeTrajectoryFollower;
using rtc::catching::SegmentSnapshot;
using rtc::catching::ValidateSegmentNodes;
namespace fx = rtc::testing::mpc_segment_core;
namespace dk = rtc::testing::mpc_docking;

// Sprint Contract thresholds (#739) — judged on values recomputed OUTSIDE the
// core. The core's own tol_violation (1e-6, BaseParams) is tighter on purpose.
constexpr double kViolationTol = 1e-5;
constexpr double kTerminalRestTol = 1e-6;
constexpr double kKktRelTol = 1e-4;
constexpr double kComplementarityTol = 1e-5;
constexpr int kFeasibleCases = 20;

std::int64_t NoClock() noexcept {
  return 0;
}

std::size_t G(DockingRowGroup g) {
  return static_cast<std::size_t>(g);
}

std::vector<dk::Rig> Rigs() {
  return {dk::MakeRig(fx::RealArm6()), dk::MakeRig(fx::RealArm7())};
}

std::string Describe(const MpcDockingSegmentCoreResult& r) {
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "reason=%s feasible=%d converged=%d it=%d qp=%d bt=%d mu_up=%d kkt=%.3e grad=%.3e "
                "comp=%.3e J=%.4f qp_status=%d qp_it=%d mu_ent=%.1e",
                MpcDockingReasonName(r.reason), r.feasible, r.converged, r.iterations, r.qp_solves,
                r.backtracks, r.mu_updates, r.kkt_residual, r.grad_norm, r.complementarity,
                r.cost.total, r.qp_status, r.qp_iterations, r.mu[2]);
  std::string s(buf);
  for (int g = 0; g < kNumDockingRowGroups; ++g) {
    std::snprintf(buf, sizeof(buf), " %s=%.2e",
                  DockingRowGroupName(static_cast<DockingRowGroup>(g)),
                  r.violation[static_cast<std::size_t>(g)]);
    s += buf;
  }
  return s;
}

// RecordProperty(double) goes through to_string (6 fixed decimals).
void RecordSci(const std::string& key, double value) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.4e", value);
  ::testing::Test::RecordProperty(key, buf);
}

struct Case {
  dk::Throw th;
  MpcDockingSegmentCoreInput in;
  unsigned seed{0};
};

// The first `count` seeds whose constructed trajectory satisfies every hard
// row with a margin; `tried` reports how many seeds that took.
std::vector<Case> FeasibleCases(const dk::Rig& rig, const MpcDockingSegmentCore& core, int count,
                                int* tried = nullptr) {
  std::vector<Case> cases;
  int seen = 0;
  for (unsigned seed = 1; static_cast<int>(cases.size()) < count && seed < 2000; ++seed) {
    Case c;
    c.seed = seed;
    ++seen;
    if (dk::MakeThrow(rig, core, seed, 1e-3, c.th, c.in)) {
      cases.push_back(std::move(c));
    }
  }
  if (tried != nullptr) {
    *tried = seen;
  }
  return cases;
}

dk::Nodes NodesOf(const MpcDockingSegmentCoreResult& r) {
  return dk::Nodes{r.q, r.qd, r.qdd};
}

// ∇L and the complementarity of the core's last QP, recomputed from its
// matrices and multipliers with ProxQP's signs (H x + g + Cᵀz + Aᵀy = 0).
struct Kkt {
  double grad_l{0.0};
  double grad_j{0.0};
  double complementarity{0.0};
  double qp_stationarity{0.0};
};

Kkt RecomputeKkt(const MpcDockingSegmentCore& core) {
  const rtc::tsid::QPData& qp = core.LastQp();
  const Eigen::VectorXd& x = core.LastQpSolution();
  const Eigen::VectorXd& y = core.LastEqualityDual();
  const Eigen::VectorXd& z = core.LastInequalityDual();
  const Eigen::Index nu = core.NumJerkVariables();
  const Eigen::VectorXd grad_l = qp.g + qp.C.transpose() * z + qp.A.transpose() * y;
  const Eigen::VectorXd cx = qp.C * x;
  Kkt k;
  k.grad_l = grad_l.head(nu).cwiseAbs().maxCoeff();
  k.grad_j = qp.g.head(nu).cwiseAbs().maxCoeff();
  k.qp_stationarity = (qp.H * x + grad_l).cwiseAbs().maxCoeff();
  for (Eigen::Index i = 0; i < z.size(); ++i) {
    if (z[i] > 0.0 && std::isfinite(qp.u[i])) {
      k.complementarity = std::max(k.complementarity, std::abs(z[i] * (qp.u[i] - cx[i])));
    } else if (z[i] < 0.0 && std::isfinite(qp.l[i])) {
      k.complementarity = std::max(k.complementarity, std::abs(z[i] * (cx[i] - qp.l[i])));
    }
  }
  return k;
}

// ── Feasible throws ──────────────────────────────────────────────────────────

TEST(MpcDockingSegmentCore, ConstructedFeasibleThrowsConverge) {
  for (const dk::Rig& rig : Rigs()) {
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult out;
    core.ResizeResult(out);
    int tried = 0;
    std::vector<Case> cases = FeasibleCases(rig, core, kFeasibleCases, &tried);
    ASSERT_EQ(static_cast<int>(cases.size()), kFeasibleCases) << rig.arm.name;
    int max_iterations = 0;
    int total_iterations = 0;
    int total_backtracks = 0;
    int max_backtracks = 0;
    double worst_violation = 0.0;
    double worst_kkt_ratio = 0.0;
    double worst_comp = 0.0;
    for (Case& c : cases) {
      dk::PerturbTarget(c.in, c.seed);
      const std::string where = rig.arm.name + " seed " + std::to_string(c.seed);
      ASSERT_TRUE(core.Solve(c.in, out)) << where << " " << Describe(out);
      EXPECT_EQ(out.reason, MpcDockingReason::kConverged) << where << " " << Describe(out);
      EXPECT_TRUE(out.converged) << where;
      EXPECT_TRUE(out.feasible) << where;
      EXPECT_LE(out.iterations, rig.params.max_iterations) << where;

      // Hard rows, recomputed outside the core.
      const std::array<double, kNumDockingRowGroups> viol =
          dk::HardRowViolations(rig, core, NodesOf(out), c.in);
      for (int g = 0; g < kNumDockingRowGroups; ++g) {
        const auto group = static_cast<DockingRowGroup>(g);
        if (group == DockingRowGroup::kImpact) {
          continue;  // rows off in this fixture
        }
        const double tol = group == DockingRowGroup::kTerminal ? kTerminalRestTol : kViolationTol;
        EXPECT_LE(viol[static_cast<std::size_t>(g)], tol)
            << where << " group " << DockingRowGroupName(group);
        worst_violation = std::max(worst_violation, viol[static_cast<std::size_t>(g)]);
      }
      // KKT, recomputed from the last QP's matrices and multipliers.
      const Kkt kkt = RecomputeKkt(core);
      EXPECT_LE(kkt.grad_l, kKktRelTol * std::max(1.0, kkt.grad_j)) << where;
      EXPECT_LE(kkt.complementarity, kComplementarityTol) << where;
      EXPECT_NEAR(kkt.grad_l, out.kkt_residual, 1e-12) << where;
      EXPECT_NEAR(kkt.grad_j, out.grad_norm, 1e-12) << where;
      EXPECT_NEAR(kkt.complementarity, out.complementarity, 1e-12) << where;
      // The multipliers are the QP's: its own stationarity holds to its
      // tolerance (otherwise ∇L above would be measured with the wrong signs).
      EXPECT_LE(kkt.qp_stationarity, 1e-5) << where;

      // The reported cost is consistent, and the solution is no worse than
      // the constructed trajectory it was allowed to differ from.
      EXPECT_NEAR(out.cost.total, out.cost.reference + out.cost.stop, 1e-12);
      EXPECT_GT(out.cost.reference, 0.0);
      EXPECT_EQ(out.approach_nodes, core.NumApproachNodes());
      EXPECT_GT(out.c_catch, rig.params.c_min);
      EXPECT_FALSE(out.c_guarded);
      EXPECT_GT(out.sigma_t, 0.0);

      max_iterations = std::max(max_iterations, out.iterations);
      total_iterations += out.iterations;
      total_backtracks += out.backtracks;
      max_backtracks = std::max(max_backtracks, out.backtracks);
      worst_kkt_ratio = std::max(worst_kkt_ratio, kkt.grad_l / std::max(1.0, kkt.grad_j));
      worst_comp = std::max(worst_comp, kkt.complementarity);
    }
    const std::string tag = rig.arm.name;
    ::testing::Test::RecordProperty(tag + "_seeds_tried", tried);
    ::testing::Test::RecordProperty(tag + "_iterations_max", max_iterations);
    ::testing::Test::RecordProperty(tag + "_iterations_total", total_iterations);
    ::testing::Test::RecordProperty(tag + "_backtracks_total", total_backtracks);
    ::testing::Test::RecordProperty(tag + "_backtracks_max_per_solve", max_backtracks);
    RecordSci(tag + "_worst_violation", worst_violation);
    RecordSci(tag + "_worst_kkt_ratio", worst_kkt_ratio);
    RecordSci(tag + "_worst_complementarity", worst_comp);
  }
}

// The fixture's own claim: the constructed trajectory is a feasible point —
// through the core's evaluation as well as the independent checker — and the
// two evaluations agree row group by row group.
TEST(MpcDockingSegmentCore, ConstructedTrajectoryIsFeasibleByBothEvaluations) {
  for (const dk::Rig& rig : Rigs()) {
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult out;
    core.ResizeResult(out);
    for (Case& c : FeasibleCases(rig, core, 5)) {
      c.in.initial_valid = true;
      c.in.q_init = c.th.known.q;
      c.in.qd_init = c.th.known.qd;
      c.in.qdd_init = c.th.known.qdd;
      ASSERT_TRUE(core.Evaluate(c.in, out));
      EXPECT_TRUE(out.feasible) << rig.arm.name << " seed " << c.seed << " " << Describe(out);
      EXPECT_EQ(out.reason, MpcDockingReason::kNone);
      EXPECT_FALSE(out.converged);
      // Projection onto the block jerk reproduces a representable trajectory.
      EXPECT_LT((out.q - c.th.known.q).cwiseAbs().maxCoeff(), 1e-12);
      EXPECT_LT((out.qdd - c.th.known.qdd).cwiseAbs().maxCoeff(), 1e-9);
      const auto viol = dk::HardRowViolations(rig, core, c.th.known, c.in);
      for (int g = 0; g < kNumDockingRowGroups; ++g) {
        // Same sign convention on the positive side; the core clips at 0.
        EXPECT_NEAR(out.violation[static_cast<std::size_t>(g)],
                    std::max(viol[static_cast<std::size_t>(g)], 0.0), 1e-9)
            << DockingRowGroupName(static_cast<DockingRowGroup>(g));
      }
    }
  }
}

// ── The assembled QP against finite differences of the nonlinear problem ─────

// Every term of the cost on, so the assembled gradient covers all of them.
dk::Rig FullCostRig(const fx::ArmModel& arm) {
  dk::Rig rig = dk::MakeRig(arm);
  rig.params.w_manip = 0.02;
  rig.params.manip_d_lin = 0.5;
  rig.params.w_impact = 0.5;
  rig.params.e_ref = 0.05;
  rig.params.e_max = 0.2;
  rig.params.p_max = 0.3;
  rig.params.contact_point_hand = Eigen::Vector3d(0.01, -0.01, 0.02);
  rig.params.w_perp = 30.0;
  rig.params.accel_box = true;
  rig.limits.qdd_max = Eigen::VectorXd::Constant(arm.model->nv, 80.0);
  // One iteration, and a start that is taken as given even when a
  // finite-difference step breaks the terminal rest by a hair.
  rig.params.max_iterations = 1;
  rig.params.tol_linear = 1e-2;
  return rig;
}

TEST(MpcDockingSegmentCore, AssembledGradientAndRowsMatchFiniteDifferences) {
  for (const fx::ArmModel& arm : {fx::RealArm6(), fx::RealArm7()}) {
    const dk::Rig rig = FullCostRig(arm);
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult out;
    core.ResizeResult(out);
    dk::Throw th;
    MpcDockingSegmentCoreInput in;
    (void)dk::MakeThrow(rig, core, 3, 0.0, th, in);
    in.p_line = th.p_b + Eigen::Vector3d(0.02, -0.03, 0.01);
    in.d_line = Eigen::Vector3d(0.3, -0.5, 0.8).normalized();
    in.initial_valid = true;
    const Eigen::Index n = core.Nv();
    const Eigen::Index nu = core.NumJerkVariables();
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n);
    // The base point: the constructed trajectory, pushed off its optimum.
    // The push stays in the null space of the terminal equality (per joint,
    // the two rows ĝ_v(N), ĝ_a(N)), so the start is still at rest at node N
    // and is taken as given.
    Eigen::VectorXd z0 = dk::JerkThrough(core, th.q0, th.q_c, th.v_c);
    {
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
          z0[b * n + j] += kept[b];
        }
      }
    }
    const auto set_start = [&](const Eigen::VectorXd& z) {
      const dk::Nodes nodes = dk::NodesFromJerk(core, th.q0, zero, zero, z);
      in.q_init = nodes.q;
      in.qd_init = nodes.qd;
      in.qdd_init = nodes.qdd;
    };
    set_start(z0);
    ASSERT_TRUE(core.Solve(in, out)) << Describe(out);
    ASSERT_FALSE(out.init_qp_used) << "the start must be taken as given";
    const rtc::tsid::QPData base = core.LastQp();  // copy

    // ── Cost gradient: central differences of the smooth part of J ──
    const auto smooth_cost = [&](const Eigen::VectorXd& z) {
      set_start(z);
      EXPECT_TRUE(core.Evaluate(in, out));
      return out.cost.total - out.cost.slack;
    };
    const double h = 1e-6;
    double worst_grad = 0.0;
    for (Eigen::Index i = 0; i < nu; ++i) {
      Eigen::VectorXd zp = z0;
      Eigen::VectorXd zm = z0;
      zp[i] += h;
      zm[i] -= h;
      const double fd = (smooth_cost(zp) - smooth_cost(zm)) / (2.0 * h);
      EXPECT_NEAR(base.g[i], fd, 2e-6 * std::max(1.0, std::abs(fd)))
          << arm.name << " variable " << i;
      worst_grad = std::max(worst_grad, std::abs(base.g[i] - fd));
    }
    EXPECT_GT(base.g.head(nu).cwiseAbs().maxCoeff(), 1e-2);
    RecordSci(arm.name + "_gradient_fd_max_abs_diff", worst_grad);

    // ── Rows: a row's bound, as a function of the linearisation point, moves
    // by minus the row's coefficients (bound = const − h(z)). ──
    struct Check {
      int row;
      bool upper;
    };

    std::vector<Check> checks;
    const int tau0 = core.GroupRowBegin(DockingRowGroup::kTorque);
    const int half = core.GroupRowCount(DockingRowGroup::kTorque) / 2;
    for (int r = 0; r < half; ++r) {
      checks.push_back({tau0 + r, true});
      checks.push_back({tau0 + half + r, false});
    }
    const int app0 = core.GroupRowBegin(DockingRowGroup::kGap);
    set_start(z0);
    ASSERT_TRUE(core.Evaluate(in, out));
    int corridor_checked = 0;
    for (int i = 0; i < core.NumApproachNodes(); ++i) {
      checks.push_back({app0 + 3 * i, false});
      // The corridor row's bound carries the slack it was linearised at; only
      // with that slack at zero is it "const − g(z)".
      if (out.slack_c[static_cast<std::size_t>(i)] == 0.0) {
        checks.push_back({app0 + 3 * i + 1, true});
        ++corridor_checked;
      }
      checks.push_back({app0 + 3 * i + 2, true});
    }
    EXPECT_GT(core.NumApproachNodes(), 0);
    EXPECT_GT(corridor_checked, 0) << "no corridor row is checked on this fixture";
    const int ent0 = core.GroupRowBegin(DockingRowGroup::kEntrance);
    checks.push_back({ent0, true});
    checks.push_back({ent0 + 1, false});
    const int lat0 = core.GroupRowBegin(DockingRowGroup::kLateral);
    for (int i = 0; i < core.GroupRowCount(DockingRowGroup::kLateral); ++i) {
      checks.push_back({lat0 + i, true});
    }
    checks.push_back({core.GroupRowBegin(DockingRowGroup::kTiming), false});
    const int vel0 = core.GroupRowBegin(DockingRowGroup::kVelocitySet);
    checks.push_back({vel0, false});
    for (int i = 1; i < core.GroupRowCount(DockingRowGroup::kVelocitySet); ++i) {
      checks.push_back({vel0 + i, true});
    }
    const int imp0 = core.GroupRowBegin(DockingRowGroup::kImpact);
    for (int i = 0; i < 3; ++i) {
      checks.push_back({imp0 + i, true});
    }
    for (const Check& c : checks) {
      ASSERT_TRUE(std::isfinite(c.upper ? base.u[c.row] : base.l[c.row])) << "row " << c.row;
    }
    Eigen::MatrixXd fd_rows(static_cast<Eigen::Index>(checks.size()), nu);
    for (Eigen::Index i = 0; i < nu; ++i) {
      Eigen::VectorXd zp = z0;
      Eigen::VectorXd zm = z0;
      zp[i] += h;
      zm[i] -= h;
      set_start(zp);
      ASSERT_TRUE(core.Solve(in, out));
      const Eigen::VectorXd lp = core.LastQp().l;
      const Eigen::VectorXd up = core.LastQp().u;
      set_start(zm);
      ASSERT_TRUE(core.Solve(in, out));
      const Eigen::VectorXd& lm = core.LastQp().l;
      const Eigen::VectorXd& um = core.LastQp().u;
      for (std::size_t c = 0; c < checks.size(); ++c) {
        const int row = checks[c].row;
        const double dbound =
            checks[c].upper ? (up[row] - um[row]) / (2.0 * h) : (lp[row] - lm[row]) / (2.0 * h);
        fd_rows(static_cast<Eigen::Index>(c), i) = -dbound;
      }
    }
    double worst_row = 0.0;
    for (std::size_t c = 0; c < checks.size(); ++c) {
      const Eigen::RowVectorXd coeff = base.C.row(checks[c].row).head(nu);
      const Eigen::RowVectorXd fd = fd_rows.row(static_cast<Eigen::Index>(c));
      const double scale = std::max(1.0, fd.cwiseAbs().maxCoeff());
      const double diff = (coeff - fd).cwiseAbs().maxCoeff();
      EXPECT_LT(diff, 2e-5 * scale) << arm.name << " row " << checks[c].row;
      EXPECT_GT(fd.cwiseAbs().maxCoeff(), 1e-6)
          << arm.name << " row " << checks[c].row << " is not exercised";
      worst_row = std::max(worst_row, diff / scale);
    }
    RecordSci(arm.name + "_rows_fd_max_rel_diff", worst_row);
    ::testing::Test::RecordProperty(arm.name + "_rows_checked", static_cast<int>(checks.size()));
  }
}

// ── Infeasible problems say so ───────────────────────────────────────────────

struct InfeasibleCase {
  std::string name;
  dk::Rig rig;
  std::set<DockingRowGroup> expected;
  // Fill `in` for this case (the core is already initialised on `rig`).
  void (*fill)(const dk::Rig&, const MpcDockingSegmentCore&, MpcDockingSegmentCoreInput&);
};

void FillFeasibleSeed(const dk::Rig& rig, const MpcDockingSegmentCore& core,
                      MpcDockingSegmentCoreInput& in) {
  dk::Throw th;
  (void)dk::MakeThrow(rig, core, 2, 0.0, th, in);
}

std::vector<InfeasibleCase> InfeasibleCases(const fx::ArmModel& arm) {
  std::vector<InfeasibleCase> cases;
  // 1. The catch point is beyond the arm's reach: 1.5 m further UP the capture
  //    axis of the known catch pose. Along the axis, so that the hand already
  //    points at the ball and what cannot be closed is the axial gap alone
  //    (off the axis, "a plane through the ball" would trade the same
  //    distance into the lateral rows and the residual group would be a coin
  //    toss between the two).
  cases.push_back(
      {"out_of_reach",
       dk::MakeRig(arm),
       {DockingRowGroup::kEntrance},
       [](const dk::Rig& rig, const MpcDockingSegmentCore& core, MpcDockingSegmentCoreInput& in) {
         dk::Throw th;
         (void)dk::MakeThrow(rig, core, 2, 0.0, th, in);
         const dk::HandState h = dk::HandAt(rig, th.q_c, th.v_c);
         dk::FillBall(core, th.p_b + 1.5 * h.R.col(2), th.v_b, th.cov, in);
       }});
  // 2. The lead is too short for the joint speed limits: the ball crosses the
  //    entrance plane's place 0.25 m up the capture axis of the hand's
  //    PRESENT pose, 40 ms from now. The hand would need ~6 m/s to be there.
  {
    dk::Rig rig = dk::MakeRig(arm);
    rig.params.n_pre = 2;
    rig.params.dt_pre = 0.02;
    rig.params.n_blocks = 7;
    rig.params.block_sizes = {1, 1, 1, 1, 2, 2, 2};
    cases.push_back(
        {"lead_too_short",
         rig,
         {DockingRowGroup::kEntrance},
         [](const dk::Rig& r, const MpcDockingSegmentCore& core, MpcDockingSegmentCoreInput& in) {
           const Eigen::Index n = r.model.nv;
           const Eigen::VectorXd q0 = r.arm.q_nominal;
           const dk::HandState h = dk::HandAt(r, q0, Eigen::VectorXd::Zero(n));
           const Eigen::Vector3d r_h(0.0, 0.0, r.params.s_ent + 0.25);
           const Eigen::Vector3d nu_h(0.0, 0.0, -0.8);
           std::mt19937 gen(5);
           core.ResizeInput(in);
           in.q0 = q0;
           in.catch_target_valid = true;
           in.q_catch_target = q0;
           dk::FillBall(core, h.p + h.R * r_h, h.R * nu_h, dk::SmallCovariance(gen), in);
         }});
  }
  // 3. The torque bounds are below what holding the arm takes.
  {
    dk::Rig rig = dk::MakeRig(arm);
    rig.limits.tau_lo = -0.02 * rig.limits.tau_max;
    rig.limits.tau_hi = 0.02 * rig.limits.tau_max;
    cases.push_back({"torque_exceeded",
                     rig,
                     {DockingRowGroup::kTorque, DockingRowGroup::kEntrance},
                     &FillFeasibleSeed});
  }
  // 4. The axial position spread is so large that the timing row needs a
  //    closing speed above what the capture set allows (faces widened so the
  //    lateral rows are not what fails).
  {
    dk::Rig rig = dk::MakeRig(arm);
    rig.params.face_b = {0.5, 0.5, 0.5, 0.5};
    cases.push_back(
        {"timing_vs_velocity_set",
         rig,
         {DockingRowGroup::kTiming, DockingRowGroup::kVelocitySet},
         [](const dk::Rig& r, const MpcDockingSegmentCore& core, MpcDockingSegmentCoreInput& in) {
           dk::Throw th;
           (void)dk::MakeThrow(r, core, 2, 0.0, th, in);
           BallCovariance cov = th.cov;
           cov.topLeftCorner<3, 3>() += (0.03 * 0.03) * Eigen::Matrix3d::Identity();
           dk::FillBall(core, th.p_b, th.v_b, cov, in);
         }});
  }
  return cases;
}

TEST(MpcDockingSegmentCore, InfeasibleProblemsAreReportedWithTheirRowGroup) {
  for (const fx::ArmModel& arm : {fx::RealArm6(), fx::RealArm7()}) {
    for (const InfeasibleCase& c : InfeasibleCases(arm)) {
      const std::string where = arm.name + " " + c.name;
      MpcDockingSegmentCore core;
      ASSERT_EQ(core.Init(c.rig.model, c.rig.arm.frame, c.rig.params, c.rig.limits, &NoClock),
                MpcDockingReason::kNone)
          << where;
      MpcDockingSegmentCoreResult out;
      core.ResizeResult(out);
      MpcDockingSegmentCoreInput in;
      c.fill(c.rig, core, in);
      ASSERT_TRUE(core.Solve(in, out)) << where << " " << Describe(out);
      EXPECT_FALSE(out.feasible) << where << " " << Describe(out);
      EXPECT_FALSE(out.converged) << where;
      EXPECT_EQ(out.reason, MpcDockingReason::kInfeasible) << where << " " << Describe(out);
      EXPECT_TRUE(c.expected.count(out.infeasible_group) == 1)
          << where << " residual group " << DockingRowGroupName(out.infeasible_group) << " "
          << Describe(out);
      // Every group the last QP left an elastic in is one the case expects —
      // and at least one is. (The QP's elastic, not the nonlinear violation:
      // the solve stops as soon as the violation stalls, and at that iterate
      // other groups may still be off by an amount the next step would
      // remove; what the QP cannot remove is the diagnosis.)
      int residual_groups = 0;
      for (int g = 0; g < kNumDockingElasticGroups; ++g) {
        const auto group = static_cast<DockingRowGroup>(g);
        if (out.elastic[static_cast<std::size_t>(g)] > kViolationTol) {
          ++residual_groups;
          EXPECT_TRUE(c.expected.count(group) == 1)
              << where << " unexpected residual elastic in " << DockingRowGroupName(group) << " "
              << Describe(out);
        }
      }
      EXPECT_GT(residual_groups, 0) << where;
      // The residual group is violated on the nonlinear model too, by far
      // more than the tolerance.
      EXPECT_GT(out.violation[static_cast<std::size_t>(out.infeasible_group)],
                100.0 * kViolationTol)
          << where;
      // The linear rows are never relaxed.
      EXPECT_LE(out.violation[G(DockingRowGroup::kBox)], kViolationTol) << where;
      EXPECT_LE(out.violation[G(DockingRowGroup::kTerminal)], kTerminalRestTol) << where;
      const std::string tag = arm.name + "_" + c.name;
      ::testing::Test::RecordProperty(tag + "_iterations", out.iterations);
      ::testing::Test::RecordProperty(tag + "_group", DockingRowGroupName(out.infeasible_group));
      ::testing::Test::RecordProperty(tag + "_mu_updates", out.mu_updates);
    }
  }
}

// ── Recorded conditions (no verdict) ─────────────────────────────────────────

// A short lead on the 7-DOF arm, on four grids: one coarse pre-catch interval
// (the QP then has no running terms and no approach rows at all), two, and
// four fine ones. The stop is the shipped 7 × 0.05 s with blocks 1, 1, 2, 3.
TEST(MpcDockingSegmentCore, RecordsShortLeadGrids) {
  struct Grid {
    const char* tag;
    int n_pre;
    double dt_pre;
  };

  for (const Grid& g : {Grid{"lead_0p10_x1", 1, 0.1}, Grid{"lead_0p10_x2", 2, 0.1},
                        Grid{"lead_0p04_x4", 4, 0.04}, Grid{"lead_0p05_x4", 4, 0.05}}) {
    dk::Rig rig = dk::MakeRig(fx::RealArm7());
    dk::SetShortLeadGrid(rig.params, g.n_pre, g.dt_pre);
    dk::RecordTally(std::string("real_7dof_") + g.tag, dk::SolveGeneratedThrows(rig, 20));
  }
}

// A real throw: the ball arrives at 4–5 m/s and the capture set allows a
// closing speed of at most 1.0 m/s, so the hand must move with the ball at
// most of its speed. Whether the arm's limits allow that is what this records.
TEST(MpcDockingSegmentCore, RecordsRealThrowCondition) {
  for (const fx::ArmModel& arm : {fx::RealArm6(), fx::RealArm7()}) {
    dk::Rig rig = dk::MakeRig(arm);
    rig.params.c_cap_max = 1.0;
    rig.params.c_ent_max = 1.0;
    dk::RecordTally(arm.name + "_real_throw_4p0_to_5p0", dk::SolveRealThrows(rig, 4.0, 5.0, 10));
  }
}

// ── Payload and the RT sampler ───────────────────────────────────────────────

TEST(MpcDockingSegmentCore, SolutionRoundTripsThroughThePayloadAndTheRtSampler) {
  // device_of_model[m]: a non-identity permutation, so a copy that skipped
  // the mapping would be seen.
  const std::array<int, 7> device_of_model{2, 0, 5, 1, 4, 3, 6};
  for (const dk::Rig& rig : Rigs()) {
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult out;
    core.ResizeResult(out);
    std::vector<Case> cases = FeasibleCases(rig, core, 1);
    ASSERT_EQ(cases.size(), 1U);
    ASSERT_TRUE(core.Solve(cases[0].in, out));
    ASSERT_TRUE(out.converged);

    const int nv = core.Nv();
    const int N = core.NumNodes();
    const int kc = core.CatchNode();
    ASSERT_LE(N, rtc::catching::kMaxSegmentNodes);
    SegmentSnapshot p{};
    p.valid = true;
    p.nv = nv;
    p.n_nodes = N;
    p.n_pre = kc;
    p.k0 = 0;
    p.dt_ns = static_cast<std::int64_t>(std::llround(rig.params.dt_stop * 1e9));
    p.dt_pre_ns = static_cast<std::int64_t>(std::llround(rig.params.dt_pre * 1e9));
    p.t_c_ns = 1'727'000'000'123'456'789LL;
    p.t0_ns = p.t_c_ns - static_cast<std::int64_t>(kc) * p.dt_pre_ns;
    const auto idx = [](int k, int j) { return static_cast<std::size_t>(k * kMaxSegmentNv + j); };
    for (int k = 0; k <= N; ++k) {
      for (int m = 0; m < nv; ++m) {
        const int d = nv == 7 ? device_of_model[static_cast<std::size_t>(m)]
                              : device_of_model[static_cast<std::size_t>(m)] % 6;
        p.q[idx(k, d)] = out.q(m, k);
        p.qd[idx(k, d)] = out.qd(m, k);
        p.qdd[idx(k, d)] = out.qdd(m, k);
      }
    }
    // The 6-joint mapping must still be a permutation.
    if (nv == 6) {
      std::set<int> seen;
      for (int m = 0; m < 6; ++m) {
        seen.insert(device_of_model[static_cast<std::size_t>(m)] % 6);
      }
      ASSERT_EQ(seen.size(), 6U);
    }
    ASSERT_TRUE(ValidateSegmentNodes(p)) << rig.arm.name;

    std::array<double, kMaxSegmentNv> q{};
    std::array<double, kMaxSegmentNv> qd{};
    std::array<double, kMaxSegmentNv> qdd{};
    const auto device = [&](int m) {
      return static_cast<std::size_t>(nv == 7 ? device_of_model[static_cast<std::size_t>(m)]
                                              : device_of_model[static_cast<std::size_t>(m)] % 6);
    };
    for (int k = 0; k <= N; ++k) {
      const std::int64_t t_ns = rtc::catching::SegmentNodeTimeNs(p, k);
      // The payload's node instants are the core's.
      EXPECT_NEAR(static_cast<double>(t_ns - p.t0_ns) * 1e-9, core.NodeTime(k), 1e-12);
      bool held = false;
      ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(p, t_ns, q, qd, qdd, &held));
      for (int m = 0; m < nv; ++m) {
        EXPECT_NEAR(q[device(m)], out.q(m, k), 1e-12) << rig.arm.name << " node " << k;
        EXPECT_NEAR(qd[device(m)], out.qd(m, k), 1e-12) << rig.arm.name << " node " << k;
        EXPECT_NEAR(qdd[device(m)], out.qdd(m, k), 1e-12) << rig.arm.name << " node " << k;
      }
    }
    // Between nodes the sampler reproduces the core's constant-jerk model:
    // q(τ) = q_k + q̇_k τ + ½ q̈_k τ² + ⅙ u_k τ³ with the core's own jerk u_k.
    for (int k = 0; k < N; ++k) {
      const std::int64_t t0 = rtc::catching::SegmentNodeTimeNs(p, k);
      const std::int64_t t1 = rtc::catching::SegmentNodeTimeNs(p, k + 1);
      const std::int64_t t_ns = t0 + (t1 - t0) * 3 / 8;
      const double tau = static_cast<double>(t_ns - t0) * 1e-9;
      ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(p, t_ns, q, qd, qdd, nullptr));
      for (int m = 0; m < nv; ++m) {
        const double u = out.u(m, k);
        const double want = out.q(m, k) + out.qd(m, k) * tau + 0.5 * out.qdd(m, k) * tau * tau +
                            u * tau * tau * tau / 6.0;
        EXPECT_NEAR(q[device(m)], want, 1e-9) << rig.arm.name << " interval " << k;
        EXPECT_NEAR(qdd[device(m)], out.qdd(m, k) + u * tau, 1e-6);
      }
    }
  }
}

TEST(MpcDockingSegmentCore, StageGainsEqualTheSegmentCores) {
  for (const dk::Rig& rig : Rigs()) {
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcSegmentCoreParams sp;
    sp.n_pre = rig.params.n_pre;
    sp.dt_pre = rig.params.dt_pre;
    sp.n_nodes = rig.params.n_stop;
    sp.dt = rig.params.dt_stop;
    sp.n_blocks = rig.params.n_blocks;
    sp.block_sizes = rig.params.block_sizes;
    sp.u_scale = rig.params.u_scale;
    MpcSegmentCore reference;
    ASSERT_EQ(reference.Init(*rig.arm.model, rig.arm.frame, sp,
                             fx::LimitsFromModel(*rig.arm.model, 0.05)),
              rtc::catching::MpcSegmentCoreReason::kNone);
    ASSERT_EQ(core.NumNodes(), reference.NumNodes());
    ASSERT_EQ(core.CatchNode(), reference.CatchNode());
    ASSERT_EQ(core.NumBlocks(), reference.NumBlocks());
    double largest = 0.0;
    for (int k = 0; k <= core.NumNodes(); ++k) {
      EXPECT_EQ(core.NodeTime(k), reference.NodeTime(k));
      for (int b = 0; b < core.NumBlocks(); ++b) {
        for (int m = 0; m < 3; ++m) {
          EXPECT_EQ(core.StageGain(m, k, b), reference.StageGain(m, k, b))
              << "m " << m << " node " << k << " block " << b;
          largest = std::max(largest, std::abs(core.StageGain(m, k, b)));
        }
      }
    }
    EXPECT_GT(largest, 1.0) << "the gains compared are not all zero";
    EXPECT_TRUE(std::isnan(core.StageGain(3, 1, 0)));
    EXPECT_TRUE(std::isnan(core.StageGain(0, core.NumNodes() + 1, 0)));
    EXPECT_TRUE(std::isnan(core.NodeTime(-1)));
    EXPECT_EQ(core.TerminalRank(), 2 * core.Nv());
  }
}

// ── Allocation ───────────────────────────────────────────────────────────────

static_assert(noexcept(
    std::declval<MpcDockingSegmentCore&>().Solve(std::declval<const MpcDockingSegmentCoreInput&>(),
                                                 std::declval<MpcDockingSegmentCoreResult&>())));
static_assert(noexcept(std::declval<MpcDockingSegmentCore&>().Evaluate(
    std::declval<const MpcDockingSegmentCoreInput&>(),
    std::declval<MpcDockingSegmentCoreResult&>())));

constexpr std::size_t kStages = 6;

struct StageRecord {
  std::array<std::size_t, kStages> mallocs{};
  std::array<int, kStages> entries{};
};

// Arms the C-level malloc gate for the stage's duration (the gate's own
// counter, read at the stage's end).
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

const char* StageName(std::size_t s) {
  constexpr std::array<const char*, kStages> names{"start",   "linearize", "assemble",
                                                   "post_qp", "merit",     "finish"};
  return names[s];
}

// The core's own code allocates nothing: every stage outside the QP solver is
// bracketed by the hook and counted by the C-level gate — on EVERY iteration,
// which is the point (a path that stops before the first QP sees only the
// first linearisation, not the line search or the penalty update).
TEST(MpcDockingSegmentCore, EveryStageOutsideTheSolverAllocatesNothing) {
  // Positive control: the gate counts an allocation made inside a library.
  {
    const rtc::testing::ScopedMallocGate gate;
    const pinocchio::Data probe(*fx::RealArm7().model);
    ASSERT_GT(gate.count(), 0U) << "the malloc gate does not see library allocations";
  }
  // Every optional term on, so every stage runs all of its code.
  for (const fx::ArmModel& arm : {fx::RealArm6(), fx::RealArm7()}) {
    dk::Rig rig = dk::MakeRig(arm);
    rig.params.w_manip = 0.02;
    rig.params.w_impact = 0.5;
    rig.params.e_ref = 0.05;
    rig.params.e_max = 0.5;
    rig.params.p_max = 0.5;
    rig.params.w_perp = 30.0;
    rig.params.accel_box = true;
    rig.params.jerk_box = true;
    rig.limits.qdd_max = Eigen::VectorXd::Constant(arm.model->nv, 80.0);
    rig.limits.jerk_max = Eigen::VectorXd::Constant(arm.model->nv, 5e3);
    // A small penalty first, so the penalty update runs too.
    rig.params.mu_init.fill(1e-2);
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult out;
    core.ResizeResult(out);
    std::vector<Case> cases = FeasibleCases(rig, core, 3);
    ASSERT_EQ(cases.size(), 3U);
    StageRecord rec;
    core.SetStageHook(&CountingHook, &rec);
    std::size_t news = 0;
    std::size_t whole_solve_mallocs = 0;
    int iterations = 0;
    int backtracks = 0;
    int mu_updates = 0;
    int restarts = 0;
    for (Case& c : cases) {
      c.in.p_line = c.th.p_b;
      dk::PerturbTarget(c.in, c.seed);
      {
        const rtc::testing::ScopedAllocGate new_gate;
        const bool ok = core.Solve(c.in, out);
        news += new_gate.count();
        ASSERT_TRUE(ok) << Describe(out);
      }
      iterations += out.iterations;
      backtracks += out.backtracks;
      mu_updates += out.mu_updates;
      // …and once more from its own solution: the projected start path.
      MpcDockingSegmentCoreInput again = c.in;
      again.initial_valid = true;
      again.q_init = out.q;
      again.qd_init = out.qd;
      again.qdd_init = out.qdd;
      {
        const rtc::testing::ScopedAllocGate new_gate;
        const bool ok = core.Solve(again, out);
        news += new_gate.count();
        ASSERT_TRUE(ok);
      }
      restarts += out.init_qp_used ? 0 : 1;
      {
        const rtc::testing::ScopedAllocGate new_gate;
        const bool ok = core.Evaluate(again, out);
        news += new_gate.count();
        ASSERT_TRUE(ok);
      }
    }
    core.SetStageHook(nullptr, nullptr);
    // The whole Solve, solver included — recorded, not judged (#654).
    {
      const rtc::testing::ScopedMallocGate gate;
      ASSERT_TRUE(core.Solve(cases[0].in, out));
      whole_solve_mallocs = gate.count();
    }
    EXPECT_EQ(news, 0U) << arm.name << ": operator new inside Solve";
    for (std::size_t s = 0; s < kStages; ++s) {
      EXPECT_EQ(rec.mallocs[s], 0U) << arm.name << " stage " << StageName(s);
      EXPECT_GT(rec.entries[s], 0) << arm.name << " stage " << StageName(s) << " never ran";
    }
    // The run must have exercised more than a first iteration.
    EXPECT_GT(iterations, 6);
    EXPECT_GT(mu_updates, 0) << "the penalty update did not run";
    EXPECT_GT(restarts, 0) << "the projected-start path did not run";
    ::testing::Test::RecordProperty(arm.name + "_iterations", iterations);
    ::testing::Test::RecordProperty(arm.name + "_backtracks", backtracks);
    ::testing::Test::RecordProperty(arm.name + "_mu_updates", mu_updates);
    ::testing::Test::RecordProperty(arm.name + "_whole_solve_c_mallocs",
                                    static_cast<int>(whole_solve_mallocs));
    ::testing::Test::RecordProperty(arm.name + "_whole_solve_qp_solves", out.qp_solves);
  }
}

// ── Deadline, single iteration, restart ──────────────────────────────────────

std::atomic<std::int64_t> g_fake_now{0};
std::atomic<int> g_clock_reads{0};

std::int64_t FakeClock() noexcept {
  g_clock_reads.fetch_add(1);
  return g_fake_now.load();
}

// Whatever the reason, the numbers in the result belong to the trajectory in
// the result: evaluating that trajectory again reproduces them.
void ExpectResultDescribesItsTrajectory(MpcDockingSegmentCore& core,
                                        const MpcDockingSegmentCoreInput& in,
                                        const MpcDockingSegmentCoreResult& solved) {
  MpcDockingSegmentCoreInput again = in;
  again.initial_valid = true;
  again.q_init = solved.q;
  again.qd_init = solved.qd;
  again.qdd_init = solved.qdd;
  MpcDockingSegmentCoreResult eval;
  core.ResizeResult(eval);
  ASSERT_TRUE(core.Evaluate(again, eval));
  EXPECT_LT((eval.q - solved.q).cwiseAbs().maxCoeff(), 1e-10);
  EXPECT_NEAR(eval.cost.total, solved.cost.total, 1e-9 * std::max(1.0, solved.cost.total));
  for (int g = 0; g < kNumDockingRowGroups; ++g) {
    EXPECT_NEAR(eval.violation[static_cast<std::size_t>(g)],
                solved.violation[static_cast<std::size_t>(g)], 1e-9)
        << DockingRowGroupName(static_cast<DockingRowGroup>(g));
  }
  EXPECT_EQ(eval.feasible, solved.feasible);
}

TEST(MpcDockingSegmentCore, DeadlineReturnsTheLastAcceptedIterate) {
  const dk::Rig rig = dk::MakeRig(fx::RealArm7());
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &FakeClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  MpcDockingSegmentCoreInput in = cases[0].in;
  dk::PerturbTarget(in, 1);

  // No deadline: the reference run.
  g_fake_now = 1000;
  in.deadline_ns = 0;
  g_clock_reads = 0;
  ASSERT_TRUE(core.Solve(in, out));
  ASSERT_TRUE(out.converged);
  const int full_iterations = out.iterations;
  ASSERT_GT(full_iterations, 2);
  EXPECT_EQ(g_clock_reads.load(), 0) << "no deadline, no clock read";

  // A deadline already behind the clock: the check sits BETWEEN iterations, so
  // exactly one iteration runs and its accepted step is what comes back.
  in.deadline_ns = 500;
  ASSERT_TRUE(core.Solve(in, out));
  EXPECT_EQ(out.reason, MpcDockingReason::kDeadline);
  EXPECT_FALSE(out.converged);
  EXPECT_EQ(out.iterations, 1);
  EXPECT_GT(out.kkt_residual, 0.0);
  EXPECT_TRUE(out.q.allFinite());
  EXPECT_LE(out.violation[G(DockingRowGroup::kBox)], kViolationTol);
  EXPECT_LE(out.violation[G(DockingRowGroup::kTerminal)], kTerminalRestTol);
  ExpectResultDescribesItsTrajectory(core, in, out);
  const Eigen::MatrixXd after_one = out.q;

  // The same iterate a core limited to that many iterations returns.
  dk::Rig two = rig;
  two.params.max_iterations = 2;
  MpcDockingSegmentCore limited;
  ASSERT_EQ(limited.Init(two.model, two.arm.frame, two.params, two.limits, &FakeClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out2;
  limited.ResizeResult(out2);
  in.deadline_ns = 500;
  ASSERT_TRUE(limited.Solve(in, out2));
  EXPECT_EQ(out2.reason, MpcDockingReason::kDeadline);
  EXPECT_EQ(out2.q, after_one);

  // A deadline ahead of the clock changes nothing.
  in.deadline_ns = 5000;
  ASSERT_TRUE(core.Solve(in, out));
  EXPECT_TRUE(out.converged);
  EXPECT_EQ(out.iterations, full_iterations);

  // No clock at all: the deadline is ignored.
  core.SetClock(nullptr);
  in.deadline_ns = 500;
  ASSERT_TRUE(core.Solve(in, out));
  EXPECT_TRUE(out.converged);
}

TEST(MpcDockingSegmentCore, IterationLimitReturnsAnIterateThatIsNotConverged) {
  dk::Rig rig = dk::MakeRig(fx::RealArm6());
  rig.params.max_iterations = 2;
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  dk::PerturbTarget(cases[0].in, 4);
  ASSERT_TRUE(core.Solve(cases[0].in, out));
  EXPECT_EQ(out.reason, MpcDockingReason::kIterationLimit);
  EXPECT_FALSE(out.converged);
  EXPECT_EQ(out.iterations, 2);
  ExpectResultDescribesItsTrajectory(core, cases[0].in, out);
}

// One real-time iteration: a full step with no line search. It can be feasible
// but never `converged` — the planner judges it by the violations.
TEST(MpcDockingSegmentCore, SingleIterationTakesAFullStepAndNeverReportsConverged) {
  dk::Rig rig = dk::MakeRig(fx::RealArm7());
  rig.params.max_iterations = 1;
  MpcDockingSegmentCore rti;
  ASSERT_EQ(rti.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  dk::Rig full_rig = dk::MakeRig(fx::RealArm7());
  MpcDockingSegmentCore full;
  ASSERT_EQ(
      full.Init(full_rig.model, full_rig.arm.frame, full_rig.params, full_rig.limits, &NoClock),
      MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  rti.ResizeResult(out);
  MpcDockingSegmentCoreResult solved;
  full.ResizeResult(solved);
  std::vector<Case> cases = FeasibleCases(rig, rti, 1);
  ASSERT_EQ(cases.size(), 1U);
  MpcDockingSegmentCoreInput in = cases[0].in;
  dk::PerturbTarget(in, 9);
  ASSERT_TRUE(rti.Solve(in, out));
  EXPECT_EQ(out.reason, MpcDockingReason::kIterationLimit);
  EXPECT_FALSE(out.converged);
  EXPECT_EQ(out.iterations, 1);
  EXPECT_EQ(out.backtracks, 0);
  ExpectResultDescribesItsTrajectory(rti, in, out);
  // The step it took is the QP's: start + d, for every block.
  const Eigen::VectorXd& d = rti.LastQpSolution();
  EXPECT_GT(d.head(rti.NumJerkVariables()).cwiseAbs().maxCoeff(), 1e-4);

  // Real-time iterations from the converged solution stay at it.
  ASSERT_TRUE(full.Solve(in, solved));
  ASSERT_TRUE(solved.converged);
  MpcDockingSegmentCoreInput warm = in;
  warm.initial_valid = true;
  warm.q_init = solved.q;
  warm.qd_init = solved.qd;
  warm.qdd_init = solved.qdd;
  ASSERT_TRUE(rti.Solve(warm, out));
  EXPECT_FALSE(out.init_qp_used);
  EXPECT_LT((out.q - solved.q).cwiseAbs().maxCoeff(), 1e-4);
  EXPECT_TRUE(out.feasible) << Describe(out);
  // It is the one-iteration case of "converged at the start": the core
  // recognises the KKT point and says so, without stepping.
  EXPECT_EQ(out.reason, MpcDockingReason::kConverged);
}

TEST(MpcDockingSegmentCore, SameInputGivesTheSameAnswerWhateverWasSolvedBefore) {
  for (const dk::Rig& rig : Rigs()) {
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult a;
    MpcDockingSegmentCoreResult b;
    MpcDockingSegmentCoreResult other;
    core.ResizeResult(a);
    core.ResizeResult(b);
    core.ResizeResult(other);
    std::vector<Case> cases = FeasibleCases(rig, core, 2);
    ASSERT_EQ(cases.size(), 2U);
    dk::PerturbTarget(cases[0].in, 1);
    dk::PerturbTarget(cases[1].in, 2);
    ASSERT_TRUE(core.Solve(cases[0].in, a));
    ASSERT_TRUE(core.Solve(cases[1].in, other));
    ASSERT_TRUE(core.Solve(cases[0].in, b));
    EXPECT_EQ(a.q, b.q) << rig.arm.name;
    EXPECT_EQ(a.u, b.u) << rig.arm.name;
    EXPECT_EQ(a.iterations, b.iterations);
    EXPECT_EQ(a.cost.total, b.cost.total);
    EXPECT_GT((a.q - other.q).cwiseAbs().maxCoeff(), 1e-3) << "the two cases must differ";
  }
}

TEST(MpcDockingSegmentCore, RestartFromItsOwnSolutionConvergesAtOnce) {
  for (const dk::Rig& rig : Rigs()) {
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult first;
    MpcDockingSegmentCoreResult second;
    core.ResizeResult(first);
    core.ResizeResult(second);
    std::vector<Case> cases = FeasibleCases(rig, core, 1);
    ASSERT_EQ(cases.size(), 1U);
    dk::PerturbTarget(cases[0].in, 3);
    ASSERT_TRUE(core.Solve(cases[0].in, first));
    ASSERT_TRUE(first.converged);
    EXPECT_TRUE(first.init_qp_used);
    MpcDockingSegmentCoreInput again = cases[0].in;
    again.initial_valid = true;
    again.q_init = first.q;
    again.qd_init = first.qd;
    again.qdd_init = first.qdd;
    ASSERT_TRUE(core.Solve(again, second));
    EXPECT_FALSE(second.init_qp_used) << "its own solution satisfies the linear rows";
    EXPECT_TRUE(second.converged) << Describe(second);
    EXPECT_EQ(second.iterations, 1);
    EXPECT_LT((second.q - first.q).cwiseAbs().maxCoeff(), 1e-9);

    // A start that violates the linear rows is repaired by the initialisation
    // QP instead of being linearised around. The projection reads the node
    // ACCELERATIONS (the trajectory is rebuilt from x_0), so that is where
    // the violation has to be: 40× the accelerations breaks the speed box.
    again.qdd_init *= 40.0;
    ASSERT_TRUE(core.Solve(again, second));
    EXPECT_TRUE(second.init_qp_used);
    EXPECT_TRUE(second.converged) << Describe(second);
    EXPECT_LE(second.violation[G(DockingRowGroup::kBox)], kViolationTol);
  }
}

// ── Optional rows and terms ──────────────────────────────────────────────────

TEST(MpcDockingSegmentCore, OptionalRowsAndTermsStillConverge) {
  for (const fx::ArmModel& arm : {fx::RealArm6(), fx::RealArm7()}) {
    dk::Rig rig = dk::MakeRig(arm);
    rig.params.w_manip = 0.02;
    rig.params.w_impact = 0.5;
    rig.params.e_ref = 0.05;
    rig.params.e_max = 0.5;
    rig.params.p_max = 0.5;
    rig.params.w_perp = 30.0;
    rig.params.accel_box = true;
    rig.params.jerk_box = true;
    rig.limits.qdd_max = Eigen::VectorXd::Constant(arm.model->nv, 80.0);
    rig.limits.jerk_max = Eigen::VectorXd::Constant(arm.model->nv, 5e3);
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult out;
    core.ResizeResult(out);
    for (Case& c : FeasibleCases(rig, core, 5)) {
      c.in.p_line = c.th.p_b;
      c.in.d_line = Eigen::Vector3d::UnitZ();
      dk::PerturbTarget(c.in, c.seed);
      ASSERT_TRUE(core.Solve(c.in, out));
      const std::string where = arm.name + " seed " + std::to_string(c.seed) + " " + Describe(out);
      EXPECT_TRUE(out.converged) << where;
      EXPECT_TRUE(out.feasible) << where;
      EXPECT_GT(out.cost.manip, -1e9);
      EXPECT_GT(out.cost.impact, 0.0) << where;
      EXPECT_GT(out.cost.stop_line, 0.0) << where;
      // The accel and jerk boxes, recomputed from the nodes.
      for (int k = 0; k < core.NumNodes(); ++k) {
        for (Eigen::Index j = 0; j < core.Nv(); ++j) {
          EXPECT_LE(std::abs(out.qdd(j, k + 1)), 80.0 + 1e-5) << where;
          EXPECT_LE(std::abs(out.u(j, k)), 5e3 + 1e-2) << where;
          const double dt = core.NodeTime(k + 1) - core.NodeTime(k);
          EXPECT_NEAR(out.u(j, k), (out.qdd(j, k + 1) - out.qdd(j, k)) / dt, 1e-6) << where;
        }
      }
    }
  }
}

TEST(MpcDockingSegmentCore, WithoutChanceRowsNoCovarianceIsNeeded) {
  dk::Rig rig = dk::MakeRig(fx::RealArm6());
  MpcDockingSegmentCore with_chance;
  ASSERT_EQ(with_chance.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  EXPECT_GT(with_chance.TimingSigmaMax(), 0.0);
  rig.params.chance = false;
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  EXPECT_EQ(core.TimingSigmaMax(), 0.0) << "the timing row is not built";
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  MpcDockingSegmentCoreInput in = cases[0].in;
  in.ball[static_cast<std::size_t>(core.CatchNode())].cov_valid = false;
  in.ball[static_cast<std::size_t>(core.CatchNode())].cov.setConstant(
      std::numeric_limits<double>::quiet_NaN());
  ASSERT_TRUE(core.Solve(in, out));
  EXPECT_TRUE(out.converged) << Describe(out);
  EXPECT_EQ(out.sigma_s, 0.0);
  EXPECT_FALSE(out.linearization_ratio_defined);

  // The same input is refused when the chance rows are on.
  with_chance.ResizeResult(out);
  out.q.setConstant(7.0);
  EXPECT_FALSE(with_chance.Solve(in, out));
  EXPECT_EQ(out.reason, MpcDockingReason::kCovarianceInvalid);
  EXPECT_EQ(out.q(0, 0), 7.0) << "a rejected call leaves the trajectory untouched";
}

// The timing row's σ_max: with the closure centred in the window it is the
// reference's Δ_win / (2 κ_t).
TEST(MpcDockingSegmentCore, TimingSigmaMaxCentredIsTheReferenceFormula) {
  dk::Rig rig = dk::MakeRig(fx::RealArm6());
  rig.params.delta_lo = 0.004;
  rig.params.delta_hi = 0.034;
  rig.params.delta_0 = 0.019;
  rig.params.eps_t = 0.05;
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  double kappa_t = 0.0;
  ASSERT_TRUE(rtc::catching::NormalQuantile(1.0 - 0.5 * rig.params.eps_t, kappa_t));
  EXPECT_NEAR(core.TimingSigmaMax(), 0.030 / (2.0 * kappa_t), 1e-12);
  // The risks became the κ the reference defines.
  double kappa_face = 0.0;
  ASSERT_TRUE(rtc::catching::NormalQuantile(1.0 - rig.params.face_eps[0], kappa_face));
  EXPECT_DOUBLE_EQ(core.FaceKappa(0), kappa_face);
  EXPECT_TRUE(std::isnan(core.FaceKappa(rig.params.n_faces)));
  double kappa_nu = 0.0;
  ASSERT_TRUE(rtc::catching::NormalQuantile(1.0 - rig.params.eps_nu, kappa_nu));
  EXPECT_DOUBLE_EQ(core.VelocityKappa(), kappa_nu);
}

// ── Rejections ───────────────────────────────────────────────────────────────

TEST(MpcDockingSegmentCore, InitRejectsWhatItCannotSolve) {
  const dk::Rig base = dk::MakeRig(fx::RealArm6());
  const auto init = [&](const dk::Rig& rig) {
    MpcDockingSegmentCore core;
    const MpcDockingReason why =
        core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock);
    EXPECT_EQ(core.IsInitialized(), why == MpcDockingReason::kNone);
    return why;
  };
  ASSERT_EQ(init(base), MpcDockingReason::kNone);
  const auto with = [&](const std::function<void(dk::Rig&)>& edit) {
    dk::Rig rig = base;
    edit(rig);
    return init(rig);
  };
  EXPECT_EQ(with([](dk::Rig& r) { r.params.n_pre = 0; }), MpcDockingReason::kParamsInvalid);
  EXPECT_EQ(with([](dk::Rig& r) { r.params.dt_pre = 0.0; }), MpcDockingReason::kParamsInvalid);
  EXPECT_EQ(with([](dk::Rig& r) { r.params.block_sizes[0] = 2; }), MpcDockingReason::kParamsInvalid)
      << "block sizes must sum to N";
  EXPECT_EQ(with([](dk::Rig& r) {
              // 5 + 2 across the catch node (k_c = 6), same total.
              r.params.block_sizes = {1, 1, 1, 1, 1, 2, 1, 2, 2, 2};
              r.params.n_blocks = 10;
            }),
            MpcDockingReason::kBlocksAcrossCatch);
  EXPECT_EQ(with([](dk::Rig& r) {
              r.params.block_sizes = {1, 1, 1, 1, 1, 1, 4, 4};
              r.params.n_blocks = 8;
            }),
            MpcDockingReason::kBlocksTooFew);
  EXPECT_EQ(with([](dk::Rig& r) { r.params.r_jerk[1] = 0.0; }), MpcDockingReason::kParamsInvalid);
  EXPECT_EQ(with([](dk::Rig& r) { r.params.nu_ref.z() = 0.3; }), MpcDockingReason::kParamsInvalid)
      << "the reference must approach";
  EXPECT_EQ(with([](dk::Rig& r) { r.params.c_cap_max = r.params.c_min; }),
            MpcDockingReason::kParamsInvalid);
  EXPECT_EQ(with([](dk::Rig& r) { r.params.face_eps[2] = 0.7; }), MpcDockingReason::kParamsInvalid)
      << "a risk above one half would loosen the row";
  EXPECT_EQ(with([](dk::Rig& r) { r.params.eps_sigma = 0.0; }), MpcDockingReason::kParamsInvalid);
  EXPECT_EQ(with([](dk::Rig& r) { r.params.mu_init[3] = 1e9; }), MpcDockingReason::kParamsInvalid)
      << "above mu_max";
  EXPECT_EQ(with([](dk::Rig& r) { r.params.max_iterations = 0; }),
            MpcDockingReason::kParamsInvalid);
  EXPECT_EQ(with([](dk::Rig& r) { r.params.delta_0 = r.params.delta_hi; }),
            MpcDockingReason::kTimingWindowInvalid)
      << "the closure must be strictly inside the window";
  EXPECT_EQ(with([](dk::Rig& r) { r.params.sigma_tau = 0.1; }),
            MpcDockingReason::kTimingWindowInvalid)
      << "the latency jitter alone exceeds what the window tolerates";
  EXPECT_EQ(with([](dk::Rig& r) {
              r.params.sigma_tau = 0.1;
              r.params.chance = false;
            }),
            MpcDockingReason::kNone)
      << "without chance rows the timing parameters are not read";
  EXPECT_EQ(with([](dk::Rig& r) { r.limits.qd_max[0] = 0.0; }), MpcDockingReason::kLimitsInvalid);
  EXPECT_EQ(with([](dk::Rig& r) { r.limits.tau_lo = r.limits.tau_max; }),
            MpcDockingReason::kLimitsInvalid)
      << "tau_lo without tau_hi";
  EXPECT_EQ(with([](dk::Rig& r) { r.params.accel_box = true; }), MpcDockingReason::kLimitsInvalid)
      << "the box needs its limit";
  EXPECT_EQ(with([](dk::Rig& r) { r.arm.frame = r.model.frames.size(); }),
            MpcDockingReason::kFrameUnknown);
}

TEST(MpcDockingSegmentCore, SolveRejectsBadInputAndLeavesTheResultUntouched) {
  const dk::Rig rig = dk::MakeRig(fx::RealArm6());
  MpcDockingSegmentCore core;
  MpcDockingSegmentCoreResult out;
  MpcDockingSegmentCoreInput good;
  EXPECT_FALSE(core.Solve(good, out));
  EXPECT_EQ(out.reason, MpcDockingReason::kNotInitialized);
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  core.ResizeResult(out);
  std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  good = cases[0].in;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::size_t kc = static_cast<std::size_t>(core.CatchNode());
  const auto expect = [&](const std::function<void(MpcDockingSegmentCoreInput&)>& edit,
                          MpcDockingReason want, const char* what) {
    MpcDockingSegmentCoreInput in = good;
    edit(in);
    out.q.setConstant(7.0);
    EXPECT_FALSE(core.Solve(in, out)) << what;
    EXPECT_EQ(out.reason, want) << what << ": " << MpcDockingReasonName(out.reason);
    EXPECT_EQ(out.q.minCoeff(), 7.0) << what;
    EXPECT_EQ(out.q.maxCoeff(), 7.0) << what;
    EXPECT_FALSE(out.feasible);
    EXPECT_FALSE(out.converged);
  };
  expect([](auto& in) { in.q0.conservativeResize(3); }, MpcDockingReason::kDimMismatch, "q0 size");
  expect([&](auto& in) { in.q0[1] = nan; }, MpcDockingReason::kNonFinite, "NaN q0");
  expect([&](auto& in) { in.q0[1] = rig.limits.q_max[1] + 0.1; },
         MpcDockingReason::kInitialStateOutsideBox, "q0 outside");
  expect([&](auto& in) { in.qd0[2] = 1.01 * rig.limits.qd_max[2]; },
         MpcDockingReason::kInitialStateOutsideBox, "speed above the limit");
  expect([](auto& in) { in.catch_target_valid = false; }, MpcDockingReason::kTargetRequired,
         "no start, no target");
  expect([&](auto& in) { in.q_catch_target[0] = nan; }, MpcDockingReason::kNonFinite, "NaN target");
  expect([](auto& in) { in.ball[2].valid = false; }, MpcDockingReason::kBallInvalid,
         "a node without a ball");
  expect([&](auto& in) { in.ball[0].p.x() = nan; }, MpcDockingReason::kBallInvalid, "NaN ball");
  expect([&](auto& in) { in.ball[kc].cov_valid = false; }, MpcDockingReason::kCovarianceInvalid,
         "no covariance");
  expect([&](auto& in) { in.ball[kc].cov(4, 4) = nan; }, MpcDockingReason::kCovarianceInvalid,
         "NaN covariance");
  expect([&](auto& in) { in.ball[kc].cov(1, 1) = -1e-4; }, MpcDockingReason::kCovarianceInvalid,
         "a covariance that is not PSD");
  expect(
      [](auto& in) {
        in.initial_valid = true;
        in.q_init.resize(2, 2);
      },
      MpcDockingReason::kDimMismatch, "start trajectory shape");
  // Moving at the speed limit into a position limit: no trajectory stops in
  // the box, so there is no start point at all.
  expect(
      [&](auto& in) {
        in.q0[0] = rig.limits.q_max[0] - 1e-3;
        in.qd0[0] = 0.99 * rig.limits.qd_max[0];
        in.q_catch_target = in.q0;
      },
      MpcDockingReason::kLinearInfeasible, "cannot stop inside the box");

  // A wrongly sized result.
  MpcDockingSegmentCoreResult small;
  EXPECT_FALSE(core.Solve(good, small));
  EXPECT_EQ(small.reason, MpcDockingReason::kDimMismatch);
  // Evaluate needs a trajectory.
  EXPECT_FALSE(core.Evaluate(good, out));
  EXPECT_EQ(out.reason, MpcDockingReason::kTargetRequired);
  // And the good input still solves afterwards.
  ASSERT_TRUE(core.Solve(good, out));
  EXPECT_TRUE(out.converged);
}

TEST(MpcDockingSegmentCore, StopLineNeedsAUnitDirection) {
  dk::Rig rig = dk::MakeRig(fx::RealArm6());
  rig.params.w_perp = 10.0;
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  MpcDockingSegmentCoreInput in = cases[0].in;
  in.d_line = Eigen::Vector3d(0.0, 0.0, 2.0);
  EXPECT_FALSE(core.Solve(in, out));
  EXPECT_EQ(out.reason, MpcDockingReason::kInputOutOfRange);
  in.d_line = Eigen::Vector3d::UnitZ();
  in.p_line = cases[0].th.p_b;
  EXPECT_TRUE(core.Solve(in, out));
}

TEST(MpcDockingSegmentCore, ReasonAndGroupNamesAreDistinct) {
  std::set<std::string> names;
  for (int r = 0; r <= static_cast<int>(MpcDockingReason::kSolutionNonFinite); ++r) {
    const std::string name = MpcDockingReasonName(static_cast<MpcDockingReason>(r));
    EXPECT_NE(name, "unknown");
    EXPECT_TRUE(names.insert(name).second) << name;
  }
  names.clear();
  for (int g = 0; g < kNumDockingRowGroups; ++g) {
    const std::string name = DockingRowGroupName(static_cast<DockingRowGroup>(g));
    EXPECT_NE(name, "unknown");
    EXPECT_TRUE(names.insert(name).second) << name;
  }
}

// ── Timing (recorded) ────────────────────────────────────────────────────────

TEST(MpcDockingSegmentCore, RecordsSolveTimeByArmAndGrid) {
  struct Grid {
    const char* tag;
    int n_pre;
    double dt_pre;
    int n_stop;
    std::vector<int> stop_blocks;
  };

  const std::vector<Grid> grids{{"pre6_stop8", 6, 0.05, 8, {1, 1, 2, 2, 2}},
                                {"pre4_stop7", 4, 0.05, 7, {1, 1, 2, 3}},
                                {"pre2_stop7", 2, 0.1, 7, {1, 1, 2, 3}}};
  for (const fx::ArmModel& arm : {fx::RealArm6(), fx::RealArm7()}) {
    for (const Grid& g : grids) {
      dk::Rig rig = dk::MakeRig(arm);
      rig.params.n_pre = g.n_pre;
      rig.params.dt_pre = g.dt_pre;
      rig.params.n_stop = g.n_stop;
      rig.params.n_blocks = g.n_pre + static_cast<int>(g.stop_blocks.size());
      rig.params.block_sizes.fill(1);
      for (std::size_t i = 0; i < g.stop_blocks.size(); ++i) {
        rig.params.block_sizes[static_cast<std::size_t>(g.n_pre) + i] = g.stop_blocks[i];
      }
      MpcDockingSegmentCore core;
      ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
                MpcDockingReason::kNone);
      MpcDockingSegmentCoreResult out;
      core.ResizeResult(out);
      std::vector<Case> cases = FeasibleCases(rig, core, 20);
      std::vector<double> total;
      std::vector<double> per_iteration;
      std::vector<double> qp;
      std::vector<double> rest;
      int iterations = 0;
      int converged = 0;
      for (int rep = 0; rep < 3; ++rep) {
        for (Case& c : cases) {
          MpcDockingSegmentCoreInput in = c.in;
          dk::PerturbTarget(in, c.seed);
          ASSERT_TRUE(core.Solve(in, out));
          converged += out.converged ? 1 : 0;
          iterations += out.iterations;
          total.push_back(out.total_us);
          qp.push_back(out.qp_us);
          rest.push_back(out.total_us - out.qp_us);
          if (out.iterations > 0) {
            per_iteration.push_back(out.total_us / out.iterations);
          }
        }
      }
      const std::string tag = arm.name + "_" + g.tag;
      ::testing::Test::RecordProperty(tag + "_solves", static_cast<int>(total.size()));
      ::testing::Test::RecordProperty(tag + "_converged", converged);
      ::testing::Test::RecordProperty(tag + "_nodes", core.NumNodes());
      ::testing::Test::RecordProperty(tag + "_jerk_variables", core.NumJerkVariables());
      ::testing::Test::RecordProperty(tag + "_qp_variables",
                                      static_cast<int>(core.LastQp().H.rows()));
      ::testing::Test::RecordProperty(tag + "_qp_rows", static_cast<int>(core.LastQp().C.rows()));
      ::testing::Test::RecordProperty(tag + "_iterations_total", iterations);
      fx::RecordMicros(tag + "_total_us_p50", fx::Percentile(total, 0.5));
      fx::RecordMicros(tag + "_total_us_p99", fx::Percentile(total, 0.99));
      fx::RecordMicros(tag + "_per_iteration_us_p50", fx::Percentile(per_iteration, 0.5));
      fx::RecordMicros(tag + "_per_iteration_us_p99", fx::Percentile(per_iteration, 0.99));
      fx::RecordMicros(tag + "_qp_us_p50", fx::Percentile(qp, 0.5));
      fx::RecordMicros(tag + "_outside_qp_us_p50", fx::Percentile(rest, 0.5));
    }
  }
#ifdef RTC_TEST_BUILD_TYPE
  ::testing::Test::RecordProperty("build_type", RTC_TEST_BUILD_TYPE);
#endif
}

}  // namespace
