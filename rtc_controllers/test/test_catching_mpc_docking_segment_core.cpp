// mpc_docking numeric core (E1-F13, #739) — the SQP on synthetic throws.
//
// Feasible cases are CONSTRUCTED (mpc_docking_fixture.hpp): a trajectory inside
// every limit exists by construction, so "the solver did not converge" can
// never be excused by "perhaps the problem was infeasible". Every hard row of
// the returned solution is re-evaluated outside the core, by a route that
// shares no code with it.
//
// The C-level malloc gate (rtc_base's malloc_gate.hpp) is defined in this TU
// (one per binary).
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_core.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"
#include "rtc_controllers/testing/planner_trace_digest.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
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
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

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

// Which group an infeasible problem leaves its residual in is decided by the
// RATIO of the penalties (the core grows them together, so the ratio is the
// caller's): "the ball is not in the capture set" can be paid as an axial gap
// or as a lateral miss, and with equal penalties the cheaper of the two is a
// matter of geometry. These cases price the lateral rows ten times higher, so
// a miss is reported as the axial gap.
dk::Rig InfeasibleRig(const fx::ArmModel& arm) {
  dk::Rig rig = dk::MakeRig(arm);
  rig.params.mu_init[G(DockingRowGroup::kLateral)] = 1e3;
  return rig;
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
       InfeasibleRig(arm),
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
    dk::Rig rig = InfeasibleRig(arm);
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
  // 3. The torque bounds are below what holding the arm takes, let alone
  //    moving it: 2 % of the rating. The ball crosses 0.10 m up the capture
  //    axis of the present pose in 0.3 s — an easy reach, and on the axis for
  //    the same reason as above: what the arm gives up when it cannot afford
  //    the motion is then the axial gap, not a lateral miss.
  {
    dk::Rig rig = InfeasibleRig(arm);
    rig.limits.tau_lo = -0.02 * rig.limits.tau_max;
    rig.limits.tau_hi = 0.02 * rig.limits.tau_max;
    cases.push_back(
        {"torque_exceeded",
         rig,
         {DockingRowGroup::kTorque, DockingRowGroup::kEntrance},
         [](const dk::Rig& r, const MpcDockingSegmentCore& core, MpcDockingSegmentCoreInput& in) {
           const Eigen::Index n = r.model.nv;
           const Eigen::VectorXd q0 = r.arm.q_nominal;
           const dk::HandState h = dk::HandAt(r, q0, Eigen::VectorXd::Zero(n));
           const Eigen::Vector3d r_h(0.0, 0.0, r.params.s_ent + 0.10);
           const Eigen::Vector3d nu_h(0.0, 0.0, -0.8);
           std::mt19937 gen(6);
           core.ResizeInput(in);
           in.q0 = q0;
           in.catch_target_valid = true;
           in.q_catch_target = q0;
           dk::FillBall(core, h.p + h.R * r_h, h.R * nu_h, dk::SmallCovariance(gen), in);
         }});
  }
  // 4. The axial position spread is so large that the timing row needs a
  //    closing speed above what the capture set allows (faces widened so the
  //    lateral rows are not what fails).
  {
    dk::Rig rig = InfeasibleRig(arm);
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
// four fine ones. The stop is 7 × 0.05 s with blocks 1, 1, 2, 3.
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

// The instants a solve is cut by in the tests below: FakeClock stands before
// the deadline until the first iteration's step has been judged, and past it
// from then on — the deadline passes DURING the first iteration.
constexpr std::int64_t kFakeDeadlineNs = 500;
constexpr std::int64_t kFakePastNs = 1000;

void PassTheDeadlineAfterTheFirstStep(MpcDockingStage stage, bool begin, void* /*user*/) noexcept {
  if (!begin && stage == MpcDockingStage::kMerit) {
    g_fake_now = kFakePastNs;
  }
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

  // The deadline passes while the first iteration runs: the check before the
  // second iteration's QP sees it, so exactly one iteration runs and its
  // accepted step is what comes back. (A deadline already behind the clock
  // when the call is made starts no QP at all — the tests after this one.)
  in.deadline_ns = kFakeDeadlineNs;
  g_fake_now = 0;
  core.SetStageHook(&PassTheDeadlineAfterTheFirstStep, nullptr);
  ASSERT_TRUE(core.Solve(in, out));
  core.SetStageHook(nullptr, nullptr);
  EXPECT_EQ(out.reason, MpcDockingReason::kDeadline);
  EXPECT_EQ(out.cut_site, rtc::catching::MpcDockingCutSite::kIteration);
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
  in.deadline_ns = kFakeDeadlineNs;
  g_fake_now = 0;
  limited.SetStageHook(&PassTheDeadlineAfterTheFirstStep, nullptr);
  ASSERT_TRUE(limited.Solve(in, out2));
  limited.SetStageHook(nullptr, nullptr);
  EXPECT_EQ(out2.reason, MpcDockingReason::kDeadline);
  EXPECT_EQ(out2.q, after_one);

  // A deadline ahead of the clock changes nothing.
  g_fake_now = kFakePastNs;
  in.deadline_ns = 5000;
  ASSERT_TRUE(core.Solve(in, out));
  EXPECT_TRUE(out.converged);
  EXPECT_EQ(out.iterations, full_iterations);

  // No clock at all: the deadline is ignored.
  core.SetClock(nullptr);
  in.deadline_ns = kFakeDeadlineNs;
  ASSERT_TRUE(core.Solve(in, out));
  EXPECT_TRUE(out.converged);
}

// ── The deadline before every QP ─────────────────────────────────────────────
// A clock that stands at 0 until its `g_trip_at`-th read (counted from 0) and
// past the deadline from that read on. The core reads its clock only where it
// is about to start a QP, so one solve per read of an undisturbed solve passes
// the deadline once before each QP that solve runs.
std::atomic<int> g_trip_at{-1};
constexpr std::int64_t kTripDeadlineNs = 500;
constexpr double kUntouched = 7.0;  // what a result's nodes hold before a call

std::int64_t TripClock() noexcept {
  const int read = g_clock_reads.fetch_add(1);
  const int at = g_trip_at.load();
  return at >= 0 && read >= at ? 2 * kTripDeadlineNs : 0;
}

struct CutSolve {
  int trip{0};
  bool returned{false};
  MpcDockingSegmentCoreResult out;
};

// `in` solved with the deadline ahead for good (`whole`), then once per clock
// read of that solve (every `stride`-th) with the deadline passing AT that
// read.
std::vector<CutSolve> SolveCutAtEveryRead(MpcDockingSegmentCore& core,
                                          MpcDockingSegmentCoreInput in,
                                          MpcDockingSegmentCoreResult& whole, int stride = 1) {
  core.SetClock(&TripClock);
  in.deadline_ns = kTripDeadlineNs;
  g_trip_at = -1;
  g_clock_reads = 0;
  core.ResizeResult(whole);
  EXPECT_TRUE(core.Solve(in, whole));
  const int reads = g_clock_reads.load();
  std::vector<CutSolve> cuts(static_cast<std::size_t>((reads + stride - 1) / stride));
  for (int k = 0; k < reads; k += stride) {
    CutSolve& c = cuts[static_cast<std::size_t>(k / stride)];
    c.trip = k;
    core.ResizeResult(c.out);
    c.out.q.setConstant(kUntouched);
    g_trip_at = k;
    g_clock_reads = 0;
    c.returned = core.Solve(in, c.out);
    // The read that saw the deadline pass is the solve's last: nothing is
    // started after it, so nothing asks the clock again.
    EXPECT_EQ(g_clock_reads.load(), k + 1) << "read " << k;
  }
  g_trip_at = -1;
  return cuts;
}

// What every cut solve owes, wherever it was cut. Returns the sites seen.
std::set<rtc::catching::MpcDockingCutSite> ExpectCutContract(
    const MpcDockingSegmentCoreParams& p, const MpcDockingSegmentCoreResult& whole,
    const std::vector<CutSolve>& cuts) {
  using rtc::catching::MpcDockingCutSite;
  std::set<MpcDockingCutSite> seen;
  int iterations_before = 0;
  const CutSolve* same_iteration = nullptr;
  for (const CutSolve& c : cuts) {
    const std::string where = "read " + std::to_string(c.trip) + ": " +
                              rtc::catching::MpcDockingCutSiteName(c.out.cut_site) + " " +
                              Describe(c.out);
    EXPECT_EQ(c.out.reason, MpcDockingReason::kDeadline) << where;
    EXPECT_NE(c.out.cut_site, MpcDockingCutSite::kNone) << where;
    EXPECT_FALSE(c.out.converged) << where;
    seen.insert(c.out.cut_site);
    if (c.out.cut_site == MpcDockingCutSite::kInitQp) {
      // No iterate: the call is refused, and the result's nodes are not its.
      EXPECT_FALSE(c.returned) << where;
      EXPECT_EQ(c.out.qp_solves, 0) << where;
      EXPECT_FALSE(c.out.init_qp_used) << where;
      EXPECT_TRUE((c.out.q.array() == kUntouched).all()) << where;
      continue;
    }
    EXPECT_TRUE(c.returned) << where;
    EXPECT_TRUE(c.out.q.allFinite()) << where;
    // A later cut has done at least what an earlier one had, never more than
    // the whole solve.
    EXPECT_GE(c.out.iterations, iterations_before) << where;
    EXPECT_LE(c.out.iterations, whole.iterations) << where;
    EXPECT_LE(c.out.qp_solves, whole.qp_solves) << where;
    if (c.out.iterations == 0) {
      // The start point: no QP's step was judged, and the per-QP numbers say
      // so by being 0 — not by being small.
      EXPECT_EQ(c.out.kkt_residual, 0.0) << where;
      EXPECT_EQ(c.out.grad_norm, 0.0) << where;
      EXPECT_EQ(c.out.complementarity, 0.0) << where;
      EXPECT_FALSE(c.out.step_capped) << where;
      for (const double e : c.out.elastic) {
        EXPECT_EQ(e, 0.0) << where;
      }
    }
    // The iterate moves only where a step is accepted: every cut inside one
    // iteration — before its QP, its cold run, its fallback or a probe —
    // returns the same nodes.
    if (same_iteration != nullptr && same_iteration->out.iterations == c.out.iterations) {
      EXPECT_EQ(c.out.q, same_iteration->out.q) << where;
      EXPECT_EQ(c.out.delta_ns, same_iteration->out.delta_ns) << where;
    } else {
      same_iteration = &c;
    }
    iterations_before = c.out.iterations;
    // Every group's penalty is the initial one times the growth steps KEPT: a
    // probe that was cut is undone like one that bought nothing.
    if (c.out.mu_resets == 0) {
      const double factor =
          std::min(std::pow(p.mu_growth, c.out.mu_updates), p.mu_max / p.mu_init[0]);
      for (std::size_t g = 0; g < c.out.mu.size(); ++g) {
        EXPECT_DOUBLE_EQ(c.out.mu[g], p.mu_init[g] * factor) << where << " group " << g;
      }
    }
  }
  return seen;
}

TEST(MpcDockingSegmentCore, PastItsDeadlineNoQpIsStartedWhereverTheSolveStands) {
  using rtc::catching::MpcDockingCutSite;
  dk::Rig rig = dk::MakeRig(fx::RealArm7());
  // A small penalty first, so that the solve grows it: probes are QPs too.
  rig.params.mu_init.fill(1e-2);
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &TripClock),
            MpcDockingReason::kNone);
  std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  MpcDockingSegmentCoreInput in = cases[0].in;
  dk::PerturbTarget(in, 1);

  // The reference: the same solve with no deadline at all.
  MpcDockingSegmentCoreResult free;
  core.ResizeResult(free);
  g_clock_reads = 0;
  ASSERT_TRUE(core.Solve(in, free));
  ASSERT_EQ(g_clock_reads.load(), 0);
  ASSERT_TRUE(free.converged) << Describe(free);
  ASSERT_TRUE(free.init_qp_used);
  ASSERT_GT(free.mu_updates, 0) << "no penalty probe ran: the test does not reach that site";
  ASSERT_GT(free.iterations, 3);

  MpcDockingSegmentCoreResult whole;
  const std::vector<CutSolve> cuts = SolveCutAtEveryRead(core, in, whole);
  // A deadline that never passes changes nothing.
  EXPECT_EQ(whole.q, free.q);
  EXPECT_EQ(whole.iterations, free.iterations);
  EXPECT_EQ(whole.qp_solves, free.qp_solves);
  EXPECT_EQ(whole.reason, free.reason);
  EXPECT_EQ(whole.cut_site, MpcDockingCutSite::kNone);
  // One read before the initialisation QP, one before each iteration's QP and
  // one before each probe (kept or not) — and none anywhere else.
  ASSERT_GE(cuts.size(), static_cast<std::size_t>(1 + whole.iterations + whole.mu_updates));
  const std::set<MpcDockingCutSite> seen = ExpectCutContract(rig.params, whole, cuts);
  EXPECT_EQ(seen.count(MpcDockingCutSite::kInitQp), 1U);
  EXPECT_EQ(seen.count(MpcDockingCutSite::kIteration), 1U);
  EXPECT_EQ(seen.count(MpcDockingCutSite::kPenaltyProbe), 1U);
  EXPECT_EQ(cuts[0].out.cut_site, MpcDockingCutSite::kInitQp);
  // Past the initialisation QP and before the first iteration's: the start
  // point comes back, and it is the initialisation QP's — the linear rows
  // hold on it.
  ASSERT_EQ(cuts[1].out.cut_site, MpcDockingCutSite::kIteration);
  EXPECT_EQ(cuts[1].out.iterations, 0);
  EXPECT_TRUE(cuts[1].out.init_qp_used);
  EXPECT_EQ(cuts[1].out.qp_solves, 1);
  EXPECT_LE(cuts[1].out.violation[G(DockingRowGroup::kBox)], kViolationTol);
  EXPECT_LE(cuts[1].out.violation[G(DockingRowGroup::kTerminal)], kTerminalRestTol);

  // Each returned iterate is described by its own numbers, and after m ≥ 2
  // iterations it is the iterate of a core limited to m.
  std::set<int> compared;
  for (const CutSolve& c : cuts) {
    if (!c.returned) {
      continue;
    }
    ExpectResultDescribesItsTrajectory(core, in, c.out);
    const int m = c.out.iterations;
    if (m < 2 || m > 4 || !compared.insert(m).second) {
      continue;
    }
    dk::Rig limited_rig = rig;
    limited_rig.params.max_iterations = m;
    MpcDockingSegmentCore limited;
    ASSERT_EQ(limited.Init(limited_rig.model, limited_rig.arm.frame, limited_rig.params,
                           limited_rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult out;
    limited.ResizeResult(out);
    MpcDockingSegmentCoreInput unbounded = in;
    unbounded.deadline_ns = 0;
    ASSERT_TRUE(limited.Solve(unbounded, out));
    ASSERT_EQ(out.reason, MpcDockingReason::kIterationLimit) << m;
    EXPECT_EQ(c.out.q, out.q) << "after " << m << " iterations";
  }
  EXPECT_GE(compared.size(), 2U);
  core.SetClock(&NoClock);
}

// A start taken as given has no initialisation QP to be cut before: past its
// deadline the solve hands the start back, evaluated.
TEST(MpcDockingSegmentCore, AStartTakenAsGivenComesBackWhenTheDeadlineHasPassed) {
  using rtc::catching::MpcDockingCutSite;
  const dk::Rig rig = dk::MakeRig(fx::RealArm7());
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &TripClock),
            MpcDockingReason::kNone);
  std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  MpcDockingSegmentCoreInput in = cases[0].in;
  dk::PerturbTarget(in, 1);
  MpcDockingSegmentCoreResult solved;
  core.ResizeResult(solved);
  ASSERT_TRUE(core.Solve(in, solved));
  ASSERT_TRUE(solved.converged);
  // A few iterations short of the solution, so that there is work left.
  dk::Rig short_rig = rig;
  short_rig.params.max_iterations = 2;
  MpcDockingSegmentCore limited;
  ASSERT_EQ(limited.Init(short_rig.model, short_rig.arm.frame, short_rig.params, short_rig.limits,
                         &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult part;
  limited.ResizeResult(part);
  ASSERT_TRUE(limited.Solve(in, part));
  ASSERT_FALSE(part.converged);

  MpcDockingSegmentCoreInput again = in;
  again.initial_valid = true;
  again.q_init = part.q;
  again.qd_init = part.qd;
  again.qdd_init = part.qdd;
  MpcDockingSegmentCoreResult eval;
  core.ResizeResult(eval);
  ASSERT_TRUE(core.Evaluate(again, eval));

  again.deadline_ns = kTripDeadlineNs;
  g_trip_at = 0;
  g_clock_reads = 0;
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  ASSERT_TRUE(core.Solve(again, out));
  g_trip_at = -1;
  EXPECT_EQ(g_clock_reads.load(), 1);
  EXPECT_EQ(out.reason, MpcDockingReason::kDeadline);
  EXPECT_EQ(out.cut_site, MpcDockingCutSite::kIteration);
  EXPECT_FALSE(out.init_qp_used);
  EXPECT_EQ(out.qp_solves, 0);
  EXPECT_EQ(out.iterations, 0);
  EXPECT_EQ(out.kkt_residual, 0.0);
  EXPECT_FALSE(out.converged);
  // The start, as the evaluation of the same input reports it.
  EXPECT_EQ(out.q, eval.q);
  EXPECT_EQ(out.qd, eval.qd);
  EXPECT_EQ(out.cost.total, eval.cost.total);
  EXPECT_EQ(out.violation, eval.violation);
  EXPECT_EQ(out.feasible, eval.feasible);
  // … with the deadline ahead, the same call goes on from there.
  g_clock_reads = 0;
  ASSERT_TRUE(core.Solve(again, out));
  EXPECT_TRUE(out.converged) << Describe(out);
  EXPECT_GT(out.iterations, 0);
  EXPECT_EQ(out.cut_site, MpcDockingCutSite::kNone);
  core.SetClock(&NoClock);
}

// One real-time iteration called past its deadline takes no step either.
TEST(MpcDockingSegmentCore, ASingleIterationPastItsDeadlineTakesNoStep) {
  using rtc::catching::MpcDockingCutSite;
  dk::Rig rig = dk::MakeRig(fx::RealArm7());
  rig.params.max_iterations = 1;
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &TripClock),
            MpcDockingReason::kNone);
  std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  MpcDockingSegmentCoreInput in = cases[0].in;
  dk::PerturbTarget(in, 9);
  MpcDockingSegmentCoreResult whole;
  const std::vector<CutSolve> cuts = SolveCutAtEveryRead(core, in, whole);
  ASSERT_EQ(whole.reason, MpcDockingReason::kIterationLimit);
  ASSERT_EQ(whole.iterations, 1);
  // Two reads: before the initialisation QP and before the one iteration's.
  ASSERT_EQ(cuts.size(), 2U);
  EXPECT_FALSE(cuts[0].returned);
  EXPECT_EQ(cuts[0].out.cut_site, MpcDockingCutSite::kInitQp);
  ASSERT_TRUE(cuts[1].returned);
  EXPECT_EQ(cuts[1].out.reason, MpcDockingReason::kDeadline);
  EXPECT_EQ(cuts[1].out.cut_site, MpcDockingCutSite::kIteration);
  EXPECT_EQ(cuts[1].out.iterations, 0);
  // The full step was not taken: the start is not where the whole call ended.
  EXPECT_GT((cuts[1].out.q - whole.q).cwiseAbs().maxCoeff(), 1e-6);
  ExpectResultDescribesItsTrajectory(core, in, cuts[1].out);
  core.SetClock(&NoClock);
}

// The two QPs a solve runs only after one FAILED — the cold run after a
// warm-started QP, and the run at the initial penalties — are not started past
// the deadline either. A QP fails here the way
// DefaultInfeasibilityThresholdMisreportsAFeasibleQp makes one fail: ProxQP's
// own primal-infeasibility test under a trust region, at a threshold loose
// enough to fire after a penalty has grown.
TEST(MpcDockingSegmentCore, PastItsDeadlineAFailedQpIsNotRunAgain) {
  using rtc::catching::MpcDockingCutSite;
  constexpr int kQpSolved = 0;  // proxsuite::proxqp::QPSolverOutput
  for (const fx::ArmModel& arm : {fx::RealArm6(), fx::RealArm7()}) {
    dk::Rig rig = dk::MakeRig(arm);
    rig.params.delta_tr = 0.05;
    rig.params.solver.eps_primal_inf = 1e-2;
    rig.params.mu_init.fill(1e-2);
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &TripClock),
              MpcDockingReason::kNone);
    // The first throw whose solve falls back to the initial penalties: a QP
    // of it failed warm, failed cold, and failed with the penalties raised.
    MpcDockingSegmentCoreInput in;
    MpcDockingSegmentCoreResult free;
    core.ResizeResult(free);
    bool found = false;
    for (Case& c : FeasibleCases(rig, core, 8)) {
      dk::PerturbTarget(c.in, c.seed);
      if (core.Solve(c.in, free) && free.mu_resets > 0 && free.converged) {
        in = c.in;
        found = true;
        break;
      }
    }
    ASSERT_TRUE(found) << arm.name << ": no solve fell back to the initial penalties";
    MpcDockingSegmentCoreResult whole;
    const std::vector<CutSolve> cuts = SolveCutAtEveryRead(core, in, whole);
    EXPECT_EQ(whole.q, free.q) << arm.name;
    const std::set<MpcDockingCutSite> seen = ExpectCutContract(rig.params, whole, cuts);
    int cold = 0;
    int fallback = 0;
    for (const CutSolve& cut : cuts) {
      const std::string where = arm.name + " read " + std::to_string(cut.trip) + Describe(cut.out);
      if (cut.out.cut_site == MpcDockingCutSite::kColdRetry) {
        ++cold;
        // The record is the warm run's, which did not converge.
        EXPECT_NE(cut.out.qp_status, kQpSolved) << where;
        EXPECT_GT(cut.out.qp_us, 0.0) << where;
      }
      if (cut.out.cut_site == MpcDockingCutSite::kPenaltyReset) {
        ++fallback;
        // The fallback did not run: the penalties are still the raised ones,
        // and the QP that failed under them was run cold as well.
        EXPECT_EQ(cut.out.mu_resets, 0) << where;
        EXPECT_GT(cut.out.mu[0], rig.params.mu_init[0]) << where;
        EXPECT_NE(cut.out.qp_status, kQpSolved) << where;
      }
      if (cut.returned) {
        ExpectResultDescribesItsTrajectory(core, in, cut.out);
      }
    }
    EXPECT_EQ(seen.count(MpcDockingCutSite::kColdRetry), 1U) << arm.name;
    EXPECT_EQ(seen.count(MpcDockingCutSite::kPenaltyReset), 1U) << arm.name;
    // One of each per fallback of the undisturbed solve, at least.
    EXPECT_GE(cold, whole.mu_resets) << arm.name;
    EXPECT_EQ(fallback, whole.mu_resets) << arm.name;
    core.SetClock(&NoClock);
  }
}

// ── The cap on a warm-started QP (solver_max_iter_warm) ──────────────────────
// Every QP of a solve after its first starts from the iterates of the QP
// before it. Under a cap of ONE solver iteration such a start does not
// converge on a QP whose linearisation has moved, so the QP is run again from
// zero under the solver's own cap — and the solve ends where the uncapped one
// does, having solved more QPs on the way.
TEST(MpcDockingSegmentCore, AWarmStartedQpThatMissesItsCapIsRunAgainFromZero) {
  for (const dk::Rig& base : Rigs()) {
    dk::Rig rig = base;
    MpcDockingSegmentCore free;
    ASSERT_EQ(free.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    rig.params.solver_max_iter_warm = 1;
    MpcDockingSegmentCore capped;
    ASSERT_EQ(capped.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    // At the solver's own cap, and above it, there is no separate cap.
    rig.params.solver_max_iter_warm = rig.params.solver.max_iter;
    MpcDockingSegmentCore at_cap;
    ASSERT_EQ(at_cap.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    rig.params.solver_max_iter_warm = rig.params.solver.max_iter + 1;
    MpcDockingSegmentCore above_cap;
    ASSERT_EQ(above_cap.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult a;
    MpcDockingSegmentCoreResult b;
    MpcDockingSegmentCoreResult c;
    free.ResizeResult(a);
    capped.ResizeResult(b);
    at_cap.ResizeResult(c);
    std::vector<Case> cases = FeasibleCases(base, free, 3);
    ASSERT_EQ(cases.size(), 3U);
    int reruns_free = 0;
    int reruns_capped = 0;
    for (Case& cs : cases) {
      dk::PerturbTarget(cs.in, cs.seed);
      const std::string where = base.arm.name + " seed " + std::to_string(cs.seed);
      ASSERT_TRUE(free.Solve(cs.in, a)) << where;
      ASSERT_TRUE(a.converged) << where << Describe(a);
      ASSERT_GT(a.iterations, 1) << where << ": the solve needs a second QP for a warm start";
      ASSERT_TRUE(capped.Solve(cs.in, b)) << where;
      EXPECT_TRUE(b.converged) << where << Describe(b);
      // No penalty was raised and kept, or reset: the QPs beyond one per
      // iteration are re-runs from zero and probes that were undone, and a
      // probe is undone by either core alike.
      EXPECT_EQ(a.mu_updates + a.mu_resets, 0) << where;
      EXPECT_EQ(b.mu_updates + b.mu_resets, 0) << where;
      reruns_free += a.qp_solves - a.iterations;
      reruns_capped += b.qp_solves - b.iterations;
      // Two paths to the same solution, each stopped by the core's own test.
      EXPECT_LT((b.q - a.q).cwiseAbs().maxCoeff(), 1e-4) << where;
      for (MpcDockingSegmentCore* core : {&at_cap, &above_cap}) {
        ASSERT_TRUE(core->Solve(cs.in, c)) << where;
        EXPECT_EQ(c.q, a.q) << where;
        EXPECT_EQ(c.qp_solves, a.qp_solves) << where;
        EXPECT_EQ(c.qp_iterations, a.qp_iterations) << where;
      }
    }
    // A warm start may already be the next QP's solution; over the cases it
    // is not, and each miss is one more QP.
    EXPECT_GT(reruns_capped, reruns_free) << base.arm.name;
  }
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
// One real-time iteration takes the QP's full step unjudged. It can say
// `converged` only about the point it started from — and then takes no step.
TEST(MpcDockingSegmentCore, SingleIterationTakesAFullStepAndConvergesOnlyWhereItStarted) {
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
  EXPECT_TRUE(out.converged);
  EXPECT_EQ(out.iterations, 1);
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

// lateral_margin and timing_margin are the two groups' violation with its sign
// kept: a row that holds reads 0 in `violation` and its room here; a row that
// does not reads the same number in both, negated here. A problem without the
// timing row has no timing margin, and a call refused before any iterate none
// at all.
TEST(MpcDockingSegmentCore, TheMarginsAreTheSignedFormOfTheLateralAndTimingViolation) {
  const auto check = [](const MpcDockingSegmentCoreResult& out, const std::string& where) {
    ASSERT_TRUE(std::isfinite(out.lateral_margin)) << where;
    ASSERT_TRUE(std::isfinite(out.timing_margin)) << where;
    EXPECT_DOUBLE_EQ(out.violation[G(DockingRowGroup::kLateral)],
                     std::max(0.0, -out.lateral_margin))
        << where;
    EXPECT_DOUBLE_EQ(out.violation[G(DockingRowGroup::kTiming)], std::max(0.0, -out.timing_margin))
        << where;
  };
  int holding = 0;
  for (const dk::Rig& rig : Rigs()) {
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult out;
    core.ResizeResult(out);
    for (Case& c : FeasibleCases(rig, core, 2)) {
      const std::string where = rig.arm.name + " seed " + std::to_string(c.seed);
      ASSERT_TRUE(core.Solve(c.in, out)) << where;
      ASSERT_TRUE(out.feasible) << where << " " << Describe(out);
      ASSERT_NO_FATAL_FAILURE(check(out, where));
      // Feasible: both rows hold, so both margins are room (to the tolerance
      // the rows are held to).
      EXPECT_GE(out.lateral_margin, -kViolationTol) << where;
      EXPECT_GE(out.timing_margin, -kViolationTol) << where;
      holding += out.lateral_margin > 0.0 && out.timing_margin > 0.0 ? 1 : 0;
    }
  }
  EXPECT_GT(holding, 0) << "no case with room on both rows: the sign was never exercised";
  int violated = 0;
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
      ASSERT_TRUE(core.Solve(in, out)) << where;
      ASSERT_NO_FATAL_FAILURE(check(out, where));
      violated += out.lateral_margin < -kViolationTol || out.timing_margin < -kViolationTol ? 1 : 0;
    }
  }
  ::testing::Test::RecordProperty("cases_with_a_violated_margin", violated);

  // Without chance rows the timing row is not built: no margin to report. The
  // lateral faces are still rows of the problem.
  dk::Rig rig = dk::MakeRig(fx::RealArm6());
  rig.params.chance = false;
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  ASSERT_TRUE(core.Solve(cases[0].in, out));
  EXPECT_TRUE(std::isnan(out.timing_margin));
  EXPECT_TRUE(std::isfinite(out.lateral_margin));
  // Refused before any iterate: the solve before it leaves nothing behind.
  MpcDockingSegmentCoreInput bad = cases[0].in;
  bad.q0.setConstant(std::numeric_limits<double>::quiet_NaN());
  ASSERT_FALSE(core.Solve(bad, out));
  EXPECT_TRUE(std::isnan(out.lateral_margin));
  EXPECT_TRUE(std::isnan(out.timing_margin));
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
  EXPECT_EQ(with([](dk::Rig& r) { r.params.solver_max_iter_warm = -1; }),
            MpcDockingReason::kParamsInvalid);
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
  EXPECT_EQ(with([](dk::Rig& r) { r.params.eps_sigma = 1e-200; }), MpcDockingReason::kParamsInvalid)
      << "ε_σ² underflows to zero — the square root loses its derivative again";
  EXPECT_EQ(with([](dk::Rig& r) { r.params.face_a[1] = Eigen::Vector2d(-2.0, 0.0); }),
            MpcDockingReason::kParamsInvalid)
      << "face normals must be unit";
  EXPECT_EQ(with([](dk::Rig& r) {
              r.params.lambda1_c = 0.0;
              r.params.lambda2_c = 0.0;
            }),
            MpcDockingReason::kParamsInvalid)
      << "a slack that costs nothing switches the corridor off";
  EXPECT_EQ(with([](dk::Rig& r) {
              r.params.lambda1_v = 0.0;
              r.params.lambda2_v = 0.3;
            }),
            MpcDockingReason::kNone);
  EXPECT_EQ(with([](dk::Rig& r) { r.params.stall_window = 0; }), MpcDockingReason::kParamsInvalid);
  EXPECT_EQ(with([](dk::Rig& r) { r.params.stall_window = 17; }), MpcDockingReason::kParamsInvalid);
  EXPECT_EQ(with([](dk::Rig& r) { r.params.stall_window = 16; }), MpcDockingReason::kNone);
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
  // … and it is the solver's infeasibility certificate that says so, not its
  // iteration cap: the initialisation QP is the one QP here that CAN be
  // infeasible, so it keeps ProxQP's test on.
  constexpr int kQpPrimalInfeasible = 2;  // proxsuite::proxqp::QPSolverOutput
  EXPECT_EQ(out.qp_status, kQpPrimalInfeasible);
  EXPECT_LT(out.qp_iterations, rig.params.solver.max_iter);
  ::testing::Test::RecordProperty("linear_infeasible_qp_iterations", out.qp_iterations);
  ::testing::Test::RecordProperty("linear_infeasible_total_us", static_cast<int>(out.total_us));
  // The catch instant past the end of the prediction: the sampler extrapolates
  // the mean there and says so, and the core does not plan a catch on it.
  expect([&](auto& in) { in.ball[kc].after_horizon = true; }, MpcDockingReason::kBallInvalid,
         "catch node past the prediction");

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

// A rejected call leaves no record of the solve before it: the caller that
// reads `violation` or `cost` next to a rejection reason must not be reading
// the previous candidate's.
TEST(MpcDockingSegmentCore, RejectedCallLeavesNoRecordOfTheSolveBefore) {
  const dk::Rig rig = dk::MakeRig(fx::RealArm7());
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  ASSERT_TRUE(core.Solve(cases[0].in, out));
  ASSERT_TRUE(out.converged) << Describe(out);
  // The first solve left something in every field checked below.
  ASSERT_GT(out.cost.total, 0.0);
  ASSERT_GT(out.mu[0], 0.0);
  ASSERT_GT(out.c_catch, 0.0);
  ASSERT_GT(out.sigma_s, 0.0);
  ASSERT_GT(out.tau_ratio_max, 0.0);
  ASSERT_GT(out.approach_nodes, 0);
  ASSERT_TRUE(out.linearization_ratio_defined);
  out.infeasible_group = DockingRowGroup::kLateral;
  out.violation.fill(0.25);
  out.slack_c.fill(0.25);
  out.slack_v.fill(0.25);

  MpcDockingSegmentCoreInput bad = cases[0].in;
  bad.ball[static_cast<std::size_t>(core.CatchNode())].cov_valid = false;
  ASSERT_FALSE(core.Solve(bad, out));
  EXPECT_EQ(out.reason, MpcDockingReason::kCovarianceInvalid);
  EXPECT_EQ(out.cost.total, 0.0);
  EXPECT_EQ(out.cost.reference, 0.0);
  EXPECT_EQ(out.c_catch, 0.0);
  EXPECT_EQ(out.sigma_s, 0.0);
  EXPECT_EQ(out.sigma_t, 0.0);
  EXPECT_EQ(out.tau_ratio_max, 0.0);
  EXPECT_EQ(out.approach_nodes, 0);
  EXPECT_FALSE(out.c_guarded);
  EXPECT_FALSE(out.linearization_ratio_defined);
  EXPECT_EQ(out.linearization_ratio, 0.0);
  EXPECT_EQ(out.infeasible_group, DockingRowGroup::kTorque);
  for (const double v : out.violation) {
    EXPECT_EQ(v, 0.0);
  }
  for (const double v : out.mu) {
    EXPECT_EQ(v, 0.0);
  }
  for (std::size_t i = 0; i < out.slack_c.size(); ++i) {
    EXPECT_EQ(out.slack_c[i], 0.0);
    EXPECT_EQ(out.slack_v[i], 0.0);
  }
  // Evaluate() rejects the same way.
  ASSERT_TRUE(core.Solve(cases[0].in, out));
  MpcDockingSegmentCoreInput no_trajectory = cases[0].in;
  no_trajectory.initial_valid = false;
  ASSERT_FALSE(core.Evaluate(no_trajectory, out));
  EXPECT_EQ(out.cost.total, 0.0);
  EXPECT_EQ(out.mu[0], 0.0);
}

// The rule a penalty growth step is kept by, on the cases a solve rarely
// reaches. (elastic totals and the violation are sums over the groups, in the
// rows' own units.)
TEST(MpcDockingSegmentCore, PenaltyGrowthIsKeptOnlyWhenItBuysLinearisedFeasibility) {
  using rtc::catching::DockingPenaltyGrowthKept;
  constexpr double kGain = 0.1;
  constexpr double kTol = 1e-6;
  // The elastic total falls by the gain: kept, wherever the iterate is. (In
  // the first case the step already removed most of the violation, 0.80 of
  // 1.0, and removes 4 % more — only the elastic's own 15 % drop keeps it.)
  EXPECT_TRUE(DockingPenaltyGrowthKept(1.0, 0.2, 0.17, kGain, kTol));
  EXPECT_TRUE(DockingPenaltyGrowthKept(0.0, 1e-2, 0.0, kGain, kTol));
  // Neither the elastic (−1.25 %) nor what the step removes (+5 %) gains.
  EXPECT_FALSE(DockingPenaltyGrowthKept(0.5, 0.4, 0.395, kGain, kTol));
  // Under a trust region the total barely moves; what counts is that the step
  // removes more of the violation: 0.05 → 0.06 is +20 %, 0.05 → 0.052 is +4 %.
  EXPECT_TRUE(DockingPenaltyGrowthKept(0.5, 0.45, 0.44, kGain, kTol));
  EXPECT_FALSE(DockingPenaltyGrowthKept(0.5, 0.45, 0.448, kGain, kTol));
  // A step that removed nothing and now removes something.
  EXPECT_TRUE(DockingPenaltyGrowthKept(0.5, 0.5, 0.48, kGain, kTol));
  // … but not by less than the tolerance the rows are judged to.
  EXPECT_FALSE(DockingPenaltyGrowthKept(0.5, 0.5, 0.5 - 0.5 * kTol, kGain, kTol));
  // A near-feasible iterate whose QP steps INTO a violation (the elastic
  // exceeds the violation, so the step "removes" a negative amount). A 1 %
  // drop of the elastic is not a gain; measured against the negative amount
  // it would pass, at every growth step, up to the cap.
  EXPECT_FALSE(DockingPenaltyGrowthKept(0.0, 1e-2, 0.99e-2, kGain, kTol));
  EXPECT_FALSE(DockingPenaltyGrowthKept(1e-3, 1e-2, 0.95e-2, kGain, kTol));
  // No change, or worse: never.
  EXPECT_FALSE(DockingPenaltyGrowthKept(0.5, 0.4, 0.4, kGain, kTol));
  EXPECT_FALSE(DockingPenaltyGrowthKept(0.5, 0.4, 0.45, kGain, kTol));
}

// A trust region smaller than the box tolerance must not cross the box rows
// (lower > upper would make the QP infeasible), and a usable one still
// converges — in more, smaller steps.
TEST(MpcDockingSegmentCore, TrustRegionNeverMakesTheQpInfeasible) {
  for (const double delta_tr : {1e-9, 0.05}) {
    dk::Rig rig = dk::MakeRig(fx::RealArm7());
    rig.params.delta_tr = delta_tr;
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult out;
    core.ResizeResult(out);
    for (Case& c : FeasibleCases(rig, core, 3)) {
      dk::PerturbTarget(c.in, c.seed);
      ASSERT_TRUE(core.Solve(c.in, out));
      EXPECT_NE(out.reason, MpcDockingReason::kQpFailed) << delta_tr << " " << Describe(out);
      EXPECT_LE(out.violation[G(DockingRowGroup::kBox)], kViolationTol) << Describe(out);
      if (delta_tr > 1e-3) {
        EXPECT_TRUE(out.converged) << Describe(out);
        // At the solution the step is zero: no trust region rests on it.
        EXPECT_FALSE(out.step_capped) << Describe(out);
      } else {
        EXPECT_FALSE(out.converged) << "a 1e-9 rad step cannot reach the catch in 50 iterations";
        // … and that is ALL it says: the problem is feasible, the step is
        // capped. A stall or a small ‖H d‖ under a binding trust region is
        // evidence about the cap, not about the rows.
        EXPECT_EQ(out.reason, MpcDockingReason::kIterationLimit) << Describe(out);
        EXPECT_TRUE(out.step_capped) << Describe(out);
        EXPECT_EQ(out.iterations, rig.params.max_iterations);
      }
    }
  }
}

// Why the core turns ProxQP's primal-infeasibility test off. Every QP here is
// feasible by construction, yet with ProxQP's default threshold the solver
// reports PRIMAL_INFEASIBLE on one of these — at the default penalty, under a
// 0.05 rad trust region. This is a control on the SOLVER's behaviour: if it
// stops failing (another ProxQP version), the setting may no longer be needed
// — look again rather than deleting this test.
TEST(MpcDockingSegmentCore, DefaultInfeasibilityThresholdMisreportsAFeasibleQp) {
  dk::Rig rig = dk::MakeRig(fx::RealArm7());
  rig.params.delta_tr = 0.05;
  rig.params.solver.eps_primal_inf = 1e-4;  // ProxQP's default
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  int misreported = 0;
  for (Case& c : FeasibleCases(rig, core, 3)) {
    dk::PerturbTarget(c.in, c.seed);
    ASSERT_TRUE(core.Solve(c.in, out));
    constexpr int kPrimalInfeasible = 2;  // proxsuite::proxqp::QPSolverOutput
    misreported +=
        (out.reason == MpcDockingReason::kQpFailed && out.qp_status == kPrimalInfeasible) ? 1 : 0;
  }
  EXPECT_GT(misreported, 0)
      << "ProxQP no longer misreports this QP — eps_primal_inf = 0 may be unnecessary now";
}

// The longest stall window (the whole history ring) must still compare with
// the PAST: a feasible throw is not cut short as "stalled".
TEST(MpcDockingSegmentCore, LongestStallWindowStillComparesWithThePast) {
  dk::Rig rig = dk::MakeRig(fx::RealArm6());
  rig.params.stall_window = rtc::catching::kDockingStallHistory;
  rig.params.delta_tr = 0.02;  // many small steps: well past 16 iterations
  rig.params.mu_init.fill(1e-2);
  rig.params.max_iterations = 200;
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  int long_runs = 0;
  for (Case& c : FeasibleCases(rig, core, 5)) {
    dk::PerturbTarget(c.in, c.seed);
    ASSERT_TRUE(core.Solve(c.in, out));
    EXPECT_NE(out.reason, MpcDockingReason::kInfeasible) << Describe(out);
    EXPECT_TRUE(out.feasible) << Describe(out);
    long_runs += out.iterations > rtc::catching::kDockingStallHistory ? 1 : 0;
  }
  EXPECT_GT(long_runs, 0) << "no solve ran past the window — the test does not reach the ring";
}

// A covariance whose quadratic forms overflow gives non-finite row values. A
// NaN must not vanish inside a max and come back as "no violation".
TEST(MpcDockingSegmentCore, NonFiniteRowValuesAreNotReportedFeasible) {
  const dk::Rig rig = dk::MakeRig(fx::RealArm6());
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  MpcDockingSegmentCoreInput in = cases[0].in;
  in.ball[static_cast<std::size_t>(core.CatchNode())].cov =
      1e200 * BallCovariance::Identity();  // finite, PSD — and its forms overflow
  const bool returned = core.Solve(in, out);
  EXPECT_FALSE(out.feasible) << Describe(out);
  EXPECT_FALSE(out.converged);
  // … nor as rows with room: a maximum over the faces would drop a NaN face
  // and report the room of the others.
  EXPECT_FALSE(out.lateral_margin > 0.0) << out.lateral_margin;
  EXPECT_FALSE(out.timing_margin > 0.0) << out.timing_margin;
  if (returned) {
    EXPECT_TRUE(out.reason == MpcDockingReason::kSolutionNonFinite ||
                out.reason == MpcDockingReason::kInfeasible ||
                out.reason == MpcDockingReason::kQpFailed)
        << Describe(out);
  }
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

// ── The catch instant as a variable (E1-F14 PR 2, #740) ──────────────────────
// A core with catch_time_variable on a throw whose catch instant the test
// knows: MakeThrow's constructed trajectory catches the ball at `t_true_ns`,
// the prediction is that ball's (under gravity), and the core is anchored
// `shift_ns` BEFORE that instant — so the constructed catch is at
// δt_c = shift_ns, and a core that could not move the catch instant would
// have to catch another ball than the one the trajectory was built for.

constexpr std::int64_t kTcMs = 1'000'000;
constexpr std::int64_t kTrueCatchNs = 1'727'000'000'123'456'789LL;  // a realistic steady ns

struct TcCase {
  dk::Rig rig;  // catch_time_variable on
  dk::Throw th;
  rtc::catching::TrajectorySnapshot traj{};
  MpcDockingSegmentCoreInput in;
  std::int64_t t_hat_ns{0};
  unsigned seed{0};
};

dk::Rig WithCatchTime(dk::Rig rig) {
  rig.params.catch_time_variable = true;
  rig.params.delta_t_step = 0.02;
  return rig;
}

// The first feasible seed from `from`, anchored `shift_ns` before the true
// catch instant, with δt_c free in ±`half_box_ns` and starting at 0.
std::unique_ptr<TcCase> MakeTcCase(const dk::Rig& fixed_rig, unsigned from, std::int64_t shift_ns,
                                   std::int64_t half_box_ns) {
  auto c = std::make_unique<TcCase>();
  // The throw is built on a core WITHOUT the variable: its gains are the
  // δt_c = 0 grid's whatever was solved (mpc_docking_fixture.hpp).
  dk::Rig plain = fixed_rig;
  plain.params.catch_time_variable = false;
  MpcDockingSegmentCore ref;
  if (ref.Init(plain.model, plain.arm.frame, plain.params, plain.limits, &NoClock) !=
      MpcDockingReason::kNone) {
    ADD_FAILURE() << "the reference core did not initialise";
    return c;
  }
  bool found = false;
  for (unsigned seed = from; !found && seed < from + 400; ++seed) {
    found = dk::MakeThrow(plain, ref, seed, 1e-3, c->th, c->in);
    c->seed = seed;
  }
  EXPECT_TRUE(found) << "no feasible throw";
  c->rig = WithCatchTime(fixed_rig);
  // 40 samples, 20 ms apart, from 0.6 s before the true catch instant.
  c->traj = dk::BallisticPrediction(c->th.p_b, c->th.v_b, dk::kGravity, kTrueCatchNs,
                                    kTrueCatchNs - 600 * kTcMs, 20 * kTcMs, 40);
  c->t_hat_ns = kTrueCatchNs - shift_ns;
  dk::FillBallAt(c->rig.params, c->traj, c->th.cov, c->t_hat_ns, c->in);
  c->in.delta_start_ns = 0;
  c->in.delta_lo_ns = -half_box_ns;
  c->in.delta_hi_ns = half_box_ns;
  return c;
}

// The core's objective: what it minimises, apart from the penalties.
double Objective(const MpcDockingSegmentCoreResult& r) {
  return r.cost.total + r.cost.time;
}

std::string DescribeTc(const MpcDockingSegmentCoreResult& r) {
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                " delta=%.6f ms moves=%d dL/dtheta=%.3e time=%.4f e_box=%.2e e_term=%.2e "
                "capped=%d/%d",
                static_cast<double>(r.delta_ns) * 1e-6, r.catch_time_steps, r.catch_time_gradient,
                r.cost.time, r.elastic_post_box, r.elastic_terminal, r.capped_iterations,
                r.iterations);
  return Describe(r) + buf;
}

// The start trajectory of `z` on the grid stretched by `delta_ns`.
void SetStartAt(const TcCase& c, const Eigen::VectorXd& z, std::int64_t delta_ns,
                MpcDockingSegmentCoreInput& in) {
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(c.th.q0.size());
  const dk::Nodes nodes = dk::NodesFromJerkAt(c.rig.params, c.th.q0, zero, zero, z,
                                              static_cast<double>(delta_ns) / 1e9);
  in.q_init = nodes.q;
  in.qd_init = nodes.qd;
  in.qdd_init = nodes.qdd;
  in.initial_valid = true;
  in.delta_start_ns = delta_ns;
}

TEST(MpcDockingSegmentCoreCatchTime, GradientAndRowsMatchFiniteDifferencesInZAndInTheCatchInstant) {
  for (const fx::ArmModel& arm : {fx::RealArm6(), fx::RealArm7()}) {
    dk::Rig base_rig = FullCostRig(arm);
    // The start is taken as given whatever it leaves of the (elastic) rest
    // and box; a finite trust region far from binding, so that its rows exist.
    base_rig.params.tol_linear = 0.5;
    base_rig.params.delta_tr = 50.0;
    const std::unique_ptr<TcCase> c = MakeTcCase(base_rig, 3, 0, 30 * kTcMs);
    const dk::Rig& rig = c->rig;
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
              MpcDockingReason::kNone);
    ASSERT_TRUE(core.CatchTimeVariable());
    MpcDockingSegmentCoreResult out;
    core.ResizeResult(out);
    MpcDockingSegmentCoreInput in = c->in;
    in.p_line = c->th.p_b + Eigen::Vector3d(0.02, -0.03, 0.01);
    in.d_line = Eigen::Vector3d(0.3, -0.5, 0.8).normalized();
    in.time_c1 = 0.7;
    in.time_c2 = 40.0;
    in.delta_ref_ns = -4 * kTcMs;
    const Eigen::Index n = core.Nv();
    const Eigen::Index nu = core.NumJerkVariables();
    const Eigen::Index th = core.CatchTimeColumn();
    ASSERT_EQ(th, nu);
    const int kc = core.CatchNode();
    const int N = core.NumNodes();
    const double da = rig.params.dt_pre;
    // The base point: the constructed trajectory pushed off its optimum, at a
    // catch instant that is NOT the anchor.
    const std::int64_t delta0_ns = 6 * kTcMs + 137;
    Eigen::VectorXd z0;
    {
      MpcDockingSegmentCore ref;
      dk::Rig plain = rig;
      plain.params.catch_time_variable = false;
      ASSERT_EQ(ref.Init(plain.model, plain.arm.frame, plain.params, plain.limits, &NoClock),
                MpcDockingReason::kNone);
      z0 = dk::JerkThrough(ref, c->th.q0, c->th.q_c, c->th.v_c);
      // The push stays in the null space of the δt_c = 0 grid's terminal
      // equality, as in the test without the variable: what the start then
      // leaves of the rest is what the stretched interval adds.
      const int nb = ref.NumBlocks();
      Eigen::MatrixXd m(2, nb);
      for (int b = 0; b < nb; ++b) {
        m(0, b) = ref.StageGain(1, N, b);
        m(1, b) = ref.StageGain(2, N, b);
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
    // The stretched interval's jerk is what makes ∂x_kc/∂δ's third entry.
    const Eigen::Index catch_block = kc - 1;  // FullCostRig: one block per pre-catch interval
    ASSERT_GT(z0.segment(catch_block * n, n).cwiseAbs().maxCoeff(), 1e-3);
    SetStartAt(*c, z0, delta0_ns, in);
    ASSERT_TRUE(core.Solve(in, out)) << DescribeTc(out);
    ASSERT_FALSE(out.init_qp_used) << "the start must be taken as given";
    const rtc::tsid::QPData base = core.LastQp();  // copy

    // ── The cost: central differences of J's smooth part, in z and in δt_c ──
    const auto smooth_cost = [&](const Eigen::VectorXd& z, std::int64_t delta_ns) {
      SetStartAt(*c, z, delta_ns, in);
      EXPECT_TRUE(core.Evaluate(in, out));
      EXPECT_EQ(out.delta_ns, delta_ns);
      return out.cost.total - out.cost.slack + out.cost.time;
    };
    const double h = 1e-6;
    const std::int64_t h_ns = 2000;
    const double h_s = static_cast<double>(h_ns) / 1e9;
    double worst_grad = 0.0;
    for (Eigen::Index i = 0; i < nu; ++i) {
      Eigen::VectorXd zp = z0;
      Eigen::VectorXd zm = z0;
      zp[i] += h;
      zm[i] -= h;
      const double fd = (smooth_cost(zp, delta0_ns) - smooth_cost(zm, delta0_ns)) / (2.0 * h);
      EXPECT_NEAR(base.g[i], fd, 2e-6 * std::max(1.0, std::abs(fd)))
          << arm.name << " variable " << i;
      worst_grad = std::max(worst_grad, std::abs(base.g[i] - fd));
    }
    // θ = δt_c/Δ_a: ∂J/∂θ = Δ_a ∂J/∂δt_c.
    const double fd_theta =
        da * (smooth_cost(z0, delta0_ns + h_ns) - smooth_cost(z0, delta0_ns - h_ns)) / (2.0 * h_s);
    EXPECT_NEAR(base.g[th], fd_theta, 2e-6 * std::max(1.0, std::abs(fd_theta))) << arm.name;
    EXPECT_GT(std::abs(fd_theta), 1e-3) << arm.name << ": the cost does not move with δt_c";
    RecordSci(arm.name + "_tc_gradient_fd_max_abs_diff", worst_grad);
    RecordSci(arm.name + "_tc_theta_gradient", base.g[th]);
    RecordSci(arm.name + "_tc_theta_gradient_fd_abs_diff", std::abs(base.g[th] - fd_theta));

    // ── Rows: a row's bound moves by minus the row's coefficients ──
    struct Check {
      int row;
      bool upper;
      bool before_catch;  // a row of a node k < k_c: no θ column, exactly
      bool after_catch;   // a row whose θ column must be exercised
    };

    std::vector<Check> checks;
    const int nN = static_cast<int>(n) * N;
    const int tau0 = core.GroupRowBegin(DockingRowGroup::kTorque);
    for (int k = 1; k <= N; ++k) {
      for (int j = 0; j < static_cast<int>(n); ++j) {
        const int r = (k - 1) * static_cast<int>(n) + j;
        checks.push_back({tau0 + r, true, k < kc, k >= kc});
        checks.push_back({tau0 + nN + r, false, k < kc, k >= kc});
      }
    }
    const int app0 = core.GroupRowBegin(DockingRowGroup::kGap);
    SetStartAt(*c, z0, delta0_ns, in);
    ASSERT_TRUE(core.Evaluate(in, out));
    ASSERT_GT(core.NumApproachNodes(), 0);
    for (int i = 0; i < core.NumApproachNodes(); ++i) {
      checks.push_back({app0 + 3 * i, false, true, false});
      if (out.slack_c[static_cast<std::size_t>(i)] == 0.0) {
        checks.push_back({app0 + 3 * i + 1, true, true, false});
      }
      checks.push_back({app0 + 3 * i + 2, true, true, false});
    }
    // The hard box rows of the nodes before the catch node (q, q̇, q̈).
    const int box0 = core.GroupRowBegin(DockingRowGroup::kBox);
    for (int block = 0; block < 3; ++block) {
      for (int r = 0; r < (kc - 1) * static_cast<int>(n); ++r) {
        checks.push_back({box0 + block * nN + r, true, true, false});
        checks.push_back({box0 + block * nN + r, false, true, false});
      }
    }
    const int ent0 = core.GroupRowBegin(DockingRowGroup::kEntrance);
    checks.push_back({ent0, true, false, true});
    checks.push_back({ent0 + 1, false, false, true});
    const int lat0 = core.GroupRowBegin(DockingRowGroup::kLateral);
    for (int i = 0; i < core.GroupRowCount(DockingRowGroup::kLateral); ++i) {
      checks.push_back({lat0 + i, true, false, true});
    }
    checks.push_back({core.GroupRowBegin(DockingRowGroup::kTiming), false, false, true});
    const int vel0 = core.GroupRowBegin(DockingRowGroup::kVelocitySet);
    checks.push_back({vel0, false, false, true});
    for (int i = 1; i < core.GroupRowCount(DockingRowGroup::kVelocitySet); ++i) {
      checks.push_back({vel0 + i, true, false, true});
    }
    const int imp0 = core.GroupRowBegin(DockingRowGroup::kImpact);
    for (int i = 0; i < 3; ++i) {
      checks.push_back({imp0 + i, true, false, true});
    }
    // The elastic box of the nodes k ≥ k_c and the elastic terminal rest.
    const int pbox0 = core.PostCatchBoxRowBegin();
    const int pbox_n = core.PostCatchBoxRowCount();
    ASSERT_EQ(pbox_n, (N - kc + 1) * static_cast<int>(n) * 3);
    for (int r = 0; r < pbox_n; ++r) {
      checks.push_back({pbox0 + r, true, false, true});
      checks.push_back({pbox0 + pbox_n + r, false, false, true});
    }
    const int term0 = core.TerminalRowBegin();
    for (int r = 0; r < 2 * static_cast<int>(n); ++r) {
      checks.push_back({term0 + r, true, false, true});
      checks.push_back({term0 + 2 * static_cast<int>(n) + r, false, false, true});
    }
    for (const Check& ck : checks) {
      ASSERT_TRUE(std::isfinite(ck.upper ? base.u[ck.row] : base.l[ck.row])) << "row " << ck.row;
    }
    const auto bounds_at = [&](const Eigen::VectorXd& z, std::int64_t delta_ns) {
      SetStartAt(*c, z, delta_ns, in);
      EXPECT_TRUE(core.Solve(in, out));
      EXPECT_FALSE(out.init_qp_used);
      return std::make_pair(Eigen::VectorXd(core.LastQp().l), Eigen::VectorXd(core.LastQp().u));
    };
    Eigen::MatrixXd fd_rows(static_cast<Eigen::Index>(checks.size()), nu + 1);
    for (Eigen::Index i = 0; i <= nu; ++i) {
      Eigen::VectorXd zp = z0;
      Eigen::VectorXd zm = z0;
      std::int64_t dp = delta0_ns;
      std::int64_t dm = delta0_ns;
      double step = h;
      if (i < nu) {
        zp[i] += h;
        zm[i] -= h;
      } else {
        dp += h_ns;
        dm -= h_ns;
        step = h_s / da;  // in θ
      }
      const auto [lp, up] = bounds_at(zp, dp);
      const auto [lm, um] = bounds_at(zm, dm);
      for (std::size_t r = 0; r < checks.size(); ++r) {
        const int row = checks[r].row;
        const double dbound = checks[r].upper ? (up[row] - um[row]) / (2.0 * step)
                                              : (lp[row] - lm[row]) / (2.0 * step);
        fd_rows(static_cast<Eigen::Index>(r), i) = -dbound;
      }
    }
    double worst_row = 0.0;
    double worst_theta = 0.0;
    int theta_exercised = 0;
    int theta_expected = 0;
    for (std::size_t r = 0; r < checks.size(); ++r) {
      const Check& ck = checks[r];
      const Eigen::RowVectorXd coeff = base.C.row(ck.row).head(nu + 1);
      const Eigen::RowVectorXd fd = fd_rows.row(static_cast<Eigen::Index>(r));
      const double scale = std::max(1.0, fd.cwiseAbs().maxCoeff());
      const double diff = (coeff - fd).cwiseAbs().maxCoeff();
      EXPECT_LT(diff, 2e-5 * scale) << arm.name << " row " << ck.row;
      worst_row = std::max(worst_row, diff / scale);
      worst_theta = std::max(worst_theta, std::abs(coeff[th] - fd[th]) / scale);
      if (ck.before_catch) {
        // A node before the catch node does not move with the catch instant:
        // no θ column at all, and the row's bound does not move with δt_c.
        EXPECT_EQ(coeff[th], 0.0) << arm.name << " row " << ck.row;
        EXPECT_LT(std::abs(fd[th]), 1e-7) << arm.name << " row " << ck.row;
      }
      if (ck.after_catch) {
        ++theta_expected;
        theta_exercised += std::abs(fd[th]) > 1e-6 ? 1 : 0;
      }
    }
    // The θ column is exercised on (nearly) every row that should have one —
    // not all: the box rows of q̈ move at ∂q̈_k/∂δ = u_{kc−1} whatever k is,
    // but a torque row of one joint may not feel another's.
    EXPECT_GT(theta_exercised, (theta_expected * 9) / 10) << arm.name;
    // The trust region's rows on the nodes k ≥ k_c are the linearised
    // displacement of q_k — the elastic box's q rows, coefficient for
    // coefficient — bounded by ±δ.
    for (int k = kc; k <= N; ++k) {
      for (int j = 0; j < static_cast<int>(n); ++j) {
        const int trust = box0 + (k - 1) * static_cast<int>(n) + j;
        const int elastic = pbox0 + (k - kc) * 3 * static_cast<int>(n) + j;
        EXPECT_EQ(base.C.row(trust).head(nu + 1), base.C.row(elastic).head(nu + 1));
        EXPECT_EQ(base.l[trust], -rig.params.delta_tr);
        EXPECT_EQ(base.u[trust], rig.params.delta_tr);
      }
    }
    RecordSci(arm.name + "_tc_rows_fd_max_rel_diff", worst_row);
    RecordSci(arm.name + "_tc_theta_column_fd_max_rel_diff", worst_theta);
    ::testing::Test::RecordProperty(arm.name + "_tc_rows_checked", static_cast<int>(checks.size()));
    ::testing::Test::RecordProperty(arm.name + "_tc_theta_rows_exercised", theta_exercised);
  }
}

// With δt_c's box closed at 0 the problem is the fixed grid's: the two cores
// reach the same trajectory (the elastic rest and box are then just another
// way to write rows that hold).
TEST(MpcDockingSegmentCoreCatchTime, ABoxClosedAtZeroIsTheFixedGridProblem) {
  for (const dk::Rig& fixed_rig : Rigs()) {
    const std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, 1, 0, 0);
    MpcDockingSegmentCore fixed;
    MpcDockingSegmentCore moving;
    ASSERT_EQ(fixed.Init(fixed_rig.model, fixed_rig.arm.frame, fixed_rig.params, fixed_rig.limits,
                         &NoClock),
              MpcDockingReason::kNone);
    ASSERT_EQ(moving.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult a;
    MpcDockingSegmentCoreResult b;
    fixed.ResizeResult(a);
    moving.ResizeResult(b);
    MpcDockingSegmentCoreInput in = c->in;
    dk::PerturbTarget(in, 3);
    ASSERT_TRUE(fixed.Solve(in, a));
    ASSERT_TRUE(moving.Solve(in, b));
    ASSERT_TRUE(a.converged) << Describe(a);
    EXPECT_TRUE(b.converged) << DescribeTc(b);
    EXPECT_EQ(b.delta_ns, 0);
    EXPECT_EQ(b.cost.time, 0.0);
    EXPECT_LT((a.q - b.q).cwiseAbs().maxCoeff(), 1e-4) << fixed_rig.arm.name;
    EXPECT_NEAR(a.cost.total, b.cost.total, 1e-5 * std::max(1.0, a.cost.total));
    // Without the variable the result's fields for it stay at zero.
    EXPECT_EQ(a.delta_ns, 0);
    EXPECT_EQ(a.cost.time, 0.0);
    EXPECT_EQ(a.mu_post_box, 0.0);
    EXPECT_GT(b.mu_post_box, 0.0);
  }
}

// The fixed-grid solution of a case: δt_c's box closed at 0, from the IK
// target. What the search starts the continuous solve from.
bool SolveFixed(MpcDockingSegmentCore& core, const TcCase& c, MpcDockingSegmentCoreResult& out) {
  MpcDockingSegmentCoreInput in = c.in;
  in.delta_start_ns = 0;
  in.delta_lo_ns = 0;
  in.delta_hi_ns = 0;
  dk::PerturbTarget(in, c.seed);
  return core.Solve(in, out);
}

MpcDockingSegmentCoreInput StartFrom(const MpcDockingSegmentCoreInput& in,
                                     const MpcDockingSegmentCoreResult& from) {
  MpcDockingSegmentCoreInput next = in;
  next.initial_valid = true;
  next.q_init = from.q;
  next.qd_init = from.qd;
  next.qdd_init = from.qdd;
  next.delta_start_ns = from.delta_ns;
  return next;
}

// The hard rows of a returned iterate, re-evaluated outside the core with the
// ball read from the prediction at the returned catch instant.
std::array<double, kNumDockingRowGroups> ViolationsOutsideTheCore(
    const TcCase& c, const MpcDockingSegmentCore& core, const MpcDockingSegmentCoreResult& r) {
  MpcDockingSegmentCoreInput at = c.in;
  int hint = 0;
  const rtc::catching::BallNodeSample ball = rtc::catching::SampleBallNode(
      c.traj, nullptr, false, rtc::catching::BallTime{c.t_hat_ns + r.delta_ns}, hint);
  auto& slot = at.ball[static_cast<std::size_t>(core.CatchNode())];
  slot.p = ball.p;
  slot.v = ball.v;
  slot.a = ball.a;
  return dk::HardRowViolations(c.rig, core, NodesOf(r), at);
}

// The block jerk of a result, in the core's scaled variable.
Eigen::VectorXd JerkOf(const dk::Rig& rig, const MpcDockingSegmentCoreResult& r) {
  const Eigen::Index n = r.u.rows();
  Eigen::VectorXd z(n * rig.params.n_blocks);
  int k = 0;
  for (int b = 0; b < rig.params.n_blocks; ++b) {
    z.segment(b * n, n) = r.u.col(k) / rig.params.u_scale;
    k += rig.params.block_sizes[static_cast<std::size_t>(b)];
  }
  return z;
}

// An anchor off the instant the throw is best caught at: the catch instant
// moves, the objective falls, and what comes back is a trajectory of the
// stretched grid that holds every hard row at the instant it reports.
TEST(MpcDockingSegmentCoreCatchTime, TheCatchInstantMovesAndTheObjectiveDoesNotRise) {
  for (const dk::Rig& fixed_rig : Rigs()) {
    for (const std::int64_t shift_ns : {7 * kTcMs + 311, -9 * kTcMs - 77}) {
      const std::string where =
          fixed_rig.arm.name + " shift " + std::to_string(shift_ns / kTcMs) + " ms";
      const std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, 1, shift_ns, 20 * kTcMs);
      MpcDockingSegmentCore core;
      ASSERT_EQ(core.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &NoClock),
                MpcDockingReason::kNone);
      MpcDockingSegmentCoreResult fixed;
      MpcDockingSegmentCoreResult moved;
      core.ResizeResult(fixed);
      core.ResizeResult(moved);
      ASSERT_TRUE(SolveFixed(core, *c, fixed)) << where;
      // The anchor is off: the fixed grid may or may not hold every row.
      std::printf("[ record ] %s fixed: %s\n", where.c_str(), DescribeTc(fixed).c_str());
      const MpcDockingSegmentCoreInput in = StartFrom(c->in, fixed);
      ASSERT_TRUE(core.Solve(in, moved)) << where;
      std::printf("[ record ] %s moved: %s\n", where.c_str(), DescribeTc(moved).c_str());
      EXPECT_FALSE(moved.init_qp_used) << where << ": the fixed solution is a start";
      EXPECT_TRUE(moved.converged) << where << DescribeTc(moved);
      EXPECT_TRUE(moved.feasible) << where;
      // The catch instant moved — by more than the lattice could have told —
      // and stayed inside the box. (Not necessarily toward the instant the
      // throw was built for: that trajectory is feasible, not optimal.)
      EXPECT_GE(std::abs(moved.delta_ns), kTcMs) << where;
      EXPECT_GE(moved.delta_ns, in.delta_lo_ns);
      EXPECT_LE(moved.delta_ns, in.delta_hi_ns);
      EXPECT_GT(moved.catch_time_steps, 0) << where;
      ASSERT_TRUE(fixed.converged) << where;
      EXPECT_LT(Objective(moved), Objective(fixed) - 1e-4) << where;
      // It IS a minimum over the catch instant: the same problem solved with
      // the instant held 1 ms either side costs more (no multiplier is read
      // for this — two more solves and three numbers). At an end of the box
      // only the inner side exists.
      for (const std::int64_t off : {-kTcMs, kTcMs}) {
        const std::int64_t at = moved.delta_ns + off;
        if (at < in.delta_lo_ns || at > in.delta_hi_ns) {
          continue;
        }
        MpcDockingSegmentCoreInput held = StartFrom(c->in, moved);
        held.delta_lo_ns = at;
        held.delta_hi_ns = at;
        MpcDockingSegmentCoreResult beside;
        core.ResizeResult(beside);
        // The start is the neighbouring instant's trajectory: the node
        // matrices are taken as nodes of THIS grid (delta_start_ns says so).
        held.delta_start_ns = at;
        ASSERT_TRUE(core.Solve(held, beside)) << where;
        ASSERT_TRUE(beside.converged) << where << DescribeTc(beside);
        EXPECT_EQ(beside.delta_ns, at);
        EXPECT_GE(Objective(beside), Objective(moved) - 1e-7) << where << " " << off;
      }
      // The nodes are the stretched grid's, integrated from the returned jerk.
      const Eigen::VectorXd zero = Eigen::VectorXd::Zero(core.Nv());
      const dk::Nodes nodes =
          dk::NodesFromJerkAt(c->rig.params, c->th.q0, zero, zero, JerkOf(c->rig, moved),
                              static_cast<double>(moved.delta_ns) / 1e9);
      EXPECT_LT((nodes.q - moved.q).cwiseAbs().maxCoeff(), 1e-10) << where;
      EXPECT_LT((nodes.qd - moved.qd).cwiseAbs().maxCoeff(), 1e-9) << where;
      EXPECT_LT((nodes.qdd - moved.qdd).cwiseAbs().maxCoeff(), 1e-8) << where;
      // Every hard row, outside the core, at the reported instant.
      const std::array<double, kNumDockingRowGroups> viol =
          ViolationsOutsideTheCore(*c, core, moved);
      for (int g = 0; g < kNumDockingRowGroups; ++g) {
        const double limit =
            g == static_cast<int>(DockingRowGroup::kTerminal) ? kTerminalRestTol : kViolationTol;
        EXPECT_LE(viol[static_cast<std::size_t>(g)], limit)
            << where << " " << DockingRowGroupName(static_cast<DockingRowGroup>(g));
      }
      // A restart from its own solution converges at once and gives it back.
      MpcDockingSegmentCoreResult again;
      core.ResizeResult(again);
      ASSERT_TRUE(core.Solve(StartFrom(c->in, moved), again));
      EXPECT_FALSE(again.init_qp_used) << where;
      EXPECT_TRUE(again.converged) << where << DescribeTc(again);
      EXPECT_EQ(again.iterations, 1) << where;
      EXPECT_EQ(again.delta_ns, moved.delta_ns) << where;
      EXPECT_LT((again.q - moved.q).cwiseAbs().maxCoeff(), 1e-9) << where;
      RecordSci(where + " delta_ms", static_cast<double>(moved.delta_ns) * 1e-6);
      RecordSci(where + " objective_fixed", Objective(fixed));
      RecordSci(where + " objective_moved", Objective(moved));
      ::testing::Test::RecordProperty(where + " iterations", moved.iterations);
    }
  }
}

// What the catch instant is moved on: ∂L/∂θ at a settled fixed-instant
// problem is the derivative of that problem's OPTIMAL VALUE in θ (the
// envelope theorem). Checked against the value itself: the problem solved
// with the instant held either side of 0, and a central difference of the two
// objectives.
TEST(MpcDockingSegmentCoreCatchTime, TheReportedDerivativeIsTheOptimalValuesSlope) {
  for (const dk::Rig& fixed_rig : Rigs()) {
    const std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, 1, 7 * kTcMs + 311, 0);
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult at;
    core.ResizeResult(at);
    MpcDockingSegmentCoreInput in = c->in;
    in.time_c1 = 0.4;
    in.time_c2 = 25.0;
    in.delta_ref_ns = 3 * kTcMs;
    dk::PerturbTarget(in, c->seed);
    ASSERT_TRUE(core.Solve(in, at));
    ASSERT_TRUE(at.converged) << DescribeTc(at);
    // Held by the closed box, the derivative is not a residual …
    EXPECT_EQ(at.catch_time_steps, 0);
    EXPECT_LE(at.kkt_residual, c->rig.params.tol_kkt * std::max(1.0, at.grad_norm));
    // … but it is far from zero here, or this test would compare two noises.
    ASSERT_GT(std::abs(at.catch_time_gradient), 1e-2) << DescribeTc(at);
    const std::int64_t h_ns = 500'000;
    std::array<double, 2> value{};
    for (int side = 0; side < 2; ++side) {
      const std::int64_t delta = side == 0 ? -h_ns : h_ns;
      MpcDockingSegmentCoreInput held = StartFrom(in, at);
      held.delta_start_ns = delta;
      held.delta_lo_ns = delta;
      held.delta_hi_ns = delta;
      MpcDockingSegmentCoreResult beside;
      core.ResizeResult(beside);
      ASSERT_TRUE(core.Solve(held, beside));
      ASSERT_TRUE(beside.converged) << DescribeTc(beside);
      value[static_cast<std::size_t>(side)] = Objective(beside);
    }
    // θ = δt_c/Δ_a.
    const double slope =
        c->rig.params.dt_pre * (value[1] - value[0]) / (2.0 * static_cast<double>(h_ns) / 1e9);
    EXPECT_NEAR(at.catch_time_gradient, slope, 2e-3 * std::max(1.0, std::abs(slope)))
        << fixed_rig.arm.name << DescribeTc(at);
    RecordSci(fixed_rig.arm.name + "_dvalue_dtheta", at.catch_time_gradient);
    RecordSci(fixed_rig.arm.name + "_dvalue_dtheta_fd", slope);
  }
}

// δt_c's box is held exactly, and one move is at most delta_t_step.
TEST(MpcDockingSegmentCoreCatchTime, TheBoxIsHeldAndAMoveIsAtMostTheStepLimit) {
  for (const dk::Rig& fixed_rig : Rigs()) {
    // Free in ±20 ms first: where the catch instant goes on its own.
    const std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, 1, 7 * kTcMs + 311, 20 * kTcMs);
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult fixed;
    MpcDockingSegmentCoreResult free_run;
    core.ResizeResult(fixed);
    core.ResizeResult(free_run);
    ASSERT_TRUE(SolveFixed(core, *c, fixed));
    ASSERT_TRUE(core.Solve(StartFrom(c->in, fixed), free_run));
    ASSERT_TRUE(free_run.converged) << DescribeTc(free_run);
    ASSERT_GE(std::abs(free_run.delta_ns), 2 * kTcMs) << DescribeTc(free_run);
    const std::int64_t toward = free_run.delta_ns > 0 ? 1 : -1;

    // A box that ends 1.5 ms toward that instant: the answer is the box's end,
    // and it is an answer (the box's multiplier is the problem's).
    MpcDockingSegmentCoreInput boxed = StartFrom(c->in, fixed);
    boxed.delta_lo_ns = -1'500'017;
    boxed.delta_hi_ns = 1'500'017;
    MpcDockingSegmentCoreResult held;
    core.ResizeResult(held);
    ASSERT_TRUE(core.Solve(boxed, held));
    EXPECT_TRUE(held.converged) << DescribeTc(held);
    EXPECT_EQ(held.delta_ns, toward * 1'500'017) << DescribeTc(held);
    EXPECT_GT(-toward * held.catch_time_gradient,
              c->rig.params.tol_kkt * std::max(1.0, held.grad_norm))
        << "the derivative must still point out of the box";
    EXPECT_LE(held.kkt_residual, c->rig.params.tol_kkt * std::max(1.0, held.grad_norm));
    EXPECT_LT(Objective(held), Objective(fixed));
    EXPECT_GT(Objective(held), Objective(free_run));

    // A step limit of 0.5 ms: after m moves the instant is within m steps of
    // its start, whatever the iteration budget cut the solve at — and the
    // full solve ends at the instant the unlimited one found.
    dk::Rig slow_rig = c->rig;
    slow_rig.params.delta_t_step = 0.0005;
    for (const int budget : {3, 6, 12, 200}) {
      slow_rig.params.max_iterations = budget;
      MpcDockingSegmentCore slow;
      ASSERT_EQ(
          slow.Init(slow_rig.model, slow_rig.arm.frame, slow_rig.params, slow_rig.limits, &NoClock),
          MpcDockingReason::kNone);
      MpcDockingSegmentCoreResult out;
      slow.ResizeResult(out);
      ASSERT_TRUE(slow.Solve(StartFrom(c->in, fixed), out));
      EXPECT_LE(std::abs(out.delta_ns), static_cast<std::int64_t>(out.catch_time_steps) * 500'000)
          << budget << DescribeTc(out);
      if (budget == 3) {
        EXPECT_GT(out.catch_time_steps, 0) << DescribeTc(out);
        EXPECT_FALSE(out.converged) << "three iterations cannot reach it at 0.5 ms a move";
        EXPECT_EQ(out.reason, MpcDockingReason::kIterationLimit);
      }
      if (budget == 200) {
        EXPECT_TRUE(out.converged) << DescribeTc(out);
        EXPECT_GE(out.catch_time_steps, 4);
        EXPECT_NEAR(static_cast<double>(out.delta_ns), static_cast<double>(free_run.delta_ns),
                    0.2 * static_cast<double>(kTcMs))
            << DescribeTc(out);
        EXPECT_NEAR(Objective(out), Objective(free_run), 1e-6);
      }
    }
  }
}

// One real-time iteration cannot move the catch instant: it takes the jerk's
// full step at the instant it started at.
TEST(MpcDockingSegmentCoreCatchTime, ASingleIterationStaysAtItsCatchInstant) {
  dk::Rig fixed_rig = dk::MakeRig(fx::RealArm7());
  fixed_rig.params.max_iterations = 1;
  const std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, 1, 7 * kTcMs, 20 * kTcMs);
  MpcDockingSegmentCore rti;
  ASSERT_EQ(rti.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  rti.ResizeResult(out);
  MpcDockingSegmentCoreInput in = c->in;
  in.delta_start_ns = 2 * kTcMs + 5;
  dk::PerturbTarget(in, 9);
  ASSERT_TRUE(rti.Solve(in, out));
  EXPECT_EQ(out.reason, MpcDockingReason::kIterationLimit);
  EXPECT_EQ(out.iterations, 1);
  EXPECT_EQ(out.delta_ns, 2 * kTcMs + 5);
  EXPECT_EQ(out.catch_time_steps, 0);
  EXPECT_GT(rti.LastQpSolution().head(rti.NumJerkVariables()).cwiseAbs().maxCoeff(), 1e-4);
  // θ's own entry of the step is pinned to zero.
  EXPECT_LT(std::abs(rti.LastQpSolution()[rti.CatchTimeColumn()]), 1e-9);
}

// A direction the objective is flat in: no time term, no near, terminal,
// impact or stop-line cost — what is left that reads the catch instant is the
// rows. Recorded, not judged on its count: how such a solve ends.
TEST(MpcDockingSegmentCoreCatchTime, RecordsASolveWhoseObjectiveDoesNotReadTheCatchInstant) {
  for (const fx::ArmModel& arm : {fx::RealArm6(), fx::RealArm7()}) {
    dk::Rig fixed_rig = dk::MakeRig(arm);
    fixed_rig.params.q_p.setZero();
    fixed_rig.params.q_v.setZero();
    fixed_rig.params.q_rho_f.setZero();
    fixed_rig.params.q_nu_f.setZero();
    const std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, 1, 5 * kTcMs, 20 * kTcMs);
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult fixed;
    MpcDockingSegmentCoreResult out;
    core.ResizeResult(fixed);
    core.ResizeResult(out);
    ASSERT_TRUE(SolveFixed(core, *c, fixed));
    ASSERT_TRUE(core.Solve(StartFrom(c->in, fixed), out));
    std::printf("[ record ] %s flat objective: %s\n", arm.name.c_str(), DescribeTc(out).c_str());
    EXPECT_TRUE(out.q.allFinite());
    EXPECT_GE(out.delta_ns, -20 * kTcMs);
    EXPECT_LE(out.delta_ns, 20 * kTcMs);
    if (out.converged && fixed.converged) {
      EXPECT_LE(Objective(out), Objective(fixed) + 1e-9);
    }
    ::testing::Test::RecordProperty(arm.name + "_flat_reason", MpcDockingReasonName(out.reason));
    ::testing::Test::RecordProperty(arm.name + "_flat_iterations", out.iterations);
    ::testing::Test::RecordProperty(arm.name + "_flat_moves", out.catch_time_steps);
    ::testing::Test::RecordProperty(arm.name + "_flat_capped_iterations", out.capped_iterations);
    RecordSci(arm.name + "_flat_delta_ms", static_cast<double>(out.delta_ns) * 1e-6);
  }
}

TEST(MpcDockingSegmentCoreCatchTime, RefusesWhatItCannotStartFrom) {
  const dk::Rig fixed_rig = dk::MakeRig(fx::RealArm6());
  const std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, 1, 0, 10 * kTcMs);
  // Init: the step limit and the two penalties are read with the flag only.
  {
    dk::Rig bad = c->rig;
    bad.params.delta_t_step = 0.0;
    MpcDockingSegmentCore core;
    EXPECT_EQ(core.Init(bad.model, bad.arm.frame, bad.params, bad.limits, &NoClock),
              MpcDockingReason::kParamsInvalid);
    bad = c->rig;
    bad.params.mu_init_terminal = 2.0 * bad.params.mu_max;
    EXPECT_EQ(core.Init(bad.model, bad.arm.frame, bad.params, bad.limits, &NoClock),
              MpcDockingReason::kParamsInvalid);
    bad = c->rig;
    bad.params.mu_init_post_box = 0.0;
    EXPECT_EQ(core.Init(bad.model, bad.arm.frame, bad.params, bad.limits, &NoClock),
              MpcDockingReason::kParamsInvalid);
    // Without the flag they are not read at all.
    bad.params.catch_time_variable = false;
    bad.params.delta_t_step = -1.0;
    EXPECT_EQ(core.Init(bad.model, bad.arm.frame, bad.params, bad.limits, &NoClock),
              MpcDockingReason::kNone);
    EXPECT_FALSE(core.CatchTimeVariable());
    EXPECT_EQ(core.CatchTimeColumn(), -1);
    EXPECT_EQ(core.PostCatchBoxRowCount(), 0);
  }
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  out.q.setConstant(7.0);
  const auto refused = [&](auto mutate, MpcDockingReason why, const char* what) {
    MpcDockingSegmentCoreInput in = c->in;
    mutate(in);
    EXPECT_FALSE(core.Solve(in, out)) << what;
    EXPECT_EQ(out.reason, why) << what << ": " << MpcDockingReasonName(out.reason);
    EXPECT_EQ(out.q(0, 0), 7.0) << what << ": the trajectory must stay untouched";
    EXPECT_EQ(out.delta_ns, 0) << what;
    EXPECT_FALSE(core.Evaluate(in, out)) << what;
  };
  refused([](MpcDockingSegmentCoreInput& in) { in.prediction = nullptr; },
          MpcDockingReason::kBallInvalid, "no prediction");
  rtc::catching::TrajectorySnapshot invalid = c->traj;
  invalid.valid = false;
  refused([&](MpcDockingSegmentCoreInput& in) { in.prediction = &invalid; },
          MpcDockingReason::kBallInvalid, "an invalid prediction");
  refused([](MpcDockingSegmentCoreInput& in) { in.delta_start_ns = in.delta_hi_ns + 1; },
          MpcDockingReason::kInputOutOfRange, "a start past the box");
  refused([](MpcDockingSegmentCoreInput& in) { in.delta_start_ns = in.delta_lo_ns - 1; },
          MpcDockingReason::kInputOutOfRange, "a start before the box");
  refused(
      [&](MpcDockingSegmentCoreInput& in) {
        in.delta_lo_ns = -static_cast<std::int64_t>(std::llround(c->rig.params.dt_pre * 1e9));
      },
      MpcDockingReason::kInputOutOfRange, "a box that empties the catch interval");
  refused([](MpcDockingSegmentCoreInput& in) { in.time_c2 = -1.0; },
          MpcDockingReason::kInputOutOfRange, "a negative curvature");
  refused([](MpcDockingSegmentCoreInput& in) { in.time_c1 = kNan; }, MpcDockingReason::kNonFinite,
          "a NaN slope");
  // The start instant past the end of the prediction: an extrapolated ball.
  refused(
      [&](MpcDockingSegmentCoreInput& in) {
        const std::int64_t last = c->traj.s[static_cast<std::size_t>(c->traj.n - 1)].t_ns;
        in.delta_hi_ns = last - c->t_hat_ns + 5 * kTcMs;
        in.delta_start_ns = in.delta_hi_ns;
      },
      MpcDockingReason::kBallInvalid, "a start past the prediction");
  // …and it still solves what it can start from.
  MpcDockingSegmentCoreInput in = c->in;
  dk::PerturbTarget(in, 2);
  ASSERT_TRUE(core.Solve(in, out));
  EXPECT_TRUE(out.converged) << DescribeTc(out);
}

// A box that reaches past the end of the prediction: the catch instant is
// drawn there (a reward for catching later), and the ball cannot be read
// there. The box ends at the prediction's last sample.
TEST(MpcDockingSegmentCoreCatchTime, TheBoxEndsWhereThePredictionDoes) {
  const dk::Rig fixed_rig = dk::MakeRig(fx::RealArm6());
  // Anchored 15 ms before the true catch instant; the prediction is cut at
  // the sample AT that instant, so it ends 15 ms into a ±20 ms box.
  std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, 1, 15 * kTcMs, 20 * kTcMs);
  int kept = 0;
  while (kept < c->traj.n && c->traj.s[static_cast<std::size_t>(kept)].t_ns <= kTrueCatchNs) {
    ++kept;
  }
  c->traj.n = kept;
  const std::int64_t last = c->traj.s[static_cast<std::size_t>(kept - 1)].t_ns;
  ASSERT_EQ(last - c->t_hat_ns, 15 * kTcMs);
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult fixed;
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(fixed);
  core.ResizeResult(out);
  ASSERT_TRUE(SolveFixed(core, *c, fixed));
  MpcDockingSegmentCoreInput in = StartFrom(c->in, fixed);
  in.time_c1 = -50.0;  // later is cheaper, by far
  ASSERT_TRUE(core.Solve(in, out));
  EXPECT_TRUE(out.q.allFinite());
  EXPECT_TRUE(out.converged) << DescribeTc(out);
  EXPECT_EQ(out.delta_ns, 15 * kTcMs) << DescribeTc(out);
  EXPECT_LT(out.catch_time_gradient, 0.0) << "it would go later still";
}

// A start that is off the terminal rest, where putting that right COSTS
// objective: the fixed-grid solution with the stop part's jerk taken out — the
// arm does not brake, and saves the braking's cost. The merit counts the rest
// (and the box on the nodes after the catch node), so the step that pays is
// taken; a merit that left the two groups out would see only a rising cost
// and refuse every step.
TEST(MpcDockingSegmentCoreCatchTime, AStartThatMustPayForTheTerminalRestStillConverges) {
  for (dk::Rig fixed_rig : Rigs()) {
    // The start is taken as given though it is far off the rest.
    fixed_rig.params.tol_linear = 20.0;
    const std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, 1, 0, 0);
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult fixed;
    MpcDockingSegmentCoreResult at_start;
    MpcDockingSegmentCoreResult out;
    core.ResizeResult(fixed);
    core.ResizeResult(at_start);
    core.ResizeResult(out);
    ASSERT_TRUE(SolveFixed(core, *c, fixed));
    ASSERT_TRUE(fixed.converged) << DescribeTc(fixed);
    Eigen::VectorXd z = JerkOf(c->rig, fixed);
    const Eigen::Index n = core.Nv();
    // One block per pre-catch interval (BaseParams): the stop part's blocks
    // are the ones from n_pre on.
    z.tail(z.size() - c->rig.params.n_pre * n).setZero();
    MpcDockingSegmentCoreInput in = c->in;
    SetStartAt(*c, z, 0, in);
    in.delta_lo_ns = 0;
    in.delta_hi_ns = 0;
    ASSERT_TRUE(core.Evaluate(in, at_start));
    const double off = at_start.violation[G(DockingRowGroup::kTerminal)];
    ASSERT_GT(off, 1e-2) << fixed_rig.arm.name << ": the start is at rest already";
    // The premise: the start is cheaper than the solution it must become.
    ASSERT_LT(Objective(at_start), Objective(fixed) - 1e-4) << fixed_rig.arm.name;
    ASSERT_TRUE(core.Solve(in, out));
    EXPECT_FALSE(out.init_qp_used) << fixed_rig.arm.name << ": the start must be taken as given";
    EXPECT_TRUE(out.converged) << fixed_rig.arm.name << DescribeTc(out);
    EXPECT_LE(out.violation[G(DockingRowGroup::kTerminal)], kTerminalRestTol);
    EXPECT_GT(Objective(out), Objective(at_start) + 1e-4) << "feasibility was not paid for";
    EXPECT_NEAR(Objective(out), Objective(fixed), 1e-6 * std::max(1.0, Objective(fixed)));
    std::printf(
        "[ record ] %s: terminal rest off by %.3f, objective %.5f -> %.5f (%d iterations)\n",
        fixed_rig.arm.name.c_str(), off, Objective(at_start), Objective(out), out.iterations);
  }
}

// A solve cut short after the catch instant has moved gives back the best
// point it SOLVED on the way — a solution of the problem at the instant it
// reports — not the half-finished iterate at the instant it was cut at.
TEST(MpcDockingSegmentCoreCatchTime, ASolveCutShortGivesBackTheBestPointItSolved) {
  for (const dk::Rig& fixed_rig : Rigs()) {
    const std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, 1, 7 * kTcMs + 311, 20 * kTcMs);
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult fixed;
    core.ResizeResult(fixed);
    ASSERT_TRUE(SolveFixed(core, *c, fixed));
    ASSERT_TRUE(fixed.converged) << DescribeTc(fixed);
    dk::Rig slow_rig = c->rig;
    slow_rig.params.delta_t_step = 0.0005;
    int cut_short = 0;
    int past_the_start = 0;
    for (const int budget : {2, 3, 4, 6, 9}) {
      const std::string where = fixed_rig.arm.name + " budget " + std::to_string(budget);
      slow_rig.params.max_iterations = budget;
      MpcDockingSegmentCore slow;
      ASSERT_EQ(
          slow.Init(slow_rig.model, slow_rig.arm.frame, slow_rig.params, slow_rig.limits, &NoClock),
          MpcDockingReason::kNone);
      MpcDockingSegmentCoreResult out;
      slow.ResizeResult(out);
      ASSERT_TRUE(slow.Solve(StartFrom(c->in, fixed), out));
      ASSERT_FALSE(out.converged) << where << ": 0.5 ms a move cannot reach it" << DescribeTc(out);
      ASSERT_EQ(out.reason, MpcDockingReason::kIterationLimit) << where;
      ASSERT_GT(out.catch_time_steps, 0) << where;
      ++cut_short;
      // It reports a solved point, and says that the search did not finish.
      EXPECT_TRUE(out.catch_time_settled) << where << DescribeTc(out);
      EXPECT_TRUE(out.feasible) << where << DescribeTc(out);
      // The instant the solve was cut at was not solved: the last move's
      // destination never is (it would have moved again, or converged).
      EXPECT_LE(std::abs(out.delta_ns),
                static_cast<std::int64_t>(out.catch_time_steps - 1) * 500'000)
          << where << DescribeTc(out);
      past_the_start += out.delta_ns != 0 ? 1 : 0;
      // It never costs more than the start's instant does …
      EXPECT_LE(Objective(out), Objective(fixed) + 1e-9) << where;
      // … and it IS the solution at its instant: the full core, the instant
      // held there, starts from it and has nothing left to do.
      MpcDockingSegmentCoreInput held = StartFrom(c->in, out);
      held.delta_lo_ns = out.delta_ns;
      held.delta_hi_ns = out.delta_ns;
      MpcDockingSegmentCoreResult again;
      core.ResizeResult(again);
      ASSERT_TRUE(core.Solve(held, again)) << where;
      EXPECT_FALSE(again.init_qp_used) << where;
      EXPECT_TRUE(again.converged) << where << DescribeTc(again);
      EXPECT_EQ(again.iterations, 1) << where;
      EXPECT_LT((again.q - out.q).cwiseAbs().maxCoeff(), 1e-9) << where;
      // The derivative it reports is that instant's, not the cut one's.
      EXPECT_NEAR(out.catch_time_gradient, again.catch_time_gradient,
                  1e-3 * std::max(1.0, std::abs(again.catch_time_gradient)))
          << where;
      std::printf("[ record ] %s: %s\n", where.c_str(), DescribeTc(out).c_str());
    }
    EXPECT_EQ(cut_short, 5);
    EXPECT_GE(past_the_start, 2) << "every cut solve gave its start back";
  }
}

// What a call reports of the catch instant is its own. A call that reads no
// derivative reports none and a solve that solved nothing reports no solved
// point — whatever the call before it on the same core left of either.
TEST(MpcDockingSegmentCoreCatchTime, ACallReportsNothingAnEarlierCallLeft) {
  const dk::Rig fixed_rig = dk::MakeRig(fx::RealArm7());
  const std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, 1, 7 * kTcMs + 311, 0);
  MpcDockingSegmentCore full;
  ASSERT_EQ(full.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &NoClock),
            MpcDockingReason::kNone);
  // Two iterations a solve: enough to confirm a solution it is handed, far
  // too few to find one from the IK target.
  dk::Rig short_rig = c->rig;
  short_rig.params.max_iterations = 2;
  MpcDockingSegmentCore core;
  ASSERT_EQ(
      core.Init(short_rig.model, short_rig.arm.frame, short_rig.params, short_rig.limits, &NoClock),
      MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult at;
  MpcDockingSegmentCoreResult out;
  full.ResizeResult(at);
  core.ResizeResult(out);
  MpcDockingSegmentCoreInput in = c->in;
  in.time_c1 = 0.4;
  in.time_c2 = 25.0;
  in.delta_ref_ns = 3 * kTcMs;
  dk::PerturbTarget(in, c->seed);
  ASSERT_TRUE(full.Solve(in, at));
  ASSERT_TRUE(at.converged) << DescribeTc(at);
  // What every case below is preceded by: a solve that leaves a solved point
  // and — held by the closed box off its best instant — a derivative far from
  // zero.
  const auto leave_both = [&] {
    ASSERT_TRUE(core.Solve(StartFrom(in, at), out));
    ASSERT_TRUE(out.converged) << DescribeTc(out);
    ASSERT_TRUE(out.catch_time_settled);
    ASSERT_GT(std::abs(out.catch_time_gradient), 1e-2) << DescribeTc(out);
  };
  leave_both();
  ASSERT_TRUE(core.Evaluate(StartFrom(in, at), out));
  EXPECT_EQ(out.catch_time_gradient, 0.0);
  EXPECT_FALSE(out.catch_time_settled);
  // A solve cut before it solved anything gives back the point it was cut at.
  leave_both();
  ASSERT_TRUE(core.Solve(in, out));
  ASSERT_EQ(out.reason, MpcDockingReason::kIterationLimit) << DescribeTc(out);
  EXPECT_FALSE(out.catch_time_settled) << DescribeTc(out);
  EXPECT_GT((out.q - at.q).cwiseAbs().maxCoeff(), 1e-6) << "the earlier call's solution came back";
  // A call refused before any iterate.
  leave_both();
  MpcDockingSegmentCoreInput refused = StartFrom(in, at);
  refused.prediction = nullptr;
  ASSERT_FALSE(core.Solve(refused, out));
  EXPECT_EQ(out.catch_time_gradient, 0.0);
  EXPECT_FALSE(out.catch_time_settled);
}

// The penalties of every group move as one: grown by the same factor, put
// back together, and — when the solver fails on a QP after a growth step —
// fallen back to their initial values together. At exit each group's penalty
// over its initial value is therefore one number, the two groups that exist
// only with the catch instant a variable included.
TEST(MpcDockingSegmentCoreCatchTime, ThePenaltiesOfEveryGroupRiseAndFallBackTogether) {
  int resets = 0;
  int grown = 0;
  for (const fx::ArmModel& arm : {fx::RealArm6(), fx::RealArm7()}) {
    dk::Rig fixed_rig = dk::MakeRig(arm);
    // What makes the solver give a QP up after a penalty has grown: ProxQP's
    // own primal-infeasibility test under a trust region
    // (DefaultInfeasibilityThresholdMisreportsAFeasibleQp), at a threshold
    // loose enough to fire on several of these solves.
    fixed_rig.params.delta_tr = 0.05;
    fixed_rig.params.solver.eps_primal_inf = 1e-2;
    fixed_rig.params.mu_init.fill(1e-2);
    fixed_rig.params.mu_init_post_box = 3e-2;
    fixed_rig.params.mu_init_terminal = 5e-2;
    for (const unsigned from : {1U, 20U, 40U, 60U, 80U}) {
      const std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, from, 7 * kTcMs + 311, 20 * kTcMs);
      MpcDockingSegmentCore core;
      ASSERT_EQ(core.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &NoClock),
                MpcDockingReason::kNone);
      MpcDockingSegmentCoreResult out;
      core.ResizeResult(out);
      MpcDockingSegmentCoreInput in = c->in;
      dk::PerturbTarget(in, c->seed);
      if (!core.Solve(in, out)) {
        continue;
      }
      const double factor = out.mu[0] / c->rig.params.mu_init[0];
      for (std::size_t g = 0; g < out.mu.size(); ++g) {
        EXPECT_DOUBLE_EQ(out.mu[g] / c->rig.params.mu_init[g], factor) << arm.name << " " << g;
      }
      EXPECT_DOUBLE_EQ(out.mu_post_box / c->rig.params.mu_init_post_box, factor)
          << arm.name << " seed " << c->seed << DescribeTc(out);
      EXPECT_DOUBLE_EQ(out.mu_terminal / c->rig.params.mu_init_terminal, factor)
          << arm.name << " seed " << c->seed << DescribeTc(out);
      resets += out.mu_resets;
      grown += out.mu_updates;
      std::printf("[ record ] %s seed %u: mu x%.0e, %d growth steps, %d resets:%s\n",
                  arm.name.c_str(), c->seed, factor, out.mu_updates, out.mu_resets,
                  DescribeTc(out).c_str());
    }
  }
  EXPECT_GT(grown, 0) << "no penalty grew";
  EXPECT_GT(resets, 0)
      << "no solve fell back to the initial penalties — the test does not reach it";
  ::testing::Test::RecordProperty("mu_resets", resets);
}

// With the catch instant a variable the same sites cut the same way: the two
// groups that core adds keep their penalties with the others, and what comes
// back is a point the solve stood at.
TEST(MpcDockingSegmentCoreCatchTime, PastItsDeadlineNoQpIsStartedEither) {
  using rtc::catching::MpcDockingCutSite;
  dk::Rig fixed_rig = dk::MakeRig(fx::RealArm6());
  fixed_rig.params.mu_init.fill(1e-2);
  fixed_rig.params.mu_init_post_box = 3e-2;
  fixed_rig.params.mu_init_terminal = 5e-2;
  const std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, 1, 7 * kTcMs + 311, 20 * kTcMs);
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &TripClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreInput in = c->in;
  dk::PerturbTarget(in, c->seed);
  MpcDockingSegmentCoreResult whole;
  // Every fifth read: a solve of this core is long.
  const std::vector<CutSolve> cuts = SolveCutAtEveryRead(core, in, whole, 5);
  ASSERT_GT(whole.catch_time_steps, 0) << DescribeTc(whole);
  ASSERT_GE(cuts.size(), 3U);
  const std::set<MpcDockingCutSite> seen = ExpectCutContract(c->rig.params, whole, cuts);
  EXPECT_EQ(seen.count(MpcDockingCutSite::kInitQp), 1U);
  EXPECT_EQ(seen.count(MpcDockingCutSite::kIteration), 1U);
  for (const CutSolve& cut : cuts) {
    if (!cut.returned) {
      continue;
    }
    const std::string where = "read " + std::to_string(cut.trip) + DescribeTc(cut.out);
    EXPECT_GE(cut.out.delta_ns, in.delta_lo_ns) << where;
    EXPECT_LE(cut.out.delta_ns, in.delta_hi_ns) << where;
    if (cut.out.mu_resets == 0) {
      const double factor = cut.out.mu[0] / c->rig.params.mu_init[0];
      EXPECT_DOUBLE_EQ(cut.out.mu_post_box / c->rig.params.mu_init_post_box, factor) << where;
      EXPECT_DOUBLE_EQ(cut.out.mu_terminal / c->rig.params.mu_init_terminal, factor) << where;
    }
    // A point reported as solved at its instant is feasible there.
    if (cut.out.catch_time_settled) {
      EXPECT_TRUE(cut.out.feasible) << where;
    }
  }
  core.SetClock(&NoClock);
}

// The core's own code allocates nothing with the catch instant a variable
// either — the same gate, stage by stage, over solves that move the instant.
TEST(MpcDockingSegmentCoreCatchTime, EveryStageOutsideTheSolverAllocatesNothing) {
  for (const fx::ArmModel& arm : {fx::RealArm6(), fx::RealArm7()}) {
    dk::Rig fixed_rig = dk::MakeRig(arm);
    fixed_rig.params.w_manip = 0.02;
    fixed_rig.params.w_impact = 0.5;
    fixed_rig.params.e_ref = 0.05;
    fixed_rig.params.e_max = 0.5;
    fixed_rig.params.p_max = 0.5;
    fixed_rig.params.w_perp = 30.0;
    fixed_rig.params.accel_box = true;
    fixed_rig.params.jerk_box = true;
    fixed_rig.limits.qdd_max = Eigen::VectorXd::Constant(arm.model->nv, 80.0);
    fixed_rig.limits.jerk_max = Eigen::VectorXd::Constant(arm.model->nv, 5e3);
    fixed_rig.params.mu_init.fill(1e-2);
    fixed_rig.params.mu_init_post_box = 1e-2;
    fixed_rig.params.mu_init_terminal = 1e-2;
    const std::unique_ptr<TcCase> c = MakeTcCase(fixed_rig, 1, 7 * kTcMs + 311, 20 * kTcMs);
    MpcDockingSegmentCore core;
    ASSERT_EQ(core.Init(c->rig.model, c->rig.arm.frame, c->rig.params, c->rig.limits, &NoClock),
              MpcDockingReason::kNone);
    MpcDockingSegmentCoreResult out;
    MpcDockingSegmentCoreResult next;
    core.ResizeResult(out);
    core.ResizeResult(next);
    MpcDockingSegmentCoreInput in = c->in;
    in.p_line = c->th.p_b;
    in.time_c1 = 0.3;
    in.time_c2 = 10.0;
    dk::PerturbTarget(in, c->seed);
    StageRecord rec;
    core.SetStageHook(&CountingHook, &rec);
    std::size_t news = 0;
    int moves = 0;
    int iterations = 0;
    {
      const rtc::testing::ScopedAllocGate new_gate;
      // From the IK target (the initialisation QP, at this δt_c's gains) …
      const bool ok = core.Solve(in, out);
      news += new_gate.count();
      ASSERT_TRUE(ok) << DescribeTc(out);
    }
    moves += out.catch_time_steps;
    iterations += out.iterations;
    const MpcDockingSegmentCoreInput again = StartFrom(in, out);
    {
      const rtc::testing::ScopedAllocGate new_gate;
      // … from its own solution, and judged without solving.
      const bool ok = core.Solve(again, next) && core.Evaluate(again, next);
      news += new_gate.count();
      ASSERT_TRUE(ok);
    }
    core.SetStageHook(nullptr, nullptr);
    EXPECT_EQ(news, 0U) << arm.name << ": operator new inside Solve";
    for (std::size_t s = 0; s < kStages; ++s) {
      EXPECT_EQ(rec.mallocs[s], 0U) << arm.name << " stage " << StageName(s);
      EXPECT_GT(rec.entries[s], 0) << arm.name << " stage " << StageName(s) << " never ran";
    }
    EXPECT_GT(moves, 0) << arm.name << ": the catch instant never moved" << DescribeTc(out);
    EXPECT_GT(iterations, 4);
    EXPECT_TRUE(out.init_qp_used);
    ::testing::Test::RecordProperty(arm.name + "_tc_gate_iterations", iterations);
    ::testing::Test::RecordProperty(arm.name + "_tc_gate_moves", moves);
  }
}

// ── The core as it is, pinned bit for bit (E1-F14 PR 2, #740) ────────────────
// The catch instant becomes a variable of this core (δt_c). A core that leaves
// it off must stay the core it was: same dimensions, same code path, same
// bits. These digests were taken BEFORE that change, over every way a solve
// ends and over the QP it last assembled.

void AddMatrix(rtc::testing::ValueDigest& h, const Eigen::MatrixXd& m) {
  h.Add(m.rows());
  h.Add(m.cols());
  for (Eigen::Index c = 0; c < m.cols(); ++c) {
    for (Eigen::Index r = 0; r < m.rows(); ++r) {
      h.Add(m(r, c));
    }
  }
}

// Every value a solve leaves in its result — all but the wall-clock fields.
void AddResult(rtc::testing::ValueDigest& h, bool returned, const MpcDockingSegmentCoreResult& r) {
  h.Add(returned);
  AddMatrix(h, r.q);
  AddMatrix(h, r.qd);
  AddMatrix(h, r.qdd);
  AddMatrix(h, r.u);
  h.Add(r.cost.total);
  h.Add(r.cost.reference);
  h.Add(r.cost.stop);
  h.Add(r.cost.tau);
  h.Add(r.cost.acc);
  h.Add(r.cost.jerk);
  h.Add(r.cost.posture);
  h.Add(r.cost.manip);
  h.Add(r.cost.near);
  h.Add(r.cost.terminal);
  h.Add(r.cost.impact);
  h.Add(r.cost.slack);
  h.Add(r.cost.stop_jerk);
  h.Add(r.cost.stop_line);
  h.Add(r.violation);
  h.Add(r.elastic);
  h.Add(r.mu);
  h.Add(r.slack_c);
  h.Add(r.slack_v);
  h.Add(r.kkt_residual);
  h.Add(r.grad_norm);
  h.Add(r.complementarity);
  h.Add(r.step_capped);
  h.Add(r.c_catch);
  h.Add(r.sigma_s);
  h.Add(r.sigma_t);
  h.Add(r.c_guarded);
  h.Add(r.linearization_ratio);
  h.Add(r.linearization_ratio_defined);
  h.Add(r.tau_ratio_max);
  h.Add(r.approach_nodes);
  h.Add(r.iterations);
  h.Add(r.qp_solves);
  h.Add(r.qp_iterations);
  h.Add(r.backtracks);
  h.Add(r.mu_updates);
  h.Add(r.init_qp_used);
  h.Add(r.qp_status);
  h.Add(r.reason);
  h.Add(r.infeasible_group);
  h.Add(r.feasible);
  h.Add(r.converged);
}

// The last QP: its matrices, bounds, solution and multipliers.
void AddLastQp(rtc::testing::ValueDigest& h, const MpcDockingSegmentCore& core) {
  const rtc::tsid::QPData& qp = core.LastQp();
  AddMatrix(h, qp.H);
  AddMatrix(h, qp.g);
  AddMatrix(h, qp.A);
  AddMatrix(h, qp.b);
  AddMatrix(h, qp.C);
  AddMatrix(h, qp.l);
  AddMatrix(h, qp.u);
  AddMatrix(h, core.LastQpSolution());
  AddMatrix(h, core.LastEqualityDual());
  AddMatrix(h, core.LastInequalityDual());
  h.Add(core.NumJerkVariables());
  for (int g = 0; g < kNumDockingRowGroups; ++g) {
    h.Add(core.GroupRowBegin(static_cast<DockingRowGroup>(g)));
    h.Add(core.GroupRowCount(static_cast<DockingRowGroup>(g)));
  }
}

struct PinnedSolve {
  std::string name;
  std::uint64_t digest{0};
  MpcDockingReason reason{MpcDockingReason::kNone};
  bool init_qp_used{false};
  bool step_capped_seen{false};
};

// The solves that are pinned, in a fixed order. One core per path, so that no
// digest depends on which path ran before it.
std::vector<PinnedSolve> PinnedSolves() {
  std::vector<PinnedSolve> all;
  const auto run = [&all](const std::string& name, const dk::Rig& rig,
                          MpcDockingSegmentCore::ClockFn clock, auto&& body) {
    MpcDockingSegmentCore core;
    if (core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, clock) !=
        MpcDockingReason::kNone) {
      ADD_FAILURE() << name << ": Init";
      return;
    }
    MpcDockingSegmentCoreResult out;
    core.ResizeResult(out);
    rtc::testing::ValueDigest h;
    PinnedSolve rec;
    rec.name = name;
    body(core, out, h, rec);
    AddLastQp(h, core);
    rec.digest = h.Value();
    rec.reason = out.reason;
    all.push_back(rec);
  };
  const auto solve = [](MpcDockingSegmentCore& core, const MpcDockingSegmentCoreInput& in,
                        MpcDockingSegmentCoreResult& out, rtc::testing::ValueDigest& h,
                        PinnedSolve& rec) {
    const bool returned = core.Solve(in, out);
    AddResult(h, returned, out);
    rec.init_qp_used = rec.init_qp_used || out.init_qp_used;
    rec.step_capped_seen = rec.step_capped_seen || out.step_capped;
  };
  const auto first_case = [](const dk::Rig& rig, const MpcDockingSegmentCore& core, unsigned push,
                             MpcDockingSegmentCoreInput& in) {
    std::vector<Case> cases = FeasibleCases(rig, core, 1);
    if (cases.size() != 1U) {
      ADD_FAILURE() << "no feasible case";
      return false;
    }
    in = cases[0].in;
    dk::PerturbTarget(in, push);
    return true;
  };
  for (const fx::ArmModel& arm : {fx::RealArm6(), fx::RealArm7()}) {
    const dk::Rig base = dk::MakeRig(arm);
    // The whole path of a feasible throw: from the IK target (initialisation
    // QP), again from its own solution (start taken as given), and that
    // solution judged without solving.
    run(arm.name + "_converged", base, &NoClock,
        [&](MpcDockingSegmentCore& core, MpcDockingSegmentCoreResult& out,
            rtc::testing::ValueDigest& h, PinnedSolve& rec) {
          MpcDockingSegmentCoreInput in;
          if (!first_case(base, core, 1, in)) {
            return;
          }
          solve(core, in, out, h, rec);
          MpcDockingSegmentCoreInput again = in;
          again.initial_valid = true;
          again.q_init = out.q;
          again.qd_init = out.qd;
          again.qdd_init = out.qdd;
          MpcDockingSegmentCoreResult second;
          core.ResizeResult(second);
          solve(core, again, second, h, rec);
          MpcDockingSegmentCoreResult eval;
          core.ResizeResult(eval);
          AddResult(h, core.Evaluate(again, eval), eval);
          // A start that breaks the linear rows goes through the
          // initialisation QP's other target (the whole trajectory).
          again.qdd_init *= 40.0;
          solve(core, again, second, h, rec);
        });
    {
      dk::Rig rig = base;
      rig.params.max_iterations = 1;
      run(arm.name + "_rti", rig, &NoClock,
          [&](MpcDockingSegmentCore& core, MpcDockingSegmentCoreResult& out,
              rtc::testing::ValueDigest& h, PinnedSolve& rec) {
            MpcDockingSegmentCoreInput in;
            if (first_case(rig, core, 9, in)) {
              solve(core, in, out, h, rec);
            }
          });
    }
    {
      dk::Rig rig = base;
      rig.params.max_iterations = 2;
      run(arm.name + "_iteration_limit", rig, &NoClock,
          [&](MpcDockingSegmentCore& core, MpcDockingSegmentCoreResult& out,
              rtc::testing::ValueDigest& h, PinnedSolve& rec) {
            MpcDockingSegmentCoreInput in;
            if (first_case(rig, core, 4, in)) {
              solve(core, in, out, h, rec);
            }
          });
    }
    run(arm.name + "_deadline", base, &FakeClock,
        [&](MpcDockingSegmentCore& core, MpcDockingSegmentCoreResult& out,
            rtc::testing::ValueDigest& h, PinnedSolve& rec) {
          MpcDockingSegmentCoreInput in;
          if (!first_case(base, core, 1, in)) {
            return;
          }
          // Cut after one iteration: the deadline passes during the first.
          g_fake_now = 0;
          in.deadline_ns = kFakeDeadlineNs;
          core.SetStageHook(&PassTheDeadlineAfterTheFirstStep, nullptr);
          solve(core, in, out, h, rec);
          core.SetStageHook(nullptr, nullptr);
        });
    {
      // Every optional row and term, under a trust region small enough to cap
      // the first steps.
      dk::Rig rig = FullCostRig(arm);
      rig.params.max_iterations = 50;
      rig.params.tol_linear = dk::BaseParams(arm.model->nv).tol_linear;
      rig.params.jerk_box = true;
      rig.limits.jerk_max = Eigen::VectorXd::Constant(arm.model->nv, 4e3);
      rig.params.delta_tr = 0.05;
      run(arm.name + "_every_term_trust_region", rig, &NoClock,
          [&](MpcDockingSegmentCore& core, MpcDockingSegmentCoreResult& out,
              rtc::testing::ValueDigest& h, PinnedSolve& rec) {
            MpcDockingSegmentCoreInput in;
            if (!first_case(rig, core, 5, in)) {
              return;
            }
            in.p_line = in.ball[static_cast<std::size_t>(core.CatchNode())].p +
                        Eigen::Vector3d(0.02, -0.03, 0.01);
            in.d_line = Eigen::Vector3d(0.3, -0.5, 0.8).normalized();
            solve(core, in, out, h, rec);
          });
    }
    {
      // Three iterations under a trust region the first steps rest on: the
      // last QP's step is a capped one.
      dk::Rig rig = base;
      rig.params.delta_tr = 0.02;
      rig.params.max_iterations = 3;
      run(arm.name + "_trust_region_capped", rig, &NoClock,
          [&](MpcDockingSegmentCore& core, MpcDockingSegmentCoreResult& out,
              rtc::testing::ValueDigest& h, PinnedSolve& rec) {
            MpcDockingSegmentCoreInput in;
            if (first_case(rig, core, 6, in)) {
              solve(core, in, out, h, rec);
            }
          });
    }
    {
      dk::Rig rig = base;
      rig.params.chance = false;
      run(arm.name + "_no_chance_rows", rig, &NoClock,
          [&](MpcDockingSegmentCore& core, MpcDockingSegmentCoreResult& out,
              rtc::testing::ValueDigest& h, PinnedSolve& rec) {
            MpcDockingSegmentCoreInput in;
            if (first_case(rig, core, 2, in)) {
              solve(core, in, out, h, rec);
            }
          });
    }
    for (const InfeasibleCase& c : InfeasibleCases(arm)) {
      run(arm.name + "_infeasible_" + c.name, c.rig, &NoClock,
          [&](MpcDockingSegmentCore& core, MpcDockingSegmentCoreResult& out,
              rtc::testing::ValueDigest& h, PinnedSolve& rec) {
            MpcDockingSegmentCoreInput in;
            c.fill(c.rig, core, in);
            solve(core, in, out, h, rec);
          });
    }
  }
  return all;
}

/// PinnedSolves()' digests on the code BEFORE the catch instant became a
/// variable of the core (E1-F14 PR 2). Numbers to compare against, tied to
/// where they were taken — the rule of kWholeCatchDigest
/// (test_catching_approach_cycle.cpp): a change MEANT to alter what the core
/// without δt_c computes replaces them in a commit of its own that says why; a
/// red on another host or after a library upgrade is the environment.
struct PinnedDigest {
  const char* name;
  std::uint64_t digest;
};

constexpr std::array<PinnedDigest, 22> kPinnedCoreDigest{{
    {"real_6dof_converged", 0x036c88c487850ddfULL},
    {"real_6dof_rti", 0xcbb5fb83b4eee7a2ULL},
    {"real_6dof_iteration_limit", 0x55b83155a1120b78ULL},
    {"real_6dof_deadline", 0x575d790570b5f3d0ULL},
    {"real_6dof_every_term_trust_region", 0x9a4efbdab2c85406ULL},
    {"real_6dof_trust_region_capped", 0x4f1ee5530566f9ccULL},
    {"real_6dof_no_chance_rows", 0x1046d94161545a8dULL},
    {"real_6dof_infeasible_out_of_reach", 0x2e0d831fe3b13014ULL},
    {"real_6dof_infeasible_lead_too_short", 0x792ce31699646d31ULL},
    {"real_6dof_infeasible_torque_exceeded", 0x2d4c7bdc490b094fULL},
    {"real_6dof_infeasible_timing_vs_velocity_set", 0xa04639ca40c95860ULL},
    {"real_7dof_converged", 0x5fc0c4c751a4e522ULL},
    {"real_7dof_rti", 0xfaeec49f91279b77ULL},
    {"real_7dof_iteration_limit", 0x3d5f64b9ac649bb7ULL},
    {"real_7dof_deadline", 0x7e29ad6726113dc8ULL},
    {"real_7dof_every_term_trust_region", 0xa5ecddb9ceca693aULL},
    {"real_7dof_trust_region_capped", 0x774b3cc22930d8deULL},
    {"real_7dof_no_chance_rows", 0x760f11d31fe9bd63ULL},
    {"real_7dof_infeasible_out_of_reach", 0x4ff27dc4d9533507ULL},
    {"real_7dof_infeasible_lead_too_short", 0x5eace2e75d101ea8ULL},
    {"real_7dof_infeasible_torque_exceeded", 0x0d853ce60be940cdULL},
    {"real_7dof_infeasible_timing_vs_velocity_set", 0x90c3664d20b8f542ULL},
}};

TEST(MpcDockingSegmentCore, TheCoreWithoutACatchTimeVariableIsUnchangedBitForBit) {
  const std::vector<PinnedSolve> solves = PinnedSolves();
  ASSERT_EQ(solves.size(), kPinnedCoreDigest.size());
  std::set<std::uint64_t> distinct;
  std::set<MpcDockingReason> reasons;
  bool init_qp = false;
  bool capped = false;
  for (std::size_t i = 0; i < solves.size(); ++i) {
    const PinnedSolve& s = solves[i];
    std::array<char, 32> hex{};
    std::snprintf(hex.data(), hex.size(), "0x%016llx", static_cast<unsigned long long>(s.digest));
    RecordProperty("core_digest_" + s.name, hex.data());
    std::printf("[ record ] core_digest %s: %s (%s)\n", s.name.c_str(), hex.data(),
                MpcDockingReasonName(s.reason));
    EXPECT_EQ(s.name, kPinnedCoreDigest[i].name);
    EXPECT_EQ(s.digest, kPinnedCoreDigest[i].digest)
        << s.name << ": the core without δt_c changed: digest " << hex.data()
        << ". A core that leaves the catch instant fixed must compute what it did before "
           "(see kPinnedCoreDigest).";
    distinct.insert(s.digest);
    reasons.insert(s.reason);
    init_qp = init_qp || s.init_qp_used;
    capped = capped || s.step_capped_seen;
  }
  // The pinned solves went through every way a solve ends.
  EXPECT_EQ(distinct.size(), solves.size()) << "two paths left the same digest";
  for (const MpcDockingReason r : {MpcDockingReason::kConverged, MpcDockingReason::kIterationLimit,
                                   MpcDockingReason::kDeadline, MpcDockingReason::kInfeasible}) {
    EXPECT_EQ(reasons.count(r), 1U) << MpcDockingReasonName(r);
  }
  EXPECT_TRUE(init_qp);
  EXPECT_TRUE(capped) << "no pinned solve rested on the trust region";
}

// The digest can tell: one bit of one node, or of one QP coefficient, moves it.
TEST(MpcDockingSegmentCore, ThePinnedDigestSeesASingleBit) {
  const dk::Rig rig = dk::MakeRig(fx::RealArm6());
  MpcDockingSegmentCore core;
  ASSERT_EQ(core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, &NoClock),
            MpcDockingReason::kNone);
  MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  std::vector<Case> cases = FeasibleCases(rig, core, 1);
  ASSERT_EQ(cases.size(), 1U);
  ASSERT_TRUE(core.Solve(cases[0].in, out));
  const auto digest = [](const MpcDockingSegmentCoreResult& r) {
    rtc::testing::ValueDigest h;
    AddResult(h, true, r);
    return h.Value();
  };
  const std::uint64_t base = digest(out);
  MpcDockingSegmentCoreResult moved = out;
  const Eigen::Index last = moved.q.cols() - 1;
  moved.q(0, last) = std::nextafter(moved.q(0, last), std::numeric_limits<double>::infinity());
  EXPECT_NE(digest(moved), base);
  moved = out;
  moved.cost.near = std::nextafter(moved.cost.near, std::numeric_limits<double>::infinity());
  EXPECT_NE(digest(moved), base);
  moved = out;
  moved.reason = MpcDockingReason::kIterationLimit;
  EXPECT_NE(digest(moved), base);
}

}  // namespace
