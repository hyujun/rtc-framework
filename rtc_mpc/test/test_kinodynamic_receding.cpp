/// @file test_kinodynamic_receding.cpp
/// @brief The kinodynamic MPC tick after tick, through the handler core, on
///        the public G1 with the shipped walk: the standing solve (its budget,
///        then the online one), the receding walk window (one stride, the
///        state rolled out from the previous plan) against the out-of-tree
///        probe's measurements, and the multipliers of the rows a node's mode
///        makes inactive (every tick, +0.0 bit for bit).
///
/// The receding gates are registered on this window as the suite itself
/// measured it once on the shipped walk (single support 0.45 s; shipped
/// reference · soft wrench cone, v_x 0.5, x0 = the previous solution's node 1,
/// semi-implicit Euler, 40 ticks of 0.035 s from t = 20), by the rule the
/// first registration used (twice the probe's iterations and residuals, 0.9 ×
/// its converged ticks): with 50 iterations and 20 AL iterations a tick, on
/// the window led by kWalkLead, 15 of 40 ticks converged in 1675 iterations
/// (median 50 a tick); with 10 and 1, on the window without the lead, the
/// tick primal residual had median 0.540 and maximum 3.24. Gates: converged
/// ticks ≥ 0.9 × 15, Σ iterations ≤ 2 × 1675, median ≤ 2 × 50, converged
/// ticks' prim ≤ the tolerance; online prim median ≤ 2 × 0.540 (the maximum
/// is reported — see that test). RK2 is reported (no control).
///
/// On the led window the double-support → single-support switches fall on
/// the node grid: the same instant is `t0 + k·dt` one tick and
/// `t0' + (k − 1)·dt` the next, rounded to either side of the event, so the
/// half-open mode lookup gives a node already in the horizon another mode.
/// The shift then carries multipliers into rows the node's new mode does not
/// bind — what the OCP's ClearInactiveMultipliers clears. The report prints
/// the flips; TheRecedingWalkWindowsScheduleFlips holds the premise.
///
/// The shipped online iteration cap is held to the standing window: through
/// the phase manager with a zero command, every online tick converges within
/// it (the solver reports a solve converged in N iterations only with a cap
/// of at least N + 1).
///
/// The same window through the locomotion phase manager (its node grid: node
/// times from the grid index, the warm start shifted by the index difference)
/// has no flip: a node keeps its mode from tick to tick, the hook clears
/// nothing, every stage's dynamics and constraint data carry the node's mode
/// through touch-downs and lift-offs, and the shift itself is checked bit for
/// bit (0, 1 and 2 nodes; the nodes it appends start from the reference
/// control). Those runs also report the online residual of the whole horizon
/// and of the segment a consumer reads (the publish rule's input).

#include "handler/internal/mpc_handler_core.hpp"
#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/contact_constraints.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp_config.hpp"
#include "rtc_mpc/phase/locomotion_phase_manager.hpp"
#include "rtc_mpc/reference/contact_schedule.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "rtc_mpc/thread/handler_mpc_thread.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/kinodynamic_ocp_fixture.hpp"
#include "test_utils/shipped_walk_fixture.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/stage-data.hpp>
#include <aligator/core/traj-opt-data.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>
#include <omp.h>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace tu = rtc::mpc::test_utils;
using rtc::mpc::HorizonReference;
using rtc::mpc::KinodynamicOCP;
using rtc::mpc::KinodynamicOCPConfig;
using rtc::mpc::MPCSolution;
using rtc::mpc::MPCSolveError;
using rtc::mpc::MPCSolverConfig;
using rtc::mpc::MPCStateSnapshot;
using rtc::mpc::PhaseContext;
using Core = rtc::mpc::internal::MPCHandlerCore;

constexpr int kTicks = 40;
constexpr double kT0 = 20.0;
// The walk window's warm-up lead [s] (ShippedWalkWindow): the smallest of
// 0, 0.005, … whose 40 ticks' mode grid flips, chosen from the schedule alone
// before any gate below was measured on it. Without it the shipped walk's
// switches (every 0.55 s) miss the node grid in this window and nothing flips.
constexpr double kWalkLead = 0.010;

/// The tolerances and budgets these gates were registered with (the online
/// pair then shipped; the walk window's bounds below are measured on it).
MPCSolverConfig RegisteredSolver() {
  MPCSolverConfig s;
  s.prim_tol = 1e-4;
  s.dual_tol = 1e-4;
  s.mu_init = 1e-2;
  s.max_iters = 50;
  s.max_al_iters = 20;
  s.max_initial_iters = 300;
  s.max_online_al_iters = 20;
  return s;
}

/// What a solve left behind, for comparing two runs tick by tick.
struct Capture {
  aligator::ResultsTpl<double> results;
  std::array<double, rtc::mpc::kMaxHorizon + 1> node_prim_inf{};
};

/// The leading nodes of a plan, as a consumer of a two-node period reads them
/// (nodes 0..3) plus the two the next plan's leading nodes line up with (4, 5).
struct PlanSlice {
  static constexpr int kNodes = 6;
  std::array<std::array<double, rtc::mpc::kMaxNq>, kNodes> q{};
  std::array<std::array<double, rtc::mpc::kMaxNv>, kNodes> v{};
  std::array<std::array<double, rtc::mpc::kMaxContactVars>, kNodes> lambda{};
  int nq{0};
  int nv{0};
  int n_contact_vars{0};
};

/// One tick of a receding run.
struct Tick {
  int iterations{0};
  bool converged{false};
  bool initial{false};
  double prim{0.0};
  double ms{0.0};
  std::size_t cleared{0};   ///< multipliers the OCP cleared after the shift
  int inactive_nonzero{0};  ///< inactive-row multipliers not +0.0 after the solve
  int mode_flips{0};        ///< nodes whose mode differs from the shifted one
  std::string modes;
};

double Median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return n % 2 == 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

class KinodynamicReceding : public ::testing::TestWithParam<const char*> {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(tu::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << tu::kG1Urdf;
    }
    model_ = tu::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(tu::kG1SolesYaml)),
              rtc::mpc::RobotModelInitError::kNoError);
    stand_ = tu::G1Stand(model_);
    ASSERT_EQ(rtc::mpc::LoadKinodynamicOCPConfig(tu::G1KinodynamicNode(GetParam()), handler_, cfg_),
              rtc::mpc::KinodynamicOCPConfigError::kNoError);
  }

  [[nodiscard]] static PhaseContext Context(const HorizonReference* h) {
    PhaseContext ctx;
    ctx.ocp_type = std::string(KinodynamicOCP::kType);
    ctx.cost_config.horizon_length = tu::kKinoNodes;
    ctx.cost_config.dt = tu::kKinoDt;
    ctx.horizon = h;
    return ctx;
  }

  /// The snapshot at the solution's node 1 (the plan rolled out one node).
  [[nodiscard]] MPCStateSnapshot NodeOne(const MPCSolution& out, std::uint64_t sequence) const {
    MPCStateSnapshot s{};
    s.nq = model_.nq;
    s.nv = model_.nv;
    for (int i = 0; i < model_.nq; ++i) {
      s.q[static_cast<std::size_t>(i)] = out.q_traj[1][static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < model_.nv; ++i) {
      s.v[static_cast<std::size_t>(i)] = out.v_traj[1][static_cast<std::size_t>(i)];
    }
    s.sequence = sequence;
    s.valid = true;
    return s;
  }

  /// Inactive-row multipliers (by the stage's current mode) that are not the
  /// +0.0 bit pattern, after the last solve.
  [[nodiscard]] int InactiveNonZero(const Core& core, const KinodynamicOCP& ocp) const {
    const auto& vs = core.solver()->results_.vs;
    const int nc = handler_.n_contacts();
    const int rows = rtc::mpc::ContactConstraintResidual::EqualityRows(6);
    int bad = 0;
    for (int k = 0; k < tu::kKinoNodes; ++k) {
      const Eigen::VectorXd& v = vs[static_cast<std::size_t>(k)];
      for (int c = 0; c < nc; ++c) {
        const bool stance = ((ocp.StageMode(k) >> c) & 1U) != 0U;
        for (int r = 0; r < rows; ++r) {
          const bool swing_row = r == rows - 1;
          if (stance == swing_row && (v[c * rows + r] != 0.0 || std::signbit(v[c * rows + r]))) {
            ++bad;
          }
        }
      }
    }
    return bad;
  }

  /// The mode flips of Run's window over ticks 1 … kTicks from the schedule
  /// alone: the scheduler and the reference builder, no solve, the stand pose
  /// as every tick's state (a node's mode is the schedule's, not the state's).
  [[nodiscard]] int ScheduleFlips(double lead) const {
    tu::ShippedWalkWindow walk(model_, handler_, stand_, kT0, 0.5, lead);
    const MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
    std::vector<std::uint32_t> prev(static_cast<std::size_t>(tu::kKinoNodes + 1), 0U);
    int flips = 0;
    for (int n = 0; n <= kTicks; ++n) {
      if (n > 0) {
        const double t = kT0 + n * tu::kKinoDt;
        EXPECT_EQ(walk.scheduler.Advance(t, walk.cmd, rtc::mpc::PlanarVelocity{}),
                  rtc::mpc::GaitSchedulerError::kNoError);
        EXPECT_EQ(walk.builder.Build(walk.scheduler, snap, walk.cmd),
                  rtc::mpc::ReferenceError::kNoError);
      }
      const HorizonReference& h = walk.horizon();
      if (n > 0) {
        flips += ModeFlips(prev, h);
      }
      KeepModes(h, prev);
    }
    return flips;
  }

  /// The nodes of @p h whose mode differs from the next node's mode in
  /// @p prev, the previous tick's modes (the window moves one node a tick).
  [[nodiscard]] static int ModeFlips(const std::vector<std::uint32_t>& prev,
                                     const HorizonReference& h) {
    int flips = 0;
    for (int k = 0; k < tu::kKinoNodes; ++k) {
      flips += prev[static_cast<std::size_t>(k + 1)] != h.stages[static_cast<std::size_t>(k)].mode;
    }
    return flips;
  }

  /// @p h's node modes into @p modes (kKinoNodes + 1 of them).
  static void KeepModes(const HorizonReference& h, std::vector<std::uint32_t>& modes) {
    modes.assign(static_cast<std::size_t>(tu::kKinoNodes + 1), 0U);
    for (int k = 0; k <= tu::kKinoNodes; ++k) {
      modes[static_cast<std::size_t>(k)] = h.stages[static_cast<std::size_t>(k)].mode;
    }
  }

  /// Ticks 0 … kTicks of the shipped walk from the stand pose, each tick's
  /// state the previous solution's node 1. Tick 0 is the cold start (the
  /// standing solve); `solver` sets both budgets; `lead` is the window's
  /// warm-up lead (ShippedWalkWindow).
  std::vector<Tick> Run(const MPCSolverConfig& solver, std::vector<Capture>* captures = nullptr,
                        KinodynamicOCP::QuadraticCostDerivatives derivatives =
                            KinodynamicOCP::QuadraticCostDerivatives::kStructured,
                        double lead = 0.0) {
    tu::ShippedWalkWindow walk(model_, handler_, stand_, kT0, 0.5, lead);
    KinodynamicOCP ocp(cfg_, KinodynamicOCP::StageKinematics::kShared, derivatives);
    Core core;
    PhaseContext ctx = Context(&walk.horizon());
    EXPECT_EQ(core.Init(solver, handler_, {}, ctx, ocp), rtc::mpc::MPCInitError::kNoError);
    auto out = std::make_unique<MPCSolution>();
    MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
    std::vector<Tick> ticks;
    std::vector<std::uint32_t> prev_modes;
    for (int n = 0; n <= kTicks; ++n) {
      if (n > 0) {
        snap = NodeOne(*out, static_cast<std::uint64_t>(n));
        const double t = kT0 + n * tu::kKinoDt;
        EXPECT_EQ(walk.scheduler.Advance(t, walk.cmd, rtc::mpc::PlanarVelocity{}),
                  rtc::mpc::GaitSchedulerError::kNoError);
        EXPECT_EQ(walk.builder.Build(walk.scheduler, snap, walk.cmd),
                  rtc::mpc::ReferenceError::kNoError);
        ctx = Context(&walk.horizon());
      }
      const HorizonReference& h = walk.horizon();
      Tick tick;
      for (int k = 0; k <= tu::kKinoNodes; ++k) {
        tick.modes += static_cast<char>('0' + h.stages[static_cast<std::size_t>(k)].mode);
      }
      if (!prev_modes.empty()) {
        tick.mode_flips = ModeFlips(prev_modes, h);
      }
      KeepModes(h, prev_modes);
      const auto t0 = std::chrono::steady_clock::now();
      const MPCSolveError e = core.Solve(ctx, snap, ocp, *out);
      tick.ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      EXPECT_EQ(e, MPCSolveError::kNoError) << "tick " << n;
      if (e != MPCSolveError::kNoError) {
        break;
      }
      tick.iterations = out->iterations;
      tick.converged = out->converged;
      tick.initial = out->initial_solve;
      tick.prim = out->residuals.prim_inf;
      tick.cleared = core.ClearedMultipliers();
      tick.inactive_nonzero = InactiveNonZero(core, ocp);
      ticks.push_back(tick);
      if (captures != nullptr) {
        captures->push_back(Capture{core.solver()->results_, out->node_prim_inf});
      }
    }
    return ticks;
  }

  static void Report(const char* name, const std::vector<Tick>& ticks) {
    for (std::size_t n = 0; n < ticks.size(); ++n) {
      const Tick& t = ticks[n];
      std::printf(
          "[report] %s tick %2zu: it %3d conv %d initial %d prim %.3g %.1f ms, cleared %zu, "
          "mode flips %d, modes %s\n",
          name, n, t.iterations, t.converged ? 1 : 0, t.initial ? 1 : 0, t.prim, t.ms, t.cleared,
          t.mode_flips, t.modes.c_str());
    }
  }

  /// The phase manager on the shipped walk: anchored at 16 s in stance, the
  /// walk requested at once (straight at @p v_x) and run until it has taken
  /// over (no solve). Returns the grid index of the last tick.
  std::int64_t WalkManager(rtc::mpc::LocomotionPhaseManager& m, double v_x = 0.5) const {
    EXPECT_EQ(m.Load(tu::ShippedLocomotionYaml()), rtc::mpc::LocomotionPhaseError::kNoError);
    rtc::mpc::FilteredCommand cmd = tu::RegisteredCommand();
    cmd.v_x = v_x;
    cmd.v_y = 0.0;
    cmd.omega_z = 0.0;
    cmd.base_height = 0.7925;
    m.SetCommand(cmd);
    m.SetPolicyEnabled(false);
    const MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
    EXPECT_NE(GridTick(m, snap, 16.0).horizon, nullptr);
    m.RequestGait(2);
    std::int64_t index = 0;
    while (m.scheduler().current_gait() != 2U || m.scheduler().switch_pending() || index < 115) {
      ++index;
      EXPECT_NE(GridTick(m, snap, m.grid().Time(index)).horizon, nullptr);
    }
    return index;
  }

  /// The shipped stance gait with a zero command, 30 grid ticks on from the
  /// first; returns the grid index.
  std::int64_t StanceManager(rtc::mpc::LocomotionPhaseManager& m) const {
    EXPECT_EQ(m.Load(tu::ShippedLocomotionYaml()), rtc::mpc::LocomotionPhaseError::kNoError);
    rtc::mpc::FilteredCommand cmd = tu::RegisteredCommand();
    cmd.v_x = 0.0;
    cmd.v_y = 0.0;
    cmd.omega_z = 0.0;
    cmd.base_height = 0.7925;
    m.SetCommand(cmd);
    m.SetPolicyEnabled(false);
    const MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
    EXPECT_NE(GridTick(m, snap, 16.0).horizon, nullptr);
    std::int64_t index = 0;
    for (int i = 0; i < 30; ++i) {
      EXPECT_NE(GridTick(m, snap, m.grid().Time(++index)).horizon, nullptr);
    }
    return index;
  }

  [[nodiscard]] PhaseContext GridTick(rtc::mpc::LocomotionPhaseManager& m,
                                      const MPCStateSnapshot& snap, double t) const {
    const Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(snap.q.data(), model_.nq);
    const Eigen::VectorXd v = Eigen::Map<const Eigen::VectorXd>(snap.v.data(), model_.nv);
    rtc::mpc::PhaseManagerBase& base = m;
    return base.Update(snap, q, v, Eigen::VectorXd(), pinocchio::SE3::Identity(), t);
  }

  /// The snapshot at the solution's node @p k.
  [[nodiscard]] MPCStateSnapshot NodeAt(const MPCSolution& out, int k,
                                        std::uint64_t sequence) const {
    MPCStateSnapshot s = NodeOne(out, sequence);
    for (int i = 0; i < model_.nq; ++i) {
      s.q[static_cast<std::size_t>(i)] =
          out.q_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < model_.nv; ++i) {
      s.v[static_cast<std::size_t>(i)] =
          out.v_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)];
    }
    return s;
  }

  /// Stages whose dynamics or constraint data do not carry the horizon's
  /// mode of their node, after the last solve.
  [[nodiscard]] static int ModeLeaks(const Core& core, KinodynamicOCP& ocp,
                                     const HorizonReference& h, int n_contacts) {
    int leaks = 0;
    auto& problem = ocp.problem();
    const auto& data = core.solver()->workspace_.problem_data;
    for (std::size_t k = 0; k < problem.stages_.size(); ++k) {
      const std::uint32_t mode = h.stages[k].mode;
      const auto* ode = rtc::mpc::KinodynamicDynamics::OdeOf(*problem.stages_[k]);
      const auto* cd = dynamic_cast<const rtc::mpc::ContactConstraintData*>(
          data.stage_data[k]->constraint_data[0].get());
      if (ode == nullptr || cd == nullptr ||
          cd->stance.size() != static_cast<std::size_t>(n_contacts)) {
        ++leaks;
        continue;
      }
      for (int c = 0; c < n_contacts; ++c) {
        const bool stance = ((mode >> c) & 1U) != 0U;
        leaks += ode->contact_states_[static_cast<std::size_t>(c)] != stance;
        leaks += (cd->stance[static_cast<std::size_t>(c)] != 0) != stance;
      }
    }
    return leaks;
  }

  /// One tick of a grid run.
  struct GridTickReport {
    int shift{0};
    int iterations{0};
    bool converged{false};
    bool cold{false};     ///< the solve was cold-seeded
    bool initial{false};  ///< the solve ran on the standing budget
    double prim{0.0};
    double dual{0.0};
    double consumed{0.0};  ///< over the nodes a consumer at GridRun's period reads
    double node_max{0.0};  ///< max of node_prim_inf
    double ms{0.0};        ///< wall time of the solve (this host — a report, never a gate)
    std::int64_t offset_ns{0};
    double expected_offset_ns{0.0};
    std::size_t cleared{0};
    int leaks{0};
    int flips{0};
  };

  /// What a grid run takes beyond the solver and the shifts.
  struct GridOptions {
    bool stance{false};  ///< the stance window instead of the walk
    KinodynamicOCP::StageKinematics kinematics{KinodynamicOCP::StageKinematics::kShared};
    int period_nodes{1};  ///< the consumer's period, in node intervals
    double v_x{0.5};      ///< the walk's command [m/s] (the stance window ignores it)
    double base_v0{0.0};  ///< the first state's base linear velocity on every axis [m/s]
    /// The tick (1-based, online) whose state arrives with a new
    /// `state_epoch`; −1 never. The core cold-seeds that tick.
    int reseed_at{-1};
    std::vector<Capture>* captures{nullptr};
    std::vector<PlanSlice>* plans{nullptr};
  };

  /// The shipped walk through the phase manager (@p stance: the stance
  /// window instead): a standing solve on the window, then one tick per entry
  /// of @p shifts (the grid index moving by it), each tick 0.4·dt into its
  /// node interval, the state the previous solution's node at the shift.
  /// Each tick's solve is appended to @p captures when given; @p kinematics
  /// is the OCP's build option; the consumed segment is that of a consumer
  /// with a period of @p period_nodes node intervals.
  std::vector<GridTickReport> GridRun(
      const MPCSolverConfig& solver, const std::vector<int>& shifts, bool stance = false,
      std::vector<Capture>* captures = nullptr,
      KinodynamicOCP::StageKinematics kinematics = KinodynamicOCP::StageKinematics::kShared,
      int period_nodes = 1) {
    GridOptions opts;
    opts.stance = stance;
    opts.captures = captures;
    opts.kinematics = kinematics;
    opts.period_nodes = period_nodes;
    return GridRun(solver, shifts, opts);
  }

  /// The grid run with every option (GridOptions).
  std::vector<GridTickReport> GridRun(const MPCSolverConfig& solver, const std::vector<int>& shifts,
                                      const GridOptions& opts) {
    rtc::mpc::LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
    std::int64_t index = opts.stance ? StanceManager(m) : WalkManager(m, opts.v_x);
    const double dt = tu::kKinoDt;
    MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
    for (std::size_t i = 0; i < 3; ++i) {
      snap.v[i] = opts.base_v0;
    }
    std::uint32_t epoch = 0;
    PhaseContext ctx = GridTick(m, snap, m.grid().Time(index) + 0.4 * dt);
    KinodynamicOCP ocp(cfg_, opts.kinematics);
    Core core;
    EXPECT_EQ(core.Init(solver, handler_, {}, ctx, ocp), rtc::mpc::MPCInitError::kNoError);
    auto out = std::make_unique<MPCSolution>();
    auto prev_h = std::make_unique<HorizonReference>();
    std::vector<GridTickReport> ticks;
    const auto period_ns = static_cast<std::uint64_t>(std::llround(opts.period_nodes * dt * 1e9));
    for (std::size_t n = 0; n <= shifts.size(); ++n) {
      int shift = 0;
      bool reseed = false;
      if (n > 0) {
        shift = shifts[n - 1];
        snap = NodeAt(*out, shift, n);
        reseed = static_cast<int>(n) == opts.reseed_at;
        epoch += reseed ? 1U : 0U;
        snap.state_epoch = epoch;
        index += shift;
        ctx = GridTick(m, snap, m.grid().Time(index) + 0.4 * dt);
      }
      EXPECT_NE(ctx.horizon, nullptr);
      if (ctx.horizon == nullptr) {
        break;
      }
      const HorizonReference& h = *ctx.horizon;
      GridTickReport r;
      r.shift = shift;
      if (n > 0) {
        for (std::uint32_t k = 0; k + static_cast<std::uint32_t>(shift) < h.n_stages; ++k) {
          r.flips += prev_h->stages[k + static_cast<std::uint32_t>(shift)].mode != h.stages[k].mode;
        }
      }
      const auto t0 = std::chrono::steady_clock::now();
      const MPCSolveError e = core.Solve(ctx, snap, ocp, *out);
      r.ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      EXPECT_EQ(e, MPCSolveError::kNoError) << "tick " << n;
      if (e != MPCSolveError::kNoError) {
        break;
      }
      EXPECT_EQ(core.LastNodeShift(), reseed ? 0 : shift) << "tick " << n;
      r.iterations = out->iterations;
      r.converged = out->converged;
      r.cold = out->cold_start;
      r.initial = out->initial_solve;
      r.prim = out->residuals.prim_inf;
      r.dual = out->residuals.dual_inf;
      r.consumed = rtc::mpc::HandlerMPCThread::ConsumedPrimInf(*out, period_ns);
      for (int k = 0; k <= out->horizon_length; ++k) {
        r.node_max = std::max(r.node_max, out->node_prim_inf[static_cast<std::size_t>(k)]);
      }
      r.offset_ns = out->horizon_offset_ns;
      r.expected_offset_ns = ctx.horizon_offset * 1e9;
      r.cleared = core.ClearedMultipliers();
      r.leaks = ModeLeaks(core, ocp, h, handler_.n_contacts());
      ticks.push_back(r);
      if (opts.captures != nullptr) {
        opts.captures->push_back(Capture{core.solver()->results_, out->node_prim_inf});
      }
      if (opts.plans != nullptr) {
        opts.plans->push_back(Slice(*out));
      }
      std::memcpy(static_cast<void*>(prev_h.get()), &h, sizeof(HorizonReference));
    }
    return ticks;
  }

  /// The leading nodes of @p out (PlanSlice).
  [[nodiscard]] static PlanSlice Slice(const MPCSolution& out) {
    PlanSlice s;
    s.nq = out.nq;
    s.nv = out.nv;
    s.n_contact_vars = out.n_contact_vars;
    for (std::size_t k = 0; k < static_cast<std::size_t>(PlanSlice::kNodes); ++k) {
      s.q[k] = out.q_traj[k];
      s.v[k] = out.v_traj[k];
      s.lambda[k] = out.lambda_traj[k];
    }
    return s;
  }

  static void ReportGrid(const char* name, const char* what,
                         const std::vector<GridTickReport>& ticks) {
    for (std::size_t n = 0; n < ticks.size(); ++n) {
      const GridTickReport& t = ticks[n];
      std::printf(
          "[report] %s %s tick %2zu: shift %d it %3d conv %d prim %.3g consumed %.3g "
          "cleared %zu flips %d leaks %d\n",
          name, what, n, t.shift, t.iterations, t.converged ? 1 : 0, t.prim, t.consumed, t.cleared,
          t.flips, t.leaks);
    }
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  Eigen::VectorXd stand_;
  KinodynamicOCPConfig cfg_;
};

/// Bit equality of two iterate lists over entries `[from, to)`.
bool SameBits(const std::vector<Eigen::VectorXd>& a, const std::vector<Eigen::VectorXd>& b,
              std::size_t from = 0, std::size_t to = static_cast<std::size_t>(-1)) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = from; i < std::min(to, a.size()); ++i) {
    if (a[i].size() != b[i].size() ||
        std::memcmp(a[i].data(), b[i].data(),
                    sizeof(double) * static_cast<std::size_t>(a[i].size())) != 0) {
      return false;
    }
  }
  return true;
}

bool SameBits(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  return a.size() == b.size() &&
         std::memcmp(a.data(), b.data(), sizeof(double) * static_cast<std::size_t>(a.size())) == 0;
}

bool AllPlusZero(const Eigen::VectorXd& v) {
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    if (v[i] != 0.0 || std::signbit(v[i])) {
      return false;
    }
  }
  return true;
}

// ── Standing solve ──────────────────────────────────────────────────────────

// From the stand pose on a stance window the first solve runs on the standing
// budget and converges (probe: 1 iteration; from a moving base 3); the next
// tick runs on the online budget.
TEST_P(KinodynamicReceding, TheStandingSolveConvergesThenTheOnlineBudgetRuns) {
  const auto h = tu::StanceHorizon(handler_, stand_);

  struct Case {
    const char* name;
    double v;
  };

  for (const Case c : {Case{"stand", 0.0}, Case{"moving base", 0.05}}) {
    MPCStateSnapshot state = tu::SnapshotAt(model_, stand_);
    state.v[0] = c.v;
    state.v[1] = c.v;
    state.v[2] = c.v;
    KinodynamicOCP ocp(cfg_);
    Core core;
    const PhaseContext ctx = Context(h.get());
    ASSERT_EQ(core.Init(RegisteredSolver(), handler_, {}, ctx, ocp),
              rtc::mpc::MPCInitError::kNoError);
    ASSERT_TRUE(core.InitialSolvePending());
    auto out = std::make_unique<MPCSolution>();
    const auto t0 = std::chrono::steady_clock::now();
    ASSERT_EQ(core.Solve(ctx, state, ocp, *out), MPCSolveError::kNoError) << c.name;
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("[report] %s standing solve (%s): %d iterations, %.1f ms\n", GetParam(), c.name,
                out->iterations, ms);
    EXPECT_TRUE(out->initial_solve) << c.name;
    EXPECT_EQ(core.solver()->max_iters, 300U) << c.name;
    EXPECT_EQ(core.solver()->max_al_iters, 20U) << c.name;
    EXPECT_TRUE(out->converged) << c.name;
    EXPECT_LE(out->iterations, 10) << c.name;
    EXPECT_FALSE(core.InitialSolvePending()) << c.name;

    state.sequence = 1;
    ASSERT_EQ(core.Solve(ctx, state, ocp, *out), MPCSolveError::kNoError) << c.name;
    EXPECT_FALSE(out->initial_solve) << c.name;
    EXPECT_EQ(core.solver()->max_iters, 50U) << c.name;
    EXPECT_EQ(core.solver()->max_al_iters, 20U) << c.name;
    EXPECT_EQ(core.ClearedMultipliers(), 0U) << "a stance window has no inactive multiplier";
  }
}

// The shipped online budget and period on the standing window, through the
// phase manager (zero command, the stance gait, the state the previous plan's
// node at the shift): the standing solve converges, then every online tick's
// consumed segment passes the shipped publish rule (one AL iteration never
// reports convergence). A guard against gross failures only: the window sits
// orders below the rule on any budget, so the real-time sim judges the budget.
// Semi-implicit Euler (the shipped integrator) only.
TEST_P(KinodynamicReceding, TheShippedOnlineBudgetKeepsTheStandingWindowPublishable) {
  if (std::string_view(GetParam()) != "semi_implicit_euler") {
    GTEST_SKIP() << "the shipped integrator only";
  }
  MPCSolverConfig solver = RegisteredSolver();
  solver.max_iters = tu::kShippedOnlineMaxIters;
  solver.max_online_al_iters = tu::kShippedOnlineAlIters;
  const std::vector<GridTickReport> ticks =
      GridRun(solver, std::vector<int>(kTicks, tu::kShippedPeriodNodes), /*stance=*/true, nullptr,
              KinodynamicOCP::StageKinematics::kShared, tu::kShippedPeriodNodes);
  ASSERT_EQ(ticks.size(), static_cast<std::size_t>(kTicks + 1));
  ReportGrid(GetParam(), "stance window, shipped online budget", ticks);
  EXPECT_TRUE(ticks[0].converged) << "the standing solve";
  double worst = 0.0;
  double horizon_worst = 0.0;
  int most = 0;
  for (std::size_t n = 1; n < ticks.size(); ++n) {
    worst = std::max(worst, ticks[n].consumed);
    horizon_worst = std::max(horizon_worst, ticks[n].prim);
    most = std::max(most, ticks[n].iterations);
    EXPECT_LE(ticks[n].consumed, tu::kShippedPublishPrimTol)
        << "tick " << n << ": " << ticks[n].iterations << " iterations";
  }
  std::printf(
      "[report] %s stance window, budget %d / %d every %d nodes: consumed max %.3g (rule %.3g), "
      "whole horizon max %.3g, last %.3g, most %d iterations\n",
      GetParam(), solver.max_iters, solver.max_online_al_iters, tu::kShippedPeriodNodes, worst,
      tu::kShippedPublishPrimTol, horizon_worst, ticks.back().prim, most);
}

// ── Receding walk window ────────────────────────────────────────────────────

// The 50 / 20 window's premise, from the schedule alone: its mode grid flips
// (the case the inactive-multiplier hook is for). A gait change that moves
// every switch off the node grid fails here, not as a silent zero below.
TEST_P(KinodynamicReceding, TheRecedingWalkWindowsScheduleFlips) {
  EXPECT_GT(ScheduleFlips(kWalkLead), 0) << "lead " << kWalkLead;
}

// 50 iterations and 20 AL iterations a tick (the standing budget the same, so
// every tick runs as the probe's did), on the window led by kWalkLead; every
// tick's inactive-row multipliers are +0.0 after the solve.
TEST_P(KinodynamicReceding, TheWalkWindowRecedesWithinThePreregisteredBounds) {
  MPCSolverConfig solver = RegisteredSolver();
  solver.max_initial_iters = 50;
  const std::vector<Tick> ticks =
      Run(solver, nullptr, KinodynamicOCP::QuadraticCostDerivatives::kStructured, kWalkLead);
  ASSERT_EQ(ticks.size(), static_cast<std::size_t>(kTicks + 1));
  Report(GetParam(), ticks);

  int conv = 0;
  int sum_it = 0;
  int flips = 0;
  std::size_t cleared = 0;
  double worst_conv_prim = 0.0;
  std::vector<double> its;
  for (std::size_t n = 1; n < ticks.size(); ++n) {
    const Tick& t = ticks[n];
    conv += t.converged ? 1 : 0;
    sum_it += t.iterations;
    its.push_back(t.iterations);
    flips += t.mode_flips;
    cleared += t.cleared;
    if (t.converged) {
      worst_conv_prim = std::max(worst_conv_prim, t.prim);
    }
  }
  std::printf(
      "[report] %s receding (50 / 20): converged %d / %d, sum %d iterations, median %.1f, "
      "converged prim max %.3g, mode flips %d, multipliers cleared %zu\n",
      GetParam(), conv, kTicks, sum_it, Median(its), worst_conv_prim, flips, cleared);
  for (std::size_t n = 0; n < ticks.size(); ++n) {
    EXPECT_EQ(ticks[n].inactive_nonzero, 0) << "tick " << n;
  }
  EXPECT_GT(flips, 0) << "the window's mode grid flips (the case the hook is for)";
  EXPECT_GT(cleared, 0U) << "the hook fired on the flipped nodes";
  EXPECT_LE(worst_conv_prim, solver.prim_tol);
  if (std::string_view(GetParam()) == "semi_implicit_euler") {
    // 0.9 × the probe's converged ticks: 15 when registered, 13 once the shipped
    // walk swings its arms from opened shoulder rolls (W8-F05, #1155).
    EXPECT_GE(conv, 12) << "0.9 × 13";
    EXPECT_LE(sum_it, 2 * 1675);
    EXPECT_LE(Median(its), 2 * 50.0);
  }
}

// The online budget (10 iterations, 1 AL iteration a tick) after a standing
// solve that converged: the tick primal residual — what a publish rule on the
// residual would read — has a median within twice the probe's. The maximum is
// reported, not gated: the first probe's bound (2 × 1.25) was registered with
// a tick 0 of 50 iterations and 1 AL iteration, which the standing solve
// replaces (it runs until converged); from the converged tick 0 the
// semi-implicit Euler run peaks at 3.2 on a tick where the next phase enters
// the horizon's end — the publish rule's question, not this gate's. This test
// runs the window without kWalkLead: it does not need a flip, and on the led
// window RK2's standing solve does not converge in its budget.
TEST_P(KinodynamicReceding, TheOnlineBudgetKeepsThePrimalResidualBounded) {
  MPCSolverConfig solver = RegisteredSolver();
  solver.max_iters = 10;
  solver.max_online_al_iters = 1;
  const std::vector<Tick> ticks = Run(solver);
  ASSERT_EQ(ticks.size(), static_cast<std::size_t>(kTicks + 1));
  Report(GetParam(), ticks);
  ASSERT_TRUE(ticks[0].converged) << "the standing solve converges first";
  std::vector<double> prims;
  int initial = 0;
  for (std::size_t n = 1; n < ticks.size(); ++n) {
    prims.push_back(ticks[n].prim);
    initial += ticks[n].initial ? 1 : 0;
  }
  const double med = Median(prims);
  const double worst = *std::max_element(prims.begin(), prims.end());
  std::printf(
      "[report] %s online (10 / 1): standing tick %d iterations; prim median %.3g, max %.3g\n",
      GetParam(), ticks[0].iterations, med, worst);
  EXPECT_EQ(initial, 0) << "every tick after the converged one is online";
  for (std::size_t n = 0; n < ticks.size(); ++n) {
    EXPECT_EQ(ticks[n].inactive_nonzero, 0) << "tick " << n;
  }
  if (std::string_view(GetParam()) == "semi_implicit_euler") {
    EXPECT_LE(med, 2 * 0.540);
    std::printf("[report] %s online prim max %.3g against the first probe's 2 x 1.25 (not gated)\n",
                GetParam(), worst);
  }
}

// ── Node grid ───────────────────────────────────────────────────────────────

// The warm start shifted by 0, 1 and 2 nodes, bit for bit, on the iterates of
// a solved walk tick and the references of the next one: one node moves
// every state, control and multiplier one node on (node 0 the measured
// state, the last control the node's reference control, the multipliers
// that fall off zero); none moves only node 0's state; two is two rotations,
// the last two controls their reference controls.
TEST_P(KinodynamicReceding, TheWarmStartShiftsByTheGridIndexBitForBit) {
  rtc::mpc::LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
  std::int64_t index = WalkManager(m);
  MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
  PhaseContext ctx = GridTick(m, snap, m.grid().Time(index));
  KinodynamicOCP ocp(cfg_);
  Core core;
  MPCSolverConfig solver = RegisteredSolver();
  solver.max_iters = 10;
  solver.max_online_al_iters = 1;
  ASSERT_EQ(core.Init(solver, handler_, {}, ctx, ocp), rtc::mpc::MPCInitError::kNoError);
  auto out = std::make_unique<MPCSolution>();
  for (int n = 0; n < 4; ++n) {  // the standing solve, then three online ticks
    if (n > 0) {
      snap = NodeOne(*out, static_cast<std::uint64_t>(n));
      ctx = GridTick(m, snap, m.grid().Time(++index));
    }
    ASSERT_EQ(core.Solve(ctx, snap, ocp, *out), MPCSolveError::kNoError);
  }
  const aligator::ResultsTpl<double> prev = core.solver()->results_;
  snap = NodeOne(*out, 9);
  Eigen::VectorXd x0(model_.nq + model_.nv);
  x0.head(model_.nq) = Eigen::Map<const Eigen::VectorXd>(snap.q.data(), model_.nq);
  x0.tail(model_.nv) = Eigen::Map<const Eigen::VectorXd>(snap.v.data(), model_.nv);
  const auto& problem = ocp.problem();
  const std::size_t N = prev.us.size();
  Eigen::VectorXd u_ref(prev.us[0].size());

  // No node: the same grid index (the OCP still holds its references).
  aligator::ResultsTpl<double> r0 = prev;
  EXPECT_EQ(rtc::mpc::internal::ShiftWarmStart(r0, problem, x0, 0, ocp, true), 0U);
  EXPECT_TRUE(SameBits(r0.xs[0], x0));
  EXPECT_TRUE(SameBits(r0.xs, prev.xs, 1));
  EXPECT_TRUE(SameBits(r0.us, prev.us));
  EXPECT_TRUE(SameBits(r0.vs, prev.vs));
  EXPECT_TRUE(SameBits(r0.lams, prev.lams));

  ctx = GridTick(m, snap, m.grid().Time(index + 1));
  ASSERT_EQ(ocp.UpdateReferences(ctx), rtc::mpc::OCPBuildError::kNoError);

  // One node.
  aligator::ResultsTpl<double> r1 = prev;
  EXPECT_EQ(rtc::mpc::internal::ShiftWarmStart(r1, problem, x0, 1, ocp, true), 0U)
      << "a grid node keeps its mode: nothing to clear";
  EXPECT_TRUE(SameBits(r1.xs[0], x0));
  for (std::size_t i = 1; i < N; ++i) {
    EXPECT_TRUE(SameBits(r1.xs[i], prev.xs[i + 1])) << "xs " << i;
    EXPECT_TRUE(SameBits(r1.lams[i], prev.lams[i + 1])) << "lams " << i;
  }
  EXPECT_TRUE(SameBits(r1.xs[N], prev.xs[N])) << "the appended node keeps the last state";
  for (std::size_t i = 0; i + 1 < N; ++i) {
    EXPECT_TRUE(SameBits(r1.us[i], prev.us[i + 1])) << "us " << i;
    EXPECT_TRUE(SameBits(r1.vs[i], prev.vs[i + 1])) << "vs " << i;
  }
  ASSERT_TRUE(ocp.ReferenceControl(static_cast<int>(N - 1), u_ref));
  EXPECT_TRUE(SameBits(r1.us[N - 1], u_ref)) << "the appended control is the reference";
  EXPECT_TRUE(AllPlusZero(r1.vs[N - 1]));
  EXPECT_TRUE(AllPlusZero(r1.lams[0]));
  EXPECT_TRUE(AllPlusZero(r1.lams[N]));

  // Two nodes: two rotations, the references two nodes on.
  ASSERT_EQ(ocp.UpdateReferences(GridTick(m, snap, m.grid().Time(index + 2))),
            rtc::mpc::OCPBuildError::kNoError);
  aligator::ResultsTpl<double> r2 = prev;
  aligator::ResultsTpl<double> twice = prev;
  EXPECT_EQ(rtc::mpc::internal::ShiftWarmStart(r2, problem, x0, 2, ocp, false), 0U);
  twice.cycleAppend(problem, x0);
  twice.cycleAppend(problem, x0);
  EXPECT_TRUE(SameBits(r2.xs, twice.xs));
  EXPECT_TRUE(SameBits(r2.us, twice.us));
  EXPECT_TRUE(SameBits(r2.vs, twice.vs));
  EXPECT_TRUE(SameBits(r2.lams, twice.lams));
  aligator::ResultsTpl<double> r2s = prev;
  static_cast<void>(rtc::mpc::internal::ShiftWarmStart(r2s, problem, x0, 2, ocp, true));
  EXPECT_TRUE(SameBits(r2s.us, twice.us, 0, N - 2));
  for (std::size_t i = N - 2; i < N; ++i) {
    ASSERT_TRUE(ocp.ReferenceControl(static_cast<int>(i), u_ref));
    EXPECT_TRUE(SameBits(r2s.us[i], u_ref)) << "us " << i;
  }
  EXPECT_TRUE(SameBits(r2s.xs, twice.xs));
}

// Through the phase manager with the online budget, a tick each node (the
// rate 1/dt), as the receding run above: no mode flip, nothing cleared, no
// stage whose dynamics or constraint data miss its node's mode; the shift,
// the horizon offset and the per-node residuals as packed. Reports the tick
// primal residual of the whole horizon and of the consumed segment.
TEST_P(KinodynamicReceding, OnTheGridTheWalkRecedesWithoutAModeFlip) {
  MPCSolverConfig solver = RegisteredSolver();
  solver.max_iters = 10;
  solver.max_online_al_iters = 1;
  const std::vector<GridTickReport> ticks = GridRun(solver, std::vector<int>(kTicks, 1));
  ASSERT_EQ(ticks.size(), static_cast<std::size_t>(kTicks + 1));
  ReportGrid(GetParam(), "grid 1/dt", ticks);
  ASSERT_TRUE(ticks[0].converged) << "the standing solve converges first";
  std::vector<double> prims;
  std::vector<double> consumed;
  int flips = 0;
  int leaks = 0;
  std::size_t cleared = 0;
  for (std::size_t n = 0; n < ticks.size(); ++n) {
    const GridTickReport& t = ticks[n];
    if (n > 0) {
      prims.push_back(t.prim);
      consumed.push_back(t.consumed);
    }
    flips += t.flips;
    leaks += t.leaks;
    cleared += t.cleared;
    EXPECT_EQ(t.node_max, t.prim) << "tick " << n << ": the node residuals' max is prim_inf";
    EXPECT_LE(std::abs(static_cast<double>(t.offset_ns) - t.expected_offset_ns), 1.0)
        << "tick " << n;
  }
  std::printf(
      "[report] %s grid 1/dt online (10 / 1): standing %d iterations; prim median %.3g max %.3g; "
      "consumed (nodes 0-2) median %.3g max %.3g; flips %d cleared %zu leaks %d\n",
      GetParam(), ticks[0].iterations, Median(prims), *std::max_element(prims.begin(), prims.end()),
      Median(consumed), *std::max_element(consumed.begin(), consumed.end()), flips, cleared, leaks);
  EXPECT_EQ(flips, 0);
  EXPECT_EQ(cleared, 0U);
  EXPECT_EQ(leaks, 0);
}

// Shifts of 1, 1, 0, 2, 1 nodes (an MPC period that is not the node spacing,
// an overrun): the warm start moves by each, and through the touch-downs and
// lift-offs of a stride no stage carries another mode than its node's.
TEST_P(KinodynamicReceding, OnTheGridAnOverrunShiftsTwoNodesWithoutAModeLeak) {
  MPCSolverConfig solver = RegisteredSolver();
  solver.max_iters = 10;
  solver.max_online_al_iters = 1;
  constexpr std::array<int, 5> kPattern = {1, 1, 0, 2, 1};
  std::vector<int> shifts;
  for (std::size_t n = 0; n < 50; ++n) {
    shifts.push_back(kPattern[n % kPattern.size()]);
  }
  const std::vector<GridTickReport> ticks = GridRun(solver, shifts);
  ASSERT_EQ(ticks.size(), shifts.size() + 1);
  ReportGrid(GetParam(), "grid overrun", ticks);
  std::vector<double> prims;
  std::vector<double> consumed;
  int flips = 0;
  int leaks = 0;
  for (std::size_t n = 0; n < ticks.size(); ++n) {
    flips += ticks[n].flips;
    leaks += ticks[n].leaks;
    if (n > 0) {
      prims.push_back(ticks[n].prim);
      consumed.push_back(ticks[n].consumed);
    }
  }
  std::printf(
      "[report] %s grid overrun online (10 / 1): prim median %.3g max %.3g; consumed median "
      "%.3g max %.3g; flips %d leaks %d\n",
      GetParam(), Median(prims), *std::max_element(prims.begin(), prims.end()), Median(consumed),
      *std::max_element(consumed.begin(), consumed.end()), flips, leaks);
  EXPECT_EQ(flips, 0);
  EXPECT_EQ(leaks, 0);
}

// The index the core shifts by: the difference from its last solve's, a cold
// seed (shift 0) when it goes back or moves by the horizon or more, one node
// a tick off the grid (and no horizon offset).
TEST_P(KinodynamicReceding, AGridIndexThatGoesBackOrJumpsColdSeeds) {
  rtc::mpc::LocomotionPhaseManager m(handler_, tu::ShippedContactNames());
  const std::int64_t index = WalkManager(m);
  const MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
  PhaseContext ctx = GridTick(m, snap, m.grid().Time(index));
  KinodynamicOCP ocp(cfg_);
  Core core;
  MPCSolverConfig solver = RegisteredSolver();
  solver.max_iters = 3;
  solver.max_initial_iters = 3;
  solver.max_online_al_iters = 1;
  ASSERT_EQ(core.Init(solver, handler_, {}, ctx, ocp), rtc::mpc::MPCInitError::kNoError);
  auto out = std::make_unique<MPCSolution>();
  ASSERT_EQ(core.Solve(ctx, snap, ocp, *out), MPCSolveError::kNoError);
  EXPECT_EQ(core.LastNodeShift(), 0) << "the first solve is cold";

  struct Case {
    const char* name;
    std::int64_t delta;
    int shift;
  };

  for (const Case c : {Case{"same node", 0, 0}, Case{"next node", 1, 1}, Case{"three", 3, 3},
                       Case{"back", -1, 0}, Case{"a horizon on", tu::kKinoNodes, 0},
                       Case{"just inside", tu::kKinoNodes - 1, tu::kKinoNodes - 1}}) {
    PhaseContext moved = ctx;
    moved.node_index = ctx.node_index + c.delta;
    ASSERT_EQ(core.Solve(moved, snap, ocp, *out), MPCSolveError::kNoError) << c.name;
    EXPECT_EQ(core.LastNodeShift(), c.shift) << c.name;
    EXPECT_EQ(out->cold_start, c.delta < 0 || c.delta >= tu::kKinoNodes) << c.name;
    ctx = moved;
  }
  PhaseContext off = ctx;
  off.on_node_grid = false;
  ASSERT_EQ(core.Solve(off, snap, ocp, *out), MPCSolveError::kNoError);
  EXPECT_EQ(core.LastNodeShift(), 1) << "off the grid: a node a tick";
  EXPECT_EQ(out->horizon_offset_ns, 0);
}

// ── One budget for every solve (P0 of #1168) ─────────────────────────────────

/// The shipped budget as one budget: every solve on the online pair (6
/// iterations, 1 AL iteration), the standing one included. `max_initial_iters
/// 0` makes the standing solve's ceiling `max_iters`; one AL iteration never
/// reports convergence, so the pending standing flag stays up and the budget
/// stays the same either way.
MPCSolverConfig SingleBudget() {
  MPCSolverConfig s = RegisteredSolver();
  s.max_iters = tu::kShippedOnlineMaxIters;
  s.max_al_iters = 1;
  s.max_initial_iters = 0;
  s.max_online_al_iters = tu::kShippedOnlineAlIters;
  return s;
}

/// The budget shipped today: the standing solve on 300 iterations and 20 AL
/// iterations until it converges, then the online pair.
MPCSolverConfig ShippedLineage() {
  MPCSolverConfig s = RegisteredSolver();
  s.max_iters = tu::kShippedOnlineMaxIters;
  s.max_al_iters = 20;
  s.max_initial_iters = 300;
  s.max_online_al_iters = tu::kShippedOnlineAlIters;
  return s;
}

/// Distances between two plans over the nodes a two-node consumer reads.
struct PlanDistance {
  double q{0.0};       ///< max over the nodes of ‖Δq‖∞
  double v{0.0};       ///< max over the nodes of ‖Δv‖∞
  double lambda{0.0};  ///< max over the nodes of ‖Δλ‖∞ / (m·g)

  [[nodiscard]] bool Within(const PlanDistance& eps) const {
    return q <= eps.q && v <= eps.v && lambda <= eps.lambda;
  }
};

constexpr int kConsumedNodes = 4;  ///< nodes 0..3: ConsumedPrimInf's window at period 2

/// The distance between node `j + offset_a` of @p a and node `j` of @p b over
/// the consumed nodes j.
PlanDistance Distance(const PlanSlice& a, const PlanSlice& b, int offset_a, double mg) {
  PlanDistance d;
  for (int j = 0; j < kConsumedNodes; ++j) {
    const auto ka = static_cast<std::size_t>(j + offset_a);
    const auto kb = static_cast<std::size_t>(j);
    for (int i = 0; i < a.nq; ++i) {
      const auto ii = static_cast<std::size_t>(i);
      d.q = std::max(d.q, std::abs(a.q[ka][ii] - b.q[kb][ii]));
    }
    for (int i = 0; i < a.nv; ++i) {
      const auto ii = static_cast<std::size_t>(i);
      d.v = std::max(d.v, std::abs(a.v[ka][ii] - b.v[kb][ii]));
    }
    for (int i = 0; i < a.n_contact_vars; ++i) {
      const auto ii = static_cast<std::size_t>(i);
      d.lambda = std::max(d.lambda, std::abs(a.lambda[ka][ii] - b.lambda[kb][ii]) / mg);
    }
  }
  return d;
}

/// The p99 of @p v (the value at index 0.99·(n−1), rounded).
double P99(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v[static_cast<std::size_t>(std::lround(0.99 * static_cast<double>(v.size() - 1)))];
}

/// ε of a lineage: the p99 over its online solves of the jump a consumer
/// sees between consecutive plans — plan k's node j + 2 against plan k + 1's
/// node j over the consumed nodes.
PlanDistance ConsecutiveJumpP99(const std::vector<PlanSlice>& plans, std::size_t from, double mg) {
  std::vector<double> q;
  std::vector<double> v;
  std::vector<double> lambda;
  for (std::size_t k = from; k + 1 < plans.size(); ++k) {
    const PlanDistance d = Distance(plans[k], plans[k + 1], tu::kShippedPeriodNodes, mg);
    q.push_back(d.q);
    v.push_back(d.v);
    lambda.push_back(d.lambda);
  }
  PlanDistance eps;
  eps.q = P99(q);
  eps.v = P99(v);
  eps.lambda = P99(lambda);
  return eps;
}

// The question PR 0-ii of #1168 rests on, registered on that issue before it
// was measured: with one budget for every solve — the shipped online pair —
// does a cold start reach a plan the shipped publish rule (consumed nodes'
// primal residual ≤ publish_prim_tol) lets through within the plan-age cap,
// and is that plan as close to today's lineage (the standing solve on 300 /
// 20, then the online pair) as today's consecutive plans are to each other?
// Five scenarios, each 40 solves of a two-node period from the stand pose:
// stance cold (A), stance from a moving base (B), the walk window cold at
// v_x 0.5 (C1) and 0 (C0), and a re-seed 20 solves into the walk (D, the
// state arriving with a new state_epoch). Pass: the first publishable index
// k* ≤ 2 (A, B) or ≤ 3 (C1, C0, D after the re-seed; (k+1)·70 ms within the
// 0.35 s cap with a solve to spare), and at k* the distance to today's plan
// at the same index is within ε, the p99 of today's consecutive-plan jump in
// that scenario. Must fail: the cold seed after one iteration (max_iters 1,
// index 0) is farther than ε from today's plan on the walk scenarios — else
// the distance has no power. Times are this host's and are reported only.
TEST_P(KinodynamicReceding, OneBudgetReachesAPublishablePlanFromColdWithinTheRegisteredBounds) {
  if (std::string_view(GetParam()) != "semi_implicit_euler") {
    GTEST_SKIP() << "the shipped integrator only";
  }
  const double mg = pinocchio::computeTotalMass(model_) * model_.gravity.linear().norm();
  constexpr int kReseedAt = 20;

  struct Scenario {
    const char* name;
    bool stance;
    double v_x;
    double base_v0;
    int reseed_at;
    int k_bound;
    bool walk;  ///< the must-fail control runs on it
  };

  const Scenario scenarios[] = {
      {"A stance cold", true, 0.0, 0.0, -1, 2, false},
      {"B stance, moving base", true, 0.0, 0.05, -1, 2, false},
      {"C1 walk cold v_x 0.5", false, 0.5, 0.0, -1, 3, true},
      {"C0 walk cold v_x 0", false, 0.0, 0.0, -1, 3, true},
      {"D walk re-seed", false, 0.5, 0.0, kReseedAt, 3, false},
  };
  const std::vector<int> shifts(kTicks, tu::kShippedPeriodNodes);

  for (const Scenario& sc : scenarios) {
    GridOptions opts;
    opts.stance = sc.stance;
    opts.period_nodes = tu::kShippedPeriodNodes;
    opts.v_x = sc.v_x;
    opts.base_v0 = sc.base_v0;
    opts.reseed_at = sc.reseed_at;
    std::vector<PlanSlice> one_plans;
    std::vector<PlanSlice> today_plans;
    opts.plans = &one_plans;
    const std::vector<GridTickReport> one = GridRun(SingleBudget(), shifts, opts);
    opts.plans = &today_plans;
    const std::vector<GridTickReport> today = GridRun(ShippedLineage(), shifts, opts);
    ASSERT_EQ(one.size(), static_cast<std::size_t>(kTicks + 1)) << sc.name;
    ASSERT_EQ(today.size(), static_cast<std::size_t>(kTicks + 1)) << sc.name;

    const std::size_t origin = sc.reseed_at > 0 ? static_cast<std::size_t>(sc.reseed_at) : 0;
    for (std::size_t n = 0; n < one.size(); ++n) {
      const GridTickReport& t = one[n];
      const GridTickReport& s = today[n];
      const PlanDistance d = Distance(one_plans[n], today_plans[n], 0, mg);
      std::printf(
          "[report] P0 %s solve %2zu: one budget: cold %d conv %d it %d prim %.3g dual %.3g "
          "consumed %.3g %s %.1f ms | today: initial %d conv %d it %3d consumed %.3g %.1f ms | "
          "dq %.3g dv %.3g dlam %.3g\n",
          sc.name, n, t.cold ? 1 : 0, t.converged ? 1 : 0, t.iterations, t.prim, t.dual, t.consumed,
          !(t.cold && !t.converged) && t.consumed <= tu::kShippedPublishPrimTol ? "PUB"
                                                                                : "withheld",
          t.ms, s.initial ? 1 : 0, s.converged ? 1 : 0, s.iterations, s.consumed, s.ms, d.q, d.v,
          d.lambda);
    }

    // (i) the first publishable index from the origin (the re-seed tick in D),
    // by the thread's rule: a cold-seeded solve that did not converge is
    // withheld whatever its residual, then the consumed nodes' residual.
    int k_star = -1;
    for (std::size_t n = origin; n < one.size(); ++n) {
      if (!(one[n].cold && !one[n].converged) && one[n].consumed <= tu::kShippedPublishPrimTol) {
        k_star = static_cast<int>(n - origin);
        break;
      }
    }
    EXPECT_TRUE(one[origin].cold) << sc.name << ": the origin solve is cold-seeded";
    ASSERT_GE(k_star, 0) << sc.name << ": no publishable plan in " << kTicks + 1 << " solves";
    EXPECT_LE(k_star, sc.k_bound) << sc.name;

    // (ii) at k*, within ε of today's plan at the same index.
    const PlanDistance eps = ConsecutiveJumpP99(today_plans, origin + 1, mg);
    const std::size_t at = origin + static_cast<std::size_t>(k_star);
    const PlanDistance d = Distance(one_plans[at], today_plans[at], 0, mg);
    EXPECT_LE(d.q, eps.q) << sc.name;
    EXPECT_LE(d.v, eps.v) << sc.name;
    EXPECT_LE(d.lambda, eps.lambda) << sc.name;
    std::printf(
        "[report] P0 %s: k* %d (bound %d); at k*: dq %.3g dv %.3g dlam %.3g; eps (today's "
        "consecutive jump p99) q %.3g v %.3g lambda %.3g; today's origin solve: %d iterations "
        "%.1f ms conv %d\n",
        sc.name, k_star, sc.k_bound, d.q, d.v, d.lambda, eps.q, eps.v, eps.lambda,
        today[origin].iterations, today[origin].ms, today[origin].converged ? 1 : 0);

    // Must fail: the cold seed after one iteration is not within ε on the walk.
    if (sc.walk) {
      MPCSolverConfig one_iter = SingleBudget();
      one_iter.max_iters = 1;
      std::vector<PlanSlice> seed_plans;
      opts.plans = &seed_plans;
      const std::vector<GridTickReport> seed = GridRun(one_iter, std::vector<int>{}, opts);
      ASSERT_EQ(seed.size(), 1U) << sc.name;
      const PlanDistance ds = Distance(seed_plans[0], today_plans[0], 0, mg);
      std::printf("[report] P0 %s control (1 iteration, index 0): dq %.3g dv %.3g dlam %.3g\n",
                  sc.name, ds.q, ds.v, ds.lambda);
      EXPECT_FALSE(ds.Within(eps)) << sc.name << ": the distance cannot tell a cold seed";
    }
  }
}

INSTANTIATE_TEST_SUITE_P(Integrators, KinodynamicReceding,
                         ::testing::Values("semi_implicit_euler", "rk2"));

}  // namespace

// ── Stage-parallel solves ───────────────────────────────────────────────────

namespace {

bool AllFinite(const std::vector<Eigen::VectorXd>& v) {
  return std::all_of(v.begin(), v.end(), [](const Eigen::VectorXd& x) { return x.allFinite(); });
}

bool AllFinite(const std::vector<Eigen::MatrixXd>& v) {
  return std::all_of(v.begin(), v.end(), [](const Eigen::MatrixXd& x) { return x.allFinite(); });
}

bool SameBits(const std::vector<Eigen::MatrixXd>& a, const std::vector<Eigen::MatrixXd>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].rows() != b[i].rows() || a[i].cols() != b[i].cols() ||
        std::memcmp(a[i].data(), b[i].data(),
                    sizeof(double) * static_cast<std::size_t>(a[i].size())) != 0) {
      return false;
    }
  }
  return true;
}

bool SameBits(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

/// Every quantity a solve hands on — iterates, multipliers, every node's
/// feedback gain, the residuals, the cost, the per-node defects — finite on
/// both sides first (two NaNs can share a bit pattern), then bit for bit.
void ExpectSameSolve(const Capture& par, const Capture& ser, const std::string& where) {
  const auto& p = par.results;
  const auto& q = ser.results;
  const auto finite = [](const aligator::ResultsTpl<double>& r) {
    return AllFinite(r.xs) && AllFinite(r.us) && AllFinite(r.vs) && AllFinite(r.lams) &&
           AllFinite(r.gains_) && std::isfinite(r.prim_infeas) && std::isfinite(r.dual_infeas) &&
           std::isfinite(r.traj_cost_) && std::isfinite(r.merit_value_);
  };
  EXPECT_TRUE(finite(p) && finite(q)) << where << ": a non-finite value";
  for (std::size_t k = 0; k < par.node_prim_inf.size(); ++k) {
    EXPECT_TRUE(std::isfinite(par.node_prim_inf[k]) && std::isfinite(ser.node_prim_inf[k]))
        << where << ": node_prim_inf " << k << " is not finite";
  }
  EXPECT_TRUE(SameBits(p.xs, q.xs)) << where << ": xs";
  EXPECT_TRUE(SameBits(p.us, q.us)) << where << ": us";
  EXPECT_TRUE(SameBits(p.vs, q.vs)) << where << ": vs";
  // Every constraint's multipliers, the terminal one included (one per node).
  EXPECT_EQ(p.lams.size(), p.xs.size()) << where << ": lams";
  EXPECT_TRUE(SameBits(p.lams, q.lams)) << where << ": lams";
  EXPECT_TRUE(SameBits(p.gains_, q.gains_)) << where << ": gains (every node)";
  EXPECT_TRUE(SameBits(p.prim_infeas, q.prim_infeas)) << where << ": prim";
  EXPECT_TRUE(SameBits(p.dual_infeas, q.dual_infeas)) << where << ": dual";
  EXPECT_TRUE(SameBits(p.traj_cost_, q.traj_cost_)) << where << ": cost";
  // The next tick's line search starts from it.
  EXPECT_TRUE(SameBits(p.merit_value_, q.merit_value_)) << where << ": merit";
  EXPECT_EQ(p.num_iters, q.num_iters) << where << ": iterations";
  EXPECT_EQ(p.conv, q.conv) << where << ": converged";
  EXPECT_EQ(std::memcmp(par.node_prim_inf.data(), ser.node_prim_inf.data(),
                        sizeof(double) * par.node_prim_inf.size()),
            0)
      << where << ": node_prim_inf";
}

/// Sets the calling thread's OpenMP ICVs as the MPC thread does (dynamic off,
/// default team size 1 — the solver's regions name their own size) and puts
/// them back on scope exit, however the test leaves.
class SolverThreadIcvs {
 public:
  SolverThreadIcvs() noexcept : max_(omp_get_max_threads()), dynamic_(omp_get_dynamic()) {
    omp_set_num_threads(1);
    omp_set_dynamic(0);
  }

  ~SolverThreadIcvs() {
    omp_set_num_threads(max_);
    omp_set_dynamic(dynamic_);
  }

  SolverThreadIcvs(const SolverThreadIcvs&) = delete;
  SolverThreadIcvs& operator=(const SolverThreadIcvs&) = delete;
  SolverThreadIcvs(SolverThreadIcvs&&) = delete;
  SolverThreadIcvs& operator=(SolverThreadIcvs&&) = delete;

 private:
  int max_;
  int dynamic_;
};

/// The online budget the serial reference was measured with on the dev PC.
MPCSolverConfig MeasuredOnlineBudget() {
  MPCSolverConfig s = RegisteredSolver();
  s.max_iters = 3;
  s.max_online_al_iters = 1;
  return s;
}

}  // namespace

// The comparison itself: two solves that share a NaN share its bit pattern, so
// bit equality alone would call them the same. The finiteness checks must fail.
TEST(StageParallelComparison, TwoNonFiniteSolvesAreNotTheSameSolve) {
  Capture a;
  a.results.xs = {Eigen::VectorXd::Constant(3, std::numeric_limits<double>::quiet_NaN())};
  a.results.us = {Eigen::VectorXd::Zero(2)};
  a.results.vs = a.results.us;
  a.results.lams = a.results.us;
  a.results.gains_ = {Eigen::MatrixXd::Zero(2, 3)};
  const Capture b = a;
  EXPECT_NONFATAL_FAILURE(ExpectSameSolve(a, b, "nan"), "non-finite");
}

// The stage loops (evaluation and derivatives) split across threads; the
// Riccati recursion stays serial. Every tick of the walk window — the standing
// solve, the online ticks, and the mode changes the window crosses — must
// reproduce the serial solve exactly, for every thread count the layout can
// give the solver. The window is the one led by kWalkLead, so it crosses mode
// flips. The threads here are unpinned: equivalence does not depend on
// placement, and a host with fewer cores still runs every count.
TEST_P(KinodynamicReceding, StageParallelSolvesReproduceTheSerialSolveBitForBit) {
  const SolverThreadIcvs icvs;
  const MPCSolverConfig serial = MeasuredOnlineBudget();
  constexpr auto kDerivatives = KinodynamicOCP::QuadraticCostDerivatives::kStructured;
  std::vector<Capture> ser;
  const std::vector<Tick> ser_ticks = Run(serial, &ser, kDerivatives, kWalkLead);
  ASSERT_EQ(ser.size(), static_cast<std::size_t>(kTicks + 1));
  int flips = 0;
  for (const Tick& t : ser_ticks) {
    flips += t.mode_flips;
  }
  std::printf("[report] %s: %zu ticks compared per thread count, %d mode flips crossed\n",
              GetParam(), ser.size(), flips);
  EXPECT_GT(flips, 0) << "the window crosses mode flips";
  for (int n = 2; n <= 4; ++n) {
    MPCSolverConfig par = serial;
    par.num_threads = n;
    par.worker_capacity = n - 1;
    std::vector<Capture> got;
    const std::vector<Tick> ticks = Run(par, &got, kDerivatives, kWalkLead);
    ASSERT_EQ(got.size(), ser.size()) << n << " threads";
    for (std::size_t k = 0; k < got.size(); ++k) {
      ExpectSameSolve(got[k], ser[k], std::to_string(n) + " threads, tick " + std::to_string(k));
      EXPECT_EQ(ticks[k].inactive_nonzero, 0) << n << " threads, tick " << k;
    }
  }
}

// The OCP built with the shared stage kinematics (one ODE and one frame
// kinematics evaluation per stage) against the per-residual build, through
// the handler core on the grid: the stance window and the walk window, 40
// ticks each on the registered budgets (the standing solve, then the online
// ticks), the shared build serial and on two solver threads. Every tick's
// solve — iterates, multipliers, gains, residuals, iterations — is the same
// bits as the serial per-residual solve (which the stage-parallel test above
// holds equal to its own two-thread solve). The shipped integrator only, to
// keep the run short: one stage's bits on both integrators are
// test_kinodynamic_ocp's SharedStageKinematicsGiveThePerResidualBits.
TEST_P(KinodynamicReceding, SharedStageKinematicsReproduceThePerResidualSolveBitForBit) {
  if (std::string(GetParam()) != "semi_implicit_euler") {
    GTEST_SKIP() << "run on the shipped integrator only";
  }
  const SolverThreadIcvs icvs;
  for (const bool stance : {true, false}) {
    const std::string window = stance ? "stance" : "walk";
    std::vector<Capture> own;
    const std::vector<GridTickReport> own_ticks =
        GridRun(RegisteredSolver(), std::vector<int>(kTicks, 1), stance, &own,
                KinodynamicOCP::StageKinematics::kPerResidual);
    ASSERT_EQ(own.size(), static_cast<std::size_t>(kTicks + 1)) << window;
    for (const int n : {1, 2}) {
      MPCSolverConfig solver = RegisteredSolver();
      solver.num_threads = n;
      solver.worker_capacity = n - 1;
      std::vector<Capture> shared;
      const std::vector<GridTickReport> ticks =
          GridRun(solver, std::vector<int>(kTicks, 1), stance, &shared,
                  KinodynamicOCP::StageKinematics::kShared);
      const std::string what = window + ", " + std::to_string(n) + " thread(s)";
      ASSERT_EQ(shared.size(), own.size()) << what;
      int iterations = 0;
      for (std::size_t k = 0; k < own.size(); ++k) {
        ExpectSameSolve(shared[k], own[k], what + ", tick " + std::to_string(k));
        EXPECT_EQ(ticks[k].iterations, own_ticks[k].iterations) << what << ", tick " << k;
        iterations += ticks[k].iterations;
      }
      std::printf("[report] %s shared stage kinematics, %s: %zu ticks, %d iterations\n", GetParam(),
                  what.c_str(), own.size(), iterations);
    }
  }
}

// The quality frame a non-bit-identical backend would be judged by, run live
// against the serial solve in the same process: the consumed segment's
// residual (nodes 0-2) by median and p95, and the standing solve's
// convergence and iteration count. Bit-identical solves meet it trivially;
// it is kept so the bound exists before such a backend does.
TEST_P(KinodynamicReceding, AStageParallelSolveMeetsTheSerialQualityFrame) {
  const SolverThreadIcvs icvs;
  const MPCSolverConfig serial = MeasuredOnlineBudget();
  MPCSolverConfig par = serial;
  par.num_threads = 4;
  par.worker_capacity = 3;
  std::vector<Capture> ser;
  std::vector<Capture> got;
  const std::vector<Tick> ser_ticks = Run(serial, &ser);
  const std::vector<Tick> par_ticks = Run(par, &got);
  ASSERT_EQ(got.size(), ser.size());
  ASSERT_FALSE(ser_ticks.empty());
  const auto consumed = [](const std::vector<Capture>& c) {
    std::vector<double> r;
    for (std::size_t k = 1; k < c.size(); ++k) {  // online ticks
      r.push_back(std::max({c[k].node_prim_inf[0], c[k].node_prim_inf[1], c[k].node_prim_inf[2]}));
    }
    return r;
  };
  const auto p95 = [](std::vector<double> v) {
    std::sort(v.begin(), v.end());
    return v[static_cast<std::size_t>(std::lround(0.95 * static_cast<double>(v.size() - 1)))];
  };
  const std::vector<double> rs = consumed(ser);
  const std::vector<double> rp = consumed(got);
  ASSERT_FALSE(rs.empty());
  const auto all_finite = [](const std::vector<double>& v) {
    return std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); });
  };
  ASSERT_TRUE(all_finite(rs) && all_finite(rp)) << "a consumed-segment residual is not finite";
  const double med_s = Median(rs);
  const double med_p = Median(rp);
  // Ratios against a floor: a serial residual at round-off makes a ratio noise.
  constexpr double kFloor = 1e-9;
  EXPECT_GE(med_p, 0.9 * std::max(med_s, kFloor) - kFloor);
  EXPECT_LE(med_p, 1.1 * std::max(med_s, kFloor));
  EXPECT_LE(p95(rp), 2.0 * std::max(p95(rs), kFloor));
  EXPECT_TRUE(ser_ticks.front().converged && par_ticks.front().converged);
  EXPECT_LE(par_ticks.front().prim, serial.prim_tol);
  const int it_s = ser_ticks.front().iterations;
  const int it_p = par_ticks.front().iterations;
  EXPECT_LE(std::abs(it_p - it_s), std::max(5, static_cast<int>(std::ceil(0.1 * it_s))));
  std::printf(
      "[report] %s quality: consumed median %.3g vs %.3g, p95 %.3g vs %.3g, standing "
      "iterations %d vs %d\n",
      GetParam(), med_p, med_s, p95(rp), p95(rs), it_p, it_s);
}

// Four threads on the shipped cost derivatives against four threads on
// Aligator's dense ones, measured in one process (same run A/B). Cheaper
// derivatives leave less work per iteration for the threads to share, so the
// serial / four-thread ratio drops while every time drops: the bound is on the
// four-thread time, not on the ratio, which is reported. Armed only with
// RTC_RUN_TIMING_GATES=1 (Release, idle host): an unarmed run reports and grades
// nothing. The control PCs judge the absolute times (soak).
TEST_P(KinodynamicReceding, FourThreadsSpeedUpTheOnlineBudget) {
  if (std::string(GetParam()) != "semi_implicit_euler") {
    GTEST_SKIP() << "measured on the shipped integrator only";
  }
  using Derivatives = KinodynamicOCP::QuadraticCostDerivatives;
  const char* armed = std::getenv("RTC_RUN_TIMING_GATES");
  const MPCSolverConfig serial = MeasuredOnlineBudget();
  MPCSolverConfig par = serial;
  par.num_threads = 4;
  par.worker_capacity = 3;
  const auto online_p50 = [](const std::vector<Tick>& ticks) {
    std::vector<double> ms;
    for (std::size_t k = 1; k < ticks.size(); ++k) {
      ms.push_back(ticks[k].ms);
    }
    return Median(ms);
  };
  const SolverThreadIcvs icvs;
  const double dense_ms = online_p50(Run(par, nullptr, Derivatives::kDense));
  const double par_ms = online_p50(Run(par, nullptr, Derivatives::kStructured));
  const double ser_ms = online_p50(Run(serial, nullptr, Derivatives::kStructured));
  ASSERT_GT(par_ms, 0.0);
  ASSERT_GT(dense_ms, 0.0);
  std::printf(
      "[report] online solve p50: 4 threads %.2f ms (dense derivatives %.2f ms, %.2f), serial "
      "%.2f ms, ratio %.2f (%s)\n",
      par_ms, dense_ms, par_ms / dense_ms, ser_ms, ser_ms / par_ms,
      armed != nullptr && std::string(armed) == "1" ? "armed" : "report only");
  if (armed == nullptr || std::string(armed) != "1") {
    return;
  }
  EXPECT_LE(par_ms, 1.10 * dense_ms);
}
