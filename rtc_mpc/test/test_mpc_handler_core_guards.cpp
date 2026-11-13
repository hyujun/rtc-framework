/// @file test_mpc_handler_core_guards.cpp
/// @brief The handler core's seeding and finiteness guards: the first solve
///        starts from the cold seed, a non-finite solution is never
///        published (and does not poison the next tick's warm start), and a
///        non-finite state is refused before it touches the solver.
///
/// The OCP here is a stub whose controls enter neither the dynamics nor the
/// cost (identity dynamics with an exact zero control Jacobian, constant
/// cost). From a guess that already satisfies the dynamics, ProxDDP meets its
/// tolerance on iteration 0 without a line search, so a non-finite control in
/// the warm start comes back from `run()` as a converged result — the same
/// shape as a contact slot the dynamics mask out. A non-finite state at a
/// late node also comes back from `run()` without a throw (measured with
/// Aligator 0.19.0), so the core cannot lean on the solver to reject either.
///
/// A second stub drives the core's budgets and its rebuild gate: its cost
/// chases a target that moves at every evaluation (no solve converges) until
/// told to stop, it counts its builds, and it can refuse a context as a
/// topology change while saying whether a rebuild could meet it.

#include "handler/internal/mpc_handler_core.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/ocp_handler_base.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"
#include "test_utils/alloc_counter.hpp"
#include "test_utils/panda_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/core/cost-abstract.hpp>
#include <aligator/core/explicit-dynamics.hpp>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/core/vector-space.hpp>
#include <aligator/modelling/costs/constant-cost.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

// Global operator new / delete overrides: test_utils/alloc_counter_overrides.hpp.
#include "test_utils/alloc_counter_overrides.hpp"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using rtc::mpc::MPCSolution;
using rtc::mpc::MPCSolveError;
using rtc::mpc::MPCStateSnapshot;
using rtc::mpc::SolverStatus;
using rtc::mpc::test_utils::kPandaTwoFingerYaml;
using rtc::mpc::test_utils::kPandaUrdf;
using rtc::mpc::test_utils::NeutralStateSnapshot;

constexpr int kHorizon = 8;
constexpr double kNodeDt = 0.01;
constexpr std::string_view kStubType{"inert_stub"};

/// x_{k+1} = x_k, with an exact zero control Jacobian: no control value can
/// reach the next state, not even as 0 · inf.
struct HoldStateDynamics : aligator::ExplicitDynamicsModelTpl<double> {
  using Base = aligator::ExplicitDynamicsModelTpl<double>;

  HoldStateDynamics(int nx, int nu) : Base(aligator::VectorSpaceTpl<double>(nx), nu) {}

  void forward(const ConstVectorRef& x, const ConstVectorRef& /*u*/, Data& data) const override {
    data.xnext_ = x;
  }

  void dForward(const ConstVectorRef& /*x*/, const ConstVectorRef& /*u*/,
                Data& data) const override {
    data.Jx().setIdentity();
    data.Ju().setZero();
  }
};

/// What the stub cost injects while armed.
enum class Poison { kNone, kValue };

/// Zero cost that, while armed, reports a NaN value: the trajectories stay
/// finite and only the shipped cost residual does not. The mode is shared by
/// every copy the problem makes of the cost. (An exception thrown from a cost
/// does not reach the caller of `run()` — it terminates — so the stub poisons
/// values instead of throwing.)
struct PoisonableCost : aligator::ConstantCostTpl<double> {
  PoisonableCost(const aligator::VectorSpaceTpl<double>& space, int nu,
                 std::shared_ptr<Poison> mode)
      : aligator::ConstantCostTpl<double>(space, nu, 0.0), mode_(std::move(mode)) {}

  void evaluate(const ConstVectorRef& x, const ConstVectorRef& u, CostData& data) const override {
    aligator::ConstantCostTpl<double>::evaluate(x, u, data);
    if (*mode_ == Poison::kValue) {
      data.value_ = std::numeric_limits<double>::quiet_NaN();
    }
  }

  std::shared_ptr<Poison> mode_;
};

/// OCP whose controls are inert: identity dynamics and a constant cost.
class InertOCP final : public rtc::mpc::OCPHandlerBase {
 public:
  /// What the cost injects from now on.
  void SetPoison(Poison mode) noexcept { *poison_ = mode; }

  rtc::mpc::OCPBuildError Build(const rtc::mpc::PhaseContext& /*ctx*/,
                                const rtc::mpc::RobotModelHandler& model,
                                const rtc::mpc::OCPLimits& /*limits*/) noexcept override {
    try {
      const int nx = model.nq() + model.nv();
      const int nu = model.nu();
      const aligator::VectorSpaceTpl<double> space(nx);
      const PoisonableCost cost(space, nu, poison_);
      std::vector<xyz::polymorphic<aligator::StageModelTpl<double>>> stages;
      for (int k = 0; k < kHorizon; ++k) {
        stages.emplace_back(aligator::StageModelTpl<double>(cost, HoldStateDynamics(nx, nu)));
      }
      problem_.emplace(Eigen::VectorXd::Zero(nx), stages, cost);
    } catch (...) {
      return rtc::mpc::OCPBuildError::kAligatorInstantiationFailure;
    }
    return rtc::mpc::OCPBuildError::kNoError;
  }

  rtc::mpc::OCPBuildError UpdateReferences(
      const rtc::mpc::PhaseContext& /*ctx*/) noexcept override {
    return rtc::mpc::OCPBuildError::kNoError;
  }

  bool Built() const noexcept override { return problem_.has_value(); }

  aligator::TrajOptProblemTpl<double>& problem() override { return *problem_; }

  int horizon_length() const noexcept override { return kHorizon; }

  std::string_view ocp_type() const noexcept override { return kStubType; }

 private:
  std::shared_ptr<Poison> poison_ = std::make_shared<Poison>(Poison::kNone);
  std::optional<aligator::TrajOptProblemTpl<double>> problem_;
};

/// What the chasing stub's cost and context check do (shared by every copy
/// the problem makes of the cost).
struct ChaseState {
  bool moving{true};           ///< the target moves at every evaluation
  double target{0.0};          ///< every control entry's target
  bool refuse_context{false};  ///< UpdateReferences reports a topology change
};

/// `½·w·‖u − r‖²` with `r` the shared target, moved by 1 at every evaluation
/// while `moving`: the dual residual never meets a tolerance. Controls do not
/// reach the dynamics, so the iterates stay finite.
struct ChaseCost : aligator::CostAbstractTpl<double> {
  ChaseCost(const aligator::VectorSpaceTpl<double>& space, int nu, std::shared_ptr<ChaseState> st)
      : aligator::CostAbstractTpl<double>(space, nu), st_(std::move(st)) {}

  void evaluate(const ConstVectorRef& /*x*/, const ConstVectorRef& u,
                CostData& data) const override {
    if (st_->moving) {
      st_->target += 1.0;
    }
    data.value_ = 0.5 * (u.array() - st_->target).square().sum();
  }

  void computeGradients(const ConstVectorRef& /*x*/, const ConstVectorRef& u,
                        CostData& data) const override {
    data.Lx_.setZero();
    data.Lu_ = (u.array() - st_->target).matrix();
  }

  void computeHessians(const ConstVectorRef& /*x*/, const ConstVectorRef& /*u*/,
                       CostData& data) const override {
    data.hess_.setZero();
    data.Luu_.setIdentity();
  }

  std::shared_ptr<ChaseState> st_;
};

/// OCP with the chasing cost on the hold-state dynamics.
class ChaseOCP final : public rtc::mpc::OCPHandlerBase {
 public:
  explicit ChaseOCP(bool rebuildable) : rebuildable_(rebuildable) {}

  ChaseState& state() noexcept { return *st_; }

  [[nodiscard]] int builds() const noexcept { return builds_; }

  rtc::mpc::OCPBuildError Build(const rtc::mpc::PhaseContext& /*ctx*/,
                                const rtc::mpc::RobotModelHandler& model,
                                const rtc::mpc::OCPLimits& /*limits*/) noexcept override {
    ++builds_;
    try {
      const int nx = model.nq() + model.nv();
      const int nu = model.nu();
      const aligator::VectorSpaceTpl<double> space(nx);
      const ChaseCost cost(space, nu, st_);
      std::vector<xyz::polymorphic<aligator::StageModelTpl<double>>> stages;
      for (int k = 0; k < kHorizon; ++k) {
        stages.emplace_back(aligator::StageModelTpl<double>(cost, HoldStateDynamics(nx, nu)));
      }
      problem_.emplace(Eigen::VectorXd::Zero(nx), stages,
                       aligator::ConstantCostTpl<double>(space, nu, 0.0));
    } catch (...) {
      return rtc::mpc::OCPBuildError::kAligatorInstantiationFailure;
    }
    st_->refuse_context = false;  // the new topology is the context's
    return rtc::mpc::OCPBuildError::kNoError;
  }

  rtc::mpc::OCPBuildError UpdateReferences(
      const rtc::mpc::PhaseContext& /*ctx*/) noexcept override {
    return st_->refuse_context ? rtc::mpc::OCPBuildError::kInvalidPhaseContext
                               : rtc::mpc::OCPBuildError::kNoError;
  }

  bool Built() const noexcept override { return problem_.has_value(); }

  aligator::TrajOptProblemTpl<double>& problem() override { return *problem_; }

  int horizon_length() const noexcept override { return kHorizon; }

  std::string_view ocp_type() const noexcept override { return kStubType; }

  bool RebuildsOnContextChange() const noexcept override { return rebuildable_; }

 private:
  bool rebuildable_;
  int builds_{0};
  std::shared_ptr<ChaseState> st_ = std::make_shared<ChaseState>();
  std::optional<aligator::TrajOptProblemTpl<double>> problem_;
};

/// The shipped q / v / u trajectories only (not the gains).
[[nodiscard]] bool ShippedTrajectoryFinite(const MPCSolution& s) {
  for (int k = 0; k <= s.horizon_length; ++k) {
    for (int i = 0; i < s.nq; ++i) {
      if (!std::isfinite(s.q_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)])) {
        return false;
      }
    }
    for (int i = 0; i < s.nv; ++i) {
      if (!std::isfinite(s.v_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)])) {
        return false;
      }
    }
  }
  for (int k = 0; k < s.horizon_length; ++k) {
    for (int i = 0; i < s.nu; ++i) {
      if (!std::isfinite(s.u_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)])) {
        return false;
      }
    }
  }
  return true;
}

class MPCHandlerCoreGuardTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
    }
    pinocchio::urdf::buildModel(kPandaUrdf, model_);
    ASSERT_EQ(handler_.Init(model_, YAML::Load(kPandaTwoFingerYaml)),
              rtc::mpc::RobotModelInitError::kNoError);

    ctx_.ocp_type = std::string{kStubType};
    ctx_.cost_config.dt = kNodeDt;

    solver_cfg_.prim_tol = 1e-6;
    solver_cfg_.dual_tol = 1e-6;
    solver_cfg_.mu_init = 1e-2;
    solver_cfg_.max_iters = 5;
    solver_cfg_.max_al_iters = 2;

    ASSERT_EQ(core_.Init(solver_cfg_, handler_, limits_, ctx_, ocp_),
              rtc::mpc::MPCInitError::kNoError);
    state_ = NeutralStateSnapshot(model_, handler_);
  }

  /// One clean tick: the stub converges on iteration 0 from the cold seed.
  void SolveClean(MPCSolution& out) {
    ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kNoError);
    ASSERT_TRUE(out.IsValid());
    ASSERT_TRUE(ShippedTrajectoryFinite(out));
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  rtc::mpc::PhaseContext ctx_;
  rtc::mpc::OCPLimits limits_;
  rtc::mpc::MPCSolverConfig solver_cfg_;
  InertOCP ocp_;
  rtc::mpc::internal::MPCHandlerCore core_;
  MPCStateSnapshot state_{};
};

TEST_F(MPCHandlerCoreGuardTest, SetupAppliesTheSolverSettings) {
  const aligator::SolverProxDDPTpl<double>* solver = core_.solver();
  ASSERT_NE(solver, nullptr);
  EXPECT_EQ(solver->rollout_type_, aligator::RolloutType::LINEAR);
  EXPECT_EQ(solver->linear_solver_choice, aligator::LQSolverChoice::SERIAL);
  EXPECT_TRUE(solver->force_initial_condition_);
  EXPECT_EQ(solver->getNumThreads(), 1U);

  rtc::mpc::MPCSolverConfig nonlinear = solver_cfg_;
  nonlinear.rollout_type = rtc::mpc::MPCRolloutType::kNonlinear;
  InertOCP ocp;
  rtc::mpc::internal::MPCHandlerCore core;
  ASSERT_EQ(core.Init(nonlinear, handler_, limits_, ctx_, ocp), rtc::mpc::MPCInitError::kNoError);
  EXPECT_EQ(core.solver()->rollout_type_, aligator::RolloutType::NONLINEAR);
  MPCSolution out{};
  EXPECT_EQ(core.Solve(ctx_, state_, ocp, out), MPCSolveError::kNoError);
}

TEST_F(MPCHandlerCoreGuardTest, FirstSolveStartsFromTheColdSeed) {
  // Away from the problem's placeholder initial state, so a guess that is
  // not the cold seed cannot pass for it.
  for (int i = 0; i < state_.nq; ++i) {
    state_.q[static_cast<std::size_t>(i)] = 0.1 * (i + 1);
  }
  MPCSolution out{};
  SolveClean(out);
  // The stub is solved by its guess, so the result is the seed itself: the
  // measured state at every node and zero controls (a non-gravity OCP type).
  EXPECT_EQ(out.iterations, 0);
  for (int k = 0; k <= out.horizon_length; ++k) {
    for (int i = 0; i < out.nq; ++i) {
      EXPECT_EQ(out.q_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)],
                state_.q[static_cast<std::size_t>(i)])
          << "node " << k << " q[" << i << "]";
    }
  }
  for (int k = 0; k < out.horizon_length; ++k) {
    for (int i = 0; i < out.nu; ++i) {
      EXPECT_EQ(out.u_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)], 0.0);
    }
  }
}

TEST_F(MPCHandlerCoreGuardTest, CleanWarmStartStaysValid) {
  MPCSolution first{};
  SolveClean(first);
  core_.SeedWarmStart(first, ocp_);
  MPCSolution second{};
  SolveClean(second);
}

TEST_F(MPCHandlerCoreGuardTest, NonFiniteControlInWarmStartIsDiverged) {
  for (const double poison :
       {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    MPCSolution first{};
    SolveClean(first);
    MPCSolution poisoned = first;
    // A late node: the shift moves it one node earlier, still past node 0.
    poisoned.u_traj[5][2] = poison;
    core_.SeedWarmStart(poisoned, ocp_);

    MPCSolution out{};
    EXPECT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kSolverDiverged)
        << "poison " << poison;
    EXPECT_EQ(out.status, SolverStatus::kDiverged);
    EXPECT_FALSE(out.IsValid());
  }
}

TEST_F(MPCHandlerCoreGuardTest, NonFiniteStateInWarmStartIsDiverged) {
  for (const double poison :
       {std::numeric_limits<double>::quiet_NaN(), -std::numeric_limits<double>::infinity()}) {
    MPCSolution first{};
    SolveClean(first);
    MPCSolution poisoned = first;
    poisoned.q_traj[6][1] = poison;
    poisoned.v_traj[kHorizon][0] = poison;  // the terminal node too
    core_.SeedWarmStart(poisoned, ocp_);

    MPCSolution out{};
    EXPECT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kSolverDiverged)
        << "poison " << poison;
    EXPECT_EQ(out.status, SolverStatus::kDiverged);
    EXPECT_FALSE(out.IsValid());
  }
}

TEST_F(MPCHandlerCoreGuardTest, DivergedTickDoesNotPoisonTheNext) {
  MPCSolution first{};
  SolveClean(first);
  MPCSolution poisoned = first;
  poisoned.q_traj[4][0] = std::numeric_limits<double>::quiet_NaN();
  poisoned.u_traj[3][1] = std::numeric_limits<double>::infinity();
  core_.SeedWarmStart(poisoned, ocp_);
  MPCSolution out{};
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kSolverDiverged);

  // Without a reseed the next tick would shift the rejected trajectory in as
  // its warm start and diverge again.
  for (int tick = 0; tick < 3; ++tick) {
    MPCSolution next{};
    SolveClean(next);
  }
}

TEST_F(MPCHandlerCoreGuardTest, PartialSeedAfterDivergenceStaysFinite) {
  MPCSolution first{};
  SolveClean(first);
  MPCSolution poisoned = first;
  poisoned.q_traj[kHorizon][2] = std::numeric_limits<double>::quiet_NaN();
  poisoned.u_traj[kHorizon - 1][0] = std::numeric_limits<double>::infinity();
  core_.SeedWarmStart(poisoned, ocp_);
  MPCSolution out{};
  ASSERT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kSolverDiverged);

  // A seed shorter than the horizon overwrites only its own nodes; the rest
  // must not still hold the rejected iterate.
  MPCSolution short_seed = first;
  short_seed.horizon_length = 3;
  core_.SeedWarmStart(short_seed, ocp_);
  MPCSolution next{};
  SolveClean(next);
}

TEST_F(MPCHandlerCoreGuardTest, NonFiniteResidualIsDiverged) {
  MPCSolution first{};
  SolveClean(first);
  ocp_.SetPoison(Poison::kValue);  // trajectories stay finite, the cost does not
  MPCSolution out{};
  EXPECT_EQ(core_.Solve(ctx_, state_, ocp_, out), MPCSolveError::kSolverDiverged);
  EXPECT_FALSE(out.IsValid());
  ocp_.SetPoison(Poison::kNone);
  MPCSolution next{};
  SolveClean(next);
}

TEST_F(MPCHandlerCoreGuardTest, NonFiniteOrNonPositiveNodeDtIsRefused) {
  for (const double dt : {std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity(), 0.0, -kNodeDt}) {
    rtc::mpc::PhaseContext bad = ctx_;
    bad.cost_config.dt = dt;
    MPCSolution out{};
    EXPECT_EQ(core_.Solve(bad, state_, ocp_, out), MPCSolveError::kOCPRebuildFailed) << "dt " << dt;
    EXPECT_FALSE(out.IsValid());

    InertOCP fresh_ocp;
    rtc::mpc::internal::MPCHandlerCore fresh;
    EXPECT_EQ(fresh.Init(solver_cfg_, handler_, limits_, bad, fresh_ocp),
              rtc::mpc::MPCInitError::kOCPBuildFailed)
        << "dt " << dt;
  }
  MPCSolution out{};
  SolveClean(out);
}

TEST_F(MPCHandlerCoreGuardTest, NonFiniteEntryStateIsRejectedBeforeTheSolver) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  // A distinctive warm start: the inert controls come back as seeded.
  MPCSolution first{};
  SolveClean(first);
  MPCSolution seed = first;
  for (int k = 0; k < kHorizon; ++k) {
    for (int i = 0; i < seed.nu; ++i) {
      seed.u_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] = 0.5;
    }
  }
  core_.SeedWarmStart(seed, ocp_);

  struct Case {
    bool in_q;
    int index;
    double value;
  };

  for (const Case c : {Case{true, 0, nan}, Case{true, handler_.nq() - 1, inf}, Case{false, 0, -inf},
                       Case{false, handler_.nv() - 1, nan}}) {
    MPCStateSnapshot bad = state_;
    double* const slot = c.in_q ? bad.q.data() : bad.v.data();
    slot[c.index] = c.value;
    MPCSolution out{};
    EXPECT_EQ(core_.Solve(ctx_, bad, ocp_, out), MPCSolveError::kStateDimMismatch)
        << (c.in_q ? "q[" : "v[") << c.index << "] = " << c.value;
    EXPECT_EQ(out.status, SolverStatus::kNotSolved);
  }
  // The rejected ticks touched nothing: the next tick still warm-starts from
  // the seed (a cold seed would have zero controls).
  MPCSolution out{};
  SolveClean(out);
  for (int k = 0; k < kHorizon; ++k) {
    for (int i = 0; i < out.nu; ++i) {
      EXPECT_EQ(out.u_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)], 0.5)
          << "node " << k << " u[" << i << "]";
    }
  }
}

// ── Budgets and the rebuild gate ────────────────────────────────────────────

class MPCHandlerCoreBudgetTest : public MPCHandlerCoreGuardTest {
 protected:
  void SetUp() override {
    MPCHandlerCoreGuardTest::SetUp();
    if (IsSkipped()) {
      return;
    }
    budget_.prim_tol = 1e-8;
    budget_.dual_tol = 1e-8;
    budget_.mu_init = 1e-2;
    budget_.max_iters = 2;
    budget_.max_al_iters = 3;
    budget_.max_initial_iters = 6;
    budget_.max_online_al_iters = 1;
  }

  /// The ceilings the solver ran the last solve with.
  void ExpectCeilings(const rtc::mpc::internal::MPCHandlerCore& core, std::size_t iters,
                      std::size_t al_iters, const char* what) const {
    ASSERT_NE(core.solver(), nullptr);
    EXPECT_EQ(core.solver()->max_iters, iters) << what;
    EXPECT_EQ(core.solver()->max_al_iters, al_iters) << what;
  }

  rtc::mpc::MPCSolverConfig budget_;
};

TEST_F(MPCHandlerCoreBudgetTest, TheStandingBudgetHoldsUntilASolveConverges) {
  ChaseOCP ocp(true);
  rtc::mpc::internal::MPCHandlerCore core;
  ASSERT_EQ(core.Init(budget_, handler_, limits_, ctx_, ocp), rtc::mpc::MPCInitError::kNoError);
  EXPECT_TRUE(core.InitialSolvePending());

  // The moving target: no solve converges, each one uses the whole standing
  // budget, and the next one gets it again.
  for (int tick = 0; tick < 2; ++tick) {
    MPCSolution out{};
    ASSERT_EQ(core.Solve(ctx_, state_, ocp, out), MPCSolveError::kNoError) << "tick " << tick;
    EXPECT_FALSE(out.converged) << "tick " << tick;
    EXPECT_TRUE(out.initial_solve) << "tick " << tick;
    EXPECT_EQ(out.iterations, 6) << "tick " << tick;
    ExpectCeilings(core, 6, 3, "unconverged standing solve");
    EXPECT_TRUE(core.InitialSolvePending()) << "tick " << tick;
  }

  // The target stops: this solve converges, still on the standing budget.
  ocp.state().moving = false;
  MPCSolution out{};
  ASSERT_EQ(core.Solve(ctx_, state_, ocp, out), MPCSolveError::kNoError);
  EXPECT_TRUE(out.converged);
  EXPECT_TRUE(out.initial_solve);
  ExpectCeilings(core, 6, 3, "converging standing solve");
  EXPECT_FALSE(core.InitialSolvePending());

  // From now on the online budget, converged or not.
  ASSERT_EQ(core.Solve(ctx_, state_, ocp, out), MPCSolveError::kNoError);
  EXPECT_FALSE(out.initial_solve);
  ExpectCeilings(core, 2, 1, "online solve");
  ocp.state().moving = true;
  ASSERT_EQ(core.Solve(ctx_, state_, ocp, out), MPCSolveError::kNoError);
  EXPECT_FALSE(out.converged);
  EXPECT_FALSE(out.initial_solve);
  EXPECT_LE(out.iterations, 2);
  ExpectCeilings(core, 2, 1, "unconverged online solve");
}

TEST_F(MPCHandlerCoreBudgetTest, ZeroCeilingsMeanTheOtherBudgetsValues) {
  budget_.max_initial_iters = 0;
  budget_.max_online_al_iters = 0;
  ChaseOCP ocp(true);
  rtc::mpc::internal::MPCHandlerCore core;
  ASSERT_EQ(core.Init(budget_, handler_, limits_, ctx_, ocp), rtc::mpc::MPCInitError::kNoError);
  ocp.state().moving = false;
  MPCSolution out{};
  ASSERT_EQ(core.Solve(ctx_, state_, ocp, out), MPCSolveError::kNoError);
  ASSERT_TRUE(out.converged);
  ExpectCeilings(core, 2, 3, "standing solve");
  ASSERT_EQ(core.Solve(ctx_, state_, ocp, out), MPCSolveError::kNoError);
  ExpectCeilings(core, 2, 3, "online solve");
}

TEST_F(MPCHandlerCoreBudgetTest, ARejectedSolveOrARebuildReturnsToTheStandingBudget) {
  ChaseOCP ocp(true);
  rtc::mpc::internal::MPCHandlerCore core;
  ASSERT_EQ(core.Init(budget_, handler_, limits_, ctx_, ocp), rtc::mpc::MPCInitError::kNoError);
  ocp.state().moving = false;
  MPCSolution first{};
  ASSERT_EQ(core.Solve(ctx_, state_, ocp, first), MPCSolveError::kNoError);
  ASSERT_FALSE(core.InitialSolvePending());

  // A rejected solve (a non-finite control in the warm start) cold-seeds the
  // next tick, and that tick is a standing solve again.
  MPCSolution poisoned = first;
  poisoned.u_traj[3][1] = std::numeric_limits<double>::quiet_NaN();
  core.SeedWarmStart(poisoned, ocp);
  MPCSolution out{};
  const MPCSolveError e = core.Solve(ctx_, state_, ocp, out);
  EXPECT_TRUE(e == MPCSolveError::kSolverDiverged || e == MPCSolveError::kSolverException)
      << static_cast<int>(e);
  EXPECT_TRUE(core.InitialSolvePending());
  ASSERT_EQ(core.Solve(ctx_, state_, ocp, out), MPCSolveError::kNoError);
  EXPECT_TRUE(out.initial_solve);
  ExpectCeilings(core, 6, 3, "after the rejection");
  ASSERT_TRUE(out.converged);
  ASSERT_FALSE(core.InitialSolvePending());

  // A rebuild: the tick that rebuilds solves on the standing budget.
  ocp.state().refuse_context = true;
  ASSERT_EQ(core.Solve(ctx_, state_, ocp, out), MPCSolveError::kNoError);
  EXPECT_EQ(ocp.builds(), 2);
  EXPECT_TRUE(out.initial_solve);
  ExpectCeilings(core, 6, 3, "after the rebuild");
}

TEST_F(MPCHandlerCoreBudgetTest, AnOcpThatCannotRebuildIsRefusedWithoutBuildOrAllocation) {
  ChaseOCP fixed(false);
  rtc::mpc::internal::MPCHandlerCore core;
  ASSERT_EQ(core.Init(budget_, handler_, limits_, ctx_, fixed), rtc::mpc::MPCInitError::kNoError);
  fixed.state().moving = false;
  MPCSolution out{};
  ASSERT_EQ(core.Solve(ctx_, state_, fixed, out), MPCSolveError::kNoError);
  ASSERT_TRUE(out.converged);
  const aligator::TrajOptProblemTpl<double>* problem = &fixed.problem();

  fixed.state().refuse_context = true;
  MPCSolution refused{};
  rtc::mpc::test_utils::AllocCounter::Arm();
  const MPCSolveError e = core.Solve(ctx_, state_, fixed, refused);
  rtc::mpc::test_utils::AllocCounter::Disarm();
  EXPECT_EQ(e, MPCSolveError::kOCPRebuildFailed);
  EXPECT_EQ(rtc::mpc::test_utils::AllocCounter::AllocCount(), 0)
      << "no Build, no solver setup: nothing allocates";
  EXPECT_EQ(fixed.builds(), 1) << "only Init built";
  EXPECT_EQ(&fixed.problem(), problem);
  EXPECT_FALSE(refused.IsValid());
  // The refusal touched no iterate: the next tick is online and warm.
  EXPECT_FALSE(core.InitialSolvePending());
  fixed.state().refuse_context = false;
  ASSERT_EQ(core.Solve(ctx_, state_, fixed, out), MPCSolveError::kNoError);
  EXPECT_FALSE(out.initial_solve);

  // A rebuildable OCP still rebuilds on the same refusal.
  ChaseOCP rebuildable(true);
  rtc::mpc::internal::MPCHandlerCore core2;
  ASSERT_EQ(core2.Init(budget_, handler_, limits_, ctx_, rebuildable),
            rtc::mpc::MPCInitError::kNoError);
  rebuildable.state().moving = false;
  rebuildable.state().refuse_context = true;
  ASSERT_EQ(core2.Solve(ctx_, state_, rebuildable, out), MPCSolveError::kNoError);
  EXPECT_EQ(rebuildable.builds(), 2);
}

}  // namespace
