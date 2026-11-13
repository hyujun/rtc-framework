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

#include "handler/internal/mpc_handler_core.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/ocp_handler_base.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"
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
#include <aligator/core/explicit-dynamics.hpp>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/core/vector-space.hpp>
#include <aligator/modelling/costs/constant-cost.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

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

}  // namespace
