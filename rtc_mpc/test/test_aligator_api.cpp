/// @file test_aligator_api.cpp
/// @brief Pins the Aligator API that the floating-base (humanoid) MPC design
///        relies on, against the deps version this package builds with
///        (`deps.repos`). Nothing in `rtc_mpc/src` uses these entry points
///        yet, so a deps bump that renames or drops one fails here first.
///
/// Covered, on the generic Panda fixture lifted onto a free-flyer root:
/// - `MultibodyPhaseSpace` + `KinodynamicsFwdDynamicsTpl` dimensions, and the
///   centroidal law at rest (no contact force, no joint acceleration → the
///   base falls with gravity).
/// - A stage's ODE is its own copy, and its contact mask switches in place.
/// - `StageModel::addConstraint`.
/// - `SolverProxDDP` settings (tol · mu_init · rollout · linear solver ·
///   threads) round-trip, `setup` + `run` for each valid rollout × linear
///   solver pair, and the pair v0.19.0 rejects (nonlinear + parallel).
/// - The receding-horizon tick: `TrajOptProblem::replaceStageCircular`,
///   `setInitState`, `SolverProxDDP::cycleProblem` (the solution shifts one
///   knot), terminal data, then a warm-started `run` with
///   `force_initial_condition_` — and the v0.19.0 behaviour that fixes that
///   order (the first knot is never moved off the guess).
///
/// Convergence is not asserted — the OCP is a stand-in, not a controller.

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/constraints/box-constraint.hpp>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/dynamics/integrator-semi-euler.hpp>
#include <aligator/modelling/dynamics/kinodynamics-fwd.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#include <aligator/modelling/state-error.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>
#include <aligator/threads.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "test_utils/panda_fixture.hpp"

#include <cstddef>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Space = aligator::MultibodyPhaseSpace<double>;
using Kinodynamics = aligator::dynamics::KinodynamicsFwdDynamicsTpl<double>;
using Integrator = aligator::dynamics::IntegratorSemiImplEulerTpl<double>;
using StageModel = aligator::StageModelTpl<double>;
using Problem = aligator::TrajOptProblemTpl<double>;
using Solver = aligator::SolverProxDDPTpl<double>;
using CostStack = aligator::CostStackTpl<double>;
using StateCost = aligator::QuadraticStateCostTpl<double>;
using ControlCost = aligator::QuadraticControlCostTpl<double>;
using ControlRes = aligator::ControlErrorResidualTpl<double>;
using Box = aligator::BoxConstraintTpl<double>;
using PolyStage = xyz::polymorphic<StageModel>;

using rtc::mpc::test_utils::kPandaUrdf;

constexpr int kForceSize = 3;  // point contacts
constexpr int kHorizon = 6;
constexpr double kDt = 0.02;
constexpr double kGravityZ = -9.81;

class AligatorApiTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
    }
    pinocchio::urdf::buildModel(kPandaUrdf, pinocchio::JointModelFreeFlyer(), model_);
    contact_ids_ = {model_.getFrameId("panda_leftfinger"), model_.getFrameId("panda_rightfinger")};
    ASSERT_LT(contact_ids_[0], model_.frames.size());
    ASSERT_LT(contact_ids_[1], model_.frames.size());
    space_ = std::make_unique<Space>(model_);
    x0_ = space_->neutral();
#ifdef ALIGATOR_MULTITHREADING
    omp_threads_ = omp_get_max_threads();
    omp_dynamic_ = omp_get_dynamic();
#endif
  }

  // SolverProxDDP::setNumThreads sets the process-wide OpenMP defaults; put
  // them back so test order does not matter.
  void TearDown() override {
#ifdef ALIGATOR_MULTITHREADING
    omp_set_num_threads(omp_threads_);
    omp_set_dynamic(omp_dynamic_);
#endif
  }

  Kinodynamics MakeOde(const std::vector<bool>& contact_states) const {
    return Kinodynamics(*space_, model_, Eigen::Vector3d(0.0, 0.0, kGravityZ), contact_states,
                        contact_ids_, kForceSize);
  }

  int Nu() const { return kForceSize * static_cast<int>(contact_ids_.size()) + model_.nv - 6; }

  // One stage: state + control regularisation, optionally a control box, and
  // the kinodynamics ODE discretised with semi-implicit Euler.
  PolyStage MakeStage(const std::vector<bool>& contact_states, bool with_box = true) const {
    const int nu = Nu();
    const int ndx = space_->ndx();
    CostStack cost(*space_, nu);
    cost.addCost("x_reg", StateCost(*space_, nu, x0_, Eigen::MatrixXd::Identity(ndx, ndx)));
    cost.addCost("u_reg", ControlCost(*space_, nu, 1e-3 * Eigen::MatrixXd::Identity(nu, nu)));
    StageModel stage(cost, Integrator(MakeOde(contact_states), kDt));
    if (with_box) {
      stage.addConstraint(ControlRes(ndx, nu),
                          Box(-1e3 * Eigen::VectorXd::Ones(nu), 1e3 * Eigen::VectorXd::Ones(nu)));
    }
    return PolyStage(stage);
  }

  Problem MakeProblem() const {
    std::vector<PolyStage> stages;
    stages.reserve(static_cast<std::size_t>(kHorizon));
    for (int k = 0; k < kHorizon; ++k) {
      stages.push_back(MakeStage({true, true}));
    }
    return Problem(x0_, stages, CostStack(*space_, Nu()));
  }

  pinocchio::Model model_;
  std::vector<pinocchio::FrameIndex> contact_ids_;
  std::unique_ptr<Space> space_;
  Eigen::VectorXd x0_;
  int omp_threads_{1};
  int omp_dynamic_{0};
};

TEST_F(AligatorApiTest, KinodynamicsDimensionsOnFloatingBase) {
  ASSERT_EQ(model_.nq, model_.nv + 1);  // free-flyer quaternion
  EXPECT_EQ(space_->nx(), model_.nq + model_.nv);
  EXPECT_EQ(space_->ndx(), 2 * model_.nv);

  const Kinodynamics ode = MakeOde({true, true});
  EXPECT_EQ(ode.nu(), Nu());
  EXPECT_EQ(ode.force_size_, kForceSize);
  EXPECT_EQ(ode.contact_ids_, contact_ids_);
}

// v = 0, contact forces 0, joint accelerations 0: the centroidal momentum rate
// is m·g, so the whole body translates with gravity — the base acceleration
// (local frame, identity orientation at neutral) is (0, 0, g, 0, 0, 0).
TEST_F(AligatorApiTest, KinodynamicsBaseFallsWithGravityAtRest) {
  const Kinodynamics ode = MakeOde({true, true});
  auto data = ode.createData();
  const Eigen::VectorXd u = Eigen::VectorXd::Zero(Nu());
  ode.forward(x0_, u, *data);

  const Eigen::VectorXd xdot = data->xdot_;
  ASSERT_EQ(xdot.size(), 2 * model_.nv);
  ASSERT_TRUE(xdot.allFinite());
  EXPECT_TRUE(xdot.head(model_.nv).isZero(1e-12));  // q̇ = v = 0
  Eigen::Matrix<double, 6, 1> expected_base;
  expected_base << 0.0, 0.0, kGravityZ, 0.0, 0.0, 0.0;
  EXPECT_TRUE(xdot.segment(model_.nv, 6).isApprox(expected_base, 1e-9))
      << xdot.segment(model_.nv, 6).transpose();
  EXPECT_TRUE(xdot.tail(model_.nv - 6).isZero(1e-12));  // commanded a_j = 0
}

// A stage owns its own copy of the ODE (integrator and stage hold it by
// value). Writing that copy's contact flags switches the mode in place — no
// rebuild — and the stage's integrator follows; the ODE the stage was built
// from is a separate object.
TEST_F(AligatorApiTest, StageOdeContactMaskSwitchesInPlace) {
  Kinodynamics source = MakeOde({true, true});
  StageModel stage(CostStack(*space_, Nu()), Integrator(source, kDt));
  auto* integrator = stage.getDynamics<Integrator>();
  ASSERT_NE(integrator, nullptr);
  auto* in_stage = integrator->getDynamics<Kinodynamics>();
  ASSERT_NE(in_stage, nullptr);
  ASSERT_NE(in_stage, &source);

  Eigen::VectorXd u = Eigen::VectorXd::Zero(Nu());
  u.segment<3>(0) << 1.0, -2.0, 30.0;  // contact 0 only
  auto step = [&](const Integrator& integ) {
    auto d = integ.createData();
    integ.forward(x0_, u, *d);
    return Eigen::VectorXd(d->xnext_);
  };
  const Eigen::VectorXd both = step(*integrator);
  in_stage->contact_states_[0] = false;
  const Eigen::VectorXd second_only = step(*integrator);
  EXPECT_FALSE(second_only.isApprox(both));
  // Same as an integrator built for that mode from the start.
  EXPECT_EQ(second_only, step(Integrator(MakeOde({false, true}), kDt)));
  // The source ODE was not touched, nor does writing it reach the stage.
  EXPECT_TRUE(source.contact_states_[0]);
  source.contact_states_[1] = false;
  EXPECT_TRUE(in_stage->contact_states_[1]);
}

TEST_F(AligatorApiTest, StageAddConstraint) {
  const PolyStage stage = MakeStage({true, true});
  EXPECT_EQ(stage->numConstraints(), 1U);
  EXPECT_EQ(stage->nc(), Nu());
  EXPECT_EQ(stage->nu(), Nu());
}

// v0.19.0 refuses this pair at setup (solver-proxddp.hxx) — a design that wants
// a nonlinear rollout has to take the serial Riccati solver. If a deps bump
// lifts the restriction this fails, and the design note should be updated.
TEST_F(AligatorApiTest, ProxDdpParallelSolverRejectsNonlinearRollout) {
  const Problem problem = MakeProblem();
  Solver solver(1e-4, 1e-2, 5);
  solver.rollout_type_ = aligator::RolloutType::NONLINEAR;
  solver.linear_solver_choice = aligator::LQSolverChoice::PARALLEL;
  solver.setNumThreads(2);
  try {
    solver.setup(problem);
    FAIL() << "setup accepted a nonlinear rollout with the parallel solver";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("Nonlinear rollouts not supported"), std::string::npos)
        << e.what();
  }
}

struct SolverCombo {
  aligator::RolloutType rollout;
  aligator::LQSolverChoice linear_solver;
  std::size_t num_threads;
};

class AligatorApiSolveTest : public AligatorApiTest,
                             public ::testing::WithParamInterface<SolverCombo> {};

// The receding-horizon tick with a warm start. Aligator's own MPC test
// (tests/mpc-cycle.cpp) calls setInitState after cycleProblem and then run()
// with no guess, which re-initialises the trajectory from x0 and drops the
// shift. To keep the shift, setInitState goes first so cycleProblem writes the
// measurement into the first knot, and the shifted xs/us are passed to run() —
// force_initial_condition_ never moves the first knot off the guess (see
// ForcedInitialConditionKeepsTheGuessFirstKnot).
TEST_P(AligatorApiSolveTest, SettingsSetupRunAndWarmStartedRecedingHorizonCycle) {
  const SolverCombo combo = GetParam();
  Problem problem = MakeProblem();
  ASSERT_EQ(problem.numSteps(), static_cast<std::size_t>(kHorizon));

  constexpr double kTol = 1e-4;
  constexpr double kMuInit = 1e-2;
  constexpr std::size_t kMaxIters = 5;
  Solver solver(kTol, kMuInit, kMaxIters);
  solver.rollout_type_ = combo.rollout;
  solver.linear_solver_choice = combo.linear_solver;
  solver.setNumThreads(combo.num_threads);
  solver.force_initial_condition_ = true;
  EXPECT_DOUBLE_EQ(solver.target_tol_, kTol);
  EXPECT_DOUBLE_EQ(solver.mu_init_, kMuInit);
  EXPECT_EQ(solver.max_iters, kMaxIters);
  EXPECT_EQ(solver.getNumThreads(), combo.num_threads);

  solver.setup(problem);
  solver.run(problem);
  ASSERT_EQ(solver.results_.xs.size(), static_cast<std::size_t>(kHorizon + 1));
  ASSERT_EQ(solver.results_.us.size(), static_cast<std::size_t>(kHorizon));
  for (const auto& u : solver.results_.us) {
    ASSERT_TRUE(u.allFinite());
  }
  const std::vector<Eigen::VectorXd> xs_prev = solver.results_.xs;
  const std::vector<Eigen::VectorXd> us_prev = solver.results_.us;
  const Eigen::VectorXd x_measured = xs_prev[1];  // stands in for the next measurement
  ASSERT_FALSE(x_measured.isApprox(x0_));         // the first knot has to move

  // The old first stage comes back, everything moves up one slot, and the new
  // stage (one contact lifted, no box) is last.
  const StageModel* old_first = &*problem.stages_[0];
  const StageModel* old_second = &*problem.stages_[1];
  const PolyStage popped = problem.replaceStageCircular(MakeStage({true, false}, false));
  EXPECT_EQ(&*popped, old_first);
  EXPECT_EQ(&*problem.stages_[0], old_second);
  EXPECT_EQ(problem.numSteps(), static_cast<std::size_t>(kHorizon));
  EXPECT_EQ(problem.stages_.back()->numConstraints(), 0U);

  // cycleProblem shifts the previous solution one knot and puts the init state
  // in the first one.
  problem.setInitState(x_measured);
  solver.cycleProblem(problem, problem.stages_.back()->createData());
  EXPECT_TRUE(solver.results_.xs[0].isApprox(x_measured));
  for (int k = 0; k + 1 < kHorizon; ++k) {
    const auto i = static_cast<std::size_t>(k);
    EXPECT_TRUE(solver.results_.us[i].isApprox(us_prev[i + 1])) << "u knot " << k;
    if (k > 0) {
      EXPECT_TRUE(solver.results_.xs[i].isApprox(xs_prev[i + 1])) << "x knot " << k;
    }
  }

  problem.term_cost_ = CostStack(*space_, Nu());
  solver.workspace_.problem_data.term_cost_data = problem.term_cost_->createData();

  const std::vector<Eigen::VectorXd> xs_guess = solver.results_.xs;
  const std::vector<Eigen::VectorXd> us_guess = solver.results_.us;
  solver.run(problem, xs_guess, us_guess);
  ASSERT_EQ(solver.results_.xs.size(), static_cast<std::size_t>(kHorizon + 1));
  for (const auto& x : solver.results_.xs) {
    EXPECT_TRUE(x.allFinite());
  }
  EXPECT_TRUE(solver.results_.xs[0].isApprox(x_measured));
}

// v0.19.0: with force_initial_condition_ the first-knot step is zeroed, so a
// guess whose first knot is stale stays stale — the init state is not pulled
// in. If a deps bump changes this, the warm-start note above should be updated.
TEST_F(AligatorApiTest, ForcedInitialConditionKeepsTheGuessFirstKnot) {
  Problem problem = MakeProblem();
  Solver solver(1e-4, 1e-2, 5);
  solver.force_initial_condition_ = true;
  solver.setup(problem);
  solver.run(problem);
  const std::vector<Eigen::VectorXd> xs_guess = solver.results_.xs;
  const std::vector<Eigen::VectorXd> us_guess = solver.results_.us;
  const Eigen::VectorXd x_new = xs_guess[1];
  ASSERT_FALSE(x_new.isApprox(xs_guess[0]));

  problem.setInitState(x_new);
  solver.run(problem, xs_guess, us_guess);
  EXPECT_TRUE(solver.results_.xs[0].isApprox(xs_guess[0]));
  EXPECT_FALSE(solver.results_.xs[0].isApprox(x_new));
}

INSTANTIATE_TEST_SUITE_P(ValidCombos, AligatorApiSolveTest,
                         ::testing::Values(SolverCombo{aligator::RolloutType::NONLINEAR,
                                                       aligator::LQSolverChoice::SERIAL, 1},
                                           SolverCombo{aligator::RolloutType::LINEAR,
                                                       aligator::LQSolverChoice::PARALLEL, 2}),
                         [](const ::testing::TestParamInfo<SolverCombo>& info) {
                           return info.param.linear_solver == aligator::LQSolverChoice::SERIAL
                                      ? std::string("NonlinearSerial")
                                      : std::string("LinearParallel");
                         });

}  // namespace
