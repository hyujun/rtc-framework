/// @file test_kinodynamic_ocp_alloc_gate.cpp
/// @brief Heap allocations of the kinodynamic OCP's per-tick path on the
///        public G1 (32 nodes, both soles): `UpdateReferences` over a whole
///        horizon makes no C++ and no C allocation, through mode changes in
///        both directions and a refused horizon; nor does a tick whose
///        horizon has another shape, through the handler core. The reference
///        control and the constraint residuals of a trajectory allocate
///        nothing beyond the ODE evaluations they make, nor does a running
///        stage (evaluate, first-order derivatives) beyond its dynamics step.
///        The quadratic state, input and terminal costs of the default build
///        (structured derivatives) allocate nothing through evaluate,
///        gradients and Hessians.
///
/// Both counters are armed: the operator-new counter (C++ allocations) and
/// the C-level malloc gate (any library). malloc_gate.hpp may be included in
/// exactly one translation unit per binary.

#include "ocp/internal/structured_quadratic_costs.hpp"
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_mpc/handler/kinodynamic_mpc.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp_config.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "test_utils/alloc_counter.hpp"
#include "test_utils/alloc_counter_overrides.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/kinodynamic_ocp_fixture.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/explicit-dynamics.hpp>
#include <aligator/core/stage-data.hpp>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>

namespace {

namespace tu = rtc::mpc::test_utils;
using rtc::mpc::HorizonReference;
using rtc::mpc::KinodynamicOCP;
using rtc::mpc::OCPBuildError;
using rtc::mpc::PhaseContext;
using tu::AllocCounter;

/// Allocations of one call of @p fn: {operator new, C malloc family}.
template <typename Fn>
std::pair<std::int64_t, std::size_t> Count(Fn&& fn) {
  std::size_t c = 0;
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    fn();
    c = gate.count();
  }
  AllocCounter::Disarm();
  return {AllocCounter::AllocCount(), c};
}

class KinodynamicOCPAllocGate : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(tu::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << tu::kG1Urdf;
    }
    model_ = tu::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(tu::kG1SolesYaml)),
              rtc::mpc::RobotModelInitError::kNoError);
    ASSERT_EQ(rtc::mpc::LoadKinodynamicOCPConfig(tu::G1KinodynamicNode(), handler_, cfg_),
              rtc::mpc::KinodynamicOCPConfigError::kNoError);
    stand_ = tu::G1Stand(model_);
  }

  [[nodiscard]] static PhaseContext Context(const HorizonReference* h) {
    PhaseContext ctx;
    ctx.ocp_type = std::string(KinodynamicOCP::kType);
    ctx.cost_config.horizon_length = tu::kKinoNodes;
    ctx.cost_config.dt = tu::kKinoDt;
    ctx.horizon = h;
    return ctx;
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  rtc::mpc::KinodynamicOCPConfig cfg_;
  Eigen::VectorXd stand_;
};

TEST_F(KinodynamicOCPAllocGate, PositiveControlsAreCounted) {
  const auto [news, mallocs] = Count([] {
    auto p = std::make_unique<int>(3);
    static_cast<void>(p);
  });
  EXPECT_GE(news, 1);
  EXPECT_GE(mallocs, 1U);
}

// Stance → one foot swinging → the other swinging → stance: every mode change
// in place, and a refused horizon too — nothing allocates.
TEST_F(KinodynamicOCPAllocGate, UpdateReferencesAllocatesNothing) {
  const auto stance = tu::StanceHorizon(handler_, stand_);
  auto right = tu::StanceHorizon(handler_, stand_);
  tu::SwingContact(handler_, *right, 1, 8, 19, 0.05, 0.5);
  auto left = tu::StanceHorizon(handler_, stand_);
  tu::SwingContact(handler_, *left, 0, 3, 14, 0.05, 0.005);
  auto bad = tu::StanceHorizon(handler_, stand_);
  bad->stages[tu::kKinoNodes - 1].mode = 4;

  KinodynamicOCP ocp(cfg_);
  ASSERT_EQ(ocp.Build(Context(stance.get()), handler_, {}), OCPBuildError::kNoError);
  // Warm the path once (nothing in it is lazily allocated, but the gate
  // should not depend on that).
  ASSERT_EQ(ocp.UpdateReferences(Context(right.get())), OCPBuildError::kNoError);

  const PhaseContext ctx_r = Context(right.get());
  const PhaseContext ctx_l = Context(left.get());
  const PhaseContext ctx_s = Context(stance.get());
  const PhaseContext ctx_b = Context(bad.get());
  OCPBuildError e1 = OCPBuildError::kNoError;
  OCPBuildError e2 = OCPBuildError::kNoError;
  OCPBuildError e3 = OCPBuildError::kNoError;
  OCPBuildError e4 = OCPBuildError::kNoError;
  const auto [news, mallocs] = Count([&] {
    e1 = ocp.UpdateReferences(ctx_l);
    e2 = ocp.UpdateReferences(ctx_s);
    e3 = ocp.UpdateReferences(ctx_r);
    e4 = ocp.UpdateReferences(ctx_b);
  });
  EXPECT_EQ(e1, OCPBuildError::kNoError);
  EXPECT_EQ(e2, OCPBuildError::kNoError);
  EXPECT_EQ(e3, OCPBuildError::kNoError);
  EXPECT_EQ(e4, OCPBuildError::kInvalidHorizonReference);
  EXPECT_EQ(news, 0);
  EXPECT_EQ(mallocs, 0U);
}

// The reference control and the constraint residuals of a whole trajectory:
// no C++ allocation; the residuals' C allocations are those of the ODE
// `forward` each stage's equality makes (measured on the same ODE, the same
// number of times), and two calls count the same.
TEST_F(KinodynamicOCPAllocGate, ResidualsAndTheReferenceControlAllocateOnlyWhatTheOdeDoes) {
  auto h = tu::StanceHorizon(handler_, stand_);
  tu::SwingContact(handler_, *h, 1, 8, 19, 0.05, 0.5);
  KinodynamicOCP ocp(cfg_);
  ASSERT_EQ(ocp.Build(Context(h.get()), handler_, {}), OCPBuildError::kNoError);
  const int n = tu::kKinoNodes;
  Eigen::VectorXd x0(model_.nq + model_.nv);
  x0 << stand_, Eigen::VectorXd::Zero(model_.nv);
  std::vector<Eigen::VectorXd> xs(static_cast<std::size_t>(n + 1), x0);
  std::vector<Eigen::VectorXd> us(static_cast<std::size_t>(n),
                                  Eigen::VectorXd::Zero(handler_.nu()));
  for (int k = 0; k < n; ++k) {
    ASSERT_TRUE(ocp.ReferenceControl(k, us[static_cast<std::size_t>(k)]));
  }

  Eigen::VectorXd u(handler_.nu());
  bool ok = true;
  const auto [ref_news, ref_mallocs] = Count([&] {
    for (int k = 0; k < n; ++k) {
      ok = ocp.ReferenceControl(k, u) && ok;
    }
  });
  EXPECT_TRUE(ok);
  EXPECT_EQ(ref_news, 0);
  EXPECT_EQ(ref_mallocs, 0U);

  // What the ODE's forward allocates by itself, over the same stages.
  auto& stages = ocp.problem().stages_;
  std::vector<std::shared_ptr<aligator::dynamics::ContinuousDynamicsDataTpl<double>>> ode_data;
  for (auto& stage : stages) {
    ode_data.push_back(rtc::mpc::KinodynamicDynamics::OdeOf(*stage)->createData());
  }
  const auto [ode_news, ode_mallocs] = Count([&] {
    for (std::size_t k = 0; k < stages.size(); ++k) {
      rtc::mpc::KinodynamicDynamics::OdeOf(*stages[k])->forward(xs[k], us[k], *ode_data[k]);
    }
  });

  rtc::mpc::ConstraintResidualReport r;
  ASSERT_TRUE(ocp.ConstraintResiduals(xs, us, r));  // warm
  bool ok1 = false;
  bool ok2 = false;
  const auto [news1, mallocs1] = Count([&] { ok1 = ocp.ConstraintResiduals(xs, us, r); });
  const auto [news2, mallocs2] = Count([&] { ok2 = ocp.ConstraintResiduals(xs, us, r); });
  std::printf("[report] ConstraintResiduals: %zu / %zu C allocations (ODE forward alone %zu)\n",
              mallocs1, mallocs2, ode_mallocs);
  EXPECT_TRUE(ok1);
  EXPECT_TRUE(ok2);
  EXPECT_EQ(ode_news, 0);
  EXPECT_EQ(news1, 0);
  EXPECT_EQ(news2, 0);
  EXPECT_EQ(mallocs1, ode_mallocs);
  EXPECT_EQ(mallocs2, mallocs1);
}

// A running stage with the shared stage kinematics (the default build), on a
// stance node and on a node with a swing contact tracked: its evaluate and
// first-order derivatives allocate nothing beyond what its own dynamics step
// (the integrator's forward / dForward, alone, on data of its own) does; the
// per-residual build is reported beside it.
TEST_F(KinodynamicOCPAllocGate, ASharedKinematicsStageAllocatesOnlyWhatItsDynamicsDoes) {
  auto h = tu::StanceHorizon(handler_, stand_);
  tu::SwingContact(handler_, *h, 1, 8, 19, 0.05, 0.5);
  KinodynamicOCP shared(cfg_);
  KinodynamicOCP own(cfg_, KinodynamicOCP::StageKinematics::kPerResidual);
  ASSERT_EQ(shared.Build(Context(h.get()), handler_, {}), OCPBuildError::kNoError);
  ASSERT_EQ(own.Build(Context(h.get()), handler_, {}), OCPBuildError::kNoError);
  for (const std::size_t k : {std::size_t{0}, std::size_t{12}}) {
    Eigen::VectorXd x(model_.nq + model_.nv);
    x << stand_, Eigen::VectorXd::Constant(model_.nv, 0.1);
    Eigen::VectorXd u(handler_.nu());
    ASSERT_TRUE(shared.ReferenceControl(static_cast<int>(k), u));
    auto& stage = *shared.problem().stages_[k];
    auto& stage_own = *own.problem().stages_[k];
    const auto data = stage.createData();
    const auto data_own = stage_own.createData();
    const auto dyn_data = stage.dynamics_->createData();
    // Warm-up.
    stage.evaluate(x, u, *data);
    stage.computeFirstOrderDerivatives(x, u, *data);
    stage_own.evaluate(x, u, *data_own);
    stage_own.computeFirstOrderDerivatives(x, u, *data_own);
    stage.dynamics_->forward(x, u, *dyn_data);
    stage.dynamics_->dForward(x, u, *dyn_data);

    const auto fwd = Count([&] { stage.dynamics_->forward(x, u, *dyn_data); });
    const auto dfwd = Count([&] { stage.dynamics_->dForward(x, u, *dyn_data); });
    const auto eval = Count([&] { stage.evaluate(x, u, *data); });
    const auto der = Count([&] { stage.computeFirstOrderDerivatives(x, u, *data); });
    const auto eval_own = Count([&] { stage_own.evaluate(x, u, *data_own); });
    const auto der_own = Count([&] { stage_own.computeFirstOrderDerivatives(x, u, *data_own); });
    std::printf(
        "[report] stage %zu C allocations: evaluate %zu (dynamics %zu, per residual %zu), "
        "derivatives %zu (dynamics %zu, per residual %zu)\n",
        k, eval.second, fwd.second, eval_own.second, der.second, dfwd.second, der_own.second);
    EXPECT_EQ(eval.first, 0) << "stage " << k;
    EXPECT_EQ(der.first, 0) << "stage " << k;
    EXPECT_EQ(eval.second, fwd.second) << "stage " << k;
    EXPECT_EQ(der.second, dfwd.second) << "stage " << k;
  }
}

// The quadratic state and input costs of a running stage (stance and swing)
// and the terminal cost: evaluate, gradients and Hessians allocate nothing
// with the structured derivatives (the default build, checked to be the
// structured classes and to have found the structure). Aligator's dense path
// is reported beside it.
TEST_F(KinodynamicOCPAllocGate, TheQuadraticCostsAllocateNothing) {
  using Derivatives = KinodynamicOCP::QuadraticCostDerivatives;
  using CostAbstract = aligator::CostAbstractTpl<double>;
  using CostStack = aligator::CostStackTpl<double>;
  auto h = tu::StanceHorizon(handler_, stand_);
  tu::SwingContact(handler_, *h, 1, 8, 19, 0.05, 0.5);
  for (const Derivatives derivatives : {Derivatives::kStructured, Derivatives::kDense}) {
    const bool structured = derivatives == Derivatives::kStructured;
    KinodynamicOCP ocp(cfg_, KinodynamicOCP::StageKinematics::kShared, derivatives);
    ASSERT_EQ(ocp.Build(Context(h.get()), handler_, {}), OCPBuildError::kNoError);
    // State and input of stages 0 and 12, then the terminal cost.
    std::vector<const CostAbstract*> costs;
    for (const std::size_t k : {std::size_t{0}, std::size_t{12}}) {
      CostStack* stack = ocp.problem().stages_[k]->getCost<CostStack>();
      ASSERT_NE(stack, nullptr);
      costs.push_back(
          stack->getComponent<CostAbstract>(std::string(KinodynamicOCP::kStateCostKey)));
      costs.push_back(
          stack->getComponent<CostAbstract>(std::string(KinodynamicOCP::kInputCostKey)));
    }
    costs.push_back(&*ocp.problem().term_cost_);
    for (std::size_t i = 0; i < costs.size(); ++i) {
      ASSERT_NE(costs[i], nullptr);
      const bool input = i < 4 && i % 2 == 1;
      const bool is_aligator = input ? typeid(*costs[i]) == typeid(KinodynamicOCP::QuadControlCost)
                                     : typeid(*costs[i]) == typeid(KinodynamicOCP::QuadStateCost);
      EXPECT_EQ(is_aligator, !structured) << "cost " << i;
      if (structured) {
        // A structured class that fell back to Aligator's path would pass the
        // count below without covering the structured one.
        const auto* sx = dynamic_cast<const rtc::mpc::internal::StructuredQuadStateCost*>(costs[i]);
        const auto* su =
            dynamic_cast<const rtc::mpc::internal::StructuredQuadControlCost*>(costs[i]);
        ASSERT_TRUE(input ? su != nullptr : sx != nullptr) << "cost " << i;
        ASSERT_TRUE(input ? su->structured() : sx->structured()) << "cost " << i;
      }
    }
    Eigen::VectorXd x(model_.nq + model_.nv);
    x << stand_, Eigen::VectorXd::Constant(model_.nv, 0.1);
    x[0] += 0.03;
    Eigen::VectorXd u(handler_.nu());
    ASSERT_TRUE(ocp.ReferenceControl(12, u));
    u.array() += 0.5;
    std::vector<std::shared_ptr<aligator::CostDataAbstractTpl<double>>> data;
    for (const CostAbstract* c : costs) {
      data.push_back(c->createData());
    }
    for (std::size_t i = 0; i < costs.size(); ++i) {  // warm-up
      costs[i]->evaluate(x, u, *data[i]);
      costs[i]->computeGradients(x, u, *data[i]);
      costs[i]->computeHessians(x, u, *data[i]);
    }
    const auto [news, mallocs] = Count([&] {
      for (std::size_t i = 0; i < costs.size(); ++i) {
        costs[i]->evaluate(x, u, *data[i]);
        costs[i]->computeGradients(x, u, *data[i]);
        costs[i]->computeHessians(x, u, *data[i]);
      }
    });
    std::printf("[report] quadratic costs (%s): %lld C++ / %zu C allocations\n",
                structured ? "structured" : "dense", static_cast<long long>(news), mallocs);
    if (structured) {
      EXPECT_EQ(news, 0);
      EXPECT_EQ(mallocs, 0U);
    }
  }
}

// The multipliers of the rows a mode makes inactive are cleared in place.
TEST_F(KinodynamicOCPAllocGate, ClearingInactiveMultipliersAllocatesNothing) {
  auto right = tu::StanceHorizon(handler_, stand_);
  tu::SwingContact(handler_, *right, 1, 8, 19, 0.05, 0.5);
  KinodynamicOCP ocp(cfg_);
  ASSERT_EQ(ocp.Build(Context(right.get()), handler_, {}), OCPBuildError::kNoError);
  const int rows = handler_.n_contacts() * rtc::mpc::ContactConstraintResidual::EqualityRows(6);
  std::vector<Eigen::VectorXd> vs(tu::kKinoNodes + 1, Eigen::VectorXd::Ones(rows));
  std::size_t cleared = 0;
  const auto [news, mallocs] = Count([&] { cleared = ocp.ClearInactiveMultipliers(vs); });
  EXPECT_GT(cleared, 0U);
  EXPECT_EQ(news, 0);
  EXPECT_EQ(mallocs, 0U);
}

// Through the handler core: a tick whose horizon has another shape makes the
// core try a rebuild, which the OCP refuses before it touches anything — the
// tick allocates nothing.
TEST_F(KinodynamicOCPAllocGate, AMismatchedTickThroughTheCoreAllocatesNothing) {
  const auto good = tu::StanceHorizon(handler_, stand_);
  auto bad = tu::StanceHorizon(handler_, stand_);
  bad->n_stages = tu::kKinoNodes;
  rtc::mpc::MPCSolverConfig solver;
  solver.prim_tol = 1e-4;
  solver.dual_tol = 1e-4;
  rtc::mpc::KinodynamicMPC mpc(cfg_);
  ASSERT_EQ(mpc.Init(solver, handler_, {}, Context(good.get())), rtc::mpc::MPCInitError::kNoError);
  auto out = std::make_unique<rtc::mpc::MPCSolution>();
  const rtc::mpc::MPCStateSnapshot snap = tu::SnapshotAt(model_, stand_);
  ASSERT_EQ(mpc.Solve(Context(good.get()), snap, *out), rtc::mpc::MPCSolveError::kNoError);
  const PhaseContext ctx_bad = Context(bad.get());
  rtc::mpc::MPCSolveError e = rtc::mpc::MPCSolveError::kNoError;
  const auto [news, mallocs] = Count([&] { e = mpc.Solve(ctx_bad, snap, *out); });
  EXPECT_EQ(e, rtc::mpc::MPCSolveError::kOCPRebuildFailed);
  EXPECT_EQ(news, 0);
  EXPECT_EQ(mallocs, 0U);
}

}  // namespace
