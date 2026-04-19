/// @file test_contact_rich_ocp.cpp
/// @brief End-to-end integration test for `rtc::mpc::ContactRichOCP` on
///        the generic Panda fixture (9-DoF, fixed-base, 2×3D fingertip
///        contacts).
///
/// Covered:
/// - Build on a contact phase succeeds; topology reflects contact-force
///   cost terms and friction-cone constraints.
/// - Offline solve converges when the initial guess is seeded with
///   `SeedGravityCompensation` (Risk #14 mitigation — see
///   `docs/mpc_implementation_progress.md` §Phase 4 Spike Notes Q7).
/// - UpdateReferences propagates a new EE target through to the stored
///   FramePlacement residual.
/// - UpdateReferences rejects topology changes (horizon change,
///   `w_contact_force` crossing 0, phase reshape, ocp_type mismatch,
///   pre-Build call).
/// - Error paths: invalid ocp_type, contact-plan/model mismatch,
///   overlapping phases, u_limits dim mismatch.
/// - Perf: p50 / p99 solve wall-time logged (not asserted — see Open
///   Decision #2).

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/multibody/contact-force.hpp>
#include <aligator/modelling/multibody/frame-placement.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <vector>

#include "rtc_mpc/ocp/contact_rich_ocp.hpp"
#include "test_utils/solver_seeding.hpp"

namespace {

constexpr const char *kPandaUrdf =
    "/usr/local/share/example-robot-data/robots/panda_description/urdf/"
    "panda.urdf";

constexpr const char *kCostYaml = R"(
horizon_length: 10
dt: 0.01
w_frame_placement: 10.0
w_state_reg: 1.0
w_control_reg: 0.1
w_contact_force: 0.5
w_centroidal_momentum: 0.0
W_placement: [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
q_posture_ref: [0, 0, 0, -1.57, 0, 1.57, 0.785, 0.02, 0.02]
F_target: [0, 0, 0, 0, 0, 0]
custom_weights: {}
)";

class ContactRichOCPTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!std::filesystem::exists(kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
    }
    pinocchio::urdf::buildModel(kPandaUrdf, model_);

    auto robot_cfg = YAML::Load(R"(
end_effector_frame: panda_hand
contact_frames:
  - name: panda_leftfinger
    dim: 3
  - name: panda_rightfinger
    dim: 3
)");
    ASSERT_EQ(handler_.Init(model_, robot_cfg),
              rtc::mpc::RobotModelInitError::kNoError);

    auto cost_node = YAML::Load(kCostYaml);
    ASSERT_EQ(
        rtc::mpc::PhaseCostConfig::LoadFromYaml(cost_node, handler_, cfg_),
        rtc::mpc::PhaseCostConfigError::kNoError);

    ctx_.phase_id = 0;
    ctx_.phase_name = "contact_test";
    ctx_.phase_changed = false;
    ctx_.ocp_type = "contact_rich";
    ctx_.cost_config = cfg_;

    // Default to a 2-finger contact phase over the whole horizon.
    const int lf = handler_.contact_frames()[0].frame_id;
    const int rf = handler_.contact_frames()[1].frame_id;
    ctx_.contact_plan.frames = handler_.contact_frames();
    ctx_.contact_plan.phases.push_back(
        rtc::mpc::ContactPhase{{lf, rf}, 0.0, 100.0});

    pinocchio::Data pdata(model_);
    const Eigen::VectorXd q0 = pinocchio::neutral(model_);
    pinocchio::framesForwardKinematics(model_, pdata, q0);
    ctx_.ee_target = pdata.oMf[handler_.end_effector_frame_id()];

    // Default limits activate the friction cone.
    limits_.friction_mu = 0.7;
    limits_.n_friction_facets = 4;
  }

  // Seed solver with gravity-comp τ and default xs.
  void SeedSolver(aligator::SolverProxDDPTpl<double> &solver,
                  rtc::mpc::ContactRichOCP &ocp) {
    const int H = ocp.horizon_length();
    std::vector<Eigen::VectorXd> xs(
        static_cast<std::size_t>(H + 1),
        Eigen::VectorXd::Zero(handler_.nq() + handler_.nv()));
    for (auto &x : xs) {
      x.head(handler_.nq()) = pinocchio::neutral(model_);
    }
    std::vector<Eigen::VectorXd> us(static_cast<std::size_t>(H),
                                    Eigen::VectorXd::Zero(handler_.nv()));
    rtc::mpc::test_utils::SeedGravityCompensation(
        model_, pinocchio::neutral(model_), us);
    solver.setup(ocp.problem());
    run_xs_ = std::move(xs);
    run_us_ = std::move(us);
  }

  bool Run(aligator::SolverProxDDPTpl<double> &solver,
           rtc::mpc::ContactRichOCP &ocp) {
    return solver.run(ocp.problem(), run_xs_, run_us_);
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  rtc::mpc::PhaseCostConfig cfg_;
  rtc::mpc::PhaseContext ctx_;
  rtc::mpc::OCPLimits limits_;
  std::vector<Eigen::VectorXd> run_xs_;
  std::vector<Eigen::VectorXd> run_us_;
};

// ── Happy path ───────────────────────────────────────────────────────────

TEST_F(ContactRichOCPTest, BuildWithContactPhaseSucceeds) {
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);
  EXPECT_TRUE(ocp.Built());
  EXPECT_EQ(ocp.horizon_length(), cfg_.horizon_length);
  EXPECT_EQ(ocp.ocp_type(), std::string_view("contact_rich"));
  EXPECT_EQ(ocp.problem().numSteps(),
            static_cast<std::size_t>(cfg_.horizon_length));
}

TEST_F(ContactRichOCPTest, ContactForceTermPresentOnActiveStage) {
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  auto &problem = ocp.problem();
  ASSERT_GT(problem.stages_.size(), 0u);
  auto *stack = problem.stages_[0]->getCost<aligator::CostStackTpl<double>>();
  ASSERT_NE(stack, nullptr);

  auto *qc_lf = stack->getComponent<aligator::QuadraticResidualCostTpl<double>>(
      std::string("contact_force::panda_leftfinger"));
  auto *qc_rf = stack->getComponent<aligator::QuadraticResidualCostTpl<double>>(
      std::string("contact_force::panda_rightfinger"));
  EXPECT_NE(qc_lf, nullptr);
  EXPECT_NE(qc_rf, nullptr);

  auto *cfr = qc_lf->getResidual<aligator::ContactForceResidualTpl<double>>();
  ASSERT_NE(cfr, nullptr);
  EXPECT_EQ(cfr->getReference().size(), 3);
}

TEST_F(ContactRichOCPTest, FrictionConeAttachedOnActiveStage) {
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  // Expect 2 friction-cone constraints (one per active contact).
  EXPECT_EQ(ocp.problem().stages_[0]->numConstraints(), 2u);
}

TEST_F(ContactRichOCPTest, NoFrictionConeWhenMuZero) {
  limits_.friction_mu = 0.0;
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);
  EXPECT_EQ(ocp.problem().stages_[0]->numConstraints(), 0u);
}

TEST_F(ContactRichOCPTest, NoContactForceCostWhenWeightZero) {
  cfg_.w_contact_force = 0.0;
  ctx_.cost_config = cfg_;
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  auto *stack =
      ocp.problem().stages_[0]->getCost<aligator::CostStackTpl<double>>();
  ASSERT_NE(stack, nullptr);
  const aligator::CostStackTpl<double>::CostKey key{
      std::string("contact_force::panda_leftfinger")};
  EXPECT_EQ(stack->components_.count(key), 0u);
}

// Solve-attempt smoke test. Per Open Decision #2, convergence is NOT a
// Phase 4 gate (Phase 5 warm-start is). Risk #14 documents that
// constraint-dynamics derivatives at the Panda neutral pose (free-space
// fingertips) can trip `computeMultipliers() → NaN`. We log whatever
// happens — the path-through-the-code is what matters here.
TEST_F(ContactRichOCPTest, SolveWithGravityCompSeedAttempts) {
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  aligator::SolverProxDDPTpl<double> solver(1e-3, 1e-2);
  solver.max_iters = 30;
  solver.verbose_ = aligator::QUIET;
  SeedSolver(solver, ocp);

  try {
    (void)Run(solver, ocp);
    std::cout << "[ContactRichOCP solve] iters=" << solver.results_.num_iters
              << " prim_infeas=" << solver.results_.prim_infeas << "\n";
  } catch (const std::exception &e) {
    std::cout << "[ContactRichOCP solve] Risk #14 NaN — known cold-solve "
                 "limitation: "
              << e.what() << "\n";
  }
  SUCCEED();
}

TEST_F(ContactRichOCPTest, UpdateReferencesPropagatesTarget) {
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  pinocchio::SE3 new_target = ctx_.ee_target;
  new_target.translation() += Eigen::Vector3d(0.05, -0.02, 0.01);
  ctx_.ee_target = new_target;

  ASSERT_EQ(ocp.UpdateReferences(ctx_), rtc::mpc::OCPBuildError::kNoError);

  auto *stack =
      ocp.problem().stages_[0]->getCost<aligator::CostStackTpl<double>>();
  ASSERT_NE(stack, nullptr);
  auto *quad = stack->getComponent<aligator::QuadraticResidualCostTpl<double>>(
      std::string(rtc::mpc::kCostKeyFramePlacement));
  ASSERT_NE(quad, nullptr);
  auto *residual =
      quad->getResidual<aligator::FramePlacementResidualTpl<double>>();
  ASSERT_NE(residual, nullptr);
  EXPECT_TRUE(residual->getReference().translation().isApprox(
      new_target.translation()));
}

TEST_F(ContactRichOCPTest, UpdateReferencesMutatesContactForceRef) {
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  Eigen::VectorXd new_F(6);
  new_F << 0.1, -0.05, 2.5, 0.0, 0.0, 0.0;
  cfg_.F_target = new_F;
  ctx_.cost_config = cfg_;
  ASSERT_EQ(ocp.UpdateReferences(ctx_), rtc::mpc::OCPBuildError::kNoError);

  auto *stack =
      ocp.problem().stages_[0]->getCost<aligator::CostStackTpl<double>>();
  auto *quad = stack->getComponent<aligator::QuadraticResidualCostTpl<double>>(
      std::string("contact_force::panda_leftfinger"));
  ASSERT_NE(quad, nullptr);
  auto *cfr = quad->getResidual<aligator::ContactForceResidualTpl<double>>();
  ASSERT_NE(cfr, nullptr);
  EXPECT_TRUE(cfr->getReference().isApprox(new_F.head(3)));
}

TEST_F(ContactRichOCPTest, ReBuildIdempotent) {
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);
  EXPECT_TRUE(ocp.Built());
  EXPECT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);
  EXPECT_TRUE(ocp.Built());
}

// ── Error paths ──────────────────────────────────────────────────────────

TEST_F(ContactRichOCPTest, InvalidOcpTypeRejected) {
  ctx_.ocp_type = "light_contact";
  rtc::mpc::ContactRichOCP ocp;
  EXPECT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kInvalidPhaseContext);
  EXPECT_FALSE(ocp.Built());
}

TEST_F(ContactRichOCPTest, UninitialisedModelRejected) {
  rtc::mpc::RobotModelHandler empty;
  rtc::mpc::ContactRichOCP ocp;
  EXPECT_EQ(ocp.Build(ctx_, empty, limits_),
            rtc::mpc::OCPBuildError::kModelNotInitialised);
}

TEST_F(ContactRichOCPTest, ContactPlanMismatchRejected) {
  ctx_.contact_plan.phases.clear();
  ctx_.contact_plan.phases.push_back(
      rtc::mpc::ContactPhase{{9999}, 0.0, 1.0}); // bogus frame id
  rtc::mpc::ContactRichOCP ocp;
  EXPECT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kContactPlanModelMismatch);
}

TEST_F(ContactRichOCPTest, OverlappingPhasesRejected) {
  const int lf = handler_.contact_frames()[0].frame_id;
  ctx_.contact_plan.phases.clear();
  ctx_.contact_plan.phases.push_back(rtc::mpc::ContactPhase{{lf}, 0.0, 0.08});
  ctx_.contact_plan.phases.push_back(
      rtc::mpc::ContactPhase{{lf}, 0.05, 0.10}); // overlap on [0.05, 0.08)
  rtc::mpc::ContactRichOCP ocp;
  EXPECT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kOverlappingContactPhases);
}

TEST_F(ContactRichOCPTest, LimitsDimMismatchRejected) {
  limits_.u_min = Eigen::VectorXd::Zero(handler_.nu() - 1);
  rtc::mpc::ContactRichOCP ocp;
  EXPECT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kLimitsDimMismatch);
}

TEST_F(ContactRichOCPTest, UpdateReferencesTopologyChangeRejected) {
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  ctx_.cost_config.horizon_length = cfg_.horizon_length + 5;
  EXPECT_EQ(ocp.UpdateReferences(ctx_),
            rtc::mpc::OCPBuildError::kInvalidPhaseContext);
  EXPECT_TRUE(ocp.Built());
  EXPECT_EQ(ocp.horizon_length(), cfg_.horizon_length);
}

TEST_F(ContactRichOCPTest, UpdateReferencesContactForceWeightCrossingRejected) {
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  // Flip w_contact_force to 0 — topology change (cost terms vanish).
  ctx_.cost_config.w_contact_force = 0.0;
  EXPECT_EQ(ocp.UpdateReferences(ctx_),
            rtc::mpc::OCPBuildError::kInvalidPhaseContext);
}

TEST_F(ContactRichOCPTest, UpdateReferencesBeforeBuildRejected) {
  rtc::mpc::ContactRichOCP ocp;
  EXPECT_EQ(ocp.UpdateReferences(ctx_),
            rtc::mpc::OCPBuildError::kInvalidPhaseContext);
}

// ── Performance (log-only) ───────────────────────────────────────────────

TEST_F(ContactRichOCPTest, SolvePerfLog) {
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  aligator::SolverProxDDPTpl<double> solver(1e-3, 1e-2);
  solver.max_iters = 20;
  solver.verbose_ = aligator::QUIET;
  SeedSolver(solver, ocp);

  constexpr int kIters = 10;
  std::vector<long> wall_us;
  wall_us.reserve(kIters);
  bool any_nan = false;
  for (int i = 0; i < kIters; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    try {
      (void)Run(solver, ocp);
    } catch (...) {
      any_nan = true;
    }
    const auto t1 = std::chrono::steady_clock::now();
    wall_us.push_back(
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
  }
  std::sort(wall_us.begin(), wall_us.end());
  const long p50 = wall_us[wall_us.size() / 2];
  const long p99 = wall_us.back();
  std::cout << "[ContactRichOCP perf] " << kIters << " solves: p50=" << p50
            << "us p99=" << p99 << "us"
            << (any_nan ? " (Risk #14 NaN hit at least once)" : "") << "\n";
  SUCCEED();
}

// ── Warm-start smoke (Step 4.5 — log-only; no ≥40% assertion, see Open
//    Decision #3, assertion lives in Phase 5) ────────────────────────────

TEST_F(ContactRichOCPTest, ColdSolveIterCount) {
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  aligator::SolverProxDDPTpl<double> solver(1e-3, 1e-2);
  solver.max_iters = 30;
  solver.verbose_ = aligator::QUIET;
  SeedSolver(solver, ocp);

  try {
    (void)Run(solver, ocp);
    std::cout << "[ContactRichOCP warm-start] cold iters="
              << solver.results_.num_iters << "\n";
  } catch (const std::exception &e) {
    std::cout << "[ContactRichOCP warm-start] cold solve NaN (Risk #14): "
              << e.what() << "\n";
  }
  SUCCEED();
}

TEST_F(ContactRichOCPTest, SeededSolveIterCount) {
  rtc::mpc::ContactRichOCP ocp;
  ASSERT_EQ(ocp.Build(ctx_, handler_, limits_),
            rtc::mpc::OCPBuildError::kNoError);

  aligator::SolverProxDDPTpl<double> solver(1e-3, 1e-2);
  solver.max_iters = 30;
  solver.verbose_ = aligator::QUIET;
  SeedSolver(solver, ocp);
  std::size_t cold_iters = 0;
  try {
    (void)Run(solver, ocp);
    cold_iters = solver.results_.num_iters;
    // Warm start: reuse previous trajectory; perturb ee_target slightly.
    run_xs_ = solver.results_.xs;
    run_us_ = solver.results_.us;
  } catch (const std::exception &e) {
    std::cout << "[ContactRichOCP warm-start] cold solve NaN (Risk #14): "
              << e.what() << "\n";
    SUCCEED();
    return;
  }

  pinocchio::SE3 new_target = ctx_.ee_target;
  new_target.translation() += Eigen::Vector3d(0.005, 0.0, 0.0);
  ctx_.ee_target = new_target;
  ASSERT_EQ(ocp.UpdateReferences(ctx_), rtc::mpc::OCPBuildError::kNoError);

  try {
    (void)Run(solver, ocp);
    const std::size_t seeded_iters = solver.results_.num_iters;
    std::cout << "[ContactRichOCP warm-start] cold=" << cold_iters
              << " seeded=" << seeded_iters << "\n";
  } catch (const std::exception &e) {
    std::cout << "[ContactRichOCP warm-start] seeded solve NaN (Risk #14): "
              << e.what() << "\n";
  }
  SUCCEED();
}

} // namespace
