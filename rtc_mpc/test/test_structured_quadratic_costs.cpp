/// @file test_structured_quadratic_costs.cpp
/// @brief The structured quadratic state and control costs
///        (`ocp/internal/structured_quadratic_costs.hpp`) against Aligator's
///        own `QuadraticStateCostTpl` / `QuadraticControlCostTpl`:
///
/// - on the public G1, with the G1 fixture weights (`G1KinodynamicNode()`) and
///   with unit weights, over
///   1000 random `(x, x_ref, u, u_ref)` (seed 20261117, `x` and `x_ref` on
///   the manifold): value, gradient and Hessian of the running state cost,
///   the input cost and the terminal cost agree to `1e-12 · max(1, |ref|)`
///   per entry, and an entry is 0 exactly where Aligator's is;
/// - a model without the structure (a spherical or unbounded joint, a fixed
///   base), non-diagonal weights and `gauss_newton` cleared all fall back to
///   Aligator's path: the same bits;
/// - the OCP builds the structured costs by default and Aligator's with
///   `QuadraticCostDerivatives::kDense`, and a whole running stage and the
///   terminal cost of the two builds agree.

#include "ocp/internal/structured_quadratic_costs.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp_config.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/kinodynamic_ocp_fixture.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <aligator/core/stage-data.hpp>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <typeinfo>

namespace {

namespace tu = rtc::mpc::test_utils;
using rtc::mpc::KinodynamicOCP;
using rtc::mpc::OCPBuildError;
using rtc::mpc::PhaseContext;
using rtc::mpc::internal::HasFreeFlyerRootAndEuclideanJoints;
using rtc::mpc::internal::StructuredQuadControlCost;
using rtc::mpc::internal::StructuredQuadStateCost;
using Space = aligator::MultibodyPhaseSpace<double>;
using CostData = aligator::CostDataAbstractTpl<double>;
using CostAbstract = aligator::CostAbstractTpl<double>;
using QuadStateCost = KinodynamicOCP::QuadStateCost;
using QuadControlCost = KinodynamicOCP::QuadControlCost;
using CostStack = aligator::CostStackTpl<double>;

constexpr std::uint32_t kSeed = 20261117U;
constexpr int kDraws = 1000;
constexpr double kTol = 1e-12;

/// Largest `|Δ| / max(1, |ref|)` per part, and the entries whose zero pattern
/// differs.
struct Agreement {
  double value{0.0};
  double grad{0.0};
  double hess{0.0};
  long pattern_mismatches{0};

  void Merge(const Agreement& o) {
    value = std::max(value, o.value);
    grad = std::max(grad, o.grad);
    hess = std::max(hess, o.hess);
    pattern_mismatches += o.pattern_mismatches;
  }
};

[[nodiscard]] double RelDelta(double ref, double got) {
  return std::fabs(got - ref) / std::max(1.0, std::fabs(ref));
}

[[nodiscard]] Agreement Compare(const CostData& ref, const CostData& got) {
  Agreement a;
  a.value = RelDelta(ref.value_, got.value_);
  for (Eigen::Index i = 0; i < ref.grad_.size(); ++i) {
    a.grad = std::max(a.grad, RelDelta(ref.grad_[i], got.grad_[i]));
    a.pattern_mismatches += (ref.grad_[i] == 0.0) != (got.grad_[i] == 0.0) ? 1 : 0;
  }
  for (Eigen::Index j = 0; j < ref.hess_.cols(); ++j) {
    for (Eigen::Index i = 0; i < ref.hess_.rows(); ++i) {
      a.hess = std::max(a.hess, RelDelta(ref.hess_(i, j), got.hess_(i, j)));
      a.pattern_mismatches += (ref.hess_(i, j) == 0.0) != (got.hess_(i, j) == 0.0) ? 1 : 0;
    }
  }
  return a;
}

/// Same bits: value, every gradient and Hessian entry.
[[nodiscard]] bool SameBits(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
  return a.rows() == b.rows() && a.cols() == b.cols() &&
         std::memcmp(a.data(), b.data(), sizeof(double) * static_cast<std::size_t>(a.size())) == 0;
}

[[nodiscard]] bool BitEqual(const CostData& a, const CostData& b) {
  return std::memcmp(&a.value_, &b.value_, sizeof(double)) == 0 && SameBits(a.grad_, b.grad_) &&
         SameBits(a.hess_, b.hess_);
}

/// A point of @p space: its neutral element moved along a random tangent
/// (base translation ±1 m, every rotation and joint coordinate ±π, velocities
/// ±3).
[[nodiscard]] Eigen::VectorXd RandomState(const Space& space, std::mt19937& rng) {
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  const int nv = space.getModel().nv;
  Eigen::VectorXd dx(space.ndx());
  for (int i = 0; i < space.ndx(); ++i) {
    const double scale = i < 3 ? 1.0 : (i < nv ? M_PI : 3.0);
    dx[i] = scale * unit(rng);
  }
  Eigen::VectorXd x(space.nx());
  space.integrate(space.neutral(), dx, x);
  return x;
}

[[nodiscard]] Eigen::VectorXd RandomVector(int n, double scale, std::mt19937& rng) {
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  Eigen::VectorXd v(n);
  for (int i = 0; i < n; ++i) {
    v[i] = scale * unit(rng);
  }
  return v;
}

/// evaluate → computeGradients → computeHessians, as the solver calls them.
void Derivatives(const CostAbstract& cost, const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                 CostData& d) {
  cost.evaluate(x, u, d);
  cost.computeGradients(x, u, d);
  cost.computeHessians(x, u, d);
}

/// @p kDraws random `(x, x_ref, u, u_ref)` through Aligator's state cost and
/// the structured one, on one data object each (the zero entries a draw
/// leaves are the next draw's).
[[nodiscard]] Agreement CompareStateCosts(const Space& space, int nu, const Eigen::MatrixXd& w) {
  std::mt19937 rng(kSeed);
  QuadStateCost ref(space, nu, space.neutral(), w);
  StructuredQuadStateCost got(space, nu, space.neutral(), w);
  EXPECT_TRUE(got.structured());
  const auto d_ref = ref.createData();
  const auto d_got = got.createData();
  Agreement worst;
  for (int k = 0; k < kDraws; ++k) {
    const Eigen::VectorXd x = RandomState(space, rng);
    const Eigen::VectorXd x_ref = RandomState(space, rng);
    const Eigen::VectorXd u = RandomVector(nu, 50.0, rng);
    ref.setTarget(x_ref);
    got.setTarget(x_ref);
    Derivatives(ref, x, u, *d_ref);
    Derivatives(got, x, u, *d_got);
    worst.Merge(Compare(*d_ref, *d_got));
  }
  return worst;
}

[[nodiscard]] Agreement CompareControlCosts(const Space& space, int nu, const Eigen::MatrixXd& w) {
  std::mt19937 rng(kSeed);
  QuadControlCost ref(space, Eigen::VectorXd::Zero(nu), w);
  StructuredQuadControlCost got(space, Eigen::VectorXd::Zero(nu), w);
  EXPECT_TRUE(got.structured());
  const auto d_ref = ref.createData();
  const auto d_got = got.createData();
  Agreement worst;
  for (int k = 0; k < kDraws; ++k) {
    const Eigen::VectorXd x = RandomState(space, rng);
    const Eigen::VectorXd u = RandomVector(nu, 50.0, rng);
    const Eigen::VectorXd u_ref = RandomVector(nu, 50.0, rng);
    ref.setTarget(u_ref);
    got.setTarget(u_ref);
    Derivatives(ref, x, u, *d_ref);
    Derivatives(got, x, u, *d_got);
    worst.Merge(Compare(*d_ref, *d_got));
  }
  return worst;
}

void ExpectAgreement(const Agreement& a, const char* what) {
  std::printf(
      "[report] %s: max relative delta value %.3g, gradient %.3g, Hessian %.3g; "
      "zero-pattern mismatches %ld\n",
      what, a.value, a.grad, a.hess, a.pattern_mismatches);
  EXPECT_LE(a.value, kTol) << what;
  EXPECT_LE(a.grad, kTol) << what;
  EXPECT_LE(a.hess, kTol) << what;
  EXPECT_EQ(a.pattern_mismatches, 0) << what;
}

// ── The public G1 ────────────────────────────────────────────────────────────

class StructuredQuadraticCostsG1 : public ::testing::Test {
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
    space_ = std::make_unique<Space>(handler_.model());
    nu_ = handler_.nu();
  }

  [[nodiscard]] static PhaseContext Context() {
    PhaseContext ctx;
    ctx.ocp_type = std::string(KinodynamicOCP::kType);
    ctx.cost_config.horizon_length = tu::kKinoNodes;
    ctx.cost_config.dt = tu::kKinoDt;
    return ctx;
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  rtc::mpc::KinodynamicOCPConfig cfg_;
  std::unique_ptr<Space> space_;
  int nu_{0};
};

TEST_F(StructuredQuadraticCostsG1, TheModelHasTheStructure) {
  EXPECT_TRUE(HasFreeFlyerRootAndEuclideanJoints(handler_.model()));
}

TEST_F(StructuredQuadraticCostsG1, TheRunningStateCostAgreesWithAligator) {
  const Eigen::MatrixXd fixture = cfg_.state_weights.asDiagonal();
  ExpectAgreement(CompareStateCosts(*space_, nu_, fixture), "running state, G1 fixture weights");
  const Eigen::MatrixXd unit = Eigen::MatrixXd::Identity(space_->ndx(), space_->ndx());
  ExpectAgreement(CompareStateCosts(*space_, nu_, unit), "running state, unit weights");
}

TEST_F(StructuredQuadraticCostsG1, TheTerminalStateCostAgreesWithAligator) {
  const Eigen::MatrixXd fixture = (cfg_.terminal_scaling * cfg_.terminal_weights).asDiagonal();
  ExpectAgreement(CompareStateCosts(*space_, nu_, fixture), "terminal state, G1 fixture weights");
}

TEST_F(StructuredQuadraticCostsG1, TheInputCostAgreesWithAligator) {
  const Eigen::MatrixXd fixture = cfg_.input_weights.asDiagonal();
  ExpectAgreement(CompareControlCosts(*space_, nu_, fixture), "input, G1 fixture weights");
  const Eigen::MatrixXd unit = Eigen::MatrixXd::Identity(nu_, nu_);
  ExpectAgreement(CompareControlCosts(*space_, nu_, unit), "input, unit weights");
}

// One off-diagonal weight: no structure, Aligator's path, the same bits.
TEST_F(StructuredQuadraticCostsG1, NonDiagonalWeightsFallBackToAligator) {
  std::mt19937 rng(kSeed);
  Eigen::MatrixXd wx = Eigen::MatrixXd::Identity(space_->ndx(), space_->ndx());
  wx(7, 9) = wx(9, 7) = 0.25;
  Eigen::MatrixXd wu = Eigen::MatrixXd::Identity(nu_, nu_);
  wu(1, 0) = wu(0, 1) = 0.25;
  QuadStateCost xref(*space_, nu_, space_->neutral(), wx);
  StructuredQuadStateCost xgot(*space_, nu_, space_->neutral(), wx);
  QuadControlCost uref(*space_, Eigen::VectorXd::Zero(nu_), wu);
  StructuredQuadControlCost ugot(*space_, Eigen::VectorXd::Zero(nu_), wu);
  EXPECT_FALSE(xgot.structured());
  EXPECT_FALSE(ugot.structured());
  const auto dx_ref = xref.createData();
  const auto dx_got = xgot.createData();
  const auto du_ref = uref.createData();
  const auto du_got = ugot.createData();
  for (int k = 0; k < 50; ++k) {
    const Eigen::VectorXd x = RandomState(*space_, rng);
    const Eigen::VectorXd x_ref = RandomState(*space_, rng);
    const Eigen::VectorXd u = RandomVector(nu_, 50.0, rng);
    xref.setTarget(x_ref);
    xgot.setTarget(x_ref);
    Derivatives(xref, x, u, *dx_ref);
    Derivatives(xgot, x, u, *dx_got);
    Derivatives(uref, x, u, *du_ref);
    Derivatives(ugot, x, u, *du_got);
    ASSERT_TRUE(BitEqual(*dx_ref, *dx_got)) << "draw " << k;
    ASSERT_TRUE(BitEqual(*du_ref, *du_got)) << "draw " << k;
  }
}

// `gauss_newton` cleared after construction: those calls are Aligator's (the
// same bits); set again, the structured path re-establishes its zero entries
// over what Aligator's path left.
TEST_F(StructuredQuadraticCostsG1, ClearingGaussNewtonFallsBackAndSettingItRestoresThePattern) {
  std::mt19937 rng(kSeed);
  const Eigen::MatrixXd wx = cfg_.state_weights.asDiagonal();
  const Eigen::MatrixXd wu = cfg_.input_weights.asDiagonal();
  QuadStateCost xref(*space_, nu_, space_->neutral(), wx);
  StructuredQuadStateCost xgot(*space_, nu_, space_->neutral(), wx);
  QuadControlCost uref(*space_, Eigen::VectorXd::Zero(nu_), wu);
  StructuredQuadControlCost ugot(*space_, Eigen::VectorXd::Zero(nu_), wu);
  const auto dx_ref = xref.createData();
  const auto dx_got = xgot.createData();
  const auto du_ref = uref.createData();
  const auto du_got = ugot.createData();
  const Eigen::VectorXd x = RandomState(*space_, rng);
  const Eigen::VectorXd x_ref = RandomState(*space_, rng);
  const Eigen::VectorXd u = RandomVector(nu_, 50.0, rng);
  xref.setTarget(x_ref);
  xgot.setTarget(x_ref);
  for (const bool gn : {false, true}) {
    xref.gauss_newton = gn;
    xgot.gauss_newton = gn;
    uref.gauss_newton = gn;
    ugot.gauss_newton = gn;
    // Poison what a structured call must not rely on.
    dx_got->grad_.setConstant(7.0);
    dx_got->hess_.setConstant(7.0);
    du_got->grad_.setConstant(7.0);
    du_got->hess_.setConstant(7.0);
    Derivatives(xref, x, u, *dx_ref);
    Derivatives(xgot, x, u, *dx_got);
    Derivatives(uref, x, u, *du_ref);
    Derivatives(ugot, x, u, *du_got);
    if (!gn) {
      EXPECT_TRUE(BitEqual(*dx_ref, *dx_got));
      EXPECT_TRUE(BitEqual(*du_ref, *du_got));
      // Leave stale non-structural entries behind for the structured call.
      dx_got->grad_.setConstant(7.0);
      dx_got->hess_.setConstant(7.0);
      du_got->grad_.setConstant(7.0);
      du_got->hess_.setConstant(7.0);
    } else {
      ExpectAgreement(Compare(*dx_ref, *dx_got), "state, after a fallback");
      ExpectAgreement(Compare(*du_ref, *du_got), "input, after a fallback");
    }
  }
}

// The OCP builds the structured costs by default (and they find the
// structure), Aligator's with kDense; a running stage's cost and the terminal
// cost of the two builds agree to rounding.
TEST_F(StructuredQuadraticCostsG1, TheOcpBuildsEitherAndBothAgree) {
  KinodynamicOCP structured(cfg_);
  KinodynamicOCP dense(cfg_, KinodynamicOCP::StageKinematics::kShared,
                       KinodynamicOCP::QuadraticCostDerivatives::kDense);
  EXPECT_EQ(structured.quadratic_cost_derivatives(),
            KinodynamicOCP::QuadraticCostDerivatives::kStructured);
  EXPECT_EQ(dense.quadratic_cost_derivatives(), KinodynamicOCP::QuadraticCostDerivatives::kDense);
  ASSERT_EQ(structured.Build(Context(), handler_, {}), OCPBuildError::kNoError);
  ASSERT_EQ(dense.Build(Context(), handler_, {}), OCPBuildError::kNoError);

  for (std::size_t k = 0; k < static_cast<std::size_t>(tu::kKinoNodes); ++k) {
    auto* s_stack = structured.problem().stages_[k]->getCost<CostStack>();
    auto* d_stack = dense.problem().stages_[k]->getCost<CostStack>();
    ASSERT_NE(s_stack, nullptr);
    ASSERT_NE(d_stack, nullptr);
    const auto* sx =
        s_stack->getComponent<StructuredQuadStateCost>(std::string(KinodynamicOCP::kStateCostKey));
    const auto* su = s_stack->getComponent<StructuredQuadControlCost>(
        std::string(KinodynamicOCP::kInputCostKey));
    ASSERT_NE(sx, nullptr);
    ASSERT_NE(su, nullptr);
    EXPECT_TRUE(sx->structured());
    EXPECT_TRUE(su->structured());
    const auto* dx =
        d_stack->getComponent<QuadStateCost>(std::string(KinodynamicOCP::kStateCostKey));
    const auto* du =
        d_stack->getComponent<QuadControlCost>(std::string(KinodynamicOCP::kInputCostKey));
    ASSERT_NE(dx, nullptr);
    ASSERT_NE(du, nullptr);
    EXPECT_TRUE(typeid(*dx) == typeid(QuadStateCost));
    EXPECT_TRUE(typeid(*du) == typeid(QuadControlCost));
  }
  const auto* s_term =
      dynamic_cast<const StructuredQuadStateCost*>(&*structured.problem().term_cost_);
  ASSERT_NE(s_term, nullptr);
  EXPECT_TRUE(s_term->structured());
  EXPECT_TRUE(typeid(*dense.problem().term_cost_) == typeid(QuadStateCost));

  // A whole running stage (every cost term) and the terminal cost: the sum of
  // the terms rounds differently, so the bound is that of the sum.
  constexpr double kStageTol = 1e-10;
  std::mt19937 rng(kSeed);
  const Eigen::VectorXd stand = tu::G1Stand(model_);
  for (const std::size_t k : {std::size_t{0}, std::size_t{17}}) {
    auto& s_stage = *structured.problem().stages_[k];
    auto& d_stage = *dense.problem().stages_[k];
    const auto s_data = s_stage.createData();
    const auto d_data = d_stage.createData();
    Agreement worst;
    for (int draw = 0; draw < 20; ++draw) {
      Eigen::VectorXd dx = 0.05 * RandomVector(space_->ndx(), 1.0, rng);
      Eigen::VectorXd x0(space_->nx());
      x0 << stand, Eigen::VectorXd::Zero(model_.nv);
      Eigen::VectorXd x(space_->nx());
      space_->integrate(x0, dx, x);
      Eigen::VectorXd u(nu_);
      ASSERT_TRUE(structured.ReferenceControl(static_cast<int>(k), u));
      u += RandomVector(nu_, 5.0, rng);
      s_stage.evaluate(x, u, *s_data);
      s_stage.computeFirstOrderDerivatives(x, u, *s_data);
      s_stage.computeSecondOrderDerivatives(x, u, *s_data);
      d_stage.evaluate(x, u, *d_data);
      d_stage.computeFirstOrderDerivatives(x, u, *d_data);
      d_stage.computeSecondOrderDerivatives(x, u, *d_data);
      worst.Merge(Compare(*d_data->cost_data, *s_data->cost_data));
    }
    std::printf(
        "[report] stage %zu cost: max relative delta value %.3g, gradient %.3g, "
        "Hessian %.3g\n",
        k, worst.value, worst.grad, worst.hess);
    EXPECT_LE(worst.value, kStageTol) << "stage " << k;
    EXPECT_LE(worst.grad, kStageTol) << "stage " << k;
    EXPECT_LE(worst.hess, kStageTol) << "stage " << k;
  }
  const auto s_term_data = structured.problem().term_cost_->createData();
  const auto d_term_data = dense.problem().term_cost_->createData();
  Agreement term;
  for (int draw = 0; draw < 20; ++draw) {
    const Eigen::VectorXd x = RandomState(*space_, rng);
    const Eigen::VectorXd u = Eigen::VectorXd::Zero(nu_);
    Derivatives(*structured.problem().term_cost_, x, u, *s_term_data);
    Derivatives(*dense.problem().term_cost_, x, u, *d_term_data);
    term.Merge(Compare(*d_term_data, *s_term_data));
  }
  ExpectAgreement(term, "OCP terminal cost");
}

// ── Models without the structure ─────────────────────────────────────────────

/// A three-body chain: @p root on the world (or none for a fixed base), then
/// @p middle, then a revolute joint.
template <typename Root, typename Middle>
[[nodiscard]] pinocchio::Model Chain(const Root* root, const Middle& middle) {
  pinocchio::Model m;
  pinocchio::JointIndex parent = 0;
  const pinocchio::Inertia body(1.0, Eigen::Vector3d(0.0, 0.0, 0.1),
                                0.1 * Eigen::Matrix3d::Identity());
  const pinocchio::SE3 offset(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.3));
  if (root != nullptr) {
    parent = m.addJoint(0, *root, pinocchio::SE3::Identity(), "root");
    m.appendBodyToJoint(parent, body, pinocchio::SE3::Identity());
  }
  parent = m.addJoint(parent, middle, offset, "middle");
  m.appendBodyToJoint(parent, body, pinocchio::SE3::Identity());
  parent = m.addJoint(parent, pinocchio::JointModelRY(), offset, "last");
  m.appendBodyToJoint(parent, body, pinocchio::SE3::Identity());
  return m;
}

/// 50 random draws on @p model: the structured costs report @p expect_structured;
/// without the structure their results are Aligator's bits, with it they agree.
void ExpectFallback(const pinocchio::Model& model, bool expect_structured) {
  EXPECT_EQ(HasFreeFlyerRootAndEuclideanJoints(model), expect_structured);
  const Space space(model);
  const int nu = 4;
  const Eigen::MatrixXd wx = Eigen::VectorXd::LinSpaced(space.ndx(), 0.5, 3.0).asDiagonal();
  const Eigen::MatrixXd wu = Eigen::VectorXd::LinSpaced(nu, 0.5, 2.0).asDiagonal();
  QuadStateCost xref(space, nu, space.neutral(), wx);
  StructuredQuadStateCost xgot(space, nu, space.neutral(), wx);
  EXPECT_EQ(xgot.structured(), expect_structured);
  const auto d_ref = xref.createData();
  const auto d_got = xgot.createData();
  std::mt19937 rng(kSeed);
  Agreement worst;
  for (int k = 0; k < 50; ++k) {
    const Eigen::VectorXd x = RandomState(space, rng);
    const Eigen::VectorXd x_ref = RandomState(space, rng);
    const Eigen::VectorXd u = RandomVector(nu, 5.0, rng);
    xref.setTarget(x_ref);
    xgot.setTarget(x_ref);
    Derivatives(xref, x, u, *d_ref);
    Derivatives(xgot, x, u, *d_got);
    if (!expect_structured) {
      ASSERT_TRUE(BitEqual(*d_ref, *d_got)) << "draw " << k;
    }
    worst.Merge(Compare(*d_ref, *d_got));
  }
  ExpectAgreement(worst, expect_structured ? "small model with the structure"
                                           : "small model without the structure");
  // The control cost does not depend on the state manifold's joints.
  StructuredQuadControlCost ugot(space, Eigen::VectorXd::Zero(nu), wu);
  EXPECT_TRUE(ugot.structured());
}

TEST(StructuredQuadraticCostsFallback, AFreeFlyerOverRevoluteJointsHasTheStructure) {
  const pinocchio::JointModelFreeFlyer ff;
  ExpectFallback(Chain(&ff, pinocchio::JointModelRX()), true);
  ExpectFallback(Chain(&ff, pinocchio::JointModelPZ()), true);
}

// A free flyer over one joint of each type the structure accepts.
TEST(StructuredQuadraticCostsFallback, EveryAcceptedJointTypeHasTheStructure) {
  pinocchio::Model m;
  const pinocchio::Inertia body(1.0, Eigen::Vector3d(0.0, 0.0, 0.1),
                                0.1 * Eigen::Matrix3d::Identity());
  const pinocchio::SE3 offset(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.3));
  pinocchio::JointIndex parent =
      m.addJoint(0, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), "root");
  m.appendBodyToJoint(parent, body, pinocchio::SE3::Identity());
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, 2.0, -0.5).normalized();
  const pinocchio::JointModel joints[] = {
      pinocchio::JointModelRX(),          pinocchio::JointModelRY(),
      pinocchio::JointModelRZ(),          pinocchio::JointModelRevoluteUnaligned(axis),
      pinocchio::JointModelPX(),          pinocchio::JointModelPY(),
      pinocchio::JointModelPZ(),          pinocchio::JointModelPrismaticUnaligned(axis),
      pinocchio::JointModelTranslation(),
  };
  int n = 0;
  for (const pinocchio::JointModel& joint : joints) {
    parent = m.addJoint(parent, joint, offset, "joint_" + std::to_string(n++));
    m.appendBodyToJoint(parent, body, pinocchio::SE3::Identity());
  }
  ASSERT_EQ(m.njoints, 11);
  ExpectFallback(m, true);
}

TEST(StructuredQuadraticCostsFallback, ASphericalJointFallsBack) {
  const pinocchio::JointModelFreeFlyer ff;
  ExpectFallback(Chain(&ff, pinocchio::JointModelSpherical()), false);
}

TEST(StructuredQuadraticCostsFallback, AnUnboundedRevoluteJointFallsBack) {
  const pinocchio::JointModelFreeFlyer ff;
  ExpectFallback(Chain(&ff, pinocchio::JointModelRUBZ()), false);
}

TEST(StructuredQuadraticCostsFallback, AFixedBaseFallsBack) {
  ExpectFallback(
      Chain(static_cast<const pinocchio::JointModelFreeFlyer*>(nullptr), pinocchio::JointModelRX()),
      false);
}

TEST(StructuredQuadraticCostsFallback, APlanarRootFallsBack) {
  const pinocchio::JointModelPlanar planar;
  ExpectFallback(Chain(&planar, pinocchio::JointModelRX()), false);
}

}  // namespace
