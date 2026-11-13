/// @file test_kinodynamic_integrator.cpp
/// @brief The two stage discretisations of the kinodynamic ODE on the public
///        G1: convergence order against a fine reference, exactness of the
///        joint blocks, an in-place mode switch under the solver (no setup in
///        between), configure-time refusals and quaternion norm over a long
///        rollout; plus a rollout-error report (not a gate).
///
/// Pre-registered order gate: 20 states (seed 20261114, both feet in
/// contact), T = 8 nodes of 0.035 s, `u` constant per node. Reference: RK4
/// on the manifold with 1024 sub-steps per node, checked against 2048 (max
/// tangent difference ≤ 1e-6). The dt → dt/2 error ratio's median over the
/// 20 states, for base position, attitude, linear and angular velocity, lies
/// in [1.6, 2.4] for semi-implicit Euler and [3.2, 4.8] for RK2. The joint
/// blocks are exact up to the integrator's own order: joint velocity error
/// ≤ 1e-12 (both), RK2 joint position ≤ 1e-12, semi-implicit Euler joint
/// position ratio in [1.99, 2.01] for every state.

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/kinodynamic_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Geometry>
#include <aligator/core/stage-data.hpp>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-data.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace {

using rtc::mpc::KinodynamicDynamics;
using rtc::mpc::KinodynamicError;
using rtc::mpc::KinodynamicIntegrator;
using rtc::mpc::test_utils::KinoOracle;
using rtc::mpc::test_utils::KinoSample;
using Ode = KinodynamicDynamics::Ode;
using OdeData = KinodynamicDynamics::OdeData;
using Space = KinodynamicDynamics::Space;
using Vec = Eigen::VectorXd;

constexpr std::uint32_t kGateSeed = 20261114U;
constexpr double kDt = 0.035;
constexpr int kOrderNodes = 8;
constexpr int kStates = 20;

double Median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

double Max(const std::vector<double>& v) {
  return *std::max_element(v.begin(), v.end());
}

/// One node of RK4 on the manifold, @p nsub sub-steps, @p u held.
Vec Rk4(const Space& space, const Ode& ode, OdeData& d, const Vec& x0, const Vec& u, double dt,
        int nsub) {
  Vec x = x0;
  Vec xt(x0.size());
  const double h = dt / nsub;
  for (int s = 0; s < nsub; ++s) {
    ode.forward(x, u, d);
    const Vec k1 = d.xdot_;
    space.integrate(x, 0.5 * h * k1, xt);
    ode.forward(xt, u, d);
    const Vec k2 = d.xdot_;
    space.integrate(x, 0.5 * h * k2, xt);
    ode.forward(xt, u, d);
    const Vec k3 = d.xdot_;
    space.integrate(x, h * k3, xt);
    ode.forward(xt, u, d);
    const Vec k4 = d.xdot_;
    space.integrate(x, (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4), xt);
    x = xt;
  }
  return x;
}

/// One RK4 step in the embedding vector space: ṗ = R v_lin, q̇ = ½ q ⊗ ω,
/// q̇_j = v_j, v̇ from the ODE (its quaternion normalised before the call);
/// the output quaternion is normalised.
Vec VectorRk4(const Ode& ode, OdeData& d, const Vec& x0, const Vec& u, double dt, int nq, int nv) {
  auto f = [&](const Vec& y) {
    Vec yn = y;
    yn.segment<4>(3).normalize();
    ode.forward(yn, u, d);
    const Eigen::Quaterniond quat(yn[6], yn[3], yn[4], yn[5]);
    Vec dy(nq + nv);
    dy.head<3>() = quat.toRotationMatrix() * y.segment<3>(nq);
    const Eigen::Quaterniond w(0.0, y[nq + 3], y[nq + 4], y[nq + 5]);
    const Eigen::Quaterniond qd = Eigen::Quaterniond(quat.coeffs() * 0.5) * w;
    dy.segment<4>(3) = qd.coeffs();
    dy.segment(7, nq - 7) = y.segment(nq + 6, nv - 6);
    dy.tail(nv) = d.xdot_.tail(nv);
    return dy;
  };
  const Vec k1 = f(x0);
  const Vec k2 = f(x0 + 0.5 * dt * k1);
  const Vec k3 = f(x0 + 0.5 * dt * k2);
  const Vec k4 = f(x0 + dt * k3);
  Vec x = x0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  x.segment<4>(3).normalize();
  return x;
}

/// Errors of @p x against @p ref, per block.
struct BlockErr {
  double p, R, qj, vl, vw, vj;
};

BlockErr Blocks(const Space& space, const Vec& ref, const Vec& x, int nv) {
  Vec dx(2 * nv);
  space.difference(ref, x, dx);
  const Eigen::Quaterniond qa(ref[6], ref[3], ref[4], ref[5]);
  const Eigen::Quaterniond qb(x[6], x[3], x[4], x[5]);
  BlockErr e{};
  e.p = (x.head<3>() - ref.head<3>()).norm();
  e.R = qa.angularDistance(qb);
  e.qj = dx.segment(6, nv - 6).cwiseAbs().maxCoeff();
  e.vl = dx.segment<3>(nv).norm();
  e.vw = dx.segment<3>(nv + 3).norm();
  e.vj = dx.tail(nv - 6).cwiseAbs().maxCoeff();
  return e;
}

class KinodynamicIntegratorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(rtc::mpc::test_utils::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << rtc::mpc::test_utils::kG1Urdf;
    }
    model_ = rtc::mpc::test_utils::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(rtc::mpc::test_utils::kG1SolesYaml)),
              rtc::mpc::RobotModelInitError::kNoError);
    ASSERT_EQ(dyn_.Init(handler_), KinodynamicError::kNoError);
    oracle_.emplace(model_, rtc::mpc::test_utils::G1SoleFrames(model_), 6);
  }

  /// Roll the stage dynamics of @p integrator over `us[k / hold]`.
  Vec Roll(KinodynamicIntegrator integrator, double h, int steps, int hold, const Vec& x0,
           const std::vector<Vec>& us) const {
    std::optional<KinodynamicDynamics::StageDynamics> dyn;
    EXPECT_EQ(dyn_.MakeStageDynamics(0b11, h, integrator, dyn), KinodynamicError::kNoError);
    auto data = (*dyn)->createData();
    Vec x = x0;
    for (int k = 0; k < steps; ++k) {
      (*dyn)->forward(x, us[static_cast<std::size_t>(k / hold)], *data);
      x = data->xnext_;
    }
    return x;
  }

  /// The reference over @p nodes nodes of kDt, @p nsub RK4 sub-steps each.
  Vec Reference(const Vec& x0, const std::vector<Vec>& us, int nodes, int nsub) const {
    const Ode ode = dyn_.MakeOde(0b11).value();
    auto data = ode.createData();
    Vec x = x0;
    for (int k = 0; k < nodes; ++k) {
      x = Rk4(*dyn_.space(), ode, static_cast<OdeData&>(*data), x, us[static_cast<std::size_t>(k)],
              kDt, nsub);
    }
    return x;
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  KinodynamicDynamics dyn_;
  std::optional<KinoOracle> oracle_;
};

TEST_F(KinodynamicIntegratorTest, ConvergenceOrderAndExactJointBlocks) {
  const Space& space = *dyn_.space();
  const int nv = model_.nv;
  std::mt19937 rng(kGateSeed);
  std::array<std::vector<double>, 4> ratio_sie;
  std::array<std::vector<double>, 4> ratio_rk2;
  double ref_conv = 0.0;
  for (int i = 0; i < kStates; ++i) {
    const KinoSample s0 = oracle_->Draw(rng, 0b11);
    std::vector<Vec> us;
    for (int k = 0; k < kOrderNodes; ++k) {
      us.push_back(oracle_->Draw(rng, 0b11).u);
    }
    const Vec ref = Reference(s0.x, us, kOrderNodes, 1024);
    Vec dref(2 * nv);
    space.difference(ref, Reference(s0.x, us, kOrderNodes, 2048), dref);
    ref_conv = std::max(ref_conv, dref.cwiseAbs().maxCoeff());

    const BlockErr e1 =
        Blocks(space, ref,
               Roll(KinodynamicIntegrator::kSemiImplicitEuler, kDt, kOrderNodes, 1, s0.x, us), nv);
    const BlockErr e2 = Blocks(
        space, ref,
        Roll(KinodynamicIntegrator::kSemiImplicitEuler, kDt / 2, 2 * kOrderNodes, 2, s0.x, us), nv);
    const BlockErr f1 =
        Blocks(space, ref, Roll(KinodynamicIntegrator::kRk2, kDt, kOrderNodes, 1, s0.x, us), nv);
    const BlockErr f2 = Blocks(
        space, ref, Roll(KinodynamicIntegrator::kRk2, kDt / 2, 2 * kOrderNodes, 2, s0.x, us), nv);
    const std::array<double, 4> a1{e1.p, e1.R, e1.vl, e1.vw};
    const std::array<double, 4> a2{e2.p, e2.R, e2.vl, e2.vw};
    const std::array<double, 4> b1{f1.p, f1.R, f1.vl, f1.vw};
    const std::array<double, 4> b2{f2.p, f2.R, f2.vl, f2.vw};
    for (std::size_t b = 0; b < 4; ++b) {
      ratio_sie[b].push_back(a1[b] / a2[b]);
      ratio_rk2[b].push_back(b1[b] / b2[b]);
    }
    // Joint blocks: constant q̈_j per node is integrated exactly up to order.
    EXPECT_LE(std::max({e1.vj, e2.vj, f1.vj, f2.vj}), 1e-12) << "state " << i;
    EXPECT_LE(std::max(f1.qj, f2.qj), 1e-12) << "state " << i;
    const double r = e1.qj / e2.qj;
    EXPECT_GE(r, 1.99) << "state " << i;
    EXPECT_LE(r, 2.01) << "state " << i;
  }
  EXPECT_LE(ref_conv, 1e-6) << "the reference has not converged";
  const std::array<const char*, 4> names{"base p", "base R", "v lin", "v ang"};
  for (std::size_t b = 0; b < 4; ++b) {
    const double ms = Median(ratio_sie[b]);
    const double mr = Median(ratio_rk2[b]);
    EXPECT_GE(ms, 1.6) << names[b];
    EXPECT_LE(ms, 2.4) << names[b];
    EXPECT_GE(mr, 3.2) << names[b];
    EXPECT_LE(mr, 4.8) << names[b];
    std::printf("order %-7s semi-implicit Euler median %.3f | RK2 median %.3f\n", names[b], ms, mr);
  }
  std::printf("reference 1024 vs 2048 sub-steps: max %.3e\n", ref_conv);
}

// Modes written into a solved problem's stages take effect on the next run
// without a solver setup, and the solver's own data then evaluates the new
// modes exactly as a problem built with them from the start.
TEST_F(KinodynamicIntegratorTest, SolverFollowsModesSwitchedInPlace) {
  using Stage = aligator::StageModelTpl<double>;
  using Problem = aligator::TrajOptProblemTpl<double>;
  const Space& space = *dyn_.space();
  const int nu = dyn_.nu();
  const int ndx = space.ndx();
  aligator::CostStackTpl<double> cost(space, nu);
  cost.addCost("x", aligator::QuadraticStateCostTpl<double>(space, nu, space.neutral(),
                                                            Eigen::MatrixXd::Identity(ndx, ndx)));
  cost.addCost("u", aligator::QuadraticControlCostTpl<double>(
                        space, nu, 1e-4 * Eigen::MatrixXd::Identity(nu, nu)));
  auto make_problem = [&](const std::array<std::uint32_t, 4>& modes) {
    std::vector<xyz::polymorphic<Stage>> stages;
    for (const std::uint32_t m : modes) {
      std::optional<KinodynamicDynamics::StageDynamics> d;
      EXPECT_EQ(dyn_.MakeStageDynamics(m, kDt, KinodynamicIntegrator::kSemiImplicitEuler, d),
                KinodynamicError::kNoError);
      stages.emplace_back(Stage(cost, *d));
    }
    return Problem(space.neutral(), stages, aligator::CostStackTpl<double>(space, nu));
  };
  Problem problem = make_problem({3U, 3U, 3U, 3U});
  aligator::SolverProxDDPTpl<double> solver(1e-6, 1e-2, 3);
  solver.setup(problem);
  ASSERT_NO_THROW((void)solver.run(problem));

  const std::array<std::uint32_t, 4> modes{1U, 2U, 0U, 3U};
  for (std::size_t k = 0; k < modes.size(); ++k) {
    Ode* ode = KinodynamicDynamics::OdeOf(*problem.stages_[k]);
    ASSERT_NE(ode, nullptr);
    ASSERT_TRUE(dyn_.SetMode(*ode, modes[k]));
  }
  ASSERT_NO_THROW((void)solver.run(problem, solver.results_.xs, solver.results_.us));

  const auto& xs = solver.results_.xs;
  const auto& us = solver.results_.us;
  ASSERT_TRUE(std::all_of(xs.begin(), xs.end(), [](const Vec& x) { return x.allFinite(); }));
  (void)problem.evaluate(xs, us, solver.workspace_.problem_data);
  Problem fresh = make_problem(modes);
  aligator::TrajOptDataTpl<double> fresh_data(fresh);
  (void)fresh.evaluate(xs, us, fresh_data);
  for (std::size_t k = 0; k < modes.size(); ++k) {
    const Vec& got = solver.workspace_.problem_data.stage_data[k]->dynamics_data->xnext_;
    const Vec& want = fresh_data.stage_data[k]->dynamics_data->xnext_;
    ASSERT_EQ(got.size(), want.size());
    EXPECT_EQ(
        std::memcmp(got.data(), want.data(), sizeof(double) * static_cast<std::size_t>(got.size())),
        0)
        << "stage " << k;
  }
}

TEST_F(KinodynamicIntegratorTest, RefusesBadStepsModesAndIntegrators) {
  std::optional<KinodynamicDynamics::StageDynamics> out;
  for (const double dt : {std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity(), 0.0, -kDt}) {
    EXPECT_EQ(dyn_.MakeStageDynamics(0, dt, KinodynamicIntegrator::kRk2, out),
              KinodynamicError::kInvalidDt)
        << dt;
    EXPECT_FALSE(out.has_value());
  }
  // A failure empties a previously filled result.
  ASSERT_EQ(dyn_.MakeStageDynamics(0, kDt, KinodynamicIntegrator::kRk2, out),
            KinodynamicError::kNoError);
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(dyn_.MakeStageDynamics(4, kDt, KinodynamicIntegrator::kRk2, out),
            KinodynamicError::kInvalidMode);
  EXPECT_FALSE(out.has_value());
  EXPECT_EQ(dyn_.MakeStageDynamics(0, kDt, static_cast<KinodynamicIntegrator>(7), out),
            KinodynamicError::kUnknownIntegrator);
  EXPECT_FALSE(out.has_value());
  KinodynamicDynamics uninit;
  EXPECT_EQ(uninit.MakeStageDynamics(0, kDt, KinodynamicIntegrator::kRk2, out),
            KinodynamicError::kNotInitialised);
  EXPECT_EQ(uninit.space(), nullptr);

  KinodynamicIntegrator parsed = KinodynamicIntegrator::kRk2;
  EXPECT_EQ(rtc::mpc::ParseKinodynamicIntegrator("semi_implicit_euler", parsed),
            KinodynamicError::kNoError);
  EXPECT_EQ(parsed, KinodynamicIntegrator::kSemiImplicitEuler);
  EXPECT_EQ(rtc::mpc::ParseKinodynamicIntegrator("rk2", parsed), KinodynamicError::kNoError);
  EXPECT_EQ(parsed, KinodynamicIntegrator::kRk2);
  for (const char* bad : {"rk4", "RK2", "", "euler", "semi_implicit_euler "}) {
    EXPECT_EQ(rtc::mpc::ParseKinodynamicIntegrator(bad, parsed),
              KinodynamicError::kUnknownIntegrator)
        << "'" << bad << "'";
    EXPECT_EQ(parsed, KinodynamicIntegrator::kRk2);
  }
  ASSERT_EQ(dyn_.MakeStageDynamics(0b01, kDt, KinodynamicIntegrator::kSemiImplicitEuler, out),
            KinodynamicError::kNoError);
  ASSERT_TRUE(out.has_value());
  // The stage built from it carries that mode, and OdeOf finds it.
  aligator::StageModelTpl<double> stage(aligator::CostStackTpl<double>(*dyn_.space(), dyn_.nu()),
                                        *out);
  const Ode* in_stage = KinodynamicDynamics::OdeOf(stage);
  ASSERT_NE(in_stage, nullptr);
  EXPECT_TRUE(in_stage->contact_states_[0]);
  EXPECT_FALSE(in_stage->contact_states_[1]);
}

// pinocchio's integrate renormalises at every step: a long rollout keeps the
// base quaternion unit (a sanity check, not an order claim).
TEST_F(KinodynamicIntegratorTest, BaseQuaternionStaysUnitOverALongRollout) {
  std::mt19937 rng(kGateSeed);
  const KinoSample s0 = oracle_->Draw(rng, 0b11);
  std::vector<Vec> us;
  for (int k = 0; k < 64; ++k) {
    us.push_back(oracle_->Draw(rng, 0b11).u);
  }
  for (const auto integrator :
       {KinodynamicIntegrator::kSemiImplicitEuler, KinodynamicIntegrator::kRk2}) {
    const Vec x = Roll(integrator, kDt, 64, 1, s0.x, us);
    EXPECT_LE(std::abs(x.segment<4>(3).norm() - 1.0), 1e-12);
  }
}

// Report only: rollout error against the reference after 1, 8 and 32 nodes
// of one rollout per state, near weight compensation, for semi-implicit
// Euler, RK2 and a vector-space RK4 (one step per node).
TEST_F(KinodynamicIntegratorTest, RolloutErrorReport) {
  const Space& space = *dyn_.space();
  const int nv = model_.nv;
  const int nq = model_.nq;
  std::mt19937 rng(kGateSeed + 1U);
  std::uniform_real_distribution<double> sym(-1.0, 1.0);
  const Ode ode = dyn_.MakeOde(0b11).value();
  auto data = ode.createData();
  auto& d = static_cast<OdeData&>(*data);
  const std::array<const char*, 3> who{"semi_implicit_euler", "rk2", "vector_rk4"};
  const std::array<const char*, 6> blk{"p[m]", "R[rad]", "q_j[rad]", "v_lin", "v_ang", "v_j"};
  constexpr std::array<int, 3> kCheckpoints{1, 8, 32};
  std::array<std::array<std::array<std::vector<double>, 6>, 3>, 3> tab;  // [checkpoint][who][block]
  std::array<std::optional<KinodynamicDynamics::StageDynamics>, 2> stage;
  ASSERT_EQ(dyn_.MakeStageDynamics(0b11, kDt, KinodynamicIntegrator::kSemiImplicitEuler, stage[0]),
            KinodynamicError::kNoError);
  ASSERT_EQ(dyn_.MakeStageDynamics(0b11, kDt, KinodynamicIntegrator::kRk2, stage[1]),
            KinodynamicError::kNoError);
  auto sd0 = (*stage[0])->createData();
  auto sd1 = (*stage[1])->createData();
  for (int i = 0; i < kStates; ++i) {
    Vec x0(nq + nv);
    x0 << oracle_->DrawQ(rng), Eigen::VectorXd::Zero(nv);
    for (int j = 0; j < nv; ++j) {
      x0[nq + j] = 0.1 * sym(rng);
    }
    // Inputs near weight compensation of the initial pose, one per node.
    std::array<Vec, 4> x{x0, x0, x0, x0};  // reference, SIE, RK2, vector RK4
    std::size_t next = 0;
    for (int k = 0; k < kCheckpoints.back(); ++k) {
      Vec u = oracle_->WeightCompensation(x0.head(nq), 0b11);
      for (int j = 0; j < 12; ++j) {
        u[j] += 0.05 * oracle_->mass() * 9.81 * sym(rng) * (j % 6 < 3 ? 1.0 : 0.02);
      }
      for (int j = 12; j < u.size(); ++j) {
        u[j] = sym(rng);
      }
      x[0] = Rk4(space, ode, d, x[0], u, kDt, 1024);
      (*stage[0])->forward(x[1], u, *sd0);
      x[1] = sd0->xnext_;
      (*stage[1])->forward(x[2], u, *sd1);
      x[2] = sd1->xnext_;
      x[3] = VectorRk4(ode, d, x[3], u, kDt, nq, nv);
      if (k + 1 == kCheckpoints[next]) {
        for (std::size_t w = 0; w < 3; ++w) {
          const BlockErr e = Blocks(space, x[0], x[w + 1], nv);
          const std::array<double, 6> a{e.p, e.R, e.qj, e.vl, e.vw, e.vj};
          for (std::size_t b = 0; b < 6; ++b) {
            tab[next][w][b].push_back(a[b]);
          }
        }
        ++next;
      }
    }
  }
  for (std::size_t c = 0; c < kCheckpoints.size(); ++c) {
    for (std::size_t w = 0; w < 3; ++w) {
      std::printf("rollout %2d nodes %-19s", kCheckpoints[c], who[w]);
      for (std::size_t b = 0; b < 6; ++b) {
        std::printf(" | %s max %.2e med %.2e", blk[b], Max(tab[c][w][b]), Median(tab[c][w][b]));
        RecordProperty(std::string("rollout_") + std::to_string(kCheckpoints[c]) + "_" + who[w] +
                           "_" + blk[b] + "_max",
                       std::to_string(Max(tab[c][w][b])));
      }
      std::printf("\n");
    }
  }
}

}  // namespace
