/// @file test_kinodynamic_derivatives.cpp
/// @brief The kinodynamic Jacobians against central finite differences on the
///        public G1, for the continuous ODE and for both stage
///        discretisations. Pre-registered: the G1 gate set (seed 20261114,
///        250 samples), step h = 1e-6 on the state tangent and the input,
///        finite differences on a data object of their own, and `dForward`
///        always right after `forward` on the data it reads.
///
/// Tolerances, element-wise: continuous `|J − J_fd| ≤ 1e-6 + 1e-5·|J_fd|`;
/// discrete (dt 0.035, output as the tangent difference at the nominal next
/// state) `|J − J_fd| ≤ 1e-7 + 1e-5·|J_fd|`.

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/jacobian_compare.hpp"
#include "test_utils/kinodynamic_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace {

using rtc::mpc::KinodynamicDynamics;
using rtc::mpc::KinodynamicError;
using rtc::mpc::KinodynamicIntegrator;
using rtc::mpc::test_utils::KinoOracle;
using rtc::mpc::test_utils::KinoSample;
using Ode = KinodynamicDynamics::Ode;

constexpr std::uint32_t kGateSeed = 20261114U;
constexpr double kH = 1e-6;
constexpr double kDt = 0.035;

using rtc::mpc::test_utils::ColumnErr;

using rtc::mpc::test_utils::CompareColumns;

class KinodynamicDerivativesTest : public ::testing::Test {
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

  /// Column groups of `[Jx | Ju]`: q · v (state) and force · moment · q̈_j
  /// (input). Index 0–1 into Jx, 2–4 into Ju.
  struct Groups {
    std::array<ColumnErr, 5> e{};

    void Fold(int g, const ColumnErr& c) {
      e[static_cast<std::size_t>(g)].ratio =
          std::max(e[static_cast<std::size_t>(g)].ratio, c.ratio);
      e[static_cast<std::size_t>(g)].abs = std::max(e[static_cast<std::size_t>(g)].abs, c.abs);
    }
  };

  void FoldGroups(Groups& g, const Eigen::MatrixXd& jx, const Eigen::MatrixXd& jx_fd,
                  const Eigen::MatrixXd& ju, const Eigen::MatrixXd& ju_fd, double abs_tol,
                  double rel_tol) {
    const int nv = model_.nv;
    const int fs = dyn_.force_size();
    g.Fold(0, CompareColumns(jx, jx_fd, 0, nv, abs_tol, rel_tol));
    g.Fold(1, CompareColumns(jx, jx_fd, nv, nv, abs_tol, rel_tol));
    for (int c = 0; c < dyn_.n_contacts(); ++c) {
      g.Fold(2, CompareColumns(ju, ju_fd, c * fs, 3, abs_tol, rel_tol));
      g.Fold(3, CompareColumns(ju, ju_fd, c * fs + 3, fs - 3, abs_tol, rel_tol));
    }
    g.Fold(4, CompareColumns(ju, ju_fd, dyn_.n_contacts() * fs, model_.nv - 6, abs_tol, rel_tol));
  }

  static void Report(const char* what, const Groups& g) {
    const std::array<const char*, 5> names{"q", "v", "force", "moment", "qdd_j"};
    for (std::size_t i = 0; i < names.size(); ++i) {
      std::printf("%s %-7s worst err/tol %.3e  abs %.3e\n", what, names[i], g.e[i].ratio,
                  g.e[i].abs);
    }
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  KinodynamicDynamics dyn_;
  std::optional<KinoOracle> oracle_;
};

TEST_F(KinodynamicDerivativesTest, ContinuousJacobiansMatchCentralDifferences) {
  const KinodynamicDynamics::Space& space = *dyn_.space();
  const int ndx = space.ndx();
  const int nu = dyn_.nu();
  int pass = 0;
  Groups groups;
  const std::vector<KinoSample> samples = rtc::mpc::test_utils::G1GateSamples(*oracle_, kGateSeed);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const KinoSample& s = samples[i];
    const Ode ode = dyn_.MakeOde(s.mode).value();
    auto data = ode.createData();
    ode.forward(s.x, s.u, *data);
    ode.dForward(s.x, s.u, *data);

    auto fd = ode.createData();
    Eigen::MatrixXd jx_fd(ndx, ndx);
    Eigen::MatrixXd ju_fd(ndx, nu);
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(ndx);
    Eigen::VectorXd xp(s.x.size());
    Eigen::VectorXd xm(s.x.size());
    for (int j = 0; j < ndx; ++j) {
      dx[j] = kH;
      space.integrate(s.x, dx, xp);
      dx[j] = -kH;
      space.integrate(s.x, dx, xm);
      dx[j] = 0.0;
      ode.forward(xp, s.u, *fd);
      const Eigen::VectorXd fp = fd->xdot_;
      ode.forward(xm, s.u, *fd);
      jx_fd.col(j) = (fp - fd->xdot_) / (2.0 * kH);
    }
    for (int j = 0; j < nu; ++j) {
      Eigen::VectorXd up = s.u;
      Eigen::VectorXd um = s.u;
      up[j] += kH;
      um[j] -= kH;
      ode.forward(s.x, up, *fd);
      const Eigen::VectorXd fp = fd->xdot_;
      ode.forward(s.x, um, *fd);
      ju_fd.col(j) = (fp - fd->xdot_) / (2.0 * kH);
    }
    const ColumnErr ex = CompareColumns(data->Jx_, jx_fd, 0, ndx, 1e-6, 1e-5);
    const ColumnErr eu = CompareColumns(data->Ju_, ju_fd, 0, nu, 1e-6, 1e-5);
    FoldGroups(groups, data->Jx_, jx_fd, data->Ju_, ju_fd, 1e-6, 1e-5);
    const bool ok = ex.ratio <= 1.0 && eu.ratio <= 1.0;
    pass += ok ? 1 : 0;
    EXPECT_TRUE(ok) << "sample " << i << " mode " << s.mode << " Jx " << ex.ratio << " Ju "
                    << eu.ratio;
  }
  EXPECT_EQ(pass, static_cast<int>(samples.size()));
  Report("continuous", groups);
}

TEST_F(KinodynamicDerivativesTest, DiscreteJacobiansMatchCentralDifferences) {
  const KinodynamicDynamics::Space& space = *dyn_.space();
  const int ndx = space.ndx();
  const int nu = dyn_.nu();
  const std::vector<KinoSample> samples = rtc::mpc::test_utils::G1GateSamples(*oracle_, kGateSeed);
  for (const auto& [integrator, name] :
       {std::pair{KinodynamicIntegrator::kSemiImplicitEuler, "semi_implicit_euler"},
        std::pair{KinodynamicIntegrator::kRk2, "rk2"}}) {
    int pass = 0;
    Groups groups;
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const KinoSample& s = samples[i];
      std::optional<KinodynamicDynamics::StageDynamics> dyn;
      ASSERT_EQ(dyn_.MakeStageDynamics(s.mode, kDt, integrator, dyn), KinodynamicError::kNoError);
      auto data = (*dyn)->createData();
      (*dyn)->forward(s.x, s.u, *data);
      (*dyn)->dForward(s.x, s.u, *data);
      const Eigen::VectorXd xn = data->xnext_;

      auto fd = (*dyn)->createData();
      Eigen::MatrixXd jx_fd(ndx, ndx);
      Eigen::MatrixXd ju_fd(ndx, nu);
      Eigen::VectorXd dx = Eigen::VectorXd::Zero(ndx);
      Eigen::VectorXd xp(s.x.size());
      Eigen::VectorXd xm(s.x.size());
      Eigen::VectorXd dp(ndx);
      Eigen::VectorXd dm(ndx);
      for (int j = 0; j < ndx; ++j) {
        dx[j] = kH;
        space.integrate(s.x, dx, xp);
        dx[j] = -kH;
        space.integrate(s.x, dx, xm);
        dx[j] = 0.0;
        (*dyn)->forward(xp, s.u, *fd);
        space.difference(xn, fd->xnext_, dp);
        (*dyn)->forward(xm, s.u, *fd);
        space.difference(xn, fd->xnext_, dm);
        jx_fd.col(j) = (dp - dm) / (2.0 * kH);
      }
      for (int j = 0; j < nu; ++j) {
        Eigen::VectorXd up = s.u;
        Eigen::VectorXd um = s.u;
        up[j] += kH;
        um[j] -= kH;
        (*dyn)->forward(s.x, up, *fd);
        space.difference(xn, fd->xnext_, dp);
        (*dyn)->forward(s.x, um, *fd);
        space.difference(xn, fd->xnext_, dm);
        ju_fd.col(j) = (dp - dm) / (2.0 * kH);
      }
      const Eigen::MatrixXd jx = data->Jx();
      const Eigen::MatrixXd ju = data->Ju();
      const ColumnErr ex = CompareColumns(jx, jx_fd, 0, ndx, 1e-7, 1e-5);
      const ColumnErr eu = CompareColumns(ju, ju_fd, 0, nu, 1e-7, 1e-5);
      FoldGroups(groups, jx, jx_fd, ju, ju_fd, 1e-7, 1e-5);
      const bool ok = ex.ratio <= 1.0 && eu.ratio <= 1.0;
      pass += ok ? 1 : 0;
      EXPECT_TRUE(ok) << name << " sample " << i << " mode " << s.mode << " Jx " << ex.ratio
                      << " Ju " << eu.ratio;
    }
    EXPECT_EQ(pass, static_cast<int>(samples.size())) << name;
    Report(name, groups);
  }
}

}  // namespace
