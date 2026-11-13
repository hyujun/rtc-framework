/// @file test_contact_tracking_residual.cpp
/// @brief The contact tracking residual on the public G1 (two 6-D soles) and
///        on a synthetic one-contact floating body: dimensions and block
///        order, the value against independent kinematics, the acceleration
///        against the stage's own ODE, the tilt term, the scale, the 1/2
///        convention of the quadratic cost over it, and the analytic Jacobians
///        against central finite differences.
///
/// Pre-registered: the G1 gate set (seed 20261114, 250 samples, mode i mod 4
/// for the dynamics), every contact tracked with scale 1, step h = 1e-6 on
/// the state tangent (`space.integrate`) and the input, finite differences on
/// a data object of their own. Tolerance, element-wise:
/// `|J − J_fd| ≤ 1e-6 + 1e-5·|J_fd|`.

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/contact_tracking_residual.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/jacobian_compare.hpp"
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
#include <aligator/core/stage-model.hpp>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#pragma GCC diagnostic pop

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace tu = rtc::mpc::test_utils;
using rtc::mpc::ContactTrackingData;
using rtc::mpc::ContactTrackingResidual;
using rtc::mpc::ContactTrackingTarget;
using rtc::mpc::KinodynamicDynamics;
using rtc::mpc::KinodynamicError;
using rtc::mpc::KinodynamicIntegrator;
using tu::ColumnErr;
using tu::CompareColumns;
using tu::KinoSample;
using Ode = KinodynamicDynamics::Ode;
using Space = KinodynamicDynamics::Space;

constexpr std::uint32_t kGateSeed = 20261114U;
constexpr double kH = 1e-6;
constexpr double kAbsTol = 1e-6;
constexpr double kRelTol = 1e-5;
constexpr int kRows = ContactTrackingResidual::kRowsPerContact;

/// Targets that differ from anything the samples produce, so every row is
/// non-zero.
ContactTrackingTarget DrawTarget(std::mt19937& rng) {
  std::uniform_real_distribution<double> sym(-1.0, 1.0);
  auto v3 = [&](double scale) {
    return Eigen::Vector3d(scale * sym(rng), scale * sym(rng), scale * sym(rng));
  };
  ContactTrackingTarget t;
  t.position = v3(1.0);
  t.linear_velocity = v3(0.5);
  t.angular_velocity = v3(0.5);
  t.linear_acceleration = v3(2.0);
  t.angular_acceleration = v3(2.0);
  return t;
}

/// The expected 18-row block of one contact from scratch: the ODE's v̇ on a
/// data of its own, pinocchio kinematics on a data of its own, and the tilt
/// from Eigen's shortest-arc quaternion (independent of the residual's own).
Eigen::Matrix<double, kRows, 1> ExpectedBlock(const Ode& ode, const Eigen::VectorXd& x,
                                              const Eigen::VectorXd& u, pinocchio::FrameIndex fid,
                                              const ContactTrackingTarget& t,
                                              const Eigen::Vector3d& n, double s) {
  const pinocchio::Model& model = ode.pin_model_;
  auto od = ode.createData();
  ode.forward(x, u, *od);
  const Eigen::VectorXd vdot = od->xdot_.tail(model.nv);
  pinocchio::Data data(model);
  pinocchio::forwardKinematics(model, data, x.head(model.nq), x.tail(model.nv), vdot);
  pinocchio::updateFramePlacements(model, data);
  const pinocchio::SE3& m = data.oMf[fid];
  const pinocchio::Motion vel =
      pinocchio::getFrameVelocity(model, data, fid, pinocchio::LOCAL_WORLD_ALIGNED);
  const pinocchio::Motion acc =
      pinocchio::getFrameClassicalAcceleration(model, data, fid, pinocchio::LOCAL_WORLD_ALIGNED);
  const Eigen::Vector3d z = m.rotation().col(2);
  const Eigen::Quaterniond arc = Eigen::Quaterniond::FromTwoVectors(z, n);
  Eigen::Matrix<double, kRows, 1> r;
  r << m.translation() - t.position, -arc.vec(), vel.linear() - t.linear_velocity,
      vel.angular() - t.angular_velocity, acc.linear() - t.linear_acceleration,
      acc.angular() - t.angular_acceleration;
  return s * r;
}

bool AllPositiveZero(const Eigen::Ref<const Eigen::MatrixXd>& m) {
  for (Eigen::Index j = 0; j < m.cols(); ++j) {
    for (Eigen::Index i = 0; i < m.rows(); ++i) {
      if (m(i, j) != 0.0 || std::signbit(m(i, j))) {
        return false;
      }
    }
  }
  return true;
}

class ContactTrackingResidualTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(tu::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << tu::kG1Urdf;
    }
    model_ = tu::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(tu::kG1SolesYaml)),
              rtc::mpc::RobotModelInitError::kNoError);
    ASSERT_EQ(dyn_.Init(handler_), KinodynamicError::kNoError);
    oracle_.emplace(model_, tu::G1SoleFrames(model_), 6);
  }

  /// A residual bound to @p ode with every contact at @p s and seeded targets.
  ContactTrackingResidual MakeResidual(const Ode& ode, double s, std::uint32_t target_seed = 7U) {
    ContactTrackingResidual r(ode);
    EXPECT_TRUE(r.Bind(&ode));
    std::mt19937 rng(target_seed);
    for (int k = 0; k < r.n_contacts(); ++k) {
      EXPECT_TRUE(r.SetScale(k, s));
      EXPECT_TRUE(r.SetTarget(k, DrawTarget(rng)));
    }
    return r;
  }

  void RunFdGate(double scale0, const Eigen::Vector3d& normal, const std::string& tag);

  static Eigen::VectorXd Value(const ContactTrackingResidual& r, const KinoSample& s) {
    auto d = r.createData();
    r.evaluate(s.x, s.u, *d);
    return d->value_;
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  KinodynamicDynamics dyn_;
  std::optional<tu::KinoOracle> oracle_;
};

TEST_F(ContactTrackingResidualTest, DimensionsFollowTheModel) {
  const Ode ode = dyn_.MakeOde(0b00).value();
  const ContactTrackingResidual r(ode);
  const int nc = static_cast<int>(handler_.contact_frames().size());
  EXPECT_EQ(r.n_contacts(), nc);
  EXPECT_EQ(r.nr, kRows * nc);
  EXPECT_EQ(r.ndx1, 2 * model_.nv);
  EXPECT_EQ(r.nu, handler_.nu());
  // The public G1 with two 6-D soles.
  EXPECT_EQ(r.nr, 36);
  EXPECT_EQ(r.ndx1, 58);
  EXPECT_EQ(r.nu, 35);
}

// Block order [p; e_R; v; ω; a; α] per contact, the scale applied to the
// whole block, against kinematics computed from scratch.
TEST_F(ContactTrackingResidualTest, ValueMatchesIndependentKinematics) {
  const std::vector<KinoSample> samples = tu::G1GateSamples(*oracle_, kGateSeed);
  double worst = 0.0;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const KinoSample& s = samples[i];
    const Ode ode = dyn_.MakeOde(s.mode).value();
    const double scale = i % 2 == 0 ? 1.0 : 0.37;
    const ContactTrackingResidual r = MakeResidual(ode, scale);
    const Eigen::VectorXd value = Value(r, s);
    ASSERT_EQ(value.size(), r.nr);
    for (int k = 0; k < r.n_contacts(); ++k) {
      const Eigen::Matrix<double, kRows, 1> expected =
          ExpectedBlock(ode, s.x, s.u, oracle_->contacts()[static_cast<std::size_t>(k)],
                        r.target(k), r.plane_normal(), scale);
      const Eigen::VectorXd got = value.segment(k * kRows, kRows);
      const double err =
          ((got - expected).array().abs() / (1.0 + expected.array().abs())).maxCoeff();
      worst = std::max(worst, err);
      EXPECT_LE(err, 1e-9) << "sample " << i << " contact " << k << "\n got " << got.transpose()
                           << "\n exp " << expected.transpose();
    }
  }
  std::printf("value vs independent kinematics: worst rel err %.3e\n", worst);
}

// The acceleration rows come from the stage's own ODE: bound to the ODE inside
// a stage (either integrator), they equal pinocchio's classical acceleration
// under that ODE's v̇, and a mode switched on that ODE reaches them.
TEST_F(ContactTrackingResidualTest, AccelerationIsTheStageOdeAcceleration) {
  using Stage = aligator::StageModelTpl<double>;
  const std::vector<KinoSample> samples = tu::G1GateSamples(*oracle_, kGateSeed);
  for (const KinodynamicIntegrator integrator :
       {KinodynamicIntegrator::kSemiImplicitEuler, KinodynamicIntegrator::kRk2}) {
    std::optional<KinodynamicDynamics::StageDynamics> sd;
    ASSERT_EQ(dyn_.MakeStageDynamics(0b00, 0.035, integrator, sd), KinodynamicError::kNoError);
    Stage stage(aligator::CostStackTpl<double>(*dyn_.space(), dyn_.nu()), *sd);
    Ode* in_stage = KinodynamicDynamics::OdeOf(stage);
    ASSERT_NE(in_stage, nullptr);
    // Zero targets: the acceleration rows are the acceleration itself.
    ContactTrackingResidual r(*in_stage);
    ASSERT_TRUE(r.Bind(in_stage));
    ASSERT_EQ(r.bound_ode(), in_stage);
    for (int k = 0; k < r.n_contacts(); ++k) {
      ASSERT_TRUE(r.SetScale(k, 1.0));
    }
    int pass = 0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const KinoSample& s = samples[i];
      ASSERT_TRUE(dyn_.SetMode(*in_stage, s.mode));
      const Eigen::VectorXd value = Value(r, s);
      bool ok = true;
      for (int k = 0; k < r.n_contacts(); ++k) {
        const Eigen::Matrix<double, kRows, 1> expected =
            ExpectedBlock(*in_stage, s.x, s.u, oracle_->contacts()[static_cast<std::size_t>(k)],
                          r.target(k), r.plane_normal(), 1.0);
        const Eigen::VectorXd got = value.segment(k * kRows + 12, 6);
        const Eigen::VectorXd exp = expected.tail<6>();
        const double tol = 1e-9 * std::max(1.0, exp.norm());
        ok = ok && (got - exp).cwiseAbs().maxCoeff() <= tol;
      }
      pass += ok ? 1 : 0;
      EXPECT_TRUE(ok) << "sample " << i << " mode " << s.mode;
    }
    EXPECT_EQ(pass, static_cast<int>(samples.size()));
  }
}

// One mode written on the stage ODE changes the acceleration rows; the same
// residual follows without a rebind.
TEST_F(ContactTrackingResidualTest, ModeSwitchOnTheBoundOdeReachesTheResidual) {
  std::mt19937 rng(kGateSeed);
  const KinoSample s = oracle_->Draw(rng, 0b11);
  Ode ode = dyn_.MakeOde(0b11).value();
  const ContactTrackingResidual r = MakeResidual(ode, 1.0);
  const Eigen::VectorXd both = Value(r, s);
  ASSERT_TRUE(dyn_.SetMode(ode, 0b00));
  const Eigen::VectorXd flight = Value(r, s);
  const Ode fresh = dyn_.MakeOde(0b00).value();
  const Eigen::VectorXd fresh_value = Value(MakeResidual(fresh, 1.0), s);
  EXPECT_FALSE(both.isApprox(flight));
  EXPECT_EQ(flight, fresh_value);
  // Position, tilt and velocity rows do not depend on the mode.
  for (int k = 0; k < r.n_contacts(); ++k) {
    EXPECT_EQ(both.segment(k * kRows, 12), flight.segment(k * kRows, 12));
  }
}

// cost(s) = s²·cost(1); a zero scale and a stance bit give a block whose value
// and Jacobian rows are exactly +0.
TEST_F(ContactTrackingResidualTest, ScaleEntersSquaredAndZeroScaleZeroesTheBlock) {
  using Cost = aligator::QuadraticResidualCostTpl<double>;
  std::mt19937 rng(kGateSeed);
  const KinoSample s = oracle_->Draw(rng, 0b01);
  const Ode ode = dyn_.MakeOde(s.mode).value();
  std::uniform_real_distribution<double> w01(0.1, 2.0);
  Eigen::VectorXd w(kRows * 2);
  for (Eigen::Index i = 0; i < w.size(); ++i) {
    w[i] = w01(rng);
  }
  auto cost_at = [&](double scale) {
    const Cost cost(*dyn_.space(), MakeResidual(ode, scale), Eigen::MatrixXd(w.asDiagonal()));
    auto d = cost.createData();
    cost.evaluate(s.x, s.u, *d);
    return d->value_;
  };
  const double c1 = cost_at(1.0);
  ASSERT_GT(c1, 0.0);
  for (const double scale : {0.005, 0.5, 1.0}) {
    EXPECT_NEAR(cost_at(scale) / (scale * scale * c1), 1.0, 1e-12) << "s " << scale;
  }

  auto expect_block_zero = [&](const ContactTrackingResidual& r, int zero_k, const char* why) {
    auto d = r.createData();
    r.evaluate(s.x, s.u, *d);
    r.computeJacobians(s.x, s.u, *d);
    const int other = 1 - zero_k;
    EXPECT_TRUE(AllPositiveZero(d->value_.segment(zero_k * kRows, kRows))) << why;
    EXPECT_TRUE(AllPositiveZero(d->Jx_.middleRows(zero_k * kRows, kRows))) << why;
    EXPECT_TRUE(AllPositiveZero(d->Ju_.middleRows(zero_k * kRows, kRows))) << why;
    EXPECT_FALSE(d->value_.segment(other * kRows, kRows).isZero()) << why;
    EXPECT_FALSE(d->Jx_.middleRows(other * kRows, kRows).isZero()) << why;
  };
  for (int k = 0; k < 2; ++k) {
    ContactTrackingResidual r = MakeResidual(ode, 1.0);
    // Leave a non-zero Jacobian on the data first: the zeroed block must not
    // keep it.
    ASSERT_TRUE(r.SetScale(k, 0.0));
    expect_block_zero(r, k, "scale 0");

    rtc::mpc::StageReference stage{};
    stage.mode = 1U << static_cast<unsigned>(k);
    stage.impact_proximity[0] = 0.8;
    stage.impact_proximity[1] = 0.6;
    ASSERT_TRUE(r.SetFromStage(stage, rtc::mpc::FlatSurface(0.0)));
    EXPECT_EQ(r.scale(k), 0.0);
    EXPECT_EQ(r.scale(1 - k), stage.impact_proximity[static_cast<std::size_t>(1 - k)]);
    expect_block_zero(r, k, "stance bit");
  }

  // A data that once held a tracked block, then evaluated with that contact
  // at scale 0, comes back to +0.
  ContactTrackingResidual r = MakeResidual(ode, 1.0);
  auto d = r.createData();
  r.evaluate(s.x, s.u, *d);
  r.computeJacobians(s.x, s.u, *d);
  ASSERT_FALSE(d->Jx_.topRows(kRows).isZero());
  ASSERT_TRUE(r.SetScale(0, 0.0));
  ASSERT_TRUE(r.SetScale(1, 0.0));
  r.evaluate(s.x, s.u, *d);
  r.computeJacobians(s.x, s.u, *d);
  EXPECT_TRUE(AllPositiveZero(d->value_));
  EXPECT_TRUE(AllPositiveZero(d->Jx_));
  EXPECT_TRUE(AllPositiveZero(d->Ju_));
}

// The cost over the residual is 1/2 rᵀ W r with W used as given: a one-hot
// weight on row i gives 1/2·r_i².
TEST_F(ContactTrackingResidualTest, QuadraticCostIsHalfWeightedSquare) {
  using Cost = aligator::QuadraticResidualCostTpl<double>;
  std::mt19937 rng(kGateSeed + 1U);
  const KinoSample s = oracle_->Draw(rng, 0b10);
  const Ode ode = dyn_.MakeOde(s.mode).value();
  ContactTrackingResidual r = MakeResidual(ode, 0.5);
  // A tilted plane, so no tilt row is zero by symmetry.
  ASSERT_TRUE(r.SetPlaneNormal(Eigen::Vector3d(0.1, -0.2, 1.0).normalized()));
  const Eigen::VectorXd value = Value(r, s);
  for (int i = 0; i < r.nr; ++i) {
    ASSERT_NE(value[i], 0.0) << "row " << i;
    Eigen::MatrixXd w = Eigen::MatrixXd::Zero(r.nr, r.nr);
    w(i, i) = 3.0;
    const Cost cost(*dyn_.space(), r, w);
    auto d = cost.createData();
    cost.evaluate(s.x, s.u, *d);
    EXPECT_NEAR(d->value_ / (0.5 * 3.0 * value[i] * value[i]), 1.0, 1e-12) << "row " << i;
  }
}

// The gate over the G1 set with contact 0 at @p scale0 (contact 1 at 1) and
// the plane normal @p normal; reports the column groups under @p tag.
void ContactTrackingResidualTest::RunFdGate(double scale0, const Eigen::Vector3d& normal,
                                            const std::string& tag) {
  const Space& space = *dyn_.space();
  const int ndx = space.ndx();
  const int nu = dyn_.nu();
  const int nv = model_.nv;
  const int fs = dyn_.force_size();
  const std::vector<KinoSample> samples = tu::G1GateSamples(*oracle_, kGateSeed);
  // Column groups: q · v (state), force · moment · q̈_j (input).
  std::array<ColumnErr, 5> groups{};
  int pass_x = 0;
  int pass_u = 0;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const KinoSample& s = samples[i];
    const Ode ode = dyn_.MakeOde(s.mode).value();
    ContactTrackingResidual r = MakeResidual(ode, 1.0, static_cast<std::uint32_t>(i));
    ASSERT_TRUE(r.SetScale(0, scale0));
    ASSERT_TRUE(r.SetPlaneNormal(normal));
    auto data = r.createData();
    r.evaluate(s.x, s.u, *data);
    r.computeJacobians(s.x, s.u, *data);

    auto fd = r.createData();
    Eigen::MatrixXd jx_fd(r.nr, ndx);
    Eigen::MatrixXd ju_fd(r.nr, nu);
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(ndx);
    Eigen::VectorXd xp(s.x.size());
    Eigen::VectorXd xm(s.x.size());
    for (int j = 0; j < ndx; ++j) {
      dx[j] = kH;
      space.integrate(s.x, dx, xp);
      dx[j] = -kH;
      space.integrate(s.x, dx, xm);
      dx[j] = 0.0;
      r.evaluate(xp, s.u, *fd);
      const Eigen::VectorXd fp = fd->value_;
      r.evaluate(xm, s.u, *fd);
      jx_fd.col(j) = (fp - fd->value_) / (2.0 * kH);
    }
    for (int j = 0; j < nu; ++j) {
      Eigen::VectorXd up = s.u;
      Eigen::VectorXd um = s.u;
      up[j] += kH;
      um[j] -= kH;
      r.evaluate(s.x, up, *fd);
      const Eigen::VectorXd fp = fd->value_;
      r.evaluate(s.x, um, *fd);
      ju_fd.col(j) = (fp - fd->value_) / (2.0 * kH);
    }
    const Eigen::MatrixXd jx = data->Jx_;
    const Eigen::MatrixXd ju = data->Ju_;
    const ColumnErr ex = CompareColumns(jx, jx_fd, 0, ndx, kAbsTol, kRelTol);
    const ColumnErr eu = CompareColumns(ju, ju_fd, 0, nu, kAbsTol, kRelTol);
    groups[0].Fold(CompareColumns(jx, jx_fd, 0, nv, kAbsTol, kRelTol));
    groups[1].Fold(CompareColumns(jx, jx_fd, nv, nv, kAbsTol, kRelTol));
    for (int c = 0; c < dyn_.n_contacts(); ++c) {
      groups[2].Fold(CompareColumns(ju, ju_fd, c * fs, 3, kAbsTol, kRelTol));
      groups[3].Fold(CompareColumns(ju, ju_fd, c * fs + 3, fs - 3, kAbsTol, kRelTol));
    }
    groups[4].Fold(CompareColumns(ju, ju_fd, dyn_.n_contacts() * fs, nv - 6, kAbsTol, kRelTol));
    pass_x += ex.ratio <= 1.0 ? 1 : 0;
    pass_u += eu.ratio <= 1.0 ? 1 : 0;
    EXPECT_LE(ex.ratio, 1.0) << tag << " sample " << i << " mode " << s.mode << " Jx abs "
                             << ex.abs;
    EXPECT_LE(eu.ratio, 1.0) << tag << " sample " << i << " mode " << s.mode << " Ju abs "
                             << eu.abs;
  }
  EXPECT_EQ(pass_x, static_cast<int>(samples.size())) << tag;
  EXPECT_EQ(pass_u, static_cast<int>(samples.size())) << tag;
  const std::array<const char*, 5> names{"q", "v", "force", "moment", "qdd_j"};
  for (std::size_t g = 0; g < names.size(); ++g) {
    std::printf("contact tracking (%s) %-7s worst err/tol %.3e  abs %.3e\n", tag.c_str(), names[g],
                groups[g].ratio, groups[g].abs);
    RecordProperty("fd_worst_ratio_" + tag + "_" + names[g], std::to_string(groups[g].ratio));
  }
}

TEST_F(ContactTrackingResidualTest, JacobiansMatchCentralDifferences) {
  RunFdGate(1.0, Eigen::Vector3d::UnitZ(), "registered");
}

// Beyond the registered gate: a scaled block and a tilted plane.
TEST_F(ContactTrackingResidualTest, JacobiansMatchCentralDifferencesScaledOnATiltedPlane) {
  RunFdGate(0.37, Eigen::Vector3d(0.15, -0.1, 1.0).normalized(), "scaled_tilted");
}

// Binding, setter and data-creation contracts.
TEST_F(ContactTrackingResidualTest, BindingAndSettersRefuseWhatDoesNotFit) {
  const Ode ode = dyn_.MakeOde(0b11).value();
  ContactTrackingResidual r(ode);
  EXPECT_EQ(r.bound_ode(), nullptr);
  EXPECT_THROW((void)r.createData(), std::logic_error);

  // Another contact list (one sole only) does not fit.
  const Ode one(*dyn_.space(), model_, model_.gravity.linear(), std::vector<bool>{true},
                std::vector<pinocchio::FrameIndex>{oracle_->contacts()[0]}, 6);
  EXPECT_FALSE(r.Bind(&one));
  EXPECT_EQ(r.bound_ode(), nullptr);
  ASSERT_TRUE(r.Bind(&ode));
  EXPECT_FALSE(r.Bind(&one));
  EXPECT_EQ(r.bound_ode(), &ode);

  EXPECT_FALSE(r.SetScale(-1, 0.5));
  EXPECT_FALSE(r.SetScale(2, 0.5));
  EXPECT_FALSE(r.SetScale(0, 1.0 + 1e-12));
  EXPECT_FALSE(r.SetScale(0, -1e-12));
  EXPECT_FALSE(r.SetScale(0, std::nan("")));
  EXPECT_EQ(r.scale(0), 0.0);
  ContactTrackingTarget bad;
  bad.angular_acceleration.y() = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(r.SetTarget(0, bad));
  EXPECT_FALSE(r.SetTarget(2, ContactTrackingTarget{}));
  EXPECT_FALSE(r.SetPlaneNormal(Eigen::Vector3d(0.0, 0.0, 2.0)));
  EXPECT_FALSE(r.SetPlaneNormal(Eigen::Vector3d(0.0, std::nan(""), 1.0)));
  EXPECT_EQ(r.plane_normal(), Eigen::Vector3d::UnitZ());

  // A refused node writes nothing.
  ASSERT_TRUE(r.SetScale(0, 0.25));
  rtc::mpc::StageReference stage{};
  stage.impact_proximity[0] = 0.5;
  stage.impact_proximity[1] = 1.5;  // out of range, on a swing contact
  stage.contact_pose[0] = 9.0;
  EXPECT_FALSE(r.SetFromStage(stage, rtc::mpc::FlatSurface(0.0)));
  EXPECT_EQ(r.scale(0), 0.25);
  EXPECT_EQ(r.target(0).position.x(), 0.0);
  stage.mode = 0b10;  // the out-of-range entry now belongs to a stance contact
  rtc::mpc::SurfaceFrame tilted = rtc::mpc::FlatSurface(0.0);
  tilted.normal = {0.0, 0.0, 1.5};
  EXPECT_FALSE(r.SetFromStage(stage, tilted));
  EXPECT_TRUE(r.SetFromStage(stage, rtc::mpc::FlatSurface(0.0)));
  EXPECT_EQ(r.scale(0), 0.5);
  EXPECT_EQ(r.scale(1), 0.0);
  EXPECT_EQ(r.target(0).position.x(), 9.0);

  // Unbound after the data exists: NaN, never a stale value.
  std::mt19937 rng(1U);
  const KinoSample s = oracle_->Draw(rng, 0b11);
  auto d = r.createData();
  ASSERT_TRUE(r.Bind(nullptr));
  r.evaluate(s.x, s.u, *d);
  r.computeJacobians(s.x, s.u, *d);
  EXPECT_TRUE(d->value_.array().isNaN().all());
  EXPECT_TRUE(d->Jx_.array().isNaN().all());
  EXPECT_TRUE(d->Ju_.array().isNaN().all());
}

// ── The tilt term, on a synthetic floating body whose contact frame is the
//    root frame, so its orientation is exactly the base quaternion. ─────────

class ContactTrackingTiltTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const pinocchio::JointIndex root =
        model_.addJoint(0, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), "root");
    model_.appendBodyToJoint(
        root, pinocchio::Inertia(8.0, Eigen::Vector3d(0.0, 0.0, 0.2), Eigen::Matrix3d::Identity()),
        pinocchio::SE3::Identity());
    const pinocchio::JointIndex hinge = model_.addJoint(
        root, pinocchio::JointModelRY(),
        pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.4)), "hinge");
    model_.appendBodyToJoint(
        hinge,
        pinocchio::Inertia(1.0, Eigen::Vector3d(0.1, 0.0, 0.0), 0.1 * Eigen::Matrix3d::Identity()),
        pinocchio::SE3::Identity());
    frame_ = model_.addFrame(
        pinocchio::Frame("pad", root, pinocchio::SE3::Identity(), pinocchio::OP_FRAME));
    space_.emplace(model_);
    ode_.emplace(*space_, model_, model_.gravity.linear(), std::vector<bool>{false},
                 std::vector<pinocchio::FrameIndex>{frame_}, 6);
    residual_.emplace(*ode_);
    ASSERT_TRUE(residual_->Bind(&*ode_));
    ASSERT_TRUE(residual_->SetScale(0, 1.0));
  }

  /// The tilt rows at base orientation @p rot (positions, velocities zero).
  Eigen::Vector3d TiltAt(const Eigen::Matrix3d& rot) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(model_.nq + model_.nv);
    x.head(model_.nq) = pinocchio::neutral(model_);
    x.segment<4>(3) = Eigen::Quaterniond(rot).normalized().coeffs();
    const Eigen::VectorXd u = Eigen::VectorXd::Zero(ode_->nu());
    auto d = residual_->createData();
    residual_->evaluate(x, u, *d);
    return d->value_.segment<3>(3);
  }

  pinocchio::Model model_;
  pinocchio::FrameIndex frame_{0};
  std::optional<Space> space_;
  std::optional<Ode> ode_;
  std::optional<ContactTrackingResidual> residual_;
};

TEST_F(ContactTrackingTiltTest, ZeroWhenTheFrameZAxisIsAlongTheNormal) {
  EXPECT_LE(TiltAt(Eigen::Matrix3d::Identity()).norm(), 1e-15);
  // A tilted plane, and the frame tilted the same way.
  const Eigen::Matrix3d tilt(Eigen::AngleAxisd(0.3, Eigen::Vector3d(1.0, -2.0, 0.0).normalized()));
  ASSERT_TRUE(residual_->SetPlaneNormal(tilt.col(2)));
  EXPECT_LE(TiltAt(tilt).norm(), 1e-15);
}

TEST_F(ContactTrackingTiltTest, NormIsSineOfHalfTheTilt) {
  constexpr double kTheta = 0.2;
  const Eigen::Vector3d e = TiltAt(Eigen::AngleAxisd(kTheta, Eigen::Vector3d::UnitX()).matrix());
  EXPECT_NEAR(e.norm(), std::sin(kTheta / 2.0), 1e-12);
  // Along +x: minus the vector part of the levelling arc, a turn of −θ about x.
  EXPECT_LE(std::abs(e.y()) + std::abs(e.z()), 1e-15);
  EXPECT_GT(e.x(), 0.0);
}

// Heading is not tracked: turning the frame about its own z axis leaves the
// tilt unchanged; turning it about the plane normal turns the tilt vector
// with it and keeps its norm.
TEST_F(ContactTrackingTiltTest, HeadingDoesNotChangeTheTilt) {
  const Eigen::Matrix3d tilt = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).matrix();
  const Eigen::Vector3d e0 = TiltAt(tilt);
  for (const double yaw : {0.3, -1.1, 2.9}) {
    const Eigen::Matrix3d about_z = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).matrix();
    EXPECT_LE((TiltAt(tilt * about_z) - e0).norm(), 1e-15) << "own z, yaw " << yaw;
    const Eigen::Vector3d e_world = TiltAt(about_z * tilt);
    EXPECT_LE((e_world - about_z * e0).norm(), 1e-15) << "world z, yaw " << yaw;
    EXPECT_NEAR(e_world.norm(), e0.norm(), 1e-15);
  }
  // On a level frame both are the zero vector.
  EXPECT_LE(TiltAt(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).matrix()).norm(), 1e-15);
}

// A frame flipped against the plane (the shortest arc is undefined there):
// on, inside and just outside the held divisor, the tilt value stays within
// [0, 1] and its Jacobian finite and of order 1/kMinArcNorm.
TEST_F(ContactTrackingTiltTest, FlippedFrameStaysBounded) {
  constexpr double kMin = ContactTrackingResidual::kMinArcNorm;
  for (const double gap : {0.0, 0.5 * kMin, 4.0 * kMin}) {
    const Eigen::Matrix3d rot =
        Eigen::AngleAxisd(M_PI - gap, Eigen::Vector3d(1.0, 0.3, 0.0).normalized()).matrix();
    Eigen::VectorXd x = Eigen::VectorXd::Zero(model_.nq + model_.nv);
    x.head(model_.nq) = pinocchio::neutral(model_);
    x.segment<4>(3) = Eigen::Quaterniond(rot).coeffs();
    const Eigen::VectorXd u = Eigen::VectorXd::Zero(ode_->nu());
    auto d = residual_->createData();
    residual_->evaluate(x, u, *d);
    residual_->computeJacobians(x, u, *d);
    const Eigen::Vector3d e = d->value_.segment<3>(3);
    EXPECT_TRUE(e.allFinite()) << "gap " << gap;
    EXPECT_LE(e.norm(), 1.0 + 1e-12) << "gap " << gap;
    EXPECT_TRUE(d->Jx_.allFinite()) << "gap " << gap;
    EXPECT_LE(d->Jx_.middleRows<3>(3).cwiseAbs().maxCoeff(), 10.0 / kMin) << "gap " << gap;
  }
}

}  // namespace
