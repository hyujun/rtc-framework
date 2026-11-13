/// @file test_contact_constraints.cpp
/// @brief The contact equality and the wrench cone inequality of the
///        kinodynamic OCP on the public G1 (two 6-D soles, and the same soles
///        declared 3-D), and the frame kinematics they share: row counts from
///        the force size, the mode mask read from the bound ODE, sign fixtures
///        of the friction and centre-of-pressure rows, the equality values and
///        Jacobians against their gain combination of the frame kinematics,
///        and every analytic Jacobian against central finite differences.
///
/// Pre-registered: the G1 gate set (seed 20261114, 250 samples, mode i mod 4);
/// gains position_z 100, orientation 80, linear_velocity_xy 20,
/// linear_velocity_z 10, angular_velocity 20, accelerations 1 (and a case of
/// non-unit acceleration gains); friction μ 0.4, ε 25 N², grip 0 (and a grip
/// case); the rectangle x ±0.09 m, y ±0.03 m. Mask: for every mode on every
/// sample, inactive equality rows +0.0 (sign bit clear), inactive inequality
/// rows exactly −1, their Jacobian rows bitwise zero. Friction fixtures to
/// 1e-12 (the value against −h to 1e-15·max(1, |h|)); CoP fixtures to
/// 1e-9·F_z on an edge, τ_z invariance to 1e-12·max(1, |w|). Equality values
/// and composed Jacobians: rel 1e-12. Finite differences: central, h = 1e-6
/// on the state tangent (`space.integrate`) and the input, on data objects of
/// their own; element-wise `|J − J_fd| ≤ 1e-6 + 1e-5·|J_fd|`, every sample.

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/contact_constraints.hpp"
#include "rtc_mpc/ocp/contact_kinematics.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "rtc_mpc/reference/surface_frame.hpp"
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
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace {

namespace tu = rtc::mpc::test_utils;
using rtc::mpc::ContactConstraintResidual;
using rtc::mpc::ContactKinematics;
using rtc::mpc::ContactRectangle;
using rtc::mpc::ContactWrenchConeResidual;
using rtc::mpc::FootConstraintGains;
using rtc::mpc::FrameMotion;
using rtc::mpc::FrictionCone;
using rtc::mpc::KinodynamicDynamics;
using rtc::mpc::KinodynamicError;
using rtc::mpc::StageReference;
using rtc::mpc::SurfaceFrame;
using tu::ColumnErr;
using tu::CompareColumns;
using tu::KinoSample;
using Ode = KinodynamicDynamics::Ode;
using Space = KinodynamicDynamics::Space;

constexpr std::uint32_t kGateSeed = 20261114U;
constexpr double kH = 1e-6;
constexpr double kAbsTol = 1e-6;
constexpr double kRelTol = 1e-5;
constexpr double kValueRelTol = 1e-12;
constexpr int kFailuresShown = 5;

/// The public G1's soles declared as 3-D point contacts.
constexpr const char* kG1PointSolesYaml = R"(
end_effector_frame: pelvis
base_frame: universe
contact_frames:
  - name: left_sole
    dim: 3
  - name: right_sole
    dim: 3
)";

FootConstraintGains RegisteredGains() {
  FootConstraintGains g;
  g.position_z = 100.0;
  g.orientation = 80.0;
  g.linear_velocity_xy = 20.0;
  g.linear_velocity_z = 10.0;
  g.angular_velocity = 20.0;
  g.linear_acceleration_xy = 1.0;
  g.linear_acceleration_z = 1.0;
  g.angular_acceleration = 1.0;
  return g;
}

FootConstraintGains NonUnitAccelerationGains() {
  FootConstraintGains g = RegisteredGains();
  g.linear_acceleration_xy = 0.6;
  g.linear_acceleration_z = 1.7;
  g.angular_acceleration = 0.35;
  return g;
}

FrictionCone RegisteredFriction(double grip = 0.0) {
  FrictionCone f;
  f.mu = 0.4;
  f.grip_force = grip;
  f.regularization = 25.0;
  return f;
}

ContactRectangle RegisteredRectangle() {
  ContactRectangle r;
  r.x_min = -0.09;
  r.x_max = 0.09;
  r.y_min = -0.03;
  r.y_max = 0.03;
  return r;
}

bool BitwiseZero(double v) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  return bits == 0U;
}

bool AllBitwiseZero(const Eigen::Ref<const Eigen::MatrixXd>& m) {
  for (Eigen::Index j = 0; j < m.cols(); ++j) {
    for (Eigen::Index i = 0; i < m.rows(); ++i) {
      if (!BitwiseZero(m(i, j))) {
        return false;
      }
    }
  }
  return true;
}

bool AnyNonZero(const Eigen::Ref<const Eigen::MatrixXd>& m) {
  return (m.array() != 0.0).any();
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& w) {
  Eigen::Matrix3d s;
  s << 0.0, -w.z(), w.y(), w.z(), 0.0, -w.x(), -w.y(), w.x(), 0.0;
  return s;
}

/// Worst element-wise `|got − exp| / max(1, |exp|)`.
double RelErr(const Eigen::Ref<const Eigen::MatrixXd>& got,
              const Eigen::Ref<const Eigen::MatrixXd>& exp) {
  double worst = 0.0;
  for (Eigen::Index j = 0; j < exp.cols(); ++j) {
    for (Eigen::Index i = 0; i < exp.rows(); ++i) {
      worst = std::max(worst, std::abs(got(i, j) - exp(i, j)) / std::max(1.0, std::abs(exp(i, j))));
    }
  }
  return worst;
}

/// The FD gate's ratio over a whole block (≤ 1 passes).
ColumnErr CompareBlock(const Eigen::MatrixXd& j, const Eigen::MatrixXd& j_fd) {
  return CompareColumns(j, j_fd, 0, static_cast<int>(j.cols()), kAbsTol, kRelTol);
}

/// A node reference with every pose, twist and acceleration entry drawn (the
/// residual reads only the z ones).
StageReference DrawStage(std::mt19937& rng, int nc) {
  std::uniform_real_distribution<double> sym(-1.0, 1.0);
  StageReference s{};
  for (int k = 0; k < nc; ++k) {
    const auto o = static_cast<std::size_t>(k);
    for (std::size_t i = 0; i < 7; ++i) {
      s.contact_pose[7 * o + i] = sym(rng);
    }
    for (std::size_t i = 0; i < 6; ++i) {
      s.contact_twist[6 * o + i] = 0.5 * sym(rng);
      s.contact_acceleration[6 * o + i] = 2.0 * sym(rng);
    }
  }
  return s;
}

SurfaceFrame TerrainWithNormal(const Eigen::Vector3d& n) {
  SurfaceFrame t = rtc::mpc::FlatSurface(0.0);
  t.normal = {n.x(), n.y(), n.z()};
  return t;
}

const Eigen::Vector3d kTiltedNormal = Eigen::Vector3d(0.1, -0.15, 1.0).normalized();

/// Central differences of @p f(x, u) on the state tangent and the input.
template <typename F>
void CentralDifferences(const Space& space, const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                        F&& f, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) {
  const int ndx = space.ndx();
  Eigen::VectorXd dx = Eigen::VectorXd::Zero(ndx);
  Eigen::VectorXd xp(x.size());
  Eigen::VectorXd xm(x.size());
  for (int j = 0; j < ndx; ++j) {
    dx[j] = kH;
    space.integrate(x, dx, xp);
    dx[j] = -kH;
    space.integrate(x, dx, xm);
    dx[j] = 0.0;
    const Eigen::VectorXd fp = f(xp, u);
    jx.col(j) = (fp - f(xm, u)) / (2.0 * kH);
  }
  for (Eigen::Index j = 0; j < u.size(); ++j) {
    Eigen::VectorXd up = u;
    Eigen::VectorXd um = u;
    up[j] += kH;
    um[j] -= kH;
    const Eigen::VectorXd fp = f(x, up);
    ju.col(j) = (fp - f(x, um)) / (2.0 * kH);
  }
}

/// One contact declaration of the G1: its handler, dynamics, oracle and the
/// gate set drawn on its input layout.
struct Rig {
  rtc::mpc::RobotModelHandler handler;
  KinodynamicDynamics dyn;
  std::optional<tu::KinoOracle> oracle;
  std::vector<KinoSample> samples;

  [[nodiscard]] static bool Stance(std::uint32_t mode, int k) {
    return ((mode >> static_cast<unsigned>(k)) & 1U) != 0U;
  }

  [[nodiscard]] std::uint32_t full_stance() const { return dyn.n_modes() - 1U; }

  [[nodiscard]] int stance_rows() const { return dyn.force_size() == 6 ? 6 : 3; }
};

class ContactConstraintsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(tu::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << tu::kG1Urdf;
    }
    model_ = tu::BuildG1Floating();
    InitRig(rig6_, tu::kG1SolesYaml);
    InitRig(rig3_, kG1PointSolesYaml);
  }

  void InitRig(Rig& rig, const char* yaml) {
    ASSERT_EQ(rig.handler.Init(model_, YAML::Load(yaml)), rtc::mpc::RobotModelInitError::kNoError);
    ASSERT_EQ(rig.dyn.Init(rig.handler), KinodynamicError::kNoError);
    rig.oracle.emplace(model_, tu::G1SoleFrames(model_), rig.dyn.force_size());
    rig.samples = tu::G1GateSamples(*rig.oracle, kGateSeed);
  }

  void RunMask(Rig& rig, const char* tag);
  void RunValueGate(Rig& rig, const FootConstraintGains& g, const Eigen::Vector3d& n,
                    const char* tag);
  void RunComposedJacobianGate(Rig& rig, const FootConstraintGains& g, const Eigen::Vector3d& n,
                               const char* tag);
  void RunEqualityFdGate(Rig& rig, const FootConstraintGains& g, const Eigen::Vector3d& n,
                         const std::string& tag);
  void RunInequalityFdGate(Rig& rig, const std::string& tag);

  pinocchio::Model model_;
  Rig rig6_;
  Rig rig3_;
};

// ── Dimensions ──────────────────────────────────────────────────────────────

TEST_F(ContactConstraintsTest, RowCountsFollowTheForceSize) {
  for (Rig* rig : {&rig6_, &rig3_}) {
    const Ode ode = rig->dyn.MakeOde(0).value();
    const int fs = rig->dyn.force_size();
    const int nc = rig->dyn.n_contacts();
    const ContactConstraintResidual eq(ode, RegisteredGains());
    const ContactWrenchConeResidual ineq(ode, RegisteredFriction(), RegisteredRectangle());
    const int eq_rows = (fs == 6 ? 6 : 3) + 1;
    const int ineq_rows = fs == 6 ? 5 : 1;
    EXPECT_EQ(ContactConstraintResidual::EqualityRows(fs), eq_rows) << "fs " << fs;
    EXPECT_EQ(ContactWrenchConeResidual::InequalityRows(fs), ineq_rows) << "fs " << fs;
    EXPECT_EQ(eq.force_size(), fs);
    EXPECT_EQ(ineq.force_size(), fs);
    EXPECT_EQ(eq.n_contacts(), nc);
    EXPECT_EQ(ineq.n_contacts(), nc);
    EXPECT_EQ(eq.rows_per_contact(), eq_rows);
    EXPECT_EQ(ineq.rows_per_contact(), ineq_rows);
    EXPECT_EQ(eq.nr, nc * eq_rows) << "fs " << fs;
    EXPECT_EQ(ineq.nr, nc * ineq_rows) << "fs " << fs;
    EXPECT_EQ(eq.ndx1, 2 * model_.nv);
    EXPECT_EQ(ineq.ndx1, 2 * model_.nv);
    EXPECT_EQ(eq.nu, rig->dyn.nu());
    EXPECT_EQ(ineq.nu, rig->dyn.nu());
  }
  // The public G1: two 6-D soles give 14 + 10 rows, two 3-D ones 8 + 2.
  ASSERT_EQ(rig6_.dyn.force_size(), 6);
  ASSERT_EQ(rig3_.dyn.force_size(), 3);
  const Ode ode6 = rig6_.dyn.MakeOde(0).value();
  const Ode ode3 = rig3_.dyn.MakeOde(0).value();
  EXPECT_EQ(ContactConstraintResidual(ode6, RegisteredGains()).nr, 14);
  EXPECT_EQ(ContactWrenchConeResidual(ode6, RegisteredFriction(), RegisteredRectangle()).nr, 10);
  EXPECT_EQ(ContactConstraintResidual(ode3, RegisteredGains()).nr, 8);
  EXPECT_EQ(ContactWrenchConeResidual(ode3, RegisteredFriction(), RegisteredRectangle()).nr, 2);
}

// ── Mask ────────────────────────────────────────────────────────────────────

// Every mode written on the one bound ODE, every gate sample: the rows of the
// phase a contact is not in hold the inactive value with bitwise-zero
// Jacobian rows; the rows of the phase it is in do not.
void ContactConstraintsTest::RunMask(Rig& rig, const char* tag) {
  Ode ode = rig.dyn.MakeOde(0).value();
  ContactConstraintResidual eq(ode, RegisteredGains());
  ContactWrenchConeResidual ineq(ode, RegisteredFriction(), RegisteredRectangle());
  ASSERT_TRUE(eq.Bind(&ode));
  ASSERT_TRUE(ineq.Bind(&ode));
  const int nc = rig.dyn.n_contacts();
  std::mt19937 rng(kGateSeed);
  ASSERT_TRUE(eq.SetFromStage(DrawStage(rng, nc), rtc::mpc::FlatSurface(0.0)));
  auto de = eq.createData();
  auto di = ineq.createData();
  const int re = eq.rows_per_contact();
  const int ri = ineq.rows_per_contact();
  const int sr = rig.stance_rows();
  int pass = 0;
  int total = 0;
  int shown = 0;
  for (std::uint32_t mode = 0; mode < rig.dyn.n_modes(); ++mode) {
    ASSERT_TRUE(rig.dyn.SetMode(ode, mode));
    for (std::size_t i = 0; i < rig.samples.size(); ++i) {
      const KinoSample& s = rig.samples[i];
      eq.evaluate(s.x, s.u, *de);
      eq.computeJacobians(s.x, s.u, *de);
      ineq.evaluate(s.x, s.u, *di);
      ineq.computeJacobians(s.x, s.u, *di);
      bool ok = true;
      for (int k = 0; k < nc; ++k) {
        const bool stance = Rig::Stance(mode, k);
        // Equality: in stance the swing row is inactive, in swing the stance rows.
        const int in0 = stance ? k * re + sr : k * re;
        const int in_n = stance ? 1 : sr;
        const int act0 = stance ? k * re : k * re + sr;
        const int act_n = stance ? sr : 1;
        ok = ok && AllBitwiseZero(de->value_.segment(in0, in_n));
        ok = ok && AllBitwiseZero(de->Jx_.middleRows(in0, in_n));
        ok = ok && AllBitwiseZero(de->Ju_.middleRows(in0, in_n));
        ok = ok && AnyNonZero(de->value_.segment(act0, act_n));
        ok = ok && AnyNonZero(de->Jx_.middleRows(act0, act_n));
        // Inequality: active in stance only.
        const Eigen::VectorXd vi = di->value_.segment(k * ri, ri);
        if (stance) {
          ok = ok && (vi.array() != ContactWrenchConeResidual::kInactiveValue).any();
          ok = ok && AnyNonZero(di->Ju_.middleRows(k * ri, ri));
        } else {
          ok = ok && (vi.array() == -1.0).all();
          ok = ok && AllBitwiseZero(di->Jx_.middleRows(k * ri, ri));
          ok = ok && AllBitwiseZero(di->Ju_.middleRows(k * ri, ri));
        }
      }
      ++total;
      pass += ok ? 1 : 0;
      if (!ok && shown++ < kFailuresShown) {
        ADD_FAILURE() << tag << " mode " << mode << " sample " << i << "\n eq  "
                      << de->value_.transpose() << "\n ineq " << di->value_.transpose();
      }
    }
  }
  EXPECT_EQ(pass, total) << tag;
  EXPECT_EQ(total, static_cast<int>(rig.dyn.n_modes() * rig.samples.size()));
  std::printf("mask (%s): %d / %d (mode, sample) pairs\n", tag, pass, total);
}

TEST_F(ContactConstraintsTest, InactiveRowsHoldTheInactiveValueForEveryMode6D) {
  RunMask(rig6_, "6-D");
}

TEST_F(ContactConstraintsTest, InactiveRowsHoldTheInactiveValueForEveryMode3D) {
  RunMask(rig3_, "3-D");
}

// The mode is read from the bound ODE at evaluate: a SetMode after binding
// moves the active rows, and the result is that of a residual bound to an
// ODE made in the new mode.
// A mode switched on the bound ODE between `evaluate` and
// `computeJacobians` would pair the evaluated rows with the other mode's ODE
// derivatives: the equality refuses it with a NaN Jacobian.
TEST_F(ContactConstraintsTest, AModeSwitchBetweenEvaluateAndJacobiansGivesANaNJacobian) {
  Rig& rig = rig6_;
  const KinoSample& s = rig.samples[5];
  Ode ode = rig.dyn.MakeOde(0b11).value();
  ContactConstraintResidual eq(ode, RegisteredGains());
  ASSERT_TRUE(eq.Bind(&ode));
  auto d = eq.createData();
  eq.evaluate(s.x, s.u, *d);
  ASSERT_TRUE(rig.dyn.SetMode(ode, 0b01));
  eq.computeJacobians(s.x, s.u, *d);
  EXPECT_TRUE(d->Jx_.array().isNaN().all());
  EXPECT_TRUE(d->Ju_.array().isNaN().all());
  // Evaluated again in the new mode, the Jacobian is finite.
  eq.evaluate(s.x, s.u, *d);
  eq.computeJacobians(s.x, s.u, *d);
  EXPECT_TRUE(d->Jx_.allFinite());
  EXPECT_TRUE(d->Ju_.allFinite());
}

TEST_F(ContactConstraintsTest, SetModeAfterBindingMovesTheActiveRows) {
  Rig& rig = rig6_;
  const KinoSample& s = rig.samples[3];
  const int nc = rig.dyn.n_contacts();
  ASSERT_EQ(nc, 2);
  std::mt19937 rng(kGateSeed + 1U);
  const StageReference stage = DrawStage(rng, nc);
  const SurfaceFrame flat = rtc::mpc::FlatSurface(0.0);

  Ode ode = rig.dyn.MakeOde(0b01).value();
  ContactConstraintResidual eq(ode, RegisteredGains());
  ContactWrenchConeResidual ineq(ode, RegisteredFriction(), RegisteredRectangle());
  ASSERT_TRUE(eq.Bind(&ode));
  ASSERT_TRUE(ineq.Bind(&ode));
  ASSERT_TRUE(eq.SetFromStage(stage, flat));
  auto de = eq.createData();
  auto di = ineq.createData();
  const int re = eq.rows_per_contact();
  const int ri = ineq.rows_per_contact();

  auto run = [&] {
    eq.evaluate(s.x, s.u, *de);
    eq.computeJacobians(s.x, s.u, *de);
    ineq.evaluate(s.x, s.u, *di);
    ineq.computeJacobians(s.x, s.u, *di);
  };
  run();
  // Contact 0 in stance, contact 1 in swing.
  EXPECT_TRUE(AnyNonZero(de->value_.segment(0, 6)));
  EXPECT_TRUE(AllBitwiseZero(de->value_.segment(6, 1)));
  EXPECT_TRUE(AllBitwiseZero(de->value_.segment(re, 6)));
  EXPECT_NE(de->value_[re + 6], 0.0);
  EXPECT_TRUE((di->value_.segment(0, ri).array() != -1.0).any());
  EXPECT_TRUE((di->value_.segment(ri, ri).array() == -1.0).all());

  ASSERT_TRUE(rig.dyn.SetMode(ode, 0b10));
  run();
  EXPECT_TRUE(AllBitwiseZero(de->value_.segment(0, 6)));
  EXPECT_TRUE(AllBitwiseZero(de->Jx_.middleRows(0, 6)));
  EXPECT_NE(de->value_[6], 0.0);
  EXPECT_TRUE(AnyNonZero(de->value_.segment(re, 6)));
  EXPECT_TRUE(AllBitwiseZero(de->value_.segment(re + 6, 1)));
  EXPECT_TRUE(AllBitwiseZero(de->Jx_.middleRows(re + 6, 1)));
  EXPECT_TRUE((di->value_.segment(0, ri).array() == -1.0).all());
  EXPECT_TRUE(AllBitwiseZero(di->Ju_.middleRows(0, ri)));
  EXPECT_TRUE((di->value_.segment(ri, ri).array() != -1.0).any());

  // The same as residuals bound to an ODE made in the new mode.
  Ode fresh = rig.dyn.MakeOde(0b10).value();
  ContactConstraintResidual eq2(fresh, RegisteredGains());
  ContactWrenchConeResidual ineq2(fresh, RegisteredFriction(), RegisteredRectangle());
  ASSERT_TRUE(eq2.Bind(&fresh));
  ASSERT_TRUE(ineq2.Bind(&fresh));
  ASSERT_TRUE(eq2.SetFromStage(stage, flat));
  auto de2 = eq2.createData();
  auto di2 = ineq2.createData();
  eq2.evaluate(s.x, s.u, *de2);
  eq2.computeJacobians(s.x, s.u, *de2);
  ineq2.evaluate(s.x, s.u, *di2);
  ineq2.computeJacobians(s.x, s.u, *di2);
  EXPECT_TRUE(de->value_ == de2->value_);
  EXPECT_TRUE(de->Jx_ == de2->Jx_);
  EXPECT_TRUE(de->Ju_ == de2->Ju_);
  EXPECT_TRUE(di->value_ == di2->value_);
  EXPECT_TRUE(di->Jx_ == di2->Jx_);
  EXPECT_TRUE(di->Ju_ == di2->Ju_);
}

// ── Friction ────────────────────────────────────────────────────────────────

// On world axes, at any q: the apex boundary F_z = √ε/μ − grip, ±0.5 N
// either side, on-cone tangential forces, and the value against −h over the
// gate set.
TEST_F(ContactConstraintsTest, FrictionRowSignsAndValue) {
  for (Rig* rig : {&rig6_, &rig3_}) {
    const int fs = rig->dyn.force_size();
    const int nc = rig->dyn.n_contacts();
    const Ode ode = rig->dyn.MakeOde(rig->full_stance()).value();
    for (const double grip : {0.0, 5.0}) {
      const FrictionCone fc = RegisteredFriction(grip);
      ContactWrenchConeResidual ineq(ode, fc, RegisteredRectangle());
      ASSERT_TRUE(ineq.Bind(&ode));
      auto d = ineq.createData();
      const int ri = ineq.rows_per_contact();
      const double fz0 = std::sqrt(fc.regularization) / fc.mu - fc.grip_force;
      double worst_boundary = 0.0;
      double worst_h = 0.0;
      int h_pass = 0;
      int h_total = 0;
      for (std::size_t i = 0; i < rig->samples.size(); ++i) {
        const KinoSample& s = rig->samples[i];
        // −h over the drawn wrenches.
        ineq.evaluate(s.x, s.u, *d);
        for (int k = 0; k < nc; ++k) {
          const Eigen::Vector3d f = s.u.segment<3>(k * fs);
          const double h = fc.mu * (f.z() + fc.grip_force) -
                           std::sqrt(f.x() * f.x() + f.y() * f.y() + fc.regularization);
          const double err = std::abs(d->value_[k * ri] + h) / std::max(1.0, std::abs(h));
          worst_h = std::max(worst_h, err);
          ++h_total;
          h_pass += err <= 1e-15 ? 1 : 0;
        }
        if (i >= 10) {
          continue;  // the fixtures on the first ten configurations
        }
        for (int k = 0; k < nc; ++k) {
          auto c_at = [&](const Eigen::Vector3d& f) {
            Eigen::VectorXd u = s.u;
            u.segment<3>(k * fs) = f;
            ineq.evaluate(s.x, u, *d);
            return d->value_[k * ri];
          };
          const double on_apex = c_at(Eigen::Vector3d(0.0, 0.0, fz0));
          worst_boundary = std::max(worst_boundary, std::abs(on_apex));
          EXPECT_LE(std::abs(on_apex), 1e-12) << "fs " << fs << " grip " << grip;
          EXPECT_LT(c_at(Eigen::Vector3d(0.0, 0.0, fz0 + 0.5)), 0.0);
          EXPECT_GT(c_at(Eigen::Vector3d(0.0, 0.0, fz0 - 0.5)), 0.0);
          constexpr double kFz = 100.0;
          const double cone = fc.mu * (kFz + fc.grip_force);
          const double ft = std::sqrt(cone * cone - fc.regularization);
          for (const double phi : {0.0, 0.5 * M_PI, 2.1, -0.7}) {
            const double c = c_at(Eigen::Vector3d(ft * std::cos(phi), ft * std::sin(phi), kFz));
            worst_boundary = std::max(worst_boundary, std::abs(c));
            EXPECT_LE(std::abs(c), 1e-12) << "fs " << fs << " grip " << grip << " phi " << phi;
            EXPECT_LT(
                c_at(Eigen::Vector3d(0.9 * ft * std::cos(phi), 0.9 * ft * std::sin(phi), kFz)),
                0.0);
            EXPECT_GT(
                c_at(Eigen::Vector3d(1.1 * ft * std::cos(phi), 1.1 * ft * std::sin(phi), kFz)),
                0.0);
          }
        }
      }
      EXPECT_EQ(h_pass, h_total) << "fs " << fs << " grip " << grip;
      std::printf("friction (fs %d, grip %.1f): boundary worst |c| %.3e, value vs -h worst %.3e\n",
                  fs, grip, worst_boundary, worst_h);
    }
  }
}

// ── Centre of pressure ──────────────────────────────────────────────────────

/// @p q with the base turned so frame @p fid has world rotation @p target.
Eigen::VectorXd WithFrameRotation(const pinocchio::Model& model, Eigen::VectorXd q,
                                  pinocchio::FrameIndex fid, const Eigen::Matrix3d& target) {
  pinocchio::Data data(model);
  q.segment<4>(3) = Eigen::Quaterniond::Identity().coeffs();
  pinocchio::framesForwardKinematics(model, data, q);
  const Eigen::Matrix3d base = target * data.oMf[fid].rotation().transpose();
  q.segment<4>(3) = Eigen::Quaterniond(base).normalized().coeffs();
  return q;
}

// The sole turned (yaw 0.3, tilt 0.1 rad), the wrench built on its own axes:
// the CoP at the centre, on each edge, 0.01 m inside and outside it.
TEST_F(ContactConstraintsTest, CentreOfPressureRowSigns) {
  Rig& rig = rig6_;
  const int nc = rig.dyn.n_contacts();
  const Ode ode = rig.dyn.MakeOde(rig.full_stance()).value();
  const ContactRectangle rect = RegisteredRectangle();
  ContactWrenchConeResidual ineq(ode, RegisteredFriction(), rect);
  ASSERT_TRUE(ineq.Bind(&ode));
  auto d = ineq.createData();
  const Eigen::Matrix3d target =
      (Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(0.1, Eigen::Vector3d(1.0, 0.4, 0.0).normalized()))
          .toRotationMatrix();
  constexpr double kFz = 300.0;
  constexpr double kStep = 0.01;

  struct Edge {
    int row;  // 1..4 within the contact block
    double px, py;
    double in_x, in_y;  // inward unit direction
  };

  const std::array<Edge, 4> edges{{{1, 0.0, rect.y_min, 0.0, 1.0},
                                   {2, 0.0, rect.y_max, 0.0, -1.0},
                                   {3, rect.x_min, 0.0, 1.0, 0.0},
                                   {4, rect.x_max, 0.0, -1.0, 0.0}}};
  double worst_edge = 0.0;
  double worst_out = 0.0;
  double worst_tz = 0.0;
  double worst_rot = 0.0;
  for (std::size_t i = 0; i < 10; ++i) {
    const KinoSample& s = rig.samples[i];
    for (int k = 0; k < nc; ++k) {
      const pinocchio::FrameIndex fid = ode.contact_ids_[static_cast<std::size_t>(k)];
      Eigen::VectorXd x = s.x;
      x.head(model_.nq) = WithFrameRotation(model_, s.x.head(model_.nq), fid, target);
      pinocchio::Data pd(model_);
      pinocchio::framesForwardKinematics(model_, pd, x.head(model_.nq));
      const Eigen::Matrix3d rot = pd.oMf[fid].rotation();
      worst_rot = std::max(worst_rot, (rot - target).cwiseAbs().maxCoeff());
      EXPECT_LE((rot - target).cwiseAbs().maxCoeff(), 1e-12);
      double wrench_norm = 0.0;
      auto rows_at = [&](double px, double py, double tz) {
        const Eigen::Vector3d f_l(20.0, -15.0, kFz);
        const Eigen::Vector3d tau_l(py * kFz, -px * kFz, tz);
        Eigen::VectorXd u = s.u;
        u.segment<3>(k * 6) = rot * f_l;
        u.segment<3>(k * 6 + 3) = rot * tau_l;
        wrench_norm = std::max(wrench_norm, u.segment<6>(k * 6).norm());
        ineq.evaluate(x, u, *d);
        return Eigen::Matrix<double, 5, 1>(d->value_.segment<5>(k * 5));
      };
      const Eigen::Matrix<double, 5, 1> centre = rows_at(0.0, 0.0, 3.0);
      for (int r = 1; r <= 4; ++r) {
        EXPECT_LT(centre[r], 0.0) << "centre row " << r;
      }
      EXPECT_LT(centre[0], 0.0) << "inside the friction cone";
      for (const Edge& e : edges) {
        const Eigen::Matrix<double, 5, 1> on = rows_at(e.px, e.py, 3.0);
        const Eigen::Matrix<double, 5, 1> in =
            rows_at(e.px + kStep * e.in_x, e.py + kStep * e.in_y, 3.0);
        const Eigen::Matrix<double, 5, 1> out =
            rows_at(e.px - kStep * e.in_x, e.py - kStep * e.in_y, 3.0);
        worst_edge = std::max(worst_edge, std::abs(on[e.row]) / kFz);
        worst_out = std::max(worst_out, std::abs(out[e.row] - kStep * kFz) / kFz);
        EXPECT_LE(std::abs(on[e.row]), 1e-9 * kFz) << "edge row " << e.row;
        EXPECT_LT(in[e.row], 0.0) << "edge row " << e.row;
        EXPECT_GT(out[e.row], 0.0) << "edge row " << e.row;
        EXPECT_LE(std::abs(out[e.row] - kStep * kFz), 1e-9 * kFz) << "edge row " << e.row;
        for (int r = 1; r <= 4; ++r) {
          if (r == e.row) {
            continue;
          }
          EXPECT_LT(on[r], 0.0) << "edge " << e.row << " other row " << r;
          EXPECT_LT(in[r], 0.0) << "edge " << e.row << " other row " << r;
          EXPECT_LT(out[r], 0.0) << "edge " << e.row << " other row " << r;
        }
      }
      // No τ_z bound: the local yaw moment changes no row.
      wrench_norm = 0.0;
      const Eigen::Matrix<double, 5, 1> a = rows_at(0.02, -0.01, 3.0);
      const Eigen::Matrix<double, 5, 1> b = rows_at(0.02, -0.01, -40.0);
      const double dtz = (a - b).cwiseAbs().maxCoeff();
      worst_tz = std::max(worst_tz, dtz / std::max(1.0, wrench_norm));
      EXPECT_LE(dtz, 1e-12 * std::max(1.0, wrench_norm)) << "sample " << i << " contact " << k;
    }
  }
  std::printf(
      "CoP: frame rotation err %.3e, on-edge |c|/F_z %.3e, outside (c - 0.01 F_z)/F_z %.3e, "
      "tau_z change / |w| %.3e\n",
      worst_rot, worst_edge, worst_out, worst_tz);
}

// ── Frame kinematics ────────────────────────────────────────────────────────

// FrameMotionOf against pinocchio from scratch (the ODE's v̇ on a data of its
// own), and FrameTilt against Eigen's shortest-arc quaternion.
TEST_F(ContactConstraintsTest, FrameMotionAndTiltMatchIndependentKinematics) {
  Rig& rig = rig6_;
  double worst = 0.0;
  double worst_tilt = 0.0;
  for (const KinoSample& s : rig.samples) {
    const Ode ode = rig.dyn.MakeOde(s.mode).value();
    ContactKinematics kin(ode);
    kin.Forward(ode, s.x, s.u);
    auto od = ode.createData();
    ode.forward(s.x, s.u, *od);
    const Eigen::VectorXd vdot = od->xdot_.tail(model_.nv);
    pinocchio::Data pd(model_);
    pinocchio::forwardKinematics(model_, pd, s.x.head(model_.nq), s.x.tail(model_.nv), vdot);
    pinocchio::updateFramePlacements(model_, pd);
    for (const pinocchio::FrameIndex fid : ode.contact_ids_) {
      FrameMotion f;
      kin.FrameMotionOf(model_, fid, f);
      const pinocchio::Motion vel =
          pinocchio::getFrameVelocity(model_, pd, fid, pinocchio::LOCAL_WORLD_ALIGNED);
      const pinocchio::Motion acc =
          pinocchio::getFrameClassicalAcceleration(model_, pd, fid, pinocchio::LOCAL_WORLD_ALIGNED);
      Eigen::Matrix<double, 15, 1> got;
      Eigen::Matrix<double, 15, 1> exp;
      got << f.position, f.linear_velocity, f.angular_velocity, f.linear_acceleration,
          f.angular_acceleration;
      exp << pd.oMf[fid].translation(), vel.linear(), vel.angular(), acc.linear(), acc.angular();
      const double err = std::max(RelErr(got, exp), RelErr(f.rotation, pd.oMf[fid].rotation()));
      worst = std::max(worst, err);
      EXPECT_LE(err, kValueRelTol);
      for (const Eigen::Vector3d& n : {Eigen::Vector3d(Eigen::Vector3d::UnitZ()), kTiltedNormal}) {
        Eigen::Vector3d e;
        rtc::mpc::FrameTilt(f.rotation.col(2), n, ContactConstraintResidual::kMinArcNorm, e,
                            nullptr);
        const Eigen::Vector3d arc =
            -Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(f.rotation.col(2)), n).vec();
        worst_tilt = std::max(worst_tilt, (e - arc).cwiseAbs().maxCoeff());
        EXPECT_LE((e - arc).cwiseAbs().maxCoeff(), 1e-12);
      }
    }
  }
  std::printf("frame motion vs pinocchio worst rel %.3e, tilt vs shortest arc worst abs %.3e\n",
              worst, worst_tilt);
}

// FrameTilt's ∂e/∂a against central differences in a (unfloored).
TEST_F(ContactConstraintsTest, FrameTiltDerivativeMatchesCentralDifferences) {
  std::mt19937 rng(kGateSeed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  ColumnErr worst;
  int pass = 0;
  constexpr int kDraws = 250;
  for (int i = 0; i < kDraws; ++i) {
    const Eigen::Vector3d a = Eigen::Vector3d(gauss(rng), gauss(rng), gauss(rng)).normalized();
    const Eigen::Vector3d n =
        i % 2 == 0 ? Eigen::Vector3d(Eigen::Vector3d::UnitZ()) : kTiltedNormal;
    Eigen::Vector3d e;
    Eigen::Matrix3d de_da;
    rtc::mpc::FrameTilt(a, n, ContactConstraintResidual::kMinArcNorm, e, &de_da);
    Eigen::MatrixXd fd(3, 3);
    for (int j = 0; j < 3; ++j) {
      Eigen::Vector3d ap = a;
      Eigen::Vector3d am = a;
      ap[j] += kH;
      am[j] -= kH;
      Eigen::Vector3d ep;
      Eigen::Vector3d em;
      rtc::mpc::FrameTilt(ap, n, ContactConstraintResidual::kMinArcNorm, ep, nullptr);
      rtc::mpc::FrameTilt(am, n, ContactConstraintResidual::kMinArcNorm, em, nullptr);
      fd.col(j) = (ep - em) / (2.0 * kH);
    }
    const ColumnErr err = CompareBlock(Eigen::MatrixXd(de_da), fd);
    worst.Fold(err);
    pass += err.ratio <= 1.0 ? 1 : 0;
  }
  EXPECT_EQ(pass, kDraws);
  std::printf("FrameTilt de/da vs FD: worst err/tol %.3e abs %.3e\n", worst.ratio, worst.abs);
}

// ContactKinematics::FrameJacobian against central differences of the frame
// quantities (θ: log3(R(x+h)·R(x−h)ᵀ)/(2h)), each sample in its own mode.
TEST_F(ContactConstraintsTest, FrameJacobianMatchesCentralDifferences) {
  Rig& rig = rig6_;
  const Space& space = *rig.dyn.space();
  const int ndx = space.ndx();
  const int nu = rig.dyn.nu();
  constexpr int kRows = rtc::mpc::kFrameKinematicsRows;
  const std::array<const char*, 6> names{"p", "theta", "v", "omega", "a", "alpha"};
  std::array<ColumnErr, 6> gx{};
  std::array<ColumnErr, 6> gu{};
  int pass_x = 0;
  int pass_u = 0;
  for (std::size_t i = 0; i < rig.samples.size(); ++i) {
    const KinoSample& s = rig.samples[i];
    const Ode ode = rig.dyn.MakeOde(s.mode).value();
    const std::size_t nc = ode.contact_ids_.size();
    ContactKinematics kin(ode);
    kin.Forward(ode, s.x, s.u);
    kin.Derivatives(ode, s.x, s.u);
    std::vector<Eigen::MatrixXd> jx(nc, Eigen::MatrixXd(kRows, ndx));
    std::vector<Eigen::MatrixXd> ju(nc, Eigen::MatrixXd(kRows, nu));
    for (std::size_t k = 0; k < nc; ++k) {
      kin.FrameJacobian(model_, ode.contact_ids_[k], jx[k], ju[k]);
    }

    ContactKinematics fd_kin(ode);
    auto motions = [&](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
      fd_kin.Forward(ode, x, u);
      std::vector<FrameMotion> out(nc);
      for (std::size_t k = 0; k < nc; ++k) {
        fd_kin.FrameMotionOf(model_, ode.contact_ids_[k], out[k]);
      }
      return out;
    };
    auto column = [&](const std::vector<FrameMotion>& p, const std::vector<FrameMotion>& m,
                      std::size_t k) {
      Eigen::Matrix<double, kRows, 1> c;
      const Eigen::Matrix3d dr = p[k].rotation * m[k].rotation.transpose();
      c << p[k].position - m[k].position, pinocchio::log3(dr),
          p[k].linear_velocity - m[k].linear_velocity,
          p[k].angular_velocity - m[k].angular_velocity,
          p[k].linear_acceleration - m[k].linear_acceleration,
          p[k].angular_acceleration - m[k].angular_acceleration;
      return Eigen::Matrix<double, kRows, 1>(c / (2.0 * kH));
    };
    std::vector<Eigen::MatrixXd> jx_fd(nc, Eigen::MatrixXd(kRows, ndx));
    std::vector<Eigen::MatrixXd> ju_fd(nc, Eigen::MatrixXd(kRows, nu));
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(ndx);
    Eigen::VectorXd xp(s.x.size());
    Eigen::VectorXd xm(s.x.size());
    for (int j = 0; j < ndx; ++j) {
      dx[j] = kH;
      space.integrate(s.x, dx, xp);
      dx[j] = -kH;
      space.integrate(s.x, dx, xm);
      dx[j] = 0.0;
      const auto p = motions(xp, s.u);
      const auto m = motions(xm, s.u);
      for (std::size_t k = 0; k < nc; ++k) {
        jx_fd[k].col(j) = column(p, m, k);
      }
    }
    for (int j = 0; j < nu; ++j) {
      Eigen::VectorXd up = s.u;
      Eigen::VectorXd um = s.u;
      up[j] += kH;
      um[j] -= kH;
      const auto p = motions(s.x, up);
      const auto m = motions(s.x, um);
      for (std::size_t k = 0; k < nc; ++k) {
        ju_fd[k].col(j) = column(p, m, k);
      }
    }
    bool ok_x = true;
    bool ok_u = true;
    for (std::size_t k = 0; k < nc; ++k) {
      for (std::size_t g = 0; g < names.size(); ++g) {
        const auto r0 = static_cast<Eigen::Index>(3 * g);
        const ColumnErr ex =
            CompareBlock(Eigen::MatrixXd(jx[k].middleRows(r0, 3)), jx_fd[k].middleRows(r0, 3));
        const ColumnErr eu =
            CompareBlock(Eigen::MatrixXd(ju[k].middleRows(r0, 3)), ju_fd[k].middleRows(r0, 3));
        gx[g].Fold(ex);
        gu[g].Fold(eu);
        ok_x = ok_x && ex.ratio <= 1.0;
        ok_u = ok_u && eu.ratio <= 1.0;
      }
    }
    pass_x += ok_x ? 1 : 0;
    pass_u += ok_u ? 1 : 0;
    EXPECT_TRUE(ok_x) << "sample " << i << " mode " << s.mode;
    EXPECT_TRUE(ok_u) << "sample " << i << " mode " << s.mode;
  }
  EXPECT_EQ(pass_x, static_cast<int>(rig.samples.size()));
  EXPECT_EQ(pass_u, static_cast<int>(rig.samples.size()));
  for (std::size_t g = 0; g < names.size(); ++g) {
    std::printf("frame jacobian %-6s worst err/tol  x %.3e (abs %.3e)  u %.3e (abs %.3e)\n",
                names[g], gx[g].ratio, gx[g].abs, gu[g].ratio, gu[g].abs);
    RecordProperty(std::string("fd_worst_ratio_frame_x_") + names[g], std::to_string(gx[g].ratio));
    RecordProperty(std::string("fd_worst_ratio_frame_u_") + names[g], std::to_string(gu[g].ratio));
  }
}

// ── Equality values and composed Jacobians ──────────────────────────────────

void ContactConstraintsTest::RunValueGate(Rig& rig, const FootConstraintGains& g,
                                          const Eigen::Vector3d& n, const char* tag) {
  const int nc = rig.dyn.n_contacts();
  const int sr = rig.stance_rows();
  double worst_stance = 0.0;
  double worst_swing = 0.0;
  int pass = 0;
  for (std::size_t i = 0; i < rig.samples.size(); ++i) {
    const KinoSample& s = rig.samples[i];
    const Ode ode = rig.dyn.MakeOde(s.mode).value();
    ContactConstraintResidual eq(ode, g);
    ASSERT_TRUE(eq.Bind(&ode));
    std::mt19937 rng(static_cast<std::uint32_t>(i));
    const StageReference stage = DrawStage(rng, nc);
    ASSERT_TRUE(eq.SetFromStage(stage, TerrainWithNormal(n)));
    auto d = eq.createData();
    eq.evaluate(s.x, s.u, *d);

    ContactKinematics kin(ode);
    kin.Forward(ode, s.x, s.u);
    const int re = sr + 1;
    bool ok = true;
    for (int k = 0; k < nc; ++k) {
      FrameMotion f;
      kin.FrameMotionOf(model_, ode.contact_ids_[static_cast<std::size_t>(k)], f);
      const auto o = static_cast<std::size_t>(k);
      const double z_t = stage.contact_pose[7 * o + 2];
      const double vz_t = stage.contact_twist[6 * o + 2];
      const double az_t = stage.contact_acceleration[6 * o + 2];
      if (Rig::Stance(s.mode, k)) {
        Eigen::VectorXd exp(sr);
        exp[0] = g.linear_velocity_xy * f.linear_velocity.x() +
                 g.linear_acceleration_xy * f.linear_acceleration.x();
        exp[1] = g.linear_velocity_xy * f.linear_velocity.y() +
                 g.linear_acceleration_xy * f.linear_acceleration.y();
        exp[2] = g.position_z * (f.position.z() - z_t) +
                 g.linear_velocity_z * f.linear_velocity.z() +
                 g.linear_acceleration_z * f.linear_acceleration.z();
        if (sr == 6) {
          Eigen::Vector3d e;
          rtc::mpc::FrameTilt(f.rotation.col(2), n, ContactConstraintResidual::kMinArcNorm, e,
                              nullptr);
          exp.segment<3>(3) = g.orientation * e + g.angular_velocity * f.angular_velocity +
                              g.angular_acceleration * f.angular_acceleration;
        }
        const double err = RelErr(d->value_.segment(k * re, sr), exp);
        worst_stance = std::max(worst_stance, err);
        ok = ok && err <= kValueRelTol;
      } else {
        const double exp = g.position_z * (f.position.z() - z_t) +
                           g.linear_velocity_z * (f.linear_velocity.z() - vz_t) +
                           g.linear_acceleration_z * (f.linear_acceleration.z() - az_t);
        const double err = std::abs(d->value_[k * re + sr] - exp) / std::max(1.0, std::abs(exp));
        worst_swing = std::max(worst_swing, err);
        ok = ok && err <= kValueRelTol;
      }
    }
    pass += ok ? 1 : 0;
    EXPECT_TRUE(ok) << tag << " sample " << i << " mode " << s.mode;
  }
  EXPECT_EQ(pass, static_cast<int>(rig.samples.size())) << tag;
  std::printf("equality value (%s): worst rel stance %.3e swing %.3e\n", tag, worst_stance,
              worst_swing);
}

TEST_F(ContactConstraintsTest, EqualityValueIsTheGainCombinationOfTheFrameKinematics) {
  RunValueGate(rig6_, RegisteredGains(), Eigen::Vector3d::UnitZ(), "6-D registered");
  RunValueGate(rig6_, NonUnitAccelerationGains(), Eigen::Vector3d::UnitZ(), "6-D non-unit acc");
  RunValueGate(rig6_, NonUnitAccelerationGains(), kTiltedNormal, "6-D non-unit acc tilted");
  RunValueGate(rig3_, RegisteredGains(), Eigen::Vector3d::UnitZ(), "3-D registered");
  RunValueGate(rig3_, NonUnitAccelerationGains(), Eigen::Vector3d::UnitZ(), "3-D non-unit acc");
}

void ContactConstraintsTest::RunComposedJacobianGate(Rig& rig, const FootConstraintGains& g,
                                                     const Eigen::Vector3d& n, const char* tag) {
  constexpr int kRows = rtc::mpc::kFrameKinematicsRows;
  constexpr int kP = 0;
  constexpr int kTheta = 3;
  constexpr int kV = 6;
  constexpr int kW = 9;
  constexpr int kA = 12;
  constexpr int kAlpha = 15;
  const int nc = rig.dyn.n_contacts();
  const int sr = rig.stance_rows();
  const int re = sr + 1;
  const int ndx = rig.dyn.space()->ndx();
  const int nu = rig.dyn.nu();
  const int nv = model_.nv;
  double worst = 0.0;
  int pass = 0;
  for (std::size_t i = 0; i < rig.samples.size(); ++i) {
    const KinoSample& s = rig.samples[i];
    const Ode ode = rig.dyn.MakeOde(s.mode).value();
    ContactConstraintResidual eq(ode, g);
    ASSERT_TRUE(eq.Bind(&ode));
    std::mt19937 rng(static_cast<std::uint32_t>(i));
    ASSERT_TRUE(eq.SetFromStage(DrawStage(rng, nc), TerrainWithNormal(n)));
    auto d = eq.createData();
    eq.evaluate(s.x, s.u, *d);
    eq.computeJacobians(s.x, s.u, *d);

    ContactKinematics kin(ode);
    kin.Forward(ode, s.x, s.u);
    kin.Derivatives(ode, s.x, s.u);
    Eigen::MatrixXd kx(kRows, ndx);
    Eigen::MatrixXd ku(kRows, nu);
    Eigen::MatrixXd ex = Eigen::MatrixXd::Zero(eq.nr, ndx);
    Eigen::MatrixXd eu = Eigen::MatrixXd::Zero(eq.nr, nu);
    for (int k = 0; k < nc; ++k) {
      const pinocchio::FrameIndex fid = ode.contact_ids_[static_cast<std::size_t>(k)];
      kin.FrameJacobian(model_, fid, kx, ku);
      const int o = k * re;
      if (Rig::Stance(s.mode, k)) {
        for (int r = 0; r < 2; ++r) {
          ex.row(o + r) =
              g.linear_velocity_xy * kx.row(kV + r) + g.linear_acceleration_xy * kx.row(kA + r);
          eu.row(o + r) =
              g.linear_velocity_xy * ku.row(kV + r) + g.linear_acceleration_xy * ku.row(kA + r);
        }
        ex.row(o + 2) = g.position_z * kx.row(kP + 2) + g.linear_velocity_z * kx.row(kV + 2) +
                        g.linear_acceleration_z * kx.row(kA + 2);
        eu.row(o + 2) = g.position_z * ku.row(kP + 2) + g.linear_velocity_z * ku.row(kV + 2) +
                        g.linear_acceleration_z * ku.row(kA + 2);
        if (sr == 6) {
          FrameMotion f;
          kin.FrameMotionOf(model_, fid, f);
          const Eigen::Vector3d a = f.rotation.col(2);
          Eigen::Vector3d e;
          Eigen::Matrix3d de_da;
          rtc::mpc::FrameTilt(a, n, ContactConstraintResidual::kMinArcNorm, e, &de_da);
          // δa = δθ × a = −[a]× δθ.
          const Eigen::Matrix3d de_dtheta = de_da * (-Skew(a));
          ex.middleRows(o + 3, 3) = g.orientation * de_dtheta * kx.middleRows(kTheta, 3) +
                                    g.angular_velocity * kx.middleRows(kW, 3) +
                                    g.angular_acceleration * kx.middleRows(kAlpha, 3);
          eu.middleRows(o + 3, 3) = g.orientation * de_dtheta * ku.middleRows(kTheta, 3) +
                                    g.angular_velocity * ku.middleRows(kW, 3) +
                                    g.angular_acceleration * ku.middleRows(kAlpha, 3);
        }
      } else {
        ex.row(o + sr) = g.position_z * kx.row(kP + 2) + g.linear_velocity_z * kx.row(kV + 2) +
                         g.linear_acceleration_z * kx.row(kA + 2);
        eu.row(o + sr) = g.position_z * ku.row(kP + 2) + g.linear_velocity_z * ku.row(kV + 2) +
                         g.linear_acceleration_z * ku.row(kA + 2);
      }
    }
    // The θ rows have no v or u dependence.
    static_cast<void>(nv);
    const double err = std::max(RelErr(d->Jx_, ex), RelErr(d->Ju_, eu));
    worst = std::max(worst, err);
    pass += err <= kValueRelTol ? 1 : 0;
    EXPECT_LE(err, kValueRelTol) << tag << " sample " << i << " mode " << s.mode;
  }
  EXPECT_EQ(pass, static_cast<int>(rig.samples.size())) << tag;
  std::printf("equality jacobian vs composed frame jacobian (%s): worst rel %.3e\n", tag, worst);
}

TEST_F(ContactConstraintsTest, EqualityJacobianIsTheGainCombinationOfTheFrameJacobian) {
  RunComposedJacobianGate(rig6_, RegisteredGains(), Eigen::Vector3d::UnitZ(), "6-D registered");
  RunComposedJacobianGate(rig6_, NonUnitAccelerationGains(), kTiltedNormal,
                          "6-D non-unit acc tilted");
  RunComposedJacobianGate(rig3_, RegisteredGains(), Eigen::Vector3d::UnitZ(), "3-D registered");
}

// ── Finite differences ──────────────────────────────────────────────────────

namespace {

/// Worst ratios per column group: q · v (state), force · moment · q̈_j (input).
struct GroupErr {
  std::array<ColumnErr, 5> g{};

  void Fold(const Eigen::MatrixXd& jx, const Eigen::MatrixXd& jx_fd, const Eigen::MatrixXd& ju,
            const Eigen::MatrixXd& ju_fd, int nv, int nc, int fs) {
    g[0].Fold(CompareColumns(jx, jx_fd, 0, nv, kAbsTol, kRelTol));
    g[1].Fold(CompareColumns(jx, jx_fd, nv, nv, kAbsTol, kRelTol));
    for (int c = 0; c < nc; ++c) {
      g[2].Fold(CompareColumns(ju, ju_fd, c * fs, 3, kAbsTol, kRelTol));
      g[3].Fold(CompareColumns(ju, ju_fd, c * fs + 3, fs - 3, kAbsTol, kRelTol));
    }
    g[4].Fold(CompareColumns(ju, ju_fd, nc * fs, nv - 6, kAbsTol, kRelTol));
  }

  void Print(const std::string& what, const std::string& tag) const {
    const std::array<const char*, 5> names{"q", "v", "force", "moment", "qdd_j"};
    for (std::size_t i = 0; i < names.size(); ++i) {
      std::printf("%s (%s) %-7s worst err/tol %.3e  abs %.3e\n", what.c_str(), tag.c_str(),
                  names[i], g[i].ratio, g[i].abs);
      ::testing::Test::RecordProperty("fd_worst_ratio_" + what + "_" + tag + "_" + names[i],
                                      std::to_string(g[i].ratio));
    }
  }
};

}  // namespace

void ContactConstraintsTest::RunEqualityFdGate(Rig& rig, const FootConstraintGains& g,
                                               const Eigen::Vector3d& n, const std::string& tag) {
  const Space& space = *rig.dyn.space();
  const int ndx = space.ndx();
  const int nu = rig.dyn.nu();
  const int nc = rig.dyn.n_contacts();
  GroupErr groups;
  int pass_x = 0;
  int pass_u = 0;
  for (std::size_t i = 0; i < rig.samples.size(); ++i) {
    const KinoSample& s = rig.samples[i];
    const Ode ode = rig.dyn.MakeOde(s.mode).value();
    ContactConstraintResidual eq(ode, g);
    ASSERT_TRUE(eq.Bind(&ode));
    std::mt19937 rng(static_cast<std::uint32_t>(i));
    ASSERT_TRUE(eq.SetFromStage(DrawStage(rng, nc), TerrainWithNormal(n)));
    auto data = eq.createData();
    eq.evaluate(s.x, s.u, *data);
    eq.computeJacobians(s.x, s.u, *data);
    auto fd = eq.createData();
    Eigen::MatrixXd jx_fd(eq.nr, ndx);
    Eigen::MatrixXd ju_fd(eq.nr, nu);
    CentralDifferences(
        space, s.x, s.u,
        [&](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
          eq.evaluate(x, u, *fd);
          return Eigen::VectorXd(fd->value_);
        },
        jx_fd, ju_fd);
    const Eigen::MatrixXd jx = data->Jx_;
    const Eigen::MatrixXd ju = data->Ju_;
    const ColumnErr ex = CompareBlock(jx, jx_fd);
    const ColumnErr eu = CompareBlock(ju, ju_fd);
    groups.Fold(jx, jx_fd, ju, ju_fd, model_.nv, nc, rig.dyn.force_size());
    pass_x += ex.ratio <= 1.0 ? 1 : 0;
    pass_u += eu.ratio <= 1.0 ? 1 : 0;
    EXPECT_LE(ex.ratio, 1.0) << tag << " sample " << i << " mode " << s.mode << " Jx abs "
                             << ex.abs;
    EXPECT_LE(eu.ratio, 1.0) << tag << " sample " << i << " mode " << s.mode << " Ju abs "
                             << eu.abs;
  }
  EXPECT_EQ(pass_x, static_cast<int>(rig.samples.size())) << tag;
  EXPECT_EQ(pass_u, static_cast<int>(rig.samples.size())) << tag;
  groups.Print("equality", tag);
}

void ContactConstraintsTest::RunInequalityFdGate(Rig& rig, const std::string& tag) {
  const Space& space = *rig.dyn.space();
  const int ndx = space.ndx();
  const int nu = rig.dyn.nu();
  const int nc = rig.dyn.n_contacts();
  GroupErr groups;
  int pass_x = 0;
  int pass_u = 0;
  Ode ode = rig.dyn.MakeOde(0).value();
  ContactWrenchConeResidual ineq(ode, RegisteredFriction(), RegisteredRectangle());
  ASSERT_TRUE(ineq.Bind(&ode));
  auto data = ineq.createData();
  auto fd = ineq.createData();
  for (std::size_t i = 0; i < rig.samples.size(); ++i) {
    const KinoSample& s = rig.samples[i];
    // Modes with a contact in stance: 1 … n_modes − 1.
    const auto mode = static_cast<std::uint32_t>(1U + i % (rig.dyn.n_modes() - 1U));
    ASSERT_TRUE(rig.dyn.SetMode(ode, mode));
    ineq.evaluate(s.x, s.u, *data);
    ineq.computeJacobians(s.x, s.u, *data);
    Eigen::MatrixXd jx_fd(ineq.nr, ndx);
    Eigen::MatrixXd ju_fd(ineq.nr, nu);
    CentralDifferences(
        space, s.x, s.u,
        [&](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
          ineq.evaluate(x, u, *fd);
          return Eigen::VectorXd(fd->value_);
        },
        jx_fd, ju_fd);
    const Eigen::MatrixXd jx = data->Jx_;
    const Eigen::MatrixXd ju = data->Ju_;
    const ColumnErr ex = CompareBlock(jx, jx_fd);
    const ColumnErr eu = CompareBlock(ju, ju_fd);
    groups.Fold(jx, jx_fd, ju, ju_fd, model_.nv, nc, rig.dyn.force_size());
    pass_x += ex.ratio <= 1.0 ? 1 : 0;
    pass_u += eu.ratio <= 1.0 ? 1 : 0;
    EXPECT_LE(ex.ratio, 1.0) << tag << " sample " << i << " mode " << mode << " Jx abs " << ex.abs;
    EXPECT_LE(eu.ratio, 1.0) << tag << " sample " << i << " mode " << mode << " Ju abs " << eu.abs;
  }
  EXPECT_EQ(pass_x, static_cast<int>(rig.samples.size())) << tag;
  EXPECT_EQ(pass_u, static_cast<int>(rig.samples.size())) << tag;
  groups.Print("inequality", tag);
}

TEST_F(ContactConstraintsTest, InequalityJacobiansMatchCentralDifferences) {
  RunInequalityFdGate(rig6_, "6D");
}

// Beyond the registered gate: the 3-D declaration (friction rows only).
TEST_F(ContactConstraintsTest, InequalityJacobiansMatchCentralDifferences3D) {
  RunInequalityFdGate(rig3_, "3D");
}

TEST_F(ContactConstraintsTest, EqualityJacobiansMatchCentralDifferences) {
  RunEqualityFdGate(rig6_, RegisteredGains(), Eigen::Vector3d::UnitZ(), "6D_registered");
}

// Beyond the registered gate: non-unit acceleration gains on a tilted plane,
// and the 3-D declaration.
TEST_F(ContactConstraintsTest, EqualityJacobiansMatchCentralDifferencesNonUnitGainsTilted) {
  RunEqualityFdGate(rig6_, NonUnitAccelerationGains(), kTiltedNormal, "6D_nonunit_tilted");
}

TEST_F(ContactConstraintsTest, EqualityJacobiansMatchCentralDifferences3D) {
  RunEqualityFdGate(rig3_, RegisteredGains(), Eigen::Vector3d::UnitZ(), "3D_registered");
}

// ── Binding contracts ───────────────────────────────────────────────────────

TEST_F(ContactConstraintsTest, UnboundAndForeignOdes) {
  Rig& rig = rig6_;
  const Ode ode = rig.dyn.MakeOde(rig.full_stance()).value();
  ContactConstraintResidual eq(ode, RegisteredGains());
  ContactWrenchConeResidual ineq(ode, RegisteredFriction(), RegisteredRectangle());
  EXPECT_EQ(eq.bound_ode(), nullptr);
  EXPECT_EQ(ineq.bound_ode(), nullptr);
  EXPECT_THROW((void)eq.createData(), std::logic_error);
  EXPECT_THROW((void)ineq.createData(), std::logic_error);
  // The 3-D declaration is another structure.
  const Ode ode3 = rig3_.dyn.MakeOde(rig3_.full_stance()).value();
  EXPECT_FALSE(eq.Bind(&ode3));
  EXPECT_FALSE(ineq.Bind(&ode3));
  ASSERT_TRUE(eq.Bind(&ode));
  ASSERT_TRUE(ineq.Bind(&ode));
  auto de = eq.createData();
  auto di = ineq.createData();
  ASSERT_TRUE(eq.Bind(nullptr));
  ASSERT_TRUE(ineq.Bind(nullptr));
  const KinoSample& s = rig.samples[0];
  eq.evaluate(s.x, s.u, *de);
  eq.computeJacobians(s.x, s.u, *de);
  ineq.evaluate(s.x, s.u, *di);
  ineq.computeJacobians(s.x, s.u, *di);
  EXPECT_TRUE(de->value_.array().isNaN().all());
  EXPECT_TRUE(de->Jx_.array().isNaN().all());
  EXPECT_TRUE(de->Ju_.array().isNaN().all());
  EXPECT_TRUE(di->value_.array().isNaN().all());
  EXPECT_TRUE(di->Jx_.array().isNaN().all());
  EXPECT_TRUE(di->Ju_.array().isNaN().all());
}

}  // namespace
