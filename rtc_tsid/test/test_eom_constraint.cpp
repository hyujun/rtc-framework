#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include "g1_fixture.hpp"
#include "panda_fixture.hpp"
#include "rtc_tsid/constraints/eom_constraint.hpp"

namespace rtc::tsid {
namespace {

class EomConstraintTest : public test::PandaNoContactTest {};

// The floating cases run on the G1 free-flyer: since the floating base is the
// root joint's type, a fixed model can no longer be declared floating in YAML.
class FloatingEomTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = test::SharedG1Model();
    info_.Build(*model_, YAML::Node{});
    q_ = pinocchio::neutral(*model_);
    v_ = Eigen::VectorXd::Zero(model_->nv);
  }

  /// Point contact on the left foot, surface contact on the right.
  ContactManagerConfig PointAndSurface() const {
    ContactManagerConfig mgr;
    mgr.contacts.push_back(test::G1FootContact(*model_, test::kG1LeftFoot));
    mgr.contacts.back().contact_dim = 3;
    mgr.contacts.push_back(test::G1FootContact(*model_, test::kG1RightFoot));
    mgr.max_contacts = 2;
    mgr.max_contact_vars = 9;
    return mgr;
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo info_;
  Eigen::VectorXd q_, v_;
};

TEST_F(EomConstraintTest, FixedBaseNoDimension) {
  EomConstraint eom;
  YAML::Node cfg;
  eom.Init(*model_, robot_info_, cache_, cfg);

  // Fixed-base → eq_dim = 0
  EXPECT_EQ(eom.EqDim(contacts_), 0);
  EXPECT_EQ(eom.IneqDim(contacts_), 0);
  EXPECT_EQ(eom.Name(), "eom");
}

TEST_F(FloatingEomTest, FloatingBaseHasDimension) {
  ASSERT_TRUE(info_.floating_base);
  PinocchioCache cache;
  cache.Init(model_, {});
  ContactState cs;
  cs.Init(0);

  EomConstraint eom;
  eom.Init(*model_, info_, cache, YAML::Node{});

  // Floating-base → eq_dim = 6
  EXPECT_EQ(eom.EqDim(cs), 6);
  EXPECT_EQ(eom.IneqDim(cs), 0);
}

// A-2: surface(cdim=6) contact 가 활성일 때 λ block column offset 이 정확한지.
// manager_ 미주입(legacy) 시 fallback(cdim=3) 로 surface 의 moment column 이
// 잘못된 위치를 채우는 silent breakage 가 있었음 — manager_ 주입 후 회귀 방지.
TEST_F(FloatingEomTest, MixedPointSurfaceContactOffsets) {
  const ContactManagerConfig mgr = PointAndSurface();
  PinocchioCache cache;
  cache.Init(model_, rtc::tsid::ContactFrameIds(mgr));

  ContactState cs;
  cs.Init(2);
  cs.contacts[0].active = true;
  cs.contacts[1].active = true;
  cs.RecomputeActive(mgr);

  EomConstraint eom;
  eom.Init(*model_, info_, cache, YAML::Node{});
  eom.SetContactManager(&mgr);
  cache.Update(q_, v_);

  const int n_eq = eom.EqDim(cs);   // 6
  const int n_vars = info_.nv + 9;  // nv + 3 + 6
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_eq, n_vars);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n_eq);
  eom.ComputeEquality(cache, cs, info_, n_vars, A, b);

  // Hand-computed reference — A.block(0, nv+offset_i, n_unactuated, cdim_i)
  //   = -Jc_i.topRows(cdim_i).leftCols(n_unactuated).transpose()
  const int nu = n_eq;
  Eigen::MatrixXd expected_point = -cache.contact_frames[0].J.topRows(3).leftCols(nu).transpose();
  Eigen::MatrixXd expected_surface = -cache.contact_frames[1].J.topRows(6).leftCols(nu).transpose();

  // Point contact λ block: columns [nv .. nv+3).
  EXPECT_TRUE(A.block(0, info_.nv, nu, 3).isApprox(expected_point, 1e-10))
      << "Point contact λ block must match -Jc[:3].T.topRows(6)";
  // Surface contact λ block: columns [nv+3 .. nv+9) — 6 columns 모두 정확값.
  // 회귀: cdim=3 하드코딩이면 마지막 3 columns 가 0; transposed/negated 면 isApprox 실패.
  EXPECT_TRUE(A.block(0, info_.nv + 3, nu, 6).isApprox(expected_surface, 1e-10))
      << "Surface contact λ block must match -Jc[:6].T.topRows(6)";
  // 명시적 silent-miss sentinel: 마지막 3 columns (surface moment) norm > 0.
  EXPECT_GT(A.block(0, info_.nv + 6, nu, 3).norm(), 0.0)
      << "Surface moment columns must be non-zero — cdim=3 hardcode regression";
}

// A-2: manager_ 미주입(legacy) 시 point(cdim=3) fallback 으로 crash 없이 동작.
TEST_F(FloatingEomTest, ManagerNullGuardPointFallback) {
  ContactManagerConfig mgr;
  mgr.contacts.push_back(test::G1FootContact(*model_, test::kG1LeftFoot));
  mgr.contacts.back().contact_dim = 3;
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 3;

  PinocchioCache cache;
  cache.Init(model_, rtc::tsid::ContactFrameIds(mgr));
  ContactState cs;
  cs.Init(1);
  cs.contacts[0].active = true;
  cs.RecomputeActive(mgr);

  EomConstraint eom;
  eom.Init(*model_, info_, cache, YAML::Node{});
  // SetContactManager 호출 의도적 생략 — legacy 회로 시뮬레이션.
  cache.Update(q_, v_);

  const int n_eq = eom.EqDim(cs);
  const int n_vars = info_.nv + 3;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_eq, n_vars);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n_eq);

  // Crash 없이 동작 + point block 채워짐.
  eom.ComputeEquality(cache, cs, info_, n_vars, A, b);
  EXPECT_GT(A.block(0, info_.nv, n_eq, 3).norm(), 0.0);
}

// Init refuses a floating RobotModelInfo the base rows would be wrong for.
TEST_F(FloatingEomTest, InitRejectsWhatTheBaseRowsDoNotDescribe) {
  PinocchioCache cache;
  cache.Init(model_, {});
  EomConstraint eom;
  EXPECT_NO_THROW(eom.Init(*model_, info_, cache, YAML::Node{}));  // control

  RobotModelInfo passive = info_;  // an actuated joint made passive in S
  passive.S(0, 6) = 0.0;
  EXPECT_THROW(eom.Init(*model_, passive, cache, YAML::Node{}), std::invalid_argument);

  RobotModelInfo shifted = info_;  // S = [I | 0]: the base taken as actuated
  shifted.S.setZero();
  shifted.S.leftCols(info_.n_actuated).setIdentity();
  EXPECT_THROW(eom.Init(*model_, shifted, cache, YAML::Node{}), std::invalid_argument);

  // A RobotModelInfo built for another model (nv differs).
  RobotModelInfo panda;
  panda.Build(*test::SharedPandaModel(), YAML::Node{});
  EXPECT_THROW(eom.Init(*model_, panda, cache, YAML::Node{}), std::invalid_argument);

  // Floating with S = [0 | I] but on a composite root of the same size.
  const auto composite = test::CompositeRootPandaModel();
  RobotModelInfo fake;
  fake.Build(*composite, YAML::Node{});
  fake.floating_base = true;
  fake.n_actuated = composite->nv - 6;
  fake.S.setZero(fake.n_actuated, composite->nv);
  fake.S.rightCols(fake.n_actuated).setIdentity();
  PinocchioCache composite_cache;
  composite_cache.Init(composite, {});
  EXPECT_THROW(eom.Init(*composite, fake, composite_cache, YAML::Node{}), std::invalid_argument);
}

TEST_F(FloatingEomTest, FloatingBaseMatrixDimensions) {
  PinocchioCache cache;
  cache.Init(model_, {});
  ContactState cs;
  cs.Init(0);

  EomConstraint eom;
  eom.Init(*model_, info_, cache, YAML::Node{});
  cache.Update(q_, v_);

  const int n_eq = eom.EqDim(cs);
  const int n_vars = info_.nv;  // no contacts
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_eq, n_vars);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n_eq);
  eom.ComputeEquality(cache, cs, info_, n_vars, A, b);

  // The base rows of the EoM: A = M[0:6, :], b = −h[0:6] (gravity on the base).
  EXPECT_GT(A.norm(), 0.0);
  EXPECT_GT(b.norm(), 0.0);
  EXPECT_TRUE(A.isApprox(cache.M.topRows(6), 1e-12));
  EXPECT_TRUE(b.isApprox(-cache.h.head(6), 1e-12));
}

}  // namespace
}  // namespace rtc::tsid
