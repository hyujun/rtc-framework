#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/constraints/eom_constraint.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;

class EomConstraintTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.Build(*model_, config);

    ContactManagerConfig contact_cfg;
    contact_cfg.max_contacts = 0;
    cache_.Init(model_, rtc::tsid::ContactFrameIds(contact_cfg));

    contacts_.Init(0);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  PinocchioCache cache_;
  ContactState contacts_;
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

TEST_F(EomConstraintTest, FloatingBaseHasDimension) {
  // Override to floating-base
  YAML::Node config;
  config["floating_base"] = true;
  config["n_actuated"] = robot_info_.nv - 6;

  RobotModelInfo fb_info;
  fb_info.Build(*model_, config);

  EomConstraint eom;
  YAML::Node cfg;
  eom.Init(*model_, fb_info, cache_, cfg);

  // Floating-base → eq_dim = 6
  EXPECT_EQ(eom.EqDim(contacts_), 6);
  EXPECT_EQ(eom.IneqDim(contacts_), 0);
}

// A-2: surface(cdim=6) contact 가 활성일 때 λ block column offset 이 정확한지.
// manager_ 미주입(legacy) 시 fallback(cdim=3) 로 surface 의 moment column 이
// 잘못된 위치를 채우는 silent breakage 가 있었음 — manager_ 주입 후 회귀 방지.
TEST_F(EomConstraintTest, MixedPointSurfaceContactOffsets) {
  // Floating-base panda + 가짜 2-contact (panda_link8 frame 재사용 — frame_id
  // 중복은 EoM 산출에 무관, λ offset 만 검증).
  YAML::Node config;
  config["floating_base"] = true;
  config["n_actuated"] = robot_info_.nv - 6;

  RobotModelInfo fb_info;
  fb_info.Build(*model_, config);

  ContactManagerConfig mgr;
  mgr.contacts.resize(2);
  mgr.contacts[0].name = "point0";
  mgr.contacts[0].frame_name = "panda_link8";
  mgr.contacts[0].frame_id = static_cast<int>(model_->getFrameId("panda_link8"));
  mgr.contacts[0].contact_dim = 3;
  mgr.contacts[1].name = "surface0";
  mgr.contacts[1].frame_name = "panda_link8";
  mgr.contacts[1].frame_id = static_cast<int>(model_->getFrameId("panda_link8"));
  mgr.contacts[1].contact_dim = 6;
  mgr.max_contacts = 2;
  mgr.max_contact_vars = 9;

  PinocchioCache cache;
  cache.Init(model_, rtc::tsid::ContactFrameIds(mgr));

  ContactState cs;
  cs.Init(2);
  cs.contacts[0].active = true;
  cs.contacts[1].active = true;
  cs.RecomputeActive(mgr);

  EomConstraint eom;
  YAML::Node cfg;
  eom.Init(*model_, fb_info, cache, cfg);
  eom.SetContactManager(&mgr);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(fb_info.nv);
  cache.Update(q, v);

  const int n_eq = eom.EqDim(cs);    // 6
  const int n_vars = fb_info.nv + 9;  // 6 + n_actuated + 3 + 6
  Eigen::MatrixXd A(n_eq, n_vars);
  Eigen::VectorXd b(n_eq);
  A.setZero();
  b.setZero();

  eom.ComputeEquality(cache, cs, fb_info, n_vars, A, b);

  // Hand-computed reference — A.block(0, nv+offset_i, n_unactuated, cdim_i)
  //   = -Jc_i.topRows(cdim_i).leftCols(n_unactuated).transpose()
  // n_unactuated = 6 (floating-base). 같은 frame 두 contact 라 Jc 동일.
  const int nu = n_eq;                         // 6
  const auto& Jc = cache.contact_frames[0].J;  // contact 0 (= contact 1 동일 frame)
  Eigen::MatrixXd expected_point = -Jc.topRows(3).leftCols(nu).transpose();
  Eigen::MatrixXd expected_surface = -Jc.topRows(6).leftCols(nu).transpose();

  // Point contact λ block: columns [nv .. nv+3) — 정확한 값 비교 (norm-only 강화).
  EXPECT_TRUE(A.block(0, fb_info.nv, nu, 3).isApprox(expected_point, 1e-10))
      << "Point contact λ block must match -Jc[:3].T.topRows(6)";

  // Surface contact λ block: columns [nv+3 .. nv+9) — 6 columns 모두 정확값.
  // 회귀: cdim=3 하드코딩이면 마지막 3 columns 가 0; transposed/negated 면 isApprox 실패.
  EXPECT_TRUE(A.block(0, fb_info.nv + 3, nu, 6).isApprox(expected_surface, 1e-10))
      << "Surface contact λ block must match -Jc[:6].T.topRows(6)";

  // 명시적 silent-miss sentinel: 마지막 3 columns (surface moment) norm > 0.
  EXPECT_GT(A.block(0, fb_info.nv + 6, nu, 3).norm(), 0.0)
      << "Surface moment columns must be non-zero — cdim=3 hardcode regression";
}

// A-2: manager_ 미주입(legacy) 시 point(cdim=3) fallback 으로 crash 없이 동작.
TEST_F(EomConstraintTest, ManagerNullGuardPointFallback) {
  YAML::Node config;
  config["floating_base"] = true;
  config["n_actuated"] = robot_info_.nv - 6;

  RobotModelInfo fb_info;
  fb_info.Build(*model_, config);

  ContactManagerConfig mgr;
  mgr.contacts.resize(1);
  mgr.contacts[0].name = "p0";
  mgr.contacts[0].frame_name = "panda_link8";
  mgr.contacts[0].frame_id = static_cast<int>(model_->getFrameId("panda_link8"));
  mgr.contacts[0].contact_dim = 3;
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 3;

  PinocchioCache cache;
  cache.Init(model_, rtc::tsid::ContactFrameIds(mgr));
  ContactState cs;
  cs.Init(1);
  cs.contacts[0].active = true;
  cs.RecomputeActive(mgr);

  EomConstraint eom;
  YAML::Node cfg;
  eom.Init(*model_, fb_info, cache, cfg);
  // set_contact_manager 호출 의도적 생략 — legacy 회로 시뮬레이션.

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(fb_info.nv);
  cache.Update(q, v);

  const int n_eq = eom.EqDim(cs);
  const int n_vars = fb_info.nv + 3;
  Eigen::MatrixXd A(n_eq, n_vars);
  Eigen::VectorXd b(n_eq);
  A.setZero();
  b.setZero();

  // Crash 없이 동작 + point block 채워짐.
  eom.ComputeEquality(cache, cs, fb_info, n_vars, A, b);
  EXPECT_GT(A.block(0, fb_info.nv, n_eq, 3).norm(), 0.0);
}

TEST_F(EomConstraintTest, FloatingBaseMatrixDimensions) {
  YAML::Node config;
  config["floating_base"] = true;
  config["n_actuated"] = robot_info_.nv - 6;

  RobotModelInfo fb_info;
  fb_info.Build(*model_, config);

  EomConstraint eom;
  YAML::Node cfg;
  eom.Init(*model_, fb_info, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  const int n_eq = eom.EqDim(contacts_);
  const int n_vars = robot_info_.nv;  // no contacts

  Eigen::MatrixXd A(n_eq, n_vars);
  Eigen::VectorXd b(n_eq);
  A.setZero();
  b.setZero();

  eom.ComputeEquality(cache_, contacts_, fb_info, n_vars, A, b);

  // A should be non-zero (P*M)
  EXPECT_GT(A.norm(), 0.0);
  // b should be non-zero (gravity)
  EXPECT_GT(b.norm(), 0.0);
}

}  // namespace
}  // namespace rtc::tsid
