#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/constraints/friction_cone_constraint.hpp"

namespace rtc::tsid {
namespace {

TEST(FrictionConeTest, Dimensions) {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(
      RTC_PANDA_URDF_PATH,
      *model);

  RobotModelInfo info;
  YAML::Node config;
  info.build(*model, config);

  ContactManagerConfig mgr;
  mgr.contacts.resize(2);
  mgr.contacts[0].contact_dim = 3;
  mgr.contacts[0].friction_faces = 4;
  mgr.contacts[1].contact_dim = 3;
  mgr.contacts[1].friction_faces = 8;
  mgr.max_contacts = 2;
  mgr.max_contact_vars = 6;

  ContactState cs;
  cs.init(2);
  cs.contacts[0].active = true;
  cs.contacts[1].active = true;
  cs.recompute_active(mgr);

  FrictionConeConstraint fc;
  PinocchioCache cache;
  cache.init(model, mgr);
  YAML::Node cfg;
  fc.init(*model, info, cache, cfg);
  fc.set_contact_manager(&mgr);

  EXPECT_EQ(fc.eq_dim(cs), 0);
  // Contact 0: 4+1=5, Contact 1: 8+1=9 → total = 14
  EXPECT_EQ(fc.ineq_dim(cs), 14);

  // Deactivate contact 1 → 5
  cs.contacts[1].active = false;
  cs.recompute_active(mgr);
  EXPECT_EQ(fc.ineq_dim(cs), 5);
}

TEST(FrictionConeTest, ConeMatrixStructure) {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(
      RTC_PANDA_URDF_PATH,
      *model);

  RobotModelInfo info;
  YAML::Node config;
  info.build(*model, config);

  ContactManagerConfig mgr;
  mgr.contacts.resize(1);
  mgr.contacts[0].contact_dim = 3;
  mgr.contacts[0].friction_coeff = 0.7;
  mgr.contacts[0].friction_faces = 4;
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 3;

  ContactState cs;
  cs.init(1);
  cs.contacts[0].active = true;
  cs.recompute_active(mgr);

  FrictionConeConstraint fc;
  PinocchioCache cache;
  cache.init(model, mgr);
  YAML::Node cfg;
  fc.init(*model, info, cache, cfg);
  fc.set_contact_manager(&mgr);

  const int nv = info.nv;
  const int n_vars = nv + 3;
  const int n_ineq = fc.ineq_dim(cs);  // 5
  ASSERT_EQ(n_ineq, 5);

  Eigen::MatrixXd C(n_ineq, n_vars);
  Eigen::VectorXd l(n_ineq), u(n_ineq);
  C.setZero();
  l.setZero();
  u.setZero();

  fc.compute_inequality(cache, cs, info, n_vars, C, l, u);

  // a 열 (0:nv)은 모두 0
  EXPECT_NEAR(C.leftCols(nv).norm(), 0.0, 1e-15);

  // λ 열 (nv:nv+3)은 non-zero
  EXPECT_GT(C.rightCols(3).norm(), 0.0);

  // 모든 upper bound = 0
  for (int i = 0; i < n_ineq; ++i) {
    EXPECT_DOUBLE_EQ(u(i), 0.0);
  }

  // Unilateral row (last): C[4, nv+2] = -1 (fz direction)
  EXPECT_DOUBLE_EQ(C(4, nv + 2), -1.0);

  // Cone rows: last column of λ block (fz) should be -μ = -0.7
  for (int k = 0; k < 4; ++k) {
    EXPECT_DOUBLE_EQ(C(k, nv + 2), -0.7);
  }
}

}  // namespace
}  // namespace rtc::tsid
