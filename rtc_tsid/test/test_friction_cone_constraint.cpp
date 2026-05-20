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
  pinocchio::urdf::buildModel(RTC_PANDA_URDF_PATH, *model);

  RobotModelInfo info;
  YAML::Node config;
  info.Build(*model, config);

  ContactManagerConfig mgr;
  mgr.contacts.resize(2);
  mgr.contacts[0].contact_dim = 3;
  mgr.contacts[0].friction_faces = 4;
  mgr.contacts[1].contact_dim = 3;
  mgr.contacts[1].friction_faces = 8;
  mgr.max_contacts = 2;
  mgr.max_contact_vars = 6;

  ContactState cs;
  cs.Init(2);
  cs.contacts[0].active = true;
  cs.contacts[1].active = true;
  cs.RecomputeActive(mgr);

  FrictionConeConstraint fc;
  PinocchioCache cache;
  cache.Init(model, mgr);
  YAML::Node cfg;
  fc.Init(*model, info, cache, cfg);
  fc.SetContactManager(&mgr);

  EXPECT_EQ(fc.EqDim(cs), 0);
  // Contact 0: 4+1=5, Contact 1: 8+1=9 → total = 14
  EXPECT_EQ(fc.IneqDim(cs), 14);

  // Deactivate contact 1 → 5
  cs.contacts[1].active = false;
  cs.RecomputeActive(mgr);
  EXPECT_EQ(fc.IneqDim(cs), 5);
}

TEST(FrictionConeTest, ConeMatrixStructure) {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(RTC_PANDA_URDF_PATH, *model);

  RobotModelInfo info;
  YAML::Node config;
  info.Build(*model, config);

  ContactManagerConfig mgr;
  mgr.contacts.resize(1);
  mgr.contacts[0].contact_dim = 3;
  mgr.contacts[0].friction_coeff = 0.7;
  mgr.contacts[0].friction_faces = 4;
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 3;

  ContactState cs;
  cs.Init(1);
  cs.contacts[0].active = true;
  cs.RecomputeActive(mgr);

  FrictionConeConstraint fc;
  PinocchioCache cache;
  cache.Init(model, mgr);
  YAML::Node cfg;
  fc.Init(*model, info, cache, cfg);
  fc.SetContactManager(&mgr);

  const int nv = info.nv;
  const int n_vars = nv + 3;
  const int n_ineq = fc.IneqDim(cs);  // 5
  ASSERT_EQ(n_ineq, 5);

  Eigen::MatrixXd C(n_ineq, n_vars);
  Eigen::VectorXd l(n_ineq), u(n_ineq);
  C.setZero();
  l.setZero();
  u.setZero();

  fc.ComputeInequality(cache, cs, info, n_vars, C, l, u);

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

// A-3: surface contact (cdim=6) → ineq_dim = friction_faces + 1 + 6 (CoP rect + yaw).
TEST(FrictionConeTest, SurfaceContactDimensions) {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(RTC_PANDA_URDF_PATH, *model);

  RobotModelInfo info;
  YAML::Node config;
  info.Build(*model, config);

  ContactManagerConfig mgr;
  mgr.contacts.resize(2);
  mgr.contacts[0].contact_dim = 3;  // point
  mgr.contacts[0].friction_faces = 4;
  mgr.contacts[1].contact_dim = 6;  // surface
  mgr.contacts[1].friction_faces = 4;
  mgr.contacts[1].patch_half_length_x = 0.03;
  mgr.contacts[1].patch_half_length_y = 0.03;
  mgr.contacts[1].torsional_friction_coeff = 0.02;
  mgr.max_contacts = 2;
  mgr.max_contact_vars = 9;

  ContactState cs;
  cs.Init(2);
  cs.contacts[0].active = true;
  cs.contacts[1].active = true;
  cs.RecomputeActive(mgr);

  FrictionConeConstraint fc;
  PinocchioCache cache;
  cache.Init(model, mgr);
  YAML::Node cfg;
  fc.Init(*model, info, cache, cfg);
  fc.SetContactManager(&mgr);

  // Point: 4 + 1 = 5; Surface: 4 + 1 + 6 = 11 → total 16
  EXPECT_EQ(fc.IneqDim(cs), 16);

  cs.contacts[0].active = false;
  cs.RecomputeActive(mgr);
  EXPECT_EQ(fc.IneqDim(cs), 11);  // surface only
}

// A-3: CoP rectangle row 수치 검증 — m_x bound: ±m_x - l_y·f_z ≤ 0.
TEST(FrictionConeTest, SurfaceCopRectangleMatrix) {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(RTC_PANDA_URDF_PATH, *model);

  RobotModelInfo info;
  YAML::Node config;
  info.Build(*model, config);

  ContactManagerConfig mgr;
  mgr.contacts.resize(1);
  mgr.contacts[0].contact_dim = 6;
  mgr.contacts[0].friction_coeff = 0.7;
  mgr.contacts[0].friction_faces = 4;
  mgr.contacts[0].patch_half_length_x = 0.04;
  mgr.contacts[0].patch_half_length_y = 0.03;
  mgr.contacts[0].torsional_friction_coeff = 0.0;  // yaw rows trivial
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 6;

  ContactState cs;
  cs.Init(1);
  cs.contacts[0].active = true;
  cs.RecomputeActive(mgr);

  FrictionConeConstraint fc;
  PinocchioCache cache;
  cache.Init(model, mgr);
  YAML::Node cfg;
  fc.Init(*model, info, cache, cfg);
  fc.SetContactManager(&mgr);

  const int nv = info.nv;
  const int n_vars = nv + 6;
  const int n_ineq = fc.IneqDim(cs);
  ASSERT_EQ(n_ineq, 11);  // 4 cone + 1 unilateral + 4 CoP + 2 yaw

  Eigen::MatrixXd C(n_ineq, n_vars);
  Eigen::VectorXd l(n_ineq), u(n_ineq);
  C.setZero();
  l.setZero();
  u.setZero();
  fc.ComputeInequality(cache, cs, info, n_vars, C, l, u);

  // Row layout: [0..3] cone, [4] unilateral, [5..6] CoP_x (±m_y - l_x·fz),
  // [7..8] CoP_y (±m_x - l_y·fz), [9..10] yaw (±m_z - μ_τ·fz).
  // CoP_x (m_y bound) — l_x = 0.04
  EXPECT_DOUBLE_EQ(C(5, nv + 2), -0.04);  // fz column
  EXPECT_DOUBLE_EQ(C(5, nv + 4), 1.0);    // +m_y
  EXPECT_DOUBLE_EQ(C(6, nv + 2), -0.04);
  EXPECT_DOUBLE_EQ(C(6, nv + 4), -1.0);  // -m_y
  // CoP_y (m_x bound) — l_y = 0.03
  EXPECT_DOUBLE_EQ(C(7, nv + 2), -0.03);
  EXPECT_DOUBLE_EQ(C(7, nv + 3), 1.0);  // +m_x
  EXPECT_DOUBLE_EQ(C(8, nv + 2), -0.03);
  EXPECT_DOUBLE_EQ(C(8, nv + 3), -1.0);  // -m_x
  // Yaw (μ_τ = 0) — m_z column ±1, fz column 0
  EXPECT_DOUBLE_EQ(C(9, nv + 2), 0.0);
  EXPECT_DOUBLE_EQ(C(9, nv + 5), 1.0);
  EXPECT_DOUBLE_EQ(C(10, nv + 2), 0.0);
  EXPECT_DOUBLE_EQ(C(10, nv + 5), -1.0);

  // Feasibility check via C·z (manual):
  // λ = [0, 0, fz=10, m_x=0.2, 0, 0]; lambda block starts at column nv.
  // Row 7 (m_x ≤ l_y·fz): C·z = -0.03·10 + 1·0.2 = -0.1 ≤ 0 ✓
  // Row 8 (-m_x ≤ l_y·fz): -0.03·10 + (-1)·0.2 = -0.5 ≤ 0 ✓
  Eigen::VectorXd z = Eigen::VectorXd::Zero(n_vars);
  z(nv + 2) = 10.0;  // fz
  z(nv + 3) = 0.2;   // m_x
  Eigen::VectorXd Cz = C * z;
  EXPECT_LE(Cz(7), 0.0);
  EXPECT_LE(Cz(8), 0.0);

  // Infeasible: m_x = 0.5 → Cz(7) = -0.3 + 0.5 = 0.2 > 0 (CoP exits patch)
  z(nv + 3) = 0.5;
  Cz = C * z;
  EXPECT_GT(Cz(7), 0.0);
}

// A-3: μ_τ > 0 → yaw rows have non-zero fz column.
TEST(FrictionConeTest, SurfaceYawMoment) {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(RTC_PANDA_URDF_PATH, *model);

  RobotModelInfo info;
  YAML::Node config;
  info.Build(*model, config);

  ContactManagerConfig mgr;
  mgr.contacts.resize(1);
  mgr.contacts[0].contact_dim = 6;
  mgr.contacts[0].friction_faces = 4;
  mgr.contacts[0].torsional_friction_coeff = 0.05;
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 6;

  ContactState cs;
  cs.Init(1);
  cs.contacts[0].active = true;
  cs.RecomputeActive(mgr);

  FrictionConeConstraint fc;
  PinocchioCache cache;
  cache.Init(model, mgr);
  YAML::Node cfg;
  fc.Init(*model, info, cache, cfg);
  fc.SetContactManager(&mgr);

  const int nv = info.nv;
  const int n_vars = nv + 6;
  const int n_ineq = fc.IneqDim(cs);
  Eigen::MatrixXd C(n_ineq, n_vars);
  Eigen::VectorXd l(n_ineq), u(n_ineq);
  C.setZero();
  l.setZero();
  u.setZero();
  fc.ComputeInequality(cache, cs, info, n_vars, C, l, u);

  // Yaw rows 9, 10 — fz column = -μ_τ
  EXPECT_DOUBLE_EQ(C(9, nv + 2), -0.05);
  EXPECT_DOUBLE_EQ(C(10, nv + 2), -0.05);

  // Feasibility: m_z = 0.4, fz = 10 → 0.4 - 0.5 = -0.1 ≤ 0 ✓
  Eigen::VectorXd z = Eigen::VectorXd::Zero(n_vars);
  z(nv + 2) = 10.0;
  z(nv + 5) = 0.4;
  EXPECT_LE((C * z)(9), 0.0);
  // Infeasible: m_z = 0.6 → 0.6 - 0.5 = 0.1 > 0
  z(nv + 5) = 0.6;
  EXPECT_GT((C * z)(9), 0.0);
}

// A-3: patch=0, μ_τ=0 surface → fz 열은 0 이 되고 moment 열 ±1 이 살아 m=0 강제.
// "trivial 행" 아니라 *point-like restraint* (moment 0 equality). docs 의 의도된 동작.
TEST(FrictionConeTest, SurfaceZeroPatchForcesZeroMoment) {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(RTC_PANDA_URDF_PATH, *model);

  RobotModelInfo info;
  YAML::Node config;
  info.Build(*model, config);

  ContactManagerConfig mgr;
  mgr.contacts.resize(1);
  mgr.contacts[0].contact_dim = 6;
  mgr.contacts[0].friction_faces = 4;
  // patch_half_length_x/y, torsional_friction_coeff 모두 default 0
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 6;

  ContactState cs;
  cs.Init(1);
  cs.contacts[0].active = true;
  cs.RecomputeActive(mgr);

  FrictionConeConstraint fc;
  PinocchioCache cache;
  cache.Init(model, mgr);
  YAML::Node cfg;
  fc.Init(*model, info, cache, cfg);
  fc.SetContactManager(&mgr);

  const int nv = info.nv;
  const int n_vars = nv + 6;
  const int n_ineq = fc.IneqDim(cs);
  EXPECT_EQ(n_ineq, 11);  // surface 행 자체는 유지

  Eigen::MatrixXd C(n_ineq, n_vars);
  Eigen::VectorXd l(n_ineq), u(n_ineq);
  C.setZero();
  l.setZero();
  u.setZero();
  fc.ComputeInequality(cache, cs, info, n_vars, C, l, u);

  // Rows 5..10 의 fz column 은 모두 0 (l_x=l_y=μ_τ=0)
  for (int r = 5; r <= 10; ++r) {
    EXPECT_DOUBLE_EQ(C(r, nv + 2), 0.0) << "row " << r;
  }
  // moment column ±1 패턴 유지 — 결과적으로 m=0 강제 (point-like restraint).
  EXPECT_DOUBLE_EQ(C(5, nv + 4), 1.0);
  EXPECT_DOUBLE_EQ(C(9, nv + 5), 1.0);

  // 의미 lock-in: m_z = 1e-6, f_z = 10 → row 9 (m_z ≤ μ_τ·f_z = 0) 에서 1e-6 > 0
  // 이므로 *infeasible*. 즉 patch=0 surface 는 moment 를 정확히 0 으로 강제.
  Eigen::VectorXd z = Eigen::VectorXd::Zero(n_vars);
  z(nv + 2) = 10.0;  // f_z
  z(nv + 5) = 1e-6;  // m_z tiny non-zero
  EXPECT_GT((C * z)(9), 0.0) << "patch=0 surface must reject any non-zero m_z";
  z(nv + 5) = 0.0;
  z(nv + 3) = 1e-6;  // m_x tiny
  EXPECT_GT((C * z)(7), 0.0) << "patch=0 surface must reject any non-zero m_x";
}

// A-3: mixed point + surface — λ block offset 이 정확히 진행하는지.
TEST(FrictionConeTest, MixedPointSurfaceLambdaOffsets) {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(RTC_PANDA_URDF_PATH, *model);

  RobotModelInfo info;
  YAML::Node config;
  info.Build(*model, config);

  ContactManagerConfig mgr;
  mgr.contacts.resize(2);
  mgr.contacts[0].contact_dim = 3;
  mgr.contacts[0].friction_coeff = 0.7;
  mgr.contacts[0].friction_faces = 4;
  mgr.contacts[1].contact_dim = 6;
  mgr.contacts[1].friction_coeff = 0.7;
  mgr.contacts[1].friction_faces = 4;
  mgr.contacts[1].patch_half_length_x = 0.02;
  mgr.contacts[1].patch_half_length_y = 0.02;
  mgr.contacts[1].torsional_friction_coeff = 0.01;
  mgr.max_contacts = 2;
  mgr.max_contact_vars = 9;

  ContactState cs;
  cs.Init(2);
  cs.contacts[0].active = true;
  cs.contacts[1].active = true;
  cs.RecomputeActive(mgr);

  FrictionConeConstraint fc;
  PinocchioCache cache;
  cache.Init(model, mgr);
  YAML::Node cfg;
  fc.Init(*model, info, cache, cfg);
  fc.SetContactManager(&mgr);

  const int nv = info.nv;
  const int n_vars = nv + 9;
  const int n_ineq = fc.IneqDim(cs);
  ASSERT_EQ(n_ineq, 5 + 11);  // 16

  Eigen::MatrixXd C(n_ineq, n_vars);
  Eigen::VectorXd l(n_ineq), u(n_ineq);
  C.setZero();
  l.setZero();
  u.setZero();
  fc.ComputeInequality(cache, cs, info, n_vars, C, l, u);

  // Rows 0..4: point contact — λ columns [nv .. nv+3).
  // Row 4 (point unilateral): C(4, nv+2) = -1.
  EXPECT_DOUBLE_EQ(C(4, nv + 2), -1.0);
  // Point rows must not touch surface λ columns [nv+3 .. nv+9).
  EXPECT_DOUBLE_EQ(C.block(0, nv + 3, 5, 6).norm(), 0.0);

  // Rows 5..15: surface — λ columns [nv+3 .. nv+9). fz column = nv+5.
  // Surface unilateral (row 9 inside surface = absolute row 5+4=9):
  EXPECT_DOUBLE_EQ(C(9, nv + 5), -1.0);
  // Surface rows must not touch point λ columns [nv .. nv+3).
  EXPECT_DOUBLE_EQ(C.block(5, nv, 11, 3).norm(), 0.0);
}

}  // namespace
}  // namespace rtc::tsid
