#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic pop

#include "box_test_util.hpp"
#include "panda_fixture.hpp"
#include "rtc_tsid/constraints/friction_cone_constraint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace rtc::tsid {
namespace {

// Fixture: build the Panda model once per test in SetUp(), shared via model_ /
// info_. Each TEST_F gets a fresh fixture instance, so behaviour is identical
// to the previous per-TEST inline build — only the boilerplate is removed.
class FrictionConeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = test::LoadPandaModel();
    YAML::Node config;
    info_.Build(*model_, config);
  }

  std::shared_ptr<pinocchio::Model> model_;
  RobotModelInfo info_;
};

TEST_F(FrictionConeTest, Dimensions) {
  auto& model = model_;
  RobotModelInfo& info = info_;

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
  cache.Init(model, rtc::tsid::ContactFrameIds(mgr));
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

TEST_F(FrictionConeTest, ConeMatrixStructure) {
  auto& model = model_;
  RobotModelInfo& info = info_;

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
  cache.Init(model, rtc::tsid::ContactFrameIds(mgr));
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

  test::ExpectBoxNonEmpty(l, u);

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
TEST_F(FrictionConeTest, SurfaceContactDimensions) {
  auto& model = model_;
  RobotModelInfo& info = info_;

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
  cache.Init(model, rtc::tsid::ContactFrameIds(mgr));
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
TEST_F(FrictionConeTest, SurfaceCopRectangleMatrix) {
  auto& model = model_;
  RobotModelInfo& info = info_;

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
  cache.Init(model, rtc::tsid::ContactFrameIds(mgr));
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
  test::ExpectBoxNonEmpty(l, u);

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
TEST_F(FrictionConeTest, SurfaceYawMoment) {
  auto& model = model_;
  RobotModelInfo& info = info_;

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
  cache.Init(model, rtc::tsid::ContactFrameIds(mgr));
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
  test::ExpectBoxNonEmpty(l, u);

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
TEST_F(FrictionConeTest, SurfaceZeroPatchForcesZeroMoment) {
  auto& model = model_;
  RobotModelInfo& info = info_;

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
  cache.Init(model, rtc::tsid::ContactFrameIds(mgr));
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
  test::ExpectBoxNonEmpty(l, u);

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
TEST_F(FrictionConeTest, MixedPointSurfaceLambdaOffsets) {
  auto& model = model_;
  RobotModelInfo& info = info_;

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
  cache.Init(model, rtc::tsid::ContactFrameIds(mgr));
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
  test::ExpectBoxNonEmpty(l, u);

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

// ════════════════════════════════════════════════
// Stage A-4: normal-aware cone rotation
// ════════════════════════════════════════════════

// A-4: normal = (0,0,1) explicit → cone matrix byte-identical to default-init.
// Validates the +Z short-circuit path stays regression-safe.
TEST_F(FrictionConeTest, NormalDefaultsToWorldZ) {
  auto& model = model_;
  RobotModelInfo& info = info_;

  ContactManagerConfig mgr;
  mgr.contacts.resize(1);
  mgr.contacts[0].contact_dim = 3;
  mgr.contacts[0].friction_coeff = 0.5;
  mgr.contacts[0].friction_faces = 4;
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 3;

  ContactState cs_default;
  cs_default.Init(1);
  cs_default.contacts[0].active = true;
  // Normal stays at default (0,0,1).
  cs_default.RecomputeActive(mgr);

  ContactState cs_explicit;
  cs_explicit.Init(1);
  cs_explicit.contacts[0].active = true;
  cs_explicit.contacts[0].normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  cs_explicit.RecomputeActive(mgr);

  FrictionConeConstraint fc;
  PinocchioCache cache;
  cache.Init(model, rtc::tsid::ContactFrameIds(mgr));
  YAML::Node cfg;
  fc.Init(*model, info, cache, cfg);
  fc.SetContactManager(&mgr);

  const int nv = info.nv;
  const int n_vars = nv + 3;
  const int n_ineq = fc.IneqDim(cs_default);

  Eigen::MatrixXd C1(n_ineq, n_vars);
  Eigen::MatrixXd C2(n_ineq, n_vars);
  Eigen::VectorXd l(n_ineq);
  Eigen::VectorXd u(n_ineq);
  C1.setZero();
  C2.setZero();
  fc.ComputeInequality(cache, cs_default, info, n_vars, C1, l, u);
  test::ExpectBoxNonEmpty(l, u);
  fc.ComputeInequality(cache, cs_explicit, info, n_vars, C2, l, u);
  test::ExpectBoxNonEmpty(l, u);

  // Byte-equal: short-circuit must produce identical matrices.
  EXPECT_LT((C1 - C2).norm(), 1e-15);
}

// A-4: normal tilted around X axis by θ=30° → rotated cone columns match
// the analytical R_c · row_local prediction.
TEST_F(FrictionConeTest, NormalAwareTiltedZ) {
  auto& model = model_;
  RobotModelInfo& info = info_;

  ContactManagerConfig mgr;
  mgr.contacts.resize(1);
  mgr.contacts[0].contact_dim = 3;
  mgr.contacts[0].friction_coeff = 0.5;
  mgr.contacts[0].friction_faces = 4;
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 3;

  // Normal tilted 30° from +Z around X axis: n = (0, sin θ, cos θ).
  constexpr double kTheta = M_PI / 6.0;  // 30°
  const Eigen::Vector3d n_world(0.0, std::sin(kTheta), std::cos(kTheta));

  ContactState cs;
  cs.Init(1);
  cs.contacts[0].active = true;
  cs.contacts[0].normal = n_world;
  cs.RecomputeActive(mgr);

  FrictionConeConstraint fc;
  PinocchioCache cache;
  cache.Init(model, rtc::tsid::ContactFrameIds(mgr));
  YAML::Node cfg;
  fc.Init(*model, info, cache, cfg);
  fc.SetContactManager(&mgr);

  const int nv = info.nv;
  const int n_vars = nv + 3;
  const int n_ineq = fc.IneqDim(cs);
  ASSERT_EQ(n_ineq, 5);  // 4 cone faces + 1 unilateral

  Eigen::MatrixXd C(n_ineq, n_vars);
  Eigen::VectorXd l(n_ineq);
  Eigen::VectorXd u(n_ineq);
  C.setZero();
  fc.ComputeInequality(cache, cs, info, n_vars, C, l, u);
  test::ExpectBoxNonEmpty(l, u);

  // Independent R_c computation: same Gram-Schmidt the constraint uses.
  // Seed = +X since |n.x|=0 < 0.9.
  Eigen::Vector3d t1 = Eigen::Vector3d::UnitX();
  t1 = t1 - n_world * t1.dot(n_world);
  t1.normalize();
  const Eigen::Vector3d t2 = n_world.cross(t1);
  Eigen::Matrix3d R_c;
  R_c.col(0) = t1;
  R_c.col(1) = t2;
  R_c.col(2) = n_world;

  // Each cone row: world coeffs = R_c · [cos θk, sin θk, -μ]ᵀ.
  for (int k = 0; k < 4; ++k) {
    const double phi = 2.0 * M_PI * static_cast<double>(k) / 4.0;
    const Eigen::Vector3d row_local(std::cos(phi), std::sin(phi), -0.5);
    const Eigen::Vector3d row_world = R_c * row_local;
    EXPECT_NEAR(C(k, nv + 0), row_world.x(), 1e-12) << "cone row " << k << " fx";
    EXPECT_NEAR(C(k, nv + 1), row_world.y(), 1e-12) << "cone row " << k << " fy";
    EXPECT_NEAR(C(k, nv + 2), row_world.z(), 1e-12) << "cone row " << k << " fz";
  }
  // Unilateral row: world coeffs = -n.
  EXPECT_NEAR(C(4, nv + 0), -n_world.x(), 1e-12);
  EXPECT_NEAR(C(4, nv + 1), -n_world.y(), 1e-12);
  EXPECT_NEAR(C(4, nv + 2), -n_world.z(), 1e-12);

  // Feasibility check: a force purely along +n_world with magnitude 10
  // should satisfy all rows (cone interior since μ > 0).
  Eigen::VectorXd z = Eigen::VectorXd::Zero(n_vars);
  z(nv + 0) = 10.0 * n_world.x();
  z(nv + 1) = 10.0 * n_world.y();
  z(nv + 2) = 10.0 * n_world.z();
  const Eigen::VectorXd Cz = C * z;
  for (int k = 0; k < 4; ++k) {
    EXPECT_LE(Cz(k), 1e-9) << "tilted-normal cone row " << k << " rejects pure normal force";
  }
  EXPECT_LE(Cz(4), 1e-9) << "unilateral rejects pure normal force";

  // Infeasibility: force *opposite* to n_world must violate unilateral.
  z(nv + 0) = -10.0 * n_world.x();
  z(nv + 1) = -10.0 * n_world.y();
  z(nv + 2) = -10.0 * n_world.z();
  EXPECT_GT((C * z)(4), 0.0) << "anti-normal force must violate unilateral";
}

// A-4: ContactState::UpdateNormal LPF — pushing a step normal converges
// monotonically toward target and stays unit-norm.
TEST_F(FrictionConeTest, NormalLowPassConvergence) {
  ContactManagerConfig mgr;
  mgr.contacts.resize(1);
  mgr.contacts[0].contact_dim = 3;
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 3;

  ContactState cs;
  cs.Init(1);
  cs.SeedNormals(mgr);
  ASSERT_NEAR(cs.contacts[0].normal.z(), 1.0, 1e-15) << "seed must be +Z";

  // Target normal tilted 45° around Y axis.
  const Eigen::Vector3d n_target = Eigen::Vector3d(std::sin(M_PI / 4.0), 0.0, std::cos(M_PI / 4.0));

  double prev_err = (cs.contacts[0].normal - n_target).norm();
  for (int k = 0; k < 200; ++k) {
    cs.UpdateNormal(0, n_target, /*alpha=*/0.1);
    // Unit norm preserved.
    EXPECT_NEAR(cs.contacts[0].normal.norm(), 1.0, 1e-9);
    // Monotonic convergence on a constant-target LPF: each step reduces error.
    const double err = (cs.contacts[0].normal - n_target).norm();
    EXPECT_LE(err, prev_err + 1e-12) << "LPF must not diverge at step " << k;
    prev_err = err;
  }
  // After 200 steps at α=0.1 the error should be well under 1e-3.
  EXPECT_LT((cs.contacts[0].normal - n_target).norm(), 1e-3);

  // Degenerate input is a no-op.
  const Eigen::Vector3d n_before = cs.contacts[0].normal;
  cs.UpdateNormal(0, Eigen::Vector3d::Zero(), 0.1);
  EXPECT_LT((cs.contacts[0].normal - n_before).norm(), 1e-15);

  // Out-of-range idx is a no-op.
  cs.UpdateNormal(-1, n_target, 0.1);
  cs.UpdateNormal(99, n_target, 0.1);
  EXPECT_LT((cs.contacts[0].normal - n_before).norm(), 1e-15);
}

// ── Surface CoP axes follow the contact frame; normal-force bounds ──────────
//
// One surface contact, n = +Z, lx 0.12 ≠ ly 0.05, μ_τ 0.5, 4 faces. The cache's
// contact placement is set by hand (no FK): a foot yawed +90°, or not.
// Rows (11): [0..3] pyramid, [4] unilateral −f_n, [5,6] ±m_y − lx·f_n,
// [7,8] ±m_x − ly·f_n, [9,10] ±m_z − μ_τ·f_n (all in contact-frame axes).
class SurfaceFrameTest : public FrictionConeTest {
 protected:
  void SetUp() override {
    FrictionConeTest::SetUp();
    mgr_.contacts.resize(1);
    mgr_.contacts[0].contact_dim = 6;
    mgr_.contacts[0].friction_coeff = 0.7;
    mgr_.contacts[0].friction_faces = 4;
    mgr_.contacts[0].patch_half_length_x = 0.12;
    mgr_.contacts[0].patch_half_length_y = 0.05;
    mgr_.contacts[0].torsional_friction_coeff = 0.5;
    mgr_.max_contacts = 1;
    mgr_.max_contact_vars = 6;
    cs_.Init(1);
    cs_.contacts[0].active = true;
    cs_.RecomputeActive(mgr_);
    cache_.Init(model_, rtc::tsid::ContactFrameIds(mgr_));
    fc_.Init(*model_, info_, cache_, YAML::Node{});
    fc_.SetContactManager(&mgr_);
  }

  void SetFrameRotation(const Eigen::Matrix3d& R) {
    cache_.contact_frames[0].oMf = pinocchio::SE3(R, Eigen::Vector3d::Zero());
  }

  void Assemble() {
    const int n_vars = info_.nv + 6;
    C_.setZero(11, n_vars);
    l_.setZero(11);
    u_.setZero(11);
    ASSERT_EQ(fc_.IneqDim(cs_), 11);
    fc_.ComputeInequality(cache_, cs_, info_, n_vars, C_, l_, u_);
  }

  // C·z for z = [0_a | λ_world]
  [[nodiscard]] Eigen::VectorXd Rows(const Eigen::Matrix<double, 6, 1>& lambda_world) const {
    Eigen::VectorXd z = Eigen::VectorXd::Zero(info_.nv + 6);
    z.tail(6) = lambda_world;
    return C_ * z;
  }

  // Largest row excess over [l, u] (> 0 = violated).
  [[nodiscard]] double Excess(const Eigen::Matrix<double, 6, 1>& lambda_world) const {
    const Eigen::VectorXd r = Rows(lambda_world);
    return std::max((r - u_).maxCoeff(), (l_ - r).maxCoeff());
  }

  static Eigen::Matrix<double, 6, 1> Wrench(double fx, double fy, double fz, double mx, double my,
                                            double mz) {
    Eigen::Matrix<double, 6, 1> w;
    w << fx, fy, fz, mx, my, mz;
    return w;
  }

  ContactManagerConfig mgr_;
  ContactState cs_;
  PinocchioCache cache_;
  FrictionConeConstraint fc_;
  Eigen::MatrixXd C_;
  Eigen::VectorXd l_, u_;
};

// CoP x = +0.10 along the foot (inside lx 0.12): λ_local = [0,0,100, 0,−10,0];
// in world axes with the foot yawed +90° that is [0,0,100, 10,0,0].
TEST_F(SurfaceFrameTest, TheCopRectangleTurnsWithAYawedFoot) {
  const Eigen::Matrix3d Rz90 =
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  SetFrameRotation(Rz90);
  Assemble();
  const auto w = Wrench(0, 0, 100, 10, 0, 0);
  EXPECT_LE(Excess(w), 1e-12);
  // Margin 2 N·m on the foot's x rows: m_y_local = −10, lx·f_n = 12.
  EXPECT_NEAR(Rows(w)(6), -2.0, 1e-12);

  // The same world wrench on a foot that is NOT yawed: its y half-length 0.05
  // is now along the world x moment → |m_x| 10 > 5, violated by 5.
  SetFrameRotation(Eigen::Matrix3d::Identity());
  Assemble();
  EXPECT_NEAR(Excess(w), 5.0, 1e-12);
}

// The tangent basis read back from the rows is right-handed and orthonormal:
// rows 7 / 5 carry t1 / t2 in the moment columns, row 4 carries −n.
TEST_F(SurfaceFrameTest, TheBasisIsRightHandedAndTiltsWithTheNormal) {
  SetFrameRotation(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix());
  cs_.contacts[0].normal = Eigen::Vector3d(0.2, -0.1, 1.0).normalized();
  Assemble();
  const int c = info_.nv;
  Eigen::Matrix3d R;
  R.col(0) = C_.block<1, 3>(7, c + 3).transpose();
  R.col(1) = C_.block<1, 3>(5, c + 3).transpose();
  R.col(2) = -C_.block<1, 3>(4, c).transpose();
  EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
  EXPECT_TRUE((R.transpose() * R).isIdentity(1e-12));
  EXPECT_LT((R.col(2) - cs_.contacts[0].normal).norm(), 1e-12);
  // t1 is the frame's x projected on the plane ⊥ n.
  const Eigen::Vector3d x = cache_.contact_frames[0].oMf.rotation().col(0);
  const Eigen::Vector3d t1 = (x - R.col(2) * x.dot(R.col(2))).normalized();
  EXPECT_LT((R.col(0) - t1).norm(), 1e-12);
}

// A frame whose x is (nearly) the normal: its y is projected instead.
TEST_F(SurfaceFrameTest, AFrameXAlongTheNormalFallsBackToItsY) {
  const Eigen::Matrix3d R_f =
      Eigen::AngleAxisd(-M_PI / 2 + 0.01, Eigen::Vector3d::UnitY()).toRotationMatrix();
  ASSERT_GT(R_f.col(0).z(), 0.99);
  SetFrameRotation(R_f);
  Assemble();
  const int c = info_.nv;
  Eigen::Matrix3d R;
  R.col(0) = C_.block<1, 3>(7, c + 3).transpose();
  R.col(1) = C_.block<1, 3>(5, c + 3).transpose();
  R.col(2) = -C_.block<1, 3>(4, c).transpose();
  EXPECT_TRUE(R.allFinite());
  EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
  const Eigen::Vector3d y = R_f.col(1);
  EXPECT_LT((R.col(1) - (y - R.col(2) * y.dot(R.col(2))).normalized()).norm(), 1e-12);
}

// Either side of the switch: a frame x whose projection ⊥ n is 0.11 long is
// used (t1 along it); at 0.09 the frame's y is used instead.
TEST_F(SurfaceFrameTest, TheFallbackSwitchesAtATenthOfTheFrameX) {
  const int c = info_.nv;
  for (const double proj : {0.11, 0.09}) {
    // Frame x tilted towards +Z so that its horizontal part has length proj.
    const double pitch = -std::acos(proj);  // about y: x → (cos, 0, −sin(pitch))
    const Eigen::Matrix3d R_f =
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    ASSERT_NEAR(std::hypot(R_f(0, 0), R_f(1, 0)), proj, 1e-12);
    SetFrameRotation(R_f);
    Assemble();
    const Eigen::Vector3d t1 = C_.block<1, 3>(7, c + 3).transpose();
    if (proj > 0.1) {
      EXPECT_LT((t1 - Eigen::Vector3d::UnitX()).norm(), 1e-12) << "x projection used";
    } else {
      // The y rule; with this frame (y = +Y) it also gives +X — the roll case
      // below is where the two rules differ.
      EXPECT_LT((t1 - Eigen::Vector3d::UnitY().cross(Eigen::Vector3d::UnitZ())).norm(), 1e-12);
    }
  }
  // At 0.09 with the frame rolled about its x (y leaves the world Y axis),
  // t1 follows y × n, not x's projection.
  const Eigen::Matrix3d R_f = (Eigen::AngleAxisd(-std::acos(0.09), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitX()))
                                  .toRotationMatrix();
  SetFrameRotation(R_f);
  Assemble();
  const Eigen::Vector3d n = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d y_p = (R_f.col(1) - n * R_f.col(1).dot(n)).normalized();
  const Eigen::Vector3d x_p = (R_f.col(0) - n * R_f.col(0).dot(n)).normalized();
  const Eigen::Vector3d t1 = C_.block<1, 3>(7, c + 3).transpose();
  EXPECT_LT((t1 - y_p.cross(n)).norm(), 1e-12);
  EXPECT_GT((t1 - x_p).norm(), 0.1) << "control: the two rules give different t1 here";
}

// Surface-basis hysteresis (#1018). The frame is pitched so that its x
// projected ⊥ n (n = +Z) has length `proj`, then rolled 0.6 about its x: the
// x rule then gives t1 = +X for every proj, the y rule y_p × n, which differs
// from it here (control in TheFallbackSwitchesAtATenthOfTheFrameX).
class SurfaceBasisHysteresisTest : public SurfaceFrameTest {
 protected:
  static Eigen::Matrix3d Frame(double proj) {
    return (Eigen::AngleAxisd(-std::acos(proj), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
  }

  // Assembles at `proj` and returns t1 (row 7's moment columns).
  Eigen::Vector3d T1At(double proj) {
    SetFrameRotation(Frame(proj));
    Assemble();
    return C_.block<1, 3>(7, info_.nv + 3).transpose();
  }

  // Which rule built t1 at `proj`: true = the frame-x rule.
  static bool IsXRule(const Eigen::Vector3d& t1, double proj) {
    const Eigen::Matrix3d R = Frame(proj);
    const Eigen::Vector3d n = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d x_p = (R.col(0) - n * R.col(0).dot(n)).normalized();
    const Eigen::Vector3d y_rule = (R.col(1) - n * R.col(1).dot(n)).normalized().cross(n);
    const bool x = (t1 - x_p).norm() < 1e-12;
    const bool y = (t1 - y_rule).norm() < 1e-12;
    EXPECT_NE(x, y) << "t1 matches exactly one rule at proj " << proj;
    return x;
  }

  static double Angle(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return std::acos(std::clamp(a.dot(b), -1.0, 1.0));
  }
};

// A frame whose x hovers inside the band (0.085 ↔ 0.115) keeps the rule it
// started with — fresh at 0.115 (x) or at 0.085 (y) — and t1 never jumps.
TEST_F(SurfaceBasisHysteresisTest, InsideTheBandTheBasisKeepsItsRule) {
  for (const double start : {0.115, 0.085}) {
    fc_.SetContactManager(&mgr_);  // fresh
    const bool start_x = start >= FrictionConeConstraint::kSurfaceBasisFresh;
    Eigen::Vector3d prev = T1At(start);
    ASSERT_EQ(IsXRule(prev, start), start_x);
    double worst = 0.0;
    int ticks = 0;
    for (int cycle = 0; cycle < 3; ++cycle) {
      for (int k = 0; k <= 12; ++k) {  // start → the other edge → back
        const double proj =
            start_x ? 0.115 - 0.005 * std::min(k, 12 - k) : 0.085 + 0.005 * std::min(k, 12 - k);
        const Eigen::Vector3d t1 = T1At(proj);
        EXPECT_EQ(IsXRule(t1, proj), start_x) << "start " << start << " proj " << proj;
        worst = std::max(worst, Angle(prev, t1));
        prev = t1;
        ++ticks;
      }
    }
    ASSERT_GT(ticks, 30);
    EXPECT_LE(worst, 0.05) << "start " << start << ": t1 turned this far in one tick";
  }
}

// Across the band (0.125 ↔ 0.075) the rule switches exactly once per
// crossing: x → y below kSurfaceBasisXToY, y → x above kSurfaceBasisYToX. The
// path keeps off the thresholds themselves, where the projection's rounding
// decides.
TEST_F(SurfaceBasisHysteresisTest, CrossingTheBandSwitchesOncePerCrossing) {
  fc_.SetContactManager(&mgr_);
  const std::vector<double> path = {0.125, 0.115, 0.105, 0.095, 0.085, 0.075,
                                    0.085, 0.095, 0.105, 0.115, 0.125};
  std::vector<double> switched_at;
  bool rule_x = IsXRule(T1At(path.front()), path.front());
  ASSERT_TRUE(rule_x);
  for (std::size_t k = 1; k < path.size(); ++k) {
    const bool x = IsXRule(T1At(path[k]), path[k]);
    if (x != rule_x) {
      switched_at.push_back(path[k]);
    }
    rule_x = x;
  }
  ASSERT_EQ(switched_at.size(), 2U);
  EXPECT_DOUBLE_EQ(switched_at[0], 0.075);
  EXPECT_DOUBLE_EQ(switched_at[1], 0.125);
}

// A contact that was inactive for a tick chooses afresh: on the x branch 0.09
// stays x, after one inactive tick 0.09 gives the y rule.
TEST_F(SurfaceBasisHysteresisTest, AContactBackFromInactivePicksItsBasisAfresh) {
  fc_.SetContactManager(&mgr_);
  ASSERT_TRUE(IsXRule(T1At(0.11), 0.11));
  ASSERT_TRUE(IsXRule(T1At(0.09), 0.09)) << "premise: the x branch holds inside the band";

  cs_.contacts[0].active = false;
  cs_.RecomputeActive(mgr_);
  ASSERT_EQ(fc_.IneqDim(cs_), 0);
  const int n_vars = info_.nv + 6;
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(11, n_vars);
  Eigen::VectorXd l = Eigen::VectorXd::Zero(11), u = l;
  fc_.ComputeInequality(cache_, cs_, info_, n_vars, C, l, u);
  cs_.contacts[0].active = true;
  cs_.RecomputeActive(mgr_);

  EXPECT_FALSE(IsXRule(T1At(0.09), 0.09)) << "back from inactive: split at 0.1 again";
}

// Without a cached placement for the contact the world-axis basis stands.
TEST_F(SurfaceFrameTest, NoCachedPlacementKeepsTheWorldAxes) {
  PinocchioCache bare;
  bare.Init(model_, {});
  const int n_vars = info_.nv + 6;
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(11, n_vars);
  Eigen::VectorXd l = Eigen::VectorXd::Zero(11), u = l;
  fc_.ComputeInequality(bare, cs_, info_, n_vars, C, l, u);
  EXPECT_DOUBLE_EQ(C(7, n_vars - 3), 1.0);  // +m_x row on the world x moment
  EXPECT_DOUBLE_EQ(C(5, n_vars - 2), 1.0);  // +m_y row on the world y moment
}

// Point contacts keep the world-normal basis whatever the frame's yaw.
TEST_F(FrictionConeTest, PointRowsIgnoreTheFrameYaw) {
  ContactManagerConfig mgr;
  mgr.contacts.resize(1);
  mgr.contacts[0].contact_dim = 3;
  mgr.contacts[0].friction_coeff = 0.5;
  mgr.contacts[0].friction_faces = 8;
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 3;
  ContactState cs;
  cs.Init(1);
  cs.contacts[0].active = true;
  cs.RecomputeActive(mgr);
  PinocchioCache cache;
  cache.Init(model_, rtc::tsid::ContactFrameIds(mgr));
  FrictionConeConstraint fc;
  fc.Init(*model_, info_, cache, YAML::Node{});
  fc.SetContactManager(&mgr);
  const int n_vars = info_.nv + 3;
  Eigen::MatrixXd C0 = Eigen::MatrixXd::Zero(9, n_vars), C1 = C0;
  Eigen::VectorXd l0 = Eigen::VectorXd::Zero(9), u0 = l0, l1 = l0, u1 = l0;
  fc.ComputeInequality(cache, cs, info_, n_vars, C0, l0, u0);
  cache.contact_frames[0].oMf = pinocchio::SE3(
      Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ()).toRotationMatrix(), Eigen::Vector3d::Zero());
  fc.ComputeInequality(cache, cs, info_, n_vars, C1, l1, u1);
  EXPECT_EQ(C0, C1);
  EXPECT_EQ(l0, l1);
  EXPECT_EQ(u0, u1);
}

// activation·min ≤ f_n ≤ max on the unilateral row (row 4 = −f_n), and the
// yaw-moment rows at μ_τ·f_n.
TEST_F(SurfaceFrameTest, NormalForceBoundsAndYawMomentEdges) {
  mgr_.contacts[0].normal_force_min = 20.0;
  mgr_.contacts[0].normal_force_max = 300.0;
  SetFrameRotation(Eigen::Matrix3d::Identity());
  Assemble();
  EXPECT_DOUBLE_EQ(l_(4), -300.0);
  EXPECT_DOUBLE_EQ(u_(4), -20.0);
  const double e = 1e-9;
  const auto at = [&](double fn) { return Rows(Wrench(0, 0, fn, 0, 0, 0))(4); };
  EXPECT_GT(at(20.0 - e), u_(4));  // below the floor
  EXPECT_LE(at(20.0), u_(4));
  EXPECT_GE(at(300.0), l_(4));
  EXPECT_LT(at(300.0 + e), l_(4));  // above the ceiling
  // |m_z| ≤ μ_τ·f_n: rows 9 / 10.
  EXPECT_LE(Rows(Wrench(0, 0, 100, 0, 0, 50.0 - e))(9), u_(9));
  EXPECT_GT(Rows(Wrench(0, 0, 100, 0, 0, 50.0 + e))(9), u_(9));
  EXPECT_GT(Rows(Wrench(0, 0, 100, 0, 0, -50.0 - e))(10), u_(10));

  // The floor follows the activation ramp.
  cs_.contacts[0].activation = 0.5;
  Assemble();
  EXPECT_DOUBLE_EQ(u_(4), -10.0);
  EXPECT_DOUBLE_EQ(l_(4), -300.0);
}

// The defaults (0 / +∞) leave the unilateral row exactly as before.
TEST_F(SurfaceFrameTest, DefaultBoundsKeepTheUnboundedRow) {
  SetFrameRotation(Eigen::Matrix3d::Identity());
  Assemble();
  EXPECT_EQ(l_(4), -1e10);
  EXPECT_EQ(u_(4), 0.0);
  EXPECT_FALSE(std::signbit(u_(4)));
}

TEST_F(FrictionConeTest, NormalForceBoundsAreCheckedOnLoad) {
  const auto load = [&](double lo, double hi) {
    YAML::Node cfg;
    YAML::Node c;
    c["name"] = "c0";
    c["frame"] = "panda_link8";
    c["type"] = "surface";
    c["normal_force_min"] = lo;
    c["normal_force_max"] = hi;
    cfg["contacts"].push_back(c);
    ContactManagerConfig mgr;
    mgr.Load(cfg, *model_);
    return mgr;
  };
  EXPECT_THROW(load(-1.0, 10.0), std::invalid_argument);
  EXPECT_THROW(load(50.0, 10.0), std::invalid_argument);
  EXPECT_THROW(load(std::numeric_limits<double>::quiet_NaN(), 10.0), std::invalid_argument);
  const ContactManagerConfig ok = load(10.0, 50.0);
  EXPECT_DOUBLE_EQ(ok.contacts[0].normal_force_min, 10.0);
  EXPECT_DOUBLE_EQ(ok.contacts[0].normal_force_max, 50.0);

  // Absent keys: 0 and +∞.
  YAML::Node cfg;
  YAML::Node c;
  c["name"] = "c0";
  c["frame"] = "panda_link8";
  cfg["contacts"].push_back(c);
  ContactManagerConfig mgr;
  mgr.Load(cfg, *model_);
  EXPECT_EQ(mgr.contacts[0].normal_force_min, 0.0);
  EXPECT_TRUE(std::isinf(mgr.contacts[0].normal_force_max));
}

}  // namespace
}  // namespace rtc::tsid
