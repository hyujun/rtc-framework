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

// Fixture: build the Panda model once per test in SetUp(), shared via model_ /
// info_. Each TEST_F gets a fresh fixture instance, so behaviour is identical
// to the previous per-TEST inline build — only the boilerplate is removed.
class FrictionConeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(RTC_PANDA_URDF_PATH, *model_);
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
  fc.ComputeInequality(cache, cs_explicit, info, n_vars, C2, l, u);

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

}  // namespace
}  // namespace rtc::tsid
