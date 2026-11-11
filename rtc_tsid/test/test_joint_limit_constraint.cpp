#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include "box_test_util.hpp"
#include "g1_fixture.hpp"
#include "panda_fixture.hpp"
#include "rtc_tsid/constraints/joint_limit_constraint.hpp"

namespace rtc::tsid {
namespace {

class JointLimitConstraintTest : public test::PandaNoContactTest {
 protected:
  static constexpr double kDt = 0.002;
  static constexpr double kMarginPos = 0.05;
  static constexpr double kMarginVel = 0.1;

  void InitDefault(JointLimitConstraint& c) {
    YAML::Node cfg;
    cfg["dt"] = kDt;
    cfg["position_margin"] = kMarginPos;
    cfg["velocity_margin"] = kMarginVel;
    c.Init(*model_, robot_info_, cache_, cfg);
  }

  struct Box {
    Eigen::MatrixXd C;
    Eigen::VectorXd l;
    Eigen::VectorXd u;
  };

  Box Assemble(JointLimitConstraint& c, const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    cache_.Update(q, v);
    const int n = robot_info_.nv;
    Box b;
    b.C.setZero(n, n);
    b.l.setZero(n);
    b.u.setZero(n);
    c.ComputeInequality(cache_, contacts_, robot_info_, n, b.C, b.l, b.u);
    return b;
  }

  Eigen::VectorXd MidConfig() const { return 0.5 * (robot_info_.q_lower + robot_info_.q_upper); }

  // Effective (margin-applied) limits of joint i, as Init computes them.
  double QMax(int i) const { return robot_info_.q_upper(i) - kMarginPos; }

  double QMin(int i) const { return robot_info_.q_lower(i) + kMarginPos; }

  double VLim(int i) const { return robot_info_.v_max(i) - kMarginVel; }

  double AVelLb(int i, double vi) const { return (-VLim(i) - vi) / kDt; }

  double AVelUb(int i, double vi) const { return (VLim(i) - vi) / kDt; }
};

// The coefficient (#1019). A plant holding a over one period in N semi-
// implicit substeps moves q by v·dt + c·a·dt², c = (1 + 1/N)/2. Commanded the
// box's bound, every c in [½, 1] lands inside the margin-applied limit, and
// the tight c lands on it: c = 1 (N = 1) for a joint that would stay inside at
// a = 0, c = ½ (continuous plant) for one that would cross. Both ends.
TEST_F(JointLimitConstraintTest, TheBoundKeepsEverySubstepCountInsideTheLimit) {
  JointLimitConstraint c;
  InitDefault(c);
  const auto move = [](double v, double a, double coeff) {
    return v * kDt + coeff * a * kDt * kDt;
  };
  // (gap to the limit, speed toward it): inside at a = 0, then crossing.
  for (const auto& [gap, toward] : {std::pair{1e-4, 0.02}, std::pair{1e-4, 0.08}}) {
    const bool crosses = toward * kDt > gap;
    const double tight = crosses ? 0.5 : 1.0;
    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
    // Upper end.
    Eigen::VectorXd q = MidConfig();
    q(0) = QMax(0) - gap;
    v(0) = toward;
    Box b = Assemble(c, q, v);
    ASSERT_LT(b.u(0), AVelUb(0, toward)) << "premise: the position bound binds";
    ASSERT_GE(b.u(0), AVelLb(0, toward)) << "premise: the row is not inverted";
    EXPECT_NEAR(q(0) + move(toward, b.u(0), tight), QMax(0), 1e-12) << "toward " << toward;
    for (const double coeff : {0.5, 0.75, 1.0}) {
      EXPECT_LE(q(0) + move(toward, b.u(0), coeff), QMax(0) + 1e-12) << coeff << " " << toward;
    }
    // Lower end, mirrored.
    q = MidConfig();
    q(0) = QMin(0) + gap;
    v(0) = -toward;
    b = Assemble(c, q, v);
    ASSERT_GT(b.l(0), AVelLb(0, -toward)) << "premise: the position bound binds";
    EXPECT_NEAR(q(0) + move(-toward, b.l(0), tight), QMin(0), 1e-12) << "toward " << toward;
    for (const double coeff : {0.5, 0.75, 1.0}) {
      EXPECT_GE(q(0) + move(-toward, b.l(0), coeff), QMin(0) - 1e-12) << coeff << " " << toward;
    }
  }
  EXPECT_DOUBLE_EQ(c.Dt(), kDt);
}

TEST_F(JointLimitConstraintTest, Dimensions) {
  JointLimitConstraint constraint;
  YAML::Node cfg;
  cfg["dt"] = 0.002;

  constraint.Init(*model_, robot_info_, cache_, cfg);

  EXPECT_EQ(constraint.Name(), "joint_limit");
  EXPECT_EQ(constraint.EqDim(contacts_), 0);
  EXPECT_EQ(constraint.IneqDim(contacts_), robot_info_.nv);  // 7 for Panda
  EXPECT_TRUE(constraint.IsActive());
}

TEST_F(JointLimitConstraintTest, BoundsAtMidConfig) {
  JointLimitConstraint constraint;
  YAML::Node cfg;
  cfg["dt"] = 0.002;
  cfg["position_margin"] = 0.05;
  cfg["velocity_margin"] = 0.1;

  constraint.Init(*model_, robot_info_, cache_, cfg);

  // Joint limit 중앙 설정 (neutral()은 일부 joint에서 범위 밖)
  Eigen::VectorXd q = 0.5 * (robot_info_.q_lower + robot_info_.q_upper);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd C(robot_info_.nv, n_vars);
  Eigen::VectorXd l(robot_info_.nv);
  Eigen::VectorXd u(robot_info_.nv);
  C.setZero();
  l.setZero();
  u.setZero();

  constraint.ComputeInequality(cache_, contacts_, robot_info_, n_vars, C, l, u);
  test::ExpectBoxNonEmpty(l, u);

  // C = Identity
  EXPECT_TRUE(C.isApprox(Eigen::MatrixXd::Identity(n_vars, n_vars)));

  // 중앙 config에서는 모든 bound가 넓어야 함 (l < 0 < u)
  for (int i = 0; i < robot_info_.nv; ++i) {
    EXPECT_LT(l(i), 0.0) << "joint " << i << " lower bound should be < 0";
    EXPECT_GT(u(i), 0.0) << "joint " << i << " upper bound should be > 0";
    EXPECT_LT(l(i), u(i)) << "joint " << i << " l < u";
  }
}

TEST_F(JointLimitConstraintTest, TighterBoundsNearLimit) {
  JointLimitConstraint constraint;
  YAML::Node cfg;
  cfg["dt"] = 0.002;
  cfg["position_margin"] = 0.05;
  cfg["velocity_margin"] = 0.1;

  constraint.Init(*model_, robot_info_, cache_, cfg);

  // 중앙 config에서의 bounds
  Eigen::VectorXd q_mid = 0.5 * (robot_info_.q_lower + robot_info_.q_upper);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q_mid, v);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd C(robot_info_.nv, n_vars);
  Eigen::VectorXd l_neutral(robot_info_.nv);
  Eigen::VectorXd u_neutral(robot_info_.nv);
  C.setZero();
  l_neutral.setZero();
  u_neutral.setZero();

  constraint.ComputeInequality(cache_, contacts_, robot_info_, n_vars, C, l_neutral, u_neutral);
  test::ExpectBoxNonEmpty(l_neutral, u_neutral);

  // upper limit 아주 가까이로 이동 (joint 0)
  Eigen::VectorXd q_near_limit = q_mid;
  q_near_limit(0) = robot_info_.q_upper(0) - 0.001;  // margin 안쪽 아슬아슬
  cache_.Update(q_near_limit, v);

  Eigen::VectorXd l_near(robot_info_.nv);
  Eigen::VectorXd u_near(robot_info_.nv);
  C.setZero();
  l_near.setZero();
  u_near.setZero();

  constraint.ComputeInequality(cache_, contacts_, robot_info_, n_vars, C, l_near, u_near);
  // Pre-#551 this box was inverted (l = vel_lb > u = pos_ub) and the test
  // only looked at tightness. The box must stay non-empty.
  test::ExpectBoxNonEmpty(l_near, u_near);

  // upper limit 근처: position-based u가 더 tight해야 함
  EXPECT_LT(u_near(0), u_neutral(0)) << "upper bound should be tighter near upper limit";
}

TEST_F(JointLimitConstraintTest, VelocityBoundsActive) {
  JointLimitConstraint constraint;
  YAML::Node cfg;
  cfg["dt"] = 0.002;
  cfg["position_margin"] = 0.05;
  cfg["velocity_margin"] = 0.1;

  constraint.Init(*model_, robot_info_, cache_, cfg);

  // neutral config, 높은 속도
  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  v(0) = robot_info_.v_max(0) * 0.95;  // v_max 근처
  cache_.Update(q, v);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd C(robot_info_.nv, n_vars);
  Eigen::VectorXd l(robot_info_.nv);
  Eigen::VectorXd u(robot_info_.nv);
  C.setZero();
  l.setZero();
  u.setZero();

  constraint.ComputeInequality(cache_, contacts_, robot_info_, n_vars, C, l, u);
  test::ExpectBoxNonEmpty(l, u);

  // velocity limit 근처 → u가 tight해야 함 (감속 요구)
  // vel_ub = (v_max - margin - v_curr) / dt
  const double expected_vel_ub = (robot_info_.v_max(0) - 0.1 - v(0)) / 0.002;
  EXPECT_NEAR(u(0), std::min(u(0), expected_vel_ub), 1e-6);
}

TEST_F(JointLimitConstraintTest, EmptyEquality) {
  JointLimitConstraint constraint;
  YAML::Node cfg;
  cfg["dt"] = 0.002;

  constraint.Init(*model_, robot_info_, cache_, cfg);

  // compute_equality는 아무것도 하지 않아야 함
  Eigen::MatrixXd A(0, 0);
  Eigen::VectorXd b(0);
  constraint.ComputeEquality(cache_, contacts_, robot_info_, 0, A, b);
  // 정상적으로 return하면 pass
  EXPECT_EQ(constraint.EqDim(contacts_), 0);
}

// ── #551: an empty row inside the position-margin band ──────────────────────

TEST_F(JointLimitConstraintTest, InvertedBoxCollapsesToFeasible) {
  JointLimitConstraint c;
  InitDefault(c);

  // Joint 0 parked just under its upper limit, joint 1 just over its lower
  // limit — both inside the margin band, both at rest.
  Eigen::VectorXd q = MidConfig();
  q(0) = robot_info_.q_upper(0) - 0.001;
  q(1) = robot_info_.q_lower(1) + 0.001;
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);

  const Box b = Assemble(c, q, v);

  // The raw bounds cross on both rows (counted before the repair) …
  EXPECT_EQ(c.InvertedCount(), 2);
  // … and the assembled box is still non-empty on every row.
  test::ExpectBoxNonEmpty(b.l, b.u);
  // The collapsed row sits on the velocity bound it crossed: full braking
  // toward the range (same repair as ClikReferenceGenerator::AssembleBox).
  EXPECT_DOUBLE_EQ(b.l(0), AVelLb(0, 0.0));
  EXPECT_DOUBLE_EQ(b.u(0), AVelLb(0, 0.0));
  EXPECT_DOUBLE_EQ(b.l(1), AVelUb(1, 0.0));
  EXPECT_DOUBLE_EQ(b.u(1), AVelUb(1, 0.0));
}

TEST_F(JointLimitConstraintTest, NonInvertedRowsBitIdentical) {
  JointLimitConstraint c;
  InitDefault(c);

  // Sweep joint 0 across its whole range at three velocities and compare every
  // non-inverted row with the textbook assembly, bit for bit: the repair must
  // not touch a row that did not need it.
  const Eigen::VectorXd q_mid = MidConfig();
  const double dt2 = kDt * kDt;
  int rows_checked = 0;
  for (double s = 0.0; s <= 1.0; s += 0.025) {
    for (double vf : {-0.5, 0.0, 0.5}) {
      Eigen::VectorXd q = q_mid;
      q(0) = robot_info_.q_lower(0) + s * (robot_info_.q_upper(0) - robot_info_.q_lower(0));
      Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
      v(0) = vf * robot_info_.v_max(0);
      const Box b = Assemble(c, q, v);
      for (int i = 0; i < robot_info_.nv; ++i) {
        // Doubled once q would cross the limit at a = 0 (#1019, header).
        const double d_lb = QMin(i) - q(i) - v(i) * kDt;
        const double d_ub = QMax(i) - q(i) - v(i) * kDt;
        const double pos_lb = (d_lb > 0.0 ? 2.0 : 1.0) * d_lb / dt2;
        const double pos_ub = (d_ub < 0.0 ? 2.0 : 1.0) * d_ub / dt2;
        const double l_ref = std::max(pos_lb, AVelLb(i, v(i)));
        const double u_ref = std::min(pos_ub, AVelUb(i, v(i)));
        if (l_ref > u_ref)
          continue;  // inverted → repaired row, covered by the other tests
        EXPECT_EQ(b.l(i), l_ref) << "s=" << s << " vf=" << vf << " i=" << i;
        EXPECT_EQ(b.u(i), u_ref) << "s=" << s << " vf=" << vf << " i=" << i;
        ++rows_checked;
      }
    }
  }
  EXPECT_GT(rows_checked, 500);
}

TEST_F(JointLimitConstraintTest, BoundaryFollowsVelocity) {
  JointLimitConstraint c;
  InitDefault(c);
  const Eigen::VectorXd q_mid = MidConfig();
  constexpr double kEps = 1e-4;  // rad; 2·eps/dt² = 50 rad/s² clear of the crossing

  // Upper band: the row inverts iff q > q_max + ½·(v_lim − v)·dt — a boundary
  // that moves with the joint's velocity, in both directions.
  for (double v0 : {-1.0, 0.0, 1.0}) {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
    v(0) = v0;
    const double q_b = QMax(0) + 0.5 * (VLim(0) - v0) * kDt;
    Eigen::VectorXd q = q_mid;
    q(0) = q_b - kEps;
    Assemble(c, q, v);
    EXPECT_EQ(c.InvertedCount(), 0) << "upper, v0=" << v0;
    q(0) = q_b + kEps;
    Assemble(c, q, v);
    EXPECT_EQ(c.InvertedCount(), 1) << "upper, v0=" << v0;
  }
  // Lower band: q < q_min − ½·(v_lim + v)·dt.
  for (double v0 : {-1.0, 0.0, 1.0}) {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
    v(0) = v0;
    const double q_b = QMin(0) - 0.5 * (VLim(0) + v0) * kDt;
    Eigen::VectorXd q = q_mid;
    q(0) = q_b + kEps;
    Assemble(c, q, v);
    EXPECT_EQ(c.InvertedCount(), 0) << "lower, v0=" << v0;
    q(0) = q_b - kEps;
    Assemble(c, q, v);
    EXPECT_EQ(c.InvertedCount(), 1) << "lower, v0=" << v0;
  }
  // A velocity over its limit at mid range never inverts a row on its own:
  // both velocity bounds move together and vel_lb < vel_ub always holds.
  {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
    v(0) = VLim(0) + 0.5;
    const Box b = Assemble(c, q_mid, v);
    EXPECT_EQ(c.InvertedCount(), 0);
    test::ExpectBoxNonEmpty(b.l, b.u);
    EXPECT_LT(b.u(0), 0.0);  // but it does demand braking
  }
}

TEST_F(JointLimitConstraintTest, NonFiniteInputBypassesGuard) {
  JointLimitConstraint c;
  InitDefault(c);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  // A NaN position is not a box to repair: the row stays non-finite for the
  // solve to fail on (NUM-7) and is not counted as inverted.
  Eigen::VectorXd q = MidConfig();
  q(0) = nan;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  Box b = Assemble(c, q, v);
  EXPECT_FALSE(std::isfinite(b.l(0)));
  EXPECT_FALSE(std::isfinite(b.u(0)));
  EXPECT_EQ(c.InvertedCount(), 0);

  // q = +inf is the case that exercises the guard: pos_ub = -inf < vel_lb, so
  // without the isfinite check the row would be "repaired" to the finite
  // vel_lb and the infinity laundered away. It must stay non-finite.
  q = MidConfig();
  q(0) = inf;
  b = Assemble(c, q, v);
  EXPECT_FALSE(std::isfinite(b.u(0)));
  EXPECT_EQ(c.InvertedCount(), 0);
}

TEST_F(JointLimitConstraintTest, InitRejectsNonPositiveDtAndNonFiniteMargin) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  {
    JointLimitConstraint c;
    YAML::Node cfg;
    cfg["dt"] = 0.0;
    EXPECT_THROW(c.Init(*model_, robot_info_, cache_, cfg), std::invalid_argument);
  }
  {
    JointLimitConstraint c;
    YAML::Node cfg;
    cfg["dt"] = kDt;
    cfg["position_margin"] = nan;
    EXPECT_THROW(c.Init(*model_, robot_info_, cache_, cfg), std::invalid_argument);
  }
  {
    JointLimitConstraint c;
    YAML::Node cfg;
    cfg["dt"] = kDt;
    cfg["velocity_margin"] = nan;
    EXPECT_THROW(c.Init(*model_, robot_info_, cache_, cfg), std::invalid_argument);
  }
  {
    // A negative margin would put q_min_ above q_max_ and let both sides of a
    // row cross at once, which the collapse assumes cannot happen.
    JointLimitConstraint c;
    YAML::Node cfg;
    cfg["dt"] = kDt;
    cfg["position_margin"] = -0.01;
    EXPECT_THROW(c.Init(*model_, robot_info_, cache_, cfg), std::invalid_argument);
  }
  {
    JointLimitConstraint c;
    RobotModelInfo info = robot_info_;
    info.q_lower(0) = info.q_upper(0) + 0.1;  // inverted position limits
    YAML::Node cfg;
    cfg["dt"] = kDt;
    EXPECT_THROW(c.Init(*model_, info, cache_, cfg), std::invalid_argument);
  }
  {
    // A locked joint (v_max == 0) is legitimate; only negative / non-finite
    // velocity limits are rejected.
    JointLimitConstraint c;
    RobotModelInfo info = robot_info_;
    info.v_max(0) = 0.0;
    YAML::Node cfg;
    cfg["dt"] = kDt;
    EXPECT_NO_THROW(c.Init(*model_, info, cache_, cfg));
    info.v_max(0) = -1.0;
    EXPECT_THROW(c.Init(*model_, info, cache_, cfg), std::invalid_argument);
  }
}

// ── Floating base (G1 free-flyer) ────────────────────────────────────────────

// One row per actuated joint, none for the six free-flyer DoF: row i is
// e_(6+i) and its box comes from q[7+i], v[6+i].
TEST(FloatingJointLimitTest, RowsAreTheActuatedJointsOnly) {
  const auto model = test::SharedG1Model();
  RobotModelInfo info;
  info.Build(*model, YAML::Node{});
  PinocchioCache cache;
  cache.Init(model, {});
  ContactState cs;
  cs.Init(0);

  JointLimitConstraint jl;
  YAML::Node cfg;
  cfg["dt"] = 0.002;
  jl.Init(*model, info, cache, cfg);
  const int na = test::kG1Actuated;
  ASSERT_EQ(jl.IneqDim(cs), na);
  ASSERT_EQ(jl.MaxIneqDim(ContactManagerConfig{}), na);

  // Fixture premise: every actuated joint has a velocity limit (else it would
  // be a locked joint with a zero box).
  EXPECT_GT(info.v_max.tail(na).minCoeff(), 0.0);

  // A base pose far from the origin and a spinning base: neither may reach a row.
  Eigen::VectorXd q = pinocchio::neutral(*model);
  q.head(3) << 50.0, -20.0, 3.0;
  q.segment(3, 4) = Eigen::Quaterniond(Eigen::AngleAxisd(2.0, Eigen::Vector3d::UnitX())).coeffs();
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model->nv);
  v.head(6) << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  v(6 + 3) = 0.4;
  cache.Update(q, v);

  const int n_vars = model->nv;
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(na, n_vars);
  Eigen::VectorXd l(na), u(na);
  jl.ComputeInequality(cache, cs, info, n_vars, C, l, u);

  Eigen::MatrixXd expected_C = Eigen::MatrixXd::Zero(na, n_vars);
  expected_C.rightCols(na).setIdentity();
  EXPECT_EQ(C, expected_C);
  // Row 3 against the one-tick formulas on q[10], v[9] (default margins 0.05 /
  // 0.1, clamped to 0 on a range narrower than twice the margin).
  const double dt = 0.002;
  const double qi = q(7 + 3);
  const double vi = v(6 + 3);
  const double range = info.q_upper(7 + 3) - info.q_lower(7 + 3);
  const double m_pos = range > 0.1 ? 0.05 : 0.0;
  const double m_vel = info.v_max(6 + 3) > 0.2 ? 0.1 : 0.0;
  const double d_ub = info.q_upper(7 + 3) - m_pos - qi - vi * dt;
  const double pos_ub = (d_ub < 0.0 ? 2.0 : 1.0) * d_ub / (dt * dt);
  const double vel_ub = (info.v_max(6 + 3) - m_vel - vi) / dt;
  EXPECT_NEAR(u(3), std::min(pos_ub, vel_ub), 1e-9 * std::abs(std::min(pos_ub, vel_ub)));
  EXPECT_TRUE(l.allFinite() && u.allFinite());
  EXPECT_LT(u.cwiseAbs().maxCoeff(), 1e9) << "no ±1e10 base sentinel rows";
}

// A joint that is not 1-DoF after the root (here the composite 7/6 root of a
// model that is therefore fixed-base) is refused: its q / v entries do not pair.
TEST(FloatingJointLimitTest, ANonOneDofJointIsRefused) {
  const auto model = test::CompositeRootPandaModel();
  RobotModelInfo info;
  info.Build(*model, YAML::Node{});
  PinocchioCache cache;
  cache.Init(model, {});
  JointLimitConstraint jl;
  EXPECT_THROW(jl.Init(*model, info, cache, YAML::Node{}), std::invalid_argument);

  // Control: the same Panda on its usual fixed root is accepted.
  const auto panda = test::SharedPandaModel();
  RobotModelInfo panda_info;
  panda_info.Build(*panda, YAML::Node{});
  PinocchioCache panda_cache;
  panda_cache.Init(panda, {});
  EXPECT_NO_THROW(jl.Init(*panda, panda_info, panda_cache, YAML::Node{}));
}

// A continuous joint (nq 2 / nv 1) fills its velocity row like a 1-DoF joint
// would, but its q entries are cos / sin: refused by name, not boxed on cos.
TEST(FloatingJointLimitTest, AContinuousJointIsRefused) {
  auto model = std::make_shared<pinocchio::Model>();
  const auto inertia = pinocchio::Inertia::FromSphere(1.0, 0.1);
  const auto wheel =
      model->addJoint(0, pinocchio::JointModelRUBZ(), pinocchio::SE3::Identity(), "wheel");
  model->appendBodyToJoint(wheel, inertia);
  const auto j2 =
      model->addJoint(wheel, pinocchio::JointModelRZ(), pinocchio::SE3::Identity(), "j2");
  model->appendBodyToJoint(j2, inertia);
  model->lowerPositionLimit.setConstant(-1.0);
  model->upperPositionLimit.setConstant(1.0);
  model->upperVelocityLimit.setConstant(1.0);
  model->upperEffortLimit.setConstant(10.0);
  ASSERT_EQ(model->nq, 3);
  ASSERT_EQ(model->nv, 2);

  RobotModelInfo info;
  info.Build(*model, YAML::Node{});
  PinocchioCache cache;
  cache.Init(model, {});
  JointLimitConstraint jl;
  try {
    jl.Init(*model, info, cache, YAML::Node{});
    ADD_FAILURE() << "a continuous joint was accepted";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("not 1-DoF"), std::string::npos) << e.what();
  }
}

}  // namespace
}  // namespace rtc::tsid
