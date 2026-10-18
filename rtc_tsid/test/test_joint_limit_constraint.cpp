#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include "box_test_util.hpp"
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
        const double pos_lb = 2.0 * (QMin(i) - q(i) - v(i) * kDt) / dt2;
        const double pos_ub = 2.0 * (QMax(i) - q(i) - v(i) * kDt) / dt2;
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

}  // namespace
}  // namespace rtc::tsid
