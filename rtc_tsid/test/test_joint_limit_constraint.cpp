#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/constraints/joint_limit_constraint.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf =
    "/usr/local/share/example-robot-data/robots/panda_description/urdf/"
    "panda.urdf";

class JointLimitConstraintTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.build(*model_, config);

    ContactManagerConfig contact_cfg;
    contact_cfg.max_contacts = 0;
    cache_.init(model_, contact_cfg);

    contacts_.init(0);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  PinocchioCache cache_;
  ContactState contacts_;
};

TEST_F(JointLimitConstraintTest, Dimensions) {
  JointLimitConstraint constraint;
  YAML::Node cfg;
  cfg["dt"] = 0.002;

  constraint.init(*model_, robot_info_, cache_, cfg);

  EXPECT_EQ(constraint.name(), "joint_limit");
  EXPECT_EQ(constraint.eq_dim(contacts_), 0);
  EXPECT_EQ(constraint.ineq_dim(contacts_), robot_info_.nv);  // 7 for Panda
  EXPECT_TRUE(constraint.is_active());
}

TEST_F(JointLimitConstraintTest, BoundsAtMidConfig) {
  JointLimitConstraint constraint;
  YAML::Node cfg;
  cfg["dt"] = 0.002;
  cfg["position_margin"] = 0.05;
  cfg["velocity_margin"] = 0.1;

  constraint.init(*model_, robot_info_, cache_, cfg);

  // Joint limit 중앙 설정 (neutral()은 일부 joint에서 범위 밖)
  Eigen::VectorXd q = 0.5 * (robot_info_.q_lower + robot_info_.q_upper);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd C(robot_info_.nv, n_vars);
  Eigen::VectorXd l(robot_info_.nv);
  Eigen::VectorXd u(robot_info_.nv);
  C.setZero();
  l.setZero();
  u.setZero();

  constraint.compute_inequality(cache_, contacts_, robot_info_, n_vars, C, l, u);

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

  constraint.init(*model_, robot_info_, cache_, cfg);

  // 중앙 config에서의 bounds
  Eigen::VectorXd q_mid = 0.5 * (robot_info_.q_lower + robot_info_.q_upper);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q_mid, v, contacts_);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd C(robot_info_.nv, n_vars);
  Eigen::VectorXd l_neutral(robot_info_.nv);
  Eigen::VectorXd u_neutral(robot_info_.nv);
  C.setZero(); l_neutral.setZero(); u_neutral.setZero();

  constraint.compute_inequality(cache_, contacts_, robot_info_, n_vars,
                                C, l_neutral, u_neutral);

  // upper limit 아주 가까이로 이동 (joint 0)
  Eigen::VectorXd q_near_limit = q_mid;
  q_near_limit(0) = robot_info_.q_upper(0) - 0.001;  // margin 안쪽 아슬아슬
  cache_.update(q_near_limit, v, contacts_);

  Eigen::VectorXd l_near(robot_info_.nv);
  Eigen::VectorXd u_near(robot_info_.nv);
  C.setZero(); l_near.setZero(); u_near.setZero();

  constraint.compute_inequality(cache_, contacts_, robot_info_, n_vars,
                                C, l_near, u_near);

  // upper limit 근처: position-based u가 더 tight해야 함
  EXPECT_LT(u_near(0), u_neutral(0))
      << "upper bound should be tighter near upper limit";
}

TEST_F(JointLimitConstraintTest, VelocityBoundsActive) {
  JointLimitConstraint constraint;
  YAML::Node cfg;
  cfg["dt"] = 0.002;
  cfg["position_margin"] = 0.05;
  cfg["velocity_margin"] = 0.1;

  constraint.init(*model_, robot_info_, cache_, cfg);

  // neutral config, 높은 속도
  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  v(0) = robot_info_.v_max(0) * 0.95;  // v_max 근처
  cache_.update(q, v, contacts_);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd C(robot_info_.nv, n_vars);
  Eigen::VectorXd l(robot_info_.nv);
  Eigen::VectorXd u(robot_info_.nv);
  C.setZero(); l.setZero(); u.setZero();

  constraint.compute_inequality(cache_, contacts_, robot_info_, n_vars, C, l, u);

  // velocity limit 근처 → u가 tight해야 함 (감속 요구)
  // vel_ub = (v_max - margin - v_curr) / dt
  const double expected_vel_ub =
      (robot_info_.v_max(0) - 0.1 - v(0)) / 0.002;
  EXPECT_NEAR(u(0), std::min(u(0), expected_vel_ub), 1e-6);
}

TEST_F(JointLimitConstraintTest, EmptyEquality) {
  JointLimitConstraint constraint;
  YAML::Node cfg;
  cfg["dt"] = 0.002;

  constraint.init(*model_, robot_info_, cache_, cfg);

  // compute_equality는 아무것도 하지 않아야 함
  Eigen::MatrixXd A(0, 0);
  Eigen::VectorXd b(0);
  constraint.compute_equality(cache_, contacts_, robot_info_, 0, A, b);
  // 정상적으로 return하면 pass
  EXPECT_EQ(constraint.eq_dim(contacts_), 0);
}

}  // namespace
}  // namespace rtc::tsid
