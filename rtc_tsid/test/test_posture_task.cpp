#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/tasks/posture_task.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf =
    RTC_PANDA_URDF_PATH;

class PostureTaskTest : public ::testing::Test {
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

    ref_.init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated, 0);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  PinocchioCache cache_;
  ContactState contacts_;
  ControlReference ref_;
};

TEST_F(PostureTaskTest, InitAndDimension) {
  PostureTask task;
  YAML::Node cfg;
  task.init(*model_, robot_info_, cache_, cfg);

  EXPECT_EQ(task.residual_dim(), robot_info_.nv);
  EXPECT_EQ(task.name(), "posture");
  EXPECT_TRUE(task.is_active());
  EXPECT_DOUBLE_EQ(task.weight(), 1.0);
  EXPECT_EQ(task.priority(), 0);
}

TEST_F(PostureTaskTest, ComputeResidualAtRest) {
  PostureTask task;
  YAML::Node cfg;
  cfg["kp"] = 10.0;
  cfg["kd"] = 1.0;
  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  ref_.q_des = q;  // same as current → error = 0
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(robot_info_.nv, n_vars);
  Eigen::VectorXd r(robot_info_.nv);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // J should be identity in the a columns
  EXPECT_TRUE(J.isApprox(Eigen::MatrixXd::Identity(n_vars, n_vars)));

  // r should be zero (q_des == q, v_des == v, a_ff == 0)
  EXPECT_LT(r.norm(), 1e-10);
}

TEST_F(PostureTaskTest, ComputeResidualWithError) {
  PostureTask task;
  YAML::Node cfg;
  cfg["kp"] = 10.0;
  cfg["kd"] = 2.0;
  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  // Set desired = current + offset
  ref_.q_des = q;
  ref_.q_des.head(robot_info_.nv) += Eigen::VectorXd::Constant(robot_info_.nv, 0.1);
  ref_.v_des.setZero(robot_info_.nv);
  ref_.a_des.setZero(robot_info_.nv);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(robot_info_.nv, n_vars);
  Eigen::VectorXd r(robot_info_.nv);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // r = Kp * (q_des - q) + Kd * (0 - 0) + 0 = 10 * 0.1 = 1.0
  for (int i = 0; i < robot_info_.nv; ++i) {
    EXPECT_NEAR(r(i), 1.0, 1e-10);
  }
}

TEST_F(PostureTaskTest, LocalReference) {
  PostureTask task;
  YAML::Node cfg;
  cfg["kp"] = 5.0;
  cfg["kd"] = 1.0;
  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  // Set local reference (overrides ControlReference)
  Eigen::VectorXd q_des = q;
  q_des.head(robot_info_.nv) += Eigen::VectorXd::Constant(robot_info_.nv, 0.2);
  task.set_reference(q_des, v, Eigen::VectorXd::Zero(robot_info_.nv));

  // ControlReference with different values (should be ignored)
  ref_.q_des = q;
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(robot_info_.nv, n_vars);
  Eigen::VectorXd r(robot_info_.nv);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // r = 5.0 * 0.2 = 1.0 (using local ref, not ControlReference)
  for (int i = 0; i < robot_info_.nv; ++i) {
    EXPECT_NEAR(r(i), 1.0, 1e-10);
  }
}

TEST_F(PostureTaskTest, WeightAndPriority) {
  PostureTask task;
  YAML::Node cfg;
  cfg["weight"] = 2.5;
  cfg["priority"] = 3;
  task.init(*model_, robot_info_, cache_, cfg);

  EXPECT_DOUBLE_EQ(task.weight(), 2.5);
  EXPECT_EQ(task.priority(), 3);

  task.set_weight(0.5);
  task.set_priority(1);
  EXPECT_DOUBLE_EQ(task.weight(), 0.5);
  EXPECT_EQ(task.priority(), 1);
}

}  // namespace
}  // namespace rtc::tsid
