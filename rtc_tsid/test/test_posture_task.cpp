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

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;

class PostureTaskTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.Build(*model_, config);

    ContactManagerConfig contact_cfg;
    contact_cfg.max_contacts = 0;
    cache_.Init(model_, contact_cfg);

    contacts_.Init(0);

    ref_.Init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated, 0);
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
  task.Init(*model_, robot_info_, cache_, cfg);

  EXPECT_EQ(task.ResidualDim(), robot_info_.nv);
  EXPECT_EQ(task.Name(), "posture");
  EXPECT_TRUE(task.IsActive());
  EXPECT_DOUBLE_EQ(task.Weight(), 1.0);
  EXPECT_EQ(task.Priority(), 0);
}

TEST_F(PostureTaskTest, ComputeResidualAtRest) {
  PostureTask task;
  YAML::Node cfg;
  cfg["kp"] = 10.0;
  cfg["kd"] = 1.0;
  task.Init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v, contacts_);

  ref_.q_des = q;  // same as current → error = 0
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(robot_info_.nv, n_vars);
  Eigen::VectorXd r(robot_info_.nv);
  J.setZero();
  r.setZero();

  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J, r);

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
  task.Init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v, contacts_);

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

  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J, r);

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
  task.Init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v, contacts_);

  // Set local reference (overrides ControlReference)
  Eigen::VectorXd q_des = q;
  q_des.head(robot_info_.nv) += Eigen::VectorXd::Constant(robot_info_.nv, 0.2);
  task.SetReference(q_des, v, Eigen::VectorXd::Zero(robot_info_.nv));

  // ControlReference with different values (should be ignored)
  ref_.q_des = q;
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(robot_info_.nv, n_vars);
  Eigen::VectorXd r(robot_info_.nv);
  J.setZero();
  r.setZero();

  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J, r);

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
  task.Init(*model_, robot_info_, cache_, cfg);

  EXPECT_DOUBLE_EQ(task.Weight(), 2.5);
  EXPECT_EQ(task.Priority(), 3);

  task.SetWeight(0.5);
  task.SetPriority(1);
  EXPECT_DOUBLE_EQ(task.Weight(), 0.5);
  EXPECT_EQ(task.Priority(), 1);
}

}  // namespace
}  // namespace rtc::tsid
