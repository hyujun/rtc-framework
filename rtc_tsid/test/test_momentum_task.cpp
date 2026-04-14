#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/tasks/momentum_task.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf =
    "/usr/local/share/example-robot-data/robots/panda_description/urdf/"
    "panda.urdf";

class MomentumTaskTest : public ::testing::Test {
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

TEST_F(MomentumTaskTest, InitAngularRegularize) {
  MomentumTask task;
  YAML::Node cfg;
  cfg["mode"] = "angular_regularize";
  cfg["weight"] = 1.0;
  cfg["priority"] = 2;

  task.init(*model_, robot_info_, cache_, cfg);

  EXPECT_EQ(task.residual_dim(), 3);
  EXPECT_EQ(task.name(), "momentum");
  EXPECT_DOUBLE_EQ(task.weight(), 1.0);
  EXPECT_EQ(task.priority(), 2);
  EXPECT_TRUE(cache_.compute_centroidal);
}

TEST_F(MomentumTaskTest, InitFullTrack) {
  MomentumTask task;
  YAML::Node cfg;
  cfg["mode"] = "full_track";

  task.init(*model_, robot_info_, cache_, cfg);

  EXPECT_EQ(task.residual_dim(), 6);
}

TEST_F(MomentumTaskTest, DefaultModeIsAngularRegularize) {
  MomentumTask task;
  YAML::Node cfg;

  task.init(*model_, robot_info_, cache_, cfg);

  EXPECT_EQ(task.residual_dim(), 3);
}

TEST_F(MomentumTaskTest, AngularRegularizeJBlock) {
  MomentumTask task;
  YAML::Node cfg;
  cfg["mode"] = "angular_regularize";

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(3, n_vars);
  Eigen::VectorXd r(3);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // J = Ag의 angular 부분 (하위 3행)
  EXPECT_TRUE(J.isApprox(cache_.Ag.bottomRows(3)));
}

TEST_F(MomentumTaskTest, FullTrackJBlock) {
  MomentumTask task;
  YAML::Node cfg;
  cfg["mode"] = "full_track";

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(6, n_vars);
  Eigen::VectorXd r(6);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // J = Ag 전체
  EXPECT_TRUE(J.isApprox(cache_.Ag));
}

TEST_F(MomentumTaskTest, AngularRegularizeResidualAtZeroVelocity) {
  MomentumTask task;
  YAML::Node cfg;
  cfg["mode"] = "angular_regularize";

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(3, n_vars);
  Eigen::VectorXd r(3);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // v=0 → hg_drift=0 → r = -hg_drift_angular = 0
  EXPECT_LT(r.norm(), 1e-10);
}

TEST_F(MomentumTaskTest, DriftWithVelocity) {
  MomentumTask task;
  YAML::Node cfg;
  cfg["mode"] = "angular_regularize";

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  v(0) = 1.0;
  v(2) = -0.5;
  cache_.update(q, v, contacts_);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(3, n_vars);
  Eigen::VectorXd r(3);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // r = -hg_drift.tail(3), v≠0이므로 hg_drift≠0
  EXPECT_TRUE(r.isApprox(-cache_.hg_drift.tail(3), 1e-10));
}

TEST_F(MomentumTaskTest, FullTrackWithReference) {
  MomentumTask task;
  YAML::Node cfg;
  cfg["mode"] = "full_track";

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  Eigen::Matrix<double, 6, 1> hg_dot_des;
  hg_dot_des << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  task.set_momentum_reference(hg_dot_des);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(6, n_vars);
  Eigen::VectorXd r(6);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // r = hg_dot_des - hg_drift, v=0 → hg_drift=0
  EXPECT_TRUE(r.isApprox(hg_dot_des, 1e-10));
}

}  // namespace
}  // namespace rtc::tsid
