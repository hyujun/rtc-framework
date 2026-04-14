#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/tasks/com_task.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf =
    "/usr/local/share/example-robot-data/robots/panda_description/urdf/"
    "panda.urdf";

class CoMTaskTest : public ::testing::Test {
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

TEST_F(CoMTaskTest, InitAndDimension) {
  CoMTask task;
  YAML::Node cfg;
  cfg["kp"] = 100.0;
  cfg["kd"] = 20.0;
  cfg["weight"] = 50.0;
  cfg["priority"] = 0;

  task.init(*model_, robot_info_, cache_, cfg);

  EXPECT_EQ(task.residual_dim(), 3);
  EXPECT_EQ(task.name(), "com");
  EXPECT_TRUE(task.is_active());
  EXPECT_DOUBLE_EQ(task.weight(), 50.0);
  EXPECT_EQ(task.priority(), 0);

  // compute_com 플래그가 활성화되어야 함
  EXPECT_TRUE(cache_.compute_com);
}

TEST_F(CoMTaskTest, ZeroErrorAtCurrentCoM) {
  CoMTask task;
  YAML::Node cfg;
  cfg["kp"] = 100.0;
  cfg["kd"] = 20.0;

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  // 현재 CoM을 목표로 설정
  task.set_com_reference(cache_.com_position);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(3, n_vars);
  Eigen::VectorXd r(3);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // J = Jcom (non-zero)
  EXPECT_GT(J.norm(), 0.0);

  // r = a_des - com_drift
  // v=0 → e_vel=0, com_drift≈0 → r ≈ kp*e_pos + kd*e_vel = 0
  // com_drift는 v=0이면 0
  EXPECT_LT(r.norm(), 1e-8);
}

TEST_F(CoMTaskTest, PositionErrorResponse) {
  CoMTask task;
  YAML::Node cfg;
  cfg["kp"] = 100.0;
  cfg["kd"] = 20.0;

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  // CoM에서 z축으로 0.1m offset 추가
  Eigen::Vector3d target = cache_.com_position;
  target(2) += 0.1;
  task.set_com_reference(target);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(3, n_vars);
  Eigen::VectorXd r(3);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // r_z = kp * 0.1 - com_drift_z = 10.0 - 0 = 10.0 (v=0)
  EXPECT_NEAR(r(0), 0.0, 1e-8);   // x: no error
  EXPECT_NEAR(r(1), 0.0, 1e-8);   // y: no error
  EXPECT_NEAR(r(2), 10.0, 1e-8);  // z: 100 * 0.1
}

TEST_F(CoMTaskTest, JcomStructure) {
  CoMTask task;
  YAML::Node cfg;
  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  task.set_com_reference(cache_.com_position);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(3, n_vars);
  Eigen::VectorXd r(3);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // J_block[:, :nv] = Jcom
  EXPECT_TRUE(J.isApprox(cache_.Jcom));
}

TEST_F(CoMTaskTest, GainsUpdate) {
  CoMTask task;
  YAML::Node cfg;
  cfg["kp"] = 100.0;
  cfg["kd"] = 20.0;

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  Eigen::Vector3d target = cache_.com_position;
  target(0) += 0.1;
  task.set_com_reference(target);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(3, n_vars);
  Eigen::VectorXd r1(3);
  J.setZero();
  r1.setZero();
  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r1);

  // gains 2배
  task.set_gains(Eigen::Vector3d::Constant(200.0),
                 Eigen::Vector3d::Constant(20.0));

  Eigen::VectorXd r2(3);
  J.setZero();
  r2.setZero();
  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r2);

  // kp 2배 → r_x도 2배
  EXPECT_NEAR(r2(0), 2.0 * r1(0), 1e-10);
}

TEST_F(CoMTaskTest, ComDriftWithVelocity) {
  CoMTask task;
  YAML::Node cfg;
  cfg["kp"] = 0.0;
  cfg["kd"] = 0.0;

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  v(0) = 1.0;  // joint 0에 속도 부여
  cache_.update(q, v, contacts_);

  task.set_com_reference(cache_.com_position);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(3, n_vars);
  Eigen::VectorXd r(3);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // r = -com_drift (kp=kd=0, e_pos=0이므로 a_des=0)
  // v≠0이면 com_drift≠0
  EXPECT_TRUE(r.isApprox(-cache_.com_drift, 1e-10));
}

}  // namespace
}  // namespace rtc::tsid
