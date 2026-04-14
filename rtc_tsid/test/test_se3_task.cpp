#include <gtest/gtest.h>

#include <cmath>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/tasks/se3_task.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf =
    "/usr/local/share/example-robot-data/robots/panda_description/urdf/"
    "panda.urdf";

class SE3TaskTest : public ::testing::Test {
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

TEST_F(SE3TaskTest, InitAndDimension6D) {
  SE3Task task;
  YAML::Node cfg;
  cfg["frame"] = "panda_hand";
  cfg["mask"] = std::vector<int>{1, 1, 1, 1, 1, 1};
  cfg["kp"] = 100.0;
  cfg["kd"] = 20.0;
  cfg["weight"] = 50.0;
  cfg["priority"] = 1;

  task.init(*model_, robot_info_, cache_, cfg);

  EXPECT_EQ(task.residual_dim(), 6);
  EXPECT_EQ(task.name(), "se3");
  EXPECT_TRUE(task.is_active());
  EXPECT_DOUBLE_EQ(task.weight(), 50.0);
  EXPECT_EQ(task.priority(), 1);
}

TEST_F(SE3TaskTest, InitAndDimensionPositionOnly) {
  SE3Task task;
  YAML::Node cfg;
  cfg["frame"] = "panda_hand";
  cfg["mask"] = std::vector<int>{1, 1, 1, 0, 0, 0};

  task.init(*model_, robot_info_, cache_, cfg);

  EXPECT_EQ(task.residual_dim(), 3);
}

TEST_F(SE3TaskTest, InitAndDimensionRotationOnly) {
  SE3Task task;
  YAML::Node cfg;
  cfg["frame"] = "panda_hand";
  cfg["mask"] = std::vector<int>{0, 0, 0, 1, 1, 1};

  task.init(*model_, robot_info_, cache_, cfg);

  EXPECT_EQ(task.residual_dim(), 3);
}

TEST_F(SE3TaskTest, CustomName) {
  SE3Task task;
  YAML::Node cfg;
  cfg["frame"] = "panda_hand";
  cfg["name"] = "se3_tcp";

  task.init(*model_, robot_info_, cache_, cfg);

  EXPECT_EQ(task.name(), "se3_tcp");
}

TEST_F(SE3TaskTest, ZeroErrorAtCurrentPose) {
  SE3Task task;
  YAML::Node cfg;
  cfg["frame"] = "panda_hand";
  cfg["kp"] = 100.0;
  cfg["kd"] = 20.0;

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  // 현재 pose를 목표로 설정 → error = 0
  const auto& rf = cache_.registered_frames[0];
  task.set_se3_reference(rf.oMf);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(6, n_vars);
  Eigen::VectorXd r(6);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // r ≈ -dJv (q=neutral, v=0이므로 dJv=0 → r ≈ 0)
  EXPECT_LT(r.norm(), 1e-10);

  // J는 frame Jacobian이므로 non-zero
  EXPECT_GT(J.norm(), 0.0);
}

TEST_F(SE3TaskTest, PositionErrorResponse) {
  SE3Task task;
  YAML::Node cfg;
  cfg["frame"] = "panda_hand";
  cfg["mask"] = std::vector<int>{1, 1, 1, 0, 0, 0};  // position only
  cfg["kp"] = 100.0;
  cfg["kd"] = 20.0;

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  // 현재 pose에서 z축으로 0.1m offset 추가
  auto des = cache_.registered_frames[0].oMf;
  des.translation()(2) += 0.1;
  task.set_se3_reference(des);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(3, n_vars);
  Eigen::VectorXd r(3);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // r = Kp * e_pos - dJv = 100 * [0, 0, 0.1] - 0
  // v=0 → velocity error=0 → r_z = 100*0.1 = 10
  EXPECT_NEAR(r(0), 0.0, 1e-10);   // x: no error
  EXPECT_NEAR(r(1), 0.0, 1e-10);   // y: no error
  EXPECT_NEAR(r(2), 10.0, 1e-10);  // z: Kp * 0.1 = 10
}

TEST_F(SE3TaskTest, OrientationErrorSmallAngle) {
  SE3Task task;
  YAML::Node cfg;
  cfg["frame"] = "panda_hand";
  cfg["mask"] = std::vector<int>{0, 0, 0, 1, 1, 1};  // rotation only
  cfg["kp"] = 50.0;
  cfg["kd"] = 10.0;

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  // 현재 pose에서 z축 0.1rad 회전 추가
  auto des = cache_.registered_frames[0].oMf;
  Eigen::AngleAxisd rot(0.1, Eigen::Vector3d::UnitZ());
  des.rotation() = des.rotation() * rot.toRotationMatrix();
  task.set_se3_reference(des);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(3, n_vars);
  Eigen::VectorXd r(3);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // orientation error의 크기 = 0.1rad → r ≈ Kp * 0.1 = 5.0 (z축)
  EXPECT_GT(r.norm(), 1.0);  // 유의미한 response
}

TEST_F(SE3TaskTest, OrientationErrorNearPi) {
  // log3 singularity 근처 (θ → π) 에서도 crash 없이 동작
  SE3Task task;
  YAML::Node cfg;
  cfg["frame"] = "panda_hand";
  cfg["kp"] = 50.0;
  cfg["kd"] = 10.0;

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  // 현재 pose에서 x축 π-0.001 rad 회전 (거의 π)
  auto des = cache_.registered_frames[0].oMf;
  Eigen::AngleAxisd rot(M_PI - 0.001, Eigen::Vector3d::UnitX());
  des.rotation() = des.rotation() * rot.toRotationMatrix();
  task.set_se3_reference(des);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(6, n_vars);
  Eigen::VectorXd r(6);
  J.setZero();
  r.setZero();

  // crash/NaN 없이 compute 가능해야 함
  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  EXPECT_FALSE(std::isnan(r.norm()));
  EXPECT_FALSE(std::isinf(r.norm()));
  EXPECT_GT(r.norm(), 0.0);
}

TEST_F(SE3TaskTest, GainsUpdate) {
  SE3Task task;
  YAML::Node cfg;
  cfg["frame"] = "panda_hand";
  cfg["kp"] = 100.0;
  cfg["kd"] = 20.0;

  task.init(*model_, robot_info_, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  auto des = cache_.registered_frames[0].oMf;
  des.translation()(0) += 0.1;
  task.set_se3_reference(des);

  // 첫 번째 계산 (kp=100)
  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(6, n_vars);
  Eigen::VectorXd r1(6);
  J.setZero();
  r1.setZero();
  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r1);

  // gains 변경 (kp=200)
  Eigen::Matrix<double, 6, 1> new_kp = Eigen::Matrix<double, 6, 1>::Constant(200.0);
  Eigen::Matrix<double, 6, 1> new_kd = Eigen::Matrix<double, 6, 1>::Constant(20.0);
  task.set_gains(new_kp, new_kd);

  Eigen::VectorXd r2(6);
  J.setZero();
  r2.setZero();
  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r2);

  // kp 2배 → position 관련 residual도 2배
  // r = kp * e_pos + kd * e_vel - dJv, v=0이므로 e_vel=0, dJv=0
  EXPECT_NEAR(r2(0), 2.0 * r1(0), 1e-10);
}

TEST_F(SE3TaskTest, JBlockDimensionWithMask) {
  SE3Task task;
  YAML::Node cfg;
  cfg["frame"] = "panda_hand";
  cfg["mask"] = std::vector<int>{1, 0, 1, 0, 1, 0};  // 3 axes active

  task.init(*model_, robot_info_, cache_, cfg);
  EXPECT_EQ(task.residual_dim(), 3);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  auto des = cache_.registered_frames[0].oMf;
  task.set_se3_reference(des);

  const int n_vars = robot_info_.nv;
  Eigen::MatrixXd J(3, n_vars);
  Eigen::VectorXd r(3);
  J.setZero();
  r.setZero();

  task.compute_residual(cache_, ref_, contacts_, n_vars, J, r);

  // J는 masked Jacobian → rows = {0, 2, 4} of full J
  const auto& rf = cache_.registered_frames[0];
  EXPECT_TRUE(J.row(0).isApprox(rf.J.row(0).head(n_vars)));  // vx
  EXPECT_TRUE(J.row(1).isApprox(rf.J.row(2).head(n_vars)));  // vz
  EXPECT_TRUE(J.row(2).isApprox(rf.J.row(4).head(n_vars)));  // wy
}

}  // namespace
}  // namespace rtc::tsid
