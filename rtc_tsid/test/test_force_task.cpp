#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/tasks/force_task.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;

class ForceTaskTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.Build(*model_, config);

    // 1개 point contact 설정
    YAML::Node contact_yaml;
    YAML::Node c1;
    c1["name"] = "fingertip";
    c1["frame"] = "panda_hand";
    c1["type"] = "point";
    c1["friction_coeff"] = 0.5;
    contact_yaml["contacts"].push_back(c1);

    contact_cfg_.Load(contact_yaml, *model_);
    cache_.Init(model_, rtc::tsid::ContactFrameIds(contact_cfg_));

    contacts_.Init(contact_cfg_.max_contacts);

    ref_.Init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated,
              contact_cfg_.max_contact_vars);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  ControlReference ref_;
};

TEST_F(ForceTaskTest, InitAndNameWeight) {
  ForceTask task;
  YAML::Node cfg;
  cfg["weight"] = 10.0;
  cfg["priority"] = 1;

  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);

  EXPECT_EQ(task.Name(), "force");
  EXPECT_DOUBLE_EQ(task.Weight(), 10.0);
  EXPECT_EQ(task.Priority(), 1);
  EXPECT_TRUE(task.IsActive());
}

TEST_F(ForceTaskTest, ResidualDimWithNoActiveContact) {
  ForceTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);

  // 모든 contact 비활성
  contacts_.RecomputeActive(contact_cfg_);
  task.UpdateResidualDim(contacts_);

  EXPECT_EQ(task.ResidualDim(), 0);
}

TEST_F(ForceTaskTest, ResidualDimWithActiveContact) {
  ForceTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);

  // contact 0 활성화 (point contact, dim=3)
  contacts_.contacts[0].active = true;
  contacts_.RecomputeActive(contact_cfg_);
  task.UpdateResidualDim(contacts_);

  EXPECT_EQ(task.ResidualDim(), 3);
}

TEST_F(ForceTaskTest, ComputeResidualIdentityOnLambda) {
  ForceTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);

  // contact 활성화
  contacts_.contacts[0].active = true;
  contacts_.RecomputeActive(contact_cfg_);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  // reference: [10, 0, 5]
  ref_.lambda_des.head(3) << 10.0, 0.0, 5.0;

  const int n_vars = robot_info_.nv + contacts_.active_contact_vars;  // 7 + 3
  const int rdim = contacts_.active_contact_vars;                     // 3
  Eigen::MatrixXd J(rdim, n_vars);
  Eigen::VectorXd r(rdim);
  J.setZero();
  r.setZero();

  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J, r);

  // J: 처음 nv 열은 0, 마지막 3열은 Identity
  for (int i = 0; i < rdim; ++i) {
    for (int j = 0; j < robot_info_.nv; ++j) {
      EXPECT_DOUBLE_EQ(J(i, j), 0.0) << "J(" << i << "," << j << ")";
    }
    for (int j = 0; j < rdim; ++j) {
      double expected = (i == j) ? 1.0 : 0.0;
      EXPECT_DOUBLE_EQ(J(i, robot_info_.nv + j), expected)
          << "J(" << i << "," << (robot_info_.nv + j) << ")";
    }
  }

  // r = lambda_des
  EXPECT_NEAR(r(0), 10.0, 1e-10);
  EXPECT_NEAR(r(1), 0.0, 1e-10);
  EXPECT_NEAR(r(2), 5.0, 1e-10);
}

TEST_F(ForceTaskTest, LocalForceReference) {
  ForceTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);

  contacts_.contacts[0].active = true;
  contacts_.RecomputeActive(contact_cfg_);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  // global reference (should be ignored)
  ref_.lambda_des.head(3) << 1.0, 2.0, 3.0;

  // local reference
  Eigen::VectorXd local_ref(3);
  local_ref << 100.0, 200.0, 300.0;
  task.SetForceReference(0, local_ref);

  const int n_vars = robot_info_.nv + contacts_.active_contact_vars;
  const int rdim = contacts_.active_contact_vars;
  Eigen::MatrixXd J(rdim, n_vars);
  Eigen::VectorXd r(rdim);
  J.setZero();
  r.setZero();

  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J, r);

  // local ref가 우선
  EXPECT_NEAR(r(0), 100.0, 1e-10);
  EXPECT_NEAR(r(1), 200.0, 1e-10);
  EXPECT_NEAR(r(2), 300.0, 1e-10);
}

TEST_F(ForceTaskTest, InactiveContactSkipped) {
  ForceTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);

  // 모든 contact 비활성
  contacts_.RecomputeActive(contact_cfg_);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  const int n_vars = robot_info_.nv;  // no active contacts → n_vars = nv
  // residual dim = 0 → J/r 크기도 0이므로 compute 호출만 확인
  Eigen::MatrixXd J(0, n_vars);
  Eigen::VectorXd r(0);

  // crash 없이 반환해야 함
  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J, r);
  EXPECT_EQ(contacts_.active_contact_vars, 0);
}

}  // namespace
}  // namespace rtc::tsid
