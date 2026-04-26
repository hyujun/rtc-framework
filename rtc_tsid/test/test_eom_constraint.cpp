#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/constraints/eom_constraint.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf =
    RTC_PANDA_URDF_PATH;

class EomConstraintTest : public ::testing::Test {
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

TEST_F(EomConstraintTest, FixedBaseNoDimension) {
  EomConstraint eom;
  YAML::Node cfg;
  eom.init(*model_, robot_info_, cache_, cfg);

  // Fixed-base → eq_dim = 0
  EXPECT_EQ(eom.eq_dim(contacts_), 0);
  EXPECT_EQ(eom.ineq_dim(contacts_), 0);
  EXPECT_EQ(eom.name(), "eom");
}

TEST_F(EomConstraintTest, FloatingBaseHasDimension) {
  // Override to floating-base
  YAML::Node config;
  config["floating_base"] = true;
  config["n_actuated"] = robot_info_.nv - 6;

  RobotModelInfo fb_info;
  fb_info.build(*model_, config);

  EomConstraint eom;
  YAML::Node cfg;
  eom.init(*model_, fb_info, cache_, cfg);

  // Floating-base → eq_dim = 6
  EXPECT_EQ(eom.eq_dim(contacts_), 6);
  EXPECT_EQ(eom.ineq_dim(contacts_), 0);
}

TEST_F(EomConstraintTest, FloatingBaseMatrixDimensions) {
  YAML::Node config;
  config["floating_base"] = true;
  config["n_actuated"] = robot_info_.nv - 6;

  RobotModelInfo fb_info;
  fb_info.build(*model_, config);

  EomConstraint eom;
  YAML::Node cfg;
  eom.init(*model_, fb_info, cache_, cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  const int n_eq = eom.eq_dim(contacts_);
  const int n_vars = robot_info_.nv;  // no contacts

  Eigen::MatrixXd A(n_eq, n_vars);
  Eigen::VectorXd b(n_eq);
  A.setZero();
  b.setZero();

  eom.compute_equality(cache_, contacts_, fb_info, n_vars, A, b);

  // A should be non-zero (P*M)
  EXPECT_GT(A.norm(), 0.0);
  // b should be non-zero (gravity)
  EXPECT_GT(b.norm(), 0.0);
}

}  // namespace
}  // namespace rtc::tsid
