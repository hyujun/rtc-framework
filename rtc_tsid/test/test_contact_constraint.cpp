#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/constraints/contact_constraint.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf =
    RTC_PANDA_URDF_PATH;

class ContactConstraintTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.build(*model_, config);

    // Find a valid frame for contact
    for (size_t i = 0; i < model_->frames.size(); ++i) {
      if (model_->frames[i].name.find("link7") != std::string::npos) {
        frame_name_ = model_->frames[i].name;
        break;
      }
    }
    ASSERT_FALSE(frame_name_.empty());

    // 1 point contact
    contact_cfg_.contacts.resize(1);
    contact_cfg_.contacts[0].name = "ee";
    contact_cfg_.contacts[0].frame_name = frame_name_;
    contact_cfg_.contacts[0].frame_id =
        static_cast<int>(model_->getFrameId(frame_name_));
    contact_cfg_.contacts[0].contact_dim = 3;
    contact_cfg_.contacts[0].friction_coeff = 0.7;
    contact_cfg_.contacts[0].friction_faces = 4;
    contact_cfg_.max_contacts = 1;
    contact_cfg_.max_contact_vars = 3;

    cache_.init(model_, contact_cfg_);

    contacts_.init(1);
    contacts_.contacts[0].active = true;
    contacts_.recompute_active(contact_cfg_);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  std::string frame_name_;
};

TEST_F(ContactConstraintTest, Dimensions) {
  ContactConstraint cc;
  YAML::Node cfg;
  cc.init(*model_, robot_info_, cache_, cfg);
  cc.set_contact_manager(&contact_cfg_);

  EXPECT_EQ(cc.eq_dim(contacts_), 3);  // 1 point contact = 3
  EXPECT_EQ(cc.ineq_dim(contacts_), 0);

  // Deactivate → eq_dim = 0
  contacts_.contacts[0].active = false;
  contacts_.recompute_active(contact_cfg_);
  EXPECT_EQ(cc.eq_dim(contacts_), 0);
}

TEST_F(ContactConstraintTest, MatrixShape) {
  ContactConstraint cc;
  YAML::Node cfg;
  cc.init(*model_, robot_info_, cache_, cfg);
  cc.set_contact_manager(&contact_cfg_);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  const int n_vars = robot_info_.nv + contacts_.active_contact_vars;
  const int n_eq = cc.eq_dim(contacts_);

  Eigen::MatrixXd A(n_eq, n_vars);
  Eigen::VectorXd b(n_eq);
  A.setZero();
  b.setZero();

  cc.compute_equality(cache_, contacts_, robot_info_, n_vars, A, b);

  // A[:, 0:nv] = Jc[:3, :] (should be non-zero)
  EXPECT_GT(A.leftCols(robot_info_.nv).norm(), 0.0);
  // A[:, nv:] = 0 (no coupling with λ in contact constraint)
  EXPECT_NEAR(A.rightCols(contacts_.active_contact_vars).norm(), 0.0, 1e-15);
}

}  // namespace
}  // namespace rtc::tsid
