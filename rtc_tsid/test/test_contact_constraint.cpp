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

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;

class ContactConstraintTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.Build(*model_, config);

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
    contact_cfg_.contacts[0].frame_id = static_cast<int>(model_->getFrameId(frame_name_));
    contact_cfg_.contacts[0].contact_dim = 3;
    contact_cfg_.contacts[0].friction_coeff = 0.7;
    contact_cfg_.contacts[0].friction_faces = 4;
    contact_cfg_.max_contacts = 1;
    contact_cfg_.max_contact_vars = 3;

    cache_.Init(model_, rtc::tsid::ContactFrameIds(contact_cfg_));

    contacts_.Init(1);
    contacts_.contacts[0].active = true;
    contacts_.RecomputeActive(contact_cfg_);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  std::string frame_name_;
};

// Stage A-5a: ContactConstraint::EqDim returns max_contact_vars (fixed)
// regardless of activation. Inactive contacts contribute zero rows so the
// QP sees a stable equality block size across contact on/off transitions.
TEST_F(ContactConstraintTest, Dimensions) {
  ContactConstraint cc;
  YAML::Node cfg;
  cc.Init(*model_, robot_info_, cache_, cfg);
  cc.SetContactManager(&contact_cfg_);

  EXPECT_EQ(cc.EqDim(contacts_), 3);  // max_contact_vars = 3 (1 point contact)
  EXPECT_EQ(cc.IneqDim(contacts_), 0);

  // Deactivate → eq_dim stays at max_contact_vars; inactive rows must be
  // zero so the equality is trivially satisfied (0·a = 0).
  contacts_.contacts[0].active = false;
  contacts_.RecomputeActive(contact_cfg_);
  EXPECT_EQ(cc.EqDim(contacts_), 3);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);
  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;
  Eigen::MatrixXd A(3, n_vars);
  Eigen::VectorXd b(3);
  A.setZero();
  b.setZero();
  cc.ComputeEquality(cache_, contacts_, robot_info_, n_vars, A, b);
  EXPECT_LT(A.norm(), 1e-15);
  EXPECT_LT(b.norm(), 1e-15);
}

TEST_F(ContactConstraintTest, MatrixShape) {
  ContactConstraint cc;
  YAML::Node cfg;
  cc.Init(*model_, robot_info_, cache_, cfg);
  cc.SetContactManager(&contact_cfg_);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  // Stage A-5a: n_vars uses max_contact_vars (fixed) rather than
  // active_contact_vars — the QP variable dimension is constant.
  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;
  const int n_eq = cc.EqDim(contacts_);

  Eigen::MatrixXd A(n_eq, n_vars);
  Eigen::VectorXd b(n_eq);
  A.setZero();
  b.setZero();

  cc.ComputeEquality(cache_, contacts_, robot_info_, n_vars, A, b);

  // A[:, 0:nv] = Jc[:3, :] (should be non-zero on the active contact's row block)
  EXPECT_GT(A.leftCols(robot_info_.nv).norm(), 0.0);
  // A[:, nv:] = 0 (no coupling with λ in contact constraint)
  EXPECT_NEAR(A.rightCols(contact_cfg_.max_contact_vars).norm(), 0.0, 1e-15);
}

}  // namespace
}  // namespace rtc::tsid
