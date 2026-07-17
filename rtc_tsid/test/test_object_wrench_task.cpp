#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/contact/contact_manager.hpp"
#include "rtc_tsid/tasks/object_wrench_task.hpp"
#include "rtc_tsid/types/object_frame.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

#include <string>
#include <vector>

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;

class ObjectWrenchTaskTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.Build(*model_, config);

    q_ = pinocchio::neutral(*model_);
    v_ = Eigen::VectorXd::Zero(robot_info_.nv);
  }

  void Configure(const std::vector<std::pair<std::string, int>>& contacts) {
    contact_cfg_ = ContactManagerConfig{};
    contact_cfg_.contacts.resize(contacts.size());
    int total = 0;
    for (size_t i = 0; i < contacts.size(); ++i) {
      auto& c = contact_cfg_.contacts[i];
      c.name = "c" + std::to_string(i);
      c.frame_name = contacts[i].first;
      c.frame_id = static_cast<int>(model_->getFrameId(c.frame_name));
      c.contact_dim = contacts[i].second;
      total += c.contact_dim;
    }
    contact_cfg_.max_contacts = static_cast<int>(contacts.size());
    contact_cfg_.max_contact_vars = total;

    cache_.Init(model_, rtc::tsid::ContactFrameIds(contact_cfg_));

    contacts_.Init(static_cast<int>(contacts.size()));
    for (auto& e : contacts_.contacts) {
      e.active = true;
    }
    contacts_.RecomputeActive(contact_cfg_);
    cache_.Update(q_, v_);

    mgr_.Init(contact_cfg_, robot_info_.nv);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  ContactManager mgr_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
};

// J_block layout: left nv columns must be zero (no `a` dependence),
// right max_contact_vars columns must equal the active-G unpacked into
// fixed contact slot offsets.
TEST_F(ObjectWrenchTaskTest, JacobianStructure_ZerosThenG) {
  Configure({{"panda_link5", 3}, {"panda_link7", 3}});

  ObjectWrenchTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);
  task.SetContactManagerRuntime(&mgr_);

  ObjectFrame obj;
  obj.p_w = Eigen::Vector3d{0.1, 0.0, 0.4};
  task.SetObjectFrame(&obj);

  task.UpdateResidualDim(contacts_);
  ASSERT_EQ(task.ResidualDim(), 6);

  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;
  Eigen::MatrixXd J(6, n_vars);
  Eigen::VectorXd r(6);
  J.setZero();
  r.setZero();
  ControlReference ref;
  task.ComputeResidual(cache_, ref, contacts_, n_vars, J, r);

  // Left nv columns: identically zero (no `a` dependence).
  EXPECT_LT(J.leftCols(robot_info_.nv).norm(), 1e-15);

  // Right max_contact_vars columns: must equal G assembled directly via
  // ContactManager (active-packed) since both contacts are active and
  // active-packed offsets equal slot offsets when all contacts are on.
  Eigen::MatrixXd G_ref(6, contact_cfg_.max_contact_vars);
  G_ref.setZero();
  mgr_.ComputeGraspMatrix(cache_, contacts_, obj, G_ref);

  const Eigen::MatrixXd J_lambda = J.rightCols(contact_cfg_.max_contact_vars);
  EXPECT_LT((J_lambda - G_ref).norm(), 1e-12);
}

// Residual r equals the configured desired wrench exactly (no kinematic
// term — pure force-feedback task).
TEST_F(ObjectWrenchTaskTest, ResidualMatchesDesiredWrench) {
  Configure({{"panda_link5", 3}});

  ObjectWrenchTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);
  task.SetContactManagerRuntime(&mgr_);

  ObjectFrame obj;
  task.SetObjectFrame(&obj);

  Eigen::Matrix<double, 6, 1> w_des;
  w_des << 1.0, -2.0, 9.81, 0.5, -0.25, 0.1;
  task.SetDesiredWrench(w_des);

  task.UpdateResidualDim(contacts_);
  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;
  Eigen::MatrixXd J(6, n_vars);
  Eigen::VectorXd r(6);
  J.setZero();
  r.setZero();
  ControlReference ref;
  task.ComputeResidual(cache_, ref, contacts_, n_vars, J, r);

  EXPECT_LT((r - w_des).norm(), 1e-15);
}

// ResidualDim flips with active count: 0 contacts → 0, ≥1 contact → 6.
// Inactive contact column blocks in J_block stay zero (active slot offset
// scheme = same convention as ForceTask).
TEST_F(ObjectWrenchTaskTest, DimensionsActiveCount) {
  Configure({{"panda_link3", 3}, {"panda_link5", 3}, {"panda_link7", 3}});

  ObjectWrenchTask task;
  YAML::Node cfg;
  task.Init(*model_, robot_info_, cache_, cfg);
  task.SetContactManager(&contact_cfg_);
  task.SetContactManagerRuntime(&mgr_);

  ObjectFrame obj;
  task.SetObjectFrame(&obj);

  // All active → residual_dim 6.
  task.UpdateResidualDim(contacts_);
  EXPECT_EQ(task.ResidualDim(), 6);

  // Deactivate all → residual_dim 0.
  for (auto& e : contacts_.contacts) {
    e.active = false;
  }
  contacts_.RecomputeActive(contact_cfg_);
  task.UpdateResidualDim(contacts_);
  EXPECT_EQ(task.ResidualDim(), 0);

  // Activate only middle → residual_dim 6, but J's λ columns must be 0
  // for the inactive contact slots (slots 0..2 and 6..8) and equal G
  // for the active slot (3..5).
  contacts_.contacts[1].active = true;
  contacts_.RecomputeActive(contact_cfg_);
  task.UpdateResidualDim(contacts_);
  EXPECT_EQ(task.ResidualDim(), 6);

  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;
  Eigen::MatrixXd J(6, n_vars);
  Eigen::VectorXd r(6);
  J.setZero();
  r.setZero();
  ControlReference ref;
  task.ComputeResidual(cache_, ref, contacts_, n_vars, J, r);

  const int nv = robot_info_.nv;
  // Slots 0..2 (contact 0, inactive): all zero.
  EXPECT_LT(J.block(0, nv + 0, 6, 3).norm(), 1e-15);
  // Slots 6..8 (contact 2, inactive): all zero.
  EXPECT_LT(J.block(0, nv + 6, 6, 3).norm(), 1e-15);

  // Slots 3..5 (contact 1, active): equal to the active-packed G column.
  Eigen::MatrixXd G_active(6, 3);
  G_active.setZero();
  mgr_.ComputeGraspMatrix(cache_, contacts_, obj, G_active);
  EXPECT_LT((J.block(0, nv + 3, 6, 3) - G_active).norm(), 1e-12);
}

}  // namespace
}  // namespace rtc::tsid
