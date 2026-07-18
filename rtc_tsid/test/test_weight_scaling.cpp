// Stage A-5b: sqrt(s_i) cost scaling on tasks that touch per-contact rows.
//
// Cost surface should scale as s_i for cost = w·‖J_i·z − r_i‖², so both
// J and r blocks of the i-th contact must be multiplied by √s_i. Default
// s_i = 1 (post-Init) gives sqrt(1) = 1 → byte-identical to pre-A-5b.

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/tasks/contact_consistency_task.hpp"
#include "rtc_tsid/tasks/force_task.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;

class WeightScalingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.Build(*model_, config);

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
    contacts_.contacts[0].active = true;
    contacts_.contacts[0].activation = 1.0;
    contacts_.RecomputeActive(contact_cfg_);

    ref_.Init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated,
              contact_cfg_.max_contact_vars);
    Eigen::VectorXd q = pinocchio::neutral(*model_);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
    cache_.Update(q, v);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  ControlReference ref_;
};

// ForceTask rows for an s=0.5 contact must scale by sqrt(0.5) ≈ 0.7071
// vs the s=1 baseline. λ_des reference also scales by √s.
TEST_F(WeightScalingTest, ForceTaskBlockScaling) {
  ForceTask task;
  YAML::Node tcfg;
  task.Init(*model_, robot_info_, cache_, tcfg);
  task.SetContactManager(&contact_cfg_);

  // Set a non-zero λ_des so the r block has signal to scale.
  Eigen::VectorXd lambda_des(3);
  lambda_des << 0.0, 0.0, 5.0;
  task.SetForceReferences(lambda_des);

  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;
  const int rdim = contacts_.active_contact_vars;

  Eigen::MatrixXd J_full(rdim, n_vars);
  Eigen::VectorXd r_full(rdim);
  Eigen::MatrixXd J_half(rdim, n_vars);
  Eigen::VectorXd r_half(rdim);
  J_full.setZero();
  r_full.setZero();
  J_half.setZero();
  r_half.setZero();

  // Baseline: s = 1.
  contacts_.contacts[0].activation = 1.0;
  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J_full, r_full);

  // Half-active: s = 0.5.
  contacts_.contacts[0].activation = 0.5;
  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J_half, r_half);

  const double sqrt_half = std::sqrt(0.5);
  EXPECT_NEAR((J_half - sqrt_half * J_full).norm(), 0.0, 1e-12)
      << "ForceTask J block must scale by √s";
  EXPECT_NEAR((r_half - sqrt_half * r_full).norm(), 0.0, 1e-12)
      << "ForceTask r block must scale by √s";

  // Sanity: s = 1 baseline has non-zero r (reference fed through).
  EXPECT_GT(r_full.norm(), 0.0);
}

// ContactConsistencyTask J / r blocks must scale by sqrt(s) per contact.
TEST_F(WeightScalingTest, ContactConsistencyBlockScaling) {
  ContactConsistencyTask task;
  YAML::Node tcfg;
  task.Init(*model_, robot_info_, cache_, tcfg);
  task.SetContactManager(&contact_cfg_);

  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;

  // ResidualDim is updated by a previous call (one-tick lag pattern).
  // Drive a baseline pass first to set current_residual_dim_.
  contacts_.contacts[0].activation = 1.0;
  Eigen::MatrixXd J_warm(3, n_vars);
  Eigen::VectorXd r_warm(3);
  J_warm.setZero();
  r_warm.setZero();
  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J_warm, r_warm);

  const int rdim = task.ResidualDim();
  ASSERT_GT(rdim, 0);

  Eigen::MatrixXd J_full(rdim, n_vars);
  Eigen::VectorXd r_full(rdim);
  Eigen::MatrixXd J_half(rdim, n_vars);
  Eigen::VectorXd r_half(rdim);
  J_full.setZero();
  r_full.setZero();
  J_half.setZero();
  r_half.setZero();

  contacts_.contacts[0].activation = 1.0;
  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J_full, r_full);

  contacts_.contacts[0].activation = 0.5;
  task.ComputeResidual(cache_, ref_, contacts_, n_vars, J_half, r_half);

  const double sqrt_half = std::sqrt(0.5);
  EXPECT_NEAR((J_half - sqrt_half * J_full).norm(), 0.0, 1e-12)
      << "ContactConsistency J block must scale by √s";
  EXPECT_NEAR((r_half - sqrt_half * r_full).norm(), 0.0, 1e-12)
      << "ContactConsistency r block must scale by √s";

  // Sanity: s = 1 J has non-zero acceleration columns (Jc row block).
  EXPECT_GT(J_full.leftCols(robot_info_.nv).norm(), 0.0);
}

}  // namespace
}  // namespace rtc::tsid
