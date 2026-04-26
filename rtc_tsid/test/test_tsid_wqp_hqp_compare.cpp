#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/controller/tsid_controller.hpp"
#include "rtc_tsid/formulation/formulation_factory.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf =
    RTC_PANDA_URDF_PATH;

class WQPvsHQPTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.build(*model_, config);

    contact_cfg_.max_contacts = 0;
    contact_cfg_.max_contact_vars = 0;

    cache_.init(model_, contact_cfg_);
    contacts_.init(0);
    ref_.init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated, 0);

    q_ = pinocchio::neutral(*model_);
    v_ = Eigen::VectorXd::Zero(robot_info_.nv);
  }

  std::unique_ptr<FormulationBase> make_formulation(const std::string& type) {
    YAML::Node config;
    config["formulation_type"] = type;

    auto f = create_formulation(*model_, robot_info_, contact_cfg_, config);

    auto posture = std::make_unique<PostureTask>();
    YAML::Node task_cfg;
    task_cfg["kp"] = 100.0;
    task_cfg["kd"] = 20.0;
    task_cfg["priority"] = 0;
    posture->init(*model_, robot_info_, cache_, task_cfg);
    f->add_task(std::move(posture));

    return f;
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  ControlReference ref_;
  Eigen::VectorXd q_, v_;
};

// Single-level (1 task, priority 0): WQP and HQP should produce ≈ same result
TEST_F(WQPvsHQPTest, SingleLevelEquivalence) {
  auto wqp = make_formulation("wqp");
  auto hqp = make_formulation("hqp");

  cache_.update(q_, v_, contacts_);
  ref_.q_des = q_;
  ref_.v_des = v_;
  ref_.a_des.setZero(robot_info_.nv);

  const auto& r_wqp = wqp->solve(cache_, ref_, contacts_, robot_info_);
  const auto& r_hqp = hqp->solve(cache_, ref_, contacts_, robot_info_);

  ASSERT_TRUE(r_wqp.converged);
  ASSERT_TRUE(r_hqp.converged);

  const int nv = robot_info_.nv;
  Eigen::VectorXd diff = r_wqp.x_opt.head(nv) - r_hqp.x_opt.head(nv);
  EXPECT_LT(diff.norm(), 1e-3)
      << "WQP and HQP should match for single level\n"
      << "  WQP: " << r_wqp.x_opt.head(nv).transpose() << "\n"
      << "  HQP: " << r_hqp.x_opt.head(nv).transpose();
}

// With position error: WQP and HQP should agree on direction
TEST_F(WQPvsHQPTest, SingleLevelWithError) {
  auto wqp = make_formulation("wqp");
  auto hqp = make_formulation("hqp");

  cache_.update(q_, v_, contacts_);

  ref_.q_des = q_;
  ref_.q_des.head(robot_info_.nv) +=
      Eigen::VectorXd::Constant(robot_info_.nv, 0.1);
  ref_.v_des = v_;
  ref_.a_des.setZero(robot_info_.nv);

  const auto& r_wqp = wqp->solve(cache_, ref_, contacts_, robot_info_);
  const auto& r_hqp = hqp->solve(cache_, ref_, contacts_, robot_info_);

  ASSERT_TRUE(r_wqp.converged);
  ASSERT_TRUE(r_hqp.converged);

  const int nv = robot_info_.nv;
  Eigen::VectorXd diff = r_wqp.x_opt.head(nv) - r_hqp.x_opt.head(nv);
  EXPECT_LT(diff.norm(), 1e-2)
      << "diff norm = " << diff.norm();
}

// TSIDController end-to-end: gravity compensation torque
TEST_F(WQPvsHQPTest, TSIDControllerGravityComp) {
  TSIDController ctrl;

  YAML::Node config;
  config["formulation_type"] = "wqp";

  ctrl.init(*model_, robot_info_, config);

  // Register posture task
  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  task_cfg["kp"] = 100.0;
  task_cfg["kd"] = 20.0;
  posture->init(*model_, robot_info_, cache_, task_cfg);
  ctrl.formulation().add_task(std::move(posture));

  cache_.update(q_, v_, contacts_);

  ref_.q_des = q_;
  ref_.v_des = v_;
  ref_.a_des.setZero(robot_info_.nv);

  ControlState state;
  state.q = q_;
  state.v = v_;

  auto output = ctrl.compute(state, ref_, cache_, contacts_);

  ASSERT_TRUE(output.qp_converged);
  EXPECT_EQ(output.solve_levels, 1);

  // τ ≈ gravity compensation
  Eigen::VectorXd g = robot_info_.S * cache_.g;
  EXPECT_LT((output.tau.head(robot_info_.n_actuated) - g).norm(), 0.5)
      << "tau should approximate gravity comp\n"
      << "  tau = " << output.tau.head(robot_info_.n_actuated).transpose()
      << "\n"
      << "  g   = " << g.transpose();
}

}  // namespace
}  // namespace rtc::tsid
