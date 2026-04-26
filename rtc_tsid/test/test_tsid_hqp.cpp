#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/formulation/formulation_factory.hpp"
#include "rtc_tsid/formulation/hqp_formulation.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf =
    RTC_PANDA_URDF_PATH;

class HQPFormulationTest : public ::testing::Test {
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
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  ControlReference ref_;
};

TEST_F(HQPFormulationTest, FactoryCreatesHQP) {
  YAML::Node config;
  config["formulation_type"] = "hqp";

  auto f = create_formulation(*model_, robot_info_, contact_cfg_, config);
  ASSERT_NE(f, nullptr);
  EXPECT_EQ(f->type(), "hqp");
}

TEST_F(HQPFormulationTest, SingleLevelConverges) {
  YAML::Node config;
  config["formulation_type"] = "hqp";

  auto formulation = create_formulation(
      *model_, robot_info_, contact_cfg_, config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  task_cfg["kp"] = 100.0;
  task_cfg["kd"] = 20.0;
  task_cfg["priority"] = 0;
  posture->init(*model_, robot_info_, cache_, task_cfg);
  formulation->add_task(std::move(posture));

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  ref_.q_des = q;
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const auto& result = formulation->solve(cache_, ref_, contacts_, robot_info_);

  ASSERT_TRUE(result.converged);
  EXPECT_EQ(result.levels_solved, 1);

  Eigen::VectorXd a_opt = result.x_opt.head(robot_info_.nv);
  EXPECT_LT(a_opt.norm(), 1e-4);
}

TEST_F(HQPFormulationTest, MultiLevelStrictHierarchy) {
  // Level 0: posture → a_des ≈ 0 (q_des = q_current)
  // Level 1: posture → a_des = kp*0.5 = 50 (different target)
  // Since J_level0 = I (full rank), null-space is empty
  // → Level 1 cannot change anything → a remains ≈ 0

  YAML::Node config;
  config["formulation_type"] = "hqp";

  auto formulation = create_formulation(
      *model_, robot_info_, contact_cfg_, config);

  auto posture_high = std::make_unique<PostureTask>();
  YAML::Node cfg_high;
  cfg_high["kp"] = 100.0;
  cfg_high["kd"] = 20.0;
  cfg_high["weight"] = 1.0;
  cfg_high["priority"] = 0;
  posture_high->init(*model_, robot_info_, cache_, cfg_high);
  formulation->add_task(std::move(posture_high));

  auto posture_low = std::make_unique<PostureTask>();
  YAML::Node cfg_low;
  cfg_low["kp"] = 100.0;
  cfg_low["kd"] = 20.0;
  cfg_low["weight"] = 1.0;
  cfg_low["priority"] = 1;
  posture_low->init(*model_, robot_info_, cache_, cfg_low);

  Eigen::VectorXd q_neutral = pinocchio::neutral(*model_);
  Eigen::VectorXd q_offset = q_neutral;
  q_offset.head(robot_info_.nv) +=
      Eigen::VectorXd::Constant(robot_info_.nv, 0.5);
  posture_low->set_reference(
      q_offset,
      Eigen::VectorXd::Zero(robot_info_.nv),
      Eigen::VectorXd::Zero(robot_info_.nv));
  formulation->add_task(std::move(posture_low));

  Eigen::VectorXd q = q_neutral;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  ref_.q_des = q;
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const auto& result = formulation->solve(cache_, ref_, contacts_, robot_info_);

  ASSERT_TRUE(result.converged);
  EXPECT_EQ(result.levels_solved, 2);

  // Level 0 dominates → a ≈ 0
  Eigen::VectorXd a_opt = result.x_opt.head(robot_info_.nv);
  EXPECT_LT(a_opt.norm(), 0.5)
      << "Level 0 should dominate. a_opt = " << a_opt.transpose();
}

TEST_F(HQPFormulationTest, PresetSetsCorrectPriority) {
  YAML::Node config;
  config["formulation_type"] = "hqp";

  auto formulation = create_formulation(
      *model_, robot_info_, contact_cfg_, config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  posture->init(*model_, robot_info_, cache_, task_cfg);
  formulation->add_task(std::move(posture));

  PhasePreset preset;
  preset.phase_name = "test";
  TaskPreset tp;
  tp.task_name = "posture";
  tp.active = true;
  tp.weight = 2.0;
  tp.priority = 3;
  preset.task_presets.push_back(tp);

  formulation->apply_preset(preset);

  auto* task = formulation->get_task("posture");
  ASSERT_NE(task, nullptr);
  EXPECT_DOUBLE_EQ(task->weight(), 2.0);
  EXPECT_EQ(task->priority(), 3);
}

}  // namespace
}  // namespace rtc::tsid
