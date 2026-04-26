#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/constraints/eom_constraint.hpp"
#include "rtc_tsid/formulation/formulation_factory.hpp"
#include "rtc_tsid/formulation/wqp_formulation.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf =
    RTC_PANDA_URDF_PATH;

class WQPFormulationTest : public ::testing::Test {
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

// ──────────────────────────────────────────────
// Posture-only WQP (no contacts)
// ──────────────────────────────────────────────
TEST_F(WQPFormulationTest, PostureOnlyConverges) {
  YAML::Node formulation_config;
  formulation_config["formulation_type"] = "wqp";

  auto formulation = create_formulation(
      *model_, robot_info_, contact_cfg_, formulation_config);
  ASSERT_NE(formulation, nullptr);
  EXPECT_EQ(formulation->type(), "wqp");

  // PostureTask 등록
  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  task_cfg["kp"] = 100.0;
  task_cfg["kd"] = 20.0;
  task_cfg["weight"] = 1.0;
  posture->init(*model_, robot_info_, cache_, task_cfg);
  formulation->add_task(std::move(posture));

  // q_des = q_current, v_des = 0 → 가속도 ≈ 0
  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  ref_.q_des = q;
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const auto& result = formulation->solve(cache_, ref_, contacts_, robot_info_);

  ASSERT_TRUE(result.converged);
  EXPECT_EQ(result.levels_solved, 1);
  EXPECT_GT(result.solve_time_us, 0.0);

  // a_opt ≈ 0 (at desired posture with zero velocity)
  Eigen::VectorXd a_opt = result.x_opt.head(robot_info_.nv);
  EXPECT_LT(a_opt.norm(), 1e-4) << "a_opt = " << a_opt.transpose();
}

TEST_F(WQPFormulationTest, PostureTrackingNonZeroAccel) {
  YAML::Node formulation_config;

  auto formulation = std::make_unique<WQPFormulation>();
  formulation->init(*model_, robot_info_, contact_cfg_, formulation_config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  task_cfg["kp"] = 100.0;
  task_cfg["kd"] = 20.0;
  posture->init(*model_, robot_info_, cache_, task_cfg);
  formulation->add_task(std::move(posture));

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  // q_des != q → non-zero acceleration
  ref_.q_des = q;
  ref_.q_des.head(robot_info_.nv) +=
      Eigen::VectorXd::Constant(robot_info_.nv, 0.1);
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const auto& result = formulation->solve(cache_, ref_, contacts_, robot_info_);

  ASSERT_TRUE(result.converged);
  Eigen::VectorXd a_opt = result.x_opt.head(robot_info_.nv);

  // 가속도가 q_des 방향을 향해야 (kp * 0.1 = 10.0)
  for (int i = 0; i < robot_info_.nv; ++i) {
    EXPECT_GT(a_opt(i), 0.0) << "Joint " << i << " should accelerate toward q_des";
  }
}

TEST_F(WQPFormulationTest, GravityCompensationTorque) {
  // Posture at rest → τ = M*a + h, a ≈ 0 → τ ≈ h ≈ g
  YAML::Node formulation_config;

  auto formulation = std::make_unique<WQPFormulation>();
  formulation->init(*model_, robot_info_, contact_cfg_, formulation_config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  task_cfg["kp"] = 100.0;
  task_cfg["kd"] = 20.0;
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

  // τ = S * (M * a_opt + h)  (no contacts → Jcᵀλ = 0)
  Eigen::VectorXd a_opt = result.x_opt.head(robot_info_.nv);
  Eigen::VectorXd tau = robot_info_.S * (cache_.M * a_opt + cache_.h);

  // a_opt ≈ 0 → τ ≈ g (gravity compensation)
  Eigen::VectorXd g = robot_info_.S * cache_.g;
  EXPECT_LT((tau - g).norm(), 0.1)
      << "tau should be close to gravity compensation\n"
      << "  tau = " << tau.transpose() << "\n"
      << "  g   = " << g.transpose();
}

TEST_F(WQPFormulationTest, FactoryCreatesWQP) {
  YAML::Node config;
  config["formulation_type"] = "wqp";

  auto f = create_formulation(*model_, robot_info_, contact_cfg_, config);
  ASSERT_NE(f, nullptr);
  EXPECT_EQ(f->type(), "wqp");
}

TEST_F(WQPFormulationTest, FactoryDefaultIsWQP) {
  YAML::Node config;  // no formulation_type

  auto f = create_formulation(*model_, robot_info_, contact_cfg_, config);
  ASSERT_NE(f, nullptr);
  EXPECT_EQ(f->type(), "wqp");
}

TEST_F(WQPFormulationTest, PresetApplication) {
  YAML::Node formulation_config;
  auto formulation = std::make_unique<WQPFormulation>();
  formulation->init(*model_, robot_info_, contact_cfg_, formulation_config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  posture->init(*model_, robot_info_, cache_, task_cfg);
  formulation->add_task(std::move(posture));

  // Apply preset that deactivates posture
  PhasePreset preset;
  preset.phase_name = "test";
  TaskPreset tp;
  tp.task_name = "posture";
  tp.active = false;
  tp.weight = 0.5;
  tp.priority = 2;
  preset.task_presets.push_back(tp);

  formulation->apply_preset(preset);

  auto* task = formulation->get_task("posture");
  ASSERT_NE(task, nullptr);
  EXPECT_FALSE(task->is_active());
  EXPECT_DOUBLE_EQ(task->weight(), 0.5);
  EXPECT_EQ(task->priority(), 2);
}

TEST_F(WQPFormulationTest, ConsecutiveSolvesWarmStart) {
  YAML::Node formulation_config;
  auto formulation = std::make_unique<WQPFormulation>();
  formulation->init(*model_, robot_info_, contact_cfg_, formulation_config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  task_cfg["kp"] = 100.0;
  task_cfg["kd"] = 20.0;
  posture->init(*model_, robot_info_, cache_, task_cfg);
  formulation->add_task(std::move(posture));

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  ref_.q_des = q;
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  // Multiple solves should all converge
  for (int i = 0; i < 10; ++i) {
    const auto& result =
        formulation->solve(cache_, ref_, contacts_, robot_info_);
    ASSERT_TRUE(result.converged) << "Solve " << i << " failed";
  }
}

}  // namespace
}  // namespace rtc::tsid
