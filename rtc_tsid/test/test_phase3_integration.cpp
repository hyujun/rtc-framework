#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/constraints/eom_constraint.hpp"
#include "rtc_tsid/constraints/joint_limit_constraint.hpp"
#include "rtc_tsid/constraints/torque_limit_constraint.hpp"
#include "rtc_tsid/formulation/wqp_formulation.hpp"
#include "rtc_tsid/formulation/hqp_formulation.hpp"
#include "rtc_tsid/tasks/com_task.hpp"
#include "rtc_tsid/tasks/force_task.hpp"
#include "rtc_tsid/tasks/momentum_task.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"
#include "rtc_tsid/tasks/se3_task.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf =
    RTC_PANDA_URDF_PATH;

// ──────────────────────────────────────────────
// Panda 7DoF fixed-base 통합 테스트 (WQP)
// PostureTask + SE3Task + TorqueLimitConstraint + JointLimitConstraint
// ──────────────────────────────────────────────
class Phase3WQPTest : public ::testing::Test {
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

TEST_F(Phase3WQPTest, PostureAndSE3Solve) {
  // Formulation 생성
  YAML::Node form_cfg;
  form_cfg["formulation_type"] = "wqp";
  WQPFormulation formulation;
  formulation.init(*model_, robot_info_, contact_cfg_, form_cfg);

  // PostureTask
  auto posture = std::make_unique<PostureTask>();
  YAML::Node posture_cfg;
  posture_cfg["kp"] = 10.0;
  posture_cfg["kd"] = 1.0;
  posture_cfg["weight"] = 1.0;
  posture->init(*model_, robot_info_, cache_, posture_cfg);
  formulation.add_task(std::move(posture));

  // SE3Task
  auto se3 = std::make_unique<SE3Task>();
  YAML::Node se3_cfg;
  se3_cfg["frame"] = "panda_hand";
  se3_cfg["name"] = "se3_tcp";
  se3_cfg["kp"] = 100.0;
  se3_cfg["kd"] = 20.0;
  se3_cfg["weight"] = 100.0;
  se3->init(*model_, robot_info_, cache_, se3_cfg);

  // 현재 pose를 목표로 설정 (joint limit 중앙 사용)
  Eigen::VectorXd q = 0.5 * (robot_info_.q_lower + robot_info_.q_upper);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  auto* se3_ptr = se3.get();
  se3_ptr->set_se3_reference(cache_.registered_frames[0].oMf);
  formulation.add_task(std::move(se3));

  // TorqueLimitConstraint
  auto torque = std::make_unique<TorqueLimitConstraint>();
  YAML::Node tc_cfg;
  torque->init(*model_, robot_info_, cache_, tc_cfg);
  formulation.add_constraint(std::move(torque));

  // JointLimitConstraint
  auto jlimit = std::make_unique<JointLimitConstraint>();
  YAML::Node jl_cfg;
  jl_cfg["dt"] = 0.002;
  jl_cfg["position_margin"] = 0.05;
  jl_cfg["velocity_margin"] = 0.1;
  jlimit->init(*model_, robot_info_, cache_, jl_cfg);
  formulation.add_constraint(std::move(jlimit));

  // Reference 설정
  ref_.q_des = q;
  ref_.v_des.setZero(robot_info_.nv);
  ref_.a_des.setZero(robot_info_.nv);

  // Solve
  cache_.update(q, v, contacts_);
  const auto& result = formulation.solve(cache_, ref_, contacts_, robot_info_);

  EXPECT_TRUE(result.converged) << "WQP should converge";
  EXPECT_EQ(result.levels_solved, 1);

  // 균형 상태에서 a ≈ 0
  const auto a_opt = result.x_opt.head(robot_info_.nv);
  EXPECT_LT(a_opt.norm(), 10.0)
      << "Optimal acceleration should be small at equilibrium";
}

TEST_F(Phase3WQPTest, SE3WithPositionOffset) {
  YAML::Node form_cfg;
  form_cfg["formulation_type"] = "wqp";
  WQPFormulation formulation;
  formulation.init(*model_, robot_info_, contact_cfg_, form_cfg);

  // PostureTask (low weight)
  auto posture = std::make_unique<PostureTask>();
  YAML::Node posture_cfg;
  posture_cfg["kp"] = 10.0;
  posture_cfg["kd"] = 1.0;
  posture_cfg["weight"] = 0.1;
  posture->init(*model_, robot_info_, cache_, posture_cfg);
  formulation.add_task(std::move(posture));

  // SE3Task (high weight)
  auto se3 = std::make_unique<SE3Task>();
  YAML::Node se3_cfg;
  se3_cfg["frame"] = "panda_hand";
  se3_cfg["kp"] = 100.0;
  se3_cfg["kd"] = 20.0;
  se3_cfg["weight"] = 100.0;
  se3->init(*model_, robot_info_, cache_, se3_cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  // 목표: 현재에서 z축 0.1m offset
  auto target = cache_.registered_frames[0].oMf;
  target.translation()(2) += 0.1;
  auto* se3_ptr = se3.get();
  se3_ptr->set_se3_reference(target);
  formulation.add_task(std::move(se3));

  ref_.q_des = q;
  ref_.v_des.setZero(robot_info_.nv);
  ref_.a_des.setZero(robot_info_.nv);

  const auto& result = formulation.solve(cache_, ref_, contacts_, robot_info_);

  EXPECT_TRUE(result.converged);

  // 위치 오차가 있으므로 a ≠ 0
  const auto a_opt = result.x_opt.head(robot_info_.nv);
  EXPECT_GT(a_opt.norm(), 0.1)
      << "Acceleration should be non-zero to track SE3 target";
}

// ──────────────────────────────────────────────
// HQP: SE3Task(priority 0) > PostureTask(priority 1)
// ──────────────────────────────────────────────
TEST_F(Phase3WQPTest, HQPWithSE3Priority) {
  YAML::Node form_cfg;
  form_cfg["formulation_type"] = "hqp";

  YAML::Node hqp_cfg;
  hqp_cfg["max_levels"] = 3;
  form_cfg["hqp"] = hqp_cfg;

  HQPFormulation formulation;
  formulation.init(*model_, robot_info_, contact_cfg_, form_cfg);

  // PostureTask (priority 1 — lower priority)
  auto posture = std::make_unique<PostureTask>();
  YAML::Node posture_cfg;
  posture_cfg["kp"] = 10.0;
  posture_cfg["kd"] = 1.0;
  posture_cfg["weight"] = 1.0;
  posture_cfg["priority"] = 1;
  posture->init(*model_, robot_info_, cache_, posture_cfg);
  formulation.add_task(std::move(posture));

  // SE3Task (priority 0 — highest)
  auto se3 = std::make_unique<SE3Task>();
  YAML::Node se3_cfg;
  se3_cfg["frame"] = "panda_hand";
  se3_cfg["kp"] = 100.0;
  se3_cfg["kd"] = 20.0;
  se3_cfg["weight"] = 100.0;
  se3_cfg["priority"] = 0;
  se3->init(*model_, robot_info_, cache_, se3_cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  auto* se3_ptr = se3.get();
  se3_ptr->set_se3_reference(cache_.registered_frames[0].oMf);
  formulation.add_task(std::move(se3));

  ref_.q_des = q;
  ref_.v_des.setZero(robot_info_.nv);
  ref_.a_des.setZero(robot_info_.nv);

  const auto& result = formulation.solve(cache_, ref_, contacts_, robot_info_);

  EXPECT_TRUE(result.converged);
  EXPECT_GE(result.levels_solved, 1);
}

// ──────────────────────────────────────────────
// Phase preset 전환: approach → closure 시뮬레이션
// ──────────────────────────────────────────────
TEST_F(Phase3WQPTest, PhasePresetSwitch) {
  YAML::Node form_cfg;
  form_cfg["formulation_type"] = "wqp";
  WQPFormulation formulation;
  formulation.init(*model_, robot_info_, contact_cfg_, form_cfg);

  // PostureTask
  auto posture = std::make_unique<PostureTask>();
  YAML::Node posture_cfg;
  posture_cfg["kp"] = 10.0;
  posture_cfg["kd"] = 1.0;
  posture_cfg["weight"] = 1.0;
  posture->init(*model_, robot_info_, cache_, posture_cfg);
  formulation.add_task(std::move(posture));

  // SE3Task
  auto se3 = std::make_unique<SE3Task>();
  YAML::Node se3_cfg;
  se3_cfg["frame"] = "panda_hand";
  se3_cfg["name"] = "se3_tcp";
  se3_cfg["kp"] = 100.0;
  se3_cfg["kd"] = 20.0;
  se3_cfg["weight"] = 100.0;
  se3->init(*model_, robot_info_, cache_, se3_cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  auto* se3_ptr = se3.get();
  auto target = cache_.registered_frames[0].oMf;
  target.translation()(0) += 0.05;
  se3_ptr->set_se3_reference(target);
  formulation.add_task(std::move(se3));

  // CoMTask (처음에는 비활성)
  auto com = std::make_unique<CoMTask>();
  YAML::Node com_cfg;
  com_cfg["kp"] = 100.0;
  com_cfg["kd"] = 20.0;
  com_cfg["weight"] = 50.0;
  com->init(*model_, robot_info_, cache_, com_cfg);
  auto* com_ptr = com.get();
  com_ptr->set_com_reference(cache_.com_position);
  com_ptr->set_active(false);
  formulation.add_task(std::move(com));

  ref_.q_des = q;
  ref_.v_des.setZero(robot_info_.nv);
  ref_.a_des.setZero(robot_info_.nv);

  // Phase 1: approach — se3 active, com inactive
  cache_.update(q, v, contacts_);
  const auto& result1 = formulation.solve(cache_, ref_, contacts_, robot_info_);
  EXPECT_TRUE(result1.converged);

  // Phase 2: closure — se3 weight 감소, com 활성화
  PhasePreset closure;
  closure.phase_name = "closure";

  TaskPreset tp_posture;
  tp_posture.task_name = "posture";
  tp_posture.active = true;
  tp_posture.weight = 0.1;
  closure.task_presets.push_back(tp_posture);

  TaskPreset tp_se3;
  tp_se3.task_name = "se3_tcp";
  tp_se3.active = true;
  tp_se3.weight = 50.0;
  closure.task_presets.push_back(tp_se3);

  TaskPreset tp_com;
  tp_com.task_name = "com";
  tp_com.active = true;
  tp_com.weight = 100.0;
  closure.task_presets.push_back(tp_com);

  formulation.apply_preset(closure);

  // com이 활성화되었는지 확인
  auto* com_task = formulation.get_task("com");
  ASSERT_NE(com_task, nullptr);
  EXPECT_TRUE(com_task->is_active());
  EXPECT_DOUBLE_EQ(com_task->weight(), 100.0);

  auto* se3_task = formulation.get_task("se3_tcp");
  ASSERT_NE(se3_task, nullptr);
  EXPECT_DOUBLE_EQ(se3_task->weight(), 50.0);

  cache_.update(q, v, contacts_);
  const auto& result2 = formulation.solve(cache_, ref_, contacts_, robot_info_);
  EXPECT_TRUE(result2.converged);
}

// ──────────────────────────────────────────────
// CoM + Momentum 동시 사용 (centroidal + com 플래그 모두 활성)
// ──────────────────────────────────────────────
TEST_F(Phase3WQPTest, CoMAndMomentumTogether) {
  YAML::Node form_cfg;
  form_cfg["formulation_type"] = "wqp";
  WQPFormulation formulation;
  formulation.init(*model_, robot_info_, contact_cfg_, form_cfg);

  // PostureTask
  auto posture = std::make_unique<PostureTask>();
  YAML::Node posture_cfg;
  posture_cfg["kp"] = 10.0;
  posture_cfg["kd"] = 1.0;
  posture_cfg["weight"] = 1.0;
  posture->init(*model_, robot_info_, cache_, posture_cfg);
  formulation.add_task(std::move(posture));

  // CoMTask
  auto com = std::make_unique<CoMTask>();
  YAML::Node com_cfg;
  com_cfg["kp"] = 100.0;
  com_cfg["kd"] = 20.0;
  com_cfg["weight"] = 50.0;
  com->init(*model_, robot_info_, cache_, com_cfg);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.update(q, v, contacts_);

  auto* com_ptr = com.get();
  com_ptr->set_com_reference(cache_.com_position);
  formulation.add_task(std::move(com));

  // MomentumTask (angular regularize)
  auto momentum = std::make_unique<MomentumTask>();
  YAML::Node mom_cfg;
  mom_cfg["mode"] = "angular_regularize";
  mom_cfg["weight"] = 1.0;
  momentum->init(*model_, robot_info_, cache_, mom_cfg);
  formulation.add_task(std::move(momentum));

  EXPECT_TRUE(cache_.compute_com);
  EXPECT_TRUE(cache_.compute_centroidal);

  ref_.q_des = q;
  ref_.v_des.setZero(robot_info_.nv);
  ref_.a_des.setZero(robot_info_.nv);

  cache_.update(q, v, contacts_);
  const auto& result = formulation.solve(cache_, ref_, contacts_, robot_info_);

  EXPECT_TRUE(result.converged);
}

}  // namespace
}  // namespace rtc::tsid
