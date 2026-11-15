#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include "panda_fixture.hpp"
#include "rtc_tsid/formulation/formulation_factory.hpp"
#include "rtc_tsid/formulation/hqp_formulation.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"

#include <cmath>

namespace rtc::tsid {
namespace {

class HQPFormulationTest : public test::PandaNoContactTest {};

TEST_F(HQPFormulationTest, FactoryCreatesHQP) {
  YAML::Node config;
  config["formulation_type"] = "hqp";

  auto f = CreateFormulation(*model_, robot_info_, contact_cfg_, config);
  ASSERT_NE(f, nullptr);
  EXPECT_EQ(f->Type(), "hqp");
}

TEST_F(HQPFormulationTest, SingleLevelConverges) {
  YAML::Node config;
  config["formulation_type"] = "hqp";

  auto formulation = CreateFormulation(*model_, robot_info_, contact_cfg_, config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  task_cfg["kp"] = 100.0;
  task_cfg["kd"] = 20.0;
  task_cfg["priority"] = 0;
  posture->Init(*model_, robot_info_, cache_, task_cfg);
  formulation->AddTask(std::move(posture));

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  ref_.q_des = q;
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const auto& result = formulation->Solve(cache_, ref_, contacts_, robot_info_);

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

  auto formulation = CreateFormulation(*model_, robot_info_, contact_cfg_, config);

  auto posture_high = std::make_unique<PostureTask>();
  YAML::Node cfg_high;
  cfg_high["kp"] = 100.0;
  cfg_high["kd"] = 20.0;
  cfg_high["weight"] = 1.0;
  cfg_high["priority"] = 0;
  posture_high->Init(*model_, robot_info_, cache_, cfg_high);
  formulation->AddTask(std::move(posture_high));

  auto posture_low = std::make_unique<PostureTask>();
  YAML::Node cfg_low;
  cfg_low["kp"] = 100.0;
  cfg_low["kd"] = 20.0;
  cfg_low["weight"] = 1.0;
  cfg_low["priority"] = 1;
  posture_low->Init(*model_, robot_info_, cache_, cfg_low);

  Eigen::VectorXd q_neutral = pinocchio::neutral(*model_);
  Eigen::VectorXd q_offset = q_neutral;
  q_offset.head(robot_info_.nv) += Eigen::VectorXd::Constant(robot_info_.nv, 0.5);
  ASSERT_TRUE(posture_low->SetReference(q_offset, Eigen::VectorXd::Zero(robot_info_.nv),
                                        Eigen::VectorXd::Zero(robot_info_.nv)));
  formulation->AddTask(std::move(posture_low));

  Eigen::VectorXd q = q_neutral;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  ref_.q_des = q;
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const auto& result = formulation->Solve(cache_, ref_, contacts_, robot_info_);

  ASSERT_TRUE(result.converged);
  EXPECT_EQ(result.levels_solved, 2);

  // Level 0 dominates → a ≈ 0
  Eigen::VectorXd a_opt = result.x_opt.head(robot_info_.nv);
  EXPECT_LT(a_opt.norm(), 0.5) << "Level 0 should dominate. a_opt = " << a_opt.transpose();
}

TEST_F(HQPFormulationTest, PresetSetsCorrectPriority) {
  YAML::Node config;
  config["formulation_type"] = "hqp";

  auto formulation = CreateFormulation(*model_, robot_info_, contact_cfg_, config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  posture->Init(*model_, robot_info_, cache_, task_cfg);
  formulation->AddTask(std::move(posture));

  PhasePreset preset;
  preset.phase_name = "test";
  TaskPreset tp;
  tp.task_name = "posture";
  tp.active = true;
  tp.weight = 2.0;
  tp.priority = 3;
  preset.task_presets.push_back(tp);

  formulation->ApplyPreset(preset);

  auto* task = formulation->GetTask("posture");
  ASSERT_NE(task, nullptr);
  EXPECT_DOUBLE_EQ(task->Weight(), 2.0);
  EXPECT_EQ(task->Priority(), 3);
}

// The reset-μ warm start through the HQP's per-level solvers: both levels
// (the second under the first one's rows) converge on every tick of a moving
// state, the first solve starts cold and the later ones from the previous
// result, and the answer is the default warm start's to solver tolerance.
TEST_F(HQPFormulationTest, TheResetMuWarmStartSolvesTheCascadeLikeTheDefault) {
  const auto build = [this](const char* warm_start) {
    YAML::Node config;
    config["formulation_type"] = "hqp";
    config["hqp"]["solver_per_level"]["eps_abs"] = 1e-8;
    config["hqp"]["solver_per_level"]["max_iter"] = 200;
    if (warm_start != nullptr) {
      config["hqp"]["solver_per_level"]["warm_start"] = warm_start;
    }
    auto f = CreateFormulation(*model_, robot_info_, contact_cfg_, config);
    for (int level = 0; level < 2; ++level) {
      auto posture = std::make_unique<PostureTask>();
      YAML::Node cfg;
      cfg["kp"] = 100.0;
      cfg["kd"] = 20.0;
      cfg["priority"] = level;
      posture->Init(*model_, robot_info_, cache_, cfg);
      if (level == 1) {  // a second target, held by level 1 under level 0's rows
        Eigen::VectorXd q1 = pinocchio::neutral(*model_);
        q1.array() += 0.3;
        EXPECT_TRUE(posture->SetReference(q1, Eigen::VectorXd::Zero(robot_info_.nv),
                                          Eigen::VectorXd::Zero(robot_info_.nv)));
      }
      f->AddTask(std::move(posture));
    }
    return f;
  };
  auto reset_mu = build("previous_result_reset_mu");
  auto shipped = build(nullptr);
  ASSERT_EQ(dynamic_cast<HQPFormulation&>(*reset_mu).SolverConfig().warm_start,
            WarmStartMode::kPreviousResultResetMu);
  ASSERT_EQ(dynamic_cast<HQPFormulation&>(*shipped).SolverConfig().warm_start,
            WarmStartMode::kPreviousResult);

  const Eigen::VectorXd q0 = pinocchio::neutral(*model_);
  for (int k = 0; k < 20; ++k) {
    Eigen::VectorXd q = q0;
    for (int j = 0; j < robot_info_.nv; ++j) {
      q(j) += 0.2 * std::sin(0.3 * k + 0.5 * j);
    }
    const Eigen::VectorXd v = Eigen::VectorXd::Constant(robot_info_.nv, 0.1 * std::cos(0.2 * k));
    cache_.Update(q, v);
    ref_.q_des = q0;
    ref_.v_des.setZero(robot_info_.nv);
    ref_.a_des.setZero(robot_info_.nv);
    const SolveResult& r = reset_mu->Solve(cache_, ref_, contacts_, robot_info_);
    ASSERT_TRUE(r.converged) << "tick " << k;
    EXPECT_EQ(r.levels_solved, 2) << "tick " << k;
    EXPECT_EQ(r.warm_started, k > 0) << "tick " << k;
    EXPECT_LE(r.pri_res, 1e-8);
    const Eigen::VectorXd a = r.x_opt.head(robot_info_.nv);
    const SolveResult& d = shipped->Solve(cache_, ref_, contacts_, robot_info_);
    ASSERT_TRUE(d.converged) << "tick " << k;
    EXPECT_LE((a - d.x_opt.head(robot_info_.nv)).lpNorm<Eigen::Infinity>(), 1e-5) << "tick " << k;
  }
}

}  // namespace
}  // namespace rtc::tsid
