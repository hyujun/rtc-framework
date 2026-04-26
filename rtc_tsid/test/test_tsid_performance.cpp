#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <numeric>
#include <vector>

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

struct TimingStats {
  double mean_us;
  double median_us;
  double max_us;
  double min_us;
};

TimingStats compute_stats(std::vector<double>& times) {
  std::sort(times.begin(), times.end());
  const double sum = std::accumulate(times.begin(), times.end(), 0.0);
  return {
      sum / static_cast<double>(times.size()),
      times[times.size() / 2],
      times.back(),
      times.front(),
  };
}

class TSIDPerformanceTest : public ::testing::Test {
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

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  ControlReference ref_;
  Eigen::VectorXd q_, v_;
};

TEST_F(TSIDPerformanceTest, PandaWQPSolveTime) {
  YAML::Node config;
  config["formulation_type"] = "wqp";

  auto formulation = create_formulation(
      *model_, robot_info_, contact_cfg_, config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  task_cfg["kp"] = 100.0;
  task_cfg["kd"] = 20.0;
  posture->init(*model_, robot_info_, cache_, task_cfg);
  formulation->add_task(std::move(posture));

  cache_.update(q_, v_, contacts_);
  ref_.q_des = q_;
  ref_.v_des = v_;
  ref_.a_des.setZero(robot_info_.nv);

  const int n_iters = 100;
  std::vector<double> times(static_cast<size_t>(n_iters));

  // Warm-up
  for (int i = 0; i < 5; ++i) {
    formulation->solve(cache_, ref_, contacts_, robot_info_);
  }

  for (int i = 0; i < n_iters; ++i) {
    // Slightly vary q to prevent caching artifacts
    q_(0) += 1e-6;
    cache_.update(q_, v_, contacts_);

    const auto t0 = std::chrono::steady_clock::now();
    const auto& result =
        formulation->solve(cache_, ref_, contacts_, robot_info_);
    const auto t1 = std::chrono::steady_clock::now();

    ASSERT_TRUE(result.converged) << "Iteration " << i;
    times[static_cast<size_t>(i)] =
        std::chrono::duration<double, std::micro>(t1 - t0).count();
  }

  auto stats = compute_stats(times);

  // Report
  std::cout << "[WQP Panda 7DoF] solve time (us): "
            << "mean=" << stats.mean_us << " median=" << stats.median_us
            << " min=" << stats.min_us << " max=" << stats.max_us
            << std::endl;

  // Assert: median < 500us (conservative for 500Hz RT)
  EXPECT_LT(stats.median_us, 500.0)
      << "WQP solve median too slow for 500Hz RT loop";
}

TEST_F(TSIDPerformanceTest, PandaHQP2LevelsSolveTime) {
  YAML::Node config;
  config["formulation_type"] = "hqp";

  auto formulation = create_formulation(
      *model_, robot_info_, contact_cfg_, config);

  // Level 0
  auto posture0 = std::make_unique<PostureTask>();
  YAML::Node cfg0;
  cfg0["kp"] = 100.0;
  cfg0["kd"] = 20.0;
  cfg0["priority"] = 0;
  posture0->init(*model_, robot_info_, cache_, cfg0);
  formulation->add_task(std::move(posture0));

  // Level 1
  auto posture1 = std::make_unique<PostureTask>();
  YAML::Node cfg1;
  cfg1["kp"] = 50.0;
  cfg1["kd"] = 10.0;
  cfg1["priority"] = 1;
  posture1->init(*model_, robot_info_, cache_, cfg1);
  formulation->add_task(std::move(posture1));

  cache_.update(q_, v_, contacts_);
  ref_.q_des = q_;
  ref_.v_des = v_;
  ref_.a_des.setZero(robot_info_.nv);

  const int n_iters = 100;
  std::vector<double> times(static_cast<size_t>(n_iters));

  // Warm-up
  for (int i = 0; i < 5; ++i) {
    formulation->solve(cache_, ref_, contacts_, robot_info_);
  }

  for (int i = 0; i < n_iters; ++i) {
    q_(0) += 1e-6;
    cache_.update(q_, v_, contacts_);

    const auto t0 = std::chrono::steady_clock::now();
    const auto& result =
        formulation->solve(cache_, ref_, contacts_, robot_info_);
    const auto t1 = std::chrono::steady_clock::now();

    ASSERT_TRUE(result.converged) << "Iteration " << i;
    EXPECT_EQ(result.levels_solved, 2);
    times[static_cast<size_t>(i)] =
        std::chrono::duration<double, std::micro>(t1 - t0).count();
  }

  auto stats = compute_stats(times);

  std::cout << "[HQP Panda 7DoF 2-levels] solve time (us): "
            << "mean=" << stats.mean_us << " median=" << stats.median_us
            << " min=" << stats.min_us << " max=" << stats.max_us
            << std::endl;

  // HQP 2 levels ≈ 2× WQP. Assert median < 1000us
  EXPECT_LT(stats.median_us, 1000.0)
      << "HQP 2-level solve median too slow";
}

TEST_F(TSIDPerformanceTest, FullPipelineWithCacheUpdate) {
  // Measure full pipeline: PinocchioCache::update() + WQP solve + τ recovery
  TSIDController ctrl;
  YAML::Node config;
  config["formulation_type"] = "wqp";
  ctrl.init(*model_, robot_info_, config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  task_cfg["kp"] = 100.0;
  task_cfg["kd"] = 20.0;
  posture->init(*model_, robot_info_, cache_, task_cfg);
  ctrl.formulation().add_task(std::move(posture));

  ref_.q_des = q_;
  ref_.v_des = v_;
  ref_.a_des.setZero(robot_info_.nv);

  ControlState state;
  state.q = q_;
  state.v = v_;

  const int n_iters = 100;
  std::vector<double> times(static_cast<size_t>(n_iters));

  // Warm-up
  for (int i = 0; i < 5; ++i) {
    cache_.update(q_, v_, contacts_);
    ctrl.compute(state, ref_, cache_, contacts_);
  }

  for (int i = 0; i < n_iters; ++i) {
    q_(0) += 1e-6;
    state.q = q_;

    const auto t0 = std::chrono::steady_clock::now();
    cache_.update(q_, v_, contacts_);
    auto output = ctrl.compute(state, ref_, cache_, contacts_);
    const auto t1 = std::chrono::steady_clock::now();

    ASSERT_TRUE(output.qp_converged) << "Iteration " << i;
    times[static_cast<size_t>(i)] =
        std::chrono::duration<double, std::micro>(t1 - t0).count();
  }

  auto stats = compute_stats(times);

  std::cout << "[Full Pipeline Panda WQP] time (us): "
            << "mean=" << stats.mean_us << " median=" << stats.median_us
            << " min=" << stats.min_us << " max=" << stats.max_us
            << std::endl;

  // Full pipeline (cache + solve + tau) < 2000us (500Hz budget)
  EXPECT_LT(stats.median_us, 2000.0)
      << "Full pipeline too slow for 500Hz";
}

}  // namespace
}  // namespace rtc::tsid
