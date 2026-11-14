/// @file test_policy_sampler_cost.cpp
/// @brief Cost of one `SampleReference` tick on the public G1 (free-flyer +
///        joints, two 6-D contacts, four gain nodes, the floating-base
///        Riccati `Δx` through `WbLayout::StateDifference`), split in two:
///        a normal tick (the held plan) and an install tick (a fresh plan
///        taken: the payload copy, the gains' finiteness check, then the
///        sample, after the cache was evicted). The same run reports the
///        fixed-base kernel at the joint dimensions.
///
/// Armed (`RTC_RUN_TIMING_GATES=1`, Release, a fixed core): 5 runs, the
/// median of the per-run p99. Pre-registered gates: normal tick ≤ 20 µs (1 %
/// of a 2 ms tick); install tick ≤ 18.2 + 20 µs (the cold payload copy's
/// fixed control plus the normal-tick budget). Unarmed it runs a few ticks
/// and prints.

#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/model/wb_layout.hpp"
#include "test_utils/g1_fixture.hpp"

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <random>
#include <vector>

namespace rtc::mpc {
namespace {

constexpr double kNormalP99LimitUs = 20.0;
constexpr double kInstallP99LimitUs = 18.2 + 20.0;
constexpr int kH = 32;
constexpr double kDt = 0.035;
constexpr int64_t kDtNs = 35'000'000;
constexpr uint64_t kObs = 10'000'000'000ULL;
constexpr std::size_t kEvictBytes = 32u * 1024u * 1024u;

bool Armed() {
  const char* v = std::getenv("RTC_RUN_TIMING_GATES");
  return v != nullptr && std::strcmp(v, "1") == 0;
}

double P99(std::vector<double>& s) {
  std::sort(s.begin(), s.end());
  const auto idx = static_cast<std::size_t>(0.99 * static_cast<double>(s.size() - 1) + 0.5);
  return s[std::min(idx, s.size() - 1)];
}

double Median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

/// A plan of @p layout with finite random trajectories and gains.
std::unique_ptr<MPCSolution> RandomPlan(const WbLayout& l, const Eigen::VectorXd& q0) {
  auto s = std::make_unique<MPCSolution>();
  s->horizon_length = kH;
  s->dt_node = kDt;
  MarkSolved(*s, true);
  s->nq = l.nq;
  s->nv = l.nv;
  s->nu = l.nu;
  s->nx = l.nq + l.nv;
  s->ndx = l.ndx;
  s->n_contact_vars = l.n_contact_vars;
  s->observation_stamp_ns = kObs;
  s->gain_nodes = kMaxGainNodes;
  std::mt19937_64 rng(20261114);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int k = 0; k <= kH; ++k) {
    const auto kk = static_cast<std::size_t>(k);
    for (int i = 0; i < l.nq; ++i) {
      s->q_traj[kk][static_cast<std::size_t>(i)] = q0[i] + (i >= 7 || i < 3 ? 0.01 * u(rng) : 0.0);
    }
    for (int i = 0; i < l.nv; ++i) {
      s->v_traj[kk][static_cast<std::size_t>(i)] = 0.1 * u(rng);
    }
    s->node_mode[kk] = 0b11u;
  }
  for (int k = 0; k < kH; ++k) {
    for (int i = 0; i < l.n_contact_vars; ++i) {
      s->lambda_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] = 100.0 * u(rng);
    }
    for (int i = 0; i < l.nv; ++i) {
      s->a_ff_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] = u(rng);
    }
  }
  for (int g = 0; g < kMaxGainNodes; ++g) {
    for (std::size_t i = 0; i < static_cast<std::size_t>(l.nu * l.ndx); ++i) {
      s->K_riccati[static_cast<std::size_t>(g)][i] = u(rng);
    }
  }
  return s;
}

struct Times {
  double normal_us{0.0};
  double install_us{0.0};
};

/// Median over @p runs of the per-run p99 of a normal and of an install tick.
Times Measure(MPCSolutionManager& mgr, const MPCSolution& plan, const Eigen::VectorXd& q,
              const Eigen::VectorXd& v, int runs, int ticks, int installs, bool& all_valid) {
  using Clock = std::chrono::steady_clock;
  std::vector<std::byte> evict(kEvictBytes);
  volatile std::byte sink{};
  WbcReference ref{};
  std::vector<double> normal_p99;
  std::vector<double> install_p99;
  all_valid = true;
  for (int r = 0; r < runs; ++r) {
    std::vector<double> normal;
    std::vector<double> install;
    mgr.PublishSolution(plan);
    all_valid = mgr.SampleReference(q, v, kObs, ref) && all_valid;
    for (int t = 0; t < ticks; ++t) {
      const uint64_t now = kObs + static_cast<uint64_t>(t) * 100'000ULL;
      const auto t0 = Clock::now();
      const bool ok = mgr.SampleReference(q, v, now, ref);
      const auto t1 = Clock::now();
      all_valid = ok && all_valid;
      normal.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    for (int t = 0; t < installs; ++t) {
      mgr.PublishSolution(plan);
      for (std::size_t i = 0; i < evict.size(); i += 64) {
        evict[i] = static_cast<std::byte>(i);
      }
      sink = evict[static_cast<std::size_t>(t) * 64 % evict.size()];
      const uint64_t now = kObs + static_cast<uint64_t>(t) * 100'000ULL;
      const auto t0 = Clock::now();
      const bool ok = mgr.SampleReference(q, v, now, ref);
      const auto t1 = Clock::now();
      all_valid = ok && all_valid;
      install.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    normal_p99.push_back(P99(normal));
    install_p99.push_back(P99(install));
  }
  (void)sink;
  return {Median(normal_p99), Median(install_p99)};
}

TEST(PolicySamplerCost, NormalAndInstallTicksOnTheG1) {
  if (!std::filesystem::exists(test_utils::kG1Urdf)) {
    GTEST_SKIP() << "G1 URDF not found at " << test_utils::kG1Urdf;
  }
  const bool armed = Armed();
  const int runs = armed ? 5 : 1;
  const int ticks = armed ? 1000 : 5;
  const int installs = armed ? 200 : 3;

  const pinocchio::Model model = test_utils::BuildG1Floating();
  RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model, YAML::Load(test_utils::kG1SolesYaml)),
            RobotModelInitError::kNoError);
  const WbLayout layout = WbLayout::FromHandler(handler);
  Eigen::VectorXd q0 = pinocchio::neutral(model);
  q0[2] = 0.75;
  const auto plan = RandomPlan(layout, q0);
  Eigen::VectorXd dq = Eigen::VectorXd::Constant(model.nv, 0.01);
  Eigen::VectorXd q(model.nq);
  pinocchio::integrate(model, q0, dq, q);
  const Eigen::VectorXd v = Eigen::VectorXd::Constant(model.nv, 0.05);

  YAML::Node cfg;
  cfg["enabled"] = true;
  MPCSolutionManager mgr;
  mgr.Init(cfg, layout, &model);
  ASSERT_TRUE(mgr.ConfigValid());
  bool valid = false;
  const Times floating = Measure(mgr, *plan, q, v, runs, ticks, installs, valid);
  EXPECT_TRUE(valid);

  // The fixed-base kernel at the joint dimensions, for the record.
  const int nj = layout.nv - layout.n_base_v;
  const WbLayout fixed_layout = WbLayout::FixedBase(nj, nj, 0);
  auto fixed_plan = std::make_unique<MPCSolution>(*plan);
  fixed_plan->nq = nj;
  fixed_plan->nv = nj;
  fixed_plan->nu = nj;
  fixed_plan->nx = 2 * nj;
  fixed_plan->ndx = 2 * nj;
  fixed_plan->n_contact_vars = 0;
  MPCSolutionManager fixed;
  fixed.Init(cfg, fixed_layout);
  bool fixed_valid = false;
  const Times hermite = Measure(fixed, *fixed_plan, Eigen::VectorXd::Zero(nj),
                                Eigen::VectorXd::Zero(nj), runs, ticks, installs, fixed_valid);
  EXPECT_TRUE(fixed_valid);

  std::printf(
      "[sampler-cost] G1 nq %d nv %d nu %d: normal tick p99 %.2f us (gate %.1f), install tick "
      "p99 %.2f us (gate %.1f); fixed-base %d joints: normal %.2f us, install %.2f us; runs %d, "
      "%s\n",
      layout.nq, layout.nv, layout.nu, floating.normal_us, kNormalP99LimitUs, floating.install_us,
      kInstallP99LimitUs, nj, hermite.normal_us, hermite.install_us, runs,
      armed ? "ARMED" : "not armed (RTC_RUN_TIMING_GATES=1 arms it)");
  RecordProperty("normal_p99_us", std::to_string(floating.normal_us));
  RecordProperty("install_p99_us", std::to_string(floating.install_us));
  RecordProperty("fixed_normal_p99_us", std::to_string(hermite.normal_us));
  RecordProperty("fixed_install_p99_us", std::to_string(hermite.install_us));
  if (armed) {
    EXPECT_LE(floating.normal_us, kNormalP99LimitUs);
    EXPECT_LE(floating.install_us, kInstallP99LimitUs);
  }
}

}  // namespace
}  // namespace rtc::mpc
