/// @file test_mpc_state_epoch_threads.cpp
/// @brief Epoch bumps racing a producer and a consumer: no reference the
///        consumer gets after a bump comes from a plan of an older epoch —
///        the plan being sampled is evicted, not kept until a newer one
///        arrives. Three threads: the MPC-side producer publishes a plan per
///        snapshot it reads (carrying that snapshot's epochs); the consumer
///        writes states, samples and, as the state-epoch contract says,
///        bumps the state epoch itself between two of its writes; a third
///        thread bumps the model epoch (any thread may).

#include "rtc_mpc/manager/mpc_solution_manager.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <thread>

namespace rtc::mpc {
namespace {

constexpr int kN = 3;
constexpr int kHorizon = 10;
constexpr double kDtNode = 0.01;
constexpr std::uint32_t kBumps = 200;

MPCSolution PlanFor(const MPCStateSnapshot& s) {
  MPCSolution sol{};
  sol.horizon_length = kHorizon;
  sol.dt_node = kDtNode;
  MarkSolved(sol, true);
  sol.nq = kN;
  sol.nv = kN;
  sol.nu = kN;
  sol.nx = 2 * kN;
  sol.ndx = 2 * kN;
  sol.observation_stamp_ns = s.timestamp_ns;
  sol.observation_sequence = s.sequence;
  sol.model_epoch = s.model_epoch;
  sol.state_epoch = s.state_epoch;
  return sol;
}

TEST(MpcStateEpochThreads, NoSampleAfterABumpComesFromAPlanOfAnOlderEpoch) {
  MPCSolutionManager mgr;
  YAML::Node cfg;
  cfg["enabled"] = true;
  cfg["max_stale_solutions"] = 1 << 30;
  cfg["riccati"]["enabled"] = false;
  mgr.Init(cfg, kN, kN, 0);

  std::atomic<bool> stop{false};
  std::atomic<std::uint32_t> model_bumped{0};

  std::thread producer([&] {
    std::uint64_t last = 0;
    while (!stop.load(std::memory_order_relaxed)) {
      const MPCStateSnapshot s = mgr.ReadState();
      if (s.valid && s.sequence != last) {
        last = s.sequence;
        mgr.PublishSolution(PlanFor(s));
      }
      std::this_thread::yield();
    }
  });
  std::thread model_bumper([&] {
    for (std::uint32_t e = 1; e <= kBumps; ++e) {
      std::this_thread::sleep_for(std::chrono::microseconds(700));
      mgr.SetModelEpoch(e);
      model_bumped.store(e, std::memory_order_release);
    }
  });

  const Eigen::VectorXd q = Eigen::VectorXd::Zero(kN);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(kN);
  Eigen::VectorXd q_ref(kN), v_ref(kN), a_ff(kN), lambda(1), u_fb(kN);
  InterpMeta meta{};
  std::uint64_t state_violations = 0;
  std::uint64_t model_violations = 0;
  std::uint64_t valid_after_a_state_bump = 0;
  std::uint64_t valid_after_a_model_bump = 0;
  std::uint32_t state_epoch = 0;
  std::uint64_t tick = 0;
  // The consumer clock: 1 µs a tick — far inside the age cap.
  while (model_bumped.load(std::memory_order_acquire) < kBumps || state_epoch < kBumps) {
    ++tick;
    if (tick % 50 == 0 && state_epoch < kBumps) {
      mgr.SetStateEpoch(++state_epoch);  // between two writes, on this thread
    }
    const std::uint32_t model_seen = model_bumped.load(std::memory_order_acquire);
    const std::uint64_t now = tick * 1000;
    mgr.WriteState(q, v, now);
    if (mgr.ComputeReference(q, v, now, q_ref, v_ref, a_ff, lambda, u_fb, meta)) {
      state_violations += mgr.ActiveStateEpoch() != state_epoch ? 1U : 0U;
      model_violations += mgr.ActiveModelEpoch() < model_seen ? 1U : 0U;
      valid_after_a_state_bump += state_epoch > 0 ? 1U : 0U;
      valid_after_a_model_bump += model_seen > 0 ? 1U : 0U;
    }
  }
  stop.store(true, std::memory_order_relaxed);
  producer.join();
  model_bumper.join();

  EXPECT_EQ(state_violations, 0U);
  EXPECT_EQ(model_violations, 0U);
  // Pass controls: plans were sampled across the bumps, and some were evicted.
  EXPECT_GT(valid_after_a_state_bump, 0U);
  EXPECT_GT(valid_after_a_model_bump, 0U);
  EXPECT_GT(mgr.StateEpochRejections(), 0U);
  EXPECT_GT(mgr.EpochRejections(), 0U);
}

}  // namespace
}  // namespace rtc::mpc
