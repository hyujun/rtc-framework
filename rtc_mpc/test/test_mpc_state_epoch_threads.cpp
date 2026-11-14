/// @file test_mpc_state_epoch_threads.cpp
/// @brief A state-epoch bump racing a producer and a consumer: once the
///        consumer has seen a bump (acquire), no reference it gets in a
///        later tick comes from a plan of an older state epoch — the plan
///        being sampled is evicted, not kept until a newer one arrives.
///        Three threads: the MPC-side producer publishes a plan per
///        snapshot it reads (carrying that snapshot's epochs), the bumper
///        raises the state epoch, the consumer writes states and samples.

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

TEST(MpcStateEpochThreads, NoTickAfterASeenBumpSamplesAPlanOfAnOlderEpoch) {
  MPCSolutionManager mgr;
  YAML::Node cfg;
  cfg["enabled"] = true;
  cfg["max_stale_solutions"] = 1 << 30;
  cfg["riccati"]["enabled"] = false;
  mgr.Init(cfg, kN, kN, 0);

  std::atomic<bool> stop{false};
  std::atomic<std::uint32_t> bumped{0};

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
  std::thread bumper([&] {
    for (std::uint32_t e = 1; e <= kBumps; ++e) {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
      mgr.SetStateEpoch(e);
      bumped.store(e, std::memory_order_release);
    }
  });

  const Eigen::VectorXd q = Eigen::VectorXd::Zero(kN);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(kN);
  Eigen::VectorXd q_ref(kN), v_ref(kN), a_ff(kN), lambda(1), u_fb(kN);
  InterpMeta meta{};
  std::uint64_t violations = 0;
  std::uint64_t valid_ticks = 0;
  std::uint64_t valid_after_a_bump = 0;
  std::uint64_t tick = 0;
  // The consumer clock: 1 µs a tick — far inside the age cap.
  while (bumped.load(std::memory_order_acquire) < kBumps || tick < 1000) {
    ++tick;
    const std::uint32_t seen = bumped.load(std::memory_order_acquire);
    const std::uint64_t now = tick * 1000;
    mgr.WriteState(q, v, now);
    if (mgr.ComputeReference(q, v, now, q_ref, v_ref, a_ff, lambda, u_fb, meta)) {
      ++valid_ticks;
      if (mgr.ActiveStateEpoch() < seen) {
        ++violations;
      }
      if (seen > 0) {
        ++valid_after_a_bump;
      }
    }
  }
  stop.store(true, std::memory_order_relaxed);
  producer.join();
  bumper.join();

  EXPECT_EQ(violations, 0U) << "of " << valid_ticks << " valid ticks";
  // Pass controls: plans were sampled across the bumps, and some were evicted.
  EXPECT_GT(valid_after_a_bump, 0U);
  EXPECT_GT(mgr.StateEpochRejections(), 0U);
}

}  // namespace
}  // namespace rtc::mpc
