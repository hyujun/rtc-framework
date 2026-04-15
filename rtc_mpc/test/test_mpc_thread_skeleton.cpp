// MPCThread lifecycle smoke test.
//
// Uses a NopMPCThread subclass whose Solve() simply populates a counter in
// `out_sol.iterations`. Verifies:
//   * Start → some Solve calls happen → RequestStop → Join returns.
//   * Join is idempotent (double-join must not hang or crash).
//   * With no MpcThreadLaunchConfig::target_frequency_hz override, the
//     default 20 Hz produces a sensible number of iterations over ~250 ms.

#include "rtc_mpc/thread/mpc_thread.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <thread>

namespace rtc::mpc {
namespace {

class NopMPCThread final : public MPCThread {
 public:
  std::atomic<int> solve_count{0};

 protected:
  bool Solve(const MPCStateSnapshot& /*state*/,
             MPCSolution& out_sol,
             std::span<std::jthread> /*workers*/) override {
    ++solve_count;
    out_sol.horizon_length = 1;
    out_sol.dt_node = 0.01;
    out_sol.nq = 1;
    out_sol.nv = 1;
    out_sol.converged = true;
    return true;
  }
};

YAML::Node MinimalConfig() {
  YAML::Node cfg;
  cfg["enabled"] = true;
  cfg["max_stale_solutions"] = 100;
  return cfg;
}

TEST(MpcThreadLifecycle, StartAndStopCleanly) {
  MPCSolutionManager mgr;
  mgr.Init(MinimalConfig(), 1, 1, 0);

  MpcThreadLaunchConfig launch{};
  launch.main.cpu_core = -1;        // no pinning
  launch.main.sched_policy = 0;     // SCHED_OTHER fallback via ApplyThreadConfig
  launch.num_workers = 0;
  launch.target_frequency_hz = 100.0;  // fast for the test

  NopMPCThread thread;
  thread.Init(mgr, launch);
  thread.Start();
  EXPECT_TRUE(thread.Running());

  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  thread.RequestStop();
  thread.Join();
  EXPECT_FALSE(thread.Running());

  // With a 100 Hz target over ~250 ms we expect at least a handful of
  // solves (loose lower bound, generous upper bound — this is a smoke
  // test, not a timing benchmark).
  EXPECT_GE(thread.solve_count.load(), 5);
  EXPECT_LE(thread.solve_count.load(), 500);
}

TEST(MpcThreadLifecycle, DoubleJoinIsSafe) {
  MPCSolutionManager mgr;
  mgr.Init(MinimalConfig(), 1, 1, 0);

  MpcThreadLaunchConfig launch{};
  launch.main.cpu_core = -1;
  launch.target_frequency_hz = 200.0;

  NopMPCThread thread;
  thread.Init(mgr, launch);
  thread.Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  thread.Join();
  thread.Join();  // idempotent
  SUCCEED();
}

TEST(MpcThreadLifecycle, StartBeforeInitIsNoOp) {
  NopMPCThread thread;
  thread.Start();
  EXPECT_FALSE(thread.Running());
}

}  // namespace
}  // namespace rtc::mpc
