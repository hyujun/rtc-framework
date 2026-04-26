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
  bool Solve(const MPCStateSnapshot & /*state*/, MPCSolution &out_sol,
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
  launch.main.cpu_core = -1;    // no pinning
  launch.main.sched_policy = 0; // SCHED_OTHER fallback via ApplyThreadConfig
  launch.num_workers = 0;
  launch.target_frequency_hz = 100.0; // fast for the test

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
  thread.Join(); // idempotent
  SUCCEED();
}

TEST(MpcThreadLifecycle, StartBeforeInitIsNoOp) {
  NopMPCThread thread;
  thread.Start();
  EXPECT_FALSE(thread.Running());
}

// Pause/Resume: solve counter must freeze while paused and resume after
// Resume(). Tolerates one trailing iteration that may slip past the pause
// gate if Pause() races with the loop's per-iteration check.
TEST(MpcThreadLifecycle, PauseFreezesSolveLoop) {
  MPCSolutionManager mgr;
  mgr.Init(MinimalConfig(), 1, 1, 0);

  MpcThreadLaunchConfig launch{};
  launch.main.cpu_core = -1;
  launch.target_frequency_hz = 200.0; // ~5 ms period

  NopMPCThread thread;
  thread.Init(mgr, launch);
  thread.Start();
  EXPECT_FALSE(thread.Paused());

  // Let the loop spin up.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const int before_pause = thread.solve_count.load();
  EXPECT_GT(before_pause, 0);

  thread.Pause();
  EXPECT_TRUE(thread.Paused());
  // Allow one in-flight iteration to drain into the pause gate.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const int after_pause_settle = thread.solve_count.load();

  // Now stay paused for >>1 period and confirm no further solves.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const int while_paused = thread.solve_count.load();
  EXPECT_EQ(while_paused, after_pause_settle)
      << "solve_count advanced while paused";

  // Resume and verify the loop wakes up.
  thread.Resume();
  EXPECT_FALSE(thread.Paused());
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  const int after_resume = thread.solve_count.load();
  EXPECT_GT(after_resume, while_paused)
      << "solve_count did not advance after Resume";

  thread.RequestStop();
  thread.Join();
}

// Pause/Resume idempotency: repeated Pause / repeated Resume must not
// deadlock or skip the wake-up.
TEST(MpcThreadLifecycle, PauseResumeIdempotent) {
  MPCSolutionManager mgr;
  mgr.Init(MinimalConfig(), 1, 1, 0);

  MpcThreadLaunchConfig launch{};
  launch.main.cpu_core = -1;
  launch.target_frequency_hz = 200.0;

  NopMPCThread thread;
  thread.Init(mgr, launch);
  thread.Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  thread.Pause();
  thread.Pause(); // idempotent
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const int frozen = thread.solve_count.load();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(thread.solve_count.load(), frozen);

  thread.Resume();
  thread.Resume(); // idempotent
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_GT(thread.solve_count.load(), frozen);

  thread.RequestStop();
  thread.Join();
}

// RequestStop must wake a paused thread promptly so Join returns.
TEST(MpcThreadLifecycle, RequestStopWakesPausedThread) {
  MPCSolutionManager mgr;
  mgr.Init(MinimalConfig(), 1, 1, 0);

  MpcThreadLaunchConfig launch{};
  launch.main.cpu_core = -1;
  launch.target_frequency_hz = 50.0;

  NopMPCThread thread;
  thread.Init(mgr, launch);
  thread.Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  thread.Pause();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // If RequestStop fails to notify the cv, this Join will hang past the
  // gtest default timeout. We guard with an explicit duration check.
  const auto t0 = std::chrono::steady_clock::now();
  thread.RequestStop();
  thread.Join();
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0);
  EXPECT_LT(elapsed.count(), 500)
      << "RequestStop did not wake paused loop within 500 ms";
  EXPECT_FALSE(thread.Running());
}

} // namespace
} // namespace rtc::mpc
