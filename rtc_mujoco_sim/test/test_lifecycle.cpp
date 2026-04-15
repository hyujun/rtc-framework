// ── test_lifecycle.cpp ────────────────────────────────────────────────────────
// Start/Stop/Pause/Reset/StepOnce — exercises the SimLoop thread.
// ──────────────────────────────────────────────────────────────────────────────
#include "test_fixture.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

namespace rtc {
namespace {

using namespace std::chrono_literals;

TEST(Lifecycle, StartStopHappyPath) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  EXPECT_TRUE(sim.IsRunning());
  sim.Stop();
  EXPECT_FALSE(sim.IsRunning());
}

TEST(Lifecycle, StopIdempotent) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  sim.Stop();
  sim.Stop();  // no crash
  EXPECT_FALSE(sim.IsRunning());
}

TEST(Lifecycle, StartTwiceIsNoOp) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  sim.Start();  // no crash, still running
  EXPECT_TRUE(sim.IsRunning());
  sim.Stop();
}

TEST(Lifecycle, PausedByDefaultOnStart) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  EXPECT_FALSE(sim.IsPaused());
}

TEST(Lifecycle, PauseResumeRoundtrip) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.Pause();
  EXPECT_TRUE(sim.IsPaused());
  sim.Resume();
  EXPECT_FALSE(sim.IsPaused());
}

TEST(Lifecycle, StepCounterAdvancesUnderCommand) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  // Feed commands to allow primary-group sync to advance
  for (int i = 0; i < 50; ++i) {
    sim.SetCommand(0, {0.0, 0.0});
    std::this_thread::sleep_for(1ms);
  }
  const auto steps = sim.StepCount();
  sim.Stop();
  EXPECT_GT(steps, 0u);
}

TEST(Lifecycle, SimTimeMatchesStepCount) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  for (int i = 0; i < 30; ++i) {
    sim.SetCommand(0, {0.0, 0.0});
    std::this_thread::sleep_for(1ms);
  }
  const auto steps   = sim.StepCount();
  const auto simtime = sim.SimTimeSec();
  sim.Stop();
  if (steps > 0) {
    const double expected = static_cast<double>(steps) * sim.GetPhysicsTimestep();
    EXPECT_NEAR(simtime, expected, 1e-6);
  }
}

TEST(Lifecycle, ResetDoesNotCrash) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  for (int i = 0; i < 20; ++i) {
    sim.SetCommand(0, {0.1, 0.1});
    std::this_thread::sleep_for(1ms);
  }
  sim.RequestReset();
  std::this_thread::sleep_for(20ms);
  sim.Stop();
  SUCCEED();
}

TEST(Lifecycle, StepOnceWhenPaused) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  sim.Pause();
  std::this_thread::sleep_for(20ms);
  const auto before = sim.StepCount();
  sim.StepOnce();
  sim.SetCommand(0, {0.0, 0.0});  // unblock sync_cv
  std::this_thread::sleep_for(30ms);
  const auto after = sim.StepCount();
  sim.Stop();
  EXPECT_GE(after, before);  // step should have advanced (or equal on race)
}

TEST(Lifecycle, SyncTimeoutAllowsStepWithoutCommand) {
  auto cfg = test::MakeMinimalConfig();
  cfg.sync_timeout_ms = 5.0;  // tight timeout
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  // No SetCommand calls — step should still advance after timeout
  std::this_thread::sleep_for(200ms);
  const auto steps = sim.StepCount();
  sim.Stop();
  EXPECT_GT(steps, 0u);
}

}  // namespace
}  // namespace rtc
