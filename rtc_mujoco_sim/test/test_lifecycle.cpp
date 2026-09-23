// ── test_lifecycle.cpp ────────────────────────────────────────────────────────
// Start/Stop/Pause/Reset/StepOnce — exercises the SimLoop thread.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_base/testing/wait_until.hpp"
#include "test_fixture.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <vector>

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
  const auto steps = sim.StepCount();
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

// ── Lock-step over two robot groups (issue #566) ────────────────────────────
// The step used to wait for the first ("primary") group's command only, so it
// ran on as soon as the arm's command landed and applied the hand's a step
// late. It now waits for a command from every robot group.

using rtc::testing::WaitUntil;

TEST(Lifecycle, StepWaitsForACommandFromEveryRobotGroup) {
  MuJoCoSimulator sim(test::MakeTwoGroupConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.Start();

  // The arm's command alone does not complete the step.
  sim.SetCommand(0, {0.0});
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(0U, sim.StepCount()) << "stepped on the first group's command alone";

  // The hand's completes it: exactly one step.
  sim.SetCommand(1, {0.0});
  ASSERT_TRUE(WaitUntil([&] { return sim.StepCount() >= 1; }, 1000ms));
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(1U, sim.StepCount());

  // Either order, a few ms apart — one step each.
  constexpr std::uint64_t kSteps = 10;
  for (std::uint64_t i = 0; i < kSteps; ++i) {
    const std::size_t first = i % 2;
    sim.SetCommand(first, {0.0});
    std::this_thread::sleep_for(5ms);
    ASSERT_EQ(1U + i, sim.StepCount()) << "step " << i << " ran on one group's command";
    sim.SetCommand(1 - first, {0.0});
    ASSERT_TRUE(WaitUntil([&] { return sim.StepCount() >= 2 + i; }, 1000ms)) << "step " << i;
  }
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(1U + kSteps, sim.StepCount());
  sim.Stop();
}

TEST(Lifecycle, AResetWaitsForTheStepsCommandsSoNoneIsLeftInFlight) {
  MuJoCoSimulator sim(test::MakeTwoGroupConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  sim.SetCommand(0, {0.0});
  sim.SetCommand(1, {0.0});
  ASSERT_TRUE(WaitUntil([&] { return sim.StepCount() >= 1; }, 1000ms));

  // A reset lands while the controller is still computing its reply to the
  // state just published. The reset waits for that reply instead of cutting
  // the wait: a reply arriving after the reset would drive the first
  // post-reset step with a pre-reset command, and the command stream would
  // stay a step behind from then on.
  sim.RequestReset();
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(1U, sim.StepCount()) << "the reset cut the command wait short";
  sim.SetCommand(0, {0.5});
  sim.SetCommand(1, {0.5});
  ASSERT_TRUE(WaitUntil([&] { return sim.StepCount() == 0; }, 1000ms)) << "reset not handled";

  // Those replies were for the pre-reset state and went with it: one fresh
  // group's command does not complete the first post-reset step.
  sim.SetCommand(0, {0.0});
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(0U, sim.StepCount()) << "a pre-reset command drove the first post-reset step";
  sim.SetCommand(1, {0.0});
  EXPECT_TRUE(WaitUntil([&] { return sim.StepCount() >= 1; }, 1000ms));
  sim.Stop();
}

TEST(Lifecycle, ABallResetDuringTheWaitStillStepsOncePerPublishedState) {
  MuJoCoSimulator sim(test::MakeTwoGroupConfig());
  ASSERT_TRUE(sim.Initialize());
  std::atomic<std::uint64_t> published{0};
  sim.SetStateCallback(0, [&](const std::vector<double>&, const std::vector<double>&,
                              const std::vector<double>&) { published.fetch_add(1); });
  sim.Start();
  sim.SetCommand(0, {0.0});
  sim.SetCommand(1, {0.0});
  ASSERT_TRUE(WaitUntil([&] { return sim.StepCount() >= 1; }, 1000ms));

  // It used to `continue` after the wait: the same step's state went out a
  // second time with the commands still pending, so the controller ticked
  // twice on one physics step and the sim stepped at once on the stale pair.
  sim.RequestProjectileBallReset();
  std::this_thread::sleep_for(20ms);
  sim.SetCommand(0, {0.0});
  sim.SetCommand(1, {0.0});
  ASSERT_TRUE(WaitUntil([&] { return sim.StepCount() >= 2; }, 1000ms));
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(2U, sim.StepCount());
  EXPECT_EQ(sim.StepCount() + 1, published.load()) << "a state was published twice for one step";
  sim.Stop();
}

TEST(Lifecycle, AGroupThatOptsOutOfTheCommandWaitDoesNotHoldTheStep) {
  auto cfg = test::MakeTwoGroupConfig();
  cfg.groups[1].wait_for_command = false;  // the controller does not command the hand
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  sim.SetCommand(0, {0.0});
  EXPECT_TRUE(WaitUntil([&] { return sim.StepCount() >= 1; }, 500ms));
  sim.Stop();
}

TEST(Lifecycle, ATimeoutWithSomeCommandsInIsCountedButOneWithNoneIsNot) {
  auto cfg = test::MakeTwoGroupConfig();
  cfg.sync_timeout_ms = 5.0;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  // No controller at all: steps run on the timeout, which is the normal
  // startup case and not a partial one.
  std::this_thread::sleep_for(60ms);
  EXPECT_GT(sim.StepCount(), 1U);
  EXPECT_EQ(0U, sim.PartialCommandSteps());
  // The arm is commanded, the hand never is.
  for (int i = 0; i < 10; ++i) {
    sim.SetCommand(0, {0.0});
    std::this_thread::sleep_for(8ms);
  }
  EXPECT_GT(sim.PartialCommandSteps(), 0U);
  sim.Stop();
}

TEST(Lifecycle, TwoGroupSyncTimeoutStillStepsWithoutCommands) {
  auto cfg = test::MakeTwoGroupConfig();
  cfg.sync_timeout_ms = 5.0;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  sim.SetCommand(0, {0.0});  // one group only, then silence
  std::this_thread::sleep_for(200ms);
  const auto steps = sim.StepCount();
  sim.Stop();
  EXPECT_GT(steps, 1U);
}

}  // namespace
}  // namespace rtc
