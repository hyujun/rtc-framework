// ── test_sync_timeout_armed.cpp ──────────────────────────────────────────────
// The two-stage command wait (Config::sync_timeout_armed_ms): the short wait
// until a command has arrived, the long one after, one expiry without a
// command disarming it, a reset disarming it; and the count of steps that ran
// on the timeout with no command in. Judged on the minimal scene without a
// controller: the step cadence is the wait that bounds it, so "many steps in
// a window" is the short wait and "one step at most" the long one. Windows
// are generous against a loaded host (a long wait of 400 ms, judged over
// 150 ms).
// ──────────────────────────────────────────────────────────────────────────────

#include "rtc_base/testing/wait_until.hpp"
#include "test_fixture.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

namespace rtc {
namespace {

using namespace std::chrono_literals;

/// The minimal scene with a 1 ms wait before the first command and @p armed_ms
/// after it.
MuJoCoSimulator::Config ArmedConfig(double armed_ms) {
  auto cfg = test::MakeMinimalConfig();
  cfg.sync_timeout_ms = 1.0;
  cfg.sync_timeout_armed_ms = armed_ms;
  return cfg;
}

// With the armed wait off (the default) every step without a command runs on
// the short wait and is counted; the wait never arms.
TEST(SyncTimeoutArmed, OffItIsTheShortWaitThroughoutAndEveryTimeoutStepCounts) {
  MuJoCoSimulator sim(ArmedConfig(0.0));
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return sim.StepCount() >= 20; }, 2s));
  sim.SetCommand(0, {0.0, 0.0});
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return sim.SyncArmed(); }, 2s))
      << "a command arms the wait even when it is the same length";
  const auto steps = sim.StepCount();
  std::this_thread::sleep_for(100ms);
  EXPECT_GE(sim.StepCount(), steps + 20) << "the armed wait is the short one";
  sim.Stop();
  EXPECT_GE(sim.NoCommandTimeoutSteps(), 40U);
  EXPECT_LE(sim.NoCommandTimeoutSteps(), sim.StepCount())
      << "a step with the command in is not a timeout step";
}

// Before the first command the wait is the short one; after it the long one:
// no step for 150 ms, then one timeout step that disarms it and the short
// cadence again.
TEST(SyncTimeoutArmed, TheLongWaitStartsWithTheFirstCommandAndOneExpiryDisarmsIt) {
  MuJoCoSimulator sim(ArmedConfig(400.0));
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return sim.StepCount() >= 20; }, 2s))
      << "the short wait before any command";
  EXPECT_FALSE(sim.SyncArmed());

  sim.SetCommand(0, {0.0, 0.0});
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return sim.SyncArmed(); }, 2s));
  const auto armed_at = sim.StepCount();
  const auto timeouts_at = sim.NoCommandTimeoutSteps();
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(sim.StepCount(), armed_at) << "the long wait holds the step";
  EXPECT_EQ(sim.NoCommandTimeoutSteps(), timeouts_at);

  // The expiry: one timeout step, the wait disarmed, the short cadence back.
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return !sim.SyncArmed(); }, 2s));
  EXPECT_EQ(sim.NoCommandTimeoutSteps(), timeouts_at + 1);
  const auto disarmed_at = sim.StepCount();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return sim.StepCount() >= disarmed_at + 20; }, 2s))
      << "the short wait again";
  sim.Stop();
}

// A command re-arms the wait after an expiry; a reset disarms it.
TEST(SyncTimeoutArmed, ACommandReArmsAndAResetDisarms) {
  MuJoCoSimulator sim(ArmedConfig(400.0));
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  sim.SetCommand(0, {0.0, 0.0});
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return sim.SyncArmed(); }, 2s));
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return !sim.SyncArmed(); }, 2s)) << "the expiry";

  sim.SetCommand(0, {0.0, 0.0});
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return sim.SyncArmed(); }, 2s)) << "re-armed";
  sim.RequestReset();
  // The reset lands on the sim thread within the long wait it cuts short.
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return !sim.SyncArmed(); }, 2s)) << "disarmed";
  const auto after_reset = sim.StepCount();
  EXPECT_TRUE(rtc::testing::WaitUntil([&] { return sim.StepCount() >= after_reset + 20; }, 2s))
      << "the short wait after the reset";
  sim.Stop();
}

}  // namespace
}  // namespace rtc
