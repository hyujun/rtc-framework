// ── test_data_flow.cpp ────────────────────────────────────────────────────────
// End-to-end data flow: command → step → state callback → state readback.
// ──────────────────────────────────────────────────────────────────────────────
#include "test_fixture.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace rtc {
namespace {

using namespace std::chrono_literals;

TEST(DataFlow, StateCallbackFiresDuringSimLoop) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());

  std::atomic<int> callback_count{0};
  std::mutex last_mutex;
  std::vector<double> last_positions;
  sim.SetStateCallback(0, [&](const std::vector<double>& pos, const std::vector<double>&,
                              const std::vector<double>&) {
    callback_count.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard lock(last_mutex);
    last_positions = pos;
  });

  sim.Start();
  for (int i = 0; i < 30; ++i) {
    sim.SetCommand(0, {0.0, 0.0});
    std::this_thread::sleep_for(1ms);
  }
  sim.Stop();

  EXPECT_GT(callback_count.load(), 0);
  std::lock_guard lock(last_mutex);
  EXPECT_EQ(last_positions.size(), 2u);
}

TEST(DataFlow, SensorCallbackFiresWhenConfigured) {
  auto cfg = test::MakeMinimalConfig();
  cfg.groups[0].sensor_names = {"auto"};
  cfg.groups[0].sensor_topic = "/arm/sensors";
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());

  std::atomic<int> sensor_count{0};
  sim.SetSensorCallback(
      0, [&](const std::vector<JointGroup::SensorInfo>&, const std::vector<double>&) {
        sensor_count.fetch_add(1, std::memory_order_relaxed);
      });

  sim.Start();
  for (int i = 0; i < 30; ++i) {
    sim.SetCommand(0, {0.0, 0.0});
    std::this_thread::sleep_for(1ms);
  }
  sim.Stop();

  EXPECT_GT(sensor_count.load(), 0);
}

TEST(DataFlow, PositionStateUpdatesAfterCommand) {
  auto cfg = test::MakeMinimalConfig();
  cfg.use_yaml_servo_gains = true;
  cfg.servo_kp = {500.0, 500.0};
  cfg.servo_kd = {50.0, 50.0};
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());

  sim.Start();
  for (int i = 0; i < 200; ++i) {
    sim.SetCommand(0, {0.5, 0.3});
    std::this_thread::sleep_for(1ms);
  }
  const auto pos = sim.GetPositions(0);
  sim.Stop();

  ASSERT_EQ(pos.size(), 2u);
  // Position should have moved toward target (not strictly equal due to dynamics)
  EXPECT_GT(std::abs(pos[0]), 1e-4);
}

TEST(DataFlow, RtfIsPositiveWhilstRunning) {
  auto cfg = test::MakeMinimalConfig();
  cfg.max_rtf = 1.0;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  for (int i = 0; i < 100; ++i) {
    sim.SetCommand(0, {0.0, 0.0});
    std::this_thread::sleep_for(1ms);
  }
  const double rtf = sim.GetRtf();
  sim.Stop();
  EXPECT_GE(rtf, 0.0);  // can be 0 early, positive after warm-up
}

TEST(DataFlow, StepCountMonotonicIncrease) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  sim.Start();
  uint64_t prev = 0;
  for (int i = 0; i < 50; ++i) {
    sim.SetCommand(0, {0.0, 0.0});
    std::this_thread::sleep_for(2ms);
    const auto cur = sim.StepCount();
    EXPECT_GE(cur, prev);
    prev = cur;
  }
  sim.Stop();
}

// ── Publish decimation (issue #566) ─────────────────────────────────────────
// A group with state_publish_divisor N publishes on steps 0, N, 2N, ... while
// the physics and the other groups run every step. Asserted as an exact
// relation between the two groups' callback counts, both counted on the sim
// thread, so no step count read races the loop.

TEST(DataFlow, StatePublishDivisorDecimatesOnlyThatGroup) {
  auto cfg = test::MakeTwoGroupConfig();
  cfg.sync_timeout_ms = 1.0;  // free-run on the timeout; no controller here
  cfg.groups[1].state_publish_divisor = 5;
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());

  std::atomic<int> arm_n{0};
  std::atomic<int> hand_n{0};
  sim.SetStateCallback(0, [&](const std::vector<double>&, const std::vector<double>&,
                              const std::vector<double>&) { arm_n.fetch_add(1); });
  sim.SetStateCallback(1, [&](const std::vector<double>&, const std::vector<double>&,
                              const std::vector<double>&) { hand_n.fetch_add(1); });
  sim.Start();
  std::this_thread::sleep_for(150ms);
  sim.Stop();

  const int arm = arm_n.load();
  ASSERT_GE(arm, 20) << "too few steps to tell 1:5 from 1:1";
  EXPECT_EQ((arm + 4) / 5, hand_n.load()) << "arm published " << arm << " times";
}

TEST(DataFlow, StatePublishDivisorBelowOneFailsInitialize) {
  auto cfg = test::MakeTwoGroupConfig();
  cfg.groups[1].state_publish_divisor = 0;
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

TEST(DataFlow, FakeGroupRejectsAStatePublishDivisor) {
  auto cfg = test::MakeTwoGroupConfig();
  cfg.groups[1].is_robot = false;
  cfg.groups[1].state_publish_divisor = 5;
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

}  // namespace
}  // namespace rtc
