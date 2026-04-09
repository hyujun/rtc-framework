// Unit tests for hand_failure_detector.hpp — failure detection logic.
//
// Tier 2: Uses fake_hand HandController. Requires rclcpp for logging.
// Tests run the detector's 50Hz thread and verify failure detection.

#include <gtest/gtest.h>

#include "ur5e_hand_driver/hand_failure_detector.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>

namespace rtc::test {

using namespace std::chrono_literals;

// ── Fixture: fake-hand controller + failure detector ────────────────────────

class HandFailureDetectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    controller_ = std::make_unique<HandController>(
        "127.0.0.1", 55151, kUdpRecvConfig, 10.0,
        false, 1, 4, true);  // fake_hand=true
    ASSERT_TRUE(controller_->Start());
  }

  void TearDown() override {
    if (controller_) {
      controller_->Stop();
    }
  }

  // Helper: feed N identical commands to the controller
  void FeedCommands(const std::array<float, kNumHandMotors>& cmd, int count) {
    for (int i = 0; i < count; ++i) {
      controller_->SendCommandAndRequestStates(cmd);
    }
  }

  std::unique_ptr<HandController> controller_;
};

// ── Motor all-zero detection ────────────────────────────────────────────────

TEST_F(HandFailureDetectorTest, MotorAllZero_TriggersFailure)
{
  HandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 3;
  cfg.check_motor = true;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;  // disable rate check

  HandFailureDetector detector(*controller_, cfg);

  std::string failure_reason;
  detector.SetFailureCallback([&](const std::string& reason) {
    failure_reason = reason;
  });

  // Feed zero commands
  std::array<float, kNumHandMotors> zero_cmd{};
  FeedCommands(zero_cmd, 10);

  detector.Start();

  // Wait for detector to pick up the all-zero state
  std::this_thread::sleep_for(200ms);

  detector.Stop();

  EXPECT_TRUE(detector.failed());
  EXPECT_NE(failure_reason.find("hand_motor_all_zero"), std::string::npos);
}

TEST_F(HandFailureDetectorTest, MotorNonZero_NoAllZeroFailure)
{
  HandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 3;
  cfg.check_motor = true;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  HandFailureDetector detector(*controller_, cfg);

  std::string failure_reason;
  detector.SetFailureCallback([&](const std::string& reason) {
    failure_reason = reason;
  });

  // Feed varying non-zero commands
  for (int i = 0; i < 10; ++i) {
    std::array<float, kNumHandMotors> cmd{};
    cmd[0] = static_cast<float>(i + 1);
    controller_->SendCommandAndRequestStates(cmd);
  }

  detector.Start();
  std::this_thread::sleep_for(200ms);
  detector.Stop();

  // State stops changing once we stop feeding commands, so duplicate detection
  // may fire. But all-zero should NOT fire since cmd[0] != 0.
  if (detector.failed()) {
    EXPECT_EQ(failure_reason.find("all_zero"), std::string::npos);
  }
}

// ── Motor duplicate detection ───────────────────────────────────────────────

TEST_F(HandFailureDetectorTest, MotorDuplicate_TriggersFailure)
{
  HandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 3;
  cfg.check_motor = true;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  HandFailureDetector detector(*controller_, cfg);

  std::string failure_reason;
  detector.SetFailureCallback([&](const std::string& reason) {
    failure_reason = reason;
  });

  // Feed same non-zero command many times (fake_hand echoes back)
  std::array<float, kNumHandMotors> same_cmd{};
  same_cmd[0] = 5.0f;
  FeedCommands(same_cmd, 20);

  detector.Start();
  std::this_thread::sleep_for(200ms);
  detector.Stop();

  EXPECT_TRUE(detector.failed());
  // Could be "all_zero" or "duplicate" — the non-zero channels prevent all_zero
  // but duplicate should trigger since we send same value
  EXPECT_NE(failure_reason.find("duplicate"), std::string::npos);
}

// ── Sensor all-zero detection ───────────────────────────────────────────────

TEST_F(HandFailureDetectorTest, SensorAllZero_TriggersFailure)
{
  HandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 3;
  cfg.check_motor = false;
  cfg.check_sensor = true;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  HandFailureDetector detector(*controller_, cfg);

  std::string failure_reason;
  detector.SetFailureCallback([&](const std::string& reason) {
    failure_reason = reason;
  });

  // Fake mode echoes sensor_data as all-zeros
  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 1.0f;  // non-zero motor to avoid motor failures
  FeedCommands(cmd, 10);

  detector.Start();
  std::this_thread::sleep_for(200ms);
  detector.Stop();

  EXPECT_TRUE(detector.failed());
  EXPECT_NE(failure_reason.find("hand_sensor_all_zero"), std::string::npos);
}

// ── Config: disable checks ──────────────────────────────────────────────────

TEST_F(HandFailureDetectorTest, DisableMotorCheck_NoFailureOnZero)
{
  HandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 3;
  cfg.check_motor = false;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  HandFailureDetector detector(*controller_, cfg);

  bool callback_called = false;
  detector.SetFailureCallback([&](const std::string& /*reason*/) {
    callback_called = true;
  });

  std::array<float, kNumHandMotors> zero_cmd{};
  FeedCommands(zero_cmd, 10);

  detector.Start();
  std::this_thread::sleep_for(200ms);
  detector.Stop();

  EXPECT_FALSE(detector.failed());
  EXPECT_FALSE(callback_called);
}

TEST_F(HandFailureDetectorTest, CustomThreshold_HighThresholdNoFailure)
{
  HandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 1000;  // very high threshold
  cfg.check_motor = true;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  HandFailureDetector detector(*controller_, cfg);

  bool callback_called = false;
  detector.SetFailureCallback([&](const std::string& /*reason*/) {
    callback_called = true;
  });

  std::array<float, kNumHandMotors> zero_cmd{};
  FeedCommands(zero_cmd, 5);

  detector.Start();
  std::this_thread::sleep_for(150ms);
  detector.Stop();

  // Only 5 zeros fed, threshold is 1000 → no failure
  EXPECT_FALSE(detector.failed());
  EXPECT_FALSE(callback_called);
}

// ── Idempotent failure callback ─────────────────────────────────────────────

TEST_F(HandFailureDetectorTest, FailureCallbackOnceOnly)
{
  HandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 2;
  cfg.check_motor = true;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  HandFailureDetector detector(*controller_, cfg);

  int callback_count = 0;
  detector.SetFailureCallback([&](const std::string& /*reason*/) {
    ++callback_count;
  });

  std::array<float, kNumHandMotors> zero_cmd{};
  FeedCommands(zero_cmd, 20);

  detector.Start();
  std::this_thread::sleep_for(300ms);
  detector.Stop();

  // compare_exchange_strong ensures only one callback
  EXPECT_EQ(callback_count, 1);
}

// ── Lifecycle ───────────────────────────────────────────────────────────────

TEST_F(HandFailureDetectorTest, StartStop_Clean)
{
  HandFailureDetectorConfig cfg{};
  HandFailureDetector detector(*controller_, cfg);

  detector.Start();
  std::this_thread::sleep_for(50ms);
  detector.Stop();
  EXPECT_FALSE(detector.failed());
}

TEST_F(HandFailureDetectorTest, StopWithoutStart_Safe)
{
  HandFailureDetectorConfig cfg{};
  HandFailureDetector detector(*controller_, cfg);
  detector.Stop();  // Should not crash
}

TEST_F(HandFailureDetectorTest, DoubleStart_Ignored)
{
  HandFailureDetectorConfig cfg{};
  cfg.check_motor = false;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;
  HandFailureDetector detector(*controller_, cfg);

  detector.Start();
  detector.Start();  // Second start should be ignored
  std::this_thread::sleep_for(50ms);
  detector.Stop();
}

// ── Link health (fake_hand always has 0 failures) ───────────────────────────

TEST_F(HandFailureDetectorTest, LinkCheck_FakeHandNoFailure)
{
  HandFailureDetectorConfig cfg{};
  cfg.check_motor = false;
  cfg.check_sensor = false;
  cfg.check_link = true;
  cfg.link_fail_threshold = 5;
  cfg.min_rate_hz = 0.0;

  HandFailureDetector detector(*controller_, cfg);

  bool callback_called = false;
  detector.SetFailureCallback([&](const std::string& /*reason*/) {
    callback_called = true;
  });

  // Feed some commands so there are active cycles
  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 1.0f;
  FeedCommands(cmd, 5);

  detector.Start();
  std::this_thread::sleep_for(200ms);
  detector.Stop();

  // Fake hand has no recv failures
  EXPECT_FALSE(detector.failed());
  EXPECT_FALSE(callback_called);
}

}  // namespace rtc::test
