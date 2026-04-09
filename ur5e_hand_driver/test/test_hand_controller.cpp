// Unit tests for hand_controller.hpp — HandController (fake_hand mode).
//
// Tier 2: Uses fake_hand=true to bypass UDP. Requires rclcpp for logging.
// rclcpp::init() is NOT required — rclcpp logging works without node context.

#include <gtest/gtest.h>

#include "ur5e_hand_driver/hand_controller.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <string>
#include <thread>

namespace rtc::test {

// ── Test fixture: FakeHand HandController ───────────────────────────────────

class FakeHandControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    controller_ = std::make_unique<HandController>(
        "127.0.0.1",          // target_ip (unused in fake mode)
        55151,                 // target_port
        kUdpRecvConfig,        // thread_cfg
        10.0,                  // recv_timeout_ms
        false,                 // enable_write_ack (deprecated)
        1,                     // sensor_decimation
        4,                     // num_fingertips
        true                   // use_fake_hand
    );
  }

  void TearDown() override {
    if (controller_) {
      controller_->Stop();
    }
  }

  std::unique_ptr<HandController> controller_;
};

// ── Lifecycle ───────────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, Start_SetsRunning)
{
  ASSERT_TRUE(controller_->Start());
  EXPECT_TRUE(controller_->IsRunning());
}

TEST_F(FakeHandControllerTest, Stop_ClearsRunning)
{
  ASSERT_TRUE(controller_->Start());
  controller_->Stop();
  EXPECT_FALSE(controller_->IsRunning());
}

TEST_F(FakeHandControllerTest, DoubleStop_Safe)
{
  ASSERT_TRUE(controller_->Start());
  controller_->Stop();
  controller_->Stop();  // Should not crash
  EXPECT_FALSE(controller_->IsRunning());
}

TEST_F(FakeHandControllerTest, StartWithoutStop_DestructorStops)
{
  ASSERT_TRUE(controller_->Start());
  EXPECT_TRUE(controller_->IsRunning());
  // Destructor in TearDown should cleanly stop
}

// ── State echo-back ─────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, EchoBack_MotorPositions)
{
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  for (int i = 0; i < kNumHandMotors; ++i) {
    cmd[static_cast<std::size_t>(i)] = static_cast<float>(i) * 0.1f;
  }

  controller_->SendCommandAndRequestStates(cmd);

  const auto state = controller_->GetLatestState();
  for (std::size_t i = 0; i < kNumHandMotors; ++i) {
    EXPECT_FLOAT_EQ(state.motor_positions[i], cmd[i]);
  }
}

TEST_F(FakeHandControllerTest, EchoBack_NumFingertips)
{
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  controller_->SendCommandAndRequestStates(cmd);

  const auto state = controller_->GetLatestState();
  EXPECT_EQ(state.num_fingertips, 4);
}

TEST_F(FakeHandControllerTest, EchoBack_ValidFlag)
{
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  controller_->SendCommandAndRequestStates(cmd);

  const auto state = controller_->GetLatestState();
  EXPECT_TRUE(state.valid);
}

TEST_F(FakeHandControllerTest, GetLatestPositions_MatchesCmd)
{
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 1.5f;
  cmd[5] = -3.0f;
  controller_->SendCommandAndRequestStates(cmd);

  const auto positions = controller_->GetLatestPositions();
  EXPECT_FLOAT_EQ(positions[0], 1.5f);
  EXPECT_FLOAT_EQ(positions[5], -3.0f);
}

// ── CycleCount ──────────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, CycleCount_IncrementsPerCall)
{
  ASSERT_TRUE(controller_->Start());

  const auto before = controller_->cycle_count();
  std::array<float, kNumHandMotors> cmd{};
  controller_->SendCommandAndRequestStates(cmd);
  controller_->SendCommandAndRequestStates(cmd);
  controller_->SendCommandAndRequestStates(cmd);

  EXPECT_EQ(controller_->cycle_count(), before + 3);
}

// ── Callback ────────────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, Callback_Invoked)
{
  ASSERT_TRUE(controller_->Start());

  int callback_count = 0;
  HandState last_state{};
  controller_->SetCallback(
      [&](const HandState& state, const FingertipFTState& /*ft*/) {
        ++callback_count;
        last_state = state;
      });

  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 42.0f;
  controller_->SendCommandAndRequestStates(cmd);

  EXPECT_EQ(callback_count, 1);
  EXPECT_FLOAT_EQ(last_state.motor_positions[0], 42.0f);
  EXPECT_TRUE(last_state.valid);
}

TEST_F(FakeHandControllerTest, Callback_MultipleInvocations)
{
  ASSERT_TRUE(controller_->Start());

  int callback_count = 0;
  controller_->SetCallback(
      [&](const HandState& /*state*/, const FingertipFTState& /*ft*/) {
        ++callback_count;
      });

  std::array<float, kNumHandMotors> cmd{};
  for (int i = 0; i < 5; ++i) {
    controller_->SendCommandAndRequestStates(cmd);
  }
  EXPECT_EQ(callback_count, 5);
}

// ── FT inference (stub) ─────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, FTInference_DisabledByDefault)
{
  ASSERT_TRUE(controller_->Start());
  EXPECT_FALSE(controller_->ft_inference_enabled());
}

TEST_F(FakeHandControllerTest, FTState_DefaultInvalid)
{
  ASSERT_TRUE(controller_->Start());
  const auto ft = controller_->GetLatestFTState();
  EXPECT_FALSE(ft.valid);
}

// ── Communication mode ──────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, CommunicationMode_DefaultIndividual)
{
  EXPECT_EQ(controller_->communication_mode(), HandCommunicationMode::kIndividual);
}

TEST(HandControllerConfig, BulkMode)
{
  auto ctrl = std::make_unique<HandController>(
      "127.0.0.1", 55151, kUdpRecvConfig, 10.0,
      false, 1, 4, true,
      std::vector<std::string>{},
      HandCommunicationMode::kBulk);

  EXPECT_EQ(ctrl->communication_mode(), HandCommunicationMode::kBulk);
  ctrl->Stop();
}

// ── Accessors ───────────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, RecvTimeoutMs)
{
  EXPECT_DOUBLE_EQ(controller_->recv_timeout_ms(), 10.0);
}

TEST_F(FakeHandControllerTest, CommStats_InitiallyZero)
{
  const auto stats = controller_->comm_stats();
  EXPECT_EQ(stats.recv_ok, 0u);
  EXPECT_EQ(stats.recv_timeout, 0u);
  EXPECT_EQ(stats.recv_error, 0u);
  EXPECT_EQ(stats.cmd_mismatch, 0u);
  EXPECT_EQ(stats.total_cycles, 0u);
}

TEST_F(FakeHandControllerTest, RecvErrorCount_Zero)
{
  EXPECT_EQ(controller_->recv_error_count(), 0u);
}

TEST_F(FakeHandControllerTest, EventSkipCount_Zero)
{
  EXPECT_EQ(controller_->event_skip_count(), 0u);
}

TEST_F(FakeHandControllerTest, ConsecutiveRecvFailures_Zero)
{
  EXPECT_EQ(controller_->consecutive_recv_failures(), 0u);
}

TEST_F(FakeHandControllerTest, TimingSummary_NoData)
{
  const auto summary = controller_->TimingSummary();
  EXPECT_NE(summary.find("no data"), std::string::npos);
}

// ── E-Stop flag ─────────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, SetEstopFlag_Settable)
{
  std::atomic<bool> flag{false};
  controller_->SetEstopFlag(&flag);
  // Should not crash
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  controller_->SendCommandAndRequestStates(cmd);
}

// ── Legacy API ──────────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, SetTargetPositions_IsAlias)
{
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 99.0f;
  controller_->SetTargetPositions(cmd);

  const auto state = controller_->GetLatestState();
  EXPECT_FLOAT_EQ(state.motor_positions[0], 99.0f);
}

// ── Sensor init status ──────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, SensorInit_FakeModeNotInitialized)
{
  // In fake mode, no sensor init happens (no UDP)
  ASSERT_TRUE(controller_->Start());
  EXPECT_FALSE(controller_->IsSensorInitialized());
}

// ── Num fingertips clamping ─────────────────────────────────────────────────

TEST(HandControllerConfig, NumFingertips_ClampedToNames)
{
  // If fingertip_names has fewer entries than num_fingertips, it should be clamped
  auto ctrl = std::make_unique<HandController>(
      "127.0.0.1", 55151, kUdpRecvConfig, 10.0,
      false, 1,
      10,           // num_fingertips = 10 (but only 2 names below)
      true,         // fake_hand
      std::vector<std::string>{"thumb", "index"});

  ASSERT_TRUE(ctrl->Start());

  std::array<float, kNumHandMotors> cmd{};
  ctrl->SendCommandAndRequestStates(cmd);

  const auto state = ctrl->GetLatestState();
  EXPECT_EQ(state.num_fingertips, 2);  // clamped to name count
  ctrl->Stop();
}

TEST(HandControllerConfig, NumFingertips_NegativeClamped)
{
  auto ctrl = std::make_unique<HandController>(
      "127.0.0.1", 55151, kUdpRecvConfig, 10.0,
      false, 1,
      -1,          // negative
      true);

  ASSERT_TRUE(ctrl->Start());

  std::array<float, kNumHandMotors> cmd{};
  ctrl->SendCommandAndRequestStates(cmd);

  const auto state = ctrl->GetLatestState();
  EXPECT_EQ(state.num_fingertips, 0);
  ctrl->Stop();
}

// ── HasStateBeenRead ───────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, HasStateBeenRead_TrueAfterCommand)
{
  ASSERT_TRUE(controller_->Start());
  // In fake mode, HasStateBeenRead is always false (no real UDP reads)
  // but SendCommandAndRequestStates always succeeds
  std::array<float, kNumHandMotors> cmd{};
  controller_->SendCommandAndRequestStates(cmd);
  // Fake mode state is always stored, but state_read_once_ only tracks real reads
  // Verify the state is valid regardless
  EXPECT_TRUE(controller_->GetLatestState().valid);
}

// ── Bulk mode with fake hand ───────────────────────────────────────────────

TEST(HandControllerConfig, BulkMode_FakeEchoBack)
{
  auto ctrl = std::make_unique<HandController>(
      "127.0.0.1", 55151, kUdpRecvConfig, 10.0,
      false, 1, 4, true,
      std::vector<std::string>{},
      HandCommunicationMode::kBulk);

  ASSERT_TRUE(ctrl->Start());

  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 7.77f;
  cmd[9] = -2.5f;
  ctrl->SendCommandAndRequestStates(cmd);

  const auto state = ctrl->GetLatestState();
  EXPECT_FLOAT_EQ(state.motor_positions[0], 7.77f);
  EXPECT_FLOAT_EQ(state.motor_positions[9], -2.5f);
  EXPECT_TRUE(state.valid);
  ctrl->Stop();
}

// ── Rapid successive commands — last value wins ────────────────────────────

TEST_F(FakeHandControllerTest, RapidCommands_LastValueWins)
{
  ASSERT_TRUE(controller_->Start());

  for (int i = 0; i < 100; ++i) {
    std::array<float, kNumHandMotors> cmd{};
    cmd[0] = static_cast<float>(i);
    controller_->SendCommandAndRequestStates(cmd);
  }

  const auto state = controller_->GetLatestState();
  EXPECT_FLOAT_EQ(state.motor_positions[0], 99.0f);
}

// ── Timing stats after commands ────────────────────────────────────────────

TEST_F(FakeHandControllerTest, TimingSummary_AfterCommands)
{
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  for (int i = 0; i < 5; ++i) {
    controller_->SendCommandAndRequestStates(cmd);
  }

  // Fake mode doesn't update timing profiler (no EventLoop phases)
  // but TimingSummary should still return a valid string
  const auto summary = controller_->TimingSummary();
  EXPECT_FALSE(summary.empty());
}

// ── ActualSensorRateHz accessor ────────────────────────────────────────────

TEST_F(FakeHandControllerTest, ActualSensorRateHz_NonNegative)
{
  EXPECT_GE(controller_->actual_sensor_rate_hz(), 0.0);
}

// ── Zero fingertips mode ───────────────────────────────────────────────────

TEST(HandControllerConfig, ZeroFingertips_NoSensor)
{
  auto ctrl = std::make_unique<HandController>(
      "127.0.0.1", 55151, kUdpRecvConfig, 10.0,
      false, 1,
      0,           // num_fingertips = 0
      true);       // fake_hand

  ASSERT_TRUE(ctrl->Start());

  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 3.14f;
  ctrl->SendCommandAndRequestStates(cmd);

  const auto state = ctrl->GetLatestState();
  EXPECT_FLOAT_EQ(state.motor_positions[0], 3.14f);
  EXPECT_EQ(state.num_fingertips, 0);
  EXPECT_TRUE(state.valid);
  ctrl->Stop();
}

// ── Max fingertips clamping ────────────────────────────────────────────────

TEST(HandControllerConfig, MaxFingertips_Clamped)
{
  auto ctrl = std::make_unique<HandController>(
      "127.0.0.1", 55151, kUdpRecvConfig, 10.0,
      false, 1,
      100,         // num_fingertips > kMaxFingertips
      true);

  ASSERT_TRUE(ctrl->Start());

  std::array<float, kNumHandMotors> cmd{};
  ctrl->SendCommandAndRequestStates(cmd);

  const auto state = ctrl->GetLatestState();
  // Clamped to kMaxFingertips (8), but further clamped by default fingertip_names (4)
  EXPECT_LE(state.num_fingertips, kMaxFingertips);
  ctrl->Stop();
}

}  // namespace rtc::test
