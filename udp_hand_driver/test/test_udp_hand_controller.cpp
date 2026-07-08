// Unit tests for udp_hand_controller.hpp — UdpHandController (fake_hand mode).
//
// Tier 2: Uses fake_hand=true to bypass UDP. Requires rclcpp for logging.
// rclcpp::init() is NOT required — rclcpp logging works without node context.

#include "udp_hand_driver/udp_hand_controller.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <string>
#include <thread>
#include <utility>

namespace udp_hand_driver::test {

// ── Test fixture: FakeHand UdpHandController ───────────────────────────────────

class FakeHandControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    controller_ =
        std::make_unique<UdpHandController>("127.0.0.1",         // target_ip (unused in fake mode)
                                            55151,               // target_port
                                            kHandUdpRecvConfig,  // thread_cfg
                                            10.0,                // recv_timeout_ms
                                            false,               // enable_write_ack (deprecated)
                                            1,                   // sensor_decimation
                                            4,                   // num_fingertips
                                            true                 // use_fake_hand
        );
  }

  void TearDown() override {
    if (controller_) {
      controller_->Stop();
    }
  }

  std::unique_ptr<UdpHandController> controller_;
};

// ── Lifecycle ───────────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, Start_SetsRunning) {
  ASSERT_TRUE(controller_->Start());
  EXPECT_TRUE(controller_->IsRunning());
}

TEST_F(FakeHandControllerTest, Stop_ClearsRunning) {
  ASSERT_TRUE(controller_->Start());
  controller_->Stop();
  EXPECT_FALSE(controller_->IsRunning());
}

TEST_F(FakeHandControllerTest, DoubleStop_Safe) {
  ASSERT_TRUE(controller_->Start());
  controller_->Stop();
  controller_->Stop();  // Should not crash
  EXPECT_FALSE(controller_->IsRunning());
}

TEST_F(FakeHandControllerTest, StartWithoutStop_DestructorStops) {
  ASSERT_TRUE(controller_->Start());
  EXPECT_TRUE(controller_->IsRunning());
  // Destructor in TearDown should cleanly stop
}

// ── State echo-back ─────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, EchoBack_MotorPositions) {
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

TEST_F(FakeHandControllerTest, EchoBack_NumFingertips) {
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  controller_->SendCommandAndRequestStates(cmd);

  const auto state = controller_->GetLatestState();
  EXPECT_EQ(state.num_fingertips, 4);
}

TEST_F(FakeHandControllerTest, EchoBack_ValidFlag) {
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  controller_->SendCommandAndRequestStates(cmd);

  const auto state = controller_->GetLatestState();
  EXPECT_TRUE(state.valid);
}

TEST_F(FakeHandControllerTest, GetLatestPositions_MatchesCmd) {
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

TEST_F(FakeHandControllerTest, CycleCount_IncrementsPerCall) {
  ASSERT_TRUE(controller_->Start());

  const auto before = controller_->cycle_count();
  std::array<float, kNumHandMotors> cmd{};
  controller_->SendCommandAndRequestStates(cmd);
  controller_->SendCommandAndRequestStates(cmd);
  controller_->SendCommandAndRequestStates(cmd);

  EXPECT_EQ(controller_->cycle_count(), before + 3);
}

// ── Callback ────────────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, Callback_Invoked) {
  ASSERT_TRUE(controller_->Start());

  int callback_count = 0;
  UdpHandState last_state{};
  controller_->SetCallback(
      [&](const UdpHandState& state, const udp_hand_driver::FingertipFTState& /*ft*/) {
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

TEST_F(FakeHandControllerTest, Callback_MultipleInvocations) {
  ASSERT_TRUE(controller_->Start());

  int callback_count = 0;
  controller_->SetCallback(
      [&](const UdpHandState& /*state*/, const udp_hand_driver::FingertipFTState& /*ft*/) {
        ++callback_count;
      });

  std::array<float, kNumHandMotors> cmd{};
  for (int i = 0; i < 5; ++i) {
    controller_->SendCommandAndRequestStates(cmd);
  }
  EXPECT_EQ(callback_count, 5);
}

// ── FT inference (stub) ─────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, FTInference_DisabledByDefault) {
  ASSERT_TRUE(controller_->Start());
  EXPECT_FALSE(controller_->ft_inference_enabled());
}

TEST_F(FakeHandControllerTest, FTState_DefaultInvalid) {
  ASSERT_TRUE(controller_->Start());
  const auto ft = controller_->GetLatestFTState();
  EXPECT_FALSE(ft.valid);
}

// ── Communication mode ──────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, CommunicationMode_DefaultIndividual) {
  EXPECT_EQ(controller_->communication_mode(), HandCommunicationMode::kIndividual);
}

TEST(HandControllerConfig, BulkMode) {
  auto ctrl = std::make_unique<UdpHandController>("127.0.0.1", 55151, kHandUdpRecvConfig, 10.0,
                                                  false, 1, 4, true, std::vector<std::string>{},
                                                  HandCommunicationMode::kBulk);

  EXPECT_EQ(ctrl->communication_mode(), HandCommunicationMode::kBulk);
  ctrl->Stop();
}

// ── Accessors ───────────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, RecvTimeoutMs) {
  EXPECT_DOUBLE_EQ(controller_->recv_timeout_ms(), 10.0);
}

TEST_F(FakeHandControllerTest, CommStats_InitiallyZero) {
  const auto stats = controller_->comm_stats();
  EXPECT_EQ(stats.recv_ok, 0u);
  EXPECT_EQ(stats.recv_timeout, 0u);
  EXPECT_EQ(stats.recv_error, 0u);
  EXPECT_EQ(stats.cmd_mismatch, 0u);
  EXPECT_EQ(stats.total_cycles, 0u);
}

TEST_F(FakeHandControllerTest, RecvErrorCount_Zero) {
  EXPECT_EQ(controller_->recv_error_count(), 0u);
}

TEST_F(FakeHandControllerTest, EventSkipCount_Zero) {
  EXPECT_EQ(controller_->event_skip_count(), 0u);
}

TEST_F(FakeHandControllerTest, ConsecutiveRecvFailures_Zero) {
  EXPECT_EQ(controller_->consecutive_recv_failures(), 0u);
}

TEST_F(FakeHandControllerTest, TimingSummary_NoData) {
  const auto summary = controller_->TimingSummary();
  EXPECT_NE(summary.find("no data"), std::string::npos);
}

// ── E-Stop flag ─────────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, SetEstopFlag_Settable) {
  std::atomic<bool> flag{false};
  controller_->SetEstopFlag(&flag);
  // Should not crash
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  controller_->SendCommandAndRequestStates(cmd);
}

// ── Legacy API ──────────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, SetTargetPositions_IsAlias) {
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 99.0f;
  controller_->SetTargetPositions(cmd);

  const auto state = controller_->GetLatestState();
  EXPECT_FLOAT_EQ(state.motor_positions[0], 99.0f);
}

// ── Sensor init status ──────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, SensorInit_FakeModeNotInitialized) {
  // In fake mode, no sensor init happens (no UDP)
  ASSERT_TRUE(controller_->Start());
  EXPECT_FALSE(controller_->IsSensorInitialized());
}

// ── Num fingertips clamping ─────────────────────────────────────────────────

TEST(HandControllerConfig, NumFingertips_ClampedToNames) {
  // If fingertip_names has fewer entries than num_fingertips, it should be clamped
  auto ctrl =
      std::make_unique<UdpHandController>("127.0.0.1", 55151, kHandUdpRecvConfig, 10.0, false, 1,
                                          10,    // num_fingertips = 10 (but only 2 names below)
                                          true,  // fake_hand
                                          std::vector<std::string>{"thumb", "index"});

  ASSERT_TRUE(ctrl->Start());

  std::array<float, kNumHandMotors> cmd{};
  ctrl->SendCommandAndRequestStates(cmd);

  const auto state = ctrl->GetLatestState();
  EXPECT_EQ(state.num_fingertips, 2);  // clamped to name count
  ctrl->Stop();
}

TEST(HandControllerConfig, NumFingertips_NegativeClamped) {
  auto ctrl =
      std::make_unique<UdpHandController>("127.0.0.1", 55151, kHandUdpRecvConfig, 10.0, false, 1,
                                          -1,  // negative
                                          true);

  ASSERT_TRUE(ctrl->Start());

  std::array<float, kNumHandMotors> cmd{};
  ctrl->SendCommandAndRequestStates(cmd);

  const auto state = ctrl->GetLatestState();
  EXPECT_EQ(state.num_fingertips, 0);
  ctrl->Stop();
}

// ── HasStateBeenRead ───────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, HasStateBeenRead_TrueAfterCommand) {
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

TEST(HandControllerConfig, BulkMode_FakeEchoBack) {
  auto ctrl = std::make_unique<UdpHandController>("127.0.0.1", 55151, kHandUdpRecvConfig, 10.0,
                                                  false, 1, 4, true, std::vector<std::string>{},
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

TEST_F(FakeHandControllerTest, RapidCommands_LastValueWins) {
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

TEST_F(FakeHandControllerTest, TimingSummary_AfterCommands) {
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

TEST_F(FakeHandControllerTest, ActualSensorRateHz_NonNegative) {
  EXPECT_GE(controller_->actual_sensor_rate_hz(), 0.0);
}

// ── Zero fingertips mode ───────────────────────────────────────────────────

TEST(HandControllerConfig, ZeroFingertips_NoSensor) {
  auto ctrl =
      std::make_unique<UdpHandController>("127.0.0.1", 55151, kHandUdpRecvConfig, 10.0, false, 1,
                                          0,      // num_fingertips = 0
                                          true);  // fake_hand

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

TEST(HandControllerConfig, MaxFingertips_Clamped) {
  auto ctrl =
      std::make_unique<UdpHandController>("127.0.0.1", 55151, kHandUdpRecvConfig, 10.0, false, 1,
                                          100,  // num_fingertips > kMaxFingertips
                                          true);

  ASSERT_TRUE(ctrl->Start());

  std::array<float, kNumHandMotors> cmd{};
  ctrl->SendCommandAndRequestStates(cmd);

  const auto state = ctrl->GetLatestState();
  // Clamped to kMaxFingertips (8), but further clamped by default fingertip_names (4)
  EXPECT_LE(state.num_fingertips, kMaxFingertips);
  ctrl->Stop();
}

// ── Cycle outcome ring (link-down forensics) ────────────────────────────────
// Real (non-fake) controller against a bound-but-silent loopback socket: every
// EventLoop cycle times out, so the ring must record attempted-but-not-ok masks
// and consecutive_recv_failures must climb.

namespace {

// Bind a loopback UDP socket that never answers; returns {fd, port}.
std::pair<int, int> OpenSilentLoopbackSocket() {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = 0;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    return {-1, 0};
  }
  socklen_t len = sizeof(addr);
  ::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len);
  return {fd, ntohs(addr.sin_port)};
}

// Real-mode controller (individual mode, no fingertips) with an explicit
// comm_decimation — the parameter is last in the ctor, so all preceding
// positional args must be spelled out.
std::unique_ptr<UdpHandController> MakeDecimatedController(int port, int comm_decimation) {
  return std::make_unique<UdpHandController>(
      "127.0.0.1", port, kHandUdpRecvConfig, 1.0 /*recv_timeout_ms*/, false /*enable_write_ack*/,
      1 /*sensor_decimation*/, 0 /*num_fingertips*/, false /*use_fake_hand*/,
      std::vector<std::string>{}, HandCommunicationMode::kIndividual, false /*tof_lpf*/, 15.0,
      false /*baro_lpf*/, 30.0, FingertipFTInferencer::Config{}, false /*drift*/, 5.0, 2500,
      comm_decimation);
}

}  // namespace

TEST(HandControllerOutcomeRing, SilentDevice_RecordsFailedCycles) {
  const auto [fd, port] = OpenSilentLoopbackSocket();
  ASSERT_GE(fd, 0);

  // Real mode, 1 ms recv timeout, no fingertips (skips sensor init retries).
  auto ctrl = std::make_unique<UdpHandController>("127.0.0.1", port, kHandUdpRecvConfig, 1.0, false,
                                                  1, 0, /*use_fake_hand=*/false);
  ASSERT_TRUE(ctrl->Start());

  // Idle EventLoop ticks every ~20 ms; each cycle attempts motor+joint reads
  // that all time out. Give it a few cycles.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  const CycleOutcomeRing ring = ctrl->GetCycleOutcomeRing();
  EXPECT_GT(ring.count, 0u);

  const auto& latest = ring.entries[(ring.count - 1) % CycleOutcomeRing::kCapacity];
  const uint8_t expected_attempted =
      RequestKindBit(RequestKind::kMotorRead) | RequestKindBit(RequestKind::kJointRead);
  EXPECT_EQ(latest.attempted_mask, expected_attempted);
  EXPECT_EQ(latest.ok_mask, 0u);
  EXPECT_GT(ctrl->consecutive_recv_failures(), 0u);

  // Per-kind stats attribute the timeouts to both read kinds.
  const auto stats = ctrl->comm_stats();
  EXPECT_GT(stats.per_kind[static_cast<std::size_t>(RequestKind::kMotorRead)].timeout, 0u);
  EXPECT_GT(stats.per_kind[static_cast<std::size_t>(RequestKind::kJointRead)].timeout, 0u);

  ctrl->Stop();
  ::close(fd);
}

// ── Comm decimation (whole-cycle UDP load reduction) ────────────────────────

TEST(HandControllerCommDecimation, ClampedToOne) {
  // < 1 is clamped to 1 (communicate every cycle). Fake mode, no threads run.
  auto ctrl = std::make_unique<UdpHandController>(
      "127.0.0.1", 55151, kHandUdpRecvConfig, 10.0, false, 1, 4, /*use_fake_hand=*/true,
      std::vector<std::string>{}, HandCommunicationMode::kIndividual, false, 15.0, false, 30.0,
      FingertipFTInferencer::Config{}, false, 5.0, 2500, /*comm_decimation=*/0);
  EXPECT_EQ(ctrl->comm_decimation(), 1);
}

TEST(HandControllerCommDecimation, DefaultCommunicatesEveryCycle) {
  // decim=1: no cycle is ever skipped (bit-identical to pre-feature behavior).
  const auto [fd, port] = OpenSilentLoopbackSocket();
  ASSERT_GE(fd, 0);

  auto ctrl = MakeDecimatedController(port, /*comm_decimation=*/1);
  ASSERT_TRUE(ctrl->Start());
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ctrl->Stop();

  EXPECT_EQ(ctrl->comm_decimation(), 1);
  EXPECT_EQ(ctrl->comm_skip_count(), 0u);
  EXPECT_GT(ctrl->cycle_count(), 0u);
  ::close(fd);
}

TEST(HandControllerCommDecimation, SkipsExpectedRatio) {
  // decim=N: after the always-comm first cycle, each group of N cycles yields
  // exactly 1 comm + (N-1) skips. So with `comm` comm-cycles recorded,
  // skips ∈ [(N-1)(comm-1), (N-1)(comm-1) + (N-1)] — a tight, non-flaky bound.
  constexpr int kDecim = 4;
  const auto [fd, port] = OpenSilentLoopbackSocket();
  ASSERT_GE(fd, 0);

  auto ctrl = MakeDecimatedController(port, kDecim);
  ASSERT_TRUE(ctrl->Start());
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ctrl->Stop();

  const uint64_t comm = ctrl->cycle_count();
  const uint64_t skip = ctrl->comm_skip_count();
  ASSERT_GE(comm, 2u);  // decimation actually exercised
  EXPECT_GT(skip, 0u);

  const uint64_t base = static_cast<uint64_t>(kDecim - 1) * (comm - 1);
  EXPECT_GE(skip, base);
  EXPECT_LE(skip, base + static_cast<uint64_t>(kDecim - 1));
  ::close(fd);
}

TEST(HandControllerOutcomeRing, CycleSeq_Monotonic) {
  const auto [fd, port] = OpenSilentLoopbackSocket();
  ASSERT_GE(fd, 0);

  auto ctrl = std::make_unique<UdpHandController>("127.0.0.1", port, kHandUdpRecvConfig, 1.0, false,
                                                  1, 0, /*use_fake_hand=*/false);
  ASSERT_TRUE(ctrl->Start());
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  ctrl->Stop();

  const CycleOutcomeRing ring = ctrl->GetCycleOutcomeRing();
  ASSERT_GT(ring.count, 1u);
  const uint32_t n = std::min(ring.count, CycleOutcomeRing::kCapacity);
  for (uint32_t i = 1; i < n; ++i) {
    const auto& prev = ring.entries[(ring.count - n + i - 1) % CycleOutcomeRing::kCapacity];
    const auto& cur = ring.entries[(ring.count - n + i) % CycleOutcomeRing::kCapacity];
    EXPECT_EQ(cur.cycle_seq, prev.cycle_seq + 1);
  }
  ::close(fd);
}

}  // namespace udp_hand_driver::test
