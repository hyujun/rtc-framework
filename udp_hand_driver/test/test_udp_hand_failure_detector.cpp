// Unit tests for hand_failure_detector.hpp — failure detection logic.
//
// Tier 2: Uses fake_hand UdpHandController. Requires rclcpp for logging.
// Tests run the detector's 50Hz thread and verify failure detection.

#include "udp_hand_driver/udp_hand_failure_detector.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>

namespace udp_hand_driver::test {

using namespace std::chrono_literals;

// ── Fixture: fake-hand controller + failure detector ────────────────────────

class HandFailureDetectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    controller_ = std::make_unique<UdpHandController>(UdpHandControllerConfig{
        .target_ip = "127.0.0.1",
        .target_port = 55151,
        .num_fingertips = 4,
        .use_fake_hand = true,
    });
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

  std::unique_ptr<UdpHandController> controller_;
};

// ── Motor all-zero detection ────────────────────────────────────────────────

TEST_F(HandFailureDetectorTest, MotorAllZero_TriggersFailure) {
  UdpHandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 3;
  cfg.check_motor = true;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;  // disable rate check

  UdpHandFailureDetector detector(*controller_, cfg);

  std::string failure_reason;
  detector.SetFailureCallback([&](const std::string& reason) { failure_reason = reason; });

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

TEST_F(HandFailureDetectorTest, MotorNonZero_NoAllZeroFailure) {
  UdpHandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 3;
  cfg.check_motor = true;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  UdpHandFailureDetector detector(*controller_, cfg);

  std::string failure_reason;
  detector.SetFailureCallback([&](const std::string& reason) { failure_reason = reason; });

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

TEST_F(HandFailureDetectorTest, MotorDuplicate_TriggersFailure) {
  UdpHandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 3;
  cfg.check_motor = true;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  UdpHandFailureDetector detector(*controller_, cfg);

  std::string failure_reason;
  detector.SetFailureCallback([&](const std::string& reason) { failure_reason = reason; });

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

TEST_F(HandFailureDetectorTest, SensorAllZero_TriggersFailure) {
  UdpHandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 3;
  cfg.check_motor = false;
  cfg.check_sensor = true;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  UdpHandFailureDetector detector(*controller_, cfg);

  std::string failure_reason;
  detector.SetFailureCallback([&](const std::string& reason) { failure_reason = reason; });

  // Fake mode echoes sensor_data as all-zeros. Feed DURING the detector run so
  // each fresh read advances sensor_seq — the freshness gate evaluates only
  // fresh snapshots, exactly as a genuinely dead sensor read every cycle.
  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 1.0f;  // non-zero motor to avoid motor failures
  detector.Start();
  for (int i = 0; i < 40; ++i) {
    controller_->SendCommandAndRequestStates(cmd);
    std::this_thread::sleep_for(5ms);
  }
  detector.Stop();

  EXPECT_TRUE(detector.failed());
  EXPECT_NE(failure_reason.find("hand_sensor_all_zero"), std::string::npos);
}

// ── 1b force-layout sensor detection (sensor_force fx,fy,fz) ─────────────────
// Fake mode mirrors the command into sensor_force, so a zero command → all-zero
// force, a constant command → duplicate force, and a changing command → neither.

TEST_F(HandFailureDetectorTest, SensorForceAllZero_TriggersFailure) {
  UdpHandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 3;
  cfg.check_motor = false;
  cfg.check_sensor = true;
  cfg.sensor_force_layout = true;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  UdpHandFailureDetector detector(*controller_, cfg);

  std::string failure_reason;
  detector.SetFailureCallback([&](const std::string& reason) { failure_reason = reason; });

  // Zero command → sensor_force is all-zero (dead/disconnected sensor signature).
  // Feed DURING the run so each fresh read advances sensor_seq (freshness gate).
  std::array<float, kNumHandMotors> cmd{};
  detector.Start();
  for (int i = 0; i < 40; ++i) {
    controller_->SendCommandAndRequestStates(cmd);
    std::this_thread::sleep_for(5ms);
  }
  detector.Stop();

  EXPECT_TRUE(detector.failed());
  EXPECT_NE(failure_reason.find("hand_sensor_all_zero"), std::string::npos);
}

TEST_F(HandFailureDetectorTest, SensorForceDuplicate_TriggersFailure) {
  UdpHandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 3;
  cfg.check_motor = false;
  cfg.check_sensor = true;
  cfg.sensor_force_layout = true;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  UdpHandFailureDetector detector(*controller_, cfg);

  std::string failure_reason;
  detector.SetFailureCallback([&](const std::string& reason) { failure_reason = reason; });

  // Constant non-zero command → non-zero but identical force on every fresh read
  // (genuine stalled feed: sensor IS read each cycle, seq advances, value frozen).
  // all_zero cannot fire (non-zero), so duplicate is the expected signal. Feed
  // DURING the run so seq advances on each read (freshness gate).
  std::array<float, kNumHandMotors> same_cmd{};
  same_cmd[0] = 3.0f;
  detector.Start();
  for (int i = 0; i < 40; ++i) {
    controller_->SendCommandAndRequestStates(same_cmd);
    std::this_thread::sleep_for(5ms);
  }
  detector.Stop();

  EXPECT_TRUE(detector.failed());
  EXPECT_NE(failure_reason.find("hand_sensor_duplicate"), std::string::npos);
  EXPECT_EQ(failure_reason.find("all_zero"), std::string::npos);
}

TEST_F(HandFailureDetectorTest, SensorForceChanging_NoFailure) {
  UdpHandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 5;
  cfg.check_motor = false;
  cfg.check_sensor = true;
  cfg.sensor_force_layout = true;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  UdpHandFailureDetector detector(*controller_, cfg);

  std::string failure_reason;
  detector.SetFailureCallback([&](const std::string& reason) { failure_reason = reason; });

  // Feed a changing non-zero command faster than the 50 Hz detector polls so each
  // poll sees a fresh, distinct force block — neither all-zero nor duplicate.
  detector.Start();
  for (int i = 0; i < 40; ++i) {
    std::array<float, kNumHandMotors> cmd{};
    cmd[0] = 1.0f + 0.1f * static_cast<float>(i);
    controller_->SendCommandAndRequestStates(cmd);
    std::this_thread::sleep_for(10ms);
  }
  detector.Stop();

  EXPECT_FALSE(detector.failed());
}

// ── Freshness gate: frozen sensor_seq must not false-fire (#4) ────────────────
// The detector (~50 Hz) polls faster than the decimated sensor rate, so most
// polls carry the SAME published snapshot. A single fresh read followed by many
// polls of that frozen snapshot (sensor_seq unchanged) is a decimated republish,
// NOT a stalled feed — the freshness gate must evaluate it only once, so no
// duplicate accumulates. (Pre-fix this false-fired hand_sensor_duplicate.)
TEST_F(HandFailureDetectorTest, SensorFrozenSeq_NoFalseDuplicate) {
  UdpHandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 3;
  cfg.check_motor = false;
  cfg.check_sensor = true;
  cfg.sensor_force_layout = true;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  UdpHandFailureDetector detector(*controller_, cfg);

  std::string failure_reason;
  detector.SetFailureCallback([&](const std::string& reason) { failure_reason = reason; });

  // One fresh non-zero read → sensor_seq advances exactly once, then freezes
  // (no further feeds), mimicking a snapshot republished at the decimated rate.
  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 3.0f;
  controller_->SendCommandAndRequestStates(cmd);

  detector.Start();
  std::this_thread::sleep_for(200ms);  // ~10 polls of the frozen snapshot
  detector.Stop();

  EXPECT_FALSE(detector.failed());
}

// ── Config: disable checks ──────────────────────────────────────────────────

TEST_F(HandFailureDetectorTest, DisableMotorCheck_NoFailureOnZero) {
  UdpHandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 3;
  cfg.check_motor = false;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  UdpHandFailureDetector detector(*controller_, cfg);

  bool callback_called = false;
  detector.SetFailureCallback([&](const std::string& /*reason*/) { callback_called = true; });

  std::array<float, kNumHandMotors> zero_cmd{};
  FeedCommands(zero_cmd, 10);

  detector.Start();
  std::this_thread::sleep_for(200ms);
  detector.Stop();

  EXPECT_FALSE(detector.failed());
  EXPECT_FALSE(callback_called);
}

TEST_F(HandFailureDetectorTest, CustomThreshold_HighThresholdNoFailure) {
  UdpHandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 1000;  // very high threshold
  cfg.check_motor = true;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  UdpHandFailureDetector detector(*controller_, cfg);

  bool callback_called = false;
  detector.SetFailureCallback([&](const std::string& /*reason*/) { callback_called = true; });

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

TEST_F(HandFailureDetectorTest, FailureCallbackOnceOnly) {
  UdpHandFailureDetectorConfig cfg{};
  cfg.failure_threshold = 2;
  cfg.check_motor = true;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;

  UdpHandFailureDetector detector(*controller_, cfg);

  int callback_count = 0;
  detector.SetFailureCallback([&](const std::string& /*reason*/) { ++callback_count; });

  std::array<float, kNumHandMotors> zero_cmd{};
  FeedCommands(zero_cmd, 20);

  detector.Start();
  std::this_thread::sleep_for(300ms);
  detector.Stop();

  // compare_exchange_strong ensures only one callback
  EXPECT_EQ(callback_count, 1);
}

// ── Lifecycle ───────────────────────────────────────────────────────────────

TEST_F(HandFailureDetectorTest, StartStop_Clean) {
  UdpHandFailureDetectorConfig cfg{};
  UdpHandFailureDetector detector(*controller_, cfg);

  detector.Start();
  std::this_thread::sleep_for(50ms);
  detector.Stop();
  EXPECT_FALSE(detector.failed());
}

TEST_F(HandFailureDetectorTest, StopWithoutStart_Safe) {
  UdpHandFailureDetectorConfig cfg{};
  UdpHandFailureDetector detector(*controller_, cfg);
  detector.Stop();  // Should not crash
}

TEST_F(HandFailureDetectorTest, DoubleStart_Ignored) {
  UdpHandFailureDetectorConfig cfg{};
  cfg.check_motor = false;
  cfg.check_sensor = false;
  cfg.check_link = false;
  cfg.min_rate_hz = 0.0;
  UdpHandFailureDetector detector(*controller_, cfg);

  detector.Start();
  detector.Start();  // Second start should be ignored
  std::this_thread::sleep_for(50ms);
  detector.Stop();
}

// ── Link-down forensic dump + single-shot latch ─────────────────────────────
// Real (non-fake) controller against a bound-but-silent loopback socket: every
// EventLoop cycle times out, consecutive_recv_failures crosses the threshold,
// CheckLink runs the one-shot forensic dump (per-kind table + ring decode) and
// RaiseFailure latches exactly once even though the condition persists.

TEST(HandFailureDetectorLinkDown, DumpRunsAndFailureLatchesOnce) {
  // Silent loopback socket (never answers).
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(fd, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = 0;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ASSERT_EQ(::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
  socklen_t len = sizeof(addr);
  ::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len);
  const int port = ntohs(addr.sin_port);

  // Real mode, 1 ms recv timeout, no fingertips (skips sensor init retries).
  auto controller = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = port,
      .recv_timeout_ms = 1.0,
      .num_fingertips = 0,
  });
  ASSERT_TRUE(controller->Start());

  UdpHandFailureDetectorConfig cfg{};
  cfg.check_motor = false;
  cfg.check_sensor = false;
  cfg.check_link = true;
  cfg.link_fail_threshold = 3;
  cfg.min_rate_hz = 0.0;

  UdpHandFailureDetector detector(*controller, cfg);

  std::atomic<int> callback_count{0};
  std::string failure_reason;
  detector.SetFailureCallback([&](const std::string& reason) {
    failure_reason = reason;
    callback_count.fetch_add(1);
  });

  detector.Start();
  // Failures accumulate at the ~50 Hz idle EventLoop cadence; the link stays
  // down for many detector polls — the latch must still fire only once.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  detector.Stop();

  EXPECT_TRUE(detector.failed());
  EXPECT_EQ(callback_count.load(), 1);
  EXPECT_NE(failure_reason.find("hand_udp_link_down"), std::string::npos);

  controller->Stop();
  ::close(fd);
}

// ── Link health (fake_hand always has 0 failures) ───────────────────────────

TEST_F(HandFailureDetectorTest, LinkCheck_FakeHandNoFailure) {
  UdpHandFailureDetectorConfig cfg{};
  cfg.check_motor = false;
  cfg.check_sensor = false;
  cfg.check_link = true;
  cfg.link_fail_threshold = 5;
  cfg.min_rate_hz = 0.0;

  UdpHandFailureDetector detector(*controller_, cfg);

  bool callback_called = false;
  detector.SetFailureCallback([&](const std::string& /*reason*/) { callback_called = true; });

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

// ── link_fail_timeout_ms → cycles conversion (node-layer helper) ─────────────
// The node converts the time-based link-fail budget into the cycle count the
// detector's CheckLink consumes: cycles = ms/1000 · loop_rate_hz / comm_dec,
// floored at 1. Verified directly on the free function.

TEST(LinkFailCyclesFromTimeoutMs, NominalRate) {
  // 100 ms at 500 Hz, no decimation → 50 cycles.
  EXPECT_EQ(LinkFailCyclesFromTimeoutMs(100.0, 500.0, 1), 50);
}

TEST(LinkFailCyclesFromTimeoutMs, DecimationScalesDown) {
  // comm_decimation halves the effective read rate → half the cycles.
  EXPECT_EQ(LinkFailCyclesFromTimeoutMs(100.0, 500.0, 2), 25);
}

TEST(LinkFailCyclesFromTimeoutMs, RoundsToNearest) {
  // 100 ms at 333 Hz → 33.3 → 33.
  EXPECT_EQ(LinkFailCyclesFromTimeoutMs(100.0, 333.0, 1), 33);
}

TEST(LinkFailCyclesFromTimeoutMs, FlooredAtOne) {
  // A tiny-but-positive timeout must still map to at least one cycle.
  EXPECT_EQ(LinkFailCyclesFromTimeoutMs(0.1, 100.0, 1), 1);
  // Guards against a zero/negative comm_decimation dividing by zero.
  EXPECT_EQ(LinkFailCyclesFromTimeoutMs(100.0, 500.0, 0), 50);
}

// ── ClampCommDecimation: shared divisor floor (constants header) ─────────────
TEST(ClampCommDecimation, PassesThroughValidValues) {
  EXPECT_EQ(udp_hand_driver::ClampCommDecimation(1), 1);
  EXPECT_EQ(udp_hand_driver::ClampCommDecimation(2), 2);
  EXPECT_EQ(udp_hand_driver::ClampCommDecimation(100), 100);
}

TEST(ClampCommDecimation, FloorsNonPositiveToOne) {
  // A < 1 decimation would divide by zero / invert the rate → clamp to 1.
  EXPECT_EQ(udp_hand_driver::ClampCommDecimation(0), 1);
  EXPECT_EQ(udp_hand_driver::ClampCommDecimation(-5), 1);
}

// constexpr-evaluable: the floor rule holds at compile time.
static_assert(udp_hand_driver::ClampCommDecimation(0) == 1, "clamp floor");
static_assert(udp_hand_driver::ClampCommDecimation(3) == 3, "clamp passthrough");

// ── Startup grace: suppress link/rate failure during the boot transient ──────
// A silent socket makes every EventLoop cycle time out; with a large startup
// grace the link-down failure must NOT fire within the window, and with a short
// grace it must fire once the window elapses. Default grace 0 leaves every test
// above unaffected (RT-7).

namespace {
// Bind a silent loopback UDP socket (never answers). Returns {fd, port}.
std::pair<int, int> MakeSilentSocket() {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  EXPECT_GE(fd, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = 0;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  EXPECT_EQ(::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
  socklen_t len = sizeof(addr);
  ::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len);
  return {fd, ntohs(addr.sin_port)};
}

std::unique_ptr<UdpHandController> MakeSilentController(int port) {
  auto controller = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = port,
      .recv_timeout_ms = 1.0,
      .num_fingertips = 0,
  });
  return controller;
}
}  // namespace

TEST(HandFailureDetectorStartupGrace, LargeGrace_SuppressesLinkDown) {
  auto [fd, port] = MakeSilentSocket();
  ASSERT_GE(fd, 0);
  auto controller = MakeSilentController(port);
  ASSERT_TRUE(controller->Start());

  UdpHandFailureDetectorConfig cfg{};
  cfg.check_motor = false;
  cfg.check_sensor = false;
  cfg.check_link = true;
  cfg.link_fail_threshold = 3;
  cfg.min_rate_hz = 0.0;
  cfg.startup_grace_ms = 10000.0;  // 10 s — far longer than the run

  UdpHandFailureDetector detector(*controller, cfg);
  std::atomic<bool> fired{false};
  detector.SetFailureCallback([&](const std::string&) { fired.store(true); });

  detector.Start();
  std::this_thread::sleep_for(300ms);
  detector.Stop();

  EXPECT_FALSE(detector.failed());
  EXPECT_FALSE(fired.load());

  controller->Stop();
  ::close(fd);
}

TEST(HandFailureDetectorStartupGrace, ShortGrace_FiresAfterElapse) {
  auto [fd, port] = MakeSilentSocket();
  ASSERT_GE(fd, 0);
  auto controller = MakeSilentController(port);
  ASSERT_TRUE(controller->Start());

  UdpHandFailureDetectorConfig cfg{};
  cfg.check_motor = false;
  cfg.check_sensor = false;
  cfg.check_link = true;
  cfg.link_fail_threshold = 3;
  cfg.min_rate_hz = 0.0;
  cfg.startup_grace_ms = 50.0;  // elapses well within the run

  UdpHandFailureDetector detector(*controller, cfg);
  std::string reason;
  detector.SetFailureCallback([&](const std::string& r) { reason = r; });

  detector.Start();
  std::this_thread::sleep_for(400ms);
  detector.Stop();

  EXPECT_TRUE(detector.failed());
  EXPECT_NE(reason.find("hand_udp_link_down"), std::string::npos);

  controller->Stop();
  ::close(fd);
}

}  // namespace udp_hand_driver::test
