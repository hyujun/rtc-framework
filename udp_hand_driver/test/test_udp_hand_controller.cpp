// Unit tests for udp_hand_controller.hpp — UdpHandController.
//
// Tier 2: fake_hand=true bypasses UDP but — like ros2_control use_fake_hardware
// — still runs the CommLoop RT thread, driving commands through the first-order
// LPF model (RunFakeCommCycle / StepFakeModel). State therefore flows
// ASYNCHRONOUSLY through the self-clocked loop, so tests poll the published
// state instead of expecting a synchronous echo. The LPF math itself is covered
// deterministically (thread-free) by the FakeLpfStep suite. rclcpp::init() is
// NOT required — rclcpp logging works without node context.

#include "rtc_base/testing/wait_until.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "silent_loopback_socket.hpp"
#include "udp_hand_driver/hand_comm.hpp"
#include "udp_hand_driver/udp_hand_comm.hpp"
#include "udp_hand_driver/udp_hand_controller.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <utility>

namespace udp_hand_driver::test {

using namespace std::chrono_literals;

namespace {

// One CommLoop tick at the fixture's default 500 Hz — polling faster than the
// loop can produce just burns wakeups, and polling slower blurs the "advanced
// by N cycles" assertions below.
inline constexpr std::chrono::milliseconds kCommTickPoll{2};

// Polls `pred` every CommLoop tick up to `timeout`, returning the final
// predicate value. Binds this suite's poll budget for its 17 call sites; the
// loop itself is rtc::testing::WaitUntil (#356).
template <typename Pred>
[[nodiscard]] bool PollUntil(Pred&& pred, std::chrono::milliseconds timeout) {
  return rtc::testing::WaitUntil(pred, timeout, kCommTickPoll);
}

// Wait for the self-clocked fake CommLoop to publish at least one valid state.
UdpHandState WaitForFirstState(UdpHandController& ctrl) {
  UdpHandState st{};
  static_cast<void>(PollUntil(
      [&] {
        st = ctrl.GetLatestState();
        return st.valid;
      },
      1s));
  return st;
}

// Stage a command and wait until the LPF settles the read-back position onto it
// (fixtures use τ=0 → alpha=1 → settles within a couple of ticks). Returns the
// settled state (or the last observed state on timeout).
UdpHandState SendAndSettle(UdpHandController& ctrl, const std::array<float, kNumHandMotors>& cmd) {
  ctrl.SendCommandAndRequestStates(cmd);
  UdpHandState st{};
  static_cast<void>(PollUntil(
      [&] {
        st = ctrl.GetLatestState();
        if (!st.valid) {
          return false;
        }
        for (int i = 0; i < kNumHandMotors; ++i) {
          const auto iu = static_cast<std::size_t>(i);
          if (std::abs(st.motor_positions[iu] - cmd[iu]) > 1e-4f) {
            return false;
          }
        }
        return true;
      },
      1s));
  return st;
}

}  // namespace

// ── Test fixture: FakeHand UdpHandController ───────────────────────────────────

class FakeHandControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    controller_ = std::make_unique<UdpHandController>(UdpHandControllerConfig{
        .target_ip = "127.0.0.1",  // unused in fake mode
        .target_port = 55151,
        .num_fingertips = 4,
        .use_fake_hand = true,
        // τ=0 → LPF alpha=1 → instantaneous follow, so the read-back position
        // equals the command at steady state. Keeps the follow assertions exact
        // even though state now flows asynchronously through the CommLoop.
        .fake_lpf_time_constant_s = 0.0,
    });
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
  // Destructor in TearDown should cleanly stop (CommLoop Join()).
}

// ── LPF follow (fake mode; τ=0 → exact follow) ───────────────────────────────

TEST_F(FakeHandControllerTest, LpfFollow_MotorPositions) {
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  for (int i = 0; i < kNumHandMotors; ++i) {
    cmd[static_cast<std::size_t>(i)] = static_cast<float>(i) * 0.1f;
  }

  const auto state = SendAndSettle(*controller_, cmd);
  for (std::size_t i = 0; i < kNumHandMotors; ++i) {
    EXPECT_FLOAT_EQ(state.motor_positions[i], cmd[i]);
  }
}

TEST_F(FakeHandControllerTest, State_NumFingertips) {
  ASSERT_TRUE(controller_->Start());
  EXPECT_EQ(WaitForFirstState(*controller_).num_fingertips, 4);
}

TEST_F(FakeHandControllerTest, State_ValidFlag) {
  ASSERT_TRUE(controller_->Start());
  EXPECT_TRUE(WaitForFirstState(*controller_).valid);
}

TEST_F(FakeHandControllerTest, GetLatestPositions_FollowsCmd) {
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 1.5f;
  cmd[5] = -3.0f;
  SendAndSettle(*controller_, cmd);

  const auto positions = controller_->GetLatestPositions();
  EXPECT_FLOAT_EQ(positions[0], 1.5f);
  EXPECT_FLOAT_EQ(positions[5], -3.0f);
}

// ── CycleCount (self-clocked, advances on its own) ───────────────────────────

TEST_F(FakeHandControllerTest, CycleCount_AdvancesWhileRunning) {
  ASSERT_TRUE(controller_->Start());
  // The CommLoop self-clocks in fake mode — cycles accumulate without commands.
  ASSERT_TRUE(PollUntil([&] { return controller_->cycle_count() > 0; }, 1s));
  const auto c1 = controller_->cycle_count();
  ASSERT_TRUE(PollUntil([&] { return controller_->cycle_count() > c1 + 5; }, 1s));
  EXPECT_GT(controller_->cycle_count(), c1);
}

// ── Sensor freshness sequence (advances per fake cycle) ──────────────────────

TEST_F(FakeHandControllerTest, SensorSeq_AdvancesWhileRunning) {
  ASSERT_TRUE(controller_->Start());
  // Each fake cycle mirrors a fresh sensor read, so sensor_seq climbs monotonically
  // (the failure detector's freshness gate keys on the delta).
  ASSERT_TRUE(PollUntil([&] { return controller_->GetLatestState().sensor_seq > 0u; }, 1s));
  const auto s1 = controller_->GetLatestState().sensor_seq;
  ASSERT_TRUE(PollUntil([&] { return controller_->GetLatestState().sensor_seq > s1; }, 1s));
  EXPECT_GT(controller_->GetLatestState().sensor_seq, s1);
}

// ── Fake-hand LPF model (pure, deterministic — no threads) ───────────────────
// StepFakeModel delegates its per-joint math to the static FakeLpfStep, so the
// dynamics are verified here without the CommLoop.

TEST(FakeLpfStep, InstantFollowWhenAlphaOne) {
  // alpha=1 (τ=0): position jumps straight to the target in one step.
  const auto s = UdpHandController::FakeLpfStep(/*target=*/2.0f, /*pos_old=*/0.0f,
                                                /*alpha=*/1.0f, /*inv_dt=*/500.0f,
                                                /*kp=*/1.0f, /*kd=*/0.0f);
  EXPECT_FLOAT_EQ(s.pos, 2.0f);
  EXPECT_FLOAT_EQ(s.vel, (2.0f - 0.0f) * 500.0f);
  EXPECT_FLOAT_EQ(s.effort, 0.0f);  // kp*(target-pos) = kp*0
}

TEST(FakeLpfStep, SettledStateZeroVelZeroEffort) {
  // pos already at target → no motion, no PD torque.
  const auto s = UdpHandController::FakeLpfStep(1.0f, 1.0f, 0.02f, 500.0f, 3.0f, 0.5f);
  EXPECT_FLOAT_EQ(s.pos, 1.0f);
  EXPECT_FLOAT_EQ(s.vel, 0.0f);
  EXPECT_FLOAT_EQ(s.effort, 0.0f);
}

TEST(FakeLpfStep, FirstOrderLag) {
  // One step moves a fraction alpha toward the target; vel is its derivative.
  const auto s =
      UdpHandController::FakeLpfStep(4.0f, 0.0f, /*alpha=*/0.25f, /*inv_dt=*/500.0f, 0.0f, 0.0f);
  EXPECT_FLOAT_EQ(s.pos, 1.0f);           // 0 + 0.25*(4-0)
  EXPECT_FLOAT_EQ(s.vel, 1.0f * 500.0f);  // (1-0)*inv_dt
}

TEST(FakeLpfStep, ConvergesOverManySteps) {
  float pos = 0.0f;
  constexpr float kTarget = 1.0f;
  for (int i = 0; i < 2000; ++i) {
    pos =
        UdpHandController::FakeLpfStep(kTarget, pos, /*alpha=*/0.02f, /*inv_dt=*/500.0f, 0.0f, 0.0f)
            .pos;
  }
  EXPECT_NEAR(pos, kTarget, 1e-3f);
}

TEST(FakeLpfStep, EffortPdForm) {
  // Transient effort = kp*(target-pos) - kd*vel.
  constexpr float kTarget = 2.0f, kPosOld = 0.0f, kAlpha = 0.5f, kInvDt = 100.0f, kKp = 2.0f,
                  kKd = 0.1f;
  const auto s = UdpHandController::FakeLpfStep(kTarget, kPosOld, kAlpha, kInvDt, kKp, kKd);
  const float pos = kPosOld + kAlpha * (kTarget - kPosOld);      // 1.0
  const float vel = (pos - kPosOld) * kInvDt;                    // 100
  EXPECT_FLOAT_EQ(s.effort, kKp * (kTarget - pos) - kKd * vel);  // 2*1 - 0.1*100 = -8
}

// ── Strict per-channel link-down policy (#1) ─────────────────────────────────
// LinkDownDecision is the pure policy: a single dead channel latches even while
// others stay healthy. Crafted streak vectors exercise the channel→threshold
// mapping, write-echo/set-mode exclusion, and the threshold-0 disable.

namespace {
constexpr uint32_t kCommThr = 50;
constexpr uint32_t kSensorThr = 10;  // shorter (sensor polled at 1/5 the rate)

std::array<uint32_t, kNumRequestKinds> MakeFails(RequestKind kind, uint32_t streak) {
  std::array<uint32_t, kNumRequestKinds> fails{};
  fails[static_cast<std::size_t>(kind)] = streak;
  return fails;
}
}  // namespace

TEST(LinkDownDecision, AllHealthy_NoLatch) {
  const std::array<uint32_t, kNumRequestKinds> fails{};
  EXPECT_FALSE(UdpHandController::LinkDownDecision(fails, kCommThr, kSensorThr));
}

TEST(LinkDownDecision, MotorLatchesAtCommThreshold) {
  EXPECT_TRUE(UdpHandController::LinkDownDecision(MakeFails(RequestKind::kMotorRead, kCommThr),
                                                  kCommThr, kSensorThr));
  EXPECT_FALSE(UdpHandController::LinkDownDecision(MakeFails(RequestKind::kMotorRead, kCommThr - 1),
                                                   kCommThr, kSensorThr));
}

TEST(LinkDownDecision, JointDeadWhileMotorHealthy_StillLatches) {
  // The core #1 case: one channel dead, another perfectly healthy → strict latch.
  auto fails = MakeFails(RequestKind::kJointRead, kCommThr);
  fails[static_cast<std::size_t>(RequestKind::kMotorRead)] = 0;  // motor fully healthy
  EXPECT_TRUE(UdpHandController::LinkDownDecision(fails, kCommThr, kSensorThr));
}

TEST(LinkDownDecision, SensorUsesShorterSensorThreshold) {
  // Sensor latches at the sensor threshold, NOT the (longer) comm threshold.
  EXPECT_TRUE(UdpHandController::LinkDownDecision(MakeFails(RequestKind::kBulkSensor, kSensorThr),
                                                  kCommThr, kSensorThr));
  EXPECT_FALSE(UdpHandController::LinkDownDecision(
      MakeFails(RequestKind::kBulkSensor, kSensorThr - 1), kCommThr, kSensorThr));
  // Individual-mode sensor kind behaves identically.
  EXPECT_TRUE(UdpHandController::LinkDownDecision(MakeFails(RequestKind::kSensorRead, kSensorThr),
                                                  kCommThr, kSensorThr));
}

TEST(LinkDownDecision, CommandDrivenChannelsExcluded) {
  // write-echo + set-mode are not periodic health polls → never latch link-down,
  // even with an enormous streak.
  EXPECT_FALSE(UdpHandController::LinkDownDecision(MakeFails(RequestKind::kWriteEcho, 100000),
                                                   kCommThr, kSensorThr));
  EXPECT_FALSE(UdpHandController::LinkDownDecision(MakeFails(RequestKind::kSetMode, 100000),
                                                   kCommThr, kSensorThr));
}

TEST(LinkDownDecision, ZeroThresholdDisablesChannelClass) {
  // comm_threshold 0 disables motor/joint; sensor still active.
  EXPECT_FALSE(UdpHandController::LinkDownDecision(MakeFails(RequestKind::kMotorRead, 100000), 0,
                                                   kSensorThr));
  EXPECT_TRUE(UdpHandController::LinkDownDecision(MakeFails(RequestKind::kBulkSensor, kSensorThr),
                                                  0, kSensorThr));
}

// ── State mailbox ───────────────────────────────────────────────────────────
// These two tests used to observe the loop through SetCallback(), a
// std::function the CommLoop invoked on the SCHED_FIFO 65 thread. Issue #345
// removed it — the node now pulls from the SeqLock — so they observe the same
// two behaviours through the mailbox that replaced it. The assertions are not
// weakened: state_sequence() is the exact quantity the NRT consumer counts
// cycles with, so a loop that stops ticking or stops reflecting commands still
// fails here, and it fails one level closer to the mechanism.

TEST_F(FakeHandControllerTest, Mailbox_CarriesCommandedState) {
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 42.0f;
  SendAndSettle(*controller_, cmd);

  // Poll rather than assert instantly: the loop is self-clocked and
  // asynchronous, so the settle above does not pin which tick published last
  // (race observed as a CI failure under parallel package tests, PR #187).
  EXPECT_TRUE(
      PollUntil([&] { return controller_->GetLatestState().motor_positions[0] == 42.0f; }, 1s));
  EXPECT_FLOAT_EQ(controller_->GetLatestState().motor_positions[0], 42.0f);
}

TEST_F(FakeHandControllerTest, Mailbox_SequenceAdvancesPerTick) {
  ASSERT_TRUE(controller_->Start());

  // Self-clocked: the sequence advances without any command. SeqLock steps by 2
  // per Store, so >= 5 completed cycles is a delta of >= 10.
  const std::uint32_t s0 = controller_->state_sequence() & ~1U;
  ASSERT_TRUE(PollUntil([&] { return ((controller_->state_sequence() & ~1U) - s0) >= 10U; }, 1s));

  // And it is monotonic: a second window advances further, never backwards.
  const std::uint32_t s1 = controller_->state_sequence() & ~1U;
  EXPECT_GE(s1 - s0, 10U);
  ASSERT_TRUE(PollUntil([&] { return ((controller_->state_sequence() & ~1U) - s1) >= 10U; }, 1s));
}

// The low bit is the writer-in-progress flag, never cycle progress. The NRT
// consumer masks it off before differencing; a Store() that left the sequence
// odd would make every consumer poll see phantom progress.
TEST_F(FakeHandControllerTest, Mailbox_SequenceIsEvenBetweenWrites) {
  ASSERT_TRUE(controller_->Start());
  ASSERT_TRUE(PollUntil([&] { return controller_->state_sequence() >= 4U; }, 1s));

  int odd_observations = 0;
  for (int i = 0; i < 200; ++i) {
    if ((controller_->state_sequence() & 1U) != 0U) {
      ++odd_observations;
    }
  }
  // A 500 Hz loop whose Store() is two atomic stores plus one memcpy leaves the
  // odd window vanishingly small; catching it in every one of 200 samples would
  // mean the sequence is parked odd, i.e. a Store never completed.
  EXPECT_LT(odd_observations, 200);
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
  auto ctrl = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = 55151,
      .num_fingertips = 4,
      .use_fake_hand = true,
      .communication_mode = HandCommunicationMode::kBulk,
  });

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

TEST_F(FakeHandControllerTest, ConsecutiveRecvFailures_Zero) {
  EXPECT_EQ(controller_->consecutive_recv_failures(), 0u);
}

TEST_F(FakeHandControllerTest, TimingSummary_NoData) {
  // No Start() → no cycles → summary reports "no data".
  const auto summary = controller_->TimingSummary();
  EXPECT_NE(summary.find("no data"), std::string::npos);
}

// ── E-Stop flag ─────────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, SetEstopFlag_Settable) {
  std::atomic<bool> flag{false};
  controller_->SetEstopFlag(&flag);  // set before Start (read on the loop thread)
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  controller_->SendCommandAndRequestStates(cmd);
  // Loop runs with the E-Stop unset — cycles accumulate, no crash.
  EXPECT_TRUE(PollUntil([&] { return controller_->cycle_count() > 0; }, 1s));
}

// ── Run length (#927 I-2) ───────────────────────────────────────────────────
// ElapsedSeconds() is the run length the stats JSON reports: 0 before any
// Start, wall time while the loop runs, and frozen at the end of the last
// completed cycle once it stops. An E-Stop self-exit leaves the node ACTIVE
// with a dead loop, and the idle after it is not part of the run. The 300 ms
// waits below exceed the 0.25 s tolerance, so a "now − start" run length fails
// them by construction.

TEST_F(FakeHandControllerTest, ElapsedSeconds_ZeroBeforeStart) {
  EXPECT_EQ(controller_->ElapsedSeconds(), 0.0);
}

TEST_F(FakeHandControllerTest, ElapsedSeconds_FreezesAtTheLastCycleAfterAnEStop) {
  std::atomic<bool> estop{false};
  controller_->SetEstopFlag(&estop);
  ASSERT_TRUE(controller_->Start());
  std::this_thread::sleep_for(400ms);
  const double running_a = controller_->ElapsedSeconds();
  std::this_thread::sleep_for(50ms);
  const double running_b = controller_->ElapsedSeconds();
  EXPECT_GE(running_a, 0.4);
  EXPECT_GT(running_b, running_a) << "ElapsedSeconds() must advance while the loop runs";

  estop.store(true);
  ASSERT_TRUE(PollUntil([&] { return !controller_->IsRunning(); }, 500ms));
  const double at_trip = controller_->ElapsedSeconds();
  // The run that ticked for ≥ 0.45 s is counted — loosely, because the last
  // completed cycle may predate running_b by however long the loop was
  // descheduled (no SCHED_FIFO in CI).
  EXPECT_GT(at_trip, 0.2);
  std::this_thread::sleep_for(300ms);
  EXPECT_LT(controller_->ElapsedSeconds() - at_trip, 0.25)
      << "the idle after the E-Stop leaked into the run length";

  controller_->Stop();
  EXPECT_LT(controller_->ElapsedSeconds() - at_trip, 0.25) << "Stop() restarted the clock";
}

TEST_F(FakeHandControllerTest, ElapsedSeconds_SynchronousDriveFreezesAtTheLastStep) {
  ASSERT_TRUE(controller_->InitFakeForTest());
  const std::array<float, kNumHandMotors> cmd{};
  for (int i = 0; i < 5; ++i) {
    controller_->StepFakeCycleForTest(cmd);
  }
  std::this_thread::sleep_for(300ms);
  controller_->Stop();
  EXPECT_LT(controller_->ElapsedSeconds(), 0.25)
      << "the wait after the last fake cycle leaked into the run length";
}

TEST_F(FakeHandControllerTest, ElapsedSeconds_RestartDoesNotInheritThePreviousRun) {
  ASSERT_TRUE(controller_->Start());
  std::this_thread::sleep_for(400ms);
  controller_->Stop();
  ASSERT_GE(controller_->ElapsedSeconds(), 0.35);

  ASSERT_TRUE(controller_->Start());
  EXPECT_LT(controller_->ElapsedSeconds(), 0.25);
  controller_->Stop();
  EXPECT_LT(controller_->ElapsedSeconds(), 0.25);
}

// ── Sensor init status ──────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, SensorInit_FakeModeNotInitialized) {
  // In fake mode, no sensor init happens (no UDP handshake).
  ASSERT_TRUE(controller_->Start());
  EXPECT_FALSE(controller_->IsSensorInitialized());
}

// ── HasStateBeenRead ─────────────────────────────────────────────────────────

TEST_F(FakeHandControllerTest, HasStateBeenRead_TrueAfterFirstCycle) {
  ASSERT_TRUE(controller_->Start());
  // Fake cycles report any_recv_ok=true, so the first-state latch flips true.
  ASSERT_TRUE(PollUntil([&] { return controller_->HasStateBeenRead(); }, 1s));
  EXPECT_TRUE(controller_->GetLatestState().valid);
}

// ── Num fingertips clamping ─────────────────────────────────────────────────

TEST(HandControllerConfig, NumFingertips_ClampedToNames) {
  // If fingertip_names has fewer entries than num_fingertips, it should be clamped.
  auto ctrl = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = 55151,
      .num_fingertips = 10,  // only 2 names below
      .use_fake_hand = true,
      .fingertip_names = {"thumb", "index"},
  });

  ASSERT_TRUE(ctrl->Start());
  const auto state = WaitForFirstState(*ctrl);
  EXPECT_EQ(state.num_fingertips, 2);  // clamped to name count
  ctrl->Stop();
}

TEST(HandControllerConfig, NumFingertips_NegativeClamped) {
  auto ctrl = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = 55151,
      .num_fingertips = -1,
      .use_fake_hand = true,
  });

  ASSERT_TRUE(ctrl->Start());
  const auto state = WaitForFirstState(*ctrl);
  EXPECT_EQ(state.num_fingertips, 0);
  ctrl->Stop();
}

// ── Bulk mode with fake hand ───────────────────────────────────────────────

TEST(HandControllerConfig, BulkMode_FakeFollow) {
  auto ctrl = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = 55151,
      .num_fingertips = 4,
      .use_fake_hand = true,
      .communication_mode = HandCommunicationMode::kBulk,
      .fake_lpf_time_constant_s = 0.0,  // exact follow
  });

  ASSERT_TRUE(ctrl->Start());

  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 7.77f;
  cmd[9] = -2.5f;
  const auto state = SendAndSettle(*ctrl, cmd);
  EXPECT_FLOAT_EQ(state.motor_positions[0], 7.77f);
  EXPECT_FLOAT_EQ(state.motor_positions[9], -2.5f);
  EXPECT_TRUE(state.valid);
  ctrl->Stop();
}

// ── Rapid successive commands — last value wins ────────────────────────────

TEST_F(FakeHandControllerTest, RapidCommands_LastValueWins) {
  ASSERT_TRUE(controller_->Start());

  std::array<float, kNumHandMotors> cmd{};
  for (int i = 0; i < 100; ++i) {
    cmd[0] = static_cast<float>(i);
    controller_->SendCommandAndRequestStates(cmd);
  }

  // The staged SeqLock is last-wins; the loop latches 99 and (τ=0) tracks it.
  ASSERT_TRUE(PollUntil(
      [&] { return std::abs(controller_->GetLatestState().motor_positions[0] - 99.0f) < 1e-4f; },
      1s));
  EXPECT_FLOAT_EQ(controller_->GetLatestState().motor_positions[0], 99.0f);
}

// ── Timing stats after cycles ──────────────────────────────────────────────

TEST_F(FakeHandControllerTest, TimingSummary_AfterCycles) {
  ASSERT_TRUE(controller_->Start());
  // Fake cycles now push per-tick timing records, so the summary is populated.
  ASSERT_TRUE(PollUntil([&] { return controller_->cycle_count() > 5; }, 1s));
  EXPECT_FALSE(controller_->TimingSummary().empty());
}

// ── ActualSensorRateHz accessor ────────────────────────────────────────────

TEST_F(FakeHandControllerTest, ActualSensorRateHz_NonNegative) {
  EXPECT_GE(controller_->actual_sensor_rate_hz(), 0.0);
}

// ── Zero fingertips mode ───────────────────────────────────────────────────

TEST(HandControllerConfig, ZeroFingertips_NoSensor) {
  auto ctrl = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = 55151,
      .num_fingertips = 0,
      .use_fake_hand = true,
      .fake_lpf_time_constant_s = 0.0,
  });

  ASSERT_TRUE(ctrl->Start());

  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 3.14f;
  const auto state = SendAndSettle(*ctrl, cmd);
  EXPECT_FLOAT_EQ(state.motor_positions[0], 3.14f);
  EXPECT_EQ(state.num_fingertips, 0);
  EXPECT_TRUE(state.valid);
  ctrl->Stop();
}

// ── Max fingertips clamping ────────────────────────────────────────────────

TEST(HandControllerConfig, MaxFingertips_Clamped) {
  auto ctrl = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = 55151,
      .num_fingertips = 100,  // > kMaxFingertips
      .use_fake_hand = true,
  });

  ASSERT_TRUE(ctrl->Start());
  const auto state = WaitForFirstState(*ctrl);
  // Clamped to kMaxFingertips (8), but further clamped by default fingertip_names (4).
  EXPECT_LE(state.num_fingertips, kMaxFingertips);
  ctrl->Stop();
}

// ── Cycle outcome ring (link-down forensics) ────────────────────────────────
// Real (non-fake) controller against a bound-but-silent loopback socket: every
// EventLoop cycle times out, so the ring must record attempted-but-not-ok masks
// and consecutive_recv_failures must climb.

namespace {

// Real-mode controller (individual mode, no fingertips) with an explicit
// comm_decimation.
std::unique_ptr<UdpHandController> MakeDecimatedController(int port, int comm_decimation) {
  return std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = port,
      .recv_timeout_ms = 1.0,
      .num_fingertips = 0,
      .comm_decimation = comm_decimation,
  });
}

}  // namespace

TEST(HandControllerOutcomeRing, SilentDevice_RecordsFailedCycles) {
  udp_hand_driver::test_support::SilentLoopbackSocket silent;
  ASSERT_TRUE(silent.ok());
  const int port = silent.port();

  // Real mode, 1 ms recv timeout, no fingertips (skips sensor init retries).
  auto ctrl = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = port,
      .recv_timeout_ms = 1.0,
      .num_fingertips = 0,
  });
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
}

// ── Comm decimation (whole-cycle UDP load reduction) ────────────────────────

TEST(HandControllerCommDecimation, ClampedToOne) {
  // < 1 is clamped to 1 (communicate every cycle). Fake mode.
  auto ctrl = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = 55151,
      .num_fingertips = 4,
      .use_fake_hand = true,
      .comm_decimation = 0,
  });
  EXPECT_EQ(ctrl->comm_decimation(), 1);
}

TEST(HandControllerCommDecimation, DefaultCommunicatesEveryCycle) {
  // decim=1: no cycle is ever skipped (bit-identical to pre-feature behavior).
  udp_hand_driver::test_support::SilentLoopbackSocket silent;
  ASSERT_TRUE(silent.ok());
  const int port = silent.port();

  auto ctrl = MakeDecimatedController(port, /*comm_decimation=*/1);
  ASSERT_TRUE(ctrl->Start());
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ctrl->Stop();

  EXPECT_EQ(ctrl->comm_decimation(), 1);
  EXPECT_EQ(ctrl->comm_skip_count(), 0u);
  EXPECT_GT(ctrl->cycle_count(), 0u);
}

TEST(HandControllerCommDecimation, SkipsExpectedRatio) {
  // decim=N: after the always-comm first cycle, each group of N cycles yields
  // exactly 1 comm + (N-1) skips. So with `comm` comm-cycles recorded,
  // skips ∈ [(N-1)(comm-1), (N-1)(comm-1) + (N-1)] — a tight, non-flaky bound.
  constexpr int kDecim = 4;
  udp_hand_driver::test_support::SilentLoopbackSocket silent;
  ASSERT_TRUE(silent.ok());
  const int port = silent.port();

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
}

// ── Self-clocked read loop + command-gated write ────────────────────────────
// Real (non-fake) controller on a silent loopback socket: the CommLoop ticks
// autonomously (no command needed), and the write UDP stays suppressed until a
// command has been staged AND the first state has been read. The positive path
// (write attempted exactly when commanded + state-read) needs a live echo peer
// and is covered by the Phase 5 hardware verification, not here.

TEST(HandControllerSelfClocked, CyclesWithoutCommand) {
  udp_hand_driver::test_support::SilentLoopbackSocket silent;
  ASSERT_TRUE(silent.ok());
  const int port = silent.port();

  auto ctrl = MakeDecimatedController(port, /*comm_decimation=*/1);
  ASSERT_TRUE(ctrl->Start());
  // No SendCommandAndRequestStates() — the loop is self-clocked, so cycles must
  // accumulate on their own.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ctrl->Stop();

  // Lenient lower bound (timing-robust; the exact rate depends on recv timeouts
  // and scheduler, so do not assert it).
  EXPECT_GT(ctrl->cycle_count(), 20u);
}

TEST(HandControllerSelfClocked, NoWriteAttempted_WithoutCommand) {
  udp_hand_driver::test_support::SilentLoopbackSocket silent;
  ASSERT_TRUE(silent.ok());
  const int port = silent.port();

  auto ctrl = MakeDecimatedController(port, /*comm_decimation=*/1);
  ASSERT_TRUE(ctrl->Start());
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ctrl->Stop();

  // Every recorded cycle ran read-only: with no command ever staged, the
  // write-echo bit must be absent from every attempted_mask.
  const CycleOutcomeRing ring = ctrl->GetCycleOutcomeRing();
  ASSERT_GT(ring.count, 0u);
  const uint32_t n = std::min(ring.count, CycleOutcomeRing::kCapacity);
  const uint8_t write_bit = RequestKindBit(RequestKind::kWriteEcho);
  for (uint32_t i = 0; i < n; ++i) {
    const auto& e = ring.entries[(ring.count - n + i) % CycleOutcomeRing::kCapacity];
    EXPECT_EQ(e.attempted_mask & write_bit, 0u);
  }
}

TEST(HandControllerSelfClocked, NoWriteAttempted_BeforeFirstStateRead) {
  udp_hand_driver::test_support::SilentLoopbackSocket silent;
  ASSERT_TRUE(silent.ok());
  const int port = silent.port();

  auto ctrl = MakeDecimatedController(port, /*comm_decimation=*/1);
  ASSERT_TRUE(ctrl->Start());

  // Stage commands repeatedly, but the silent socket fails every read so
  // state_read_once_ never flips true — the startup no-jump gate must still
  // suppress the write (write requires state_read_once_ AND a pending command).
  for (int i = 0; i < 20; ++i) {
    std::array<float, kNumHandMotors> cmd{};
    cmd.fill(0.3f);
    ctrl->SendCommandAndRequestStates(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ctrl->Stop();

  ASSERT_FALSE(ctrl->HasStateBeenRead());
  const CycleOutcomeRing ring = ctrl->GetCycleOutcomeRing();
  ASSERT_GT(ring.count, 0u);
  const uint32_t n = std::min(ring.count, CycleOutcomeRing::kCapacity);
  const uint8_t write_bit = RequestKindBit(RequestKind::kWriteEcho);
  for (uint32_t i = 0; i < n; ++i) {
    const auto& e = ring.entries[(ring.count - n + i) % CycleOutcomeRing::kCapacity];
    EXPECT_EQ(e.attempted_mask & write_bit, 0u);
  }
}

TEST(HandControllerOutcomeRing, CycleSeq_Monotonic) {
  udp_hand_driver::test_support::SilentLoopbackSocket silent;
  ASSERT_TRUE(silent.ok());
  const int port = silent.port();

  auto ctrl = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = port,
      .recv_timeout_ms = 1.0,
      .num_fingertips = 0,
  });
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
}

// ── E-Stop self-exit + restart reset (review #1/#2-A/#4) ────────────────────
// Real (non-fake) controller on a silent loopback socket with an E-Stop flag.
// The E-Stop is now serviced by a pure-atomic RequestLoopExit() from inside the
// CommLoop (no mutex/CV on the RT hot path), which clears running_ and unwinds
// the loop; the zero-write happens off the hot path in OnLoopAborted.

TEST(HandControllerEStop, SelfExitClearsRunning) {
  udp_hand_driver::test_support::SilentLoopbackSocket silent;
  ASSERT_TRUE(silent.ok());
  const int port = silent.port();

  std::atomic<bool> estop{true};  // asserted before Start → first tick self-exits
  auto ctrl = MakeDecimatedController(port, /*comm_decimation=*/1);
  ctrl->SetEstopFlag(&estop);
  ASSERT_TRUE(ctrl->Start());

  // The first CommLoop tick observes the E-Stop and self-exits: running_ is
  // cleared from inside the loop (#4) and the loop breaks via RequestLoopExit
  // (#2-A) with no mutex/CV on the hot path.
  EXPECT_TRUE(PollUntil([&] { return !ctrl->IsRunning(); }, std::chrono::milliseconds(500)))
      << "E-Stop did not clear IsRunning() within timeout";

  // The E-Stop branch returns before any read / cycle accounting, so nothing is
  // recorded from the terminal tick.
  EXPECT_EQ(ctrl->cycle_count(), 0u);
  EXPECT_EQ(ctrl->consecutive_recv_failures(), 0u);

  ctrl->Stop();  // idempotent after a self-exit
}

TEST(HandControllerEStop, RestartResetsCountersAfterFailures) {
  udp_hand_driver::test_support::SilentLoopbackSocket silent;
  ASSERT_TRUE(silent.ok());
  const int port = silent.port();

  std::atomic<bool> estop{false};
  auto ctrl = MakeDecimatedController(port, /*comm_decimation=*/1);
  ctrl->SetEstopFlag(&estop);

  // Run 1: no E-Stop; the silent device makes every read time out, so recv
  // failures and cycles accumulate.
  ASSERT_TRUE(ctrl->Start());
  ASSERT_TRUE(
      PollUntil([&] { return ctrl->consecutive_recv_failures() > 3; }, std::chrono::seconds(1)))
      << "failures did not accumulate against the silent device";
  ASSERT_GT(ctrl->cycle_count(), 0u);
  ctrl->Stop();
  // Stop() does not reset the link counters — stale state persists into restart.
  ASSERT_GT(ctrl->consecutive_recv_failures(), 0u);

  // Run 2: restart with E-Stop asserted so the first tick self-exits before it
  // can accrue fresh failures. This isolates the Start() reset (#1): the stale
  // consecutive_recv_failures / cycle_count / recv_error_count must be cleared
  // so the failure detector cannot observe a false link-down E-STOP on restart.
  estop.store(true);
  ASSERT_TRUE(ctrl->Start());
  EXPECT_TRUE(PollUntil([&] { return !ctrl->IsRunning(); }, std::chrono::milliseconds(500)))
      << "restarted controller did not self-exit on the asserted E-Stop";

  EXPECT_EQ(ctrl->consecutive_recv_failures(), 0u);
  EXPECT_EQ(ctrl->cycle_count(), 0u);
  EXPECT_EQ(ctrl->recv_error_count(), 0u);

  ctrl->Stop();
}

// ── HandComm seam: scripted comm double (epic #895 D25, #897) ──────────────
// Drives the REAL (non-fake) RunCommCycle against a deterministic HandComm so
// the plumbing the seam moved — plan hoisting, outcome → state / mask / streak,
// stats stamping, the write gate's feedforward argument and the E-Stop
// fire-and-forget — is pinned without sockets or timing. The double fills the
// read buffers with a recognisable pattern (motor i → base + i, joint i → base
// + 100 + i, finger f value k → 1000·(f+1) + k) and records every call.
//
// Threading: the script fields are written before Start() and read on the
// CommLoop thread; the counters are atomics so the test thread may poll them
// while the loop runs; the "last_*" captures are plain and are read only after
// Stop() (which joins the loop) or after the loop self-exited AND Stop() joined.

class ScriptedHandComm : public HandComm {
 public:
  // Script.
  bool script_motor_ok{true};  // bulk motor-space / individual pos
  bool script_joint_ok{true};  // bulk joint-space / individual joint pos
  bool script_vel_ok{true};    // individual vel
  ssize_t script_sensor_recvd{-1};
  uint8_t script_finger_mask{0};
  bool script_write_ok{true};  // WriteCommandWithEcho result
  float pattern_base{1.0f};

  // Observations.
  std::atomic<int> open_calls{0};
  std::atomic<int> close_calls{0};
  std::atomic<int> init_sensor_calls{0};
  std::atomic<int> reset_stats_calls{0};
  std::atomic<int> drain_calls{0};
  std::atomic<int> read_cycle_calls{0};
  std::atomic<int> read_individual_calls{0};
  std::atomic<int> write_echo_calls{0};
  std::atomic<int> fire_and_forget_calls{0};
  std::atomic<bool> verify_response_mode{true};
  std::atomic<bool> verify_bulk_sensor_mode{true};
  std::atomic<uint8_t> joint_io_mode{0xFF};
  HandReadPlan last_plan{};
  HandMotorArray last_echo_pos{};
  HandMotorArray last_echo_ff{};
  HandMotorArray last_faf_pos{};
  HandMotorArray last_faf_ff{};
  // Order of the RT building blocks within one process lifetime: 'W' =
  // WriteCommandWithEcho, 'R' = ReadCycle (test-only bookkeeping, not RT).
  std::string call_order;

  [[nodiscard]] bool Open() noexcept override {
    ++open_calls;
    return true;
  }

  void Close() noexcept override { ++close_calls; }

  [[nodiscard]] bool InitializeSensors() noexcept override {
    ++init_sensor_calls;
    return true;
  }

  void set_verify_response_mode(bool v) noexcept override { verify_response_mode = v; }

  void set_verify_bulk_sensor_mode(bool v) noexcept override { verify_bulk_sensor_mode = v; }

  void set_joint_io_mode(packets::JointMode m) noexcept override {
    joint_io_mode = static_cast<uint8_t>(m);
  }

  void ResetCommStats() noexcept override {
    ++reset_stats_calls;
    stats_ = {};
    stats_seqlock_.Store(stats_);
  }

  [[nodiscard]] UdpHandCommStats comm_stats() const noexcept override {
    return stats_seqlock_.Load();
  }

  [[nodiscard]] uint64_t recv_error_count() const noexcept override { return 0; }

  [[nodiscard]] double recv_timeout_ms() const noexcept override { return 1.0; }

  [[nodiscard]] bool SupportsIndividualMode() const noexcept override { return true; }

  int DrainStale() noexcept override {
    ++drain_calls;
    return 0;
  }

  [[nodiscard]] bool WriteCommandWithEcho(const HandMotorArray& pos,
                                          const HandMotorArray& ff_nm) noexcept override {
    last_echo_pos = pos;
    last_echo_ff = ff_nm;
    ++write_echo_calls;
    call_order.push_back('W');
    return script_write_ok;
  }

  void WriteCommandFireAndForget(const HandMotorArray& pos,
                                 const HandMotorArray& ff_nm) noexcept override {
    last_faf_pos = pos;
    last_faf_ff = ff_nm;
    ++fire_and_forget_calls;
  }

  void ReadCycle(const HandReadPlan& plan, HandReadBuffers& bufs,
                 HandReadOutcome& out) noexcept override {
    last_plan = plan;
    ++read_cycle_calls;
    call_order.push_back('R');
    FillMotorPattern(bufs);
    out = HandReadOutcome{};
    out.motor_ok = plan.motor_space_read && script_motor_ok;
    out.StampPhase(ReadPhase::kMotor);
    out.joint_ok = script_joint_ok;
    out.StampPhase(ReadPhase::kJoint);
    out.sensor_recvd = plan.sensors ? script_sensor_recvd : -1;
    out.received_motor_mode = packets::JointMode::kMotor;
  }

  void ReadCycleIndividual(const HandReadPlan& plan, HandReadBuffers& bufs,
                           HandReadOutcome& out) noexcept override {
    last_plan = plan;
    ++read_individual_calls;
    FillMotorPattern(bufs);
    out = HandReadOutcome{};
    out.motor_ok = script_motor_ok;
    out.StampPhase(ReadPhase::kMotor);
    out.joint_ok = script_joint_ok;
    out.StampPhase(ReadPhase::kJoint);
    out.vel_ok = script_vel_ok;
    out.StampPhase(ReadPhase::kVel);
    out.received_motor_mode = packets::JointMode::kMotor;
    if (plan.sensors) {
      out.finger_ok_mask = script_finger_mask;
      for (int f = 0; f < plan.num_fingertips && f < kMaxFingertips; ++f) {
        for (int k = 0; k < kSensorValuesPerFingertip; ++k) {
          bufs.sensor_values[static_cast<std::size_t>(FingerOffset(f) + k)] = 1000 * (f + 1) + k;
        }
      }
    }
  }

  [[nodiscard]] UdpHandCommStats& comm_stats_mut() noexcept override { return stats_; }

  void PublishCommStats() noexcept override { stats_seqlock_.Store(stats_); }

  [[nodiscard]] uint8_t last_unexpected_cmd() const noexcept override { return 0; }

  [[nodiscard]] uint16_t last_unexpected_len() const noexcept override { return 0; }

 protected:
  void FillMotorPattern(HandReadBuffers& bufs) const noexcept {
    for (std::size_t i = 0; i < packets::kMotorDataCount; ++i) {
      const auto fi = static_cast<float>(i);
      bufs.motor.pos[i] = pattern_base + fi;
      bufs.motor.vel[i] = pattern_base + 10.0f + fi;
      bufs.motor.cur[i] = pattern_base + 20.0f + fi;
      bufs.joint.pos[i] = pattern_base + 100.0f + fi;
      bufs.joint.vel[i] = pattern_base + 110.0f + fi;
      bufs.joint.cur[i] = pattern_base + 120.0f + fi;
    }
  }

  UdpHandCommStats stats_{};
  rtc::SeqLock<UdpHandCommStats> stats_seqlock_{};
};

// Real-mode controller (sockets never opened — the scripted comm answers) with
// the comm injected; the raw pointer observes it while the controller owns it.
struct ScriptedRig {
  std::unique_ptr<UdpHandController> ctrl;
  ScriptedHandComm* comm{nullptr};
};

ScriptedRig MakeScriptedRig(HandCommunicationMode mode, int num_fingertips) {
  ScriptedRig rig;
  rig.ctrl = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = 1,
      .recv_timeout_ms = 1.0,
      .num_fingertips = num_fingertips,
      .communication_mode = mode,
  });
  auto comm = std::make_unique<ScriptedHandComm>();
  rig.comm = comm.get();
  EXPECT_TRUE(rig.ctrl->SetHandComm(std::move(comm)));
  return rig;
}

TEST(HandCommSeam, DefaultCommIsUdp_NullOrRunningInjectionRefused) {
  auto ctrl = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = 1,
      .use_fake_hand = true,
  });
  EXPECT_NE(dynamic_cast<const UdpHandComm*>(&ctrl->hand_comm()), nullptr);
  // A null comm (a factory that failed) is refused, not silently replaced by UDP.
  EXPECT_FALSE(ctrl->SetHandComm(nullptr));
  EXPECT_NE(dynamic_cast<const UdpHandComm*>(&ctrl->hand_comm()), nullptr);
  // While running the swap would free the comm under the CommLoop: refused.
  ASSERT_TRUE(ctrl->Start());
  auto late = std::make_unique<ScriptedHandComm>();
  const ScriptedHandComm* late_ptr = late.get();
  EXPECT_FALSE(ctrl->SetHandComm(std::move(late)));
  EXPECT_NE(&ctrl->hand_comm(), static_cast<const HandComm*>(late_ptr));
  ctrl->Stop();
}

// D16: a link without the individual read sequence must be refused at Start(),
// before the CommLoop launches — not run the unsupported default every tick.
TEST(HandCommSeam, Individual_RefusedByCommWithoutIndividualSupport) {
  class BulkOnlyComm final : public ScriptedHandComm {
   public:
    [[nodiscard]] bool SupportsIndividualMode() const noexcept override { return false; }
  };

  auto ctrl = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = 1,
      .num_fingertips = 0,
      .communication_mode = HandCommunicationMode::kIndividual,
  });
  auto comm = std::make_unique<BulkOnlyComm>();
  const BulkOnlyComm* probe = comm.get();
  ASSERT_TRUE(ctrl->SetHandComm(std::move(comm)));
  EXPECT_FALSE(ctrl->Start());
  EXPECT_FALSE(ctrl->IsRunning());
  EXPECT_EQ(probe->open_calls, 0);  // refused before the link is opened

  // The same comm starts fine in bulk mode.
  auto bulk = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = 1,
      .num_fingertips = 0,
      .communication_mode = HandCommunicationMode::kBulk,
  });
  ASSERT_TRUE(bulk->SetHandComm(std::make_unique<BulkOnlyComm>()));
  EXPECT_TRUE(bulk->Start());
  bulk->Stop();
}

TEST(HandCommSeam, Bulk_PlanHoistedFromProtocol_OutcomeMapsToStateMasksAndStats) {
  auto rig = MakeScriptedRig(HandCommunicationMode::kBulk, /*num_fingertips=*/4);
  auto& comm = *rig.comm;
  comm.script_sensor_recvd = -1;  // sensor channel fails every cycle

  ASSERT_TRUE(rig.ctrl->Start());
  ASSERT_TRUE(PollUntil([&] { return rig.ctrl->cycle_count() >= 3; }, 2s));
  rig.ctrl->Stop();

  // Non-RT setup went through the seam, in PrepareRun order.
  EXPECT_EQ(comm.open_calls, 1);
  EXPECT_EQ(comm.init_sensor_calls, 1);
  EXPECT_EQ(comm.reset_stats_calls, 1);
  EXPECT_EQ(comm.close_calls, 1);
  // 1a default protocol: strict MODE gates, kJoint I/O, motor-space read, 259 B.
  EXPECT_TRUE(comm.verify_response_mode);
  EXPECT_TRUE(comm.verify_bulk_sensor_mode);
  EXPECT_EQ(comm.joint_io_mode, static_cast<uint8_t>(packets::JointMode::kJoint));
  EXPECT_TRUE(comm.last_plan.motor_space_read);
  EXPECT_TRUE(comm.last_plan.sensors);  // sensor_decimation 1: every cycle
  EXPECT_EQ(comm.last_plan.num_fingertips, 4);
  EXPECT_EQ(comm.last_plan.sensor_response_size, packets::kAllSensorResponseSize);
  EXPECT_GT(comm.drain_calls, 0);
  EXPECT_GT(comm.read_cycle_calls, 0);
  EXPECT_EQ(comm.read_individual_calls, 0);

  // Outcome → published state.
  const UdpHandState st = rig.ctrl->GetLatestState();
  EXPECT_TRUE(st.valid);
  EXPECT_TRUE(st.motor_valid);
  EXPECT_TRUE(st.joint_valid);
  EXPECT_FLOAT_EQ(st.motor_positions[2], 3.0f);
  EXPECT_FLOAT_EQ(st.motor_velocities[2], 13.0f);
  EXPECT_FLOAT_EQ(st.motor_currents[2], 23.0f);
  EXPECT_FLOAT_EQ(st.joint_positions[2], 103.0f);
  EXPECT_FLOAT_EQ(st.joint_velocities[2], 113.0f);
  EXPECT_FLOAT_EQ(st.joint_currents[2], 123.0f);
  EXPECT_EQ(st.received_joint_mode, static_cast<uint8_t>(packets::JointMode::kMotor));
  EXPECT_EQ(st.sensor_seq, 0u);  // no validated sensor read ever

  // Outcome → masks / streaks: sensor attempted and failed, reads ok.
  const CycleOutcomeRing ring = rig.ctrl->GetCycleOutcomeRing();
  ASSERT_GT(ring.count, 0u);
  const auto& latest = ring.entries[(ring.count - 1) % CycleOutcomeRing::kCapacity];
  EXPECT_EQ(latest.attempted_mask, RequestKindBit(RequestKind::kMotorRead) |
                                       RequestKindBit(RequestKind::kJointRead) |
                                       RequestKindBit(RequestKind::kBulkSensor));
  EXPECT_EQ(latest.ok_mask,
            RequestKindBit(RequestKind::kMotorRead) | RequestKindBit(RequestKind::kJointRead));
  EXPECT_GT(rig.ctrl->channel_consec_fail(RequestKind::kBulkSensor), 0u);
  EXPECT_EQ(rig.ctrl->channel_consec_fail(RequestKind::kJointRead), 0u);
  EXPECT_EQ(rig.ctrl->consecutive_recv_failures(), 0u);  // some channel answered

  // Stats stamping goes through comm_stats_mut()/PublishCommStats().
  EXPECT_EQ(rig.ctrl->comm_stats().total_cycles, rig.ctrl->cycle_count());
}

TEST(HandCommSeam, Write_GatedOnFirstState_ForwardsCommandWithZeroFeedforward) {
  auto rig = MakeScriptedRig(HandCommunicationMode::kBulk, /*num_fingertips=*/0);
  auto& comm = *rig.comm;

  ASSERT_TRUE(rig.ctrl->Start());
  ASSERT_TRUE(PollUntil([&] { return rig.ctrl->HasStateBeenRead(); }, 2s));
  HandMotorArray cmd{};
  for (std::size_t i = 0; i < kNumHandMotors; ++i) {
    cmd[i] = 0.25f * static_cast<float>(i);
  }
  rig.ctrl->SendCommandAndRequestStates(cmd);
  ASSERT_TRUE(PollUntil([&] { return comm.write_echo_calls >= 1; }, 2s));
  rig.ctrl->Stop();

  EXPECT_EQ(comm.write_echo_calls, 1);  // per-command gate: one write per staged command
  EXPECT_EQ(comm.last_echo_pos, cmd);
  EXPECT_EQ(comm.last_echo_ff, HandMotorArray{});  // the 1-arg overload stages a zero feedforward
  EXPECT_EQ(comm.fire_and_forget_calls, 0);
}

// D39: the 2-arg overload stages position and feedforward as ONE payload, so
// the write that latches the command hands the comm both of that same command.
TEST(HandCommSeam, Write_ForwardsTheFeedforwardStagedWithTheCommand) {
  auto rig = MakeScriptedRig(HandCommunicationMode::kBulk, /*num_fingertips=*/0);
  auto& comm = *rig.comm;

  ASSERT_TRUE(rig.ctrl->Start());
  ASSERT_TRUE(PollUntil([&] { return rig.ctrl->HasStateBeenRead(); }, 2s));
  HandMotorArray cmd{};
  HandMotorArray ff{};
  for (std::size_t i = 0; i < kNumHandMotors; ++i) {
    cmd[i] = 0.25f * static_cast<float>(i);
    ff[i] = -0.5f + 0.1f * static_cast<float>(i);
  }
  rig.ctrl->SendCommandAndRequestStates(cmd, ff);
  ASSERT_TRUE(PollUntil([&] { return comm.write_echo_calls >= 1; }, 2s));
  rig.ctrl->Stop();

  EXPECT_EQ(comm.write_echo_calls, 1);
  EXPECT_EQ(comm.last_echo_pos, cmd);
  EXPECT_EQ(comm.last_echo_ff, ff);
  EXPECT_EQ(comm.fire_and_forget_calls, 0);
}

// E-8 regression pin for the seam: the terminal E-Stop zero-write goes through
// HandComm::WriteCommandFireAndForget exactly once, with zero position AND zero
// feedforward, before any cycle I/O (the E-Stop branch precedes the drain).
TEST(HandCommSeam, EStop_FireAndForgetOnce_ZeroPositionAndZeroFeedforward) {
  auto rig = MakeScriptedRig(HandCommunicationMode::kBulk, /*num_fingertips=*/0);
  auto& comm = *rig.comm;
  std::atomic<bool> estop{true};  // asserted before Start → first tick self-exits
  rig.ctrl->SetEstopFlag(&estop);

  ASSERT_TRUE(rig.ctrl->Start());
  ASSERT_TRUE(PollUntil([&] { return !rig.ctrl->IsRunning(); }, 2s));
  rig.ctrl->Stop();  // joins the loop → the captures below are stable

  EXPECT_EQ(comm.fire_and_forget_calls, 1);
  EXPECT_EQ(comm.last_faf_pos, HandMotorArray{});
  EXPECT_EQ(comm.last_faf_ff, HandMotorArray{});
  EXPECT_EQ(comm.write_echo_calls, 0);
  EXPECT_EQ(comm.read_cycle_calls, 0);
  EXPECT_EQ(comm.drain_calls, 0);
  EXPECT_EQ(rig.ctrl->cycle_count(), 0u);
}

TEST(HandCommSeam, Individual_UsesIndividualCycle_PartialFingersRefreshOnlyTheirSlots) {
  auto rig = MakeScriptedRig(HandCommunicationMode::kIndividual, /*num_fingertips=*/2);
  auto& comm = *rig.comm;
  comm.script_vel_ok = false;      // 0x12 fails every cycle
  comm.script_finger_mask = 0x01;  // finger 0 answers, finger 1 times out

  ASSERT_TRUE(rig.ctrl->Start());
  ASSERT_TRUE(PollUntil([&] { return rig.ctrl->cycle_count() >= 3; }, 2s));
  rig.ctrl->Stop();

  EXPECT_GT(comm.read_individual_calls, 0);
  EXPECT_EQ(comm.read_cycle_calls, 0);
  EXPECT_EQ(comm.last_plan.num_fingertips, 2);

  const UdpHandState st = rig.ctrl->GetLatestState();
  EXPECT_TRUE(st.motor_valid);
  EXPECT_TRUE(st.joint_valid);
  EXPECT_FLOAT_EQ(st.motor_positions[4], 5.0f);
  EXPECT_FLOAT_EQ(st.motor_velocities[4], 0.0f);  // vel channel failed: never copied
  EXPECT_FLOAT_EQ(st.joint_positions[4], 105.0f);
  // Finger 0's slot refreshed from the comm; finger 1's slot never written.
  EXPECT_EQ(st.sensor_data[0], 1000);
  EXPECT_EQ(st.sensor_data[kSensorValuesPerFingertip - 1], 1000 + kSensorValuesPerFingertip - 1);
  EXPECT_EQ(st.sensor_data[kSensorValuesPerFingertip], 0);
  EXPECT_GE(st.sensor_seq, 1u);  // ≥1 fresh finger per cycle bumps the freshness seq

  const CycleOutcomeRing ring = rig.ctrl->GetCycleOutcomeRing();
  ASSERT_GT(ring.count, 0u);
  const auto& latest = ring.entries[(ring.count - 1) % CycleOutcomeRing::kCapacity];
  EXPECT_EQ(latest.attempted_mask, RequestKindBit(RequestKind::kMotorRead) |
                                       RequestKindBit(RequestKind::kJointRead) |
                                       RequestKindBit(RequestKind::kSensorRead));
  // pos ok sets the motor bit even though vel failed; one ok finger sets sensor.
  EXPECT_EQ(latest.ok_mask, latest.attempted_mask);
}

// D60: a comm that owns the whole bulk cycle transaction. Records every
// RunCycle call (whether a command came with it, and which) and answers from
// the script; the building blocks are never called, so any direct
// WriteCommandWithEcho / ReadCycle from the controller shows up in the
// ScriptedHandComm counters.
class TxnHandComm final : public ScriptedHandComm {
 public:
  std::atomic<int> run_cycle_calls{0};
  std::atomic<int> run_cycle_with_cmd{0};
  std::atomic<int> run_cycle_half_cmd{0};  // exactly one of the two pointers null
  HandMotorArray last_cmd_pos{};
  HandMotorArray last_cmd_ff{};
  // write_end = call time + this offset (a stage that "took" write_stage_us).
  std::atomic<int> write_stage_us{0};
  // write_rx = write_ok && this (false: a link that confirms a write by its
  // send, CAN FD cmd_ack none — D68). Atomic: flipped while the loop runs.
  std::atomic<bool> script_write_rx{true};
  // Every read of every cycle fails while set (atomic, like script_write_rx).
  std::atomic<bool> script_reads_fail{false};

  void RunCycle(const HandReadPlan& plan, const HandMotorArray* cmd_pos,
                const HandMotorArray* cmd_ff, HandReadBuffers& bufs,
                HandReadOutcome& out) noexcept override {
    const auto t_call = HandReadOutcome::Clock::now();
    ++run_cycle_calls;
    if ((cmd_pos == nullptr) != (cmd_ff == nullptr))
      ++run_cycle_half_cmd;
    last_plan = plan;
    FillMotorPattern(bufs);
    out = HandReadOutcome{};
    if (cmd_pos != nullptr && cmd_ff != nullptr) {
      ++run_cycle_with_cmd;
      last_cmd_pos = *cmd_pos;
      last_cmd_ff = *cmd_ff;
      out.write_ok = script_write_ok;
      out.write_rx = script_write_ok && script_write_rx.load();
      out.write_end = t_call + std::chrono::microseconds(write_stage_us.load());
    }
    const bool reads_ok = !script_reads_fail.load();
    out.motor_ok = plan.motor_space_read && script_motor_ok && reads_ok;
    out.StampPhase(ReadPhase::kMotor);
    out.joint_ok = script_joint_ok && reads_ok;
    out.StampPhase(ReadPhase::kJoint);
    out.sensor_recvd = plan.sensors ? script_sensor_recvd : -1;
    out.received_motor_mode = packets::JointMode::kMotor;
  }
};

struct TxnRig {
  std::unique_ptr<UdpHandController> ctrl;
  TxnHandComm* comm{nullptr};
};

TxnRig MakeTxnRig() {
  TxnRig rig;
  rig.ctrl = std::make_unique<UdpHandController>(UdpHandControllerConfig{
      .target_ip = "127.0.0.1",
      .target_port = 1,
      .recv_timeout_ms = 1.0,
      .num_fingertips = 0,
      .communication_mode = HandCommunicationMode::kBulk,
  });
  auto comm = std::make_unique<TxnHandComm>();
  rig.comm = comm.get();
  EXPECT_TRUE(rig.ctrl->SetHandComm(std::move(comm)));
  return rig;
}

// D60: the bulk tick issues exactly one RunCycle and never the building
// blocks directly; a tick without a pending command hands in no command, the
// tick that latches one hands in that command (position AND feedforward) once.
TEST(HandCommSeam, RunCycle_OneTransactionPerBulkTick_CommandHandedInOnce) {
  auto rig = MakeTxnRig();
  auto& comm = *rig.comm;

  ASSERT_TRUE(rig.ctrl->Start());
  ASSERT_TRUE(PollUntil([&] { return rig.ctrl->cycle_count() >= 3; }, 2s));
  EXPECT_EQ(comm.run_cycle_with_cmd, 0);  // no command staged yet: nullptr every tick
  HandMotorArray cmd{};
  HandMotorArray ff{};
  for (std::size_t i = 0; i < kNumHandMotors; ++i) {
    cmd[i] = 0.25f * static_cast<float>(i);
    ff[i] = 1.0f - 0.1f * static_cast<float>(i);
  }
  rig.ctrl->SendCommandAndRequestStates(cmd, ff);
  ASSERT_TRUE(PollUntil([&] { return comm.run_cycle_with_cmd >= 1; }, 2s));
  ASSERT_TRUE(PollUntil([&] { return rig.ctrl->cycle_count() >= 8; }, 2s));
  rig.ctrl->Stop();

  EXPECT_EQ(static_cast<std::size_t>(comm.run_cycle_calls.load()), rig.ctrl->cycle_count());
  EXPECT_EQ(comm.run_cycle_with_cmd, 1);  // per-command gate: once per staged command
  EXPECT_EQ(comm.run_cycle_half_cmd, 0);
  EXPECT_EQ(comm.last_cmd_pos, cmd);
  EXPECT_EQ(comm.last_cmd_ff, ff);
  EXPECT_EQ(comm.write_echo_calls, 0);  // the building blocks are the comm's business
  EXPECT_EQ(comm.read_cycle_calls, 0);
  EXPECT_EQ(comm.fire_and_forget_calls, 0);
  EXPECT_EQ(rig.ctrl->channel_consec_fail(RequestKind::kWriteEcho), 0u);
}

// D60: the controller maps write_ok into the cycle masks / streaks and clears
// its pending flag whether or not the write validated — a failed write is not
// re-sent (one RunCycle with a command, then none).
TEST(HandCommSeam, RunCycle_FailedWrite_LeavesOkMask_ClearsPending_NotResent) {
  auto rig = MakeTxnRig();
  auto& comm = *rig.comm;
  comm.script_write_ok = false;

  ASSERT_TRUE(rig.ctrl->Start());
  ASSERT_TRUE(PollUntil([&] { return rig.ctrl->HasStateBeenRead(); }, 2s));
  rig.ctrl->SendCommandAndRequestStates(HandMotorArray{});
  ASSERT_TRUE(PollUntil([&] { return comm.run_cycle_with_cmd >= 1; }, 2s));
  const std::size_t cycles_at_write = rig.ctrl->cycle_count();
  ASSERT_TRUE(PollUntil([&] { return rig.ctrl->cycle_count() >= cycles_at_write + 4; }, 2s));
  rig.ctrl->Stop();

  EXPECT_EQ(comm.run_cycle_with_cmd, 1);
  // The write-echo streak counts only the attempted tick (1), the reads were ok.
  EXPECT_EQ(rig.ctrl->channel_consec_fail(RequestKind::kWriteEcho), 1u);
  EXPECT_EQ(rig.ctrl->channel_consec_fail(RequestKind::kJointRead), 0u);
  bool saw_failed_write = false;
  const CycleOutcomeRing ring = rig.ctrl->GetCycleOutcomeRing();
  const std::size_t n = std::min<std::size_t>(ring.count, CycleOutcomeRing::kCapacity);
  for (std::size_t i = 0; i < n; ++i) {
    const auto& e = ring.entries[i];
    if ((e.attempted_mask & RequestKindBit(RequestKind::kWriteEcho)) != 0) {
      saw_failed_write = true;
      EXPECT_EQ(e.ok_mask & RequestKindBit(RequestKind::kWriteEcho), 0u);
    }
  }
  EXPECT_TRUE(saw_failed_write);
}

// D68: a write the comm confirmed by its send alone (write_ok, !write_rx —
// CAN FD cmd_ack none) still sets the write-echo ok bit, but is not a receive:
// with every read failing, the write tick does not reset the receive-failure
// streak. Control: the same write with write_rx (an echo) does reset it.
namespace {
// Reads fail from c0 on; one command lands >= 3 cycles later. Returns the
// streak minus the cycles counted since c0 (>= -1 iff the streak never reset).
int64_t StreakSinceReadsFailed(bool write_rx) {
  auto rig = MakeTxnRig();
  auto& comm = *rig.comm;
  comm.script_write_rx = write_rx;
  EXPECT_TRUE(rig.ctrl->Start());
  EXPECT_TRUE(PollUntil([&] { return rig.ctrl->HasStateBeenRead(); }, 2s));
  comm.script_reads_fail = true;
  const auto c0 = static_cast<int64_t>(rig.ctrl->cycle_count());
  EXPECT_TRUE(
      PollUntil([&] { return static_cast<int64_t>(rig.ctrl->cycle_count()) >= c0 + 3; }, 2s));
  rig.ctrl->SendCommandAndRequestStates(HandMotorArray{});
  EXPECT_TRUE(PollUntil([&] { return comm.run_cycle_with_cmd >= 1; }, 2s));
  const auto cw = static_cast<int64_t>(rig.ctrl->cycle_count());
  EXPECT_TRUE(
      PollUntil([&] { return static_cast<int64_t>(rig.ctrl->cycle_count()) >= cw + 3; }, 2s));
  rig.ctrl->Stop();
  const auto c1 = static_cast<int64_t>(rig.ctrl->cycle_count());
  // The write tick still reports the write as ok in its outcome.
  bool saw_ok_write = false;
  const CycleOutcomeRing ring = rig.ctrl->GetCycleOutcomeRing();
  const std::size_t n = std::min<std::size_t>(ring.count, CycleOutcomeRing::kCapacity);
  for (std::size_t i = 0; i < n; ++i) {
    const auto& e = ring.entries[i];
    if ((e.attempted_mask & RequestKindBit(RequestKind::kWriteEcho)) != 0)
      saw_ok_write = (e.ok_mask & RequestKindBit(RequestKind::kWriteEcho)) != 0;
  }
  EXPECT_TRUE(saw_ok_write);
  EXPECT_FALSE(rig.ctrl->GetLatestState().valid);
  return static_cast<int64_t>(rig.ctrl->consecutive_recv_failures()) - (c1 - c0);
}
}  // namespace

TEST(HandCommSeam, RunCycle_WriteRxFalse_DoesNotCountAsRecv) {
  EXPECT_GE(StreakSinceReadsFailed(/*write_rx=*/false), -1)
      << "the send-confirmed write reset the receive-failure streak";
}

TEST(HandCommSeam, RunCycle_WriteRxTrue_CountsAsRecv) {
  EXPECT_LT(StreakSinceReadsFailed(/*write_rx=*/true), -1)
      << "an echoed write is a receive and resets the streak";
}

// D60 / C1: write_us is the comm-stamped write stage (write_end − cycle
// start), not the controller's own clock around the call. A comm that reports
// a 3 ms stage puts >= 3000 us into the write phase while the read phases
// that follow it clamp to zero-length spans.
TEST(HandCommSeam, RunCycle_WriteUsIsWriteEndMinusCycleStart) {
  auto rig = MakeTxnRig();
  auto& comm = *rig.comm;
  comm.write_stage_us = 3000;

  ASSERT_TRUE(rig.ctrl->Start());
  ASSERT_TRUE(PollUntil([&] { return rig.ctrl->HasStateBeenRead(); }, 2s));
  rig.ctrl->SendCommandAndRequestStates(HandMotorArray{});
  ASSERT_TRUE(PollUntil([&] { return comm.run_cycle_with_cmd >= 1; }, 2s));
  ASSERT_TRUE(PollUntil([&] { return rig.ctrl->timing_stats().write.max_us > 0.0; }, 2s));
  rig.ctrl->Stop();

  // The one write tick is the only write sample (min == max); its span is the
  // comm's 3 ms stage plus the controller's own overhead, never a whole tick
  // of the mock's own wall time (the call itself returns at once).
  const auto ts = rig.ctrl->timing_stats();
  EXPECT_GE(ts.write.min_us, 3000.0);
  EXPECT_EQ(ts.write.min_us, ts.write.max_us);
  EXPECT_LT(ts.write.max_us, 3000.0 + 1000.0);
}

// E-8 / D60: the terminal E-Stop tick issues no cycle transaction at all —
// the fire-and-forget zero-write is the only I/O (pin beside the
// ScriptedHandComm version above, now for the RunCycle path).
TEST(HandCommSeam, EStop_NoRunCycle_FireAndForgetOnce) {
  auto rig = MakeTxnRig();
  auto& comm = *rig.comm;
  std::atomic<bool> estop{true};
  rig.ctrl->SetEstopFlag(&estop);

  ASSERT_TRUE(rig.ctrl->Start());
  ASSERT_TRUE(PollUntil([&] { return !rig.ctrl->IsRunning(); }, 2s));
  rig.ctrl->Stop();

  EXPECT_EQ(comm.run_cycle_calls, 0);
  EXPECT_EQ(comm.fire_and_forget_calls, 1);
  EXPECT_EQ(comm.last_faf_pos, HandMotorArray{});
  EXPECT_EQ(comm.last_faf_ff, HandMotorArray{});
  EXPECT_EQ(comm.write_echo_calls, 0);
  EXPECT_EQ(comm.read_cycle_calls, 0);
}

// D60 default implementation (what UdpHandComm and every mock without an
// override run): WriteCommandWithEcho first, ReadCycle second, the same
// position / feedforward handed through, and the write result surviving the
// reset ReadCycle performs on `out`. No command → no write, write fields at
// their defaults. A failed write still stamps write_end (the stage ran).
TEST(HandCommSeam, DefaultRunCycle_WriteThenRead_SamePayload_WriteResultSurvivesReset) {
  ScriptedHandComm comm;
  HandReadPlan plan{};
  plan.motor_space_read = true;
  HandReadBuffers bufs{};
  HandReadOutcome out{};
  HandMotorArray pos{};
  HandMotorArray ff{};
  for (std::size_t i = 0; i < kNumHandMotors; ++i) {
    pos[i] = 0.1f * static_cast<float>(i);
    ff[i] = 2.0f + static_cast<float>(i);
  }

  const auto t_before = HandReadOutcome::Clock::now();
  comm.RunCycle(plan, &pos, &ff, bufs, out);
  EXPECT_EQ(comm.call_order, "WR");
  EXPECT_EQ(comm.last_echo_pos, pos);
  EXPECT_EQ(comm.last_echo_ff, ff);
  EXPECT_TRUE(out.write_ok);
  EXPECT_TRUE(out.write_rx);  // the default is an echoing link: write_rx = write_ok (D68)
  EXPECT_GE(out.write_end, t_before);
  EXPECT_LE(out.write_end, out.PhaseEnd(ReadPhase::kMotor));  // write stage closed before the reads
  EXPECT_TRUE(out.motor_ok);
  EXPECT_TRUE(out.joint_ok);

  out.write_ok = true;  // stale content must not leak into a no-command cycle
  out.write_rx = true;
  comm.RunCycle(plan, nullptr, nullptr, bufs, out);
  EXPECT_EQ(comm.call_order, "WRR");
  EXPECT_EQ(comm.write_echo_calls, 1);
  EXPECT_FALSE(out.write_ok);
  EXPECT_FALSE(out.write_rx);
  EXPECT_EQ(out.write_end, HandReadOutcome::Clock::time_point{});
  EXPECT_TRUE(out.joint_ok);

  comm.script_write_ok = false;
  comm.RunCycle(plan, &pos, &ff, bufs, out);
  EXPECT_EQ(comm.call_order, "WRRWR");
  EXPECT_FALSE(out.write_ok);
  EXPECT_FALSE(out.write_rx);
  EXPECT_NE(out.write_end, HandReadOutcome::Clock::time_point{});
  EXPECT_TRUE(out.joint_ok);  // the read still ran after the failed write
}

// A comm that leaves ReadCycleIndividual at the HandComm default (a non-UDP
// link): nothing read, every flag false, phases left at the epoch — which the
// controller clamps to zero-length spans. SupportsIndividualMode() is what
// keeps this body from ever running in production (see the test above).
TEST(HandCommSeam, DefaultReadCycleIndividual_NothingReadFlagsFalse) {
  class DefaultIndividualComm final : public ScriptedHandComm {
   public:
    void ReadCycleIndividual(const HandReadPlan& plan, HandReadBuffers& bufs,
                             HandReadOutcome& out) noexcept override {
      HandComm::ReadCycleIndividual(plan, bufs, out);
    }
  };

  DefaultIndividualComm comm;
  HandReadPlan plan{};
  plan.sensors = true;
  plan.num_fingertips = 4;
  HandReadBuffers bufs{};
  HandReadOutcome out{};
  out.motor_ok = true;  // stale content must be cleared
  out.finger_ok_mask = 0xFF;
  comm.ReadCycleIndividual(plan, bufs, out);
  EXPECT_FALSE(out.motor_ok);
  EXPECT_FALSE(out.joint_ok);
  EXPECT_FALSE(out.vel_ok);
  EXPECT_EQ(out.finger_ok_mask, 0u);
  EXPECT_EQ(out.sensor_recvd, -1);
  for (const auto& t : out.phase_end) {
    EXPECT_EQ(t, HandReadOutcome::Clock::time_point{});
  }
}

}  // namespace udp_hand_driver::test
