// ── LocomotionController — the MPC lock-step (mpc.sync.mode: lockstep) ──────
//
// The RT tick requests a solve every N ticks (the MPC period in control
// ticks) and waits for it L ticks later, so every plan is taken at exactly L
// ticks of age however long the solve took; the simulator's real-time factor
// follows (#1168). Pinned here, on the stub handler (locomotion_mpc_stub.hpp)
// through the rig's immediate ticks:
//   * the mode is refused without a sim-synced controller manager
//     (cm_sim_time_sync, fail-closed) and the sync keys are checked;
//   * a new plan's first row has policy_age_us == L · tick exactly, triggers
//     are N ticks apart, a slow solve changes the wait, not the timing;
//   * a wait past wait_timeout_s latches the fault mpc_wait;
//   * a deactivation releases a tick waiting on the channel within a few ms;
//   * a tick whose state source failed on a grid tick skips the trigger and
//     keeps the grid;
//   * the control: free_run plans do not arrive at that age.

#include "locomotion_mpc_stub.hpp"

#include <geometry_msgs/msg/twist.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace loco_rig;  // NOLINT(google-build-using-namespace) — the shared rig
using integrated_bringup::LocomotionDiagLogPod;
using Source = LocomotionController::ReferenceSource;
using Cause = LocomotionDiagLogPod::FaultCause;
using namespace std::chrono_literals;

constexpr std::int64_t kTickUs = 2000;  // 500 Hz

YAML::Node LockstepConfig(double wait_timeout_s = 2.0, int delivery_ticks = 0) {
  YAML::Node cfg = MpcConfig();
  cfg["mpc"]["sync"]["mode"] = "lockstep";
  cfg["mpc"]["sync"]["delivery_ticks"] = delivery_ticks;
  cfg["mpc"]["sync"]["wait_timeout_s"] = wait_timeout_s;
  return cfg;
}

/// A node as a sim-synced controller manager builds it: the fact declared true.
rclcpp::NodeOptions SimSyncedNode() {
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  opts.parameter_overrides({rclcpp::Parameter("cm_sim_time_sync", true)});
  return opts;
}

struct Row {
  std::uint64_t tick{0};
  std::uint64_t plan_id{0};
  double policy_age_us{0.0};
  std::uint64_t trigger_tick{0};
  double wait_us{0.0};
};

class LocomotionMpcLockstepTest : public LocomotionControllerTest {
 protected:
  void BringUpLockstep(YAML::Node cfg = LockstepConfig()) {
    Configure(cfg, SimSyncedNode(), "test_locomotion_mpc_lockstep");
    ASSERT_TRUE(ctrl_->MpcEnabled());
    ASSERT_TRUE(ctrl_->MpcLockstep());
    stub_ = SpawnStandingStub(*ctrl_);
    ASSERT_NE(stub_, nullptr);
    Activate();
    MakeStanding();
    n_ = ctrl_->MpcGridTicks();
    l_ = ctrl_->MpcDeliveryTicks();
    ASSERT_EQ(n_, 35) << "2 nodes of 35 ms at 500 Hz";
  }

  Row Record() {
    static_cast<void>(Tick());
    const auto& d = ctrl_->LastDiag();
    return Row{d.tick, d.plan_id, d.policy_age_us, d.mpc_trigger_tick, d.mpc_wait_us};
  }

  /// @p count rows; the plan active before the first is remembered for
  /// ExpectLockstepInvariants.
  std::vector<Row> Ticks(int count) {
    plan_before_ = ctrl_->LastDiag().plan_id;
    std::vector<Row> rows;
    for (int i = 0; i < count; ++i) {
      rows.push_back(Record());
    }
    return rows;
  }

  /// Every plan that first appears in @p rows has the delivery age on its
  /// first row (the plan active before the recording is not judged: its first
  /// row is not in the recording), and the triggers are a period apart.
  void ExpectLockstepInvariants(const std::vector<Row>& rows) const {
    std::uint64_t last_plan = plan_before_;
    std::uint64_t last_trigger = 0;
    int plans = 0;
    for (const Row& r : rows) {
      if (r.plan_id != 0 && r.plan_id != last_plan) {
        EXPECT_EQ(r.policy_age_us, static_cast<double>(l_ * kTickUs)) << "tick " << r.tick;
        last_plan = r.plan_id;
        ++plans;
      }
      if (r.trigger_tick != 0 && r.trigger_tick != last_trigger) {
        if (last_trigger != 0) {
          EXPECT_EQ(r.trigger_tick - last_trigger, static_cast<std::uint64_t>(n_))
              << "tick " << r.tick;
        }
        last_trigger = r.trigger_tick;
      }
    }
    EXPECT_GE(plans, 2) << "too few plans to judge";
  }

  StandingStub* stub_{nullptr};
  std::int64_t n_{0};
  std::int64_t l_{0};
  std::uint64_t plan_before_{0};
};

// ── Configure ───────────────────────────────────────────────────────────────

TEST_F(LocomotionMpcLockstepTest, LockstepIsRefusedWithoutASimSyncedCm) {
  const YAML::Node cfg = LockstepConfig();
  rclcpp::NodeOptions opts;
  opts.use_global_arguments(false);
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_lockstep_refused", "", opts);
  ctrl_ = MakeController();
  ASSERT_EQ(ctrl_->PreConfigure(node_, cfg), CallbackReturn::SUCCESS) << "the schema is fine";
  ctrl_->SetDeviceNameConfigs(ShippedDevices(cfg));
  EXPECT_EQ(ctrl_->on_configure(rclcpp_lifecycle::State{}, node_, cfg), CallbackReturn::FAILURE)
      << "no cm_sim_time_sync: real hardware";
  EXPECT_FALSE(ctrl_->MpcChannel().Opened());
}

TEST_F(LocomotionMpcLockstepTest, FreeRunNeedsNoSimSyncedCm) {
  YAML::Node cfg = MpcConfig();
  cfg["mpc"]["sync"]["mode"] = "free_run";
  Configure(cfg);  // the rig's plain node
  EXPECT_FALSE(ctrl_->MpcLockstep());
  EXPECT_FALSE(ctrl_->MpcChannel().Opened());
}

TEST_F(LocomotionMpcLockstepTest, TheSyncKeysAreChecked) {
  const auto refused = [](const std::function<void(YAML::Node&)>& spoil, const char* what) {
    ExpectRefused(LockstepConfig(), spoil, what);
  };
  refused([](YAML::Node& c) { c["mpc"]["sync"]["mode"] = "hop"; }, "unknown mode");
  refused([](YAML::Node& c) { c["mpc"]["sync"]["bogus"] = 1; }, "unknown sync key");
  refused([](YAML::Node& c) { c["mpc"]["sync"].remove("mode"); }, "no mode");
  refused([](YAML::Node& c) { c["mpc"]["sync"]["delivery_ticks"] = -1; }, "negative delivery");
  refused([](YAML::Node& c) { c["mpc"]["sync"]["wait_timeout_s"] = 0.0; }, "zero wait");
  // The tick counts need the control rate: checked at the controller's Pass 1.
  const auto pass1_refused = [](const std::function<void(YAML::Node&)>& spoil, const char* what) {
    YAML::Node cfg = LockstepConfig();
    spoil(cfg);
    auto ctrl = MakeController();
    EXPECT_THROW(ctrl->LoadConfig(cfg), std::invalid_argument) << what;
  };
  pass1_refused([](YAML::Node& c) { c["mpc"]["sync"]["delivery_ticks"] = 36; },
                "delivery past the period (35 ticks)");
  pass1_refused([](YAML::Node& c) { c["mpc"]["period_nodes"] = 1; },
                "a period of 17.5 ticks is not whole");
  // Pass control: the shipped values parse, and L = 0 means the period.
  const auto parsed = ParseLocomotionControllerConfig(LockstepConfig());
  EXPECT_TRUE(parsed.mpc.lockstep);
  EXPECT_EQ(parsed.mpc.delivery_ticks, 0);
  EXPECT_EQ(parsed.mpc.wait_timeout_s, 2.0);
  EXPECT_FALSE(ParseLocomotionControllerConfig(MpcConfig()).mpc.lockstep)
      << "the shipped mode is free_run";
}

// ── The tick ────────────────────────────────────────────────────────────────

TEST_F(LocomotionMpcLockstepTest, APlanArrivesExactlyLTicksAfterItsStateAndTriggersAreNTicksApart) {
  BringUpLockstep();
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kStand; }));
  const std::vector<Row> rows = Ticks(static_cast<int>(4 * n_));
  ExpectLockstepInvariants(rows);
  EXPECT_EQ(ctrl_->LastDiag().fault_count, 0U);
  EXPECT_GE(ctrl_->LastDiag().mpc_wait_count, 4U);
  EXPECT_EQ(ctrl_->MpcChannel().IgnoredRequests(), 0U);
  EXPECT_EQ(ctrl_->MpcManager().SupersededPendingCount(), 0U);
}

TEST_F(LocomotionMpcLockstepTest, ASlowSolveChangesTheWaitNotTheTiming) {
  BringUpLockstep();
  stub_->sleep_us = 30000;  // 15 ticks of solve: the tick waits for it
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kStand; }));
  const std::vector<Row> rows = Ticks(static_cast<int>(3 * n_));
  ExpectLockstepInvariants(rows);
  double waited = 0.0;
  for (const Row& r : rows) {
    waited += r.wait_us;
  }
  EXPECT_GT(waited, 3 * 25000.0) << "the delivery ticks waited for the solve";
  EXPECT_EQ(ctrl_->LastDiag().fault_count, 0U);
  EXPECT_EQ(ctrl_->Source(), Source::kStand);
}

TEST_F(LocomotionMpcLockstepTest, AWaitPastTheTimeoutLatchesTheFault) {
  BringUpLockstep(LockstepConfig(/*wait_timeout_s=*/0.05));
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kStand; }));
  stub_->sleep_us = 300000;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kFault; }, 5s));
  EXPECT_EQ(static_cast<Cause>(ctrl_->LastDiag().fault_cause), Cause::kMpcWait);
  EXPECT_EQ(LastMode(), LocomotionDiagLogPod::Mode::kFault);
}

TEST_F(LocomotionMpcLockstepTest, ADeactivationReleasesATickWaitingOnTheSolve) {
  BringUpLockstep(LockstepConfig(/*wait_timeout_s=*/5.0));
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kStand; }));
  stub_->sleep_us = 600000;  // a solve the wait would sit on for 0.6 s
  std::atomic<bool> stop{false};
  std::thread ticker([this, &stop] {
    // Ticks until told to stop — the one inside the wait when the stop comes
    // is the last (no tick runs on the deactivated controller).
    for (std::int64_t i = 0; i < n_ + 2 && !stop.load(std::memory_order_acquire); ++i) {
      static_cast<void>(Tick());
    }
  });
  std::this_thread::sleep_for(100ms);  // the ticker is inside the wait by now
  stop.store(true, std::memory_order_release);
  const auto t0 = std::chrono::steady_clock::now();
  Deactivate();
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 500ms) << "the deactivation did not wait";
  ticker.join();
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 1s) << "the tick was released";
  EXPECT_EQ(ctrl_->LastDiag().fault_count, 0U) << "cancelled, not timed out";
  // The stub is still inside its 0.6 s solve; the fixture's teardown joins it.
}

TEST_F(LocomotionMpcLockstepTest, AStaleTickOnTheGridSkipsTheTriggerAndKeepsTheGrid) {
  BringUpLockstep();
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kStand; }));
  // Walk to the tick just before a grid tick.
  std::uint64_t trigger = ctrl_->LastDiag().mpc_trigger_tick;
  ASSERT_NE(trigger, 0U);
  while (ctrl_->LastDiag().tick + 1 < trigger + static_cast<std::uint64_t>(n_)) {
    static_cast<void>(Tick());
  }
  static_cast<void>(StaleTick());  // the grid tick with the source failed
  EXPECT_EQ(ctrl_->LastDiag().mpc_trigger_tick, trigger) << "no state to send: no trigger";
  const std::uint64_t skipped_grid = ctrl_->LastDiag().tick;
  // The next trigger is the next grid tick, not the next good tick.
  ASSERT_TRUE(TickUntil([this, trigger] { return ctrl_->LastDiag().mpc_trigger_tick != trigger; }));
  EXPECT_EQ(ctrl_->LastDiag().mpc_trigger_tick, skipped_grid + static_cast<std::uint64_t>(n_));
}

// ── The command filter on the tick ──────────────────────────────────────────

/// A Twist straight into the subscription's slot (as the executor would).
void SendTwist(LocomotionController& ctrl, double v_x) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = v_x;
  integrated_bringup::LocomotionControllerTestAccess::OnTwist(ctrl, twist);
}

// The filter steps every K = tick rate / sample_rate_hz ticks (5 at 500 Hz /
// 100 Hz) on the RT tick, its input's age on the tick clock: a command decays
// to rest after t_stale of TICKS without a message, however fast the ticks
// run on the wall clock (the free-run timer would still see it as fresh).
TEST_F(LocomotionMpcLockstepTest, TheCommandFilterStepsOnTheTickClock) {
  BringUpLockstep();
  ASSERT_TRUE(ctrl_->VelocityCommandSubscribed());
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kStand; }));
  const auto seq0 = ctrl_->LastCommand().sequence;
  const auto planner0 = stub_->horizon_command_sequence.load();
  // A publisher at 20 Hz of ticks (every 25 ticks) for a second of ticks —
  // well past the Bessel settling, never stale.
  const int kTicks = 500;
  for (int i = 0; i < kTicks; ++i) {
    if (i % 25 == 0) {
      SendTwist(*ctrl_, 0.3);
    }
    static_cast<void>(Tick());
  }
  const auto cmd = ctrl_->LastCommand();
  EXPECT_EQ(cmd.sequence - seq0, static_cast<std::uint64_t>(kTicks / 5)) << "one step per 5 ticks";
  EXPECT_TRUE(cmd.valid);
  EXPECT_TRUE(cmd.fresh);
  EXPECT_NEAR(cmd.v_x, 0.3, 0.02) << "the Bessel stage settled on the command";
  EXPECT_EQ(cmd.nonfinite_count, 0U);
  // The planner read the filter's output on the trigger ticks: the command
  // sequence it carries is a step's, the latest one no later than the last
  // trigger (a free-run timer would hand it any step's).
  const auto planner_seq = stub_->horizon_command_sequence.load();
  EXPECT_GT(planner_seq, planner0);
  EXPECT_LE(planner_seq, cmd.sequence);
  EXPECT_GE(planner_seq + static_cast<std::uint64_t>(n_ / 5) + 1, cmd.sequence)
      << "no older than the last trigger's step";
  EXPECT_NEAR(stub_->horizon_base_vx.load(), cmd.v_x, 0.02);

  // No message for t_stale (0.5 s) of ticks, then the Bessel decay (another
  // half second of ticks) — a few wall milliseconds in all.
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < 700; ++i) {
    static_cast<void>(Tick());
  }
  const auto wall_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
          .count();
  const auto stale = ctrl_->LastCommand();
  EXPECT_FALSE(stale.fresh) << "stale on the tick clock after " << wall_ms << " wall ms";
  EXPECT_GT(stale.stale_count, 0U);
  EXPECT_NEAR(stale.v_x, 0.0, 1e-3) << "decayed to rest";
  EXPECT_TRUE(stale.valid);
}

// mpc.activation_height: measured — the activation's first latch puts the
// command at the height the robot stands at; the planner plans at it.
YAML::Node MeasuredHeightConfig() {
  YAML::Node cfg = LockstepConfig();
  cfg["mpc"]["activation_height"] = "measured";
  return cfg;
}

TEST_F(LocomotionMpcLockstepTest, TheActivationHeightKeyIsChecked) {
  ExpectRefused(
      LockstepConfig(), [](YAML::Node& c) { c["mpc"]["activation_height"] = "latched"; },
      "unknown source");
  ExpectRefused(
      LockstepConfig(), [](YAML::Node& c) { c["mpc"]["activation_height"] = YAML::Node(); },
      "no value");
  YAML::Node absent = LockstepConfig();
  absent["mpc"].remove("activation_height");
  EXPECT_FALSE(ParseLocomotionControllerConfig(absent).mpc.activation_height_measured)
      << "absent = config";
  YAML::Node config = LockstepConfig();
  config["mpc"]["activation_height"] = "config";
  EXPECT_FALSE(ParseLocomotionControllerConfig(config).mpc.activation_height_measured);
  EXPECT_TRUE(
      ParseLocomotionControllerConfig(MeasuredHeightConfig()).mpc.activation_height_measured);
}

TEST_F(LocomotionMpcLockstepTest, AConfiguredActivationHeightIsTheCommandsBaseHeight) {
  YAML::Node cfg = LockstepConfig();
  cfg["mpc"]["activation_height"] = "config";
  BringUpLockstep(cfg);
  const double configured = cfg["command"]["base_height"].as<double>();
  ASSERT_NE(state_.devices[kBody].base_pose[2], configured) << "the rig stands elsewhere";
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kStand; }));
  EXPECT_EQ(ctrl_->LastCommand().base_height, configured);
  EXPECT_EQ(ctrl_->LastDiag().cmd_h, configured);
}

TEST_F(LocomotionMpcLockstepTest, AMeasuredActivationHeightIsWhereTheRobotStands) {
  const YAML::Node cfg = MeasuredHeightConfig();
  BringUpLockstep(cfg);
  const double stands = state_.devices[kBody].base_pose[2];
  ASSERT_NE(stands, cfg["command"]["base_height"].as<double>());
  ASSERT_GT(stands, cfg["command"]["h_min"].as<double>());
  ASSERT_LT(stands, cfg["command"]["h_max"].as<double>());
  EXPECT_EQ(ctrl_->LastCommand().base_height, cfg["command"]["base_height"].as<double>())
      << "before the first tick: the activation's reset";
  static_cast<void>(Tick());  // the latch
  EXPECT_EQ(ctrl_->LastCommand().base_height, stands);
  EXPECT_EQ(ctrl_->LastCommand().sequence, 0U) << "a reset, not a step";
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kStand; }));
  EXPECT_EQ(ctrl_->LastCommand().base_height, stands) << "the steps keep it";
  EXPECT_EQ(ctrl_->LastDiag().cmd_h, stands);
}

// Only the activation's latch: an E-STOP edge restarts the reference with the
// robot elsewhere and the commanded height stays.
TEST_F(LocomotionMpcLockstepTest, ALaterLatchKeepsTheCommandedHeight) {
  BringUpLockstep(MeasuredHeightConfig());
  const double stands = state_.devices[kBody].base_pose[2];
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kStand; }));
  ctrl_->TriggerEstop();
  static_cast<void>(Tick());
  state_.devices[kBody].base_pose[2] = stands - 0.004;
  ctrl_->ClearEstop();
  for (int i = 0; i < 2 * static_cast<int>(n_); ++i) {
    static_cast<void>(Tick());
  }
  EXPECT_EQ(ctrl_->LastCommand().base_height, stands);
}

TEST_F(LocomotionMpcLockstepTest, AMeasuredActivationHeightIsClampedIntoTheCommandRange) {
  const YAML::Node cfg = MeasuredHeightConfig();
  BringUpLockstep(cfg);
  const double h_max = cfg["command"]["h_max"].as<double>();
  state_.devices[kBody].base_pose[2] = h_max + 0.2;
  static_cast<void>(Tick());
  EXPECT_EQ(ctrl_->LastCommand().base_height, h_max);
}

// A second activation latches again, at where the robot stands then.
TEST_F(LocomotionMpcLockstepTest, EachActivationLatchesItsOwnHeight) {
  BringUpLockstep(MeasuredHeightConfig());
  const double first = state_.devices[kBody].base_pose[2];
  static_cast<void>(Tick());
  ASSERT_EQ(ctrl_->LastCommand().base_height, first);
  Deactivate();
  state_.devices[kBody].base_pose[2] = first - 0.006;
  Activate();
  static_cast<void>(Tick());
  EXPECT_EQ(ctrl_->LastCommand().base_height, first - 0.006);
}

TEST_F(LocomotionMpcLockstepTest, ACommandRateOffTheTickGridIsRefusedUnderLockstep) {
  YAML::Node cfg = LockstepConfig();
  cfg["command"]["sample_rate_hz"] = 300.0;  // 500 / 300 ticks a step
  auto ctrl = MakeController();
  EXPECT_THROW(ctrl->LoadConfig(cfg), std::invalid_argument);
  YAML::Node free_run = MpcConfig();
  free_run["command"]["sample_rate_hz"] = 300.0;
  auto other = MakeController();
  EXPECT_NO_THROW(other->LoadConfig(free_run)) << "the wall timer takes any rate";
}

// Control: the free-run thread publishes on the wall clock; the rig's ticks
// are immediate, so a plan's age at its first row is whatever the race gave,
// not the delivery.
TEST_F(LocomotionMpcLockstepTest, FreeRunPlansDoNotArriveAtTheDeliveryAge) {
  YAML::Node cfg = MpcConfig();
  cfg["mpc"]["sync"]["mode"] = "free_run";
  Configure(cfg, SimSyncedNode(), "test_locomotion_mpc_free_run");
  stub_ = SpawnStandingStub(*ctrl_);
  ASSERT_NE(stub_, nullptr);
  Activate();
  MakeStanding();
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kStand; }));
  int plans = 0;
  int at_delivery_age = 0;
  std::uint64_t last_plan = 0;
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (plans < 10 && std::chrono::steady_clock::now() < deadline) {
    static_cast<void>(Tick());
    const auto& d = ctrl_->LastDiag();
    if (d.plan_id != 0 && d.plan_id != last_plan) {
      last_plan = d.plan_id;
      ++plans;
      at_delivery_age += d.policy_age_us == 35.0 * kTickUs ? 1 : 0;
    }
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_GE(plans, 5);
  EXPECT_LT(at_delivery_age * 10, plans * 9) << "free-run ages are not the delivery's";
  EXPECT_FALSE(ctrl_->LastDiag().mpc_sync_lockstep);
}

}  // namespace
