// Tier 1 — the global E-STOP hold is a LATCH, not a follower (issue #588).
//
// WHY THE FIXTURE MOVES THE MEASUREMENT. The E-STOP substitution tests in
// test_rt_loop_pipeline.cpp feed a state that never changes, and against such
// a state "command the measured position" and "command the position latched
// when the stop began" are indistinguishable. The live-device creep #588
// measured in sim (thumb ~3.4 mrad/s) is exactly the difference: a position
// servo with a steady-state error, re-targeted to its own measurement every
// tick, walks. So every backend here reports a position the test moves between
// ticks, and each case asserts that the measurement DID move — a case whose
// measurement stood still would pass against either implementation.
//
// Ticks are driven by hand on the test thread (ControlLoop(), no RT thread),
// which makes every write the backend receives attributable to one tick.

#include "rclcpp_suite_fixture.hpp"
#include "rt_cm_test_access.hpp"
#include <rtc_msgs/srv/clear_estop.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace rtc {
namespace {

using Access = ControllerLifecycleTestAccess;

// A command no hold can produce: the controller's output is recognisable on
// the wire, so "the controller reached the actuator" is one comparison.
constexpr double kControllerCmd = 9.0;

// Emits kControllerCmd on every channel of every device it is handed — a
// valid, in-contract output, so nothing but the E-STOP guard can replace it.
// Never overrides TriggerEstop/ClearEstop: like PipelineTestController, its
// output under E-STOP is the controller's normal command, and only the CM's
// substitution keeps it off the wire.
class EchoController : public RTControllerInterface {
 public:
  explicit EchoController(std::string name) : name_(std::move(name)) {}

  ControllerOutput Compute(const ControllerState& state) noexcept override {
    ControllerOutput out{};
    out.num_devices = state.num_devices;
    for (int d = 0; d < state.num_devices; ++d) {
      auto& dout = out.devices[static_cast<std::size_t>(d)];
      dout.num_channels = state.devices[static_cast<std::size_t>(d)].num_channels;
      for (int c = 0; c < dout.num_channels; ++c) {
        dout.commands[static_cast<std::size_t>(c)] = kControllerCmd;
      }
    }
    return out;
  }

  void SetDeviceTarget(int /*device_idx*/, std::span<const double> /*target*/) noexcept override {}

  std::string_view Name() const noexcept override { return name_; }

 private:
  std::string name_;
};

// A device whose measured position the test moves. Channel c reports
// base[c] + offset. A channel in `hole_mask` is reported as a hole and its
// cache value is kHoleValue — the persistent cache holding something the
// latest message did not write, which the hold must not treat as a reading.
class DriftingBackend : public DeviceBackend {
 public:
  static constexpr int kChannels = 3;
  static constexpr double kHoleValue = 42.0;

  explicit DriftingBackend(double base) {
    for (int c = 0; c < kChannels; ++c) {
      base_[static_cast<std::size_t>(c)] = base + 0.1 * c;
    }
  }

  void Configure(rclcpp_lifecycle::LifecycleNode* /*node*/, const DeviceBackendConfig& /*config*/,
                 rclcpp::CallbackGroup::SharedPtr /*group*/) override {}

  void Activate() override {}

  void Deactivate() override {}

  bool ReadState(DeviceStateCache& cache) noexcept override {
    const uint64_t holes = hole_mask.load(std::memory_order_relaxed);
    cache.num_channels = kChannels;
    for (int c = 0; c < kChannels; ++c) {
      const auto uc = static_cast<std::size_t>(c);
      if (((holes >> c) & 1U) != 0U) {
        cache.positions[uc] = kHoleValue;
      } else if (c == 0 && nan_ch0.load(std::memory_order_relaxed)) {
        cache.positions[uc] = std::numeric_limits<double>::quiet_NaN();
      } else {
        cache.positions[uc] = Measured(c);
      }
    }
    cache.hole_mask = holes;
    cache.valid = true;
    return true;
  }

  // Atomics throughout: the service cases write from a stand-in RT thread
  // and read from the test thread, and this file also runs under TSAN.
  void WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                    CommandType /*ct*/) noexcept override {
    last_num_channels_.store(slot.num_channels, std::memory_order_relaxed);
    for (int c = 0; c < kChannels; ++c) {
      last_commands_[static_cast<std::size_t>(c)].store(slot.commands[static_cast<std::size_t>(c)],
                                                        std::memory_order_relaxed);
    }
    if (slot.num_channels > 0 && slot.commands[0] == kControllerCmd) {
      controller_writes_.fetch_add(1, std::memory_order_relaxed);
    }
    writes_.fetch_add(1, std::memory_order_release);
  }

  void WriteSafeCommand() noexcept override {
    safe_writes_.fetch_add(1, std::memory_order_relaxed);
  }

  // `stale` backdates the liveness stamp, so the CM watchdog sees a device
  // that stopped reporting — the E-STOP cause that is still present when a
  // clear is attempted.
  std::chrono::steady_clock::time_point LastStateStamp() const noexcept override {
    const auto now = std::chrono::steady_clock::now();
    return stale.load(std::memory_order_relaxed) ? now - std::chrono::seconds(10) : now;
  }

  // What ReadState reports for a readable channel right now.
  [[nodiscard]] double Measured(int c) const noexcept {
    return base_[static_cast<std::size_t>(c)] + offset.load(std::memory_order_relaxed);
  }

  [[nodiscard]] double LastCommand(int c) const {
    return last_commands_[static_cast<std::size_t>(c)].load(std::memory_order_relaxed);
  }

  [[nodiscard]] int LastNumChannels() const {
    return last_num_channels_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] int Writes() const { return writes_.load(std::memory_order_acquire); }

  [[nodiscard]] int SafeWrites() const { return safe_writes_.load(std::memory_order_relaxed); }

  // Writes that carried the controller's own command (kControllerCmd).
  [[nodiscard]] int ControllerWrites() const {
    return controller_writes_.load(std::memory_order_relaxed);
  }

  std::atomic<double> offset{0.0};
  std::atomic<uint64_t> hole_mask{0};
  std::atomic<bool> nan_ch0{false};
  std::atomic<bool> stale{false};

 private:
  std::array<double, kChannels> base_{};
  std::array<std::atomic<double>, kChannels> last_commands_{};
  std::atomic<int> last_num_channels_{0};
  std::atomic<int> writes_{0};
  std::atomic<int> safe_writes_{0};
  std::atomic<int> controller_writes_{0};
};

// One controller ("ctrl_a") claiming one device group on slot 0. Built the
// way test_catching_cm_services hosts a controller: the CM state bring-up
// leaves behind, without YAML or an RT thread.
class EstopHoldLatchTest : public RclcppSuiteTest {
 protected:
  void SetUp() override {
    node_ = std::make_shared<RtControllerNode>("test_estop_hold_latch_node");
    std::vector<std::unique_ptr<RTControllerInterface>> ctrls;
    ctrls.push_back(std::make_unique<EchoController>("ctrl_a"));
    ctrls.push_back(std::make_unique<EchoController>("ctrl_b"));
    Access::InjectControllers(*node_, std::move(ctrls), {"type_a", "type_b"});
    Access::HostGroups(*node_, 0, {"arm"}, {0});
    Access::HostGroups(*node_, 1, {"hand"}, {1});
    auto arm = std::make_unique<DriftingBackend>(0.5);
    auto hand = std::make_unique<DriftingBackend>(-0.3);
    arm_ = arm.get();
    hand_ = hand.get();
    Access::InjectBackend(*node_, 0, std::move(arm));
    Access::InjectBackend(*node_, 1, std::move(hand));
    Access::CacheCommandTypeMasks(*node_);
    Access::EnsureEstopPublisher(*node_);
    Access::MarkActive(*node_, 0);
  }

  void Tick() { Access::Tick(*node_); }

  // Moves the arm's measurement, ticks once, and returns what the tick wrote.
  void DriftAndTick(double step) {
    arm_->offset.store(arm_->offset.load() + step);
    Tick();
  }

  std::shared_ptr<RtControllerNode> node_;
  DriftingBackend* arm_{nullptr};
  DriftingBackend* hand_{nullptr};
};

constexpr double kStep = 1e-3;
constexpr int kDriftTicks = 50;

TEST_F(EstopHoldLatchTest, EstopHoldCommandsTheLatchedPositionWhileTheMeasurementDrifts) {
  Tick();
  ASSERT_DOUBLE_EQ(arm_->LastCommand(0), kControllerCmd) << "precondition: controller reaches wire";

  Access::CallTriggerEstop(*node_, "test_estop");
  Tick();
  const std::array<double, 3> latched{arm_->Measured(0), arm_->Measured(1), arm_->Measured(2)};
  for (int c = 0; c < 3; ++c) {
    ASSERT_DOUBLE_EQ(arm_->LastCommand(c), latched[static_cast<std::size_t>(c)])
        << "the stop tick must hold where the device is";
  }

  for (int t = 0; t < kDriftTicks; ++t) {
    DriftAndTick(kStep);
    for (int c = 0; c < 3; ++c) {
      ASSERT_DOUBLE_EQ(arm_->LastCommand(c), latched[static_cast<std::size_t>(c)])
          << "tick " << t << " channel " << c
          << ": the hold followed the measurement instead of latching it";
    }
  }
  // Non-vacuity: the measurement really moved away from the latched value.
  EXPECT_GT(std::abs(arm_->Measured(0) - latched[0]), 0.9 * kStep * kDriftTicks);
}

TEST_F(EstopHoldLatchTest, AChannelUnreadableAtTheLatchTakesItsLastReadableValue) {
  Tick();
  const double last_good_ch1 = arm_->Measured(1);

  // Channel 1 stops being written; the cache now reports kHoleValue for it.
  arm_->hole_mask.store(0b010);
  arm_->offset.store(0.25);
  Access::CallTriggerEstop(*node_, "test_estop");
  Tick();

  EXPECT_DOUBLE_EQ(arm_->LastCommand(1), last_good_ch1)
      << "latched the hole's cache value instead of the last reading";
  EXPECT_DOUBLE_EQ(arm_->LastCommand(0), arm_->Measured(0)) << "readable channels hold here";
  EXPECT_DOUBLE_EQ(arm_->LastCommand(2), arm_->Measured(2));
}

TEST_F(EstopHoldLatchTest, AChannelNeverReadableFallsBackToTheCacheValue) {
  // No reading ever arrives for channel 2 — the cache value is the only
  // position there is, as in the inference controller's latch (6bbc4d22).
  arm_->hole_mask.store(0b100);
  Tick();
  Access::CallTriggerEstop(*node_, "test_estop");
  Tick();
  EXPECT_DOUBLE_EQ(arm_->LastCommand(2), DriftingBackend::kHoleValue);
  EXPECT_DOUBLE_EQ(arm_->LastCommand(0), arm_->Measured(0));
}

TEST_F(EstopHoldLatchTest, NonFiniteWithNoReadingDropsToSafeOutputThenLatchesOnceFinite) {
  arm_->nan_ch0.store(true);
  Tick();
  Access::CallTriggerEstop(*node_, "test_estop");
  const int safe_before = arm_->SafeWrites();
  Tick();
  EXPECT_EQ(arm_->SafeWrites(), safe_before + 1) << "no honest position — hand it to the backend";

  arm_->nan_ch0.store(false);
  Tick();
  const double latched = arm_->Measured(0);
  EXPECT_DOUBLE_EQ(arm_->LastCommand(0), latched) << "latched on the first finite tick";
  DriftAndTick(kStep);
  EXPECT_DOUBLE_EQ(arm_->LastCommand(0), latched) << "and it stays latched";
}

TEST_F(EstopHoldLatchTest, ANonFiniteReadingAfterTheLatchDoesNotDisturbIt) {
  Tick();
  Access::CallTriggerEstop(*node_, "test_estop");
  Tick();
  const double latched = arm_->Measured(0);
  arm_->nan_ch0.store(true);
  const int safe_before = arm_->SafeWrites();
  Tick();
  EXPECT_DOUBLE_EQ(arm_->LastCommand(0), latched);
  EXPECT_EQ(arm_->SafeWrites(), safe_before);
}

TEST_F(EstopHoldLatchTest, ARepeatedTriggerWhileLatchedDoesNotRecapture) {
  Tick();
  Access::CallTriggerEstop(*node_, "first");
  Tick();
  const double latched = arm_->Measured(0);
  DriftAndTick(kStep);
  Access::CallTriggerEstop(*node_, "second");
  DriftAndTick(kStep);
  EXPECT_DOUBLE_EQ(arm_->LastCommand(0), latched);
}

TEST_F(EstopHoldLatchTest, ADirectClearDiscardsTheLatchAndTheNextStopLatchesAfresh) {
  Tick();
  Access::CallTriggerEstop(*node_, "first");
  Tick();
  const double first_latch = arm_->Measured(0);

  // No verification token: a direct ClearGlobalEstop() (on_deactivate's path)
  // lowers the latch at once, as #299's tests pin.
  ASSERT_EQ(Access::CallClearEstop(*node_), Access::EstopClearOutcome::kCleared);
  DriftAndTick(kStep);
  EXPECT_DOUBLE_EQ(arm_->LastCommand(0), kControllerCmd) << "the controller resumes";

  DriftAndTick(kStep);
  Access::CallTriggerEstop(*node_, "second");
  Tick();
  EXPECT_DOUBLE_EQ(arm_->LastCommand(0), arm_->Measured(0)) << "the new stop latches afresh";
  EXPECT_NE(arm_->LastCommand(0), first_latch) << "not the discarded latch";
}

TEST_F(EstopHoldLatchTest, TorqueModeStillHoldsZeroNewtonMetres) {
  // Unchanged by the latch (#588 Q1): the CM has no dynamic model, so a
  // torque-mode hold is 0 N·m whatever the latched position is.
  class TorqueEcho : public EchoController {
   public:
    using EchoController::EchoController;

    CommandType GetCommandType() const noexcept override { return CommandType::kTorque; }
  };

  class TorqueDrifting : public DriftingBackend {
   public:
    using DriftingBackend::DriftingBackend;

    bool AcceptsCommandType(CommandType ct) const noexcept override {
      return ct == CommandType::kTorque;
    }
  };

  std::vector<std::unique_ptr<RTControllerInterface>> ctrls;
  ctrls.push_back(std::make_unique<TorqueEcho>("ctrl_t"));
  Access::InjectControllers(*node_, std::move(ctrls), {"type_t"});
  Access::HostGroups(*node_, 0, {"arm"}, {0});
  auto arm = std::make_unique<TorqueDrifting>(0.5);
  auto* torque_arm = arm.get();
  Access::InjectBackend(*node_, 0, std::move(arm));
  Access::CacheCommandTypeMasks(*node_);
  Access::MarkActive(*node_, 0);

  Tick();
  Access::CallTriggerEstop(*node_, "test_estop");
  for (int t = 0; t < 5; ++t) {
    torque_arm->offset.store(torque_arm->offset.load() + kStep);
    Tick();
    EXPECT_DOUBLE_EQ(torque_arm->LastCommand(0), 0.0);
  }
}

// ── Clear verification window (decisions Q2 · Q12 (a) · Q13) ─────────────────
//
// BeginClearVerification + ClearGlobalEstop is exactly what /rtc_cm/clear_estop
// does; driving it by hand makes every tick of the window observable.

TEST_F(EstopHoldLatchTest, AVerifiedClearHoldsForTheWholeWindowThenReleases) {
  Tick();
  Access::CallTriggerEstop(*node_, "test_estop");
  Tick();
  const double latched = arm_->Measured(0);
  const auto window = Access::VerifyWindowTicks(*node_);
  ASSERT_GE(window, Access::GetWatchdogCheckDivisor(*node_) + 1U)
      << "the window must outlast one watchdog period";
  Access::CallFlushEstopStatus(*node_);  // the trigger's own publish, out of the way

  static_cast<void>(Access::BeginClearVerification(*node_));
  ASSERT_EQ(Access::CallClearEstop(*node_), Access::EstopClearOutcome::kCleared);
  ASSERT_FALSE(Access::IsEstopped(*node_)) << "the latch itself drops at once (#299)";
  EXPECT_FALSE(Access::GetEstopStatusPending(*node_))
      << "the clear must not publish — the reported value is still true";
  EXPECT_TRUE(Access::EstopStatusValue(*node_));

  const auto substituted_before = Access::GetEstopSubstitutedOutputCount(*node_);
  std::uint64_t held = 0;
  while (Access::IsClearVerifying(*node_) && held < 10 * window) {
    DriftAndTick(kStep);
    if (!Access::IsClearVerifying(*node_)) {
      break;  // this tick closed the window — its output is the controller's
    }
    ++held;
    ASSERT_DOUBLE_EQ(arm_->LastCommand(0), latched)
        << "tick " << held << " of the window let something other than the hold out";
    ASSERT_TRUE(Access::EstopStatusValue(*node_));
  }
  EXPECT_EQ(held, window) << "held ticks != EstopVerifyWindowTicks()";
  EXPECT_EQ(Access::GetEstopVerifyHeldOutputCount(*node_), window);
  EXPECT_EQ(Access::GetEstopSubstitutedOutputCount(*node_), substituted_before)
      << "window ticks are counted apart from latch ticks";

  EXPECT_DOUBLE_EQ(arm_->LastCommand(0), kControllerCmd) << "the closing tick releases";
  EXPECT_FALSE(Access::EstopStatusValue(*node_));
  EXPECT_TRUE(Access::GetEstopStatusPending(*node_)) << "the RT side schedules the false publish";

  // Released means released: the next stop latches afresh.
  DriftAndTick(kStep);
  Access::CallTriggerEstop(*node_, "again");
  Tick();
  EXPECT_DOUBLE_EQ(arm_->LastCommand(0), arm_->Measured(0));
  EXPECT_NE(arm_->LastCommand(0), latched);
}

TEST_F(EstopHoldLatchTest, AStopInsideTheWindowKeepsTheOriginalLatch) {
  Tick();
  Access::CallTriggerEstop(*node_, "first");
  Tick();
  const double latched = arm_->Measured(0);
  static_cast<void>(Access::BeginClearVerification(*node_));
  ASSERT_EQ(Access::CallClearEstop(*node_), Access::EstopClearOutcome::kCleared);
  DriftAndTick(kStep);
  DriftAndTick(kStep);

  // The hold never paused, so nothing was released: no recapture (#588 (a)).
  Access::CallTriggerEstop(*node_, "second");
  const auto window = Access::VerifyWindowTicks(*node_);
  for (std::uint64_t t = 0; t < 3 * window; ++t) {
    DriftAndTick(kStep);
    ASSERT_DOUBLE_EQ(arm_->LastCommand(0), latched) << "tick " << t;
  }
  EXPECT_TRUE(Access::EstopStatusValue(*node_));
  EXPECT_TRUE(Access::IsClearVerifying(*node_))
      << "the window stays open behind the re-raised latch — the RT side never "
         "drops it while latched";
}

TEST_F(EstopHoldLatchTest, TheControllerCannotBeSwitchedDuringTheWindow) {
  Tick();
  Access::CallTriggerEstop(*node_, "test_estop");
  Tick();
  std::string message;
  EXPECT_FALSE(Access::CallSwitch(*node_, "ctrl_b", message));
  EXPECT_EQ(message, "E-STOP active");

  static_cast<void>(Access::BeginClearVerification(*node_));
  ASSERT_EQ(Access::CallClearEstop(*node_), Access::EstopClearOutcome::kCleared);
  message.clear();
  EXPECT_FALSE(Access::CallSwitch(*node_, "ctrl_b", message))
      << "the window is E-STOP everywhere outside the latch (#588 (b′))";
  EXPECT_NE(message.find("verified"), std::string::npos) << message;
  EXPECT_EQ(Access::GetActiveIdx(*node_), 0);
}

TEST_F(EstopHoldLatchTest, ASlotFirstHeldAfterASwitchLatchesItsOwnPosition) {
  // The switch service checks the latch and then activates — a trigger can
  // land in between. The hand group (slot 1) enters the hold only after the
  // arm (slot 0) latched; it must latch ITS position, never the arm's.
  Tick();
  Access::CallTriggerEstop(*node_, "test_estop");
  Tick();
  const double arm_latched = arm_->Measured(0);

  hand_->offset.store(0.07);
  Access::MarkActive(*node_, 1);  // ctrl_b: device 0 is the hand, on slot 1
  Tick();
  const double hand_latched = hand_->Measured(0);
  EXPECT_DOUBLE_EQ(hand_->LastCommand(0), hand_latched);
  EXPECT_NE(hand_->LastCommand(0), arm_latched) << "the arm's latch went to the hand";

  hand_->offset.store(hand_->offset.load() + 0.01);
  Tick();
  EXPECT_DOUBLE_EQ(hand_->LastCommand(0), hand_latched) << "and it is a latch, too";

  // Back to the arm: its latch is still the one from the stop.
  Access::MarkActive(*node_, 0);
  DriftAndTick(kStep);
  EXPECT_DOUBLE_EQ(arm_->LastCommand(0), arm_latched);
}

TEST_F(EstopHoldLatchTest, OnErrorLeavesNoWindowOpen) {
  Tick();
  Access::CallTriggerEstop(*node_, "test_estop");
  static_cast<void>(Access::BeginClearVerification(*node_));
  ASSERT_TRUE(Access::IsClearVerifying(*node_));
  const rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                         "inactive");
  ASSERT_EQ(node_->on_error(inactive), RtControllerNode::CallbackReturn::SUCCESS);
  EXPECT_FALSE(Access::IsClearVerifying(*node_));
  EXPECT_TRUE(Access::IsEstopped(*node_)) << "on_error latches regardless";
}

// ── Over the real service, with a stand-in RT thread ─────────────────────────
//
// The watchdog is what makes a cause "still present": a device that stopped
// reporting re-latches the E-STOP at its next check, which is the case the
// verification window exists for — before #588 the controller's output went
// out during the ticks between the clear and that check (#588 ③).

class EstopClearServiceWindowTest : public EstopHoldLatchTest {
 protected:
  void SetUp() override {
    EstopHoldLatchTest::SetUp();
    Access::AddDeviceTimeout(*node_, "arm", std::chrono::milliseconds(50), 0);
    Access::CallCreateFixedSafetyPublishers(*node_);
    Access::BringServicesOnline(*node_);

    obs_node_ = std::make_shared<rclcpp::Node>("test_estop_hold_latch_obs");
    // Volatile and deep, so every sample published while it is up is kept in
    // order — what the "never a false edge from a refused clear" cases read.
    // (The in-tree readers are transient_local; they see the same edges.)
    status_sub_ = obs_node_->create_subscription<std_msgs::msg::Bool>(
        "/system/estop_status", rclcpp::QoS(10), [this](std_msgs::msg::Bool::SharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex_);
          statuses_.push_back(m->data);
        });
    client_ = obs_node_->create_client<rtc_msgs::srv::ClearEstop>("/rtc_cm/clear_estop");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(obs_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
    ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(2)));
  }

  void TearDown() override {
    StopTicking();
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  // Mirrors ControlLoopThread::OnTick — ControlLoop, then the watchdog every
  // divisor ticks — plus the log drain that publishes /system/estop_status.
  // A violation is a tick that put the controller's command on the wire while
  // the hold was still active after it (only this thread closes a window).
  void StartTicking() {
    ticking_.store(true);
    tick_thread_ = std::thread([this]() {
      const auto divisor = Access::GetWatchdogCheckDivisor(*node_);
      const auto period = Access::GetControlPeriod(*node_);
      std::uint64_t n = 0;
      while (ticking_.load()) {
        const int before = arm_->ControllerWrites();
        Access::Tick(*node_);
        // Judged before the watchdog runs: a check that latches right after a
        // tick which legitimately wrote the controller's command is not a leak.
        if (arm_->ControllerWrites() != before && Access::IsHoldActive(*node_)) {
          violations_.fetch_add(1);
        }
        if (++n % divisor == 0) {
          Access::CallCheckTimeouts(*node_);
        }
        if (n % 5 == 0) {
          Access::CallDrainLog(*node_);
        }
        std::this_thread::sleep_for(period);
      }
    });
  }

  void StopTicking() {
    ticking_.store(false);
    if (tick_thread_.joinable()) {
      tick_thread_.join();
    }
  }

  rtc_msgs::srv::ClearEstop::Response::SharedPtr CallClear(const std::string& ack) {
    auto req = std::make_shared<rtc_msgs::srv::ClearEstop::Request>();
    req->reason_ack = ack;
    auto fut = client_->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      return nullptr;
    }
    return fut.get();
  }

  static bool WaitFor(const std::function<bool()>& pred,
                      std::chrono::milliseconds timeout = std::chrono::milliseconds(3000)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return pred();
  }

  std::vector<bool> Statuses() {
    std::lock_guard<std::mutex> lock(mutex_);
    return statuses_;
  }

  std::shared_ptr<rclcpp::Node> obs_node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
  rclcpp::Client<rtc_msgs::srv::ClearEstop>::SharedPtr client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::thread tick_thread_;
  std::atomic<bool> ticking_{false};
  std::atomic<int> violations_{0};
  std::mutex mutex_;
  std::vector<bool> statuses_;
};

TEST_F(EstopClearServiceWindowTest, ARefusedClearNeverReachesTheWireNorReportsFalse) {
  StartTicking();
  ASSERT_TRUE(WaitFor([this] { return arm_->ControllerWrites() > 3; })) << "never ran";

  arm_->stale.store(true);  // the device stops reporting; the watchdog latches
  ASSERT_TRUE(WaitFor([this] { return Access::IsEstopped(*node_); }));
  ASSERT_TRUE(WaitFor([this] {
    const auto s = Statuses();
    return !s.empty() && s.back();
  })) << "the latch was never reported";
  const int controller_writes = arm_->ControllerWrites();
  const std::size_t reported = Statuses().size();

  const auto resp = CallClear("arm_timeout");
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->ok) << resp->message;
  EXPECT_NE(resp->message.find("re-latched"), std::string::npos) << resp->message;

  // Let any queued drain run.
  const auto window = static_cast<int>(Access::VerifyWindowTicks(*node_));
  std::this_thread::sleep_for(Access::GetControlPeriod(*node_) * (4 * window));
  EXPECT_EQ(arm_->ControllerWrites(), controller_writes)
      << "the controller's output reached the wire during the refused clear";
  EXPECT_EQ(violations_.load(), 0);
  const auto s = Statuses();
  for (std::size_t i = reported; i < s.size(); ++i) {
    EXPECT_TRUE(s[i]) << "estop_status reported false during a refused clear (#588 ③, Q13)";
  }
  EXPECT_TRUE(Access::IsEstopped(*node_));
}

TEST_F(EstopClearServiceWindowTest, AnAcceptedClearRepliesOnlyOnceTheWindowHasClosed) {
  StartTicking();
  ASSERT_TRUE(WaitFor([this] { return arm_->ControllerWrites() > 3; })) << "never ran";
  Access::CallTriggerEstop(*node_, "test_estop");
  ASSERT_TRUE(WaitFor([this] { return Access::GetEstopSubstitutedOutputCount(*node_) > 3U; }));
  // This trigger came from the test thread, so it can land between a tick that
  // legitimately wrote the controller's command and the tick thread's check —
  // a false count that says nothing about the window. Counted from here on.
  violations_.store(0);

  const auto resp = CallClear("test_estop");
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->ok) << resp->message;
  EXPECT_FALSE(Access::IsClearVerifying(*node_)) << "replied ok with the window still open";
  EXPECT_GE(Access::GetEstopVerifyHeldOutputCount(*node_), Access::VerifyWindowTicks(*node_) - 1U);

  ASSERT_TRUE(WaitFor([this] { return arm_->ControllerWrites() > 0; }));
  EXPECT_EQ(violations_.load(), 0) << "controller output reached the wire inside the window";
  ASSERT_TRUE(WaitFor([this] {
    const auto s = Statuses();
    return !s.empty() && !s.back();
  })) << "the verified clear was never reported";
}

TEST_F(EstopClearServiceWindowTest, ATimedOutReplyLeavesTheArmHeldUntilTheLoopRunsTheWindow) {
  // No stand-in RT thread yet: the service waits out its deadline.
  Access::Tick(*node_);
  Access::CallTriggerEstop(*node_, "test_estop");
  Access::Tick(*node_);
  const double latched = arm_->Measured(0);

  const auto resp = CallClear("test_estop");
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->ok);
  EXPECT_NE(resp->message.find("the latch is DOWN"), std::string::npos) << resp->message;
  EXPECT_NE(resp->message.find("hold stays on"), std::string::npos) << resp->message;
  EXPECT_TRUE(Access::IsClearVerifying(*node_)) << "the service must never close the window";

  // A second call while that window is open is not "nothing latched".
  const auto again = CallClear("");
  ASSERT_NE(again, nullptr);
  EXPECT_FALSE(again->ok) << again->message;
  EXPECT_NE(again->message.find("still being verified"), std::string::npos) << again->message;

  // The loop comes back: the hold lasts the window, then releases.
  const auto window = Access::VerifyWindowTicks(*node_);
  for (std::uint64_t t = 0; t < window; ++t) {
    DriftAndTick(kStep);
    ASSERT_DOUBLE_EQ(arm_->LastCommand(0), latched) << "tick " << t;
  }
  DriftAndTick(kStep);
  EXPECT_FALSE(Access::IsClearVerifying(*node_));
  EXPECT_DOUBLE_EQ(arm_->LastCommand(0), kControllerCmd);
}

TEST_F(EstopClearServiceWindowTest, ALateTransientLocalSubscriberSeesTheLatchedEstop) {
  Access::CallTriggerEstop(*node_, "test_estop");
  Access::CallFlushEstopStatus(*node_);
  ASSERT_TRUE(WaitFor([this] {
    const auto s = Statuses();
    return !s.empty() && s.back();
  })) << "a volatile subscriber that was already up stopped receiving";

  // Started after the only publish — the GUI launched after the latch (#588 ②).
  auto late_node = std::make_shared<rclcpp::Node>("test_estop_hold_latch_late");
  std::atomic<int> received_true{0};
  rclcpp::QoS latched{1};
  latched.transient_local();
  auto late_sub = late_node->create_subscription<std_msgs::msg::Bool>(
      "/system/estop_status", latched, [&received_true](std_msgs::msg::Bool::SharedPtr m) {
        if (m->data) {
          received_true.fetch_add(1);
        }
      });
  rclcpp::executors::SingleThreadedExecutor late_exec;
  late_exec.add_node(late_node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (received_true.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    late_exec.spin_some(std::chrono::milliseconds(20));
  }
  EXPECT_GT(received_true.load(), 0) << "a late subscriber did not get the latched E-STOP";
}

TEST_F(EstopClearServiceWindowTest, ADeactivateMidWindowStillReportsTheClear) {
  // The RT thread is joined before the window's closing tick, and the latch is
  // already down — the only thing left to report the clear is the reset.
  Access::CallTriggerEstop(*node_, "test_estop");
  Access::CallFlushEstopStatus(*node_);
  ASSERT_TRUE(WaitFor([this] {
    const auto s = Statuses();
    return !s.empty() && s.back();
  }));
  static_cast<void>(Access::BeginClearVerification(*node_));
  ASSERT_EQ(Access::CallClearEstop(*node_), Access::EstopClearOutcome::kCleared);
  ASSERT_TRUE(Access::IsClearVerifying(*node_));

  const rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(node_->on_deactivate(active), RtControllerNode::CallbackReturn::SUCCESS);
  EXPECT_FALSE(Access::IsClearVerifying(*node_));
  EXPECT_TRUE(WaitFor([this] {
    const auto s = Statuses();
    return !s.empty() && !s.back();
  })) << "estop_status stuck at true after a mid-window deactivate";
}

TEST_F(EstopClearServiceWindowTest, AWindowDroppedByALifecycleResetIsNotReportedAsVerified) {
  // #608: the service waits for the window to go away and took "gone" to mean
  // "the RT loop ran it". A lifecycle transition drops the window too — no
  // tick, no watchdog turn, nothing re-evaluated the cause.
  Access::CallTriggerEstop(*node_, "test_estop");
  Access::CallFlushEstopStatus(*node_);
  // No ticking: whatever closes the window below, it is not the RT loop.
  auto req = std::make_shared<rtc_msgs::srv::ClearEstop::Request>();
  req->reason_ack = "test_estop";
  auto fut = client_->async_send_request(req);
  ASSERT_TRUE(WaitFor([this] {
    return !Access::IsEstopped(*node_) && Access::IsClearVerifying(*node_);
  })) << "precondition: the service lowered the latch and is waiting on its window";

  const rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(node_->on_deactivate(active), RtControllerNode::CallbackReturn::SUCCESS);

  ASSERT_EQ(fut.wait_for(std::chrono::seconds(10)), std::future_status::ready);
  const auto resp = fut.get();
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->ok) << "replied '" << resp->message << "' for a window nobody ran";
  EXPECT_NE(resp->message.find("NOT verified"), std::string::npos) << resp->message;
}

// ── A publisher that starts with the value it stands for (#607) ──────────────
//
// transient_local keeps the LAST PUBLISHED sample, and the status is published
// on change only — so a publisher created after the change holds nothing, and
// a subscriber that joins it reads "no E-STOP" by default.

namespace {

/// What a transient_local subscriber created NOW receives within `timeout`.
std::vector<bool> LateSubscriberReceives(
    const char* name, std::chrono::milliseconds timeout = std::chrono::milliseconds(1500)) {
  auto node = std::make_shared<rclcpp::Node>(name);
  std::vector<bool> got;
  rclcpp::QoS latched{1};
  latched.transient_local();
  auto sub = node->create_subscription<std_msgs::msg::Bool>(
      "/system/estop_status", latched,
      [&got](std_msgs::msg::Bool::SharedPtr m) { got.push_back(m->data); });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (got.empty() && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some(std::chrono::milliseconds(20));
  }
  return got;
}

}  // namespace

TEST_F(EstopClearServiceWindowTest, AFreshPublisherReportsNoEstopWithoutWaitingForAChange) {
  // SetUp created the publisher; nothing has happened since.
  Access::CallDrainLog(*node_);
  const auto got = LateSubscriberReceives("test_estop_hold_latch_fresh");
  ASSERT_FALSE(got.empty()) << "a subscriber cannot tell 'no E-STOP' from 'no controller manager'";
  EXPECT_FALSE(got.back());
}

TEST_F(EstopClearServiceWindowTest, APublisherRecreatedUnderALatchedEstopReportsIt) {
  Access::CallTriggerEstop(*node_, "test_estop");
  Access::CallFlushEstopStatus(*node_);
  const rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                         "inactive");
  ASSERT_EQ(node_->on_error(inactive), RtControllerNode::CallbackReturn::SUCCESS);
  ASSERT_TRUE(Access::IsEstopped(*node_)) << "precondition: on_error leaves the latch up";

  // What the next on_configure does, then one pass of the drain.
  Access::CallCreateFixedSafetyPublishers(*node_);
  Access::CallDrainLog(*node_);
  const auto got = LateSubscriberReceives("test_estop_hold_latch_recreated");
  ASSERT_FALSE(got.empty()) << "the recreated publisher holds no sample: a GUI started now "
                               "shows NORMAL over a held arm";
  EXPECT_TRUE(got.back());
}

// ── Concurrency (for the TSAN build) ─────────────────────────────────────────
//
// The RT tick, a non-RT trigger (on_error's path) and a clear with its window
// (the service's path) all at once. Under a normal build this checks only that
// the state converges; under -fsanitize=thread it is the race sensor for the
// token and the latch buffers (#588 SPRINT ④).
TEST_F(EstopHoldLatchTest, TriggerClearAndTickConcurrentlyConverge) {
  std::atomic<bool> run{true};
  std::thread ticker([&]() {
    while (run.load()) {
      Access::Tick(*node_);
    }
  });
  std::thread reader([&]() {
    // Atomics only. DrainLog is left out on purpose: its read of the reason
    // buffer against a concurrent trigger is a documented, pre-existing
    // tolerance (GlobalEstopReason) and would drown this sensor.
    while (run.load()) {
      static_cast<void>(Access::EstopStatusValue(*node_));
      static_cast<void>(Access::IsClearVerifying(*node_));
    }
  });
  for (int i = 0; i < 2000; ++i) {
    Access::CallTriggerEstop(*node_, "stress");
    static_cast<void>(Access::BeginClearVerification(*node_));
    static_cast<void>(Access::CallClearEstop(*node_));
  }
  run.store(false);
  ticker.join();
  reader.join();

  // Quiescent: the last clear's window closes after exactly its ticks.
  ASSERT_FALSE(Access::IsEstopped(*node_));
  for (std::uint64_t t = 0; t <= Access::VerifyWindowTicks(*node_) + 1U; ++t) {
    Tick();
  }
  EXPECT_FALSE(Access::IsClearVerifying(*node_));
  EXPECT_DOUBLE_EQ(arm_->LastCommand(0), kControllerCmd);
}

}  // namespace
}  // namespace rtc
