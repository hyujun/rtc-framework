// Phase 2 — controller-level lifecycle tests for RtControllerNode.
//
// Verifies controller_states_ transitions, ActivateController /
// DeactivateController helpers, and SwitchActiveController sequence
// (precondition rejection + happy-path swap with N=20 cycles).
//
// Uses a friend-class accessor (ControllerLifecycleTestAccess) so the
// helpers can be exercised without going through the full on_configure
// pipeline (which requires a complete URDF + controllers YAML config).

#include "rtc_controller_manager/rt_controller_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace rtc {

// Minimal RTControllerInterface stub for lifecycle tests. Records the
// number of on_activate / on_deactivate calls and the snapshot received.
class MockController : public RTControllerInterface {
public:
  explicit MockController(std::string name) : name_(std::move(name)) {}

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &prev,
              const ControllerState &snapshot) noexcept override {
    activate_count_.fetch_add(1, std::memory_order_relaxed);
    last_snapshot_devices_ = snapshot.num_devices;
    return RTControllerInterface::on_activate(prev, snapshot);
  }

  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & /*prev*/) noexcept override {
    deactivate_count_.fetch_add(1, std::memory_order_relaxed);
    return CallbackReturn::SUCCESS;
  }

  // Pure-virtual implementations — minimal no-ops.
  ControllerOutput
  Compute(const ControllerState & /*state*/) noexcept override {
    return ControllerOutput{};
  }
  void SetDeviceTarget(int /*device_idx*/,
                       std::span<const double> /*target*/) noexcept override {}
  std::string_view Name() const noexcept override { return name_; }
  void
  InitializeHoldPosition(const ControllerState & /*state*/) noexcept override {
    hold_init_count_.fetch_add(1, std::memory_order_relaxed);
  }

  int ActivateCount() const {
    return activate_count_.load(std::memory_order_relaxed);
  }
  int DeactivateCount() const {
    return deactivate_count_.load(std::memory_order_relaxed);
  }
  int HoldInitCount() const {
    return hold_init_count_.load(std::memory_order_relaxed);
  }
  int LastSnapshotDevices() const { return last_snapshot_devices_; }

private:
  std::string name_;
  std::atomic<int> activate_count_{0};
  std::atomic<int> deactivate_count_{0};
  std::atomic<int> hold_init_count_{0};
  int last_snapshot_devices_{-1};
};

// Test-only friend accessor. Lives in rtc:: so it can call the private
// helpers and read the private state vector.
class ControllerLifecycleTestAccess {
public:
  static void
  InjectControllers(RtControllerNode &node,
                    std::vector<std::unique_ptr<RTControllerInterface>> ctrls) {
    node.controllers_ = std::move(ctrls);
    node.controller_states_ =
        std::vector<std::atomic<int>>(node.controllers_.size());
    node.controller_topic_configs_.assign(node.controllers_.size(), {});
    node.controller_slot_mappings_.assign(node.controllers_.size(), {});
    node.controller_name_to_idx_.clear();
    for (std::size_t i = 0; i < node.controllers_.size(); ++i) {
      node.controller_name_to_idx_[std::string(node.controllers_[i]->Name())] =
          static_cast<int>(i);
    }
    // Bypass startup auto-hold: set state_received_ true so SwitchActive
    // builds non-empty snapshots, but only when the test explicitly opts in
    // via SetStateReceived. Default: false (caller controls).
  }

  static void SetStateReceived(RtControllerNode &node, bool v) {
    node.state_received_.store(v, std::memory_order_release);
  }

  static void EnsureActivePublisher(RtControllerNode &node) {
    if (!node.active_ctrl_name_pub_) {
      node.active_ctrl_name_pub_ = node.create_publisher<std_msgs::msg::String>(
          "/" + node.robot_ns_ + "/active_controller_name", 1);
    }
  }

  static void SetActiveIdx(RtControllerNode &node, int idx) {
    node.active_controller_idx_.store(idx, std::memory_order_release);
  }

  static int GetState(const RtControllerNode &node, std::size_t idx) {
    return node.controller_states_[idx].load(std::memory_order_acquire);
  }

  static int GetActiveIdx(const RtControllerNode &node) {
    return node.active_controller_idx_.load(std::memory_order_acquire);
  }

  static RtControllerNode::CallbackReturn
  CallActivate(RtControllerNode &node, std::size_t idx,
               const rclcpp_lifecycle::State &prev,
               const ControllerState &snap) {
    return node.ActivateController(idx, prev, snap);
  }

  static RtControllerNode::CallbackReturn
  CallDeactivate(RtControllerNode &node, std::size_t idx,
                 const rclcpp_lifecycle::State &prev) {
    return node.DeactivateController(idx, prev);
  }

  static bool CallSwitch(RtControllerNode &node, const std::string &name,
                         std::string &message) {
    return node.SwitchActiveController(name, message);
  }

  static ControllerState BuildSnapshot(const RtControllerNode &node,
                                       std::size_t idx) {
    return node.BuildDeviceSnapshot(idx);
  }

  // Set the E-STOP flag directly — bypasses PublishEstopStatus() which
  // would touch estop_pub_ (uninitialised in the test fixture).
  static void SetEstopFlag(RtControllerNode &node, bool v) {
    node.global_estop_.store(v, std::memory_order_release);
  }
};

// ── Fixture ─────────────────────────────────────────────────────────────

class ControllerLifecycleTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    node_ = std::make_shared<RtControllerNode>("test_lifecycle_node");

    std::vector<std::unique_ptr<RTControllerInterface>> ctrls;
    auto a = std::make_unique<MockController>("ctrl_a");
    auto b = std::make_unique<MockController>("ctrl_b");
    ctrl_a_ = a.get();
    ctrl_b_ = b.get();
    ctrls.push_back(std::move(a));
    ctrls.push_back(std::move(b));
    ControllerLifecycleTestAccess::InjectControllers(*node_, std::move(ctrls));
    ControllerLifecycleTestAccess::EnsureActivePublisher(*node_);
    ControllerLifecycleTestAccess::SetActiveIdx(*node_, 0);
  }

  rclcpp_lifecycle::State InactiveState() const {
    return rclcpp_lifecycle::State(1, "inactive");
  }

  std::shared_ptr<RtControllerNode> node_;
  MockController *ctrl_a_{nullptr};
  MockController *ctrl_b_{nullptr};
};

// ── ActivateController / DeactivateController state transitions ─────────

TEST_F(ControllerLifecycleTest, ActivateFlipsStateAndCallsHook) {
  EXPECT_EQ(0, ControllerLifecycleTestAccess::GetState(*node_, 0));
  EXPECT_EQ(0, ctrl_a_->ActivateCount());

  ControllerState empty{};
  const auto rc = ControllerLifecycleTestAccess::CallActivate(
      *node_, 0, InactiveState(), empty);
  EXPECT_EQ(RTControllerInterface::CallbackReturn::SUCCESS, rc);
  EXPECT_EQ(1, ControllerLifecycleTestAccess::GetState(*node_, 0));
  EXPECT_EQ(1, ctrl_a_->ActivateCount());
  // Empty snapshot → base must skip InitializeHoldPosition.
  EXPECT_EQ(0, ctrl_a_->HoldInitCount());
  EXPECT_EQ(0, ctrl_a_->LastSnapshotDevices());
}

TEST_F(ControllerLifecycleTest, ActivateWithSnapshotTriggersHoldInit) {
  ControllerState snap{};
  snap.num_devices = 1;
  snap.devices[0].num_channels = 6;
  snap.devices[0].valid = true;
  const auto rc = ControllerLifecycleTestAccess::CallActivate(
      *node_, 1, InactiveState(), snap);
  EXPECT_EQ(RTControllerInterface::CallbackReturn::SUCCESS, rc);
  EXPECT_EQ(1, ControllerLifecycleTestAccess::GetState(*node_, 1));
  EXPECT_EQ(1, ctrl_b_->HoldInitCount());
  EXPECT_EQ(1, ctrl_b_->LastSnapshotDevices());
}

TEST_F(ControllerLifecycleTest, DeactivateFlipsStateBackToInactive) {
  ControllerState empty{};
  ControllerLifecycleTestAccess::CallActivate(*node_, 0, InactiveState(),
                                              empty);
  EXPECT_EQ(1, ControllerLifecycleTestAccess::GetState(*node_, 0));

  ControllerLifecycleTestAccess::CallDeactivate(*node_, 0, InactiveState());
  EXPECT_EQ(0, ControllerLifecycleTestAccess::GetState(*node_, 0));
  EXPECT_EQ(1, ctrl_a_->DeactivateCount());
}

// ── BuildDeviceSnapshot ─────────────────────────────────────────────────

TEST_F(ControllerLifecycleTest, BuildSnapshotEmptyWhenStateNotReceived) {
  ControllerLifecycleTestAccess::SetStateReceived(*node_, false);
  const auto snap = ControllerLifecycleTestAccess::BuildSnapshot(*node_, 0);
  EXPECT_EQ(0, snap.num_devices);
}

// ── SwitchActiveController preconditions ────────────────────────────────

TEST_F(ControllerLifecycleTest, SwitchRejectsUnknownName) {
  std::string msg;
  EXPECT_FALSE(
      ControllerLifecycleTestAccess::CallSwitch(*node_, "ctrl_xyz", msg));
  EXPECT_NE(std::string::npos, msg.find("Unknown"));
  // No state changes.
  EXPECT_EQ(0, ControllerLifecycleTestAccess::GetState(*node_, 0));
  EXPECT_EQ(0, ControllerLifecycleTestAccess::GetState(*node_, 1));
}

TEST_F(ControllerLifecycleTest, SwitchRejectsWhileEstopped) {
  ControllerLifecycleTestAccess::SetEstopFlag(*node_, true);
  std::string msg;
  EXPECT_FALSE(
      ControllerLifecycleTestAccess::CallSwitch(*node_, "ctrl_b", msg));
  EXPECT_NE(std::string::npos, msg.find("E-STOP"));
  // Recover for fixture teardown.
  ControllerLifecycleTestAccess::SetEstopFlag(*node_, false);
}

TEST_F(ControllerLifecycleTest, SwitchSameNameIsNoOpSuccess) {
  // Pre-arrange: ctrl_a is the active idx (set in SetUp), but state vector
  // says Inactive. Switch to "ctrl_a" should succeed (no-op) without
  // touching state vectors.
  std::string msg;
  EXPECT_TRUE(ControllerLifecycleTestAccess::CallSwitch(*node_, "ctrl_a", msg));
  EXPECT_EQ(0, ctrl_a_->ActivateCount());
  EXPECT_EQ(0, ctrl_a_->DeactivateCount());
}

// ── SwitchActiveController happy-path + N=20 cycles ─────────────────────

TEST_F(ControllerLifecycleTest, SwitchHappyPathFlipsActiveIdxAndStates) {
  // Arrange: initial state has ctrl_a (idx 0) marked Active in state vector
  // (matches CM's on_activate behavior for the initial controller).
  ControllerState empty{};
  ControllerLifecycleTestAccess::CallActivate(*node_, 0, InactiveState(),
                                              empty);
  EXPECT_EQ(1, ControllerLifecycleTestAccess::GetState(*node_, 0));

  // Switch to ctrl_b.
  std::string msg;
  EXPECT_TRUE(ControllerLifecycleTestAccess::CallSwitch(*node_, "ctrl_b", msg));
  EXPECT_EQ(1, ControllerLifecycleTestAccess::GetActiveIdx(*node_));
  EXPECT_EQ(0, ControllerLifecycleTestAccess::GetState(*node_, 0));
  EXPECT_EQ(1, ControllerLifecycleTestAccess::GetState(*node_, 1));
  EXPECT_EQ(1, ctrl_b_->ActivateCount());
  EXPECT_EQ(1, ctrl_a_->DeactivateCount());
}

TEST_F(ControllerLifecycleTest, RepeatedSwitchN20IsStable) {
  ControllerState empty{};
  ControllerLifecycleTestAccess::CallActivate(*node_, 0, InactiveState(),
                                              empty);
  std::string msg;
  // 20 alternating switches a → b → a → b → ...
  for (int i = 0; i < 20; ++i) {
    const std::string target = (i % 2 == 0) ? "ctrl_b" : "ctrl_a";
    EXPECT_TRUE(ControllerLifecycleTestAccess::CallSwitch(*node_, target, msg))
        << "iteration " << i << " message=" << msg;
  }
  // After 20 alternating switches starting from a: ends on a (10 to b, 10
  // back to a). Single-active invariant holds throughout.
  EXPECT_EQ(0, ControllerLifecycleTestAccess::GetActiveIdx(*node_));
  EXPECT_EQ(1, ControllerLifecycleTestAccess::GetState(*node_, 0));
  EXPECT_EQ(0, ControllerLifecycleTestAccess::GetState(*node_, 1));
  // 1 initial activate + 10 switches to b + 10 back to a → ctrl_a 11 acts,
  // ctrl_b 10 acts; deactivates: ctrl_a 10, ctrl_b 10.
  EXPECT_EQ(11, ctrl_a_->ActivateCount());
  EXPECT_EQ(10, ctrl_b_->ActivateCount());
  EXPECT_EQ(10, ctrl_a_->DeactivateCount());
  EXPECT_EQ(10, ctrl_b_->DeactivateCount());
}

} // namespace rtc
