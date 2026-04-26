// Phase 3 — /rtc_cm/list_controllers + /rtc_cm/switch_controller srv tests.
//
// Spins the CM node + a client node on a MultiThreadedExecutor, drives the
// services through the wire (not as direct method calls) and verifies:
//   - ListControllers returns one entry per loaded controller, with
//     state/type/claimed_groups derived from controller_states_ and the
//     controller registry.
//   - SwitchController happy-path flips the active idx and updates
//     controller_states_ + the latched /<robot_ns>/active_controller_name
//     topic.
//   - STRICT rejects multi-activate; BEST_EFFORT trims to first.
//   - Pure-deactivate (no activate target) is rejected.
//   - E-STOP active → ok=false, message contains "E-STOP".
//   - Unknown controller name → ok=false.
//
// Uses ControllerLifecycleTestAccess (defined in test_controller_lifecycle.cpp
// and re-declared here as a friend bridge) to inject MockController instances
// without going through the full on_configure pipeline.

#include "rtc_controller_manager/rt_controller_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rtc_msgs/srv/list_controllers.hpp>
#include <rtc_msgs/srv/switch_controller.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rtc {

// Minimal RTControllerInterface stub (mirrors test_controller_lifecycle.cpp's
// MockController). Lives in this TU so the test can link standalone — the
// other test's symbols live in a separate gtest binary.
class SrvMockController : public RTControllerInterface {
public:
  explicit SrvMockController(std::string name) : name_(std::move(name)) {}

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &prev,
              const ControllerState &snapshot) noexcept override {
    activate_count_.fetch_add(1, std::memory_order_relaxed);
    return RTControllerInterface::on_activate(prev, snapshot);
  }
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & /*prev*/) noexcept override {
    deactivate_count_.fetch_add(1, std::memory_order_relaxed);
    return CallbackReturn::SUCCESS;
  }
  ControllerOutput
  Compute(const ControllerState & /*state*/) noexcept override {
    return ControllerOutput{};
  }
  void SetDeviceTarget(int /*device_idx*/,
                       std::span<const double> /*target*/) noexcept override {}
  std::string_view Name() const noexcept override { return name_; }
  void
  InitializeHoldPosition(const ControllerState & /*state*/) noexcept override {}

  int ActivateCount() const {
    return activate_count_.load(std::memory_order_relaxed);
  }
  int DeactivateCount() const {
    return deactivate_count_.load(std::memory_order_relaxed);
  }

private:
  std::string name_;
  std::atomic<int> activate_count_{0};
  std::atomic<int> deactivate_count_{0};
};

// Friend bridge for srv tests. Same pattern as
// ControllerLifecycleTestAccess in test_controller_lifecycle.cpp — distinct
// class so the two test binaries link independently. Defined in the rtc::
// namespace so it matches the friend declaration in rt_controller_node.hpp.
class ControllerLifecycleTestAccess {
public:
  static void
  InjectControllers(RtControllerNode &node,
                    std::vector<std::unique_ptr<RTControllerInterface>> ctrls,
                    const std::vector<std::string> &types) {
    node.controllers_ = std::move(ctrls);
    node.controller_states_ =
        std::vector<std::atomic<int>>(node.controllers_.size());
    node.controller_topic_configs_.assign(node.controllers_.size(), {});
    node.controller_slot_mappings_.assign(node.controllers_.size(), {});
    node.controller_name_to_idx_.clear();
    node.controller_types_.clear();
    for (std::size_t i = 0; i < node.controllers_.size(); ++i) {
      node.controller_name_to_idx_[std::string(node.controllers_[i]->Name())] =
          static_cast<int>(i);
      node.controller_types_.push_back(i < types.size() ? types[i] : "");
    }
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

  // Mark a controller Active in the state vector — mirrors what CM's
  // on_activate does for the initial controller.
  static void SetState(RtControllerNode &node, std::size_t idx, int v) {
    node.controller_states_[idx].store(v, std::memory_order_release);
  }

  static int GetState(const RtControllerNode &node, std::size_t idx) {
    return node.controller_states_[idx].load(std::memory_order_acquire);
  }

  static int GetActiveIdx(const RtControllerNode &node) {
    return node.active_controller_idx_.load(std::memory_order_acquire);
  }

  static void SetEstopFlag(RtControllerNode &node, bool v) {
    node.global_estop_.store(v, std::memory_order_release);
  }

  // Bind cb_group_aux_ + bring the services online without driving the full
  // lifecycle. CM's CreateServices() requires cb_group_aux_ already set.
  static void BringServicesOnline(RtControllerNode &node) {
    if (!node.cb_group_aux_) {
      node.cb_group_aux_ = node.create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    node.CreateServices();
  }
};

// ── Fixture ─────────────────────────────────────────────────────────────

class SwitchServiceTest : public ::testing::Test {
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
    node_ = std::make_shared<RtControllerNode>("test_switch_node");

    std::vector<std::unique_ptr<RTControllerInterface>> ctrls;
    auto a = std::make_unique<SrvMockController>("ctrl_a");
    auto b = std::make_unique<SrvMockController>("ctrl_b");
    ctrl_a_ = a.get();
    ctrl_b_ = b.get();
    ctrls.push_back(std::move(a));
    ctrls.push_back(std::move(b));

    ControllerLifecycleTestAccess::InjectControllers(*node_, std::move(ctrls),
                                                     {"type_a", "type_b"});
    ControllerLifecycleTestAccess::EnsureActivePublisher(*node_);
    ControllerLifecycleTestAccess::SetActiveIdx(*node_, 0);
    // Mark ctrl_a Active so the switch happy-path has a real prev to
    // deactivate.
    ControllerLifecycleTestAccess::SetState(*node_, 0, 1);
    ControllerLifecycleTestAccess::BringServicesOnline(*node_);

    client_node_ = std::make_shared<rclcpp::Node>("test_switch_client");
    list_client_ = client_node_->create_client<rtc_msgs::srv::ListControllers>(
        "/rtc_cm/list_controllers");
    switch_client_ =
        client_node_->create_client<rtc_msgs::srv::SwitchController>(
            "/rtc_cm/switch_controller");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    // Wait for service discovery (server side is local — should be fast).
    ASSERT_TRUE(list_client_->wait_for_service(std::chrono::seconds(2)))
        << "list_controllers service did not appear";
    ASSERT_TRUE(switch_client_->wait_for_service(std::chrono::seconds(2)))
        << "switch_controller service did not appear";
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  // Helper: send a SwitchController request and wait synchronously for the
  // response (executor is already spinning on its own thread).
  rtc_msgs::srv::SwitchController::Response::SharedPtr CallSwitch(
      const std::vector<std::string> &activate,
      const std::vector<std::string> &deactivate,
      int32_t strictness = rtc_msgs::srv::SwitchController::Request::STRICT) {
    auto req = std::make_shared<rtc_msgs::srv::SwitchController::Request>();
    req->activate_controllers = activate;
    req->deactivate_controllers = deactivate;
    req->strictness = strictness;
    req->timeout.sec = 1;
    req->timeout.nanosec = 0;
    auto fut = switch_client_->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      return nullptr;
    }
    return fut.get();
  }

  rtc_msgs::srv::ListControllers::Response::SharedPtr CallList() {
    auto req = std::make_shared<rtc_msgs::srv::ListControllers::Request>();
    auto fut = list_client_->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      return nullptr;
    }
    return fut.get();
  }

  std::shared_ptr<RtControllerNode> node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Client<rtc_msgs::srv::ListControllers>::SharedPtr list_client_;
  rclcpp::Client<rtc_msgs::srv::SwitchController>::SharedPtr switch_client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  SrvMockController *ctrl_a_{nullptr};
  SrvMockController *ctrl_b_{nullptr};
};

// ── ListControllers ─────────────────────────────────────────────────────

TEST_F(SwitchServiceTest, ListControllersReturnsAllWithCorrectStateAndType) {
  auto resp = CallList();
  ASSERT_NE(nullptr, resp);
  ASSERT_EQ(2u, resp->controllers.size());

  // Order matches controllers_ insertion order.
  EXPECT_EQ("ctrl_a", resp->controllers[0].name);
  EXPECT_EQ("active", resp->controllers[0].state);
  EXPECT_TRUE(resp->controllers[0].is_active);
  EXPECT_EQ("type_a", resp->controllers[0].type);

  EXPECT_EQ("ctrl_b", resp->controllers[1].name);
  EXPECT_EQ("inactive", resp->controllers[1].state);
  EXPECT_FALSE(resp->controllers[1].is_active);
  EXPECT_EQ("type_b", resp->controllers[1].type);
}

TEST_F(SwitchServiceTest, ListControllersReflectsSwitch) {
  // Switch to ctrl_b, then list should report ctrl_b active.
  auto sw_resp = CallSwitch({"ctrl_b"}, {});
  ASSERT_NE(nullptr, sw_resp);
  ASSERT_TRUE(sw_resp->ok) << sw_resp->message;

  auto list_resp = CallList();
  ASSERT_NE(nullptr, list_resp);
  ASSERT_EQ(2u, list_resp->controllers.size());
  EXPECT_EQ("inactive", list_resp->controllers[0].state);
  EXPECT_FALSE(list_resp->controllers[0].is_active);
  EXPECT_EQ("active", list_resp->controllers[1].state);
  EXPECT_TRUE(list_resp->controllers[1].is_active);
}

// ── SwitchController happy path ─────────────────────────────────────────

TEST_F(SwitchServiceTest, SwitchHappyPathFlipsActiveIdx) {
  auto resp = CallSwitch({"ctrl_b"}, {});
  ASSERT_NE(nullptr, resp);
  EXPECT_TRUE(resp->ok) << resp->message;
  EXPECT_EQ(1, ControllerLifecycleTestAccess::GetActiveIdx(*node_));
  EXPECT_EQ(0, ControllerLifecycleTestAccess::GetState(*node_, 0));
  EXPECT_EQ(1, ControllerLifecycleTestAccess::GetState(*node_, 1));
  EXPECT_EQ(1, ctrl_b_->ActivateCount());
  EXPECT_EQ(1, ctrl_a_->DeactivateCount());
}

TEST_F(SwitchServiceTest, SwitchEmptyRequestIsNoOpSuccess) {
  auto resp = CallSwitch({}, {});
  ASSERT_NE(nullptr, resp);
  EXPECT_TRUE(resp->ok);
  EXPECT_NE(std::string::npos, resp->message.find("no-op"));
  // No state changes.
  EXPECT_EQ(0, ctrl_a_->ActivateCount());
  EXPECT_EQ(0, ctrl_b_->ActivateCount());
}

// ── SwitchController validation ─────────────────────────────────────────

TEST_F(SwitchServiceTest, SwitchUnknownNameRejected) {
  auto resp = CallSwitch({"ctrl_xyz"}, {});
  ASSERT_NE(nullptr, resp);
  EXPECT_FALSE(resp->ok);
  EXPECT_NE(std::string::npos, resp->message.find("Unknown"));
  EXPECT_EQ(0, ControllerLifecycleTestAccess::GetActiveIdx(*node_));
}

TEST_F(SwitchServiceTest, SwitchStrictRejectsMultiActivate) {
  auto resp = CallSwitch({"ctrl_a", "ctrl_b"}, {});
  ASSERT_NE(nullptr, resp);
  EXPECT_FALSE(resp->ok);
  EXPECT_NE(std::string::npos, resp->message.find("Single-active"));
  // No state changes.
  EXPECT_EQ(0, ctrl_b_->ActivateCount());
}

TEST_F(SwitchServiceTest, SwitchBestEffortTrimsMultiActivate) {
  auto resp = CallSwitch({"ctrl_b", "ctrl_a"}, {},
                         rtc_msgs::srv::SwitchController::Request::BEST_EFFORT);
  ASSERT_NE(nullptr, resp);
  EXPECT_TRUE(resp->ok) << resp->message;
  // Only the first entry (ctrl_b) is honoured.
  EXPECT_EQ(1, ControllerLifecycleTestAccess::GetActiveIdx(*node_));
  EXPECT_EQ(1, ctrl_b_->ActivateCount());
}

TEST_F(SwitchServiceTest, SwitchPureDeactivateRejected) {
  auto resp = CallSwitch({}, {"ctrl_a"});
  ASSERT_NE(nullptr, resp);
  EXPECT_FALSE(resp->ok);
  EXPECT_NE(std::string::npos, resp->message.find("Pure deactivate"));
}

TEST_F(SwitchServiceTest, SwitchRejectedWhileEstopped) {
  ControllerLifecycleTestAccess::SetEstopFlag(*node_, true);
  auto resp = CallSwitch({"ctrl_b"}, {});
  ASSERT_NE(nullptr, resp);
  EXPECT_FALSE(resp->ok);
  EXPECT_NE(std::string::npos, resp->message.find("E-STOP"));
  // Recover so teardown does not leak the flag.
  ControllerLifecycleTestAccess::SetEstopFlag(*node_, false);
}

} // namespace rtc
