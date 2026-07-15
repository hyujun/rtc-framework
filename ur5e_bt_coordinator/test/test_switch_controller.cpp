/// Unit tests for SwitchController action node.

#include "test_helpers.hpp"
#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"
#include <rtc_msgs/srv/switch_controller.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <atomic>
#include <functional>
#include <thread>

using namespace rtc_bt;
using namespace rtc_bt::test;

class SwitchControllerTest : public RosTestFixture {
 protected:
  void SetUp() override {
    RosTestFixture::SetUp();
    factory_.registerNodeType<SwitchController>("SwitchController", bridge_);
  }

  BT::Tree CreateTree(const std::string& xml) {
    const std::string full =
        R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" + xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

  // TickUntilComplete (poll a StatefulActionNode across ticks) lives in
  // test_helpers.hpp — shared with test_set_gains.

  BT::BehaviorTreeFactory factory_;
};

TEST_F(SwitchControllerTest, AlreadyActive) {
  // Fixture defaults to active="demo_task_controller". Re-target same name.
  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_task_controller"
                          timeout_s="3.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SwitchControllerTest, NormalizedNameComparison) {
  // Underscore + casing normalisation: "DemoTaskController" should match
  // the active "demo_task_controller".
  auto tree = CreateTree(
      R"(<SwitchController controller_name="DemoTaskController"
                          timeout_s="3.0"/>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// ── srv-path tests (mock /rtc_cm/switch_controller server) ────────────────
//
// SwitchControllerSrvTest spins a mock /rtc_cm/switch_controller server
// alongside the bridge. Each test calls SetSwitchHandler() to configure
// what the server returns, then ticks the BT node.
//
// A handler that answers ok MUST also latch the name, because that is the real
// CM's contract (commit the swap → latch /rtc_cm/active_controller_name →
// answer ok) and SwitchController now waits for the latched name to reach the
// bridge before reporting SUCCESS. A handler that answers ok without latching
// models a broken CM, and SrvOkWithoutRewireTimesOut asserts we reject it.

class SwitchControllerSrvTest : public SwitchControllerTest {
 protected:
  using SwitchSrv = rtc_msgs::srv::SwitchController;

  void SetUp() override {
    SwitchControllerTest::SetUp();
    srv_node_ = std::make_shared<rclcpp::Node>("srv_mock_node");
    handler_ = [this](const SwitchSrv::Request::SharedPtr req,
                      SwitchSrv::Response::SharedPtr resp) {
      // Default: a well-behaved CM — latch the committed name, then answer ok.
      if (!req->activate_controllers.empty()) {
        LatchActiveController(req->activate_controllers.front());
      }
      resp->ok = true;
      resp->message = "ok";
    };
    srv_server_ = srv_node_->create_service<SwitchSrv>(
        "/rtc_cm/switch_controller",
        [this](const SwitchSrv::Request::SharedPtr req, SwitchSrv::Response::SharedPtr resp) {
          handler_(req, resp);
        });
    srv_spin_running_.store(true);
    srv_spin_thread_ = std::thread([this]() {
      while (srv_spin_running_.load()) {
        rclcpp::spin_some(srv_node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void TearDown() override {
    srv_spin_running_.store(false);
    if (srv_spin_thread_.joinable()) {
      srv_spin_thread_.join();
    }
    srv_server_.reset();
    srv_node_.reset();
    SwitchControllerTest::TearDown();
  }

  void SetSwitchHandler(
      std::function<void(const SwitchSrv::Request::SharedPtr, SwitchSrv::Response::SharedPtr)>
          handler) {
    handler_ = std::move(handler);
  }

  std::shared_ptr<rclcpp::Node> srv_node_;
  rclcpp::Service<SwitchSrv>::SharedPtr srv_server_;
  std::atomic<bool> srv_spin_running_{false};
  std::thread srv_spin_thread_;
  std::function<void(const SwitchSrv::Request::SharedPtr, SwitchSrv::Response::SharedPtr)> handler_;
};

TEST_F(SwitchControllerSrvTest, SrvSwitchSucceedsImmediately) {
  PublishActiveController("old_controller");
  Spin();

  std::string captured_target;
  SetSwitchHandler([this, &captured_target](const SwitchSrv::Request::SharedPtr req,
                                            SwitchSrv::Response::SharedPtr resp) {
    if (!req->activate_controllers.empty()) {
      captured_target = req->activate_controllers.front();
      LatchActiveController(captured_target);
    }
    resp->ok = true;
    resp->message = "switched -> " + captured_target;
  });

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_joint_controller"
                          timeout_s="2.0"/>)");

  EXPECT_EQ(TickUntilComplete(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(captured_target, "demo_joint_controller");
  // SUCCESS must mean the bridge is reachable at the new controller, not just
  // that CM answered — this is what a following SetGains relies on (#158).
  EXPECT_EQ(bridge_->GetActiveController(), "demo_joint_controller");
}

TEST_F(SwitchControllerSrvTest, SrvOkWithoutRewireTimesOut) {
  PublishActiveController("old_controller");
  Spin();

  // A CM that commits the swap and answers ok, but whose latched name never
  // reaches this node's bridge (dropped delivery / participant churn) — the
  // #158 gap. Reporting SUCCESS here is what let a following SetGains build
  // params from the stale name and configure "old_controller" instead.
  SetSwitchHandler([](const SwitchSrv::Request::SharedPtr, SwitchSrv::Response::SharedPtr resp) {
    resp->ok = true;
    resp->message = "ok";
  });

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_joint_controller"
                          timeout_s="0.5"/>)");

  EXPECT_EQ(TickUntilComplete(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(bridge_->GetActiveController(), "old_controller");
}

TEST_F(SwitchControllerSrvTest, SrvRejectionPropagatesAsFailure) {
  PublishActiveController("old_controller");
  Spin();

  SetSwitchHandler([](const SwitchSrv::Request::SharedPtr, SwitchSrv::Response::SharedPtr resp) {
    resp->ok = false;
    resp->message = "E-STOP active";
  });

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_joint_controller"
                          timeout_s="2.0"/>)");

  EXPECT_EQ(TickUntilComplete(tree), BT::NodeStatus::FAILURE);
}
