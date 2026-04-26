/// Unit tests for SwitchController action node.

#include "test_helpers.hpp"
#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <rtc_msgs/srv/switch_controller.hpp>

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

  BT::Tree CreateTree(const std::string &xml) {
    const std::string full = R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" +
                             xml + R"(</BehaviorTree></root>)";
    return factory_.createTreeFromText(full);
  }

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

class SwitchControllerSrvTest : public SwitchControllerTest {
protected:
  using SwitchSrv = rtc_msgs::srv::SwitchController;

  void SetUp() override {
    SwitchControllerTest::SetUp();
    srv_node_ = std::make_shared<rclcpp::Node>("srv_mock_node");
    handler_ = [](const SwitchSrv::Request::SharedPtr,
                  SwitchSrv::Response::SharedPtr resp) {
      resp->ok = true;
      resp->message = "ok";
    };
    srv_server_ = srv_node_->create_service<SwitchSrv>(
        "/rtc_cm/switch_controller",
        [this](const SwitchSrv::Request::SharedPtr req,
               SwitchSrv::Response::SharedPtr resp) { handler_(req, resp); });
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

  void SetSwitchHandler(std::function<void(const SwitchSrv::Request::SharedPtr,
                                           SwitchSrv::Response::SharedPtr)>
                            handler) {
    handler_ = std::move(handler);
  }

  std::shared_ptr<rclcpp::Node> srv_node_;
  rclcpp::Service<SwitchSrv>::SharedPtr srv_server_;
  std::atomic<bool> srv_spin_running_{false};
  std::thread srv_spin_thread_;
  std::function<void(const SwitchSrv::Request::SharedPtr,
                     SwitchSrv::Response::SharedPtr)>
      handler_;
};

TEST_F(SwitchControllerSrvTest, SrvSwitchSucceedsImmediately) {
  PublishActiveController("old_controller");
  Spin();

  std::string captured_target;
  SetSwitchHandler([&captured_target](const SwitchSrv::Request::SharedPtr req,
                                      SwitchSrv::Response::SharedPtr resp) {
    if (!req->activate_controllers.empty()) {
      captured_target = req->activate_controllers.front();
    }
    resp->ok = true;
    resp->message = "switched -> " + captured_target;
  });

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_joint_controller"
                          timeout_s="2.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(captured_target, "demo_joint_controller");
}

TEST_F(SwitchControllerSrvTest, SrvRejectionPropagatesAsFailure) {
  PublishActiveController("old_controller");
  Spin();

  SetSwitchHandler([](const SwitchSrv::Request::SharedPtr,
                      SwitchSrv::Response::SharedPtr resp) {
    resp->ok = false;
    resp->message = "E-STOP active";
  });

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_joint_controller"
                          timeout_s="2.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}
