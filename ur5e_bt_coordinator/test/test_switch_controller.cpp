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

TEST_F(SwitchControllerTest, AlreadyActiveNoGains) {
  PublishActiveController("demo_joint_controller");
  Spin();

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_joint_controller"
                          load_gains="false" timeout_s="3.0"/>)");
  // Already active + no gains → immediate SUCCESS
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SwitchControllerTest, AlreadyActiveWithGainsLoading) {
  PublishActiveController("demo_task_controller");
  Spin();

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_task_controller"
                          load_gains="true" timeout_s="3.0"
                          current_gains="{cg}"/>)");

  // Already active + load_gains=true → RUNNING (waiting for gains)
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Publish gains
  PublishCurrentGains({1.0, 2.0, 3.0});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto gains = tree.rootBlackboard()->get<std::vector<double>>("cg");
  ASSERT_EQ(gains.size(), 3u);
  EXPECT_DOUBLE_EQ(gains[0], 1.0);
}

TEST_F(SwitchControllerTest, NormalizedNameComparison) {
  // Test that "DemoJointController" matches "demo_joint_controller"
  PublishActiveController("DemoJointController");
  Spin();

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_joint_controller"
                          load_gains="false" timeout_s="3.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(SwitchControllerTest, GainsTimeoutStillSucceeds) {
  // If gains timeout, SwitchController still returns SUCCESS (warns but
  // continues)
  PublishActiveController("demo_task_controller");
  Spin();

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_task_controller"
                          load_gains="true" timeout_s="0.05"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // Don't publish gains, just wait for timeout
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// ── Phase 4: srv-path tests ───────────────────────────────────────────────
//
// SwitchControllerSrvTest spins a mock /rtc_cm/switch_controller server
// alongside the bridge. Each test calls SetSwitchHandler() to configure
// what the server returns, then ticks the BT node. Because the bridge's
// underlying srv call uses async_send_request + future.wait_for, the test
// has to spin both the bridge node (already done in fixture) and the
// server node so that the request/response round-trips. A dedicated spin
// thread on the server node keeps the response unblocked while the BT
// tick is awaiting the future.

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
    // Background spin loop drives both the mock server and the bridge
    // node. Using spin_some on each avoids add_node ownership conflicts
    // with the parent fixture's Spin() helper, which transiently binds
    // node_ to an ephemeral executor.
    spin_running_.store(true);
    spin_thread_ = std::thread([this]() {
      while (spin_running_.load()) {
        rclcpp::spin_some(srv_node_);
        rclcpp::spin_some(node_->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    // Allow service discovery to settle before any test invokes the bridge.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void TearDown() override {
    spin_running_.store(false);
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    srv_server_.reset();
    srv_node_.reset();
    SwitchControllerTest::TearDown();
  }

  // Shadow parent's Spin() so test bodies don't call spin_some on node_
  // concurrently with the background spin loop (which would trip rclcpp's
  // "already added to an executor" guard). The background thread already
  // drains both nodes — a sleep is enough to let messages flow.
  void
  Spin(std::chrono::milliseconds duration = std::chrono::milliseconds(50)) {
    std::this_thread::sleep_for(duration);
  }

  void SetSwitchHandler(std::function<void(const SwitchSrv::Request::SharedPtr,
                                           SwitchSrv::Response::SharedPtr)>
                            handler) {
    handler_ = std::move(handler);
  }

  std::shared_ptr<rclcpp::Node> srv_node_;
  rclcpp::Service<SwitchSrv>::SharedPtr srv_server_;
  std::atomic<bool> spin_running_{false};
  std::thread spin_thread_;
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
                          load_gains="false" timeout_s="2.0"/>)");

  // Default use_service=true → single tick returns SUCCESS once the srv
  // call resolves. tickOnce drives onStart which blocks on the srv future
  // (server runs on srv_executor_).
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
                          load_gains="false" timeout_s="2.0"/>)");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(SwitchControllerSrvTest, SrvSuccessThenLoadGainsRunsUntilGainsArrive) {
  PublishActiveController("old_controller");
  Spin();

  SetSwitchHandler([](const SwitchSrv::Request::SharedPtr,
                      SwitchSrv::Response::SharedPtr resp) {
    resp->ok = true;
    resp->message = "ok";
  });

  auto tree = CreateTree(
      R"(<SwitchController controller_name="demo_task_controller"
                          load_gains="true" timeout_s="2.0"
                          current_gains="{cg}"/>)");

  // Srv resolves inside onStart → switch_confirmed_ + gains_requested_;
  // returns RUNNING while waiting for /current_gains.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  PublishCurrentGains({0.5, 1.5, 2.5});
  Spin();

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto gains = tree.rootBlackboard()->get<std::vector<double>>("cg");
  ASSERT_EQ(gains.size(), 3u);
  EXPECT_DOUBLE_EQ(gains[0], 0.5);
}
