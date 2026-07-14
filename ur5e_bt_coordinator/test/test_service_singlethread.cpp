/// Production-thread-structure regression for the async service nodes.
///
/// main.cpp runs a SINGLE-threaded executor (`rclcpp::spin`) that ticks the BT
/// from a wall-timer callback. The former *blocking* SwitchController / SetGains
/// implementations deadlocked in exactly that structure: a tick waited on a
/// service future that only the SAME (now busy) executor could fulfil, so the
/// response was never dispatched.
///
/// These tests reproduce that structure — one executor owns the bridge, the
/// mock servers, AND the ticker timer, with NO separate spin thread — and
/// assert the async StatefulAction nodes complete. Under the old blocking code
/// they would hang until the wall-clock guard fired (→ RUNNING, i.e. FAILURE of
/// the EXPECT). The other Tier-2 fixtures mask this because they spin the bridge
/// on a dedicated background thread; here the tick and the response share one
/// thread, just like production.

#include "ur5e_bt_coordinator/action_nodes/set_gains.hpp"
#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"
#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include <rtc_msgs/srv/grasp_command.hpp>
#include <rtc_msgs/srv/switch_controller.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

using namespace rtc_bt;

namespace {

/// Drive one executor that owns every node and ticks `tree` from a wall timer
/// (main.cpp structure). Ticking is gated on `ready` so a test can wait for a
/// latched active-controller message to land before the first tick. Returns the
/// terminal status, or RUNNING if the wall-clock budget expires first (the
/// deadlock signature). No node is spun on any other thread — the lone `guard`
/// thread only trips the executor's thread-safe cancel() on timeout.
BT::NodeStatus SpinTickToCompletion(
    const rclcpp::Node::SharedPtr& ticker, rclcpp::executors::SingleThreadedExecutor& exec,
    BT::Tree& tree, const std::function<bool()>& ready = [] { return true; },
    std::chrono::milliseconds budget = std::chrono::seconds(8)) {
  std::atomic<int> status{static_cast<int>(BT::NodeStatus::RUNNING)};
  std::atomic<bool> done{false};
  auto timer = ticker->create_wall_timer(std::chrono::milliseconds(20), [&]() {
    if (done.load() || !ready())
      return;
    const auto s = tree.tickOnce();
    if (s != BT::NodeStatus::RUNNING) {
      status.store(static_cast<int>(s));
      done.store(true);
      exec.cancel();
    }
  });
  std::thread guard([&]() {
    const auto start = std::chrono::steady_clock::now();
    while (!done.load()) {
      if (std::chrono::steady_clock::now() - start > budget) {
        exec.cancel();
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });
  exec.spin();
  done.store(true);
  guard.join();
  timer.reset();
  return static_cast<BT::NodeStatus>(status.load());
}

void EnsureRos() {
  // Shared context for the whole binary (never shutdown per-test — repeated
  // init/shutdown churns FastDDS discovery, the #154 flake source).
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
}

}  // namespace

TEST(ServiceSingleThread, SwitchControllerCompletesUnderSharedExecutor) {
  EnsureRos();

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("bt_st_switch_node");
  auto bridge = std::make_shared<BtRosBridge>(node);

  auto srv_node = std::make_shared<rclcpp::Node>("srv_mock_switch_st");
  auto srv = srv_node->create_service<rtc_msgs::srv::SwitchController>(
      "/rtc_cm/switch_controller", [](const rtc_msgs::srv::SwitchController::Request::SharedPtr,
                                      rtc_msgs::srv::SwitchController::Response::SharedPtr resp) {
        resp->ok = true;
        resp->message = "switched";
      });

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SwitchController>("SwitchController", bridge);
  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="T">
           <SwitchController controller_name="demo_joint_controller" timeout_s="5.0"/>
         </BehaviorTree></root>)");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(srv_node);
  auto ticker = std::make_shared<rclcpp::Node>("ticker_switch_st");
  exec.add_node(ticker);

  // Under the old blocking RequestSwitchController this deadlocks and the guard
  // fires at 8s → RUNNING. The async node polls the future across ticks → SUCCESS.
  EXPECT_EQ(SpinTickToCompletion(ticker, exec, tree), BT::NodeStatus::SUCCESS);
}

TEST(ServiceSingleThread, SetGainsGraspCompletesUnderSharedExecutor) {
  EnsureRos();

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("bt_st_gains_node");
  auto bridge = std::make_shared<BtRosBridge>(node);

  // Mock controller exposing the grasp_command srv the active controller owns.
  const std::string ctrl = "demo_joint_controller";
  auto ctrl_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      ctrl, "/" + ctrl, rclcpp::NodeOptions().use_global_arguments(false));
  std::atomic<int> grasp_calls{0};
  auto grasp_srv = ctrl_node->create_service<rtc_msgs::srv::GraspCommand>(
      "grasp_command", [&grasp_calls](const rtc_msgs::srv::GraspCommand::Request::SharedPtr,
                                      rtc_msgs::srv::GraspCommand::Response::SharedPtr resp) {
        grasp_calls.fetch_add(1);
        resp->ok = true;
        resp->message = "grasped";
      });

  // Latched active-controller publisher: the bridge rebinds grasp_command_client_
  // to /demo_joint_controller/grasp_command when it processes this.
  auto ticker = std::make_shared<rclcpp::Node>("ticker_gains_st");
  auto active_pub = ticker->create_publisher<std_msgs::msg::String>(
      "/rtc_cm/active_controller_name", rclcpp::QoS{1}.transient_local());
  std_msgs::msg::String active_msg;
  active_msg.data = ctrl;
  active_pub->publish(active_msg);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SetGains>("SetGains", bridge);
  // Grasp-only (no gain ports) → exercises the async grasp_command stage.
  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="T">
           <SetGains grasp_command="2"/>
         </BehaviorTree></root>)");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(ctrl_node->get_node_base_interface());
  exec.add_node(ticker);

  // Only start ticking once the bridge has adopted the active controller, so
  // BuildParams resolves demo_joint_controller (not the empty default).
  const auto status = SpinTickToCompletion(
      ticker, exec, tree, [&bridge, &ctrl]() { return bridge->GetActiveController() == ctrl; });

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_GE(grasp_calls.load(), 1);
}
