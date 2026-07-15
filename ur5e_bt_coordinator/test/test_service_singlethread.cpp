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
/// terminal status, or RUNNING if a budget expires first (the deadlock
/// signature). No node is spun on any other thread — the lone `guard` thread
/// only trips the executor's thread-safe cancel() on timeout.
///
/// TWO clocks, because the wait before the first tick and the wait for the tree
/// to finish are different phenomena and only the second one is what this file
/// regression-tests (issue #160):
///
///   - `discovery_budget` runs from spin() until `ready` first holds. That span
///     is pure DDS: the latched /rtc_cm/active_controller_name has to be
///     discovered and delivered, and the bridge has to rewire on it. It stretches
///     under participant churn and says nothing about deadlock, so it gets a
///     budget generous enough to never be the thing that fails.
///   - `deadlock_budget` runs from that moment on, and keeps the old 8s meaning:
///     the tree is being ticked and must reach a terminal status. A blocking
///     service wait shows up here, and only here.
///
/// Sharing one 8s wall clock across both is what made this flake: discovery ate
/// the budget, the guard fired before the tree had ticked its way to an answer,
/// and the result was indistinguishable from a deadlock. Each overrun now
/// reports its own message so a failure says which clock ran out.
BT::NodeStatus SpinTickToCompletion(
    const rclcpp::Node::SharedPtr& ticker, rclcpp::executors::SingleThreadedExecutor& exec,
    BT::Tree& tree, const std::function<bool()>& ready = [] { return true; },
    std::chrono::milliseconds deadlock_budget = std::chrono::seconds(8),
    std::chrono::milliseconds discovery_budget = std::chrono::seconds(30)) {
  std::atomic<int> status{static_cast<int>(BT::NodeStatus::RUNNING)};
  std::atomic<bool> done{false};
  std::atomic<bool> ready_seen{false};
  auto timer = ticker->create_wall_timer(std::chrono::milliseconds(20), [&]() {
    if (done.load() || !ready())
      return;
    ready_seen.store(true);
    const auto s = tree.tickOnce();
    if (s != BT::NodeStatus::RUNNING) {
      status.store(static_cast<int>(s));
      done.store(true);
      exec.cancel();
    }
  });
  std::atomic<bool> deadlock_expired{false};
  std::thread guard([&]() {
    // Start on the discovery clock; switch to the deadlock clock the moment the
    // timer reports `ready` held. Polling at 5ms bounds the handover error.
    auto deadline = std::chrono::steady_clock::now() + discovery_budget;
    bool ticking = false;
    while (!done.load()) {
      if (!ticking && ready_seen.load()) {
        ticking = true;
        deadline = std::chrono::steady_clock::now() + deadlock_budget;
      }
      if (std::chrono::steady_clock::now() > deadline) {
        deadlock_expired.store(ticking);
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

  const auto final_status = static_cast<BT::NodeStatus>(status.load());
  if (final_status == BT::NodeStatus::RUNNING) {
    if (deadlock_expired.load()) {
      ADD_FAILURE() << "deadlock budget (" << deadlock_budget.count()
                    << "ms) expired: the tree was being ticked but never reached a terminal "
                       "status — a service wait that only this executor could fulfil";
    } else {
      ADD_FAILURE() << "discovery budget (" << discovery_budget.count()
                    << "ms) expired: `ready` never held, so the tree was never ticked — DDS "
                       "discovery/rewire, not a deadlock";
    }
  }
  return final_status;
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
  auto active_ctrl_pub = srv_node->create_publisher<std_msgs::msg::String>(
      "/rtc_cm/active_controller_name", rclcpp::QoS{1}.transient_local());
  auto srv = srv_node->create_service<rtc_msgs::srv::SwitchController>(
      "/rtc_cm/switch_controller",
      [active_ctrl_pub](const rtc_msgs::srv::SwitchController::Request::SharedPtr req,
                        rtc_msgs::srv::SwitchController::Response::SharedPtr resp) {
        // Mirror CM's ordering: commit the swap by latching the name, then
        // answer ok. SwitchController's stage 2 then waits for the bridge to
        // consume that name — under this one executor the delivery can only
        // happen on a later spin iteration, interleaved with the ticks, which
        // is precisely the structure this file regression-tests.
        if (!req->activate_controllers.empty()) {
          std_msgs::msg::String msg;
          msg.data = req->activate_controllers.front();
          active_ctrl_pub->publish(msg);
        }
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
  // fires at 8s → RUNNING. The async node polls the future across ticks, then
  // polls for the bridge's rewire across further ticks → SUCCESS. Both waits
  // are fulfilled by the same executor the ticks run on, so a blocking wait in
  // either stage would show up here as RUNNING.
  EXPECT_EQ(SpinTickToCompletion(ticker, exec, tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bridge->GetActiveController(), "demo_joint_controller");
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
