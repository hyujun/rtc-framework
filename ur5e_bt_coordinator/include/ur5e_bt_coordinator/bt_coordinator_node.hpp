#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/failure_logger.hpp"

#include <behaviortree_cpp/bt_factory.h>

#if __has_include(<behaviortree_cpp/loggers/groot2_publisher.h>)
#define BT_GROOT2_AVAILABLE 1
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#endif

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace rtc_bt {

namespace test {
struct CoordinatorTickInjector;  // test-only tick-path seam (test/test_rewire_gate.cpp, issue #158)
}  // namespace test

/// Main ROS2 node that ticks a BehaviorTree at a configurable rate.
///
/// Features:
///   - Loads BT XML from the `trees/` directory (supports absolute paths)
///   - Registers all custom BT nodes (action + condition)
///   - Shares a BtRosBridge instance with all BT nodes
///   - Ticks the tree at `tick_rate_hz` (default 20 Hz)
///   - Runtime tree switching via `tree_file` parameter change
///   - Blackboard variable initialization from `bb.*` parameters
///   - Groot2 monitoring via `groot2_port` parameter
///   - Topic health watchdog with configurable timeout
///   - Step-by-step debugging mode
///   - Failure diagnosis logging
///
/// Usage:
///   auto node = std::make_shared<BtCoordinatorNode>(options);
///   // Launch event handler triggers configure/activate automatically.
///   rclcpp::spin(node->get_node_base_interface());
class BtCoordinatorNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit BtCoordinatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

 private:
  // Test-only seam (issue #158). Grants read access to bridge_ / tree_ so a
  // test can drive the real on_activate → TickCallback path and inject the
  // active-controller transition through the bridge's handler without DDS.
  // Mirrors the BridgeStateInjector seam (issue #154): observation only — the
  // seam never pokes fields or calls private transitions.
  friend struct test::CoordinatorTickInjector;

  void DeclareParameters();
  void RegisterBtNodes();
  void LoadTree();
  void TickCallback();

  /// Why a tick cannot run right now, or kNone if it can.
  ///
  /// Callers that only need "may I tick?" compare against kNone; the specific
  /// value exists because the blockers are not equivalent. kEstopped is a
  /// healthy, expected state, while kControllerUnwired is only healthy for the
  /// first moments after activation — TickCallback escalates that one once it
  /// outlasts controller_wait_timeout_s_.
  enum class TickBlocker { kNone, kNoTree, kEstopped, kControllerUnwired };

  /// Preconditions every tick must satisfy, shared by both tick paths: the
  /// timer (TickCallback) and the ~/step service (StepCallback). Ticking a
  /// tree that fails any of these is what issue #158 was — a gate on one path
  /// only leaves the other reachable.
  ///
  /// `paused_` is deliberately NOT checked here: it gates *auto*-ticking, so a
  /// manual ~/step is still expected to work while paused.
  ///
  /// `reason` gets a human-readable cause suitable for both a log line and a
  /// Trigger response message; it is left untouched when kNone is returned.
  [[nodiscard]] TickBlocker TickBlockedBy(std::string& reason) const;

  /// Escalate a controller wait that has outlasted controller_wait_timeout_s_.
  /// Logs ERROR exactly once per activation and keeps waiting — a CM that is
  /// merely slow still recovers, while a misconfigured one stops being a
  /// silent stall behind a throttled WARN.
  void ReportControllerWaitTimeout();

  /// Inject bb.* parameters into the tree's root blackboard.
  void InitializeBlackboard();

  /// Log detailed failure diagnosis: which nodes failed and why.
  void LogFailureDiagnosis();

  /// Attach the real-time FailureLogger to the current tree root.
  /// Must be called after LoadTree() and after any runtime tree switch.
  void InstallFailureLogger();

  /// Log topic health status.
  void WatchdogCheck();

  /// Cancel the tick/repeat/watchdog timers and halt the tree. Shared by the
  /// transitions that leave the ACTIVE state (on_deactivate, on_error).
  void CancelActiveTimersAndHalt();

  /// Destroy all owned resources (timers, service, param callback, failure
  /// logger, Groot2 publisher, tree, bridge). Shared by on_cleanup and on_error.
  void ReleaseAllResources();

  /// Parameter change callback for runtime tree switching and pause control.
  rcl_interfaces::msg::SetParametersResult OnParameterChange(
      const std::vector<rclcpp::Parameter>& params);

  /// Service callback for single-step tick.
  void StepCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // optional so on_configure can rebuild it in place with emplace(). Never
  // move a BehaviorTreeFactory: its ctor binds an internal XMLParser to the
  // object's own address and the defaulted move ops (BT.CPP 4.9) don't rebind
  // it, so any moved-into factory SIGSEGVs in createTree* via the dangling
  // parser→factory reference.
  std::optional<BT::BehaviorTreeFactory> factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::shared_ptr<BtRosBridge> bridge_;

  rclcpp::TimerBase::SharedPtr tick_timer_;

  // Parameters
  std::string tree_file_;
  // Robot profile (Seam A/B): device groups woven into every topic path +
  // joint widths (default 6/10) so DoF is not hardcoded (kDefault*Dof).
  std::string arm_group_{"ur5e"};
  std::string hand_group_{"hand"};
  int arm_dof_{kDefaultArmDof};
  int hand_dof_{kDefaultHandDof};
  // Optional-sensor capabilities (Seam D) — false skips that node group's
  // registration. Default true reproduces the legacy register-everything path.
  bool has_grasp_sensing_{true};
  bool has_tof_{true};
  bool has_shape_{true};
  double tick_rate_hz_{20.0};
  bool repeat_{false};
  double repeat_delay_s_{1.0};
  bool paused_{false};
  bool step_mode_{false};
  int groot2_port_{0};
  double watchdog_timeout_s_{2.0};
  double watchdog_interval_s_{5.0};
  /// How long the tick gate may wait for the first active controller before
  /// saying so at ERROR (<= 0 disables the escalation). The wait itself is
  /// never bounded — see ReportControllerWaitTimeout.
  double controller_wait_timeout_s_{10.0};

  // Controller-wait escalation state, re-armed on every on_activate.
  std::chrono::steady_clock::time_point activate_time_;
  bool controller_wait_reported_{false};

  // Repeat state
  rclcpp::TimerBase::SharedPtr repeat_timer_;

  // Watchdog timer
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Step mode service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr step_service_;

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  // Groot2 publisher
#ifdef BT_GROOT2_AVAILABLE
  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_;
#endif

  // Real-time failure logger — prints every FAILURE transition via rclcpp.
  std::unique_ptr<FailureLogger> failure_logger_;
};

}  // namespace rtc_bt
