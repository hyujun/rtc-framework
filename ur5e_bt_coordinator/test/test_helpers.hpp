#pragma once
/// Test helpers for ur5e_bt_coordinator unit tests.
///
/// Provides:
///   - RosTestFixture: RAII rclcpp::init/shutdown with a test ROS2 node +
///                     BtRosBridge + three mock controller LifecycleNodes
///                     (one per demo controller name) hosting the parameter
///                     API + grasp_command srv. Tests publish the desired
///                     active controller name and the bridge rebinds its
///                     AsyncParametersClient + grasp_command_client_ to the
///                     matching mock automatically.

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/bt_types.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/polygon.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rtc_msgs/msg/grasp_state.hpp>
#include <rtc_msgs/msg/gui_position.hpp>
#include <rtc_msgs/srv/grasp_command.hpp>
#include <shape_estimation_msgs/msg/shape_estimate.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rtc_bt::test {

/// Spin the node until a condition is met or timeout expires.
inline void
SpinUntil(rclcpp::Node::SharedPtr node, std::function<bool()> condition,
          std::chrono::milliseconds timeout = std::chrono::milliseconds(500)) {
  auto start = std::chrono::steady_clock::now();
  while (!condition()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (std::chrono::steady_clock::now() - start > timeout)
      break;
  }
}

/// Mock per-controller LifecycleNode hosting a representative gain parameter
/// set + grasp_command srv. Mirrors the real demo controllers' surface so
/// the bridge's AsyncParametersClient + grasp_command_client_ can round-trip
/// against it.
struct MockController {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::Service<rtc_msgs::srv::GraspCommand>::SharedPtr grasp_srv;
  uint8_t last_grasp_cmd{0};
  double last_grasp_force{0.0};
  std::atomic<int> grasp_cmd_calls{0};
};

/// RAII test fixture that manages rclcpp lifecycle and provides:
///   - bridge node    (`node_`)               — hosts BtRosBridge
///   - mock controllers (`mock_joint_`, `mock_task_`, `mock_wbc_`) at
///     /<name>/<name>, each with its name's parameter subset declared and
///     a grasp_command srv server attached.
///   - background spin drives all four nodes so bridge clients' futures
///     resolve during BT ticks.
class RosTestFixture : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owns_rclcpp_ = true;
    }
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "bt_test_node", rclcpp::NodeOptions().use_intra_process_comms(true));
    bridge_ = std::make_shared<BtRosBridge>(node_);

    SpawnMock("demo_joint_controller", mock_joint_);
    SpawnMock("demo_task_controller", mock_task_);
    SpawnMock("demo_wbc_controller", mock_wbc_);

    // State injection publishers — pick demo_task_controller as the default
    // active for state-injection tests; SetActiveAlias() can override later.
    const std::string ctrl_ns = "/demo_task_controller";
    arm_gui_pub_ = node_->create_publisher<rtc_msgs::msg::GuiPosition>(
        ctrl_ns + "/ur5e/gui_position", rclcpp::QoS{10});
    hand_gui_pub_ = node_->create_publisher<rtc_msgs::msg::GuiPosition>(
        ctrl_ns + "/hand/gui_position", rclcpp::QoS{10});
    grasp_state_pub_ = node_->create_publisher<rtc_msgs::msg::GraspState>(
        ctrl_ns + "/hand/grasp_state", rclcpp::QoS{10});
    world_target_pub_ = node_->create_publisher<geometry_msgs::msg::Polygon>(
        "/world_target_info", rclcpp::QoS{10});
    active_ctrl_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/rtc_cm/active_controller_name", rclcpp::QoS{1}.transient_local());
    estop_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "/system/estop_status", rclcpp::QoS{10});
    shape_estimate_pub_ =
        node_->create_publisher<shape_estimation_msgs::msg::ShapeEstimate>(
            "/shape/estimate", rclcpp::QoS{10});

    // Background spin drives bridge + all mock nodes.
    spin_running_.store(true);
    spin_thread_ = std::thread([this]() {
      while (spin_running_.load()) {
        rclcpp::spin_some(node_->get_node_base_interface());
        rclcpp::spin_some(mock_joint_.node->get_node_base_interface());
        rclcpp::spin_some(mock_task_.node->get_node_base_interface());
        rclcpp::spin_some(mock_wbc_.node->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });

    // Default active = demo_task_controller (covers most parameter cases).
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    SetActiveAlias("demo_task_controller");
  }

  void TearDown() override {
    spin_running_.store(false);
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    mock_joint_.grasp_srv.reset();
    mock_task_.grasp_srv.reset();
    mock_wbc_.grasp_srv.reset();
    bridge_.reset();
    mock_joint_.node.reset();
    mock_task_.node.reset();
    mock_wbc_.node.reset();
    node_.reset();
    if (owns_rclcpp_ && rclcpp::ok()) {
      rclcpp::shutdown();
      owns_rclcpp_ = false;
    }
  }

  /// Publish the active controller name and wait for the bridge to rebind.
  void SetActiveAlias(const std::string &name) {
    PublishActiveController(name);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }

  /// Sleep wrapper preserved for legacy test bodies.
  void
  Spin(std::chrono::milliseconds duration = std::chrono::milliseconds(50)) {
    std::this_thread::sleep_for(duration);
  }

  // ── State injection helpers ─────────────────────────────────────────────

  void PublishArmState(const Pose6D &tcp, const std::vector<double> &joints) {
    rtc_msgs::msg::GuiPosition msg;
    msg.task_positions = {tcp.x, tcp.y, tcp.z, tcp.roll, tcp.pitch, tcp.yaw};
    msg.joint_positions.assign(joints.begin(), joints.end());
    arm_gui_pub_->publish(msg);
  }

  void PublishHandState(const std::vector<double> &joints) {
    rtc_msgs::msg::GuiPosition msg;
    msg.joint_positions.assign(joints.begin(), joints.end());
    hand_gui_pub_->publish(msg);
  }

  void PublishGraspState(const CachedGraspState &gs) {
    rtc_msgs::msg::GraspState msg;
    msg.num_active_contacts = gs.num_active_contacts;
    msg.max_force = gs.max_force;
    msg.grasp_detected = gs.grasp_detected;
    msg.force_threshold = gs.force_threshold;
    msg.min_fingertips = gs.min_fingertips;
    msg.grasp_phase = gs.grasp_phase;
    msg.grasp_target_force = gs.grasp_target_force;
    for (const auto &ft : gs.fingertips) {
      msg.fingertip_names.push_back(ft.name);
      msg.force_magnitude.push_back(ft.force_magnitude);
      msg.contact_flag.push_back(ft.contact_flag);
      msg.inference_valid.push_back(ft.inference_valid);
    }
    msg.finger_s = gs.finger_s;
    msg.finger_filtered_force = gs.finger_filtered_force;
    msg.finger_force_error = gs.finger_force_error;
    grasp_state_pub_->publish(msg);
  }

  void PublishWorldTarget(double x, double y, double z) {
    geometry_msgs::msg::Polygon msg;
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(x);
    pt.y = static_cast<float>(y);
    pt.z = static_cast<float>(z);
    msg.points.push_back(pt);
    world_target_pub_->publish(msg);
  }

  void PublishActiveController(const std::string &name) {
    std_msgs::msg::String msg;
    msg.data = name;
    active_ctrl_pub_->publish(msg);
  }

  void PublishEstop(bool active) {
    std_msgs::msg::Bool msg;
    msg.data = active;
    estop_pub_->publish(msg);
  }

  void PublishShapeEstimate(uint8_t shape_type, double confidence,
                            uint32_t num_points = 100) {
    shape_estimation_msgs::msg::ShapeEstimate msg;
    msg.shape_type = shape_type;
    msg.confidence = confidence;
    msg.num_points_used = num_points;
    shape_estimate_pub_->publish(msg);
  }

  // ── Data members ────────────────────────────────────────────────────────

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<BtRosBridge> bridge_;

  MockController mock_joint_;
  MockController mock_task_;
  MockController mock_wbc_;

  rclcpp::Publisher<rtc_msgs::msg::GuiPosition>::SharedPtr arm_gui_pub_;
  rclcpp::Publisher<rtc_msgs::msg::GuiPosition>::SharedPtr hand_gui_pub_;
  rclcpp::Publisher<rtc_msgs::msg::GraspState>::SharedPtr grasp_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr world_target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_ctrl_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<shape_estimation_msgs::msg::ShapeEstimate>::SharedPtr
      shape_estimate_pub_;

private:
  void SpawnMock(const std::string &name, MockController &out) {
    const std::string ns = "/" + name;
    const auto opts = rclcpp::NodeOptions().use_global_arguments(false);
    out.node =
        std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, ns, opts);

    // Declare the union of demo controllers' parameter names. Tests only
    // touch the ones relevant to the alias they activate; the rest are
    // declared but unused.
    out.node->declare_parameter<double>("trajectory_speed", 0.1);
    out.node->declare_parameter<double>("trajectory_angular_speed", 0.5);
    out.node->declare_parameter<double>("hand_trajectory_speed", 1.0);
    out.node->declare_parameter<double>("robot_trajectory_speed", 0.5);
    out.node->declare_parameter<double>("arm_trajectory_speed", 0.5);
    out.node->declare_parameter<std::vector<double>>(
        "kp_translation", std::vector<double>{5.0, 5.0, 5.0});
    out.node->declare_parameter<std::vector<double>>(
        "kp_rotation", std::vector<double>{2.0, 2.0, 2.0});
    out.node->declare_parameter<double>("damping", 0.01);
    out.node->declare_parameter<double>("null_kp", 0.5);
    out.node->declare_parameter<bool>("enable_null_space", false);
    out.node->declare_parameter<bool>("control_6dof", true);
    out.node->declare_parameter<double>("grasp_contact_threshold", 0.5);
    out.node->declare_parameter<double>("grasp_force_threshold", 1.0);
    out.node->declare_parameter<int64_t>("grasp_min_fingertips", 2);
    out.node->declare_parameter<double>("se3_weight", 100.0);
    out.node->declare_parameter<double>("force_weight", 10.0);
    out.node->declare_parameter<double>("posture_weight", 1.0);
    out.node->declare_parameter<bool>("mpc_enable", false);
    out.node->declare_parameter<double>("riccati_gain_scale", 1.0);

    out.grasp_srv = out.node->create_service<rtc_msgs::srv::GraspCommand>(
        "grasp_command",
        [&out](const std::shared_ptr<rtc_msgs::srv::GraspCommand::Request> req,
               std::shared_ptr<rtc_msgs::srv::GraspCommand::Response> resp) {
          out.last_grasp_cmd = req->command;
          out.last_grasp_force = req->target_force;
          out.grasp_cmd_calls.fetch_add(1);
          resp->ok = true;
          resp->message = "mock ok";
        });
  }

  bool owns_rclcpp_{false};
  std::atomic<bool> spin_running_{false};
  std::thread spin_thread_;
};

} // namespace rtc_bt::test
