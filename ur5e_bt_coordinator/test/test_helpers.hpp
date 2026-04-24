#pragma once
/// Test helpers for ur5e_bt_coordinator unit tests.
///
/// Provides:
///   - RosTestFixture: RAII rclcpp::init/shutdown with a test ROS2 node +
///   BtRosBridge
///   - Helper functions to inject state into BtRosBridge via topic publishing
///   - BT factory helpers for creating single-node trees

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/bt_types.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/polygon.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rtc_msgs/msg/grasp_state.hpp>
#include <rtc_msgs/msg/gui_position.hpp>
#include <shape_estimation_msgs/msg/shape_estimate.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

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

/// RAII test fixture that manages rclcpp lifecycle and provides a BtRosBridge.
///
/// Each test case gets a fresh ROS2 node and bridge instance.
/// State injection publishers are created for all topics the bridge subscribes
/// to.
class RosTestFixture : public ::testing::Test {
protected:
  /// Controller name used for Phase 4 namespaced topics (/<name>/...).
  static constexpr const char *kTestControllerName = "test_controller";

  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owns_rclcpp_ = true;
    }
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "bt_test_node", rclcpp::NodeOptions().use_intra_process_comms(true));
    bridge_ = std::make_shared<BtRosBridge>(node_);

    // Phase 4: controller-owned topics resolve under /<ctrl>/... so the
    // fixture pretends to be "test_controller" — matches what the bridge
    // rewires to on receipt of /ur5e/active_controller_name.
    const std::string ctrl_ns = std::string("/") + kTestControllerName;

    arm_gui_pub_ = node_->create_publisher<rtc_msgs::msg::GuiPosition>(
        ctrl_ns + "/ur5e/gui_position", rclcpp::QoS{10});
    hand_gui_pub_ = node_->create_publisher<rtc_msgs::msg::GuiPosition>(
        ctrl_ns + "/hand/gui_position", rclcpp::QoS{10});
    grasp_state_pub_ = node_->create_publisher<rtc_msgs::msg::GraspState>(
        ctrl_ns + "/hand/grasp_state", rclcpp::QoS{10});
    world_target_pub_ = node_->create_publisher<geometry_msgs::msg::Polygon>(
        "/world_target_info", rclcpp::QoS{10});
    active_ctrl_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/ur5e/active_controller_name", rclcpp::QoS{1}.transient_local());
    estop_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "/system/estop_status", rclcpp::QoS{10});
    current_gains_pub_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/ur5e/current_gains", rclcpp::QoS{10});
    shape_estimate_pub_ =
        node_->create_publisher<shape_estimation_msgs::msg::ShapeEstimate>(
            "/shape/estimate", rclcpp::QoS{10});

    // Fire the active_controller signal so the bridge rewires to the test
    // controller namespace before any state injection.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    {
      std_msgs::msg::String msg;
      msg.data = kTestControllerName;
      active_ctrl_pub_->publish(msg);
    }
    // Wait for intra-process connections + rewire to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    Spin(std::chrono::milliseconds(50));
  }

  void TearDown() override {
    bridge_.reset();
    node_.reset();
    if (owns_rclcpp_ && rclcpp::ok()) {
      rclcpp::shutdown();
      owns_rclcpp_ = false;
    }
  }

  /// Spin the test node to process subscription callbacks.
  void
  Spin(std::chrono::milliseconds duration = std::chrono::milliseconds(50)) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      rclcpp::spin_some(node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
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

  void PublishCurrentGains(const std::vector<double> &gains) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = gains;
    current_gains_pub_->publish(msg);
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

  rclcpp::Publisher<rtc_msgs::msg::GuiPosition>::SharedPtr arm_gui_pub_;
  rclcpp::Publisher<rtc_msgs::msg::GuiPosition>::SharedPtr hand_gui_pub_;
  rclcpp::Publisher<rtc_msgs::msg::GraspState>::SharedPtr grasp_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr world_target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_ctrl_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      current_gains_pub_;
  rclcpp::Publisher<shape_estimation_msgs::msg::ShapeEstimate>::SharedPtr
      shape_estimate_pub_;

private:
  bool owns_rclcpp_{false};
};

} // namespace rtc_bt::test
