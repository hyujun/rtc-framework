#pragma once

#include "ur5e_bt_coordinator/bt_types.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtc_msgs/msg/gui_position.hpp>
#include <rtc_msgs/msg/grasp_state.hpp>
#include <rtc_msgs/msg/robot_target.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace rtc_bt {

/// Bridges ROS2 topics to/from the BT Blackboard.
///
/// Subscribers cache the latest value from each RELIABLE topic.
/// BT nodes read/write via the public accessors (mutex-protected).
/// Publishers send commands to the RT control layer.
class BtRosBridge {
public:
  explicit BtRosBridge(rclcpp::Node::SharedPtr node);

  // ── Cached state (thread-safe reads) ──────────────────────────────────────

  /// Current TCP pose from /ur5e/gui_position
  Pose6D GetTcpPose() const;

  /// Current arm joint positions from /ur5e/gui_position
  std::vector<double> GetArmJointPositions() const;

  /// Current hand joint positions from /hand/gui_position
  std::vector<double> GetHandJointPositions() const;

  /// Cached grasp state from /hand/grasp_state (500Hz pre-computed)
  CachedGraspState GetGraspState() const;

  /// Latest vision object pose from /vision/object_pose
  bool GetObjectPose(Pose6D& pose) const;

  /// Active controller name
  std::string GetActiveController() const;

  /// E-STOP status
  bool IsEstopped() const;

  // ── Publishers (send commands) ────────────────────────────────────────────

  /// Publish arm task-space target [x,y,z,roll,pitch,yaw]
  void PublishArmTarget(const Pose6D& target);

  /// Publish arm joint-space target [q0..q5]
  void PublishArmJointTarget(const std::vector<double>& target);

  /// Publish hand motor target [m0..m9]
  void PublishHandTarget(const std::vector<double>& target);

  /// Publish gain update (16-element array)
  void PublishGains(const std::vector<double>& gains);

  /// Publish controller switch command
  void PublishSelectController(const std::string& name);

private:
  rclcpp::Node::SharedPtr node_;

  // ── Subscribers ───────────────────────────────────────────────────────────
  rclcpp::Subscription<rtc_msgs::msg::GuiPosition>::SharedPtr      arm_gui_sub_;
  rclcpp::Subscription<rtc_msgs::msg::GuiPosition>::SharedPtr      hand_gui_sub_;
  rclcpp::Subscription<rtc_msgs::msg::GraspState>::SharedPtr       grasp_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr           active_ctrl_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             estop_sub_;

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<rtc_msgs::msg::RobotTarget>::SharedPtr arm_target_pub_;
  rclcpp::Publisher<rtc_msgs::msg::RobotTarget>::SharedPtr hand_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gains_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            select_ctrl_pub_;

  // ── Cached state ──────────────────────────────────────────────────────────
  mutable std::mutex state_mutex_;
  Pose6D tcp_pose_;
  std::vector<double> arm_joint_positions_;
  std::vector<double> hand_joint_positions_;
  CachedGraspState grasp_state_;
  Pose6D object_pose_;
  bool object_detected_{false};
  std::string active_controller_;
  bool estopped_{false};
};

}  // namespace rtc_bt
