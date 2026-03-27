#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace rtc_bt {

BtRosBridge::BtRosBridge(rclcpp::Node::SharedPtr node)
  : node_(std::move(node))
{
  // ── Subscribers (all RELIABLE QoS) ──────────────────────────────────────

  arm_gui_sub_ = node_->create_subscription<rtc_msgs::msg::GuiPosition>(
      "/ur5e/gui_position", rclcpp::QoS{10},
      [this](rtc_msgs::msg::GuiPosition::SharedPtr msg) {
        std::lock_guard lock(state_mutex_);
        tcp_pose_.x = msg->task_positions[0];
        tcp_pose_.y = msg->task_positions[1];
        tcp_pose_.z = msg->task_positions[2];
        tcp_pose_.roll  = msg->task_positions[3];
        tcp_pose_.pitch = msg->task_positions[4];
        tcp_pose_.yaw   = msg->task_positions[5];
        arm_joint_positions_.assign(
            msg->joint_positions.begin(), msg->joint_positions.end());
      });

  hand_gui_sub_ = node_->create_subscription<rtc_msgs::msg::GuiPosition>(
      "/hand/gui_position", rclcpp::QoS{10},
      [this](rtc_msgs::msg::GuiPosition::SharedPtr msg) {
        std::lock_guard lock(state_mutex_);
        hand_joint_positions_.assign(
            msg->joint_positions.begin(), msg->joint_positions.end());
      });

  sensor_monitor_sub_ = node_->create_subscription<rtc_msgs::msg::HandSensorState>(
      "/hand/sensor_states/monitor", rclcpp::QoS{10},
      [this](rtc_msgs::msg::HandSensorState::SharedPtr msg) {
        std::lock_guard lock(state_mutex_);
        fingertip_forces_.resize(msg->fingertips.size());
        for (std::size_t i = 0; i < msg->fingertips.size(); ++i) {
          auto& dst = fingertip_forces_[i];
          const auto& src = msg->fingertips[i];
          dst.name = src.name;
          dst.fx = src.f[0];
          dst.fy = src.f[1];
          dst.fz = src.f[2];
          dst.contact_flag = src.contact_flag;
          dst.inference_enable = src.inference_enable;
        }
      });

  vision_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vision/object_pose", rclcpp::QoS{10},
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard lock(state_mutex_);
        object_pose_.x = msg->pose.position.x;
        object_pose_.y = msg->pose.position.y;
        object_pose_.z = msg->pose.position.z;

        // Quaternion → RPY
        tf2::Quaternion q(
            msg->pose.orientation.x, msg->pose.orientation.y,
            msg->pose.orientation.z, msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(object_pose_.roll, object_pose_.pitch, object_pose_.yaw);
        object_detected_ = true;
      });

  active_ctrl_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/ur5e/active_controller_name",
      rclcpp::QoS{1}.transient_local(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard lock(state_mutex_);
        active_controller_ = msg->data;
      });

  estop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/system/estop_status", rclcpp::QoS{10},
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard lock(state_mutex_);
        estopped_ = msg->data;
      });

  // ── Publishers ──────────────────────────────────────────────────────────

  arm_target_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/ur5e/joint_goal", rclcpp::QoS{10});

  hand_target_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/hand/joint_goal", rclcpp::QoS{10});

  gains_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/ur5e/gains", rclcpp::QoS{10});

  select_ctrl_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/ur5e/select_controller", rclcpp::QoS{10});

  RCLCPP_INFO(node_->get_logger(), "[BtRosBridge] Initialized");
}

// ── Cached state accessors ────────────────────────────────────────────────

Pose6D BtRosBridge::GetTcpPose() const {
  std::lock_guard lock(state_mutex_);
  return tcp_pose_;
}

std::vector<double> BtRosBridge::GetArmJointPositions() const {
  std::lock_guard lock(state_mutex_);
  return arm_joint_positions_;
}

std::vector<double> BtRosBridge::GetHandJointPositions() const {
  std::lock_guard lock(state_mutex_);
  return hand_joint_positions_;
}

std::vector<FingertipForce> BtRosBridge::GetFingertipForces() const {
  std::lock_guard lock(state_mutex_);
  return fingertip_forces_;
}

bool BtRosBridge::GetObjectPose(Pose6D& pose) const {
  std::lock_guard lock(state_mutex_);
  if (!object_detected_) return false;
  pose = object_pose_;
  return true;
}

std::string BtRosBridge::GetActiveController() const {
  std::lock_guard lock(state_mutex_);
  return active_controller_;
}

bool BtRosBridge::IsEstopped() const {
  std::lock_guard lock(state_mutex_);
  return estopped_;
}

// ── Publishers ────────────────────────────────────────────────────────────

void BtRosBridge::PublishArmTarget(const Pose6D& target) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {target.x, target.y, target.z,
              target.roll, target.pitch, target.yaw};
  arm_target_pub_->publish(msg);
}

void BtRosBridge::PublishArmJointTarget(const std::vector<double>& target) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = target;
  arm_target_pub_->publish(msg);
}

void BtRosBridge::PublishHandTarget(const std::vector<double>& target) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = target;
  hand_target_pub_->publish(msg);
}

void BtRosBridge::PublishGains(const std::vector<double>& gains) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = gains;
  gains_pub_->publish(msg);
}

void BtRosBridge::PublishSelectController(const std::string& name) {
  std_msgs::msg::String msg;
  msg.data = name;
  select_ctrl_pub_->publish(msg);
}

}  // namespace rtc_bt
