#include "ur5e_hand_udp/hand_udp_sender.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <string>

namespace urtc = ur5e_rt_controller;

// Bridges the ROS2 /hand/command topic to UDP packets for the hand controller.
class HandUdpSenderNode : public rclcpp::Node {
 public:
  HandUdpSenderNode() : Node("hand_udp_sender_node") {
    declare_parameter("target_ip",   std::string{"192.168.1.2"});
    declare_parameter("target_port", 55151);

    const std::string ip   = get_parameter("target_ip").as_string();
    const int         port = get_parameter("target_port").as_int();

    sender_ = std::make_unique<urtc::HandUdpSender>(ip, port);

    if (!sender_->Initialize()) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to initialise UDP sender to %s:%d", ip.c_str(), port);
      return;
    }

    command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/hand/command", 10,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          CommandCallback(std::move(msg));
        });

    RCLCPP_INFO(get_logger(), "HandUdpSender ready — target %s:%d",
                ip.c_str(), port);
  }

 private:
  void CommandCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != static_cast<std::size_t>(urtc::kNumHandJoints)) {
      RCLCPP_WARN(get_logger(),
                  "Unexpected command size %zu (expected %d)",
                  msg->data.size(), urtc::kNumHandJoints);
      return;
    }

    std::array<double, urtc::kNumHandJoints> command;
    std::copy_n(msg->data.begin(), urtc::kNumHandJoints, command.begin());

    if (sender_->SendCommand(command)) {
      if (++send_count_ % 100 == 0) {
        RCLCPP_DEBUG(get_logger(), "Commands sent: %zu", sender_->send_count());
      }
    } else {
      RCLCPP_ERROR(get_logger(), "UDP send failed");
    }
  }

  std::unique_ptr<urtc::HandUdpSender>                           sender_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  std::size_t send_count_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandUdpSenderNode>());
  rclcpp::shutdown();
  return 0;
}
