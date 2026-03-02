// hand_udp_sender_node.cpp - v1
#include "ur5e_rt_controller/hand_udp_sender.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <memory>

class HandUdpSenderNode : public rclcpp::Node {
public:
  HandUdpSenderNode()
    : Node("hand_udp_sender_node")
  {
    // Parameters
    this->declare_parameter("target_ip", "192.168.1.100");
    this->declare_parameter("target_port", 50002);
    
    std::string ip = this->get_parameter("target_ip").as_string();
    int port = this->get_parameter("target_port").as_int();
    
    // Create UDP sender
    sender_ = std::make_unique<ur5e_controller::HandUdpSender>(ip, port);
    
    if (!sender_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP sender");
      return;
    }
    
    // Subscriber
    command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/hand/command", 10,
        std::bind(&HandUdpSenderNode::command_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), 
                "Hand UDP Sender initialized");
    RCLCPP_INFO(this->get_logger(), 
                "Target: %s:%d", ip.c_str(), port);
  }

private:
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != ur5e_controller::HandUdpSender::NUM_HAND_JOINTS) {
      RCLCPP_WARN(this->get_logger(), 
                  "Invalid command size: %zu (expected %zu)",
                  msg->data.size(),
                  ur5e_controller::HandUdpSender::NUM_HAND_JOINTS);
      return;
    }
    
    std::array<double, ur5e_controller::HandUdpSender::NUM_HAND_JOINTS> command;
    std::copy(msg->data.begin(), msg->data.end(), command.begin());
    
    if (sender_->send_command(command)) {
      send_count_++;
      
      if (send_count_ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "Commands sent: %zu", sender_->get_send_count());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to send command");
    }
  }

  std::unique_ptr<ur5e_controller::HandUdpSender> sender_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  size_t send_count_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandUdpSenderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
