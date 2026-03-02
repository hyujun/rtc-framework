// hand_udp_receiver_node.cpp - v1
#include "ur5e_rt_controller/hand_udp_receiver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class HandUdpReceiverNode : public rclcpp::Node {
public:
  HandUdpReceiverNode()
    : Node("hand_udp_receiver_node")
  {
    // Parameters
    this->declare_parameter("udp_port", 50001);
    this->declare_parameter("publish_rate", 100.0);
    
    int port = this->get_parameter("udp_port").as_int();
    double rate = this->get_parameter("publish_rate").as_double();
    
    // Create UDP receiver
    receiver_ = std::make_unique<ur5e_controller::HandUdpReceiver>(port);
    
    // Publisher
    hand_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/hand/joint_states", 10);
    
    // Set callback
    receiver_->set_callback([this](const auto& data) {
      this->hand_data_callback(data);
    });
    
    // Start receiver
    if (!receiver_->start()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start UDP receiver");
      return;
    }
    
    // Publishing timer
    auto period = std::chrono::microseconds(
        static_cast<int>(1000000.0 / rate));
    publish_timer_ = this->create_wall_timer(
        period,
        std::bind(&HandUdpReceiverNode::publish_loop, this));
    
    RCLCPP_INFO(this->get_logger(), 
                "Hand UDP Receiver started on port %d", port);
    RCLCPP_INFO(this->get_logger(), 
                "Publishing at %.1f Hz", rate);
  }
  
  ~HandUdpReceiverNode() {
    if (receiver_) {
      receiver_->stop();
    }
  }

private:
  void hand_data_callback(
      const std::array<double, ur5e_controller::HandUdpReceiver::NUM_HAND_JOINTS>& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_data_ = data;
    data_received_ = true;
  }
  
  void publish_loop() {
    if (!data_received_) return;
    
    std::array<double, ur5e_controller::HandUdpReceiver::NUM_HAND_JOINTS> data;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      data = latest_data_;
    }
    
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.assign(data.begin(), data.end());
    hand_state_pub_->publish(msg);
    
    publish_count_++;
    
    if (publish_count_ % 100 == 0) {
      size_t packets = receiver_->get_packet_count();
      double rate = receiver_->get_update_rate();
      RCLCPP_INFO(this->get_logger(),
                  "Packets: %zu, Rate: %.1f Hz", packets, rate);
    }
  }

  std::unique_ptr<ur5e_controller::HandUdpReceiver> receiver_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr hand_state_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  
  std::array<double, ur5e_controller::HandUdpReceiver::NUM_HAND_JOINTS> latest_data_{};
  std::mutex data_mutex_;
  bool data_received_{false};
  size_t publish_count_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandUdpReceiverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
