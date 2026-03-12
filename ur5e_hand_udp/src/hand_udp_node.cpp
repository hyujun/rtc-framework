#include "ur5e_hand_udp/hand_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sys/mman.h>  // mlockall

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;
namespace urtc = ur5e_rt_controller;

// Unified hand UDP node — replaces the separate receiver + sender nodes.
//
// Owns a single HandController that manages both directions:
//   recv: hand controller → /hand/joint_states (publishes at publish_rate Hz)
//   send: /hand/command → hand controller (forwards on subscription callback)
class HandUdpNode : public rclcpp::Node {
 public:
  HandUdpNode() : Node("hand_udp_node") {
    // ── Parameters ─────────────────────────────────────────────────────
    declare_parameter("recv_port",    50001);
    declare_parameter("target_ip",    std::string{"192.168.1.100"});
    declare_parameter("target_port",  50002);
    declare_parameter("publish_rate", 100.0);

    const int         recv_port   = get_parameter("recv_port").as_int();
    const std::string target_ip   = get_parameter("target_ip").as_string();
    const int         target_port = get_parameter("target_port").as_int();
    const double      rate        = get_parameter("publish_rate").as_double();

    // ── HandController ─────────────────────────────────────────────────
    controller_ = std::make_unique<urtc::HandController>(recv_port);

    controller_->SetCallback([this](const urtc::HandState& state) {
      std::lock_guard lock(data_mutex_);
      latest_state_  = state;
      data_received_ = true;
    });

    if (!controller_->Start()) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to start HandController recv on port %d", recv_port);
      return;
    }

    if (!controller_->InitSend(target_ip, target_port)) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to init HandController send to %s:%d",
                   target_ip.c_str(), target_port);
      return;
    }

    // ── ROS2 pub/sub ───────────────────────────────────────────────────
    state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/hand/joint_states", 10);

    command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/hand/command", 10,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          OnCommand(std::move(msg));
        });

    const auto period = std::chrono::microseconds(
        static_cast<int>(1'000'000.0 / rate));
    publish_timer_ = create_wall_timer(period, [this]() { PublishState(); });

    RCLCPP_INFO(get_logger(),
                "HandUdpNode: recv :%d, send %s:%d, pub %.0f Hz",
                recv_port, target_ip.c_str(), target_port, rate);
  }

  ~HandUdpNode() override {
    if (controller_) controller_->Stop();
  }

 private:
  void PublishState() {
    if (!data_received_) return;

    std::array<double, urtc::kNumHandJoints> snapshot;
    {
      std::lock_guard lock(data_mutex_);
      snapshot = latest_state_.motor_positions;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(snapshot.begin(), snapshot.end());
    state_pub_->publish(msg);

    if (++publish_count_ % 100 == 0) {
      RCLCPP_DEBUG(get_logger(), "recv: %zu pkts, send: %zu pkts",
                   controller_->recv_count(), controller_->send_count());
    }
  }

  void OnCommand(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != static_cast<std::size_t>(urtc::kNumHandJoints)) {
      RCLCPP_WARN(get_logger(),
                  "Unexpected command size %zu (expected %d)",
                  msg->data.size(), urtc::kNumHandJoints);
      return;
    }

    std::array<double, urtc::kNumHandJoints> cmd;
    std::copy_n(msg->data.begin(), urtc::kNumHandJoints, cmd.begin());

    if (!controller_->SendCommand(cmd)) {
      RCLCPP_ERROR(get_logger(), "UDP send failed");
    }
  }

  std::unique_ptr<urtc::HandController> controller_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  mutable std::mutex  data_mutex_;
  urtc::HandState     latest_state_{};
  bool                data_received_{false};
  std::size_t         publish_count_{0};
};

int main(int argc, char** argv) {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    fprintf(stderr, "[WARN] hand_udp_node: mlockall failed\n");
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandUdpNode>());
  rclcpp::shutdown();
  return 0;
}
