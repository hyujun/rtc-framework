#include "ur5e_hand_udp/hand_udp_receiver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sys/mman.h>  // mlockall

#include <array>
#include <chrono>
#include <memory>
#include <mutex>

using namespace std::chrono_literals;
namespace urtc = ur5e_rt_controller;

// Bridges HandUdpReceiver into the ROS2 graph.
//
// Received UDP packets are buffered via HandUdpReceiver's callback and
// re-published on /hand/joint_states at `publish_rate` Hz.
class HandUdpReceiverNode : public rclcpp::Node {
 public:
  HandUdpReceiverNode() : Node("hand_udp_receiver_node") {
    declare_parameter("udp_port",     50001);
    declare_parameter("publish_rate", 100.0);

    const int    port = get_parameter("udp_port").as_int();
    const double rate = get_parameter("publish_rate").as_double();

    receiver_ = std::make_unique<urtc::HandUdpReceiver>(port);

    hand_state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/hand/joint_states", 10);

    // Callback receives the full HandState; we buffer it for the publish timer.
    receiver_->SetCallback([this](const urtc::HandState& state) {
      std::lock_guard lock(data_mutex_);
      latest_state_ = state;
      data_received_ = true;
    });

    if (!receiver_->Start()) {
      RCLCPP_ERROR(get_logger(), "Failed to start UDP receiver on port %d", port);
      return;
    }

    const auto period = std::chrono::microseconds(
        static_cast<int>(1'000'000.0 / rate));
    publish_timer_ = create_wall_timer(period, [this]() { PublishLoop(); });

    RCLCPP_INFO(get_logger(), "HandUdpReceiver: port %d, pub %.0f Hz", port, rate);
  }

  ~HandUdpReceiverNode() override {
    if (receiver_) {
      receiver_->Stop();
    }
  }

 private:
  void PublishLoop() {
    if (!data_received_) {
      return;
    }

    std::array<double, urtc::kNumHandJoints> snapshot;
    {
      std::lock_guard lock(data_mutex_);
      snapshot = latest_state_.motor_positions;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(snapshot.begin(), snapshot.end());
    hand_state_pub_->publish(msg);

    if (++publish_count_ % 100 == 0) {
      RCLCPP_DEBUG(get_logger(), "Packets received: %zu, rate: %.1f Hz",
                   receiver_->packet_count(), receiver_->GetUpdateRate());
    }
  }

  std::unique_ptr<urtc::HandUdpReceiver>                          receiver_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  hand_state_pub_;
  rclcpp::TimerBase::SharedPtr                                    publish_timer_;

  mutable std::mutex   data_mutex_;
  urtc::HandState      latest_state_{};
  bool                 data_received_{false};
  std::size_t          publish_count_{0};
};

int main(int argc, char** argv) {
  // Lock all current and future pages — prevents page faults in the UDP recv
  // jthread which runs at SCHED_FIFO 65.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    fprintf(stderr, "[WARN] hand_udp_receiver_node: mlockall failed\n");
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandUdpReceiverNode>());
  rclcpp::shutdown();
  return 0;
}
