#include "ur5e_hand_udp/hand_controller.hpp"
#include "ur5e_hand_udp/hand_failure_detector.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sys/mman.h>  // mlockall

#include <array>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

using namespace std::chrono_literals;
namespace urtc = ur5e_rt_controller;

// Unified hand UDP node using request-response protocol.
//
// Owns a HandController that polls the hand device:
//   write position → read position → read velocity → read sensors × 4
//
// Publishes full state on /hand/joint_states.
// Receives commands on /hand/command.
class HandUdpNode : public rclcpp::Node {
 public:
  HandUdpNode() : Node("hand_udp_node") {
    // ── Parameters ─────────────────────────────────────────────────────
    declare_parameter("target_ip",       std::string{"192.168.1.2"});
    declare_parameter("target_port",     55151);
    declare_parameter("publish_rate",    100.0);
    declare_parameter("recv_timeout_ms", 10);
    declare_parameter("enable_write_ack", false);
    declare_parameter("enable_failure_detector", true);
    declare_parameter("failure_threshold", 5);
    declare_parameter("check_motor", true);
    declare_parameter("check_sensor", true);
    declare_parameter("min_rate_hz", 30.0);
    declare_parameter("rate_fail_threshold", 5);

    const std::string target_ip       = get_parameter("target_ip").as_string();
    const int         target_port     = get_parameter("target_port").as_int();
    const double      rate            = get_parameter("publish_rate").as_double();
    const int         recv_timeout_ms = get_parameter("recv_timeout_ms").as_int();
    const bool        enable_write_ack = get_parameter("enable_write_ack").as_bool();

    // ── HandController ─────────────────────────────────────────────────
    controller_ = std::make_unique<urtc::HandController>(
        target_ip, target_port, urtc::kUdpRecvConfig, recv_timeout_ms,
        enable_write_ack);

    controller_->SetCallback([this](const urtc::HandState& state) {
      std::lock_guard lock(data_mutex_);
      latest_state_  = state;
      data_received_ = true;
    });

    if (!controller_->Start()) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to start HandController to %s:%d",
                   target_ip.c_str(), target_port);
      return;
    }

    // ── Failure Detector (optional) ──────────────────────────────────
    const bool enable_fd = get_parameter("enable_failure_detector").as_bool();
    if (enable_fd) {
      urtc::HandFailureDetector::Config fd_cfg;
      fd_cfg.failure_threshold = static_cast<int>(
          get_parameter("failure_threshold").as_int());
      fd_cfg.check_motor  = get_parameter("check_motor").as_bool();
      fd_cfg.check_sensor = get_parameter("check_sensor").as_bool();
      fd_cfg.min_rate_hz  = get_parameter("min_rate_hz").as_double();
      fd_cfg.rate_fail_threshold = static_cast<int>(
          get_parameter("rate_fail_threshold").as_int());

      failure_detector_ = std::make_unique<urtc::HandFailureDetector>(
          *controller_, fd_cfg);
      failure_detector_->SetFailureCallback(
          [this](const std::string& reason) {
            RCLCPP_ERROR(get_logger(), "Hand failure detected: %s", reason.c_str());
          });
      failure_detector_->Start();
      RCLCPP_INFO(get_logger(), "HandFailureDetector started (50 Hz, threshold=%d)",
                  fd_cfg.failure_threshold);
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
                "HandUdpNode: target %s:%d, pub %.0f Hz",
                target_ip.c_str(), target_port, rate);
  }

  ~HandUdpNode() override {
    SaveCommStats();
    if (failure_detector_) failure_detector_->Stop();
    if (controller_) controller_->Stop();
  }

 private:
  void PublishState() {
    if (!data_received_) return;

    urtc::HandState snapshot;
    {
      std::lock_guard lock(data_mutex_);
      snapshot = latest_state_;
    }

    // Publish: [10 positions] + [10 velocities] + [44 sensors]
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(urtc::kNumHandMotors * 2 + urtc::kNumHandSensors);

    for (int i = 0; i < urtc::kNumHandMotors; ++i)
      msg.data.push_back(static_cast<double>(snapshot.motor_positions[i]));
    for (int i = 0; i < urtc::kNumHandMotors; ++i)
      msg.data.push_back(static_cast<double>(snapshot.motor_velocities[i]));
    for (int i = 0; i < urtc::kNumHandSensors; ++i)
      msg.data.push_back(static_cast<double>(snapshot.sensor_data[i]));

    state_pub_->publish(msg);

    if (++publish_count_ % 100 == 0) {
      RCLCPP_DEBUG(get_logger(), "cycles: %zu",
                   controller_->cycle_count());
    }
  }

  void SaveCommStats() const {
    if (!controller_) return;

    const auto stats = controller_->comm_stats();
    const bool fd_failed = failure_detector_ ? failure_detector_->failed() : false;

    // 평균 rate 계산 (start_time_ 기준)
    const auto elapsed = std::chrono::steady_clock::now() - start_time_;
    const double elapsed_sec = std::chrono::duration<double>(elapsed).count();
    const double avg_rate_hz = (elapsed_sec > 0.0)
        ? static_cast<double>(stats.total_cycles) / elapsed_sec
        : 0.0;

    // 세션 디렉토리 기반 경로 결정
    std::string output_dir;
    const char* session_env = std::getenv("UR5E_SESSION_DIR");
    if (session_env != nullptr && session_env[0] != '\0') {
      output_dir = std::string(session_env) + "/hand";
    } else {
      output_dir = "/tmp";
    }

    const std::string path = output_dir + "/hand_udp_stats.json";
    std::ofstream ofs(path);
    if (!ofs.is_open()) return;

    ofs << "{\n"
        << "  \"total_cycles\": "    << stats.total_cycles   << ",\n"
        << "  \"recv_ok\": "         << stats.recv_ok        << ",\n"
        << "  \"recv_timeout\": "    << stats.recv_timeout    << ",\n"
        << "  \"recv_error\": "      << stats.recv_error      << ",\n"
        << "  \"avg_rate_hz\": "     << std::fixed << std::setprecision(2) << avg_rate_hz << ",\n"
        << "  \"elapsed_sec\": "     << std::fixed << std::setprecision(2) << elapsed_sec << ",\n"
        << "  \"failure_detected\": " << (fd_failed ? "true" : "false") << "\n"
        << "}\n";
    ofs.close();

    RCLCPP_INFO(get_logger(),
                "Hand comm stats saved to %s (cycles=%lu, ok=%lu, timeout=%lu, error=%lu, rate=%.1f Hz)",
                path.c_str(), stats.total_cycles, stats.recv_ok,
                stats.recv_timeout, stats.recv_error, avg_rate_hz);
  }

  void OnCommand(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != static_cast<std::size_t>(urtc::kNumHandMotors)) {
      RCLCPP_WARN(get_logger(),
                  "Unexpected command size %zu (expected %d)",
                  msg->data.size(), urtc::kNumHandMotors);
      return;
    }

    std::array<float, urtc::kNumHandMotors> cmd;
    for (int i = 0; i < urtc::kNumHandMotors; ++i) {
      cmd[i] = static_cast<float>(msg->data[i]);
    }

    controller_->SetTargetPositions(cmd);
  }

  std::unique_ptr<urtc::HandController>      controller_;
  std::unique_ptr<urtc::HandFailureDetector> failure_detector_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  mutable std::mutex  data_mutex_;
  urtc::HandState     latest_state_{};
  bool                data_received_{false};
  std::size_t         publish_count_{0};

  std::chrono::steady_clock::time_point start_time_{std::chrono::steady_clock::now()};
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
