#include "ur5e_hand_udp/hand_controller.hpp"
#include "ur5e_hand_udp/hand_failure_detector.hpp"

#include <ur5e_rt_base/threading/thread_utils.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sys/mman.h>  // mlockall

#include <array>
#include <atomic>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <memory>
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
    declare_parameter("recv_timeout_ms", 10.0);
    // enable_write_ack is deprecated — echo is always consumed for RT safety.
    // Parameter kept for backward compatibility but ignored.
    declare_parameter("enable_write_ack", false);
    declare_parameter("enable_failure_detector", true);
    declare_parameter("failure_threshold", 5);
    declare_parameter("check_motor", true);
    declare_parameter("check_sensor", true);
    declare_parameter("min_rate_hz", 30.0);
    declare_parameter("rate_fail_threshold", 5);
    declare_parameter("check_link", true);
    declare_parameter("link_fail_threshold", 10);

    // Hand motor/fingertip names (이름 기반 매핑용)
    declare_parameter("hand_motor_names", std::vector<std::string>{});
    declare_parameter("hand_fingertip_names", std::vector<std::string>{});

    // Communication mode: "individual" (0x11+0x12+0x14~0x17) or "bulk" (0x10+0x19)
    declare_parameter("communication_mode", std::string{"individual"});

    // Sensor LPF
    declare_parameter("baro_lpf_enabled", false);
    declare_parameter("baro_lpf_cutoff_hz", 30.0);
    declare_parameter("tof_lpf_enabled", false);
    declare_parameter("tof_lpf_cutoff_hz", 15.0);

    const std::string target_ip       = get_parameter("target_ip").as_string();
    const int         target_port     = get_parameter("target_port").as_int();
    const double      rate            = get_parameter("publish_rate").as_double();
    const double      recv_timeout_ms = get_parameter("recv_timeout_ms").as_double();
    // ── Communication mode ──────────────────────────────────────────────
    const std::string comm_mode_str = get_parameter("communication_mode").as_string();
    const auto comm_mode = (comm_mode_str == "bulk")
        ? urtc::HandCommunicationMode::kBulk
        : urtc::HandCommunicationMode::kIndividual;

    // ── Sensor LPF ────────────────────────────────────────────────────────
    const bool   baro_lpf_enabled   = get_parameter("baro_lpf_enabled").as_bool();
    const double baro_lpf_cutoff_hz = get_parameter("baro_lpf_cutoff_hz").as_double();
    const bool   tof_lpf_enabled    = get_parameter("tof_lpf_enabled").as_bool();
    const double tof_lpf_cutoff_hz  = get_parameter("tof_lpf_cutoff_hz").as_double();

    // ── HandController ─────────────────────────────────────────────────
    const auto ft_names = get_parameter("hand_fingertip_names").as_string_array();
    controller_ = std::make_unique<urtc::HandController>(
        target_ip, target_port, urtc::kUdpRecvConfig, recv_timeout_ms,
        false /* enable_write_ack: deprecated */, 1,
        urtc::kDefaultNumFingertips, false, ft_names, comm_mode,
        tof_lpf_enabled, tof_lpf_cutoff_hz,
        baro_lpf_enabled, baro_lpf_cutoff_hz);

    controller_->SetCallback([this](const urtc::HandState& /*state*/) {
      data_received_.store(true, std::memory_order_relaxed);
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
      fd_cfg.check_link  = get_parameter("check_link").as_bool();
      fd_cfg.link_fail_threshold = static_cast<int>(
          get_parameter("link_fail_threshold").as_int());

      const auto cfgs = urtc::SelectThreadConfigs();
      failure_detector_ = std::make_unique<urtc::HandFailureDetector>(
          *controller_, fd_cfg, cfgs.hand_failure);
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
    link_status_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/hand/link_status", 10);
    link_fail_threshold_ = static_cast<uint64_t>(
        get_parameter("link_fail_threshold").as_int());

    command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/hand/command", 10,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          OnCommand(std::move(msg));
        });

    const auto period = std::chrono::microseconds(
        static_cast<int>(1'000'000.0 / rate));
    publish_timer_ = create_wall_timer(period, [this]() { PublishState(); });

    // Hand motor names 로드 및 로그
    auto motor_names = get_parameter("hand_motor_names").as_string_array();
    if (motor_names.empty()) { motor_names = urtc::kDefaultHandMotorNames; }
    auto fingertip_names = get_parameter("hand_fingertip_names").as_string_array();
    if (fingertip_names.empty()) { fingertip_names = urtc::kDefaultFingertipNames; }

    {
      std::string names_str;
      for (std::size_t i = 0; i < motor_names.size(); ++i) {
        if (i > 0) names_str += ", ";
        names_str += motor_names[i];
      }
      RCLCPP_INFO(get_logger(),
                  "Hand motor order (%zu): [%s]",
                  motor_names.size(), names_str.c_str());
    }

    RCLCPP_INFO(get_logger(),
                "HandUdpNode: target %s:%d, pub %.0f Hz, mode=%s",
                target_ip.c_str(), target_port, rate, comm_mode_str.c_str());
  }

  ~HandUdpNode() override {
    SaveCommStats();
    if (failure_detector_) failure_detector_->Stop();
    if (controller_) controller_->Stop();
  }

 private:
  void PublishState() {
    if (!data_received_.load(std::memory_order_relaxed)) return;

    const urtc::HandState snapshot = controller_->GetLatestState();

    // Publish: [10 positions] + [10 velocities] + [44 sensors]
    std_msgs::msg::Float64MultiArray msg;
    const int num_sensors = snapshot.num_fingertips * urtc::kSensorValuesPerFingertip;
    msg.data.reserve(static_cast<std::size_t>(urtc::kNumHandMotors * 2 + num_sensors));

    for (int i = 0; i < urtc::kNumHandMotors; ++i)
      msg.data.push_back(static_cast<double>(snapshot.motor_positions[static_cast<std::size_t>(i)]));
    for (int i = 0; i < urtc::kNumHandMotors; ++i)
      msg.data.push_back(static_cast<double>(snapshot.motor_velocities[static_cast<std::size_t>(i)]));
    for (int i = 0; i < num_sensors; ++i)
      msg.data.push_back(static_cast<double>(snapshot.sensor_data[static_cast<std::size_t>(i)]));

    state_pub_->publish(msg);

    // UDP link status 발행
    const uint64_t failures = controller_->consecutive_recv_failures();
    const bool link_ok = (failures < link_fail_threshold_);
    if (link_ok != prev_link_ok_) {
      if (link_ok) {
        RCLCPP_INFO(get_logger(), "Hand UDP link UP (recovered)");
      } else {
        RCLCPP_WARN(get_logger(),
                    "Hand UDP link DOWN (consecutive_recv_failures=%lu)",
                    static_cast<unsigned long>(failures));
      }
      prev_link_ok_ = link_ok;
    }
    std_msgs::msg::Bool link_msg;
    link_msg.data = link_ok;
    link_status_pub_->publish(link_msg);

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

    // 타이밍 통계
    const auto ts = controller_->timing_stats();

    const bool is_bulk = (controller_->communication_mode() == urtc::HandCommunicationMode::kBulk);
    const char* mode_str = is_bulk ? "bulk" : "individual";

    ofs << "{\n"
        << "  \"comm_stats\": {\n"
        << "    \"communication_mode\": \"" << mode_str << "\",\n"
        << "    \"recv_timeout_ms\": " << std::fixed << std::setprecision(3) << controller_->recv_timeout_ms() << ",\n"
        << "    \"total_cycles\": "    << stats.total_cycles   << ",\n"
        << "    \"recv_ok\": "         << stats.recv_ok        << ",\n"
        << "    \"recv_timeout\": "    << stats.recv_timeout    << ",\n"
        << "    \"recv_error\": "      << stats.recv_error      << ",\n"
        << "    \"event_skip_count\": " << stats.event_skip_count << ",\n"
        << "    \"avg_rate_hz\": "     << std::fixed << std::setprecision(2) << avg_rate_hz << ",\n"
        << "    \"elapsed_sec\": "     << std::fixed << std::setprecision(2) << elapsed_sec << ",\n"
        << "    \"failure_detected\": " << (fd_failed ? "true" : "false") << ",\n"
        << "    \"consecutive_recv_failures\": " << controller_->consecutive_recv_failures() << ",\n"
        << "    \"link_ok\": " << (controller_->consecutive_recv_failures() < link_fail_threshold_ ? "true" : "false") << "\n"
        << "  },\n"
        << "  \"timing_stats\": {\n"
        << "    \"count\": " << ts.count << ",\n"
        << "    \"total_us\": {"
        << " \"mean\": " << std::setprecision(1) << ts.mean_us
        << ", \"min\": " << ts.min_us
        << ", \"max\": " << ts.max_us
        << ", \"stddev\": " << ts.stddev_us
        << ", \"p95\": " << ts.p95_us
        << ", \"p99\": " << ts.p99_us
        << " },\n"
        << "    \"write_us\": {"
        << " \"mean\": " << ts.write.mean_us
        << ", \"min\": " << ts.write.min_us
        << ", \"max\": " << ts.write.max_us
        << " },\n";

    if (is_bulk) {
      ofs << "    \"read_all_motor_us\": {"
          << " \"mean\": " << ts.read_all_motor.mean_us
          << ", \"min\": " << ts.read_all_motor.min_us
          << ", \"max\": " << ts.read_all_motor.max_us
          << " },\n"
          << "    \"read_all_sensor_us\": {"
          << " \"mean\": " << ts.read_all_sensor.mean_us
          << ", \"min\": " << ts.read_all_sensor.min_us
          << ", \"max\": " << ts.read_all_sensor.max_us
          << ", \"sensor_cycles\": " << ts.sensor_cycle_count
          << " },\n";
    } else {
      ofs << "    \"read_pos_us\": {"
          << " \"mean\": " << ts.read_pos.mean_us
          << ", \"min\": " << ts.read_pos.min_us
          << ", \"max\": " << ts.read_pos.max_us
          << " },\n"
          << "    \"read_vel_us\": {"
          << " \"mean\": " << ts.read_vel.mean_us
          << ", \"min\": " << ts.read_vel.min_us
          << ", \"max\": " << ts.read_vel.max_us
          << " },\n"
          << "    \"read_sensor_us\": {"
          << " \"mean\": " << ts.read_sensor.mean_us
          << ", \"min\": " << ts.read_sensor.min_us
          << ", \"max\": " << ts.read_sensor.max_us
          << ", \"sensor_cycles\": " << ts.sensor_cycle_count
          << " },\n";
    }

    ofs << "    \"over_budget\": " << ts.over_budget << "\n"
        << "  }\n"
        << "}\n";
    ofs.close();

    // 타이밍 요약 로그 출력
    RCLCPP_INFO(get_logger(), "%s", controller_->TimingSummary().c_str());

    RCLCPP_INFO(get_logger(),
                "Hand stats saved to %s (cycles=%lu, ok=%lu, timeout=%lu, error=%lu, rate=%.1f Hz)",
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

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr link_status_pub_;
  uint64_t            link_fail_threshold_{10};
  bool                prev_link_ok_{true};

  std::atomic<bool>   data_received_{false};
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
