#include "ur5e_hand_driver/hand_controller.hpp"
#include "ur5e_hand_driver/hand_failure_detector.hpp"

#include <rtc_base/threading/thread_utils.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rtc_msgs/msg/joint_command.hpp>
#include <rtc_msgs/msg/hand_sensor_state.hpp>
#include <rtc_msgs/msg/fingertip_sensor.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pthread.h>   // pthread_setaffinity_np
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
namespace urtc = rtc;

// Unified hand UDP node using request-response protocol.
//
// Owns a HandController that polls the hand device:
//   write position → read position → read velocity → read sensors × 4
//
// Publishes full state directly from EventLoop callback (no timer).
// This eliminates the 100 Hz timer bottleneck — state is published at the
// EventLoop rate (500 Hz when driven by rt_controller).
//
// Pre-allocated messages avoid dynamic allocation on the publish path.
// Receives commands on /hand/command.
class HandUdpNode : public rclcpp::Node {
 public:
  HandUdpNode() : Node("hand_udp_node") {
    // ── Parameters ─────────────────────────────────────────────────────
    declare_parameter("target_ip",       std::string{"192.168.1.2"});
    declare_parameter("target_port",     55151);
    // publish_rate kept for backward compatibility but no longer used for timer.
    // Controls link_status decimation: link status published every (EventLoop Hz / publish_rate) cycles.
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

    // F/T inference
    declare_parameter("ft_inferencer.enabled", false);
    declare_parameter("ft_inferencer.num_fingertips", 4);
    declare_parameter("ft_inferencer.history_length", urtc::kFTHistoryLength);
    declare_parameter("ft_inferencer.model_paths", std::vector<std::string>{});
    declare_parameter("ft_inferencer.calibration_enabled", true);
    declare_parameter("ft_inferencer.calibration_samples", 500);
    // Per-fingertip normalization (input_max arrays)
    for (const auto& name : {"thumb", "index", "middle", "ring"}) {
      declare_parameter("ft_inferencer." + std::string(name) + "_max",
                        std::vector<double>(16, 1.0));
    }

    const std::string target_ip       = get_parameter("target_ip").as_string();
    const int         target_port     = static_cast<int>(get_parameter("target_port").as_int());
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

    // ── F/T Inferencer Config ────────────────────────────────────────
    urtc::FingertipFTInferencer::Config ft_config;
    ft_config.enabled = get_parameter("ft_inferencer.enabled").as_bool();
    ft_config.num_fingertips = static_cast<int>(
        get_parameter("ft_inferencer.num_fingertips").as_int());
    ft_config.history_length = static_cast<int>(
        get_parameter("ft_inferencer.history_length").as_int());
    ft_config.model_paths = get_parameter("ft_inferencer.model_paths").as_string_array();

    // Resolve relative model paths against ur5e_hand_driver package's models/ directory
    {
      const std::string models_dir =
          ament_index_cpp::get_package_share_directory("ur5e_hand_driver") + "/models/";
      for (auto& p : ft_config.model_paths) {
        if (!p.empty() && p[0] != '/') {
          p = models_dir + p;
        }
      }
    }

    ft_config.calibration_enabled = get_parameter("ft_inferencer.calibration_enabled").as_bool();
    ft_config.calibration_samples = static_cast<int>(
        get_parameter("ft_inferencer.calibration_samples").as_int());

    // Per-fingertip input_max 로드
    const std::array<const char*, 4> ft_param_names = {"thumb", "index", "middle", "ring"};
    for (int f = 0; f < 4 && f < ft_config.num_fingertips; ++f) {
      auto max_vec = get_parameter(
          "ft_inferencer." + std::string(ft_param_names[static_cast<std::size_t>(f)]) + "_max")
          .as_double_array();
      for (int b = 0; b < urtc::kFTInputSize && b < static_cast<int>(max_vec.size()); ++b) {
        ft_config.input_max[static_cast<std::size_t>(f)][static_cast<std::size_t>(b)] =
            static_cast<float>(max_vec[static_cast<std::size_t>(b)]);
      }
    }

    // ── HandController ─────────────────────────────────────────────────
    const auto ft_names = get_parameter("hand_fingertip_names").as_string_array();
    num_fingertips_ = urtc::kDefaultNumFingertips;
    controller_ = std::make_unique<urtc::HandController>(
        target_ip, target_port, urtc::kUdpRecvConfig, recv_timeout_ms,
        false /* enable_write_ack: deprecated */, 1,
        num_fingertips_, false, ft_names, comm_mode,
        tof_lpf_enabled, tof_lpf_cutoff_hz,
        baro_lpf_enabled, baro_lpf_cutoff_hz, ft_config);

    // ── Topic names (configurable) ────────────────────────────────────
    declare_parameter("command_topic", std::string("/hand/joint_command"));
    declare_parameter("state_topic", std::string("/hand/joint_states"));
    declare_parameter("sensor_topic", std::string("/hand/sensor_states"));
    declare_parameter("link_status_topic", std::string("/hand/link_status"));
    const std::string cmd_topic = get_parameter("command_topic").as_string();
    const std::string state_topic = get_parameter("state_topic").as_string();
    const std::string sensor_topic = get_parameter("sensor_topic").as_string();
    const std::string link_status_topic = get_parameter("link_status_topic").as_string();

    // ── ROS2 pub/sub ───────────────────────────────────────────────────
    // BEST_EFFORT + depth 1: 500 Hz RT sensor data — only latest value matters.
    // Eliminates DDS ACK/NACK/retransmit overhead on the EventLoop publish path.
    // Subscribers (rt_controller) MUST also use BEST_EFFORT to connect.
    rclcpp::QoS sensor_pub_qos{1};
    sensor_pub_qos.best_effort();

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        state_topic, sensor_pub_qos);
    sensor_state_pub_ = create_publisher<rtc_msgs::msg::HandSensorState>(
        sensor_topic, sensor_pub_qos);

    // Link status: RELIABLE + TRANSIENT_LOCAL + depth 1.
    // Low-rate (~100 Hz decimated), latch ensures late subscribers get last status.
    rclcpp::QoS link_pub_qos{1};
    link_pub_qos.transient_local();
    link_status_pub_ = create_publisher<std_msgs::msg::Bool>(
        link_status_topic, link_pub_qos);
    link_fail_threshold_ = static_cast<uint64_t>(
        get_parameter("link_fail_threshold").as_int());

    ft_enabled_ = ft_config.enabled;

    // ── Hand motor/fingertip names ──────────────────────────────────────
    auto motor_names = get_parameter("hand_motor_names").as_string_array();
    if (motor_names.empty()) { motor_names = urtc::kDefaultHandMotorNames; }
    joint_names_ = motor_names;
    auto fingertip_names = get_parameter("hand_fingertip_names").as_string_array();
    if (fingertip_names.empty()) { fingertip_names = urtc::kDefaultFingertipNames; }
    fingertip_names_ = fingertip_names;

    // ── Link status decimation ─────────────────────────────────────────
    // link_status는 500Hz로 보낼 필요 없음 — publish_rate 기준으로 decimation.
    const double publish_rate = get_parameter("publish_rate").as_double();
    // EventLoop rate assumed 500Hz; decimation = 500/publish_rate (e.g. 500/100=5)
    link_decimation_ = std::max(1, static_cast<int>(500.0 / publish_rate));

    // ── Pre-allocate ROS2 messages (non-RT, 1회만 할당) ─────────────────
    PreallocateMessages();

    // ── Set EventLoop callback: direct publish (no timer) ───────────────
    controller_->SetCallback(
        [this](const urtc::HandState& state,
               const urtc::FingertipFTState& ft_state) {
          PublishFromEventLoop(state, ft_state);
        });

    // ── Subscriptions (Start 실패와 무관하게 항상 생성) ────────────────
    // Start() 실패 시에도 구독을 생성하여 ros2 topic info에서 subscriber 확인 가능.
    // 구독이 없으면 subscriber=0으로 표시되어 문제 진단이 어려워짐.

    // JointCommand subscription (from rt_controller or external)
    // rt_controller publishes with BEST_EFFORT + depth 1 for minimal DDS overhead.
    // Subscriber must match: RELIABLE sub cannot connect to BEST_EFFORT pub.
    // BEST_EFFORT + depth 1: rt_controller publishes with BEST_EFFORT/1,
    // only the latest command matters — stale commands in queue cause lag.
    rclcpp::QoS cmd_sub_qos{1};
    cmd_sub_qos.best_effort();

    joint_command_sub_ = create_subscription<rtc_msgs::msg::JointCommand>(
        cmd_topic, cmd_sub_qos,
        [this](rtc_msgs::msg::JointCommand::SharedPtr msg) {
          if (!controller_->IsRunning()) {
            return;  // Start 실패 시 명령 무시 (EventLoop 미동작)
          }
          if (msg->values.size() < static_cast<std::size_t>(urtc::kNumHandMotors)) {
            RCLCPP_WARN(get_logger(),
                        "JointCommand values size %zu (expected %d)",
                        msg->values.size(), urtc::kNumHandMotors);
            return;
          }
          std::array<float, urtc::kNumHandMotors> cmd;
          for (std::size_t i = 0; i < static_cast<std::size_t>(urtc::kNumHandMotors); ++i) {
            cmd[i] = static_cast<float>(msg->values[i]);
          }
          controller_->SetTargetPositions(cmd);
        });

    // Legacy Float64MultiArray subscription (/hand/command) removed.
    // rt_controller and mujoco_sim both use JointCommand on /hand/joint_command.
    // See: git log for migration history.

    if (!controller_->Start()) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to start HandController to %s:%d  "
                   "(subscriptions created — check ros2 topic info for diagnostics)",
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
          *controller_, fd_cfg, cfgs.status_monitor);
      failure_detector_->SetFailureCallback(
          [this](const std::string& reason) {
            RCLCPP_ERROR(get_logger(), "Hand failure detected: %s", reason.c_str());
          });
      failure_detector_->Start();
      RCLCPP_INFO(get_logger(), "HandFailureDetector started (50 Hz, threshold=%d)",
                  fd_cfg.failure_threshold);
    }

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
                "HandUdpNode: target %s:%d, direct publish from EventLoop, mode=%s",
                target_ip.c_str(), target_port, comm_mode_str.c_str());
  }

  ~HandUdpNode() override {
    SaveCommStats();
    if (failure_detector_) failure_detector_->Stop();
    if (controller_) controller_->Stop();
  }

 private:
  // Pre-allocate ROS2 messages once in the constructor (non-RT).
  // This avoids dynamic allocation on the EventLoop publish path.
  void PreallocateMessages() {
    // JointState message
    js_msg_.name = joint_names_;
    js_msg_.position.resize(urtc::kNumHandMotors);
    js_msg_.velocity.resize(urtc::kNumHandMotors);

    // HandSensorState message — pre-allocate fingertips array
    sensor_msg_.fingertips.resize(static_cast<std::size_t>(num_fingertips_));
    for (int f = 0; f < num_fingertips_; ++f) {
      auto& fs = sensor_msg_.fingertips[static_cast<std::size_t>(f)];
      fs.name = (static_cast<std::size_t>(f) < fingertip_names_.size())
          ? fingertip_names_[static_cast<std::size_t>(f)]
          : "f" + std::to_string(f);
    }
  }

  // Called directly from EventLoop thread — publishes state at EventLoop rate.
  // Uses pre-allocated messages to avoid dynamic allocation.
  //
  // NOTE: publish() may trigger DDS serialization which is not strictly RT-safe,
  // but the overhead is small (~10-20µs for these message sizes) and acceptable
  // within the 2ms EventLoop budget.
  void PublishFromEventLoop(const urtc::HandState& state,
                            const urtc::FingertipFTState& ft_state) {
    // ── JointState (motor positions + velocities) ──────────────────────
    js_msg_.header.stamp = this->now();
    for (int i = 0; i < urtc::kNumHandMotors; ++i) {
      const auto iu = static_cast<std::size_t>(i);
      js_msg_.position[iu] =
          static_cast<double>(state.motor_positions[iu]);
      js_msg_.velocity[iu] =
          static_cast<double>(state.motor_velocities[iu]);
    }
    joint_state_pub_->publish(js_msg_);

    // ── HandSensorState (sensor + inference) ───────────────────────────
    if (state.num_fingertips > 0) {
      sensor_msg_.header.stamp = this->now();

      const bool ft_valid = ft_enabled_ && ft_state.valid;

      for (int f = 0; f < state.num_fingertips &&
           f < static_cast<int>(sensor_msg_.fingertips.size()); ++f) {
        auto& fs = sensor_msg_.fingertips[static_cast<std::size_t>(f)];

        const int sensor_base = f * urtc::kSensorValuesPerFingertip;
        for (int b = 0; b < urtc::kBarometerCount; ++b) {
          const auto bu = static_cast<std::size_t>(b);
          const auto si = static_cast<std::size_t>(sensor_base + b);
          fs.barometer[bu] = static_cast<float>(state.sensor_data[si]);
          fs.barometer_raw[bu] = static_cast<float>(state.sensor_data_raw[si]);
        }
        for (int t = 0; t < urtc::kTofCount; ++t) {
          const auto tu = static_cast<std::size_t>(t);
          const auto si = static_cast<std::size_t>(
              sensor_base + urtc::kBarometerCount + t);
          fs.tof[tu] = static_cast<float>(state.sensor_data[si]);
          fs.tof_raw[tu] = static_cast<float>(state.sensor_data_raw[si]);
        }

        if (ft_valid && f < ft_state.num_fingertips
            && ft_state.per_fingertip_valid[static_cast<std::size_t>(f)]) {
          const int ft_base = f * urtc::kFTValuesPerFingertip;
          fs.inference_enable = true;
          fs.contact_flag = ft_state.ft_data[static_cast<std::size_t>(ft_base)];
          for (int j = 0; j < 3; ++j) {
            const auto ju = static_cast<std::size_t>(j);
            fs.f[ju] = ft_state.ft_data[static_cast<std::size_t>(ft_base + 1 + j)];
            fs.u[ju] = ft_state.ft_data[static_cast<std::size_t>(ft_base + 4 + j)];
          }
        } else {
          fs.inference_enable = false;
          fs.contact_flag = 0.0f;
          fs.f = {};
          fs.u = {};
        }
      }
      sensor_state_pub_->publish(sensor_msg_);
    }

    // ── Link status (decimated — not every cycle) ──────────────────────
    ++link_cycle_counter_;
    if (link_cycle_counter_ >= link_decimation_) {
      link_cycle_counter_ = 0;

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
      link_msg_.data = link_ok;
      link_status_pub_->publish(link_msg_);
    }

    if (++publish_count_ % 500 == 0) {
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

    // F/T inference timing
    if (ts.ft_infer_count > 0) {
      ofs << "    \"ft_infer_us\": {"
          << " \"mean\": " << ts.ft_infer.mean_us
          << ", \"min\": " << ts.ft_infer.min_us
          << ", \"max\": " << ts.ft_infer.max_us
          << ", \"count\": " << ts.ft_infer_count
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

  std::unique_ptr<urtc::HandController>      controller_;
  std::unique_ptr<urtc::HandFailureDetector> failure_detector_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr         joint_state_pub_;
  rclcpp::Publisher<rtc_msgs::msg::HandSensorState>::SharedPtr      sensor_state_pub_;
  rclcpp::Subscription<rtc_msgs::msg::JointCommand>::SharedPtr      joint_command_sub_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> fingertip_names_;
  int num_fingertips_{urtc::kDefaultNumFingertips};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr link_status_pub_;
  bool ft_enabled_{false};
  uint64_t            link_fail_threshold_{10};
  bool                prev_link_ok_{true};

  // Pre-allocated messages (populated once in constructor, values overwritten per cycle)
  sensor_msgs::msg::JointState           js_msg_;
  rtc_msgs::msg::HandSensorState         sensor_msg_;
  std_msgs::msg::Bool                    link_msg_;

  // Link status decimation (500Hz → publish_rate Hz)
  int link_decimation_{5};     // default: 500/100 = 5
  int link_cycle_counter_{0};

  std::size_t         publish_count_{0};

  std::chrono::steady_clock::time_point start_time_{std::chrono::steady_clock::now()};
};

int main(int argc, char** argv) {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    fprintf(stderr, "[WARN] hand_udp_node: mlockall failed\n");
  }

  // Pin main thread (ROS2 executor, DDS) to Core 0-1 (OS/DDS cores).
  // EventLoop thread has its own affinity (Core 5, SCHED_FIFO 65).
  // Without this, the executor could land on isolated RT cores (2-5).
  {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    CPU_SET(1, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
      fprintf(stderr, "[WARN] hand_udp_node: main thread CPU affinity failed\n");
    }
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandUdpNode>());
  rclcpp::shutdown();
  return 0;
}
