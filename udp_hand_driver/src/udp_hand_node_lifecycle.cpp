#include "udp_hand_driver/udp_hand_logging.hpp"
#include "udp_hand_driver/udp_hand_node.hpp"
#include <rtc_base/logging/session_dir.hpp>
#include <rtc_base/threading/thread_utils.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <system_error>

using namespace std::chrono_literals;

UdpHandNode::UdpHandNode() : LifecycleNode("udp_hand_node") {
  // Lifecycle design: constructor is intentionally empty.
  // All resource allocation happens in on_configure().
}

UdpHandNode::~UdpHandNode() {
  // Safety net — idempotent cleanup in case lifecycle callbacks were not
  // invoked (e.g. SIGTERM without graceful shutdown).
  if (failure_detector_)
    failure_detector_->Stop();
  if (controller_ && controller_->IsRunning())
    controller_->Stop();
  SaveCommStats();
}

UdpHandNode::CallbackReturn UdpHandNode::on_configure(const rclcpp_lifecycle::State& /*state*/) {
  // ── Parameters ─────────────────────────────────────────────────────
  declare_parameter("target_ip", std::string{"192.168.1.2"});
  declare_parameter("target_port", 55151);
  declare_parameter("publish_rate", 100.0);
  declare_parameter("recv_timeout_ms", 10.0);
  declare_parameter("enable_write_ack", false);
  declare_parameter("enable_failure_detector", true);
  declare_parameter("failure_threshold", 5);
  declare_parameter("check_motor", true);
  declare_parameter("check_sensor", true);
  declare_parameter("min_rate_hz", 30.0);
  declare_parameter("rate_fail_threshold", 5);
  declare_parameter("check_link", true);
  declare_parameter("link_fail_threshold", 10);

  declare_parameter("use_fake_hand", false);
  declare_parameter("fake_tick_rate_hz", 500.0);

  declare_parameter("joint_state_names", std::vector<std::string>{});
  declare_parameter("motor_state_names", std::vector<std::string>{});
  declare_parameter("hand_fingertip_names", std::vector<std::string>{});

  declare_parameter("communication_mode", std::string{"individual"});

  declare_parameter("baro_lpf_enabled", false);
  declare_parameter("baro_lpf_cutoff_hz", 30.0);
  declare_parameter("tof_lpf_enabled", false);
  declare_parameter("tof_lpf_cutoff_hz", 15.0);

  declare_parameter("drift_detection_enabled", false);
  declare_parameter("drift_threshold", 5.0);
  declare_parameter("drift_window_size", 2500);

  declare_parameter("ft_inferencer.enabled", false);
  declare_parameter("ft_inferencer.num_fingertips", 4);
  declare_parameter("ft_inferencer.history_length", udp_hand_driver::kFTHistoryLength);
  declare_parameter("ft_inferencer.model_paths", std::vector<std::string>{});
  declare_parameter("ft_inferencer.calibration_enabled", true);
  declare_parameter("ft_inferencer.calibration_samples", 500);
  for (const auto& name : {"thumb", "index", "middle", "ring"}) {
    declare_parameter("ft_inferencer." + std::string(name) + "_max", std::vector<double>(16, 1.0));
  }

  const std::string target_ip = get_parameter("target_ip").as_string();
  const int target_port = static_cast<int>(get_parameter("target_port").as_int());
  const double recv_timeout_ms = get_parameter("recv_timeout_ms").as_double();
  const std::string comm_mode_str = get_parameter("communication_mode").as_string();
  const auto comm_mode = (comm_mode_str == "bulk")
                             ? udp_hand_driver::HandCommunicationMode::kBulk
                             : udp_hand_driver::HandCommunicationMode::kIndividual;

  const bool baro_lpf_enabled = get_parameter("baro_lpf_enabled").as_bool();
  const double baro_lpf_cutoff_hz = get_parameter("baro_lpf_cutoff_hz").as_double();
  const bool tof_lpf_enabled = get_parameter("tof_lpf_enabled").as_bool();
  const double tof_lpf_cutoff_hz = get_parameter("tof_lpf_cutoff_hz").as_double();

  udp_hand_driver::FingertipFTInferencer::Config ft_config;
  ft_config.enabled = get_parameter("ft_inferencer.enabled").as_bool();
  ft_config.num_fingertips =
      static_cast<int>(get_parameter("ft_inferencer.num_fingertips").as_int());
  ft_config.history_length =
      static_cast<int>(get_parameter("ft_inferencer.history_length").as_int());
  ft_config.model_paths = get_parameter("ft_inferencer.model_paths").as_string_array();

  {
    const std::string models_dir =
        ament_index_cpp::get_package_share_directory("udp_hand_driver") + "/models/";
    for (auto& p : ft_config.model_paths) {
      if (!p.empty() && p[0] != '/') {
        p = models_dir + p;
      }
    }
  }

  ft_config.calibration_enabled = get_parameter("ft_inferencer.calibration_enabled").as_bool();
  ft_config.calibration_samples =
      static_cast<int>(get_parameter("ft_inferencer.calibration_samples").as_int());

  const std::array<const char*, 4> ft_param_names = {"thumb", "index", "middle", "ring"};
  for (int f = 0; f < 4 && f < ft_config.num_fingertips; ++f) {
    auto max_vec = get_parameter("ft_inferencer." +
                                 std::string(ft_param_names[static_cast<std::size_t>(f)]) + "_max")
                       .as_double_array();
    for (int b = 0; b < udp_hand_driver::kFTInputSize && b < static_cast<int>(max_vec.size());
         ++b) {
      ft_config.input_max[static_cast<std::size_t>(f)][static_cast<std::size_t>(b)] =
          static_cast<float>(max_vec[static_cast<std::size_t>(b)]);
    }
  }

  const bool drift_enabled = get_parameter("drift_detection_enabled").as_bool();
  const double drift_threshold = get_parameter("drift_threshold").as_double();
  const int drift_window_size = static_cast<int>(get_parameter("drift_window_size").as_int());

  // ── UdpHandController ─────────────────────────────────────────────────
  const auto ft_names = get_parameter("hand_fingertip_names").as_string_array();
  num_fingertips_ = udp_hand_driver::kDefaultNumFingertips;
  use_fake_hand_ = get_parameter("use_fake_hand").as_bool();
  controller_ = std::make_unique<udp_hand_driver::UdpHandController>(
      target_ip, target_port, rtc::kUdpRecvConfig, recv_timeout_ms,
      false /* enable_write_ack: deprecated */, 1, num_fingertips_, use_fake_hand_, ft_names,
      comm_mode, tof_lpf_enabled, tof_lpf_cutoff_hz, baro_lpf_enabled, baro_lpf_cutoff_hz,
      ft_config, drift_enabled, drift_threshold, drift_window_size);

  // ── Topic names ──────────────────────────────────────────────────
  declare_parameter("command_topic", std::string("/hand/joint_command"));
  declare_parameter("joint_state_topic", std::string("/hand/joint_states"));
  declare_parameter("motor_state_topic", std::string("/hand/motor_states"));
  declare_parameter("sensor_topic", std::string("/hand/sensor_states"));
  declare_parameter("link_status_topic", std::string("/hand/link_status"));
  declare_parameter("calibration_command_topic", std::string("/hand/calibration/command"));
  declare_parameter("calibration_status_topic", std::string("/hand/calibration/status"));
  declare_parameter("calibration_status_rate_hz", 5.0);
  const std::string cmd_topic = get_parameter("command_topic").as_string();
  const std::string joint_state_topic = get_parameter("joint_state_topic").as_string();
  const std::string motor_state_topic = get_parameter("motor_state_topic").as_string();
  const std::string sensor_topic = get_parameter("sensor_topic").as_string();
  const std::string link_status_topic = get_parameter("link_status_topic").as_string();
  const std::string calib_cmd_topic = get_parameter("calibration_command_topic").as_string();
  const std::string calib_status_topic = get_parameter("calibration_status_topic").as_string();
  const double calib_status_rate_hz = get_parameter("calibration_status_rate_hz").as_double();

  // ── Publishers (LifecyclePublisher — gated by lifecycle state) ─────
  rclcpp::QoS sensor_pub_qos{1};
  sensor_pub_qos.best_effort();

  joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, sensor_pub_qos);
  motor_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>(motor_state_topic, sensor_pub_qos);
  sensor_state_pub_ =
      create_publisher<rtc_msgs::msg::HandSensorState>(sensor_topic, sensor_pub_qos);

  sensor_monitor_pub_ =
      create_publisher<rtc_msgs::msg::HandSensorState>(sensor_topic + "/monitor", rclcpp::QoS{10});

  // Link status: standalone rclcpp::Publisher (NOT LifecyclePublisher).
  // Must remain publishable in any lifecycle state — link status is a
  // safety-relevant signal that should report even when Inactive.
  rclcpp::QoS link_pub_qos{1};
  link_pub_qos.transient_local();
  link_status_pub_ = rclcpp::create_publisher<std_msgs::msg::Bool>(
      this->get_node_topics_interface(), link_status_topic, link_pub_qos);
  link_fail_threshold_ = static_cast<uint64_t>(get_parameter("link_fail_threshold").as_int());

  ft_enabled_ = ft_config.enabled;

  // ── Hand joint/motor/fingertip names ────────────────────────────────
  auto joint_names = get_parameter("joint_state_names").as_string_array();
  if (joint_names.empty()) {
    joint_names = udp_hand_driver::kDefaultHandMotorNames;
  }
  joint_names_ = joint_names;
  auto motor_names = get_parameter("motor_state_names").as_string_array();
  if (motor_names.empty()) {
    motor_names = udp_hand_driver::kDefaultHandMotorNames;
  }
  motor_names_ = motor_names;
  auto fingertip_names = get_parameter("hand_fingertip_names").as_string_array();
  if (fingertip_names.empty()) {
    fingertip_names = udp_hand_driver::kDefaultFingertipNames;
  }
  fingertip_names_ = fingertip_names;

  // ── Link status decimation ─────────────────────────────────────────
  const double publish_rate = get_parameter("publish_rate").as_double();
  link_decimation_ = std::max(1, static_cast<int>(500.0 / publish_rate));

  // ── Pre-allocate ROS2 messages ─────────────────────────────────────
  PreallocateMessages();

  // ── EventLoop callback ─────────────────────────────────────────────
  controller_->SetCallback([this](const udp_hand_driver::UdpHandState& state,
                                  const udp_hand_driver::FingertipFTState& ft_state) {
    PublishFromEventLoop(state, ft_state);
  });

  // ── Subscriptions ──────────────────────────────────────────────────
  rclcpp::QoS cmd_sub_qos{1};
  cmd_sub_qos.best_effort();

  joint_command_sub_ = create_subscription<rtc_msgs::msg::JointCommand>(
      cmd_topic, cmd_sub_qos, [this](rtc_msgs::msg::JointCommand::SharedPtr msg) {
        if (!controller_->IsRunning()) {
          return;
        }
        if (msg->values.size() < static_cast<std::size_t>(udp_hand_driver::kNumHandMotors)) {
          RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                      "JointCommand values size %zu (expected %d)", msg->values.size(),
                      udp_hand_driver::kNumHandMotors);
          return;
        }
        std::array<float, udp_hand_driver::kNumHandMotors> cmd;
        for (std::size_t i = 0; i < static_cast<std::size_t>(udp_hand_driver::kNumHandMotors);
             ++i) {
          cmd[i] = static_cast<float>(msg->values[i]);
        }
        if (use_fake_hand_) {
          std::lock_guard lock(last_cmd_mutex_);
          last_cmd_ = cmd;
        }
        controller_->SetTargetPositions(cmd);
      });

  calib_cmd_sub_ = create_subscription<rtc_msgs::msg::CalibrationCommand>(
      calib_cmd_topic, rclcpp::QoS(1).reliable(),
      [this](rtc_msgs::msg::CalibrationCommand::SharedPtr msg) {
        if (!controller_ || !controller_->IsRunning()) {
          RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                      "Calibration command ignored: controller not running");
          return;
        }
        RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(),
                    "Calibration command: sensor_type=%u action=%u sample_count=%u",
                    static_cast<unsigned>(msg->sensor_type), static_cast<unsigned>(msg->action),
                    static_cast<unsigned>(msg->sample_count));
        controller_->RequestCalibration(msg->sensor_type, msg->action, msg->sample_count);
      });

  rclcpp::QoS calib_status_qos{1};
  calib_status_qos.reliable().transient_local();
  calib_status_pub_ =
      create_publisher<rtc_msgs::msg::CalibrationStatus>(calib_status_topic, calib_status_qos);

  const int status_period_ms =
      (calib_status_rate_hz > 0.0) ? static_cast<int>(1000.0 / calib_status_rate_hz) : 200;
  calib_status_timer_ = create_wall_timer(std::chrono::milliseconds(status_period_ms),
                                          [this]() { PublishCalibrationStatus(); });

  // Cache config values needed by on_activate
  target_ip_ = target_ip;
  target_port_ = target_port;
  comm_mode_str_ = comm_mode_str;

  {
    std::string names_str;
    for (std::size_t i = 0; i < motor_names.size(); ++i) {
      if (i > 0)
        names_str += ", ";
      names_str += motor_names[i];
    }
    RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "Hand motor order (%zu): [%s]",
                motor_names.size(), names_str.c_str());
  }

  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(),
              "UdpHandNode configured: target %s:%d, direct publish from "
              "EventLoop, comm=%s",
              target_ip.c_str(), target_port, comm_mode_str.c_str());
  return CallbackReturn::SUCCESS;
}

UdpHandNode::CallbackReturn UdpHandNode::on_activate(const rclcpp_lifecycle::State& state) {
  // Parent call activates LifecyclePublishers — must be called first.
  LifecycleNode::on_activate(state);

  start_time_ = std::chrono::steady_clock::now();

  // ── Per-tick timing CSV setup ──────────────────────────────────────
  // Inject producer before Start() so the EventLoop thread sees a non-null
  // producer on its first iteration. Expected period is the publish_rate
  // setpoint (set at on_configure); jitter is computed against it.
  {
    const double publish_rate = get_parameter("publish_rate").as_double();
    const double expected_period_us = (publish_rate > 0.0) ? (1.0e6 / publish_rate) : 0.0;
    controller_->SetTimingProducer(&hand_udp_timing_producer_, expected_period_us);
  }

  if (!controller_->Start()) {
    RCLCPP_ERROR(::udp_hand_driver::logging::NodeLogger(),
                 "Failed to start UdpHandController to %s:%d", target_ip_.c_str(), target_port_);
    return CallbackReturn::FAILURE;
  }

  // Fake-hand self tick
  if (use_fake_hand_) {
    const double fake_rate = get_parameter("fake_tick_rate_hz").as_double();
    if (fake_rate > 0.0) {
      const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1.0e9 / fake_rate));
      fake_tick_timer_ = create_wall_timer(period, [this]() {
        if (!controller_ || !controller_->IsRunning())
          return;
        std::array<float, udp_hand_driver::kNumHandMotors> cmd;
        {
          std::lock_guard lock(last_cmd_mutex_);
          cmd = last_cmd_;
        }
        controller_->SendCommandAndRequestStates(cmd);
      });
      RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                  "UdpHandNode: FAKE mode enabled — no UDP, self-tick=%.1f Hz", fake_rate);
    } else {
      RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                  "UdpHandNode: FAKE mode enabled — self-tick disabled "
                  "(fake_tick_rate_hz=%.1f). Drive SendCommandAndRequestStates "
                  "externally (e.g. rtc_controller_manager ControlLoop).",
                  fake_rate);
    }
  }

  // Failure Detector (skipped in fake mode)
  const bool enable_fd = get_parameter("enable_failure_detector").as_bool() && !use_fake_hand_;
  if (enable_fd) {
    udp_hand_driver::UdpHandFailureDetector::Config fd_cfg;
    fd_cfg.failure_threshold = static_cast<int>(get_parameter("failure_threshold").as_int());
    fd_cfg.check_motor = get_parameter("check_motor").as_bool();
    fd_cfg.check_sensor = get_parameter("check_sensor").as_bool();
    fd_cfg.min_rate_hz = get_parameter("min_rate_hz").as_double();
    fd_cfg.rate_fail_threshold = static_cast<int>(get_parameter("rate_fail_threshold").as_int());
    fd_cfg.check_link = get_parameter("check_link").as_bool();
    fd_cfg.link_fail_threshold = static_cast<int>(get_parameter("link_fail_threshold").as_int());

    const auto cfgs = rtc::SelectThreadConfigs();
    failure_detector_ = std::make_unique<udp_hand_driver::UdpHandFailureDetector>(
        *controller_, fd_cfg, cfgs.logging);
    failure_detector_->SetFailureCallback([this](const std::string& reason) {
      RCLCPP_ERROR(::udp_hand_driver::logging::NodeLogger(), "Hand failure detected: %s",
                   reason.c_str());
    });
    failure_detector_->Start();
    RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(),
                "UdpHandFailureDetector started (50 Hz, threshold=%d)", fd_cfg.failure_threshold);
  }

  // ── Per-tick timing CSV: open + start drain timer ──────────────────
  if (!hand_udp_timing_initialized_) {
    if (!hand_udp_timing_logger_.Open()) {
      RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                  "UdpHandTimingLogger::Open() failed — timing CSV disabled");
    } else {
      RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "Hand UDP tick-timing CSV: %s",
                  hand_udp_timing_logger_.Path().c_str());
    }
    hand_udp_timing_timer_ =
        create_wall_timer(std::chrono::seconds(1), [this]() { DrainHandUdpTiming(); });
    hand_udp_timing_initialized_ = true;
  }

  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "UdpHandNode activated");
  return CallbackReturn::SUCCESS;
}

void UdpHandNode::DrainHandUdpTiming() noexcept {
  if (!controller_)
    return;
  hand_udp_timing_producer_.Drain(
      [this](const rtc::RtTickTimingSample& s) { hand_udp_timing_logger_.Log(s); });
  const std::uint64_t drops = hand_udp_timing_producer_.DropCount();
  if (drops > hand_udp_timing_drop_baseline_) {
    const std::uint64_t delta = drops - hand_udp_timing_drop_baseline_;
    hand_udp_timing_drop_baseline_ = drops;
    RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                "Hand UDP timing SPSC dropped %lu samples since last drain",
                static_cast<unsigned long>(delta));
  }
}

UdpHandNode::CallbackReturn UdpHandNode::on_deactivate(const rclcpp_lifecycle::State& state) {
  if (failure_detector_) {
    failure_detector_->Stop();
    failure_detector_.reset();
  }
  if (controller_ && controller_->IsRunning()) {
    controller_->Stop();
  }
  if (controller_) {
    // Drop the producer pointer so a stopped EventLoop cannot push into a
    // potentially-recreated buffer on a future activation.
    controller_->SetTimingProducer(nullptr, 0.0);
  }
  if (fake_tick_timer_) {
    fake_tick_timer_->cancel();
    fake_tick_timer_.reset();
  }
  // Drain any stragglers so the CSV reflects everything pushed before stop.
  if (hand_udp_timing_initialized_) {
    DrainHandUdpTiming();
  }

  // Parent call deactivates LifecyclePublishers.
  LifecycleNode::on_deactivate(state);

  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "UdpHandNode deactivated");
  return CallbackReturn::SUCCESS;
}

UdpHandNode::CallbackReturn UdpHandNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/) {
  SaveCommStats();

  calib_status_timer_.reset();
  calib_cmd_sub_.reset();
  joint_command_sub_.reset();
  link_status_pub_.reset();
  sensor_monitor_pub_.reset();
  sensor_state_pub_.reset();
  motor_state_pub_.reset();
  joint_state_pub_.reset();
  calib_status_pub_.reset();
  controller_.reset();

  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "UdpHandNode cleaned up");
  return CallbackReturn::SUCCESS;
}

UdpHandNode::CallbackReturn UdpHandNode::on_shutdown(const rclcpp_lifecycle::State& state) {
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(state);
  }
  return on_cleanup(state);
}

UdpHandNode::CallbackReturn UdpHandNode::on_error(const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_ERROR(::udp_hand_driver::logging::NodeLogger(), "UdpHandNode error — attempting recovery");
  if (failure_detector_) {
    failure_detector_->Stop();
    failure_detector_.reset();
  }
  if (controller_ && controller_->IsRunning()) {
    controller_->Stop();
  }
  if (fake_tick_timer_) {
    fake_tick_timer_->cancel();
    fake_tick_timer_.reset();
  }
  SaveCommStats();

  calib_status_timer_.reset();
  calib_cmd_sub_.reset();
  joint_command_sub_.reset();
  link_status_pub_.reset();
  sensor_monitor_pub_.reset();
  sensor_state_pub_.reset();
  motor_state_pub_.reset();
  joint_state_pub_.reset();
  calib_status_pub_.reset();
  controller_.reset();

  // SUCCESS -> Unconfigured (recoverable), not Finalized.
  return CallbackReturn::SUCCESS;
}

void UdpHandNode::SaveCommStats() const {
  if (!controller_)
    return;

  const auto stats = controller_->comm_stats();
  const bool fd_failed = failure_detector_ ? failure_detector_->failed() : false;

  const auto elapsed = std::chrono::steady_clock::now() - start_time_;
  const double elapsed_sec = std::chrono::duration<double>(elapsed).count();
  const double avg_rate_hz =
      (elapsed_sec > 0.0) ? static_cast<double>(stats.total_cycles) / elapsed_sec : 0.0;

  const std::filesystem::path session = rtc::ResolveSessionDir();
  std::filesystem::path output_dir_path = session / "device";
  std::error_code dir_ec;
  std::filesystem::create_directories(output_dir_path, dir_ec);
  const std::string output_dir = output_dir_path.string();

  const std::string path = output_dir + "/hand_udp_stats.json";
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(), "Failed to save hand stats to %s",
                path.c_str());
    return;
  }

  const auto ts = controller_->timing_stats();

  const bool is_bulk =
      (controller_->communication_mode() == udp_hand_driver::HandCommunicationMode::kBulk);
  const char* mode_str = is_bulk ? "bulk" : "individual";

  ofs << "{\n"
      << "  \"comm_stats\": {\n"
      << "    \"communication_mode\": \"" << mode_str << "\",\n"
      << "    \"recv_timeout_ms\": " << std::fixed << std::setprecision(3)
      << controller_->recv_timeout_ms() << ",\n"
      << "    \"total_cycles\": " << stats.total_cycles << ",\n"
      << "    \"recv_ok\": " << stats.recv_ok << ",\n"
      << "    \"recv_timeout\": " << stats.recv_timeout << ",\n"
      << "    \"recv_error\": " << stats.recv_error << ",\n"
      << "    \"event_skip_count\": " << stats.event_skip_count << ",\n"
      << "    \"avg_rate_hz\": " << std::fixed << std::setprecision(2) << avg_rate_hz << ",\n"
      << "    \"elapsed_sec\": " << std::fixed << std::setprecision(2) << elapsed_sec << ",\n"
      << "    \"failure_detected\": " << (fd_failed ? "true" : "false") << ",\n"
      << "    \"consecutive_recv_failures\": " << controller_->consecutive_recv_failures() << ",\n"
      << "    \"link_ok\": "
      << (controller_->consecutive_recv_failures() < link_fail_threshold_ ? "true" : "false")
      << "\n"
      << "  },\n"
      << "  \"timing_stats\": {\n"
      << "    \"count\": " << ts.count << ",\n"
      << "    \"total_us\": {" << " \"mean\": " << std::setprecision(1) << ts.mean_us
      << ", \"min\": " << ts.min_us << ", \"max\": " << ts.max_us
      << ", \"stddev\": " << ts.stddev_us << ", \"p95\": " << ts.p95_us
      << ", \"p99\": " << ts.p99_us << " },\n"
      << "    \"write_us\": {" << " \"mean\": " << ts.write.mean_us
      << ", \"min\": " << ts.write.min_us << ", \"max\": " << ts.write.max_us << " },\n";

  if (is_bulk) {
    ofs << "    \"read_all_motor_us\": {" << " \"mean\": " << ts.read_all_motor.mean_us
        << ", \"min\": " << ts.read_all_motor.min_us << ", \"max\": " << ts.read_all_motor.max_us
        << " },\n"
        << "    \"read_all_joint_motor_us\": {" << " \"mean\": " << ts.read_all_joint_motor.mean_us
        << ", \"min\": " << ts.read_all_joint_motor.min_us
        << ", \"max\": " << ts.read_all_joint_motor.max_us << " },\n"
        << "    \"read_all_sensor_us\": {" << " \"mean\": " << ts.read_all_sensor.mean_us
        << ", \"min\": " << ts.read_all_sensor.min_us << ", \"max\": " << ts.read_all_sensor.max_us
        << ", \"sensor_cycles\": " << ts.sensor_cycle_count << " },\n";
  } else {
    ofs << "    \"read_pos_us\": {" << " \"mean\": " << ts.read_pos.mean_us
        << ", \"min\": " << ts.read_pos.min_us << ", \"max\": " << ts.read_pos.max_us << " },\n"
        << "    \"read_joint_pos_us\": {" << " \"mean\": " << ts.read_joint_pos.mean_us
        << ", \"min\": " << ts.read_joint_pos.min_us << ", \"max\": " << ts.read_joint_pos.max_us
        << " },\n"
        << "    \"read_vel_us\": {" << " \"mean\": " << ts.read_vel.mean_us
        << ", \"min\": " << ts.read_vel.min_us << ", \"max\": " << ts.read_vel.max_us << " },\n"
        << "    \"read_sensor_us\": {" << " \"mean\": " << ts.read_sensor.mean_us
        << ", \"min\": " << ts.read_sensor.min_us << ", \"max\": " << ts.read_sensor.max_us
        << ", \"sensor_cycles\": " << ts.sensor_cycle_count << " },\n";
  }

  if (ts.sensor_cycle_count > 0) {
    ofs << "    \"sensor_proc_us\": {" << " \"mean\": " << ts.sensor_proc.mean_us
        << ", \"min\": " << ts.sensor_proc.min_us << ", \"max\": " << ts.sensor_proc.max_us
        << " },\n";
  }

  if (ts.ft_infer_count > 0) {
    ofs << "    \"ft_infer_us\": {" << " \"mean\": " << ts.ft_infer.mean_us
        << ", \"min\": " << ts.ft_infer.min_us << ", \"max\": " << ts.ft_infer.max_us
        << ", \"count\": " << ts.ft_infer_count << " },\n";
  }

  ofs << "    \"actual_sensor_rate_hz\": " << std::setprecision(1)
      << controller_->actual_sensor_rate_hz() << ",\n"
      << "    \"over_budget\": " << ts.over_budget << "\n"
      << "  }\n"
      << "}\n";
  ofs.close();

  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "%s", controller_->TimingSummary().c_str());

  const double total = static_cast<double>(stats.total_cycles > 0 ? stats.total_cycles : 1);
  const double ok_pct = 100.0 * static_cast<double>(stats.recv_ok) / total;
  const double timeout_pct = 100.0 * static_cast<double>(stats.recv_timeout) / total;
  const double error_pct = 100.0 * static_cast<double>(stats.recv_error) / total;
  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(),
              "Hand stats saved: %s | %lu cycles in %.1fs "
              "(avg=%.1fHz ok=%.1f%% timeout=%.1f%% err=%.1f%% fd_failed=%d)",
              path.c_str(), static_cast<unsigned long>(stats.total_cycles), elapsed_sec,
              avg_rate_hz, ok_pct, timeout_pct, error_pct, fd_failed ? 1 : 0);
}
