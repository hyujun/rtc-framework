#include "ur5e_hand_driver/hand_controller.hpp"
#include "ur5e_hand_driver/hand_failure_detector.hpp"
#include "ur5e_hand_driver/hand_logging.hpp"

#include <rtc_base/logging/session_dir.hpp>
#include <rtc_base/threading/thread_utils.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rtc_msgs/msg/calibration_command.hpp>
#include <rtc_msgs/msg/calibration_status.hpp>
#include <rtc_msgs/msg/fingertip_sensor.hpp>
#include <rtc_msgs/msg/hand_sensor_state.hpp>
#include <rtc_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pthread.h>  // pthread_setaffinity_np
#include <sys/mman.h> // mlockall

#include <array>
#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <system_error>

using namespace std::chrono_literals;
namespace urtc = rtc;

// Unified hand UDP node using request-response protocol.
//
// Owns a HandController that polls the hand device:
//   write position -> read position -> read velocity -> read sensors x 4
//
// Publishes full state directly from EventLoop callback (no timer).
// This eliminates the 100 Hz timer bottleneck -- state is published at the
// EventLoop rate (500 Hz when driven by rtc_controller_manager).
//
// Pre-allocated messages avoid dynamic allocation on the publish path.
// Receives commands on /hand/command.
//
// Lifecycle states:
//   Unconfigured -> on_configure -> Inactive -> on_activate -> Active
//   Active -> on_deactivate -> Inactive -> on_cleanup -> Unconfigured
//
// Tier 1 (on_configure): parameters, controller, publishers, subscribers,
//   timers, pre-allocated messages, EventLoop callback.
// Tier 2 (on_activate): controller Start, fake tick timer, failure detector.
class HandUdpNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  HandUdpNode() : LifecycleNode("hand_udp_node") {
    // Lifecycle design: constructor is intentionally empty.
    // All resource allocation happens in on_configure().
  }

  ~HandUdpNode() override {
    // Safety net — idempotent cleanup in case lifecycle callbacks were not
    // invoked (e.g. SIGTERM without graceful shutdown).
    if (failure_detector_)
      failure_detector_->Stop();
    if (controller_ && controller_->IsRunning())
      controller_->Stop();
    SaveCommStats();
  }

  /// Tier 1: allocate all persistent resources (parameters, controller,
  /// publishers, subscribers, timers, pre-allocated messages).
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State & /*state*/) override {
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
    declare_parameter("ft_inferencer.history_length", urtc::kFTHistoryLength);
    declare_parameter("ft_inferencer.model_paths", std::vector<std::string>{});
    declare_parameter("ft_inferencer.calibration_enabled", true);
    declare_parameter("ft_inferencer.calibration_samples", 500);
    for (const auto &name : {"thumb", "index", "middle", "ring"}) {
      declare_parameter("ft_inferencer." + std::string(name) + "_max",
                        std::vector<double>(16, 1.0));
    }

    const std::string target_ip = get_parameter("target_ip").as_string();
    const int target_port =
        static_cast<int>(get_parameter("target_port").as_int());
    const double recv_timeout_ms = get_parameter("recv_timeout_ms").as_double();
    const std::string comm_mode_str =
        get_parameter("communication_mode").as_string();
    const auto comm_mode = (comm_mode_str == "bulk")
                               ? urtc::HandCommunicationMode::kBulk
                               : urtc::HandCommunicationMode::kIndividual;

    const bool baro_lpf_enabled = get_parameter("baro_lpf_enabled").as_bool();
    const double baro_lpf_cutoff_hz =
        get_parameter("baro_lpf_cutoff_hz").as_double();
    const bool tof_lpf_enabled = get_parameter("tof_lpf_enabled").as_bool();
    const double tof_lpf_cutoff_hz =
        get_parameter("tof_lpf_cutoff_hz").as_double();

    urtc::FingertipFTInferencer::Config ft_config;
    ft_config.enabled = get_parameter("ft_inferencer.enabled").as_bool();
    ft_config.num_fingertips = static_cast<int>(
        get_parameter("ft_inferencer.num_fingertips").as_int());
    ft_config.history_length = static_cast<int>(
        get_parameter("ft_inferencer.history_length").as_int());
    ft_config.model_paths =
        get_parameter("ft_inferencer.model_paths").as_string_array();

    {
      const std::string models_dir =
          ament_index_cpp::get_package_share_directory("ur5e_hand_driver") +
          "/models/";
      for (auto &p : ft_config.model_paths) {
        if (!p.empty() && p[0] != '/') {
          p = models_dir + p;
        }
      }
    }

    ft_config.calibration_enabled =
        get_parameter("ft_inferencer.calibration_enabled").as_bool();
    ft_config.calibration_samples = static_cast<int>(
        get_parameter("ft_inferencer.calibration_samples").as_int());

    const std::array<const char *, 4> ft_param_names = {"thumb", "index",
                                                        "middle", "ring"};
    for (int f = 0; f < 4 && f < ft_config.num_fingertips; ++f) {
      auto max_vec =
          get_parameter(
              "ft_inferencer." +
              std::string(ft_param_names[static_cast<std::size_t>(f)]) + "_max")
              .as_double_array();
      for (int b = 0;
           b < urtc::kFTInputSize && b < static_cast<int>(max_vec.size());
           ++b) {
        ft_config.input_max[static_cast<std::size_t>(f)]
                           [static_cast<std::size_t>(b)] =
            static_cast<float>(max_vec[static_cast<std::size_t>(b)]);
      }
    }

    const bool drift_enabled =
        get_parameter("drift_detection_enabled").as_bool();
    const double drift_threshold = get_parameter("drift_threshold").as_double();
    const int drift_window_size =
        static_cast<int>(get_parameter("drift_window_size").as_int());

    // ── HandController ─────────────────────────────────────────────────
    const auto ft_names =
        get_parameter("hand_fingertip_names").as_string_array();
    num_fingertips_ = urtc::kDefaultNumFingertips;
    use_fake_hand_ = get_parameter("use_fake_hand").as_bool();
    controller_ = std::make_unique<urtc::HandController>(
        target_ip, target_port, urtc::kUdpRecvConfig, recv_timeout_ms,
        false /* enable_write_ack: deprecated */, 1, num_fingertips_,
        use_fake_hand_, ft_names, comm_mode, tof_lpf_enabled, tof_lpf_cutoff_hz,
        baro_lpf_enabled, baro_lpf_cutoff_hz, ft_config, drift_enabled,
        drift_threshold, drift_window_size);

    // ── Topic names ──────────────────────────────────────────────────
    declare_parameter("command_topic", std::string("/hand/joint_command"));
    declare_parameter("joint_state_topic", std::string("/hand/joint_states"));
    declare_parameter("motor_state_topic", std::string("/hand/motor_states"));
    declare_parameter("sensor_topic", std::string("/hand/sensor_states"));
    declare_parameter("link_status_topic", std::string("/hand/link_status"));
    declare_parameter("calibration_command_topic",
                      std::string("/hand/calibration/command"));
    declare_parameter("calibration_status_topic",
                      std::string("/hand/calibration/status"));
    declare_parameter("calibration_status_rate_hz", 5.0);
    const std::string cmd_topic = get_parameter("command_topic").as_string();
    const std::string joint_state_topic =
        get_parameter("joint_state_topic").as_string();
    const std::string motor_state_topic =
        get_parameter("motor_state_topic").as_string();
    const std::string sensor_topic = get_parameter("sensor_topic").as_string();
    const std::string link_status_topic =
        get_parameter("link_status_topic").as_string();
    const std::string calib_cmd_topic =
        get_parameter("calibration_command_topic").as_string();
    const std::string calib_status_topic =
        get_parameter("calibration_status_topic").as_string();
    const double calib_status_rate_hz =
        get_parameter("calibration_status_rate_hz").as_double();

    // ── Publishers (LifecyclePublisher — gated by lifecycle state) ─────
    rclcpp::QoS sensor_pub_qos{1};
    sensor_pub_qos.best_effort();

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        joint_state_topic, sensor_pub_qos);
    motor_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        motor_state_topic, sensor_pub_qos);
    sensor_state_pub_ = create_publisher<rtc_msgs::msg::HandSensorState>(
        sensor_topic, sensor_pub_qos);

    sensor_monitor_pub_ = create_publisher<rtc_msgs::msg::HandSensorState>(
        sensor_topic + "/monitor", rclcpp::QoS{10});

    // Link status: standalone rclcpp::Publisher (NOT LifecyclePublisher).
    // Must remain publishable in any lifecycle state — link status is a
    // safety-relevant signal that should report even when Inactive.
    rclcpp::QoS link_pub_qos{1};
    link_pub_qos.transient_local();
    link_status_pub_ = rclcpp::create_publisher<std_msgs::msg::Bool>(
        this->get_node_topics_interface(), link_status_topic, link_pub_qos);
    link_fail_threshold_ =
        static_cast<uint64_t>(get_parameter("link_fail_threshold").as_int());

    ft_enabled_ = ft_config.enabled;

    // ── Hand joint/motor/fingertip names ────────────────────────────────
    auto joint_names = get_parameter("joint_state_names").as_string_array();
    if (joint_names.empty()) {
      joint_names = urtc::kDefaultHandMotorNames;
    }
    joint_names_ = joint_names;
    auto motor_names = get_parameter("motor_state_names").as_string_array();
    if (motor_names.empty()) {
      motor_names = urtc::kDefaultHandMotorNames;
    }
    motor_names_ = motor_names;
    auto fingertip_names =
        get_parameter("hand_fingertip_names").as_string_array();
    if (fingertip_names.empty()) {
      fingertip_names = urtc::kDefaultFingertipNames;
    }
    fingertip_names_ = fingertip_names;

    // ── Link status decimation ─────────────────────────────────────────
    const double publish_rate = get_parameter("publish_rate").as_double();
    link_decimation_ = std::max(1, static_cast<int>(500.0 / publish_rate));

    // ── Pre-allocate ROS2 messages ─────────────────────────────────────
    PreallocateMessages();

    // ── EventLoop callback ─────────────────────────────────────────────
    controller_->SetCallback([this](const urtc::HandState &state,
                                    const urtc::FingertipFTState &ft_state) {
      PublishFromEventLoop(state, ft_state);
    });

    // ── Subscriptions ──────────────────────────────────────────────────
    rclcpp::QoS cmd_sub_qos{1};
    cmd_sub_qos.best_effort();

    joint_command_sub_ = create_subscription<rtc_msgs::msg::JointCommand>(
        cmd_topic, cmd_sub_qos,
        [this](rtc_msgs::msg::JointCommand::SharedPtr msg) {
          if (!controller_->IsRunning()) {
            return;
          }
          if (msg->values.size() <
              static_cast<std::size_t>(urtc::kNumHandMotors)) {
            RCLCPP_WARN(::ur5e_hand_driver::logging::NodeLogger(),
                        "JointCommand values size %zu (expected %d)",
                        msg->values.size(), urtc::kNumHandMotors);
            return;
          }
          std::array<float, urtc::kNumHandMotors> cmd;
          for (std::size_t i = 0;
               i < static_cast<std::size_t>(urtc::kNumHandMotors); ++i) {
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
            RCLCPP_WARN(::ur5e_hand_driver::logging::NodeLogger(),
                        "Calibration command ignored: controller not running");
            return;
          }
          RCLCPP_INFO(
              ::ur5e_hand_driver::logging::NodeLogger(),
              "Calibration command: sensor_type=%u action=%u sample_count=%u",
              static_cast<unsigned>(msg->sensor_type),
              static_cast<unsigned>(msg->action),
              static_cast<unsigned>(msg->sample_count));
          controller_->RequestCalibration(msg->sensor_type, msg->action,
                                          msg->sample_count);
        });

    rclcpp::QoS calib_status_qos{1};
    calib_status_qos.reliable().transient_local();
    calib_status_pub_ = create_publisher<rtc_msgs::msg::CalibrationStatus>(
        calib_status_topic, calib_status_qos);

    const int status_period_ms =
        (calib_status_rate_hz > 0.0)
            ? static_cast<int>(1000.0 / calib_status_rate_hz)
            : 200;
    calib_status_timer_ =
        create_wall_timer(std::chrono::milliseconds(status_period_ms),
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
      RCLCPP_INFO(::ur5e_hand_driver::logging::NodeLogger(),
                  "Hand motor order (%zu): [%s]", motor_names.size(),
                  names_str.c_str());
    }

    RCLCPP_INFO(::ur5e_hand_driver::logging::NodeLogger(),
                "HandUdpNode configured: target %s:%d, direct publish from "
                "EventLoop, comm=%s",
                target_ip.c_str(), target_port, comm_mode_str.c_str());
    return CallbackReturn::SUCCESS;
  }

  /// Tier 2: start hardware communication and monitoring threads.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override {
    // Parent call activates LifecyclePublishers — must be called first.
    LifecycleNode::on_activate(state);

    start_time_ = std::chrono::steady_clock::now();

    if (!controller_->Start()) {
      RCLCPP_ERROR(::ur5e_hand_driver::logging::NodeLogger(),
                   "Failed to start HandController to %s:%d",
                   target_ip_.c_str(), target_port_);
      return CallbackReturn::FAILURE;
    }

    // Fake-hand self tick
    if (use_fake_hand_) {
      const double fake_rate = get_parameter("fake_tick_rate_hz").as_double();
      if (fake_rate > 0.0) {
        const auto period =
            std::chrono::nanoseconds(static_cast<int64_t>(1.0e9 / fake_rate));
        fake_tick_timer_ = create_wall_timer(period, [this]() {
          if (!controller_ || !controller_->IsRunning())
            return;
          std::array<float, urtc::kNumHandMotors> cmd;
          {
            std::lock_guard lock(last_cmd_mutex_);
            cmd = last_cmd_;
          }
          controller_->SendCommandAndRequestStates(cmd);
        });
        RCLCPP_WARN(
            ::ur5e_hand_driver::logging::NodeLogger(),
            "HandUdpNode: FAKE mode enabled — no UDP, self-tick=%.1f Hz",
            fake_rate);
      } else {
        RCLCPP_WARN(
            ::ur5e_hand_driver::logging::NodeLogger(),
            "HandUdpNode: FAKE mode enabled — self-tick disabled "
            "(fake_tick_rate_hz=%.1f). Drive SendCommandAndRequestStates "
            "externally (e.g. rtc_controller_manager ControlLoop).",
            fake_rate);
      }
    }

    // Failure Detector (skipped in fake mode)
    const bool enable_fd =
        get_parameter("enable_failure_detector").as_bool() && !use_fake_hand_;
    if (enable_fd) {
      urtc::HandFailureDetector::Config fd_cfg;
      fd_cfg.failure_threshold =
          static_cast<int>(get_parameter("failure_threshold").as_int());
      fd_cfg.check_motor = get_parameter("check_motor").as_bool();
      fd_cfg.check_sensor = get_parameter("check_sensor").as_bool();
      fd_cfg.min_rate_hz = get_parameter("min_rate_hz").as_double();
      fd_cfg.rate_fail_threshold =
          static_cast<int>(get_parameter("rate_fail_threshold").as_int());
      fd_cfg.check_link = get_parameter("check_link").as_bool();
      fd_cfg.link_fail_threshold =
          static_cast<int>(get_parameter("link_fail_threshold").as_int());

      const auto cfgs = urtc::SelectThreadConfigs();
      failure_detector_ = std::make_unique<urtc::HandFailureDetector>(
          *controller_, fd_cfg, cfgs.logging);
      failure_detector_->SetFailureCallback([this](const std::string &reason) {
        RCLCPP_ERROR(::ur5e_hand_driver::logging::NodeLogger(),
                     "Hand failure detected: %s", reason.c_str());
      });
      failure_detector_->Start();
      RCLCPP_INFO(::ur5e_hand_driver::logging::NodeLogger(),
                  "HandFailureDetector started (50 Hz, threshold=%d)",
                  fd_cfg.failure_threshold);
    }

    RCLCPP_INFO(::ur5e_hand_driver::logging::NodeLogger(),
                "HandUdpNode activated");
    return CallbackReturn::SUCCESS;
  }

  /// Tier 2 teardown: stop hardware communication and monitoring.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override {
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

    // Parent call deactivates LifecyclePublishers.
    LifecycleNode::on_deactivate(state);

    RCLCPP_INFO(::ur5e_hand_driver::logging::NodeLogger(),
                "HandUdpNode deactivated");
    return CallbackReturn::SUCCESS;
  }

  /// Tier 1 teardown: release all resources in reverse order of on_configure.
  CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & /*state*/) override {
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

    RCLCPP_INFO(::ur5e_hand_driver::logging::NodeLogger(),
                "HandUdpNode cleaned up");
    return CallbackReturn::SUCCESS;
  }

  /// Can be called from any primary state.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override {
    if (get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      on_deactivate(state);
    }
    return on_cleanup(state);
  }

  /// Error recovery — clean up and return to Unconfigured.
  CallbackReturn on_error(const rclcpp_lifecycle::State & /*state*/) override {
    RCLCPP_ERROR(::ur5e_hand_driver::logging::NodeLogger(),
                 "HandUdpNode error — attempting recovery");
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

private:
  // Pre-allocate ROS2 messages once in on_configure (non-RT).
  // This avoids dynamic allocation on the EventLoop publish path.
  void PreallocateMessages() {
    joint_js_msg_.name = joint_names_;
    joint_js_msg_.position.resize(urtc::kNumHandMotors);
    joint_js_msg_.velocity.resize(urtc::kNumHandMotors);
    joint_js_msg_.effort.resize(urtc::kNumHandMotors);

    motor_js_msg_.name = motor_names_;
    motor_js_msg_.position.resize(urtc::kNumHandMotors);
    motor_js_msg_.velocity.resize(urtc::kNumHandMotors);
    motor_js_msg_.effort.resize(urtc::kNumHandMotors);

    sensor_msg_.fingertips.resize(static_cast<std::size_t>(num_fingertips_));
    for (int f = 0; f < num_fingertips_; ++f) {
      auto &fs = sensor_msg_.fingertips[static_cast<std::size_t>(f)];
      fs.name = (static_cast<std::size_t>(f) < fingertip_names_.size())
                    ? fingertip_names_[static_cast<std::size_t>(f)]
                    : "f" + std::to_string(f);
    }
  }

  // Called directly from EventLoop thread — publishes state at EventLoop rate.
  // Uses pre-allocated messages to avoid dynamic allocation.
  void PublishFromEventLoop(const urtc::HandState &state,
                            const urtc::FingertipFTState &ft_state) {
    const auto stamp = this->now();

    if (state.joint_valid) {
      joint_js_msg_.header.stamp = stamp;
      for (int i = 0; i < urtc::kNumHandMotors; ++i) {
        const auto iu = static_cast<std::size_t>(i);
        joint_js_msg_.position[iu] =
            static_cast<double>(state.joint_positions[iu]);
        joint_js_msg_.velocity[iu] =
            static_cast<double>(state.joint_velocities[iu]);
        joint_js_msg_.effort[iu] =
            static_cast<double>(state.joint_currents[iu]);
      }
      joint_state_pub_->publish(joint_js_msg_);
    }

    if (state.motor_valid) {
      motor_js_msg_.header.stamp = stamp;
      for (int i = 0; i < urtc::kNumHandMotors; ++i) {
        const auto iu = static_cast<std::size_t>(i);
        motor_js_msg_.position[iu] =
            static_cast<double>(state.motor_positions[iu]);
        motor_js_msg_.velocity[iu] =
            static_cast<double>(state.motor_velocities[iu]);
        motor_js_msg_.effort[iu] =
            static_cast<double>(state.motor_currents[iu]);
      }
      motor_state_pub_->publish(motor_js_msg_);
    }

    if (state.num_fingertips > 0) {
      sensor_msg_.header.stamp = this->now();

      const bool ft_valid = ft_enabled_ && ft_state.valid;

      for (int f = 0; f < state.num_fingertips &&
                      f < static_cast<int>(sensor_msg_.fingertips.size());
           ++f) {
        auto &fs = sensor_msg_.fingertips[static_cast<std::size_t>(f)];

        const int sensor_base = f * urtc::kSensorValuesPerFingertip;
        for (int b = 0; b < urtc::kBarometerCount; ++b) {
          const auto bu = static_cast<std::size_t>(b);
          const auto si = static_cast<std::size_t>(sensor_base + b);
          fs.barometer[bu] = static_cast<float>(state.sensor_data[si]);
          fs.barometer_raw[bu] = static_cast<float>(state.sensor_data_raw[si]);
        }
        for (int t = 0; t < urtc::kTofCount; ++t) {
          const auto tu = static_cast<std::size_t>(t);
          const auto si =
              static_cast<std::size_t>(sensor_base + urtc::kBarometerCount + t);
          fs.tof[tu] = static_cast<float>(state.sensor_data[si]);
          fs.tof_raw[tu] = static_cast<float>(state.sensor_data_raw[si]);
        }

        if (ft_valid && f < ft_state.num_fingertips &&
            ft_state.per_fingertip_valid[static_cast<std::size_t>(f)]) {
          const int ft_base = f * urtc::kFTValuesPerFingertip;
          fs.inference_enable = true;
          fs.contact_flag = ft_state.ft_data[static_cast<std::size_t>(ft_base)];
          for (int j = 0; j < 3; ++j) {
            const auto ju = static_cast<std::size_t>(j);
            fs.f[ju] =
                ft_state.ft_data[static_cast<std::size_t>(ft_base + 1 + j)];
            fs.u[ju] =
                ft_state.ft_data[static_cast<std::size_t>(ft_base + 4 + j)];
          }
        } else {
          fs.inference_enable = false;
          fs.contact_flag = 0.0f;
          fs.f = {};
          fs.u = {};
        }
      }
      sensor_state_pub_->publish(sensor_msg_);
      sensor_monitor_pub_->publish(sensor_msg_);
    }

    // Link status (decimated — not every cycle).
    // Published via standalone rclcpp::Publisher (not LifecyclePublisher),
    // so it works regardless of lifecycle state.
    ++link_cycle_counter_;
    if (link_cycle_counter_ >= link_decimation_) {
      link_cycle_counter_ = 0;

      const uint64_t failures = controller_->consecutive_recv_failures();
      const bool link_ok = (failures < link_fail_threshold_);
      if (link_ok != prev_link_ok_) {
        if (link_ok) {
          RCLCPP_INFO(::ur5e_hand_driver::logging::NodeLogger(),
                      "Hand UDP link UP");
        } else {
          RCLCPP_WARN(::ur5e_hand_driver::logging::NodeLogger(),
                      "Hand UDP link DOWN (failures=%lu)",
                      static_cast<unsigned long>(failures));
        }
        prev_link_ok_ = link_ok;
      }
      link_msg_.data = link_ok;
      link_status_pub_->publish(link_msg_);
    }

    if (++publish_count_ % 500 == 0) {
      RCLCPP_DEBUG(::ur5e_hand_driver::logging::NodeLogger(), "cycles: %zu",
                   controller_->cycle_count());
    }
  }

  void PublishCalibrationStatus() {
    if (!controller_ || !calib_status_pub_)
      return;

    static constexpr std::array<uint8_t, 1> kTrackedSensors = {
        urtc::hand_calibration::kSensorBarometer,
    };

    for (const auto sensor_type : kTrackedSensors) {
      const auto snap = controller_->GetCalibrationStatus(sensor_type);
      rtc_msgs::msg::CalibrationStatus msg;
      msg.header.stamp = this->now();
      msg.sensor_type = snap.sensor_type;
      msg.state = snap.state;
      msg.progress_count = snap.progress_count;
      msg.target_count = snap.target_count;
      calib_status_pub_->publish(msg);
    }
  }

  void SaveCommStats() const {
    if (!controller_)
      return;

    const auto stats = controller_->comm_stats();
    const bool fd_failed =
        failure_detector_ ? failure_detector_->failed() : false;

    const auto elapsed = std::chrono::steady_clock::now() - start_time_;
    const double elapsed_sec = std::chrono::duration<double>(elapsed).count();
    const double avg_rate_hz =
        (elapsed_sec > 0.0)
            ? static_cast<double>(stats.total_cycles) / elapsed_sec
            : 0.0;

    const std::filesystem::path session = urtc::ResolveSessionDir();
    std::filesystem::path output_dir_path = session / "device";
    std::error_code dir_ec;
    std::filesystem::create_directories(output_dir_path, dir_ec);
    const std::string output_dir = output_dir_path.string();

    const std::string path = output_dir + "/hand_udp_stats.json";
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
      RCLCPP_WARN(::ur5e_hand_driver::logging::NodeLogger(),
                  "Failed to save hand stats to %s", path.c_str());
      return;
    }

    const auto ts = controller_->timing_stats();

    const bool is_bulk = (controller_->communication_mode() ==
                          urtc::HandCommunicationMode::kBulk);
    const char *mode_str = is_bulk ? "bulk" : "individual";

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
        << "    \"avg_rate_hz\": " << std::fixed << std::setprecision(2)
        << avg_rate_hz << ",\n"
        << "    \"elapsed_sec\": " << std::fixed << std::setprecision(2)
        << elapsed_sec << ",\n"
        << "    \"failure_detected\": " << (fd_failed ? "true" : "false")
        << ",\n"
        << "    \"consecutive_recv_failures\": "
        << controller_->consecutive_recv_failures() << ",\n"
        << "    \"link_ok\": "
        << (controller_->consecutive_recv_failures() < link_fail_threshold_
                ? "true"
                : "false")
        << "\n"
        << "  },\n"
        << "  \"timing_stats\": {\n"
        << "    \"count\": " << ts.count << ",\n"
        << "    \"total_us\": {" << " \"mean\": " << std::setprecision(1)
        << ts.mean_us << ", \"min\": " << ts.min_us
        << ", \"max\": " << ts.max_us << ", \"stddev\": " << ts.stddev_us
        << ", \"p95\": " << ts.p95_us << ", \"p99\": " << ts.p99_us << " },\n"
        << "    \"write_us\": {" << " \"mean\": " << ts.write.mean_us
        << ", \"min\": " << ts.write.min_us << ", \"max\": " << ts.write.max_us
        << " },\n";

    if (is_bulk) {
      ofs << "    \"read_all_motor_us\": {"
          << " \"mean\": " << ts.read_all_motor.mean_us
          << ", \"min\": " << ts.read_all_motor.min_us
          << ", \"max\": " << ts.read_all_motor.max_us << " },\n"
          << "    \"read_all_joint_motor_us\": {"
          << " \"mean\": " << ts.read_all_joint_motor.mean_us
          << ", \"min\": " << ts.read_all_joint_motor.min_us
          << ", \"max\": " << ts.read_all_joint_motor.max_us << " },\n"
          << "    \"read_all_sensor_us\": {"
          << " \"mean\": " << ts.read_all_sensor.mean_us
          << ", \"min\": " << ts.read_all_sensor.min_us
          << ", \"max\": " << ts.read_all_sensor.max_us
          << ", \"sensor_cycles\": " << ts.sensor_cycle_count << " },\n";
    } else {
      ofs << "    \"read_pos_us\": {" << " \"mean\": " << ts.read_pos.mean_us
          << ", \"min\": " << ts.read_pos.min_us
          << ", \"max\": " << ts.read_pos.max_us << " },\n"
          << "    \"read_joint_pos_us\": {"
          << " \"mean\": " << ts.read_joint_pos.mean_us
          << ", \"min\": " << ts.read_joint_pos.min_us
          << ", \"max\": " << ts.read_joint_pos.max_us << " },\n"
          << "    \"read_vel_us\": {" << " \"mean\": " << ts.read_vel.mean_us
          << ", \"min\": " << ts.read_vel.min_us
          << ", \"max\": " << ts.read_vel.max_us << " },\n"
          << "    \"read_sensor_us\": {"
          << " \"mean\": " << ts.read_sensor.mean_us
          << ", \"min\": " << ts.read_sensor.min_us
          << ", \"max\": " << ts.read_sensor.max_us
          << ", \"sensor_cycles\": " << ts.sensor_cycle_count << " },\n";
    }

    if (ts.sensor_cycle_count > 0) {
      ofs << "    \"sensor_proc_us\": {"
          << " \"mean\": " << ts.sensor_proc.mean_us
          << ", \"min\": " << ts.sensor_proc.min_us
          << ", \"max\": " << ts.sensor_proc.max_us << " },\n";
    }

    if (ts.ft_infer_count > 0) {
      ofs << "    \"ft_infer_us\": {" << " \"mean\": " << ts.ft_infer.mean_us
          << ", \"min\": " << ts.ft_infer.min_us
          << ", \"max\": " << ts.ft_infer.max_us
          << ", \"count\": " << ts.ft_infer_count << " },\n";
    }

    ofs << "    \"actual_sensor_rate_hz\": " << std::setprecision(1)
        << controller_->actual_sensor_rate_hz() << ",\n"
        << "    \"over_budget\": " << ts.over_budget << "\n"
        << "  }\n"
        << "}\n";
    ofs.close();

    RCLCPP_INFO(::ur5e_hand_driver::logging::NodeLogger(), "%s",
                controller_->TimingSummary().c_str());

    const double total =
        static_cast<double>(stats.total_cycles > 0 ? stats.total_cycles : 1);
    const double ok_pct = 100.0 * static_cast<double>(stats.recv_ok) / total;
    const double timeout_pct =
        100.0 * static_cast<double>(stats.recv_timeout) / total;
    const double error_pct =
        100.0 * static_cast<double>(stats.recv_error) / total;
    RCLCPP_INFO(::ur5e_hand_driver::logging::NodeLogger(),
                "Hand stats saved: %s | %lu cycles in %.1fs "
                "(avg=%.1fHz ok=%.1f%% timeout=%.1f%% err=%.1f%% fd_failed=%d)",
                path.c_str(), static_cast<unsigned long>(stats.total_cycles),
                elapsed_sec, avg_rate_hz, ok_pct, timeout_pct, error_pct,
                fd_failed ? 1 : 0);
  }

  std::unique_ptr<urtc::HandController> controller_;
  std::unique_ptr<urtc::HandFailureDetector> failure_detector_;

  // Data publishers — LifecyclePublisher (gated by lifecycle state).
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
      motor_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      rtc_msgs::msg::HandSensorState>::SharedPtr sensor_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      rtc_msgs::msg::HandSensorState>::SharedPtr sensor_monitor_pub_;
  rclcpp::Subscription<rtc_msgs::msg::JointCommand>::SharedPtr
      joint_command_sub_;
  rclcpp::Subscription<rtc_msgs::msg::CalibrationCommand>::SharedPtr
      calib_cmd_sub_;
  rclcpp_lifecycle::LifecyclePublisher<
      rtc_msgs::msg::CalibrationStatus>::SharedPtr calib_status_pub_;
  rclcpp::TimerBase::SharedPtr calib_status_timer_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> motor_names_;
  std::vector<std::string> fingertip_names_;
  int num_fingertips_{urtc::kDefaultNumFingertips};

  // Link status — standalone rclcpp::Publisher (NOT LifecyclePublisher).
  // Safety-relevant: must remain publishable in any lifecycle state.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr link_status_pub_;
  bool ft_enabled_{false};
  uint64_t link_fail_threshold_{10};
  bool prev_link_ok_{true};

  // Pre-allocated messages
  sensor_msgs::msg::JointState joint_js_msg_;
  sensor_msgs::msg::JointState motor_js_msg_;
  rtc_msgs::msg::HandSensorState sensor_msg_;
  std_msgs::msg::Bool link_msg_;

  // Link status decimation
  int link_decimation_{5};
  int link_cycle_counter_{0};

  std::size_t publish_count_{0};

  // Fake-hand standalone support
  bool use_fake_hand_{false};
  rclcpp::TimerBase::SharedPtr fake_tick_timer_;
  std::mutex last_cmd_mutex_;
  std::array<float, urtc::kNumHandMotors> last_cmd_{};

  std::chrono::steady_clock::time_point start_time_{
      std::chrono::steady_clock::now()};

  // Cached config for on_activate logging
  std::string target_ip_;
  int target_port_{0};
  std::string comm_mode_str_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  const auto logger = ::ur5e_hand_driver::logging::NodeLogger();

  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    RCLCPP_WARN(logger, "mlockall failed (errno=%d: %s)", errno,
                strerror(errno));
  }

  // Pin main thread (ROS2 executor, DDS) to Core 0-1 (OS/DDS cores).
  {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    CPU_SET(1, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
      RCLCPP_WARN(logger, "Main thread CPU affinity failed (errno=%d)", errno);
    }
  }

  auto node = std::make_shared<HandUdpNode>();
  // Constructor is empty — launch event handler triggers configure/activate.
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
