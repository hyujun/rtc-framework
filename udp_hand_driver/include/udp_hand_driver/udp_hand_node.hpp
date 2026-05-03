#pragma once

#include "udp_hand_driver/udp_hand_controller.hpp"
#include "udp_hand_driver/udp_hand_failure_detector.hpp"
#include "udp_hand_driver/udp_hand_timing_logger.hpp"
#include <rtc_msgs/msg/calibration_command.hpp>
#include <rtc_msgs/msg/calibration_status.hpp>
#include <rtc_msgs/msg/hand_sensor_state.hpp>
#include <rtc_msgs/msg/joint_command.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Unified hand UDP node using request-response protocol.
//
// Owns a UdpHandController that polls the hand device:
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
class UdpHandNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  UdpHandNode();
  ~UdpHandNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

 private:
  // Drain the EventLoop's timing producer into the CSV (1 Hz, non-RT).
  // Runs on the LifecycleNode's default executor thread.
  void DrainHandUdpTiming() noexcept;

  // Pre-allocate ROS2 messages once in on_configure (non-RT).
  // Avoids dynamic allocation on the EventLoop publish path.
  void PreallocateMessages();

  // Called directly from EventLoop thread — publishes state at EventLoop rate.
  // Uses pre-allocated messages to avoid dynamic allocation.
  void PublishFromEventLoop(const udp_hand_driver::UdpHandState& state,
                            const udp_hand_driver::FingertipFTState& ft_state);

  void PublishCalibrationStatus();

  void SaveCommStats() const;

  std::unique_ptr<udp_hand_driver::UdpHandController> controller_;
  std::unique_ptr<udp_hand_driver::UdpHandFailureDetector> failure_detector_;

  // Data publishers — LifecyclePublisher (gated by lifecycle state).
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr motor_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::HandSensorState>::SharedPtr sensor_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::HandSensorState>::SharedPtr
      sensor_monitor_pub_;
  rclcpp::Subscription<rtc_msgs::msg::JointCommand>::SharedPtr joint_command_sub_;
  rclcpp::Subscription<rtc_msgs::msg::CalibrationCommand>::SharedPtr calib_cmd_sub_;
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::CalibrationStatus>::SharedPtr
      calib_status_pub_;
  rclcpp::TimerBase::SharedPtr calib_status_timer_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> motor_names_;
  std::vector<std::string> fingertip_names_;
  int num_fingertips_{udp_hand_driver::kDefaultNumFingertips};

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
  std::array<float, udp_hand_driver::kNumHandMotors> last_cmd_{};

  std::chrono::steady_clock::time_point start_time_{std::chrono::steady_clock::now()};

  // Cached config for on_activate logging
  std::string target_ip_;
  int target_port_{0};
  std::string comm_mode_str_;

  // ── Per-EventLoop-tick timing CSV (mpc_timing_log pattern) ─────────────
  // Producer (filled on the EventLoop thread) → 1 Hz drain on aux timer →
  // rtc::ThreadTimingCsvLogger writes one row per tick to
  // <session>/timing/hand_udp_timing_log.csv. Open() runs once on the first
  // on_activate and is gated by `hand_udp_timing_initialized_` so reactivation
  // does not truncate or re-write the header.
  rtc::HandUdpTimingBuffer hand_udp_timing_producer_;
  udp_hand_driver::UdpHandTimingLogger hand_udp_timing_logger_;
  rclcpp::TimerBase::SharedPtr hand_udp_timing_timer_;
  bool hand_udp_timing_initialized_{false};
  std::uint64_t hand_udp_timing_drop_baseline_{0};
};
