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
// Owns a UdpHandController with a self-clocked CommLoop that polls the hand
// device at loop_rate_hz (default 500 Hz):
//   [write position (only when commanded)] -> read motors -> read sensors
//
// Publishes full state directly from the CommLoop callback (no ROS timer). The
// read/state-publish rate is autonomous at loop_rate_hz and does not depend on
// command arrival; the write UDP runs only when /…/joint_command was received.
//
// Pre-allocated messages avoid dynamic allocation on the publish path.
// Receives commands on the command_topic (default /hand/joint_command).
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

  // Persist comm/timing stats to <session>/device/hand_udp_stats.json.
  // Callable from the executor thread (lifecycle, periodic timer) and the
  // failure-detector callback thread — serialized by save_stats_mutex_.
  // verbose=false (periodic saves) skips the INFO summary logs.
  // Defined in udp_hand_node_stats.cpp.
  void SaveCommStats(bool verbose = true) const;

  // Cancel timers and stop the worker threads (failure detector + comm loop),
  // leaving controller_ constructed for possible reactivation. Idempotent;
  // shared by on_deactivate and on_error.
  void StopRuntime();

  // Release all publisher/subscription/timer handles and the controller.
  // Idempotent; shared by on_cleanup and on_error.
  void ReleaseResources();

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

  // Per-joint position offset (URDF/controller frame − firmware zero), radians.
  // Parsed once in on_configure from the `joint_position_offsets_deg` YAML list
  // (degrees, joint_state_names order). Applied at the two ROS boundaries only:
  //   read/publish : controller_pos = udp_pos + offset  (PublishFromEventLoop)
  //   command/write: udp_cmd        = controller_cmd − offset  (command sub)
  // so the round-trip is identity. Zero by default → backward compatible.
  // Position-only: velocity/effort and motor-space positions are untouched.
  std::array<float, udp_hand_driver::kNumHandMotors> joint_offset_rad_{};

  // Sensor-message layout selector, cached from the SensorProtocol capability
  // at on_configure (off the hot path). true → publish the 1b per-fingertip
  // force vector (fs.f) from state.sensor_force; false → publish the 1a
  // barometer/ToF + F/T-inference payload. The publish branch reads this bool,
  // never a version string (ARCH-3).
  bool sensor_uses_force_layout_{false};

  // Link status — standalone rclcpp::Publisher (NOT LifecyclePublisher).
  // Safety-relevant: must remain publishable in any lifecycle state.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr link_status_pub_;
  bool ft_enabled_{false};
  // Per-channel link-down thresholds (strict, #1). Computed once in on_configure
  // from link_fail_timeout_ms at each channel's effective attempt rate; the SSoT
  // the detector cfg + publish + stats all read (via UdpHandController::LinkDown)
  // so the E-STOP decision and the link_status Bool cannot disagree. comm =
  // motor/joint (every comm cycle); sensor = the shorter sensor-rate budget.
  uint64_t link_fail_threshold_{10};
  uint64_t link_fail_threshold_sensor_{10};
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

  // Periodic stats persistence (crash-safety): without it the stats JSON is
  // written only on graceful teardown and is lost on SIGKILL/crash. Interval
  // is a fixed constant — see on_activate (no ROS param on purpose).
  rclcpp::TimerBase::SharedPtr stats_save_timer_;
  mutable std::mutex save_stats_mutex_;

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
