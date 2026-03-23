#pragma once

// ── Project headers ──────────────────────────────────────────────────────────
#include "rtc_status_monitor/rtc_status_monitor.hpp"
#include "rtc_status_monitor/failure_types.hpp"
#include "rtc_base/threading/thread_config.hpp"

// ── ROS2 ─────────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rtc_msgs/msg/hand_sensor_state.hpp>

// ── C++ stdlib ───────────────────────────────────────────────────────────────
#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rtc {

/// UR5e + Hand status monitor.
///
/// Composes RtcStatusMonitor (robot monitoring) and adds hand device monitoring:
///   - Hand topic timeout (watchdog)
///   - Motor data quality (all-zero, duplicate detection)
///   - Sensor data quality (all-zero, duplicate detection)
///   - Hand message rate monitoring
///
/// Architecture:
///   RT Control Thread (500 Hz)
///        ↕ atomic reads: isReady() / getFailure() / isJointLimitWarning()
///   Ur5eHandStatusMonitor
///        ├─ RtcStatusMonitor (10 Hz jthread) — robot mode, safety, tracking, limits
///        └─ HandMonitorLoop  (10 Hz jthread) — hand data quality, rate, timeout
///        ↕ ROS2 subscriptions
///   ur_robot_driver / hand_udp_node
///
class Ur5eHandStatusMonitor {
public:
  explicit Ur5eHandStatusMonitor(rclcpp::Node::SharedPtr node);
  ~Ur5eHandStatusMonitor();

  // Non-copyable, non-movable
  Ur5eHandStatusMonitor(const Ur5eHandStatusMonitor&) = delete;
  Ur5eHandStatusMonitor& operator=(const Ur5eHandStatusMonitor&) = delete;
  Ur5eHandStatusMonitor(Ur5eHandStatusMonitor&&) = delete;
  Ur5eHandStatusMonitor& operator=(Ur5eHandStatusMonitor&&) = delete;

  // ── RT-safe accessors (delegated to RtcStatusMonitor) ─────────────────────
  [[nodiscard]] bool isReady() const noexcept;
  [[nodiscard]] FailureType getFailure() const noexcept;
  [[nodiscard]] bool isJointLimitWarning() const noexcept;
  [[nodiscard]] RtcStatusMonitor::MessageStats getJointStateStats() const noexcept;
  [[nodiscard]] RtcStatusMonitor::MessageStats getHandStats() const noexcept;

  // ── Blocking wait ─────────────────────────────────────────────────────────
  bool waitForReady(double timeout_sec);

  // ── RT thread interface ───────────────────────────────────────────────────
  void setJointReference(const std::array<double, kNumRobotJoints>& q_ref,
                         const std::array<double, kNumRobotJoints>& qd_ref) noexcept;

  // ── Callback registration ─────────────────────────────────────────────────
  void registerOnReady(std::function<void()> cb);
  void registerOnFailure(
      std::function<void(FailureType, const FailureContext&)> cb);
  void registerOnWarning(
      std::function<void(WarningType, const std::string&)> cb);
  void registerOnRecovery(std::function<void()> cb);

  // ── Lifecycle ─────────────────────────────────────────────────────────────
  void start(const ThreadConfig& thread_cfg = kStatusMonitorConfig);
  void stop();
  [[nodiscard]] rclcpp::CallbackGroup::SharedPtr GetCallbackGroup() const;

private:
  // ── Parameter loading ─────────────────────────────────────────────────────
  void DeclareAndLoadHandParameters();

  // ── Hand subscription callbacks ───────────────────────────────────────────
  void OnHandJointState(sensor_msgs::msg::JointState::SharedPtr msg);
  void OnHandSensorState(rtc_msgs::msg::HandSensorState::SharedPtr msg);

  // ── Hand monitor thread ───────────────────────────────────────────────────
  void HandMonitorLoop(std::stop_token stop_token);

  // ── Hand check functions (called from HandMonitorLoop at 10 Hz) ───────────
  void CheckHandWatchdog();
  void CheckHandMotorData();
  void CheckHandSensorData();
  void CheckHandRate();

  // ── Hand failure handling ─────────────────────────────────────────────────
  void RaiseHandFailure(FailureType type, const std::string& description);

  // ── ROS2 node ─────────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;

  // ── Robot monitor (composed) ──────────────────────────────────────────────
  std::unique_ptr<RtcStatusMonitor> robot_monitor_;

  // ── Hand subscriptions ────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr hand_joint_state_sub_;
  rclcpp::Subscription<rtc_msgs::msg::HandSensorState>::SharedPtr hand_sensor_state_sub_;

  // ── Hand monitoring config ────────────────────────────────────────────────
  bool    hand_monitoring_enabled_{false};
  double  hand_watchdog_timeout_sec_{1.0};
  double  hand_min_rate_hz_{30.0};
  int     hand_rate_fail_threshold_{5};
  int     hand_data_fail_threshold_{5};
  bool    hand_check_motor_{true};
  bool    hand_check_sensor_{true};
  std::string hand_joint_states_topic_{"/hand/joint_states"};
  std::string hand_sensor_states_topic_{"/hand/sensor_states"};

  // ── Hand atomic state ─────────────────────────────────────────────────────
  std::atomic<bool>    hand_state_received_{false};
  std::atomic<int64_t> last_hand_state_time_ns_{0};
  std::atomic<FailureType> hand_failure_type_{FailureType::kNone};

  // ── Hand data quality (mutex-guarded) ─────────────────────────────────────
  mutable std::mutex hand_data_mutex_;
  std::vector<double> hand_motor_positions_;
  std::vector<double> prev_hand_motor_positions_;
  int hand_motor_zero_count_{0};
  int hand_motor_dup_count_{0};
  std::vector<float> hand_sensor_values_;
  std::vector<float> prev_hand_sensor_values_;
  int hand_sensor_zero_count_{0};
  int hand_sensor_dup_count_{0};

  // ── Hand rate monitoring ──────────────────────────────────────────────────
  RtcStatusMonitor::MessageStats hand_stats_;
  std::chrono::steady_clock::time_point hand_rate_window_start_{};
  uint64_t hand_rate_window_count_{0};
  int hand_rate_fail_count_{0};

  // ── Hand monitor thread ───────────────────────────────────────────────────
  std::jthread hand_monitor_thread_;
  ThreadConfig hand_monitor_cfg_{};

  // ── Callbacks ─────────────────────────────────────────────────────────────
  std::function<void(FailureType, const FailureContext&)> on_failure_cb_;
  std::function<void(WarningType, const std::string&)>    on_warning_cb_;
};

}  // namespace rtc
