#pragma once

// ── Workspace Analysis ───────────────────────────────────────────────────────
// Package:      rtc_status_monitor (v5.2.2)
// Build type:   Shared library — composed into RtControllerNode
// Executor:     Monitor callback group → aux_executor (non-RT)
// Thread model: Internal std::jthread at 10 Hz for periodic checks
// RT safety:    isReady(), getFailure(), isJointLimitWarning() are lock-free
// ─────────────────────────────────────────────────────────────────────────────

// ── Project headers ──────────────────────────────────────────────────────────
#include "rtc_status_monitor/failure_types.hpp"
#include "rtc_status_monitor/state_history.hpp"
#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/threading/thread_utils.hpp"

// ── ROS2 ─────────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

// ── C++ stdlib ───────────────────────────────────────────────────────────────
#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace rtc_status_monitor {

// ── Default joint limits for UR5e (rad) ──────────────────────────────────────
// Each pair is {lower, upper}. Overridable via ROS2 parameters.
inline constexpr std::array<std::array<double, 2>, kNumJoints> kDefaultJointLimits{{
  {{ -6.2832,  6.2832}},  // joint 0: ±360 deg
  {{ -6.2832,  6.2832}},  // joint 1: ±360 deg
  {{ -3.1416,  3.1416}},  // joint 2: ±180 deg (elbow)
  {{ -6.2832,  6.2832}},  // joint 3: ±360 deg
  {{ -6.2832,  6.2832}},  // joint 4: ±360 deg
  {{ -6.2832,  6.2832}},  // joint 5: ±360 deg
}};

// ── UR5eStatusMonitor ────────────────────────────────────────────────────────
//
// Non-RT status monitor for UR5e servoJ control.
// Composed into an existing ROS2 node via shared_ptr.
//
// Architecture:
//   RT Control Thread (500 Hz)
//        ↕ atomic reads: isReady() / getFailure() / isJointLimitWarning()
//   UR5eStatusMonitor (10 Hz std::jthread)
//        ↕ ROS2 subscriptions / service calls
//   ur_robot_driver / controller_manager
//
class UR5eStatusMonitor {
public:
  // ── Message statistics ────────────────────────────────────────────────────
  struct MessageStats {
    uint64_t total_count{0};
    uint64_t timeout_count{0};
    double current_rate_hz{0.0};
  };

  // ── Per-controller statistics ─────────────────────────────────────────────
  struct ControllerStats {
    std::string name;
    std::chrono::steady_clock::time_point first_active{};
    std::chrono::steady_clock::time_point last_active{};
    double total_active_sec{0.0};
    uint64_t js_packets{0};
    uint64_t js_timeouts{0};
    uint64_t failure_count{0};
  };

  /// Construct the monitor and attach to an existing ROS2 node.
  /// Does NOT start monitoring — call start() explicitly.
  /// @param node  Shared pointer to the host node (must outlive this object).
  explicit UR5eStatusMonitor(rclcpp::Node::SharedPtr node);

  /// Destructor — stops the monitor thread if running.
  ~UR5eStatusMonitor();

  // Non-copyable, non-movable
  UR5eStatusMonitor(const UR5eStatusMonitor&) = delete;
  UR5eStatusMonitor& operator=(const UR5eStatusMonitor&) = delete;
  UR5eStatusMonitor(UR5eStatusMonitor&&) = delete;
  UR5eStatusMonitor& operator=(UR5eStatusMonitor&&) = delete;

  // ── RT-safe accessors (lock-free, called from 500 Hz control loop) ─────────

  /// True when all readiness conditions are met and no failure is active.
  [[nodiscard]] bool isReady() const noexcept;

  /// Current failure type (kNone if no failure).
  [[nodiscard]] FailureType getFailure() const noexcept;

  /// True when any joint is within the warning margin of its limit.
  [[nodiscard]] bool isJointLimitWarning() const noexcept;

  /// Get joint state message statistics (packet count, rate, timeouts).
  [[nodiscard]] MessageStats getJointStateStats() const noexcept;

  // ── Blocking wait ──────────────────────────────────────────────────────────

  /// Block until monitor reports ready or timeout expires.
  /// Call before starting the RT control loop.
  /// @param timeout_sec  Maximum wait time in seconds.
  /// @return true if ready, false if timed out.
  bool waitForReady(double timeout_sec);

  // ── RT thread interface ────────────────────────────────────────────────────

  /// Feed reference positions from the RT thread for tracking error calculation.
  /// Uses try_lock — never blocks. If the monitor thread holds the mutex,
  /// this call silently skips (stale by at most one monitor cycle = 100 ms).
  void setJointReference(const std::array<double, kNumJoints>& q_ref,
                         const std::array<double, kNumJoints>& qd_ref) noexcept;

  // ── Callback registration (call before start()) ────────────────────────────

  /// Invoked once when all readiness conditions are first met.
  void registerOnReady(std::function<void()> cb);

  /// Invoked when a failure is detected (from the monitor thread).
  void registerOnFailure(
      std::function<void(FailureType, const FailureContext&)> cb);

  /// Invoked on warning-level conditions (from the monitor thread).
  void registerOnWarning(
      std::function<void(WarningType, const std::string&)> cb);

  /// Invoked when auto-recovery succeeds (from the monitor thread).
  void registerOnRecovery(std::function<void()> cb);

  // ── Lifecycle ──────────────────────────────────────────────────────────────

  /// Start the 10 Hz monitor thread.
  /// @param thread_cfg  Thread scheduling / CPU affinity configuration.
  void start(const rtc::ThreadConfig& thread_cfg =
                 rtc::kStatusMonitorConfig);

  /// Request the monitor thread to stop and join.
  void stop();

  /// Expose the callback group so main() can add it to an executor.
  [[nodiscard]] rclcpp::CallbackGroup::SharedPtr GetCallbackGroup() const;

private:
  // ── Parameter helpers ──────────────────────────────────────────────────────
  void DeclareAndLoadParameters();

  // ── Subscription callbacks ─────────────────────────────────────────────────
  void OnJointState(sensor_msgs::msg::JointState::SharedPtr msg);
  void OnRobotMode(std_msgs::msg::Int32::SharedPtr msg);
  void OnSafetyMode(std_msgs::msg::Int32::SharedPtr msg);
  void OnProgramRunning(std_msgs::msg::Bool::SharedPtr msg);
  void OnControllerNameChanged(const std_msgs::msg::String::SharedPtr msg);

  // ── Monitor thread ─────────────────────────────────────────────────────────
  void MonitorLoop(std::stop_token stop_token);

  // ── Check functions (called from MonitorLoop at 10 Hz) ─────────────────────
  void CheckReadiness();
  void CheckJointStateWatchdog();
  void CheckRobotStatus();
  void CheckTrackingErrors();
  void CheckJointLimits();
  void CheckControllerManager();

  // ── Diagnostics ────────────────────────────────────────────────────────────
  void PublishDiagnostics();

  // ── Controller stats ──────────────────────────────────────────────────────
  void SaveControllerStatsJson() const;

  // ── Failure handling ───────────────────────────────────────────────────────
  void RaiseFailure(FailureType type, const std::string& description);
  void ClearFailure();
  void LogFailureToFile(const FailureContext& ctx);

  // ── Recovery ───────────────────────────────────────────────────────────────
  void AttemptRecovery(FailureType type);

  // ── ROS2 node ──────────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_monitor_;

  // ── Subscriptions ──────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr         robot_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr         safety_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          program_running_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr       controller_name_sub_;

  // ── Publisher ──────────────────────────────────────────────────────────────
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // ── Service client ─────────────────────────────────────────────────────────
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
      list_controllers_client_;

  // ── Atomic state (RT-readable, lock-free) ──────────────────────────────────
  std::atomic<bool>        is_ready_{false};
  std::atomic<FailureType> failure_type_{FailureType::kNone};
  std::atomic<bool>        joint_limit_warning_{false};

  // ── Atomic state from subscription callbacks ───────────────────────────────
  std::atomic<int32_t> robot_mode_{static_cast<int32_t>(URRobotMode::kDisconnected)};
  std::atomic<int32_t> safety_mode_{static_cast<int32_t>(URSafetyMode::kUndefined)};
  std::atomic<bool>    program_running_{false};
  std::atomic<bool>    joint_state_received_{false};

  // last_joint_state_time_ stores steady_clock::time_point::rep (int64_t nanoseconds)
  // so it can be stored atomically without a mutex.
  std::atomic<int64_t> last_joint_state_time_{0};

  // ── Mutex-guarded state: reference from RT thread ──────────────────────────
  mutable std::mutex ref_mutex_;
  std::array<double, kNumJoints> q_ref_{};
  std::array<double, kNumJoints> qd_ref_{};
  bool ref_valid_{false};

  // ── Mutex-guarded state: actual joint state from subscription ──────────────
  mutable std::mutex joint_state_mutex_;
  std::array<double, kNumJoints> q_actual_{};
  std::array<double, kNumJoints> qd_actual_{};

  // ── Mutex-guarded state: history ring buffer ───────────────────────────────
  mutable std::mutex history_mutex_;
  StateHistory state_history_{};

  // ── Callbacks ──────────────────────────────────────────────────────────────
  mutable std::mutex callback_mutex_;
  std::function<void()>                                      on_ready_cb_;
  std::function<void(FailureType, const FailureContext&)>    on_failure_cb_;
  std::function<void(WarningType, const std::string&)>       on_warning_cb_;
  std::function<void()>                                      on_recovery_cb_;
  bool ready_callback_fired_{false};

  // ── Monitor thread ─────────────────────────────────────────────────────────
  std::jthread monitor_thread_;

  // ── Parameters ─────────────────────────────────────────────────────────────
  // Tracking error thresholds (rad)
  double tracking_error_pos_warn_rad_{0.05};
  double tracking_error_pos_fault_rad_{0.15};
  double tracking_error_vel_warn_rad_{0.1};
  double tracking_error_vel_fault_rad_{0.3};

  // Joint limit margins (rad, converted from deg parameters)
  double joint_limit_warn_margin_rad_{0.0873};   // ~5.0 deg
  double joint_limit_fault_margin_rad_{0.0175};   // ~1.0 deg

  // Joint limits
  std::array<std::array<double, 2>, kNumJoints> joint_limits_{kDefaultJointLimits};

  // Watchdog
  double watchdog_timeout_sec_{1.0};

  // Controller manager
  double controller_poll_interval_sec_{5.0};
  std::string target_controller_{"scaled_joint_trajectory_controller"};

  // Recovery
  bool   auto_recovery_{false};
  int    max_recovery_attempts_{3};
  double recovery_interval_sec_{5.0};
  int    recovery_attempts_{0};
  std::chrono::steady_clock::time_point last_recovery_time_{};

  // Logging
  std::string log_output_dir_;

  // Topic names (configurable)
  std::string joint_states_topic_{"/joint_states"};
  std::string robot_mode_topic_{"/io_and_status_controller/robot_mode"};
  std::string safety_mode_topic_{"/io_and_status_controller/safety_mode"};
  std::string program_running_topic_{"/io_and_status_controller/robot_program_running"};

  // Diagnostics timing
  std::chrono::steady_clock::time_point last_diag_time_{};
  std::chrono::steady_clock::time_point last_controller_poll_time_{};
  std::chrono::steady_clock::time_point start_time_{};

  // Controller state cache (from last list_controllers call)
  bool target_controller_active_{false};

  // ── Message stats (packet counting & rate monitoring) ─────────────────────
  MessageStats js_stats_;
  std::chrono::steady_clock::time_point rate_window_start_{};
  uint64_t rate_window_count_{0};

  // ── Per-controller statistics ─────────────────────────────────────────────
  bool enable_controller_stats_{true};
  std::unordered_map<std::string, ControllerStats> controller_stats_;
  std::string current_controller_name_{"unknown"};
};

}  // namespace rtc_status_monitor
