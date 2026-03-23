// ── Workspace Analysis ───────────────────────────────────────────────────────
// Package:      rtc_status_monitor (v5.2.2)
// Build type:   Shared library — composed into RtControllerNode
// Code style:   K&R braces, PascalCase methods, snake_case_ members
// Thread model: std::jthread for monitor loop, MutuallyExclusive callback group
// Dependencies: rclcpp, std_msgs, sensor_msgs, diagnostic_msgs,
//               controller_manager_msgs, rtc_base
// ─────────────────────────────────────────────────────────────────────────────

// ── Project headers ──────────────────────────────────────────────────────────
#include "rtc_status_monitor/rtc_status_monitor.hpp"

// ── ROS2 ─────────────────────────────────────────────────────────────────────
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

// ── C++ stdlib ───────────────────────────────────────────────────────────────
#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace rtc {

namespace {

// ── Helpers ──────────────────────────────────────────────────────────────────

constexpr double kDegToRad = M_PI / 180.0;

std::string SteadyToIso8601() {
  const auto sys_now  = std::chrono::system_clock::now();
  const auto time_t   = std::chrono::system_clock::to_time_t(sys_now);
  const auto ms       = std::chrono::duration_cast<std::chrono::milliseconds>(
                            sys_now.time_since_epoch()) % 1000;
  std::tm tm_buf{};
  localtime_r(&time_t, &tm_buf);
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y%m%dT%H%M%S")
      << '_' << std::setfill('0') << std::setw(3) << ms.count();
  return oss.str();
}

std::string FormatArray(const std::array<double, kNumJoints>& arr) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6) << '[';
  for (int i = 0; i < kNumJoints; ++i) {
    if (i > 0) oss << ", ";
    oss << arr[static_cast<std::size_t>(i)];
  }
  oss << ']';
  return oss.str();
}

}  // namespace

// ── Constructor ──────────────────────────────────────────────────────────────

RtcStatusMonitor::RtcStatusMonitor(rclcpp::Node::SharedPtr node)
  : node_(std::move(node)) {
  // Create a dedicated callback group for monitor subscriptions.
  // Added to aux_executor in main() via GetCallbackGroup().
  cb_group_monitor_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  DeclareAndLoadParameters();

  // ── Subscriptions (all atomic stores — lightweight) ────────────────────────
  auto sub_opts = rclcpp::SubscriptionOptions();
  sub_opts.callback_group = cb_group_monitor_;

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnJointState(std::move(msg)); },
      sub_opts);

  robot_mode_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      robot_mode_topic_, 10,
      [this](std_msgs::msg::Int32::SharedPtr msg) { OnRobotMode(std::move(msg)); },
      sub_opts);

  safety_mode_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      safety_mode_topic_, 10,
      [this](std_msgs::msg::Int32::SharedPtr msg) { OnSafetyMode(std::move(msg)); },
      sub_opts);

  program_running_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      program_running_topic_, 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) { OnProgramRunning(std::move(msg)); },
      sub_opts);

  // ── Active controller name subscription (transient_local QoS) ─────────────
  if (enable_controller_stats_) {
    auto ctrl_qos = rclcpp::QoS(1)
        .reliable()
        .transient_local();
    controller_name_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/ur5e/active_controller_name", ctrl_qos,
        [this](std_msgs::msg::String::SharedPtr msg) {
          OnControllerNameChanged(std::move(msg));
        },
        sub_opts);
  }

  // ── Publisher ──────────────────────────────────────────────────────────────
  diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

  // ── Service client ─────────────────────────────────────────────────────────
  list_controllers_client_ =
      node_->create_client<controller_manager_msgs::srv::ListControllers>(
          "/controller_manager/list_controllers",
          rmw_qos_profile_services_default,
          cb_group_monitor_);

  RCLCPP_INFO(node_->get_logger(), "[StatusMonitor] Initialized — topics: %s, %s, %s, %s",
              joint_states_topic_.c_str(), robot_mode_topic_.c_str(),
              safety_mode_topic_.c_str(), program_running_topic_.c_str());
}

// ── Destructor ───────────────────────────────────────────────────────────────

RtcStatusMonitor::~RtcStatusMonitor() {
  stop();
}

// ── DeclareAndLoadParameters ─────────────────────────────────────────────────

void RtcStatusMonitor::DeclareAndLoadParameters() {
  auto declare = [this](const std::string& name, auto default_val) {
    return node_->declare_parameter(
        "status_monitor." + name, default_val);
  };

  // Topic names
  joint_states_topic_    = declare("joint_states_topic",    std::string("/joint_states"));
  robot_mode_topic_      = declare("robot_mode_topic",
                                   std::string("/io_and_status_controller/robot_mode"));
  safety_mode_topic_     = declare("safety_mode_topic",
                                   std::string("/io_and_status_controller/safety_mode"));
  program_running_topic_ = declare("program_running_topic",
                                   std::string("/io_and_status_controller/robot_program_running"));

  // Tracking error thresholds
  tracking_error_pos_warn_rad_  = declare("tracking_error_pos_warn_rad",  0.05);
  tracking_error_pos_fault_rad_ = declare("tracking_error_pos_fault_rad", 0.15);
  tracking_error_vel_warn_rad_  = declare("tracking_error_vel_warn_rad",  0.1);
  tracking_error_vel_fault_rad_ = declare("tracking_error_vel_fault_rad", 0.3);

  // Joint limit margins (parameters in degrees, stored in radians)
  const double warn_deg  = declare("joint_limit_warn_margin_deg",  5.0);
  const double fault_deg = declare("joint_limit_fault_margin_deg", 1.0);
  joint_limit_warn_margin_rad_  = warn_deg  * kDegToRad;
  joint_limit_fault_margin_rad_ = fault_deg * kDegToRad;

  // Watchdog
  watchdog_timeout_sec_ = declare("watchdog_timeout_sec", 1.0);

  // Controller manager
  controller_poll_interval_sec_ = declare("controller_poll_interval_sec", 5.0);
  target_controller_ = declare("target_controller",
                               std::string("scaled_joint_trajectory_controller"));

  // Recovery
  auto_recovery_          = declare("auto_recovery",          false);
  max_recovery_attempts_  = static_cast<int>(declare("max_recovery_attempts",  3L));
  recovery_interval_sec_  = declare("recovery_interval_sec",  5.0);

  // Controller stats
  enable_controller_stats_ = declare("enable_controller_stats", true);

  // Logging — 세션 디렉토리 통합
  log_output_dir_ = declare("log_output_dir", std::string(""));

  if (log_output_dir_.empty()) {
    // UR5E_SESSION_DIR 환경변수에서 세션 디렉토리 읽기
    const char* session_env = std::getenv("UR5E_SESSION_DIR");
    if (session_env != nullptr && session_env[0] != '\0') {
      log_output_dir_ = std::string(session_env) + "/monitor";
    } else {
      // fallback: ~/.ros/
      const char* home = std::getenv("HOME");
      log_output_dir_ = home ? std::string(home) + "/.ros" : "/tmp";
    }
  } else if (!log_output_dir_.empty() && log_output_dir_[0] == '~') {
    // Expand ~ to HOME
    const char* home = std::getenv("HOME");
    if (home != nullptr) {
      log_output_dir_ = std::string(home) + log_output_dir_.substr(1);
    }
  }

  // Joint limits (overridable via parameter array)
  // Default: UR5e limits per spec
  for (int i = 0; i < kNumJoints; ++i) {
    const std::string prefix = "status_monitor.joint_limits.joint_" + std::to_string(i);
    const auto lower = node_->declare_parameter(prefix + ".lower",
                                                kDefaultJointLimits[static_cast<std::size_t>(i)][0]);
    const auto upper = node_->declare_parameter(prefix + ".upper",
                                                kDefaultJointLimits[static_cast<std::size_t>(i)][1]);
    joint_limits_[static_cast<std::size_t>(i)] = {lower, upper};
  }
}

// ── Subscription Callbacks ───────────────────────────────────────────────────

void RtcStatusMonitor::OnJointState(sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!msg || msg->position.size() < static_cast<std::size_t>(kNumJoints)) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    for (int i = 0; i < kNumJoints; ++i) {
      q_actual_[static_cast<std::size_t>(i)]  = msg->position[static_cast<std::size_t>(i)];
      qd_actual_[static_cast<std::size_t>(i)] =
          (msg->velocity.size() > static_cast<std::size_t>(i))
              ? msg->velocity[static_cast<std::size_t>(i)]
              : 0.0;
    }
  }

  joint_state_received_.store(true, std::memory_order_release);
  last_joint_state_time_.store(
      std::chrono::steady_clock::now().time_since_epoch().count(),
      std::memory_order_release);

  // ── Packet counting ─────────────────────────────────────────────────────
  ++js_stats_.total_count;
  ++rate_window_count_;
}

void RtcStatusMonitor::OnRobotMode(std_msgs::msg::Int32::SharedPtr msg) {
  if (!msg) return;
  const auto prev = robot_mode_.exchange(msg->data, std::memory_order_release);
  if (prev != msg->data) {
    RCLCPP_INFO(node_->get_logger(),
                "[StatusMonitor] RobotMode: %s → %s (%d → %d)",
                std::string(RobotModeToString(prev)).c_str(),
                std::string(RobotModeToString(msg->data)).c_str(),
                prev, msg->data);
  }
}

void RtcStatusMonitor::OnSafetyMode(std_msgs::msg::Int32::SharedPtr msg) {
  if (!msg) return;
  const auto prev = safety_mode_.exchange(msg->data, std::memory_order_release);
  if (prev != msg->data) {
    RCLCPP_INFO(node_->get_logger(),
                "[StatusMonitor] SafetyMode: %s → %s (%d → %d)",
                std::string(SafetyModeToString(prev)).c_str(),
                std::string(SafetyModeToString(msg->data)).c_str(),
                prev, msg->data);
  }
}

void RtcStatusMonitor::OnProgramRunning(std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg) return;
  const bool prev = program_running_.exchange(msg->data, std::memory_order_release);
  if (prev != msg->data) {
    RCLCPP_INFO(node_->get_logger(),
                "[StatusMonitor] ProgramRunning: %s → %s",
                prev ? "true" : "false",
                msg->data ? "true" : "false");
  }
}

// ── RT-safe accessors ────────────────────────────────────────────────────────

bool RtcStatusMonitor::isReady() const noexcept {
  return is_ready_.load(std::memory_order_acquire);
}

FailureType RtcStatusMonitor::getFailure() const noexcept {
  return failure_type_.load(std::memory_order_acquire);
}

bool RtcStatusMonitor::isJointLimitWarning() const noexcept {
  return joint_limit_warning_.load(std::memory_order_acquire);
}

// ── waitForReady ─────────────────────────────────────────────────────────────

bool RtcStatusMonitor::waitForReady(double timeout_sec) {
  const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::duration<double>(timeout_sec);
  while (!is_ready_.load(std::memory_order_acquire)) {
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_WARN(node_->get_logger(),
                  "[StatusMonitor] waitForReady timed out after %.1f s", timeout_sec);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  RCLCPP_INFO(node_->get_logger(), "[StatusMonitor] Robot is ready");
  return true;
}

// ── setJointReference ────────────────────────────────────────────────────────

void RtcStatusMonitor::setJointReference(
    const std::array<double, kNumJoints>& q_ref,
    const std::array<double, kNumJoints>& qd_ref) noexcept {
  // try_lock: never block the RT thread
  std::unique_lock<std::mutex> lock(ref_mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    q_ref_     = q_ref;
    qd_ref_    = qd_ref;
    ref_valid_ = true;
  }
}

// ── Callback registration ────────────────────────────────────────────────────

void RtcStatusMonitor::registerOnReady(std::function<void()> cb) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  on_ready_cb_ = std::move(cb);
}

void RtcStatusMonitor::registerOnFailure(
    std::function<void(FailureType, const FailureContext&)> cb) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  on_failure_cb_ = std::move(cb);
}

void RtcStatusMonitor::registerOnWarning(
    std::function<void(WarningType, const std::string&)> cb) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  on_warning_cb_ = std::move(cb);
}

void RtcStatusMonitor::registerOnRecovery(std::function<void()> cb) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  on_recovery_cb_ = std::move(cb);
}

// ── Lifecycle ────────────────────────────────────────────────────────────────

void RtcStatusMonitor::start(
    const rtc::ThreadConfig& thread_cfg) {
  if (monitor_thread_.joinable()) {
    RCLCPP_WARN(node_->get_logger(), "[StatusMonitor] Already running");
    return;
  }
  start_time_ = std::chrono::steady_clock::now();
  rate_window_start_ = start_time_;
  monitor_thread_ = std::jthread([this, thread_cfg](std::stop_token st) {
    rtc::ApplyThreadConfigWithFallback(thread_cfg);
    MonitorLoop(std::move(st));
  });
  RCLCPP_INFO(node_->get_logger(), "[StatusMonitor] Monitor thread started (10 Hz)");
}

void RtcStatusMonitor::stop() {
  if (monitor_thread_.joinable()) {
    monitor_thread_.request_stop();
    monitor_thread_.join();
    RCLCPP_INFO(node_->get_logger(), "[StatusMonitor] Monitor thread stopped");
  }
  if (enable_controller_stats_) {
    SaveControllerStatsJson();
  }
}

rclcpp::CallbackGroup::SharedPtr RtcStatusMonitor::GetCallbackGroup() const {
  return cb_group_monitor_;
}

// ── MonitorLoop ──────────────────────────────────────────────────────────────

void RtcStatusMonitor::MonitorLoop(std::stop_token stop_token) {
  constexpr auto kMonitorPeriod = std::chrono::milliseconds(100);  // 10 Hz
  uint64_t iteration = 0;

  RCLCPP_INFO(node_->get_logger(), "[StatusMonitor] Monitor loop running");

  while (!stop_token.stop_requested()) {
    const auto cycle_start = std::chrono::steady_clock::now();

    // ── Per-cycle checks ─────────────────────────────────────────────────────
    CheckJointStateWatchdog();
    CheckRobotStatus();
    CheckTrackingErrors();
    CheckJointLimits();

    // ── Rate calculation ──────────────────────────────────────────────────────
    const auto now = std::chrono::steady_clock::now();
    {
      const double elapsed =
          std::chrono::duration<double>(now - rate_window_start_).count();
      if (elapsed >= 1.0) {
        js_stats_.current_rate_hz =
            static_cast<double>(rate_window_count_) / elapsed;
        rate_window_count_ = 0;
        rate_window_start_ = now;
      }
    }

    // ── Per-controller stats update ─────────────────────────────────────────
    if (enable_controller_stats_) {
      auto it = controller_stats_.find(current_controller_name_);
      if (it != controller_stats_.end()) {
        it->second.js_packets = js_stats_.total_count;
        it->second.js_timeouts = js_stats_.timeout_count;
        it->second.last_active = now;
      }
    }

    // Controller manager poll (less frequent)
    const auto poll_interval = std::chrono::duration<double>(controller_poll_interval_sec_);
    if (now - last_controller_poll_time_ >= poll_interval) {
      CheckControllerManager();
      last_controller_poll_time_ = now;
    }

    // ── Readiness check ──────────────────────────────────────────────────────
    CheckReadiness();

    // ── Record state history ─────────────────────────────────────────────────
    {
      StateHistoryEntry entry;
      entry.timestamp          = now;
      entry.robot_mode         = robot_mode_.load(std::memory_order_acquire);
      entry.safety_mode        = safety_mode_.load(std::memory_order_acquire);
      entry.program_running    = program_running_.load(std::memory_order_acquire);
      entry.joint_limit_warning = joint_limit_warning_.load(std::memory_order_acquire);

      {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        entry.q  = q_actual_;
        entry.qd = qd_actual_;
      }

      // Compute tracking error for history
      {
        std::lock_guard<std::mutex> lock(ref_mutex_);
        if (ref_valid_) {
          for (int i = 0; i < kNumJoints; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            entry.tracking_error[idx] = q_ref_[idx] - entry.q[idx];
          }
        }
      }

      {
        std::lock_guard<std::mutex> lock(history_mutex_);
        state_history_.Push(entry);
      }
    }

    // ── Diagnostics (1 Hz) ───────────────────────────────────────────────────
    if (now - last_diag_time_ >= std::chrono::seconds(1)) {
      PublishDiagnostics();
      last_diag_time_ = now;
    }

    ++iteration;

    // ── Sleep remainder of period ────────────────────────────────────────────
    const auto elapsed = std::chrono::steady_clock::now() - cycle_start;
    if (elapsed < kMonitorPeriod) {
      std::this_thread::sleep_for(kMonitorPeriod - elapsed);
    }
  }

  RCLCPP_INFO(node_->get_logger(),
              "[StatusMonitor] Monitor loop exited after %lu iterations",
              static_cast<unsigned long>(iteration));
}

// ── CheckReadiness ───────────────────────────────────────────────────────────

void RtcStatusMonitor::CheckReadiness() {
  // Don't report ready if a failure is active
  if (failure_type_.load(std::memory_order_acquire) != FailureType::kNone) {
    is_ready_.store(false, std::memory_order_release);
    return;
  }

  const bool robot_running =
      (robot_mode_.load(std::memory_order_acquire) ==
       static_cast<int32_t>(URRobotMode::kRunning));
  const bool safety_normal =
      (safety_mode_.load(std::memory_order_acquire) ==
       static_cast<int32_t>(URSafetyMode::kNormal));
  const bool prog_running  = program_running_.load(std::memory_order_acquire);
  const bool js_received   = joint_state_received_.load(std::memory_order_acquire);

  // Check joint_states recency
  bool js_recent = false;
  if (js_received) {
    const auto last_ns = last_joint_state_time_.load(std::memory_order_acquire);
    const auto last_tp = std::chrono::steady_clock::time_point(
        std::chrono::steady_clock::duration(last_ns));
    const auto age = std::chrono::steady_clock::now() - last_tp;
    js_recent = (age < std::chrono::duration<double>(watchdog_timeout_sec_));
  }

  const bool all_ready = robot_running && safety_normal && prog_running && js_recent;

  const bool was_ready = is_ready_.load(std::memory_order_acquire);
  is_ready_.store(all_ready, std::memory_order_release);

  // Fire on-ready callback on first transition to ready
  if (all_ready && !was_ready && !ready_callback_fired_) {
    ready_callback_fired_ = true;
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (on_ready_cb_) {
      on_ready_cb_();
    }
  }
}

// ── CheckJointStateWatchdog ──────────────────────────────────────────────────

void RtcStatusMonitor::CheckJointStateWatchdog() {
  if (!joint_state_received_.load(std::memory_order_acquire)) {
    return;  // Not received yet — don't trigger during startup
  }

  const auto last_ns = last_joint_state_time_.load(std::memory_order_acquire);
  const auto last_tp = std::chrono::steady_clock::time_point(
      std::chrono::steady_clock::duration(last_ns));
  const auto age     = std::chrono::steady_clock::now() - last_tp;
  const auto timeout = std::chrono::duration<double>(watchdog_timeout_sec_);

  if (age > timeout) {
    ++js_stats_.timeout_count;
    RaiseFailure(FailureType::kWatchdogTimeout,
        "joint_states not received for " +
        std::to_string(std::chrono::duration<double>(age).count()) + " s "
        "(timeout: " + std::to_string(watchdog_timeout_sec_) + " s)");
  }
}

// ── CheckRobotStatus ─────────────────────────────────────────────────────────

void RtcStatusMonitor::CheckRobotStatus() {
  const auto safety = static_cast<URSafetyMode>(
      safety_mode_.load(std::memory_order_acquire));

  switch (safety) {
    case URSafetyMode::kProtectiveStop:
      RaiseFailure(FailureType::kProtectiveStop, "Protective stop triggered");
      return;
    case URSafetyMode::kSafeguardStop:
    case URSafetyMode::kSystemEmergencyStop:
    case URSafetyMode::kRobotEmergencyStop:
      RaiseFailure(FailureType::kEstop, "Emergency stop — safety_mode: " +
                   std::string(SafetyModeToString(safety)));
      return;
    case URSafetyMode::kViolation:
    case URSafetyMode::kFault:
      RaiseFailure(FailureType::kSafetyViolation, "Safety violation — safety_mode: " +
                   std::string(SafetyModeToString(safety)));
      return;
    default:
      break;
  }

  // Check program running
  if (joint_state_received_.load(std::memory_order_acquire) &&
      !program_running_.load(std::memory_order_acquire)) {
    // Only warn if we previously had the program running (avoid startup false alarm)
    if (is_ready_.load(std::memory_order_acquire)) {
      RaiseFailure(FailureType::kProgramDisconnected,
                   "Robot program stopped running");
    }
  }
}

// ── CheckTrackingErrors ──────────────────────────────────────────────────────

void RtcStatusMonitor::CheckTrackingErrors() {
  // Snapshot reference (try_lock — skip if contended)
  std::array<double, kNumJoints> q_ref{};
  std::array<double, kNumJoints> qd_ref{};
  bool valid = false;
  {
    std::unique_lock<std::mutex> lock(ref_mutex_, std::try_to_lock);
    if (lock.owns_lock() && ref_valid_) {
      q_ref  = q_ref_;
      qd_ref = qd_ref_;
      valid  = true;
    }
  }
  if (!valid) return;

  // Snapshot actual
  std::array<double, kNumJoints> q_act{};
  std::array<double, kNumJoints> qd_act{};
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    q_act  = q_actual_;
    qd_act = qd_actual_;
  }

  bool any_warn = false;
  for (int i = 0; i < kNumJoints; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    const double pos_err = std::abs(q_ref[idx] - q_act[idx]);
    const double vel_err = std::abs(qd_ref[idx] - qd_act[idx]);

    // Fault level
    if (pos_err > tracking_error_pos_fault_rad_) {
      RaiseFailure(FailureType::kTrackingError,
          "Joint " + std::to_string(i) + " position tracking error: " +
          std::to_string(pos_err) + " rad (threshold: " +
          std::to_string(tracking_error_pos_fault_rad_) + " rad)");
      return;
    }
    if (vel_err > tracking_error_vel_fault_rad_) {
      RaiseFailure(FailureType::kTrackingError,
          "Joint " + std::to_string(i) + " velocity tracking error: " +
          std::to_string(vel_err) + " rad/s (threshold: " +
          std::to_string(tracking_error_vel_fault_rad_) + " rad/s)");
      return;
    }

    // Warning level
    if (pos_err > tracking_error_pos_warn_rad_ ||
        vel_err > tracking_error_vel_warn_rad_) {
      any_warn = true;
      std::lock_guard<std::mutex> lock(callback_mutex_);
      if (on_warning_cb_) {
        on_warning_cb_(WarningType::kTrackingErrorHigh,
            "Joint " + std::to_string(i) +
            " tracking: pos_err=" + std::to_string(pos_err) +
            " vel_err=" + std::to_string(vel_err));
      }
    }
  }

  // No separate atomic for tracking error warning in the user spec,
  // but useful for RT thread awareness
  (void)any_warn;
}

// ── CheckJointLimits ─────────────────────────────────────────────────────────

void RtcStatusMonitor::CheckJointLimits() {
  std::array<double, kNumJoints> q{};
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    q = q_actual_;
  }

  bool any_warning = false;

  for (int i = 0; i < kNumJoints; ++i) {
    const auto idx     = static_cast<std::size_t>(i);
    const double lower = joint_limits_[idx][0];
    const double upper = joint_limits_[idx][1];
    const double pos   = q[idx];

    const double margin_lower = pos - lower;
    const double margin_upper = upper - pos;
    const double min_margin   = std::min(margin_lower, margin_upper);

    // Fault zone
    if (min_margin < joint_limit_fault_margin_rad_) {
      RaiseFailure(FailureType::kJointLimitViolation,
          "Joint " + std::to_string(i) + " at limit — pos: " +
          std::to_string(pos) + " rad, margin: " +
          std::to_string(min_margin) + " rad");
      return;
    }

    // Warning zone
    if (min_margin < joint_limit_warn_margin_rad_) {
      any_warning = true;
      std::lock_guard<std::mutex> lock(callback_mutex_);
      if (on_warning_cb_) {
        on_warning_cb_(WarningType::kJointLimitProximity,
            "Joint " + std::to_string(i) + " near limit — margin: " +
            std::to_string(min_margin / kDegToRad) + " deg");
      }
    }
  }

  joint_limit_warning_.store(any_warning, std::memory_order_release);
}

// ── CheckControllerManager ───────────────────────────────────────────────────

void RtcStatusMonitor::CheckControllerManager() {
  if (!list_controllers_client_->service_is_ready()) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "[StatusMonitor] controller_manager service not available");
    target_controller_active_ = false;
    return;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future  = list_controllers_client_->async_send_request(request);

  // Wait with timeout (from monitor thread — not blocking any executor)
  const auto status = future.wait_for(std::chrono::seconds(2));
  if (status != std::future_status::ready) {
    RCLCPP_WARN(node_->get_logger(),
                "[StatusMonitor] list_controllers timed out");
    target_controller_active_ = false;
    return;
  }

  auto response = future.get();
  bool found_active = false;
  bool found_jbs    = false;

  for (const auto& ctrl : response->controller) {
    if (ctrl.name == target_controller_ && ctrl.state == "active") {
      found_active = true;
    }
    if (ctrl.name == "joint_state_broadcaster" && ctrl.state == "active") {
      found_jbs = true;
    }
  }

  target_controller_active_ = found_active;

  if (is_ready_.load(std::memory_order_acquire) && !found_active) {
    RaiseFailure(FailureType::kControllerInactive,
        "Target controller '" + target_controller_ + "' not active");
  }

  if (is_ready_.load(std::memory_order_acquire) && !found_jbs) {
    RCLCPP_WARN(node_->get_logger(),
                "[StatusMonitor] joint_state_broadcaster not active");
  }
}

// ── PublishDiagnostics ───────────────────────────────────────────────────────

void RtcStatusMonitor::PublishDiagnostics() {
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = node_->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name        = "rtc_status_monitor";
  status.hardware_id = "ur5e";

  const auto failure = failure_type_.load(std::memory_order_acquire);
  const bool ready   = is_ready_.load(std::memory_order_acquire);

  if (failure != FailureType::kNone) {
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "FAILURE: " + std::string(FailureTypeToString(failure));
  } else if (ready) {
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Ready";
  } else {
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Not ready";
  }

  // Key-value pairs
  auto add_kv = [&status](const std::string& key, const std::string& val) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key   = key;
    kv.value = val;
    status.values.push_back(kv);
  };

  add_kv("is_ready",       ready ? "true" : "false");
  add_kv("failure_type",   std::string(FailureTypeToString(failure)));
  add_kv("robot_mode",     std::string(RobotModeToString(
                               robot_mode_.load(std::memory_order_acquire))));
  add_kv("safety_mode",    std::string(SafetyModeToString(
                               safety_mode_.load(std::memory_order_acquire))));
  add_kv("program_running",
         program_running_.load(std::memory_order_acquire) ? "true" : "false");

  const auto uptime = std::chrono::steady_clock::now() - start_time_;
  add_kv("uptime_sec",     std::to_string(
      std::chrono::duration<double>(uptime).count()));

  add_kv("joint_limit_warning",
         joint_limit_warning_.load(std::memory_order_acquire) ? "true" : "false");
  add_kv("target_controller", target_controller_);
  add_kv("target_controller_active", target_controller_active_ ? "true" : "false");
  add_kv("recovery_attempts", std::to_string(recovery_attempts_));
  add_kv("joint_state_rate",     std::to_string(js_stats_.current_rate_hz));
  add_kv("joint_state_total",    std::to_string(js_stats_.total_count));
  add_kv("joint_state_timeouts", std::to_string(js_stats_.timeout_count));
  add_kv("active_controller",    current_controller_name_);

  // Compute max tracking error for diagnostics
  {
    std::array<double, kNumJoints> q_ref_snap{};
    std::array<double, kNumJoints> q_act_snap{};
    bool valid = false;
    {
      std::unique_lock<std::mutex> lock(ref_mutex_, std::try_to_lock);
      if (lock.owns_lock() && ref_valid_) {
        q_ref_snap = q_ref_;
        valid = true;
      }
    }
    if (valid) {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      q_act_snap = q_actual_;
    }
    if (valid) {
      double max_err = 0.0;
      for (int i = 0; i < kNumJoints; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        max_err = std::max(max_err, std::abs(q_ref_snap[idx] - q_act_snap[idx]));
      }
      add_kv("tracking_error_max_rad", std::to_string(max_err));
    }
  }

  // Compute closest joint limit margin
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    double min_margin_deg = 360.0;
    for (int i = 0; i < kNumJoints; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      const double margin_lower = q_actual_[idx] - joint_limits_[idx][0];
      const double margin_upper = joint_limits_[idx][1] - q_actual_[idx];
      const double margin = std::min(margin_lower, margin_upper) / kDegToRad;
      min_margin_deg = std::min(min_margin_deg, margin);
    }
    add_kv("joint_limit_closest_margin_deg", std::to_string(min_margin_deg));
  }

  diag_array.status.push_back(status);
  diag_pub_->publish(diag_array);
}

// ── RaiseFailure ─────────────────────────────────────────────────────────────

void RtcStatusMonitor::RaiseFailure(FailureType type, const std::string& description) {
  // Only raise if no failure is already active
  auto expected = FailureType::kNone;
  if (!failure_type_.compare_exchange_strong(expected, type,
          std::memory_order_release, std::memory_order_acquire)) {
    return;  // A failure is already active
  }

  is_ready_.store(false, std::memory_order_release);

  // Increment per-controller failure count
  if (enable_controller_stats_) {
    auto it = controller_stats_.find(current_controller_name_);
    if (it != controller_stats_.end()) {
      ++it->second.failure_count;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "[StatusMonitor] FAILURE: %s — %s",
               std::string(FailureTypeToString(type)).c_str(),
               description.c_str());

  // Build failure context
  FailureContext ctx;
  ctx.type         = type;
  ctx.timestamp_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  ctx.robot_mode   = robot_mode_.load(std::memory_order_acquire);
  ctx.safety_mode  = safety_mode_.load(std::memory_order_acquire);
  ctx.description  = description;

  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    ctx.q_actual_at_failure  = q_actual_;
    ctx.qd_actual_at_failure = qd_actual_;
  }
  {
    std::lock_guard<std::mutex> lock(ref_mutex_);
    ctx.q_ref_at_failure = q_ref_;
    for (int i = 0; i < kNumJoints; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      ctx.tracking_error_at_failure[idx] =
          ctx.q_ref_at_failure[idx] - ctx.q_actual_at_failure[idx];
    }
  }

  // Invoke failure callback
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (on_failure_cb_) {
      on_failure_cb_(type, ctx);
    }
  }

  // Log to file
  LogFailureToFile(ctx);

  // Attempt recovery if enabled
  if (auto_recovery_) {
    AttemptRecovery(type);
  }
}

// ── ClearFailure ─────────────────────────────────────────────────────────────

void RtcStatusMonitor::ClearFailure() {
  failure_type_.store(FailureType::kNone, std::memory_order_release);
  ready_callback_fired_ = false;
  RCLCPP_INFO(node_->get_logger(), "[StatusMonitor] Failure cleared");
}

// ── LogFailureToFile ─────────────────────────────────────────────────────────

void RtcStatusMonitor::LogFailureToFile(const FailureContext& ctx) {
  try {
    std::filesystem::create_directories(log_output_dir_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[StatusMonitor] Failed to create log directory '%s': %s",
                 log_output_dir_.c_str(), e.what());
    return;
  }

  const std::string filename = log_output_dir_ + "/ur5e_failure_" +
                               SteadyToIso8601() + ".log";
  std::ofstream ofs(filename);
  if (!ofs.is_open()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[StatusMonitor] Failed to open failure log: %s", filename.c_str());
    return;
  }

  // ── [HEADER] ───────────────────────────────────────────────────────────────
  ofs << "[HEADER]\n";
  ofs << "Timestamp:        " << SteadyToIso8601() << '\n';
  ofs << "FailureType:      " << FailureTypeToString(ctx.type) << '\n';
  ofs << "Description:      " << ctx.description << '\n';
  ofs << "RobotMode:        " << ctx.robot_mode << " ("
      << RobotModeToString(ctx.robot_mode) << ")\n";
  ofs << "SafetyMode:       " << ctx.safety_mode << " ("
      << SafetyModeToString(ctx.safety_mode) << ")\n";
  ofs << '\n';

  // ── [STATE AT FAILURE] ─────────────────────────────────────────────────────
  ofs << "[STATE AT FAILURE]\n";
  ofs << "q_actual:         " << FormatArray(ctx.q_actual_at_failure) << " rad\n";
  ofs << "qd_actual:        " << FormatArray(ctx.qd_actual_at_failure) << " rad/s\n";
  ofs << "q_reference:      " << FormatArray(ctx.q_ref_at_failure) << " rad\n";
  ofs << "tracking_error:   " << FormatArray(ctx.tracking_error_at_failure) << " rad\n";
  ofs << '\n';

  // ── [STATE HISTORY] ────────────────────────────────────────────────────────
  ofs << "[STATE HISTORY — last 10 seconds at 10Hz]\n";
  ofs << "timestamp_ms, robot_mode, safety_mode, "
         "q[0], q[1], q[2], q[3], q[4], q[5], "
         "qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], "
         "tracking_err[0], tracking_err[1], tracking_err[2], "
         "tracking_err[3], tracking_err[4], tracking_err[5], "
         "joint_limit_warning\n";

  std::vector<StateHistoryEntry> history;
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    history = state_history_.GetAll();
  }

  for (const auto& entry : history) {
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        entry.timestamp.time_since_epoch()).count();
    ofs << ms << ", "
        << entry.robot_mode << ", " << entry.safety_mode;
    for (int i = 0; i < kNumJoints; ++i) {
      ofs << ", " << std::fixed << std::setprecision(6) << entry.q[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < kNumJoints; ++i) {
      ofs << ", " << std::fixed << std::setprecision(6) << entry.qd[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < kNumJoints; ++i) {
      ofs << ", " << std::fixed << std::setprecision(6)
          << entry.tracking_error[static_cast<std::size_t>(i)];
    }
    ofs << ", " << (entry.joint_limit_warning ? 1 : 0) << '\n';
  }
  ofs << '\n';

  // ── [CONTROLLER STATE AT FAILURE] ──────────────────────────────────────────
  ofs << "[CONTROLLER STATE AT FAILURE]\n";
  ofs << target_controller_ << ": "
      << (target_controller_active_ ? "active" : "inactive") << '\n';

  ofs.close();
  RCLCPP_INFO(node_->get_logger(),
              "[StatusMonitor] Failure log written: %s", filename.c_str());
}

// ── AttemptRecovery ──────────────────────────────────────────────────────────

void RtcStatusMonitor::AttemptRecovery(FailureType type) {
  // Only attempt recovery for specific failure types
  if (type != FailureType::kProtectiveStop &&
      type != FailureType::kProgramDisconnected) {
    RCLCPP_WARN(node_->get_logger(),
                "[StatusMonitor] No auto-recovery for %s",
                std::string(FailureTypeToString(type)).c_str());
    return;
  }

  // Check cooldown
  const auto now = std::chrono::steady_clock::now();
  const auto cooldown = std::chrono::duration<double>(recovery_interval_sec_);
  if (now - last_recovery_time_ < cooldown) {
    RCLCPP_INFO(node_->get_logger(),
                "[StatusMonitor] Recovery cooldown active — skipping");
    return;
  }

  // Check max attempts
  if (recovery_attempts_ >= max_recovery_attempts_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[StatusMonitor] Max recovery attempts (%d) exceeded — staying failed",
                 max_recovery_attempts_);
    return;
  }

  ++recovery_attempts_;
  last_recovery_time_ = now;

  RCLCPP_INFO(node_->get_logger(),
              "[StatusMonitor] Attempting recovery (%d/%d) for %s",
              recovery_attempts_, max_recovery_attempts_,
              std::string(FailureTypeToString(type)).c_str());

  // NOTE: Actual dashboard service calls require ur_dashboard_msgs.
  // When HAVE_UR_DASHBOARD_MSGS is defined, this would call:
  //   - /dashboard_client/unlock_protective_stop (for kProtectiveStop)
  //   - /dashboard_client/play (for kProgramDisconnected)
  // Currently, recovery is logged but dashboard calls are not implemented
  // without the ur_dashboard_msgs dependency.

  RCLCPP_WARN(node_->get_logger(),
              "[StatusMonitor] Dashboard recovery not available "
              "(build with ur_dashboard_msgs for full recovery support)");

  // If recovery succeeds (e.g., safety mode returns to normal),
  // CheckReadiness() will detect it and ClearFailure() + fire callbacks.
  // For now, we clear the failure to allow re-checking.
  ClearFailure();

  // Wait briefly and re-check
  std::this_thread::sleep_for(std::chrono::seconds(2));

  const auto safety = static_cast<URSafetyMode>(
      safety_mode_.load(std::memory_order_acquire));
  if (safety == URSafetyMode::kNormal) {
    RCLCPP_INFO(node_->get_logger(), "[StatusMonitor] Recovery successful!");
    recovery_attempts_ = 0;
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (on_recovery_cb_) {
      on_recovery_cb_();
    }
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "[StatusMonitor] Recovery attempt %d failed — safety_mode: %s",
                recovery_attempts_,
                std::string(SafetyModeToString(safety)).c_str());
  }
}

// ── getJointStateStats ────────────────────────────────────────────────────────

RtcStatusMonitor::MessageStats RtcStatusMonitor::getJointStateStats() const noexcept {
  return js_stats_;
}

// ── OnControllerNameChanged ──────────────────────────────────────────────────

void RtcStatusMonitor::OnControllerNameChanged(
    const std_msgs::msg::String::SharedPtr msg) {
  if (!msg) return;

  const std::string& new_name = msg->data;
  if (new_name == current_controller_name_) {
    return;  // No change
  }

  const auto now = std::chrono::steady_clock::now();

  // Update total_active_sec for the previous controller
  auto prev_it = controller_stats_.find(current_controller_name_);
  if (prev_it != controller_stats_.end() &&
      prev_it->second.last_active.time_since_epoch().count() > 0) {
    prev_it->second.total_active_sec +=
        std::chrono::duration<double>(now - prev_it->second.last_active).count();
  }

  RCLCPP_INFO(node_->get_logger(),
              "[StatusMonitor] Active controller: %s → %s",
              current_controller_name_.c_str(), new_name.c_str());

  current_controller_name_ = new_name;

  // Create new entry if not exists
  auto [it, inserted] = controller_stats_.try_emplace(new_name);
  if (inserted) {
    it->second.name = new_name;
    it->second.first_active = now;
  }
  it->second.last_active = now;
}

// ── SaveControllerStatsJson ──────────────────────────────────────────────────

void RtcStatusMonitor::SaveControllerStatsJson() const {
  if (controller_stats_.empty()) {
    return;
  }

  try {
    std::filesystem::create_directories(log_output_dir_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[StatusMonitor] Failed to create log directory '%s': %s",
                 log_output_dir_.c_str(), e.what());
    return;
  }

  const std::string filename = log_output_dir_ + "/controller_stats_" +
                               SteadyToIso8601() + ".json";
  std::ofstream ofs(filename);
  if (!ofs.is_open()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[StatusMonitor] Failed to open controller stats file: %s",
                 filename.c_str());
    return;
  }

  // Finalize active time for current controller
  const auto now = std::chrono::steady_clock::now();
  const double uptime =
      std::chrono::duration<double>(now - start_time_).count();

  ofs << "{\n";
  ofs << "  \"timestamp\": \"" << SteadyToIso8601() << "\",\n";
  ofs << "  \"uptime_sec\": " << std::fixed << std::setprecision(3) << uptime << ",\n";
  ofs << "  \"joint_state_total\": " << js_stats_.total_count << ",\n";
  ofs << "  \"joint_state_timeouts\": " << js_stats_.timeout_count << ",\n";
  ofs << "  \"joint_state_last_rate_hz\": " << std::fixed << std::setprecision(1)
      << js_stats_.current_rate_hz << ",\n";
  ofs << "  \"controllers\": {\n";

  std::size_t idx = 0;
  for (const auto& [name, stats] : controller_stats_) {
    // Calculate final active time for the current controller
    double active_sec = stats.total_active_sec;
    if (name == current_controller_name_ &&
        stats.last_active.time_since_epoch().count() > 0) {
      active_sec +=
          std::chrono::duration<double>(now - stats.last_active).count();
    }

    ofs << "    \"" << name << "\": {\n";
    ofs << "      \"total_active_sec\": " << std::fixed << std::setprecision(3)
        << active_sec << ",\n";
    ofs << "      \"js_packets\": " << stats.js_packets << ",\n";
    ofs << "      \"js_timeouts\": " << stats.js_timeouts << ",\n";
    ofs << "      \"failure_count\": " << stats.failure_count << "\n";
    ofs << "    }";
    if (++idx < controller_stats_.size()) {
      ofs << ',';
    }
    ofs << '\n';
  }

  ofs << "  }\n";
  ofs << "}\n";

  ofs.close();
  RCLCPP_INFO(node_->get_logger(),
              "[StatusMonitor] Controller stats written: %s", filename.c_str());
}

}  // namespace rtc
