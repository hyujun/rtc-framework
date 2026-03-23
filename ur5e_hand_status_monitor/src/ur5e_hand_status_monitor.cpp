// ── Project headers ──────────────────────────────────────────────────────────
#include "ur5e_hand_status_monitor/ur5e_hand_status_monitor.hpp"
#include "rtc_base/threading/thread_utils.hpp"

// ── C++ stdlib ───────────────────────────────────────────────────────────────
#include <algorithm>
#include <cmath>
#include <numeric>

using namespace std::chrono_literals;

namespace rtc {

namespace {

/// Helper to declare a parameter only if not already declared.
template <typename T>
T safe_get(rclcpp::Node& node, const std::string& name, const T& default_val)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, rclcpp::ParameterValue(default_val));
  }
  return node.get_parameter(name).get_value<T>();
}

}  // namespace

// ── Constructor ─────────────────────────────────────────────────────────────

Ur5eHandStatusMonitor::Ur5eHandStatusMonitor(rclcpp::Node::SharedPtr node)
  : node_(std::move(node))
{
  // Create the composed robot monitor
  robot_monitor_ = std::make_unique<RtcStatusMonitor>(node_);

  // Load hand-specific parameters
  DeclareAndLoadHandParameters();

  // Create hand subscriptions if enabled
  if (hand_monitoring_enabled_) {
    auto cb_group = robot_monitor_->GetCallbackGroup();
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group;

    hand_joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        hand_joint_states_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnHandJointState(std::move(msg)); },
        sub_opts);

    hand_sensor_state_sub_ = node_->create_subscription<rtc_msgs::msg::HandSensorState>(
        hand_sensor_states_topic_, rclcpp::SensorDataQoS(),
        [this](rtc_msgs::msg::HandSensorState::SharedPtr msg) { OnHandSensorState(std::move(msg)); },
        sub_opts);

    RCLCPP_INFO(node_->get_logger(),
        "[HandStatusMonitor] Hand monitoring enabled — sub: %s, %s",
        hand_joint_states_topic_.c_str(), hand_sensor_states_topic_.c_str());
  }
}

Ur5eHandStatusMonitor::~Ur5eHandStatusMonitor()
{
  stop();
}

// ── Parameter loading ───────────────────────────────────────────────────────

void Ur5eHandStatusMonitor::DeclareAndLoadHandParameters()
{
  hand_monitoring_enabled_   = safe_get(*node_, "status_monitor.hand.enabled", false);
  hand_joint_states_topic_   = safe_get<std::string>(*node_, "status_monitor.hand.joint_states_topic", "/hand/joint_states");
  hand_sensor_states_topic_  = safe_get<std::string>(*node_, "status_monitor.hand.sensor_states_topic", "/hand/sensor_states");
  hand_watchdog_timeout_sec_ = safe_get(*node_, "status_monitor.hand.watchdog_timeout_sec", 1.0);
  hand_min_rate_hz_          = safe_get(*node_, "status_monitor.hand.min_rate_hz", 30.0);
  hand_rate_fail_threshold_  = safe_get(*node_, "status_monitor.hand.rate_fail_threshold", 5);
  hand_data_fail_threshold_  = safe_get(*node_, "status_monitor.hand.data_fail_threshold", 5);
  hand_check_motor_          = safe_get(*node_, "status_monitor.hand.check_motor", true);
  hand_check_sensor_         = safe_get(*node_, "status_monitor.hand.check_sensor", true);
}

// ── RT-safe accessors (delegated) ───────────────────────────────────────────

bool Ur5eHandStatusMonitor::isReady() const noexcept
{
  if (!robot_monitor_->isReady()) return false;
  // If hand monitoring is active and a hand failure is set, not ready
  if (hand_monitoring_enabled_ &&
      hand_failure_type_.load(std::memory_order_acquire) != FailureType::kNone) {
    return false;
  }
  return true;
}

FailureType Ur5eHandStatusMonitor::getFailure() const noexcept
{
  auto robot_failure = robot_monitor_->getFailure();
  if (robot_failure != FailureType::kNone) return robot_failure;
  return hand_failure_type_.load(std::memory_order_acquire);
}

bool Ur5eHandStatusMonitor::isJointLimitWarning() const noexcept
{
  return robot_monitor_->isJointLimitWarning();
}

RtcStatusMonitor::MessageStats Ur5eHandStatusMonitor::getJointStateStats() const noexcept
{
  return robot_monitor_->getJointStateStats();
}

RtcStatusMonitor::MessageStats Ur5eHandStatusMonitor::getHandStats() const noexcept
{
  return hand_stats_;
}

bool Ur5eHandStatusMonitor::waitForReady(double timeout_sec)
{
  return robot_monitor_->waitForReady(timeout_sec);
}

void Ur5eHandStatusMonitor::setJointReference(
    const std::array<double, kNumRobotJoints>& q_ref,
    const std::array<double, kNumRobotJoints>& qd_ref) noexcept
{
  robot_monitor_->setJointReference(q_ref, qd_ref);
}

// ── Callback registration ───────────────────────────────────────────────────

void Ur5eHandStatusMonitor::registerOnReady(std::function<void()> cb)
{
  robot_monitor_->registerOnReady(std::move(cb));
}

void Ur5eHandStatusMonitor::registerOnFailure(
    std::function<void(FailureType, const FailureContext&)> cb)
{
  on_failure_cb_ = cb;
  // Also register with the robot monitor so robot failures trigger the same callback
  robot_monitor_->registerOnFailure(cb);
}

void Ur5eHandStatusMonitor::registerOnWarning(
    std::function<void(WarningType, const std::string&)> cb)
{
  on_warning_cb_ = cb;
  robot_monitor_->registerOnWarning(cb);
}

void Ur5eHandStatusMonitor::registerOnRecovery(std::function<void()> cb)
{
  robot_monitor_->registerOnRecovery(std::move(cb));
}

// ── Lifecycle ───────────────────────────────────────────────────────────────

void Ur5eHandStatusMonitor::start(const ThreadConfig& thread_cfg)
{
  // Start the robot monitor
  robot_monitor_->start(thread_cfg);

  // Start hand monitor thread if enabled
  if (hand_monitoring_enabled_) {
    hand_monitor_cfg_ = thread_cfg;
    hand_monitor_thread_ = std::jthread(
        [this](std::stop_token st) { HandMonitorLoop(std::move(st)); });

    RCLCPP_INFO(node_->get_logger(),
        "[HandStatusMonitor] Hand monitor started (10 Hz, Core %d)",
        thread_cfg.cpu_core);
  }
}

void Ur5eHandStatusMonitor::stop()
{
  if (hand_monitor_thread_.joinable()) {
    hand_monitor_thread_.request_stop();
    hand_monitor_thread_.join();
  }
  if (robot_monitor_) {
    robot_monitor_->stop();
  }
}

rclcpp::CallbackGroup::SharedPtr Ur5eHandStatusMonitor::GetCallbackGroup() const
{
  return robot_monitor_->GetCallbackGroup();
}

// ── Hand subscription callbacks ─────────────────────────────────────────────

void Ur5eHandStatusMonitor::OnHandJointState(
    sensor_msgs::msg::JointState::SharedPtr msg)
{
  const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  last_hand_state_time_ns_.store(now_ns, std::memory_order_release);
  hand_state_received_.store(true, std::memory_order_release);
  ++hand_rate_window_count_;

  // Store motor positions for data quality checks
  {
    std::lock_guard lock(hand_data_mutex_);
    hand_motor_positions_.assign(msg->position.begin(), msg->position.end());
  }
}

void Ur5eHandStatusMonitor::OnHandSensorState(
    rtc_msgs::msg::HandSensorState::SharedPtr msg)
{
  // Extract barometer values from fingertip sensors
  std::lock_guard lock(hand_data_mutex_);
  hand_sensor_values_.clear();
  for (const auto& ft : msg->fingertips) {
    for (const auto& v : ft.barometer) {
      hand_sensor_values_.push_back(v);
    }
    for (const auto& v : ft.tof) {
      hand_sensor_values_.push_back(v);
    }
  }
}

// ── Hand monitor thread ─────────────────────────────────────────────────────

void Ur5eHandStatusMonitor::HandMonitorLoop(std::stop_token stop_token)
{
  // Apply thread config (same core as status_monitor)
  ApplyThreadConfig(hand_monitor_cfg_);

  hand_rate_window_start_ = std::chrono::steady_clock::now();
  hand_rate_window_count_ = 0;

  while (!stop_token.stop_requested()) {
    std::this_thread::sleep_for(100ms);  // 10 Hz

    if (!hand_state_received_.load(std::memory_order_acquire)) {
      continue;  // No hand data yet — skip checks
    }

    CheckHandWatchdog();
    if (hand_check_motor_) CheckHandMotorData();
    if (hand_check_sensor_) CheckHandSensorData();
    CheckHandRate();
  }
}

// ── Hand check functions ────────────────────────────────────────────────────

void Ur5eHandStatusMonitor::CheckHandWatchdog()
{
  const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto last_ns = last_hand_state_time_ns_.load(std::memory_order_acquire);
  if (last_ns == 0) return;

  const double elapsed_sec = static_cast<double>(now_ns - last_ns) * 1e-9;
  if (elapsed_sec > hand_watchdog_timeout_sec_) {
    RaiseHandFailure(FailureType::kHandTimeout,
        "hand_timeout: no /hand/joint_states for " +
        std::to_string(elapsed_sec) + " sec");
  }
}

void Ur5eHandStatusMonitor::CheckHandMotorData()
{
  std::lock_guard lock(hand_data_mutex_);
  if (hand_motor_positions_.empty()) return;

  // All-zero check
  const bool all_zero = std::all_of(hand_motor_positions_.begin(),
      hand_motor_positions_.end(),
      [](double v) { return std::abs(v) < 1e-9; });

  if (all_zero) {
    ++hand_motor_zero_count_;
    if (hand_motor_zero_count_ >= hand_data_fail_threshold_) {
      RaiseHandFailure(FailureType::kHandMotorDataFault,
          "hand_motor_all_zero (count=" +
          std::to_string(hand_motor_zero_count_) + ")");
    }
  } else {
    hand_motor_zero_count_ = 0;
  }

  // Duplicate check
  if (!prev_hand_motor_positions_.empty() &&
      prev_hand_motor_positions_.size() == hand_motor_positions_.size()) {
    const bool duplicate = std::equal(
        hand_motor_positions_.begin(), hand_motor_positions_.end(),
        prev_hand_motor_positions_.begin(),
        [](double a, double b) { return std::abs(a - b) < 1e-12; });

    if (duplicate) {
      ++hand_motor_dup_count_;
      if (hand_motor_dup_count_ >= hand_data_fail_threshold_) {
        RaiseHandFailure(FailureType::kHandMotorDataFault,
            "hand_motor_duplicate (count=" +
            std::to_string(hand_motor_dup_count_) + ")");
      }
    } else {
      hand_motor_dup_count_ = 0;
    }
  }

  prev_hand_motor_positions_ = hand_motor_positions_;
}

void Ur5eHandStatusMonitor::CheckHandSensorData()
{
  std::lock_guard lock(hand_data_mutex_);
  if (hand_sensor_values_.empty()) return;

  // All-zero check
  const bool all_zero = std::all_of(hand_sensor_values_.begin(),
      hand_sensor_values_.end(),
      [](float v) { return std::abs(v) < 1e-9f; });

  if (all_zero) {
    ++hand_sensor_zero_count_;
    if (hand_sensor_zero_count_ >= hand_data_fail_threshold_) {
      RaiseHandFailure(FailureType::kHandSensorDataFault,
          "hand_sensor_all_zero (count=" +
          std::to_string(hand_sensor_zero_count_) + ")");
    }
  } else {
    hand_sensor_zero_count_ = 0;
  }

  // Duplicate check
  if (!prev_hand_sensor_values_.empty() &&
      prev_hand_sensor_values_.size() == hand_sensor_values_.size()) {
    const bool duplicate = std::equal(
        hand_sensor_values_.begin(), hand_sensor_values_.end(),
        prev_hand_sensor_values_.begin(),
        [](float a, float b) { return std::abs(a - b) < 1e-9f; });

    if (duplicate) {
      ++hand_sensor_dup_count_;
      if (hand_sensor_dup_count_ >= hand_data_fail_threshold_) {
        RaiseHandFailure(FailureType::kHandSensorDataFault,
            "hand_sensor_duplicate (count=" +
            std::to_string(hand_sensor_dup_count_) + ")");
      }
    } else {
      hand_sensor_dup_count_ = 0;
    }
  }

  prev_hand_sensor_values_ = hand_sensor_values_;
}

void Ur5eHandStatusMonitor::CheckHandRate()
{
  const auto now = std::chrono::steady_clock::now();
  const double elapsed_sec = std::chrono::duration<double>(
      now - hand_rate_window_start_).count();

  if (elapsed_sec < 1.0) return;  // Compute rate every 1 second

  const double rate = static_cast<double>(hand_rate_window_count_) / elapsed_sec;
  hand_stats_.current_rate_hz = rate;
  hand_stats_.total_count += hand_rate_window_count_;

  // Reset window
  hand_rate_window_start_ = now;
  hand_rate_window_count_ = 0;

  if (rate < hand_min_rate_hz_) {
    ++hand_rate_fail_count_;
    if (hand_rate_fail_count_ >= hand_rate_fail_threshold_) {
      RaiseHandFailure(FailureType::kHandRateLow,
          "hand_rate_low (rate=" + std::to_string(rate) +
          " Hz, min=" + std::to_string(hand_min_rate_hz_) + " Hz)");
    }
    if (on_warning_cb_ && hand_rate_fail_count_ == 1) {
      on_warning_cb_(WarningType::kHandRateDegraded,
          "hand rate degraded: " + std::to_string(rate) + " Hz");
    }
  } else {
    hand_rate_fail_count_ = 0;
  }
}

// ── Hand failure handling ───────────────────────────────────────────────────

void Ur5eHandStatusMonitor::RaiseHandFailure(
    FailureType type, const std::string& description)
{
  // Idempotent — only first failure takes effect
  auto expected = FailureType::kNone;
  if (!hand_failure_type_.compare_exchange_strong(
          expected, type,
          std::memory_order_acq_rel, std::memory_order_relaxed)) {
    return;
  }

  RCLCPP_ERROR(node_->get_logger(),
      "[HandStatusMonitor] HAND FAILURE: %s", description.c_str());

  if (on_failure_cb_) {
    FailureContext ctx;
    ctx.type = type;
    ctx.timestamp_ns = std::chrono::steady_clock::now().time_since_epoch().count();
    ctx.description = description;
    on_failure_cb_(type, ctx);
  }
}

}  // namespace rtc
