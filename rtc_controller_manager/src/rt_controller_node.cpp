// ── Core lifecycle: constructor, destructor, callback groups, timers,
//    lifecycle callbacks (on_configure / activate / deactivate / cleanup /
//    shutdown / error) ─────────────────────────────────────────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <rtc_base/logging/session_dir.hpp>
#include <rtc_base/threading/thread_utils.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <sys/eventfd.h>
#include <unistd.h>  // close

using namespace std::chrono_literals;
namespace urtc = rtc;

// ── Controller registry ──────────────────────────────────────────────────────
//
// To add a new controller:
//   1. Implement RTControllerInterface in your package
//   2. Create a config/controllers/<subdir>/<key>.yaml
//   3. Use RTC_REGISTER_CONTROLLER() macro in a .cpp file
//   4. Link the final executable against your library
//
// See rtc_controllers/src/controller_registration.cpp for built-in examples.
// ─────────────────────────────────────────────────────────────────────────────

// ── Constructor / destructor ──────────────────────────────────────────────────
//
// Lifecycle design: constructor is intentionally minimal.
// All resource allocation happens in on_configure / on_activate.
RtControllerNode::RtControllerNode()
: LifecycleNode("rt_controller",
       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  logger_(nullptr)
{
}

RtControllerNode::~RtControllerNode()
{
  // Safety net — idempotent cleanup in case lifecycle callbacks were not
  // invoked (e.g. SIGTERM without graceful shutdown).
  StopRtLoop();
  StopPublishLoop();

  if (publish_eventfd_ >= 0) {
    close(publish_eventfd_);
    publish_eventfd_ = -1;
  }

  if (logger_) {
    logger_->DrainBuffer(log_buffer_);
    logger_->Flush();
  }
}

// ── Session directory helpers ─────────────────────────────────────────────────
std::filesystem::path RtControllerNode::ResolveAndSetupSessionDir()
{
  return urtc::ResolveSessionDir();
}

// ── CallbackGroup creation ────────────────────────────────────────────────────
void RtControllerNode::CreateCallbackGroups()
{
  cb_group_sensor_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_log_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_aux_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

void RtControllerNode::CreateTimers()
{
  // Drain the SPSC log ring buffer from the log thread (Core 4).
  // File I/O stays entirely out of the 500 Hz RT thread.
  static constexpr auto kLogDrainPeriod = 10ms;
  drain_timer_ =
    create_wall_timer(kLogDrainPeriod, [this]() {DrainLog();}, cb_group_log_);
}

// ── Lifecycle callbacks ──────────────────────────────────────────────────────
//
// Tier 1 (on_configure): callback groups, parameters, controllers, publishers,
//   subscribers, timers, eventfd, pre-allocated messages.
// Tier 2 (on_activate): RT loop + publish offload thread start.

RtControllerNode::CallbackReturn
RtControllerNode::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring RtControllerNode...");

  CreateCallbackGroups();
  DeclareAndLoadParameters();
  CreateSubscriptions();
  CreatePublishers();
  ExposeTopicParameters();
  CreateTimers();

  // eventfd for RT→publish thread wakeup (non-blocking to avoid RT stalls)
  publish_eventfd_ = eventfd(0, EFD_NONBLOCK);
  if (publish_eventfd_ < 0) {
    RCLCPP_WARN(get_logger(),
        "eventfd() failed: publish thread will use polling fallback");
  }

  // Publish initial active controller name (transient_local so late
  // subscribers receive it). Safety publishers are non-lifecycle, so they
  // can publish regardless of state.
  {
    std_msgs::msg::String ctrl_name_msg;
    ctrl_name_msg.data = std::string(
      controllers_[static_cast<std::size_t>(
          active_controller_idx_.load(std::memory_order_acquire))]->Name());
    active_ctrl_name_pub_->publish(ctrl_name_msg);
  }

  RCLCPP_INFO(get_logger(), "RtControllerNode configured — %.0f Hz, E-STOP: %s",
              control_rate_, enable_estop_ ? "ON" : "OFF");

  return CallbackReturn::SUCCESS;
}

RtControllerNode::CallbackReturn
RtControllerNode::on_activate(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_activate(state);  // activates LifecyclePublishers

  const auto cfgs = urtc::SelectThreadConfigs();
  StartRtLoop(cfgs.rt_control);
  StartPublishLoop(cfgs.publish);

  RCLCPP_INFO(get_logger(),
      "RtControllerNode active — RT loop + publish offload started");

  return CallbackReturn::SUCCESS;
}

RtControllerNode::CallbackReturn
RtControllerNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Deactivating RtControllerNode...");

  StopRtLoop();
  StopPublishLoop();

  ClearGlobalEstop();

  // Reset initialization state for clean re-activate
  init_complete_ = false;
  init_wait_ticks_ = 0;
  state_received_.store(false, std::memory_order_release);
  target_received_.store(false, std::memory_order_release);
  loop_count_ = 0;
  overrun_count_.store(0, std::memory_order_relaxed);
  compute_overrun_count_.store(0, std::memory_order_relaxed);
  skip_count_.store(0, std::memory_order_relaxed);
  consecutive_overruns_.store(0, std::memory_order_relaxed);

  LifecycleNode::on_deactivate(state);  // deactivates LifecyclePublishers

  RCLCPP_INFO(get_logger(), "RtControllerNode deactivated");
  return CallbackReturn::SUCCESS;
}

RtControllerNode::CallbackReturn
RtControllerNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up RtControllerNode...");

  // Reverse order of on_configure:

  // 7. eventfd
  if (publish_eventfd_ >= 0) {
    close(publish_eventfd_);
    publish_eventfd_ = -1;
  }

  // 6. timers
  drain_timer_.reset();

  // 5. parameter callback
  param_callback_handle_.reset();

  // 4. publishers (all maps + fixed)
  topic_publishers_.clear();
  joint_command_publishers_.clear();
  gui_position_publishers_.clear();
  robot_target_publishers_.clear();
  device_state_log_publishers_.clear();
  device_sensor_log_publishers_.clear();
  grasp_state_publishers_.clear();
  tof_snapshot_publishers_.clear();
  digital_twin_publishers_.clear();
  slot_to_dt_topic_.clear();
  estop_pub_.reset();
  active_ctrl_name_pub_.reset();
  current_gains_pub_.reset();

  // 3. subscribers
  topic_subscriptions_.clear();
  controller_selector_sub_.reset();
  controller_gains_sub_.reset();
  request_gains_sub_.reset();

  // 2. parameters / controllers / logger
  if (logger_) {
    logger_->DrainBuffer(log_buffer_);
    logger_->Flush();
  }
  controllers_.clear();
  controller_topic_configs_.clear();
  controller_name_to_idx_.clear();
  controller_slot_mappings_.clear();
  device_name_configs_.clear();
  slot_to_group_name_.clear();
  active_groups_.clear();
  group_slot_map_.clear();
  device_timeouts_.clear();
  logger_.reset();

  // 1. callback groups — kept alive (executor may still reference them)

  RCLCPP_INFO(get_logger(), "RtControllerNode cleaned up");
  return CallbackReturn::SUCCESS;
}

RtControllerNode::CallbackReturn
RtControllerNode::on_shutdown(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Shutting down RtControllerNode...");

  // on_shutdown can be called from any primary state
  if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(state);
  }
  if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
      state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_cleanup(state);
  }

  return CallbackReturn::SUCCESS;
}

RtControllerNode::CallbackReturn
RtControllerNode::on_error(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_ERROR(get_logger(), "RtControllerNode error — attempting recovery");

  TriggerGlobalEstop("lifecycle_error");
  StopRtLoop();
  StopPublishLoop();

  // Full cleanup for recovery to Unconfigured state
  if (publish_eventfd_ >= 0) {
    close(publish_eventfd_);
    publish_eventfd_ = -1;
  }
  drain_timer_.reset();
  param_callback_handle_.reset();
  topic_publishers_.clear();
  joint_command_publishers_.clear();
  gui_position_publishers_.clear();
  robot_target_publishers_.clear();
  device_state_log_publishers_.clear();
  device_sensor_log_publishers_.clear();
  grasp_state_publishers_.clear();
  tof_snapshot_publishers_.clear();
  digital_twin_publishers_.clear();
  slot_to_dt_topic_.clear();
  estop_pub_.reset();
  active_ctrl_name_pub_.reset();
  current_gains_pub_.reset();
  topic_subscriptions_.clear();
  controller_selector_sub_.reset();
  controller_gains_sub_.reset();
  request_gains_sub_.reset();
  if (logger_) {
    logger_->DrainBuffer(log_buffer_);
    logger_->Flush();
  }
  controllers_.clear();
  controller_topic_configs_.clear();
  controller_name_to_idx_.clear();
  controller_slot_mappings_.clear();
  device_name_configs_.clear();
  slot_to_group_name_.clear();
  active_groups_.clear();
  group_slot_map_.clear();
  device_timeouts_.clear();
  logger_.reset();

  // SUCCESS → recovers to Unconfigured (can be reconfigured)
  return CallbackReturn::SUCCESS;
}
