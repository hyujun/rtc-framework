// ── Core lifecycle: constructor, destructor, callback groups, timers,
//    lifecycle callbacks (on_configure / activate / deactivate / cleanup /
//    shutdown / error)
//    ─────────────────────────────────────────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <rtc_base/logging/session_dir.hpp>
#include <rtc_base/threading/thread_utils.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <sys/eventfd.h>
#include <unistd.h> // close

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

// ── Constructor / destructor
// ──────────────────────────────────────────────────
//
// Lifecycle design: constructor is intentionally minimal.
// All resource allocation happens in on_configure / on_activate.
RtControllerNode::RtControllerNode(const std::string &node_name)
    : LifecycleNode(
          node_name,
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
              true)),
      logger_(nullptr) {}

RtControllerNode::~RtControllerNode() {
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

// ── Session directory helpers
// ─────────────────────────────────────────────────
std::filesystem::path RtControllerNode::ResolveAndSetupSessionDir() {
  return urtc::ResolveSessionDir();
}

// ── CallbackGroup creation
// ────────────────────────────────────────────────────
void RtControllerNode::CreateCallbackGroups() {
  cb_group_sensor_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_log_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_aux_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

void RtControllerNode::CreateTimers() {
  // Drain the SPSC log ring buffer from the log thread (Core 4).
  // File I/O stays entirely out of the 500 Hz RT thread.
  static constexpr auto kLogDrainPeriod = 10ms;
  drain_timer_ = create_wall_timer(
      kLogDrainPeriod, [this]() { DrainLog(); }, cb_group_log_);
}

// ── Lifecycle callbacks ──────────────────────────────────────────────────────
//
// Tier 1 (on_configure): callback groups, parameters, controllers, publishers,
//   subscribers, timers, eventfd, pre-allocated messages.
// Tier 2 (on_activate): RT loop + publish offload thread start.

RtControllerNode::CallbackReturn
RtControllerNode::on_configure(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Configuring RtControllerNode...");

  CreateCallbackGroups();
  DeclareAndLoadParameters();
  CreateSubscriptions();
  CreatePublishers();
  CreateServices();
  ExposeTopicParameters();
  CreateTimers();

  // eventfd for RT→publish thread wakeup (non-blocking to avoid RT stalls)
  publish_eventfd_ = eventfd(0, EFD_NONBLOCK);
  if (publish_eventfd_ < 0) {
    RCLCPP_WARN(get_logger(),
                "eventfd() failed: publish thread will use polling fallback");
  }

  // Per-controller lifecycle state, parallel to controllers_. All start
  // Inactive; CM's on_activate flips the initial controller to Active via
  // ActivateController(). std::vector<std::atomic<int>> is non-copyable, so
  // we resize-from-default rather than fill-with-value.
  controller_states_ = std::vector<std::atomic<int>>(controllers_.size());
  for (auto &s : controller_states_) {
    s.store(0, std::memory_order_relaxed); // 0 = Inactive
  }

  // Publish initial active controller name (transient_local so late
  // subscribers receive it). Safety publishers are non-lifecycle, so they
  // can publish regardless of state.
  {
    std_msgs::msg::String ctrl_name_msg;
    ctrl_name_msg.data = std::string(
        controllers_[static_cast<std::size_t>(active_controller_idx_.load(
                         std::memory_order_acquire))]
            ->Name());
    active_ctrl_name_pub_->publish(ctrl_name_msg);
  }

  RCLCPP_INFO(get_logger(), "RtControllerNode configured — %.0f Hz, E-STOP: %s",
              control_rate_, enable_estop_ ? "ON" : "OFF");

  return CallbackReturn::SUCCESS;
}

RtControllerNode::CallbackReturn
RtControllerNode::on_activate(const rclcpp_lifecycle::State &state) {
  LifecycleNode::on_activate(state); // activates CM-owned LifecyclePublishers

  // Activate ONLY the initial controller (single-active invariant, D-A1).
  // Snapshot is empty because the RT loop has not yet received any sensor
  // data; the loop's auto-hold path (rt_loop.cpp) takes over once state
  // arrives. Controller-owned LifecyclePublishers are toggled inside the
  // controller's on_activate (e.g. demo controllers' ActivateOwnedTopics).
  const int initial_idx =
      active_controller_idx_.load(std::memory_order_acquire);
  if (initial_idx < 0 || initial_idx >= static_cast<int>(controllers_.size())) {
    RCLCPP_ERROR(get_logger(),
                 "on_activate: invalid initial_controller_idx %d (size=%zu)",
                 initial_idx, controllers_.size());
    return CallbackReturn::FAILURE;
  }
  const rtc::ControllerState empty_snapshot{};
  const auto rc = ActivateController(static_cast<std::size_t>(initial_idx),
                                     state, empty_snapshot);
  if (rc != CallbackReturn::SUCCESS) {
    return rc;
  }

  const auto cfgs = urtc::SelectThreadConfigs();
  StartRtLoop(cfgs.rt_control);
  StartPublishLoop(cfgs.publish);

  RCLCPP_INFO(
      get_logger(),
      "RtControllerNode active — initial controller '%s', RT loop + "
      "publish offload started",
      controllers_[static_cast<std::size_t>(initial_idx)]->Name().data());

  return CallbackReturn::SUCCESS;
}

RtControllerNode::CallbackReturn
RtControllerNode::on_deactivate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating RtControllerNode...");

  StopRtLoop();
  StopPublishLoop();

  // Deactivate the active controller after the publish thread has stopped so
  // no one tries to publish via a newly-Inactive LifecyclePublisher. Other
  // controllers were already Inactive (single-active invariant).
  const int active_idx = active_controller_idx_.load(std::memory_order_acquire);
  if (active_idx >= 0 && active_idx < static_cast<int>(controllers_.size()) &&
      controller_states_[static_cast<std::size_t>(active_idx)].load(
          std::memory_order_acquire) == 1) {
    (void)DeactivateController(static_cast<std::size_t>(active_idx), state);
  }

  // Reset all controller states to Inactive so a subsequent on_activate
  // starts from a known-clean baseline.
  for (auto &s : controller_states_) {
    s.store(0, std::memory_order_release);
  }

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

  LifecycleNode::on_deactivate(state); // deactivates LifecyclePublishers

  RCLCPP_INFO(get_logger(), "RtControllerNode deactivated");
  return CallbackReturn::SUCCESS;
}

RtControllerNode::CallbackReturn
RtControllerNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
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
  // Drive each controller's lifecycle on_cleanup before destruction so any
  // sub/pub they created in on_configure is released deterministically.  The
  // injected LifecycleNode is reset inside the controller; we then drop our
  // parallel reference.
  {
    const rclcpp_lifecycle::State inactive_state(
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    for (auto &ctrl : controllers_) {
      if (ctrl) {
        (void)ctrl->on_cleanup(inactive_state);
      }
    }
  }
  controller_nodes_.clear();
  controllers_.clear();
  controller_states_.clear();
  controller_topic_configs_.clear();
  controller_name_to_idx_.clear();
  controller_types_.clear();
  controller_slot_mappings_.clear();
  device_name_configs_.clear();
  slot_to_group_name_.clear();
  active_groups_.clear();
  group_slot_map_.clear();
  device_timeouts_.clear();
  list_controllers_srv_.reset();
  switch_controller_srv_.reset();
  logger_.reset();

  // 1. callback groups — kept alive (executor may still reference them)

  RCLCPP_INFO(get_logger(), "RtControllerNode cleaned up");
  return CallbackReturn::SUCCESS;
}

RtControllerNode::CallbackReturn
RtControllerNode::on_shutdown(const rclcpp_lifecycle::State &state) {
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
RtControllerNode::on_error(const rclcpp_lifecycle::State & /*state*/) {
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
  controller_states_.clear();
  controller_topic_configs_.clear();
  controller_name_to_idx_.clear();
  controller_types_.clear();
  controller_slot_mappings_.clear();
  device_name_configs_.clear();
  slot_to_group_name_.clear();
  active_groups_.clear();
  group_slot_map_.clear();
  list_controllers_srv_.reset();
  switch_controller_srv_.reset();
  device_timeouts_.clear();
  logger_.reset();

  // SUCCESS → recovers to Unconfigured (can be reconfigured)
  return CallbackReturn::SUCCESS;
}

// ── Controller-level lifecycle helpers ───────────────────────────────────
//
// Aux-thread only. The single-active invariant (D-A1) is enforced by
// SwitchActiveController; ActivateController/DeactivateController do not
// validate it themselves so they can be reused by CM startup
// (initial-controller-only activation) and shutdown (active-only
// deactivation) without redundant checks.

rtc::ControllerState
RtControllerNode::BuildDeviceSnapshot(std::size_t ctrl_idx) const noexcept {
  rtc::ControllerState snapshot{};
  if (ctrl_idx >= controller_topic_configs_.size() ||
      !state_received_.load(std::memory_order_acquire)) {
    return snapshot; // empty — caller skips hold init
  }
  const auto &slots = controller_slot_mappings_[ctrl_idx];
  std::size_t di = 0;
  for ([[maybe_unused]] const auto &[gname, ggroup] :
       controller_topic_configs_[ctrl_idx].groups) {
    const auto slot = static_cast<std::size_t>(slots.slots[di]);
    auto &dev = snapshot.devices[di];
    const auto cache = device_states_[slot].Load();
    dev.num_channels = cache.num_channels;
    dev.positions = cache.positions;
    dev.velocities = cache.velocities;
    dev.efforts = cache.efforts;
    dev.valid = cache.valid;
    ++di;
  }
  snapshot.num_devices = static_cast<int>(di);
  snapshot.dt = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.002;
  return snapshot;
}

RtControllerNode::CallbackReturn
RtControllerNode::ActivateController(std::size_t ctrl_idx,
                                     const rclcpp_lifecycle::State &prev_state,
                                     const rtc::ControllerState &snapshot) {
  if (ctrl_idx >= controllers_.size() || !controllers_[ctrl_idx]) {
    return CallbackReturn::FAILURE;
  }
  const auto rc = controllers_[ctrl_idx]->on_activate(prev_state, snapshot);
  if (rc == CallbackReturn::SUCCESS) {
    controller_states_[ctrl_idx].store(1, std::memory_order_release);
  }
  return rc;
}

RtControllerNode::CallbackReturn RtControllerNode::DeactivateController(
    std::size_t ctrl_idx, const rclcpp_lifecycle::State &prev_state) {
  if (ctrl_idx >= controllers_.size() || !controllers_[ctrl_idx]) {
    return CallbackReturn::FAILURE;
  }
  const auto rc = controllers_[ctrl_idx]->on_deactivate(prev_state);
  controller_states_[ctrl_idx].store(0, std::memory_order_release);
  return rc;
}
