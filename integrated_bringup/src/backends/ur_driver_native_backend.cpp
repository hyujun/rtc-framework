#include "integrated_bringup/backends/ur_driver_native_backend.hpp"

#include "rtc_controller_manager/device_backend_registry.hpp"
#include <rtc_base/tracing/trace_scope.hpp>

#include <rclcpp/qos.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace rtc {

void UrDriverNativeBackend::Configure(rclcpp_lifecycle::LifecycleNode* node,
                                      const DeviceBackendConfig& config,
                                      rclcpp::CallbackGroup::SharedPtr state_cb_group) {
  config_ = config;

  // Command-side: ros2_control forward_position_controller expects a fixed-
  // size Float64MultiArray. Pre-size from joint_command_names (or fall back
  // to kMaxRobotDOF when names are unset).
  const std::size_t out_size = config_.joint_command_names.empty()
                                   ? static_cast<std::size_t>(kMaxRobotDOF)
                                   : config_.joint_command_names.size();
  cmd_msg_.data.assign(out_size, 0.0);

  rclcpp::QoS state_qos{1};
  state_qos.reliable();  // UR driver publishes /joint_states RELIABLE.
  rclcpp::QoS cmd_qos{1};
  cmd_qos.reliable();

  // Route /joint_states onto the rt_callback callback group (FIFO 70) so the
  // RT loop reads see fresh data on the controller↔hardware boundary.
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = state_cb_group;

  if (!config_.state_topic.empty()) {
    state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        config_.state_topic, state_qos,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnJointState(std::move(msg)); },
        sub_opts);
  }
  if (!config_.command_topic.empty()) {
    cmd_pub_ =
        node->create_publisher<std_msgs::msg::Float64MultiArray>(config_.command_topic, cmd_qos);
  }
}

void UrDriverNativeBackend::Activate() {
  if (cmd_pub_)
    cmd_pub_->on_activate();
}

void UrDriverNativeBackend::Deactivate() {
  if (cmd_pub_)
    cmd_pub_->on_deactivate();
}

void UrDriverNativeBackend::OnJointState(sensor_msgs::msg::JointState::SharedPtr msg) {
  // L2 under the rt_callback executor's ros2:callback_* (L1) — the joint-state
  // decode + SeqLock write on the rt_callback lane.
  RTC_TRACE_SCOPE("UrDriverNativeBackend::OnJointState");
  if (msg->position.empty())
    return;

  // One-shot reorder build from the first message that carries names —
  // fixed-capacity, allocation-free (RT callback rule, issue #156). Reference
  // order = joint_command_names (the device-canonical order).
  if (!state_reorder_built_.load(std::memory_order_acquire) && !msg->name.empty()) {
    BuildJointStateReorder(msg->name, config_.joint_command_names, state_reorder_);
    state_reorder_built_.store(true, std::memory_order_release);
  }

  auto ds = state_cache_.Load();
  WriteJointStateToCache(*msg, state_reorder_, ds);
  state_cache_.Store(ds);

  last_state_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count(),
                       std::memory_order_release);

  NotifyStateReady();
}

bool UrDriverNativeBackend::ReadState(DeviceStateCache& cache) noexcept {
  // L3 under CM::ReadDeviceState — the RT-tick SeqLock load (not the OnJointState
  // callback lane above, which is the non-RT write side).
  RTC_TRACE_SCOPE("UrDriverNativeBackend::ReadState");
  cache = state_cache_.Load();
  return cache.valid;
}

void UrDriverNativeBackend::WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                                         CommandType /*command_type*/) noexcept {
  // L3 under CM::WriteCommand — the RT-tick actuator publish.
  RTC_TRACE_SCOPE("UrDriverNativeBackend::WriteCommand");
  // UR's forward_position_controller is position-only — torque commands have
  // no destination on this backend. Caller is expected to not emit torque
  // when bound to this backend (validated at YAML time, not RT).
  if (!cmd_pub_)
    return;
  const int nc = slot.num_channels;
  if (nc <= 0)
    return;

  // Direct copy: slot.commands is always in joint_command_names order, which
  // is exactly the Float64MultiArray packing order this backend declares.
  const std::size_t n = std::min(static_cast<std::size_t>(nc), cmd_msg_.data.size());
  for (std::size_t i = 0; i < n; ++i)
    cmd_msg_.data[i] = slot.commands[i];
  cmd_pub_->publish(cmd_msg_);
  cmd_published_ = true;
}

void UrDriverNativeBackend::WriteSafeCommand() noexcept {
  // L3 under CM::WriteCommand — same RT tick, same publisher.
  RTC_TRACE_SCOPE("UrDriverNativeBackend::WriteSafeCommand");
  // Nothing published yet means there is no known-good command to fall back
  // on, and this backend has no disable to reach for instead. Inventing a
  // value here would be worse than the silence: 0.0 on a position lane is
  // "go to the origin", which is the failure #196 spent a whole phase
  // keeping off the wire.
  if (!cmd_pub_ || !cmd_published_) {
    return;
  }
  // Re-publish the last command known to have gone out. Physically this
  // leaves the device where the silent path left it — the difference is that
  // the intent is now explicit on the wire and the lane stays live.
  cmd_pub_->publish(cmd_msg_);
}

}  // namespace rtc

RTC_REGISTER_DEVICE_BACKEND(ur_driver_native, std::make_unique<rtc::UrDriverNativeBackend>())
