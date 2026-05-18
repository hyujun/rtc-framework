#include "integrated_bringup/backends/ur_driver_native_backend.hpp"

#include "rtc_controller_manager/device_backend_registry.hpp"

#include <rclcpp/qos.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace rtc {

namespace {

std::vector<int> BuildReorderMap(const std::vector<std::string>& state_names,
                                 const std::vector<std::string>& cmd_names) {
  if (state_names.empty() || cmd_names.empty() || state_names == cmd_names) {
    return {};
  }
  std::vector<int> map(cmd_names.size(), -1);
  for (std::size_t ci = 0; ci < cmd_names.size(); ++ci) {
    for (std::size_t si = 0; si < state_names.size(); ++si) {
      if (cmd_names[ci] == state_names[si]) {
        map[ci] = static_cast<int>(si);
        break;
      }
    }
  }
  return map;
}

}  // namespace

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

  rclcpp::QoS state_qos{10};
  state_qos.reliable();  // UR driver publishes /joint_states RELIABLE.
  rclcpp::QoS cmd_qos{1};
  cmd_qos.reliable();

  // Route /joint_states onto the rt_inbound callback group (FIFO 70) so the
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
  if (msg->position.empty())
    return;

  // Lazy state reorder build (same algorithm as CM's BuildDeviceReorderMap).
  // Reference order = joint_command_names (the device-canonical order).
  if (!state_reorder_built_.load(std::memory_order_acquire) && !msg->name.empty()) {
    const auto& ref_names = config_.joint_command_names;
    if (!ref_names.empty()) {
      std::vector<int> map(msg->name.size(), -1);
      for (std::size_t msg_i = 0; msg_i < msg->name.size(); ++msg_i) {
        for (std::size_t ref_i = 0; ref_i < ref_names.size(); ++ref_i) {
          if (msg->name[msg_i] == ref_names[ref_i]) {
            map[msg_i] = static_cast<int>(ref_i);
            break;
          }
        }
      }
      state_reorder_ = std::move(map);

      // Also build the command-side reorder once we have both lists. For UR
      // the controller emits values in joint_state order; we re-pack to
      // joint_command order (often identical, hence map may be empty).
      cmd_reorder_ = BuildReorderMap(config_.joint_command_names, config_.joint_command_names);
    }
    state_reorder_built_.store(true, std::memory_order_release);
  }

  auto ds = state_cache_.Load();
  ds.num_channels = static_cast<int>(msg->position.size());

  if (!state_reorder_.empty()) {
    const std::size_t n = std::min(msg->position.size(), state_reorder_.size());
    for (std::size_t src = 0; src < n; ++src) {
      const int idx = state_reorder_[src];
      if (idx < 0 || idx >= kMaxDeviceChannels)
        continue;
      const auto uidx = static_cast<std::size_t>(idx);
      ds.positions[uidx] = msg->position[src];
      if (src < msg->velocity.size())
        ds.velocities[uidx] = msg->velocity[src];
      if (src < msg->effort.size())
        ds.efforts[uidx] = msg->effort[src];
    }
  } else {
    const std::size_t n =
        std::min(msg->position.size(), static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < n; ++i)
      ds.positions[i] = msg->position[i];
    const std::size_t nv =
        std::min(msg->velocity.size(), static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < nv; ++i)
      ds.velocities[i] = msg->velocity[i];
    const std::size_t ne =
        std::min(msg->effort.size(), static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < ne; ++i)
      ds.efforts[i] = msg->effort[i];
  }
  ds.valid = true;
  state_cache_.Store(ds);

  last_state_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count(),
                       std::memory_order_release);

  NotifyStateReady();
}

bool UrDriverNativeBackend::ReadState(DeviceStateCache& cache) noexcept {
  cache = state_cache_.Load();
  return cache.valid;
}

void UrDriverNativeBackend::WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                                         CommandType /*command_type*/) noexcept {
  // UR's forward_position_controller is position-only — torque commands have
  // no destination on this backend. Caller is expected to not emit torque
  // when bound to this backend (validated at YAML time, not RT).
  if (!cmd_pub_)
    return;
  const int nc = slot.num_channels;
  if (nc <= 0)
    return;

  const std::size_t n = std::min(static_cast<std::size_t>(nc), cmd_msg_.data.size());
  if (!cmd_reorder_.empty()) {
    for (std::size_t i = 0; i < n; ++i) {
      const int src = (i < cmd_reorder_.size()) ? cmd_reorder_[i] : -1;
      cmd_msg_.data[i] =
          (src >= 0 && src < nc) ? slot.commands[static_cast<std::size_t>(src)] : 0.0;
    }
  } else {
    for (std::size_t i = 0; i < n; ++i)
      cmd_msg_.data[i] = slot.commands[i];
  }
  cmd_pub_->publish(cmd_msg_);
}

}  // namespace rtc

RTC_REGISTER_DEVICE_BACKEND(ur_driver_native, std::make_unique<rtc::UrDriverNativeBackend>())
