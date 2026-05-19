#include "integrated_bringup/backends/mujoco_native_backend.hpp"

#include "rtc_controller_manager/device_backend_registry.hpp"

#include <rclcpp/qos.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace rtc {

namespace {

// Same algorithm as BuildPublisherReorderMap in CM. Returns empty when no
// reorder is needed (both empty or already aligned).
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

void MujocoNativeBackend::SetNameConfig(std::vector<std::string> joint_state_names,
                                        std::vector<std::string> joint_command_names) {
  cmd_msg_.joint_names = joint_command_names;
  cmd_msg_.values.assign(joint_command_names.size(), 0.0);
  cmd_reorder_ = BuildReorderMap(joint_state_names, joint_command_names);
}

void MujocoNativeBackend::Configure(rclcpp_lifecycle::LifecycleNode* node,
                                    const DeviceBackendConfig& config,
                                    rclcpp::CallbackGroup::SharedPtr state_cb_group) {
  config_ = config;

  if (config_.command_topic.empty() && config_.state_topic.empty()) {
    return;  // Nothing to bind; degenerate config — caller logs.
  }

  // joint_command_names → command publisher layout (when provided).
  if (cmd_msg_.joint_names.empty() && !config_.joint_command_names.empty()) {
    cmd_msg_.joint_names = config_.joint_command_names;
    cmd_msg_.values.assign(config_.joint_command_names.size(), 0.0);
  }
  cmd_msg_.command_type = "position";

  rclcpp::QoS qos{1};
  qos.best_effort();

  // RT inbound boundary: route the state-lane sub onto the rt_inbound
  // callback group (FIFO 70) so RT loop reads see fresh data without
  // bouncing through the non-RT default executor.
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = state_cb_group;

  if (!config_.state_topic.empty()) {
    state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        config_.state_topic, qos,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnJointState(std::move(msg)); },
        sub_opts);
  }
  if (!config_.command_topic.empty()) {
    cmd_pub_ = node->create_publisher<rtc_msgs::msg::JointCommand>(config_.command_topic, qos);
  }
}

void MujocoNativeBackend::Activate() {
  if (cmd_pub_) {
    cmd_pub_->on_activate();
  }
}

void MujocoNativeBackend::Deactivate() {
  if (cmd_pub_) {
    cmd_pub_->on_deactivate();
  }
}

void MujocoNativeBackend::OnJointState(sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.empty())
    return;

  // Lazy reorder build from the first message that carries names. Matches CM's
  // DeviceJointStateCallback semantics.
  if (!state_reorder_built_.load(std::memory_order_acquire) && !msg->name.empty()) {
    const auto& ref_names = config_.joint_command_names;  // device-order reference
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

bool MujocoNativeBackend::ReadState(DeviceStateCache& cache) noexcept {
  cache = state_cache_.Load();
  return cache.valid;
}

void MujocoNativeBackend::WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                                       CommandType command_type) noexcept {
  if (!cmd_pub_)
    return;
  const int nc = slot.num_channels;
  if (nc <= 0)
    return;  // Skip until the controller has output.

  cmd_msg_.command_type = (command_type == CommandType::kTorque) ? "torque" : "position";

  const std::size_t n = std::min(static_cast<std::size_t>(nc), cmd_msg_.values.size());
  if (!cmd_reorder_.empty()) {
    for (std::size_t i = 0; i < n; ++i) {
      const int src = (i < cmd_reorder_.size()) ? cmd_reorder_[i] : -1;
      cmd_msg_.values[i] =
          (src >= 0 && src < nc) ? slot.commands[static_cast<std::size_t>(src)] : 0.0;
    }
  } else {
    for (std::size_t i = 0; i < n; ++i)
      cmd_msg_.values[i] = slot.commands[i];
  }
  cmd_pub_->publish(cmd_msg_);
}

}  // namespace rtc

RTC_REGISTER_DEVICE_BACKEND(mujoco_native, std::make_unique<rtc::MujocoNativeBackend>())
