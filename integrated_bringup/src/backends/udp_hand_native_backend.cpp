#include "integrated_bringup/backends/udp_hand_native_backend.hpp"

#include "rtc_controller_manager/device_backend_registry.hpp"
#include <rtc_base/tracing/trace_scope.hpp>

#include <rclcpp/qos.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace rtc {

void UdpHandNativeBackend::Configure(rclcpp_lifecycle::LifecycleNode* node,
                                     const DeviceBackendConfig& config,
                                     rclcpp::CallbackGroup::SharedPtr state_cb_group) {
  config_ = config;

  if (!config_.joint_command_names.empty()) {
    cmd_msg_.joint_names = config_.joint_command_names;
    cmd_msg_.values.assign(config_.joint_command_names.size(), 0.0);
  }
  cmd_msg_.command_type = "position";

  rclcpp::QoS qos{1};
  qos.best_effort();

  // All three state lanes (joint / motor / sensor) ride the rt_callback
  // callback group (FIFO 70) so the RT loop reads see fresh data on the
  // controller↔hardware boundary.
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = state_cb_group;

  if (!config_.state_topic.empty()) {
    state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        config_.state_topic, qos,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnJointState(std::move(msg)); },
        sub_opts);
  }
  if (!config_.motor_topic.empty()) {
    motor_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        config_.motor_topic, qos,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnMotorState(std::move(msg)); },
        sub_opts);
  }
  if (!config_.sensor_topic.empty()) {
    sensor_sub_ = node->create_subscription<rtc_msgs::msg::HandSensorState>(
        config_.sensor_topic, qos,
        [this](rtc_msgs::msg::HandSensorState::SharedPtr msg) { OnSensorState(std::move(msg)); },
        sub_opts);
  }
  if (!config_.command_topic.empty()) {
    cmd_pub_ = node->create_publisher<rtc_msgs::msg::JointCommand>(config_.command_topic, qos);
  }
}

void UdpHandNativeBackend::Activate() {
  if (cmd_pub_)
    cmd_pub_->on_activate();
}

void UdpHandNativeBackend::Deactivate() {
  if (cmd_pub_)
    cmd_pub_->on_deactivate();
}

void UdpHandNativeBackend::OnJointState(sensor_msgs::msg::JointState::SharedPtr msg) {
  // L2 under the rt_callback executor's ros2:callback_* (L1) — the joint-lane
  // decode + SeqLock write on the rt_callback lane.
  RTC_TRACE_SCOPE("UdpHandNativeBackend::OnJointState");
  if (msg->position.empty())
    return;

  // One-shot reorder build from the first message that carries names —
  // fixed-capacity, allocation-free (RT callback rule, issue #156). Reference
  // order = joint_command_names (the device-canonical order).
  if (!state_reorder_built_.load(std::memory_order_acquire) && !msg->name.empty()) {
    BuildJointStateReorder(msg->name, config_.joint_command_names, state_reorder_);
    state_reorder_built_.store(true, std::memory_order_release);
  }

  auto ds = joint_cache_.Load();
  WriteJointStateToCache(*msg, state_reorder_, ds);
  joint_cache_.Store(ds);

  last_state_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count(),
                       std::memory_order_release);

  NotifyStateReady();
}

void UdpHandNativeBackend::OnMotorState(sensor_msgs::msg::JointState::SharedPtr msg) {
  // L2 under the rt_callback executor's ros2:callback_* (L1) — the motor-lane
  // decode + SeqLock write on the rt_callback lane.
  RTC_TRACE_SCOPE("UdpHandNativeBackend::OnMotorState");
  if (msg->position.empty())
    return;

  auto ds = motor_cache_.Load();
  ds.num_motor_channels = static_cast<int>(msg->position.size());

  const std::size_t n =
      std::min(msg->position.size(), static_cast<std::size_t>(kMaxDeviceChannels));
  for (std::size_t i = 0; i < n; ++i)
    ds.motor_positions[i] = msg->position[i];
  const std::size_t nv =
      std::min(msg->velocity.size(), static_cast<std::size_t>(kMaxDeviceChannels));
  for (std::size_t i = 0; i < nv; ++i)
    ds.motor_velocities[i] = msg->velocity[i];
  const std::size_t ne = std::min(msg->effort.size(), static_cast<std::size_t>(kMaxDeviceChannels));
  for (std::size_t i = 0; i < ne; ++i)
    ds.motor_efforts[i] = msg->effort[i];

  motor_cache_.Store(ds);
}

void UdpHandNativeBackend::OnSensorState(rtc_msgs::msg::HandSensorState::SharedPtr msg) {
  // L2 under the rt_callback executor's ros2:callback_* (L1) — the sensor-lane
  // decode + SeqLock write on the rt_callback lane.
  RTC_TRACE_SCOPE("UdpHandNativeBackend::OnSensorState");
  if (!sensor_layout_.has_value())
    return;  // YAML did not configure the layout.

  const auto& sl = sensor_layout_.value();
  const int primary = sl.primary_count_per_group;
  const int secondary = sl.secondary_count_per_group;
  const int values_per_group = sl.values_per_group;
  const int infer_values_per_group = sl.inference_values_per_group;

  auto ds = sensor_cache_.Load();

  const int n_ft = static_cast<int>(msg->fingertips.size());
  for (int f = 0; f < n_ft && f < kMaxSensorGroups; ++f) {
    const auto& fs = msg->fingertips[static_cast<std::size_t>(f)];
    const int base = f * values_per_group;

    for (int b = 0; b < primary && b < static_cast<int>(fs.barometer.size()); ++b) {
      ds.sensor_data[static_cast<std::size_t>(base + b)] =
          static_cast<int32_t>(fs.barometer[static_cast<std::size_t>(b)]);
    }
    for (int t = 0; t < secondary && t < static_cast<int>(fs.tof.size()); ++t) {
      ds.sensor_data[static_cast<std::size_t>(base + primary + t)] =
          static_cast<int32_t>(fs.tof[static_cast<std::size_t>(t)]);
    }

    for (int b = 0; b < primary && b < static_cast<int>(fs.barometer_raw.size()); ++b) {
      ds.sensor_data_raw[static_cast<std::size_t>(base + b)] =
          static_cast<int32_t>(fs.barometer_raw[static_cast<std::size_t>(b)]);
    }
    for (int t = 0; t < secondary && t < static_cast<int>(fs.tof_raw.size()); ++t) {
      ds.sensor_data_raw[static_cast<std::size_t>(base + primary + t)] =
          static_cast<int32_t>(fs.tof_raw[static_cast<std::size_t>(t)]);
    }

    ds.inference_enable[static_cast<std::size_t>(f)] = fs.inference_enable;
    if (fs.inference_enable && infer_values_per_group >= 7) {
      const int ft_base = f * infer_values_per_group;
      ds.inference_data[static_cast<std::size_t>(ft_base)] = fs.contact_flag;
      for (int j = 0; j < 3; ++j) {
        const auto ju = static_cast<std::size_t>(j);
        ds.inference_data[static_cast<std::size_t>(ft_base + 1 + j)] = fs.f[ju];
        ds.inference_data[static_cast<std::size_t>(ft_base + 4 + j)] = fs.u[ju];
      }
    }
  }

  ds.num_sensor_channels = n_ft * values_per_group;
  ds.num_inference_groups = n_ft;
  sensor_cache_.Store(ds);
}

bool UdpHandNativeBackend::ReadState(DeviceStateCache& cache) noexcept {
  cache = joint_cache_.Load();
  return cache.valid;
}

void UdpHandNativeBackend::ReadMotorState(DeviceStateCache& cache) noexcept {
  const auto m = motor_cache_.Load();
  cache.num_motor_channels = m.num_motor_channels;
  cache.motor_positions = m.motor_positions;
  cache.motor_velocities = m.motor_velocities;
  cache.motor_efforts = m.motor_efforts;
}

void UdpHandNativeBackend::ReadSensorState(DeviceStateCache& cache) noexcept {
  const auto s = sensor_cache_.Load();
  cache.num_sensor_channels = s.num_sensor_channels;
  cache.sensor_data = s.sensor_data;
  cache.sensor_data_raw = s.sensor_data_raw;
  cache.inference_data = s.inference_data;
  cache.inference_enable = s.inference_enable;
  cache.num_inference_groups = s.num_inference_groups;
}

void UdpHandNativeBackend::WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                                        CommandType command_type) noexcept {
  if (!cmd_pub_)
    return;
  const int nc = slot.num_channels;
  if (nc <= 0)
    return;

  // The real LEAP hand firmware is position-only: udp_hand_driver ignores
  // command_type / feedforward / kp / kd and servos toward values[] as a joint
  // position target. We still label the mode honestly on the wire, but the
  // feedforward torque channel is intentionally NOT transmitted here — real
  // torque/current drive is a separate backend/firmware track (τ→I, mode bits).
  cmd_msg_.command_type = CommandTypeToString(command_type);

  // Stamp the JointCommand header from the RT-loop wall-clock capture (integer
  // div/mod only — RT-safe). slot.stamp_ns is CLOCK_REALTIME ns so the topic
  // shares the ROS wall time axis; 0 (unset) yields a zero stamp harmlessly.
  cmd_msg_.header.stamp.sec = static_cast<int32_t>(slot.stamp_ns / 1'000'000'000LL);
  cmd_msg_.header.stamp.nanosec = static_cast<uint32_t>(slot.stamp_ns % 1'000'000'000LL);

  // Direct copy: slot.commands is always in joint_command_names order (same
  // order the message labels carry) — wire-order differences are the
  // receiver's job (udp_hand_node reorders by joint_names).
  const std::size_t n = std::min(static_cast<std::size_t>(nc), cmd_msg_.values.size());
  for (std::size_t i = 0; i < n; ++i)
    cmd_msg_.values[i] = slot.commands[i];
  cmd_pub_->publish(cmd_msg_);
}

}  // namespace rtc

RTC_REGISTER_DEVICE_BACKEND(udp_hand_native, std::make_unique<rtc::UdpHandNativeBackend>())
