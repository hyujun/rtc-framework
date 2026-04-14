// ── Device sensor and target subscription callbacks ──────────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"

namespace urtc = rtc;

// ── Subscription callbacks (unified per-device) ──────────────────────────────
void RtControllerNode::DeviceJointStateCallback(
    int device_slot, sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.empty()) return;

  // Build reorder map from the first real message that carries joint names.
  const auto uslot = static_cast<std::size_t>(device_slot);
  auto& reorder = device_reorder_maps_[uslot];
  if (!msg->name.empty() && !reorder.built_from_msg) {
    BuildDeviceReorderMap(device_slot, msg->name);
    reorder.built_from_msg = true;
  }

  auto ds = device_states_[uslot].Load();
  ds.num_channels = static_cast<int>(msg->position.size());

  if (reorder.built_from_msg) {
    for (std::size_t src = 0; src < msg->position.size() &&
         src < reorder.reorder.size(); ++src) {
      const int idx = reorder.reorder[src];
      if (idx >= 0 && idx < urtc::kMaxDeviceChannels) {
        const auto uidx = static_cast<std::size_t>(idx);
        ds.positions[uidx] = msg->position[src];
        if (src < msg->velocity.size()) ds.velocities[uidx] = msg->velocity[src];
        if (src < msg->effort.size())   ds.efforts[uidx]    = msg->effort[src];
      }
    }
  } else {
    for (std::size_t i = 0; i < msg->position.size() &&
         i < static_cast<std::size_t>(urtc::kMaxDeviceChannels); ++i) {
      ds.positions[i] = msg->position[i];
    }
    for (std::size_t i = 0; i < msg->velocity.size() &&
         i < static_cast<std::size_t>(urtc::kMaxDeviceChannels); ++i) {
      ds.velocities[i] = msg->velocity[i];
    }
    for (std::size_t i = 0; i < msg->effort.size() &&
         i < static_cast<std::size_t>(urtc::kMaxDeviceChannels); ++i) {
      ds.efforts[i] = msg->effort[i];
    }
  }
  ds.valid = true;
  device_states_[uslot].Store(ds);
  state_received_.store(true, std::memory_order_release);

  // Forward to digital twin (RELIABLE republish, config joint order).
  {
    auto dt_it = slot_to_dt_topic_.find(device_slot);
    if (dt_it != slot_to_dt_topic_.end()) {
      auto pub_it = digital_twin_publishers_.find(dt_it->second);
      if (pub_it != digital_twin_publishers_.end()) {
        auto& dte = pub_it->second;
        dte.msg.header = msg->header;
        const auto n = dte.msg.name.size();
        for (std::size_t i = 0; i < n; ++i) {
          dte.msg.position[i] = ds.positions[i];
          dte.msg.velocity[i] = ds.velocities[i];
          dte.msg.effort[i]   = ds.efforts[i];
        }
        dte.publisher->publish(dte.msg);
      }
    }
  }

  // Simulation sync: wake rt_loop immediately on fresh state
  if (use_sim_time_sync_) {
    state_fresh_.store(true, std::memory_order_release);
    state_cv_.notify_one();
  }
}

void RtControllerNode::DeviceMotorStateCallback(
    int device_slot, sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.empty()) return;

  const auto uslot = static_cast<std::size_t>(device_slot);
  auto ds = device_states_[uslot].Load();
  ds.num_motor_channels = static_cast<int>(msg->position.size());

  for (std::size_t i = 0; i < msg->position.size() &&
       i < static_cast<std::size_t>(urtc::kMaxDeviceChannels); ++i) {
    ds.motor_positions[i] = msg->position[i];
  }
  for (std::size_t i = 0; i < msg->velocity.size() &&
       i < static_cast<std::size_t>(urtc::kMaxDeviceChannels); ++i) {
    ds.motor_velocities[i] = msg->velocity[i];
  }
  for (std::size_t i = 0; i < msg->effort.size() &&
       i < static_cast<std::size_t>(urtc::kMaxDeviceChannels); ++i) {
    ds.motor_efforts[i] = msg->effort[i];
  }
  device_states_[uslot].Store(ds);
}

void RtControllerNode::DeviceTargetCallback(
    int device_slot, rtc_msgs::msg::RobotTarget::SharedPtr msg)
{
  // Select data source based on goal_type
  const double * data_ptr = nullptr;
  int data_size = 0;

  if (msg->goal_type == "task") {
    data_ptr = msg->task_target.data();
    data_size = static_cast<int>(msg->task_target.size());
  } else {
    // Default to joint target
    if (msg->joint_target.empty()) return;
    data_ptr = msg->joint_target.data();
    data_size = static_cast<int>(msg->joint_target.size());
  }

  // Reorder by joint_names if provided (same pattern as BuildDeviceReorderMap)
  std::array<double, urtc::kMaxDeviceChannels> reordered{};
  const double * ordered_ptr = data_ptr;
  int ordered_size = data_size;

  if (msg->goal_type != "task" && !msg->joint_names.empty()) {
    const auto slot_idx = static_cast<std::size_t>(device_slot);
    if (slot_idx < slot_to_group_name_.size()) {
      const auto& group_name = slot_to_group_name_[slot_idx];
      auto it = device_name_configs_.find(group_name);
      if (it != device_name_configs_.end()) {
        const auto& ref_names = it->second.joint_state_names;
        if (!ref_names.empty()) {
          for (std::size_t msg_i = 0; msg_i < msg->joint_names.size() &&
               msg_i < msg->joint_target.size(); ++msg_i) {
            for (std::size_t ref_i = 0; ref_i < ref_names.size(); ++ref_i) {
              if (msg->joint_names[msg_i] == ref_names[ref_i]) {
                reordered[ref_i] = msg->joint_target[msg_i];
                break;
              }
            }
          }
          ordered_ptr = reordered.data();
          ordered_size = std::min(static_cast<int>(ref_names.size()),
                                  urtc::kMaxDeviceChannels);
        }
      }
    }
  }

  {
    std::lock_guard lock(target_mutex_);
    const int n = std::min(ordered_size, urtc::kMaxDeviceChannels);
    for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
      device_targets_[static_cast<std::size_t>(device_slot)][i] = ordered_ptr[i];
    }
  }
  target_received_.store(true, std::memory_order_release);
  const int idx = active_controller_idx_.load(std::memory_order_acquire);
  controllers_[static_cast<std::size_t>(idx)]->SetDeviceTarget(device_slot,
      std::span<const double>(ordered_ptr,
                              static_cast<std::size_t>(ordered_size)));
}

// ── Device sensor callback (unified per-device) ──────────────────────────────
void RtControllerNode::DeviceSensorCallback(
    int device_slot, std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  const auto uslot = static_cast<std::size_t>(device_slot);
  auto ds = device_states_[uslot].Load();
  for (std::size_t i = 0; i < msg->data.size() &&
       i < static_cast<std::size_t>(urtc::kMaxSensorChannels); ++i) {
    ds.sensor_data[i] = static_cast<int32_t>(msg->data[i]);
  }
  ds.num_sensor_channels = static_cast<int>(
      std::min(msg->data.size(), static_cast<std::size_t>(urtc::kMaxSensorChannels)));
  device_states_[uslot].Store(ds);
}

void RtControllerNode::HandSensorStateCallback(
    int device_slot, rtc_msgs::msg::HandSensorState::SharedPtr msg)
{
  const auto uslot = static_cast<std::size_t>(device_slot);
  auto ds = device_states_[uslot].Load();

  const int n_ft = static_cast<int>(msg->fingertips.size());
  for (int f = 0; f < n_ft && f < urtc::kMaxFingertips; ++f) {
    const auto& fs = msg->fingertips[static_cast<std::size_t>(f)];
    const int base = f * urtc::kSensorValuesPerFingertip;

    // Filtered sensor data
    for (int b = 0; b < urtc::kBarometerCount; ++b) {
      ds.sensor_data[static_cast<std::size_t>(base + b)] =
          static_cast<int32_t>(fs.barometer[static_cast<std::size_t>(b)]);
    }
    for (int t = 0; t < urtc::kTofCount; ++t) {
      ds.sensor_data[static_cast<std::size_t>(base + urtc::kBarometerCount + t)] =
          static_cast<int32_t>(fs.tof[static_cast<std::size_t>(t)]);
    }

    // Raw sensor data
    for (int b = 0; b < urtc::kBarometerCount; ++b) {
      ds.sensor_data_raw[static_cast<std::size_t>(base + b)] =
          static_cast<int32_t>(fs.barometer_raw[static_cast<std::size_t>(b)]);
    }
    for (int t = 0; t < urtc::kTofCount; ++t) {
      ds.sensor_data_raw[static_cast<std::size_t>(base + urtc::kBarometerCount + t)] =
          static_cast<int32_t>(fs.tof_raw[static_cast<std::size_t>(t)]);
    }

    // Inference data
    ds.inference_enable[static_cast<std::size_t>(f)] = fs.inference_enable;
    if (fs.inference_enable) {
      const int ft_base = f * urtc::kFTValuesPerFingertip;
      ds.inference_data[static_cast<std::size_t>(ft_base)]     = fs.contact_flag;
      for (int j = 0; j < 3; ++j) {
        const auto ju = static_cast<std::size_t>(j);
        ds.inference_data[static_cast<std::size_t>(ft_base + 1 + j)] = fs.f[ju];
        ds.inference_data[static_cast<std::size_t>(ft_base + 4 + j)] = fs.u[ju];
      }
    }
  }

  ds.num_sensor_channels = n_ft * urtc::kSensorValuesPerFingertip;
  ds.num_inference_fingertips = n_ft;
  device_states_[uslot].Store(ds);
}
