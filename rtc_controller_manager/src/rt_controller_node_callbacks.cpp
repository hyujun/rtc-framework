// ── Device target subscription callback ─────────────────────────────────────
//
// State / motor / sensor lanes are owned by DeviceBackend implementations
// (created in CreateDeviceBackends — see device_config.cpp). CM only binds
// kTarget itself, since it is controller-target lane (RobotTarget message)
// independent of the HW/sim adapter.
#include "rtc_controller_manager/rt_controller_node.hpp"

namespace urtc = rtc;

void RtControllerNode::DeviceTargetCallback(int device_slot,
                                            rtc_msgs::msg::RobotTarget::SharedPtr msg) {
  // Select data source based on goal_type
  const double* data_ptr = nullptr;
  int data_size = 0;

  if (msg->goal_type == "task") {
    data_ptr = msg->task_target.data();
    data_size = static_cast<int>(msg->task_target.size());
  } else {
    // Default to joint target
    if (msg->joint_target.empty())
      return;
    data_ptr = msg->joint_target.data();
    data_size = static_cast<int>(msg->joint_target.size());
  }

  // Reorder by joint_names if provided (same pattern as backend reorder).
  std::array<double, urtc::kMaxDeviceChannels> reordered{};
  const double* ordered_ptr = data_ptr;
  int ordered_size = data_size;

  if (msg->goal_type != "task" && !msg->joint_names.empty()) {
    const auto slot_idx = static_cast<std::size_t>(device_slot);
    if (slot_idx < slot_to_group_name_.size()) {
      const auto& group_name = slot_to_group_name_[slot_idx];
      auto it = device_name_configs_.find(group_name);
      if (it != device_name_configs_.end()) {
        const auto& ref_names = it->second.joint_state_names;
        if (!ref_names.empty()) {
          for (std::size_t msg_i = 0;
               msg_i < msg->joint_names.size() && msg_i < msg->joint_target.size(); ++msg_i) {
            for (std::size_t ref_i = 0; ref_i < ref_names.size(); ++ref_i) {
              if (msg->joint_names[msg_i] == ref_names[ref_i]) {
                reordered[ref_i] = msg->joint_target[msg_i];
                break;
              }
            }
          }
          ordered_ptr = reordered.data();
          ordered_size = std::min(static_cast<int>(ref_names.size()), urtc::kMaxDeviceChannels);
        }
      }
    }
  }

  // Forward to the active controller's SetDeviceTarget, which marshals the
  // payload onto its own SPSC queue (drained by the RT thread in Compute).
  // No CM-side mirror — target ownership lives with the controller now.
  const int clipped_size = std::min(ordered_size, urtc::kMaxDeviceChannels);
  const int idx = active_controller_idx_.load(std::memory_order_acquire);
  controllers_[static_cast<std::size_t>(idx)]->SetDeviceTarget(
      device_slot, std::span<const double>(ordered_ptr, static_cast<std::size_t>(clipped_size)));
}
