#ifndef INTEGRATED_BRINGUP_BACKENDS_MUJOCO_NATIVE_BACKEND_H_
#define INTEGRATED_BRINGUP_BACKENDS_MUJOCO_NATIVE_BACKEND_H_

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"
#include <rtc_msgs/msg/joint_command.hpp>

#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <atomic>
#include <chrono>
#include <vector>

namespace rtc {

// ── MujocoNativeBackend ──────────────────────────────────────────────────────
//
// Sim-side adapter: both the state subscription and the command publication
// use sensor_msgs/JointState. Used by rtc_mujoco_sim (and any other simulator
// that mirrors that ABI).
//
// State path (HW → controller):
//   - Subscribes to `state_topic` (BEST_EFFORT, depth 1).
//   - Builds a name → device-order reorder map lazily on the first message
//     that carries names; rebuilds are not allowed (writer-side stability).
//
// Command path (controller → HW):
//   - Publishes JointCommand on `command_topic` (BEST_EFFORT, depth 1).
//   - Reorders from device order (`joint_state_names`) to command order
//     (`joint_command_names`) at build time; the RT path only copies values.
class MujocoNativeBackend : public DeviceBackend {
 public:
  MujocoNativeBackend() = default;
  ~MujocoNativeBackend() override = default;

  MujocoNativeBackend(const MujocoNativeBackend&) = delete;
  MujocoNativeBackend& operator=(const MujocoNativeBackend&) = delete;

  void Configure(rclcpp_lifecycle::LifecycleNode* node, const DeviceBackendConfig& config,
                 rclcpp::CallbackGroup::SharedPtr state_cb_group) override;
  void Activate() override;
  void Deactivate() override;

  [[nodiscard]] bool ReadState(DeviceStateCache& cache) noexcept override;
  void WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                    CommandType command_type) noexcept override;

  [[nodiscard]] std::chrono::steady_clock::time_point LastStateStamp() const noexcept override {
    const auto ns = last_state_ns_.load(std::memory_order_acquire);
    return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ns));
  }

  /// Used by tests + Phase 3 cutover to inspect/inject device naming. Empty
  /// vectors mean the YAML did not provide names — caller should handle that
  /// in Configure(). Must be called before Activate().
  void SetNameConfig(std::vector<std::string> joint_state_names,
                     std::vector<std::string> joint_command_names);

 private:
  void OnJointState(sensor_msgs::msg::JointState::SharedPtr msg);

  DeviceBackendConfig config_{};

  // Reorder maps (built at Configure or first-message time).
  // state_reorder_[msg_idx] = device_slot_idx (state path, lazy build).
  std::vector<int> state_reorder_;
  std::atomic<bool> state_reorder_built_{false};

  // cmd_reorder_[output_idx] = input_idx into GroupCommandSlot::commands.
  // Built deterministically at Configure() from name lists.
  std::vector<int> cmd_reorder_;

  // SeqLock holds the decoded state — sensor callback writes, RT reads via
  // ReadState(). Trivially copyable (POD).
  SeqLock<DeviceStateCache> state_cache_{};
  std::atomic<int64_t> last_state_ns_{0};

  // ROS handles
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::JointCommand>::SharedPtr cmd_pub_;

  // Pre-allocated JointCommand message (no per-tick allocation).
  rtc_msgs::msg::JointCommand cmd_msg_{};
};

}  // namespace rtc

#endif  // INTEGRATED_BRINGUP_BACKENDS_MUJOCO_NATIVE_BACKEND_H_
