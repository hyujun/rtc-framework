#ifndef INTEGRATED_BRINGUP_BACKENDS_UR_DRIVER_NATIVE_BACKEND_H_
#define INTEGRATED_BRINGUP_BACKENDS_UR_DRIVER_NATIVE_BACKEND_H_

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"

#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <atomic>
#include <chrono>
#include <vector>

namespace rtc {

// ── UrDriverNativeBackend ────────────────────────────────────────────────────
//
// UR5e robot-side adapter. The Universal_Robots_ROS2_Driver exposes:
//   - State: sensor_msgs/JointState on `/joint_states` (RELIABLE).
//   - Command: std_msgs/Float64MultiArray on
//     `/forward_position_controller/commands` (name-less, fixed ordering).
//
// The Float64MultiArray ordering must match `joint_command_names` (from YAML),
// which is the ros2_control resource order. Controllers emit values in the
// device's `joint_state_names` order; the backend reorders at publish time.
class UrDriverNativeBackend : public DeviceBackend {
 public:
  UrDriverNativeBackend() = default;
  ~UrDriverNativeBackend() override = default;

  UrDriverNativeBackend(const UrDriverNativeBackend&) = delete;
  UrDriverNativeBackend& operator=(const UrDriverNativeBackend&) = delete;

  void Configure(rclcpp_lifecycle::LifecycleNode* node, const DeviceBackendConfig& config) override;
  void Activate() override;
  void Deactivate() override;

  [[nodiscard]] bool ReadState(DeviceStateCache& cache) noexcept override;
  void WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                    CommandType command_type) noexcept override;

  [[nodiscard]] std::chrono::steady_clock::time_point LastStateStamp() const noexcept override {
    const auto ns = last_state_ns_.load(std::memory_order_acquire);
    return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ns));
  }

 private:
  void OnJointState(sensor_msgs::msg::JointState::SharedPtr msg);

  DeviceBackendConfig config_{};

  // Lazy state reorder from incoming `msg->name` order to device-config order.
  std::vector<int> state_reorder_;
  std::atomic<bool> state_reorder_built_{false};

  // Command-side reorder: cmd_reorder_[output_idx] = input_idx (gc.commands
  // is in device joint_state_names order; output is in joint_command_names
  // order).
  std::vector<int> cmd_reorder_;

  SeqLock<DeviceStateCache> state_cache_{};
  std::atomic<int64_t> last_state_ns_{0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;

  std_msgs::msg::Float64MultiArray cmd_msg_{};
};

}  // namespace rtc

#endif  // INTEGRATED_BRINGUP_BACKENDS_UR_DRIVER_NATIVE_BACKEND_H_
