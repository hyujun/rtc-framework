#ifndef INTEGRATED_BRINGUP_BACKENDS_UR_DRIVER_NATIVE_BACKEND_H_
#define INTEGRATED_BRINGUP_BACKENDS_UR_DRIVER_NATIVE_BACKEND_H_

#include "integrated_bringup/backends/joint_state_reorder.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"

#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <atomic>
#include <chrono>

namespace rtc {

// ── UrDriverNativeBackend ────────────────────────────────────────────────────
//
// UR5e robot-side adapter. The Universal_Robots_ROS2_Driver exposes:
//   - State: sensor_msgs/JointState on `/joint_states` (RELIABLE).
//   - Command: std_msgs/Float64MultiArray on
//     `/forward_position_controller/commands` (name-less, fixed ordering).
//
// The Float64MultiArray ordering must match `joint_command_names` (from YAML),
// which is the ros2_control resource order. `slot.commands` is already in
// `joint_command_names` order, so WriteCommand is a direct copy.
class UrDriverNativeBackend : public DeviceBackend {
 public:
  UrDriverNativeBackend() = default;
  ~UrDriverNativeBackend() override = default;

  UrDriverNativeBackend(const UrDriverNativeBackend&) = delete;
  UrDriverNativeBackend& operator=(const UrDriverNativeBackend&) = delete;

  void Configure(rclcpp_lifecycle::LifecycleNode* node, const DeviceBackendConfig& config,
                 rclcpp::CallbackGroup::SharedPtr state_cb_group) override;
  void Activate() override;
  void Deactivate() override;

  [[nodiscard]] bool ReadState(DeviceStateCache& cache) noexcept override;
  void WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                    CommandType command_type) noexcept override;

  // Safe output (issue #198 Phase 4). This lane is position-only on the wire,
  // so the safest thing it can emit without a controller output is the last
  // command it is known to have published — an explicit "stay put" rather
  // than the silence CM used to send, which the receiver cannot distinguish
  // from a stalled publisher. There is no disable or torque-off to reach for
  // here; if the hardware ever gains one, it belongs in this function.
  void WriteSafeCommand() noexcept override;

  [[nodiscard]] std::chrono::steady_clock::time_point LastStateStamp() const noexcept override {
    const auto ns = last_state_ns_.load(std::memory_order_acquire);
    return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ns));
  }

 private:
  void OnJointState(sensor_msgs::msg::JointState::SharedPtr msg);

  DeviceBackendConfig config_{};

  // State reorder from incoming `msg->name` order to device-config order,
  // built once from the first named message. Fixed-capacity — the build runs
  // inside the rt_callback lane (RT callback rule, issue #156).
  JointStateReorder state_reorder_{};
  std::atomic<bool> state_reorder_built_{false};

  SeqLock<DeviceStateCache> state_cache_{};
  std::atomic<int64_t> last_state_ns_{0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;

  std_msgs::msg::Float64MultiArray cmd_msg_{};

  // Set once cmd_msg_ has been published at least once — before that there

  // is no known-good command to fall back on.

  bool cmd_published_{false};
};

}  // namespace rtc

#endif  // INTEGRATED_BRINGUP_BACKENDS_UR_DRIVER_NATIVE_BACKEND_H_
