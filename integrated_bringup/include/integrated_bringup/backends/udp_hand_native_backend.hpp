#ifndef INTEGRATED_BRINGUP_BACKENDS_UDP_HAND_NATIVE_BACKEND_H_
#define INTEGRATED_BRINGUP_BACKENDS_UDP_HAND_NATIVE_BACKEND_H_

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_base/types/types.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"
#include <rtc_msgs/msg/hand_sensor_state.hpp>
#include <rtc_msgs/msg/joint_command.hpp>

#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <atomic>
#include <chrono>
#include <optional>
#include <vector>

namespace rtc {

// ── UdpHandNativeBackend ─────────────────────────────────────────────────────
//
// assm_v1 hand robot-side adapter (talks to udp_hand_driver, not the UDP
// socket directly — udp_hand_driver owns the wire protocol).
//
// Three subscription lanes:
//   - state_topic  : sensor_msgs/JointState   (joint position/velocity/effort)
//   - motor_topic  : sensor_msgs/JointState   (motor-space lane, encoder values)
//   - sensor_topic : rtc_msgs/HandSensorState (packed primary+secondary sensor
//                    + ML inference output per fingertip)
//
// Single publish lane:
//   - command_topic: rtc_msgs/JointCommand    (joint position commands)
//
// Sensor packing layout (DeviceSensorLayout) is forwarded via
// SetSensorLayout() before Configure() — the layout originates from the
// device YAML's `sensor_layout` block and is resolved by CM.
class UdpHandNativeBackend : public DeviceBackend {
 public:
  UdpHandNativeBackend() = default;
  ~UdpHandNativeBackend() override = default;

  UdpHandNativeBackend(const UdpHandNativeBackend&) = delete;
  UdpHandNativeBackend& operator=(const UdpHandNativeBackend&) = delete;

  void Configure(rclcpp_lifecycle::LifecycleNode* node, const DeviceBackendConfig& config) override;
  void Activate() override;
  void Deactivate() override;

  [[nodiscard]] bool ReadState(DeviceStateCache& cache) noexcept override;
  void WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                    CommandType command_type) noexcept override;

  [[nodiscard]] bool HasMotorState() const noexcept override { return true; }

  [[nodiscard]] bool HasSensorState() const noexcept override { return true; }

  void ReadMotorState(DeviceStateCache& cache) noexcept override;
  void ReadSensorState(DeviceStateCache& cache) noexcept override;

  [[nodiscard]] std::chrono::steady_clock::time_point LastStateStamp() const noexcept override {
    const auto ns = last_state_ns_.load(std::memory_order_acquire);
    return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ns));
  }

  /// Forward the device sensor packing layout. Must be called before
  /// Configure() — without it the sensor lane drops messages.
  void SetSensorLayout(const DeviceSensorLayout& layout) noexcept override {
    sensor_layout_ = layout;
  }

 private:
  void OnJointState(sensor_msgs::msg::JointState::SharedPtr msg);
  void OnMotorState(sensor_msgs::msg::JointState::SharedPtr msg);
  void OnSensorState(rtc_msgs::msg::HandSensorState::SharedPtr msg);

  DeviceBackendConfig config_{};
  std::optional<DeviceSensorLayout> sensor_layout_;

  std::vector<int> state_reorder_;
  std::atomic<bool> state_reorder_built_{false};
  std::vector<int> cmd_reorder_;

  // Separate SeqLocks per lane keeps writers independent — joint, motor and
  // sensor callbacks each fire on the sensor executor; serializing through one
  // SeqLock would force a needless reader-side retry on every cross-lane
  // store.
  SeqLock<DeviceStateCache> joint_cache_{};
  SeqLock<DeviceStateCache> motor_cache_{};
  SeqLock<DeviceStateCache> sensor_cache_{};
  std::atomic<int64_t> last_state_ns_{0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_sub_;
  rclcpp::Subscription<rtc_msgs::msg::HandSensorState>::SharedPtr sensor_sub_;
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::JointCommand>::SharedPtr cmd_pub_;

  rtc_msgs::msg::JointCommand cmd_msg_{};
};

}  // namespace rtc

#endif  // INTEGRATED_BRINGUP_BACKENDS_UDP_HAND_NATIVE_BACKEND_H_
