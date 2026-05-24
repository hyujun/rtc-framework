#ifndef INTEGRATED_BRINGUP_BACKENDS_MUJOCO_NATIVE_BACKEND_H_
#define INTEGRATED_BRINGUP_BACKENDS_MUJOCO_NATIVE_BACKEND_H_

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"
#include <rtc_msgs/msg/joint_command.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
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

  // Fingertip wrench lane (mujoco-only extension — fills inference_data slots
  // 1..3 with fx/fy/fz per fingertip, slots 0 / 4..6 zeroed). When the YAML
  // `devices.<group>.backend.fingertip_wrench_topics` list is empty
  // HasSensorState() stays true but ReadSensorState reports
  // num_inference_groups=0 and inference_enable all false.
  [[nodiscard]] bool HasSensorState() const noexcept override { return true; }
  void ReadSensorState(DeviceStateCache& cache) noexcept override;

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
  void OnWrench(int finger_idx, const geometry_msgs::msg::WrenchStamped& msg) noexcept;

  // Fingertip wrench mirror — sized to rtc::kMaxSensorGroups so a SensorMirror
  // fits the same per-group capacity as DeviceStateCache::inference_enable.
  // Stride 7 mirrors the legacy udp_hand layout (slot 0 contact_flag, 1..3
  // fx/fy/fz, 4..6 displacement); mujoco only publishes fx/fy/fz so dead
  // slots are zero-filled by ReadSensorState.
  static constexpr int kInferenceStride = 7;

  struct FingertipForceMirror {
    float fx{0.0F};
    float fy{0.0F};
    float fz{0.0F};
    bool received_at_least_once{false};
  };
  struct SensorMirror {
    std::array<FingertipForceMirror, kMaxSensorGroups> tips{};
    int num_tips{0};
  };
  static_assert(std::is_trivially_copyable_v<SensorMirror>,
                "SensorMirror must be trivially copyable for SeqLock");

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

  // Fingertip wrench SeqLock — non-RT executor writes (OnWrench), RT reads
  // (ReadSensorState). SPSC contract: every wrench callback MUST run on a
  // single-threaded callback group (MutuallyExclusive) so the load-modify-
  // store sequence in OnWrench is serialized across all fingertip indices.
  // Configure() WARN-logs if the cb_group is Reentrant. Per-tip seq counters
  // give RT-side miss accounting without touching the mirror.
  SeqLock<SensorMirror> sensor_mirror_{};
  std::array<std::atomic<uint64_t>, kMaxSensorGroups> wrench_seq_{};

  // RT-thread local state — only ReadSensorState touches these so no race.
  std::array<uint64_t, kMaxSensorGroups> rt_last_seen_seq_{};
  std::array<int, kMaxSensorGroups> rt_miss_count_{};
  int max_missed_ticks_{5};

  std::vector<rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr> wrench_subs_;

  // Diagnostics for the wrench lane (NaN/Inf drop throttle, empty-topics
  // INFO). Steady clock so RCLCPP_*_THROTTLE works without a node clock.
  rclcpp::Logger logger_{rclcpp::get_logger("mujoco_native_backend")};
  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

}  // namespace rtc

#endif  // INTEGRATED_BRINGUP_BACKENDS_MUJOCO_NATIVE_BACKEND_H_
