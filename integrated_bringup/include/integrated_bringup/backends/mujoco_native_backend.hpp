#ifndef INTEGRATED_BRINGUP_BACKENDS_MUJOCO_NATIVE_BACKEND_H_
#define INTEGRATED_BRINGUP_BACKENDS_MUJOCO_NATIVE_BACKEND_H_

#include "integrated_bringup/backends/floating_base_conventions.hpp"
#include "integrated_bringup/backends/joint_state_reorder.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"
#include <rtc_msgs/msg/joint_command.hpp>
#include <rtc_msgs/msg/sim_sensor_state.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
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
//   - Direct copy: `slot.commands` is already in `joint_command_names` order
//     (the message label order). Wire-order differences are absorbed by the
//     receiver, which reorders by `joint_names`.
//
// Floating-base lane (WD-16, #976) — optional, YAML-enabled:
//   - `devices.<group>.backend.base_state.sensor_topic` names the group's
//     SimSensorState topic (rtc_mujoco_sim `sensor_topic`); `.pos` / `.quat` /
//     `.linvel` / `.angvel` name the MJCF framepos / framequat / framelinvel /
//     frameangvel sensors of the base body, and `.imu_gyro` / `.imu_accel` are
//     parallel lists (index = IMU site) of gyro / accelerometer sensor names.
//     The names are the MJCF's own (ARCH-1: nothing here knows a robot).
//   - Empty `sensor_topic` → HasFloatingBase() false and the lane is not
//     claimed; the controller reads DeviceState's identity default.
//   - Conversion (quaternion order, twist frame) lives in ONE place:
//     floating_base_conventions.hpp. This backend only looks sensors up by
//     name, guards them, and hands the arrays over.
//   - Same shape as the wrench lane: SeqLock mirror written on the rt_callback
//     lane, read on the RT tick with a miss counter (`max_consecutive_missed_
//     ticks`, shared with the wrench lane) that clears `base_valid` while the
//     values stay in place.
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

  // The only backend in this tree that carries the mode through: WriteCommand
  // stamps JointCommand.command_type from the enum and forwards the
  // feedforward channel for kPdFeedforward, and mujoco_sim reads both. The
  // hardware backends silently reinterpret whatever they are handed, so they
  // keep the position-only default.
  [[nodiscard]] bool AcceptsCommandType(CommandType /*ct*/) const noexcept override { return true; }

  void ReadSensorState(DeviceStateCache& cache) noexcept override;

  // Floating-base lane — claimed only when `base_state.sensor_topic` is set
  // (see the class banner). CM derives DeviceCapability::kFloatingBase from
  // this at configure, so the answer must not change after Configure().
  [[nodiscard]] bool HasFloatingBase() const noexcept override { return base_lane_enabled_; }

  void ReadBaseState(DeviceStateCache& cache) noexcept override;

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
    // Steady-clock receipt time of the sample above.
    // Stamped by OnWrench on the same lane that stamps `last_state_ns_` for
    // the joint callback, and carried out through DeviceStateCache so a
    // consumer can apply its OWN staleness deadline instead of inheriting
    // `max_missed_ticks_`. 0 = this tip has never reported.
    int64_t recv_steady_ns{0};
  };

  struct SensorMirror {
    std::array<FingertipForceMirror, kMaxSensorGroups> tips{};
    int num_tips{0};
  };

  static_assert(std::is_trivially_copyable_v<SensorMirror>,
                "SensorMirror must be trivially copyable for SeqLock");

  DeviceBackendConfig config_{};

  // State reorder map (built once from the first named message).
  // state_reorder_.map[msg_idx] = device_slot_idx. Fixed-capacity — the build
  // runs inside the rt_callback lane (RT callback rule, issue #156).
  JointStateReorder state_reorder_{};
  std::atomic<bool> state_reorder_built_{false};

  // SeqLock holds the decoded state — sensor callback writes, RT reads via
  // ReadState(). Trivially copyable (POD).
  SeqLock<DeviceStateCache> state_cache_{};
  std::atomic<int64_t> last_state_ns_{0};

  // ROS handles
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::JointCommand>::SharedPtr cmd_pub_;

  // Pre-allocated JointCommand message (no per-tick allocation).
  rtc_msgs::msg::JointCommand cmd_msg_{};
  // Set once cmd_msg_ has been published at least once — before that there
  // is no known-good command to fall back on.
  bool cmd_published_{false};

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

  // ── Floating-base lane ─────────────────────────────────────────────────────
  void OnSimSensorState(rtc_msgs::msg::SimSensorState::ConstSharedPtr msg) noexcept;

  // The MJCF sensor names this group's base lane looks up, from YAML
  // `base_state.*`. Fixed after Configure; read on the rt_callback lane.
  struct BaseLaneNames {
    std::string pos;
    std::string quat;
    std::string linvel;
    std::string angvel;
    std::array<std::string, kMaxImuSites> imu_gyro{};
    std::array<std::string, kMaxImuSites> imu_accel{};
    int num_imu_sites{0};
  };

  // Latest accepted base sample, already in DeviceState's convention.
  // `recv_steady_ns` is stamped on the rt_callback lane for an ACCEPTED sample
  // only — the same rule as FingertipForceMirror::recv_steady_ns, for the
  // same reason (a dropped sample must not refresh the age of the one still
  // in the mirror).
  struct BaseMirror {
    FloatingBaseSample sample{};
    std::array<std::array<double, kImuChannelsPerSite>, kMaxImuSites> imu{};
    int num_imu_sites{0};
    int64_t recv_steady_ns{0};
    bool received_at_least_once{false};
  };

  static_assert(std::is_trivially_copyable_v<BaseMirror>,
                "BaseMirror must be trivially copyable for SeqLock");

  bool base_lane_enabled_{false};
  BaseLaneNames base_names_{};

  // SPSC like the wrench lane: OnSimSensorState (rt_callback, MutuallyExclusive
  // group) stores the mirror THEN bumps the counter; ReadBaseState (RT tick)
  // reads the counter BEFORE the mirror.
  SeqLock<BaseMirror> base_mirror_{};
  std::atomic<uint64_t> base_seq_{0};
  // rt_callback-lane local: where each configured name was last found in the
  // message (-1 = not yet), so a steady layout costs one compare per name.
  static constexpr std::size_t kBaseSlotPos = 0;
  static constexpr std::size_t kBaseSlotQuat = 1;
  static constexpr std::size_t kBaseSlotLinvel = 2;
  static constexpr std::size_t kBaseSlotAngvel = 3;
  static constexpr std::size_t kBaseSlotImu = 4;  // gyro s at +2s, accel s at +2s+1
  std::array<int, kBaseSlotImu + 2 * static_cast<std::size_t>(kMaxImuSites)> base_sensor_idx_{};

  // RT-thread local — only ReadBaseState touches these. The base lane has its
  // own tick budget (`base_state.max_consecutive_missed_ticks`, defaulting to
  // the wrench lane's) because the two topics need not share a rate.
  uint64_t rt_base_last_seen_seq_{0};
  int rt_base_miss_count_{0};
  int base_max_missed_ticks_{5};

  // Declared LAST so it is destroyed FIRST: a callback still in flight at
  // destruction must find the mirror and names above it alive (the wrench and
  // joint subs follow the same rule relative to their mirrors). CloseStateLane
  // normally makes this moot; the order keeps it true when it was not called.
  rclcpp::Subscription<rtc_msgs::msg::SimSensorState>::SharedPtr base_sub_;
};

}  // namespace rtc

#endif  // INTEGRATED_BRINGUP_BACKENDS_MUJOCO_NATIVE_BACKEND_H_
