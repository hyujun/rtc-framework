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
#include <type_traits>
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
// Floating-base lane — optional, YAML-enabled:
//   - `devices.<group>.backend.base_state.sensor_topic` names the group's
//     SimSensorState topic (rtc_mujoco_sim `sensor_topic`); `.pos` / `.quat` /
//     `.linvel` / `.angvel` name the MJCF framepos / framequat / framelinvel /
//     frameangvel sensors of the base body. The names are the MJCF's own
//     (ARCH-1: nothing here knows a robot).
//   - Empty `sensor_topic` → HasFloatingBase() false and the lane is not
//     claimed; the controller reads DeviceState's identity default.
//   - Conversion (quaternion order, twist frame) lives in ONE place:
//     floating_base_conventions.hpp. This backend only looks sensors up by
//     name, guards them, and hands the arrays over.
//   - The base lane carries no IMU. The keys `base_state.imu_gyro` /
//     `.imu_accel` of the bundled lane are refused at configure (ERROR, and
//     NEITHER lane is claimed), so a profile that still has them fails loudly
//     instead of quietly losing its IMU.
//
// IMU lane — optional, YAML-enabled, independent of the base lane:
//   - `devices.<group>.backend.imu_state.sensor_topic` names a SimSensorState
//     topic; `.site_names[]` labels the IMU sites (index = DeviceState IMU
//     site), and `.gyro[]` / `.accel[]` are the parallel lists of the MJCF
//     gyro / accelerometer sensor names of each site. HasImuState() is true
//     when the lane is configured and valid.
//   - Values are passed through raw, in the site's own frame (gyro, then
//     specific force) — no rotation, debias or gravity compensation.
//   - Verdict, stamp and counter are PER SITE: a site whose sample is
//     non-finite or absent is dropped alone, and the other site and the base
//     lane keep accepting.
//   - A configuration error of one lane (missing names, duplicate
//     `site_names`, more than kMaxImuSites sites, `gyro` / `accel` of another
//     length) refuses that lane only (ERROR); the other one is unaffected.
//
// Subscriptions of the two lanes:
//   - Same `sensor_topic` → ONE subscription; its callback reads the receipt
//     clock once and runs DecodeBase then DecodeImu, so both lanes record the
//     same stamp for one message and a refusal by one decoder never skips the
//     other. The two mirrors are still read separately: one RT tick may see
//     the base of one message and the IMU of the next.
//   - Different topics → two subscriptions; only one lane configured → that
//     one only. Every subscription is on the state callback group and goes
//     through GuardStateLane.
//   - Both lanes have the wrench lane's shape: a SeqLock mirror written on the
//     rt_callback lane, read on the RT tick with a miss counter (the lane's
//     own `max_consecutive_missed_ticks`, defaulting to the wrench lane's)
//     that clears the verdict while the values stay in place. The accepted-
//     sample counter is part of the mirror, so the RT side can never pair a
//     counter with a payload of another sample.
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

  // IMU lane — claimed only when `imu_state.sensor_topic` is set and the
  // lane's configuration is valid (see the class banner). Same configure-time
  // rule as HasFloatingBase().
  [[nodiscard]] bool HasImuState() const noexcept override { return imu_lane_enabled_; }

  void ReadImuState(DeviceStateCache& cache) noexcept override;

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

  // ── Floating-base and IMU lanes ────────────────────────────────────────────
  // One SimSensorState message, decoded for the lanes its subscription feeds.
  // The receipt clock is read ONCE here and handed to both decoders, so both
  // lanes record the same stamp for a shared message; each decoder accepts or
  // drops its own lane and neither one's verdict skips the other.
  void OnSimSensorState(const rtc_msgs::msg::SimSensorState& msg, bool decode_base,
                        bool decode_imu) noexcept;
  // True when the lane took the sample (base: the whole pose; IMU: at least
  // one site). Both run on the rt_callback lane — no allocation, no throw.
  bool DecodeBase(const rtc_msgs::msg::SimSensorState& msg, int64_t recv_steady_ns) noexcept;
  bool DecodeImu(const rtc_msgs::msg::SimSensorState& msg, int64_t recv_steady_ns) noexcept;

  // The MJCF sensor names the base lane looks up, from YAML `base_state.*`.
  // Fixed after Configure; read on the rt_callback lane.
  struct BaseLaneNames {
    std::string pos;
    std::string quat;
    std::string linvel;
    std::string angvel;
  };

  // The IMU lane's sites, from YAML `imu_state.*` (index = DeviceState IMU
  // site). Fixed after Configure; read on the rt_callback lane.
  struct ImuLaneNames {
    std::array<std::string, kMaxImuSites> site{};
    std::array<std::string, kMaxImuSites> gyro{};
    std::array<std::string, kMaxImuSites> accel{};
    int num_sites{0};
  };

  // Latest accepted base sample, already in DeviceState's convention.
  // `recv_steady_ns` is stamped on the rt_callback lane for an ACCEPTED sample
  // only — the same rule as FingertipForceMirror::recv_steady_ns, for the
  // same reason (a dropped sample must not refresh the age of the one still
  // in the mirror). `sequence` counts accepted samples (0 = never) and lives
  // INSIDE the payload: one Load hands the RT side a counter and the sample it
  // counts, never a counter of the next sample with the payload of this one.
  struct BaseMirror {
    FloatingBaseSample sample{};
    int64_t recv_steady_ns{0};
    uint64_t sequence{0};
    bool received_at_least_once{false};
  };

  // Latest accepted sample of each IMU site, raw (site frame: gyro xyz then
  // accelerometer xyz). Per-site stamp, counter and received flag follow
  // BaseMirror's rules site by site; a dropped site keeps its previous values,
  // stamp and counter while another site of the same message moves on.
  // `num_sites` is the configured count once any site has been accepted, 0
  // before — the "never received" zero of DeviceState::num_imu_sites.
  struct ImuMirror {
    std::array<std::array<double, kImuChannelsPerSite>, kMaxImuSites> imu{};
    std::array<int64_t, kMaxImuSites> recv_steady_ns{};
    std::array<uint64_t, kMaxImuSites> sequence{};
    std::array<bool, kMaxImuSites> received{};
    int num_sites{0};
  };

  static_assert(std::is_trivially_copyable_v<BaseMirror>,
                "BaseMirror must be trivially copyable for SeqLock");
  static_assert(std::is_trivially_copyable_v<ImuMirror>,
                "ImuMirror must be trivially copyable for SeqLock");

  bool base_lane_enabled_{false};
  bool imu_lane_enabled_{false};
  BaseLaneNames base_names_{};
  ImuLaneNames imu_names_{};

  // Single writer per mirror: each is written only by its own decoder, and
  // every SimSensorState callback runs on the one MutuallyExclusive state
  // group, so the IMU decoder's Load → update → Store never overlaps another
  // Store of the same mirror. ReadBaseState / ReadImuState (RT tick) Load once.
  SeqLock<BaseMirror> base_mirror_{};
  SeqLock<ImuMirror> imu_mirror_{};
  // rt_callback-lane local: where each configured name was last found in the
  // message (-1 = not yet), so a steady layout costs one compare per name.
  // One array per lane — the lanes may read different messages.
  static constexpr std::size_t kBaseSlotPos = 0;
  static constexpr std::size_t kBaseSlotQuat = 1;
  static constexpr std::size_t kBaseSlotLinvel = 2;
  static constexpr std::size_t kBaseSlotAngvel = 3;
  std::array<int, 4> base_sensor_idx_{};
  std::array<int, 2 * static_cast<std::size_t>(kMaxImuSites)>
      imu_sensor_idx_{};  // gyro 2s, accel 2s+1

  // RT-thread local — only ReadBaseState / ReadImuState touch these. Each lane
  // has its own tick budget (`<lane>.max_consecutive_missed_ticks`, defaulting
  // to the wrench lane's) because the topics need not share a rate.
  uint64_t rt_base_last_seen_seq_{0};
  int rt_base_miss_count_{0};
  int base_max_missed_ticks_{5};
  std::array<uint64_t, kMaxImuSites> rt_imu_last_seen_seq_{};
  std::array<int, kMaxImuSites> rt_imu_miss_count_{};
  int imu_max_missed_ticks_{5};

  // Declared LAST so they are destroyed FIRST: a callback still in flight at
  // destruction must find the mirrors and names above alive (the wrench and
  // joint subs follow the same rule relative to their mirrors). CloseStateLane
  // normally makes this moot; the order keeps it true when it was not called.
  // `base_sub_` feeds the base lane, or both lanes when they share a topic;
  // `imu_sub_` exists only for an IMU lane on a topic of its own.
  rclcpp::Subscription<rtc_msgs::msg::SimSensorState>::SharedPtr base_sub_;
  rclcpp::Subscription<rtc_msgs::msg::SimSensorState>::SharedPtr imu_sub_;
};

}  // namespace rtc

#endif  // INTEGRATED_BRINGUP_BACKENDS_MUJOCO_NATIVE_BACKEND_H_
