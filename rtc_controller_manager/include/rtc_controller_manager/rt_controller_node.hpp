#pragma once

// ── Project headers ───────────────────────────────────────────────────────────
#include "rtc_base/logging/data_logger.hpp"
#include "rtc_base/logging/log_buffer.hpp"
#include "rtc_base/threading/publish_buffer.hpp"
#include "rtc_controller_manager/controller_timing_profiler.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
// ── ROS2 ─────────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <rtc_msgs/msg/joint_command.hpp>
#include <rtc_msgs/msg/gui_position.hpp>
#include <rtc_msgs/msg/robot_target.hpp>
#include <rtc_msgs/msg/device_state_log.hpp>
#include <rtc_msgs/msg/device_sensor_log.hpp>
#include <rtc_msgs/msg/grasp_state.hpp>
#include <rtc_msgs/msg/hand_sensor_state.hpp>

// ── C++ stdlib ────────────────────────────────────────────────────────────────
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// ── RtControllerNode ──────────────────────────────────────────────────────────
//
// 500 Hz position controller node.
//
// Threading model:
//   - rt_loop (jthread):   clock_nanosleep RT loop — ControlLoop() + CheckTimeouts()
//   - publish_thread (jthread): SPSC drain → all ROS2 publish() calls
//   - cb_group_sensor_:    joint_state_sub_, target_sub_, hand_state_sub_  (Sensor core)
//   - cb_group_log_:       drain_timer_  (non-RT core)
//   - cb_group_aux_:       estop_pub_  (aux core)
class RtControllerNode : public rclcpp::Node
{
public:
  RtControllerNode();
  ~RtControllerNode() override;

  // Public accessors for main() to retrieve callback groups
  rclcpp::CallbackGroup::SharedPtr GetSensorGroup() const {return cb_group_sensor_;}
  rclcpp::CallbackGroup::SharedPtr GetLogGroup()    const {return cb_group_log_;}
  rclcpp::CallbackGroup::SharedPtr GetAuxGroup()    const {return cb_group_aux_;}

  // RT loop lifecycle — called from main()
  void StartRtLoop(const rtc::ThreadConfig& rt_cfg);
  void StartPublishLoop(const rtc::ThreadConfig& pub_cfg);
  void StopRtLoop();
  void StopPublishLoop();

private:
  // ── Session directory helpers ─────────────────────────────────────────────
  // Resolves session directory from UR5E_SESSION_DIR env var or creates one.
  std::filesystem::path ResolveAndSetupSessionDir();

  // ── Initialisation helpers ────────────────────────────────────────────────
  void CreateCallbackGroups();
  void DeclareAndLoadParameters();
  void CreateSubscriptions();
  void CreatePublishers();
  void ExposeTopicParameters();
  void CreateTimers();

  // ── Subscription callbacks (unified per-device) ──────────────────────────
  void DeviceJointStateCallback(int device_slot,
      sensor_msgs::msg::JointState::SharedPtr msg);
  void DeviceMotorStateCallback(int device_slot,
      sensor_msgs::msg::JointState::SharedPtr msg);
  void DeviceSensorCallback(int device_slot,
      std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void HandSensorStateCallback(int device_slot,
      rtc_msgs::msg::HandSensorState::SharedPtr msg);
  void DeviceTargetCallback(int device_slot,
      rtc_msgs::msg::RobotTarget::SharedPtr msg);

  // ── Device name configuration ────────────────────────────────────────────
  void LoadDeviceNameConfigs();
  void BuildDeviceReorderMap(int device_slot,
      const std::vector<std::string>& msg_names);

  // ── RT loop (clock_nanosleep) ─────────────────────────────────────────────
  void RtLoopEntry(const rtc::ThreadConfig& cfg);
  void CheckTimeouts();   // 50 Hz watchdog — called inline from RT loop
  void ControlLoop();     // 500 Hz control loop

  // ── Publish offload (SPSC drain → publish) ──────────────────────────────
  void PublishLoopEntry(const rtc::ThreadConfig& cfg);
  void DrainLog();        // Log drain (non-RT core)

  void PublishEstopStatus(bool estopped);

  /// Trigger a global E-Stop that propagates to all subsystems.
  /// Safe to call from any thread. Idempotent — second call is a no-op.
  void TriggerGlobalEstop(std::string_view reason) noexcept;
  /// Clear global E-Stop and re-enable all subsystems.
  void ClearGlobalEstop() noexcept;
  [[nodiscard]] bool IsGlobalEstopped() const noexcept
  {
    return global_estop_.load(std::memory_order_acquire);
  }

  // ── ROS2 handles ──────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_sensor_;
  rclcpp::CallbackGroup::SharedPtr cb_group_log_;
  rclcpp::CallbackGroup::SharedPtr cb_group_aux_;

  // ── Configurable topic subscriptions (created from controller YAML) ──────
  // Key = topic name, value = subscription handle (kept alive for node lifetime)
  std::vector<rclcpp::SubscriptionBase::SharedPtr> topic_subscriptions_;

  // Fixed control subscriptions (always present)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             controller_selector_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  controller_gains_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr               request_gains_sub_;

  // ── Configurable topic publishers (created from controller YAML) ──────────
  // Key = topic name, value = publisher + pre-allocated message
  struct PublisherEntry {
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
    std_msgs::msg::Float64MultiArray msg;
    // Reorder map: output index → input (gc.commands) index.
    // Built from joint_state_names → joint_command_names mapping.
    // Empty when no reorder is needed (names are identical or absent).
    std::vector<int> reorder_map;
  };
  std::unordered_map<std::string, PublisherEntry> topic_publishers_;

  // Fixed publishers (always present)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              estop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            active_ctrl_name_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr current_gains_pub_;

  // JointCommand publishers (created from controller YAML kJointCommand roles)
  struct JointCommandPublisherEntry {
    rclcpp::Publisher<rtc_msgs::msg::JointCommand>::SharedPtr publisher;
    rtc_msgs::msg::JointCommand msg;  // pre-allocated
    std::vector<int> reorder_map;     // config (joint_state_names) → command order
  };
  std::unordered_map<std::string, JointCommandPublisherEntry> joint_command_publishers_;

  // Typed publishers for new topic roles
  template <typename MsgT>
  struct TypedPublisherEntry {
    typename rclcpp::Publisher<MsgT>::SharedPtr publisher;
    MsgT msg;  // pre-allocated
  };
  std::unordered_map<std::string, TypedPublisherEntry<rtc_msgs::msg::GuiPosition>>
      gui_position_publishers_;
  std::unordered_map<std::string, TypedPublisherEntry<rtc_msgs::msg::RobotTarget>>
      robot_target_publishers_;
  std::unordered_map<std::string, TypedPublisherEntry<rtc_msgs::msg::DeviceStateLog>>
      device_state_log_publishers_;
  std::unordered_map<std::string, TypedPublisherEntry<rtc_msgs::msg::DeviceSensorLog>>
      device_sensor_log_publishers_;
  std::unordered_map<std::string, TypedPublisherEntry<rtc_msgs::msg::GraspState>>
      grasp_state_publishers_;

  // ── Digital Twin JointState republishers (RELIABLE, depth 10) ────────────
  // key = "/{group}/digital_twin/joint_states"
  struct DigitalTwinEntry {
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    sensor_msgs::msg::JointState msg;  // pre-allocated with config joint_state_names
  };
  std::unordered_map<std::string, DigitalTwinEntry> digital_twin_publishers_;
  // group_slot → digital_twin topic name mapping
  std::unordered_map<int, std::string> slot_to_dt_topic_;

  // Per-controller topic config cache (index = controller index)
  std::vector<rtc::TopicConfig> controller_topic_configs_;

  // Controller name → index lookup (populated once at startup)
  // Maps both display names ("DemoJointController") and config keys ("demo_joint_controller")
  std::unordered_map<std::string, int> controller_name_to_idx_;

  // ── Dynamic device group management ──────────────────────────────────────
  std::set<std::string> active_groups_;           // union of all controller groups
  std::map<std::string, int> group_slot_map_;     // group name → PublishSnapshot slot index

  // ── Device timeout entries (E-STOP watchdog) ──────────────────────────────
  struct DeviceTimeoutEntry {
    std::string group_name;
    std::string state_topic;
    std::chrono::milliseconds timeout{100};
    std::chrono::steady_clock::time_point last_update{};
    // Not std::atomic — written from a single subscriber callback,
    // read from RT thread. Worst case: 1-tick delay (acceptable).
    volatile bool received{false};
  };
  std::vector<DeviceTimeoutEntry> device_timeouts_;
  [[nodiscard]] bool AllTimeoutDevicesReceived() const noexcept;

  // ── Per-device state caches (indexed by group_slot_map_) ─────────────────
  static constexpr int kMaxDevices = rtc::PublishSnapshot::kMaxGroups;
  struct DeviceStateCache {
    int num_channels{0};
    std::array<double, rtc::kMaxDeviceChannels> positions{};
    std::array<double, rtc::kMaxDeviceChannels> velocities{};
    std::array<double, rtc::kMaxDeviceChannels> efforts{};
    // Motor-space data (separate from joint-space)
    int num_motor_channels{0};
    std::array<double, rtc::kMaxDeviceChannels> motor_positions{};
    std::array<double, rtc::kMaxDeviceChannels> motor_velocities{};
    std::array<double, rtc::kMaxDeviceChannels> motor_efforts{};
    std::array<int32_t, rtc::kMaxSensorChannels> sensor_data{};
    std::array<int32_t, rtc::kMaxSensorChannels> sensor_data_raw{};
    int num_sensor_channels{0};
    // Inference (force/displacement per fingertip)
    std::array<float, rtc::kMaxInferenceValues> inference_data{};
    std::array<bool, rtc::kMaxFingertips> inference_enable{};
    int num_inference_fingertips{0};
    bool valid{false};
  };
  std::array<DeviceStateCache, kMaxDevices> device_states_{};
  std::array<DeviceStateCache, kMaxDevices> cached_device_states_{};
  std::mutex device_state_mutex_;

  // Per-device targets
  std::array<std::array<double, rtc::kMaxDeviceChannels>, kMaxDevices> device_targets_{};
  std::array<std::array<double, rtc::kMaxDeviceChannels>, kMaxDevices> device_target_snapshots_{};

  // Read-only parameter guard handle (topic params immutable after init)
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rclcpp::TimerBase::SharedPtr drain_timer_;  // log drain (log thread)

  // ── RT loop (clock_nanosleep, replaces control_timer_ + timeout_timer_) ──
  std::jthread rt_loop_thread_;
  std::atomic<bool> rt_loop_running_{false};

  // ── Publish offload (SPSC buffer + dedicated thread) ────────────────────
  rtc::ControlPublishBuffer publish_buffer_{};
  std::jthread publish_thread_;
  std::atomic<bool> publish_running_{false};

  // ── Domain objects ────────────────────────────────────────────────────────
  std::vector<std::unique_ptr<rtc::RTControllerInterface>> controllers_;
  std::atomic<int> active_controller_idx_{1};
  std::unique_ptr<rtc::DataLogger> logger_;
  rtc::ControlLogBuffer              log_buffer_{};              // SPSC ring buffer
  rtc::ControllerTimingProfiler      timing_profiler_{};         // Compute() timing

  // ── Shared state ──────────────────────────────────────────────────────────
  // All device state is now in device_states_[] / cached_device_states_[]
  // guarded by device_state_mutex_.

  mutable std::mutex target_mutex_;

  std::atomic<bool> print_timing_summary_{false};
  std::atomic<bool> state_received_{false};
  std::atomic<bool> target_received_{false};

  // ── Per-device name configuration ────────────────────────────────────────
  std::map<std::string, rtc::DeviceNameConfig> device_name_configs_;
  std::vector<std::string> slot_to_group_name_;  // reverse: slot index → group name

  // Per-device reorder maps (indexed by device slot)
  struct DeviceReorderMap {
    std::vector<int> reorder;
    bool built{false};
    bool built_from_msg{false};  // true only when built from actual device msg
  };
  std::array<DeviceReorderMap, kMaxDevices> device_reorder_maps_{};

  // ── Simulation sync (CV-based wakeup) ──────────────────────────────────
  bool                    use_sim_time_sync_{false};
  double                  sim_sync_timeout_sec_{5.0};
  std::mutex              state_cv_mutex_;
  std::condition_variable state_cv_;
  std::atomic<bool>       state_fresh_{false};

  // ── Parameters ────────────────────────────────────────────────────────────
  double      control_rate_{500.0};
  bool        enable_logging_{true};
  bool        enable_estop_{true};
  std::string robot_ns_{"ur5e"};  // robot namespace for manager-level topics

  std::size_t loop_count_{0};

  // ── Initialization timeout ──────────────────────────────────────────────────
  bool     init_complete_{false};
  uint64_t init_wait_ticks_{0};
  uint64_t init_timeout_ticks_{2500};  // default 5s at 500Hz

  // ── Auto-hold position ─────────────────────────────────────────────────────
  bool auto_hold_position_{true};

  // Baseline for log timestamps — captured on first ControlLoop() iteration.
  std::chrono::steady_clock::time_point log_start_time_{};

  // ── Global E-Stop ──────────────────────────────────────────────────────────
  std::atomic<bool> global_estop_{false};
  std::string       estop_reason_;

  // ── Per-tick timing & overrun detection ──────────────────────────────────
  std::chrono::steady_clock::time_point prev_loop_start_{};
  double budget_us_{2000.0};
  std::atomic<uint64_t> overrun_count_{0};
  std::atomic<uint64_t> compute_overrun_count_{0};
  std::atomic<uint64_t> skip_count_{0};
  std::atomic<uint64_t> consecutive_overruns_{0};
  static constexpr uint64_t kMaxConsecutiveOverruns = 10;
};
