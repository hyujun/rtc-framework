#pragma once

#include "udp_hand_driver/udp_hand_controller.hpp"
#include "udp_hand_driver/udp_hand_failure_detector.hpp"
#include "udp_hand_driver/udp_hand_timing_logger.hpp"
#include <rtc_msgs/msg/calibration_command.hpp>
#include <rtc_msgs/msg/calibration_status.hpp>
#include <rtc_msgs/msg/hand_sensor_state.hpp>
#include <rtc_msgs/msg/joint_command.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Unified hand UDP node using request-response protocol.
//
// Owns a UdpHandController with a self-clocked CommLoop that polls the hand
// device at loop_rate_hz (default 500 Hz):
//   [write position (only when commanded)] -> read motors -> read sensors
//
// Publishes full state directly from the CommLoop callback (no ROS timer). The
// read/state-publish rate is autonomous at loop_rate_hz and does not depend on
// command arrival; the write UDP runs only when /…/joint_command was received.
//
// Pre-allocated messages avoid dynamic allocation on the publish path.
// Receives commands on the command_topic (default /hand/joint_command).
//
// Lifecycle states:
//   Unconfigured -> on_configure -> Inactive -> on_activate -> Active
//   Active -> on_deactivate -> Inactive -> on_cleanup -> Unconfigured
//
// Tier 1 (on_configure): parameters, controller, publishers, subscribers,
//   timers, pre-allocated messages, EventLoop callback.
// Tier 2 (on_activate): controller Start, fake tick timer, failure detector.
// Tier 0 (constructor): *only* what main() must know before it spins — the aux
//   callback group and the thread-layout parameter. See UdpHandNode().

// Non-RT auxiliary I/O lane (issue #345). The blocking file writes — the timing
// CSV drain (up to kHandUdpTimingBufferCapacity rows in a single 1 Hz burst) and
// the stats JSON save — run here instead of the node's default executor thread,
// which also serves the joint-command subscription. CFS: this lane must never
// preempt the CommLoop, and it is the only hand thread that leaves the
// hand_driver core. `cpu_core` is filled at runtime from the `aux_cpu_slot`
// parameter, whose default 0 mirrors the shell SSoT
// repo_scripts/scripts/lib/rt_common.sh::get_os_cores (the OS/housekeeping slot,
// which `get_cm_shield_cpus` excludes from the shield — so pinning here needs no
// cpuset change and cannot EINVAL under an active `cset shield`).
namespace udp_hand_driver {
inline const rtc::ThreadConfig kHandAuxIoConfig{.cpu_core = -1,
                                                .sched_policy = SCHED_OTHER,
                                                .sched_priority = 0,
                                                .nice_value = 0,
                                                .name = "hand_aux_io"};
}  // namespace udp_hand_driver

class UdpHandNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  UdpHandNode();
  ~UdpHandNode() override;

  /// Callback group holding the blocking file-I/O timers (timing CSV drain,
  /// stats JSON save). main() services it on the dedicated `hand_aux_io`
  /// executor thread, so it is created with
  /// `automatically_add_to_executor_with_node = false` — otherwise the executor
  /// that adds this node would claim it and the lane split would be a no-op.
  [[nodiscard]] rclcpp::CallbackGroup::SharedPtr GetAuxCallbackGroup() const {
    return cb_group_aux_;
  }

  /// Slot index for the aux lane, or a negative value meaning "do not pin".
  /// Read from the `aux_cpu_slot` parameter; slot -> logical CPU translation is
  /// ApplyThreadConfig's job (issue #163).
  [[nodiscard]] int AuxCpuSlot() const;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

 private:
  // Drain the EventLoop's timing producer into the CSV (1 Hz, non-RT).
  // Runs on the `hand_aux_io` executor thread via cb_group_aux_ (issue #345),
  // NOT on the default executor thread — the burst is a blocking file write.
  void DrainHandUdpTiming() noexcept;

  // Pre-allocate ROS2 messages once in on_configure (non-RT).
  // Avoids dynamic allocation on the EventLoop publish path.
  void PreallocateMessages();

  // Called directly from EventLoop thread — publishes state at EventLoop rate.
  // Uses pre-allocated messages to avoid dynamic allocation.
  void PublishFromEventLoop(const udp_hand_driver::UdpHandState& state,
                            const udp_hand_driver::FingertipFTState& ft_state);

  void PublishCalibrationStatus();

  // Persist comm/timing stats to <session>/device/hand_udp_stats.json.
  // Three concurrent call contexts, serialized by save_stats_mutex_:
  //   1. the periodic timer, on the `hand_aux_io` thread (cb_group_aux_)
  //   2. the failure-detector thread, from its failure callback
  //   3. lifecycle teardown (on_deactivate / on_cleanup / on_error / ~UdpHandNode),
  //      on the default executor thread
  // The lane split moved (1) off the default executor thread, so (1) and (3) are
  // now genuinely concurrent rather than serialized by being the same thread.
  // verbose=false (periodic saves) skips the INFO summary logs.
  // Defined in udp_hand_node_stats.cpp.
  void SaveCommStats(bool verbose = true) const;

  // Cancel timers and stop the worker threads (failure detector + comm loop),
  // leaving controller_ constructed for possible reactivation. Idempotent;
  // shared by on_deactivate and on_error.
  void StopRuntime();

  // Release all publisher/subscription/timer handles and the controller.
  // Idempotent; shared by on_cleanup and on_error.
  void ReleaseResources();

  std::unique_ptr<udp_hand_driver::UdpHandController> controller_;
  std::unique_ptr<udp_hand_driver::UdpHandFailureDetector> failure_detector_;

  // Data publishers — LifecyclePublisher (gated by lifecycle state).
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr motor_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::HandSensorState>::SharedPtr sensor_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::HandSensorState>::SharedPtr
      sensor_monitor_pub_;
  rclcpp::Subscription<rtc_msgs::msg::JointCommand>::SharedPtr joint_command_sub_;
  rclcpp::Subscription<rtc_msgs::msg::CalibrationCommand>::SharedPtr calib_cmd_sub_;
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::CalibrationStatus>::SharedPtr
      calib_status_pub_;
  rclcpp::TimerBase::SharedPtr calib_status_timer_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> motor_names_;
  std::vector<std::string> fingertip_names_;
  int num_fingertips_{udp_hand_driver::kDefaultNumFingertips};

  // Per-joint position offset (URDF/controller frame − firmware zero), radians.
  // Parsed once in on_configure from the `joint_position_offsets_deg` YAML list
  // (degrees, joint_state_names order). Applied at the two ROS boundaries only:
  //   read/publish : controller_pos = udp_pos + offset  (PublishFromEventLoop)
  //   command/write: udp_cmd        = controller_cmd − offset  (command sub)
  // so the round-trip is identity. Zero by default → backward compatible.
  // Position-only: velocity/effort and motor-space positions are untouched.
  std::array<float, udp_hand_driver::kNumHandMotors> joint_offset_rad_{};

  // Firmware-slot ← command-index map for incoming /…/joint_command. The
  // publisher (controller/backend) labels the message in its own joint_names
  // order (URDF / _base.yaml) and — per the "receiver reorders by name"
  // convention that the backend already applies to /…/joint_states — this node
  // remaps each value into firmware slot order (joint_names_ = joint_state_names,
  // the UDP wire order) before the positional CommLoop write. Built once from
  // the first message that carries joint_names; -1 / no-names ⇒ positional
  // fallback. Callback-thread only (no cross-thread access), so no atomic.
  std::array<int, udp_hand_driver::kNumHandMotors> cmd_name_reorder_{};
  bool cmd_name_reorder_ready_{false};

  // Sensor-message layout selector, cached from the SensorProtocol capability
  // at on_configure (off the hot path). true → publish the 1b per-fingertip
  // force vector (fs.f) from state.sensor_force; false → publish the 1a
  // barometer/ToF + F/T-inference payload. The publish branch reads this bool,
  // never a version string (ARCH-3).
  bool sensor_uses_force_layout_{false};

  // Link status — standalone rclcpp::Publisher (NOT LifecyclePublisher).
  // Safety-relevant: must remain publishable in any lifecycle state.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr link_status_pub_;
  bool ft_enabled_{false};
  // Per-channel link-down thresholds (strict, #1). Computed once in on_configure
  // from link_fail_timeout_ms at each channel's effective attempt rate; the SSoT
  // the detector cfg + publish + stats all read (via UdpHandController::LinkDown)
  // so the E-STOP decision and the link_status Bool cannot disagree. comm =
  // motor/joint (every comm cycle); sensor = the shorter sensor-rate budget.
  uint64_t link_fail_threshold_{10};
  uint64_t link_fail_threshold_sensor_{10};
  bool prev_link_ok_{true};

  // Pre-allocated messages
  sensor_msgs::msg::JointState joint_js_msg_;
  sensor_msgs::msg::JointState motor_js_msg_;
  rtc_msgs::msg::HandSensorState sensor_msg_;
  std_msgs::msg::Bool link_msg_;

  // Link status decimation
  int link_decimation_{5};
  int link_cycle_counter_{0};

  std::size_t publish_count_{0};

  // Fake-hand standalone support. The controller's CommLoop RT thread self-clocks
  // and runs the LPF model in fake mode (UdpHandController::RunFakeCommCycle), so
  // the node needs no self-tick timer or command cache — commands flow through the
  // normal command sub → SendCommandAndRequestStates path.
  bool use_fake_hand_{false};

  std::chrono::steady_clock::time_point start_time_{std::chrono::steady_clock::now()};

  // Periodic stats persistence (crash-safety): without it the stats JSON is
  // written only on graceful teardown and is lost on SIGKILL/crash. Interval
  // is a fixed constant — see on_activate (no ROS param on purpose).
  rclcpp::TimerBase::SharedPtr stats_save_timer_;

  // Guards every aux-lane callback body (stats JSON save, timing CSV drain) and
  // is taken empty by StopRuntime() as a quiesce barrier. Before the lane split
  // these callbacks and lifecycle teardown were the same thread and could not
  // overlap; now they can, and they share `controller_` and the SPSC timing ring
  // (single-consumer). One mutex rather than two: neither callback calls the
  // other, so there is no lock order to get wrong, and both are non-RT (issue #345).
  mutable std::mutex aux_lane_mutex_;

  // Cached config for on_activate logging
  std::string target_ip_;
  int target_port_{0};
  std::string comm_mode_str_;

  // ── Per-EventLoop-tick timing CSV (mpc_timing_log pattern) ─────────────
  // Producer (filled on the EventLoop thread) → 1 Hz drain on aux timer →
  // rtc::ThreadTimingCsvLogger writes one row per tick to
  // <session>/timing/hand_udp_timing_log.csv. Open() runs once on the first
  // on_activate and is gated by `hand_udp_timing_opened_` so reactivation does
  // not truncate or re-write the header. The *timer* has a shorter lifetime than
  // the logger — StopRuntime() cancels it to quiesce the aux lane and every
  // on_activate recreates it — so the two are separate members (issue #345).
  rtc::HandUdpTimingBuffer hand_udp_timing_producer_;
  udp_hand_driver::UdpHandTimingLogger hand_udp_timing_logger_;
  rclcpp::TimerBase::SharedPtr hand_udp_timing_timer_;
  bool hand_udp_timing_opened_{false};
  std::uint64_t hand_udp_timing_drop_baseline_{0};

  // Aux lane (issue #345). Created in the constructor, NOT on_configure: the
  // group is handed to main()'s aux executor before the first lifecycle
  // transition, and it must survive an on_cleanup -> on_configure cycle. A group
  // recreated in on_configure would be a *different* object that the executor
  // never saw, and the aux timers would silently stop firing.
  rclcpp::CallbackGroup::SharedPtr cb_group_aux_;
};
