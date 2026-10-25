#pragma once

#include "udp_hand_driver/hand_comm_loop.hpp"
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
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

// Unified hand node using request-response protocol over UDP (protocol_version
// 1a / 1b) or CAN FD (protocol_version 2, epic #895).
//
// Owns one UdpHandController per hand and the self-clocked HandCommLoop
// (`hand_comm`) they all run on, which polls the hands at loop_rate_hz
// (default 500 Hz):
//   [write position (only when commanded)] -> read motors -> read sensors
//
// One hand (`hand_side`, the default) or two (`hand_sides: [right, left]` —
// epic #906 D69 · D71): one process, one loop, one CAN socket per hand, and
// per hand its own topics under `<side>/`, stats JSON and failure detector.
// The loop runs the hands back to back in list order every tick, which
// serializes the bus; one E-Stop topic stops all of them.
//
// The CommLoop publishes nothing itself: it stores a POD snapshot into a SeqLock
// and a timer on the executor thread polls that sequence and publishes (issue
// #345 — before it, now() + message fill + five publishes ran on the SCHED_FIFO
// 65 thread). The read/state-publish rate is autonomous at loop_rate_hz and does
// not depend on command arrival; the write UDP runs only when /…/joint_command
// was received.
//
// Pre-allocated messages avoid dynamic allocation on the publish path.
// Receives commands on the command_topic (default `joint_command`, relative to
// the node namespace — the standalone launch runs the node under /hand).
// Receives the E-Stop on the estop_topic (default `estop`, relative likewise):
// a `true` latches estop_flag_, which the CommLoop checks every tick and
// answers with one zero-write + self-exit (UdpHandController::RunCommCycle).
//
// Lifecycle states:
//   Unconfigured -> on_configure -> Inactive -> on_activate -> Active
//   Active -> on_deactivate -> Inactive -> on_cleanup -> Unconfigured
//
// Tier 1 (on_configure): parameters, controller, publishers, subscribers,
//   timers (including the NRT publish poll), pre-allocated messages.
// Tier 2 (on_activate): controller Start, fake tick timer, failure detector.
// Tier 0 (constructor): *only* what main() must know before it spins — the aux
//   callback group and the thread-layout parameter. See UdpHandNode().

// Non-RT auxiliary I/O lane (issue #345). The blocking file writes — the timing
// CSV drain (up to kHandTimingBufferCapacity rows in a single 1 Hz burst) and
// the stats JSON save — run here instead of the node's default executor thread,
// which also serves the joint-command subscription. CFS: this lane must never
// preempt the CommLoop, and it is the only hand thread that leaves the
// hand_driver core. `cpu_core` is filled at runtime from the `aux_cpu_slot`
// parameter, whose default 0 mirrors the shell SSoT
// repo_scripts/scripts/lib/rt_common.sh::get_os_cores (the OS/housekeeping slot,
// which `get_cm_shield_cpus` excludes from the shield — so pinning here needs no
// cpuset change and cannot EINVAL under an active `cset shield`).
namespace udp_hand_driver {
// "Do not pin" sentinel. Mirrors ThreadConfig::cpu_core == -1
// (rtc_base/threading/thread_config.hpp), which ApplyThreadConfig reads as
// "skip the affinity step and keep policy/priority", and NO_PIN_SENTINEL in
// rtc_tools/launch/pinning.py on the launch side.
inline constexpr int kNoPinSlot = -1;

/// The slot a thread should actually pin to, given the configured slot and the
/// process-wide affinity switch.
///
/// One rule, two callers — main()'s own thread and the hand_aux_io thread — so
/// `use_cpu_affinity:=false` cannot end up honoured in one lane and ignored in
/// the other. Disabling affinity is deliberately *not* disabling the scheduler
/// policy: ApplyThreadConfig skips only the affinity step for a negative slot,
/// so the CommLoop keeps SCHED_FIFO 65 and merely stops being confined to a
/// core (issue #345).
[[nodiscard]] constexpr int ResolvePinSlot(int configured_slot, bool affinity_enabled) noexcept {
  return affinity_enabled ? configured_slot : kNoPinSlot;
}

// How much faster than loop_rate_hz the NRT publish lane polls the state
// SeqLock. The mailbox is latest-wins, so a poll can carry at most one publish:
// sampling at exactly the production rate loses a sample every time two comm
// cycles fall between two polls, and the topic rate lands measurably under
// loop_rate_hz. 2x makes the lossless case the common one; raising it trades
// wakeups (one atomic load each when idle) for a smaller loss tail.
inline constexpr double kPublishPollOversampling = 2.0;

/// Round a raw SeqLock sequence sample down to the last *completed* write.
///
/// SeqLock steps the sequence twice per Store and leaves it odd while the write
/// is in flight. The consumer must carry an even baseline forward: adopting an
/// odd sample as the baseline permanently offsets every later difference by one
/// step, so cycles are silently lost from then on (the counting itself tolerates
/// an odd input — integer division discards the low bit — which is exactly why
/// this belongs in its own named step rather than folded into the subtraction).
[[nodiscard]] constexpr std::uint32_t LastCompletedWrite(std::uint32_t seq) noexcept {
  return seq & ~1U;
}

/// Comm cycles completed between two `LastCompletedWrite` values.
///
/// A completed cycle is 2 sequence steps. Unsigned arithmetic wraps correctly at
/// 2^32, which a 500 Hz loop reaches after ~99 days of continuous running.
[[nodiscard]] constexpr std::uint32_t CyclesBetween(std::uint32_t prev,
                                                    std::uint32_t current) noexcept {
  return (current - prev) / 2U;
}

/// Rate-derived decimation under a pull consumer.
///
/// Adds `cycles` to `accum` and reports whether the decimated event is due,
/// carrying the remainder rather than zeroing it — otherwise a poll that
/// observed 3 cycles would reset the same as one that observed 1, and the
/// effective rate would drift below the configured one. Returns true at most
/// once per call by design: a decimated publish reports current state, so a
/// poll that jumped several cycles still emits one message, not several.
[[nodiscard]] constexpr bool AdvanceDecimation(int& accum, std::uint32_t cycles,
                                               int decimation) noexcept {
  if (decimation <= 0) {
    return true;  // no decimation configured — every cycle is due
  }
  accum += static_cast<int>(cycles);
  if (accum < decimation) {
    return false;
  }
  accum %= decimation;
  return true;
}

/// Endpoint the transport actually opens, after `sil_mode` derivation. The
/// UDP triple serves protocol_version 1a / 1b; `can_interface` serves 2.
struct HandEndpoint {
  std::string target_ip;
  std::string local_ip;
  std::string local_interface;
  std::string can_interface{};
};

/// Apply the `sil_mode` overrides to the yaml-configured endpoint.
///
/// `firmware` SIL moves the device onto loopback, so the node forces
/// `target_ip=127.0.0.1` — and has to drop the local binding with it. A socket
/// pinned to a real NIC cannot reach 127.0.0.1: `SO_BINDTODEVICE` forces egress
/// out that device, so `connect()` and `send()` both report success while every
/// datagram is dropped, surfacing only as recv timeouts with no diagnostic (and
/// on a host that lacks the NIC, as an ENODEV hard failure in `Open()`). The
/// local binding exists solely to disambiguate same-subnet NICs against real
/// hardware, so loopback is precisely the case where it must not apply.
///
/// `off` and `loopmodel` pass through unchanged (`loopmodel` opens no socket).
[[nodiscard]] inline HandEndpoint DeriveHandEndpoint(const std::string& sil_mode,
                                                     HandEndpoint configured) {
  if (sil_mode == "firmware") {
    configured.target_ip = "127.0.0.1";
    configured.local_ip.clear();
    configured.local_interface.clear();
  }
  return configured;
}

/// Loopback device of the CAN FD firmware SIL: the virtual CAN interface the
/// repo's vcan setup creates and `test_canfd_hand_comm` / the proto2
/// fake_hand_firmware default to.
inline constexpr const char* kFirmwareSilCanInterface = "vcan0";

/// Protocol-aware form. `firmware` SIL moves the device onto the loopback of
/// the protocol's transport: UDP (1a / 1b) → the 2-arg rule above; CAN FD (2)
/// → `can_interface` forced to kFirmwareSilCanInterface, the UDP triple left
/// untouched (it is not used on that transport). `off` / `loopmodel` pass
/// through unchanged on every protocol.
[[nodiscard]] inline HandEndpoint DeriveHandEndpoint(const std::string& sil_mode,
                                                     const std::string& protocol_version,
                                                     HandEndpoint configured) {
  if (protocol_version == "2") {
    if (sil_mode == "firmware") {
      configured.can_interface = kFirmwareSilCanInterface;
    }
    return configured;
  }
  return DeriveHandEndpoint(sil_mode, std::move(configured));
}

class CanFdHandComm;  // canfd_hand_comm.hpp — only the lifecycle / stats TUs need the type

inline const rtc::ThreadConfig kHandAuxIoConfig{.cpu_core = -1,
                                                .sched_policy = SCHED_OTHER,
                                                .sched_priority = 0,
                                                .nice_value = 0,
                                                .name = "hand_aux_io"};

/// Topic name of one hand: a relative `name` gets the hand's `prefix`
/// (`"right/"` under `hand_sides`, epic #906 D71); an absolute (`/…`) or
/// private (`~…`) override is taken as written, and an empty prefix — the
/// one-hand mode — changes nothing.
[[nodiscard]] inline std::string PrefixRelativeTopic(const std::string& prefix,
                                                     const std::string& name) {
  if (prefix.empty() || name.empty() || name.front() == '/' || name.front() == '~') {
    return name;
  }
  return prefix + name;
}

/// Free room left in the comm slot once every hand's two receive stages are
/// paid for (D42 generalized to N hands, D71): `slot − N × 2T`, in ms.
/// Configure refuses a non-positive result and warns below 10 % of the slot.
[[nodiscard]] constexpr double CommSlotMarginMs(double slot_ms, std::size_t hands,
                                                double recv_timeout_ms) noexcept {
  return slot_ms - static_cast<double>(hands) * 2.0 * recv_timeout_ms;
}
}  // namespace udp_hand_driver

class UdpHandNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// `options` exists for parameter_overrides injection (tests, composition):
  /// most parameters — sil_mode above all — are declared in on_configure, so an
  /// override in NodeOptions is the only way to set them before the first
  /// lifecycle transition. main() constructs with the default.
  explicit UdpHandNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~UdpHandNode() override;

  /// Callback group holding the blocking file-I/O timers (timing CSV drain,
  /// stats JSON save). main() services it on the dedicated `hand_aux_io`
  /// executor thread, so it is created with
  /// `automatically_add_to_executor_with_node = false` — otherwise the executor
  /// that adds this node would claim it and the lane split would be a no-op.
  [[nodiscard]] rclcpp::CallbackGroup::SharedPtr GetAuxCallbackGroup() const {
    return cb_group_aux_;
  }

  /// Slot the `hand_aux_io` thread should pin to — the `aux_cpu_slot` parameter
  /// passed through ResolvePinSlot, so it is kNoPinSlot when affinity is off.
  /// Slot -> logical CPU translation is ApplyThreadConfig's job (issue #163).
  [[nodiscard]] int AuxCpuSlot() const;

  /// Whether this process pins threads at all (`use_cpu_affinity`, default true).
  ///
  /// Declared in the constructor rather than on_configure because main() applies
  /// it before spinning — see UdpHandNode(). This is what closes the gap the old
  /// main() self-pin left: it ran before the node existed, so it could not read
  /// the switch and pinned unconditionally (issue #345, decision D3).
  [[nodiscard]] bool UseCpuAffinity() const;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

 protected:
  /// Start hand `index` of this configuration (on_activate, in hand order):
  /// prepares its run on the shared loop, whose thread starts only after every
  /// hand did. Virtual solely as the test seam for the activate rollback — a
  /// hand whose start fails must leave the hands before it closed (#906 B3).
  [[nodiscard]] virtual bool StartHand(std::size_t index);

  /// Whether hand `index`'s controller is running. Tests of the rollback.
  [[nodiscard]] bool HandRunning(std::size_t index) const;

 private:
  // Everything one hand owns (epic #906 D69): its controller and detector,
  // its topics, its stats JSON, its command-name map. One hand per entry of
  // `hand_sides`, or the single hand of the `hand_side` mode. Not movable (the
  // atomics, and the callbacks that hold a pointer to it) — units_ holds them
  // by unique_ptr. Destroyed by the next on_configure or ReleaseResources().
  struct HandUnit {
    // "right" | "left" — the CAN FD node id (D51).
    std::string side;
    // Prefix of this hand's relative topic names: empty in the hand_side mode
    // (the namespace alone, D53), "<side>/" under hand_sides (D71).
    std::string topic_prefix;
    // Stats JSON suffix: "_<side>" under hand_sides or for an explicit
    // hand_side, else empty so the one-hand default name stays (D51).
    std::string file_suffix;
    // Log tag: "[<side>] " under hand_sides, empty otherwise, so one-hand logs
    // read as before.
    std::string log_tag;
    // The link in words ("192.168.0.100:55151" or "CAN FD can0 (right)").
    std::string link_desc;

    std::unique_ptr<udp_hand_driver::UdpHandController> controller;
    // Declared after controller: it holds a reference to it, so it goes first.
    std::unique_ptr<udp_hand_driver::UdpHandFailureDetector> failure_detector;

    // The detector's verdict for the current run, kept past the detector itself
    // (#927 I-1): StopRuntime() destroys the detector, and the teardown save that
    // follows reports this instead of a default `false`. Written by StopRuntime()
    // from a live detector only, cleared by on_activate once a new run has
    // started (a failed start keeps the previous run's verdict); guarded by
    // aux_lane_mutex_ like failure_detector.
    bool failure_latched{false};

    // Non-owning view of the CAN FD link when protocol_version is "2" (null
    // otherwise): SaveHandStats reads CanFdHandComm::canfd_stats() through it,
    // which HandComm deliberately does not expose. The controller owns the comm,
    // so this pointer is valid exactly while `controller` holds the object it was
    // taken from: assigned only after SetHandComm() accepted the comm, and the
    // unit (with both) is destroyed as a whole.
    const udp_hand_driver::CanFdHandComm* canfd_comm{nullptr};

    // #945 D63: whether SaveHandStats already warned that `cmd_ack` disagrees
    // with the firmware (silent firmware under echo / echoes under none) — each
    // once per run, like the counters it reads. Written and read only under
    // aux_lane_mutex_ (in the const save, hence mutable); cleared by
    // on_activate with has_run_.
    mutable bool warned_cmd_ack_silent{false};
    mutable bool warned_cmd_ack_echoes{false};

    // D39: JointCommand messages whose feedforward size matched neither 0 nor
    // values.size() (feedforward zeroed, position applied), and messages whose
    // feedforward carried a NaN / ±inf (that entry zeroed — a non-finite current
    // command must never reach the wire; the position is still applied). Per
    // run: reset in on_activate beside the controller's own counters. Written on
    // the command callback thread, read by the save on the aux lane → atomic.
    std::atomic<std::uint64_t> feedforward_size_mismatch{0};
    std::atomic<std::uint64_t> feedforward_nonfinite{0};

    // Data publishers — LifecyclePublisher (gated by lifecycle state).
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr motor_state_pub;
    rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::HandSensorState>::SharedPtr
        sensor_state_pub;
    rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::HandSensorState>::SharedPtr
        sensor_monitor_pub;
    rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::CalibrationStatus>::SharedPtr
        calib_status_pub;
    // Link status — standalone rclcpp::Publisher (NOT LifecyclePublisher).
    // Safety-relevant: must remain publishable in any lifecycle state.
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr link_status_pub;
    rclcpp::Subscription<rtc_msgs::msg::JointCommand>::SharedPtr joint_command_sub;
    rclcpp::Subscription<rtc_msgs::msg::CalibrationCommand>::SharedPtr calib_cmd_sub;

    // NRT publish lane: the last *completed* SeqLock write consumed (always
    // even). Baselined in on_configure, where the controller is new and its
    // sequence restarts at 0.
    std::uint32_t last_published_seq{0};
    std::vector<std::string> joint_names;
    std::vector<std::string> motor_names;
    std::vector<std::string> fingertip_names;

    // Per-joint position offset (URDF/controller frame − firmware zero), radians.
    // Parsed once in on_configure from `joint_position_offsets_deg` (degrees,
    // joint_state_names order). Applied at the two ROS boundaries only:
    //   read/publish : controller_pos = udp_pos + offset  (PublishState)
    //   command/write: udp_cmd        = controller_cmd − offset  (command sub)
    // so the round-trip is identity. Zero by default → backward compatible.
    // Position-only: velocity/effort and motor-space positions are untouched.
    std::array<float, udp_hand_driver::kNumHandMotors> joint_offset_rad{};

    // Firmware-slot ← command-index map for incoming joint_command. The
    // publisher (controller/backend) labels the message in its own joint_names
    // order (URDF / _base.yaml) and — per the "receiver reorders by name"
    // convention that the backend already applies to joint_states — this node
    // remaps each value into firmware slot order (joint_names = joint_state_names,
    // the wire order) before the positional CommLoop write. Built once from the
    // first message that carries joint_names; -1 / no-names ⇒ positional
    // fallback. Callback-thread only (no cross-thread access), so no atomic.
    std::array<int, udp_hand_driver::kNumHandMotors> cmd_name_reorder{};
    bool cmd_name_reorder_ready{false};

    bool prev_link_ok{true};

    // Pre-allocated messages (PreallocateMessages).
    sensor_msgs::msg::JointState joint_js_msg;
    sensor_msgs::msg::JointState motor_js_msg;
    rtc_msgs::msg::HandSensorState sensor_msg;
    std_msgs::msg::Bool link_msg;

    // Link-status and cycle-log decimation. Same type so both feed
    // AdvanceDecimation; both carry modulo their interval, so the value range is
    // bounded by that interval and never approaches the int range.
    int link_cycle_counter{0};
    int publish_count{0};
  };

  // Drain the loop's timing producer into the CSV (1 Hz, non-RT).
  // Runs on the `hand_aux_io` executor thread via cb_group_aux_ (issue #345),
  // NOT on the default executor thread — the burst is a blocking file write.
  // Deliberately not noexcept: the body takes aux_lane_mutex_, and
  // std::mutex::lock may throw std::system_error — under noexcept that would be
  // std::terminate() instead of a catchable error.
  void DrainHandTiming();

  // Pre-allocate one hand's ROS2 messages once in on_configure (non-RT).
  // Avoids dynamic allocation on the publish path.
  void PreallocateMessages(HandUnit& unit);

  // NRT publish lane (issue #345). Runs on the default executor thread, driven
  // by publish_timer_: reads each hand's SeqLock sequence and publishes the
  // latest snapshot when it has advanced. Nothing here runs on the RT thread.
  void PollAndPublish();

  // Publish one snapshot of `unit`. `cycles` is how many comm cycles the
  // snapshot advanced by — 1 in the common case, more when polling lost a
  // race, 0 never. Rate-derived decimation accumulates it instead of counting
  // invocations, because "one invocation == one comm cycle" stopped being true
  // with the pull model. Uses pre-allocated messages.
  void PublishState(HandUnit& unit, const udp_hand_driver::UdpHandState& state,
                    const udp_hand_driver::FingertipFTState& ft_state, std::uint32_t cycles);

  void PublishCalibrationStatus();

  // Persist comm/timing stats to <session>/device/hand_stats<suffix>.json, one
  // file per hand. Three concurrent call contexts, serialized by
  // aux_lane_mutex_:
  //   1. the periodic timer, on the `hand_aux_io` thread (cb_group_aux_)
  //   2. a failure-detector thread, from its failure callback (that hand only)
  //   3. lifecycle teardown (on_deactivate / on_cleanup / on_error / ~UdpHandNode),
  //      on the default executor thread
  // The lane split moved (1) off the default executor thread, so (1) and (3) are
  // genuinely concurrent rather than serialized by being the same thread.
  // verbose=false (periodic saves) skips the INFO summary logs.
  // Defined in udp_hand_node_stats.cpp.
  void SaveCommStats(bool verbose = true) const;
  void SaveCommStats(const HandUnit& unit, bool verbose) const;
  // The body of both: caller holds aux_lane_mutex_ and has checked has_run_.
  void SaveHandStatsLocked(const HandUnit& unit, bool verbose) const;

  // Cancel timers and stop the worker threads (failure detectors + comm loop),
  // leaving the hands constructed for possible reactivation. Idempotent;
  // shared by on_deactivate and on_error.
  void StopRuntime();

  // Release all publisher/subscription/timer handles and the hands.
  // Idempotent; shared by on_cleanup and on_error.
  void ReleaseResources();

  // Guards every aux-lane callback body (stats JSON save, timing CSV drain) and
  // is taken empty by StopRuntime() as a quiesce barrier. Before the lane split
  // these callbacks and lifecycle teardown were the same thread and could not
  // overlap; now they can, and they share the hands and the SPSC timing ring
  // (single-consumer). One mutex rather than two: neither callback calls the
  // other, so there is no lock order to get wrong, and both are non-RT (issue #345).
  //
  // Declared before every member it guards on purpose: members are destroyed in
  // reverse declaration order, so a mutex declared after units_ / the aux
  // timers would be destroyed while they still exist — an aux callback racing
  // ~UdpHandNode would then lock a destroyed mutex rather than merely observe a
  // half-torn-down node.
  mutable std::mutex aux_lane_mutex_;

  // E-Stop latch (E-8), shared by every hand (D70). Set to true by the
  // estop_topic subscription callback (executor thread), read by the CommLoop
  // at the head of every hand's cycle and between hands, through the pointer
  // handed over with SetEstopFlag() in on_configure; cleared by on_activate
  // right before the hands start so every run begins untripped — a `true`
  // received while Inactive has no loop to stop and must not become a
  // zero-write on the next activate before the first state read (the startup
  // no-jump contract). Declared before the loop and the hands: the loop
  // dereferences it until it is joined, so the flag has to outlive both
  // (members are destroyed in reverse declaration order).
  std::atomic<bool> estop_flag_{false};

  // The process's one comm thread (`hand_comm`, SCHED_FIFO 65 — epic #906
  // D69). Every hand is attached to it in on_configure and it is started in
  // on_activate once every hand has started. Declared after the latch it reads
  // and before units_: a controller's destructor joins it (Stop()), so it has
  // to outlive the controllers, and its thread dereferences them, so
  // StopRuntime() joins it before any of them goes.
  udp_hand_driver::HandCommLoop comm_loop_;
  // loop_rate_hz as configured — the loop's frequency at on_activate.
  double loop_rate_hz_{500.0};

  // The hands of this configuration, in tick order (hand_sides order; one
  // entry in the hand_side mode).
  std::vector<std::unique_ptr<HandUnit>> units_;

  // Whether the current configuration has run (#927): set by on_activate once
  // every hand started, cleared by on_configure. The stats JSON describes a
  // run, so a teardown save without one — configure → cleanup on a node that
  // already ran — would overwrite that run's JSON with an empty, clean report;
  // the save skips it. Guarded by aux_lane_mutex_.
  bool has_run_{false};

  // File suffix of the timing CSV: the one-hand mode follows that hand's
  // stats suffix (D51); under hand_sides it is empty — one loop, one CSV (D72).
  // Fixed in on_configure before any timer runs; read in on_activate.
  std::string timing_file_suffix_;

  // The hands the forensic files are named for ("right", or "right,left"
  // under hand_sides, which a "[…]" marks). Follows each configure until the
  // timing CSV opens (once per process, hand_timing_opened_); from then on a
  // configure with other hands is refused, or one run's artifacts would split
  // across two names. Executor thread only.
  std::string artifact_hands_;

  // E-Stop input, one for every hand (D70).
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr calib_status_timer_;

  // NRT publish lane (issue #345). Default callback group, so this runs on the
  // main executor thread — the same hand_driver core the CommLoop is on, which
  // is fine because that lane is light once the file I/O left for hand_aux_io.
  // One timer serves every hand.
  rclcpp::TimerBase::SharedPtr publish_timer_;
  int num_fingertips_{udp_hand_driver::kDefaultNumFingertips};

  // Sensor-message layout selector, cached from the SensorProtocol capability
  // at on_configure (off the hot path). true → publish the 1b per-fingertip
  // force vector (fs.f) from state.sensor_force; false → publish the 1a
  // barometer/ToF + F/T-inference payload. The publish branch reads this bool,
  // never a version string (ARCH-3).
  bool sensor_uses_force_layout_{false};

  bool ft_enabled_{false};
  // Per-channel link-down thresholds (strict, #1). Computed once in on_configure
  // from link_fail_timeout_ms at each channel's effective attempt rate; the SSoT
  // the detector cfg + publish + stats all read (via UdpHandController::LinkDown)
  // so the E-STOP decision and the link_status Bool cannot disagree. comm =
  // motor/joint (every comm cycle); sensor = the shorter sensor-rate budget.
  // Shared: every hand runs at the loop's rate.
  uint64_t link_fail_threshold_{10};
  uint64_t link_fail_threshold_sensor_{10};

  // Link status decimation (loop cycles per link_status publish).
  int link_decimation_{5};

  // Fake-hand standalone support. The CommLoop RT thread self-clocks and runs
  // the LPF model in fake mode (UdpHandController::RunFakeCommCycle), so the
  // node needs no self-tick timer or command cache — commands flow through the
  // normal command sub → SendCommandAndRequestStates path.
  bool use_fake_hand_{false};

  // Periodic stats persistence (crash-safety): without it the stats JSON is
  // written only on graceful teardown and is lost on SIGKILL/crash. Interval
  // is a fixed constant — see on_activate (no ROS param on purpose).
  rclcpp::TimerBase::SharedPtr stats_save_timer_;

  // Cached config for on_activate logging and the stats JSON.
  std::string comm_mode_str_;
  std::string protocol_version_;

  // ── Per-tick timing CSV (mpc_timing_log pattern) ───────────────────────
  // Producer (filled on the CommLoop thread) → 1 Hz drain on aux timer →
  // rtc::ThreadTimingCsvLogger writes one row per tick to
  // <session>/timing/hand_timing_log.csv. Open() runs once on the first
  // on_activate and is gated by `hand_timing_opened_` so reactivation does
  // not truncate or re-write the header. The *timer* has a shorter lifetime than
  // the logger — StopRuntime() cancels it to quiesce the aux lane and every
  // on_activate recreates it — so the two are separate members (issue #345).
  rtc::HandTimingBuffer hand_timing_producer_;
  udp_hand_driver::UdpHandTimingLogger hand_timing_logger_;
  rclcpp::TimerBase::SharedPtr hand_timing_timer_;
  bool hand_timing_opened_{false};
  std::uint64_t hand_timing_drop_baseline_{0};

  // Aux lane (issue #345). Created in the constructor, NOT on_configure: the
  // group is handed to main()'s aux executor before the first lifecycle
  // transition, and it must survive an on_cleanup -> on_configure cycle. A group
  // recreated in on_configure would be a *different* object that the executor
  // never saw, and the aux timers would silently stop firing.
  rclcpp::CallbackGroup::SharedPtr cb_group_aux_;
};
