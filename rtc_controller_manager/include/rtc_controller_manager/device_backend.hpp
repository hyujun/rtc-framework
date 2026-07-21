#ifndef RTC_CONTROLLER_MANAGER_DEVICE_BACKEND_H_
#define RTC_CONTROLLER_MANAGER_DEVICE_BACKEND_H_

#include "rtc_base/threading/publish_buffer.hpp"
#include "rtc_base/types/types.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"

#include <rclcpp/callback_group.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <chrono>
#include <functional>
#include <string>
#include <vector>

namespace rtc {

// ── DeviceBackendConfig ──────────────────────────────────────────────────────
//
// Plain-data view of the YAML `devices.<group>.backend:` block. CM parses YAML
// once at configure time and hands a DeviceBackendConfig to each backend's
// Configure(). All keys are staging strings — backend implementations interpret
// only the keys they need; unknown keys are ignored (forward-compatible).
//
// Examples (topic strings are robot-specific YAML values; the rtc_* packages
// stay robot-agnostic by treating these as opaque):
//   type = "mujoco_native"
//     state_topic   = "/<arm>/joint_states"
//     command_topic = "/<arm>/joint_command"
//
//   type = "ur_driver_native"
//     state_topic         = "/joint_states"
//     command_topic       = "/forward_position_controller/commands"
//     joint_command_names = ["<joint_0>", ...]            // packing order
//
//   type = "udp_hand_native"
//     state_topic   = "/<hand>/joint_states"
//     command_topic = "/<hand>/joint_command"
//     motor_topic   = "/<hand>/motor_states"
//     sensor_topic  = "/<hand>/sensor_states"
//     sensor_layout = { ... }  // resolved separately via DeviceSensorLayout
struct DeviceBackendConfig {
  std::string group_name;     ///< owning device-group key
  std::string type;           ///< type tag matching a registered backend
  std::string state_topic;    ///< joint-state subscription
  std::string command_topic;  ///< joint-command publication
  /// Output ordering (optional). Ordering contract: `slot.commands` handed to
  /// WriteCommand and the published JointCommand are ALWAYS in this order —
  /// backends direct-copy, never reorder on the command side. Wire-order
  /// differences are absorbed by the receiver (udp_hand_node /
  /// mujoco_simulator_node), which reorders by the message's `joint_names`.
  std::vector<std::string> joint_command_names;
  std::string motor_topic;   ///< optional motor-lane state topic
  std::string sensor_topic;  ///< optional sensor-lane state topic
  // Sensor layout is resolved by CM (DeviceSensorLayout) and forwarded to the
  // backend via a separate setter to avoid pulling rtc_base into this header.
};

// ── DeviceBackend ────────────────────────────────────────────────────────────
//
// Abstract HW/sim adapter. One instance per device group. Owns its ROS 2
// subscriptions/publishers (created on the supplied LifecycleNode) and the
// reorder/packing logic specific to that backend.
//
// Lifecycle (paired with the LifecycleNode that hosts it):
//   - Configure()   — called in node on_configure; create subs/pubs.
//   - Activate()    — called in node on_activate; enable lifecycle publishers.
//   - Deactivate()  — called in node on_deactivate; disable publishers.
//
// RT contract:
//   - ReadState() and WriteCommand() run on the RT path. Implementations must
//     follow rtc invariants (no heap, no logging, no mutex). State reads come
//     from a SeqLock written by the sensor-callback group; command writes push
//     into the SPSC publish buffer (handled by CM, not the backend).
//   - The backend itself does NOT touch the SPSC buffer. CM remains the sole
//     producer onto the publish thread; the backend only translates between
//     `DeviceStateCache` <-> wire-format messages.
//
// View type (per Phase 1 decision, option (a)):
//   - `DeviceStateCache` is reused directly. It is trivially copyable and POD,
//     so backends can fill it without extra mapping layers.
class DeviceBackend {
 public:
  virtual ~DeviceBackend() = default;

  /// One-shot configure. Backend creates its subscriptions/publishers on
  /// `node` and stashes the config for later use. Must be safe to call on the
  /// main thread; not RT.
  ///
  /// `state_cb_group` is the callback group that ALL state-lane subscriptions
  /// (joint / motor / sensor) created by the backend must be attached to via
  /// `rclcpp::SubscriptionOptions::callback_group`. CM passes its
  /// `cb_group_rt_callback_` (RT path, FIFO 70 executor on Core 2 — layout
  /// v4.1) so the controller↔hardware/sim boundary subs are dispatched on the
  /// rt_callback thread. May be null in test fixtures that don't wire an
  /// executor — backends must tolerate this and fall back to the default
  /// callback group.
  ///
  /// The supplied group MUST be `MutuallyExclusive` (CM creates it as such).
  /// Each backend writes into its own `SeqLock<DeviceStateCache>` from the
  /// state callback — the single-writer invariant relies on the callback
  /// group serializing concurrent callbacks. A `Reentrant` group would break
  /// it; do not substitute one.
  virtual void Configure(rclcpp_lifecycle::LifecycleNode* node, const DeviceBackendConfig& config,
                         rclcpp::CallbackGroup::SharedPtr state_cb_group) = 0;

  /// Enable lifecycle publishers. Not RT.
  virtual void Activate() = 0;

  /// Disable lifecycle publishers. Not RT.
  virtual void Deactivate() = 0;

  /// RT-safe state read. Backend copies the latest decoded state into `cache`.
  /// Returns true when fresh data was written. Never blocks; never allocates.
  [[nodiscard]] virtual bool ReadState(DeviceStateCache& cache) noexcept = 0;

  /// RT-safe command write. Backend encodes the controller's per-group output
  /// slot and hands the encoded message to its publisher path. `command_type`
  /// comes from the per-group GroupCommandSlot::command_type (kPosition /
  /// kTorque / kPdFeedforward) and lets backends emit JointCommand.command_type
  /// or pick a different ros2_control chain if needed. Never blocks; never
  /// allocates.
  virtual void WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                            CommandType command_type) noexcept = 0;

  /// True when this backend can actually honour `ct` on the wire.
  ///
  /// The default is position-only, which is the honest floor: a backend that
  /// says nothing is assumed to do the one thing every actuator lane in this
  /// tree does. Declaring more is opt-in and means the wire format carries the
  /// distinction — not merely that WriteCommand accepts the enum.
  ///
  /// This exists because `WriteCommand`'s `command_type` argument was
  /// advisory. `ur_driver_native` ignores it outright and publishes every
  /// value into a `forward_position_controller` Float64MultiArray, so a
  /// torque-mode controller bound to it would have sent newton-metres as
  /// joint angles — and CM's own hold command, which uses 0.0 for torque,
  /// would have commanded the arm to its zero configuration instead of
  /// releasing it. The backend's source claimed the pairing was "validated at
  /// YAML time"; nothing validated it anywhere. CM now checks this at
  /// configure and refuses the mismatch (ValidateCommandTypeSupport).
  ///
  /// Not RT — called once during configure.
  [[nodiscard]] virtual bool AcceptsCommandType(CommandType ct) const noexcept {
    return ct == CommandType::kPosition;
  }

  /// Emit the safest command this backend can produce **from its own state**,
  /// with no controller output to work from. RT-safe: never blocks, never
  /// allocates.
  ///
  /// CM calls this when it has decided the actuator must not receive a normal
  /// command AND it cannot build a hold either — the device reported a
  /// non-finite position, so "servo to where you are" has no value to servo
  /// to. Previously CM emitted a zero-length command there and treated that
  /// as fail-closed. It is not: every backend in this tree early-returns on a
  /// zero-length slot, so "CM sent nothing" reaches the hardware as "keep
  /// doing what you were doing, motors still driven". Silence is not a safe
  /// state; it is the absence of a decision.
  ///
  /// Deliberately takes no arguments. The whole point is that CM's view of
  /// the device is untrustworthy at this moment — the backend is the only
  /// party left holding a value it knows to be good (typically the last
  /// command it successfully published). It is also why there is no
  /// SafetyMode enum: the safest action is backend-specific and the backend
  /// owns it. A backend that gains a real drive-disable implements it here;
  /// nothing above needs to learn a new mode for that.
  ///
  /// Pure virtual on purpose. A default that did nothing would reproduce
  /// exactly the silence this exists to replace, and a new backend author
  /// would never be asked the question.
  ///
  /// Honest scope note: on all three backends shipped today the only
  /// available answer is "re-publish the last good command", which leaves the
  /// device physically where the silent path left it. What changes is that
  /// the intent is explicit on the wire and in a rosbag, the actuator lane
  /// stays live for any receiver that watches it, and the contract exists
  /// before a backend that can do better arrives (ARCH-3).
  virtual void WriteSafeCommand() noexcept = 0;

  // ── Optional capabilities (defaults: not provided) ────────────────────────

  /// True when this backend exposes a motor-space lane separate from joint
  /// space (e.g. udp_hand_native).
  [[nodiscard]] virtual bool HasMotorState() const noexcept { return false; }

  /// True when this backend exposes a packed sensor lane (e.g. udp_hand_native
  /// hand-sensor packets).
  [[nodiscard]] virtual bool HasSensorState() const noexcept { return false; }

  /// RT-safe motor-state read. Default no-op; only meaningful when
  /// HasMotorState() is true.
  virtual void ReadMotorState(DeviceStateCache& /*cache*/) noexcept {}

  /// RT-safe sensor-state read. Default no-op; only meaningful when
  /// HasSensorState() is true.
  virtual void ReadSensorState(DeviceStateCache& /*cache*/) noexcept {}

  /// Last time a state message arrived on this backend (steady clock). Used
  /// by CM's E-STOP watchdog. RT-safe.
  ///
  /// This is the single source of truth for per-device liveness, on both the
  /// startup gate and the steady-state watchdog (issue #198 §1/§2). Two
  /// properties are load-bearing for that:
  ///
  ///   - A default-constructed time_point means "no state has ever arrived".
  ///     CM refuses to run the control law until every configured device has
  ///     reported at least once, so a backend that returns a synthetic
  ///     non-epoch stamp before its first message would re-open exactly the
  ///     fail-open this contract exists to close.
  ///   - The stamp must be published (release) BEFORE the state-ready callback
  ///     fires, so a CM reader that observes the notification also observes the
  ///     stamp. It is read from the RT thread while the state lane writes it,
  ///     so the backing store must be atomic — the plain `time_point` CM used
  ///     to keep alongside this was a data race, which is why it is gone.
  [[nodiscard]] virtual std::chrono::steady_clock::time_point LastStateStamp() const noexcept = 0;

  /// Forward the per-group sensor packing layout resolved by CM. Default
  /// no-op; backends that consume a packed sensor lane (udp_hand_native)
  /// override to stash the layout before Configure().
  virtual void SetSensorLayout(const DeviceSensorLayout& /*layout*/) noexcept {}

  /// Callback fired by the backend on every fresh state arrival (joint /
  /// motor / sensor lanes — all routed through the same hook). Slot identity
  /// is supplied via lambda capture by the caller.
  ///
  /// It runs on the callback group handed to Configure() — CM passes its
  /// `cb_group_rt_callback_` (SCHED_FIFO 70), so this is the RT boundary and
  /// CM's handler is mailbox-only: it raises a bit and writes an eventfd,
  /// nothing more (issue #198 Phase 2). Backends must not assume the handler
  /// does any work of its own; in particular liveness comes from
  /// LastStateStamp() above, not from anything this callback records, so a
  /// backend that fires the callback without first publishing its stamp
  /// leaves CM's gate shut.
  ///
  /// Must be set before Configure(); default no-op for tests / fixtures that
  /// don't wire the hook.
  using StateReadyCallback = std::function<void()>;

  void SetStateReadyCallback(StateReadyCallback callback) noexcept {
    state_ready_cb_ = std::move(callback);
  }

 protected:
  /// Backends call this at the end of each sub callback (joint / motor /
  /// sensor lane). Safe to call when no callback is registered.
  void NotifyStateReady() noexcept {
    if (state_ready_cb_)
      state_ready_cb_();
  }

 private:
  StateReadyCallback state_ready_cb_;
};

}  // namespace rtc

#endif  // RTC_CONTROLLER_MANAGER_DEVICE_BACKEND_H_
