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
// Examples:
//   type = "mujoco_native"
//     state_topic   = "/ur5e/joint_states"
//     command_topic = "/ur5e/joint_command"
//
//   type = "ur_driver_native"
//     state_topic         = "/joint_states"
//     command_topic       = "/forward_position_controller/commands"
//     joint_command_names = ["shoulder_pan_joint", ...]   // packing order
//
//   type = "udp_hand_native"
//     state_topic   = "/hand/joint_states"
//     command_topic = "/hand/joint_command"
//     motor_topic   = "/hand/motor_states"
//     sensor_topic  = "/hand/sensor_states"
//     sensor_layout = { ... }  // resolved separately via DeviceSensorLayout
struct DeviceBackendConfig {
  std::string group_name;                        ///< owning device-group key
  std::string type;                              ///< type tag matching a registered backend
  std::string state_topic;                       ///< joint-state subscription
  std::string command_topic;                     ///< joint-command publication
  std::vector<std::string> joint_command_names;  ///< output ordering (optional)
  std::string motor_topic;                       ///< optional motor-lane state topic
  std::string sensor_topic;                      ///< optional sensor-lane state topic
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
  /// `cb_group_rt_inbound_` (RT path, FIFO 70 executor) so the
  /// controller↔hardware/sim boundary subs are dispatched on the rt_inbound
  /// thread, matching the RT definition in
  /// `~/.claude/plans/arm-hand-core-allocation.md`. May be null in test
  /// fixtures that don't wire an executor — backends must tolerate this and
  /// fall back to the default callback group.
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
  /// comes from PublishSnapshot::command_type (kPosition / kTorque) and lets
  /// backends emit JointCommand.command_type or pick a different ros2_control
  /// chain if needed. Never blocks; never allocates.
  virtual void WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                            CommandType command_type) noexcept = 0;

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
  [[nodiscard]] virtual std::chrono::steady_clock::time_point LastStateStamp() const noexcept = 0;

  /// Forward the per-group sensor packing layout resolved by CM. Default
  /// no-op; backends that consume a packed sensor lane (udp_hand_native)
  /// override to stash the layout before Configure().
  virtual void SetSensorLayout(const DeviceSensorLayout& /*layout*/) noexcept {}

  /// Callback fired by the backend on every fresh state arrival (joint /
  /// motor / sensor lanes — all routed through the same hook). CM uses this
  /// to refresh the E-STOP watchdog timestamp, set `state_received_`,
  /// republish digital-twin joint states, and notify the sim-sync condvar.
  /// Slot identity is supplied via lambda capture by the caller.
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
