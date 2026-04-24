#ifndef RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_
#define RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_

// Shared types (constants, data structs) live in rtc_base.
// This header re-exports them and adds the abstract Strategy interface.
#include "rtc_base/threading/publish_buffer.hpp"
#include "rtc_base/timing/mpc_solve_stats.hpp"
#include "rtc_base/types/types.hpp"

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <yaml-cpp/yaml.h>

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

// Forward declaration — full definition only needed in .cpp
namespace rtc_urdf_bridge {
struct ModelConfig;
}

namespace rtc {

// ── Abstract interface (Strategy Pattern)
// ─────────────────────────────────────
//
// All virtual methods are noexcept to guarantee real-time safety: any
// exception thrown inside a 500 Hz timer would terminate the process.
class RTControllerInterface {
public:
  ~RTControllerInterface();

  RTControllerInterface(const RTControllerInterface &) = delete;
  RTControllerInterface &operator=(const RTControllerInterface &) = delete;
  RTControllerInterface(RTControllerInterface &&) = delete;
  RTControllerInterface &operator=(RTControllerInterface &&) = delete;

  // Signature-equivalent to rclcpp_lifecycle so future inheritance migration
  // ("RTControllerInterface : public rclcpp_lifecycle::LifecycleNode") is a
  // near-mechanical change — see agent_docs/modification-guide.md.
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // ── Lifecycle hooks (ros2_control-aligned signatures) ────────────────────
  //
  // These are driven by RtControllerNode (CM) as direct C++ method calls;
  // the injected LifecycleNode is stored in node_ for each controller's own
  // sub/pub ownership.  All hooks are noexcept — RT-adjacent code (on_activate
  // arming a publish flag) must not throw.  Non-RT configuration that can
  // throw (YAML parsing, dynamic allocation) must be caught inside.
  //
  // Default contract:
  //   on_configure:  store node_, invoke LoadConfig(yaml_cfg) in try/catch
  //   on_activate:   no-op SUCCESS
  //   on_deactivate: no-op SUCCESS
  //   on_cleanup:    release node_
  //   on_shutdown:   delegate to on_cleanup
  //   on_error:      no-op SUCCESS  (subclass may TriggerEstop() here)
  //
  // Overrides that add work MUST call the base implementation first (or
  // handle LoadConfig / node_ themselves).
  virtual CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state,
               rclcpp_lifecycle::LifecycleNode::SharedPtr node,
               const YAML::Node &yaml_cfg) noexcept;

  virtual CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) noexcept;

  virtual CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) noexcept;

  virtual CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) noexcept;

  virtual CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) noexcept;

  virtual CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) noexcept;

  // Controller-owned LifecycleNode accessor.  Non-null after on_configure
  // succeeds; null after on_cleanup.  Intended for CM to add the node to an
  // executor (aux_executor) for non-RT callback processing.
  [[nodiscard]] rclcpp_lifecycle::LifecycleNode::SharedPtr
  get_lifecycle_node() const noexcept {
    return node_;
  }

  // Compute one control step. Must be noexcept for RT safety.
  [[nodiscard]] virtual ControllerOutput
  Compute(const ControllerState &state) noexcept = 0;

  virtual void SetDeviceTarget(int device_idx,
                               std::span<const double> target) noexcept = 0;

  [[nodiscard]] virtual std::string_view Name() const noexcept = 0;

  // Auto-hold: initialise target from current state so the robot holds
  // its position when no external goal has been received.
  // Each controller implements this according to its own target format
  // (joint-space, task-space, etc.).  Called once from ControlLoop Phase 0
  // when state_received_ is true but target_received_ is false.
  virtual void
  InitializeHoldPosition(const ControllerState &state) noexcept = 0;

  // E-STOP interface — default no-ops for controllers that do not need it.
  virtual void TriggerEstop() noexcept {}
  virtual void ClearEstop() noexcept {}
  [[nodiscard]] virtual bool IsEstopped() const noexcept { return false; }
  virtual void SetHandEstop(bool /*enabled*/) noexcept {}

  // ── Extensibility hooks for the controller registry ──────────────────────
  //
  // LoadConfig()
  //   Called once at node startup.  `cfg` is the YAML node already scoped to
  //   this controller's key (e.g. the content under `pd_controller:` in its
  //   YAML file).  Override to read per-controller gains / flags from disk.
  //   Not noexcept — YAML parsing can throw; the call site wraps it in
  //   try/catch.
  //
  // UpdateGainsFromMsg()
  //   Called from the ~/controller_gains subscriber (sensor thread).
  //   `gains` is a flat array whose layout is controller-specific:
  //   document the layout in the controller's header.  Default is a no-op.
  // Returns the type of command this controller outputs (position or torque).
  // Value is set by LoadConfig() from the YAML command_type field.
  [[nodiscard]] virtual CommandType GetCommandType() const noexcept {
    return CommandType::kPosition;
  }

  virtual void LoadConfig(const YAML::Node &cfg);
  virtual void UpdateGainsFromMsg(std::span<const double> gains) noexcept {
    (void)gains;
  }

  // GetCurrentGains()
  //   Returns the current gains as a flat array matching the layout expected
  //   by UpdateGainsFromMsg().  Used by the GUI "Load Gain" feature to read
  //   back the active controller's runtime gains.  Default returns empty.
  [[nodiscard]] virtual std::vector<double> GetCurrentGains() const noexcept {
    return {};
  }

  // GetMpcSolveStats()
  //   Observability hook — returns the most recent MPC solve-timing window
  //   if this controller runs an MPC loop, or std::nullopt otherwise.
  //   Non-RT: RtControllerNode polls this from the aux callback group at
  //   1 Hz for CSV logging and periodic INFO output. Controllers that do
  //   not own an MPCSolutionManager leave the default nullopt.
  [[nodiscard]] virtual std::optional<MpcSolveStats>
  GetMpcSolveStats() const noexcept {
    return std::nullopt;
  }

  // GetTopicConfig()
  //   Returns the per-controller topic configuration (subscribe/publish
  //   topics). Populated by LoadConfig() from the YAML "topics" section. If no
  //   "topics" section exists, returns the default topic set.
  [[nodiscard]] const TopicConfig &GetTopicConfig() const noexcept {
    return topic_config_;
  }

  // PublishNonRtSnapshot()
  //   Called from the non-RT publish thread after CM has drained the SPSC
  //   snapshot and dispatched manager-owned publishers. Override to publish
  //   controller-owned topics (ownership == TopicOwnership::kController) via
  //   publishers the controller created in on_configure/on_activate.
  //
  //   Group iteration order in `snap.group_commands` matches the controller's
  //   own `topic_config_.groups` order (set by the RT loop), so overrides can
  //   index by position.
  //
  //   Must be noexcept — any allocation / exception would stall the publish
  //   thread. Must not touch device_target_ or other RT-written state.
  virtual void PublishNonRtSnapshot(const PublishSnapshot &snap) noexcept {
    (void)snap;
  }

  // SetTargetReceivedNotifier()
  //   Injected by CM before on_configure. Controllers that move target
  //   subscriptions to their own LifecycleNode (ownership == kController)
  //   must invoke notifier_() in the subscription callback so CM's
  //   target_received_ gate flips exactly once, matching the legacy behavior
  //   where CM's own DeviceTargetCallback set the flag. No-op when unset.
  void SetTargetReceivedNotifier(std::function<void()> notifier) {
    target_received_notifier_ = std::move(notifier);
  }

  // ── Device name configuration ──────────────────────────────────────────
  //   SetDeviceNameConfigs() is called by RtControllerNode after all
  //   controllers are constructed and device configs are loaded from YAML.
  //   After setting, OnDeviceConfigsSet() is called for controllers to
  //   resolve kinematics (e.g. end_id_ from tip_link).
  void SetDeviceNameConfigs(std::map<std::string, DeviceNameConfig> configs) {
    device_name_configs_ = std::move(configs);
    OnDeviceConfigsSet();
  }

  [[nodiscard]] const DeviceNameConfig *
  GetDeviceNameConfig(const std::string &device_name) const noexcept {
    auto it = device_name_configs_.find(device_name);
    return (it != device_name_configs_.end()) ? &it->second : nullptr;
  }

  // Returns the name of the primary device (first group in topic config).
  // Use instead of hardcoding "ur5e" to support arbitrary robot names.
  [[nodiscard]] std::string GetPrimaryDeviceName() const noexcept {
    if (!topic_config_.groups.empty()) {
      return topic_config_.groups.front().first;
    }
    if (!device_name_configs_.empty()) {
      return device_name_configs_.begin()->first;
    }
    return {};
  }

  // ── System model configuration ──────────────────────────────────────────
  //   SetSystemModelConfig() is called by RtControllerNode after controllers
  //   are constructed, passing the system-level URDF + model topology
  //   (sub_models, tree_models, passive_joints) parsed from the top-level
  //   "urdf:" YAML section.  Controllers can override OnSystemModelConfigSet()
  //   to build Pinocchio models from the shared config.
  void SetSystemModelConfig(const rtc_urdf_bridge::ModelConfig &config);
  [[nodiscard]] const rtc_urdf_bridge::ModelConfig *
  GetSystemModelConfig() const noexcept;

  // Set the control loop rate (Hz). Called by the manager at init time.
  void SetControlRate(double hz) noexcept { control_rate_ = hz; }
  [[nodiscard]] double GetDefaultDt() const noexcept {
    return (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.002;
  }

protected:
  RTControllerInterface();

  // Called after SetDeviceNameConfigs(). Override to resolve URDF-based
  // kinematics (e.g. tip_link → end_id_).
  virtual void OnDeviceConfigsSet() {}

  // Called after SetSystemModelConfig(). Override to build Pinocchio models
  // from the system-level URDF + model topology (sub_models, tree_models).
  virtual void OnSystemModelConfigSet() {}

  // Parses the "topics" section of a controller YAML node.
  // Called by the base LoadConfig(); subclasses that override LoadConfig()
  // should call RTControllerInterface::LoadConfig(cfg) to inherit this.
  static TopicConfig ParseTopicConfig(const YAML::Node &topics_node);

  // Default topic configuration — device_name determines topic namespace.
  static TopicConfig
  MakeDefaultTopicConfig(const std::string &device_name = "ur5e");

  TopicConfig topic_config_;
  std::map<std::string, DeviceNameConfig> device_name_configs_;
  std::unique_ptr<rtc_urdf_bridge::ModelConfig> system_model_config_;
  double control_rate_{500.0};

  // Controller-owned LifecycleNode injected by RtControllerNode in
  // on_configure.  Subclasses use `node_->create_subscription(...)` etc. for
  // their own ROS I/O.  Kept as a protected member (composition pattern) so a
  // future migration to "RTControllerInterface : public LifecycleNode" is a
  // mechanical `node_->` → `this->` replacement.
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Invoked by subclasses (from controller-owned target subscription
  // callbacks) to signal CM that a target has been received. Safe to call
  // even when the notifier is unset — guarded internally.
  void NotifyTargetReceived() const {
    if (target_received_notifier_) {
      target_received_notifier_();
    }
  }

private:
  std::function<void()> target_received_notifier_;
};

} // namespace rtc

#endif // RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_
