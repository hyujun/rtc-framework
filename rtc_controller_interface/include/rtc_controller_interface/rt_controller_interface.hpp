#ifndef RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_
#define RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_

// Shared types (constants, data structs) live in rtc_base.
// This header re-exports them and adds the abstract Strategy interface.
#include "rtc_base/threading/publish_buffer.hpp"
#include "rtc_base/types/types.hpp"
#include <rtc_msgs/msg/robot_target.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <yaml-cpp/yaml.h>

#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

// Forward declaration — full definition only needed in .cpp
namespace rtc_urdf_bridge {
struct ModelConfig;
class PinocchioModelBuilder;
}  // namespace rtc_urdf_bridge

namespace rtc {

// ── Ingress target limit check ────────────────────────────────────────────
//
// Result of screening one external joint-goal message against a device's
// configured position limits. Purely descriptive: the check never rejects nor
// modifies a command — the RT-path ClampRange in each controller's
// WriteJointCommand remains the sole enforcement. This exists so an operator
// sees *why* a goal did not land where they asked, instead of the command
// being silently absorbed by that clamp.
//
// `count` aggregates every violating joint, but only the first is described
// (first_idx/value/lower/upper) so the warning can be formatted with plain
// scalars — DeliverTargetMessage is noexcept, and building a per-joint string
// there risks a throwing allocation.
struct TargetLimitViolation {
  int count = 0;            ///< number of joints outside limits (0 → goal is clean)
  int first_idx = -1;       ///< index of the first violating joint, -1 when count==0
  double value = 0.0;       ///< commanded value at first_idx
  double lower = 0.0;       ///< configured lower bound at first_idx
  double upper = 0.0;       ///< configured upper bound at first_idx
  bool non_finite = false;  ///< true when the joint at first_idx is NaN/Inf
};

// Screens `ordered` (a joint goal already reordered into device-channel order)
// against `lim`. Pure: no ROS, no allocation, no logging — the logging wrapper
// is RTControllerInterface::WarnIfTargetOutOfLimits.
//
// Compares over min(ordered, position_lower, position_upper) entries so a
// short/ragged config can never read out of bounds. A non-finite command
// counts as a violation: std::clamp(NaN, lo, hi) returns NaN (both comparisons
// are false), so the RT clamp does not stop it — the operator must be told.
[[nodiscard]] TargetLimitViolation CheckTargetLimits(std::span<const double> ordered,
                                                     const DeviceJointLimits& lim) noexcept;

// ── Abstract interface (Strategy Pattern)
// ─────────────────────────────────────
//
// All virtual methods are noexcept to guarantee real-time safety: any
// exception thrown inside the RT timer would terminate the process.
class RTControllerInterface {
 public:
  // Polymorphic base — controllers are owned via
  // `std::vector<std::unique_ptr<RTControllerInterface>>` in
  // RtControllerNode and deleted through the base pointer. A non-virtual
  // dtor here makes that delete UB and silently skips derived destructors,
  // which leaks owned threads/buffers and was the root cause of the sim
  // shutdown SEGV in `mpc_main` (Pinocchio model torn down by base-only
  // teardown while the still-running solve thread dereferenced it).
  virtual ~RTControllerInterface();

  RTControllerInterface(const RTControllerInterface&) = delete;
  RTControllerInterface& operator=(const RTControllerInterface&) = delete;
  RTControllerInterface(RTControllerInterface&&) = delete;
  RTControllerInterface& operator=(RTControllerInterface&&) = delete;

  // Signature-equivalent to rclcpp_lifecycle so future inheritance migration
  // ("RTControllerInterface : public rclcpp_lifecycle::LifecycleNode") is a
  // near-mechanical change — see agent_docs/modification-guide.md.
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // ── Configuration state (fail-closed, issue #196 §1) ─────────────────────
  //
  // Before this existed, PreConfigure() stored node_ *before* LoadConfig() and
  // returned FAILURE on a parse error, but on_configure() then saw a non-null
  // node_, skipped re-parsing, and reported SUCCESS — so a controller whose
  // config never loaded still became an active RT Compute() candidate.
  //
  // The state is one-way into kFailed: once any configure step fails the
  // controller stays failed and every later on_configure() reports FAILURE.
  // Only on_cleanup() clears it back to kUnconfigured, which is the explicit
  // lifecycle path for a retry.
  enum class ConfigState {
    kUnconfigured,   // nothing stored yet, or cleaned up
    kPreConfigured,  // PreConfigure() succeeded; awaiting on_configure()
    kConfigured,     // on_configure() succeeded
    kFailed,         // a configure step failed — terminal until on_cleanup()
  };

  [[nodiscard]] ConfigState GetConfigState() const noexcept { return config_state_; }

  // ── Lifecycle hooks (ros2_control-aligned signatures) ────────────────────
  //
  // These are driven by RtControllerNode (CM) as direct C++ method calls;
  // the injected LifecycleNode is stored in node_ for each controller's own
  // sub/pub ownership.  All hooks are noexcept — RT-adjacent code (on_activate
  // arming a publish flag) must not throw.  Non-RT configuration that can
  // throw (YAML parsing, dynamic allocation) must be caught inside.
  //
  // Default contract:
  //   PreConfigure: store node_, invoke LoadConfig(yaml_cfg) in try/catch
  //                  → topic_config_ available; no resource allocation,
  //                    no RegisterLog, no lifecycle state change. CM uses
  //                    the result to build active_groups_ + device_name_configs_
  //                    and calls SetDeviceNameConfigs() before on_configure().
  //   on_configure:  store node_ (idempotent), LoadConfig (idempotent if
  //                  PreConfigure ran), then subclass work (RegisterLog,
  //                  publishers, parameter declares, …)
  //   on_activate:   no-op SUCCESS (controller-internal init policy — each
  //                    derived controller self-initialises its target slot on
  //                    the first Compute() tick using the current device state)
  //   on_deactivate: no-op SUCCESS
  //   on_cleanup:    release node_
  //   on_shutdown:   delegate to on_cleanup
  //   on_error:      no-op SUCCESS  (subclass may TriggerEstop() here)
  //
  // Overrides that add work MUST call the base implementation first (or
  // handle LoadConfig / node_ themselves).
  //
  // Hold-target initialisation is owned by the controller (not CM): on the
  // first Compute() tick after activation each controller seeds its target
  // slot from the current device state, eliminating the writer-multiplicity
  // race that auto-hold + on_activate + ROS sub callbacks used to create on
  // the legacy target_mutex_ path.
  // PreConfigure: lightweight first pass. Stores node_ and calls LoadConfig(yaml)
  // so that GetTopicConfig() returns valid data BEFORE CM resolves
  // device_name_configs_. Must precede on_configure() in the CM bring-up
  // sequence; CM also calls SetDeviceNameConfigs() between PreConfigure and
  // on_configure, so on_configure can rely on GetDeviceNameConfig(...) being
  // populated when it allocates resources / registers log channels.
  //
  // Subclasses must NOT override — base implementation suffices. A controller
  // never instantiated through CM (e.g. unit tests calling on_configure
  // directly) skips PreConfigure; on_configure's idempotent guard handles that
  // legacy path.
  CallbackReturn PreConfigure(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                              const YAML::Node& yaml_cfg) noexcept;

  virtual CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state,
                                      rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                      const YAML::Node& yaml_cfg) noexcept;

  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) noexcept;

  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) noexcept;

  virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) noexcept;

  virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) noexcept;

  virtual CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) noexcept;

  // Controller-owned LifecycleNode accessor.  Non-null after on_configure
  // succeeds; null after on_cleanup.  Intended for CM to add the node to an
  // executor (nrt_callback_executor) for non-RT callback processing.
  [[nodiscard]] rclcpp_lifecycle::LifecycleNode::SharedPtr get_lifecycle_node() const noexcept {
    return node_;
  }

  // Compute one control step. Must be noexcept for RT safety.
  [[nodiscard]] virtual ControllerOutput Compute(const ControllerState& state) noexcept = 0;

  virtual void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept = 0;

  // SetDeviceTaskTarget()
  //   Deliver a 6-DoF task-space (SE3) target for `device_idx` — used when the
  //   inbound RobotTarget has goal_type=="task". `task6` is (x,y,z,r,p,y).
  //   Default forwards to SetDeviceTarget, preserving the behaviour of existing
  //   task controllers (DemoTask already interprets its device-0 buffer as an
  //   SE3 pose). Controllers that keep joint and task targets in separate slots
  //   (e.g. DemoWbc: arm joint posture vs. commanded SE3) override this so a
  //   task target lands in the SE3 slot without clobbering the joint slot.
  //   Must be noexcept / RT-marshal-safe like SetDeviceTarget.
  virtual void SetDeviceTaskTarget(int device_idx, std::span<const double> task6) noexcept {
    SetDeviceTarget(device_idx, task6);
  }

  [[nodiscard]] virtual std::string_view Name() const noexcept = 0;

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
  //   YAML file).  Override to read per-controller structural config from
  //   disk (e.g. estop, fsm, topics, command_type).  Not noexcept — YAML
  //   parsing can throw; the call site wraps it in try/catch.
  //
  // Runtime-tunable gains have migrated to ROS 2 parameters declared on each
  // controller's own LifecycleNode (in on_configure). The CM no longer
  // routes a /<robot>/{controller_gains, request_gains, current_gains}
  // topic — BT calls SetActiveControllerGains directly against the active
  // controller's parameter services.
  //
  // Returns the type of command this controller outputs (position or torque).
  // Value is set by LoadConfig() from the YAML command_type field.
  [[nodiscard]] virtual CommandType GetCommandType() const noexcept {
    return CommandType::kPosition;
  }

  virtual void LoadConfig(const YAML::Node& cfg);

  // Note: per-thread observability CSVs (CM RT loop, MPC solve timing, ...)
  // are owned by the producing thread itself — see rtc_base/timing/
  // thread_timing_*.hpp for the generic infra. The base interface
  // intentionally has no domain-specific observability virtuals so adding
  // a new timing channel never requires touching this header.

  // GetTopicConfig()
  //   Returns the per-controller topic configuration (subscribe/publish
  //   topics). Populated by LoadConfig() from the YAML "topics" section. If no
  //   "topics" section exists, returns the default topic set.
  [[nodiscard]] const TopicConfig& GetTopicConfig() const noexcept { return topic_config_; }

  // PublishNonRtSnapshot()
  //   Called from the non-RT publish thread after CM has drained the SPSC
  //   snapshot and dispatched its fixed publishers. Override to publish
  //   controller-owned topics via publishers the controller created in
  //   on_configure/on_activate.
  //
  //   Group iteration order in `snap.group_commands` matches the controller's
  //   own `topic_config_.groups` order (set by the RT loop), so overrides can
  //   index by position.
  //
  //   Must be noexcept — any allocation / exception would stall the publish
  //   thread. Must not touch device_target_ or other RT-written state.
  virtual void PublishNonRtSnapshot(const PublishSnapshot& snap) noexcept { (void)snap; }

  // DeliverTargetMessage()
  //   Owns the target reorder/route/clip logic for controller-owned target
  //   subscriptions (formerly CM's manager-owned callback, removed in #138).
  //   Use from controller-owned target-topic subscription callbacks:
  //     - joint_target goal: reorders by msg.joint_names against
  //       device_name_configs_[group_name].joint_state_names;
  //     - task_target goal: forwarded as-is.
  //   Dispatches via SetDeviceTarget(device_idx, ordered_span). `device_idx`
  //   is the controller-local group index (position in topic_config_.groups).
  //   Controller's SetDeviceTarget marshals onto an SPSC queue drained by
  //   the RT thread in Compute(), so no off-RT writer touches the target
  //   slot directly.
  void DeliverTargetMessage(const std::string& group_name, int device_idx,
                            const rtc_msgs::msg::RobotTarget& msg) noexcept;

  // ── Device name configuration ──────────────────────────────────────────
  //   SetDeviceNameConfigs() is called by RtControllerNode after all
  //   controllers are constructed and device configs are loaded from YAML.
  //   After setting, OnDeviceConfigsSet() is called for controllers to
  //   resolve kinematics (e.g. end_id_ from tip_link).
  void SetDeviceNameConfigs(std::map<std::string, DeviceNameConfig> configs) {
    device_name_configs_ = std::move(configs);
    OnDeviceConfigsSet();
  }

  [[nodiscard]] const DeviceNameConfig* GetDeviceNameConfig(
      const std::string& device_name) const noexcept {
    auto it = device_name_configs_.find(device_name);
    return (it != device_name_configs_.end()) ? &it->second : nullptr;
  }

  // Convenience accessor — returns the sensor packing layout for `device_name`
  // (incl. capability flags `has_native_contact` / `has_native_displacement`).
  // Empty optional when the device is unknown OR when its YAML did not declare
  // a `sensor_layout` block (i.e. the device has no packed sensor lane).
  // Callers (controllers) typically read this once in OnConfigure/
  // OnDeviceConfigsSet and cache capability bools into members; the RT loop
  // must not depend on the optional object itself.
  [[nodiscard]] std::optional<DeviceSensorLayout> GetSensorLayout(
      const std::string& device_name) const noexcept {
    const auto* cfg = GetDeviceNameConfig(device_name);
    if (cfg == nullptr) {
      return std::nullopt;
    }
    return cfg->sensor_layout;
  }

  // Returns the name of the primary device (first group in topic config).
  // Use instead of hardcoding a robot-specific name to support arbitrary
  // robot configurations.
  [[nodiscard]] std::string GetPrimaryDeviceName() const noexcept {
    if (!topic_config_.groups.empty()) {
      return topic_config_.groups.front().first;
    }
    if (!device_name_configs_.empty()) {
      return device_name_configs_.begin()->first;
    }
    return {};
  }

  // Returns the name of the secondary device (second group in topic config).
  // Empty when topic_config_ has fewer than two groups — callers must handle
  // single-device controllers by null-checking GetDeviceNameConfig(...) on
  // the returned name.
  [[nodiscard]] std::string GetSecondaryDeviceName() const noexcept {
    if (topic_config_.groups.size() >= 2) {
      return topic_config_.groups[1].first;
    }
    return {};
  }

  // ── System model configuration ──────────────────────────────────────────
  //   SetSystemModelConfig() is called by RtControllerNode after controllers
  //   are constructed, passing the system-level URDF + model topology
  //   (sub_models, tree_models, passive_joints) parsed from the top-level
  //   "urdf:" YAML section.  Controllers can override OnSystemModelConfigSet()
  //   to build Pinocchio models from the shared config.
  void SetSystemModelConfig(const rtc_urdf_bridge::ModelConfig& config);
  [[nodiscard]] const rtc_urdf_bridge::ModelConfig* GetSystemModelConfig() const noexcept;

  // Optional shared PinocchioModelBuilder. RtControllerNode builds a single
  // PinocchioModelBuilder from the system URDF + sub/tree topology and shares
  // it with every controller via this setter, so that controllers can avoid
  // re-parsing the URDF and re-building the same Pinocchio models. Returns
  // null if no shared builder was injected; controllers must then build
  // their own from GetSystemModelConfig().
  void SetSharedModelBuilder(
      std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder) noexcept;
  [[nodiscard]] std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> GetSharedModelBuilder()
      const noexcept;

  // Set the control loop rate (Hz). Called by the manager at init time from
  // the YAML `control_rate` parameter. The framework is rate-agnostic
  // (typically kMinControlRateHz..kMaxControlRateHz); controllers must
  // derive any per-tick dt from this rate, never from a hard-coded constant.
  void SetControlRate(double hz) noexcept { control_rate_ = hz; }

  [[nodiscard]] double GetDefaultDt() const noexcept {
    // The fallback to kDefaultControlDtSec only fires if SetControlRate()
    // was never called — a misconfiguration. Normal callers always observe
    // 1/control_rate_ at the runtime-configured rate.
    return (control_rate_ > 0.0) ? (1.0 / control_rate_) : kDefaultControlDtSec;
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
  static TopicConfig ParseTopicConfig(const YAML::Node& topics_node);

  // ── L1: Device limits loader ───────────────────────────────────────────
  //   Populates per-device joint limit vectors from the resolved
  //   device_name_configs_, using `topic_config_.groups` order as the
  //   controller-local device index. Each output array slot `i` corresponds
  //   to the i-th group in topic_config_; slots without a matching device
  //   config (or with empty joint_limits) are filled with `default_*` of
  //   length kMaxDeviceChannels so RT-path clamping always has valid bounds.
  //
  //   Robot-agnostic: never references "ur5e" / "hand" / robot joint count.
  //   Call from OnDeviceConfigsSet() after CM has injected device configs.
  //
  //   Caller supplies fallback values explicitly — the base intentionally
  //   does not provide defaults so that each controller's safety envelope
  //   is visible at the call site (no hidden ±2π / 2 rad/s assumptions).
  void LoadDeviceLimitsFromConfig(
      std::array<std::vector<double>, ControllerState::kMaxDevices>& position_lower,
      std::array<std::vector<double>, ControllerState::kMaxDevices>& position_upper,
      std::array<std::vector<double>, ControllerState::kMaxDevices>& max_velocity,
      double default_lower, double default_upper, double default_velocity) const;

  // ── L2: E-STOP safe-position YAML parser ───────────────────────────────
  //   Extracts and validates a fixed-length sequence under
  //   `cfg["estop"]["arm_safe_position"]`. `expected_size` is the controller's
  //   own arm DOF (caller-supplied — base does not assume any specific count;
  //   6-DOF UR5/UR10, 7-DOF KUKA iiwa, etc. all valid).
  //
  //   `controller_name` is included in error messages so users can locate
  //   the offending YAML quickly.
  //
  //   Throws std::runtime_error on missing / wrong-type / wrong-length input.
  //   Non-throwing in the steady state — designed to be called from
  //   LoadConfig() inside the controller's existing try/catch.
  static std::vector<double> ParseArmSafePosition(const YAML::Node& cfg, std::size_t expected_size,
                                                  const std::string& controller_name);

  TopicConfig topic_config_;
  std::map<std::string, DeviceNameConfig> device_name_configs_;
  std::unique_ptr<rtc_urdf_bridge::ModelConfig> system_model_config_;
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> shared_model_builder_;
  // Configured RT loop rate [Hz]. Initialised to kDefaultControlRateHz so
  // that controllers used in unit tests (without CM) see a sensible dt; in
  // production CM always overrides this via SetControlRate() during
  // PreConfigure. The framework supports the full kMin..kMaxControlRateHz
  // range — never hard-code 500 Hz here or in derived classes.
  double control_rate_{kDefaultControlRateHz};

  // Controller-owned LifecycleNode injected by RtControllerNode in
  // on_configure.  Subclasses use `node_->create_subscription(...)` etc. for
  // their own ROS I/O.  Kept as a protected member (composition pattern) so a
  // future migration to "RTControllerInterface : public LifecycleNode" is a
  // mechanical `node_->` → `this->` replacement.
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

 private:
  // Screens an incoming joint goal against `group_name`'s configured position
  // limits and emits one throttled WARN describing the violation. Never alters
  // the command — see TargetLimitViolation. Silent when the device declares no
  // joint_limits: LoadDeviceLimitsFromConfig's ±2π fallback is a placeholder,
  // not a real envelope, so warning against it would be pure false-positive.
  //
  // Non-RT: the caller (DeliverTargetMessage) runs on the LifecycleNode's
  // default callback group → nrt_callback_executor, an invariant locked by
  // integrated_bringup/test/test_controller_target_cb_group_invariant.cpp.
  // A plain RCLCPP_WARN_THROTTLE is therefore legal here (no RT-3 SPSC defer).
  void WarnIfTargetOutOfLimits(const std::string& group_name,
                               std::span<const double> ordered) const noexcept;

  // Rolls back base-owned configure state (node_, topic_config_) and latches
  // kFailed. Called from every configure failure path.
  void FailConfigure() noexcept;

  // Private, not protected: the fail-closed guarantee only holds if the base
  // is the sole writer. Derived classes read it through GetConfigState().
  // Written on the configure/cleanup path only (single-threaded bring-up),
  // never from the RT thread — no synchronisation needed.
  ConfigState config_state_{ConfigState::kUnconfigured};
};

}  // namespace rtc

#endif  // RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_
