#ifndef RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_
#define RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_

// Shared types (constants, data structs) live in rtc_base.
// This header re-exports them and adds the abstract Strategy interface.
//
// device_readability.hpp carries the other half of the base's per-tick
// contract — the F5 gate that decides whether devices[0] may be used as this
// tick's joint state at all. It is included here rather than left to each
// binding so the gate is discoverable from the interface a binding already
// implements; see its header comment for why the two predicates it exports
// are deliberately named apart.
#include "rtc_base/concurrency/spsc_queue.hpp"
#include "rtc_base/threading/publish_buffer.hpp"
#include "rtc_base/types/types.hpp"
#include "rtc_controller_interface/device_readability.hpp"
#include <rtc_msgs/msg/robot_target.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <type_traits>
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

// ── Actuator-boundary output validation ───────────────────────────────────
//
// Why a controller's own output needs screening: ControllerOutput is an
// out-of-tree contract (any registry-loaded controller fills it), and the
// controller manager hands it straight to DeviceBackend::WriteCommand — the
// last software stage before a physical actuator. Nothing between Compute()
// and the wire vets it, so a NaN command or a channel count past what the
// device actually reported leaves the process as motion. This is the ingress
// check of CheckTargetLimits mirrored onto the egress side (issue #196
// Phase 4).
//
// Scope is deliberately narrow: only the fields that reach an actuator —
// `commands` and `feedforward`, bounded by `num_channels`. Everything else in
// ControllerOutput (task poses, trajectory references, goal positions) feeds
// log/GUI lanes where a bad value is visible but not dangerous, and screening
// them would cost RT budget for no safety gain.
enum class OutputRejectReason : uint8_t {
  kNone = 0,                ///< output is usable
  kInvalidFlag,             ///< ControllerOutput::valid == false
  kDeviceCountMismatch,     ///< num_devices disagrees with the state, or is out of range
  kChannelCountOutOfRange,  ///< a device's num_channels is negative, past capacity, or
                            ///< past what the device reported this tick
  kNonFiniteCommand,        ///< a command / feedforward value is NaN or Inf
};

// First rejection found, or kNone. Descriptive only — the caller decides the
// policy (the CM holds position and counts consecutive rejects). Plain
// scalars so the RT caller can format a deferred log line without allocating.
struct ControllerOutputValidation {
  OutputRejectReason reason{OutputRejectReason::kNone};
  int device_idx{-1};   ///< offending device, -1 when not device-specific
  int channel_idx{-1};  ///< offending channel, -1 when not channel-specific

  [[nodiscard]] constexpr bool Ok() const noexcept { return reason == OutputRejectReason::kNone; }
};

// Screens `out` against the `state` that produced it. Pure: no ROS, no
// allocation, no logging, no throw — safe to call on the RT tick path.
//
// `state` is the trust anchor. It was already bounded on the device read path,
// so every count comparison here is against a value the framework itself
// clamped; the output side supplies nothing that is taken on faith. Returns on
// the first violation because the caller's reaction (discard the whole output)
// is the same regardless of how many more there are.
[[nodiscard]] ControllerOutputValidation ValidateControllerOutput(
    const ControllerOutput& out, const ControllerState& state) noexcept;

// Human-readable form of `reason`, for deferred (non-RT) log lines. Returns a
// static string literal — no allocation, safe to capture from the RT path.
[[nodiscard]] const char* OutputRejectReasonToString(OutputRejectReason reason) noexcept;

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
  //   on_activate:   bump the activation generation (invalidating targets
  //                    queued while Inactive) and call
  //                    ResetTargetInitialization() so the derived controller
  //                    self-initialises its target slot on the first Compute()
  //                    tick using the current device state
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
  //
  // The CM propagates BOTH directions while its global latch still reads true
  // (issue #299): TriggerGlobalEstop raises the latch then propagates,
  // ClearGlobalEstop propagates then lowers it. That symmetry is what keeps a
  // tick landing mid-propagation on the substituting side, since the RT loop
  // decides substitution from the latch alone. The consequence for
  // implementers: an override must NOT branch on the global latch — it reads
  // the same value in both, so it cannot tell a clear from a trigger. Store
  // your own atomic and return.
  virtual void TriggerEstop() noexcept {}

  virtual void ClearEstop() noexcept {}

  [[nodiscard]] virtual bool IsEstopped() const noexcept { return false; }

  virtual void SetHandEstop(bool /*enabled*/) noexcept {}

  // ── Controller-local fault latch (issue #260) ────────────────────────────
  //
  // A controller may latch a fault of its own that no fault input can leave —
  // the compliance family's SAFE_STOP (§10.6 "자동 복구 금지"). That latch is
  // DELIBERATELY SEPARATE from the global E-STOP above (E-8): ClearEstop() must
  // not release a controller fault and ResetFault() must not release the global
  // latch. Merging them into one bool is the design #236 slice 1 rejected
  // outright, because a global clear would then launder every controller fault
  // it happens to pass over.
  //
  // Default no-ops, so a controller with no latch of its own needs neither
  // override and reads as "never latched" to the CM's /rtc_cm/reset_fault.

  // Request that a latched controller-local fault be cleared. Call OFF the RT
  // thread (the CM service callback is the intended caller). Implementations
  // must be wait-free — an atomic flag store consumed by the next Compute()
  // tick is the intended body, which is why this returns void: the clear
  // happens on the RT thread, one tick later, and can be refused there if the
  // fault cause is still present. Poll HasLatchedFault() to find out.
  virtual void ResetFault() noexcept {}

  // True while a controller-local fault latch is up. Read OFF the RT thread;
  // implementations must not block (a SeqLock load of the per-tick diagnostic
  // snapshot is the intended body). This is the observable half of ResetFault:
  // without it the reset service could only report "delivered", never "cleared".
  [[nodiscard]] virtual bool HasLatchedFault() const noexcept { return false; }

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
  //   Validating ingress for controller-owned target subscriptions (formerly
  //   CM's manager-owned callback, removed in #138). Use from controller-owned
  //   target-topic subscription callbacks; `device_idx` is the controller-local
  //   group index (position in topic_config_.groups).
  //
  //   Fail-closed contract (issue #196 §2). A message is delivered only if it
  //   satisfies ALL of:
  //     - goal_type is exactly "joint" or "task";
  //     - every scalar is finite (std::clamp does not remove NaN, so a
  //       non-finite command would otherwise survive to the actuator);
  //     - task goal: task_target holds exactly kTaskSpaceDim values;
  //     - joint goal: joint_target is non-empty;
  //     - named joint goal: joint_names and joint_target are the same length,
  //       and — when the device's joint_state_names are known — the names form
  //       an exact permutation of them (no unknown, duplicate, or missing
  //       name). Partial named goals are rejected rather than zero-filling the
  //       joints the sender omitted, which used to command them to 0 rad;
  //     - unnamed joint goal: positional, and no longer than the device's
  //       channel count (it used to be silently truncated).
  //
  //   Anything else is dropped without calling SetDeviceTarget /
  //   SetDeviceTaskTarget, bumps GetTargetRejectCount(), and emits one
  //   throttled warning naming the reason. Validation is allocation-free and
  //   runs on the controller's non-RT callback group, never the RT thread.
  //
  //   A group with no registered DeviceNameConfig cannot be checked against a
  //   reference joint list; such messages keep the historical positional
  //   behaviour (the goal_type/finite checks still apply).
  //
  //   Accepted goals dispatch via SetDeviceTarget(device_idx, ordered_span) or
  //   SetDeviceTaskTarget. The controller marshals onto an SPSC queue drained
  //   by the RT thread in Compute(), so no off-RT writer touches the target
  //   slot directly.
  void DeliverTargetMessage(const std::string& group_name, int device_idx,
                            const rtc_msgs::msg::RobotTarget& msg) noexcept;

  // Number of targets refused before they reached the queue, since
  // construction. Monotonic; intended for diagnostics and tests.
  //
  // Two producers bump it: DeliverTargetMessage's message validation above, and
  // PushPendingTarget's device_idx bound. Both mean "this goal was malformed",
  // as opposed to GetTargetDropCount()'s "this goal was well-formed but late".
  [[nodiscard]] std::uint64_t GetTargetRejectCount() const noexcept {
    return target_reject_count_.load(std::memory_order_relaxed);
  }

  // Number of accepted targets the mailbox itself dropped because the queue was
  // full when PushPendingTarget ran, since construction. Monotonic; safe to read
  // from any thread (the counter is relaxed-atomic inside SpscQueue).
  //
  // Distinct from GetTargetRejectCount(): that one counts goals refused as
  // malformed, this one counts well-formed goals the RT thread never saw
  // because the producer outran one control tick. Every copy of the marshal
  // used to discard Push()'s bool return, so a saturating producer lost goals
  // with no trace at all (#206 §4); the drop now both increments this counter
  // and emits one throttled warning naming the controller.
  //
  // Reported out of the process as ControllerState.target_drop_count in the
  // /rtc_cm/list_controllers response (#287), read straight off this accessor —
  // no controller override is involved. It is a POLLED surface, not a
  // diagnostics topic: a remote operator sees saturation by watching the value
  // move between two requests, which is what monotonicity is for.
  [[nodiscard]] std::uint64_t GetTargetDropCount() const noexcept {
    return pending_targets_.drop_count();
  }

  // Number of drained entries that reached the base's default (no-op)
  // ApplyPendingTarget, since construction. Monotonic, relaxed-atomic.
  //
  // Nonzero means a controller pushes onto the mailbox but never overrode
  // ApplyPendingTarget, so every goal it accepts is popped and thrown away.
  // That failure is otherwise perfectly silent — the drain pops the entry, so
  // GetTargetDropCount() stays 0 and DrainPendingTargets() reports it as
  // applied. A controller that does not use the mailbox at all never pushes,
  // so this stays 0 for it.
  [[nodiscard]] std::uint64_t GetTargetUnhandledCount() const noexcept {
    return target_unhandled_count_.load(std::memory_order_relaxed);
  }

  // ── Activation generation gate (fail-closed, issue #196 §3) ──────────────
  //
  // rclcpp_lifecycle gates publishers only — a controller's target
  // subscriptions stay alive while it is Inactive, so goals addressed to a
  // deactivated controller keep landing on its pending-target queue. Those
  // entries used to survive the Inactive→Active cycle and overwrite the
  // current-state hold on the first RT tick after re-activation.
  //
  // The queue cannot simply be flushed at activation: it is an SpscQueue whose
  // single consumer is the RT thread, so a lifecycle-thread Pop() would break
  // the SPSC contract. Instead every activation bumps this counter; the off-RT
  // SetDeviceTarget stamps the entry it pushes with the generation it observes,
  // and the RT-side drain discards any entry whose stamp is no longer current.
  //
  // Only on_activate bumps. An entry pushed while Inactive carries the stamp of
  // the activation that preceded it, which the next activation invalidates —
  // deactivation needs no bump of its own. Entries pushed concurrently with
  // on_activate may carry either generation; that ambiguity is inherent at the
  // boundary and harmless, since both readings correspond to a goal an operator
  // issued at activation time.
  //
  // Controllers constructed outside CM (unit tests calling Compute() directly)
  // never activate, so both sides observe generation 0 and nothing is dropped.
  [[nodiscard]] std::uint32_t ActivationGeneration() const noexcept {
    return activation_generation_.load(std::memory_order_acquire);
  }

  // True when `generation` (a stamp taken by an earlier ActivationGeneration()
  // call) still refers to the current activation. RT-side drain predicate.
  [[nodiscard]] bool IsCurrentGeneration(std::uint32_t generation) const noexcept {
    return generation == ActivationGeneration();
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

  // Called by the base on_activate() on every activation, after the generation
  // bump. Override to clear whatever "target slot already seeded" latch the
  // controller keeps (target_initialized_, arm_/hand_target_initialized_, …)
  // so the first Compute() tick after activation re-seeds the hold target from
  // the current device state.
  //
  // Centralised here because four controllers (P, CLIK, OperationalSpace,
  // JointPD) had no on_activate override at all and therefore kept the slot of
  // whatever state the robot was in when they were last deactivated. Derived
  // classes should reset the latch here rather than inside their own
  // on_activate, so a controller that adds no other activation work needs no
  // override of the lifecycle hook.
  //
  // noexcept — invoked from the noexcept lifecycle hook. Must not allocate or
  // block; a flag store is the intended body.
  virtual void ResetTargetInitialization() noexcept {}

  // ── Target mailbox: off-RT ingress → RT drain (issue #206) ───────────────
  //
  // Every controller needs the same marshal: a target arriving on the
  // controller's non-RT callback group cannot touch the RT-owned target slot
  // directly, so it is copied into a bounded lock-free queue that the RT tick
  // drains inside Compute(). That skeleton was reimplemented per controller and
  // the copies drifted — the same defect had to be fixed once per controller
  // and was missed in some (PR #256 F5/F7/F8/F9; PR #263's ComputeNoJointState
  // drain omission). It lives here now because it is identical everywhere.
  //
  // The split follows the three-tier rule (design-principles.md §3계층 배치):
  //   base    — ingress, generation gate, bounds, queue, drop accounting.
  //   derived — what a target MEANS (its TargetSlot layout, the SeqLock it
  //             publishes through, self-init seeding, trajectory re-arm flags).
  // A canonical shared slot POD was considered and rejected: WBC keeps joint
  // and task targets in independent slots and writes them from FSM phase
  // transitions too, so a common slot would promote that controller's private
  // semantics into this public contract (#206 §2).

  struct PendingTarget {
    int device_idx{0};
    int num_values{0};
    std::array<double, kMaxDeviceChannels> values{};
    // Set by the SetDeviceTaskTarget ingress; base never interprets it, it is
    // forwarded to ApplyPendingTarget so a controller with separate joint/task
    // slots can route the entry. Controllers with one slot ignore it.
    bool is_task{false};
    // Activation generation observed by the off-RT pusher. The RT drain drops
    // entries a later activation has invalidated — see ActivationGeneration().
    std::uint32_t generation{0};
  };

  static_assert(std::is_trivially_copyable_v<PendingTarget>,
                "PendingTarget must be trivially copyable for SpscQueue");

  // Four slots, i.e. three usable (the ring keeps one empty to distinguish full
  // from empty). Overflow is newest-drop, counted by GetTargetDropCount(): a
  // producer that outruns one control tick loses the goals it could not queue,
  // never the ones already accepted, and never blocks.
  static constexpr std::size_t kPendingTargetDepth = 4;

  // Off-RT producer. Stamps the current activation generation, bounds
  // `device_idx` against ControllerState::kMaxDevices and `values` against
  // kMaxDeviceChannels, then enqueues.
  //
  // An out-of-range device_idx is refused and counted by GetTargetRejectCount()
  // — the per-controller copies returned silently, which was safe only while
  // DeliverTargetMessage (whose device_idx is a topic_config_ group position)
  // was the sole producer. A caller that computes the index some other way used
  // to lose every goal with nothing anywhere to show for it.
  //
  // SINGLE PRODUCER. The queue is SPSC: in production the sole caller path is
  // DeliverTargetMessage() → SetDevice{,Task}Target() on the controller's
  // LifecycleNode default callback group (nrt_callback_executor, one thread).
  // Out-of-tree callers that push from a second thread break the queue's
  // contract — marshal onto that callback group instead.
  void PushPendingTarget(int device_idx, std::span<const double> values, bool is_task) noexcept;

  // RT consumer. Pops everything queued, discards entries invalidated by a
  // later activation or addressed to a device out of range, and hands each
  // survivor to ApplyPendingTarget(). Returns the number applied, so the caller
  // can decide whether its slot changed this tick.
  //
  // RT tick path: no allocation, no logging, no locks (the virtual dispatch is
  // an indirect branch, which is permitted — .claude/rules/rt-path.md).
  [[nodiscard]] int DrainPendingTargets() noexcept;

  // RT consumer, drop without applying. E-STOP and device-invalid early
  // returns both need this: a controller that returns before draining leaves
  // stale goals to land on the tick it resumes. PR #263 was exactly that —
  // one controller's E-STOP path discarded while its device-invalid path did
  // not, so the two diverged inside a single controller.
  //
  // WHEN to call is deliberately left to the derived controller. When the
  // mailbox was surveyed for #206 the pre-existing copies were split on it —
  // some discarded on E-STOP, most did not — so discarding here on the base's
  // own initiative would have silently changed the behaviour of the majority.
  // That split is why the choice sits in the derived class and not here; it is
  // not a statement about which controllers exist today.
  void DiscardPendingTargets() noexcept;

  // Apply one drained target. Called once per surviving entry from
  // DrainPendingTargets(), on the RT tick, with `values` already bounded to the
  // device's channel capacity and `device_idx` already range-checked. `values`
  // points at the drain's stack frame — copy what is needed, do not retain it.
  //
  // Default no-op rather than pure virtual, so a controller that does not use
  // the mailbox needs no stub (#206 R4). What the default does instead of
  // nothing is COUNT: a controller that pushes but forgets to override would
  // otherwise lose every target in perfect silence, since the drain pops the
  // entry and reports it as applied. GetTargetUnhandledCount() is the only
  // place that shows up.
  //
  // RT tick path: one relaxed fetch_add, no allocation, no logging, no locks.
  virtual void ApplyPendingTarget(int /*device_idx*/, std::span<const double> /*values*/,
                                  bool /*is_task*/) noexcept {
    target_unhandled_count_.fetch_add(1, std::memory_order_relaxed);
  }

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

  // Counts one rejected target and emits a throttled warning naming `reason`.
  // `reason` must be a string literal — the caller is noexcept and must not
  // assemble a message. Silent (counter only) when no node has been injected,
  // so unit tests can assert the counter without a LifecycleNode.
  void RejectTarget(const std::string& group_name, const char* reason) noexcept;

  // Same accounting for the mailbox ingress, which has no group name to quote:
  // PushPendingTarget is reached after DeliverTargetMessage has already resolved
  // (or bypassed) the group. Non-RT, same throttle window as RejectTarget.
  void RejectPushedTarget(const char* reason) noexcept;

  // One throttled line when a well-formed goal is lost to queue saturation.
  // Non-RT (PushPendingTarget's contract). Counter-only when no node has been
  // injected, matching RejectTarget's unit-test path.
  void WarnTargetDropped() noexcept;

  // Monotonic count of targets refused before the queue — DeliverTargetMessage's
  // message validation and PushPendingTarget's device_idx bound. Atomic because
  // diagnostics may read it from another thread while the controller's non-RT
  // callback group writes it.
  std::atomic<std::uint64_t> target_reject_count_{0};

  // Monotonic count of entries that reached the base's default ApplyPendingTarget.
  // Written on the RT thread (relaxed fetch_add), read from anywhere.
  std::atomic<std::uint64_t> target_unhandled_count_{0};

  // Bumped by on_activate; read off-RT (SetDeviceTarget stamp) and on the RT
  // thread (drain predicate), hence atomic. Wrap-around is harmless: a stamp
  // would have to survive 2^32 activations to alias the current generation,
  // and the queue is four entries deep.
  std::atomic<std::uint32_t> activation_generation_{0};

  // Private, not protected: SPSC holds only while Push and Pop each have
  // exactly one caller, and that is provable only if the base owns both ends.
  // Derived controllers reach it through PushPendingTarget (off-RT) and
  // DrainPendingTargets / DiscardPendingTargets (RT).
  SpscQueue<PendingTarget, kPendingTargetDepth> pending_targets_;

  // Private, not protected: the fail-closed guarantee only holds if the base
  // is the sole writer. Derived classes read it through GetConfigState().
  // Written on the configure/cleanup path only (single-threaded bring-up),
  // never from the RT thread — no synchronisation needed.
  ConfigState config_state_{ConfigState::kUnconfigured};
};

}  // namespace rtc

#endif  // RTC_CONTROLLER_INTERFACE_RT_CONTROLLER_INTERFACE_H_
