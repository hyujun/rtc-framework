#pragma once

// ── Project headers
// ───────────────────────────────────────────────────────────
#include "rtc_base/threading/periodic_rt_thread.hpp"
#include "rtc_base/threading/publish_buffer.hpp"
#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/timing/rt_tick_timing_sample.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controller_manager/controller_timing_profiler.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"
#include "rtc_urdf_bridge/types.hpp"
// ── ROS2 ─────────────────────────────────────────────────────────────────────
#include <rtc_msgs/msg/robot_target.hpp>
#include <rtc_msgs/srv/clear_estop.hpp>
#include <rtc_msgs/srv/list_controllers.hpp>
#include <rtc_msgs/srv/reset_fault.hpp>
#include <rtc_msgs/srv/switch_controller.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

// ── C++ stdlib
// ────────────────────────────────────────────────────────────────
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>  // ::strnlen — bounded read of the fixed-size E-STOP reason buffer
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// ── RtControllerNode
// ──────────────────────────────────────────────────────────
//
// Configurable-rate (`control_rate` YAML, default 500 Hz) RT position
// controller node.
//
// Threading model (layout v4):
//   - rt_loop (jthread, rt_control): clock_nanosleep RT loop — ControlLoop()
//                          + CheckTimeouts() + inline DeviceBackend.WriteCommand
//                          (actuator command publish, controller↔hardware RT
//                          boundary, SCHED_FIFO 90 on Core 2).
//   - nrt_publish_thread (jthread, nrt_callback): SPSC drain →
//                          controller.PublishNonRtSnapshot (controller-owned
//                          non-RT topics: RobotTarget/Transforms/DigitalTwin,
//                          SCHED_OTHER nice 0). Off-RT lane because these
//                          publishes are outside the controller↔hardware RT
//                          boundary.
//   - cb_group_rt_callback_:   backend state/motor/sensor subs (created by each
//                          DeviceBackend, MutuallyExclusive, FIFO 70 on Core 2
//                          — controller↔hardware RT boundary only, layout
//                          v4.1). DDS recv thread co-pinned to the same core
//                          via launch taskset for cache locality.
//   - cb_group_nrt_logging_:   drain_timer_  (non-RT core)
//   - cb_group_nrt_callback_:  estop_pub_ + lifecycle services (aux core).
//                          RobotTarget (external intent input) is NOT here —
//                          issue #138 moved it to controller-owned subs on each
//                          controller LifecycleNode's default group, which
//                          main() attaches to the same nrt_callback_executor.
//                          Spec §0d's "off the RT path" intent is preserved.
// Forward declaration for friend access — defined in
// test/test_controller_lifecycle.cpp.
namespace rtc {
class ControllerLifecycleTestAccess;
}

class RtControllerNode : public rclcpp_lifecycle::LifecycleNode {
  // Test-only access to private lifecycle helpers (controller_states_,
  // ActivateController, DeactivateController, SwitchActiveController)
  // without exposing them as public API.
  friend class rtc::ControllerLifecycleTestAccess;

 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Node name is supplied by the robot-specific bringup executable.
  // rtc_controller_manager is robot-agnostic — it does not own a runtime
  // identity. See agent_docs/design-principles.md.
  explicit RtControllerNode(const std::string& node_name);
  ~RtControllerNode() override;

  // ── Lifecycle callbacks ──────────────────────────────────────────────────
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

  // Public accessors for main() to retrieve callback groups
  rclcpp::CallbackGroup::SharedPtr GetRtCallbackGroup() const { return cb_group_rt_callback_; }

  rclcpp::CallbackGroup::SharedPtr GetNrtLoggingGroup() const { return cb_group_nrt_logging_; }

  rclcpp::CallbackGroup::SharedPtr GetNrtCallbackGroup() const { return cb_group_nrt_callback_; }

  // Per-controller LifecycleNodes created in on_configure.  main() attaches
  // these to nrt_callback_executor so controller-owned subscriptions/publishers
  // are processed off the RT path.  Valid after CM's on_configure has
  // succeeded.
  [[nodiscard]] const std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr>& GetControllerNodes()
      const {
    return controller_nodes_;
  }

  // RT loop lifecycle — public until Step 8 (RtControllerMain redesign)
  void StartRtLoop(const rtc::ThreadConfig& rt_cfg);
  void StartNrtPublishLoop(const rtc::ThreadConfig& nrt_pub_cfg);
  void StopRtLoop();
  void StopNrtPublishLoop();

 private:
  // ── Teardown helpers ──────────────────────────────────────────────────────
  // Closes both wake-lane eventfds and leaves them at -1. Call ONLY after the
  // device backends have been released: their state-ready callback writes
  // these fds and outlives on_deactivate (issue #224). Shared by on_cleanup /
  // on_error / the destructor so the three paths cannot drift apart again.
  void CloseTeardownEventfds() noexcept;

  // ── Session directory helpers ─────────────────────────────────────────────
  // Resolves session directory via rtc::ResolveSessionDir() 3-tier chain.
  std::filesystem::path ResolveAndSetupSessionDir();

  // ── Initialisation helpers ────────────────────────────────────────────────
  void CreateCallbackGroups();
  // Returns false when controller bring-up failed and the whole configure
  // must be refused (issue #196 §1 — fail-closed). A controller whose config
  // is invalid, or whose PreConfigure/on_configure reported non-SUCCESS, is
  // never left registered as an active/switch candidate.
  [[nodiscard]] bool DeclareAndLoadParameters();

  // Drops everything controller bring-up accumulates. Called before Pass 1 so
  // a retry cannot register a controller twice, and on every refusal so a
  // failed configure leaves no controller behind as an active/switch
  // candidate.
  void ResetControllerBringUpState();
  // Fills controller_slot_mappings_ from controller_topic_configs_ +
  // group_slot_map_. Returns false when a controller declares more device
  // groups than the RT path's fixed arrays hold, or names a group that has no
  // device slot — either way configure is refused rather than routed to a
  // default slot. See the definition for why both are unreachable via YAML
  // today and defended anyway.
  [[nodiscard]] bool BuildControllerSlotMappings();
  // Issue #138: controller-YAML target subscriptions are controller-owned
  // (per-controller LifecycleNode, created in integrated_bringup owned_topics).
  // Device-wire state/motor/sensor lanes are owned by DeviceBackend impls
  // (CreateDeviceBackends). CM therefore creates no controller-target subs.
  void CreatePublishers();
  void CreateDigitalTwinPublishers();
  void CreateFixedSafetyPublishers();
  void CreateServices();
  void ExposeTopicParameters();
  void CreateTimers();

  // ── Device backends (state/command HW/sim adapters per group) ────────────
  // One backend per active device group. Owns the state / motor / sensor
  // subscriptions and the joint / ros2 command publishers, plus the reorder
  // maps that translate between wire formats and device-config order. Built
  // in on_configure after LoadDeviceNameConfigs (sensor_layout forwarding).
  // Phase 4: backend type + wire-format topics come from
  // devices.<group>.backend: in sim.yaml / robot.yaml (parsed into
  // DeviceNameConfig::backend).
  void CreateDeviceBackends();
  // Phase 4: capability per slot is unknown until backends are constructed
  // (HasMotorState / HasSensorState are backend-impl details). After
  // CreateDeviceBackends, this helper patches `controller_slot_mappings_`
  // with `slot_to_capability_` so the RT loop has correct gating bits.
  void PropagateCapabilitiesIntoMappings();
  // Refuses a controller whose command type no backend behind it can honour.
  // Runs after CreateDeviceBackends (which is when the backends exist) and
  // after controller on_configure (which is when GetCommandType() is final).
  [[nodiscard]] bool ValidateCommandTypeSupport();

  // ── System model configuration (top-level "urdf:" YAML) ──────────────────
  void ParseSystemModelConfig(rtc_urdf_bridge::ModelConfig& config);
  void ParseSubModels(rtc_urdf_bridge::ModelConfig& config);
  void ParseTreeModels(rtc_urdf_bridge::ModelConfig& config);

  // ── Device name configuration ────────────────────────────────────────────
  // Returns false when a device config cannot be represented by the RT path's
  // fixed-size arrays (e.g. more joint_state_names than kMaxDeviceChannels),
  // which refuses the whole configure — see DeclareAndLoadParameters.
  [[nodiscard]] bool LoadDeviceNameConfigs();

  // ── RT loop (rtc::PeriodicRtThread) ───────────────────────────────────────
  // The RT loop itself is owned by a nested PeriodicRtThread subclass that
  // forwards OnTick() into ControlLoop(), and overrides WaitForNextTick()
  // for the simulation-mode CV wakeup, OnOverrun() for E-STOP escalation,
  // OnLoopAborted() for sim-sync timeouts, OnRequestStop() to wake the
  // CV, and JitterMeaningful() to suppress jitter measurement in sim mode
  // where the CV cadence makes |actual_period − budget| meaningless.
  // RtControllerNode itself only owns the tick body and the CM-specific
  // counters / readiness gates.
  class ControlLoopThread : public rtc::PeriodicRtThread {
   public:
    explicit ControlLoopThread(RtControllerNode* owner) noexcept : owner_(owner) {}

    // Public forwarders so ControlLoop() (member of the enclosing class)
    // can stamp the per-phase timing breakpoints captured by the base.
    void StampStateAcquired() noexcept { MarkStateAcquired(); }

    void StampComputeDone() noexcept { MarkComputeDone(); }

   protected:
    void OnTick() noexcept override;
    WaitResult WaitForNextTick() noexcept override;
    void OnOverrun(std::uint64_t consecutive) noexcept override;
    void OnLoopAborted() noexcept override;
    void OnRequestStop() noexcept override;
    [[nodiscard]] bool JitterMeaningful() const noexcept override;

   private:
    RtControllerNode* owner_;
  };

  void CheckTimeouts();  // 50 Hz watchdog — called inline from RT loop
  void ControlLoop();    // RT control loop (period = 1 / control_rate)

  // ── Publish offload (SPSC drain → publish) ──────────────────────────────
  // Layout v4: actuator command publish is done inline in ControlLoop()
  // (rt_control thread). Only controller-owned non-RT topics use SPSC.
  void NrtPublishLoopEntry(const rtc::ThreadConfig& cfg);
  void WaitForNrtPublishWakeup();
  void DrainLog();  // Log drain (non-RT core)

  // Digital-twin republish, drained on the nrt_publish thread. The device
  // state-lane callback (rt_callback, FIFO 70) only raises dt_dirty_[slot];
  // the actual ReadState + RELIABLE publish happens here, off the RT
  // boundary (issue #198 Phase 2). Returns true when at least one slot was
  // published, so the caller knows the drain did work this pass.
  bool DrainDigitalTwin();

  void PublishEstopStatus(bool estopped);
  // Publishes a pending /system/estop_status transition right away instead of
  // waiting for DrainLog(). Lifecycle teardown destroys drain_timer_, so the
  // last transition before teardown (notably on_error's "lifecycle_error"
  // trigger) would otherwise never reach the topic. Non-RT callers only.
  void FlushEstopStatus();

  // ── Controller-level lifecycle (aux thread only) ─────────────────────────
  // Activate / deactivate a single controller. Updates controller_states_
  // (release store) and invokes the controller's lifecycle hook. Aux-thread
  // only. Caller must hold lifecycle ordering invariants (e.g. don't
  // activate while another controller is active — use SwitchActiveController
  // for transitions).
  //
  // Hold-target initialisation is owned by the controller (not CM): on the
  // first Compute() tick after activation each controller seeds its target
  // slot from the current device state. CM therefore does not snapshot
  // device state at switch time.
  CallbackReturn ActivateController(std::size_t ctrl_idx,
                                    const rclcpp_lifecycle::State& prev_state);
  CallbackReturn DeactivateController(std::size_t ctrl_idx,
                                      const rclcpp_lifecycle::State& prev_state);

  // Swap the active controller to `name`. Sync sequence: precondition →
  // target.on_activate → store active idx → wait one RT tick →
  // previous.on_deactivate → publish active_controller_name.
  // Returns true on success; sets `message` on failure. Aux-thread only.
  bool SwitchActiveController(const std::string& name, std::string& message);

  /// Trigger a global E-Stop that propagates to all subsystems.
  /// Safe to call from any thread. Idempotent — second call is a no-op.
  void TriggerGlobalEstop(std::string_view reason) noexcept;

  /// What a ClearGlobalEstop() call actually did. The latch is only down after
  /// kCleared — the other two leave it exactly as it was found (issue #288).
  enum class EstopClearOutcome : std::uint8_t {
    kCleared,      ///< Latch was up, controllers were told, latch is now down.
    kNotLatched,   ///< Nothing was latched; no controller was touched.
    kRetriggered,  ///< A trigger arrived mid-propagation — latch LEFT UP.
  };

  /// Clear global E-Stop and re-enable all subsystems.
  /// Refuses (kRetriggered) if anything requested an E-STOP while the clear was
  /// propagating; see the implementation for why that request would otherwise
  /// be lost. Safe to call from any thread.
  [[nodiscard]] EstopClearOutcome ClearGlobalEstop() noexcept;

  [[nodiscard]] bool IsGlobalEstopped() const noexcept {
    return global_estop_.load(std::memory_order_acquire);
  }

  /// Every TriggerGlobalEstop() ENTRY, including the calls its CAS turns away
  /// because the latch is already up. Monotonic for the node's lifetime, never
  /// reset by a clear. ClearGlobalEstop() compares it across its propagation
  /// loop; that is the only thing that can see a trigger the CAS swallowed.
  [[nodiscard]] std::uint64_t EstopTriggerRequestCount() const noexcept {
    return estop_trigger_requests_.load(std::memory_order_acquire);
  }

  /// Copy of the reason captured by the most recent trigger ("" once a cleared
  /// latch has been drained). Aux-thread read of a buffer the RT thread writes
  /// with snprintf, so a copy taken during a concurrent re-trigger can mix two
  /// reasons — the same tolerance DrainLog's deferred log line already takes,
  /// and bounded here because the read never runs past the array. Callers use
  /// it for operator-facing text and confirmation, never for control flow that
  /// has to be exact.
  [[nodiscard]] std::string GlobalEstopReason() const {
    return std::string(estop_reason_.data(), ::strnlen(estop_reason_.data(), estop_reason_.size()));
  }

  /// One configured control period. Every aux-thread wait that has to let the
  /// RT loop observe an off-RT store sizes itself from this, so the period has
  /// a single definition and the margin each caller adds is visible at its own
  /// call site (switch: 1.5 × this; reset_fault: a tick-counter deadline).
  [[nodiscard]] std::chrono::microseconds ControlPeriod() const noexcept {
    const double rate_hz = (control_rate_ > 0.0) ? control_rate_ : rtc::kDefaultControlRateHz;
    return std::chrono::microseconds(static_cast<long>(1'000'000.0 / rate_hz));
  }

  /// Completed RT ticks (loop_count_). Advances only at the END of a
  /// ControlLoop() that ran the full tick body, so an aux thread that sees it
  /// advance knows Compute() ran AND everything Compute() published this tick
  /// (the controllers' diagnostic SeqLock included) is visible to it — that is
  /// what the release/acquire pair on loop_count_ buys. Reading it is how
  /// /rtc_cm/reset_fault tells "nothing consumed the request" apart from "the
  /// fault cause is still present" (#260).
  [[nodiscard]] std::uint64_t RtTickCount() const noexcept {
    return loop_count_.load(std::memory_order_acquire);
  }

  /// Total ticks whose ControllerOutput failed actuator-boundary validation
  /// and was replaced by a hold command (issue #196 Phase 4). Monotonic for
  /// the node's lifetime — never reset, including across E-STOP clear.
  [[nodiscard]] uint64_t RejectedOutputCount() const noexcept {
    return rejected_output_count_.load(std::memory_order_relaxed);
  }

  /// Consecutive rejected ticks up to now (0 once a valid output lands).
  [[nodiscard]] uint64_t ConsecutiveRejectedOutputCount() const noexcept {
    return consecutive_rejected_outputs_.load(std::memory_order_relaxed);
  }

  /// Total ticks whose ControllerOutput was replaced by a hold *because the
  /// global E-STOP latch was set* (issue #198 Phase 3), counted separately
  /// from RejectedOutputCount so a test can tell which guard blocked the
  /// write — the two produce the same command and would otherwise be
  /// indistinguishable at the backend. Monotonic for the node's lifetime.
  [[nodiscard]] uint64_t EstopSubstitutedOutputCount() const noexcept {
    return estop_substituted_outputs_.load(std::memory_order_relaxed);
  }

  /// Consecutive-reject count at which the hold escalates to a global E-STOP.
  /// Rate-derived — see invalid_output_estop_ticks_.
  [[nodiscard]] uint64_t OutputRejectEstopTicks() const noexcept {
    return invalid_output_estop_ticks_;
  }

  // ── ROS2 handles ──────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_rt_callback_;
  rclcpp::CallbackGroup::SharedPtr cb_group_nrt_logging_;
  rclcpp::CallbackGroup::SharedPtr cb_group_nrt_callback_;

  // No CM-owned subscription container lives here. Controller-YAML target subs
  // are controller-owned (integrated_bringup CreateOwnedTopics) and device-wire
  // state/motor/sensor subs are owned by DeviceBackend impls — see
  // CreatePublishers' comment for the issue #138 split.

  // Fixed publishers (always present)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_ctrl_name_pub_;

  // ── /rtc_cm/* services (Phase 3) ─────────────────────────────────────────
  // All callbacks run on cb_group_nrt_callback_ — never on the RT path. The switch
  // service is a thin wrapper around SwitchActiveController(name, message);
  // list_controllers builds its response from controller_states_ +
  // controller_topic_configs_ + controller_types_; reset_fault clears a
  // controller-local fault latch on the active controller (#260, E-8 — NOT the
  // global E-STOP latch, which ClearGlobalEstop owns).
  rclcpp::Service<rtc_msgs::srv::ListControllers>::SharedPtr list_controllers_srv_;
  rclcpp::Service<rtc_msgs::srv::SwitchController>::SharedPtr switch_controller_srv_;
  rclcpp::Service<rtc_msgs::srv::ResetFault>::SharedPtr reset_fault_srv_;
  rclcpp::Service<rtc_msgs::srv::ClearEstop>::SharedPtr clear_estop_srv_;

  // The controller-output publish role (kRobotTransforms — the only one left
  // after issue #196 Phase 5) is owned by each controller's LifecycleNode via
  // owned_topics.cpp + PublishNonRtSnapshot — CM does not host them.
  // GraspState / WbcState / ToFSnapshot bypass YAML role mapping and ride
  // per-controller SeqLock<T> handoffs.

  // ── Per-group JointState republishers (RELIABLE, depth 1) ───────────────
  // "/rtc_cm/{group}/joint_states" — single source of truth for the measured
  // joint state per device group, regardless of which controller is active.
  // rtc_digital_twin (or any external tool) merges these into a single
  // /joint_states for robot_state_publisher / RViz.
  //
  // Indexed by device slot, not keyed by topic name: the producer side is a
  // state-lane callback on the rt_callback executor, which must not do map
  // lookups (issue #198 Phase 2). The slot→entry resolution that used to
  // cost two unordered_map probes per state message is now the array index
  // itself, fixed at CreateDigitalTwinPublishers() time.
  struct DigitalTwinEntry {
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    sensor_msgs::msg::JointState msg;  // pre-allocated with config joint_state_names
  };

  std::array<DigitalTwinEntry, rtc::kMaxDevices> digital_twin_by_slot_;

  // Raised by the state-lane callback, cleared by DrainDigitalTwin() on the
  // nrt_publish thread. A bit, not a queue: the payload is already in the
  // backend's SeqLock, so the drain always republishes the newest state.
  // Bursts that outrun the drain therefore coalesce rather than queue — a
  // twin that lags never shows a stale sample, only fewer of them.
  std::array<std::atomic<bool>, rtc::kMaxDevices> dt_dirty_{};

  // Per-controller topic config cache (index = controller index)
  std::vector<rtc::TopicConfig> controller_topic_configs_;

  // Controller name → index lookup (populated once at startup)
  // Maps both display names ("DemoJointController") and config keys
  // ("demo_joint_controller")
  std::unordered_map<std::string, int> controller_name_to_idx_;

  // Per-controller registry plugin name (config_key from
  // RTC_REGISTER_CONTROLLER), parallel to controllers_. Returned as the
  // `type` field by /rtc_cm/list_controllers.
  std::vector<std::string> controller_types_;

  // ── Dynamic device group management ──────────────────────────────────────
  std::set<std::string> active_groups_;        // union of all controller groups
  std::map<std::string, int> group_slot_map_;  // group name → PublishSnapshot slot index

  // Pre-resolved group→slot mapping per controller (avoids map lookup on RT
  // path). controller_slot_mappings_[ctrl_idx].slots[group_iteration_order] =
  // device slot index.
  struct ControllerSlotMapping {
    static constexpr int kMaxSlots = rtc::PublishSnapshot::kMaxGroups;
    std::array<int, kMaxSlots> slots{};
    std::array<uint16_t, kMaxSlots> capabilities{};  ///< DeviceCapability per group
    int num_groups{0};
  };

  std::vector<ControllerSlotMapping> controller_slot_mappings_;

  // ── Device liveness entries (startup gate + E-STOP watchdog) ──────────────
  //
  // One entry per CONFIGURED DEVICE GROUP, not per `device_timeout_names`
  // entry (issue #198 §1). Deriving the set from the groups that actually got
  // a backend is what makes "every required device" mean something: a group
  // left out of the timeout list used to be invisible to both the watchdog and
  // the startup gate, so forgetting one line of YAML silently disabled the
  // check for that device. The timeout list now supplies the *value* only.
  //
  // The entry carries no timestamp of its own. Liveness is read from
  // `backends_[slot]->LastStateStamp()`, which is atomic by contract — the
  // plain `steady_clock::time_point` that used to live here was written by the
  // state-lane callback and read by the RT thread with no synchronisation at
  // all (§2), and it duplicated a stamp every backend already maintained.
  struct DeviceTimeoutEntry {
    std::string group_name;
    std::string state_topic;
    std::chrono::milliseconds timeout{100};
    int slot{-1};  ///< index into backends_ — the liveness source for this group
  };

  std::vector<DeviceTimeoutEntry> device_timeouts_;

  /// Liveness of one entry: a state message has arrived AND it is within the
  /// group's timeout. Both the startup gate and the watchdog use this single
  /// predicate, so "ready to start" and "still alive" cannot drift apart — a
  /// re-activate after the device went quiet is refused for the same reason a
  /// running loop E-STOPs. RT-safe: atomic load plus comparisons.
  [[nodiscard]] bool DeviceLive(const DeviceTimeoutEntry& entry,
                                std::chrono::steady_clock::time_point now) const noexcept;

  /// Index of the first configured device that is not live, or -1 when all
  /// are. Returning the index (rather than a bool) is what lets the startup
  /// deadline name the device that never reported. RT-safe.
  [[nodiscard]] int FirstUnreadyDevice(std::chrono::steady_clock::time_point now) const noexcept;

  // ── Per-device backends (HW/sim adapters, indexed by group_slot_map_) ────
  // Each backend owns its own SeqLock<DeviceStateCache>; CM reads via
  // backend->ReadState/ReadMotorState/ReadSensorState in the RT loop.
  using DeviceStateCache = rtc::DeviceStateCache;
  static constexpr int kMaxDevices = rtc::kMaxDevices;

  std::array<std::unique_ptr<rtc::DeviceBackend>, kMaxDevices> backends_;

  // Read-only parameter guard handle (topic params immutable after init)
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rclcpp::TimerBase::SharedPtr drain_timer_;  // log drain (log thread)

  // ── RT loop ────────────────────────────────────────────────────────────
  // The PeriodicRtThread owns the loop thread + clock_nanosleep cadence +
  // overrun bookkeeping; RtControllerNode keeps the CM-specific state below
  // (loop_count_, init_complete_, …).
  ControlLoopThread rt_loop_{this};

  // ── Publish offload (controller-owned non-RT lane only) ─────────────────
  // Layout v4: actuator command lane removed — RT loop calls
  // DeviceBackend.WriteCommand inline on the rt_control thread (Core 2
  // FIFO 90) at the end of each tick. Only controller-owned non-RT
  // publishes (RobotTarget/Transforms/DigitalTwin) ride an SPSC + dedicated
  // jthread because those publishes are outside the controller↔hardware RT
  // boundary and must not preempt RT work.
  rtc::NrtPublishBuffer nrt_publish_buffer_{};
  std::jthread nrt_publish_thread_;
  std::atomic<bool> nrt_publish_running_{false};
  // Atomic because the teardown paths (on_cleanup / on_error / destructor, on
  // the lifecycle executor) reset it to -1 while producers on other threads —
  // the rt_control loop and the backends' state-ready callback on
  // cb_group_rt_callback_ — still read it as their write guard (issue #224).
  // Producers must load once into a local: re-reading the member between the
  // guard and the eventfd_write reintroduces the check-then-use gap.
  std::atomic<int> nrt_publish_eventfd_{-1};

  // ── Domain objects ────────────────────────────────────────────────────────
  std::vector<std::unique_ptr<rtc::RTControllerInterface>> controllers_;
  // Owned in parallel with controllers_: nodes_[i] is the LifecycleNode
  // injected into controllers_[i] via on_configure().  Index-aligned so
  // switch/activate logic can pair them without extra lookup.
  std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> controller_nodes_;
  std::atomic<int> active_controller_idx_{1};
  // Per-controller lifecycle state, parallel to controllers_. Values map
  // 1:1 to rtc_msgs::msg::ControllerState (0=Inactive, 1=Active). Single-
  // active is enforced by SwitchActiveController; the vector permits a
  // future relaxation to multi-active without a schema change.
  std::vector<std::atomic<int>> controller_states_;
  rtc::ControllerTimingProfiler timing_profiler_{};  // Compute() timing

  // Per-tick RT-loop timing → <session>/timing/cm_timing_log.csv.
  // Producer (RT thread, ControlLoop) pushes one RtTickTimingPayload per
  // tick; consumer (log thread, DrainLog at 100 Hz) drains into the CSV
  // logger. Schema is shared with the MPC thread — see
  // rtc_base/timing/rt_tick_timing_sample.hpp.
  rtc::CmTimingBuffer cm_timing_producer_{};
  rtc::ThreadTimingCsvLogger<rtc::RtTickTimingPayload> cm_timing_logger_{};

  // ── Shared state ──────────────────────────────────────────────────────────
  // Device state lives inside each DeviceBackend (per-device SeqLock,
  // lock-free single-writer/multi-reader). Writer: backend's own sub
  // callbacks (cb_group_rt_callback_, MutuallyExclusive). Readers: RT loop
  // (ControlLoop) via backend->ReadState.
  //
  // Per-controller target slots live on each controller (SeqLock<TargetSlot>
  // + SpscQueue marshal). The controller-owned target subscription forwards
  // into its own SetDeviceTarget (DeliverTargetMessage) — there is no CM-side
  // target subscription or mirror (issue #138).

  std::atomic<bool> print_timing_summary_{false};

  // Wall-clock timestamp of the previous timing-summary print, used by
  // DrainLog() (log thread, single accessor) to compute the robot-mode
  // `elapsed=` field as a wall delta. Default-constructed (epoch) marks the
  // first print since (re)activation; reset in on_deactivate().
  std::chrono::steady_clock::time_point last_summary_wall_{};

  // ── System-level model configuration (top-level "urdf:" YAML section) ────
  rtc_urdf_bridge::ModelConfig system_model_config_;

  // ── Per-device name configuration ────────────────────────────────────────
  std::map<std::string, rtc::DeviceNameConfig> device_name_configs_;
  std::vector<std::string> slot_to_group_name_;  // reverse: slot index → group name

  // Per-slot sensor packing layout cache (resolved from device_name_configs_
  // at LoadDeviceNameConfigs time). Indexed by device slot. Empty optional
  // means the device exposes no packed sensor block.
  std::vector<std::optional<rtc::DeviceSensorLayout>> slot_to_sensor_layout_;

  // Per-slot DeviceCapability bitmask derived from the backend impl
  // (HasMotorState/HasSensorState + sensor_layout.inference_values_per_group)
  // in CreateDeviceBackends. Propagated into ControllerSlotMapping at the
  // same time so the RT loop can skip whole copy blocks per slot.
  std::vector<uint16_t> slot_to_capability_;

  // ── Simulation sync (eventfd wakeup) ────────────────────────────────────
  // Producer: device state-lane callback (rt_callback, FIFO 70) — one
  // non-blocking eventfd_write per state message. Consumer: WaitForNextTick()
  // on the rt_control thread (FIFO 90). It was a condition_variable, which
  // made the two RT-priority threads share a mutex (issue #198 §3 —
  // RT-4 + RT-10). An eventfd removes the mutex entirely, and because a
  // non-semaphore eventfd read drains the whole counter, a burst of state
  // messages still yields exactly one wake-up with no notification lost.
  bool use_sim_time_sync_{false};
  double sim_sync_timeout_sec_{5.0};
  // Atomic for the same reason as nrt_publish_eventfd_ — see there.
  std::atomic<int> sim_wake_eventfd_{-1};

  // ── Parameters ────────────────────────────────────────────────────────────
  // RT loop rate [Hz]. Loaded from the YAML `control_rate` parameter in
  // DeclareAndLoadParameters(); kDefaultControlRateHz is only the
  // declare_parameter default (used when YAML omits the field). The
  // framework is rate-agnostic — any kMin..kMaxControlRateHz value is
  // valid, and all derived quantities (budget_us_, init_timeout_ticks_,
  // ControllerState::dt, PeriodicRtThread::frequency_hz) are recomputed
  // from this rate at parameter load time.
  double control_rate_{rtc::kDefaultControlRateHz};
  bool enable_logging_{true};
  bool enable_estop_{true};

  // Device-timeout watchdog cadence. CheckTimeouts() runs every Nth RT tick,
  // and the contract is a rate-independent kWatchdogCheckHz — a fixed divisor
  // would mean 10 Hz at the bottom of the supported range and 500 Hz at the
  // top, i.e. up to ~100 ms of extra detection latency where the loop is
  // slowest and a needless hot-path cost where it is fastest. Derived in
  // DeclareAndLoadParameters(); the floor of 1 keeps the watchdog running
  // every tick if the rate is ever configured at or below the check rate.
  static constexpr double kWatchdogCheckHz = 50.0;
  std::uint32_t watchdog_check_divisor_{
      static_cast<std::uint32_t>(rtc::kDefaultControlRateHz / kWatchdogCheckHz)};

  // Completed RT ticks. Written only by the RT thread (one relaxed-cost
  // release store per tick, wait-free), but read off-RT by RtTickCount() —
  // hence atomic rather than a plain counter. The release side is what makes
  // everything the tick published before the increment visible to an aux
  // thread that observes the new value.
  std::atomic<std::uint64_t> loop_count_{0};

  // ── Initialization timeout
  // ──────────────────────────────────────────────────
  bool init_complete_{false};
  uint64_t init_wait_ticks_{0};
  // Placeholder default; overwritten in DeclareAndLoadParameters() with
  // `init_timeout_sec * control_rate_` once the runtime rate is known.
  // (5 s × kDefaultControlRateHz at the default rate.)
  uint64_t init_timeout_ticks_{static_cast<uint64_t>(5.0 * rtc::kDefaultControlRateHz)};

  // Session-wide t=0 origin for ControllerState::t_relative_s (and the
  // legacy log timestamp). Captured on the very first ControlLoop()
  // iteration regardless of logging flags, NEVER reset on controller
  // switch — keeps cross-controller log alignment monotonic.
  std::chrono::steady_clock::time_point log_start_time_{};

  // ── Global E-Stop ──────────────────────────────────────────────────────────
  std::atomic<bool> global_estop_{false};
  // Every TriggerGlobalEstop() entry, counted before its CAS so the swallowed
  // calls are counted too (issue #288). See EstopTriggerRequestCount().
  std::atomic<std::uint64_t> estop_trigger_requests_{0};
  std::array<char, 128> estop_reason_{};        // fixed-size — no heap alloc on RT path
  std::atomic<bool> estop_log_pending_{false};  // deferred logging flag
  // Deferred /system/estop_status publish. TriggerGlobalEstop /
  // ClearGlobalEstop are reachable from the RT loop, where a plain
  // rclcpp::Publisher::publish is a RT-10 violation, so they raise this flag
  // and DrainLog() (nrt_logging, 100 Hz) publishes the current global_estop_
  // value. Eventual consistency by construction: the drain always publishes
  // the latch's value at drain time, so a trigger/clear pair that lands
  // between two drains collapses to the final state rather than a stale one.
  std::atomic<bool> estop_status_pending_{false};

  // ── Per-tick timing & overrun ────────────────────────────────────────────
  // Period budget kept here (us) for the timing summary log line (window
  // duration = count × period). overrun / skip / consecutive counters live
  // on the base (rt_loop_.OverrunCount() etc.); per-tick over-budget detail
  // is recoverable from cm_timing_log.csv.
  double budget_us_{2000.0};
  static constexpr uint64_t kMaxConsecutiveOverruns = 10;

  // ── ControllerOutput actuator-boundary validation (issue #196 Phase 4) ────
  //
  // A rejected output is replaced by a hold command rather than dropped: the
  // backend keeps receiving a well-formed command every tick, so a device that
  // interprets a gap in the command stream as "fault" or "free-run" never sees
  // one. Persistent rejection is a controller fault the hold cannot fix, hence
  // the escalation to E-STOP below.
  //
  // Pre-allocated so the RT path never constructs a ~28 kB ControllerOutput on
  // the tick stack. Written and read only by the rt_control thread.
  rtc::ControllerOutput hold_output_{};

  // Consecutive-reject threshold for E-STOP escalation. Derived from the
  // configured rate in DeclareAndLoadParameters(): a fixed tick count would
  // mean 100 ms at 100 Hz and 2 ms at 5 kHz — a 50× swing in how long bad
  // commands keep flowing. The 10-tick floor matches kMaxConsecutiveOverruns
  // so the two RT escalation paths stay comparable at low rates.
  static constexpr double kOutputRejectEstopSeconds = 0.1;
  uint64_t invalid_output_estop_ticks_{
      static_cast<uint64_t>(kOutputRejectEstopSeconds * rtc::kDefaultControlRateHz)};

  // Atomics for cross-thread test/telemetry reads only — the RT thread is the
  // sole writer, so relaxed ordering is sufficient.
  std::atomic<uint64_t> rejected_output_count_{0};
  std::atomic<uint64_t> consecutive_rejected_outputs_{0};
  std::atomic<uint64_t> estop_substituted_outputs_{0};

  // Size of the fixed E-STOP reason buffer built on the RT path. Sized to fit
  // the longest reason this file formats, with room to spare; std::snprintf
  // truncates and NUL-terminates regardless, and TriggerGlobalEstop copies
  // into its own fixed buffer.
  static constexpr std::size_t kEstopReasonBufferSize = 64;

  // Fills hold_output_ from `state` and `cmd_type` (the active controller's
  // configured type, YAML-derived). Deliberately ignores every field of the
  // rejected output, including its per-device command_type override: reading
  // a mode selector out of the data that just failed validation would make
  // the check meaningless. RT-safe — fixed-size writes, no allocation, no
  // logging.
  //
  // Returns false when a device's measured positions are not all finite, in
  // which case that device is given a zero-length command instead of a
  // position built from garbage. The state's *counts* are trustworthy (the
  // read path bounds them), but its *values* are copied verbatim from the
  // backend, so the hold cannot assume them finite.
  [[nodiscard]] bool BuildHoldOutput(const rtc::ControllerState& state,
                                     rtc::CommandType cmd_type) noexcept;
};
