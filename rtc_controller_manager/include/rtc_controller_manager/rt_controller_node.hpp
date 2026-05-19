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
#include <rtc_msgs/srv/list_controllers.hpp>
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

// ── RtControllerNode
// ──────────────────────────────────────────────────────────
//
// Configurable-rate (`control_rate` YAML, default 500 Hz) RT position
// controller node.
//
// Threading model:
//   - rt_loop (jthread):   clock_nanosleep RT loop — ControlLoop() +
//   CheckTimeouts()
//   - publish_thread (jthread, rt_outbound): SPSC drain →
//                          DeviceBackend.WriteCommand only (actuator command,
//                          RT controller↔hardware boundary, SCHED_FIFO 65).
//   - nrt_publish_thread (jthread, nrt_callback): SPSC drain →
//                          controller.PublishNonRtSnapshot (controller-owned
//                          non-RT topics: RobotTarget/Transforms/DigitalTwin,
//                          SCHED_OTHER nice 0). Separate lane because these
//                          publishes are outside the controller↔hardware RT
//                          boundary.
//   - cb_group_rt_inbound_:    backend state/motor/sensor subs (created by each
//                          DeviceBackend) + target_sub_ (Sensor core)
//   - cb_group_nrt_logging_:       drain_timer_  (non-RT core)
//   - cb_group_nrt_callback_:       estop_pub_  (aux core)
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
  rclcpp::CallbackGroup::SharedPtr GetRtInboundGroup() const { return cb_group_rt_inbound_; }

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
  void StartPublishLoop(const rtc::ThreadConfig& pub_cfg);
  void StartNrtPublishLoop(const rtc::ThreadConfig& nrt_pub_cfg);
  void StopRtLoop();
  void StopPublishLoop();
  void StopNrtPublishLoop();

 private:
  // ── Session directory helpers ─────────────────────────────────────────────
  // Resolves session directory via rtc::ResolveSessionDir() 3-tier chain.
  std::filesystem::path ResolveAndSetupSessionDir();

  // ── Initialisation helpers ────────────────────────────────────────────────
  void CreateCallbackGroups();
  void DeclareAndLoadParameters();
  void CreateSubscriptions();
  // CreateSubscriptions helper — state/motor/sensor lanes are owned by
  // DeviceBackend implementations (CreateDeviceBackends); CM only binds
  // kTarget itself.
  void CreateTargetSubscription(const rtc::SubscribeTopicEntry& entry,
                                const std::string& group_name, int slot,
                                const rclcpp::SubscriptionOptions& sub_options);
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

  // ── Subscription callbacks (unified per-device) ──────────────────────────
  void DeviceTargetCallback(int device_slot, rtc_msgs::msg::RobotTarget::SharedPtr msg);

  // ── System model configuration (top-level "urdf:" YAML) ──────────────────
  void ParseSystemModelConfig(rtc_urdf_bridge::ModelConfig& config);
  void ParseSubModels(rtc_urdf_bridge::ModelConfig& config);
  void ParseTreeModels(rtc_urdf_bridge::ModelConfig& config);

  // ── Device name configuration ────────────────────────────────────────────
  void LoadDeviceNameConfigs();

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
  void PublishLoopEntry(const rtc::ThreadConfig& cfg);
  void WaitForPublishWakeup();
  void NrtPublishLoopEntry(const rtc::ThreadConfig& cfg);
  void WaitForNrtPublishWakeup();
  void DrainLog();  // Log drain (non-RT core)

  void PublishEstopStatus(bool estopped);

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
  /// Clear global E-Stop and re-enable all subsystems.
  void ClearGlobalEstop() noexcept;

  [[nodiscard]] bool IsGlobalEstopped() const noexcept {
    return global_estop_.load(std::memory_order_acquire);
  }

  // ── ROS2 handles ──────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_rt_inbound_;
  rclcpp::CallbackGroup::SharedPtr cb_group_nrt_logging_;
  rclcpp::CallbackGroup::SharedPtr cb_group_nrt_callback_;

  // ── Configurable topic subscriptions (created from controller YAML) ──────
  // Key = topic name, value = subscription handle (kept alive for node
  // lifetime). Only kTarget remains here — kState/kMotorState/kSensorState
  // moved into DeviceBackend implementations.
  std::vector<rclcpp::SubscriptionBase::SharedPtr> topic_subscriptions_;

  // Fixed publishers (always present)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_ctrl_name_pub_;

  // ── /rtc_cm/* services (Phase 3) ─────────────────────────────────────────
  // Both callbacks run on cb_group_nrt_callback_ — never on the RT path. The switch
  // service is a thin wrapper around SwitchActiveController(name, message);
  // list_controllers builds its response from controller_states_ +
  // controller_topic_configs_ + controller_types_.
  rclcpp::Service<rtc_msgs::srv::ListControllers>::SharedPtr list_controllers_srv_;
  rclcpp::Service<rtc_msgs::srv::SwitchController>::SharedPtr switch_controller_srv_;

  // Controller-output publish roles (kRobotTarget / kRobotTransforms /
  // kDigitalTwinState) are owned by each controller's LifecycleNode via
  // owned_topics.cpp + PublishNonRtSnapshot — CM does not host them.
  // GraspState / WbcState / ToFSnapshot bypass YAML role mapping and ride
  // per-controller SeqLock<T> handoffs.

  // ── Per-group JointState republishers (RELIABLE, depth 10) ──────────────
  // key = "/rtc_cm/{group}/joint_states" — single source of truth for the
  // measured joint state per device group, regardless of which controller is
  // active. rtc_digital_twin (or any external tool) merges these into a
  // single /joint_states for robot_state_publisher / RViz.
  struct DigitalTwinEntry {
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    sensor_msgs::msg::JointState msg;  // pre-allocated with config joint_state_names
  };

  std::unordered_map<std::string, DigitalTwinEntry> digital_twin_publishers_;
  // group_slot → JointState topic name mapping
  std::unordered_map<int, std::string> slot_to_dt_topic_;

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

  // ── Device timeout entries (E-STOP watchdog) ──────────────────────────────
  struct DeviceTimeoutEntry {
    std::string group_name;
    std::string state_topic;
    std::chrono::milliseconds timeout{100};
    std::chrono::steady_clock::time_point last_update{};
    std::atomic<bool> received{false};

    DeviceTimeoutEntry() = default;

    DeviceTimeoutEntry(DeviceTimeoutEntry&& o) noexcept
        : group_name(std::move(o.group_name)),
          state_topic(std::move(o.state_topic)),
          timeout(o.timeout),
          last_update(o.last_update),
          received(o.received.load(std::memory_order_relaxed)) {}

    DeviceTimeoutEntry& operator=(DeviceTimeoutEntry&&) = delete;
    DeviceTimeoutEntry(const DeviceTimeoutEntry&) = delete;
    DeviceTimeoutEntry& operator=(const DeviceTimeoutEntry&) = delete;
  };

  std::vector<DeviceTimeoutEntry> device_timeouts_;
  [[nodiscard]] bool AllTimeoutDevicesReceived() const noexcept;

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

  // ── Publish offload (SPSC buffer + dedicated thread) ────────────────────
  // Actuator command lane (rt_outbound, SCHED_FIFO 65 after Phase 4):
  //   producer = RT loop, consumer = publish_thread_ → backend.WriteCommand.
  rtc::ControlPublishBuffer publish_buffer_{};
  std::jthread publish_thread_;
  std::atomic<bool> publish_running_{false};
  int publish_eventfd_{-1};  // eventfd for RT→publish wakeup (replaces sched_yield)

  // Controller-owned non-RT topic lane (nrt_callback, SCHED_OTHER nice 0):
  //   producer = RT loop, consumer = nrt_publish_thread_ →
  //   controller.PublishNonRtSnapshot. Separate from publish_buffer_ so the
  //   rt_outbound thread carries only actuator commands (controller↔hardware
  //   boundary); controller-owned publishes (RobotTarget/Transforms/
  //   DigitalTwin) ride a non-RT consumer.
  rtc::NrtPublishBuffer nrt_publish_buffer_{};
  std::jthread nrt_publish_thread_;
  std::atomic<bool> nrt_publish_running_{false};
  int nrt_publish_eventfd_{-1};

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
  // callbacks (cb_group_rt_inbound_, MutuallyExclusive). Readers: RT loop
  // (ControlLoop) via backend->ReadState.
  //
  // Per-controller target slots live on each controller (SeqLock<TargetSlot>
  // + SpscQueue marshal). CM forwards DeviceTargetCallback into the active
  // controller's SetDeviceTarget — there is no CM-side target mirror.

  std::atomic<bool> print_timing_summary_{false};
  std::atomic<bool> state_received_{false};

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

  // ── Simulation sync (CV-based wakeup) ──────────────────────────────────
  bool use_sim_time_sync_{false};
  double sim_sync_timeout_sec_{5.0};
  std::mutex state_cv_mutex_;
  std::condition_variable state_cv_;
  std::atomic<bool> state_fresh_{false};

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

  std::size_t loop_count_{0};

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
  std::array<char, 128> estop_reason_{};        // fixed-size — no heap alloc on RT path
  std::atomic<bool> estop_log_pending_{false};  // deferred logging flag

  // ── Per-tick timing & overrun ────────────────────────────────────────────
  // Period budget kept here (us) for the timing summary log line (window
  // duration = count × period). overrun / skip / consecutive counters live
  // on the base (rt_loop_.OverrunCount() etc.); per-tick over-budget detail
  // is recoverable from cm_timing_log.csv.
  double budget_us_{2000.0};
  static constexpr uint64_t kMaxConsecutiveOverruns = 10;
};
