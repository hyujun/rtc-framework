#ifndef RTC_CONTROLLER_MANAGER_DEVICE_BACKEND_H_
#define RTC_CONTROLLER_MANAGER_DEVICE_BACKEND_H_

#include "rtc_base/threading/publish_buffer.hpp"
#include "rtc_base/timing/rt_tick_timing_sample.hpp"
#include "rtc_base/types/types.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"
#include "rtc_controller_manager/state_lane_gate.hpp"

#include <rclcpp/callback_group.hpp>
#include <rclcpp/function_traits.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
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
//   - CloseStateLane() — called by CM right before the backend is destroyed
//                   (on_cleanup / on_error / refused configure / destructor);
//                   refuses and drains the state-lane callbacks (issue #941).
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
  /// or pick a different ros2_control chain if needed.
  ///
  /// This used to read "Never blocks; never allocates." Both halves were
  /// unverified assertions, and issue #222 measured them on the UR actuator
  /// lane (RELIABLE depth 1, 6-double Float64MultiArray, 500 Hz):
  ///
  ///   - **It allocates.** Every call, 2 blocks / 156 B — an 88 B serdata
  ///     object built by dds_write's sertype callback and a 68 B CDR buffer.
  ///     The allocation is QoS-independent (a BEST_EFFORT arm matched it byte
  ///     for byte) and lives inside libddsc/rmw_cyclonedds_cpp, so it is not
  ///     something a backend can avoid while publishing on this thread. This
  ///     is a known, accepted RT-1 violation, not a licence for new ones —
  ///     see invariants.md §RT-1.
  ///   - **It does not block**, and the reason is the QoS, not the code: a
  ///     KEEP_LAST(1) writer drops unacked samples instead of retaining them,
  ///     so the write history cache never approaches its high-water mark. A
  ///     frozen reliable reader left the publish at 47 us max; the same
  ///     harness with a KEEP_ALL writer blocked one publish for 2.9 s.
  ///     **Raising this lane's history depth would reintroduce blocking.**
  ///
  /// Implementations must still not block, log, lock, or allocate anything of
  /// their own — the measured 156 B is the rmw floor, not a budget.
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
  /// configure and refuses the mismatch (ValidateCommandTypePairing).
  ///
  /// Declaring a mode answers "the wire carries this", not "the wire carries
  /// this safely". kTorque in particular still gets CM's 0 N·m hold on E-STOP,
  /// because CM has no dynamic model; the same configure pass warns about that
  /// separately rather than reading it out of this answer (issue #339).
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

  /// Callback fired by the backend on every fresh JOINT-state arrival. Every
  /// shipped backend calls NotifyStateReady() from its joint lane only; motor,
  /// sensor and wrench lanes are read latest-value and never fire it. In
  /// sim-sync mode this is what completes a simulator step for the slot —
  /// CM ticks once every slot in `sim_sync_tick_devices` has fired (issue
  /// #566) — so a backend must fire it exactly once per joint state it
  /// publishes. Slot identity is supplied via lambda capture by the caller.
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

  /// Install the rt_callback-lane timing sink (issue #349). CM owns the
  /// producer and drains it on the log thread; backends only push.
  ///
  /// Null — the default — disables instrumentation entirely: the scope below
  /// then does nothing but one predictable null test, so a backend built
  /// without a sink pays no clock reads. Not RT; call before Configure().
  ///
  /// Every backend CM configures shares ONE sink, which is safe only because
  /// they also share one MutuallyExclusive callback group (see Configure()).
  /// That serialisation is what makes the single-producer contract of the
  /// underlying SPSC ring hold across backends, not just within one.
  void SetStateLaneTimingSink(RtCallbackTimingBuffer* sink) noexcept { state_lane_timing_ = sink; }

  // ── State-lane teardown (issue #941) ───────────────────────────────────────

  /// Bound on CloseStateLane()'s wait. The measured joint-lane body is ~7 us
  /// mean, < 200 us p99 and < 4 ms max (issue #941), so a callback still inside
  /// at 50 ms is stuck, not slow — waiting longer would only delay the leak.
  static constexpr std::chrono::milliseconds kStateLaneCloseTimeout{50};

  /// Refuses every further state-lane callback and waits for the ones already
  /// executing to return. Returns false when one is still running at
  /// kStateLaneCloseTimeout — the caller must then NOT destroy this backend
  /// (CM leaks it instead: a leaked backend is a bounded loss, a backend freed
  /// under its own callback is a use-after-free on the RT thread).
  ///
  /// The lane stays closed for the life of the object: there is no reopen,
  /// because CM builds a fresh backend on every configure. Not RT — the
  /// lifecycle teardown paths (on_cleanup / on_error / destructor / a refused
  /// configure) call it right before the reset.
  [[nodiscard]] bool CloseStateLane() noexcept {
    return state_lane_gate_->Close(kStateLaneCloseTimeout);
  }

  /// True once CloseStateLane() has been called. Readable from any thread.
  [[nodiscard]] bool StateLaneClosed() const noexcept { return state_lane_gate_->IsClosed(); }

 protected:
  /// Wraps a state-lane subscription callback so the backend's teardown can
  /// drain it (issue #941). Every subscription a backend creates on the
  /// `state_cb_group` — joint, motor, sensor, wrench — goes through this:
  ///
  ///     state_sub_ = node->create_subscription<Msg>(
  ///         topic, qos, GuardStateLane([this](Msg::SharedPtr msg) { OnState(std::move(msg)); }),
  ///         sub_opts);
  ///
  /// The returned callable has the same single message parameter as `callback`
  /// (deduced through rclcpp::function_traits, so rclcpp's dispatch overload
  /// resolution is unchanged — this is why a generic lambda is not accepted).
  /// Per message it costs one StateLaneEntry: two atomic RMWs and a load, no
  /// allocation, no lock. The gate is captured by shared_ptr so that a
  /// callback the executor has already picked up can still refuse or exit
  /// cleanly after the backend itself is gone — that is the whole point, and
  /// why the gate is not reached through `this`.
  ///
  /// A lane created with a bare lambda compiles and runs; what it loses is the
  /// drain, and integrated_bringup's test_state_lane_close exists to catch it.
  template <typename F>
  [[nodiscard]] auto GuardStateLane(F&& callback) const {
    if (state_lane_gate_->IsClosed()) {
      // A closed gate never reopens: a lane wrapped now would take every
      // message and drop it, looking healthy from the outside. CM never
      // reconfigures a backend instance (a fresh one per configure), so this
      // is an embedder's mistake, named here rather than at the watchdog.
      RCLCPP_WARN(rclcpp::get_logger("rtc_controller_manager"),
                  "GuardStateLane on a closed state lane: this subscription will drop every "
                  "message (CloseStateLane is final — configure a new backend instead)");
    }
    using Callback = std::decay_t<F>;
    using Traits = rclcpp::function_traits::function_traits<Callback>;
    static_assert(Traits::arity == 1,
                  "GuardStateLane: a state-lane callback takes exactly one argument, the message");
    using Message = typename Traits::template argument_type<0>;
    return [gate = state_lane_gate_, callback = Callback(std::forward<F>(callback))](Message msg) {
      const StateLaneEntry entry(*gate);
      if (!entry) {
        return;  // closed: the backend is being torn down
      }
      callback(std::forward<Message>(msg));
    };
  }

  /// Backends call this at the end of each sub callback (joint / motor /
  /// sensor lane). Safe to call when no callback is registered.
  ///
  /// Also marks the state/publish split for StateLaneTimingScope: everything
  /// before this point is decode, everything after is the mailbox hand-off.
  /// Stamped before the callback runs so the callback's own cost lands in
  /// `t_publish_us`.
  void NotifyStateReady() noexcept {
    if (state_lane_timing_ != nullptr) {
      state_lane_notify_ns_ = StateLaneNowNs();
    }
    if (state_ready_cb_)
      state_ready_cb_();
  }

  /// RAII span covering one state-lane callback, pushed to the rt_callback
  /// timing CSV (issue #349). Declare it as the first statement of every
  /// joint / motor / sensor callback:
  ///
  ///     void MyBackend::OnJointState(Msg::SharedPtr msg) {
  ///       RTC_TRACE_SCOPE("MyBackend::OnJointState");
  ///       StateLaneTimingScope timing_scope(*this);
  ///       ...
  ///
  /// RAII rather than a begin/end pair because these callbacks early-return
  /// on empty payloads; a paired call would leak an unmatched begin on every
  /// such message and silently skew the lane.
  ///
  /// RT-safe (this runs on cb_group_rt_callback_, SCHED_FIFO 70): two
  /// steady_clock reads, one wait-free SPSC push, no allocation, no lock, no
  /// logging. On a full ring the sample is dropped and CM reports the count
  /// in its timing summary.
  class StateLaneTimingScope {
   public:
    explicit StateLaneTimingScope(DeviceBackend& backend) noexcept
        : backend_(backend.state_lane_timing_ != nullptr ? &backend : nullptr) {
      if (backend_ != nullptr) {
        // No need to clear the split marker here: steady_clock is monotonic,
        // so a stamp left by an earlier callback necessarily predates
        // t_begin_ns_ and the destructor's range check rejects it. A reset
        // was tried and removed — mutation testing showed nothing could
        // distinguish it, because the range check is the actual defence.
        t_begin_ns_ = StateLaneNowNs();
      }
    }

    ~StateLaneTimingScope() noexcept {
      if (backend_ == nullptr) {
        return;
      }
      const std::uint64_t t_end_ns = StateLaneNowNs();
      const std::uint64_t notify_ns = backend_->state_lane_notify_ns_;
      // Lanes that do not notify (and early returns taken before the notify)
      // collapse the split to the end of the callback: all decode, no
      // hand-off.
      //
      // The range check carries that case. `state_lane_notify_ns_` persists
      // across callbacks, so on a non-notifying lane it still holds whatever
      // the previous callback wrote; being older than t_begin_ns_ it falls
      // outside the span and is discarded. Dropping this check does not merely
      // mis-split — `split_ns - t_begin_ns_` is unsigned, so a stale stamp
      // wraps t_state_us to a ~10^10 µs value. That is the failure this
      // guards, and test_state_lane_timing_scope pins it.
      const std::uint64_t split_ns =
          (notify_ns >= t_begin_ns_ && notify_ns <= t_end_ns) ? notify_ns : t_end_ns;

      constexpr double kNsToUs = 1e-3;
      RtTickTimingPayload payload{};
      payload.t_state_us = static_cast<double>(split_ns - t_begin_ns_) * kNsToUs;
      payload.t_publish_us = static_cast<double>(t_end_ns - split_ns) * kNsToUs;
      payload.t_total_us = static_cast<double>(t_end_ns - t_begin_ns_) * kNsToUs;
      // t_compute_us / jitter_us stay 0 — see RtCallbackTimingBuffer.
      static_cast<void>(backend_->state_lane_timing_->Push(payload));
    }

    StateLaneTimingScope(const StateLaneTimingScope&) = delete;
    StateLaneTimingScope& operator=(const StateLaneTimingScope&) = delete;
    StateLaneTimingScope(StateLaneTimingScope&&) = delete;
    StateLaneTimingScope& operator=(StateLaneTimingScope&&) = delete;

   private:
    /// Null when instrumentation is off — the whole scope is then inert.
    DeviceBackend* backend_;
    std::uint64_t t_begin_ns_{0};
  };

 private:
  static std::uint64_t StateLaneNowNs() noexcept {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());
  }

  StateReadyCallback state_ready_cb_;

  /// rt_callback-lane instrumentation (issue #349). Both members are touched
  /// only from the state-lane callbacks, which the MutuallyExclusive group
  /// serialises onto the rt_callback thread — single accessor, no atomics.
  RtCallbackTimingBuffer* state_lane_timing_{nullptr};
  std::uint64_t state_lane_notify_ns_{0};

  /// Shared with every GuardStateLane() wrapper this backend created (issue
  /// #941). Allocated once here, never copied on the RT path: the wrapper
  /// holds its own reference and dereferences it.
  std::shared_ptr<StateLaneGate> state_lane_gate_{std::make_shared<StateLaneGate>()};
};

}  // namespace rtc

#endif  // RTC_CONTROLLER_MANAGER_DEVICE_BACKEND_H_
