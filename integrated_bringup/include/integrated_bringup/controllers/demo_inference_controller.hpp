#pragma once

// ── DemoInferenceController — learned policy → joint commands ────────────────
//
// Runs an ONNX policy on the RT tick and turns its output heads into absolute
// joint targets for the arm and the hand. The binding half of the pair whose
// core is `rtc_controllers/inference/policy_io.hpp`: this file knows
// `RTControllerInterface`, `ControllerState` and the inference engine; the core
// knows only spans.
//
// ── Why decimation instead of a lower control rate ─────────────────────────
// A policy wants one action per fixed interval of SIMULATED time. In this sim
// the RT loop is lock-step with MuJoCo — one control tick advances exactly
// `physics_timestep` of sim time, and `mujoco_simulator.cpp` derives the
// integrator step as `physics_timestep / n_substeps`. So running the model
// every N-th tick of the existing loop buys the trained cadence without
// touching `control_rate`, `rtc::kMinControlRateHz`, or the MJCF's `<option
// timestep>`. Lowering `control_rate` instead would ALSO desynchronise
// `ControllerState::dt` (which is `1 / control_rate`) from the sim time
// actually advanced per tick, silently scaling every rate-derived law by N.
//
// ── Why the engine is injected ─────────────────────────────────────────────
// `Init()` on the stub `OnnxEngine` (the one compiled when ONNX Runtime is
// absent) is a no-op that does not throw, and its `Run()` just returns false —
// a controller that trusted `Init()` would configure "successfully" and then
// hold position forever with no diagnostic. The gate is `is_initialized()`,
// checked in `on_configure`. Injection also lets the tests drive every success
// path with a deterministic fake, which is why no test needs a model file.
//
// ── What the policy observes, and in which frame ───────────────────────────
// Every pose the policy sees — link poses and the object pose — is expressed in
// ONE frame, named by `inference.policy_frame` after a frame of the system URDF.
// It is a name and not a transform on purpose: the frame a policy was trained in
// is a fact about the training asset, and on this robot the two candidates
// (`base` and `base_link`) differ by a half turn about z — a pose read in the
// wrong one has x and y negated and still looks ordinary. Naming a URDF frame
// lets the model, not a hand-typed rotation, supply that half turn.
//
// WHICH one a policy wants cannot be settled by reading the numbers: both make
// the trained nominal come out a round number, differing only in sign. What
// separates them is behaviour — under the wrong one the policy ANTI-TRACKS,
// sending the hand +y when the object moves −y, and never comes close enough to
// close on anything. Deciding it therefore costs a run, not an inspection.
//
// Link poses come from one of two places, decided per link at configure time:
// the combined-model cache when the link is upstream of every loop, or the
// closed-chain hand FK when a loop-passive joint sits between it and the root.
// The cache's model locks the loop-passive joints at zero, so reading a fingertip
// from it gives a pose that is finite, smooth, and several centimetres wrong.
//
// Joint lanes cross a JOINT CONVENTION on the way in and out
// (`inference.joint_convention`): the training asset may define a joint's axis
// opposite to this robot's URDF, which is a sign flip no range check can see.
//
// ── Failure policy ─────────────────────────────────────────────────────────
// Any invalid input, a failed `Run()`, or a non-finite output holds EVERY
// device at its latched position for that tick. Never partially: the policy
// observes the whole robot at once, so half an observation makes the other
// half's action meaningless. The one thing that is clamped rather than held is
// the hand posture scalar landing outside [0, 1] — the blend stays inside the
// two configured postures, so the command is safe, but it is warned about
// because an out-of-range scalar means the model's normalisation and the YAML
// disagree.

#include "integrated_bringup/logging/device_state_log_pod.hpp"
#include "integrated_bringup/logging/inference_diag_log_pod.hpp"
#include "integrated_bringup/support/closed_chain_hand_fk.hpp"
#include "integrated_bringup/support/combined_model_cache.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_interface/controller_log_set.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/inference/reach_gate.hpp"
#include "rtc_controllers/params/policy_io_params.hpp"
#include "rtc_controllers/params/reach_gate_params.hpp"
#include "rtc_inference/inference_engine.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <rclcpp/callback_group.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace integrated_bringup {

using rtc::CommandType;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::RTControllerInterface;

/// Which feature ids this binding can produce.
///
/// Robot-specific by design — that is exactly why it lives here and not in
/// `rtc_controllers` (ARCH-1). Widths that depend on the loaded config
/// (`ur5e.position` is as wide as the primary device's joint roster) are
/// resolved at configure time, not hardcoded.
enum class PolicyFeature : std::uint8_t {
  kArmPosition,           ///< primary device measured joint positions [rad], policy convention
  kHandPosition,          ///< secondary device measured joint positions [rad], policy convention
  kArmVelocity,           ///< primary device measured joint velocities [rad/s], policy convention
  kHandVelocity,          ///< secondary device measured joint velocities [rad/s], policy convention
  kFingertipForceNorm,    ///< ‖f‖ per fingertip group [N]; 0 when the lane is stale
  kGroupForceNorm,        ///< ‖f‖ of ONE sensor group [N]; 0 when the lane is stale
  kLinkPosition,          ///< link origin, in `policy_frame` [m]
  kLinkOrientationXyzw,   ///< link orientation in `policy_frame`, Hamilton, x,y,z,w
  kReachPhase,            ///< reach gate scalar (rtc_controllers/inference/reach_gate.hpp)
  kObjectPosition,        ///< tracked object origin, in `policy_frame` [m]
  kObjectOrientationXyzw  ///< tracked object orientation in `policy_frame`, x,y,z,w
};

/// What an output slice drives. Declared in YAML as `role:`, never inferred.
///
/// The binding this replaced read the role off the entry's name suffix in a
/// loop with no `break`, so a policy commanding two devices in the same role —
/// `ur5e.target_position` and `p1b.target_position`, the natural shape of a
/// multi-output policy — left whichever came last driving the arm. Both
/// commands stayed finite and inside the joint limits (#511 B-2, D-4).
enum class PolicyOutputRole : std::uint8_t {
  kJointTarget,   ///< absolute joint positions [rad], as wide as the device
  kPostureScalar  ///< one scalar interpolating hand_posture.open ↔ close
};

/// One object pose sample, as handed from the subscription callback (non-RT)
/// to the tick (RT) through a SeqLock. Trivially copyable by construction —
/// that is the SeqLock's type requirement, so no std::string lives here and
/// the matched frame name is resolved to a bool at write time.
struct ObjectPoseSample {
  std::array<double, 3> position{};
  std::array<double, 4> orientation_xyzw{};
  /// False when the last message carried no acceptable match: either nothing
  /// matched the configured name, or more than one thing did. "More than one"
  /// is a refusal rather than a pick-the-first, because the sim publishes every
  /// non-parked free body in one message and picking the first would silently
  /// track the wrong object the day the scene gains another.
  bool valid{false};
  /// Bumped on every accepted sample. The tick watches this rather than a
  /// clock: it needs no `now()` on the RT path and it stays on the same time
  /// axis as the simulation (age is accumulated in `dt`).
  std::uint32_t sequence{0};
};

class DemoInferenceController final : public RTControllerInterface {
 public:
  /// Fixed capacities. The tick path indexes these, so they bound every buffer
  /// the policy touches and nothing resizes after configure.
  static constexpr int kMaxArmDof = 16;
  static constexpr int kMaxHandDof = 32;
  static constexpr int kMaxFingertips = 8;
  static constexpr int kMaxInputElements = 512;
  static constexpr int kMaxOutputElements = 128;
  /// Input tensors the tick can address. Bounds the span table `PackObservation`
  /// walks, which is a stack array so the walk stays allocation-free (RT-1).
  /// `kMaxInputElements` is PER TENSOR, not a total: each tensor is its own
  /// engine-owned buffer. 24 because an export that splits every observation
  /// term into its own tensor (one per contact sensor, one per root-pose half)
  /// reaches 16 before it carries anything unusual.
  static constexpr int kMaxInputTensors = 24;
  /// Distinct links whose pose the policy (or the reach gate) reads. An export
  /// that lists every body of the robot is ~30 rows, so a policy that observed
  /// all of them would still fit.
  static constexpr int kMaxLinks = 32;
  /// Tips the reach gate averages over.
  static constexpr int kMaxReachTips = 8;
  /// Closure residual above which a closed-chain fingertip is not trusted this
  /// tick [m]. Handed to `ClosedChainHandFk::Configure` AND used for the tick's
  /// own freshness verdict, so the two can never disagree about a tick.
  static constexpr double kClosureErrorThreshold = 1e-3;
  /// Squared norm below which an incoming object quaternion carries no
  /// direction and normalising it would MANUFACTURE a NaN. Loose on purpose:
  /// this rejects garbage (all-zero, denormal), not an unnormalised-but-real
  /// orientation, which normalising is exactly the right answer for.
  static constexpr double kMinObjectQuatNormSq = 1e-12;

  /// @param urdf_path system URDF (as handed to every binding by the registry).
  /// @param engine    owned inference backend. Production passes an
  ///                  `rtc::OnnxEngine`; tests pass a deterministic fake. Never
  ///                  null — a null engine is refused at configure.
  DemoInferenceController(std::string_view urdf_path, std::unique_ptr<rtc::InferenceEngine> engine);

  ~DemoInferenceController() override = default;

  DemoInferenceController(const DemoInferenceController&) = delete;
  DemoInferenceController& operator=(const DemoInferenceController&) = delete;
  DemoInferenceController(DemoInferenceController&&) = delete;
  DemoInferenceController& operator=(DemoInferenceController&&) = delete;

  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;

  /// Records that an external joint goal arrived and otherwise ignores it.
  ///
  /// The policy owns the command stream, so an external target has no meaning
  /// while this controller is active. It cannot simply be left unimplemented:
  /// the controller must declare a `topics:` section (that is the only way to
  /// establish the device-group ORDER that `GetSecondaryDeviceName()` and the
  /// per-device limit table are indexed by), and the only legal subscribe role
  /// is `target` — so the subscription exists whether it is wanted or not.
  ///
  /// Ignoring it silently is the thing to avoid: an operator sending a goal and
  /// seeing nothing move has no way to tell that from a broken controller. The
  /// flag set here produces exactly one warning from the tick.
  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override {
    return "DemoInferenceController";
  }

  [[nodiscard]] CommandType GetCommandType() const noexcept override { return command_type_; }

  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev,
                              rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                              const YAML::Node& yaml) noexcept override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) noexcept override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) noexcept override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) noexcept override;

  void LoadConfig(const YAML::Node& cfg) override;
  void OnDeviceConfigsSet() override;

  /// Expand `${VAR}` and a leading `~` in @p raw (non-RT, configure time).
  ///
  /// Returns false when a referenced variable is unset or empty, naming it in
  /// @p missing — a policy directory that is not set on this machine is the
  /// ordinary state of a checkout without the model, and the caller decides
  /// whether that is a hold mode or a failure. An unterminated `${` or an
  /// invalid variable name throws `std::invalid_argument`: that is a typo in
  /// the config, not a property of the machine.
  [[nodiscard]] static bool ExpandModelPath(const std::string& raw, std::string& out,
                                            std::string& missing);

  // ── Test seams ────────────────────────────────────────────────────────────
  // Deliberately narrow: the fake engine is injected through the constructor,
  // so tests need only to observe what the tick decided.

  /// How many times the engine has been asked to run since activation. The
  /// decimation contract is "exactly one call per `decimation` ticks", which is
  /// only observable as a count.
  [[nodiscard]] std::uint64_t InferenceCountForTesting() const noexcept { return inference_count_; }

  /// True when the last tick shipped a held (latched-position) command rather
  /// than a policy action.
  [[nodiscard]] bool LastTickHeldForTesting() const noexcept { return last_tick_held_; }

  /// True when the last policy evaluation clamped its posture scalar.
  [[nodiscard]] bool LastScalarClampedForTesting() const noexcept { return last_scalar_clamped_; }

  /// The reach gate's tactile hold as of the last ACCEPTED policy step.
  [[nodiscard]] bool ReachHoldForTesting() const noexcept { return reach_state_.hold; }

  /// True when a link feature is served by the closed-chain hand FK.
  [[nodiscard]] bool ClosedChainLinksActiveForTesting() const noexcept {
    return has_closed_links_ && closed_fk_.active();
  }

  /// True when configure accepted a missing model and every tick holds.
  [[nodiscard]] bool HoldModeForTesting() const noexcept { return hold_mode_; }

  /// Why the last tick held (kNone when it emitted an action).
  [[nodiscard]] InferenceHoldReason LastHoldReasonForTesting() const noexcept {
    return last_hold_reason_;
  }

  /// Held ticks per reason since activation.
  [[nodiscard]] std::uint64_t HoldCountForTesting(InferenceHoldReason reason) const noexcept {
    const auto i = static_cast<std::size_t>(reason);
    return i < hold_counts_.size() ? hold_counts_[i] : 0;
  }

  /// Bind the diag channel to a caller-owned producer, so a test can read the
  /// rows the tick pushes without a session directory.
  /// The non-RT poll the log timer runs, for tests that drive Compute() by hand
  /// and have no executor spinning that timer.
  void PollDiagnosticsForTesting() { PollDiagnostics(); }

  /// How many times this activation has warned about a stalled projection —
  /// the once-per-activation contract is only checkable by counting.
  [[nodiscard]] int ClosedChainWarningsForTesting() const noexcept {
    return closed_chain_warnings_.load(std::memory_order_relaxed);
  }

  void SetInferenceDiagLogHandleForTesting(rtc::LogHandle<InferenceDiagLogPod> h) noexcept {
    inference_diag_log_handle_ = h;
  }

  /// Feed one TFMessage through the production callback. The subscription
  /// lambda calls exactly this, so a test needs no DDS round-trip to exercise
  /// the matching, frame and staleness rules.
  void InjectObjectTransformsForTesting(const tf2_msgs::msg::TFMessage& msg) noexcept {
    OnObjectTransforms(msg);
  }

  [[nodiscard]] const rtc::params::PolicyIoParams& IoParamsForTesting() const noexcept {
    return io_;
  }

 private:
  /// A feature id resolved against the device rosters (configure time only).
  struct ResolvedFeature {
    PolicyFeature kind{PolicyFeature::kArmPosition};
    int width{0};                   ///< element count, <= 0 when unknown
    int group{-1};                  ///< sensor group index (kGroupForceNorm)
    std::string link;               ///< link name (kLinkPosition / kLinkOrientationXyzw)
    std::vector<std::string> rows;  ///< row names for by-name placement, or empty
  };

  /// One link whose pose some consumer reads, and where the pose comes from.
  /// Exactly one of the two sources is set after configure.
  struct LinkSlot {
    std::string name;
    int cache_idx{-1};   ///< registered frame in `combined_cache_` (upstream of every loop)
    int closed_tip{-1};  ///< fingertip slot in `closed_fk_` (downstream of a loop)
  };

  /// What a recurrent input tensor is reset to. Parallel to
  /// `io_.recurrent_links`; `has_seed == false` resets to zeros.
  struct RecurrentSeed {
    int tensor{0};
    bool has_seed{false};
    PolicyFeature kind{PolicyFeature::kArmPosition};
    int arg{-1};
    int width{0};
  };

  /// Parse the parts of the schema that need the device rosters. Called from
  /// on_configure (Pass 3), never from LoadConfig (Pass 1) — see the comment at
  /// its definition. Throws `std::invalid_argument`; on_configure turns that
  /// into a configure FAILURE.
  void ApplyIoSchema(const YAML::Node& cfg);

  /// Resolve a YAML feature id. Returns false when this binding cannot produce
  /// it, which the schema parser turns into a configure failure naming the id.
  [[nodiscard]] bool ResolveFeature(std::string_view id, ResolvedFeature& out) const;

  /// Index of @p name in `links_`, appending it when new (configure time).
  int InternLink(const std::string& name);

  /// Fill every input tensor from `state`. `bufs[t]` is the engine's buffer for
  /// input tensor t, already length-checked by the caller. Returns false the
  /// moment any required source is unreadable.
  ///
  /// Order per evaluation: fill / constant → features → affine. The filler goes
  /// first so an element no feature covers holds its declared value rather than
  /// whatever the engine's allocation or a previous tick left there; the affine
  /// lane goes last, and only on a tensor whose features cover it completely
  /// (the parser refuses an affine lane on a partially covered tensor).
  ///
  /// A refusal DOES leave the segments packed before it in the engine's
  /// buffers, and that is safe because the caller holds without calling
  /// `Run()`, and the next attempt rewrites every non-recurrent element from
  /// the filler up.
  [[nodiscard]] bool PackObservation(const ControllerState& state,
                                     std::span<const std::span<float>> bufs) noexcept;

  /// Write one feature's values into `out`, which must be EXACTLY the feature's
  /// width. Returns false when its source is unreadable this tick or the width
  /// disagrees — a span wider than the lane would ship its tail from whatever
  /// the scratch buffer last held.
  [[nodiscard]] bool ExtractFeature(PolicyFeature kind, int arg, const ControllerState& state,
                                    std::span<double> out) noexcept;

  /// ‖f‖ of sensor group @p g in the hand's inference lane, or 0 when the group
  /// is stale or its slots are out of range.
  [[nodiscard]] double GroupForceNorm(const rtc::DeviceState& hand, int g) const noexcept;

  /// Pose of every entry of `links_`, in the policy frame, into `link_pos_` /
  /// `link_quat_`. False when any is unavailable this tick.
  [[nodiscard]] bool ComputeLinkPoses() noexcept;

  /// This step's reach gate from the link poses, forces and object pose. Writes
  /// the advanced trigger into `reach_state_pending_`; the caller commits it
  /// only when the step's action is accepted.
  [[nodiscard]] bool ComputeReachPhase(const ControllerState& state, double& phase) noexcept;

  /// Build the model and resolve every link, the policy frame and the object
  /// source frame. Returns false on any failure, with the reason logged;
  /// on_configure turns that into FAILURE.
  [[nodiscard]] bool ConfigureKinematics();

  /// Non-RT: the object-pose subscription callback. Matches the configured
  /// name against every transform in the message and publishes the result
  /// through the SeqLock.
  void OnObjectTransforms(const tf2_msgs::msg::TFMessage& msg) noexcept;

  /// Emit the hold command: the latched entry position, seeded from the last
  /// readable state. Latches on the first hold tick of a run and stays put
  /// until a policy action is accepted again. `reason` is recorded for the
  /// diagnostics (every hold path passes through here, so none can go unnamed).
  void HoldPosition(const ControllerState& state, ControllerOutput& out,
                    InferenceHoldReason reason) noexcept;

  /// The tick proper. `Compute()` wraps it so the logs are pushed at ONE place
  /// after whichever of its many early returns was taken.
  [[nodiscard]] ControllerOutput ComputeCommand(const ControllerState& state) noexcept;

  /// Push this tick's rows (RT path: wait-free, drop-on-full, no allocation).
  void PushLogs(const ControllerState& state, const ControllerOutput& out) noexcept;

  /// Non-RT. What the log-drain timer does besides draining: reads the counters
  /// the tick publishes and logs what only a human can act on.
  void PollDiagnostics();

  /// Close every CSV channel and unbind every handle (non-RT).
  void ResetLogState() noexcept;

  /// Apply position clamp + per-tick rate bound to one device's command, using
  /// the shared §7.3 joint command tail so the MUST ordering (clamp, then
  /// rebound against the base) is not re-derived here. `base` is the previous
  /// command (see `cmd_base_valid_`).
  void BoundDeviceCommand(int device_idx, std::span<const double> base, std::span<double> command,
                          double dt) noexcept;

  // ── Config ────────────────────────────────────────────────────────────────
  CommandType command_type_{CommandType::kPosition};
  rtc::params::PolicyIoParams io_{};
  /// Every input feature of every tensor, flattened in declaration order.
  /// `feature_kinds_[i]` is the extractor, `feature_args_[i]` its argument (a
  /// link slot or a sensor group, -1 otherwise) and `flat_segments_[i]` where it
  /// lands — the segment carries its own tensor index, so the tick routes by
  /// descriptor rather than by walking the grouped structure.
  std::vector<PolicyFeature> feature_kinds_;
  std::vector<int> feature_args_;
  std::vector<rtc::inference::InputSegment> flat_segments_;
  std::vector<RecurrentSeed> seeds_;

  /// `inference.model_path` as written, and after `${VAR}` / `~` expansion.
  std::string model_path_raw_;
  std::string model_path_;
  std::string optimized_model_path_;
  int intra_op_threads_{1};
  bool allow_missing_model_{false};
  double joint_limit_margin_{0.0};

  /// How long a HOLD may last before the recurrent state is reset on resume
  /// [s]. `0` resets after any hold, a NEGATIVE value never resets outside
  /// activation, and the default is 0.1 s (#511 D-3).
  ///
  /// Why a threshold and not one of the two extremes. A hidden state describes
  /// the situation the policy was last reasoning about; after a long hold that
  /// situation is gone, and resuming from it produces actions that are finite,
  /// in range, and about a moment that has passed — the failure has no signal.
  /// Resetting after EVERY hold is the other error: transient holds are not
  /// rare here (one hole in a device lane closes `IsDeviceReadable`), and
  /// erasing memory on each would gut the recurrence it exists to provide.
  double reset_after_hold_sec_{0.1};

  /// Hand postures the scalar interpolates between, in secondary-device joint
  /// order. Parsed in Pass 3 and only when a `posture_scalar` role is declared
  /// (D-9).
  std::vector<double> posture_open_;
  std::vector<double> posture_close_;

  /// Indices into `io_.output_features` for the two commands this binding
  /// emits, resolved once at configure so the tick does not search.
  int arm_target_idx_{-1};
  int hand_command_idx_{-1};
  PolicyOutputRole hand_role_{PolicyOutputRole::kPostureScalar};

  /// By-name gather into device joint order, when the output tensor names its
  /// elements (`ResolveNamedIndices`); empty = the positional slice.
  std::vector<int> arm_out_idx_;
  std::vector<int> hand_out_idx_;

  /// `q_policy = sign · q_device + offset` per joint, device order. Velocities
  /// take the sign only. Identity (1, 0) unless `inference.joint_convention`
  /// names the joint.
  std::array<double, kMaxArmDof> arm_sign_{};
  std::array<double, kMaxArmDof> arm_offset_{};
  std::array<double, kMaxHandDof> hand_sign_{};
  std::array<double, kMaxHandDof> hand_offset_{};

  // ── Runtime ───────────────────────────────────────────────────────────────
  std::unique_ptr<rtc::InferenceEngine> engine_;

  /// True when configure accepted a missing model under `allow_missing_model`.
  /// The tick then holds forever, by design, so the wiring can be brought up
  /// and smoke-tested on a machine without the policy file.
  bool hold_mode_{false};

  int arm_dof_{0};
  int hand_dof_{0};
  int num_fingertips_{0};
  /// Elements per sensor group in the hand's inference lane, resolved once.
  /// Read per tick it would mean `GetDeviceNameConfig(GetSecondaryDeviceName())`
  /// on the RT path, and that returns a `std::string` BY VALUE (RT-1 the day a
  /// group name outgrows the small-string buffer).
  int fingertip_stride_{0};

  std::uint64_t tick_{0};
  std::uint64_t inference_count_{0};
  bool last_tick_held_{true};
  /// Diagnostics of the current tick: why it held, and whether an action was
  /// accepted on it. Reset at the top of Compute(), read by PushLogs().
  InferenceHoldReason last_hold_reason_{InferenceHoldReason::kNone};
  bool policy_step_this_tick_{false};
  std::array<std::uint64_t, kNumInferenceHoldReasons> hold_counts_{};
  /// Set by PackObservation / ComputeLinkPoses immediately before they refuse,
  /// so the caller can name the hold without re-deriving why.
  InferenceHoldReason pack_failure_{InferenceHoldReason::kFeature};
  bool last_scalar_clamped_{false};
  bool warned_scalar_range_{false};

  /// Recurrent state bookkeeping. `hold_elapsed_sec_` accumulates on hold ticks
  /// and is cleared when an action is accepted; crossing
  /// `reset_after_hold_sec_` arms `recurrent_reset_pending_`. Every evaluation
  /// that finds it armed re-seeds the state tensors BEFORE packing, and only an
  /// ACCEPTED action disarms it — so a seed taken from the measured joints is
  /// taken on the step that actually runs, not on an earlier attempt that held.
  double hold_elapsed_sec_{0.0};
  bool recurrent_reset_pending_{true};
  bool warned_state_reset_{false};

  /// Set by SetDeviceTarget (mailbox delivery, off this controller's tick) and
  /// consumed once by the tick. Atomic because the two run on different
  /// threads; relaxed because it carries no data, only "say this once".
  std::atomic<bool> external_target_seen_{false};
  bool warned_external_target_{false};

  /// Last accepted policy action, device order and device convention, held on
  /// the ticks between evaluations.
  std::array<double, kMaxArmDof> arm_action_{};
  std::array<double, kMaxHandDof> hand_action_{};
  bool have_action_{false};

  /// The previous command, per device — the base the §7.3 rate bound steps
  /// from (D2).
  ///
  /// Not the measured position. With a measured base the command can lead the
  /// joint by at most one tick of `v_max`, so a position servo whose steady
  /// lag exceeds that is throttled by its OWN lag: on the ur5e_p1b sim (kp 2000,
  /// kv 400) the arm crept at ≈0.02 rad/s against a policy slewing at 0.5.
  ///
  /// Re-seeded from the measured position whenever the stream restarts — on
  /// activation and on the first action after a hold. Stepping from the hold
  /// latch instead would drag a joint that drifted during the hold back to
  /// where the hold began before letting it go anywhere.
  std::array<double, kMaxArmDof> last_cmd_arm_{};
  std::array<double, kMaxHandDof> last_cmd_hand_{};
  bool cmd_base_valid_{false};

  /// The position a hold commands, LATCHED when the hold begins.
  ///
  /// Not the continuously measured position, which is the obvious reading of
  /// "hold the current joint positions" and is wrong. Commanding whatever the
  /// joints currently read gives the position servo zero error every tick, so
  /// it produces no torque, and the arm sags under gravity — a zero-stiffness
  /// follower rather than a hold. Measured on the ur5e_p1b sim: 0.2 rad away
  /// from the startup pose and still creeping 0.015 rad per 8 s.
  ///
  /// Seeded from the last READABLE state, because a hold entered because the
  /// device went unreadable must not latch the unreadable reading.
  std::array<double, kMaxArmDof> hold_arm_{};
  std::array<double, kMaxHandDof> hold_hand_{};
  bool hold_latched_{false};
  std::array<double, kMaxArmDof> last_readable_arm_{};
  std::array<double, kMaxHandDof> last_readable_hand_{};
  bool have_readable_{false};

  /// Scratch. Fixed capacity, never resized on the tick path.
  std::array<double, kMaxArmDof + kMaxHandDof> scratch_measured_{};
  std::array<double, kMaxFingertips> scratch_force_norm_{};
  std::array<double, kMaxOutputElements> scratch_head_{};

  // ── Device limits (configure-time, indexed by device) ─────────────────────
  // Slot i is the i-th `topic_config_.groups` entry, which is the same index
  // the CM uses to fill `ControllerState::devices` — so device 0 is the arm and
  // device 1 the hand without this binding having to name either.
  std::array<std::vector<double>, rtc::ControllerState::kMaxDevices> device_position_lower_;
  std::array<std::vector<double>, rtc::ControllerState::kMaxDevices> device_position_upper_;
  std::array<std::vector<double>, rtc::ControllerState::kMaxDevices> device_max_velocity_;

  // ── Link poses ────────────────────────────────────────────────────────────
  std::string urdf_path_;
  /// The frame every pose feature is expressed in (a URDF frame name).
  std::string policy_frame_;
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  CombinedModelCache combined_cache_;
  int policy_frame_idx_{-1};
  int hand_root_idx_{-1};
  std::vector<LinkSlot> links_;
  /// Links downstream of a loop, served by the closed-chain projection. Updated
  /// on EVERY readable tick, not only on policy steps: its seed increment is
  /// clamped per update (NUM-5), so skipping ticks would turn ordinary motion
  /// into a walk-in and hold the policy for no reason.
  ClosedChainHandFk closed_fk_;
  bool has_closed_links_{false};
  /// This tick's projection was trustworthy (not held, finite, converged, not
  /// singular). The wrapper keeps serving its last good pose otherwise, which is
  /// right for a display and wrong for an observation.
  bool closed_fk_fresh_{false};
  /// This tick's projection status, for the diagnostics only — the control
  /// decision is `closed_fk_fresh_`. `closed_fk_ran_` is false on a tick that
  /// never reached the projection, so a stale status is not logged as current.
  rtc_urdf_bridge::RtClosedChainHandle::Status closed_fk_status_{};
  bool closed_fk_ran_{false};
  /// A projection that never walks in is the quietest failure this controller
  /// has: the fingertip poses stay finite and stale, every tick holds, and
  /// nothing says so. The tick only PUBLISHES the counter (RT-3 forbids it
  /// logging); `PollDiagnostics()`, on the 10 Hz non-RT timer, decides whether
  /// to speak, and speaks once per activation. `closed_chain_warn_ticks_ <= 0`
  /// disables it.
  std::atomic<std::int32_t> closed_chain_held_ticks_{0};
  int closed_chain_warn_ticks_{250};
  /// Written by the diagnostic poll and reset by `on_activate`, which run on
  /// DIFFERENT callback groups. They are serialised today only because the
  /// bring-up puts both on one single-threaded executor — that is a decision
  /// three files away, so the guarantee lives here instead.
  std::atomic<bool> closed_chain_warned_{false};
  std::atomic<int> closed_chain_warnings_{0};
  std::array<std::array<double, 3>, kMaxLinks> link_pos_{};
  std::array<std::array<double, 4>, kMaxLinks> link_quat_{};

  // ── Reach gate ────────────────────────────────────────────────────────────
  bool reach_enabled_{false};
  rtc::params::ReachGateParams reach_{};
  int reach_tips_{0};
  std::array<int, kMaxReachTips> reach_link_slot_{};
  std::array<int, kMaxReachTips> reach_force_group_{};
  /// c_i, object frame, packed xyz.
  std::array<double, std::size_t{3} * kMaxReachTips> reach_contacts_{};
  /// The trigger as of the last ACCEPTED step, and the one this step would
  /// advance it to. Two copies for the reason the recurrent feedback sits behind
  /// the `ok` gate: a step whose action is thrown away must not move the
  /// debounce counters, or a run of held steps would count as grasped ones.
  rtc::inference::ReachHoldState reach_state_{};
  rtc::inference::ReachHoldState reach_state_pending_{};
  bool reach_pending_{false};
  /// The gate value and mean tip distance the policy was shown, committed with
  /// the trigger (diagnostics only). NaN until the first accepted step.
  double reach_phase_pending_{InferenceDiagLogPod::kNaN};
  double tip_distance_pending_{InferenceDiagLogPod::kNaN};
  double last_reach_phase_{InferenceDiagLogPod::kNaN};
  double last_tip_distance_{InferenceDiagLogPod::kNaN};

  // ── Object pose lane ──────────────────────────────────────────────────────
  // Not a device lane, so none of the framework's freshness gates cover it —
  // this controller has to own the staleness question itself. `timeout_sec` is
  // configuration and not a constant because the two sources are an order of
  // magnitude apart: the sim republishes every tick (~500 Hz) while a real
  // perception stack runs at 10-30 Hz.
  std::string object_topic_;
  std::string object_frame_match_;
  bool object_match_prefix_{true};
  /// The frame the incoming transforms are expected to be expressed in,
  /// checked against `header.frame_id` on every message. Without this the
  /// controller would be silently ASSUMING the sim's `world`; a perception
  /// stack publishing the same message type in a different frame would produce
  /// poses that are finite, plausible, and rotated.
  std::string object_source_frame_id_;
  /// The URDF frame that `object_source_frame_id_` IS. The message speaks the
  /// publisher's frame names, the model speaks the URDF's; this key is the one
  /// place the two are joined, and it is what lets the model supply
  /// `pf_from_src_` instead of a hand-typed rotation.
  std::string object_source_frame_link_;
  /// Static transform policy_frame ← source frame, computed once at configure
  /// and applied in the (non-RT) callback. Identity-initialised (pinocchio 4's
  /// SE3 default constructor leaves the rotation uninitialised).
  pinocchio::SE3 pf_from_src_{pinocchio::SE3::Identity()};
  double object_timeout_sec_{0.2};
  bool wants_object_{false};
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr object_sub_;
  rtc::SeqLock<ObjectPoseSample> object_pose_lock_;
  std::uint32_t object_seq_written_{0};
  std::uint32_t last_object_seq_seen_{0};
  double object_age_sec_{0.0};
  bool object_ever_seen_{false};
  /// This tick's snapshot, taken once so both halves of the pose come from one
  /// message. A per-feature Load() could straddle a callback and pair a
  /// position with the next message's orientation — finite, plausible, wrong.
  ObjectPoseSample object_this_tick_{};
  bool object_valid_this_tick_{false};
  std::atomic<bool> object_frame_mismatch_{false};
  bool warned_object_frame_{false};

  // ── Controller-owned CSV logs (`logs:` in the YAML) ───────────────────────
  // Same schema and registration helper as the sibling demo controllers:
  // `<session>/controllers/demo_inference_controller/<instance>.csv`.
  struct ParsedLogEntry {
    std::string msg_type;
    std::string instance;
  };

  std::vector<ParsedLogEntry> parsed_log_entries_;
  rtc::ControllerLogSet log_set_{"demo_inference_controller"};
  rtc::LogHandle<DeviceStateLogPod> primary_state_log_handle_;
  rtc::LogHandle<DeviceStateLogPod> secondary_state_log_handle_;
  rtc::LogHandle<InferenceDiagLogPod> inference_diag_log_handle_;
  rclcpp::CallbackGroup::SharedPtr log_drain_cb_group_;
  rclcpp::TimerBase::SharedPtr log_drain_timer_;
  std::uint64_t log_drops_reported_{0};

  rclcpp::Logger logger_{rclcpp::get_logger("integrated_bringup.demo_inference_controller")};
};

}  // namespace integrated_bringup
