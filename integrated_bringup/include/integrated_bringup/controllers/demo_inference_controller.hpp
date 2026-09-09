#pragma once

// ── DemoInferenceController — learned policy → joint commands ────────────────
//
// Runs an ONNX policy on the RT tick and turns its two output heads into an
// absolute arm joint target and a hand posture. The binding half of the pair
// whose core is `rtc_controllers/inference/policy_io.hpp`: this file knows
// `RTControllerInterface`, `ControllerState` and the inference engine; the core
// knows only spans.
//
// ── Why decimation instead of a 50 Hz control rate ─────────────────────────
// The policy wants one action per 20 ms of SIMULATED time. In this sim the RT
// loop is lock-step with MuJoCo — one control tick advances exactly
// `physics_timestep` (2 ms) of sim time, and `mujoco_simulator.cpp` derives the
// integrator step as `physics_timestep / n_substeps`. So ten ticks of the
// existing 500 Hz loop are exactly the 20 ms the policy expects, and running
// the model every tenth tick buys that cadence without touching `control_rate`,
// `rtc::kMinControlRateHz`, or the MJCF's `<option timestep>`. Lowering
// `control_rate` to 50 instead would ALSO desynchronise `ControllerState::dt`
// (which is `1 / control_rate`) from the sim time actually advanced per tick,
// silently scaling every rate-derived law in the process by ten.
//
// ── Why the engine is injected ─────────────────────────────────────────────
// `Init()` on the stub `OnnxEngine` (the one compiled when ONNX Runtime is
// absent) is a no-op that does not throw, and its `Run()` just returns false —
// a controller that trusted `Init()` would configure "successfully" and then
// hold position forever with no diagnostic. The gate is `is_initialized()`,
// checked in `on_configure`. Injection also lets the tests drive every success
// path with a deterministic fake, which matters because there is no model file
// yet.
//
// ── Failure policy ─────────────────────────────────────────────────────────
// Any invalid input, a failed `Run()`, or a non-finite output holds EVERY
// device at its measured position for that tick. Never partially: the policy
// observes the whole robot at once, so half an observation makes the other
// half's action meaningless. The one thing that is clamped rather than held is
// the hand posture scalar landing outside [0, 1] — the blend stays inside the
// two configured postures, so the command is safe, but it is warned about
// because an out-of-range scalar means the model's normalisation and the YAML
// disagree.

#include "integrated_bringup/support/combined_model_cache.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_controller_interface/rt_controller_interface.hpp"
#include "rtc_controllers/params/policy_io_params.hpp"
#include "rtc_inference/inference_engine.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <tf2_msgs/msg/tf_message.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>

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

/// Which feature ids this binding can produce, and how wide each one is.
///
/// Robot-specific by design — that is exactly why it lives here and not in
/// `rtc_controllers` (ARCH-1). Widths that depend on the loaded config
/// (`ur5e.position` is as wide as the primary device's joint roster) are
/// resolved at configure time, not hardcoded.
enum class PolicyFeature : std::uint8_t {
  kArmPosition,           ///< primary device measured joint positions [rad]
  kHandPosition,          ///< secondary device measured joint positions [rad]
  kFingertipForceNorm,    ///< ‖f‖ per fingertip group [N]; 0 when the lane is stale
  kPalmPosition,          ///< palm origin, in `palm.reference_frame` [m]
  kPalmOrientationXyzw,   ///< palm orientation, Hamilton, serialised x,y,z,w
  kObjectPosition,        ///< tracked object origin, in `object_pose.reference_frame` [m]
  kObjectOrientationXyzw  ///< tracked object orientation, Hamilton, x,y,z,w
};

/// Which frame a pose feature is expressed in.
///
/// The two are NOT interchangeable on this robot: its MJCF mounts `base` with
/// `quat="0 0 0 -1"`, a 180 deg rotation about z, so a pose read in the wrong
/// one has its x and y negated — and still looks like a perfectly ordinary
/// pose. Every consumer inside this repo works in `base`; the sim's object
/// ground truth is published in `world`.
enum class PoseFrame : std::uint8_t { kBase, kWorld };

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

  void LoadConfig(const YAML::Node& cfg) override;
  void OnDeviceConfigsSet() override;

  // ── Test seams ────────────────────────────────────────────────────────────
  // Deliberately narrow: the fake engine is injected through the constructor,
  // so tests need only to observe what the tick decided.

  /// How many times the engine has been asked to run since activation. The
  /// decimation contract is "exactly one call per `decimation` ticks", which is
  /// only observable as a count.
  [[nodiscard]] std::uint64_t InferenceCountForTesting() const noexcept {
    return inference_count_;
  }

  /// True when the last tick shipped a held (measured-position) command rather
  /// than a policy action.
  [[nodiscard]] bool LastTickHeldForTesting() const noexcept { return last_tick_held_; }

  /// True when the last policy evaluation clamped its posture scalar.
  [[nodiscard]] bool LastScalarClampedForTesting() const noexcept {
    return last_scalar_clamped_;
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
  /// Parse the parts of the schema that need the device rosters. Called from
  /// on_configure (Pass 3), never from LoadConfig (Pass 1) — see the comment at
  /// its definition. Throws `std::invalid_argument`; on_configure turns that
  /// into a configure FAILURE.
  void ApplyIoSchema(const YAML::Node& cfg);

  /// Resolve a YAML feature id to its element count, or <= 0 when this binding
  /// cannot produce it. Handed to `ParsePolicyIoParams` so an id this build
  /// does not know becomes a configure failure naming the id.
  [[nodiscard]] int FeatureSize(std::string_view id) const;

  /// Map a resolved feature id to its enum. Only called for ids `FeatureSize`
  /// already accepted.
  [[nodiscard]] static bool FeatureFromId(std::string_view id, PolicyFeature& out);

  /// Fill `input_buffer_` from `state`. Returns false the moment any required
  /// source is unreadable — the caller then holds without running the model.
  [[nodiscard]] bool PackObservation(const ControllerState& state, std::span<float> buf) noexcept;

  /// Build the model, register the palm and arm-root frames. Returns false on
  /// any failure, with the reason logged; on_configure turns that into FAILURE.
  [[nodiscard]] bool ConfigurePalmFk();

  /// Palm pose from the combined-model cache, already expressed in
  /// `palm_frame_`. Returns false when the cache is not ready or the pose is
  /// not finite — the caller turns that into a hold.
  [[nodiscard]] bool PalmPose(std::array<double, 3>& position,
                              std::array<double, 4>& orientation_xyzw) noexcept;

  /// Non-RT: the object-pose subscription callback. Matches the configured
  /// name against every transform in the message and publishes the result
  /// through the SeqLock.
  void OnObjectTransforms(const tf2_msgs::msg::TFMessage& msg) noexcept;

  /// Emit the hold command: the latched entry position, seeded from the last
  /// readable state. Latches on the first hold tick of a run and stays put
  /// until a policy action is accepted again.
  void HoldPosition(const ControllerState& state, ControllerOutput& out) noexcept;

  /// Apply position clamp + per-tick rate bound to one device's command, using
  /// the shared §7.3 joint command tail so the MUST ordering (clamp, then
  /// rebound against the measured base) is not re-derived here.
  void BoundDeviceCommand(int device_idx, std::span<const double> measured,
                          std::span<double> command, double dt) noexcept;

  // ── Config ────────────────────────────────────────────────────────────────
  CommandType command_type_{CommandType::kPosition};
  rtc::params::PolicyIoParams io_{};
  std::vector<PolicyFeature> feature_kinds_;  ///< parallel to io_.input_features

  std::string model_path_;
  std::string optimized_model_path_;
  int intra_op_threads_{1};
  bool allow_missing_model_{false};
  double joint_limit_margin_{0.0};

  /// Hand postures the scalar interpolates between, in secondary-device joint
  /// order. Both are validated against the device's position limits at
  /// configure — a posture outside the band would be silently pulled back by
  /// the command clamp, which turns "close" into "very nearly open" without
  /// any diagnostic.
  std::vector<double> posture_open_;
  std::vector<double> posture_close_;

  /// Index into `io_.output_names` for the two heads this binding consumes.
  /// Resolved once at configure so the tick does not search by name.
  int arm_output_idx_{-1};
  int hand_output_idx_{-1};

  // ── Runtime ───────────────────────────────────────────────────────────────
  std::unique_ptr<rtc::InferenceEngine> engine_;

  /// True when configure accepted an empty `model_path` under
  /// `allow_missing_model`. The tick then holds forever, by design, so the
  /// wiring can be brought up and smoke-tested before a policy file exists.
  bool hold_mode_{false};

  int arm_dof_{0};
  int hand_dof_{0};
  int num_fingertips_{0};
  /// Elements per sensor group in the hand's inference lane, resolved once.
  ///
  /// Read per tick it would have meant `GetDeviceNameConfig(GetSecondaryDeviceName())`
  /// on the RT path — and `GetSecondaryDeviceName()` returns a `std::string` BY
  /// VALUE. Today's group names ("p1b") fit in the small-string buffer so
  /// nothing allocates, which is exactly what makes it a trap: the day a device
  /// group is named something longer than the SSO threshold, the RT tick starts
  /// calling operator new and nothing in the code looks different (RT-1).
  int fingertip_stride_{0};

  std::uint64_t tick_{0};
  std::uint64_t inference_count_{0};
  bool last_tick_held_{true};
  bool last_scalar_clamped_{false};
  bool warned_scalar_range_{false};

  /// Set by SetDeviceTarget (mailbox delivery, off this controller's tick) and
  /// consumed once by the tick. Atomic because the two run on different
  /// threads; relaxed because it carries no data, only "say this once".
  std::atomic<bool> external_target_seen_{false};
  bool warned_external_target_{false};

  /// Last accepted policy action, held on the ticks between evaluations.
  std::array<double, kMaxArmDof> arm_action_{};
  std::array<double, kMaxHandDof> hand_action_{};
  bool have_action_{false};

  /// The position a hold commands, LATCHED when the hold begins.
  ///
  /// Not the continuously measured position, which is the obvious reading of
  /// "hold the current joint positions" and is wrong. Commanding whatever the
  /// joints currently read gives the position servo zero error every tick, so
  /// it produces no torque, and the arm sags under gravity — a zero-stiffness
  /// follower rather than a hold. Measured on the ur5e_p1b sim: 0.2 rad away
  /// from the startup pose and still creeping 0.015 rad per 8 s. Latching the
  /// entry position gives the servo something to pull against.
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

  // ── Palm FK ───────────────────────────────────────────────────────────────
  std::string urdf_path_;
  std::string palm_link_;
  PoseFrame palm_frame_{PoseFrame::kWorld};
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  CombinedModelCache combined_cache_;
  int palm_frame_idx_{-1};
  int arm_base_frame_idx_{-1};

  /// The static transform that makes `world` mean something. Read from YAML
  /// (`inference.base_pose_in_world`) rather than hardcoded: this robot's base
  /// is mounted 180 deg about z, but that is a fact about this mounting, and a
  /// wrong sign here produces poses that are plausible everywhere except in
  /// the policy's reasoning. Identity by default.
  pinocchio::SE3 world_from_base_{pinocchio::SE3::Identity()};

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
  PoseFrame object_frame_{PoseFrame::kWorld};
  double object_timeout_sec_{0.2};
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

  rclcpp::Logger logger_{rclcpp::get_logger("integrated_bringup.demo_inference_controller")};
};

}  // namespace integrated_bringup
