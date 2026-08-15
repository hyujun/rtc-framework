#ifndef UR5E_BRINGUP_SUPPORT_DEMO_SHARED_CONFIG_HPP_
#define UR5E_BRINGUP_SUPPORT_DEMO_SHARED_CONFIG_HPP_

#include "integrated_bringup/support/virtual_tcp.hpp"
#include "rtc_controllers/grasp/grasp_controller.hpp"
#include "rtc_controllers/grasp/grasp_types.hpp"
#include "rtc_controllers/grasp/pull_force_estimator.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace integrated_bringup {

// Hand grasp-intervention mode, resolved once from the whitelisted
// `grasp_controller_type` string at on_configure so the RT hot path branches on
// an enum instead of re-comparing a std::string every tick.
//   kContactStop — freeze hand trajectory on contact (default)
//   kForcePi     — adaptive PI force controller
//   kNone        — no hand intervention (trajectory passes through)
enum class GraspHandMode { kContactStop, kForcePi, kNone };

// Parse + validate the `grasp_controller_type` whitelist. Throws
// std::runtime_error on any value outside {force_pi, contact_stop, none}; runs
// on the non-RT on_configure path and propagates to CallbackReturn::FAILURE.
[[nodiscard]] GraspHandMode ParseGraspHandMode(const std::string& type);

// Canonical string for a mode (logging). Never allocates.
[[nodiscard]] const char* GraspHandModeName(GraspHandMode mode) noexcept;

// Where the object-plane normal fed to PullForceEstimator::Update comes from
// (#167). Resolved once from the whitelisted `pull_estimator.plane_normal_source`
// string at on_configure so the RT hot path branches on an enum.
//   kFixed         — constant normal from YAML (robot base frame)
//   kPinchGeometry — n = (p_IM - p_T)/‖·‖ from per-tick fingertip positions
enum class PullPlaneNormalSource { kFixed, kPinchGeometry };

// Parse + validate the `plane_normal_source` whitelist. Throws
// std::runtime_error on any value outside {fixed, pinch_geometry} ("vision" is
// a planned future source and is rejected until implemented); runs on the
// non-RT on_configure path and propagates to CallbackReturn::FAILURE.
[[nodiscard]] PullPlaneNormalSource ParsePullPlaneNormalSource(const std::string& source);

// Canonical string for a source (logging). Never allocates.
[[nodiscard]] const char* PullPlaneNormalSourceName(PullPlaneNormalSource source) noexcept;

// Default cutoff for the Force-PI force feedback LPF. Named because the two
// controllers need it before any YAML is read (their constructors Init the
// filter bank so the RT path never sees un-Init'd coefficients) and because it
// is now the sole home of a default that used to live in GraspParams.
inline constexpr double kDefaultForcePiLpfCutoffHz = 25.0;

// Default momentum-observer gain K_I [1/s] when a `momentum_observer` block
// carries no `gains` key (#135 Layer 1b). 10 /s puts the residual's first-order
// time constant at 0.1 s — slow enough that encoder/torque noise does not
// dominate, fast enough that a quasi-static payload change settles inside a few
// hundred ms. It is a starting point, not an identified value: K_I is the
// responsiveness-vs-noise knob and every deployment is expected to set it.
inline constexpr double kDefaultMomentumObserverGain = 10.0;

// The `momentum_observer` YAML block (#135 Layer 1b), mirrored as a struct so
// the three demo controllers can carry it from LoadConfig (where the YAML is
// read) to OnDeviceConfigsSet (where the arm device's joint order first
// exists), which is the earliest point the wiring can be built.
//
// No block ⇒ disabled, and that is the behaviour for any variant that does not
// opt in — the observer costs one arm-sized dynamics pass per tick (crba +
// computeCoriolisMatrix + gravity), so it is not something a config should
// acquire by accident.
struct MomentumObserverParams {
  bool has_block{false};
  bool enabled{true};
  // K_I per arm joint [1/s]. Empty ⇒ kDefaultMomentumObserverGain, one entry
  // broadcasts to every joint, otherwise exactly the arm's dof. Any other
  // length is a config error the wiring rejects at configure rather than
  // truncating or padding, since either would silently leave the tail joints
  // on a gain nobody chose and the residual it produces looks reasonable.
  // Larger is faster to converge and noisier; a zero or negative entry is
  // rejected by MomentumObserver::Init (it would pin that component of r at 0
  // forever, which is indistinguishable from a measured absence of load).
  std::vector<double> gains{};
};

// Parameters that DemoJointController and DemoTaskController share.
// Defaults live in config/ur5e_p1a/controllers/demo_shared.yaml; per-controller YAMLs
// may override individual keys via a second ApplyDemoSharedConfig() pass.
struct DemoSharedConfig {
  VirtualTcpConfig vtcp{};
  // grasp_contact_threshold gates native contact_flag probability (sensor A
  // path only — udp_hand_native + ft_inferencer.enabled). On sensor B (force-
  // only backends like mujoco wrench), controllers skip this check and rely
  // on grasp_force_threshold alone.
  float grasp_contact_threshold{0.5f};
  // grasp_force_threshold and grasp_min_fingertips are common across sensor
  // types. force_threshold gates |F| > N; min_fingertips_for_grasp = N
  // decides grasp_detected.
  float grasp_force_threshold{1.0f};
  int grasp_min_fingertips{2};

  std::string grasp_controller_type{"contact_stop"};

  // Hand joint topology — per-finger hand-joint index groups (ragged; fingers
  // may have different DoF, e.g. proto_1b thumb:4/index:3/middle:2/ring:1).
  // hand_finger_joint_map[f][0 .. finger_dof[f]) = index into device 1 (hand)
  // joint arrays. num_grasp_fingers rows are valid. Loaded from demo_shared.yaml
  // `hand_finger_joint_map`; default mirrors the assm_v1 3-finger × 3-DoF layout.
  int num_grasp_fingers{3};
  std::array<int, rtc::grasp::kMaxGraspFingers> finger_dof{{3, 3, 3}};
  std::array<std::array<int, rtc::grasp::kMaxDoFPerFinger>, rtc::grasp::kMaxGraspFingers>
      hand_finger_joint_map{{{{0, 1, 2}}, {{3, 4, 5}}, {{6, 7, 8}}}};

  // contact_stop release-phase gate, per finger. joint_index = the hand-joint
  // index whose motion signals loosening; loosen_sign encodes the convention:
  //   +1 → 각도 증가 시 이완 (e.g. thumb CMC_FE)
  //   -1 → 각도 감소 시 이완 (e.g. index/middle MCP_FE)
  // release_phase = 모든 gate finger 가 이완 방향일 때. num_release_gates 유효.
  struct HandReleaseGate {
    int joint_index{0};
    int loosen_sign{+1};
  };

  int num_release_gates{3};
  std::array<HandReleaseGate, rtc::grasp::kMaxGraspFingers> hand_release_gate{
      {{1, +1}, {4, -1}, {7, -1}}};

  // Force-PI grasp parameters; only consumed when grasp_controller_type ==
  // "force_pi". force_pi_fingers[0 .. num_grasp_fingers) are valid; each
  // FingerConfig::dof gives that finger's joint count.
  rtc::grasp::GraspParams force_pi_params{};
  std::array<rtc::grasp::FingerConfig, rtc::grasp::kMaxGraspFingers> force_pi_fingers{};
  bool has_force_pi_block{false};
  /// Bessel LPF cutoff [Hz] for the force feedback the Force-PI law servos on.
  /// YAML key is still `force_pi_grasp.lpf_cutoff_hz` — only the filter's
  /// *location* moved. It is applied PER AXIS in the controller (upstream of the
  /// |F| collapse) rather than to the already-collapsed magnitude inside
  /// GraspController, because `‖LPF(F)‖` needs the three axes and
  /// `GraspController::Update()` only ever sees a magnitude. Filtering after the
  /// norm leaves a rectification bias: zero-mean sensor noise of σ per axis
  /// shows up as a ≈1.6·σ phantom force floor that no low-pass can remove.
  double force_pi_lpf_cutoff_hz{kDefaultForcePiLpfCutoffHz};

  // ── In-plane pull-force estimator (#167) ──────────────────────────────────
  // Parsed from the `pull_estimator` YAML block; consumed by
  // BuildPullForceEstimator. pull_contacts / pull_tip_roles / pull_tip_links
  // [0 .. num_pull_contacts) are valid, in `tips.tip_names` order — the same
  // order P3 wiring must use for PullContactInput. pull_tip_links carry the
  // fingertip link names to resolve FK slots against the tree-model
  // `tip_links` at on_configure; roles feed the pinch-geometry normal (thumb
  // vs opposing tips) and the required-role mask.
  bool has_pull_estimator_block{false};
  bool pull_estimator_enabled{true};
  int num_pull_contacts{0};
  rtc::grasp::PullEstimatorParams pull_estimator_params{};
  std::array<rtc::grasp::PullContactConfig, rtc::grasp::kMaxPullContacts> pull_contacts{};
  std::array<std::string, rtc::grasp::kMaxPullContacts> pull_tip_roles{};
  std::array<std::string, rtc::grasp::kMaxPullContacts> pull_tip_links{};
  PullPlaneNormalSource pull_plane_normal_source{PullPlaneNormalSource::kFixed};
  // Fixed-source object-plane normal, robot base frame (fingertip FK rotations
  // are T_base_fingertip, so all reference-frame vectors here are base-frame).
  Eigen::Vector3d pull_plane_normal{Eigen::Vector3d::UnitZ()};
  // Arm the grasp-time baseline snapshot instead of (or on top of) the
  // gravity_force model — removes in-plane gravity + constant calib residual.
  bool pull_use_baseline_subtraction{false};
  // Optional known pull direction (base frame) for the directional scalar F_d.
  bool pull_has_direction{false};
  Eigen::Vector3d pull_direction{Eigen::Vector3d::Zero()};

  // ── Generalized-momentum observer (#135 Layer 1b) ─────────────────────────
  MomentumObserverParams momentum_observer{};
};

// Overlay any keys present in `node` onto `cfg`. Missing keys are left
// untouched, so callers can stack defaults -> shared file -> per-controller
// overrides.
void ApplyDemoSharedConfig(const YAML::Node& node, DemoSharedConfig& cfg);

// Load demo_shared.yaml from the integrated_bringup package share dir.
// On error (file missing / parse failure), `cfg` is left unchanged.
//
// `config_variant` mirrors the rt_controller_node `config_variant` ROS
// parameter (e.g. "ur5e_p1a", "iiwa7_leap").  Empty → legacy flat layout
// (config/controllers/demo_shared.yaml); non-empty → variant path
// (config/<config_variant>/controllers/demo_shared.yaml).  Per-controller call
// sites forward the value from their LifecycleNode's `config_variant` param.
void LoadDemoSharedYamlFile(DemoSharedConfig& cfg, const std::string& config_variant = "");

// Build (or reset) the GraspController based on `cfg`. Resets to nullptr unless a
// force-pi block was provided.
//
// The MODE is deliberately not consulted. What a force-pi block buys is the
// *capability* to run the PI law; which law actually drives the hand each tick is
// `Gains::grasp_hand_mode`, and that is now writable at runtime — so a build that
// skipped construction under contact_stop would make the mode a one-way door.
// Every consumer that used to read `grasp_controller_ != nullptr` as "force_pi is
// running" must therefore check the mode itself: the control law and the
// diagnostics fill do (see the two controllers' compute paths), and the
// grasp_command service uses GraspCommandRejectReason below.
void BuildGraspController(const DemoSharedConfig& cfg,
                          std::unique_ptr<rtc::grasp::GraspController>& grasp_controller);

/// Why a `grasp_command` (GRASP / RELEASE) must be refused right now, or nullptr
/// to accept. Never allocates; the returned string is a literal.
///
/// `has_controller` alone is not the answer any more. The FSM this service steps
/// only reaches the hand through the force_pi control law, so under contact_stop
/// or none a command would be accepted, answer "grasp started", step the FSM and
/// change nothing the hand can feel. That is reachable in a shipped config:
/// ur5e_p1b carries a force_pi_grasp block with `grasp_controller_type: "none"`.
[[nodiscard]] const char* GraspCommandRejectReason(GraspHandMode mode,
                                                   bool has_controller) noexcept;

/// What a controller can tell the quiet gate about its hand right now. Every
/// field is observable off the RT thread without racing it: the two mirrors are
/// atomics the RT tick stores, and `has_controller` is fixed at configure time.
struct GraspModeSwitchState {
  bool has_controller{false};   ///< a force_pi_grasp block was configured
  bool contact_latched{false};  ///< contact_stop hold engaged (RT mirror)
  bool grasp_phase_idle{true};  ///< PI FSM at kIdle (true when there is no FSM)
};

/// Why a runtime `grasp_controller_type` change must be refused right now, or
/// nullptr to accept. Never allocates; the returned string is a literal.
///
/// Policy (confirmed 2026-07-30): the mode may only move while the hand is
/// QUIET. The rejected alternative was "always allow + auto-safe", which needs
/// more RT code and turns a missing re-seed into the hazard itself. Quiet also
/// buys something else: leaving force_pi is only possible at kIdle, so the PI
/// diagnostics left in the published GraspState are an idle-consistent setpoint
/// rather than a stale mid-grasp state — which is why nothing clears them.
///
/// A request for the mode already in force is never refused: it changes nothing,
/// so refusing it would make an idempotent set fail while an object is held.
[[nodiscard]] const char* GraspModeChangeRejectReason(GraspHandMode current,
                                                      GraspHandMode requested,
                                                      const GraspModeSwitchState& state) noexcept;

// Build (or reset) the PullForceEstimator based on `cfg` (#167). Resets to
// nullptr unless a pull_estimator block was provided and `enabled` is true.
// Overwrites sample_rate_hz with `control_rate_hz` (YAML never sets it) and
// applies the optional pull direction. Init() validation failures throw
// std::invalid_argument (non-RT on_configure path → CallbackReturn::FAILURE).
void BuildPullForceEstimator(const DemoSharedConfig& cfg, double control_rate_hz,
                             std::unique_ptr<rtc::grasp::PullForceEstimator>& estimator);

}  // namespace integrated_bringup

#endif  // UR5E_BRINGUP_SUPPORT_DEMO_SHARED_CONFIG_HPP_
