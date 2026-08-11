#ifndef RTC_CONTROLLERS_GRASP_GRASP_CONTROLLER_HPP_
#define RTC_CONTROLLERS_GRASP_GRASP_CONTROLLER_HPP_

#include "rtc_controllers/grasp/grasp_types.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <span>

namespace rtc::grasp {

/// Position-control-based adaptive PI force controller for a multi-finger hand.
///
/// Uses a scalar grasp parameter s in [0,1] per finger to linearly interpolate
/// between pre-defined open and close postures.  An outer-loop PI controller
/// with online stiffness estimation drives s to achieve the desired contact
/// force, measured via fingertip normal force sensors.
///
/// The finger count and each finger's DoF are runtime values supplied to
/// Init() (fingers may have different DoF, e.g. thumb:4/index:3/middle:2);
/// storage is fixed-capacity (kMaxGraspFingers / kMaxDoFPerFinger) so the
/// class remains ROS2-independent and RT-safe after Init().
class GraspController {
 public:
  GraspController() = default;

  /// Initialise with finger configurations and parameters.
  /// Must be called before Update().  The number of fingers is configs.size()
  /// (clamped to kMaxGraspFingers) and each finger's DoF is FingerConfig::dof.
  void Init(std::span<const FingerConfig> configs, const GraspParams& params);

  /// Main control update — call once per control cycle on the RT thread.
  ///
  /// FILTERING IS THE CALLER'S JOB. This class holds no filter: the force it is
  /// handed is what the PI law, the online stiffness estimate and the per-tick
  /// slip derivative all see, unsmoothed. The filter used to live here and was
  /// moved out because the only correct place for it is upstream — `‖LPF(F)‖`
  /// needs the three axes, and this interface has already collapsed them to a
  /// magnitude. Low-passing a magnitude cannot undo the collapse: |F| rectifies
  /// zero-mean axis noise into a positive DC offset (≈1.6σ for 3-axis Gaussian)
  /// that sits under f_contact_threshold forever. A caller that skips filtering
  /// entirely also puts raw wire artefacts — spikes, NaN — straight into an
  /// integrator and, through s, into joint commands.
  ///
  /// @param f_filtered  Filtered force magnitude [N] per finger; one entry per
  ///                    finger, in the same order as the Init() configs. Extra
  ///                    entries are ignored; missing entries are treated as 0.
  /// @param dt          Control period [s].
  /// @return Joint position commands (num_fingers × per-finger dof).
  [[nodiscard]] GraspJointCommands Update(std::span<const double> f_filtered, double dt) noexcept;

  /// Request grasp start.  If target_force > 0, overrides params_.f_target.
  void CommandGrasp(double target_force = 0.0) noexcept;

  /// Request grasp release.
  void CommandRelease() noexcept;

  /// Return the FSM to Idle and drop both pending requests, without touching the
  /// configuration Init() computed.
  ///
  /// For callers that own this controller but do not always run it: whoever stops
  /// calling Update() also stops advancing the FSM, so a grasp interrupted by a
  /// control-law change or a deactivate stays frozen mid-phase, with its request
  /// flags still armed. The next Update() would then resume a squeeze — against
  /// whatever the hand is holding by then — and any observer gating on phase()
  /// sees a non-Idle value that nothing can clear. Calling Reset() on the ticks
  /// where the PI law is not the one driving the hand, and at activation, turns
  /// "the FSM is idle whenever this controller is not running it" into an
  /// unconditional property instead of one that depends on every caller path.
  ///
  /// RT-safe: fixed-size loops, no allocation, no logging. The active target
  /// force is deliberately kept — it is a setpoint, not state, and a GRASP
  /// always supplies its own. Note that this no longer discards a filter tail:
  /// the upstream filter is shared with whatever else consumes fingertip force
  /// and is deliberately kept warm, so the first tick back under this law reads
  /// the settled force instead of ramping up from zero.
  void Reset() noexcept;

  /// Current state machine phase.
  [[nodiscard]] GraspPhase phase() const noexcept { return phase_; }

  /// Per-finger runtime states (for logging / monitoring). Spans the
  /// num_fingers_ active fingers configured at Init().
  [[nodiscard]] std::span<const FingerState> finger_states() const noexcept {
    return {fingers_.data(), static_cast<std::size_t>(num_fingers_)};
  }

  /// Active target force [N].
  [[nodiscard]] double target_force() const noexcept { return active_target_force_; }

  /// Runtime parameter adjustment.
  void set_target_force(double f) noexcept;
  void set_params(const GraspParams& params) noexcept;

 private:
  // ── Phase update functions ────────────────────────────────────────────────
  void UpdateIdle() noexcept;
  void UpdateApproaching(double dt) noexcept;
  void UpdateContact(double dt) noexcept;
  void UpdateForceControl(double dt) noexcept;
  void UpdateHolding(double dt) noexcept;
  void UpdateReleasing(double dt) noexcept;

  // ── Helpers ────────────────────────────────────────────────────────────────

  /// Linear interpolation: q(s) = (1-s)*q_open + s*q_close, for cfg.dof joints.
  /// Entries beyond cfg.dof are left zero.
  [[nodiscard]] static std::array<double, kMaxDoFPerFinger> InterpolatePosture(
      const FingerConfig& cfg, double s) noexcept;

  /// Clamp ds according to deformation guard logic.
  void ApplyDeformationGuard(int finger, double& ds) noexcept;

  /// Adaptive PI computation for one finger, returns ds.
  [[nodiscard]] double ComputeAdaptivePI(int finger, double dt) noexcept;

  /// Reset all per-finger state.
  void ResetFingers() noexcept;

  // ── State ──────────────────────────────────────────────────────────────────
  GraspPhase phase_{GraspPhase::kIdle};
  int num_fingers_{0};  // active finger count (set at Init, ≤ kMaxGraspFingers)
  std::array<FingerConfig, kMaxGraspFingers> configs_{};
  GraspParams params_{};
  std::array<FingerState, kMaxGraspFingers> fingers_{};

  // Timers
  double contact_settle_timer_{0.0};
  double force_settle_timer_{0.0};

  // Atomic flags for cross-thread command interface
  std::atomic<bool> grasp_requested_{false};
  std::atomic<bool> release_requested_{false};
  double active_target_force_{0.0};

  bool initialized_{false};

  // Small epsilon for stiffness estimation denominator check
  static constexpr double kDeltaSEpsilon = 1e-6;
};

}  // namespace rtc::grasp

#endif  // RTC_CONTROLLERS_GRASP_GRASP_CONTROLLER_HPP_
