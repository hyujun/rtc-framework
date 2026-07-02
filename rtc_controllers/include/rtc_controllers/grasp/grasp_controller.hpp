#ifndef RTC_CONTROLLERS_GRASP_GRASP_CONTROLLER_HPP_
#define RTC_CONTROLLERS_GRASP_GRASP_CONTROLLER_HPP_

#include "rtc_base/filters/bessel_filter.hpp"
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
  /// Must be called before Update().  Not RT-safe (may compute filter coeffs).
  /// The number of fingers is configs.size() (clamped to kMaxGraspFingers) and
  /// each finger's DoF is FingerConfig::dof.
  void Init(std::span<const FingerConfig> configs, const GraspParams& params);

  /// Main control update — call once per control cycle on the RT thread.
  /// @param f_raw  Force magnitude [N] per finger (3-axis norm); one entry per
  ///               finger, in the same order as the Init() configs. Extra
  ///               entries are ignored; missing entries are treated as 0.
  /// @param dt     Control period [s].
  /// @return Joint position commands (num_fingers × per-finger dof).
  [[nodiscard]] GraspJointCommands Update(std::span<const double> f_raw, double dt) noexcept;

  /// Request grasp start.  If target_force > 0, overrides params_.f_target.
  void CommandGrasp(double target_force = 0.0) noexcept;

  /// Request grasp release.
  void CommandRelease() noexcept;

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

  // Per-channel Bessel 4th-order LPF for force filtering (one channel per
  // finger; only the first num_fingers_ channels carry real signal).
  BesselFilterN<kMaxGraspFingers> force_filter_;

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
