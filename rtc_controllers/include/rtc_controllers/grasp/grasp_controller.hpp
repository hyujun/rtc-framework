#ifndef RTC_CONTROLLERS_GRASP_GRASP_CONTROLLER_HPP_
#define RTC_CONTROLLERS_GRASP_GRASP_CONTROLLER_HPP_

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <span>

#include "rtc_base/filters/bessel_filter.hpp"
#include "rtc_controllers/grasp/grasp_types.hpp"

namespace rtc::grasp {

/// Position-control-based adaptive PI force controller for a 3-finger hand.
///
/// Uses a scalar grasp parameter s in [0,1] per finger to linearly interpolate
/// between pre-defined open and close postures.  An outer-loop PI controller
/// with online stiffness estimation drives s to achieve the desired contact
/// force, measured via fingertip normal force sensors.
///
/// This class is ROS2-independent and RT-safe after Init().
class GraspController {
public:
  GraspController() = default;

  /// Initialise with finger configurations and parameters.
  /// Must be called before Update().  Not RT-safe (may compute filter coeffs).
  void Init(const std::array<FingerConfig, kNumGraspFingers>& configs,
            const GraspParams& params);

  /// Main control update — call once per control cycle on the RT thread.
  /// @param f_raw  Force magnitude [N] per finger (3-axis norm).
  /// @param dt     Control period [s].
  /// @return Joint position commands for 3 fingers x 3 DOF.
  [[nodiscard]] GraspJointCommands Update(
    std::span<const double, kNumGraspFingers> f_raw, double dt) noexcept;

  /// Request grasp start.  If target_force > 0, overrides params_.f_target.
  void CommandGrasp(double target_force = 0.0) noexcept;

  /// Request grasp release.
  void CommandRelease() noexcept;

  /// Current state machine phase.
  [[nodiscard]] GraspPhase phase() const noexcept { return phase_; }

  /// Per-finger runtime states (for logging / monitoring).
  [[nodiscard]] const std::array<FingerState, kNumGraspFingers>&
  finger_states() const noexcept { return fingers_; }

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

  /// Linear interpolation: q(s) = (1-s)*q_open + s*q_close
  [[nodiscard]] static std::array<double, kDoFPerFinger> InterpolatePosture(
    const FingerConfig& cfg, double s) noexcept;

  /// Clamp ds according to deformation guard logic.
  void ApplyDeformationGuard(int finger, double& ds) noexcept;

  /// Adaptive PI computation for one finger, returns ds.
  [[nodiscard]] double ComputeAdaptivePI(int finger, double dt) noexcept;

  /// Count fingers with contact_detected == true.
  [[nodiscard]] int CountContactFingers() const noexcept;

  /// Reset all per-finger state.
  void ResetFingers() noexcept;

  // ── State ──────────────────────────────────────────────────────────────────
  GraspPhase phase_{GraspPhase::kIdle};
  std::array<FingerConfig, kNumGraspFingers> configs_{};
  GraspParams params_{};
  std::array<FingerState, kNumGraspFingers> fingers_{};

  // 3-channel Bessel 4th-order LPF for force filtering
  BesselFilterN<kNumGraspFingers> force_filter_;

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
