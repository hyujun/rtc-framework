#ifndef INTEGRATED_BRINGUP_CONTROLLERS_WBC_FORCE_REFERENCE_UPDATER_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_WBC_FORCE_REFERENCE_UPDATER_HPP_

// ForceReferenceUpdater — per-contact normal-force PI for TSID ForceTask.
//
// Closed-loop replacement for the open-loop λ_des = f_des push that
// phase.cpp does at kClosure/kHold entry. Runs every RT tick:
//   e_i = f_des - |f_meas_i|
//   I_i = clamp(I_i + ki * e_i * dt, -i_max, +i_max)
//   λ_des_i_n = clamp(kp * e_i + I_i + f_des, lambda_min, lambda_max)
// and writes the +Z slot of each active contact's λ block.
//
// RT contract: alloc-free, exception-free, fixed-size storage indexed by
// contact_index in [0, kMaxContacts). No heap, no logging, no locks.
// All inputs/outputs are plain values; the controller owns the wiring to
// FingertipSensorData + ForceTask::SetForceReferences.

#include <Eigen/Core>

#include <array>
#include <cstddef>

namespace integrated_bringup::wbc {

// Bound on the number of WBC contacts. Matches kMaxWbcFingertips (8) so
// the helper can be reused if/when surface contacts are added.
inline constexpr int kMaxForceContacts = 8;

struct ForceReferenceUpdaterConfig {
  double kp{0.5};             // proportional gain on |f_meas| error [N/N]
  double ki{2.0};             // integral gain [1/s] — I_i units: N
  double i_max{2.0};          // anti-windup clamp on integrator [N]
  double lambda_min{0.0};     // output saturation lower bound [N]
  double lambda_max{20.0};    // output saturation upper bound [N]
  double f_des_default{2.0};  // fallback target if gains.grasp_target_force <= 0
};

class ForceReferenceUpdater {
 public:
  // Reset all integrators + first-tick flags. Call on phase entry into
  // kClosure (RT-safe — only touches POD members).
  void Reset() noexcept;

  // Replace gains/limits. RT-safe (POD copy).
  void SetConfig(const ForceReferenceUpdaterConfig& cfg) noexcept { cfg_ = cfg; }

  [[nodiscard]] const ForceReferenceUpdaterConfig& GetConfig() const noexcept { return cfg_; }

  // Update a single contact's integrator. Returns the new λ_n value (the
  // +Z component to push into the contact's λ block).
  //
  //   contact_index ∈ [0, kMaxForceContacts). Out-of-range → returns 0.
  //   measured_valid : if false, integrator is reset and 0 is returned
  //                    (caller should also zero the λ block for this contact).
  //   f_des          : per-contact target force [N]. Negative → cfg.f_des_default.
  //   f_meas         : measured |f| at the fingertip [N].
  //   dt             : tick period [s]; non-positive → no integrator advance.
  [[nodiscard]] double Update(int contact_index, bool measured_valid, double f_des, double f_meas,
                              double dt) noexcept;

  // Test accessors (not RT-safe semantically — fine for unit tests).
  [[nodiscard]] double GetIntegratorForTesting(int contact_index) const noexcept;
  [[nodiscard]] double GetLastOutputForTesting(int contact_index) const noexcept;

 private:
  ForceReferenceUpdaterConfig cfg_{};
  std::array<double, kMaxForceContacts> integrator_{};
  std::array<double, kMaxForceContacts> last_output_{};
};

}  // namespace integrated_bringup::wbc

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_WBC_FORCE_REFERENCE_UPDATER_HPP_
