// ── The pose guard of locomotion_controller ──────────────────────────────────
//
// Before the controller latches a standing reference from the measured pose —
// at an activation, after a fault reset, after an E-STOP clear, and when a
// plan is lost — it asks whether that pose is one to stand on. A reference
// latched from a robot that is tipping over, or from a base lane that holds
// no real sample, would command the whole-body QP to hold a fall.
//
// EvaluatePoseGuard is a pure function of the measured base and the contact
// heights: no controller state, no clock, so the tests can hand it a NaN, the
// zero quaternion or a pose a millionth of a radian off the limit directly
// (the state source refuses most of those before the controller sees them).
// Every comparison is written in the POSITIVE form (`r22 >= cos_tilt`,
// `spread <= max`), so a NaN fails it. The verdict is a bit mask (0 = pass);
// the controller logs it as `pose_guard_mask`.
//
// Two uses, one function. The stance entries (activation · reset · clear)
// check the base lane, finiteness, the unit norm and the tilt. A plan lost
// off the MPC also checks the stance the plan left behind (`check_stance`):
// every contact frame within `max_spread_m` of the lowest and the base slower
// than `max_speed` — a swing foot or a moving base is not re-latched.
//
// The tilt: body z · world z = R(2,2) = 1 − 2 (x² + y²) / ‖q‖² of the quaternion
// (x, y, z, w), compared with cos(base_tilt_rad) computed once by the caller —
// written as ‖q‖² − 2 (x² + y²) ≥ cos θ · ‖q‖² (no division, NaN fails it), so
// a quaternion inside the norm tolerance is judged on its direction, not on
// how far off unit it is. At base_tilt_rad = π/2 (the largest limit accepted)
// a body is refused once it tips past horizontal; the exact horizontal is a
// rounding coin toss there.
#ifndef INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_POSE_GUARD_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_POSE_GUARD_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace integrated_bringup::locomotion {

/// Bits of a guard verdict (0 = the pose passes).
enum PoseGuardBit : std::uint32_t {
  kPoseGuardBaseInvalid = 1U << 0,  ///< the base lane holds no real sample
  kPoseGuardNonFinite =
      1U << 1,  ///< a quaternion component, base velocity or contact height is not finite
  kPoseGuardQuaternion =
      1U << 2,  ///< |‖q‖² − 1| ≥ kPoseGuardQuaternionTolerance (the zero quaternion included)
  kPoseGuardTilt = 1U << 3,  ///< body z · world z < cos(base_tilt_rad)
  kPoseGuardSpread =
      1U << 4,  ///< stance only: contact heights further apart than max_spread_m, or no contact
  kPoseGuardSpeed = 1U << 5,  ///< stance only: base speed ≥ max_speed
};

/// A unit quaternion to this tolerance on its squared norm.
inline constexpr double kPoseGuardQuaternionTolerance = 1e-6;

struct PoseGuardInput {
  bool base_valid{false};          ///< the base lane holds a real sample
  std::array<double, 4> q_xyzw{};  ///< measured base orientation
  std::array<double, 3> v_base{};  ///< measured base linear velocity (only its norm is read) [m/s]
  const double* contact_z{
      nullptr};  ///< contact frame heights, world z [m]; num_contacts of them, always read
  int num_contacts{0};
};

struct PoseGuardLimits {
  double cos_tilt{1.0};  ///< cos(base_tilt_rad)
  bool check_stance{false};
  double max_spread_m{0.0};  ///< every contact this close to the lowest [m]
  double max_speed{0.0};     ///< the base slower than this [m/s]
};

[[nodiscard]] inline std::uint32_t EvaluatePoseGuard(const PoseGuardInput& in,
                                                     const PoseGuardLimits& lim) noexcept {
  std::uint32_t mask = 0;
  if (!in.base_valid) {
    mask |= kPoseGuardBaseInvalid;
  }
  bool finite = true;
  for (const double x : in.q_xyzw) {
    finite = finite && std::isfinite(x);
  }
  for (const double x : in.v_base) {
    finite = finite && std::isfinite(x);
  }
  bool z_finite = true;
  for (int c = 0; c < in.num_contacts; ++c) {
    z_finite = z_finite && std::isfinite(in.contact_z[c]);
  }
  finite = finite && z_finite;
  if (!finite) {
    mask |= kPoseGuardNonFinite;
  }
  const double x = in.q_xyzw[0];
  const double y = in.q_xyzw[1];
  const double z = in.q_xyzw[2];
  const double w = in.q_xyzw[3];
  const double n2 = x * x + y * y + z * z + w * w;
  if (!(std::abs(n2 - 1.0) < kPoseGuardQuaternionTolerance)) {
    mask |= kPoseGuardQuaternion;
  }
  // R22 · ‖q‖² ≥ cos θ · ‖q‖² (header): the direction alone decides.
  if (!(n2 - 2.0 * (x * x + y * y) >= lim.cos_tilt * n2)) {
    mask |= kPoseGuardTilt;
  }
  if (lim.check_stance) {
    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    for (int c = 0; c < in.num_contacts; ++c) {
      lo = std::min(lo, in.contact_z[c]);
      hi = std::max(hi, in.contact_z[c]);
    }
    // std::min / max skip a NaN, so the spread of NaN heights would read as a
    // stand: the finiteness is part of the check, not only of its own bit.
    if (!(in.num_contacts > 0 && z_finite && hi - lo <= lim.max_spread_m)) {
      mask |= kPoseGuardSpread;
    }
    const double speed = std::hypot(in.v_base[0], in.v_base[1], in.v_base[2]);
    if (!(speed < lim.max_speed)) {
      mask |= kPoseGuardSpeed;
    }
  }
  return mask;
}

}  // namespace integrated_bringup::locomotion

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_POSE_GUARD_HPP_
