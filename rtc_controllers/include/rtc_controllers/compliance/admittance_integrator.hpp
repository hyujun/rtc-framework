// ── Compliant-frame admittance integrator (spec §7.2, §7.5) ─────────────────
// Simulates the virtual compliant frame X_c of a task-space admittance law
//
//     Λ_d ẍ̃_c + K_d ẋ̃_c + K_p x̃_c = S f^ext_LWA
//
// where x̃_c is the deviation of X_c from the desired frame X_d. This class owns
// ONLY that ODE and its §7.5 workspace guards — it knows nothing about robots,
// Jacobians or wrench sensors, which is what makes the energy / drift
// properties below testable without a URDF.
//
// ── Deviation representation (why a rotation MATRIX, not a 3-vector) ─────────
// x̃_c = e(X_d, X_c) per §7.2 — note this is the OPPOSITE direction to §1.3's
// pose error (which runs current → desired), and slice 2 already shipped one
// sign defect at exactly this seam (compliance-conventions.md §3.3). Written out
// in the repo's SplitWorld convention:
//
//     x̃_lin = p_c − p_d,        R̃ = R_c R_dᵀ,       x̃_rot = log3(R̃)
//
// The rotation half is stored as R̃ and retracted every step (§7.2 MUST):
//
//     R̃ ← exp3(ω_c·dt) · R̃
//
// LEFT multiplication, not the spec snippet's `R_c = R_c * exp3(ω dt)`: the
// snippet assumes ω in the BODY frame, while everything in this package —
// Jacobian, wrench, K_p axes — is LOCAL_WORLD_ALIGNED, so ω_c here is a WORLD
// angular velocity and Ṙ_c = [ω]× R_c ⇒ Ṙ̃ = [ω]× R̃. Accumulating x̃_rot in the
// tangent space instead (`x_rot += ω·dt`) is what §7.2 forbids: the sum of
// non-coaxial rotation vectors is not the rotation vector of the composition, so
// the compliant frame drifts away from the physical rotation it represents.
//
// ── Integration (semi-implicit / symplectic Euler, §7.2 MUST) ───────────────
//     ẍ̃ = Λ_d⁻¹(f − K_d ẋ̃ − K_p x̃);   ẋ̃ += ẍ̃·dt;   x̃ += ẋ̃·dt   ← velocity FIRST
// Explicit Euler (position first) pumps energy into an oscillator and is the
// common cause of "admittance that diverges for no reason" on real hardware.
// AdmittanceEnergyDoesNotGrow / ExplicitEulerReferenceGainsEnergy pin both
// halves of that claim.
//
// ── §7.5 workspace guards ───────────────────────────────────────────────────
// K_p = 0 (hand-guiding) leaves x̃ unbounded, so two INDEPENDENT bounds apply:
//   • displacement — at ‖x̃‖ ≥ max_displacement the OUTWARD radial velocity is
//     removed and a saturating spring is added to the force. Not a clamp of x̃:
//     clamping steps the velocity discontinuously (§7.5 MUST). The projection is
//     what makes the bound an invariant (‖x̃‖ cannot grow past it), the spring is
//     what makes the return smooth and lets the wall be felt. The radial speed
//     is solved for so the step lands ON the bound, which is what stops a
//     sliding frame from creeping past it by (v·dt)²/2d per step.
//   • velocity — a displacement bound alone does not stop a large impulse from
//     producing a huge instantaneous ẋ̃. Limited by NORM (direction-preserving),
//     not per component: a component-wise clamp silently rotates the motion.
#pragma once

#include <rtc_math/se3/so3.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace rtc::compliance {

/// Λ_d / K_d / K_p diagonals plus the §7.5 bounds. Trivially copyable so a
/// controller can carry it inside a SeqLock'd Gains POD.
struct AdmittanceParams {
  /// Λ_d diagonal [kg ×3, kg·m² ×3]. Λ_d is inverted every step, so the §7.4
  /// lower bounds below are applied on the fly — a zero here is a divide, not a
  /// tuning mistake (NUM-2).
  std::array<double, 6> inertia{{2.0, 2.0, 2.0, 0.05, 0.05, 0.05}};
  std::array<double, 6> damping{{40.0, 40.0, 40.0, 1.0, 1.0, 1.0}};  ///< K_d
  std::array<double, 6> stiffness{
      {200.0, 200.0, 200.0, 5.0, 5.0, 5.0}};  ///< K_p (0 = hand-guiding)

  /// §7.4 MUST — contact stability degrades as Λ_d shrinks, so a floor is
  /// enforced rather than trusted to the operator. Defaults are the spec's.
  double min_inertia_lin{2.0};   ///< [kg]
  double min_inertia_ang{0.05};  ///< [kg·m²]

  // §7.5 bounds. ≤ 0 disables the corresponding guard.
  double max_displacement_lin{0.15};  ///< [m]
  double max_displacement_ang{0.5};   ///< [rad]
  double max_velocity_lin{0.25};      ///< [m/s]
  double max_velocity_ang{0.8};       ///< [rad/s]

  /// Saturating-spring stiffness applied BEYOND the displacement bound. Sized so
  /// that with the minimum admissible Λ_d the barrier frequency is √(k/Λ) ≈ 32
  /// rad/s — well inside semi-implicit Euler's ω·dt < 2 stability bound at every
  /// supported control_rate. A stiffer barrier buys nothing: the outward-velocity
  /// projection, not the spring, is what enforces the bound.
  double barrier_stiffness_lin{2000.0};  ///< [N/m]
  double barrier_stiffness_ang{50.0};    ///< [N·m/rad]
};

/// One Step() report. No logging from here — this is RT (RT-3).
struct AdmittanceStatus {
  bool displacement_limited{false};  ///< the §7.5 barrier engaged this step
  bool velocity_limited{false};      ///< ‖ẋ̃‖ was scaled back this step
  bool finite{true};                 ///< false ⇒ the step was REFUSED, state untouched
};

/// State of the virtual compliant frame, relative to X_d. RT-safe throughout:
/// noexcept, fixed-size Eigen only, no heap, no throw.
class AdmittanceIntegrator {
 public:
  /// Collapse the compliant frame onto X_d at rest. Called on every
  /// (re)activation — an activation must not inherit a deviation accrued before
  /// it (§10.7: the arm must not jump when control resumes).
  void Reset() noexcept {
    x_lin_.setZero();
    R_.setIdentity();
    v_.setZero();
  }

  /// Advance one control tick under the external wrench `f_ext` (LWA,
  /// [force;torque], environment-on-robot sign). `dt` ≤ 0 or a non-finite input
  /// leaves the state EXACTLY as it was and reports it — a degenerate tick must
  /// not be able to poison the compliant frame, which nothing downstream resets.
  AdmittanceStatus Step(const AdmittanceParams& params, const Eigen::Matrix<double, 6, 1>& f_ext,
                        double dt) noexcept {
    AdmittanceStatus st;
    st.finite = f_ext.allFinite() && x_lin_.allFinite() && v_.allFinite() && R_.allFinite();
    if (!st.finite || !(dt > 0.0))
      return st;

    // x̃ = [p̃ ; log3(R̃)]. The rotation half is derived from R̃ every step rather
    // than carried, which is precisely what keeps it a true rotation vector.
    const Eigen::Vector3d phi = rtc::math::se3::log3(R_);

    // ── §7.5 saturating spring, added to the FORCE (not clamping the state) ──
    Eigen::Matrix<double, 6, 1> f = f_ext;
    if (AddBarrier(f.head<3>(), x_lin_, params.max_displacement_lin, params.barrier_stiffness_lin))
      st.displacement_limited = true;
    if (AddBarrier(f.tail<3>(), phi, params.max_displacement_ang, params.barrier_stiffness_ang))
      st.displacement_limited = true;

    // ── Semi-implicit Euler: acceleration → VELOCITY → position ──────────────
    for (int i = 0; i < 6; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      const double lambda =
          std::max(params.inertia[ui], (i < 3) ? params.min_inertia_lin : params.min_inertia_ang);
      const double x_i = (i < 3) ? x_lin_(i) : phi(i - 3);
      // λ ≥ min_inertia; the controller floors those at configure, and a
      // pathological zero here would be a divide (NUM-2) — hence the guard.
      const double acc =
          (lambda > 0.0) ? (f(i) - params.damping[ui] * v_(i) - params.stiffness[ui] * x_i) / lambda
                         : 0.0;
      v_(i) += acc * dt;
    }

    // ── Velocity bound (norm-preserving) ────────────────────────────────────
    if (LimitNorm(v_.head<3>(), params.max_velocity_lin))
      st.velocity_limited = true;
    if (LimitNorm(v_.tail<3>(), params.max_velocity_ang))
      st.velocity_limited = true;

    // ── Displacement bound: strip the OUTWARD radial velocity ───────────────
    // Tested on the PREDICTED position x̃ + ẋ̃·dt, not the current one. Reacting
    // only once already outside lets the crossing step overshoot by up to
    // max_velocity·dt (2.5 mm at the default 0.25 m/s and a 100 Hz tick) before
    // anything engages, and §7.5 states the bound without that asterisk.
    // Predicting keeps the guard velocity-only — no positional clamp, which is
    // what §7.5 rules out.
    //
    // With θ̇ = ωᵀn (n = the rotation axis of R̃) the angular case is the same
    // radial projection as the linear one, not an approximation.
    if (ProjectOutward(v_.head<3>(), x_lin_, params.max_displacement_lin, dt,
                       params.max_velocity_lin))
      st.displacement_limited = true;
    if (ProjectOutward(v_.tail<3>(), phi, params.max_displacement_ang, dt, params.max_velocity_ang))
      st.displacement_limited = true;

    // ── Retract (§7.2 MUST) ─────────────────────────────────────────────────
    x_lin_ += v_.head<3>() * dt;
    R_ = rtc::math::se3::exp3(Eigen::Vector3d(v_.tail<3>() * dt)) * R_;
    return st;
  }

  /// p̃ = p_c − p_d [m].
  [[nodiscard]] const Eigen::Vector3d& translation() const noexcept { return x_lin_; }

  /// R̃ = R_c R_dᵀ.
  [[nodiscard]] const Eigen::Matrix3d& rotation() const noexcept { return R_; }

  /// ẋ̃_c = ν_c [linear; angular], LWA. X_d is static within a tick, so this is
  /// also the twist of the compliant frame itself — what the IK feeds forward.
  [[nodiscard]] const Eigen::Matrix<double, 6, 1>& velocity() const noexcept { return v_; }

  /// x̃_c = [p̃ ; log3(R̃)] — the §7.2 deviation, for diagnostics and the §7.5
  /// bound reporting. Recomputed rather than cached: one log3 per tick is cheap
  /// and a cache is one more thing that can disagree with R̃.
  [[nodiscard]] Eigen::Matrix<double, 6, 1> deviation() const noexcept {
    Eigen::Matrix<double, 6, 1> x;
    x.head<3>() = x_lin_;
    x.tail<3>() = rtc::math::se3::log3(R_);
    return x;
  }

  /// Mechanical energy of the virtual system, E = ½ẋ̃ᵀΛ_dẋ̃ + ½x̃ᵀK_px̃ (§11.8).
  /// Uses the SAME inertia floor Step() applies, so the two cannot disagree
  /// about which Λ_d is in force.
  [[nodiscard]] double Energy(const AdmittanceParams& params) const noexcept {
    const Eigen::Matrix<double, 6, 1> x = deviation();
    double e = 0.0;
    for (int i = 0; i < 6; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      const double lambda =
          std::max(params.inertia[ui], (i < 3) ? params.min_inertia_lin : params.min_inertia_ang);
      e += 0.5 * lambda * v_(i) * v_(i) + 0.5 * params.stiffness[ui] * x(i) * x(i);
    }
    return e;
  }

 private:
  // f += −k·(‖x‖ − d_max)·x̂ once past the bound. Returns true when it engaged.
  [[nodiscard]] static bool AddBarrier(Eigen::Ref<Eigen::Vector3d> f, const Eigen::Vector3d& x,
                                       double d_max, double k) noexcept {
    if (!(d_max > 0.0) || !(k > 0.0))
      return false;
    const double n = x.norm();
    if (n <= d_max)
      return false;
    f -= (k * (n - d_max) / n) * x;  // n > d_max > 0 (NUM-2)
    return true;
  }

  // Scale u back onto the ‖u‖ ≤ v_max ball, preserving direction.
  [[nodiscard]] static bool LimitNorm(Eigen::Ref<Eigen::Vector3d> u, double v_max) noexcept {
    if (!(v_max > 0.0))
      return false;
    const double n = u.norm();
    if (n <= v_max)
      return false;
    u *= v_max / n;  // n > v_max > 0 (NUM-2)
    return true;
  }

  // Retune the RADIAL component of u — never the tangential one — so the step it
  // is about to take lands on the bound instead of outside it.
  //
  // Simply zeroing the outward radial part is not enough. A purely tangential
  // step off a sphere of radius d still grows the norm by (v·dt)²/2d, and that
  // creep accumulates: 4000 steps of sliding at the default limits walks 1e-5 m
  // past a bound §7.5 states without an asterisk. Solving for the radial speed
  // that puts ‖x + ẋ·dt‖ exactly at d_max removes the creep at its source,
  // costs one sqrt, and leaves the tangential component bit-for-bit intact —
  // which is what keeps this a velocity correction and not the positional clamp
  // §7.5 rules out.
  //
  // When x is ALREADY outside (a rounding hair past the bound, or d_max lowered
  // at runtime) the same solve produces an INWARD radial speed, so the guard
  // recovers by itself. It is capped at `v_return_max` — otherwise shrinking
  // d_max mid-run would demand a −δ/dt impulse, i.e. exactly the velocity
  // discontinuity §7.5 exists to avoid. Handling this in the one formula rather
  // than as a separate "already outside" branch matters: that branch could only
  // strip the outward component, which re-opens the tangential creep for good
  // the first time round-off puts the state a nanometre outside.
  [[nodiscard]] static bool ProjectOutward(Eigen::Ref<Eigen::Vector3d> u, const Eigen::Vector3d& x,
                                           double d_max, double dt, double v_return_max) noexcept {
    if (!(d_max > 0.0) || !(dt > 0.0))
      return false;
    if ((x + u * dt).norm() <= d_max)
      return false;
    const double n = x.norm();
    // At x = 0 there is no radial direction yet; the whole step is "outward", so
    // the velocity's own direction is the one to strip. Reachable only when a
    // single step would cross the entire bound from rest, i.e. a huge impulse
    // with the velocity limit disabled.
    const Eigen::Vector3d dir = (n > 0.0) ? Eigen::Vector3d(x / n) : u.normalized();
    const double radial = u.dot(dir);
    const Eigen::Vector3d tangential = u - radial * dir;
    const double tangential_step_sq = tangential.squaredNorm() * dt * dt;

    // Exact landing: (n + r·dt)² + ‖v_t·dt‖² = d_max². When the tangential step
    // alone already overshoots there is no such r; stopping radial growth is
    // then the most the radial component can contribute.
    double r_target = (tangential_step_sq < d_max * d_max)
                          ? (std::sqrt(d_max * d_max - tangential_step_sq) - n) / dt
                          : 0.0;
    if (v_return_max > 0.0)
      r_target = std::max(r_target, -v_return_max);
    if (r_target >= radial)
      return false;  // the step already lands inside, or is heading back in
    u = tangential + r_target * dir;
    return true;
  }

  Eigen::Vector3d x_lin_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix<double, 6, 1> v_{Eigen::Matrix<double, 6, 1>::Zero()};
};

}  // namespace rtc::compliance
