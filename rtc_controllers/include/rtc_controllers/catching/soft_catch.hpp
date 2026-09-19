// ── Soft-catch translational reference (dynamic_catching S1.4, L4) ───────────
// A second-order DS in the intercept frame: the reference x tracks
// p_c + γ(t)·(o − p_c), where o is the ball (or, in DECEL, a virtual
// decelerating target) and γ ramps 0 → γ_f by a quintic profile ending at t_c.
// With e = (x − p_c) − γ·(o − p_c):
//
//   u_des = γ·a_o + 2γ̇·v_o + γ̈·(o − p_c)   (feedforward)
//           − ω²·e − 2ζω·ė                    (error dynamics e'' = −ω²e − 2ζωė)
//
// then |u| ≤ a_max and |ẋ| ≤ v_max by radial saturation, semi-implicit Euler.
//
// Ported from docs/dynamic_catching/soft_catch_reference.hpp (v0.4) with:
//  • γ derate (derateGamma / derateJump / DerateResult) NOT ported — D-8 takes
//    it out of v1: after COMMITTED no plan change is allowed;
//  • the axis-alignment functions NOT ported here — they move to rtc_math se3
//    in S2.1 (L4.4);
//  • a NaN guard (L4 §5.1, G4-I): the reference let one non-finite target
//    poison x_/ẋ_ for good, and `u.norm() > a_max` is false for NaN so the
//    saturation flag never rose. Here any non-finite target, t or dt — or a
//    command that overflows to non-finite — leaves the state untouched and
//    returns valid = false with saturated = true; the next finite call
//    continues from the preserved state;
//  • dt ≤ 0 is invalid (L4 §5.1). The reference used step(…, dt = 0) to read
//    e / ė without integrating; that is Evaluate() now.
//
// Time axis (plan §3, L4 §5.2): `t` and the profile's t0 / t1 are relative
// seconds from ONE origin on the lead axis (now + T_arm). ProfileSeconds() and
// MakeGammaProfile() form them from the absolute instants at the core boundary.
//
// RT-safe: fixed size, no allocation, noexcept.
#pragma once

#include "rtc_controllers/catching/time_types.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace rtc::catching {

/// Target the reference follows (ball sample or virtual decelerating target),
/// world frame.
struct TargetState {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};  // [m]
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};  // [m/s]
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};  // [m/s²]
};

/// γ(t): quintic g0 → gf on [t0, t1], first and second derivatives zero at both
/// ends (so clamping outside the ramp is consistent). Times are relative
/// seconds on the lead axis.
struct GammaProfile {
  double g0{0.0};
  double gf{0.0};
  double t0{0.0};
  double t1{1.0};

  /// A degenerate ramp (t1 ≤ t0) is a step at t0 with zero derivatives — the
  /// reference floored the duration at 1e-6 s instead, which turns the same
  /// input into a 1e12-scale γ̈ spike.
  void Eval(double t, double& g, double& gd, double& gdd) const noexcept {
    const double d = gf - g0;
    if (!(t1 > t0)) {
      g = (t >= t0) ? gf : g0;
      gd = 0.0;
      gdd = 0.0;
      return;
    }
    const double T = t1 - t0;
    const double s = std::clamp((t - t0) / T, 0.0, 1.0);
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double s4 = s3 * s;
    const double s5 = s4 * s;
    g = g0 + d * (10.0 * s3 - 15.0 * s4 + 6.0 * s5);
    gd = d * (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / T;
    gdd = d * (60.0 * s - 180.0 * s2 + 120.0 * s3) / (T * T);
  }

  [[nodiscard]] bool IsFinite() const noexcept {
    return std::isfinite(g0) && std::isfinite(gf) && std::isfinite(t0) && std::isfinite(t1);
  }
};

/// Relative seconds of lead instant `now_lead` from `origin` — the `t` passed to
/// Step()/Evaluate(). The origin is any instant fixed for the whole plan (e.g.
/// the plan's γ-ramp start); only differences on one axis are formed.
[[nodiscard]] constexpr double ProfileSeconds(NowLead now_lead, BallTime origin) noexcept {
  return -LeadSecondsUntil(now_lead, origin);
}

/// γ profile from absolute BallTime ramp instants, relative to `origin` (same
/// origin as ProfileSeconds()).
[[nodiscard]] constexpr GammaProfile MakeGammaProfile(double g0, double gf, BallTime t0,
                                                      BallTime t1, BallTime origin) noexcept {
  return GammaProfile{g0, gf, SecondsBetween(origin, t0), SecondsBetween(origin, t1)};
}

// Time bases of the output (L4 §5.2): x, xd are the reference for t + dt (the
// next tick's command); xdd is the acceleration actually realised over
// [t, t + dt] (velocity saturation included); u_des is the pre-saturation DS
// demand (diagnostic, planner rollout); e, ed are at t (diagnostic).
struct TranslationOutput {
  Eigen::Vector3d x{Eigen::Vector3d::Zero()};
  Eigen::Vector3d xd{Eigen::Vector3d::Zero()};
  Eigen::Vector3d xdd{Eigen::Vector3d::Zero()};
  Eigen::Vector3d u_des{Eigen::Vector3d::Zero()};
  Eigen::Vector3d e{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ed{Eigen::Vector3d::Zero()};
  double gamma{0.0};
  double gamma_d{0.0};
  double gamma_dd{0.0};
  bool saturated{false};  // |u| or |ẋ| limited — or the step was rejected
  bool valid{false};      // false: inputs or command non-finite, state untouched
};

class SoftCatchTranslation {
 public:
  struct Params {
    double omega{10.0};  // [rad/s]  ω·dt < 2√2 − 2 ≈ 0.828 for discrete stability (L4 §4.7)
    double zeta{1.0};    //          the planner's closed-form terminal error assumes ζ = 1
    double a_max{15.0};  // [m/s²]   L7 supervisor.decel.a_dec must not exceed it (L7 §4.3)
    double v_max{2.0};   // [m/s]    the γ window uses η_v·v_max (D-9)
  };

  explicit SoftCatchTranslation(const Params& p) noexcept : prm_(p) {}

  /// Activation / re-arm: resets the reference state AND the intercept and γ
  /// profile. With γ ≡ 0 the attractor is p_c, so p_c = x makes this a hold at
  /// x (the reference kept p_c / γ and returned to the previous trial's catch
  /// point before v0.3). Rejects (returns false, state untouched) non-finite x/ẋ.
  bool Reset(const Eigen::Vector3d& x, const Eigen::Vector3d& xd) noexcept {
    if (!x.allFinite() || !xd.allFinite())
      return false;
    x_ = x;
    xd_ = xd;
    p_c_ = x;
    gp_ = GammaProfile{};
    return true;
  }

  /// Set the intercept and γ profile. NOTE: with γ ≡ 0 the target o has no
  /// influence at all — u = −ω²(x − p_c) − 2ζωẋ — so a stationary goal (home,
  /// wait pose) must be given as p_c (L4 §5.3). Rejects non-finite input.
  bool SetIntercept(const Eigen::Vector3d& p_c, const GammaProfile& gp) noexcept {
    if (!p_c.allFinite() || !gp.IsFinite())
      return false;
    p_c_ = p_c;
    gp_ = gp;
    return true;
  }

  [[nodiscard]] const Eigen::Vector3d& Intercept() const noexcept { return p_c_; }

  [[nodiscard]] const GammaProfile& Gamma() const noexcept { return gp_; }

  [[nodiscard]] const Eigen::Vector3d& Position() const noexcept { return x_; }

  [[nodiscard]] const Eigen::Vector3d& Velocity() const noexcept { return xd_; }

  /// e, ė, γ and u_des at t without integrating (state untouched). x/xd report
  /// the current state, xdd the saturated demand.
  [[nodiscard]] TranslationOutput Evaluate(const TargetState& o, double t) const noexcept {
    TranslationOutput out = Rejected();
    if (!Finite(o) || !std::isfinite(t))
      return out;
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    if (!Demand(o, t, out, u))
      return out;
    out.xdd = u;
    out.valid = true;
    return out;
  }

  /// One tick. t: relative seconds on the lead axis (same origin as the γ
  /// profile); dt: control period [s], > 0.
  [[nodiscard]] TranslationOutput Step(const TargetState& o, double t, double dt) noexcept {
    TranslationOutput out = Rejected();
    if (!Finite(o) || !std::isfinite(t) || !std::isfinite(dt) || !(dt > 0.0))
      return out;
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    if (!Demand(o, t, out, u))
      return out;

    Eigen::Vector3d xd_next = xd_ + u * dt;  // semi-implicit Euler
    const double vn = xd_next.norm();
    if (!std::isfinite(vn))
      return out;  // state untouched
    if (!(vn <= prm_.v_max)) {
      xd_next *= prm_.v_max / vn;
      out.saturated = true;
    }
    const Eigen::Vector3d x_next = x_ + xd_next * dt;
    if (!x_next.allFinite())
      return out;

    // Under velocity saturation u is no longer the realised acceleration.
    out.xdd = (xd_next - xd_) / dt;
    xd_ = xd_next;
    x_ = x_next;
    out.x = x_;
    out.xd = xd_;
    out.valid = true;
    return out;
  }

 private:
  [[nodiscard]] static bool Finite(const TargetState& o) noexcept {
    return o.p.allFinite() && o.v.allFinite() && o.a.allFinite();
  }

  /// Rejected-step output: current state, flagged invalid and saturated.
  [[nodiscard]] TranslationOutput Rejected() const noexcept {
    TranslationOutput out{};
    out.x = x_;
    out.xd = xd_;
    out.saturated = true;
    out.valid = false;
    return out;
  }

  /// Fills e, ė, γ terms and u_des of `out`, and the acceleration-saturated
  /// command u. Returns false (out untouched beyond diagnostics) if the demand
  /// is not finite. The comparison is negated so a NaN norm counts as saturated.
  bool Demand(const TargetState& o, double t, TranslationOutput& out,
              Eigen::Vector3d& u) const noexcept {
    const Eigen::Vector3d xo = o.p - p_c_;  // intercept-frame target
    double g = 0.0;
    double gd = 0.0;
    double gdd = 0.0;
    gp_.Eval(t, g, gd, gdd);
    const Eigen::Vector3d e = (x_ - p_c_) - g * xo;
    const Eigen::Vector3d ed = xd_ - (g * o.v + gd * xo);
    const double w = prm_.omega;
    const Eigen::Vector3d u_des =
        g * o.a + 2.0 * gd * o.v + gdd * xo - w * w * e - 2.0 * prm_.zeta * w * ed;
    out.e = e;
    out.ed = ed;
    out.u_des = u_des;
    out.gamma = g;
    out.gamma_d = gd;
    out.gamma_dd = gdd;
    const double un = u_des.norm();
    if (!std::isfinite(un))
      return false;  // saturated stays true from Rejected()
    out.saturated = false;
    u = u_des;
    if (!(un <= prm_.a_max)) {
      u *= prm_.a_max / un;
      out.saturated = true;
    }
    return true;
  }

  Params prm_;
  Eigen::Vector3d x_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d xd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d p_c_{Eigen::Vector3d::Zero()};
  GammaProfile gp_{};
};

/// Closed-form error of the critically damped (ζ = 1) error dynamics after t —
/// the planner's terminal-error prediction (L3, L4 §4.4). Valid only for ζ = 1,
/// which is why the validator rejects ζ ≠ 1.
inline void CriticallyDampedError(const Eigen::Vector3d& e0, const Eigen::Vector3d& ed0,
                                  double omega, double t, Eigen::Vector3d& e,
                                  Eigen::Vector3d& ed) noexcept {
  const Eigen::Vector3d c = ed0 + omega * e0;
  const double ex = std::exp(-omega * t);
  e = (e0 + c * t) * ex;
  ed = (ed0 - omega * t * c) * ex;
}

}  // namespace rtc::catching
