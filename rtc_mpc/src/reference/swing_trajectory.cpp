#include "rtc_mpc/reference/swing_trajectory.hpp"

#include "reference/internal/yaml_fields.hpp"

#include <algorithm>
#include <cmath>

namespace rtc::mpc {
namespace {

using internal::FieldRead;

[[nodiscard]] bool AllFinite(const SwingConfig& c) noexcept {
  return std::isfinite(c.height) && std::isfinite(c.lift_off_velocity) &&
         std::isfinite(c.touch_down_velocity) && std::isfinite(c.touch_down_offset) &&
         std::isfinite(c.time_scale) && std::isfinite(c.impact_proximity.mid) &&
         std::isfinite(c.impact_proximity.lift_off_velocity) &&
         std::isfinite(c.impact_proximity.touch_down_velocity);
}

// The segment through (t0, p0, v0) and (t1, p1, v1); the caller has checked t1 > t0.
[[nodiscard]] CubicSegment Segment(double t0, double p0, double v0, double t1, double p1,
                                   double v1) noexcept {
  CubicSegment s;
  s.t0 = t0;
  s.dt = t1 - t0;
  s.inv_dt = 1.0 / s.dt;
  s.inv_dt2 = s.inv_dt * s.inv_dt;
  const double dp = p1 - p0;
  const double dv = v1 - v0;
  s.c0 = p0;
  s.c1 = v0 * s.dt;
  s.c2 = -(3.0 * v0 + dv) * s.dt + 3.0 * dp;
  s.c3 = (2.0 * v0 + dv) * s.dt - 2.0 * dp;
  return s;
}

/// The end values of one two-segment curve: (p0, v0) → p_mid → (p1, v1).
struct Ends {
  double p0, v0, p_mid, p1, v1;
};

}  // namespace

// ── YAML ──────────────────────────────────────────────────────────────────────

SwingError ValidateSwingConfig(const SwingConfig& c) noexcept {
  if (!AllFinite(c)) {
    return SwingError::kNonFinite;
  }
  if (c.height <= 0.0) {
    return SwingError::kInvalidHeight;
  }
  if (c.time_scale <= 0.0) {
    return SwingError::kInvalidTimeScale;
  }
  if (c.impact_proximity.mid < 0.0 || c.impact_proximity.mid > 1.0) {
    return SwingError::kInvalidImpactMid;
  }
  return SwingError::kNoError;
}

SwingError LoadSwingConfig(const YAML::Node& node, SwingConfig& out) noexcept {
  try {
    if (!node.IsDefined() || !node.IsMap()) {
      return SwingError::kNotAMap;
    }
    if (!internal::HasOnlyKeys(node, {"height", "lift_off_velocity", "touch_down_velocity",
                                      "touch_down_offset", "time_scale", "impact_proximity"})) {
      return SwingError::kUnknownKey;
    }
    const YAML::Node impact = internal::Child(node, "impact_proximity");
    if (!internal::Present(impact)) {
      return SwingError::kMissingKey;
    }
    if (!impact.IsMap()) {
      return SwingError::kWrongType;
    }
    if (!internal::HasOnlyKeys(impact, {"mid", "lift_off_velocity", "touch_down_velocity"})) {
      return SwingError::kUnknownKey;
    }
    SwingConfig c;
    const FieldRead reads[] = {
        internal::ReadScalar(internal::Child(node, "height"), c.height),
        internal::ReadScalar(internal::Child(node, "lift_off_velocity"), c.lift_off_velocity),
        internal::ReadScalar(internal::Child(node, "touch_down_velocity"), c.touch_down_velocity),
        internal::ReadScalar(internal::Child(node, "touch_down_offset"), c.touch_down_offset),
        internal::ReadScalar(internal::Child(node, "time_scale"), c.time_scale),
        internal::ReadScalar(internal::Child(impact, "mid"), c.impact_proximity.mid),
        internal::ReadScalar(internal::Child(impact, "lift_off_velocity"),
                             c.impact_proximity.lift_off_velocity),
        internal::ReadScalar(internal::Child(impact, "touch_down_velocity"),
                             c.impact_proximity.touch_down_velocity),
    };
    for (const FieldRead r : reads) {
      if (r != FieldRead::kOk) {
        return internal::ToError<SwingError>(r);
      }
    }
    const SwingError e = ValidateSwingConfig(c);
    if (e == SwingError::kNoError) {
      out = c;
    }
    return e;
  } catch (...) {
    return SwingError::kWrongType;
  }
}

// ── Segments ──────────────────────────────────────────────────────────────────

void CubicSegment::Evaluate(double t, double& value, double& rate,
                            double& acceleration) const noexcept {
  // `!(t > t0)` also catches NaN → the segment start.
  const double tc = !(t > t0) ? t0 : (t > t0 + dt ? t0 + dt : t);
  const double tau = (tc - t0) * inv_dt;
  value = ((c3 * tau + c2) * tau + c1) * tau + c0;
  rate = ((3.0 * c3 * tau + 2.0 * c2) * tau + c1) * inv_dt;
  acceleration = (6.0 * c3 * tau + 2.0 * c2) * inv_dt2;
}

SwingError TwoSegmentCubic::Build(double t0, double p0, double v0, double p_mid, double t1,
                                  double p1, double v1) noexcept {
  if (!std::isfinite(t0) || !std::isfinite(t1) || !std::isfinite(p0) || !std::isfinite(v0) ||
      !std::isfinite(p_mid) || !std::isfinite(p1) || !std::isfinite(v1)) {
    return SwingError::kNonFinite;
  }
  const double t_mid = 0.5 * (t0 + t1);
  // The absolute bound, and the midpoint must stay strictly inside: a stretch
  // of one ulp at a large t rounds t_mid onto an end (a zero-length segment).
  if (!(t1 - t0 > kMinSwingDuration) || !(t0 < t_mid && t_mid < t1)) {
    return SwingError::kDegenerateInterval;
  }
  const CubicSegment left = Segment(t0, p0, v0, t_mid, p_mid, 0.0);
  const CubicSegment right = Segment(t_mid, p_mid, 0.0, t1, p1, v1);
  // Finite inputs can still overflow a coefficient (|Δp|·3 beyond DBL_MAX).
  if (!std::isfinite(left.c2) || !std::isfinite(left.c3) || !std::isfinite(right.c2) ||
      !std::isfinite(right.c3)) {
    return SwingError::kCoefficientOverflow;
  }
  left_ = left;
  right_ = right;
  t_mid_ = t_mid;
  return SwingError::kNoError;
}

void TwoSegmentCubic::Evaluate(double t, double& value, double& rate,
                               double& acceleration) const noexcept {
  if (!(t >= t_mid_)) {  // NaN → the left segment, which reads it as its start
    left_.Evaluate(t, value, rate, acceleration);
  } else {
    right_.Evaluate(t, value, rate, acceleration);
  }
}

// ── Swing ─────────────────────────────────────────────────────────────────────

SwingTrajectory SwingTrajectory::Stance(double terrain_z) noexcept {
  SwingTrajectory s;
  s.terrain_z_ = terrain_z;
  return s;
}

SwingSample SwingTrajectory::Evaluate(double t) const noexcept {
  SwingSample out;
  out.z = terrain_z_;
  // Half-open [t_lo, t_td): touch-down reads as stance (an open end holds the
  // curve). A non-finite `t` is stance too.
  if (!swinging_ || !std::isfinite(t) || t < t_lo_ || (!open_end_ && t >= t_td_)) {
    return out;
  }
  z_.Evaluate(t, out.z, out.z_dot, out.z_ddot);
  double unused = 0.0;
  impact_.Evaluate(t, out.impact, out.impact_dot, unused);
  if (out.impact < 0.0 || out.impact > 1.0) {
    out.impact = std::clamp(out.impact, 0.0, 1.0);
    out.impact_dot = 0.0;
  }
  return out;
}

SwingError BuildSwing(const SwingInterval& iv, const SwingConfig& cfg, double terrain_z,
                      SwingTrajectory& out) noexcept {
  const SwingError e = ValidateSwingConfig(cfg);
  if (e != SwingError::kNoError) {
    return e;
  }
  const double z_lo = terrain_z;
  const double z_td = terrain_z + cfg.touch_down_offset;
  const double h = cfg.height;
  const double s = std::min(1.0, (iv.touch_down - iv.lift_off) / cfg.time_scale);
  const ImpactProximityConfig& ip = cfg.impact_proximity;

  // The four cases (header): a closed stretch is scaled by s, an open one is not.
  Ends z{};
  Ends w{};
  if (!iv.open_start && !iv.open_end) {
    z = {z_lo, s * cfg.lift_off_velocity, std::min(z_lo, z_td) + s * h, z_td,
         s * cfg.touch_down_velocity};
    w = {1.0, s * ip.lift_off_velocity, ip.mid, 1.0, s * ip.touch_down_velocity};
  } else if (!iv.open_start) {
    z = {z_lo, cfg.lift_off_velocity, z_lo + h, z_lo + h, 0.0};
    w = {1.0, ip.lift_off_velocity, ip.mid, ip.mid, 0.0};
  } else if (!iv.open_end) {
    z = {z_td + h, 0.0, z_td + h, z_td, cfg.touch_down_velocity};
    w = {ip.mid, 0.0, ip.mid, 1.0, ip.touch_down_velocity};
  } else {
    z = {z_td + h, 0.0, z_td + h, z_td + h, 0.0};
    w = {ip.mid, 0.0, ip.mid, ip.mid, 0.0};
  }

  // Build validates the stretch (finite, > kMinSwingDuration, midpoint inside)
  // and the coefficients; a non-finite terrain reaches it through z.p0.
  SwingTrajectory r;
  const SwingError ez = r.z_.Build(iv.lift_off, z.p0, z.v0, z.p_mid, iv.touch_down, z.p1, z.v1);
  if (ez != SwingError::kNoError) {
    return ez;
  }
  const SwingError ei =
      r.impact_.Build(iv.lift_off, w.p0, w.v0, w.p_mid, iv.touch_down, w.p1, w.v1);
  if (ei != SwingError::kNoError) {
    return ei;
  }
  r.terrain_z_ = terrain_z;
  r.t_lo_ = iv.lift_off;
  r.t_td_ = iv.touch_down;
  r.scaling_ = s;
  r.swinging_ = true;
  r.open_start_ = iv.open_start;
  r.open_end_ = iv.open_end;
  out = r;
  return SwingError::kNoError;
}

}  // namespace rtc::mpc
