#include "rtc_controllers/grasp/pull_force_estimator.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace rtc::grasp {

namespace {
/// Guard against division by zero in the friction utilization ratio.
constexpr double kSlipEpsilon = 1e-6;
/// Below this norm a direction/normal vector is considered degenerate.
constexpr double kDegenerateNorm = 1e-9;
/// The carried e_x is re-projected onto each tick's plane; below this residual
/// norm the carry has collapsed (plane rotated ~90 deg from it) and amplifying
/// it would be noise, so the basis reseeds from a world axis instead.
constexpr double kBasisCarryMin = 1e-3;

/// Throw unless `v` is finite. NaN/Inf silently pass most of the ordered
/// comparisons below (`NaN <= 0.0` is false), so a config that came from a
/// mis-typed YAML scalar would configure cleanly and then poison every tick
/// (#234, cross-review §3). Checked before the range tests so the message
/// names the real problem.
void RequireFinite(double v, const char* what) {
  if (!std::isfinite(v)) {
    throw std::invalid_argument(std::string("PullForceEstimator: ") + what + " must be finite");
  }
}
}  // namespace

void PullForceEstimator::Init(std::span<const PullContactConfig> configs,
                              const PullEstimatorParams& params) {
  if (configs.empty()) {
    throw std::invalid_argument("PullForceEstimator: no contacts configured");
  }
  if (configs.size() > static_cast<std::size_t>(kMaxPullContacts)) {
    throw std::invalid_argument("PullForceEstimator: too many contacts");
  }
  if (params.min_valid_contacts < 1) {
    throw std::invalid_argument("PullForceEstimator: min_valid_contacts must be >= 1");
  }
  if (params.min_valid_contacts > static_cast<int>(configs.size())) {
    // Every other Init check hard-fails on nonsense; this one used to pass and
    // leave a permanently invalid estimator running silently (#234 P-6). The
    // required-role count is deliberately *not* cross-checked: validity is
    // `!required_missing && count >= min`, so a required set larger than `min`
    // is already enforced by the AND and constraining it would reject sound
    // configs (cross-review §3).
    throw std::invalid_argument(
        "PullForceEstimator: min_valid_contacts exceeds the configured contact count");
  }
  RequireFinite(params.decay_time_constant_s, "decay_time_constant_s");
  RequireFinite(params.slip_risk_threshold, "slip_risk_threshold");
  RequireFinite(params.alignment_error_rad, "alignment_error_rad");
  if (params.decay_time_constant_s <= 0.0) {
    throw std::invalid_argument("PullForceEstimator: decay_time_constant_s must be > 0");
  }
  if (params.slip_risk_threshold <= 0.0) {
    throw std::invalid_argument("PullForceEstimator: slip_risk_threshold must be > 0");
  }
  if (params.alignment_error_rad < 0.0) {
    throw std::invalid_argument("PullForceEstimator: alignment_error_rad must be >= 0");
  }
  if (!params.gravity_force.allFinite()) {
    throw std::invalid_argument("PullForceEstimator: gravity_force must be finite");
  }
  if (!params.inplane_x_reference.allFinite()) {
    throw std::invalid_argument("PullForceEstimator: inplane_x_reference must be finite");
  }

  for (const PullContactConfig& cfg : configs) {
    RequireFinite(cfg.contact_on_threshold, "contact_on_threshold");
    RequireFinite(cfg.contact_off_threshold, "contact_off_threshold");
    RequireFinite(cfg.touch_on_threshold, "touch_on_threshold");
    RequireFinite(cfg.touch_off_threshold, "touch_off_threshold");
    RequireFinite(cfg.force_saturation, "force_saturation");
    RequireFinite(cfg.friction_coeff, "friction_coeff");
    RequireFinite(cfg.force_sign, "force_sign");

    if (!(cfg.contact_on_threshold > cfg.contact_off_threshold) ||
        !(cfg.contact_off_threshold > 0.0)) {
      throw std::invalid_argument("PullForceEstimator: hysteresis requires on > off > 0 [N]");
    }
    // Touch pair is optional (zero = unset ⇒ reuse the contact pair), but a
    // half-set pair is a typo, not a policy: silently pairing a configured
    // on-threshold with an implicit zero off-threshold would latch the touch
    // gate on forever (#234 P-16).
    const bool touch_on_set = cfg.touch_on_threshold != 0.0;
    const bool touch_off_set = cfg.touch_off_threshold != 0.0;
    if (touch_on_set != touch_off_set) {
      throw std::invalid_argument(
          "PullForceEstimator: touch_on_threshold and touch_off_threshold must both be set or "
          "both omitted");
    }
    if (touch_on_set &&
        (!(cfg.touch_on_threshold > cfg.touch_off_threshold) || !(cfg.touch_off_threshold > 0.0))) {
      throw std::invalid_argument("PullForceEstimator: touch hysteresis requires on > off > 0 [N]");
    }
    if (cfg.force_saturation <= 0.0) {
      throw std::invalid_argument("PullForceEstimator: force_saturation must be > 0");
    }
    if (cfg.friction_coeff <= 0.0) {
      throw std::invalid_argument("PullForceEstimator: friction_coeff must be > 0");
    }
    // The header documents force_sign as a convention selector, not a gain:
    // anything else silently rescales the whole estimate (and a 0 would zero
    // it) while every downstream unit still reads as newtons.
    if (cfg.force_sign != 1.0 && cfg.force_sign != -1.0) {
      throw std::invalid_argument("PullForceEstimator: force_sign must be +1 or -1");
    }
    if (!cfg.force_calibration.allFinite() || !cfg.force_bias.allFinite()) {
      throw std::invalid_argument("PullForceEstimator: calibration must be finite");
    }
  }

  num_contacts_ = static_cast<int>(configs.size());
  for (int i = 0; i < num_contacts_; ++i) {
    configs_[static_cast<std::size_t>(i)] = configs[static_cast<std::size_t>(i)];
  }
  params_ = params;
  sin_alignment_error_ = std::sin(params.alignment_error_rad);

  // May throw on invalid cutoff/sample rates. Must run before any Apply() —
  // an uninitialized BesselFilterN silently returns zero.
  filter_.Init(params.filter_cutoff_hz, params.sample_rate_hz);

  ResetRtState();
  initialized_ = true;
}

void PullForceEstimator::ResetRtState() noexcept {
  contact_active_.fill(false);
  touch_active_.fill(false);
  filtered_.setZero();
  prev_e_x_.setZero();
  prev_e_x_valid_ = false;
  // Nothing to reseed: the filter and the published output are both zero here,
  // so the two already agree and a cold start keeps its documented ramp from
  // rest rather than snapping to the first raw sample.
  filter_reseed_pending_ = false;
  ClearBaseline();
  filter_.Reset();
  estimate_ = PullEstimate{};
}

void PullForceEstimator::SetPullDirection(const Eigen::Vector3d& direction) noexcept {
  const double norm = direction.norm();
  if (std::isfinite(norm) && norm >= kDegenerateNorm) {
    pull_direction_ = direction / norm;
    pull_direction_set_ = true;
  } else {
    pull_direction_.setZero();
    pull_direction_set_ = false;
  }
}

PullBasisSource PullForceEstimator::MakePlaneBasis(const Eigen::Vector3d& n, Eigen::Vector3d& e_x,
                                                   Eigen::Vector3d& e_y) noexcept {
  // Continuity first: re-project the previous e_x onto this tick's plane. A
  // seed-only basis is discontinuous in n (the |n.x()| branch flips e_x by 180
  // deg at the boundary), and the plane normal now moves discretely whenever
  // the grasp shape changes — that would step force_inplane while the filtered
  // force stays smooth, reading as a spurious lateral pull.
  Eigen::Vector3d candidate = Eigen::Vector3d::Zero();
  double norm = 0.0;
  PullBasisSource source = PullBasisSource::kNone;

  // Preferred: the configured reference direction (anti-gravity by default)
  // projected into the plane. This *names* the axes — force_inplane[0] is the
  // vertical pull component — and is a pure function of n, so it is also
  // path-independent, unlike the carry below. It collapses only when the plane
  // turns edge-on to the reference (pinch axis parallel to gravity).
  if (params_.inplane_x_reference.squaredNorm() > 0.0) {
    candidate = params_.inplane_x_reference - (params_.inplane_x_reference.dot(n)) * n;
    norm = candidate.norm();
    // Guard the ill-conditioned band, not just the exact singularity: near
    // edge-on, the projection is dominated by numerical noise in n and the
    // axis would jitter. kBasisCarryMin is the same collapse threshold the
    // carry path uses.
    if (!(std::isfinite(norm) && norm >= kBasisCarryMin)) {
      norm = 0.0;
    } else {
      source = PullBasisSource::kReference;
    }
  }

  if (norm == 0.0 && prev_e_x_valid_) {
    candidate = prev_e_x_ - (prev_e_x_.dot(n)) * n;
    norm = candidate.norm();
    source = PullBasisSource::kCarry;
  }
  if (!(std::isfinite(norm) && norm >= kBasisCarryMin)) {
    // No usable carry: pick the world axis least aligned with n. Path-
    // independent, so this is also the deterministic entry point after Reset.
    const Eigen::Vector3d seed =
        (std::abs(n.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    candidate = seed - (seed.dot(n)) * n;
    norm = candidate.norm();
    source = PullBasisSource::kSeed;
  }
  if (!(std::isfinite(norm) && norm >= kDegenerateNorm)) {
    // n is not a unit vector (caller contract broken) — leave the basis zero
    // rather than emitting NaN into the published in-plane coordinates.
    e_x.setZero();
    e_y.setZero();
    return PullBasisSource::kNone;
  }
  e_x = candidate / norm;
  e_y = n.cross(e_x);
  prev_e_x_ = e_x;
  prev_e_x_valid_ = true;
  return source;
}

const PullEstimate& PullForceEstimator::Update(std::span<const PullContactInput> inputs,
                                               const Eigen::Vector3d& plane_normal,
                                               double dt) noexcept {
  // Reset per-tick diagnostics; keep filtered_ / baseline state across ticks.
  estimate_.valid_contact_count = 0;
  estimate_.max_friction_utilization = 0.0;
  estimate_.leakage_bound = 0.0;
  estimate_.any_saturated = false;
  estimate_.slip_risk = false;
  estimate_.valid = false;
  estimate_.baseline_applied = baseline_captured_;
  estimate_.contact_mask = 0;
  estimate_.touch_mask = 0;
  estimate_.plane_normal.setZero();
  estimate_.basis_x.setZero();
  estimate_.basis_source = PullBasisSource::kNone;

  const double n_norm = plane_normal.norm();
  const bool normal_ok = initialized_ && std::isfinite(n_norm) && n_norm >= kDegenerateNorm;

  Eigen::Vector3d force_sum = Eigen::Vector3d::Zero();
  bool required_missing = false;

  // The normal is only needed from the contact-hysteresis step onwards. The
  // sensor/saturation gates and the *touch* hysteresis run unconditionally, so
  // a degenerate normal cannot freeze touch_active_ — a caller that derives the
  // normal from the touch set would otherwise have no way back once the normal
  // went to zero (the very deadlock touch_active exists to avoid).
  const Eigen::Vector3d n =
      normal_ok ? Eigen::Vector3d(plane_normal / n_norm) : Eigen::Vector3d::Zero();

  for (int i = 0; i < num_contacts_; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    const PullContactConfig& cfg = configs_[idx];

    // Missing or invalid sensor: reset hysteresis (invalid-sensor policy)
    // and skip — the contact contributes nothing this tick.
    if (idx >= inputs.size() || !inputs[idx].valid || !inputs[idx].force.allFinite() ||
        !inputs[idx].rotation.allFinite()) {
      contact_active_[idx] = false;
      touch_active_[idx] = false;
      if (cfg.required) {
        required_missing = true;
      }
      continue;
    }
    const PullContactInput& in = inputs[idx];

    // Saturation gates on the *raw* sensor value (sensor-limit semantics).
    if (in.force.cwiseAbs().maxCoeff() >= cfg.force_saturation) {
      estimate_.any_saturated = true;
      contact_active_[idx] = false;
      touch_active_[idx] = false;
      if (cfg.required) {
        required_missing = true;
      }
      continue;
    }

    // Wire contract → finger-on-object, common reference frame.
    const Eigen::Vector3d f_obj =
        cfg.force_sign * (in.rotation * (cfg.force_calibration * (in.force - cfg.force_bias)));

    // Touch hysteresis on |f_obj| — "this tip is pressing on something",
    // independent of any plane normal. Defaults to the contact thresholds, in
    // which case |f_obj| >= |f_n| makes the touch set a superset of the contact
    // set; a config that sets the touch pair explicitly (#234 P-16) buys
    // independent axis-selection sensitivity and gives that containment up.
    const bool touch_split = cfg.touch_on_threshold != 0.0;
    const double touch_on = touch_split ? cfg.touch_on_threshold : cfg.contact_on_threshold;
    const double touch_off = touch_split ? cfg.touch_off_threshold : cfg.contact_off_threshold;
    const double f_mag = f_obj.norm();
    touch_active_[idx] = touch_active_[idx] ? (f_mag > touch_off) : (f_mag > touch_on);
    if (touch_active_[idx]) {
      estimate_.touch_mask = static_cast<std::uint8_t>(estimate_.touch_mask | (1U << idx));
    }

    if (!normal_ok) {
      contact_active_[idx] = false;
      if (cfg.required) {
        required_missing = true;
      }
      continue;
    }

    // Per-tick contact normal, already in the reference frame (FK-resolved by
    // the caller from pinch geometry — not a body-fixed axis). Degenerate or
    // non-finite ⇒ this contact is unusable this tick: reset hysteresis, skip.
    const double cn_norm = in.contact_normal.norm();
    if (!(std::isfinite(cn_norm) && cn_norm >= kDegenerateNorm)) {
      contact_active_[idx] = false;
      if (cfg.required) {
        required_missing = true;
      }
      continue;
    }
    const Eigen::Vector3d n_contact = in.contact_normal / cn_norm;

    // Compression positive: n_contact points from object into the finger.
    const double f_n = -n_contact.dot(f_obj);

    // Contact hysteresis on the normal force.
    contact_active_[idx] =
        contact_active_[idx] ? (f_n > cfg.contact_off_threshold) : (f_n > cfg.contact_on_threshold);
    if (!contact_active_[idx]) {
      if (cfg.required) {
        required_missing = true;
      }
      continue;
    }

    force_sum += f_obj;
    ++estimate_.valid_contact_count;
    estimate_.contact_mask = static_cast<std::uint8_t>(estimate_.contact_mask | (1U << idx));

    const Eigen::Vector3d f_t = f_obj - (n_contact.dot(f_obj)) * n_contact;
    const double utilization = f_t.norm() / (cfg.friction_coeff * f_n + kSlipEpsilon);
    estimate_.max_friction_utilization = std::max(estimate_.max_friction_utilization, utilization);
    estimate_.leakage_bound += std::abs(f_n) * sin_alignment_error_;
  }

  // Why the tick failed, in gate order — the three causes are indistinguishable
  // from `valid` alone, which is what makes a joint-vs-WBC trace unreadable
  // (#234 P-11: WBC additionally requires a TSID-routing tick, so its release /
  // fallback ticks land on kInsufficientContacts while joint's land nowhere).
  if (!initialized_) {
    estimate_.invalid_reason = PullInvalidReason::kNotInitialized;
  } else if (!normal_ok) {
    estimate_.invalid_reason = PullInvalidReason::kDegenerateNormal;
  } else if (required_missing) {
    estimate_.invalid_reason = PullInvalidReason::kRequiredContactMissing;
  } else if (estimate_.valid_contact_count < params_.min_valid_contacts) {
    estimate_.invalid_reason = PullInvalidReason::kInsufficientContacts;
  } else {
    estimate_.invalid_reason = PullInvalidReason::kNone;
  }

  // Friction utilization is a per-contact quantity: it needs an active contact,
  // not an in-plane vector. Evaluating it inside the `valid` gate reported
  // slip_risk=false exactly when a required role dropped out — including when
  // it dropped out *because* that tip was sliding (#234 P-7). The remaining
  // active contacts still have a well-defined ratio, so publish it; with no
  // active contact the max is vacuously zero and the flag stays false.
  estimate_.slip_risk = estimate_.valid_contact_count > 0 &&
                        estimate_.max_friction_utilization >= params_.slip_risk_threshold;

  if (normal_ok) {
    estimate_.valid = estimate_.invalid_reason == PullInvalidReason::kNone;
    estimate_.plane_normal = n;

    if (estimate_.valid) {
      // F̂ = -P∥ (Σ f_obj + m·g); P∥ x = x - n(nᵀx).
      const Eigen::Vector3d total = force_sum + params_.gravity_force;

      if (baseline_armed_) {
        // Store the *unprojected* reference so the subtraction follows the
        // plane. Capturing -P∥(total) instead would leave a constant residual
        // in the old plane the moment the grasp shape rotates the normal.
        baseline_ref_ = total;
        baseline_armed_ = false;
        baseline_captured_ = true;
        estimate_.baseline_applied = true;
      }
      const Eigen::Vector3d effective = baseline_captured_ ? (total - baseline_ref_) : total;
      const Eigen::Vector3d raw = -(effective - n * (n.dot(effective)));
      estimate_.force_raw = raw;

      if (filter_reseed_pending_) {
        // First valid sample after a gap. The delay line still describes the
        // pre-gap force while `filtered_` has been decaying independently of
        // it, so continuing would snap the output back in one sample and undo
        // every decayed tick already published (#234 P-4 / D3). Seeding makes
        // this tick's output exactly `raw` and the filter resumes from there.
        filter_.Reset();
        filter_.Seed({raw.x(), raw.y(), raw.z()});
        filter_reseed_pending_ = false;
      }
      const std::array<double, 3> filtered_arr = filter_.Apply({raw.x(), raw.y(), raw.z()});
      filtered_ = Eigen::Vector3d(filtered_arr[0], filtered_arr[1], filtered_arr[2]);

      Eigen::Vector3d e_x;
      Eigen::Vector3d e_y;
      estimate_.basis_source = MakePlaneBasis(n, e_x, e_y);
      estimate_.basis_x = e_x;
      estimate_.force_inplane = Eigen::Vector2d(e_x.dot(filtered_), e_y.dot(filtered_));
    }
  }
  // No else: on a degenerate normal the loop above already cleared every
  // contact_active_ (while leaving touch_active_ live), and estimate_.valid
  // stays false from the per-tick reset.

  if (!estimate_.valid) {
    // Bounded decay instead of a hard zero — avoids output discontinuities
    // on transient contact loss; valid=false tells consumers not to act on it.
    //
    // dt <= 0 collapses to a hard zero rather than holding: a clock that did
    // not advance says nothing about how far the estimate should have decayed,
    // and holding would let a stalled caller keep republishing a stale force.
    const double decay = (dt > 0.0) ? std::exp(-dt / params_.decay_time_constant_s) : 0.0;
    filtered_ *= decay;
    estimate_.force_raw.setZero();
    estimate_.force_inplane *= decay;
    // The filter is no longer the thing producing `filtered_`; whatever its
    // delay line holds is now stale by construction. Reseed on return.
    filter_reseed_pending_ = true;
  }

  estimate_.force_filtered = filtered_;
  estimate_.magnitude = filtered_.norm();
  estimate_.directional = pull_direction_set_ ? pull_direction_.dot(filtered_) : 0.0;
  return estimate_;
}

void FillPullEstimateData(const PullEstimate& in, PullEstimateData& out) noexcept {
  out.force = {static_cast<float>(in.force_filtered.x()), static_cast<float>(in.force_filtered.y()),
               static_cast<float>(in.force_filtered.z())};
  out.force_inplane = {static_cast<float>(in.force_inplane.x()),
                       static_cast<float>(in.force_inplane.y())};
  out.plane_normal = {static_cast<float>(in.plane_normal.x()),
                      static_cast<float>(in.plane_normal.y()),
                      static_cast<float>(in.plane_normal.z())};
  out.basis_x = {static_cast<float>(in.basis_x.x()), static_cast<float>(in.basis_x.y()),
                 static_cast<float>(in.basis_x.z())};
  out.basis_source = static_cast<std::uint8_t>(in.basis_source);
  out.invalid_reason = static_cast<std::uint8_t>(in.invalid_reason);
  out.contact_mask = in.contact_mask;
  out.touch_mask = in.touch_mask;
  out.magnitude = static_cast<float>(in.magnitude);
  out.directional = static_cast<float>(in.directional);
  out.friction_utilization = static_cast<float>(in.max_friction_utilization);
  out.leakage_bound = static_cast<float>(in.leakage_bound);
  out.valid_contact_count = in.valid_contact_count;
  out.valid = in.valid;
  out.slip_risk = in.slip_risk;
  out.any_saturated = in.any_saturated;
  out.baseline_applied = in.baseline_applied;
}

}  // namespace rtc::grasp
