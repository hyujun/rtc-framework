#include "rtc_controllers/grasp/pull_force_estimator.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace rtc::grasp {

namespace {
/// Guard against division by zero in the friction utilization ratio.
constexpr double kSlipEpsilon = 1e-6;
/// Below this norm a direction/normal vector is considered degenerate.
constexpr double kDegenerateNorm = 1e-9;
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

  for (const PullContactConfig& cfg : configs) {
    if (cfg.contact_normal_local.norm() < kDegenerateNorm) {
      throw std::invalid_argument("PullForceEstimator: contact_normal_local is degenerate");
    }
    if (!(cfg.contact_on_threshold > cfg.contact_off_threshold) ||
        !(cfg.contact_off_threshold > 0.0)) {
      throw std::invalid_argument("PullForceEstimator: hysteresis requires on > off > 0 [N]");
    }
    if (cfg.force_saturation <= 0.0) {
      throw std::invalid_argument("PullForceEstimator: force_saturation must be > 0");
    }
    if (cfg.friction_coeff <= 0.0) {
      throw std::invalid_argument("PullForceEstimator: friction_coeff must be > 0");
    }
    if (!cfg.force_calibration.allFinite() || !cfg.force_bias.allFinite()) {
      throw std::invalid_argument("PullForceEstimator: calibration must be finite");
    }
  }

  num_contacts_ = static_cast<int>(configs.size());
  for (int i = 0; i < num_contacts_; ++i) {
    auto& cfg = configs_[static_cast<std::size_t>(i)];
    cfg = configs[static_cast<std::size_t>(i)];
    cfg.contact_normal_local.normalize();
  }
  params_ = params;
  sin_alignment_error_ = std::sin(params.alignment_error_rad);

  // May throw on invalid cutoff/sample rates. Must run before any Apply() —
  // an uninitialized BesselFilterN silently returns zero.
  filter_.Init(params.filter_cutoff_hz, params.sample_rate_hz);

  contact_active_.fill(false);
  filtered_.setZero();
  ClearBaseline();
  estimate_ = PullEstimate{};
  initialized_ = true;
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

void PullForceEstimator::MakePlaneBasis(const Eigen::Vector3d& n, Eigen::Vector3d& e_x,
                                        Eigen::Vector3d& e_y) noexcept {
  // Pick the world axis least aligned with n, project it into the plane.
  const Eigen::Vector3d seed =
      (std::abs(n.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
  e_x = seed - (seed.dot(n)) * n;
  e_x.normalize();
  e_y = n.cross(e_x);
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

  const double n_norm = plane_normal.norm();
  const bool normal_ok = initialized_ && std::isfinite(n_norm) && n_norm >= kDegenerateNorm;

  Eigen::Vector3d force_sum = Eigen::Vector3d::Zero();
  bool required_missing = false;

  if (normal_ok) {
    const Eigen::Vector3d n = plane_normal / n_norm;

    for (int i = 0; i < num_contacts_; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      const PullContactConfig& cfg = configs_[idx];

      // Missing or invalid sensor: reset hysteresis (invalid-sensor policy)
      // and skip — the contact contributes nothing this tick.
      if (idx >= inputs.size() || !inputs[idx].valid || !inputs[idx].force.allFinite() ||
          !inputs[idx].rotation.allFinite()) {
        contact_active_[idx] = false;
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
        if (cfg.required) {
          required_missing = true;
        }
        continue;
      }

      // Wire contract → finger-on-object, common reference frame.
      const Eigen::Vector3d f_obj =
          cfg.force_sign * (in.rotation * (cfg.force_calibration * (in.force - cfg.force_bias)));
      const Eigen::Vector3d n_contact = in.rotation * cfg.contact_normal_local;

      // Compression positive: n_contact points from object into the finger.
      const double f_n = -n_contact.dot(f_obj);

      // Contact hysteresis on the normal force.
      contact_active_[idx] = contact_active_[idx] ? (f_n > cfg.contact_off_threshold)
                                                  : (f_n > cfg.contact_on_threshold);
      if (!contact_active_[idx]) {
        if (cfg.required) {
          required_missing = true;
        }
        continue;
      }

      force_sum += f_obj;
      ++estimate_.valid_contact_count;

      const Eigen::Vector3d f_t = f_obj - (n_contact.dot(f_obj)) * n_contact;
      const double utilization = f_t.norm() / (cfg.friction_coeff * f_n + kSlipEpsilon);
      estimate_.max_friction_utilization =
          std::max(estimate_.max_friction_utilization, utilization);
      estimate_.leakage_bound += std::abs(f_n) * sin_alignment_error_;
    }

    estimate_.valid =
        !required_missing && estimate_.valid_contact_count >= params_.min_valid_contacts;

    if (estimate_.valid) {
      // F̂ = -P∥ (Σ f_obj + m·g); P∥ x = x - n(nᵀx).
      const Eigen::Vector3d total = force_sum + params_.gravity_force;
      Eigen::Vector3d raw = -(total - n * (n.dot(total)));

      if (baseline_armed_) {
        baseline_ = raw;
        baseline_armed_ = false;
        baseline_captured_ = true;
        estimate_.baseline_applied = true;
      }
      if (baseline_captured_) {
        raw -= baseline_;
      }
      estimate_.force_raw = raw;

      const std::array<double, 3> filtered_arr = filter_.Apply({raw.x(), raw.y(), raw.z()});
      filtered_ = Eigen::Vector3d(filtered_arr[0], filtered_arr[1], filtered_arr[2]);

      Eigen::Vector3d e_x;
      Eigen::Vector3d e_y;
      MakePlaneBasis(n, e_x, e_y);
      estimate_.force_inplane = Eigen::Vector2d(e_x.dot(filtered_), e_y.dot(filtered_));
      estimate_.slip_risk = estimate_.max_friction_utilization >= params_.slip_risk_threshold;
    }
  } else {
    // Degenerate normal / uninitialized: every contact is unusable this tick.
    contact_active_.fill(false);
  }

  if (!estimate_.valid) {
    // Bounded decay instead of a hard zero — avoids output discontinuities
    // on transient contact loss; valid=false tells consumers not to act on it.
    const double decay = (dt > 0.0) ? std::exp(-dt / params_.decay_time_constant_s) : 0.0;
    filtered_ *= decay;
    estimate_.force_raw.setZero();
    estimate_.force_inplane *= decay;
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
