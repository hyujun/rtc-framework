#include "integrated_bringup/controllers/wbc/force_reference_updater.hpp"

#include <algorithm>

namespace integrated_bringup::wbc {

namespace {

constexpr double Clamp(double v, double lo, double hi) noexcept {
  return std::min(std::max(v, lo), hi);
}

}  // namespace

void ForceReferenceUpdater::Reset() noexcept {
  integrator_.fill(0.0);
  last_output_.fill(0.0);
}

double ForceReferenceUpdater::Update(int contact_index, bool measured_valid, double f_des,
                                     double f_meas, double dt) noexcept {
  if (contact_index < 0 || contact_index >= kMaxForceContacts) {
    return 0.0;
  }
  const auto idx = static_cast<std::size_t>(contact_index);

  if (!measured_valid) {
    // Reset this contact's integrator and zero the output so the next
    // valid tick starts from a known state (no spike from stale I_i).
    integrator_[idx] = 0.0;
    last_output_[idx] = 0.0;
    return 0.0;
  }

  const double f_des_eff = (f_des > 0.0) ? f_des : cfg_.f_des_default;
  const double err = f_des_eff - f_meas;

  // Provisional integrator step (clamped to ±i_max).
  const double i_prev = integrator_[idx];
  double i_next = i_prev;
  if (dt > 0.0) {
    i_next = Clamp(i_prev + cfg_.ki * err * dt, -cfg_.i_max, cfg_.i_max);
  }

  // Conditional integration: only commit i_next if it does not push
  // further into the saturated direction. This avoids wind-up while
  // letting the integrator move freely when the output is in-range or
  // when the step would *unwind* a previously saturated state.
  const double raw_with_step = cfg_.kp * err + i_next + f_des_eff;
  const bool clipped_high = raw_with_step > cfg_.lambda_max;
  const bool clipped_low = raw_with_step < cfg_.lambda_min;
  const bool step_pushes_higher = i_next > i_prev;
  const bool step_pushes_lower = i_next < i_prev;
  const bool wind_up_high = clipped_high && step_pushes_higher;
  const bool wind_up_low = clipped_low && step_pushes_lower;

  integrator_[idx] = (wind_up_high || wind_up_low) ? i_prev : i_next;

  // Recompute output with the committed integrator and saturate.
  double raw = cfg_.kp * err + integrator_[idx] + f_des_eff;
  raw = Clamp(raw, cfg_.lambda_min, cfg_.lambda_max);

  last_output_[idx] = raw;
  return raw;
}

double ForceReferenceUpdater::GetIntegratorForTesting(int contact_index) const noexcept {
  if (contact_index < 0 || contact_index >= kMaxForceContacts) {
    return 0.0;
  }
  return integrator_[static_cast<std::size_t>(contact_index)];
}

double ForceReferenceUpdater::GetLastOutputForTesting(int contact_index) const noexcept {
  if (contact_index < 0 || contact_index >= kMaxForceContacts) {
    return 0.0;
  }
  return last_output_[static_cast<std::size_t>(contact_index)];
}

}  // namespace integrated_bringup::wbc
