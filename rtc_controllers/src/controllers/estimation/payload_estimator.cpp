#include "rtc_controllers/estimation/payload_estimator.hpp"

#include "rtc_controllers/compliance/task_dynamics.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace rtc::estimation {
namespace {

/// Reject a threshold that cannot do its job. Non-finite is checked FIRST and
/// separately from the sign: `0.0 < NaN` is false, so a lone `<= 0.0` test lets
/// a NaN threshold through and every later comparison against it is false too —
/// the gate would be permanently open while reading as configured.
void RequirePositive(double v, const char* name) {
  if (!std::isfinite(v) || v <= 0.0) {
    throw std::invalid_argument(std::string("PayloadEstimator::Init: ") + name +
                                " must be finite and > 0, got " + std::to_string(v));
  }
}

}  // namespace

void PayloadEstimator::Init(int nv, const Config& config) {
  if (nv < 1 || nv > kMaxObserverDof) {
    throw std::invalid_argument("PayloadEstimator::Init: nv out of range [1, " +
                                std::to_string(kMaxObserverDof) + "], got " +
                                std::to_string(nv));
  }
  RequirePositive(config.sigma0, "sigma0");
  RequirePositive(config.lambda_max, "lambda_max");
  RequirePositive(config.min_sigma, "min_sigma");
  RequirePositive(config.max_fit_error, "max_fit_error");
  RequirePositive(config.min_gravity, "min_gravity");

  // Every allocation this class will ever make happens here (non-RT).
  nv_ = nv;
  config_ = config;
  dik_.Resize(nv, 6);
  r_buf_ = Eigen::VectorXd::Zero(nv);
  fit_buf_ = Eigen::VectorXd::Zero(nv);

  initialized_ = true;
  ResetRtState();
}

void PayloadEstimator::ResetRtState() noexcept {
  estimate_ = PayloadEstimate{};
  wrench_.setZero();
  reason_ = initialized_ ? PayloadInvalidReason::kHeld : PayloadInvalidReason::kNotInitialized;
}

void PayloadEstimator::Hold(PayloadInvalidReason reason) noexcept {
  if (!initialized_) {
    reason_ = PayloadInvalidReason::kNotInitialized;
    return;
  }
  // kNone would silently assert this tick was good; a caller that means "valid"
  // calls Update(). Anything else is taken at face value, including kHeld.
  reason_ = (reason == PayloadInvalidReason::kNone) ? PayloadInvalidReason::kHeld : reason;
}

void PayloadEstimator::Update(const Eigen::Ref<const Eigen::MatrixXd>& J,
                              std::span<const double> r,
                              const Eigen::Vector3d& gravity) noexcept {
  if (!initialized_) {
    reason_ = PayloadInvalidReason::kNotInitialized;
    return;
  }
  if (J.rows() != 6 || J.cols() != nv_ || r.size() < static_cast<std::size_t>(nv_)) {
    reason_ = PayloadInvalidReason::kShortInput;
    return;
  }
  // J is checked inside DifferentialIk::Compute; r and gravity are ours. All
  // three matter: a NaN anywhere would reach the LLT, which reports Success on
  // a matrix full of NaN because every pivot test is a comparison.
  for (int i = 0; i < nv_; ++i) {
    const double ri = r[static_cast<std::size_t>(i)];
    if (!std::isfinite(ri)) {
      reason_ = PayloadInvalidReason::kNonFiniteInput;
      return;
    }
    r_buf_(i) = ri;
  }
  if (!gravity.allFinite()) {
    reason_ = PayloadInvalidReason::kNonFiniteInput;
    return;
  }

  // [MASS-A] divides by ‖ᵂg‖², so this guard comes BEFORE the solve: a model
  // without usable gravity has no mass map, and reporting a wrench whose mass
  // we would then have to fabricate is worse than rejecting the tick (NUM-2).
  const double g_sq = gravity.squaredNorm();
  if (std::sqrt(g_sq) < config_.min_gravity) {
    reason_ = PayloadInvalidReason::kDegenerateGravity;
    return;
  }

  // [WRENCH-A]. λ_max is floored at the point of use, not only where it was
  // parsed, because a Config written straight into this struct bypasses any
  // parser (the NUM-1 rule compliance:: applies to the same law).
  const auto dls = dik_.Compute(J, config_.sigma0,
                                rtc::compliance::FloorMaxDamping(config_.lambda_max));
  if (!dls.ok) {
    reason_ = PayloadInvalidReason::kSolverFailed;
    return;
  }
  if (dls.sigma_min < config_.min_sigma) {
    // Damping keeps the arithmetic finite here, so this is not a numerical
    // failure — it is an OBSERVABILITY one. At least one wrench direction lies
    // in the near-null space of Jᵀ, meaning a payload could be present and
    // leave no trace in r. A finite answer would still be unjustified.
    reason_ = PayloadInvalidReason::kRankDeficient;
    return;
  }

  // ŵ = (J⁺)ᵀ r. J⁺ is nv×6, so this is one 6×nv · nv product — no allocation,
  // and the factorisation was already paid for above.
  wrench_.noalias() = dik_.PseudoInverse().transpose() * r_buf_;

  // Reconstruction gate. This is what separates a payload from an unmodelled
  // joint-level torque (armature / damping / friction, #135 [MO-1u]): those do
  // not lie in the range of Jᵀ, so no wrench explains them and the leftover is
  // large. Computed against the SAME J and r the solve used.
  fit_buf_.noalias() = J.transpose() * wrench_;
  fit_buf_ -= r_buf_;
  const double fit_error = fit_buf_.lpNorm<Eigen::Infinity>();
  if (!std::isfinite(fit_error) || fit_error > config_.max_fit_error) {
    reason_ = PayloadInvalidReason::kPoorFit;
    return;
  }

  // [MASS-A]: m̂ = (f̂ · ᵂg) / ‖ᵂg‖². Positive for a real payload precisely
  // because f̂ points along ᵂg — see the sign contract in the header.
  const Eigen::Vector3d f = wrench_.head<3>();
  const double mass = f.dot(gravity) / g_sq;
  if (!std::isfinite(mass)) {
    reason_ = PayloadInvalidReason::kPoorFit;
    return;
  }

  for (int i = 0; i < 6; ++i)
    estimate_.wrench[static_cast<std::size_t>(i)] = wrench_(i);
  estimate_.mass = mass;
  estimate_.sigma_min = dls.sigma_min;
  estimate_.lambda_sq = dls.lambda_sq;
  estimate_.fit_error = fit_error;
  reason_ = PayloadInvalidReason::kNone;
}

}  // namespace rtc::estimation
