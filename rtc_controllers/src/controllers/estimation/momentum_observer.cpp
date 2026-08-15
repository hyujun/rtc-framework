#include "rtc_controllers/estimation/momentum_observer.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace rtc::estimation {
namespace {

/// True iff `s` covers at least `n` entries and every one of the first `n` is
/// finite. Width and finiteness are checked together because they have the same
/// consequence — the tick cannot be evaluated — and separating them would let a
/// caller believe a short span had been inspected.
[[nodiscard]] bool UsableInput(std::span<const double> s, int n) noexcept {
  if (s.size() < static_cast<std::size_t>(n))
    return false;
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(s[static_cast<std::size_t>(i)]))
      return false;
  }
  return true;
}

}  // namespace

void MomentumObserver::Init(int nv, std::span<const double> gains) {
  if (nv < 1 || nv > kMaxObserverDof) {
    throw std::invalid_argument("MomentumObserver::Init: nv out of range [1, " +
                                std::to_string(kMaxObserverDof) +
                                "], got " + std::to_string(nv));
  }
  if (gains.size() < static_cast<std::size_t>(nv)) {
    throw std::invalid_argument("MomentumObserver::Init: gains shorter than nv (" +
                                std::to_string(gains.size()) + " < " + std::to_string(nv) + ")");
  }
  for (int i = 0; i < nv; ++i) {
    const double k = gains[static_cast<std::size_t>(i)];
    if (!std::isfinite(k) || k <= 0.0) {
      throw std::invalid_argument("MomentumObserver::Init: gain[" + std::to_string(i) +
                                  "] must be finite and > 0, got " + std::to_string(k));
    }
  }

  dof_ = nv;
  gains_.fill(0.0);
  for (int i = 0; i < nv; ++i)
    gains_[static_cast<std::size_t>(i)] = gains[static_cast<std::size_t>(i)];

  initialized_ = true;
  ResetRtState();
}

void MomentumObserver::ResetRtState() noexcept {
  p0_.fill(0.0);
  intg_.fill(0.0);
  residual_.fill(0.0);
  seed_pending_ = true;
  ticks_since_seed_ = 0;
  reason_ = initialized_ ? MomentumInvalidReason::kHeld : MomentumInvalidReason::kNotInitialized;
}

void MomentumObserver::RejectTick(MomentumInvalidReason reason) noexcept {
  // The residual and the integrator are left exactly as they were: a rejected
  // tick carries no information about how they should have moved. The reseed is
  // armed because `p` keeps evolving while `intg` does not, so the reference
  // captured before this gap no longer describes the trajectory (see the
  // ResetRtState doc for what continuing would fabricate).
  reason_ = reason;
  seed_pending_ = true;
  ticks_since_seed_ = 0;
}

void MomentumObserver::Hold() noexcept {
  if (!initialized_) {
    reason_ = MomentumInvalidReason::kNotInitialized;
    return;
  }
  RejectTick(MomentumInvalidReason::kHeld);
}

void MomentumObserver::Update(std::span<const double> p, std::span<const double> c_t_v,
                              std::span<const double> g, std::span<const double> tau_m,
                              double dt) noexcept {
  if (!initialized_) {
    reason_ = MomentumInvalidReason::kNotInitialized;
    return;
  }

  // dt first: it scales every term below, and "the clock did not advance" is a
  // different statement from "an input was malformed".
  if (!std::isfinite(dt) || dt <= 0.0) {
    RejectTick(MomentumInvalidReason::kNonPositiveDt);
    return;
  }

  const int n = dof_;
  const bool widths_ok = p.size() >= static_cast<std::size_t>(n) &&
                         c_t_v.size() >= static_cast<std::size_t>(n) &&
                         g.size() >= static_cast<std::size_t>(n) &&
                         tau_m.size() >= static_cast<std::size_t>(n);
  if (!widths_ok) {
    RejectTick(MomentumInvalidReason::kShortInput);
    return;
  }
  if (!UsableInput(p, n) || !UsableInput(c_t_v, n) || !UsableInput(g, n) ||
      !UsableInput(tau_m, n)) {
    // Rejected, never clamped. Clamping a non-finite input would launder it into
    // a plausible number and hand the consumer a residual that looks measured.
    RejectTick(MomentumInvalidReason::kNonFiniteInput);
    return;
  }

  // Re-capture the momentum reference on the first tick after any gap. Done
  // BEFORE the residual is formed so this tick is already consistent: with
  // p0 == p and intg == 0 the bracket in [MO-3b] reduces to -dt·beta, which is
  // the one-sample residual of a fresh observer rather than a jump inherited
  // from before the gap.
  if (seed_pending_) {
    for (int i = 0; i < n; ++i)
      p0_[static_cast<std::size_t>(i)] = p[static_cast<std::size_t>(i)];
    intg_.fill(0.0);
    residual_.fill(0.0);
    seed_pending_ = false;
    ticks_since_seed_ = 0;
  }

  for (int i = 0; i < n; ++i) {
    const auto u = static_cast<std::size_t>(i);

    // [MO-3a] — tau_f is not modeled (see header).
    const double beta = tau_m[u] + c_t_v[u] - g[u];

    // [MO-3b] — backward-Euler implicit residual feedback. The implicit form is
    // why the gain divides by (1 + dt·K_I): the explicit alternative uses
    // r_{k-1} and diverges once dt·K_I grows past ~1, which is reachable at the
    // low end of the supported control rates.
    const double k = gains_[u];
    const double r = k / (1.0 + dt * k) * (p[u] - p0_[u] - intg_[u] - dt * beta);

    // [MO-3c] — the `+ r` is load-bearing: it closes the loop that turns the
    // open integral into the first-order filter of [MO-2]. Without it the
    // bracket above accumulates tau_ext without bound and r ramps forever.
    intg_[u] += dt * (beta + r);
    residual_[u] = r;
  }

  reason_ = MomentumInvalidReason::kNone;
  if (ticks_since_seed_ != std::numeric_limits<std::uint32_t>::max())
    ++ticks_since_seed_;
}

std::span<const double> MomentumObserver::residual() const noexcept {
  return std::span<const double>(residual_.data(), static_cast<std::size_t>(dof_));
}

}  // namespace rtc::estimation
