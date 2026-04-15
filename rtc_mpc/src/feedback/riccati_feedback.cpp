#include "rtc_mpc/feedback/riccati_feedback.hpp"

#include <algorithm>
#include <cmath>

namespace rtc::mpc {

void RiccatiFeedback::Init(int max_nv, int max_nu, int max_nx) {
  if (max_nv <= 0 || max_nu <= 0 || max_nx <= 0) {
    max_nv_ = 0;
    max_nu_ = 0;
    max_nx_ = 0;
    nu_ = 0;
    nx_ = 0;
    return;
  }
  max_nv_ = max_nv;
  max_nu_ = max_nu;
  max_nx_ = max_nx;
  K_.setZero(max_nu, max_nx);
  dx_.setZero(max_nx);
  u_fb_raw_.setZero(max_nu);
  nu_ = 0;
  nx_ = 0;
}

void RiccatiFeedback::SetGain(const double* K_data, int nu, int nx) noexcept {
  if (K_data == nullptr || nu <= 0 || nx <= 0 || nu > max_nu_ ||
      nx > max_nx_) {
    nu_ = 0;
    nx_ = 0;
    return;
  }
  // K_data is row-major nu × nx. Copy into the top-left nu × nx block.
  for (int row = 0; row < nu; ++row) {
    for (int col = 0; col < nx; ++col) {
      K_(row, col) = K_data[row * nx + col];
    }
  }
  nu_ = nu;
  nx_ = nx;
}

void RiccatiFeedback::Compute(
    const Eigen::Ref<const Eigen::VectorXd>& q_curr,
    const Eigen::Ref<const Eigen::VectorXd>& v_curr,
    const Eigen::Ref<const Eigen::VectorXd>& q_ref,
    const Eigen::Ref<const Eigen::VectorXd>& v_ref,
    Eigen::Ref<Eigen::VectorXd> u_fb_out) noexcept {
  if (nu_ <= 0 || nx_ <= 0) {
    u_fb_out.setZero();
    return;
  }

  const int nq = static_cast<int>(q_curr.size());
  const int nv = static_cast<int>(v_curr.size());
  if (nq != q_ref.size() || nv != v_ref.size() || nq + nv > nx_) {
    u_fb_out.setZero();
    return;
  }

  // Pack Δx = [Δq; Δv].
  dx_.head(nq) = q_curr.head(nq) - q_ref.head(nq);
  dx_.segment(nq, nv) = v_curr.head(nv) - v_ref.head(nv);
  // Zero any tail rows not covered by the current state dims.
  if (nq + nv < nx_) {
    dx_.segment(nq + nv, nx_ - nq - nv).setZero();
  }

  // Magnitude guard: linearly scale Δx down when its norm exceeds the cap.
  const double dx_norm = dx_.head(nx_).norm();
  double effective_scale = gain_scale_;
  if (dx_norm > max_delta_x_norm_) {
    effective_scale *= max_delta_x_norm_ / dx_norm;
  }

  // u_fb = scale * K * Δx  (noalias: avoid Eigen temporaries).
  u_fb_raw_.head(nu_).noalias() =
      effective_scale * K_.topLeftCorner(nu_, nx_) * dx_.head(nx_);

  // Write to the caller's buffer. In accel_only mode we populate only the
  // first `nv` entries (the portion consumed by TSID acceleration tasks).
  const int n_write = accel_only_ ? std::min(nv, nu_)
                                   : std::min(static_cast<int>(u_fb_out.size()),
                                              nu_);
  u_fb_out.head(n_write) = u_fb_raw_.head(n_write);
  if (u_fb_out.size() > n_write) {
    u_fb_out.tail(u_fb_out.size() - n_write).setZero();
  }
}

void RiccatiFeedback::SetGainScale(double scale) noexcept {
  gain_scale_ = std::clamp(scale, 0.0, 1.0);
}

void RiccatiFeedback::SetMaxDeltaXNorm(double value) noexcept {
  if (value > 0.0 && std::isfinite(value)) {
    max_delta_x_norm_ = value;
  }
}

}  // namespace rtc::mpc
