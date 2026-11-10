#include "rtc_mpc/feedback/riccati_feedback.hpp"

#include <algorithm>
#include <cmath>

namespace rtc::mpc {

bool PackRiccatiGainRowMajor(const RiccatiGainBlockRef& K, double* flat,
                             std::size_t flat_capacity) noexcept {
  if (flat == nullptr) {
    return false;
  }
  const std::size_t rows = static_cast<std::size_t>(K.rows());
  const std::size_t cols = static_cast<std::size_t>(K.cols());
  if (rows * cols > flat_capacity) {
    return false;
  }
  // The row-major contract lives in the Map's type: element (i, j) lands at
  // flat[i * cols + j]. Plain assignment, no temporary.
  using RowMajorMap =
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
  RowMajorMap(flat, K.rows(), K.cols()) = K;
  return true;
}

void RiccatiFeedback::Init(int max_nv, int max_nu, int max_ndx) {
  if (max_nv <= 0 || max_nu <= 0 || max_ndx <= 0) {
    max_nv_ = 0;
    max_nu_ = 0;
    max_ndx_ = 0;
    nu_ = 0;
    ndx_ = 0;
    return;
  }
  max_nv_ = max_nv;
  max_nu_ = max_nu;
  max_ndx_ = max_ndx;
  K_.setZero(max_nu, max_ndx);
  dx_.setZero(max_ndx);
  u_fb_raw_.setZero(max_nu);
  nu_ = 0;
  ndx_ = 0;
  // Fixed-base torque default: the accelerations are the first nv rows.
  acc_row_offset_ = 0;
  acc_rows_ = std::min(max_nv, max_nu);
}

void RiccatiFeedback::SetGain(const double* K_data, int nu, int ndx) noexcept {
  if (K_data == nullptr || nu <= 0 || ndx <= 0 || nu > max_nu_ || ndx > max_ndx_) {
    nu_ = 0;
    ndx_ = 0;
    return;
  }
  // K_data is row-major nu × ndx. Copy into the top-left nu × ndx block.
  for (int row = 0; row < nu; ++row) {
    for (int col = 0; col < ndx; ++col) {
      K_(row, col) = K_data[row * ndx + col];
    }
  }
  if (!K_.topLeftCorner(nu, ndx).allFinite()) {
    nu_ = 0;
    ndx_ = 0;
    return;
  }
  nu_ = nu;
  ndx_ = ndx;
}

bool RiccatiFeedback::SetAccelRows(int offset, int n) noexcept {
  if (offset < 0 || n < 0 || offset + n > max_nu_) {
    return false;
  }
  acc_row_offset_ = offset;
  acc_rows_ = n;
  return true;
}

void RiccatiFeedback::Compute(const Eigen::Ref<const Eigen::VectorXd>& q_curr,
                              const Eigen::Ref<const Eigen::VectorXd>& v_curr,
                              const Eigen::Ref<const Eigen::VectorXd>& q_ref,
                              const Eigen::Ref<const Eigen::VectorXd>& v_ref,
                              Eigen::Ref<Eigen::VectorXd> u_fb_out) noexcept {
  if (nu_ <= 0 || ndx_ <= 0) {
    u_fb_out.setZero();
    return;
  }

  const int nq = static_cast<int>(q_curr.size());
  const int nv = static_cast<int>(v_curr.size());
  // The Euclidean Δq is a tangent vector only when nq == nv; a configuration
  // with extra coordinates (continuous joint, free-flyer) must come through
  // ComputeFromDelta with a proper tangent difference.
  if (nq != q_ref.size() || nv != v_ref.size() || nq + nv != ndx_) {
    u_fb_out.setZero();
    return;
  }

  // Pack Δx = [Δq; Δv].
  dx_.head(nq) = q_curr.head(nq) - q_ref.head(nq);
  dx_.segment(nq, nv) = v_curr.head(nv) - v_ref.head(nv);
  (void)Apply(u_fb_out);
}

bool RiccatiFeedback::ComputeFromDelta(const Eigen::Ref<const Eigen::VectorXd>& dx,
                                       Eigen::Ref<Eigen::VectorXd> u_fb_out) noexcept {
  if (nu_ <= 0 || ndx_ <= 0 || dx.size() != ndx_) {
    u_fb_out.setZero();
    return false;
  }
  dx_.head(ndx_) = dx;
  return Apply(u_fb_out);
}

bool RiccatiFeedback::Apply(Eigen::Ref<Eigen::VectorXd> u_fb_out) noexcept {
  // Magnitude guard: linearly scale Δx down when its norm exceeds the cap.
  // A non-finite Δx would slip past the comparison, so it is refused here —
  // this is the last stage before the controller's acceleration reference.
  const double dx_norm = dx_.head(ndx_).norm();
  if (!std::isfinite(dx_norm)) {
    u_fb_out.setZero();
    return false;
  }
  double effective_scale = gain_scale_;
  if (dx_norm > max_delta_x_norm_) {
    effective_scale *= max_delta_x_norm_ / dx_norm;
  }

  // u_fb = scale * K * Δx  (noalias: avoid Eigen temporaries).
  u_fb_raw_.head(nu_).noalias() = effective_scale * K_.topLeftCorner(nu_, ndx_) * dx_.head(ndx_);

  const int out_size = static_cast<int>(u_fb_out.size());
  if (!accel_only_) {
    const int n_write = std::min(out_size, nu_);
    u_fb_out.head(n_write) = u_fb_raw_.head(n_write);
    if (out_size > n_write) {
      u_fb_out.tail(out_size - n_write).setZero();
    }
    return true;
  }

  // Accel-only: the acceleration rows of the gain output go to the front of
  // the caller's buffer; everything else is zero.
  const int available = std::max(0, nu_ - acc_row_offset_);
  const int n_write = std::min({acc_rows_, available, out_size});
  u_fb_out.head(n_write) = u_fb_raw_.segment(acc_row_offset_, n_write);
  if (out_size > n_write) {
    u_fb_out.tail(out_size - n_write).setZero();
  }
  return true;
}

void RiccatiFeedback::SetGainScale(double scale) noexcept {
  if (!std::isfinite(scale)) {
    return;  // std::clamp would latch a NaN scale forever
  }
  gain_scale_ = std::clamp(scale, 0.0, 1.0);
}

void RiccatiFeedback::SetMaxDeltaXNorm(double value) noexcept {
  if (value > 0.0 && std::isfinite(value)) {
    max_delta_x_norm_ = value;
  }
}

}  // namespace rtc::mpc
