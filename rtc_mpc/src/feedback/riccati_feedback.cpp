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

void RiccatiFeedback::UseGain(const double* K_data, int nu, int ndx) noexcept {
  if (K_data == nullptr || nu <= 0 || ndx <= 0 || nu > max_nu_ || ndx > max_ndx_) {
    view_ = nullptr;
    nu_ = 0;
    ndx_ = 0;
    return;
  }
  view_ = K_data;
  nu_ = nu;
  ndx_ = ndx;
}

bool RiccatiFeedback::SetBlockCaps(const RiccatiBlockCaps& caps, int nv, bool floating) noexcept {
  for (const double c : caps) {
    if (!(c > 0.0) || !std::isfinite(c)) {
      return false;
    }
  }
  const int nb = floating ? 3 : 0;
  if (nv <= 2 * nb || 2 * nv > max_ndx_) {
    return false;
  }
  caps_ = caps;
  floating_ = floating;
  block_start_ = {0, nb, 2 * nb, nv, nv + nb, nv + 2 * nb, 2 * nv};
  block_caps_ = true;
  return true;
}

void RiccatiFeedback::SetGain(const double* K_data, int nu, int ndx) noexcept {
  view_ = nullptr;
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

double RiccatiFeedback::CapScale() const noexcept {
  const double dx_norm = dx_.head(ndx_).norm();
  if (!std::isfinite(dx_norm)) {
    return -1.0;
  }
  if (!block_caps_ || block_start_[kRiccatiBlockCount] != ndx_) {
    return dx_norm > max_delta_x_norm_ ? max_delta_x_norm_ / dx_norm : 1.0;
  }
  double s = 1.0;
  for (int b = 0; b < kRiccatiBlockCount; ++b) {
    const auto bb = static_cast<std::size_t>(b);
    const int start = block_start_[bb];
    const int n = block_start_[bb + 1] - start;
    if (n <= 0) {
      continue;
    }
    const double nb = dx_.segment(start, n).norm();
    if (nb > caps_[bb]) {
      s = std::min(s, caps_[bb] / nb);
    }
  }
  if (floating_) {
    // The rotation block is a log map: its norm is at most π, where its
    // axis flips.
    constexpr double kPi = 3.14159265358979323846;
    const auto rot = static_cast<std::size_t>(RiccatiBlock::kBaseOrientation);
    const double theta = dx_.segment(block_start_[rot], 3).norm();
    if (theta > kPi - kRotationFadeRad) {
      s *= std::max(0.0, (kPi - theta) / kRotationFadeRad);
    }
  }
  return s;
}

bool RiccatiFeedback::Apply(Eigen::Ref<Eigen::VectorXd> u_fb_out) noexcept {
  // Magnitude guard. A non-finite Δx would slip past the comparisons, so it
  // is refused here — this is the last stage before the controller's
  // acceleration reference.
  const double cap = CapScale();
  if (cap < 0.0) {
    u_fb_out.setZero();
    last_scale_ = 0.0;
    return false;
  }
  const double effective_scale = gain_scale_ * cap;
  last_scale_ = effective_scale;

  // Only the rows written out are multiplied: u = (K_rows · Δx) · scale.
  const int out_size = static_cast<int>(u_fb_out.size());
  const int row0 = accel_only_ ? acc_row_offset_ : 0;
  const int rows = accel_only_ ? std::min(acc_rows_, std::max(0, nu_ - acc_row_offset_)) : nu_;
  const int n_write = std::min(rows, out_size);
  if (n_write > 0) {
    if (view_ != nullptr) {
      using RowMajorConstMap =
          Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
      const RowMajorConstMap K(view_, nu_, ndx_);
      u_fb_raw_.head(n_write).noalias() = K.middleRows(row0, n_write) * dx_.head(ndx_);
    } else {
      u_fb_raw_.head(n_write).noalias() = K_.block(row0, 0, n_write, ndx_) * dx_.head(ndx_);
    }
    u_fb_raw_.head(n_write) *= effective_scale;
    u_fb_out.head(n_write) = u_fb_raw_.head(n_write);
  }
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
