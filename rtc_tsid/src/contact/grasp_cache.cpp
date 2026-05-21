#include "rtc_tsid/contact/grasp_cache.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rtc::tsid {

void GraspCache::Init(int max_contact_vars) noexcept {
  max_lambda_ = max_contact_vars;
  n_lambda_ = 0;
  rank_g_ = 0;

  // Worst-case allocation. All Compute() calls write into top-left blocks
  // sized to the current n_lambda_, leaving the rest untouched.
  g_pinv_.setZero(max_lambda_, 6);
  gt_pinv_.setZero(6, max_lambda_);
  p_n_.setZero(max_lambda_, max_lambda_);

  m6_.setZero();
  minv6_.setZero();
  nlam_.setZero(max_lambda_, max_lambda_);
  ninv_.setZero(max_lambda_, max_lambda_);
  minv_g_.setZero(6, max_lambda_);

  // Pre-allocate LDLT storage for the n_λ × n_λ path (worst case).
  ldlt_n_ = Eigen::LDLT<Eigen::MatrixXd>(max_lambda_);
}

void GraspCache::Compute(const Eigen::Ref<const Eigen::MatrixXd>& G, int n_lambda_active) noexcept {
  n_lambda_ = n_lambda_active;
  rank_g_ = 0;

  if (n_lambda_ == 0) {
    return;
  }

  // Path A: n_λ ≥ 6 — solve via M = G·Gᵀ (6×6 stack).
  // Path B: n_λ < 6 — solve via N = Gᵀ·G (n_λ × n_λ, ≤ 5×5).
  //
  // Both produce the damped Moore–Penrose pinv. solveInPlace into
  // pre-allocated workspace keeps heap traffic at zero.
  if (n_lambda_ >= 6) {
    // G is 6 × n_λ.
    m6_.noalias() = G * G.transpose();
    const double trace = m6_.diagonal().sum();
    const double eps = std::max(tol_damp_, std::numeric_limits<double>::epsilon() * trace);
    m6_.diagonal().array() += eps;

    ldlt_6_.compute(m6_);

    // Rank counted by comparing post-damping pivots to the damping floor;
    // a "true zero" eigenvalue manifests as a pivot ≈ ε.
    const auto& D6 = ldlt_6_.vectorD();
    for (int i = 0; i < 6; ++i) {
      if (std::abs(D6(i) - eps) > tol_pivot_) {
        ++rank_g_;
      }
    }

    // minv6_ = M⁻¹ via solveInPlace on a workspace seeded with I_6.
    minv6_.setIdentity();
    ldlt_6_.solveInPlace(minv6_);

    // minv_g_ = M⁻¹ · G  → 6 × n_λ. Reuse the workspace top-left slice.
    auto minv_g = minv_g_.topLeftCorner(6, n_lambda_);
    minv_g.noalias() = minv6_ * G;

    // G_pinv = Gᵀ · M⁻¹  → n_λ × 6
    g_pinv_.topLeftCorner(n_lambda_, 6).noalias() = G.transpose() * minv6_;

    // GT_pinv = M⁻¹ · G  → 6 × n_λ
    gt_pinv_.topLeftCorner(6, n_lambda_) = minv_g;

    // P_N = I - G_pinv · G  → n_λ × n_λ
    auto P = p_n_.topLeftCorner(n_lambda_, n_lambda_);
    P.noalias() = -g_pinv_.topLeftCorner(n_lambda_, 6) * G;
    P.diagonal().array() += 1.0;
  } else {
    // n_λ < 6: use the n_λ × n_λ normal equations.
    auto N = nlam_.topLeftCorner(n_lambda_, n_lambda_);
    N.noalias() = G.transpose() * G;
    const double trace = N.diagonal().sum();
    const double eps = std::max(tol_damp_, std::numeric_limits<double>::epsilon() * trace);
    N.diagonal().array() += eps;

    ldlt_n_.compute(N);

    const auto& Dn = ldlt_n_.vectorD();
    for (int i = 0; i < n_lambda_; ++i) {
      if (std::abs(Dn(i) - eps) > tol_pivot_) {
        ++rank_g_;
      }
    }

    // ninv_ = N⁻¹ via solveInPlace on a workspace seeded with I_{n_λ}.
    auto Ninv = ninv_.topLeftCorner(n_lambda_, n_lambda_);
    Ninv.setIdentity();
    ldlt_n_.solveInPlace(Ninv);

    // G_pinv = N⁻¹ · Gᵀ  → n_λ × 6
    g_pinv_.topLeftCorner(n_lambda_, 6).noalias() = Ninv * G.transpose();

    // GT_pinv = G · N⁻¹  → 6 × n_λ
    gt_pinv_.topLeftCorner(6, n_lambda_).noalias() = G * Ninv;

    // P_N = I - G_pinv · G  → n_λ × n_λ
    auto P = p_n_.topLeftCorner(n_lambda_, n_lambda_);
    P.noalias() = -g_pinv_.topLeftCorner(n_lambda_, 6) * G;
    P.diagonal().array() += 1.0;
  }
}

}  // namespace rtc::tsid
