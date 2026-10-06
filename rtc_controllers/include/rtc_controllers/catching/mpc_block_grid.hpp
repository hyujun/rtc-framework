// ── Move-blocked jerk grid shared by the segment cores ─────────────────────────
// (dynamic_catching E1-F01 #627, E1-F07 #660, E1-F13 #739)
//
// MpcSegmentCore and MpcDockingSegmentCore condense the same joint-space triple
// integrator onto the same kind of grid: N intervals split at a catch node
// (n_pre before it), with the jerk held constant over B consecutive blocks of
// intervals. What the two must agree on to the last bit lives here once — the
// rule a block partition has to satisfy, the interval → block map, and the
// stage gains — so that a trajectory one core produces is a trajectory the
// other (and the RT sampler behind both) reads the same way.
//
// Non-RT: called from Init().
#pragma once

#include "rtc_controllers/catching/trajectory.hpp"  // kMaxSegmentNodes

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rtc::catching {

/// Why a block partition is not usable.
enum class BlockGridCheck : std::uint8_t {
  kOk = 0,
  kInvalid,      ///< count out of range, an empty block, or Σ sizes ≠ N
  kTooFew,       ///< fewer than 3 blocks, or fewer than 3 after the catch node
  kAcrossCatch,  ///< a block spans the catch node
};

/// @brief Check a block partition of N intervals split at node `n_pre`.
///
/// The terminal equality (q̇_N = q̈_N = 0) takes two blocks after the catch
/// node; a third leaves the stop free — hence three there. No block may span
/// the catch node: the two sides have different interval lengths and different
/// costs. A rank check of the assembled terminal equality cannot stand in for
/// the second rule — pre-catch blocks supply rank too.
/// @param n_blocks    B (the first B entries of `block_sizes` are read)
/// @param block_sizes intervals per block
/// @param n_pre       intervals before the catch node (≥ 0)
/// @param n_nodes     N
[[nodiscard]] inline BlockGridCheck CheckBlockGrid(
    int n_blocks, const std::array<int, kMaxSegmentNodes>& block_sizes, int n_pre,
    int n_nodes) noexcept {
  if (n_blocks < 3) {
    return BlockGridCheck::kTooFew;
  }
  // block_sizes holds kMaxSegmentNodes entries whatever N is.
  if (n_blocks > n_nodes || n_blocks > kMaxSegmentNodes) {
    return BlockGridCheck::kInvalid;
  }
  int sum = 0;
  for (int b = 0; b < n_blocks; ++b) {
    const int size = block_sizes[static_cast<std::size_t>(b)];
    if (size < 1) {
      return BlockGridCheck::kInvalid;
    }
    sum += size;
  }
  if (sum != n_nodes) {
    return BlockGridCheck::kInvalid;
  }
  int start = 0;
  int after_catch = 0;
  for (int b = 0; b < n_blocks; ++b) {
    const int end = start + block_sizes[static_cast<std::size_t>(b)];
    if (start < n_pre && end > n_pre) {
      return BlockGridCheck::kAcrossCatch;
    }
    if (start >= n_pre) {
      ++after_catch;
    }
    start = end;
  }
  return after_catch < 3 ? BlockGridCheck::kTooFew : BlockGridCheck::kOk;
}

/// @brief Fill the interval → block map of a partition CheckBlockGrid accepted.
/// @param[out] block_of_interval entry k = the block interval [k, k+1] lies in;
///             at least Σ sizes entries
inline void FillBlockOfInterval(int n_blocks, const std::array<int, kMaxSegmentNodes>& block_sizes,
                                std::span<int> block_of_interval) noexcept {
  std::size_t k = 0;
  for (int b = 0; b < n_blocks; ++b) {
    const int size = block_sizes[static_cast<std::size_t>(b)];
    for (int i = 0; i < size && k < block_of_interval.size(); ++i) {
      block_of_interval[k++] = b;
    }
  }
}

/// @brief Stage gains ĝ_{m,k}[b] of the scalar triple integrator: its response
///        (q, q̇, q̈ for m = 0, 1, 2) at node k to the jerk `u_scale` held over
///        block b, all other blocks zero, from rest.
/// @param dt_interval       Δ_k of interval [k, k+1], N entries
/// @param block_of_interval the block of each interval, N entries
/// @param[out] gq, gv, ga   (N+1) × B; row 0 is zero
inline void ComputeBlockStageGains(std::span<const double> dt_interval,
                                   std::span<const int> block_of_interval, int n_blocks,
                                   double u_scale, Eigen::MatrixXd& gq, Eigen::MatrixXd& gv,
                                   Eigen::MatrixXd& ga) {
  const auto n_nodes = static_cast<Eigen::Index>(dt_interval.size());
  const auto nb = static_cast<Eigen::Index>(n_blocks);
  gq.setZero(n_nodes + 1, nb);
  gv.setZero(n_nodes + 1, nb);
  ga.setZero(n_nodes + 1, nb);
  for (Eigen::Index b = 0; b < nb; ++b) {
    double q = 0.0;
    double v = 0.0;
    double a = 0.0;
    for (Eigen::Index k = 0; k < n_nodes; ++k) {
      const auto ki = static_cast<std::size_t>(k);
      const double dt = dt_interval[ki];
      const double u = block_of_interval[ki] == static_cast<int>(b) ? u_scale : 0.0;
      q += dt * v + 0.5 * dt * dt * a + dt * dt * dt * u / 6.0;
      v += dt * a + 0.5 * dt * dt * u;
      a += dt * u;
      gq(k + 1, b) = q;
      gv(k + 1, b) = v;
      ga(k + 1, b) = a;
    }
  }
}

}  // namespace rtc::catching
