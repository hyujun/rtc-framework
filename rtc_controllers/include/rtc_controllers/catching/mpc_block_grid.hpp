// ── Move-blocked jerk grid shared by the segment cores ─────────────────────────
//
// MpcSegmentCore and MpcDockingSegmentCore condense the same joint-space triple
// integrator onto the same kind of grid: N intervals split at a catch node
// (n_pre before it), with the jerk held constant over B consecutive blocks of
// intervals. What the two must agree on to the last bit lives here once — the
// rule a block partition has to satisfy, the interval → block map, and the
// stage gains — so that a trajectory one core produces is a trajectory the
// other (and the RT sampler behind both) reads the same way.
//
// Per joint group each group may have its own partition. Groups whose
// partitions are the same share one BLOCK CLASS; BlockLayout says which class a
// joint is in and where each (joint, block) jerk sits in the variable vector.
// With one class of every joint the layout is z_{b·n+j} (block-major).
//
// Non-RT: called from Init().
#pragma once

#include "rtc_controllers/catching/joint_groups.hpp"  // kMaxJointGroups
#include "rtc_controllers/catching/trajectory.hpp"    // kMaxSegmentNodes, kMaxPlanNv

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
///        block b, all other blocks zero, from rest. Exact for piecewise-constant
///        jerk u_k on [k, k+1] (u_k = u_scale if interval k is in block b, else 0):
///          q_{k+1} = q_k + Δ_k q̇_k + Δ_k²/2 q̈_k + Δ_k³/6 u_k
///          q̇_{k+1} = q̇_k + Δ_k q̈_k + Δ_k²/2 u_k
///          q̈_{k+1} = q̈_k + Δ_k u_k
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

/// One joint group's partition: the first n_blocks entries of block_sizes.
struct BlockPattern {
  int n_blocks{0};
  std::array<int, kMaxSegmentNodes> block_sizes{};

  [[nodiscard]] bool operator==(const BlockPattern& o) const noexcept {
    if (n_blocks != o.n_blocks) {
      return false;
    }
    for (int b = 0; b < n_blocks && b < kMaxSegmentNodes; ++b) {
      if (block_sizes[static_cast<std::size_t>(b)] != o.block_sizes[static_cast<std::size_t>(b)]) {
        return false;
      }
    }
    return true;
  }
};

/// Joints by block class and the variable column of each (joint, block).
/// Classes are ordered by their lowest joint index; within a class the
/// variables are block-major over its joints in model order:
///   column(j, b) = offset[c] + b·n_joints[c] + rank[j],  c = class_of[j].
struct BlockLayout {
  int nv{0};
  int n_classes{0};
  int nu{0};  ///< Σ_c n_joints[c]·pattern[c].n_blocks
  std::array<BlockPattern, kMaxJointGroups> pattern{};
  std::array<int, kMaxJointGroups> n_joints{};
  std::array<int, kMaxJointGroups> offset{};
  std::array<std::array<int, kMaxPlanNv>, kMaxJointGroups> joints{};  ///< ascending
  std::array<std::int8_t, kMaxPlanNv> class_of{};
  std::array<std::int8_t, kMaxPlanNv> rank{};

  [[nodiscard]] int Col(int j, int b) const noexcept {
    const auto c = static_cast<std::size_t>(class_of[static_cast<std::size_t>(j)]);
    return offset[c] + b * n_joints[c] + rank[static_cast<std::size_t>(j)];
  }

  /// One class holding every joint: the layout z_{b·n+j}.
  [[nodiscard]] bool Single() const noexcept { return n_classes == 1; }
};

/// @brief Build the layout of nv joints from their groups' partitions.
///
/// Each partition a joint uses is checked with CheckBlockGrid; a group no
/// joint is in is not read. With n_groups ≤ 1 every joint is in group 0.
/// @param group_of_joint model joint j's group (nv entries, read with n_groups ≥ 2)
/// @param patterns       group g's partition (n_groups entries; 1 with n_groups ≤ 1)
/// @return the first failed check (out untouched then), else kOk
[[nodiscard]] inline BlockGridCheck BuildBlockLayout(int nv, int n_groups,
                                                     std::span<const std::int8_t> group_of_joint,
                                                     std::span<const BlockPattern> patterns,
                                                     int n_pre, int n_nodes,
                                                     BlockLayout& out) noexcept {
  const int groups = n_groups <= 1 ? 1 : n_groups;
  if (nv < 1 || nv > kMaxPlanNv || groups > kMaxJointGroups ||
      patterns.size() < static_cast<std::size_t>(groups) ||
      (groups > 1 && group_of_joint.size() < static_cast<std::size_t>(nv))) {
    return BlockGridCheck::kInvalid;
  }
  BlockLayout l;
  l.nv = nv;
  std::array<int, kMaxJointGroups> class_of_group{};
  class_of_group.fill(-1);
  for (int j = 0; j < nv; ++j) {
    const int g = groups > 1 ? group_of_joint[static_cast<std::size_t>(j)] : 0;
    if (g < 0 || g >= groups) {
      return BlockGridCheck::kInvalid;
    }
    int& c = class_of_group[static_cast<std::size_t>(g)];
    if (c < 0) {
      const BlockPattern& p = patterns[static_cast<std::size_t>(g)];
      for (int other = 0; other < l.n_classes; ++other) {
        if (l.pattern[static_cast<std::size_t>(other)] == p) {
          c = other;
          break;
        }
      }
      if (c < 0) {
        const BlockGridCheck check = CheckBlockGrid(p.n_blocks, p.block_sizes, n_pre, n_nodes);
        if (check != BlockGridCheck::kOk) {
          return check;
        }
        c = l.n_classes++;
        l.pattern[static_cast<std::size_t>(c)] = p;
      }
    }
    const auto cu = static_cast<std::size_t>(c);
    l.class_of[static_cast<std::size_t>(j)] = static_cast<std::int8_t>(c);
    l.rank[static_cast<std::size_t>(j)] = static_cast<std::int8_t>(l.n_joints[cu]);
    l.joints[cu][static_cast<std::size_t>(l.n_joints[cu]++)] = j;
  }
  for (int c = 0; c < l.n_classes; ++c) {
    const auto cu = static_cast<std::size_t>(c);
    l.offset[cu] = l.nu;
    l.nu += l.n_joints[cu] * l.pattern[cu].n_blocks;
  }
  out = l;
  return BlockGridCheck::kOk;
}

}  // namespace rtc::catching
