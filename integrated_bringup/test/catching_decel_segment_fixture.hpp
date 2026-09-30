#pragma once

// ── Hand-built decel MPC stop segments (MPC E1-F04 tests) ───────────────────
//
// The suites that play the decel planner (the oracle profile has no planner
// thread, so the test is the decel box's one writer) need a segment the RT
// can follow from the command it holds on the DECEL entry tick. This builds
// one in closed form: node 0 is placed so that the segment's state at the
// entry tick's sample instant s is EXACTLY (q_c, q̇_c, 0) — the first interval
// has zero jerk, so the sampler's cubic from node 0 reproduces it to rounding
// — then every joint takes a velocity bump of `bump` rad/s (so the follow has
// something to measure) and comes to rest by node N with zero acceleration,
// all with piecewise-constant jerk, i.e. the nodes are CONSISTENT and the
// sampler is exact between them (jerk_segment.hpp).

#include "rtc_controllers/catching/trajectory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace integrated_bringup::testfx {

inline constexpr std::int64_t kDecelFollowDtNs = 25'000'000;  // Δ_s (MD-24)
inline constexpr int kDecelFollowNodes = 15;                  // the bump needs 14 + 1 intervals

/// Piecewise-constant jerk of interval k (0 .. kDecelFollowNodes − 1) for a
/// joint entering with velocity `v0`: none on interval 0, a +bump / −bump
/// velocity bump over intervals 1 – 8, then a triangle-acceleration stop of v0
/// over intervals 9 – 14 (each phase ends at zero acceleration).
inline double FollowJerk(int k, double v0, double bump, double dt) {
  const double j1 = bump / (4.0 * dt * dt);
  const double j2 = v0 / (9.0 * dt * dt);
  if (k == 0) {
    return 0.0;
  }
  if (k <= 2 || (k >= 7 && k <= 8)) {
    return j1;
  }
  if (k <= 6) {
    return -j1;
  }
  if (k <= 11) {
    return -j2;
  }
  return j2;
}

/// A stop segment for plan (`plan_id`, `t_c_ns`) whose node 0 sits at
/// t_c + k0·Δ and whose state at `s_ns` (0 ≤ s − t0 ≤ Δ) is (q_c, q̇_c, 0).
/// Device order; `n` joints. Stamps (`publish_ns`, `rt_state_ns`, `decel_seq`,
/// the token) are the caller's.
template <std::size_t N>
rtc::catching::DecelPlanSnapshot MakeFollowSegment(const std::array<double, N>& q_c,
                                                   const std::array<double, N>& qd_c, int n,
                                                   std::uint32_t plan_id, std::int64_t t_c_ns,
                                                   int k0, std::int64_t s_ns, double bump) {
  rtc::catching::DecelPlanSnapshot seg{};
  seg.valid = true;
  seg.plan_id = plan_id;
  seg.t_c_ns = t_c_ns;
  seg.k0 = k0;
  seg.dt_ns = kDecelFollowDtNs;
  seg.t0_ns = t_c_ns + static_cast<std::int64_t>(k0) * kDecelFollowDtNs;
  seg.n_nodes = kDecelFollowNodes;
  seg.nv = n;
  const double dt = static_cast<double>(kDecelFollowDtNs) * 1e-9;
  const double tau = static_cast<double>(s_ns - seg.t0_ns) * 1e-9;
  for (int j = 0; j < n; ++j) {
    const auto u = static_cast<std::size_t>(j);
    // Alternating sign, so the bump is not a pure scaling of one direction.
    const double b = (j % 2 == 0 ? 1.0 : -1.0) * bump;
    double q = q_c[u] - tau * qd_c[u];
    double v = qd_c[u];
    double a = 0.0;
    for (int k = 0; k <= kDecelFollowNodes; ++k) {
      const auto e = static_cast<std::size_t>(k * rtc::catching::kMaxDecelNv + j);
      seg.q[e] = q;
      seg.qd[e] = v;
      seg.qdd[e] = a;
      if (k == kDecelFollowNodes) {
        break;
      }
      const double jerk = FollowJerk(k, qd_c[u], b, dt);
      q += v * dt + 0.5 * a * dt * dt + jerk * dt * dt * dt / 6.0;
      v += a * dt + 0.5 * jerk * dt * dt;
      a += jerk * dt;
    }
  }
  return seg;
}

/// The same trajectory from grid point k0 + shift on: nodes shift.. of `seg`
/// as nodes 0.., the replan a planner that predicted exactly would publish.
inline rtc::catching::DecelPlanSnapshot ShiftSegment(const rtc::catching::DecelPlanSnapshot& seg,
                                                     int shift) {
  rtc::catching::DecelPlanSnapshot out = seg;
  out.k0 = seg.k0 + shift;
  out.t0_ns = seg.t0_ns + static_cast<std::int64_t>(shift) * seg.dt_ns;
  out.n_nodes = seg.n_nodes - shift;
  out.q.fill(0.0);
  out.qd.fill(0.0);
  out.qdd.fill(0.0);
  for (int k = 0; k <= out.n_nodes; ++k) {
    for (int j = 0; j < seg.nv; ++j) {
      const auto dst = static_cast<std::size_t>(k * rtc::catching::kMaxDecelNv + j);
      const auto src = static_cast<std::size_t>((k + shift) * rtc::catching::kMaxDecelNv + j);
      out.q[dst] = seg.q[src];
      out.qd[dst] = seg.qd[src];
      out.qdd[dst] = seg.qdd[src];
    }
  }
  return out;
}

}  // namespace integrated_bringup::testfx
