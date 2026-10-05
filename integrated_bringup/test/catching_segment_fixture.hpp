#pragma once

// ── Hand-built APPROACH–stop segments (MPC E1-F04 · E1-F09 tests) ───────────
//
// The suites that play the decel planner (the oracle profile has no planner
// thread, so the test is the decel box's one writer) need segments the RT can
// follow from the command it holds. This builds one in closed form on the
// two-spacing grid (MD-54): `n_pre` pre-catch intervals of Δ_pre before t_c,
// then kApproachNStop stop intervals of Δ_s. Node 0 is placed so that the
// segment's state at a sample instant s inside its first interval is EXACTLY
// (q_c, q̇_c, 0) — that interval has zero jerk, so the sampler's cubic from
// node 0 reproduces it to rounding. A FIRST segment starts at rest (q̇_c = 0),
// which is what a planner that solved from the arm at its wait pose publishes;
// a replan built from a moving command is what a planner with a perfect
// initial-state prediction would publish.
//
// From there every joint takes a velocity step of `bump` rad/s over the two
// pre-catch intervals before the last one, cruises through the last, and stops
// over the first two stop intervals — so the arm is MOVING at t_c — all with
// piecewise-constant jerk, i.e. the nodes are CONSISTENT and the sampler is
// exact between them (jerk_segment.hpp).

#include "rtc_controllers/catching/trajectory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace integrated_bringup::testfx {

inline constexpr std::int64_t kApproachDtPreNs = 100'000'000;  // Δ_pre (MD-54)
inline constexpr std::int64_t kApproachDtNs = 50'000'000;      // Δ_s
inline constexpr int kApproachNPre = 4;                        // pre-catch intervals, first segment
inline constexpr int kApproachNStop = 7;                       // stop intervals (0.35 s)

/// Piecewise-constant jerk of interval k (0 .. n_pre + kApproachNStop − 1) of a
/// segment with `n_pre` pre-catch intervals, for a joint that passes t_c with
/// velocity `v_c` after a step of `bump`: +bump over the third- and
/// second-to-last pre-catch intervals (triangle acceleration, zero at both
/// ends), none on the others, then a triangle-acceleration stop of v_c over
/// stop intervals 0 – 1.
inline double ApproachJerk(int k, int n_pre, double v_c, double bump, double dt_pre, double dt) {
  if (k < n_pre) {
    const int from_end = n_pre - 1 - k;
    if (from_end == 2) {
      return bump / (dt_pre * dt_pre);
    }
    if (from_end == 1) {
      return -bump / (dt_pre * dt_pre);
    }
    return 0.0;
  }
  const int m = k - n_pre;
  if (m == 0) {
    return -v_c / (dt * dt);
  }
  if (m == 1) {
    return v_c / (dt * dt);
  }
  return 0.0;
}

/// An APPROACH–stop segment for plan (`plan_id`, `t_c_ns`) with `n_pre`
/// pre-catch intervals — node 0 at t_c − n_pre·Δ_pre — whose state at `s_ns`
/// (0 ≤ s − t0 ≤ Δ_pre) is (q_c, q̇_c, 0). The velocity step needs three
/// pre-catch intervals after the first (n_pre ≥ 4); with fewer the joint
/// cruises at q̇_c to t_c and stops. Device order; `n` joints. Stamps
/// (`publish_ns`, `rt_state_ns`, `segment_seq`, the token) are the caller's.
template <std::size_t N>
rtc::catching::SegmentSnapshot MakeApproachSegment(const std::array<double, N>& q_c,
                                                   const std::array<double, N>& qd_c, int n,
                                                   std::uint32_t plan_id, std::int64_t t_c_ns,
                                                   int n_pre, std::int64_t s_ns, double bump) {
  rtc::catching::SegmentSnapshot seg{};
  seg.valid = true;
  seg.plan_id = plan_id;
  seg.t_c_ns = t_c_ns;
  seg.k0 = 0;
  seg.dt_ns = kApproachDtNs;
  seg.dt_pre_ns = kApproachDtPreNs;
  seg.n_pre = n_pre;
  seg.t0_ns = t_c_ns - static_cast<std::int64_t>(n_pre) * kApproachDtPreNs;
  seg.n_nodes = n_pre + kApproachNStop;
  seg.nv = n;
  const double dt_pre = static_cast<double>(kApproachDtPreNs) * 1e-9;
  const double dt = static_cast<double>(kApproachDtNs) * 1e-9;
  const double tau = static_cast<double>(s_ns - seg.t0_ns) * 1e-9;
  const bool stepped = n_pre >= 4;
  for (int j = 0; j < n; ++j) {
    const auto u = static_cast<std::size_t>(j);
    // Alternating sign, so the step is not a pure scaling of one direction.
    const double b = stepped ? (j % 2 == 0 ? 1.0 : -1.0) * bump : 0.0;
    const double v_c = qd_c[u] + b;
    double q = q_c[u] - tau * qd_c[u];
    double v = qd_c[u];
    double a = 0.0;
    for (int k = 0; k <= seg.n_nodes; ++k) {
      const auto e = static_cast<std::size_t>(k * rtc::catching::kMaxSegmentNv + j);
      seg.q[e] = q;
      seg.qd[e] = v;
      seg.qdd[e] = a;
      if (k == seg.n_nodes) {
        break;
      }
      const double h = k < n_pre ? dt_pre : dt;
      const double jerk = ApproachJerk(k, n_pre, v_c, b, dt_pre, dt);
      q += v * h + 0.5 * a * h * h + jerk * h * h * h / 6.0;
      v += a * h + 0.5 * jerk * h * h;
      a += jerk * h;
    }
  }
  return seg;
}

/// The same trajectory from its node `shift` on: nodes shift.. of `seg` as
/// nodes 0.., the replan a planner that predicted exactly would publish. A
/// shift inside the pre-catch part keeps the remaining pre-catch intervals; one
/// past the catch node is a stop-only segment at grid point k0 = shift − n_pre.
inline rtc::catching::SegmentSnapshot ShiftSegment(const rtc::catching::SegmentSnapshot& seg,
                                                   int shift) {
  rtc::catching::SegmentSnapshot out = seg;
  out.n_nodes = seg.n_nodes - shift;
  if (shift <= seg.n_pre) {
    out.n_pre = seg.n_pre - shift;
    out.k0 = seg.k0;
    out.t0_ns = seg.t0_ns + static_cast<std::int64_t>(shift) * seg.dt_pre_ns;
  } else {
    out.n_pre = 0;
    out.k0 = seg.k0 + shift - seg.n_pre;
    out.t0_ns = seg.t_c_ns + static_cast<std::int64_t>(out.k0) * seg.dt_ns;
  }
  out.q.fill(0.0);
  out.qd.fill(0.0);
  out.qdd.fill(0.0);
  for (int k = 0; k <= out.n_nodes; ++k) {
    for (int j = 0; j < seg.nv; ++j) {
      const auto dst = static_cast<std::size_t>(k * rtc::catching::kMaxSegmentNv + j);
      const auto src = static_cast<std::size_t>((k + shift) * rtc::catching::kMaxSegmentNv + j);
      out.q[dst] = seg.q[src];
      out.qd[dst] = seg.qd[src];
      out.qdd[dst] = seg.qdd[src];
    }
  }
  return out;
}

}  // namespace integrated_bringup::testfx
