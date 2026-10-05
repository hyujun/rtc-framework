// ── Inputs of the planner's search, shared by its suites (test-only) ─────────
// test_catching_grid_catch_search.cpp drives GridCatchSearch alone; the APPROACH
// cycle suite (test_catching_approach_cycle.cpp, E1-F08 #661) drives the same
// search through PlannerCycle with the decel planner behind it. Both build the
// search's three inputs — a ball trajectory through a catch point, its
// covariance, the RT state of a TRACKING arm — and must build them the same
// way, or a cycle failure could be an input the search never saw in its own
// suite. That is why they live here once (design-principles P5, as
// catch_arm_fixture.hpp records for the arms).
//
// Test-only, NEVER installed: ament's symlink install ignores
// install(PATTERN EXCLUDE), so a header placed under include/ would ship.
#pragma once

#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <span>

namespace rtc::testing {

/// A constant-velocity ball: `n` samples `spacing_ns` apart from `first_ns`,
/// sample `at` passing through `p_c`. Token: `activation`, track `gen`,
/// sequence `seq`, received at `recv_ns`.
[[nodiscard]] inline catching::TrajectorySnapshot LineTrajectory(
    const Eigen::Vector3d& p_c, const Eigen::Vector3d& v_ball, std::int64_t first_ns,
    std::int64_t spacing_ns, int n, int at, std::uint64_t seq, std::uint64_t gen,
    std::uint64_t activation, std::int64_t recv_ns) {
  catching::TrajectorySnapshot t{};
  t.valid = true;
  t.n = n;
  t.token.activation_generation = activation;
  t.token.generation = gen;
  t.token.snapshot_sequence = seq;
  t.token.traj_recv_ns = recv_ns;
  // Divided, not multiplied by 1e-9: the quotient is correctly rounded, so
  // 50 ms gives exactly the literal 0.05 the suites were written with.
  const double spacing_s = static_cast<double>(spacing_ns) / 1e9;
  for (int k = 0; k < t.n; ++k) {
    auto& s = t.s[static_cast<std::size_t>(k)];
    s.t_ns = first_ns + k * spacing_ns;
    const double dt = (k - at) * spacing_s;
    const Eigen::Vector3d p = p_c + v_ball * dt;
    s.p = {p.x(), p.y(), p.z()};
    s.v = {v_ball.x(), v_ball.y(), v_ball.z()};
  }
  return t;
}

/// A covariance matched to `t` (same token and count), isotropic σ on every
/// position and velocity axis of every sample.
[[nodiscard]] inline catching::CovarianceSnapshot IsotropicCovariance(
    const catching::TrajectorySnapshot& t, double sigma) {
  catching::CovarianceSnapshot c{};
  c.valid = true;
  c.n = t.n;
  c.token = t.token;
  for (int k = 0; k < t.n; ++k) {
    auto& e = c.c[static_cast<std::size_t>(k)];
    e.fill(0.0);
    for (int d = 0; d < 6; ++d) {
      e[static_cast<std::size_t>(d * 6 + d)] = sigma * sigma;
    }
  }
  return c;
}

/// The RT state of a TRACKING arm: valid, seeded, at rest at `q_cmd` (DEVICE
/// order, one entry per joint), following no plan.
[[nodiscard]] inline catching::PlannerRtState TrackingRtState(std::uint64_t activation,
                                                              std::span<const double> q_cmd) {
  catching::PlannerRtState rt{};
  rt.valid = true;
  rt.activation_generation = activation;
  rt.mode = static_cast<std::uint8_t>(catching::Mode::kTracking);
  rt.nv = static_cast<std::int32_t>(q_cmd.size());
  rt.cmd_seeded = true;
  for (std::size_t j = 0; j < q_cmd.size(); ++j) {
    rt.q_cmd[j] = q_cmd[j];
  }
  return rt;
}

}  // namespace rtc::testing
