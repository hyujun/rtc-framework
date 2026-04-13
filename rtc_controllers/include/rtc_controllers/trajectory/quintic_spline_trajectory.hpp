#ifndef RTC_CONTROLLERS_TRAJECTORY_QUINTIC_SPLINE_TRAJECTORY_HPP_
#define RTC_CONTROLLERS_TRAJECTORY_QUINTIC_SPLINE_TRAJECTORY_HPP_

#include "rtc_controllers/trajectory/trajectory_utils.hpp"
#include "rtc_controllers/trajectory/quintic_blend_trajectory.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include <array>
#include <cstddef>
#include <algorithm>

namespace rtc
{
namespace trajectory
{

/// Multi-waypoint quintic spline trajectory with global C4 continuity.
/// Solves a linear system per DOF to determine internal velocities and accelerations
/// such that position, velocity, acceleration, jerk, and snap are continuous at
/// all internal knots. Boundary conditions: natural (vel=0, acc=0) or clamped.
template<std::size_t DOF>
class QuinticSplineTrajectory {
  static_assert(DOF > 0, "QuinticSplineTrajectory: DOF must be at least 1");

public:
  using Waypoint = typename QuinticBlendTrajectory<DOF>::Waypoint;

  struct State
  {
    std::array<double, DOF> positions{};
    std::array<double, DOF> velocities{};
    std::array<double, DOF> accelerations{};
  };

  QuinticSplineTrajectory() = default;

  /// Natural spline: start/end vel=0, acc=0 (non-RT path).
  void initialize(
    const std::array<Waypoint, kMaxWaypoints> & waypoints,
    std::size_t num_waypoints) noexcept
  {
    std::array<double, DOF> zero{};
    initialize(waypoints, num_waypoints, zero, zero, zero, zero);
  }

  /// Clamped spline: user-specified start/end vel and acc (non-RT path).
  void initialize(
    const std::array<Waypoint, kMaxWaypoints> & waypoints,
    std::size_t num_waypoints,
    const std::array<double, DOF> & start_vel,
    const std::array<double, DOF> & start_acc,
    const std::array<double, DOF> & end_vel,
    const std::array<double, DOF> & end_acc) noexcept
  {
    if (num_waypoints < 2) {
      num_segments_ = 0;
      duration_ = 0.0;
      if (num_waypoints == 1) {
        hold_position_ = waypoints[0].positions;
      } else {
        hold_position_ = {};
      }
      return;
    }

    const std::size_t n = std::min(num_waypoints, kMaxWaypoints);
    num_segments_ = n - 1;
    duration_ = waypoints[n - 1].time;

    for (std::size_t i = 0; i < n; ++i) {
      segment_start_times_[i] = waypoints[i].time;
    }

    // Special case: 2 waypoints = single segment, direct quintic
    if (n == 2) {
      const double h = waypoints[1].time - waypoints[0].time;
      for (std::size_t j = 0; j < DOF; ++j) {
        segments_[0][j].compute_coefficients(
          waypoints[0].positions[j], start_vel[j], start_acc[j],
          waypoints[1].positions[j], end_vel[j], end_acc[j],
          h);
      }
      return;
    }

    // 6*(N-1) unknowns (6 coefficients per segment).
    // Conditions: N position interpolation, 4 boundary (start/end vel & acc),
    // 5*(N-2) internal continuity (pos, vel, acc, jerk, snap).
    // Total: N + 4 + 5*(N-2) = 6N - 6 = 6*(N-1). Square system.

    const auto ns = num_segments_;
    const auto sys_size = static_cast<Eigen::Index>(6 * ns);

    // Build coefficient matrix (same for all DOF) and RHS per DOF
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(sys_size, sys_size);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(sys_size, static_cast<Eigen::Index>(DOF));

    // Segment durations
    std::array<double, kMaxWaypoints - 1> h{};
    for (std::size_t i = 0; i < ns; ++i) {
      h[i] = waypoints[i + 1].time - waypoints[i].time;
    }

    Eigen::Index row = 0;

    // --- Position at start of each segment: p_k(0) = wp[k].pos ---
    // p_k(0) = c0_k
    for (std::size_t k = 0; k < ns; ++k) {
      const auto base = static_cast<Eigen::Index>(6 * k);
      A(row, base) = 1.0;  // c0
      for (std::size_t j = 0; j < DOF; ++j) {
        B(row, static_cast<Eigen::Index>(j)) = waypoints[k].positions[j];
      }
      ++row;
    }

    // --- Position at end of last segment: p_{ns-1}(h_{ns-1}) = wp[n-1].pos ---
    {
      const auto k = ns - 1;
      const auto base = static_cast<Eigen::Index>(6 * k);
      const double hk = h[k];
      const double hk2 = hk * hk;
      const double hk3 = hk2 * hk;
      const double hk4 = hk3 * hk;
      const double hk5 = hk4 * hk;
      A(row, base) = 1.0;
      A(row, base + 1) = hk;
      A(row, base + 2) = hk2;
      A(row, base + 3) = hk3;
      A(row, base + 4) = hk4;
      A(row, base + 5) = hk5;
      for (std::size_t j = 0; j < DOF; ++j) {
        B(row, static_cast<Eigen::Index>(j)) = waypoints[n - 1].positions[j];
      }
      ++row;
    }

    // --- Boundary: start velocity = start_vel ---
    // p_0'(0) = c1_0
    {
      A(row, 1) = 1.0;
      for (std::size_t j = 0; j < DOF; ++j) {
        B(row, static_cast<Eigen::Index>(j)) = start_vel[j];
      }
      ++row;
    }

    // --- Boundary: start acceleration = start_acc ---
    // p_0''(0) = 2*c2_0
    {
      A(row, 2) = 2.0;
      for (std::size_t j = 0; j < DOF; ++j) {
        B(row, static_cast<Eigen::Index>(j)) = start_acc[j];
      }
      ++row;
    }

    // --- Boundary: end velocity = end_vel ---
    // p_{ns-1}'(h_{ns-1})
    {
      const auto k = ns - 1;
      const auto base = static_cast<Eigen::Index>(6 * k);
      const double hk = h[k];
      const double hk2 = hk * hk;
      const double hk3 = hk2 * hk;
      const double hk4 = hk3 * hk;
      A(row, base + 1) = 1.0;
      A(row, base + 2) = 2.0 * hk;
      A(row, base + 3) = 3.0 * hk2;
      A(row, base + 4) = 4.0 * hk3;
      A(row, base + 5) = 5.0 * hk4;
      for (std::size_t j = 0; j < DOF; ++j) {
        B(row, static_cast<Eigen::Index>(j)) = end_vel[j];
      }
      ++row;
    }

    // --- Boundary: end acceleration = end_acc ---
    // p_{ns-1}''(h_{ns-1})
    {
      const auto k = ns - 1;
      const auto base = static_cast<Eigen::Index>(6 * k);
      const double hk = h[k];
      const double hk2 = hk * hk;
      const double hk3 = hk2 * hk;
      A(row, base + 2) = 2.0;
      A(row, base + 3) = 6.0 * hk;
      A(row, base + 4) = 12.0 * hk2;
      A(row, base + 5) = 20.0 * hk3;
      for (std::size_t j = 0; j < DOF; ++j) {
        B(row, static_cast<Eigen::Index>(j)) = end_acc[j];
      }
      ++row;
    }

    // --- Internal continuity conditions at knots k = 0..ns-2 ---
    // Position continuity (C0): p_k(h_k) = p_{k+1}(0)
    // Velocity (C1): p_k'(h_k) = p_{k+1}'(0)
    // Acceleration (C2): p_k''(h_k) = p_{k+1}''(0)
    // Jerk (C3): p_k'''(h_k) = p_{k+1}'''(0)
    // Snap (C4): p_k''''(h_k) = p_{k+1}''''(0)

    for (std::size_t k = 0; k < ns - 1; ++k) {
      const auto base_k = static_cast<Eigen::Index>(6 * k);
      const auto base_k1 = static_cast<Eigen::Index>(6 * (k + 1));
      const double hk = h[k];
      const double hk2 = hk * hk;
      const double hk3 = hk2 * hk;
      const double hk4 = hk3 * hk;
      const double hk5 = hk4 * hk;

      // Position continuity: p_k(h_k) - p_{k+1}(0) = 0
      A(row, base_k)     = 1.0;
      A(row, base_k + 1) = hk;
      A(row, base_k + 2) = hk2;
      A(row, base_k + 3) = hk3;
      A(row, base_k + 4) = hk4;
      A(row, base_k + 5) = hk5;
      A(row, base_k1)    = -1.0;
      ++row;

      // Velocity continuity: p_k'(h_k) - p_{k+1}'(0) = 0
      A(row, base_k + 1) = 1.0;
      A(row, base_k + 2) = 2.0 * hk;
      A(row, base_k + 3) = 3.0 * hk2;
      A(row, base_k + 4) = 4.0 * hk3;
      A(row, base_k + 5) = 5.0 * hk4;
      A(row, base_k1 + 1) = -1.0;
      ++row;

      // Acceleration continuity: p_k''(h_k) - p_{k+1}''(0) = 0
      A(row, base_k + 2) = 2.0;
      A(row, base_k + 3) = 6.0 * hk;
      A(row, base_k + 4) = 12.0 * hk2;
      A(row, base_k + 5) = 20.0 * hk3;
      A(row, base_k1 + 2) = -2.0;
      ++row;

      // Jerk continuity: p_k'''(h_k) - p_{k+1}'''(0) = 0
      // p_k'''(t) = 6*c3 + 24*c4*t + 60*c5*t^2
      A(row, base_k + 3) = 6.0;
      A(row, base_k + 4) = 24.0 * hk;
      A(row, base_k + 5) = 60.0 * hk2;
      A(row, base_k1 + 3) = -6.0;
      ++row;

      // Snap continuity: p_k''''(h_k) - p_{k+1}''''(0) = 0
      // p_k''''(t) = 24*c4 + 120*c5*t
      A(row, base_k + 4) = 24.0;
      A(row, base_k + 5) = 120.0 * hk;
      A(row, base_k1 + 4) = -24.0;
      ++row;
    }

    // Solve: A * X = B, where X has shape (6*ns, DOF)
    Eigen::PartialPivLU<Eigen::MatrixXd> lu(A);
    Eigen::MatrixXd X = lu.solve(B);

    // Extract coefficients and set up QuinticPolynomials
    for (std::size_t k = 0; k < ns; ++k) {
      const auto base = static_cast<Eigen::Index>(6 * k);
      for (std::size_t j = 0; j < DOF; ++j) {
        const auto jj = static_cast<Eigen::Index>(j);
        const double c0 = X(base, jj);
        const double c1 = X(base + 1, jj);
        const double c2 = X(base + 2, jj);
        const double c3 = X(base + 3, jj);
        const double c4 = X(base + 4, jj);
        const double c5 = X(base + 5, jj);
        // QuinticPolynomial stores: c0, c1, c2 = a0/2, c3, c4, c5
        // and compute_coefficients sets c2 = a0/2.
        // So we need: p0=c0, v0=c1, a0=2*c2, pf, vf, af from end of segment.
        // Easiest: use compute_coefficients with boundary values.
        const double hk = h[k];
        const double hk2 = hk * hk;
        const double hk3 = hk2 * hk;
        const double hk4 = hk3 * hk;
        const double hk5 = hk4 * hk;
        const double pf = c0 + c1 * hk + c2 * hk2 + c3 * hk3 + c4 * hk4 + c5 * hk5;
        const double vf = c1 + 2.0 * c2 * hk + 3.0 * c3 * hk2 + 4.0 * c4 * hk3 + 5.0 * c5 * hk4;
        const double af = 2.0 * c2 + 6.0 * c3 * hk + 12.0 * c4 * hk2 + 20.0 * c5 * hk3;
        segments_[k][j].compute_coefficients(
          c0, c1, 2.0 * c2,
          pf, vf, af,
          hk);
      }
    }
  }

  /// Compute trajectory state at given time (RT path).
  [[nodiscard]] State compute(double time) const noexcept
  {
    State state;

    if (num_segments_ == 0) {
      state.positions = hold_position_;
      return state;
    }

    time = std::clamp(time, 0.0, duration_);

    // Linear search for segment
    std::size_t seg = 0;
    for (std::size_t i = 0; i < num_segments_ - 1; ++i) {
      if (time >= segment_start_times_[i + 1]) {
        seg = i + 1;
      } else {
        break;
      }
    }

    const double local_time = time - segment_start_times_[seg];

    for (std::size_t j = 0; j < DOF; ++j) {
      auto s = segments_[seg][j].compute(local_time);
      state.positions[j] = s.pos;
      state.velocities[j] = s.vel;
      state.accelerations[j] = s.acc;
    }

    return state;
  }

  [[nodiscard]] double duration() const noexcept { return duration_; }
  [[nodiscard]] std::size_t num_segments() const noexcept { return num_segments_; }

private:
  std::array<std::array<QuinticPolynomial, DOF>, kMaxWaypoints - 1> segments_{};
  std::array<double, kMaxWaypoints> segment_start_times_{};
  std::array<double, DOF> hold_position_{};
  double duration_{0.0};
  std::size_t num_segments_{0};
};

}  // namespace trajectory
}  // namespace rtc

#endif  // RTC_CONTROLLERS_TRAJECTORY_QUINTIC_SPLINE_TRAJECTORY_HPP_
