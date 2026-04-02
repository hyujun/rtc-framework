#ifndef RTC_CONTROLLERS_TRAJECTORY_QUINTIC_BLEND_TRAJECTORY_HPP_
#define RTC_CONTROLLERS_TRAJECTORY_QUINTIC_BLEND_TRAJECTORY_HPP_

#include "rtc_controllers/trajectory/trajectory_utils.hpp"
#include <array>
#include <cstddef>
#include <algorithm>

namespace rtc
{
namespace trajectory
{

/// Maximum number of waypoints for blend/spline trajectories.
inline constexpr std::size_t kMaxWaypoints = 32;

/// Multi-waypoint quintic blend trajectory in joint space.
/// Each segment uses a quintic polynomial with C2 continuity at via-points.
/// Via-point velocities are computed as the average of adjacent segment velocities.
/// Start/end velocities and all accelerations are zero (rest-to-rest).
template<std::size_t DOF>
class QuinticBlendTrajectory {
  static_assert(DOF > 0, "QuinticBlendTrajectory: DOF must be at least 1");

public:
  struct Waypoint
  {
    std::array<double, DOF> positions{};
    double time{0.0};
  };

  struct State
  {
    std::array<double, DOF> positions{};
    std::array<double, DOF> velocities{};
    std::array<double, DOF> accelerations{};
  };

  QuinticBlendTrajectory() = default;

  /// Initialize blend trajectory from waypoints (non-RT path).
  /// @param waypoints  Array of waypoints (2 to kMaxWaypoints).
  /// @param num_waypoints  Number of valid waypoints.
  /// @pre waypoints[0].time == 0.0, times monotonically increasing.
  void initialize(
    const std::array<Waypoint, kMaxWaypoints> & waypoints,
    std::size_t num_waypoints) noexcept
  {
    if (num_waypoints < 2) {
      // Hold position at first waypoint
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

    // Store segment start times
    for (std::size_t i = 0; i < n; ++i) {
      segment_start_times_[i] = waypoints[i].time;
    }

    // Compute via-point velocities (average of adjacent segment velocities)
    std::array<std::array<double, DOF>, kMaxWaypoints> velocities{};
    // Start and end: zero velocity (rest-to-rest)
    // velocities[0] and velocities[n-1] are already zero-initialized

    for (std::size_t i = 1; i < n - 1; ++i) {
      const double h_prev = waypoints[i].time - waypoints[i - 1].time;
      const double h_next = waypoints[i + 1].time - waypoints[i].time;
      for (std::size_t j = 0; j < DOF; ++j) {
        const double v_prev = (waypoints[i].positions[j] - waypoints[i - 1].positions[j]) / h_prev;
        const double v_next = (waypoints[i + 1].positions[j] - waypoints[i].positions[j]) / h_next;
        velocities[i][j] = 0.5 * (v_prev + v_next);
      }
    }

    // Build quintic polynomials for each segment and DOF
    for (std::size_t seg = 0; seg < num_segments_; ++seg) {
      const double h = waypoints[seg + 1].time - waypoints[seg].time;
      for (std::size_t j = 0; j < DOF; ++j) {
        segments_[seg][j].compute_coefficients(
          waypoints[seg].positions[j], velocities[seg][j], 0.0,
          waypoints[seg + 1].positions[j], velocities[seg + 1][j], 0.0,
          h);
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

    // Clamp time
    time = std::clamp(time, 0.0, duration_);

    // Find segment index via linear search
    std::size_t seg = 0;
    for (std::size_t i = 0; i < num_segments_ - 1; ++i) {
      if (time >= segment_start_times_[i + 1]) {
        seg = i + 1;
      } else {
        break;
      }
    }

    // Local time within segment
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

#endif  // RTC_CONTROLLERS_TRAJECTORY_QUINTIC_BLEND_TRAJECTORY_HPP_
