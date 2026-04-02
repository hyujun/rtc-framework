#ifndef RTC_CONTROLLERS_TRAJECTORY_TASK_SPACE_BLEND_TRAJECTORY_HPP_
#define RTC_CONTROLLERS_TRAJECTORY_TASK_SPACE_BLEND_TRAJECTORY_HPP_

#include "rtc_controllers/trajectory/trajectory_utils.hpp"
#include "rtc_controllers/trajectory/quintic_blend_trajectory.hpp"
#include "rtc_controllers/trajectory/task_space_trajectory.hpp"

// Suppress warnings emitted by Pinocchio
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/explog.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <algorithm>

namespace rtc
{
namespace trajectory
{

/// Multi-waypoint quintic blend trajectory in SE(3) task space.
/// Each segment interpolates in the local tangent space of the segment's start pose
/// using quintic polynomials. Via-point velocities are averaged from adjacent segments.
/// Start/end velocities are zero (rest-to-rest).
class TaskSpaceBlendTrajectory {
public:
  struct Waypoint
  {
    pinocchio::SE3 pose;
    double time{0.0};
  };

  using State = TaskSpaceTrajectory::State;

  TaskSpaceBlendTrajectory() = default;

  /// Initialize from waypoints (non-RT path).
  void initialize(
    const std::array<Waypoint, kMaxWaypoints> & waypoints,
    std::size_t num_waypoints) noexcept
  {
    if (num_waypoints < 2) {
      num_segments_ = 0;
      duration_ = 0.0;
      if (num_waypoints == 1) {
        hold_pose_ = waypoints[0].pose;
      } else {
        hold_pose_.setIdentity();
      }
      return;
    }

    const std::size_t n = std::min(num_waypoints, kMaxWaypoints);
    num_segments_ = n - 1;
    duration_ = waypoints[n - 1].time;

    for (std::size_t i = 0; i < n; ++i) {
      segment_start_times_[i] = waypoints[i].time;
      segment_start_poses_[i] = waypoints[i].pose;
    }

    // Compute log6 deltas for each segment (in start pose local frame)
    std::array<Eigen::Matrix<double, 6, 1>, kMaxWaypoints - 1> deltas{};
    for (std::size_t i = 0; i < num_segments_; ++i) {
      pinocchio::SE3 T_rel = waypoints[i].pose.actInv(waypoints[i + 1].pose);
      deltas[i] = pinocchio::log6(T_rel).toVector();
    }

    // Compute via-point velocities in each waypoint's local frame.
    // For internal points, average the "arriving" and "departing" velocities.
    // Arriving velocity at wp[i]: delta[i-1] / h[i-1] (in wp[i-1] frame)
    //   -> transform to wp[i] frame: T_{i-1,i}^{-1}.act(v)
    // Departing velocity at wp[i]: delta[i] / h[i] (already in wp[i] frame)
    std::array<Eigen::Matrix<double, 6, 1>, kMaxWaypoints> velocities{};
    // velocities[0] = 0 (rest), velocities[n-1] = 0 (rest) — already zero-initialized

    for (std::size_t i = 1; i < n - 1; ++i) {
      const double h_prev = waypoints[i].time - waypoints[i - 1].time;
      const double h_next = waypoints[i + 1].time - waypoints[i].time;

      // Departing velocity at wp[i] in wp[i] local frame
      Eigen::Matrix<double, 6, 1> v_depart = deltas[i] / h_next;

      // Arriving velocity: delta[i-1]/h_prev is in wp[i-1] frame.
      // Transform to wp[i] frame via T_{i-1,i} = wp[i-1]^{-1} * wp[i]
      // Actually, arriving vel at wp[i] from segment i-1 is the end velocity
      // of that segment. In wp[i-1]'s tangent space it's delta[i-1]/h_prev.
      // To express in wp[i]'s frame: use adjoint of T_{i-1,i}^{-1}
      pinocchio::SE3 T_prev_to_i = waypoints[i - 1].pose.actInv(waypoints[i].pose);
      pinocchio::Motion v_arrive_prev_frame(deltas[i - 1] / h_prev);
      // Transform: v_in_i = T_prev_to_i^{-1}.act(v_in_prev)
      // actInv: SE3.actInv(Motion) = inverse(T) * v
      Eigen::Matrix<double, 6, 1> v_arrive = T_prev_to_i.actInv(v_arrive_prev_frame).toVector();

      velocities[i] = 0.5 * (v_arrive + v_depart);
    }

    // Build quintic polynomials for each segment (6D in local tangent space)
    for (std::size_t seg = 0; seg < num_segments_; ++seg) {
      const double h = waypoints[seg + 1].time - waypoints[seg].time;

      // Start velocity in segment's local frame (wp[seg] frame)
      Eigen::Matrix<double, 6, 1> v_start = velocities[seg];

      // End velocity: velocities[seg+1] is in wp[seg+1] frame.
      // Transform to wp[seg] frame via T_{seg, seg+1}.act()
      pinocchio::SE3 T_seg_to_next = waypoints[seg].pose.actInv(waypoints[seg + 1].pose);
      Eigen::Matrix<double, 6, 1> v_end = T_seg_to_next.act(
        pinocchio::Motion(velocities[seg + 1])).toVector();

      for (std::size_t j = 0; j < 6; ++j) {
        const auto jj = static_cast<Eigen::Index>(j);
        segments_[seg][j].compute_coefficients(
          0.0, v_start[jj], 0.0,
          deltas[seg][jj], v_end[jj], 0.0,
          h);
      }
    }
  }

  /// Compute trajectory state at given time (RT path).
  [[nodiscard]] State compute(double time) const noexcept
  {
    State state;

    if (num_segments_ == 0) {
      state.pose = hold_pose_;
      state.velocity = pinocchio::Motion::Zero();
      state.acceleration = pinocchio::Motion::Zero();
      return state;
    }

    time = std::clamp(time, 0.0, duration_);

    // Find segment
    std::size_t seg = 0;
    for (std::size_t i = 0; i < num_segments_ - 1; ++i) {
      if (time >= segment_start_times_[i + 1]) {
        seg = i + 1;
      } else {
        break;
      }
    }

    const double local_time = time - segment_start_times_[seg];

    Eigen::Matrix<double, 6, 1> p;
    Eigen::Matrix<double, 6, 1> v;
    Eigen::Matrix<double, 6, 1> a;

    for (std::size_t j = 0; j < 6; ++j) {
      auto s = segments_[seg][j].compute(local_time);
      const auto jj = static_cast<Eigen::Index>(j);
      p[jj] = s.pos;
      v[jj] = s.vel;
      a[jj] = s.acc;
    }

    pinocchio::Motion delta_X(p);
    pinocchio::SE3 T_start_current = pinocchio::exp6(delta_X);
    state.pose = segment_start_poses_[seg] * T_start_current;

    Eigen::Matrix<double, 6, 6> Jexp;
    pinocchio::Jexp6(delta_X, Jexp);
    state.velocity = pinocchio::Motion(Jexp * v);
    state.acceleration = pinocchio::Motion(Jexp * a);

    return state;
  }

  [[nodiscard]] double duration() const noexcept { return duration_; }
  [[nodiscard]] std::size_t num_segments() const noexcept { return num_segments_; }

private:
  std::array<std::array<QuinticPolynomial, 6>, kMaxWaypoints - 1> segments_{};
  std::array<double, kMaxWaypoints> segment_start_times_{};
  std::array<pinocchio::SE3, kMaxWaypoints> segment_start_poses_{};
  pinocchio::SE3 hold_pose_;
  double duration_{0.0};
  std::size_t num_segments_{0};
};

}  // namespace trajectory
}  // namespace rtc

#endif  // RTC_CONTROLLERS_TRAJECTORY_TASK_SPACE_BLEND_TRAJECTORY_HPP_
