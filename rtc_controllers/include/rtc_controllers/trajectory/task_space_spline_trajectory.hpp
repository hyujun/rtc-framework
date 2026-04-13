#ifndef RTC_CONTROLLERS_TRAJECTORY_TASK_SPACE_SPLINE_TRAJECTORY_HPP_
#define RTC_CONTROLLERS_TRAJECTORY_TASK_SPACE_SPLINE_TRAJECTORY_HPP_

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

/// Multi-waypoint quintic spline trajectory in SE(3) task space with global C4 continuity.
/// Uses piecewise tangent-space interpolation: each segment operates in the local tangent
/// space of its start pose. A global linear system is solved per tangent-space component
/// (mapped to a common reference frame) to determine smooth internal boundary conditions.
class TaskSpaceSplineTrajectory {
public:
  struct Waypoint
  {
    pinocchio::SE3 pose;
    double time{0.0};
  };

  using State = TaskSpaceTrajectory::State;

  TaskSpaceSplineTrajectory() = default;

  /// Natural spline: start/end vel=0, acc=0 (non-RT path).
  void initialize(
    const std::array<Waypoint, kMaxWaypoints> & waypoints,
    std::size_t num_waypoints) noexcept
  {
    Eigen::Matrix<double, 6, 1> zero = Eigen::Matrix<double, 6, 1>::Zero();
    initialize(waypoints, num_waypoints, zero, zero, zero, zero);
  }

  /// Clamped spline: user-specified start/end vel and acc in local frame (non-RT path).
  void initialize(
    const std::array<Waypoint, kMaxWaypoints> & waypoints,
    std::size_t num_waypoints,
    const Eigen::Matrix<double, 6, 1> & start_vel,
    const Eigen::Matrix<double, 6, 1> & start_acc,
    const Eigen::Matrix<double, 6, 1> & end_vel,
    const Eigen::Matrix<double, 6, 1> & end_acc) noexcept
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

    // Compute tangent-space deltas per segment (in segment start's local frame)
    std::array<Eigen::Matrix<double, 6, 1>, kMaxWaypoints - 1> deltas{};
    // Relative transforms between consecutive waypoints
    std::array<pinocchio::SE3, kMaxWaypoints - 1> T_rel{};
    for (std::size_t i = 0; i < num_segments_; ++i) {
      T_rel[i] = waypoints[i].pose.actInv(waypoints[i + 1].pose);
      deltas[i] = pinocchio::log6(T_rel[i]).toVector();
    }

    // Special case: 2 waypoints = single segment
    if (n == 2) {
      const double h = waypoints[1].time - waypoints[0].time;
      // End velocity: transform from wp[1] frame to wp[0] frame
      Eigen::Matrix<double, 6, 1> v_end_local = T_rel[0].act(
        pinocchio::Motion(end_vel)).toVector();
      for (std::size_t j = 0; j < 6; ++j) {
        const auto jj = static_cast<Eigen::Index>(j);
        segments_[0][j].compute_coefficients(
          0.0, start_vel[jj], start_acc[jj],
          deltas[0][jj], v_end_local[jj], end_acc[jj],
          h);
      }
      return;
    }

    // For N>=3 waypoints, solve a 6D spline problem.
    // Strategy: Work in the first waypoint's frame as common reference.
    // Map all waypoint "positions" to wp[0]'s tangent space cumulatively.
    // This linearizes the problem, allowing us to reuse the joint-space spline solver.
    // The cumulative tangent vector from wp[0] to wp[k] is computed by chaining log maps.

    // Cumulative tangent positions in wp[0] frame
    std::array<Eigen::Matrix<double, 6, 1>, kMaxWaypoints> cum_pos{};
    cum_pos[0] = Eigen::Matrix<double, 6, 1>::Zero();
    for (std::size_t i = 1; i < n; ++i) {
      pinocchio::SE3 T_0_i = waypoints[0].pose.actInv(waypoints[i].pose);
      cum_pos[i] = pinocchio::log6(T_0_i).toVector();
    }

    // Build the spline system in 6D (same structure as QuinticSplineTrajectory)
    const auto ns = num_segments_;
    const auto sys_size = static_cast<Eigen::Index>(6 * ns);

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(sys_size, sys_size);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(sys_size, 6);

    std::array<double, kMaxWaypoints - 1> h{};
    for (std::size_t i = 0; i < ns; ++i) {
      h[i] = waypoints[i + 1].time - waypoints[i].time;
    }

    Eigen::Index row = 0;

    // Position at start of each segment
    for (std::size_t k = 0; k < ns; ++k) {
      const auto base = static_cast<Eigen::Index>(6 * k);
      A(row, base) = 1.0;
      for (Eigen::Index j = 0; j < 6; ++j) {
        B(row, j) = cum_pos[k][j];
      }
      ++row;
    }

    // Position at end of last segment
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
      for (Eigen::Index j = 0; j < 6; ++j) {
        B(row, j) = cum_pos[n - 1][j];
      }
      ++row;
    }

    // Boundary: start velocity
    {
      A(row, 1) = 1.0;
      for (Eigen::Index j = 0; j < 6; ++j) {
        B(row, j) = start_vel[j];
      }
      ++row;
    }

    // Boundary: start acceleration
    {
      A(row, 2) = 2.0;
      for (Eigen::Index j = 0; j < 6; ++j) {
        B(row, j) = start_acc[j];
      }
      ++row;
    }

    // Boundary: end velocity (transform end_vel from wp[n-1] frame to wp[0] frame)
    {
      pinocchio::SE3 T_0_last = waypoints[0].pose.actInv(waypoints[n - 1].pose);
      Eigen::Matrix<double, 6, 1> v_end_0 = T_0_last.act(
        pinocchio::Motion(end_vel)).toVector();

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
      for (Eigen::Index j = 0; j < 6; ++j) {
        B(row, j) = v_end_0[j];
      }
      ++row;
    }

    // Boundary: end acceleration
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
      for (Eigen::Index j = 0; j < 6; ++j) {
        B(row, j) = end_acc[j];
      }
      ++row;
    }

    // Internal continuity: pos (C0), vel (C1), acc (C2), jerk (C3), snap (C4)
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

      // Velocity
      A(row, base_k + 1) = 1.0;
      A(row, base_k + 2) = 2.0 * hk;
      A(row, base_k + 3) = 3.0 * hk2;
      A(row, base_k + 4) = 4.0 * hk3;
      A(row, base_k + 5) = 5.0 * hk4;
      A(row, base_k1 + 1) = -1.0;
      ++row;

      // Acceleration
      A(row, base_k + 2) = 2.0;
      A(row, base_k + 3) = 6.0 * hk;
      A(row, base_k + 4) = 12.0 * hk2;
      A(row, base_k + 5) = 20.0 * hk3;
      A(row, base_k1 + 2) = -2.0;
      ++row;

      // Jerk
      A(row, base_k + 3) = 6.0;
      A(row, base_k + 4) = 24.0 * hk;
      A(row, base_k + 5) = 60.0 * hk2;
      A(row, base_k1 + 3) = -6.0;
      ++row;

      // Snap
      A(row, base_k + 4) = 24.0;
      A(row, base_k + 5) = 120.0 * hk;
      A(row, base_k1 + 4) = -24.0;
      ++row;
    }

    // Solve the global system in wp[0]'s tangent space
    Eigen::PartialPivLU<Eigen::MatrixXd> lu(A);
    Eigen::MatrixXd X = lu.solve(B);

    // Now convert the global spline coefficients (in wp[0] frame) to per-segment
    // local tangent space polynomials (in each wp[k]'s frame).
    // For each segment k, evaluate boundary conditions from the global solution,
    // then transform to wp[k]'s local frame and build local quintic.
    for (std::size_t k = 0; k < ns; ++k) {
      const auto base = static_cast<Eigen::Index>(6 * k);
      const double hk = h[k];
      const double hk2 = hk * hk;
      const double hk3 = hk2 * hk;
      const double hk4 = hk3 * hk;

      // Extract start/end boundary in global (wp[0]) frame
      Eigen::Matrix<double, 6, 1> v_start_global;
      Eigen::Matrix<double, 6, 1> a_start_global;
      Eigen::Matrix<double, 6, 1> v_end_global;
      Eigen::Matrix<double, 6, 1> a_end_global;

      for (Eigen::Index j = 0; j < 6; ++j) {
        // Velocity at segment start: c1
        v_start_global[j] = X(base + 1, j);
        // Acceleration at segment start: 2*c2
        a_start_global[j] = 2.0 * X(base + 2, j);
        // Velocity at segment end
        v_end_global[j] = X(base + 1, j) + 2.0 * X(base + 2, j) * hk
                        + 3.0 * X(base + 3, j) * hk2 + 4.0 * X(base + 4, j) * hk3
                        + 5.0 * X(base + 5, j) * hk4;
        // Acceleration at segment end
        a_end_global[j] = 2.0 * X(base + 2, j) + 6.0 * X(base + 3, j) * hk
                        + 12.0 * X(base + 4, j) * hk2 + 20.0 * X(base + 5, j) * hk3;
      }

      // Transform velocities from wp[0] frame to segment start frame (wp[k])
      pinocchio::SE3 T_0_k = waypoints[0].pose.actInv(waypoints[k].pose);

      // v in wp[k] frame = T_0_k^{-1}.act(v in wp[0] frame)
      Eigen::Matrix<double, 6, 1> v_start_local = T_0_k.actInv(
        pinocchio::Motion(v_start_global)).toVector();
      Eigen::Matrix<double, 6, 1> a_start_local = T_0_k.actInv(
        pinocchio::Motion(a_start_global)).toVector();

      // End velocity in wp[0] frame -> transform to wp[k] frame (segment local)
      // (not wp[k+1] frame, since our segment polynomial is in wp[k]'s tangent space)
      Eigen::Matrix<double, 6, 1> v_end_local = T_0_k.actInv(
        pinocchio::Motion(v_end_global)).toVector();
      Eigen::Matrix<double, 6, 1> a_end_local = T_0_k.actInv(
        pinocchio::Motion(a_end_global)).toVector();

      for (std::size_t j = 0; j < 6; ++j) {
        const auto jj = static_cast<Eigen::Index>(j);
        segments_[k][j].compute_coefficients(
          0.0, v_start_local[jj], a_start_local[jj],
          deltas[k][jj], v_end_local[jj], a_end_local[jj],
          hk);
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

#endif  // RTC_CONTROLLERS_TRAJECTORY_TASK_SPACE_SPLINE_TRAJECTORY_HPP_
