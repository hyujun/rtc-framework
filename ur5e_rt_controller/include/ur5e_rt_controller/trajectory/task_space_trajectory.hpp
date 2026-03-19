#ifndef UR5E_RT_CONTROLLER_TASK_SPACE_TRAJECTORY_HPP_
#define UR5E_RT_CONTROLLER_TASK_SPACE_TRAJECTORY_HPP_

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

#include "ur5e_rt_controller/trajectory/trajectory_utils.hpp"
#include <array>

namespace ur5e_rt_controller
{
namespace trajectory
{

class TaskSpaceTrajectory {
public:
  struct State
  {
    pinocchio::SE3 pose;
    pinocchio::Motion velocity;       // spatial velocity in LOCAL frame
    pinocchio::Motion acceleration;   // spatial acceleration in LOCAL frame
  };

  TaskSpaceTrajectory()
  {
    pose_start_.setIdentity();
  }

  void initialize(
    const pinocchio::SE3 & start_pose,
    const pinocchio::Motion & start_velocity_local,
    const pinocchio::SE3 & goal_pose,
    const pinocchio::Motion & goal_velocity_local,
    double duration)
  {
    duration_ = duration;
    pose_start_ = start_pose;

    // We wish to interpolate in the tangent space of start_pose.
    // Let delta_X = log6(start_pose^-1 * goal_pose) -> a 6D vector (linear, angular)
    pinocchio::SE3 T_start_goal = start_pose.actInv(goal_pose);
    pinocchio::Motion delta_X = pinocchio::log6(T_start_goal);

    // Initial velocity is already in start_pose local frame.
    pinocchio::Motion delta_v0 = start_velocity_local;

    // Transform goal_velocity to start_pose frame
    // goal_velocity_local is in goal_pose frame. Its representation in start_pose frame:
    pinocchio::Motion delta_vf = T_start_goal.act(goal_velocity_local);

    Eigen::VectorXd p_goal = delta_X.toVector();
    Eigen::VectorXd v_start = delta_v0.toVector();
    Eigen::VectorXd v_goal = delta_vf.toVector();

    for (int i = 0; i < 6; ++i) {
      polynomials_[i].compute_coefficients(
          0.0, v_start[i], 0.0,
          p_goal[i], v_goal[i], 0.0,
          duration);
    }
  }

  State compute(double time) const
  {
    State state;
    Eigen::Matrix<double, 6, 1> p;
    Eigen::Matrix<double, 6, 1> v;
    Eigen::Matrix<double, 6, 1> a;

    for (int i = 0; i < 6; ++i) {
      auto p_state = polynomials_[i].compute(time);
      p[i] = p_state.pos;
      v[i] = p_state.vel;
      a[i] = p_state.acc;
    }

    pinocchio::Motion delta_X(p);
    pinocchio::SE3 T_start_current = pinocchio::exp6(delta_X);

    state.pose = pose_start_ * T_start_current;

    // The right Jacobian of exp6 relates \dot{p} to the spatial velocity in the local frame.
    // T(t) = T_start * exp6(p(t))
    // \dot{T} = T_start * \dot{exp6}(p) = T_start * (exp6(p) * (Jexp6(p) * \dot{p})^)
    // Hence, the spatial twist in T(t)'s local frame is exactly Jexp6(p) * \dot{p}.
    Eigen::Matrix<double, 6, 6> Jexp;
    pinocchio::Jexp6(delta_X, Jexp);

    state.velocity = pinocchio::Motion(Jexp * v);

    // Approximate acceleration as Jexp * a + \dot{Jext} * v \approx Jexp * a
    state.acceleration = pinocchio::Motion(Jexp * a);

    return state;
  }

  double duration() const {return duration_;}

private:
  std::array<QuinticPolynomial, 6> polynomials_;
  pinocchio::SE3 pose_start_;
  double duration_{0.0};
};

}  // namespace trajectory
}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_TASK_SPACE_TRAJECTORY_HPP_
