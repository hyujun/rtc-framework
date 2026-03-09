#ifndef UR5E_RT_CONTROLLER_JOINT_SPACE_TRAJECTORY_HPP_
#define UR5E_RT_CONTROLLER_JOINT_SPACE_TRAJECTORY_HPP_

#include "ur5e_rt_controller/trajectory/trajectory_utils.hpp"
#include <array>
#include <cstddef>

namespace ur5e_rt_controller
{
namespace trajectory
{

template<std::size_t N>
class JointSpaceTrajectory {
public:
  struct State
  {
    std::array<double, N> positions{};
    std::array<double, N> velocities{};
    std::array<double, N> accelerations{};
  };

  JointSpaceTrajectory() = default;

  void initialize(const State & start_state, const State & goal_state, double duration)
  {
    duration_ = duration;
    for (std::size_t i = 0; i < N; ++i) {
      polynomials_[i].compute_coefficients(
          start_state.positions[i], start_state.velocities[i], start_state.accelerations[i],
          goal_state.positions[i], goal_state.velocities[i], goal_state.accelerations[i],
          duration);
    }
  }

  State compute(double time) const
  {
    State state;
    for (std::size_t i = 0; i < N; ++i) {
      auto p_state = polynomials_[i].compute(time);
      state.positions[i] = p_state.pos;
      state.velocities[i] = p_state.vel;
      state.accelerations[i] = p_state.acc;
    }
    return state;
  }

  double duration() const {return duration_;}

private:
  std::array<QuinticPolynomial, N> polynomials_;
  double duration_{0.0};
};

}  // namespace trajectory
}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_JOINT_SPACE_TRAJECTORY_HPP_
