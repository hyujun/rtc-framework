#ifndef UR5E_RT_CONTROLLER_TRAJECTORY_UTILS_HPP_
#define UR5E_RT_CONTROLLER_TRAJECTORY_UTILS_HPP_

#include <cmath>
#include <algorithm>

namespace ur5e_rt_controller
{
namespace trajectory
{

struct TrajectoryState
{
  double pos{0.0};
  double vel{0.0};
  double acc{0.0};
};

class QuinticPolynomial {
public:
  QuinticPolynomial() = default;

  QuinticPolynomial(double p0, double v0, double a0, double pf, double vf, double af, double T)
  {
    compute_coefficients(p0, v0, a0, pf, vf, af, T);
  }

  void compute_coefficients(
    double p0, double v0, double a0, double pf, double vf, double af,
    double T)
  {
    if (T <= 0.0) {
      c0_ = pf;
      c1_ = vf;
      c2_ = af / 2.0;
      c3_ = 0.0;
      c4_ = 0.0;
      c5_ = 0.0;
      T_ = 0.0;
      return;
    }

    T_ = T;
    c0_ = p0;
    c1_ = v0;
    c2_ = a0 / 2.0;

    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    double dp = pf - p0 - v0 * T - a0 * T2 / 2.0;
    double dv = vf - v0 - a0 * T;
    double da = af - a0;

    c3_ = 10.0 * dp / T3 - 4.0 * dv / T2 + da / (2.0 * T);
    c4_ = -15.0 * dp / T4 + 7.0 * dv / T3 - da / T2;
    c5_ = 6.0 * dp / T5 - 3.0 * dv / T4 + da / (2.0 * T3);
  }

  TrajectoryState compute(double t) const
  {
    TrajectoryState state;
    if (T_ <= 0.0) {
      state.pos = c0_;
      state.vel = c1_;
      state.acc = 2.0 * c2_;
      return state;
    }

    t = std::clamp(t, 0.0, T_);
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    state.pos = c0_ + c1_ * t + c2_ * t2 + c3_ * t3 + c4_ * t4 + c5_ * t5;
    state.vel = c1_ + 2.0 * c2_ * t + 3.0 * c3_ * t2 + 4.0 * c4_ * t3 + 5.0 * c5_ * t4;
    state.acc = 2.0 * c2_ + 6.0 * c3_ * t + 12.0 * c4_ * t2 + 20.0 * c5_ * t3;

    return state;
  }

  double duration() const {return T_;}

private:
  double c0_{0.0};
  double c1_{0.0};
  double c2_{0.0};
  double c3_{0.0};
  double c4_{0.0};
  double c5_{0.0};
  double T_{0.0};
};

}  // namespace trajectory
}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_TRAJECTORY_UTILS_HPP_
