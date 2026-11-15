#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// How far the published QP solution sits from each kind of inequality bound:
// the smallest row slack min(u − c·x, c·x − l) over the rows of that kind, in
// the rows' own units (each row is linear in a, λ with SI coefficients). A
// negative value is a violated row (within the solver's tolerance ±eps_abs).
// +inf: no such rows this tick (the constraint is absent or off, every contact
// of that kind released). The margin is the QP's own rows — a linearised
// friction pyramid is measured as the pyramid, not as the cone it
// approximates. Rows that together hold an equality (the yaw pair with a zero
// torsional coefficient, the CoP pair of a zero patch, a joint box collapsed
// to one value) sit at 0 ± eps_abs on every tick: a floor on that kind has to
// be below −eps_abs, or off.
//
// Filled by FormulationBase::AccumulateSlack from the rows the last solve
// assembled (ConstraintBase::MinSlack). Plain data, no allocation.
// ────────────────────────────────────────────────
struct WbcMargins {
  static constexpr double kNone = std::numeric_limits<double>::infinity();

  double friction_n{kNone};       // friction faces + normal-force bounds [N]
  double cop_nm{kNone};           // centre-of-pressure rectangle [N·m]
  double yaw_nm{kNone};           // yaw (torsional) moment [N·m]
  double torque_nm{kNone};        // actuator torque (and torque-rate) box [N·m]
  double joint_accel_box{kNone};  // joint position / velocity / acceleration box [rad/s²]
  // max |A x − b| over the EOM rows at the published solution [N, N·m] (on a
  // WQP the tick's CommandOutput::eom_residual_inf); NaN when unknown.
  double eom_residual_inf{std::numeric_limits<double>::quiet_NaN()};
  // Not this tick's: no tick has published a solution yet, or the last tick
  // did not (tau_valid false) — the values are the last valid tick's.
  bool stale{true};

  // Every margin back to "no rows" (stale untouched).
  void ClearSlack() noexcept {
    friction_n = cop_nm = yaw_nm = torque_nm = joint_accel_box = kNone;
    eom_residual_inf = std::numeric_limits<double>::quiet_NaN();
  }
};

// The slack of row `i` of (C, l, u) at x: min(u_i − c_i·x, c_i·x − l_i).
// NaN when the row or x is not finite.
[[nodiscard]] inline double RowSlack(const Eigen::Ref<const Eigen::MatrixXd>& C,
                                     const Eigen::Ref<const Eigen::VectorXd>& l,
                                     const Eigen::Ref<const Eigen::VectorXd>& u,
                                     const Eigen::VectorXd& x, Eigen::Index i) noexcept {
  const double cx = C.row(i).dot(x);
  const double up = u(i) - cx;
  const double lo = cx - l(i);
  if (std::isnan(up) || std::isnan(lo)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::min(up, lo);
}

// `margin` lowered to `slack` (a NaN slack makes the margin NaN).
inline void LowerTo(double& margin, double slack) noexcept {
  if (std::isnan(slack) || std::isnan(margin)) {
    margin = std::numeric_limits<double>::quiet_NaN();
  } else {
    margin = std::min(margin, slack);
  }
}

}  // namespace rtc::tsid
