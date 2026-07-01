// ── loop_projection 구현 ─────────────────────────────────────────────────────
#include "rtc_urdf_bridge/loop_projection.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Dense>

namespace rtc_urdf_bridge {

ProjectionResult ProjectToConstraint(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q_init,
    const ProjectionOptions& opts) {
  ProjectionResult result;
  result.q = q_init;

  const int m = TotalConstraintDim(constraints);
  if (m == 0) {
    result.converged = true;
    return result;
  }

  const double lambda2 = opts.damping * opts.damping;  // λ²
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m, m);

  for (int iter = 0; iter < opts.max_iterations; ++iter) {
    const ConstraintKinematics kin =
        ComputeConstraintKinematics(model, data, constraints, result.q);
    const double err = kin.phi.norm();
    result.iterations = iter;
    result.final_error = err;
    if (err < opts.tolerance) {
      result.converged = true;
      return result;
    }

    // dq = −Jcᵀ (Jc Jcᵀ + λ²I)⁻¹ φ
    const Eigen::MatrixXd JJt = kin.Jc * kin.Jc.transpose() + lambda2 * I;
    const Eigen::VectorXd y = JJt.ldlt().solve(kin.phi);
    const Eigen::VectorXd dq = -kin.Jc.transpose() * y;  // nv

    // manifold 적분: q ← integrate(model, q, dq)
    Eigen::VectorXd q_next(model.nq);
    pinocchio::integrate(model, result.q, dq, q_next);
    result.q = q_next;
  }

  // 마지막 상태로 최종 error 재평가
  const ConstraintKinematics kin = ComputeConstraintKinematics(model, data, constraints, result.q);
  result.final_error = kin.phi.norm();
  result.converged = result.final_error < opts.tolerance;
  return result;
}

Eigen::VectorXd ProjectVelocity(const pinocchio::Model& model, pinocchio::Data& data,
                                const std::vector<pinocchio::RigidConstraintModel>& constraints,
                                const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                double damping) {
  const int m = TotalConstraintDim(constraints);
  if (m == 0) {
    return v;
  }
  const ConstraintKinematics kin = ComputeConstraintKinematics(model, data, constraints, q);
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m, m);
  const Eigen::MatrixXd JJt = kin.Jc * kin.Jc.transpose() + (damping * damping) * I;
  const Eigen::VectorXd Jv = kin.Jc * v;
  const Eigen::VectorXd correction = kin.Jc.transpose() * JJt.ldlt().solve(Jv);
  return v - correction;
}

}  // namespace rtc_urdf_bridge
