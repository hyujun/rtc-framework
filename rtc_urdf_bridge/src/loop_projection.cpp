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

ProjectionResult ProjectPassiveToConstraint(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q_init,
    const std::vector<pinocchio::JointIndex>& actuated_joint_ids, const ProjectionOptions& opts) {
  ProjectionResult result;
  result.q = q_init;

  const int m = TotalConstraintDim(constraints);
  if (m == 0) {
    result.converged = true;
    return result;
  }

  // actuated velocity 열 마스크 → passive 열 인덱스 목록 (1회 구성).
  std::vector<bool> is_actuated_col(static_cast<std::size_t>(model.nv), false);
  for (const auto jid : actuated_joint_ids) {
    if (jid == 0 || jid >= static_cast<pinocchio::JointIndex>(model.njoints)) {
      continue;  // universe(0)/invalid — skip
    }
    const int vs = model.idx_vs[jid];
    const int nvj = model.nvs[jid];
    for (int k = 0; k < nvj; ++k) {
      is_actuated_col[static_cast<std::size_t>(vs + k)] = true;
    }
  }
  std::vector<int> passive_cols;
  passive_cols.reserve(static_cast<std::size_t>(model.nv));
  for (int c = 0; c < model.nv; ++c) {
    if (!is_actuated_col[static_cast<std::size_t>(c)]) {
      passive_cols.push_back(c);
    }
  }
  const int n_pass = static_cast<int>(passive_cols.size());

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

    // passive 열만 추출: Jc_pass (m × n_pass)
    Eigen::MatrixXd Jc_pass(m, n_pass);
    for (int j = 0; j < n_pass; ++j) {
      Jc_pass.col(j) = kin.Jc.col(passive_cols[static_cast<std::size_t>(j)]);
    }

    // dv_pass = −Jc_passᵀ (Jc_pass Jc_passᵀ + λ²I)⁻¹ φ
    const Eigen::MatrixXd JJt = Jc_pass * Jc_pass.transpose() + lambda2 * I;
    const Eigen::VectorXd y = JJt.ldlt().solve(kin.phi);
    const Eigen::VectorXd dv_pass = -Jc_pass.transpose() * y;  // n_pass

    // 전체 tangent 로 scatter: actuated 열 = 0 → integrate 시 actuated q 정확히 불변.
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(model.nv);
    for (int j = 0; j < n_pass; ++j) {
      dq[passive_cols[static_cast<std::size_t>(j)]] = dv_pass[j];
    }

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
