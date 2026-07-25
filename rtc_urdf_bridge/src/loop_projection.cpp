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

#include <algorithm>
#include <cmath>

namespace rtc_urdf_bridge {

namespace {

// Damped least-squares Newton projection over a chosen set of velocity columns.
// Only the tangent columns in `free_cols` may move; every other column keeps a
// zero increment, so `integrate` leaves its q exactly unchanged. Shared core of
// ProjectToConstraint (all columns free) and ProjectPassiveToConstraint (only
// passive columns free — actuated q held fixed).
//   dv_free = −Jc_freeᵀ (Jc_free Jc_freeᵀ + λ²I)⁻¹ φ(q); q ← integrate(model, q, dq).
ProjectionResult ProjectOverColumns(const pinocchio::Model& model, pinocchio::Data& data,
                                    const std::vector<pinocchio::RigidConstraintModel>& constraints,
                                    const Eigen::VectorXd& q_init,
                                    const std::vector<int>& free_cols,
                                    const ProjectionOptions& opts) {
  ProjectionResult result;
  result.q = q_init;

  const int m = TotalConstraintDim(constraints);
  if (m == 0) {
    result.converged = true;
    return result;
  }

  const int n_free = static_cast<int>(free_cols.size());
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

    // free 열만 추출: Jc_free (m × n_free)
    Eigen::MatrixXd Jc_free(m, n_free);
    for (int j = 0; j < n_free; ++j) {
      Jc_free.col(j) = kin.Jc.col(free_cols[static_cast<std::size_t>(j)]);
    }

    // dv_free = −Jc_freeᵀ (Jc_free Jc_freeᵀ + λ²I)⁻¹ φ
    const Eigen::MatrixXd JJt = Jc_free * Jc_free.transpose() + lambda2 * I;
    const Eigen::VectorXd y = JJt.ldlt().solve(kin.phi);
    const Eigen::VectorXd dv_free = -Jc_free.transpose() * y;  // n_free

    // 전체 tangent 로 scatter: 미포함(고정) 열 = 0 → integrate 시 해당 q 정확히 불변.
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(model.nv);
    for (int j = 0; j < n_free; ++j) {
      dq[free_cols[static_cast<std::size_t>(j)]] = dv_free[j];
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

}  // namespace

ProjectionResult ProjectToConstraint(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q_init,
    const ProjectionOptions& opts) {
  // 모든 velocity 열이 자유 (구속 없는 nv 전체 사영).
  std::vector<int> all_cols(static_cast<std::size_t>(model.nv));
  for (int c = 0; c < model.nv; ++c) {
    all_cols[static_cast<std::size_t>(c)] = c;
  }
  return ProjectOverColumns(model, data, constraints, q_init, all_cols, opts);
}

ProjectionResult ProjectPassiveToConstraint(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q_init,
    const std::vector<pinocchio::JointIndex>& actuated_joint_ids, const ProjectionOptions& opts) {
  // actuated velocity 열 마스크 → passive(자유) 열 인덱스 목록.
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
  return ProjectOverColumns(model, data, constraints, q_init, passive_cols, opts);
}

ProjectionResult ProjectPassiveWithContinuation(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q_prev,
    const Eigen::VectorXd& q_target, const std::vector<pinocchio::JointIndex>& actuated_joint_ids,
    const ProjectionOptions& opts, double max_actuated_increment) {
  // 크기 계약 위반 / continuation 비활성 → 단일 사영으로 위임 (기존 동작).
  const bool sizes_ok = (q_prev.size() == model.nq) && (q_target.size() == model.nq);
  if (!sizes_ok || max_actuated_increment <= 0.0) {
    return ProjectPassiveToConstraint(model, data, constraints, q_target, actuated_joint_ids, opts);
  }

  // actuated 열에서의 seed 증분 크기. difference() 는 continuous(cos,sin)·multi-DoF 관절도
  // tangent 로 올바르게 다룬다 (스칼라 뺄셈은 wrap 을 놓친다).
  const Eigen::VectorXd dv = pinocchio::difference(model, q_prev, q_target);
  double max_delta = 0.0;
  for (const auto jid : actuated_joint_ids) {
    if (jid == 0 || jid >= static_cast<pinocchio::JointIndex>(model.njoints)) {
      continue;  // universe(0)/invalid — skip
    }
    const int vs = model.idx_vs[jid];
    const int nvj = model.nvs[jid];
    for (int k = 0; k < nvj; ++k) {
      max_delta = std::max(max_delta, std::abs(dv[vs + k]));
    }
  }
  if (!std::isfinite(max_delta)) {
    // 비유한 측정 — 쪼개도 의미가 없다. 단일 사영에 맡기고 소비자의 finite guard 가 처리.
    return ProjectPassiveToConstraint(model, data, constraints, q_target, actuated_joint_ids, opts);
  }

  const int n_sub = (max_delta <= max_actuated_increment)
                        ? 1
                        : std::min(kMaxContinuationSubsteps,
                                   static_cast<int>(std::ceil(max_delta / max_actuated_increment)));

  ProjectionResult result;
  result.q = q_prev;  // passive warm-start 원천 (actuated 는 아래에서 매 sub-step 덮어쓴다)
  Eigen::VectorXd q_interp(model.nq);
  Eigen::VectorXd q_seed(model.nq);
  for (int s = 1; s <= n_sub; ++s) {
    const double u = static_cast<double>(s) / static_cast<double>(n_sub);
    pinocchio::interpolate(model, q_prev, q_target, u, q_interp);
    // passive = 직전 sub-step 해, actuated = 보간값. q_target 의 passive 슬롯은 쓰지 않는다.
    q_seed = result.q;
    for (const auto jid : actuated_joint_ids) {
      if (jid == 0 || jid >= static_cast<pinocchio::JointIndex>(model.njoints)) {
        continue;
      }
      const int qs = model.idx_qs[jid];
      const int nqj = model.nqs[jid];
      q_seed.segment(qs, nqj) = q_interp.segment(qs, nqj);
    }
    result = ProjectPassiveToConstraint(model, data, constraints, q_seed, actuated_joint_ids, opts);
    // 중간 sub-step 이 미수렴이면 **즉시 중단**한다. 그 해는 이미 loop-consistent 가 아니므로
    // 이어서 warm-start 로 쓰면 homotopy 보장이 깨진 경로를 따라가고, 그럼에도 마지막
    // sub-step 만 수렴하면 converged=true 로 돌아가 소비자가 그 형상을 커밋한다 — 단일 사영
    // 시절에는 없던 실패 은폐 경로다. 실패 지점의 결과(부분 진행 q + converged=false)를 그대로
    // 돌려 소비자의 기존 hold 정책이 작동하게 한다.
    if (!result.converged) {
      return result;
    }
  }
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
