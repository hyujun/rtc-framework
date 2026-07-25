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
#include <limits>
#include <stdexcept>
#include <string>

namespace rtc_urdf_bridge {

namespace {

// Stall(residual floor) 감지: 연속 kStallIterations 회 상대 개선이 kStallRelImprovement
// 미만이면 더 진행해도 residual 이 내려가지 않는다고 보고 조기 종료한다. URDF 좌표
// 불일치로 floor 가 있으면 strict tolerance 에 원리적으로 도달 불가라, 이 감지가 없으면
// 매 호출 max_iterations 를 전부 소진한다 (#250 D3). Newton 은 해 근방에서 초선형
// (iteration 당 수십 % 이상 개선) 이므로 1% 임계는 정상 수렴을 자르지 않고, floor 는
// 개선율 ≈ 0 이라 2회 연속으로 확실히 걸린다. 정밀도는 보존된다 — 개선이 계속되는 한
// strict tolerance 까지 반복한다. (acceptance_tolerance 는 여기 관여하지 않는다 —
// stopping 기준이 아니라 종료 후 분류다.)
constexpr double kStallRelImprovement = 1e-2;
constexpr int kStallIterations = 2;

// ProjectionOptions 검증 (position-level 진입점 공통). 로드 타임/비-RT 전용이라 예외 허용.
// tolerance == 0 은 "early-stop 금지 (정확히 max_iterations 스텝)" 의 기존 관용구라 허용
// (test_closed_chain_fk_measurement fixed-K 스파이크) — 음수·비유한만 거부. 이 관용구를
// 지키기 위해 stall 감지도 tolerance == 0 이면 비활성이다 (ProjectOverColumns 참조).
void ValidateOptions(const ProjectionOptions& opts) {
  if (!std::isfinite(opts.tolerance) || opts.tolerance < 0.0) {
    throw std::invalid_argument(
        "loop_projection: tolerance 는 유한 음이 아닌 값이어야 합니다 (given " +
        std::to_string(opts.tolerance) + ")");
  }
  if (!std::isfinite(opts.acceptance_tolerance) || opts.acceptance_tolerance <= 0.0) {
    throw std::invalid_argument(
        "loop_projection: acceptance_tolerance 는 유한 양수여야 합니다 (given " +
        std::to_string(opts.acceptance_tolerance) + ")");
  }
  if (opts.acceptance_tolerance < opts.tolerance) {
    throw std::invalid_argument("loop_projection: acceptance_tolerance (" +
                                std::to_string(opts.acceptance_tolerance) +
                                ") 가 strict tolerance (" + std::to_string(opts.tolerance) +
                                ") 보다 작습니다 — acceptance 는 strict 의 완화 임계여야 합니다");
  }
}

// 종료 후 수용 분류 (#250 D4). φ 유한 ∧ 모든 CONTACT_6D segment 가 strict 충족 ∧
// ‖φ‖ ≤ acceptance. 6D residual 은 병진(m)·회전(rad) 혼합 norm 이라 m 단위 acceptance 를
// 적용하지 않는다 — 6D constraint 는 strict 로만 수용된다 (segment 가 strict 미만이면
// 전체 norm 기여도 무시 가능 수준이라 3D rows 판정을 오염시키지 않는다).
// 함의: tolerance == 0 (fixed-K 관용구) 에서는 6D segment 가 strict 를 충족할 수 없어
// (norm ≥ 0 항상 참) 6D loop 모델의 acceptable 은 항상 false 다 — converged 가 정의상
// 항상 false 인 것과 일관된 의미이며, fixed-K 소비자는 acceptable 을 소비하지 않는다.
bool IsResidualAcceptable(const ConstraintKinematics& kin, const ProjectionOptions& opts) {
  if (!kin.phi.allFinite()) {
    return false;
  }
  for (std::size_t i = 0; i < kin.row_sizes.size(); ++i) {
    if (kin.row_sizes[i] == 6 &&
        kin.phi.segment(kin.row_offsets[i], kin.row_sizes[i]).norm() >= opts.tolerance) {
      return false;
    }
  }
  return kin.phi.norm() <= opts.acceptance_tolerance;
}

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
  ValidateOptions(opts);

  ProjectionResult result;
  result.q = q_init;

  const int m = TotalConstraintDim(constraints);
  if (m == 0) {
    result.converged = true;
    result.acceptable = true;
    return result;
  }

  const int n_free = static_cast<int>(free_cols.size());
  const double lambda2 = opts.damping * opts.damping;  // λ²
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m, m);

  double prev_err = std::numeric_limits<double>::infinity();
  int stall_count = 0;
  for (int iter = 0; iter < opts.max_iterations; ++iter) {
    const ConstraintKinematics kin =
        ComputeConstraintKinematics(model, data, constraints, result.q);
    const double err = kin.phi.norm();
    result.iterations = iter;
    result.final_error = err;
    if (err < opts.tolerance) {
      result.converged = true;
      result.acceptable = true;
      return result;
    }

    // Stall(residual floor): 개선이 멈추면 반복해도 내려가지 않는다 — 조기 종료 후
    // 종료-후 분류만 수행. (NaN err 는 모든 비교가 false 라 여기 걸리지 않고
    // max_iterations 까지 돈 뒤 acceptable=false 로 분류된다.) tolerance == 0 은
    // "early-stop 금지 (정확히 max_iterations 스텝)" 의 fixed-K 관용구이므로 stall 도
    // early-stop 의 일종인 이상 비활성 — 아니면 machine floor 에서 K 미만 스텝으로 조기
    // 반환해 fixed-K 결정성이 조용히 깨진다 (PR #251 리뷰).
    if (opts.tolerance > 0.0 && err >= prev_err * (1.0 - kStallRelImprovement)) {
      if (++stall_count >= kStallIterations) {
        result.acceptable = IsResidualAcceptable(kin, opts);
        return result;
      }
    } else {
      stall_count = 0;
    }
    prev_err = err;

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

  // 마지막 상태로 최종 error 재평가 + 종료 후 수용 분류
  const ConstraintKinematics kin = ComputeConstraintKinematics(model, data, constraints, result.q);
  result.final_error = kin.phi.norm();
  result.converged = result.final_error < opts.tolerance;
  result.acceptable = result.converged || IsResidualAcceptable(kin, opts);
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
    // 중간 sub-step 이 **비수용(!acceptable)** 이면 즉시 중단한다. 그 해는 loop-consistent 가
    // 아니므로 이어서 warm-start 로 쓰면 homotopy 보장이 깨진 경로를 따라가고, 그럼에도 마지막
    // sub-step 만 통과하면 성공으로 돌아가 소비자가 그 형상을 커밋한다 — 단일 사영 시절에는
    // 없던 실패 은폐 경로다. 실패 지점의 결과(부분 진행 q + acceptable=false)를 그대로 돌려
    // 소비자의 기존 hold 정책이 작동하게 한다. strict 미달이어도 acceptable 인 sub-step
    // (residual floor ≤1 µm — URDF 좌표 불일치는 형상 무관 상수라 모든 sub-step 이 겪는다) 은
    // 실질 loop-consistent 라 계속 진행한다 — 분기 간 거리는 rad 단위라 µm floor 로 homotopy 는
    // 훼손되지 않는다 (#250 D2, invariants.md NUM-5).
    if (!result.acceptable) {
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
