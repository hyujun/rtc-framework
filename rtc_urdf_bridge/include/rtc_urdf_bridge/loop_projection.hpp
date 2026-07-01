// ── loop_projection: q 를 loop-consistent configuration 으로 사영 ─────────────
#pragma once

#include "rtc_urdf_bridge/loop_verification.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/constraints.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include <vector>

namespace rtc_urdf_bridge {

/// @brief position-level projection 옵션 (damped least-squares Newton).
struct ProjectionOptions {
  double tolerance{1e-10};  // 수렴 판정 ‖φ‖ 임계
  int max_iterations{100};
  double damping{1e-8};  // λ (DLS regularization coefficient; λ² 로 사용)
};

/// @brief projection 결과.
struct ProjectionResult {
  Eigen::VectorXd q;        // 사영된 configuration
  bool converged{false};    // ‖φ‖ < tolerance 도달 여부
  int iterations{0};        // 소요 반복
  double final_error{0.0};  // 최종 ‖φ‖
};

/// @brief φ(q)=0 을 damped least-squares Newton 으로 풀어 q 를 loop-consistent 로 사영.
///   dq = −Jcᵀ (Jc Jcᵀ + λ²I)⁻¹ φ(q),  q ← integrate(model, q, dq) 반복.
/// 수렴 실패 시 converged=false 반환(마지막 q 유지). **RT 밖에서만 호출.**
[[nodiscard]] ProjectionResult ProjectToConstraint(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q_init,
    const ProjectionOptions& opts = {});

/// @brief velocity-level projection: v 를 Jc v = 0 을 만족하는 가장 가까운 벡터로 사영.
///   v_proj = v − Jcᵀ (Jc Jcᵀ + λ²I)⁻¹ Jc v. **RT 밖에서만 호출.**
[[nodiscard]] Eigen::VectorXd ProjectVelocity(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q,
    const Eigen::VectorXd& v, double damping = 1e-8);

}  // namespace rtc_urdf_bridge
