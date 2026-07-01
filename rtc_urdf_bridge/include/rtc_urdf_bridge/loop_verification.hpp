// ── loop_verification: closure error / constraint Jacobian rank·conditioning ──
#pragma once

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

#include <string>
#include <vector>

namespace rtc_urdf_bridge {

/// @brief 구속 전체의 residual φ(q) 와 Jacobian Jc(q). **로드 타임 전용.**
/// 좌표계 규약: 각 constraint 의 residual 은 c1 frame(LOCAL) 기준 c2 의 상대 변위.
///   6D → log6(c1Mc2) ([linear; angular], 6), 3D → c1Mc2.translation() (3).
/// dφ/dt = Jc · v (동일 규약) 이므로 projection·rank 분석에 그대로 사용 가능.
struct ConstraintKinematics {
  Eigen::VectorXd phi;           // (Σ dim)
  Eigen::MatrixXd Jc;            // (Σ dim) × nv
  std::vector<int> row_offsets;  // constraint i 의 시작 row
  std::vector<int> row_sizes;    // constraint i 의 dim (3 또는 6)
};

/// @brief 주어진 q 에서 φ 와 Jc 계산. computeJointJacobians(q) 를 내부 호출한다.
/// @note 로드 타임 전용 (heap 할당·예외 허용). RT 경로에서 호출 금지.
[[nodiscard]] ConstraintKinematics ComputeConstraintKinematics(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q);

/// @brief 개별 constraint 의 closure error norm.
struct ClosureError {
  std::string name;
  int dim{0};        // 3 또는 6
  double norm{0.0};  // 3D: translation norm, 6D: log6 norm
};

/// @brief 각 constraint 의 closure error (직접 forward-kinematics 계산).
[[nodiscard]] std::vector<ClosureError> ComputeClosureErrors(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q);

/// @brief constraint Jacobian rank / Delassus 조건수 리포트.
struct JacobianReport {
  int rows{0};                          // Σ dim
  int rank{0};                          // rank(Jc)
  bool full_rank{false};                // rank == rows
  double delassus_condition{0.0};       // cond(Jc M⁻¹ Jcᵀ); 특이 시 +inf
  double smallest_singular_value{0.0};  // Jc 의 최소 특이값 (특이형상 근접 지표)
};

/// @brief Jc rank 충족 + Delassus 조건수 계산. 특이형상(4-bar 정렬) 근처 경고.
/// @note 로드 타임 전용.
[[nodiscard]] JacobianReport AnalyzeConstraintJacobian(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q);

/// @brief 모든 constraint dim 합 (CONTACT_6D→6, else 3).
[[nodiscard]] int TotalConstraintDim(
    const std::vector<pinocchio::RigidConstraintModel>& constraints) noexcept;

}  // namespace rtc_urdf_bridge
