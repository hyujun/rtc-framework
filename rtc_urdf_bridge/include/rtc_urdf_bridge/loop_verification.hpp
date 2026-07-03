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

/// @brief 단일 constraint 의 residual 차원 (CONTACT_6D→6, else 3). RT/non-RT 공용.
[[nodiscard]] int ConstraintDim(const pinocchio::RigidConstraintModel& cm) noexcept;

/// @brief @ref FillConstraintKinematicsRow 용 재사용 스크래치. RT 소비자는 멤버로 보유해
///   힙 할당을 회피한다. non-RT 소비자는 loop 밖에서 한 번 Resize 해 재사용한다.
struct ConstraintKinScratch {
  pinocchio::Data::Matrix6x J1, J2, Jc1, Jc2, Jrel;  ///< 6 × nv
  Eigen::Matrix<double, 3, Eigen::Dynamic> tmp3;     ///< 3 × nv (3D 커플링 항)
  /// nv 크기로 사전 할당 (non-RT — 생성/초기화 시 1회).
  void Resize(int nv);
};

/// @brief constraint @p cm 의 residual 을 @p phi.segment(row,dim) 에, Jacobian 을
///   @p Jc.middleRows(row,dim) 에 채운다. **호출 전 computeJointJacobians(model,data,q) 필수.**
///   @p scratch 와 phi/Jc 가 모두 사전 할당돼 있으면 힙 할당·예외 없음(noexcept) — 폐쇄 체인
///   constraint kinematics 규약(c1 LOCAL 상대 변위, 3D 는 −[p]×ω 커플링 포함)의 **단일 출처**.
///   @ref ComputeConstraintKinematics (non-RT) 와 RtClosedChainHandle (RT) 가 공유한다.
void FillConstraintKinematicsRow(const pinocchio::Model& model, pinocchio::Data& data,
                                 const pinocchio::RigidConstraintModel& cm, int row, int dim,
                                 ConstraintKinScratch& scratch, Eigen::Ref<Eigen::VectorXd> phi,
                                 Eigen::Ref<Eigen::MatrixXd> Jc) noexcept;

/// @brief 축약(reduction) Jacobian Jc_D 의 특이 조립형상 판정 임계 (σ_min(Jc_D) 대리).
/// non-RT @ref ClosedChainHandle (SVD σ_min) 과 RT @ref RtClosedChainHandle (LDLT pivot 대리)
/// 이 **동일 임계**를 공유해 singularity 의미를 동기화한다. 두 곳에 상수를 복제하면 한쪽만
/// 튜닝 시 semantics 가 조용히 desync 되므로 여기 단일 출처로 둔다.
inline constexpr double kClosedChainSingularSvThreshold = 1e-6;

}  // namespace rtc_urdf_bridge
