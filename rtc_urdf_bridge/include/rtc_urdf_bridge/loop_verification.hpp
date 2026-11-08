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

#include <limits>
#include <span>
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

/// @brief constraint Jacobian 의 rank 리포트 — 서로 다른 두 결손을 구분한다.
///   · **구속 중복** (행 종속, `redundant_rows > 0`): 평면 loop 에 contact_3d 를 걸었을 때의
///     축방향 항등-0 행처럼 자세와 무관한 구조적 중복. Delassus 기반 solver (constraintDynamics
///     plain overload) 만 특이해지고, 축약 핸들 (Jc_D 좌-역) 에는 영향이 없다.
///   · **조립 특이** (열 결손, `reduction_singular`): loop-passive 열 Jc_D (@ref
///     LoopPassiveVelocityIndices) 가 column rank 를 잃어 passive 속도가 actuated 로 결정되지
///     않는 자세. @ref ClosedChainHandle / @ref RtClosedChainHandle 의 `Status::singular` 와 같은
///     임계 (@ref kClosedChainSingularSvThreshold). actuated 를 받는 overload 만 채운다.
struct JacobianReport {
  int rows{0};                          // Σ dim
  int rank{0};                          // rank(Jc) — SVD, σ > kJacobianRankRelTol · σ_max
  bool full_rank{false};                // rank == rows
  int redundant_rows{0};                // rows − rank (구속 중복 행 수)
  double delassus_condition{0.0};       // cond(Jc M⁻¹ Jcᵀ); 특이 시 +inf
  double smallest_singular_value{0.0};  // σ_min(Jc) (행 기준 특이형상 근접 지표)
  int passive_cols{0};                  // loop-passive 열 수 (actuated 미지정이면 0)
  // σ_min(Jc_D), loop-passive 열만. actuated 미지정 또는 passive 없음 → +inf (판정 안 함);
  // passive_cols > rows → 0 (column rank 결손이 형상 무관으로 성립 — actuated 가 부족하다).
  double sigma_min_reduction{std::numeric_limits<double>::infinity()};
  bool reduction_singular{false};  // sigma_min_reduction < kClosedChainSingularSvThreshold
};

/// @brief Jc 행 rank 의 SVD 상대 임계 (σ > kJacobianRankRelTol · σ_max 를 rank 로). Eigen
/// FullPivLU 의 기본 임계 (ε · diagonalSize) 는 반올림 크기의 null 행을 rank 로 세어 다중 평면
/// loop 의 중복 수를 잘못 보고했다. 조립 특이 판정은 @ref kClosedChainSingularSvThreshold.
inline constexpr double kJacobianRankRelTol = 1e-9;

/// @brief loop-passive velocity 인덱스 (오름차순): 어떤 constraint 의 두 끝 frame 사이 경로
///   (두 parent joint 의 support 대칭차) 위에 있는 movable 관절 중 @p actuated_joint_ids 에 없는
///   것. 두 끝의 **공통 조상** (floating root, loop 상류의 arm 관절) 은 Jc 열이 항등 0 이라 loop 가
///   결정하는 DoF 가 아니므로 제외한다.
///   ⚠ @ref ClosedChainHandle::Initialize 의 종속 집합 ("모든 movable ∧ ¬actuated") 보다 좁다 —
///   핸들은 loop 밖 비구동 관절이 있으면 dep > m 으로 생성을 거부하므로 두 정의가 갈리는 모델은
///   핸들로 만들 수 없고, 빌더 진단은 그 모델 (병합 arm+hand · floating base) 도 판정해야 한다.
[[nodiscard]] std::vector<int> LoopPassiveVelocityIndices(
    const pinocchio::Model& model, const std::vector<pinocchio::RigidConstraintModel>& constraints,
    std::span<const pinocchio::JointIndex> actuated_joint_ids);

/// @brief Jc 행 rank + Delassus 조건수. 열 판정 (passive_cols · reduction_*) 은 채우지 않는다
///   (actuated 를 모르면 passive 집합을 정의할 수 없다) — 아래 overload 에 빈 actuated 로 위임.
/// @note 로드 타임 전용.
[[nodiscard]] JacobianReport AnalyzeConstraintJacobian(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q);

/// @brief 위 + loop-passive 열 Jc_D 의 σ_min 으로 조립 특이 판정 (@ref LoopPassiveVelocityIndices).
///   @p actuated_joint_ids 가 비면 열 판정 필드는 기본값 (0 / +inf / false) 으로 남는다.
/// @note 로드 타임 전용.
[[nodiscard]] JacobianReport AnalyzeConstraintJacobian(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q,
    std::span<const pinocchio::JointIndex> actuated_joint_ids);

/// @brief 모든 constraint dim 합 (CONTACT_6D→6, else 3).
[[nodiscard]] int TotalConstraintDim(
    const std::vector<pinocchio::RigidConstraintModel>& constraints) noexcept;

/// @brief 단일 constraint 의 residual 차원 (CONTACT_6D→6, else 3). RT/non-RT 공용.
[[nodiscard]] int ConstraintDim(const pinocchio::RigidConstraintModel& cm) noexcept;

/// @brief frame @p frame_id 가 loop-passive(movable & non-actuated) 관절의 하류인가.
///   frame → 부모 joint → universe 의 support(조상) 경로에 movable non-actuated 관절이 하나라도
///   있으면 true. @p frame_id 가 model.frames 범위 밖이면 false. **RT/non-RT 공용 pure 함수**
///   (topology-driven wiring). @ref ClosedChainHandle 과 @ref RtClosedChainHandle 이 공유한다 —
///   두 곳에 복제하면 loop-membership 규칙이 한쪽만 바뀌어 closed FK 활성화가 조용히 desync 된다.
[[nodiscard]] bool IsFrameDownstreamOfLoop(
    const pinocchio::Model& model, std::span<const pinocchio::JointIndex> actuated_joint_ids,
    pinocchio::FrameIndex frame_id) noexcept;

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
