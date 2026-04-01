// ── RtModelHandle: RT-safe Pinocchio 래퍼 ────────────────────────────────────
#pragma once

#include "urdf_pinocchio_bridge/types.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/contact-info.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include <memory>
#include <span>
#include <string_view>
#include <vector>

namespace urdf_pinocchio_bridge
{

/// RT-safe Pinocchio wrapper.
///
/// - Model을 const 참조로 받음 (소유하지 않음, shared_ptr로 수명 보장)
/// - Data + 작업 버퍼를 자체 소유 (인스턴스별 독립)
/// - 모든 compute 함수는 noexcept — 힙 할당, IO, 예외, mutex 없음
/// - 복사 금지, 이동 허용
/// - thread-per-handle: 하나의 handle은 하나의 스레드에서만 사용
///
/// ### 사용 패턴
/// @code
///   // [non-RT] 초기화
///   auto model = builder.GetReducedModel("arm");
///   RtModelHandle handle(model);
///
///   // [RT 루프]
///   handle.ComputeForwardKinematics(q_span);
///   handle.ComputeJacobians(q_span);
///   handle.GetFrameJacobian(fid, pinocchio::LOCAL, J_out);
///   handle.ComputeNonLinearEffects(q_span, v_span);
/// @endcode
class RtModelHandle
{
public:
  /// @brief 생성자 (non-RT). 모든 버퍼 사전 할당.
  /// @param model Pinocchio Model (shared_ptr로 수명 보장)
  /// @param constraint_models 폐쇄 체인 구속 (빈 벡터면 구속 없음)
  explicit RtModelHandle(
    std::shared_ptr<const pinocchio::Model> model,
    std::vector<pinocchio::RigidConstraintModel> constraint_models = {});

  // 복사 금지, 이동 허용
  RtModelHandle(const RtModelHandle &) = delete;
  RtModelHandle & operator=(const RtModelHandle &) = delete;
  RtModelHandle(RtModelHandle &&) noexcept = default;
  RtModelHandle & operator=(RtModelHandle &&) noexcept = default;

  ~RtModelHandle() = default;

  // ── RT-safe compute 함수 ───────────────────────────────────────────────────
  // 모두 noexcept. 힙 할당 없음. 사전 할당 버퍼에 결과 기록.

  /// FK: q → oMi, oMf 갱신
  void ComputeForwardKinematics(std::span<const double> q) noexcept;

  /// FK + 속도: (q, v) → oMi, oMf, v 갱신
  void ComputeForwardKinematics(
    std::span<const double> q, std::span<const double> v) noexcept;

  /// 전체 관절 자코비안 계산: q → data_.J, oMf 갱신 포함
  void ComputeJacobians(std::span<const double> q) noexcept;

  /// 특정 프레임 자코비안 추출 (ComputeJacobians 후 호출)
  /// @param frame_id 프레임 인덱스
  /// @param ref_frame LOCAL / WORLD / LOCAL_WORLD_ALIGNED
  /// @param J_out 6 x nv pre-allocated matrix
  void GetFrameJacobian(
    pinocchio::FrameIndex frame_id,
    pinocchio::ReferenceFrame ref_frame,
    Eigen::Ref<Eigen::MatrixXd> J_out) noexcept;

  /// RNEA 역동역학: τ = M(q)·a + C(q,v)·v + g(q)
  void ComputeInverseDynamics(
    std::span<const double> q,
    std::span<const double> v,
    std::span<const double> a) noexcept;

  /// ABA 순동역학: ddq = M⁻¹(τ - C·v - g)
  void ComputeForwardDynamics(
    std::span<const double> q,
    std::span<const double> v,
    std::span<const double> tau) noexcept;

  /// 비선형 효과: nle = C(q,v)·v + g(q)
  void ComputeNonLinearEffects(
    std::span<const double> q,
    std::span<const double> v) noexcept;

  /// 일반화 중력 벡터: g(q)
  void ComputeGeneralizedGravity(std::span<const double> q) noexcept;

  /// 코리올리 행렬: C(q, v) — C(q,v)·v 는 GetCoriolisMatrix() * v 로 계산
  void ComputeCoriolisMatrix(
    std::span<const double> q,
    std::span<const double> v) noexcept;

  /// 질량 행렬: M(q)
  void ComputeMassMatrix(std::span<const double> q) noexcept;

  /// 구속 동역학 (폐쇄 체인)
  void ComputeConstraintDynamics(
    std::span<const double> q,
    std::span<const double> v,
    std::span<const double> tau) noexcept;

  // ── 결과 접근 (compute 호출 이후 유효) ─────────────────────────────────────

  /// 프레임 SE3 (위치 + 회전). ComputeForwardKinematics 후 유효.
  [[nodiscard]] const pinocchio::SE3 & GetFramePlacement(
    pinocchio::FrameIndex frame_id) const noexcept;

  /// 프레임 위치 (world frame)
  [[nodiscard]] Eigen::Vector3d GetFramePosition(
    pinocchio::FrameIndex frame_id) const noexcept;

  /// 프레임 회전 행렬
  [[nodiscard]] Eigen::Matrix3d GetFrameRotation(
    pinocchio::FrameIndex frame_id) const noexcept;

  /// RNEA 결과 토크 벡터
  [[nodiscard]] Eigen::Ref<const Eigen::VectorXd> GetTau() const noexcept;

  /// ABA 결과 가속도 벡터
  [[nodiscard]] Eigen::Ref<const Eigen::VectorXd> GetDdq() const noexcept;

  /// 비선형 효과 벡터
  [[nodiscard]] Eigen::Ref<const Eigen::VectorXd> GetNonLinearEffects() const noexcept;

  /// 일반화 중력 벡터
  [[nodiscard]] Eigen::Ref<const Eigen::VectorXd> GetGeneralizedGravity() const noexcept;

  /// 코리올리 행렬 C(q, v) — nv × nv
  [[nodiscard]] Eigen::Ref<const Eigen::MatrixXd> GetCoriolisMatrix() const noexcept;

  /// 질량 행렬 (upper triangular → 대칭화 필요)
  [[nodiscard]] Eigen::Ref<const Eigen::MatrixXd> GetMassMatrix() const noexcept;

  // ── 모델 메타데이터 ────────────────────────────────────────────────────────

  [[nodiscard]] int nq() const noexcept;
  [[nodiscard]] int nv() const noexcept;

  /// 프레임 이름 → FrameIndex. 없으면 0 반환 (universe).
  [[nodiscard]] pinocchio::FrameIndex GetFrameId(
    std::string_view frame_name) const noexcept;

  /// 내부 Model const 참조
  [[nodiscard]] const pinocchio::Model & GetModel() const noexcept;

  /// 내부 Data const 참조 (디버깅용)
  [[nodiscard]] const pinocchio::Data & GetData() const noexcept;

  /// mimic 관절 위치 계산: q_mimic = multiplier * q_mimicked + offset
  [[nodiscard]] static double ComputeMimicPosition(
    double mimicked_q, double multiplier, double offset) noexcept;

private:
  /// std::span → Eigen::VectorXd 복사 (noexcept, 인라인)
  void CopyToEigen(std::span<const double> src, Eigen::VectorXd & dst) noexcept;

  // ── 내부 데이터 ────────────────────────────────────────────────────────────
  std::shared_ptr<const pinocchio::Model> model_;
  pinocchio::Data data_;

  // 사전 할당 작업 버퍼
  Eigen::VectorXd q_;      // nq
  Eigen::VectorXd v_;      // nv
  Eigen::VectorXd a_;      // nv
  Eigen::VectorXd tau_;    // nv
  Eigen::MatrixXd J_;      // 6 x nv

  // 폐쇄 체인 구속
  std::vector<pinocchio::RigidConstraintModel> constraint_models_;
  std::vector<pinocchio::RigidConstraintData> constraint_datas_;
};

}  // namespace urdf_pinocchio_bridge
