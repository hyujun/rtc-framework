// ── RtModelHandle: RT-safe Pinocchio 래퍼 ────────────────────────────────────
#pragma once

#include "rtc_urdf_bridge/types.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/constraints.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rtc_urdf_bridge {

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
class RtModelHandle {
 public:
  /// @brief 생성자 (non-RT). 모든 버퍼 사전 할당.
  /// @param model Pinocchio Model (shared_ptr로 수명 보장)
  /// @param constraint_models 폐쇄 체인 구속 (빈 벡터면 구속 없음)
  explicit RtModelHandle(std::shared_ptr<const pinocchio::Model> model,
                         std::vector<pinocchio::RigidConstraintModel> constraint_models = {});

  // 복사 금지, 이동 허용
  RtModelHandle(const RtModelHandle&) = delete;
  RtModelHandle& operator=(const RtModelHandle&) = delete;
  RtModelHandle(RtModelHandle&&) noexcept = default;
  RtModelHandle& operator=(RtModelHandle&&) noexcept = default;

  ~RtModelHandle() = default;

  // ── RT-safe compute 함수 ───────────────────────────────────────────────────
  // 모두 noexcept. 힙 할당 없음. 사전 할당 버퍼에 결과 기록.

  /// FK: q → oMi, oMf 갱신
  void ComputeForwardKinematics(std::span<const double> q) noexcept;

  /// FK + 속도: (q, v) → oMi, oMf, v 갱신
  void ComputeForwardKinematics(std::span<const double> q, std::span<const double> v) noexcept;

  /// 전체 관절 자코비안 계산: q → data_.J, oMf 갱신 포함
  void ComputeJacobians(std::span<const double> q) noexcept;

  /// 특정 프레임 자코비안 추출 (ComputeJacobians 후 호출)
  /// @param frame_id 프레임 인덱스
  /// @param ref_frame LOCAL / WORLD / LOCAL_WORLD_ALIGNED
  /// @param J_out 6 x nv pre-allocated matrix
  void GetFrameJacobian(pinocchio::FrameIndex frame_id, pinocchio::ReferenceFrame ref_frame,
                        Eigen::Ref<Eigen::MatrixXd> J_out) noexcept;

  /// RNEA 역동역학: τ = M(q)·a + C(q,v)·v + g(q)
  void ComputeInverseDynamics(std::span<const double> q, std::span<const double> v,
                              std::span<const double> a) noexcept;

  /// ABA 순동역학: ddq = M⁻¹(τ - C·v - g)
  void ComputeForwardDynamics(std::span<const double> q, std::span<const double> v,
                              std::span<const double> tau) noexcept;

  /// 비선형 효과: nle = C(q,v)·v + g(q)
  void ComputeNonLinearEffects(std::span<const double> q, std::span<const double> v) noexcept;

  /// 일반화 중력 벡터: g(q)
  void ComputeGeneralizedGravity(std::span<const double> q) noexcept;

  /// 코리올리 행렬: C(q, v) — C(q,v)·v 는 GetCoriolisMatrix() * v 로 계산
  void ComputeCoriolisMatrix(std::span<const double> q, std::span<const double> v) noexcept;

  /// 질량 행렬: M(q)
  void ComputeMassMatrix(std::span<const double> q) noexcept;

  /// Payload 관성 회귀자: Y_L = J_F(LOCAL)ᵀ · frameBodyRegressor(F) — nv × 10
  ///
  /// 프레임 `frame_id` 에 강체로 매달린 payload 의 10-parameter 관성 집합 φ_L 에 대해
  /// τ_payload = Y_L · φ_L 을 만족한다 (#455 Layer 2B, `[MASS-B0]`).
  ///
  /// **열 순서는 `pinocchio::Inertia::toDynamicParameters()` 와 동일하다** —
  ///   `[m, m·c_x, m·c_y, m·c_z, I_xx, I_xy, I_yy, I_xz, I_yz, I_zz]`
  /// lower-triangular column-major 이므로 `I_xz` 가 `I_yy` **뒤**에 온다. 그리고 그 `I` 는
  /// 프레임 ORIGIN 기준이지 CoM 기준이 아니다 (평행축 항이 이미 더해져 있다). 둘 중
  /// 어느 쪽을 착각해도 유한하고 매끄러운 틀린 답이 나오므로 test_payload_regressor.cpp
  /// 가 원소별로 고정한다.
  ///
  /// **행은 PINOCCHIO v-공간 순서다** (GetTau() 와 동일). device 순서가 필요하면 호출자가
  /// ReorderOutput 한다. q/v/a 입력은 반대로 외부(device) 순서로 받아 내부에서 재배열한다.
  ///
  /// quasi-static (v=a=0) 에서 **`I` 6열은 정확히 0 이 된다** — 중력만으로는 회전 관성이
  /// 관측되지 않는다 (#455 [C2]: 60자세를 쌓아도 rank 는 10 이 아니라 4). 그 자세에서
  /// 나오는 어떤 `Î` 도 regularization 이 만든 숫자이므로 추정 대상이 아니다.
  ///
  /// @param q,v,a 외부(device) 순서 관절 상태
  /// @param frame_id payload 가 부착된 프레임
  void ComputePayloadRegressor(std::span<const double> q, std::span<const double> v,
                               std::span<const double> a, pinocchio::FrameIndex frame_id) noexcept;

  /// 구속 동역학 (폐쇄 체인)
  void ComputeConstraintDynamics(std::span<const double> q, std::span<const double> v,
                                 std::span<const double> tau) noexcept;

  // ── 결과 접근 (compute 호출 이후 유효) ─────────────────────────────────────

  /// 프레임 SE3 (위치 + 회전). ComputeForwardKinematics 후 유효.
  [[nodiscard]] const pinocchio::SE3& GetFramePlacement(
      pinocchio::FrameIndex frame_id) const noexcept;

  /// 프레임 위치 (world frame)
  [[nodiscard]] Eigen::Vector3d GetFramePosition(pinocchio::FrameIndex frame_id) const noexcept;

  /// 프레임 회전 행렬
  [[nodiscard]] Eigen::Matrix3d GetFrameRotation(pinocchio::FrameIndex frame_id) const noexcept;

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

  /// Payload 관성 회귀자 Y_L — nv × 10. ComputePayloadRegressor 후 유효.
  [[nodiscard]] Eigen::Ref<const Eigen::MatrixXd> GetPayloadRegressor() const noexcept;

  /// 질량 행렬 (upper triangular → 대칭화 필요)
  [[nodiscard]] Eigen::Ref<const Eigen::MatrixXd> GetMassMatrix() const noexcept;

  // ── 모델 메타데이터 ────────────────────────────────────────────────────────

  [[nodiscard]] int nq() const noexcept;
  [[nodiscard]] int nv() const noexcept;

  /// 프레임 이름 → FrameIndex. 없으면 0 반환 (universe).
  [[nodiscard]] pinocchio::FrameIndex GetFrameId(std::string_view frame_name) const noexcept;

  /// 내부 Model const 참조
  [[nodiscard]] const pinocchio::Model& GetModel() const noexcept;

  /// 내부 Data const 참조 (디버깅용)
  [[nodiscard]] const pinocchio::Data& GetData() const noexcept;

  /// mimic 관절 위치 계산: q_mimic = multiplier * q_mimicked + offset
  [[nodiscard]] static double ComputeMimicPosition(double mimicked_q, double multiplier,
                                                   double offset) noexcept;

  // ── 관절 순서 재배열 (Non-RT 설정) ────────────────────────────────────────

  /// 외부 관절 이름 순서 설정 (예: YAML joint_state_names).
  /// 설정 후 모든 compute 함수에 전달되는 q/v/a/tau는 이 순서로 해석됨.
  /// @param external_joint_names 호출자 측 관절 이름 순서
  /// @return true 모든 이름을 모델에서 찾은 경우; false 하나라도 없으면 (매핑 미설정)
  /// @note Non-RT. init 시 1회 호출. thread-per-handle 전제.
  [[nodiscard]] bool SetJointOrder(std::span<const std::string> external_joint_names);

  /// reorder 매핑 활성 여부
  [[nodiscard]] bool HasJointReorder() const noexcept;

  /// Pinocchio 내부 관절 이름 순서 반환 (디버깅용).
  [[nodiscard]] std::vector<std::string> GetPinocchioJointNames() const;

  /// Pinocchio 순서 벡터 → 외부 순서 span으로 재배열 (출력용).
  /// @param pinocchio_vec Pinocchio v-space 벡터 (GetTau, GetDdq 등)
  /// @param external_out 외부 순서 출력 버퍼 (크기 >= v_reorder_map_ 크기)
  void ReorderOutput(Eigen::Ref<const Eigen::VectorXd> pinocchio_vec,
                     std::span<double> external_out) const noexcept;

  /// 외부(device) 순서 v-공간 벡터 → Pinocchio 순서 Eigen 벡터로 재배열 (입력용).
  /// ReorderOutput 의 역방향(scatter). reorder 매핑이 없으면 memcpy fallback 이라
  /// identity 순서에서 zero-overhead. Coriolis·null-space 처럼 device 순서로 형성한
  /// 항(safe_position, null_target, q, v 의 선형결합)을 모델(Pinocchio) 순서 투영/
  /// 행렬과 곱하기 직전에 1회 gather 하는 데 쓴다.
  /// @param external_in 외부(device) 순서 입력 (길이 >= v_reorder_map_ 크기)
  /// @param pinocchio_out Pinocchio v-공간 출력 버퍼 (크기 nv, 연속 메모리)
  /// @note nq==nv 고정베이스 매니퓰레이터 전제 (컨트롤러 전역 가정과 동일).
  void ReorderInput(std::span<const double> external_in,
                    Eigen::VectorXd& pinocchio_out) const noexcept;

 private:
  /// std::span → Eigen::VectorXd 직접 복사 (noexcept, memcpy)
  void CopyToEigen(std::span<const double> src, Eigen::VectorXd& dst) noexcept;

  /// span → Eigen 복사 (reorder_map이 비어있으면 CopyToEigen fallback).
  /// reorder_map[i] = Pinocchio 벡터 내 대상 인덱스.
  void CopyToEigenReordered(std::span<const double> src, Eigen::VectorXd& dst,
                            const std::vector<int>& reorder_map) noexcept;

  // ── 내부 데이터 ────────────────────────────────────────────────────────────
  std::shared_ptr<const pinocchio::Model> model_;
  pinocchio::Data data_;

  // 사전 할당 작업 버퍼
  Eigen::VectorXd q_;    // nq
  Eigen::VectorXd v_;    // nv
  Eigen::VectorXd a_;    // nv
  Eigen::VectorXd tau_;  // nv
  Eigen::MatrixXd J_;    // 6 x nv
  Eigen::MatrixXd payload_regressor_;  // nv x 10 (Layer 2B, #455)

  // 폐쇄 체인 구속
  std::vector<pinocchio::RigidConstraintModel> constraint_models_;
  std::vector<pinocchio::RigidConstraintData> constraint_datas_;

  // ── 관절 순서 재배열 매핑 (비어있으면 비활성) ──────────────────────────────
  std::vector<int> q_reorder_map_;  ///< external[i] → Pinocchio q index
  std::vector<int> v_reorder_map_;  ///< external[i] → Pinocchio v index
};

}  // namespace rtc_urdf_bridge
