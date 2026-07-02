// ── ClosedChainHandle: closed-chain 로봇의 constraint-consistent(축약) 동역학 질의 ──
#pragma once

#include "rtc_urdf_bridge/loop_projection.hpp"

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

#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rtc_urdf_bridge {

class PinocchioModelBuilder;

/// closed-chain 로봇의 동역학을 **serial chain 과 동일한 인터페이스**로 질의하는 핸들.
///
/// 동기: 폐쇄 체인은 spanning-tree 모델(nv 좌표) 위에서 m 개의 loop 구속을 가지므로,
/// 실제 자유도는 독립(independent) 좌표 n_a = nv − m 개다. 이 핸들은 호출자가 넘긴
/// **actuated(독립) 관절 좌표 q_a** 로부터 passive(종속) DoF 를 구속에 사영해 복원하고
/// (closed-loop FK), 독립 좌표 기준의 **축약된** 관성 M_a·중력 g_a·비선형효과 h_a·
/// 프레임 Jacobian J_a·FK 를 돌려준다. 구속이 없는 serial 로봇이면 축약이 항등이 되어
/// 값이 full 모델과 동일해진다 ("동일 인터페이스" 를 형태로 보장).
///
/// ### RT 계약 (RtModelHandle 과의 차이)
/// 이 핸들은 **non-RT** 다. `Update()` 는 damped-least-squares Newton projection,
/// Jc 분할·역행렬, null-space 조립을 수행하므로 heap 할당·반복이 발생한다. RT 핫패스에서
/// 매 tick 호출하지 말 것 — init / 저주기 query / off-RT 컨트롤러 준비용이다. RT-safe 한
/// 개방 체인 kinematics/dynamics 는 @ref RtModelHandle 을 쓴다.
///
/// ### 축약 동역학 정의
/// - 속도 map `G` (nv × n_a): `Jc·G = 0`, 독립 블록 = 항등 → `v_full = G·v_a`.
/// - `M_a = Gᵀ·M·G`,  `g_a = Gᵀ·g`,  `J_a(frame) = J_full(frame)·G`.
/// - `h_a = Gᵀ·rnea(q, v_full, a_drift)` — 여기서 `a_drift` 는 `a_I=0` 일 때 구속-정합
///   가속(종속 성분 `= −Jc_D⁻¹·γ`, `γ = J̇c·v_full` 은 동일 Jc 함수의 유한차분으로 산출해
///   규약 정합을 보장). `h_a = C_a·v_a + g_a` (비선형 bias) 이며 v=0 이면 `h_a = g_a`.
///
/// ### 사용 패턴
/// @code
///   PinocchioModelBuilder builder(config);           // closure_yaml_path 설정
///   ClosedChainHandle handle(builder);
///   handle.Update(q_a_span, v_a_span);               // [non-RT]
///   const auto& M = handle.GetMassMatrix();           // n_a × n_a
///   const auto& g = handle.GetGeneralizedGravity();   // n_a
///   handle.GetFrameJacobian(fid, pinocchio::LOCAL_WORLD_ALIGNED, J_out);  // 6 × n_a
/// @endcode
class ClosedChainHandle {
 public:
  /// @brief `Update()` 결과 상태 (loop-consistency / 특이성).
  ///
  /// @warning `singular==true` 이면 M_a/g_a/h_a/J_a 는 damped pseudo-inverse 결과로
  ///   **유한하지만 신뢰 불가** (특이 조립형상에서 증폭 — damping λ 로 상계는 두지만 물리적
  ///   부정확). getter 는 이 값을 그대로 돌려주므로 **소비자는 쓰기 전 `GetStatus().singular`
  ///   를 반드시 확인**하고 true 면 해당 tick 을 버리거나 직전 유효 해를 hold 해야 한다
  ///   (inverse-dynamics 로 직결 금지). `converged==false` (사영 미수렴) 도 동일 취급 —
  ///   이 경우 q_full 은 직전 해를 hold 한다.
  struct Status {
    bool converged{false};  ///< passive 사영 ‖φ‖ < tol 도달
    bool singular{false};  ///< Jc rank 결손 (특이 조립형상) — 축약값 damped, 신뢰도 저하
    double closure_error{0.0};  ///< 사영 후 최종 ‖φ‖
    int iterations{0};          ///< 사영 반복 수
  };

  /// @brief Extended-URDF closure builder 로부터 구성 (권장 경로).
  ///   full model + constraints + actuated_joint_ids + q_ref(warm-start seed) 를 인출한다.
  ///   builder 에 closure 가 없으면(plain URDF) 구속 0 → 항등 축약(serial 등가).
  /// @throws std::runtime_error 독립 좌표 수 ≠ nv − m (well-posed 아님) 또는 독립 관절이
  ///   단일-DoF(nv==1) 가 아닐 때.
  explicit ClosedChainHandle(const PinocchioModelBuilder& builder);

  /// @brief 명시적 구성요소로 구성 (테스트 / builder 외 경로).
  /// @param model full spanning-tree 모델 (shared_ptr 로 수명 보장)
  /// @param constraints loop 구속 (빈 벡터면 serial 등가 항등 축약)
  /// @param actuated_joint_ids 독립(구동) 관절 JointIndex — 각 단일-DoF
  /// @param q_ref loop-consistent seed (빈 벡터면 neutral)
  ClosedChainHandle(std::shared_ptr<const pinocchio::Model> model,
                    std::vector<pinocchio::RigidConstraintModel> constraints,
                    std::vector<pinocchio::JointIndex> actuated_joint_ids,
                    Eigen::VectorXd q_ref = {});

  // 복사 금지, 이동 허용
  ClosedChainHandle(const ClosedChainHandle&) = delete;
  ClosedChainHandle& operator=(const ClosedChainHandle&) = delete;
  ClosedChainHandle(ClosedChainHandle&&) = default;
  ClosedChainHandle& operator=(ClosedChainHandle&&) = default;
  ~ClosedChainHandle() = default;

  // ── 갱신 (non-RT) ───────────────────────────────────────────────────────────

  /// @brief actuated(독립) 좌표로 passive DoF 사영 후 축약 M/g/h/J/FK 를 갱신한다.
  /// @param q_a 독립 관절 위치 (GetIndependentJointNames() 순서, 크기 n_a)
  /// @param v_a 독립 관절 속도 (선택; 비우면 0 → h_a = g_a, Jacobian/FK 만 유효)
  /// @return 사영 수렴/특이성 상태. 미수렴 시 직전 해를 hold 한다 (NaN 누출 방지).
  /// @note **non-RT.** 반복·할당 포함.
  Status Update(std::span<const double> q_a, std::span<const double> v_a = {});

  // ── 축약 결과 접근 (Update 이후 유효, 독립 좌표 n_a 기준) ────────────────────

  /// 축약 질량행렬 M_a = Gᵀ M G (n_a × n_a, 대칭 SPD)
  [[nodiscard]] Eigen::Ref<const Eigen::MatrixXd> GetMassMatrix() const noexcept;

  /// 축약 일반화 중력 g_a = Gᵀ g (n_a)
  [[nodiscard]] Eigen::Ref<const Eigen::VectorXd> GetGeneralizedGravity() const noexcept;

  /// 축약 비선형효과 h_a = C_a v_a + g_a (n_a). v_a=0 이면 g_a 와 동일.
  [[nodiscard]] Eigen::Ref<const Eigen::VectorXd> GetNonLinearEffects() const noexcept;

  /// 축약 프레임 Jacobian J_a = J_full(frame)·G (6 × n_a) 를 J_out 에 기록.
  /// @param frame_id 프레임 인덱스 (full 모델 기준; GetFrameId 로 조회)
  /// @param ref_frame LOCAL / WORLD / LOCAL_WORLD_ALIGNED
  /// @param J_out 6 × n_a pre-allocated
  void GetFrameJacobian(pinocchio::FrameIndex frame_id, pinocchio::ReferenceFrame ref_frame,
                        Eigen::Ref<Eigen::MatrixXd> J_out);

  /// 속도 축약 map G (nv × n_a): v_full = G v_a. 고급 소비자용.
  [[nodiscard]] Eigen::Ref<const Eigen::MatrixXd> GetReductionMap() const noexcept;

  // ── FK 결과 (loop-consistent full q 기준, full 모델 frame 인덱스) ─────────────

  [[nodiscard]] const pinocchio::SE3& GetFramePlacement(
      pinocchio::FrameIndex frame_id) const noexcept;
  [[nodiscard]] Eigen::Vector3d GetFramePosition(pinocchio::FrameIndex frame_id) const noexcept;
  [[nodiscard]] Eigen::Matrix3d GetFrameRotation(pinocchio::FrameIndex frame_id) const noexcept;

  /// 사영된 loop-consistent 전체 configuration (nq). 소비자가 passive q 도 필요할 때.
  [[nodiscard]] const Eigen::VectorXd& GetFullConfiguration() const noexcept;

  // ── 메타데이터 ──────────────────────────────────────────────────────────────

  /// 독립(구동) 좌표 수 n_a = nv − m.
  [[nodiscard]] int nv_independent() const noexcept;

  /// full spanning-tree 모델 nv / nq.
  [[nodiscard]] int nv_full() const noexcept;
  [[nodiscard]] int nq_full() const noexcept;

  /// 총 구속 차원 m.
  [[nodiscard]] int constraint_dim() const noexcept;

  /// 독립 관절 이름 (q_a / v_a 입력 순서 = velocity 인덱스 오름차순).
  [[nodiscard]] std::vector<std::string> GetIndependentJointNames() const;

  /// 프레임 이름 → FrameIndex. 없으면 0 (universe).
  [[nodiscard]] pinocchio::FrameIndex GetFrameId(std::string_view frame_name) const noexcept;

  /// 내부 full 모델 const 참조.
  [[nodiscard]] const pinocchio::Model& GetModel() const noexcept;

  /// 직전 Update 상태.
  [[nodiscard]] const Status& GetStatus() const noexcept { return status_; }

 private:
  /// 독립 관절의 좌표 슬롯 (q_a → full q, v_a → full v 매핑용).
  struct IndependentSlot {
    int q_idx{0};               ///< full model q 시작 인덱스
    int v_idx{0};               ///< full model v 인덱스
    bool is_continuous{false};  ///< nq==2 (cos,sin) 관절 여부
    pinocchio::JointIndex jid{0};
  };

  /// 공통 초기화: 독립/종속 인덱스 분할, well-posed 검증, 버퍼 사전 할당.
  void Initialize();

  /// 현재 q_full_ 와 독립 속도 v_indep(n_a) 에서 G, v_full, M_a, g_a, h_a 를 재계산한다.
  void RebuildReducedDynamics(bool have_velocity, const Eigen::VectorXd& v_indep);

  std::shared_ptr<const pinocchio::Model> model_;
  pinocchio::Data data_;
  std::vector<pinocchio::RigidConstraintModel> constraints_;
  std::vector<pinocchio::JointIndex> actuated_joint_ids_;
  ProjectionOptions projection_opts_;

  int nv_{0};
  int m_{0};       ///< 총 구속 차원
  int n_a_{0};     ///< 독립 좌표 수 (= nv − m)
  bool identity_;  ///< 구속 없음 → 항등 축약 (serial 등가)

  std::vector<IndependentSlot> independent_;  ///< 독립 관절 슬롯 (v_idx 오름차순)
  std::vector<int> indep_v_idx_;              ///< 독립 velocity 인덱스 (오름차순)
  std::vector<int> dep_v_idx_;                ///< 종속 velocity 인덱스 (오름차순)

  // 상태 / 결과 버퍼
  Eigen::VectorXd q_full_;  ///< loop-consistent 전체 configuration (warm-start 유지)
  Eigen::VectorXd v_full_;  ///< G v_a
  Eigen::MatrixXd G_;       ///< nv × n_a 속도 축약 map
  Eigen::MatrixXd M_a_;     ///< n_a × n_a
  Eigen::VectorXd g_a_;     ///< n_a
  Eigen::VectorXd h_a_;     ///< n_a
  Eigen::MatrixXd J_full_;  ///< 6 × nv 작업 버퍼 (GetFrameJacobian)
  Status status_;
};

}  // namespace rtc_urdf_bridge
