// ── RtClosedChainHandle: RT-safe closed-chain FK (fixed-step loop projection) ─
#pragma once

// loop_projection.hpp 는 선언 전용 — 사영 임계 상수(kDefaultActuatedIncrement /
// kDefaultMaxPassiveDeviation)를 non-RT 사영과 **한 곳에서** 공유하기 위해 포함한다. RT 경로가
// 그 함수들을 호출하지는 않는다 (여기 fixed-K 재구현이 대응).
#include "rtc_urdf_bridge/loop_projection.hpp"
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

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rtc_urdf_bridge {

/// closed-chain 로봇의 **loop-consistent FK 를 RT 핫패스에서** 얻는 핸들.
///
/// @ref ClosedChainHandle 은 동일 사영을 수렴 while-loop + SVD 로 정확히 풀지만 heap 할당·
/// 가변 반복이라 **non-RT** 다. 이 핸들은 그 FK 부분만을 RT-safe 로 재구현한다:
///
///   - **warm-start**: 직전 tick 의 loop-consistent full q 를 seed 로 재사용.
///   - **고정 K Newton iteration**: `ProjectPassiveToConstraint` 와 동일한 damped
///     least-squares 사영을 정확히 `num_iterations` 스텝만 수행 (수렴 판정 없음 →
///     deterministic). RT rate 에서 tick 당 actuated 이동이 작아 loop 이 거의 닫혀 있어
///     K=2 로 fully-converged FK 를 μm 수준으로 추종한다 (#121 Phase 2a 실측).
///   - **preallocated + in-place LDLT**: 모든 버퍼는 생성자에서 사전 할당, m×m / dep×dep
///     solve 는 미리 크기가 잡힌 `Eigen::LDLT` 로 in-place → 힙 할당·예외·mutex·로깅 없음.
///   - **NaN/비유한 guard → hold**: 사영 결과가 비유한이면 직전 유효 해를 유지하고
///     `Status.held=true` 를 돌린다 (NaN 누출 방지).
///
/// 축약 프레임 Jacobian `J_a = J_full(frame)·G` 도 제공한다. 속도 축약 map G 의 종속 블록은
/// non-RT 핸들의 SVD damped-pinv 대신 **damped 정규방정식 left-pinv**
/// `(Jc_Dᵀ Jc_D + λ²I)⁻¹ Jc_Dᵀ` (dep×dep LDLT) 로 RT-safe 하게 계산한다 — 생성자에서
/// 강제하는 well-posedness `dep ≤ m` 덕에 정의된다. 특이(조립형상 근접)는 LDLT 최소 pivot
/// 을 σ_min(Jc_D) 대리로 사용해 `Status.singular` 로 flag 한다.
///
/// ### RT 계약
/// - `Update()` / getter / `GetFrameJacobian()` 는 모두 `noexcept`, 힙 할당 없음, thread-per-
///   handle (하나의 handle 은 하나의 스레드에서만). 복사 금지, 이동 허용.
/// - **소비자는 매 tick `GetStatus()` 확인**: `held==true` 면 이번 결과를 버리고 직전 FK 를
///   쓴다 (핸들은 직전 해를 유지). `singular==true` 면 J_a 는 damped 라 신뢰 저하 —
///   task-space 사용을 hold 하거나 직전 유효 해 유지 권장.
///
/// ### 사용 패턴
/// @code
///   // [non-RT] init
///   RtClosedChainHandle handle(model, constraints, actuated_ids, q_seed);
///   const auto fid = handle.GetFrameId("c1");
///   // [RT tick]
///   const auto st = handle.Update(q_a_span);     // 고정 K=2 사영
///   if (!st.held) {
///     const pinocchio::SE3& X = handle.GetFramePlacement(fid);
///     if (!st.singular) handle.GetFrameJacobian(fid, pinocchio::LOCAL_WORLD_ALIGNED, J_out);
///   }
/// @endcode
class RtClosedChainHandle {
 public:
  /// @brief `Update()` 결과 상태 (loop-consistency / 특이성).
  struct Status {
    /// 이번 tick 결과를 버리고 직전 해를 쓰라는 신호. FK/J_a 는 직전 값. 세 경우에 선다:
    ///   (a) 사영 결과가 비유한(측정 NaN/Inf),
    ///   (b) actuated seed 증분이 `max_seed_increment` 를 넘어 **클램프**된 tick — 이 tick 의
    ///       actuated 슬롯은 측정값과 다르므로 FK 를 신뢰할 수 없다. 나머지 증분은 다음
    ///       tick 들에서 이어 가므로 `⌈Δ/증분⌉` tick 뒤 자연히 해제된다 (#248),
    ///   (c) 사영 후 passive 가 `max_passive_deviation` 을 넘게 이탈 (분기 이탈 의심).
    /// (b)·(c) 는 activate 직후 seed→측정 q 점프에서 연속 발생할 수 있다 — 소비자는 held 를
    /// **일시적 열화**로 다뤄야 하며 fault 로 승격하면 안 된다.
    bool held{false};
    /// Jc_D near-singular (LDLT pivot 대리). J_a 는 damped 라 신뢰 저하.
    bool singular{false};
    /// 고정 K 스텝 후 최종 closure residual ‖φ‖. 소비자가 임계로 hold 정책을 정할 수 있다.
    /// ⚠ CONTACT_6D 구속은 φ 가 [linear(m); angular(rad)] 혼합이라 ‖φ‖ 단위가 혼합된다 — m 단위
    /// 임계(예 ClosedChainHandFk 의 closure_error_threshold, 기본 1e-3 m)와 비교하면 회전 잔차가
    /// 가중 없이 섞인다. 현재 hand 링키지는 3D point 구속이라 무영향. 6D 구속 도입 시 병진/회전
    /// 분리 임계 또는 가중 norm 필요 (review #7, 현재 defer).
    double closure_error{0.0};
  };

  /// @brief 명시적 구성요소로 구성 (non-RT — 모든 버퍼 사전 할당·well-posedness 검증).
  /// @param model full spanning-tree 모델 (shared_ptr 로 수명 보장)
  /// @param constraints loop 구속 (빈 벡터면 serial 등가 항등 — 사영 없이 FK passthrough)
  /// @param actuated_joint_ids 독립(구동) 관절 JointIndex — 각 단일-DoF
  /// @param q_seed loop-consistent warm-start seed (빈 벡터면 neutral; 그 외 크기≠nq 면 throw)
  /// @param num_iterations 고정 Newton 스텝 수 K (기본 2, #121 Phase 2a 채택)
  /// @param projection_damping 사영 DLS 정규화 λ (기본 1e-6). λ²=1e-12 로 double rounding
  ///   floor(~2e-16) 위에서 dead-center 근접 시 유효 정규화. 더 작으면(예 1e-8→λ²=1e-16)
  ///   near-singular 에서 DLS 가 무력화된다 (review #6).
  /// @param reduction_damping G left-pinv 정규화 λ (기본 1e-6 = kClosedChainSingularSvThreshold).
  ///   **비-기본값 주의 (#120)**: non-RT `ClosedChainHandle` 은 reduction λ 를 특이 임계값에
  ///   하드코딩(`kReductionDamping = kSingularSvThreshold`)하므로, 여기에 1e-6 이 아닌 값을 넘기면
  ///   축약 M_a/g_a/h_a·J_a 가 non-RT ground truth 와 조용히 갈라지고 singular flag 는 여전히
  ///   임계값에 고정돼 λ 와 decouple 된다. 기본값을 유지하는 한 non-RT 와 수치 등가다.
  /// @param max_seed_increment tick 당 actuated seed 증분 상한 (기본
  ///   @ref kDefaultActuatedIncrement, ≤0 → 비활성). 초과분은 **증분 벡터 전체를 균일 스케일**해
  ///   잘라내고 그 tick 은 `held=true` 로 보고한다 (per-joint 클램프는 homotopy 경로를 꺾으므로
  ///   쓰지 않는다). 나머지는 다음 tick 들에서 이어 간다 — 즉 **tick loop 자체가 continuation
  ///   경로**가 되어, 고정 K 를 유지한 채(추가 연산 0·결정적) 조립 분기 이탈을 막는다.
  ///   비활성화하면 activate 직후 q_seed→측정 q 의 큰 점프가 반대편 분기로 착지할 수 있고,
  ///   뒤집힌 분기도 ‖φ‖≈0 이라 `closure_error` 임계로는 검출되지 않는다 (#248).
  /// @param max_passive_deviation K 스텝 후 passive q 가 이번 tick seed 대비 이만큼 넘게
  ///   움직이면 분기 이탈로 보고 `held=true` + q_full_ 미커밋 (기본
  ///   @ref kDefaultMaxPassiveDeviation, ≤0 → 비활성). seed clamp 를 통과했더라도 near-singular
  ///   조립형상에서 DLS 스텝이 폭주할 수 있어 남겨 두는 2차 가드다.
  /// @throws std::invalid_argument model 이 null, 또는 q_seed 가 비어있지 않은데 크기≠nq
  /// @throws std::runtime_error 독립 관절 non-single-DoF, 구속 有인데 독립 관절 無,
  ///   종속 DoF > 구속 rows m (reduction underdetermined)
  RtClosedChainHandle(std::shared_ptr<const pinocchio::Model> model,
                      std::vector<pinocchio::RigidConstraintModel> constraints,
                      std::vector<pinocchio::JointIndex> actuated_joint_ids,
                      Eigen::VectorXd q_seed = {}, int num_iterations = 2,
                      double projection_damping = 1e-6, double reduction_damping = 1e-6,
                      double max_seed_increment = kDefaultActuatedIncrement,
                      double max_passive_deviation = kDefaultMaxPassiveDeviation);

  // 복사 금지, 이동 허용
  RtClosedChainHandle(const RtClosedChainHandle&) = delete;
  RtClosedChainHandle& operator=(const RtClosedChainHandle&) = delete;
  RtClosedChainHandle(RtClosedChainHandle&&) = default;
  RtClosedChainHandle& operator=(RtClosedChainHandle&&) = default;
  ~RtClosedChainHandle() = default;

  // ── 갱신 (RT-safe) ──────────────────────────────────────────────────────────

  /// @brief 측정 actuated q 로 passive DoF 를 고정 K 스텝 사영 후 FK/G 를 갱신한다.
  /// @param q_a 독립 관절 위치 (GetIndependentJointNames() 순서, 크기 n_a)
  /// @return loop-consistency/특이성 상태. 비유한 시 직전 해 hold.
  /// @note **RT-safe.** noexcept, 힙 할당 없음. q_a 크기가 n_a 와 다르면 held=true 로 즉시
  ///   반환 (직전 해 유지) — RT 에서 throw 대신 hold.
  [[nodiscard]] Status Update(std::span<const double> q_a) noexcept;

  /// @brief 직전 `Update(q_a)` 형상에서 **축약 동역학** M_a/g_a/h_a 를 갱신한다.
  ///
  /// @ref ClosedChainHandle::RebuildReducedDynamics (`closed_chain_handle.cpp`) 의 수학을
  /// RT-safe 로 포팅한다:
  ///   - `M_a = Gᵀ·M·G` (crba 후 대칭화), `g_a = Gᵀ·g` (computeGeneralizedGravity),
  ///   - `h_a = Gᵀ·rnea(q, v_full, a_drift)`, `v_full = G·v_a`, `a_drift` 는 a_I=0 일 때
  ///     구속-정합 가속 (종속 = −Jc_D⁺·γ, γ = J̇c·v_full 는 Jc 중앙차분). v_a=0 이면 h_a=g_a.
  /// non-RT 핸들의 SVD damped-pinv 대신 직전 Update 가 factor 한 damped 정규방정식 LDLT
  /// (`ldlt_G_`) 와 Jc_D (`Jc_free_`) 를 재사용해 동일 결과를 힙 할당 없이 얻는다.
  ///
  /// @param v_a 독립 관절 속도 (n_a). 비우거나 크기≠n_a 면 v_full=0 → h_a=g_a. 비유한
  ///   (NaN/Inf) 성분이 있으면 `held=true` 로 직전 동역학·drift 를 유지한다 (`Update` 의
  ///   q allFinite guard 와 대칭 — NaN 이 M/h/g·dJv 로 누출되지 않는다).
  /// @return 직전 `Update` 상태를 그대로 반환 (비유한 v_a 는 held 로 격상). `held==true` 면
  ///   동역학 미갱신 (직전값 hold). `singular==true` 면 M_a/g_a/h_a 는 damped — 소비자 hold 정책.
  /// @note **RT-safe.** noexcept, 힙 할당 없음. **반드시 같은 tick 의 `Update(q_a)` 직후 호출** —
  ///   G/Jc_D/ldlt_G_ (q_full 형상) 를 재사용한다. drift 유한차분이 `data_` 를 오염시키므로
  ///   내부에서 2차 FK 상태(q_full, v_full, a_drift)로 복원 → 이후 `GetFrame*` getter 와
  ///   @ref GetFrameClassicalAccelerationDrift 는 계속 유효.
  [[nodiscard]] Status UpdateDynamics(std::span<const double> v_a = {}) noexcept;

  // ── 축약 동역학 결과 (UpdateDynamics 이후 유효, 독립 좌표 n_a 기준) ────────────

  /// 축약 질량행렬 M_a = Gᵀ M G (n_a × n_a, 대칭 SPD). **RT-safe.**
  [[nodiscard]] Eigen::Ref<const Eigen::MatrixXd> GetMassMatrix() const noexcept;

  /// 축약 일반화 중력 g_a = Gᵀ g (n_a). **RT-safe.**
  [[nodiscard]] Eigen::Ref<const Eigen::VectorXd> GetGeneralizedGravity() const noexcept;

  /// 축약 비선형효과 h_a = C_a v_a + g_a (n_a). v_a=0 이면 g_a. **RT-safe.**
  [[nodiscard]] Eigen::Ref<const Eigen::VectorXd> GetNonLinearEffects() const noexcept;

  // ── FK 결과 (loop-consistent full q 기준, full 모델 frame 인덱스) ─────────────

  [[nodiscard]] const pinocchio::SE3& GetFramePlacement(
      pinocchio::FrameIndex frame_id) const noexcept;
  [[nodiscard]] Eigen::Vector3d GetFramePosition(pinocchio::FrameIndex frame_id) const noexcept;
  [[nodiscard]] Eigen::Matrix3d GetFrameRotation(pinocchio::FrameIndex frame_id) const noexcept;

  /// 사영된 loop-consistent 전체 configuration (nq).
  [[nodiscard]] const Eigen::VectorXd& GetFullConfiguration() const noexcept;

  /// 축약 프레임 Jacobian J_a = J_full(frame)·G (6 × n_a) 를 J_out 에 기록.
  /// @param frame_id 프레임 인덱스 (full 모델 기준; GetFrameId 로 조회)
  /// @param ref_frame LOCAL / WORLD / LOCAL_WORLD_ALIGNED
  /// @param J_out 6 × n_a pre-allocated. **RT-safe.**
  void GetFrameJacobian(pinocchio::FrameIndex frame_id, pinocchio::ReferenceFrame ref_frame,
                        Eigen::Ref<Eigen::MatrixXd> J_out) noexcept;

  /// loop-consistent frame classical acceleration **drift** J̇_a·v_a (6×1) 를 @p a_out 에 기록.
  ///
  /// 직전 `UpdateDynamics(v_a)` 가 구한 구속-정합 drift 가속 a_drift(축약 좌표에서 a_indep=0
  /// 일 때의 full 가속 = Ġ·v_a)로 2차 FK 를 수행한 `data_` 에서 pinocchio classical
  /// acceleration 을 읽는다 — 반환값은 축약 Jacobian 의 drift `J̇_a·v_a` 다. open-chain
  /// `PinocchioCache` 의 dJv 와 동일 규약 (classical accel, 호출측이 ref_frame 선택).
  ///
  /// @param frame_id 프레임 인덱스 (full 모델 기준; GetFrameId 로 조회). 범위 밖 → 0 기록.
  /// @param ref_frame LOCAL / WORLD / LOCAL_WORLD_ALIGNED
  /// @param a_out 6×1 pre-allocated 출력. **RT-safe.**
  /// @note **같은 tick 의 `UpdateDynamics()` 이후에만 유효** (그 2차 FK 상태를 읽는다).
  ///   `Update()` 만 호출한 tick / held tick 은 직전 유효 tick 의 drift 가 반환된다 — 소비자는
  ///   `GetStatus()` 로 fresh 여부를 판정해 held/singular 시 last-good 정책을 적용한다.
  ///   v_a 미제공(`UpdateDynamics({})`) 이면 0 이 기록된다 (v=0 → drift 0).
  void GetFrameClassicalAccelerationDrift(
      pinocchio::FrameIndex frame_id, pinocchio::ReferenceFrame ref_frame,
      Eigen::Ref<Eigen::Matrix<double, 6, 1>> a_out) const noexcept;

  /// 속도 축약 map G (nv × n_a): v_full = G v_a. 고급 소비자용.
  [[nodiscard]] Eigen::Ref<const Eigen::MatrixXd> GetReductionMap() const noexcept;

  // ── 메타데이터 ──────────────────────────────────────────────────────────────

  [[nodiscard]] int nv_independent() const noexcept { return n_a_; }

  [[nodiscard]] int nv_full() const noexcept { return nv_; }

  [[nodiscard]] int nq_full() const noexcept;

  [[nodiscard]] int constraint_dim() const noexcept { return m_; }

  /// 독립 관절 이름 (q_a 입력 순서 = velocity 인덱스 오름차순). **non-RT.**
  [[nodiscard]] std::vector<std::string> GetIndependentJointNames() const;

  /// 프레임 이름 → FrameIndex. 없으면 0 (universe). **non-RT (string).**
  [[nodiscard]] pinocchio::FrameIndex GetFrameId(std::string_view frame_name) const noexcept;

  /// 프레임 @p frame_id 가 loop-passive 관절의 하류(downstream)인가 (topology-driven wiring).
  /// serial 등가(항등)거나 frame_id 범위 밖이면 false. 의미·근거는 @ref ClosedChainHandle
  /// ::IsFrameDownstreamOfLoop 와 동일.
  [[nodiscard]] bool IsFrameDownstreamOfLoop(pinocchio::FrameIndex frame_id) const noexcept;

  [[nodiscard]] const pinocchio::Model& GetModel() const noexcept { return *model_; }

  [[nodiscard]] const Status& GetStatus() const noexcept { return status_; }

 private:
  /// 독립 관절의 좌표 슬롯 (q_a → full q 매핑용).
  struct IndependentSlot {
    int q_idx{0};               ///< full model q 시작 인덱스
    int v_idx{0};               ///< full model v 인덱스
    bool is_continuous{false};  ///< nq==2 (cos,sin) 관절 여부
    pinocchio::JointIndex jid{0};
  };

  /// 공통 초기화: 독립/종속 인덱스 분할, well-posedness 검증, 버퍼 사전 할당.
  void Initialize();

  /// 현재 q(=q_work_) 에서 φ 와 Jc 를 preallocated phi_/Jc_ 에 채운다 (RT-safe 재구현).
  void ComputeConstraintKinematicsRt(const Eigen::VectorXd& q) noexcept;

  /// 사영된 q_full_ 에서 G 를 재계산 (damped 정규방정식 left-pinv, dep×dep LDLT). RT-safe.
  void RebuildReductionMap() noexcept;

  /// 현재 q_full_/G_ 에서 M_a/g_a/h_a 를 재계산 (UpdateDynamics 본체). RT-safe.
  /// @param have_velocity v_full_ 가 채워졌는가 (false → h_a = g_a).
  void RebuildReducedDynamics(bool have_velocity) noexcept;

  std::shared_ptr<const pinocchio::Model> model_;
  pinocchio::Data data_;
  std::vector<pinocchio::RigidConstraintModel> constraints_;
  std::vector<pinocchio::JointIndex> actuated_joint_ids_;

  int nv_{0};
  int m_{0};    ///< 총 구속 차원
  int n_a_{0};  ///< 독립 좌표 수
  int dep_{0};  ///< 종속(=passive) DoF 수 = nv − n_a
  int num_iterations_{2};
  double proj_lambda2_{0.0};        ///< 사영 λ²
  double reduction_lambda_{0.0};    ///< G left-pinv λ
  double max_seed_increment_{0.0};  ///< tick 당 actuated seed 증분 상한 (≤0 → 비활성)
  double max_passive_deviation_{0.0};  ///< 사영 후 passive 이탈 상한 (≤0 → 비활성)
  bool identity_{false};               ///< 구속 없음 → 항등 (serial 등가)

  std::vector<IndependentSlot> independent_;  ///< 독립 관절 슬롯 (v_idx 오름차순)
  std::vector<int> dep_v_idx_;  ///< 종속(=passive) velocity 인덱스 (오름차순)

  // ── 사전 할당 작업 버퍼 (Update 핫패스에서 재사용) ──────────────────────────
  Eigen::VectorXd q_full_;  ///< loop-consistent 전체 configuration (warm-start 유지)
  Eigen::VectorXd q_work_;  ///< 사영 중간 configuration (커밋 전)
  Eigen::VectorXd q_next_;  ///< integrate 출력
  Eigen::VectorXd dq_;      ///< nv tangent 증분
  Eigen::VectorXd q_seed_;  ///< 이번 tick 사영 **직전** seed 스냅샷 (passive 이탈 판정 기준)
  Eigen::VectorXd dq_a_;  ///< (n_a) actuated seed 증분 (클램프 전) — 균일 스케일용

  // 구속 kinematics 버퍼
  Eigen::VectorXd phi_;           ///< (m)
  Eigen::MatrixXd Jc_;            ///< (m × nv)
  ConstraintKinScratch scratch_;  ///< 6×nv/3×nv 스크래치 (FillConstraintKinematicsRow 공용)

  // 사영 solve 버퍼 (m×m)
  Eigen::MatrixXd Jc_free_;                 ///< (m × dep) 종속 열
  Eigen::MatrixXd JJt_;                     ///< (m × m)
  Eigen::VectorXd y_;                       ///< (m)
  Eigen::VectorXd dv_free_;                 ///< (dep)
  Eigen::LDLT<Eigen::MatrixXd> ldlt_proj_;  ///< m×m in-place 분해

  // G reduction 버퍼
  Eigen::MatrixXd G_;                    ///< (nv × n_a)
  Eigen::MatrixXd Jc_I_;                 ///< (m × n_a) 독립 열
  Eigen::MatrixXd A_dep_;                ///< (dep × dep) Jc_Dᵀ Jc_D + λ²I
  Eigen::MatrixXd B_dep_;                ///< (dep × n_a) Jc_Dᵀ Jc_I
  Eigen::MatrixXd M_dep_;                ///< (dep × n_a) −A⁻¹ B
  Eigen::LDLT<Eigen::MatrixXd> ldlt_G_;  ///< dep×dep in-place 분해

  Eigen::MatrixXd J_full_;  ///< (6 × nv) GetFrameJacobian scratch

  // ── 축약 동역학 버퍼 (UpdateDynamics 핫패스에서 재사용) ─────────────────────
  Eigen::VectorXd v_indep_;   ///< (n_a) span → Eigen
  Eigen::VectorXd v_full_;    ///< (nv) G v_a
  Eigen::MatrixXd MG_;        ///< (nv × n_a) M·G
  Eigen::MatrixXd M_a_;       ///< (n_a × n_a) Gᵀ M G
  Eigen::MatrixXd M_sym_;     ///< (n_a × n_a) 대칭화 임시 (M_aᵀ)
  Eigen::VectorXd g_a_;       ///< (n_a) Gᵀ g
  Eigen::VectorXd h_a_;       ///< (n_a) Gᵀ rnea(q, v_full, a_drift)
  Eigen::VectorXd dq_drift_;  ///< (nv) ±ε v_full (유한차분 스텝)
  Eigen::VectorXd q_plus_;    ///< (nq) integrate(q_full, +dq)
  Eigen::VectorXd q_minus_;   ///< (nq) integrate(q_full, −dq)
  Eigen::VectorXd gamma_;     ///< (m) J̇c v_full 중앙차분
  Eigen::VectorXd JtGamma_;   ///< (dep) Jc_Dᵀ γ
  Eigen::VectorXd a_dep_;     ///< (dep) (Jc_Dᵀ Jc_D + λ²)⁻¹ Jc_Dᵀ γ
  Eigen::VectorXd a_drift_;   ///< (nv) 구속-정합 drift 가속 (scatter)

  Status status_;
};

}  // namespace rtc_urdf_bridge
