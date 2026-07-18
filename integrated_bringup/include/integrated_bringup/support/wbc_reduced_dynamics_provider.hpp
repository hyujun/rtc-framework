// ── WbcReducedDynamicsProvider: closed-chain 축약 동역학을 PinocchioCache 에 주입 (#120) ──
#pragma once

#include "rtc_tsid/types/reduced_dynamics_provider.hpp"
#include "rtc_urdf_bridge/pinocchio_cache.hpp"
#include "rtc_urdf_bridge/rt_closed_chain_handle.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

namespace integrated_bringup {

/// @brief WBC 의 TSID EOM(PinocchioCache M/h/g)을 constraint-consistent 축약 동역학으로 덮는
///   @ref rtc::tsid::ReducedDynamicsProvider 구체 구현.
///
/// `urdf.extended: true` 인 로봇에서 WBC 는 control model 로 `GetActuatedModel()` (loop-passive 만
/// 잠근 nq==nv actuated 모델) 을 쓴다. 이 모델의 open-chain M/h/g 는 loop 구속을 무시하므로,
/// loop-passive DoF 를 사영해 얻은 **축약** M_a/g_a/h_a (@ref rtc_urdf_bridge::RtClosedChainHandle)
/// 로 대체한다.
///
/// ### 좌표 정렬 (Configure 시 1회, 안전-by-construction)
/// closed 핸들은 **full spanning-tree 모델** 위에서 동작하고 독립좌표를 full-model velocity 인덱스
/// 오름차순으로 노출한다. cache(=actuated control model) 의 좌표 순서와 다를 수 있으므로,
/// **이름 기반 permutation** 을 구성한다: 핸들 독립좌표 k ↔ cache 모델의 (q_idx, v_idx). 모든
/// 핸들 독립 관절이 cache 모델에 단일-DoF 로 존재하고 n_a == cache.nv (bijection) 일 때만 활성.
/// 어긋나면 비활성 → PinocchioCache 는 open-chain 값을 그대로 유지 (byte-for-byte fallback).
/// buildReducedModel 의 관절 순서를 **가정하지 않는다**.
///
/// ### held / singular 정책
/// 사영이 held(비유한) 또는 singular(특이 조립) 인 tick 은 **직전 유효(last-good) 축약 M/h/g 를
/// 유지**한다 — degraded 동역학을 실기 QP 에 주입하지 않기 위함. 최초 유효 tick 이전이면
/// `FillReducedDynamics` 가 false 를 돌려 open-chain 값이 유지된다.
///
/// ### Phase ③ — loop-consistent contact frame kinematics (unified kin&dyn)
/// closed-chain 활성 시 WBC contact frame 의 J·oMf 를 frozen-loop(open-chain) 근사 대신
/// **loop-consistent** 값(@ref FillReducedFrameKinematics)으로 격상한다. provider 는 M/h/g 를 위해
/// 매 tick 이미 사영한 **같은 핸들**을 재사용하므로 (`FillReducedDynamics` 가 먼저 호출돼 핸들
/// Update·UpdateDynamics 를 돌린 직후) contact J/oMf 는 M/h/g 와 **동일 사영 형상**에서 나온다 —
/// provider 내부 사영 1회. @ref ConfigureContactFrames 로 하류 contact frame 을 미리 판정한다.
///
/// **frozen-loop 미주입 정책 (사용자 확정 2026-07-18)**:
///  - **L1**: held/singular tick 은 open-chain fallback 이 아니라 직전 유효(last-good)
///  loop-consistent
///    J/oMf 를 유지한다 (최초 유효 tick 이전만 open-chain — 불가피). degraded J 를 QP 에 안 넣는다.
///  - **L2-zero**: 하류 frame **dJv(drift)=0** (RtClosedChainHandle 에 가속도 API 없음 →
///    frozen-loop drift 대신 미보상). 완전 loop-consistent dJv 는 issue #173 후속.
///  - 비하류(serial 등가) contact frame 은 open-chain 이 **정확값**이라 미변경 (byte-for-byte).
///
/// ### RT 계약
/// `FillReducedDynamics` / `FillReducedFrameKinematics` 는 noexcept·힙 할당 없음 (핸들·모든 버퍼
/// preallocated).
class WbcReducedDynamicsProvider final : public rtc::tsid::ReducedDynamicsProvider {
 public:
  WbcReducedDynamicsProvider() = default;

  /// @brief closed 핸들 + cache 좌표 permutation 구성 (non-RT). 활성 여부를 돌린다.
  /// @param full_model full spanning-tree 모델 (builder->GetFullModel()).
  /// @param constraints loop 구속 (builder->GetConstraintModels()).
  /// @param actuated_joint_ids 독립(구동) 관절 (builder->GetClosureActuatedJointIds()).
  /// @param q_seed loop-consistent warm-start seed (builder->GetClosureReferenceConfig()).
  /// @param control_model cache 가 계산하는 control model (= WBC full_model_ptr_ = actuated model).
  ///   M/h/g 를 이 모델의 좌표계로 덮는다.
  /// @return true → 활성 (모든 정렬 매칭·well-posed closure). false → 비활성 (open-chain 유지).
  /// @note ill-posed closure(dep>m 등)로 핸들 생성이 throw 하면 catch → 비활성 (graceful).
  [[nodiscard]] bool Configure(std::shared_ptr<const pinocchio::Model> full_model,
                               std::vector<pinocchio::RigidConstraintModel> constraints,
                               std::vector<pinocchio::JointIndex> actuated_joint_ids,
                               Eigen::VectorXd q_seed, const pinocchio::Model& control_model);

  /// @brief (Phase ③) contact frame 을 loop-consistent override 대상으로 미리 판정한다 (non-RT).
  ///   각 contact frame(cache/control 모델 frame id)을 이름으로 full 모델 fid 에 매핑하고
  ///   `IsFrameDownstreamOfLoop` 로 loop-하류 여부를 기록한다. 비활성 provider 이거나 매핑 실패/
  ///   비하류 frame 은 override 대상에서 제외돼 open-chain(정확값) 이 유지된다.
  /// @param control_model cache 가 계산하는 control model (contact frame id 의 이름 조회원).
  /// @param contact_frame_ids cache.contact_frames 와 동일 순서의 control-model FrameIndex 리스트
  ///   (`rtc::tsid::ContactFrameIds(mgr)`).
  /// @note `Configure` 성공 후 호출해야 한다 (핸들·좌표 permutation 이 있어야 매핑 가능).
  void ConfigureContactFrames(const pinocchio::Model& control_model,
                              const std::vector<pinocchio::FrameIndex>& contact_frame_ids);

  [[nodiscard]] bool active() const noexcept { return active_; }

  [[nodiscard]] int n_a() const noexcept { return n_a_; }

  /// 정렬 미매칭 시(비활성) 매핑 실패한 관절 이름 (로깅용).
  [[nodiscard]] const std::string& missing_joint() const noexcept { return missing_joint_; }

  /// (Phase ③) `ConfigureContactFrames` 에서 full 모델에 매핑 실패한 contact frame 이름들
  /// (로깅용, non-RT). 매핑 실패 frame 은 loop-하류 여부를 판정할 수 없어 override 대상에서
  /// 빠지고 open-chain(frozen-loop) 값이 조용히 유지되므로, 호출측이 이를 WARN 으로 노출한다.
  [[nodiscard]] const std::vector<std::string>& unmapped_contact_frames() const noexcept {
    return unmapped_contact_frames_;
  }

  // ── ReducedDynamicsProvider ────────────────────────────────────────────────
  [[nodiscard]] bool FillReducedDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                         Eigen::MatrixXd& M, Eigen::VectorXd& h,
                                         Eigen::VectorXd& g) noexcept override;

  /// @brief (Phase ③) loop-하류 contact frame 의 J·oMf 를 loop-consistent 값으로 덮는다.
  ///   provider 가 `FillReducedDynamics` 에서 이미 사영한 핸들을 재사용 (재사영 없음). frozen-loop
  ///   미주입 정책 (L1 last-good hold / L2-zero dJv / 비하류 미변경) 은 클래스 주석 참조.
  /// @return 하나 이상 frame 을 덮었으면 true. **RT-safe.**
  /// @warning **순서 계약**: 반드시 같은 `PinocchioCache::Update()` 안에서 `FillReducedDynamics`
  ///   (핸들 Update)가 먼저 돈 뒤 호출해야 한다 — q/v 를 무시하고 핸들의 사영 kinematics 를
  ///   재사용하기 때문. 이 순서는 `pinocchio_cache.cpp` 배선이 보증하며, 어기면 stale kinematics
  ///   주입이 되고 debug 빌드에서 assert 로 잡힌다.
  [[nodiscard]] bool FillReducedFrameKinematics(
      const Eigen::VectorXd& q, const Eigen::VectorXd& v,
      std::vector<rtc_urdf_bridge::PinocchioCache::FrameCache>& contact_frames) noexcept override;

 private:
  /// last-good(또는 fresh) 축약 M_a/h_a/g_a(핸들 순서)를 cache 좌표 M/h/g 로 흩뿌린다. RT-safe.
  void ScatterInto(Eigen::MatrixXd& M, Eigen::VectorXd& h, Eigen::VectorXd& g,
                   const Eigen::MatrixXd& M_src, const Eigen::VectorXd& h_src,
                   const Eigen::VectorXd& g_src) const noexcept;

  std::unique_ptr<rtc_urdf_bridge::RtClosedChainHandle> handle_;
  bool active_{false};
  int n_a_{0};
  std::string missing_joint_;

  // 핸들 독립좌표 k → cache 모델 인덱스 (Configure 산출, 크기 n_a).
  std::vector<int> cache_q_idx_;     ///< cache q 시작 인덱스
  std::vector<int> cache_v_idx_;     ///< cache v 인덱스
  std::vector<char> is_continuous_;  ///< cache 관절이 nq==2 (cos,sin)

  // RT 스크래치 (preallocated).
  Eigen::VectorXd q_a_;     ///< n_a, 핸들 입력 (스칼라 각)
  Eigen::VectorXd v_a_;     ///< n_a, 핸들 입력
  Eigen::MatrixXd M_last_;  ///< n_a × n_a, 직전 유효 축약 관성
  Eigen::VectorXd h_last_;  ///< n_a
  Eigen::VectorXd g_last_;  ///< n_a
  bool have_last_{false};

  // ── Phase ③ contact frame loop-consistent override (ConfigureContactFrames 산출) ──
  /// 축약 J_a(핸들 독립좌표 순서, 6×n_a)를 cache 좌표(cache_v_idx_) J(6×nv)로 열 흩뿌림. RT-safe.
  void ScatterFrameJacobian(Eigen::MatrixXd& J_dst, const Eigen::MatrixXd& J_a) const noexcept;

  bool contact_frames_configured_{false};
  int n_contacts_{0};
  std::vector<pinocchio::FrameIndex> contact_full_fid_;  ///< contact i → full-model fid (0=미매핑)
  std::vector<char> contact_downstream_;  ///< contact i loop-하류 & 매핑됨 → override
  std::vector<std::string>
      unmapped_contact_frames_;  ///< full 모델 매핑 실패 이름 (WARN 용, non-RT)
  /// F3 handshake: 같은 PinocchioCache::Update() 안에서 FillReducedDynamics(핸들 Update)가
  /// FillReducedFrameKinematics 앞에 돌았는지 검증하는 per-tick 토큰. 전자에서 set, 후자에서
  /// (핸들 사용 직전) assert 후 소비(reset). Release(NDEBUG)에서는 assert 컴파일 아웃 → RT no-op.
  bool dynamics_projected_{false};
  // L1: 하류 frame 별 직전 유효(last-good) loop-consistent J_a(6×n_a, 핸들 순서)·oMf.
  std::vector<Eigen::MatrixXd> contact_J_last_;
  std::vector<pinocchio::SE3> contact_oMf_last_;
  std::vector<char> contact_have_last_;
  Eigen::MatrixXd J_a_scratch_;  ///< 6 × n_a, GetFrameJacobian 출력 (preallocated)
};

}  // namespace integrated_bringup
