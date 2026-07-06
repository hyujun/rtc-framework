// ── WbcReducedDynamicsProvider: closed-chain 축약 동역학을 PinocchioCache 에 주입 (#120) ──
#pragma once

#include "rtc_tsid/types/reduced_dynamics_provider.hpp"
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
/// ### RT 계약
/// `FillReducedDynamics` 는 noexcept·힙 할당 없음 (핸들·모든 버퍼 preallocated).
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

  [[nodiscard]] bool active() const noexcept { return active_; }

  [[nodiscard]] int n_a() const noexcept { return n_a_; }

  /// 정렬 미매칭 시(비활성) 매핑 실패한 관절 이름 (로깅용).
  [[nodiscard]] const std::string& missing_joint() const noexcept { return missing_joint_; }

  // ── ReducedDynamicsProvider ────────────────────────────────────────────────
  [[nodiscard]] bool FillReducedDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                         Eigen::MatrixXd& M, Eigen::VectorXd& h,
                                         Eigen::VectorXd& g) noexcept override;

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
};

}  // namespace integrated_bringup
