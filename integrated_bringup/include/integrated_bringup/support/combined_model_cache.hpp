// ── CombinedModelCache: joint/task/wbc 공유 결합-모델(arm+hand) cache 배선 (#174) ──
#pragma once

#include "rtc_urdf_bridge/pinocchio_cache.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <rclcpp/logger.hpp>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace rtc {
struct ControllerState;
}  // namespace rtc

namespace rtc_urdf_bridge {
class PinocchioModelBuilder;
}  // namespace rtc_urdf_bridge

namespace integrated_bringup {

/// @brief joint/task/wbc 데모 컨트롤러가 공유하는 결합-모델(arm+hand) cache 배선 (#174).
///
/// unified kin&dyn 리팩터로 세 컨트롤러가 동일한 combined-model @ref
/// rtc_urdf_bridge::PinocchioCache 로 arm 기구학을 획득하게 되면서, 아래 네 조각이 삼중
/// 복제됐다: 모델 선택(actuated closed-chain → reduced tree "wbc" → raw full) + cache Init,
/// ext(device joint-state 순서)→Pinocchio q/v reorder map, 매 tick state scatter, arm-TCP FK.
/// 이 타입이 그 공통 배선을 소유한다. 컨트롤러는 멤버로 소유하고 위임한다.
///
/// ### 범위 (TSID-agnostic)
/// WBC 의 TSID contact 프레임과 축약 동역학 provider 는 이 헬퍼의 **superset** 이다: contact
/// 프레임 id 는 @ref InitModel 인자로 전달(joint/task 는 빈 리스트), @ref
/// rtc_urdf_bridge::PinocchioCache::reduced_provider 는 컨트롤러가 @ref cache() 로 직접 주입한다.
/// 헬퍼 자체는 TSID 를 알지 못한다.
///
/// ### frame 소유
/// arm TCP tip/base frame 은 컨트롤러가 @ref cache() 의 `RegisterFrame` 으로 등록하고 그
/// 인덱스를 소유한다. @ref ArmTcpPoseFromCache 는 그 인덱스를 인자로 받는다 — 헬퍼는
/// 컨트롤러별 frame 이름(arm_tcp / task_tcp / clik_tcp)에 무관하다.
class CombinedModelCache {
 public:
  /// arm+hand 합산 DoF 상한 (ext→pin reorder 버퍼 고정 용량). 세 컨트롤러의 기존
  /// `MaxFullDof`(32+32) 와 동일. 실 로봇 DoF ≪ 이 값.
  static constexpr int kMaxFullDof = 64;

  CombinedModelCache() = default;

  /// 모델 선택(actuated closed-chain → reduced tree "wbc" → raw full model) + 그 위에서
  /// @ref rtc_urdf_bridge::PinocchioCache::Init + q/v 버퍼 alloc. @p contact_frame_ids 는
  /// joint/task 에서 비어 있고 WBC 는 자기 TSID contact 프레임을 전달한다. @p log_prefix 는
  /// "[joint]" 등 컨트롤러 태그.
  /// @return builder 가 유효하고 모델을 얻으면 true (Init 수행), 그 외 false.
  [[nodiscard]] bool InitModel(rtc_urdf_bridge::PinocchioModelBuilder& builder,
                               const std::vector<pinocchio::FrameIndex>& contact_frame_ids,
                               std::string_view log_prefix, const rclcpp::Logger& logger);

  /// ext(device joint-state 순서) → Pinocchio q/v reorder map 구성. @p arm_joint_names 가
  /// nullptr(primary device config 부재)이면 identity fallback. @p hand_joint_names 가 nullptr
  /// 이면 hand 관절 없음. @p full_dof = arm_dof + hand_dof. 완료 후 @ref reorder_valid() 갱신.
  void BuildReorderMap(const std::vector<std::string>* arm_joint_names,
                       const std::vector<std::string>* hand_joint_names, int full_dof,
                       std::string_view log_prefix, const rclcpp::Logger& logger);

  /// 측정 device pos/vel 을 Pinocchio 순서 q/v 로 scatter (arm=device0[0,arm_dof), hand=
  /// device1[0,hand_dof) → ext[arm_dof, arm_dof+hand_dof)). @ref reorder_valid() 무효 시 no-op.
  /// device channel 수로 clamp. RT-safe, no alloc.
  void ExtractFullState(const rtc::ControllerState& state, int arm_dof, int hand_dof) noexcept;

  /// scatter 된 q/v 로 cache 의 FK/J/M/h/g/oMf 갱신. RT-safe 래퍼.
  void Update() noexcept;

  /// arm TCP pose (world tip; @p base_idx>=0 이면 base frame-relative). 미구성(@p tcp_idx<0)
  /// 또는 non-fresh(@ref reorder_valid() 무효) 시 Identity — 등록됐지만 한 번도 Update 안 된
  /// frame 의 stale oMf 를 읽지 않게 게이트. RT-safe, no alloc.
  [[nodiscard]] pinocchio::SE3 ArmTcpPoseFromCache(int tcp_idx, int base_idx) const noexcept;

  // ── accessors ────────────────────────────────────────────────────────────
  [[nodiscard]] rtc_urdf_bridge::PinocchioCache& cache() noexcept { return cache_; }

  [[nodiscard]] const rtc_urdf_bridge::PinocchioCache& cache() const noexcept { return cache_; }

  [[nodiscard]] const std::shared_ptr<const pinocchio::Model>& model() const noexcept {
    return full_model_ptr_;
  }

  [[nodiscard]] bool reorder_valid() const noexcept { return joint_reorder_valid_; }

  /// ext(device joint-state) 인덱스 → Pinocchio q/v 인덱스. arm-column Jacobian 추출 등
  /// reorder map 을 직접 소비하는 컨트롤러(task)용. 호출자는 ext_idx ∈ [0, full_dof) 보장.
  [[nodiscard]] int ext_to_pin_q(int ext_idx) const noexcept {
    return ext_to_pin_q_[static_cast<std::size_t>(ext_idx)];
  }

  [[nodiscard]] int ext_to_pin_v(int ext_idx) const noexcept {
    return ext_to_pin_v_[static_cast<std::size_t>(ext_idx)];
  }

  [[nodiscard]] const Eigen::VectorXd& q() const noexcept { return q_curr_full_; }

  [[nodiscard]] const Eigen::VectorXd& v() const noexcept { return v_curr_full_; }

  /// mutable q/v — 컨트롤러가 posture reference 조립 등으로 in-place 갱신하는 드문 경로용.
  [[nodiscard]] Eigen::VectorXd& q_mutable() noexcept { return q_curr_full_; }

  [[nodiscard]] Eigen::VectorXd& v_mutable() noexcept { return v_curr_full_; }

 private:
  std::shared_ptr<const pinocchio::Model> full_model_ptr_;
  rtc_urdf_bridge::PinocchioCache cache_;
  std::array<int, kMaxFullDof> ext_to_pin_q_{};
  std::array<int, kMaxFullDof> ext_to_pin_v_{};
  bool joint_reorder_valid_{false};
  int full_dof_{0};
  Eigen::VectorXd q_curr_full_;  ///< [nq] current q in Pinocchio order (per tick)
  Eigen::VectorXd v_curr_full_;  ///< [nv] current v in Pinocchio order (per tick)
};

}  // namespace integrated_bringup
