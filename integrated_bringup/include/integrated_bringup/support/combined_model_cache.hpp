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

  /// @ref InitModel 의 앞 절반: 모델 선택(actuated → tree "wbc" → full) + q/v 버퍼 alloc만
  /// 수행하고 cache Init 은 하지 않는다. WBC 처럼 contact frame id 파싱이 선택된 모델을
  /// 요구해 cache Init 을 뒤로 미뤄야 하는(select → contact 파싱 → cache Init) 컨트롤러용.
  /// 이후 호출자는 @ref InitCacheDeferred 로 cache 를 초기화한다(@ref cache() 의 raw `Init`
  /// 직접 호출 대신 — 그래야 @ref Update / @ref ArmTcpPoseFromCache 의 cache-ready 게이트가
  /// self-enforcing 하게 유지된다). joint/task 는 대신 묶음 @ref InitModel 을 쓴다.
  /// @return 모델을 얻으면 true (@ref model() 유효), 그 외 false.
  [[nodiscard]] bool SelectModel(rtc_urdf_bridge::PinocchioModelBuilder& builder,
                                 std::string_view log_prefix, const rclcpp::Logger& logger);

  /// @ref SelectModel 뒤 지연 cache Init: 선택된 모델(@ref model())과 @p contact_frame_ids 로
  /// @ref rtc_urdf_bridge::PinocchioCache::Init 을 수행하고 cache-ready 플래그를 세운다. WBC 의
  /// select → contact 파싱 → cache Init 순서용. @ref SelectModel 선행 필수(@ref model() 유효).
  void InitCacheDeferred(const std::vector<pinocchio::FrameIndex>& contact_frame_ids);

  /// ext(device joint-state 순서) → Pinocchio q/v reorder map 구성. @p arm_joint_names 가
  /// nullptr(primary device config 부재)이면 identity fallback. @p hand_joint_names 가 nullptr
  /// 이면 hand 관절 없음. @p full_dof = arm_dof + hand_dof. 완료 후 @ref reorder_valid() 갱신.
  void BuildReorderMap(const std::vector<std::string>* arm_joint_names,
                       const std::vector<std::string>* hand_joint_names, int full_dof,
                       std::string_view log_prefix, const rclcpp::Logger& logger);

  /// 측정 device pos/vel 을 Pinocchio 순서 q/v 로 scatter (arm=device0[0,arm_dof), hand=
  /// device1[0,hand_dof) → ext[arm_dof, arm_dof+hand_dof)). @ref reorder_valid() 무효 시 no-op.
  /// device channel 수로 clamp. RT-safe, no alloc.
  ///
  /// **F5 게이트 내장** (#236 S7b): `rtc::IsDeviceReadable(devices[0], arm_dof)` 가 false 면
  /// arm scatter 를 아예 하지 않는다 (no-op). clamp 만으로는 안전해지지 않기 때문이다 —
  /// `q_curr_full_` 은 tick 을 넘어 **지속**하므로 건너뛴 슬롯이 이전값(첫 tick 엔 0)을
  /// 유지하고, 모델이 보는 configuration 은 미보고 채널을 그냥 읽은 것과 수치적으로 같아진다.
  /// 계약·근거는 `rtc_controller_interface/device_readability.hpp` 와
  /// `rtc_controllers/docs/compliance-conventions.md` §3.7.
  void ExtractFullState(const rtc::ControllerState& state, int arm_dof, int hand_dof) noexcept;

  /// scatter 된 q/v 로 cache 의 FK/J/M/h/g/oMf 갱신. RT-safe 래퍼. cache 미초기화
  /// (@ref InitModel / @ref InitCacheDeferred 미수행) 시 no-op — un-init cache_ 소비 방지.
  void Update() noexcept;

  /// arm TCP pose (world tip; @p base_idx>=0 이면 base frame-relative). 미구성(@p tcp_idx<0),
  /// non-fresh(@ref reorder_valid() 무효), 또는 cache 미초기화 시 Identity — 등록됐지만 한 번도
  /// Update 안 된(또는 Init 안 된) frame 의 stale oMf 를 읽지 않게 게이트. RT-safe, no alloc.
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

  /// ext→Pinocchio v-index reorder map 전체 (첫 full_dof_ 슬롯만 유효, 나머지 0-init).
  /// 배열 전체를 받는 정적 헬퍼(WBC 의 @c BuildClikJointIndexSets /
  /// @c AssemblePostureGains — URDF 없이 unit-test 가능하도록 배열 인자로 고정)용.
  [[nodiscard]] const std::array<int, kMaxFullDof>& ext_to_pin_v_map() const noexcept {
    return ext_to_pin_v_;
  }

  [[nodiscard]] const Eigen::VectorXd& q() const noexcept { return q_curr_full_; }

  [[nodiscard]] const Eigen::VectorXd& v() const noexcept { return v_curr_full_; }

 private:
  std::shared_ptr<const pinocchio::Model> full_model_ptr_;
  rtc_urdf_bridge::PinocchioCache cache_;
  std::array<int, kMaxFullDof> ext_to_pin_q_{};
  std::array<int, kMaxFullDof> ext_to_pin_v_{};
  bool joint_reorder_valid_{false};
  /// cache_ 가 Init 됐는지(@ref InitModel / @ref InitCacheDeferred). @ref Update /
  /// @ref ArmTcpPoseFromCache 의 self-enforcing 게이트 — un-init cache_ 소비를 막는다.
  bool cache_initialized_{false};
  int full_dof_{0};
  Eigen::VectorXd q_curr_full_;  ///< [nq] current q in Pinocchio order (per tick)
  Eigen::VectorXd v_curr_full_;  ///< [nv] current v in Pinocchio order (per tick)
};

}  // namespace integrated_bringup
