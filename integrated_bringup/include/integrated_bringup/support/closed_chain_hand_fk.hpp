// ── ClosedChainHandFk: 컨트롤러용 closed-chain hand fingertip FK 래퍼 (#121) ───
#pragma once

#include "rtc_base/types/types.hpp"
#include "rtc_urdf_bridge/rt_closed_chain_handle.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace integrated_bringup {

/// @brief Configure 결과 — 왜 (비)활성인지. 컨트롤러가 로깅에 사용.
enum class HandFkWiringResult {
  kInactiveNoClosure,  ///< 구속 없음 (plain URDF) → serial FK 사용 (byte-for-byte)
  kInactiveNoDownstream,  ///< closure 有이나 어떤 fingertip 도 loop 하류 아님 → 보정 불필요
  kInactiveBridgeIncomplete,  ///< 독립 관절 이름이 device joint_state_names 에 없음 → q_a 조립 불가
  kActive,                    ///< closed-chain FK 활성
};

/// @brief task/joint 컨트롤러의 **hand fingertip FK** 를 closed-chain-consistent 로 얻는 래퍼.
///
/// `urdf.extended: true` (loop closure sidecar) 인 hand 에서, loop-passive 관절 하류의 fingertip
/// 프레임은 frozen-loop(tree 모델) FK 로 계산하면 큰 오차가 난다 (#121 Phase 1: 운영점 이탈 시
/// ~5.6mm/°). 이 래퍼는 @ref rtc_urdf_bridge::RtClosedChainHandle 를 감싸 measured actuated q 로
/// passive DoF 를 RT-safe fixed-step 사영한 뒤 fingertip 의 **hand-root 상대** placement 를
/// 돌려준다. closure 가 없거나 어떤 fingertip 도 loop 하류가 아니면 **비활성** 이 되어 컨트롤러는
/// 기존 serial `RtModelHandle` 경로를 그대로 쓴다 (topology-driven, byte-for-byte).
///
/// ### 이름 브릿지 (Configure 시 1회)
/// closed 핸들은 **full spanning-tree 모델** (arm+hand) 위에서 동작하므로 컨트롤러의 device/서브
/// 모델 인덱싱과 다르다. 두 브릿지를 구성한다:
///  - (a) fingertip / hand-root link 이름 → full-model FrameIndex,
///  - (b) 독립(actuated) 관절 이름 → 측정 q 의 (device index, channel) — 매 tick `Update(state)`
///        가 이 브릿지로 device 전반에서 q_a 를 모은다 (독립 관절은 arm+hand 에 걸칠 수 있다).
///
/// ### RT 계약
/// `Update` / `GetFingertipHandRootPose` 는 noexcept·힙 할당 없음 (내부 핸들이 preallocated).
/// 소비자는 `status().held` (비유한→직전 해) / `status().singular` (J 신뢰 저하) /
/// `status().closure_error` (미수렴 임계) 를 확인해 hold 정책을 건다.
class ClosedChainHandFk {
 public:
  static constexpr std::size_t kMaxFingertips = 4;

  ClosedChainHandFk() = default;

  /// @brief closure 감지 + 핸들·브릿지 구성 (non-RT). 활성 여부를 결과로 돌린다.
  /// @param model full spanning-tree 모델 (builder->GetFullModel()); null 이면 kInactiveNoClosure.
  /// @param constraints loop 구속 (builder->GetConstraintModels()); 빈 벡터면 kInactiveNoClosure.
  /// @param actuated_joint_ids 독립 관절 (builder->GetClosureActuatedJointIds()).
  /// @param q_seed loop-consistent warm-start seed (builder->GetClosureReferenceConfig()).
  /// @param device_joint_names device index 순서의 joint_state_names (q_a 브릿지 소스).
  /// @param fingertip_links fingertip link 이름 (최대 kMaxFingertips). 빈 슬롯은 무시.
  /// @param hand_root_link fingertip 을 표현할 기준 프레임 (빈 문자열이면 full-model world).
  [[nodiscard]] HandFkWiringResult Configure(
      std::shared_ptr<const pinocchio::Model> model,
      std::vector<pinocchio::RigidConstraintModel> constraints,
      std::vector<pinocchio::JointIndex> actuated_joint_ids, Eigen::VectorXd q_seed,
      const std::vector<std::vector<std::string>>& device_joint_names,
      std::span<const std::string> fingertip_links, std::string_view hand_root_link);

  [[nodiscard]] bool active() const noexcept { return active_; }

  /// @brief measured 상태에서 actuated q_a 를 모아 closed-chain 사영 + FK 갱신. **RT-safe.**
  /// 비활성이면 no-op. 크기 계약 위반은 내부 핸들이 held 로 처리.
  void Update(const rtc::ControllerState& state) noexcept;

  /// @brief fingertip @p f 의 **hand-root 상대** placement 를 @p out 에 기록.
  /// @return fingertip 이 활성(loop 하류로 배선됨)이면 true. 비활성/OOB 면 false (out 미변경).
  /// **RT-safe.** hand_root_link 이 지정됐으면 `oMroot.actInv(oMtip)`, 아니면 full-model world.
  [[nodiscard]] bool GetFingertipHandRootPose(std::size_t f, pinocchio::SE3& out) const noexcept;

  /// @brief 직전 Update 의 loop-consistency/특이 상태.
  [[nodiscard]] rtc_urdf_bridge::RtClosedChainHandle::Status status() const noexcept;

  /// @brief 브릿지 미완성 시(kInactiveBridgeIncomplete) 매핑 실패한 관절 이름 (로깅용).
  [[nodiscard]] const std::string& missing_joint() const noexcept { return missing_joint_; }

 private:
  struct QSource {
    int device{0};   ///< ControllerState.devices index
    int channel{0};  ///< device.positions 내 channel (= joint_state_names 위치)
  };

  std::unique_ptr<rtc_urdf_bridge::RtClosedChainHandle> handle_;
  bool active_{false};
  bool use_hand_root_{false};
  pinocchio::FrameIndex hand_root_fid_{0};
  std::array<pinocchio::FrameIndex, kMaxFingertips> fingertip_fid_{};
  std::array<bool, kMaxFingertips> fingertip_active_{};

  std::vector<QSource>
      bridge_;  ///< 독립 관절별 측정 소스 (크기 n_a, GetIndependentJointNames 순서)
  Eigen::VectorXd q_a_;  ///< n_a, preallocated
  std::string missing_joint_;
};

}  // namespace integrated_bringup
