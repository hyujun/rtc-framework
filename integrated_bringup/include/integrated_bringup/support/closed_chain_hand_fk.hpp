// ── ClosedChainHandFk: 컨트롤러용 closed-chain hand fingertip FK 래퍼 (#121) ───
#pragma once

#include "rtc_base/types/types.hpp"
#include "rtc_urdf_bridge/rt_closed_chain_handle.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <rclcpp/logger.hpp>

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

/// @brief Configure 결과 — 왜 (비)활성인지. 컨트롤러가 로깅에 사용. 활성이 아닌 모든 경우
///   컨트롤러는 serial `RtModelHandle` 경로를 그대로 쓴다 (byte-for-byte).
enum class HandFkWiringResult {
  kInactiveNoClosure,  ///< 구속 없음 (plain URDF) → serial FK 사용 (byte-for-byte)
  kInactiveNoDownstream,  ///< closure 有이나 어떤 fingertip 도 loop 하류 아님 → 보정 불필요
  kInactiveNoHandRoot,  ///< hand_root 프레임이 full model 에서 해결 안 됨 → serial 합성과 정합 불가
  kInactiveBridgeIncomplete,  ///< 독립 관절 이름이 device joint_state_names 에 없음 → q_a 조립 불가
  kInactiveConstructionFailed,  ///< RtClosedChainHandle 생성 실패(ill-posed closure) → serial
                                ///< fallback
  kActive,                      ///< closed-chain FK 활성
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
/// 소비자는 `status().held` / `status().singular` (J 신뢰 저하) / `status().closure_error`
/// (미수렴 임계) 를 확인해 hold 정책을 건다. `held` 는 비유한 tick 뿐 아니라 **actuated seed
/// 증분 클램프 tick** 에도 선다 (@ref rtc_urdf_bridge::RtClosedChainHandle::Status::held) —
/// activate 직후 seed(q_ref)→측정 q 점프를 여러 tick 에 나눠 따라가는 동안 fingertip pose 캐시가
/// 채워지지 않아 `GetFingertipHandRootPose` 가 false 를 돌린다. 이는 tick 0 과 동일한 기존
/// 경로다 (#248). fault 로 승격하지 말 것 — 정상 walk-in 을 죽인다.
///
/// ⚠ 다만 **"일시적"이 보장되는 것은 클램프(walk-in) tick 뿐**이다. 비유한 측정과 분기 이탈
/// 가드(`max_passive_deviation`)로 인한 held 는 입력이 정상으로 돌아올 때까지 지속되며, 후자는
/// 핸들이 해를 커밋하지 않아 **같은 입력이 계속 들어오면 무기한** 이어질 수 있다. 그 구간 내내
/// fingertip pose 는 조용히 stale 이므로, 오래가는 held 는 `status().held_ticks` 로 관측해
/// off-RT 진단으로 넘긴다 (예상 walk-in ≈ `⌈Δ/max_seed_increment⌉` tick).
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
  /// @param hand_root_link fingertip 을 표현할 기준 프레임. **full model 에서 해결돼야** 활성화된다
  ///   (해결 안 되면 kInactiveNoHandRoot → serial 경로. closed 핸들은 arm-base world 라 hand-root
  ///   anchor 없이는 arm TCP 합성이 틀리기 때문).
  /// @param closure_error_threshold 이 값 이상의 ‖φ‖(미수렴/특이) tick 은 신뢰 불가로 보고 직전
  ///   유효 fingertip pose 를 hold 한다 (기본 1e-3 m).
  /// @note ill-posed closure 로 RtClosedChainHandle 생성이 throw 하면 catch 해서
  ///   kInactiveConstructionFailed 로 graceful 하게 serial 로 떨어진다 (컨트롤러 config abort
  ///   방지).
  [[nodiscard]] HandFkWiringResult Configure(
      std::shared_ptr<const pinocchio::Model> model,
      std::vector<pinocchio::RigidConstraintModel> constraints,
      std::vector<pinocchio::JointIndex> actuated_joint_ids, Eigen::VectorXd q_seed,
      const std::vector<std::vector<std::string>>& device_joint_names,
      std::span<const std::string> fingertip_links, std::string_view hand_root_link,
      double closure_error_threshold = 1e-3);

  [[nodiscard]] bool active() const noexcept { return active_; }

  /// @brief measured 상태에서 actuated q_a 를 모아 closed-chain 사영 + FK 갱신. **RT-safe.**
  /// 비활성이면 no-op. 소스가 유효하고 결과가 유한하면(sources_ok && !held && finite closure) 각
  /// 활성 fingertip pose 캐시를 갱신하되, **loop 하류** tip 은 loop-trustworthy(!singular &&
  /// closure_error<threshold)한 tick 에서만, **비하류** tip 은 유한 tick 이면 항상 갱신한다 —
  /// 비하류 pose 는 actuated q 만의 함수라 loop 미수렴/특이와 무관하기 때문(#3). 아니면 캐시(직전
  /// 유효 해)를 유지한다 — status 소비를 래퍼가 내부화한다.
  void Update(const rtc::ControllerState& state) noexcept;

  /// @brief fingertip @p f 의 **hand-root 상대** placement(직전 신뢰 tick 값)를 @p out 에 기록.
  /// @return 활성 fingertip 이고 유효 캐시가 있으면 true. 비활성/OOB/캐시없음이면 false (out
  /// 미변경).
  /// **RT-safe.** loop 하류 fingertip 은 loop-consistent, 비하류 fingertip 은 full-model FK(serial
  /// 등가)로 둘 다 서비스된다 (#121: 활성 시 혼합 손의 serial 손가락도 유지).
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
  double closure_error_threshold_{1e-3};
  pinocchio::FrameIndex hand_root_fid_{0};
  std::array<pinocchio::FrameIndex, kMaxFingertips> fingertip_fid_{};
  std::array<bool, kMaxFingertips> fingertip_active_{};
  // loop-passive 관절 하류 여부(Configure 판정). 비하류(serial 등가) tip 의 pose 는 actuated q 만의
  // 함수라 loop 미수렴/특이와 무관 — untrustworthy tick 에도 유효하게 갱신한다 (#3).
  std::array<bool, kMaxFingertips> fingertip_downstream_{};

  // 직전 신뢰 tick 의 hand-root 상대 fingertip pose 캐시 (untrustworthy tick 은 hold).
  std::array<pinocchio::SE3, kMaxFingertips> last_pose_{};
  std::array<bool, kMaxFingertips> last_pose_valid_{};

  std::vector<QSource>
      bridge_;  ///< 독립 관절별 측정 소스 (크기 n_a, GetIndependentJointNames 순서)
  Eigen::VectorXd q_a_;  ///< n_a, preallocated
  std::string missing_joint_;
};

// ── 컨트롤러 wiring 공용 dispatch (task/joint 공유, #121 Phase 2c) ─────────────
// task/joint 컨트롤러가 동일하게 쓰는 hand-FK 분기·로깅을 단일 출처로 둔다.

/// @brief 이번 tick 의 hand FK 를 실행한다: closed 활성이면 @p fk.Update(state), 아니면 serial
///   @p hand_handle 의 ComputeForwardKinematics(measured hand q). **RT-safe.**
/// @return hand device(devices[1]) 가 유효해 FK 를 돌렸으면 true.
[[nodiscard]] bool RunHandForwardKinematics(ClosedChainHandFk& fk,
                                            rtc_urdf_bridge::RtModelHandle* hand_handle,
                                            Eigen::VectorXd& hand_q,
                                            const rtc::ControllerState& state) noexcept;

/// @brief fingertip @p f 의 **hand-root 상대** pose 를 closed(활성) 또는 serial 에서 얻어 @p out
///   에 기록. serial 경로는 기존 tree-model 계산과 byte-for-byte 동일. **RT-safe.**
/// @return 유효 pose 를 얻었으면 true (비활성/미해결 fingertip 이면 false, out 미변경).
[[nodiscard]] bool HandFingertipPoseDispatch(
    const ClosedChainHandFk& fk, const rtc_urdf_bridge::RtModelHandle* hand_handle,
    const std::array<pinocchio::FrameIndex, ClosedChainHandFk::kMaxFingertips>& fingertip_ids,
    bool use_hand_root, pinocchio::FrameIndex hand_root_id, std::size_t f,
    pinocchio::SE3& out) noexcept;

/// @brief Configure 결과를 컨트롤러 태그(@p tag, 예: "[task]")와 함께 로깅 (task/joint 공용).
void LogHandFkWiring(const rclcpp::Logger& logger, const char* tag, HandFkWiringResult result,
                     const std::string& missing_joint);

}  // namespace integrated_bringup
