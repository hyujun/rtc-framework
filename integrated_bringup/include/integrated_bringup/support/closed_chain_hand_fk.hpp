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
#include <cstdint>
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
///
/// ⚠ 위 진단 처방은 **owning 모드에 한정**된다. borrowed 모드(@ref ConfigureBorrowed)에서는 이
/// 래퍼가 핸들을 들고 있지 않으므로 @ref status 가 답할 수 없고 기본값(held=false·held_ticks=0)을
/// 돌린다 — "건강함" 이 아니라 **"모름"** 이다. 그 모드의 진단은 사영 소유자에게 묻는다
/// (`WbcReducedDynamicsProvider::kinematic_status()`).
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

  // ── (#175) borrowed 모드 — 사영을 남이 돌리고 이 래퍼는 정책만 적용한다 ────────
  //
  // WBC 는 closed-chain tick 마다 `WbcReducedDynamicsProvider` 가 이미 같은 closure 를 사영한다.
  // 그 tick 에 이 래퍼까지 자기 핸들로 한 번 더 사영하면 사영이 2회가 된다 — borrowed 모드는
  // 그 두 번째를 지운다. **정책(하류 판정 · finite/trust 게이트 · hand-root 상대 · last_pose
  // hold)은 owning 모드와 같은 코드**를 타고, 달라지는 것은 사영을 누가 돌렸는가 하나뿐이다.

  /// @brief 외부 사영을 **빌려** 배선한다 (핸들 비소유, non-RT).
  ///
  /// ⚠ 이 래퍼는 @p projection 을 **저장하지 않는다** — 프레임 id·하류 판정만 뽑아 두고, 매 tick
  /// 쓸 사영은 @ref UpdateFromProjection 이 인자로 받는다. 포인터를 들고 있으면 소유자가
  /// 재구성(예: lifecycle 전이가 config 를 다시 로드)하는 순간 dangling 이 되는데, 그 증상은
  /// 크래시가 아니라 **placement 가 조용히 쓰레기값**이라 pose 가 그럴듯하게 틀린다.
  /// @param projection 배선 시점의 사영 — 프레임 해석에만 쓰이고 참조는 남지 않는다.
  /// @param fingertip_links / @param hand_root_link / @param closure_error_threshold — owning
  ///   @ref Configure 와 동일 의미. q_a 브릿지는 만들지 않는다 (사영 입력은 빌려준 쪽 소관).
  /// @return `kActive` 또는 프레임 해결 실패 사유. 소유 모드 전용 사유
  ///   (`kInactiveNoClosure`/`kInactiveConstructionFailed`/`kInactiveBridgeIncomplete`)는 나오지
  ///   않는다 — 호출측이 이미 활성 사영을 갖고 있을 때만 부르기 때문.
  [[nodiscard]] HandFkWiringResult ConfigureBorrowed(
      const rtc_urdf_bridge::RtClosedChainHandle& projection,
      std::span<const std::string> fingertip_links, std::string_view hand_root_link,
      double closure_error_threshold = 1e-3);

  /// @brief 빌린 사영 결과로 fingertip pose 캐시를 갱신한다 (**사영하지 않는다**). **RT-safe.**
  /// @param projection 이번 tick 에 쓸 사영. 호출측이 매 tick 소유자에게서 다시 얻어 넘긴다 —
  ///   그래야 소유자가 재구성해도 이 래퍼가 죽은 핸들을 읽지 않는다.
  /// @param projection_fresh 이 tick 의 사영을 신뢰해도 되는가. 호출측이 **두 사실을 AND** 해서
  ///   넘겨야 한다: (a) 이번 tick 에 사영이 실제로 돌았고(사영 소유자의 실행 카운터 증가),
  ///   (b) 그 사영의 입력 q 가 전부 이번 tick 측정값이다(**입력 provenance** — 캐시가 블록을
  ///   hold 한 tick 은 false). (a) 만으로는 부족하다: 사영이 이번 tick 에 돌아도 입력이 직전
  ///   tick 값이면 결과는 stale 인데 실행 사실만으로는 그것을 구분할 수 없다.
  /// @param kin_status 사영 **직후**(동역학 갱신 전) status. 동역학 사유의 held 를 여기 넘기면
  ///   유효한 pose 를 버리게 된다 — 사영 소유자가 스냅샷을 제공한다.
  void UpdateFromProjection(
      const rtc_urdf_bridge::RtClosedChainHandle& projection, bool projection_fresh,
      const rtc_urdf_bridge::RtClosedChainHandle::Status& kin_status) noexcept;

  /// @brief 이 래퍼가 사영 핸들을 **소유**하는가. 배선 단언용.
  /// @note 술어는 "핸들을 갖고 있는가" 이므로 **비활성 래퍼도 false** 다 — false 를 곧바로
  ///   "borrowed" 로 읽지 말 것. borrowed 판정은 `active() && !owns_projection()` 이다
  ///   (모든 호출처가 그 형태를 쓴다).
  [[nodiscard]] bool owns_projection() const noexcept { return handle_ != nullptr; }

  /// @brief 이 래퍼가 직접 돌린 사영 횟수. borrowed 모드에서는 절대 증가하지 않으므로
  ///   "tick 당 사영 1회" 를 wrapper 축에서 재는 계측 지점이다 (`rtc_urdf_bridge` 무변경).
  ///   재배선(`Configure`/`ConfigureBorrowed`) 시 0 으로 돌아간다 — 현재 배선의 횟수다.
  [[nodiscard]] std::uint32_t projection_count() const noexcept { return projection_count_; }

  /// @brief fingertip @p f 의 **hand-root 상대** placement(직전 신뢰 tick 값)를 @p out 에 기록.
  /// @return 활성 fingertip 이고 유효 캐시가 있으면 true. 비활성/OOB/캐시없음이면 false (out
  /// 미변경).
  /// **RT-safe.** loop 하류 fingertip 은 loop-consistent, 비하류 fingertip 은 full-model FK(serial
  /// 등가)로 둘 다 서비스된다 (#121: 활성 시 혼합 손의 serial 손가락도 유지).
  [[nodiscard]] bool GetFingertipHandRootPose(std::size_t f, pinocchio::SE3& out) const noexcept;

  /// @brief 직전 Update 의 loop-consistency/특이 상태.
  /// @note **owning 모드 전용**이다. borrowed 모드/비활성에서는 핸들이 없어 답할 수 없고
  ///   기본값(held=false · held_ticks=0 · closure_error=0)을 돌린다 — "건강함" 이 아니라 "모름"
  ///   이므로 그대로 진단에 쓰면 무기한 held 를 수렴 중으로 읽는다. borrowed 소비자는 사영
  ///   소유자의 status 를 읽는다 (#175).
  [[nodiscard]] rtc_urdf_bridge::RtClosedChainHandle::Status status() const noexcept;

  /// @brief 브릿지 미완성 시(kInactiveBridgeIncomplete) 매핑 실패한 관절 이름 (로깅용).
  [[nodiscard]] const std::string& missing_joint() const noexcept { return missing_joint_; }

 private:
  struct QSource {
    int device{0};   ///< ControllerState.devices index
    int channel{0};  ///< device.positions 내 channel (= joint_state_names 위치)
  };

  /// Configure 의 (a) hand-root · (b) fingertip 프레임 해석 — owning/borrowed 공용 (non-RT).
  /// 넘어온 사영 위에서 판정하므로 두 모드가 같은 topology 결론을 얻는다.
  [[nodiscard]] HandFkWiringResult ResolveFrames(const rtc_urdf_bridge::RtClosedChainHandle& src,
                                                 std::span<const std::string> fingertip_links,
                                                 std::string_view hand_root_link);

  /// 사영 결과에 정책을 적용해 fingertip pose 캐시를 갱신한다 — **두 모드의 유일한 정책 코드**.
  /// @param inputs_ok 이번 tick 사영 입력을 신뢰하는가 (owning: 소스 per-slot freshness /
  ///   borrowed: 호출측이 준 provenance).
  void ApplyProjection(const rtc_urdf_bridge::RtClosedChainHandle& src, bool inputs_ok,
                       const rtc_urdf_bridge::RtClosedChainHandle::Status& st) noexcept;

  /// 모든 배선 상태를 초기화 (threshold 제외 — Configure 진입 시 설정된다).
  void ResetWiring() noexcept;

  std::unique_ptr<rtc_urdf_bridge::RtClosedChainHandle> handle_;
  /// borrowed 모드에서 배선 시점 사영이 서 있던 모델. 매 tick 넘어오는 사영이 같은 모델 위에
  /// 있는지 debug 빌드에서 확인한다 (프레임 id 는 모델 종속이므로 다른 모델이면 엉뚱한 프레임을
  /// 읽는다). **핸들 자체는 저장하지 않는다** — 소유자가 재구성하면 dangling 이 된다.
  const pinocchio::Model* borrowed_model_{nullptr};
  /// 이 래퍼가 직접 돌린 사영 횟수 (borrowed 모드에서는 증가하지 않는다). @ref projection_count.
  std::uint32_t projection_count_{0};
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
/// @note **owning 래퍼만 구동한다** — borrowed 래퍼(@ref ClosedChainHandFk::ConfigureBorrowed)를
///   넘기면 사영할 수단이 없으므로 false 를 돌려 withhold 시킨다 (조용한 stale 방지). WBC 의
///   borrowed 경로는 이 dispatch 를 **우회**하고 `UpdateFromProjection` 을 직접 부른다: 채택
///   판정에 필요한 provenance(사영 소유자의 실행 카운터 · 컨트롤러의 device readability)가
///   task/joint 에는 없는 WBC 전용 상태라 여기 접을 수 없다 (#175).
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
