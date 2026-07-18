// ── ReducedDynamicsProvider: PinocchioCache 의 M/h/g (및 Phase ③ 이후 contact frame
//    J·oMf) 를 축약(constraint-consistent) 값으로 덮어쓰기 위한 추상 주입점 ──────────
//
// Unified kin&dyn 솔버 Phase 1: rtc_tsid → rtc_urdf_bridge 이관 (PinocchioCache 와 동거).
// rtc::tsid::ReducedDynamicsProvider 심볼은 rtc_tsid 측 alias 재export 로 유지.
#pragma once

#include "rtc_urdf_bridge/pinocchio_cache.hpp"

#include <Eigen/Core>

#include <vector>

namespace rtc_urdf_bridge {

/// @brief closed-chain 축약 동역학을 PinocchioCache 에 주입하는 추상 인터페이스.
///
/// 동기: 원래 `rtc_tsid` 는 `rtc_urdf_bridge` 를 의존하지 않아 PinocchioCache 가
/// `RtClosedChainHandle` 을 직접 참조할 수 없었고, 그 결합을 이 순수 인터페이스로 끊었다.
/// Phase 1 이후 두 타입은 같은 패키지에 살지만, 구체 구현(closed-chain 핸들 소유·좌표 permutation)
/// 은 여전히 두 계층을 아우르는 상위 패키지(`integrated_bringup`)에 두어 provider 는 추상으로
/// 유지한다.
///
/// PinocchioCache::Update 는 open-chain M/h/g (crba/nle/gravity) 를 채운 **직후**, provider 가
/// set 돼 있으면 `FillReducedDynamics` 를 호출한다. provider 가 true 를 반환하면 M/h/g 는
/// constraint-consistent 축약값으로 덮어써진 상태이고, false 면 open-chain 값이 그대로 유지된다
/// (graceful fallback — 최초 tick / 사영 실패 / 정렬 미매칭).
///
/// ### RT 계약
/// - 콜백은 RT 정기 tick 에서 매번 호출된다 → **noexcept, 힙 할당 금지**.
/// - `q`/`v` 는 **cache 모델(control model) 좌표**로 주어진다 (PinocchioCache::model_ptr 기준).
/// - `M`(nv×nv)/`h`(nv)/`g`(nv) 도 같은 좌표계·차원. provider 는 자신의 축약 결과(핸들 독립좌표
///   순서)를 이 좌표계로 permutation 해 덮어쓸 책임이 있다.
class ReducedDynamicsProvider {
 public:
  virtual ~ReducedDynamicsProvider() = default;

  /// @brief open-chain M/h/g 를 constraint-consistent 축약값으로 덮어쓴다.
  /// @param q cache 모델 좌표의 configuration (nq). read-only.
  /// @param v cache 모델 좌표의 velocity (nv). read-only.
  /// @param M in/out: 덮어쓰기 대상 질량행렬 (nv × nv, cache 좌표).
  /// @param h in/out: 비선형효과 (nv).
  /// @param g in/out: 중력 (nv).
  /// @return true → M/h/g 를 축약값으로 갱신함. false → 미변경 (open-chain 유지).
  /// @note **RT-safe.** noexcept, 힙 할당 없음.
  [[nodiscard]] virtual bool FillReducedDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                 Eigen::MatrixXd& M, Eigen::VectorXd& h,
                                                 Eigen::VectorXd& g) noexcept = 0;

  /// @brief (Phase ③ 대비 — 계약 선확정) closed-chain 활성 시 contact frame 의 J·oMf 를
  ///   loop-consistent 축약값(J_a = J_full·G, 6×n_a permutation)으로 덮는다.
  ///
  /// PinocchioCache::Update 가 open-chain contact frame Jacobian/oMf 를 채운 **직후** 호출된다.
  /// 기본 구현은 no-op(false) → open-chain(frozen-loop 근사) 값 유지. serial 손(G=identity 등가)
  /// 이나 provider 미구현 시 byte-for-byte 불변. Phase ③ 에서 WbcReducedDynamicsProvider 가
  /// override 해 loop-consistent J/oMf 를 주입한다 (dJv 는 open-chain 유지 — 명시적 한계).
  ///
  /// @param q cache 모델 좌표 configuration (nq). read-only.
  /// @param v cache 모델 좌표 velocity (nv). read-only.
  /// @param contact_frames in/out: cache 의 contact frame 캐시 (인덱스=contact index). 활성
  ///   loop-downstream frame 의 J/oMf 를 덮되, 비하류/비활성은 미변경.
  /// @return true → 하나 이상의 frame 을 덮음. false → 미변경 (open-chain 유지).
  /// @note **RT-safe.** noexcept, 힙 할당 없음.
  [[nodiscard]] virtual bool FillReducedFrameKinematics(
      const Eigen::VectorXd& q, const Eigen::VectorXd& v,
      std::vector<PinocchioCache::FrameCache>& contact_frames) noexcept {
    (void)q;
    (void)v;
    (void)contact_frames;
    return false;
  }
};

}  // namespace rtc_urdf_bridge
