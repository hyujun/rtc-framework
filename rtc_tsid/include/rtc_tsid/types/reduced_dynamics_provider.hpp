// ── ReducedDynamicsProvider: PinocchioCache 의 M/h/g 를 축약(constraint-consistent)
//    동역학으로 덮어쓰기 위한 추상 주입점 ────────────────────────────────────────
#pragma once

#include <Eigen/Core>

namespace rtc::tsid {

/// @brief closed-chain 축약 동역학을 PinocchioCache 에 주입하는 추상 인터페이스.
///
/// 동기: `rtc_tsid` 는 `rtc_urdf_bridge` 를 의존하지 않는다 (ARCH-2 의존 방향). 따라서
/// PinocchioCache 가 `RtClosedChainHandle` 을 직접 참조할 수 없다. 대신 이 순수 인터페이스를
/// 두고, 구체 구현(closed-chain 핸들 소유)은 두 패키지를 모두 의존하는 상위 패키지
/// (`integrated_bringup`) 에 둔다.
///
/// PinocchioCache::Update 는 open-chain M/h/g (crba/nle/gravity) 를 채운 **직후**, provider 가
/// set 돼 있으면 `FillReducedDynamics` 를 호출한다. provider 가 true 를 반환하면 M/h/g 는
/// constraint-consistent 축약값으로 덮어써진 상태이고, false 면 open-chain 값이 그대로 유지된다
/// (graceful fallback — 최초 tick / 사영 실패 / 정렬 미매칭).
///
/// ### RT 계약
/// - `FillReducedDynamics` 는 RT 정기 tick 에서 매번 호출된다 → **noexcept, 힙 할당 금지**.
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
};

}  // namespace rtc::tsid
