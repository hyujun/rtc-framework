// closure_test_fixtures.hpp — shared Extended-URDF closed-chain test fixtures
// (crank_rocker: 비특이 crank-rocker linkage; four_bar: 대칭 특이 4-bar). Both
// tests below build them via the same BuildClosedChainModelFromExtendedUrdf
// pipeline used in production, so φ/DoF ground truth stays in one place.
// Header-only (inline), never installed — test-compile time only.
#pragma once

#include "rtc_urdf_bridge/closed_chain_model.hpp"
#include "test_urdf_path.hpp"

namespace rtc::test {

// 비특이 crank-rocker (neutral 근방 crank 유효 구간). actuated = j_crank.
inline rtc_urdf_bridge::ClosedChainModel CrankRocker() {
  return rtc_urdf_bridge::BuildClosedChainModelFromExtendedUrdf(
      TestUrdfPath("crank_rocker.urdf"), TestUrdfPath("crank_rocker.closure.yaml"));
}

// 대칭 특이 4-bar (q_ref 특이). actuated = joint_a.
inline rtc_urdf_bridge::ClosedChainModel FourBar() {
  return rtc_urdf_bridge::BuildClosedChainModelFromExtendedUrdf(
      TestUrdfPath("four_bar_tree.urdf"), TestUrdfPath("four_bar.closure.yaml"));
}

// 도달 가능한 residual floor 를 가진 crank_rocker (#250). 평면(전 관절 z축 회전) 기구의
// closure anchor 를 z 방향으로 z_offset 만큼 어긋내면 어떤 관절 운동으로도 보상할 수 없어
// ‖φ‖ floor 가 정확히 z_offset 이 된다 — hand-description ring closure 의 ~20 nm URDF 좌표
// 불일치를 synthetic 으로 재현한다 (외부 패키지 경로 하드코딩 없이).
// ⚠ q_ref/q_ref_converged 필드는 offset **미반영** (loader 가 원본 constraint 로 계산) —
// 직접 projection 호출 테스트 용도로만 쓴다.
inline rtc_urdf_bridge::ClosedChainModel FlooredCrankRocker(double z_offset) {
  rtc_urdf_bridge::ClosedChainModel cc = CrankRocker();
  cc.constraints.front().joint2_placement.translation().z() += z_offset;
  return cc;
}

}  // namespace rtc::test
