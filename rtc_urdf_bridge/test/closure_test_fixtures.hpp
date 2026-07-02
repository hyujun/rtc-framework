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

}  // namespace rtc::test
