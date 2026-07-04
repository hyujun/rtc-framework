// file: include/ur5e_bt_coordinator/hand_pose_config.hpp
#pragma once

#include <map>
#include <string>
#include <vector>

namespace rtc_bt {

// ── 상수 ────────────────────────────────────────────────────────────────────
// DoF 폭은 더 이상 컴파일타임 상수가 아니다 — RobotProfile 의 arm_dof/hand_dof
// (default kDefaultArmDof/kDefaultHandDof, robot_profile.hpp) 런타임 값이 SSoT.
inline constexpr double kMinDuration = 0.1;  // 최소 허용 duration [s]

// ── 단위 변환 ───────────────────────────────────────────────────────────────
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDeg2Rad = kPi / 180.0;

/// 포즈 벡터를 deg → rad로 변환.
inline std::vector<double> DegToRad(std::vector<double> pose_deg) {
  for (auto& v : pose_deg)
    v *= kDeg2Rad;
  return pose_deg;
}

// ── 손가락-관절 인덱스 매핑 (joint-name 기반, 가변 DoF) ─────────────────────
// per-finger DoF 를 상수로 박지 않는다. hand joint_states 의 name 을 접두사
// 매칭해 finger→joint index 리스트를 구성하므로, 손가락별 DoF 가 달라도
// (예: proto_1b thumb:4/index:3/middle:2/ring:1) URDF/joint 이름만으로 동작한다.
//
// key 규칙: `<key>_` 로 시작하는 joint 이름들의 인덱스를 joint_states 순서대로.
//   "thumb"      → thumb_* 전부           (whole-finger group)
//   "thumb_mcp"  → thumb_mcp_* 만          (sub-group; FlexExtendFinger 부분 flex)
//   "index_dip"  → index_dip_* / "ring" → ring_* …
// assm_v1 예시 (10-DoF hand joint_states 순서):
//   thumb_cmc_aa(0) thumb_cmc_fe(1) thumb_mcp_fe(2) index_mcp_aa(3) index_mcp_fe(4)
//   index_dip_fe(5) middle_mcp_aa(6) middle_mcp_fe(7) middle_dip_fe(8) ring_mcp_fe(9)
// → "thumb"={0,1,2} "thumb_mcp"={2} "index"={3,4,5} "index_dip"={5}
//   "middle"={6,7,8} "middle_dip"={8} "ring"={9}
[[nodiscard]] inline std::vector<int> FingerJointIndices(
    const std::vector<std::string>& joint_names, const std::string& key) {
  std::vector<int> out;
  const std::string prefix = key + "_";
  for (int i = 0; i < static_cast<int>(joint_names.size()); ++i) {
    if (joint_names[static_cast<std::size_t>(i)].rfind(prefix, 0) == 0) {
      out.push_back(i);
    }
  }
  return out;
}

// ── Hand 포즈 (10-DoF, 단위: deg → 자동 rad 변환) ──────────────────────────
// Placeholder 값 — 실제 하드웨어 캘리브레이션 후 교체
using HandPose = std::vector<double>;

inline const std::map<std::string, HandPose> kHandPoses = {
    //                        Thumb              Index              Middle           Ring
    //                        CMCab CMCfe MCPfe  MCPab MCPfe DIPfe  MCPab MCPfe DIPfe MCPfe
    // 기본 포즈
    {"home", DegToRad(HandPose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})},
    {"full_flex", DegToRad(HandPose{30.0, 60.0, 45.0, 0.0, 60.0, 45.0, 0.0, 60.0, 45.0, 60.0})},

    // Opposition 포즈: 엄지 → 각 손가락
    {"thumb_index_oppose", DegToRad(HandPose{15.0, 45.0, 35.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})},
    {"index_oppose", DegToRad(HandPose{0.0, 0.0, 0.0, -5.0, 40.0, 30.0, 0.0, 0.0, 0.0, 0.0})},
    {"thumb_middle_oppose",
     DegToRad(HandPose{25.0, 40.0, 30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})},
    {"middle_oppose", DegToRad(HandPose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -5.0, 40.0, 30.0, 0.0})},
    {"thumb_ring_oppose", DegToRad(HandPose{30.0, 35.0, 25.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})},
    {"ring_oppose", DegToRad(HandPose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 45.0})},

    // Flex 타겟 (FlexExtendFinger용, 손가락별)
    {"thumb_flex", DegToRad(HandPose{30.0, 60.0, 45.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})},
    {"thumb_mcp_flex", DegToRad(HandPose{0.0, 0.0, 45.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})},
    {"index_flex", DegToRad(HandPose{0.0, 0.0, 0.0, 0.0, 60.0, 45.0, 0.0, 0.0, 0.0, 0.0})},
    {"index_dip_flex", DegToRad(HandPose{0.0, 0.0, 0.0, 0.0, 0.0, 45.0, 0.0, 0.0, 0.0, 0.0})},
    {"middle_flex", DegToRad(HandPose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 60.0, 45.0, 0.0})},
    {"middle_dip_flex", DegToRad(HandPose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 45.0, 0.0})},
    {"ring_flex", DegToRad(HandPose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 60.0})},
};
// TODO: 하드웨어 캘리브레이션 후 placeholder 값 교체

// ── UR5e 포즈 (6-DoF, 단위: deg → 자동 rad 변환) ───────────────────────────
using ArmPose = std::vector<double>;

inline const std::map<std::string, ArmPose> kUR5ePoses = {
    {"home_pose", DegToRad(ArmPose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0})},
    {"demo_pose", DegToRad(ArmPose{0.0, -90.0, 90.0, -90.0, -90.0, 0.0})},
};
// TODO: 하드웨어 캘리브레이션 후 placeholder 값 교체

}  // namespace rtc_bt
