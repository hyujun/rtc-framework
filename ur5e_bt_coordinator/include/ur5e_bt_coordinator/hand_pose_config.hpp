// file: include/ur5e_bt_coordinator/hand_pose_config.hpp
#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

namespace rtc_bt {

// ── 상수 ────────────────────────────────────────────────────────────────────
inline constexpr int kHandDofCount = 10;
inline constexpr int kArmDofCount = 6;
inline constexpr double kMinDuration = 0.1;  // 최소 허용 duration [s]

// ── 단위 변환 ───────────────────────────────────────────────────────────────
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDeg2Rad = kPi / 180.0;

/// 포즈 배열을 deg → rad로 변환 (컴파일 타임)
template <std::size_t N>
inline constexpr std::array<double, N> DegToRad(std::array<double, N> pose_deg) {
  for (auto& v : pose_deg) v *= kDeg2Rad;
  return pose_deg;
}

// ── 손가락-관절 인덱스 매핑 ─────────────────────────────────────────────────
// | Finger | Joints                                  | DoF | Index |
// |--------|-----------------------------------------|-----|-------|
// | Thumb  | CMC abd/add, CMC flex/ext, MCP flex/ext | 3   | 0–2   |
// | Index  | MCP abd/add, MCP flex/ext, DIP flex/ext | 3   | 3–5   |
// | Middle | MCP abd/add, MCP flex/ext, DIP flex/ext | 3   | 6–8   |
// | Ring   | MCP flex/ext                            | 1   | 9     |
inline const std::map<std::string, std::vector<int>> kFingerJointIndices = {
    {"thumb",      {0, 1, 2}},
    {"thumb_mcp",  {2}},
    {"index",      {3, 4, 5}},
    {"index_dip",  {5}},
    {"middle",     {6, 7, 8}},
    {"middle_dip", {8}},
    {"ring",       {9}},
};

// ── Hand 포즈 (10-DoF, 단위: deg → 자동 rad 변환) ──────────────────────────
// Placeholder 값 — 실제 하드웨어 캘리브레이션 후 교체
using HandPose = std::array<double, kHandDofCount>;

inline const std::map<std::string, HandPose> kHandPoses = {
    //                        Thumb              Index              Middle           Ring
    //                        CMCab CMCfe MCPfe  MCPab MCPfe DIPfe  MCPab MCPfe DIPfe MCPfe
    // 기본 포즈
    {"home",          DegToRad(HandPose{ 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0})},
    {"full_flex",     DegToRad(HandPose{30.0, 60.0, 45.0,   0.0, 60.0, 45.0,   0.0, 60.0, 45.0,  60.0})},

    // Opposition 포즈: 엄지 → 각 손가락
    {"thumb_index_oppose",  DegToRad(HandPose{15.0, 45.0, 35.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0})},
    {"index_oppose",        DegToRad(HandPose{ 0.0,  0.0,  0.0,  -5.0, 40.0, 30.0,   0.0,  0.0,  0.0,   0.0})},
    {"thumb_middle_oppose", DegToRad(HandPose{25.0, 40.0, 30.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0})},
    {"middle_oppose",       DegToRad(HandPose{ 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,  -5.0, 40.0, 30.0,   0.0})},
    {"thumb_ring_oppose",   DegToRad(HandPose{30.0, 35.0, 25.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0})},
    {"ring_oppose",         DegToRad(HandPose{ 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0,  45.0})},

    // Flex 타겟 (FlexExtendFinger용, 손가락별)
    {"thumb_flex",     DegToRad(HandPose{30.0, 60.0, 45.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0})},
    {"thumb_mcp_flex", DegToRad(HandPose{ 0.0,  0.0, 45.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0})},
    {"index_flex",     DegToRad(HandPose{ 0.0,  0.0,  0.0,   0.0, 60.0, 45.0,   0.0,  0.0,  0.0,   0.0})},
    {"index_dip_flex", DegToRad(HandPose{ 0.0,  0.0,  0.0,   0.0,  0.0, 45.0,   0.0,  0.0,  0.0,   0.0})},
    {"middle_flex",    DegToRad(HandPose{ 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0, 60.0, 45.0,   0.0})},
    {"middle_dip_flex",DegToRad(HandPose{ 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0, 45.0,   0.0})},
    {"ring_flex",      DegToRad(HandPose{ 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0,  60.0})},
};
// TODO: 하드웨어 캘리브레이션 후 placeholder 값 교체

// ── UR5e 포즈 (6-DoF, 단위: deg → 자동 rad 변환) ───────────────────────────
using ArmPose = std::array<double, kArmDofCount>;

inline const std::map<std::string, ArmPose> kUR5ePoses = {
    {"home_pose", DegToRad(ArmPose{  0.0,   0.0,   0.0,   0.0,   0.0,  0.0})},
    {"demo_pose", DegToRad(ArmPose{  0.0, -90.0,  90.0, -90.0, -90.0,  0.0})},
};
// TODO: 하드웨어 캘리브레이션 후 placeholder 값 교체

}  // namespace rtc_bt
