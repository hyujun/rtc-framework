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
inline constexpr double kMinDuration = 0.3;  // 최소 허용 duration [s]

// ── 손가락-관절 인덱스 매핑 ─────────────────────────────────────────────────
// | Finger | Joints                                  | DoF | Index |
// |--------|-----------------------------------------|-----|-------|
// | Thumb  | CMC abd/add, CMC flex/ext, MCP flex/ext | 3   | 0–2   |
// | Index  | MCP abd/add, MCP flex/ext, DIP flex/ext | 3   | 3–5   |
// | Middle | MCP abd/add, MCP flex/ext, DIP flex/ext | 3   | 6–8   |
// | Ring   | MCP flex/ext                            | 1   | 9     |
inline const std::map<std::string, std::vector<int>> kFingerJointIndices = {
    {"thumb",  {0, 1, 2}},
    {"index",  {3, 4, 5}},
    {"middle", {6, 7, 8}},
    {"ring",   {9}},
};

// ── Hand 포즈 (10-DoF) ─────────────────────────────────────────────────────
// Placeholder 값 — 실제 하드웨어 캘리브레이션 후 교체
using HandPose = std::array<double, kHandDofCount>;

inline const std::map<std::string, HandPose> kHandPoses = {
    // 기본 포즈
    {"home",                 {0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0}},
    {"full_flex",            {0.5, 1.0, 0.8,  0.0, 1.0, 0.8,  0.0, 1.0, 0.8,  1.0}},

    // Opposition 포즈: 엄지 → 각 손가락
    {"thumb_index_oppose",   {0.3, 0.8, 0.6,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0}},
    {"index_oppose",         {0.0, 0.0, 0.0, -0.1, 0.7, 0.5,  0.0, 0.0, 0.0,  0.0}},
    {"thumb_middle_oppose",  {0.4, 0.7, 0.5,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0}},
    {"middle_oppose",        {0.0, 0.0, 0.0,  0.0, 0.0, 0.0, -0.1, 0.7, 0.5,  0.0}},
    {"thumb_ring_oppose",    {0.5, 0.6, 0.4,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0}},
    {"ring_oppose",          {0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.8}},

    // Flex 타겟 (FlexExtendFinger용, 손가락별)
    {"thumb_flex",           {0.5, 1.0, 0.8,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0}},
    {"index_flex",           {0.0, 0.0, 0.0,  0.0, 1.0, 0.8,  0.0, 0.0, 0.0,  0.0}},
    {"middle_flex",          {0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 1.0, 0.8,  0.0}},
    {"ring_flex",            {0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  1.0}},
};
// TODO: 하드웨어 캘리브레이션 후 placeholder 값 교체

// ── UR5e 포즈 (6-DoF) ──────────────────────────────────────────────────────
using ArmPose = std::array<double, kArmDofCount>;

inline const std::map<std::string, ArmPose> kUR5ePoses = {
    {"home_pose", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
    {"demo_pose", {0.0, -1.57, 1.57, -1.57, -1.57, 0.0}},
};
// TODO: 하드웨어 캘리브레이션 후 placeholder 값 교체

}  // namespace rtc_bt
