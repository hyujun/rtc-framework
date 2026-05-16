#ifndef INTEGRATED_BRINGUP_CONTROLLERS_TOF_SNAPSHOT_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_TOF_SNAPSHOT_HPP_

// ToF snapshot data — produced by demo_* controllers when the ur5e_hand
// fingertip ToF sensor stack is active. Lives in integrated_bringup
// (not rtc_base) because ToF is a hand-domain payload specific to this
// integration; rtc_base stays robot-agnostic.
//
// Trivially copyable so SeqLock<ToFSnapshotData> can memcpy on the RT
// publish path.

#include "rtc_base/types/types.hpp"  // rtc::Pose

#include <array>
#include <type_traits>

namespace integrated_bringup {

// ToF snapshot data — fixed-size POD, RT-safe.
// 상한값(kMax*) 기반 고정 배열 + 런타임 num_fingers/sensors_per_finger
struct ToFSnapshotData {
  static constexpr int kMaxFingers = 8;                                        // upper bound
  static constexpr int kMaxSensorsPerFinger = 3;                               // upper bound
  static constexpr int kMaxTotalSensors = kMaxFingers * kMaxSensorsPerFinger;  // 24

  // 거리 [m]
  std::array<double, kMaxTotalSensors> distances{};
  std::array<bool, kMaxTotalSensors> valid{};

  // Fingertip SE3 poses — world frame. Re-uses the free-standing rtc::Pose
  // type from rtc_base so other consumers can share.
  std::array<rtc::Pose, kMaxFingers> tip_poses{};

  int num_fingers{0};         // 실제 사용 핑거 수
  int sensors_per_finger{0};  // 실제 핑거당 센서 수
  bool populated{false};      // true when controller has valid ToF + FK data
};

static_assert(std::is_trivially_copyable_v<ToFSnapshotData>,
              "ToFSnapshotData must be trivially copyable for SeqLock");

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_TOF_SNAPSHOT_HPP_
