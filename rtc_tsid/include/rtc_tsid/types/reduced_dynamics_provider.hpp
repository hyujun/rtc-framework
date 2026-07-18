// ── compat shim ─────────────────────────────────────────────────────────────
// #unified-kindyn Phase 1: ReducedDynamicsProvider 는 rtc_urdf_bridge 로 이관됐다.
// 기존 `#include "rtc_tsid/types/reduced_dynamics_provider.hpp"` 사용처(integrated_bringup 등)의
// 호환을 위해 bridge 헤더를 재include 하고 `rtc::tsid::ReducedDynamicsProvider` 심볼을 재export
// 한다.
#pragma once

#include "rtc_urdf_bridge/reduced_dynamics_provider.hpp"

namespace rtc::tsid {
using rtc_urdf_bridge::ReducedDynamicsProvider;
}  // namespace rtc::tsid
