// ── free_flyer_dims: the free-flyer root's sizes, with no pinocchio include ──
#pragma once

namespace rtc_urdf_bridge {

/// A pinocchio free-flyer root: `[p_xyz, quaternion_xyzw]` in the
/// configuration, a six-dimensional spatial velocity in the tangent. Every
/// consumer's base size (rtc_tsid's `n_base_q / n_base_v`, rtc_mpc's layout
/// and capacity constants) is these when `IsFreeFlyerRoot` (free_flyer_root.hpp)
/// holds and 0 otherwise (#1016). Kept apart from that header so a std-only
/// RT interchange header can take the sizes without pinocchio.
inline constexpr int kFreeFlyerNq = 7;
inline constexpr int kFreeFlyerNv = 6;

}  // namespace rtc_urdf_bridge
