// ── SE(3) wrench transform (Eigen-only, RT-safe) ────────────────────────────
// A wrench does NOT transform like a twist. Twists ride the adjoint Ad_T; a
// wrench is the dual (co-vector) and rides its inverse-transpose Ad_T^{-T}, so
// that mechanical power ⟨ν, f⟩ is frame-invariant (power duality). Confusing
// Ad_T^{-T} with Ad_T^{T} is the single most common wrench bug — the two are
// different maps. This module fixes the direction with tests (power duality),
// never by assuming a library's act/actInv orientation.
//
// Ordering is [linear; angular] for twists ⇔ [force; torque] for wrenches,
// identical to se3.hpp / pinocchio::Motion|Force. Do NOT mix with the
// [angular, linear] convention.
//
// References: Murray–Li–Sastry 1994 §2.5 (wrench = co-vector, Ad^{-T});
// Lynch & Park 2017 §3.4 (spatial force / power invariance); Featherstone 2008.
#pragma once

#include "rtc_math/se3/se3.hpp"

namespace rtc::math::se3 {

// ── transformWrench ─────────────────────────────────────────────────────────
//
// Given T = ᴬT_B (frame B expressed in frame A) and a wrench ᴮf expressed in B,
// return ᴬf expressed in A:
//
//   ᴬf = Ad_{ᴬT_B}^{-T} · ᴮf = Ad_{ᴮT_A}^{T} · ᴮf,   power-dual to ᴬν = Ad_{ᴬT_B}·ᴮν.
//
// We avoid a numeric 6×6 inverse via the group identity Ad_T^{-1} = Ad_{T^{-1}}:
//   Ad_T^{-T} = (Ad_{T^{-1}})^T = adjoint(T.inverse()).transpose().
// This is exact (no matrix inversion) and equals the expanded block form
//   [[R, 0], [[p]×R, R]]   (with R = ᴬR_B, p = ᴬp_B) — a moment picks up the
// lever-arm coupling p×f. The two forms are pinned equal by the tests.
[[nodiscard]] inline Vec6 transformWrench(const Iso3& T, const Vec6& f) noexcept {
  return adjoint(T.inverse()).transpose() * f;
}

}  // namespace rtc::math::se3
