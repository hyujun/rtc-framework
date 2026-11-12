// ── free_flyer_root: the one definition of "this model's base floats" ───────
#pragma once

#include "rtc_urdf_bridge/free_flyer_dims.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/fwd.hpp>
#pragma GCC diagnostic pop

namespace rtc_urdf_bridge {

/// @return true when the model's root (joint 1; joint 0 is the universe) is a
///         `JointModelFreeFlyer` — by joint TYPE, not by its dimensions: a
///         composite translation + spherical root has the same nq 7 / nv 6
///         but not the free-flyer's configuration layout, and is not floating
///         here. A model without a root joint is not floating. A model-level
///         fact, independent of any YAML. Allocation-free (a joint-variant type
///         check), so RT-callable helpers may use it.
[[nodiscard]] bool IsFreeFlyerRoot(const pinocchio::Model& model) noexcept;

}  // namespace rtc_urdf_bridge
