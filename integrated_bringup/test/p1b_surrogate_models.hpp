// ── Test models of the profiles whose shipped model is not in this repo ──────
//
// `ur5e_p1b` and `g1_p1b` load their model from `hand_description`, a separate
// project this workspace does not depend on, whose files are not public. Their
// tests load these instead, `robot_descriptions` share-relative: the same arm or
// G1 body with `robots/p1b_surrogate/`, a SYNTHETIC hand that has the real hand's
// link and joint names and loop topology but made-up dimensions and inertias
// (that directory's README). Everything else a test reads from the profile —
// sub-models, tree models, extra frames, devices — stays the shipped YAML's.
#pragma once

namespace integrated_bringup::testfx {

inline constexpr const char* kUr5eP1bUrdfRel =
    "robots/p1b_surrogate/urdf/ur5e_with_p1b_surrogate.urdf.xacro";
inline constexpr const char* kUr5eP1bClosureRel =
    "robots/p1b_surrogate/urdf/ur5e_with_p1b_surrogate.closure.yaml";

inline constexpr const char* kG1P1bUrdfRel =
    "robots/g1/urdf/g1_with_p1b_surrogate_fixed_base.urdf.xacro";
inline constexpr const char* kG1P1bClosureRel =
    "robots/g1/urdf/g1_with_p1b_surrogate_fixed_base.closure.yaml";

}  // namespace integrated_bringup::testfx
