// ── Test models of the profiles whose shipped model is not in this repo ──────
//
// `ur5e_p1b` and `g1_p1b` load their model from `hand_description`, a separate
// project this workspace does not depend on, whose files are not public. Their
// tests load these instead, `robot_descriptions` share-relative: the same arm or
// G1 body with `robots/p1b_surrogate/`, a SYNTHETIC hand that has the real hand's
// link and joint names and loop topology but made-up dimensions and inertias
// (that directory's README). Everything else a test reads from the profile —
// sub-models, tree models, extra frames, devices — stays the shipped YAML's.
//
// A loader that reads a profile's `urdf:` block resolves its files through
// TestModelFiles, so a profile whose shipped model moves (into this repo, or to
// another file) fails loudly instead of silently keeping the stand-in.
#pragma once

#include <array>
#include <stdexcept>
#include <string>

namespace integrated_bringup::testfx {

inline constexpr const char* kUr5eP1bUrdfRel =
    "robots/p1b_surrogate/urdf/ur5e_with_p1b_surrogate.urdf.xacro";
inline constexpr const char* kUr5eP1bClosureRel =
    "robots/p1b_surrogate/urdf/ur5e_with_p1b_surrogate.closure.yaml";

inline constexpr const char* kG1P1bUrdfRel =
    "robots/g1/urdf/g1_with_p1b_surrogate_fixed_base.urdf.xacro";
inline constexpr const char* kG1P1bClosureRel =
    "robots/g1/urdf/g1_with_p1b_surrogate_fixed_base.closure.yaml";

/// One profile whose shipped model a test model stands in for.
struct TestModel {
  const char* profile;
  const char* shipped_package;  // the profile's `urdf.package`
  const char* shipped_path;     // its `urdf.path`
  const char* urdf_rel;         // the stand-in, robot_descriptions share-relative
  const char* closure_rel;
};

inline constexpr std::array<TestModel, 2> kTestModels = {{
    {"ur5e_p1b", "hand_description", "robots/ur5e_p1b/urdf/ur5e_with_proto_1b.urdf.xacro",
     kUr5eP1bUrdfRel, kUr5eP1bClosureRel},
    {"g1_p1b", "hand_description",
     "robots/unitree_g1_p1b/urdf/g1_with_proto_1b_fixed_base.urdf.xacro", kG1P1bUrdfRel,
     kG1P1bClosureRel},
}};

/// The files a test opens, `robot_descriptions` share-relative.
struct ModelFiles {
  std::string urdf_rel;
  std::string closure_rel;  // empty for a plain URDF
};

/// The files a test opens for `profile`'s shipped `urdf.package` / `urdf.path`
/// / `urdf.closure_path` (pass "" when the profile is not `extended`). A model
/// already in `robot_descriptions` is opened as shipped; any other must be the
/// one a kTestModels row stands in for, or this throws.
inline ModelFiles TestModelFiles(const std::string& profile, const std::string& package,
                                 const std::string& path, const std::string& closure_path) {
  if (package == "robot_descriptions") {
    return {path, closure_path};
  }
  for (const TestModel& m : kTestModels) {
    if (profile == m.profile) {
      if (package != m.shipped_package || path != m.shipped_path) {
        throw std::runtime_error(profile + ": the shipped model is now " + package + "/" + path +
                                 ", not the " + m.shipped_package + "/" + m.shipped_path +
                                 " this test model stands in for — update kTestModels");
      }
      return {m.urdf_rel, closure_path.empty() ? std::string() : std::string(m.closure_rel)};
    }
  }
  throw std::runtime_error(profile + ": the shipped model is in package '" + package +
                           "', and no test model stands in for it (p1b_surrogate_models.hpp)");
}

}  // namespace integrated_bringup::testfx
