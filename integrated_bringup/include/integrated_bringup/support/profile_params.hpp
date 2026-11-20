#ifndef INTEGRATED_BRINGUP_SUPPORT_PROFILE_PARAMS_HPP_
#define INTEGRATED_BRINGUP_SUPPORT_PROFILE_PARAMS_HPP_

// A robot profile's RT-node parameters read from its files, without a node:
// the parameter files layered as a launch hands them to the controller
// manager (each file's `/**: ros__parameters`, later files over earlier ones),
// and the system model config built from their `urdf` block. For tools that
// rebuild a profile's model offline (the estimator replay) and the tests that
// bring a controller up from a shipped profile. Configure / tool time only.
//
//   urdf:
//     package: robot_descriptions      # resolved by the caller
//     path: robots/<robot>/urdf/<file>
//     root_joint_type: floating        # default "fixed"
//     passive_joints: [<joint>, ...]   # optional
//     extended: false                  # true: closure_path names the sidecar
//     closure_path: <rel path>
//     sub_models:   {<name>: {root_link, tip_link}}      # document order
//     tree_models:  {<name>: {root_link, tip_links: []}}
//     extra_frames: {<name>: {parent, xyz: [3], rpy: [3] (0), provisional (true)}}

#include "rtc_urdf_bridge/types.hpp"

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace integrated_bringup {

/// @brief Lay `src` onto `dst` key by key — a map merges, anything else
///        replaces — the way a node's parameter files overlay one another.
void MergeParamsInto(YAML::Node& dst, const YAML::Node& src);

/// @brief The `/**: ros__parameters` maps of `files`, merged in order (a later
///        file wins). A file without that map contributes nothing.
/// @throws YAML::Exception when a file cannot be read or parsed.
[[nodiscard]] YAML::Node LoadProfileParams(const std::vector<std::string>& files);

/// @brief The system model config of a profile's `urdf` block (header).
/// @param urdf_path     the resolved URDF / xacro file (`package` + `path`)
/// @param closure_path  the resolved closure sidecar; required when `extended`
/// @throws std::invalid_argument naming the key on a block that is not a map, an
///         extended model without a closure path, or a malformed model entry.
[[nodiscard]] rtc_urdf_bridge::ModelConfig ModelConfigFromUrdfParams(
    const YAML::Node& urdf, const std::string& urdf_path, const std::string& closure_path);

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_SUPPORT_PROFILE_PARAMS_HPP_
