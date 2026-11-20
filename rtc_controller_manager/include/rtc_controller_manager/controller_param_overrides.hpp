// The bridge from ROS parameters to a controller's YAML configuration.
//
// A controller's configuration is a YAML file the controller manager loads; it
// is not part of the ROS parameter tree. A launch argument or a params overlay
// that names `<config_key>.<dotted.path>` reaches it only through this function,
// which the manager calls on the composed tree before the controller parses it.
// Declared here so that a test of a shipped overlay can push it through the
// same code the node runs, rather than through a copy of it.
#ifndef RTC_CONTROLLER_MANAGER_CONTROLLER_PARAM_OVERRIDES_HPP_
#define RTC_CONTROLLER_MANAGER_CONTROLLER_PARAM_OVERRIDES_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <yaml-cpp/yaml.h>

#include <string>

namespace rtc {

/// @brief Write every parameter of @p node named `<config_key>.<path>` into
///        @p ctrl_node at `<path>` (`.` separates YAML map keys).
///
/// Every path is judged against the tree as loaded before anything is written:
/// a path the tree lacks is warned about and written; a path that would break
/// the tree's structure (through a scalar or a sequence) is an error and then
/// NOTHING is written. Bool, integer, double and string parameters and their
/// arrays are written with their own type; other types are skipped.
///
/// @param node        the node whose parameter overrides are read
/// @param ctrl_node   the controller's composed YAML tree, modified in place
///                    (YAML::Node is a handle: the caller's tree changes)
/// @param config_key  the controller's key, the parameter name prefix
/// @note Not RT-safe: allocates and logs. Called at configure.
void ApplyControllerParamOverrides(rclcpp_lifecycle::LifecycleNode& node, YAML::Node ctrl_node,
                                   const std::string& config_key);

}  // namespace rtc

#endif  // RTC_CONTROLLER_MANAGER_CONTROLLER_PARAM_OVERRIDES_HPP_
