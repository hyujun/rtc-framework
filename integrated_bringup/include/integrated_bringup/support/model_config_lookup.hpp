#ifndef INTEGRATED_BRINGUP_SUPPORT_MODEL_CONFIG_LOOKUP_HPP_
#define INTEGRATED_BRINGUP_SUPPORT_MODEL_CONFIG_LOOKUP_HPP_

// Lookups on the system model config (`urdf.sub_models` / `urdf.tree_models`)
// by name. A device group and the model declared for it share one name, so
// "the tree of the secondary group" is a lookup every demo controller makes;
// a controller key may name a model too (the catching planner's). Configure
// time only, never on the RT path.

#include "rtc_urdf_bridge/types.hpp"

#include <string>

namespace integrated_bringup {

/// @brief The `urdf.tree_models` entry called @p name.
/// @return a pointer into @p config (valid while it lives), or nullptr when no
///         tree model carries that name — including an empty @p name.
[[nodiscard]] inline const rtc_urdf_bridge::TreeModelConfig* FindTreeModel(
    const rtc_urdf_bridge::ModelConfig& config, const std::string& name) noexcept {
  if (name.empty()) {
    return nullptr;
  }
  for (const auto& tm : config.tree_models) {
    if (tm.name == name) {
      return &tm;
    }
  }
  return nullptr;
}

/// @brief The `urdf.sub_models` entry (a serial chain, root → tip) called
///        @p name.
/// @return a pointer into @p config (valid while it lives), or nullptr when no
///         sub-model carries that name — including an empty @p name.
[[nodiscard]] inline const rtc_urdf_bridge::SubModelConfig* FindSubModel(
    const rtc_urdf_bridge::ModelConfig& config, const std::string& name) noexcept {
  if (name.empty()) {
    return nullptr;
  }
  for (const auto& sm : config.sub_models) {
    if (sm.name == name) {
      return &sm;
    }
  }
  return nullptr;
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_SUPPORT_MODEL_CONFIG_LOOKUP_HPP_
