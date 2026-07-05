// file: include/ur5e_bt_coordinator/finger_resolver.hpp
#pragma once

#include "ur5e_bt_coordinator/hand_pose_config.hpp"  // FingerJointIndices

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace rtc_bt {

/// Strategy that maps a finger key (e.g. "thumb", "index_dip") to the list of
/// hand-joint indices it controls, in hand joint_states order (Seam C).
///
/// ARCH-3: the abstract interface is defined first; PrefixFingerResolver and
/// ExplicitFingerResolver are the first concrete pair, so the coordinator never
/// branches on a hardcoded switch / #ifdef to pick a hand's finger layout.
class FingerResolver {
 public:
  virtual ~FingerResolver() = default;

  /// Indices (into `joint_names` order) of the joints belonging to `key`.
  /// Empty when the finger is unknown (callers treat empty as an error).
  [[nodiscard]] virtual std::vector<int> Resolve(const std::vector<std::string>& joint_names,
                                                 const std::string& key) const = 0;
};

/// Default strategy: prefix-match `<key>_` against the joint_states names
/// (legacy behavior). Works when the URDF joint names encode the finger, e.g.
/// the ur5e assm_v1 / proto_1b hands ("thumb_cmc_aa", "index_mcp_fe", ...).
class PrefixFingerResolver : public FingerResolver {
 public:
  [[nodiscard]] std::vector<int> Resolve(const std::vector<std::string>& joint_names,
                                         const std::string& key) const override {
    return FingerJointIndices(joint_names, key);
  }
};

/// Config-driven strategy: an explicit finger -> joint-index map. Ignores the
/// joint_names (the map is authoritative), so it works for hands whose joint
/// names carry no finger prefix — e.g. the LEAP hand's numeric names
/// ("0".."15"). Sub-group keys (e.g. "thumb_mcp") must be listed explicitly.
class ExplicitFingerResolver : public FingerResolver {
 public:
  explicit ExplicitFingerResolver(std::map<std::string, std::vector<int>> finger_map)
      : finger_map_(std::move(finger_map)) {}

  [[nodiscard]] std::vector<int> Resolve(const std::vector<std::string>& /*joint_names*/,
                                         const std::string& key) const override {
    auto it = finger_map_.find(key);
    return (it != finger_map_.end()) ? it->second : std::vector<int>{};
  }

 private:
  std::map<std::string, std::vector<int>> finger_map_;
};

}  // namespace rtc_bt
