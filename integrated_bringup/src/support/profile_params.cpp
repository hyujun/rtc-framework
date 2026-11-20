#include "integrated_bringup/support/profile_params.hpp"

#include <Eigen/Core>

#include <stdexcept>
#include <string>
#include <vector>

namespace integrated_bringup {

void MergeParamsInto(YAML::Node& dst, const YAML::Node& src) {
  for (auto it = src.begin(); it != src.end(); ++it) {
    const auto key = it->first.as<std::string>();
    YAML::Node slot = dst[key];
    if (it->second.IsMap() && slot.IsDefined() && slot.IsMap()) {
      MergeParamsInto(slot, it->second);
    } else {
      dst[key] = YAML::Clone(it->second);
    }
  }
}

YAML::Node LoadProfileParams(const std::vector<std::string>& files) {
  YAML::Node merged(YAML::NodeType::Map);
  for (const auto& file : files) {
    const YAML::Node params = YAML::LoadFile(file)["/**"]["ros__parameters"];
    if (params && params.IsMap()) {
      MergeParamsInto(merged, params);
    }
  }
  return merged;
}

namespace {

[[noreturn]] void Refuse(const std::string& key, const std::string& why) {
  throw std::invalid_argument("urdf." + key + ": " + why);
}

// A map entry, or an empty map when absent; a present non-map is refused.
YAML::Node OptionalMap(const YAML::Node& urdf, const std::string& key) {
  const YAML::Node node = urdf[key];
  if (!node) {
    return YAML::Node(YAML::NodeType::Map);
  }
  if (!node.IsMap()) {
    Refuse(key, "expected a map");
  }
  return node;
}

}  // namespace

rtc_urdf_bridge::ModelConfig ModelConfigFromUrdfParams(const YAML::Node& urdf,
                                                       const std::string& urdf_path,
                                                       const std::string& closure_path) {
  if (!urdf || !urdf.IsMap()) {
    throw std::invalid_argument("urdf: expected a map");
  }
  rtc_urdf_bridge::ModelConfig cfg;
  cfg.urdf_path = urdf_path;
  cfg.root_joint_type = urdf["root_joint_type"].as<std::string>("fixed");
  if (urdf["passive_joints"]) {
    cfg.passive_joints = urdf["passive_joints"].as<std::vector<std::string>>();
  }
  if (urdf["extended"].as<bool>(false)) {
    if (closure_path.empty()) {
      Refuse("closure_path", "an extended model needs its closure sidecar");
    }
    cfg.closure_yaml_path = closure_path;
  }
  try {
    // Document order: a controller takes the FIRST sub-model as the arm.
    const YAML::Node subs = OptionalMap(urdf, "sub_models");
    for (auto it = subs.begin(); it != subs.end(); ++it) {
      cfg.sub_models.push_back({it->first.as<std::string>(),
                                it->second["root_link"].as<std::string>(),
                                it->second["tip_link"].as<std::string>()});
    }
    const YAML::Node trees = OptionalMap(urdf, "tree_models");
    for (auto it = trees.begin(); it != trees.end(); ++it) {
      cfg.tree_models.push_back({it->first.as<std::string>(),
                                 it->second["root_link"].as<std::string>(),
                                 it->second["tip_links"].as<std::vector<std::string>>()});
    }
    const YAML::Node frames = OptionalMap(urdf, "extra_frames");
    for (auto it = frames.begin(); it != frames.end(); ++it) {
      rtc_urdf_bridge::ExtraFrameConfig frame;
      frame.name = it->first.as<std::string>();
      frame.parent = it->second["parent"].as<std::string>();
      const auto xyz = it->second["xyz"].as<std::vector<double>>();
      const auto rpy = it->second["rpy"].as<std::vector<double>>(std::vector<double>(3, 0.0));
      if (xyz.size() != 3 || rpy.size() != 3) {
        Refuse("extra_frames." + frame.name, "xyz and rpy take 3 values");
      }
      frame.xyz = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
      frame.rpy = Eigen::Vector3d(rpy[0], rpy[1], rpy[2]);
      frame.provisional = it->second["provisional"].as<bool>(true);
      cfg.extra_frames.push_back(frame);
    }
  } catch (const YAML::Exception& e) {
    throw std::invalid_argument(std::string("urdf: a malformed model entry: ") + e.what());
  }
  return cfg;
}

}  // namespace integrated_bringup
