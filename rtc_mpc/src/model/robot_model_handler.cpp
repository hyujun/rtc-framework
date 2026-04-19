#include "rtc_mpc/model/robot_model_handler.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/fwd.hpp>
#pragma GCC diagnostic pop

namespace rtc::mpc {

namespace {

// Find a frame by name without throwing. Pinocchio's `getFrameId` returns
// `model.frames.size()` when the name is absent; wrap that into std::optional.
std::optional<int> FindFrameId(const pinocchio::Model &m,
                               const std::string &name) noexcept {
  const auto id = m.getFrameId(name);
  if (id >= static_cast<pinocchio::FrameIndex>(m.frames.size())) {
    return std::nullopt;
  }
  return static_cast<int>(id);
}

} // namespace

RobotModelInitError RobotModelHandler::Init(const pinocchio::Model &model,
                                            const YAML::Node &cfg) noexcept {
  if (model_ != nullptr) {
    return RobotModelInitError::kModelAlreadyInitialised;
  }

  // yaml-cpp throws on .as<T>() when the node is missing/wrong-typed; all
  // probing here is nothrow (IsDefined / IsScalar / IsSequence / IsMap).
  if (!cfg.IsMap()) {
    return RobotModelInitError::kInvalidYamlSchema;
  }

  const YAML::Node ee_node = cfg["end_effector_frame"];
  if (!ee_node.IsDefined() || !ee_node.IsScalar()) {
    return RobotModelInitError::kInvalidYamlSchema;
  }
  std::string ee_name;
  try {
    ee_name = ee_node.as<std::string>();
  } catch (...) {
    return RobotModelInitError::kInvalidYamlSchema;
  }
  const auto ee_id = FindFrameId(model, ee_name);
  if (!ee_id.has_value()) {
    return RobotModelInitError::kMissingEndEffectorFrame;
  }

  std::vector<ContactFrameInfo> contacts;
  const YAML::Node cf_node = cfg["contact_frames"];
  if (cf_node.IsDefined()) {
    if (!cf_node.IsSequence()) {
      return RobotModelInitError::kInvalidYamlSchema;
    }
    contacts.reserve(cf_node.size());
    for (const auto &entry : cf_node) {
      if (!entry.IsMap() || !entry["name"].IsDefined()) {
        return RobotModelInitError::kInvalidYamlSchema;
      }
      std::string cname;
      int cdim = 3;
      try {
        cname = entry["name"].as<std::string>();
        if (entry["dim"].IsDefined()) {
          cdim = entry["dim"].as<int>();
        }
      } catch (...) {
        return RobotModelInitError::kInvalidYamlSchema;
      }
      if (cdim != 3 && cdim != 6) {
        return RobotModelInitError::kInvalidContactDim;
      }
      const auto fid = FindFrameId(model, cname);
      if (!fid.has_value()) {
        return RobotModelInitError::kMissingContactFrame;
      }
      contacts.push_back(ContactFrameInfo{*fid, cdim, std::move(cname)});
    }
  }

  // All validation passed — commit.
  model_ = &model;
  ee_frame_id_ = *ee_id;
  contact_frames_ = std::move(contacts);
  return RobotModelInitError::kNoError;
}

int RobotModelHandler::nq() const noexcept {
  return model_ != nullptr ? static_cast<int>(model_->nq) : 0;
}
int RobotModelHandler::nv() const noexcept {
  return model_ != nullptr ? static_cast<int>(model_->nv) : 0;
}
int RobotModelHandler::nu() const noexcept {
  // Fixed-base manipulators only (Phase 1 scope; see header note).
  return nv();
}

std::optional<int>
RobotModelHandler::FrameId(std::string_view name) const noexcept {
  if (model_ == nullptr) {
    return std::nullopt;
  }
  return FindFrameId(*model_, std::string{name});
}

} // namespace rtc::mpc
