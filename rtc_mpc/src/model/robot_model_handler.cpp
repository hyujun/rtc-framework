#include "rtc_mpc/model/robot_model_handler.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fwd.hpp>
#pragma GCC diagnostic pop

namespace rtc::mpc {

namespace {

// Find a frame by name without throwing. Pinocchio's `getFrameId` returns
// `model.frames.size()` when the name is absent; wrap that into std::optional.
std::optional<int> FindFrameId(const pinocchio::Model& m, const std::string& name) noexcept {
  const auto id = m.getFrameId(name);
  if (id >= static_cast<pinocchio::FrameIndex>(m.frames.size())) {
    return std::nullopt;
  }
  return static_cast<int>(id);
}

}  // namespace

RobotModelInitError RobotModelHandler::Init(const pinocchio::Model& model,
                                            const YAML::Node& cfg) noexcept {
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

  // Base frame (필수). 누락 시 kInvalidYamlSchema로 거부 — silent universe
  // fallback은 F-4에서 제거되었다. 명시적 "universe"(frame_id 0)는 허용되며
  // fast-path 분기를 그대로 활용한다.
  const YAML::Node base_node = cfg["base_frame"];
  if (!base_node.IsDefined() || !base_node.IsScalar()) {
    return RobotModelInitError::kInvalidYamlSchema;
  }
  std::string base_name;
  try {
    base_name = base_node.as<std::string>();
  } catch (...) {
    return RobotModelInitError::kInvalidYamlSchema;
  }
  const auto resolved = FindFrameId(model, base_name);
  if (!resolved.has_value()) {
    return RobotModelInitError::kMissingBaseFrame;
  }
  const int base_id = *resolved;
  const bool base_is_universe = (base_id == 0);

  std::vector<ContactFrameInfo> contacts;
  const YAML::Node cf_node = cfg["contact_frames"];
  if (cf_node.IsDefined()) {
    if (!cf_node.IsSequence()) {
      return RobotModelInitError::kInvalidYamlSchema;
    }
    contacts.reserve(cf_node.size());
    for (const auto& entry : cf_node) {
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

  // base_frame placement at neutral configuration. base_frame은 fixed
  // (q-independent)이라는 invariant 하에 1회 계산해 저장; cost_factory가
  // ee_target(base 기준) → world 변환에 사용한다.
  pinocchio::SE3 base_oMf = pinocchio::SE3::Identity();
  if (!base_is_universe) {
    try {
      pinocchio::Data data(model);
      const Eigen::VectorXd q0 = pinocchio::neutral(model);
      pinocchio::forwardKinematics(model, data, q0);
      pinocchio::updateFramePlacement(model, data, static_cast<pinocchio::FrameIndex>(base_id));
      base_oMf = data.oMf[static_cast<std::size_t>(base_id)];
    } catch (...) {
      return RobotModelInitError::kInvalidYamlSchema;
    }
  }

  // All validation passed — commit.
  model_ = &model;
  ee_frame_id_ = *ee_id;
  base_frame_id_ = base_id;
  base_is_universe_ = base_is_universe;
  base_oMf_ = base_oMf;
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

std::optional<int> RobotModelHandler::FrameId(std::string_view name) const noexcept {
  if (model_ == nullptr) {
    return std::nullopt;
  }
  return FindFrameId(*model_, std::string{name});
}

}  // namespace rtc::mpc
