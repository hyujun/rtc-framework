#include "integrated_bringup/support/task_target_frames.hpp"

#include <rtc_math/se3/so3.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

#include <cmath>

namespace integrated_bringup {

namespace {

[[nodiscard]] bool AllFinite(std::span<const double> v) noexcept {
  for (const double x : v) {
    if (!std::isfinite(x)) {
      return false;
    }
  }
  return true;
}

}  // namespace

std::string TaskTargetFrames::Configure(const pinocchio::Model& model,
                                        const std::string& root_link) {
  root_link_.clear();
  world_defined_ = false;
  root_T_world_.setIdentity();

  if (root_link.empty() || !model.existFrame(root_link)) {
    return "root_link '" + root_link + "' is not a frame of the model";
  }
  root_link_ = root_link;

  const std::string world(kSceneWorld);
  if (!model.existFrame(world)) {
    return {};  // absent: world goals are refused at ingress
  }
  const auto& root = model.frames[model.getFrameId(root_link)];
  const auto& sw = model.frames[model.getFrameId(world)];
  // Both placements are relative to their parent joint; on the same joint the
  // relative placement is constant in every configuration. On different
  // joints the world would move with the arm.
  if (root.parentJoint != sw.parentJoint) {
    return "scene_world is not rigidly attached to root_link '" + root_link +
           "' (they hang off different joints) — set urdf.extra_frames.scene_world.parent to it";
  }
  const pinocchio::SE3 placement = root.placement.actInv(sw.placement);
  if (!placement.translation().allFinite() || !placement.rotation().allFinite()) {
    return "scene_world's placement is not finite";
  }
  root_T_world_ = placement;
  world_defined_ = true;
  return {};
}

void TaskTargetFrames::ConfigureWithoutWorld(const std::string& root_link) {
  root_link_ = root_link;
  world_defined_ = false;
  root_T_world_.setIdentity();
}

const char* TaskTargetFrames::ToRoot(std::string_view frame_id, std::span<const double> task6,
                                     Layout layout, std::array<double, 6>& out) const noexcept {
  if (task6.size() != out.size()) {
    return "task goal must hold 6 values";
  }
  if (!Configured()) {
    return "the controller has no base frame to resolve a task goal against";
  }
  if (frame_id.empty()) {
    return "task goal has no header.frame_id (expected the arm's root_link or scene_world)";
  }
  if (frame_id == root_link_) {
    for (std::size_t i = 0; i < out.size(); ++i) {
      out[i] = task6[i];
    }
    return nullptr;
  }
  if (frame_id != kSceneWorld) {
    return "header.frame_id is neither the arm's root_link nor scene_world";
  }
  if (!world_defined_) {
    return "scene_world is not defined on this bring-up (no urdf.extra_frames.scene_world)";
  }

  const Eigen::Vector3d p_world(task6[0], task6[1], task6[2]);
  const Eigen::Vector3d p_root = root_T_world_.act(p_world);
  out[0] = p_root.x();
  out[1] = p_root.y();
  out[2] = p_root.z();
  if (layout == Layout::kPositionOnly) {
    // Entries 3..5 are not an orientation in this layout (C3, #779).
    out[3] = task6[3];
    out[4] = task6[4];
    out[5] = task6[5];
  } else {
    const Eigen::Matrix3d R_world =
        rtc::math::se3::RpyToRotationZyx(Eigen::Vector3d(task6[3], task6[4], task6[5]));
    const Eigen::Matrix3d R_root = root_T_world_.rotation() * R_world;
    // The inverse of RpyToRotationZyx (same Z-Y'-X'' convention); pinocchio's
    // is the one the bindings already use to report poses.
    const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(R_root);
    out[3] = rpy.x();
    out[4] = rpy.y();
    out[5] = rpy.z();
  }
  if (!AllFinite(out)) {
    return "task goal converted from scene_world is not finite";
  }
  return nullptr;
}

}  // namespace integrated_bringup
