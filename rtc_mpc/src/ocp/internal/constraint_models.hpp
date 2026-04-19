#pragma once

/// @file constraint_models.hpp
/// @brief Internal helper: build a `pinocchio::RigidConstraintModelVector`
///        from a `RobotModelHandler` and a list of active contact frame
///        ids. Shared by `LightContactOCP` and `ContactRichOCP`.
///
/// Not installed. Not part of the public `rtc_mpc` API. Lives under
/// `src/ocp/internal/` so TUs inside the library can `#include` it via
/// the build-tree include path (`${CMAKE_CURRENT_SOURCE_DIR}/src`).
///
/// The helper is `inline` so each translation unit gets its own copy;
/// this avoids a new `.cpp` + symbol export for a small pure function.

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/types/contact_plan_types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/contact-info.hpp>
#include <pinocchio/multibody/fwd.hpp>
#pragma GCC diagnostic pop

#include <cstddef>
#include <string>
#include <vector>

namespace rtc::mpc::internal {

using RigidConstraintModel = pinocchio::RigidConstraintModel;
using RigidConstraintVec = PINOCCHIO_ALIGNED_STD_VECTOR(RigidConstraintModel);

/// @brief Build the rigid-contact models vector for a given active-contact
///        set. Each entry references a frame from the Pinocchio model;
///        placement is taken from the frame itself. Contact type is
///        CONTACT_3D (point) or CONTACT_6D (wrench) per
///        `ContactFrameInfo::dim`. The `.name` field is set from the
///        matching `ContactFrameInfo::name` so downstream residuals (e.g.
///        `ContactForceResidualTpl`, `MultibodyFrictionConeResidualTpl`)
///        can locate the contact by string.
///
/// Precondition: every id in `active_frame_ids` must also appear in
/// `model.contact_frames()`. Callers enforce this before calling
/// (`kContactPlanModelMismatch` is surfaced at the OCP level).
inline RigidConstraintVec
BuildConstraintModels(const RobotModelHandler &model,
                      const std::vector<int> &active_frame_ids) noexcept {
  RigidConstraintVec out{};
  const auto &pin_model = model.model();
  const auto &frames = model.contact_frames();

  for (int fid : active_frame_ids) {
    int dim = 3;
    std::string name = "contact_" + std::to_string(fid);
    for (const auto &info : frames) {
      if (info.frame_id == fid) {
        dim = info.dim;
        name = info.name;
        break;
      }
    }
    const auto joint_id = static_cast<pinocchio::JointIndex>(
        pin_model.frames[static_cast<std::size_t>(fid)].parentJoint);
    const auto &placement =
        pin_model.frames[static_cast<std::size_t>(fid)].placement;
    const auto contact_type =
        (dim == 6) ? pinocchio::CONTACT_6D : pinocchio::CONTACT_3D;
    out.emplace_back(contact_type, pin_model, joint_id, placement,
                     pinocchio::LOCAL);
    out.back().name = name;
  }
  return out;
}

} // namespace rtc::mpc::internal
