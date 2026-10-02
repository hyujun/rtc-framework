#include "integrated_bringup/support/hand_fk_wiring.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace integrated_bringup {

namespace rub = rtc_urdf_bridge;

std::string InstallHandJointOrder(rub::RtModelHandle* hand_handle,
                                  const rtc::DeviceNameConfig* hand_device,
                                  bool closed_chain_fk_active) {
  if (hand_handle == nullptr || hand_device == nullptr || closed_chain_fk_active ||
      hand_device->joint_state_names.empty()) {
    return {};
  }
  const pinocchio::Model& model = hand_handle->GetModel();
  const std::string who = "secondary device '" + hand_device->device_name + "': ";

  // Every position of the hand model has to be fed by exactly one device slot.
  // RtModelHandle::SetJointOrder only asks that each name exists, so a list
  // that names a strict subset of the model's joints — or one joint twice —
  // installs a map and leaves the rest of q unwritten.
  std::vector<int> slots_per_q(static_cast<std::size_t>(model.nq), 0);
  for (const auto& name : hand_device->joint_state_names) {
    if (!model.existJointName(name)) {
      return who + "joint '" + name + "' is not on the hand model";
    }
    const auto jid = model.getJointId(name);
    for (int k = 0; k < model.nqs[jid]; ++k) {
      ++slots_per_q[static_cast<std::size_t>(model.idx_qs[jid]) + static_cast<std::size_t>(k)];
    }
  }
  for (pinocchio::JointIndex jid = 1; jid < static_cast<pinocchio::JointIndex>(model.njoints);
       ++jid) {
    for (int k = 0; k < model.nqs[jid]; ++k) {
      const int slots =
          slots_per_q[static_cast<std::size_t>(model.idx_qs[jid]) + static_cast<std::size_t>(k)];
      if (slots == 0) {
        return who + "hand model joint '" + model.names[jid] + "' has no slot in joint_state_names";
      }
      if (slots > 1) {
        return who + "joint '" + model.names[jid] + "' is listed more than once";
      }
    }
  }

  if (!hand_handle->SetJointOrder(std::span<const std::string>(hand_device->joint_state_names))) {
    // Unreachable after the checks above; kept so a change in what
    // SetJointOrder accepts cannot turn into a silently missing map.
    return who + "joint_state_names could not be mapped onto the hand model";
  }
  return {};
}

HandFkWiring WireHandFk(const HandFkWiringRequest& request) {
  HandFkWiring wiring;
  if (request.hand_handle == nullptr) {
    return wiring;
  }
  wiring.joint_order_error = InstallHandJointOrder(request.hand_handle, request.hand_device,
                                                   request.closed_chain_fk_active);

  if (request.arm_tip_link.empty()) {
    return wiring;
  }
  const std::string tip(request.arm_tip_link);
  const std::string root(request.hand_root_link);
  if (request.model == nullptr) {
    wiring.mount_error = "no full model to resolve the hand mount on";
    return wiring;
  }
  const pinocchio::Model& model = *request.model;
  if (root.empty()) {
    wiring.mount_error =
        "the hand tree model declares no root_link to mount on arm tip '" + tip + "'";
    return wiring;
  }
  if (!model.existFrame(root)) {
    wiring.mount_error = "hand root link '" + root + "' is not on the model";
    return wiring;
  }
  if (!model.existFrame(tip)) {
    wiring.mount_error = "arm tip link '" + tip + "' is not on the model";
    return wiring;
  }
  const auto tip_id = model.getFrameId(tip);
  const auto root_id = model.getFrameId(root);
  if (tip_id == root_id) {
    // The hand is built on the arm tip itself: the exact identity, not the
    // rounding of R^T R.
    return wiring;
  }
  const auto& tip_frame = model.frames[tip_id];
  const auto& root_frame = model.frames[root_id];
  // Two frames on the same joint are rigidly attached, and their relative
  // placement is then a model constant — no FK, no configuration.
  if (tip_frame.parentJoint != root_frame.parentJoint) {
    wiring.mount_error = "hand root link '" + root + "' is not rigidly attached to arm tip link '" +
                         tip +
                         "' (a joint moves between them) — fingertip poses are composed through "
                         "a constant transform between the two";
    return wiring;
  }
  wiring.T_tip_mount = tip_frame.placement.actInv(root_frame.placement);
  return wiring;
}

}  // namespace integrated_bringup
