#ifndef INTEGRATED_BRINGUP_SUPPORT_ARM_TIP_RESOLUTION_HPP_
#define INTEGRATED_BRINGUP_SUPPORT_ARM_TIP_RESOLUTION_HPP_

// The configure-time verdict on a controller's arm tip frame.
//
// A demo controller with an arm model works from the arm tip's pose — the TF
// source, the task-space log columns, the frame the hand FK is composed onto,
// the seed a Cartesian hold starts from. The frame is looked up once, in
// OnDeviceConfigsSet, from the link name the controller manager resolved for
// the primary device group. When that lookup finds nothing the frame id stays
// 0, pinocchio's universe frame, and what follows depends on the lane:
//
//   - joint / task / compliance, normal tick: the arm tip is never registered
//     on the combined-model cache, whose read then returns the identity pose —
//     published as the arm tip, flagged valid;
//   - joint / task / compliance, E-STOP tick: the arm handle is read at frame
//     0, the universe placement (or its inverse-root), flagged valid the same;
//   - wbc: no arm tip pose is published at all, and the Cartesian hold seed is
//     read through frame 0.
//
// Nothing on the tick can tell. So a controller that has an arm model and no
// arm tip refuses the configure. A controller with no arm model at all (a
// fixture that drives the hooks with no URDF) has no tip to resolve and passes.
//
// What "resolved" means here is only that the name is a frame of the model. A
// reduced model keeps every link of the robot as a frame, so a link that is on
// the robot but not at the end of this arm resolves too; naming the wrong link
// is not something this verdict sees.

#include "rtc_base/types/types.hpp"

#include <string>
#include <string_view>

namespace integrated_bringup {

/// @brief Why the arm tip is not usable, or an empty string when it is.
///
/// Judged by on_configure from the controller's state at that moment, not
/// latched earlier: the frame id is written by OnDeviceConfigsSet and has to
/// survive a config that is loaded again after it.
///
/// @param has_arm_model   the controller built an arm model handle
/// @param tip_resolved    its arm tip frame id is a real frame (non-zero)
/// @param primary_device  the primary device group's name
/// @param primary_config  that group's device config — where the tip link the
///                        controller manager resolved arrives (may be null)
[[nodiscard]] inline std::string ArmTipUnresolvedReason(
    bool has_arm_model, bool tip_resolved, std::string_view primary_device,
    const rtc::DeviceNameConfig* primary_config) {
  if (!has_arm_model || tip_resolved) {
    return {};
  }
  const std::string device(primary_device);
  const std::string how_to_fix = " (a chain group takes it from urdf.sub_models." + device +
                                 ".tip_link; any group can set devices." + device +
                                 ".urdf.tip_link)";
  const std::string tip_link =
      (primary_config != nullptr && primary_config->urdf) ? primary_config->urdf->tip_link : "";
  if (tip_link.empty()) {
    return "primary device '" + device +
           "' has an arm model but no arm tip link: its device config carries none" + how_to_fix;
  }
  return "primary device '" + device + "': arm tip link '" + tip_link +
         "' is not a frame of the model" + how_to_fix;
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_SUPPORT_ARM_TIP_RESOLUTION_HPP_
