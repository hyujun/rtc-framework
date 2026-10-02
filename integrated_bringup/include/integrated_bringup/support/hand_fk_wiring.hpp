#ifndef INTEGRATED_BRINGUP_SUPPORT_HAND_FK_WIRING_HPP_
#define INTEGRATED_BRINGUP_SUPPORT_HAND_FK_WIRING_HPP_

// How a controller's hand fingertip FK is tied to the two things that live
// outside the hand model: the device group that feeds it, and the arm tip its
// result is composed onto.
//
//   T_root_fingertip = T_root_armtip · T_tip_mount · T_handroot_fingertip
//
// The hand FK is computed on the secondary group's tree model and comes back
// expressed in that tree's root link. Two facts decide whether the product
// above is the fingertip's pose:
//
//   - the hand model has to be fed the device's positions under the device's
//     joint order. A device may list its joints in any order (thumb first,
//     say) while the model keeps the URDF's;
//   - the hand's root link is not, in general, the arm's tip link. The two are
//     rigidly attached, so the transform between them is a model constant —
//     but it is the identity only on a robot whose arm chain happens to end on
//     the link the hand is built on.
//
// Both are resolved once at configure time, here, for every demo controller
// that publishes fingertip poses. Neither can be detected on the tick: a hand
// read in the wrong joint order, or composed on the wrong link, yields a pose
// that is finite, smooth and wrong by centimetres — so a wiring that cannot be
// resolved refuses the configure instead.

#include "rtc_base/types/types.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <string>
#include <string_view>

namespace integrated_bringup {

/// @brief What a controller keeps of its hand FK wiring.
struct HandFkWiring {
  /// Arm tip link → hand tree root link. Identity until resolved, which is also
  /// what a hand built directly on the arm tip resolves to.
  pinocchio::SE3 T_tip_mount{pinocchio::SE3::Identity()};
  /// Why the serial hand handle could not be given the device's joint order
  /// (empty: it was, or there was nothing to give it).
  std::string joint_order_error;
  /// Why @ref T_tip_mount could not be resolved (empty: it was, or there is no
  /// hand / no arm tip to mount it on).
  std::string mount_error;

  /// @return the reason on_configure must refuse, or an empty string.
  [[nodiscard]] const std::string& Error() const noexcept {
    return joint_order_error.empty() ? mount_error : joint_order_error;
  }
};

/// @brief Give the serial hand handle the device's joint order (non-RT).
///
/// Has to run wherever the handle is (re)built AND wherever the device configs
/// arrive: a controller builds its hand handle in LoadConfig, which the
/// controller manager runs before the device configs exist, and which a caller
/// that skips PreConfigure runs a second time after them. One call site covers
/// one of those orders only.
///
/// Nothing to do — and nothing to refuse — when there is no hand handle, no
/// device config or no joint names yet, or when the closed-chain FK is active
/// (it reads the device through its own name bridge and never consults the
/// serial handle; such a hand's device joints are not all on the serial tree).
///
/// @return empty on success or no-op. Otherwise the reason, and the handle is
///   left as it was: a name the hand model does not carry, or a list that does
///   not cover every position of the hand model exactly once (a strict subset
///   would map, and leave the other joints at whatever the buffer holds).
[[nodiscard]] std::string InstallHandJointOrder(rtc_urdf_bridge::RtModelHandle* hand_handle,
                                                const rtc::DeviceNameConfig* hand_device,
                                                bool closed_chain_fk_active);

/// @brief Inputs of @ref WireHandFk — what OnDeviceConfigsSet knows.
struct HandFkWiringRequest {
  rtc_urdf_bridge::RtModelHandle* hand_handle{nullptr};  ///< null: no hand model
  const rtc::DeviceNameConfig* hand_device{nullptr};     ///< null: no device config
  bool closed_chain_fk_active{false};
  /// The full model — the one model that carries both links below with every
  /// joint between them still a joint.
  const pinocchio::Model* model{nullptr};
  /// The link the arm tip pose is reported at. Empty when it did not resolve:
  /// there is then no arm tip to mount on, and the mount is left alone.
  std::string_view arm_tip_link;
  /// The secondary tree model's root link.
  std::string_view hand_root_link;
};

/// @brief Resolve the whole wiring at OnDeviceConfigsSet (non-RT): the joint
///   order (@ref InstallHandJointOrder) and the mount.
///
/// The mount is the placement of the hand root in the arm tip. It is a constant
/// only if both links hang off the same joint, which is checked on the full
/// model — on a reduced arm model every hand joint is locked, and any hand link
/// would pass. A hand whose root is not on the model, or is separated from the
/// arm tip by a joint, has no constant mount and is refused.
///
/// A config with no hand model has nothing to wire and comes back clean.
[[nodiscard]] HandFkWiring WireHandFk(const HandFkWiringRequest& request);

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_SUPPORT_HAND_FK_WIRING_HPP_
