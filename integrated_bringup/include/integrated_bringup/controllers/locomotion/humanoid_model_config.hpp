// ── `locomotion_controller.model.*` — which frames and device groups the humanoid
// controller works on (W1-F03, #1026) ───────────────────────────────────────────
//
//   locomotion_controller:
//     model:
//       body_group: "<device group>"   # the body's joints: tree model + joint_state_names
//       base_frame: "universe"         # a floating model accepts only universe (WD-19)
//       torso_frame: "<frame>"         # the torso tracking task's frame — not an end-effector
//       end_effectors:                 # the arm ends (hand mount when a hand is attached);
//         - {name: "<n>", frame: "<f>"}  #   order = index, {left, right} by convention
//       contacts:                      # order = rtc_mpc contact index
//         - {name: "<n>", frame: "<f>", dim: 6}  # dim: 3 (point) | 6 (surface)
//       hand_groups: ["<device group>"]  # held at the hold pose (WD-12); [] = no hand
//       imu:
//         sites: ["<label>"]           # the body group's IMU sites, in its backend's
//                                      #   imu_state.site_names order (1 to 2)
//         primary_site: "<label>"      # one of sites
//
// The three frame roles are separate on purpose (WD-35): contacts carry the load
// (feet now, a braced hand later), end_effectors are what manipulation moves, the
// torso frame is what the posture task tracks.
//
// Checks are split by what they need (AP-PROC-9):
//   Pass 1  LoadHumanoidModelConfig   the YAML alone — unknown / missing keys,
//                                     lengths, dim, empty or duplicate names / frames
//   Pass 1  ValidateAgainstModel      + the system model — frames exist, a floating
//                                     model's base is universe, no contact on the base
//   Pass 3  ValidateDevices           + the device rosters — groups exist, the body
//                                     roster IS the model's joint order (no identity
//                                     fallback for a missing roster, AP-CTRL-5)
// Every check throws std::invalid_argument naming the key path. Configure time only.
#pragma once

#include "rtc_base/types/types.hpp"

#include <pinocchio/multibody/fwd.hpp>
#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace integrated_bringup::locomotion {

/// Policy cap, not a capacity: a humanoid has two arm ends. A third end-effector
/// (a head camera, say) is a deliberate schema change here.
inline constexpr std::size_t kMaxEndEffectors = 2;

/// @brief A named frame role (an end-effector).
struct NamedFrame {
  std::string name;   ///< role name, unique across end_effectors and contacts
  std::string frame;  ///< model frame name
};

/// @brief One contact, in rtc_mpc contact-index order.
struct ContactSpec {
  std::string name;   ///< role name, unique across end_effectors and contacts
  std::string frame;  ///< model frame name
  int dim{6};         ///< 3 = point (force), 6 = surface (wrench)
};

/// @brief The `model.imu` section: which IMU sites the controller knows and which one
///        it propagates on.
///
/// Labels, not sensor names: the device backend maps each label to its own sensors
/// (`imu_state.site_names` lists the same labels in the same order, so index s here is
/// DeviceState IMU site s), and the controller never sees a simulator's names.
struct ImuSitesSpec {
  std::vector<std::string> sites;  ///< 1 to rtc::kMaxImuSites unique labels, site order
  std::string primary_site;        ///< one of `sites`
};

/// @brief The parsed `model` section.
struct HumanoidModelConfig {
  std::string body_group;
  std::string base_frame;
  std::string torso_frame;
  std::vector<NamedFrame> end_effectors;
  std::vector<ContactSpec> contacts;
  std::vector<std::string> hand_groups;
  ImuSitesSpec imu;
};

/// @brief Pass 1, schema only: parse the `model` section.
/// @param model    the `model:` node of the controller's section
/// @param context  the key path for messages, e.g. "locomotion_controller.model"
/// @throws std::invalid_argument on an unknown or missing key, a non-string name,
///         an empty or over-capacity list (end_effectors ≤ kMaxEndEffectors; contacts
///         ≤ rtc_mpc kMaxContacts and Σ dim ≤ kMaxContactVars), a dim not in {3, 6}, a
///         name / frame used twice (contacts and end_effectors share one name and one
///         frame space), a torso frame that is also one of those frames, a device
///         group named twice (hand_groups and body_group share one space), or an
///         `imu` whose sites are empty, over rtc::kMaxImuSites, or name a label twice,
///         or whose primary_site is not one of them.
[[nodiscard]] HumanoidModelConfig LoadHumanoidModelConfig(const YAML::Node& model,
                                                          const std::string& context);

/// @brief Pass 1, model-derived: check `cfg` against the system model.
/// @throws std::invalid_argument when a contact / end-effector / torso / base frame
///         is not in `model`, when `model` has a free-flyer root and the base frame
///         is not universe (WD-19), or when a contact frame is the base frame.
void ValidateAgainstModel(const HumanoidModelConfig& cfg, const pinocchio::Model& model,
                          const std::string& context);

/// @brief The model's joint names in configuration order, without the universe and
///        without a free-flyer root — the order a device roster must have.
[[nodiscard]] std::vector<std::string> JointOrder(const pinocchio::Model& model);

/// @brief The first place two joint orders differ.
struct JointOrderMismatch {
  std::size_t index{0};
  std::string expected;  ///< "<end>" when `expected` is the shorter list
  std::string actual;    ///< "<end>" when `actual` is the shorter list
};

/// @brief Compare two joint orders element by element.
/// @return nullopt when they are equal (same length, same names, same order).
[[nodiscard]] std::optional<JointOrderMismatch> CompareJointOrder(
    const std::vector<std::string>& expected, const std::vector<std::string>& actual);

/// @brief "index 3: expected 'a', actual 'b'" — for a message.
[[nodiscard]] std::string Describe(const JointOrderMismatch& mismatch);

/// Device roster lookup: the controller's GetDeviceNameConfig (nullptr = no such group).
using DeviceLookup = std::function<const rtc::DeviceNameConfig*(const std::string&)>;

/// @brief Pass 3, device-derived: check the groups `cfg` names against the devices.
/// @param body_joint_order  the body model's @ref JointOrder
/// @throws std::invalid_argument when the body group or a hand group has no device,
///         a hand group's roster (joint_state_names) is empty, or the body roster is
///         empty or not `body_joint_order` element by element — a missing roster is
///         refused, never taken as identity (AP-CTRL-5).
void ValidateDevices(const HumanoidModelConfig& cfg,
                     const std::vector<std::string>& body_joint_order, const DeviceLookup& lookup,
                     const std::string& context);

/// @brief The `RobotModelHandler::Init` YAML for `cfg` (rtc_mpc).
///
/// Every `end_effectors[i].frame` goes to the handler's `end_effector_frames` list in
/// order, so its `end_effector_frame_ids()[i]` is `cfg.end_effectors[i]` and its single
/// `end_effector_frame_id()` is the first. Contacts keep their order too, so contact
/// index i is `cfg.contacts[i]`.
/// @throws std::invalid_argument when `cfg.end_effectors` is empty (a config not
///         made by @ref LoadHumanoidModelConfig).
[[nodiscard]] YAML::Node ToRobotModelHandlerYaml(const HumanoidModelConfig& cfg);

}  // namespace integrated_bringup::locomotion
