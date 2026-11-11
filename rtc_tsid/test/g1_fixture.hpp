// Free-flyer fixtures for the rtc_tsid suites.
//
// SharedG1Model(): the public 29-DoF floating G1 URDF from robot_descriptions,
// looked up through ament at run time (robot_descriptions is a <test_depend>
// only) with a pinocchio free-flyer root — nq 30, nv 29, 23 actuated (the
// wrist joints are fixed in this URDF). Its ankle-roll links carry the foot
// contacts; their origin is the ankle, not the sole.
//
// CompositeRootPandaModel(): the Panda with a translation + spherical
// composite root — nq 7 / nv 6 like a free-flyer, but not one. A rule that
// reads the root's dimensions instead of its type takes it for floating.
#pragma once

#include "panda_fixture.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/joint/joint-spherical.hpp>
#include <pinocchio/multibody/joint/joint-translation.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <memory>
#include <string>

namespace rtc::tsid::test {

inline constexpr int kG1Nq = 30;
inline constexpr int kG1Nv = 29;
inline constexpr int kG1Actuated = 23;
inline constexpr const char* kG1LeftFoot = "left_ankle_roll_link";
inline constexpr const char* kG1RightFoot = "right_ankle_roll_link";

inline std::string G1UrdfPath() {
  return ament_index_cpp::get_package_share_directory("robot_descriptions") +
         "/robots/g1/urdf/g1_29dof_floating.urdf";
}

/// One G1 free-flyer model per process, read-only.
inline std::shared_ptr<const pinocchio::Model> SharedG1Model() {
  static const std::shared_ptr<const pinocchio::Model> model = [] {
    auto m = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(G1UrdfPath(), pinocchio::JointModelFreeFlyer(), *m);
    return std::shared_ptr<const pinocchio::Model>(std::move(m));
  }();
  return model;
}

/// The Panda on a composite (translation + spherical) root, nq 7 / nv 6.
inline std::shared_ptr<const pinocchio::Model> CompositeRootPandaModel() {
  static const std::shared_ptr<const pinocchio::Model> model = [] {
    pinocchio::JointModelComposite root;
    root.addJoint(pinocchio::JointModelTranslation());
    root.addJoint(pinocchio::JointModelSpherical());
    auto m = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(RTC_PANDA_URDF_PATH, root, *m);
    return std::shared_ptr<const pinocchio::Model>(std::move(m));
  }();
  return model;
}

/// A foot (surface) contact on `frame`: 0.12 × 0.05 half-lengths (lx ≠ ly).
inline ContactConfig G1FootContact(const pinocchio::Model& model, const char* frame,
                                   int faces = 4) {
  ContactConfig c;
  c.name = frame;
  c.frame_name = frame;
  c.frame_id = static_cast<int>(model.getFrameId(frame));
  c.contact_dim = 6;
  c.friction_coeff = 0.5;
  c.friction_faces = faces;
  c.patch_half_length_x = 0.12;
  c.patch_half_length_y = 0.05;
  c.torsional_friction_coeff = 0.5;
  return c;
}

/// Both feet as surface contacts, max_* filled in.
inline ContactManagerConfig G1FeetContacts(const pinocchio::Model& model) {
  ContactManagerConfig mgr;
  mgr.contacts.push_back(G1FootContact(model, kG1LeftFoot));
  mgr.contacts.push_back(G1FootContact(model, kG1RightFoot));
  mgr.max_contacts = 2;
  mgr.max_contact_vars = 12;
  return mgr;
}

}  // namespace rtc::tsid::test
