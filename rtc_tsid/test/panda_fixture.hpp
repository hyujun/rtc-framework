// The Panda model the rtc_tsid suites run on (RTC_PANDA_URDF_PATH, set per
// test target in CMakeLists.txt), and the two fixture prefixes most of them
// share. A suite that needs contacts or extra state derives and extends
// SetUp(); one that mutates or copies the model calls LoadPandaModel() for its
// own, since PandaTest's model is parsed once and shared read-only.

#pragma once

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/types/wbc_types.hpp"

#include <memory>
#include <utility>

namespace rtc::tsid::test {

/// A fresh Panda model, parsed from the URDF on every call.
inline std::shared_ptr<pinocchio::Model> LoadPandaModel() {
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(RTC_PANDA_URDF_PATH, *model);
  return model;
}

/// One Panda model per process, read-only — what PandaTest cases share.
inline std::shared_ptr<const pinocchio::Model> SharedPandaModel() {
  static const std::shared_ptr<const pinocchio::Model> model = LoadPandaModel();
  return model;
}

/// Name of the frame SharedPandaWithRotatedBase() adds.
inline constexpr const char* kRotatedBaseFrame = "rotated_base";

/// The rotation of kRotatedBaseFrame in the world: about two axes, so R, Rᵀ
/// and I are three different matrices and a suite that ignores the base's
/// rotation, or applies it the wrong way round, gets a different answer.
inline Eigen::Matrix3d RotatedBaseRotation() {
  return (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

/// The Panda model plus kRotatedBaseFrame: a frame fixed to the universe at a
/// non-zero offset, turned by RotatedBaseRotation(). One per process,
/// read-only.
inline std::shared_ptr<const pinocchio::Model> SharedPandaWithRotatedBase() {
  static const std::shared_ptr<const pinocchio::Model> model = [] {
    auto m = LoadPandaModel();
    m->addFrame(
        pinocchio::Frame(kRotatedBaseFrame, 0, 0,
                         pinocchio::SE3(RotatedBaseRotation(), Eigen::Vector3d(0.12, -0.08, 0.05)),
                         pinocchio::OP_FRAME));
    return std::shared_ptr<const pinocchio::Model>(std::move(m));
  }();
  return model;
}

/// The Panda model and its RobotModelInfo built from an empty config.
class PandaTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = SharedPandaModel();
    robot_info_.Build(*model_, YAML::Node{});
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
};

/// PandaTest plus a contact-free cache, contact state and reference.
class PandaNoContactTest : public PandaTest {
 protected:
  void SetUp() override {
    PandaTest::SetUp();
    cache_.Init(model_, ContactFrameIds(contact_cfg_));
    contacts_.Init(0);
    ref_.Init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated, 0);
  }

  ContactManagerConfig contact_cfg_;  // no contacts
  PinocchioCache cache_;
  ContactState contacts_;
  ControlReference ref_;
};

}  // namespace rtc::tsid::test
